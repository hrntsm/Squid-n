//! プッシュオーバー解析の司令塔。
//!
//! - [`pushover_analysis`] — 既存 API（節点変位を記録しない薄いラッパー）
//! - [`pushover_analysis_recording`] — 長期載荷・荷重制御・変位制御・弧長法の各
//!   フェーズを実行し、崩壊機構・部材別応答を集約する本体
//!
//! 確定ステップに残す記録（性能曲線・ヒンジ・せん断降伏・部材応答履歴・塑性率）は
//! [`super::step_record::StepRecorder`] が一手に引き受ける。ここはフェーズの制御
//! （増分の刻み・収束判定・終了目標の判定・フェーズ間の引き継ぎ）だけを持つ。

use super::diagnosis::{nonconvergence_detail, tangent_singular_diagnosis};
use super::mechanism::determine_mechanism;
use super::member_response::compute_member_response;
use super::response::{get_roof_dof, story_heights};
use super::step_record::StepRecorder;
use super::types::{
    DuctilityMethod, MemberHistory, PushoverControl, PushoverResult, PushoverTarget,
    PushoverTermination,
};
use crate::common::constraint::Reducer;
use crate::common::csc_cache::CscCache;
use crate::common::elem_loop::apply_du_trial;
use crate::common::newton::{l2_norm, STATIC_NEWTON};
use crate::common::tangent::{add_support_spring_f_int, assemble_k_cached, compute_f_int};
use crate::common::transaction::StateSnapshot;
use crate::nonlinear::arc_length::ArcLengthSolver;
use crate::statics::analysis::{
    building_height_mm, distribute_pi_over_diaphragms, steel_height_ratio, SeismicDir,
};
use squid_n_core::dof::DofMap;
use squid_n_core::model::Model;
use squid_n_element::behavior::ElementBehavior;
use squid_n_element::factory::{build_nonlinear_behavior, StrengthBasis};
use squid_n_math::solver::{make_solver, LinearSolver, SolverBackend};

/// 全フェーズで持ち回るソルバインスタンス・CSC 組立てキャッシュ・作業バッファ。
/// `solver` は `DirectSparseCholesky` を明示する。
///
/// 各バッファ・キャッシュはフェーズをまたいで使い回し、呼び出しのたびに上書きされる。
struct SolverState {
    solver: Box<dyn LinearSolver>,
    k_free_cache: CscCache,
    k_red_cache: CscCache,
    r_red: Vec<f64>,
    f_ext_red: Vec<f64>,
    du_red: Vec<f64>,
    du_free: Vec<f64>,
    q_red: Vec<f64>,
    du_r_red: Vec<f64>,
    du_q_red: Vec<f64>,
    du_r: Vec<f64>,
    du_q: Vec<f64>,
}

impl SolverState {
    fn new(n_active: usize, n_indep: usize) -> Self {
        Self {
            solver: make_solver(SolverBackend::DirectSparseCholesky),
            k_free_cache: CscCache::new(),
            k_red_cache: CscCache::new(),
            r_red: vec![0.0; n_indep],
            f_ext_red: vec![0.0; n_indep],
            du_red: vec![0.0; n_indep],
            du_free: vec![0.0; n_active],
            q_red: vec![0.0; n_indep],
            du_r_red: vec![0.0; n_indep],
            du_q_red: vec![0.0; n_indep],
            du_r: vec![0.0; n_active],
            du_q: vec![0.0; n_active],
        }
    }
}

/// 増分解析（プッシュオーバー解析）。
/// `max_disp` は目標変位 [mm] のみの終了判定に変換して本体へ渡すラッパー。
#[allow(clippy::too_many_arguments)]
pub fn pushover_analysis(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    dir: SeismicDir,
    max_steps: usize,
    max_disp: f64,
    use_kg: bool,
    use_arc_length: bool,
    arc_length_dl: f64,
) -> Result<PushoverResult, String> {
    pushover_analysis_recording(
        model,
        dofmap,
        reducer,
        dir,
        max_steps,
        PushoverTarget::from_max_disp(max_disp),
        PushoverControl::default(),
        true,
        use_kg,
        use_arc_length,
        arc_length_dl,
        DuctilityMethod::default(),
    )
}

/// 増分解析（プッシュオーバー解析）。終了目標は [`PushoverTarget`] で
/// 指定する。`apply_long_term` が真の場合、長期系荷重を水平力増分の前に載荷する。
#[allow(clippy::too_many_arguments)]
pub fn pushover_analysis_recording(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    dir: SeismicDir,
    max_steps: usize,
    target: PushoverTarget,
    control: PushoverControl,
    apply_long_term: bool,
    use_kg: bool,
    use_arc_length: bool,
    arc_length_dl: f64,
    ductility_method: DuctilityMethod,
) -> Result<PushoverResult, String> {
    let n_active = dofmap.n_active();
    if n_active == 0 {
        return Err("no active DOF".into());
    }

    let mut st = SolverState::new(n_active, reducer.n_indep);

    squid_n_element::factory::ensure_nonlinear_input(model)?;

    let mut behaviors: Vec<Box<dyn ElementBehavior>> = Vec::new();
    for elem in &model.elements {
        let b = build_nonlinear_behavior(
            elem,
            model,
            StrengthBasis::MaterialStrength,
            squid_n_element::factory::AnalysisKind::Incremental,
        );
        behaviors.push(b);
    }

    let layers = model.layers();
    if layers.is_empty() {
        return Err("no stories defined".into());
    }
    let height_m = building_height_mm(model) / 1000.0;
    let steel_ratio = steel_height_ratio(model);
    let t = squid_n_load::ai::approx_t(height_m, steel_ratio);
    let z = 1.0;
    let tc = squid_n_load::ai::tc_of(squid_n_load::ai::SoilClass::II);
    let rt_val = squid_n_load::ai::rt(t, tc);
    let c0 = 0.2;
    let story_weights: Vec<f64> = layers.iter().map(|l| l.weight.unwrap_or(0.0)).collect();
    if story_weights.iter().all(|&w| w == 0.0) {
        return Err("no seismic weight defined".into());
    }
    let ai = squid_n_load::ai::ai_distribution(&story_weights, z, rt_val, c0, t);

    let dir_vec = match dir {
        SeismicDir::X => [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        SeismicDir::Y => [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    };
    let mut q = vec![0.0; n_active];
    for layer in &layers {
        let pi = ai.pi.get(layer.index).copied().unwrap_or(0.0);
        if pi == 0.0 {
            continue;
        }
        let Some(story) = model.stories.get(layer.top.index()) else {
            continue;
        };
        for (master, share) in distribute_pi_over_diaphragms(model, story, pi) {
            let ni = master.index();
            for d in 0..6 {
                let g = ni * 6 + d;
                if let Some(a) = dofmap.active(g) {
                    q[a as usize] += dir_vec[d] * share;
                }
            }
        }
    }

    if reducer.n_indep > 0 {
        let k_free = assemble_k_cached(model, dofmap, &behaviors, use_kg, &mut st.k_free_cache);
        let k_red = reducer.reduce_k_cached(&k_free, &mut st.k_red_cache);
        if st.solver.factorize(&k_red).is_err() {
            return Err(tangent_singular_diagnosis(
                model,
                dofmap,
                reducer,
                &k_red,
                "増分解析の初期接線剛性",
            ));
        }
    }

    let heights = story_heights(model);
    let mut recorder = StepRecorder::new(model, dir, &heights, ductility_method);
    let mut total_disp = vec![0.0; n_active];
    let n_steps = max_steps.clamp(1, 100);
    let dlambda = 1.0 / n_steps as f64;
    let mut target_reached = false;
    let mut lambda = 0.0;
    let mut phase_outcome = PushoverTermination::Unknown;

    let sum_h: f64 = heights.iter().sum();
    let mut roof_bound = f64::INFINITY;
    if let Some(d) = target.max_disp {
        roof_bound = roof_bound.min(d);
    }
    if let Some(a) = target.max_drift_angle {
        if sum_h > 0.0 {
            roof_bound = roof_bound.min(a * sum_h);
        }
    }

    let du_uniform = (matches!(control, PushoverControl::Phased) && roof_bound.is_finite())
        .then(|| roof_bound / n_steps as f64)
        .filter(|du| *du > 0.0);
    let mut roof_slope = du_uniform.and_then(|_| {
        elastic_roof_slope(model, dofmap, reducer, &behaviors, use_kg, dir, &q, &mut st)
    });
    let adaptive = du_uniform.is_some() && roof_slope.is_some();

    let lambda_cap = match control {
        PushoverControl::LoadOnly if target.is_enabled() => 10.0,
        _ => 1.0,
    };
    let max_load_steps = n_steps * 10;

    let mut last_roof = 0.0_f64;

    let f0: Vec<f64> = if apply_long_term {
        let mut f = vec![0.0; n_active];
        for lc in model.load_cases.iter().filter(|l| l.kind.is_long_term()) {
            let flc = crate::common::assemble::assemble_global_f(model, dofmap, lc.id);
            for (acc, v) in f.iter_mut().zip(flc) {
                *acc += v;
            }
        }
        f
    } else {
        vec![0.0; n_active]
    };
    if f0.iter().any(|v| v.abs() > 0.0) {
        let n_grav = 5usize;
        let mut applied = 0.0_f64;
        for gstep in 0..n_grav {
            let mut mu_target = (gstep + 1) as f64 / n_grav as f64;
            let mut step_ok = false;
            for _attempt in 0..5 {
                let snap = StateSnapshot::capture(&behaviors);
                let f_ext: Vec<f64> = f0.iter().map(|&v| v * mu_target).collect();
                match newton_converge(
                    model,
                    dofmap,
                    reducer,
                    &mut behaviors,
                    &f_ext,
                    use_kg,
                    n_active,
                    &total_disp,
                    &mut st,
                ) {
                    Some(step_du_free) => {
                        for b in behaviors.iter_mut() {
                            b.commit_state();
                        }
                        for (&du, td) in step_du_free.iter().zip(total_disp.iter_mut()) {
                            *td += du;
                        }
                        applied = mu_target;
                        step_ok = true;
                        break;
                    }
                    None => {
                        snap.restore(&mut behaviors);
                        mu_target = applied + (mu_target - applied) * 0.5;
                    }
                }
            }
            if !step_ok {
                let detail = current_failure_detail(
                    model,
                    dofmap,
                    reducer,
                    &behaviors,
                    use_kg,
                    &mut st,
                    "長期荷重の初期載荷における接線剛性",
                );
                return Err(format!(
                    "長期荷重の初期載荷が収束しません（長期荷重に対して構造が不安定な可能性）。{detail}"
                ));
            }
        }
        let rec = recorder.record(0.0, model, dofmap, &behaviors, &total_disp);
        last_roof = rec.roof;
    }

    for _step in 0..max_load_steps {
        if lambda >= lambda_cap - 1e-12 {
            phase_outcome = PushoverTermination::LambdaCap { lambda };
            break;
        }
        let prev_lambda = lambda;
        let mut current_lambda = if adaptive {
            let du = du_uniform.unwrap_or(0.0);
            let slope = roof_slope.unwrap_or(f64::INFINITY);
            let dl = (du / slope).max(dlambda * 1e-3);
            (lambda + dl).min(lambda_cap)
        } else {
            (lambda + dlambda).min(lambda_cap)
        };
        let mut step_ok = false;

        for _attempt in 0..5 {
            let snap = StateSnapshot::capture(&behaviors);
            let f_ext: Vec<f64> = f0
                .iter()
                .zip(q.iter())
                .map(|(&f0i, &qi)| f0i + qi * current_lambda)
                .collect();
            let converged = newton_converge(
                model,
                dofmap,
                reducer,
                &mut behaviors,
                &f_ext,
                use_kg,
                n_active,
                &total_disp,
                &mut st,
            );

            if let Some(step_du_free) = converged {
                for b in behaviors.iter_mut() {
                    b.commit_state();
                }
                for (&du, td) in step_du_free.iter().zip(total_disp.iter_mut()) {
                    *td += du;
                }
                let rec = recorder.record(current_lambda, model, dofmap, &behaviors, &total_disp);
                let roof = rec.roof;
                let drift_angle_now = rec.drift_angle;
                if adaptive {
                    let d_roof = (roof - last_roof).abs();
                    let d_lambda = current_lambda - prev_lambda;
                    if d_lambda > 1e-12 && d_roof > 1e-12 {
                        roof_slope = Some(d_roof / d_lambda);
                    }
                }
                last_roof = roof;
                lambda = current_lambda;
                step_ok = true;
                if target.reached(roof, drift_angle_now) {
                    target_reached = true;
                }
                break;
            } else {
                snap.restore(&mut behaviors);
                current_lambda = prev_lambda + (current_lambda - prev_lambda) * 0.5;
            }
        }
        if !step_ok {
            phase_outcome = PushoverTermination::NonConvergence {
                phase: "荷重制御".into(),
                load_factor: lambda,
            };
            break;
        }
        if target_reached {
            break;
        }
        phase_outcome = PushoverTermination::ScheduleCompleted;
    }

    let disp_control_roof =
        if matches!(control, PushoverControl::Phased) && target.is_enabled() && !target_reached {
            get_roof_dof(model, dofmap, dir)
        } else {
            None
        };
    if let Some(roof_active) = disp_control_roof {
        let initial_disp = total_disp[roof_active];
        if roof_bound.is_finite() && roof_bound > initial_disp {
            let n_disp_steps = match du_uniform {
                Some(du) if adaptive => (((roof_bound - initial_disp) / du).ceil() as usize)
                    .clamp(1, n_steps.saturating_mul(2)),
                _ => 10usize,
            };
            let du_target = (roof_bound - initial_disp) / n_disp_steps as f64;

            for step in 0..n_disp_steps {
                let roof_target = initial_disp + du_target * (step + 1) as f64;
                let mut step_ok = false;

                for attempt in 0..5 {
                    let committed_roof = total_disp[roof_active];
                    let sub_target =
                        committed_roof + (roof_target - committed_roof) * 0.5_f64.powi(attempt);
                    let snap = StateSnapshot::capture(&behaviors);
                    let lambda_snap = lambda;
                    let mut converged = false;
                    let mut step_du_free = vec![0.0; n_active];

                    for _iter in STATIC_NEWTON.iters() {
                        let k_free = assemble_k_cached(
                            model,
                            dofmap,
                            &behaviors,
                            use_kg,
                            &mut st.k_free_cache,
                        );
                        let k_red = reducer.reduce_k_cached(&k_free, &mut st.k_red_cache);
                        let mut f_int = compute_f_int(model, dofmap, &behaviors);
                        let u_trial: Vec<f64> = total_disp
                            .iter()
                            .zip(step_du_free.iter())
                            .map(|(&t, &s)| t + s)
                            .collect();
                        add_support_spring_f_int(model, dofmap, &u_trial, &mut f_int);

                        let f_ext: Vec<f64> = f0
                            .iter()
                            .zip(q.iter())
                            .map(|(&f0i, &qi)| f0i + qi * lambda)
                            .collect();
                        let r_free: Vec<f64> =
                            f_ext.iter().zip(f_int.iter()).map(|(e, i)| e - i).collect();
                        reducer.reduce_f_into(&r_free, &mut st.r_red);

                        let u_roof = total_disp[roof_active] + step_du_free[roof_active];
                        let gap = sub_target - u_roof;
                        let r_norm = l2_norm(&st.r_red);
                        reducer.reduce_f_into(&f_ext, &mut st.f_ext_red);
                        let f_scale = l2_norm(&st.f_ext_red).max(1.0);
                        if STATIC_NEWTON.converged(r_norm, f_scale)
                            && gap.abs() < (sub_target.abs() * STATIC_NEWTON.tol).max(1e-9)
                        {
                            converged = true;
                            break;
                        }

                        if st.solver.factorize(&k_red).is_err() {
                            break;
                        }
                        if st.solver.solve_into(&st.r_red, &mut st.du_r_red).is_err() {
                            break;
                        }
                        reducer.reduce_f_into(&q, &mut st.q_red);
                        if st.solver.solve_into(&st.q_red, &mut st.du_q_red).is_err() {
                            break;
                        }
                        reducer.expand_u_into(&st.du_r_red, &mut st.du_r);
                        reducer.expand_u_into(&st.du_q_red, &mut st.du_q);
                        let denom = st.du_q[roof_active];
                        if denom.abs() < 1e-30 {
                            break;
                        }
                        let dlambda_ctrl = (gap - st.du_r[roof_active]) / denom;
                        lambda += dlambda_ctrl;
                        for i in 0..n_active {
                            st.du_free[i] = st.du_r[i] + dlambda_ctrl * st.du_q[i];
                        }
                        for (acc, &d) in step_du_free.iter_mut().zip(st.du_free.iter()) {
                            *acc += d;
                        }
                        apply_du_trial(model, dofmap, &mut behaviors, &st.du_free);
                    }

                    if converged {
                        for b in behaviors.iter_mut() {
                            b.commit_state();
                        }
                        for (&du, td) in step_du_free.iter().zip(total_disp.iter_mut()) {
                            *td += du;
                        }
                        let rec = recorder.record(lambda, model, dofmap, &behaviors, &total_disp);
                        let roof = rec.roof;
                        let drift_angle_now = rec.drift_angle;
                        step_ok = true;
                        if target.reached(roof, drift_angle_now) {
                            target_reached = true;
                        }
                        break;
                    } else {
                        snap.restore(&mut behaviors);
                        lambda = lambda_snap;
                    }
                }
                if !step_ok {
                    phase_outcome = PushoverTermination::NonConvergence {
                        phase: "変位制御".into(),
                        load_factor: lambda,
                    };
                    break;
                }
                phase_outcome = PushoverTermination::ScheduleCompleted;
                if target_reached {
                    break;
                }
            }
        }
    }

    if use_arc_length && matches!(control, PushoverControl::Phased) && !target_reached {
        let arc_solver = ArcLengthSolver::new(arc_length_dl);
        let mut prev_du: Vec<f64> = Vec::new();
        let mut arc_lambda = if lambda > 0.0 { lambda } else { 1.0 };

        for _step in 0..20 {
            let snap = StateSnapshot::capture(&behaviors);
            let k_free = assemble_k_cached(model, dofmap, &behaviors, use_kg, &mut st.k_free_cache);
            let k_red = reducer.reduce_k_cached(&k_free, &mut st.k_red_cache);

            if st.solver.factorize(&k_red).is_err() {
                snap.restore(&mut behaviors);
                phase_outcome = PushoverTermination::TangentSingular {
                    phase: "弧長法".into(),
                    load_factor: arc_lambda,
                };
                break;
            }

            let mut cum_du = vec![0.0; n_active];
            let result = {
                let model_ref: &Model = model;
                let behaviors_ref = &mut behaviors;
                let total_disp_ref: &Vec<f64> = &total_disp;
                let st_ref = &mut st;
                arc_solver.step(
                    &q,
                    &mut |r: &[f64], out: &mut Vec<f64>| -> Result<(), String> {
                        reducer.reduce_f_into(r, &mut st_ref.r_red);
                        st_ref
                            .solver
                            .solve_into(&st_ref.r_red, &mut st_ref.du_red)
                            .map_err(|e| format!("{:?}", e))?;
                        reducer.expand_u_into(&st_ref.du_red, out);
                        Ok(())
                    },
                    &mut |delta_u: &[f64]| -> Result<Vec<f64>, String> {
                        apply_du_trial(model_ref, dofmap, behaviors_ref, delta_u);
                        for (acc, &d) in cum_du.iter_mut().zip(delta_u.iter()) {
                            *acc += d;
                        }
                        let mut f_int = compute_f_int(model_ref, dofmap, behaviors_ref);
                        let u_trial: Vec<f64> = total_disp_ref
                            .iter()
                            .zip(cum_du.iter())
                            .map(|(&t, &c)| t + c)
                            .collect();
                        add_support_spring_f_int(model_ref, dofmap, &u_trial, &mut f_int);
                        Ok(f_int
                            .iter()
                            .zip(f0.iter())
                            .map(|(&fi, &f0i)| fi - f0i)
                            .collect())
                    },
                    &prev_du,
                    arc_lambda,
                )
            };

            match result {
                Ok(step_result) if step_result.converged => {
                    for b in behaviors.iter_mut() {
                        b.commit_state();
                    }
                    for (&du, td) in step_result.du.iter().zip(total_disp.iter_mut()) {
                        *td += du;
                    }
                    arc_lambda += step_result.dlambda;
                    prev_du = step_result.du;

                    let rec = recorder.record(arc_lambda, model, dofmap, &behaviors, &total_disp);
                    if target.reached(rec.roof, rec.drift_angle) {
                        target_reached = true;
                        break;
                    }
                }
                _ => {
                    snap.restore(&mut behaviors);
                    phase_outcome = PushoverTermination::NonConvergence {
                        phase: "弧長法".into(),
                        load_factor: arc_lambda,
                    };
                    break;
                }
            }
            phase_outcome = PushoverTermination::ScheduleCompleted;
        }
    }

    if recorder.is_empty() {
        let detail = current_failure_detail(
            model,
            dofmap,
            reducer,
            &behaviors,
            use_kg,
            &mut st,
            "接線剛性",
        );
        return Err(format!(
            "水平力の増分が 1 ステップも収束しませんでした（性能曲線が得られません）。{detail}"
        ));
    }

    let mechanism = determine_mechanism(recorder.hinges(), model, dir);
    let qu = recorder.qu();
    let member_response = compute_member_response(model, dofmap, &behaviors, &total_disp, dir);
    let detail_elems: std::collections::HashSet<squid_n_core::ids::ElemId> = recorder
        .hinges()
        .iter()
        .map(|h| h.elem)
        .chain(recorder.shear_yields().iter().map(|s| s.elem))
        .collect();
    let recorded = recorder.finish();
    let member_history: Vec<MemberHistory> = model
        .elements
        .iter()
        .enumerate()
        .filter(|(_, e)| detail_elems.contains(&e.id))
        .map(|(i, e)| MemberHistory {
            elem: e.id,
            records: recorded
                .member_history_steps
                .iter()
                .filter_map(|s| s.get(i).copied())
                .collect(),
        })
        .collect();
    let fiber_states: Vec<(
        squid_n_core::ids::ElemId,
        Vec<squid_n_element::behavior::FiberSectionState>,
    )> = model
        .elements
        .iter()
        .zip(&behaviors)
        .filter(|(e, _)| detail_elems.contains(&e.id))
        .filter_map(|(e, b)| b.fiber_section_states().map(|s| (e.id, s)))
        .collect();
    let termination = if target_reached {
        PushoverTermination::TargetReached
    } else {
        phase_outcome
    };
    Ok(PushoverResult {
        steps: recorded.steps,
        capacity_curve: recorded.capacity_curve,
        hinges: recorded.hinges,
        shear_yields: recorded.shear_yields,
        mechanism,
        qu,
        member_response,
        control,
        member_history,
        fiber_states,
        termination,
    })
}

/// 現時点の要素状態における接線剛性を組み立て直し、非収束の原因を切り分けた
/// 診断メッセージを返す。
///
/// エラーを返す直前でのみ呼ぶこと。
fn current_failure_detail(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    behaviors: &[Box<dyn ElementBehavior>],
    use_kg: bool,
    st: &mut SolverState,
    phase: &str,
) -> String {
    let k_free = assemble_k_cached(model, dofmap, behaviors, use_kg, &mut st.k_free_cache);
    let k_red = reducer.reduce_k_cached(&k_free, &mut st.k_red_cache);
    let factorizable = st.solver.factorize(&k_red).is_ok();
    nonconvergence_detail(model, dofmap, reducer, &k_red, factorizable, phase)
}

/// 固定外力 `f_ext` に対する Newton 反復。
///
/// 収束判定は力の相対ノルム r < tol·max(|f_ext|, 1)（規約は [`STATIC_NEWTON`]）。
/// 収束したらステップ内の全 Newton 修正量の累積を `Some` で返し、
/// 収束しなければ `None`。接線剛性の分解・求解の失敗も `None` として返す。
#[allow(clippy::too_many_arguments)]
fn newton_converge(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    behaviors: &mut [Box<dyn ElementBehavior>],
    f_ext: &[f64],
    use_kg: bool,
    n_active: usize,
    total_disp_base: &[f64],
    st: &mut SolverState,
) -> Option<Vec<f64>> {
    let mut step_du_free = vec![0.0; n_active];
    for _iter in STATIC_NEWTON.iters() {
        let k_free = assemble_k_cached(model, dofmap, behaviors, use_kg, &mut st.k_free_cache);
        let k_red = reducer.reduce_k_cached(&k_free, &mut st.k_red_cache);
        let mut f_int = compute_f_int(model, dofmap, behaviors);
        let u_trial: Vec<f64> = total_disp_base
            .iter()
            .zip(step_du_free.iter())
            .map(|(&t, &s)| t + s)
            .collect();
        add_support_spring_f_int(model, dofmap, &u_trial, &mut f_int);
        let r_free: Vec<f64> = f_ext.iter().zip(f_int.iter()).map(|(e, i)| e - i).collect();
        reducer.reduce_f_into(&r_free, &mut st.r_red);
        reducer.reduce_f_into(f_ext, &mut st.f_ext_red);
        let r_norm = l2_norm(&st.r_red);
        let f_norm = l2_norm(&st.f_ext_red);
        if STATIC_NEWTON.converged(r_norm, f_norm.max(1.0)) {
            return Some(step_du_free);
        }
        if st.solver.factorize(&k_red).is_err() {
            return None;
        }
        if st.solver.solve_into(&st.r_red, &mut st.du_red).is_err() {
            return None;
        }
        reducer.expand_u_into(&st.du_red, &mut st.du_free);
        for (acc, &d) in step_du_free.iter_mut().zip(st.du_free.iter()) {
            *acc += d;
        }
        apply_du_trial(model, dofmap, behaviors, &st.du_free);
    }
    None
}

/// 初期接線剛性で K·δu = q を 1 回解き、荷重係数 λ あたりの頂部変位の弾性勾配
/// [mm/λ] を推定する。推定できない場合は `None` を返す。
#[allow(clippy::too_many_arguments)]
fn elastic_roof_slope(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    behaviors: &[Box<dyn ElementBehavior>],
    use_kg: bool,
    dir: SeismicDir,
    q: &[f64],
    st: &mut SolverState,
) -> Option<f64> {
    let roof_active = get_roof_dof(model, dofmap, dir)?;
    let k_free = assemble_k_cached(model, dofmap, behaviors, use_kg, &mut st.k_free_cache);
    let k_red = reducer.reduce_k_cached(&k_free, &mut st.k_red_cache);
    st.solver.factorize(&k_red).ok()?;
    reducer.reduce_f_into(q, &mut st.r_red);
    st.solver.solve_into(&st.r_red, &mut st.du_red).ok()?;
    reducer.expand_u_into(&st.du_red, &mut st.du_free);
    let slope = st.du_free.get(roof_active).copied().unwrap_or(0.0).abs();
    (slope > 1e-12).then_some(slope)
}
