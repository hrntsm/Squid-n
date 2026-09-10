//! 線形時刻歴応答解析（Newmark-β 法、基盤一様加振）。
//!
//! - [`linear_time_history_analysis`] — 線形時刻歴応答解析
//! - [`linear_time_history_with_state`] — 最終状態付き（チェックポイント保存用）
//! - [`linear_time_history_from_state`] — チェックポイントからの再開

use super::common::{
    accel_dir_at, accel_xy_at, empty_response, expand_peak_disp, mass_accel_free_into,
    reduced_vec_from, resolve_dt, solve_initial_accel, sparse_matvec_into, GroundInfluence,
    NewmarkCoeffs,
};
use super::config::{GroundMotion, NewmarkCfg};
use super::history::{
    choose_record_dir_y, pick_record_node, record_history_step, total_mass, update_story_drift,
};
use super::recording::{member_forces_linear, ThRecorder};
use super::result::{ResponseHistory, ResponseResult, TimeStepState};
use crate::common::assemble::{assemble_global_k, assemble_global_m};
use crate::common::constraint::Reducer;
use crate::dynamic::damping::Damping;
use squid_n_core::dof::DofMap;
use squid_n_core::model::Model;
use squid_n_element::behavior::{ElementBehavior, MassOption};
use squid_n_element::factory::build_behavior;
use squid_n_math::solver::{make_solver, SolveError, SolverBackend};
use squid_n_math::sparse::sparse_matvec;

/// 線形時刻歴応答解析（Newmark-β 法、基盤一様加振）。
///
/// `initial_disp`/`initial_vel` は縮約空間（n_indep 長）の初期値。
/// 自由振動（地震波なし）の場合は `wave.accel_x` をゼロ埋めして呼ぶ。
/// `newmark.dt == 0.0` のときは `wave.dt` を採用する。
/// `record_every` は詳細記録（[`super::ThRecording`]）の間引き係数。`None` は
/// 自動決定（[`super::recording::auto_record_every`]）。
#[allow(clippy::too_many_arguments)]
pub fn linear_time_history_analysis(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    wave: &GroundMotion,
    newmark: &NewmarkCfg,
    damping: &Damping,
    initial_disp: &[f64],
    initial_vel: &[f64],
    use_kg: bool,
    record_every: Option<usize>,
) -> Result<ResponseResult, SolveError> {
    let (result, _state) = linear_time_history_with_state(
        model,
        dofmap,
        reducer,
        wave,
        newmark,
        damping,
        initial_disp,
        initial_vel,
        use_kg,
        record_every,
    )?;
    Ok(result)
}

/// 線形時刻歴応答解析（最終状態付き）。チェックポイント保存用に最終状態を返す。
#[allow(clippy::too_many_arguments)]
pub fn linear_time_history_with_state(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    wave: &GroundMotion,
    newmark: &NewmarkCfg,
    damping: &Damping,
    initial_disp: &[f64],
    initial_vel: &[f64],
    use_kg: bool,
    record_every: Option<usize>,
) -> Result<(ResponseResult, TimeStepState), SolveError> {
    squid_n_math::parallelism::apply_to_faer();

    let dt = resolve_dt(newmark.dt, wave)?;

    let n_indep = reducer.n_indep;
    if n_indep == 0 {
        return Ok((
            empty_response(model, false, false),
            TimeStepState {
                step: 0,
                time: 0.0,
                disp_red: vec![],
                vel_red: vec![],
                accel_red: vec![],
            },
        ));
    }

    let mut setup = LinearSetup::build(model, dofmap, reducer, newmark, damping, dt, use_kg)?;

    let u = reduced_vec_from(n_indep, initial_disp);
    let v = reduced_vec_from(n_indep, initial_vel);

    let p_red_0 = reducer.reduce_f(&setup.infl.force_at(wave, 0));

    let cv0 = sparse_matvec(&setup.c_red, &v);
    let ku0 = sparse_matvec(&setup.k_red, &u);
    let mut rhs_a0 = vec![0.0; n_indep];
    for i in 0..n_indep {
        rhs_a0[i] = p_red_0[i] - cv0[i] - ku0[i];
    }
    let a = solve_initial_accel(&setup.m_red, &rhs_a0, n_indep)?;

    run_steps(
        model,
        dofmap,
        reducer,
        wave,
        dt,
        0,
        &mut setup,
        u,
        v,
        a,
        record_every,
    )
}

/// チェックポイントから線形時刻歴を再開する。
/// `state.step` の次のステップから `wave` の終端まで進める。
/// `wave` は全ステップ分の地震波（先頭から）。`state.step` 以降を使用する。
///
/// 戻り値の `ResponseResult` は**再開区間のみ**の部分記録である
/// （`time`・`history`・`recording`・`peak_disp`・`story_drift_angle` いずれも
/// `state.step` 以降のステップのみを対象に集計される。それ以前のステップの
/// 応答・ピークは含まれないため、チェックポイント再開前後の全区間を通じた
/// ピークが必要な場合は、区間ごとの `ResponseResult` を呼び出し側で合成すること）。
#[allow(clippy::too_many_arguments)]
pub fn linear_time_history_from_state(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    wave: &GroundMotion,
    newmark: &NewmarkCfg,
    damping: &Damping,
    state: &TimeStepState,
    use_kg: bool,
    record_every: Option<usize>,
) -> Result<(ResponseResult, TimeStepState), SolveError> {
    squid_n_math::parallelism::apply_to_faer();

    let dt = resolve_dt(newmark.dt, wave)?;

    let n_indep = reducer.n_indep;
    if n_indep == 0 || state.disp_red.len() != n_indep {
        return Err(SolveError::Backend(
            "time history restart: state dimension mismatch".into(),
        ));
    }

    let mut setup = LinearSetup::build(model, dofmap, reducer, newmark, damping, dt, use_kg)?;

    let u = state.disp_red.clone();
    let v = state.vel_red.clone();
    let a = state.accel_red.clone();

    run_steps(
        model,
        dofmap,
        reducer,
        wave,
        dt,
        state.step,
        &mut setup,
        u,
        v,
        a,
        record_every,
    )
}

/// 線形時刻歴の前処理一式。要素の弾性 behavior・質量／減衰行列（縮約空間）・
/// 地動入力の影響ベクトル・Newmark 係数・分解済みの有効剛性ソルバを持つ。
struct LinearSetup {
    /// 部材内力記録用の弾性 behavior。線形なので時刻歴を通じて不変。
    behaviors: Vec<Box<dyn ElementBehavior>>,
    m_free: faer::sparse::SparseColMat<usize, f64>,
    m_red: faer::sparse::SparseColMat<usize, f64>,
    /// 初期加速度の算定（K·u₀）で使う。
    k_red: faer::sparse::SparseColMat<usize, f64>,
    c_red: faer::sparse::SparseColMat<usize, f64>,
    infl: GroundInfluence,
    coeffs: NewmarkCoeffs,
    /// 有効剛性 K^ = K + c2·C + c1·M を分解済みのソルバ。
    solver: Box<dyn squid_n_math::solver::LinearSolver>,
}

impl LinearSetup {
    fn build(
        model: &Model,
        dofmap: &DofMap,
        reducer: &Reducer,
        newmark: &NewmarkCfg,
        damping: &Damping,
        dt: f64,
        use_kg: bool,
    ) -> Result<Self, SolveError> {
        if use_kg {
            return Err(SolveError::InvalidInput(
                "線形時刻歴応答解析の幾何剛性（P-Δ、use_kg）は未対応です。\
                 P-Δ を考慮する場合は非線形時刻歴応答解析を使用してください。"
                    .into(),
            ));
        }
        let behaviors: Vec<Box<dyn ElementBehavior>> = model
            .elements
            .iter()
            .map(|e| build_behavior(e, model))
            .collect();

        let m_free = assemble_global_m(model, dofmap, MassOption::Consistent);
        let k_free = assemble_global_k(model, dofmap);
        let m_red = reducer.reduce_k(&m_free);
        let k_red = reducer.reduce_k(&k_free);
        let c_red = damping.assemble_c(&m_red, &k_red);

        let infl = GroundInfluence::build(model, dofmap, &m_free);
        let coeffs = NewmarkCoeffs::new(newmark, dt);

        let k_eff = squid_n_math::sparse::weighted_sum_csc(
            reducer.n_indep,
            &[(1.0, &k_red), (coeffs.c2, &c_red), (coeffs.c1, &m_red)],
        );
        let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
        solver.factorize(&k_eff)?;

        Ok(Self {
            behaviors,
            m_free,
            m_red,
            k_red,
            c_red,
            infl,
            coeffs,
            solver,
        })
    }
}

/// 時刻歴ステップを `start_step` から `wave` の終端まで進める内部関数。
/// `start_step` は既に確定した状態（u, v, a は step `start_step` の値）。
/// 次のステップ `start_step` → `start_step+1` は `wave[start_step]` を使う。
#[allow(clippy::too_many_arguments)]
fn run_steps(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    wave: &GroundMotion,
    dt: f64,
    start_step: u64,
    setup: &mut LinearSetup,
    mut u: Vec<f64>,
    mut v: Vec<f64>,
    mut a: Vec<f64>,
    record_every: Option<usize>,
) -> Result<(ResponseResult, TimeStepState), SolveError> {
    let LinearSetup {
        behaviors,
        m_free,
        m_red,
        c_red,
        infl,
        coeffs,
        solver,
        ..
    } = setup;
    let NewmarkCoeffs {
        gamma,
        c1,
        c2,
        c3,
        c4,
        c5,
        c6,
        ..
    } = *coeffs;
    let m_r_x: &[f64] = &infl.m_r_x;
    let m_r_y: &[f64] = &infl.m_r_y;

    let n_indep = reducer.n_indep;
    let n_free = dofmap.n_active();

    let mut u_free = vec![0.0f64; n_free];
    let mut v_free = vec![0.0f64; n_free];
    let mut a_free = vec![0.0f64; n_free];
    reducer.expand_u_into(&u, &mut u_free);
    reducer.expand_u_into(&v, &mut v_free);
    reducer.expand_u_into(&a, &mut a_free);

    let mut peak_disp_free = vec![0.0f64; n_free];
    for i in 0..n_free {
        peak_disp_free[i] = peak_disp_free[i].max(u_free[i].abs());
    }
    let mut story_drift_angle = vec![0.0f64; model.layer_count()];
    update_story_drift(model, dofmap, &u_free, &mut story_drift_angle);

    let mut time = Vec::with_capacity(wave.accel_x.len() - start_step as usize + 1);
    time.push(start_step as f64 * dt);

    let mut ma_free = vec![0.0f64; n_free];
    mass_accel_free_into(m_free, &a_free, &mut ma_free);

    let mut recorder = ThRecorder::new(
        model,
        dofmap,
        wave.accel_x.len(),
        model.elements.len(),
        record_every,
    );
    let xg_x_init = wave
        .accel_x
        .get(start_step as usize)
        .copied()
        .unwrap_or(0.0);
    let xg_y_init = wave
        .accel_y
        .as_ref()
        .and_then(|acc| acc.get(start_step as usize).copied())
        .unwrap_or(0.0);
    let mf_init = member_forces_linear(dofmap, behaviors, &u_free);
    recorder.record_step(
        start_step,
        start_step as f64 * dt,
        model,
        dofmap,
        m_r_x,
        m_r_y,
        &ma_free,
        &u_free,
        &v_free,
        &a_free,
        xg_x_init,
        xg_y_init,
        &mf_init,
    );

    let record_dir_y = choose_record_dir_y(wave);
    let dir_idx = if record_dir_y { 1 } else { 0 };
    let m_r_record = if record_dir_y { m_r_y } else { m_r_x };
    let mut history = ResponseHistory {
        node: pick_record_node(model, dofmap, dir_idx),
        record_dir_y,
        ..Default::default()
    };
    let rmr_record = total_mass(m_r_record, dofmap, model.nodes.len(), dir_idx);
    let xg_init = if record_dir_y {
        wave.accel_y
            .as_ref()
            .and_then(|a| a.get(start_step as usize).copied())
            .unwrap_or(0.0)
    } else {
        wave.accel_x
            .get(start_step as usize)
            .copied()
            .unwrap_or(0.0)
    };
    record_history_step(
        &mut history,
        model,
        dofmap,
        dir_idx,
        rmr_record,
        &u_free,
        &ma_free,
        xg_init,
    );

    let mut p_free_buf = vec![0.0f64; n_free];
    let mut p_red_buf = vec![0.0f64; n_indep];
    let mut mw_buf = vec![0.0f64; n_indep];
    let mut cw_buf = vec![0.0f64; n_indep];
    let mut m_mw_buf = vec![0.0f64; n_indep];
    let mut c_cw_buf = vec![0.0f64; n_indep];
    let mut p_eff_buf = vec![0.0f64; n_indep];
    let mut u_next_buf = vec![0.0f64; n_indep];

    for n in start_step as usize..wave.accel_x.len() {
        let t_next = (n + 1) as f64 * dt;
        infl.force_at_into(wave, n, &mut p_free_buf);
        reducer.reduce_f_into(&p_free_buf, &mut p_red_buf);

        for i in 0..n_indep {
            mw_buf[i] = c1 * u[i] + c3 * v[i] + c4 * a[i];
            cw_buf[i] = c2 * u[i] + c5 * v[i] + c6 * a[i];
        }
        sparse_matvec_into(m_red, &mw_buf, &mut m_mw_buf);
        sparse_matvec_into(c_red, &cw_buf, &mut c_cw_buf);

        for i in 0..n_indep {
            p_eff_buf[i] = p_red_buf[i] + m_mw_buf[i] + c_cw_buf[i];
        }

        solver.solve_into(&p_eff_buf, &mut u_next_buf)?;
        let u_next = &u_next_buf;

        for i in 0..n_indep {
            let a_new = c1 * (u_next[i] - u[i]) - c3 * v[i] - c4 * a[i];
            let v_new = v[i] + dt * ((1.0 - gamma) * a[i] + gamma * a_new);
            v[i] = v_new;
            a[i] = a_new;
            u[i] = u_next[i];
        }
        time.push(t_next);

        reducer.expand_u_into(&u, &mut u_free);
        for i in 0..n_free {
            peak_disp_free[i] = peak_disp_free[i].max(u_free[i].abs());
        }
        update_story_drift(model, dofmap, &u_free, &mut story_drift_angle);
        reducer.expand_u_into(&v, &mut v_free);
        reducer.expand_u_into(&a, &mut a_free);
        mass_accel_free_into(m_free, &a_free, &mut ma_free);
        let xg_next = accel_dir_at(wave, n + 1, record_dir_y);
        record_history_step(
            &mut history,
            model,
            dofmap,
            dir_idx,
            rmr_record,
            &u_free,
            &ma_free,
            xg_next,
        );

        let (xg_x_next, xg_y_next) = accel_xy_at(wave, n + 1);
        let mf_now = member_forces_linear(dofmap, behaviors, &u_free);
        recorder.record_step(
            (n + 1) as u64,
            t_next,
            model,
            dofmap,
            m_r_x,
            m_r_y,
            &ma_free,
            &u_free,
            &v_free,
            &a_free,
            xg_x_next,
            xg_y_next,
            &mf_now,
        );
    }

    let final_step = wave.accel_x.len() as u64;
    let final_time = final_step as f64 * dt;
    let final_state = TimeStepState {
        step: final_step,
        time: final_time,
        disp_red: u.clone(),
        vel_red: v.clone(),
        accel_red: a.clone(),
    };

    let peak_disp = expand_peak_disp(model, dofmap, &peak_disp_free);

    Ok((
        ResponseResult {
            non_converged_steps: 0,
            time,
            peak_disp,
            story_drift_angle,
            cumulative_ductility: vec![0.0; model.elements.len()],
            history,
            recording: Some(recorder.finish()),
            nonlinear: false,
            applied_long_term: false,
        },
        final_state,
    ))
}
