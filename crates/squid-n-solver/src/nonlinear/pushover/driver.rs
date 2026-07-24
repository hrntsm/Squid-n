//! プッシュオーバー解析の司令塔（P5 §7）。
//!
//! - [`pushover_analysis`] — 既存 API（節点変位を記録しない薄いラッパー）
//! - [`pushover_analysis_recording`] — 荷重制御・変位制御・弧長法の各フェーズを
//!   実行し、ヒンジ・せん断降伏・崩壊機構・部材別応答を集約する本体

use super::assembly::{assemble_k, compute_f_int};
use super::ductility::{compute_ductility_refs, update_ductility, DuctilityTracker};
use super::hinge::{compute_hinge_thresholds, track_hinges};
use super::mechanism::determine_mechanism;
use super::member_response::compute_member_response;
use super::response::{
    compute_base_shear, compute_story_drift, compute_story_shear, get_roof_disp, get_roof_dof,
};
use super::shear_yield::{compute_shear_yield_thresholds, track_shear_yield};
use super::types::{CapacityPoint, DuctilityMethod, PushoverResult, PushoverStep};
use crate::analysis::{
    building_height_mm, distribute_pi_over_diaphragms, steel_height_ratio, SeismicDir,
};
use crate::arc_length::ArcLengthSolver;
use crate::constraint::Reducer;
use crate::transaction::{StateSnapshot, StatefulModel};
use smallvec::SmallVec;
use squid_n_core::dof::DofMap;
use squid_n_core::model::Model;
use squid_n_element::behavior::{Ctx, ElementBehavior, LocalVec};
use squid_n_element::factory::{build_nonlinear_behavior, StrengthBasis};
use squid_n_math::solver::{make_solver, SolverBackend};

/// プッシュオーバー解析（P5 §7）
#[allow(clippy::too_many_arguments)]
pub fn pushover_analysis(
    model: &mut Model,
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
        max_disp,
        use_kg,
        use_arc_length,
        arc_length_dl,
        false,
        DuctilityMethod::default(),
    )
}

/// プッシュオーバー解析（P5 §7）。`record_node_disp` が真の場合、各ステップの
/// `PushoverStep::node_disp` に全自由節点変位を記録する（段階的耐力喪失解析の
/// 部材変形角算定用、`strength_loss` モジュール参照）。既存 API を壊さないよう
/// `pushover_analysis` は本関数に `record_node_disp = false` で委譲する薄いラッパー。
#[allow(clippy::too_many_arguments)]
pub fn pushover_analysis_recording(
    model: &mut Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    dir: SeismicDir,
    max_steps: usize,
    max_disp: f64,
    use_kg: bool,
    use_arc_length: bool,
    arc_length_dl: f64,
    record_node_disp: bool,
    ductility_method: DuctilityMethod,
) -> Result<PushoverResult, String> {
    let n_active = dofmap.n_active();
    if n_active == 0 {
        return Err("no active DOF".into());
    }

    // 保有水平耐力計算の材料強度: 部材組み立て時に鋼材 fy・RC 主筋 σy へ
    // 材料強度係数（鋼材1.1倍/590N級1.05倍/RC主筋1.1倍、直接入力係数優先）を
    // 都度乗じる（`StrengthBasis::MaterialStrength`）。モデル自体は複製しない。
    let mut behaviors: Vec<Box<dyn ElementBehavior>> = Vec::new();
    for elem in &model.elements {
        let (b, _) = build_nonlinear_behavior(elem, model, StrengthBasis::MaterialStrength);
        behaviors.push(b);
    }
    // 静的解析: コンクリート履歴は逆行型（本実装の既定）。
    for b in behaviors.iter_mut() {
        b.set_concrete_hysteresis(false);
    }

    // 塑性率（ductility）トラッカー: 各部材の塑性率基点曲率・最大応答曲率を追跡する。
    let ductility_refs = compute_ductility_refs(model);
    let mut ductility_trackers: Vec<DuctilityTracker> =
        vec![DuctilityTracker::default(); model.elements.len()];

    let stories = &model.stories;
    if stories.is_empty() {
        return Err("no stories defined".into());
    }
    // h は建築物の高さ（GL〜PH 階を除く最上階。令88条・告示1793号）。
    // steel_height_ratio / building_height_mm は analysis.rs の
    // seismic_static_with と共有する実装。
    let height_m = building_height_mm(model) / 1000.0;
    let steel_ratio = steel_height_ratio(model);
    let t = squid_n_load::ai::approx_t(height_m, steel_ratio);
    let z = 1.0;
    let tc = squid_n_load::ai::tc_of(squid_n_load::ai::SoilClass::II);
    let rt_val = squid_n_load::ai::rt(t, tc);
    let c0 = 0.2;
    let story_weights: Vec<f64> = stories
        .iter()
        .map(|s| s.seismic_weight.unwrap_or(0.0))
        .collect();
    if story_weights.iter().all(|&w| w == 0.0) {
        return Err("no seismic weight defined".into());
    }
    let ai = squid_n_load::ai::ai_distribution(&story_weights, z, rt_val, c0, t);

    let dir_vec = match dir {
        SeismicDir::X => [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        SeismicDir::Y => [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    };
    let mut q = vec![0.0; n_active];
    for (i, story) in stories.iter().enumerate() {
        let pi = ai.pi.get(i).copied().unwrap_or(0.0);
        if pi == 0.0 {
            continue;
        }
        // 多剛床の階では重量比で按分する（レビュー §1.6、analysis.rs と同じ規則。
        // 従来は各剛床へ pi をそのまま重複して載せていた）。
        for (master, share) in distribute_pi_over_diaphragms(story, pi) {
            let ni = master.index();
            for d in 0..6 {
                let g = ni * 6 + d;
                if let Some(a) = dofmap.active(g) {
                    q[a as usize] += dir_vec[d] * share;
                }
            }
        }
    }

    let thresholds = compute_hinge_thresholds(model);
    let shear_thresholds = compute_shear_yield_thresholds(model);
    let mut hinges = Vec::new();
    let mut shear_yields = Vec::new();
    let mut capacity_curve = Vec::new();
    let mut steps: Vec<PushoverStep> = Vec::new();
    let mut total_disp = vec![0.0; n_active];
    let n_steps = max_steps.clamp(1, 100);
    let dlambda = 1.0 / n_steps as f64;

    for step in 0..n_steps {
        // 前確定点の荷重係数（前ステップまでが満 λ で確定している前提の下限）。
        let prev_lambda = step as f64 * dlambda;
        let mut current_lambda = (step + 1) as f64 * dlambda;
        let mut step_ok = false;

        for _attempt in 0..5 {
            let snap = StateSnapshot::capture(&behaviors);
            let f_ext: Vec<f64> = q.iter().map(|&qi| qi * current_lambda).collect();
            let mut converged = false;
            // このステップ内の全 Newton 修正量の累積（＝ステップ変位増分）。
            // 従来は last_du_free に「最後の修正量」だけを保持しており、
            // 収束に 2 反復以上要する塑性ステップで途中の修正量が total_disp から
            // 脱落し、荷重−変位曲線の変位軸が過小評価されていた（要素内部状態は
            // 全修正量を累積しているため base_shear は正しく、変位のみ不整合）。
            let mut step_du_free = vec![0.0; n_active];

            // Newton 反復上限。全要素がトライアル追従（internal_force が反復中の
            // 未確定変位を反映する）となったため、弾性支配の状態ではほぼ 1〜2 回で
            // 収束する。上限 50 は塑性進行時（接線更新を要する反復）の余裕。
            for _iter in 0..50 {
                let k_free = assemble_k(model, dofmap, &behaviors, use_kg, None);
                let k_red = reducer.reduce_k(&k_free);
                let f_int = compute_f_int(model, dofmap, &behaviors);
                let r_free: Vec<f64> = f_ext.iter().zip(f_int.iter()).map(|(e, i)| e - i).collect();
                let r_red = reducer.reduce_f(&r_free);

                let f_ext_red = reducer.reduce_f(&f_ext);
                let r_norm: f64 = r_red.iter().map(|x| x * x).sum::<f64>().sqrt();
                let f_norm: f64 = f_ext_red.iter().map(|x| x * x).sum::<f64>().sqrt();
                if r_norm < 1e-6 * f_norm.max(1.0) {
                    converged = true;
                    break;
                }

                let mut solver = make_solver(SolverBackend::Auto);
                solver
                    .factorize(&k_red)
                    .map_err(|e| format!("factor: {:?}", e))?;
                let du_red = solver
                    .solve(&r_red)
                    .map_err(|e| format!("solve: {:?}", e))?;
                let du_free = reducer.expand_u(&du_red);
                for (acc, &d) in step_du_free.iter_mut().zip(du_free.iter()) {
                    *acc += d;
                }

                let model_ref: &Model = model;
                for (_elem, b) in model_ref.elements.iter().zip(behaviors.iter_mut()) {
                    let gdofs = b.global_dofs(dofmap);
                    let mut du_elem = LocalVec {
                        data: SmallVec::from_elem(0.0, gdofs.len()),
                    };
                    for (i, &g) in gdofs.iter().enumerate() {
                        if g != usize::MAX {
                            du_elem.data[i] = du_free[g];
                        }
                    }
                    let ctx = Ctx { model: model_ref };
                    b.update_state(&du_elem, false, &ctx);
                }
            }

            if converged {
                for b in behaviors.iter_mut() {
                    b.commit_state();
                }
                for (&du, td) in step_du_free.iter().zip(total_disp.iter_mut()) {
                    *td += du;
                }
                let roof = get_roof_disp(&total_disp, model, dofmap, dir);
                // ベースシアは内力の釣合いから算定（載荷ベクトル総和でも一致するが、
                // 変位制御フェーズと統一し反力ベースで求める）。
                let f_int_now = compute_f_int(model, dofmap, &behaviors);
                let base_shear = compute_base_shear(model, dofmap, &f_int_now, dir);
                let story_drift = compute_story_drift(model, dofmap, &total_disp, dir);
                capacity_curve.push(CapacityPoint {
                    step: step as u32,
                    roof_disp: roof,
                    base_shear,
                    story_shear: compute_story_shear(model, dofmap, &f_int_now, dir),
                    story_drift: story_drift.clone(),
                });
                steps.push(PushoverStep {
                    // 荷重制御フェーズ: 参照外力ベクトル q に対する倍率 current_lambda を
                    // そのまま荷重係数として記録する。
                    load_factor: current_lambda,
                    top_disp: roof,
                    base_shear,
                    story_drifts: story_drift,
                    node_disp: record_node_disp.then(|| total_disp.clone()),
                });
                let mu = update_ductility(
                    &behaviors,
                    &mut ductility_trackers,
                    &ductility_refs,
                    ductility_method,
                );
                track_hinges(
                    model,
                    &behaviors,
                    &thresholds,
                    &mu,
                    step as u32,
                    &mut hinges,
                );
                track_shear_yield(
                    model,
                    &behaviors,
                    &shear_thresholds,
                    step as u32,
                    &mut shear_yields,
                );
                step_ok = true;
                if max_disp > 0.0 && roof >= max_disp {
                    break;
                }
                break;
            } else {
                model.restore(&snap, &mut behaviors);
                // 収束失敗時は「前確定点 prev_lambda からの増分」を半減する。絶対 λ を
                // 半減すると prev_lambda を下回り、前確定状態から除荷方向に解いて荷重−変位
                // 経路が非物理的にジグザグする（ヒンジ／せん断降伏追跡も汚染される）。
                // 増分のみを縮めることで単調載荷を保つ。
                current_lambda = prev_lambda + (current_lambda - prev_lambda) * 0.5;
            }
        }
        if !step_ok {
            // 収束に至らなかった step はスキップ
        }
    }

    // 変位制御フェーズ（P5 §7.1）
    if max_disp > 0.0 {
        if let Some(roof_active) = get_roof_dof(model, dofmap, dir) {
            let initial_disp = total_disp[roof_active];
            let n_disp_steps = 10usize;
            let du_target = (max_disp - initial_disp) / n_disp_steps as f64;

            // 変位制御のペナルティ剛性は「拘束する屋根 DOF 自身の対角剛性（＝加力方向の
            // 水平剛性）」の 1e6 倍とする。固定 1e16 も、全体最大対角（軸剛性 ~1e9 に支配）
            // 基準の過大な penalty も、残差 penalty·(target − u) の桁落ちで収束判定が
            // 成立せず、変位制御が1点も確定しないまま即終了していた（→ Qu が荷重制御
            // λ=1＝C0=0.2 級で頭打ちとなり崩壊機構へ到達しない致命的欠陥）。
            // 屋根 DOF 対角に比例させると、拘束は常に相対 1e-6 精度で強制されつつ、
            // 桁落ち残差は収束閾値に対し十分小さく保たれる（スケール不変）。
            let penalty = {
                let k0 = assemble_k(model, dofmap, &behaviors, use_kg, None);
                let k_roof_diag = squid_n_math::sparse::sparse_to_triplets(&k0)
                    .iter()
                    .filter(|t| t.row == roof_active && t.col == roof_active)
                    .map(|t| t.val.abs())
                    .fold(0.0_f64, f64::max)
                    .max(1.0);
                k_roof_diag * 1e6
            };

            for step in 0..n_disp_steps {
                let target = initial_disp + du_target * (step + 1) as f64;
                let mut step_ok = false;

                for _attempt in 0..5 {
                    let snap = StateSnapshot::capture(&behaviors);
                    let mut converged = false;
                    // 荷重制御フェーズと同じく、ステップ内の全 Newton 修正量を累積する。
                    let mut step_du_free = vec![0.0; n_active];

                    // 反復上限は荷重制御フェーズと同じ理由（準ニュートン形式）で 50 回。
                    for _iter in 0..50 {
                        let k_free = assemble_k(
                            model,
                            dofmap,
                            &behaviors,
                            use_kg,
                            Some((roof_active, penalty)),
                        );
                        let k_red = reducer.reduce_k(&k_free);
                        let f_int = compute_f_int(model, dofmap, &behaviors);

                        // ペナルティ法の残差: 自由 DOF は釣合い残差 −f_int、拘束 DOF（roof）
                        // はペナルティばね反力 penalty·(target − u_roof) を加える。
                        // u_roof = 確定変位 + ステップ内累積修正量。
                        // 従来は f_ext[roof]=penalty·target を作って −f_int するのみで、
                        // ペナルティばね内力 penalty·u_roof が残差から欠落していたため、
                        // 拘束 DOF の残差が常に penalty·target で頭打ちとなり収束不能だった。
                        // gap 形式 penalty·(target − u_roof) で桁落ちを避ける。
                        let u_roof = total_disp[roof_active] + step_du_free[roof_active];
                        let mut r_free: Vec<f64> = f_int.iter().map(|fi| -fi).collect();
                        r_free[roof_active] += penalty * (target - u_roof);
                        let r_red = reducer.reduce_f(&r_free);

                        // 収束は力の相対ノルム（内力ノルム基準＝ベースシア相当）で判定する。
                        let r_norm: f64 = r_red.iter().map(|x| x * x).sum::<f64>().sqrt();
                        let f_int_red = reducer.reduce_f(&f_int);
                        let f_scale: f64 =
                            f_int_red.iter().map(|x| x * x).sum::<f64>().sqrt().max(1.0);
                        if r_norm < 1e-6 * f_scale {
                            converged = true;
                            break;
                        }

                        let mut solver = make_solver(SolverBackend::Auto);
                        solver.factorize(&k_red).map_err(|e| format!("{:?}", e))?;
                        let du_red = solver.solve(&r_red).map_err(|e| format!("{:?}", e))?;
                        let du_free = reducer.expand_u(&du_red);
                        for (acc, &d) in step_du_free.iter_mut().zip(du_free.iter()) {
                            *acc += d;
                        }

                        let model_ref: &Model = model;
                        for (_elem, b) in model_ref.elements.iter().zip(behaviors.iter_mut()) {
                            let gdofs = b.global_dofs(dofmap);
                            let mut du_elem = LocalVec {
                                data: SmallVec::from_elem(0.0, gdofs.len()),
                            };
                            for (i, &g) in gdofs.iter().enumerate() {
                                if g != usize::MAX {
                                    du_elem.data[i] = du_free[g];
                                }
                            }
                            let ctx = Ctx { model: model_ref };
                            b.update_state(&du_elem, false, &ctx);
                        }
                    }

                    if converged {
                        for b in behaviors.iter_mut() {
                            b.commit_state();
                        }
                        for (&du, td) in step_du_free.iter().zip(total_disp.iter_mut()) {
                            *td += du;
                        }
                        let roof = get_roof_disp(&total_disp, model, dofmap, dir);
                        let f_int_now = compute_f_int(model, dofmap, &behaviors);
                        let base_shear = compute_base_shear(model, dofmap, &f_int_now, dir);
                        let story_drift = compute_story_drift(model, dofmap, &total_disp, dir);
                        let cstep = (n_steps + 1 + step) as u32;
                        capacity_curve.push(CapacityPoint {
                            step: cstep,
                            roof_disp: roof,
                            base_shear,
                            story_shear: compute_story_shear(model, dofmap, &f_int_now, dir),
                            story_drift: story_drift.clone(),
                        });
                        steps.push(PushoverStep {
                            // 変位制御フェーズ: 荷重制御フェーズで λ=1 まで到達した後の継続で、
                            // 目標頂部変位をペナルティ法で強制するため比例載荷の λ という概念が
                            // 存在しない。荷重制御完了時点の値(1.0)をそのまま保持して記録する。
                            load_factor: 1.0,
                            top_disp: roof,
                            base_shear,
                            story_drifts: story_drift,
                            node_disp: record_node_disp.then(|| total_disp.clone()),
                        });
                        let mu = update_ductility(
                            &behaviors,
                            &mut ductility_trackers,
                            &ductility_refs,
                            ductility_method,
                        );
                        track_hinges(model, &behaviors, &thresholds, &mu, cstep, &mut hinges);
                        track_shear_yield(
                            model,
                            &behaviors,
                            &shear_thresholds,
                            cstep,
                            &mut shear_yields,
                        );
                        step_ok = true;
                        break;
                    } else {
                        model.restore(&snap, &mut behaviors);
                    }
                }
                if !step_ok {
                    break;
                }
            }
        }
    }

    if use_arc_length {
        let arc_solver = ArcLengthSolver::new(arc_length_dl);
        let mut prev_du: Vec<f64> = Vec::new();
        let mut arc_lambda = 1.0;

        for _step in 0..20 {
            let snap = StateSnapshot::capture(&behaviors);
            let k_free = assemble_k(model, dofmap, &behaviors, use_kg, None);
            let k_red = reducer.reduce_k(&k_free);

            // ここは分解の失敗（正定値でない＝不安定化）を耐力喪失の終了判定に
            // 使うため、factorize が失敗し得る直接法を明示する（Auto の PCG 経路は
            // factorize では失敗しないので判定が効かなくなる）。
            let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
            if solver.factorize(&k_red).is_err() {
                model.restore(&snap, &mut behaviors);
                break;
            }

            // 弧長修正子の各反復で内力を再評価するため、変位増分 δu を要素状態へ
            // 反映して更新後 f_int を返すクロージャを渡す（接線 K はステップ開始時で固定＝修正 Newton）。
            let result = {
                let model_ref: &Model = &*model;
                let behaviors_ref = &mut behaviors;
                arc_solver.step(
                    &q,
                    &mut |r: &[f64]| -> Result<Vec<f64>, String> {
                        let r_red = reducer.reduce_f(r);
                        let du_red = solver.solve(&r_red).map_err(|e| format!("{:?}", e))?;
                        Ok(reducer.expand_u(&du_red))
                    },
                    &mut |delta_u: &[f64]| -> Result<Vec<f64>, String> {
                        let ctx = Ctx { model: model_ref };
                        for b in behaviors_ref.iter_mut() {
                            let gdofs = b.global_dofs(dofmap);
                            let mut du_elem = LocalVec {
                                data: SmallVec::from_elem(0.0, gdofs.len()),
                            };
                            for (i, &g) in gdofs.iter().enumerate() {
                                if g != usize::MAX && g < delta_u.len() {
                                    du_elem.data[i] = delta_u[g];
                                }
                            }
                            b.update_state(&du_elem, false, &ctx);
                        }
                        Ok(compute_f_int(model_ref, dofmap, behaviors_ref))
                    },
                    &prev_du,
                    arc_lambda,
                )
            };

            match result {
                Ok(step_result) if step_result.converged => {
                    // 要素状態は eval_fint で既に δu 反映済み。ここでは確定のみ。
                    for b in behaviors.iter_mut() {
                        b.commit_state();
                    }
                    for (&du, td) in step_result.du.iter().zip(total_disp.iter_mut()) {
                        *td += du;
                    }
                    arc_lambda += step_result.dlambda;
                    prev_du = step_result.du;

                    let roof = get_roof_disp(&total_disp, model, dofmap, dir);
                    let f_int_now = compute_f_int(model, dofmap, &behaviors);
                    let base_shear = compute_base_shear(model, dofmap, &f_int_now, dir);
                    let story_drift = compute_story_drift(model, dofmap, &total_disp, dir);
                    capacity_curve.push(CapacityPoint {
                        step: (n_steps + 1 + _step) as u32,
                        roof_disp: roof,
                        base_shear,
                        story_shear: compute_story_shear(model, dofmap, &f_int_now, dir),
                        story_drift: story_drift.clone(),
                    });
                    steps.push(PushoverStep {
                        // 弧長法: 各増分後に更新される荷重倍率 arc_lambda をそのまま記録する。
                        load_factor: arc_lambda,
                        top_disp: roof,
                        base_shear,
                        story_drifts: story_drift,
                        node_disp: record_node_disp.then(|| total_disp.clone()),
                    });
                }
                _ => {
                    model.restore(&snap, &mut behaviors);
                    break;
                }
            }
        }
    }

    let mechanism = determine_mechanism(&hinges, model, dir);
    // 保有水平耐力 Qu = 性能曲線上の最大ベースシア（崩壊機構形成時の水平耐力）。
    // 単調載荷では機構形成後に頭打ちとなるため、ピーク値を採る。
    let qu = capacity_curve
        .iter()
        .map(|c| c.base_shear)
        .fold(0.0_f64, f64::max);
    // 最終確定ステップの部材別応答（終局検定の設計用応力・部材別 Rp の直接反映用）。
    // ステップが 1 つも確定しなかった場合は空を返す。
    let member_response = if steps.is_empty() {
        Vec::new()
    } else {
        compute_member_response(model, dofmap, &behaviors, &total_disp)
    };
    Ok(PushoverResult {
        steps,
        capacity_curve,
        hinges,
        shear_yields,
        mechanism,
        qu,
        member_response,
    })
}
