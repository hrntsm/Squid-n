//! 非線形時刻歴応答解析（Newmark-β + Newton 反復 + commit/rollback）。
//!
//! - [`nonlinear_time_history_analysis`] — 非線形時刻歴応答解析

use super::common::{solve_initial_accel, theta_accel_at, theta_influence_m};
use super::config::{GroundMotion, NewmarkCfg};
use super::history::{
    choose_record_dir_y, pick_record_node, record_history_step, total_mass, update_story_drift,
};
use super::result::{ResponseHistory, ResponseResult};
use crate::assemble::{assemble_global_k, assemble_global_m};
use crate::constraint::Reducer;
use crate::damping::{Damping, DampingAccumulation};
use crate::pushover::{assemble_k, compute_f_int};
use crate::transaction::{StateSnapshot, StatefulModel};
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::model::Model;
use squid_n_element::behavior::{Ctx, LocalVec, MassOption};
use squid_n_element::factory::{build_nonlinear_behavior, StrengthBasis};
use squid_n_math::solver::{make_solver, SolveError, SolverBackend};
use squid_n_math::sparse::{sparse_matvec, weighted_sum_csc};

/// 非線形時刻歴応答解析（Newmark-β + Newton反復 + commit/rollback）。
///
/// 各時刻ステップで Newton 反復により内力（非線形復元力）と慣性力・減衰力・
/// 地震外力の動的釣合いを満たす解を求める。収束時は要素状態を commit、
/// 不収束時は Step 開始時の状態へ rollback する。
#[allow(clippy::too_many_arguments)]
pub fn nonlinear_time_history_analysis(
    model: &mut Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    wave: &GroundMotion,
    newmark: &NewmarkCfg,
    damping: &Damping,
    accumulation: DampingAccumulation,
    initial_disp: &[f64],
    initial_vel: &[f64],
    use_kg: bool,
    max_iter: usize,
    tol: f64,
) -> Result<ResponseResult, SolveError> {
    squid_n_math::parallelism::apply_to_faer();

    // 部材の終局耐力を算定できない設定不備（耐震壁の Qu、線材の材料強度未入力）は、
    // 代替値で埋めず解析を止める（プッシュオーバーと同じ規約）。耐力が定まらない
    // 部材は弾性のまま際限なく応力を負担し、応答を過小評価する（危険側）。
    squid_n_element::factory::ensure_nonlinear_input(model).map_err(SolveError::InvalidInput)?;

    let dt = if newmark.dt > 0.0 {
        newmark.dt
    } else {
        wave.dt
    };
    if dt <= 0.0 {
        return Err(SolveError::Backend(
            "time history: dt must be positive".into(),
        ));
    }

    let n_indep = reducer.n_indep;
    if n_indep == 0 {
        return Ok(ResponseResult {
            time: vec![],
            peak_disp: vec![[0.0; 6]; model.nodes.len()],
            story_drift_angle: vec![0.0; model.stories.len()],
            cumulative_ductility: vec![0.0; model.elements.len()],
            history: ResponseHistory::default(),
        });
    }

    let mut behaviors = build_behaviors(model);
    // 制振（速度依存）要素へ時間刻みを通知する（制振要素、Maxwell モデル等）。マクスウェル
    // 要素はこれで後退 Euler のダッシュポット積分が有効になる。dt<=0 の静的・線形解析
    // では通知されず不活性のまま。
    for b in behaviors.iter_mut() {
        b.set_time_step(dt);
    }
    // 累積損傷度用の塑性率 μ 時刻歴（要素ごと。塑性率プローブを持つ要素のみ収集）。
    // レインフロー法（ASTM E1049-85）・Miner 則による鉄骨梁端部の累積損傷度計算。
    let mut mu_hist: Vec<Vec<f64>> = vec![Vec::new(); model.elements.len()];

    // 行列組立（縮約空間）
    let m_free = assemble_global_m(model, dofmap, MassOption::Consistent);
    let k_free = assemble_global_k(model, dofmap);
    let _ = use_kg;
    let m_red = reducer.reduce_k(&m_free);
    let k_red = reducer.reduce_k(&k_free);
    let c_red = damping.assemble_c(&m_red, &k_red);

    // 影響ベクトルと M·r
    let n_free = dofmap.n_active();
    let mut r_x_free = vec![0.0; n_free];
    let mut r_y_free = vec![0.0; n_free];
    for ni in 0..model.nodes.len() {
        let g_ux = ni * DOF_PER_NODE + 0;
        let g_uy = ni * DOF_PER_NODE + 1;
        if let Some(a) = dofmap.active(g_ux) {
            r_x_free[a as usize] = 1.0;
        }
        if let Some(a) = dofmap.active(g_uy) {
            r_y_free[a as usize] = 1.0;
        }
    }
    let m_r_x = sparse_matvec(&m_free, &r_x_free);
    let m_r_y = sparse_matvec(&m_free, &r_y_free);
    // 位相差入力（ねじれ加振）用の回転影響 M·r_θ。
    let m_r_theta = theta_influence_m(model, dofmap, &m_free);

    // Newmark-β 係数
    let beta = newmark.beta;
    let gamma = newmark.gamma;
    let c1 = 1.0 / (beta * dt * dt);
    let c2 = gamma / (beta * dt);
    let c3 = 1.0 / (beta * dt);
    let c4 = 1.0 / (2.0 * beta) - 1.0;
    let c5 = gamma / beta - 1.0;
    let c6 = dt * (gamma / (2.0 * beta) - 1.0);

    // 初期条件
    let mut u = vec![0.0; n_indep];
    let mut v = vec![0.0; n_indep];
    let n_init_d = n_indep.min(initial_disp.len());
    u[..n_init_d].copy_from_slice(&initial_disp[..n_init_d]);
    let n_init_v = n_indep.min(initial_vel.len());
    v[..n_init_v].copy_from_slice(&initial_vel[..n_init_v]);

    // 累積型減衰力 {Cn}（初期は C·v0）と、各ステップ収束時の減衰力（累積更新用）。
    let mut f_damp = sparse_matvec(&c_red, &v);
    let mut c_v_last = vec![0.0; n_indep];

    // h1 一定減衰の {u} は「初期剛性による1次の固有ベクトル」（時刻歴を通じて固定）。
    // 現在変位を用いると高次成分・剛体成分が混入し ω1 の推定が乱れる。
    // 固有値解析が失敗した場合は零ベクトルとし、assemble_c_tangent 側の
    // フォールバック（ω1 = ω1e）に委ねる。
    let u_mode1: Vec<f64> = if matches!(damping, Damping::TangentStiffnessConstantH { .. }) {
        crate::eigen::solve_eigen(model, dofmap, reducer, 1)
            .ok()
            .and_then(|modal| modal.shapes.into_iter().next())
            .filter(|s| s.len() == n_indep)
            .unwrap_or_else(|| vec![0.0; n_indep])
    } else {
        vec![0.0; n_indep]
    };

    // 初期変位を要素状態に反映
    {
        let u_free_init = reducer.expand_u(&u);
        let model_ref: &Model = model;
        for (_elem, b) in model_ref.elements.iter().zip(behaviors.iter_mut()) {
            let gdofs = b.global_dofs(dofmap);
            let mut du_elem = LocalVec {
                data: SmallVec::from_elem(0.0, gdofs.len()),
            };
            for (i, &g) in gdofs.iter().enumerate() {
                if g != usize::MAX && g < u_free_init.len() {
                    du_elem.data[i] = u_free_init[g];
                }
            }
            let ctx = Ctx { model: model_ref };
            b.update_state(&du_elem, false, &ctx);
        }
        for b in behaviors.iter_mut() {
            b.commit_state();
        }
    }

    // 初期加速度: M·a_0 = -C·v_0 - f_int(u_0) - p_red(0)
    let xg0_x = wave.accel_x.first().copied().unwrap_or(0.0);
    let xg0_y = wave
        .accel_y
        .as_ref()
        .and_then(|a| a.first())
        .copied()
        .unwrap_or(0.0);
    let xg0_theta = theta_accel_at(wave, 0);
    let p_free_0: Vec<f64> = m_r_x
        .iter()
        .zip(m_r_y.iter())
        .zip(m_r_theta.iter())
        .map(|((mx, my), mt)| -(mx * xg0_x + my * xg0_y + mt * xg0_theta))
        .collect();
    let p_red_0 = reducer.reduce_f(&p_free_0);

    let f_int0_free = compute_f_int(model, dofmap, &behaviors);
    let f_int0_red = reducer.reduce_f(&f_int0_free);
    let cv0 = sparse_matvec(&c_red, &v);
    let mut rhs_a0 = vec![0.0; n_indep];
    for i in 0..n_indep {
        // p(0) は符号込み（−M·r·ẍg）。線形版と同じく加算が正しい。
        rhs_a0[i] = p_red_0[i] - cv0[i] - f_int0_red[i];
    }
    let mut a = solve_initial_accel(&m_red, &rhs_a0, n_indep)?;

    // --- 時刻歴ループ ---
    let n_steps = wave.accel_x.len();
    let mut peak_disp_free = vec![0.0f64; n_free];
    {
        let u_free_init = reducer.expand_u(&u);
        for i in 0..n_free {
            peak_disp_free[i] = peak_disp_free[i].max(u_free_init[i].abs());
        }
    }
    let mut story_drift_angle = vec![0.0f64; model.stories.len()];
    {
        let u_free_init = reducer.expand_u(&u);
        update_story_drift(model, dofmap, &u_free_init, &mut story_drift_angle);
    }

    // UI 用の代表応答記録（記録方向は入力加速度の絶対値和が大きい方を自動選択）
    let record_dir_y = choose_record_dir_y(wave);
    let dir_idx = if record_dir_y { 1 } else { 0 };
    let m_r_record: &[f64] = if record_dir_y { &m_r_y } else { &m_r_x };
    let mut history = ResponseHistory {
        node: pick_record_node(model, dofmap, dir_idx),
        record_dir_y,
        ..Default::default()
    };
    let rmr_record = total_mass(m_r_record, dofmap, model.nodes.len(), dir_idx);
    {
        let u_free_init = reducer.expand_u(&u);
        let xg_init = if record_dir_y {
            wave.accel_y
                .as_ref()
                .and_then(|a| a.first().copied())
                .unwrap_or(0.0)
        } else {
            wave.accel_x.first().copied().unwrap_or(0.0)
        };
        record_history_step(
            &mut history,
            model,
            dofmap,
            reducer,
            dir_idx,
            m_r_record,
            rmr_record,
            &u_free_init,
            &a,
            xg_init,
        );
    }
    let mut time = Vec::with_capacity(n_steps + 1);
    time.push(0.0);

    for n in 0..n_steps {
        let t_next = (n + 1) as f64 * dt;

        // スナップショット
        let snap = StateSnapshot::capture(&behaviors);

        // 地震荷重
        let xg_x = wave.accel_x[n];
        let xg_y = wave
            .accel_y
            .as_ref()
            .map(|a| a.get(n).copied().unwrap_or(0.0))
            .unwrap_or(0.0);
        let xg_theta = theta_accel_at(wave, n);
        let p_free: Vec<f64> = m_r_x
            .iter()
            .zip(m_r_y.iter())
            .zip(m_r_theta.iter())
            .map(|((mx, my), mt)| -(mx * xg_x + my * xg_y + mt * xg_theta))
            .collect();
        let p_red = reducer.reduce_f(&p_free);

        // 予測子: Δu = 0 での a, v
        let mut a_trial = vec![0.0; n_indep];
        let mut v_trial = vec![0.0; n_indep];
        for i in 0..n_indep {
            a_trial[i] = -c3 * v[i] - c4 * a[i];
            v_trial[i] = -c5 * v[i] - c6 * a[i];
        }

        let mut du_total = vec![0.0; n_indep];
        let mut converged = false;

        for _iter in 0..max_iter {
            // 接線剛性
            let k_t_free = assemble_k(model, dofmap, &behaviors, use_kg);
            let k_t_red = reducer.reduce_k(&k_t_free);

            // 接線比例減衰（α1 一定 / h1 一定）は瞬間剛性から C を毎ステップ再構成する
            // （剛性変更に伴う減衰項の変更、構造動力学）。それ以外は
            // 初期減衰 c_red を用いる。
            let c_tan = if damping.is_tangent_based() {
                // h1 一定の {u} は初期剛性の1次固有ベクトル（u_mode1、固定）。
                // α1 一定は u を参照しない。
                Some(damping.assemble_c_tangent(&m_red, &k_t_red, &k_red, &u_mode1))
            } else {
                None
            };
            let c_cur = c_tan.as_ref().unwrap_or(&c_red);

            // 有効剛性
            let k_eff = weighted_sum_csc(n_indep, &[(1.0, &k_t_red), (c2, c_cur), (c1, &m_red)]);

            let mut solver = make_solver(SolverBackend::Auto);
            solver
                .factorize(&k_eff)
                .map_err(|e| SolveError::Backend(format!("factor: {:?}", e)))?;

            // 内力
            let f_int_free = compute_f_int(model, dofmap, &behaviors);
            let f_int_red = reducer.reduce_f(&f_int_free);

            // 減衰力（縮約空間）。非累積型は瞬間 C×速度、累積型は増分減衰力の積分
            // （{Cn}={Cn−1}+[Cn]{Δẋn}、Δẋn=v_trial−v_前ステップ）。
            let c_v_red = match accumulation {
                DampingAccumulation::NonCumulative => sparse_matvec(c_cur, &v_trial),
                DampingAccumulation::Cumulative => {
                    let dv: Vec<f64> = (0..n_indep).map(|i| v_trial[i] - v[i]).collect();
                    let c_dv = sparse_matvec(c_cur, &dv);
                    (0..n_indep).map(|i| f_damp[i] + c_dv[i]).collect()
                }
            };
            c_v_last.clone_from(&c_v_red);
            let m_a_red = sparse_matvec(&m_red, &a_trial);

            // 残差
            let mut r_red = vec![0.0; n_indep];
            for i in 0..n_indep {
                r_red[i] = p_red[i] - f_int_red[i] - c_v_red[i] - m_a_red[i];
            }

            // 収束判定
            let r_norm: f64 = r_red.iter().map(|x| x * x).sum::<f64>().sqrt();
            let p_norm: f64 = p_red.iter().map(|x| x * x).sum::<f64>().sqrt();
            if r_norm < tol * p_norm.max(1.0) {
                converged = true;
                break;
            }

            // δu を解く
            let du_red = solver.solve(&r_red)?;
            let du_free = reducer.expand_u(&du_red);

            // a, v を更新
            for i in 0..n_indep {
                a_trial[i] += c1 * du_red[i];
                v_trial[i] += c2 * du_red[i];
                du_total[i] += du_red[i];
            }

            // 要素状態を trial 更新
            let model_ref: &Model = model;
            for (_elem, b) in model_ref.elements.iter().zip(behaviors.iter_mut()) {
                let gdofs = b.global_dofs(dofmap);
                let mut du_elem = LocalVec {
                    data: SmallVec::from_elem(0.0, gdofs.len()),
                };
                for (i, &g) in gdofs.iter().enumerate() {
                    if g != usize::MAX && g < du_free.len() {
                        du_elem.data[i] = du_free[g];
                    }
                }
                let ctx = Ctx { model: model_ref };
                b.update_state(&du_elem, false, &ctx);
            }
        }

        if converged {
            for i in 0..n_indep {
                u[i] += du_total[i];
            }
            // 累積型: 収束した減衰力を次ステップの積分開始値として保持する。
            if accumulation == DampingAccumulation::Cumulative {
                f_damp.clone_from(&c_v_last);
            }
            v.copy_from_slice(&v_trial);
            a.copy_from_slice(&a_trial);

            for b in behaviors.iter_mut() {
                b.commit_state();
            }

            // 累積損傷度用に、各要素の危険断面塑性率 μ（=max_yield_ratio）を収集する。
            for (i, b) in behaviors.iter().enumerate() {
                if let Some(p) = b.ductility_probe() {
                    mu_hist[i].push(p.max_yield_ratio);
                }
            }

            time.push(t_next);

            let u_free = reducer.expand_u(&u);
            for i in 0..n_free {
                peak_disp_free[i] = peak_disp_free[i].max(u_free[i].abs());
            }
            update_story_drift(model, dofmap, &u_free, &mut story_drift_angle);
            let xg_next = if record_dir_y {
                wave.accel_y
                    .as_ref()
                    .and_then(|a| a.get(n + 1).copied())
                    .unwrap_or(0.0)
            } else {
                wave.accel_x.get(n + 1).copied().unwrap_or(0.0)
            };
            record_history_step(
                &mut history,
                model,
                dofmap,
                reducer,
                dir_idx,
                m_r_record,
                rmr_record,
                &u_free,
                &a,
                xg_next,
            );
        } else {
            // 不収束: rollback
            model.restore(&snap, &mut behaviors);
            return Err(SolveError::Backend(format!(
                "nonlinear time history: step {} did not converge",
                n
            )));
        }
    }

    let mut peak_disp = vec![[0.0f64; 6]; model.nodes.len()];
    for ni in 0..model.nodes.len() {
        for d in 0..DOF_PER_NODE {
            let g = ni * DOF_PER_NODE + d;
            if let Some(a) = dofmap.active(g) {
                peak_disp[ni][d] = peak_disp_free[a as usize];
            }
        }
    }

    // 各要素の μ 時刻歴からレインフロー法で累積損傷度 D を算定する
    // （レインフロー法（ASTM E1049-85）・Miner 則。鉄骨梁端部の累積損傷度計算）。μ 時刻歴が空（塑性率プローブ
    // 非対応要素）の場合は 0。疲労特性 C・β は既定（要原典照合）。
    let fatigue = crate::damage::FatigueParams::default();
    let cumulative_ductility: Vec<f64> = mu_hist
        .iter()
        .map(|series| crate::damage::cumulative_damage_rainflow(series, fatigue))
        .collect();

    Ok(ResponseResult {
        time,
        peak_disp,
        story_drift_angle,
        cumulative_ductility,
        history,
    })
}

fn build_behaviors(model: &Model) -> Vec<Box<dyn squid_n_element::behavior::ElementBehavior>> {
    let mut behaviors = Vec::new();
    for elem in &model.elements {
        // 時刻歴応答解析は公称値（材料強度割増なし）。
        let (mut b, _) = build_nonlinear_behavior(elem, model, StrengthBasis::Nominal);
        // 動的解析: コンクリート履歴は原点指向型（各履歴則の原典）。
        b.set_concrete_hysteresis(true);
        behaviors.push(b);
    }
    behaviors
}
