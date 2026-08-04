//! 線形時刻歴応答解析（Newmark-β 法、基盤一様加振）。
//!
//! - [`linear_time_history_analysis`] — 線形時刻歴応答解析
//! - [`linear_time_history_with_state`] — 最終状態付き（チェックポイント保存用）
//! - [`linear_time_history_from_state`] — チェックポイントからの再開

use super::common::{
    horizontal_influence_m, mass_accel_free_into, solve_initial_accel, sparse_matvec_into,
    theta_accel_at, theta_influence_m,
};
use super::config::{GroundMotion, NewmarkCfg};
use super::history::{
    choose_record_dir_y, pick_record_node, record_history_step, total_mass, update_story_drift,
};
use super::recording::{member_forces_linear, ThRecorder};
use super::result::{ResponseHistory, ResponseResult, TimeStepState};
use crate::assemble::{assemble_global_k, assemble_global_m};
use crate::constraint::Reducer;
use crate::damping::Damping;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
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
        return Ok((
            ResponseResult {
                time: vec![],
                peak_disp: vec![[0.0; 6]; model.nodes.len()],
                story_drift_angle: vec![0.0; model.stories.len()],
                cumulative_ductility: vec![0.0; model.elements.len()],
                history: ResponseHistory::default(),
                recording: None,
                nonlinear: false,
                applied_long_term: false,
            },
            TimeStepState {
                step: 0,
                time: 0.0,
                disp_red: vec![],
                vel_red: vec![],
                accel_red: vec![],
            },
        ));
    }

    // 部材内力記録（`ThRecording::member_forces`）用: 要素の弾性 behavior は
    // 時刻歴を通じて不変（線形解析）のため、ループ前に 1 回だけ構築して共有する。
    let behaviors: Vec<Box<dyn ElementBehavior>> = model
        .elements
        .iter()
        .map(|e| build_behavior(e, model))
        .collect();

    // --- 行列組立（縮約空間） ---
    let m_free = assemble_global_m(model, dofmap, MassOption::Consistent);
    let k_free = assemble_global_k(model, dofmap);
    // 幾何剛性（P-Δ）は線形時刻歴では未実装。かつては `let _ = use_kg;` で
    // 無言に捨てており、P-Δ を有効化したつもりの呼び出しでも考慮されないまま
    // 解析が通っていた。未対応である事実を明示エラーで返す。
    if use_kg {
        return Err(SolveError::InvalidInput(
            "線形時刻歴応答解析の幾何剛性（P-Δ、use_kg）は未対応です。\
             P-Δ を考慮する場合は非線形時刻歴応答解析を使用してください。"
                .into(),
        ));
    }
    let m_red = reducer.reduce_k(&m_free);
    let k_red = reducer.reduce_k(&k_free);
    let c_red = damping.assemble_c(&m_red, &k_red);

    // --- 影響ベクトルと M·r の事前計算 ---
    let (m_r_x, m_r_y) = horizontal_influence_m(model, dofmap, &m_free);
    // 位相差入力（ねじれ加振）用の回転影響 M·r_θ。
    let m_r_theta = theta_influence_m(model, dofmap, &m_free);

    // --- Newmark-β 係数 ---
    let beta = newmark.beta;
    let gamma = newmark.gamma;
    let c1 = 1.0 / (beta * dt * dt);
    let c2 = gamma / (beta * dt);
    let c3 = 1.0 / (beta * dt);
    let c4 = 1.0 / (2.0 * beta) - 1.0;
    let c5 = gamma / beta - 1.0;
    let c6 = dt * (gamma / (2.0 * beta) - 1.0);

    // --- 有効剛性 K^ = K + c2·C + c1・M ---
    let k_eff = squid_n_math::sparse::weighted_sum_csc(
        n_indep,
        &[(1.0, &k_red), (c2, &c_red), (c1, &m_red)],
    );

    // 有効剛性は全ステップ共通で、1回の分解を全時刻ステップの求解で再利用する。
    // 反復法（PCG）はステップごとに反復をやり直すため不利であり、直接法を明示する。
    let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
    solver.factorize(&k_eff)?;

    // --- 初期条件 ---
    let mut u = vec![0.0; n_indep];
    let mut v = vec![0.0; n_indep];
    let n_init_d = n_indep.min(initial_disp.len());
    u[..n_init_d].copy_from_slice(&initial_disp[..n_init_d]);
    let n_init_v = n_indep.min(initial_vel.len());
    v[..n_init_v].copy_from_slice(&initial_vel[..n_init_v]);

    // 初期加速度: M·a_0 = p(0) − C·v_0 − K·u_0（p(0) = −M·r·ẍg(0) は符号込みで構築済み）
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

    let cv0 = sparse_matvec(&c_red, &v);
    let ku0 = sparse_matvec(&k_red, &u);
    let mut rhs_a0 = vec![0.0; n_indep];
    for i in 0..n_indep {
        // p(0) は −M·r·ẍg として符号込みで構築済みのため、ここでは加算する
        // （従来は誤って減算しており、外力項の符号が逆＝初期加速度が
        // +r·ẍg(0) 側に立ち上がっていた。ẍg(0)=0 の波形では影響なし）。
        rhs_a0[i] = p_red_0[i] - cv0[i] - ku0[i];
    }
    let a = solve_initial_accel(&m_red, &rhs_a0, n_indep)?;

    // --- 時刻歴ループ（start_step=0 から） ---
    run_steps(
        model,
        dofmap,
        reducer,
        wave,
        dt,
        0,
        &m_r_x,
        &m_r_y,
        &m_r_theta,
        &m_free,
        &m_red,
        &c_red,
        &mut solver,
        &behaviors,
        c1,
        c2,
        c3,
        c4,
        c5,
        c6,
        gamma,
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
    if n_indep == 0 || state.disp_red.len() != n_indep {
        return Err(SolveError::Backend(
            "time history restart: state dimension mismatch".into(),
        ));
    }

    // 部材内力記録用の弾性 behavior（線形解析なので時刻歴を通じて不変）。
    let behaviors: Vec<Box<dyn ElementBehavior>> = model
        .elements
        .iter()
        .map(|e| build_behavior(e, model))
        .collect();

    // 行列・係数の再計算（線形なので同一）
    let m_free = assemble_global_m(model, dofmap, MassOption::Consistent);
    let k_free = assemble_global_k(model, dofmap);
    // 幾何剛性（P-Δ）は線形時刻歴では未実装。かつては `let _ = use_kg;` で
    // 無言に捨てており、P-Δ を有効化したつもりの呼び出しでも考慮されないまま
    // 解析が通っていた。未対応である事実を明示エラーで返す。
    if use_kg {
        return Err(SolveError::InvalidInput(
            "線形時刻歴応答解析の幾何剛性（P-Δ、use_kg）は未対応です。\
             P-Δ を考慮する場合は非線形時刻歴応答解析を使用してください。"
                .into(),
        ));
    }
    let m_red = reducer.reduce_k(&m_free);
    let k_red = reducer.reduce_k(&k_free);
    let c_red = damping.assemble_c(&m_red, &k_red);

    let (m_r_x, m_r_y) = horizontal_influence_m(model, dofmap, &m_free);
    // 位相差入力（ねじれ加振）用の回転影響 M·r_θ。
    let m_r_theta = theta_influence_m(model, dofmap, &m_free);

    let beta = newmark.beta;
    let gamma = newmark.gamma;
    let c1 = 1.0 / (beta * dt * dt);
    let c2 = gamma / (beta * dt);
    let c3 = 1.0 / (beta * dt);
    let c4 = 1.0 / (2.0 * beta) - 1.0;
    let c5 = gamma / beta - 1.0;
    let c6 = dt * (gamma / (2.0 * beta) - 1.0);

    let k_eff = squid_n_math::sparse::weighted_sum_csc(
        n_indep,
        &[(1.0, &k_red), (c2, &c_red), (c1, &m_red)],
    );
    let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
    solver.factorize(&k_eff)?;

    // チェックポイントから状態を復元
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
        &m_r_x,
        &m_r_y,
        &m_r_theta,
        &m_free,
        &m_red,
        &c_red,
        &mut solver,
        &behaviors,
        c1,
        c2,
        c3,
        c4,
        c5,
        c6,
        gamma,
        u,
        v,
        a,
        record_every,
    )
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
    m_r_x: &[f64],
    m_r_y: &[f64],
    m_r_theta: &[f64],
    m_free: &faer::sparse::SparseColMat<usize, f64>,
    m_red: &faer::sparse::SparseColMat<usize, f64>,
    c_red: &faer::sparse::SparseColMat<usize, f64>,
    solver: &mut Box<dyn squid_n_math::solver::LinearSolver>,
    behaviors: &[Box<dyn ElementBehavior>],
    c1: f64,
    c2: f64,
    c3: f64,
    c4: f64,
    c5: f64,
    c6: f64,
    gamma: f64,
    mut u: Vec<f64>,
    mut v: Vec<f64>,
    mut a: Vec<f64>,
    record_every: Option<usize>,
) -> Result<(ResponseResult, TimeStepState), SolveError> {
    let n_indep = reducer.n_indep;
    let n_free = dofmap.n_active();

    // P9: u_free/v_free/a_free（自由 DOF 空間への展開）は 1 ステップに 1 回だけ
    // 展開し、以後（ピーク変位・層間変形角・部材内力復元・record_history_step・
    // recorder.record_step）で使い回す（従来は `record_step` 内部でも同じ展開を
    // やり直しており、u_free が 1 ステップに 2 回展開されていた）。
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
    let mut story_drift_angle = vec![0.0f64; model.stories.len()];
    update_story_drift(model, dofmap, &u_free, &mut story_drift_angle);

    let mut time = Vec::with_capacity(wave.accel_x.len() - start_step as usize + 1);
    time.push(start_step as f64 * dt);

    // 節点慣性力ベクトル算定用の M·a_free（自由 DOF 空間）。ベースシア・層せん断力の
    // 双方で共有する（1 ステップに 1 回だけ疎行列ベクトル積を計算する）。
    let mut ma_free = vec![0.0f64; n_free];
    mass_accel_free_into(m_free, &a_free, &mut ma_free);

    // 詳細記録（3D アニメーション・層応答グラフ・部材履歴用。record_every は
    // 呼び出し元（UI 等）が指定できる。None は自動決定）。
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

    // UI 用の代表応答記録（記録方向は入力加速度の絶対値和が大きい方を自動選択）
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

    // P8/P9: ループ内で毎ステップ確保していた作業バッファをループ外で 1 回だけ
    // 確保し、以後は書き込みのみで再利用する（p_free・p_red・mw・cw・m_mw・
    // c_cw・p_eff）。`u_next_buf` は時刻歴応答解析高速化・第2波で追加された
    // `LinearSolver::solve_into`（squid-n-math）を使い、`solver.solve` の
    // 戻り値確保（P9 当時は避けられなかった）も無くしている。
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
        let xg_x = wave.accel_x[n];
        let xg_y = wave
            .accel_y
            .as_ref()
            .map(|a| a.get(n).copied().unwrap_or(0.0))
            .unwrap_or(0.0);

        let xg_theta = theta_accel_at(wave, n);
        for i in 0..n_free {
            p_free_buf[i] = -(m_r_x[i] * xg_x + m_r_y[i] * xg_y + m_r_theta[i] * xg_theta);
        }
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

        // a・v・u を単一パスでその場更新する（P9: 従来は a_next・v_next を別の
        // Vec に確保する 2 パスだったのを統合。各 i の計算はいずれも他の i に
        // 依存しないため、1 パスへ統合しても各成分の演算順序・使用値は元の
        // 2 パス版と完全に同じで、結果はビット完全一致する）。
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
        // 節点慣性力ベクトル算定用の M·a_free（自由 DOF 空間）。ベースシア・
        // 層せん断力の双方で共有する（1 ステップに 1 回だけ算定）。
        reducer.expand_u_into(&v, &mut v_free);
        reducer.expand_u_into(&a, &mut a_free);
        mass_accel_free_into(m_free, &a_free, &mut ma_free);
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
            dir_idx,
            rmr_record,
            &u_free,
            &ma_free,
            xg_next,
        );

        let xg_x_next = wave.accel_x.get(n + 1).copied().unwrap_or(0.0);
        let xg_y_next = wave
            .accel_y
            .as_ref()
            .and_then(|acc| acc.get(n + 1).copied())
            .unwrap_or(0.0);
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

    let mut peak_disp = vec![[0.0f64; 6]; model.nodes.len()];
    for ni in 0..model.nodes.len() {
        for d in 0..DOF_PER_NODE {
            let g = ni * DOF_PER_NODE + d;
            if let Some(a) = dofmap.active(g) {
                peak_disp[ni][d] = peak_disp_free[a as usize];
            }
        }
    }

    Ok((
        ResponseResult {
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
