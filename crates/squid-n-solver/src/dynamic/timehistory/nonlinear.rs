//! 非線形時刻歴応答解析（Newmark-β + Newton 反復 + commit/rollback）。
//!
//! - [`NonlinearThCfg`] — 解析設定（Newton 収束条件・幾何剛性・長期荷重初期化・記録間引き）
//! - [`nonlinear_time_history_analysis`] — 非線形時刻歴応答解析

use super::common::{
    mass_accel_free_into, solve_initial_accel, sparse_matvec_into, theta_accel_at,
    theta_influence_m,
};
use super::config::{GroundMotion, NewmarkCfg};
use super::history::{
    choose_record_dir_y, pick_record_node, record_history_step, total_mass, update_story_drift,
};
use super::recording::{member_forces_nonlinear, ThRecorder};
use super::result::{ResponseHistory, ResponseResult};
use crate::assemble::assemble_global_m;
use crate::common::csc_cache::{CscCache, WeightedSumGuard};
use crate::constraint::Reducer;
use crate::damping::{Damping, DampingAccumulation};
use crate::pushover::{add_support_spring_f_int, assemble_k, assemble_k_cached, compute_f_int};
use crate::transaction::{StateSnapshot, StatefulModel};
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::model::Model;
use squid_n_element::behavior::{Ctx, ElementBehavior, LocalVec, MassOption};
use squid_n_element::factory::{build_nonlinear_behavior, StrengthBasis};
use squid_n_math::solver::{make_solver, LinearSolver, SolveError, SolverBackend};
use squid_n_math::sparse::sparse_matvec;

/// 非線形時刻歴応答解析の設定（Newton 収束条件・幾何剛性・長期荷重初期化・記録間引き）。
///
/// 引数が多くなるため、`use_kg`・`max_iter`・`tol` に加えて長期荷重初期化・記録間引きの
/// 設定を 1 つの構造体へまとめている（呼び出し元は本モジュールの `tests` のみのため、
/// 破壊的変更として導入した）。
#[derive(Clone, Copy, Debug)]
pub struct NonlinearThCfg {
    /// 各時刻ステップの Newton 反復の最大回数。
    pub max_iter: usize,
    /// Newton 収束判定の相対許容誤差（残差ノルム / 外力ノルムの最大値基準）。
    pub tol: f64,
    /// 幾何剛性（P-Δ、Kg）を接線剛性に含めるか。
    pub use_kg: bool,
    /// 時刻歴開始前に長期荷重（固定・積載等、`LoadCaseKind::is_long_term`）を
    /// 静的 Newton 反復で載荷し、その変位・応力状態を時刻歴の初期条件とするか
    /// （プッシュオーバーの長期載荷フェーズと同じ考え方）。長期系荷重ケースが
    /// 無いモデルでは何もしない。
    pub apply_long_term: bool,
    /// 詳細記録（[`super::ThRecording`]）の間引き係数。`None` は自動決定
    /// （記録フレーム数が概ね 1000 になるよう [`super::recording::auto_record_every`] で調整）。
    pub record_every: Option<usize>,
}

impl NonlinearThCfg {
    /// 既定値: 長期荷重初期化あり・幾何剛性なし・記録間引きは自動決定。
    /// `max_iter`・`tol` のみ呼び出し側で指定する。
    pub fn new(max_iter: usize, tol: f64) -> Self {
        Self {
            max_iter,
            tol,
            use_kg: false,
            apply_long_term: true,
            record_every: None,
        }
    }
}

impl Default for NonlinearThCfg {
    /// Newton 反復 20 回・相対許容誤差 1e-6・長期荷重初期化あり・幾何剛性なし・
    /// 記録間引きは自動決定。
    fn default() -> Self {
        Self::new(20, 1e-6)
    }
}

/// 非線形時刻歴応答解析（Newmark-β + Newton反復 + commit/rollback）。
///
/// 各時刻ステップで Newton 反復により内力（非線形復元力）と慣性力・減衰力・
/// 地震外力の動的釣合いを満たす解を求める。収束時は要素状態を commit、
/// 不収束時は Step 開始時の状態へ rollback する。
/// `cfg.apply_long_term` が真の場合、時刻歴開始前に長期荷重を静的載荷し
/// （プッシュオーバーの長期載荷フェーズと同じ経路）、その変位・応力状態を初期条件とする。
/// `initial_disp`/`initial_vel` は「長期解からの増分」としての動的初期条件で、
/// 記録される変位は長期変位を含む全変位である。
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
    cfg: NonlinearThCfg,
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
            recording: None,
            nonlinear: true,
            applied_long_term: cfg.apply_long_term,
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

    // 質量行列（縮約空間）。
    let m_free = assemble_global_m(model, dofmap, MassOption::Consistent);
    let m_red = reducer.reduce_k(&m_free);

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

    // ── 長期荷重ベクトル（apply_long_term） ─────────────────────────────
    // 長期系荷重ケース（固定・積載等、`LoadCaseKind::is_long_term`）の外力を、
    // プッシュオーバーの長期載荷フェーズ（driver.rs）と同じ経路で組み立てる。
    // `cfg.apply_long_term` が偽、または該当荷重ケースが無い場合はゼロベクトル。
    let f0_free: Vec<f64> = if cfg.apply_long_term {
        let mut f = vec![0.0; n_free];
        for lc in model.load_cases.iter().filter(|l| l.kind.is_long_term()) {
            let flc = crate::assemble::assemble_global_f(model, dofmap, lc.id);
            for (acc, v) in f.iter_mut().zip(flc) {
                *acc += v;
            }
        }
        f
    } else {
        vec![0.0; n_free]
    };
    let f0_red = reducer.reduce_f(&f0_free);
    let has_long_term = f0_red.iter().any(|&v| v.abs() > 0.0);

    // 長期荷重を静的 Newton 反復で載荷する（収束時は要素状態を commit）。
    // 時刻歴の初期変位 u0 はこの長期解（縮約空間）から始める。
    let mut u = vec![0.0; n_indep];
    if has_long_term {
        apply_long_term_static(
            model,
            dofmap,
            reducer,
            &mut behaviors,
            &f0_red,
            cfg.use_kg,
            &mut u,
        )?;
    }

    // 初期剛性の参照（長期荷重載荷後の状態、`use_kg` を反映）から減衰行列を組み立てる。
    // 従来は幾何剛性を持たない `assemble_global_k`（線形弾性 behavior）を用いており、
    // Newton 反復内の接線剛性（`pushover::assemble_k`、`use_kg` 反映）と不整合だった。
    let k_free = assemble_k(model, dofmap, &behaviors, cfg.use_kg);
    let k_red = reducer.reduce_k(&k_free);
    let c_red = damping.assemble_c(&m_red, &k_red);

    // 動的初期条件（initial_disp）は「長期解からの増分」として要素状態へ反映する。
    {
        let n_init_d = n_indep.min(initial_disp.len());
        let mut du_dyn = vec![0.0; n_indep];
        du_dyn[..n_init_d].copy_from_slice(&initial_disp[..n_init_d]);
        for i in 0..n_indep {
            u[i] += du_dyn[i];
        }
        let du_free = reducer.expand_u(&du_dyn);
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
        for b in behaviors.iter_mut() {
            b.commit_state();
        }
    }

    let mut v = vec![0.0; n_indep];
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

    // 初期加速度: M·a_0 = p(0) + f0 − C·v_0 − f_int(u_0)
    // p(0) は地震外力（符号込み −M·r·ẍg）、f0 は長期荷重（時刻歴を通じて一定、
    // プッシュオーバーの f0+λ·q と同じ扱い）。f_int(u_0) は長期荷重＋動的初期変位を
    // 反映した現在の要素状態から求まる（既に commit 済み）。
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

    // 内力（支点ばねの寄与を含む。u_trial は縮約前の全体変位、プッシュオーバーの
    // Newton 反復（`driver.rs`）と同じ経路）。
    let mut f_int0_free = compute_f_int(model, dofmap, &behaviors);
    {
        let u0_free = reducer.expand_u(&u);
        add_support_spring_f_int(model, dofmap, &u0_free, &mut f_int0_free);
    }
    let f_int0_red = reducer.reduce_f(&f_int0_free);
    let cv0 = sparse_matvec(&c_red, &v);
    let mut rhs_a0 = vec![0.0; n_indep];
    for i in 0..n_indep {
        rhs_a0[i] = p_red_0[i] + f0_red[i] - cv0[i] - f_int0_red[i];
    }
    let mut a = solve_initial_accel(&m_red, &rhs_a0, n_indep)?;

    // --- 時刻歴ループ ---
    let n_steps = wave.accel_x.len();
    // P9: u_free/v_free/a_free（自由 DOF 空間への展開）は 1 ステップに 1 回だけ
    // 展開し、以後（ピーク変位・層間変形角・record_history_step・
    // recorder.record_step）で使い回す（linear.rs/hht.rs と同じ方針。従来は
    // ここだけでも同じ `reducer.expand_u(&u)` を 2 回呼んでいた上、
    // `record_step` 内部でももう一度展開していた）。
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
    // 節点慣性力ベクトル算定用の M·a_free（自由 DOF 空間）。ベースシア・層せん断力の
    // 双方で共有する（1 ステップに 1 回だけ疎行列ベクトル積を計算する）。
    let mut ma_free = vec![0.0f64; n_free];
    mass_accel_free_into(&m_free, &a_free, &mut ma_free);
    {
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
            dir_idx,
            rmr_record,
            &u_free,
            &ma_free,
            xg_init,
        );
    }
    let mut time = Vec::with_capacity(n_steps + 1);
    time.push(0.0);

    // 詳細記録（3D アニメーション・層応答グラフ・部材履歴用、record_every は cfg 経由）。
    let mut recorder = ThRecorder::new(
        model,
        dofmap,
        n_steps,
        model.elements.len(),
        cfg.record_every,
    );
    {
        let xg_x_init = wave.accel_x.first().copied().unwrap_or(0.0);
        let xg_y_init = wave
            .accel_y
            .as_ref()
            .and_then(|a| a.first().copied())
            .unwrap_or(0.0);
        let mf_init = member_forces_nonlinear(model, &behaviors);
        recorder.record_step(
            0, 0.0, model, dofmap, &m_r_x, &m_r_y, &ma_free, &u_free, &v_free, &a_free, xg_x_init,
            xg_y_init, &mf_init,
        );
    }

    // P8/P9: Newton 反復内・ステップ末で毎回確保していた作業バッファを
    // ループ外で 1 回だけ確保し、以後は書き込みのみで再利用する。
    let mut p_free_buf = vec![0.0f64; n_free];
    let mut p_dyn_red_buf = vec![0.0f64; n_indep];
    let mut p_red_buf = vec![0.0f64; n_indep];
    let mut u_trial_red_buf = vec![0.0f64; n_indep];
    let mut u_trial_free_buf = vec![0.0f64; n_free];
    let mut f_int_red_buf = vec![0.0f64; n_indep];
    let mut du_free_buf = vec![0.0f64; n_free];
    let mut dv_buf = vec![0.0f64; n_indep];
    let mut c_dv_buf = vec![0.0f64; n_indep];
    let mut c_v_red_buf = vec![0.0f64; n_indep];
    let mut m_a_red_buf = vec![0.0f64; n_indep];
    let mut r_red_buf = vec![0.0f64; n_indep];
    let mut du_red_buf = vec![0.0f64; n_indep];

    // ── ソルバインスタンス・CSC 組立てキャッシュの持ち回り（時刻歴応答解析高速化・
    // 第2波） ──────────────────────────────────────────────────
    // Newton 反復・ステップループを跨いで同一インスタンスを保持する。K_eff は毎反復
    // 組み立て直すが、以下はいずれも「同一箇所から呼ばれる限り非ゼロパターンは
    // ほぼ不変」という前提が成り立つため、キャッシュ・symbolic 分解の再利用が効く:
    //
    // - `k_eff_solver`（`CholeskySolver`）: `factorize` を同一インスタンスへ繰り返し
    //   呼ぶと、直前と同じスパースパターンなら symbolic 分解（AMD順序付け）を
    //   再利用し数値分解のみ行う（`squid_n_math::cholesky::CholeskySolver` 参照）。
    //   K_eff は対称正定値を前提とする（旧 `SolverBackend::Auto` も本解析の
    //   自由度規模では常に疎 Cholesky 直接法を選ぶため、`DirectSparseCholesky` を
    //   明示しても既存挙動と同一。`Auto`（`AutoSolver`）は `factorize` のたびに
    //   内部ソルバを新規生成するため、これを持ち回っても symbolic キャッシュは
    //   効かない＝ここでは明示的に直接法ソルバを使う必要がある）。
    // - `k_t_free_cache`／`k_t_red_cache`（`CscCache`）: 接線剛性 K_t（全体・縮約後）
    //   の CSC 組立て。要素接続・拘束構成は不変なので、triplet の座標・並び順も
    //   （弾塑性要素の接線剛性が厳密 0.0 を跨がない限り）不変。
    // - `k_eff_cache`（`WeightedSumGuard`）: K_eff = K_t + c2·C + c1·M の重み付き和。
    //
    // 各キャッシュはパターン変化（弾塑性要素の完全塑性化等で非ゼロ数が変わる場合）を
    // 自動検知し、その回のみ安全側（パターンの作り直し）へフォールバックする
    // （[`crate::common::csc_cache`] 参照）。結果は常に非キャッシュ版とビット一致する。
    let mut k_eff_solver: Box<dyn LinearSolver> = make_solver(SolverBackend::DirectSparseCholesky);
    let mut k_t_free_cache = CscCache::new();
    let mut k_t_red_cache = CscCache::new();
    let mut k_eff_cache = WeightedSumGuard::new();

    for n in 0..n_steps {
        let t_next = (n + 1) as f64 * dt;

        // P3: 全要素の commit 済み状態のスナップショット（`StateSnapshot::capture`）は
        // 取らない。各ステップ開始時点では、直前ステップが収束していれば
        // trial==committed（収束時のみ commit_state を呼ぶため）であり、不収束時は
        // 本ループ自体が Err を返して打ち切るため、次ステップへは進まない。
        // したがって「あるステップ開始時に trial!=committed のまま次ステップへ
        // 入る」ことは起こらず、不収束時の rollback は
        // `model.revert_all(&mut behaviors)`（各要素の trial←committed）で
        // スナップショット捕捉・復元と厳密に等価になる。ファイバーモデルでは
        // `snapshot_state` が全要素・全ゲージ点の状態を Box 確保して複製するため
        // （毎ステップ数万 Box）、この等価性を利用して捕捉自体を省く。

        // 地震荷重（動的分）＋長期荷重（f0、時刻歴を通じて一定）。
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
        // 収束判定の分母に使う「動的外力のみ」のノルム基準（長期荷重 f0 を含まない）。
        // 長期荷重が卓越するモデルでは f0 を含めると分母が過大になり、動的外力に
        // 対する収束判定が実質的に緩んでしまう。
        reducer.reduce_f_into(&p_free_buf, &mut p_dyn_red_buf);
        let p_dyn_red = &p_dyn_red_buf;
        // P9: p_dyn_red の複製（.clone()）を避け、要素ごとに f0_red を加算した
        // 値を直接書き込む。
        for i in 0..n_indep {
            p_red_buf[i] = p_dyn_red[i] + f0_red[i];
        }
        let p_red = &p_red_buf;

        // 予測子: Δu = 0 での a, v
        let mut a_trial = vec![0.0; n_indep];
        let mut v_trial = vec![0.0; n_indep];
        for i in 0..n_indep {
            a_trial[i] = -c3 * v[i] - c4 * a[i];
            v_trial[i] = -c5 * v[i] - c6 * a[i];
        }

        let mut du_total = vec![0.0; n_indep];
        let mut converged = false;

        for _iter in 0..cfg.max_iter {
            // P1: 残差（f_int・C·v・M·a のみで計算可能、K は不要）を先に評価し、
            // 収束していれば接線剛性の組立・有効剛性の組立・分解（このループ内で
            // 最もコストが大きい）を一切行わずに break する。反復が収束するまで
            // K を必要としない点は変えず、単に「K を使う手前で判定する」よう
            // 計算順序を並べ替えただけで、残差自体の計算式・使用値は元のままの
            // ため数値結果は完全不変（決定性ガードテスト参照）。
            //
            // ただし接線比例減衰（α1 一定・h1 一定）は瞬間剛性 k_t_red から C を
            // 毎反復再構成するため、この場合に限り残差の減衰力項 c_v_red の計算に
            // k_t_red が要る。そのため接線減衰のときだけ、ここで k_t_red・c_tan を
            // 先に組み立てて使い回す（後段で二重に組み立てない）。
            let mut k_t_red_precomputed: Option<faer::sparse::SparseColMat<usize, f64>> = None;
            let c_tan = if damping.is_tangent_based() {
                let k_t_free =
                    assemble_k_cached(model, dofmap, &behaviors, cfg.use_kg, &mut k_t_free_cache);
                let k_t_red = reducer.reduce_k_cached(&k_t_free, &mut k_t_red_cache);
                // h1 一定の {u} は初期剛性の1次固有ベクトル（u_mode1、固定）。
                // α1 一定は u を参照しない。
                let c = damping.assemble_c_tangent(&m_red, &k_t_red, &k_red, &u_mode1);
                k_t_red_precomputed = Some(k_t_red);
                Some(c)
            } else {
                None
            };
            let c_cur = c_tan.as_ref().unwrap_or(&c_red);

            // 内力（支点ばねの寄与を含む。u_trial は縮約前の全体変位＝
            // 収束済み u ＋ステップ内累積修正量 du_total、プッシュオーバーの
            // Newton 反復（`driver.rs`）と同じ経路）。
            let mut f_int_free = compute_f_int(model, dofmap, &behaviors);
            {
                // P9: u_trial_red/u_trial_free をループ外バッファへ書き込む
                // （毎反復の Vec 確保を避ける）。
                for i in 0..n_indep {
                    u_trial_red_buf[i] = u[i] + du_total[i];
                }
                reducer.expand_u_into(&u_trial_red_buf, &mut u_trial_free_buf);
                add_support_spring_f_int(model, dofmap, &u_trial_free_buf, &mut f_int_free);
            }
            reducer.reduce_f_into(&f_int_free, &mut f_int_red_buf);
            let f_int_red = &f_int_red_buf;

            // 減衰力（縮約空間）。非累積型は瞬間 C×速度、累積型は増分減衰力の積分
            // （{Cn}={Cn−1}+[Cn]{Δẋn}、Δẋn=v_trial−v_前ステップ）。P9: いずれも
            // ループ外バッファへ書き込む（毎反復の Vec 確保を避ける）。
            match accumulation {
                DampingAccumulation::NonCumulative => {
                    sparse_matvec_into(c_cur, &v_trial, &mut c_v_red_buf)
                }
                DampingAccumulation::Cumulative => {
                    for i in 0..n_indep {
                        dv_buf[i] = v_trial[i] - v[i];
                    }
                    sparse_matvec_into(c_cur, &dv_buf, &mut c_dv_buf);
                    for i in 0..n_indep {
                        c_v_red_buf[i] = f_damp[i] + c_dv_buf[i];
                    }
                }
            };
            let c_v_red = &c_v_red_buf;
            c_v_last.clone_from(c_v_red);
            sparse_matvec_into(&m_red, &a_trial, &mut m_a_red_buf);
            let m_a_red = &m_a_red_buf;

            // 残差
            for i in 0..n_indep {
                r_red_buf[i] = p_red[i] - f_int_red[i] - c_v_red[i] - m_a_red[i];
            }
            let r_red = &r_red_buf;

            // 収束判定（分母は長期荷重 f0 を除いた動的外力ノルムと 1.0 の大きい方。
            // f0 を含めると長期荷重が大きいモデルで判定が過度に緩む）。
            let r_norm: f64 = r_red.iter().map(|x| x * x).sum::<f64>().sqrt();
            let p_norm: f64 = p_dyn_red.iter().map(|x| x * x).sum::<f64>().sqrt();
            if r_norm < cfg.tol * p_norm.max(1.0) {
                converged = true;
                break;
            }

            // 未収束のときのみ、接線剛性（未組立なら今組み立てる。接線減衰で既に
            // 組み立て済みなら再利用）・有効剛性・分解を行い、δu を解く。
            let k_t_red = match k_t_red_precomputed {
                Some(k) => k,
                None => {
                    let k_t_free = assemble_k_cached(
                        model,
                        dofmap,
                        &behaviors,
                        cfg.use_kg,
                        &mut k_t_free_cache,
                    );
                    reducer.reduce_k_cached(&k_t_free, &mut k_t_red_cache)
                }
            };
            let k_eff = k_eff_cache.combine(n_indep, &[(1.0, &k_t_red), (c2, c_cur), (c1, &m_red)]);
            k_eff_solver
                .factorize(&k_eff)
                .map_err(|e| SolveError::Backend(format!("factor: {:?}", e)))?;
            k_eff_solver.solve_into(r_red, &mut du_red_buf)?;
            let du_red = &du_red_buf;
            reducer.expand_u_into(du_red, &mut du_free_buf);
            let du_free = &du_free_buf;

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

            // P9: u_free/v_free/a_free をループ外バッファへ書き込んで使い回す
            // （record_history_step・recorder.record_step と共有）。
            reducer.expand_u_into(&u, &mut u_free);
            for i in 0..n_free {
                peak_disp_free[i] = peak_disp_free[i].max(u_free[i].abs());
            }
            update_story_drift(model, dofmap, &u_free, &mut story_drift_angle);
            reducer.expand_u_into(&v, &mut v_free);
            reducer.expand_u_into(&a, &mut a_free);
            // 節点慣性力ベクトル算定用の M·a_free（自由 DOF 空間）。ベースシア・
            // 層せん断力の双方で共有する（1 ステップに 1 回だけ算定）。
            mass_accel_free_into(&m_free, &a_free, &mut ma_free);
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
            let mf_now = member_forces_nonlinear(model, &behaviors);
            recorder.record_step(
                (n + 1) as u64,
                t_next,
                model,
                dofmap,
                &m_r_x,
                &m_r_y,
                &ma_free,
                &u_free,
                &v_free,
                &a_free,
                xg_x_next,
                xg_y_next,
                &mf_now,
            );
        } else {
            // 不収束: rollback（P3: ステップ開始時点は trial==committed のため、
            // 全要素 revert_state（trial←committed）はスナップショット復元と等価。
            // 上のコメント参照）。
            model.revert_all(&mut behaviors);
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
        recording: Some(recorder.finish()),
        nonlinear: true,
        applied_long_term: cfg.apply_long_term,
    })
}

/// 長期漸増載荷の載荷率（0〜1）を追跡する状態機械。
///
/// 基準増分（`1/n_grav`）ごとに漸増し、収束失敗時は増分半減で再試行する
/// （最大 `max_attempts` 回、超えたら [`Self::record_failure`] が `false` を返す）。
/// **半減リトライで成功した場合も、次回は基準増分から再開し、載荷率が 100% に
/// 達するまで残増分を継続する**（while ループ、[`Self::next_target`] が `None` を
/// 返すまで）。呼び出し側（`apply_long_term_static`）は FEM の Newton 収束判定
/// （snapshot/restore/commit）を挟むため、この構造体自体は FEM に依存せず、
/// 載荷率の遷移ロジックのみを単体テストできるよう切り出している。
#[derive(Debug, Clone, Copy)]
struct LoadFractionState {
    /// 収束が確定した載荷率。
    applied: f64,
    /// 失敗時に半減する前の基準増分（`1/n_grav`）。成功のたびにこれへ戻す。
    base_increment: f64,
    /// 次回試行する増分（半減リトライ中は `base_increment` より小さい）。
    increment: f64,
    /// 現在の載荷率に対する連続失敗回数。
    attempts: usize,
    /// 連続失敗の許容回数（これを超えたら `record_failure` が `false` を返す）。
    max_attempts: usize,
}

impl LoadFractionState {
    fn new(n_grav: usize) -> Self {
        let base_increment = 1.0 / n_grav.max(1) as f64;
        Self {
            applied: 0.0,
            base_increment,
            increment: base_increment,
            attempts: 0,
            max_attempts: 5,
        }
    }

    /// 次に試行すべき載荷率。100% に達していれば `None`。
    fn next_target(&self) -> Option<f64> {
        if self.applied >= 1.0 - 1e-9 {
            None
        } else {
            Some((self.applied + self.increment).min(1.0))
        }
    }

    /// 直前の [`Self::next_target`] の載荷率が収束した。載荷率を確定し、
    /// 次回の増分を基準増分へ戻す（半減リトライ後も残増分を基準ペースで継続する）。
    fn record_success(&mut self, mu_target: f64) {
        self.applied = mu_target;
        self.increment = self.base_increment;
        self.attempts = 0;
    }

    /// 直前の載荷率が収束しなかった。増分を半減して再試行する。
    /// 連続失敗が `max_attempts` に達したら `false`（呼び出し側は Err とする）。
    fn record_failure(&mut self) -> bool {
        self.attempts += 1;
        if self.attempts >= self.max_attempts {
            return false;
        }
        self.increment *= 0.5;
        true
    }
}

/// 長期荷重（`f0_red`、縮約空間）を静的 Newton 反復で載荷する
/// （プッシュオーバーの長期載荷フェーズ、`driver.rs` と同じ考え方: 弾性域で収まらない
/// 場合に備えて基準 5 分割で漸増し、収束失敗時は増分半減で再試行する。半減リトライで
/// 成功した場合も、載荷率が 100% に達するまで残増分を基準ペースで継続する。
/// [`LoadFractionState`] 参照）。収束したステップごとに要素状態を commit し、
/// 載荷完了時の変位（縮約空間）を `u_out` に加算する。
fn apply_long_term_static(
    model: &mut Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    behaviors: &mut [Box<dyn ElementBehavior>],
    f0_red: &[f64],
    use_kg: bool,
    u_out: &mut [f64],
) -> Result<(), SolveError> {
    let n_indep = reducer.n_indep;
    let mut state = LoadFractionState::new(5);
    // 長期荷重の漸増ステップ全体（複数の載荷率試行・各試行内の Newton 反復）で
    // ソルバインスタンス・CSC 組立てキャッシュを保持する（時刻歴応答解析高速化・
    // 第2波、`nonlinear_time_history_analysis` 本体の Newton ループと同じ方針。
    // 理由は呼び出し先 [`newton_static_converge`] のコメント参照）。
    let mut solver: Box<dyn LinearSolver> = make_solver(SolverBackend::DirectSparseCholesky);
    let mut k_free_cache = CscCache::new();
    let mut k_red_cache = CscCache::new();
    let mut du_red_buf = vec![0.0f64; n_indep];
    while let Some(mu_target) = state.next_target() {
        let snap = StateSnapshot::capture(behaviors);
        let f_target: Vec<f64> = f0_red.iter().map(|&v| v * mu_target).collect();
        match newton_static_converge(
            model,
            dofmap,
            reducer,
            behaviors,
            &f_target,
            use_kg,
            n_indep,
            &*u_out,
            &mut solver,
            &mut k_free_cache,
            &mut k_red_cache,
            &mut du_red_buf,
        )? {
            Some(du) => {
                for b in behaviors.iter_mut() {
                    b.commit_state();
                }
                for i in 0..n_indep {
                    u_out[i] += du[i];
                }
                state.record_success(mu_target);
            }
            None => {
                model.restore(&snap, behaviors);
                if !state.record_failure() {
                    return Err(SolveError::InvalidInput(
                        "長期荷重の初期載荷が収束しません（長期荷重に対して構造が不安定な可能性）"
                            .into(),
                    ));
                }
            }
        }
    }
    Ok(())
}

/// 固定外力 `f_target_red`（縮約空間）に対する静的 Newton 反復
/// （長期荷重の各漸増ステップの共通経路）。収束時はステップ内の全修正量の累積
/// （縮約空間の変位増分）を `Some` で返し、要素状態はトライアル反映済み・未確定の
/// まま戻す（確定・巻き戻しは呼び出し側の責務）。収束しなければ `Ok(None)`。
/// `u_base_red` はこの呼び出し開始時点の全体変位（縮約空間、これまでの漸増ステップの
/// 確定累積）。支点ばね内力 `add_support_spring_f_int` に渡す縮約前の全体変位
/// `u_base_red + du_total` の算定に使う。
///
/// `solver`・`k_free_cache`・`k_red_cache`・`du_red_buf` は呼び出し元
/// [`apply_long_term_static`] が漸増ステップ全体を通じて保持するソルバインスタンス・
/// CSC 組立てキャッシュ・作業バッファ（時刻歴応答解析高速化・第2波）。K は対称正定値を
/// 前提とする（旧 `SolverBackend::Auto` も本解析の自由度規模では常に疎 Cholesky
/// 直接法を選ぶため、`DirectSparseCholesky` を明示しても既存挙動と同一）。
#[allow(clippy::too_many_arguments)]
fn newton_static_converge(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    behaviors: &mut [Box<dyn ElementBehavior>],
    f_target_red: &[f64],
    use_kg: bool,
    n_indep: usize,
    u_base_red: &[f64],
    solver: &mut Box<dyn LinearSolver>,
    k_free_cache: &mut CscCache,
    k_red_cache: &mut CscCache,
    du_red_buf: &mut Vec<f64>,
) -> Result<Option<Vec<f64>>, SolveError> {
    let mut du_total = vec![0.0; n_indep];
    for _iter in 0..50 {
        let k_free = assemble_k_cached(model, dofmap, behaviors, use_kg, k_free_cache);
        let k_red = reducer.reduce_k_cached(&k_free, k_red_cache);
        // 内力（支点ばねの寄与を含む。u_trial は縮約前の全体変位、プッシュオーバーの
        // 長期載荷フェーズ（`driver.rs`）と同じ経路）。
        let mut f_int_free = compute_f_int(model, dofmap, behaviors);
        {
            let u_trial_red: Vec<f64> = (0..n_indep).map(|i| u_base_red[i] + du_total[i]).collect();
            let u_trial_free = reducer.expand_u(&u_trial_red);
            add_support_spring_f_int(model, dofmap, &u_trial_free, &mut f_int_free);
        }
        let f_int_red = reducer.reduce_f(&f_int_free);
        let mut r_red = vec![0.0; n_indep];
        for i in 0..n_indep {
            r_red[i] = f_target_red[i] - f_int_red[i];
        }
        let r_norm: f64 = r_red.iter().map(|x| x * x).sum::<f64>().sqrt();
        let f_norm: f64 = f_target_red.iter().map(|x| x * x).sum::<f64>().sqrt();
        if r_norm < 1e-6 * f_norm.max(1.0) {
            return Ok(Some(du_total));
        }
        solver
            .factorize(&k_red)
            .map_err(|e| SolveError::Backend(format!("factor: {:?}", e)))?;
        solver.solve_into(&r_red, du_red_buf)?;
        let du_free = reducer.expand_u(du_red_buf.as_slice());
        for i in 0..n_indep {
            du_total[i] += du_red_buf[i];
        }
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
    Ok(None)
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

#[cfg(test)]
mod load_fraction_tests {
    use super::LoadFractionState;

    /// 全ステップ成功する場合、基準増分（1/5=0.2）刻みで 5 回で 100% へ到達する。
    #[test]
    fn all_success_reaches_full_load_in_five_steps() {
        let mut state = LoadFractionState::new(5);
        let mut targets = Vec::new();
        while let Some(mu) = state.next_target() {
            targets.push(mu);
            state.record_success(mu);
        }
        assert_eq!(targets.len(), 5);
        assert!((targets.last().copied().unwrap() - 1.0).abs() < 1e-12);
        assert!((state.applied - 1.0).abs() < 1e-12);
    }

    /// 途中で失敗→増分半減で成功するパスでも、最終的に載荷率は 100% に到達する
    /// （半減成功後に打ち切らず残増分を継続する、高-1 の回帰防止）。
    #[test]
    fn half_step_success_still_reaches_full_load() {
        let mut state = LoadFractionState::new(5);
        let mut n_targets = 0usize;
        while let Some(mu) = state.next_target() {
            n_targets += 1;
            assert!(n_targets < 1000, "無限ループの疑い");
            // 最初の載荷率（1回目の目標=0.2）だけ 1 回失敗させ、以降は毎回成功させる。
            if n_targets == 1 {
                assert!(state.record_failure());
                continue;
            }
            state.record_success(mu);
        }
        assert!((state.applied - 1.0).abs() < 1e-9);
        // 100% に到達するまで、半減で増えた分の目標を含め複数回試行しているはず。
        assert!(n_targets > 5);
    }

    /// 最大試行回数を超えて失敗し続けると `record_failure` が false を返す
    /// （呼び出し側はこれを Err に変換する）。
    #[test]
    fn repeated_failure_exceeds_max_attempts() {
        let mut state = LoadFractionState::new(5);
        let mut ok = true;
        for _ in 0..10 {
            if state.next_target().is_none() {
                break;
            }
            ok = state.record_failure();
            if !ok {
                break;
            }
        }
        assert!(!ok);
    }
}
