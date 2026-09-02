//! 積分スキーム間で共有する下位ルーチン。
//!
//! - [`horizontal_influence_m`] — 水平 2 方向の地動入力用 `(M·r_x, M·r_y)`
//! - [`theta_influence_m`] — 位相差入力（ねじれ加振）の回転影響ベクトル `M·r_θ`
//! - [`theta_accel_at`] — ステップ `n` のねじれ地動加速度取得
//! - [`solve_initial_accel`] — 初期加速度 `M·a₀ = rhs` の求解
//! - [`mass_accel_free_into`] — 節点慣性力ベクトル算定用の `M·a_free`
//!   （自由 DOF 空間）
//! - [`sparse_matvec_into`] — `squid_n_math::sparse::sparse_matvec_into` の再エクスポート
//!   （時刻歴応答解析高速化・第1波で暫定実装したローカル版を、squid-n-math に同等
//!   API が追加された第2波で置き換えた）
//! - [`resolve_dt`] — 時間刻みの解決（設定値と波形の刻みの優先順）
//! - [`NewmarkCoeffs`] — Newmark-β の積分係数 c1〜c6
//! - [`GroundInfluence`] — 影響ベクトル束 `M·r` と等価地震力 `p = −M·r·ẍg`
//! - [`empty_response`] — 独立自由度が無い退化ケースの応答
//! - [`reduced_vec_from`] — 縮約空間の初期値ベクトル生成

use super::config::{GroundMotion, NewmarkCfg};
use super::result::{ResponseHistory, ResponseResult};
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::model::Model;
use squid_n_math::solver::{make_solver, SolveError, SolverBackend};
use squid_n_math::sparse::sparse_matvec;

/// [`squid_n_math::sparse::sparse_matvec_into`] の再エクスポート。時刻歴応答解析
/// 各所（`linear.rs`・`nonlinear.rs`）は本モジュール経由でこの名前を使う
/// （第1波はここにローカル実装を置いていたが、第2波で squid-n-math 側の実装へ寄せた。
/// 呼び出し側の変更は不要）。
pub(crate) use squid_n_math::sparse::sparse_matvec_into;

/// 水平 2 方向（X・Y）の地動入力用影響ベクトル × 質量 `(M·r_x, M·r_y)` を構築する。
///
/// 影響ベクトル r は当該方向の並進自由度に 1 を立てた単位剛体並進で、
/// `M·r` が各ステップの等価地震力 `−M·r·üg` の係数になる。積分スキーム
/// （Newmark 線形・非線形）で同一のため単一実装とする。
fn horizontal_influence_m(
    model: &Model,
    dofmap: &DofMap,
    m_free: &faer::sparse::SparseColMat<usize, f64>,
) -> (Vec<f64>, Vec<f64>) {
    let n_free = dofmap.n_active();
    let mut r_x_free = vec![0.0; n_free];
    let mut r_y_free = vec![0.0; n_free];
    for ni in 0..model.nodes.len() {
        let g_ux = ni * DOF_PER_NODE;
        let g_uy = ni * DOF_PER_NODE + 1;
        if let Some(a) = dofmap.active(g_ux) {
            r_x_free[a as usize] = 1.0;
        }
        if let Some(a) = dofmap.active(g_uy) {
            r_y_free[a as usize] = 1.0;
        }
    }
    (
        sparse_matvec(m_free, &r_x_free),
        sparse_matvec(m_free, &r_y_free),
    )
}

/// 位相差入力（ねじれ加振）用の回転影響ベクトル × 質量 `M·r_θ` を構築する
/// （多点位相差入力、構造力学）。鉛直（Z）軸まわりの単位角加速度に対し、各節点は
/// 剛体回転 `ax=−(y−yc)`, `ay=(x−xc)` の並進と、回転自由度 rz=1 の影響を受ける
/// （`(xc,yc)`＝節点幾何重心）。返り値は自由 DOF 空間の `M·r_θ`。
fn theta_influence_m(
    model: &Model,
    dofmap: &DofMap,
    m_free: &faer::sparse::SparseColMat<usize, f64>,
) -> Vec<f64> {
    let n_free = dofmap.n_active();
    // 節点幾何重心。
    let (mut cx, mut cy, mut cnt) = (0.0, 0.0, 0.0f64);
    for node in &model.nodes {
        cx += node.coord[0];
        cy += node.coord[1];
        cnt += 1.0;
    }
    if cnt > 0.0 {
        cx /= cnt;
        cy /= cnt;
    }
    let mut r_theta = vec![0.0; n_free];
    for (ni, node) in model.nodes.iter().enumerate() {
        let g_ux = ni * DOF_PER_NODE;
        let g_uy = ni * DOF_PER_NODE + 1;
        let g_rz = ni * DOF_PER_NODE + 5;
        if let Some(a) = dofmap.active(g_ux) {
            r_theta[a as usize] = -(node.coord[1] - cy);
        }
        if let Some(a) = dofmap.active(g_uy) {
            r_theta[a as usize] = node.coord[0] - cx;
        }
        if let Some(a) = dofmap.active(g_rz) {
            r_theta[a as usize] = 1.0;
        }
    }
    sparse_matvec(m_free, &r_theta)
}

/// 位相差入力のねじれ地動加速度をステップ `n` で取得（未指定は 0）。
fn theta_accel_at(wave: &GroundMotion, n: usize) -> f64 {
    wave.accel_theta
        .as_ref()
        .and_then(|a| a.get(n).copied())
        .unwrap_or(0.0)
}

/// 初期加速度 M·a₀ = rhs を解く。
/// 質量行列は回転自由度などに質量ゼロ行を含み特異になり得るため、
/// Cholesky → LU の順に試し、いずれも失敗した場合は
/// rhs≈0（静止開始）なら a₀ = 0 とみなす。
pub(crate) fn solve_initial_accel(
    m_red: &faer::sparse::SparseColMat<usize, f64>,
    rhs: &[f64],
    n_indep: usize,
) -> Result<Vec<f64>, SolveError> {
    let mut chol = make_solver(SolverBackend::DirectSparseCholesky);
    if chol.factorize(m_red).is_ok() {
        return chol.solve(rhs);
    }
    let mut lu = make_solver(SolverBackend::DirectSparseLu);
    if lu.factorize(m_red).is_ok() {
        if let Ok(a) = lu.solve(rhs) {
            if a.iter().all(|v| v.is_finite()) {
                return Ok(a);
            }
        }
    }
    let rhs_norm: f64 = rhs.iter().map(|v| v * v).sum::<f64>().sqrt();
    if rhs_norm < 1e-9 {
        // 静止開始（初期外力ゼロ）なら初期加速度もゼロ
        return Ok(vec![0.0; n_indep]);
    }
    Err(SolveError::InvalidInput(
        "質量行列が特異で初期加速度を計算できません。地震波の先頭を 0 から始めるか、全自由度に質量を与えてください。".into(),
    ))
}

/// 節点慣性力ベクトルの算定に使う `M·a_free`（自由 DOF 空間、`dofmap` の
/// アクティブ添字順）を、呼び出し側の既存バッファへ書き込む（毎ステップの
/// Vec 確保を避ける、P9）。層せん断力・ベースシアの算定（[`super::recording`]・
/// [`super::history::record_history_step`]）で共有するため、各積分ループで
/// 1 ステップに 1 回だけ呼び出す。
///
/// `a_free` は呼び出し側で展開済みの自由 DOF 空間の加速度（`Reducer::expand_u`/
/// [`Reducer::expand_u_into`](crate::constraint::Reducer::expand_u_into)）を渡す
/// （`ThRecorder::record_step` 等でも同じ展開済み `a_free` を使い回すため、
/// 展開そのものは呼び出し側で 1 ステップに 1 回だけ行う）。
pub(crate) fn mass_accel_free_into(
    m_free: &faer::sparse::SparseColMat<usize, f64>,
    a_free: &[f64],
    out: &mut [f64],
) {
    sparse_matvec_into(m_free, a_free, out);
}

/// 時刻歴の時間刻み [s] を決める。設定値が正ならそれを、さもなくば波形の刻みを使う。
///
/// 各積分器（`linear`・`nonlinear`）が同じ規約・同じエラー文言を持つため、
/// 解決規則をここに一本化する。
pub(crate) fn resolve_dt(cfg_dt: f64, wave: &GroundMotion) -> Result<f64, SolveError> {
    let dt = if cfg_dt > 0.0 { cfg_dt } else { wave.dt };
    if dt <= 0.0 {
        return Err(SolveError::Backend(
            "time history: dt must be positive".into(),
        ));
    }
    Ok(dt)
}

/// Newmark-β の積分係数（構造動力学の標準形）。
///
/// 有効剛性・有効荷重の組立と、ステップ確定後の速度・加速度更新に使う。
/// 線形・非線形の双方が同一の式を用いるため、算定をここへ集約する。
pub(crate) struct NewmarkCoeffs {
    pub gamma: f64,
    pub c1: f64,
    pub c2: f64,
    pub c3: f64,
    pub c4: f64,
    pub c5: f64,
    pub c6: f64,
}

impl NewmarkCoeffs {
    pub(crate) fn new(newmark: &NewmarkCfg, dt: f64) -> Self {
        let beta = newmark.beta;
        let gamma = newmark.gamma;
        Self {
            gamma,
            c1: 1.0 / (beta * dt * dt),
            c2: gamma / (beta * dt),
            c3: 1.0 / (beta * dt),
            c4: 1.0 / (2.0 * beta) - 1.0,
            c5: gamma / beta - 1.0,
            c6: dt * (gamma / (2.0 * beta) - 1.0),
        }
    }
}

/// 地動入力の影響ベクトル束 `M·r`（自由 DOF 空間）。
///
/// 水平 2 方向（[`horizontal_influence_m`]）と位相差入力の回転
/// （[`theta_influence_m`]）は常に 3 本まとめて使うため、束ねて持つ。
/// 等価地震力 `p = −M·r·ẍg` の構築（[`Self::force_at_into`]）もここに置き、
/// 各積分器のホットループが同じ式を通るようにする。
pub(crate) struct GroundInfluence {
    pub m_r_x: Vec<f64>,
    pub m_r_y: Vec<f64>,
    pub m_r_theta: Vec<f64>,
}

impl GroundInfluence {
    pub(crate) fn build(
        model: &Model,
        dofmap: &DofMap,
        m_free: &faer::sparse::SparseColMat<usize, f64>,
    ) -> Self {
        let (m_r_x, m_r_y) = horizontal_influence_m(model, dofmap, m_free);
        Self {
            m_r_x,
            m_r_y,
            m_r_theta: theta_influence_m(model, dofmap, m_free),
        }
    }

    /// ステップ `n` の等価地震力 `p = −M·r·ẍg(n)`（自由 DOF 空間）を `out` へ書き込む。
    ///
    /// 範囲外のステップ・未指定の成分は加速度 0 として扱う（自由振動・
    /// 片方向入力の波形をそのまま渡せるようにするため）。
    pub(crate) fn force_at_into(&self, wave: &GroundMotion, n: usize, out: &mut [f64]) {
        debug_assert_eq!(
            out.len(),
            self.m_r_x.len(),
            "等価地震力の書き込み先は自由 DOF 空間の長さであること"
        );
        let xg_x = wave.accel_x.get(n).copied().unwrap_or(0.0);
        let xg_y = wave
            .accel_y
            .as_ref()
            .map(|a| a.get(n).copied().unwrap_or(0.0))
            .unwrap_or(0.0);
        let xg_theta = theta_accel_at(wave, n);
        for (i, o) in out.iter_mut().enumerate() {
            *o = -(self.m_r_x[i] * xg_x + self.m_r_y[i] * xg_y + self.m_r_theta[i] * xg_theta);
        }
    }

    /// [`Self::force_at_into`] の新規確保版。ループ外で 1 回だけ使う経路向け。
    pub(crate) fn force_at(&self, wave: &GroundMotion, n: usize) -> Vec<f64> {
        let mut out = vec![0.0; self.m_r_x.len()];
        self.force_at_into(wave, n, &mut out);
        out
    }
}

/// 解くべき独立自由度がない退化ケースの応答（全ステップ 0）。
///
/// `nonlinear`・`applied_long_term` は解析種別によって決まるため引数で受ける。
/// それ以外の欄は「解かなかった」ことを表す空・零の値で揃える。
pub(crate) fn empty_response(
    model: &Model,
    nonlinear: bool,
    applied_long_term: bool,
) -> ResponseResult {
    ResponseResult {
        // 解くべきステップがないため Newton 反復も行われない。
        non_converged_steps: 0,
        time: vec![],
        peak_disp: vec![[0.0; 6]; model.nodes.len()],
        story_drift_angle: vec![0.0; model.layer_count()],
        cumulative_ductility: vec![0.0; model.elements.len()],
        history: ResponseHistory::default(),
        recording: None,
        nonlinear,
        applied_long_term,
    }
}

/// 縮約空間（`n_indep` 長）のベクトルを作り、`values` の先頭を写す。
///
/// 初期変位・初期速度は呼び出し側が短い配列を渡しうるため、長さを切り詰めて
/// 受ける（余りは 0 のまま）。
pub(crate) fn reduced_vec_from(n_indep: usize, values: &[f64]) -> Vec<f64> {
    let mut v = vec![0.0; n_indep];
    let n = n_indep.min(values.len());
    v[..n].copy_from_slice(&values[..n]);
    v
}
