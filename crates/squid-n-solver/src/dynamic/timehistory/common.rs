//! 積分スキーム間で共有する下位ルーチン。
//!
//! - [`theta_influence_m`] — 位相差入力（ねじれ加振）の回転影響ベクトル `M·r_θ`
//! - [`theta_accel_at`] — ステップ `n` のねじれ地動加速度取得
//! - [`solve_initial_accel`] — 初期加速度 `M·a₀ = rhs` の求解
//! - [`mass_accel_free_into`] — 節点慣性力ベクトル算定用の `M·a_free`
//!   （自由 DOF 空間）
//! - [`sparse_matvec_into`] — `squid_n_math::sparse::sparse_matvec` の出力バッファ
//!   再利用版（squid-n-math に同等 API が入るまでのローカル実装、P9）

use super::config::GroundMotion;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::model::Model;
use squid_n_math::solver::{make_solver, SolveError, SolverBackend};
use squid_n_math::sparse::sparse_matvec;

/// 位相差入力（ねじれ加振）用の回転影響ベクトル × 質量 `M·r_θ` を構築する
/// （多点位相差入力、構造力学）。鉛直（Z）軸まわりの単位角加速度に対し、各節点は
/// 剛体回転 `ax=−(y−yc)`, `ay=(x−xc)` の並進と、回転自由度 rz=1 の影響を受ける
/// （`(xc,yc)`＝節点幾何重心）。返り値は自由 DOF 空間の `M·r_θ`。
pub(crate) fn theta_influence_m(
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
pub(crate) fn theta_accel_at(wave: &GroundMotion, n: usize) -> f64 {
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

/// 疎行列ベクトル積 `y = A·x` を呼び出し側の既存バッファへ書き込む
/// （`squid_n_math::sparse::sparse_matvec` と同一の走査順・演算順で、結果はビット
/// 完全一致。squid-n-math に `sparse_matvec_into` が追加されるまでの間、
/// 時刻歴応答解析側でローカルに実装する。P9）。
pub(crate) fn sparse_matvec_into(
    mat: &faer::sparse::SparseColMat<usize, f64>,
    x: &[f64],
    y: &mut [f64],
) {
    for v in y.iter_mut() {
        *v = 0.0;
    }
    let (sym, vals) = mat.parts();
    let ncols = sym.ncols();
    for j in 0..ncols {
        let range = sym.col_range(j);
        let rows = sym.row_idx_of_col_raw(j);
        let xj = x[j];
        if xj == 0.0 {
            continue;
        }
        for (k, &row) in rows.iter().enumerate() {
            y[row] += vals[range.start + k] * xj;
        }
    }
}
