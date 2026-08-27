//! 剛域を考慮した大梁 CMQ（レビュー §1.13 ギャップ「剛域考慮 CMQ」対応）。
//!
//! - [`RigidZoneCmqMode`] — 剛域部分の荷重計算方法（3 方式）
//! - [`RigidZoneCmqResult`] — 剛域考慮 CMQ の計算結果（梁 CMQ＋柱集中荷重）
//! - [`cmq_with_rigid_zone`] — 剛域を考慮した大梁の CMQ を求める

use super::fem::fem_uniform;
use super::types::{Cmq, LoadShape};

// ---------------------------------------------------------------------------
// 剛域考慮 CMQ（レビュー §1.13 ギャップ「剛域考慮 CMQ」対応）
// ---------------------------------------------------------------------------

const SIMPSON_N: usize = 2000;

/// 合成シンプソン則による定積分（決定的・十分な分割数）。`n` は偶数に丸める。
fn simpson_integrate<F: Fn(f64) -> f64>(f: F, a: f64, b: f64, n: usize) -> f64 {
    let n = if n % 2 == 1 { n + 1 } else { n.max(2) };
    let h = (b - a) / n as f64;
    let mut sum = f(a) + f(b);
    for k in 1..n {
        let x = a + k as f64 * h;
        sum += if k % 2 == 0 { 2.0 } else { 4.0 } * f(x);
    }
    sum * h / 3.0
}

/// 荷重形状 `shape`（節点 i 起点、全長 `l_total` の局所座標 x∈[0,l_total] で定義）の
/// 位置 `x` における荷重強度。`fem_uniform`/`fem_triangle`/`fem_trapezoid` が暗黙に
/// 前提とする形状定義と整合させている
/// （三角形＝中央ピーク対称、台形＝両端 a 区間で 0→w0 立上り・中央等高）。
fn shape_intensity(shape: &LoadShape, l_total: f64, x: f64) -> f64 {
    match shape {
        LoadShape::Uniform { w } => *w,
        LoadShape::Linear { w_i, w_j } => {
            if l_total <= 0.0 {
                return 0.0;
            }
            w_i + (w_j - w_i) * (x / l_total)
        }
        LoadShape::Triangle { w0 } => {
            let half = l_total / 2.0;
            if half <= 0.0 {
                return 0.0;
            }
            if x <= half {
                w0 * x / half
            } else {
                w0 * (l_total - x) / half
            }
        }
        LoadShape::Trapezoid { w0, a, .. } => {
            if *a <= 1e-12 {
                *w0
            } else if x < *a {
                w0 * x / a
            } else if x > l_total - a {
                w0 * (l_total - x) / a
            } else {
                *w0
            }
        }
        LoadShape::Point { .. } => 0.0,
    }
}

/// 区間 `[x_lo, x_lo+len]`（`shape` の局所座標系、全長 `l_total`）に作用する荷重の
/// 合力と、その合力作用点までの `x_lo` からの距離（モーメント腕）。
fn zone_load_resultant(shape: &LoadShape, l_total: f64, x_lo: f64, len: f64) -> (f64, f64) {
    if len <= 0.0 {
        return (0.0, 0.0);
    }
    match shape {
        LoadShape::Point { p, x } => {
            if *x >= x_lo && *x <= x_lo + len {
                (*p, x - x_lo)
            } else {
                (0.0, 0.0)
            }
        }
        _ => {
            let total = simpson_integrate(
                |xi| shape_intensity(shape, l_total, x_lo + xi),
                0.0,
                len,
                SIMPSON_N,
            );
            if total.abs() < 1e-12 {
                return (0.0, 0.0);
            }
            let moment = simpson_integrate(
                |xi| shape_intensity(shape, l_total, x_lo + xi) * xi,
                0.0,
                len,
                SIMPSON_N,
            );
            (total, moment / total)
        }
    }
}

/// 可撓区間（長さ `l_flex`、`shape` の局所座標で `[x_start, x_start+l_flex]` に相当）に
/// 切り出した荷重による、その可撓区間だけを長さ `l_flex` の両端固定梁とみなした場合の
/// 固定端モーメント・せん断（`C'`・`Q'`）。一般に非対称（`lam_i ≠ lam_j` で切り出した場合）
/// となるため、対称専用ではない一般公式を用いる:
///   FEM_i = (1/L'²)∫w(ξ)ξ(L'−ξ)²dξ,  FEM_j = (1/L'²)∫w(ξ)ξ²(L'−ξ)dξ
///   Q_i = R_i0 + (FEM_i−FEM_j)/L',   Q_j = R_j0 − (FEM_i−FEM_j)/L'
/// （R_i0, R_j0 は単純梁反力）。`lam_i = lam_j` の対称切り出しでは FEM_i=FEM_j となり
/// 既存の対称専用式（`c_j=−c_i`, `q_i=q_j`）に一致する。
fn cmq_flexible_span(shape: &LoadShape, l_total: f64, x_start: f64, l_flex: f64) -> Cmq {
    match shape {
        LoadShape::Uniform { w } => fem_uniform(*w, l_flex),
        LoadShape::Point { p, x } => {
            let xi = x - x_start;
            if xi < 0.0 || xi > l_flex {
                return Cmq {
                    c_i: 0.0,
                    c_j: 0.0,
                    q_i: 0.0,
                    q_j: 0.0,
                };
            }
            let a = xi;
            let b = l_flex - xi;
            let fem_i = *p * a * b * b / (l_flex * l_flex);
            let fem_j = *p * a * a * b / (l_flex * l_flex);
            let r_i0 = *p * b / l_flex;
            let r_j0 = *p * a / l_flex;
            let delta = (fem_i - fem_j) / l_flex;
            Cmq {
                c_i: fem_i,
                c_j: -fem_j,
                q_i: r_i0 + delta,
                q_j: r_j0 - delta,
            }
        }
        _ => {
            let intensity = |xi: f64| shape_intensity(shape, l_total, x_start + xi);
            let fem_i = simpson_integrate(
                |xi| intensity(xi) * xi * (l_flex - xi).powi(2),
                0.0,
                l_flex,
                SIMPSON_N,
            ) / (l_flex * l_flex);
            let fem_j = simpson_integrate(
                |xi| intensity(xi) * xi * xi * (l_flex - xi),
                0.0,
                l_flex,
                SIMPSON_N,
            ) / (l_flex * l_flex);
            let r_i0 =
                simpson_integrate(|xi| intensity(xi) * (l_flex - xi), 0.0, l_flex, SIMPSON_N)
                    / l_flex;
            let total = simpson_integrate(intensity, 0.0, l_flex, SIMPSON_N);
            let r_j0 = total - r_i0;
            let delta = (fem_i - fem_j) / l_flex;
            Cmq {
                c_i: fem_i,
                c_j: -fem_j,
                q_i: r_i0 + delta,
                q_j: r_j0 - delta,
            }
        }
    }
}

/// 剛域部分の荷重計算方法（3 方式）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RigidZoneCmqMode {
    /// 「剛域を考慮する（剛域部外力はCMoQに加算する）」: 剛域内の荷重を当該端の C・Q に加算する。
    IncludeInCmq,
    /// 「剛域を考慮する（剛域部外力は柱に伝達する）」: 剛域内の荷重は柱への集中荷重として
    /// `column_loads` に集計し、梁の CMQ には含めない。
    TransferToColumn,
    /// 剛域内の荷重を無視する（簡易評価用）。
    Ignore,
}

pub struct RigidZoneCmqResult {
    pub cmq: Cmq,
    /// 剛域内荷重を柱へ伝達する場合の (i側, j側) 集中荷重。`TransferToColumn` 以外は `(0.0, 0.0)`。
    pub column_loads: (f64, f64),
}

/// 剛域を考慮した大梁の CMQ。
///
/// アルゴリズム:
/// 1. 可撓長 `L' = L − λi − λj` の区間に切り出した荷重で `C'/Q'` を求める
///    （[`cmq_flexible_span`]）。等分布はそのまま同じ強度、三角形・台形は切り出し区間の
///    荷重を数値積分（合成シンプソン則）で評価する。
/// 2. 可撓部の端部応力 `C', Q'` を、剛域を片持ち梁とみなしてその先端（可撓部端）から
///    節点（剛域基部）へ伝達する: `C_i = C'_i + Q'_i·λi`、`Q_i = Q'_i`
///    （j側は符号規約 `c_j=−c_i` に整合させ `C_j = C'_j − Q'_j·λj`、`Q_j = Q'_j`）。
/// 3. 剛域内に直接作用する荷重成分（区間 `[0,λi]`・`[L−λj,L]`）は `mode` により:
///    - `IncludeInCmq`: 剛域内荷重 `W` と荷重重心から節点までのモーメント腕 `x̄` を用いて
///      `C_i += W_i·x̄i`、`Q_i += W_i`（j側も同様、符号は `c_j` の向きに合わせて減算）。
///    - `TransferToColumn`: `column_loads` に集計し、CMQ には加えない。
///    - `Ignore`: 無視する（CMQ にも column_loads にも計上しない＝荷重を捨てる）。
///
/// `λi = λj = 0` のとき、いずれの `mode` でも `cmq` は既存の `fem_uniform`/`fem_triangle`/
/// `fem_trapezoid` と厳密に一致する（剛域内荷重・柱集中荷重ともにゼロになるため）。
pub fn cmq_with_rigid_zone(
    shape: &LoadShape,
    l_total: f64,
    lam_i: f64,
    lam_j: f64,
    mode: RigidZoneCmqMode,
) -> RigidZoneCmqResult {
    let l_flex = l_total - lam_i - lam_j;
    if l_flex <= 0.0 {
        return RigidZoneCmqResult {
            cmq: Cmq {
                c_i: 0.0,
                c_j: 0.0,
                q_i: 0.0,
                q_j: 0.0,
            },
            column_loads: (0.0, 0.0),
        };
    }

    // 1) 可撓部分の C'/Q'。
    let flex = cmq_flexible_span(shape, l_total, lam_i, l_flex);

    // 2) 剛域片持ち梁として節点へ伝達。
    let mut c_i = flex.c_i + flex.q_i * lam_i;
    let mut c_j = flex.c_j - flex.q_j * lam_j;
    let mut q_i = flex.q_i;
    let mut q_j = flex.q_j;

    // 3) 剛域内直接荷重。
    let (w_i, xbar_i) = zone_load_resultant(shape, l_total, 0.0, lam_i);
    let (w_j, xbar_j_from_start) = zone_load_resultant(shape, l_total, l_total - lam_j, lam_j);
    let xbar_j = lam_j - xbar_j_from_start; // j端（節点）からの距離に変換

    let mut column_loads = (0.0, 0.0);
    match mode {
        RigidZoneCmqMode::IncludeInCmq => {
            c_i += w_i * xbar_i;
            q_i += w_i;
            c_j -= w_j * xbar_j;
            q_j += w_j;
        }
        RigidZoneCmqMode::TransferToColumn => {
            column_loads = (w_i, w_j);
        }
        RigidZoneCmqMode::Ignore => {}
    }

    RigidZoneCmqResult {
        cmq: Cmq { c_i, c_j, q_i, q_j },
        column_loads,
    }
}
