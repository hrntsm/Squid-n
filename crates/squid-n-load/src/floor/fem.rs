//! 両端固定梁の固定端モーメント・せん断（CMQ）の閉形式公式。
//!
//! - [`fem_uniform`] — 等分布荷重の CMQ
//! - [`fem_linear`] — 線形変化分布（始端 w_i → 終端 w_j）の CMQ
//! - [`fem_triangle`] — 対称三角形荷重の CMQ
//! - [`fem_trapezoid`] — 対称台形荷重の CMQ（閉形式評価）
//! - [`simple_reactions`] — `MemberLoadKind` の単純梁反力
//! - [`simple_beam_moment_at`] — `MemberLoadKind` 列の単純梁曲げモーメント M(x)
//! - [`fixed_end_moments`] — `MemberLoadKind` の両端固定端モーメント（点は閉形式、
//!   分布は Gauss-Legendre 3点で厳密積分）

use super::types::Cmq;
use squid_n_core::model::MemberLoadKind;

pub(crate) fn fem_uniform(w: f64, l: f64) -> Cmq {
    Cmq {
        c_i: w * l * l / 12.0,
        c_j: -w * l * l / 12.0,
        q_i: w * l / 2.0,
        q_j: w * l / 2.0,
    }
}

/// 線形変化分布（始端強度 `w_i`、終端強度 `w_j`、全長 `l`）の両端固定梁 CMQ。
/// `w_i = w_j = w` のとき [`fem_uniform`] に一致する。
pub(crate) fn fem_linear(w_i: f64, w_j: f64, l: f64) -> Cmq {
    Cmq {
        c_i: l * l * (3.0 * w_i + 2.0 * w_j) / 60.0,
        c_j: -l * l * (2.0 * w_i + 3.0 * w_j) / 60.0,
        q_i: l * (7.0 * w_i + 3.0 * w_j) / 20.0,
        q_j: l * (3.0 * w_i + 7.0 * w_j) / 20.0,
    }
}

pub(crate) fn fem_triangle(w0: f64, l: f64) -> Cmq {
    Cmq {
        c_i: 5.0 * w0 * l * l / 96.0,
        c_j: -5.0 * w0 * l * l / 96.0,
        q_i: w0 * l / 4.0,
        q_j: w0 * l / 4.0,
    }
}

/// 対称台形荷重（両端 a 区間で 0→w0 に線形立上り、中央 L−2a 区間は等高 w0）の
/// 両端固定梁の固定端モーメント・せん断。
/// 固定端モーメントは閉形式 FEM = (1/L²)∫₀ᴸ w(x)·x·(L−x)² dx を評価して求める。
/// 検算: a→L/2 で対称三角形 5w0L²/96、a→0 で等分布 w0L²/12 に一致する。
#[allow(unused_variables)]
pub(crate) fn fem_trapezoid(w0: f64, a: f64, b: f64, l: f64) -> Cmq {
    // ∫ x(L-x)² dx の不定積分
    let g = |x: f64| l * l * x * x / 2.0 - 2.0 * l * x * x * x / 3.0 + x.powi(4) / 4.0;
    // 両端の三角形立上り区間（[0,a] と [L-a,L]）の寄与（/a を約分済みの閉形式）
    let i_ends = w0 * l * a * a * (l / 3.0 - a / 4.0);
    // 中央の等分布区間 [a, L-a] の寄与
    let i_mid = w0 * (g(l - a) - g(a));
    let fem = (i_ends + i_mid) / (l * l);
    // 総荷重 = 台形面積（単位幅あたり）= w0·(L−a)。せん断は対称なので両端で W/2。
    let total = w0 * (l - a);
    Cmq {
        c_i: fem,
        c_j: -fem,
        q_i: total / 2.0,
        q_j: total / 2.0,
    }
}

/// 区間分布荷重 [a,b]（強度 w1→w2 の線形分布）の合計荷重と、i 端からの荷重重心位置。
/// `b<=a`（無効区間）は合計 0・重心は区間中点を返す。
fn distributed_total_and_centroid(a: f64, b: f64, w1: f64, w2: f64) -> (f64, f64) {
    let len = b - a;
    if len <= 1e-9 {
        return (0.0, a);
    }
    let total = (w1 + w2) / 2.0 * len;
    let xbar = if (w1 + w2).abs() < 1e-12 {
        a + len / 2.0
    } else {
        // 台形（三角形含む）の重心: 立上り側 w2 寄りに偏る（w1=0,w2=w0 の三角形なら
        // 底辺から 2/3 の位置＝ a+2len/3 になることを確認済み）。
        a + len * (w1 + 2.0 * w2) / (3.0 * (w1 + w2))
    };
    (total, xbar)
}

/// `MemberLoadKind` 1件の単純梁（両端ピン支持）反力 (R_i, R_j)。
/// 集中荷重は P·b/L, P·a/L、分布荷重は合計荷重を重心位置で按分する。
pub fn simple_reactions(load: &MemberLoadKind, l: f64) -> (f64, f64) {
    let (total, xbar) = match *load {
        MemberLoadKind::Point { a, p } => (p, a.clamp(0.0, l)),
        MemberLoadKind::Distributed { a, b, w1, w2 } => {
            distributed_total_and_centroid(a, b, w1, w2)
        }
    };
    if l <= 1e-9 {
        return (total / 2.0, total / 2.0);
    }
    let t = (xbar / l).clamp(0.0, 1.0);
    (total * (1.0 - t), total * t)
}

/// `MemberLoadKind` 1件のみが作用する単純梁の、位置 x [mm]（i 端から）における
/// 曲げモーメント。`simple_beam_moment_at` が複数荷重の重ね合わせに使う内部評価関数。
fn single_load_moment_at(load: &MemberLoadKind, l: f64, x: f64) -> f64 {
    match *load {
        MemberLoadKind::Point { a, p } => {
            let a = a.clamp(0.0, l);
            if x <= a {
                p * (l - a) / l * x
            } else {
                p * a / l * (l - x)
            }
        }
        MemberLoadKind::Distributed { a, b, w1, w2 } => {
            if b <= a {
                return 0.0;
            }
            let (r_i, _) = simple_reactions(load, l);
            // w(s) = m·s + c（区間 [a,b] の線形分布、区間外は 0）
            let m = (w2 - w1) / (b - a);
            let c = w1 - m * a;
            // ∫ w(s)(x-s) ds の不定積分（s について）
            let f =
                |s: f64| m * x * s * s / 2.0 - m * s * s * s / 3.0 + c * x * s - c * s * s / 2.0;
            // x < a はまだ荷重区間に入らないため s2=a（積分 0）、a<=x<=b は [a,x]、
            // x > b は区間全体 [a,b] を積分する。
            let s2 = x.min(b).max(a);
            let integral = f(s2) - f(a);
            r_i * x - integral
        }
    }
}

/// `loads` 列（同一部材に載る全 `MemberLoadKind`）の単純梁曲げモーメントを、
/// 位置 x [mm]（i 端から）で評価する。単純梁の曲げモーメントは荷重に対して
/// 線形（重ね合わせ可能）なので、各荷重を単独載荷したときのモーメントの和で求まる。
pub fn simple_beam_moment_at(loads: &[MemberLoadKind], l: f64, x: f64) -> f64 {
    loads
        .iter()
        .map(|load| single_load_moment_at(load, l, x))
        .sum()
}

/// [-1,1] 上の Gauss-Legendre 3点則の節点・重み。
const GAUSS3_X: [f64; 3] = [-0.774_596_669_241_483_4, 0.0, 0.774_596_669_241_483_4];
const GAUSS3_W: [f64; 3] = [5.0 / 9.0, 8.0 / 9.0, 5.0 / 9.0];

/// Gauss-Legendre 3点則による [a,b] 区間の定積分（被積分関数が5次以下なら厳密）。
fn gauss3(f: impl Fn(f64) -> f64, a: f64, b: f64) -> f64 {
    let mid = (a + b) / 2.0;
    let half = (b - a) / 2.0;
    GAUSS3_X
        .iter()
        .zip(GAUSS3_W.iter())
        .map(|(&xi, &wi)| wi * f(mid + half * xi))
        .sum::<f64>()
        * half
}

/// `MemberLoadKind` 1件の両端固定端モーメント (c_i, c_j)（符号規約は [`Cmq`] と同一。
/// 等分布のように i 端寄りが正曲げなら c_i が正・c_j が負になる）。
///
/// 集中荷重は閉形式 `C_i = P·a·b²/L²`、`C_j = −P·a²·b/L²`。線形分布区間は
/// `FEM_i = ∫w(x)·x·(L−x)²/L² dx`、`FEM_j = −∫w(x)·x²·(L−x)/L² dx` を
/// Gauss-Legendre 3点で区間積分する（被積分関数は x の4次式なので厳密）。
pub fn fixed_end_moments(load: &MemberLoadKind, l: f64) -> (f64, f64) {
    if l <= 1e-9 {
        return (0.0, 0.0);
    }
    match *load {
        MemberLoadKind::Point { a, p } => {
            let a = a.clamp(0.0, l);
            let b = l - a;
            let c_i = p * a * b * b / (l * l);
            let c_j = -p * a * a * b / (l * l);
            (c_i, c_j)
        }
        MemberLoadKind::Distributed { a, b, w1, w2 } => {
            if b <= a {
                return (0.0, 0.0);
            }
            let w = |s: f64| {
                let t = (s - a) / (b - a);
                w1 + (w2 - w1) * t
            };
            let f_i = |s: f64| w(s) * s * (l - s).powi(2) / (l * l);
            let f_j = |s: f64| -w(s) * s * s * (l - s) / (l * l);
            (gauss3(f_i, a, b), gauss3(f_j, a, b))
        }
    }
}
