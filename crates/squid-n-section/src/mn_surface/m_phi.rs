//! 塑性化域を考慮した M-φ / M-θ 曲線（材端剛塑性ばねモデルと適合するファイバーモデル化）。
//!
//! **GUI のヒンジ詳細表示専用**の弾完全塑性評価であり、非線形解析本体
//! （`squid-n-element` のファイバ定式化）や V&V 参照実装（`squid-n-skeleton` の
//! RC トリリニア骨格）とは塑性化域の集約規則が異なる。適用範囲の使い分けは
//! `squid_n_skeleton` クレートの doc（M–φ エンジンの使い分け）を参照。

use super::plastic::axial_capacity;
use super::types::PlasticFiber;

/// 一定軸力下の断面 M-φ 曲線（塑性化進展を追う弾完全塑性評価）。
pub struct MPhiCurve {
    /// [φ (1/mm), M (N·mm)] の点列（φ=0 から単調増加）
    pub points: Vec<[f64; 2]>,
    /// 初期断面曲げ剛性 EI₀ [N·mm²]（最初の載荷ステップの割線剛性）
    pub ei0: f64,
}

/// 曲げ方向 (ky, kz)（正規化）・一定軸力 `n_target` の下で、曲率 φ を漸増させた
/// 断面 M-φ 曲線を返す。各ファイバは弾完全塑性 σ = clamp(E·ε, σc, σt) とし、
/// 各 φ で軸ひずみ ε0 を二分法で調整して軸力を `n_target` に一致させる。
///
/// 断面内の降伏の進展（塑性化域の広がり）が M-φ の丸みとして現れる:
/// マルチファイバーは滑らか、マルチスプリングは少数バネの逐次降伏で折れ線状になる。
/// φ の上限は最外縁ひずみが降伏ひずみの約12倍となる曲率とし、`n_steps` 等分で返す。
/// `n_target` が軸耐力範囲外、またはファイバが空なら `None`。
pub fn m_phi_curve(
    fibers: &[PlasticFiber],
    ky: f64,
    kz: f64,
    n_target: f64,
    n_steps: usize,
) -> Option<MPhiCurve> {
    let (nc, nt) = axial_capacity(fibers);
    if fibers.is_empty() || n_target <= nc || n_target >= nt {
        return None;
    }

    // てこ距離（中立軸直交方向の縁距離）と降伏ひずみの代表値
    let d_max = fibers
        .iter()
        .map(|f| (ky * f.z - kz * f.y).abs())
        .fold(0.0, f64::max)
        .max(1.0);
    let eps_y_max = fibers
        .iter()
        .map(|f| (f.sigma_t.abs().max(f.sigma_c.abs())) / f.young.max(1.0))
        .fold(0.0, f64::max)
        .max(1e-6);
    let phi_max = 12.0 * eps_y_max / d_max;

    // 一定軸力を満たす ε0 を二分法で求め、そのときの M（曲げ方向成分）を返す
    let section_m = |phi: f64| -> f64 {
        let force = |e0: f64| -> (f64, f64) {
            let mut n = 0.0;
            let mut m = 0.0;
            for f in fibers {
                let d = ky * f.z - kz * f.y;
                let eps = e0 + phi * d;
                let sigma = (f.young * eps).clamp(f.sigma_c, f.sigma_t);
                n += sigma * f.area;
                m += sigma * f.area * d;
            }
            (n, m)
        };
        // 探索区間: 全ファイバが引張/圧縮降伏しきる ε0 で挟めば N は nc/nt に到達する
        let mut lo = -(phi * d_max + 2.0 * eps_y_max);
        let mut hi = phi * d_max + 2.0 * eps_y_max;
        for _ in 0..80 {
            let mid = 0.5 * (lo + hi);
            let (n, _) = force(mid);
            if n < n_target {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        force(0.5 * (lo + hi)).1
    };

    let n_steps = n_steps.max(2);
    let mut points = Vec::with_capacity(n_steps + 1);
    for i in 0..=n_steps {
        let phi = phi_max * i as f64 / n_steps as f64;
        points.push([phi, section_m(phi)]);
    }

    // 初期剛性は最初のステップの割線（φ=0 の点は M が 0 とは限らないため差分をとる）
    let ei0 = (points[1][1] - points[0][1]) / (points[1][0] - points[0][0]).max(1e-30);

    Some(MPhiCurve { points, ei0 })
}

/// 塑性化領域長さ Lp を考慮した材端 M-θ 骨格曲線への換算。
///
/// 部材（内法スパン `span`、逆対称曲げ・反曲点は部材中央）の端部に長さ Lp の
/// 塑性化領域を仮定し、材端回転角を
///
/// ```text
/// θ(M) = M·L/(6·EI₀) + Lp·(φ(M) − M/EI₀)
/// ```
///
/// で評価する（第1項: 弾性部材の材端回転、第2項: 塑性化領域の塑性曲率 φp を
/// Lp で集約した塑性回転 θp = φp·Lp）。材端剛塑性ばねモデルと適合する
/// 集中塑性ヒンジ型ファイバーモデル化（塑性ヒンジ長 Lp による塑性回転集約）の
/// 考え方に対応する。
/// 返り値は [θ (rad), M (N·mm)] の点列。
pub fn m_theta_curve(mphi: &MPhiCurve, span: f64, lp: f64) -> Vec<[f64; 2]> {
    let ei0 = mphi.ei0.max(1.0);
    mphi.points
        .iter()
        .map(|&[phi, m]| {
            let theta_el = m * span / (6.0 * ei0);
            let phi_p = (phi - m / ei0).max(0.0);
            [theta_el + lp * phi_p, m]
        })
        .collect()
}
