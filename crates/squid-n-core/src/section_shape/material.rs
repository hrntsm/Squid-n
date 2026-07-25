//! 材料に関する換算関数。
//!
//! - [`concrete_young_modulus`] — コンクリート強度 Fc からヤング係数 Ec を算定
//! - [`wall_shear_shape_factor`] — 耐震壁のせん断形状係数

use super::constants::{GAMMA_CONCRETE, KAPPA_RC};

/// コンクリート強度 Fc [N/mm²] からヤング係数 Ec [N/mm²] を算定する
/// （RC 規準の Ec=3.35·10⁴·(γ/24)²·(Fc/60)^(1/3)、γ=23 固定）。
pub fn concrete_young_modulus(fc: f64) -> f64 {
    concrete_young_modulus_gamma(fc, GAMMA_CONCRETE)
}

/// コンクリート強度 Fc [N/mm²]・気乾単位体積重量 γ [kN/m³] から
/// ヤング係数 Ec [N/mm²] を算定する（RC 規準の Ec=3.35·10⁴·(γ/24)²·(Fc/60)^(1/3)）。
pub fn concrete_young_modulus_gamma(fc: f64, gamma_kn_m3: f64) -> f64 {
    if fc <= 0.0 {
        return 0.0;
    }
    3.35e4 * (gamma_kn_m3 / 24.0).powi(2) * (fc / 60.0).powf(1.0 / 3.0)
}

/// 耐震壁（壁板＋両側柱＝平面 I 形断面）のせん断形状係数
/// （側柱付き壁を I 形断面とみなしたせん断形状係数。材料力学）。
///
/// κ = 3(1+ξ)/(5·(1−ξ³(1−η))²)·[η + ξ(1−η)·((15/8)(1−ξ²)² − ξ⁴·η)]
///
/// ξ・η の定義は原典ページに明示がないため、
/// ξ=壁板内法長さ/全長（側柱外面間）、η=壁厚/側柱幅 と仮定する
/// （式の読み・記号定義とも dev_docs/specs/原典照合リスト.md に要照合として登録）。
/// ξ=1（側柱なし＝矩形断面）で κ=1.2（=`KAPPA_RC`）に一致する。
/// 退化（非有限・非正）時は矩形の 1.2 にフォールバックする。
/// 平面 I 形断面（壁板＝ウェブ、両端の側柱＝フランジ）の**厳密な**せん断形状係数
/// κ = A/I²·∫(Q(y)²/b(y))dy（材料力学。Timoshenko 梁のせん断補正係数の定義）。
///
/// 面内曲げの断面は「壁長方向を断面のせい、壁厚方向を断面の幅」とみなす:
/// - `d_total`: 側柱外面間の全長 D（せい方向）
/// - `dc_each`: 側柱 1 本の沿壁方向せい dc（両端に 1 本ずつ。0 なら側柱なし）
/// - `bc`: 側柱の壁直交方向の幅（フランジ幅）
/// - `t`: 壁板厚（ウェブ幅）
///
/// 一様矩形（`dc_each = 0` または `bc == t`）では厳密に 1.2（=`KAPPA_RC`）を返す。
/// 側柱が大きくなるほど κ は増大し、極限では A/A_web に漸近する（せん断をウェブが
/// ほぼ全負担する I 形断面の性質）。
///
/// 従来の閉形式 [`wall_shear_shape_factor`] は記号定義が原典で確認できず、
/// η=1（＝側柱幅が壁厚に等しい＝一様矩形）でも ξ に依存して 0.6(1+ξ) を返すなど
/// 内部整合性を満たさなかった（側柱が大きいほど κ が 1.2 から**減少**し、
/// せん断断面積が総断面積を超える非物理な値を与えていた）。本関数はその代替。
///
/// 退化入力（非正・非有限）では 1.2 を返す。
pub fn wall_shear_shape_factor_isection(d_total: f64, dc_each: f64, bc: f64, t: f64) -> f64 {
    if !(d_total.is_finite() && dc_each.is_finite() && bc.is_finite() && t.is_finite())
        || d_total <= 0.0
        || t <= 0.0
        || bc <= 0.0
    {
        return KAPPA_RC;
    }
    let c = d_total / 2.0;
    // 側柱せいは全長の半分を超えない（超える指定はウェブ無しとみなす）。
    let dc = dc_each.clamp(0.0, c);
    let a = c - dc; // ウェブの半せい
                    // フランジ幅がウェブ幅以下なら実質一様矩形。
    if dc <= 0.0 || bc <= t {
        return KAPPA_RC;
    }
    let width = |y: f64| -> f64 {
        if y.abs() > a {
            bc
        } else {
            t
        }
    };
    // 断面積・断面二次モーメント（矩形 bc×D から、ウェブ部の (bc−t) を控除）。
    let area = 2.0 * dc * bc + 2.0 * a * t;
    let i = bc * d_total.powi(3) / 12.0 - (bc - t) * (2.0 * a).powi(3) / 12.0;
    if i <= 0.0 || area <= 0.0 {
        return KAPPA_RC;
    }
    // Q(y): y より上の断面一次モーメント。
    let q_of = |y: f64| -> f64 {
        if y >= a {
            // フランジ内
            bc * (c * c - y * y) / 2.0
        } else {
            bc * (c * c - a * a) / 2.0 + t * (a * a - y * y) / 2.0
        }
    };
    // ∫_{-c}^{c} Q²/b dy を Simpson 則で数値積分する（フランジ/ウェブ境界 ±a で
    // 幅が不連続なため、区間を分割して各々で積分する）。対称なので 0..c を 2 倍。
    let integrate = |lo: f64, hi: f64, n: usize| -> f64 {
        if hi <= lo {
            return 0.0;
        }
        let n = if n.is_multiple_of(2) { n } else { n + 1 };
        let dx = (hi - lo) / n as f64;
        let mut s = 0.0;
        for k in 0..=n {
            let y = lo + dx * k as f64;
            // 境界の幅の曖昧さを避けるため、区間内部の幅で評価する。
            let yy = y.clamp(lo + dx * 1e-9, hi - dx * 1e-9);
            let v = q_of(y).powi(2) / width(yy);
            let w = if k == 0 || k == n {
                1.0
            } else if k % 2 == 1 {
                4.0
            } else {
                2.0
            };
            s += w * v;
        }
        s * dx / 3.0
    };
    let integral = 2.0 * (integrate(0.0, a, 200) + integrate(a, c, 200));
    let kappa = area / (i * i) * integral;
    if kappa.is_finite() && kappa >= 1.0 {
        kappa
    } else {
        KAPPA_RC
    }
}

pub fn wall_shear_shape_factor(xi: f64, eta: f64) -> f64 {
    let xi = xi.clamp(0.0, 1.0);
    let eta = eta.clamp(1e-6, 1.0);
    let denom = 5.0 * (1.0 - xi.powi(3) * (1.0 - eta)).powi(2);
    if denom <= 1e-12 {
        return KAPPA_RC;
    }
    let bracket =
        eta + xi * (1.0 - eta) * ((15.0 / 8.0) * (1.0 - xi * xi).powi(2) - xi.powi(4) * eta);
    let k = 3.0 * (1.0 + xi) / denom * bracket;
    if k.is_finite() && k > 0.0 {
        k
    } else {
        KAPPA_RC
    }
}

#[cfg(test)]
mod isection_kappa_tests {
    use super::*;

    /// 一様矩形（側柱なし、または側柱幅＝壁厚）では厳密に 1.2。
    #[test]
    fn test_kappa_uniform_rectangle_is_12() {
        assert!((wall_shear_shape_factor_isection(4000.0, 0.0, 150.0, 150.0) - 1.2).abs() < 1e-9);
        assert!((wall_shear_shape_factor_isection(4000.0, 600.0, 150.0, 150.0) - 1.2).abs() < 1e-9);
    }

    /// I 形（側柱がウェブより厚い）では κ > 1.2 で、側柱が大きいほど増大する。
    /// 従来の閉形式は逆に 1.2 から減少していた（非物理）。
    #[test]
    fn test_kappa_increases_with_flange_size() {
        let t = 150.0;
        let k300 = wall_shear_shape_factor_isection(4000.0, 300.0, 300.0, t);
        let k600 = wall_shear_shape_factor_isection(4000.0, 600.0, 600.0, t);
        let k900 = wall_shear_shape_factor_isection(4000.0, 900.0, 900.0, t);
        assert!(k300 > 1.2, "k300={}", k300);
        assert!(k600 > k300, "k600={} k300={}", k600, k300);
        assert!(k900 > k600, "k900={} k600={}", k900, k600);
    }

    /// せん断断面積 A/κ は総断面積を超えず、ウェブ断面積を下回らない
    /// （I 形断面のせん断はウェブがほぼ全負担する）。従来式は A/κ > A となり破綻していた。
    #[test]
    fn test_shear_area_is_between_web_and_gross() {
        let (d, dc, bc, t) = (4000.0, 600.0, 600.0, 150.0_f64);
        let kappa = wall_shear_shape_factor_isection(d, dc, bc, t);
        let area = 2.0 * dc * bc + 2.0 * (d / 2.0 - dc) * t;
        let web = 2.0 * (d / 2.0 - dc) * t;
        let as_shear = area / kappa;
        assert!(as_shear <= area, "As={} > A={}", as_shear, area);
        assert!(as_shear >= web * 0.8, "As={} << Aweb={}", as_shear, web);
    }

    /// 退化入力は 1.2 へフォールバック。
    #[test]
    fn test_kappa_degenerate_inputs() {
        assert!((wall_shear_shape_factor_isection(0.0, 100.0, 300.0, 150.0) - 1.2).abs() < 1e-12);
        assert!((wall_shear_shape_factor_isection(4000.0, 100.0, 300.0, 0.0) - 1.2).abs() < 1e-12);
    }
}
