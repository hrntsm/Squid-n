//! Mander, Priestley & Park (1988) の拘束コンクリート応力–ひずみモデルにおける
//! 包絡線パラメータ計算（純関数群）。
//!
//! 出典: J.B. Mander, M.J.N. Priestley, R. Park, "Theoretical Stress-Strain Model
//! for Confined Concrete", Journal of Structural Engineering, ASCE, 114(8), 1988。
//!
//! 単位規約: 応力 [N/mm²]、長さ [mm]、ひずみは無次元。
//! 圧縮を正の大きさで扱う（fco > 0, eps_co > 0 のように、圧縮側の値を正で表す）。

/// Mander 包絡線のパラメータ（圧縮側、大きさ表記）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ManderParams {
    /// 拘束圧縮強度 f'cc [N/mm²]（正）。
    pub fcc: f64,
    /// f'cc 時ひずみ εcc（正）。
    pub eps_cc: f64,
}

/// [0, 1] にクランプする。
fn clamp01(x: f64) -> f64 {
    if x.is_nan() {
        0.0
    } else {
        x.clamp(0.0, 1.0)
    }
}

/// 拘束強度比 f'cc/f'co を求める。x = f'l/f'co（有効拘束圧比）。
/// f'cc/f'co = -1.254 + 2.254·√(1 + 7.94·x) - 2·x。
pub fn confined_strength_ratio(fl_ratio: f64) -> f64 {
    let x = fl_ratio.max(0.0);
    -1.254 + 2.254 * (1.0 + 7.94 * x).sqrt() - 2.0 * x
}

/// 非拘束強度 fco・ピークひずみ eps_co・有効拘束圧 fl_eff から拘束後パラメータを計算する。
pub fn confined_params(fco: f64, eps_co: f64, fl_eff: f64) -> ManderParams {
    if fco <= 0.0 || eps_co <= 0.0 {
        return ManderParams {
            fcc: 0.0,
            eps_cc: 0.0,
        };
    }
    let fl_ratio = fl_eff.max(0.0) / fco;
    let ratio = confined_strength_ratio(fl_ratio);
    let fcc = fco * ratio;
    let eps_cc = eps_co * (1.0 + 5.0 * (ratio - 1.0));
    ManderParams { fcc, eps_cc }
}

/// 円形断面の横拘束筋データ。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CircularHoop {
    /// コア径 ds [mm]。
    pub ds: f64,
    /// フープのピッチ s [mm]。
    pub s: f64,
    /// フープの純間隔 s' [mm]。
    pub s_clear: f64,
    /// フープ 1 本の断面積 Asp [mm²]。
    pub asp: f64,
    /// フープの降伏強度 fyh [N/mm²]。
    pub fyh: f64,
    /// コア面積に対する主筋比 ρcc。
    pub rho_cc: f64,
    /// スパイラル筋なら true。
    pub spiral: bool,
}

/// 円形の有効拘束圧を求める。
/// ρs = 4·Asp/(ds·s)、ke は円形フープとスパイラルで式が異なる。
pub fn circular_effective_lateral_stress(h: &CircularHoop) -> f64 {
    if h.ds <= 0.0 || h.s <= 0.0 {
        return 0.0;
    }
    let rho_s = 4.0 * h.asp.max(0.0) / (h.ds * h.s);
    let denom = 1.0 - h.rho_cc;
    let geom = 1.0 - h.s_clear.max(0.0) / (2.0 * h.ds);
    let ke_raw = if denom.abs() < 1e-9 {
        0.0
    } else if h.spiral {
        geom / denom
    } else {
        geom * geom / denom
    };
    let ke = clamp01(ke_raw);
    ke * 0.5 * rho_s * h.fyh.max(0.0)
}

/// 矩形断面の横拘束筋データ。
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularHoop {
    /// コア幅 bc [mm]。
    pub bc: f64,
    /// コアせい dc [mm]。
    pub dc: f64,
    /// フープのピッチ s [mm]。
    pub s: f64,
    /// フープの純間隔 s' [mm]。
    pub s_clear: f64,
    /// x 方向に有効な脚断面積合計 Asx [mm²]。
    pub asx: f64,
    /// y 方向に有効な脚断面積合計 Asy [mm²]。
    pub asy: f64,
    /// フープ降伏強度 fyh [N/mm²]。
    pub fyh: f64,
    /// コア面積に対する主筋比 ρcc。
    pub rho_cc: f64,
    /// 隣接主筋間の純間隔 wi [mm] のリスト。
    pub w_clear: Vec<f64>,
}

/// 矩形の有効拘束圧 (f'lx,eff, f'ly,eff) を求める。
pub fn rectangular_effective_lateral_stress(h: &RectangularHoop) -> (f64, f64) {
    if h.bc <= 0.0 || h.dc <= 0.0 || h.s <= 0.0 {
        return (0.0, 0.0);
    }
    let sum_wi2: f64 = h.w_clear.iter().map(|w| w.max(0.0).powi(2)).sum();
    let denom = 1.0 - h.rho_cc;
    let ke_raw = if denom.abs() < 1e-9 {
        0.0
    } else {
        (1.0 - sum_wi2 / (6.0 * h.bc * h.dc))
            * (1.0 - h.s_clear.max(0.0) / (2.0 * h.bc))
            * (1.0 - h.s_clear.max(0.0) / (2.0 * h.dc))
            / denom
    };
    let ke = clamp01(ke_raw);
    let rho_x = h.asx.max(0.0) / (h.s * h.dc);
    let rho_y = h.asy.max(0.0) / (h.s * h.bc);
    let fyh = h.fyh.max(0.0);
    (ke * rho_x * fyh, ke * rho_y * fyh)
}

/// 矩形の代表有効拘束圧（2 方向の算術平均）。
pub fn rectangular_representative_lateral_stress(h: &RectangularHoop) -> f64 {
    let (flx, fly) = rectangular_effective_lateral_stress(h);
    0.5 * (flx + fly)
}

/// 終局圧縮ひずみ εcu（Priestley, Seible & Calvi 1996 のエネルギー近似式）。
pub fn ultimate_strain_priestley(rho_s: f64, fyh: f64, eps_su: f64, fcc: f64) -> f64 {
    let fcc_safe = fcc.max(1e-9);
    0.004 + 1.4 * rho_s.max(0.0) * fyh.max(0.0) * eps_su.max(0.0) / fcc_safe
}

/// Popovics 型包絡線（Mander 式）の評価。`x_strain` は圧縮ひずみの大きさ。
pub fn popovics_envelope(fcc: f64, eps_cc: f64, ec: f64, x_strain: f64) -> (f64, f64) {
    if fcc <= 0.0 || eps_cc <= 0.0 {
        return (0.0, 0.0);
    }
    let e_sec = fcc / eps_cc;
    let ec_clamped = ec.max(e_sec * 1.0001);
    let r = ec_clamped / (ec_clamped - e_sec);
    let x = (x_strain.max(0.0)) / eps_cc;

    let x_pow_r = x.powf(r);
    let denom = r - 1.0 + x_pow_r;

    let sigma = fcc * x * r / denom;
    let tangent = e_sec * r * (r - 1.0) * (1.0 - x_pow_r) / (denom * denom);

    (sigma, tangent)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn confined_strength_ratio_zero_is_exactly_one() {
        // x=0 のとき -1.254 + 2.254·1 - 0 = 1.0（厳密に一致するはず）。
        assert_eq!(confined_strength_ratio(0.0), 1.0);
    }

    #[test]
    fn confined_strength_ratio_matches_hand_calculation() {
        // x=0.1: -1.254 + 2.254·√1.794 - 0.2
        assert_relative_eq!(confined_strength_ratio(0.1), 1.56501_40, epsilon = 1e-4);
        // x=0.3: -1.254 + 2.254·√3.382 - 0.6
        assert_relative_eq!(confined_strength_ratio(0.3), 2.29115_44, epsilon = 1e-4);
    }

    #[test]
    fn confined_strength_ratio_clamps_negative_input() {
        // x<0 は 0 にクランプされ、ratio(0)=1.0 と一致する。
        assert_relative_eq!(confined_strength_ratio(-1.0), 1.0, epsilon = 1e-12);
        assert!(!confined_strength_ratio(-1.0).is_nan());
    }

    #[test]
    fn confined_params_hand_calculation() {
        let p = confined_params(30.0, 0.002, 3.0);
        let ratio = confined_strength_ratio(0.1);
        assert_relative_eq!(p.fcc, 30.0 * ratio, epsilon = 1e-6);
        assert_relative_eq!(p.fcc, 46.95, epsilon = 1e-2);
        assert_relative_eq!(
            p.eps_cc,
            0.002 * (1.0 + 5.0 * (ratio - 1.0)),
            epsilon = 1e-9
        );
        assert_relative_eq!(p.eps_cc, 0.00765, epsilon = 1e-4);
    }

    #[test]
    fn confined_params_nonpositive_inputs_do_not_panic() {
        let p = confined_params(0.0, 0.002, 3.0);
        assert_eq!(p.fcc, 0.0);
        assert_eq!(p.eps_cc, 0.0);
        let p2 = confined_params(30.0, -0.002, 3.0);
        assert_eq!(p2.fcc, 0.0);
        assert!(!p2.eps_cc.is_nan());
    }

    #[test]
    fn circular_effective_lateral_stress_hand_calculation() {
        let h = CircularHoop {
            ds: 400.0,
            s: 100.0,
            s_clear: 90.0,
            asp: 71.0,
            fyh: 295.0,
            rho_cc: 0.02,
            spiral: false,
        };
        // ρs = 4·71/(400·100) = 0.0071
        // ke = (1 - 90/800)² / 0.98
        let rho_s = 4.0 * 71.0 / (400.0 * 100.0);
        let geom = 1.0 - 90.0 / 800.0;
        let ke = geom * geom / 0.98;
        let expected = ke * 0.5 * rho_s * 295.0;
        let fl_eff = circular_effective_lateral_stress(&h);
        assert_relative_eq!(fl_eff, expected, epsilon = 1e-6);
        assert_relative_eq!(fl_eff, 0.84171, epsilon = 1e-3);
    }

    #[test]
    fn circular_effective_lateral_stress_spiral_differs_from_hoop() {
        let hoop = CircularHoop {
            ds: 400.0,
            s: 100.0,
            s_clear: 90.0,
            asp: 71.0,
            fyh: 295.0,
            rho_cc: 0.02,
            spiral: false,
        };
        let mut spiral = hoop;
        spiral.spiral = true;
        let fl_hoop = circular_effective_lateral_stress(&hoop);
        let fl_spiral = circular_effective_lateral_stress(&spiral);
        // 円形フープは (1-s'/2ds)² 、スパイラルは (1-s'/2ds) なので値が異なる。
        assert!(fl_spiral > fl_hoop);
    }

    #[test]
    fn circular_effective_lateral_stress_does_not_panic_on_bad_input() {
        let h = CircularHoop {
            ds: 0.0,
            s: 0.0,
            s_clear: -10.0,
            asp: -5.0,
            fyh: -100.0,
            rho_cc: 1.0,
            spiral: false,
        };
        let fl = circular_effective_lateral_stress(&h);
        assert!(!fl.is_nan());
        assert!(fl.is_finite());
    }

    #[test]
    fn rectangular_effective_lateral_stress_ke_bounded_and_asymmetric() {
        let base = RectangularHoop {
            bc: 500.0,
            dc: 500.0,
            s: 100.0,
            s_clear: 80.0,
            asx: 200.0,
            asy: 300.0,
            fyh: 295.0,
            rho_cc: 0.02,
            w_clear: vec![100.0, 100.0, 100.0, 100.0],
        };
        let (flx, fly) = rectangular_effective_lateral_stress(&base);
        assert!(flx >= 0.0 && fly >= 0.0);
        // Asx ≠ Asy なので flx ≠ fly。
        assert!((flx - fly).abs() > 1e-9);

        // w_clear を大きくすると ke（したがって fl）が下がる。
        let mut wide = base.clone();
        wide.w_clear = vec![300.0, 300.0, 300.0, 300.0];
        let (flx_wide, fly_wide) = rectangular_effective_lateral_stress(&wide);
        assert!(flx_wide < flx);
        assert!(fly_wide < fly);
    }

    #[test]
    fn rectangular_effective_lateral_stress_empty_w_clear_is_ideal_confinement() {
        let h = RectangularHoop {
            bc: 500.0,
            dc: 500.0,
            s: 100.0,
            s_clear: 80.0,
            asx: 200.0,
            asy: 300.0,
            fyh: 295.0,
            rho_cc: 0.02,
            w_clear: vec![],
        };
        // Σwi²=0 なので ke の第 1 項は 1.0（理想拘束）。
        let (flx, fly) = rectangular_effective_lateral_stress(&h);
        assert!(flx.is_finite() && fly.is_finite());
        assert!(flx > 0.0 && fly > 0.0);
    }

    #[test]
    fn rectangular_representative_lateral_stress_is_arithmetic_mean() {
        let h = RectangularHoop {
            bc: 500.0,
            dc: 400.0,
            s: 100.0,
            s_clear: 80.0,
            asx: 200.0,
            asy: 300.0,
            fyh: 295.0,
            rho_cc: 0.02,
            w_clear: vec![100.0, 100.0, 100.0, 100.0],
        };
        let (flx, fly) = rectangular_effective_lateral_stress(&h);
        let rep = rectangular_representative_lateral_stress(&h);
        assert_relative_eq!(rep, 0.5 * (flx + fly), epsilon = 1e-9);
    }

    #[test]
    fn rectangular_effective_lateral_stress_does_not_panic_on_bad_input() {
        let h = RectangularHoop {
            bc: 0.0,
            dc: -10.0,
            s: 0.0,
            s_clear: -5.0,
            asx: -1.0,
            asy: -1.0,
            fyh: -1.0,
            rho_cc: 2.0,
            w_clear: vec![-100.0],
        };
        let (flx, fly) = rectangular_effective_lateral_stress(&h);
        assert!(!flx.is_nan() && flx.is_finite());
        assert!(!fly.is_nan() && fly.is_finite());
    }

    #[test]
    fn ultimate_strain_priestley_hand_calculation() {
        let eps_cu = ultimate_strain_priestley(0.0071, 295.0, 0.10, 46.95);
        // 0.004 + 1.4·0.0071·295·0.10/46.95
        let expected = 0.004 + 1.4 * 0.0071 * 295.0 * 0.10 / 46.95;
        assert_relative_eq!(eps_cu, expected, epsilon = 1e-9);
        assert_relative_eq!(eps_cu, 0.010246, epsilon = 1e-4);
    }

    #[test]
    fn ultimate_strain_priestley_zero_fcc_does_not_panic() {
        let eps_cu = ultimate_strain_priestley(0.0, 0.0, 0.0, 0.0);
        assert!(!eps_cu.is_nan());
        assert!(eps_cu.is_finite());
    }

    #[test]
    fn popovics_envelope_peak_matches_fcc_exactly() {
        let fcc = 46.95;
        let eps_cc = 0.00765;
        let ec = 30000.0;
        let (sigma, _) = popovics_envelope(fcc, eps_cc, ec, eps_cc);
        // x=1 のとき denom=r-1+1=r となり σ=fcc·r/r=fcc（厳密）。
        assert_relative_eq!(sigma, fcc, epsilon = 1e-9);
    }

    #[test]
    fn popovics_envelope_initial_tangent_equals_ec() {
        let fcc = 46.95;
        let eps_cc = 0.00765;
        let ec = 30000.0;
        let (_, tangent) = popovics_envelope(fcc, eps_cc, ec, 0.0);
        // x=0 では r/(r-1)·Esec=Ec の恒等式により、接線は Ec に厳密一致する。
        assert_relative_eq!(tangent, ec, epsilon = 1e-9);
    }

    #[test]
    fn popovics_envelope_stress_decreases_beyond_peak() {
        let fcc = 46.95;
        let eps_cc = 0.00765;
        let ec = 30000.0;
        let (s_peak, _) = popovics_envelope(fcc, eps_cc, ec, eps_cc);
        let (s_1_5, _) = popovics_envelope(fcc, eps_cc, ec, 1.5 * eps_cc);
        let (s_2_0, _) = popovics_envelope(fcc, eps_cc, ec, 2.0 * eps_cc);
        assert!(s_1_5 < s_peak);
        assert!(s_2_0 < s_1_5);
    }

    #[test]
    fn popovics_envelope_guards_ec_le_esec() {
        // Ec ≤ Esec の異常入力でも NaN・パニックを起こさない。
        let fcc = 46.95;
        let eps_cc = 0.00765;
        let e_sec = fcc / eps_cc;
        let (sigma, tangent) = popovics_envelope(fcc, eps_cc, e_sec * 0.5, eps_cc * 0.5);
        assert!(!sigma.is_nan() && sigma.is_finite());
        assert!(!tangent.is_nan() && tangent.is_finite());
    }

    #[test]
    fn popovics_envelope_nonpositive_material_does_not_panic() {
        let (sigma, tangent) = popovics_envelope(0.0, 0.0, 30000.0, 0.001);
        assert_eq!(sigma, 0.0);
        assert_eq!(tangent, 0.0);
        let (sigma2, tangent2) = popovics_envelope(-1.0, -1.0, 30000.0, -0.001);
        assert!(!sigma2.is_nan() && !tangent2.is_nan());
    }
}
