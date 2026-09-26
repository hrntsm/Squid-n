//! 鉄筋コンクリート造梁付着の断面検定（RC 規準の梁主筋の付着検討）。
//! 既定の検定経路は RC 規準1999 方式。

use super::section_props::RcRebarInfo;
use super::{concrete_allowable_bond, one_bar_area};

/// RC 規準 1991 方式の付着検定結果。
pub struct Bond1991Result {
    /// 検定比 = τa / fa。
    pub ratio: f64,
    /// 設計用付着応力度 τa = Q/(φ・j) [N/mm²]。
    pub tau: f64,
    /// 付着許容応力度 fa [N/mm²]。
    pub fa: f64,
}

/// RC 梁付着の断面検定（RC 規準 1991 方式）: `τa = Q/(φ・j) ≦ fa`。
/// `top_bar`: 上端筋なら true（fa が低減される）。
/// `phi`: 引張鉄筋の周長総和 Σφ [mm]。
pub fn rc_beam_bond_check_1991(
    q_abs: f64,
    j: f64,
    phi: f64,
    fc_raw: f64,
    top_bar: bool,
    long_term: bool,
) -> Option<Bond1991Result> {
    if q_abs < 0.0 || j <= 0.0 || phi <= 0.0 || fc_raw <= 0.0 {
        return None;
    }
    let tau = q_abs / (phi * j);
    let fa = concrete_allowable_bond(fc_raw, top_bar, long_term);
    if fa <= 0.0 {
        return None;
    }
    Some(Bond1991Result {
        ratio: tau / fa,
        tau,
        fa,
    })
}

/// RC 梁付着の断面検定結果。
pub struct BondCheckResult {
    /// 検定比 = ldb / ld。
    pub ratio: f64,
    /// 付着長さ ld [mm]（カットオフ無しを仮定した表値）。
    pub ld: f64,
    /// 必要付着長さ ldb [mm]。
    pub ldb: f64,
    /// 付着割裂の検討用係数 K。
    pub k: f64,
    /// 横補強筋効果を表す換算長さ W [mm]。
    pub w: f64,
    /// 付着許容応力度 fb [N/mm²]。
    pub fb: f64,
    /// 検定断面の鉄筋引張応力度 σt = |M|/(at・j) [N/mm²]。
    pub sigma_t: f64,
    /// 端部（左端・右端）の検定断面であれば true、中央であれば false。
    pub is_end: bool,
}

/// 主筋 1 本あたりの周長 φ = π・dia [mm]。
fn one_bar_perimeter(dia: f64) -> f64 {
    std::f64::consts::PI * dia
}

/// RC 梁の付着の断面検定（RC 規準1999、通し筋・カットオフ無しを仮定）。
/// `pos`: 検定断面の部材内位置（0.0〜1.0）。
/// `lo`: 柱面間距離 Lo [mm]。`lo<=0` の場合は `None` を返す（検定省略）。
/// `fc_raw`: コンクリート設計基準強度 Fc [N/mm²]。
#[allow(clippy::too_many_arguments)]
pub(crate) fn rc_beam_bond_check(
    pos: f64,
    lo: f64,
    b: f64,
    d_eff: f64,
    j: f64,
    at: f64,
    mz_abs: f64,
    info: &RcRebarInfo,
    fc_raw: f64,
    long_term: bool,
) -> Option<BondCheckResult> {
    if lo <= 0.0 || info.tension_count == 0 || info.main_dia <= 0.0 || at <= 0.0 || j <= 0.0 {
        return None;
    }

    let db = info.main_dia;
    let is_end = !(0.25 < pos && pos < 0.75);

    let ld = if is_end { (lo + d_eff) / 2.0 } else { lo / 2.0 };
    if ld <= 0.0 {
        return None;
    }

    let n1 = info.tension_first_layer_count;

    let clear_spacing = if n1 <= 1.0 {
        5.0 * db
    } else {
        (b - 2.0 * (info.cover + info.shear_dia) - n1 * db) / (n1 - 1.0)
    };
    let c = clear_spacing.min(3.0 * info.cover).min(5.0 * db);

    let ast = info.shear_legs as f64 * one_bar_area(info.shear_dia);
    let w = if info.shear_pitch > 0.0 {
        (20.0 * ast / (info.shear_pitch * n1)).min(2.5 * db)
    } else {
        0.0
    };

    let k_raw = if long_term {
        0.3 * c / db + 0.4
    } else {
        0.3 * (c + w) / db + 0.4
    };
    let k = k_raw.min(2.5);
    if k <= 0.0 {
        return None;
    }

    let fb_other = fc_raw / 60.0 + 0.6;
    let fb_long = if is_end { 0.8 * fb_other } else { fb_other };
    let fb_row1 = if long_term { fb_long } else { fb_long * 1.5 };
    let fb = if info.tension_layers >= 2 {
        0.6 * fb_row1
    } else {
        fb_row1
    };
    if fb <= 0.0 {
        return None;
    }

    let sigma_t = mz_abs / (at * j);
    let as_bar = one_bar_area(db);
    let phi = one_bar_perimeter(db);
    let ldb = sigma_t * as_bar / (k * fb * phi);

    Some(BondCheckResult {
        ratio: ldb / ld,
        ld,
        ldb,
        k,
        w,
        fb,
        sigma_t,
        is_end,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::section_shape::{BeamStirrup, RcBeamRebar, SectionShape};

    #[test]
    fn test_rc_beam_bond_check_1991_hand_calc() {
        // 4-D22（φ=4×π×22）、j=7/8×540、Q=180kN、Fc=24、上端筋・短期。
        let phi = 4.0 * std::f64::consts::PI * 22.0;
        let j = 7.0 / 8.0 * 540.0;
        let res = rc_beam_bond_check_1991(180_000.0, j, phi, 24.0, true, false)
            .expect("有効入力なら Some");
        let tau = 180_000.0 / (phi * j);
        let fa = 1.54 * 1.5;
        assert!((res.tau - tau).abs() < 1e-9);
        assert!((res.fa - fa).abs() < 1e-9);
        assert!((res.ratio - tau / fa).abs() < 1e-9);
        // 不正入力は None。
        assert!(rc_beam_bond_check_1991(1.0, 0.0, phi, 24.0, true, true).is_none());
    }

    fn bond_test_info() -> RcRebarInfo {
        RcRebarInfo {
            cover: 40.0,
            main_dia: 22.0,
            main_count: 6,
            main_count_per_side: 6,
            main_area: 6.0 * one_bar_area(22.0),
            tension_count: 6,
            tension_layers: 1,
            tension_first_layer_count: 6.0,
            tension_count_1991: 3.0,
            shear_dia: 10.0,
            shear_pitch: 100.0,
            shear_legs: 2,
            is_circle: false,
        }
    }

    #[test]
    fn test_bond_lo_zero_skips_check() {
        let info = bond_test_info();
        let result = rc_beam_bond_check(
            0.1,
            0.0,
            300.0,
            539.0,
            471.625,
            1140.4,
            30_000_000.0,
            &info,
            24.0,
            false,
        );
        assert!(result.is_none(), "Lo<=0 のときは付着検定を省略するはず");
    }

    #[test]
    fn test_bond_ld_end_vs_middle() {
        let info = bond_test_info();
        let lo = 3000.0;
        let d_eff = 539.0;
        let end = rc_beam_bond_check(
            0.1,
            lo,
            300.0,
            d_eff,
            471.625,
            1140.4,
            30_000_000.0,
            &info,
            24.0,
            false,
        )
        .unwrap();
        let mid = rc_beam_bond_check(
            0.5,
            lo,
            300.0,
            d_eff,
            471.625,
            1140.4,
            30_000_000.0,
            &info,
            24.0,
            false,
        )
        .unwrap();
        let right = rc_beam_bond_check(
            0.9,
            lo,
            300.0,
            d_eff,
            471.625,
            1140.4,
            30_000_000.0,
            &info,
            24.0,
            false,
        )
        .unwrap();

        assert!((end.ld - (lo + d_eff) / 2.0).abs() < 1e-6);
        assert!((mid.ld - lo / 2.0).abs() < 1e-6);
        assert!((right.ld - (lo + d_eff) / 2.0).abs() < 1e-6);
        assert!(end.is_end);
        assert!(!mid.is_end);
        assert!(right.is_end);
    }

    #[test]
    fn test_bond_fb_top_bar_factor_0_8() {
        // 端部（上端筋想定）の fb は中央（下端筋想定）の 0.8 倍。
        let info = bond_test_info();
        let lo = 3000.0;
        let end = rc_beam_bond_check(
            0.1,
            lo,
            300.0,
            539.0,
            471.625,
            1140.4,
            30_000_000.0,
            &info,
            24.0,
            false,
        )
        .unwrap();
        let mid = rc_beam_bond_check(
            0.5,
            lo,
            300.0,
            539.0,
            471.625,
            1140.4,
            30_000_000.0,
            &info,
            24.0,
            false,
        )
        .unwrap();
        assert!((end.fb - 0.8 * mid.fb).abs() < 1e-9);
    }

    #[test]
    fn test_bond_c_selects_minimum_of_spacing_cover_5db() {
        let info = bond_test_info();
        let b = 300.0;
        let n1 = 6.0;
        let db = 22.0;
        let expected_spacing = (b - 2.0 * (40.0 + 10.0) - n1 * db) / (n1 - 1.0); // = 13.6
        assert!(expected_spacing < 3.0 * 40.0 && expected_spacing < 5.0 * db);

        // 長期は K=0.3・C/db+0.4（W 項なし）なので C を逆算で検証できる。
        let result = rc_beam_bond_check(
            0.1,
            3000.0,
            b,
            539.0,
            471.625,
            1140.4,
            30_000_000.0,
            &info,
            24.0,
            true,
        )
        .unwrap();
        let expected_k = (0.3 * expected_spacing / db + 0.4).min(2.5);
        assert!((result.k - expected_k).abs() < 1e-6);
    }

    /// 新型 `RcBeamRect`（上 4+2 / 下 3+2）の Rc1999 付着で、n1 は引張側最外段の
    /// 本数（上端引張 4 本・下端引張 3 本）を用いる。`w` は n1 に反比例するため
    /// その値で n1 の配線を確認する。
    #[test]
    fn test_bond_1999_n1_uses_first_layer_count_for_new_model() {
        let shape = SectionShape::RcBeamRect {
            b: 400.0,
            d: 600.0,
            rebar: RcBeamRebar {
                main_dia: 22.0,
                top: vec![4, 2],
                bottom: vec![3, 2],
                cover: 40.0,
                stirrup: BeamStirrup {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
        };
        let top = super::super::rebar_info_from_shape(&shape, true).unwrap();
        let bottom = super::super::rebar_info_from_shape(&shape, false).unwrap();
        let run = |info: &RcRebarInfo| {
            rc_beam_bond_check(
                0.1,
                3000.0,
                400.0,
                539.0,
                471.625,
                6.0 * one_bar_area(22.0),
                30_000_000.0,
                info,
                24.0,
                false,
            )
            .unwrap()
        };
        let ast = 2.0 * one_bar_area(10.0);
        let expected_top = (20.0 * ast / (100.0 * 4.0)).min(2.5 * 22.0);
        let expected_bottom = (20.0 * ast / (100.0 * 3.0)).min(2.5 * 22.0);
        assert!((run(&top).w - expected_top).abs() < 1e-9);
        assert!((run(&bottom).w - expected_bottom).abs() < 1e-9);
    }

    #[test]
    fn test_bond_k_clamped_at_2_5() {
        // C・W ともに上限（5db・2.5db）近くまで大きくし、K が 2.5 にクランプ
        // されることを確認する。
        let info = RcRebarInfo {
            cover: 100.0,
            main_dia: 10.0,
            main_count: 2,
            main_count_per_side: 2,
            main_area: 2.0 * one_bar_area(10.0),
            tension_count: 2,
            tension_layers: 1,
            tension_first_layer_count: 2.0,
            tension_count_1991: 1.0,
            shear_dia: 12.0,
            shear_pitch: 50.0,
            shear_legs: 10,
            is_circle: false,
        };
        let result = rc_beam_bond_check(
            0.1,
            3000.0,
            1000.0,
            539.0,
            471.625,
            1140.4,
            30_000_000.0,
            &info,
            24.0,
            false,
        )
        .unwrap();
        assert!((result.k - 2.5).abs() < 1e-9, "K={}", result.k);
    }

    #[test]
    fn test_bond_fb_second_row_factor_0_6() {
        // 多段配筋（layers=2）の fb は 1 段配筋の 0.6 倍（2段目以降の鉄筋列で
        // 代表して検定。RC 規準1999「1段筋以外は 0.6 を乗じる」）。
        // 1段あたり本数 n1=count/layers を 6 本で揃え、K・W を同一にして
        // fb の低減だけを比較する。
        let info1 = bond_test_info();
        let info2 = RcRebarInfo {
            tension_count: 12,
            tension_layers: 2,
            tension_first_layer_count: 6.0,
            tension_count_1991: 6.0,
            ..info1
        };
        let run = |info: &RcRebarInfo| {
            rc_beam_bond_check(
                0.1,
                3000.0,
                300.0,
                539.0,
                471.625,
                1140.4,
                30_000_000.0,
                info,
                24.0,
                false,
            )
            .unwrap()
        };
        let one = run(&info1);
        let two = run(&info2);
        assert!((two.k - one.k).abs() < 1e-9, "K は同一条件");
        assert!((two.fb - 0.6 * one.fb).abs() < 1e-9, "fb={}", two.fb);
        assert!(
            (two.ratio - one.ratio / 0.6).abs() / one.ratio < 1e-9,
            "検定比は 1/0.6 倍厳しくなる"
        );
    }

    #[test]
    fn test_bond_ldb_over_ld_handcalc() {
        let info = bond_test_info();
        let b = 300.0;
        let d_eff = 539.0;
        let j = 471.625;
        let at = 6.0 * std::f64::consts::PI * (22.0_f64 / 2.0).powi(2) / 2.0;
        let lo = 3000.0;
        let mz_abs = 30_000_000.0;
        let fc = 24.0;

        let result =
            rc_beam_bond_check(0.1, lo, b, d_eff, j, at, mz_abs, &info, fc, false).unwrap();

        // 独立に手計算した期待値。
        let n1 = 6.0;
        let db = 22.0;
        let spacing = (b - 2.0 * (40.0 + 10.0) - n1 * db) / (n1 - 1.0);
        let c = spacing.min(3.0 * 40.0).min(5.0 * db);
        let ast = 2.0 * std::f64::consts::PI / 4.0 * 10.0 * 10.0;
        let w = (20.0 * ast / (100.0 * n1)).min(2.5 * db);
        let k = (0.3 * (c + w) / db + 0.4).min(2.5);
        let fb_other = fc / 60.0 + 0.6;
        let fb = 0.8 * fb_other * 1.5; // 端部・短期
        let sigma_t = mz_abs / (at * j);
        let as_bar = std::f64::consts::PI * (db / 2.0).powi(2);
        let phi = std::f64::consts::PI * db;
        let ldb = sigma_t * as_bar / (k * fb * phi);
        let ld = (lo + d_eff) / 2.0;
        let expected_ratio = ldb / ld;

        assert!((result.ldb - ldb).abs() / ldb < 1e-6);
        assert!((result.ld - ld).abs() / ld < 1e-6);
        assert!((result.ratio - expected_ratio).abs() / expected_ratio < 1e-6);
    }
}
