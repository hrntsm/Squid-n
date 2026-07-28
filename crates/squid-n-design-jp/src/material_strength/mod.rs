//! 材料強度・許容応力度（許容応力度検定で用いる材料の許容応力度・材料定数）。
//!
//! 断面検定で用いる材料の許容応力度・材料定数を、材種横断でまとめる:
//! - コンクリート（許容圧縮・許容せん断・ヤング係数・ヤング係数比 n・付着）
//! - 鉄筋（異形鉄筋の許容引張/圧縮・せん断補強筋・降伏点）
//! - 高強度せん断補強筋（製品別 w_ft・pw 上限表）
//! - 鋼材（F 値・許容引張/圧縮/曲げ/せん断）
//!
//! 準拠する規準:
//! - コンクリート・鉄筋の許容応力度・ヤング係数比: 2010年版 RC 規準・構造規定
//!   （建築基準法施行令 第91条・第90/96条）
//! - 鋼材の F 値・許容応力度: 鋼構造設計規準 1973・構造規定（令90条・令98条/告示）
//!
//! 材種ごとにサブモジュールへ分割し、公開 API は本モジュールから再エクスポートする。

mod concrete;
mod high_strength_hoop;
mod rebar;
mod steel;

pub use concrete::{
    concrete_allowable_bond, concrete_allowable_compression, concrete_allowable_compression_class,
    concrete_allowable_shear, concrete_allowable_shear_class, concrete_young_modulus,
    young_ratio_n,
};
pub use high_strength_hoop::{
    high_strength_group, high_strength_pw_cap, high_strength_w_ft, is_high_strength_shear_grade,
    ultimate_hoop_nu0, ultimate_hoop_pw_cap, ultimate_hoop_sigma_wy, HighStrengthGroup,
};
pub use rebar::{
    main_rebar_grade, rebar_allowable_shear, rebar_allowable_tension, rebar_sigma_y,
    rebar_sigma_y_of, shear_rebar_grade,
};
pub use steel::{
    big_lambda, plate_thickness, steel_f_value, steel_f_value_prefix, steel_fc, steel_fs, steel_ft,
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LoadTerm;
    use squid_n_core::model::Material;
    use squid_n_core::units::ConcreteClass;

    // ------------------------------------------------------------------
    // コンクリート
    // ------------------------------------------------------------------

    #[test]
    fn test_concrete_shear_long_term_min_branch() {
        // Fc=21: Fc/30=0.7, 0.49+Fc/100=0.7 で同値。
        assert!((concrete_allowable_shear(21.0, true) - 0.7).abs() < 1e-9);
    }

    #[test]
    fn test_concrete_shear_short_term_is_1_5x_long() {
        let long = concrete_allowable_shear(24.0, true);
        let short = concrete_allowable_shear(24.0, false);
        assert!((short - long * 1.5).abs() < 1e-9);
    }

    #[test]
    fn test_concrete_compression_short_is_2x_long() {
        let long = concrete_allowable_compression(24.0, true);
        assert!((long - 8.0).abs() < 1e-9);
        assert!((concrete_allowable_compression(24.0, false) - 16.0).abs() < 1e-9);
    }

    #[test]
    fn test_lightweight_concrete_is_0_9x() {
        let normal = concrete_allowable_shear_class(24.0, ConcreteClass::Normal, false);
        let light = concrete_allowable_shear_class(24.0, ConcreteClass::Lightweight1, false);
        assert!((light - normal * 0.9).abs() < 1e-12);
        let normal_c = concrete_allowable_compression_class(24.0, ConcreteClass::Normal, true);
        let light_c = concrete_allowable_compression_class(24.0, ConcreteClass::Lightweight2, true);
        assert!((light_c - normal_c * 0.9).abs() < 1e-12);
    }

    #[test]
    fn test_young_ratio_n_buckets() {
        assert_eq!(young_ratio_n(24.0), 15.0);
        assert_eq!(young_ratio_n(27.0), 15.0);
        assert_eq!(young_ratio_n(30.0), 13.0);
        assert_eq!(young_ratio_n(42.0), 11.0);
        assert_eq!(young_ratio_n(60.0), 9.0);
        assert_eq!(young_ratio_n(80.0), 7.0);
    }

    #[test]
    fn test_concrete_allowable_bond_table() {
        // Fc=24 上端筋: min(24/15, 0.9+2/75×24) = min(1.6, 1.54) = 1.54
        assert!((concrete_allowable_bond(24.0, true, true) - 1.54).abs() < 1e-9);
        // Fc=24 その他: min(24/10, 1.35+24/25) = min(2.4, 2.31) = 2.31
        assert!((concrete_allowable_bond(24.0, false, true) - 2.31).abs() < 1e-9);
        assert!(
            (concrete_allowable_bond(24.0, true, false)
                - concrete_allowable_bond(24.0, true, true) * 1.5)
                .abs()
                < 1e-9
        );
        // 低強度側の分岐: Fc=15 上端筋 min(1.0, 1.3) = 1.0（Fc/15 側が支配）。
        assert!((concrete_allowable_bond(15.0, true, true) - 1.0).abs() < 1e-9);
    }

    // ------------------------------------------------------------------
    // 鉄筋
    // ------------------------------------------------------------------

    #[test]
    fn test_rebar_tension_sd345_d29_reduction() {
        assert!((rebar_allowable_tension("SD345", 25.0, true) - 215.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD345", 29.0, true) - 195.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD345", 25.0, false) - 345.0).abs() < 1e-9);
    }

    #[test]
    fn test_rebar_usd685() {
        assert!((rebar_allowable_tension("USD685", 32.0, true) - 215.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("USD685", 32.0, false) - 685.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("USD685", true) - 195.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("USD685", false) - 590.0).abs() < 1e-9);
    }

    #[test]
    fn test_rebar_sigma_y_sources() {
        let mut m = Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: squid_n_core::ids::MaterialId(0),
            name: "SD390".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: Some(24.0),
            fy: None,
        };
        assert!((rebar_sigma_y(&m) - 390.0).abs() < 1e-9);
        m.fy = Some(400.0);
        assert!((rebar_sigma_y(&m) - 400.0).abs() < 1e-9);
        m.fy = None;
        m.name = "unknown".to_string();
        assert!((rebar_sigma_y(&m) - 345.0).abs() < 1e-9);
    }

    // ------------------------------------------------------------------
    // 高強度せん断補強筋
    // ------------------------------------------------------------------

    #[test]
    fn test_high_strength_pw_cap_groups() {
        // ウルボン系(UB785)・SPR785: 短期 1.2%(損傷制御)/1.0%(安全確保)。
        assert!((high_strength_pw_cap("UB785", LoadTerm::Short, true, 24.0) - 0.012).abs() < 1e-9);
        assert!((high_strength_pw_cap("UB785", LoadTerm::Short, false, 24.0) - 0.010).abs() < 1e-9);
        // KW785/KSS785/HDC685: 0.8%。
        assert!((high_strength_pw_cap("KW785", LoadTerm::Short, true, 24.0) - 0.008).abs() < 1e-9);
        // SHD685・MK785: 1.2% 固定。
        assert!((high_strength_pw_cap("SHD685", LoadTerm::Short, true, 24.0) - 0.012).abs() < 1e-9);
        assert!((high_strength_pw_cap("MK785", LoadTerm::Short, false, 24.0) - 0.012).abs() < 1e-9);
        // KH785: min(1.2%, 1.0%・Fc/27)。Fc=24 → 0.010×24/27≈0.008889。
        assert!(
            (high_strength_pw_cap("KH785", LoadTerm::Short, false, 24.0) - 0.010 * 24.0 / 27.0)
                .abs()
                < 1e-9
        );
        // KH685/SPR685: min(1.2%, 1.2%・Fc/27)。Fc=36 → 頭打ち 1.2%。
        assert!((high_strength_pw_cap("KH685", LoadTerm::Short, true, 36.0) - 0.012).abs() < 1e-9);
        assert!(
            (high_strength_pw_cap("SPR685", LoadTerm::Short, false, 24.0) - 0.012 * 24.0 / 27.0)
                .abs()
                < 1e-9
        );
        // 未知品・長期。
        assert!((high_strength_pw_cap("XYZ999", LoadTerm::Short, true, 24.0) - 0.008).abs() < 1e-9);
        assert!((high_strength_pw_cap("UB785", LoadTerm::Long, true, 24.0) - 0.006).abs() < 1e-9);
    }

    #[test]
    fn test_high_strength_w_ft() {
        assert!((high_strength_w_ft("SBPD1275", false) - 585.0).abs() < 1e-9);
        assert!((high_strength_w_ft("UB785", false) - 590.0).abs() < 1e-9);
        assert!((high_strength_w_ft("KH785", true) - 195.0).abs() < 1e-9);
    }

    #[test]
    fn test_ultimate_hoop_sigma_wy_products() {
        // min(25Fc, 上限) 系: Fc=24 → 25·24=600 が支配。Fc=60 → 上限が支配。
        assert!((ultimate_hoop_sigma_wy("SBPD1275", 24.0).unwrap() - 600.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("SBPD1275", 60.0).unwrap() - 1275.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("SBPDN1275/1420", 60.0).unwrap() - 1275.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("UB785", 60.0).unwrap() - 785.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("KSS785", 24.0).unwrap() - 600.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("SHD685", 60.0).unwrap() - 685.0).abs() < 1e-9);
        // HDC685 は Fc 非依存の 685。
        assert!((ultimate_hoop_sigma_wy("HDC685", 24.0).unwrap() - 685.0).abs() < 1e-9);
        // しきい値切替系: KH785 は Fc=27.4 で 25Fc→785 に跳ぶ。
        assert!((ultimate_hoop_sigma_wy("KH785", 27.0).unwrap() - 675.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("KH785", 27.4).unwrap() - 785.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("SPR785", 31.0).unwrap() - 775.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("SPR785", 32.0).unwrap() - 785.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("MK785", 31.0).unwrap() - 775.0).abs() < 1e-9);
        assert!((ultimate_hoop_sigma_wy("MK785", 31.4).unwrap() - 785.0).abs() < 1e-9);
        // 未知製品は None。
        assert!(ultimate_hoop_sigma_wy("SD295", 24.0).is_none());
        assert!(ultimate_hoop_sigma_wy("XYZ999", 24.0).is_none());
    }

    #[test]
    fn test_ultimate_hoop_nu0_and_pw_cap() {
        // 1275 級: ν0 = 0.7·(1.0−Fc/140)。
        let nu = ultimate_hoop_nu0("SBPD1275", 24.0).unwrap();
        assert!((nu - 0.7 * (1.0 - 24.0 / 140.0)).abs() < 1e-12);
        // 785/685 級: ν0 = 0.7·(0.7−Fc/200)。
        let nu2 = ultimate_hoop_nu0("UB785", 24.0).unwrap();
        assert!((nu2 - 0.7 * (0.7 - 24.0 / 200.0)).abs() < 1e-12);
        assert!(ultimate_hoop_nu0("SD295", 24.0).is_none());
        // pw 上限: 1275 級の柱かつ Fc<27 のみ 0.8%、それ以外 1.2%。
        assert!((ultimate_hoop_pw_cap("SBPD1275", 24.0, true).unwrap() - 0.008).abs() < 1e-12);
        assert!((ultimate_hoop_pw_cap("SBPD1275", 24.0, false).unwrap() - 0.012).abs() < 1e-12);
        assert!((ultimate_hoop_pw_cap("SBPD1275", 30.0, true).unwrap() - 0.012).abs() < 1e-12);
        assert!((ultimate_hoop_pw_cap("KH785", 24.0, true).unwrap() - 0.012).abs() < 1e-12);
        assert!(ultimate_hoop_pw_cap("SD295", 24.0, true).is_none());
    }

    // ------------------------------------------------------------------
    // 鋼材
    // ------------------------------------------------------------------

    #[test]
    fn test_f_value_buckets() {
        assert!((steel_f_value("SS400", 40.0).unwrap() - 235.0).abs() < 1e-9);
        assert!((steel_f_value("SS400", 40.1).unwrap() - 215.0).abs() < 1e-9);
        assert!((steel_f_value("SM520", 75.0).unwrap() - 335.0).abs() < 1e-9);
        assert!((steel_f_value("SM520", 76.0).unwrap() - 325.0).abs() < 1e-9);
    }

    /// squid_n_core::material_grade への委譲後も TMCP・LY 系（板厚区分なし）が
    /// 解決できることを確認する。
    #[test]
    fn test_f_value_tmcp_ly() {
        assert_eq!(steel_f_value("TMCP440", 41.0), Some(440.0));
        assert_eq!(steel_f_value("LY225", 40.0), Some(205.0));
    }

    #[test]
    fn test_f_value_prefix_longest_match() {
        assert!((steel_f_value_prefix("SN400B", 30.0).unwrap() - 235.0).abs() < 1e-9);
        assert!((steel_f_value_prefix("SN490B", 30.0).unwrap() - 325.0).abs() < 1e-9);
        assert!(steel_f_value_prefix("UNKNOWN999", 30.0).is_none());
    }

    #[test]
    fn test_steel_ft_fs_short_is_1_5x() {
        assert!((steel_ft(235.0, LoadTerm::Long) - 235.0 / 1.5).abs() < 1e-9);
        assert!((steel_ft(235.0, LoadTerm::Short) - 235.0).abs() < 1e-9);
        assert!(
            (steel_fs(235.0, LoadTerm::Short) - steel_fs(235.0, LoadTerm::Long) * 1.5).abs() < 1e-9
        );
    }

    #[test]
    fn test_steel_fc_continuous_at_lambda() {
        // λ=0 で fc = F/1.5（=ft長期）、λ=Λ で両分岐が連続。
        let f = 235.0;
        assert!((steel_fc(f, 0.0, LoadTerm::Long) - f / 1.5).abs() < 1e-6);
        let big_l = big_lambda(f);
        let below = steel_fc(f, big_l - 1e-6, LoadTerm::Long);
        let above = steel_fc(f, big_l + 1e-6, LoadTerm::Long);
        assert!((below - above).abs() < 1e-3);
        assert!((below - (18.0 / 65.0) * f).abs() < 1e-2);
    }
}
