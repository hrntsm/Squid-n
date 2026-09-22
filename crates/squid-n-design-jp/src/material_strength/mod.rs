//! 材料強度・許容応力度（許容応力度検定で用いる材料の許容応力度・材料定数）。
//! コンクリート・鉄筋は RC 規準・構造規定、鋼材は鋼構造設計規準 1973・構造規定に準拠する。

mod concrete;
mod rebar;
mod steel;

pub use concrete::{
    concrete_allowable_bond, concrete_allowable_compression, concrete_allowable_shear,
    concrete_allowable_shear_class, concrete_young_modulus, young_ratio_n,
};
pub use rebar::{
    main_rebar_grade, rebar_allowable_shear, rebar_allowable_tension, rebar_sigma_y_of,
    shear_rebar_grade,
};
pub use steel::{
    big_lambda, plate_thickness, steel_f_value, steel_f_value_prefix, steel_fc, steel_fs, steel_ft,
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LoadTerm;
    use squid_n_core::model::{Material, MaterialCategory};
    use squid_n_core::units::ConcreteClass;

    #[test]
    fn test_concrete_shear_long_term_min_branch() {
        // Fc=21: Fc/30=0.7, 0.5+Fc/100=0.71 → Fc/30 側が支配。
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
        // 許容圧縮応力度はコンクリート種類に依存しない（普通コンクリートと同値）。
        assert!((concrete_allowable_compression(24.0, true) - 8.0).abs() < 1e-12);
    }

    #[test]
    fn test_concrete_fc24_representative_values() {
        assert!((concrete_allowable_compression(24.0, true) - 8.0).abs() < 1e-12);
        assert!((concrete_allowable_compression(24.0, false) - 16.0).abs() < 1e-12);
        assert!(
            (concrete_allowable_shear_class(24.0, ConcreteClass::Normal, true) - 0.74).abs()
                < 1e-12
        );
        assert!(
            (concrete_allowable_shear_class(24.0, ConcreteClass::Normal, false) - 1.11).abs()
                < 1e-12
        );
        for class in [ConcreteClass::Lightweight1, ConcreteClass::Lightweight2] {
            assert!((concrete_allowable_shear_class(24.0, class, true) - 0.666).abs() < 1e-12);
            assert!((concrete_allowable_shear_class(24.0, class, false) - 0.999).abs() < 1e-12);
        }
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
    }

    /// σy は断面の主筋材料の `fy` だけから決まる。材料名からの推定は行わない
    /// （材料名は許容応力度表の引き当てにのみ用いる）。
    #[test]
    fn test_rebar_sigma_y_sources() {
        let mut m = Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: squid_n_core::ids::MaterialId(0),
            name: "SD390".to_string(),
            category: MaterialCategory::Rebar,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: Some(24.0),
            fy: None,
        };
        // fy が無ければ 0（検定の入口で止まるため耐力算定へは流れない）。
        assert!(rebar_sigma_y_of(Some(&m)).abs() < 1e-9);
        m.fy = Some(400.0);
        assert!((rebar_sigma_y_of(Some(&m)) - 400.0).abs() < 1e-9);
        // 主筋の材料が未割当でも 0 とし、既定値をでっち上げない。
        assert!(rebar_sigma_y_of(None).abs() < 1e-9);
    }

    /// F 値表・prefix の詳細は `squid_n_core::material_grade` を正とする。
    /// 本クレートは再エクスポートの配線のみを確認する。
    #[test]
    fn test_steel_f_value_reexport_wires_to_core() {
        assert_eq!(steel_f_value("SS400", 40.0), Some(235.0));
        assert_eq!(steel_f_value_prefix("SN400B", 30.0), Some(235.0));
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
        // λ=0 で fc = F/1.5（=ft長期）、λ=Λ で両分岐が連続（0.277F 近傍）。
        let f = 235.0;
        let e = 205_000.0;
        assert!((steel_fc(f, e, 0.0, LoadTerm::Long) - f / 1.5).abs() < 1e-6);
        let big_l = big_lambda(f, e);
        let below = steel_fc(f, e, big_l - 1e-9, LoadTerm::Long);
        let above = steel_fc(f, e, big_l + 1e-9, LoadTerm::Long);
        // 両分岐の差は 0.277 と 3.6/13 の丸め分のみ（F の 1e-4 未満）。
        assert!(
            (below - above).abs() < 1e-4 * f,
            "below={} above={}",
            below,
            above
        );
        // λ>Λ 側は λ=Λ（r=1）で 0.277F に一致する。
        assert!((above - 0.277 * f).abs() < 1e-6, "above={}", above);
    }

    /// 代表値: `big_lambda(235, 205000) = √(π²·205000/(0.6·235)) ≈ 119.7891`。
    #[test]
    fn test_big_lambda_representative_value() {
        let expected = (std::f64::consts::PI.powi(2) * 205_000.0 / (0.6 * 235.0)).sqrt();
        assert!((expected - 119.7891).abs() < 1e-3, "expected={}", expected);
        assert!((big_lambda(235.0, 205_000.0) - expected).abs() < 1e-12);
    }

    /// E を小さくすると Λ が小さくなり、同じ λ（λ<Λ 側）で r=λ/Λ が増えて
    /// fc が下がることを確認する。
    #[test]
    fn test_steel_fc_decreases_with_smaller_e() {
        let f = 235.0;
        let lambda = 50.0;
        let fc_e205 = steel_fc(f, 205_000.0, lambda, LoadTerm::Long);
        let fc_e100 = steel_fc(f, 100_000.0, lambda, LoadTerm::Long);
        assert!(
            fc_e100 < fc_e205,
            "fc(E=100000)={} fc(E=205000)={}",
            fc_e100,
            fc_e205
        );
    }

    /// λ>Λ 側は `0.277·F/(λ/Λ)²` に一致する。
    #[test]
    fn test_steel_fc_elastic_branch_matches_formula() {
        let f = 235.0;
        let e = 205_000.0;
        let lambda = 300.0;
        let big_l = big_lambda(f, e);
        let r = lambda / big_l;
        let expected = 0.277 * f / (r * r);
        assert!((steel_fc(f, e, lambda, LoadTerm::Long) - expected).abs() < 1e-9);
    }

    /// 短期は長期の 1.5 倍。
    #[test]
    fn test_steel_fc_short_is_1_5x_long() {
        let f = 235.0;
        let e = 205_000.0;
        for lambda in [0.0, 50.0, 300.0] {
            let long = steel_fc(f, e, lambda, LoadTerm::Long);
            let short = steel_fc(f, e, lambda, LoadTerm::Short);
            assert!((short - long * 1.5).abs() < 1e-9, "λ={}", lambda);
        }
    }
}
