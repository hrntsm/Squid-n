pub const GRAVITY_MM_S2: f64 = 9_806.65;

/// コンクリートの種類（単位体積重量表の行。固定荷重の自重算定に用いる）。
/// 許容応力度低減（軽量1種・2種は普通コンクリートの 0.9 倍。技術基準解説書）にも用いる。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ConcreteClass {
    #[default]
    Normal,
    Lightweight1,
    Lightweight2,
}

/// コンクリート系構造の区分（γC/γRC/γSRC の列に対応）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ConcreteComposition {
    /// 無筋（気乾単位体積重量 γC）
    Plain,
    /// 鉄筋コンクリート（γRC = γC + 1.0）
    #[default]
    Rc,
    /// 鉄骨鉄筋コンクリート（γSRC = γC + 2.0）
    Src,
}

/// コンクリートの単位体積重量 [kN/m³]。
/// 固定荷重の単位体積重量表（設計基準強度 Fc・種類・構造区分ごと）による。
/// 軽量コンクリートで表の範囲を超える Fc は最上段の値で頭打ちとする。
pub fn concrete_unit_weight_kn_m3(fc: f64, class: ConcreteClass, comp: ConcreteComposition) -> f64 {
    let gamma_c = match class {
        ConcreteClass::Normal => {
            if fc <= 36.0 {
                23.0
            } else if fc <= 48.0 {
                23.5
            } else if fc <= 120.0 {
                24.0
            } else {
                24.5
            }
        }
        ConcreteClass::Lightweight1 => {
            if fc <= 27.0 {
                19.0
            } else {
                20.0
            }
        }
        ConcreteClass::Lightweight2 => 17.0,
    };
    // 軽量1種 27<Fc≦36 は γRC=22.0（+2.0）と表の増分が他と異なるため個別に扱う。
    match (class, comp) {
        (ConcreteClass::Lightweight1, ConcreteComposition::Rc) if fc > 27.0 => 22.0,
        (ConcreteClass::Lightweight1, ConcreteComposition::Src) if fc > 27.0 => 23.0,
        (_, ConcreteComposition::Plain) => gamma_c,
        (_, ConcreteComposition::Rc) => gamma_c + 1.0,
        (_, ConcreteComposition::Src) => gamma_c + 2.0,
    }
}

/// 鋼材の単位体積重量 [kN/m³]（固定荷重: γs = 77 kN/m³）。
pub const STEEL_UNIT_WEIGHT_KN_M3: f64 = 77.0;

/// 鋼材の単位重量 [t/m³]（数量積算の慣用値）。
///
/// 積算分野では鋼材比重 7.85 を用いるのが慣用であり、固定荷重の
/// γs = 77 kN/m³（[`STEEL_UNIT_WEIGHT_KN_M3`]。質量換算 ≒7.8518 t/m³）とは
/// 約 0.02% の系統差がある。荷重・質量は 77 kN/m³、数量積算は 7.85 t/m³ を
/// それぞれ正とする分野別の慣用値として使い分ける（仕様）。
pub const STEEL_UNIT_WEIGHT_TAKEOFF_T_M3: f64 = 7.85;

pub mod to_internal {
    pub fn length_m(m: f64) -> f64 {
        m * 1_000.0
    }
    pub fn force_kn(kn: f64) -> f64 {
        kn * 1_000.0
    }
    /// バネ定数 kN/mm → N/mm。
    pub fn stiffness_kn_per_mm(kn_per_mm: f64) -> f64 {
        kn_per_mm * 1_000.0
    }
    /// 粘性係数 C0 [kN·(s/mm)^α] → [N·(s/mm)^α]。
    pub fn viscous_c0_kn(c0_kn: f64) -> f64 {
        c0_kn * 1_000.0
    }
    pub fn line_load_kn_per_m(v: f64) -> f64 {
        v
    }
    pub fn area_load_kn_per_m2(v: f64) -> f64 {
        v / 1_000.0
    }
    pub fn stress_n_per_mm2(v: f64) -> f64 {
        v
    }
    pub fn mass_density_g_per_cm3(v: f64) -> f64 {
        v * 1.0e-9
    }
    pub fn unit_weight_kn_per_m3(v: f64) -> f64 {
        v * 1.0e-6
    }
    pub fn weight_n_to_mass(w_n: f64) -> f64 {
        w_n / super::GRAVITY_MM_S2
    }
    /// 単位体積重量 [kN/m³] → 質量密度 [ton/mm³]（内部単位系 N-mm-s）。
    /// 例: γRC=24.0 kN/m³ → 2.4473e-9 ton/mm³。
    pub fn mass_density_from_unit_weight_kn_m3(v: f64) -> f64 {
        unit_weight_kn_per_m3(v) / super::GRAVITY_MM_S2
    }
}

/// 内部単位系（N-mm）から表示用の単位への換算と、その単位ラベル。
///
/// **どの量をどの単位で見せるか**は日本の構造設計の慣例で決まっており（力は kN、
/// 断面性能は cm 系、応力度は N/mm² など）、量ごとに異なる。その慣例をここに 1 つ
/// だけ置き、画面ごとに違う単位で同じ量を表示することを防ぐ。
///
/// 値の関数とラベル定数は**分離**している。軸ラベル（`"層せん断力 [kN]"`）だけ
/// 要る場面と、値だけ要る場面の双方があるためである。小数桁は規約化しない
/// （一覧表では `{:.0}`、詳細表示では `{:.2}` のような使い分けが正当なため）。
pub mod to_display {
    /// 力の表示単位ラベル。
    pub const LABEL_FORCE: &str = "kN";
    /// モーメントの表示単位ラベル。
    pub const LABEL_MOMENT: &str = "kN·m";
    /// 長さ（階高・建物高さ・スパン）の表示単位ラベル。
    pub const LABEL_LENGTH: &str = "m";
    /// 断面寸法・鉄筋径・変位の表示単位ラベル（内部単位のまま）。
    pub const LABEL_LENGTH_MM: &str = "mm";
    /// 応力度・材料強度の表示単位ラベル（内部単位のまま）。
    pub const LABEL_STRESS: &str = "N/mm²";
    /// 断面積の表示単位ラベル。
    pub const LABEL_AREA: &str = "cm²";
    /// 断面二次モーメント・ねじり定数の表示単位ラベル。
    pub const LABEL_INERTIA: &str = "cm⁴";
    /// 断面係数の表示単位ラベル。
    pub const LABEL_MODULUS: &str = "cm³";
    /// 断面二次半径の表示単位ラベル。
    pub const LABEL_RADIUS: &str = "cm";
    /// 線剛性・バネ定数の表示単位ラベル。
    pub const LABEL_STIFFNESS: &str = "kN/mm";
    /// 面荷重（床・雑壁）の表示単位ラベル。
    pub const LABEL_AREA_LOAD: &str = "kN/m²";
    /// 単位幅あたりモーメント（スラブ検定など）の表示単位ラベル。
    pub const LABEL_MOMENT_PER_WIDTH: &str = "kN·m/m";
    /// 粘性係数 C0 の表示単位ラベル（マクスウェルダンパー）。
    pub const LABEL_VISCOUS_C0: &str = "kN·(s/mm)^α";

    /// 力 N → kN。重量・せん断力・軸力に用いる。
    pub fn force_kn(n: f64) -> f64 {
        n / 1_000.0
    }

    /// モーメント N·mm → kN·m。
    pub fn moment_kn_m(n_mm: f64) -> f64 {
        n_mm / 1.0e6
    }

    /// 長さ mm → m。階高・建物高さ・スパンに用いる。
    pub fn length_m(mm: f64) -> f64 {
        mm / 1_000.0
    }

    /// バネ定数 N/mm → kN/mm。
    pub fn stiffness_kn_per_mm(n_per_mm: f64) -> f64 {
        n_per_mm / 1_000.0
    }

    /// 面荷重 N/mm² → kN/m²。
    pub fn area_load_kn_per_m2(n_per_mm2: f64) -> f64 {
        n_per_mm2 * 1_000.0
    }

    /// 単位幅あたりモーメント N·mm/mm → kN·m/m。
    pub fn moment_kn_m_per_m(n_mm_per_mm: f64) -> f64 {
        n_mm_per_mm / 1_000.0
    }

    /// 粘性係数 C0 N·(s/mm)^α → kN·(s/mm)^α。
    pub fn viscous_c0_kn(c0_n: f64) -> f64 {
        c0_n / 1_000.0
    }

    /// 断面積 mm² → cm²（JIS 形鋼表の慣例）。
    pub fn area_cm2(mm2: f64) -> f64 {
        mm2 / 1.0e2
    }

    /// 断面二次モーメント・ねじり定数 mm⁴ → cm⁴（同上）。
    pub fn inertia_cm4(mm4: f64) -> f64 {
        mm4 / 1.0e4
    }

    /// 断面係数（弾性・塑性）mm³ → cm³（同上）。
    pub fn modulus_cm3(mm3: f64) -> f64 {
        mm3 / 1.0e3
    }

    /// 断面二次半径 mm → cm（同上）。
    pub fn radius_cm(mm: f64) -> f64 {
        mm / 1.0e1
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_unit_conversions() {
        assert_relative_eq!(to_internal::length_m(6.0), 6000.0, max_relative = 1e-12);
        assert_relative_eq!(
            to_internal::line_load_kn_per_m(10.0),
            10.0,
            max_relative = 1e-12
        );
        assert_relative_eq!(to_internal::force_kn(50.0), 50000.0, max_relative = 1e-12);
        assert_relative_eq!(
            to_internal::stiffness_kn_per_mm(10.0),
            10000.0,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            to_display::stiffness_kn_per_mm(10000.0),
            10.0,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            to_display::area_load_kn_per_m2(0.0029),
            2.9,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            to_internal::area_load_kn_per_m2(2.9),
            0.0029,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            to_display::moment_kn_m_per_m(5000.0),
            5.0,
            max_relative = 1e-12
        );
        assert_relative_eq!(to_display::viscous_c0_kn(1000.0), 1.0, max_relative = 1e-12);
        assert_relative_eq!(
            to_internal::viscous_c0_kn(1.0),
            1000.0,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            to_internal::stress_n_per_mm2(24.0),
            24.0,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            to_internal::mass_density_g_per_cm3(2.4),
            2.4e-9,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            to_internal::unit_weight_kn_per_m3(24.0),
            2.4e-5,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            to_internal::weight_n_to_mass(1.0e6),
            101.971_621_297_792_82,
            max_relative = 1e-12
        );
    }

    #[test]
    fn test_concrete_unit_weight_table() {
        use ConcreteClass::*;
        use ConcreteComposition::*;
        // 普通コンクリート（単位体積重量表の代表値）
        assert_eq!(concrete_unit_weight_kn_m3(24.0, Normal, Plain), 23.0);
        assert_eq!(concrete_unit_weight_kn_m3(24.0, Normal, Rc), 24.0);
        assert_eq!(concrete_unit_weight_kn_m3(24.0, Normal, Src), 25.0);
        assert_eq!(concrete_unit_weight_kn_m3(42.0, Normal, Rc), 24.5);
        assert_eq!(concrete_unit_weight_kn_m3(60.0, Normal, Rc), 25.0);
        assert_eq!(concrete_unit_weight_kn_m3(100.0, Normal, Rc), 25.0);
        assert_eq!(concrete_unit_weight_kn_m3(150.0, Normal, Rc), 25.5);
        // 軽量コンクリート
        assert_eq!(concrete_unit_weight_kn_m3(24.0, Lightweight1, Rc), 20.0);
        assert_eq!(concrete_unit_weight_kn_m3(30.0, Lightweight1, Rc), 22.0);
        assert_eq!(concrete_unit_weight_kn_m3(30.0, Lightweight1, Src), 23.0);
        assert_eq!(concrete_unit_weight_kn_m3(21.0, Lightweight2, Rc), 18.0);
    }

    #[test]
    fn test_mass_density_from_unit_weight() {
        // γRC=24 kN/m³ → 24e-6 N/mm³ / 9806.65 mm/s² ≈ 2.4473e-9 t/mm³
        let rho = to_internal::mass_density_from_unit_weight_kn_m3(24.0);
        assert_relative_eq!(rho, 24.0e-6 / GRAVITY_MM_S2, max_relative = 1e-12);
        assert!((rho - 2.4473e-9).abs() / rho < 1e-3);
    }
}
