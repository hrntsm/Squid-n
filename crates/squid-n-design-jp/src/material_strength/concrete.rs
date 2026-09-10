//! コンクリートの許容応力度・材料定数（許容圧縮・許容せん断・ヤング係数・
//! ヤング係数比 n・付着）。

use squid_n_core::units::ConcreteClass;

/// コンクリートの許容圧縮応力度 fc [N/mm²]。
///
/// `長期 = Fc/3`、`短期 = 長期 × 2`（令91条）。
pub fn concrete_allowable_compression(fc: f64, long_term: bool) -> f64 {
    let long = fc / 3.0;
    if long_term {
        long
    } else {
        long * 2.0
    }
}

/// コンクリートの許容せん断応力度 fs [N/mm²]。
///
/// `長期 = min(Fc/30, 0.49 + Fc/100)`、`短期 = 長期 × 1.5`。
pub fn concrete_allowable_shear(fc: f64, long_term: bool) -> f64 {
    let long = (fc / 30.0).min(0.49 + fc / 100.0);
    if long_term {
        long
    } else {
        long * 1.5
    }
}

/// コンクリート種類による許容応力度の低減係数。
///
/// 軽量コンクリート1種・2種の許容応力度（圧縮・せん断）は普通コンクリートの
/// `0.9 倍`（RC規準・構造規定）。
fn concrete_class_factor(class: ConcreteClass) -> f64 {
    match class {
        ConcreteClass::Normal => 1.0,
        ConcreteClass::Lightweight1 | ConcreteClass::Lightweight2 => 0.9,
    }
}

/// コンクリートの許容圧縮応力度 fc [N/mm²]（コンクリート種類対応版）。
///
/// `fc = concrete_allowable_compression × concrete_class_factor`。
pub fn concrete_allowable_compression_class(fc: f64, class: ConcreteClass, long_term: bool) -> f64 {
    concrete_allowable_compression(fc, long_term) * concrete_class_factor(class)
}

/// コンクリートの許容せん断応力度 fs [N/mm²]（コンクリート種類対応版）。
///
/// `fs = concrete_allowable_shear × concrete_class_factor`。
pub fn concrete_allowable_shear_class(fc: f64, class: ConcreteClass, long_term: bool) -> f64 {
    concrete_allowable_shear(fc, long_term) * concrete_class_factor(class)
}

/// 断面算定用のヤング係数比 n [無次元]（Fc に応じた区分値）。
pub fn young_ratio_n(fc: f64) -> f64 {
    if fc <= 27.0 {
        15.0
    } else if fc <= 36.0 {
        13.0
    } else if fc <= 48.0 {
        11.0
    } else if fc <= 60.0 {
        9.0
    } else {
        7.0
    }
}

/// コンクリートのヤング係数 Ec [N/mm²]。
///
/// `Ec = 3.35×10⁴・(γ/24)²・(Fc/60)^(1/3)`、γ は単位容積重量 [kN/m³]
/// （`None` なら 23）。
pub fn concrete_young_modulus(fc: f64, gamma_kn_m3: Option<f64>) -> f64 {
    match gamma_kn_m3 {
        Some(gamma) => squid_n_core::section_shape::concrete_young_modulus_gamma(fc, gamma),
        None => squid_n_core::section_shape::concrete_young_modulus(fc),
    }
}

/// コンクリートの付着許容応力度 fa [N/mm²]（異形鉄筋のみ）。
///
/// 長期は上端筋・その他で 2 分岐、短期は長期の 1.5 倍。
pub fn concrete_allowable_bond(fc: f64, top_bar: bool, long_term: bool) -> f64 {
    let long = if top_bar {
        (fc / 15.0).min(0.9 + 2.0 / 75.0 * fc)
    } else {
        (fc / 10.0).min(1.35 + fc / 25.0)
    };
    if long_term {
        long
    } else {
        long * 1.5
    }
}
