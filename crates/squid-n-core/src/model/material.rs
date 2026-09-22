//! 材料の型。

use super::*;

/// 材料の区分。
///
/// 部材が S 造か RC 造かは、断面形状ではなくこの区分で判定する。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum MaterialCategory {
    /// 構造用鋼材。
    Steel,
    /// 鉄筋。RC 部材の主筋・せん断補強筋に用いる。
    Rebar,
    /// コンクリート。
    Concrete,
}

impl MaterialCategory {
    /// UI 表示名。
    pub fn label(&self) -> &'static str {
        match self {
            MaterialCategory::Steel => "鋼材",
            MaterialCategory::Rebar => "鉄筋",
            MaterialCategory::Concrete => "コンクリート",
        }
    }
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Material {
    pub id: MaterialId,
    pub name: String,
    /// 材料の区分。
    pub category: MaterialCategory,
    pub young: f64,
    pub poisson: f64,
    pub density: f64,
    #[serde(default)]
    pub shear: Option<f64>,
    /// コンクリート設計基準強度 Fc [N/mm²]。鋼材では `None`。
    #[serde(default)]
    pub fc: Option<f64>,
    /// 降伏応力 fy [N/mm²]。
    /// `None` の場合、ファイバ材料は弾性（降伏しない）として扱う。
    #[serde(default)]
    pub fy: Option<f64>,
    /// コンクリートの種類（普通/軽量1種/軽量2種）。鋼材では意味を持たない（既定 Normal）。
    #[serde(default)]
    pub concrete_class: crate::units::ConcreteClass,
    /// 保有水平耐力計算用の材料強度割増係数の直接入力。
    /// `None`（既定）の場合は自動判定。
    #[serde(default)]
    pub strength_factor: Option<f64>,
}

impl Material {
    pub fn shear_modulus(&self) -> f64 {
        self.shear
            .unwrap_or_else(|| self.young / (2.0 * (1.0 + self.poisson)))
    }

    /// 固定荷重（DL・地震用重量）算定に用いる単位体積重量 [N/mm³]。
    /// 鋼材は基準資料の 78.5 kN/m³、その他は質量密度×g（内部単位 N-mm-s）。
    ///
    /// 鉄筋は鋼材に含めない。RC/SRC の主材料はコンクリートであり、鉄筋の自重は
    /// コンクリートの単位体積重量（γRC/γSRC）に内包されるため別加算しない。
    pub fn design_unit_weight_n_per_mm3(&self) -> f64 {
        if self.category == MaterialCategory::Steel {
            crate::units::to_internal::unit_weight_kn_per_m3(crate::units::STEEL_UNIT_WEIGHT_KN_M3)
        } else {
            self.density * crate::units::GRAVITY_MM_S2
        }
    }
}
