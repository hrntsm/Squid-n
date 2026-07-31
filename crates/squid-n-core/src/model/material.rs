//! 材料の型。
//!
//! - [`MaterialCategory`] — 材料の区分（鋼材・鉄筋・コンクリート）。
//! - [`Material`] — 材料（区分・ヤング率・ポアソン比・密度・強度等）。

use super::*;

/// 材料の区分。
///
/// 部材が S 造か RC 造かは、断面形状ではなく**この区分で判定する**
/// （[`crate::structure_kind`]）。断面形状は見た目であって力学的な性質ではなく、
/// H 形のコンクリート部材・矩形断面の鋼部材のいずれもありうるためである。
/// 任意の材料と任意の断面の組み合わせでも、どの検定式を適用すべきかが定まる。
///
/// SRC・CFT は 1 つの材料では表せない複合断面なので、断面形状の側で判定する
/// （`SrcRect` は内蔵鉄骨のグレードを断面に持ち、CFT は `Material::fc` を
/// 充填コンクリートの強度として使う）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum MaterialCategory {
    /// 構造用鋼材。
    Steel,
    /// 鉄筋。RC 部材の主筋・せん断補強筋に用いる。
    ///
    /// 材料としては鋼だが、これを割り当てた線材は S 造ではない。RC 断面の配筋は
    /// 断面側（`RcRebar`）にグレード名として持つため、線材の材料として
    /// 鉄筋を割り当てるのは入力の誤りである。
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
    /// 材料の区分。S 造・RC 造の判定はこの値による（[`MaterialCategory`]）。
    pub category: MaterialCategory,
    pub young: f64,
    pub poisson: f64,
    pub density: f64,
    #[serde(default)]
    pub shear: Option<f64>,
    /// コンクリート設計基準強度 Fc [N/mm²]。
    /// 鋼材では `None`。RC 設計（令91条）の許容圧縮・せん断に用いる。
    #[serde(default)]
    pub fc: Option<f64>,
    /// 降伏応力 fy [N/mm²]。鋼材の弾塑性挙動（ファイバ材料・端ばねスケルトン）に用いる。
    /// `None` の場合、ファイバ材料は弾性（降伏しない）として扱う（P5 非線形）。
    #[serde(default)]
    pub fy: Option<f64>,
    /// コンクリートの種類（普通/軽量1種/軽量2種）。固定荷重の
    /// 単位体積重量表・許容応力度低減（軽量コンクリートは
    /// 普通コンクリートの 0.9 倍。技術基準解説書）に用いる。鋼材では意味を持たない（既定 Normal）。
    /// 旧スキーマ（フィールド無し）は Normal 扱い。
    #[serde(default)]
    pub concrete_class: crate::units::ConcreteClass,
    /// 保有水平耐力計算（プッシュオーバー）用の材料強度割増係数の直接入力。
    /// `None`（既定）の場合は自動判定（鋼材グレードは 1.1、590N 級の
    /// SA440・TMCP440 は 1.05、RC 主筋は 1.1。せん断補強筋は割増対象外）。
    /// 直接入力材料など自動判定できない材料に対して割増を指定する用途。
    /// 許容応力度計算（一次設計）には影響しない。
    /// 旧スキーマ（フィールド無し）は None（自動判定）扱い。
    #[serde(default)]
    pub strength_factor: Option<f64>,
}

impl Material {
    pub fn shear_modulus(&self) -> f64 {
        self.shear
            .unwrap_or_else(|| self.young / (2.0 * (1.0 + self.poisson)))
    }
}
