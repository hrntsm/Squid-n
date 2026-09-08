//! 断面形状の型定義。

/// RC 配筋の主筋セット（方向別）。
///
/// `count`: 本数, `dia`: 径 [mm], `layers`: 段数。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BarSet {
    pub count: u32,
    pub dia: f64,
    pub layers: u32,
}

/// RC せん断補強筋。
///
/// `dia`: 径 [mm], `pitch`: ピッチ [mm], `legs`: 組数。
///
/// 材質は形状ではなく断面が持つ（`crate::model::Section::shear_rebar_material`）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ShearBar {
    pub dia: f64,
    pub pitch: f64,
    pub legs: u32,
}

/// RC 配筋情報。
///
/// `main_x`: せい方向（X）主筋, `main_y`: 幅方向（Y）主筋,
/// `cover`: かぶり [mm], `shear`: せん断補強筋。
///
/// 材質は形状ではなく断面が持つ（`crate::model::Section::rebar_material`・
/// `crate::model::Section::shear_rebar_material`）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RcRebar {
    pub main_x: BarSet,
    pub main_y: BarSet,
    pub cover: f64,
    pub shear: ShearBar,
}

/// Parametric section shape definition.
///
/// Each variant carries the minimal parameters needed to define the geometry.
/// Call `to_section()` to compute the derived section properties (A, Iy, Iz, J, ...)
/// and produce a `squid_n_core::Section`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum SectionShape {
    /// Steel H‑shape (H形鋼).
    SteelH {
        height: f64,
        width: f64,
        web_thick: f64,
        flange_thick: f64,
    },
    /// Steel rectangular hollow section / box (角形鋼管).
    SteelBox {
        height: f64,
        width: f64,
        thick: f64,
        /// 角部外半径 r [mm]。0 は角部を直角とみなす。
        #[serde(default)]
        corner_r: f64,
    },
    /// Steel L‑angle (山形鋼).
    SteelAngle { leg_a: f64, leg_b: f64, thick: f64 },
    /// Steel C‑channel (溝形鋼).
    SteelChannel {
        height: f64,
        width: f64,
        web_thick: f64,
        flange_thick: f64,
    },
    /// Steel T‑shape (T形鋼).
    SteelTee {
        height: f64,
        width: f64,
        web_thick: f64,
        flange_thick: f64,
    },
    /// Steel round pipe (鋼管).
    SteelPipe { outer_dia: f64, thick: f64 },
    /// Steel flat bar / plate (平鋼・鋼板). 中実矩形。
    ///
    /// `width`: 幅 B [mm]（Z 方向）、`thick`: 板厚 t [mm]（Y 方向＝せい）。
    /// 部材のせい/幅の向きは局所座標（`ref_vector`）で与える。
    SteelFlatBar { width: f64, thick: f64 },
    /// Steel solid round bar (中実丸鋼).
    ///
    /// `dia`: 直径 D [mm]。
    SteelRoundBar { dia: f64 },
    /// Steel welded built-up H with unequal flanges (非対称組立 H 形鋼). `StbSecBuild-H`。
    ///
    /// 上下フランジの幅・厚が異なる溶接組立断面。`height`: せい H（外〜外）、
    /// `upper_width`/`upper_thick`: 上フランジ、`lower_width`/`lower_thick`: 下フランジ、
    /// `web_thick`: ウェブ厚。
    SteelBuiltH {
        height: f64,
        upper_width: f64,
        upper_thick: f64,
        lower_width: f64,
        lower_thick: f64,
        web_thick: f64,
    },
    /// Steel cold-formed lipped channel (リップ溝形鋼). `StbSecRoll-LipC`。
    ///
    /// `height`: せい H [mm]（Y 方向）、`width`: フランジ幅 B [mm]（Z 方向。ウェブ外面〜
    /// フランジ先端）、`lip`: リップ長 C [mm]（Y 方向）、`thick`: 板厚 t [mm]（全要素一様）。
    SteelLipChannel {
        height: f64,
        width: f64,
        lip: f64,
        thick: f64,
    },
    /// Reinforced concrete rectangle (RC 矩形).
    RcRect { b: f64, d: f64, rebar: RcRebar },
    /// Reinforced concrete circle column (RC 円形).
    RcCircle { d: f64, rebar: RcRebar },
    /// SRC 矩形断面（RC 矩形 + 内蔵 H 形鉄骨）。
    ///
    /// 内蔵鉄骨の鋼種・コンクリート強度・主筋の材質は、いずれも断面が材料として持つ。
    SrcRect {
        b: f64,
        d: f64,
        rebar: RcRebar,
        steel_height: f64,
        steel_width: f64,
        steel_web_thick: f64,
        steel_flange_thick: f64,
    },
    /// CFT 角形（角形鋼管 + 充填コンクリート）。検定では `Material.fc` の充填コンクリート強度を用いる。
    CftBox { height: f64, width: f64, thick: f64 },
    /// CFT 円形（円形鋼管 + 充填コンクリート）。
    CftPipe { outer_dia: f64, thick: f64 },
    /// RC 耐震壁（壁エレメント用）。
    ///
    /// `thickness`: 壁板厚 [mm]、`ps`: 壁板の直交する各方向のせん断補強筋比の
    /// うち小さい方（小数。例 0.0025）。
    RcWall { thickness: f64, ps: f64 },
    /// RC スラブ（床）。
    ///
    /// `thickness`: 板厚 [mm]。
    RcSlab { thickness: f64 },
}

impl SectionShape {
    /// 配筋情報（主筋・せん断補強筋）を持つ形状はその参照を返す。
    /// 配筋を持たない形状（鋼断面・CFT・壁）は `None`。
    pub fn rebar(&self) -> Option<&RcRebar> {
        match self {
            SectionShape::RcRect { rebar, .. }
            | SectionShape::RcCircle { rebar, .. }
            | SectionShape::SrcRect { rebar, .. } => Some(rebar),
            _ => None,
        }
    }

    /// コンクリート系（RC / SRC / CFT）の断面形状か。
    pub fn is_concrete_like(&self) -> bool {
        matches!(
            self,
            SectionShape::RcRect { .. }
                | SectionShape::RcCircle { .. }
                | SectionShape::SrcRect { .. }
                | SectionShape::CftBox { .. }
                | SectionShape::CftPipe { .. }
                | SectionShape::RcWall { .. }
                | SectionShape::RcSlab { .. }
        )
    }
}

/// 主筋 1 本あたりの断面積 [mm²]。
pub fn one_bar_area(dia: f64) -> f64 {
    let r = dia / 2.0;
    std::f64::consts::PI * r * r
}

/// 主筋セットの総断面積 [mm²]。
pub fn bar_set_area(bs: &BarSet) -> f64 {
    bs.count as f64 * one_bar_area(bs.dia)
}

/// せん断補強筋 1 組（`legs` 本）の断面積 [mm²]。
pub fn shear_legs_area(shear: &ShearBar) -> f64 {
    shear.legs as f64 * one_bar_area(shear.dia)
}
