//! 断面形状の型定義。

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
    /// RC 梁（実配筋）。`b`: 幅 [mm], `d`: せい [mm]。
    RcBeamRect { b: f64, d: f64, rebar: RcBeamRebar },
    /// RC 矩形柱（実配筋）。
    RcColumnRect {
        b: f64,
        d: f64,
        rebar: RcRectColumnRebar,
    },
    /// RC 円形柱（実配筋）。
    RcColumnCircle { d: f64, rebar: RcCircleColumnRebar },
    /// SRC 梁（RC 梁 + 内蔵 H 形鉄骨）。
    SrcBeamRect {
        b: f64,
        d: f64,
        rebar: RcBeamRebar,
        steel_height: f64,
        steel_width: f64,
        steel_web_thick: f64,
        steel_flange_thick: f64,
    },
    /// SRC 矩形柱（RC 矩形柱 + 内蔵 H 形鉄骨）。
    SrcColumnRect {
        b: f64,
        d: f64,
        rebar: RcRectColumnRebar,
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
    /// 梁の実配筋を持つ形状はその参照を返す。
    pub fn beam_rebar(&self) -> Option<&RcBeamRebar> {
        match self {
            SectionShape::RcBeamRect { rebar, .. } | SectionShape::SrcBeamRect { rebar, .. } => {
                Some(rebar)
            }
            _ => None,
        }
    }

    /// 矩形柱の実配筋を持つ形状はその参照を返す。
    pub fn rect_column_rebar(&self) -> Option<&RcRectColumnRebar> {
        match self {
            SectionShape::RcColumnRect { rebar, .. }
            | SectionShape::SrcColumnRect { rebar, .. } => Some(rebar),
            _ => None,
        }
    }

    /// 円形柱の実配筋を持つ形状はその参照を返す。
    pub fn circle_column_rebar(&self) -> Option<&RcCircleColumnRebar> {
        match self {
            SectionShape::RcColumnCircle { rebar, .. } => Some(rebar),
            _ => None,
        }
    }

    /// 実配筋型（`RcBeamRect`・`RcColumnRect`・`RcColumnCircle`・`SrcBeamRect`・
    /// `SrcColumnRect`）の幾何検証。実鉄筋座標を生成できない場合は
    /// [`crate::error::RebarGeometryError`] を返す。旧型・配筋なし形状は `Ok(())`。
    pub fn validate_rebar(&self) -> Result<(), crate::error::RebarGeometryError> {
        match self {
            SectionShape::RcBeamRect { b, d, rebar } => rebar.validate(*b, *d),
            SectionShape::RcColumnRect { b, d, rebar } => rebar.validate(*b, *d),
            SectionShape::RcColumnCircle { d, rebar } => rebar.validate(*d),
            SectionShape::SrcBeamRect { b, d, rebar, .. } => rebar.validate(*b, *d),
            SectionShape::SrcColumnRect { b, d, rebar, .. } => rebar.validate(*b, *d),
            _ => Ok(()),
        }
    }

    /// コンクリート系（RC / SRC / CFT）の断面形状か。
    pub fn is_concrete_like(&self) -> bool {
        matches!(
            self,
            SectionShape::RcBeamRect { .. }
                | SectionShape::RcColumnRect { .. }
                | SectionShape::RcColumnCircle { .. }
                | SectionShape::SrcBeamRect { .. }
                | SectionShape::SrcColumnRect { .. }
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
/// RC 梁の実配筋。
///
/// `main_dia`: 主筋径 [mm]（全段共通）。`top`/`bottom`: 上端筋・下端筋の
/// かぶり側から内側へ数えた段別本数。`cover`: かぶり [mm]。`stirrup`: あばら筋。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RcBeamRebar {
    pub main_dia: f64,
    pub top: Vec<u32>,
    pub bottom: Vec<u32>,
    pub cover: f64,
    pub stirrup: BeamStirrup,
}

impl RcBeamRebar {
    /// 上端筋・下端筋とも未入力（段が空）か。
    pub fn is_unset(&self) -> bool {
        self.top.is_empty() && self.bottom.is_empty()
    }
}

/// RC 矩形柱の実配筋。
///
/// `main_dia`: 主筋径 [mm]（全段共通）。`x`/`y`: X 方向・Y 方向の段別本数。
/// 各段は断面中心対称な 2 本の配筋線を 1 段とし、本数は片側 1 線あたり。
/// `cover`: かぶり [mm]。`hoop`: 帯筋。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RcRectColumnRebar {
    pub main_dia: f64,
    pub x: Vec<u32>,
    pub y: Vec<u32>,
    pub cover: f64,
    pub hoop: RectColumnHoop,
}

impl RcRectColumnRebar {
    /// X 方向・Y 方向とも未入力（段が空）か。
    pub fn is_unset(&self) -> bool {
        self.x.is_empty() && self.y.is_empty()
    }
}

/// RC 円形柱の実配筋。
///
/// `main_dia`: 主筋径 [mm]。`count`: 主筋総本数（円周上へ等間隔配置）。
/// `cover`: かぶり [mm]。`hoop`: 帯筋。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RcCircleColumnRebar {
    pub main_dia: f64,
    pub count: u32,
    pub cover: f64,
    pub hoop: CircleColumnHoop,
}

impl RcCircleColumnRebar {
    /// 主筋総本数が未入力（0）か。
    pub fn is_unset(&self) -> bool {
        self.count == 0
    }
}

/// RC 梁のあばら筋。
///
/// `dia`: 径 [mm]。`pitch`: 間隔 [mm]。`legs`: 脚数。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BeamStirrup {
    pub dia: f64,
    pub pitch: f64,
    pub legs: u32,
}

/// RC 矩形柱の帯筋。
///
/// `dia`: 径 [mm]。`pitch`: 間隔 [mm]。`legs_x`/`legs_y`: X 方向・Y 方向の脚数。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RectColumnHoop {
    pub dia: f64,
    pub pitch: f64,
    pub legs_x: u32,
    pub legs_y: u32,
}

/// RC 円形柱の帯筋。
///
/// `dia`: 径 [mm]。`pitch`: 間隔 [mm]。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CircleColumnHoop {
    pub dia: f64,
    pub pitch: f64,
}

/// 主筋 1 本の断面内座標。
///
/// `x`: 幅 b 方向 [mm]、`y`: せい d 方向 [mm]。断面中心を原点とする。
/// 段別配筋から都度生成する派生値であり、モデルへは保存しない。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RebarPoint {
    pub x: f64,
    pub y: f64,
}
