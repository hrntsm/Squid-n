//! 部材の構造種別（RC・S・SRC・CFT）の判定。
//!
//! 剛域長の算定式・仕口パネルのモデル化対象・断面検定で用いる式・略算周期の
//! 構造種別・数量集計の分類は、いずれも「その部材が何造か」で分岐する。判定が
//! 箇所ごとにずれると、剛域長 0 の接合部にパネルが設けられない、鋼部材が RC の
//! 検定式で検定される、といった食い違いが生じるため、判定を本モジュールへ
//! 一元化する。
//!
//! # 判定の順序
//!
//! 1. 断面形状が**複合断面**なら、その種別で決まる
//!    - `SrcRect` → [`StructureKind::Src`]
//!    - `CftBox` / `CftPipe` → [`StructureKind::Cft`]
//! 2. それ以外は**材料の区分**（[`MaterialCategory`]）で決まる
//!    - `Steel` → [`StructureKind::S`]
//!    - `Concrete` / `Rebar` → [`StructureKind::Rc`]
//! 3. 材料が解決できない場合だけ、断面形状の系統で補う（[`shape_default_kind`]）
//! 4. 断面形状もなければ [`StructureKind::Rc`] とする
//!
//! # 断面形状ではなく材料で判定する理由
//!
//! 断面形状は見た目であって力学的な性質ではない。H 形のコンクリート部材も、
//! 矩形断面の鋼部材もありうる。材料の区分で判定すれば、任意の材料と任意の断面の
//! 組み合わせに対して、どの検定式を適用すべきかが定まる。
//!
//! SRC・CFT だけを断面形状で判定するのは、これらが 1 つの材料では表せない
//! **複合断面**だからである。`SrcRect` は内蔵鉄骨のグレードを断面側に持ち、
//! CFT は `Material::fc` を充填コンクリートの強度として使う。
//!
//! # 用途ごとの畳み込み
//!
//! 4 種別をそのまま使うのは断面検定と数量集計で、他の用途はより粗い区分へ
//! 畳み込む。畳み込みは本モジュールのメソッドとして定義し、各所が `matches!` で
//! 書き下すのを避ける。`SectionShape` にバリアントが増えたとき、追随が必要なのは
//! [`shape_composite_kind`] の網羅 `match` 1 箇所だけになる。
//!
//! | 用途 | 畳み込み |
//! |---|---|
//! | 剛域長の算定式・仕口パネルの対象 | [`StructureKind::is_steel_like`]（S・CFT） |
//! | 略算周期の構造種別 | `StoryStructure`（CFT は SRC へ寄せる） |
//! | 断面検定の式の選択・数量集計 | 4 種別をそのまま使う |

use crate::model::{ElementData, MaterialCategory, Model, Section};
use crate::section_shape::SectionShape;

/// 部材の構造種別。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StructureKind {
    /// 鉄筋コンクリート造。
    Rc,
    /// 鉄骨造。
    S,
    /// 鉄骨鉄筋コンクリート造。
    Src,
    /// コンクリート充填鋼管造。
    Cft,
}

impl StructureKind {
    /// 表示名。
    pub fn label(self) -> &'static str {
        match self {
            StructureKind::Rc => "RC",
            StructureKind::S => "S",
            StructureKind::Src => "SRC",
            StructureKind::Cft => "CFT",
        }
    }

    /// 鋼系（S・CFT）か。
    ///
    /// 剛域長の算定式（S・CFT 造は `D_self/4` を控除しない）と、仕口パネルの
    /// 対象判定に用いる区分。CFT を S と同じ側へ置くのは、いずれも接合部の
    /// 剛域長が 0 になり、接合部の有限寸法を剛域で評価しないためである。
    pub fn is_steel_like(self) -> bool {
        matches!(self, StructureKind::S | StructureKind::Cft)
    }
}

/// 断面形状が複合断面（SRC・CFT）なら、その構造種別を返す。
///
/// 単一材料で表せる形状は `None` を返し、呼び出し側が材料の区分で判定する。
/// `SectionShape` にバリアントが増えたときに追随が要るのはこの網羅 `match` と
/// [`shape_default_kind`] だけで、追随を忘れるとコンパイルエラーになる。
pub fn shape_composite_kind(shape: &SectionShape) -> Option<StructureKind> {
    match shape {
        SectionShape::SrcRect { .. } => Some(StructureKind::Src),
        SectionShape::CftBox { .. } | SectionShape::CftPipe { .. } => Some(StructureKind::Cft),
        SectionShape::RcRect { .. }
        | SectionShape::RcCircle { .. }
        | SectionShape::RcWall { .. }
        | SectionShape::SteelH { .. }
        | SectionShape::SteelBox { .. }
        | SectionShape::SteelAngle { .. }
        | SectionShape::SteelChannel { .. }
        | SectionShape::SteelTee { .. }
        | SectionShape::SteelPipe { .. }
        | SectionShape::SteelFlatBar { .. }
        | SectionShape::SteelRoundBar { .. }
        | SectionShape::SteelLipChannel { .. }
        | SectionShape::SteelBuiltH { .. } => None,
    }
}

/// 材料が解決できないときに断面形状から補う構造種別。
///
/// 判定の主は材料の区分だが、材料が未割当の部材まで一律 RC とすると、材料を
/// 付け忘れた鋼部材に RC の剛域が入って架構が硬くなる。形状名の系統は入力の
/// 意図をよく表すため、材料がないときに限ってこれを既定として採る。
pub fn shape_default_kind(shape: &SectionShape) -> StructureKind {
    match shape {
        SectionShape::SrcRect { .. } => StructureKind::Src,
        SectionShape::CftBox { .. } | SectionShape::CftPipe { .. } => StructureKind::Cft,
        SectionShape::RcRect { .. }
        | SectionShape::RcCircle { .. }
        | SectionShape::RcWall { .. } => StructureKind::Rc,
        SectionShape::SteelH { .. }
        | SectionShape::SteelBox { .. }
        | SectionShape::SteelAngle { .. }
        | SectionShape::SteelChannel { .. }
        | SectionShape::SteelTee { .. }
        | SectionShape::SteelPipe { .. }
        | SectionShape::SteelFlatBar { .. }
        | SectionShape::SteelRoundBar { .. }
        | SectionShape::SteelLipChannel { .. }
        | SectionShape::SteelBuiltH { .. } => StructureKind::S,
    }
}

/// 材料の区分から構造種別を求める。
///
/// 鉄筋は材料としては鋼だが、これを割り当てた線材は S 造ではないため RC とする
/// （RC 断面の配筋は断面側にグレード名として持ち、線材の材料として鉄筋を
/// 割り当てるのは入力の誤り）。
pub fn material_structure_kind(category: MaterialCategory) -> StructureKind {
    match category {
        MaterialCategory::Steel => StructureKind::S,
        MaterialCategory::Concrete | MaterialCategory::Rebar => StructureKind::Rc,
    }
}

/// 断面と材料から構造種別を判定する（モジュール冒頭「判定の順序」）。
pub fn structure_kind_of(
    sec: Option<&Section>,
    category: Option<MaterialCategory>,
) -> StructureKind {
    let shape = sec.and_then(|s| s.shape.as_ref());
    if let Some(kind) = shape.and_then(shape_composite_kind) {
        return kind;
    }
    match category {
        Some(category) => material_structure_kind(category),
        // 材料が解決できない場合は断面形状の系統で補い、形状もなければ RC とする。
        None => shape.map_or(StructureKind::Rc, shape_default_kind),
    }
}

/// 要素の構造種別を判定する。
pub fn member_structure_kind(model: &Model, elem: &ElementData) -> StructureKind {
    let sec = elem.section.and_then(|sid| model.sections.get(sid.index()));
    let category = elem
        .material
        .and_then(|mid| model.materials.get(mid.index()))
        .map(|m| m.category);
    structure_kind_of(sec, category)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{ElemId, MaterialId, SectionId};
    use crate::model::{ElementKind, EndCondition, ForceRegime, LocalAxis, Material};

    fn material(category: MaterialCategory) -> Material {
        Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: String::new(),
            category,
            young: 205_000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }
    }

    fn section(shape: Option<SectionShape>) -> Section {
        Section {
            id: SectionId(0),
            name: String::new(),
            area: 1.0e4,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e7,
            depth: 400.0,
            width: 400.0,
            as_y: 4.0e3,
            as_z: 4.0e3,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape,
        }
    }

    fn model_with(shape: Option<SectionShape>, category: MaterialCategory) -> Model {
        Model {
            sections: vec![section(shape)],
            materials: vec![material(category)],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![crate::ids::NodeId(0), crate::ids::NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 1.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            }],
            ..Default::default()
        }
    }

    fn h_shape() -> SectionShape {
        SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 8.0,
            flange_thick: 13.0,
        }
    }

    fn rect_shape() -> SectionShape {
        SectionShape::SteelBox {
            height: 400.0,
            width: 400.0,
            thick: 16.0,
            corner_r: 0.0,
        }
    }

    fn rc_rebar() -> crate::section_shape::RcRebar {
        use crate::section_shape::{BarSet, RcRebar, ShearBar};
        let bars = BarSet {
            dia: 25.0,
            count: 4,
            layers: 1,
        };
        RcRebar {
            main_x: bars.clone(),
            main_y: bars,
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
            main_grade: None,
        }
    }

    /// 断面形状ではなく材料の区分で判定する。
    /// H 形のコンクリート部材・矩形断面の鋼部材のいずれも正しく分類できる。
    #[test]
    fn test_material_decides_kind_not_shape() {
        let m = model_with(Some(h_shape()), MaterialCategory::Concrete);
        assert_eq!(
            member_structure_kind(&m, &m.elements[0]),
            StructureKind::Rc,
            "H 形でも材料がコンクリートなら RC"
        );

        let m = model_with(Some(rect_shape()), MaterialCategory::Steel);
        assert_eq!(
            member_structure_kind(&m, &m.elements[0]),
            StructureKind::S,
            "矩形でも材料が鋼材なら S"
        );
    }

    /// 断面形状を持たない断面（カタログ数値の直入力）でも材料で判定できる。
    #[test]
    fn test_shapeless_section_uses_material() {
        let m = model_with(None, MaterialCategory::Steel);
        assert_eq!(member_structure_kind(&m, &m.elements[0]), StructureKind::S);

        let m = model_with(None, MaterialCategory::Concrete);
        assert_eq!(member_structure_kind(&m, &m.elements[0]), StructureKind::Rc);
    }

    /// 複合断面は材料に依らず断面形状で決まる。
    #[test]
    fn test_composite_shape_wins_over_material() {
        let src = SectionShape::SrcRect {
            b: 700.0,
            d: 700.0,
            rebar: rc_rebar(),
            steel_height: 400.0,
            steel_width: 200.0,
            steel_web_thick: 8.0,
            steel_flange_thick: 13.0,
            steel_grade: "SN400B".into(),
        };
        let m = model_with(Some(src), MaterialCategory::Steel);
        assert_eq!(
            member_structure_kind(&m, &m.elements[0]),
            StructureKind::Src
        );

        let cft = SectionShape::CftBox {
            height: 400.0,
            width: 400.0,
            thick: 16.0,
        };
        let m = model_with(Some(cft), MaterialCategory::Concrete);
        assert_eq!(
            member_structure_kind(&m, &m.elements[0]),
            StructureKind::Cft
        );
    }

    /// 鉄筋は材料としては鋼だが、割り当てた線材は S 造ではない。
    #[test]
    fn test_rebar_is_not_steel_structure() {
        let m = model_with(Some(h_shape()), MaterialCategory::Rebar);
        assert_eq!(member_structure_kind(&m, &m.elements[0]), StructureKind::Rc);
    }

    /// 材料が未割当の部材は断面形状の系統で補う。
    /// 材料を付け忘れた鋼部材に RC の剛域が入らないようにするための既定。
    #[test]
    fn test_missing_material_falls_back_to_shape() {
        let mut m = model_with(Some(h_shape()), MaterialCategory::Steel);
        m.elements[0].material = None;
        assert_eq!(member_structure_kind(&m, &m.elements[0]), StructureKind::S);

        let rc = SectionShape::RcRect {
            b: 700.0,
            d: 700.0,
            rebar: rc_rebar(),
        };
        let mut m = model_with(Some(rc), MaterialCategory::Steel);
        m.elements[0].material = None;
        assert_eq!(member_structure_kind(&m, &m.elements[0]), StructureKind::Rc);
    }

    /// 断面形状も材料もない部材は RC とする。
    #[test]
    fn test_no_section_no_material_is_rc() {
        let mut m = model_with(None, MaterialCategory::Steel);
        m.elements[0].material = None;
        m.elements[0].section = None;
        assert_eq!(member_structure_kind(&m, &m.elements[0]), StructureKind::Rc);
    }

    /// 剛域式・仕口パネルの判定に使う畳み込みは S と CFT を同じ側へ置く。
    #[test]
    fn test_steel_like_folds_s_and_cft() {
        assert!(StructureKind::S.is_steel_like());
        assert!(StructureKind::Cft.is_steel_like());
        assert!(!StructureKind::Rc.is_steel_like());
        assert!(!StructureKind::Src.is_steel_like());
    }
}
