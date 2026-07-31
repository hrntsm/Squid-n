//! 部材の構造種別（RC/SRC 系・S/CFT 系）の判定。
//!
//! 剛域長の算定式・仕口パネルのモデル化対象・S 造パネルゾーンの検定対象は、
//! いずれも「その部材が RC/SRC 系か S/CFT 系か」で分岐する。判定が箇所ごとに
//! ずれると、剛域長 0 の接合部にパネルが設けられない（またはその逆）といった
//! 食い違いが生じるため、判定を本モジュールへ一元化する。
//!
//! # 判定の順序
//!
//! 1. 断面に形状（[`SectionShape`]）があれば形状で判定する
//! 2. 形状が無い（カタログ数値の直入力など）場合は材料で判定する。
//!    コンクリート設計基準強度 `fc` があれば RC/SRC 系、降伏応力 `fy` のみなら
//!    S/CFT 系
//! 3. どちらも無ければ判定材料が無いため RC/SRC 系とする（剛域式を変えない側）

use crate::model::{ElementData, Model};
use crate::section_shape::SectionShape;

/// 部材の構造種別（技術基準解説書「剛域の計算」の RC/SRC 系・S 系区分）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StructureKind {
    /// RC・SRC 系（RC 造柱・梁・耐震壁、SRC 造柱・梁）。
    RcSrc,
    /// S・CFT 系（CFT は S 造と同様に扱う）。
    Steel,
}

/// 断面形状から構造種別を判定する。
pub fn shape_structure_kind(shape: &SectionShape) -> StructureKind {
    match shape {
        SectionShape::RcRect { .. }
        | SectionShape::RcCircle { .. }
        | SectionShape::RcWall { .. }
        | SectionShape::SrcRect { .. } => StructureKind::RcSrc,
        SectionShape::SteelH { .. }
        | SectionShape::SteelBox { .. }
        | SectionShape::SteelAngle { .. }
        | SectionShape::SteelChannel { .. }
        | SectionShape::SteelTee { .. }
        | SectionShape::SteelPipe { .. }
        | SectionShape::SteelFlatBar { .. }
        | SectionShape::SteelRoundBar { .. }
        | SectionShape::SteelLipChannel { .. }
        | SectionShape::SteelBuiltH { .. }
        | SectionShape::CftBox { .. }
        | SectionShape::CftPipe { .. } => StructureKind::Steel,
    }
}

/// 要素の構造種別を判定する（モジュール冒頭「判定の順序」）。
pub fn member_structure_kind(model: &Model, elem: &ElementData) -> StructureKind {
    let sec = elem.section.and_then(|sid| model.sections.get(sid.index()));
    if let Some(shape) = sec.and_then(|s| s.shape.as_ref()) {
        return shape_structure_kind(shape);
    }
    let mat = elem
        .material
        .and_then(|mid| model.materials.get(mid.index()));
    if let Some(mat) = mat {
        if mat.fc.is_some() {
            return StructureKind::RcSrc;
        }
        if mat.fy.is_some() {
            return StructureKind::Steel;
        }
    }
    StructureKind::RcSrc
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{ElemId, MaterialId, SectionId};
    use crate::model::{ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Section};
    use crate::section_shape::{BarSet, RcRebar, ShearBar};

    fn material(name: &str, fc: Option<f64>, fy: Option<f64>) -> Material {
        Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: name.into(),
            young: 205_000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc,
            fy,
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
            panel_thickness: None,
            thickness: None,
            shape,
        }
    }

    fn elem() -> ElementData {
        ElementData {
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
        }
    }

    fn model_with(shape: Option<SectionShape>, mat: Material) -> Model {
        Model {
            sections: vec![section(shape)],
            materials: vec![mat],
            elements: vec![elem()],
            ..Default::default()
        }
    }

    fn rc_rect() -> SectionShape {
        let bars = BarSet {
            dia: 25.0,
            count: 4,
            layers: 1,
        };
        SectionShape::RcRect {
            b: 700.0,
            d: 700.0,
            rebar: RcRebar {
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
            },
        }
    }

    /// 断面形状があれば形状で判定する（材料は見ない）。
    #[test]
    fn test_shape_wins_over_material() {
        // 形状は S だが材料は fc を持つ（形状優先で Steel）。
        let m = model_with(
            Some(SectionShape::SteelH {
                height: 400.0,
                width: 200.0,
                web_thick: 8.0,
                flange_thick: 13.0,
            }),
            material("SN400B", Some(24.0), None),
        );
        assert_eq!(
            member_structure_kind(&m, &m.elements[0]),
            StructureKind::Steel
        );

        let m = model_with(Some(rc_rect()), material("SN400B", None, Some(235.0)));
        assert_eq!(
            member_structure_kind(&m, &m.elements[0]),
            StructureKind::RcSrc
        );
    }

    /// CFT は S 系として扱う（剛域式・パネルの構造種別判定とも）。
    #[test]
    fn test_cft_is_steel() {
        for shape in [
            SectionShape::CftBox {
                height: 400.0,
                width: 400.0,
                thick: 16.0,
            },
            SectionShape::CftPipe {
                outer_dia: 400.0,
                thick: 12.0,
            },
        ] {
            assert_eq!(shape_structure_kind(&shape), StructureKind::Steel);
        }
    }

    /// 形状が無い断面は材料で判定する（fc → RC/SRC、fy のみ → S）。
    #[test]
    fn test_material_fallback() {
        let m = model_with(None, material("FC24", Some(24.0), None));
        assert_eq!(
            member_structure_kind(&m, &m.elements[0]),
            StructureKind::RcSrc
        );

        let m = model_with(None, material("SN400B", None, Some(235.0)));
        assert_eq!(
            member_structure_kind(&m, &m.elements[0]),
            StructureKind::Steel
        );
    }

    /// 形状も材料の判定材料も無い場合は RC/SRC 扱い（剛域式を変えない側）。
    #[test]
    fn test_unknown_defaults_to_rc_src() {
        let m = model_with(None, material("UNKNOWN", None, None));
        assert_eq!(
            member_structure_kind(&m, &m.elements[0]),
            StructureKind::RcSrc
        );
    }
}
