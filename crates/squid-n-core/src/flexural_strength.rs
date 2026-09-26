//! 部材の曲げ降伏モーメント My の共通算定。

use crate::material_grade::rebar_yield_strength;
use crate::model::{ElementData, Material, Model, Section};
use crate::rc_capacity::{rc_mu_simple, RcCapacityInput};
use crate::section_shape::SectionShape;

/// 曲げ降伏 My 算定に用いる材料強度係数。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlexuralStrengthFactors {
    /// 鋼材（部材材料）の σy 倍率。
    pub steel: f64,
    /// RC 主筋の σy 倍率。
    pub rebar: f64,
}

impl FlexuralStrengthFactors {
    /// 公称値（割増なし）。
    pub const NOMINAL: Self = Self {
        steel: 1.0,
        rebar: 1.0,
    };
}

/// 断面の弾性断面係数 Ze [mm³]。
pub fn section_elastic_modulus(sec: &Section) -> f64 {
    let depth = sec.depth.max(sec.width);
    let i_gross = sec.iz.max(sec.iy);
    if depth > 0.0 {
        i_gross / (depth / 2.0)
    } else {
        0.0
    }
}

/// 断面の弱軸側弾性断面係数 Ze [mm³]。
pub fn section_elastic_modulus_weak(sec: &Section) -> f64 {
    let depth = sec.depth.min(sec.width);
    let i_gross = sec.iz.min(sec.iy);
    if depth > 0.0 {
        i_gross / (depth / 2.0)
    } else {
        0.0
    }
}

/// 部材の曲げ降伏（終局）モーメント My [N·mm]。
///
/// - RC 配筋形状: `0.9·at·σy·j`（[`rc_mu_simple`]）
/// - 塑性断面係数を持つ形状: `Zp·σy`（全塑性 Mp）
/// - それ以外: `σy·Ze`（弾性断面係数フォールバック）
pub fn member_flexural_yield_moment(
    elem: &ElementData,
    model: &Model,
    factors: FlexuralStrengthFactors,
) -> f64 {
    let sec = elem.section.and_then(|sid| model.sections.get(sid.index()));
    let mat = model.element_material(elem);
    let ze = sec.map(section_elastic_modulus).unwrap_or(0.0);
    let fy = mat.and_then(|m| m.fy);
    match sec.and_then(|s| s.shape.as_ref()) {
        Some(SectionShape::RcBeamRect { b: _, d, rebar }) => {
            let bottom = rebar.bending_steel(*d, false);
            let top = rebar.bending_steel(*d, true);
            let my_bottom = rc_flexural_yield_moment(
                elem,
                model,
                mat,
                bottom.tension.area_mm2,
                bottom.tension.effective_depth_mm,
                *d,
                ze,
                factors.rebar,
            );
            let my_top = rc_flexural_yield_moment(
                elem,
                model,
                mat,
                top.tension.area_mm2,
                top.tension.effective_depth_mm,
                *d,
                ze,
                factors.rebar,
            );
            my_bottom.min(my_top)
        }
        Some(SectionShape::RcColumnRect { b, d, rebar }) => {
            let strong = rebar.edge_steel(crate::rc_rebar_geom::RectEdge::Top, *b, *d);
            let weak = rebar.edge_steel(crate::rc_rebar_geom::RectEdge::Left, *b, *d);
            let ze_strong = sec.map(section_elastic_modulus).unwrap_or(0.0);
            let ze_weak = sec.map(section_elastic_modulus_weak).unwrap_or(0.0);
            let my_strong = rc_flexural_yield_moment(
                elem,
                model,
                mat,
                strong.area_mm2,
                strong.effective_depth_mm,
                *d,
                ze_strong,
                factors.rebar,
            );
            let my_weak = rc_flexural_yield_moment(
                elem,
                model,
                mat,
                weak.area_mm2,
                weak.effective_depth_mm,
                *b,
                ze_weak,
                factors.rebar,
            );
            my_strong.min(my_weak)
        }
        Some(SectionShape::RcColumnCircle { d, rebar }) => rc_flexural_yield_moment(
            elem,
            model,
            mat,
            rebar.equivalent_tension_area_mm2(),
            rebar.equivalent_effective_depth_mm(*d),
            *d,
            ze,
            factors.rebar,
        ),
        Some(shape) => {
            let sy = fy.unwrap_or(235.0) * factors.steel;
            match shape.plastic_modulus_strong() {
                Some(zp) => sy * zp,
                None => sy * ze,
            }
        }
        None => fy.unwrap_or(235.0) * factors.steel * ze,
    }
}

#[allow(clippy::too_many_arguments)]
fn rc_flexural_yield_moment(
    elem: &ElementData,
    model: &Model,
    mat: Option<&Material>,
    at: f64,
    d_eff: f64,
    d: f64,
    ze: f64,
    rebar_factor: f64,
) -> f64 {
    let rebar_mat = model.element_rebar_material(elem);
    let sy = rebar_yield_strength(rebar_mat)
        .or_else(|| mat.and_then(|m| m.fy))
        .unwrap_or(345.0)
        * rebar_factor;
    let fc = mat.and_then(|m| m.fc).unwrap_or(0.0);
    let my = rc_mu_simple(&RcCapacityInput {
        b: 1.0,
        d,
        at,
        d_eff,
        sigma_y: sy,
        fc: fc.max(1e-9),
        pw: 0.0,
        sigma_wy: 0.0,
        clear_span: 1.0,
        sigma_0: 0.0,
    });
    if my > 0.0 {
        my
    } else {
        sy * ze
    }
}

#[cfg(test)]
mod tests {
    use super::member_flexural_yield_moment;
    use super::FlexuralStrengthFactors;
    use crate::ids::{ElemId, MaterialId, SectionId};
    use crate::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, MaterialCategory,
        Model, Node, RigidZone,
    };
    use crate::section_shape::SectionShape;
    #[test]
    fn steel_yield_uses_plastic_modulus_and_strength_factor() {
        let mut model = Model::default();
        model.materials.push(Material {
            id: MaterialId(0),
            name: "SN400".into(),
            category: MaterialCategory::Steel,
            young: 205_000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: Some(235.0),
            concrete_class: Default::default(),
            strength_factor: Some(1.1),
        });
        let mut sec = SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 9.0,
            flange_thick: 16.0,
        }
        .to_section(SectionId(0), "H-400".into());
        sec.material = Some(MaterialId(0));
        model.sections.push(sec);
        model.nodes.extend([
            Node {
                id: crate::ids::NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: crate::ids::NodeId(1),
                coord: [3000.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
        ]);
        let elem = ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![crate::ids::NodeId(0), crate::ids::NodeId(1)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        };
        let my = member_flexural_yield_moment(
            &elem,
            &model,
            FlexuralStrengthFactors {
                steel: 1.1,
                rebar: 1.0,
            },
        );
        let sec = &model.sections[0];
        let zp = sec
            .shape
            .as_ref()
            .unwrap()
            .plastic_modulus_strong()
            .unwrap();
        assert!((my - 235.0 * 1.1 * zp).abs() < 1e-3 * my.max(1.0));
    }

    #[test]
    fn rect_column_yield_uses_min_of_strong_and_weak_axes() {
        use crate::rc_rebar_geom::RectEdge;
        use crate::section_shape::{one_bar_area, RcRectColumnRebar, RectColumnHoop};
        let mut model = Model::default();
        model.materials.push(Material {
            id: MaterialId(0),
            name: "Fc24".into(),
            category: MaterialCategory::Concrete,
            young: 23_000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
            concrete_class: Default::default(),
            strength_factor: None,
        });
        let rebar = RcRectColumnRebar {
            main_dia: 25.0,
            x: vec![5],
            y: vec![2],
            cover: 40.0,
            hoop: RectColumnHoop {
                dia: 10.0,
                pitch: 100.0,
                legs_x: 2,
                legs_y: 2,
            },
        };
        let (b, d) = (400.0, 600.0);
        let mut sec = SectionShape::RcColumnRect {
            b,
            d,
            rebar: rebar.clone(),
        }
        .to_section(SectionId(0), "C1".into());
        sec.material = Some(MaterialId(0));
        model.sections.push(sec);
        model.nodes.extend([
            Node {
                id: crate::ids::NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: crate::ids::NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
        ]);
        let elem = ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![crate::ids::NodeId(0), crate::ids::NodeId(1)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        };
        let my = member_flexural_yield_moment(&elem, &model, FlexuralStrengthFactors::NOMINAL);
        let a1 = one_bar_area(25.0);
        let k0 = 40.0 + 10.0 + 25.0 / 2.0;
        let strong = rebar.edge_steel(RectEdge::Top, b, d);
        let weak = rebar.edge_steel(RectEdge::Left, b, d);
        assert!((strong.effective_depth_mm - (d - k0)).abs() < 1e-9);
        assert!((weak.effective_depth_mm - (b - k0)).abs() < 1e-9);
        let my_strong = 0.9 * (5.0 * a1) * 345.0 * (d - k0);
        let my_weak = 0.9 * (2.0 * a1) * 345.0 * (b - k0);
        assert!(my_weak < my_strong);
        assert!(
            (my - my_weak).abs() < 1e-6 * my_weak.max(1.0),
            "My={my} 弱軸手計算={my_weak} 強軸手計算={my_strong}"
        );
    }
}
