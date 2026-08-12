//! 部材の曲げ降伏モーメント My の共通算定。
//!
//! プッシュオーバー曲げヒンジ判定（`squid_n_solver::pushover::hinge`）と
//! 材端曲げバネ（`squid_n_element::factory::springs`）が同じ My を使うため、
//! 算定の情報源を 1 つに保つ。

use crate::material_grade::rebar_yield_strength;
use crate::model::{ElementData, Material, Model, Section};
use crate::rc_capacity::{rc_mu_simple, RcCapacityInput};
use crate::rc_rebar_geom::rebar_effective_depth;
use crate::section_shape::{bar_set_area, SectionShape};

/// 曲げ降伏 My 算定に用いる材料強度係数。
///
/// 公称値解析では `(1.0, 1.0)`、保有水平耐力計算では鋼材・主筋それぞれの
/// 材料強度割増係数を与える。
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

/// 断面の弾性断面係数 Ze [mm³]（強軸 I とせい D から `Ze = I / (D/2)`）。
pub fn section_elastic_modulus(sec: &Section) -> f64 {
    let depth = sec.depth.max(sec.width);
    let i_gross = sec.iz.max(sec.iy);
    if depth > 0.0 {
        i_gross / (depth / 2.0)
    } else {
        0.0
    }
}

/// 部材の曲げ降伏（終局）モーメント My [N·mm]。
///
/// - RC 配筋形状（`RcRect` / `RcCircle`）: `0.9·at·σy·j`（[`rc_mu_simple`]）
/// - 塑性断面係数を持つ形状: `Zp·σy`（全塑性 Mp）
/// - それ以外: `σy·Ze`（弾性断面係数フォールバック）
///
/// 分岐順序は材端曲げバネ（`squid_n_element::factory::springs`）と同一。
/// 曲げヒンジ判定も本関数を My の情報源とする。
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
        Some(SectionShape::RcRect { rebar, d, .. }) | Some(SectionShape::RcCircle { rebar, d }) => {
            rc_flexural_yield_moment(elem, model, mat, rebar, *d, ze, factors.rebar)
        }
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

fn rc_flexural_yield_moment(
    elem: &ElementData,
    model: &Model,
    mat: Option<&Material>,
    rebar: &crate::section_shape::RcRebar,
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
    let at = bar_set_area(&rebar.main_x) / 2.0;
    let d_eff = rebar_effective_depth(d, rebar);
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
}
