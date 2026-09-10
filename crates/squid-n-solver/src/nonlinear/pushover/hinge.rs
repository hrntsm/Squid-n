//! 曲げヒンジの閾値算定と発生追跡。
//!
//! - [`HingeThreshold`] — 部材の曲げひび割れ・降伏モーメント閾値
//! - [`compute_hinge_thresholds`] — 全部材の閾値を算定
//! - [`track_hinges`] — 各ステップのヒンジ発生・レベルを判定し記録

use super::geom::member_end_forces_at_face;
use super::types::{HingeEvent, HingeLevel};
use squid_n_core::flexural_strength::{
    member_flexural_yield_moment, section_elastic_modulus, FlexuralStrengthFactors,
};
use squid_n_core::material_grade::{
    material_strength_factor_rebar, material_strength_factor_steel,
};
use squid_n_core::model::{ElementData, Model};
use squid_n_core::section_shape::SectionShape;
use squid_n_core::structure_kind::StructureKind;
use squid_n_element::behavior::{Ctx, ElementBehavior};

/// 部材塑性率の終局ヒンジ判定値（既定値 4.0）。この値以上のヒンジを Ultimate と分類する。
const ULTIMATE_DUCTILITY: f64 = 4.0;

/// ヒンジ判定のモーメント閾値（実スケルトンの折れ点）。
/// RC はひび割れ Mc=κ·Fc·Ze・降伏 My、鉄骨は全塑性 Mp（Mc=My）。
pub(crate) struct HingeThreshold {
    /// 曲げひび割れモーメント Mc [N·mm]（RC のみ有意。鉄骨は My と同値）。
    pub(crate) mc: f64,
    /// 曲げ降伏モーメント My [N·mm]。
    pub(crate) my: f64,
}

/// 部材の曲げヒンジ閾値を算定する。
fn member_moment_thresholds(elem: &ElementData, model: &Model) -> HingeThreshold {
    let Some(sec) = elem.section.and_then(|sid| model.sections.get(sid.index())) else {
        return HingeThreshold { mc: 0.0, my: 0.0 };
    };
    let mat = model.element_material(elem);
    let rebar_mat = model.element_rebar_material(elem);
    let factors = FlexuralStrengthFactors {
        steel: mat.map(material_strength_factor_steel).unwrap_or(1.0),
        rebar: rebar_mat.map(material_strength_factor_rebar).unwrap_or(1.0),
    };
    let ze = section_elastic_modulus(sec);
    let kind = squid_n_core::structure_kind::structure_kind_of(Some(sec), mat.map(|m| m.category));
    match (&sec.shape, kind) {
        (Some(SectionShape::RcRect { .. }) | Some(SectionShape::RcCircle { .. }), _) => {
            let fc = mat.and_then(|m| m.fc).unwrap_or(0.0);
            let mc = squid_n_core::rc_capacity::rc_crack_moment(fc, ze);
            let my = member_flexural_yield_moment(elem, model, factors);
            HingeThreshold { mc: mc.min(my), my }
        }
        (Some(_shape), StructureKind::S) => {
            let my = member_flexural_yield_moment(elem, model, factors);
            HingeThreshold { mc: my, my }
        }
        _ => {
            let my = member_flexural_yield_moment(elem, model, factors);
            let fc = mat.and_then(|m| m.fc).unwrap_or(0.0);
            let mc = if fc > 0.0 {
                squid_n_core::rc_capacity::rc_crack_moment(fc, ze).min(my)
            } else {
                my
            };
            HingeThreshold { mc, my }
        }
    }
}

pub(crate) fn compute_hinge_thresholds(model: &Model) -> Vec<HingeThreshold> {
    model
        .elements
        .iter()
        .map(|elem| member_moment_thresholds(elem, model))
        .collect()
}

pub(crate) fn track_hinges(
    model: &Model,
    behaviors: &[Box<dyn ElementBehavior>],
    thresholds: &[HingeThreshold],
    ductility: &[f64],
    step: u32,
    hinges: &mut Vec<HingeEvent>,
) {
    let ctx = Ctx { model };
    for (i, (elem, b)) in model.elements.iter().zip(behaviors).enumerate() {
        let f = b.internal_force(&ctx);
        let Some(fl) = member_end_forces_at_face(model, elem, &f.data) else {
            continue;
        };
        let m_i = fl[4].abs().max(fl[5].abs());
        let m_j = fl[10].abs().max(fl[11].abs());
        let m_max = m_i.max(m_j);
        let th = &thresholds[i];
        if th.mc <= 0.0 || m_max < th.mc {
            continue;
        }
        let mu = if ductility.get(i).copied().unwrap_or(0.0) > 0.0 {
            ductility[i]
        } else if th.my > 0.0 {
            m_max / th.my
        } else {
            0.0
        };
        for (end_idx, m_end) in [(0usize, m_i), (1usize, m_j)] {
            if m_end < th.mc {
                continue;
            }
            let level = if m_end >= th.my {
                if mu >= ULTIMATE_DUCTILITY {
                    HingeLevel::Ultimate
                } else {
                    HingeLevel::Yield
                }
            } else {
                HingeLevel::Crack
            };
            hinges.push(HingeEvent {
                step,
                elem: elem.id,
                pos: end_idx as f64,
                level,
                ductility: mu,
            });
        }
    }
}
