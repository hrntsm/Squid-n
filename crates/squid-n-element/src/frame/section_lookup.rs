//! モデルからの断面・材料の引き当て。

use squid_n_core::ids::{MaterialId, SectionId};
use squid_n_core::model::{ElementData, Material, MaterialCategory, Model, Section};

/// 断面 ID から断面を引く。未割当・範囲外・世代違いは物性ゼロの断面へ落とす。
pub(crate) fn get_section(model: &Model, sid: Option<SectionId>) -> Section {
    sid.and_then(|s| {
        if s.index() < model.sections.len() {
            let sec = &model.sections[s.index()];
            if sec.id == s {
                Some(sec.clone())
            } else {
                None
            }
        } else {
            None
        }
    })
    .unwrap_or_else(|| Section::zero(SectionId(0), String::new()))
}

/// 材料 ID から材料を引く。未割当・範囲外・世代違いは物性ゼロの材料へ落とす。
pub(crate) fn get_material(model: &Model, mid: Option<MaterialId>) -> Material {
    mid.and_then(|m| {
        if m.index() < model.materials.len() {
            let mat = &model.materials[m.index()];
            if mat.id == m {
                Some(mat.clone())
            } else {
                None
            }
        } else {
            None
        }
    })
    .unwrap_or_else(|| Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: String::new(),
        category: MaterialCategory::Steel,
        young: 0.0,
        poisson: 0.0,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    })
}

/// 要素の主材料 ID を断面経由で引く（材料は断面が持つ）。
pub(crate) fn sec_material(model: &Model, data: &ElementData) -> Option<MaterialId> {
    model.element_section(data).and_then(|s| s.material)
}
