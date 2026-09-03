//! モデルからの断面・材料の引き当て（未割当時のフォールバックを含む）。
//!
//! 線材要素の構築（[`super::truss`]・[`super::beam`]）が共有する。ID が範囲外・
//! 世代違い・未割当のいずれでも「物性ゼロの断面／材料」へ落として構築を続け、
//! 解析前チェック（`solver` の `precheck_model`・`factory::ensure_nonlinear_input`）
//! に検出を委ねる。ここで架空のもっともらしい断面を与えると、チェックを通らない
//! 経路から来たモデルが無音のまま解析されてしまう。

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
    .unwrap_or_else(|| Section {
        id: SectionId(0),
        name: String::new(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 0.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: None,
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    })
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
