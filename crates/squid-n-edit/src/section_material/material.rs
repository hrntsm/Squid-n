//! 材料の編集コマンド（追加・削除・プロパティ編集）。

use super::*;
use squid_n_core::ids::*;
use squid_n_core::model::MaterialCategory;

/// 材料追加。末尾に `MaterialId(len)` で追加する（ID＝配列インデックスの不変条件を維持）。
/// 逆操作は材料削除。
pub struct AddMaterial {
    pub name: String,
    /// 材料の区分。部材が S 造か RC 造かはこの値で決まる
    /// （`squid_n_core::structure_kind`）。
    pub category: MaterialCategory,
    pub young: f64,
    pub poisson: f64,
    pub density: f64,
    pub fc: Option<f64>,
    pub fy: Option<f64>,
    pub strength_factor: Option<f64>,
}

impl EditCommand for AddMaterial {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let new_id = MaterialId(model.materials.len() as u32);
        model.materials.push(squid_n_core::model::Material {
            strength_factor: self.strength_factor,
            concrete_class: Default::default(),
            id: new_id,
            name: self.name.clone(),
            category: self.category,
            young: self.young,
            poisson: self.poisson,
            density: self.density,
            shear: None,
            fc: self.fc,
            fy: self.fy,
        });
        Box::new(DeleteMaterial { id: new_id })
    }

    fn label(&self) -> &str {
        "材料追加"
    }
}

/// 材料削除。部材から参照中の材料は Noop。逆操作は [`InsertMaterial`]。
/// ID＝配列インデックスの不変条件を保つため、後続の材料 ID と部材からの参照を繰り上げる。
pub struct DeleteMaterial {
    pub id: MaterialId,
}

impl EditCommand for DeleteMaterial {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.materials.len() || model.materials[idx].id != self.id {
            return Box::new(Noop);
        }
        if material_in_use(model, self.id) {
            return Box::new(Noop);
        }
        let removed = model.materials.remove(idx);
        shift_material_ids(model, |mid| {
            if mid.0 > self.id.0 {
                mid.0 -= 1;
            }
        });
        Box::new(InsertMaterial {
            index: idx,
            old: removed,
        })
    }

    fn label(&self) -> &str {
        "材料削除"
    }
}

/// 指定インデックスへ材料を再挿入する（[`DeleteMaterial`] の逆操作専用）。
pub struct InsertMaterial {
    pub index: usize,
    pub old: squid_n_core::model::Material,
}

impl EditCommand for InsertMaterial {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.materials.len() {
            return Box::new(Noop);
        }
        let id = MaterialId(self.index as u32);
        shift_material_ids(model, |mid| {
            if mid.0 >= id.0 {
                mid.0 += 1;
            }
        });
        let mut mat = self.old.clone();
        mat.id = id;
        model.materials.insert(self.index, mat);
        Box::new(DeleteMaterial { id })
    }

    fn label(&self) -> &str {
        "材料削除の取り消し"
    }
}

/// モデル内の全ての `MaterialId` 参照（材料自身の ID を含む）に `f` を適用する。
/// 走査は core 側（[`Model::visit_material_ids`]）が単一情報源として持つ
/// （新フィールド追加時の追随漏れを防ぐ）。
fn shift_material_ids(model: &mut Model, f: impl FnMut(&mut MaterialId)) {
    model.visit_material_ids(f);
}

/// 指定材料を参照している要素・二次部材が存在するか（削除ガード用）。
fn material_in_use(model: &Model, id: MaterialId) -> bool {
    model.elements.iter().any(|e| e.material == Some(id))
        || model
            .secondary_members
            .iter()
            .any(|sm| sm.material == Some(id))
}

/// 編集対象の材料プロパティ。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MaterialField {
    Young,
    Poisson,
    Density,
    Fc,
    Fy,
    StrengthFactor,
}

/// 材料プロパティ変更（E・ポアソン比・密度・Fc・Fy・保有耐力計算用強度割増係数）。
pub struct SetMaterialField {
    pub id: MaterialId,
    pub field: MaterialField,
    pub value: Option<f64>,
}

impl EditCommand for SetMaterialField {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.materials.len() || model.materials[idx].id != self.id {
            return Box::new(Noop);
        }
        let mat = &mut model.materials[idx];
        let old = match self.field {
            MaterialField::Young => {
                let old = Some(mat.young);
                mat.young = self.value.unwrap_or(mat.young);
                old
            }
            MaterialField::Poisson => {
                let old = Some(mat.poisson);
                mat.poisson = self.value.unwrap_or(mat.poisson);
                old
            }
            MaterialField::Density => {
                let old = Some(mat.density);
                mat.density = self.value.unwrap_or(mat.density);
                old
            }
            MaterialField::Fc => std::mem::replace(&mut mat.fc, self.value),
            MaterialField::Fy => std::mem::replace(&mut mat.fy, self.value),
            MaterialField::StrengthFactor => {
                std::mem::replace(&mut mat.strength_factor, self.value)
            }
        };
        Box::new(SetMaterialField {
            id: self.id,
            field: self.field,
            value: old,
        })
    }

    fn label(&self) -> &str {
        "材料プロパティ変更"
    }
}

/// 材料名変更。
pub struct SetMaterialName {
    pub id: MaterialId,
    pub name: String,
}

impl EditCommand for SetMaterialName {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.materials.len() || model.materials[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut model.materials[idx].name, self.name.clone());
        Box::new(SetMaterialName {
            id: self.id,
            name: old,
        })
    }

    fn label(&self) -> &str {
        "材料名変更"
    }
}

/// 材料の区分変更（鋼材・鉄筋・コンクリート）。
///
/// 区分は部材の構造種別を決めるため、変更すると剛域長・仕口パネルの対象・
/// 断面検定の式・数量集計がまとめて変わる。
pub struct SetMaterialCategory {
    pub id: MaterialId,
    pub category: MaterialCategory,
}

impl EditCommand for SetMaterialCategory {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.materials.len() || model.materials[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut model.materials[idx].category, self.category);
        Box::new(SetMaterialCategory {
            id: self.id,
            category: old,
        })
    }

    fn label(&self) -> &str {
        "材料区分変更"
    }
}
