//! 断面の編集コマンド（追加・削除・複製・形状/プロパティ編集）。

use super::*;
use squid_n_core::ids::*;

/// 断面プロパティ変更（名称・A・Iy・Iz・J 等）。
pub struct SetSectionField {
    pub id: SectionId,
    pub field: SectionField,
    pub value: f64,
}

/// 編集対象の断面プロパティ。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SectionField {
    Area,
    Iy,
    Iz,
    J,
    Depth,
    Width,
    AsY,
    AsZ,
    /// 仕口パネルの板厚 [mm]（ダイアフラム補強・ダブラープレートによる増厚の
    /// 明示指定）。0 以下は未入力として扱い、柱の断面形状から算定する。
    PanelThickness,
}

impl EditCommand for SetSectionField {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.id {
            return Box::new(Noop);
        }
        let sec = &mut model.sections[idx];
        let old = match self.field {
            SectionField::Area => {
                let o = sec.area;
                sec.area = self.value;
                o
            }
            SectionField::Iy => {
                let o = sec.iy;
                sec.iy = self.value;
                o
            }
            SectionField::Iz => {
                let o = sec.iz;
                sec.iz = self.value;
                o
            }
            SectionField::J => {
                let o = sec.j;
                sec.j = self.value;
                o
            }
            SectionField::Depth => {
                let o = sec.depth;
                sec.depth = self.value;
                o
            }
            SectionField::Width => {
                let o = sec.width;
                sec.width = self.value;
                o
            }
            SectionField::AsY => {
                let o = sec.as_y;
                sec.as_y = self.value;
                o
            }
            SectionField::AsZ => {
                let o = sec.as_z;
                sec.as_z = self.value;
                o
            }
            SectionField::PanelThickness => {
                let o = sec.panel_thickness.unwrap_or(0.0);
                // 0 以下は「未入力」を表す。諸元解決も 0 以下を未入力として扱う
                // （`squid_n_core::panel_zone::PanelGeometry::from_column`）。
                sec.panel_thickness = (self.value > 0.0).then_some(self.value);
                o
            }
        };
        Box::new(SetSectionField {
            id: self.id,
            field: self.field,
            value: old,
        })
    }

    fn label(&self) -> &str {
        "断面プロパティ変更"
    }
}

/// 断面名称変更。
pub struct SetSectionName {
    pub id: SectionId,
    pub name: String,
}

impl EditCommand for SetSectionName {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut model.sections[idx].name, self.name.clone());
        Box::new(SetSectionName {
            id: self.id,
            name: old,
        })
    }

    fn label(&self) -> &str {
        "断面名称変更"
    }
}

/// 断面形状を新規追加（UI-3 の新規断面作成）。
pub struct AddSectionShape {
    pub shape: squid_n_section::shape::SectionShape,
    pub new_id: SectionId,
    pub name: String,
}

impl EditCommand for AddSectionShape {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let sec = self.shape.to_section(self.new_id, self.name.clone());
        model.sections.push(sec);
        Box::new(DeleteSection { id: self.new_id })
    }

    fn label(&self) -> &str {
        "断面形状追加"
    }
}

/// 断面形状変更。逆操作は RestoreSection。
pub struct EditSectionShape {
    pub section: SectionId,
    pub new_shape: squid_n_section::shape::SectionShape,
}

impl EditCommand for EditSectionShape {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.section.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.section {
            return Box::new(Noop);
        }
        let old = model.sections[idx].clone();
        let new_sec = self.new_shape.to_section(self.section, old.name.clone());
        model.sections[idx] = new_sec;
        Box::new(RestoreSection { old })
    }

    fn label(&self) -> &str {
        "断面形状変更"
    }
}

/// 断面データを指定した Section で復元する（EditSectionShape の逆操作）。
pub struct RestoreSection {
    pub old: squid_n_core::model::Section,
}

impl EditCommand for RestoreSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.old.id.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.old.id {
            return Box::new(Noop);
        }
        let replaced = std::mem::replace(&mut model.sections[idx], self.old.clone());
        Box::new(RestoreSection { old: replaced })
    }

    fn label(&self) -> &str {
        "断面復元"
    }
}

/// 部材が参照する断面を複製し、部材に新断面を割り当てる。
pub struct DuplicateSectionForMember {
    pub member: ElemId,
}

impl EditCommand for DuplicateSectionForMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let elem_idx = self.member.index();
        if elem_idx >= model.elements.len() || model.elements[elem_idx].id != self.member {
            return Box::new(Noop);
        }
        let sid = match model.elements[elem_idx].section {
            Some(s) => s,
            None => return Box::new(Noop),
        };
        let sec_idx = sid.index();
        if sec_idx >= model.sections.len() || model.sections[sec_idx].id != sid {
            return Box::new(Noop);
        }
        let orig = &model.sections[sec_idx];
        let new_id = SectionId(model.sections.len() as u32);
        let mut new_sec = orig.clone();
        new_sec.id = new_id;
        new_sec.name = format!("{}(複製)", orig.name);
        model.sections.push(new_sec);
        model.elements[elem_idx].section = Some(new_id);
        Box::new(RestoreElementSectionAndDeleteSection {
            elem: self.member,
            old_section: Some(sid),
            new_section: new_id,
        })
    }

    fn label(&self) -> &str {
        "部材断面複製"
    }
}

/// DuplicateSectionForMember の逆操作。
pub struct RestoreElementSectionAndDeleteSection {
    pub elem: ElemId,
    pub old_section: Option<SectionId>,
    pub new_section: SectionId,
}

impl EditCommand for RestoreElementSectionAndDeleteSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let elem_idx = self.elem.index();
        if elem_idx >= model.elements.len() || model.elements[elem_idx].id != self.elem {
            return Box::new(Noop);
        }
        let new_idx = self.new_section.index();
        if new_idx >= model.sections.len() || model.sections[new_idx].id != self.new_section {
            return Box::new(Noop);
        }
        model.elements[elem_idx].section = self.old_section;
        model.sections.remove(new_idx);
        let removed_id = self.new_section;
        shift_section_ids(model, |sid| {
            if sid.0 > removed_id.0 {
                sid.0 -= 1;
            }
        });
        Box::new(DuplicateSectionForMember { member: self.elem })
    }

    fn label(&self) -> &str {
        "部材断面複製解除"
    }
}

/// 断面削除。逆操作は AddSection。
///
/// 部材から参照中の断面は削除すると参照が壊れるため Noop とする
/// （先に割当を解除するか、UI 側でボタンを無効化する）。
/// ID＝配列インデックスの不変条件を保つため、削除後は後続の断面 ID と
/// 部材からの参照を 1 つずつ繰り上げる。
pub struct DeleteSection {
    pub id: SectionId,
}

impl EditCommand for DeleteSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.id {
            return Box::new(Noop);
        }
        if section_in_use(model, self.id) {
            return Box::new(Noop);
        }
        let removed = model.sections.remove(idx);
        shift_section_ids(model, |sid| {
            if sid.0 > self.id.0 {
                sid.0 -= 1;
            }
        });
        Box::new(AddSection {
            old: removed,
            index: idx,
        })
    }

    fn label(&self) -> &str {
        "断面削除"
    }
}

/// 断面追加（DeleteSection の逆操作）。後続の断面 ID・参照を 1 つ繰り下げてから挿入する。
pub struct AddSection {
    pub old: squid_n_core::model::Section,
    pub index: usize,
}

impl EditCommand for AddSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.sections.len() {
            return Box::new(Noop);
        }
        let id = SectionId(self.index as u32);
        shift_section_ids(model, |sid| {
            if sid.0 >= id.0 {
                sid.0 += 1;
            }
        });
        let mut sec = self.old.clone();
        sec.id = id;
        model.sections.insert(self.index, sec);
        Box::new(DeleteSection { id })
    }

    fn label(&self) -> &str {
        "断面追加"
    }
}

/// モデル内の全ての `SectionId` 参照（断面自身の ID を含む）に `f` を適用する。
fn shift_section_ids(model: &mut Model, mut f: impl FnMut(&mut SectionId)) {
    for sec in &mut model.sections {
        f(&mut sec.id);
    }
    for elem in &mut model.elements {
        if let Some(sid) = &mut elem.section {
            f(sid);
        }
    }
    // 小梁（床設計用）の断面参照も追従させる（陳腐化した参照を防ぐ）。
    for slab in &mut model.slabs {
        for j in &mut slab.joists {
            if let Some(sid) = &mut j.section {
                f(sid);
            }
        }
    }
}

/// 指定断面を参照している要素または小梁が存在するか（削除ガード用）。
fn section_in_use(model: &Model, id: SectionId) -> bool {
    model.elements.iter().any(|e| e.section == Some(id))
        || model
            .slabs
            .iter()
            .any(|s| s.joists.iter().any(|j| j.section == Some(id)))
}
