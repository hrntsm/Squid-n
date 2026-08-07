//! 部材への断面・材料・履歴則・制振ダンパーの割当編集コマンド。

use super::*;
use squid_n_core::ids::*;

/// 部材の断面割当変更。
pub struct SetElementSection {
    pub elem: ElemId,
    pub section: Option<SectionId>,
}

impl EditCommand for SetElementSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.elem.index();
        if idx >= model.elements.len() || model.elements[idx].id != self.elem {
            return Box::new(Noop);
        }
        let old = model.elements[idx].section;
        model.elements[idx].section = self.section;
        Box::new(SetElementSection {
            elem: self.elem,
            section: old,
        })
    }

    fn label(&self) -> &str {
        "部材断面割当変更"
    }
}

/// 断面の材料割当変更。
///
/// **材料は断面が持つ**（`Section::material` ほか）。役割ごとに欄が分かれており、
/// どれを変更するかは [`SectionMaterialRole`] で指定する。
pub struct SetSectionMaterial {
    pub section: squid_n_core::ids::SectionId,
    pub role: SectionMaterialRole,
    pub material: Option<squid_n_core::ids::MaterialId>,
}

/// 断面が持つ材料の役割。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SectionMaterialRole {
    /// 主材料（弾性剛性 E・ν と自重の密度を決める）。
    Main,
    /// 主筋。
    Rebar,
    /// せん断補強筋。
    ShearRebar,
    /// SRC 断面の内蔵鉄骨。
    Steel,
}

impl SectionMaterialRole {
    fn slot(
        self,
        sec: &mut squid_n_core::model::Section,
    ) -> &mut Option<squid_n_core::ids::MaterialId> {
        match self {
            SectionMaterialRole::Main => &mut sec.material,
            SectionMaterialRole::Rebar => &mut sec.rebar_material,
            SectionMaterialRole::ShearRebar => &mut sec.shear_rebar_material,
            SectionMaterialRole::Steel => &mut sec.steel_material,
        }
    }
}

impl EditCommand for SetSectionMaterial {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.section.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.section {
            return Box::new(Noop);
        }
        let slot = self.role.slot(&mut model.sections[idx]);
        let old = std::mem::replace(slot, self.material);
        Box::new(SetSectionMaterial {
            section: self.section,
            role: self.role,
            material: old,
        })
    }

    fn label(&self) -> &str {
        "断面材料割当変更"
    }
}

/// 部材の履歴則（復元力特性）変更（各履歴則の原典による）。
/// `HysteresisModel::Auto` を指定すると個別指定を解除し既定へ戻す。
pub struct SetMemberHysteresis {
    pub elem: ElemId,
    pub rule: squid_n_core::model::HysteresisModel,
}

impl EditCommand for SetMemberHysteresis {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.elem.index();
        if idx >= model.elements.len() || model.elements[idx].id != self.elem {
            return Box::new(Noop);
        }
        let old = model.set_member_hysteresis(self.elem, self.rule);
        Box::new(SetMemberHysteresis {
            elem: self.elem,
            rule: old.unwrap_or(squid_n_core::model::HysteresisModel::Auto),
        })
    }

    fn label(&self) -> &str {
        "部材履歴則変更"
    }
}

/// 部材の履歴則(時刻歴応答解析用スロット)変更。`None` は「増分と同じ」へ戻す。
pub struct SetMemberHysteresisTh {
    pub elem: ElemId,
    pub rule_th: Option<squid_n_core::model::HysteresisModel>,
}

impl EditCommand for SetMemberHysteresisTh {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.elem.index();
        if idx >= model.elements.len() || model.elements[idx].id != self.elem {
            return Box::new(Noop);
        }
        let old = model.set_member_hysteresis_th(self.elem, self.rule_th);
        Box::new(SetMemberHysteresisTh {
            elem: self.elem,
            rule_th: old,
        })
    }

    fn label(&self) -> &str {
        "部材履歴則変更(時刻歴)"
    }
}

/// 制振ダンパーの特性（Kd・C0・α）変更（制振部材の力学モデル: Maxwell モデル等）。
/// `props=None` で指定を解除する。
pub struct SetDamperProps {
    pub elem: ElemId,
    pub props: Option<squid_n_core::model::DamperProps>,
}

impl EditCommand for SetDamperProps {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.elem.index();
        if idx >= model.elements.len() || model.elements[idx].id != self.elem {
            return Box::new(Noop);
        }
        let old = model.set_damper_props(self.elem, self.props);
        Box::new(SetDamperProps {
            elem: self.elem,
            props: old,
        })
    }

    fn label(&self) -> &str {
        "制振ダンパー特性変更"
    }
}
