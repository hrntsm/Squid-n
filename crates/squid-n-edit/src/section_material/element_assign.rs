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

/// 部材の材料割当変更。
pub struct SetElementMaterial {
    pub elem: ElemId,
    pub material: Option<squid_n_core::ids::MaterialId>,
}

impl EditCommand for SetElementMaterial {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.elem.index();
        if idx >= model.elements.len() || model.elements[idx].id != self.elem {
            return Box::new(Noop);
        }
        let old = model.elements[idx].material;
        model.elements[idx].material = self.material;
        Box::new(SetElementMaterial {
            elem: self.elem,
            material: old,
        })
    }

    fn label(&self) -> &str {
        "部材材料割当変更"
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
