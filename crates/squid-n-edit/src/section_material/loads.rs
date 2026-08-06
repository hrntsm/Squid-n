//! 荷重（荷重ケース名・節点荷重・部材荷重）の編集コマンド。

use super::*;
use squid_n_core::ids::*;

/// 荷重ケース名変更。
pub struct SetLoadCaseName {
    pub id: LoadCaseId,
    pub name: String,
}

impl EditCommand for SetLoadCaseName {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.load_cases.len() || model.load_cases[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut model.load_cases[idx].name, self.name.clone());
        Box::new(SetLoadCaseName {
            id: self.id,
            name: old,
        })
    }

    fn label(&self) -> &str {
        "荷重ケース名変更"
    }
}

/// 荷重ケースが存在すれば添字を返す（`id == 添字` 規約の検証込み）。
fn load_case_index(model: &Model, lc: LoadCaseId) -> Option<usize> {
    let idx = lc.index();
    (idx < model.load_cases.len() && model.load_cases[idx].id == lc).then_some(idx)
}

/// 節点荷重を荷重ケースへ追加。逆操作は末尾要素の削除。
/// 1 つの節点に何件でも追加できる（解析では全件が加算される）。
pub struct AddNodalLoad {
    pub lc: LoadCaseId,
    pub load: squid_n_core::model::NodalLoad,
}

impl EditCommand for AddNodalLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let Some(idx) = load_case_index(model, self.lc) else {
            return Box::new(Noop);
        };
        model.load_cases[idx].nodal.push(self.load.clone());
        Box::new(DeleteNodalLoad {
            lc: self.lc,
            index: model.load_cases[idx].nodal.len() - 1,
        })
    }

    fn label(&self) -> &str {
        "節点荷重追加"
    }
}

/// 節点荷重を index 指定で丸ごと差し替える（対象節点・成分値・名称）。
/// 準備計算が生成した荷重（`LoadSource::Auto`）は同期のたびに作り直されるため
/// 変更できない（Noop）。
pub struct SetNodalLoad {
    pub lc: LoadCaseId,
    pub index: usize,
    pub load: squid_n_core::model::NodalLoad,
}

impl EditCommand for SetNodalLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let Some(idx) = load_case_index(model, self.lc) else {
            return Box::new(Noop);
        };
        let nodal = &mut model.load_cases[idx].nodal;
        if self.index >= nodal.len() || nodal[self.index].source.is_auto() {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut nodal[self.index], self.load.clone());
        Box::new(SetNodalLoad {
            lc: self.lc,
            index: self.index,
            load: old,
        })
    }

    fn label(&self) -> &str {
        "節点荷重変更"
    }
}

/// 節点荷重を index 指定で削除。逆操作は同位置への挿入。
/// 自動生成分は削除できない（[`SetNodalLoad`] と同じ理由）。
pub struct DeleteNodalLoad {
    pub lc: LoadCaseId,
    pub index: usize,
}

impl EditCommand for DeleteNodalLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let Some(idx) = load_case_index(model, self.lc) else {
            return Box::new(Noop);
        };
        let nodal = &mut model.load_cases[idx].nodal;
        if self.index >= nodal.len() || nodal[self.index].source.is_auto() {
            return Box::new(Noop);
        }
        let removed = nodal.remove(self.index);
        Box::new(InsertNodalLoad {
            lc: self.lc,
            index: self.index,
            load: removed,
        })
    }

    fn label(&self) -> &str {
        "節点荷重削除"
    }
}

/// 節点荷重を index 位置へ挿入（[`DeleteNodalLoad`] の逆操作）。
pub struct InsertNodalLoad {
    pub lc: LoadCaseId,
    pub index: usize,
    pub load: squid_n_core::model::NodalLoad,
}

impl EditCommand for InsertNodalLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let Some(idx) = load_case_index(model, self.lc) else {
            return Box::new(Noop);
        };
        let nodal = &mut model.load_cases[idx].nodal;
        if self.index > nodal.len() {
            return Box::new(Noop);
        }
        nodal.insert(self.index, self.load.clone());
        Box::new(DeleteNodalLoad {
            lc: self.lc,
            index: self.index,
        })
    }

    fn label(&self) -> &str {
        "節点荷重挿入"
    }
}

/// 部材（梁）荷重を荷重ケースへ追加。逆操作は末尾要素の削除。
pub struct AddMemberLoad {
    pub lc: LoadCaseId,
    pub load: squid_n_core::model::MemberLoad,
}

impl EditCommand for AddMemberLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let Some(idx) = load_case_index(model, self.lc) else {
            return Box::new(Noop);
        };
        model.load_cases[idx].member.push(self.load.clone());
        let pos = model.load_cases[idx].member.len() - 1;
        Box::new(DeleteMemberLoad {
            lc: self.lc,
            index: pos,
        })
    }

    fn label(&self) -> &str {
        "部材荷重追加"
    }
}

/// 部材荷重を index 指定で丸ごと差し替える（対象部材・方向・種別・名称）。
/// 自動生成分は変更できない（[`SetNodalLoad`] と同じ理由）。
pub struct SetMemberLoad {
    pub lc: LoadCaseId,
    pub index: usize,
    pub load: squid_n_core::model::MemberLoad,
}

impl EditCommand for SetMemberLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let Some(idx) = load_case_index(model, self.lc) else {
            return Box::new(Noop);
        };
        let member = &mut model.load_cases[idx].member;
        if self.index >= member.len() || member[self.index].source.is_auto() {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut member[self.index], self.load.clone());
        Box::new(SetMemberLoad {
            lc: self.lc,
            index: self.index,
            load: old,
        })
    }

    fn label(&self) -> &str {
        "部材荷重変更"
    }
}

/// 部材荷重を index 指定で削除。逆操作は同位置への挿入。
pub struct DeleteMemberLoad {
    pub lc: LoadCaseId,
    pub index: usize,
}

impl EditCommand for DeleteMemberLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let Some(idx) = load_case_index(model, self.lc) else {
            return Box::new(Noop);
        };
        let member = &mut model.load_cases[idx].member;
        if self.index >= member.len() || member[self.index].source.is_auto() {
            return Box::new(Noop);
        }
        let removed = member.remove(self.index);
        Box::new(InsertMemberLoad {
            lc: self.lc,
            index: self.index,
            load: removed,
        })
    }

    fn label(&self) -> &str {
        "部材荷重削除"
    }
}

/// 部材荷重を index 位置へ挿入（DeleteMemberLoad の逆操作）。
pub struct InsertMemberLoad {
    pub lc: LoadCaseId,
    pub index: usize,
    pub load: squid_n_core::model::MemberLoad,
}

impl EditCommand for InsertMemberLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let Some(idx) = load_case_index(model, self.lc) else {
            return Box::new(Noop);
        };
        let member = &mut model.load_cases[idx].member;
        if self.index > member.len() {
            return Box::new(Noop);
        }
        member.insert(self.index, self.load.clone());
        Box::new(DeleteMemberLoad {
            lc: self.lc,
            index: self.index,
        })
    }

    fn label(&self) -> &str {
        "部材荷重挿入"
    }
}
