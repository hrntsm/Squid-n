//! 制振ダンパー定義（プリセットライブラリ、`Model::damper_defs`）の編集コマンド。
//!
//! 「断面を選ぶように制振要素を選んで部材に割当てる」UX の土台。定義は
//! `ElemId` への参照を持たない名前付きプリセットであり、部材への割当は
//! `DamperProps` の値コピー（`Model::damper_attrs`）で行うため、本ファイルの
//! コマンドで定義を更新・削除しても既存の割当済み部材は壊れない。

use super::*;

/// ダンパー定義の追加（末尾へ追加）。逆操作は削除（[`RemoveDamperDef`]）。
pub struct AddDamperDef {
    pub def: squid_n_core::model::DamperDef,
}

impl EditCommand for AddDamperDef {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        model.damper_defs.push(self.def.clone());
        let index = model.damper_defs.len() - 1;
        Box::new(RemoveDamperDef { index })
    }

    fn label(&self) -> &str {
        "制振ダンパー定義追加"
    }
}

/// ダンパー定義の更新（内容の書き換え）。逆操作は変更前の内容への復元。
pub struct UpdateDamperDef {
    pub index: usize,
    pub def: squid_n_core::model::DamperDef,
}

impl EditCommand for UpdateDamperDef {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let Some(slot) = model.damper_defs.get_mut(self.index) else {
            return Box::new(Noop);
        };
        let old = std::mem::replace(slot, self.def.clone());
        Box::new(UpdateDamperDef {
            index: self.index,
            def: old,
        })
    }

    fn label(&self) -> &str {
        "制振ダンパー定義更新"
    }
}

/// ダンパー定義の削除。逆操作は同じ位置への再挿入（[`InsertDamperDef`]）。
/// 部材への割当は値コピーのため（[`AddDamperDef`] のモジュール docs参照）、
/// 削除しても既存の割当済み部材（`Model::damper_attrs`）は影響を受けない。
pub struct RemoveDamperDef {
    pub index: usize,
}

impl EditCommand for RemoveDamperDef {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index >= model.damper_defs.len() {
            return Box::new(Noop);
        }
        let removed = model.damper_defs.remove(self.index);
        Box::new(InsertDamperDef {
            index: self.index,
            def: removed,
        })
    }

    fn label(&self) -> &str {
        "制振ダンパー定義削除"
    }
}

/// [`RemoveDamperDef`] の逆操作（同じ位置への再挿入）。
pub struct InsertDamperDef {
    pub index: usize,
    pub def: squid_n_core::model::DamperDef,
}

impl EditCommand for InsertDamperDef {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.damper_defs.len() {
            return Box::new(Noop);
        }
        model.damper_defs.insert(self.index, self.def.clone());
        Box::new(RemoveDamperDef { index: self.index })
    }

    fn label(&self) -> &str {
        "制振ダンパー定義削除の取り消し"
    }
}
