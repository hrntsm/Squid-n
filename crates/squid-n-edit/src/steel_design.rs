//! S 造部材の断面検定用属性（`SteelDesignAttr`: 継手・スカラップ欠損率、
//! 横座屈長さ・座屈長さの直接入力）の編集コマンド。

use super::*;
use squid_n_core::ids::*;

/// S 造部材の断面検定用属性（`SteelDesignAttr`）を追加/更新する。`attr.elem` に
/// 一致する既存エントリがあれば置換し、なければ末尾に追加する。逆操作は変更前の
/// 状態への復元（既存エントリの置換なら変更前の `SteelDesignAttr` で
/// [`SetSteelDesignAttr`] を再実行、新規追加なら [`RemoveSteelDesignAttr`] で
/// 取り消す）。[`SetWallAttr`](crate::SetWallAttr)・
/// [`SetMemberDetailAttr`](crate::SetMemberDetailAttr) と同じパターン。
pub struct SetSteelDesignAttr {
    pub attr: squid_n_core::model::SteelDesignAttr,
}

impl EditCommand for SetSteelDesignAttr {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if !crate::refs::elem_exists(model, self.attr.elem) {
            return Box::new(Noop);
        }
        if let Some(pos) = model
            .steel_design_attrs
            .iter()
            .position(|a| a.elem == self.attr.elem)
        {
            let old = model.steel_design_attrs[pos].clone();
            model.steel_design_attrs[pos] = self.attr.clone();
            Box::new(SetSteelDesignAttr { attr: old })
        } else {
            model.steel_design_attrs.push(self.attr.clone());
            Box::new(RemoveSteelDesignAttr {
                elem: self.attr.elem,
            })
        }
    }

    fn label(&self) -> &str {
        "S造検定属性変更"
    }
}

/// S 造部材の断面検定用属性エントリを削除する（`elem` に一致するものを削除）。
/// 一致するエントリがなければ Noop。逆操作は削除前の値を復元する
/// [`SetSteelDesignAttr`]（このエントリの `elem` は削除時点で存在しないため、
/// `SetSteelDesignAttr` は「既存エントリなし→末尾追加」の枝を通り、元の位置
/// には戻らないが、`steel_design_attrs` は `ElemId` をキーとする集合的な
/// データであり配列順に意味はないため問題ない）。
pub struct RemoveSteelDesignAttr {
    pub elem: ElemId,
}

impl EditCommand for RemoveSteelDesignAttr {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if let Some(pos) = model
            .steel_design_attrs
            .iter()
            .position(|a| a.elem == self.elem)
        {
            let old = model.steel_design_attrs.remove(pos);
            Box::new(SetSteelDesignAttr { attr: old })
        } else {
            Box::new(Noop)
        }
    }

    fn label(&self) -> &str {
        "S造検定属性削除"
    }
}
