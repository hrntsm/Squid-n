//! 部材の付帯情報（ハンチ・継手位置、`MemberDetailAttr`）の編集コマンド。

use super::*;
use squid_n_core::ids::*;

/// 部材の付帯情報（`MemberDetailAttr`）を追加/更新する。`attr.elem` に一致する
/// 既存エントリがあれば置換し、なければ末尾に追加する。逆操作は変更前の状態への
/// 復元（既存エントリの置換なら変更前の `MemberDetailAttr` で
/// [`SetMemberDetailAttr`] を再実行、新規追加なら [`RemoveMemberDetailAttr`]
/// で取り消す）。[`SetWallAttr`](crate::SetWallAttr) と同じパターン。
pub struct SetMemberDetailAttr {
    pub attr: squid_n_core::model::MemberDetailAttr,
}

impl EditCommand for SetMemberDetailAttr {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if !crate::refs::elem_exists(model, self.attr.elem) {
            return Box::new(Noop);
        }
        if let Some(pos) = model
            .member_detail_attrs
            .iter()
            .position(|a| a.elem == self.attr.elem)
        {
            let old = model.member_detail_attrs[pos].clone();
            model.member_detail_attrs[pos] = self.attr.clone();
            Box::new(SetMemberDetailAttr { attr: old })
        } else {
            model.member_detail_attrs.push(self.attr.clone());
            Box::new(RemoveMemberDetailAttr {
                elem: self.attr.elem,
            })
        }
    }

    fn label(&self) -> &str {
        "部材付帯情報変更"
    }
}

/// 部材付帯情報エントリを削除する（`elem` に一致するものを削除）。一致する
/// エントリがなければ Noop。逆操作は削除前の値を復元する
/// [`SetMemberDetailAttr`]（このエントリの `elem` は削除時点で存在しないため、
/// `SetMemberDetailAttr` は「既存エントリなし→末尾追加」の枝を通り、元の位置
/// には戻らないが、`member_detail_attrs` は `ElemId` をキーとする集合的な
/// データであり配列順に意味はないため問題ない）。
pub struct RemoveMemberDetailAttr {
    pub elem: ElemId,
}

impl EditCommand for RemoveMemberDetailAttr {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if let Some(pos) = model
            .member_detail_attrs
            .iter()
            .position(|a| a.elem == self.elem)
        {
            let old = model.member_detail_attrs.remove(pos);
            Box::new(SetMemberDetailAttr { attr: old })
        } else {
            Box::new(Noop)
        }
    }

    fn label(&self) -> &str {
        "部材付帯情報削除"
    }
}
