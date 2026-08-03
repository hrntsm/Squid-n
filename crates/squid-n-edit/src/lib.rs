use squid_n_core::model::Model;

/// 編集コマンド。`Send` を要求するのは、MCP サーバ(P8)が `UndoStack` を
/// スレッド間で共有する(`rmcp::ServerHandler: Send + Sync`)ため。
/// コマンドはモデルデータの断片のみを保持するプレーンな構造体であり、
/// 全実装が自然に `Send` を満たす。
pub trait EditCommand: Send {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand>;
    fn label(&self) -> &str;
    /// 何も変更しないコマンド（適用失敗時の安全なフォールバック `Noop` 等）か。
    /// [`UndoStack::run`] は逆コマンドがこれに該当する場合、undo 履歴へ積まない。
    fn is_noop(&self) -> bool {
        false
    }
}

impl<T: EditCommand + 'static> From<T> for Box<dyn EditCommand> {
    fn from(cmd: T) -> Self {
        Box::new(cmd)
    }
}

pub struct UndoStack {
    done: Vec<Box<dyn EditCommand>>,
    undone: Vec<Box<dyn EditCommand>>,
    max_undo: usize,
}

impl UndoStack {
    pub fn new() -> Self {
        Self {
            done: Vec::new(),
            undone: Vec::new(),
            max_undo: 100,
        }
    }

    pub fn with_max(max_undo: usize) -> Self {
        Self {
            done: Vec::new(),
            undone: Vec::new(),
            max_undo,
        }
    }

    /// コマンドを適用し、モデルが変更されたか（適用に成功したか）を返す。
    ///
    /// 適用に失敗したコマンド（逆コマンドが `Noop`）は undo 履歴へ積まず、
    /// redo 履歴も消さない。かつては失敗時も `Noop` を積んでいたため、
    /// undo ラベルに「Noop」が表示される・undo が 1 段を無駄に消費する・
    /// 失敗した操作で redo 履歴が失われる、という不整合が生じていた。
    pub fn run(&mut self, model: &mut Model, cmd: Box<dyn EditCommand>) -> bool {
        let inv = cmd.apply(model);
        if inv.is_noop() {
            return false;
        }
        self.done.push(inv);
        if self.done.len() > self.max_undo {
            self.done.remove(0);
        }
        self.undone.clear();
        true
    }

    pub fn undo(&mut self, model: &mut Model) {
        if let Some(cmd) = self.done.pop() {
            let redo_cmd = cmd.apply(model);
            self.undone.push(redo_cmd);
        }
    }

    pub fn redo(&mut self, model: &mut Model) {
        if let Some(cmd) = self.undone.pop() {
            let undo_cmd = cmd.apply(model);
            self.done.push(undo_cmd);
        }
    }

    pub fn can_undo(&self) -> bool {
        !self.done.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.undone.is_empty()
    }

    pub fn undo_label(&self) -> Option<&str> {
        self.done.last().map(|c| c.label())
    }

    pub fn redo_label(&self) -> Option<&str> {
        self.undone.last().map(|c| c.label())
    }
}

impl Default for UndoStack {
    fn default() -> Self {
        Self::new()
    }
}

pub fn push_edit_command(model: &mut Model, stack: &mut UndoStack, cmd: Box<dyn EditCommand>) {
    stack.run(model, cmd);
}

/// ID＝配列添字エンティティの「削除／再挿入」コマンド対を生成するマクロ。
///
/// 材料・断面・荷重ケース・床で同一の定型（存在・同一性チェック → 使用中ガード →
/// remove → 後続 ID の繰り上げ → 逆操作の生成、およびその逆）が 4 通りに手書き
/// コピーされ、ID 繰り上げ漏れ（本クレートの過去の不具合の温床）になっていた
/// ため、機構部分を単一情報源化する。エンティティ固有の退避データを持つ
/// 節点・部材（`node_member.rs`）は対象外（個別実装のまま）。
///
/// - Delete 側: `pub struct $delete { pub id: $id }`。存在しない・ID 不一致・
///   `guard`（使用中）のときは `Noop` を返す。
/// - Insert 側: `pub struct $insert { pub index: usize, pub old: $entity }`。
///   挿入位置以降の ID を繰り下げてから、`old` の ID を振り直して挿入する。
/// - `shift`: モデル内の全 ID 参照へ `f` を適用する関数
///   （`fn(&mut Model, impl FnMut(&mut Id))`。走査本体は core の `visit_*_ids`）。
/// - `guard`: `fn(&Model, Id) -> bool`。真なら削除を拒否して `Noop`。
macro_rules! id_indexed_delete_insert {
    (
        $(#[$del_meta:meta])*
        $delete:ident,
        $(#[$ins_meta:meta])*
        $insert:ident,
        id = $id_ty:ident,
        entity = $ent_ty:ty,
        vec = $vecf:ident,
        shift = $shift:expr,
        guard = $guard:expr,
        del_label = $dl:literal,
        ins_label = $il:literal $(,)?
    ) => {
        $(#[$del_meta])*
        pub struct $delete {
            pub id: $id_ty,
        }

        impl EditCommand for $delete {
            fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
                let idx = self.id.index();
                if idx >= model.$vecf.len() || model.$vecf[idx].id != self.id {
                    return Box::new(Noop);
                }
                #[allow(clippy::redundant_closure_call)]
                if ($guard)(model, self.id) {
                    return Box::new(Noop);
                }
                let removed = model.$vecf.remove(idx);
                let target = self.id.0;
                $shift(model, |id: &mut $id_ty| {
                    if id.0 > target {
                        id.0 -= 1;
                    }
                });
                Box::new($insert {
                    index: idx,
                    old: removed,
                })
            }

            fn label(&self) -> &str {
                $dl
            }
        }

        $(#[$ins_meta])*
        pub struct $insert {
            pub index: usize,
            pub old: $ent_ty,
        }

        impl EditCommand for $insert {
            fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
                if self.index > model.$vecf.len() {
                    return Box::new(Noop);
                }
                let id = $id_ty(self.index as u32);
                $shift(model, |x: &mut $id_ty| {
                    if x.0 >= id.0 {
                        x.0 += 1;
                    }
                });
                let mut entity = self.old.clone();
                entity.id = id;
                model.$vecf.insert(self.index, entity);
                Box::new($delete { id })
            }

            fn label(&self) -> &str {
                $il
            }
        }
    };
}

/// ID を持たない（配列添字のみで管理され他データから参照されない）エンティティの
/// 「削除／再挿入」コマンド対を生成するマクロ。ID 再採番は不要のため、
/// index の範囲チェックと remove／insert のみを行う。
macro_rules! indexed_delete_insert {
    (
        $(#[$del_meta:meta])*
        $delete:ident,
        $(#[$ins_meta:meta])*
        $insert:ident,
        entity = $ent_ty:ty,
        vec = $vecf:ident,
        field = $field:ident,
        del_label = $dl:literal,
        ins_label = $il:literal $(,)?
    ) => {
        $(#[$del_meta])*
        pub struct $delete {
            pub index: usize,
        }

        impl EditCommand for $delete {
            fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
                if self.index >= model.$vecf.len() {
                    return Box::new(Noop);
                }
                let removed = model.$vecf.remove(self.index);
                Box::new($insert {
                    index: self.index,
                    $field: removed,
                })
            }

            fn label(&self) -> &str {
                $dl
            }
        }

        $(#[$ins_meta])*
        pub struct $insert {
            pub index: usize,
            pub $field: $ent_ty,
        }

        impl EditCommand for $insert {
            fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
                if self.index > model.$vecf.len() {
                    return Box::new(Noop);
                }
                model.$vecf.insert(self.index, self.$field.clone());
                Box::new($delete { index: self.index })
            }

            fn label(&self) -> &str {
                $il
            }
        }
    };
}

mod axis;
mod composite;
mod load_case;
mod member_detail;
mod node_member;
mod section_material;
mod steel_design;
mod wall_misc;

pub use axis::*;
pub use composite::*;
pub use load_case::*;
pub use member_detail::*;
pub use node_member::*;
pub use section_material::*;
pub use steel_design::*;
pub use wall_misc::*;

#[cfg(test)]
mod tests;
