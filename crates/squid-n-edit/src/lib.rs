use squid_n_core::model::Model;

/// 編集コマンド。`Send` を要求するのは、MCP サーバ(P8)が `UndoStack` を
/// スレッド間で共有する(`rmcp::ServerHandler: Send + Sync`)ため。
/// コマンドはモデルデータの断片のみを保持するプレーンな構造体であり、
/// 全実装が自然に `Send` を満たす。
pub trait EditCommand: Send {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand>;
    fn label(&self) -> &str;
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

    pub fn run(&mut self, model: &mut Model, cmd: Box<dyn EditCommand>) {
        let inv = cmd.apply(model);
        self.done.push(inv);
        if self.done.len() > self.max_undo {
            self.done.remove(0);
        }
        self.undone.clear();
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

mod composite;
mod load_case;
mod member_detail;
mod node_member;
mod section_material;
mod steel_design;
mod wall_misc;

pub use composite::*;
pub use load_case::*;
pub use member_detail::*;
pub use node_member::*;
pub use section_material::*;
pub use steel_design::*;
pub use wall_misc::*;

#[cfg(test)]
mod tests;
