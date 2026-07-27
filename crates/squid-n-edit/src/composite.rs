//! 複合コマンド。複数の編集コマンドを undo/redo 1 回の単位にまとめる。

use super::*;

/// 子コマンド列を順に適用し、逆操作を「逆順の逆コマンド列」として返す複合コマンド。
///
/// グリッド操作のペースト・範囲クリア・行削除など、複数セル（＋行追加・行削除）に
/// またがる変更を undo 1 回で丸ごと戻すための基盤（dev_docs/specs/グリッド操作.md §3.5）。
/// 行削除は [`DeleteNode`] 等を行番号の降順に並べて構成すること
/// （昇順だと先行する削除で後続の ID がずれる）。
pub struct CompositeCommand {
    pub label: String,
    pub children: Vec<Box<dyn EditCommand>>,
}

impl EditCommand for CompositeCommand {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let inverses: Vec<_> = self.children.iter().map(|c| c.apply(model)).collect();
        Box::new(CompositeCommand {
            label: self.label.clone(),
            children: inverses.into_iter().rev().collect(),
        })
    }

    fn label(&self) -> &str {
        &self.label
    }
}
