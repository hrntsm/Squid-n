//! 節点テーブル（座標 3 列）用のグリッドアダプタ（グリッド操作 T4）。
//!
//! egui 非依存で、ヘッドレステスト（app/tests.rs）からモデル・undo と
//! 組み合わせて検証できる。モデル編集はすべて squid-n-edit のコマンド
//! （複数変更は CompositeCommand 1 個）として UndoStack 経由で行い、
//! ペースト・クリア・行削除が undo 1 回で丸ごと戻る
//! （dev_docs/specs/グリッド操作.md §3.5・§5.2.6）。

use std::collections::BTreeMap;

use crate::grid::GridAdapter;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::NodeId;
use squid_n_core::model::Model;
use squid_n_edit::{AddNode, CompositeCommand, DeleteNode, EditCommand, SetNodeCoord, UndoStack};

/// 節点テーブルの [`GridAdapter`] 実装。テーブル描画のフレーム中だけ
/// モデルと undo スタックを借用する使い捨て構造体。
pub struct NodeGridAdapter<'a> {
    pub model: &'a mut Model,
    pub undo: &'a mut UndoStack,
    /// この借用中にモデルを変更したか（呼び出し元が staleness.mark_edited する）
    pub edited: bool,
}

impl NodeGridAdapter<'_> {
    fn run(&mut self, cmd: Box<dyn EditCommand>) {
        self.undo.run(self.model, cmd);
        self.edited = true;
    }

    /// 0 個 = 何もしない、1 個 = 単独コマンド（固有の undo ラベルを保つ）、
    /// 複数 = CompositeCommand 1 個（undo 1 回で丸ごと戻す）として実行する
    fn run_all(&mut self, mut children: Vec<Box<dyn EditCommand>>, label: &str) {
        match children.len() {
            0 => {}
            1 => {
                let cmd = children.pop().expect("len==1 を確認済み");
                self.run(cmd);
            }
            _ => self.run(Box::new(CompositeCommand {
                label: label.to_string(),
                children,
            })),
        }
    }

    /// 既存行のセル変更を「1 行 1 SetNodeCoord」へまとめる
    /// （現座標に変更列を重ねた座標で置き換える）
    fn coord_updates(&self, cells: &[(usize, usize, f64)]) -> BTreeMap<usize, [f64; 3]> {
        let mut updates: BTreeMap<usize, [f64; 3]> = BTreeMap::new();
        for (r, c, v) in cells {
            if let Some(node) = self.model.nodes.get(*r) {
                updates.entry(*r).or_insert(node.coord)[*c] = *v;
            }
        }
        updates
    }
}

impl GridAdapter for NodeGridAdapter<'_> {
    fn rows(&self) -> usize {
        self.model.nodes.len()
    }

    fn cols(&self) -> usize {
        3
    }

    fn row_label(&self, row: usize) -> String {
        // 節点 ID は 0 始まり（ID＝配列位置の不変条件）。ナビゲータ等の
        // N0, N1… 表記と行を対応づけられるよう ID そのものを表示する
        row.to_string()
    }

    fn cell_text(&self, row: usize, col: usize) -> String {
        // f64 の Display = 内部値の正準文字列（表示用の丸めはしない。§5.1）
        self.model
            .nodes
            .get(row)
            .map(|n| format!("{}", n.coord[col]))
            .unwrap_or_default()
    }

    fn validate_cell(&self, _row: usize, _col: usize, text: &str) -> Result<(), String> {
        text.parse::<f64>()
            .map(|_| ())
            .map_err(|_| "数値として解釈できません".to_string())
    }

    fn apply_block(&mut self, cells: &[(usize, usize, String)], append_rows: usize) {
        let n0 = self.model.nodes.len();
        // validate 済みの前提だが、防御的にパース失敗セルは黙って落とさず無視のみ
        let parsed: Vec<(usize, usize, f64)> = cells
            .iter()
            .filter_map(|(r, c, t)| t.parse::<f64>().ok().map(|v| (*r, *c, v)))
            .collect();
        let existing: Vec<_> = parsed.iter().filter(|(r, _, _)| *r < n0).copied().collect();
        let updates = self.coord_updates(&existing);
        // 追加行は既定座標 [0,0,0] に貼り付け値を重ね、AddNode 自体に座標を持たせる
        // （AddNode は末尾 ID＝配列位置で追加するため、行順に並べれば ID が対応する）
        let mut added = vec![[0.0f64; 3]; append_rows];
        for (r, c, v) in parsed.iter().filter(|(r, _, _)| *r >= n0) {
            if let Some(coord) = added.get_mut(r - n0) {
                coord[*c] = *v;
            }
        }
        let mut children: Vec<Box<dyn EditCommand>> = Vec::new();
        for (row, coord) in &updates {
            children.push(Box::new(SetNodeCoord {
                node: NodeId(*row as u32),
                coord: *coord,
            }));
        }
        for coord in added {
            children.push(Box::new(AddNode {
                coord,
                restraint: Dof6Mask::FREE,
            }));
        }
        self.run_all(children, "節点座標の貼り付け");
    }

    fn clear_cells(&mut self, cells: &[(usize, usize)]) -> usize {
        // 節点座標のクリア = 0 埋め（本テーブルの決め。§3.4）
        let zeros: Vec<(usize, usize, f64)> = cells
            .iter()
            .filter(|(r, _)| *r < self.model.nodes.len())
            .map(|(r, c)| (*r, *c, 0.0))
            .collect();
        let updates = self.coord_updates(&zeros);
        let children: Vec<Box<dyn EditCommand>> = updates
            .iter()
            .map(|(row, coord)| {
                Box::new(SetNodeCoord {
                    node: NodeId(*row as u32),
                    coord: *coord,
                }) as Box<dyn EditCommand>
            })
            .collect();
        self.run_all(children, "節点座標のクリア");
        zeros.len()
    }

    fn can_append_rows(&self) -> bool {
        true
    }

    fn can_delete_rows(&self) -> bool {
        true
    }

    fn validate_row_deletion(&self, row: usize) -> Result<(), String> {
        let id = NodeId(row as u32);
        if self.model.node_in_use(id) {
            Err(
                "部材・荷重などから参照されているため削除できません（先に参照を解消してください）"
                    .to_string(),
            )
        } else {
            Ok(())
        }
    }

    fn delete_rows(&mut self, rows: &[usize]) {
        // ID＝配列位置の繰り上げと undo の整合のため、行番号の降順で
        // DeleteNode を並べる（§3.4・§4.6）
        let mut sorted: Vec<usize> = rows
            .iter()
            .copied()
            .filter(|r| *r < self.model.nodes.len())
            .collect();
        sorted.sort_unstable_by(|a, b| b.cmp(a));
        sorted.dedup();
        let label = format!("節点 {} 行の削除", sorted.len());
        let children: Vec<Box<dyn EditCommand>> = sorted
            .iter()
            .map(|r| {
                Box::new(DeleteNode {
                    id: NodeId(*r as u32),
                }) as Box<dyn EditCommand>
            })
            .collect();
        self.run_all(children, &label);
    }
}
