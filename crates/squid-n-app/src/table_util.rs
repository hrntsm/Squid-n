//! 表の定型（striped・列定義・strong 見出し・共通行高の本文）を 1 箇所へまとめる
//! ヘルパ。かつては同じ組み立てが各ビューへコピーされていた。
//!
//! 行の中身は呼び出し側のクロージャがそのまま描くため、セルに編集ウィジェット
//! （`TextEdit`・`ComboBox` 等）を置く表にも使える。定型から外れる表
//! （見出しの変数展開・見出しセルごとの装飾・`body.row` 混在・行ごとに異なる
//! 行高）は、無理に畳まず `TableBuilder` の直書きのままとする。

use egui_extras::{Column, TableBuilder, TableRow};

/// 定型の表を描く。`columns` と `headers` の数は一致させること。
/// `salt` は同一パネル内に複数テーブルを置くときの egui Id 衝突を避ける。
/// 行の中身（`row_fn`）はセルごとに `row.col(...)` で描く（従来の body クロージャと同じ）。
pub(crate) fn standard_table(
    ui: &mut egui::Ui,
    salt: &str,
    columns: &[Column],
    headers: &[&str],
    n_rows: usize,
    mut row_fn: impl FnMut(&mut TableRow),
) {
    let row_h = crate::theme::table_row_height(ui);
    let mut tb = TableBuilder::new(ui).id_salt(salt).striped(true);
    for c in columns {
        tb = tb.column(*c);
    }
    tb.header(row_h, |mut h| {
        for t in headers {
            h.col(|ui| {
                ui.strong(*t);
            });
        }
    })
    .body(|body| {
        body.rows(row_h, n_rows, |mut row| row_fn(&mut row));
    });
}
