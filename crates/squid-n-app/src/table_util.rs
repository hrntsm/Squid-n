//! 表の定型（striped・列定義・strong 見出し・共通行高の本文）を 1 箇所へまとめる
//! ヘルパ。かつては同じ組み立てが各ビューへコピーされていた。
//!
//! 行の中身は呼び出し側のクロージャがそのまま描くため、セルに編集ウィジェット
//! （`TextEdit`・`ComboBox` 等）を置く表にも使える。定型から外れる表
//! （見出しの変数展開・見出しセルごとの装飾・`body.row` 混在・行ごとに異なる
//! 行高）は、無理に畳まず `TableBuilder` の直書きのままとする。

use egui_extras::{Column, TableBuilder, TableRow};

/// 列の最小幅 [pt]。クリップを有効にすると列は内容より狭くもなれるため、
/// 見出しすら読めない幅まで潰れないように下限を与える。
const MIN_COLUMN_WIDTH: f32 = 32.0;

/// 定型の表を描く。`columns` と `headers` の数は一致させること。
/// `salt` は同一パネル内に複数テーブルを置くときの egui Id 衝突を避ける。
/// 行の中身（`row_fn`）はセルごとに `row.col(...)` で描く（従来の body クロージャと同じ）。
///
/// 列はすべてクリップ有効・リサイズ可能にする。`egui_extras::Column` は既定で
/// クリップしないため、セルの内容が列幅を超えるとその行だけ以降のセルが右へ
/// 押し出され、行ごとに列位置がずれる（内容の広い行が 1 つあるだけで表が崩れる）。
/// クリップを有効にすると内容はセル内で切り詰められ、列位置は全行で揃う。
/// 切り詰められた内容は呼び出し側がツールチップで補うこと。
pub(crate) fn standard_table(
    ui: &mut egui::Ui,
    salt: &str,
    columns: &[Column],
    headers: &[&str],
    n_rows: usize,
    mut row_fn: impl FnMut(&mut TableRow),
) {
    let row_h = crate::theme::table_row_height(ui);
    let mut tb = TableBuilder::new(ui)
        .id_salt(salt)
        .striped(true)
        .resizable(true);
    for c in columns {
        tb = tb.column(c.clip(true).at_least(MIN_COLUMN_WIDTH));
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
