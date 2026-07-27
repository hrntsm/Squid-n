//! grid_core（純ロジック層）の単体テスト（dev_docs/specs/グリッド操作.md §9.1）。

use super::*;

fn cell(row: usize, col: usize) -> CellRef {
    CellRef { row, col }
}

fn rect(r0: usize, r1: usize, c0: usize, c1: usize) -> SelRect {
    SelRect { r0, r1, c0, c1 }
}

/// 数値列相当の validate（プロトタイプと同じ f64 パース）
fn validate_f64(_row: usize, _col: usize, text: &str) -> Result<(), String> {
    text.parse::<f64>()
        .map(|_| ())
        .map_err(|_| "数値として解釈できません".to_string())
}

fn block(rows: &[&[&str]]) -> Vec<Vec<String>> {
    rows.iter()
        .map(|r| r.iter().map(|s| s.to_string()).collect())
        .collect()
}

// ---- 選択状態機械 ----

#[test]
fn test_grid_state_starts_inactive() {
    let g = GridState::new(3, 4);
    assert!(!g.active);
    assert!(g.editing.is_none());
}

#[test]
fn test_click_moves_anchor_and_collapses() {
    let mut g = GridState::new(3, 4);
    g.click(cell(1, 2), false);
    assert!(g.active);
    assert_eq!(g.anchor, cell(1, 2));
    assert_eq!(g.cursor, cell(1, 2));
    assert_eq!(g.rect(), rect(1, 1, 2, 2));
}

#[test]
fn test_shift_click_extends_without_moving_anchor() {
    let mut g = GridState::new(5, 4);
    g.click(cell(1, 1), false);
    g.click(cell(3, 3), true);
    assert_eq!(g.anchor, cell(1, 1));
    assert_eq!(g.cursor, cell(3, 3));
    assert_eq!(g.rect(), rect(1, 3, 1, 3));
}

#[test]
fn test_click_out_of_range_is_clamped() {
    let mut g = GridState::new(3, 4);
    g.click(cell(10, 10), false);
    assert_eq!(g.anchor, cell(2, 3));
}

#[test]
fn test_drag_moves_cursor_only() {
    let mut g = GridState::new(5, 4);
    g.click(cell(0, 0), false);
    g.drag_to(cell(2, 3));
    assert_eq!(g.anchor, cell(0, 0));
    assert_eq!(g.cursor, cell(2, 3));
    assert_eq!(g.rect(), rect(0, 2, 0, 3));
}

#[test]
fn test_move_cursor_collapses_selection() {
    let mut g = GridState::new(5, 4);
    g.click(cell(1, 1), false);
    g.drag_to(cell(3, 3));
    // 通常移動は anchor 基準で動き、選択を単一セルに畳む
    g.move_cursor(1, 0, false);
    assert_eq!(g.anchor, cell(2, 1));
    assert_eq!(g.cursor, cell(2, 1));
}

#[test]
fn test_move_cursor_extend_moves_cursor_side() {
    let mut g = GridState::new(5, 4);
    g.click(cell(1, 1), false);
    g.move_cursor(1, 1, true);
    g.move_cursor(1, 0, true);
    assert_eq!(g.anchor, cell(1, 1));
    assert_eq!(g.cursor, cell(3, 2));
}

#[test]
fn test_move_cursor_saturates_at_edges() {
    let mut g = GridState::new(3, 3);
    g.click(cell(0, 0), false);
    g.move_cursor(-1, -1, false);
    assert_eq!(g.anchor, cell(0, 0));
    g.click(cell(2, 2), false);
    g.move_cursor(1, 1, false);
    assert_eq!(g.anchor, cell(2, 2));
}

#[test]
fn test_deactivate_keeps_anchor_and_move_restores() {
    let mut g = GridState::new(5, 4);
    g.click(cell(2, 3), false);
    g.deactivate();
    assert!(!g.active);
    // 解除中の矢印キーは移動せず、直前のアクティブセルで選択を復帰する（§4.4）
    g.move_cursor(1, 0, false);
    assert!(g.active);
    assert_eq!(g.anchor, cell(2, 3));
    assert_eq!(g.cursor, cell(2, 3));
}

#[test]
fn test_select_row_and_extend_spans_all_cols() {
    let mut g = GridState::new(5, 4);
    g.select_row(1, false);
    assert_eq!(g.rect(), rect(1, 1, 0, 3));
    assert_eq!(g.anchor, cell(1, 0));
    // extend は「起点行〜対象行 × 全列」（Excel 準拠。§3.3）
    g.select_row(3, true);
    assert_eq!(g.rect(), rect(1, 3, 0, 3));
    assert_eq!(g.anchor, cell(1, 0));
}

#[test]
fn test_select_col_and_extend_spans_all_rows() {
    let mut g = GridState::new(5, 4);
    g.select_col(2, false);
    assert_eq!(g.rect(), rect(0, 4, 2, 2));
    assert_eq!(g.anchor, cell(0, 2));
    // extend は「起点列〜対象列 × 全行」（Excel 準拠。§3.3）
    g.select_col(0, true);
    assert_eq!(g.rect(), rect(0, 4, 0, 2));
    assert_eq!(g.anchor, cell(0, 2));
}

#[test]
fn test_select_all() {
    let mut g = GridState::new(3, 4);
    g.select_all();
    assert!(g.active);
    assert_eq!(g.rect(), rect(0, 2, 0, 3));
    assert_eq!(g.anchor, cell(0, 0));
}

#[test]
fn test_clamp_selection_after_row_removal() {
    let mut g = GridState::new(5, 4);
    g.click(cell(4, 3), false);
    // 行数減少（行削除・リセット）後は選択を表内へ収める
    g.rows = 2;
    g.clamp_selection();
    assert_eq!(g.anchor, cell(1, 3));
    assert_eq!(g.cursor, cell(1, 3));
}

#[test]
fn test_begin_edit_collapses_to_anchor() {
    let mut g = GridState::new(5, 4);
    g.click(cell(1, 1), false);
    g.drag_to(cell(3, 3));
    g.begin_edit();
    assert_eq!(g.editing, Some(cell(1, 1)));
    assert_eq!(g.cursor, cell(1, 1));
    g.end_edit();
    assert!(g.editing.is_none());
}

// ---- 空グリッド不変条件 ----

#[test]
fn test_empty_grid_selection_methods_are_noop() {
    // rows == 0（列はある）と cols == 0 の両方で、全選択系メソッドが no-op
    for (rows, cols) in [(0, 4), (3, 0), (0, 0)] {
        let mut g = GridState::new(rows, cols);
        g.click(cell(0, 0), false);
        assert!(!g.active, "click: rows={rows} cols={cols}");
        g.drag_to(cell(1, 1));
        g.select_row(0, false);
        assert!(!g.active, "select_row: rows={rows} cols={cols}");
        g.select_col(0, false);
        assert!(!g.active, "select_col: rows={rows} cols={cols}");
        g.select_all();
        assert!(!g.active, "select_all: rows={rows} cols={cols}");
        g.move_cursor(1, 1, false);
        assert!(!g.active, "move_cursor: rows={rows} cols={cols}");
    }
}

// ---- parse_tsv ----

#[test]
fn test_parse_tsv_basic() {
    assert_eq!(
        parse_tsv("1\t2\n3\t4"),
        vec![vec!["1", "2"], vec!["3", "4"]]
    );
}

#[test]
fn test_parse_tsv_absorbs_crlf_and_trailing_newlines() {
    // Excel の CRLF と末尾改行を吸収する（§5.2.1）
    assert_eq!(
        parse_tsv("1\t2\r\n3\t4\r\n\r\n"),
        vec![vec!["1", "2"], vec!["3", "4"]]
    );
    assert_eq!(parse_tsv("1\n"), vec![vec!["1"]]);
}

#[test]
fn test_parse_tsv_keeps_empty_cells() {
    // 行中の空セルは保持する（「変更なし」マーカー。§5.2.4）
    assert_eq!(
        parse_tsv("1\t\t3\n\t5\t"),
        vec![vec!["1", "", "3"], vec!["", "5", ""]]
    );
}

// ---- tile_block ----

#[test]
fn test_tile_block_multiple_both_axes() {
    // 1×1 → 3×2: 単一値での範囲埋め
    let t = tile_block(&block(&[&["7"]]), 3, 2);
    assert_eq!(t, block(&[&["7", "7"], &["7", "7"], &["7", "7"]]));
}

#[test]
fn test_tile_block_multiple_rows_only() {
    // 2×1 → 4×1: 行方向に 2 回繰り返し
    let t = tile_block(&block(&[&["1"], &["2"]]), 4, 1);
    assert_eq!(t, block(&[&["1"], &["2"], &["1"], &["2"]]));
}

#[test]
fn test_tile_block_multiple_cols_only() {
    // 1×2 → 1×4: 列方向に 2 回繰り返し
    let t = tile_block(&block(&[&["1", "2"]]), 1, 4);
    assert_eq!(t, block(&[&["1", "2", "1", "2"]]));
}

#[test]
fn test_tile_block_not_divisible_returns_original() {
    // 2×1 → 3×1: 割り切れないので展開しない（起点から 2 セルだけ貼る）
    let b = block(&[&["1"], &["2"]]);
    assert_eq!(tile_block(&b, 3, 1), b);
}

#[test]
fn test_tile_block_same_size_returns_original() {
    // 拡大方向でない（1 倍）は展開扱いにしない
    let b = block(&[&["1", "2"]]);
    assert_eq!(tile_block(&b, 1, 2), b);
}

#[test]
fn test_tile_block_empty_block() {
    let b: Vec<Vec<String>> = vec![];
    assert_eq!(tile_block(&b, 3, 3), b);
    // 空行だけのブロック（最大列数 0）も素通し
    let b = vec![vec![]];
    assert_eq!(tile_block(&b, 2, 2), b);
}

#[test]
fn test_tile_block_ragged_rows_fill_with_empty() {
    // 行ごとに列数が違うブロックは最大列数で数え、足りないセルは空で埋める
    let t = tile_block(&block(&[&["1", "2"], &["3"]]), 2, 4);
    assert_eq!(t, block(&[&["1", "2", "1", "2"], &["3", "", "3", ""]]));
}

// ---- plan_paste ----

#[test]
fn test_plan_paste_normal() {
    let plan = plan_paste(
        &block(&[&["1", "2"], &["3", "4"]]),
        cell(1, 1),
        5,
        4,
        validate_f64,
    )
    .expect("正常ペーストは成功する");
    assert_eq!(
        plan.set,
        vec![
            (1, 1, "1".to_string()),
            (1, 2, "2".to_string()),
            (2, 1, "3".to_string()),
            (2, 2, "4".to_string()),
        ]
    );
    assert_eq!(plan.extra_rows, 0);
    assert_eq!(plan.skipped_empty, 0);
    assert_eq!(plan.anchor, cell(1, 1));
    assert_eq!((plan.block_rows, plan.block_cols), (2, 2));
}

#[test]
fn test_plan_paste_skips_empty_cells() {
    // 空セルは「変更なし」として数だけ報告し、不正には数えない（§5.2.4）
    let plan = plan_paste(&block(&[&["1", "", "3"]]), cell(0, 0), 3, 3, validate_f64)
        .expect("空セル混じりも成功する");
    assert_eq!(
        plan.set,
        vec![(0, 0, "1".to_string()), (0, 2, "3".to_string())]
    );
    assert_eq!(plan.skipped_empty, 1);
}

#[test]
fn test_plan_paste_rejects_column_overflow() {
    // データ列の範囲を右に超えたら全体拒否（§5.2.4）
    let err = plan_paste(&block(&[&["1", "2"]]), cell(0, 2), 3, 3, validate_f64)
        .expect_err("列はみ出しは拒否される");
    assert_eq!(err.len(), 1);
    assert!(err[0].contains("列範囲外"), "{err:?}");
}

#[test]
fn test_plan_paste_rejects_validation_failures_listing_all() {
    // 検証失敗は全体拒否し、不正セルを全件列挙する（§5.2.3）
    let err = plan_paste(
        &block(&[&["abc", "2"], &["3", "xyz"]]),
        cell(0, 0),
        5,
        5,
        validate_f64,
    )
    .expect_err("値不正は拒否される");
    assert_eq!(err.len(), 2);
    assert!(err[0].contains("ブロック1行1列目「abc」"), "{err:?}");
    assert!(err[0].contains("数値として解釈できません"), "{err:?}");
    assert!(err[1].contains("ブロック2行2列目「xyz」"), "{err:?}");
}

#[test]
fn test_plan_paste_counts_extra_rows() {
    // 表の末尾を超える行数を算定する（自動行追加の対象。§5.2.5）
    let plan = plan_paste(
        &block(&[&["1"], &["2"], &["3"], &["4"]]),
        cell(1, 0),
        3,
        1,
        validate_f64,
    )
    .expect("はみ出し行があっても計画は成功する");
    assert_eq!(plan.extra_rows, 2); // 行 1..=4 に貼る → 表 3 行を 2 行超える
    assert_eq!(plan.set.len(), 4);
}

#[test]
fn test_plan_paste_extra_rows_zero_when_all_cells_empty() {
    // 全セル空なら適用対象がなく、はみ出し行も 0
    let plan = plan_paste(&block(&[&[""], &[""]]), cell(2, 0), 3, 1, validate_f64)
        .expect("全空セルも成功する");
    assert!(plan.set.is_empty());
    assert_eq!(plan.extra_rows, 0);
    assert_eq!(plan.skipped_empty, 2);
}

#[test]
fn test_plan_paste_rejects_oversized_block() {
    // サイズ上限（MAX_PASTE_CELLS）超過は検証前に全体拒否する（§5.2.1）
    let big = vec![vec!["1".to_string(); MAX_PASTE_CELLS / 1000 + 1]; 1000];
    let err = plan_paste(&big, cell(0, 0), 10, 10, validate_f64).expect_err("上限超過は拒否される");
    assert_eq!(err.len(), 1);
    assert!(err[0].contains("上限"), "{err:?}");
}

#[test]
fn test_plan_paste_validate_receives_target_coords() {
    // validate にはブロック内位置ではなく貼り付け先の (行, 列) が渡る
    let seen = std::cell::RefCell::new(Vec::new());
    let _ = plan_paste(&block(&[&["1", "2"]]), cell(3, 1), 10, 10, |r, c, _t| {
        seen.borrow_mut().push((r, c));
        Ok(())
    });
    assert_eq!(seen.into_inner(), vec![(3, 1), (3, 2)]);
}

// ---- rect_to_tsv ----

#[test]
fn test_rect_to_tsv_single_cell() {
    let s = rect_to_tsv(rect(1, 1, 2, 2), |r, c| format!("{r}-{c}"));
    assert_eq!(s, "1-2");
}

#[test]
fn test_rect_to_tsv_rectangle() {
    let s = rect_to_tsv(rect(0, 1, 0, 2), |r, c| format!("{r}{c}"));
    assert_eq!(s, "00\t01\t02\n10\t11\t12");
}

#[test]
fn test_rect_to_tsv_row_col_and_all_selection() {
    // 行選択・列選択・全選択の rect は select_row/col/all が作る（結合テスト）
    let mut g = GridState::new(2, 3);
    g.select_row(1, false);
    assert_eq!(
        rect_to_tsv(g.rect(), |r, c| format!("{r}{c}")),
        "10\t11\t12"
    );
    g.select_col(1, false);
    assert_eq!(rect_to_tsv(g.rect(), |r, c| format!("{r}{c}")), "01\n11");
    g.select_all();
    assert_eq!(
        rect_to_tsv(g.rect(), |r, c| format!("{r}{c}")),
        "00\t01\t02\n10\t11\t12"
    );
}
