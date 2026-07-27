//! グリッド操作の egui グルー（描画・入力処理。gui feature 限定）。
//!
//! ロジックは持ち込まず薄く保つ（dev_docs/specs/グリッド操作.md §3.1）。
//! 見た目は §6（スプレッドシート様式・TONMANUAL 準拠）、操作は §4、
//! egui 固有の落とし穴への対処は §7 に従う。テーブルごとの差は
//! [`GridAdapter`] に隔離し、本モジュールはテーブル固有の知識を持たない。

use super::{parse_tsv, plan_paste, rect_to_tsv, tile_block};
use super::{CellRef, GridAdapter, GridState, PastePlan, SelRect, MAX_PASTE_CELLS};
use crate::theme;

/// 失敗フラッシュの表示時間（TONMANUAL §8: モーションは控えめ・短く）
const FLASH_SECS: f64 = 1.0;

/// イベントログの保持上限（呼び出し元が take_log で回収するまでのバッファ）
const LOG_CAP: usize = 200;

/// グリッド操作ウィジェット。テーブル 1 つにつき 1 インスタンスを保持し、
/// 毎フレーム [`GridWidget::show`] を呼ぶ。テーブル固有の読み書きは
/// [`GridAdapter`] 経由で行う。
///
/// ステータス表示・イベントログの描画はアプリ側の責務とし、本ウィジェットは
/// ログ行（本文, エラーか）を蓄積して [`GridWidget::take_log`] で引き渡す。
pub struct GridWidget {
    pub grid: GridState,
    /// 行末尾に 1 行削除の 🗑 ボタン列を表示するか（既存テーブルの
    /// 削除ボタンを維持する T4 パイロット用。右クリック行削除と等価な操作）
    pub delete_buttons: bool,
    /// 🗑 ボタンで削除要求された行。描画中の行数変化を避けるため
    /// テーブル描画後に処理する
    pending_row_delete: Option<usize>,
    edit_buf: String,
    edit_needs_focus: bool,
    /// セル起点のドラッグ選択が進行中か（スクロールバー等のドラッグと区別する。§7.2）
    drag_selecting: bool,
    /// ドラッグ選択の起点が行ヘッダか（行単位選択モード）
    drag_rows: bool,
    /// ドラッグ選択の起点が列ヘッダか（列単位選択モード）
    drag_cols: bool,
    /// テーブル領域（スクロールバー含む外形矩形）。この外の押下を
    /// 「表外クリック」として選択解除に使う（§7.3。セル単位ではなく領域で判定し、
    /// スクロールバー操作で選択が消えないようにする）
    table_rect: egui::Rect,
    /// このフレームでコンテキストメニューが開いているか。メニュー操作の
    /// クリックが「表外クリック＝選択解除」に誤判定されるのを防ぐ（§7.7）
    menu_open: bool,
    /// フィードバックのフラッシュ対象範囲・終了時刻・色（egui の time 基準。§6.4）。
    /// 赤 = 全体拒否・編集失敗（何も適用されていない）、
    /// 黄 = サイズ不一致の貼り付け（適用されたが確認を促す）
    flash_rect: Option<SelRect>,
    flash_until: f64,
    flash_color: egui::Color32,
    /// イベントログ（本文, エラーか）。エラー行は赤字で描画すること（§6.4）
    log: Vec<(String, bool)>,
}

impl Default for GridWidget {
    fn default() -> Self {
        Self {
            grid: GridState::new(0, 0),
            delete_buttons: false,
            pending_row_delete: None,
            edit_buf: String::new(),
            edit_needs_focus: false,
            drag_selecting: false,
            drag_rows: false,
            drag_cols: false,
            table_rect: egui::Rect::NOTHING,
            menu_open: false,
            flash_rect: None,
            flash_until: 0.0,
            flash_color: theme::ERROR_RED,
            log: Vec::new(),
        }
    }
}

impl GridWidget {
    pub fn new() -> Self {
        Self::default()
    }

    /// 溜まったイベントログ（本文, エラーか）を取り出す。
    /// 呼び出し元がアプリのイベントログへ転記する（エラー行は赤字で。§6.4）
    pub fn take_log(&mut self) -> Vec<(String, bool)> {
        std::mem::take(&mut self.log)
    }

    fn push_log(&mut self, s: impl Into<String>) {
        self.log.push((s.into(), false));
        if self.log.len() > LOG_CAP {
            self.log.remove(0);
        }
    }

    fn push_err(&mut self, s: impl Into<String>) {
        self.log.push((s.into(), true));
        if self.log.len() > LOG_CAP {
            self.log.remove(0);
        }
    }

    /// フィードバック: 対象範囲を指定色でフラッシュする（赤=拒否、黄=注意。§6.4）
    fn start_flash(&mut self, rect: SelRect, now: f64, color: egui::Color32) {
        self.flash_rect = Some(rect);
        self.flash_until = now + FLASH_SECS;
        self.flash_color = color;
    }

    /// セルの表示文字列。新規行プレースホルダは空（§4.5: コピー対象は空文字列）
    fn cell_display(&self, adapter: &dyn GridAdapter, row: usize, col: usize) -> String {
        if row >= adapter.rows() {
            String::new()
        } else {
            adapter.cell_text(row, col)
        }
    }

    /// ログ用のセル名（「3行目のX」「新規行のY」）
    fn cell_name(&self, adapter: &dyn GridAdapter, headers: &[&str], cell: CellRef) -> String {
        let col = headers.get(cell.col).copied().unwrap_or("?");
        if cell.row >= adapter.rows() {
            format!("新規行の{col}")
        } else {
            format!("{}行目の{col}", cell.row + 1)
        }
    }

    fn begin_edit_with(&mut self, buf: String) {
        self.grid.begin_edit();
        self.edit_buf = buf;
        self.edit_needs_focus = true;
    }

    /// プレースホルダを含む表示行数で GridState を同期する
    fn sync_rows(&mut self, adapter: &dyn GridAdapter) {
        self.grid.rows = adapter.rows() + adapter.can_append_rows() as usize;
        self.grid.cols = adapter.cols();
        self.grid.clamp_selection();
    }

    /// 編集確定。空のまま確定は「変更なし」（§4.3。Backspace→Enter は no-op）。
    /// 不正値は変更せず理由をログ＋当該セルを赤フラッシュ。
    /// プレースホルダへの確定は「行追加＋値設定」としてアダプタに渡す（§4.5）
    fn commit_edit(
        &mut self,
        adapter: &mut dyn GridAdapter,
        headers: &[&str],
        cell: CellRef,
        now: f64,
    ) {
        let t = self.edit_buf.trim().to_string();
        let outcome = super::commit_cell_text(adapter, cell, &t);
        // セル名は確定後に引く（プレースホルダ確定では行が実データ化し
        // 「N行目のX」と表示できる）
        let name = self.cell_name(adapter, headers, cell);
        match outcome {
            super::CommitOutcome::NoChange => {}
            super::CommitOutcome::Rejected(reason) => {
                self.push_err(format!("「{t}」: {reason}。{name} は変更しませんでした"));
                self.start_flash(
                    SelRect {
                        r0: cell.row,
                        r1: cell.row,
                        c0: cell.col,
                        c1: cell.col,
                    },
                    now,
                    theme::ERROR_RED,
                );
            }
            super::CommitOutcome::Applied { appended } => {
                if appended {
                    self.push_log(format!("新規行を追加し {name} ← {t}"));
                } else {
                    self.push_log(format!("{name} ← {t}"));
                }
                self.sync_rows(adapter);
            }
        }
    }

    /// 選択範囲を TSV コピー（§5.1）。選択なしはログに理由を出す（§4.4）
    fn do_copy(&mut self, ctx: &egui::Context, adapter: &dyn GridAdapter) {
        if !self.grid.active {
            self.push_log("選択がないためコピーをスキップ（セルをクリックして選択してください）");
            return;
        }
        let rect = self.grid.rect();
        let tsv = rect_to_tsv(rect, |r, c| self.cell_display(adapter, r, c));
        ctx.copy_text(tsv);
        self.push_log(format!(
            "コピー: {}行×{}列を TSV でクリップボードへ",
            rect.r1 - rect.r0 + 1,
            rect.c1 - rect.c0 + 1
        ));
    }

    /// TSV ペースト（§5.2: パース → タイル展開 → 検証 all-or-nothing → 適用）。
    /// 選択なしは無言で無視する（§4.4: 貼り付け先が定まっていない操作に
    /// フィードバックは出さない）
    fn do_paste(&mut self, adapter: &mut dyn GridAdapter, text: &str, now: f64) {
        if !self.grid.active {
            return;
        }
        let block = parse_tsv(text);
        if block.is_empty() {
            self.push_log("クリップボードが空のため貼り付けをスキップ");
            return;
        }
        let rect = self.grid.rect();
        let anchor = CellRef {
            row: rect.r0,
            col: rect.c0,
        };
        // 巨大ペーストの暴発防止（§5.2.1）。plan_paste にも同じガードがあるが、
        // サイズ超過は「選択範囲」をフラッシュする決まり（§6.4）のため
        // タイル展開の前にここで判定する
        let orig_rows = block.len();
        let orig_cols = block.iter().map(Vec::len).max().unwrap_or(0);
        if orig_rows.saturating_mul(orig_cols) > MAX_PASTE_CELLS {
            self.push_err(format!(
                "貼り付けを全体拒否: ブロックが大きすぎます（{orig_rows}×{orig_cols} > 上限 {MAX_PASTE_CELLS} セル）"
            ));
            self.start_flash(rect, now, theme::ERROR_RED);
            return;
        }
        // 選択範囲がブロックの整数倍ならタイル展開（Excel 互換。§5.2.2）
        let (sel_rows, sel_cols) = (rect.r1 - rect.r0 + 1, rect.c1 - rect.c0 + 1);
        let block = tile_block(&block, sel_rows, sel_cols);
        let tiled_cols = block.iter().map(Vec::len).max().unwrap_or(0);
        let tiled = block.len() != orig_rows || tiled_cols != orig_cols;
        // 選択サイズとブロックサイズの不一致（タイル倍数でもない）。
        // Excel 互換で「起点から貼る」成功動作だが、選択ミスに気づけるよう
        // ログ＋貼り付け範囲の黄フラッシュ（注意色）で知らせる（§5.2.2）
        let mismatch = !tiled
            && (sel_rows, sel_cols) != (orig_rows, orig_cols)
            && (sel_rows > 1 || sel_cols > 1);
        if tiled {
            self.push_log(format!(
                "コピー元 {orig_rows}×{orig_cols} を選択範囲 {sel_rows}×{sel_cols} にタイル展開"
            ));
        } else if mismatch {
            self.push_log(format!(
                "注意: 選択範囲 {sel_rows}×{sel_cols} とコピー元 {orig_rows}×{orig_cols} のサイズが一致しません。起点から {orig_rows}×{orig_cols} で貼り付けます"
            ));
        }
        let plan = plan_paste(&block, anchor, adapter.rows(), adapter.cols(), |r, c, t| {
            adapter.validate_cell(r, c, t)
        });
        match plan {
            Err(errors) => {
                self.push_err(format!(
                    "貼り付けを全体拒否（all-or-nothing）: 不正 {} 件",
                    errors.len()
                ));
                for e in errors.iter().take(10) {
                    self.push_err(format!("  {e}"));
                }
                if errors.len() > 10 {
                    self.push_err(format!("  …ほか {} 件", errors.len() - 10));
                }
                // 貼り付け先になるはずだった範囲（表内に収まる部分）を赤フラッシュし、
                // 「どこに貼ろうとして失敗したか」を表上で示す（§6.4）
                let fr = SelRect {
                    r0: anchor.row,
                    r1: (anchor.row + block.len().saturating_sub(1))
                        .min(self.grid.rows.saturating_sub(1)),
                    c0: anchor.col,
                    c1: (anchor.col + tiled_cols.max(1) - 1).min(self.grid.cols.saturating_sub(1)),
                };
                self.start_flash(fr, now, theme::ERROR_RED);
            }
            Ok(plan) => {
                self.apply_plan(adapter, plan);
                if mismatch {
                    // 適用後の選択＝貼り付けられた範囲。注意色でフラッシュし、
                    // 「思っていた選択と違う場所・大きさに入った」ことに気づかせる
                    self.start_flash(self.grid.rect(), now, theme::BEST_YELLOW);
                }
            }
        }
    }

    /// ペースト計画の適用。行追加対応テーブルでははみ出し行を自動追加し（§5.2.5）、
    /// 非対応テーブルでははみ出し分を確認なしで切り捨ててログに通知する（§3.4）。
    /// 適用結果は 1 行でログに報告し（§5.2.6）、貼り付けブロックの矩形を
    /// 選択状態にする（§5.3）
    fn apply_plan(&mut self, adapter: &mut dyn GridAdapter, plan: PastePlan) {
        let (applied, dropped, extra_rows);
        if adapter.can_append_rows() {
            applied = plan.set.len();
            dropped = 0;
            extra_rows = plan.extra_rows;
            adapter.apply_block(&plan.set, plan.extra_rows);
        } else {
            let rows = adapter.rows();
            let kept: Vec<_> = plan
                .set
                .iter()
                .filter(|(r, _, _)| *r < rows)
                .cloned()
                .collect();
            applied = kept.len();
            dropped = plan.set.len() - kept.len();
            extra_rows = 0;
            adapter.apply_block(&kept, 0);
        }
        let mut msg = format!("貼り付け: {applied} セル適用");
        if plan.skipped_empty > 0 {
            msg += &format!("、空セル {} 個は既存値維持", plan.skipped_empty);
        }
        if extra_rows > 0 {
            msg += &format!("、{extra_rows} 行を追加");
        }
        if dropped > 0 {
            msg += &format!("、はみ出し {dropped} セルを切り捨て");
        }
        self.push_log(msg);
        self.sync_rows(adapter);
        // 貼り付けたブロック範囲を選択状態にする（Excel と同じ挙動。§5.3。
        // どこに何が入ったかが一目で分かり、続けてコピーや Delete もできる）
        if applied > 0 && plan.block_rows > 0 && plan.block_cols > 0 {
            self.grid.anchor = plan.anchor;
            self.grid.cursor = CellRef {
                row: plan.anchor.row + plan.block_rows - 1,
                col: plan.anchor.col + plan.block_cols - 1,
            };
            self.grid.clamp_selection();
        }
    }

    /// 選択範囲のクリア（Delete）。クリアの意味はアダプタが決める（§3.4）。
    /// プレースホルダ行は対象にならない（§4.5）
    fn clear_selection(&mut self, adapter: &mut dyn GridAdapter) {
        if !self.grid.active {
            return;
        }
        let rect = self.grid.rect();
        let rows = adapter.rows();
        let mut cells = Vec::new();
        for r in rect.r0..=rect.r1.min(rows.saturating_sub(1)) {
            if r >= rows {
                continue;
            }
            for c in rect.c0..=rect.c1 {
                cells.push((r, c));
            }
        }
        if cells.is_empty() {
            return;
        }
        let n = adapter.clear_cells(&cells);
        if n > 0 {
            self.push_log(format!("選択範囲 {n} セルをクリア"));
        } else {
            self.push_log("このテーブルは選択範囲のクリアに対応していません");
        }
        self.sync_rows(adapter);
    }

    /// 行削除メニューのラベル。対象行数を明示し、
    /// 「複数行選択したのに 1 行しか消えない／逆」の驚きを防ぐ（§4.6）
    fn delete_menu_label(&self, adapter: &dyn GridAdapter) -> String {
        let rect = self.grid.rect();
        let rows = adapter.rows();
        let n = if rows == 0 || rect.r0 >= rows {
            0
        } else {
            rect.r1.min(rows - 1) - rect.r0 + 1
        };
        if n <= 1 {
            "この行を削除".to_string()
        } else {
            format!("選択した {n} 行を削除")
        }
    }

    /// 行削除（右クリックメニュー／Ctrl+Delete。§4.6）。対象は選択が跨ぐ実データ行
    /// （プレースホルダは対象外）。参照中の行が 1 つでもあれば全体拒否
    /// （ペーストと同じ all-or-nothing）
    fn delete_selected_rows(&mut self, adapter: &mut dyn GridAdapter, now: f64) {
        if !self.grid.active || !adapter.can_delete_rows() {
            return;
        }
        let rect = self.grid.rect();
        let rows = adapter.rows();
        if rows == 0 || rect.r0 >= rows {
            self.push_log("削除対象の行がありません（新規行は削除できません）");
            return;
        }
        let r0 = rect.r0;
        let r1 = rect.r1.min(rows - 1);
        let blocked: Vec<(usize, String)> = (r0..=r1)
            .filter_map(|r| adapter.validate_row_deletion(r).err().map(|e| (r, e)))
            .collect();
        if !blocked.is_empty() {
            self.push_err(format!(
                "行削除を全体拒否: 削除できない行が {} 行含まれています",
                blocked.len()
            ));
            for (r, reason) in blocked.iter().take(10) {
                self.push_err(format!("  {}行目: {}", r + 1, reason));
            }
            if blocked.len() > 10 {
                self.push_err(format!("  …ほか {} 件", blocked.len() - 10));
            }
            self.start_flash(
                SelRect {
                    r0,
                    r1,
                    c0: 0,
                    c1: self.grid.cols.saturating_sub(1),
                },
                now,
                theme::ERROR_RED,
            );
            return;
        }
        let targets: Vec<usize> = (r0..=r1).collect();
        adapter.delete_rows(&targets);
        self.push_log(format!(
            "{} 行を削除（{}〜{} 行目）",
            r1 - r0 + 1,
            r0 + 1,
            r1 + 1
        ));
        self.sync_rows(adapter);
        // 削除位置に続く行を行選択状態にする（Excel と同じ。§4.6。
        // 最終行まで消えた場合は clamp によりプレースホルダが選択される）
        self.grid.select_row(r0, false);
        self.grid.clamp_selection();
    }

    /// 全選択（実データ行のみ。新規行プレースホルダは含めない。§4.5）
    fn select_all_real(&mut self, adapter: &dyn GridAdapter) {
        if adapter.rows() == 0 {
            return;
        }
        self.grid.select_all();
        if self.grid.active {
            self.grid.cursor.row = adapter.rows() - 1;
        }
    }

    /// 選択モード時のグローバル入力（§7.1: 編集モード中は一切処理しない。
    /// TextEdit が Ctrl+C/V・矢印・文字入力を消費する）
    fn handle_events(&mut self, ctx: &egui::Context, adapter: &mut dyn GridAdapter) {
        if self.grid.editing.is_some() {
            return;
        }
        let events = ctx.input(|i| i.events.clone());
        for e in events {
            match e {
                egui::Event::Copy | egui::Event::Cut => self.do_copy(ctx, adapter),
                egui::Event::Paste(t) => {
                    let now = ctx.input(|i| i.time);
                    self.do_paste(adapter, &t, now);
                }
                egui::Event::Text(t) => {
                    // 文字キー入力で即編集開始（Excel 同様。§4.3）。制御文字は無視。
                    // 選択がない間は編集開始しない（§4.4）
                    let clean: String = t.chars().filter(|ch| !ch.is_control()).collect();
                    if !clean.is_empty() && self.grid.active {
                        self.begin_edit_with(clean);
                    }
                }
                egui::Event::Key {
                    key,
                    pressed: true,
                    modifiers,
                    ..
                } => match key {
                    egui::Key::A if modifiers.command => {
                        self.select_all_real(adapter);
                    }
                    // 行削除は Ctrl+Delete（Delete=クリアの「強い版」として段階的。
                    // Excel の Ctrl+マイナスは egui の UI ズームと競合するため不採用。§4.6）
                    egui::Key::Delete if modifiers.command => {
                        let now = ctx.input(|i| i.time);
                        self.delete_selected_rows(adapter, now);
                    }
                    egui::Key::Delete => self.clear_selection(adapter),
                    egui::Key::ArrowUp => self.grid.move_cursor(-1, 0, modifiers.shift),
                    egui::Key::ArrowDown => self.grid.move_cursor(1, 0, modifiers.shift),
                    egui::Key::ArrowLeft => self.grid.move_cursor(0, -1, modifiers.shift),
                    egui::Key::ArrowRight => self.grid.move_cursor(0, 1, modifiers.shift),
                    egui::Key::Tab => {
                        self.grid
                            .move_cursor(0, if modifiers.shift { -1 } else { 1 }, false)
                    }
                    egui::Key::Enter => {
                        self.grid
                            .move_cursor(if modifiers.shift { -1 } else { 1 }, 0, false)
                    }
                    egui::Key::F2 => {
                        if self.grid.active {
                            self.begin_edit_with(self.cell_display(
                                adapter,
                                self.grid.anchor.row,
                                self.grid.anchor.col,
                            ));
                        }
                    }
                    // Excel 準拠: Backspace はアクティブセルを空の状態で編集開始
                    // （範囲クリアは Delete のみ。§4.3）
                    egui::Key::Backspace => {
                        if self.grid.active {
                            self.begin_edit_with(String::new());
                        }
                    }
                    egui::Key::Escape if self.grid.active => {
                        self.grid.deactivate();
                        self.push_log("選択を解除（Esc）");
                    }
                    _ => {}
                },
                _ => {}
            }
        }
    }

    /// 行ヘッダ（ID 列）の描画。データセルではないためフラットなラベルで描き、
    /// クリック＝行全体選択、ドラッグ/Shift＝行範囲拡張（Excel の行番号ヘッダ）。
    /// プレースホルダ行は「＋」（§4.5）
    fn row_header_cell(&mut self, ui: &mut egui::Ui, adapter: &mut dyn GridAdapter, row: usize) {
        let rect = ui.available_rect_before_wrap();
        let resp = ui.interact(
            rect,
            ui.id().with(("grid_row_header", row)),
            egui::Sense::click_and_drag(),
        );
        // 右クリックメニュー（発見可能性のため。Ctrl+Delete と同じ操作）。
        // 選択外の行を右クリックした場合は、その行を選択してからメニューを出す（§4.6）
        let is_real = row < adapter.rows();
        if is_real && adapter.can_delete_rows() {
            if resp.secondary_clicked() {
                let sel = self.grid.rect();
                if !self.grid.active || !(sel.r0..=sel.r1).contains(&row) {
                    self.grid.select_row(row, false);
                }
            }
            let label = self.delete_menu_label(adapter);
            resp.context_menu(|ui| {
                self.menu_open = true;
                if ui.button(label.clone()).clicked() {
                    let now = ui.input(|i| i.time);
                    self.delete_selected_rows(adapter, now);
                    ui.close();
                }
            });
        }
        let sp = ui.spacing().item_spacing;
        let g = rect.expand2(egui::vec2(sp.x * 0.5, sp.y * 0.5));
        // ヘッダ地は gray-100、罫線は本体と共有（§6.3）
        ui.painter()
            .rect_filled(g, egui::CornerRadius::ZERO, theme::GRAY_100);
        let grid_stroke = egui::Stroke::new(1.0_f32, theme::GRAY_200);
        ui.painter()
            .line_segment([g.right_top(), g.right_bottom()], grid_stroke);
        ui.painter()
            .line_segment([g.left_bottom(), g.right_bottom()], grid_stroke);
        // 表の左外周は行ヘッダ列が担当する（各セルは右・下辺のみ描くため。§6.1）
        ui.painter()
            .line_segment([g.left_top(), g.left_bottom()], grid_stroke);
        // 行が選択に含まれる間は薄いハイライトで示す（Excel の行番号ヘッダと同じ合図。§6.3）
        let sel = self.grid.rect();
        if self.grid.active && (sel.r0..=sel.r1).contains(&row) {
            ui.painter().rect_filled(
                g,
                egui::CornerRadius::ZERO,
                theme::translucent(theme::BLUE_300, 80),
            );
        }
        let font = egui::TextStyle::Body.resolve(ui.style());
        ui.painter().text(
            rect.left_center() + egui::vec2(4.0, 0.0),
            egui::Align2::LEFT_CENTER,
            if is_real {
                adapter.row_label(row)
            } else {
                "＋".to_string()
            },
            font,
            if is_real {
                theme::GRAY_700
            } else {
                theme::GRAY_600
            },
        );
        let (shift, primary_pressed, primary_down, pointer_pos) = ui.input(|i| {
            (
                i.modifiers.shift,
                i.pointer.primary_pressed(),
                i.pointer.primary_down(),
                i.pointer.interact_pos(),
            )
        });
        let contains_pointer = pointer_pos.is_some_and(|p| rect.contains(p));
        // 押下は hovered()（レイヤ考慮あり）、ドラッグ継続の追従のみ座標判定（§7.2/7.2b）
        if primary_pressed && resp.hovered() {
            self.grid.select_row(row, shift);
            self.drag_selecting = true;
            self.drag_rows = true;
            self.drag_cols = false;
        } else if primary_down && self.drag_selecting && self.drag_rows && contains_pointer {
            self.grid.select_row(row, true);
        }
    }

    /// 左上コーナー（ID 見出し）の描画。クリック＝全選択（実データ行のみ。§6.3）
    fn corner_header_cell(&mut self, ui: &mut egui::Ui, adapter: &dyn GridAdapter) {
        let rect = ui.available_rect_before_wrap();
        let resp = ui.interact(
            rect,
            ui.id().with("grid_corner_header"),
            egui::Sense::click(),
        );
        let sp = ui.spacing().item_spacing;
        let g = rect.expand2(egui::vec2(sp.x * 0.5, sp.y * 0.5));
        ui.painter()
            .rect_filled(g, egui::CornerRadius::ZERO, theme::GRAY_100);
        let grid_stroke = egui::Stroke::new(1.0_f32, theme::GRAY_200);
        ui.painter()
            .line_segment([g.right_top(), g.right_bottom()], grid_stroke);
        ui.painter()
            .line_segment([g.left_bottom(), g.right_bottom()], grid_stroke);
        // コーナーは格子の左上角を兼ねるため上辺・左辺も描く
        ui.painter()
            .line_segment([g.left_top(), g.right_top()], grid_stroke);
        ui.painter()
            .line_segment([g.left_top(), g.left_bottom()], grid_stroke);
        let font = egui::TextStyle::Body.resolve(ui.style());
        ui.painter().text(
            rect.left_center() + egui::vec2(4.0, 0.0),
            egui::Align2::LEFT_CENTER,
            "ID",
            font,
            theme::GRAY_900,
        );
        let primary_pressed = ui.input(|i| i.pointer.primary_pressed());
        if primary_pressed && resp.hovered() {
            self.select_all_real(adapter);
        }
    }

    /// 列ヘッダの描画。行ヘッダと対称: 列が選択に含まれる間は薄いハイライト、
    /// クリック＝列全体選択、ドラッグ/Shift＝列範囲拡張（§6.3）
    fn col_header_cell(&mut self, ui: &mut egui::Ui, headers: &[&str], col: usize) {
        let rect = ui.available_rect_before_wrap();
        let resp = ui.interact(
            rect,
            ui.id().with(("grid_col_header", col)),
            egui::Sense::click_and_drag(),
        );
        let sp = ui.spacing().item_spacing;
        let g = rect.expand2(egui::vec2(sp.x * 0.5, sp.y * 0.5));
        ui.painter()
            .rect_filled(g, egui::CornerRadius::ZERO, theme::GRAY_100);
        let grid_stroke = egui::Stroke::new(1.0_f32, theme::GRAY_200);
        ui.painter()
            .line_segment([g.right_top(), g.right_bottom()], grid_stroke);
        ui.painter()
            .line_segment([g.left_bottom(), g.right_bottom()], grid_stroke);
        // 表の上外周はヘッダ行が担当する（§6.1）
        ui.painter()
            .line_segment([g.left_top(), g.right_top()], grid_stroke);
        let sel = self.grid.rect();
        if self.grid.active && (sel.c0..=sel.c1).contains(&col) {
            ui.painter().rect_filled(
                g,
                egui::CornerRadius::ZERO,
                theme::translucent(theme::BLUE_300, 80),
            );
        }
        let font = egui::TextStyle::Body.resolve(ui.style());
        ui.painter().text(
            rect.left_center() + egui::vec2(4.0, 0.0),
            egui::Align2::LEFT_CENTER,
            headers.get(col).copied().unwrap_or(""),
            font,
            theme::GRAY_900,
        );
        let (shift, primary_pressed, primary_down, pointer_pos) = ui.input(|i| {
            (
                i.modifiers.shift,
                i.pointer.primary_pressed(),
                i.pointer.primary_down(),
                i.pointer.interact_pos(),
            )
        });
        let contains_pointer = pointer_pos.is_some_and(|p| rect.contains(p));
        if primary_pressed && resp.hovered() {
            self.grid.select_col(col, shift);
            self.drag_selecting = true;
            self.drag_cols = true;
            self.drag_rows = false;
        } else if primary_down && self.drag_selecting && self.drag_cols && contains_pointer {
            self.grid.select_col(col, true);
        }
    }

    /// 選択モードのデータセル描画（スプレッドシート風＋クリック/ドラッグ判定）。
    /// 罫線は各セルが右辺・下辺のみを描き、隣接セルと 1 本の線を共有する（§6.1）。
    /// 選択は Excel 式: 範囲の外周を青枠で囲い、中を薄く塗り、
    /// アクティブセルだけ白抜き（§6.2）
    fn select_mode_cell(
        &mut self,
        ui: &mut egui::Ui,
        adapter: &mut dyn GridAdapter,
        cell: CellRef,
    ) {
        let rect = ui.available_rect_before_wrap();
        let resp = ui.interact(
            rect,
            ui.id().with(("grid_cell", cell.row, cell.col)),
            egui::Sense::click_and_drag(),
        );
        // セル間スペーシングを跨いで罫線・塗りが連続するよう半スペース分広げる（§7.5）
        let sp = ui.spacing().item_spacing;
        let g = rect.expand2(egui::vec2(sp.x * 0.5, sp.y * 0.5));
        let grid_stroke = egui::Stroke::new(1.0_f32, theme::GRAY_200);
        ui.painter()
            .line_segment([g.right_top(), g.right_bottom()], grid_stroke);
        ui.painter()
            .line_segment([g.left_bottom(), g.right_bottom()], grid_stroke);
        // 新規行プレースホルダはごく薄い地色で「まだデータではない」ことを示す（§6.1）
        let is_real = cell.row < adapter.rows();
        if !is_real {
            ui.painter().rect_filled(
                g,
                egui::CornerRadius::ZERO,
                theme::translucent(theme::GRAY_100, 140),
            );
        }
        let sel = self.grid.rect();
        let selected = self.grid.active && sel.contains(cell.row, cell.col);
        let is_anchor = self.grid.active && self.grid.anchor == cell;
        // 選択範囲は薄い blue-300 で塗る。アクティブセル（anchor）だけ白抜きにして
        // 「入力が向かう先」を示す（§6.2）
        if selected && !is_anchor {
            ui.painter().rect_filled(
                g,
                egui::CornerRadius::ZERO,
                theme::translucent(theme::BLUE_300, 70),
            );
        }
        // 選択範囲の外周に blue-500 の枠。単一セル選択ではこれがカーソル枠になる（§6.2）
        if selected {
            let b = egui::Stroke::new(2.0_f32, theme::BLUE_500);
            if cell.row == sel.r0 {
                ui.painter().line_segment([g.left_top(), g.right_top()], b);
            }
            if cell.row == sel.r1 {
                ui.painter()
                    .line_segment([g.left_bottom(), g.right_bottom()], b);
            }
            if cell.col == sel.c0 {
                ui.painter()
                    .line_segment([g.left_top(), g.left_bottom()], b);
            }
            if cell.col == sel.c1 {
                ui.painter()
                    .line_segment([g.right_top(), g.right_bottom()], b);
            }
        }
        // フィードバックフラッシュ: 対象範囲を色付きでフェードアウト表示（§6.4）
        if let Some(fr) = self.flash_rect {
            let now = ui.input(|i| i.time);
            if now < self.flash_until && fr.contains(cell.row, cell.col) {
                let t = ((self.flash_until - now) / FLASH_SECS).clamp(0.0, 1.0) as f32;
                let alpha = (110.0 * t) as u8;
                ui.painter().rect_filled(
                    g,
                    egui::CornerRadius::ZERO,
                    theme::translucent(self.flash_color, alpha),
                );
            }
        }
        let font = egui::TextStyle::Body.resolve(ui.style());
        ui.painter().text(
            rect.left_center() + egui::vec2(4.0, 0.0),
            egui::Align2::LEFT_CENTER,
            self.cell_display(adapter, cell.row, cell.col),
            font,
            theme::GRAY_700,
        );
        // ドラッグ中は egui がドラッグ元以外の hovered() を false にするため、
        // 範囲選択の追従はポインタ座標とセル矩形の包含判定で行う（§7.2）
        let (shift, primary_pressed, primary_down, pointer_pos) = ui.input(|i| {
            (
                i.modifiers.shift,
                i.pointer.primary_pressed(),
                i.pointer.primary_down(),
                i.pointer.interact_pos(),
            )
        });
        let contains_pointer = pointer_pos.is_some_and(|p| rect.contains(p));
        if resp.double_clicked() {
            self.grid.click(cell, false);
            self.begin_edit_with(self.cell_display(adapter, cell.row, cell.col));
        } else if primary_pressed && resp.hovered() {
            // 押下判定は resp.hovered()（egui のレイヤ考慮あり）で行うこと。
            // 生のポインタ座標×矩形だと、表の上に浮いたメニュー等への
            // クリックが真下のセルに貫通し、選択が意図せず潰れる（§7.2b）
            self.grid.click(cell, shift);
            self.drag_selecting = true;
            self.drag_rows = false;
            self.drag_cols = false;
        } else if primary_down
            && self.drag_selecting
            && !self.drag_rows
            && !self.drag_cols
            && contains_pointer
        {
            self.grid.drag_to(cell);
        }
        // セル上の右クリックにも行削除メニューを出す（セルドラッグで行範囲を
        // 選んでから右クリックする流れに対応。§4.6）。選択外のセルならまず選択を移す
        if is_real && adapter.can_delete_rows() {
            if resp.secondary_clicked() {
                let sel = self.grid.rect();
                if !self.grid.active || !sel.contains(cell.row, cell.col) {
                    self.grid.click(cell, false);
                }
            }
            let label = self.delete_menu_label(adapter);
            resp.context_menu(|ui| {
                self.menu_open = true;
                if ui.button(label.clone()).clicked() {
                    let now = ui.input(|i| i.time);
                    self.delete_selected_rows(adapter, now);
                    ui.close();
                }
            });
        }
    }

    /// 1 行削除の 🗑 ボタンセル（`delete_buttons` 有効時のみ。§4.6 の
    /// メニュー削除と同じ検証を通す）。削除は行数変化を避けるため
    /// テーブル描画後に処理する
    fn delete_button_cell(&mut self, ui: &mut egui::Ui, adapter: &dyn GridAdapter, row: usize) {
        if row >= adapter.rows() {
            return; // プレースホルダ行にはボタンを出さない
        }
        let deletable = adapter.validate_row_deletion(row);
        let resp = ui.add_enabled(deletable.is_ok(), egui::Button::new("🗑").small());
        match deletable {
            Err(reason) => {
                resp.on_hover_text(reason);
            }
            Ok(()) => {
                if resp.on_hover_text("この行を削除").clicked() {
                    self.pending_row_delete = Some(row);
                }
            }
        }
    }

    /// 編集モードのセル描画（TextEdit＋確定/キャンセル判定）。
    /// 選択モードのセルと同じ矩形・同じ左余白（4px）へ `put` で固定し、
    /// モード切替時に文字が動かないようにする（§7.4）
    fn edit_mode_cell(
        &mut self,
        ui: &mut egui::Ui,
        adapter: &mut dyn GridAdapter,
        headers: &[&str],
        cell: CellRef,
    ) {
        let rect = ui.available_rect_before_wrap();
        let resp = ui.put(
            rect,
            egui::TextEdit::singleline(&mut self.edit_buf)
                .font(egui::TextStyle::Body)
                .vertical_align(egui::Align::Center)
                .margin(egui::Margin::symmetric(4, 0))
                .clip_text(false),
        );
        if self.edit_needs_focus {
            resp.request_focus();
            self.edit_needs_focus = false;
        }
        let (enter, tab, esc) = ui.input(|i| {
            (
                i.key_pressed(egui::Key::Enter),
                i.key_pressed(egui::Key::Tab),
                i.key_pressed(egui::Key::Escape),
            )
        });
        if resp.lost_focus() {
            self.grid.end_edit();
            if esc {
                self.push_log("編集をキャンセル（Esc）");
            } else {
                let now = ui.input(|i| i.time);
                self.commit_edit(adapter, headers, cell, now);
                if enter {
                    self.grid.move_cursor(1, 0, false);
                } else if tab {
                    self.grid.move_cursor(0, 1, false);
                }
            }
            // Tab 確定時に egui のフォーカスが次のウィジェット（ボタン等）へ移ると、
            // 直後の Enter がそのボタンを押してしまう。確定後はフォーカスを放棄する（§7.4）
            ui.ctx().memory_mut(|m| {
                if let Some(f) = m.focused() {
                    m.surrender_focus(f);
                }
            });
        }
    }

    /// グリッドを描画し、入力を処理する。毎フレーム呼ぶこと。
    /// `headers` はデータ列の見出し（`adapter.cols()` と同数）。
    pub fn show(&mut self, ui: &mut egui::Ui, adapter: &mut dyn GridAdapter, headers: &[&str]) {
        let ctx = ui.ctx().clone();
        // 行数が増減したフレームでも選択が表内に収まるよう毎フレーム同期する（§7.7）
        self.sync_rows(adapter);
        if !ctx.input(|i| i.pointer.primary_down()) {
            self.drag_selecting = false;
        }
        // メニュー表示状態はフレームごとに再判定（開いていれば描画中に true になる）
        self.menu_open = false;
        // フラッシュ中はアニメーションのため再描画を要求し、終了したら消す
        // （TONMANUAL §8: アニメーション中のみ再描画）
        if self.flash_rect.is_some() {
            if ctx.input(|i| i.time) < self.flash_until {
                ctx.request_repaint();
            } else {
                self.flash_rect = None;
            }
        }
        self.handle_events(&ctx, adapter);

        use egui_extras::{Column, TableBuilder};
        let n = self.grid.rows;
        // 行高は固定 px でなくフォントから導出する（TONMANUAL §4。§6.5）
        let row_h = theme::table_row_height(ui);
        // スプレッドシート風: ストライプではなく白地＋共有罫線で見せる（§6.1）。
        // scope で囲み、テーブル領域（スクロールバー含む）の矩形を
        // 表外クリック判定用に取得する（§7.3）
        let delete_buttons = self.delete_buttons && adapter.can_delete_rows();
        let table_scope = ui.scope(|ui| {
            let mut builder = TableBuilder::new(ui)
                .striped(false)
                .column(Column::auto().at_least(40.0))
                .columns(Column::initial(90.0), adapter.cols());
            if delete_buttons {
                builder = builder.column(Column::auto());
            }
            builder
                .header(row_h, |mut h| {
                    h.col(|ui| {
                        self.corner_header_cell(ui, adapter);
                    });
                    for c in 0..adapter.cols() {
                        h.col(|ui| {
                            self.col_header_cell(ui, headers, c);
                        });
                    }
                    if delete_buttons {
                        h.col(|_ui| {});
                    }
                })
                .body(|body| {
                    body.rows(row_h, n, |mut row| {
                        let r = row.index();
                        // 行ヘッダ（グリッド座標系の外。§3.2）
                        row.col(|ui| {
                            self.row_header_cell(ui, adapter, r);
                        });
                        // データセル（グリッド座標系: col 0..cols）
                        for c in 0..adapter.cols() {
                            row.col(|ui| {
                                let cell = CellRef { row: r, col: c };
                                if self.grid.editing == Some(cell) {
                                    self.edit_mode_cell(ui, adapter, headers, cell);
                                } else {
                                    self.select_mode_cell(ui, adapter, cell);
                                }
                            });
                        }
                        if delete_buttons {
                            row.col(|ui| {
                                self.delete_button_cell(ui, adapter, r);
                            });
                        }
                    });
                });
        });
        self.table_rect = table_scope.response.rect;

        // 🗑 ボタンの 1 行削除（描画後に処理して行数変化の齟齬を避ける）
        if let Some(r) = self.pending_row_delete.take() {
            if adapter.validate_row_deletion(r).is_ok() {
                adapter.delete_rows(&[r]);
                self.push_log(format!("{} 行目を削除", r + 1));
                self.sync_rows(adapter);
                // 削除位置に続く行を行選択状態にする（§4.6 と同じ後処理）
                self.grid.select_row(r, false);
                self.grid.clamp_selection();
            }
        }

        // 表外クリックで選択を完全解除する（§4.4。編集中・メニュー表示中は対象外）。
        // 「表外」= テーブル領域（スクロールバー含む外形矩形）の外（§7.3。
        // セル単位の判定にするとスクロールバー操作で選択が消えてしまう）
        let (pressed, pos) = ctx.input(|i| (i.pointer.primary_pressed(), i.pointer.interact_pos()));
        if pressed
            && pos.is_some_and(|p| !self.table_rect.contains(p))
            && self.grid.editing.is_none()
            && !self.menu_open
            && self.grid.active
        {
            self.grid.deactivate();
        }
    }
}
