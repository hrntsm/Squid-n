//! ステータスバー。
//!
//! `panels` からの構造分割。アルゴリズム変更は行わない。

use super::*;

/// ステータスバーのドック/パネル切替アイコンの共通クリック挙動（Zed 風）。
/// 対象ドックが開いていて対象パネルが既にアクティブなら閉じて `false` を返す。
/// それ以外はドックを開いて `true` を返す（呼び出し側は `true` のときのみ
/// 対象パネル/タブをアクティブにする）。
fn toggle_dock_icon(dock_open: &mut bool, is_active: bool) -> bool {
    if *dock_open && is_active {
        *dock_open = false;
        false
    } else {
        *dock_open = true;
        true
    }
}

impl App {
    /// 下部ステータスバー。
    ///
    /// 部材/節点/断面サマリは常に右端に見えている必要がある一方、左側（ファイル名・
    /// 解析状況・エラーメッセージ）はいくらでも長くなりうる（特に ST-Bridge 取込警告は
    /// 複数件を連結した長文）。`ui.horizontal` 1本に全部を並べると、horizontal レイアウトの
    /// 子 `Ui` は主軸方向の幅を事実上無制限に確保するため、エラーメッセージに
    /// `Label::truncate()` を付けても truncate の基準となる幅がなく効かず、右側のサマリと
    /// 重なって表示されてしまう。そのためサマリの表示幅を先に採寸し、行を「左ゾーン
    /// （明示的に幅を制限する）」と「右ゾーン（サマリ専用）」へ矩形分割してから描画する。
    pub(crate) fn status_bar(&mut self, ui: &mut egui::Ui) {
        let summary = format!(
            "部材 {}. 節点 {}. 断面 {}.",
            self.model.elements.len(),
            self.model.nodes.len(),
            self.model.sections.len()
        );
        let body_font = egui::TextStyle::Body.resolve(ui.style());
        let summary_width = ui
            .painter()
            .layout_no_wrap(summary.clone(), body_font.clone(), crate::theme::GRAY_700)
            .size()
            .x;
        // 右ゾーンはサマリに加えて右ドックのパネル切替アイコン（🔍・⚙）も描くため、
        // アイコン2個分の幅＋ボタン余白＋アイコン間の間隔ぶんを確保幅に含める
        // （不足すると左ゾーンと重なる）。
        let icon_width = ui
            .painter()
            .layout_no_wrap("🔍".to_string(), body_font, crate::theme::GRAY_700)
            .size()
            .x
            + ui.spacing().button_padding.x * 2.0;
        let toggle_width = icon_width * 2.0 + ui.spacing().item_spacing.x;

        let row_rect = ui.available_rect_before_wrap();
        let gap = ui.spacing().item_spacing.x;
        let right_width = summary_width + gap + toggle_width;
        let right_rect = egui::Rect::from_min_max(
            egui::pos2(
                (row_rect.max.x - right_width - gap).max(row_rect.min.x),
                row_rect.min.y,
            ),
            row_rect.max,
        );
        let left_rect = egui::Rect::from_min_max(
            row_rect.min,
            egui::pos2((right_rect.min.x - gap).max(row_rect.min.x), row_rect.max.y),
        );

        #[allow(deprecated)]
        ui.allocate_ui_at_rect(left_rect, |ui| {
            ui.horizontal(|ui| {
                // ドック/パネル切替アイコン（Zed 風）。対象ドックが開いていて対象パネルが
                // アクティブなら閉じる。それ以外は開いてそのパネルをアクティブにする。
                let is_nav_active = self.left_dock_open && self.left_panel == LeftPanel::Navigator;
                if ui
                    .selectable_label(is_nav_active, "🗂")
                    .on_hover_text("ナビゲータ")
                    .clicked()
                    && toggle_dock_icon(&mut self.left_dock_open, is_nav_active)
                {
                    self.left_panel = LeftPanel::Navigator;
                }
                let is_draw_active = self.left_dock_open && self.left_panel == LeftPanel::DrawTools;
                if ui
                    .selectable_label(is_draw_active, "✏")
                    .on_hover_text("作成パレット")
                    .clicked()
                    && toggle_dock_icon(&mut self.left_dock_open, is_draw_active)
                {
                    self.left_panel = LeftPanel::DrawTools;
                }
                // 左ドック用と下ドック用のアイコン群の間に区切りを入れ、
                // どのアイコンがどの領域を操作するのかを見分けられるようにする。
                ui.separator();
                let is_log_active = self.bottom_dock_open && self.bottom_tab == BottomTab::Log;
                if ui
                    .selectable_label(is_log_active, "📜")
                    .on_hover_text("ログ")
                    .clicked()
                    && toggle_dock_icon(&mut self.bottom_dock_open, is_log_active)
                {
                    self.bottom_tab = BottomTab::Log;
                }
                let is_model_active = self.bottom_dock_open && self.bottom_tab == BottomTab::Model;
                if ui
                    .selectable_label(is_model_active, "📋")
                    .on_hover_text("モデル表")
                    .clicked()
                    && toggle_dock_icon(&mut self.bottom_dock_open, is_model_active)
                {
                    self.bottom_tab = BottomTab::Model;
                }
                let is_loads_active = self.bottom_dock_open && self.bottom_tab == BottomTab::Loads;
                if ui
                    .selectable_label(is_loads_active, "⚡")
                    .on_hover_text("荷重表")
                    .clicked()
                    && toggle_dock_icon(&mut self.bottom_dock_open, is_loads_active)
                {
                    self.bottom_tab = BottomTab::Loads;
                }
                let is_prep_active =
                    self.bottom_dock_open && self.bottom_tab == BottomTab::Preparation;
                if ui
                    .selectable_label(is_prep_active, "🛠")
                    .on_hover_text("準備計算の結果")
                    .clicked()
                    && toggle_dock_icon(&mut self.bottom_dock_open, is_prep_active)
                {
                    self.bottom_tab = BottomTab::Preparation;
                }
                let is_diag_active =
                    self.bottom_dock_open && self.bottom_tab == BottomTab::Diagnostics;
                if ui
                    .selectable_label(is_diag_active, "⚠")
                    .on_hover_text("診断")
                    .clicked()
                    && toggle_dock_icon(&mut self.bottom_dock_open, is_diag_active)
                {
                    self.bottom_tab = BottomTab::Diagnostics;
                }
                ui.separator();
                // プロジェクトファイル名 + 未保存マーカー
                let file_label = self
                    .project_path
                    .as_ref()
                    .and_then(|p| p.file_name())
                    .map(|n| n.to_string_lossy().to_string())
                    .unwrap_or_else(|| "(未保存プロジェクト)".to_string());
                let marker = if self.staleness.unsaved_changes {
                    " ●"
                } else {
                    ""
                };
                ui.label(format!("{}{}", file_label, marker));
                ui.separator();
                // バックグラウンド解析ジョブの実行状況
                if let Some(job) = &self.job {
                    let elapsed = job.started.elapsed().unwrap_or_default().as_secs_f64();
                    ui.colored_label(
                        crate::theme::GOOD_GREEN,
                        format!("⏳ {} 実行中… {:.0}s", job.label, elapsed),
                    );
                    ui.separator();
                }
                // stale アイコン
                if self.staleness.results_stale {
                    ui.colored_label(crate::theme::BEST_YELLOW, "⚠ stale");
                } else if self.results.is_some() {
                    ui.colored_label(crate::theme::GOOD_GREEN, "✓ 最新");
                } else {
                    ui.colored_label(crate::theme::GRAY_600, "▷ 未実行");
                }
                if let Some(err) = &self.last_error {
                    ui.separator();
                    // ST-Bridge 取込警告（複数件を \n 区切りで連結）など改行を含む
                    // メッセージは1行に畳んでから truncate する（\n はレイアウト上
                    // 明示的な改行として扱われ、行の高さ・幅の見積りが崩れるため）。
                    // 全文はホバーで表示する。クリックでログパネルを開けるようにする
                    // （エラーの詳細な経緯はログに残っているため）。
                    let one_line = err.replace('\n', " ");
                    let clicked = ui
                        .add(
                            egui::Label::new(
                                egui::RichText::new(format!("⚠ {}", one_line))
                                    .color(crate::theme::ERROR_RED),
                            )
                            .truncate()
                            .sense(egui::Sense::click()),
                        )
                        .on_hover_text(format!("{}\n\nクリックでログを開く", err))
                        .clicked();
                    if clicked {
                        self.bottom_dock_open = true;
                    }
                }
                // last_error（赤・処理を止める）とは別枠の注意事項（例: 精算周期
                // (SemiPrecise)選択時に固有値解析が未実行で EX/EY が未更新である旨）。
                // 情報色（BEST_YELLOW）で表示し、解析自体は継続してよいことを示す。
                if let Some(notice) = &self.last_notice {
                    ui.separator();
                    let one_line = notice.replace('\n', " ");
                    ui.add(
                        egui::Label::new(
                            egui::RichText::new(format!("ℹ {}", one_line))
                                .color(crate::theme::BEST_YELLOW),
                        )
                        .truncate(),
                    )
                    .on_hover_text(notice);
                }
            });
        });

        #[allow(deprecated)]
        ui.allocate_ui_at_rect(right_rect, |ui| {
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // サマリの右に配置（right_to_left のため先に追加する）。左ゾーンと同じ
                // toggle_dock_icon 方式（アクティブなら閉じる／それ以外は開いてそのパネルを
                // アクティブにする）で右ドックのパネルを切り替える。
                let is_analysis_active =
                    self.right_dock_open && self.right_panel == RightPanel::Analysis;
                if ui
                    .selectable_label(is_analysis_active, "⚙")
                    .on_hover_text("② 解析（実行）")
                    .clicked()
                    && toggle_dock_icon(&mut self.right_dock_open, is_analysis_active)
                {
                    self.right_panel = RightPanel::Analysis;
                }
                let is_prep_panel_active =
                    self.right_dock_open && self.right_panel == RightPanel::Preparation;
                if ui
                    .selectable_label(is_prep_panel_active, "🛠")
                    .on_hover_text("① 準備計算（解析条件の入力・実行）")
                    .clicked()
                    && toggle_dock_icon(&mut self.right_dock_open, is_prep_panel_active)
                {
                    self.right_panel = RightPanel::Preparation;
                }
                let is_inspector_active =
                    self.right_dock_open && self.right_panel == RightPanel::Inspector;
                if ui
                    .selectable_label(is_inspector_active, "🔍")
                    .on_hover_text("インスペクタ")
                    .clicked()
                    && toggle_dock_icon(&mut self.right_dock_open, is_inspector_active)
                {
                    self.right_panel = RightPanel::Inspector;
                }
                ui.label(summary);
            });
        });
    }
}
