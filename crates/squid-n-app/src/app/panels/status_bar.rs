//! ステータスバー。
//!
//! `panels` からの構造分割。アルゴリズム変更は行わない。
//! 下ドックの切替アイコンとファイル名・ジョブ・stale・エラー表示を担う。
//! 左右ドックの切替は [`super::activity_bar`] が担う。

use super::*;

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
            self.core.model.elements.len(),
            self.core.model.nodes.len(),
            self.core.model.sections.len()
        );
        let body_font = egui::TextStyle::Body.resolve(ui.style());
        let summary_width = ui
            .painter()
            .layout_no_wrap(summary.clone(), body_font, crate::theme::WHITE)
            .size()
            .x;

        let row_rect = ui.available_rect_before_wrap();
        let gap = ui.spacing().item_spacing.x;
        let right_width = summary_width + gap;
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
                // 下ドック切替アイコン。対象ドックが開いていて対象タブが
                // アクティブなら閉じる。それ以外は開いてそのタブをアクティブにする。
                let is_log_active =
                    self.ui.view.bottom_dock_open && self.ui.view.bottom_tab == BottomTab::Log;
                if ui
                    .selectable_label(is_log_active, "📜")
                    .on_hover_text("ログ")
                    .clicked()
                    && toggle_dock_icon(&mut self.ui.view.bottom_dock_open, is_log_active)
                {
                    self.ui.view.bottom_tab = BottomTab::Log;
                }
                let is_model_active =
                    self.ui.view.bottom_dock_open && self.ui.view.bottom_tab == BottomTab::Model;
                if ui
                    .selectable_label(is_model_active, "📋")
                    .on_hover_text("モデル表")
                    .clicked()
                    && toggle_dock_icon(&mut self.ui.view.bottom_dock_open, is_model_active)
                {
                    self.ui.view.bottom_tab = BottomTab::Model;
                }
                let is_loads_active =
                    self.ui.view.bottom_dock_open && self.ui.view.bottom_tab == BottomTab::Loads;
                if ui
                    .selectable_label(is_loads_active, "⚡")
                    .on_hover_text("荷重表")
                    .clicked()
                    && toggle_dock_icon(&mut self.ui.view.bottom_dock_open, is_loads_active)
                {
                    self.ui.view.bottom_tab = BottomTab::Loads;
                }
                let is_prep_active = self.ui.view.bottom_dock_open
                    && self.ui.view.bottom_tab == BottomTab::Preparation;
                if ui
                    .selectable_label(is_prep_active, "🛠")
                    .on_hover_text("準備計算の結果")
                    .clicked()
                    && toggle_dock_icon(&mut self.ui.view.bottom_dock_open, is_prep_active)
                {
                    self.ui.view.bottom_tab = BottomTab::Preparation;
                }
                let is_diag_active = self.ui.view.bottom_dock_open
                    && self.ui.view.bottom_tab == BottomTab::Diagnostics;
                if ui
                    .selectable_label(is_diag_active, "⚠")
                    .on_hover_text("診断")
                    .clicked()
                    && toggle_dock_icon(&mut self.ui.view.bottom_dock_open, is_diag_active)
                {
                    self.ui.view.bottom_tab = BottomTab::Diagnostics;
                }
                ui.separator();
                // プロジェクトファイル名 + 未保存マーカー
                let file_label = self
                    .core
                    .scoped
                    .project_path
                    .as_ref()
                    .and_then(|p| p.file_name())
                    .map(|n| n.to_string_lossy().to_string())
                    .unwrap_or_else(|| "(未保存プロジェクト)".to_string());
                let marker = if self.core.scoped.staleness.unsaved_changes {
                    " ●"
                } else {
                    ""
                };
                ui.label(format!("{}{}", file_label, marker));
                ui.separator();
                // バックグラウンド解析ジョブの実行状況
                if let Some(job) = &self.core.scoped.job {
                    let elapsed = job.started.elapsed().unwrap_or_default().as_secs_f64();
                    ui.colored_label(
                        crate::theme::WHITE,
                        format!("⏳ {} 実行中… {:.0}s", job.label, elapsed),
                    );
                    ui.separator();
                }
                // stale アイコン。意味色は下ドック／ログ側。青地のバー上は白。
                if self.core.scoped.staleness.results_stale {
                    ui.colored_label(crate::theme::WHITE, "⚠ stale");
                } else if self.core.scoped.results.is_some() {
                    ui.colored_label(crate::theme::WHITE, "✓ 最新");
                } else {
                    ui.colored_label(crate::theme::WHITE, "▷ 未実行");
                }
                if let Some(err) = &self.core.scoped.last_error {
                    ui.separator();
                    // ST-Bridge 取込警告（複数件を \n 区切りで連結）など改行を含む
                    // メッセージは1行に畳んでから truncate する（\n はレイアウト上
                    // 明示的な改行として扱われ、行の高さ・幅の見積りが崩れるため）。
                    // 全文はホバーで表示する。クリックでログタブを前面にする
                    // （エラーの詳細な経緯はログに残っているため）。
                    let one_line = err.replace('\n', " ");
                    let clicked = ui
                        .add(
                            egui::Label::new(
                                egui::RichText::new(format!("⚠ {}", one_line))
                                    .color(crate::theme::WHITE),
                            )
                            .truncate()
                            .sense(egui::Sense::click()),
                        )
                        .on_hover_text(format!("{}\n\nクリックでログを開く", err))
                        .clicked();
                    if clicked {
                        self.open_log_dock();
                    }
                }
                // last_error（処理を止める）とは別枠の注意事項（例: 精算周期
                // (SemiPrecise)選択時に固有値解析が未実行で EX/EY が未更新である旨）。
                // バー上は白。意味色（黄）はログ側。解析自体は継続してよい。
                if let Some(notice) = &self.core.scoped.last_notice {
                    ui.separator();
                    let one_line = notice.replace('\n', " ");
                    ui.add(
                        egui::Label::new(
                            egui::RichText::new(format!("ℹ {}", one_line))
                                .color(crate::theme::WHITE),
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
                ui.label(summary);
            });
        });
    }
}
