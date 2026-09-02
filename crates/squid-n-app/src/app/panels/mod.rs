//! `App` の egui パネル描画メソッド。
//!
//! サブモジュール: `navigator`（左ナビ）、`draw_tools`（作成パレット）、
//! `preparation`（①準備計算）、`analysis`（静的・固有値・増分・時刻歴の各解析実行）、
//! `activity_bar`（左右アイコン列）、`results`（結果タブ）、
//! `inspector`（インスペクタ）、`status_bar`（ステータスバー）。
//! 薄いタブスイッチャー・ファイルダイアログは本モジュール（ハブ）に残す。

mod activity_bar;
mod analysis;
mod draw_tools;
mod inspector;
mod navigator;
mod preparation;
mod results;
mod status_bar;

use super::*;

/// ドック/パネル切替アイコンの共通クリック挙動。
/// 対象ドックが開いていて対象パネルが既にアクティブなら閉じて `false` を返す。
/// それ以外はドックを開いて `true` を返す（呼び出し側は `true` のときのみ
/// 対象パネル/タブをアクティブにする）。
pub(crate) fn toggle_dock_icon(dock_open: &mut bool, is_active: bool) -> bool {
    if *dock_open && is_active {
        *dock_open = false;
        false
    } else {
        *dock_open = true;
        true
    }
}

/// 左右アクティビティバーの幅（スロット＋外側アクセント線）。
pub(crate) fn activity_bar_width() -> f32 {
    activity_bar::activity_bar_width()
}

/// 左右アクティビティバーのパネル枠（blue-200）。
pub(crate) fn activity_bar_frame() -> egui::Frame {
    activity_bar::activity_bar_frame()
}

impl App {
    /// 「開く…」ダイアログを表示して読み込む。
    pub(crate) fn open_project_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("Squid-n プロジェクト", &["scz"])
            .pick_file()
        {
            self.open_project_from(path);
        }
    }

    /// 保存する。`force_ask` またはパス未設定時はダイアログで保存先を尋ねる。
    pub(crate) fn save_project_dialog(&mut self, force_ask: bool) {
        let path = if force_ask {
            None
        } else {
            self.core.scoped.project_path.clone()
        };
        let path = path.or_else(|| {
            rfd::FileDialog::new()
                .add_filter("Squid-n プロジェクト", &["scz"])
                .set_file_name("model.scz")
                .save_file()
        });
        if let Some(path) = path {
            self.save_project_to(path);
        }
    }

    /// 「ST-Bridge 読込…」ダイアログを表示して読み込む。
    pub(crate) fn import_stbridge_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("ST-Bridge", &["stb", "xml"])
            .pick_file()
        {
            self.import_stbridge_from(path);
        }
    }

    /// 「ST-Bridge 書出…」ダイアログを表示して保存先を尋ね、標準 ST-Bridge で書き出す。
    pub(crate) fn export_stbridge_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("ST-Bridge", &["stb", "xml"])
            .set_file_name("model.stb")
            .save_file()
        {
            self.export_stbridge_to(path);
        }
    }
    /// モデルタブ：サブタブ切替で節点/部材/断面/材料を編集するテーブルを表示。
    pub(crate) fn model_tab_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            let subs = [
                ("節点", ModelTab::Nodes),
                ("境界条件", ModelTab::BoundaryConditions),
                ("部材", ModelTab::Members),
                ("断面", ModelTab::Sections),
                ("材料", ModelTab::Materials),
                ("スラブ", ModelTab::Slabs),
                ("壁版", ModelTab::WallPlates),
                ("部材付帯情報", ModelTab::MemberDetails),
                ("S造検定属性", ModelTab::SteelAttrs),
                ("通り芯", ModelTab::Axes),
            ];
            for (label, sub) in &subs {
                let sel = self.ui.view.model_tab == *sub;
                if ui.selectable_label(sel, *label).clicked() {
                    self.ui.view.model_tab = *sub;
                }
            }
        });
        ui.separator();
        match self.ui.view.model_tab {
            ModelTab::Nodes => crate::tables::nodes::nodes_table(ui, self),
            ModelTab::BoundaryConditions => {
                crate::tables::nodes::boundary_condition_panel(ui, self)
            }
            ModelTab::Members => crate::tables::members::members_table(ui, self),
            ModelTab::Sections => {
                crate::tables::sections::sections_table(ui, self);
                ui.add_space(8.0);
                crate::section_editor::catalog_section_panel(ui, self);
                ui.add_space(8.0);
                crate::section_editor::section_editor_panel(ui, self);
                ui.add_space(8.0);
                crate::damper_def_editor::damper_def_panel(ui, self);
            }
            ModelTab::Materials => crate::tables::materials::materials_table(ui, self),
            ModelTab::Slabs => crate::tables::slabs::slabs_table(ui, self),
            ModelTab::WallPlates => crate::tables::wall_plates::wall_plates_table(ui, self),
            ModelTab::MemberDetails => {
                crate::tables::member_details::member_details_table(ui, self)
            }
            ModelTab::SteelAttrs => crate::tables::steel_attrs::steel_attrs_table(ui, self),
            ModelTab::Axes => crate::tables::axes::axes_table(ui, self),
        }
    }
    /// 結果タブの「表示対象」ドロップダウン用の選択肢（キーと表示名）を収集する。
    /// 静的ケース（ユーザー荷重・地震静的）に続けて荷重組合せを並べる。
    /// ラベルはナビゲータの葉ノードと揃える（[`super::nav_results`]）。
    fn result_display_options(&self) -> Vec<(StaticKey, String)> {
        let mut opts = Vec::new();
        if let Some(r) = &self.core.scoped.results {
            for (key, _) in r.statics.iter() {
                let label = match key {
                    StaticCaseKey::User(id) => {
                        let nm = self
                            .core
                            .model
                            .load_cases
                            .iter()
                            .find(|lc| lc.id == *id)
                            .map(|lc| lc.name.as_str())
                            .unwrap_or("");
                        super::nav_results::user_static_label(nm)
                    }
                    StaticCaseKey::Seismic(dir) => {
                        super::nav_results::seismic_static_label(*dir).to_string()
                    }
                };
                opts.push((StaticKey::Case(*key), label));
            }
            for (i, (name, _)) in r.combos.iter().enumerate() {
                opts.push((StaticKey::Combo(i), name.clone()));
            }
        }
        opts
    }
    /// 設計タブ：検定表（許容応力度・保有水平耐力）と MN 相関曲面ビューを切り替える。
    pub(crate) fn design_tab_panel(&mut self, ui: &mut egui::Ui) {
        // 断面算定の対象荷重（ケース／組合せ）を選ぶドロップダウン用の選択肢。
        // 長期/短期区分は選んだ組合せ名から自動判定され（令82条の荷重組合せ:
        // G+P=長期、地震・積雪・風入り=短期）、対象荷重の右に読み取り専用で表示する。
        let result_options = self.result_display_options();
        let current_key = self
            .ui
            .scoped
            .nav
            .focus_result
            .or(self.core.scoped.last_static);
        let mut selected_result: Option<StaticKey> = None;
        ui.horizontal(|ui| {
            let sel_table = self.ui.view.design_view == DesignView::Table;
            let sel_ult = self.ui.view.design_view == DesignView::Ultimate;
            let sel_mn = self.ui.view.design_view == DesignView::MnSurface;
            let sel_qty = self.ui.view.design_view == DesignView::Quantities;
            if ui.selectable_label(sel_table, "検定表").clicked() {
                self.ui.view.design_view = DesignView::Table;
            }
            if ui.selectable_label(sel_ult, "終局検定").clicked() {
                self.ui.view.design_view = DesignView::Ultimate;
            }
            if ui.selectable_label(sel_mn, "MN相関曲面").clicked() {
                self.ui.view.design_view = DesignView::MnSurface;
            }
            if ui.selectable_label(sel_qty, "数量積算").clicked() {
                self.ui.view.design_view = DesignView::Quantities;
            }
            // 対象荷重の選択。選ぶとその組合せの内力・長期/短期で断面算定が再実行される。
            if !result_options.is_empty() {
                ui.separator();
                ui.label("対象荷重:");
                let cur_label = current_key
                    .and_then(|k| result_options.iter().find(|(o, _)| *o == k))
                    .map(|(_, l)| l.clone())
                    .unwrap_or_else(|| "（選択）".to_string());
                egui::ComboBox::from_id_salt("design_display_selector")
                    .selected_text(cur_label)
                    .show_ui(ui, |ui| {
                        for (opt_key, label) in &result_options {
                            if ui
                                .selectable_label(current_key == Some(*opt_key), label)
                                .clicked()
                            {
                                selected_result = Some(*opt_key);
                            }
                        }
                    });
                // 荷重継続性区分（許容応力度の長期/短期）。対象荷重から自動判定した
                // 結果の表示のみで、ここでの手動切替は行わない。
                let term_label = match self.core.design_term {
                    LoadTerm::Long => "長期",
                    LoadTerm::Short => "短期",
                };
                ui.label(format!("許容応力度: {term_label}")).on_hover_text(
                    "対象荷重（組合せ）の内容から自動判定します（令82条: G+P=長期、\
                         地震・積雪・風を含む組合せ=短期）。",
                );
            }
        });
        if let Some(key) = selected_result {
            self.select_displayed_result(key);
        }
        ui.separator();
        match self.ui.view.design_view {
            DesignView::Table => crate::design_view::design_table(ui, self),
            DesignView::Ultimate => crate::ultimate_view::ultimate_table(ui, self),
            DesignView::MnSurface => crate::mn_view::mn_surface_panel(ui, self),
            DesignView::Quantities => crate::quantity_view::quantity_panel(ui, self),
        }
    }
    /// レポートタブ：CSV レポートのプレビューとエクスポート。
    pub(crate) fn report_tab_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("レポート");
        if !crate::summary::has_report_content(&self.core.scoped.results) {
            ui.colored_label(
                crate::theme::GRAY_600,
                "解析結果がありません。解析タブから実行するとレポートを生成できます。",
            );
            return;
        }
        let csv = crate::summary::build_report_csv(self);
        ui.horizontal(|ui| {
            if ui.button("💾 CSV エクスポート…").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("CSV", &["csv"])
                    .set_file_name("report.csv")
                    .save_file()
                {
                    if let Err(e) = std::fs::write(&path, &csv) {
                        self.report_error(format!("レポート保存エラー: {}", e));
                    }
                }
            }
            if ui.button("📋 クリップボードへコピー").clicked() {
                ui.ctx().copy_text(csv.clone());
            }
        });
        ui.separator();
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.add(
                egui::TextEdit::multiline(&mut csv.as_str())
                    .font(egui::TextStyle::Monospace)
                    .desired_width(f32::INFINITY),
            );
        });
    }
}
