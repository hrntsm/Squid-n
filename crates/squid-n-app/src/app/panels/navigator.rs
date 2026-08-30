//! 左ドック ナビゲータ。
//!
//! `panels` からの構造分割。アルゴリズム変更は行わない。

use super::*;
use squid_n_core::units::to_display::force_kn;

impl App {
    /// 左ペイン：ナビゲータ（階/部材群/断面・材料/荷重ケース/結果ケースのツリー）。
    pub(crate) fn navigator_panel(&mut self, ui: &mut egui::Ui) {
        ui.group(|ui| {
            ui.strong("ナビゲータ");
            ui.separator();

            // 部材グループ（簡易: 材種ごと）
            let header = egui::CollapsingHeader::new("部材グループ")
                .default_open(true)
                .id_salt("nav_groups");
            header.show(ui, |ui| {
                // 鋼系（S・CFT）とそれ以外へ 1 回の走査で分ける
                // （振り分けの規約は `member_material_groups`）。
                let (steel_ids, rc_ids) = member_material_groups(&self.model);
                // selected 表示は簡易判定（先頭要素が当該グループに属するか）。
                let is_steel_sel = self
                    .selection
                    .members
                    .first()
                    .map(|id| steel_ids.contains(id))
                    .unwrap_or(false);
                if ui
                    .selectable_label(is_steel_sel, format!("鋼材部材 ({})", steel_ids.len()))
                    .on_hover_text("クリックで3Dビューにハイライト")
                    .clicked()
                {
                    self.selection.members = steel_ids.clone();
                }
                let is_rc_sel = self
                    .selection
                    .members
                    .first()
                    .map(|id| rc_ids.contains(id))
                    .unwrap_or(false);
                if ui
                    .selectable_label(is_rc_sel, format!("RC部材 ({})", rc_ids.len()))
                    .on_hover_text("クリックで3Dビューにハイライト")
                    .clicked()
                {
                    self.selection.members = rc_ids.clone();
                }
            });

            self.nav_load_cases(ui);

            self.nav_vibration_cases(ui);

            self.nav_regions(ui);

            // 部材リスト（クリックで focus_member を更新 → テーブル/インスペクタに連動）
            let header = egui::CollapsingHeader::new("部材一覧")
                .default_open(false)
                .id_salt("nav_members");
            header.show(ui, |ui| {
                use crate::table_util::{self, Col};
                let n = self.model.elements.len();
                table_util::standard_table(
                    ui,
                    "nav_members_tbl",
                    &[Col::id(), Col::label("種別")],
                    n,
                    |row| {
                        let idx = row.index();
                        let elem = self.model.elements[idx].clone();
                        let is_focus = self.nav.focus_member == Some(elem.id);
                        row.col(|ui| {
                            if table_util::id_cell(ui, is_focus, elem.id.0, "クリックで部材を選択")
                            {
                                self.nav.focus_member = Some(elem.id);
                            }
                        });
                        row.col(|ui| {
                            ui.label(format!("{:?}", elem.kind));
                        });
                    },
                );
            });

            self.nav_sections(ui);
            self.nav_materials(ui);

            self.nav_result_cases(ui);

            // 階/レベル（準備計算が生成した階を上階→下階順に表示）
            let _ = ui.collapsing("階/レベル", |ui| {
                if self.model.stories.is_empty() {
                    ui.colored_label(crate::theme::GRAY_600, "未定義");
                    if ui.small_button("🏢 解析タブで自動生成").clicked() {
                        self.active_tab = Tab::Analysis;
                    }
                } else {
                    for s in self.model.stories.iter().rev() {
                        ui.label(format!(
                            "{}  Z={:.0}mm  W={:.1}kN",
                            s.name,
                            s.elevation,
                            force_kn(s.seismic_weight.unwrap_or(0.0))
                        ));
                    }
                }
            });
        });
    }
}
