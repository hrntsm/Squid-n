//! 左ドック ナビゲータ。
//!
//! `panels` からの構造分割。アルゴリズム変更は行わない。

use super::*;
use squid_n_core::units::to_display::force_kn;

impl App {
    /// 左ペイン：ナビゲータ（階/部材群/荷重ケース/結果ケースのツリー）。
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

            // 結果ケース：静的解析結果／荷重組合せ結果をクリックで表示対象に選択できる。
            // 選択は変位図だけでなく応力図・断面検定（長期/短期）まで切り替える
            // （`select_displayed_result`）。クロージャ内では self を可変借用できないため、
            // クリックされたキーを一旦退避し、クロージャの外で適用する。
            let mut nav_selected: Option<StaticKey> = None;
            let header = egui::CollapsingHeader::new("結果ケース")
                .default_open(true)
                .id_salt("nav_result_cases");
            header.show(ui, |ui| {
                if let Some(r) = &self.results {
                    if r.statics.is_empty() && r.combos.is_empty() && r.modal.is_none() {
                        ui.label("（未実行）");
                    } else {
                        for (key, _) in r.statics.iter() {
                            let label = match key {
                                StaticCaseKey::User(id) => {
                                    let lc_name = self
                                        .model
                                        .load_cases
                                        .iter()
                                        .find(|lc| lc.id == *id)
                                        .map(|lc| lc.name.as_str())
                                        .unwrap_or("");
                                    format!("静的 LC {} {}", id.0, lc_name)
                                }
                                StaticCaseKey::Seismic(SeismicDir::X) => {
                                    "地震静的 (X方向)".to_string()
                                }
                                StaticCaseKey::Seismic(SeismicDir::Y) => {
                                    "地震静的 (Y方向)".to_string()
                                }
                            };
                            let is_sel = self.nav.focus_result == Some(StaticKey::Case(*key));
                            if ui.selectable_label(is_sel, label).clicked() {
                                nav_selected = Some(StaticKey::Case(*key));
                            }
                        }
                        for (i, (name, _)) in r.combos.iter().enumerate() {
                            let is_sel = self.nav.focus_result == Some(StaticKey::Combo(i));
                            if ui
                                .selectable_label(is_sel, format!("組合せ {}", name))
                                .clicked()
                            {
                                nav_selected = Some(StaticKey::Combo(i));
                            }
                        }
                        if r.modal.is_some() {
                            ui.label("固有値");
                        }
                    }
                } else {
                    ui.label("（未実行）");
                }
            });
            if let Some(key) = nav_selected {
                self.select_displayed_result(key);
            }

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
