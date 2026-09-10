//! 左ドック 作成パレット。
//!
//! `panels` からの構造分割。アルゴリズム変更は行わない。

use super::*;

impl App {
    /// 左ドック「作成」パネル：梁・壁・スラブ作成モードの切替と断面割当 UI。
    /// いずれもビューア（3D クリック）と連動する状態（`beam_draw_mode` 等）を操作する。
    pub(crate) fn draw_tools_panel(&mut self, ui: &mut egui::Ui) {
        ui.strong("作成");
        ui.separator();

        if self.load_pick_active() {
            ui.colored_label(
                crate::theme::WARN_TEXT,
                "荷重の対象を選択中は作成モードを使えません。\
                 3D ビューで対象を選ぶか、Esc で選択を取り消してください。",
            );
            return;
        }

        ui.horizontal(|ui| {
            let beam_was_on = self.ui.scoped.beam_draw_mode;
            ui.toggle_value(&mut self.ui.scoped.beam_draw_mode, "梁作成モード");
            if self.ui.scoped.beam_draw_mode && !beam_was_on {
                self.ui.scoped.wall_draw_mode = false;
                self.ui.scoped.slab_draw_mode = false;
            }
            if self.ui.scoped.beam_draw_mode {
                match self.ui.scoped.beam_draw_first {
                    None => {
                        ui.label("始点をクリック");
                    }
                    Some(first) => {
                        ui.label(format!("始点 {} 選択中 → 終点をクリック", first.label()));
                        if ui.button("キャンセル").clicked() {
                            self.ui.scoped.beam_draw_first = None;
                        }
                    }
                }
            }
        });
        if !self.ui.scoped.beam_draw_mode {
            self.ui.scoped.beam_draw_first = None;
        }

        ui.horizontal(|ui| {
            let wall_was_on = self.ui.scoped.wall_draw_mode;
            ui.toggle_value(&mut self.ui.scoped.wall_draw_mode, "壁作成モード");
            if self.ui.scoped.wall_draw_mode && !wall_was_on {
                self.ui.scoped.beam_draw_mode = false;
                self.ui.scoped.slab_draw_mode = false;
            }
            if self.ui.scoped.wall_draw_mode {
                let node_count = self.core.model.nodes.len() as u32;
                self.ui.scoped.wall_draw_nodes.retain(|n| n.0 < node_count);
                let picked: Vec<String> = self
                    .ui
                    .scoped
                    .wall_draw_nodes
                    .iter()
                    .map(|n| format!("N{}", n.0))
                    .collect();
                ui.label(format!(
                    "節点を4つクリック ({}/4){}",
                    self.ui.scoped.wall_draw_nodes.len(),
                    if picked.is_empty() {
                        String::new()
                    } else {
                        format!(": {}", picked.join(", "))
                    }
                ));
                ui.horizontal(|ui| {
                    ui.label("断面:");
                    ui.selectable_value(
                        &mut self.ui.scoped.wall_plate_draft.add_enclosed_section,
                        None,
                        "―",
                    );
                    for sec in &self.core.model.sections {
                        if sec.thickness.is_some_and(|t| t > 0.0) {
                            ui.selectable_value(
                                &mut self.ui.scoped.wall_plate_draft.add_enclosed_section,
                                Some(sec.id),
                                sec.display_name(),
                            );
                        }
                    }
                })
                .response
                .on_hover_text("壁の板厚と自重は断面から決まります");
                if !self.ui.scoped.wall_draw_nodes.is_empty() && ui.button("キャンセル").clicked()
                {
                    self.ui.scoped.wall_draw_nodes.clear();
                }
            }
        });
        if !self.ui.scoped.wall_draw_mode {
            self.ui.scoped.wall_draw_nodes.clear();
        }

        ui.horizontal(|ui| {
            let slab_was_on = self.ui.scoped.slab_draw_mode;
            ui.toggle_value(&mut self.ui.scoped.slab_draw_mode, "スラブ作成モード");
            if self.ui.scoped.slab_draw_mode && !slab_was_on {
                self.ui.scoped.beam_draw_mode = false;
                self.ui.scoped.wall_draw_mode = false;
            }
            if self.ui.scoped.slab_draw_mode {
                let node_count = self.core.model.nodes.len() as u32;
                self.ui.scoped.slab_draw_nodes.retain(|n| n.0 < node_count);
                let picked: Vec<String> = self
                    .ui
                    .scoped
                    .slab_draw_nodes
                    .iter()
                    .map(|n| format!("N{}", n.0))
                    .collect();
                ui.label(format!(
                    "境界節点を外周順にクリック ({}){}",
                    self.ui.scoped.slab_draw_nodes.len(),
                    if picked.is_empty() {
                        String::new()
                    } else {
                        format!(": {}", picked.join(", "))
                    }
                ));
                if self.ui.scoped.slab_draw_nodes.len() >= 3 && ui.button("確定").clicked() {
                    let boundary = self.ui.scoped.slab_draw_nodes.clone();
                    let draft_section = self.ui.scoped.slab_draft.section.filter(|sid| {
                        self.core
                            .model
                            .sections
                            .get(sid.index())
                            .is_some_and(|s| s.thickness.is_some_and(|t| t > 0.0))
                    });
                    self.core.scoped.undo.run(
                        &mut self.core.model,
                        Box::new(squid_n_edit::AddSlab {
                            boundary,
                            loads: Vec::new(),
                            method: squid_n_core::model::DistributionMethod::TriTrapezoid,
                            usage: self.ui.scoped.slab_draft.usage,
                            section: draft_section,
                        }),
                    );
                    self.core.scoped.staleness.mark_edited();
                    self.ui.scoped.slab_draw_nodes.clear();
                }
                if !self.ui.scoped.slab_draw_nodes.is_empty() && ui.button("キャンセル").clicked()
                {
                    self.ui.scoped.slab_draw_nodes.clear();
                }
            }
        });
        if !self.ui.scoped.slab_draw_mode {
            self.ui.scoped.slab_draw_nodes.clear();
        }

        let focus_id: Option<squid_n_core::ids::ElemId> = self.ui.scoped.nav.focus_member;
        let elem_info: Option<(squid_n_core::ids::ElemId, Option<SectionId>)> =
            focus_id.and_then(|eid| self.core.model.element(eid).map(|e| (e.id, e.section)));

        let mut pending_assign: Option<Option<SectionId>> = None;

        if let Some((elem_id, current_section)) = elem_info {
            ui.horizontal(|ui| {
                ui.label(format!("選択中の梁 #{}", elem_id.0));
                ui.label("断面:");
                let selected_text = current_section
                    .map(|sid| format!("S{}", sid.0))
                    .unwrap_or_else(|| "―".to_string());
                egui::ComboBox::from_id_salt("viewer_assign_section")
                    .selected_text(selected_text)
                    .show_ui(ui, |ui| {
                        if ui
                            .selectable_label(current_section.is_none(), "―")
                            .clicked()
                        {
                            pending_assign = Some(None);
                        }
                        for sec in &self.core.model.sections {
                            if ui
                                .selectable_label(
                                    current_section == Some(sec.id),
                                    format!("S{}", sec.id.0),
                                )
                                .clicked()
                            {
                                pending_assign = Some(Some(sec.id));
                            }
                        }
                    });
            });
            if let Some(section) = pending_assign {
                self.core.scoped.undo.run(
                    &mut self.core.model,
                    Box::new(squid_n_edit::SetElementSection {
                        elem: elem_id,
                        section,
                    }),
                );
                self.core.scoped.staleness.mark_edited();
            }
        } else {
            ui.label("ビューアで梁をクリックすると選択できます");
        }
    }
}
