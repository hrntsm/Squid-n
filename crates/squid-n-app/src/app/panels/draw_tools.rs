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
            ui.toggle_value(&mut self.ui.scoped.wall_draw_mode, "壁版割当モード");
            if self.ui.scoped.wall_draw_mode && !wall_was_on {
                self.ui.scoped.beam_draw_mode = false;
                self.ui.scoped.slab_draw_mode = false;
            }
            if self.ui.scoped.wall_draw_mode {
                ui.label(
                    "3D で壁版割当領域をクリック（未設定・版なしは割当画面、版ありは編集画面を開く）",
                );
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
                .on_hover_text("未設定・版なしの領域へ割り当てる壁版の断面（板厚・自重）です");
            }
        });

        ui.horizontal(|ui| {
            let slab_was_on = self.ui.scoped.slab_draw_mode;
            ui.toggle_value(&mut self.ui.scoped.slab_draw_mode, "床板割当モード");
            if self.ui.scoped.slab_draw_mode && !slab_was_on {
                self.ui.scoped.beam_draw_mode = false;
                self.ui.scoped.wall_draw_mode = false;
                self.ui.scoped.joist_place_mode = false;
                self.ui.scoped.post_place_mode = false;
                self.ui.scoped.work_scope = None;
                self.ui.scoped.member_place_first = None;
            }
            if self.ui.scoped.slab_draw_mode {
                ui.label(
                    "3D で床板割当領域をクリック（未設定・版なしは割当画面、版ありは編集画面を開く）",
                );
            }
        });

        ui.horizontal(|ui| {
            let joist_was_on = self.ui.scoped.joist_place_mode;
            ui.toggle_value(&mut self.ui.scoped.joist_place_mode, "小梁配置モード");
            if self.ui.scoped.joist_place_mode && !joist_was_on {
                self.ui.scoped.beam_draw_mode = false;
                self.ui.scoped.wall_draw_mode = false;
                self.ui.scoped.slab_draw_mode = false;
                self.ui.scoped.post_place_mode = false;
                self.ui.scoped.work_scope = None;
                self.ui.scoped.member_place_first = None;
            }
            if self.ui.scoped.joist_place_mode {
                match self.ui.scoped.work_scope {
                    None => {
                        ui.label("3D で床領域（作業範囲）をクリック");
                    }
                    Some(_) => {
                        ui.label("大梁・小梁を 2 点クリック（端 A → 端 B）");
                        if ui.button("範囲を選び直す").clicked() {
                            self.ui.scoped.work_scope = None;
                            self.ui.scoped.member_place_first = None;
                        }
                    }
                }
            }
        });

        ui.horizontal(|ui| {
            let post_was_on = self.ui.scoped.post_place_mode;
            ui.toggle_value(&mut self.ui.scoped.post_place_mode, "間柱配置モード");
            if self.ui.scoped.post_place_mode && !post_was_on {
                self.ui.scoped.beam_draw_mode = false;
                self.ui.scoped.wall_draw_mode = false;
                self.ui.scoped.slab_draw_mode = false;
                self.ui.scoped.joist_place_mode = false;
                self.ui.scoped.work_scope = None;
                self.ui.scoped.member_place_first = None;
            }
            if self.ui.scoped.post_place_mode {
                match self.ui.scoped.work_scope {
                    None => {
                        ui.label("3D で壁領域（作業範囲）をクリック（構面で選ぶ）");
                    }
                    Some(_) => {
                        ui.label("柱・梁・間柱を 2 点クリック（端 A → 端 B）");
                        if ui.button("範囲を選び直す").clicked() {
                            self.ui.scoped.work_scope = None;
                            self.ui.scoped.member_place_first = None;
                        }
                    }
                }
            }
        });

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

    /// 3D でクリックした割当領域の割当・編集ウィンドウ。
    ///
    /// 未設定・版なしの領域はその場で版の仕様を入力して割り当てる。版ありの領域は
    /// 既存版の一覧（編集画面）へ移動する。版の二重作成はしない。
    pub(crate) fn region_assign_window(&mut self, ctx: &egui::Context) {
        let Some(target) = self.ui.scoped.region_assign_dialog else {
            return;
        };
        let mut close = false;
        let mut open = true;
        egui::Window::new("割当領域")
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .default_width(380.0)
            .show(ctx, |ui| {
                close = match target {
                    crate::app::RegionAssignTarget::Floor(id) => {
                        self.floor_region_assign_ui(ui, id)
                    }
                    crate::app::RegionAssignTarget::Wall(id) => self.wall_region_assign_ui(ui, id),
                };
            });
        if close || !open {
            self.ui.scoped.region_assign_dialog = None;
        }
    }

    /// 床板割当領域の割当 UI。閉じるなら `true`。
    fn floor_region_assign_ui(
        &mut self,
        ui: &mut egui::Ui,
        region_id: squid_n_core::ids::FloorPlateAssignmentRegionId,
    ) -> bool {
        use squid_n_core::model::{AreaLoad, DistributionMethod, PlateAssignment};

        let Some(region) = self.core.model.floor_assignment_region(region_id) else {
            ui.label("この床板割当領域は存在しません（モデルが変更されました）");
            return true;
        };
        let state = region.assignment;
        let boundary_edges = region.boundary.len();
        let resolvable = self
            .core
            .model
            .floor_assignment_region_nodes(region_id)
            .is_some();

        ui.strong(format!("床板割当領域 R{}", region_id.0));
        ui.label(format!("境界: {} 辺", boundary_edges));

        if let PlateAssignment::Plate(sid) = state {
            ui.label(format!("床板 #{} が割り当て済みです。", sid.0));
            ui.label("仕様の編集は「スラブ」タブの床板一覧で行います。");
            if ui.button("床板一覧を開く").clicked() {
                self.ui.view.active_tab = crate::app::Tab::Model;
                self.ui.view.model_tab = crate::app::ModelTab::Slabs;
                return true;
            }
            return ui.button("閉じる").clicked();
        }

        if !resolvable {
            ui.colored_label(
                crate::theme::WARN_TEXT,
                "この領域は境界の頂点に対応する節点がなく、床板を割り当てられません。",
            );
            return ui.button("閉じる").clicked();
        }

        ui.label("この領域へ割り当てる床板の仕様を入力してください。");
        ui.horizontal(|ui| {
            ui.label("断面:");
            let resolved = self
                .ui
                .scoped
                .slab_draft
                .section
                .and_then(|sid| self.core.model.sections.get(sid.index()))
                .filter(|sec| sec.thickness.is_some_and(|t| t > 0.0));
            if resolved.is_none() {
                self.ui.scoped.slab_draft.section = None;
            }
            let label = resolved
                .map(|sec| sec.display_name())
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt("region_assign_slab_section")
                .selected_text(label)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.ui.scoped.slab_draft.section, None, "―");
                    for sec in &self.core.model.sections {
                        if sec.thickness.is_some_and(|t| t > 0.0) {
                            ui.selectable_value(
                                &mut self.ui.scoped.slab_draft.section,
                                Some(sec.id),
                                sec.display_name(),
                            );
                        }
                    }
                });
        });
        ui.horizontal(|ui| {
            ui.label("荷重種別:");
            ui.add(
                egui::TextEdit::singleline(&mut self.ui.scoped.slab_draft.load_kind)
                    .desired_width(60.0),
            );
            ui.label("[kN/m²]:");
            ui.add(
                egui::TextEdit::singleline(&mut self.ui.scoped.slab_draft.load_value)
                    .desired_width(80.0),
            );
        });
        ui.horizontal(|ui| {
            ui.label("分配法:");
            ui.selectable_value(
                &mut self.ui.scoped.slab_draft.method,
                DistributionMethod::TriTrapezoid,
                "三角/台形",
            );
            ui.selectable_value(
                &mut self.ui.scoped.slab_draft.method,
                DistributionMethod::OneWay,
                "一方向",
            );
            ui.selectable_value(
                &mut self.ui.scoped.slab_draft.method,
                DistributionMethod::TributaryArea,
                "負担面積",
            );
        });

        let section = self.ui.scoped.slab_draft.section;
        let value_kn_m2 = self
            .ui
            .scoped
            .slab_draft
            .load_value
            .trim()
            .parse::<f64>()
            .unwrap_or(0.0);
        let value = squid_n_core::units::to_internal::area_load_kn_per_m2(value_kn_m2);
        let kind = self.ui.scoped.slab_draft.load_kind.trim();
        let kind = if kind.is_empty() { "DL" } else { kind }.to_string();
        let plate = squid_n_core::model::SlabPlate {
            section,
            loads: vec![AreaLoad { kind, value }],
            usage: self.ui.scoped.slab_draft.usage,
            method: self.ui.scoped.slab_draft.method,
            one_way: None,
        };

        let mut close = false;
        ui.horizontal(|ui| {
            let can_assign = section.is_some();
            if ui
                .add_enabled(can_assign, egui::Button::new("この仕様で割当"))
                .on_hover_text(
                    "断面を選ぶと割り当てられます（未割当の床は解析前チェックで止まります）",
                )
                .clicked()
            {
                self.core.scoped.undo.run(
                    &mut self.core.model,
                    Box::new(squid_n_edit::AssignSlabToFloorPlateRegion {
                        region: region_id,
                        plate: plate.clone(),
                    }),
                );
                self.core.scoped.staleness.mark_edited();
                close = true;
            }
            if !state.is_no_plate() && ui.button("版なしにする").clicked() {
                self.core.scoped.undo.run(
                    &mut self.core.model,
                    Box::new(squid_n_edit::SetFloorPlateRegionNoPlate { region: region_id }),
                );
                self.core.scoped.staleness.mark_edited();
                close = true;
            }
            if !state.is_unset() && ui.button("未設定へ戻す").clicked() {
                self.core.scoped.undo.run(
                    &mut self.core.model,
                    Box::new(squid_n_edit::UnsetFloorPlateRegion { region: region_id }),
                );
                self.core.scoped.staleness.mark_edited();
                close = true;
            }
        });
        if ui.button("閉じる").clicked() {
            close = true;
        }
        close
    }

    /// 壁版割当領域の割当 UI。閉じるなら `true`。
    fn wall_region_assign_ui(
        &mut self,
        ui: &mut egui::Ui,
        region_id: squid_n_core::ids::WallPlateAssignmentRegionId,
    ) -> bool {
        use squid_n_core::model::PlateAssignment;

        let Some(region) = self.core.model.wall_assignment_region(region_id) else {
            ui.label("この壁版割当領域は存在しません（モデルが変更されました）");
            return true;
        };
        let state = region.assignment;
        let boundary_edges = region.boundary.len();
        let resolvable = self
            .core
            .model
            .wall_assignment_region_nodes(region_id)
            .is_some();

        ui.strong(format!("壁版割当領域 R{}", region_id.0));
        ui.label(format!("境界: {} 辺", boundary_edges));

        if let PlateAssignment::Plate(pid) = state {
            ui.label(format!("壁版 #{} が割り当て済みです。", pid.0));
            ui.label("仕様の編集は「壁版」タブの壁版一覧で行います。");
            if ui.button("壁版一覧を開く").clicked() {
                self.ui.view.active_tab = crate::app::Tab::Model;
                self.ui.view.model_tab = crate::app::ModelTab::WallPlates;
                return true;
            }
            return ui.button("閉じる").clicked();
        }

        if !resolvable {
            ui.colored_label(
                crate::theme::WARN_TEXT,
                "この領域は境界の頂点に対応する節点がなく、壁版を割り当てられません。",
            );
            return ui.button("閉じる").clicked();
        }

        ui.label("この領域へ割り当てる壁版の断面を選んでください。");
        ui.horizontal(|ui| {
            ui.label("断面:");
            let resolved = self
                .ui
                .scoped
                .wall_plate_draft
                .add_enclosed_section
                .and_then(|sid| self.core.model.sections.get(sid.index()))
                .filter(|sec| sec.thickness.is_some_and(|t| t > 0.0));
            if resolved.is_none() {
                self.ui.scoped.wall_plate_draft.add_enclosed_section = None;
            }
            let label = resolved
                .map(|sec| sec.display_name())
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt("region_assign_wall_section")
                .selected_text(label)
                .show_ui(ui, |ui| {
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
                });
        });

        let section = self.ui.scoped.wall_plate_draft.add_enclosed_section;
        let mut close = false;
        ui.horizontal(|ui| {
            let can_assign = section.is_some();
            if ui
                .add_enabled(can_assign, egui::Button::new("この断面で割当"))
                .on_hover_text("断面を選ぶと割り当てられます")
                .clicked()
            {
                self.core.scoped.undo.run(
                    &mut self.core.model,
                    Box::new(squid_n_edit::AssignWallPlateToRegion {
                        region: region_id,
                        section,
                        opening_area: 0.0,
                        opening_weight: 0.0,
                    }),
                );
                self.core.scoped.staleness.mark_edited();
                close = true;
            }
            if !state.is_no_plate() && ui.button("版なしにする").clicked() {
                self.core.scoped.undo.run(
                    &mut self.core.model,
                    Box::new(squid_n_edit::SetWallPlateRegionNoPlate { region: region_id }),
                );
                self.core.scoped.staleness.mark_edited();
                close = true;
            }
            if !state.is_unset() && ui.button("未設定へ戻す").clicked() {
                self.core.scoped.undo.run(
                    &mut self.core.model,
                    Box::new(squid_n_edit::UnsetWallPlateRegion { region: region_id }),
                );
                self.core.scoped.staleness.mark_edited();
                close = true;
            }
        });
        if ui.button("閉じる").clicked() {
            close = true;
        }
        close
    }
}
