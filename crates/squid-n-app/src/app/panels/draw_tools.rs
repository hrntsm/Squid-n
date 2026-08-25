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

        // 荷重の対象ピック中は 3D のクリックをそちらが受け取るため、作成モードを
        // ON にできると「切り替えたのに反応しない」状態になる。パネルごと無効にする。
        if self.load_pick_active() {
            ui.colored_label(
                crate::theme::WARN_TEXT,
                "荷重の対象を選択中は作成モードを使えません。\
                 3D ビューで対象を選ぶか、Esc で選択を取り消してください。",
            );
            return;
        }

        // --- 梁作成モード ---
        // ON 中はクリックで節点を選び、2 点目で梁を生成する（OFF 中は部材クリック=断面割当）。
        ui.horizontal(|ui| {
            let beam_was_on = self.beam_draw_mode;
            ui.toggle_value(&mut self.beam_draw_mode, "梁作成モード");
            // 梁作成を ON にしたら壁・スラブ作成は OFF（排他）
            if self.beam_draw_mode && !beam_was_on {
                self.wall_draw_mode = false;
                self.slab_draw_mode = false;
            }
            if self.beam_draw_mode {
                match self.beam_draw_first {
                    None => {
                        ui.label("始点をクリック");
                    }
                    Some(first) => {
                        ui.label(format!("始点 {} 選択中 → 終点をクリック", first.label()));
                        if ui.button("キャンセル").clicked() {
                            self.beam_draw_first = None;
                        }
                    }
                }
            }
        });
        // モード OFF 時は始点選択をクリア
        if !self.beam_draw_mode {
            self.beam_draw_first = None;
        }

        // --- 壁作成モード ---
        // ON 中はクリックで柱・梁に囲まれた 4 節点を順に選び、4 点目で壁を生成する。
        ui.horizontal(|ui| {
            let wall_was_on = self.wall_draw_mode;
            ui.toggle_value(&mut self.wall_draw_mode, "壁作成モード");
            // 壁作成を ON にしたら梁・スラブ作成は OFF（排他）
            if self.wall_draw_mode && !wall_was_on {
                self.beam_draw_mode = false;
                self.slab_draw_mode = false;
            }
            if self.wall_draw_mode {
                let picked: Vec<String> = self
                    .wall_draw_nodes
                    .iter()
                    .map(|n| format!("N{}", n.0))
                    .collect();
                ui.label(format!(
                    "節点を4つクリック ({}/4){}",
                    self.wall_draw_nodes.len(),
                    if picked.is_empty() {
                        String::new()
                    } else {
                        format!(": {}", picked.join(", "))
                    }
                ));
                if !self.wall_draw_nodes.is_empty() && ui.button("キャンセル").clicked() {
                    self.wall_draw_nodes.clear();
                }
            }
        });
        // モード OFF 時は選択をクリア
        if !self.wall_draw_mode {
            self.wall_draw_nodes.clear();
        }

        // --- スラブ作成モード ---
        // ON 中はクリックで境界節点を外周順に選び、3〜N 節点そろったら「確定」で生成する。
        ui.horizontal(|ui| {
            let slab_was_on = self.slab_draw_mode;
            ui.toggle_value(&mut self.slab_draw_mode, "スラブ作成モード");
            // スラブ作成を ON にしたら梁・壁作成は OFF（排他）
            if self.slab_draw_mode && !slab_was_on {
                self.beam_draw_mode = false;
                self.wall_draw_mode = false;
            }
            if self.slab_draw_mode {
                // 節点削除などで陳腐化した参照（範囲外 id）を毎フレーム除去し、
                // 存在しない節点を境界に含むスラブの生成を防ぐ。
                let node_count = self.model.nodes.len() as u32;
                self.slab_draw_nodes.retain(|n| n.0 < node_count);
                let picked: Vec<String> = self
                    .slab_draw_nodes
                    .iter()
                    .map(|n| format!("N{}", n.0))
                    .collect();
                ui.label(format!(
                    "境界節点を外周順にクリック ({}){}",
                    self.slab_draw_nodes.len(),
                    if picked.is_empty() {
                        String::new()
                    } else {
                        format!(": {}", picked.join(", "))
                    }
                ));
                if self.slab_draw_nodes.len() >= 3 && ui.button("確定").clicked() {
                    let boundary = self.slab_draw_nodes.clone();
                    // 床タブの追加フォームと同じ下書きの断面を使う。消えた断面を
                    // 指したままだと `AddSlab` が参照検証で Noop になり無反応に
                    // 見えるため、解決できない下書きは未割当として渡す。
                    let draft_section = self.slab_draft.section.filter(|sid| {
                        self.model
                            .sections
                            .get(sid.index())
                            .is_some_and(|s| s.thickness.is_some_and(|t| t > 0.0))
                    });
                    self.undo.run(
                        &mut self.model,
                        Box::new(squid_n_edit::AddSlab {
                            boundary,
                            loads: Vec::new(),
                            method: squid_n_core::model::DistributionMethod::TriTrapezoid,
                            usage: self.slab_draft.usage,
                            section: draft_section,
                        }),
                    );
                    self.staleness.mark_edited();
                    self.slab_draw_nodes.clear();
                }
                if !self.slab_draw_nodes.is_empty() && ui.button("キャンセル").clicked() {
                    self.slab_draw_nodes.clear();
                }
            }
        });
        // モード OFF 時は選択をクリア
        if !self.slab_draw_mode {
            self.slab_draw_nodes.clear();
        }

        // --- 断面割当 UI ---
        // focus_member を先にコピーして、後段の可変借用と競合しないようにする
        let focus_id: Option<squid_n_core::ids::ElemId> = self.nav.focus_member;
        // 存在確認もここで行い、ローカルに有効性と現在断面を取得
        let elem_info: Option<(squid_n_core::ids::ElemId, Option<SectionId>)> =
            focus_id.and_then(|eid| {
                self.model
                    .elements
                    .iter()
                    .find(|e| e.id == eid)
                    .map(|e| (e.id, e.section))
            });

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
                        for sec in &self.model.sections {
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
            // クロージャ外で発行（借用ルール）
            if let Some(section) = pending_assign {
                self.undo.run(
                    &mut self.model,
                    Box::new(squid_n_edit::SetElementSection {
                        elem: elem_id,
                        section,
                    }),
                );
                self.staleness.mark_edited();
            }
        } else {
            ui.label("ビューアで梁をクリックすると選択できます");
        }
    }
}
