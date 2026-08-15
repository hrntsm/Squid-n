//! 右ドック インスペクタ。
//!
//! `panels` からの構造分割。アルゴリズム変更は行わない。

use super::*;
use crate::table_util::fmt_section_prop;
use squid_n_core::units::to_display::{area_cm2, inertia_cm4};

impl App {
    /// 右ペイン：選択要素のインスペクタ。
    /// 3D/ナビゲータ/テーブルの選択（現時点では focus_*）を表示。断面編集は UI-4 で拡充。
    pub(crate) fn inspector_panel(&mut self, ui: &mut egui::Ui) {
        // 遅延アクション（借用チェーン回避：UI 内で self.model を immutable borrow 中に
        // mut borrow できないため、複製ボタンクリックは一旦 here に保存）
        let mut duplicate_member = None;
        let mut highlight_section_members: Option<Vec<ElemId>> = None;
        ui.group(|ui| {
            ui.strong("インスペクタ");
            ui.separator();

            if let Some(id) = self.nav.focus_vibration_case {
                if let Some(case) = self.model.vibration_cases.iter().find(|c| c.id == id) {
                    ui.strong("立体振動ケース（選択中）");
                    ui.label(format!("名称: {}", case.name));
                    ui.label(format!("波形: {}", case.wave_name));
                    ui.label(format!(
                        "方向: {}",
                        match case.dir {
                            squid_n_core::model::VibrationThDir::X => "X",
                            squid_n_core::model::VibrationThDir::Y => "Y",
                            squid_n_core::model::VibrationThDir::Xy => "X+Y",
                        }
                    ));
                    ui.label(format!(
                        "解析: {}",
                        if case.nonlinear {
                            "非線形"
                        } else {
                            "線形"
                        }
                    ));
                    ui.separator();
                }
            } else if let Some(id) = self.nav.focus_lumped_vibration_case {
                if let Some(case) = self
                    .model
                    .lumped_vibration_cases
                    .iter()
                    .find(|c| c.id == id)
                {
                    ui.strong("質点系振動ケース（選択中）");
                    ui.label(format!("名称: {}", case.name));
                    ui.label(format!("波形: {}", case.wave_name));
                    ui.label(format!(
                        "方向: {}",
                        match case.dir {
                            squid_n_core::model::LumpedVibrationDir::X => "X",
                            squid_n_core::model::LumpedVibrationDir::Y => "Y",
                        }
                    ));
                    ui.label(format!(
                        "解析: {}・{}",
                        if case.nonlinear {
                            "非線形"
                        } else {
                            "線形"
                        },
                        match case.dim {
                            squid_n_core::model::LumpedVibrationDim::Planar => "2次元",
                            squid_n_core::model::LumpedVibrationDim::Spatial => "3次元",
                        }
                    ));
                    ui.separator();
                }
            }

            // 選択された部材の諸元
            if let Some(elem_id) = self.nav.focus_member {
                if let Some(e) = self.model.element(elem_id) {
                    ui.label(format!("部材 ID: {}", e.id.0));
                    let n0 = e.nodes.first().map(|n| n.0).unwrap_or(0);
                    let n1 = e.nodes.get(1).map(|n| n.0).unwrap_or(0);
                    ui.label(format!("節点 I/J: {} / {}", n0, n1));
                    if let Some(sec_id) = e.section {
                        if let Some(sec) = self
                            .model
                            .sections
                            .get(sec_id.index())
                            .filter(|s| s.id == sec_id)
                        {
                            ui.label(format!("断面: {} ({})", sec.name, sec_id.0));
                            ui.label(format!(
                                "  A = {} cm²",
                                fmt_section_prop(area_cm2(sec.area))
                            ));
                            ui.label(format!(
                                "  Iy= {} cm⁴",
                                fmt_section_prop(inertia_cm4(sec.iy))
                            ));
                            ui.label(format!(
                                "  Iz= {} cm⁴",
                                fmt_section_prop(inertia_cm4(sec.iz))
                            ));
                            // 影響数: 同一断面を使う部材数
                            let n_used = self
                                .model
                                .elements
                                .iter()
                                .filter(|o| o.section == Some(sec_id))
                                .count();
                            ui.colored_label(
                                crate::theme::BLUE_500,
                                format!("この断面を使う {} 部材に影響", n_used),
                            );
                            // UI-4: 複製ボタン（UI設計 §3）。同断面を新規IDで複製し、
                            // 当該部材のみ新断面に割当。
                            if ui.button("📋 複製してこの部材だけ別断面に").clicked()
                            {
                                duplicate_member = Some(elem_id);
                            }
                        }
                    } else {
                        ui.label("断面: 未割当");
                    }
                    // 材料は断面が持つ。ここでは断面から引いた実効値を表示する。
                    if let Some(mat) = self.model.element_material(e) {
                        ui.label(format!("材料: {} ({})", mat.name, mat.id.0));
                        ui.label(format!("  E = {:.1} N/mm²", mat.young));
                        if let Some(fc) = mat.fc {
                            ui.label(format!("  Fc = {:.1} N/mm²", fc));
                        }
                    }
                    ui.separator();
                    // 検定結果サマリ（同一部材）
                    if let Some(r) = &self.results {
                        let positions = r
                            .member_checks
                            .iter()
                            .find(|m| m.elem == elem_id)
                            .map(|m| m.positions.as_slice())
                            .unwrap_or(&[]);
                        ui.label(format!("検定結果（{} 位置）", positions.len()));
                        for p in positions.iter().take(8) {
                            match &p.outcome {
                                squid_n_design_jp::CheckOutcome::Checked(cr) => {
                                    let ratio = cr.ratio();
                                    let color = crate::theme::status_color(ratio);
                                    ui.colored_label(
                                        color,
                                        format!("  pos={:.2} 検定比={:.3}", p.xi, ratio),
                                    );
                                }
                                squid_n_design_jp::CheckOutcome::Skipped { reason } => {
                                    ui.colored_label(
                                        crate::theme::GRAY_600,
                                        format!("  pos={:.2} 検定不能（{reason}）", p.xi),
                                    );
                                }
                            }
                        }
                        if positions.len() > 8 {
                            ui.label(format!("  ... 他 {} 件", positions.len() - 8));
                        }
                    }
                } else {
                    ui.colored_label(crate::theme::GRAY_600, "部材を選択してください");
                }
            } else {
                ui.colored_label(
                    egui::Color32::from_rgb(150, 150, 150),
                    "部材を選択してください",
                );
            }

            // 選択された断面の諸元（断面テーブルの行選択と連動）
            if let Some(sec_id) = self.nav.focus_section {
                if let Some(sec) = self.model.sections.iter().find(|s| s.id == sec_id) {
                    ui.separator();
                    ui.strong("断面（選択中）");
                    ui.label(format!("名前: {} ({})", sec.name, sec_id.0));
                    ui.label(format!(
                        "  A = {} cm²",
                        fmt_section_prop(area_cm2(sec.area))
                    ));
                    ui.label(format!(
                        "  Iy= {} cm⁴",
                        fmt_section_prop(inertia_cm4(sec.iy))
                    ));
                    ui.label(format!(
                        "  Iz= {} cm⁴",
                        fmt_section_prop(inertia_cm4(sec.iz))
                    ));
                    let used: Vec<ElemId> = self
                        .model
                        .elements
                        .iter()
                        .filter(|e| e.section == Some(sec_id))
                        .map(|e| e.id)
                        .collect();
                    ui.label(format!("使用部材数: {}", used.len()));
                    if ui.button("🔍 使用部材を3Dハイライト").clicked() {
                        highlight_section_members = Some(used);
                    }
                }
            }

            ui.separator();
            // 選択された節点の諸元
            if let Some(node_id) = self.nav.focus_node {
                if let Some(node) = self.model.node(node_id) {
                    ui.label(format!("節点 ID: {}", node.id.0));
                    ui.label(format!(
                        "座標: ({:.3}, {:.3}, {:.3})",
                        node.coord[0], node.coord[1], node.coord[2]
                    ));
                    // 拘束情報
                    let is_fixed = node.restraint.0 != 0;
                    if is_fixed {
                        ui.label("拘束: あり");
                    } else {
                        ui.label("拘束: なし");
                    }
                }
            }
        });

        // 遅延実行: 複製ボタンが押されていたら EditCommand を叩く
        if let Some(member) = duplicate_member {
            self.undo.run(
                &mut self.model,
                Box::new(squid_n_edit::DuplicateSectionForMember { member }),
            );
            self.staleness.mark_edited();
        }
        // 遅延実行: 断面の使用部材ハイライトボタン
        if let Some(members) = highlight_section_members {
            self.selection.members = members;
        }
    }
}
