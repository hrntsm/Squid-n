//! 結果タブ画面（スイッチャー・増分結果・質点系）。
//!
//! `panels` からの構造分割。アルゴリズム変更は行わない。

use super::*;
use crate::table_util::Col;
use squid_n_core::units::to_display::{force_kn, stiffness_kn_per_mm};

impl App {
    /// 結果タブ：3Dビューア と 時刻歴グラフを切替。
    pub(crate) fn results_tab_panel(&mut self, ui: &mut egui::Ui) {
        // 表示対象（荷重ケース／組合せ）の選択肢を先に収集する
        // （クロージャ内で self を可変借用しないため）。current_key は現在の表示対象。
        let result_options = self.result_display_options();
        let current_key = self
            .ui
            .scoped
            .nav
            .focus_result
            .or(self.core.scoped.last_static);
        let mut selected_result: Option<StaticKey> = None;
        ui.horizontal(|ui| {
            let sel_spatial = self.ui.view.results_view == ResultsView::Spatial;
            let sel_th = self.ui.view.results_view == ResultsView::TimeHistory;
            let sel_po = self.ui.view.results_view == ResultsView::Pushover;
            let sel_lm = self.ui.view.results_view == ResultsView::LumpedMass;
            if ui.selectable_label(sel_spatial, "3D/応力図").clicked() {
                self.ui.view.results_view = ResultsView::Spatial;
            }
            if ui.selectable_label(sel_th, "時刻歴").clicked() {
                self.ui.view.results_view = ResultsView::TimeHistory;
            }
            if ui.selectable_label(sel_po, "増分解析").clicked() {
                self.ui.view.results_view = ResultsView::Pushover;
            }
            if ui.selectable_label(sel_lm, "質点系").clicked() {
                self.ui.view.results_view = ResultsView::LumpedMass;
            }
            ui.separator();
            // 結果サマリ
            if let Some(r) = &self.core.scoped.results {
                ui.label(format!("静的ケース数: {}", r.statics.len()));
                if let Some(m) = &r.modal {
                    let t1 = m.period.first().copied().unwrap_or(0.0);
                    ui.label(format!("固有周期 T1: {:.3} s", t1));
                }
                let n_checks: usize = r.member_checks.iter().map(|m| m.positions.len()).sum();
                ui.label(format!("検定結果数: {}", n_checks));
            } else {
                ui.colored_label(crate::theme::GRAY_600, "▷ 未実行");
            }
            // 表示対象（荷重ケース／組合せ）の選択。変位図に加え、応力図・断面検定
            // （その組合せの長期/短期）まで切り替える。
            if !result_options.is_empty() {
                ui.separator();
                ui.label("表示対象:");
                let cur_label = current_key
                    .and_then(|k| result_options.iter().find(|(o, _)| *o == k))
                    .map(|(_, l)| l.clone())
                    .unwrap_or_else(|| "（選択）".to_string());
                egui::ComboBox::from_id_salt("results_display_selector")
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
            }
        });
        if let Some(key) = selected_result {
            self.select_displayed_result(key);
        }
        ui.separator();
        match self.ui.view.results_view {
            ResultsView::Spatial => crate::viewer::viewer_panel(ui, self),
            ResultsView::TimeHistory => crate::time_history_view::time_history_panel(ui, self),
            ResultsView::Pushover => self.pushover_results_panel(ui),
            ResultsView::LumpedMass => self.lumped_mass_panel(ui),
        }
    }
    /// 増分解析結果（性能曲線・ヒンジ・崩壊機構）の表示。
    pub(crate) fn pushover_results_panel(&mut self, ui: &mut egui::Ui) {
        if self.displayed_pushover().is_none() {
            ui.colored_label(
                crate::theme::GRAY_600,
                "増分解析結果がありません。解析タブから実行してください。",
            );
            return;
        }

        // X/Y 方向切替（結果のない方向は disabled）。
        let has_x = self.pushover_for(SeismicDir::X).is_some();
        let has_y = self.pushover_for(SeismicDir::Y).is_some();
        ui.horizontal(|ui| {
            ui.label("表示方向:");
            if ui
                .add_enabled(
                    has_x,
                    egui::Button::selectable(
                        self.core.scoped.pushover_view_dir == SeismicDir::X,
                        "X",
                    ),
                )
                .clicked()
            {
                self.set_pushover_view_dir(SeismicDir::X);
            }
            if ui
                .add_enabled(
                    has_y,
                    egui::Button::selectable(
                        self.core.scoped.pushover_view_dir == SeismicDir::Y,
                        "Y",
                    ),
                )
                .clicked()
            {
                self.set_pushover_view_dir(SeismicDir::Y);
            }
        });
        ui.separator();

        // 必要保有水平耐力の総合判定（Qu ≥ Qun = Ds·Fes·Qud）を先に算定する。
        // 実行ボタン→結果画面でそのまま OK/NG を確認できるよう、性能曲線より前に
        // バナー表示する。`compute_holding_capacity` は &mut self を要するため、
        // 以降の `po` 借用より前にここで所有権付きの結果へ落とす。
        let hc_verdict = self.compute_holding_capacity().ok();

        let po = self.displayed_pushover().expect("checked above");

        // ── 必要保有水平耐力 判定バナー ──────────────────────────────
        match &hc_verdict {
            Some((res, _)) if !res.stories.is_empty() => {
                let ng = res.stories.iter().filter(|s| !s.ok).count();
                if ng == 0 {
                    ui.colored_label(
                        crate::theme::GOOD_GREEN,
                        format!(
                            "✔ 必要保有水平耐力を満足: 全 {} 層で Qu ≥ Qun（Qun = Ds·Fes·Qud）",
                            res.stories.len()
                        ),
                    );
                } else {
                    ui.colored_label(
                        crate::theme::ERROR_RED,
                        format!(
                            "✘ 必要保有水平耐力が不足: {} / {} 層で Qu < Qun。設計タブ「保有水平耐力」で詳細を確認してください。",
                            ng,
                            res.stories.len()
                        ),
                    );
                }
            }
            _ => {
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "必要保有水平耐力の判定には荷重ケース EX／EY（地震力）の実行が必要です（解析タブ）。",
                );
            }
        }
        // 崩壊機構が未形成（部分崩壊形）の警告。崩壊機構が確定しない限り Ds・
        // 目標未到達のまま打ち切られた解析（非収束・特異化）は Qu が過小評価の
        // 可能性があるため、終了理由を警告として明示する。
        if po.termination.is_premature() {
            ui.colored_label(
                crate::theme::SECONDARY_AMBER,
                format!(
                    "⚠ 増分解析は目標到達前に打ち切られました（{}）。性能曲線が途中で\
                     途切れており、Qu はその時点までの最大値です。",
                    po.termination.describe()
                ),
            );
        }
        // 必要保有水平耐力は暫定値であることを明示する（日本の慣行: 崩壊機構の確定が
        // 必要保有水平耐力算定の前提）。
        if matches!(
            po.mechanism,
            squid_n_solver::nonlinear::pushover::MechanismType::Partial
        ) {
            ui.colored_label(
                crate::theme::SECONDARY_AMBER,
                "⚠ 崩壊機構が未形成（部分崩壊形）です。目標変位を増やして再実行するか設計を\
                 見直してください。崩壊機構が確定するまで Ds・必要保有水平耐力は暫定値です。",
            );
        }
        ui.separator();

        ui.horizontal(|ui| {
            ui.label(format!("保有水平耐力 Qu = {:.1} kN", force_kn(po.qu)));
            ui.separator();
            let mech = match &po.mechanism {
                squid_n_solver::nonlinear::pushover::MechanismType::Overall => {
                    "全体崩壊形".to_string()
                }
                squid_n_solver::nonlinear::pushover::MechanismType::StoryCollapse { layer } => {
                    // 層の呼び名は下端の階名（法令の「i 階」）。
                    let name = self
                        .core
                        .model
                        .layers()
                        .get(*layer)
                        .map(|l| l.name.clone())
                        .unwrap_or_else(|| format!("{}", layer + 1));
                    format!("層崩壊形 ({name})")
                }
                squid_n_solver::nonlinear::pushover::MechanismType::Partial => {
                    "部分崩壊形".to_string()
                }
            };
            ui.label(format!("崩壊機構: {}", mech));
            ui.separator();
            ui.label(format!("ヒンジ発生 {} 件", po.hinges.len()));
            ui.separator();
            let control = match po.control {
                squid_n_solver::nonlinear::pushover::PushoverControl::Phased => "段階制御",
                squid_n_solver::nonlinear::pushover::PushoverControl::LoadOnly => "荷重増分のみ",
            };
            ui.label(format!("増分方式: {}", control));
        });
        // 塑性率（構造力学）の方式と最大値。
        ui.horizontal(|ui| {
            use squid_n_solver::nonlinear::pushover::DuctilityMethod;
            let method = match self.core.analysis_cfg.ductility_method {
                DuctilityMethod::ReferenceStrain => "基点歪み",
                DuctilityMethod::WeightedAverageJm => "重み付け平均Jm",
                DuctilityMethod::FirstYield => "降伏時",
            };
            let max_mu = po
                .hinges
                .iter()
                .map(|h| h.ductility)
                .fold(0.0_f64, f64::max);
            ui.label(format!("塑性率方式: {method}"));
            ui.separator();
            ui.label(format!("最大部材塑性率 μmax = {:.2}", max_mu));
        });

        // 層別の保有水平耐力（性能曲線・層別ピーク層せん断力）。加力方向により
        // 符号を持ちうるため絶対値を取ってから最大値を求める
        // （crates/squid-n-app/src/app/actions.rs の `story_qu` 算定と同じ着眼＝
        // capacity_curve 全点にわたる層せん断力の最大値／βu の分母）。
        let layers = self.core.model.layers();
        let n_stories = layers.len();
        let story_name = |i: usize| -> String {
            layers
                .get(i)
                .map(|l| l.name.clone())
                .unwrap_or_else(|| squid_n_core::model::default_story_name(i))
        };
        let story_qu_kn: Vec<f64> = (0..n_stories)
            .map(|i| {
                force_kn(
                    po.capacity_curve
                        .iter()
                        .filter_map(|p| p.story_shear.get(i).copied())
                        .map(f64::abs)
                        .fold(0.0_f64, f64::max),
                )
            })
            .collect();
        if !story_qu_kn.is_empty() {
            let line = story_qu_kn
                .iter()
                .enumerate()
                .map(|(i, q)| format!("{} {:.1} kN", story_name(i), q))
                .collect::<Vec<_>>()
                .join(" / ");
            ui.label(format!("層別 Qu: {line}"));
        }

        // 性能曲線（層別: 層間変位 - 層せん断力）。層ごとに 1 本の折れ線を描く
        // （既存の色（`crate::theme` のデータ系色）を層番号で巡回して使用）。
        const STORY_COLORS: [egui::Color32; 8] = [
            crate::theme::DATA_BLUE,
            crate::theme::GOOD_GREEN,
            crate::theme::PARETO_RED,
            crate::theme::BEST_YELLOW,
            crate::theme::HILITE_PURPLE,
            crate::theme::SECONDARY_AMBER,
            crate::theme::BLUE_600,
            crate::theme::GREEN_600,
        ];
        egui_plot::Plot::new("pushover_curve")
            .x_axis_label("層間変位 [mm]")
            .y_axis_label("層せん断力 [kN]")
            .legend(egui_plot::Legend::default())
            .height(ui.available_height() * 0.6)
            .show(ui, |plot_ui| {
                for i in 0..n_stories {
                    let points: Vec<[f64; 2]> = po
                        .capacity_curve
                        .iter()
                        .map(|p| {
                            let drift = p.story_drift.get(i).copied().unwrap_or(0.0).abs();
                            let shear =
                                force_kn(p.story_shear.get(i).copied().unwrap_or(0.0).abs());
                            [drift, shear]
                        })
                        .collect();
                    let color = STORY_COLORS[i % STORY_COLORS.len()];
                    plot_ui.line(
                        egui_plot::Line::new(
                            story_name(i),
                            egui_plot::PlotPoints::from(points.clone()),
                        )
                        .color(color)
                        .width(2.0_f32),
                    );
                    // 実際に釣合いを解いて確定した増分ステップの点をマーカーで示す。
                    // 点間を結ぶ折れ線は単なる補間であり計算結果ではないため、
                    // どこが計算点かをマーカーで判別できるようにする（同名で登録し
                    // 凡例のエントリは折れ線と共有する）。
                    plot_ui.points(
                        egui_plot::Points::new(story_name(i), egui_plot::PlotPoints::from(points))
                            .color(color)
                            .radius(3.0_f32)
                            .shape(egui_plot::MarkerShape::Circle),
                    );
                }
            });

        // ヒンジ発生履歴（先頭 20 件）
        ui.separator();
        ui.strong("ヒンジ発生履歴");
        egui::ScrollArea::vertical().show(ui, |ui| {
            for h in po.hinges.iter().take(20) {
                let level = match h.level {
                    squid_n_solver::nonlinear::pushover::HingeLevel::Crack => "ひび割れ",
                    squid_n_solver::nonlinear::pushover::HingeLevel::Yield => "降伏",
                    squid_n_solver::nonlinear::pushover::HingeLevel::Ultimate => "終局",
                };
                ui.label(format!(
                    "step {}: 部材 {} pos={:.2} {} (μ={:.2})",
                    h.step, h.elem.0, h.pos, level, h.ductility
                ));
            }
            if po.hinges.len() > 20 {
                ui.label(format!("... 他 {} 件", po.hinges.len() - 20));
            }
        });
    }

    /// 質点系の結果表示（実行は右バー「質点系」パネル）。
    pub(crate) fn lumped_mass_panel(&mut self, ui: &mut egui::Ui) {
        let Some(lm_res) = self
            .core
            .scoped
            .results
            .as_ref()
            .and_then(|r| r.lumped.as_ref())
        else {
            ui.colored_label(
                crate::theme::GRAY_600,
                "質点系の結果がありません。右バー「質点系」から実行してください。",
            );
            return;
        };
        let lm = &lm_res.model;
        let modal = &lm_res.modal;
        let spatial = lm.is_spatial();

        let total_mass: f64 = lm.stories.iter().map(|s| s.mass).sum();
        ui.horizontal(|ui| {
            ui.label(format!("質点数: {}", lm.stories.len()));
            ui.separator();
            ui.label(format!("総質量: {:.1} t", total_mass));
            ui.separator();
            ui.label(lm.dim.label());
            ui.separator();
            ui.label(if lm.nonlinear { "非線形" } else { "線形" });
            if !lm.nonlinear {
                ui.separator();
                ui.label(lm.stiffness_source.label());
            }
        });
        ui.separator();

        let order: Vec<usize> = (0..lm.stories.len()).rev().collect();
        if spatial {
            crate::table_util::standard_table(
                ui,
                "lumped_mass_stories_3d",
                &[
                    Col::label("階"),
                    Col::num("質量[t]"),
                    Col::num("J[t·mm²]").hover("回転慣性（剛床マスターの RZ 質量）"),
                    Col::num("Kx[kN/mm]"),
                    Col::num("Ky[kN/mm]"),
                    Col::num("KR[N·mm/rad]").hover("剛心まわりのねじり剛性"),
                    Col::wide_num("重心 x,y"),
                    Col::wide_num("剛心 x,y"),
                ],
                order.len(),
                |row| {
                    let i = order[row.index()];
                    let stick = &lm.stories[i];
                    let sp = &lm.spatial[i];
                    let name = self
                        .core
                        .model
                        .layers()
                        .get(i)
                        .map(|l| l.name.clone())
                        .unwrap_or_else(|| "-".to_string());
                    row.col(|ui| {
                        crate::table_util::text_cell(ui, &name);
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.2}", stick.mass));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.3e}", sp.j));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.1}", stiffness_kn_per_mm(sp.k1_x)));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.1}", stiffness_kn_per_mm(sp.k1_y)));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.3e}", sp.kr));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.0}, {:.0}", sp.mass_xy[0], sp.mass_xy[1]));
                    });
                    row.col(|ui| {
                        ui.label(format!(
                            "{:.0}, {:.0}",
                            sp.rigidity_xy[0], sp.rigidity_xy[1]
                        ));
                    });
                },
            );
        } else {
            crate::table_util::standard_table(
                ui,
                "lumped_mass_stories",
                &[
                    Col::label("階"),
                    Col::num("質量[t]"),
                    Col::num("階高[mm]"),
                    Col::num("K1[kN/mm]"),
                    Col::num("K2[kN/mm]"),
                    Col::num("K3[kN/mm]"),
                    Col::wide_num("第1折点 δ1/Q1"),
                    Col::wide_num("第2折点 δ2/Q2"),
                    Col::wide_num("第3折点 δ3/Q3"),
                ],
                order.len(),
                |row| {
                    let i = order[row.index()];
                    let stick = &lm.stories[i];
                    let name = self
                        .core
                        .model
                        .layers()
                        .get(i)
                        .map(|l| l.name.clone())
                        .unwrap_or_else(|| "-".to_string());
                    let sk = &stick.skeleton;
                    row.col(|ui| {
                        crate::table_util::text_cell(ui, &name);
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.2}", stick.mass));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.0}", stick.height));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.1}", stiffness_kn_per_mm(sk.k1)));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.1}", stiffness_kn_per_mm(sk.k2())));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.1}", stiffness_kn_per_mm(sk.k3())));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.2} / {:.0}", sk.d1, force_kn(sk.q1)));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.2} / {:.0}", sk.d2, force_kn(sk.q2)));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.2} / {:.0}", sk.d3, force_kn(sk.q3)));
                    });
                },
            );
        }

        ui.separator();
        ui.label("固有値");
        if modal.period.is_empty() {
            ui.colored_label(crate::theme::GRAY_600, "モードがありません。");
        } else {
            crate::table_util::standard_table(
                ui,
                "lumped_mass_modal",
                &[
                    Col::label("次数"),
                    Col::num("周期 T[s]"),
                    Col::wide_num("モード形状（下層→上層）"),
                ],
                modal.period.len(),
                |row| {
                    let j = row.index();
                    row.col(|ui| {
                        ui.label(format!("{}", j + 1));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.4}", modal.period[j]));
                    });
                    row.col(|ui| {
                        let shape = if spatial && j < modal.shapes_xyz.len() {
                            modal.shapes_xyz[j]
                                .iter()
                                .map(|v| format!("({:.2},{:.2},{:.3})", v[0], v[1], v[2]))
                                .collect::<Vec<_>>()
                                .join(" ")
                        } else {
                            modal.shapes[j]
                                .iter()
                                .map(|v| format!("{v:.2}"))
                                .collect::<Vec<_>>()
                                .join(", ")
                        };
                        crate::table_util::text_cell(ui, &shape);
                    });
                },
            );
        }

        ui.separator();
        if let Some(res) = lm_res
            .response
            .as_ref()
            .or(self.core.scoped.stick_response.as_ref())
        {
            let names: Vec<String> = (0..res.story_peak_drift.len())
                .map(|i| {
                    self.core
                        .model
                        .layers()
                        .get(i)
                        .map(|l| l.name.clone())
                        .unwrap_or_else(|| "-".to_string())
                })
                .collect();
            let (roof_x, roof_y, roof_45, roof_max) = res.roof_dir_peaks();
            let mu_x = res.ductility_dir.x.iter().cloned().fold(0.0f64, f64::max);
            let mu_y = res.ductility_dir.y.iter().cloned().fold(0.0f64, f64::max);
            let mu_45 = res
                .ductility_dir
                .deg45
                .iter()
                .cloned()
                .fold(0.0f64, f64::max);
            let mu_max = res.story_ductility.iter().cloned().fold(0.0f64, f64::max);
            if res.drift_dir.has_values() {
                ui.label(format!(
                    "頂部最大変位 [mm]  X: {:.2}  Y: {:.2}  45°: {:.2}  最大: {:.2}",
                    roof_x, roof_y, roof_45, roof_max
                ));
                ui.label(format!(
                    "最大層塑性率 μ  X: {:.2}  Y: {:.2}  45°: {:.2}  最大: {:.2}",
                    mu_x, mu_y, mu_45, mu_max
                ));
            } else {
                ui.horizontal(|ui| {
                    ui.label(format!("頂部最大変位: {:.2} mm", roof_max));
                    ui.separator();
                    ui.label(format!("最大層塑性率 μ: {:.2}", mu_max));
                });
            }
            if res.non_converged_steps > 0 {
                ui.colored_label(
                    crate::theme::SECONDARY_AMBER,
                    format!(
                        "⚠ Newton 反復が {} ステップで収束しませんでした。応答値は参考値です。",
                        res.non_converged_steps
                    ),
                );
            }
            let order: Vec<usize> = (0..res.story_peak_drift.len()).rev().collect();
            if res.drift_dir.has_values() {
                lumped_dir_peak_table(
                    ui,
                    &LumpedDirTable {
                        salt: "stick_th_drift",
                        title: "最大層間変形 [mm]",
                        hover:
                            "剛心位置の層間。45° は 45° と 135° の投影の大きい方。最大は水平合成。",
                        order: &order,
                        names: &names,
                        x: &res.drift_dir.x,
                        y: &res.drift_dir.y,
                        deg45: &res.drift_dir.deg45,
                        maxv: &res.story_peak_drift,
                    },
                    |v| format!("{v:.2}"),
                );
                lumped_dir_peak_table(
                    ui,
                    &LumpedDirTable {
                        salt: "stick_th_shear",
                        title: "最大層せん断 [kN]",
                        hover: "層ばね力。45° は 45° と 135° の投影の大きい方。最大は水平合成。",
                        order: &order,
                        names: &names,
                        x: &res.shear_dir.x,
                        y: &res.shear_dir.y,
                        deg45: &res.shear_dir.deg45,
                        maxv: &res.story_peak_shear,
                    },
                    |v| format!("{:.0}", force_kn(v)),
                );
                lumped_dir_peak_table(
                    ui,
                    &LumpedDirTable {
                        salt: "stick_th_mu",
                        title: "塑性率 μ",
                        hover:
                            "μX=δX/δ1x、μY=δY/δ1y、μ45=δ45/(√2 min(δ1x,δ1y))。最大は max(μX, μY)。",
                        order: &order,
                        names: &names,
                        x: &res.ductility_dir.x,
                        y: &res.ductility_dir.y,
                        deg45: &res.ductility_dir.deg45,
                        maxv: &res.story_ductility,
                    },
                    |v| format!("{v:.2}"),
                );
            } else {
                crate::table_util::standard_table(
                    ui,
                    "stick_th_result",
                    &[
                        Col::label("階"),
                        Col::num("最大層間変形[mm]"),
                        Col::num("最大層せん断[kN]"),
                        Col::num("塑性率μ"),
                    ],
                    order.len(),
                    |row| {
                        let i = order[row.index()];
                        row.col(|ui| {
                            crate::table_util::text_cell(ui, &names[i]);
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.2}", res.story_peak_drift[i]));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.0}", force_kn(res.story_peak_shear[i])));
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.2}", res.story_ductility[i]));
                        });
                    },
                );
            }
            let pts: Vec<[f64; 2]> = res
                .time
                .iter()
                .zip(res.roof_disp.iter())
                .map(|(&t, &d)| [t, d])
                .collect();
            egui_plot::Plot::new("stick_roof_plot")
                .height(160.0)
                .x_axis_label("時間[s]")
                .y_axis_label("頂部変位[mm]")
                .show(ui, |pu| {
                    pu.line(
                        egui_plot::Line::new("roof", egui_plot::PlotPoints::from(pts))
                            .color(crate::theme::DATA_BLUE),
                    );
                });
        } else {
            ui.colored_label(
                crate::theme::GRAY_600,
                "時刻歴は未実行です。右バー「質点系」から実行できます。",
            );
        }
    }
}

fn lumped_dir_peak_table(
    ui: &mut egui::Ui,
    spec: &LumpedDirTable<'_>,
    fmt: impl Fn(f64) -> String,
) {
    ui.add_space(4.0);
    ui.label(spec.title);
    crate::table_util::standard_table(
        ui,
        spec.salt,
        &[
            Col::label("階"),
            Col::num("X").hover(spec.hover),
            Col::num("Y").hover(spec.hover),
            Col::num("45°").hover(spec.hover),
            Col::num("最大").hover(spec.hover),
        ],
        spec.order.len(),
        |row| {
            let i = spec.order[row.index()];
            let cell = |ui: &mut egui::Ui, v: &[f64]| {
                ui.label(fmt(v.get(i).copied().unwrap_or(0.0)));
            };
            row.col(|ui| {
                crate::table_util::text_cell(
                    ui,
                    spec.names.get(i).map(String::as_str).unwrap_or("-"),
                );
            });
            row.col(|ui| cell(ui, spec.x));
            row.col(|ui| cell(ui, spec.y));
            row.col(|ui| cell(ui, spec.deg45));
            row.col(|ui| cell(ui, spec.maxv));
        },
    );
}

struct LumpedDirTable<'a> {
    salt: &'a str,
    title: &'a str,
    hover: &'a str,
    order: &'a [usize],
    names: &'a [String],
    x: &'a [f64],
    y: &'a [f64],
    deg45: &'a [f64],
    maxv: &'a [f64],
}
