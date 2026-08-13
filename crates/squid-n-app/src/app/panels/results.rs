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
        let current_key = self.nav.focus_result.or(self.last_static);
        let mut selected_result: Option<StaticKey> = None;
        ui.horizontal(|ui| {
            let sel_spatial = self.results_view == ResultsView::Spatial;
            let sel_th = self.results_view == ResultsView::TimeHistory;
            let sel_po = self.results_view == ResultsView::Pushover;
            let sel_lm = self.results_view == ResultsView::LumpedMass;
            if ui.selectable_label(sel_spatial, "3D/応力図").clicked() {
                self.results_view = ResultsView::Spatial;
            }
            if ui.selectable_label(sel_th, "時刻歴").clicked() {
                self.results_view = ResultsView::TimeHistory;
            }
            if ui.selectable_label(sel_po, "増分解析").clicked() {
                self.results_view = ResultsView::Pushover;
            }
            if ui.selectable_label(sel_lm, "質点系モデル").clicked() {
                self.results_view = ResultsView::LumpedMass;
            }
            ui.separator();
            // 結果サマリ
            if let Some(r) = &self.results {
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
        match self.results_view {
            ResultsView::Spatial => crate::viewer::viewer_panel(ui, self),
            ResultsView::TimeHistory => crate::time_history_view::time_history_panel(ui, self),
            ResultsView::Pushover => self.pushover_results_panel(ui),
            ResultsView::LumpedMass => self.lumped_mass_panel(ui),
        }
    }
    /// 増分解析結果（性能曲線・ヒンジ・崩壊機構）の表示。
    pub(crate) fn pushover_results_panel(&mut self, ui: &mut egui::Ui) {
        if self
            .results
            .as_ref()
            .and_then(|r| r.pushover.as_ref())
            .is_none()
        {
            ui.colored_label(
                crate::theme::GRAY_600,
                "増分解析結果がありません。解析タブから実行してください。",
            );
            return;
        }

        // 必要保有水平耐力の総合判定（Qu ≥ Qun = Ds·Fes·Qud）を先に算定する。
        // 実行ボタン→結果画面でそのまま OK/NG を確認できるよう、性能曲線より前に
        // バナー表示する。`compute_holding_capacity` は &mut self を要するため、
        // 以降の `po` 借用より前にここで所有権付きの結果へ落とす。
        let hc_verdict = self.compute_holding_capacity().ok();

        let po = self
            .results
            .as_ref()
            .and_then(|r| r.pushover.as_ref())
            .expect("checked above");

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
            squid_n_solver::pushover::MechanismType::Partial
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
                squid_n_solver::pushover::MechanismType::Overall => "全体崩壊形".to_string(),
                squid_n_solver::pushover::MechanismType::StoryCollapse { layer } => {
                    // 層の呼び名は下端の階名（法令の「i 階」）。
                    let name = self
                        .model
                        .layers()
                        .get(*layer)
                        .map(|l| l.name.clone())
                        .unwrap_or_else(|| format!("{}", layer + 1));
                    format!("層崩壊形 ({name})")
                }
                squid_n_solver::pushover::MechanismType::Partial => "部分崩壊形".to_string(),
            };
            ui.label(format!("崩壊機構: {}", mech));
            ui.separator();
            ui.label(format!("ヒンジ発生 {} 件", po.hinges.len()));
            ui.separator();
            let control = match po.control {
                squid_n_solver::pushover::PushoverControl::Phased => "段階制御",
                squid_n_solver::pushover::PushoverControl::LoadOnly => "荷重増分のみ",
            };
            ui.label(format!("増分方式: {}", control));
        });
        // 塑性率（構造力学）の方式と最大値。
        ui.horizontal(|ui| {
            use squid_n_solver::pushover::DuctilityMethod;
            let method = match self.analysis_cfg.ductility_method {
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
        let layers = self.model.layers();
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
                    squid_n_solver::pushover::HingeLevel::Crack => "ひび割れ",
                    squid_n_solver::pushover::HingeLevel::Yield => "降伏",
                    squid_n_solver::pushover::HingeLevel::Ultimate => "終局",
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

    /// 質点系（串団子）モデルの表示。増分解析結果から層 Q-δ を
    /// トリリニア縮約し、層ごとの質量・階高・復元力特性を一覧する
    /// （構造動力学の質点系解析モデル）。
    pub(crate) fn lumped_mass_panel(&mut self, ui: &mut egui::Ui) {
        use squid_n_solver::lumped_mass::{build_lumped_mass_model, LumpedMassType};

        let Some(po) = self.results.as_ref().and_then(|r| r.pushover.as_ref()) else {
            ui.colored_label(
                crate::theme::GRAY_600,
                "増分解析結果がありません。質点系モデルは\
                 増分解析結果から生成します。解析タブから実行してください。",
            );
            return;
        };

        // モデル化タイプ・第1折点判定の割線剛性比を選択。
        ui.horizontal(|ui| {
            ui.label("モデル化タイプ:");
            let cur = self.analysis_cfg.lumped_mass_type;
            egui::ComboBox::from_id_salt("lumped_mass_type")
                .selected_text(cur.label())
                .show_ui(ui, |ui| {
                    for t in [
                        LumpedMassType::EquivalentShear,
                        LumpedMassType::EquivalentBendingShear,
                        LumpedMassType::BendingShearSeparated,
                    ] {
                        ui.selectable_value(&mut self.analysis_cfg.lumped_mass_type, t, t.label());
                    }
                });
            ui.separator();
            ui.label("第1折点 割線比:");
            ui.add(
                egui::DragValue::new(&mut self.analysis_cfg.lumped_secant_ratio)
                    .speed(0.01)
                    .range(0.3..=0.95),
            );
        });
        ui.separator();

        // 増分解析から串団子モデルを生成（軽量なので毎フレーム再構成）。
        let lm = build_lumped_mass_model(
            &self.model,
            po,
            self.analysis_cfg.lumped_mass_type,
            self.analysis_cfg.lumped_secant_ratio,
        );

        let total_mass: f64 = lm.stories.iter().map(|s| s.mass).sum();
        ui.horizontal(|ui| {
            ui.label(format!("質点数: {}", lm.stories.len()));
            ui.separator();
            ui.label(format!("総質量: {:.1} t", total_mass));
            ui.separator();
            ui.label(format!("モデル: {}", lm.model_type.label()));
        });
        ui.separator();

        // 層ごとの質点・復元力特性（トリリニア）を一覧。上層から順に表示。
        egui::ScrollArea::vertical().show(ui, |ui| {
            // model.stories と stick は同順（build_lumped_mass_model が順に生成）。
            // 上層から順に見せるため、表示順は逆順にした索引で引く。
            let order: Vec<usize> = (0..lm.stories.len()).rev().collect();
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
                        .model
                        .stories
                        .get(i)
                        .map(|s| s.name.as_str())
                        .unwrap_or("-");
                    let sk = &stick.skeleton;
                    row.col(|ui| {
                        crate::table_util::text_cell(ui, name);
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

            ui.add_space(6.0);
            ui.colored_label(
                crate::theme::GRAY_600,
                "K は [kN/mm]、Q は [kN]、δ は [mm]。骨格は増分解析の層 Q-δ を\
                 等包絡面積則でトリリニア縮約したもの。",
            );
        });

        // ── 質点系（せん断型）固有値解析 ────────────────────────────
        // 初期剛性 K1 ベースのせん断型多質点系（立体モデルの解析タブ「① 準備計算」
        // 「固有値解析」と同じモード数設定 `n_modes` を使う）。立体モデルとの
        // 周期の比較検証を主目的とし、モード形状の値どうしは正規化基準が異なるため
        // 比較しない（下の注記参照）。
        ui.separator();
        ui.label("固有値解析（せん断型・K1 ベース）");
        match squid_n_solver::lumped_mass::lumped_mass_eigen(&lm, self.analysis_cfg.n_modes) {
            Ok(modal) if !modal.period.is_empty() => {
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
                            let shape = modal.shapes[j]
                                .iter()
                                .map(|v| format!("{v:.2}"))
                                .collect::<Vec<_>>()
                                .join(", ");
                            crate::table_util::text_cell(ui, &shape);
                        });
                    },
                );
                ui.add_space(4.0);
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "モード形状は最上階を 1.0 に正規化。立体モデルの固有値解析結果\
                     （M 正規化・別の自由度空間）とは正規化基準が異なるため、\
                     モード形状の値どうしは比較せず、周期のみを比較すること。",
                );
            }
            Ok(_) => {
                ui.colored_label(crate::theme::GRAY_600, "層がありません。");
            }
            Err(e) => {
                ui.colored_label(
                    crate::theme::SECONDARY_AMBER,
                    format!("固有値解析できません: {e}"),
                );
            }
        }

        // ── 質点系（せん断型）時刻歴応答解析 ──────────────────────────
        ui.separator();
        let mut run_stick = false;
        let mut clear_stick = false;
        ui.horizontal(|ui| {
            if ui
                .button("▶ 質点系時刻歴を実行")
                .on_hover_text(
                    "サンプル波（「時刻歴応答」の dt/継続/周期/振幅・減衰比）で串団子モデルの\
                     非線形時刻歴（Newmark-β、各層トリリニア）を実行します",
                )
                .clicked()
            {
                run_stick = true;
            }
            if self.stick_response.is_some() && ui.button("結果クリア").clicked() {
                clear_stick = true;
            }
        });
        if run_stick {
            let accel = Self::sample_wave(&self.analysis_cfg).accel_x;
            let res = squid_n_solver::lumped_mass::lumped_mass_time_history(
                &lm,
                &accel,
                self.analysis_cfg.th_dt,
                self.analysis_cfg.th_damping,
            );
            self.stick_response = Some(res);
        }
        if clear_stick {
            self.stick_response = None;
        }
        if let Some(res) = &self.stick_response {
            let roof_peak = res
                .roof_disp
                .iter()
                .cloned()
                .fold(0.0f64, |m, v| m.max(v.abs()));
            let mu_max = res.story_ductility.iter().cloned().fold(0.0f64, f64::max);
            ui.horizontal(|ui| {
                ui.label(format!("頂部最大変位: {:.2} mm", roof_peak));
                ui.separator();
                ui.label(format!("最大層塑性率 μ: {:.2}", mu_max));
            });
            if res.non_converged_steps > 0 {
                ui.colored_label(
                    crate::theme::SECONDARY_AMBER,
                    format!(
                        "⚠ Newton 反復が {} ステップで収束しませんでした。\
                         応答値は参考値です（dt を小さくすると改善する場合があります）。",
                        res.non_converged_steps
                    ),
                );
            }
            // 上層から順に見せるため、表示順は逆順にした索引で引く。
            let order: Vec<usize> = (0..res.story_peak_drift.len()).rev().collect();
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
                    let name = self
                        .model
                        .stories
                        .get(i)
                        .map(|s| s.name.as_str())
                        .unwrap_or("-");
                    row.col(|ui| {
                        crate::table_util::text_cell(ui, name);
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
        }
    }
}
