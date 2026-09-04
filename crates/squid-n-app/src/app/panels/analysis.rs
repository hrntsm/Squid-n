//! 右ドックの解析パネル群（静的・固有値・増分・時刻歴）。
//!
//! `panels` からの構造分割。アルゴリズム変更は行わない。

use super::*;

impl App {
    /// 解析パネル共通ヘッダ（最終実行・stale 表示）。
    fn analysis_status_header(&self, ui: &mut egui::Ui) {
        if let Some(when) = self.core.scoped.staleness.last_run {
            if let Ok(dur) = when.elapsed() {
                ui.label(format!("最終実行: {:.0} 秒前", dur.as_secs_f64()));
            } else {
                ui.label("最終実行: 不明");
            }
        } else {
            ui.label("最終実行: なし");
        }
        if self.core.scoped.staleness.results_stale {
            ui.colored_label(
                crate::theme::WARN_TEXT,
                "⚠ モデルが編集されました。結果は再計算が必要です。",
            );
        }
        if self.core.scoped.staleness.preparation_stale {
            ui.colored_label(
                crate::theme::WARN_TEXT,
                "⚠ 準備計算が未実行、またはモデル編集により古くなっています\
                 （解析の実行時に自動で最新化されます）。",
            );
        }
    }

    /// 右ドック「静的解析」パネル。
    pub(crate) fn static_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("静的解析");
        ui.separator();
        self.analysis_status_header(ui);
        ui.separator();
        let running = self.core.scoped.job.is_some();
        self.static_analysis_section(ui, running);
    }

    /// 右ドック「固有値」パネル。
    pub(crate) fn eigen_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("固有値");
        ui.separator();
        self.analysis_status_header(ui);
        ui.separator();
        let running = self.core.scoped.job.is_some();
        self.eigen_section(ui, running);
    }

    /// 右ドック「増分解析」パネル。
    pub(crate) fn pushover_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("増分解析");
        ui.separator();
        self.analysis_status_header(ui);
        ui.separator();
        let running = self.core.scoped.job.is_some();
        self.pushover_section(ui, running);
    }

    /// 右ドック「時刻歴応答」パネル。
    pub(crate) fn time_history_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("時刻歴応答");
        ui.separator();
        self.analysis_status_header(ui);
        ui.separator();
        let running = self.core.scoped.job.is_some();
        self.time_history_section(ui, running);
    }

    /// 静的解析（荷重ケース単体・荷重組合せ）の実行。
    ///
    /// 実行導線は「単体実行」と「一括解析」の 2 つに統一する。求解の最小単位は
    /// 荷重ケース単体で、荷重組合せはその結果の線形和として組み立てるため
    /// （重ね合わせの原理。`Analysis::linear_combination`）、荷重ケースと荷重組合せを
    /// 別の実行導線に分ける理由がない。
    ///
    /// - **単体実行**: 選択した 1 件（荷重ケースまたは荷重組合せ）を解く。
    /// - **一括解析**: 全荷重ケースを解き、全荷重組合せをその線形和で求める。
    ///
    /// 地震力（EX/EY）・風圧力（WX/WY）も準備計算が生成する荷重ケースなので、
    /// ここから同じ導線で実行する（[`App::start_load_case_job`] が方向別の結果キーへ
    /// 振り分ける）。
    fn static_analysis_section(&mut self, ui: &mut egui::Ui, running: bool) {
        let target = self.resolved_analysis_target();
        ui.horizontal_wrapped(|ui| {
            ui.label("対象:");
            let text = target
                .and_then(|t| self.static_target_label(t))
                .unwrap_or_else(|| "（なし）".to_string());
            egui::ComboBox::from_id_salt("analysis_target")
                .selected_text(text)
                .show_ui(ui, |ui| {
                    // 荷重ケースと荷重組合せを 1 つの一覧に並べる（見出しで区切る）。
                    ui.label(egui::RichText::new("荷重ケース").color(crate::theme::GRAY_600));
                    let cases: Vec<(LoadCaseId, String)> = self
                        .core
                        .model
                        .load_cases
                        .iter()
                        .map(|c| (c.id, format!("[{}] {}", c.id.0, c.name)))
                        .collect();
                    for (id, label) in cases {
                        let t = StaticTarget::Case(id);
                        if ui.selectable_label(target == Some(t), label).clicked() {
                            self.ui.scoped.analysis_target = Some(t);
                            self.ui.scoped.nav.focus_load_case = Some(id);
                        }
                    }
                    if !self.core.model.combinations.is_empty() {
                        ui.separator();
                        ui.label(egui::RichText::new("荷重組合せ").color(crate::theme::GRAY_600));
                    }
                    let combos: Vec<String> = self
                        .core
                        .model
                        .combinations
                        .iter()
                        .map(|c| c.name.clone())
                        .collect();
                    for (i, name) in combos.into_iter().enumerate() {
                        let t = StaticTarget::Combo(i);
                        if ui.selectable_label(target == Some(t), name).clicked() {
                            self.ui.scoped.analysis_target = Some(t);
                        }
                    }
                });
            if ui
                .add_enabled(
                    target.is_some() && !running,
                    egui::Button::new("▶ 単体実行"),
                )
                .on_hover_text(
                    "選択した荷重ケース／荷重組合せを 1 件だけ解析します\
                             （組合せは参照する荷重ケースを解いてから線形和で求めます）",
                )
                .clicked()
            {
                if let Some(t) = target {
                    self.start_static_target_job(t);
                    if self.core.scoped.last_error.is_none() {
                        self.ui.view.active_tab = Tab::Results;
                        self.ui.view.results_view = ResultsView::Spatial;
                    }
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(!running, egui::Button::new("▶▶ 一括解析"))
                .on_hover_text(
                    "全ての荷重ケースを解析し、全ての荷重組合せをその結果の線形和で\
                             求めます（並列スレッド数設定を使用）",
                )
                .clicked()
            {
                self.start_static_all_job();
                if self.core.scoped.last_error.is_none() {
                    self.ui.view.active_tab = Tab::Results;
                    self.ui.view.results_view = ResultsView::Spatial;
                }
            }
        });
        if self.core.model.load_cases.is_empty() {
            ui.colored_label(
                crate::theme::GRAY_600,
                "荷重ケースがありません。荷重タブで作成してください。",
            );
        } else {
            ui.colored_label(
                crate::theme::GRAY_600,
                "EX/EY（地震力）は準備計算が自動生成します。\
                     荷重組合せは荷重ケース単体の解析結果の線形和として求めます。",
            );
        }
    }

    /// 静的解析の実行対象（「対象」ドロップダウンの選択）を解決する。
    ///
    /// 優先順は「選択中の対象（モデル編集で失効していないもの）」→
    /// 「ナビゲータ／荷重表で選択中の荷重ケース」→「荷重ケースの先頭」。
    fn resolved_analysis_target(&self) -> Option<StaticTarget> {
        let valid = |t: StaticTarget| match t {
            StaticTarget::Case(id) => self.core.model.load_cases.iter().any(|c| c.id == id),
            StaticTarget::Combo(i) => i < self.core.model.combinations.len(),
        };
        self.ui
            .scoped
            .analysis_target
            .filter(|t| valid(*t))
            .or_else(|| {
                self.ui
                    .scoped
                    .nav
                    .focus_load_case
                    .filter(|id| self.core.model.load_cases.iter().any(|c| c.id == *id))
                    .map(StaticTarget::Case)
            })
            .or_else(|| {
                self.core
                    .model
                    .load_cases
                    .first()
                    .map(|c| StaticTarget::Case(c.id))
            })
    }

    /// 静的解析の実行対象の表示名（ドロップダウンの選択表示）。対象が失効している
    /// 場合は `None`。
    fn static_target_label(&self, target: StaticTarget) -> Option<String> {
        match target {
            StaticTarget::Case(id) => self
                .core
                .model
                .load_cases
                .iter()
                .find(|c| c.id == id)
                .map(|c| format!("[{}] {}", c.id.0, c.name)),
            StaticTarget::Combo(i) => self.core.model.combinations.get(i).map(|c| c.name.clone()),
        }
    }

    /// 固有値解析。
    fn eigen_section(&mut self, ui: &mut egui::Ui, running: bool) {
        ui.horizontal_wrapped(|ui| {
            ui.label("モード数:");
            let mut n = self.core.analysis_cfg.n_modes;
            ui.add(egui::DragValue::new(&mut n).range(1..=30));
            self.core.analysis_cfg.n_modes = n;
            if ui
                .add_enabled(!running, egui::Button::new("▶ 実行"))
                .clicked()
            {
                // UI スレッドをブロックしないようバックグラウンドで実行する
                // （他の解析と同じジョブ経路）。
                self.start_eigen_job(self.core.analysis_cfg.n_modes);
            }
        });
        ui.colored_label(
            crate::theme::GRAY_600,
            "質量方式は準備計算の「計算条件」で設定します。",
        );
    }

    /// 増分解析（プッシュオーバー）。
    fn pushover_section(&mut self, ui: &mut egui::Ui, running: bool) {
        ui.horizontal_wrapped(|ui| {
            ui.label("方向:");
            ui.selectable_value(&mut self.core.analysis_cfg.push_dir, SeismicDir::X, "X");
            ui.selectable_value(&mut self.core.analysis_cfg.push_dir, SeismicDir::Y, "Y");
            ui.label("ステップ:");
            ui.add(egui::DragValue::new(&mut self.core.analysis_cfg.push_steps).range(1..=100));
        });
        // 終了目標（いずれかへの到達で解析を打ち切る）。両方とも無効なら
        // 荷重制御 λ=1 まで解析する（solver 側 PushoverTarget の既定挙動）。
        ui.horizontal_wrapped(|ui| {
            ui.checkbox(
                &mut self.core.analysis_cfg.push_use_drift_angle,
                "目標層間変形角:",
            )
            .on_hover_text("全層の層間変形角がこの値に達した時点で解析を打ち切ります");
            ui.label("1/");
            ui.add_enabled(
                self.core.analysis_cfg.push_use_drift_angle,
                egui::DragValue::new(&mut self.core.analysis_cfg.push_drift_denom)
                    .speed(10.0)
                    .range(50.0..=1000.0),
            );
            ui.separator();
            ui.checkbox(
                &mut self.core.analysis_cfg.push_use_max_disp,
                "目標変位[mm]:",
            )
            .on_hover_text("頂部変位がこの値に達した時点で解析を打ち切ります");
            ui.add_enabled(
                self.core.analysis_cfg.push_use_max_disp,
                egui::DragValue::new(&mut self.core.analysis_cfg.push_max_disp)
                    .speed(10.0)
                    .range(1.0..=10000.0),
            );
        });
        if !self.core.analysis_cfg.push_use_max_disp && !self.core.analysis_cfg.push_use_drift_angle
        {
            ui.colored_label(
                crate::theme::GRAY_600,
                "目標未設定: 荷重制御(λ=1)までで終了します。",
            );
        }
        ui.horizontal_wrapped(|ui| {
            use squid_n_solver::nonlinear::pushover::DuctilityMethod;
            ui.label("塑性率方式:")
                .on_hover_text("ファイバーモデルの塑性率（構造力学）");
            egui::ComboBox::from_id_salt("ductility_method")
                .selected_text(match self.core.analysis_cfg.ductility_method {
                    DuctilityMethod::ReferenceStrain => "基点歪み",
                    DuctilityMethod::WeightedAverageJm => "重み付け平均Jm",
                    DuctilityMethod::FirstYield => "降伏時",
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        &mut self.core.analysis_cfg.ductility_method,
                        DuctilityMethod::ReferenceStrain,
                        "基点歪み（RC:引張0.01/圧縮0.005・鉄骨0.01）",
                    );
                    ui.selectable_value(
                        &mut self.core.analysis_cfg.ductility_method,
                        DuctilityMethod::WeightedAverageJm,
                        "重み付け平均塑性率 Jm≥1",
                    );
                    ui.selectable_value(
                        &mut self.core.analysis_cfg.ductility_method,
                        DuctilityMethod::FirstYield,
                        "降伏発生時（塑性率1）",
                    );
                });
        });
        ui.horizontal_wrapped(|ui| {
            ui.checkbox(
                &mut self.core.analysis_cfg.push_apply_long_term,
                "長期荷重を初期載荷",
            )
            .on_hover_text(
                "長期系荷重ケース（固定・積載等）を水平力増分の前に載荷し、\
                         その応力状態を初期条件とします。長期荷重ケースがない場合は\
                         無視されます。",
            );
        });
        ui.horizontal_wrapped(|ui| {
            use squid_n_solver::nonlinear::pushover::PushoverControl;
            ui.label("増分方式:").on_hover_text(
                "荷重増分のみは比較検証用。変位制御へ移行せず、終了目標が有効な場合は\
                         λ=1 を超えて荷重を増分し、収束しなくなった時点（耐力ピーク近傍）で\
                         打ち切ります。耐力低下域は追跡できません。",
            );
            egui::ComboBox::from_id_salt("push_control")
                .selected_text(match self.core.analysis_cfg.push_control {
                    PushoverControl::Phased => "段階制御（荷重→変位）",
                    PushoverControl::LoadOnly => "荷重増分のみ",
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        &mut self.core.analysis_cfg.push_control,
                        PushoverControl::Phased,
                        "段階制御（荷重→変位）",
                    );
                    ui.selectable_value(
                        &mut self.core.analysis_cfg.push_control,
                        PushoverControl::LoadOnly,
                        "荷重増分のみ",
                    )
                    .on_hover_text(
                        "比較検証用。変位制御へ移行せず、終了目標が有効な場合は\
                                 λ=1 を超えて荷重を増分し、収束しなくなった時点（耐力ピーク近傍）\
                                 で打ち切ります。耐力低下域は追跡できません。",
                    );
                });
        });
        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(!running, egui::Button::new("▶ 実行"))
                .clicked()
            {
                self.start_pushover_job();
            }
            if self
                .core
                .scoped
                .job
                .as_ref()
                .is_some_and(|j| j.label == "増分解析")
            {
                ui.spinner();
            }
        });
    }

    /// 時刻歴応答解析（線形／非線形）。
    fn time_history_section(&mut self, ui: &mut egui::Ui, running: bool) {
        ui.horizontal_wrapped(|ui| {
            ui.label("方向:");
            ui.selectable_value(&mut self.core.analysis_cfg.th_dir, ThDir::X, "X");
            ui.selectable_value(&mut self.core.analysis_cfg.th_dir, ThDir::Y, "Y");
            ui.selectable_value(&mut self.core.analysis_cfg.th_dir, ThDir::Xy, "X+Y")
                .on_hover_text("同一波形を両方向へ同時入力(CSV は2列)");
            ui.separator();
            ui.checkbox(
                &mut self.core.analysis_cfg.th_nonlinear,
                "非線形(復元力特性を考慮)",
            )
            .on_hover_text(
                "各部材の復元力特性（ひび割れ・降伏等）を考慮し、\
                         各時刻ステップを Newton 反復で解く時刻歴応答解析。",
            );
        });
        if self.core.analysis_cfg.th_nonlinear {
            ui.horizontal_wrapped(|ui| {
                ui.label("Newton反復: 最大回数");
                ui.add(
                    egui::DragValue::new(&mut self.core.analysis_cfg.th_max_iter).range(1..=500),
                );
                ui.label("収束許容誤差(相対):");
                ui.add(
                    egui::DragValue::new(&mut self.core.analysis_cfg.th_tol)
                        .speed(1e-7)
                        .range(1e-9..=1e-2),
                );
            });
        }
        ui.horizontal_wrapped(|ui| {
            ui.label("記録間引き(0=自動):");
            ui.add(
                egui::DragValue::new(&mut self.core.analysis_cfg.th_record_every).range(0..=100000),
            )
            .on_hover_text(
                "3D アニメーション・層応答グラフ・部材履歴用の詳細記録\
                         （ThRecording）を N ステップごとに 1 フレーム記録します。\
                         0 なら記録フレーム数が概ね 1000 になるよう自動決定します\
                         （線形・非線形のどちらの経路でも共通）。\
                         ピーク値（最大変位・最大内力・層せん断力係数の最大値）は\
                         間引かず全ステップで更新するため、この値は精度ではなく\
                         アニメーション・履歴グラフの解像度とメモリ使用量に影響します。",
            );
        });
        ui.horizontal_wrapped(|ui| {
            ui.add_enabled(
                self.core.analysis_cfg.th_nonlinear,
                egui::Checkbox::new(
                    &mut self.core.analysis_cfg.th_apply_long_term,
                    "長期荷重を初期状態として考慮",
                ),
            )
            .on_hover_text(
                "長期系荷重ケース（固定・積載等）を時刻歴開始前に静的載荷し、\
                         その応力状態を初期条件とします。長期荷重ケースがない場合は\
                         無視されます。",
            )
            .on_disabled_hover_text(
                "線形時刻歴は重ね合わせ運用のため対象外です\
                         （「非線形」をONにすると使用できます）。",
            );
        });
        ui.horizontal_wrapped(|ui| {
            ui.label("減衰:");
            ui.selectable_value(
                &mut self.core.analysis_cfg.th_damping_model,
                ThDampingModel::StiffnessProportional,
                "剛性比例",
            );
            ui.selectable_value(
                &mut self.core.analysis_cfg.th_damping_model,
                ThDampingModel::Rayleigh,
                "Rayleigh",
            );
            ui.selectable_value(
                &mut self.core.analysis_cfg.th_damping_model,
                ThDampingModel::Modal,
                "モード別",
            )
            .on_hover_text("各モードに減衰比 h を与える（非線形は初期剛性モード）");
            ui.selectable_value(
                &mut self.core.analysis_cfg.th_damping_model,
                ThDampingModel::TangentAlpha1,
                "接線(α1一定)",
            )
            .on_hover_text("瞬間剛性比例。C=2h/ω1e·Kt を毎ステップ再構成");
            ui.selectable_value(
                &mut self.core.analysis_cfg.th_damping_model,
                ThDampingModel::TangentH1,
                "接線(h1一定)",
            )
            .on_hover_text("瞬間剛性比例。ω1 を毎ステップ更新し減衰比 h1 を保つ");
            ui.separator();
            ui.label(match self.core.analysis_cfg.th_damping_model {
                ThDampingModel::StiffnessProportional
                | ThDampingModel::TangentAlpha1
                | ThDampingModel::TangentH1 => "減衰比 h:",
                ThDampingModel::Modal => "減衰比 h(全モード):",
                ThDampingModel::Rayleigh => "h1(1次):",
            });
            ui.add(
                egui::DragValue::new(&mut self.core.analysis_cfg.th_damping)
                    .speed(0.005)
                    .range(0.0..=0.3),
            );
            if self.core.analysis_cfg.th_damping_model == ThDampingModel::Rayleigh {
                ui.label("h2(2次):");
                ui.add(
                    egui::DragValue::new(&mut self.core.analysis_cfg.th_h2)
                        .speed(0.005)
                        .range(0.0..=0.3),
                );
            }
        });
        ui.horizontal_wrapped(|ui| {
            ui.label("サンプル波: dt[s]");
            ui.add(
                egui::DragValue::new(&mut self.core.analysis_cfg.th_dt)
                    .speed(0.001)
                    .range(0.001..=0.1),
            );
            ui.label("継続[s]");
            ui.add(
                egui::DragValue::new(&mut self.core.analysis_cfg.th_duration)
                    .speed(0.5)
                    .range(1.0..=120.0),
            );
            ui.label("周期[s]");
            ui.add(
                egui::DragValue::new(&mut self.core.analysis_cfg.th_period)
                    .speed(0.05)
                    .range(0.05..=5.0),
            );
            ui.label("振幅[mm/s²]");
            ui.add(
                egui::DragValue::new(&mut self.core.analysis_cfg.th_amp)
                    .speed(50.0)
                    .range(10.0..=10000.0),
            );
        });
        // 位相差入力（ねじれ加振）。構造動力学の位相差入力解析 t=(L·sinθ)/Vs。
        ui.horizontal_wrapped(|ui| {
            ui.checkbox(&mut self.core.analysis_cfg.phase_diff_enabled, "位相差入力")
                .on_hover_text("見かけ速度で地震動が矩形基礎を通過する位相差からねじれ加振を生成");
            ui.add_enabled_ui(self.core.analysis_cfg.phase_diff_enabled, |ui| {
                ui.label("Vs[m/s]");
                ui.add(
                    egui::DragValue::new(&mut self.core.analysis_cfg.phase_diff_vs)
                        .speed(10.0)
                        .range(50.0..=2000.0),
                );
                ui.label("L[m]");
                ui.add(
                    egui::DragValue::new(&mut self.core.analysis_cfg.phase_diff_length_m)
                        .speed(1.0)
                        .range(1.0..=500.0),
                );
                ui.label("θ[°]");
                ui.add(
                    egui::DragValue::new(&mut self.core.analysis_cfg.phase_diff_incidence_deg)
                        .speed(1.0)
                        .range(0.0..=90.0),
                );
                ui.selectable_value(&mut self.core.analysis_cfg.phase_diff_dir_y, false, "X");
                ui.selectable_value(&mut self.core.analysis_cfg.phase_diff_dir_y, true, "Y");
                let lag = squid_n_solver::dynamic::phase_diff::phase_lag_time(
                    self.core.analysis_cfg.phase_diff_length_m,
                    self.core.analysis_cfg.phase_diff_incidence_deg,
                    self.core.analysis_cfg.phase_diff_vs,
                );
                ui.label(format!("位相遅れ {:.4}s", lag));
            });
        });
        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(!running, egui::Button::new("▶ サンプル波で実行"))
                .on_hover_text("正弦減衰波を生成して時刻歴解析を実行します")
                .clicked()
            {
                let wave = Self::sample_wave(&self.core.analysis_cfg);
                self.start_time_history_job(wave);
            }
            if ui
                .add_enabled(!running, egui::Button::new("📂 波形CSVを開いて実行…"))
                .on_hover_text("1 行 1 値(加速度 gal)の CSV/テキスト。dt は上の設定値を使用します")
                .clicked()
            {
                self.run_time_history_from_csv();
            }
            if self
                .core
                .scoped
                .job
                .as_ref()
                .is_some_and(|j| j.label.starts_with("時刻歴応答"))
            {
                ui.spinner();
            }
        });
        // 波形ライブラリ（「🌊 波形を保存…」で登録した波形。ファイルメニュー参照）
        // から選んで実行する。ライブラリ内容は軽量なので毎フレーム再スキャンする。
        ui.horizontal_wrapped(|ui| {
            ui.label("波形ライブラリ:");
            let lib_dir = squid_n_io::wave_library::wave_library_dir();
            let names: Vec<String> = lib_dir
                .as_deref()
                .and_then(|d| squid_n_io::wave_library::list_wave_library(d).ok())
                .unwrap_or_default();
            if names.is_empty() {
                ui.colored_label(crate::theme::GRAY_600, "登録された波形がありません");
            } else {
                let selected_text = self
                    .core
                    .scoped
                    .wave_library_selection
                    .clone()
                    .unwrap_or_else(|| "(選択してください)".to_string());
                // ドロップダウンでの選び直しは、まだ実行していない＝
                // 「実行時点のハッシュ」を持たない状態に戻る
                // （`set_wave_library_selection` 参照）。`self` を直接
                // `selectable_value` へ渡すとこの破棄処理を経由できない
                // ため、いったんローカル変数で受ける。
                let mut picked = self.core.scoped.wave_library_selection.clone();
                egui::ComboBox::from_id_salt("wave_library_select")
                    .selected_text(selected_text)
                    .show_ui(ui, |ui| {
                        for name in &names {
                            ui.selectable_value(&mut picked, Some(name.clone()), name);
                        }
                    });
                self.set_wave_library_selection(picked);
            }
            if ui
                .add_enabled(
                    !running && self.core.scoped.wave_library_selection.is_some(),
                    egui::Button::new("▶ 選択した波形で実行"),
                )
                .on_hover_text("dt は上の設定値を使用します")
                .clicked()
            {
                self.run_time_history_from_library();
            }
        });
        ui.label(
            egui::RichText::new("応答グラフは入力の大きい方向を記録")
                .small()
                .color(crate::theme::GRAY_600),
        );
    }

    /// 波形 CSV（X/Y: 1 行 1 値、X+Y: 1 行 2 列、いずれも gal 単位）を選択して
    /// 時刻歴解析をジョブ実行する。
    pub(crate) fn run_time_history_from_csv(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("波形 (CSV/テキスト)", &["csv", "txt", "dat"])
            .pick_file()
        else {
            return;
        };
        let content = match std::fs::read_to_string(&path) {
            Ok(c) => c,
            Err(e) => {
                self.report_error(format!("波形読込エラー: {}", e));
                return;
            }
        };
        let cfg = self.core.analysis_cfg;
        let Some(wave) = self.ground_motion_or_report(&cfg, &content) else {
            return;
        };
        self.start_time_history_job(wave);
    }

    /// 右ドック「質点系」パネル。
    pub(crate) fn lumped_mass_analysis_panel(&mut self, ui: &mut egui::Ui) {
        use squid_n_solver::dynamic::lumped_mass::{LumpedStiffnessSource, StickDim};

        ui.heading("質点系");
        ui.separator();
        self.analysis_status_header(ui);
        ui.separator();
        let running = self.core.scoped.job.is_some();

        ui.horizontal_wrapped(|ui| {
            ui.label("次元:");
            ui.selectable_value(
                &mut self.core.analysis_cfg.lumped_dim,
                StickDim::Planar,
                "2次元",
            );
            ui.selectable_value(
                &mut self.core.analysis_cfg.lumped_dim,
                StickDim::Spatial,
                "3次元",
            );
            ui.separator();
            ui.checkbox(&mut self.core.analysis_cfg.lumped_nonlinear, "非線形");
        });
        ui.horizontal_wrapped(|ui| {
            ui.label("方向:");
            ui.selectable_value(&mut self.core.analysis_cfg.lumped_dir, SeismicDir::X, "X");
            ui.selectable_value(&mut self.core.analysis_cfg.lumped_dir, SeismicDir::Y, "Y");
            ui.separator();
            ui.label("剛性:");
            let stiff_enabled = !self.core.analysis_cfg.lumped_nonlinear;
            ui.add_enabled_ui(stiff_enabled, |ui| {
                egui::ComboBox::from_id_salt("lumped_stiffness")
                    .selected_text(self.core.analysis_cfg.lumped_stiffness.label())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut self.core.analysis_cfg.lumped_stiffness,
                            LumpedStiffnessSource::StoryQd,
                            LumpedStiffnessSource::StoryQd.label(),
                        );
                        ui.selectable_value(
                            &mut self.core.analysis_cfg.lumped_stiffness,
                            LumpedStiffnessSource::ColumnKi,
                            LumpedStiffnessSource::ColumnKi.label(),
                        );
                    });
            });
            if !stiff_enabled {
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "非線形時の並進剛性は増分の初期剛性です",
                );
            }
        });
        ui.horizontal_wrapped(|ui| {
            ui.label("モード数:");
            ui.add(egui::DragValue::new(&mut self.core.analysis_cfg.lumped_n_modes).range(1..=30));
            if self.core.analysis_cfg.lumped_nonlinear {
                ui.separator();
                ui.label("第1折点 割線比:");
                ui.add(
                    egui::DragValue::new(&mut self.core.analysis_cfg.lumped_secant_ratio)
                        .speed(0.01)
                        .range(0.3..=0.95),
                );
            }
        });

        let has_ex = self
            .core
            .scoped
            .results
            .as_ref()
            .and_then(|r| r.seismic(SeismicDir::X))
            .is_some();
        let has_ey = self
            .core
            .scoped
            .results
            .as_ref()
            .and_then(|r| r.seismic(SeismicDir::Y))
            .is_some();
        let has_px = self
            .core
            .scoped
            .results
            .as_ref()
            .and_then(|r| r.pushover_x.as_ref())
            .is_some();
        let has_py = self
            .core
            .scoped
            .results
            .as_ref()
            .and_then(|r| r.pushover_y.as_ref())
            .is_some();
        let spatial = self.core.analysis_cfg.lumped_dim == StickDim::Spatial;
        let nl = self.core.analysis_cfg.lumped_nonlinear;

        if !nl {
            let need = if spatial {
                "線形の 3 次元質点系には地震静的 EX と EY の両方が必要です。"
            } else if self.core.analysis_cfg.lumped_dir == SeismicDir::X {
                "線形の 2 次元質点系には地震静的 EX が必要です。"
            } else {
                "線形の 2 次元質点系には地震静的 EY が必要です。"
            };
            let ok = if spatial {
                has_ex && has_ey
            } else if self.core.analysis_cfg.lumped_dir == SeismicDir::X {
                has_ex
            } else {
                has_ey
            };
            if !ok {
                ui.colored_label(crate::theme::WARN_TEXT, need);
            }
        } else {
            let need = if spatial {
                "非線形の 3 次元質点系には X・Y 両方の増分解析と、ねじり剛性のための EX/EY が必要です。"
            } else if self.core.analysis_cfg.lumped_dir == SeismicDir::X {
                "非線形の 2 次元質点系には X 方向の増分解析が必要です。"
            } else {
                "非線形の 2 次元質点系には Y 方向の増分解析が必要です。"
            };
            let ok = if spatial {
                has_px && has_py && has_ex && has_ey
            } else if self.core.analysis_cfg.lumped_dir == SeismicDir::X {
                has_px
            } else {
                has_py
            };
            if !ok {
                ui.colored_label(crate::theme::WARN_TEXT, need);
            }
        }
        if spatial {
            ui.colored_label(
                crate::theme::GRAY_600,
                "3 次元のねじりばね（KR）は常に線形です。",
            );
        }

        ui.separator();
        ui.label("時刻歴（立体時刻歴とは独立）");
        ui.horizontal_wrapped(|ui| {
            ui.label("減衰比 h:");
            ui.add(
                egui::DragValue::new(&mut self.core.analysis_cfg.lumped_th_damping)
                    .speed(0.005)
                    .range(0.0..=0.2),
            );
            ui.label("dt[s]");
            ui.add(
                egui::DragValue::new(&mut self.core.analysis_cfg.lumped_th_dt)
                    .speed(0.001)
                    .range(0.001..=0.1),
            );
            ui.label("継続[s]");
            ui.add(
                egui::DragValue::new(&mut self.core.analysis_cfg.lumped_th_duration)
                    .speed(0.5)
                    .range(0.1..=60.0),
            );
        });
        ui.horizontal_wrapped(|ui| {
            ui.label("サンプル周期[s]");
            ui.add(
                egui::DragValue::new(&mut self.core.analysis_cfg.lumped_th_period)
                    .speed(0.05)
                    .range(0.05..=5.0),
            );
            ui.label("振幅[mm/s²]");
            ui.add(
                egui::DragValue::new(&mut self.core.analysis_cfg.lumped_th_amp)
                    .speed(50.0)
                    .range(1.0..=20000.0),
            );
        });

        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(!running, egui::Button::new("▶ 固有値を実行"))
                .clicked()
            {
                self.start_lumped_mass_eigen_job();
            }
            if ui
                .add_enabled(!running, egui::Button::new("▶ サンプル波で時刻歴"))
                .clicked()
            {
                self.start_lumped_mass_sample_th_job();
            }
            if self
                .core
                .scoped
                .job
                .as_ref()
                .is_some_and(|j| j.label.starts_with("質点系"))
            {
                ui.spinner();
            }
        });

        ui.horizontal_wrapped(|ui| {
            ui.label("波形ライブラリ:");
            let lib_dir = squid_n_io::wave_library::wave_library_dir();
            let names: Vec<String> = lib_dir
                .as_deref()
                .and_then(|d| squid_n_io::wave_library::list_wave_library(d).ok())
                .unwrap_or_default();
            if names.is_empty() {
                ui.colored_label(crate::theme::GRAY_600, "登録された波形がありません");
            } else {
                let selected_text = self
                    .core
                    .scoped
                    .lumped_wave_library_selection
                    .clone()
                    .unwrap_or_else(|| "(選択してください)".to_string());
                let mut picked = self.core.scoped.lumped_wave_library_selection.clone();
                egui::ComboBox::from_id_salt("lumped_wave_library_select")
                    .selected_text(selected_text)
                    .show_ui(ui, |ui| {
                        for name in &names {
                            ui.selectable_value(&mut picked, Some(name.clone()), name);
                        }
                    });
                self.set_lumped_wave_library_selection(picked);
            }
            if ui
                .add_enabled(
                    !running && self.core.scoped.lumped_wave_library_selection.is_some(),
                    egui::Button::new("▶ 選択波形で時刻歴"),
                )
                .clicked()
            {
                self.start_lumped_mass_library_th_job();
            }
        });
    }
}
