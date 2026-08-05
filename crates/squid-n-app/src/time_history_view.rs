use crate::app::App;
use crate::story_response::{
    floor_points, hover_story_index, story_axis_label, story_step_points, StoryRespDir,
    StoryResponseKind,
};

/// 時刻歴グラフの描画データ。`App::run_time_history` が
/// ソルバーの `ResponseResult.history` から充填する。
#[derive(Clone, Default)]
pub struct TimeHistoryData {
    pub time: Vec<f64>,
    /// 記録節点の X 方向相対変位 [mm]
    pub node_disp: Vec<f64>,
    /// ベースシア(X) [N]
    pub story_shear: Vec<f64>,
    /// 最上階の層間変形角 [rad]
    pub story_drift_angle: Vec<f64>,
    /// 記録節点
    pub node: Option<squid_n_core::ids::NodeId>,
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum TimeHistorySource {
    #[default]
    NodeDisp,
    StoryShear,
    StoryDriftAngle,
}

/// 結果タブ「時刻歴」の表示モード（波形／層応答分布を排他切替）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum TimeHistoryViewMode {
    #[default]
    Waveform,
    StoryResponse,
}

pub fn time_history_panel(ui: &mut egui::Ui, app: &mut App) {
    ui.horizontal(|ui| {
        ui.selectable_value(
            &mut app.time_history_view_mode,
            TimeHistoryViewMode::Waveform,
            "時刻歴波形",
        );
        ui.selectable_value(
            &mut app.time_history_view_mode,
            TimeHistoryViewMode::StoryResponse,
            "層応答分布",
        );
    });
    ui.separator();
    match app.time_history_view_mode {
        TimeHistoryViewMode::Waveform => waveform_panel(ui, app),
        TimeHistoryViewMode::StoryResponse => story_response_panel(ui, app),
    }
}

/// 時刻歴波形（代表応答: 節点変位／ベースシア／層間変形角）。従来の
/// `time_history_panel` の本体（表示モード追加に伴い切り出し）。
fn waveform_panel(ui: &mut egui::Ui, app: &mut App) {
    if app.time_history_data.time.is_empty() {
        ui.colored_label(
            crate::theme::GRAY_600,
            "時刻歴応答データがありません。解析タブの「時刻歴」から実行してください。",
        );
        return;
    }

    let mut source = app.time_history_source;

    ui.horizontal(|ui| {
        ui.label("表示項目:");
        let node_label = app
            .time_history_data
            .node
            .map(|n| format!("節点 N{} 変位", n.0))
            .unwrap_or_else(|| "節点変位".to_string());
        ui.selectable_value(&mut source, TimeHistorySource::NodeDisp, node_label);
        ui.selectable_value(&mut source, TimeHistorySource::StoryShear, "ベースシア");
        ui.selectable_value(
            &mut source,
            TimeHistorySource::StoryDriftAngle,
            "層間変形角(最上階)",
        );
    });

    ui.add_space(4.0);

    if source != app.time_history_source {
        app.time_history_source = source;
    }

    let data = &app.time_history_data;
    let series = match source {
        TimeHistorySource::NodeDisp => &data.node_disp,
        TimeHistorySource::StoryShear => &data.story_shear,
        TimeHistorySource::StoryDriftAngle => &data.story_drift_angle,
    };
    let values: Vec<[f64; 2]> = data
        .time
        .iter()
        .zip(series.iter())
        .map(|(&t, &v)| [t, v])
        .collect();

    // §3 データビジュアライゼーション配色（系列ごとに弁別可能な 3 色）
    let (ylabel, line_color) = match source {
        TimeHistorySource::NodeDisp => ("変位 [mm]", crate::theme::DATA_BLUE),
        TimeHistorySource::StoryShear => ("ベースシア [N]", crate::theme::PARETO_RED),
        TimeHistorySource::StoryDriftAngle => ("層間変形角 [rad]", crate::theme::GOOD_GREEN),
    };

    // ピーク値サマリ
    let peak = series.iter().cloned().fold(0.0f64, |m, v| m.max(v.abs()));
    ui.label(format!("最大絶対値: {:.4e}", peak));

    // レインフロー計数（累積損傷度計算で用いる ASTM E1049 3 点法）。表示中の代表応答に対する
    // 等価繰返し数・最大振れ幅を参考表示する（累積損傷度 D の梁端 μ 収集は今後の拡張）。
    let cycles = squid_n_solver::damage::rainflow_cycles(series);
    let neq: f64 = cycles.iter().map(|c| c.count).sum();
    let max_range = cycles.iter().map(|c| c.range).fold(0.0f64, f64::max);
    ui.label(format!(
        "レインフロー(代表応答): 等価繰返し数 {:.1} 回 / 最大振れ幅 {:.4e}",
        neq, max_range
    ))
    .on_hover_text("累積損傷度計算(レインフロー法)の基礎計数（ASTM E1049 3 点法）。");

    // 梁端累積損傷度 D（鉄骨梁端部の累積損傷度計算）。非線形時刻歴で
    // 各要素の危険断面塑性率 μ 時刻歴からレインフロー法で算定した値を表示する。
    if let Some(res) = app.results.as_ref().and_then(|r| r.time_history.as_ref()) {
        let dmax = res
            .cumulative_ductility
            .iter()
            .cloned()
            .fold(0.0f64, f64::max);
        let n_damaged = res
            .cumulative_ductility
            .iter()
            .filter(|&&d| d > 0.0)
            .count();
        if dmax > 0.0 {
            // 最大 D の要素 ID。
            let imax = res
                .cumulative_ductility
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(i, _)| i)
                .unwrap_or(0);
            ui.label(format!(
                "梁端累積損傷度 D: 最大 {:.3}（部材 {}） / 損傷要素 {} 件（レインフロー法）",
                dmax, imax, n_damaged
            ))
            .on_hover_text(
                "非線形時刻歴で塑性化した要素の危険断面塑性率 μ の時刻歴から算定。\
                 D≥1 で疲労破断（疲労特性 C・β は暫定既定、鋼種・接合形式で要照合）。",
            );
        } else {
            ui.colored_label(
                crate::theme::GRAY_600,
                "梁端累積損傷度 D: 塑性化要素なし（非線形時刻歴で塑性率を収集）。",
            );
        }
    }

    let plot = egui_plot::Plot::new("time_history_plot")
        .legend(egui_plot::Legend::default())
        .x_axis_label("時間 [s]")
        .y_axis_label(ylabel)
        .show(ui, |plot_ui| {
            plot_ui.line(
                egui_plot::Line::new("series", egui_plot::PlotPoints::from(values))
                    .color(line_color)
                    .width(1.5_f32),
            );
        });

    // カーソル位置の値を表示
    if let Some(pointer) = plot.response.hover_pos() {
        let pointer_value = plot.transform.value_from_position(pointer);
        let dt = if data.time.len() >= 2 {
            (data.time[data.time.len() - 1] - data.time[0]) / (data.time.len() - 1) as f64
        } else {
            1.0
        };
        let idx = ((pointer_value.x - data.time[0]) / dt).round().max(0.0) as usize;
        if idx < data.time.len() && idx < series.len() {
            let t = data.time[idx];
            let val = series[idx];
            ui.horizontal(|ui| {
                ui.label(format!("t = {:.3} s", t));
                ui.separator();
                ui.label(match source {
                    TimeHistorySource::NodeDisp => format!("変位 = {:.3} mm", val),
                    TimeHistorySource::StoryShear => format!("せん断 = {:.3} N", val),
                    TimeHistorySource::StoryDriftAngle => format!("変形角 = {:.6} rad", val),
                });
            });
        }
    }
}

/// 層応答分布（縦軸=階、横軸=層せん断力／層せん断力係数／階加速度／階速度／階変位）。
/// データは `App.results.time_history.recording`（`ThRecording`）から直接参照する
/// （コピー保持しない）。`recording` がない（旧い結果、または非線形以前の解析結果でも
/// ないことはないが念のため）場合は再解析を案内する。
fn story_response_panel(ui: &mut egui::Ui, app: &mut App) {
    let Some(recording) = app
        .results
        .as_ref()
        .and_then(|r| r.time_history.as_ref())
        .and_then(|th| th.recording.as_ref())
    else {
        ui.colored_label(
            crate::theme::GRAY_600,
            "層応答の詳細記録がありません。時刻歴応答を再実行してください\
             （この記録を持たない旧い結果、または未実行）。",
        );
        return;
    };

    let mut dir = app.story_response_dir;
    let mut kind = app.story_response_kind;

    ui.horizontal_wrapped(|ui| {
        ui.label("方向:");
        ui.selectable_value(&mut dir, StoryRespDir::X, "X");
        ui.selectable_value(&mut dir, StoryRespDir::Y, "Y");
        ui.separator();
        ui.label("項目:");
        ui.selectable_value(&mut kind, StoryResponseKind::Shear, "層せん断力");
        ui.selectable_value(&mut kind, StoryResponseKind::ShearCoeff, "層せん断力係数");
        ui.selectable_value(&mut kind, StoryResponseKind::Accel, "階加速度");
        ui.selectable_value(&mut kind, StoryResponseKind::Vel, "階速度");
        ui.selectable_value(&mut kind, StoryResponseKind::Disp, "階変位");
    });
    app.story_response_dir = dir;
    app.story_response_kind = kind;

    let story = match dir {
        StoryRespDir::X => &recording.story_x,
        StoryRespDir::Y => &recording.story_y,
    };
    let n_story = story.stories.len();
    if n_story == 0 {
        ui.colored_label(
            crate::theme::GRAY_600,
            "階情報がありません（階の自動生成が未実行の可能性があります）。",
        );
        return;
    }

    // 階名（低: 添字ではなく `StoryId` で現モデルと突き合わせる。解析後にモデルの
    // 階が編集されても別の階の名前を誤って表示しない。見つからなければ「(削除済み階)」）。
    let model_story_names: Vec<(squid_n_core::ids::StoryId, String)> = app
        .model
        .stories
        .iter()
        .map(|s| (s.id, s.name.clone()))
        .collect();
    let story_names =
        crate::story_response::story_display_names(&model_story_names, &story.stories);

    /// 層応答分布の1系列分の表示仕様（値列・軸ラベル・単位・値の書式）。
    ///
    /// いずれも `StoryResponse::peak_*`（全ステップ更新・間引きなしのピーク）を使う
    /// （中-4）。フレーム記録（`story_shear`等、`record_every` で間引き）から
    /// `story_absmax` で求めると、間引きの合間に生じたピークを取りこぼすため。
    type SeriesSpec<'a> = (Vec<f64>, &'a str, &'a str, fn(f64) -> String);
    let (values, xlabel, unit_suffix, value_fmt): SeriesSpec<'_> = match kind {
        StoryResponseKind::Shear => (
            story
                .peak_story_shear
                .iter()
                .map(|&v| crate::story_response::n_to_kn(v))
                .collect(),
            "層せん断力 [kN]",
            "kN",
            |v| format!("{:.2}", v),
        ),
        StoryResponseKind::ShearCoeff => (
            story.peak_shear_coeff.clone(),
            "層せん断力係数 Ci [-]",
            "",
            |v| format!("{:.4}", v),
        ),
        StoryResponseKind::Accel => (
            story
                .peak_floor_accel
                .iter()
                .map(|&v| crate::story_response::mm_s2_to_gal(v))
                .collect(),
            "階加速度 [gal]",
            "gal",
            |v| format!("{:.1}", v),
        ),
        StoryResponseKind::Vel => (
            story
                .peak_floor_vel
                .iter()
                .map(|&v| crate::story_response::mm_s_to_m_s(v))
                .collect(),
            "階速度 [m/s]",
            "m/s",
            |v| format!("{:.4}", v),
        ),
        StoryResponseKind::Disp => (story.peak_floor_disp.clone(), "階変位 [mm]", "mm", |v| {
            format!("{:.2}", v)
        }),
    };

    let is_story_quantity = kind.is_story_quantity();
    let color = crate::theme::DATA_BLUE;
    let names_for_axis = story_names.clone();

    let plot = egui_plot::Plot::new("story_response_plot")
        .x_axis_label(xlabel)
        .y_axis_label("階")
        .y_axis_formatter(move |mark, _range| story_axis_label(&names_for_axis, mark.value))
        .show(ui, |plot_ui| {
            if is_story_quantity {
                let pts = story_step_points(&values);
                plot_ui.line(
                    egui_plot::Line::new("層応答", egui_plot::PlotPoints::from(pts))
                        .color(color)
                        .width(2.0_f32),
                );
            } else {
                let pts = floor_points(&values);
                plot_ui.line(
                    egui_plot::Line::new("階応答", egui_plot::PlotPoints::from(pts.clone()))
                        .color(color)
                        .width(1.5_f32),
                );
                plot_ui.points(
                    egui_plot::Points::new("階応答", egui_plot::PlotPoints::from(pts))
                        .color(color)
                        .radius(3.0_f32)
                        .shape(egui_plot::MarkerShape::Circle),
                );
            }
        });

    // カーソル位置の値を表示（既存の時刻歴波形グラフと同じ方式）。
    if let Some(pointer) = plot.response.hover_pos() {
        let pointer_value = plot.transform.value_from_position(pointer);
        let idx = hover_story_index(pointer_value.y, n_story, is_story_quantity);
        let name = story_names.get(idx).cloned().unwrap_or_default();
        let val = values.get(idx).copied().unwrap_or(0.0);
        let suffix = if unit_suffix.is_empty() {
            String::new()
        } else {
            format!(" {}", unit_suffix)
        };
        ui.horizontal(|ui| {
            ui.label(format!("階: {}", name));
            ui.separator();
            ui.label(format!("値 = {}{}", value_fmt(val), suffix));
        });
    }
}
