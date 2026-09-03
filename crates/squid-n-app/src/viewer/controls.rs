//! ビューアの操作 UI（表示モード・応力図成分・検定比フィルタ・時刻歴再生・変形表示）。
//!
//! 描画領域の上に積む操作行をここへ集める。どの行も「app の表示設定を読み、
//! ウィジェットへ渡し、書き戻す」だけで、描画そのものには関与しない。
//! 表示モードの追加はこのファイルだけで閉じる。

use crate::app::App;
use crate::theme;

use super::playback::{advance_play_time, frame_at_time};
use super::{check_ratio, lumped, space_grid};
use super::{
    CheckRatioFilter, CmqComponent, ForceComponent, ForceComponents, ModelingAnalysis, ViewMode,
};

/// 描画領域の上に置く操作 UI を描き、変更を `app` へ書き戻す。
pub(super) fn view_controls(ui: &mut egui::Ui, app: &mut App) {
    let mut mode = app.ui.view.view_mode;
    let mut mode_idx = app.ui.scoped.view_mode_idx;
    let mut force_components = app.ui.view.force_components;
    let mut cmq_component = app.ui.view.cmq_component;
    let mut cmq_axes = app.ui.view.cmq_axes;
    let mut check_ratio_filter = app.ui.view.check_ratio_filter;
    let mut modeling_analysis = app.ui.view.modeling_analysis;
    // 時刻歴の詳細記録（`ThRecording`）がある場合のみ「時刻歴」モードを選択肢に出す。
    let has_th_recording = app
        .core
        .scoped
        .results
        .as_ref()
        .and_then(|r| r.time_history.as_ref())
        .and_then(|t| t.recording.as_ref())
        .is_some();

    // --- コントロール ---
    // 中央パネルが狭い場合（左パネルを広げた時など）にボタン列が右パネルへ
    // はみ出さないよう、折り返し可能なレイアウトにする。
    ui.horizontal_wrapped(|ui| {
        ui.label("表示:");
        ui.selectable_value(&mut mode, ViewMode::Shape, "形状");
        ui.selectable_value(&mut mode, ViewMode::Deformed, "変形");
        ui.selectable_value(&mut mode, ViewMode::Mode, "モード");
        ui.selectable_value(&mut mode, ViewMode::Force, "応力図");
        ui.selectable_value(&mut mode, ViewMode::Cmq, "CMQ図");
        ui.selectable_value(&mut mode, ViewMode::CheckRatio, "検定比");
        ui.selectable_value(&mut mode, ViewMode::Hinge, "ヒンジ");
        ui.selectable_value(&mut mode, ViewMode::Modeling, "モデル化");
        if has_th_recording {
            ui.selectable_value(&mut mode, ViewMode::TimeHistory, "時刻歴");
        }
        if lumped::has_lumped(app) {
            ui.selectable_value(&mut mode, ViewMode::LumpedMode, "質点モード");
            ui.selectable_value(&mut mode, ViewMode::LumpedTimeHistory, "質点時刻歴");
        }
        ui.separator();
        // 断面表示: 部材を断面形状の押し出しソリッドで立体表示（全モードと併用可）
        ui.toggle_value(&mut app.ui.view.show_sections, "断面表示");
        // 床（スラブ・小梁）・壁版・二次部材の表示切替（全モードと併用可。
        // CMQ 図は主架構の図のため設定によらず常に非表示）
        ui.toggle_value(&mut app.ui.view.show_floor_secondary, "床壁・二次部材")
            .on_hover_text(
                "床板・小梁・間柱と、解析要素にならない壁版（腰壁・垂壁・パラペット・\
                 自立壁・間柱で分割された壁）の表示",
            );
        // 支点記号。質点ビューでは立体の柱脚拘束は関係ないので選択肢自体を出さない。
        if !lumped::is_lumped_view(mode) {
            ui.toggle_value(&mut app.ui.view.show_supports, "支点")
                .on_hover_text("拘束された節点の矢印・円弧、支点ばね、免震マーカー");
        }
        // 剛床代表点（重心マスター）の表示切替。剛床がある場合のみ選択肢を出す。
        // ON にすると代表点マーカー・面内拘束マーク・スレーブへの点線を描く。
        let has_diaphragm_constraint = app
            .core
            .model
            .constraints
            .iter()
            .any(|c| matches!(c, squid_n_core::model::Constraint::RigidDiaphragm { .. }));
        if has_diaphragm_constraint {
            ui.toggle_value(&mut app.ui.view.show_diaphragm_master, "剛床代表点");
        }
        // 立体グリッド（通り芯 × 階レベル）の表示切替。通り芯と階の両方がある
        // モデルでしか格子を作れないため、そのときだけ選択肢を出す。
        if space_grid::has_grid(&app.core.model) {
            ui.toggle_value(&mut app.ui.view.show_space_grid, "通り芯グリッド")
                .on_hover_text(
                    "各階レベルに通り芯の平面格子を描きます。\
                     梁作成モードでは格子点にスナップし、節点が無ければ梁とあわせて作ります",
                );
        }
        ui.separator();
        // §3-2 の操作規約をヒント表示（左ドラッグ=回転／スクロール=ズーム）
        ui.add_enabled(
            false,
            egui::Label::new(
                egui::RichText::new("左ドラッグ:回転 / 右ドラッグ:移動 / スクロール:ズーム")
                    .size(11.0),
            ),
        );
    });
    if mode == ViewMode::Cmq {
        ui.horizontal_wrapped(|ui| {
            ui.label("成分:");
            ui.selectable_value(&mut cmq_component, CmqComponent::C, "C(モーメント)");
            ui.selectable_value(&mut cmq_component, CmqComponent::M, "M(中央)");
            ui.selectable_value(&mut cmq_component, CmqComponent::Q, "Q(せん断)");
            ui.separator();
            // 応力図の強軸(ey)/弱軸(ez)と同じ面の区別（`ForceComponent::plane`）。
            // 直交グリッド・ひねりのない部材では弱軸成分はほぼ0になるため既定は強軸のみ。
            ui.label("軸:");
            ui.checkbox(&mut cmq_axes.ey, "強軸(ey)");
            ui.checkbox(&mut cmq_axes.ez, "弱軸(ez)");
            ui.separator();
            // ケース切替は追加せず、ナビゲータ／荷重タブで選択中のケース
            // （`nav.focus_load_case`）をそのまま表示する。ここは現在値の案内のみ。
            let case_label = app
                .cmq_display_load_case()
                .map(|lc| lc.name.clone())
                .unwrap_or_else(|| "(荷重ケースなし)".to_string());
            ui.label(format!("荷重ケース: {case_label}（ナビゲータで切替）"));
        });
    }
    // モデル化図: 可視化する解析種別（静解析＝弾性／増分解析＝弾塑性）を切り替える。
    // 静解析は断面の降伏を考えないため全部材が弾性、増分解析は降伏を考慮するため
    // ファイバー要素と材端集中塑性を使い分ける、という違いを見比べられる。
    if mode == ViewMode::Modeling {
        ui.horizontal_wrapped(|ui| {
            ui.label("解析種別:");
            ui.selectable_value(
                &mut modeling_analysis,
                ModelingAnalysis::Static,
                "静解析(弾性)",
            );
            ui.selectable_value(
                &mut modeling_analysis,
                ModelingAnalysis::Incremental,
                "増分解析(弾塑性)",
            );
            ui.separator();
            ui.add_enabled(
                false,
                egui::Label::new(
                    egui::RichText::new("部材の色＝解析上の要素モデル。○=端部ピン／□=半剛")
                        .size(11.0),
                ),
            );
        });
    }
    // 応力図: 6 成分（N/Qy/Qz/Mx/My/Mz）をチェックボックスで個別に ON/OFF し、
    // 選んだ成分をすべて同時に描く。よく使う組はプリセットボタンで切り替える。
    // 単色塗り／コンター（値に応じた色分け）と数値ラベルの表示もここで切替える。
    // コンター ON 時のみカラーマップ選択（既定 Viridis。TONMANUAL §3）を表示する。
    if mode == ViewMode::Force {
        ui.horizontal_wrapped(|ui| {
            ui.label("成分:");
            for c in ForceComponent::ALL {
                // ラベルを成分固定色で描き、図・凡例・数値ラベルの色と対応づける。
                ui.checkbox(
                    force_components.flag_mut(c),
                    egui::RichText::new(c.label()).color(c.color()),
                );
            }
            ui.separator();
            if ui.button("N図").clicked() {
                force_components = ForceComponents::PRESET_N;
            }
            if ui.button("Q図").clicked() {
                force_components = ForceComponents::PRESET_Q;
            }
            if ui.button("M図").clicked() {
                force_components = ForceComponents::PRESET_M;
            }
        });
        ui.horizontal_wrapped(|ui| {
            // 応力図に変形図を重ねる（変位は自動倍率で節点座標に加味され、
            // 図も変形後の材軸に沿って描かれる）
            ui.toggle_value(&mut app.ui.view.overlay_deform, "変形表示");
            ui.toggle_value(&mut app.ui.view.diagram_contour, "コンター");
            if app.ui.view.diagram_contour {
                let mut colormap = app.ui.view.contour_colormap;
                egui::ComboBox::from_id_salt("contour_colormap")
                    .selected_text(colormap.label())
                    .show_ui(ui, |ui| {
                        for cm in [
                            theme::ColorMap::Viridis,
                            theme::ColorMap::Plasma,
                            theme::ColorMap::Turbo,
                            theme::ColorMap::Jet,
                            theme::ColorMap::BlueWhiteRed,
                        ] {
                            ui.selectable_value(&mut colormap, cm, cm.label());
                        }
                    });
                app.ui.view.contour_colormap = colormap;
            }
            ui.toggle_value(&mut app.ui.view.diagram_values, "値を表示")
                .on_hover_text(
                    "各部材の両端部と中央（ξ=0・0.5・1.0）の値を kN・kN·m で表示します\
                     （その成分の最大値の 1% 未満は表示しません）。",
                );
        });
    }
    // 検定比図: 検定式フィルタ（最大／式別、結果に現れる式のみ選択肢に出す）と
    // 位置別マーカーの表示切替。
    if mode == ViewMode::CheckRatio {
        fn checked_components(
            outcome: &squid_n_design_jp::CheckOutcome,
        ) -> Option<&[squid_n_design_jp::CheckComponent]> {
            match outcome {
                squid_n_design_jp::CheckOutcome::Checked(cr) => Some(cr.components.as_slice()),
                squid_n_design_jp::CheckOutcome::Skipped { .. } => None,
            }
        }
        let available_kinds = app
            .core
            .scoped
            .results
            .as_ref()
            .map(|r| {
                check_ratio::available_check_kinds(
                    r.member_checks
                        .iter()
                        .flat_map(|m| m.positions.iter())
                        .filter_map(|p| checked_components(&p.outcome))
                        .chain(
                            r.joint_checks
                                .iter()
                                .filter_map(|j| checked_components(&j.outcome)),
                        ),
                )
            })
            .unwrap_or_default();
        ui.horizontal_wrapped(|ui| {
            ui.label("検定式:");
            ui.selectable_value(&mut check_ratio_filter, CheckRatioFilter::Max, "最大");
            for k in &available_kinds {
                ui.selectable_value(
                    &mut check_ratio_filter,
                    CheckRatioFilter::Kind(*k),
                    k.label(),
                );
            }
            ui.separator();
            ui.checkbox(&mut app.ui.view.check_ratio_markers, "位置別マーカー");
            ui.checkbox(&mut app.ui.view.check_ratio_label_all, "全部材に数値ラベル")
                .on_hover_text(
                    "既定では検定比 0.8 以上の部材にのみ数値ラベルを表示し、\
                     それ未満は色の濃淡（グラデーション）で余裕度を示します。",
                );
        });
    }
    if mode == ViewMode::Mode {
        let n_modes = app
            .core
            .scoped
            .results
            .as_ref()
            .and_then(|r| r.modal.as_ref())
            .map(|m| m.period.len())
            .unwrap_or(0);
        if n_modes > 0 {
            ui.horizontal(|ui| {
                ui.label("モード:");
                let mut idx = mode_idx.min(n_modes - 1);
                ui.add(egui::Slider::new(&mut idx, 0..=n_modes - 1).text(""));
                mode_idx = idx;
                if let Some(t) = app
                    .core
                    .scoped
                    .results
                    .as_ref()
                    .and_then(|r| r.modal.as_ref())
                    .and_then(|m| m.period.get(idx))
                {
                    ui.label(format!("T={:.3} s", t));
                }
            });
        }
    }
    if mode == ViewMode::LumpedMode {
        let n_modes = app
            .core
            .scoped
            .results
            .as_ref()
            .and_then(|r| r.lumped.as_ref())
            .map(|m| m.modal.period.len())
            .unwrap_or(0);
        if n_modes > 0 {
            ui.horizontal(|ui| {
                ui.label("モード:");
                let mut idx = mode_idx.min(n_modes - 1);
                ui.add(egui::Slider::new(&mut idx, 0..=n_modes - 1).text(""));
                mode_idx = idx;
                if let Some(t) = app
                    .core
                    .scoped
                    .results
                    .as_ref()
                    .and_then(|r| r.lumped.as_ref())
                    .and_then(|m| m.modal.period.get(idx))
                {
                    ui.label(format!("T={:.3} s", t));
                }
            });
        }
    }
    if lumped::is_lumped_view(mode) {
        ui.horizontal(|ui| {
            ui.checkbox(&mut app.ui.view.lumped_show_frame, "骨組を重ねる");
        });
    }
    // 時刻歴モード: フレームスライダー・再生制御（§実装内容1）。
    // 現在フレームは `app.ui.scoped.th_frame`、再生経過時刻は `app.ui.scoped.th_play_time`
    // （`frame_time` に基づき現在フレームへ写像。末尾でループ）で管理する。
    if mode == ViewMode::TimeHistory {
        if app.core.scoped.staleness.results_stale {
            // 中-1(a): モデル編集後は添字ずれ（部材削除・並び替え）で別部材のデータを
            // 表示する恐れがあるため、再解析するまで変形アニメーション・部材クリックを
            // 無効化する（フレームスライダー自体も表示しない）。
            ui.colored_label(
                theme::WARN_TEXT,
                "⚠ モデルが編集されています。解析を再実行してください\
                 （変形アニメーション・部材クリックは無効化しています）。",
            );
        } else if let Some(recording) = app
            .core
            .scoped
            .results
            .as_ref()
            .and_then(|r| r.time_history.as_ref())
            .and_then(|t| t.recording.as_ref())
        {
            let n_frames = recording.frame_time.len();
            if n_frames > 0 {
                let duration = recording.frame_time.last().copied().unwrap_or(0.0);
                app.ui.scoped.th_frame = app.ui.scoped.th_frame.min(n_frames - 1);
                ui.horizontal_wrapped(|ui| {
                    if ui
                        .button(if app.ui.scoped.th_playing {
                            "⏸"
                        } else {
                            "▶"
                        })
                        .on_hover_text("再生 / 一時停止")
                        .clicked()
                    {
                        app.ui.scoped.th_playing = !app.ui.scoped.th_playing;
                    }
                    ui.label("速度:");
                    for s in [0.25_f32, 0.5, 1.0, 2.0] {
                        ui.selectable_value(&mut app.ui.view.th_speed, s, format!("×{s}"));
                    }
                    ui.separator();
                    let mut frame = app.ui.scoped.th_frame;
                    if ui
                        .add(egui::Slider::new(&mut frame, 0..=n_frames - 1).text(""))
                        .changed()
                    {
                        app.ui.scoped.th_frame = frame;
                        app.ui.scoped.th_play_time = recording.frame_time[frame];
                    }
                    let t = recording.frame_time[app.ui.scoped.th_frame];
                    ui.label(format!("t={:.2}s / {:.2}s", t, duration));
                });
                // 再生中は実時間×速度でフレームを進め、連続描画のため毎フレーム再描画を要求する。
                if app.ui.scoped.th_playing {
                    let dt = ui.input(|i| i.stable_dt);
                    app.ui.scoped.th_play_time = advance_play_time(
                        app.ui.scoped.th_play_time,
                        dt,
                        app.ui.view.th_speed,
                        duration,
                    );
                    app.ui.scoped.th_frame =
                        frame_at_time(&recording.frame_time, app.ui.scoped.th_play_time);
                    ui.ctx().request_repaint();
                }
            } else {
                ui.label("時刻歴の記録フレームがありません。");
            }
        } else {
            ui.label("時刻歴の詳細記録がありません（再解析すると記録されます）。");
        }
    }
    if mode == ViewMode::LumpedTimeHistory {
        if let Some(th) = app
            .core
            .scoped
            .results
            .as_ref()
            .and_then(|r| r.lumped.as_ref())
            .and_then(|l| l.response.as_ref())
        {
            let n_frames = th.time.len();
            if n_frames > 0 {
                let duration = th.time.last().copied().unwrap_or(0.0);
                app.ui.scoped.th_frame = app.ui.scoped.th_frame.min(n_frames - 1);
                ui.horizontal_wrapped(|ui| {
                    if ui
                        .button(if app.ui.scoped.th_playing {
                            "⏸"
                        } else {
                            "▶"
                        })
                        .clicked()
                    {
                        app.ui.scoped.th_playing = !app.ui.scoped.th_playing;
                    }
                    ui.label("速度:");
                    for s in [0.25_f32, 0.5, 1.0, 2.0] {
                        ui.selectable_value(&mut app.ui.view.th_speed, s, format!("×{s}"));
                    }
                    let mut frame = app.ui.scoped.th_frame;
                    if ui
                        .add(egui::Slider::new(&mut frame, 0..=n_frames - 1).text(""))
                        .changed()
                    {
                        app.ui.scoped.th_frame = frame;
                        app.ui.scoped.th_play_time = th.time.get(frame).copied().unwrap_or(0.0);
                        app.ui.scoped.th_playing = false;
                    }
                    ui.label(format!("t={:.3} s", app.ui.scoped.th_play_time));
                });
                if app.ui.scoped.th_playing {
                    let dt = ui.input(|i| i.stable_dt);
                    app.ui.scoped.th_play_time = advance_play_time(
                        app.ui.scoped.th_play_time,
                        dt,
                        app.ui.view.th_speed,
                        duration,
                    );
                    app.ui.scoped.th_frame = frame_at_time(&th.time, app.ui.scoped.th_play_time);
                    ui.ctx().request_repaint();
                }
            }
        } else {
            ui.colored_label(theme::GRAY_600, "質点系時刻歴の結果がありません。");
        }
    }
    // 変形表示オプション行: 変形を表示するモード（変形・モード・応力図の変形重ね）で
    // 表示する。「内部たわみ」トグルで梁の Hermite 曲線表示（＋床・二次部材の曲線
    // 追従）と直線表示（全体の変形）を切り替え、変形倍率スライダーで自動算定倍率への
    // 手動係数を対数調整（「リセット」で 1.0）する。
    let show_deform_options = matches!(
        mode,
        ViewMode::Deformed
            | ViewMode::Mode
            | ViewMode::TimeHistory
            | ViewMode::LumpedMode
            | ViewMode::LumpedTimeHistory
    ) || (mode == ViewMode::Force && app.ui.view.overlay_deform);
    if show_deform_options {
        ui.horizontal(|ui| {
            ui.toggle_value(&mut app.ui.view.show_beam_interpolation, "内部たわみ")
                .on_hover_text(
                    "梁を内部たわみ（Hermite 曲線）で描き、床・二次部材も曲線に追従。\
                     OFF で梁を直線（弦）にし全体の変形を見る",
                );
            ui.separator();
            ui.label("変形倍率:");
            ui.add(
                egui::Slider::new(&mut app.ui.view.deform_scale_factor, 0.1..=10.0)
                    .logarithmic(true)
                    .text("×（自動比）"),
            );
            if ui.button("リセット").clicked() {
                app.ui.view.deform_scale_factor = 1.0;
            }
        });
    }

    app.ui.view.view_mode = mode;
    app.ui.scoped.view_mode_idx = mode_idx;
    app.ui.view.force_components = force_components;
    app.ui.view.cmq_component = cmq_component;
    app.ui.view.cmq_axes = cmq_axes;
    app.ui.view.check_ratio_filter = check_ratio_filter;
    app.ui.view.modeling_analysis = modeling_analysis;
}
