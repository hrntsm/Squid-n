//! 時刻歴モード（[`super::ViewMode::TimeHistory`]）で部材をクリックした際に開く
//! 詳細ウィンドウ。
//!
//! - 荷重変形関係の履歴ループ（egui_plot）: 軸力系要素（ブレース・ダンパー・
//!   免震・節点ばね）は軸力 N - 軸方向相対変位 δ（材軸方向）、梁・柱は材端モーメント
//!   M - 材端回転角 θ（弦からのたわみ角、「節点端モーメント」規約に統一）。
//!   部材長がほぼ 0 の要素（免震・節点ばね）は材軸が定まらないため、要素ローカル軸の
//!   成分別（軸／せん断y／せん断z）表示に切り替わる（免震は水平せん断が既定）。
//!   現在フレームの位置をループ上にマーカー表示し、フレームスライダーと連動する。
//! - 最大応力に対する検定: `ThRecording::peak_member_forces`（全ステップの内力
//!   包絡）を用いて短期の断面検定を実行する。
//!
//! ヒンジ詳細ウィンドウ（[`super::hinge`]）と同じ構成方針（`egui::Window` +
//! `egui_plot`）を踏襲するが、対象データ（時刻歴の全フレーム記録）が異なるため
//! 実装は独立させている。
//!
//! 検定の組み立ては `app::actions::run_design_check`（断面・材料に応じた
//! RC/Steel/SRC/CFT の `DesignCheck` 実装への振り分け）と同じ考え方だが、
//! 当該関数は `app` モジュール内の非公開関数（`is_steel` 等）に
//! 依存しビューア側からは呼べないため、必要な部分だけ本ファイルへ複製する。
//! 長期静的結果がある場合は地震時 QD / 柱メカニズムを部分配線する。
//! 座屈長さの自動算定・鋼継手欠損・一本部材グループ合成・BRB 属性差し替え・
//! Q0（単純梁せん断）は簡略化のため含まない（概算検定）。

use crate::app::App;
use crate::theme;
use squid_n_core::geom::vec3::dist as member_len3;
use squid_n_core::ids::ElemId;
use squid_n_core::model::{ElementData, ElementKind, Model};
use squid_n_core::units::to_display::{force_kn, moment_kn_m};
use squid_n_design_jp::{
    CheckOutcome, DesignCheck, DesignCtx, LoadTerm, MemberForcesAt, MemberKind,
};
use squid_n_element::frame::beam::MemberForces;
use squid_n_element::transform::LocalFrame;
use squid_n_solver::dynamic::timehistory::ThRecording;

/// ループ表示の分類（要素種別による）。
#[derive(Clone, Copy, PartialEq, Eq)]
enum LoopKind {
    /// 軸力系（ブレース・ダンパー・免震・節点ばね）: N - δ
    Axial,
    /// 梁・柱（曲げ材）: M - θ
    Flexural,
    /// 上記以外（壁・シェル・パネルゾーン等）: ループ表示非対応
    Unsupported,
}

fn loop_kind_of(kind: &ElementKind) -> LoopKind {
    match kind {
        ElementKind::Brace { .. }
        | ElementKind::Damper
        | ElementKind::Isolator
        | ElementKind::NodalSpring => LoopKind::Axial,
        ElementKind::Beam | ElementKind::Fiber | ElementKind::MultiSpring => LoopKind::Flexural,
        ElementKind::Wall | ElementKind::Shell | ElementKind::PanelZone => LoopKind::Unsupported,
    }
}

/// 要素種別の日本語表示名。
fn kind_label(kind: &ElementKind) -> &'static str {
    match kind {
        ElementKind::Beam => "梁・柱",
        ElementKind::Fiber => "ファイバー梁",
        ElementKind::MultiSpring => "マルチスプリング梁",
        ElementKind::Wall => "耐震壁",
        ElementKind::PanelZone => "パネルゾーン",
        ElementKind::Brace { .. } => "ブレース",
        ElementKind::NodalSpring => "節点ばね",
        ElementKind::Isolator => "免震支承",
        ElementKind::Damper => "ダンパー",
        ElementKind::Shell => "シェル",
    }
}

/// 軸力系要素の軸方向相対変位 δ [mm]（引張を正）。
///
/// 両端節点の変位差（並進成分のみ）を、未変形材軸方向（i→j の単位ベクトル）へ
/// 射影する。微小変形の近似（弦回転による二次項は無視）。部材長がほぼ 0 の
/// 要素（免震・節点ばね）は材軸そのものが定まらないため 0 を返す
/// （零長要素は [`zero_length_local_frame`]／[`zero_length_relative_disp`] を使う）。
pub(super) fn axial_relative_disp(
    p_i: [f64; 3],
    p_j: [f64; 3],
    d_i: [f64; 6],
    d_j: [f64; 6],
) -> f64 {
    let len = member_len3(p_i, p_j);
    if len < 1e-9 {
        return 0.0;
    }
    let ex = [
        (p_j[0] - p_i[0]) / len,
        (p_j[1] - p_i[1]) / len,
        (p_j[2] - p_i[2]) / len,
    ];
    let du = [d_j[0] - d_i[0], d_j[1] - d_i[1], d_j[2] - d_i[2]];
    du[0] * ex[0] + du[1] * ex[1] + du[2] * ex[2]
}

/// 3 次元ベクトルの内積。
use squid_n_core::geom::vec3::dot as dot3;

/// 零長要素（部材長 < 1e-9。免震・節点ばね）の局所座標系（中-2）。
///
/// `squid_n_element` の各要素実装は零長時に材軸射影ができないため、要素種別ごとに
/// 固有の既定軸を用いる（`squid_n_element::springs::isolator::IsolatorElement::new`／
/// `squid_n_element::springs::spring::NodalSpringElement::new` と同じ規約を複製）。
/// - **免震支承（[`ElementKind::Isolator`]）**: 局所 x 軸＝鉛直（全体 Z）、
///   局所 y・z 軸＝水平（`ref_vector` から定まる）。
/// - **それ以外（節点ばね等）**: 全体座標系＝局所座標系（単位回転）。
pub(super) fn zero_length_local_frame(
    kind: &ElementKind,
    p_i: [f64; 3],
    ref_vector: [f64; 3],
) -> LocalFrame {
    match kind {
        ElementKind::Isolator => {
            LocalFrame::from_nodes(p_i, [p_i[0], p_i[1], p_i[2] + 1.0], ref_vector)
        }
        _ => LocalFrame {
            rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        },
    }
}

/// 零長要素の局所軸 `component` 方向の相対変位 [mm]（[`zero_length_local_frame`] 参照）。
/// 両端節点の変位差（並進成分のみ）を局所軸へ射影する（純粋関数）。
pub(super) fn zero_length_relative_disp(
    kind: &ElementKind,
    p_i: [f64; 3],
    ref_vector: [f64; 3],
    d_i: [f64; 6],
    d_j: [f64; 6],
    component: AxialComponent,
) -> f64 {
    let frame = zero_length_local_frame(kind, p_i, ref_vector);
    let du = [d_j[0] - d_i[0], d_j[1] - d_i[1], d_j[2] - d_i[2]];
    dot3(du, frame.rot[component.axis_index()])
}

/// 軸力系要素の N-δ ループで表示する成分（中-2）。
/// `MemberForces::at` の力成分の並び `[N,Qy,Qz,Mx,My,Mz]` の先頭 3 つに対応する。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AxialComponent {
    /// 軸方向（局所 x）。`MemberForces` の N。
    Axial,
    /// せん断 y（局所 y）。`MemberForces` の Qy。
    ShearY,
    /// せん断 z（局所 z）。`MemberForces` の Qz。
    ShearZ,
}

impl AxialComponent {
    /// `MemberForces::at` の力配列 `[N,Qy,Qz,Mx,My,Mz]` の添字（0..3）。
    /// [`zero_length_local_frame`] の局所軸（`rot[0..3]`）の添字とも共通。
    fn axis_index(self) -> usize {
        match self {
            Self::Axial => 0,
            Self::ShearY => 1,
            Self::ShearZ => 2,
        }
    }

    /// プロットの軸ラベル `(横軸=δ, 縦軸=力)`。
    fn axis_labels(self) -> (&'static str, &'static str) {
        match self {
            Self::Axial => ("δ [mm]", "N [kN]"),
            Self::ShearY => ("δy [mm]", "Qy [kN]"),
            Self::ShearZ => ("δz [mm]", "Qz [kN]"),
        }
    }

    /// 成分切替 UI のラベル。
    fn selector_label(self) -> &'static str {
        match self {
            Self::Axial => "軸(N-δ)",
            Self::ShearY => "せん断y(Qy-δy)",
            Self::ShearZ => "せん断z(Qz-δz)",
        }
    }
}

/// 零長要素（免震・節点ばね）の N-δ ループで既定表示する成分（中-2）。
/// 免震支承は水平せん断力 Qy を既定にする（水平ループの確認が主目的のため）。
/// それ以外（節点ばね等）は従来どおり軸方向を既定にする。
pub(super) fn default_axial_component(kind: &ElementKind) -> AxialComponent {
    match kind {
        ElementKind::Isolator => AxialComponent::ShearY,
        _ => AxialComponent::Axial,
    }
}

/// 梁・柱の材端回転角（弦からのたわみ角）[rad] を返す `(ry_i, rz_i, ry_j, rz_j)`。
///
/// `squid-n-solver` の増分解析（`nonlinear::pushover::member_response`）と同じ
/// 算定式: 局所たわみ v（ey 方向）・w（ez 方向）の弦回転を節点回転の局所成分から
/// 差し引く（x-y 面: θz-(v_j-v_i)/L、x-z 面: θy+(w_j-w_i)/L）。剛体回転（層間変形
/// による部材全体の傾き）を除いた「材端の回転」が M-θ 履歴として意味を持つため。
pub(super) fn beam_end_rotations(
    p_i: [f64; 3],
    p_j: [f64; 3],
    ref_vector: [f64; 3],
    d_i: [f64; 6],
    d_j: [f64; 6],
) -> (f64, f64, f64, f64) {
    let length = member_len3(p_i, p_j);
    if length < 1e-9 {
        return (0.0, 0.0, 0.0, 0.0);
    }
    let frame = LocalFrame::from_nodes(p_i, p_j, ref_vector);
    let ey = frame.rot[1];
    let ez = frame.rot[2];
    let u_i = [d_i[0], d_i[1], d_i[2]];
    let u_j = [d_j[0], d_j[1], d_j[2]];
    let r_i = [d_i[3], d_i[4], d_i[5]];
    let r_j = [d_j[3], d_j[4], d_j[5]];
    let chord_v = (dot3(u_j, ey) - dot3(u_i, ey)) / length;
    let chord_w = (dot3(u_j, ez) - dot3(u_i, ez)) / length;
    let ry_i = dot3(r_i, ey) + chord_w;
    let rz_i = dot3(r_i, ez) - chord_v;
    let ry_j = dot3(r_j, ey) + chord_w;
    let rz_j = dot3(r_j, ez) - chord_v;
    (ry_i, rz_i, ry_j, rz_j)
}

/// 当該要素の内力が 1 フレームでも記録されているか（要素種別によっては
/// `state_member_forces`/`recover_forces` が `None` を返し続け、記録が存在しない）。
fn elem_has_force_recording(rec: &ThRecording, elem_idx: usize) -> bool {
    rec.member_forces
        .iter()
        .any(|frame| frame.get(elem_idx).is_some_and(|o| o.is_some()))
}

/// 断面検定の対象になる部材種別か（軸力のみの減衰・免震・節点ばね要素、
/// シェル・パネルゾーンは対象外）。耐震壁は壁柱軸で柱として簡易検定する。
fn design_member_kind(elem: &ElementData, model: &Model) -> Option<MemberKind> {
    match elem.kind {
        ElementKind::Brace { .. } => Some(MemberKind::Brace),
        ElementKind::Beam | ElementKind::Fiber | ElementKind::MultiSpring => {
            Some(MemberKind::of_element(elem, model))
        }
        ElementKind::Wall => super::member_axis_endpoints(elem, model)
            .map(|ep| MemberKind::from_axis(ep.p_i, ep.p_j)),
        _ => None,
    }
}

/// 時刻歴詳細ウィンドウ（`app.ui.scoped.th_detail_elem` があれば表示）。
pub(crate) fn show_th_detail_window(ui: &egui::Ui, app: &mut App) {
    let Some(elem_id) = app.ui.scoped.th_detail_elem else {
        return;
    };
    let mut open = true;
    egui::Window::new(format!("時刻歴詳細: 部材 #{}", elem_id.0))
        .id(egui::Id::new("th_detail_window"))
        .resizable(true)
        .collapsible(true)
        .default_size([460.0, 640.0])
        .open(&mut open)
        .show(ui.ctx(), |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                draw_th_detail_content(ui, app, elem_id);
            });
        });
    if !open {
        app.ui.scoped.th_detail_elem = None;
    }
}

/// `app.ui.view.th_detail_axis_z` は一時的にローカル変数へ取り出し、`ThRecording` の
/// 借用（`app.core.scoped.results` 由来。フレーム数×要素数の内力を持つため複製すると重い）と
/// `app: &mut App` の同時使用を避け、最後にまとめて書き戻す
/// （§実装内容2 のループ本体は `app.core.model`／`app.core.scoped.results` の共有参照のみで完結する）。
fn draw_th_detail_content(ui: &mut egui::Ui, app: &mut App, elem_id: ElemId) {
    // 中-1(b): モデル編集後（他タブの ⚠ 表示と同じ判定条件）は添字ずれにより
    // 別部材のデータを表示する恐れがあるため、プロット・検定を出さず警告のみ表示する。
    if app.core.scoped.staleness.results_stale {
        ui.colored_label(
            theme::WARN_TEXT,
            "⚠ モデルが編集されています。解析を再実行してください\
             （プロット・検定は前回解析時点のままのため非表示にしています）。",
        );
        return;
    }
    let Some(th_result) = app
        .core
        .scoped
        .results
        .as_ref()
        .and_then(|r| r.time_history.as_ref())
    else {
        ui.colored_label(theme::GRAY_600, "時刻歴の詳細記録がありません。");
        return;
    };
    let Some(recording) = th_result.recording.as_ref() else {
        ui.colored_label(theme::GRAY_600, "時刻歴の詳細記録がありません。");
        return;
    };
    // 解析時のフラグ（`ResponseResult::nonlinear`/`applied_long_term`）を、後段の
    // `draw_peak_check`/`draw_long_term_note` へ渡すために先に取り出しておく
    // （借用の都合上、`app: &mut App` の可変借用と `recording` の共有借用を
    // 同時に保持できないため、値だけコピーする）。
    let th_nonlinear = th_result.nonlinear;
    let th_applied_long_term = th_result.applied_long_term;
    let display = super::wall_expanded_view_model(&app.core.model);
    let Some(elem_idx) = display.elements.iter().position(|e| e.id == elem_id) else {
        ui.colored_label(theme::GRAY_600, "この部材はモデルから削除されています。");
        return;
    };
    let elem = &display.elements[elem_idx];
    let Some(axis) = super::member_axis_endpoints(elem, display.as_ref()) else {
        ui.colored_label(theme::GRAY_600, "材軸端点を取得できません。");
        return;
    };
    let (p_i, p_j) = (axis.p_i, axis.p_j);
    let (n0, n1) = (axis.n0, axis.n1);

    ui.label(format!("部材 #{}（{}）", elem_id.0, kind_label(&elem.kind)));
    let n_frames = recording.frame_time.len();
    let frame = app.ui.scoped.th_frame.min(n_frames.saturating_sub(1));
    if let Some(t) = recording.frame_time.get(frame) {
        ui.label(format!(
            "現在フレーム: {frame}/{} (t={:.3}s)",
            n_frames.saturating_sub(1),
            t
        ));
    }
    ui.separator();

    ui.strong("荷重変形関係の履歴ループ");
    let mut axis_z = app.ui.view.th_detail_axis_z;
    // 中-2: 零長要素の成分選択は部材ごとに保持し、部材が変われば要素種別の既定へ戻す。
    let mut axial_component = match app.ui.scoped.th_detail_axial_component {
        Some((id, c)) if id == elem_id => c,
        _ => default_axial_component(&elem.kind),
    };
    match loop_kind_of(&elem.kind) {
        LoopKind::Axial => draw_axial_loop(
            ui,
            recording,
            elem_id,
            elem_idx,
            &elem.kind,
            n0,
            n1,
            p_i,
            p_j,
            elem.local_axis.ref_vector,
            frame,
            &mut axial_component,
        ),
        LoopKind::Flexural => draw_flexural_loop(
            ui,
            recording,
            elem_id,
            elem_idx,
            n0,
            n1,
            p_i,
            p_j,
            elem.local_axis.ref_vector,
            frame,
            &mut axis_z,
        ),
        LoopKind::Unsupported if matches!(elem.kind, ElementKind::Wall) => {
            if elem_has_force_recording(recording, elem_idx) {
                draw_flexural_loop(
                    ui,
                    recording,
                    elem_id,
                    elem_idx,
                    n0,
                    n1,
                    p_i,
                    p_j,
                    elem.local_axis.ref_vector,
                    frame,
                    &mut axis_z,
                );
            } else {
                ui.colored_label(theme::GRAY_600, "この耐震壁の内力は記録されていません。");
            }
        }
        LoopKind::Unsupported => {
            ui.colored_label(
                theme::GRAY_600,
                "この要素種別はループ表示に対応していません。",
            );
        }
    }
    app.ui.view.th_detail_axis_z = axis_z;
    app.ui.scoped.th_detail_axial_component = Some((elem_id, axial_component));

    ui.add_space(6.0);
    ui.separator();
    draw_peak_check(
        ui,
        app,
        elem,
        elem_idx,
        recording,
        th_nonlinear,
        th_applied_long_term,
        &axis,
    );
}

/// 軸力系要素（ブレース・ダンパー・免震・節点ばね）の荷重変形ループ。
///
/// 部材長がほぼ 0 の要素（免震・節点ばね。中-2）は材軸方向が定まらないため、
/// 通常の軸方向 N-δ の代わりに要素ローカル軸の成分別（軸／せん断y／せん断z）
/// 表示に切り替え、成分切替 UI を出す（既定は [`default_axial_component`]。
/// 免震はせん断＝水平ループを既定にする）。`component`（表示中の成分）は
/// 呼び出し側（`App::th_detail_axial_component`）が保持する状態をローカル変数
/// として受け渡す（[`draw_flexural_loop`] の `axis_z` と同じ理由）。
#[allow(clippy::too_many_arguments)]
fn draw_axial_loop(
    ui: &mut egui::Ui,
    rec: &ThRecording,
    elem_id: ElemId,
    elem_idx: usize,
    elem_kind: &ElementKind,
    n0: usize,
    n1: usize,
    p_i: [f64; 3],
    p_j: [f64; 3],
    ref_vector: [f64; 3],
    cur_frame: usize,
    component: &mut AxialComponent,
) {
    if !elem_has_force_recording(rec, elem_idx) {
        ui.colored_label(theme::GRAY_600, "この部材の内力は記録されていません。");
        return;
    }
    let zero_length = member_len3(p_i, p_j) < 1e-9;
    if zero_length {
        ui.horizontal(|ui| {
            ui.label("成分:");
            for c in [
                AxialComponent::ShearY,
                AxialComponent::ShearZ,
                AxialComponent::Axial,
            ] {
                ui.selectable_value(component, c, c.selector_label());
            }
        });
    }
    // 通常長の要素は成分切替UIを出さず、従来どおり軸方向のみ（材軸射影）を表示する。
    let component = if zero_length {
        *component
    } else {
        AxialComponent::Axial
    };

    let point = |f: usize| -> Option<[f64; 2]> {
        axial_point(
            rec, elem_idx, elem_kind, n0, n1, p_i, p_j, ref_vector, component, f,
        )
    };
    let series: Vec<[f64; 2]> = (0..rec.frame_time.len()).filter_map(point).collect();
    let current = point(cur_frame);
    let (xlabel, ylabel) = component.axis_labels();

    egui_plot::Plot::new(format!("th_axial_{}", elem_id.0))
        .x_axis_label(xlabel)
        .y_axis_label(ylabel)
        .height(220.0)
        .show(ui, |plot_ui| {
            plot_ui.line(
                egui_plot::Line::new("荷重変形関係", egui_plot::PlotPoints::from(series))
                    .color(theme::DATA_BLUE)
                    .width(1.5_f32),
            );
            if let Some(p) = current {
                plot_ui.points(
                    egui_plot::Points::new("現在フレーム", egui_plot::PlotPoints::from(vec![p]))
                        .color(theme::PARETO_RED)
                        .radius(5.0_f32)
                        .shape(egui_plot::MarkerShape::Circle),
                );
            }
        });
}

/// フレーム `f` の軸力系 (δ[単位は成分により mm]、力[kN]) を求める。内力・変位の
/// いずれかが欠けていれば `None`。零長要素（`p_i`・`p_j` がほぼ一致）は
/// [`zero_length_relative_disp`]（要素ローカル軸への射影）を、それ以外は
/// [`axial_relative_disp`]（材軸射影、常に軸方向成分）を使う。
#[allow(clippy::too_many_arguments)]
fn axial_point(
    rec: &ThRecording,
    elem_idx: usize,
    elem_kind: &ElementKind,
    n0: usize,
    n1: usize,
    p_i: [f64; 3],
    p_j: [f64; 3],
    ref_vector: [f64; 3],
    component: AxialComponent,
    f: usize,
) -> Option<[f64; 2]> {
    let mf = rec.member_forces.get(f)?.get(elem_idx)?.as_ref()?;
    let force = force_kn(mf.at.first()?.1[component.axis_index()]);
    let disp_frame = rec.node_disp.get(f)?;
    let d_i = *disp_frame.get(n0)?;
    let d_j = *disp_frame.get(n1)?;
    let delta = if member_len3(p_i, p_j) < 1e-9 {
        zero_length_relative_disp(elem_kind, p_i, ref_vector, d_i, d_j, component)
    } else {
        axial_relative_disp(p_i, p_j, d_i, d_j)
    };
    Some([delta, force])
}

/// 梁・柱の M-θ ループ（i端・j端、強軸/弱軸切替可能）。
///
/// `axis_z`（表示中の曲げ軸）は呼び出し側（`App::th_detail_axis_z`）が保持する
/// 状態をローカル変数として受け渡す。`ThRecording` の借用（`app.core.scoped.results` 由来）と
/// `app: &mut App` の同時使用を避けるため、`App` そのものは受け取らない。
#[allow(clippy::too_many_arguments)]
fn draw_flexural_loop(
    ui: &mut egui::Ui,
    rec: &ThRecording,
    elem_id: ElemId,
    elem_idx: usize,
    n0: usize,
    n1: usize,
    p_i: [f64; 3],
    p_j: [f64; 3],
    ref_vector: [f64; 3],
    cur_frame: usize,
    axis_z: &mut bool,
) {
    if !elem_has_force_recording(rec, elem_idx) {
        ui.colored_label(theme::GRAY_600, "この部材の内力は記録されていません。");
        return;
    }
    ui.horizontal(|ui| {
        ui.label("成分:");
        ui.selectable_value(axis_z, true, "強軸(Mz-θz)");
        ui.selectable_value(axis_z, false, "弱軸(My-θy)");
    });
    let axis_z = *axis_z;

    let point = |f: usize| -> Option<([f64; 2], [f64; 2])> {
        flexural_points(rec, elem_idx, n0, n1, p_i, p_j, ref_vector, axis_z, f)
    };
    let mut i_pts = Vec::with_capacity(rec.frame_time.len());
    let mut j_pts = Vec::with_capacity(rec.frame_time.len());
    for f in 0..rec.frame_time.len() {
        if let Some((pi, pj)) = point(f) {
            i_pts.push(pi);
            j_pts.push(pj);
        }
    }
    let current = point(cur_frame);
    let axis_label = if axis_z { "z" } else { "y" };

    egui_plot::Plot::new(format!("th_flex_{}", elem_id.0))
        .x_axis_label(format!("θ{axis_label} [rad]"))
        .y_axis_label(format!("M{axis_label} [kN·m]"))
        .legend(egui_plot::Legend::default())
        .height(240.0)
        .show(ui, |plot_ui| {
            plot_ui.line(
                egui_plot::Line::new("i端", egui_plot::PlotPoints::from(i_pts))
                    .color(theme::DATA_BLUE)
                    .width(1.5_f32),
            );
            plot_ui.line(
                egui_plot::Line::new("j端", egui_plot::PlotPoints::from(j_pts))
                    .color(theme::PARETO_RED)
                    .width(1.5_f32),
            );
            if let Some((pi, pj)) = current {
                plot_ui.points(
                    egui_plot::Points::new("i端(現在)", egui_plot::PlotPoints::from(vec![pi]))
                        .color(theme::DATA_BLUE)
                        .radius(5.0_f32)
                        .shape(egui_plot::MarkerShape::Circle),
                );
                plot_ui.points(
                    egui_plot::Points::new("j端(現在)", egui_plot::PlotPoints::from(vec![pj]))
                        .color(theme::PARETO_RED)
                        .radius(5.0_f32)
                        .shape(egui_plot::MarkerShape::Circle),
                );
            }
        });
}

/// フレーム `f` の i端・j端 (θ[rad], M[kN·m]) を求める（強軸/弱軸は `axis_z` で選択）。
#[allow(clippy::too_many_arguments)]
fn flexural_points(
    rec: &ThRecording,
    elem_idx: usize,
    n0: usize,
    n1: usize,
    p_i: [f64; 3],
    p_j: [f64; 3],
    ref_vector: [f64; 3],
    axis_z: bool,
    f: usize,
) -> Option<([f64; 2], [f64; 2])> {
    let mf = rec.member_forces.get(f)?.get(elem_idx)?.as_ref()?;
    let (mi, mj) = end_moments(mf, axis_z)?;
    let disp_frame = rec.node_disp.get(f)?;
    let d_i = *disp_frame.get(n0)?;
    let d_j = *disp_frame.get(n1)?;
    let (ry_i, rz_i, ry_j, rz_j) = beam_end_rotations(p_i, p_j, ref_vector, d_i, d_j);
    let (theta_i, theta_j) = if axis_z { (rz_i, rz_j) } else { (ry_i, ry_j) };
    Some(([theta_i, moment_kn_m(mi)], [theta_j, moment_kn_m(mj)]))
}

/// `MemberForces::at` から i端（最小 pos）・j端（最大 pos）の材端モーメント
/// M[N·mm]（強軸Mz/弱軸My、「節点端モーメント」規約）を取り出す（高-1）。
///
/// `MemberForces::at` の値そのものは「断面内力」規約であり、i端（ξ<0.5）側は
/// `squid_n_element::frame::beam::forces::member_forces_from_end_forces` が
/// 節点モーメントの符号を反転して格納している
/// （切断法で連続な内力場にするための反転。j端は反転なし）。
/// 一方、`beam_end_rotations` が返す θ（弦からのたわみ角）はヒンジ詳細
/// （`viewer::hinge`）・増分解析の応答抽出（`nonlinear::pushover::member_response`）
/// と同じ「節点端モーメント」規約（反転なし）を前提に算定している。
/// このため i端だけ符号を再反転し、θ と対にできる規約へ揃える
/// （反転しないと i端の M-θ 勾配の符号がヒンジ詳細と逆転する）。
fn end_moments(mf: &MemberForces, axis_z: bool) -> Option<(f64, f64)> {
    let i_at = mf
        .at
        .iter()
        .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal))?;
    let j_at = mf
        .at
        .iter()
        .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal))?;
    let idx = if axis_z { 5 } else { 4 };
    Some((-i_at.1[idx], j_at.1[idx]))
}

/// 最大応力（内力包絡）に対する短期検定。
#[allow(clippy::too_many_arguments)]
fn draw_peak_check(
    ui: &mut egui::Ui,
    app: &App,
    elem: &ElementData,
    elem_idx: usize,
    rec: &ThRecording,
    th_nonlinear: bool,
    th_applied_long_term: bool,
    axis: &super::MemberAxisEndpoints,
) {
    ui.strong("最大応力に対する検定（全ステップの内力包絡・短期）");
    draw_long_term_note(ui, th_nonlinear, th_applied_long_term);
    ui.label(
        egui::RichText::new(
            "簡易検定です（座屈長さ＝部材長として評価。継手欠損・一本部材合成は考慮しません）。\
             非線形時刻歴で長期を重ねた解析のときのみ、地震時 QD / 柱メカニズムを配線します\
             （線形時刻歴や長期未重畳では、包絡ピークと静的長期の合成が危険側になり得るため無効です）。\
             各成分（N・Qy・Qz・My・Mz）の最大値は全ステップ包絡のため、同一時刻に生じたとは限りません\
             （実際には同時に生じない組合せを検定している可能性があり、安全側ですが過大評価になり得ます）。",
        )
        .size(11.0)
        .color(theme::GRAY_600),
    );

    let Some(peak) = rec
        .peak_member_forces
        .get(elem_idx)
        .and_then(|o| o.as_ref())
    else {
        ui.colored_label(theme::GRAY_600, "内力の記録がないため検定対象外です。");
        return;
    };
    let Some(kind) = design_member_kind(elem, &app.core.model) else {
        ui.colored_label(
            theme::GRAY_600,
            "この要素種別は断面検定の対象外です（軸力のみの減衰・免震・節点ばね要素等）。",
        );
        return;
    };
    let Some(sec) = elem
        .section
        .and_then(|sid| app.core.model.sections.get(sid.index()))
    else {
        ui.colored_label(theme::GRAY_600, "断面が未設定のため検定対象外です。");
        return;
    };
    let Some(mat) = app.core.model.element_material(elem) else {
        ui.colored_label(
            theme::GRAY_600,
            "断面に材料が割り当てられていないため検定対象外です。",
        );
        return;
    };

    let length = axis.length;
    let face_sum = elem.rigid_zone.face_i_or_zero() + elem.rigid_zone.face_j_or_zero();
    let clear_span = if length - face_sum > 0.0 {
        length - face_sum
    } else {
        length
    };
    let m_at = |target: f64| {
        peak.at
            .iter()
            .find(|(p, _)| (p - target).abs() < 1e-6)
            .map(|(_, f)| f[5])
    };
    let end_moments_z = match (m_at(0.0), m_at(1.0)) {
        (Some(a), Some(b)) => Some((a, b)),
        _ => None,
    };
    let shear_span = peak
        .at
        .iter()
        .map(|(_, f)| (f[5].abs(), f[1].abs()))
        .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    let shear_span_y = peak
        .at
        .iter()
        .map(|(_, f)| (f[4].abs(), f[2].abs()))
        .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    // 非線形 TH かつ長期重畳済みのときのみ QD / 柱メカニズムを配線する。
    // 線形 TH の包絡ピークに静的長期を足すと、組合せ内力の前提が崩れ危険側になり得る。
    let wire_qd = th_nonlinear && th_applied_long_term;
    let long_mf = wire_qd
        .then(|| {
            app.core.scoped.results.as_ref().and_then(|r| {
                r.combos
                    .iter()
                    .find(|(n, _)| n == "DL + LL")
                    .or_else(|| {
                        r.combos
                            .iter()
                            .find(|(n, _)| !squid_n_load::combo::is_short_term_combo(n))
                    })
                    .map(|(_, st)| &st.member_forces)
            })
        })
        .flatten();
    let q0_by_elem = wire_qd.then(|| squid_n_job::simple_beam_q0_by_gravity_cases(&app.core.model));
    let seismic_qd = long_mf.and_then(|list| {
        list.iter()
            .find(|(id, _)| *id == elem.id)
            .map(|(_, mf)| squid_n_design_jp::SeismicQd {
                long_at: mf.at.clone(),
                n_factor: 1.5,
                n_mechanism: 1.0,
                q_simple: q0_by_elem.as_ref().and_then(|m| m.get(&elem.id).copied()),
                clear_length: clear_span,
                method: app.core.analysis_cfg.qd_method,
            })
    });
    let column_sum_my = if kind == MemberKind::Column && seismic_qd.is_some() {
        let n_at = |mf: &MemberForces, end: usize| {
            let target = if end == 0 { 0.0 } else { 1.0 };
            mf.at
                .iter()
                .min_by(|a, b| (a.0 - target).abs().total_cmp(&(b.0 - target).abs()))
                .map(|(_, f)| f[0])
                .unwrap_or(0.0)
        };
        let (n_combo_i, n_combo_j) = (n_at(peak, 0), n_at(peak, 1));
        let (n_long_i, n_long_j) = long_mf
            .and_then(|list| list.iter().find(|(id, _)| *id == elem.id))
            .map(|(_, mf)| (n_at(mf, 0), n_at(mf, 1)))
            .unwrap_or((n_combo_i, n_combo_j));
        let adj = squid_n_core::adjacency::NodeAdjacency::build(&app.core.model);
        squid_n_design_jp::rc::compute_column_mechanism_sum_my(
            &app.core.model,
            &adj,
            elem,
            n_long_i,
            n_long_j,
            n_combo_i,
            n_combo_j,
            1.0,
        )
    } else {
        None
    };

    let ctx = DesignCtx {
        term: LoadTerm::Short,
        kind,
        length,
        clear_length: Some(clear_span),
        shear_span,
        shear_span_y,
        rc_damage_control: app.core.analysis_cfg.rc_damage_control,
        bond_method: app.core.analysis_cfg.bond_method,
        end_moments_z,
        mid_moment_z: m_at(0.5),
        // 材料は断面が持つ。RC・SRC の検定は主筋・せん断補強筋・内蔵鉄骨の材料を
        // 要求するため、設計タブの検定（`actions.rs`）と同じく断面から解決して渡す。
        rebar_material: app.core.model.element_rebar_material(elem).cloned(),
        shear_rebar_material: app.core.model.element_shear_rebar_material(elem).cloned(),
        steel_material: app.core.model.element_steel_material(elem).cloned(),
        beam_has_slab: kind == MemberKind::Beam
            && squid_n_design_jp::beam_has_attached_slab(&app.core.model, elem),
        seismic_qd,
        column_sum_my,
        ..Default::default()
    };
    // 検定器の選択は構造種別による（`squid_n_core::structure_kind`。
    // 設計タブの検定と同じ規則）。
    let checker: Box<dyn DesignCheck> = squid_n_design_jp::checker_for(
        squid_n_core::structure_kind::structure_kind_of(Some(sec), Some(mat.category)),
    );

    for (pos, f) in &peak.at {
        let mfa = MemberForcesAt {
            pos: *pos,
            n: f[0],
            qy: f[1],
            qz: f[2],
            my: f[4],
            mz: f[5],
        };
        let outcome = checker.check(&mfa, sec, mat, &ctx);
        draw_outcome_row(ui, *pos, &outcome);
    }
}

/// 長期重ね合わせの有無に関する注記（線形時刻歴は重ね合わせ運用のため対象外、
/// 非線形時刻歴は長期荷重初期載荷の有無で注記を変える）。
///
/// `nonlinear`/`applied_long_term` は `ResponseResult` に記録された**解析時**の
/// フラグを渡す（解析タブの現在の設定値ではない）。解析後に設定を変更しても
/// 注記が実際の解析条件と食い違わないようにするための判断
/// （`dev_docs/handoff/時刻歴アニメーション表示_申し送り.md` 参照）。
fn draw_long_term_note(ui: &mut egui::Ui, nonlinear: bool, applied_long_term: bool) {
    let note = if !nonlinear {
        "線形時刻歴のため、この応答は地震動による応答成分のみです（長期荷重との重ね合わせは含みません）。"
    } else if applied_long_term {
        "長期荷重を初期状態として含む結果です。"
    } else {
        "長期荷重を含まない（水平力のみの）結果です。"
    };
    ui.label(egui::RichText::new(note).size(11.0).color(theme::GRAY_600));
}

/// 1 検定位置分の検定結果を 1 行で表示する（検定比・判定・式別内訳）。
fn draw_outcome_row(ui: &mut egui::Ui, pos: f64, outcome: &CheckOutcome) {
    ui.horizontal_wrapped(|ui| {
        ui.label(format!("ξ={:.2}:", pos));
        match outcome {
            CheckOutcome::Checked(cr) => {
                let ratio = cr.ratio();
                ui.colored_label(
                    theme::status_color(ratio),
                    format!(
                        "検定比 {:.3} ({})",
                        ratio,
                        if cr.ok() { "OK" } else { "NG" }
                    ),
                );
                for c in &cr.components {
                    ui.colored_label(
                        theme::status_color(c.ratio),
                        format!("{} {:.2}", c.kind.label(), c.ratio),
                    )
                    .on_hover_text(&c.detail);
                }
            }
            CheckOutcome::Skipped { reason } => {
                ui.colored_label(theme::GRAY_600, format!("検定不能: {reason}"));
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── axial_relative_disp ────────────────────────────────────────────

    /// X 軸方向部材が両端とも X 方向へ同じだけ動けば相対変位ゼロ（剛体並進）。
    #[test]
    fn axial_relative_disp_rigid_translation_is_zero() {
        let p_i = [0.0, 0.0, 0.0];
        let p_j = [1000.0, 0.0, 0.0];
        let d = [5.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert!((axial_relative_disp(p_i, p_j, d, d)).abs() < 1e-9);
    }

    /// j 端だけ材軸方向(X)へ 2mm 動けば δ=+2mm（引張伸び）。
    #[test]
    fn axial_relative_disp_elongation_positive() {
        let p_i = [0.0, 0.0, 0.0];
        let p_j = [1000.0, 0.0, 0.0];
        let d_i = [0.0; 6];
        let d_j = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let delta = axial_relative_disp(p_i, p_j, d_i, d_j);
        assert!((delta - 2.0).abs() < 1e-9);
    }

    /// 材軸に直交する変位は軸方向相対変位に寄与しない。
    #[test]
    fn axial_relative_disp_transverse_disp_ignored() {
        let p_i = [0.0, 0.0, 0.0];
        let p_j = [1000.0, 0.0, 0.0];
        let d_i = [0.0; 6];
        let d_j = [0.0, 3.0, 0.0, 0.0, 0.0, 0.0];
        assert!(axial_relative_disp(p_i, p_j, d_i, d_j).abs() < 1e-9);
    }

    /// ゼロ長部材は 0 を返す（材軸が定まらない防御的ケース）。
    #[test]
    fn axial_relative_disp_zero_length_returns_zero() {
        let p = [1.0, 2.0, 3.0];
        let d = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert_eq!(axial_relative_disp(p, p, [0.0; 6], d), 0.0);
    }

    // ── beam_end_rotations ─────────────────────────────────────────────

    // 以下のテストは ref_vector=[0,1,0] を使う。この場合 `LocalFrame::from_nodes`
    // は X 軸材（p_i→p_j=X 方向）に対し ey=[0,1,0]（グローバル Y）・ez=[0,0,1]
    // （グローバル Z）となり、グローバル回転成分の添字とローカル軸が素直に対応する
    // （rz(d[5]) が ez=Z 軸まわりの回転、y方向並進が ey=Y 成分）ため、期待値を
    // 手計算しやすい。

    /// 剛体並進（弦回転も端部回転もゼロ）なら材端回転もゼロ。
    #[test]
    fn beam_end_rotations_rigid_translation_is_zero() {
        let p_i = [0.0, 0.0, 0.0];
        let p_j = [1000.0, 0.0, 0.0];
        let d = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0];
        let (ry_i, rz_i, ry_j, rz_j) = beam_end_rotations(p_i, p_j, [0.0, 1.0, 0.0], d, d);
        assert!(ry_i.abs() < 1e-9);
        assert!(rz_i.abs() < 1e-9);
        assert!(ry_j.abs() < 1e-9);
        assert!(rz_j.abs() < 1e-9);
    }

    /// 弦回転どおりに材端が回転していれば（弦に沿った素直な傾き、曲げなし）、
    /// 弦成分が打ち消し合い材端回転（たわみ角）はゼロになる。
    #[test]
    fn beam_end_rotations_matching_chord_cancels() {
        let p_i = [0.0, 0.0, 0.0];
        let p_j = [1000.0, 0.0, 0.0];
        // j 端を Y 方向へ 10mm 変位（弦回転 = 10/1000 = 0.01 rad、Z軸=ez まわり）。
        // 両端の節点回転（Z軸まわり）も同じ弦回転角にしておくと、曲率ゼロの
        // 剛体的な傾きになり、たわみ角は両端とも 0 になるはず。
        let chord = 10.0 / 1000.0;
        let d_i = [0.0, 0.0, 0.0, 0.0, 0.0, chord];
        let d_j = [0.0, 10.0, 0.0, 0.0, 0.0, chord];
        let (_, rz_i, _, rz_j) = beam_end_rotations(p_i, p_j, [0.0, 1.0, 0.0], d_i, d_j);
        assert!(rz_i.abs() < 1e-9, "rz_i={rz_i}");
        assert!(rz_j.abs() < 1e-9, "rz_j={rz_j}");
    }

    /// 弦は動かず材端だけ回転していれば、その回転がそのままたわみ角になる。
    #[test]
    fn beam_end_rotations_pure_end_rotation() {
        let p_i = [0.0, 0.0, 0.0];
        let p_j = [1000.0, 0.0, 0.0];
        let d_i = [0.0; 6];
        let d_j = [0.0, 0.0, 0.0, 0.0, 0.0, 0.02];
        let (_, rz_i, _, rz_j) = beam_end_rotations(p_i, p_j, [0.0, 1.0, 0.0], d_i, d_j);
        assert!(rz_i.abs() < 1e-9);
        assert!((rz_j - 0.02).abs() < 1e-9);
    }

    // ── loop_kind_of ───────────────────────────────────────────────────

    #[test]
    fn loop_kind_of_classifies_axial_and_flexural() {
        assert!(matches!(
            loop_kind_of(&ElementKind::Brace {
                tension_only: false
            }),
            LoopKind::Axial
        ));
        assert!(matches!(
            loop_kind_of(&ElementKind::Damper),
            LoopKind::Axial
        ));
        assert!(matches!(
            loop_kind_of(&ElementKind::Isolator),
            LoopKind::Axial
        ));
        assert!(matches!(
            loop_kind_of(&ElementKind::NodalSpring),
            LoopKind::Axial
        ));
        assert!(matches!(
            loop_kind_of(&ElementKind::Beam),
            LoopKind::Flexural
        ));
        assert!(matches!(
            loop_kind_of(&ElementKind::Wall),
            LoopKind::Unsupported
        ));
    }

    // ── end_moments / flexural_points（高-1: i端モーメント符号規約） ──────

    /// テスト用の弾性梁要素（X 軸材、ref_vector=[0,1,0]。`beam_end_rotations` の
    /// テストと同じ設定で ey=グローバルY・ez=グローバルZ とし、期待値を
    /// 手計算しやすくする）。`squid_n_element::frame::beam::tests::make_test_beam`
    /// と同じ諸元。
    fn cantilever_test_beam() -> squid_n_element::frame::beam::BeamElement {
        use squid_n_core::ids::{ElemId as CoreElemId, NodeId};
        use squid_n_core::model::{EndCondition, RigidZone};
        squid_n_element::frame::beam::BeamElement {
            id: CoreElemId(0),
            e: 205000.0,
            g: 78846.15,
            a: 80000.0,
            a_mass: 80000.0,
            iy: 1.0666667e9,
            iz: 1.0666667e9,
            j: 0.0,
            as_y: 66666.67,
            as_z: 66666.67,
            length: 1000.0,
            density: 0.0,
            nodes: [NodeId(0), NodeId(1)],
            axis: LocalFrame::from_nodes([0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
            rigid: RigidZone::default(),
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            torsion_release: [false, false],
            eval_sections: vec![0.0, 1.0],
            section: None,
            material: None,
            committed_disp: [0.0; 12],
            trial_disp: [0.0; 12],
            local_stiffness_cache: std::sync::OnceLock::new(),
        }
    }

    /// 高-1: `flexural_points` の i端 M-θ 勾配が正になること（節点端モーメント
    /// 規約への統一を検証）。
    ///
    /// i端固定・j端自由の弾性片持ち梁（強軸 Mz 面）相当の変位を与える。荷重
    /// パラメータ P に対する理論たわみ・たわみ角 `v_j=PL³/3EI`・`θ_j=PL²/2EI`
    /// を節点変位として与え（`d_i=0`）、`BeamElement::recover_forces`（FE の
    /// 弾性解＝静定片持ち梁の理論解と一致）で内力を復元する。
    ///
    /// P を増加させた複数フレームで i端の (θ_i, M_i) を `flexural_points` から
    /// 取り出し、隣接フレーム間の勾配 ΔM/Δθ が正であることを確認する
    /// （修正前は `end_moments` が i端で符号反転しておらず、断面内力規約のまま
    /// θ と符号が逆転し負勾配になっていた）。
    #[test]
    fn flexural_points_i_end_slope_is_positive_for_elastic_cantilever() {
        let beam = cantilever_test_beam();
        let p_i = [0.0, 0.0, 0.0];
        let p_j = [1000.0, 0.0, 0.0];
        let ref_vector = [0.0, 1.0, 0.0];
        let l = 1000.0_f64;
        let ei = beam.e * beam.iz;

        let loads = [1.0_f64, 2.0, 3.0];
        let mut frame_time = Vec::new();
        let mut node_disp = Vec::new();
        let mut member_forces = Vec::new();
        for (f, &p) in loads.iter().enumerate() {
            let v_j = p * l.powi(3) / (3.0 * ei);
            let theta_j = p * l.powi(2) / (2.0 * ei);
            let d_i = [0.0; 6];
            let d_j = [0.0, v_j, 0.0, 0.0, 0.0, theta_j];
            let u_elem: [f64; 12] = [
                d_i[0], d_i[1], d_i[2], d_i[3], d_i[4], d_i[5], d_j[0], d_j[1], d_j[2], d_j[3],
                d_j[4], d_j[5],
            ];
            let mf = beam.recover_forces(&u_elem);
            frame_time.push(f as f64);
            node_disp.push(vec![d_i, d_j]);
            member_forces.push(vec![Some(mf)]);
        }
        let rec = ThRecording {
            frame_time,
            node_disp,
            member_forces,
            ..Default::default()
        };

        let mut i_pts = Vec::new();
        for f in 0..loads.len() {
            let (pi, _pj) = flexural_points(&rec, 0, 0, 1, p_i, p_j, ref_vector, true, f)
                .expect("各フレームに内力・変位の記録があるはず");
            i_pts.push(pi); // [theta_i, m_i]
        }

        for w in i_pts.windows(2) {
            let d_theta = w[1][0] - w[0][0];
            let d_m = w[1][1] - w[0][1];
            assert!(
                d_theta.abs() > 1e-12,
                "theta_i が変化していません: {i_pts:?}"
            );
            let slope = d_m / d_theta;
            assert!(
                slope > 0.0,
                "i端の M-θ 勾配が正ではありません: slope={slope} i_pts={i_pts:?}"
            );
        }
    }

    // ── zero_length_local_frame / zero_length_relative_disp（中-2） ───────

    /// 免震支承の零長局所軸は局所 x=鉛直（全体 Z）。
    #[test]
    fn zero_length_local_frame_isolator_axis_is_vertical() {
        let frame =
            zero_length_local_frame(&ElementKind::Isolator, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
        assert!(
            (frame.rot[0][2] - 1.0).abs() < 1e-9,
            "局所xは全体Z: {:?}",
            frame.rot[0]
        );
    }

    /// 節点ばね（免震以外）の零長局所軸は単位回転（全体座標系＝局所座標系）。
    #[test]
    fn zero_length_local_frame_nodal_spring_is_identity() {
        let frame =
            zero_length_local_frame(&ElementKind::NodalSpring, [0.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
        assert_eq!(
            frame.rot,
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        );
    }

    /// 免震支承の零長要素で、水平方向（局所y）の相対変位が ShearY 成分として
    /// 取り出せること（軸=鉛直方向の変位は寄与しない）。
    #[test]
    fn zero_length_relative_disp_isolator_shear_y() {
        let p_i = [0.0, 0.0, 0.0];
        let ref_vector = [1.0, 0.0, 0.0];
        let d_i = [0.0; 6];
        // j端が水平(グローバルX、局所y相当)へ5mm、鉛直(グローバルZ、局所x)へ2mm変位。
        let d_j = [5.0, 0.0, 2.0, 0.0, 0.0, 0.0];
        let shear_y = zero_length_relative_disp(
            &ElementKind::Isolator,
            p_i,
            ref_vector,
            d_i,
            d_j,
            AxialComponent::ShearY,
        );
        let axial = zero_length_relative_disp(
            &ElementKind::Isolator,
            p_i,
            ref_vector,
            d_i,
            d_j,
            AxialComponent::Axial,
        );
        assert!((shear_y - 5.0).abs() < 1e-9, "shear_y={shear_y}");
        assert!((axial - 2.0).abs() < 1e-9, "axial={axial}");
    }

    // ── default_axial_component（中-2） ────────────────────────────────

    #[test]
    fn default_axial_component_isolator_is_shear_horizontal_loop() {
        assert_eq!(
            default_axial_component(&ElementKind::Isolator),
            AxialComponent::ShearY
        );
        assert_eq!(
            default_axial_component(&ElementKind::NodalSpring),
            AxialComponent::Axial
        );
    }
}
