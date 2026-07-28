//! 時刻歴モード（[`super::ViewMode::TimeHistory`]）で部材をクリックした際に開く
//! 詳細ウィンドウ。
//!
//! - 荷重変形関係の履歴ループ（egui_plot）: 軸力系要素（ブレース・ダンパー・
//!   免震・節点ばね）は軸力 N - 軸方向相対変位 δ、梁・柱は材端モーメント M -
//!   材端回転角 θ（弦からのたわみ角）。現在フレームの位置をループ上にマーカー
//!   表示し、フレームスライダーと連動する。
//! - 最大応力に対する検定: `ThRecording::peak_member_forces`（全ステップの内力
//!   包絡）を用いて短期の断面検定を実行する。
//!
//! ヒンジ詳細ウィンドウ（[`super::hinge`]）と同じ構成方針（`egui::Window` +
//! `egui_plot`）を踏襲するが、対象データ（時刻歴の全フレーム記録）が異なるため
//! 実装は独立させている。
//!
//! 検定の組み立ては `app::actions::run_design_check`（断面・材料に応じた
//! RC/Steel/SRC/CFT の `DesignCheck` 実装への振り分け）と同じ考え方だが、
//! 当該関数は `app` モジュール内の非公開関数（`is_steel`/`member_kind_of` 等）に
//! 依存しビューア側からは呼べないため、必要な部分だけ本ファイルへ複製する
//! （地震時短期 QD の長期内力割増・座屈長さの自動算定・鋼継手欠損・一本部材
//! グループ合成・BRB 属性差し替えは簡略化のため含まない。あくまで概算検定）。

use crate::app::App;
use crate::theme;
use squid_n_core::ids::ElemId;
use squid_n_core::model::{ElementData, ElementKind, Model};
use squid_n_design_jp::{
    CheckOutcome, DesignCheck, DesignCtx, LoadTerm, MemberForcesAt, MemberKind,
};
use squid_n_element::beam::MemberForces;
use squid_n_element::transform::LocalFrame;
use squid_n_solver::timehistory::ThRecording;

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
/// 射影する。微小変形の近似（弦回転による二次項は無視）。
pub(super) fn axial_relative_disp(
    p_i: [f64; 3],
    p_j: [f64; 3],
    d_i: [f64; 6],
    d_j: [f64; 6],
) -> f64 {
    let len = super::member_len3(p_i, p_j);
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
fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
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
    let length = super::member_len3(p_i, p_j);
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

/// 部材種別の幾何判定（`app::member_kind_of` と同じ規則の複製。モジュール間で
/// private 関数を共有できないため）。鉛直成分比 |ez| により柱／梁／ブレースを
/// 区別する。
fn geometric_member_kind(elem: &ElementData, model: &Model) -> MemberKind {
    let coords: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .take(2)
        .collect();
    let (Some(&p0), Some(&p1)) = (coords.first(), coords.get(1)) else {
        return MemberKind::Beam;
    };
    let len = super::member_len3(p0, p1);
    if len < 1e-9 {
        return MemberKind::Beam;
    }
    let ez = ((p1[2] - p0[2]) / len).abs();
    if ez >= 0.8 {
        MemberKind::Column
    } else if ez <= 0.2 {
        MemberKind::Beam
    } else {
        MemberKind::Brace
    }
}

/// 断面検定の対象になる部材種別か（軸力のみの減衰・免震・節点ばね要素、
/// 壁・シェル・パネルゾーンは対象外）。
fn design_member_kind(elem: &ElementData, model: &Model) -> Option<MemberKind> {
    match elem.kind {
        ElementKind::Brace { .. } => Some(MemberKind::Brace),
        ElementKind::Beam | ElementKind::Fiber | ElementKind::MultiSpring => {
            Some(geometric_member_kind(elem, model))
        }
        _ => None,
    }
}

/// 鋼材判定（`app::is_steel` と同じ規則の複製）。
fn is_steel_material(name: &str) -> bool {
    let upper = name.to_uppercase();
    upper.starts_with("SS")
        || upper.starts_with("SN")
        || upper.starts_with("SM")
        || upper.starts_with("STK")
        || upper.starts_with("ST")
        || upper.starts_with("SA")
        || upper.starts_with("BC")
}

/// 時刻歴詳細ウィンドウ（`app.th_detail_elem` があれば表示）。
pub(crate) fn show_th_detail_window(ui: &egui::Ui, app: &mut App) {
    let Some(elem_id) = app.th_detail_elem else {
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
        app.th_detail_elem = None;
    }
}

/// `app.th_detail_axis_z` は一時的にローカル変数へ取り出し、`ThRecording` の
/// 借用（`app.results` 由来。フレーム数×要素数の内力を持つため複製すると重い）と
/// `app: &mut App` の同時使用を避け、最後にまとめて書き戻す
/// （§実装内容2 のループ本体は `app.model`／`app.results` の共有参照のみで完結する）。
fn draw_th_detail_content(ui: &mut egui::Ui, app: &mut App, elem_id: ElemId) {
    let Some(recording) = app
        .results
        .as_ref()
        .and_then(|r| r.time_history.as_ref())
        .and_then(|t| t.recording.as_ref())
    else {
        ui.colored_label(theme::GRAY_600, "時刻歴の詳細記録がありません。");
        return;
    };
    let Some(elem_idx) = app.model.elements.iter().position(|e| e.id == elem_id) else {
        ui.colored_label(theme::GRAY_600, "この部材はモデルから削除されています。");
        return;
    };
    let elem = &app.model.elements[elem_idx];
    if elem.nodes.len() < 2 {
        ui.colored_label(theme::GRAY_600, "2 節点未満の要素は対象外です。");
        return;
    }
    let n0 = elem.nodes[0].index();
    let n1 = elem.nodes[1].index();
    let (Some(node0), Some(node1)) = (app.model.nodes.get(n0), app.model.nodes.get(n1)) else {
        ui.colored_label(theme::GRAY_600, "節点情報を取得できません。");
        return;
    };
    let (p_i, p_j) = (node0.coord, node1.coord);

    ui.label(format!("部材 #{}（{}）", elem_id.0, kind_label(&elem.kind)));
    let n_frames = recording.frame_time.len();
    let frame = app.th_frame.min(n_frames.saturating_sub(1));
    if let Some(t) = recording.frame_time.get(frame) {
        ui.label(format!(
            "現在フレーム: {frame}/{} (t={:.3}s)",
            n_frames.saturating_sub(1),
            t
        ));
    }
    ui.separator();

    ui.strong("荷重変形関係の履歴ループ");
    let mut axis_z = app.th_detail_axis_z;
    match loop_kind_of(&elem.kind) {
        LoopKind::Axial => {
            draw_axial_loop(ui, recording, elem_id, elem_idx, n0, n1, p_i, p_j, frame)
        }
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
        LoopKind::Unsupported => {
            ui.colored_label(
                theme::GRAY_600,
                "この要素種別はループ表示に対応していません。",
            );
        }
    }
    app.th_detail_axis_z = axis_z;

    ui.add_space(6.0);
    ui.separator();
    let elem = &app.model.elements[elem_idx];
    draw_peak_check(ui, app, elem, elem_idx, recording);
}

/// 軸力系要素（ブレース・ダンパー・免震・節点ばね）の N-δ ループ。
#[allow(clippy::too_many_arguments)]
fn draw_axial_loop(
    ui: &mut egui::Ui,
    rec: &ThRecording,
    elem_id: ElemId,
    elem_idx: usize,
    n0: usize,
    n1: usize,
    p_i: [f64; 3],
    p_j: [f64; 3],
    cur_frame: usize,
) {
    if !elem_has_force_recording(rec, elem_idx) {
        ui.colored_label(theme::GRAY_600, "この部材の内力は記録されていません。");
        return;
    }
    let series: Vec<[f64; 2]> = (0..rec.frame_time.len())
        .filter_map(|f| axial_point(rec, elem_idx, n0, n1, p_i, p_j, f))
        .collect();
    let current = axial_point(rec, elem_idx, n0, n1, p_i, p_j, cur_frame);

    egui_plot::Plot::new(format!("th_axial_{}", elem_id.0))
        .x_axis_label("δ [mm]")
        .y_axis_label("N [kN]")
        .height(220.0)
        .show(ui, |plot_ui| {
            plot_ui.line(
                egui_plot::Line::new("N-δ", egui_plot::PlotPoints::from(series))
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

/// フレーム `f` の軸力系 (δ[mm], N[kN]) を求める。内力・変位のいずれかが
/// 欠けていれば `None`。
fn axial_point(
    rec: &ThRecording,
    elem_idx: usize,
    n0: usize,
    n1: usize,
    p_i: [f64; 3],
    p_j: [f64; 3],
    f: usize,
) -> Option<[f64; 2]> {
    let mf = rec.member_forces.get(f)?.get(elem_idx)?.as_ref()?;
    let n_kn = mf.at.first()?.1[0] / 1e3;
    let disp_frame = rec.node_disp.get(f)?;
    let d_i = *disp_frame.get(n0)?;
    let d_j = *disp_frame.get(n1)?;
    let delta = axial_relative_disp(p_i, p_j, d_i, d_j);
    Some([delta, n_kn])
}

/// 梁・柱の M-θ ループ（i端・j端、強軸/弱軸切替可能）。
///
/// `axis_z`（表示中の曲げ軸）は呼び出し側（`App::th_detail_axis_z`）が保持する
/// 状態をローカル変数として受け渡す。`ThRecording` の借用（`app.results` 由来）と
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
        .y_axis_label(format!("M{axis_label} [kN・m]"))
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

/// フレーム `f` の i端・j端 (θ[rad], M[kN・m]) を求める（強軸/弱軸は `axis_z` で選択）。
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
    Some(([theta_i, mi / 1e6], [theta_j, mj / 1e6]))
}

/// `MemberForces::at` から i端（最小 pos）・j端（最大 pos）の M[N・mm]（強軸Mz/弱軸My）を取り出す。
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
    Some((i_at.1[idx], j_at.1[idx]))
}

/// 最大応力（内力包絡）に対する短期検定。
fn draw_peak_check(
    ui: &mut egui::Ui,
    app: &App,
    elem: &ElementData,
    elem_idx: usize,
    rec: &ThRecording,
) {
    ui.strong("最大応力に対する検定（全ステップの内力包絡・短期）");
    draw_long_term_note(ui, app);
    ui.label(
        egui::RichText::new(
            "簡易検定です（座屈長さ＝部材長として評価。継手欠損・一本部材合成・地震時QDの長期割増は考慮しません）。",
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
    let Some(kind) = design_member_kind(elem, &app.model) else {
        ui.colored_label(
            theme::GRAY_600,
            "この要素種別は断面検定の対象外です（軸力のみの減衰・免震・節点ばね要素等）。",
        );
        return;
    };
    let Some(sec) = elem
        .section
        .and_then(|sid| app.model.sections.get(sid.index()))
    else {
        ui.colored_label(theme::GRAY_600, "断面が未設定のため検定対象外です。");
        return;
    };
    let Some(mat) = elem
        .material
        .and_then(|mid| app.model.materials.get(mid.index()))
    else {
        ui.colored_label(theme::GRAY_600, "材料が未設定のため検定対象外です。");
        return;
    };

    let length = super::member_len3(
        app.model.nodes[elem.nodes[0].index()].coord,
        app.model.nodes[elem.nodes[1].index()].coord,
    );
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
    let ctx = DesignCtx {
        term: LoadTerm::Short,
        kind,
        length,
        shear_span,
        shear_span_y,
        rc_damage_control: app.analysis_cfg.rc_damage_control,
        end_moments_z,
        mid_moment_z: m_at(0.5),
        ..Default::default()
    };
    let checker: Box<dyn DesignCheck> = match sec.shape {
        Some(squid_n_core::section_shape::SectionShape::SrcRect { .. }) => {
            Box::new(squid_n_design_jp::SrcDesign)
        }
        Some(squid_n_core::section_shape::SectionShape::CftBox { .. })
        | Some(squid_n_core::section_shape::SectionShape::CftPipe { .. }) => {
            Box::new(squid_n_design_jp::CftDesign)
        }
        _ if is_steel_material(&mat.name) => Box::new(squid_n_design_jp::SteelDesign),
        _ => Box::new(squid_n_design_jp::RcDesign),
    };

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
/// 非線形時刻歴は `th_apply_long_term` の有無で注記を変える）。
fn draw_long_term_note(ui: &mut egui::Ui, app: &App) {
    let note = if !app.analysis_cfg.th_nonlinear {
        "線形時刻歴のため、この応答は地震動による応答成分のみです（長期荷重との重ね合わせは含みません）。"
    } else if app.analysis_cfg.th_apply_long_term {
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

    // ── is_steel_material ─────────────────────────────────────────────

    #[test]
    fn is_steel_material_detects_jis_grades() {
        assert!(is_steel_material("SN400B"));
        assert!(is_steel_material("ss400"));
        assert!(!is_steel_material("SD295"));
        assert!(!is_steel_material("Fc21"));
    }
}
