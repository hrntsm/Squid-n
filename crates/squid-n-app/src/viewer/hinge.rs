//! ヒンジ図（増分解析のヒンジ発生位置の可視化）の描画。
//!
//! 増分解析（プッシュオーバー解析）の結果 [`PushoverResult::hinges`] は、閾値
//! （ひび割れ／降伏／終局）を超過している間、同じ (部材, 端) が毎ステップ
//! push されるため単純な件数集計はできない。本モジュールはまず
//! [`aggregate_hinges`] で (部材, 端) ごとに 1 件へ集約し（最高レベル・最大
//! 塑性率・初出 step を保持）、その集約結果を材端の少し内側にマーカーとして描く。
//!
//! マーカーは節点直上ではなく材軸に沿って 10% 内側へ寄せて描く。節点直上に描くと、
//! 同一節点に集まる複数部材のヒンジが重なって判別できなくなるため。
//!
//! ヒンジ図で部材をクリックすると、[`show_hinge_detail_window`] がその部材の
//! ヒンジ詳細ウィンドウ（M-θ カーブ・N-M 相関図・ファイバー断面の塑性化マップ）
//! を開く。データは [`PushoverResult::member_history`]（材端応答の全ステップ
//! 履歴）・[`PushoverResult::fiber_states`]（終局時のファイバー断面状態）を使う。

use std::collections::HashMap;

use crate::app::App;
use crate::theme;
use squid_n_core::ids::ElemId;
use squid_n_core::material_grade::{
    material_strength_factor_rebar, material_strength_factor_steel,
};
use squid_n_core::model::{ElementData, ElementKind, Model};
use squid_n_element::behavior::{FiberSectionState, FiberStateSample};
use squid_n_section::mn_surface::{build_surface, plastic_fibers, StrengthParams, YieldModelKind};
use squid_n_solver::pushover::{HingeEvent, HingeLevel, MemberStepState};

/// (部材, 端) ごとに集約したヒンジ情報。
///
/// `HingeLevel`（squid-n-solver）は `PartialEq` を持たないため、本構造体も
/// `PartialEq` は導出しない（比較はフィールド単位・`level_rank` 経由で行う）。
#[derive(Clone, Debug)]
pub(super) struct HingeMarker {
    pub elem: ElemId,
    /// 材端: `false` = i端（`elem.nodes[0]` 側、pos<0.5）、`true` = j端（pos≥0.5）。
    pub end_j: bool,
    /// 集約後の最高ヒンジレベル。
    pub level: HingeLevel,
    /// 集約後の最大塑性率。
    pub max_ductility: f64,
    /// 初めてヒンジ（Crack 以上）が記録された step（最小 step）。
    pub first_step: u32,
}

/// `HingeLevel` の重大度ランク（Crack < Yield < Ultimate）。
/// `HingeLevel` は `Ord` を持たないため、比較用に整数へ写像する。
fn level_rank(level: &HingeLevel) -> u8 {
    match level {
        HingeLevel::Crack => 0,
        HingeLevel::Yield => 1,
        HingeLevel::Ultimate => 2,
    }
}

/// ヒンジ発生履歴 `hinges` を (部材, 端) ごとに集約する（純粋関数）。
///
/// 同一 (部材, 端) は、その端が閾値を超過している限り毎ステップ重複記録される
/// ため（`crates/squid-n-solver/src/nonlinear/pushover/mechanism.rs` の
/// `determine_mechanism` と同様の重複排除）、以下を保持する 1 件へまとめる。
/// - 最高レベル（Crack < Yield < Ultimate）
/// - 最大塑性率（`ductility`）
/// - 初めてヒンジ（Crack 以上）が記録された step（最小 step）
pub(super) fn aggregate_hinges(hinges: &[HingeEvent]) -> Vec<HingeMarker> {
    let mut map: HashMap<(ElemId, bool), HingeMarker> = HashMap::new();
    for h in hinges {
        let end_j = h.pos >= 0.5;
        let key = (h.elem, end_j);
        map.entry(key)
            .and_modify(|m| {
                if level_rank(&h.level) > level_rank(&m.level) {
                    m.level = h.level.clone();
                }
                if h.ductility > m.max_ductility {
                    m.max_ductility = h.ductility;
                }
                if h.step < m.first_step {
                    m.first_step = h.step;
                }
            })
            .or_insert_with(|| HingeMarker {
                elem: h.elem,
                end_j,
                level: h.level.clone(),
                max_ductility: h.ductility,
                first_step: h.step,
            });
    }
    let mut result: Vec<HingeMarker> = map.into_values().collect();
    // 表示・テストの安定のため部材ID→端の順で並べる。
    result.sort_by_key(|m| (m.elem.0, m.end_j));
    result
}

/// ヒンジレベルに応じた色（[`theme`] の既存定数を流用）。
fn hinge_color(level: &HingeLevel) -> egui::Color32 {
    match level {
        HingeLevel::Crack => theme::SECONDARY_AMBER,
        HingeLevel::Yield => theme::PARETO_RED,
        HingeLevel::Ultimate => theme::HILITE_PURPLE,
    }
}

/// ヒンジレベルの表示ラベル。
fn hinge_level_label(level: &HingeLevel) -> &'static str {
    match level {
        HingeLevel::Crack => "ひび割れ",
        HingeLevel::Yield => "降伏",
        HingeLevel::Ultimate => "終局",
    }
}

/// ヒンジマーカーの塗り円半径（px）。
const MARKER_R: f32 = 4.0;
/// マーカー中心を材端から材軸に沿って内側へ寄せる比率（0.0=材端、0.5=中点）。
const INSET_T: f32 = 0.1;

/// ヒンジ図を描く。`pts` は `viewer_panel` で計算済みの節点スクリーン座標
/// （`app.model.nodes` と同じ順序）。
pub(super) fn draw_hinge(painter: &egui::Painter, app: &App, pts: &[egui::Pos2]) {
    let Some(po) = app.results.as_ref().and_then(|r| r.pushover.as_ref()) else {
        draw_no_result_legend(painter);
        return;
    };

    let markers = aggregate_hinges(&po.hinges);
    // レベル別の件数（凡例用。0=ひび割れ／1=降伏／2=終局）。
    let mut counts = [0usize; 3];

    for m in &markers {
        let Some(elem) = app.model.elements.iter().find(|e| e.id == m.elem) else {
            continue;
        };
        if elem.nodes.len() < 2 {
            continue;
        }
        let n0 = elem.nodes[0].index();
        let n1 = elem.nodes[1].index();
        if n0 >= pts.len() || n1 >= pts.len() {
            continue;
        }
        let (p0, p1) = (pts[n0], pts[n1]);
        // i端(end_j=false)は始点側から、j端(end_j=true)は終点側から内側へ寄せる。
        let t = if m.end_j { 1.0 - INSET_T } else { INSET_T };
        let center = egui::pos2(p0.x + (p1.x - p0.x) * t, p0.y + (p1.y - p0.y) * t);

        let color = hinge_color(&m.level);
        counts[level_rank(&m.level) as usize] += 1;

        painter.circle_filled(center, MARKER_R, color);
        if matches!(m.level, HingeLevel::Ultimate) {
            // 終局は外周リングを重ねて目立たせる。
            painter.circle_stroke(center, MARKER_R + 2.5, egui::Stroke::new(1.5_f32, color));
        }
    }

    draw_hinge_legend(painter, &counts);
}

/// ヒンジ図のホバー詳細ツールチップ。部材 `elem_id` にヒンジがあれば
/// i端／j端それぞれの最高レベル・最大塑性率・初出 step を表示する。
/// ヒンジの無い部材は何も表示しない。
pub(super) fn show_hinge_tooltip(ui: &egui::Ui, app: &App, elem_id: ElemId) {
    let Some(po) = app.results.as_ref().and_then(|r| r.pushover.as_ref()) else {
        return;
    };
    let markers = aggregate_hinges(&po.hinges);
    let mut rows: Vec<&HingeMarker> = markers.iter().filter(|m| m.elem == elem_id).collect();
    if rows.is_empty() {
        return;
    }
    rows.sort_by_key(|m| m.end_j);

    // `show_tooltip_at_pointer` は egui 0.34 で非推奨だが、モデル化図・検定比図と
    // 同じ方針（`#[allow(deprecated)]`）で使用する。
    #[allow(deprecated)]
    egui::show_tooltip_at_pointer(
        ui.ctx(),
        ui.layer_id(),
        egui::Id::new("hinge_tooltip"),
        |ui| {
            ui.label(format!("部材 #{}", elem_id.0));
            for m in &rows {
                let end_label = if m.end_j { "j端" } else { "i端" };
                ui.colored_label(
                    hinge_color(&m.level),
                    format!(
                        "{}: {} (μ={:.2}, step {})",
                        end_label,
                        hinge_level_label(&m.level),
                        m.max_ductility,
                        m.first_step
                    ),
                );
            }
        },
    );
}

/// 増分解析が未実行の場合の案内表示。
fn draw_no_result_legend(painter: &egui::Painter) {
    painter.text(
        egui::pos2(
            painter.clip_rect().min.x + 10.0,
            painter.clip_rect().min.y + 10.0,
        ),
        egui::Align2::LEFT_TOP,
        "増分解析が未実行です（解析タブから実行してください）。",
        egui::FontId::proportional(14.0),
        theme::GRAY_600,
    );
}

/// ビュー左上にヒンジ図の凡例（タイトル・レベル別の色見本＋件数）を描く。
fn draw_hinge_legend(painter: &egui::Painter, counts: &[usize; 3]) {
    let rect = painter.clip_rect();
    let x0 = rect.min.x + 10.0;
    let mut y = rect.min.y + 10.0;
    const LINE_H: f32 = 16.0;
    const FONT: f32 = 12.0;

    let title_rect = painter.text(
        egui::pos2(x0, y),
        egui::Align2::LEFT_TOP,
        "ヒンジ図（増分解析）",
        egui::FontId::proportional(14.0),
        theme::GRAY_700,
    );
    y = title_rect.max.y + 4.0;

    let entries = [
        (HingeLevel::Crack, counts[0]),
        (HingeLevel::Yield, counts[1]),
        (HingeLevel::Ultimate, counts[2]),
    ];
    for (level, count) in entries {
        let c = egui::pos2(x0 + 6.0, y + FONT * 0.5);
        painter.circle_filled(c, MARKER_R, hinge_color(&level));
        painter.text(
            egui::pos2(x0 + 16.0, y),
            egui::Align2::LEFT_TOP,
            format!("{} ({}件)", hinge_level_label(&level), count),
            egui::FontId::proportional(FONT),
            theme::GRAY_600,
        );
        y += LINE_H;
    }
}

// ============================================================================
// ヒンジ詳細ウィンドウ（クリックで部材のヒンジ状態を確認する図）
// ============================================================================

/// N-M 相関図用の曲線キャッシュ（部材のファイバー分割・曲面構築は数十ms
/// かかりうるため、選択部材・ステップ数が変わらない限り再計算しない）。
pub struct MnCurveCache {
    elem: ElemId,
    /// キャッシュ有効性の簡易判定に使うステップ数（増分解析を再実行すると
    /// 通常はステップ数も変わるため、同一部材のまま結果だけ更新された場合の
    /// 取りこぼしをある程度防げる）。
    step_count: usize,
    /// 正曲げ側（β 方向）の N-M 曲線 [M(kN・m, 符号付き), N(kN, 圧縮正)]。
    pos: Vec<[f64; 2]>,
    /// 負曲げ側（β+π 方向）の N-M 曲線。
    neg: Vec<[f64; 2]>,
    /// N-My-Mz 相関曲面（3D ワイヤーフレーム表示用。単位は N・N・mm、引張正の
    /// `MnSurface` 既定規約のまま保持し、描画時に正規化する）。
    surface: squid_n_section::mn_surface::MnSurface,
}

/// N-M 相関曲面の格子解像度（経線方向・周方向）。`mn_view.rs` と同じ値を使い、
/// 断面詳細ビューと同等の精度にする。
const MN_N_ALPHA: usize = 24;
const MN_N_BETA: usize = 48;

/// 部材の最終応答レコードから、採用する曲げ面（強軸 Mz／弱軸 My）を選ぶ
/// （純粋関数）。i端・j端のうち絶対値が大きい方の成分を軸ごとに比較し、
/// 大きい軸を採用する（同値なら強軸を採用）。
pub(super) fn dominant_bend_axis_z(last: &MemberStepState) -> bool {
    let mz_max = last.mz_i.abs().max(last.mz_j.abs());
    let my_max = last.my_i.abs().max(last.my_j.abs());
    mz_max >= my_max
}

/// 採用軸に応じた i端・j端の (|θ|[rad], |M|[N·mm]) 点列を全ステップから
/// 抽出する（純粋関数）。θ は弦からの材端回転、M は剛域フェイス位置の局所曲げ。
pub(super) fn m_theta_series(
    records: &[MemberStepState],
    bend_dir_z: bool,
) -> (Vec<[f64; 2]>, Vec<[f64; 2]>) {
    let i_pts = records
        .iter()
        .map(|r| {
            let (theta, m) = if bend_dir_z {
                (r.rz_i, r.mz_i)
            } else {
                (r.ry_i, r.my_i)
            };
            [theta.abs() as f64, m.abs() as f64]
        })
        .collect();
    let j_pts = records
        .iter()
        .map(|r| {
            let (theta, m) = if bend_dir_z {
                (r.rz_j, r.mz_j)
            } else {
                (r.ry_j, r.my_j)
            };
            [theta.abs() as f64, m.abs() as f64]
        })
        .collect();
    (i_pts, j_pts)
}

/// 採用軸の端最大 |M|（絶対値が大きい方の端の符号付き値）と軸力から、
/// 応答経路 [M(kN・m), N(kN、圧縮正)] を全ステップ抽出する（純粋関数）。
/// `member_history` の軸力は既に圧縮正のため、N-M 曲線側の符号変換のみで
/// 済む（[`extract_mn_meridian`] 参照）。
///
/// 先頭に原点 [0.0, 0.0]（無載荷状態）を前置する。`member_history` の記録は
/// 最初の記録ステップから始まるため、これが無いと経路の始点が分からない。
/// 長期荷重の初期載荷が実装されれば最初の記録ステップは長期荷重時点になるが、
/// その場合も「無載荷→長期荷重→水平力」の経路として原点前置のままで正しい。
pub(super) fn n_m_response_path(records: &[MemberStepState], bend_dir_z: bool) -> Vec<[f64; 2]> {
    let mut path = vec![[0.0, 0.0]];
    path.extend(records.iter().map(|r| {
        let (mi, mj) = if bend_dir_z {
            (r.mz_i, r.mz_j)
        } else {
            (r.my_i, r.my_j)
        };
        let m = if mi.abs() >= mj.abs() { mi } else { mj };
        [m as f64 / 1e6, r.n as f64 / 1e3]
    }));
    path
}

/// 3D 表示用の応答経路 [My(N・mm,符号付き), Mz(N・mm,符号付き), N(N,引張正)] を
/// 全ステップ抽出する（純粋関数）。各ステップで i端・j端のうち合成曲げ
/// （√(My²+Mz²)）が大きい方の端を採用する（[`n_m_response_path`] は採用軸
/// 1 成分のみを追うのに対し、3D 表示は My・Mz の両成分をそのまま使える）。
///
/// `MnSurface`（[`build_mn_curve_cache`]）は引張正の N 規約のため、圧縮正の
/// `member_history` の軸力符号を反転して揃える。先頭に原点
/// [0.0, 0.0, 0.0]（無載荷状態）を前置する（[`n_m_response_path`] と同じ理由）。
pub(super) fn n_my_mz_response_path_3d(records: &[MemberStepState]) -> Vec<[f64; 3]> {
    let mut path = vec![[0.0, 0.0, 0.0]];
    path.extend(records.iter().map(|r| {
        let mag_i = ((r.my_i as f64).powi(2) + (r.mz_i as f64).powi(2)).sqrt();
        let mag_j = ((r.my_j as f64).powi(2) + (r.mz_j as f64).powi(2)).sqrt();
        let (my, mz) = if mag_i >= mag_j {
            (r.my_i, r.mz_i)
        } else {
            (r.my_j, r.mz_j)
        };
        [my as f64, mz as f64, -(r.n as f64)]
    }));
    path
}

/// 曲げ方向 `bend_dir_z` に対応する N-M 相関曲面の周方向格子列（正曲げ側・
/// 負曲げ側の2方向）を返す（純粋関数）。`build_surface` の格子は
/// β=2π·j/n_beta でパラメータ化されており、β=0/π が弱軸(My)、β=π/2/3π/2 が
/// 強軸(Mz) の純曲げ方向に対応する（`plastic_point` の (ky,kz) 定義を参照）。
pub(super) fn mn_beta_columns(n_beta: usize, bend_dir_z: bool) -> (usize, usize) {
    let j_pos = if bend_dir_z { n_beta / 4 } else { 0 };
    let j_neg = (j_pos + n_beta / 2) % n_beta;
    (j_pos, j_neg)
}

/// `grid`（`MnSurface::grid`、経線方向 i × 周方向 j の格子点 [N, My, Mz]）の
/// 列 `beta_col` から、曲げ方向 `bend_dir_z` の N-M 経線を抽出する（純粋関数）。
///
/// `MnSurface` は引張正の N 規約だが、応答（`MemberStepState::n`）は圧縮正の
/// ため符号を反転して揃える。単位は表示用に [kN・m]・[kN] へ換算する。
pub(super) fn extract_mn_meridian(
    grid: &[Vec<[f64; 3]>],
    beta_col: usize,
    bend_dir_z: bool,
) -> Vec<[f64; 2]> {
    let m_index = if bend_dir_z { 2 } else { 1 };
    grid.iter()
        .filter_map(|row| row.get(beta_col))
        .map(|p| [p[m_index] / 1e6, -p[0] / 1e3])
        .collect()
}

/// 部材が軸力を受ける（N-M 相関図の対象となる）か判定する（純粋関数）。
/// 鉛直材（柱。両端節点の水平距離が 1mm 未満。`app::is_vertical_pair` と
/// 同じ判定規則をここで再実装する）、またはファイバー系
/// （Fiber／MultiSpring／Brace）要素種別を対象とする。
pub(super) fn is_axial_bending_member(elem: &ElementData, model: &Model) -> bool {
    if matches!(
        elem.kind,
        ElementKind::Fiber | ElementKind::MultiSpring | ElementKind::Brace { .. }
    ) {
        return true;
    }
    let (Some(&n0), Some(&n1)) = (elem.nodes.first(), elem.nodes.get(1)) else {
        return false;
    };
    let (Some(a), Some(b)) = (model.nodes.get(n0.index()), model.nodes.get(n1.index())) else {
        return false;
    };
    let dx = a.coord[0] - b.coord[0];
    let dy = a.coord[1] - b.coord[1];
    (dx * dx + dy * dy).sqrt() < 1.0
}

/// `sections`（ある部材のファイバー断面状態。xi 昇順とは限らない）から、
/// 指定端（i端=false→最小 xi、j端=true→最大 xi）に最も近い断面を返す
/// （純粋関数）。ヒンジは危険断面＝可撓部の材端付近に生じるため、その端に
/// 最も近いガウス点断面を代表として選ぶ。
pub(super) fn pick_fiber_section(
    sections: &[FiberSectionState],
    end_j: bool,
) -> Option<&FiberSectionState> {
    if end_j {
        sections
            .iter()
            .max_by(|a, b| a.xi.partial_cmp(&b.xi).unwrap_or(std::cmp::Ordering::Equal))
    } else {
        sections
            .iter()
            .min_by(|a, b| a.xi.partial_cmp(&b.xi).unwrap_or(std::cmp::Ordering::Equal))
    }
}

/// 部材 `elem_id` の N-M 相関曲線を算定する。断面形状が未定義、または断面から
/// ファイバーを生成できない場合は `None`。
fn build_mn_curve_cache(
    app: &App,
    elem_id: ElemId,
    bend_dir_z: bool,
    step_count: usize,
) -> Option<MnCurveCache> {
    let elem = app.model.elements.iter().find(|e| e.id == elem_id)?;
    let sec = elem
        .section
        .and_then(|sid| app.model.sections.get(sid.index()))?;
    let shape = sec.shape.clone()?;
    let mat = elem
        .material
        .and_then(|mid| app.model.materials.get(mid.index()));

    // 保有水平耐力計算（プッシュオーバー）と整合する材料強度割増を適用する
    // （`pushover/hinge.rs::member_moment_thresholds` と同じ規約）。
    let steel_fy = mat.and_then(|m| m.fy).unwrap_or(235.0)
        * mat.map(material_strength_factor_steel).unwrap_or(1.0);
    let rebar_fy = mat.and_then(|m| m.fy).unwrap_or(345.0)
        * mat.map(material_strength_factor_rebar).unwrap_or(1.0);
    let concrete_fc = mat.and_then(|m| m.fc).unwrap_or(24.0);
    let steel_e = mat.map(|m| m.young).unwrap_or(205000.0);
    let strength = StrengthParams {
        steel_fy,
        rebar_fy,
        concrete_fc,
        steel_e,
    };

    let fibers = plastic_fibers(&shape, &strength, YieldModelKind::MultiFiber);
    if fibers.is_empty() {
        return None;
    }
    let surface = build_surface(&fibers, YieldModelKind::MultiFiber, MN_N_ALPHA, MN_N_BETA);
    let (j_pos, j_neg) = mn_beta_columns(MN_N_BETA, bend_dir_z);
    let pos = extract_mn_meridian(&surface.grid, j_pos, bend_dir_z);
    let neg = extract_mn_meridian(&surface.grid, j_neg, bend_dir_z);

    Some(MnCurveCache {
        elem: elem_id,
        step_count,
        pos,
        neg,
        surface,
    })
}

/// `app.hinge_mn_cache` が古ければ（選択部材・ステップ数が変われば）再計算する。
fn ensure_mn_cache(app: &mut App, elem_id: ElemId, bend_dir_z: bool, step_count: usize) {
    let stale = match &app.hinge_mn_cache {
        Some(c) => c.elem != elem_id || c.step_count != step_count,
        None => true,
    };
    if stale {
        app.hinge_mn_cache = build_mn_curve_cache(app, elem_id, bend_dir_z, step_count);
    }
}

/// ヒンジ詳細ウィンドウ（クリックで開く）。`app.hinge_detail_elem` が `None`
/// なら何も描かない。閉じるボタン（×）で `app.hinge_detail_elem` をクリアする。
pub(crate) fn show_hinge_detail_window(ui: &egui::Ui, app: &mut App) {
    let Some(elem_id) = app.hinge_detail_elem else {
        return;
    };
    let mut open = true;
    egui::Window::new(format!("ヒンジ詳細: 部材 #{}", elem_id.0))
        .id(egui::Id::new("hinge_detail_window"))
        .resizable(true)
        .collapsible(true)
        .default_size([440.0, 620.0])
        .open(&mut open)
        .show(ui.ctx(), |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                draw_hinge_detail_content(ui, app, elem_id);
            });
        });
    if !open {
        app.hinge_detail_elem = None;
        app.hinge_mn_cache = None;
    }
}

/// ヒンジ詳細ウィンドウの中身。共通ヘッダ（i端/j端の集約ヒンジ情報）に続けて、
/// M-θ カーブ（常時）・N-M 相関図（軸力を受ける部材のみ）・ファイバー断面の
/// 塑性化マップ（ファイバー要素のみ）を該当するものだけ縦に並べる。
fn draw_hinge_detail_content(ui: &mut egui::Ui, app: &mut App, elem_id: ElemId) {
    let Some(po) = app.results.as_ref().and_then(|r| r.pushover.as_ref()) else {
        ui.colored_label(theme::GRAY_600, "増分解析が未実行です。");
        return;
    };

    // 共通ヘッダ: i端・j端それぞれの集約ヒンジ情報（第1ラウンドの集約を再利用）。
    let mine: Vec<HingeMarker> = aggregate_hinges(&po.hinges)
        .into_iter()
        .filter(|m| m.elem == elem_id)
        .collect();
    if mine.is_empty() {
        ui.label("この部材にはヒンジが記録されていません。");
        return;
    }
    for m in &mine {
        let end_label = if m.end_j { "j端" } else { "i端" };
        ui.colored_label(
            hinge_color(&m.level),
            format!(
                "{}: {} (μ={:.2}, 初出 step {})",
                end_label,
                hinge_level_label(&m.level),
                m.max_ductility,
                m.first_step
            ),
        );
    }
    ui.separator();

    // 応答履歴（M-θ・N-M 応答経路の元データ）。旧プロジェクトファイル等で
    // 空の場合は再解析を促す（`MemberStepState` は Copy のため複製は軽量）。
    let records: Vec<MemberStepState> = match po.member_history.iter().find(|h| h.elem == elem_id) {
        Some(h) if !h.records.is_empty() => h.records.clone(),
        _ => {
            ui.colored_label(
                theme::GRAY_600,
                "応答履歴データがありません（再解析すると詳細を表示できます）。",
            );
            return;
        }
    };
    // 終局時のファイバー断面状態（無ければファイバー要素以外、または旧データ）。
    let fiber_sections: Option<Vec<FiberSectionState>> = po
        .fiber_states
        .iter()
        .find(|(id, _)| *id == elem_id)
        .map(|(_, s)| s.clone());

    let Some(last) = records.last() else {
        return;
    };
    let bend_dir_z = dominant_bend_axis_z(last);
    ui.label(format!(
        "採用曲げ面: {}",
        if bend_dir_z {
            "強軸(Mz)"
        } else {
            "弱軸(My)"
        }
    ));

    // 1. M-θ カーブ（荷重変形カーブ）: 応答履歴があれば常に表示。
    ui.strong("M-θ カーブ（荷重変形カーブ）");
    draw_m_theta_plot(ui, elem_id, &records, bend_dir_z, &mine);
    ui.separator();

    // 2. N-M 相関図: 軸力を受ける部材（柱、またはファイバー系要素）のみ。
    let is_axial = app
        .model
        .elements
        .iter()
        .find(|e| e.id == elem_id)
        .is_some_and(|e| is_axial_bending_member(e, &app.model));
    if is_axial {
        ui.strong("N-M 相関図");
        ensure_mn_cache(app, elem_id, bend_dir_z, records.len());
        // カメラ状態は `app` からローカルへ複製して使う（`app.hinge_mn_cache`
        // の借用と同時に `app` を可変借用しないため）。描画後に書き戻す。
        let mut cam = app.hinge_mn_camera.clone();
        match app.hinge_mn_cache.as_ref().filter(|c| c.elem == elem_id) {
            Some(cache) => draw_mn_plot(ui, cache, elem_id, &records, bend_dir_z, &mut cam),
            None => {
                ui.colored_label(
                    theme::GRAY_600,
                    "断面形状が未定義のため N-M 相関図を表示できません。",
                );
            }
        }
        app.hinge_mn_camera = cam;
        ui.separator();
    }

    // 3. ファイバー断面の塑性化マップ: ファイバー要素のみ（fiber_states に記録あり）。
    if let Some(sections) = fiber_sections {
        ui.strong("ファイバー断面の塑性化マップ（終局時）");
        draw_fiber_maps(ui, elem_id, &sections, &mine);
    }
}

/// M-θ カーブ（i端・j端の (|θ|,|M|) 骨格）を egui_plot で描く。
///
/// `Plot` の ID に `elem_id` を含める。egui_plot はズーム／パン状態
/// （`PlotMemory`）を ID だけで永続化するため、固定 ID のままだと、ある部材の
/// グラフを操作（ドラッグ／スクロールでのズーム）した後に別の部材のヒンジ詳細を
/// 開くと、その操作で確定した表示範囲を新しい部材のデータにそのまま流用してしまい
/// 「カーブが描画領域の一部にしか収まらない／余白が過大」に見える。部材ごとに ID を
/// 分ければ、選択部材の切替時は必ず新規の `PlotMemory`（既定=自動フィット）から
/// 始まるため、デフォルト表示は常に 5%（`egui_plot` 既定の `margin_fraction`）の
/// 余白付きでカーブ全体を収める。
fn draw_m_theta_plot(
    ui: &mut egui::Ui,
    elem_id: ElemId,
    records: &[MemberStepState],
    bend_dir_z: bool,
    mine: &[HingeMarker],
) {
    let (i_pts, j_pts) = m_theta_series(records, bend_dir_z);
    let has_i = mine.iter().any(|m| !m.end_j);
    let has_j = mine.iter().any(|m| m.end_j);
    egui_plot::Plot::new(format!("hinge_m_theta_{}", elem_id.0))
        .x_axis_label("|θ| [rad]")
        .y_axis_label("|M| [kN・m]")
        .legend(egui_plot::Legend::default())
        .height(220.0)
        .show(ui, |plot_ui| {
            if has_i {
                plot_m_theta_end(plot_ui, "i端", &i_pts, theme::DATA_BLUE);
            }
            if has_j {
                plot_m_theta_end(plot_ui, "j端", &j_pts, theme::PARETO_RED);
            }
        });
}

/// [θ(rad), M(N・mm)] 点列を [θ(rad), M(kN・m)] へ換算して描き、最終点を
/// マーカーで強調する（点と折れ線は同名で登録し凡例エントリを共有する）。
fn plot_m_theta_end(
    plot_ui: &mut egui_plot::PlotUi<'_>,
    name: &str,
    pts: &[[f64; 2]],
    color: egui::Color32,
) {
    if pts.is_empty() {
        return;
    }
    let xy: Vec<[f64; 2]> = pts.iter().map(|p| [p[0], p[1] / 1e6]).collect();
    plot_ui.line(
        egui_plot::Line::new(name, egui_plot::PlotPoints::from(xy.clone()))
            .color(color)
            .width(2.0_f32),
    );
    if let Some(&last) = xy.last() {
        plot_ui.points(
            egui_plot::Points::new(name, egui_plot::PlotPoints::from(vec![last]))
                .color(color)
                .radius(5.0_f32)
                .shape(egui_plot::MarkerShape::Circle),
        );
    }
}

/// N-M 相関図: N-My-Mz 曲面の 3D ワイヤーフレーム（上段）＋従来の 2D スライス
/// （採用曲げ面での正曲げ側・負曲げ側の曲線＋応答経路、下段）を続けて描く。
fn draw_mn_plot(
    ui: &mut egui::Ui,
    cache: &MnCurveCache,
    elem_id: ElemId,
    records: &[MemberStepState],
    bend_dir_z: bool,
    cam: &mut crate::viewer::CameraState,
) {
    draw_mn_plot_3d(ui, cache, records, cam);
    ui.add_space(4.0);
    ui.separator();
    draw_mn_plot_2d(ui, cache, elem_id, records, bend_dir_z);
}

/// N-M 相関図の 3D ワイヤーフレーム（N-My-Mz 曲面）＋ 3D 応答経路。
/// カメラ操作は `mn_view.rs` の断面詳細ビューと同じ（左ドラッグ:回転／
/// 右ドラッグ:移動／スクロール・ピンチ:ズーム、`viewer::CameraState` 共通）。
/// `mn_view.rs` は既存の断面詳細ビュー用のため変更せず、ここでは同じ考え方を
/// 自己完結で再実装する（3D ワイヤーフレーム描画・投影は用途ごとにデータの
/// 正規化基準が異なるため共通化しにくく、重複させた方が安全なため）。
fn draw_mn_plot_3d(
    ui: &mut egui::Ui,
    cache: &MnCurveCache,
    records: &[MemberStepState],
    cam: &mut crate::viewer::CameraState,
) {
    let (rect, response) = ui.allocate_exact_size(
        egui::vec2(ui.available_width(), 260.0),
        egui::Sense::click_and_drag(),
    );
    if response.dragged_by(egui::PointerButton::Primary) {
        let d = response.drag_delta();
        cam.turntable_drag(d.x, d.y);
    }
    if response.dragged_by(egui::PointerButton::Secondary) {
        let d = response.drag_delta();
        cam.pan[0] += d.x;
        cam.pan[1] += d.y;
    }
    if response.hovered() {
        let scroll_y = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll_y != 0.0 {
            cam.zoom *= 1.0 + scroll_y * 0.01;
        }
        let pinch = ui.input(|i| i.zoom_delta());
        if pinch != 1.0 {
            cam.zoom *= pinch;
        }
    }
    cam.zoom = cam.zoom.clamp(0.5, 10.0);

    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, theme::VIEW_BG);
    let screen_center = [rect.center().x, rect.center().y];

    // 正規化基準（曲面自身の耐力基準、ゼロ割防止。`mn_view.rs::draw_3d` と同じ考え方）。
    let n_ref = cache
        .surface
        .n_comp
        .abs()
        .max(cache.surface.n_tens)
        .max(1.0);
    let my_ref = cache.surface.mp_y.abs().max(1.0);
    let mz_ref = cache.surface.mp_z.abs().max(1.0);
    let refs = [my_ref, mz_ref, n_ref];

    let min_dim = rect.width().min(rect.height());
    let scale = 0.32 * min_dim * (cam.zoom / 3.0);

    draw_mn_axes(&painter, cam, scale, screen_center);
    draw_mn_wireframe(&painter, &cache.surface, refs, cam, scale, screen_center);
    draw_mn_response_path_3d(&painter, records, refs, cam, scale, screen_center);

    ui.add(egui::Label::new(
        egui::RichText::new("左ドラッグ:回転 / 右ドラッグ:移動 / スクロール:ズーム").size(11.0),
    ));
}

/// N-My-Mz 曲面をワイヤーフレーム（周方向・経線方向の格子線）で描画する
/// （`mn_view.rs::draw_wireframe` と同じ考え方の自己完結版）。
fn draw_mn_wireframe(
    painter: &egui::Painter,
    surf: &squid_n_section::mn_surface::MnSurface,
    refs: [f64; 3],
    cam: &crate::viewer::CameraState,
    scale: f32,
    screen_center: [f32; 2],
) {
    let center3 = [0.0; 3];
    // 曲面格子点 [N, My, Mz] を正規化ワールド座標 [My_n, Mz_n, N_n] へ変換して投影する。
    let proj = |g: &[f64; 3]| {
        let world = [g[1] / refs[0], g[2] / refs[1], g[0] / refs[2]];
        let p = crate::viewer::project(world, center3, cam, scale, screen_center);
        egui::pos2(p[0], p[1])
    };
    let stroke = egui::Stroke::new(1.0_f32, theme::translucent(theme::DATA_BLUE, 160));

    let n_beta = match surf.grid.first() {
        Some(row) if !row.is_empty() => row.len(),
        _ => return,
    };
    // 周方向（各経線上、j=n_beta-1 と j=0 が接続する閉曲線）
    for row in &surf.grid {
        for j in 0..n_beta {
            let a = proj(&row[j]);
            let b = proj(&row[(j + 1) % n_beta]);
            painter.line_segment([a, b], stroke);
        }
    }
    // 経線方向（引張極→圧縮極）
    for j in 0..n_beta {
        for i in 0..surf.grid.len().saturating_sub(1) {
            let a = proj(&surf.grid[i][j]);
            let b = proj(&surf.grid[i + 1][j]);
            painter.line_segment([a, b], stroke);
        }
    }
}

/// 原点から ±1.3 の座標軸線とラベル「My」「Mz」「N」を描く
/// （`mn_view.rs::draw_axes` と同じ考え方の自己完結版）。
fn draw_mn_axes(
    painter: &egui::Painter,
    cam: &crate::viewer::CameraState,
    scale: f32,
    screen_center: [f32; 2],
) {
    let center3 = [0.0; 3];
    let proj = |p: [f64; 3]| {
        let s = crate::viewer::project(p, center3, cam, scale, screen_center);
        egui::pos2(s[0], s[1])
    };
    const EXT: f64 = 1.3;
    let axes: [([f64; 3], egui::Color32, &str); 3] = [
        ([EXT, 0.0, 0.0], theme::AXIS_X, "My"),
        ([0.0, EXT, 0.0], theme::AXIS_Y, "Mz"),
        ([0.0, 0.0, EXT], theme::AXIS_Z, "N"),
    ];
    for (dir, color, label) in axes {
        let neg = [-dir[0], -dir[1], -dir[2]];
        painter.line_segment([proj(neg), proj(dir)], egui::Stroke::new(1.5_f32, color));
        painter.text(
            proj(dir),
            egui::Align2::LEFT_BOTTOM,
            label,
            egui::FontId::proportional(13.0),
            color,
        );
    }
}

/// 3D 応答経路（原点前置済み）を折れ線＋終点マーカーで描く。
fn draw_mn_response_path_3d(
    painter: &egui::Painter,
    records: &[MemberStepState],
    refs: [f64; 3],
    cam: &crate::viewer::CameraState,
    scale: f32,
    screen_center: [f32; 2],
) {
    let path = n_my_mz_response_path_3d(records);
    // 原点のみ（記録なし）なら描く経路が無い。
    if path.len() < 2 {
        return;
    }
    let center3 = [0.0; 3];
    let proj = |p: &[f64; 3]| {
        let world = [p[0] / refs[0], p[1] / refs[1], p[2] / refs[2]];
        let s = crate::viewer::project(world, center3, cam, scale, screen_center);
        egui::pos2(s[0], s[1])
    };
    let pts: Vec<egui::Pos2> = path.iter().map(proj).collect();
    let stroke = egui::Stroke::new(2.5_f32, theme::PARETO_RED);
    for w in pts.windows(2) {
        painter.line_segment([w[0], w[1]], stroke);
    }
    // 始点(原点=無載荷状態)を明示する。
    painter.circle_stroke(pts[0], 4.0, egui::Stroke::new(1.5_f32, theme::GRAY_600));
    if let Some(&last) = pts.last() {
        painter.circle_filled(last, 5.0, theme::PARETO_RED);
    }
}

/// N-M 相関図の 2D スライス（採用曲げ面での正曲げ側・負曲げ側の曲線＋応答経路）
/// を egui_plot で描く（3D ワイヤーフレームの下段）。
///
/// `Plot` の ID に `elem_id` を含める理由は [`draw_m_theta_plot`] のドキュメント
/// コメントを参照（固定 ID だと部材切替時に前の部材の表示範囲を引き継いでしまう）。
fn draw_mn_plot_2d(
    ui: &mut egui::Ui,
    cache: &MnCurveCache,
    elem_id: ElemId,
    records: &[MemberStepState],
    bend_dir_z: bool,
) {
    let response_path = n_m_response_path(records, bend_dir_z);
    egui_plot::Plot::new(format!("hinge_mn_{}", elem_id.0))
        .x_axis_label("M [kN・m]")
        .y_axis_label("N [kN]（圧縮正）")
        .legend(egui_plot::Legend::default())
        .height(220.0)
        .show(ui, |plot_ui| {
            plot_ui.line(
                egui_plot::Line::new(
                    "N-M 相関(正曲げ側)",
                    egui_plot::PlotPoints::from(cache.pos.clone()),
                )
                .color(theme::GRAY_600)
                .width(1.5_f32),
            );
            plot_ui.line(
                egui_plot::Line::new(
                    "N-M 相関(負曲げ側)",
                    egui_plot::PlotPoints::from(cache.neg.clone()),
                )
                .color(theme::GRAY_300)
                .width(1.5_f32),
            );
            // 応答経路は原点 [0,0]（無載荷状態）を前置済み（`n_m_response_path`）
            // のため、記録が 1 件も無い場合でも要素数は必ず 1 以上になる。
            plot_ui.line(
                egui_plot::Line::new(
                    "応答経路",
                    egui_plot::PlotPoints::from(response_path.clone()),
                )
                .color(theme::DATA_BLUE)
                .width(2.0_f32),
            );
            if let Some(&last) = response_path.last() {
                plot_ui.points(
                    egui_plot::Points::new("応答経路", egui_plot::PlotPoints::from(vec![last]))
                        .color(theme::PARETO_RED)
                        .radius(5.0_f32)
                        .shape(egui_plot::MarkerShape::Circle),
                );
            }
        });
}

/// ヒンジのある端（i端・j端）についてファイバー断面の塑性化マップを横に並べる。
fn draw_fiber_maps(
    ui: &mut egui::Ui,
    elem_id: ElemId,
    sections: &[FiberSectionState],
    mine: &[HingeMarker],
) {
    let want_i = mine.iter().any(|m| !m.end_j);
    let want_j = mine.iter().any(|m| m.end_j);
    ui.horizontal(|ui| {
        if want_i {
            if let Some(sec) = pick_fiber_section(sections, false) {
                ui.vertical(|ui| draw_one_fiber_map(ui, elem_id, "i端", "i", sec));
            }
        }
        if want_j {
            if let Some(sec) = pick_fiber_section(sections, true) {
                ui.vertical(|ui| draw_one_fiber_map(ui, elem_id, "j端", "j", sec));
            }
        }
    });
}

/// ファイバー断面 1 断面分の塑性化マップ（散布図）を描く。`id_suffix` は
/// `egui_plot::Plot` の ID 重複を避けるための識別子（"i"/"j"）。`elem_id` も
/// ID に含める理由は [`draw_m_theta_plot`] のドキュメントコメントを参照
/// （固定 ID だと部材切替時に前の部材のズーム状態を引き継いでしまう）。
fn draw_one_fiber_map(
    ui: &mut egui::Ui,
    elem_id: ElemId,
    end_label: &str,
    id_suffix: &str,
    sec: &FiberSectionState,
) {
    let yielded = sec.fibers.iter().filter(|f| f.yield_ratio >= 1.0).count();
    ui.label(format!(
        "終局時 ξ={:.2}（{}側）／降伏ファイバー {}/{}",
        sec.xi,
        end_label,
        yielded,
        sec.fibers.len()
    ));
    // 材料色の凡例（材料区分ごとに 1 行、色見本＋名称）。
    for &(material, label, _) in FIBER_MATERIALS {
        ui.horizontal(|ui| {
            ui.colored_label(fiber_material_color(material), "■");
            ui.label(label);
        });
    }
    ui.add(egui::Label::new(
        egui::RichText::new("淡色=未降伏／濃色+輪郭=降伏（○:引張降伏 ◇:圧縮降伏）")
            .size(11.0)
            .color(theme::GRAY_600),
    ));
    egui_plot::Plot::new(format!("hinge_fiber_{id_suffix}_{}", elem_id.0))
        .data_aspect(1.0)
        .x_axis_label("y [mm]")
        .y_axis_label("z [mm]")
        .height(220.0)
        .width(220.0)
        .show(ui, |plot_ui| {
            draw_fiber_scatter(plot_ui, &sec.fibers);
        });
}

/// ファイバーの降伏状態による分類（純粋関数）。0=未降伏、1=引張降伏、2=圧縮降伏。
fn fiber_category(f: &FiberStateSample) -> usize {
    if f.yield_ratio < 1.0 {
        0
    } else if f.strain > 0.0 {
        1
    } else {
        2
    }
}

/// ファイバー材料区分（(material 値, 表示名, 塗り円半径)）。
/// `squid_n_element::behavior::FiberStateSample::material` の規約
/// （0=コンクリート／1=主筋（鉄筋）／2=鋼材（形鋼・鋼管・内蔵鉄骨））に対応する。
const FIBER_MATERIALS: &[(usize, &str, f32)] =
    &[(0, "コンクリート", 2.5), (1, "主筋", 4.5), (2, "鋼材", 4.5)];

/// 材料区分から表示色を返す（視認性の良い 3 色: コンクリート=グレー系／
/// 主筋=赤系／鋼材=青系）。未知の区分値はコンクリート（母材）として扱う。
fn fiber_material_color(material: usize) -> egui::Color32 {
    match material {
        1 => theme::PARETO_RED,
        2 => theme::DATA_BLUE,
        _ => theme::GRAY_600,
    }
}

/// ファイバー断面の散布図を材料別・降伏状態別に描く。
///
/// 色は材料区分（コンクリート=グレー系／主筋=赤系／鋼材=青系）で分け、
/// 降伏状態は明度と形状で重ねて表現する: 未降伏=材料色の淡色（小さい円）、
/// 降伏(引張)=材料色そのまま＋外周リング（円）、降伏(圧縮)=材料色そのまま＋
/// 外周リング（ひし形）。主筋・鋼材はコンクリートより大きい円で強調する
/// （点ファイバーであり本数が少ないため、視認性を優先）。
fn draw_fiber_scatter(plot_ui: &mut egui_plot::PlotUi<'_>, fibers: &[FiberStateSample]) {
    for &(material, mat_label, radius) in FIBER_MATERIALS {
        let color = fiber_material_color(material);
        let of_material: Vec<&FiberStateSample> =
            fibers.iter().filter(|f| f.material == material).collect();
        if of_material.is_empty() {
            continue;
        }

        // 未降伏: 材料色の淡色。
        let elastic: Vec<[f64; 2]> = of_material
            .iter()
            .filter(|f| fiber_category(f) == 0)
            .map(|f| [f.y, f.z])
            .collect();
        if !elastic.is_empty() {
            plot_ui.points(
                egui_plot::Points::new(
                    format!("{mat_label}(未降伏)"),
                    egui_plot::PlotPoints::from(elastic),
                )
                .color(theme::lighten(color, 0.55))
                .radius(radius)
                .shape(egui_plot::MarkerShape::Circle),
            );
        }

        // 降伏(引張): 材料色＋外周リング（円）。
        let tension: Vec<[f64; 2]> = of_material
            .iter()
            .filter(|f| fiber_category(f) == 1)
            .map(|f| [f.y, f.z])
            .collect();
        if !tension.is_empty() {
            let name = format!("{mat_label}(降伏・引張)");
            plot_ui.points(
                egui_plot::Points::new(name.clone(), egui_plot::PlotPoints::from(tension.clone()))
                    .color(color)
                    .radius(radius)
                    .shape(egui_plot::MarkerShape::Circle),
            );
            plot_ui.points(
                egui_plot::Points::new(name, egui_plot::PlotPoints::from(tension))
                    .color(theme::GRAY_900)
                    .filled(false)
                    .radius(radius)
                    .shape(egui_plot::MarkerShape::Circle),
            );
        }

        // 降伏(圧縮): 材料色＋外周リング（ひし形。引張降伏と形状で区別）。
        let compression: Vec<[f64; 2]> = of_material
            .iter()
            .filter(|f| fiber_category(f) == 2)
            .map(|f| [f.y, f.z])
            .collect();
        if !compression.is_empty() {
            let name = format!("{mat_label}(降伏・圧縮)");
            plot_ui.points(
                egui_plot::Points::new(
                    name.clone(),
                    egui_plot::PlotPoints::from(compression.clone()),
                )
                .color(color)
                .radius(radius)
                .shape(egui_plot::MarkerShape::Diamond),
            );
            plot_ui.points(
                egui_plot::Points::new(name, egui_plot::PlotPoints::from(compression))
                    .color(theme::GRAY_900)
                    .filled(false)
                    .radius(radius)
                    .shape(egui_plot::MarkerShape::Diamond),
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// テスト用のヒンジ発生イベントを組み立てる。
    fn event(elem: u32, pos: f64, level: HingeLevel, ductility: f64, step: u32) -> HingeEvent {
        HingeEvent {
            step,
            elem: ElemId(elem),
            pos,
            level,
            ductility,
        }
    }

    /// 同一 (部材, 端) の重複記録（複数ステップに渡る push）は 1 件に集約される。
    #[test]
    fn aggregate_hinges_dedups_same_elem_and_end() {
        let hinges = vec![
            event(0, 0.0, HingeLevel::Crack, 1.0, 1),
            event(0, 0.0, HingeLevel::Crack, 1.2, 2),
            event(0, 0.0, HingeLevel::Crack, 1.5, 3),
        ];
        let markers = aggregate_hinges(&hinges);
        assert_eq!(markers.len(), 1);
    }

    /// 集約後は最高レベル（Crack < Yield < Ultimate）と、全ステップ中の最大塑性率が採用される。
    #[test]
    fn aggregate_hinges_picks_highest_level_and_max_ductility() {
        let hinges = vec![
            event(0, 0.0, HingeLevel::Crack, 1.0, 1),
            event(0, 0.0, HingeLevel::Yield, 1.5, 2),
            event(0, 0.0, HingeLevel::Crack, 1.8, 3),
        ];
        let markers = aggregate_hinges(&hinges);
        assert_eq!(markers.len(), 1);
        assert!(matches!(markers[0].level, HingeLevel::Yield));
        assert!((markers[0].max_ductility - 1.8).abs() < 1e-9);
    }

    /// 初めてヒンジが記録された step（最小 step）が保持される。
    #[test]
    fn aggregate_hinges_keeps_min_step_as_first_step() {
        let hinges = vec![
            event(0, 0.0, HingeLevel::Yield, 1.0, 5),
            event(0, 0.0, HingeLevel::Yield, 1.2, 3),
            event(0, 0.0, HingeLevel::Yield, 1.1, 8),
        ];
        let markers = aggregate_hinges(&hinges);
        assert_eq!(markers.len(), 1);
        assert_eq!(markers[0].first_step, 3);
    }

    /// pos<0.5 は i端、pos≥0.5 は j端として分離集計される。
    #[test]
    fn aggregate_hinges_separates_by_end() {
        let hinges = vec![
            event(0, 0.2, HingeLevel::Crack, 1.0, 1),
            event(0, 0.8, HingeLevel::Yield, 1.0, 1),
            event(0, 0.5, HingeLevel::Ultimate, 1.0, 1),
        ];
        let markers = aggregate_hinges(&hinges);
        assert_eq!(markers.len(), 2);
        let i_end = markers.iter().find(|m| !m.end_j).unwrap();
        let j_end = markers.iter().find(|m| m.end_j).unwrap();
        assert!(matches!(i_end.level, HingeLevel::Crack));
        // pos=0.5 は j端側（0.5 は「境界」で j端扱い）に含まれ、Ultimate が最高レベルとして残る。
        assert!(matches!(j_end.level, HingeLevel::Ultimate));
    }

    /// 異なる部材は分離して集計される。
    #[test]
    fn aggregate_hinges_separates_by_elem() {
        let hinges = vec![
            event(0, 0.0, HingeLevel::Crack, 1.0, 1),
            event(1, 0.0, HingeLevel::Yield, 1.0, 1),
        ];
        let markers = aggregate_hinges(&hinges);
        assert_eq!(markers.len(), 2);
        assert!(markers.iter().any(|m| m.elem == ElemId(0)));
        assert!(markers.iter().any(|m| m.elem == ElemId(1)));
    }

    /// 空入力は空の集計結果を返す。
    #[test]
    fn aggregate_hinges_empty_input() {
        let markers = aggregate_hinges(&[]);
        assert!(markers.is_empty());
    }

    // ── dominant_bend_axis_z ────────────────────────────────────────────

    fn step(mz_i: f32, mz_j: f32, my_i: f32, my_j: f32, n: f32) -> MemberStepState {
        MemberStepState {
            n,
            my_i,
            mz_i,
            my_j,
            mz_j,
            ry_i: 0.0,
            rz_i: 0.0,
            ry_j: 0.0,
            rz_j: 0.0,
        }
    }

    /// 強軸(Mz)成分の絶対値が大きければ強軸を採用する。
    #[test]
    fn dominant_bend_axis_z_picks_larger_axis() {
        let s = step(50.0, -10.0, 5.0, 5.0, 0.0);
        assert!(dominant_bend_axis_z(&s));
    }

    /// 弱軸(My)成分の絶対値が大きければ弱軸を採用する。
    #[test]
    fn dominant_bend_axis_z_picks_weak_axis_when_larger() {
        let s = step(5.0, 5.0, 50.0, -10.0, 0.0);
        assert!(!dominant_bend_axis_z(&s));
    }

    /// 同値なら強軸を採用する（`>=` 判定）。
    #[test]
    fn dominant_bend_axis_z_ties_favor_strong_axis() {
        let s = step(10.0, 0.0, 10.0, 0.0, 0.0);
        assert!(dominant_bend_axis_z(&s));
    }

    // ── m_theta_series ──────────────────────────────────────────────────

    /// 2 点列の近似一致（レコードは f32 格納のため f64 リテラルとの完全一致は
    /// 期待できない。f32 の丸め幅を許容する）。
    fn assert_pts_near(actual: &[[f64; 2]], expected: &[[f64; 2]]) {
        assert_eq!(actual.len(), expected.len(), "点数が一致すること");
        for (a, e) in actual.iter().zip(expected) {
            for k in 0..2 {
                assert!(
                    (a[k] - e[k]).abs() <= e[k].abs().max(1.0) * 1e-6,
                    "{:?} ≈ {:?}",
                    actual,
                    expected
                );
            }
        }
    }

    /// 強軸採用時は rz/mz を絶対値化して抽出する。
    #[test]
    fn m_theta_series_extracts_strong_axis_abs_values() {
        let records = vec![step(100.0, -50.0, 1.0, 1.0, 0.0)];
        // rz_i, rz_j に値を入れたいので直接構築する。
        let mut records = records;
        records[0].rz_i = -0.01;
        records[0].rz_j = 0.02;
        let (i_pts, j_pts) = m_theta_series(&records, true);
        assert_pts_near(&i_pts, &[[0.01, 100.0]]);
        assert_pts_near(&j_pts, &[[0.02, 50.0]]);
    }

    /// 弱軸採用時は ry/my を抽出する。
    #[test]
    fn m_theta_series_extracts_weak_axis_when_selected() {
        let mut records = vec![step(0.0, 0.0, 30.0, -20.0, 0.0)];
        records[0].ry_i = 0.005;
        records[0].ry_j = -0.006;
        let (i_pts, j_pts) = m_theta_series(&records, false);
        assert_pts_near(&i_pts, &[[0.005, 30.0]]);
        assert_pts_near(&j_pts, &[[0.006, 20.0]]);
    }

    /// 空入力は空の点列を返す。
    #[test]
    fn m_theta_series_empty_input() {
        let (i_pts, j_pts) = m_theta_series(&[], true);
        assert!(i_pts.is_empty());
        assert!(j_pts.is_empty());
    }

    // ── n_m_response_path ───────────────────────────────────────────────

    /// 先頭に原点 [0,0]（無載荷状態）が前置される。
    #[test]
    fn n_m_response_path_prepends_origin() {
        let records = vec![step(-80.0, 30.0, 0.0, 0.0, 1000.0)];
        let path = n_m_response_path(&records, true);
        assert_eq!(path.len(), 2);
        assert_eq!(path[0], [0.0, 0.0]);
    }

    /// 強軸採用時、i端・j端のうち絶対値が大きい方の符号付き Mz を採用する。
    #[test]
    fn n_m_response_path_picks_larger_end_signed() {
        let records = vec![step(-80.0, 30.0, 0.0, 0.0, 1000.0)];
        let path = n_m_response_path(&records, true);
        assert_eq!(path.len(), 2);
        // -80 N·mm -> -80/1e6 kN·m、n=1000N -> 1kN。原点の次（[1]）が実データ点。
        assert!((path[1][0] - (-80.0 / 1e6)).abs() < 1e-12);
        assert!((path[1][1] - 1.0).abs() < 1e-9);
    }

    /// 弱軸採用時は my_i/my_j を対象にする。
    #[test]
    fn n_m_response_path_uses_weak_axis_when_selected() {
        let records = vec![step(0.0, 0.0, 40.0, -90.0, 0.0)];
        let path = n_m_response_path(&records, false);
        assert!((path[1][0] - (-90.0 / 1e6)).abs() < 1e-12);
    }

    /// 空入力でも原点 1 点だけは返る。
    #[test]
    fn n_m_response_path_empty_input_returns_origin_only() {
        let path = n_m_response_path(&[], true);
        assert_eq!(path, vec![[0.0, 0.0]]);
    }

    // ── n_my_mz_response_path_3d ────────────────────────────────────────

    /// 先頭に原点 [0,0,0] が前置され、各ステップは合成曲げの大きい方の端を採用する。
    /// N は member_history の圧縮正から引張正（曲面の規約）へ符号反転する。
    #[test]
    fn n_my_mz_response_path_3d_prepends_origin_and_picks_larger_end() {
        // i端の合成曲げ = sqrt(50^2+0^2)=50、j端 = sqrt(0^2+80^2)=80 → j端を採用。
        let records = vec![step(0.0, 0.0, 50.0, 80.0, 1000.0)];
        let path = n_my_mz_response_path_3d(&records);
        assert_eq!(path.len(), 2);
        assert_eq!(path[0], [0.0, 0.0, 0.0]);
        assert!((path[1][0] - 80.0).abs() < 1e-9);
        assert!((path[1][2] - (-1000.0)).abs() < 1e-9);
    }

    // ── mn_beta_columns ─────────────────────────────────────────────────

    /// 弱軸(My)は β=0（j=0）・β=π（j=n_beta/2）を採用する。
    #[test]
    fn mn_beta_columns_weak_axis() {
        assert_eq!(mn_beta_columns(48, false), (0, 24));
    }

    /// 強軸(Mz)は β=π/2（j=n_beta/4）・β=3π/2（j=3n_beta/4）を採用する。
    #[test]
    fn mn_beta_columns_strong_axis() {
        assert_eq!(mn_beta_columns(48, true), (12, 36));
    }

    // ── extract_mn_meridian ─────────────────────────────────────────────

    /// 弱軸(My)は grid[i][j][1]を抽出し、N の符号を反転（引張正→圧縮正）する。
    #[test]
    fn extract_mn_meridian_weak_axis_flips_n_sign() {
        // grid[i][beta_col] = [N, My, Mz]（引張正の N 規約）。
        let grid = vec![
            vec![[1.0e6, 2.0e6, 0.0]],  // N=1e6(引張), My=2e6
            vec![[-3.0e6, 4.0e6, 0.0]], // N=-3e6(圧縮), My=4e6
        ];
        let pts = extract_mn_meridian(&grid, 0, false);
        assert_eq!(pts.len(), 2);
        // M[kN・m] = My/1e6, N[kN] = -N/1e3（圧縮正へ変換）。
        assert!((pts[0][0] - 2.0).abs() < 1e-9);
        assert!((pts[0][1] - (-1.0e3)).abs() < 1e-6);
        assert!((pts[1][0] - 4.0).abs() < 1e-9);
        assert!((pts[1][1] - 3.0e3).abs() < 1e-6);
    }

    /// 強軸(Mz)は grid[i][j][2] を抽出する。
    #[test]
    fn extract_mn_meridian_strong_axis_uses_mz() {
        let grid = vec![vec![[0.0, 100.0, 200.0]]];
        let pts = extract_mn_meridian(&grid, 0, true);
        assert_eq!(pts.len(), 1);
        assert!((pts[0][0] - 200.0 / 1e6).abs() < 1e-12);
    }

    /// 列が範囲外の行は無視する（`row.get` が `None` を返す行はスキップ）。
    #[test]
    fn extract_mn_meridian_skips_out_of_range_rows() {
        let grid = vec![vec![[0.0, 1.0, 2.0]], vec![]];
        let pts = extract_mn_meridian(&grid, 0, false);
        assert_eq!(pts.len(), 1);
    }

    // ── is_axial_bending_member ─────────────────────────────────────────

    fn make_model_with_column() -> (ElementData, Model) {
        use smallvec::smallvec;
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::NodeId;
        use squid_n_core::model::{EndCondition, ForceRegime, LocalAxis, Node, RigidZone};

        let n0 = Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        };
        let n1 = Node {
            id: NodeId(1),
            coord: [0.0, 0.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        };
        let model = Model {
            nodes: vec![n0, n1],
            ..Default::default()
        };
        let elem = ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(0), NodeId(1)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        };
        (elem, model)
    }

    /// 両端節点の水平距離が 1mm 未満の Beam（柱）は軸力を受ける部材とみなす。
    #[test]
    fn is_axial_bending_member_vertical_beam_is_true() {
        let (elem, model) = make_model_with_column();
        assert!(is_axial_bending_member(&elem, &model));
    }

    /// 水平材（梁）は Beam 種別では対象外。
    #[test]
    fn is_axial_bending_member_horizontal_beam_is_false() {
        let (mut elem, mut model) = make_model_with_column();
        model.nodes[1].coord = [3000.0, 0.0, 0.0];
        elem.nodes =
            smallvec::smallvec![squid_n_core::ids::NodeId(0), squid_n_core::ids::NodeId(1)];
        assert!(!is_axial_bending_member(&elem, &model));
    }

    /// ElementKind が Fiber/MultiSpring/Brace なら向きに依らず対象。
    #[test]
    fn is_axial_bending_member_fiber_kind_is_always_true() {
        let (mut elem, mut model) = make_model_with_column();
        model.nodes[1].coord = [3000.0, 0.0, 0.0]; // 水平材
        elem.kind = ElementKind::Fiber;
        assert!(is_axial_bending_member(&elem, &model));

        elem.kind = ElementKind::MultiSpring;
        assert!(is_axial_bending_member(&elem, &model));

        elem.kind = ElementKind::Brace {
            tension_only: false,
        };
        assert!(is_axial_bending_member(&elem, &model));
    }

    // ── pick_fiber_section ──────────────────────────────────────────────

    fn fiber_section(xi: f64) -> FiberSectionState {
        FiberSectionState { xi, fibers: vec![] }
    }

    /// i端（end_j=false）は最小 xi の断面を選ぶ。
    #[test]
    fn pick_fiber_section_i_end_picks_min_xi() {
        let sections = vec![fiber_section(0.2), fiber_section(-0.9), fiber_section(0.9)];
        let picked = pick_fiber_section(&sections, false).unwrap();
        assert!((picked.xi - (-0.9)).abs() < 1e-9);
    }

    /// j端（end_j=true）は最大 xi の断面を選ぶ。
    #[test]
    fn pick_fiber_section_j_end_picks_max_xi() {
        let sections = vec![fiber_section(0.2), fiber_section(-0.9), fiber_section(0.9)];
        let picked = pick_fiber_section(&sections, true).unwrap();
        assert!((picked.xi - 0.9).abs() < 1e-9);
    }

    /// 空入力は `None`。
    #[test]
    fn pick_fiber_section_empty_input() {
        assert!(pick_fiber_section(&[], false).is_none());
        assert!(pick_fiber_section(&[], true).is_none());
    }

    // ── fiber_category ──────────────────────────────────────────────────

    fn fiber_sample(strain: f64, yield_ratio: f64, material: usize) -> FiberStateSample {
        FiberStateSample {
            y: 0.0,
            z: 0.0,
            area: 1.0,
            strain,
            yield_ratio,
            material,
        }
    }

    /// 降伏比 1.0 未満は未降伏(0)。
    #[test]
    fn fiber_category_elastic() {
        assert_eq!(fiber_category(&fiber_sample(0.001, 0.5, 0)), 0);
    }

    /// 降伏比 1.0 以上・引張ひずみは引張降伏(1)。
    #[test]
    fn fiber_category_tension_yield() {
        assert_eq!(fiber_category(&fiber_sample(0.01, 1.2, 0)), 1);
    }

    /// 降伏比 1.0 以上・圧縮ひずみは圧縮降伏(2)。
    #[test]
    fn fiber_category_compression_yield() {
        assert_eq!(fiber_category(&fiber_sample(-0.01, 1.2, 0)), 2);
    }
}
