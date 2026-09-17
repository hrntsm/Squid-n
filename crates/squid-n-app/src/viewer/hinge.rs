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
use std::time::SystemTime;

use crate::app::{App, Staleness};
use crate::theme;
use crate::viewer::mn_draw;
use squid_n_core::ids::{ElemId, MaterialId};
use squid_n_core::model::{
    AnalysisKind, ElementData, ElementKind, ForceRegime, HysteresisModel, Material,
    MaterialCategory, Model, RigidZone, Section,
};
use squid_n_core::section_shape::SectionShape;
use squid_n_core::units::to_display::{force_kn, moment_kn_m};
use squid_n_core::units::ConcreteClass;
use squid_n_element::behavior::{FiberSectionState, FiberStateSample};
use squid_n_element::factory::{
    build_hinge_view, resolve_fiber_concrete_hysteresis, resolve_force_regime,
    resolve_member_hysteresis, AnalysisHingeModel, HingeView, ResolvedRegime, StrengthBasis,
};
use squid_n_element::frame::concentrated::MnInteraction;
use squid_n_element::wall::side_column::wall_side_column_release;
use squid_n_element::wall::wall_element::wall_element_geometry;
use squid_n_section::mn_surface::MnSurface;
use squid_n_solver::nonlinear::pushover::{HingeEvent, HingeLevel, MemberStepState};

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

/// ヒンジマーカー描画位置（材端から [`INSET_T`] 内側）のスクリーン座標。
/// 耐震壁は壁柱（上下辺中点）を材軸とする。
pub(super) fn hinge_marker_screen_pos(
    elem: &ElementData,
    model: &Model,
    pts: &[egui::Pos2],
    proj: &super::Projector<'_>,
    end_j: bool,
) -> Option<egui::Pos2> {
    if matches!(elem.kind, ElementKind::Wall) && elem.nodes.len() >= 4 {
        let g = wall_element_geometry(elem, model)?;
        let bc = proj.project(g.bottom_center);
        let tc = proj.project(g.top_center);
        let t = if end_j { 1.0 - INSET_T } else { INSET_T };
        return Some(egui::pos2(
            bc.x + (tc.x - bc.x) * t,
            bc.y + (tc.y - bc.y) * t,
        ));
    }
    if elem.nodes.len() < 2 {
        return None;
    }
    let n0 = elem.nodes[0].index();
    let n1 = elem.nodes[1].index();
    if n0 >= pts.len() || n1 >= pts.len() {
        return None;
    }
    let (p0, p1) = (pts[n0], pts[n1]);
    let t = if end_j { 1.0 - INSET_T } else { INSET_T };
    Some(egui::pos2(
        p0.x + (p1.x - p0.x) * t,
        p0.y + (p1.y - p0.y) * t,
    ))
}

/// ヒンジ図を描く。`pts` は `viewer_panel` で計算済みの節点スクリーン座標
/// （`app.core.model.nodes` と同じ順序）。
pub(super) fn draw_hinge(
    painter: &egui::Painter,
    app: &App,
    model: &Model,
    pts: &[egui::Pos2],
    proj: &super::Projector<'_>,
    frame_filter: super::FrameFilter,
) {
    let Some(po) = app.displayed_pushover() else {
        draw_no_result_legend(painter);
        return;
    };

    let markers = aggregate_hinges(&po.hinges);
    let mut counts = [0usize; 3];

    for m in &markers {
        if !frame_filter.shows(m.elem) {
            continue;
        }
        let Some(elem) = model.element(m.elem) else {
            continue;
        };
        let Some(center) = hinge_marker_screen_pos(elem, model, pts, proj, m.end_j) else {
            continue;
        };

        let color = hinge_color(&m.level);
        counts[level_rank(&m.level) as usize] += 1;

        painter.circle_filled(center, MARKER_R, color);
        if matches!(m.level, HingeLevel::Ultimate) {
            painter.circle_stroke(center, MARKER_R + 2.5, egui::Stroke::new(1.5_f32, color));
        }
    }

    draw_hinge_legend(painter, &counts);
}

/// ヒンジ図のホバー詳細ツールチップ。部材 `elem_id` にヒンジがあれば
/// i端／j端それぞれの最高レベル・最大塑性率・初出 step を表示する。
/// ヒンジのない部材は何も表示しない。
pub(super) fn show_hinge_tooltip(ui: &egui::Ui, app: &App, elem_id: ElemId) {
    let Some(po) = app.displayed_pushover() else {
        return;
    };
    let markers = aggregate_hinges(&po.hinges);
    let mut rows: Vec<&HingeMarker> = markers.iter().filter(|m| m.elem == elem_id).collect();
    if rows.is_empty() {
        return;
    }
    rows.sort_by_key(|m| m.end_j);

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

/// ヒンジ詳細ウィンドウの表示ビューキャッシュ。キーが一致する限り、ファイバー
/// 分割・曲面構築を含む [`build_hinge_view`] を再実行しない。
pub struct HingeViewCache {
    key: HingeViewKey,
    view: HingeView,
}

impl HingeViewCache {
    /// 保存時のキー。テスト用に公開する。
    #[cfg(test)]
    fn key(&self) -> &HingeViewKey {
        &self.key
    }
}

/// [`HingeViewCache`] のキー。骨格・曲面の生成に影響する入力を一意に識別する。
///
/// 同一ステップ数で再解析した場合も `generation`（`staleness.last_run` と
/// `staleness.results_stale`）で無効化され、断面・材料・履歴則の編集は
/// `section`・`material`・`hysteresis` のフィンガープリントで無効化される。
///
/// 採用曲げ面（[`effective_bend_dir_z`]）は [`HingeView`] を入力に取らないため
/// キーには含めない。表示時にキャッシュ済みのビューから決める。
#[derive(Clone, PartialEq, Debug)]
struct HingeViewKey {
    elem: ElemId,
    /// キャッシュが生成された選択ステップ添字（`records` 上の位置）。
    /// 非集中ばねは軸力に依存せず選択ステップにも依らないため常に 0。
    step: usize,
    /// 解析結果の世代（最終実行時刻と要再計算フラグ）。
    generation: (Option<SystemTime>, bool),
    kind: ElementKind,
    force_regime: ForceRegime,
    rigid_zone: RigidZone,
    plastic_zone: Option<f64>,
    section: Option<SectionFingerprint>,
    material: Option<MaterialFingerprint>,
    rebar_material: Option<MaterialFingerprint>,
    steel_material: Option<MaterialFingerprint>,
    /// 材端集中ばねの履歴則（[`resolve_member_hysteresis`] の解決値）。
    hysteresis: HysteresisModel,
    /// 材端集中ばねが N-M 相関を用いるか（`HingeView::mn_linear` の有無に対応）。
    use_mn: bool,
}

/// 断面のうち骨格・曲面生成に用いるフィールド。
#[derive(Clone, PartialEq, Debug)]
struct SectionFingerprint {
    shape: Option<SectionShape>,
    area: f64,
    iy: f64,
    iz: f64,
    j: f64,
    depth: f64,
    width: f64,
    as_y: f64,
    as_z: f64,
    panel_thickness: Option<f64>,
    thickness: Option<f64>,
    material: Option<MaterialId>,
    rebar_material: Option<MaterialId>,
    shear_rebar_material: Option<MaterialId>,
    steel_material: Option<MaterialId>,
}

impl From<&Section> for SectionFingerprint {
    fn from(s: &Section) -> Self {
        SectionFingerprint {
            shape: s.shape.clone(),
            area: s.area,
            iy: s.iy,
            iz: s.iz,
            j: s.j,
            depth: s.depth,
            width: s.width,
            as_y: s.as_y,
            as_z: s.as_z,
            panel_thickness: s.panel_thickness,
            thickness: s.thickness,
            material: s.material,
            rebar_material: s.rebar_material,
            shear_rebar_material: s.shear_rebar_material,
            steel_material: s.steel_material,
        }
    }
}

/// 材料のうち骨格・曲面生成に用いるフィールド。
#[derive(Clone, PartialEq, Debug)]
struct MaterialFingerprint {
    name: String,
    category: MaterialCategory,
    young: f64,
    poisson: f64,
    fc: Option<f64>,
    fy: Option<f64>,
    strength_factor: Option<f64>,
    concrete_class: ConcreteClass,
}

impl From<&Material> for MaterialFingerprint {
    fn from(m: &Material) -> Self {
        MaterialFingerprint {
            name: m.name.clone(),
            category: m.category,
            young: m.young,
            poisson: m.poisson,
            fc: m.fc,
            fy: m.fy,
            strength_factor: m.strength_factor,
            concrete_class: m.concrete_class,
        }
    }
}

/// 材端集中ばねが N-M 線形相関を用いる履歴則か。
///
/// [`build_hinge_view`] が返す `mn_linear` の有無と一致する（標準型・辻山田型・
/// 座屈考慮型は `set_yield` 対応、履歴材料は非対応）。キャッシュキーの識別にのみ
/// 用い、表示の可否は `HingeView::mn_linear` から判定する。
fn concentrated_uses_mn(rule: HysteresisModel) -> bool {
    matches!(
        rule,
        HysteresisModel::Standard | HysteresisModel::TsujiYamada | HysteresisModel::SteelBuckling
    )
}

/// [`build_hinge_view`] が材端集中ばねを返す要素か（要素生成と同じ判定）。
///
/// 集中ばねだけが選択ステップの軸力で M-θ 骨格を変えるため、キャッシュキーへ
/// ステップを含めるかの判定に用いる。壁側柱（面内解放）は集中ばねではない。
fn is_concentrated_spring(model: &Model, elem: &ElementData) -> bool {
    elem.kind == ElementKind::Beam
        && wall_side_column_release(elem, model).is_none()
        && matches!(
            resolve_force_regime(elem, model),
            ResolvedRegime::ConcentratedSpring
        )
}

/// 骨格・曲面生成に影響する入力からキャッシュキーを組み立てる（純粋関数）。
///
/// 強度基準は [`StrengthBasis::MaterialStrength`]、解析種別は
/// [`AnalysisKind::Incremental`] 固定（プッシュオーバー結果の表示）のため、
/// キーには含めない。
fn hinge_view_key(
    model: &Model,
    elem: &ElementData,
    step: usize,
    staleness: &Staleness,
) -> HingeViewKey {
    let rule = resolve_member_hysteresis(elem, model, AnalysisKind::Incremental);
    let section = elem.section.and_then(|sid| model.sections.get(sid.index()));
    HingeViewKey {
        elem: elem.id,
        step: if is_concentrated_spring(model, elem) {
            step
        } else {
            0
        },
        generation: (staleness.last_run, staleness.results_stale),
        kind: elem.kind,
        force_regime: elem.force_regime,
        rigid_zone: elem.rigid_zone,
        plastic_zone: elem.plastic_zone,
        section: section.map(SectionFingerprint::from),
        material: model.element_material(elem).map(MaterialFingerprint::from),
        rebar_material: model
            .element_rebar_material(elem)
            .map(MaterialFingerprint::from),
        steel_material: model
            .element_steel_material(elem)
            .map(MaterialFingerprint::from),
        hysteresis: rule,
        use_mn: concentrated_uses_mn(rule),
    }
}

/// キーがキャッシュと一致しなければ [`build_hinge_view`] で再生成する。
fn ensure_hinge_view(app: &mut App, key: HingeViewKey, elem: &ElementData, axial_force_n: f64) {
    if app
        .ui
        .scoped
        .hinge_view_cache
        .as_ref()
        .is_some_and(|c| c.key == key)
    {
        return;
    }
    let view = build_hinge_view(
        elem,
        &app.core.model,
        StrengthBasis::MaterialStrength,
        AnalysisKind::Incremental,
        axial_force_n,
        mn_draw::N_ALPHA,
        mn_draw::N_BETA,
    );
    app.ui.scoped.hinge_view_cache = Some(HingeViewCache { key, view });
}

/// 部材の最終応答レコードから、支配的な曲げ面（強軸 Mz／弱軸 My）を選ぶ
/// （純粋関数）。i端・j端のうち絶対値が大きい方の成分を軸ごとに比較し、
/// 大きい軸を採用する（同値なら強軸を採用）。
///
/// ファイバー／マルチスプリングの採用曲げ面（[`effective_bend_dir_z`]）を決める
/// ために用いる。材端集中ばねは常に強軸へ固定するため、この関数は使わない。
pub(super) fn dominant_bend_axis_z(last: &MemberStepState) -> bool {
    let mz_max = last.mz_i.abs().max(last.mz_j.abs());
    let my_max = last.my_i.abs().max(last.my_j.abs());
    mz_max >= my_max
}

/// ヒンジ詳細で表示する採用曲げ面（強軸 Mz=true）を決める（純粋関数）。
///
/// 材端集中ばねの非線形ばねは局所 z 回り（`Mz`）のみに作用し、弱軸（`My`）は弾性。
/// N-M 線形相関も `Mz` に対する `My0` を用いるため、応答が弱軸支配でも表示は
/// 強軸に固定する（`dominant_z` を無視する）。
/// ファイバー／マルチスプリングは My・Mz の両軸をカバーする曲面を表示するため、
/// 最終ステップの支配軸 `dominant_z`（[`dominant_bend_axis_z`]）に従う。
fn effective_bend_dir_z(model: AnalysisHingeModel, dominant_z: bool) -> bool {
    match model {
        AnalysisHingeModel::ConcentratedSpring => true,
        AnalysisHingeModel::Fiber | AnalysisHingeModel::MultiSpring | AnalysisHingeModel::Other => {
            dominant_z
        }
    }
}

/// 採用軸に応じた i端・j端の (|θ|[rad], |M|[N·mm]) 点列を全ステップから
/// 抽出する（純粋関数）。M は剛域フェイス位置の局所曲げ。
///
/// θ はファイバー・マルチスプリングでは弦からの材端回転、材端集中ばねでは
/// 解析が実際に使う端ばね変形（`use_spring_rot=true`。記録の
/// `spring_rz_i`／`spring_rz_j`）を用いる。集中ばねの M-θ 骨格は端ばね変形に
/// 対して定義されるため、応答経路と基準を揃える。端ばね変形が記録されていない
/// 場合は弦からの材端回転へフォールバックする（強軸のみ。弱軸は端ばねを持たない）。
pub(super) fn m_theta_series(
    records: &[MemberStepState],
    bend_dir_z: bool,
    use_spring_rot: bool,
) -> (Vec<[f64; 2]>, Vec<[f64; 2]>) {
    let theta = |r: &MemberStepState, end_j: bool| -> f64 {
        if bend_dir_z {
            if use_spring_rot {
                let g = if end_j { r.spring_rz_j } else { r.spring_rz_i };
                if let Some(g) = g {
                    return g as f64;
                }
            }
            if end_j {
                r.rz_j as f64
            } else {
                r.rz_i as f64
            }
        } else if end_j {
            r.ry_j as f64
        } else {
            r.ry_i as f64
        }
    };
    let i_pts = records
        .iter()
        .map(|r| {
            let m = if bend_dir_z { r.mz_i } else { r.my_i };
            [theta(r, false).abs(), m.abs() as f64]
        })
        .collect();
    let j_pts = records
        .iter()
        .map(|r| {
            let m = if bend_dir_z { r.mz_j } else { r.my_j };
            [theta(r, true).abs(), m.abs() as f64]
        })
        .collect();
    (i_pts, j_pts)
}

/// 採用軸の端最大 |M|（絶対値が大きい方の端の符号付き値）と軸力から、
/// 応答経路 [M(kN·m), N(kN、圧縮正)] を全ステップ抽出する（純粋関数）。
/// `member_history` の軸力は既に圧縮正のため、N-M 曲線側の符号変換のみで
/// 済む（[`extract_mn_meridian`] 参照）。
///
/// 先頭に原点 [0.0, 0.0]（無載荷状態）を前置する。`member_history` の記録は
/// 最初の記録ステップから始まるため、これがないと経路の始点が分からない。
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
        [moment_kn_m(m as f64), force_kn(r.n as f64)]
    }));
    path
}

/// 3D 表示用の応答経路 [My(N·mm,符号付き), Mz(N·mm,符号付き), N(N,引張正)] を
/// 全ステップ抽出する（純粋関数）。各ステップで i端・j端のうち合成曲げ
/// （√(My²+Mz²)）が大きい方の端を採用する（[`n_m_response_path`] は採用軸
/// 1 成分のみを追うのに対し、3D 表示は My・Mz の両成分をそのまま使える）。
///
/// N-M 曲面 [`MnSurface`] は引張正の N 規約のため、圧縮正の
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
/// ため符号を反転して揃える。単位は表示用に [kN·m]・[kN] へ換算する。
pub(super) fn extract_mn_meridian(
    grid: &[Vec<[f64; 3]>],
    beta_col: usize,
    bend_dir_z: bool,
) -> Vec<[f64; 2]> {
    let m_index = if bend_dir_z { 2 } else { 1 };
    grid.iter()
        .filter_map(|row| row.get(beta_col))
        .map(|p| [moment_kn_m(p[m_index]), -force_kn(p[0])])
        .collect()
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

/// ヒンジ詳細ウィンドウ（クリックで開く）。`app.ui.scoped.hinge_detail_elem` が `None`
/// なら何も描かない。閉じるボタン（×）で `app.ui.scoped.hinge_detail_elem` をクリアする。
pub(crate) fn show_hinge_detail_window(ui: &egui::Ui, app: &mut App) {
    let Some(elem_id) = app.ui.scoped.hinge_detail_elem else {
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
        app.ui.scoped.hinge_detail_elem = None;
        app.ui.scoped.hinge_view_cache = None;
        app.ui.scoped.hinge_step = None;
    }
}

/// ヒンジ詳細ウィンドウの中身。共通ヘッダ（i端/j端の集約ヒンジ情報）と解析モデルの
/// 表示に続けて、表示ステップの選択、M-θ カーブ、N-M 相関図、ファイバー断面の
/// 塑性化マップを該当するものだけ縦に並べる。
fn draw_hinge_detail_content(ui: &mut egui::Ui, app: &mut App, elem_id: ElemId) {
    let (elem_snapshot, elem_section) = {
        let display = super::wall_expanded_view_model(&app.core.model);
        let Some(elem) = display.element(elem_id) else {
            ui.colored_label(theme::GRAY_600, "この部材はモデルから削除されています。");
            return;
        };
        if matches!(elem.kind, ElementKind::Wall) {
            ui.label("耐震壁（壁版から生成された解析要素）");
            ui.separator();
        }
        (elem.clone(), elem.section)
    };

    let Some(po) = app.displayed_pushover() else {
        ui.colored_label(theme::GRAY_600, "増分解析が未実行です。");
        return;
    };

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
    let fiber_sections: Option<Vec<FiberSectionState>> = po
        .fiber_states
        .iter()
        .find(|(id, _)| *id == elem_id)
        .map(|(_, s)| s.clone());
    let dominant_z = dominant_bend_axis_z(&records[records.len() - 1]);

    let step = hinge_step_selector(ui, app, &records);
    let axial_force_n = records[step].n as f64;
    let key = hinge_view_key(
        &app.core.model,
        &elem_snapshot,
        step,
        &app.core.scoped.staleness,
    );
    ensure_hinge_view(app, key, &elem_snapshot, axial_force_n);
    let view = app
        .ui
        .scoped
        .hinge_view_cache
        .as_ref()
        .map(|c| &c.view)
        .expect("ensure_hinge_view がキャッシュを設定する");
    let bend_dir_z = effective_bend_dir_z(view.model, dominant_z);

    ui.label(format!("解析モデル: {}", analysis_model_label(view.model)));
    let rule_note = match view.model {
        AnalysisHingeModel::ConcentratedSpring => Some(format!(
            "履歴則: {}",
            resolve_member_hysteresis(&elem_snapshot, &app.core.model, AnalysisKind::Incremental)
                .label()
        )),
        AnalysisHingeModel::Fiber | AnalysisHingeModel::MultiSpring => Some(format!(
            "コンクリート除荷則: {}",
            resolve_fiber_concrete_hysteresis(
                &elem_snapshot,
                &app.core.model,
                AnalysisKind::Incremental
            )
            .label()
        )),
        AnalysisHingeModel::Other => None,
    };
    if let Some(note) = rule_note {
        ui.label(note);
    }
    let bend_face_label = match view.model {
        AnalysisHingeModel::ConcentratedSpring => {
            "採用曲げ面: 強軸(Mz)固定（非線形ばねは強軸のみ。弱軸 My は弾性）。".to_string()
        }
        AnalysisHingeModel::Fiber | AnalysisHingeModel::MultiSpring | AnalysisHingeModel::Other => {
            format!(
                "採用曲げ面: {}",
                if bend_dir_z {
                    "強軸(Mz)"
                } else {
                    "弱軸(My)"
                }
            )
        }
    };
    ui.label(bend_face_label);

    ui.strong("M-θ カーブ（荷重変形カーブ）");
    ui.label(
        "横軸: 材端回転角 |θ| [rad]（弦からの回転）、縦軸: 曲げモーメント |M| [kN·m]（採用曲げ面の絶対値）。",
    );
    draw_m_theta_plot(ui, elem_id, &records, view, bend_dir_z, &mine, step);
    ui.separator();

    if view.model != AnalysisHingeModel::Other {
        ui.strong("N-M 相関図");
        match mn_display(view) {
            MnDisplay::Linear => {
                let mn = view.mn_linear.as_ref().expect("Linear は mn_linear を持つ");
                ui.label("非線形化は強軸(Mz)のみ（弱軸は弾性）。");
                ui.label(
                    "曲線: M = My0·(1−|N|/N許容) [My0=N=0 の降伏モーメント、N許容=軸許容耐力]。\
                     横軸 M [kN·m]、縦軸 N [kN]（圧縮正）。",
                );
                draw_mn_linear_plot(ui, elem_id, mn, &records, bend_dir_z, step);
            }
            MnDisplay::Surface => {
                let surface = view
                    .mn_surface
                    .as_ref()
                    .expect("Surface は mn_surface を持つ");
                let mut cam = app.ui.view.hinge_mn_camera.clone();
                draw_mn_plot(ui, surface, elem_id, &records, bend_dir_z, step, &mut cam);
                app.ui.view.hinge_mn_camera = cam;
            }
            MnDisplay::None => {
                ui.colored_label(theme::GRAY_600, mn_unavailable_message(view.model));
            }
        }
        ui.separator();
    }

    if let Some(sections) = fiber_sections {
        let sec = elem_section.and_then(|sid| app.core.model.sections.get(sid.index()));
        ui.strong("ファイバー断面の塑性化マップ（終局時）");
        draw_fiber_maps(ui, elem_id, &sections, &mine, sec);
    }
}

/// ヒンジ詳細で表示する N-M 図の種別。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MnDisplay {
    /// N-M 図を出さない（その他、または断面未定義で曲面なし）。
    None,
    /// 集中ばねの N-M 線形相関。
    Linear,
    /// ファイバー／マルチスプリングの N-M 曲面。
    Surface,
}

/// `HingeView` に対応する N-M 図の種別を返す（純粋関数）。
fn mn_display(view: &HingeView) -> MnDisplay {
    match view.model {
        AnalysisHingeModel::ConcentratedSpring if view.mn_linear.is_some() => MnDisplay::Linear,
        AnalysisHingeModel::Fiber | AnalysisHingeModel::MultiSpring
            if view.mn_surface.is_some() =>
        {
            MnDisplay::Surface
        }
        _ => MnDisplay::None,
    }
}

/// N-M 図を表示できないときの案内文。
fn mn_unavailable_message(model: AnalysisHingeModel) -> &'static str {
    match model {
        AnalysisHingeModel::ConcentratedSpring => {
            "この解析モデルでは N-M 相関を使用していません（履歴材料は N-M 相関非対応）。"
        }
        AnalysisHingeModel::Fiber | AnalysisHingeModel::MultiSpring => {
            "断面または材料の情報が不足しているため N-M 相関図を表示できません。"
        }
        AnalysisHingeModel::Other => "この要素種別では N-M 相関図を表示しません。",
    }
}

/// 解析モデルの表示ラベル。
fn analysis_model_label(model: AnalysisHingeModel) -> &'static str {
    match model {
        AnalysisHingeModel::ConcentratedSpring => "材端集中ばね",
        AnalysisHingeModel::Fiber => "ファイバー（マルチファイバー）",
        AnalysisHingeModel::MultiSpring => "マルチスプリング",
        AnalysisHingeModel::Other => "その他（非線形ヒンジなし）",
    }
}

/// 確定ステップ（`records` の添字）を選ぶスライダーを表示し、解決後の添字を返す。
/// `app.ui.scoped.hinge_step` が `None` の間は最終ステップを既定とする。
fn hinge_step_selector(ui: &mut egui::Ui, app: &mut App, records: &[MemberStepState]) -> usize {
    let last = records.len() - 1;
    let mut step = app
        .ui
        .scoped
        .hinge_step
        .map(|s| s.min(last))
        .unwrap_or(last);
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label("表示ステップ:");
        changed = ui
            .add(egui::Slider::new(&mut step, 0..=last).text(format!("/ 全 {}", records.len())))
            .changed();
    });
    if changed {
        app.ui.scoped.hinge_step = Some(step);
    }
    let axial_kn = force_kn(records[step].n as f64);
    ui.label(format!(
        "step {} / 全 {}（軸力 N = {:.1} kN、圧縮正）",
        step,
        records.len(),
        axial_kn
    ));
    step
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
    view: &HingeView,
    bend_dir_z: bool,
    mine: &[HingeMarker],
    selected_step: usize,
) {
    let use_spring_rot = view.model == AnalysisHingeModel::ConcentratedSpring;
    let (i_pts, j_pts) = m_theta_series(records, bend_dir_z, use_spring_rot);
    let has_i = mine.iter().any(|m| !m.end_j);
    let has_j = mine.iter().any(|m| m.end_j);
    egui_plot::Plot::new(format!("hinge_m_theta_{}", elem_id.0))
        .x_axis_label("|θ| [rad]")
        .y_axis_label("|M| [kN·m]")
        .legend(egui_plot::Legend::default())
        .height(220.0)
        .show(ui, |plot_ui| {
            if let Some(bp) = view.backbone.as_deref() {
                let xy: Vec<[f64; 2]> = bp.iter().map(|p| [p[0], moment_kn_m(p[1])]).collect();
                plot_ui.line(
                    egui_plot::Line::new("M-θ 骨格（解析モデル）", egui_plot::PlotPoints::from(xy))
                        .color(theme::GRAY_600)
                        .width(1.5_f32),
                );
            }
            if has_i {
                plot_m_theta_end(plot_ui, "i端", &i_pts, theme::DATA_BLUE, selected_step);
            }
            if has_j {
                plot_m_theta_end(plot_ui, "j端", &j_pts, theme::PARETO_RED, selected_step);
            }
        });
}

/// [θ(rad), M(N·mm)] 点列を [θ(rad), M(kN·m)] へ換算して描き、選択ステップの点を
/// マーカーで強調する。点と折れ線は同名で登録し凡例エントリを共有する。
fn plot_m_theta_end(
    plot_ui: &mut egui_plot::PlotUi<'_>,
    name: &str,
    pts: &[[f64; 2]],
    color: egui::Color32,
    selected_step: usize,
) {
    if pts.is_empty() {
        return;
    }
    let xy: Vec<[f64; 2]> = pts.iter().map(|p| [p[0], moment_kn_m(p[1])]).collect();
    plot_ui.line(
        egui_plot::Line::new(name, egui_plot::PlotPoints::from(xy.clone()))
            .color(color)
            .width(2.0_f32),
    );
    if let Some(&selected) = xy.get(selected_step) {
        plot_ui.points(
            egui_plot::Points::new("選択ステップ", egui_plot::PlotPoints::from(vec![selected]))
                .color(theme::HILITE_PURPLE)
                .radius(6.0_f32)
                .shape(egui_plot::MarkerShape::Circle),
        );
    }
}

/// N-M 相関図（ファイバー／マルチスプリング）: N-My-Mz 曲面の 3D ワイヤーフレーム
/// （上段）＋ 2D スライス（採用曲げ面での正曲げ側・負曲げ側の曲線＋応答経路、下段）。
fn draw_mn_plot(
    ui: &mut egui::Ui,
    surface: &MnSurface,
    elem_id: ElemId,
    records: &[MemberStepState],
    bend_dir_z: bool,
    selected_step: usize,
    cam: &mut crate::viewer::CameraState,
) {
    draw_mn_plot_3d(ui, surface, records, selected_step, cam);
    ui.add_space(4.0);
    ui.separator();
    draw_mn_plot_2d(ui, surface, elem_id, records, bend_dir_z, selected_step);
}

/// N-M 相関図の 3D ワイヤーフレーム（N-My-Mz 曲面）＋ 3D 応答経路。
///
/// 曲面そのものの描き方（格子解像度・正規化基準・投影スケール・座標軸・
/// ワイヤーフレーム）は断面詳細ビュー（`crate::mn_view`）と同じで、
/// [`crate::viewer::mn_draw`] に集約してある。この関数が持つのは、
/// そこへ応答経路を重ねるという本画面固有の部分だけである。
fn draw_mn_plot_3d(
    ui: &mut egui::Ui,
    surface: &MnSurface,
    records: &[MemberStepState],
    selected_step: usize,
    cam: &mut crate::viewer::CameraState,
) {
    let (rect, response) = ui.allocate_exact_size(
        egui::vec2(ui.available_width(), 260.0),
        egui::Sense::click_and_drag(),
    );
    cam.apply_pointer_input(ui, &response, true);

    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, theme::VIEW_BG);

    let refs = mn_draw::surface_refs(surface);
    let view = mn_draw::MnView::new(&rect, cam);

    mn_draw::draw_axes(&painter, &view);
    mn_draw::draw_wireframe(&painter, surface, refs, &view, theme::DATA_BLUE, 160);
    draw_mn_response_path_3d(&painter, records, refs, &view, selected_step);

    mn_draw::draw_camera_hint(ui);
}

/// 3D 応答経路（原点前置済み）を折れ線＋選択ステップのマーカーで描く。
fn draw_mn_response_path_3d(
    painter: &egui::Painter,
    records: &[MemberStepState],
    refs: [f64; 3],
    view: &mn_draw::MnView<'_>,
    selected_step: usize,
) {
    let path = n_my_mz_response_path_3d(records);
    if path.len() < 2 {
        return;
    }
    let pts: Vec<egui::Pos2> = path
        .iter()
        .map(|p| view.project([p[0] / refs[0], p[1] / refs[1], p[2] / refs[2]]))
        .collect();
    let stroke = egui::Stroke::new(2.5_f32, theme::PARETO_RED);
    for w in pts.windows(2) {
        painter.line_segment([w[0], w[1]], stroke);
    }
    painter.circle_stroke(pts[0], 4.0, egui::Stroke::new(1.5_f32, theme::GRAY_600));
    if let Some(&selected) = pts.get(selected_step + 1) {
        painter.circle_filled(selected, 5.0, theme::PARETO_RED);
    }
}

/// N-M 相関図の 2D スライス（採用曲げ面での正曲げ側・負曲げ側の曲線＋応答経路）
/// を egui_plot で描く（3D ワイヤーフレームの下段）。
///
/// `Plot` の ID に `elem_id` を含める理由は [`draw_m_theta_plot`] のドキュメント
/// コメントを参照（固定 ID だと部材切替時に前の部材の表示範囲を引き継いでしまう）。
fn draw_mn_plot_2d(
    ui: &mut egui::Ui,
    surface: &MnSurface,
    elem_id: ElemId,
    records: &[MemberStepState],
    bend_dir_z: bool,
    selected_step: usize,
) {
    let (j_pos, j_neg) = mn_beta_columns(mn_draw::N_BETA, bend_dir_z);
    let pos = extract_mn_meridian(&surface.grid, j_pos, bend_dir_z);
    let neg = extract_mn_meridian(&surface.grid, j_neg, bend_dir_z);
    let response_path = n_m_response_path(records, bend_dir_z);
    egui_plot::Plot::new(format!("hinge_mn_{}", elem_id.0))
        .x_axis_label("M [kN·m]")
        .y_axis_label("N [kN]（圧縮正）")
        .legend(egui_plot::Legend::default())
        .height(220.0)
        .show(ui, |plot_ui| {
            plot_ui.line(
                egui_plot::Line::new("N-M 相関(正曲げ側)", egui_plot::PlotPoints::from(pos))
                    .color(theme::GRAY_600)
                    .width(1.5_f32),
            );
            plot_ui.line(
                egui_plot::Line::new("N-M 相関(負曲げ側)", egui_plot::PlotPoints::from(neg))
                    .color(theme::GRAY_300)
                    .width(1.5_f32),
            );
            plot_ui.line(
                egui_plot::Line::new(
                    "応答経路",
                    egui_plot::PlotPoints::from(response_path.clone()),
                )
                .color(theme::DATA_BLUE)
                .width(2.0_f32),
            );
            if let Some(&selected) = response_path.get(selected_step + 1) {
                plot_ui.points(
                    egui_plot::Points::new(
                        "選択ステップ",
                        egui_plot::PlotPoints::from(vec![selected]),
                    )
                    .color(theme::HILITE_PURPLE)
                    .radius(6.0_f32)
                    .shape(egui_plot::MarkerShape::Circle),
                );
            }
        });
}

/// 集中ばねの N-M 線形相関 `M = My0·(1−|N|/N許容)` を 2D で描く。
///
/// 横軸 M [kN·m]（±両側）、縦軸 N [kN]（圧縮正）。応答経路は採用曲げ面の端
/// モーメントと軸力（[`n_m_response_path`]）を重ね、選択ステップを強調する。
fn draw_mn_linear_plot(
    ui: &mut egui::Ui,
    elem_id: ElemId,
    mn: &MnInteraction,
    records: &[MemberStepState],
    bend_dir_z: bool,
    selected_step: usize,
) {
    const N_PTS: usize = 41;
    let mut pos = Vec::with_capacity(N_PTS + 1);
    let mut neg = Vec::with_capacity(N_PTS + 1);
    for k in 0..=N_PTS {
        let n = -mn.n_allow + 2.0 * mn.n_allow * k as f64 / N_PTS as f64;
        let m = moment_kn_m(mn.moment_limit(n));
        let n_kn = force_kn(n);
        pos.push([m, n_kn]);
        neg.push([-m, n_kn]);
    }
    let response_path = n_m_response_path(records, bend_dir_z);
    egui_plot::Plot::new(format!("hinge_mn_linear_{}", elem_id.0))
        .x_axis_label("M [kN·m]")
        .y_axis_label("N [kN]（圧縮正）")
        .legend(egui_plot::Legend::default())
        .height(220.0)
        .show(ui, |plot_ui| {
            plot_ui.line(
                egui_plot::Line::new("N-M 相関(+M側)", egui_plot::PlotPoints::from(pos))
                    .color(theme::GRAY_600)
                    .width(1.5_f32),
            );
            plot_ui.line(
                egui_plot::Line::new("N-M 相関(−M側)", egui_plot::PlotPoints::from(neg))
                    .color(theme::GRAY_300)
                    .width(1.5_f32),
            );
            plot_ui.line(
                egui_plot::Line::new(
                    "応答経路",
                    egui_plot::PlotPoints::from(response_path.clone()),
                )
                .color(theme::DATA_BLUE)
                .width(2.0_f32),
            );
            if let Some(&selected) = response_path.get(selected_step + 1) {
                plot_ui.points(
                    egui_plot::Points::new(
                        "選択ステップ",
                        egui_plot::PlotPoints::from(vec![selected]),
                    )
                    .color(theme::HILITE_PURPLE)
                    .radius(6.0_f32)
                    .shape(egui_plot::MarkerShape::Circle),
                );
            }
        });
}

/// ファイバー座標系へ変換済みの断面外形線（外形, 内形（中空断面のみ））。
type SectionOutline = (Vec<[f64; 2]>, Option<Vec<[f64; 2]>>);

/// ヒンジのある端（i端・j端）についてファイバー断面の塑性化マップを横に並べる。
/// `sec` は断面外形線の重ね描き用（`None` ならファイバーのみ描く）。
fn draw_fiber_maps(
    ui: &mut egui::Ui,
    elem_id: ElemId,
    sections: &[FiberSectionState],
    mine: &[HingeMarker],
    sec: Option<&Section>,
) {
    let outline = sec.and_then(fiber_frame_outline);
    let want_i = mine.iter().any(|m| !m.end_j);
    let want_j = mine.iter().any(|m| m.end_j);
    ui.horizontal(|ui| {
        if want_i {
            if let Some(s) = pick_fiber_section(sections, false) {
                ui.vertical(|ui| draw_one_fiber_map(ui, elem_id, "i端", "i", s, outline.as_ref()));
            }
        }
        if want_j {
            if let Some(s) = pick_fiber_section(sections, true) {
                ui.vertical(|ui| draw_one_fiber_map(ui, elem_id, "j端", "j", s, outline.as_ref()));
            }
        }
    });
}

/// ファイバー断面 1 断面分の塑性化マップ（散布図）を描く。`id_suffix` は
/// `egui_plot::Plot` の ID 重複を避けるための識別子（"i"/"j"）。`elem_id` も
/// ID に含める理由は [`draw_m_theta_plot`] のドキュメントコメントを参照
/// （固定 ID だと部材切替時に前の部材のズーム状態を引き継いでしまう）。
/// `outline` は断面外形線（外形, 内形（中空断面のみ）。[`fiber_frame_outline`]）。
fn draw_one_fiber_map(
    ui: &mut egui::Ui,
    elem_id: ElemId,
    end_label: &str,
    id_suffix: &str,
    sec: &FiberSectionState,
    outline: Option<&SectionOutline>,
) {
    let yielded = sec.fibers.iter().filter(|f| f.yield_ratio >= 1.0).count();
    ui.label(format!(
        "終局時 ξ={:.2}（{}側）／降伏ファイバー {}/{}",
        sec.xi,
        end_label,
        yielded,
        sec.fibers.len()
    ));
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
            if let Some((outer, inner)) = outline {
                draw_section_outline(plot_ui, outer);
                if let Some(inner) = inner {
                    draw_section_outline(plot_ui, inner);
                }
            }
            draw_fiber_scatter(plot_ui, &sec.fibers);
        });
}

/// 断面外形線（閉多角形）を、ファイバー点より目立たない中間色の細線で描く
/// （凡例には出さない）。ファイバー配置ミス（例: 角形鋼管なのに中実配置）を
/// 外形線との対比で判別できるようにするための背景ガイド。
fn draw_section_outline(plot_ui: &mut egui_plot::PlotUi<'_>, pts: &[[f64; 2]]) {
    if pts.is_empty() {
        return;
    }
    let mut closed: Vec<[f64; 2]> = pts.to_vec();
    closed.push(pts[0]);
    plot_ui.line(
        egui_plot::Line::new("outline", egui_plot::PlotPoints::from(closed))
            .color(theme::GRAY_300)
            .width(1.2_f32)
            .allow_hover(false),
    );
}

/// 断面 `sec` の外形線を、ファイバー座標系（[`FiberStateSample`] の y/z。
/// `build_gauss_fibers` の 90°回転後: y=せい方向、z=−幅方向）へ変換して返す
/// （外形, 内形（中空断面のみ）。断面を描けなければ None）。
///
/// [`super::solid::section_outline`]／[`super::solid::section_inner_outline`]
/// が返す輪郭は「局所 y=せい, z=幅」の生の形状座標（重心補正なし）であり、
/// ファイバー側とは 2 点で異なる:
/// - 山形・溝形・T形・リップ溝形鋼・上下非対称ビルトH は、ファイバー生成時に
///   断面積重心が原点に来るよう平行移動されている
///   （`squid_n_section::mn_surface::fibers::plastic_fibers_at` 末尾の補正）。
/// - `build_gauss_fibers` の 90°回転 `(y,z)←(z,−y)` により、幅方向の符号が
///   反転している（輪郭の z=+幅方向、ファイバーの z=−幅方向）。
///
/// 生の輪郭多角形自身の面積重心を求めて原点へ平行移動する処理は、上記の
/// 断面積重心補正と数学的に同値（同一形状の連続体面積重心は求め方によらず
/// 一致する。多角形の場合はシューレース法の重心公式で厳密に求まる）ため、
/// 山形等の個別実装をせずに済む。対称断面（矩形・H・箱・円）は多角形重心が
/// 元々 (0,0) 付近になるため実質的に補正は効かない（下記のテストで検証）。
fn fiber_frame_outline(sec: &Section) -> Option<SectionOutline> {
    let outer_raw = super::solid::section_outline(sec)?;
    let inner_raw = super::solid::section_inner_outline(sec);
    let [cy, cz] = squid_n_core::geom::polygon::centroid(&outer_raw);
    let xform = |pts: &[[f64; 2]]| -> Vec<[f64; 2]> {
        pts.iter().map(|&[y, z]| [y - cy, cz - z]).collect()
    };
    Some((xform(&outer_raw), inner_raw.as_deref().map(xform)))
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
    use squid_n_core::units::to_display::moment_kn_m;

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
            spring_rz_i: None,
            spring_rz_j: None,
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
        let (i_pts, j_pts) = m_theta_series(&records, true, false);
        assert_pts_near(&i_pts, &[[0.01, 100.0]]);
        assert_pts_near(&j_pts, &[[0.02, 50.0]]);
    }

    /// 材端集中ばね（`use_spring_rot=true`）では、弦からの材端回転ではなく
    /// 解析が実際に使う端ばね変形を横軸に使う。
    #[test]
    fn m_theta_series_uses_spring_rotation_for_concentrated_spring() {
        let mut records = vec![step(100.0, -50.0, 1.0, 1.0, 0.0)];
        records[0].rz_i = -0.10;
        records[0].rz_j = 0.20;
        records[0].spring_rz_i = Some(-0.01);
        records[0].spring_rz_j = Some(0.02);
        let (i_pts, j_pts) = m_theta_series(&records, true, true);
        assert_pts_near(&i_pts, &[[0.01, 100.0]]);
        assert_pts_near(&j_pts, &[[0.02, 50.0]]);
    }

    /// 端ばね変形が記録されていない場合は弦からの材端回転へフォールバックする。
    #[test]
    fn m_theta_series_falls_back_without_spring_rotation() {
        let mut records = vec![step(100.0, 50.0, 1.0, 1.0, 0.0)];
        records[0].rz_i = -0.03;
        records[0].rz_j = 0.04;
        let (i_pts, j_pts) = m_theta_series(&records, true, true);
        assert_pts_near(&i_pts, &[[0.03, 100.0]]);
        assert_pts_near(&j_pts, &[[0.04, 50.0]]);
    }

    /// 弱軸採用時は ry/my を抽出する。
    #[test]
    fn m_theta_series_extracts_weak_axis_when_selected() {
        let mut records = vec![step(0.0, 0.0, 30.0, -20.0, 0.0)];
        records[0].ry_i = 0.005;
        records[0].ry_j = -0.006;
        let (i_pts, j_pts) = m_theta_series(&records, false, false);
        assert_pts_near(&i_pts, &[[0.005, 30.0]]);
        assert_pts_near(&j_pts, &[[0.006, 20.0]]);
    }

    /// 空入力は空の点列を返す。
    #[test]
    fn m_theta_series_empty_input() {
        let (i_pts, j_pts) = m_theta_series(&[], true, false);
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
        // -80 N·mm -> kN·m、n=1000N -> 1kN。原点の次（[1]）が実データ点。
        assert!((path[1][0] - moment_kn_m(-80.0)).abs() < 1e-12);
        assert!((path[1][1] - 1.0).abs() < 1e-9);
    }

    /// 弱軸採用時は my_i/my_j を対象にする。
    #[test]
    fn n_m_response_path_uses_weak_axis_when_selected() {
        let records = vec![step(0.0, 0.0, 40.0, -90.0, 0.0)];
        let path = n_m_response_path(&records, false);
        assert!((path[1][0] - moment_kn_m(-90.0)).abs() < 1e-12);
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
        // M[kN·m] = My/1e6, N[kN] = -N/1e3（圧縮正へ変換）。
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
        assert!((pts[0][0] - moment_kn_m(200.0)).abs() < 1e-12);
    }

    /// 列が範囲外の行は無視する（`row.get` が `None` を返す行はスキップ）。
    #[test]
    fn extract_mn_meridian_skips_out_of_range_rows() {
        let grid = vec![vec![[0.0, 1.0, 2.0]], vec![]];
        let pts = extract_mn_meridian(&grid, 0, false);
        assert_eq!(pts.len(), 1);
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

    // --- 断面外形線（ファイバー塑性化マップへの重ね描き）関連のテスト ---

    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::section_shape::SectionShape;
    use squid_n_section::mn_surface::{plastic_fibers, StrengthParams, YieldModelKind};

    /// 中心 (0,0) の正方形（原点対称）の面積重心は原点。
    #[test]
    fn section_outline_centroid_of_centered_square_is_origin() {
        let sq = vec![[10.0, -5.0], [10.0, 5.0], [-10.0, 5.0], [-10.0, -5.0]];
        let [cy, cz] = squid_n_core::geom::polygon::centroid(&sq);
        assert!(cy.abs() < 1e-9, "cy={cy}");
        assert!(cz.abs() < 1e-9, "cz={cz}");
    }

    /// 平行移動した矩形の面積重心は、移動先の幾何中心と一致する。
    #[test]
    fn section_outline_centroid_of_offset_rect_matches_geometric_center() {
        // 元は中心 (0,0)・10×4 の矩形を (100, -50) へ平行移動。
        let rect = vec![[105.0, -52.0], [105.0, -48.0], [95.0, -48.0], [95.0, -52.0]];
        let [cy, cz] = squid_n_core::geom::polygon::centroid(&rect);
        assert!((cy - 100.0).abs() < 1e-9, "cy={cy}");
        assert!((cz - (-50.0)).abs() < 1e-9, "cz={cz}");
    }

    /// テスト用の角形鋼管（SteelBox）断面。
    fn steel_box_section() -> Section {
        Section {
            id: SectionId(1),
            name: "BOX-300x200x9".into(),
            area: 1.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 300.0,
            width: 200.0,
            as_y: 0.0,
            as_z: 0.0,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: Some(SectionShape::SteelBox {
                height: 300.0,
                width: 200.0,
                thick: 9.0,
                corner_r: 0.0,
            }),
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        }
    }

    /// 点列の [y,z] 各軸のバウンディングボックス（min, max）。
    fn bbox(pts: &[[f64; 2]]) -> ([f64; 2], [f64; 2]) {
        let min = [
            pts.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min),
            pts.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min),
        ];
        let max = [
            pts.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max),
            pts.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max),
        ];
        (min, max)
    }

    /// SteelBox（角形鋼管）は外形線・内側輪郭（中空）の両方が生成され、
    /// それぞれのバウンディングボックスが height×width／(height−2t)×(width−2t)
    /// と一致する。
    #[test]
    fn fiber_frame_outline_steel_box_has_outer_and_inner() {
        let sec = steel_box_section();
        let (outer, inner) = fiber_frame_outline(&sec).expect("箱形は外形線を持つ");
        let inner = inner.expect("箱形は中空断面のため内側輪郭を持つ");

        let (omin, omax) = bbox(&outer);
        assert!((omax[0] - omin[0] - 300.0).abs() < 1e-6);
        assert!((omax[1] - omin[1] - 200.0).abs() < 1e-6);

        let (imin, imax) = bbox(&inner);
        assert!((imax[0] - imin[0] - (300.0 - 18.0)).abs() < 1e-6);
        assert!((imax[1] - imin[1] - (200.0 - 18.0)).abs() < 1e-6);
    }

    /// SteelBox のファイバー（`squid_n_section::mn_surface::plastic_fibers` に
    /// `build_gauss_fibers` と同じ 90°回転を適用したもの＝要素座標系）は、
    /// すべて外形線のバウンディングボックス内に収まり、かつ内側輪郭（中空部）の
    /// 内側には 1 本も存在しない（配置ミス＝中実配置であればこの条件が崩れる）。
    #[test]
    fn fiber_frame_outline_steel_box_fibers_lie_between_outer_and_inner() {
        let sec = steel_box_section();
        let (outer, inner) = fiber_frame_outline(&sec).unwrap();
        let inner = inner.unwrap();
        let (_, outer_max) = bbox(&outer);
        let (_, inner_max) = bbox(&inner);
        let (outer_half_y, outer_half_z) = (outer_max[0], outer_max[1]);
        let (inner_half_y, inner_half_z) = (inner_max[0], inner_max[1]);

        let shape = sec.shape.clone().unwrap();
        let strength = StrengthParams {
            steel_fy: 235.0,
            rebar_fy: 345.0,
            concrete_fc: 24.0,
            steel_e: 205_000.0,
        };
        let raw = plastic_fibers(&shape, &strength, YieldModelKind::MultiFiber);
        assert!(!raw.is_empty());
        // build_gauss_fibers と同じ 90°回転 (y,z)←(z,−y) を適用し要素座標系へ。
        let rotated: Vec<[f64; 2]> = raw.iter().map(|f| [f.z, -f.y]).collect();

        const EPS: f64 = 1e-6;
        for [y, z] in &rotated {
            assert!(
                y.abs() <= outer_half_y + EPS && z.abs() <= outer_half_z + EPS,
                "ファイバーが外形線の外側にある: y={y}, z={z}"
            );
            assert!(
                !(y.abs() < inner_half_y - EPS && z.abs() < inner_half_z - EPS),
                "ファイバーが内側輪郭（中空部）の内側にある: y={y}, z={z}"
            );
        }
    }

    /// 溝形鋼（SteelChannel、非対称断面）でも、外形線をファイバー座標系へ変換
    /// する際に断面積重心補正が輪郭側へ正しく効いており、ファイバー群
    /// （`plastic_fibers` に要素側と同じ 90°回転を適用したもの）が
    /// 外形線のバウンディングボックス内に収まる（ウェブ位置がずれて外形線と
    /// ファイバー群の左右が逆転する等の座標系不整合がない）ことを確認する。
    #[test]
    fn fiber_frame_outline_asymmetric_channel_aligns_with_fibers() {
        let sec = Section {
            id: SectionId(2),
            name: "C-200x80x7.5x11".into(),
            area: 1.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 200.0,
            width: 80.0,
            as_y: 0.0,
            as_z: 0.0,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: Some(SectionShape::SteelChannel {
                height: 200.0,
                width: 80.0,
                web_thick: 7.5,
                flange_thick: 11.0,
            }),
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        };
        let (outer, inner) = fiber_frame_outline(&sec).unwrap();
        assert!(inner.is_none(), "溝形鋼は中実断面のため内側輪郭はない");
        let (omin, omax) = bbox(&outer);

        let shape = sec.shape.clone().unwrap();
        let strength = StrengthParams {
            steel_fy: 235.0,
            rebar_fy: 345.0,
            concrete_fc: 24.0,
            steel_e: 205_000.0,
        };
        let raw = plastic_fibers(&shape, &strength, YieldModelKind::MultiFiber);
        assert!(!raw.is_empty());
        let rotated: Vec<[f64; 2]> = raw.iter().map(|f| [f.z, -f.y]).collect();

        // メッシュ分割の格子中心座標のため、境界セル半分弱の余裕を見込む
        // （目標寸法は最大寸法/40 = 200/40 = 5mm 程度）。
        const MARGIN: f64 = 6.0;
        for [y, z] in &rotated {
            assert!(
                *y >= omin[0] - MARGIN && *y <= omax[0] + MARGIN,
                "ファイバー y={y} が外形線 y 範囲 [{}, {}] から外れている",
                omin[0],
                omax[0]
            );
            assert!(
                *z >= omin[1] - MARGIN && *z <= omax[1] + MARGIN,
                "ファイバー z={z} が外形線 z 範囲 [{}, {}] から外れている",
                omin[1],
                omax[1]
            );
        }

        // ウェブ（面積の大部分）は z が負側に寄る配置のため、外形線の z 範囲も
        // 原点対称ではなく負側へ偏っている（重心補正が効いている証拠）。
        assert!(
            (omax[1] + omin[1]) < -1.0,
            "非対称断面なのに外形線が z=0 対称のまま（重心補正が効いていない）: omin={omin:?}, omax={omax:?}"
        );
    }

    // ── ヒンジ表示ビューのキャッシュキー ────────────────────────────────

    /// キャッシュキーのテスト用モデル（断面・材料を編集できる最小構成）。
    fn key_test_model() -> Model {
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::NodeId;
        use squid_n_core::model::{Material, Node};

        let node = |id: NodeId, coord: [f64; 3]| Node {
            id,
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        let section = Section {
            id: SectionId(0),
            name: "sec".into(),
            floor: None,
            area: 8000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 400.0,
            width: 200.0,
            as_y: 6666.7,
            as_z: 6666.7,
            panel_thickness: None,
            thickness: None,
            shape: None,
            material: Some(MaterialId(0)),
            rebar_material: Some(MaterialId(1)),
            shear_rebar_material: None,
            steel_material: None,
        };
        let material = |id, name: &str, category, fy: f64| Material {
            id,
            name: name.into(),
            category,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(fy),
            concrete_class: Default::default(),
            strength_factor: None,
        };
        Model {
            nodes: vec![
                node(NodeId(0), [0.0, 0.0, 0.0]),
                node(NodeId(1), [0.0, 0.0, 3000.0]),
            ],
            sections: vec![section],
            materials: vec![
                material(MaterialId(0), "SN400", MaterialCategory::Steel, 235.0),
                material(MaterialId(1), "SD345", MaterialCategory::Rebar, 345.0),
            ],
            ..Default::default()
        }
    }

    /// キャッシュキーのテスト用モデル（RC 断面形状つき。ファイバー曲面生成用）。
    fn key_test_model_rc() -> Model {
        use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

        let mut model = key_test_model();
        model.sections[0].shape = Some(SectionShape::RcRect {
            b: 400.0,
            d: 700.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 4,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 4,
                    dia: 22.0,
                    layers: 1,
                },
                cover: 50.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
        });
        model.materials[0].name = "Fc24".into();
        model.materials[0].category = MaterialCategory::Concrete;
        model.materials[0].fc = Some(24.0);
        model
    }

    /// キャッシュキーのテスト用要素。
    fn key_test_elem(kind: ElementKind, regime: ForceRegime) -> ElementData {
        use squid_n_core::ids::NodeId;
        use squid_n_core::model::{EndCondition, LocalAxis, RigidZone};

        ElementData {
            id: ElemId(0),
            kind,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: regime,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    fn key_of(model: &Model, elem: &ElementData, step: usize) -> HingeViewKey {
        hinge_view_key(model, elem, step, &Staleness::default())
    }

    /// 選択ステップが変わればキーが変わる（集中ばねの骨格・N-M 線に使う軸力が変わる）。
    #[test]
    fn hinge_view_key_changes_with_step() {
        let model = key_test_model();
        let elem = key_test_elem(ElementKind::Beam, ForceRegime::UniaxialBendingShear);
        assert_ne!(key_of(&model, &elem, 0), key_of(&model, &elem, 1));
    }

    /// ファイバー／マルチスプリングのビューは選択ステップの軸力に依存しないため、
    /// ステップが変わってもキーは変わらない（重い曲面を再生成しない）。
    #[test]
    fn hinge_view_key_ignores_step_for_non_concentrated() {
        let model = key_test_model();
        let fiber = key_test_elem(ElementKind::Fiber, ForceRegime::AxialBendingInteract);
        assert_eq!(key_of(&model, &fiber, 0), key_of(&model, &fiber, 1));
        let ms = key_test_elem(ElementKind::MultiSpring, ForceRegime::AxialBendingInteract);
        assert_eq!(key_of(&model, &ms, 0), key_of(&model, &ms, 1));
    }

    /// 同一ステップ数で再解析した場合（`last_run` のみ更新）もキーが変わる。
    /// また `results_stale` の変化も世代としてキーに含まれる。
    #[test]
    fn hinge_view_key_changes_with_result_generation_same_step_count() {
        use std::time::Duration;

        let model = key_test_model();
        let elem = key_test_elem(ElementKind::Beam, ForceRegime::UniaxialBendingShear);
        let mut stale = Staleness {
            last_run: Some(SystemTime::UNIX_EPOCH),
            ..Default::default()
        };
        let k0 = hinge_view_key(&model, &elem, 2, &stale);
        stale.last_run = Some(SystemTime::UNIX_EPOCH + Duration::from_secs(1));
        let k1 = hinge_view_key(&model, &elem, 2, &stale);
        assert_ne!(k0, k1, "同一 step 数でも再解析でキーが変わる");

        stale.results_stale = true;
        let k2 = hinge_view_key(&model, &elem, 2, &stale);
        assert_ne!(k1, k2, "results_stale も世代に含める");
    }

    /// 断面・材料・履歴則の編集でキーが変わる。
    #[test]
    fn hinge_view_key_changes_with_section_material_and_hysteresis() {
        let mut model = key_test_model();
        let elem = key_test_elem(ElementKind::Beam, ForceRegime::UniaxialBendingShear);
        let k0 = key_of(&model, &elem, 1);

        model.sections[0].area += 1.0;
        let k1 = key_of(&model, &elem, 1);
        assert_ne!(k0, k1, "断面編集でキーが変わる");

        model.materials[0].fy = Some(300.0);
        let k2 = key_of(&model, &elem, 1);
        assert_ne!(k1, k2, "材料編集でキーが変わる");

        model.set_member_hysteresis(ElemId(0), HysteresisModel::Takeda);
        let k3 = key_of(&model, &elem, 1);
        assert_ne!(k2, k3, "履歴則変更でキーが変わる");
        assert!(!k3.use_mn, "武田型は N-M 相関非対応");
    }

    /// 軸力が大きいほど集中ばね骨格の降伏モーメントが低下する
    /// （ステップ変更で軸力引数が変わる＝表示骨格が再生成されることの根拠）。
    #[test]
    fn concentrated_backbone_follows_axial_force() {
        let model = key_test_model();
        let elem = key_test_elem(ElementKind::Beam, ForceRegime::UniaxialBendingShear);
        let basis = StrengthBasis::MaterialStrength;
        let kind = AnalysisKind::Incremental;
        let b0 = build_hinge_view(
            &elem,
            &model,
            basis,
            kind,
            0.0,
            mn_draw::N_ALPHA,
            mn_draw::N_BETA,
        )
        .backbone
        .expect("集中ばねは骨格を返す");
        let b1 = build_hinge_view(
            &elem,
            &model,
            basis,
            kind,
            1.0e5,
            mn_draw::N_ALPHA,
            mn_draw::N_BETA,
        )
        .backbone
        .expect("集中ばねは骨格を返す");
        assert!(
            b0[1][1] > b1[1][1],
            "軸力が大きいほど降伏モーメントが低下する"
        );
    }

    /// 履歴材料（use_nm=false）の集中ばねは `mn_linear` が `None` で、N-M 図を出さない。
    #[test]
    fn concentrated_history_rule_has_no_mn_linear_and_no_nm_plot() {
        let mut model = key_test_model();
        model.set_member_hysteresis(ElemId(0), HysteresisModel::Takeda);
        let elem = key_test_elem(ElementKind::Beam, ForceRegime::UniaxialBendingShear);
        let view = build_hinge_view(
            &elem,
            &model,
            StrengthBasis::MaterialStrength,
            AnalysisKind::Incremental,
            0.0,
            mn_draw::N_ALPHA,
            mn_draw::N_BETA,
        );
        assert_eq!(view.model, AnalysisHingeModel::ConcentratedSpring);
        assert!(view.mn_linear.is_none());
        assert_eq!(mn_display(&view), MnDisplay::None);
    }

    /// 集中ばねは弱軸(My)支配の応答でも強軸(Mz)表示に固定し、ファイバー系は
    /// 支配軸（`dominant_bend_axis_z`）に従う。
    #[test]
    fn effective_bend_dir_z_forces_strong_axis_for_concentrated_spring() {
        let model = key_test_model();
        let elem = key_test_elem(ElementKind::Beam, ForceRegime::UniaxialBendingShear);
        let concentrated = build_hinge_view(
            &elem,
            &model,
            StrengthBasis::MaterialStrength,
            AnalysisKind::Incremental,
            0.0,
            mn_draw::N_ALPHA,
            mn_draw::N_BETA,
        );
        assert_eq!(concentrated.model, AnalysisHingeModel::ConcentratedSpring);

        let weak_dominant = step(50.0, -10.0, 200.0, -150.0, 0.0);
        assert!(!dominant_bend_axis_z(&weak_dominant), "弱軸支配の前提");
        assert!(
            effective_bend_dir_z(concentrated.model, dominant_bend_axis_z(&weak_dominant)),
            "集中ばねは弱軸支配でも強軸(Mz)に固定"
        );

        let rc = key_test_model_rc();
        let fiber = key_test_elem(ElementKind::Fiber, ForceRegime::AxialBendingInteract);
        let fiber_view = build_hinge_view(
            &fiber,
            &rc,
            StrengthBasis::MaterialStrength,
            AnalysisKind::Incremental,
            0.0,
            mn_draw::N_ALPHA,
            mn_draw::N_BETA,
        );
        assert_eq!(fiber_view.model, AnalysisHingeModel::Fiber);
        assert!(
            !effective_bend_dir_z(fiber_view.model, dominant_bend_axis_z(&weak_dominant)),
            "ファイバーは支配軸（弱軸 My）に従う"
        );
    }

    /// キャッシュは同じキーのときだけ再利用され、キーが変われば作り直される
    /// （`ensure_hinge_view` の一致判定）。
    #[test]
    fn hinge_view_cache_reuses_only_matching_key() {
        let model = key_test_model();
        let elem = key_test_elem(ElementKind::Beam, ForceRegime::UniaxialBendingShear);
        let k0 = key_of(&model, &elem, 0);
        let cache = HingeViewCache {
            key: k0.clone(),
            view: HingeView {
                model: AnalysisHingeModel::Other,
                backbone: None,
                mn_linear: None,
                mn_surface: None,
            },
        };
        assert_eq!(cache.key(), &k0);
        let k1 = key_of(&model, &elem, 1);
        assert_ne!(cache.key(), &k1);
    }

    /// 表示方向を切り替えるとキャッシュを破棄する。集中ばねの M-θ 骨格は表示中の
    /// 方向の `records` から選んだステップの軸力で作られるため、方向切替後に
    /// 古い軸力の骨格を再利用してはならない。
    #[test]
    fn set_pushover_view_dir_clears_hinge_view_cache() {
        use squid_n_solver::statics::analysis::SeismicDir;

        let model = key_test_model();
        let elem = key_test_elem(ElementKind::Beam, ForceRegime::UniaxialBendingShear);
        let mut app = App::default();
        app.core.model = model;
        app.core.scoped.pushover_view_dir = SeismicDir::X;
        let key = hinge_view_key(&app.core.model, &elem, 0, &app.core.scoped.staleness);
        ensure_hinge_view(&mut app, key, &elem, 0.0);
        assert!(app.ui.scoped.hinge_view_cache.is_some());

        app.set_pushover_view_dir(SeismicDir::Y);
        assert!(
            app.ui.scoped.hinge_view_cache.is_none(),
            "方向切替で古い軸力の骨格を破棄する"
        );
    }

    /// 要素種別に応じて `AnalysisHingeModel` と表示する N-M 図が対応する。
    #[test]
    fn mn_display_matches_element_model_kind() {
        let basis = StrengthBasis::MaterialStrength;
        let kind = AnalysisKind::Incremental;
        let rc = key_test_model_rc();

        let beam = key_test_elem(ElementKind::Beam, ForceRegime::UniaxialBendingShear);
        let beam_view = build_hinge_view(
            &beam,
            &key_test_model(),
            basis,
            kind,
            0.0,
            mn_draw::N_ALPHA,
            mn_draw::N_BETA,
        );
        assert_eq!(beam_view.model, AnalysisHingeModel::ConcentratedSpring);
        assert_eq!(mn_display(&beam_view), MnDisplay::Linear);

        let fiber = key_test_elem(ElementKind::Fiber, ForceRegime::AxialBendingInteract);
        let fiber_view = build_hinge_view(
            &fiber,
            &rc,
            basis,
            kind,
            0.0,
            mn_draw::N_ALPHA,
            mn_draw::N_BETA,
        );
        assert_eq!(fiber_view.model, AnalysisHingeModel::Fiber);
        assert_eq!(mn_display(&fiber_view), MnDisplay::Surface);

        let ms = key_test_elem(ElementKind::MultiSpring, ForceRegime::AxialBendingInteract);
        let ms_view = build_hinge_view(
            &ms,
            &rc,
            basis,
            kind,
            0.0,
            mn_draw::N_ALPHA,
            mn_draw::N_BETA,
        );
        assert_eq!(ms_view.model, AnalysisHingeModel::MultiSpring);
        assert_eq!(mn_display(&ms_view), MnDisplay::Surface);

        let other = key_test_elem(
            ElementKind::Brace {
                tension_only: false,
            },
            ForceRegime::AxialBendingInteract,
        );
        let other_view = build_hinge_view(
            &other,
            &rc,
            basis,
            kind,
            0.0,
            mn_draw::N_ALPHA,
            mn_draw::N_BETA,
        );
        assert_eq!(other_view.model, AnalysisHingeModel::Other);
        assert_eq!(mn_display(&other_view), MnDisplay::None);
    }
}
