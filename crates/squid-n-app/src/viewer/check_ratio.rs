//! 検定比図（部材検定・節点検定の検定比による着色）の描画。
//!
//! 部材・節点は [`theme::check_ratio_color`] の連続グラデーション
//! （≤0.8 淡緑→緑／≤1.0 黄→アンバー／>1.0 赤=NG）で着色する。判定境界
//! （0.8／1.0）では検定表（`design_view.rs`）の3色規約と同じ色相へ切り替わる
//! ため、表と3Dビューの見え方は判定レベルの粒度で一貫する。
//!
//! 数値ラベルは全部材に出すと過密で読めなくなるため、既定では注意域以上
//! （検定比 ≥ [`LABEL_MIN_RATIO`]）の部材にのみ表示し、それ未満は色の濃淡
//! だけで余裕度を示す（`app.ui.view.check_ratio_label_all` で全表示に切り替え可能）。
//!
//! 着色対象は [`CheckRatioFilter`]（最大＝全式の max、または特定の検定式のみ）で
//! 切り替えられ、部材内の検定位置ごとに正方形マーカーを重ねる「位置別マーカー」、
//! ホバー時に位置×式の内訳を見せるツールチップも提供する。
//!
//! 検定不能（[`CheckOutcome::Skipped`]）の位置は、検定式フィルタ適用後は
//! いずれも `None`（対象外）として扱う。ある部材の全位置が検定不能の場合、
//! その部材は未検定と同様に無着色となる（位置別マーカーも描かない）。

use std::collections::HashMap;

use crate::app::{App, MemberChecks, PositionCheck};
use crate::theme;
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_design_jp::{CheckComponent, CheckKind, CheckOutcome, CheckResult};

use super::CheckRatioFilter;

/// 部材中点の数値ラベルを表示する下限の検定比（注意域の境界）。
/// これ未満の部材はラベルを描かず、色のグラデーション
/// （[`theme::check_ratio_color`]）だけで余裕度を示す。
pub(super) const LABEL_MIN_RATIO: f64 = 0.8;

/// `CheckKind` の定義順（Bending→…→Deflection→Provision）で
/// 固定した全種一覧。フィルタ選択肢・ツールチップの列順を安定させるために使う。
const ALL_KINDS: [CheckKind; 7] = [
    CheckKind::Bending,
    CheckKind::Shear,
    CheckKind::Bond,
    CheckKind::AxialBending,
    CheckKind::Axial,
    CheckKind::Deflection,
    CheckKind::Provision,
];

/// フィルタ `filter` を検定結果 `outcome` に適用した結果（検定比, OK か）を
/// 返す（純粋関数）。
///
/// - [`CheckOutcome::Skipped`]（検定不能）は常に `None`（フィルタ対象外。
///   未検定と同様に着色しない）。
/// - `Max`: `cr.ratio()`／`cr.ok()` を返す（従来動作）。
/// - `Kind(k)`: `cr.components` から `kind == k` の最大検定比を探し
///   `Some((r, r <= 1.0))` を返す。該当する式がなければ `None`
///   （＝この検定位置は当該式の検定対象外。着色・マーカーとも描かない）。
pub(super) fn ratio_for_filter(
    outcome: &CheckOutcome,
    filter: CheckRatioFilter,
) -> Option<(f64, bool)> {
    let CheckOutcome::Checked(cr) = outcome else {
        return None;
    };
    match filter {
        CheckRatioFilter::Max => Some((cr.ratio(), cr.ok())),
        CheckRatioFilter::Kind(k) => {
            let max_ratio = cr
                .components
                .iter()
                .filter(|c| c.kind == k)
                .map(|c| c.ratio)
                .fold(None, |acc: Option<f64>, r| {
                    Some(acc.map_or(r, |a| a.max(r)))
                });
            max_ratio.map(|r| (r, r <= 1.0))
        }
    }
}

/// `components` 中の最大検定比を与える `kind`（空なら `None`）。
fn dominant_kind_of(components: &[CheckComponent]) -> Option<CheckKind> {
    components
        .iter()
        .max_by(|a, b| {
            a.ratio
                .partial_cmp(&b.ratio)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|c| c.kind)
}

/// `cr.components` 中の最大検定比を与える支配式（空なら `None`）。
pub(super) fn dominant_kind(cr: &CheckResult) -> Option<CheckKind> {
    dominant_kind_of(&cr.components)
}

/// 与えられた検定結果群の `components` に実際に現れる `CheckKind` を、定義順で
/// 重複なく返す（純粋関数）。ツールバーの検定式フィルタ選択肢・ツールチップの
/// 列を「結果に現れる式だけ」に絞るために使う（RC モデルで「軸」等の無関係な
/// 選択肢が並ばないようにする）。
pub(super) fn available_check_kinds<'a, I>(components_iter: I) -> Vec<CheckKind>
where
    I: IntoIterator<Item = &'a [CheckComponent]>,
{
    let mut present = [false; ALL_KINDS.len()];
    for comps in components_iter {
        for c in comps {
            if let Some(idx) = ALL_KINDS.iter().position(|k| *k == c.kind) {
                present[idx] = true;
            }
        }
    }
    ALL_KINDS
        .iter()
        .copied()
        .zip(present)
        .filter_map(|(k, p)| p.then_some(k))
        .collect()
}

/// 部材（または節点）ごとに、フィルタ適用後の検定比・OK フラグを集計する
/// （純粋関数）。`items` は `(キー, フィルタ適用後の (検定比, OK) または None)`。
/// `None`（フィルタ対象外の位置。検定不能を含む）は無視され、対象位置が一つも
/// ない部材・節点は集計結果に含まれない（＝未検定として扱われ、着色されない）。
fn max_ratio_by_key<K, I>(items: I) -> HashMap<K, (f64, bool)>
where
    K: Eq + std::hash::Hash,
    I: IntoIterator<Item = (K, Option<(f64, bool)>)>,
{
    let mut map: HashMap<K, (f64, bool)> = HashMap::new();
    for (key, val) in items {
        let Some((ratio, ok)) = val else {
            continue;
        };
        let entry = map.entry(key).or_insert((0.0_f64, true));
        if ratio > entry.0 {
            entry.0 = ratio;
        }
        entry.1 &= ok;
    }
    map
}

/// 部材ごとの（フィルタ適用後の）最大検定比・OK フラグを集計する。
fn max_ratio_by_elem<I: IntoIterator<Item = (ElemId, Option<(f64, bool)>)>>(
    items: I,
) -> HashMap<ElemId, (f64, bool)> {
    max_ratio_by_key(items)
}

/// 節点ごとの（フィルタ適用後の）最大検定比・OK フラグを集計する。
fn max_ratio_by_node<I: IntoIterator<Item = (NodeId, Option<(f64, bool)>)>>(
    items: I,
) -> HashMap<NodeId, (f64, bool)> {
    max_ratio_by_key(items)
}

/// 部材中点に数値ラベルを描くかを判定する（純粋関数）。
///
/// 既定（`label_all == false`）では注意域以上（検定比 ≥ [`LABEL_MIN_RATIO`]）
/// の部材のみ描き、それ未満は色のグラデーションだけで表す（全部材に数値を
/// 出すと3Dビュー上で文字が重なり読めなくなるため）。`label_all == true`
/// なら検定比によらず常に描く。
pub(super) fn should_label(ratio: f64, label_all: bool) -> bool {
    label_all || ratio >= LABEL_MIN_RATIO
}

/// 節点検定マーカー（ひし形）の半径 [px]。OK 判定用。
pub(super) const NODE_MARKER_RADIUS: f32 = 7.0;
/// 節点検定マーカーの半径 [px]。NG は一回り大きくして目立たせる。
pub(super) const NODE_MARKER_RADIUS_NG: f32 = 9.0;
/// 節点検定のホバー判定しきい値 [px]（マーカーより少し広く取る）。
pub(super) const NODE_HOVER_THRESHOLD: f32 = 10.0;

/// ホバー位置に最も近い「検定結果を持つ節点」を返す（`(節点 index, 距離)`）。
///
/// 部材のホバー判定（`pick_nearest_member`）と同じ方針で、しきい値の判定は
/// 呼び出し側が行う。検定結果を持たない節点は候補にしない。
///
/// ホバー判定は毎フレーム走るため、検定を持つ節点は先に集合へ集めてから走査する
/// （節点ごとに検定リストを線形探索すると O(節点数 × 検定数) になる）。
pub(super) fn pick_nearest_checked_node(
    app: &App,
    pts: &[egui::Pos2],
    pos: egui::Pos2,
    frame_filter: super::FrameFilter,
) -> Option<(usize, f32)> {
    let results = app.core.scoped.results.as_ref()?;
    if results.joint_checks.is_empty() {
        return None;
    }
    let checked: std::collections::HashSet<NodeId> =
        results.joint_checks.iter().map(|j| j.node).collect();
    let mut best: Option<(usize, f32)> = None;
    for (idx, node) in app.core.model.nodes.iter().enumerate() {
        // 構面表示で描いていない節点は選べない（見えない節点の検定詳細が出るのを防ぐ）。
        if idx >= pts.len() || !checked.contains(&node.id) || !frame_filter.shows_node(idx) {
            continue;
        }
        let d = pts[idx].distance(pos);
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((idx, d));
        }
    }
    best
}

/// 節点 `node` の検定詳細（種別・検定比・判定・根拠）をポインタ位置に表示する。
pub(super) fn show_node_check_tooltip(ui: &egui::Ui, app: &App, node: NodeId) {
    let Some(results) = app.core.scoped.results.as_ref() else {
        return;
    };
    let rows: Vec<&crate::app::JointCheck> = results
        .joint_checks
        .iter()
        .filter(|j| j.node == node)
        .collect();
    if rows.is_empty() {
        return;
    }
    // `show_tooltip_at_pointer` は egui 0.34 で非推奨だが、ウィジェットに紐付かない
    // 任意位置への表示という用途に代替がないため、部材側と同じ方針で使用する。
    #[allow(deprecated)]
    egui::show_tooltip_at_pointer(
        ui.ctx(),
        ui.layer_id(),
        egui::Id::new("node_check_tooltip"),
        |ui| {
            ui.label(format!("節点 {}", node.0));
            egui::Grid::new("node_check_tooltip_grid")
                .striped(true)
                .show(ui, |ui| {
                    ui.strong("種別");
                    ui.strong("検定比");
                    ui.strong("判定");
                    ui.strong("根拠");
                    ui.end_row();
                    for j in rows {
                        ui.label(&j.label);
                        match &j.outcome {
                            CheckOutcome::Checked(cr) => {
                                ui.label(format!("{:.2}", cr.ratio()));
                                if cr.ok() {
                                    ui.colored_label(theme::GOOD_GREEN, "OK");
                                } else {
                                    ui.colored_label(theme::PARETO_RED, "NG");
                                }
                                let mut detail = cr.basis.clone();
                                if let Some(c) = cr.components.first() {
                                    if !c.detail.is_empty() {
                                        detail.push_str(" / ");
                                        detail.push_str(&c.detail);
                                    }
                                }
                                ui.label(detail);
                            }
                            CheckOutcome::Skipped { reason } => {
                                ui.label("-");
                                ui.colored_label(theme::GRAY_600, "検定不能");
                                ui.label(reason);
                            }
                        }
                        ui.end_row();
                    }
                });
        },
    );
}

/// 部材中点ラベルの文字列を組み立てる（純粋関数）。支配式が分かる場合
/// （フィルタ=最大かつ components が非空）は「1.13 せん断」のように併記し、
/// それ以外（フィルタ=特定式、または components が空の部材）は数値のみ。
pub(super) fn mid_label_text(ratio: f64, dominant: Option<CheckKind>) -> String {
    match dominant {
        Some(k) => format!("{:.2} {}", ratio, k.label()),
        None => format!("{:.2}", ratio),
    }
}

/// 部材 `elem_id` の全検定位置（`xi` 昇順）を返す（純粋関数。ホバー詳細
/// ツールチップの表データ生成に使う）。`member_checks` は部材単位に
/// グループ化済みのため、線形走査ではなく直接引ける。
pub(super) fn elem_check_positions(
    member_checks: &[MemberChecks],
    elem_id: ElemId,
) -> &[PositionCheck] {
    member_checks
        .iter()
        .find(|m| m.elem == elem_id)
        .map(|m| m.positions.as_slice())
        .unwrap_or(&[])
}

/// ホバー詳細ツールチップの1行分（1検定位置）の判定表示。
pub(super) enum RowVerdict {
    Ok,
    Ng,
    /// 検定不能（理由）。
    Skipped(String),
}

/// ホバー詳細ツールチップの1行分（1検定位置）のデータ。
pub(super) struct TooltipRow {
    /// 検定位置 xi ∈ [0,1]
    pub xi: f64,
    /// 列（`kinds`）に対応する検定比。該当式がない列・検定不能の行は `None`。
    pub values: Vec<Option<f64>>,
    pub verdict: RowVerdict,
}

/// 部材1本分の「位置×式」ツールチップ表データを生成する（純粋関数）。
/// `positions` は当該部材の全検定位置（[`elem_check_positions`] の戻り値）。
///
/// 戻り値は `(列に出す式の集合＝出現順の CheckKind, 各行データ)`。
/// 検定不能（[`CheckOutcome::Skipped`]）の位置は列の判定には寄与せず
/// （`available_check_kinds` には含めない）、行の判定は
/// [`RowVerdict::Skipped`] になる。
pub(super) fn build_tooltip_rows(positions: &[PositionCheck]) -> (Vec<CheckKind>, Vec<TooltipRow>) {
    let kinds = available_check_kinds(positions.iter().filter_map(|p| match &p.outcome {
        CheckOutcome::Checked(cr) => Some(cr.components.as_slice()),
        CheckOutcome::Skipped { .. } => None,
    }));
    let rows = positions
        .iter()
        .map(|p| match &p.outcome {
            CheckOutcome::Checked(cr) => {
                let values = kinds
                    .iter()
                    .map(|k| {
                        cr.components
                            .iter()
                            .filter(|c| c.kind == *k)
                            .map(|c| c.ratio)
                            .fold(None, |acc: Option<f64>, r| {
                                Some(acc.map_or(r, |a| a.max(r)))
                            })
                    })
                    .collect();
                TooltipRow {
                    xi: p.xi,
                    values,
                    verdict: if cr.ok() {
                        RowVerdict::Ok
                    } else {
                        RowVerdict::Ng
                    },
                }
            }
            CheckOutcome::Skipped { reason } => TooltipRow {
                xi: p.xi,
                values: vec![None; kinds.len()],
                verdict: RowVerdict::Skipped(reason.clone()),
            },
        })
        .collect();
    (kinds, rows)
}

/// 検定比図を描く。`pts` は `viewer_panel` で計算済みの節点スクリーン座標
/// （`app.core.model.nodes` と同じ順序）。
pub(super) fn draw_check_ratio(
    painter: &egui::Painter,
    app: &App,
    model: &squid_n_core::model::Model,
    pts: &[egui::Pos2],
    frame_filter: super::FrameFilter,
) {
    let Some(results) = &app.core.scoped.results else {
        draw_no_result_legend(painter);
        return;
    };
    // 部材検定・節点検定のどちらかがあれば描画する（耐震壁のみのモデル等では
    // 部材検定が空でも節点検定だけが存在しうる）。
    if results.member_checks.is_empty() && results.joint_checks.is_empty() {
        draw_no_result_legend(painter);
        return;
    }

    let filter = app.ui.view.check_ratio_filter;
    let markers = app.ui.view.check_ratio_markers;
    let label_all = app.ui.view.check_ratio_label_all;

    let elem_ratios = max_ratio_by_elem(results.member_checks.iter().flat_map(|m| {
        m.positions
            .iter()
            .map(|p| (m.elem, ratio_for_filter(&p.outcome, filter)))
    }));
    let node_ratios = max_ratio_by_node(
        results
            .joint_checks
            .iter()
            .map(|j| (j.node, ratio_for_filter(&j.outcome, filter))),
    );

    // 部材ごとの検定位置索引（B-2 位置別マーカー・B-4 支配式ラベル用）。
    // `member_checks` は既に部材単位にグループ化済みのため、部材IDから直接
    // 引けるよう索引を作るだけでよい（位置ごとの全行線形走査は不要）。
    let checks_by_elem: HashMap<ElemId, &MemberChecks> =
        results.member_checks.iter().map(|m| (m.elem, m)).collect();

    // --- 部材の着色 ---
    for elem in &model.elements {
        if !frame_filter.shows(elem.id) {
            continue;
        }
        let Some(&(ratio, ok)) = elem_ratios.get(&elem.id) else {
            continue;
        };
        let color = theme::check_ratio_color(ratio);

        // 壁（面要素）: 半透明ポリゴンで塗り、輪郭を検定比の色で強調する
        if elem.kind == squid_n_core::model::ElementKind::Wall && elem.nodes.len() >= 3 {
            let poly: Vec<egui::Pos2> = elem
                .nodes
                .iter()
                .filter_map(|n| {
                    let idx = n.index();
                    (idx < pts.len()).then(|| pts[idx])
                })
                .collect();
            if poly.len() == elem.nodes.len() {
                painter.add(egui::Shape::convex_polygon(
                    poly,
                    theme::translucent(color, 70),
                    egui::Stroke::new(2.0_f32, color),
                ));
            }
            continue;
        }

        // 線材: 両端を結ぶ線を検定比の色で描き、中点に数値ラベルを添える。
        if elem.nodes.len() < 2 {
            continue;
        }
        let n0 = elem.nodes[0].index();
        let n1 = elem.nodes[1].index();
        if n0 >= pts.len() || n1 >= pts.len() {
            continue;
        }
        let p0 = pts[n0];
        let p1 = pts[n1];
        // NG 部材は太さで目立たせる
        let width = if ok { 4.0_f32 } else { 5.0_f32 };
        painter.line_segment([p0, p1], egui::Stroke::new(width, color));

        let positions: &[PositionCheck] = checks_by_elem
            .get(&elem.id)
            .map(|m| m.positions.as_slice())
            .unwrap_or(&[]);

        // B-2: 位置別マーカー（検定位置ごとに正方形。フィルタ対象外の位置
        // （検定不能を含む）は描かない）。
        if markers {
            for p in positions {
                let Some((r, _)) = ratio_for_filter(&p.outcome, filter) else {
                    continue;
                };
                let xi = p.xi;
                let mx = p0.x + (p1.x - p0.x) * xi as f32;
                let my = p0.y + (p1.y - p0.y) * xi as f32;
                let mcolor = theme::check_ratio_color(r);
                const MARK: f32 = 7.0;
                let mrect =
                    egui::Rect::from_center_size(egui::pos2(mx, my), egui::vec2(MARK, MARK));
                painter.rect_filled(mrect, 0.0, mcolor);
                painter.rect_stroke(
                    mrect,
                    0.0,
                    egui::Stroke::new(1.0_f32, theme::WHITE),
                    egui::StrokeKind::Middle,
                );
                // NG の位置のみ数値ラベルを添える（全位置に出すと過密になるため）。
                if r > 1.0 {
                    painter.text(
                        egui::pos2(mrect.max.x + 2.0, mrect.min.y),
                        egui::Align2::LEFT_BOTTOM,
                        format!("{:.2}", r),
                        egui::FontId::proportional(10.0),
                        theme::PARETO_RED,
                    );
                }
            }
        }

        // B-4: 中点ラベル（部材内最大＝ratio）。過密を避けるため注意域以上
        // （既定。`should_label` を参照）の部材にのみ描く。
        if !should_label(ratio, label_all) {
            continue;
        }

        // フィルタ=最大のときは支配式を併記する
        // （検定不能の位置は対象外。Checked の中から最大を選ぶ）。
        let dominant = if filter == CheckRatioFilter::Max {
            positions
                .iter()
                .filter_map(|p| match &p.outcome {
                    CheckOutcome::Checked(cr) => Some(cr),
                    CheckOutcome::Skipped { .. } => None,
                })
                .max_by(|a, b| {
                    a.ratio()
                        .partial_cmp(&b.ratio())
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .and_then(dominant_kind)
        } else {
            None
        };
        let mid = egui::pos2((p0.x + p1.x) * 0.5, (p0.y + p1.y) * 0.5);
        let (font_size, label_color) = if ok {
            (11.0, theme::GRAY_700)
        } else {
            // NG はフォントを大きくし赤字で目立たせる
            (12.0, theme::PARETO_RED)
        };
        painter.text(
            mid,
            egui::Align2::CENTER_BOTTOM,
            mid_label_text(ratio, dominant),
            egui::FontId::proportional(font_size),
            label_color,
        );
    }

    // --- 節点検定（接合部・パネルゾーン・耐震壁など）の表示 ---
    // NodeId の内部値はそのまま配列添字とは限らないため、`app.core.model.nodes` を
    // 走査してインデックスを求め（`enumerate` の添字が実際の `pts` の添字）、
    // `node.id` と突き合わせてから `pts` を引く。
    //
    // 節点には支点記号・ヒンジ等の他の記号も重なるため、部材の線とは別形状
    // （ひし形）で描いて識別できるようにする。NG は一回り大きくし、輪郭を
    // 背景色で縁取って他の記号に埋もれないようにする。
    for (idx, node) in app.core.model.nodes.iter().enumerate() {
        let Some(&(ratio, ok)) = node_ratios.get(&node.id) else {
            continue;
        };
        if idx >= pts.len() {
            continue;
        }
        let p = pts[idx];
        let color = theme::check_ratio_color(ratio);
        let r = if ok {
            NODE_MARKER_RADIUS
        } else {
            NODE_MARKER_RADIUS_NG
        };
        let diamond = vec![
            egui::pos2(p.x, p.y - r),
            egui::pos2(p.x + r, p.y),
            egui::pos2(p.x, p.y + r),
            egui::pos2(p.x - r, p.y),
        ];
        painter.add(egui::Shape::convex_polygon(
            diamond,
            color,
            egui::Stroke::new(1.5_f32, theme::VIEW_BG),
        ));
        // 「全ラベル表示」ON、または NG の節点は検定比を数値で添える
        // （部材の中央ラベルと同じ規約）。
        if label_all || !ok {
            let (font_size, label_color) = if ok {
                (11.0, theme::GRAY_700)
            } else {
                (12.0, theme::PARETO_RED)
            };
            painter.text(
                egui::pos2(p.x, p.y - r - 1.0),
                egui::Align2::CENTER_BOTTOM,
                format!("{:.2}", ratio),
                egui::FontId::proportional(font_size),
                label_color,
            );
        }
    }

    draw_legend(
        painter,
        app,
        &elem_ratios,
        &node_ratios,
        filter,
        markers,
        label_all,
    );
}

/// B-3: 部材 `elem_id` の検定詳細（位置×式）をポインタ位置にツールチップ表示する。
/// `app.core.scoped.results.member_checks` に当該部材の検定がなければ何も描かない。
pub(super) fn show_check_tooltip(ui: &egui::Ui, app: &App, elem_id: ElemId) {
    let Some(results) = &app.core.scoped.results else {
        return;
    };
    let positions = elem_check_positions(&results.member_checks, elem_id);
    if positions.is_empty() {
        return;
    }
    // ヘッダに添える根拠・理由: 先頭位置の検定結果（Checked なら basis、
    // Skipped なら reason）を代表値として使う。
    let basis = match &positions[0].outcome {
        CheckOutcome::Checked(cr) => cr.basis.clone(),
        CheckOutcome::Skipped { reason } => reason.clone(),
    };
    let (kinds, rows) = build_tooltip_rows(positions);

    // `show_tooltip_at_pointer` は egui 0.34 で非推奨（`Tooltip` 型を使う新 API へ
    // 移行中）だが、ウィジェットに紐付かない任意位置へのツールチップ表示という
    // 用途には他に簡潔な代替がないため、既存コード（app/panels.rs）と同じ方針で
    // `#[allow(deprecated)]` を付けて使用する。
    #[allow(deprecated)]
    egui::show_tooltip_at_pointer(
        ui.ctx(),
        ui.layer_id(),
        egui::Id::new("check_ratio_tooltip"),
        |ui| {
            ui.label(format!("部材 #{} ({basis})", elem_id.0));
            egui::Grid::new("check_ratio_tooltip_grid")
                .striped(true)
                .show(ui, |ui| {
                    ui.label("位置");
                    for k in &kinds {
                        ui.label(k.label());
                    }
                    ui.label("判定");
                    ui.end_row();
                    for row in &rows {
                        ui.label(format!("{:.2}", row.xi));
                        for v in &row.values {
                            match v {
                                Some(r) => {
                                    ui.colored_label(theme::status_color(*r), format!("{r:.2}"));
                                }
                                None => {
                                    ui.label("-");
                                }
                            }
                        }
                        match &row.verdict {
                            RowVerdict::Ok => {
                                ui.label("OK");
                            }
                            RowVerdict::Ng => {
                                ui.colored_label(theme::PARETO_RED, "NG");
                            }
                            RowVerdict::Skipped(reason) => {
                                ui.colored_label(theme::GRAY_600, format!("検定不能（{reason}）"));
                            }
                        }
                        ui.end_row();
                    }
                });
        },
    );
}

/// 検定結果がない場合の案内表示。
fn draw_no_result_legend(painter: &egui::Painter) {
    painter.text(
        egui::pos2(
            painter.clip_rect().min.x + 10.0,
            painter.clip_rect().min.y + 10.0,
        ),
        egui::Align2::LEFT_TOP,
        "検定結果がありません。解析タブから静的解析を実行してください。",
        egui::FontId::proportional(14.0),
        theme::GRAY_600,
    );
}

/// 検定式フィルタの表示名（凡例タイトル用）。
fn filter_label(filter: CheckRatioFilter) -> &'static str {
    match filter {
        CheckRatioFilter::Max => "最大",
        CheckRatioFilter::Kind(k) => k.label(),
    }
}

/// 凡例のカラーバー（検定比 0.0〜1.0 の連続グラデーション＋NG の単色見本）を
/// 左上 `(x0, y0)` に描き、描いた領域の下端 y 座標を返す。
///
/// NG（>1.0）は連続値ではなく単色（赤）のため、グラデーションのバーには含めず
/// 右隣に独立した色見本として並べる。
fn draw_ratio_color_bar(painter: &egui::Painter, x0: f32, y0: f32) -> f32 {
    const BAR_W: f32 = 160.0;
    const BAR_H: f32 = 10.0;
    const STRIPS: usize = 32;
    const SWATCH: f32 = 12.0;

    for i in 0..STRIPS {
        // 短冊の中央に相当する検定比（0〜1.0 を等分）
        let ratio = (i as f64 + 0.5) / STRIPS as f64;
        let sx0 = x0 + (i as f32 / STRIPS as f32) * BAR_W;
        let sx1 = x0 + ((i + 1) as f32 / STRIPS as f32) * BAR_W;
        painter.rect_filled(
            egui::Rect::from_min_max(egui::pos2(sx0, y0), egui::pos2(sx1, y0 + BAR_H)),
            0.0,
            theme::check_ratio_color(ratio),
        );
    }

    let ng_x = x0 + BAR_W + 10.0;
    painter.rect_filled(
        egui::Rect::from_min_size(egui::pos2(ng_x, y0), egui::vec2(SWATCH, BAR_H)),
        0.0,
        theme::PARETO_RED,
    );
    let font = egui::FontId::proportional(11.0);
    painter.text(
        egui::pos2(ng_x + SWATCH + 4.0, y0),
        egui::Align2::LEFT_TOP,
        ">1.0 NG",
        font.clone(),
        theme::GRAY_600,
    );

    // 目盛り（0 / 0.8 / 1.0）をバーの下端に添える。0.8 は良好域と注意域の境界。
    let ty = y0 + BAR_H + 1.0;
    let tick = painter.text(
        egui::pos2(x0, ty),
        egui::Align2::LEFT_TOP,
        "0",
        font.clone(),
        theme::GRAY_600,
    );
    painter.text(
        egui::pos2(x0 + BAR_W * 0.8, ty),
        egui::Align2::CENTER_TOP,
        "0.8",
        font.clone(),
        theme::GRAY_600,
    );
    painter.text(
        egui::pos2(x0 + BAR_W, ty),
        egui::Align2::RIGHT_TOP,
        "1.0",
        font.clone(),
        theme::GRAY_600,
    );
    let untested = painter.text(
        egui::pos2(ng_x, ty),
        egui::Align2::LEFT_TOP,
        "未検定・検定不能: グレー",
        font,
        theme::GRAY_600,
    );
    tick.max.y.max(untested.max.y)
}

/// ビュー左上に検定比図の凡例（対象・最大値・NG件数・カラーバー・陳腐化注記）を描く。
#[allow(clippy::too_many_arguments)]
fn draw_legend(
    painter: &egui::Painter,
    app: &App,
    elem_ratios: &HashMap<ElemId, (f64, bool)>,
    node_ratios: &HashMap<NodeId, (f64, bool)>,
    filter: CheckRatioFilter,
    markers: bool,
    label_all: bool,
) {
    let rect = painter.clip_rect();
    let x0 = rect.min.x + 10.0;
    let mut y = rect.min.y + 10.0;

    let max_ratio = elem_ratios
        .values()
        .chain(node_ratios.values())
        .map(|&(r, _)| r)
        .fold(0.0_f64, f64::max);
    let ng_count = elem_ratios
        .values()
        .chain(node_ratios.values())
        .filter(|&&(_, ok)| !ok)
        .count();

    let title_rect = painter.text(
        egui::pos2(x0, y),
        egui::Align2::LEFT_TOP,
        format!(
            "検定比図 (対象: {}, max={:.2}, NG {}件)",
            filter_label(filter),
            max_ratio,
            ng_count
        ),
        egui::FontId::proportional(14.0),
        theme::GRAY_700,
    );
    y = title_rect.max.y + 4.0;

    // 色の凡例: 0〜1.0 の連続グラデーションのカラーバー＋NG（赤）の単色見本
    y = draw_ratio_color_bar(painter, x0, y) + 4.0;

    // 数値ラベルの表示条件（既定は注意域以上のみ）と位置別マーカーの説明
    let mut note = if label_all {
        "数値ラベル: 全部材".to_string()
    } else {
        format!("数値ラベル: 検定比 {:.1} 以上のみ", LABEL_MIN_RATIO)
    };
    if markers {
        note.push_str("　■ 検定位置（NG は数値付き）");
    }
    let note_rect = painter.text(
        egui::pos2(x0, y),
        egui::Align2::LEFT_TOP,
        note,
        egui::FontId::proportional(11.0),
        theme::GRAY_600,
    );
    y = note_rect.max.y + 4.0;

    // 節点検定（接合部・仕口パネル・耐震壁）の記号説明。部材の線と区別できるよう
    // ひし形で描くため、凡例でもその旨を示す。
    if !node_ratios.is_empty() {
        let node_note = painter.text(
            egui::pos2(x0, y),
            egui::Align2::LEFT_TOP,
            format!(
                "◆ 節点検定 {} 箇所（接合部・仕口パネル・耐震壁）　ホバーで種別と根拠を表示",
                node_ratios.len()
            ),
            egui::FontId::proportional(11.0),
            theme::GRAY_600,
        );
        y = node_note.max.y + 4.0;
    }

    if app.core.scoped.staleness.design_stale {
        painter.text(
            egui::pos2(x0, y),
            egui::Align2::LEFT_TOP,
            "⚠ モデルが編集されています。解析を再実行してください。",
            egui::FontId::proportional(12.0),
            theme::WARN_TEXT,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn checked(ratio: f64, components: Vec<CheckComponent>) -> CheckOutcome {
        CheckOutcome::Checked(CheckResult {
            basis: "テスト規準".to_string(),
            detail: String::new(),
            components: if components.is_empty() {
                vec![CheckComponent {
                    kind: CheckKind::Bending,
                    ratio,
                    detail: String::new(),
                }]
            } else {
                components
            },
        })
    }

    fn skipped(reason: &str) -> CheckOutcome {
        CheckOutcome::Skipped {
            reason: reason.to_string(),
        }
    }

    // ── max_ratio_by_elem / max_ratio_by_node ──────────────────────────

    /// 同一部材に複数の検定位置がある場合、最大の検定比が採用される。
    #[test]
    fn max_ratio_by_elem_picks_max_ratio() {
        let id = ElemId(0);
        let map = max_ratio_by_elem([
            (id, Some((0.5, true))),
            (id, Some((0.9, true))),
            (id, Some((0.3, true))),
        ]);
        assert_eq!(map[&id].0, 0.9);
    }

    /// 1つでも NG（ok=false）の位置があれば、部材全体として NG（false）になる。
    #[test]
    fn max_ratio_by_elem_ng_propagates() {
        let id = ElemId(1);
        let map = max_ratio_by_elem([
            (id, Some((0.5, true))),
            (id, Some((1.2, false))),
            (id, Some((0.3, true))),
        ]);
        assert_eq!(map[&id].0, 1.2);
        assert!(!map[&id].1);
    }

    /// 全位置が OK なら OK フラグは true のまま。
    #[test]
    fn max_ratio_by_elem_all_ok_stays_ok() {
        let id = ElemId(2);
        let map = max_ratio_by_elem([(id, Some((0.4, true))), (id, Some((0.6, true)))]);
        assert!(map[&id].1);
    }

    /// 複数部材のデータは部材ごとに分離して集計される。
    #[test]
    fn max_ratio_by_elem_separates_by_id() {
        let a = ElemId(0);
        let b = ElemId(1);
        let map = max_ratio_by_elem([
            (a, Some((0.5, true))),
            (b, Some((1.5, false))),
            (a, Some((0.8, true))),
        ]);
        assert_eq!(map.len(), 2);
        assert_eq!(map[&a].0, 0.8);
        assert!(map[&a].1);
        assert_eq!(map[&b].0, 1.5);
        assert!(!map[&b].1);
    }

    /// 空入力は空の集計結果を返す。
    #[test]
    fn max_ratio_by_elem_empty_input() {
        let map = max_ratio_by_elem(std::iter::empty::<(ElemId, Option<(f64, bool)>)>());
        assert!(map.is_empty());
    }

    /// フィルタ対象外（None）の位置は集計から除外される。全位置が None なら
    /// 部材自体が集計結果に含まれない（＝未検定として扱われ着色されない）。
    #[test]
    fn max_ratio_by_elem_none_is_excluded() {
        let id = ElemId(3);
        let map = max_ratio_by_elem([(id, None), (id, Some((0.6, true))), (id, None)]);
        assert_eq!(map.len(), 1);
        assert_eq!(map[&id].0, 0.6);

        let id2 = ElemId(4);
        let map2 = max_ratio_by_elem([(id2, None), (id2, None)]);
        assert!(!map2.contains_key(&id2));
    }

    /// 節点単位の集計も同じ規則（最大値採用・NG 伝播）で動作する。
    #[test]
    fn max_ratio_by_node_picks_max_and_propagates_ng() {
        let n = NodeId(0);
        let map = max_ratio_by_node([(n, Some((0.7, true))), (n, Some((1.1, false)))]);
        assert_eq!(map[&n].0, 1.1);
        assert!(!map[&n].1);
    }

    /// 節点集計は複数節点を分離して保持する。
    #[test]
    fn max_ratio_by_node_separates_by_id() {
        let a = NodeId(0);
        let b = NodeId(1);
        let map = max_ratio_by_node([(a, Some((0.2, true))), (b, Some((0.95, true)))]);
        assert_eq!(map.len(), 2);
        assert_eq!(map[&a].0, 0.2);
        assert_eq!(map[&b].0, 0.95);
    }

    /// 節点集計の空入力は空の結果を返す。
    #[test]
    fn max_ratio_by_node_empty_input() {
        let map = max_ratio_by_node(std::iter::empty::<(NodeId, Option<(f64, bool)>)>());
        assert!(map.is_empty());
    }

    // ── ratio_for_filter ────────────────────────────────────────────────

    /// フィルタ=最大は cr.ratio() / cr.ok() をそのまま返す。
    #[test]
    fn ratio_for_filter_max_returns_ratio_and_ok() {
        let c = checked(
            1.13,
            vec![CheckComponent {
                kind: CheckKind::Shear,
                ratio: 1.13,
                detail: String::new(),
            }],
        );
        assert_eq!(
            ratio_for_filter(&c, CheckRatioFilter::Max),
            Some((1.13, false))
        );
    }

    /// フィルタ=特定式は該当式の最大検定比を返し、OK 判定は 1.0 以下かで決まる。
    #[test]
    fn ratio_for_filter_kind_picks_matching_component() {
        let c = checked(
            1.13,
            vec![
                CheckComponent {
                    kind: CheckKind::Bending,
                    ratio: 0.82,
                    detail: String::new(),
                },
                CheckComponent {
                    kind: CheckKind::Shear,
                    ratio: 1.13,
                    detail: String::new(),
                },
            ],
        );
        assert_eq!(
            ratio_for_filter(&c, CheckRatioFilter::Kind(CheckKind::Bending)),
            Some((0.82, true))
        );
        assert_eq!(
            ratio_for_filter(&c, CheckRatioFilter::Kind(CheckKind::Shear)),
            Some((1.13, false))
        );
    }

    /// 該当する式が components になければ None（フィルタ対象外）。
    #[test]
    fn ratio_for_filter_kind_absent_returns_none() {
        let c = checked(
            0.5,
            vec![CheckComponent {
                kind: CheckKind::Bending,
                ratio: 0.5,
                detail: String::new(),
            }],
        );
        assert_eq!(
            ratio_for_filter(&c, CheckRatioFilter::Kind(CheckKind::Axial)),
            None
        );
    }

    /// 同一 kind の component が複数ある場合は最大値を採用する。
    #[test]
    fn ratio_for_filter_kind_multiple_same_kind_picks_max() {
        let c = checked(
            0.9,
            vec![
                CheckComponent {
                    kind: CheckKind::Shear,
                    ratio: 0.4,
                    detail: String::new(),
                },
                CheckComponent {
                    kind: CheckKind::Shear,
                    ratio: 0.9,
                    detail: String::new(),
                },
            ],
        );
        assert_eq!(
            ratio_for_filter(&c, CheckRatioFilter::Kind(CheckKind::Shear)),
            Some((0.9, true))
        );
    }

    /// 検定不能（Skipped）はフィルタ種別によらず常に None。
    #[test]
    fn ratio_for_filter_skipped_returns_none() {
        let s = skipped("Fc 未設定");
        assert_eq!(ratio_for_filter(&s, CheckRatioFilter::Max), None);
        assert_eq!(
            ratio_for_filter(&s, CheckRatioFilter::Kind(CheckKind::Bending)),
            None
        );
    }

    // ── dominant_kind ───────────────────────────────────────────────────

    /// 最大検定比を与える component の kind を返す。
    #[test]
    fn dominant_kind_picks_max_component() {
        let cr = CheckResult {
            basis: String::new(),
            detail: String::new(),
            components: vec![
                CheckComponent {
                    kind: CheckKind::Bending,
                    ratio: 0.82,
                    detail: String::new(),
                },
                CheckComponent {
                    kind: CheckKind::Shear,
                    ratio: 1.13,
                    detail: String::new(),
                },
            ],
        };
        assert_eq!(dominant_kind(&cr), Some(CheckKind::Shear));
    }

    // ── available_check_kinds ───────────────────────────────────────────

    /// 出現した kind のみを CheckKind の定義順で返す。
    #[test]
    fn available_check_kinds_returns_present_kinds_in_definition_order() {
        let comps: Vec<Vec<CheckComponent>> = vec![
            vec![CheckComponent {
                kind: CheckKind::Shear,
                ratio: 0.5,
                detail: String::new(),
            }],
            vec![CheckComponent {
                kind: CheckKind::Bending,
                ratio: 0.6,
                detail: String::new(),
            }],
        ];
        let kinds = available_check_kinds(comps.iter().map(|c| c.as_slice()));
        // 定義順は Bending が Shear より先。
        assert_eq!(kinds, vec![CheckKind::Bending, CheckKind::Shear]);
    }

    /// 無関係な式（例: 軸力のみのモデルに存在しない「たわみ」）は含まれない。
    #[test]
    fn available_check_kinds_excludes_absent_kinds() {
        let comps: Vec<Vec<CheckComponent>> = vec![vec![CheckComponent {
            kind: CheckKind::Axial,
            ratio: 0.3,
            detail: String::new(),
        }]];
        let kinds = available_check_kinds(comps.iter().map(|c| c.as_slice()));
        assert_eq!(kinds, vec![CheckKind::Axial]);
    }

    /// 空入力は空の結果を返す。
    #[test]
    fn available_check_kinds_empty_input() {
        let kinds = available_check_kinds(std::iter::empty::<&[CheckComponent]>());
        assert!(kinds.is_empty());
    }

    // ── should_label ────────────────────────────────────────────────────

    /// 既定では注意域の境界（0.8）以上の部材にのみ数値ラベルを描く。
    #[test]
    fn should_label_only_at_or_above_threshold_by_default() {
        assert!(!should_label(0.0, false));
        assert!(!should_label(0.79, false));
        assert!(should_label(LABEL_MIN_RATIO, false));
        assert!(should_label(0.81, false));
        assert!(should_label(1.5, false));
    }

    /// 全表示（label_all）では検定比によらず常に描く。
    #[test]
    fn should_label_all_shows_every_ratio() {
        assert!(should_label(0.0, true));
        assert!(should_label(0.79, true));
        assert!(should_label(1.5, true));
    }

    // ── mid_label_text ──────────────────────────────────────────────────

    /// 支配式が分かる場合は数値と式名を併記する。
    #[test]
    fn mid_label_text_with_dominant() {
        assert_eq!(mid_label_text(1.13, Some(CheckKind::Shear)), "1.13 せん断");
    }

    /// 支配式がない場合（フィルタ=特定式、または内訳なし）は数値のみ。
    #[test]
    fn mid_label_text_without_dominant() {
        assert_eq!(mid_label_text(0.82, None), "0.82");
    }

    // ── elem_check_positions / build_tooltip_rows ───────────────────────

    /// 指定した部材の検定位置のみを xi 順そのままに抽出する。
    #[test]
    fn elem_check_positions_filters_by_elem_id() {
        let a = ElemId(0);
        let b = ElemId(1);
        let member_checks = vec![
            MemberChecks {
                elem: a,
                positions: vec![
                    PositionCheck {
                        xi: 0.0,
                        outcome: checked(0.5, vec![]),
                    },
                    PositionCheck {
                        xi: 1.0,
                        outcome: checked(0.9, vec![]),
                    },
                ],
            },
            MemberChecks {
                elem: b,
                positions: vec![PositionCheck {
                    xi: 0.0,
                    outcome: checked(1.5, vec![]),
                }],
            },
        ];
        let positions = elem_check_positions(&member_checks, a);
        assert_eq!(positions.len(), 2);
        assert_eq!(positions[0].xi, 0.0);
        assert_eq!(positions[1].xi, 1.0);
    }

    /// 検定位置のない部材は空スライスを返す。
    #[test]
    fn elem_check_positions_unknown_elem_returns_empty() {
        let member_checks: Vec<MemberChecks> = vec![];
        let positions = elem_check_positions(&member_checks, ElemId(9));
        assert!(positions.is_empty());
    }

    /// 位置×式の表データが、出現した式を列に、位置ごとの値・判定を行に持つ。
    #[test]
    fn build_tooltip_rows_builds_table() {
        let positions = vec![
            PositionCheck {
                xi: 0.0,
                outcome: checked(
                    0.5,
                    vec![
                        CheckComponent {
                            kind: CheckKind::Bending,
                            ratio: 0.5,
                            detail: String::new(),
                        },
                        CheckComponent {
                            kind: CheckKind::Shear,
                            ratio: 0.4,
                            detail: String::new(),
                        },
                    ],
                ),
            },
            PositionCheck {
                xi: 0.5,
                outcome: checked(
                    1.13,
                    vec![CheckComponent {
                        kind: CheckKind::Shear,
                        ratio: 1.13,
                        detail: String::new(),
                    }],
                ),
            },
        ];
        let (kinds, rows) = build_tooltip_rows(&positions);
        assert_eq!(kinds, vec![CheckKind::Bending, CheckKind::Shear]);
        assert_eq!(rows.len(), 2);
        assert_eq!(rows[0].values, vec![Some(0.5), Some(0.4)]);
        assert!(matches!(rows[0].verdict, RowVerdict::Ok));
        // 2 行目は Bending 式がないため None。
        assert_eq!(rows[1].values, vec![None, Some(1.13)]);
        assert!(matches!(rows[1].verdict, RowVerdict::Ng));
    }

    /// 検定不能の位置は列の判定には寄与せず、行の判定は Skipped(理由) になる。
    #[test]
    fn build_tooltip_rows_skipped_position() {
        let positions = vec![PositionCheck {
            xi: 0.0,
            outcome: skipped("Fc 未設定"),
        }];
        let (kinds, rows) = build_tooltip_rows(&positions);
        assert!(kinds.is_empty());
        assert_eq!(rows.len(), 1);
        assert!(rows[0].values.is_empty());
        match &rows[0].verdict {
            RowVerdict::Skipped(reason) => assert_eq!(reason, "Fc 未設定"),
            _ => panic!("expected Skipped"),
        }
    }

    /// 検定位置がなければ表も空。
    #[test]
    fn build_tooltip_rows_empty_positions() {
        let (kinds, rows) = build_tooltip_rows(&[]);
        assert!(kinds.is_empty());
        assert!(rows.is_empty());
    }

    // ── 節点検定のホバー判定 ────────────────────────────────────

    /// 節点検定を持つ節点だけがホバー候補になり、最も近い節点が返る。
    /// 検定結果を持たない節点は、より近くても候補にならない。
    #[test]
    fn pick_nearest_checked_node_ignores_unchecked_nodes() {
        use crate::app::JointCheck;
        use squid_n_core::model::Node;

        let mut app = App::default();
        let node = |id: u32, x: f64| Node {
            id: NodeId(id),
            coord: [x, 0.0, 0.0],
            restraint: squid_n_core::dof::Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        app.core.model.nodes = vec![node(0, 0.0), node(1, 1000.0), node(2, 2000.0)];

        let mut results = crate::app::ResultsBundle::default();
        // 節点 0 と 2 のみ検定結果を持つ。
        for id in [0u32, 2] {
            results.joint_checks.push(JointCheck {
                node: NodeId(id),
                label: "パネルゾーン(S)".into(),
                outcome: checked(0.5, vec![]),
            });
        }
        app.core.scoped.results = Some(results);

        let pts = [
            egui::pos2(0.0, 0.0),
            egui::pos2(10.0, 0.0),
            egui::pos2(100.0, 0.0),
        ];

        // 節点 1（検定なし）のすぐ近くでも、候補になるのは検定を持つ節点 0。
        let hit = pick_nearest_checked_node(&app, &pts, egui::pos2(11.0, 0.0), Default::default())
            .expect("ヒット");
        assert_eq!(hit.0, 0, "検定を持たない節点は候補にしない");

        // 節点 2 の近くではその節点が返る。
        let hit = pick_nearest_checked_node(&app, &pts, egui::pos2(98.0, 0.0), Default::default())
            .expect("ヒット");
        assert_eq!(hit.0, 2);
        assert!(hit.1 <= NODE_HOVER_THRESHOLD);
    }

    /// 検定結果がまったくなければホバー候補もない。
    #[test]
    fn pick_nearest_checked_node_without_results() {
        let app = App::default();
        assert!(pick_nearest_checked_node(
            &app,
            &[egui::pos2(0.0, 0.0)],
            egui::pos2(0.0, 0.0),
            Default::default()
        )
        .is_none());
    }

    /// マーカー半径とホバー判定しきい値の大小関係を保つ。
    ///
    /// NG のマーカーは OK より大きく描き（他の節点記号に埋もれないようにする）、
    /// ホバー判定はマーカーより広く取る（マーカーの縁でも詳細を出せるようにする）。
    /// 値を調整したときに関係が崩れていないかを押さえる。
    #[test]
    fn node_marker_radii_keep_ordering() {
        let (ok, ng, hover) = (
            NODE_MARKER_RADIUS,
            NODE_MARKER_RADIUS_NG,
            NODE_HOVER_THRESHOLD,
        );
        assert!(ng > ok, "NG のマーカーは OK より大きい: {ng} vs {ok}");
        assert!(hover >= ng, "ホバー判定はマーカーより広い: {hover} vs {ng}");
    }
}
