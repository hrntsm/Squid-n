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

use std::collections::HashMap;

use crate::app::App;
use crate::theme;
use squid_n_core::ids::ElemId;
use squid_n_solver::pushover::{HingeEvent, HingeLevel};

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
}
