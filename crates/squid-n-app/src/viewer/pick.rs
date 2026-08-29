//! ノード/部材ピック。
//!
//! `viewer` ハブからの構造分割。アルゴリズム変更は行わない。

use super::{
    scene::{element_draw_shape, DrawShape},
    FrameFilter,
};

fn dist_point_to_segment(p: egui::Pos2, a: egui::Pos2, b: egui::Pos2) -> f32 {
    let ab = b - a;
    let ap = p - a;
    let len_sq = ab.x * ab.x + ab.y * ab.y;
    if len_sq < 1e-6 {
        return ap.length();
    }
    let t = ((ap.x * ab.x + ap.y * ab.y) / len_sq).clamp(0.0, 1.0);
    let proj = egui::pos2(a.x + ab.x * t, a.y + ab.y * t);
    (p - proj).length()
}

/// スクリーン上の多角形に点 `p` が含まれるか（辺上は含む）。
fn point_in_polygon(p: egui::Pos2, poly: &[egui::Pos2]) -> bool {
    if poly.len() < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        let pi = poly[i];
        let pj = poly[j];
        let intersects = (pi.y > p.y) != (pj.y > p.y)
            && p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x;
        if intersects {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// 点 `p` から多角形までの距離 [px]。内側なら 0、外側なら最短辺距離。
fn dist_point_to_polygon(p: egui::Pos2, poly: &[egui::Pos2]) -> f32 {
    if poly.len() < 3 {
        return f32::INFINITY;
    }
    if point_in_polygon(p, poly) {
        return 0.0;
    }
    let mut best = f32::INFINITY;
    for i in 0..poly.len() {
        let j = (i + 1) % poly.len();
        best = best.min(dist_point_to_segment(p, poly[i], poly[j]));
    }
    best
}

/// ピック距離の比較用スコア。面要素（壁・シェル）は大梁の下辺と重なる辺で
/// 線材と同距離になったとき壁を優先するため、微小なバイアスを掛ける。
fn pick_score(shape: DrawShape, dist_px: f32) -> f32 {
    match shape {
        DrawShape::Polygon => dist_px - 0.5,
        DrawShape::Line => dist_px,
        DrawShape::None => f32::INFINITY,
    }
}

/// 部材荷重を載せられる部材か（荷重のピック対象の判定）。
///
/// ソルバが等価節点力を配れる 2 節点の線材に限る
/// （`squid_n_solver` の `is_member_load_target` と同じ集合）。壁・スラブ等の
/// 面要素は先頭 2 節点を材端とみなして荷重が誤適用されるため対象外にする。
pub(super) fn member_load_pickable(
    model: &squid_n_core::model::Model,
    id: squid_n_core::ids::ElemId,
) -> bool {
    use squid_n_core::model::ElementKind;
    model.elements.iter().any(|e| {
        e.id == id
            && e.nodes.len() == 2
            && matches!(
                e.kind,
                ElementKind::Beam
                    | ElementKind::Fiber
                    | ElementKind::MultiSpring
                    | ElementKind::Brace { .. }
            )
    })
}

/// スクリーン座標 `pos` に最も近い節点の `(index, 距離px)` を返す（同距離は先勝ち）。
/// ピッキング（節点選択・作成モード）で共有する。`visible` が偽の節点は
/// ビューに描かれていないため対象外にする（見えない点が選ばれるのを防ぐ）。
pub(super) fn pick_nearest_node(
    pts: &[egui::Pos2],
    visible: &[bool],
    pos: egui::Pos2,
) -> Option<(usize, f32)> {
    let mut best: Option<(usize, f32)> = None;
    for (i, &p) in pts.iter().enumerate() {
        if !visible.get(i).copied().unwrap_or(true) {
            continue;
        }
        let d = (pos - p).length();
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((i, d));
        }
    }
    best
}

/// スクリーン座標 `pos` に最も近い部材の `(ElemId, 距離px)` を返す。
///
/// 線材は先頭 2 節点の線分距離、壁・シェルは多角形の内側（距離 0）または辺距離。
/// 2 節点未満の要素・節点参照が範囲外の要素は対象外。部材ピック・ホバーで共有する。
pub(super) fn pick_nearest_member(
    model: &squid_n_core::model::Model,
    pts: &[egui::Pos2],
    pos: egui::Pos2,
    filter: FrameFilter,
) -> Option<(squid_n_core::ids::ElemId, f32)> {
    let mut best: Option<(squid_n_core::ids::ElemId, f32)> = None;
    let mut best_score = f32::INFINITY;
    for elem in &model.elements {
        // 構面表示で描いていない部材は選べない（見えない部材のツールチップが
        // 出る・見えない部材が選択されるのを防ぐ）。
        if !filter.shows(elem.id) {
            continue;
        }
        if elem.nodes.len() < 2 {
            continue;
        }
        let shape = element_draw_shape(elem.kind);
        // 描かない要素（仕口パネル）はピック対象から外す（`element_draw_shape`）。
        // 節点列が「接合部の節点 ＋ 取り付く部材の他端」であり、先頭 2 節点を
        // 結んでも部材の線にはならない（取り付く部材の 1 本と同じ線分になり、
        // 実部材の選択・ホバーを横取りする）。面要素は描いているので対象に残す。
        if shape == DrawShape::None {
            continue;
        }
        let d = match shape {
            DrawShape::Line => {
                let n0 = elem.nodes[0].index();
                let n1 = elem.nodes[1].index();
                if n0 >= pts.len() || n1 >= pts.len() {
                    continue;
                }
                dist_point_to_segment(pos, pts[n0], pts[n1])
            }
            DrawShape::Polygon => {
                let poly: Vec<egui::Pos2> = elem
                    .nodes
                    .iter()
                    .filter_map(|n| {
                        let i = n.index();
                        (i < pts.len()).then(|| pts[i])
                    })
                    .collect();
                if poly.len() != elem.nodes.len() {
                    continue;
                }
                dist_point_to_polygon(pos, &poly)
            }
            DrawShape::None => continue,
        };
        let score = pick_score(shape, d);
        if score < best_score {
            best_score = score;
            best = Some((elem.id, d));
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_in_polygon_detects_interior() {
        let poly = vec![
            egui::pos2(0.0, 0.0),
            egui::pos2(100.0, 0.0),
            egui::pos2(100.0, 100.0),
            egui::pos2(0.0, 100.0),
        ];
        assert!(point_in_polygon(egui::pos2(50.0, 50.0), &poly));
        assert!(!point_in_polygon(egui::pos2(150.0, 50.0), &poly));
    }

    #[test]
    fn polygon_interior_beats_nearby_segment() {
        let poly = vec![
            egui::pos2(0.0, 0.0),
            egui::pos2(100.0, 0.0),
            egui::pos2(100.0, 100.0),
            egui::pos2(0.0, 100.0),
        ];
        let interior = dist_point_to_polygon(egui::pos2(50.0, 50.0), &poly);
        let on_bottom = dist_point_to_segment(egui::pos2(50.0, 50.0), poly[0], poly[1]);
        assert_eq!(interior, 0.0);
        assert!(pick_score(DrawShape::Polygon, interior) < pick_score(DrawShape::Line, on_bottom));
    }
}
