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

/// スクリーン座標 `pos` に最も近い部材（2 節点線分）の `(ElemId, 距離px)` を返す。
/// 2 節点未満の要素・節点参照が範囲外の要素は対象外。部材ピック・ホバーで共有する。
pub(super) fn pick_nearest_member(
    model: &squid_n_core::model::Model,
    pts: &[egui::Pos2],
    pos: egui::Pos2,
    filter: FrameFilter,
) -> Option<(squid_n_core::ids::ElemId, f32)> {
    let mut best: Option<(squid_n_core::ids::ElemId, f32)> = None;
    for elem in &model.elements {
        // 構面表示で描いていない部材は選べない（見えない部材のツールチップが
        // 出る・見えない部材が選択されるのを防ぐ）。
        if !filter.shows(elem.id) {
            continue;
        }
        if elem.nodes.len() < 2 {
            continue;
        }
        // 描かない要素（仕口パネル）はピック対象から外す（`element_draw_shape`）。
        // 節点列が「接合部の節点 ＋ 取り付く部材の他端」であり、先頭 2 節点を
        // 結んでも部材の線にはならない（取り付く部材の 1 本と同じ線分になり、
        // 実部材の選択・ホバーを横取りする）。面要素は描いているので対象に残す。
        if element_draw_shape(elem.kind) == DrawShape::None {
            continue;
        }
        let n0 = elem.nodes[0].index();
        let n1 = elem.nodes[1].index();
        if n0 >= pts.len() || n1 >= pts.len() {
            continue;
        }
        let d = dist_point_to_segment(pos, pts[n0], pts[n1]);
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((elem.id, d));
        }
    }
    best
}
