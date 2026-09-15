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
        if !filter.shows(elem.id) {
            continue;
        }
        if elem.nodes.len() < 2 {
            continue;
        }
        let shape = element_draw_shape(elem.kind);
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
/// 3D ビューでクリック／ホバーした割当領域。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum RegionPick {
    Floor(squid_n_core::ids::FloorPlateAssignmentRegionId),
    Wall(squid_n_core::ids::WallPlateAssignmentRegionId),
}

/// スクリーン座標 `pos` に含まれる割当領域を、カメラ空間深度で手前優先で選ぶ。
///
/// `include_floor` / `include_wall` で対象種別を絞る。投影後の多角形内包判定で選び、
/// 奥行きが重なる場合は頂点の平均カメラ深度が大きい（手前の）領域を優先する。
pub(super) fn pick_assignment_region(
    model: &squid_n_core::model::Model,
    projector: &super::Projector,
    pos: egui::Pos2,
    include_floor: bool,
    include_wall: bool,
) -> Option<RegionPick> {
    let mut best: Option<(RegionPick, f32)> = None;
    let mut consider = |pick: RegionPick, coords: &[[f64; 3]]| {
        let poly: Vec<egui::Pos2> = coords.iter().map(|&c| projector.project(c)).collect();
        if poly.len() < 3 || !point_in_polygon(pos, &poly) {
            return;
        }
        let depth = coords
            .iter()
            .map(|&c| projector.cam_space(c)[2])
            .sum::<f32>()
            / coords.len() as f32;
        if best.is_none_or(|(_, best_depth)| depth > best_depth) {
            best = Some((pick, depth));
        }
    };
    if include_floor {
        for region in &model.floor_assignment_regions.regions {
            if let Some(coords) = model.floor_assignment_region_coords(region.id) {
                consider(RegionPick::Floor(region.id), &coords);
            }
        }
    }
    if include_wall {
        for region in &model.wall_assignment_regions.regions {
            if let Some(coords) = model.wall_assignment_region_coords(region.id) {
                consider(RegionPick::Wall(region.id), &coords);
            }
        }
    }
    best.map(|(pick, _)| pick)
}

/// 3D ビューでクリックした親領域（二次部材の配置作業範囲）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum ParentRegionPick {
    Floor(squid_n_core::ids::FloorRegionId),
    Wall(squid_n_core::ids::WallRegionId),
}

/// スクリーン座標 `pos` に含まれる親領域（床領域・壁領域）を手前優先で選ぶ。
pub(super) fn pick_parent_region(
    model: &squid_n_core::model::Model,
    projector: &super::Projector,
    pos: egui::Pos2,
    include_floor: bool,
    include_wall: bool,
) -> Option<ParentRegionPick> {
    let mut best: Option<(ParentRegionPick, f32)> = None;
    let mut consider = |pick: ParentRegionPick, coords: &[[f64; 3]]| {
        let poly: Vec<egui::Pos2> = coords.iter().map(|&c| projector.project(c)).collect();
        if poly.len() < 3 || !point_in_polygon(pos, &poly) {
            return;
        }
        let depth = coords
            .iter()
            .map(|&c| projector.cam_space(c)[2])
            .sum::<f32>()
            / coords.len() as f32;
        if best.is_none_or(|(_, best_depth)| depth > best_depth) {
            best = Some((pick, depth));
        }
    };
    if include_floor {
        for region in &model.floor_regions {
            if let Some(coords) = region.boundary_coords(model) {
                consider(ParentRegionPick::Floor(region.id), &coords);
            }
        }
    }
    if include_wall {
        for region in &model.wall_regions {
            if let Some(coords) = region.boundary_coords(model) {
                consider(ParentRegionPick::Wall(region.id), &coords);
            }
        }
    }
    best.map(|(pick, _)| pick)
}

/// スクリーン座標 `pos` に最も近い支持部材の材軸と、その材軸位置を返す。
///
/// 主架構の 2 節点梁と、`kind` と同じ種別の既存二次部材の材軸を投影し、画面距離が
/// `max_dist_px` 以内で最も近いものを選ぶ。材軸位置は画面線分上の最近点の媒介変数を
/// 用いる（正射投影に近い視点で十分な精度）。
pub(super) fn pick_support_anchor(
    model: &squid_n_core::model::Model,
    projector: &super::Projector,
    pos: egui::Pos2,
    kind: squid_n_core::model::SecondaryMemberKind,
    max_dist_px: f32,
) -> Option<squid_n_core::model::SecondaryMemberAnchor> {
    use squid_n_core::model::{ElementKind, SecondaryMemberAnchor, SecondaryMemberKind};
    let mut best: Option<(f32, SecondaryMemberAnchor)> = None;
    let mut consider = |axis: ([f64; 3], [f64; 3]),
                        support: squid_n_core::model::SupportMemberId| {
        let a = projector.project(axis.0);
        let b = projector.project(axis.1);
        let ab = b - a;
        let len_sq = ab.x * ab.x + ab.y * ab.y;
        let (dist, t) = if len_sq < 1e-6 {
            ((pos - a).length(), 0.0_f64)
        } else {
            let t = (((pos - a).x * ab.x + (pos - a).y * ab.y) / len_sq).clamp(0.0, 1.0);
            ((pos - (a + ab * t)).length(), f64::from(t))
        };
        if dist > max_dist_px {
            return;
        }
        if best.is_none_or(|(best_dist, _)| dist < best_dist) {
            best = Some((
                dist,
                SecondaryMemberAnchor {
                    support,
                    position: t.clamp(0.0, 1.0),
                },
            ));
        }
    };
    for e in &model.elements {
        if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
            continue;
        }
        let (Some(a), Some(b)) = (model.node(e.nodes[0]), model.node(e.nodes[1])) else {
            continue;
        };
        consider(
            (a.coord, b.coord),
            squid_n_core::model::SupportMemberId::Primary(e.id),
        );
    }
    let members = match kind {
        SecondaryMemberKind::Joist => model.joists().collect::<Vec<_>>(),
        SecondaryMemberKind::Post => model.posts().collect::<Vec<_>>(),
    };
    for sm in members {
        let support = squid_n_core::model::SupportMemberId::Secondary(sm.id);
        if let Some(axis) = model.support_member_axis(support) {
            consider(axis, support);
        }
    }
    best.map(|(_, anchor)| anchor)
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

    fn square_floor_model() -> squid_n_core::Model {
        use squid_n_core::ids::{ElemId, NodeId};
        use squid_n_core::model::{
            ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Model, Node,
        };
        let mut model = Model::default();
        for (i, (x, y)) in [(0.0, 0.0), (4000.0, 0.0), (4000.0, 4000.0), (0.0, 4000.0)]
            .into_iter()
            .enumerate()
        {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: [x, y, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            });
        }
        for (i, (a, b)) in [(0u32, 1u32), (1, 2), (2, 3), (3, 0)]
            .into_iter()
            .enumerate()
        {
            model.elements.push(ElementData {
                id: ElemId(i as u32),
                kind: ElementKind::Beam,
                nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
                section: None,
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
        }
        model.rebuild_floor_assignment_regions();
        model
    }

    /// 割当領域の内側をクリックするとその領域が選ばれ、外側では選ばれない。
    #[test]
    fn 割当領域の内側だけをピックする() {
        let model = square_floor_model();
        assert_eq!(model.floor_assignment_regions.regions.len(), 1);
        let cam = crate::viewer::CameraState::default();
        let proj = crate::viewer::Projector::new([0.0, 0.0, 0.0], &cam, 0.001, [0.0, 0.0]);
        let inside = proj.project([2000.0, 2000.0, 0.0]);
        assert!(matches!(
            pick_assignment_region(&model, &proj, inside, true, false),
            Some(RegionPick::Floor(_))
        ));
        let outside = proj.project([-1000.0, 2000.0, 0.0]);
        assert!(pick_assignment_region(&model, &proj, outside, true, false).is_none());
        // 壁を含めない指定では床は選ばれない。
        assert!(pick_assignment_region(&model, &proj, inside, false, true).is_none());
    }

    /// 親領域（床領域）と支持部材アンカーを投影から選ぶ。
    #[test]
    fn 親領域と支持部材アンカーをピックする() {
        use squid_n_core::ids::{ElemId, FloorRegionId, NodeId};
        use squid_n_core::model::{FloorRegion, SecondaryMemberKind, SupportMemberId};

        let mut model = square_floor_model();
        model.floor_regions.push(FloorRegion::new(
            FloorRegionId(0),
            vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        ));
        let cam = crate::viewer::CameraState::default();
        let proj = crate::viewer::Projector::new([0.0, 0.0, 0.0], &cam, 0.001, [0.0, 0.0]);

        let inside = proj.project([2000.0, 2000.0, 0.0]);
        assert!(matches!(
            pick_parent_region(&model, &proj, inside, true, false),
            Some(ParentRegionPick::Floor(_))
        ));
        let outside = proj.project([-1000.0, 2000.0, 0.0]);
        assert!(pick_parent_region(&model, &proj, outside, true, false).is_none());

        let on_axis = proj.project([2000.0, 0.0, 0.0]);
        let anchor = pick_support_anchor(&model, &proj, on_axis, SecondaryMemberKind::Joist, 12.0)
            .expect("大梁の材軸上");
        assert_eq!(anchor.support, SupportMemberId::Primary(ElemId(0)));
        assert!((anchor.position - 0.5).abs() < 0.2, "{anchor:?}");
    }
}
