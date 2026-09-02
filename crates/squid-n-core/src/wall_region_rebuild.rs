//! 壁領域の作り直し（取り込み後・解析前・荷重同期前）。`region_rebuild.rs`（床側）の
//! 壁側版。
//!
//! 柱・梁が囲む鉛直構面内の閉領域（[`crate::region_gen::wall::WallRegionBoundary`]）
//! から壁領域（[`WallRegion`]）を再生成し、既存の壁領域と構面・重心・面積で
//! 対応付けて名前を引き継ぐ。構面判定は候補の全頂点で行い、内包判定には
//! 局所座標の多角形重心を用いる。**壁版（[`WallPlate`]）は畳まない**。柱・梁で
//! 囲まれた壁版（`WallPlateShape::Enclosed`）の帰属（どの壁領域に属すか）を、
//! 壁版の重心が入る壁領域へ付け替えるだけである（`rebuild_floor_regions` と同じ方針）。
//! 取り付く壁版（パラペット・腰壁・垂れ壁・自立壁）はどの壁領域からも参照されない
//! 独立した壁版のまま、素通しする。
//!
//! **床側との違い**: 床は「同じレベル Z」で新旧の床領域を対応付けるが、壁は構面
//! （直線）ごとに閉領域が現れるため、候補の全頂点が「同じ構面上にあるか」
//! （[`WallRegionBoundary::is_same_plane`]）で絞り込んでから、構面内の局所座標 `(s, z)`
//! で多角形重心・内包を判定する。間柱は D7 により毎回入れ直す。
//!
//! **床側 D20 に相当する自動変換**: どの壁領域にも収まらない、柱・梁で囲まれた
//! 壁版（`WallPlateShape::Enclosed`）のうち、**水平な辺（上辺・下辺）がちょうど 1 つ**
//! 同一レベルの大梁に全長覆われていれば、その辺を取付き線とする取り付く壁版
//! （`Attached`／`RegionAnchor::Line`）へ変換する（パラペット・腰壁・垂れ壁）。
//! 判定対象は水平な辺のみで、鉛直な辺（柱沿い）は対象外とする（D13/D14 に
//! 「柱だけに取り付く壁」の用例がないため）。張り出し量 `extent` は取付き辺からの
//! 鉛直方向の高さ差で、符号つき（下方向が負）である。`RegionAnchor::FloorRegion`
//! （自立壁）は自動検出しない。ST-Bridge から自立壁を判別できる情報源がなく、
//! 手動作成専用とする（利用者との dig で確定。
//! `dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` §9）。
//!
//! 変換で境界が取付き線 2 点だけに縮んだぶん、参照が 0 になった節点は削除する
//! （D21。床側と同じ規約で、このリビルドが縮めた境界の先端節点だけを対象とし、
//! 他の理由で未使用の既存節点は対象外とする）。
//! 自由端は取付き辺上にない頂点が 1〜2 点のときに限り変換する。同じ高さでも
//! 辺から外れた頂点は自由端に数える。変換後の取り付く壁版は解析要素にせず、
//! 自重の自動分配（取付き線への分布）も行わない。

use crate::geom::LEVEL_TOL_MM;
use crate::ids::{NodeId, WallRegionId};
use crate::model::{LoadTransfer, Model, RegionAnchor, WallPlateShape, WallRegion};
use crate::region_gen::wall::{scan_wall_region_boundaries, WallRegionBoundary};
use crate::region_gen::BOUNDARY_TOL_MM;
use crate::region_rebuild::{
    delete_unref_nodes, edge_fully_covered, horizontal_girders, point_segment_dist, GirderSeg,
};

/// 重心照合で面積が近いとみなす相対許容（新旧の床領域の面積比）。
/// [`crate::region_rebuild::CENTROID_MATCH_AREA_REL`] と同じ値・同じ意味。
pub const CENTROID_MATCH_AREA_REL: f64 = 1e-3;

/// [`rebuild_wall_regions`] の件数報告。
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct WallRegionRebuildReport {
    /// 検出した壁領域（柱・梁が囲む鉛直構面内の閉領域）の数。
    pub regions: usize,
    /// 旧壁領域から名前を引き継いだ数。
    pub inherited: usize,
    /// 対応する旧壁領域が見つからなかった新規の床領域の数。
    pub new_regions: usize,
    /// 重心が新しい床領域に入らなかった旧壁領域（名前を引き継げなかった）の数。
    pub unmatched_old_regions: usize,
    /// 壁領域へ帰属し直した壁版の数。
    pub wall_plates_assigned: usize,
    /// どの壁領域にも収まらず、取り付く壁版（`Attached`／`Line`）へ変換した数。
    pub wall_plates_converted_to_attached: usize,
    /// どの壁領域にも収まらず、変換もできなかった壁版の数（警告対象。削除しない）。
    pub unassigned_wall_plates: usize,
    /// 中点がちょうど 1 つの壁領域に厳密内包されなかった間柱（Post）の本数。
    pub unassigned_posts: usize,
    /// 取り付く壁版への変換で境界が縮み、参照が 0 になり削除した節点の数。
    pub deleted_nodes: usize,
}

/// 現状の壁領域で、中点がちょうど 1 領域に厳密内包されない間柱の本数。
pub fn unassigned_post_count(model: &Model) -> usize {
    model.unassigned_posts.len()
}

/// 壁領域を柱・梁が囲む鉛直構面内の閉領域から作り直し、名前を引き継ぎ、
/// 壁版の帰属を付け替える。
///
/// 壁版そのもの（`model.wall_plates`）は畳まない。どの壁領域にも収まらない
/// 柱・梁で囲まれた壁版は、水平な辺がちょうど 1 つ大梁に全長覆われ自由端が
/// 1〜2 点なら取り付く壁版へ変換し、それもできなければ帰属なしのまま残す
/// （警告。落とさない。モジュール doc 参照）。
pub fn rebuild_wall_regions(model: &mut Model) -> WallRegionRebuildReport {
    let scan = scan_wall_region_boundaries(model);
    // 壁領域を作り直す前に、領域内間柱を未割当へ集約する（assign_posts が再配分する）。
    for r in &mut model.wall_regions {
        model.unassigned_posts.append(&mut r.posts);
    }
    let old_regions = std::mem::take(&mut model.wall_regions);
    let mut report = WallRegionRebuildReport::default();

    // 1. 新しい壁領域を床領域ごとに作り、旧壁領域と構面・重心・面積で対応付けて
    //    名前を引き継ぐ（D10 と同じ方針）。間柱は後段で D7 により入れ直す。
    let mut matched_old = vec![false; old_regions.len()];
    let mut new_regions: Vec<WallRegion> = Vec::with_capacity(scan.boundaries.len());
    for rb in &scan.boundaries {
        let mut region = WallRegion::new(WallRegionId(0), rb.boundary.clone());
        let rb_area = rb.area(model);
        if let Some((oi, old_area)) = old_regions
            .iter()
            .enumerate()
            .filter(|(oi, _)| !matched_old[*oi])
            .filter_map(|(oi, old)| match_candidate(model, rb, &old.boundary, oi))
            .min_by(|(_, a), (_, b)| a.total_cmp(b))
        {
            matched_old[oi] = true;
            let denom = rb_area.abs().max(f64::EPSILON);
            if (old_area - rb_area).abs() / denom < CENTROID_MATCH_AREA_REL {
                region.name = old_regions[oi].name.clone();
                report.inherited += 1;
            } else {
                report.new_regions += 1;
            }
        } else {
            report.new_regions += 1;
        }
        new_regions.push(region);
    }
    report.regions = new_regions.len();
    report.unmatched_old_regions = matched_old.iter().filter(|m| !**m).count();

    // 2. 壁版の帰属を、重心が入る壁領域へ付け替える。収まらない壁版は、水平な辺が
    //    1 つだけ大梁に全長覆われていれば取り付く壁版へ変換し（床側 D20 に相当。
    //    モジュール doc 参照）、それもできなければ帰属なしのまま残す。
    let beams = horizontal_girders(model);
    let mut owner: Vec<Option<usize>> = vec![None; model.wall_plates.len()];
    for (pi, plate) in model.wall_plates.iter().enumerate() {
        let WallPlateShape::Enclosed { boundary } = &plate.shape else {
            continue; // 取り付く壁版は素通し（どの壁領域にも属さない）。
        };
        if let Some((ri, _)) = scan
            .boundaries
            .iter()
            .enumerate()
            .filter_map(|(ri, rb)| match_candidate(model, rb, boundary, ri))
            .min_by(|(_, a), (_, b)| a.total_cmp(b))
        {
            owner[pi] = Some(ri);
            report.wall_plates_assigned += 1;
        }
    }
    let mut converted = Vec::new();
    // 変換すると、旧境界のうち取付き線 2 点以外の節点が参照 0 になりうる
    // （D21・床側と同じ理由）。それ以外の既存節点は削除対象にしない。
    let mut discarded_by_conversion: Vec<NodeId> = Vec::new();
    for (pi, plate) in model.wall_plates.iter().enumerate() {
        if owner[pi].is_some() {
            continue;
        }
        let WallPlateShape::Enclosed { boundary } = &plate.shape else {
            continue;
        };
        if let Some(attached) = try_convert_wall_attached(model, boundary, &beams) {
            discarded_by_conversion.extend(boundary.iter().copied());
            converted.push((pi, attached));
            report.wall_plates_converted_to_attached += 1;
        } else {
            report.unassigned_wall_plates += 1;
        }
    }
    for (pi, shape) in converted {
        model.wall_plates[pi].shape = shape;
    }
    for (pi, ri) in owner.into_iter().enumerate() {
        if let Some(ri) = ri {
            new_regions[ri]
                .wall_plate_ids
                .push(model.wall_plates[pi].id);
        }
    }

    for (i, r) in new_regions.iter_mut().enumerate() {
        r.id = WallRegionId(i as u32);
    }
    model.wall_regions = new_regions;
    report.unassigned_posts = assign_posts(model, &scan.boundaries);
    report.deleted_nodes = delete_unref_nodes(model, &discarded_by_conversion);

    report
}

/// 柱・梁で囲まれた壁版の境界のうち、**水平な辺（両端の Z が一致する辺）**が
/// ちょうど 1 つだけ同一レベルの大梁に全長覆われていれば、その辺を取付き線とする
/// 取り付く壁版の形へ変換する（床側 D20 に相当。モジュール doc 参照）。
/// 鉛直な辺（柱沿い）は判定対象にしない。変換できなければ `None`。
fn try_convert_wall_attached(
    model: &Model,
    boundary: &[NodeId],
    beams: &[GirderSeg],
) -> Option<WallPlateShape> {
    if boundary.len() < 3 {
        return None;
    }
    let pts: Vec<[f64; 3]> = boundary
        .iter()
        .map(|id| model.nodes.get(id.index()).map(|n| n.coord))
        .collect::<Option<_>>()?;
    let n = boundary.len();
    let mut covered = Vec::new();
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        if (a[2] - b[2]).abs() > LEVEL_TOL_MM {
            continue; // 鉛直な辺（柱沿い）は対象外。
        }
        let z = (a[2] + b[2]) / 2.0;
        if edge_fully_covered([a[0], a[1]], [b[0], b[1]], z, beams) {
            covered.push(i);
        }
    }
    if covered.len() != 1 {
        return None;
    }
    let ei = covered[0];
    let n0 = boundary[ei];
    let n1 = boundary[(ei + 1) % n];
    let a = pts[ei];
    let b = pts[(ei + 1) % n];
    let edge_z = (a[2] + b[2]) / 2.0;
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let len = (dx * dx + dy * dy).sqrt();
    if len <= f64::EPSILON {
        return None;
    }
    let ux = dx / len;
    let uy = dy / len;

    // 取付き辺上の点（分割点）だけを自由端から除く。同じ高さでも辺から外れた
    // 頂点は自由端に数える（床側 D20 の `point_segment_dist` と同じ。高さだけで
    // 除外すると、梁レベルに折れ曲がった頂点を持つ輪郭を誤って変換する）。
    let mut free = Vec::new();
    for p in &pts {
        let on_edge = (p[2] - edge_z).abs() <= LEVEL_TOL_MM
            && point_segment_dist([p[0], p[1]], [a[0], a[1]], [b[0], b[1]]) <= BOUNDARY_TOL_MM;
        if on_edge {
            continue;
        }
        let t = (p[0] - a[0]) * ux + (p[1] - a[1]) * uy;
        free.push((t, p[2] - edge_z));
    }
    let extent = match free.len() {
        1 => [free[0].1, free[0].1],
        2 => {
            if free[0].0 <= free[1].0 {
                [free[0].1, free[1].1]
            } else {
                [free[1].1, free[0].1]
            }
        }
        _ => return None,
    };

    Some(WallPlateShape::Attached {
        anchor: RegionAnchor::Line {
            nodes: [n0, n1],
            span: [0.0, 1.0],
            transfer: LoadTransfer::Anchor,
        },
        extent,
    })
}

/// `candidate`（旧壁領域または壁版の境界節点列）が構面 `rb` に属すか判定する。
/// 属すなら `(候補を識別する index, candidate の実面積 [mm²])` を返す。
///
/// 判定は 2 段階: (1) `candidate` の全頂点が `rb` と同じ構面上にあること
/// （[`WallRegionBoundary::is_same_plane`]）、(2) `candidate` の多角形重心
/// （`rb` の局所座標 `(s, z)` へ射影したもの）が `rb` の内部にあること
/// （[`WallRegionBoundary::contains`]）。面積は実座標（3 次元）から求める（§3.2 E3。
/// 局所座標への射影はトポロジー判定にのみ使う近似であり、面積には使わない）。
fn match_candidate(
    model: &Model,
    rb: &WallRegionBoundary,
    candidate: &[NodeId],
    index: usize,
) -> Option<(usize, f64)> {
    let coords: Vec<[f64; 3]> = candidate
        .iter()
        .map(|n| model.nodes.get(n.index()).map(|nd| nd.coord))
        .collect::<Option<_>>()?;
    if coords.len() < 3 {
        return None;
    }
    if !coords.iter().all(|c| rb.is_same_plane([c[0], c[1]])) {
        return None;
    }
    let projected: Vec<[f64; 2]> = coords
        .iter()
        .map(|c| project_onto(rb.plane_origin, rb.plane_direction, *c))
        .collect();
    let local_centroid = shoelace_centroid(&projected, shoelace_area(&projected));
    if !rb.contains(model, local_centroid) {
        return None;
    }
    let area = crate::geom::polygon_area_3d(&coords);
    Some((index, area))
}

/// 実座標を構面 `(origin, direction)` の局所座標 `(s, z)` へ射影する
/// （[`crate::region_gen::wall`] 内部の `project` と同じ計算だが `pub(crate)` では
/// ないため同じ式をここに持つ）。
fn project_onto(origin: [f64; 2], direction: [f64; 2], coord: [f64; 3]) -> [f64; 2] {
    let v = [coord[0] - origin[0], coord[1] - origin[1]];
    let s = v[0] * direction[0] + v[1] * direction[1];
    [s, coord[2]]
}

fn shoelace_area(pts: &[[f64; 2]]) -> f64 {
    if pts.len() < 3 {
        return 0.0;
    }
    let sum: f64 = pts
        .iter()
        .enumerate()
        .map(|(i, a)| {
            let b = pts[(i + 1) % pts.len()];
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    sum / 2.0
}

fn shoelace_centroid(pts: &[[f64; 2]], signed_area: f64) -> [f64; 2] {
    if signed_area.abs() <= f64::EPSILON {
        let n = pts.len() as f64;
        return [
            pts.iter().map(|p| p[0]).sum::<f64>() / n,
            pts.iter().map(|p| p[1]).sum::<f64>() / n,
        ];
    }
    let mut cx = 0.0;
    let mut cy = 0.0;
    for (i, a) in pts.iter().enumerate() {
        let b = pts[(i + 1) % pts.len()];
        let cross = a[0] * b[1] - b[0] * a[1];
        cx += (a[0] + b[0]) * cross;
        cy += (a[1] + b[1]) * cross;
    }
    let six_a = 6.0 * signed_area;
    [cx / six_a, cy / six_a]
}

fn assign_posts(model: &mut Model, boundaries: &[WallRegionBoundary]) -> usize {
    let posts: Vec<_> = model
        .wall_regions
        .iter_mut()
        .flat_map(|r| r.posts.drain(..))
        .chain(model.unassigned_posts.drain(..))
        .collect();
    for region in &mut model.wall_regions {
        region.posts.clear();
    }
    let mut unassigned = 0;
    for sm in posts {
        let nodes = sm.nodes;
        let Some(a) = model.nodes.get(nodes[0].index()).map(|n| n.coord) else {
            model.unassigned_posts.push(sm);
            unassigned += 1;
            continue;
        };
        let Some(b) = model.nodes.get(nodes[1].index()).map(|n| n.coord) else {
            model.unassigned_posts.push(sm);
            unassigned += 1;
            continue;
        };
        let midpoint = [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
        let hits: Vec<usize> = boundaries
            .iter()
            .enumerate()
            .filter(|(_, rb)| rb.is_same_plane(midpoint))
            .filter(|(_, rb)| {
                let coord = [midpoint[0], midpoint[1], (a[2] + b[2]) * 0.5];
                rb.contains(
                    model,
                    project_onto(rb.plane_origin, rb.plane_direction, coord),
                )
            })
            .map(|(i, _)| i)
            .collect();
        if hits.len() == 1 {
            model.wall_regions[hits[0]].posts.push(sm);
        } else {
            model.unassigned_posts.push(sm);
            unassigned += 1;
        }
    }
    unassigned
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{ElemId, WallPlateId};
    use crate::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node, WallPlate,
    };

    fn node(id: u32, x: f64, y: f64, z: f64) -> Node {
        Node {
            id: NodeId(id),
            coord: [x, y, z],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    fn beam(id: u32, i: u32, j: u32) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    /// 柱 2 本・梁 2 本（頂部・柱脚）で閉じた矩形（4m×3m、Y=0 面）を持つ最小モデル。
    fn one_bay_wall_model() -> Model {
        let mut model = Model::default();
        model.nodes.push(node(0, 0.0, 0.0, 0.0));
        model.nodes.push(node(1, 4000.0, 0.0, 0.0));
        model.nodes.push(node(2, 4000.0, 0.0, 3000.0));
        model.nodes.push(node(3, 0.0, 0.0, 3000.0));
        model.elements.push(beam(0, 0, 3)); // 柱（左）
        model.elements.push(beam(1, 1, 2)); // 柱（右）
        model.elements.push(beam(2, 3, 2)); // 頂部梁
        model.elements.push(beam(3, 0, 1)); // 柱脚間の梁
        model
    }

    #[test]
    fn test_rebuild_creates_wall_region_from_boundary() {
        let mut model = one_bay_wall_model();
        let report = rebuild_wall_regions(&mut model);
        assert_eq!(report.regions, 1);
        assert_eq!(report.new_regions, 1);
        assert_eq!(report.inherited, 0);
        assert_eq!(model.wall_regions.len(), 1);
        assert_eq!(model.wall_regions[0].id, WallRegionId(0));
        assert_eq!(model.wall_regions[0].boundary.len(), 4);
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    /// 再構成しても、同じ構面・同じ重心・同じ面積の壁領域には旧壁領域の名前を
    /// 引き継ぐ（D10）。間柱は再走査で帰属し、旧領域からは引き継がない（D7）。
    #[test]
    fn test_rebuild_inherits_name_without_post_ids() {
        let mut model = one_bay_wall_model();
        rebuild_wall_regions(&mut model);
        model.wall_regions[0].name = "西面耐震壁".into();
        model.unassigned_posts.push(crate::model::SecondaryMember {
            kind: crate::model::SecondaryMemberKind::Post,
            nodes: [NodeId(0), NodeId(3)],
            section: None,
            name: "P1".into(),
        });

        // 名前・間柱を付けたあとで、モデル自体は変えずに再構成する。
        let report = rebuild_wall_regions(&mut model);
        assert_eq!(report.inherited, 1, "同じ床領域は引き継がれるはず");
        assert_eq!(model.wall_regions[0].name, "西面耐震壁");
        assert!(model.wall_regions[0].posts.is_empty());
        assert!(report.unassigned_posts >= 1);
    }

    #[test]
    fn test_match_candidate_requires_all_vertices_on_same_plane() {
        let mut model = one_bay_wall_model();
        model.nodes.push(node(4, 4000.0, 3000.0, 3000.0));
        let rb = scan_wall_region_boundaries(&model).boundaries[0].clone();

        let candidate = vec![NodeId(0), NodeId(1), NodeId(4), NodeId(3)];

        assert!(match_candidate(&model, &rb, &candidate, 0).is_none());
    }

    #[test]
    fn test_match_candidate_uses_shoelace_centroid_for_l_shape() {
        let mut model = Model::default();
        for (id, (s, z)) in [
            (0, (0.0, 0.0)),
            (1, (4000.0, 0.0)),
            (2, (4000.0, 1500.0)),
            (3, (2000.0, 1500.0)),
            (4, (2000.0, 4000.0)),
            (5, (0.0, 4000.0)),
        ] {
            model.nodes.push(node(id, s, 0.0, z));
        }
        let candidate = vec![
            NodeId(0),
            NodeId(1),
            NodeId(2),
            NodeId(3),
            NodeId(4),
            NodeId(5),
        ];
        let rb = WallRegionBoundary {
            plane_origin: [0.0, 0.0],
            plane_direction: [1.0, 0.0],
            boundary: candidate.clone(),
            edges: (0..6).map(ElemId).collect(),
        };

        assert!(!rb.contains(&model, [2000.0, 1833.3333333333]));
        assert!(match_candidate(&model, &rb, &candidate, 0).is_some());
    }

    #[test]
    fn test_rebuild_assigns_internal_post_only() {
        let mut model = one_bay_wall_model();
        model.nodes.push(node(4, 2000.0, 0.0, 0.0));
        model.nodes.push(node(5, 2000.0, 0.0, 3000.0));
        model.unassigned_posts.push(crate::model::SecondaryMember {
            kind: crate::model::SecondaryMemberKind::Post,
            nodes: [NodeId(4), NodeId(5)],
            section: None,
            name: "内部間柱".into(),
        });

        let report = rebuild_wall_regions(&mut model);

        assert_eq!(report.unassigned_posts, 0);
        assert_eq!(model.wall_regions.len(), 1);
        assert_eq!(model.wall_regions[0].posts.len(), 1);
        assert_eq!(model.wall_regions[0].posts[0].nodes, [NodeId(4), NodeId(5)]);
    }

    #[test]
    fn test_rebuild_assigns_enclosed_wall_plate_to_matching_region() {
        let mut model = one_bay_wall_model();
        model.wall_plates.push(WallPlate {
            id: WallPlateId(0),
            shape: crate::model::WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: vec![],
            slit: Default::default(),
        });
        let report = rebuild_wall_regions(&mut model);
        assert_eq!(report.wall_plates_assigned, 1);
        assert_eq!(report.unassigned_wall_plates, 0);
        assert_eq!(model.wall_regions[0].wall_plate_ids, vec![WallPlateId(0)]);
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    /// 壁版の境界が実在の構面（柱・梁の閉路）と対応せず、かつどの辺も大梁に
    /// 全長覆われていない（＝取り付く壁版へも変換できない）場合、どの壁領域にも
    /// 収まらず「帰属なし」のまま残る（削除しない）。
    #[test]
    fn test_rebuild_reports_unassigned_wall_plate_off_any_plane() {
        let mut model = one_bay_wall_model();
        // Y=3000（実在しない構面）に浮いた壁版。
        model.nodes.push(node(4, 0.0, 3000.0, 0.0));
        model.nodes.push(node(5, 4000.0, 3000.0, 0.0));
        model.nodes.push(node(6, 4000.0, 3000.0, 3000.0));
        model.nodes.push(node(7, 0.0, 3000.0, 3000.0));
        model.wall_plates.push(WallPlate {
            id: WallPlateId(0),
            shape: crate::model::WallPlateShape::Enclosed {
                boundary: vec![NodeId(4), NodeId(5), NodeId(6), NodeId(7)],
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: vec![],
            slit: Default::default(),
        });
        let report = rebuild_wall_regions(&mut model);
        assert_eq!(report.wall_plates_assigned, 0);
        assert_eq!(report.unassigned_wall_plates, 1);
        assert!(model.wall_regions[0].wall_plate_ids.is_empty());
        // 壁版そのものは畳まれず残る。
        assert_eq!(model.wall_plates.len(), 1);
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    /// 頂部梁（Z=3000）の上に立つパラペット（Z=3000〜4500）は、どの壁領域にも
    /// 収まらず、下辺（頂部梁と重なる水平な辺）を取付き線とする取り付く壁版へ
    /// 自動変換される（床側 D20 に相当）。
    #[test]
    fn test_parapet_on_top_beam_converts_to_attached_line() {
        let mut model = one_bay_wall_model();
        model.nodes.push(node(4, 0.0, 0.0, 4500.0));
        model.nodes.push(node(5, 4000.0, 0.0, 4500.0));
        model.wall_plates.push(WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                // 下辺 3→2 は頂部梁（beam(2, 3, 2)）と同じ節点対・向き。
                boundary: vec![NodeId(3), NodeId(2), NodeId(5), NodeId(4)],
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: vec![],
            slit: Default::default(),
        });
        let report = rebuild_wall_regions(&mut model);
        assert_eq!(report.wall_plates_assigned, 0);
        assert_eq!(report.wall_plates_converted_to_attached, 1);
        assert_eq!(report.unassigned_wall_plates, 0);

        let plate = &model.wall_plates[0];
        assert!(plate.is_attached());
        match &plate.shape {
            WallPlateShape::Attached {
                anchor:
                    RegionAnchor::Line {
                        nodes,
                        span,
                        transfer,
                    },
                extent,
            } => {
                assert_eq!(*span, [0.0, 1.0]);
                assert_eq!(*transfer, LoadTransfer::Anchor);
                let a = model.nodes[nodes[0].index()].coord;
                let b = model.nodes[nodes[1].index()].coord;
                assert!((a[2] - 3000.0).abs() < 1e-6, "取付き線は頂部梁の高さ");
                assert!((b[2] - 3000.0).abs() < 1e-6);
                assert!((extent[0] - 1500.0).abs() < 1e-6, "上向き {extent:?}");
                assert!((extent[1] - 1500.0).abs() < 1e-6, "{extent:?}");
            }
            other => panic!("Line の Attached ではない: {other:?}"),
        }
        // パラペット先端（Z=4500）の節点は参照 0 になり削除される（D21）。
        assert!(
            !model
                .nodes
                .iter()
                .any(|n| (n.coord[2] - 4500.0).abs() < 1e-6),
            "先端節点は削除されるはず"
        );
        assert!(report.deleted_nodes >= 2);
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    /// T 字取り付き（5 節点以上）の壁版は、水平な辺が複数覆われていても
    /// 自由端が 2 点を超えるため自動変換されず、帰属なしのまま残る。
    #[test]
    fn test_wall_plate_with_more_than_two_free_vertices_is_not_converted() {
        let mut model = one_bay_wall_model();
        model.nodes.push(node(4, 0.0, 0.0, 4500.0));
        model.nodes.push(node(5, 1500.0, 0.0, 5200.0));
        model.nodes.push(node(6, 4000.0, 0.0, 4500.0));
        model.wall_plates.push(WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(3), NodeId(2), NodeId(6), NodeId(5), NodeId(4)],
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: vec![],
            slit: Default::default(),
        });
        let report = rebuild_wall_regions(&mut model);
        assert_eq!(report.wall_plates_converted_to_attached, 0);
        assert_eq!(report.unassigned_wall_plates, 1);
        assert!(matches!(
            model.wall_plates[0].shape,
            WallPlateShape::Enclosed { .. }
        ));
    }

    /// 敵対的レビューで発覚した回帰: パラペット変換で境界が縮み、D21 により
    /// 低インデックスの節点（パラペットの自由端）が削除・圧縮されると、
    /// 既存の（別の）壁領域の境界がその圧縮に追随せず、ダングリング参照へ
    /// 壊れることがあった（`Model::visit_node_ids` が `wall_regions` を
    /// 走査していなかったため）。パラペットの自由端より高いインデックスの節点を
    /// 持つ壁領域を用意し、変換後も壁領域の境界座標が元の4隅と一致することを固定する。
    #[test]
    fn test_wall_region_boundary_survives_node_compaction_from_attached_conversion() {
        let mut model = Model::default();
        // 低インデックス: パラペットの自由端（変換後に参照 0 になり削除される節点）。
        model.nodes.push(node(0, 0.0, 0.0, 4500.0));
        model.nodes.push(node(1, 4000.0, 0.0, 4500.0));
        // 高インデックス: 実在の柱・梁の区画の4隅。
        model.nodes.push(node(2, 0.0, 0.0, 0.0));
        model.nodes.push(node(3, 4000.0, 0.0, 0.0));
        model.nodes.push(node(4, 4000.0, 0.0, 3000.0));
        model.nodes.push(node(5, 0.0, 0.0, 3000.0));
        model.elements.push(beam(0, 2, 5)); // 柱（左）
        model.elements.push(beam(1, 3, 4)); // 柱（右）
        model.elements.push(beam(2, 5, 4)); // 頂部梁
        model.elements.push(beam(3, 2, 3)); // 柱脚間の梁

        // パラペット: 下辺 5->4 が頂部梁と一致、自由端は 1,0（Z=4500）。
        model.wall_plates.push(WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(5), NodeId(4), NodeId(1), NodeId(0)],
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: vec![],
            slit: Default::default(),
        });

        let before: Vec<[f64; 3]> = vec![
            model.nodes[2].coord,
            model.nodes[3].coord,
            model.nodes[4].coord,
            model.nodes[5].coord,
        ];

        let report = rebuild_wall_regions(&mut model);
        assert_eq!(report.wall_plates_converted_to_attached, 1);
        assert!(report.deleted_nodes >= 1, "自由端の節点が削除されるはず");

        assert_eq!(model.wall_regions.len(), 1);
        let after: Vec<[f64; 3]> = model.wall_regions[0]
            .boundary
            .iter()
            .map(|n| model.nodes[n.index()].coord)
            .collect();
        for c in &before {
            assert!(
                after
                    .iter()
                    .any(|a| (0..3).all(|k| (a[k] - c[k]).abs() < 1e-6)),
                "壁領域の境界座標が節点圧縮の前後で保たれていない（{c:?} が {after:?} にない）"
            );
        }
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    /// 柱脚梁の下へ垂れる壁版は、上辺が大梁に載り、張り出し量が負の取り付く壁版へ
    /// 変換される（垂れ壁）。
    #[test]
    fn test_hanging_wall_below_bottom_beam_converts_with_negative_extent() {
        let mut model = one_bay_wall_model();
        model.nodes.push(node(4, 0.0, 0.0, -1200.0));
        model.nodes.push(node(5, 4000.0, 0.0, -1200.0));
        model.wall_plates.push(WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(5), NodeId(4)],
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: vec![],
            slit: Default::default(),
        });
        let report = rebuild_wall_regions(&mut model);
        assert_eq!(report.wall_plates_converted_to_attached, 1);
        match &model.wall_plates[0].shape {
            WallPlateShape::Attached { extent, .. } => {
                assert!((extent[0] + 1200.0).abs() < 1e-6, "{extent:?}");
                assert!((extent[1] + 1200.0).abs() < 1e-6, "{extent:?}");
            }
            other => panic!("{other:?}"),
        }
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    /// 梁レベルに折れ曲がった頂点（取付き辺上ではない）を持つ輪郭は、自由端が
    /// 3 点になるため変換しない。高さだけで自由端を除外すると誤変換していた。
    #[test]
    fn test_jog_at_beam_level_is_not_converted() {
        let mut model = one_bay_wall_model();
        model.nodes.push(node(4, 0.0, 0.0, 4500.0));
        model.nodes.push(node(5, 4000.0, 0.0, 4500.0));
        // 頂部梁と同じ高さだが、構面から Y 方向へ折れた頂点。
        model.nodes.push(node(6, 2000.0, 800.0, 3000.0));
        model.wall_plates.push(WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(3), NodeId(2), NodeId(6), NodeId(5), NodeId(4)],
            },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: vec![],
            slit: Default::default(),
        });
        let report = rebuild_wall_regions(&mut model);
        assert_eq!(report.wall_plates_converted_to_attached, 0);
        assert_eq!(report.unassigned_wall_plates, 1);
        assert!(matches!(
            model.wall_plates[0].shape,
            WallPlateShape::Enclosed { .. }
        ));
    }
}
