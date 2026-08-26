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
//! **床側との違い**: 床は「同じレベル Z」で新旧区画を対応付けるが、壁は構面
//! （直線）ごとに閉領域が現れるため、候補の全頂点が「同じ構面上にあるか」
//! （[`WallRegionBoundary::is_same_plane`]）で絞り込んでから、構面内の局所座標 `(s, z)`
//! で多角形重心・内包を判定する。間柱は D7 により毎回入れ直す。
//!
//! **未対応**: 床側 D20（パネルに収まらない版を取り付き版へ自動変換する）に相当する
//! 規則は壁側でまだ決めていない。囲まれた壁版でどの壁領域にも収まらないものは、
//! 変換せず「帰属なし」として件数のみ報告する（残課題。
//! `dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` §9）。

use crate::ids::{NodeId, WallRegionId};
use crate::model::{Model, WallPlateShape, WallRegion};
use crate::region_gen::wall::{scan_wall_region_boundaries, WallRegionBoundary};

/// 重心照合で面積が近いとみなす相対許容（新旧区画の面積比）。
/// [`crate::region_rebuild::CENTROID_MATCH_AREA_REL`] と同じ値・同じ意味。
pub const CENTROID_MATCH_AREA_REL: f64 = 1e-3;

/// [`rebuild_wall_regions`] の件数報告。
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct WallRegionRebuildReport {
    /// 検出した壁領域（柱・梁が囲む鉛直構面内の閉領域）の数。
    pub regions: usize,
    /// 旧壁領域から名前を引き継いだ数。
    pub inherited: usize,
    /// 対応する旧壁領域が見つからなかった新規区画の数。
    pub new_regions: usize,
    /// 重心が新しい区画に入らなかった旧壁領域（名前を引き継げなかった）の数。
    pub unmatched_old_regions: usize,
    /// 壁領域へ帰属し直した壁版の数。
    pub wall_plates_assigned: usize,
    /// どの壁領域にも収まらなかった、柱・梁で囲まれた壁版の数（警告対象。削除しない）。
    pub unassigned_wall_plates: usize,
    /// 中点がちょうど 1 つの壁領域に厳密内包されなかった間柱（Post）の本数。
    pub unassigned_posts: usize,
}

/// 壁領域を柱・梁が囲む鉛直構面内の閉領域から作り直し、名前を引き継ぎ、
/// 壁版の帰属を付け替える。
///
/// 壁版そのもの（`model.wall_plates`）は畳まない。どの壁領域にも収まらない
/// 柱・梁で囲まれた壁版は、帰属なしのまま残す（警告。落とさない。モジュール doc 参照）。
pub fn rebuild_wall_regions(model: &mut Model) -> WallRegionRebuildReport {
    let scan = scan_wall_region_boundaries(model);
    let old_regions = std::mem::take(&mut model.wall_regions);
    let mut report = WallRegionRebuildReport::default();

    // 1. 新しい壁領域を区画ごとに作り、旧壁領域と構面・重心・面積で対応付けて
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

    // 2. 壁版の帰属を、重心が入る壁領域へ付け替える。収まらない壁版は帰属なしのまま残す
    //    （床側 D20 に相当する自動変換は未対応。モジュール doc 参照）。
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
        } else {
            report.unassigned_wall_plates += 1;
        }
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

    report
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
    let mut posts: Vec<_> = model
        .secondary_members
        .iter()
        .filter(|sm| sm.kind == crate::model::SecondaryMemberKind::Post)
        .map(|sm| (sm.id, sm.nodes))
        .collect();
    posts.sort_by_key(|(id, _)| *id);
    for region in &mut model.wall_regions {
        region.post_ids.clear();
    }
    let mut unassigned = 0;
    for (id, nodes) in posts {
        let Some(a) = model.nodes.get(nodes[0].index()).map(|n| n.coord) else {
            unassigned += 1;
            continue;
        };
        let Some(b) = model.nodes.get(nodes[1].index()).map(|n| n.coord) else {
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
            model.wall_regions[hits[0]].post_ids.push(id);
        } else {
            unassigned += 1;
        }
    }
    unassigned
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{ElemId, SecondaryMemberId, WallPlateId};
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

    /// 再構成しても、同じ構面・同じ重心・同じ面積の区画には旧壁領域の名前を
    /// 引き継ぐ（D10）。間柱は再走査で帰属し、旧領域からは引き継がない（D7）。
    #[test]
    fn test_rebuild_inherits_name_without_post_ids() {
        let mut model = one_bay_wall_model();
        rebuild_wall_regions(&mut model);
        model.wall_regions[0].name = "西面耐震壁".into();
        model.secondary_members.push(crate::model::SecondaryMember {
            id: SecondaryMemberId(0),
            kind: crate::model::SecondaryMemberKind::Post,
            nodes: [NodeId(0), NodeId(3)],
            section: None,
            name: "P1".into(),
        });
        model.wall_regions[0].post_ids = vec![SecondaryMemberId(0)];

        // 名前・間柱を付けたあとで、モデル自体は変えずに再構成する。
        let report = rebuild_wall_regions(&mut model);
        assert_eq!(report.inherited, 1, "同じ区画は引き継がれるはず");
        assert_eq!(model.wall_regions[0].name, "西面耐震壁");
        assert!(model.wall_regions[0].post_ids.is_empty());
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
        model.secondary_members.push(crate::model::SecondaryMember {
            id: SecondaryMemberId(0),
            kind: crate::model::SecondaryMemberKind::Post,
            nodes: [NodeId(4), NodeId(5)],
            section: None,
            name: "内部間柱".into(),
        });

        let report = rebuild_wall_regions(&mut model);

        assert_eq!(report.unassigned_posts, 0);
        assert_eq!(model.wall_regions.len(), 1);
        assert_eq!(model.wall_regions[0].post_ids, vec![SecondaryMemberId(0)]);
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
            three_side_slit: false,
        });
        let report = rebuild_wall_regions(&mut model);
        assert_eq!(report.wall_plates_assigned, 1);
        assert_eq!(report.unassigned_wall_plates, 0);
        assert_eq!(model.wall_regions[0].wall_plate_ids, vec![WallPlateId(0)]);
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    /// 壁版の境界が実在の構面（柱・梁の閉路）と対応しない場合、どの壁領域にも
    /// 収まらず「帰属なし」のまま残る（削除しない。床側 D20 に相当する自動変換は未対応）。
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
            three_side_slit: false,
        });
        let report = rebuild_wall_regions(&mut model);
        assert_eq!(report.wall_plates_assigned, 0);
        assert_eq!(report.unassigned_wall_plates, 1);
        assert!(model.wall_regions[0].wall_plate_ids.is_empty());
        // 壁版そのものは畳まれず残る。
        assert_eq!(model.wall_plates.len(), 1);
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }
}
