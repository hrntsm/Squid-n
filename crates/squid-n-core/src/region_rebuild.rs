//! 床領域の作り直し（取り込み後・解析前・荷重同期前）。
//!
//! 大梁の区画（[`crate::region_gen::RegionBoundary`]）から床領域（[`FloorRegion`]、
//! 大梁の 1 スパン区画）を再生成し、既存の床領域と重心・レベルで対応付けて
//! 名前・格子解析用の小梁ラインを引き継ぐ。**床板（[`Slab`]）は畳まない**。
//! 各床板の帰属（どの床領域に属すか）を、床板の重心が入る床領域へ付け替えるだけである
//! （申し送り Step 3 改訂 / D7・D10・D20・D21）。取り付く床板（片持ち・バルコニー等）は
//! どの床領域からも参照されない独立した床板のまま、素通しする。

use crate::dof::Dof6Mask;
use crate::geom::{LEVEL_TOL_MM, MEMBER_AXIS_TOL_MM};
use crate::ids::{FloorRegionId, NodeId};
use crate::model::{ElementKind, FloorRegion, LoadTransfer, Model, RegionAnchor, SlabShape};
use crate::region_gen::{
    generate_region_boundaries, polygon_contains_strict, scan_region_boundaries, BOUNDARY_TOL_MM,
};

/// 重心照合で面積が近いとみなす相対許容（新旧の床領域の面積比）。
pub const CENTROID_MATCH_AREA_REL: f64 = 1e-3;

/// [`rebuild_floor_regions`] の件数報告。
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FloorRegionRebuildReport {
    /// 検出した床領域（大梁の区画）の数。
    pub regions: usize,
    /// 旧床領域から名前・小梁ラインを引き継いだ数。
    pub inherited: usize,
    /// 対応する旧床領域が見つからなかった新規の床領域の数。
    pub new_regions: usize,
    /// 重心が新しい床領域に入らなかった旧床領域（名前・小梁ラインを引き継げなかった）の数。
    pub unmatched_old_regions: usize,
    /// 床領域へ帰属し直した床板の数。
    pub slabs_assigned: usize,
    /// どの床領域にも収まらず、取り付く床板へ変換した数。
    pub slabs_converted_to_attached: usize,
    /// どの床領域にも収まらず、変換もできなかった床板の数（警告対象。削除しない）。
    pub unassigned_slabs: usize,
    /// 中点がちょうど 1 つの床領域に厳密内包されなかった小梁の本数。
    pub unassigned_joists: usize,
    /// 参照が 0 になり削除した節点の数。
    pub deleted_nodes: usize,
}

/// 床領域を大梁の区画から作り直し、名前・小梁ラインを引き継ぎ、床板の帰属を
/// 付け替え、小梁を D7 で入れ直し、参照 0 節点を削除する。
///
/// 床板そのもの（`model.slabs`）は畳まない。どの床領域にも収まらない床板は、
/// 1 辺が大梁に全長覆われていれば取り付く床板へ変換し（D20）、それもできなければ
/// 帰属なしのまま残す（警告。落とさない）。
pub fn rebuild_floor_regions(model: &mut Model) -> FloorRegionRebuildReport {
    let scan = scan_region_boundaries(model);
    // 床領域を作り直す前に、領域内小梁を未割当へ集約する（assign_joists が再配分する）。
    for r in &mut model.floor_regions {
        model.unassigned_joists.append(&mut r.secondary_joists);
    }
    let old_regions = std::mem::take(&mut model.floor_regions);
    let mut report = FloorRegionRebuildReport::default();

    // 1. 新しい床領域を床領域ごとに作り、旧床領域と重心・レベルで対応付けて
    //    名前・小梁ラインを引き継ぐ（D10）。
    let mut matched_old = vec![false; old_regions.len()];
    let mut new_regions: Vec<FloorRegion> = Vec::with_capacity(scan.boundaries.len());
    for rb in &scan.boundaries {
        let mut region = FloorRegion::new(FloorRegionId(0), rb.boundary.clone());
        let rb_area = rb.area(model);
        if let Some((oi, old_area)) = old_regions
            .iter()
            .enumerate()
            .filter(|(oi, _)| !matched_old[*oi])
            .filter_map(|(oi, old)| {
                let (cxy, z, area) = boundary_centroid_area(model, &old.boundary)?;
                (rb.is_same_level(z) && rb.contains(model, cxy)).then_some((oi, area))
            })
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

    // 2. 床板の帰属を、重心が入る床領域へ付け替える。収まらない床板は、1 辺が大梁に
    //    全長覆われていれば取り付く床板へ変換し（D20）、それもできなければ帰属なしのまま残す。
    let beams = horizontal_girders(model);
    let mut owner: Vec<Option<usize>> = vec![None; model.slabs.len()];
    for (si, slab) in model.slabs.iter().enumerate() {
        let SlabShape::Enclosed { boundary } = &slab.shape else {
            continue; // 取り付く床板は素通し（どの床領域にも属さない）。
        };
        let Some((cxy, z, _)) = boundary_centroid_area(model, boundary) else {
            continue;
        };
        if let Some((ri, _)) = new_regions
            .iter()
            .enumerate()
            .filter(|(_, r)| region_is_same_level(model, r, z) && region_contains(model, r, cxy))
            .min_by(|(_, a), (_, b)| region_area(model, a).total_cmp(&region_area(model, b)))
        {
            owner[si] = Some(ri);
            report.slabs_assigned += 1;
        }
    }
    let mut converted = Vec::new();
    // 片持ちへ変換すると、旧境界の先端節点が Attached の取付き線 2 点だけに
    // 縮む（D15）。参照 0 になりうるのはこの縮んだぶんの節点だけであり、
    // それ以外（他の理由で未使用の既存節点）まで削除対象にしてはならない（D21）。
    let mut discarded_by_conversion: Vec<NodeId> = Vec::new();
    for (si, slab) in model.slabs.iter().enumerate() {
        if owner[si].is_some() || slab.is_attached() {
            continue;
        }
        let SlabShape::Enclosed { boundary } = &slab.shape else {
            continue;
        };
        if let Some(attached) = try_convert_cantilever(model, boundary, &beams) {
            discarded_by_conversion.extend(boundary.iter().copied());
            converted.push((si, attached));
            report.slabs_converted_to_attached += 1;
        } else {
            report.unassigned_slabs += 1;
        }
    }
    for (si, shape) in converted {
        model.slabs[si].shape = shape;
    }
    for (si, ri) in owner.into_iter().enumerate() {
        if let Some(ri) = ri {
            new_regions[ri].slab_ids.push(model.slabs[si].id);
        }
    }

    for (i, r) in new_regions.iter_mut().enumerate() {
        r.id = FloorRegionId(i as u32);
    }
    model.floor_regions = new_regions;

    // 3. 小梁を D7（中点の厳密内包＋レベル一致）で入れ直す。
    report.unassigned_joists = assign_joists(model);

    // 4. 片持ち変換で縮んだ境界の先端節点のうち、参照が 0 になったものを削除する
    // （D21）。それ以外の既存節点は、たとえ現状どこからも参照されていなくても
    // このリビルドの対象外（利用者が別の理由で置いた節点かもしれない）。
    report.deleted_nodes = delete_unref_nodes(model, &discarded_by_conversion);

    report
}

/// 現状の床領域で、中点がちょうど 1 領域に厳密内包されない小梁の本数。
pub fn unassigned_joist_count(model: &Model) -> usize {
    model.unassigned_joists.len()
}

/// 大梁または小梁で囲まれた床板（`Enclosed`）で、重心がどの床領域にも入らないものの件数。
pub fn floating_slab_count(model: &Model) -> usize {
    let regions = generate_region_boundaries(model);
    model
        .slabs
        .iter()
        .filter(|s| matches!(s.shape, SlabShape::Enclosed { .. }))
        .filter(|s| {
            let Some((cxy, z, _)) = s
                .boundary_nodes()
                .and_then(|b| boundary_centroid_area(model, b))
            else {
                return true;
            };
            !regions
                .iter()
                .any(|r| r.is_same_level(z) && r.contains(model, cxy))
        })
        .count()
}

/// 節点列（境界）の XY 重心・レベル Z・面積を返す。3 点未満は `None`。
fn boundary_centroid_area(model: &Model, boundary: &[NodeId]) -> Option<([f64; 2], f64, f64)> {
    let mut pts = Vec::with_capacity(boundary.len());
    let mut z_sum = 0.0;
    for id in boundary {
        let n = model.nodes.get(id.index())?;
        pts.push([n.coord[0], n.coord[1]]);
        z_sum += n.coord[2];
    }
    if pts.len() < 3 {
        return None;
    }
    let area = shoelace_area(&pts);
    let cxy = shoelace_centroid(&pts, area);
    let z = z_sum / pts.len() as f64;
    Some((cxy, z, area.abs()))
}

fn region_is_same_level(model: &Model, region: &FloorRegion, z: f64) -> bool {
    region
        .level(model)
        .is_some_and(|rz| (rz - z).abs() <= LEVEL_TOL_MM)
}

fn region_contains(model: &Model, region: &FloorRegion, p: [f64; 2]) -> bool {
    let Some(coords) = region.boundary_coords(model) else {
        return false;
    };
    let poly: Vec<[f64; 2]> = coords.iter().map(|c| [c[0], c[1]]).collect();
    polygon_contains_strict(&poly, p)
}

fn region_area(model: &Model, region: &FloorRegion) -> f64 {
    boundary_centroid_area(model, &region.boundary)
        .map(|(_, _, a)| a)
        .unwrap_or(f64::MAX)
}

fn shoelace_area(pts: &[[f64; 2]]) -> f64 {
    let n = pts.len();
    if n < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        sum += a[0] * b[1] - b[0] * a[1];
    }
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
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        let cross = a[0] * b[1] - b[0] * a[1];
        cx += (a[0] + b[0]) * cross;
        cy += (a[1] + b[1]) * cross;
    }
    let six_a = 6.0 * signed_area;
    [cx / six_a, cy / six_a]
}

/// 水平な大梁の 1 本ぶん（XY 線分 ＋ レベル）。壁側の D20 相当判定
/// （[`crate::wall_region_rebuild`]）も同じ「同一レベルの大梁に全長覆われているか」を
/// 使うため `pub(crate)` にしている。
pub(crate) struct GirderSeg {
    a: [f64; 2],
    b: [f64; 2],
    z: f64,
}

pub(crate) fn horizontal_girders(model: &Model) -> Vec<GirderSeg> {
    let mut out = Vec::new();
    for e in &model.elements {
        if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
            continue;
        }
        let (Some(na), Some(nb)) = (
            model.nodes.get(e.nodes[0].index()),
            model.nodes.get(e.nodes[1].index()),
        ) else {
            continue;
        };
        if (na.coord[2] - nb.coord[2]).abs() > LEVEL_TOL_MM {
            continue;
        }
        out.push(GirderSeg {
            a: [na.coord[0], na.coord[1]],
            b: [nb.coord[0], nb.coord[1]],
            z: (na.coord[2] + nb.coord[2]) / 2.0,
        });
    }
    out
}

/// 大梁または小梁で囲まれた床板の境界のうち、1 辺だけが大梁に全長覆われていれば、
/// その辺を取付き線とする取り付く床板の形へ変換する（D20）。変換できなければ `None`。
fn try_convert_cantilever(
    model: &Model,
    boundary: &[NodeId],
    beams: &[GirderSeg],
) -> Option<SlabShape> {
    if boundary.len() < 3 {
        return None;
    }
    let mut xy = Vec::with_capacity(boundary.len());
    let mut z_sum = 0.0;
    for id in boundary {
        let n = model.nodes.get(id.index())?;
        xy.push([n.coord[0], n.coord[1]]);
        z_sum += n.coord[2];
    }
    let z = z_sum / xy.len() as f64;
    let n = boundary.len();
    let mut covered = Vec::new();
    for i in 0..n {
        let a = xy[i];
        let b = xy[(i + 1) % n];
        if edge_fully_covered(a, b, z, beams) {
            covered.push(i);
        }
    }
    if covered.len() != 1 {
        return None;
    }
    let ei = covered[0];
    let n0 = boundary[ei];
    let n1 = boundary[(ei + 1) % n];
    let a = xy[ei];
    let b = xy[(ei + 1) % n];
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let len = (dx * dx + dy * dy).sqrt();
    if len <= f64::EPSILON {
        return None;
    }
    let ux = dx / len;
    let uy = dy / len;
    // 取付き線の左側が正。
    let nx = -uy;
    let ny = ux;

    let mut free = Vec::new();
    for p in &xy {
        if point_segment_dist(*p, a, b) <= BOUNDARY_TOL_MM {
            continue;
        }
        let d = (p[0] - a[0]) * nx + (p[1] - a[1]) * ny;
        let t = (p[0] - a[0]) * ux + (p[1] - a[1]) * uy;
        free.push((t, d));
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

    Some(SlabShape::Attached {
        anchor: RegionAnchor::Line {
            nodes: [n0, n1],
            span: [0.0, 1.0],
            transfer: LoadTransfer::Anchor,
        },
        extent,
    })
}

pub(crate) fn edge_fully_covered(a: [f64; 2], b: [f64; 2], z: f64, beams: &[GirderSeg]) -> bool {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let len = (dx * dx + dy * dy).sqrt();
    if len <= f64::EPSILON {
        return false;
    }
    let ux = dx / len;
    let uy = dy / len;
    let nx = -uy;
    let ny = ux;
    let mut intervals: Vec<(f64, f64)> = Vec::new();
    for beam in beams {
        if (beam.z - z).abs() > LEVEL_TOL_MM {
            continue;
        }
        let da = (beam.a[0] - a[0]) * nx + (beam.a[1] - a[1]) * ny;
        let db = (beam.b[0] - a[0]) * nx + (beam.b[1] - a[1]) * ny;
        if da.abs() > MEMBER_AXIS_TOL_MM || db.abs() > MEMBER_AXIS_TOL_MM {
            continue;
        }
        let ta = (beam.a[0] - a[0]) * ux + (beam.a[1] - a[1]) * uy;
        let tb = (beam.b[0] - a[0]) * ux + (beam.b[1] - a[1]) * uy;
        let t0 = ta.min(tb).clamp(0.0, len);
        let t1 = ta.max(tb).clamp(0.0, len);
        if t1 > t0 {
            intervals.push((t0, t1));
        }
    }
    if intervals.is_empty() {
        return false;
    }
    intervals.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut merged = vec![intervals[0]];
    for &(s, e) in intervals.iter().skip(1) {
        let last = merged.last_mut().expect("merged is non-empty");
        if s <= last.1 + MEMBER_AXIS_TOL_MM {
            last.1 = last.1.max(e);
        } else {
            merged.push((s, e));
        }
    }
    let covered: f64 = merged.iter().map(|(s, e)| e - s).sum();
    covered >= len - MEMBER_AXIS_TOL_MM
}

/// 点 `p` から線分 `a`–`b` までの距離 [mm]。壁側 D20 相当の自由端判定でも使う。
pub(crate) fn point_segment_dist(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1];
    let t = if len2 <= f64::EPSILON {
        0.0
    } else {
        (((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / len2).clamp(0.0, 1.0)
    };
    let q = [a[0] + t * ab[0], a[1] + t * ab[1]];
    ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2)).sqrt()
}

fn joist_midpoint(model: &Model, nodes: [NodeId; 2]) -> Option<([f64; 2], f64)> {
    let a = model.nodes.get(nodes[0].index())?;
    let b = model.nodes.get(nodes[1].index())?;
    Some((
        [
            (a.coord[0] + b.coord[0]) * 0.5,
            (a.coord[1] + b.coord[1]) * 0.5,
        ],
        (a.coord[2] + b.coord[2]) * 0.5,
    ))
}

fn regions_containing(model: &Model, regions: &[FloorRegion], xy: [f64; 2], z: f64) -> Vec<usize> {
    let mut hits = Vec::new();
    for (i, r) in regions.iter().enumerate() {
        if !region_is_same_level(model, r, z) {
            continue;
        }
        if region_contains(model, r, xy) {
            hits.push(i);
        }
    }
    hits
}

fn assign_joists(model: &mut Model) -> usize {
    let joists: Vec<_> = model
        .floor_regions
        .iter_mut()
        .flat_map(|r| r.secondary_joists.drain(..))
        .chain(model.unassigned_joists.drain(..))
        .collect();
    for r in &mut model.floor_regions {
        r.secondary_joists.clear();
    }
    let mut unassigned = 0;
    for sm in joists {
        let nodes = sm.nodes;
        let Some((xy, z)) = joist_midpoint(model, nodes) else {
            model.unassigned_joists.push(sm);
            unassigned += 1;
            continue;
        };
        let hits = regions_containing(model, &model.floor_regions, xy, z);
        if hits.len() == 1 {
            model.floor_regions[hits[0]].secondary_joists.push(sm);
        } else {
            model.unassigned_joists.push(sm);
            unassigned += 1;
        }
    }
    unassigned
}

/// 節点 `id` が部材・二次部材・床領域・床板・壁領域・壁版・拘束・節点荷重・
/// 支点（固定・ばね）・質量・剛床マスターのいずれかから参照されているか。
///
/// 階の節点一覧（`Story::node_ids`）と通り芯（`AxisGroup`）は含めない
/// （[`delete_unref_nodes`] の呼び出し側で扱う節点削除の判定にのみ用いるため。
/// 階の節点一覧は準備計算のたびに階に属する全節点で埋め直されるため、これを
/// 参照とみなすと削除対象の節点がほぼ必ず「参照あり」になってしまい、この関数
/// 自体が実質的に無効化される。通り芯も構造計算に用いない表示専用データである）。
///
/// 床領域・床板・壁領域・壁版・二次部材・拘束・節点荷重の判定は
/// [`Model::node_referenced_by_regions_or_plates`] へ委譲する（`Model::node_in_use`
/// の削除ガードと共有。**`NodeId` を持つフィールドを `Model` へ新設したときに
/// 更新すべき箇所を1箇所へ集約する**のが狙いで、敵対的レビューで見つかった
/// 「`wall_regions` を一切見ていなかった」回帰を踏まえた是正である）。
fn node_has_structural_ref(model: &Model, id: NodeId) -> bool {
    if model.elements.iter().any(|e| e.nodes.contains(&id)) {
        return true;
    }
    if model.node_referenced_by_regions_or_plates(id) {
        return true;
    }
    if let Some(node) = model.nodes.get(id.index()) {
        if node.restraint != Dof6Mask::FREE || node.mass.is_some() || node.support_spring.is_some()
        {
            return true;
        }
    }
    if model.generated_masters.contains(&id) {
        return true;
    }
    false
}

/// `candidates` に挙がった節点のうち、参照が 0 になったものだけを削除する。
///
/// `candidates` 以外の節点は、たとえ現状どこからも参照されていなくても対象外とする
/// （D21。このリビルドが縮めた境界の先端節点だけを削除し、利用者が別の理由で
/// 置いた既存の未使用節点まで巻き込まない）。
pub(crate) fn delete_unref_nodes(model: &mut Model, candidates: &[NodeId]) -> usize {
    let n = model.nodes.len();
    if n == 0 {
        return 0;
    }
    let candidate_set: std::collections::HashSet<NodeId> = candidates.iter().copied().collect();
    let keep: Vec<bool> = (0..n)
        .map(|i| {
            let id = NodeId(i as u32);
            !candidate_set.contains(&id) || node_has_structural_ref(model, id)
        })
        .collect();
    let deleted = keep.iter().filter(|k| !*k).count();
    if deleted == 0 {
        return 0;
    }

    for story in &mut model.stories {
        story
            .node_ids
            .retain(|id| keep.get(id.index()).copied().unwrap_or(false));
    }
    for group in &mut model.axes {
        for axis in &mut group.axes {
            axis.nodes
                .retain(|id| keep.get(id.index()).copied().unwrap_or(false));
        }
    }

    let mut map: Vec<Option<u32>> = vec![None; n];
    let mut new_i = 0u32;
    for i in 0..n {
        if keep[i] {
            map[i] = Some(new_i);
            new_i += 1;
        }
    }
    model.nodes = model
        .nodes
        .drain(..)
        .enumerate()
        .filter(|(i, _)| keep[*i])
        .map(|(_, node)| node)
        .collect();
    model.visit_node_ids(|id| {
        if let Some(new) = map.get(id.0 as usize).and_then(|m| *m) {
            id.0 = new;
        }
    });
    deleted
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{ElemId, FloorRegionId, NodeId, SectionId, SlabId};
    use crate::model::{
        AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime,
        LocalAxis, Node, RegionAnchor, SecondaryMember, SecondaryMemberKind, Slab, SlabPlate,
    };
    use crate::region_gen::generate_region_boundaries;
    use crate::section_shape::SectionShape;

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

    fn plate(section: Option<SectionId>, loads: Vec<AreaLoad>) -> SlabPlate {
        SlabPlate {
            section,
            loads,
            usage: None,
            method: DistributionMethod::TriTrapezoid,
            one_way: None,
        }
    }

    fn enclosed_slab(id: u32, boundary: Vec<u32>, plate: SlabPlate) -> Slab {
        Slab {
            id: SlabId(id),
            shape: SlabShape::Enclosed {
                boundary: boundary.into_iter().map(NodeId).collect(),
            },
            plate,
        }
    }

    fn push_slab_section(model: &mut Model, thickness: f64) -> SectionId {
        let id = SectionId(model.sections.len() as u32);
        model
            .sections
            .push(SectionShape::RcSlab { thickness }.to_section(id, format!("S{thickness:.0}")));
        id
    }

    fn joist(id: u32, i: u32, j: u32) -> SecondaryMember {
        SecondaryMember {
            kind: SecondaryMemberKind::Joist,
            nodes: [NodeId(i), NodeId(j)],
            section: None,
            name: format!("J{id}"),
        }
    }

    fn has_xy(model: &Model, x: f64, y: f64) -> bool {
        model.nodes.iter().any(|n| {
            (n.coord[0] - x).abs() < 1e-6
                && (n.coord[1] - y).abs() < 1e-6
                && n.coord[2].abs() < 1e-6
        })
    }

    /// 4 辺の大梁で閉じた 1 面。上下辺の中間節点に小梁 1 本、床板 2 枚（両方とも保たれる）。
    fn two_piece_square() -> Model {
        let mut model = Model::default();
        for (i, (x, y)) in [
            (0.0, 0.0),
            (2000.0, 0.0),
            (4000.0, 0.0),
            (4000.0, 4000.0),
            (2000.0, 4000.0),
            (0.0, 4000.0),
        ]
        .into_iter()
        .enumerate()
        {
            model.nodes.push(node(i as u32, x, y, 0.0));
        }
        model.elements.extend([
            beam(0, 0, 1),
            beam(1, 1, 2),
            beam(2, 2, 3),
            beam(3, 3, 4),
            beam(4, 4, 5),
            beam(5, 5, 0),
        ]);
        let sid = push_slab_section(&mut model, 150.0);
        model.slabs.push(enclosed_slab(
            0,
            vec![0, 1, 4, 5],
            plate(Some(sid), Vec::new()),
        ));
        model.slabs.push(enclosed_slab(
            1,
            vec![1, 2, 3, 4],
            plate(Some(sid), Vec::new()),
        ));
        model.unassigned_joists.push(joist(0, 1, 4));
        model
    }

    fn cantilever_rect() -> Model {
        let mut model = Model::default();
        model.nodes.push(node(0, 0.0, 0.0, 0.0));
        model.nodes.push(node(1, 4000.0, 0.0, 0.0));
        model.nodes.push(node(2, 4000.0, 1500.0, 0.0));
        model.nodes.push(node(3, 0.0, 1500.0, 0.0));
        model.elements.push(beam(0, 0, 1));
        let sid = push_slab_section(&mut model, 150.0);
        model.slabs.push(enclosed_slab(
            0,
            vec![0, 1, 2, 3],
            plate(Some(sid), Vec::new()),
        ));
        model
    }

    #[test]
    fn test_two_piece_square_keeps_both_slabs_in_one_region() {
        let mut model = two_piece_square();
        assert_eq!(generate_region_boundaries(&model).len(), 1);
        let report = rebuild_floor_regions(&mut model);
        assert_eq!(model.floor_regions.len(), 1, "床領域は 1 つ");
        assert_eq!(model.slabs.len(), 2, "床板は畳まず 2 枚のまま");
        assert_eq!(
            model.floor_regions[0].slab_ids.len(),
            2,
            "2 枚とも同じ床領域へ帰属"
        );
        assert_eq!(
            model.floor_regions[0].secondary_joists.len(),
            1,
            "中央小梁が属する"
        );
        assert_eq!(
            model.floor_regions[0].secondary_joists[0].nodes,
            [NodeId(1), NodeId(4)]
        );
        assert_eq!(report.regions, 1);
        assert_eq!(report.slabs_assigned, 2);
    }

    #[test]
    fn test_rebuild_is_idempotent_on_two_piece_square() {
        let mut model = two_piece_square();
        rebuild_floor_regions(&mut model);
        let first_regions = model.floor_regions.clone();
        let first_slabs = model.slabs.clone();
        let first_nodes = model.nodes.len();
        rebuild_floor_regions(&mut model);
        assert_eq!(model.floor_regions, first_regions);
        assert_eq!(model.slabs, first_slabs);
        assert_eq!(model.nodes.len(), first_nodes);
    }

    #[test]
    fn test_rebuild_preserves_secondary_joists_across_runs() {
        let mut model = two_piece_square();
        rebuild_floor_regions(&mut model);
        assert_eq!(model.joists().count(), 1);
        let first = model
            .floor_regions
            .iter()
            .flat_map(|r| r.secondary_joists.clone())
            .collect::<Vec<_>>();
        rebuild_floor_regions(&mut model);
        assert_eq!(model.joists().count(), 1);
        let second = model
            .floor_regions
            .iter()
            .flat_map(|r| r.secondary_joists.clone())
            .collect::<Vec<_>>();
        assert_eq!(first, second);
    }

    #[test]
    fn test_courtyard_two_internal_regions_no_outer_region() {
        let mut model = Model::default();
        let outer = [(0.0, 0.0), (8000.0, 0.0), (8000.0, 8000.0), (0.0, 8000.0)];
        let inner = [
            (2000.0, 2000.0),
            (6000.0, 2000.0),
            (6000.0, 6000.0),
            (2000.0, 6000.0),
        ];
        for (i, (x, y)) in outer.into_iter().enumerate() {
            model.nodes.push(node(i as u32, x, y, 0.0));
        }
        for (i, (x, y)) in inner.into_iter().enumerate() {
            model.nodes.push(node(4 + i as u32, x, y, 0.0));
        }
        model.elements.extend([
            beam(0, 0, 1),
            beam(1, 1, 2),
            beam(2, 2, 3),
            beam(3, 3, 0),
            beam(4, 4, 5),
            beam(5, 5, 6),
            beam(6, 6, 7),
            beam(7, 7, 4),
        ]);
        let regions = generate_region_boundaries(&model);
        assert_eq!(regions.len(), 2, "内部面は外周と中庭の 2 つ");
        let report = rebuild_floor_regions(&mut model);
        assert_eq!(model.floor_regions.len(), regions.len());
        assert_eq!(report.regions, regions.len());
    }

    #[test]
    fn test_cantilever_rebuild_node_ids_are_compact() {
        let mut model = cantilever_rect();
        rebuild_floor_regions(&mut model);
        assert!(model.validate().is_ok(), "{:?}", model.validate().err());
        for (i, n) in model.nodes.iter().enumerate() {
            assert_eq!(n.id, NodeId(i as u32), "nodes[{i}].id");
        }
    }

    #[test]
    fn test_cantilever_rect_converts_to_attached_line() {
        let mut model = cantilever_rect();
        let report = rebuild_floor_regions(&mut model);
        assert_eq!(model.floor_regions.len(), 0, "囲む大梁がないため床領域は 0");
        assert_eq!(model.slabs.len(), 1);
        let s = &model.slabs[0];
        assert!(s.is_attached());
        match &s.shape {
            SlabShape::Attached {
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
                let dx = b[0] - a[0];
                let dy = b[1] - a[1];
                assert!(dx.abs() > dy.abs(), "取付き線は +X");
                assert!((extent[0] - 1500.0).abs() < 1e-6, "左正 {extent:?}");
                assert!((extent[1] - 1500.0).abs() < 1e-6, "{extent:?}");
            }
            other => panic!("Line の Attached ではない: {other:?}"),
        }
        assert!(has_xy(&model, 0.0, 0.0));
        assert!(has_xy(&model, 4000.0, 0.0));
        assert!(!has_xy(&model, 4000.0, 1500.0), "先端節点は削除");
        assert!(!has_xy(&model, 0.0, 1500.0), "先端節点は削除");
        assert!(report.slabs_converted_to_attached >= 1);
        assert!(report.deleted_nodes >= 2);
    }

    #[test]
    fn test_inherits_name_and_joists_when_boundary_nodes_change() {
        let mut model = Model::default();
        for (i, (x, y)) in [(0.0, 0.0), (4000.0, 0.0), (4000.0, 4000.0), (0.0, 4000.0)]
            .into_iter()
            .enumerate()
        {
            model.nodes.push(node(i as u32, x, y, 0.0));
        }
        model
            .elements
            .extend([beam(0, 0, 1), beam(1, 1, 2), beam(2, 2, 3), beam(3, 3, 0)]);
        let sid = push_slab_section(&mut model, 150.0);
        model.slabs.push(enclosed_slab(
            0,
            vec![0, 1, 2, 3],
            plate(
                Some(sid),
                vec![AreaLoad {
                    kind: "仕上げ".into(),
                    value: 0.001,
                }],
            ),
        ));
        let mut r = FloorRegion::new(
            FloorRegionId(0),
            vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        );
        r.name = "階段室".into();
        model.floor_regions.push(r);

        model.nodes.push(node(4, 2000.0, 0.0, 0.0));
        model.elements[0] = beam(0, 0, 4);
        model.elements.push(beam(4, 4, 1));

        rebuild_floor_regions(&mut model);
        assert_eq!(model.floor_regions.len(), 1);
        assert_eq!(model.floor_regions[0].name, "階段室");
        assert_eq!(model.slabs.len(), 1, "床板は畳まずそのまま残る");
        assert_eq!(
            model.slabs[0].plate.loads,
            vec![AreaLoad {
                kind: "仕上げ".into(),
                value: 0.001,
            }]
        );
    }

    #[test]
    fn test_joist_outside_building_is_unassigned() {
        let mut model = two_piece_square();
        model.nodes.push(node(6, 10000.0, 0.0, 0.0));
        model.nodes.push(node(7, 10000.0, 4000.0, 0.0));
        model.unassigned_joists.push(joist(1, 6, 7));
        let report = rebuild_floor_regions(&mut model);
        assert_eq!(
            model.floor_regions[0].secondary_joists.len(),
            1,
            "領域内小梁"
        );
        assert_eq!(
            model.unassigned_joists.len(),
            1,
            "所属なし小梁は unassigned へ"
        );
        assert_eq!(report.unassigned_joists, 1);
    }

    #[test]
    fn test_floating_slab_stays_enclosed_and_unassigned() {
        let mut model = Model::default();
        for (i, (x, y)) in [(0.0, 0.0), (4000.0, 0.0), (4000.0, 4000.0), (0.0, 4000.0)]
            .into_iter()
            .enumerate()
        {
            model.nodes.push(node(i as u32, x, y, 0.0));
        }
        let sid = push_slab_section(&mut model, 150.0);
        model.slabs.push(enclosed_slab(
            0,
            vec![0, 1, 2, 3],
            plate(Some(sid), Vec::new()),
        ));
        let report = rebuild_floor_regions(&mut model);
        assert_eq!(model.floor_regions.len(), 0, "囲む大梁がないため床領域は 0");
        assert_eq!(model.slabs.len(), 1, "床板は削除しない");
        assert!(
            model.slabs[0].shape
                == SlabShape::Enclosed {
                    boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
                }
        );
        assert_eq!(report.unassigned_slabs, 1);
    }

    #[test]
    fn test_slab_on_two_girder_edges_stays_enclosed_and_unassigned() {
        let mut model = Model::default();
        model.nodes.push(node(0, 0.0, 0.0, 0.0));
        model.nodes.push(node(1, 4000.0, 0.0, 0.0));
        model.nodes.push(node(2, 4000.0, 4000.0, 0.0));
        model.nodes.push(node(3, 0.0, 4000.0, 0.0));
        model.elements.push(beam(0, 0, 1));
        model.elements.push(beam(1, 0, 3));
        let sid = push_slab_section(&mut model, 150.0);
        model.slabs.push(enclosed_slab(
            0,
            vec![0, 1, 2, 3],
            plate(Some(sid), Vec::new()),
        ));
        let report = rebuild_floor_regions(&mut model);
        assert_eq!(model.slabs.len(), 1);
        assert!(
            matches!(model.slabs[0].shape, SlabShape::Enclosed { .. }),
            "出隅相当は Enclosed のまま"
        );
        assert_eq!(report.slabs_converted_to_attached, 0);
        assert_eq!(report.unassigned_slabs, 1);
    }

    #[test]
    fn test_cantilever_tip_that_is_also_girder_end_is_kept() {
        let mut free = cantilever_rect();
        rebuild_floor_regions(&mut free);
        assert!(
            !has_xy(&free, 4000.0, 1500.0) && !has_xy(&free, 0.0, 1500.0),
            "参照 0 の先端は消える"
        );

        let mut shared = cantilever_rect();
        shared.nodes.push(node(4, 4000.0, 3000.0, 0.0));
        shared.elements.push(beam(1, 2, 4));
        let report = rebuild_floor_regions(&mut shared);
        assert!(
            has_xy(&shared, 4000.0, 1500.0),
            "別の大梁端でもある先端は残る"
        );
        assert!(!has_xy(&shared, 0.0, 1500.0), "参照 0 の先端だけ消える");
        assert!(report.slabs_converted_to_attached >= 1);
    }

    /// 階の節点一覧（`Story::node_ids`）に載っているだけでは参照とみなさない。
    ///
    /// ST-Bridge 取り込みは `rebuild_floor_regions` を呼ぶ前に全節点の階所属を
    /// 確定させ、`Story::node_ids` へ登録する。ここを参照とみなすと、削除対象の
    /// 節点がほぼ必ず「参照あり」になり、片持ち変換の先端節点削除が実運用の
    /// モデルでは常に無効化されてしまう。
    #[test]
    fn test_story_node_list_membership_does_not_block_deletion() {
        use crate::model::Story;

        let mut model = cantilever_rect();
        model.stories.push(Story {
            id: crate::ids::StoryId(0),
            name: "1F".into(),
            elevation: 0.0,
            node_ids: model.nodes.iter().map(|n| n.id).collect(),
            seismic_weight: None,
            weight_override: None,
            structure: Default::default(),
            level_kind: Default::default(),
        });

        let report = rebuild_floor_regions(&mut model);
        assert!(!has_xy(&model, 4000.0, 1500.0), "先端は消える");
        assert!(!has_xy(&model, 0.0, 1500.0), "先端は消える");
        assert!(report.deleted_nodes >= 2);
        // 削除した節点は Story::node_ids からも落ちている（ダングリング防止）。
        assert!(model.stories[0]
            .node_ids
            .iter()
            .all(|id| model.nodes.iter().any(|n| n.id == *id)));
    }
}
