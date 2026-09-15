//! 床領域の作り直し（取り込み後・解析前・荷重同期前）。
//!
//! 大梁の区画（[`crate::region_gen::RegionBoundary`]）から床領域（[`FloorRegion`]、
//! 大梁の 1 スパン区画）を再生成し、既存の床領域と重心・レベルで対応付けて
//! 名前を引き継ぐ。**床板（[`Slab`]）は畳まない**。
//! 各床板の帰属（どの床領域に属すか）を、床板の重心が入る床領域へ付け替えるだけである。
//! 取り付く床板（片持ち・バルコニー等）はどの床領域からも参照されない独立した床板とし、
//! 直交して先端まで届く小梁または実梁の位置で支持部材の間の床板ごとに分割し、
//! 部材がない隣り合う床板は統合する。

use crate::dof::Dof6Mask;
use crate::geom::polygon;
use crate::geom::{LEVEL_TOL_MM, MEMBER_AXIS_TOL_MM};
use crate::ids::{FloorRegionId, NodeId};
use crate::model::{
    ElementKind, FloorRegion, LoadTransfer, Model, RegionAnchor, SecondaryMember, SlabShape,
};
use crate::region_gen::{generate_region_boundaries, scan_region_boundaries};

/// 重心照合で面積が近いとみなす相対許容（新旧の床領域の面積比）。
pub const CENTROID_MATCH_AREA_REL: f64 = 1e-3;

/// [`rebuild_floor_regions`] の件数報告。
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FloorRegionRebuildReport {
    /// 検出した床領域（大梁の区画）の数。
    pub regions: usize,
    /// 旧床領域から名前を引き継いだ数。
    pub inherited: usize,
    /// 対応する旧床領域が見つからなかった新規の床領域の数。
    pub new_regions: usize,
    /// 重心が新しい床領域に入らなかった旧床領域（名前を引き継げなかった）の数。
    pub unmatched_old_regions: usize,
    /// 床領域へ帰属し直した床板の数。
    pub slabs_assigned: usize,
    /// どの床領域にも収まらなかった床板の数（警告対象。削除しない）。
    pub unassigned_slabs: usize,
    /// 中点がちょうど 1 つの床領域に厳密内包されなかった小梁の本数。
    pub unassigned_joists: usize,
}

/// 床領域を大梁の区画から作り直し、名前を引き継ぎ、床板の帰属を
/// 付け替え、小梁を入れ直し、参照 0 節点を削除する。
///
/// 床板そのもの（`model.slabs`）は畳まない。どの床領域にも収まらない床板は、
/// 1 辺が大梁に全長覆われていれば取り付く床板へ変換し、それもできなければ
/// 帰属なしのまま残す（警告。落とさない）。取り付く床板は、直交して先端まで届く
/// 小梁または実梁の位置で支持部材の間の床板ごとに分割し、部材がない隣り合う床板は統合する。
pub fn rebuild_floor_regions(model: &mut Model) -> FloorRegionRebuildReport {
    let scan = scan_region_boundaries(model);
    for r in &mut model.floor_regions {
        model.unassigned_joists.append(&mut r.secondary_joists);
    }
    let old_regions = std::mem::take(&mut model.floor_regions);
    let mut report = FloorRegionRebuildReport::default();

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

    let mut owner: Vec<Option<usize>> = vec![None; model.slabs.len()];
    for (si, slab) in model.slabs.iter().enumerate() {
        if slab.is_attached() {
            continue;
        }
        let Some(region) = model.slab_assignment_region(slab.id) else {
            continue;
        };
        let Some(coords) = model.assignment_region_boundary_coords(&region.boundary) else {
            continue;
        };
        let Some((cxy, z, _)) = coords_centroid_area(&coords) else {
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
    for (si, slab) in model.slabs.iter().enumerate() {
        if owner[si].is_none() && !slab.is_attached() {
            report.unassigned_slabs += 1;
        }
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

    report.unassigned_joists = assign_joists(model);

    merge_attached_slabs(model);
    split_attached_slabs_between_members(model);

    report
}

/// 隣り合う取り付く床板（同じ取付き線・版仕様で、境界の張り出し量が一致し、
/// 張り出し量が同じ直線上にあり、統合後の両端の張り出し量が同じ側を向き、
/// 境界に小梁・実部材がないもの）を 1 枚へ統合する。
fn merge_attached_slabs(model: &mut Model) {
    struct Group {
        nodes: [NodeId; 2],
        plate: crate::model::SlabPlate,
        members: Vec<(usize, [f64; 2], [f64; 2])>,
    }
    let mut groups: Vec<Group> = Vec::new();
    for (i, slab) in model.slabs.iter().enumerate() {
        let SlabShape::Attached {
            anchor:
                RegionAnchor::Line {
                    nodes,
                    span,
                    transfer: LoadTransfer::Anchor,
                },
            extent,
        } = &slab.shape
        else {
            continue;
        };
        match groups
            .iter_mut()
            .find(|g| g.nodes == *nodes && g.plate == slab.plate)
        {
            Some(g) => g.members.push((i, *span, *extent)),
            None => groups.push(Group {
                nodes: *nodes,
                plate: slab.plate.clone(),
                members: vec![(i, *span, *extent)],
            }),
        }
    }

    let axes = attachment_split_axes(model);
    let tol = MEMBER_AXIS_TOL_MM;
    const SPAN_EPS: f64 = 1e-9;
    let mut removed = std::collections::HashSet::new();
    let mut merged = 0usize;
    for group in &groups {
        if group.members.len() < 2 {
            continue;
        }
        let mut members = group.members.clone();
        members.sort_by(|a, b| a.1[0].total_cmp(&b.1[0]));
        let (mut keep, mut span, mut extent) = members[0];
        for &(index, cur_span, cur_extent) in &members[1..] {
            let contiguous = (cur_span[0] - span[1]).abs() <= SPAN_EPS;
            let extent_match = (cur_extent[0] - extent[1]).abs() <= tol;
            let span_len = span[1] - span[0];
            let collinear = span_len > SPAN_EPS
                && ((cur_extent[1] - extent[1])
                    - (extent[1] - extent[0]) / span_len * (cur_span[1] - span[1]))
                    .abs()
                    <= tol;
            let same_side = extent[0].abs() <= tol
                || cur_extent[1].abs() <= tol
                || extent[0].signum() == cur_extent[1].signum();
            if contiguous
                && extent_match
                && collinear
                && same_side
                && !attachment_boundary_on_member(model, group.nodes, span[1], extent[1], &axes)
            {
                span[1] = cur_span[1];
                extent[1] = cur_extent[1];
                removed.insert(model.slabs[index].id);
                merged += 1;
            } else {
                set_attached_span(model, keep, group.nodes, span, extent);
                keep = index;
                span = cur_span;
                extent = cur_extent;
            }
        }
        set_attached_span(model, keep, group.nodes, span, extent);
    }
    if merged == 0 {
        return;
    }
    model.retain_slabs(|slab| !removed.contains(&slab.id));
}

fn set_attached_span(
    model: &mut Model,
    index: usize,
    nodes: [NodeId; 2],
    span: [f64; 2],
    extent: [f64; 2],
) {
    model.slabs[index].shape = SlabShape::Attached {
        anchor: RegionAnchor::Line {
            nodes,
            span,
            transfer: LoadTransfer::Anchor,
        },
        extent,
    };
}

/// 取付き線上の位置 `span_pos`・張り出し量 `extent_pos` の境界辺が、小梁または
/// 実部材（連結した 2 節点 `Beam`）の材軸上にあるか。
fn attachment_boundary_on_member(
    model: &Model,
    nodes: [NodeId; 2],
    span_pos: f64,
    extent_pos: f64,
    axes: &[([f64; 3], [f64; 3])],
) -> bool {
    let (Some(a), Some(b)) = (
        model.nodes.get(nodes[0].index()),
        model.nodes.get(nodes[1].index()),
    ) else {
        return false;
    };
    let lerp = |t: f64| {
        [
            a.coord[0] + (b.coord[0] - a.coord[0]) * t,
            a.coord[1] + (b.coord[1] - a.coord[1]) * t,
            a.coord[2] + (b.coord[2] - a.coord[2]) * t,
        ]
    };
    let p0 = lerp(span_pos);
    let d = [b.coord[0] - a.coord[0], b.coord[1] - a.coord[1]];
    let len = (d[0] * d[0] + d[1] * d[1]).sqrt();
    if len <= 1e-9 {
        return false;
    }
    let n = [-d[1] / len, d[0] / len];
    let p1 = [p0[0] + n[0] * extent_pos, p0[1] + n[1] * extent_pos, p0[2]];
    let on_segment = |a: [f64; 3], b: [f64; 3]| {
        point_segment_dist3(p0, a, b) <= MEMBER_AXIS_TOL_MM
            && point_segment_dist3(p1, a, b) <= MEMBER_AXIS_TOL_MM
    };
    axes.iter().any(|(a, b)| on_segment(*a, *b))
}

fn point_segment_dist3(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    if len2 <= 1.0 {
        return crate::geom::vec3::dist(p, a);
    }
    let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let t = ((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2).clamp(0.0, 1.0);
    let proj = [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t];
    crate::geom::vec3::dist(p, proj)
}

/// 取り付く床板（取付き線へ分布）を、下の片持ち小梁または実梁の材軸で
/// 支持部材の間の床板ごとに分割する。
///
/// 境界にするのは、取付き線に直交し（頂点が張り出し方向の直線から
/// [`MEMBER_AXIS_TOL_MM`] 以内）、取付き線から先端まで届く部材だけとする。
/// 取付き線の区間は部分区間 `span` で表し、張り出し量は分割位置で線形に内挿する。
/// すでに支持部材の間ごとに分かれている床板は、境界の部材が内側に来ないため分割されない。
fn split_attached_slabs_between_members(model: &mut Model) {
    let axes = attachment_split_axes(model);
    if axes.is_empty() {
        return;
    }
    let mut si = 0usize;
    while si < model.slabs.len() {
        let split = {
            let slab = &model.slabs[si];
            split_attached_shape(model, slab, &axes)
        };
        let Some(split) = split else {
            si += 1;
            continue;
        };
        let plate = model.slabs[si].plate.clone();
        let (first, rest) = split.split_first().expect("2 枚以上");
        model.slabs[si].shape = first.clone();
        for shape in rest {
            let id = crate::ids::SlabId(model.slabs.len() as u32);
            model.slabs.push(crate::model::Slab {
                id,
                shape: shape.clone(),
                plate: plate.clone(),
            });
        }
        si += 1;
    }
}

/// 取り付く床板の分割・統合に使う材軸。実部材化していない小梁と、端点一致と
/// 同一直線で連結した 2 節点 `Beam`（途中節点の分割を 1 本へ束ねる）。
fn attachment_split_axes(model: &Model) -> Vec<([f64; 3], [f64; 3])> {
    let mut axes: Vec<([f64; 3], [f64; 3])> = model
        .secondary_joist_axes()
        .into_iter()
        .map(|a| (a.a, a.b))
        .collect();
    axes.extend(beam_axes(model));
    axes
}

/// 2 節点 `Beam` の材軸を、端点一致と同一直線で連結して束ねる。
fn beam_axes(model: &Model) -> Vec<([f64; 3], [f64; 3])> {
    let mut segs: Vec<([f64; 3], [f64; 3])> = Vec::new();
    for e in &model.elements {
        if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
            continue;
        }
        let (Some(a), Some(b)) = (
            model.nodes.get(e.nodes[0].index()),
            model.nodes.get(e.nodes[1].index()),
        ) else {
            continue;
        };
        segs.push((a.coord, b.coord));
    }
    let mut used = vec![false; segs.len()];
    let mut axes = Vec::new();
    for i in 0..segs.len() {
        if used[i] {
            continue;
        }
        used[i] = true;
        let (mut p0, mut p1) = segs[i];
        while let Some((j, (np0, np1))) = (0..segs.len())
            .filter(|j| !used[*j])
            .find_map(|j| extend_axis(p0, p1, segs[j].0, segs[j].1).map(|s| (j, s)))
        {
            (p0, p1) = (np0, np1);
            used[j] = true;
        }
        axes.push((p0, p1));
    }
    axes
}

/// 軸 `p0`–`p1` の端と一致する端点を持つ線分 `q0`–`q1` が同一直線上にあるとき、
/// 軸へ継ぎ足した両端を返す。
fn extend_axis(
    p0: [f64; 3],
    p1: [f64; 3],
    q0: [f64; 3],
    q1: [f64; 3],
) -> Option<([f64; 3], [f64; 3])> {
    let at = |a: [f64; 3], b: [f64; 3]| crate::geom::vec3::dist(a, b) <= MEMBER_AXIS_TOL_MM;
    let candidates = [
        (at(p1, q0), p0, q1),
        (at(p1, q1), p0, q0),
        (at(p0, q0), q1, p1),
        (at(p0, q1), q0, p1),
    ];
    candidates
        .into_iter()
        .find(|(connected, end_a, end_b)| {
            *connected
                && point_line_dist3(*end_b, p0, p1) <= MEMBER_AXIS_TOL_MM
                && crate::geom::vec3::dist(*end_a, *end_b) > 1e-9
        })
        .map(|(_, end_a, end_b)| (end_a, end_b))
}

fn point_line_dist3(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    if len2 <= 1.0 {
        return crate::geom::vec3::dist(p, a);
    }
    let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let cross = [
        ap[1] * d[2] - ap[2] * d[1],
        ap[2] * d[0] - ap[0] * d[2],
        ap[0] * d[1] - ap[1] * d[0],
    ];
    (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt() / len2.sqrt()
}

fn cross2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

/// 取り付く床板 1 枚を小梁の材軸で支持部材の間の床板ごとに分割した形。分割が要らなければ `None`。
fn split_attached_shape(
    model: &Model,
    slab: &crate::model::Slab,
    axes: &[([f64; 3], [f64; 3])],
) -> Option<Vec<SlabShape>> {
    let SlabShape::Attached {
        anchor:
            RegionAnchor::Line {
                nodes,
                span,
                transfer: LoadTransfer::Anchor,
            },
        extent,
    } = &slab.shape
    else {
        return None;
    };
    let coords = slab.boundary_coords(model)?;
    if coords.len() != 4 {
        return None;
    }
    let (p0, p1) = (coords[0], coords[1]);
    let d = [p1[0] - p0[0], p1[1] - p0[1]];
    let len = (d[0] * d[0] + d[1] * d[1]).sqrt();
    if len <= 1e-9 {
        return None;
    }
    let u = [d[0] / len, d[1] / len];
    let tol = MEMBER_AXIS_TOL_MM;
    let z = (p0[2] + p1[2]) / 2.0;
    let tip_a = coords[3];
    let tip_b = coords[2];

    let mut fractions: Vec<f64> = Vec::new();
    for (qa, qb) in axes {
        let (qa, qb) = (*qa, *qb);
        if (qa[2] - z).abs() > LEVEL_TOL_MM || (qb[2] - z).abs() > LEVEL_TOL_MM {
            continue;
        }
        let j = [qb[0] - qa[0], qb[1] - qa[1]];
        let jl = (j[0] * j[0] + j[1] * j[1]).sqrt();
        if jl <= tol {
            continue;
        }
        let denom = cross2(d, j);
        if denom.abs() <= 1e-9 {
            continue;
        }
        let r = [qa[0] - p0[0], qa[1] - p0[1]];
        let s = cross2(r, j) / denom;
        let t = cross2(r, d) / denom;
        if s * len <= tol || s * len >= len - tol {
            continue;
        }
        if t * jl < -tol || t * jl > jl + tol {
            continue;
        }
        let p_att = [p0[0] + s * d[0], p0[1] + s * d[1]];
        let perp = |q: [f64; 3]| ((q[0] - p_att[0]) * u[0] + (q[1] - p_att[1]) * u[1]).abs();
        if perp(qa) > tol || perp(qb) > tol {
            continue;
        }
        let tip_d = [tip_b[0] - tip_a[0], tip_b[1] - tip_a[1]];
        let tip_len = (tip_d[0] * tip_d[0] + tip_d[1] * tip_d[1]).sqrt();
        if tip_len <= tol {
            continue;
        }
        let denom2 = cross2(tip_d, j);
        if denom2.abs() <= 1e-9 {
            continue;
        }
        let r2 = [qa[0] - tip_a[0], qa[1] - tip_a[1]];
        let s2 = cross2(r2, j) / denom2;
        let t2 = cross2(r2, tip_d) / denom2;
        if s2 < -tol / tip_len || s2 > 1.0 + tol / tip_len {
            continue;
        }
        if t2 * jl < -tol || t2 * jl > jl + tol {
            continue;
        }
        fractions.push(s);
    }

    if fractions.is_empty() {
        return None;
    }
    fractions.sort_by(f64::total_cmp);
    let mut cuts: Vec<f64> = Vec::new();
    for f in fractions {
        if cuts.last().is_some_and(|prev| (f - prev) * len <= tol) {
            continue;
        }
        cuts.push(f);
    }

    let mut bounds = Vec::with_capacity(cuts.len() + 2);
    bounds.push(0.0);
    bounds.extend(cuts);
    bounds.push(1.0);
    let interp = |v: [f64; 2], f: f64| v[0] + (v[1] - v[0]) * f;
    let mut shapes = Vec::with_capacity(bounds.len() - 1);
    for w in bounds.windows(2) {
        let (f0, f1) = (w[0], w[1]);
        shapes.push(SlabShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: *nodes,
                span: [interp(*span, f0), interp(*span, f1)],
                transfer: LoadTransfer::Anchor,
            },
            extent: [interp(*extent, f0), interp(*extent, f1)],
        });
    }
    (shapes.len() >= 2).then_some(shapes)
}

/// 現状の床領域で、中点がちょうど 1 領域に厳密内包されない小梁の本数
/// （片持ち小梁と実部材化済みの小梁は支持辺・実要素として扱うため除く）。
pub fn unassigned_joist_count(model: &Model) -> usize {
    model
        .unassigned_joists
        .iter()
        .filter(|sm| !sm.is_cantilever() && !model.secondary_member_materialized(sm))
        .count()
}

/// 大梁または小梁で囲まれた床板（`Enclosed`）で、重心がどの床領域にも入らないものの件数。
pub fn floating_slab_count(model: &Model) -> usize {
    let regions = generate_region_boundaries(model);
    model
        .slabs
        .iter()
        .filter(|s| matches!(s.shape, SlabShape::Enclosed))
        .filter(|s| {
            let Some((cxy, z, _)) = s
                .boundary_nodes(model)
                .and_then(|b| boundary_centroid_area(model, &b))
            else {
                return true;
            };
            !regions
                .iter()
                .any(|r| r.is_same_level(z) && r.contains(model, cxy))
        })
        .count()
}

/// 境界節点列の XY 重心・レベル Z・面積を返す。3 点未満は `None`。
fn boundary_centroid_area(model: &Model, boundary: &[NodeId]) -> Option<([f64; 2], f64, f64)> {
    let mut coords = Vec::with_capacity(boundary.len());
    for id in boundary {
        coords.push(model.nodes.get(id.index())?.coord);
    }
    coords_centroid_area(&coords)
}

/// 境界座標列の XY 重心・レベル Z・面積を返す。3 点未満は `None`。
fn coords_centroid_area(coords: &[[f64; 3]]) -> Option<([f64; 2], f64, f64)> {
    if coords.len() < 3 {
        return None;
    }
    let pts: Vec<[f64; 2]> = coords.iter().map(|c| [c[0], c[1]]).collect();
    let area = polygon::signed_area(&pts);
    let cxy = polygon::centroid(&pts);
    let z = coords.iter().map(|c| c[2]).sum::<f64>() / coords.len() as f64;
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
    polygon::contains_excluding_boundary(&poly, p)
}

fn region_area(model: &Model, region: &FloorRegion) -> f64 {
    boundary_centroid_area(model, &region.boundary)
        .map(|(_, _, a)| a)
        .unwrap_or(f64::MAX)
}

/// 水平な大梁の 1 本ぶん（XY 線分 ＋ レベル）。壁側の相当判定
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

fn joist_midpoint(model: &Model, sm: &SecondaryMember) -> Option<([f64; 2], f64)> {
    let (a, b) = model.secondary_member_end_points(sm)?;
    Some((
        [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5],
        (a[2] + b[2]) * 0.5,
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
        let counted = !sm.is_cantilever() && !model.secondary_member_materialized(&sm);
        let Some((xy, z)) = joist_midpoint(model, &sm) else {
            if counted {
                unassigned += 1;
            }
            model.unassigned_joists.push(sm);
            continue;
        };
        let hits = regions_containing(model, &model.floor_regions, xy, z);
        if hits.len() == 1 {
            model.floor_regions[hits[0]].secondary_joists.push(sm);
        } else {
            if counted {
                unassigned += 1;
            }
            model.unassigned_joists.push(sm);
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
/// の削除ガードと共有）。
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
/// （このリビルドが縮めた境界の先端節点だけを削除し、利用者が別の理由で
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

    /// 境界節点列で囲まれた床板と割当領域を追加する（テスト用）。
    fn push_enclosed(model: &mut Model, boundary: &[u32], plate: SlabPlate) -> SlabId {
        let nodes: Vec<NodeId> = boundary.iter().map(|i| NodeId(*i)).collect();
        model.add_enclosed_slab_from_nodes(&nodes, plate)
    }

    fn push_slab_section(model: &mut Model, thickness: f64) -> SectionId {
        let id = SectionId(model.sections.len() as u32);
        model
            .sections
            .push(SectionShape::RcSlab { thickness }.to_section(id, format!("S{thickness:.0}")));
        id
    }

    fn joist(id: u32, coords: [[f64; 3]; 2]) -> SecondaryMember {
        SecondaryMember {
            gravity_end_shares: None,
            kind: SecondaryMemberKind::Joist,
            ends: crate::model::SecondaryMemberEnds::Detached(coords),
            section: None,
            name: format!("J{id}"),
            id: crate::ids::SecondaryMemberId(id),
        }
    }

    fn joist_of(model: &Model, id: u32, i: u32, j: u32) -> SecondaryMember {
        joist(
            id,
            [model.nodes[i as usize].coord, model.nodes[j as usize].coord],
        )
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
        model
            .unassigned_joists
            .push(joist(0, [[2000.0, 0.0, 0.0], [2000.0, 4000.0, 0.0]]));
        push_enclosed(&mut model, &[0, 1, 4, 5], plate(Some(sid), Vec::new()));
        push_enclosed(&mut model, &[1, 2, 3, 4], plate(Some(sid), Vec::new()));
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
        model.slabs.push(Slab {
            id: SlabId(0),
            shape: SlabShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: [0.0, 1.0],
                    transfer: LoadTransfer::Anchor,
                },
                extent: [1500.0, 1500.0],
            },
            plate: plate(Some(sid), Vec::new()),
        });
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
            model.floor_regions[0].secondary_joists[0].id,
            crate::ids::SecondaryMemberId(0)
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
    fn test_cantilever_rebuild_keeps_attached_slab() {
        let mut model = cantilever_rect();
        let report = rebuild_floor_regions(&mut model);
        assert_eq!(model.floor_regions.len(), 0, "囲む大梁がないため床領域は 0");
        assert_eq!(model.slabs.len(), 1);
        let s = &model.slabs[0];
        assert!(s.is_attached());
        match &s.shape {
            SlabShape::Attached {
                anchor: RegionAnchor::Line { span, transfer, .. },
                extent,
            } => {
                assert_eq!(*span, [0.0, 1.0]);
                assert_eq!(*transfer, LoadTransfer::Anchor);
                assert!((extent[0] - 1500.0).abs() < 1e-6, "{extent:?}");
                assert!((extent[1] - 1500.0).abs() < 1e-6, "{extent:?}");
            }
            other => panic!("Line の Attached ではない: {other:?}"),
        }
        assert!(report.unassigned_slabs == 0);
    }

    /// 取り付く床板は、直交して先端まで届く小梁の位置で支持部材の間の床板ごとに分割される。
    #[test]
    fn test_cantilever_splits_at_perpendicular_joists() {
        let mut model = cantilever_rect();
        model.nodes.push(node(4, 2000.0, 0.0, 0.0));
        model.nodes.push(node(5, 2000.0, 1500.0, 0.0));
        model.unassigned_joists.push(joist_of(&model, 0, 4, 5));
        rebuild_floor_regions(&mut model);
        assert_eq!(model.slabs.len(), 2);
        for (i, expected) in [[0.0, 0.5], [0.5, 1.0]].iter().enumerate() {
            match &model.slabs[i].shape {
                SlabShape::Attached {
                    anchor: RegionAnchor::Line { span, .. },
                    extent,
                } => {
                    assert!((span[0] - expected[0]).abs() < 1e-9, "{span:?}");
                    assert!((span[1] - expected[1]).abs() < 1e-9, "{span:?}");
                    assert!((extent[0] - 1500.0).abs() < 1e-6, "{extent:?}");
                    assert!((extent[1] - 1500.0).abs() < 1e-6, "{extent:?}");
                }
                other => panic!("Attached ではない: {other:?}"),
            }
        }

        rebuild_floor_regions(&mut model);
        assert_eq!(model.slabs.len(), 2, "冪等");
    }

    /// 床板の内部を通る実梁（取付き線から先端まで届く）でも支持部材の間の床板ごとに分割される。
    #[test]
    fn test_cantilever_splits_at_real_beam() {
        let mut model = cantilever_rect();
        model.nodes.push(node(4, 2000.0, 0.0, 0.0));
        model.nodes.push(node(5, 2000.0, 1500.0, 0.0));
        model.elements.push(beam(1, 4, 5));
        rebuild_floor_regions(&mut model);
        assert_eq!(
            model.slabs.len(),
            2,
            "実梁の位置で支持部材の間の床板 2 枚へ分割"
        );
        for (i, expected) in [[0.0, 0.5], [0.5, 1.0]].iter().enumerate() {
            match &model.slabs[i].shape {
                SlabShape::Attached {
                    anchor: RegionAnchor::Line { span, .. },
                    extent,
                } => {
                    assert!((span[0] - expected[0]).abs() < 1e-9, "{span:?}");
                    assert!((span[1] - expected[1]).abs() < 1e-9, "{span:?}");
                    assert!((extent[0] - 1500.0).abs() < 1e-6, "{extent:?}");
                    assert!((extent[1] - 1500.0).abs() < 1e-6, "{extent:?}");
                }
                other => panic!("Attached ではない: {other:?}"),
            }
        }

        rebuild_floor_regions(&mut model);
        assert_eq!(model.slabs.len(), 2, "冪等");
    }

    /// 実梁が途中節点で 2 要素に分かれていても、連結した全長で支持部材の間の床板ごとに分割される。
    #[test]
    fn test_cantilever_splits_at_spliced_real_beam() {
        let mut model = cantilever_rect();
        model.nodes.push(node(4, 2000.0, 0.0, 0.0));
        model.nodes.push(node(5, 2000.0, 800.0, 0.0));
        model.nodes.push(node(6, 2000.0, 1500.0, 0.0));
        model.elements.push(beam(1, 4, 5));
        model.elements.push(beam(2, 5, 6));
        rebuild_floor_regions(&mut model);
        assert_eq!(model.slabs.len(), 2, "2 要素の実梁でも分割");
        for (i, expected) in [[0.0, 0.5], [0.5, 1.0]].iter().enumerate() {
            match &model.slabs[i].shape {
                SlabShape::Attached {
                    anchor: RegionAnchor::Line { span, .. },
                    extent,
                } => {
                    assert!((span[0] - expected[0]).abs() < 1e-9, "{span:?}");
                    assert!((span[1] - expected[1]).abs() < 1e-9, "{span:?}");
                    assert!((extent[0] - 1500.0).abs() < 1e-6, "{extent:?}");
                    assert!((extent[1] - 1500.0).abs() < 1e-6, "{extent:?}");
                }
                other => panic!("Attached ではない: {other:?}"),
            }
        }

        rebuild_floor_regions(&mut model);
        assert_eq!(model.slabs.len(), 2, "冪等");
    }

    /// 小梁を消すと、同じ版仕様の隣り合う床板は 1 枚に統合される。
    #[test]
    fn test_cantilever_slabs_merge_when_joist_is_removed() {
        let mut model = cantilever_rect();
        model.nodes.push(node(4, 2000.0, 0.0, 0.0));
        model.nodes.push(node(5, 2000.0, 1500.0, 0.0));
        model.unassigned_joists.push(joist_of(&model, 0, 4, 5));
        rebuild_floor_regions(&mut model);
        assert_eq!(model.slabs.len(), 2);

        model.unassigned_joists.clear();
        rebuild_floor_regions(&mut model);
        assert_eq!(model.slabs.len(), 1, "小梁を消すと統合");
        match &model.slabs[0].shape {
            SlabShape::Attached {
                anchor: RegionAnchor::Line { span, .. },
                extent,
            } => {
                assert!((span[0] - 0.0).abs() < 1e-9);
                assert!((span[1] - 1.0).abs() < 1e-9);
                assert!((extent[0] - 1500.0).abs() < 1e-6);
                assert!((extent[1] - 1500.0).abs() < 1e-6);
            }
            other => panic!("Attached ではない: {other:?}"),
        }
        assert!(model.validate().is_ok(), "{:?}", model.validate().err());
    }

    /// 小梁が 2 本あれば支持部材の間の床板 3 枚に分割される。
    #[test]
    fn test_cantilever_splits_into_three_slabs() {
        let mut model = cantilever_rect();
        for (i, x) in [1000.0_f64, 3000.0].into_iter().enumerate() {
            let (base, tip) = (4 + 2 * i as u32, 5 + 2 * i as u32);
            model.nodes.push(node(base, x, 0.0, 0.0));
            model.nodes.push(node(tip, x, 1500.0, 0.0));
            model
                .unassigned_joists
                .push(joist_of(&model, i as u32, base, tip));
        }
        rebuild_floor_regions(&mut model);
        assert_eq!(model.slabs.len(), 3);
        let spans: Vec<[f64; 2]> = model
            .slabs
            .iter()
            .map(|s| match &s.shape {
                SlabShape::Attached {
                    anchor: RegionAnchor::Line { span, .. },
                    ..
                } => *span,
                other => panic!("Attached ではない: {other:?}"),
            })
            .collect();
        assert!((spans[0][0] - 0.0).abs() < 1e-9);
        assert!((spans[0][1] - 0.25).abs() < 1e-9, "{spans:?}");
        assert!((spans[1][0] - 0.25).abs() < 1e-9, "{spans:?}");
        assert!((spans[1][1] - 0.75).abs() < 1e-9, "{spans:?}");
        assert!((spans[2][0] - 0.75).abs() < 1e-9, "{spans:?}");
        assert!((spans[2][1] - 1.0).abs() < 1e-9, "{spans:?}");
    }

    /// 両端の柱へ集中（`Columns`）の取り付く床板は、小梁があっても分割しない。
    #[test]
    fn test_attached_columns_slab_is_not_split() {
        let mut model = Model::default();
        model.nodes.push(node(0, 0.0, 0.0, 0.0));
        model.nodes.push(node(1, 4000.0, 0.0, 0.0));
        model.nodes.push(node(2, 2000.0, 0.0, 0.0));
        model.nodes.push(node(3, 2000.0, 1500.0, 0.0));
        model.elements.push(beam(0, 0, 1));
        model.unassigned_joists.push(joist_of(&model, 0, 2, 3));
        let sid = push_slab_section(&mut model, 150.0);
        model.slabs.push(Slab {
            id: SlabId(0),
            shape: SlabShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: [0.0, 1.0],
                    transfer: LoadTransfer::Columns,
                },
                extent: [1500.0, 1500.0],
            },
            plate: plate(Some(sid), Vec::new()),
        });
        rebuild_floor_regions(&mut model);
        assert_eq!(model.slabs.len(), 1, "Columns は分割しない");
    }

    /// 張り出し量が境界で折れる・段差がある・span が連続しない場合は統合しない。
    #[test]
    fn test_attached_slabs_merge_only_when_collinear() {
        fn pair(span1: [f64; 2], ext1: [f64; 2], span2: [f64; 2], ext2: [f64; 2]) -> Model {
            let mut model = Model::default();
            model.nodes.push(node(0, 0.0, 0.0, 0.0));
            model.nodes.push(node(1, 4000.0, 0.0, 0.0));
            let sid = push_slab_section(&mut model, 150.0);
            for (id, span, extent) in [(0u32, span1, ext1), (1, span2, ext2)] {
                model.slabs.push(Slab {
                    id: SlabId(id),
                    shape: SlabShape::Attached {
                        anchor: RegionAnchor::Line {
                            nodes: [NodeId(0), NodeId(1)],
                            span,
                            transfer: LoadTransfer::Anchor,
                        },
                        extent,
                    },
                    plate: plate(Some(sid), Vec::new()),
                });
            }
            model
        }

        let mut straight = pair([0.0, 0.5], [1000.0, 2000.0], [0.5, 1.0], [2000.0, 3000.0]);
        rebuild_floor_regions(&mut straight);
        assert_eq!(straight.slabs.len(), 1, "直線的なら統合");

        let mut kinked = pair([0.0, 0.5], [1000.0, 2000.0], [0.5, 1.0], [2000.0, 1000.0]);
        rebuild_floor_regions(&mut kinked);
        assert_eq!(kinked.slabs.len(), 2, "折れるなら統合しない");

        let mut stepped = pair([0.0, 0.5], [1000.0, 2000.0], [0.5, 1.0], [2100.0, 2200.0]);
        rebuild_floor_regions(&mut stepped);
        assert_eq!(stepped.slabs.len(), 2, "段差なら統合しない");

        let mut gapped = pair([0.0, 0.4], [1000.0, 2000.0], [0.5, 1.0], [2000.0, 3000.0]);
        rebuild_floor_regions(&mut gapped);
        assert_eq!(gapped.slabs.len(), 2, "span が連続しないなら統合しない");

        // 境界の張り出し量が 0 で前後が逆側へ向かう（自己交差）なら統合しない。
        let mut flipped = pair([0.0, 0.5], [500.0, 0.0], [0.5, 1.0], [0.0, -500.0]);
        rebuild_floor_regions(&mut flipped);
        assert_eq!(flipped.slabs.len(), 2, "自己交差になるなら統合しない");

        // 境界に実部材（2 節点 Beam）があるなら統合しない。
        let mut with_beam = pair([0.0, 0.5], [1000.0, 2000.0], [0.5, 1.0], [2000.0, 3000.0]);
        with_beam.nodes.push(node(2, 2000.0, 0.0, 0.0));
        with_beam.nodes.push(node(3, 2000.0, 2000.0, 0.0));
        with_beam.elements.push(beam(0, 2, 3));
        rebuild_floor_regions(&mut with_beam);
        assert_eq!(with_beam.slabs.len(), 2, "境界に実部材があれば統合しない");

        // 境界の実部材が途中節点で 2 要素に分かれていても統合しない。
        let mut spliced = pair([0.0, 0.5], [1000.0, 2000.0], [0.5, 1.0], [2000.0, 3000.0]);
        spliced.nodes.push(node(2, 2000.0, 0.0, 0.0));
        spliced.nodes.push(node(3, 2000.0, 1000.0, 0.0));
        spliced.nodes.push(node(4, 2000.0, 2000.0, 0.0));
        spliced.elements.push(beam(0, 2, 3));
        spliced.elements.push(beam(1, 3, 4));
        rebuild_floor_regions(&mut spliced);
        assert_eq!(
            spliced.slabs.len(),
            2,
            "境界の実部材が 2 要素でも統合しない"
        );
    }

    /// 取付き線に直交しない小梁、先端まで届かない小梁は分割境界にしない。
    #[test]
    fn test_cantilever_does_not_split_at_skewed_or_partial_joists() {
        let mut skewed = cantilever_rect();
        skewed.nodes.push(node(4, 1000.0, 0.0, 0.0));
        skewed.nodes.push(node(5, 2000.0, 1500.0, 0.0));
        skewed.unassigned_joists.push(joist_of(&skewed, 0, 4, 5));
        rebuild_floor_regions(&mut skewed);
        assert_eq!(skewed.slabs.len(), 1, "斜めは分割しない");

        let mut partial = cantilever_rect();
        partial.nodes.push(node(4, 2000.0, 0.0, 0.0));
        partial.nodes.push(node(5, 2000.0, 800.0, 0.0));
        partial.unassigned_joists.push(joist_of(&partial, 0, 4, 5));
        rebuild_floor_regions(&mut partial);
        assert_eq!(partial.slabs.len(), 1, "先端まで届かない");
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
        push_enclosed(
            &mut model,
            &[0, 1, 2, 3],
            plate(
                Some(sid),
                vec![AreaLoad {
                    kind: "仕上げ".into(),
                    value: 0.001,
                }],
            ),
        );
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
        model.unassigned_joists.push(joist_of(&model, 1, 6, 7));
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
        model.slabs.push(Slab {
            id: SlabId(0),
            shape: SlabShape::Enclosed,
            plate: plate(Some(sid), Vec::new()),
        });
        let report = rebuild_floor_regions(&mut model);
        assert_eq!(model.floor_regions.len(), 0, "囲む大梁がないため床領域は 0");
        assert_eq!(model.slabs.len(), 1, "床板は削除しない");
        assert!(model.slabs[0].shape == SlabShape::Enclosed);
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
        model.slabs.push(Slab {
            id: SlabId(0),
            shape: SlabShape::Enclosed,
            plate: plate(Some(sid), Vec::new()),
        });
        let report = rebuild_floor_regions(&mut model);
        assert_eq!(model.slabs.len(), 1);
        assert!(
            matches!(model.slabs[0].shape, SlabShape::Enclosed),
            "出隅相当は Enclosed のまま"
        );
        assert_eq!(report.unassigned_slabs, 1);
    }
}
