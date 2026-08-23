//! 床領域の作り直し（取り込み後・解析前・荷重同期前）。
//!
//! 大梁パネルから囲まれ領域を再生成し、版・名前を引き継ぎ、片持ちを変換し、
//! 小梁を D7 で入れ直し、参照 0 の節点を削除する（申し送り Step 3 / D7 D10 D20 D21）。
//! Attached 領域は消さない。

use crate::dof::Dof6Mask;
use crate::geom::{LEVEL_TOL_MM, MEMBER_AXIS_TOL_MM};
use crate::ids::{FloorRegionId, NodeId};
use crate::model::{
    Constraint, ElementKind, FloorRegion, LoadTransfer, Model, RegionAnchor, RegionShape,
    SecondaryMemberKind,
};
use crate::region_gen::{
    generate_floor_panels, polygon_contains_strict, scan_floor_panels, BOUNDARY_TOL_MM,
};

/// 重心照合で面積が近いとみなす相対許容（新旧パネルの面積比）。
pub const CENTROID_MATCH_AREA_REL: f64 = 1e-3;

/// [`rebuild_floor_regions`] の件数報告。
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FloorRegionRebuildReport {
    pub enclosed: usize,
    pub attached_kept: usize,
    pub attached_converted: usize,
    pub inherited: usize,
    pub folded_panels: usize,
    pub new_plateless: usize,
    pub unmatched_old_enclosed: usize,
    pub mixed_plate_panels: usize,
    pub unassigned_plates: usize,
    pub unassigned_joists: usize,
    pub deleted_nodes: usize,
}

/// 囲まれ領域をパネルから作り直し、版・名前を引き継ぎ、片持ち変換し、
/// 小梁を D7 で入れ直し、参照 0 節点を削除する。Attached は消さない。
pub fn rebuild_floor_regions(model: &mut Model) -> FloorRegionRebuildReport {
    let scan = scan_floor_panels(model);
    let old_regions = std::mem::take(&mut model.floor_regions);
    let mut old_attached = Vec::new();
    let mut old_enclosed = Vec::new();
    for r in old_regions {
        match &r.shape {
            RegionShape::Attached { .. } => old_attached.push(r),
            RegionShape::Enclosed { .. } => old_enclosed.push(r),
        }
    }
    let attached_kept = old_attached.len();

    let mut owned: Vec<Vec<usize>> = vec![Vec::new(); scan.panels.len()];
    let mut assigned = vec![false; old_enclosed.len()];
    for (oi, old) in old_enclosed.iter().enumerate() {
        let Some((cxy, z, _)) = enclosed_centroid(model, old) else {
            continue;
        };
        if let Some(pi) = scan
            .panels
            .iter()
            .enumerate()
            .filter(|(_, p)| p.is_same_level(z) && p.contains(model, cxy))
            .min_by(|(ia, pa), (ib, pb)| {
                pa.area(model)
                    .total_cmp(&pb.area(model))
                    .then_with(|| ia.cmp(ib))
            })
            .map(|(i, _)| i)
        {
            owned[pi].push(oi);
            assigned[oi] = true;
        }
    }

    let mut report = FloorRegionRebuildReport {
        attached_kept,
        ..FloorRegionRebuildReport::default()
    };

    let mut new_regions = Vec::new();
    for (pi, panel) in scan.panels.iter().enumerate() {
        let mut region = FloorRegion::enclosed(FloorRegionId(0), panel.boundary.clone());
        match owned[pi].as_slice() {
            [] => {
                report.new_plateless += 1;
            }
            [only] => {
                let old = &old_enclosed[*only];
                region.plate = old.plate.clone();
                region.name = old.name.clone();
                let panel_a = panel.area(model);
                let old_a = enclosed_centroid(model, old)
                    .map(|(_, _, a)| a)
                    .unwrap_or(0.0);
                let denom = panel_a.abs().max(f64::EPSILON);
                if (old_a - panel_a).abs() / denom < CENTROID_MATCH_AREA_REL {
                    report.inherited += 1;
                } else {
                    report.unmatched_old_enclosed += 1;
                }
            }
            many => {
                let plates: Vec<_> = many
                    .iter()
                    .map(|&i| old_enclosed[i].plate.clone())
                    .collect();
                let all_eq = plates.windows(2).all(|w| w[0] == w[1]);
                if all_eq {
                    region.plate = plates.into_iter().next().flatten();
                    report.folded_panels += 1;
                    let names: Vec<&str> = many
                        .iter()
                        .map(|&i| old_enclosed[i].name.as_str())
                        .collect();
                    if names.iter().all(|n| !n.is_empty() && *n == names[0]) {
                        region.name = names[0].to_string();
                    }
                } else {
                    region.plate = None;
                    report.mixed_plate_panels += 1;
                }
            }
        }
        new_regions.push(region);
    }

    let mut converted = Vec::new();
    let mut leftover = Vec::new();
    let beams = horizontal_girders(model);
    for (oi, old) in old_enclosed.into_iter().enumerate() {
        if assigned[oi] {
            continue;
        }
        if let Some(attached) = try_convert_cantilever(model, &old, &beams) {
            converted.push(attached);
            report.attached_converted += 1;
        } else {
            if old.plate.is_some() {
                report.unassigned_plates += 1;
            }
            leftover.push(old);
        }
    }

    for mut r in old_attached {
        r.secondary_joist_ids.clear();
        new_regions.push(r);
    }
    new_regions.append(&mut converted);
    for mut r in leftover {
        r.secondary_joist_ids.clear();
        new_regions.push(r);
    }

    for (i, r) in new_regions.iter_mut().enumerate() {
        r.id = FloorRegionId(i as u32);
        r.secondary_joist_ids.clear();
    }

    report.unassigned_joists = assign_joists(model, &mut new_regions);
    report.enclosed = new_regions
        .iter()
        .filter(|r| matches!(r.shape, RegionShape::Enclosed { .. }))
        .count();

    model.floor_regions = new_regions;
    report.deleted_nodes = delete_unref_nodes(model);

    report
}

/// 現状の床領域で、中点がちょうど 1 領域に厳密内包されない小梁の本数。
pub fn unassigned_joist_count(model: &Model) -> usize {
    model
        .secondary_members
        .iter()
        .filter(|sm| sm.kind == SecondaryMemberKind::Joist)
        .filter(|sm| {
            let Some((xy, z)) = joist_midpoint(model, sm.nodes) else {
                return true;
            };
            regions_containing(model, &model.floor_regions, xy, z).len() != 1
        })
        .count()
}

/// 囲まれ＋版ありで、重心がどの大梁パネルにも入らないものの件数。
pub fn floating_plate_count(model: &Model) -> usize {
    let panels = generate_floor_panels(model);
    model
        .floor_regions
        .iter()
        .filter(|r| r.plate.is_some())
        .filter(|r| matches!(r.shape, RegionShape::Enclosed { .. }))
        .filter(|r| {
            let Some((cxy, z, _)) = enclosed_centroid(model, r) else {
                return true;
            };
            !panels
                .iter()
                .any(|p| p.is_same_level(z) && p.contains(model, cxy))
        })
        .count()
}

fn enclosed_centroid(model: &Model, region: &FloorRegion) -> Option<([f64; 2], f64, f64)> {
    let RegionShape::Enclosed { boundary } = &region.shape else {
        return None;
    };
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

struct GirderSeg {
    a: [f64; 2],
    b: [f64; 2],
    z: f64,
}

fn horizontal_girders(model: &Model) -> Vec<GirderSeg> {
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

fn try_convert_cantilever(
    model: &Model,
    old: &FloorRegion,
    beams: &[GirderSeg],
) -> Option<FloorRegion> {
    let RegionShape::Enclosed { boundary } = &old.shape else {
        return None;
    };
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

    Some(FloorRegion {
        id: FloorRegionId(0),
        name: old.name.clone(),
        shape: RegionShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [n0, n1],
                span: [0.0, 1.0],
                transfer: LoadTransfer::Anchor,
            },
            extent,
        },
        plate: old.plate.clone(),
        secondary_joist_ids: Vec::new(),
    })
}

fn edge_fully_covered(a: [f64; 2], b: [f64; 2], z: f64, beams: &[GirderSeg]) -> bool {
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

fn point_segment_dist(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
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

fn region_xy_poly(model: &Model, region: &FloorRegion) -> Option<(Vec<[f64; 2]>, f64)> {
    let coords = region.boundary_coords(model)?;
    if coords.len() < 3 {
        return None;
    }
    let z = coords.iter().map(|c| c[2]).sum::<f64>() / coords.len() as f64;
    let xy = coords.iter().map(|c| [c[0], c[1]]).collect();
    Some((xy, z))
}

fn regions_containing(model: &Model, regions: &[FloorRegion], xy: [f64; 2], z: f64) -> Vec<usize> {
    let mut hits = Vec::new();
    for (i, r) in regions.iter().enumerate() {
        let Some((poly, rz)) = region_xy_poly(model, r) else {
            continue;
        };
        if (rz - z).abs() > LEVEL_TOL_MM {
            continue;
        }
        if polygon_contains_strict(&poly, xy) {
            hits.push(i);
        }
    }
    hits
}

fn assign_joists(model: &Model, regions: &mut [FloorRegion]) -> usize {
    let mut joists: Vec<_> = model
        .secondary_members
        .iter()
        .filter(|sm| sm.kind == SecondaryMemberKind::Joist)
        .collect();
    joists.sort_by_key(|sm| sm.id);
    let mut unassigned = 0;
    for sm in joists {
        let Some((xy, z)) = joist_midpoint(model, sm.nodes) else {
            unassigned += 1;
            continue;
        };
        let hits = regions_containing(model, regions, xy, z);
        if hits.len() == 1 {
            regions[hits[0]].secondary_joist_ids.push(sm.id);
        } else {
            unassigned += 1;
        }
    }
    unassigned
}

fn node_has_structural_ref(model: &Model, id: NodeId) -> bool {
    if model.elements.iter().any(|e| e.nodes.contains(&id)) {
        return true;
    }
    if model
        .secondary_members
        .iter()
        .any(|sm| sm.nodes.contains(&id))
    {
        return true;
    }
    if model.floor_regions.iter().any(|r| region_refs_node(r, id)) {
        return true;
    }
    if model.constraints.iter().any(|c| match c {
        Constraint::RigidDiaphragm { master, slaves, .. }
        | Constraint::RigidLink { master, slaves, .. } => *master == id || slaves.contains(&id),
        Constraint::Mpc { master, terms } => {
            *master == id || terms.iter().any(|(n, _, _)| *n == id)
        }
    }) {
        return true;
    }
    if model
        .load_cases
        .iter()
        .any(|lc| lc.nodal.iter().any(|nl| nl.node == id))
    {
        return true;
    }
    if let Some(node) = model.nodes.get(id.index()) {
        if node.restraint != Dof6Mask::FREE || node.mass.is_some() {
            return true;
        }
    }
    if model.generated_masters.contains(&id) {
        return true;
    }
    false
}

fn region_refs_node(region: &FloorRegion, id: NodeId) -> bool {
    let in_shape = match &region.shape {
        RegionShape::Enclosed { boundary } => boundary.contains(&id),
        RegionShape::Attached { anchor, .. } => match anchor {
            RegionAnchor::Line { nodes, .. } => nodes[0] == id || nodes[1] == id,
            RegionAnchor::Point(n) => *n == id,
        },
    };
    in_shape
        || region
            .plate
            .as_ref()
            .is_some_and(|p| p.joists.iter().any(|j| j.support.contains(&id)))
}

fn delete_unref_nodes(model: &mut Model) -> usize {
    let n = model.nodes.len();
    if n == 0 {
        return 0;
    }
    let keep: Vec<bool> = (0..n)
        .map(|i| node_has_structural_ref(model, NodeId(i as u32)))
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
    use crate::ids::{ElemId, FloorRegionId, NodeId, SecondaryMemberId, SectionId};
    use crate::model::{
        AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime,
        LoadTransfer, LocalAxis, Node, RegionAnchor, RegionShape, SecondaryMember,
        SecondaryMemberKind, SlabPlate,
    };
    use crate::region_gen::generate_floor_panels;
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
            joists: Vec::new(),
        }
    }

    fn enclosed(
        id: u32,
        boundary: Vec<u32>,
        plate: Option<SlabPlate>,
    ) -> crate::model::FloorRegion {
        let mut r = crate::model::FloorRegion::enclosed(
            FloorRegionId(id),
            boundary.into_iter().map(NodeId).collect(),
        );
        r.plate = plate;
        r
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
            id: SecondaryMemberId(id),
        }
    }

    fn has_xy(model: &Model, x: f64, y: f64) -> bool {
        model.nodes.iter().any(|n| {
            (n.coord[0] - x).abs() < 1e-6
                && (n.coord[1] - y).abs() < 1e-6
                && n.coord[2].abs() < 1e-6
        })
    }

    /// 4 辺の大梁で閉じた 1 面。上下辺の中間節点に小梁 1 本、スラブ片 2。
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
        model.floor_regions.push(enclosed(
            0,
            vec![0, 1, 4, 5],
            Some(plate(Some(sid), Vec::new())),
        ));
        model.floor_regions.push(enclosed(
            1,
            vec![1, 2, 3, 4],
            Some(plate(Some(sid), Vec::new())),
        ));
        model.secondary_members.push(joist(0, 1, 4));
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
        model.floor_regions.push(enclosed(
            0,
            vec![0, 1, 2, 3],
            Some(plate(Some(sid), Vec::new())),
        ));
        model
    }

    #[test]
    fn test_two_by_two_folds_two_plates_and_one_joist() {
        let mut model = two_piece_square();
        assert_eq!(generate_floor_panels(&model).len(), 1);
        let report = rebuild_floor_regions(&mut model);
        let enclosed: Vec<_> = model
            .floor_regions
            .iter()
            .filter(|r| matches!(r.shape, RegionShape::Enclosed { .. }))
            .collect();
        assert_eq!(enclosed.len(), 1, "囲まれは 1 面");
        assert!(enclosed[0].plate.is_some(), "版を継承する");
        assert_eq!(
            enclosed[0].secondary_joist_ids,
            vec![SecondaryMemberId(0)],
            "中央小梁が属する"
        );
        assert_eq!(enclosed[0].section(), Some(SectionId(0)), "片の断面が残る");
        assert_eq!(report.enclosed, 1);
    }

    #[test]
    fn test_rebuild_is_idempotent_on_two_piece_square() {
        let mut model = two_piece_square();
        rebuild_floor_regions(&mut model);
        let first_regions = model.floor_regions.clone();
        let first_nodes = model.nodes.len();
        rebuild_floor_regions(&mut model);
        assert_eq!(model.floor_regions, first_regions);
        assert_eq!(model.nodes.len(), first_nodes);
    }

    #[test]
    fn test_courtyard_two_internal_faces_no_outer_region() {
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
        let panels = generate_floor_panels(&model);
        assert_eq!(panels.len(), 2, "内部面は外周と中庭の 2 つ");
        let report = rebuild_floor_regions(&mut model);
        assert_eq!(
            model.floor_regions.len(),
            panels.len(),
            "領域数はパネル数と一致（外周面を領域にしない）"
        );
        assert!(
            model
                .floor_regions
                .iter()
                .all(|r| matches!(r.shape, RegionShape::Enclosed { .. })),
            "内部面はどちらも Enclosed"
        );
        let poly_area = |r: &crate::model::FloorRegion| -> f64 {
            let Some(coords) = r.boundary_coords(&model) else {
                return f64::MAX;
            };
            let n = coords.len();
            if n < 3 {
                return f64::MAX;
            }
            let mut sum = 0.0;
            for i in 0..n {
                let a = coords[i];
                let b = coords[(i + 1) % n];
                sum += a[0] * b[1] - b[0] * a[1];
            }
            (sum / 2.0).abs()
        };
        let courtyard = model
            .floor_regions
            .iter()
            .min_by(|a, b| poly_area(a).partial_cmp(&poly_area(b)).unwrap())
            .expect("中庭");
        assert!(courtyard.plate.is_none(), "中庭は版なし");
        assert_eq!(report.enclosed, panels.len());
    }

    fn courtyard_girder_loops() -> Model {
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
        model
    }

    fn region_poly_area(model: &Model, r: &crate::model::FloorRegion) -> f64 {
        let Some(coords) = r.boundary_coords(model) else {
            return f64::MAX;
        };
        let n = coords.len();
        if n < 3 {
            return f64::MAX;
        }
        let mut sum = 0.0;
        for i in 0..n {
            let a = coords[i];
            let b = coords[(i + 1) % n];
            sum += a[0] * b[1] - b[0] * a[1];
        }
        (sum / 2.0).abs()
    }

    #[test]
    fn test_courtyard_inner_plate_goes_to_smaller_panel() {
        let mut model = courtyard_girder_loops();
        let sid = push_slab_section(&mut model, 150.0);
        model.floor_regions.push(enclosed(
            0,
            vec![4, 5, 6, 7],
            Some(plate(Some(sid), Vec::new())),
        ));
        rebuild_floor_regions(&mut model);
        let enclosed: Vec<_> = model
            .floor_regions
            .iter()
            .filter(|r| matches!(r.shape, RegionShape::Enclosed { .. }))
            .collect();
        assert_eq!(enclosed.len(), 2, "領域は 2 つの Enclosed のまま");
        let (smaller, larger) =
            if region_poly_area(&model, enclosed[0]) <= region_poly_area(&model, enclosed[1]) {
                (enclosed[0], enclosed[1])
            } else {
                (enclosed[1], enclosed[0])
            };
        assert!(
            smaller.plate.is_some(),
            "中庭（面積が小さい領域）が版を持つ"
        );
        assert!(larger.plate.is_none(), "面積が大きい領域は版を持たない");
    }

    #[test]
    fn test_ring_plate_goes_to_outer_panel() {
        let mut model = courtyard_girder_loops();
        let sid = push_slab_section(&mut model, 150.0);
        model.floor_regions.push(enclosed(
            0,
            vec![0, 1, 5, 4],
            Some(plate(Some(sid), Vec::new())),
        ));
        rebuild_floor_regions(&mut model);
        let enclosed: Vec<_> = model
            .floor_regions
            .iter()
            .filter(|r| matches!(r.shape, RegionShape::Enclosed { .. }))
            .collect();
        assert_eq!(enclosed.len(), 2, "領域は 2 つの Enclosed のまま");
        let (smaller, larger) =
            if region_poly_area(&model, enclosed[0]) <= region_poly_area(&model, enclosed[1]) {
                (enclosed[0], enclosed[1])
            } else {
                (enclosed[1], enclosed[0])
            };
        assert!(larger.plate.is_some(), "面積が大きい領域が版を持つ");
        assert!(smaller.plate.is_none(), "中庭（小さい方）は版を持たない");
    }

    #[test]
    fn test_cantilever_rebuild_node_ids_are_compact() {
        let mut model = cantilever_rect();
        rebuild_floor_regions(&mut model);
        assert!(model.validate().is_ok(), "{:?}", model.validate().err());
        for (i, n) in model.nodes.iter().enumerate() {
            assert_eq!(n.id, NodeId(i as u32), "nodes[{i}].id");
        }
        let mut refs = Vec::new();
        for r in &model.floor_regions {
            match &r.shape {
                RegionShape::Enclosed { boundary } => refs.extend(boundary.iter().copied()),
                RegionShape::Attached { anchor, .. } => match anchor {
                    RegionAnchor::Line { nodes, .. } => refs.extend(nodes.iter().copied()),
                    RegionAnchor::Point(n) => refs.push(*n),
                },
            }
        }
        for e in &model.elements {
            refs.extend(e.nodes.iter().copied());
        }
        for id in refs {
            assert!(
                id.index() < model.nodes.len(),
                "参照 NodeId({id:?}) が nodes.len() 以上"
            );
            assert_eq!(
                model.nodes[id.index()].id,
                id,
                "参照 NodeId({id:?}) が nodes[index] と不一致（消した ID が 0 に化けていないこと）"
            );
        }
    }

    #[test]
    fn test_cantilever_rect_converts_to_attached_line() {
        let mut model = cantilever_rect();
        let report = rebuild_floor_regions(&mut model);
        assert_eq!(model.floor_regions.len(), 1);
        let r = &model.floor_regions[0];
        assert!(r.is_attached());
        match &r.shape {
            RegionShape::Attached {
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
        assert!(report.attached_converted >= 1);
        assert!(report.deleted_nodes >= 2);
    }

    #[test]
    fn test_cantilever_triangle_extent_equal() {
        let mut model = Model::default();
        model.nodes.push(node(0, 0.0, 0.0, 0.0));
        model.nodes.push(node(1, 4000.0, 0.0, 0.0));
        model.nodes.push(node(2, 2000.0, 1500.0, 0.0));
        model.elements.push(beam(0, 0, 1));
        let sid = push_slab_section(&mut model, 150.0);
        model.floor_regions.push(enclosed(
            0,
            vec![0, 1, 2],
            Some(plate(Some(sid), Vec::new())),
        ));
        rebuild_floor_regions(&mut model);
        let r = &model.floor_regions[0];
        assert!(r.is_attached());
        match &r.shape {
            RegionShape::Attached { extent, .. } => {
                assert!(
                    (extent[0] - extent[1]).abs() < 1e-6,
                    "頂点距離で両端が等しい: {extent:?}"
                );
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn test_inherits_name_and_finish_loads_when_boundary_nodes_change() {
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
        let mut r = enclosed(
            0,
            vec![0, 1, 2, 3],
            Some(plate(
                Some(sid),
                vec![AreaLoad {
                    kind: "仕上げ".into(),
                    value: 0.001,
                }],
            )),
        );
        r.name = "階段室".into();
        model.floor_regions.push(r);

        model.nodes.push(node(4, 2000.0, 0.0, 0.0));
        model.elements[0] = beam(0, 0, 4);
        model.elements.push(beam(4, 4, 1));
        if let RegionShape::Enclosed { boundary } = &mut model.floor_regions[0].shape {
            *boundary = vec![NodeId(0), NodeId(4), NodeId(1), NodeId(2), NodeId(3)];
        }

        rebuild_floor_regions(&mut model);
        let r = model
            .floor_regions
            .iter()
            .find(|r| matches!(r.shape, RegionShape::Enclosed { .. }))
            .expect("囲まれが残る");
        assert_eq!(r.name, "階段室");
        assert_eq!(
            r.plate.as_ref().map(|p| p.loads.as_slice()),
            Some(
                [AreaLoad {
                    kind: "仕上げ".into(),
                    value: 0.001,
                }]
                .as_slice()
            )
        );
    }

    #[test]
    fn test_split_panel_inherits_plate_only_on_centroid_side() {
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
        model.floor_regions.push(enclosed(
            0,
            vec![0, 1, 2, 3],
            Some(plate(Some(sid), Vec::new())),
        ));

        model.nodes.push(node(4, 1000.0, 0.0, 0.0));
        model.nodes.push(node(5, 1000.0, 4000.0, 0.0));
        model.elements[0] = beam(0, 0, 4);
        model.elements.push(beam(4, 4, 1));
        model.elements[2] = beam(2, 2, 5);
        model.elements.push(beam(5, 5, 3));
        model.elements.push(beam(6, 4, 5));

        let report = rebuild_floor_regions(&mut model);
        let panels = generate_floor_panels(&model);
        assert_eq!(panels.len(), 2);
        assert_eq!(model.floor_regions.len(), 2);
        let old_centroid = [2000.0, 2000.0];
        let with_plate = model
            .floor_regions
            .iter()
            .filter(|r| r.plate.is_some())
            .count();
        assert_eq!(with_plate, 1, "重心が入る側だけ版を継承");
        let inherited = model
            .floor_regions
            .iter()
            .find(|r| r.plate.is_some())
            .unwrap();
        let coords = inherited.boundary_coords(&model).unwrap();
        let n = coords.len() as f64;
        let c = [
            coords.iter().map(|p| p[0]).sum::<f64>() / n,
            coords.iter().map(|p| p[1]).sum::<f64>() / n,
        ];
        assert!(
            (c[0] - old_centroid[0]).abs() < 1500.0,
            "継承側の重心が旧重心側: {c:?}"
        );
        let inherit_panel = panels
            .iter()
            .find(|p| p.contains(&model, old_centroid))
            .expect("旧重心が入るパネル");
        assert!(
            inherit_panel.contains(&model, c),
            "版つき領域は旧重心側のパネル"
        );
        assert!(report.unmatched_old_enclosed >= 1);
    }

    #[test]
    fn test_joist_outside_building_is_unassigned() {
        let mut model = two_piece_square();
        model.nodes.push(node(6, 10000.0, 0.0, 0.0));
        model.nodes.push(node(7, 10000.0, 4000.0, 0.0));
        model.secondary_members.push(joist(1, 6, 7));
        let report = rebuild_floor_regions(&mut model);
        assert_eq!(model.secondary_members.len(), 2, "所属なし小梁も残る");
        assert!(
            model
                .floor_regions
                .iter()
                .all(|r| !r.secondary_joist_ids.contains(&SecondaryMemberId(1))),
            "どの領域の secondary_joist_ids にも入らない"
        );
        assert_eq!(report.unassigned_joists, 1);
    }

    #[test]
    fn test_mixed_plate_sections_drop_plate() {
        let mut model = Model::default();
        for (i, (x, y)) in [(0.0, 0.0), (4000.0, 0.0), (4000.0, 4000.0), (0.0, 4000.0)]
            .into_iter()
            .enumerate()
        {
            model.nodes.push(node(i as u32, x, y, 0.0));
        }
        model.nodes.push(node(4, 2000.0, 0.0, 0.0));
        model.nodes.push(node(5, 2000.0, 4000.0, 0.0));
        model.elements.extend([
            beam(0, 0, 4),
            beam(1, 4, 1),
            beam(2, 1, 2),
            beam(3, 2, 5),
            beam(4, 5, 3),
            beam(5, 3, 0),
        ]);
        let s1 = push_slab_section(&mut model, 150.0);
        let s2 = push_slab_section(&mut model, 200.0);
        model.floor_regions.push(enclosed(
            0,
            vec![0, 4, 5, 3],
            Some(plate(Some(s1), Vec::new())),
        ));
        model.floor_regions.push(enclosed(
            1,
            vec![4, 1, 2, 5],
            Some(plate(Some(s2), Vec::new())),
        ));
        let report = rebuild_floor_regions(&mut model);
        assert_eq!(model.floor_regions.len(), 1);
        assert!(model.floor_regions[0].plate.is_none());
        assert_eq!(report.mixed_plate_panels, 1);
    }

    #[test]
    fn test_floating_plate_stays_enclosed_and_unassigned() {
        let mut model = Model::default();
        for (i, (x, y)) in [(0.0, 0.0), (4000.0, 0.0), (4000.0, 4000.0), (0.0, 4000.0)]
            .into_iter()
            .enumerate()
        {
            model.nodes.push(node(i as u32, x, y, 0.0));
        }
        let sid = push_slab_section(&mut model, 150.0);
        model.floor_regions.push(enclosed(
            0,
            vec![0, 1, 2, 3],
            Some(plate(Some(sid), Vec::new())),
        ));
        let n_before = model.floor_regions.len();
        let report = rebuild_floor_regions(&mut model);
        assert!(model.floor_regions.len() >= n_before, "領域数は減らない");
        assert_eq!(report.unassigned_plates, 1);
        assert!(
            model
                .floor_regions
                .iter()
                .any(|r| matches!(r.shape, RegionShape::Enclosed { .. }) && r.plate.is_some()),
            "版は Enclosed のまま残る"
        );
    }

    #[test]
    fn test_plate_on_two_girder_edges_stays_enclosed() {
        let mut model = Model::default();
        model.nodes.push(node(0, 0.0, 0.0, 0.0));
        model.nodes.push(node(1, 4000.0, 0.0, 0.0));
        model.nodes.push(node(2, 4000.0, 4000.0, 0.0));
        model.nodes.push(node(3, 0.0, 4000.0, 0.0));
        model.elements.push(beam(0, 0, 1));
        model.elements.push(beam(1, 0, 3));
        let sid = push_slab_section(&mut model, 150.0);
        model.floor_regions.push(enclosed(
            0,
            vec![0, 1, 2, 3],
            Some(plate(Some(sid), Vec::new())),
        ));
        let report = rebuild_floor_regions(&mut model);
        assert_eq!(model.floor_regions.len(), 1);
        assert!(
            matches!(model.floor_regions[0].shape, RegionShape::Enclosed { .. }),
            "出隅相当は Enclosed のまま"
        );
        assert_eq!(report.attached_converted, 0);
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
        assert!(report.attached_converted >= 1);
    }
}
