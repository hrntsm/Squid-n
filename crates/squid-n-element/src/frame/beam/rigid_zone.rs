//! 剛域の自動算定。

use squid_n_core::adjacency::NodeAdjacency;
use squid_n_core::model::{Model, RigidZone, ZoneSource};
use squid_n_core::structure_kind::member_structure_kind;

pub struct RigidZoneRule {
    /// 取り付く壁を考慮するか。
    pub consider_walls: bool,
}

impl Default for RigidZoneRule {
    fn default() -> Self {
        Self {
            consider_walls: true,
        }
    }
}

/// 部材に取り付く壁が、部材フェースから張り出す長さ [mm]。
///
/// `toward` を与えると、その向きへ張り出す壁だけを対象にする。
/// `None` なら向きを問わず最大を採る。
fn wall_protrusion(
    model: &Model,
    elem: &squid_n_core::model::ElementData,
    depth: f64,
    walls: &[crate::wall::misc_wall::InFrameMiscWallGeometry],
    toward: Option<[f64; 3]>,
) -> f64 {
    if walls.is_empty() || elem.nodes.len() < 2 {
        return 0.0;
    }
    let (n0, n1) = (elem.nodes[0], elem.nodes[elem.nodes.len() - 1]);
    let same_pair = |a: squid_n_core::ids::NodeId, b: squid_n_core::ids::NodeId| -> bool {
        (n0 == a && n1 == b) || (n0 == b && n1 == a)
    };
    let accepts = |dir: [f64; 3]| -> bool {
        match toward {
            None => true,
            Some(t) => squid_n_core::geom::vec3::dot(dir, t) > 1e-9,
        }
    };

    let axis = elem_axis(model, elem);
    let is_vertical = axis[2].abs() > squid_n_core::geom::VERTICAL_COS_TOL;
    let mut out = 0.0_f64;

    for w in walls {
        if is_vertical {
            let (Some(bottom), Some(top)) = (w.bottom_pair, w.top_pair) else {
                continue;
            };
            for s in 0..2 {
                if !same_pair(bottom[s], top[s]) {
                    continue;
                }
                let e_wall = w.bottom_dir;
                let sign = if s == 0 { 1.0 } else { -1.0 };
                let dir = [e_wall[0] * sign, e_wall[1] * sign, 0.0];
                if !accepts(dir) {
                    continue;
                }
                out = out.max((w.wing_length(s) - depth / 2.0).clamp(0.0, w.lw));
            }
        } else {
            for (matched, extent, dir) in [
                (
                    w.bottom_pair.is_some_and(|p| same_pair(p[0], p[1])),
                    w.strip_height(false),
                    [0.0, 0.0, 1.0],
                ),
                (
                    w.top_pair.is_some_and(|p| same_pair(p[0], p[1])),
                    w.strip_height(true),
                    [0.0, 0.0, -1.0],
                ),
            ] {
                if !matched || !accepts(dir) {
                    continue;
                }
                out = out.max((extent - depth / 2.0).clamp(0.0, w.h));
            }
        }
    }
    out
}

/// 壁を考慮した部材せい D [mm]。
fn depth_with_walls(
    model: &Model,
    elem: &squid_n_core::model::ElementData,
    walls: &[crate::wall::misc_wall::InFrameMiscWallGeometry],
) -> f64 {
    let depth = elem
        .section
        .and_then(|sid| model.sections.get(sid.index()))
        .map(|s| s.depth)
        .unwrap_or(0.0);
    depth + wall_protrusion(model, elem, depth, walls, None)
}

use squid_n_core::geom::element_axis as elem_axis;

/// 節点 `node` から部材フェースまでの距離 Lf [mm]。
///
/// 対象部材と概ね直交する柱・大梁の最大せいの半分に、その部材へ取り付く壁の
/// 張り出しを加えた値。直交材として数えるのは `ElementKind::Beam` のみ。
fn max_orth_face(
    model: &Model,
    node: squid_n_core::ids::NodeId,
    target_axis: [f64; 3],
    target_elem_idx: usize,
    adjacency: &NodeAdjacency,
    walls: &[crate::wall::misc_wall::InFrameMiscWallGeometry],
    toward: [f64; 3],
) -> f64 {
    let mut lf_max = 0.0_f64;
    for &ei in adjacency.indices_at(node) {
        if ei == target_elem_idx {
            continue;
        }
        let e = &model.elements[ei];
        if e.kind != squid_n_core::model::ElementKind::Beam {
            continue;
        }
        let axis = elem_axis(model, e);
        if squid_n_core::geom::vec3::dot(axis, target_axis).abs()
            >= squid_n_core::geom::ORTHOGONAL_DOT_MAX
        {
            continue;
        }
        let Some(sec) = e.section.and_then(|sid| model.sections.get(sid.index())) else {
            continue;
        };
        let lf = sec.depth / 2.0 + wall_protrusion(model, e, sec.depth, walls, Some(toward));
        lf_max = lf_max.max(lf);
    }
    lf_max
}

/// 節点 `node` に集合する柱・大梁がすべて RC/SRC 系か。
///
/// 柱・大梁以外と二次部材は判定の対象外。
fn all_rc_src_at(
    model: &Model,
    node: squid_n_core::ids::NodeId,
    adjacency: &NodeAdjacency,
) -> bool {
    adjacency.indices_at(node).iter().all(|&ei| {
        let e = &model.elements[ei];
        e.kind != squid_n_core::model::ElementKind::Beam
            || !member_structure_kind(model, e).is_steel_like()
    })
}

/// `target_elem_idx` は `model.elements` 内の対象要素の添字。
fn rigid_zone_with_adjacency(
    model: &Model,
    target_elem_idx: usize,
    adjacency: &NodeAdjacency,
    rule: &RigidZoneRule,
    walls: &[crate::wall::misc_wall::InFrameMiscWallGeometry],
    face: [f64; 2],
) -> RigidZone {
    let elem = &model.elements[target_elem_idx];
    let nodes = &elem.nodes;
    if nodes.len() < 2 {
        return RigidZone::default();
    }

    let target_axis = elem_axis(model, elem);

    let node_i = nodes[0];
    let node_j = nodes[nodes.len() - 1];
    let (ci, cj) = (
        model.nodes[node_i.index()].coord,
        model.nodes[node_j.index()].coord,
    );

    let d_self = if rule.consider_walls {
        depth_with_walls(model, elem, walls)
    } else {
        elem.section
            .and_then(|sid| model.sections.get(sid.index()))
            .map(|s| s.depth)
            .unwrap_or(0.0)
    };

    let dir_i = [cj[0] - ci[0], cj[1] - ci[1], cj[2] - ci[2]];
    let dir_j = [-dir_i[0], -dir_i[1], -dir_i[2]];
    let lf = |node, toward, walls: &[crate::wall::misc_wall::InFrameMiscWallGeometry]| {
        max_orth_face(
            model,
            node,
            target_axis,
            target_elem_idx,
            adjacency,
            walls,
            toward,
        )
    };
    let lf_i = lf(node_i, dir_i, walls);
    let lf_j = lf(node_j, dir_j, walls);
    let [face_i, face_j] = face;

    let lambda = |lf: f64, all_rc_src: bool| -> f64 {
        if !all_rc_src {
            return 0.0;
        }
        (lf - d_self / 4.0).max(0.0)
    };

    let (mut length_i, mut length_j) = (
        lambda(lf_i, all_rc_src_at(model, node_i, adjacency)),
        lambda(lf_j, all_rc_src_at(model, node_j, adjacency)),
    );

    let len = ((cj[0] - ci[0]).powi(2) + (cj[1] - ci[1]).powi(2) + (cj[2] - ci[2]).powi(2)).sqrt();
    if len > 0.0 && length_i + length_j >= len {
        let half = (len / 2.0 - d_self / 4.0).max(0.0);
        length_i = half;
        length_j = half;
    }

    RigidZone {
        length_i,
        length_j,
        source_i: ZoneSource::Auto,
        source_j: ZoneSource::Auto,
        face_i: Some(face_i),
        face_j: Some(face_j),
        panel_offset_i: 0.0,
        panel_offset_j: 0.0,
    }
}

pub fn auto_rigid_zones(
    model: &squid_n_core::model::Model,
    elem_id: squid_n_core::ids::ElemId,
    rule: &RigidZoneRule,
) -> RigidZone {
    let Some(target_elem_idx) = model.elements.iter().position(|e| e.id == elem_id) else {
        return RigidZone::default();
    };
    let adjacency = NodeAdjacency::build(model);
    let walls = if rule.consider_walls {
        crate::wall::misc_wall::collect_rigid_zone_walls(model)
    } else {
        Vec::new()
    };
    let face = squid_n_core::face_distance::face_distances(model)[target_elem_idx];
    rigid_zone_with_adjacency(model, target_elem_idx, &adjacency, rule, &walls, face)
}

pub fn recompute_auto_zones(zone: &mut RigidZone, recomputed: &RigidZone) {
    if matches!(zone.source_i, ZoneSource::Auto) {
        zone.length_i = recomputed.length_i;
    }
    if matches!(zone.source_j, ZoneSource::Auto) {
        zone.length_j = recomputed.length_j;
    }

    zone.face_i = recomputed.face_i;
    zone.face_j = recomputed.face_j;
}

/// モデル全要素の剛域を自動算定し、`ElementData::rigid_zone` を更新する。
/// `source` が `Auto` の端のみ更新し、`Manual` 端は保護する。
pub fn apply_auto_rigid_zones(model: &mut Model, rule: &RigidZoneRule) {
    let adjacency = NodeAdjacency::build(model);
    let walls = if rule.consider_walls {
        crate::wall::misc_wall::collect_rigid_zone_walls(model)
    } else {
        Vec::new()
    };
    let faces = squid_n_core::face_distance::face_distances(model);
    let recomputed: Vec<(usize, RigidZone)> = model
        .elements
        .iter()
        .enumerate()
        .filter(|(_, e)| matches!(e.kind, squid_n_core::model::ElementKind::Beam))
        .map(|(i, _)| {
            (
                i,
                rigid_zone_with_adjacency(model, i, &adjacency, rule, &walls, faces[i]),
            )
        })
        .collect();

    for (i, rz) in recomputed {
        recompute_auto_zones(&mut model.elements[i].rigid_zone, &rz);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wall::misc_wall::InFrameMiscWallGeometry;
    use squid_n_core::ids::{ElemId, NodeId, SectionId, WallPlateId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node,
    };

    fn node(id: u32, coord: [f64; 3]) -> Node {
        Node {
            id: NodeId(id),
            coord,
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    /// 左辺（節点 0-3）の柱と、それに取り付く壁 1 枚を持つモデル。
    fn column_with_wall() -> (Model, ElementData) {
        let model = Model {
            nodes: vec![
                node(0, [0.0, 0.0, 0.0]),
                node(1, [4000.0, 0.0, 0.0]),
                node(2, [4000.0, 0.0, 3000.0]),
                node(3, [0.0, 0.0, 3000.0]),
            ],
            ..Default::default()
        };
        let column = ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        (model, column)
    }

    fn wall_geometry(column_face: [bool; 2]) -> InFrameMiscWallGeometry {
        InFrameMiscWallGeometry {
            t: 150.0,
            lw: 4000.0,
            h: 3000.0,
            bottom_pair: Some([NodeId(0), NodeId(1)]),
            top_pair: Some([NodeId(3), NodeId(2)]),
            bottom_dir: [1.0, 0.0, 0.0],
            envelope: None,
            plate: WallPlateId(0),
            slit: squid_n_core::model::WallSlit {
                column_face,
                beam_face: [false, false],
            },
        }
    }

    /// 梁際が切れている辺の梁には、腰壁・垂れ壁による剛域の張り出しを与えない。
    ///
    /// 切れた辺の高さは 0 になるので（`InFrameMiscWallGeometry::strip_height`）、
    /// 剛域も伸びない。反対側の梁は全高を負担するため、折半（h/2）ではなく h から
    /// 部材せいの半分を引いた値まで伸びる。
    #[test]
    fn 梁際スリットのある側は剛域の腰壁張り出しを持たない() {
        let (model, _column) = column_with_wall();
        let beam = ElementData {
            id: ElemId(1),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        let depth = 600.0;

        let with_slit = |beam_face: [bool; 2]| {
            let mut w = wall_geometry([false, false]);
            w.slit.beam_face = beam_face;
            wall_protrusion(&model, &beam, depth, &[w], None)
        };

        assert!((with_slit([false, false]) - 1200.0).abs() < 1e-9);
        assert!(with_slit([true, false]).abs() < 1e-9);
        assert!((with_slit([false, true]) - 2700.0).abs() < 1e-9);
    }

    /// 柱際が切れている側の柱には、袖壁による剛域の張り出しを与えない。
    #[test]
    fn 柱際スリットのある側は剛域の袖壁張り出しを持たない() {
        let (model, column) = column_with_wall();
        let depth = 300.0;

        let walls = [wall_geometry([false, false])];
        let plain = wall_protrusion(&model, &column, depth, &walls, None);
        assert!((plain - 1850.0).abs() < 1e-9, "{plain}");

        let walls = [wall_geometry([true, false])];
        let slit = wall_protrusion(&model, &column, depth, &walls, None);
        assert!((slit - 0.0).abs() < 1e-9, "{slit}");

        let walls = [wall_geometry([false, true])];
        let other = wall_protrusion(&model, &column, depth, &walls, None);
        assert!((other - 1850.0).abs() < 1e-9, "{other}");
    }
}
