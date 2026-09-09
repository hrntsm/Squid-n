//! 部材のねじり剛性のモデル化（i 端ねじれ解放）。
//!
//! 線材の i 端のねじれ回転を解放する。解放すると特異になる節点がある部材は解放しない。

use squid_n_core::dof::Dof;
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{BeamTorsionMode, ElementData, ElementKind, Model};

/// i 端ねじれを解放しない理由。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TorsionReleaseSkip {
    /// 建物一律の設定が `BeamTorsionMode::Keep`。
    ModeKeep,
    /// 線材ではない。
    NotLineMember,
    /// 材軸が定まらない（2 節点未満・退化長さ）。
    DegenerateAxis,
    /// 材軸まわり回転を拘束するものがない節点がある。
    /// `node` はその節点。
    UnrestrainedRotation { node: NodeId },
}

/// 線材か。
fn is_line_member(kind: ElementKind) -> bool {
    matches!(
        kind,
        ElementKind::Beam
            | ElementKind::Fiber
            | ElementKind::MultiSpring
            | ElementKind::Brace { .. }
    )
}

/// 要素の材軸単位ベクトル。2 節点未満・退化長さは None。
fn axis_of(data: &ElementData, model: &Model) -> Option<[f64; 3]> {
    if data.nodes.len() < 2 {
        return None;
    }
    let p0 = model.nodes.get(data.nodes[0].index())?.coord;
    let p1 = model.nodes.get(data.nodes[1].index())?.coord;
    squid_n_core::geom::vec3::unit_from(p0, p1)
}

/// 単位ベクトル 2 本が平行か。
fn is_parallel(a: [f64; 3], b: [f64; 3]) -> bool {
    squid_n_core::geom::vec3::dot(a, b).abs() > 1.0 - 1e-6
}

/// 節点 `node` において、軸 `axis` まわりの回転が部材 `elem` のねじり以外で拘束されるか。
fn rotation_restrained_elsewhere(
    model: &Model,
    node: NodeId,
    axis: [f64; 3],
    elem: ElemId,
) -> bool {
    if let Some(n) = model.nodes.get(node.index()) {
        let rot_dofs = [Dof::Rx, Dof::Ry, Dof::Rz];
        let needed: Vec<Dof> = rot_dofs
            .iter()
            .enumerate()
            .filter(|(i, _)| axis[*i].abs() > 1e-9)
            .map(|(_, d)| *d)
            .collect();
        if !needed.is_empty() && needed.iter().all(|d| n.restraint.is_fixed(*d)) {
            return true;
        }
        if let Some(k) = n.support_spring {
            let ok = (0..3)
                .filter(|i| axis[*i].abs() > 1e-9)
                .all(|i| k[3 + i] > 0.0);
            if ok {
                return true;
            }
        }
    }
    for other in &model.elements {
        if other.id == elem || !other.nodes.contains(&node) {
            continue;
        }
        if matches!(other.kind, ElementKind::PanelZone) {
            continue;
        }
        if !is_line_member(other.kind) {
            return true;
        }
        match axis_of(other, model) {
            Some(a) if !is_parallel(a, axis) => return true,
            _ => {}
        }
    }
    false
}

/// 部材 `data` の i 端ねじれを解放しない理由（解放してよければ `None`）。
pub fn i_end_torsion_release_skip(data: &ElementData, model: &Model) -> Option<TorsionReleaseSkip> {
    if model.beam_torsion != BeamTorsionMode::ReleaseIEnd {
        return Some(TorsionReleaseSkip::ModeKeep);
    }
    if !is_line_member(data.kind) {
        return Some(TorsionReleaseSkip::NotLineMember);
    }
    let Some(axis) = axis_of(data, model) else {
        return Some(TorsionReleaseSkip::DegenerateAxis);
    };
    for node in data.nodes.iter().take(2) {
        if !rotation_restrained_elsewhere(model, *node, axis, data.id) {
            return Some(TorsionReleaseSkip::UnrestrainedRotation { node: *node });
        }
    }
    None
}

/// 部材 `data` の i 端ねじれを解放してよいか。
pub fn i_end_torsion_release(data: &ElementData, model: &Model) -> bool {
    i_end_torsion_release_skip(data, model).is_none()
}
