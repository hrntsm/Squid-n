//! 崩壊機構の判定。
//!
//! - [`compute_static_indeterminacy`] — 平面骨組の静的不静定次数
//! - [`determine_mechanism`] — 降伏ヒンジ分布から崩壊機構種別を分類

use super::types::{HingeEvent, HingeLevel, MechanismType};
use crate::statics::analysis::SeismicDir;
use squid_n_core::model::Model;

/// ヒンジの層への帰属。
enum HingeLayer {
    /// 第 `.0` 層（[`squid_n_core::model::Layer::index`]）のヒンジ。
    In(usize),
    /// どの層にも属さない部材のヒンジ。基部の階に収まる部材（基礎梁）が該当する。
    /// 層の分布には数えないが、**判定を妨げもしない**。
    OutsideLayers,
    /// 所属階が分からないヒンジ（節点の所属階が未設定）。層の分布が信用できない
    /// ため、層崩壊の判定を保留させる。
    Unknown,
}

/// 部材端ヒンジが属する**層**を返す。
///
/// 層は上端の階で識別し、高い側の所属階を採る。
///
/// 高い側も基部の階なら、その部材は基部の階に収まる基礎梁である。
fn hinge_layer(model: &Model, h: &HingeEvent) -> HingeLayer {
    let Some(elem) = model.element(h.elem) else {
        return HingeLayer::Unknown;
    };
    if elem.nodes.len() < 2 {
        return HingeLayer::Unknown;
    }
    let (near, far) = if h.pos < 0.5 {
        (elem.nodes[0], elem.nodes[1])
    } else {
        (elem.nodes[1], elem.nodes[0])
    };
    let story_of =
        |id: squid_n_core::ids::NodeId| model.nodes.get(id.index()).and_then(|n| n.story);
    let z_of = |id: squid_n_core::ids::NodeId| {
        model
            .nodes
            .get(id.index())
            .map(|n| n.coord[2])
            .unwrap_or(f64::NEG_INFINITY)
    };
    let top_side = if z_of(far) > z_of(near) { far } else { near };
    let Some(sid) = story_of(top_side)
        .or_else(|| story_of(near))
        .or_else(|| story_of(far))
    else {
        return HingeLayer::Unknown;
    };
    match sid.index().checked_sub(1) {
        Some(i) => HingeLayer::In(i),
        None => HingeLayer::OutsideLayers,
    }
}

/// 平面骨組の静的不静定次数 r = 3m − 3n + r_support を算出する。
pub(crate) fn compute_static_indeterminacy(model: &Model, dir: SeismicDir) -> usize {
    use squid_n_core::model::ElementKind;
    let (plane_bits, orth_axis): ([u8; 3], usize) = match dir {
        SeismicDir::X => ([0, 2, 4], 1),
        SeismicDir::Y => ([1, 2, 3], 0),
    };
    let mut counted = vec![false; model.nodes.len()];
    let mut m = 0usize;
    for e in &model.elements {
        let is_line = matches!(
            e.kind,
            ElementKind::Beam
                | ElementKind::Fiber
                | ElementKind::MultiSpring
                | ElementKind::Brace { .. }
        );
        if !is_line || e.nodes.len() != 2 {
            continue;
        }
        let (i0, i1) = (e.nodes[0].index(), e.nodes[1].index());
        let (Some(n0), Some(n1)) = (model.nodes.get(i0), model.nodes.get(i1)) else {
            continue;
        };
        let dir = squid_n_core::geom::vec3::unit_from(n0.coord, n1.coord);
        if dir.is_some_and(|d| squid_n_core::geom::axis_dominates(d, orth_axis)) {
            continue;
        }
        m += 1;
        counted[i0] = true;
        counted[i1] = true;
    }
    let n = counted.iter().filter(|&&c| c).count();
    let r_support: usize = model
        .nodes
        .iter()
        .enumerate()
        .filter(|(i, _)| counted[*i])
        .map(|(_, node)| {
            let bits = node.restraint.0;
            plane_bits
                .iter()
                .filter(|&&b| bits & (1u8 << b) != 0)
                .count()
        })
        .sum();
    (3 * m + r_support).saturating_sub(3 * n)
}

/// 崩壊機構の判定。
pub(crate) fn determine_mechanism(
    hinges: &[HingeEvent],
    model: &Model,
    dir: SeismicDir,
) -> MechanismType {
    use std::collections::{BTreeMap, BTreeSet};

    let yielded: Vec<&HingeEvent> = hinges
        .iter()
        .filter(|h| matches!(h.level, HingeLevel::Yield | HingeLevel::Ultimate))
        .collect();

    let distinct_ends: BTreeSet<(u32, u8)> = yielded
        .iter()
        .map(|h| (h.elem.index() as u32, if h.pos < 0.5 { 0u8 } else { 1u8 }))
        .collect();
    let r = compute_static_indeterminacy(model, dir);
    if yielded.is_empty() || distinct_ends.len() < r + 1 {
        return MechanismType::Partial;
    }

    let mut per_story: BTreeMap<usize, usize> = BTreeMap::new();
    let mut unmapped = 0usize;
    for h in &yielded {
        match hinge_layer(model, h) {
            HingeLayer::In(l) => *per_story.entry(l).or_default() += 1,
            HingeLayer::OutsideLayers => {}
            HingeLayer::Unknown => unmapped += 1,
        }
    }

    if model.layer_count() > 1 && per_story.len() == 1 && unmapped == 0 {
        MechanismType::StoryCollapse {
            layer: *per_story.keys().next().unwrap(),
        }
    } else {
        MechanismType::Overall
    }
}
