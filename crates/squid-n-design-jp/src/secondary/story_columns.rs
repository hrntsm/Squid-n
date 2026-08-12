//! 層に帰属する柱の列挙（中間節点分割を連なりとして 1 本とみなす）。
//!
//! 層間変位・偏心率など、当該層の柱を数える処理の判定ロジックの単一情報源。

use squid_n_core::geom::is_vertical_axis;
use squid_n_core::ids::{ElemId, NodeId, StoryId};
use squid_n_core::model::DIAPHRAGM_LEVEL_TOL_MM;
use squid_n_core::model::{ElementKind, Model};
use std::collections::{HashMap, HashSet};

/// 層に帰属する 1 本の柱（中間節点で分割された鉛直材の連なりを束ねたもの）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StoryColumn {
    /// 最上側セグメントの部材 ID（`ColumnDrift.elem` 等の代表 ID）。
    pub top_elem: ElemId,
    pub top: NodeId,
    pub bottom: NodeId,
}

/// 層の下端。床レベルの階 ID、または基部標高（階が 1 つだけの略式モデル用）。
#[derive(Clone, Copy, Debug)]
enum BottomLevel {
    Story(StoryId),
    Elevation(f64),
}

/// `story`（層の上端階）に属する柱の一覧を返す。
///
/// 上端階の床レベル上の節点から鉛直材を下へ辿り、下端階の床レベル（または基部標高）
/// に達した連なりを 1 本の柱とする。下端に達しない連なりは対象外。
pub fn story_columns(model: &Model, story: StoryId) -> Vec<StoryColumn> {
    let Some((_, bottom)) = layer_bounds(model, story) else {
        return Vec::new();
    };
    let adj = vertical_beam_adjacency(model);
    let mut out = Vec::new();
    for node in &model.nodes {
        if !model.on_diaphragm_level(story, node.coord[2]) {
            continue;
        }
        if let Some(col) = trace_column_down(model, &adj, node.id, bottom) {
            out.push(col);
        }
    }
    out
}

/// 層の上端階と下端（床レベルまたは基部標高）を返す。
fn layer_bounds(model: &Model, top_story: StoryId) -> Option<(StoryId, BottomLevel)> {
    if let Some(layer) = model.layers().into_iter().find(|l| l.top == top_story) {
        return Some((top_story, BottomLevel::Story(layer.bottom)));
    }
    // 階が 1 つだけの略式モデル（テスト用）: 基部標高を下端とする。
    if model.stories.iter().any(|s| s.id == top_story) && model.layers().is_empty() {
        return Some((top_story, BottomLevel::Elevation(model.base_elevation())));
    }
    None
}

fn on_bottom_level(model: &Model, bottom: BottomLevel, z: f64) -> bool {
    match bottom {
        BottomLevel::Story(s) => model.on_diaphragm_level(s, z),
        BottomLevel::Elevation(e) => (z - e).abs() <= DIAPHRAGM_LEVEL_TOL_MM,
    }
}

/// 鉛直 2 節点 Beam の隣接リスト（節点 → (隣接節点, 部材 ID)）。
fn vertical_beam_adjacency(model: &Model) -> HashMap<NodeId, Vec<(NodeId, ElemId)>> {
    let mut adj: HashMap<NodeId, Vec<(NodeId, ElemId)>> = HashMap::new();
    for elem in &model.elements {
        if elem.kind != ElementKind::Beam || elem.nodes.len() != 2 {
            continue;
        }
        let n0_id = elem.nodes[0];
        let n1_id = elem.nodes[1];
        let n0 = &model.nodes[n0_id.index()];
        let n1 = &model.nodes[n1_id.index()];
        if !is_vertical_axis(n0.coord, n1.coord) {
            continue;
        }
        adj.entry(n0_id).or_default().push((n1_id, elem.id));
        adj.entry(n1_id).or_default().push((n0_id, elem.id));
    }
    adj
}

/// 上端節点から鉛直材を下へ辿り、層下端に達する連なりを返す。
fn trace_column_down(
    model: &Model,
    adj: &HashMap<NodeId, Vec<(NodeId, ElemId)>>,
    top: NodeId,
    bottom: BottomLevel,
) -> Option<StoryColumn> {
    let mut current = top;
    let mut top_elem = None;
    let mut visited = HashSet::new();
    visited.insert(top);

    loop {
        let cur_node = &model.nodes[current.index()];
        if current != top && on_bottom_level(model, bottom, cur_node.coord[2]) {
            return Some(StoryColumn {
                top_elem: top_elem?,
                top,
                bottom: current,
            });
        }

        let cur_z = cur_node.coord[2];
        let neighbors: Vec<_> = adj
            .get(&current)?
            .iter()
            .filter(|(nid, _)| model.nodes[nid.index()].coord[2] < cur_z - 1e-9)
            .copied()
            .collect();

        if neighbors.len() != 1 {
            return None;
        }
        let (next, elem) = neighbors[0];
        if !visited.insert(next) {
            return None;
        }
        if top_elem.is_none() {
            top_elem = Some(elem);
        }
        current = next;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use smallvec::SmallVec;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, EndCondition, ForceRegime, LocalAxis, Node, RigidZone, Story,
    };

    /// 中間節点（z=2000）で 2 分割された 1 本柱（z=0, 2000, 4000）。
    fn build_split_column_model() -> (Model, StoryId) {
        let base = StoryId(0);
        let top_story = StoryId(1);
        let nodes = vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: Some(base),
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 2000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(top_story),
                support_spring: None,
            },
            Node {
                id: NodeId(2),
                coord: [0.0, 0.0, 4000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(top_story),
                support_spring: None,
            },
        ];
        let mk_elem = |id: u32, n0: u32, n1: u32| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: {
                let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
                v.push(NodeId(n0));
                v.push(NodeId(n1));
                v
            },
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        };
        let elements = vec![mk_elem(0, 1, 2), mk_elem(1, 0, 1)];
        let model = Model {
            nodes,
            elements,
            stories: vec![
                Story {
                    id: base,
                    name: "1F".to_string(),
                    elevation: 0.0,
                    node_ids: vec![NodeId(0)],
                    seismic_weight: None,
                    weight_override: None,
                    structure: Default::default(),
                    level_kind: Default::default(),
                },
                Story {
                    id: top_story,
                    name: "2F".to_string(),
                    elevation: 4000.0,
                    node_ids: vec![NodeId(2)],
                    seismic_weight: None,
                    weight_override: None,
                    structure: Default::default(),
                    level_kind: Default::default(),
                },
            ],
            ..Default::default()
        };
        (model, top_story)
    }

    #[test]
    fn test_story_columns_merges_split_segments() {
        let (model, top) = build_split_column_model();
        let cols = story_columns(&model, top);
        assert_eq!(cols.len(), 1, "分割柱は 1 本と数える");
        assert_eq!(cols[0].top, NodeId(2));
        assert_eq!(cols[0].bottom, NodeId(0));
        assert_eq!(cols[0].top_elem, ElemId(0), "代表 ID は最上側セグメント");
    }
}
