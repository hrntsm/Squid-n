//! 節点 → 接続する線材要素の隣接関係。
//!
//! 剛域の自動算定・仕口パネルの生成・座屈長さ係数の剛度比・モデル化図の描画は、
//! いずれも「この節点にどの線材が取り付くか」を繰り返し引く。節点ごとに全要素を
//! 走査すると `O(節点数 × 要素数)` になるため、隣接関係を 1 回だけ構築して共有する。
//!
//! # 分類前の素の隣接関係だけを持つ
//!
//! 用途ごとに必要な分類が異なるため、本モジュールは**分類しない**。
//!
//! - 剛域は「概ね直交する部材」（材軸の内積で判定）
//! - 剛度比 `G` は「柱・梁」（材軸の鉛直成分で判定）
//! - 仕口パネルは「柱・はり・斜材」（同上、ただし斜材を独立に扱う）
//!
//! 分類済みで持つと、どの分類軸で持つかを 1 つに決めねばならず、結局どこかが
//! 自前で再分類する。共有できるのは構築コストなので、そこだけを共有する。
//!
//! # 対象は線材のみ
//!
//! [`ElementKind::Beam`] の 2 節点要素だけを収める。耐震壁・シェル等が混ざると、
//! 剛域の直交材探索へ壁の名目せいが紛れ込む（「耐震壁周辺の柱・梁の剛域は
//! 考慮しない」という方針に反する）。

use crate::ids::NodeId;
use crate::model::{ElementData, ElementKind, Model};
use std::collections::HashMap;

/// 節点 → その節点に接続する線材要素の添字。
#[derive(Clone, Debug, Default)]
pub struct NodeAdjacency {
    by_node: HashMap<usize, Vec<usize>>,
}

impl NodeAdjacency {
    /// モデル全体から 1 回だけ構築する（`O(要素数)`）。
    pub fn build(model: &Model) -> Self {
        let mut by_node: HashMap<usize, Vec<usize>> = HashMap::new();
        for (ei, e) in model.elements.iter().enumerate() {
            if !matches!(e.kind, ElementKind::Beam) || e.nodes.len() < 2 {
                continue;
            }
            // 中間節点を持つ要素でも、隣接するのは両端だけとする。
            for n in e.nodes.iter().take(2) {
                let list = by_node.entry(n.index()).or_default();
                if !list.contains(&ei) {
                    list.push(ei);
                }
            }
        }
        Self { by_node }
    }

    /// 節点 `node` に接続する線材要素の添字（接続がなければ空）。
    pub fn indices_at(&self, node: NodeId) -> &[usize] {
        self.by_node
            .get(&node.index())
            .map(|v| v.as_slice())
            .unwrap_or(&[])
    }

    /// 節点 `node` に接続する線材要素（添字を解決したもの）。
    pub fn elements_at<'a>(
        &'a self,
        model: &'a Model,
        node: NodeId,
    ) -> impl Iterator<Item = &'a ElementData> + 'a {
        self.indices_at(node)
            .iter()
            .filter_map(move |&ei| model.elements.get(ei))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dof::Dof6Mask;
    use crate::ids::{ElemId, SectionId};
    use crate::model::{EndCondition, ForceRegime, LocalAxis, Node};

    fn node(id: u32) -> Node {
        Node {
            id: NodeId(id),
            coord: [id as f64 * 1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    fn elem(id: u32, kind: ElementKind, nodes: &[u32]) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind,
            nodes: nodes.iter().map(|&n| NodeId(n)).collect(),
            section: Some(SectionId(0)),
            material: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    fn model() -> Model {
        Model {
            nodes: (0..4).map(node).collect(),
            elements: vec![
                elem(0, ElementKind::Beam, &[0, 1]),
                elem(1, ElementKind::Beam, &[1, 2]),
                // 壁は線材ではないため隣接に含めない。
                elem(2, ElementKind::Wall, &[0, 1, 2, 3]),
                // 1 節点しか持たない要素も含めない。
                elem(3, ElementKind::Beam, &[3]),
            ],
            ..Default::default()
        }
    }

    /// 線材だけが隣接に入り、共有節点では両方が引ける。
    #[test]
    fn test_collects_line_members_only() {
        let m = model();
        let adj = NodeAdjacency::build(&m);
        assert_eq!(adj.indices_at(NodeId(0)), &[0]);
        assert_eq!(adj.indices_at(NodeId(1)), &[0, 1], "共有節点は 2 本");
        assert_eq!(adj.indices_at(NodeId(2)), &[1]);
        assert!(
            adj.indices_at(NodeId(3)).is_empty(),
            "壁・単節点要素は入らない"
        );
    }

    /// 接続がない節点・範囲外の節点は空を返す。
    #[test]
    fn test_unknown_node_is_empty() {
        let m = model();
        let adj = NodeAdjacency::build(&m);
        assert!(adj.indices_at(NodeId(99)).is_empty());
        assert_eq!(adj.elements_at(&m, NodeId(99)).count(), 0);
    }

    /// 要素の参照を直接引ける。
    #[test]
    fn test_elements_at_resolves_references() {
        let m = model();
        let adj = NodeAdjacency::build(&m);
        let ids: Vec<_> = adj.elements_at(&m, NodeId(1)).map(|e| e.id).collect();
        assert_eq!(ids, vec![ElemId(0), ElemId(1)]);
    }
}
