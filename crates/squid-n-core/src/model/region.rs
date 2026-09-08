//! 床領域（[`FloorRegion`]）。大梁の 1 スパン区画。
//!
//! 版の仕様は持たない（[`Slab`] が持つ）。

use super::*;

/// 取り付く床板の取付き先。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum RegionAnchor {
    /// 線に取り付く。
    ///
    /// `nodes` は取付き線の両端、`span` はその線上の無次元区間 `[t_i, t_j]`（0.0〜1.0、
    /// 全長は `[0.0, 1.0]`）。張り出し量 `extent` は `[d_i, d_j]`（区間の始端側・終端側）で、
    /// 符号は取付き線 `nodes[0]`→`nodes[1]` の左側を正とする。
    Line {
        nodes: [NodeId; 2],
        span: [f64; 2],
        transfer: LoadTransfer,
    },
    /// 点（柱）に取り付く。荷重はその節点へ集中する。
    ///
    /// 張り出し量 `extent` は全体座標の `[X 方向, Y 方向]` で、符号が向きを表す。
    Point(NodeId),
    /// 床領域に取り付く（自立壁）。荷重は壁が載っている床領域へ渡し、等価な面荷重へならして分配する。
    ///
    /// `nodes` は壁の始点・終点。荷重を渡す床領域は保存せず、壁の位置から都度求める
    /// （[`Model::self_standing_wall_coverage`]）。床板（[`super::Slab`]）の取付き先としては使わない。
    FloorRegion { nodes: [NodeId; 2] },
}

/// 取付き線の無次元区間 `span = [t_i, t_j]` が規約 `0.0 <= t_i < t_j <= 1.0`
/// を満たすか。
pub fn span_is_valid(span: [f64; 2]) -> bool {
    span[0].is_finite()
        && span[1].is_finite()
        && span[0] >= -1e-9
        && span[1] <= 1.0 + 1e-9
        && span[1] - span[0] > 1e-9
}

/// 取り付く床板の荷重の出口。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum LoadTransfer {
    /// 取付き線へ分布させる（既定）。
    #[default]
    Anchor,
    /// 取付き線の両端（柱）へ集中させる。
    Columns,
}

/// 床領域。大梁が囲む 1 スパン区画。版の仕様は持たない（[`Slab`] が持つ）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FloorRegion {
    /// 床領域 ID（`Model::floor_regions` の配列インデックスと一致すること）。
    pub id: FloorRegionId,
    /// 表示名。空文字は名前なし。
    #[serde(default)]
    pub name: String,
    /// 境界の節点列（大梁の閉路。反時計回り、始点は繰り返さない）。
    pub boundary: Vec<NodeId>,
    /// この床領域に属する小梁の実体。順序は任意。
    #[serde(default)]
    pub secondary_joists: Vec<SecondaryMember>,
    /// この床領域に属する床板の ID リスト。順序は任意。重複・他領域との共有は許さない。
    #[serde(default)]
    pub slab_ids: Vec<SlabId>,
}

impl Model {
    /// 床領域の境界節点がすべて同一の剛床に属するか。
    pub fn floor_region_on_single_diaphragm(&self, region: &FloorRegion) -> bool {
        if region.boundary.is_empty() {
            return false;
        }
        self.constraints.iter().any(|c| match c {
            Constraint::RigidDiaphragm { master, slaves, .. } => region
                .boundary
                .iter()
                .all(|n| n == master || slaves.contains(n)),
            _ => false,
        })
    }

    /// 二次部材（端点対で識別）が属する床領域。どこにも属さなければ `None`。
    pub fn floor_region_of_joist(&self, nodes: [NodeId; 2]) -> Option<&FloorRegion> {
        let key = |a: NodeId, b: NodeId| (a.0.min(b.0), a.0.max(b.0));
        let want = key(nodes[0], nodes[1]);
        self.floor_regions.iter().find(|r| {
            r.secondary_joists
                .iter()
                .any(|j| key(j.nodes[0], j.nodes[1]) == want)
        })
    }
}

impl FloorRegion {
    /// 床領域を作る（版なし・小梁なし）。
    pub fn new(id: FloorRegionId, boundary: Vec<NodeId>) -> Self {
        FloorRegion {
            id,
            name: String::new(),
            boundary,
            secondary_joists: Vec::new(),
            slab_ids: Vec::new(),
        }
    }

    /// 境界多角形の座標列 [mm]。節点が引けない場合は `None`。
    pub fn boundary_coords(&self, model: &Model) -> Option<Vec<[f64; 3]>> {
        self.boundary
            .iter()
            .map(|n| model.nodes.get(n.index()).map(|n| n.coord))
            .collect()
    }

    /// 境界の辺 `k` の両端節点。
    pub fn edge_nodes(&self, k: usize) -> Option<[NodeId; 2]> {
        let n = self.boundary.len();
        (n >= 3 && k < n).then(|| [self.boundary[k], self.boundary[(k + 1) % n]])
    }

    /// 領域を代表する節点（境界の先頭）。
    pub fn reference_node(&self) -> Option<NodeId> {
        self.boundary.first().copied()
    }

    /// 領域のレベル Z [mm]（境界座標の Z の平均）。境界が引けなければ `None`。
    pub fn level(&self, model: &Model) -> Option<f64> {
        let coords = self.boundary_coords(model)?;
        if coords.is_empty() {
            return None;
        }
        Some(coords.iter().map(|c| c[2]).sum::<f64>() / coords.len() as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{FloorRegionId, NodeId};

    fn model_with_nodes(pts: &[[f64; 3]]) -> Model {
        let mut m = Model::default();
        for (i, p) in pts.iter().enumerate() {
            m.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *p,
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            });
        }
        m
    }

    #[test]
    fn test_boundary_coords() {
        let m = model_with_nodes(&[
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 4000.0, 0.0],
            [0.0, 4000.0, 0.0],
        ]);
        let r = FloorRegion::new(
            FloorRegionId(0),
            vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        );
        let coords = r.boundary_coords(&m).expect("境界座標");
        assert_eq!(coords.len(), 4);
        assert_eq!(r.level(&m), Some(0.0));
        assert_eq!(r.reference_node(), Some(NodeId(0)));
        assert_eq!(r.edge_nodes(0), Some([NodeId(0), NodeId(1)]));
    }

    #[test]
    fn span_is_valid_は規約0_0以上t_i未満t_j以下1_0を判定する() {
        // 代表的な妥当値。
        assert!(span_is_valid([0.0, 1.0]));
        assert!(span_is_valid([0.25, 0.75]));

        // 上下端は 1e-9 の許容つき。span は無次元比のため、1.0 を意図した値が
        // 計算経路で 1.0 + ε になりうる。
        assert!(span_is_valid([-1e-10, 1.0 + 1e-10]));
        assert!(!span_is_valid([-1e-3, 1.0]));
        assert!(!span_is_valid([0.0, 1.0 + 1e-3]));

        // つぶれた区間・逆転した区間は載る範囲が無い。
        assert!(!span_is_valid([0.5, 0.5]));
        assert!(!span_is_valid([0.75, 0.25]));

        // 非有限は弾く。
        assert!(!span_is_valid([f64::NAN, 1.0]));
        assert!(!span_is_valid([0.0, f64::INFINITY]));
    }
}
