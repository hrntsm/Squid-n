//! 荷重分配の基本型と辺荷重の共通ヘルパ。
//!
//! - [`LoadShape`] — 荷重形状（等分布・線形変化・台形・三角形・集中）
//! - [`Cmq`] — 両端固定梁の固定端モーメント・せん断（CMQ）
//! - [`LoadTarget`] — 荷重の作用対象（境界辺 / 節点）
//! - [`BeamLoad`] — 分配結果1件（作用対象・荷重形状・CMQ）
//! - [`push_edge`] — 境界辺 i への辺荷重を `loads` へ追加する

use squid_n_core::ids::{ElemId, NodeId};

#[derive(Clone, Copy, Debug)]
pub enum LoadShape {
    Uniform {
        w: f64,
    },
    /// 材軸に沿って強度が線形に変わる分布（始端 `w_i` → 終端 `w_j`）。
    /// 取り付く壁版の台形（張り出し高さが両端で異なる）の自重に使う。
    /// [`LoadShape::Trapezoid`]（床の 45° 分配の対称台形）とは別物である。
    Linear {
        w_i: f64,
        w_j: f64,
    },
    Trapezoid {
        w0: f64,
        a: f64,
        b: f64,
    },
    Triangle {
        w0: f64,
    },
    Point {
        p: f64,
        x: f64,
    },
}

#[derive(Clone, Copy, Debug)]
pub struct Cmq {
    pub c_i: f64,
    pub c_j: f64,
    pub q_i: f64,
    pub q_j: f64,
}

/// 荷重の作用対象。
/// - `Edge(i)`: 境界の辺 i。`elem` にも同じ値 `ElemId(i as u32)` を設定する。
/// - `Node(id)`: 実節点への集中荷重。`elem` は番兵 `ElemId(u32::MAX)`。
/// - `Span { nodes, t }`: 実部材化された小梁への分布荷重。`t`（既定 `[0.0, 1.0]`）は無次元区間。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LoadTarget {
    Edge(usize),
    Node(NodeId),
    Span { nodes: [NodeId; 2], t: [f64; 2] },
}

#[derive(Clone, Copy, Debug)]
pub struct BeamLoad {
    pub elem: ElemId,
    pub target: LoadTarget,
    pub shape: LoadShape,
    pub cmq: Cmq,
}

pub(crate) fn push_edge(loads: &mut Vec<BeamLoad>, i: usize, shape: LoadShape, cmq: Cmq) {
    loads.push(BeamLoad {
        elem: ElemId(i as u32),
        target: LoadTarget::Edge(i),
        shape,
        cmq,
    });
}
