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
///
/// 既存の `BeamLoad.elem` はスラブ境界の「辺インデックス」(`ElemId(i)`, 辺 i =
/// `boundary[i]`→`boundary[(i+1)%n]`) を格納する規約（呼び出し側 `squid-n-app` の
/// `refresh_beam_loads` が辺→実部材へ対応付ける）。この規約は変更しない。
///
/// 一方、小梁反力のように「特定の辺に載らず、特定の節点へ集中荷重として作用する」
/// 荷重を表現する必要が出てきたため、`target` フィールドを追加した。
/// - `Edge(i)`: 従来どおり境界の辺 i。`elem` フィールドにも同じ値 `ElemId(i as u32)` を
///   設定する（互換のため二重に持つ）。
/// - `Node(id)`: 実節点 `id` への集中荷重。`elem` フィールドは無意味なので
///   `ElemId(u32::MAX)` を番兵として設定する（`squid-n-app` 側は `elem` しか見ないため、
///   境界辺数 4 を必ず超えるこの値により誤って辺荷重として処理されることを防ぐ）。
///   `squid-n-app::refresh_beam_loads` は現状 `target` を解釈しないため、`Node` 荷重を
///   実際に構造モデルへ反映する対応は後続タスク（app 側）で行う。
/// - `Span { nodes, t }`: 実部材化された小梁（`nodes[0]`↔`nodes[1]` を両端に持つ実 `Beam`
///   要素）への等分布荷重。小梁が実部材として存在する場合、点反力ではなくこの分布荷重を
///   小梁自身へ載せ、小梁が FEM で支持へ伝達する（`elem` は番兵 `ElemId(u32::MAX)`。app 側が
///   節点対から実 `ElemId` を解決する）。
///
///   `t`（既定 `[0.0, 1.0]`）は `nodes[0]`→`nodes[1]` 上の無次元区間で、荷重が
///   `nodes` 間の全長ではなく一部だけを覆う場合に使う（取付き線の部分区間。
///   `squid_n_core::model::RegionAnchor::Line::span` から渡す）。全長を覆う場合
///   （既存の小梁反力・矩形床境界など）は `t = [0.0, 1.0]` を渡す。
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
