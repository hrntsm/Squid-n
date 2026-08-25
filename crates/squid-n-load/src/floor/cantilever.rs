//! 片持ちスラブ・出隅スラブの分配戦略。
//!
//! - [`distribute_to_node`] — 取り付き領域の荷重を節点（柱）へ集中
//! - [`distribute_cantilever`] — 片持ちスラブ（取付き大梁への等分布集約）

use squid_n_core::ids::ElemId;
use squid_n_core::ids::NodeId;

use super::fem::fem_uniform;
use super::geometry::{dist3, edge_len, polygon_area};
use super::types::{push_edge, BeamLoad, Cmq, LoadShape, LoadTarget};

/// 取り付き領域の荷重を節点（柱）へ集中させる分配。
///
/// 出隅の片持ちスラブは、荷重伝達方向および片持ち梁の取付きに関わらず、節点荷重として
/// すべて柱に伝達する。本実装ではこれに従い、全荷重 `W = w × 多角形面積`
/// （[`polygon_area`]。構造芯から出隅先端までの長方形＝境界そのものの面積）に
/// `ratio` を掛けた分を、`node` への単一の集中荷重として返す。
/// 小梁反力・`distribute_rect_with_joists` の柱集中荷重と同じ
/// `LoadTarget::Node` + `LoadShape::Point`（`q_i = W`、`q_j = 0`）の機構を再利用する。
///
/// `ratio` は呼び出し側が渡す按分比。[`LoadTransfer::Columns`] は取付き線の区間中点
/// `t_mid` から `1.0 - t_mid`／`t_mid` を渡す（全長 `[0, 1]` なら `t_mid = 0.5` で
/// 両端とも 0.5）。出隅（[`RegionAnchor::Point`]）は全荷重を渡すため 1.0 を用いる。
pub(crate) fn distribute_to_node(
    node: NodeId,
    coords: &[[f64; 3]],
    w: f64,
    ratio: f64,
    loads: &mut Vec<BeamLoad>,
) {
    let area = polygon_area(coords);
    if area <= 0.0 {
        return;
    }
    let total = w * area * ratio;
    loads.push(BeamLoad {
        elem: ElemId(u32::MAX),
        target: LoadTarget::Node(node),
        shape: LoadShape::Point { p: total, x: 0.0 },
        cmq: Cmq {
            c_i: 0.0,
            c_j: 0.0,
            q_i: total,
            q_j: 0.0,
        },
    });
}

fn point_line_dist(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let ap = [p[0] - a[0], p[1] - a[1]];
    let len = (ab[0] * ab[0] + ab[1] * ab[1]).sqrt();
    if len < 1e-12 {
        return dist3(p, a);
    }
    (ap[0] * ab[1] - ap[1] * ab[0]).abs() / len
}

/// 片持ちスラブの分配（`SlabKind::Cantilever`）。
///
/// 4頂点を想定し、境界辺0（`boundary[0]`→`boundary[1]`）を取付き辺（大梁側）、
/// その対辺2を先端とみなす。出し幅 `d` は辺0の直線から頂点2・3までの垂直距離の平均。
/// 片持ち梁がない場合は全て大梁に伝達する扱いとし、取付き辺へ
/// 等分布荷重 `w_line = w·d`（先端まで一様なスラブの単純片持ち反力に相当）として
/// 集約する（`LoadShape::Uniform` + `fem_uniform`）。
///
/// 片持ち梁・先端リブ小梁がある場合の分割伝達は `slab.edge_supported` を指定することで
/// 領域が持つ二次部材から支持辺を導く経路（申し送りの Step 4）が担う予定である。
/// 出隅の片持ちスラブは `SlabKind::Corner`（[`distribute_corner`]）が別途担う。
///
/// **未対応（残課題）**: 入隅の片持ちスラブ（本実装では非対応）。
pub(crate) fn distribute_cantilever(coords: &[[f64; 3]], w: f64, loads: &mut Vec<BeamLoad>) {
    if coords.len() < 4 {
        return;
    }
    let l_attach = edge_len(coords, 0);
    let d = 0.5
        * (point_line_dist(coords[2], coords[0], coords[1])
            + point_line_dist(coords[3], coords[0], coords[1]));
    if l_attach <= 1e-9 || d <= 1e-9 {
        return;
    }
    let w_line = w * d;
    push_edge(
        loads,
        0,
        LoadShape::Uniform { w: w_line },
        fem_uniform(w_line, l_attach),
    );
}
