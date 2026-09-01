//! 解析要素にならない壁版（`WallPlateShape::Enclosed`）の自重の分配。
//!
//! 壁エレメントになるのは壁領域全体を覆う 4 節点の壁版だけである
//! （[`squid_n_core::model::Model::wall_plate_covers_region`]）。それ以外の囲まれた
//! 壁版（間柱で分割された壁版・腰壁・垂れ壁・5 節点以上の壁領域内の壁版）は
//! **荷重だけを持つ壁版**であり、その自重を境界の辺へ配るのが本モジュールである。
//! 取り付く壁版（`WallPlateShape::Attached`）は [`crate::wall_attached`] が受け持つ。
//!
//! # 一方向版として配る
//!
//! 床板の面荷重は**版に直交**して作用するため、版の曲げで支持辺へ流れ、45 度の
//! 降伏線に基づく三角形・台形分配に力学的根拠がある。**壁版の自重は版の面内**
//! （真下向き）に作用するので、版の曲げは関与せず、荷重は壁版が何に留め付けられて
//! いるかで流れる。床の分配則を鉛直面へ機械的に移植する根拠はないため、壁版は
//! 一方向版として扱う。
//!
//! - 境界の**鉛直な辺**に支持部材（間柱または柱）がちょうど 2 つあるなら、
//!   自重を**その 2 辺へ半分ずつ**配る（ALC 横張りのように鉛直材が受ける形）。
//! - そうでなければ、**最も低い水平な辺**へ全量を配る（縦張り・腰壁の形）。
//!
//! 「この間柱がこの壁を受ける」ことは、**利用者が壁版を間柱の位置で分割する**ことで
//! 表明する。床側で「床板の境界辺が小梁の材軸に載っているならその小梁が受ける」と
//! しているのと同じ規約であり、受け手を指す参照フィールドは持たない。
//!
//! # 誤りうる向き
//!
//! 鉛直辺に支持部材があり、かつ下辺でも受けている壁（ALC 縦張り＋間柱による面外
//! 拘束）を横張りとみなすと、間柱・柱の負担を過大に見る。これは間柱・柱にとって
//! 安全側であり、下の梁にとっても間柱の反力が中間集中荷重として載るぶん安全側である。
//!
//! # 間柱が受けたぶんの行き先
//!
//! 間柱は解析要素ではないため、受け持った荷重は二次部材の反力の逐次伝達
//! （[`crate::cascade`]）が単純梁の両端反力へ変えて主架構へ運ぶ。鉛直な間柱の
//! 両端配分は**既定で上下端へ 1/2 ずつ**である（`cascade` のモジュール doc）。

use std::collections::HashMap;

use squid_n_core::geom::MEMBER_AXIS_TOL_MM;
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{MemberLoadKind, Model, WallPlate, WallPlateShape};

use crate::floor::{fem_uniform, BeamLoad, LoadShape, LoadTarget};

/// 1 枚の壁版の自重のうち、1 つの辺が受け持つぶん。
#[derive(Clone, Copy, Debug)]
pub struct WallEdgeShare {
    /// 受け持つ辺の両端節点。
    pub nodes: [NodeId; 2],
    /// この辺が受け持つ重量 [N]（下向きを正）。
    pub total: f64,
    /// 受け手が間柱なら、その端点対（順不同キー）。主架構（柱・梁）なら `None`。
    pub post: Option<(NodeId, NodeId)>,
}

/// 間柱 1 本が壁版から受け持つ荷重。
#[derive(Clone, Debug)]
pub struct PostWallLoad {
    /// 間柱の端点（`SecondaryMember::nodes` と同じ順。材軸局所座標の原点は `[0]`）。
    pub span_nodes: [NodeId; 2],
    /// 材軸局所の部材荷重（下向きを正）。
    pub member_loads: Vec<MemberLoadKind>,
}

/// 要素にならない壁版の自重の分配結果。
#[derive(Clone, Debug, Default)]
pub struct EnclosedWallLoads {
    /// 間柱が受け持つ荷重（端点対キー）。
    pub posts: HashMap<(NodeId, NodeId), PostWallLoad>,
    /// 主架構（柱・大梁）が受け持つ辺荷重。床板の分配と同じ幾何解決
    /// （`squid-n-job::auto_loads::slab_load_case_content`）へ合流させる。
    pub primary: Vec<BeamLoad>,
}

/// 辺が鉛直か（水平投影が許容差以下）。
fn is_vertical(a: [f64; 3], b: [f64; 3]) -> bool {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    (dx * dx + dy * dy).sqrt() <= MEMBER_AXIS_TOL_MM && (b[2] - a[2]).abs() > MEMBER_AXIS_TOL_MM
}

/// 辺が水平か（鉛直方向の差が許容差以下）。
fn is_horizontal(a: [f64; 3], b: [f64; 3]) -> bool {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    (b[2] - a[2]).abs() <= MEMBER_AXIS_TOL_MM && (dx * dx + dy * dy).sqrt() > MEMBER_AXIS_TOL_MM
}

/// 辺の支持部材。
#[derive(Clone, Copy, Debug, PartialEq)]
enum EdgeSupport {
    /// 主架構（柱・大梁）が覆っている。
    Primary,
    /// 間柱が覆っている。
    Post((NodeId, NodeId)),
}

/// 線分 `p0`–`p1` を覆う支持部材。無ければ `None`。
///
/// **主架構を優先する。** 辺が柱・大梁に覆われているなら、その部材が直接支持して
/// いるのだから、そこで終端する。10 mm 以内に並走する間柱が主架構の荷重を奪わない
/// ようにするためでもある（逐次伝達の `support_of`・小梁の並走大梁優先と同じ考え）。
fn edge_support(model: &Model, p0: [f64; 3], p1: [f64; 3]) -> Option<EdgeSupport> {
    if primary_covering(model, p0, p1) {
        return Some(EdgeSupport::Primary);
    }
    post_covering(model, p0, p1).map(EdgeSupport::Post)
}

/// 線分 `p0`–`p1` を覆う間柱の端点対。無ければ `None`。
///
/// 間柱の材軸が線分を含む（両端が材軸上にあり、区間が重なる）ものを探す。
fn post_covering(model: &Model, p0: [f64; 3], p1: [f64; 3]) -> Option<(NodeId, NodeId)> {
    for sm in model.posts() {
        let (Some(a), Some(b)) = (
            model.nodes.get(sm.nodes[0].index()).map(|n| n.coord),
            model.nodes.get(sm.nodes[1].index()).map(|n| n.coord),
        ) else {
            continue;
        };
        if project_on_segment(p0, a, b).is_some() && project_on_segment(p1, a, b).is_some() {
            return Some(crate::floor::span_node_key(sm.nodes[0], sm.nodes[1]));
        }
    }
    None
}

/// 点 `p` の線分 `a`→`b` 上の位置 [mm]。材軸から離れている・区間外なら `None`。
fn project_on_segment(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> Option<f64> {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len = (ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]).sqrt();
    if len <= 1e-9 {
        return None;
    }
    let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let t = (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / (len * len);
    let s = t * len;
    if s < -MEMBER_AXIS_TOL_MM || s > len + MEMBER_AXIS_TOL_MM {
        return None;
    }
    let proj = [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]];
    let d = [proj[0] - p[0], proj[1] - p[1], proj[2] - p[2]];
    ((d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() <= MEMBER_AXIS_TOL_MM)
        .then(|| s.clamp(0.0, len))
}

/// 線分 `p0`–`p1` を覆う主架構の線材があるか。
fn primary_covering(model: &Model, p0: [f64; 3], p1: [f64; 3]) -> bool {
    !crate::secondary::beams_along_segment(model, p0, p1, MEMBER_AXIS_TOL_MM).is_empty()
}

/// 壁版 1 枚の自重を辺へ配る（モジュール doc「一方向版として配る」）。
///
/// 荷重の分配と地震用重量の集計が同じ規則を見るよう、辺への配分はこの関数 1 つに置く。
pub fn wall_plate_edge_shares(model: &Model, plate: &WallPlate) -> Vec<WallEdgeShare> {
    let WallPlateShape::Enclosed { boundary } = &plate.shape else {
        return Vec::new(); // 取り付く壁版は `wall_attached` が受け持つ。
    };
    if model.wall_plate_covers_region(plate) {
        return Vec::new(); // 壁エレメントになる壁版の自重は要素の頂点へ配られる。
    }
    let Some(total) = model.wall_plate_self_weight(plate, model) else {
        return Vec::new();
    };
    if total <= 0.0 || boundary.len() < 3 {
        return Vec::new();
    }
    let Some(coords) = boundary
        .iter()
        .map(|n| model.nodes.get(n.index()).map(|nd| nd.coord))
        .collect::<Option<Vec<[f64; 3]>>>()
    else {
        return Vec::new();
    };

    // 支持部材のある鉛直な辺を集める。
    let n = boundary.len();
    let mut vertical: Vec<(usize, Option<(NodeId, NodeId)>)> = Vec::new();
    let mut horizontal: Vec<usize> = Vec::new();
    for i in 0..n {
        let (a, b) = (coords[i], coords[(i + 1) % n]);
        if is_vertical(a, b) {
            match edge_support(model, a, b) {
                Some(EdgeSupport::Post(key)) => vertical.push((i, Some(key))),
                Some(EdgeSupport::Primary) => vertical.push((i, None)),
                None => {}
            }
        } else if is_horizontal(a, b) {
            horizontal.push(i);
        }
    }

    let edge_nodes = |i: usize| [boundary[i], boundary[(i + 1) % n]];

    if vertical.len() == 2 {
        return vertical
            .into_iter()
            .map(|(i, post)| WallEdgeShare {
                nodes: edge_nodes(i),
                total: total / 2.0,
                post,
            })
            .collect();
    }

    // 鉛直辺が受けない壁は、最も低い水平な辺（下の大梁）が全量を受ける。
    let Some(&bottom) = horizontal.iter().min_by(|&&i, &&j| {
        let zi = (coords[i][2] + coords[(i + 1) % n][2]) / 2.0;
        let zj = (coords[j][2] + coords[(j + 1) % n][2]) / 2.0;
        zi.total_cmp(&zj)
    }) else {
        return Vec::new(); // 水平な辺が無い壁は荷重の行き先が決まらない。
    };
    let (a, b) = (coords[bottom], coords[(bottom + 1) % n]);
    let post = match edge_support(model, a, b) {
        Some(EdgeSupport::Post(key)) => Some(key),
        _ => None,
    };
    vec![WallEdgeShare {
        nodes: edge_nodes(bottom),
        total,
        post,
    }]
}

/// 要素にならない全壁版の自重を分配する。
pub fn distribute_enclosed_wall_plates(model: &Model) -> EnclosedWallLoads {
    let mut out = EnclosedWallLoads::default();
    for plate in &model.wall_plates {
        for share in wall_plate_edge_shares(model, plate) {
            match share.post {
                Some(key) => push_post_share(model, &mut out, key, &share),
                None => push_primary_share(model, &mut out.primary, &share),
            }
        }
    }
    out
}

/// 間柱が受け持つぶんを、間柱の材軸局所の等分布荷重として積む。
fn push_post_share(
    model: &Model,
    out: &mut EnclosedWallLoads,
    key: (NodeId, NodeId),
    share: &WallEdgeShare,
) {
    let Some(sm) = model
        .posts()
        .find(|sm| crate::floor::span_node_key(sm.nodes[0], sm.nodes[1]) == key)
    else {
        return;
    };
    let (Some(pa), Some(pb)) = (
        model.nodes.get(sm.nodes[0].index()).map(|n| n.coord),
        model.nodes.get(sm.nodes[1].index()).map(|n| n.coord),
    ) else {
        return;
    };
    let (Some(e0), Some(e1)) = (
        model.nodes.get(share.nodes[0].index()).map(|n| n.coord),
        model.nodes.get(share.nodes[1].index()).map(|n| n.coord),
    ) else {
        return;
    };
    // 壁版の辺を間柱の材軸へ写す（辺が間柱の一部しか覆わない腰壁等に対応する）。
    let (Some(s0), Some(s1)) = (
        project_on_segment(e0, pa, pb),
        project_on_segment(e1, pa, pb),
    ) else {
        return;
    };
    let (lo, hi) = (s0.min(s1), s0.max(s1));
    if hi - lo <= 1e-9 {
        return;
    }
    let w = share.total / (hi - lo);
    let entry = out.posts.entry(key).or_insert_with(|| PostWallLoad {
        span_nodes: sm.nodes,
        member_loads: Vec::new(),
    });
    entry.member_loads.push(MemberLoadKind::Distributed {
        a: lo,
        b: hi,
        w1: w,
        w2: w,
    });
}

/// 主架構が受け持つぶんを、辺に沿った等分布の `LoadTarget::Span` として積む。
///
/// 実部材への割り付けは床板の辺荷重と同じ幾何解決
/// （`squid-n-job::auto_loads::slab_load_case_content`）へ委ねる。
fn push_primary_share(model: &Model, loads: &mut Vec<BeamLoad>, share: &WallEdgeShare) {
    let (Some(a), Some(b)) = (
        model.nodes.get(share.nodes[0].index()).map(|n| n.coord),
        model.nodes.get(share.nodes[1].index()).map(|n| n.coord),
    ) else {
        return;
    };
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if len <= 1e-9 || share.total.abs() <= 1e-9 {
        return;
    }
    let w = share.total / len;
    loads.push(BeamLoad {
        elem: ElemId(u32::MAX),
        target: LoadTarget::Span {
            nodes: share.nodes,
            t: [0.0, 1.0],
        },
        shape: LoadShape::Uniform { w },
        cmq: fem_uniform(w, len),
    });
}

/// 要素にならない壁版の自重を、地震用重量の節点重量へ集計する。
///
/// 辺への配分は [`wall_plate_edge_shares`] と共有し、辺が受け持ったぶんを
/// その辺の両端節点へ 1/2 ずつ載せる。矩形の壁版が左右の鉛直辺で受ける場合、
/// 上下 2 節点ずつへ 1/4 ずつとなり、壁エレメントの頂点等分配と一致する
/// （壁の重量を階高の中央で上下階の節点に分配する扱い）。
pub fn accumulate_enclosed_wall_seismic_weight(model: &Model, node_weight: &mut [f64]) {
    for plate in &model.wall_plates {
        for share in wall_plate_edge_shares(model, plate) {
            for node in share.nodes {
                if let Some(slot) = node_weight.get_mut(node.index()) {
                    *slot += share.total / 2.0;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests;
