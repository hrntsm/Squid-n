//! 要素にならない囲まれた壁版の自重を、明示された境界辺の負担率で分配する。

use std::collections::HashMap;

use squid_n_core::geom::MEMBER_AXIS_TOL_MM;
use squid_n_core::ids::{ElemId, NodeId, WallPlateId};
use squid_n_core::model::{MemberLoadKind, Model, WallPlate, WallPlateShape};

use crate::cascade::SecondaryKey;

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
    pub posts: HashMap<SecondaryKey, PostWallLoad>,
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

/// 辺の支持部材を引くための索引。
///
/// 主架構の候補列と間柱の材軸を **1 回だけ**組み立てて使い回す。辺ごとに組み直すと、
/// 呼び出しのたびに全要素を走査し直し、辺の本数 × 部材数になる（逐次伝達が
/// `beam_span_candidates` を 1 回だけ構築しているのと同じ理由）。
struct SupportIndex<'a> {
    model: &'a Model,
    beams: Vec<crate::secondary::BeamSpanCandidate>,
    posts: Vec<(SecondaryKey, [f64; 3], [f64; 3])>,
}

impl<'a> SupportIndex<'a> {
    fn new(model: &'a Model) -> Self {
        let posts = model
            .posts()
            .filter_map(|sm| {
                let a = model.nodes.get(sm.nodes[0].index())?.coord;
                let b = model.nodes.get(sm.nodes[1].index())?.coord;
                Some((crate::floor::span_node_key(sm.nodes[0], sm.nodes[1]), a, b))
            })
            .collect();
        SupportIndex {
            model,
            beams: crate::secondary::beam_span_candidates(model),
            posts,
        }
    }

    /// 線分 `p0`–`p1` を覆う支持部材。無ければ `None`。
    ///
    /// **主架構を優先する。** 辺が柱・大梁に覆われているなら、その部材が直接支持して
    /// いるのだから、そこで終端する。10 mm 以内に並走する間柱が主架構の荷重を奪わない
    /// ようにするためでもある（逐次伝達の `support_of`・小梁の並走大梁優先と同じ考え）。
    fn of(&self, p0: [f64; 3], p1: [f64; 3]) -> Option<EdgeSupport> {
        let coverage =
            crate::secondary::beams_along_segment_with(&self.beams, p0, p1, MEMBER_AXIS_TOL_MM);
        let length_mm = squid_n_core::geom::vec3::dist(p0, p1);
        let mut covered_end_mm: f64 = 0.0;
        for part in &coverage {
            if part.seg[0] < covered_end_mm - 1e-9 {
                return None;
            }
            if part.seg[0] > covered_end_mm + MEMBER_AXIS_TOL_MM {
                break;
            }
            covered_end_mm = covered_end_mm.max(part.seg[1]);
        }
        if !coverage.is_empty() && covered_end_mm >= length_mm - MEMBER_AXIS_TOL_MM {
            return Some(EdgeSupport::Primary);
        }
        self.posts
            .iter()
            .find(|(_, a, b)| {
                project_on_segment(p0, *a, *b).is_some() && project_on_segment(p1, *a, *b).is_some()
            })
            .map(|(key, _, _)| EdgeSupport::Post(*key))
    }
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

/// 境界の辺ごとに、耐震スリットで縁が切れているかを返す（`boundary` と同じ並び）。
///
/// スリットは辺ごとの縁切りなので、辺の役割（柱際か梁際か、下辺か上辺か）へ
/// 対応付ける必要がある。柱際は [`WallPlate::column_face_nodes`] の並びで、
/// 梁際は標高の低い側を下辺として引き当てる。
///
/// 境界が 4 節点でない壁版は役割を決められないため、切れていないものとして扱う
/// （スリットは 4 節点の囲まれた壁版でのみ意味を持つ。[`squid_n_core::model::WallSlit`]）。
fn slit_edge_flags(
    model: &Model,
    plate: &WallPlate,
    boundary: &[NodeId],
    coords: &[[f64; 3]],
) -> Vec<bool> {
    let n = boundary.len();
    let mut out = vec![false; n];
    if n != 4 || !plate.slit.any() {
        return out;
    }
    let faces = plate.column_face_nodes(model);
    let mid_z = |i: usize| (coords[i][2] + coords[(i + 1) % n][2]) / 2.0;
    let horizontal: Vec<usize> = (0..n)
        .filter(|&i| is_horizontal(coords[i], coords[(i + 1) % n]))
        .collect();
    let lowest = horizontal
        .iter()
        .copied()
        .min_by(|&a, &b| mid_z(a).total_cmp(&mid_z(b)));
    for i in 0..n {
        let (a, b) = (coords[i], coords[(i + 1) % n]);
        if is_vertical(a, b) {
            let lower = if a[2] <= b[2] {
                boundary[i]
            } else {
                boundary[(i + 1) % n]
            };
            if let Some([f0, f1]) = faces {
                if f0 != f1 {
                    if lower == f0 {
                        out[i] = plate.slit.column_face[0];
                    } else if lower == f1 {
                        out[i] = plate.slit.column_face[1];
                    }
                }
            }
        } else if is_horizontal(a, b) {
            let is_bottom = lowest == Some(i);
            out[i] = plate.slit.beam_face[usize::from(!is_bottom)];
        }
    }
    out
}

/// 壁版 1 枚の自重を辺へ配る。
fn edge_shares_with(index: &SupportIndex, plate: &WallPlate) -> Vec<WallEdgeShare> {
    let model = index.model;
    let WallPlateShape::Enclosed { boundary } = &plate.shape else {
        return Vec::new();
    };
    if model.wall_plate_becomes_element(plate) {
        return Vec::new();
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

    if !plate.has_valid_self_weight_shares() {
        return Vec::new();
    }
    let n = boundary.len();
    let slit_edge = slit_edge_flags(model, plate, boundary, &coords);
    let mut shares = Vec::new();
    for (i, &ratio) in plate.self_weight_shares.iter().enumerate() {
        if ratio == 0.0 {
            continue;
        }
        if slit_edge[i] {
            return Vec::new();
        }
        let Some(support) = index.of(coords[i], coords[(i + 1) % n]) else {
            return Vec::new();
        };
        shares.push(WallEdgeShare {
            nodes: [boundary[i], boundary[(i + 1) % n]],
            total: total * ratio,
            post: match support {
                EdgeSupport::Primary => None,
                EdgeSupport::Post(key) => Some(key),
            },
        });
    }
    shares
}

/// 要素にならない全壁版の自重を分配する。
pub fn distribute_enclosed_wall_plates(model: &Model) -> EnclosedWallLoads {
    let index = SupportIndex::new(model);
    let mut out = EnclosedWallLoads::default();
    for plate in &model.wall_plates {
        for share in edge_shares_with(&index, plate) {
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
    key: SecondaryKey,
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

/// 壁版と二次部材の自重を支持先へ伝え、地震用節点重量 [N] に加算する。
/// 未指定・支持欠落・循環があれば加算前にエラーを返す。
pub fn accumulate_wall_and_secondary_seismic_weight(
    model: &Model,
    node_weight: &mut [f64],
) -> Result<(), String> {
    if !wall_plates_without_load_path(model).is_empty() {
        return Err("壁版の自重支持辺が未指定・不正、または支持先へ荷重を伝えられません".into());
    }
    let transfer = crate::cascade::solve(model, |_| 0.0, true);
    if !transfer.invalid_end_shares.is_empty()
        || !transfer.unresolved.is_empty()
        || !transfer.cyclic.is_empty()
    {
        return Err("二次部材の端部負担率または支持先が不正で、自重を伝えられません".into());
    }
    let index = SupportIndex::new(model);
    for plate in &model.wall_plates {
        for share in edge_shares_with(&index, plate)
            .into_iter()
            .filter(|s| s.post.is_none())
        {
            let a = model.nodes[share.nodes[0].index()].coord;
            let b = model.nodes[share.nodes[1].index()].coord;
            let length_mm = squid_n_core::geom::vec3::dist(a, b);
            let w = share.total / length_mm;
            for part in
                crate::secondary::beams_along_segment_with(&index.beams, a, b, MEMBER_AXIS_TOL_MM)
            {
                let Some(elem) = model.elements.iter().find(|e| e.id == part.elem) else {
                    continue;
                };
                let load = MemberLoadKind::Distributed {
                    a: part.elem_pos[0].min(part.elem_pos[1]),
                    b: part.elem_pos[0].max(part.elem_pos[1]),
                    w1: w,
                    w2: w,
                };
                let (ri, rj) = crate::floor::simple_reactions(&load, model.member_length(elem));
                node_weight[elem.nodes[0].index()] += ri;
                node_weight[elem.nodes[1].index()] += rj;
            }
        }
    }
    let nodal = transfer
        .primary_node_loads()
        .into_iter()
        .map(|(node, weight)| {
            squid_n_core::model::NodalLoad::manual(node, [0.0, 0.0, -weight, 0.0, 0.0, 0.0])
        })
        .collect();
    let (nodal, member) =
        crate::secondary::resolve_nodal_to_primary(model, nodal, MEMBER_AXIS_TOL_MM);
    for load in nodal {
        node_weight[load.node.index()] -= load.values[2];
    }
    for load in member {
        let Some(elem) = model.elements.iter().find(|e| e.id == load.elem) else {
            continue;
        };
        let (ri, rj) = crate::floor::simple_reactions(&load.kind, model.member_length(elem));
        node_weight[elem.nodes[0].index()] += ri * -load.dir[2];
        node_weight[elem.nodes[1].index()] += rj * -load.dir[2];
    }
    Ok(())
}

/// 自重を持つ非要素の囲まれた壁版のうち、指定した支持辺へ伝達できないものを返す。
/// 負担率の不備・スリット・支持欠落・主架構の支持区間重複を解析前エラーの対象とする。
pub fn wall_plates_without_load_path(model: &Model) -> Vec<WallPlateId> {
    let index = SupportIndex::new(model);
    model
        .wall_plates
        .iter()
        .filter(|plate| {
            if !matches!(plate.shape, WallPlateShape::Enclosed { .. })
                || model.wall_plate_becomes_element(plate)
            {
                return false;
            }
            model
                .wall_plate_self_weight(plate, model)
                .is_some_and(|w| w > 0.0)
                && edge_shares_with(&index, plate).is_empty()
        })
        .map(|plate| plate.id)
        .collect()
}

#[cfg(test)]
mod tests;
