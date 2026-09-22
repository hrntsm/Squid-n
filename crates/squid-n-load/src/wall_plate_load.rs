//! 要素にならない囲まれた壁版の自重を、明示された境界辺の負担率で分配する。
//!
//! 各辺の支持先は、床板と同じく壁版割当領域の境界
//! （[`squid_n_core::model::SupportBoundary`]）を正本とする。境界の頂点にモデル節点が
//! 無くても（間柱端が梁中間にある場合など）支持部材と材軸区間をそのまま使える。

use std::collections::HashMap;

use squid_n_core::geom::MEMBER_AXIS_TOL_MM;
use squid_n_core::ids::{NodeId, SecondaryMemberId, WallPlateId};
use squid_n_core::model::{MemberLoadKind, Model, SupportMemberId, WallPlate, WallPlateShape};

use crate::cascade::SecondaryKey;

use crate::floor::{fem_uniform, BeamLoad, LoadShape, LoadTarget};

/// 1 枚の壁版の自重のうち、1 つの境界辺が受け持つぶん。
#[derive(Clone, Copy, Debug)]
pub struct WallEdgeShare {
    /// 辺が載る支持部材。
    pub support: SupportMemberId,
    /// 支持部材材軸上の無次元区間（割当領域の境界が持つ値）。
    pub span: [f64; 2],
    /// この辺が受け持つ重量 [N]（下向きを正）。
    pub total: f64,
}

impl WallEdgeShare {
    /// 受け手が間柱ならその安定 ID。主架構（柱・梁）なら `None`。
    pub fn post(&self) -> Option<SecondaryMemberId> {
        match self.support {
            SupportMemberId::Secondary(key) => Some(key),
            SupportMemberId::Primary(_) => None,
        }
    }
}

/// 間柱 1 本が壁版から受け持つ荷重。
#[derive(Clone, Debug)]
pub struct PostWallLoad {
    /// 材軸局所の部材荷重（下向きを正。原点は間柱の材軸始端）。
    pub member_loads: Vec<MemberLoadKind>,
}

/// 要素にならない壁版の自重の分配結果。
#[derive(Clone, Debug, Default)]
pub struct EnclosedWallLoads {
    /// 間柱が受け持つ荷重（間柱の安定 ID キー）。
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

/// 耐震スリット指定を境界辺へ対応付けたフラグ（`boundary_len` と同じ並び）。
///
/// 境界が 4 辺の囲まれた壁版で、境界の頂点にモデル節点があり、柱際・梁際の辺の
/// 役割を決められるときだけ `Some` を返す。それ以外は `None`（指定を反映できない）。
fn resolved_slit_edge_flags(
    model: &Model,
    plate: &WallPlate,
    boundary_len: usize,
) -> Option<Vec<bool>> {
    if boundary_len != 4 {
        return None;
    }
    let nodes = plate.boundary_nodes(model)?;
    let coords = plate.boundary_coords(model)?;
    (nodes.len() == 4 && coords.len() == 4).then(|| slit_edge_flags(model, plate, &nodes, &coords))
}

/// 壁版の耐震スリット指定が少なくとも 1 辺へ反映されるか。
///
/// 指定が無ければ常に `true`（警告対象にしない）。境界が 4 辺の囲まれた壁版で、
/// 境界頂点のモデル節点から辺の役割（柱際・梁際）を決められ、指定した辺が
/// 実際に対応付くときに `true`。境界が 4 辺でない、頂点にモデル節点が無い、
/// 指定した辺を柱際・梁際へ対応付けられない場合は `false` となる。
///
/// [`edge_shares_with`] が同一の判定で指定を反映するかを決めるため、診断と
/// 荷重分配で「スリットが効くか」の答えが食い違わない。
pub fn slit_specification_is_reflected(model: &Model, plate: &WallPlate) -> bool {
    if !plate.slit.any() {
        return true;
    }
    let Some(region) = model.wall_plate_assignment_region(plate.id) else {
        return false;
    };
    resolved_slit_edge_flags(model, plate, region.boundary.len())
        .is_some_and(|flags| flags.iter().any(|flag| *flag))
}

/// 壁版 1 枚の自重を辺へ配る。
///
/// 各辺の支持部材と材軸区間は、壁版が割り当てられた壁版割当領域の境界をそのまま使う
/// （境界の頂点にモデル節点が無くても支持先を引ける）。負担率の並びは境界の辺順に
/// 対応する。スリットは 4 節点の囲まれた壁版でのみ意味を持ち、境界頂点の節点を
/// 引けない場合は切れていない扱いとする。
fn edge_shares_with(
    model: &Model,
    plate: &WallPlate,
    basis: crate::cascade::SelfWeightBasis,
) -> Vec<WallEdgeShare> {
    if !matches!(plate.shape, WallPlateShape::Enclosed) {
        return Vec::new();
    }
    if model.wall_plate_becomes_element(plate) {
        return Vec::new();
    }
    let total = match basis {
        crate::cascade::SelfWeightBasis::Design => model.wall_plate_self_weight(plate, model),
        crate::cascade::SelfWeightBasis::MassEquiv => {
            model.wall_plate_physical_weight(plate, model)
        }
    };
    let Some(total) = total else {
        return Vec::new();
    };
    if total <= 0.0 || !plate.has_valid_self_weight_shares(model) {
        return Vec::new();
    }
    let Some(region) = model.wall_plate_assignment_region(plate.id) else {
        return Vec::new();
    };
    let boundary = &region.boundary;
    if boundary.len() < 3 {
        return Vec::new();
    }
    let slit_edge = resolved_slit_edge_flags(model, plate, boundary.len())
        .unwrap_or_else(|| vec![false; boundary.len()]);
    let mut shares = Vec::new();
    for (i, &ratio) in plate.self_weight_shares.iter().enumerate() {
        if ratio == 0.0 {
            continue;
        }
        if slit_edge[i] {
            return Vec::new();
        }
        let edge = boundary[i];
        shares.push(WallEdgeShare {
            support: edge.support,
            span: edge.span,
            total: total * ratio,
        });
    }
    shares
}

/// 要素にならない全壁版の自重を分配する（設計重量基準）。
pub fn distribute_enclosed_wall_plates(model: &Model) -> EnclosedWallLoads {
    distribute_enclosed_wall_plates_with_basis(model, crate::cascade::SelfWeightBasis::Design)
}

/// [`distribute_enclosed_wall_plates`] の自重基準（[`crate::cascade::SelfWeightBasis`]）を
/// 選べる版。支持経路・負担率は基準によらず同じで、躯体の単位体積重量だけが変わる。
pub fn distribute_enclosed_wall_plates_with_basis(
    model: &Model,
    basis: crate::cascade::SelfWeightBasis,
) -> EnclosedWallLoads {
    let mut out = EnclosedWallLoads::default();
    for plate in &model.wall_plates {
        for share in edge_shares_with(model, plate, basis) {
            match share.post() {
                Some(key) => push_post_share(model, &mut out, key, &share),
                None => push_primary_share(model, &mut out.primary, &share),
            }
        }
    }
    out
}

/// 間柱が受け持つぶんを、間柱の材軸局所の等分布荷重として積む。
///
/// 材軸区間は割当領域の境界が持つ無次元区間をそのまま材軸長へ写す。
fn push_post_share(
    model: &Model,
    out: &mut EnclosedWallLoads,
    key: SecondaryKey,
    share: &WallEdgeShare,
) {
    let Some(sm) = model.posts().find(|sm| sm.id == key) else {
        return;
    };
    let Some((_, _, len)) = model.secondary_member_axis(sm) else {
        return;
    };
    if len <= 1e-9 {
        return;
    }
    let s0 = share.span[0] * len;
    let s1 = share.span[1] * len;
    let (lo, hi) = (s0.min(s1), s0.max(s1));
    if hi - lo <= 1e-9 {
        return;
    }
    let w = share.total / (hi - lo);
    let entry = out.posts.entry(key).or_insert_with(|| PostWallLoad {
        member_loads: Vec::new(),
    });
    entry.member_loads.push(MemberLoadKind::Distributed {
        a: lo,
        b: hi,
        w1: w,
        w2: w,
    });
}

/// 主架構が受け持つぶんを、支持部材の材軸区間への等分布 `LoadTarget::Span` として積む。
///
/// 実部材への割り付けは床板の辺荷重と同じ解決
/// （`squid-n-job::auto_loads::slab_load_case_content`）へ委ねる。
fn push_primary_share(model: &Model, loads: &mut Vec<BeamLoad>, share: &WallEdgeShare) {
    let SupportMemberId::Primary(elem) = share.support else {
        return;
    };
    let Some(element) = model.element(elem) else {
        return;
    };
    if element.nodes.len() != 2 {
        return;
    }
    let Some((a, b)) = model.support_member_axis(share.support) else {
        return;
    };
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let member_len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    let loaded_len = (share.span[1] - share.span[0]).abs() * member_len;
    if loaded_len <= 1e-9 || share.total.abs() <= 1e-9 {
        return;
    }
    let w = share.total / loaded_len;
    loads.push(BeamLoad {
        elem,
        target: LoadTarget::Span {
            nodes: [element.nodes[0], element.nodes[1]],
            t: share.span,
        },
        shape: LoadShape::Uniform { w },
        cmq: fem_uniform(w, loaded_len),
    });
}

/// 壁版と二次部材の自重を支持先へ伝え、地震用節点重量 [N] に加算する（設計重量）。
/// 未指定・支持欠落・循環があれば加算前にエラーを返す。
pub fn accumulate_wall_and_secondary_seismic_weight(
    model: &Model,
    node_weight: &mut [f64],
) -> Result<(), String> {
    accumulate_wall_and_secondary_with_basis(
        model,
        node_weight,
        crate::cascade::SelfWeightBasis::Design,
    )
}

/// 壁版と二次部材の自重を支持先へ伝え、物理質量相当の節点重量 [N] に加算する。
/// 支持経路・端部負担率は設計重量版と同じで、二次部材の自重だけを物理密度で扱う
/// （壁版はコンクリートで設計＝物理のため値は変わらない）。
pub fn accumulate_wall_and_secondary_mass_equiv(
    model: &Model,
    node_mass: &mut [f64],
) -> Result<(), String> {
    accumulate_wall_and_secondary_with_basis(
        model,
        node_mass,
        crate::cascade::SelfWeightBasis::MassEquiv,
    )
}

fn accumulate_wall_and_secondary_with_basis(
    model: &Model,
    node_weight: &mut [f64],
    basis: crate::cascade::SelfWeightBasis,
) -> Result<(), String> {
    if !wall_plates_without_load_path(model).is_empty() {
        return Err("壁版の自重支持辺が未指定・不正、または支持先へ荷重を伝えられません".into());
    }
    let transfer = crate::cascade::solve_with_basis(model, |_| 0.0, true, basis);
    if !transfer.invalid_end_shares.is_empty()
        || !transfer.unresolved.is_empty()
        || !transfer.cyclic.is_empty()
    {
        return Err("二次部材の端部負担率または支持先が不正で、自重を伝えられません".into());
    }
    for plate in &model.wall_plates {
        for share in edge_shares_with(model, plate, basis)
            .into_iter()
            .filter(|s| s.post().is_none())
        {
            let SupportMemberId::Primary(elem) = share.support else {
                continue;
            };
            let Some(element) = model.element(elem) else {
                continue;
            };
            if element.nodes.len() != 2 {
                continue;
            }
            let len = model.member_length(element);
            let s0 = share.span[0] * len;
            let s1 = share.span[1] * len;
            let (lo, hi) = (s0.min(s1), s0.max(s1));
            if hi - lo <= 1e-9 {
                continue;
            }
            let w = share.total / (hi - lo);
            let load = MemberLoadKind::Distributed {
                a: lo,
                b: hi,
                w1: w,
                w2: w,
            };
            let (ri, rj) = crate::floor::simple_reactions(&load, len);
            node_weight[element.nodes[0].index()] += ri;
            node_weight[element.nodes[1].index()] += rj;
        }
    }
    let (nodal, member) = transfer.primary_loads(model);
    for (node, weight) in nodal {
        node_weight[node.index()] += weight;
    }
    for load in member {
        let Some(elem) = model.element(load.elem) else {
            continue;
        };
        let LoadShape::Point { p, x } = load.shape else {
            continue;
        };
        let (ri, rj) = crate::floor::simple_reactions(
            &MemberLoadKind::Point { a: x, p },
            model.member_length(elem),
        );
        node_weight[elem.nodes[0].index()] += ri;
        node_weight[elem.nodes[1].index()] += rj;
    }
    Ok(())
}

/// 自重を持つ非要素の囲まれた壁版のうち、支持先へ伝達できないものを返す。負担率の
/// 不備・割当領域の境界欠落・正の負担率の辺がスリットで切れている場合のみを判定し、
/// 支持部材の実在・種別・材軸解決・材端節点が 2 つであることは `Model::validate` が
/// 保証する前提とする。
/// 上下の梁際をともに切った納まりの可否はここでは判定せず、解析前チェックが入力方針として扱う。
pub fn wall_plates_without_load_path(model: &Model) -> Vec<WallPlateId> {
    model
        .wall_plates
        .iter()
        .filter(|plate| {
            if !matches!(plate.shape, WallPlateShape::Enclosed)
                || model.wall_plate_becomes_element(plate)
            {
                return false;
            }
            model
                .wall_plate_self_weight(plate, model)
                .is_some_and(|w| w > 0.0)
                && edge_shares_with(model, plate, crate::cascade::SelfWeightBasis::Design)
                    .is_empty()
        })
        .map(|plate| plate.id)
        .collect()
}

#[cfg(test)]
mod tests;
