//! スラブ面荷重の大梁・小梁・柱への分配。
//!
//! 責務ごとにサブモジュールへ分割している:
//! - [`types`] — 基本型（[`LoadShape`]・[`Cmq`]・[`LoadTarget`]・[`BeamLoad`]）と辺荷重ヘルパ
//! - [`geometry`] — 幾何ヘルパ（座標取得・距離・矩形判定）
//! - [`fem`] — 固定端モーメント・せん断（CMQ）の閉形式公式
//! - [`rect`] — 矩形床の分配戦略（三角形・台形／一方向／負担面積／小梁二段階）
//! - [`cantilever`] — 片持ちスラブ・出隅スラブの分配戦略
//! - [`polygon`] — 多角形床の負担面積法（最近接辺グリッドサンプリング）
//! - [`rigid_zone`] — 剛域を考慮した大梁 CMQ（[`cmq_with_rigid_zone`]）
//!
//! 本モジュールにはこれらを束ねるディスパッチャ [`distribute_slab`] と、床領域単位の
//! 束ね役 [`distribute_region`] を置く。
//!
//! # 床領域と床板の分配
//!
//! 床領域（大梁の 1 スパン区画）は、床領域内が小梁でさらに細かい打設単位に分かれていれば
//! 複数の床板（[`Slab`]）を持つ。[`distribute_region`] は床領域内の各床板を**独立に**
//! [`distribute_slab_w`] へ渡し、各辺荷重を床板割当領域の境界に従って実支持部材へ解決する
//! （[`resolve_edges_to_span`]）。主架構の辺は `LoadTarget::Span`、二次部材の辺は
//! `LoadTarget::Secondary` となり、二次部材が受け持った荷重は逐次伝達（[`crate::cascade`]）が
//! 両端反力へ変換して主架構へ渡す。支持先を解決できない辺は `LoadTarget::Edge` のまま残り、
//! 呼び出し側（`squid-n-job::auto_loads`）が捨てる。

mod cantilever;
mod fem;
mod geometry;
mod joist_design;
mod polygon;
mod rect;
mod rigid_zone;
mod types;

/// 取り付く壁版（[`crate::wall_attached`]）が、取付き線に載る等分布荷重の CMQ を
/// 床側と同じ式で求めるための再公開（`fem` 自体は非公開モジュール）。
pub(crate) use fem::{fem_linear, fem_uniform};
pub use fem::{fixed_end_moments, simple_beam_moment_at, simple_reactions};
pub use geometry::{point_in_slab_boundary, slab_dimensions, slab_dimensions_of};
pub use joist_design::{
    cantilever_extremes, covered_length_of_loads, flip_member_loads, joist_distribution_is_ready,
    joist_distribution_is_sufficient, joist_expected_slabs_covered, joist_self_weight_udl,
    load_shape_to_member_loads, orient_member_loads, secondary_joist_distribution_gaps,
    secondary_joist_distribution_loads, secondary_joist_distribution_split,
    secondary_joists_missing_distribution, simple_beam_extremes, span_node_key, BeamExtremes,
    SecondaryJoistDistributionGaps, JOIST_COVER_MIN_RATIO, JOIST_DEFLECTION_SAMPLE_DIVISIONS,
    JOIST_FORCE_SAMPLE_DIVISIONS,
};
pub use rigid_zone::{cmq_with_rigid_zone, RigidZoneCmqMode, RigidZoneCmqResult};
pub use types::{BeamLoad, Cmq, LoadShape, LoadTarget};

use cantilever::{distribute_cantilever, distribute_to_node};
use geometry::boundary_coords;
use polygon::distribute_polygon;
use rect::distribute_rect;
use squid_n_core::model::{
    FloorRegion, LoadTransfer, Model, RegionAnchor, Slab, SlabShape, SupportMemberId,
};

#[cfg(test)]
use fem::{fem_trapezoid, fem_triangle};

/// 床板の面荷重を境界（および二次部材経由の節点荷重）へ分配する。
///
/// 分岐は床板の形で決まる:
///
/// 1. **取り付く床板**（[`SlabShape::Attached`]）→ [`distribute_attached`]。
///    - 取付き先が点（出隅）: 全荷重をその節点（柱）へ集中する。荷重伝達方向にも
///      片持ち梁の取付きにも依らない（出隅の片持ちスラブの床荷重分配）。
///    - 取付き先が線 ＋ [`LoadTransfer::Anchor`]: 全長を覆う支持部材（大梁・片持ち梁・
///      小梁）がある境界辺を支持辺とし、最近接支持辺の負担面積で分配する
///      （[`distribute_cantilever`]）。支持辺が取付き線だけなら取付き辺へ等分布する。
///    - 取付き先が線 ＋ [`LoadTransfer::Columns`]: 取付き線の区間中点（無次元位置
///      `t_mid = (t_i+t_j)/2`）に集中したとみなし、単純梁の反力公式で両端の柱へ按分する
///      （全長 `[0, 1]` なら `t_mid = 0.5` で半分ずつ）。
/// 2. **大梁または小梁で囲まれた床板**（[`SlabShape::Enclosed`]）
///    - 境界が矩形（[`slab_dimensions`] が `Some` を返す）→ 矩形床の分配
///      （[`distribute_rect`]）。一方向の指定があればその方向（全体座標 X/Y）へ、
///      なければ境界辺 0・2 が負担する。
///    - それ以外（三角形・台形・五角形などの多角形）→ 多角形の負担面積法
///      （[`distribute_polygon`]）。一方向の指定があってもこの経路へ落ちる。
///
/// いずれの経路も総和保存（Σ大梁荷重 (+Σ小梁反力・Σ柱集中荷重) = w×面積）を満たすよう
/// 設計している（床は全体座標 XY 平面内（Z一定）にあることを仮定する）。
/// L 形の取り付く床板は取付き線ごとの複数の床板で表す。
pub fn distribute_slab(model: &Model, slab: &Slab) -> Vec<BeamLoad> {
    // 固定荷重 DL（版の自重＋仕上げ等）の総和を分配する。自重は断面の板厚と
    // 材料から算定する（`Model::slab_dead_intensity`）。
    distribute_slab_w(model, slab, model.slab_dead_intensity(slab))
}

/// 指定した面荷重強度 `w`（N/mm²）のみを床板の境界へ分配する。
///
/// 分岐ロジックは [`distribute_slab`] と同一で、荷重源だけを引数 `w` に差し替える。
/// これにより DL（固定荷重）と LL（積載荷重）を別々の荷重ケースへ分配できる
/// （令85条1項の床用/骨組用/地震用の使い分けや、荷重組合せでの DL/LL 係数分けに用いる）。
/// `w == 0.0` の場合は空の分配結果を返す。
pub fn distribute_slab_w(model: &Model, slab: &Slab, w: f64) -> Vec<BeamLoad> {
    let mut loads = Vec::new();
    if w == 0.0 {
        return loads;
    }
    let Some(coords) = boundary_coords(model, slab) else {
        return loads;
    };
    if coords.len() < 3 {
        return loads;
    }

    match &slab.shape {
        SlabShape::Attached { anchor, .. } => {
            distribute_attached(model, &coords, w, *anchor, &mut loads);
            return loads;
        }
        SlabShape::Enclosed => {}
    }

    match slab_dimensions_of(&coords) {
        Some((lx, ly)) => distribute_rect(slab, &coords, lx, ly, w, &mut loads),
        None => distribute_polygon(&coords, w, &mut loads),
    }

    loads
}

/// 取り付く床板（片持ちスラブ・バルコニー・出隅）の分配。
///
/// - 取付き先が点（出隅）: 全荷重をその節点（柱）へ集中する。
/// - 取付き先が線 ＋ [`LoadTransfer::Anchor`]: 全長を覆う支持部材がある辺（取付き線を
///   含む）を支持辺として最近接支持辺の負担面積で分配する。支持部材の辺がなければ
///   取付き辺へ等分布する（[`distribute_cantilever`]）。
/// - 取付き先が線 ＋ [`LoadTransfer::Columns`]: 取付き線の区間中点（無次元位置
///   `t_mid = (t_i+t_j)/2`）に集中したとみなし、単純梁の集中荷重反力公式
///   （`R0 = W(1-t_mid)`、`R1 = W・t_mid`）で両端の柱へ按分する。全長
///   （`span = [0, 1]`）なら `t_mid = 0.5` で半分ずつになる。**この按分は
///   `extent[0] == extent[1]`（張り出し量が区間の両端で等しい）のときに限り
///   厳密である。** 張り出し量が異なる場合、真の面積重心は区間中点から張り出しの
///   大きい側へずれるが、その差は見ていない。
fn distribute_attached(
    model: &Model,
    coords: &[[f64; 3]],
    w: f64,
    anchor: RegionAnchor,
    loads: &mut Vec<BeamLoad>,
) {
    match anchor {
        RegionAnchor::Point(node) => distribute_to_node(node, coords, w, 1.0, loads),
        RegionAnchor::Line {
            nodes,
            span,
            transfer,
        } => match transfer {
            LoadTransfer::Anchor => distribute_cantilever(model, coords, w, loads),
            LoadTransfer::Columns => {
                // 部分区間（span != [0, 1]）では、総荷重の作用点は取付き線上の区間中点
                // （無次元位置 t_mid）にある。単純梁の集中荷重の反力公式
                // （R0 = P(1-t), R1 = P・t）で両端の柱へ按分する。全長（t_mid=0.5）では
                // 従来どおり半分ずつになる。
                let t_mid = 0.5 * (span[0] + span[1]);
                distribute_to_node(nodes[0], coords, w, 1.0 - t_mid, loads);
                distribute_to_node(nodes[1], coords, w, t_mid, loads);
            }
        },
        // 床板の取付き先には使わない（`RegionAnchor::FloorRegion` のドキュメント参照。
        // 壁側〔自立壁〕専用のアンカーであり、`Slab::shape` 経由では到達しない）。
        RegionAnchor::FloorRegion { .. } => {}
    }
}

/// 局所辺インデックス（`Edge(k)`）を、実支持部材への作用（`Span`/`Secondary`）へ解決する。
///
/// 取り付く床板の辺 0 は取付き先の無次元区間を `Span::t` へ引き継ぎ、囲まれた床板は
/// 床板割当領域の境界（[`squid_n_core::model::SupportBoundary`]）を正本として辺の支持部材と
/// 材軸区間を引く。解決できない辺は `Edge` のまま返し、`elem` は `Primary` の `Span` では
/// 実要素 ID、それ以外は呼び出し側が解決する番兵 `ElemId(u32::MAX)` とする。
fn resolve_edges_to_span(model: &Model, slab: &Slab, loads: Vec<BeamLoad>) -> Vec<BeamLoad> {
    let attached_anchor_t = match &slab.shape {
        SlabShape::Attached {
            anchor: RegionAnchor::Line { span, .. },
            ..
        } => Some(*span),
        _ => None,
    };
    let region = match slab.shape {
        SlabShape::Enclosed => model.slab_assignment_region(slab.id),
        SlabShape::Attached { .. } => None,
    };
    loads
        .into_iter()
        .map(|mut bl| {
            let LoadTarget::Edge(k) = bl.target else {
                return bl;
            };
            if let Some(anchor_t) = attached_anchor_t {
                let Some([n0, n1]) = slab.edge_nodes(model, k) else {
                    bl.elem = squid_n_core::ids::ElemId(u32::MAX);
                    return bl;
                };
                let t = if k == 0 { anchor_t } else { [0.0, 1.0] };
                bl.target = LoadTarget::Span { nodes: [n0, n1], t };
                bl.elem = squid_n_core::ids::ElemId(u32::MAX);
                return bl;
            }
            let Some(boundary) = region.and_then(|region| region.boundary.get(k)) else {
                bl.elem = squid_n_core::ids::ElemId(u32::MAX);
                return bl;
            };
            match boundary.support {
                SupportMemberId::Secondary(member) => {
                    bl.target = LoadTarget::Secondary {
                        member,
                        t: boundary.span,
                    };
                    bl.elem = squid_n_core::ids::ElemId(u32::MAX);
                }
                SupportMemberId::Primary(elem) => {
                    let Some(element) = model.element(elem) else {
                        bl.elem = squid_n_core::ids::ElemId(u32::MAX);
                        return bl;
                    };
                    if element.nodes.len() != 2 {
                        bl.elem = squid_n_core::ids::ElemId(u32::MAX);
                        return bl;
                    }
                    bl.target = LoadTarget::Span {
                        nodes: [element.nodes[0], element.nodes[1]],
                        t: boundary.span,
                    };
                    bl.elem = elem;
                }
            }
            bl
        })
        .collect()
}

/// [`distribute_slab_w`] の戻り値を [`resolve_edges_to_span`] で解決した版。
///
/// どの床領域からも参照されない床板（片持ち・バルコニー・出隅、または帰属先が
/// 見つからない浮き床板）を、床領域とは独立に分配する用途に使う
/// （`squid-n-job::auto_loads` 参照）。戻り値の `LoadTarget` は `Node`/`Span`/
/// `Secondary` と、支持先を解決できなかった辺の `Edge`。
pub fn distribute_slab_resolved(model: &Model, slab: &Slab, w: f64) -> Vec<BeamLoad> {
    resolve_edges_to_span(model, slab, distribute_slab_w(model, slab, w))
}

/// 床領域（大梁の 1 スパン区画）の面荷重を、床領域内の床板へ束ねて分配する。
///
/// 床領域内が小梁でさらに細かい打設単位に分かれていれば、各床板を独立に
/// [`distribute_slab_w`] へ渡す（[`Self`] のモジュールドキュメント参照）。
/// `w_of` は床板ごとの面荷重強度 [N/mm²] を返す関数（DL/LL を分けるため）。
/// 戻り値の `LoadTarget` は `Node`/`Span`/`Secondary` と、支持先を解決できなかった辺の
/// `Edge`（[`resolve_edges_to_span`]）。
pub fn distribute_region(
    model: &Model,
    region: &FloorRegion,
    w_of: impl Fn(&Slab) -> f64,
) -> Vec<BeamLoad> {
    let mut loads = Vec::new();
    for &sid in &region.slab_ids {
        let Some(slab) = model.slab(sid) else {
            continue;
        };
        let slab_loads = distribute_slab_w(model, slab, w_of(slab));
        loads.extend(resolve_edges_to_span(model, slab, slab_loads));
    }
    loads
}

#[cfg(test)]
mod tests;
