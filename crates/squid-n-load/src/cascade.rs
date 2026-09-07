//! 二次部材の反力の逐次伝達（申し送り「床領域・壁領域の再設計」§3.4）。
//!
//! 二次部材（小梁・間柱）は解析要素ではないため、受け持った荷重は単純梁の両端反力に
//! 変えて支持相手へ渡す。支持相手が主架構（大梁）なら、そこで終端して梁の中間集中荷重
//! （CMQ）になる。**支持相手が別の二次部材のときは、その相手の集中荷重として渡し、
//! 相手が主架構へ行き着くまで同じ操作を繰り返す。**
//!
//! 交点は常にピン受け・架けとする（剛接十字は扱わない）。受け側・架け側は幾何で決まる。
//! 反力の分配則は支点まわりのモーメントつり合いによる。鉛直な間柱はつり合いが退化する
//! ため両端へ 1/2 ずつとする（仮定。§3.4 F8 の残課題）。

use std::collections::{HashMap, HashSet};

use squid_n_core::geom::vec3::dist as dist3;
use squid_n_core::geom::MEMBER_AXIS_TOL_MM;
use squid_n_core::ids::NodeId;
use squid_n_core::model::{
    ElementKind, MemberLoadKind, Model, SecondaryMember, SecondaryMemberKind, Slab,
};

use squid_n_core::ids::SlabId;

use crate::floor::{
    joist_distribution_is_ready, joist_self_weight_udl, orient_member_loads,
    secondary_joist_distribution_split, simple_reactions, span_node_key, BeamLoad,
};

/// 二次部材 1 本の識別キー（両端節点の順不同対）。
///
/// 二次部材はグローバル ID を持たない（実体は床領域・壁領域または未割当リスト）ため、
/// 端点の節点対で識別する（`Model::validate` が種別＋端点の重複を拒否する）。
pub type SecondaryKey = (NodeId, NodeId);

/// 二次部材の端部が載る先。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SupportAt {
    /// 主架構（要素が接続する節点、または大梁のスパン上）。逐次伝達の終端。
    Primary,
    /// 別の二次部材の内部。逐次伝達を 1 段進める。`a` は受け側の材軸上の位置 [mm]
    /// （受け側の `nodes[0]` からの距離）。
    Secondary { key: SecondaryKey, a: f64 },
    /// どこにも載っていない。荷重の行き先がない（診断のエラー対象）。
    Unresolved,
}

/// 逐次伝達を解いた二次部材 1 本。
#[derive(Clone, Debug)]
pub struct TransferredMember {
    /// 両端節点（`SecondaryMember::nodes` と同じ順）。
    pub nodes: [NodeId; 2],
    /// 支持間距離 [mm]。
    pub span: f64,
    /// この部材が受け持つ全荷重（材軸局所。`nodes[0]` を原点とする）。
    /// 床分配の辺荷重・自重・架け側から渡された集中荷重の重ね合わせ。
    pub member_loads: Vec<MemberLoadKind>,
    /// 両端反力 [N]（下向きの荷重に対して正）。`nodes` と同じ並び。
    pub reactions: [f64; 2],
    /// 各端の支持相手。`nodes` と同じ並び。
    pub supports: [SupportAt; 2],
    /// 床分配が断面検定に足りているか（期待床板が揃い、載荷長さがスパンの半分以上。
    /// `crate::floor::joist_distribution_is_ready`）。分配を持たない二次部材
    /// （間柱・床板の境界に載らない小梁）は偽。
    pub distribution_ready: bool,
    /// 分配の代表床板（検定結果の帰属先。分配が無ければ `None`）。
    pub rep_slab_id: Option<SlabId>,
}

/// 逐次伝達の結果。
#[derive(Clone, Debug, Default)]
pub struct SecondaryTransfer {
    /// 二次部材ごとの結果。
    pub members: HashMap<SecondaryKey, TransferredMember>,
    /// 端部の行き先が決まらなかった二次部材（どの主架構にも二次部材にも載らない）。
    pub unresolved: Vec<SecondaryKey>,
    /// 支持関係が循環している二次部材（互いに載せ合う）。荷重を流せない。
    pub cyclic: Vec<SecondaryKey>,
    /// どの二次部材にも載らなかった床領域分配の辺荷重。呼び出し側はこれだけを主架構へ
    /// 解決する（二次部材が受け持ったぶんは反力として渡るため、そのまま載せると
    /// 二重計上になる）。
    pub leftover_region_loads: Vec<BeamLoad>,
}

impl SecondaryTransfer {
    /// 主架構へ渡す荷重（`(節点, 下向き荷重 [N])`）。
    ///
    /// 終端（[`SupportAt::Primary`]）の端部だけを返す。呼び出し側は節点荷重として
    /// 積み、[`crate::secondary::resolve_nodal_to_primary`] で大梁の中間集中荷重へ
    /// 変換する（節点が大梁のスパン途中にあるため）。
    pub fn primary_node_loads(&self) -> Vec<(NodeId, f64)> {
        let mut out = Vec::new();
        for m in self.members.values() {
            for k in 0..2 {
                if m.supports[k] == SupportAt::Primary && m.reactions[k].abs() > 1e-9 {
                    out.push((m.nodes[k], m.reactions[k]));
                }
            }
        }
        out.sort_by(|a, b| a.0 .0.cmp(&b.0 .0).then(a.1.total_cmp(&b.1)));
        out
    }
}

/// 二次部材の幾何（逐次伝達の作業用）。
struct Axis {
    key: SecondaryKey,
    nodes: [NodeId; 2],
    a: [f64; 3],
    b: [f64; 3],
    len: f64,
}

fn coord(model: &Model, id: NodeId) -> Option<[f64; 3]> {
    model.nodes.get(id.index()).map(|n| n.coord)
}

/// 2 節点 `Beam` 要素の端点対（順不同）の集合。二次部材が実部材化済みかを
/// 部材ごとの全要素走査なしで判定するために 1 回だけ構築する。
fn beam_endpoint_keys(model: &Model) -> HashSet<SecondaryKey> {
    model
        .elements
        .iter()
        .filter(|e| e.kind == ElementKind::Beam && e.nodes.len() == 2)
        .map(|e| span_node_key(e.nodes[0], e.nodes[1]))
        .collect()
}

/// 点 `p` の線分 `a`→`b` 上の位置 [mm]（始点からの距離）。材軸から `tol` を超えて
/// 離れている、または線分の外にある場合は `None`。
fn project_on_segment(p: [f64; 3], a: [f64; 3], b: [f64; 3], tol: f64) -> Option<f64> {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len = dist3(a, b);
    if len <= 1e-9 {
        return None;
    }
    let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let t = (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / (len * len);
    let s = t * len;
    if s < -tol || s > len + tol {
        return None;
    }
    let proj = [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]];
    (dist3(proj, p) <= tol).then(|| s.clamp(0.0, len))
}

/// 逐次伝達の対象となる二次部材の材軸を集める。
///
/// 実部材化済み（両端を持つ実 `Beam` がある）・退化（両端が同一・長さ 0）・
/// 節点が引けないものは対象外（解析要素として直接扱われる、または荷重を持てない）。
fn axes(model: &Model) -> Vec<Axis> {
    let materialized = beam_endpoint_keys(model);
    let mut out = Vec::new();
    for sm in model.joists().chain(model.posts()) {
        let (n0, n1) = (sm.nodes[0], sm.nodes[1]);
        if n0 == n1 || materialized.contains(&span_node_key(n0, n1)) {
            continue;
        }
        let (Some(a), Some(b)) = (coord(model, n0), coord(model, n1)) else {
            continue;
        };
        let len = dist3(a, b);
        if len <= 1e-9 {
            continue;
        }
        out.push(Axis {
            key: span_node_key(n0, n1),
            nodes: [n0, n1],
            a,
            b,
            len,
        });
    }
    out
}

/// 端部 `node`（座標 `p`）の支持相手を幾何から決める。
///
/// **主架構を優先する。** 端部が要素の接続する節点、または大梁のスパン上にあるなら、
/// その大梁が直接支持しているのだから、そこで終端する。10 mm 以内に並走する二次部材が
/// 大梁の荷重を奪わないようにするためでもある（`joist_design` の並走大梁優先と同じ考え）。
///
/// 主架構へ届かないときだけ、別の二次部材の**内部**に載っているかを見る。載っていれば
/// その二次部材が受け側である（§3.4 F4）。端点どうしが一致するだけの取り付き
/// （L 字・端部で集まる形）は、どちらも相手を支持しないため受け側にしない。
/// どちらでもなければ行き先無しとする。
fn support_of(
    self_key: SecondaryKey,
    node: NodeId,
    p: [f64; 3],
    axes: &[Axis],
    connected: &[bool],
    beams: &[crate::secondary::BeamSpanCandidate],
) -> SupportAt {
    if connected.get(node.index()).copied().unwrap_or(false)
        || crate::secondary::best_span_position(beams, p, MEMBER_AXIS_TOL_MM).is_some()
    {
        return SupportAt::Primary;
    }
    let mut best: Option<(SecondaryKey, f64, f64)> = None;
    for other in axes {
        if other.key == self_key {
            continue;
        }
        if other.nodes.contains(&node) {
            continue;
        }
        let Some(a) = project_on_segment(p, other.a, other.b, MEMBER_AXIS_TOL_MM) else {
            continue;
        };
        if a <= MEMBER_AXIS_TOL_MM || a >= other.len - MEMBER_AXIS_TOL_MM {
            continue;
        }
        let d = {
            let t = a / other.len;
            let proj = [
                other.a[0] + (other.b[0] - other.a[0]) * t,
                other.a[1] + (other.b[1] - other.a[1]) * t,
                other.a[2] + (other.b[2] - other.a[2]) * t,
            ];
            dist3(proj, p)
        };
        if best.map(|(_, _, bd)| d < bd).unwrap_or(true) {
            best = Some((other.key, a, d));
        }
    }
    match best {
        Some((key, a, _)) => SupportAt::Secondary { key, a },
        None => SupportAt::Unresolved,
    }
}

/// 節点を共有せず交差している二次部材の組を返す。
fn crossings(axes: &[Axis]) -> Vec<(SecondaryKey, SecondaryKey)> {
    let mut out = Vec::new();
    for (i, p) in axes.iter().enumerate() {
        for q in axes.iter().skip(i + 1) {
            if p.nodes.iter().any(|n| q.nodes.contains(n)) {
                continue; // 節点を共有する取り付きは交差ではない。
            }
            let touches = [
                (p.a, q.a, q.b, q.len),
                (p.b, q.a, q.b, q.len),
                (q.a, p.a, p.b, p.len),
                (q.b, p.a, p.b, p.len),
            ]
            .iter()
            .any(|(pt, a, b, _)| project_on_segment(*pt, *a, *b, MEMBER_AXIS_TOL_MM).is_some());
            if touches {
                continue;
            }
            if segments_cross(p, q) {
                out.push((p.key, q.key));
            }
        }
    }
    out
}

/// 2 本の材軸が、どちらの端点でもない位置で交わるか（3 次元。ねじれの位置は交差としない）。
fn segments_cross(p: &Axis, q: &Axis) -> bool {
    let u = [
        (p.b[0] - p.a[0]) / p.len,
        (p.b[1] - p.a[1]) / p.len,
        (p.b[2] - p.a[2]) / p.len,
    ];
    let v = [
        (q.b[0] - q.a[0]) / q.len,
        (q.b[1] - q.a[1]) / q.len,
        (q.b[2] - q.a[2]) / q.len,
    ];
    let w = [q.a[0] - p.a[0], q.a[1] - p.a[1], q.a[2] - p.a[2]];
    let uv = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    let den = 1.0 - uv * uv;
    if den.abs() < 1e-9 {
        return false;
    }
    let wu = w[0] * u[0] + w[1] * u[1] + w[2] * u[2];
    let wv = w[0] * v[0] + w[1] * v[1] + w[2] * v[2];
    let s = (wu - uv * wv) / den;
    let t = (uv * wu - wv) / den;
    let tol = MEMBER_AXIS_TOL_MM;
    if s <= tol || s >= p.len - tol || t <= tol || t >= q.len - tol {
        return false;
    }
    let cp = [p.a[0] + s * u[0], p.a[1] + s * u[1], p.a[2] + s * u[2]];
    let cq = [q.a[0] + t * v[0], q.a[1] + t * v[1], q.a[2] + t * v[2]];
    dist3(cp, cq) <= tol
}

/// 荷重 1 件が両端へ渡す**鉛直反力**（モジュールドキュメント「反力の分配則」参照）。
///
/// `horizontal` が真（部材が水平投影を持つ）なら単純梁の反力がそのまま鉛直反力になる。
/// 偽（鉛直材）ならモーメントのつり合いが退化して不静定になるため、両端へ 1/2 ずつとする。
///
/// **成分へ分けて混ぜてはならない。** 「材軸方向成分を 1/2 ずつ、直交成分を単純梁反力」と
/// して `|u_z|` で線形に混ぜると、総和は保存するが配分が誤る。水平投影 4000・鉛直 3000 の
/// 傾斜材に材軸上 1/5 の位置で集中荷重を載せた例では、厳密解 0.8W に対して 0.62W となり、
/// **載荷側の反力を 22.5% 過小評価する**（受け側の部材にとって危険側）。
fn reactions_of(load: &MemberLoadKind, span: f64, horizontal: bool) -> (f64, f64) {
    let (r_i, r_j) = simple_reactions(load, span);
    if horizontal {
        return (r_i, r_j);
    }
    let half = (r_i + r_j) / 2.0;
    (half, half)
}

/// 二次部材の反力の逐次伝達を解く。
///
/// `w_of` は床板ごとの面荷重強度 [N/mm²]（DL・LL を分けるため床板単位で渡す）。
/// `include_self_weight` が真のとき、二次部材自身の自重（ρ·A·g·鉄骨割増）を等分布として
/// 重ねる（積載荷重のケースでは偽にする）。
pub fn solve(
    model: &Model,
    w_of: impl Fn(&Slab) -> f64,
    include_self_weight: bool,
) -> SecondaryTransfer {
    let axes = axes(model);
    if axes.is_empty() {
        let (_, leftover) = secondary_joist_distribution_split(model, w_of);
        return SecondaryTransfer {
            leftover_region_loads: leftover,
            ..SecondaryTransfer::default()
        };
    }
    let connected = crate::secondary::node_connected_flags(model);
    let beams = crate::secondary::beam_span_candidates(model);

    let mut supports: HashMap<SecondaryKey, [SupportAt; 2]> = HashMap::new();
    for ax in &axes {
        let s0 = support_of(ax.key, ax.nodes[0], ax.a, &axes, &connected, &beams);
        let s1 = support_of(ax.key, ax.nodes[1], ax.b, &axes, &connected, &beams);
        supports.insert(ax.key, [s0, s1]);
    }

    let (distribution, leftover_region_loads) = secondary_joist_distribution_split(model, w_of);
    let wall_loads = if include_self_weight {
        crate::wall_plate_load::distribute_enclosed_wall_plates(model).posts
    } else {
        HashMap::new()
    };

    let by_key: HashMap<SecondaryKey, &SecondaryMember> = model
        .joists()
        .chain(model.posts())
        .map(|sm| (span_node_key(sm.nodes[0], sm.nodes[1]), sm))
        .collect();

    let mut ready: HashMap<SecondaryKey, (bool, Option<SlabId>)> = HashMap::new();
    let mut base: HashMap<SecondaryKey, Vec<MemberLoadKind>> = HashMap::new();
    for ax in &axes {
        let mut loads = Vec::new();
        if let Some(entry) = distribution.get(&ax.key) {
            loads.extend(orient_member_loads(
                &entry.member_loads,
                ax.len,
                entry.span_nodes,
                (ax.nodes[0], ax.nodes[1]),
            ));
            ready.insert(
                ax.key,
                (
                    joist_distribution_is_ready(entry, ax.len),
                    entry.rep_slab_id,
                ),
            );
        }
        if let Some(wall) = wall_loads.get(&ax.key) {
            loads.extend(orient_member_loads(
                &wall.member_loads,
                ax.len,
                (wall.span_nodes[0], wall.span_nodes[1]),
                (ax.nodes[0], ax.nodes[1]),
            ));
        }
        if include_self_weight {
            if let Some(sm) = by_key.get(&ax.key) {
                if let Some(w) = joist_self_weight_udl(model, sm) {
                    loads.push(MemberLoadKind::Distributed {
                        a: 0.0,
                        b: ax.len,
                        w1: w,
                        w2: w,
                    });
                }
            }
        }
        base.insert(ax.key, loads);
    }

    let (order, cyclic) = transfer_order(&axes, &supports);

    let mut members: HashMap<SecondaryKey, TransferredMember> = HashMap::new();
    let index: HashMap<SecondaryKey, &Axis> = axes.iter().map(|a| (a.key, a)).collect();
    let mut extra: HashMap<SecondaryKey, Vec<MemberLoadKind>> = HashMap::new();

    for key in &order {
        let Some(ax) = index.get(key) else { continue };
        let mut loads = base.remove(key).unwrap_or_default();
        loads.extend(extra.remove(key).unwrap_or_default());

        let horizontal = {
            let (dx, dy) = (ax.b[0] - ax.a[0], ax.b[1] - ax.a[1]);
            (dx * dx + dy * dy).sqrt() > MEMBER_AXIS_TOL_MM
        };
        let mut r = [0.0_f64; 2];
        for l in &loads {
            let (ri, rj) = reactions_of(l, ax.len, horizontal);
            r[0] += ri;
            r[1] += rj;
        }

        let sup = supports
            .get(key)
            .copied()
            .unwrap_or([SupportAt::Unresolved; 2]);
        for k in 0..2 {
            if let SupportAt::Secondary { key: onto, a } = sup[k] {
                if r[k].abs() > 1e-9 {
                    extra
                        .entry(onto)
                        .or_default()
                        .push(MemberLoadKind::Point { a, p: r[k] });
                }
            }
        }

        let (distribution_ready, rep_slab_id) = ready.get(key).copied().unwrap_or((false, None));
        members.insert(
            *key,
            TransferredMember {
                nodes: ax.nodes,
                span: ax.len,
                member_loads: loads,
                reactions: r,
                supports: sup,
                distribution_ready,
                rep_slab_id,
            },
        );
    }

    let mut unresolved: Vec<SecondaryKey> = members
        .values()
        .filter(|m| {
            (0..2).any(|k| m.supports[k] == SupportAt::Unresolved && m.reactions[k].abs() > 1e-9)
        })
        .map(|m| span_node_key(m.nodes[0], m.nodes[1]))
        .collect();
    unresolved.sort();

    SecondaryTransfer {
        members,
        unresolved,
        cyclic,
        leftover_region_loads,
    }
}

/// 節点を共有せず交差している二次部材の組（診断専用。§3.4 F5）。
///
/// 荷重の同期では使わないため [`solve`] からは外してある。`solve` は荷重ケースごとに
/// 呼ばれる（DL・LL 架構用・LL 地震用）のに対し、この走査は二次部材の本数の 2 乗を
/// 要するため、診断が要るときだけ払う。
pub fn secondary_crossings(model: &Model) -> Vec<(SecondaryKey, SecondaryKey)> {
    crossings(&axes(model))
}

/// キーから二次部材の実体を引く。
fn secondary_of(model: &Model, key: SecondaryKey) -> Option<&SecondaryMember> {
    model
        .joists()
        .chain(model.posts())
        .find(|sm| span_node_key(sm.nodes[0], sm.nodes[1]) == key)
}

/// 逐次伝達の順序（架け側 → 受け側）と、循環に含まれる二次部材を返す。
///
/// 受け側は架け側の反力を受け取ってから解く必要があるため、支持グラフ
/// （架け側 → 受け側）のトポロジカル順に解く。循環（互いに載せ合う）は
/// 荷重を流せないため順序から外し、診断へ回す。
fn transfer_order(
    axes: &[Axis],
    supports: &HashMap<SecondaryKey, [SupportAt; 2]>,
) -> (Vec<SecondaryKey>, Vec<SecondaryKey>) {
    let mut pending: HashMap<SecondaryKey, usize> = axes.iter().map(|a| (a.key, 0)).collect();
    let mut onto: HashMap<SecondaryKey, Vec<SecondaryKey>> = HashMap::new();
    for ax in axes {
        let Some(s) = supports.get(&ax.key) else {
            continue;
        };
        for e in s {
            if let SupportAt::Secondary { key, .. } = e {
                if pending.contains_key(key) {
                    *pending.get_mut(key).expect("入次数") += 1;
                    onto.entry(ax.key).or_default().push(*key);
                }
            }
        }
    }
    let mut keys: Vec<SecondaryKey> = axes.iter().map(|a| a.key).collect();
    keys.sort();
    let mut ready: Vec<SecondaryKey> = keys
        .iter()
        .copied()
        .filter(|k| pending.get(k).copied().unwrap_or(0) == 0)
        .collect();
    let mut order = Vec::new();
    let mut done: HashSet<SecondaryKey> = HashSet::new();
    while let Some(k) = ready.pop() {
        if !done.insert(k) {
            continue;
        }
        order.push(k);
        let mut next: Vec<SecondaryKey> = Vec::new();
        for r in onto.get(&k).cloned().unwrap_or_default() {
            let slot = pending.get_mut(&r).expect("入次数");
            *slot -= 1;
            if *slot == 0 {
                next.push(r);
            }
        }
        next.sort();
        ready.extend(next);
    }
    let cyclic: Vec<SecondaryKey> = keys.into_iter().filter(|k| !done.contains(k)).collect();
    (order, cyclic)
}

/// 種別を問わず二次部材を数える（診断のメッセージ用）。
pub fn secondary_kind_of(model: &Model, key: SecondaryKey) -> Option<SecondaryMemberKind> {
    secondary_of(model, key).map(|sm| sm.kind)
}

#[cfg(test)]
mod tests;
