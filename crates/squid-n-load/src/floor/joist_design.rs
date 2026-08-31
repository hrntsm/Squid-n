//! 床領域の分配結果（[`super::distribute_region`]）から領域内小梁の設計部材力を求める。
//!
//! 領域内小梁（[`squid_n_core::model::FloorRegion::secondary_joists`]）の断面検定は、
//! 幾何から負担幅を再導出せず、荷重分配と同じ `BeamLoad`（`LoadTarget::Span`）を
//! 単純梁として重ね合わせる。床板境界が途中節点で分割されていても、小梁材軸上に
//! 載る `Span` は全長へ合成する（T 字取り付きの片側欠落を防ぐ）。

use std::collections::{HashMap, HashSet};

use squid_n_core::geom::MEMBER_AXIS_TOL_MM;
use squid_n_core::ids::{NodeId, SlabId};
use squid_n_core::model::{ElementKind, MemberLoadKind, Model, SecondaryMemberKind, Slab};
use squid_n_core::units::GRAVITY_MM_S2;

use super::distribute_slab_resolved;
use super::fem::{simple_beam_moment_at, simple_reactions};
use super::types::{BeamLoad, LoadShape, LoadTarget};

/// 節点対を順不同キー `(min, max)` に正規化する。
pub fn span_node_key(a: NodeId, b: NodeId) -> (NodeId, NodeId) {
    if a.0 <= b.0 {
        (a, b)
    } else {
        (b, a)
    }
}

/// 分配結果 1 件の線荷重形状を、小梁スパン上の [`MemberLoadKind`] 列へ変換する。
///
/// `len` は支持間距離 [mm]。`flip` が true のとき、荷重作用位置をスパン終端から見た
/// 座標へ鏡映する（分配側の `Span.nodes` 順と二次部材の `nodes` 順が逆のとき）。
pub fn load_shape_to_member_loads(shape: &LoadShape, len: f64, flip: bool) -> Vec<MemberLoadKind> {
    if len <= 1e-9 {
        return Vec::new();
    }
    let mut out = match *shape {
        LoadShape::Uniform { w } => vec![MemberLoadKind::Distributed {
            a: 0.0,
            b: len,
            w1: w,
            w2: w,
        }],
        LoadShape::Linear { w_i, w_j } => vec![MemberLoadKind::Distributed {
            a: 0.0,
            b: len,
            w1: w_i,
            w2: w_j,
        }],
        LoadShape::Triangle { w0 } => {
            let mid = len / 2.0;
            vec![
                MemberLoadKind::Distributed {
                    a: 0.0,
                    b: mid,
                    w1: 0.0,
                    w2: w0,
                },
                MemberLoadKind::Distributed {
                    a: mid,
                    b: len,
                    w1: w0,
                    w2: 0.0,
                },
            ]
        }
        LoadShape::Trapezoid { w0, a, b } => {
            vec![
                MemberLoadKind::Distributed {
                    a: 0.0,
                    b: a,
                    w1: 0.0,
                    w2: w0,
                },
                MemberLoadKind::Distributed {
                    a,
                    b: a + b,
                    w1: w0,
                    w2: w0,
                },
                MemberLoadKind::Distributed {
                    a: a + b,
                    b: len,
                    w1: w0,
                    w2: 0.0,
                },
            ]
        }
        LoadShape::Point { p, x } => vec![MemberLoadKind::Point { a: x, p }],
    };
    if flip {
        out = out
            .into_iter()
            .map(|load| flip_member_load(&load, len))
            .collect();
    }
    out
}

fn flip_member_load(load: &MemberLoadKind, len: f64) -> MemberLoadKind {
    match *load {
        MemberLoadKind::Point { a, p } => MemberLoadKind::Point { a: len - a, p },
        MemberLoadKind::Distributed { a, b, w1, w2 } => MemberLoadKind::Distributed {
            a: len - b,
            b: len - a,
            w1: w2,
            w2: w1,
        },
    }
}

fn member_load_total(load: &MemberLoadKind) -> f64 {
    match *load {
        MemberLoadKind::Point { p, .. } => p,
        MemberLoadKind::Distributed { a, b, w1, w2 } => {
            if b <= a {
                0.0
            } else {
                (w1 + w2) / 2.0 * (b - a)
            }
        }
    }
}

fn load_intensity_up_to(load: &MemberLoadKind, x: f64) -> f64 {
    match *load {
        MemberLoadKind::Point { a, p } => {
            if x >= a {
                p
            } else {
                0.0
            }
        }
        MemberLoadKind::Distributed { a, b, w1, w2 } => {
            if b <= a || x <= a {
                return 0.0;
            }
            let x_eff = x.min(b);
            let m = (w2 - w1) / (b - a);
            let c = w1 - m * a;
            let integral_linear = |s0: f64, s1: f64| {
                let f = |s: f64| m * s * s / 2.0 + c * s;
                f(s1) - f(s0)
            };
            integral_linear(a, x_eff)
        }
    }
}

fn simple_beam_shear_at(loads: &[MemberLoadKind], x: f64, r_i: f64) -> f64 {
    let loaded: f64 = loads.iter().map(|l| load_intensity_up_to(l, x)).sum();
    r_i - loaded
}

/// 曲げ・せん断の最大値を探す分割数（スパンをこの数で等分。端点を含む）。
pub const JOIST_FORCE_SAMPLE_DIVISIONS: usize = 64;
/// たわみ積分・最大値探索の分割数。
pub const JOIST_DEFLECTION_SAMPLE_DIVISIONS: usize = 80;

/// 単純梁（両端ピン）の最大たわみ [mm]。
fn simple_beam_max_deflection(
    loads: &[MemberLoadKind],
    span: f64,
    young: f64,
    inertia: f64,
) -> f64 {
    let ei = young * inertia;
    if span <= 1e-9 || ei <= 1e-9 || loads.is_empty() {
        return 0.0;
    }
    const N: usize = JOIST_DEFLECTION_SAMPLE_DIVISIONS;
    let h = span / N as f64;
    let kappa: Vec<f64> = (0..=N)
        .map(|i| simple_beam_moment_at(loads, span, i as f64 * h) / ei)
        .collect();
    let mut integral_kappa_l = 0.0;
    for j in 0..N {
        integral_kappa_l += (kappa[j] + kappa[j + 1]) / 2.0 * (span - (j as f64 + 0.5) * h) * h;
    }
    let mut max_d = 0.0_f64;
    for i in 0..=N {
        let x = i as f64 * h;
        let mut area = 0.0;
        for j in 0..i {
            let s_mid = (j as f64 + 0.5) * h;
            let kap = (kappa[j] + kappa[j + 1]) / 2.0;
            area += kap * (x - s_mid) * h;
        }
        let d = area - x / span * integral_kappa_l;
        max_d = max_d.max(d.abs());
    }
    max_d
}

/// 単純梁の設計部材力（重ね合わせ）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimpleBeamExtremes {
    /// 最大曲げモーメント [N·mm]（絶対値）。
    pub m_max: f64,
    /// 最大せん断力 [N]（絶対値）。
    pub q_max: f64,
    /// 最大たわみ [mm]（絶対値）。
    pub deflection: f64,
    /// 表示用の等価等分布線荷重 [N/mm]（合計荷重 / スパン）。
    pub w_equiv: f64,
}

/// `loads` を単純梁として重ね合わせ、設計に用いる最大値を返す。
pub fn simple_beam_extremes(
    loads: &[MemberLoadKind],
    span: f64,
    young: f64,
    inertia: f64,
) -> SimpleBeamExtremes {
    if span <= 1e-9 || loads.is_empty() {
        return SimpleBeamExtremes {
            m_max: 0.0,
            q_max: 0.0,
            deflection: 0.0,
            w_equiv: 0.0,
        };
    }
    const N: usize = JOIST_FORCE_SAMPLE_DIVISIONS;
    let r_i: f64 = loads.iter().map(|l| simple_reactions(l, span).0).sum();
    let mut m_max = 0.0_f64;
    let mut q_max = 0.0_f64;
    for i in 0..=N {
        let x = span * i as f64 / N as f64;
        m_max = m_max.max(simple_beam_moment_at(loads, span, x).abs());
        q_max = q_max.max(simple_beam_shear_at(loads, x, r_i).abs());
    }
    let total: f64 = loads.iter().map(member_load_total).sum();
    SimpleBeamExtremes {
        m_max,
        q_max,
        deflection: simple_beam_max_deflection(loads, span, young, inertia),
        w_equiv: total / span,
    }
}

/// 二次部材小梁 1 本ぶんの分配荷重。
#[derive(Clone, Debug, PartialEq)]
pub struct SecondaryJoistLoads {
    /// 分配 `Span` を単純梁荷重へ変換した重ね合わせ。
    pub member_loads: Vec<MemberLoadKind>,
    /// 代表床板（所属床領域の `slab_ids` 先頭。無いときは `None`）。
    pub rep_slab_id: Option<SlabId>,
    /// このエントリの荷重を載せるときの小梁節点順（`SecondaryMember.nodes`）。
    pub span_nodes: (NodeId, NodeId),
    /// この材軸へ分配がその辺へ `Span` を出すと期待される床板。
    pub expected_slab_ids: HashSet<SlabId>,
    /// 重ね合わせに実際に載った床板。
    pub contributed_slab_ids: HashSet<SlabId>,
}

fn beam_between(model: &Model, a: NodeId, b: NodeId) -> bool {
    model.elements.iter().any(|e| {
        e.kind == ElementKind::Beam
            && e.nodes.len() == 2
            && ((e.nodes[0] == a && e.nodes[1] == b) || (e.nodes[0] == b && e.nodes[1] == a))
    })
}

fn coord(model: &Model, id: NodeId) -> Option<[f64; 3]> {
    model.nodes.get(id.index()).map(|n| n.coord)
}

fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn lerp3(a: [f64; 3], b: [f64; 3], t: f64) -> [f64; 3] {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

/// 点 `p` が線分 `a`–`b` の材軸から `tol` 以内なら、`a` からの材軸距離 [mm]。
fn project_on_segment(p: [f64; 3], a: [f64; 3], b: [f64; 3], tol: f64) -> Option<f64> {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len = dist3(a, b);
    if len <= 1e-9 {
        return None;
    }
    let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let s = (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / (len * len) * len;
    if s < -tol || s > len + tol {
        return None;
    }
    let t = (s / len).clamp(0.0, 1.0);
    let proj = lerp3(a, b, t);
    let d = dist3(p, proj);
    (d <= tol).then_some(s.clamp(0.0, len))
}

#[cfg(test)]
fn shift_member_load(load: &MemberLoadKind, s0: f64) -> MemberLoadKind {
    match *load {
        MemberLoadKind::Point { a, p } => MemberLoadKind::Point { a: a + s0, p },
        MemberLoadKind::Distributed { a, b, w1, w2 } => MemberLoadKind::Distributed {
            a: a + s0,
            b: b + s0,
            w1,
            w2,
        },
    }
}

/// 分配辺の局所長さ `loaded_len` 上の荷重を、材軸区間 `[s0, s1]` へ写す。
/// 合計力は保存する（長さが変われば線荷重強度を逆比でスケールする）。
fn map_loads_onto_axis(
    loads: &[MemberLoadKind],
    loaded_len: f64,
    s0: f64,
    s1: f64,
) -> Vec<MemberLoadKind> {
    if loaded_len <= 1e-9 {
        return Vec::new();
    }
    let axis_len = (s1 - s0).abs();
    let scale = axis_len / loaded_len;
    let w_scale = if axis_len > 1e-9 {
        loaded_len / axis_len
    } else {
        1.0
    };
    loads
        .iter()
        .map(|load| match *load {
            MemberLoadKind::Point { a, p } => MemberLoadKind::Point {
                a: s0 + a * scale,
                p,
            },
            MemberLoadKind::Distributed { a, b, w1, w2 } => MemberLoadKind::Distributed {
                a: s0 + a * scale,
                b: s0 + b * scale,
                w1: w1 * w_scale,
                w2: w2 * w_scale,
            },
        })
        .collect()
}

/// 分配荷重が材軸を覆う長さの合計 [mm]（重なりは結合）。集中荷重は区間 0。
pub fn covered_length_of_loads(loads: &[MemberLoadKind], span: f64) -> f64 {
    if span <= 1e-9 {
        return 0.0;
    }
    let mut ivs: Vec<(f64, f64)> = loads
        .iter()
        .filter_map(|l| match l {
            MemberLoadKind::Point { .. } => None,
            MemberLoadKind::Distributed { a, b, .. } => {
                let lo = (*a).min(*b).clamp(0.0, span);
                let hi = (*a).max(*b).clamp(0.0, span);
                (hi - lo > 1e-9).then_some((lo, hi))
            }
        })
        .collect();
    if ivs.is_empty() {
        return 0.0;
    }
    ivs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    let mut cover = 0.0;
    let (mut cur_lo, mut cur_hi) = ivs[0];
    for &(lo, hi) in ivs.iter().skip(1) {
        if lo <= cur_hi + 1e-6 {
            cur_hi = cur_hi.max(hi);
        } else {
            cover += cur_hi - cur_lo;
            cur_lo = lo;
            cur_hi = hi;
        }
    }
    cover + (cur_hi - cur_lo)
}

/// 片側欠落とみなすカバー率の下限（材軸長に対する載荷区間）。既定 0.5。
pub const JOIST_COVER_MIN_RATIO: f64 = 0.5;

/// 分配荷重が空でなく、材軸の半分以上を覆っていれば長さカバーは足りる。
pub fn joist_distribution_is_sufficient(loads: &[MemberLoadKind], span: f64) -> bool {
    !loads.is_empty()
        && span > 1e-9
        && covered_length_of_loads(loads, span) / span + 1e-9 >= JOIST_COVER_MIN_RATIO
}

/// 期待床板の寄与がすべて重ね合わせに載っているか。
pub fn joist_expected_slabs_covered(
    expected: &HashSet<SlabId>,
    contributed: &HashSet<SlabId>,
) -> bool {
    expected.iter().all(|id| contributed.contains(id))
}

/// 期待床板が空でなく、各床板の寄与があり、載荷長さも半分以上なら検定に使える。
pub fn joist_distribution_is_ready(entry: &SecondaryJoistLoads, span: f64) -> bool {
    !entry.expected_slab_ids.is_empty()
        && joist_expected_slabs_covered(&entry.expected_slab_ids, &entry.contributed_slab_ids)
        && joist_distribution_is_sufficient(&entry.member_loads, span)
}

fn nearest_beam_dist(model: &Model, p0: [f64; 3], p1: [f64; 3]) -> f64 {
    let mut best = f64::INFINITY;
    for e in &model.elements {
        if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
            continue;
        }
        let (Some(a), Some(b)) = (coord(model, e.nodes[0]), coord(model, e.nodes[1])) else {
            continue;
        };
        let d = point_dist_to_axis(p0, a, b).max(point_dist_to_axis(p1, a, b));
        if d < best {
            best = d;
        }
    }
    best
}

struct JoistAxis {
    key: (NodeId, NodeId),
    nodes: (NodeId, NodeId),
    a: [f64; 3],
    b: [f64; 3],
}

fn joist_axes(model: &Model) -> Vec<JoistAxis> {
    let mut out = Vec::new();
    for sm in model.joists() {
        if sm.kind != SecondaryMemberKind::Joist {
            continue;
        }
        let (n0, n1) = (sm.nodes[0], sm.nodes[1]);
        if n0 == n1 || beam_between(model, n0, n1) {
            continue;
        }
        let (Some(a), Some(b)) = (coord(model, n0), coord(model, n1)) else {
            continue;
        };
        let len = dist3(a, b);
        if len <= 1e-9 {
            continue;
        }
        out.push(JoistAxis {
            key: span_node_key(n0, n1),
            nodes: (n0, n1),
            a,
            b,
        });
    }
    out
}

fn segment_on_beam_only(model: &Model, p0: [f64; 3], p1: [f64; 3], axes: &[JoistAxis]) -> bool {
    // 小梁材軸上にあれば大梁並走でも落とさない（10 mm 以内の並走で欠落するのを防ぐ）。
    let on_joist = axes.iter().any(|axis| {
        project_on_segment(p0, axis.a, axis.b, MEMBER_AXIS_TOL_MM).is_some()
            && project_on_segment(p1, axis.a, axis.b, MEMBER_AXIS_TOL_MM).is_some()
    });
    if on_joist {
        return false;
    }
    model.elements.iter().any(|e| {
        if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
            return false;
        }
        let (Some(a), Some(b)) = (coord(model, e.nodes[0]), coord(model, e.nodes[1])) else {
            return false;
        };
        project_on_segment(p0, a, b, MEMBER_AXIS_TOL_MM).is_some()
            && project_on_segment(p1, a, b, MEMBER_AXIS_TOL_MM).is_some()
    })
}

fn point_dist_to_axis(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len = dist3(a, b);
    if len <= 1e-9 {
        return dist3(p, a);
    }
    let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let t = ((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / (len * len)).clamp(0.0, 1.0);
    dist3(p, lerp3(a, b, t))
}

/// 二次部材小梁の自重を単純梁の等分布荷重 [N/mm] として返す。
///
/// 自重算定（`enumerate_self_weight`）と同じ ρ·A·g·鉄骨割増。断面または材料が
/// 無ければ `None`（その小梁の検定荷重には自重を足さない）。
pub fn joist_self_weight_udl(
    model: &Model,
    sm: &squid_n_core::model::SecondaryMember,
) -> Option<f64> {
    let mat = model.secondary_material(sm)?;
    let sec = model.sections.get(sm.section?.index())?;
    let factor = if mat.fc.is_some() {
        1.0
    } else {
        model
            .load_cfg
            .as_ref()
            .map(|c| c.effective_steel_factor())
            .unwrap_or(1.0)
    };
    let w = mat.density * sec.area * GRAVITY_MM_S2 * factor;
    (w > 0.0).then_some(w)
}

/// 全床領域の [`distribute_region`] 出力から、実部材化していない小梁へ荷重を載せる。
///
/// 床板境界の `Span` は、両端が小梁材軸上にあれば（節点対の完全一致を要求しない）
/// その小梁の局所座標へ写して重ねる。部分区間 `t` も同じ。大梁材軸上の `Span` は除外する
/// （ただし同じ位置に小梁がある場合は小梁側を優先する）。
/// 1 本の `Span` が複数の小梁材軸に載りうる場合は、端点の材軸距離が最小の 1 本だけに載せる。
///
/// 期待床板が 0 枚の材軸には載せない（並走・未割当への奪取を防ぐ）。期待床板は、
/// 境界が材軸に載り、かつその床板の分配がその辺へ `Span` を出す囲まれ床板である。
pub fn secondary_joist_distribution_loads(
    model: &Model,
    w_of: impl Fn(&Slab) -> f64,
) -> HashMap<(NodeId, NodeId), SecondaryJoistLoads> {
    let axes = joist_axes(model);
    let tagged = tagged_span_loads(model, &w_of);
    let mut expected: HashMap<(NodeId, NodeId), HashSet<SlabId>> = HashMap::new();
    for (slab_id, bl) in &tagged {
        let Some((p0, p1, loaded_len)) = span_points(model, bl) else {
            continue;
        };
        if loaded_len <= 1e-9 {
            continue;
        }
        for axis in &axes {
            if !span_belongs_to_axis(model, p0, p1, axis) {
                continue;
            }
            if !slab_in_joist_scope(model, axis.key, *slab_id) {
                continue;
            }
            expected.entry(axis.key).or_default().insert(*slab_id);
        }
    }

    let candidates: Vec<&JoistAxis> = axes
        .iter()
        .filter(|axis| expected.get(&axis.key).is_some_and(|s| !s.is_empty()))
        .collect();

    let mut map: HashMap<(NodeId, NodeId), SecondaryJoistLoads> = HashMap::new();
    for axis in &candidates {
        let expected_slab_ids = expected.get(&axis.key).cloned().unwrap_or_default();
        let rep_slab_id = expected_slab_ids.iter().copied().min_by_key(|id| id.0);
        map.insert(
            axis.key,
            SecondaryJoistLoads {
                member_loads: Vec::new(),
                rep_slab_id,
                span_nodes: axis.nodes,
                expected_slab_ids,
                contributed_slab_ids: HashSet::new(),
            },
        );
    }

    for (slab_id, bl) in &tagged {
        let Some((p0, p1, loaded_len)) = span_points(model, bl) else {
            continue;
        };
        if loaded_len <= 1e-9 {
            continue;
        }
        if segment_on_beam_only(model, p0, p1, &axes) {
            continue;
        }
        let shape_loads = load_shape_to_member_loads(&bl.shape, loaded_len, false);

        let mut best: Option<(usize, f64, f64, f64, Vec<MemberLoadKind>)> = None;
        for (ai, axis) in candidates.iter().enumerate() {
            let Some(mut s0) = project_on_segment(p0, axis.a, axis.b, MEMBER_AXIS_TOL_MM) else {
                continue;
            };
            let Some(mut s1) = project_on_segment(p1, axis.a, axis.b, MEMBER_AXIS_TOL_MM) else {
                continue;
            };
            let mut piece = shape_loads.clone();
            if s1 + 1e-9 < s0 {
                piece = piece
                    .iter()
                    .map(|l| flip_member_load(l, loaded_len))
                    .collect();
                std::mem::swap(&mut s0, &mut s1);
            }
            let d =
                point_dist_to_axis(p0, axis.a, axis.b).max(point_dist_to_axis(p1, axis.a, axis.b));
            let replace = match best {
                None => true,
                Some((_, best_d, ..)) => d + 1e-9 < best_d,
            };
            if replace {
                best = Some((ai, d, s0, s1, piece));
            }
        }
        let Some((ai, joist_d, s0, s1, piece)) = best else {
            continue;
        };
        let beam_d = nearest_beam_dist(model, p0, p1);
        if beam_d + 1e-9 < joist_d {
            continue;
        }
        let axis = candidates[ai];
        let mapped = map_loads_onto_axis(&piece, loaded_len, s0, s1);
        let entry = map.entry(axis.key).or_insert_with(|| SecondaryJoistLoads {
            member_loads: Vec::new(),
            rep_slab_id: Some(*slab_id),
            span_nodes: axis.nodes,
            expected_slab_ids: expected.get(&axis.key).cloned().unwrap_or_default(),
            contributed_slab_ids: HashSet::new(),
        });
        entry.member_loads.extend(mapped);
        entry.contributed_slab_ids.insert(*slab_id);
        if entry.rep_slab_id.is_none() {
            entry.rep_slab_id = Some(*slab_id);
        }
    }
    map
}

fn span_points(model: &Model, bl: &BeamLoad) -> Option<([f64; 3], [f64; 3], f64)> {
    let LoadTarget::Span { nodes: [n0, n1], t } = bl.target else {
        return None;
    };
    let (c0, c1) = (coord(model, n0)?, coord(model, n1)?);
    let p0 = lerp3(c0, c1, t[0]);
    let p1 = lerp3(c0, c1, t[1]);
    let loaded_len = dist3(p0, p1);
    Some((p0, p1, loaded_len))
}

fn span_belongs_to_axis(model: &Model, p0: [f64; 3], p1: [f64; 3], axis: &JoistAxis) -> bool {
    if project_on_segment(p0, axis.a, axis.b, MEMBER_AXIS_TOL_MM).is_none()
        || project_on_segment(p1, axis.a, axis.b, MEMBER_AXIS_TOL_MM).is_none()
    {
        return false;
    }
    let joist_d =
        point_dist_to_axis(p0, axis.a, axis.b).max(point_dist_to_axis(p1, axis.a, axis.b));
    let beam_d = nearest_beam_dist(model, p0, p1);
    beam_d + 1e-9 >= joist_d
}

fn tagged_span_loads(model: &Model, w_of: &impl Fn(&Slab) -> f64) -> Vec<(SlabId, BeamLoad)> {
    let mut out = Vec::new();
    for region in &model.floor_regions {
        for &sid in &region.slab_ids {
            let Some(slab) = model.slab(sid) else {
                continue;
            };
            for bl in distribute_slab_resolved(model, slab, w_of(slab)) {
                out.push((slab.id, bl));
            }
        }
    }
    out
}

fn slab_in_joist_scope(model: &Model, joist_key: (NodeId, NodeId), slab_id: SlabId) -> bool {
    for region in &model.floor_regions {
        if region
            .secondary_joists
            .iter()
            .any(|j| span_node_key(j.nodes[0], j.nodes[1]) == joist_key)
        {
            return region.slab_ids.contains(&slab_id);
        }
    }
    true
}

/// `member_loads` を二次部材の節点順 `(a, b)` に合わせて向きを揃える。
pub fn orient_member_loads(
    loads: &[MemberLoadKind],
    span_len: f64,
    distribution_nodes: (NodeId, NodeId),
    joist_nodes: (NodeId, NodeId),
) -> Vec<MemberLoadKind> {
    let flip = distribution_nodes != joist_nodes;
    if flip {
        loads
            .iter()
            .map(|l| flip_member_load(l, span_len))
            .collect()
    } else {
        loads.to_vec()
    }
}

/// 床領域分配から荷重が得られず、断面検定対象から外れる二次部材小梁の本数。
///
/// 実部材化済み・断面未割当・退化は数えない。
pub fn secondary_joists_missing_distribution(model: &Model) -> usize {
    secondary_joist_distribution_gaps(model).total()
}

/// 二次部材小梁の分配欠落を理由別に数える（診断用。1 本は最も具体的な理由へ排他集計）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SecondaryJoistDistributionGaps {
    /// 期待床板の寄与が欠けている本数。
    pub missing_expected_slabs: usize,
    /// 期待床板は揃っているが載荷長さが半分未満の本数。
    pub short_cover: usize,
    /// 分配荷重が無い、または期待床板が 0 枚の本数。
    pub no_distribution: usize,
}

impl SecondaryJoistDistributionGaps {
    /// 断面検定しない二次部材小梁の合計本数。
    pub fn total(self) -> usize {
        self.missing_expected_slabs + self.short_cover + self.no_distribution
    }
}

/// [`secondary_joists_missing_distribution`] の内訳。
pub fn secondary_joist_distribution_gaps(model: &Model) -> SecondaryJoistDistributionGaps {
    use squid_n_core::model::{LoadPurpose, SecondaryMemberKind};

    let w_of = |s: &Slab| model.slab_intensity(s, LoadPurpose::Floor);
    let distribution = secondary_joist_distribution_loads(model, w_of);

    let mut gaps = SecondaryJoistDistributionGaps::default();
    for sm in model.joists() {
        if sm.kind != SecondaryMemberKind::Joist {
            continue;
        }
        if sm.section.is_none() {
            continue;
        }
        let (a, b) = (sm.nodes[0], sm.nodes[1]);
        if a == b || beam_between(model, a, b) {
            continue;
        }
        let key = span_node_key(a, b);
        let (Some(na), Some(nb)) = (coord(model, a), coord(model, b)) else {
            gaps.no_distribution += 1;
            continue;
        };
        let span = dist3(na, nb);
        match distribution.get(&key) {
            Some(e)
                if !joist_expected_slabs_covered(&e.expected_slab_ids, &e.contributed_slab_ids) =>
            {
                gaps.missing_expected_slabs += 1;
            }
            Some(e) if joist_distribution_is_sufficient(&e.member_loads, span) => {}
            Some(e) if !e.member_loads.is_empty() => {
                gaps.short_cover += 1;
            }
            _ => {
                gaps.no_distribution += 1;
            }
        }
    }
    gaps
}

#[cfg(test)]
mod tests {
    use super::super::types::LoadShape;
    use super::*;
    use squid_n_core::ids::{ElemId, FloorRegionId};
    use squid_n_core::model::{
        AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, FloorRegion,
        ForceRegime, LocalAxis, MemberLoadKind, Node, SecondaryMember, SecondaryMemberKind, Slab,
        SlabPlate, SlabShape,
    };

    fn square_model_with_shared_joist() -> Model {
        let mk_node = |id: u32, x: f64, y: f64| Node {
            id: NodeId(id),
            coord: [x, y, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        };
        let nodes = vec![
            mk_node(0, 0.0, 0.0),
            mk_node(1, 4000.0, 0.0),
            mk_node(2, 4000.0, 4000.0),
            mk_node(3, 0.0, 4000.0),
            mk_node(4, 2000.0, 0.0),
            mk_node(5, 2000.0, 4000.0),
        ];
        let mk_beam = |id: u32, i: u32, j: u32| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        let elements = vec![
            mk_beam(0, 0, 1),
            mk_beam(1, 1, 2),
            mk_beam(2, 2, 3),
            mk_beam(3, 3, 0),
        ];
        let plate = SlabPlate {
            section: None,
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: 0.005,
            }],
            usage: None,
            method: DistributionMethod::TriTrapezoid,
            one_way: None,
        };
        let slabs = vec![
            Slab {
                id: SlabId(0),
                shape: SlabShape::Enclosed {
                    boundary: vec![NodeId(0), NodeId(4), NodeId(5), NodeId(3)],
                },
                plate: plate.clone(),
            },
            Slab {
                id: SlabId(1),
                shape: SlabShape::Enclosed {
                    boundary: vec![NodeId(4), NodeId(1), NodeId(2), NodeId(5)],
                },
                plate,
            },
        ];
        let mut region = FloorRegion::new(
            FloorRegionId(0),
            vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        );
        region.slab_ids = vec![SlabId(0), SlabId(1)];
        region.secondary_joists = vec![SecondaryMember {
            kind: SecondaryMemberKind::Joist,
            nodes: [NodeId(4), NodeId(5)],
            section: None,
            name: "J".into(),
        }];
        Model {
            nodes,
            elements,
            floor_regions: vec![region],
            slabs,
            ..Default::default()
        }
    }

    #[test]
    fn distribution_loads_on_shared_joist_match_average_width() {
        let model = square_model_with_shared_joist();
        let w_of = |_: &Slab| 0.005_f64;
        let map = secondary_joist_distribution_loads(&model, w_of);
        let key = span_node_key(NodeId(4), NodeId(5));
        let entry = map.get(&key).expect("共有辺小梁に Span 荷重がある");
        let ex = simple_beam_extremes(&entry.member_loads, 4000.0, 205_000.0, 1.0e8);
        // 2000×4000 の 2 枚が x=2000 の辺を共有。三角/台形分配の辺荷重を重ね合わせると
        // 合計 30000 N / 4000 mm = 7.5 N/mm（旧 w×spacing=15 とは分配形状が異なる）。
        // 小梁断面の需要は負担幅一様より小さくなる。解析の大梁伝達と同じ 45° 分配を
        // 検定に使うためであり、負担幅一様は小梁に対して過大評価（安全側）になりうる。
        assert!((ex.w_equiv - 7.5).abs() < 0.1, "w_equiv={}", ex.w_equiv);
    }

    #[test]
    fn simple_beam_extremes_uniform_matches_closed_form() {
        let w = 10.0_f64;
        let l = 4000.0_f64;
        let loads = vec![MemberLoadKind::Distributed {
            a: 0.0,
            b: l,
            w1: w,
            w2: w,
        }];
        let e = 205_000.0;
        let i = 1.0e8;
        let ex = simple_beam_extremes(&loads, l, e, i);
        let m = w * l * l / 8.0;
        let q = w * l / 2.0;
        let d = 5.0 * w * l.powi(4) / (384.0 * e * i);
        assert!((ex.m_max - m).abs() / m < 1e-6, "m_max={}", ex.m_max);
        assert!((ex.q_max - q).abs() / q < 1e-6, "q_max={}", ex.q_max);
        assert!(
            (ex.deflection - d).abs() / d < 2e-3,
            "defl={}",
            ex.deflection
        );
        assert!((ex.w_equiv - w).abs() < 1e-9);
    }

    #[test]
    fn split_slab_edges_compose_onto_full_joist() {
        // 左を途中節点で 2 枚に割り、右は全長 1 辺。小梁は 4–5 の全長。
        let mut model = square_model_with_shared_joist();
        model.nodes.push(Node {
            id: NodeId(6),
            coord: [2000.0, 2000.0, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
        model.nodes.push(Node {
            id: NodeId(7),
            coord: [0.0, 2000.0, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
        let plate = model.slabs[0].plate.clone();
        model.slabs = vec![
            Slab {
                id: SlabId(0),
                shape: SlabShape::Enclosed {
                    boundary: vec![NodeId(0), NodeId(4), NodeId(6), NodeId(7)],
                },
                plate: plate.clone(),
            },
            Slab {
                id: SlabId(1),
                shape: SlabShape::Enclosed {
                    boundary: vec![NodeId(7), NodeId(6), NodeId(5), NodeId(3)],
                },
                plate: plate.clone(),
            },
            Slab {
                id: SlabId(2),
                shape: SlabShape::Enclosed {
                    boundary: vec![NodeId(4), NodeId(1), NodeId(2), NodeId(5)],
                },
                plate,
            },
        ];
        model.floor_regions[0].slab_ids = vec![SlabId(0), SlabId(1), SlabId(2)];
        let w_of = |_: &Slab| 0.005_f64;
        let map = secondary_joist_distribution_loads(&model, w_of);
        let key = span_node_key(NodeId(4), NodeId(5));
        let entry = map.get(&key).expect("分割辺も全長小梁へ合成される");
        let total: f64 = entry
            .member_loads
            .iter()
            .map(|l| match l {
                MemberLoadKind::Point { p, .. } => *p,
                MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
            })
            .sum();
        let unsplit = square_model_with_shared_joist();
        let unsplit_map = secondary_joist_distribution_loads(&unsplit, w_of);
        let unsplit_total: f64 = unsplit_map[&key]
            .member_loads
            .iter()
            .map(|l| match l {
                MemberLoadKind::Point { p, .. } => *p,
                MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
            })
            .sum();
        // 左を 2 枚に割ると共有する水平辺へ 45° 分配の一部が逃げるため、
        // 小梁上の合計は未分割より小さくなりうる。欠落していたのは「節点対不一致で
        // 左側が 0 になる」ことなので、左右それぞれの半区間に十分な荷重があり、
        // 右側だけの約半分（15000 N）を十分上回ることを見る。
        let half = 2000.0_f64;
        let load_on = |s0: f64, s1: f64| -> f64 {
            entry
                .member_loads
                .iter()
                .map(|l| match l {
                    MemberLoadKind::Point { a, p } => {
                        if *a >= s0 - 1e-9 && *a <= s1 + 1e-9 {
                            *p
                        } else {
                            0.0
                        }
                    }
                    MemberLoadKind::Distributed { a, b, w1, w2 } => {
                        let lo = (*a).max(s0);
                        let hi = (*b).min(s1);
                        if hi <= lo {
                            return 0.0;
                        }
                        let t0 = (lo - *a) / (*b - *a);
                        let t1 = (hi - *a) / (*b - *a);
                        let w_lo = w1 + (w2 - w1) * t0;
                        let w_hi = w1 + (w2 - w1) * t1;
                        (w_lo + w_hi) / 2.0 * (hi - lo)
                    }
                })
                .sum()
        };
        let left = load_on(0.0, half);
        let right = load_on(half, 4000.0);
        assert!(
            left > 5000.0,
            "始端側半区間の荷重が薄い left={left}（左側欠落の兆候）"
        );
        assert!(right > 5000.0, "終端側半区間の荷重が薄い right={right}");
        assert!(
            total > 0.9 * (left + right).min(unsplit_total),
            "split={total} left+right={} unsplit={unsplit_total}",
            left + right
        );
        assert!(
            total > 20000.0,
            "右側だけだと ~15000。合成後はそれを十分上回る total={total}"
        );
        let (mut s_min, mut s_max) = (f64::INFINITY, 0.0_f64);
        for l in &entry.member_loads {
            match l {
                MemberLoadKind::Point { a, .. } => {
                    s_min = s_min.min(*a);
                    s_max = s_max.max(*a);
                }
                MemberLoadKind::Distributed { a, b, .. } => {
                    s_min = s_min.min(*a);
                    s_max = s_max.max(*b);
                }
            }
        }
        assert!(s_min < 200.0, "始端側が欠ける s_min={s_min}");
        assert!(s_max > 3800.0, "終端側が欠ける s_max={s_max}");
    }

    #[test]
    fn distribution_mq_vs_uniform_tributary_width() {
        // 共有辺: 分配重ね合わせ w_equiv=7.5。負担幅一様なら w=面荷重×間隔=0.005×3000=15。
        // （左右各 2000 の半分合計 2000 ではなく、2 枚×2000 の合計半分=2000… ここでは
        // 旧略算の代表として spacing=3000 → w=15 を使う。テストコメントと一致。）
        let model = square_model_with_shared_joist();
        let w_of = |_: &Slab| 0.005_f64;
        let map = secondary_joist_distribution_loads(&model, w_of);
        let key = span_node_key(NodeId(4), NodeId(5));
        let entry = map.get(&key).expect("共有辺");
        let l = 4000.0_f64;
        let ex = simple_beam_extremes(&entry.member_loads, l, 205_000.0, 1.0e8);
        let w_trib = 15.0_f64;
        let m_trib = w_trib * l * l / 8.0;
        let q_trib = w_trib * l / 2.0;
        // 分配経路は負担幅一様より系統的に小さい（大梁伝達と同じ 45°）。
        assert!(
            ex.m_max < 0.85 * m_trib,
            "分配 M={} が負担幅一様 M={m_trib} の 85% 未満であること",
            ex.m_max
        );
        assert!(
            ex.q_max < 0.85 * q_trib,
            "分配 Q={} が負担幅一様 Q={q_trib} の 85% 未満であること",
            ex.q_max
        );
        // 等価等分布 7.5 の閉形式と比較（三角/台形の重ねでも合計は一致）。
        let m_u = 7.5 * l * l / 8.0;
        let q_u = 7.5 * l / 2.0;
        assert!((ex.w_equiv - 7.5).abs() < 0.1, "w_equiv={}", ex.w_equiv);
        // 形状が非一様なので M は等価一様と完全一致しないが、同じオーダー。
        assert!(
            (ex.m_max - m_u).abs() / m_u < 0.25,
            "m_max={} m_uniform={m_u}",
            ex.m_max
        );
        assert!(
            (ex.q_max - q_u).abs() / q_u < 0.25,
            "q_max={} q_uniform={q_u}",
            ex.q_max
        );
    }

    #[test]
    fn span_attaches_to_nearest_joist_only() {
        let mut model = square_model_with_shared_joist();
        // 共有辺 4–5 に平行で 5 mm ずれた別小梁（近接）。荷重は近い方だけへ。
        model.nodes.push(Node {
            id: NodeId(6),
            coord: [2005.0, 0.0, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
        model.nodes.push(Node {
            id: NodeId(7),
            coord: [2005.0, 4000.0, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
        model.floor_regions[0]
            .secondary_joists
            .push(SecondaryMember {
                kind: SecondaryMemberKind::Joist,
                nodes: [NodeId(6), NodeId(7)],
                section: None,
                name: "J2".into(),
            });
        let w_of = |_: &Slab| 0.005_f64;
        let map = secondary_joist_distribution_loads(&model, w_of);
        let k1 = span_node_key(NodeId(4), NodeId(5));
        let k2 = span_node_key(NodeId(6), NodeId(7));
        let t1 = map
            .get(&k1)
            .map(|e| {
                e.member_loads
                    .iter()
                    .map(|l| match l {
                        MemberLoadKind::Point { p, .. } => *p,
                        MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
                    })
                    .sum::<f64>()
            })
            .unwrap_or(0.0);
        let t2 = map
            .get(&k2)
            .map(|e| {
                e.member_loads
                    .iter()
                    .map(|l| match l {
                        MemberLoadKind::Point { p, .. } => *p,
                        MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
                    })
                    .sum::<f64>()
            })
            .unwrap_or(0.0);
        // 境界は x=2000 なので 4–5 が近い。J2 には載らない（または無視できる量）。
        assert!(t1 > 10000.0, "近い小梁 t1={t1}");
        assert!(t2 < 1.0, "遠い小梁へ二重計上 t2={t2}");
    }

    #[test]
    fn partial_span_t_maps_onto_joist_interior() {
        let model = square_model_with_shared_joist();
        let a = coord(&model, NodeId(4)).unwrap();
        let b = coord(&model, NodeId(5)).unwrap();
        let p0 = lerp3(a, b, 0.375);
        let p1 = lerp3(a, b, 0.625);
        let s0 = project_on_segment(p0, a, b, MEMBER_AXIS_TOL_MM).unwrap();
        let s1 = project_on_segment(p1, a, b, MEMBER_AXIS_TOL_MM).unwrap();
        assert!((s0 - 1500.0).abs() < 1e-6, "s0={s0}");
        assert!((s1 - 2500.0).abs() < 1e-6, "s1={s1}");
        let piece = load_shape_to_member_loads(&LoadShape::Uniform { w: 2.0 }, s1 - s0, false);
        let shifted = shift_member_load(&piece[0], s0);
        let MemberLoadKind::Distributed { a, b, w1, w2 } = shifted else {
            panic!("{shifted:?}");
        };
        assert!((a - 1500.0).abs() < 1e-9 && (b - 2500.0).abs() < 1e-9);
        assert!((w1 - 2.0).abs() < 1e-12 && (w2 - 2.0).abs() < 1e-12);
    }

    #[test]
    fn joist_distribution_cover_rejects_half_span() {
        let l = 4000.0;
        let half = vec![MemberLoadKind::Distributed {
            a: 0.0,
            b: 1800.0,
            w1: 7.5,
            w2: 7.5,
        }];
        assert!(!joist_distribution_is_sufficient(&half, l));
        let full = vec![MemberLoadKind::Distributed {
            a: 0.0,
            b: l,
            w1: 7.5,
            w2: 7.5,
        }];
        assert!(joist_distribution_is_sufficient(&full, l));
        assert!((covered_length_of_loads(&half, l) - 1800.0).abs() < 1e-9);
    }

    #[test]
    fn map_loads_onto_axis_preserves_total_force() {
        let loaded_len = 1000.0;
        let loads = vec![MemberLoadKind::Distributed {
            a: 0.0,
            b: loaded_len,
            w1: 4.0,
            w2: 4.0,
        }];
        let mapped = map_loads_onto_axis(&loads, loaded_len, 100.0, 900.0);
        let total: f64 = mapped.iter().map(member_load_total).sum();
        assert!((total - 4000.0).abs() < 1e-6, "total={total}");
        let MemberLoadKind::Distributed { a, b, w1, w2 } = mapped[0] else {
            panic!("{:?}", mapped[0]);
        };
        assert!((a - 100.0).abs() < 1e-9 && (b - 900.0).abs() < 1e-9);
        assert!((w1 - 5.0).abs() < 1e-9 && (w2 - 5.0).abs() < 1e-9);
    }

    #[test]
    fn perimeter_parallel_joist_does_not_steal_beam_span() {
        let mut model = square_model_with_shared_joist();
        model.nodes.push(Node {
            id: NodeId(8),
            coord: [0.0, 5.0, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
        model.nodes.push(Node {
            id: NodeId(9),
            coord: [4000.0, 5.0, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
        model.floor_regions[0]
            .secondary_joists
            .push(SecondaryMember {
                kind: SecondaryMemberKind::Joist,
                nodes: [NodeId(8), NodeId(9)],
                section: None,
                name: "parallel".into(),
            });
        let w_of = |_: &Slab| 0.005_f64;
        let map = secondary_joist_distribution_loads(&model, w_of);
        let k_shared = span_node_key(NodeId(4), NodeId(5));
        let k_par = span_node_key(NodeId(8), NodeId(9));
        let t_shared = map
            .get(&k_shared)
            .map(|e| e.member_loads.iter().map(member_load_total).sum::<f64>())
            .unwrap_or(0.0);
        let t_par = map
            .get(&k_par)
            .map(|e| e.member_loads.iter().map(member_load_total).sum::<f64>())
            .unwrap_or(0.0);
        assert!(t_shared > 20000.0, "共有辺 t_shared={t_shared}");
        assert!(t_par < 1.0, "外周並走が大梁辺を奪う t_par={t_par}");
        assert!(
            joist_distribution_is_sufficient(&map[&k_shared].member_loads, 4000.0),
            "共有辺のカバーが足りない"
        );
    }

    #[test]
    fn shared_joist_expects_both_slabs() {
        let model = square_model_with_shared_joist();
        let w_of = |_: &Slab| 0.005_f64;
        let map = secondary_joist_distribution_loads(&model, w_of);
        let key = span_node_key(NodeId(4), NodeId(5));
        let entry = map.get(&key).expect("共有辺");
        assert_eq!(entry.expected_slab_ids.len(), 2);
        assert_eq!(entry.contributed_slab_ids.len(), 2);
        assert!(joist_distribution_is_ready(entry, 4000.0));
    }

    #[test]
    fn one_sided_slab_is_ready_when_only_one_slab_emits() {
        let mut model = square_model_with_shared_joist();
        model.slabs.pop();
        model.floor_regions[0].slab_ids = vec![SlabId(0)];
        let w_of = |_: &Slab| 0.005_f64;
        let map = secondary_joist_distribution_loads(&model, w_of);
        let key = span_node_key(NodeId(4), NodeId(5));
        let entry = map.get(&key).expect("片側でも期待床板があれば載る");
        assert_eq!(entry.expected_slab_ids.len(), 1);
        assert!(entry.expected_slab_ids.contains(&SlabId(0)));
        assert!(joist_distribution_is_ready(entry, 4000.0));
    }

    #[test]
    fn missing_expected_slab_is_not_ready() {
        let expected = HashSet::from([SlabId(0), SlabId(1)]);
        let contributed = HashSet::from([SlabId(0)]);
        assert!(!joist_expected_slabs_covered(&expected, &contributed));
        let loads = vec![MemberLoadKind::Distributed {
            a: 0.0,
            b: 4000.0,
            w1: 7.5,
            w2: 7.5,
        }];
        let entry = SecondaryJoistLoads {
            member_loads: loads,
            rep_slab_id: Some(SlabId(0)),
            span_nodes: (NodeId(4), NodeId(5)),
            expected_slab_ids: expected,
            contributed_slab_ids: contributed,
        };
        assert!(!joist_distribution_is_ready(&entry, 4000.0));
    }

    #[test]
    fn zero_expected_axis_does_not_receive_spans() {
        let mut model = square_model_with_shared_joist();
        model.nodes.push(Node {
            id: NodeId(10),
            coord: [8000.0, 0.0, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
        model.nodes.push(Node {
            id: NodeId(11),
            coord: [8000.0, 4000.0, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
        model.unassigned_joists.push(SecondaryMember {
            kind: SecondaryMemberKind::Joist,
            nodes: [NodeId(10), NodeId(11)],
            section: None,
            name: "far".into(),
        });
        let w_of = |_: &Slab| 0.005_f64;
        let map = secondary_joist_distribution_loads(&model, w_of);
        let k_far = span_node_key(NodeId(10), NodeId(11));
        let k_shared = span_node_key(NodeId(4), NodeId(5));
        assert!(
            !map.contains_key(&k_far),
            "期待床板 0 枚の材軸は付着先にしない"
        );
        assert!(joist_distribution_is_ready(&map[&k_shared], 4000.0));
    }
}
