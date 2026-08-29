//! 床領域の分配結果（[`super::distribute_region`]）から領域内小梁の設計部材力を求める。
//!
//! 領域内小梁（[`squid_n_core::model::FloorRegion::secondary_joists`]）の断面検定は、
//! 幾何から負担幅を再導出せず、荷重分配と同じ `BeamLoad`（`LoadTarget::Span`）を
//! 単純梁として重ね合わせる。床板境界が途中節点で分割されていても、小梁材軸上に
//! 載る `Span` は全長へ合成する（T 字取り付きの片側欠落を防ぐ）。

use std::collections::HashMap;

use squid_n_core::geom::MEMBER_AXIS_TOL_MM;
use squid_n_core::ids::{NodeId, SlabId};
use squid_n_core::model::{ElementKind, MemberLoadKind, Model, SecondaryMemberKind, Slab};
use squid_n_core::units::GRAVITY_MM_S2;

use super::distribute_region;
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
    /// 代表床板（所属床領域の `slab_ids` 先頭。表示・室用途の参照用）。
    pub rep_slab_id: SlabId,
    /// このエントリの荷重を載せるときの小梁節点順（`SecondaryMember.nodes`）。
    pub span_nodes: (NodeId, NodeId),
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

struct JoistAxis {
    key: (NodeId, NodeId),
    nodes: (NodeId, NodeId),
    a: [f64; 3],
    b: [f64; 3],
    len: f64,
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
            len,
        });
    }
    out
}

fn segment_on_beam(model: &Model, p0: [f64; 3], p1: [f64; 3]) -> bool {
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
/// その小梁の局所座標へ写して重ねる。部分区間 `t` も同じ。大梁材軸上の `Span` は除外する。
pub fn secondary_joist_distribution_loads(
    model: &Model,
    w_of: impl Fn(&Slab) -> f64,
) -> HashMap<(NodeId, NodeId), SecondaryJoistLoads> {
    let axes = joist_axes(model);
    let mut map: HashMap<(NodeId, NodeId), SecondaryJoistLoads> = HashMap::new();
    for region in &model.floor_regions {
        let rep_slab_id = region.slab_ids.first().copied().unwrap_or(SlabId(0));
        let loads = distribute_region(model, region, |s| w_of(s));
        for bl in loads {
            let BeamLoad { target, shape, .. } = bl;
            let LoadTarget::Span { nodes: [n0, n1], t } = target else {
                continue;
            };
            let (Some(c0), Some(c1)) = (coord(model, n0), coord(model, n1)) else {
                continue;
            };
            let p0 = lerp3(c0, c1, t[0]);
            let p1 = lerp3(c0, c1, t[1]);
            let loaded_len = dist3(p0, p1);
            if loaded_len <= 1e-9 {
                continue;
            }
            if segment_on_beam(model, p0, p1) {
                continue;
            }
            let shape_loads = load_shape_to_member_loads(&shape, loaded_len, false);
            for axis in &axes {
                let Some(mut s0) = project_on_segment(p0, axis.a, axis.b, MEMBER_AXIS_TOL_MM)
                else {
                    continue;
                };
                let Some(mut s1) = project_on_segment(p1, axis.a, axis.b, MEMBER_AXIS_TOL_MM)
                else {
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
                let entry = map.entry(axis.key).or_insert_with(|| SecondaryJoistLoads {
                    member_loads: Vec::new(),
                    rep_slab_id,
                    span_nodes: axis.nodes,
                });
                entry
                    .member_loads
                    .extend(piece.into_iter().map(|l| shift_member_load(&l, s0)));
            }
        }
    }
    map
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
/// 実部材化済み・`FloorRegion.joists` 重複・断面未割当・退化は数えない。
pub fn secondary_joists_missing_distribution(model: &Model) -> usize {
    use squid_n_core::model::{LoadPurpose, SecondaryMemberKind};

    let w_of = |s: &Slab| model.slab_intensity(s, LoadPurpose::Floor);
    let distribution = secondary_joist_distribution_loads(model, w_of);

    let mut joist_supports = std::collections::HashSet::new();
    for region in &model.floor_regions {
        for j in region.joist_lines() {
            let (a, b) = (j.support[0], j.support[1]);
            if a != b {
                joist_supports.insert(span_node_key(a, b));
            }
        }
    }

    let mut missing = 0usize;
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
        if joist_supports.contains(&key) {
            continue;
        }
        let has_loads = distribution
            .get(&key)
            .is_some_and(|e| !e.member_loads.is_empty());
        if !has_loads {
            missing += 1;
        }
    }
    missing
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
        assert!(
            (total - unsplit_total).abs() / unsplit_total < 0.15,
            "split={total} unsplit={unsplit_total}"
        );
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
}
