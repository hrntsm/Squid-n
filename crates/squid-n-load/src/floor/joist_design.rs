//! 床領域の分配結果（[`super::distribute_region`]）から領域内小梁の設計部材力を求める。
//!
//! ST-Bridge 取り込み小梁（床領域内の [`squid_n_core::model::SecondaryMember`]）の
//! 断面検定は、幾何から負担幅を再導出せず、荷重分配と同じ `BeamLoad`（`LoadTarget::Span`）
//! を単純梁として重ね合わせる（Step 5 で `check.rs` へ結線予定）。

#![allow(dead_code)] // Step 5 結線前のため未使用 API を許容

use std::collections::HashMap;

use squid_n_core::ids::{NodeId, SlabId};
use squid_n_core::model::{ElementKind, MemberLoadKind, Model, Slab};

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
    const N: usize = 80;
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
    const N: usize = 64;
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
    /// 分配側が最初に返した `Span.nodes`（向き判定用）。
    pub span_nodes: (NodeId, NodeId),
}

fn beam_between(model: &Model, a: NodeId, b: NodeId) -> bool {
    model.elements.iter().any(|e| {
        e.kind == ElementKind::Beam
            && e.nodes.len() == 2
            && ((e.nodes[0] == a && e.nodes[1] == b) || (e.nodes[0] == b && e.nodes[1] == a))
    })
}

fn joist_span_length(model: &Model, n0: NodeId, n1: NodeId) -> Option<f64> {
    let na = model.nodes.get(n0.index())?;
    let nb = model.nodes.get(n1.index())?;
    let dx = nb.coord[0] - na.coord[0];
    let dy = nb.coord[1] - na.coord[1];
    let dz = nb.coord[2] - na.coord[2];
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    (len > 1e-9).then_some(len)
}

fn span_is_full(t: [f64; 2]) -> bool {
    (t[0] - 0.0).abs() <= 1e-9 && (t[1] - 1.0).abs() <= 1e-9
}

/// 全床領域の [`distribute_region`] 出力から、実部材化していない小梁 `Span` 荷重を集める。
pub fn secondary_joist_distribution_loads(
    model: &Model,
    w_of: impl Fn(&Slab) -> f64,
) -> HashMap<(NodeId, NodeId), SecondaryJoistLoads> {
    let mut map: HashMap<(NodeId, NodeId), SecondaryJoistLoads> = HashMap::new();
    for region in &model.floor_regions {
        let rep_slab_id = region.slab_ids.first().copied().unwrap_or(SlabId(0));
        let loads = distribute_region(model, region, |s| w_of(s));
        for bl in loads {
            let BeamLoad { target, shape, .. } = bl;
            let LoadTarget::Span { nodes: [n0, n1], t } = target else {
                continue;
            };
            if !span_is_full(t) {
                continue;
            }
            if beam_between(model, n0, n1) {
                continue;
            }
            let Some(len) = joist_span_length(model, n0, n1) else {
                continue;
            };
            let key = span_node_key(n0, n1);
            let entry = map.entry(key).or_insert_with(|| SecondaryJoistLoads {
                member_loads: Vec::new(),
                rep_slab_id,
                span_nodes: (n0, n1),
            });
            let flip = (n0, n1) != entry.span_nodes;
            entry
                .member_loads
                .extend(load_shape_to_member_loads(&shape, len, flip));
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
    use super::*;
    use squid_n_core::ids::{ElemId, FloorRegionId};
    use squid_n_core::model::{
        AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, FloorRegion,
        ForceRegime, LocalAxis, Node, Slab, SlabPlate, SlabShape,
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
        assert!((ex.w_equiv - 7.5).abs() < 0.1, "w_equiv={}", ex.w_equiv);
    }
}
