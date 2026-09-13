//! 二次部材の反力の逐次伝達のテスト。
//!
//! 床分配を混ぜると期待値の算定が分配則に依存してしまうため、ここでは面荷重を 0 にし、
//! **自重だけ**を載せて逐次伝達の骨格（支持関係の判定・順序・反力の分解・総和保存）を
//! 確かめる。床分配との結線は `squid-n-job` 側の統合テストで見る。

use super::*;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, SectionId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, MaterialCategory,
    Node, Section,
};
use squid_n_core::units::GRAVITY_MM_S2;

/// 自重の等分布荷重 [N/mm]（テストの期待値算定用）。密度 × 断面積 × g。
const DENSITY: f64 = 7.85e-9; // t/mm³ 相当（N 系。鋼）
const AREA: f64 = 10_000.0; // mm²

fn w_self() -> f64 {
    DENSITY * AREA * GRAVITY_MM_S2
}

fn node(id: u32, x: f64, y: f64, z: f64) -> Node {
    Node {
        id: NodeId(id),
        coord: [x, y, z],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    }
}

fn beam(id: u32, i: u32, j: u32) -> ElementData {
    ElementData {
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
    }
}

fn joist(a: u32, b: u32, name: &str) -> SecondaryMember {
    SecondaryMember {
        gravity_end_shares: None,
        end_support: Default::default(),
        kind: SecondaryMemberKind::Joist,
        nodes: [NodeId(a), NodeId(b)],
        section: Some(SectionId(0)),
        name: name.to_string(),
    }
}

/// 鋼の材料と断面（自重が出る最小構成）を持つ空モデル。
fn base_model() -> Model {
    let mut m = Model::default();
    m.materials.push(Material {
        id: MaterialId(0),
        name: "SN400".into(),
        category: MaterialCategory::Steel,
        young: 205_000.0,
        poisson: 0.3,
        density: DENSITY,
        shear: None,
        fc: None,
        fy: Some(235.0),
        concrete_class: Default::default(),
        strength_factor: None,
    });
    m.sections.push(Section {
        id: SectionId(0),
        name: "H".into(),
        floor: None,
        area: AREA,
        iy: 1.0e8,
        iz: 1.0e7,
        j: 1.0e6,
        depth: 400.0,
        width: 200.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    m
}

fn solved(model: &Model) -> SecondaryTransfer {
    // 鉄骨重量割増は `load_cfg` 未設定なら 1.0（`joist_self_weight_udl`）。
    solve(model, |_| 0.0, true)
}

/// 両端が大梁に載る小梁は、自重の半分ずつを主架構へ渡して終端する。
#[test]
fn joist_on_girders_terminates_at_primary() {
    let mut m = base_model();
    // 大梁 0-1（X 方向、y=0）と 2-3（X 方向、y=4000）。小梁は 4-5（Y 方向、x=3000）。
    for (i, c) in [
        [0.0, 0.0, 0.0],
        [6000.0, 0.0, 0.0],
        [0.0, 4000.0, 0.0],
        [6000.0, 4000.0, 0.0],
        [3000.0, 0.0, 0.0],
        [3000.0, 4000.0, 0.0],
    ]
    .iter()
    .enumerate()
    {
        m.nodes.push(node(i as u32, c[0], c[1], c[2]));
    }
    m.elements.push(beam(0, 0, 1));
    m.elements.push(beam(1, 2, 3));
    m.unassigned_joists.push(joist(4, 5, "SB1"));

    let t = solved(&m);
    let key = span_node_key(NodeId(4), NodeId(5));
    let sm = t.members.get(&key).expect("小梁");
    assert_eq!(sm.supports, [SupportAt::Primary, SupportAt::Primary]);
    let expected = w_self() * 4000.0 / 2.0;
    for r in sm.reactions {
        assert!((r - expected).abs() / expected < 1e-9, "反力 {r}");
    }
    assert!(t.unresolved.is_empty());
    assert!(t.cyclic.is_empty());
    assert!(super::secondary_crossings(&m).is_empty());
}

/// 小梁 B の端点が小梁 A の内部に載るとき、B の反力は A の集中荷重として渡り、
/// 主架構へ渡る総和は 2 本の自重の合計に一致する（荷重が消えない）。
#[test]
fn joist_on_joist_cascades_to_primary() {
    let mut m = base_model();
    for (i, c) in [
        [0.0, 0.0, 0.0],       // 0 大梁端
        [6000.0, 0.0, 0.0],    // 1 大梁端
        [0.0, 4000.0, 0.0],    // 2 大梁端
        [6000.0, 4000.0, 0.0], // 3 大梁端
        [3000.0, 0.0, 0.0],    // 4 A の端（大梁上）
        [3000.0, 4000.0, 0.0], // 5 A の端（大梁上）
        [3000.0, 2000.0, 0.0], // 6 A の中央（B の端。どの大梁にも載らない）
        [6000.0, 2000.0, 0.0], // 7 B の端（右側の大梁 1-3 のスパン上）
    ]
    .iter()
    .enumerate()
    {
        m.nodes.push(node(i as u32, c[0], c[1], c[2]));
    }
    m.elements.push(beam(0, 0, 1));
    m.elements.push(beam(1, 2, 3));
    m.elements.push(beam(2, 1, 3)); // 右側の大梁（節点 7 がこのスパン上に載る）
    m.unassigned_joists.push(joist(4, 5, "A"));
    m.unassigned_joists.push(joist(6, 7, "B"));

    let t = solved(&m);
    let ka = span_node_key(NodeId(4), NodeId(5));
    let kb = span_node_key(NodeId(6), NodeId(7));
    let a = t.members.get(&ka).expect("A");
    let b = t.members.get(&kb).expect("B");

    // B の節点 6 側は A の内部に載る。
    let i6 = b.nodes.iter().position(|n| *n == NodeId(6)).expect("節点6");
    assert!(
        matches!(b.supports[i6], SupportAt::Secondary { key, .. } if key == ka),
        "B の端は A に載る: {:?}",
        b.supports
    );
    // A の両端は大梁上で終端する。
    assert_eq!(a.supports, [SupportAt::Primary, SupportAt::Primary]);

    // 主架構へ渡る総和 = A の自重 + B の自重。
    let total: f64 = t.primary_node_loads().iter().map(|(_, r)| r).sum();
    let expected = w_self() * (4000.0 + 3000.0);
    assert!(
        (total - expected).abs() / expected < 1e-9,
        "主架構へ渡る総和 {total} != 自重合計 {expected}"
    );
    assert!(t.unresolved.is_empty(), "{:?}", t.unresolved);
}

/// 間柱自重の端部負担率・総量保存・逆向き・未指定の診断を検証する。
#[test]
fn vertical_post_uses_explicit_end_shares() {
    let mut m = base_model();
    for (i, c) in [
        [0.0, 0.0, 0.0],
        [6000.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [6000.0, 0.0, 3000.0],
        [3000.0, 0.0, 0.0],
        [3000.0, 0.0, 3000.0],
    ]
    .iter()
    .enumerate()
    {
        m.nodes.push(node(i as u32, c[0], c[1], c[2]));
    }
    m.elements.push(beam(0, 0, 1)); // 下の梁
    m.elements.push(beam(1, 2, 3)); // 上の梁
    m.unassigned_posts.push(SecondaryMember {
        gravity_end_shares: Some([0.5, 0.5]),
        end_support: Default::default(),
        kind: SecondaryMemberKind::Post,
        nodes: [NodeId(4), NodeId(5)],
        section: Some(SectionId(0)),
        name: "P1".into(),
    });

    let t = solved(&m);
    let key = span_node_key(NodeId(4), NodeId(5));
    let p = t.members.get(&key).expect("間柱");
    let half = w_self() * 3000.0 / 2.0;
    for r in p.reactions {
        assert!((r - half).abs() / half < 1e-9, "反力 {r} != {half}");
    }
    for (ratios, expected) in [
        ([1.0, 0.0], [2.0 * half, 0.0]),
        ([0.25, 0.75], [0.5 * half, 1.5 * half]),
        ([0.0, 1.0], [0.0, 2.0 * half]),
    ] {
        m.unassigned_posts[0].gravity_end_shares = Some(ratios);
        let t = solved(&m);
        let p = &t.members[&key];
        for (actual, expected) in p.reactions.iter().zip(expected) {
            assert!((actual - expected).abs() < 1e-6);
        }
        assert!(
            (t.primary_node_loads().iter().map(|(_, w)| w).sum::<f64>() - 2.0 * half).abs() < 1e-6
        );
    }
    m.unassigned_posts[0].nodes.reverse();
    m.unassigned_posts[0].gravity_end_shares = Some([0.0, 1.0]);
    let t = solved(&m);
    assert_eq!(t.primary_node_loads().len(), 1);
    assert_eq!(t.primary_node_loads()[0].0, NodeId(4));

    m.unassigned_posts[0].end_support = [EndSupport::Free, EndSupport::Supported];
    let t = solved(&m);
    assert!(t.invalid_end_shares.is_empty());
    assert!((t.primary_node_loads()[0].1 - 2.0 * half).abs() < 1e-6);
    m.unassigned_posts[0].gravity_end_shares = Some([0.25, 0.75]);
    let t = solved(&m);
    assert_eq!(t.invalid_end_shares, vec![key]);
    assert!(t.primary_node_loads().is_empty());
    m.unassigned_posts[0].end_support = [EndSupport::Supported; 2];

    for ratios in [
        None,
        Some([0.0, 0.0]),
        Some([-0.5, 1.5]),
        Some([f64::NAN, 1.0]),
    ] {
        m.unassigned_posts[0].gravity_end_shares = ratios;
        let t = solved(&m);
        assert_eq!(t.invalid_end_shares, vec![key]);
        assert!(t.primary_node_loads().is_empty());
    }
}

/// 傾斜した二次部材の鉛直反力は、材軸上の按分（単純梁の反力）と一致する。
///
/// 鉛直反力は水平てこでのモーメントつり合いで決まり、荷重の材軸上の位置は水平投影へ
/// 線形に写るため、水平投影が 0 でなければ成分へ分ける必要はない。「材軸方向成分を
/// 1/2 ずつ、直交成分を単純梁反力」として `|u_z|` で混ぜると、総和は保存するが配分が
/// 誤り、載荷側の反力を過小評価する（受け側にとって危険側）。実フィクスチャの小梁は
/// すべて水平なのでこの誤りを検出できない。ここで固定する。
#[test]
fn inclined_joist_reactions_match_simple_beam() {
    let mut m = base_model();
    // 水平投影 4000・鉛直 3000（L=5000）の傾斜小梁。両端は大梁に載せて終端させる。
    for (i, c) in [
        [-1000.0, 0.0, 0.0],
        [1000.0, 0.0, 0.0],
        [3000.0, 0.0, 3000.0],
        [5000.0, 0.0, 3000.0],
        [0.0, 0.0, 0.0],       // 4 傾斜小梁の下端（下の大梁上）
        [4000.0, 0.0, 3000.0], // 5 傾斜小梁の上端（上の大梁上）
    ]
    .iter()
    .enumerate()
    {
        m.nodes.push(node(i as u32, c[0], c[1], c[2]));
    }
    m.elements.push(beam(0, 0, 1));
    m.elements.push(beam(1, 2, 3));
    m.unassigned_joists.push(joist(4, 5, "SB"));

    let t = solved(&m);
    let key = span_node_key(NodeId(4), NodeId(5));
    let sm = t.members.get(&key).expect("傾斜小梁");
    assert_eq!(sm.supports, [SupportAt::Primary, SupportAt::Primary]);

    // 自重は等分布なので、鉛直反力は両端等分（総量は ρAgL）。
    let total = w_self() * 5000.0;
    for r in sm.reactions {
        assert!(
            (r - total / 2.0).abs() / total < 1e-9,
            "等分布の鉛直反力は両端等分: {r}"
        );
    }

    // 材軸上 1/5 の位置に集中荷重を足すと、鉛直反力は 4:1 に分かれる
    // （水平てこでのモーメントつり合い。混ぜると 0.62:0.38 になってしまう）。
    let p = 1000.0_f64;
    let (ri, rj) = super::reactions_of(&MemberLoadKind::Point { a: 1000.0, p }, 5000.0, None);
    assert!((ri - 0.8 * p).abs() < 1e-9, "R_i={ri}");
    assert!((rj - 0.2 * p).abs() < 1e-9, "R_j={rj}");

    // 鉛直材では、明示した両端1/2の負担率を使用する。
    let (ri, rj) = super::reactions_of(
        &MemberLoadKind::Point { a: 1000.0, p },
        5000.0,
        Some([0.5, 0.5]),
    );
    assert!((ri - 0.5 * p).abs() < 1e-9 && (rj - 0.5 * p).abs() < 1e-9);
}

/// 端部がどの主架構にも二次部材にも載らない二次部材は `unresolved` に入る。
#[test]
fn floating_joist_is_unresolved() {
    let mut m = base_model();
    m.nodes.push(node(0, 0.0, 0.0, 0.0));
    m.nodes.push(node(1, 4000.0, 0.0, 0.0));
    m.unassigned_joists.push(joist(0, 1, "SB"));

    let t = solved(&m);
    assert_eq!(t.unresolved, vec![span_node_key(NodeId(0), NodeId(1))]);
}

/// 支持関係が一巡する二次部材は荷重を流せないので `cyclic` に入り、逐次伝達の対象から
/// 外れる（モデルの誤りであり、診断のエラーで知らせる）。
///
/// 直線 2 本では相互支持は幾何的に成立しない（互いの端点が相手の内部にある配置が
/// 作れない）ため、3 本で一巡させる。
#[test]
fn cyclic_support_is_reported() {
    let mut m = base_model();
    for (i, c) in [
        [0.0, 0.0, 0.0],         // 0 A 始点（C の内部に載る）
        [4000.0, 0.0, 0.0],      // 1 A 終点
        [2000.0, 0.0, 0.0],      // 2 B 始点（A の内部に載る）
        [2000.0, 4000.0, 0.0],   // 3 B 終点
        [2000.0, 2000.0, 0.0],   // 4 C 始点（B の内部に載る）
        [-2000.0, -2000.0, 0.0], // 5 C 終点
    ]
    .iter()
    .enumerate()
    {
        m.nodes.push(node(i as u32, c[0], c[1], c[2]));
    }
    m.unassigned_joists.push(joist(0, 1, "A"));
    m.unassigned_joists.push(joist(2, 3, "B"));
    m.unassigned_joists.push(joist(4, 5, "C"));

    let t = solved(&m);
    let keys = [
        span_node_key(NodeId(0), NodeId(1)),
        span_node_key(NodeId(2), NodeId(3)),
        span_node_key(NodeId(4), NodeId(5)),
    ];
    for k in keys {
        assert!(
            t.cyclic.contains(&k),
            "{k:?} が循環に含まれる: {:?}",
            t.cyclic
        );
        assert!(
            !t.members.contains_key(&k),
            "循環した二次部材は逐次伝達の対象から外れる"
        );
    }
}

/// 節点を共有せず交差する 2 本は、受け側・架け側を決められないので `crossings` に入る。
#[test]
fn crossing_without_shared_node_is_reported() {
    let mut m = base_model();
    for (i, c) in [
        [0.0, 2000.0, 0.0],
        [6000.0, 2000.0, 0.0],
        [3000.0, 0.0, 0.0],
        [3000.0, 4000.0, 0.0],
    ]
    .iter()
    .enumerate()
    {
        m.nodes.push(node(i as u32, c[0], c[1], c[2]));
    }
    m.unassigned_joists.push(joist(0, 1, "A"));
    m.unassigned_joists.push(joist(2, 3, "B"));

    let crossings = super::secondary_crossings(&m);
    assert_eq!(crossings.len(), 1, "交差 1 組: {crossings:?}");
}

/// 荷重を持たない二次部材は、端部の行き先が決まらなくても報告しない。
///
/// 断面が未割当なら自重も床分配も載らず、失う荷重がない。形だけ置かれた支持点で
/// 解析前チェックのエラーを出さないための扱いである（`solve` 末尾の判定）。
#[test]
fn floating_joist_without_load_is_not_reported() {
    let mut m = base_model();
    m.nodes.push(node(0, 0.0, 0.0, 0.0));
    m.nodes.push(node(1, 4000.0, 0.0, 0.0));
    m.unassigned_joists.push(SecondaryMember {
        gravity_end_shares: None,
        end_support: Default::default(),
        kind: SecondaryMemberKind::Joist,
        nodes: [NodeId(0), NodeId(1)],
        section: None,
        name: "SB".into(),
    });

    let t = solved(&m);
    let key = span_node_key(NodeId(0), NodeId(1));
    let sm = t.members.get(&key).expect("小梁");
    assert_eq!(sm.supports, [SupportAt::Unresolved; 2]);
    assert_eq!(sm.reactions, [0.0, 0.0]);
    assert!(t.unresolved.is_empty(), "{:?}", t.unresolved);
}

/// 実部材化された二次部材（両端を持つ実 `Beam` がある）は逐次伝達の対象外。
#[test]
fn materialized_joist_is_skipped() {
    let mut m = base_model();
    m.nodes.push(node(0, 0.0, 0.0, 0.0));
    m.nodes.push(node(1, 4000.0, 0.0, 0.0));
    m.elements.push(beam(0, 0, 1));
    m.unassigned_joists.push(joist(0, 1, "SB"));

    let t = solved(&m);
    assert!(t.members.is_empty(), "実部材化済みは対象外");
}

/// 片持ち小梁（`free_at` が自由端の位置）を作る。
fn cantilever(a: u32, b: u32, free_at: usize, name: &str) -> SecondaryMember {
    let mut sm = joist(a, b, name);
    sm.end_support = if free_at == 0 {
        [EndSupport::Free, EndSupport::Supported]
    } else {
        [EndSupport::Supported, EndSupport::Free]
    };
    sm
}

/// 大梁 0-1 に載る片持ち小梁の自重は、基端の反力だけになって主架構へ渡る。
#[test]
fn cantilever_joist_transfers_to_base_only() {
    let mut m = base_model();
    for (i, c) in [
        [0.0, 0.0, 0.0],       // 0 大梁端
        [6000.0, 0.0, 0.0],    // 1 大梁端
        [3000.0, 0.0, 0.0],    // 2 基端（大梁のスパン上）
        [3000.0, 4000.0, 0.0], // 3 自由端
    ]
    .iter()
    .enumerate()
    {
        m.nodes.push(node(i as u32, c[0], c[1], c[2]));
    }
    m.elements.push(beam(0, 0, 1));
    m.unassigned_joists.push(cantilever(2, 3, 1, "SB"));

    let t = solved(&m);
    let key = span_node_key(NodeId(2), NodeId(3));
    let sm = t.members.get(&key).expect("片持ち小梁");
    assert_eq!(
        sm.supports,
        [SupportAt::Primary, SupportAt::Free],
        "基端のみ支持"
    );
    let expected = w_self() * 4000.0;
    assert!((sm.reactions[0] - expected).abs() / expected < 1e-9);
    assert_eq!(sm.reactions[1], 0.0);
    assert!(t.unresolved.is_empty(), "{:?}", t.unresolved);
    assert!(t.cyclic.is_empty());
}

/// 基端が nodes[1] の片持ち小梁も、全反力が基端へ渡る。
#[test]
fn cantilever_joist_base_at_second_node() {
    let mut m = base_model();
    for (i, c) in [
        [0.0, 4000.0, 0.0],    // 0 大梁端
        [6000.0, 4000.0, 0.0], // 1 大梁端
        [3000.0, 0.0, 0.0],    // 2 自由端
        [3000.0, 4000.0, 0.0], // 3 基端（大梁のスパン上）
    ]
    .iter()
    .enumerate()
    {
        m.nodes.push(node(i as u32, c[0], c[1], c[2]));
    }
    m.elements.push(beam(0, 0, 1));
    m.unassigned_joists.push(cantilever(2, 3, 0, "SB"));

    let t = solved(&m);
    let key = span_node_key(NodeId(2), NodeId(3));
    let sm = t.members.get(&key).expect("片持ち小梁");
    assert_eq!(sm.supports, [SupportAt::Free, SupportAt::Primary]);
    let expected = w_self() * 4000.0;
    assert_eq!(sm.reactions[0], 0.0);
    assert!((sm.reactions[1] - expected).abs() / expected < 1e-9);
    assert!(t.unresolved.is_empty(), "{:?}", t.unresolved);
}

/// 先端リブは片持ち小梁の自由端に載れる。リブの反力は自由端を通り、
/// 片持ち小梁の基端へまとめて渡る（荷重は消えない）。
#[test]
fn tip_rib_on_cantilever_free_ends_cascades_to_bases() {
    let mut m = base_model();
    for (i, c) in [
        [0.0, 0.0, 0.0],       // 0 大梁端
        [6000.0, 0.0, 0.0],    // 1 大梁端
        [1000.0, 0.0, 0.0],    // 2 片持ち A 基端（大梁のスパン上）
        [1000.0, 3000.0, 0.0], // 3 片持ち A 自由端
        [5000.0, 0.0, 0.0],    // 4 片持ち B 基端（大梁のスパン上）
        [5000.0, 3000.0, 0.0], // 5 片持ち B 自由端
    ]
    .iter()
    .enumerate()
    {
        m.nodes.push(node(i as u32, c[0], c[1], c[2]));
    }
    m.elements.push(beam(0, 0, 1));
    m.unassigned_joists.push(cantilever(2, 3, 1, "CA"));
    m.unassigned_joists.push(cantilever(4, 5, 1, "CB"));
    m.unassigned_joists.push(joist(3, 5, "RIB"));

    let t = solved(&m);
    let ca = t
        .members
        .get(&span_node_key(NodeId(2), NodeId(3)))
        .expect("片持ち A");
    let cb = t
        .members
        .get(&span_node_key(NodeId(4), NodeId(5)))
        .expect("片持ち B");
    let rib = t
        .members
        .get(&span_node_key(NodeId(3), NodeId(5)))
        .expect("先端リブ");

    for (joist, base) in [(&ca, NodeId(2)), (&cb, NodeId(4))] {
        assert_eq!(joist.supports[1], SupportAt::Free);
        assert_eq!(joist.reactions[1], 0.0, "自由端の反力は 0");
        assert_eq!(joist.nodes[0], base);
    }
    assert!(matches!(rib.supports[0], SupportAt::Secondary { .. }));
    assert!(matches!(rib.supports[1], SupportAt::Secondary { .. }));

    let expected_total = w_self() * (3000.0 * 2.0 + 4000.0);
    let total = ca.reactions[0] + cb.reactions[0];
    assert!(
        (total - expected_total).abs() / expected_total < 1e-9,
        "総和={total} expected={expected_total}"
    );
    assert!(t.unresolved.is_empty(), "{:?}", t.unresolved);
    assert!(t.cyclic.is_empty());
}

/// 取り付く床板の辺荷重は、境界の片持ち小梁（二次部材）へ渡り、反力として
/// 基端の主架構へ流れる。取付き辺の残りは主架構への辺荷重として残る。
#[test]
fn attached_slab_load_reaches_side_joist() {
    use squid_n_core::ids::SlabId;
    use squid_n_core::model::{AreaLoad, LoadTransfer, RegionAnchor, Slab, SlabPlate, SlabShape};
    let w = 0.005_f64;
    let mut m = base_model();
    for (i, c) in [
        [0.0, 0.0, 0.0],    // 0 大梁端（取付き線）
        [6000.0, 0.0, 0.0], // 1 大梁端
        [0.0, 1500.0, 0.0], // 2 小梁の自由端
    ]
    .iter()
    .enumerate()
    {
        m.nodes.push(node(i as u32, c[0], c[1], c[2]));
    }
    m.elements.push(beam(0, 0, 1));
    m.unassigned_joists.push(cantilever(0, 2, 1, "J"));
    m.slabs.push(Slab {
        id: SlabId(0),
        shape: SlabShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(0), NodeId(1)],
                span: [0.0, 1.0],
                transfer: LoadTransfer::Anchor,
            },
            extent: [1500.0, 1500.0],
        },
        plate: SlabPlate {
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            ..Default::default()
        },
    });

    let t = solve(&m, |_| w, true);
    let joist = t
        .members
        .get(&span_node_key(NodeId(0), NodeId(2)))
        .expect("小梁");
    // 左辺の小梁は最寄り負担面積 d²/2 を受ける。
    let joist_slab = w * (1500.0 * 1500.0 / 2.0);
    let expected = joist_slab + w_self() * 1500.0;
    assert!(
        (joist.reactions[0] - expected).abs() / expected < 0.02,
        "基端反力={} expected={expected}",
        joist.reactions[0]
    );
    assert_eq!(joist.reactions[1], 0.0, "自由端の反力は 0");
    assert!(t.unresolved.is_empty(), "{:?}", t.unresolved);

    let leftover_total: f64 = t
        .leftover_region_loads
        .iter()
        .map(|bl| bl.cmq.q_i + bl.cmq.q_j)
        .sum();
    let rest = w * (6000.0 * 1500.0 - 1500.0 * 1500.0 / 2.0);
    assert!(
        (leftover_total - rest).abs() / rest < 0.02,
        "取付き辺 {leftover_total} expected={rest}"
    );
}
