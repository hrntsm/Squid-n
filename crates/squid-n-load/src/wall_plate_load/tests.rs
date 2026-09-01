//! 要素にならない壁版の自重分配のテスト。

use super::*;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, WallPlateId, WallRegionId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, MaterialCategory,
    Node, SecondaryMember, SecondaryMemberKind, WallPlate, WallRegion,
};

const T: f64 = 150.0;
const RHO: f64 = 2.4e-9;

fn node(id: u32, x: f64, z: f64) -> Node {
    Node {
        id: NodeId(id),
        coord: [x, 0.0, z],
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    }
}

fn beam(id: u32, a: u32, b: u32) -> ElementData {
    ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
        section: Some(SectionId(1)),
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

fn plate(id: u32, boundary: [u32; 4]) -> WallPlate {
    WallPlate {
        id: WallPlateId(id),
        shape: WallPlateShape::Enclosed {
            boundary: boundary.into_iter().map(NodeId).collect(),
        },
        section: Some(SectionId(0)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings: Vec::new(),
        three_side_slit: false,
    }
}

/// 4m×3m の 1 構面。左右に柱、上下に大梁、中央 x=2000 に間柱 1 本。
fn bay() -> Model {
    let mut m = Model::default();
    for (id, x, z) in [
        (0, 0.0, 0.0),
        (1, 4000.0, 0.0),
        (2, 4000.0, 3000.0),
        (3, 0.0, 3000.0),
        (4, 2000.0, 0.0),
        (5, 2000.0, 3000.0),
    ] {
        m.nodes.push(node(id, x, z));
    }
    // 柱 2 本・大梁 2 本（間柱の位置では分割しない）。
    m.elements = vec![beam(0, 0, 3), beam(1, 1, 2), beam(2, 0, 1), beam(3, 3, 2)];
    let mut wall_sec = squid_n_core::section_shape::SectionShape::RcWall {
        thickness: T,
        ps: 0.0025,
    }
    .to_section(SectionId(0), "W150".into());
    wall_sec.material = Some(MaterialId(0));
    let mut beam_sec = wall_sec.clone();
    beam_sec.id = SectionId(1);
    m.sections = vec![wall_sec, beam_sec];
    m.materials = vec![Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        category: MaterialCategory::Concrete,
        young: 22000.0,
        poisson: 0.2,
        density: RHO,
        shear: None,
        fc: Some(24.0),
        fy: None,
    }];
    m
}

/// 壁全体（4000×3000×150）の自重 [N]。
fn full_weight() -> f64 {
    4000.0 * 3000.0 * T * RHO * squid_n_core::units::GRAVITY_MM_S2
}

/// 間柱で 2 枚に分割された壁領域を作る。
fn split_by_post() -> Model {
    let mut m = bay();
    m.wall_plates = vec![plate(0, [0, 4, 5, 3]), plate(1, [4, 1, 2, 5])];
    m.wall_regions = vec![WallRegion {
        id: WallRegionId(0),
        name: String::new(),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        wall_plate_ids: vec![WallPlateId(0), WallPlateId(1)],
        posts: vec![SecondaryMember {
            kind: SecondaryMemberKind::Post,
            nodes: [NodeId(4), NodeId(5)],
            section: Some(SectionId(1)),
            name: "P1".into(),
        }],
    }];
    m
}

/// 壁版が壁領域全体を覆う場合、壁エレメントになるので分配しない。
#[test]
fn 領域を覆う壁版は分配の対象外() {
    let mut m = bay();
    m.wall_plates = vec![plate(0, [0, 1, 2, 3])];
    m.wall_regions = vec![WallRegion {
        id: WallRegionId(0),
        name: String::new(),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        wall_plate_ids: vec![WallPlateId(0)],
        posts: Vec::new(),
    }];
    assert!(m.wall_plate_covers_region(&m.wall_plates[0]));
    let out = distribute_enclosed_wall_plates(&m);
    assert!(out.posts.is_empty());
    assert!(out.primary.is_empty());
}

/// 間柱で分割された壁版は、左右の鉛直辺（柱・間柱）へ半分ずつ配る。
///
/// 中央の間柱は両側の壁版から半分ずつ受けるので、壁全体の 1/2 を負担する。
/// 左右の柱は 1/4 ずつで、総和は保存する。
#[test]
fn 間柱で分割された壁は左右の鉛直辺へ半分ずつ配る() {
    let m = split_by_post();
    let out = distribute_enclosed_wall_plates(&m);

    let post = out
        .posts
        .get(&(NodeId(4), NodeId(5)))
        .expect("間柱が荷重を受ける");
    let post_total: f64 = post
        .member_loads
        .iter()
        .map(|l| match *l {
            MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
            MemberLoadKind::Point { p, .. } => p,
        })
        .sum();
    assert!(
        (post_total - full_weight() / 2.0).abs() / full_weight() < 1e-9,
        "間柱は壁全体の 1/2 を受ける: {post_total}"
    );

    let primary_total: f64 = out
        .primary
        .iter()
        .map(|bl| match bl.shape {
            LoadShape::Uniform { w } => w * edge_len(&m, bl),
            _ => panic!("鉛直辺は等分布"),
        })
        .sum();
    assert!(
        (primary_total - full_weight() / 2.0).abs() / full_weight() < 1e-9,
        "左右の柱は合わせて壁全体の 1/2 を受ける: {primary_total}"
    );
    assert!(
        (post_total + primary_total - full_weight()).abs() / full_weight() < 1e-9,
        "総和保存"
    );
}

fn edge_len(model: &Model, bl: &BeamLoad) -> f64 {
    let LoadTarget::Span { nodes, .. } = bl.target else {
        panic!("Span 以外は出さない");
    };
    let a = model.nodes[nodes[0].index()].coord;
    let b = model.nodes[nodes[1].index()].coord;
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// 鉛直辺に支持部材が無い壁版（腰壁）は、最も低い水平な辺へ全量を配る。
#[test]
fn 鉛直辺に支持が無い壁版は下の梁が全量を受ける() {
    let mut m = bay();
    // 柱に載らない位置（x=1000〜3000）の腰壁。上辺は自由端。
    m.nodes.push(node(6, 1000.0, 0.0));
    m.nodes.push(node(7, 3000.0, 0.0));
    m.nodes.push(node(8, 3000.0, 900.0));
    m.nodes.push(node(9, 1000.0, 900.0));
    m.wall_plates = vec![plate(0, [6, 7, 8, 9])];
    m.wall_regions = vec![WallRegion {
        id: WallRegionId(0),
        name: String::new(),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        wall_plate_ids: vec![WallPlateId(0)],
        posts: Vec::new(),
    }];

    let out = distribute_enclosed_wall_plates(&m);
    assert!(out.posts.is_empty(), "間柱は無い");
    assert_eq!(out.primary.len(), 1, "下辺 1 本だけが受ける");
    let bl = &out.primary[0];
    let LoadTarget::Span { nodes, .. } = bl.target else {
        panic!("Span で出す");
    };
    assert_eq!(nodes, [NodeId(6), NodeId(7)], "最も低い水平な辺");
    let total = match bl.shape {
        LoadShape::Uniform { w } => w * edge_len(&m, bl),
        _ => panic!("等分布"),
    };
    let expect = 2000.0 * 900.0 * T * RHO * squid_n_core::units::GRAVITY_MM_S2;
    assert!((total - expect).abs() / expect < 1e-9, "総和保存: {total}");
}

/// 地震用重量の集計は、荷重の分配と同じ辺の割り当てを共有する。
/// 矩形の壁版が左右の鉛直辺で受ける場合、上下 2 節点ずつへ 1/4 ずつとなり、
/// 壁エレメントの頂点等分配と一致する。
#[test]
fn 地震用重量は辺の両端へ半分ずつ配り総和を保存する() {
    let m = split_by_post();
    let mut node_weight = vec![0.0; m.nodes.len()];
    accumulate_enclosed_wall_seismic_weight(&m, &mut node_weight);

    let sum: f64 = node_weight.iter().sum();
    assert!(
        (sum - full_weight()).abs() / full_weight() < 1e-9,
        "総和保存: {sum}"
    );
    // 間柱の上下端は壁全体の 1/4 ずつ、柱側の 4 節点は 1/8 ずつ。
    for n in [4, 5] {
        assert!(
            (node_weight[n] - full_weight() / 4.0).abs() / full_weight() < 1e-9,
            "間柱端 {n}: {}",
            node_weight[n]
        );
    }
    for n in [0, 1, 2, 3] {
        assert!(
            (node_weight[n] - full_weight() / 8.0).abs() / full_weight() < 1e-9,
            "柱側 {n}: {}",
            node_weight[n]
        );
    }
}
