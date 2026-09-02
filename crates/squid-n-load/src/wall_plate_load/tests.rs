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
        loads: vec![],
        slit: Default::default(),
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

/// 壁領域を覆っていても、断面が無く壁エレメントにならない壁版は分配の対象になる。
///
/// 要素経由の自重算定を通らないため、ここで落とすと仕上げ・増打ちの面荷重が
/// どちらの経路も通らずに消える。躯体の自重は断面が無いので 0 のままである。
#[test]
fn 領域を覆っても断面が無ければ分配の対象になる() {
    use squid_n_core::model::AreaLoad;

    let mut m = bay();
    let mut p = plate(0, [0, 1, 2, 3]);
    p.section = None;
    p.loads = vec![AreaLoad {
        kind: "増打ち".into(),
        value: 1.0e-3,
    }];
    m.wall_plates = vec![p];
    m.wall_regions = vec![WallRegion {
        id: WallRegionId(0),
        name: String::new(),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        wall_plate_ids: vec![WallPlateId(0)],
        posts: Vec::new(),
    }];
    assert!(m.wall_plate_covers_region(&m.wall_plates[0]));
    assert!(!m.wall_plate_becomes_element(&m.wall_plates[0]));

    let out = distribute_enclosed_wall_plates(&m);
    let primary_total: f64 = out
        .primary
        .iter()
        .map(|bl| match bl.shape {
            LoadShape::Uniform { w } => w * edge_len(&m, bl),
            _ => panic!("鉛直辺は等分布"),
        })
        .sum();
    let post_total: f64 = out
        .posts
        .values()
        .flat_map(|p| p.member_loads.iter())
        .map(|l| match *l {
            MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
            MemberLoadKind::Point { p, .. } => p,
        })
        .sum();
    let total = primary_total + post_total;
    let expected = m
        .wall_plate_self_weight(&m.wall_plates[0], &m)
        .expect("仕上げ分の自重が求まる");
    assert!(expected > 0.0, "仕上げ分の自重が 0 になっている");
    assert!(
        (total - expected).abs() / expected < 1e-9,
        "分配の総和={total} expected={expected}"
    );
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

/// 柱際にスリットを入れると、その鉛直辺は自重を受けない。
///
/// 分割壁は左右の鉛直辺（柱と間柱）へ半分ずつ配るのが既定だが、片側を切ると
/// 支持する鉛直辺が 1 つになり、規則は「もっとも低い水平な辺へ全量」へ移る。
#[test]
fn 柱際スリットのある鉛直辺は自重を受けない() {
    let mut m = split_by_post();
    // 左の壁版（節点 0-4-5-3）の柱側（節点 0 から立ち上がる辺）を切る。
    let faces = m.wall_plates[0].column_face_nodes(&m).expect("下辺 2 節点");
    let k = usize::from(faces[0] != NodeId(0));
    m.wall_plates[0].slit.column_face[k] = true;

    let out = distribute_enclosed_wall_plates(&m);
    // 間柱は右の壁版からのぶんだけを受ける（左の壁版は鉛直辺が 1 つになり、
    // 下の大梁へ全量が回るため）。
    let post_total: f64 = out
        .posts
        .get(&(NodeId(4), NodeId(5)))
        .map(|p| {
            p.member_loads
                .iter()
                .map(|l| match *l {
                    MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
                    MemberLoadKind::Point { p, .. } => p,
                })
                .sum()
        })
        .unwrap_or(0.0);
    assert!(
        (post_total - full_weight() / 4.0).abs() / full_weight() < 1e-9,
        "間柱が受けるのは右の壁版の半分だけ: {post_total}"
    );

    // 総和は保存する（切れた辺へ配らないだけで、重量は失わない）。
    let primary_total: f64 = out
        .primary
        .iter()
        .map(|bl| match bl.shape {
            LoadShape::Uniform { w } => w * edge_len(&m, bl),
            _ => panic!("等分布のみ"),
        })
        .sum();
    assert!(
        (post_total + primary_total - full_weight()).abs() / full_weight() < 1e-9,
        "総和保存: {post_total} + {primary_total}"
    );
}

/// 下辺の梁際にスリットを入れると、自重は上の梁へ回る。
///
/// 鉛直辺に支持部材が無い壁版は既定で「もっとも低い水平な辺」が全量を受けるが、
/// その辺が切れていれば次に低い（＝上の）辺が受ける。三方スリットの垂れ壁型に
/// あたる形である。
#[test]
fn 下辺の梁際スリットは自重を上の梁へ回す() {
    let mut m = bay();
    // 間柱を置かず、鉛直辺に支持を持たない壁版にする（下の梁が全量を受ける形）。
    m.wall_plates.push(plate(0, [0, 1, 2, 3]));
    m.wall_regions.push(WallRegion {
        id: WallRegionId(0),
        name: String::new(),
        // 壁領域の境界を壁版と別にして、覆っていない扱いにする。
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3), NodeId(4)],
        wall_plate_ids: vec![WallPlateId(0)],
        posts: Vec::new(),
    });
    // 柱際を切って鉛直辺の支持を外し、水平な辺で受ける形にする。
    m.wall_plates[0].slit.column_face = [true, true];

    let bottom = distribute_enclosed_wall_plates(&m);
    let bottom_edge = bottom.primary.first().expect("下の梁が全量を受ける").target;
    let LoadTarget::Span { nodes, .. } = bottom_edge else {
        panic!("Span")
    };
    let z_bottom = m.nodes[nodes[0].index()].coord[2];
    assert!(z_bottom.abs() < 1e-9, "既定では下辺（z=0）が受ける");

    // 下辺を切ると、上辺（z=3000）が受ける。
    m.wall_plates[0].slit.beam_face = [true, false];
    let top = distribute_enclosed_wall_plates(&m);
    let LoadTarget::Span { nodes, .. } = top.primary.first().expect("上の梁が受ける").target
    else {
        panic!("Span")
    };
    let z_top = m.nodes[nodes[0].index()].coord[2];
    assert!(
        (z_top - 3000.0).abs() < 1e-9,
        "下辺が切れれば上辺が受ける: {z_top}"
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

/// 柱の材軸に並走する間柱は、柱の荷重を奪わない（主架構を優先する）。
///
/// 逐次伝達の `support_of`・小梁の並走大梁優先と同じ考え方で、辺が柱・大梁に
/// 覆われていればそこで終端する。
#[test]
fn 柱に並走する間柱は柱の荷重を奪わない() {
    let mut m = split_by_post();
    // 左の柱（節点 0-3）と同じ位置に間柱を 1 本足す（重複モデル化）。
    m.wall_regions[0].posts.push(SecondaryMember {
        kind: SecondaryMemberKind::Post,
        nodes: [NodeId(0), NodeId(3)],
        section: Some(SectionId(1)),
        name: "P0".into(),
    });

    let out = distribute_enclosed_wall_plates(&m);
    assert!(
        !out.posts.contains_key(&(NodeId(0), NodeId(3))),
        "柱に並走する間柱は荷重を受けない"
    );
    assert_eq!(out.primary.len(), 2, "柱側の鉛直辺は主架構が受け続ける");
}

/// 下に大梁も間柱も無い壁版は、自重の行き先が決まらないので何も配らない。
///
/// 行き先の無い節点荷重は `DofMap` が無視するため、黙って落とすと荷重タブには
/// 見えるのに解析から消える。解析前チェックがエラーで止める対象になる。
#[test]
fn 行き先の無い壁版は配らずに診断へ回す() {
    let mut m = bay();
    // 宙に浮いた壁版（4 隅とも柱・大梁の材軸から外れている）。
    for (id, x, z) in [
        (6, 1000.0, 1000.0),
        (7, 3000.0, 1000.0),
        (8, 3000.0, 2000.0),
        (9, 1000.0, 2000.0),
    ] {
        m.nodes.push(node(id, x, z));
    }
    m.wall_plates = vec![plate(0, [6, 7, 8, 9])];
    m.wall_regions = vec![WallRegion {
        id: WallRegionId(0),
        name: String::new(),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        wall_plate_ids: vec![WallPlateId(0)],
        posts: Vec::new(),
    }];

    let out = distribute_enclosed_wall_plates(&m);
    assert!(out.posts.is_empty());
    assert!(out.primary.is_empty(), "行き先が無いので何も配らない");
    assert_eq!(
        wall_plates_without_load_path(&m),
        vec![WallPlateId(0)],
        "診断が拾う"
    );
}

/// 支持部材のある辺を持つ壁版は診断の対象にならない。
#[test]
fn 行き先のある壁版は診断に出ない() {
    assert!(wall_plates_without_load_path(&split_by_post()).is_empty());
}
