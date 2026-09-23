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

fn plate(id: u32) -> WallPlate {
    WallPlate {
        self_weight_shares: Vec::new(),
        id: WallPlateId(id),
        shape: WallPlateShape::Enclosed,
        section: Some(SectionId(0)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings: Vec::new(),
        loads: vec![],
        slit: Default::default(),
    }
}

/// 4 節点境界の囲まれた壁版を、境界支持部材と明示負担率を伴って追加する。
fn add_plate(m: &mut Model, id: u32, boundary: [u32; 4], shares: [f64; 4]) {
    let nodes: Vec<NodeId> = boundary.into_iter().map(NodeId).collect();
    let mut p = plate(id);
    p.self_weight_shares = shares.to_vec();
    m.add_enclosed_wall_plate_from_nodes(&nodes, p);
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
    // 割当領域を間柱で分割するため、間柱を安定 ID ＋取付き位置で持たせる。
    m.unassigned_posts.push(SecondaryMember {
        gravity_end_shares: Some([0.5, 0.5]),
        id: squid_n_core::ids::SecondaryMemberId(0),
        kind: SecondaryMemberKind::Post,
        ends: squid_n_core::model::SecondaryMemberEnds::Supported([
            squid_n_core::model::SecondaryMemberAnchor {
                support: squid_n_core::model::SupportMemberId::Primary(ElemId(2)),
                position: 0.5,
            },
            squid_n_core::model::SecondaryMemberAnchor {
                support: squid_n_core::model::SupportMemberId::Primary(ElemId(3)),
                position: 0.5,
            },
        ]),
        section: Some(SectionId(1)),
        name: "P1".into(),
    });
    m.rebuild_wall_assignment_regions();
    m.assign_enclosed_wall_plate_to_matching_region(
        &[NodeId(0), NodeId(4), NodeId(5), NodeId(3)],
        plate(0),
    )
    .expect("下流側の割当領域");
    m.assign_enclosed_wall_plate_to_matching_region(
        &[NodeId(4), NodeId(1), NodeId(2), NodeId(5)],
        plate(1),
    )
    .expect("上流側の割当領域");
    // 左の壁版（境界 0-4-5-3）は柱際（辺 3）と間柱際（辺 1）、右の壁版
    // （境界 1-2-5-4）は柱際（辺 0）と間柱際（辺 2）へそれぞれ半分ずつ配る。
    // 負担率は割当領域が返す境界の並び順に対応する。
    for p in &mut m.wall_plates {
        p.self_weight_shares = if p.id.0 == 0 {
            vec![0.0, 0.5, 0.0, 0.5]
        } else {
            vec![0.5, 0.0, 0.5, 0.0]
        };
    }
    // 割当領域の境界は安定 ID で間柱を参照するため、間柱は未割当から壁領域へ移す
    // （重複させない。二重計上を防ぐ）。
    let post = m.unassigned_posts.pop().expect("間柱");
    m.wall_regions = vec![WallRegion {
        id: WallRegionId(0),
        name: String::new(),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        wall_plate_ids: vec![WallPlateId(0), WallPlateId(1)],
        posts: vec![post],
    }];
    m
}

/// 壁版が壁領域全体を覆う場合、壁エレメントになるので分配しない。
#[test]
fn 領域を覆う壁版は分配の対象外() {
    let mut m = bay();
    add_plate(&mut m, 0, [0, 1, 2, 3], [1.0, 0.0, 0.0, 0.0]);
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
    add_plate(&mut m, 0, [0, 1, 2, 3], [1.0, 0.0, 0.0, 0.0]);
    m.wall_plates[0].section = None;
    m.wall_plates[0].loads = vec![AreaLoad {
        kind: "増打ち".into(),
        value: 1.0e-3,
    }];
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
        .get(&squid_n_core::ids::SecondaryMemberId(0))
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

/// 負担率を指定した辺がスリットで切れていると、その壁版は自重を伝えられない。
///
/// 切れた辺を避けて別の辺へ振り替えることはしない（ADR 0018）。
#[test]
fn 指定した辺がスリットで切れていれば別の辺へ振り替えない() {
    let mut m = split_by_post();
    // 左の壁版（節点 0-4-5-3）の柱側（節点 0 から立ち上がる辺 3）を切る。
    let faces = m.wall_plates[0].column_face_nodes(&m).expect("下辺 2 節点");
    let k = usize::from(faces[0] != NodeId(0));
    m.wall_plates[0].slit.column_face[k] = true;

    assert_eq!(wall_plates_without_load_path(&m), vec![WallPlateId(0)]);
    assert!(edge_shares_with(
        &m,
        &m.wall_plates[0],
        crate::cascade::SelfWeightBasis::Design
    )
    .is_empty());
}

/// 負担率を指定した梁際の辺がスリットで切れていると、その壁版は自重を伝えられない。
///
/// 切れた辺を避けて別の辺へ振り替えることはしない（ADR 0018）。柱際の
/// `指定した辺がスリットで切れていれば別の辺へ振り替えない` と同じ扱いを梁際でも固定する。
#[test]
fn 梁際スリットで切れた辺に負担率を指定すると振り替えない() {
    let mut m = bay();
    // 下辺（辺 0）のみ梁際スリットで切る。残る辺 2 は切れていないが、そこへは振り替えない。
    add_plate(&mut m, 0, [0, 1, 2, 3], [1.0, 0.0, 0.0, 0.0]);
    m.wall_plates[0].slit.beam_face = [true, false];

    assert_eq!(wall_plates_without_load_path(&m), vec![WallPlateId(0)]);
    assert!(edge_shares_with(
        &m,
        &m.wall_plates[0],
        crate::cascade::SelfWeightBasis::Design
    )
    .is_empty());
}

/// 明示した負担率どおりの辺へ配る。左右に柱があっても、指定した辺だけが受ける。
#[test]
fn 指定した辺の負担率どおりに壁自重を配る() {
    let mut m = bay();
    add_plate(&mut m, 0, [0, 1, 2, 3], [0.75, 0.0, 0.25, 0.0]);
    let out = edge_shares_with(
        &m,
        &m.wall_plates[0],
        crate::cascade::SelfWeightBasis::Design,
    );
    assert_eq!(out.len(), 2);
    assert_eq!(out[0].support, SupportMemberId::Primary(ElemId(2)));
    assert_eq!(out[1].support, SupportMemberId::Primary(ElemId(3)));
    assert!((out[0].total - full_weight() * 0.75).abs() < 1e-6);
    assert!((out[1].total - full_weight() * 0.25).abs() < 1e-6);
}

/// 負担率を明示すれば、下辺のみ梁際スリットの非要素壁版は切れていない上辺の梁へ自重を配る。
///
/// `beam_face` の添字は 0 が下辺・1 が上辺に対応し、負担率の辺の並び（境界の順）と
/// 同じであることを、非要素壁版の `edge_shares_with` で固定する。
#[test]
fn 負担率を明示すれば下辺のみ梁際スリットの非要素壁版は切れていない上辺の梁へ自重を配る() {
    let mut m = bay();
    add_plate(&mut m, 0, [0, 1, 2, 3], [0.0, 0.0, 1.0, 0.0]);
    m.wall_plates[0].slit.beam_face = [true, false];

    let out = edge_shares_with(
        &m,
        &m.wall_plates[0],
        crate::cascade::SelfWeightBasis::Design,
    );
    assert_eq!(out.len(), 1);
    assert_eq!(out[0].support, SupportMemberId::Primary(ElemId(3)));
    assert!(out[0].post().is_none(), "上辺は大梁が受ける");
    assert!(
        (out[0].total - full_weight()).abs() / full_weight() < 1e-9,
        "全量が切れていない上辺へ渡る: {}",
        out[0].total
    );
}

/// 負担率を明示すれば、上辺のみ梁際スリットの非要素壁版は切れていない下辺の梁へ自重を配る。
///
/// 上記の下辺のみ梁際スリットのテストと対にし、`beam_face` の添字 0 が下辺・1 が上辺に
/// 対応することを両方向から固定する（添字の取り違えを検出する）。
#[test]
fn 負担率を明示すれば上辺のみ梁際スリットの非要素壁版は切れていない下辺の梁へ自重を配る() {
    let mut m = bay();
    add_plate(&mut m, 0, [0, 1, 2, 3], [1.0, 0.0, 0.0, 0.0]);
    m.wall_plates[0].slit.beam_face = [false, true];

    let out = edge_shares_with(
        &m,
        &m.wall_plates[0],
        crate::cascade::SelfWeightBasis::Design,
    );
    assert_eq!(out.len(), 1);
    assert_eq!(out[0].support, SupportMemberId::Primary(ElemId(2)));
    assert!(out[0].post().is_none(), "下辺は大梁が受ける");
    assert!(
        (out[0].total - full_weight()).abs() / full_weight() < 1e-9,
        "全量が切れていない下辺へ渡る: {}",
        out[0].total
    );
}

/// 自重を持つ壁版は、負担率の未指定・不正値を解析前エラーとし、幾何から配らない。
///
/// 鉛直支持辺がそろった壁版でも分配しないことで、幾何フォールバックが無いことを
/// 固定する（フォールバックがあれば辺が返る）。
#[test]
fn 未指定や不正な負担率は配らず診断する() {
    for shares in [
        vec![],
        vec![1.0],
        vec![0.0; 4],
        vec![0.4, 0.0, 0.4, 0.0],
        vec![1.1, 0.0, -0.1, 0.0],
        vec![f64::NAN, 0.0, 0.0, 0.0],
        vec![f64::INFINITY, 0.0, 0.0, 0.0],
    ] {
        let mut m = split_by_post();
        m.wall_plates[0].self_weight_shares = shares;
        assert_eq!(wall_plates_without_load_path(&m), vec![WallPlateId(0)]);
        assert!(edge_shares_with(
            &m,
            &m.wall_plates[0],
            crate::cascade::SelfWeightBasis::Design
        )
        .is_empty());
    }
}

/// 境界の支持先は割当領域が正本である。辺の全長を覆う大梁に重ねて半分だけ覆う大梁を
/// 足しても、割当領域を再構築すると重複コリニアの辺は 1 本の大梁へまとまり、そこへ
/// 配る（幾何からの支持先再推定をしない）。
///
/// 同じ種類どうしの重なりは解析前チェックがエラーにする（ADR 0021）。面走査は主架構を
/// 先頭にするだけで同種どうしの選択順を定めないため、残る大梁は要素順に依存する。
/// ここでは荷重モジュール単体が割当領域の支持部材をそのまま使うことだけを確かめる。
#[test]
fn 支持区間が重複しても割当領域の支持部材へ配る() {
    let mut m = bay();
    add_plate(&mut m, 0, [0, 1, 2, 3], [1.0, 0.0, 0.0, 0.0]);

    // 下辺を全長で覆う大梁に重ねて、半分だけ覆う大梁を足す。
    m.elements.push(beam(4, 0, 4));
    m.rebuild_wall_assignment_regions();

    assert_eq!(m.wall_assignment_regions.regions.len(), 1, "1 面のまま");
    let boundary = &m.wall_assignment_regions.regions[0].boundary;
    assert_eq!(
        boundary
            .iter()
            .filter(|edge| edge.support == SupportMemberId::Primary(ElemId(2)))
            .count(),
        1,
        "重複コリニアの下辺は要素順で先の大梁 1 本が残る: {boundary:?}"
    );
    assert!(
        !boundary
            .iter()
            .any(|edge| edge.support == SupportMemberId::Primary(ElemId(4))),
        "半分だけ覆う大梁は境界に残らない: {boundary:?}"
    );

    assert!(wall_plates_without_load_path(&m).is_empty());
    let out = edge_shares_with(
        &m,
        &m.wall_plates[0],
        crate::cascade::SelfWeightBasis::Design,
    );
    assert_eq!(out.len(), 1);
    assert_eq!(
        out[0].support,
        SupportMemberId::Primary(ElemId(2)),
        "重複する大梁ではなく割当領域の支持部材へ配る"
    );
    assert!(
        (out[0].total - full_weight()).abs() / full_weight() < 1e-9,
        "全量を配る: {}",
        out[0].total
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

/// 地震用重量の集計は、荷重の分配と同じ辺の割り当てを共有する。
/// 矩形の壁版が左右の鉛直辺で受ける場合、上下 2 節点ずつへ 1/4 ずつとなり、
/// 壁エレメントの頂点等分配と一致する。
#[test]
fn 地震用重量は辺の両端へ半分ずつ配り総和を保存する() {
    let mut m = split_by_post();
    // 壁版自重の分配だけを確認するため、間柱自身の自重は外す。
    for p in m.wall_regions.iter_mut().flat_map(|r| r.posts.iter_mut()) {
        p.section = None;
    }
    let mut node_weight = vec![0.0; m.nodes.len()];
    accumulate_wall_and_secondary_seismic_weight(&m, &mut node_weight).unwrap();

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

/// 柱の材軸に並走する間柱は、割当領域の再構築で境界から外れ、柱（主架構）が残る。
///
/// 逐次伝達の `support_of`・小梁の並走大梁優先と同じ考え方で、辺が柱に覆われていれば
/// 主架構が支持部材として選ばれる。
#[test]
fn 柱に並走する間柱は柱の荷重を奪わない() {
    let mut m = split_by_post();
    // 左の柱（節点 0-3）と同じ材軸に、柱へアンカーした間柱を 1 本足す（重複モデル化）。
    m.wall_regions[0].posts.push(SecondaryMember {
        gravity_end_shares: None,
        id: squid_n_core::ids::SecondaryMemberId(1),
        kind: SecondaryMemberKind::Post,
        ends: squid_n_core::model::SecondaryMemberEnds::Supported([
            squid_n_core::model::SecondaryMemberAnchor {
                support: SupportMemberId::Primary(ElemId(0)),
                position: 0.0,
            },
            squid_n_core::model::SecondaryMemberAnchor {
                support: SupportMemberId::Primary(ElemId(0)),
                position: 1.0,
            },
        ]),
        section: Some(SectionId(1)),
        name: "P0".into(),
    });
    m.rebuild_wall_assignment_regions();

    let boundaries: Vec<_> = m
        .wall_assignment_regions
        .regions
        .iter()
        .map(|region| region.boundary.as_slice())
        .collect();
    assert!(
        boundaries.iter().any(|boundary| boundary
            .iter()
            .any(|edge| edge.support == SupportMemberId::Primary(ElemId(0)))),
        "柱側の鉛直辺は主架構が受け続ける: {boundaries:?}"
    );
    assert!(
        !boundaries.iter().any(|boundary| boundary
            .iter()
            .any(|edge| { edge.support == SupportMemberId::Secondary(SecondaryMemberId(1)) })),
        "柱に並走する間柱は割当領域の境界に残らない: {boundaries:?}"
    );

    let out = distribute_enclosed_wall_plates(&m);
    assert!(
        !out.posts
            .contains_key(&squid_n_core::ids::SecondaryMemberId(1)),
        "柱に並走する間柱は荷重を受けない"
    );
    assert_eq!(out.primary.len(), 2, "柱側の鉛直辺は主架構が受け続ける");
}

/// 支持部材のある辺を持つ壁版は診断の対象にならない。
#[test]
fn 行き先のある壁版は診断に出ない() {
    assert!(wall_plates_without_load_path(&split_by_post()).is_empty());
}

/// 中央間柱が梁の材軸中間にアンカーし、その位置にモデル節点が無い 1 構面。
/// ノードは四隅のみで、間柱端は節点を持たない。
fn midspan_post_bay() -> Model {
    let mut m = bay();
    // 間柱端の節点（4, 5）を除き、間柱が梁中間にだけ載る状態にする。
    m.nodes.retain(|n| n.id.0 < 4);
    m.unassigned_posts.push(SecondaryMember {
        gravity_end_shares: Some([0.5, 0.5]),
        id: SecondaryMemberId(0),
        kind: SecondaryMemberKind::Post,
        ends: squid_n_core::model::SecondaryMemberEnds::Supported([
            squid_n_core::model::SecondaryMemberAnchor {
                support: SupportMemberId::Primary(ElemId(2)),
                position: 0.5,
            },
            squid_n_core::model::SecondaryMemberAnchor {
                support: SupportMemberId::Primary(ElemId(3)),
                position: 0.5,
            },
        ]),
        section: Some(SectionId(1)),
        name: "P1".into(),
    });
    m
}

/// 間柱端が梁の材軸中間にありモデル節点が無くても、割当領域の `SupportBoundary` から
/// 支持先を解決して自重を配る（幾何からの支持先再推定を撤去したことの回帰）。
#[test]
fn 間柱端が梁中間にある壁版も割当領域の支持部材へ自重を配る() {
    let mut m = midspan_post_bay();
    m.rebuild_wall_assignment_regions();
    let regions: Vec<_> = m
        .wall_assignment_regions
        .regions
        .iter()
        .map(|region| (region.id, region.boundary.clone()))
        .collect();
    assert_eq!(regions.len(), 2, "中央間柱で 2 面");

    for (i, (region_id, boundary)) in regions.iter().enumerate() {
        let post_edge = boundary
            .iter()
            .position(|edge| edge.support == SupportMemberId::Secondary(SecondaryMemberId(0)))
            .expect("間柱が割当領域の境界辺になっている");
        let plate_id = WallPlateId(i as u32);
        let mut p = plate(i as u32);
        let mut shares = vec![0.0; boundary.len()];
        shares[post_edge] = 1.0;
        p.self_weight_shares = shares;
        m.wall_plates.push(p);
        m.wall_assignment_regions
            .get_mut(*region_id)
            .expect("直前に作った割当領域")
            .assignment = squid_n_core::model::PlateAssignment::Plate(plate_id);
    }

    // 支持先は割当領域の SupportBoundary（間柱）と一致する。
    let shares = edge_shares_with(
        &m,
        &m.wall_plates[0],
        crate::cascade::SelfWeightBasis::Design,
    );
    assert_eq!(shares.len(), 1);
    assert_eq!(
        shares[0].support,
        SupportMemberId::Secondary(SecondaryMemberId(0))
    );

    let out = distribute_enclosed_wall_plates(&m);
    let post = out
        .posts
        .get(&SecondaryMemberId(0))
        .expect("間柱が自重を受ける");
    let post_total: f64 = post
        .member_loads
        .iter()
        .map(|l| match *l {
            MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
            MemberLoadKind::Point { p, .. } => p,
        })
        .sum();
    assert!(
        (post_total - full_weight()).abs() / full_weight() < 1e-9,
        "2 枚合計で壁全体の自重が間柱へ渡る: {post_total}"
    );
    assert!(
        out.primary.is_empty(),
        "支持先を間柱に指定した辺は主架構へ配らない: {:?}",
        out.primary
    );
}
