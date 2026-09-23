use super::*;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, FloorRegionId, MaterialId, SectionId, WallPlateId, WallRegionId};
use squid_n_core::model::{
    DamperSpec, ElementData, EndCondition, FloorRegion, ForceRegime, LoadCase, LoadCaseKind,
    LoadCfg, LoadTransfer, LocalAxis, Material, MaterialCategory, MemberLoad, NodalLoad, Node,
    RigidZone, SecondaryMember, SecondaryMemberKind, Section, WallPlate, WallPlateShape,
    WallRegion,
};

/// 2 層 × 1 スパンの平面ラーメン（各レベル 2 節点）。
fn two_story_model() -> Model {
    let mut model = Model::default();
    let coords = [
        [0.0, 0.0, 0.0],
        [6000.0, 0.0, 0.0],
        [0.0, 0.0, 3500.0],
        [6000.0, 0.0, 3500.0],
        [0.0, 0.0, 7000.0],
        [6000.0, 0.0, 7000.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if i < 2 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.sections.push(Section {
        id: SectionId(0),
        name: "S".into(),
        area: 10000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SN400B".into(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: None,
    });
    // 柱4 + 梁2
    let conn: [(u32, u32); 6] = [(0, 2), (1, 3), (2, 4), (3, 5), (2, 3), (4, 5)];
    for (i, (a, b)) in conn.iter().enumerate() {
        model.elements.push(ElementData {
            id: ElemId(i as u32),
            kind: ElementKind::Beam,
            nodes: [NodeId(*a), NodeId(*b)].into_iter().collect(),
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "DL".into(),
        nodal: vec![NodalLoad::manual(
            NodeId(4),
            [0.0, 0.0, -50000.0, 0.0, 0.0, 0.0],
        )],
        member: vec![MemberLoad::manual(
            ElemId(4),
            [0.0, 0.0, -1.0],
            MemberLoadKind::Distributed {
                a: 0.0,
                b: 6000.0,
                w1: 10.0,
                w2: 10.0,
            },
        )],
    });
    model
}

/// 生成結果の剛床拘束から、階 `story` のスレーブ節点を取り出す。
/// 剛床は階ではなく `Constraint::RigidDiaphragm` が保持する。
fn gen_slaves(gen: &StoryGenResult, story: StoryId) -> Vec<NodeId> {
    gen.constraints
        .iter()
        .find_map(|c| match c {
            Constraint::RigidDiaphragm {
                story: s, slaves, ..
            } if *s == story => Some(slaves.clone()),
            _ => None,
        })
        .unwrap_or_default()
}

/// 階が未定義のモデルから初期化したときの既定の階名は**床基準**である。
///
/// 階は床であり、最下レベル（基部）も階として作る。したがって下から順に
/// `1F`・`2F` … となる。ST-Bridge の `StbStory` も床基準のため、取り込んだ
/// モデルとアプリ内で作ったモデルで階名の意味が一致する。
/// 利用者が名前を付けた階は上書きしない。
#[test]
fn test_generated_story_names_are_floor_based() {
    let model = two_story_model();
    assert!(model.stories.is_empty(), "階が未定義の状態から生成する");
    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    let names: Vec<&str> = gen.stories.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names, vec!["1F", "2F", "3F"]);

    // 利用者が付けた階名は再生成でも保たれる。
    let mut named = model.clone();
    named.stories = gen.stories.clone();
    named.stories[1].name = "2FL".into();
    let regen = generate_stories(&named, Some(LoadCaseId(0))).unwrap();
    let names: Vec<&str> = regen.stories.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names, vec!["1F", "2FL", "3F"]);
}

/// 階（床）は基部を含めて 3 つ、層はその間の 2 つ。
#[test]
fn test_generate_two_stories() {
    let model = two_story_model();
    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    assert_eq!(gen.stories.len(), 3, "基部の床を含めて 3 階");
    assert_eq!(gen.stories[0].elevation, 0.0, "先頭は基部の床");
    assert_eq!(gen.stories[1].elevation, 3500.0);
    assert_eq!(gen.stories[2].elevation, 7000.0);
    // 各階 2 節点 → 代表節点(慣性力重心)を新規生成 + スレーブ2（既存節点は全てスレーブ）
    assert_eq!(gen.stories[1].node_ids.len(), 2);
    assert_eq!(gen_slaves(&gen, StoryId(0)).len(), 2, "基部の床の剛床");
    assert_eq!(gen_slaves(&gen, StoryId(1)).len(), 2);
    assert_eq!(gen_slaves(&gen, StoryId(2)).len(), 2);
    assert_eq!(gen.constraints.len(), 3);
    // 基部節点は基部の床に属する（柱脚・基礎梁の伏図と数量のため）。
    assert_eq!(gen.node_story[0], Some(StoryId(0)));
    assert_eq!(gen.node_story[2], Some(StoryId(1)));
    assert_eq!(gen.node_story[4], Some(StoryId(2)));
    // 重量: 1F = 梁分布荷重 10 N/mm × 6000 = 60 kN + 自重、2F = 節点荷重 50 kN + 自重
    let w1 = gen.stories[1].seismic_weight.unwrap();
    let w2 = gen.stories[2].seismic_weight.unwrap();
    assert!(w1 > 60000.0, "w1={}", w1);
    assert!(w2 > 50000.0, "w2={}", w2);

    // 代表節点は新規生成（既存節点数=6 の末尾連番）。基部の床の分を含めて 3 つ。
    assert_eq!(gen.rep_nodes.len(), 3);
    assert_eq!(gen.generated_masters, vec![NodeId(6), NodeId(7), NodeId(8)]);
    // 基部の代表節点は水平にも拘束する（柱脚が全て支点で拘束されており、
    // 剛床を通じて水平剛性が写らないため。自由なままだと特異行列になる）。
    // 面内回転 Rz も拘束する。並進が写らない以上、剛床としての剛性は Rz にも
    // 写らず、回転慣性 j だけを持つ自由度が残って偽の低次モードを生むため。
    let base_rep = &gen.rep_nodes[0];
    assert!(base_rep.restraint.is_fixed(squid_n_core::dof::Dof::Ux));
    assert!(base_rep.restraint.is_fixed(squid_n_core::dof::Dof::Uy));
    assert!(base_rep.restraint.is_fixed(squid_n_core::dof::Dof::Rz));

    for rep in &gen.rep_nodes[1..] {
        // CorrectedLumped(既定): 柱梁の密度自重は解析の質量行列に部材密度質量として
        // 計上されるため控除され、荷重ケース分（節点・部材荷重）のみが質点質量として残る。
        let mass = rep
            .mass
            .expect("CorrectedLumped: 荷重ケース分の質点質量が設定される");
        assert_eq!(mass[0], mass[1], "並進質量 Ux=Uy");
        assert_eq!(mass[2], 0.0);
        assert_eq!(mass[3], 0.0);
        assert_eq!(mass[4], 0.0);
        assert!(mass[0] > 0.0, "mass[0]={}", mass[0]);
        assert!(rep.restraint.is_fixed(squid_n_core::dof::Dof::Uz));
        assert!(rep.restraint.is_fixed(squid_n_core::dof::Dof::Rx));
        assert!(rep.restraint.is_fixed(squid_n_core::dof::Dof::Ry));
        assert!(!rep.restraint.is_fixed(squid_n_core::dof::Dof::Ux));
        assert!(!rep.restraint.is_fixed(squid_n_core::dof::Dof::Uy));
        assert!(!rep.restraint.is_fixed(squid_n_core::dof::Dof::Rz));
    }
    assert_eq!(gen.rep_nodes[0].story, Some(StoryId(0)));
    assert_eq!(gen.rep_nodes[1].story, Some(StoryId(1)));
    assert_eq!(gen.rep_nodes[2].story, Some(StoryId(2)));
    // 2FL は左右対称な自重＋分布荷重のみなので慣性力重心の X は中央(3000)になる。
    assert!((gen.rep_nodes[1].coord[0] - 3000.0).abs() < 1e-6);
    // 3FL は節点荷重(50kN)が NodeId(4)(x=0)側のみに掛かる非対称配置なので、
    // 慣性力重心は x=0 側へ偏る(単純な幾何重心 3000 とは一致しない)。
    // 手計算(設計単位体積重量 78.5e-6 N/mm³): nw4=53728.75, nw5=3728.75,
    // gx = nw5*6000/(nw4+nw5) = 389.3747552538833
    assert!(
        (gen.rep_nodes[2].coord[0] - 389.3747552538833).abs() < 1e-6,
        "{}",
        gen.rep_nodes[2].coord[0]
    );
    assert_eq!(gen.rep_nodes[0].coord[2], 0.0);
    assert_eq!(gen.rep_nodes[1].coord[2], 3500.0);
    assert_eq!(gen.rep_nodes[2].coord[2], 7000.0);
}

/// 非構造節点（要素が接続しない床・小梁の節点）は、剛床のスレーブに含まれていても
/// 代表節点の可動性の判定に数えない。
///
/// 1FL が基部にある建物で 1FL の床に小梁があると、基部の剛床は「水平拘束された柱脚」と
/// 「拘束の無い小梁支持点」の混在になる。後者は解析自由度を持たない（剛性を写さない）
/// ため、これを自由と数えると代表節点の Ux・Uy が剛性ゼロの独立自由度として残り、
/// 剛性行列が特異になる。
#[test]
fn test_base_master_ignores_non_structural_slaves() {
    let mut model = two_story_model();
    // 基部レベルに小梁の支持点を足す（要素は接続しない＝非構造節点、拘束なし）。
    let free_id = NodeId(model.nodes.len() as u32);
    model.nodes.push(Node {
        id: free_id,
        coord: [3000.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    let ends = squid_n_core::model::SecondaryMemberEnds::Detached([
        model.nodes[0].coord,
        model.nodes[free_id.index()].coord,
    ]);
    model.unassigned_joists.push(SecondaryMember {
        gravity_end_shares: None,
        id: squid_n_core::ids::SecondaryMemberId(0),
        kind: SecondaryMemberKind::Joist,
        ends,
        section: Some(SectionId(0)),
        name: "B1".into(),
    });

    let mut base_beam = model.elements[0].clone();
    base_beam.id = ElemId(model.elements.len() as u32);
    base_beam.nodes = [NodeId(0), NodeId(1)].into_iter().collect();
    model.elements.push(base_beam);
    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    assert!(
        gen_slaves(&gen, StoryId(0)).contains(&free_id),
        "床面にある以上スレーブには入る"
    );
    let base_rep = &gen.rep_nodes[0];
    assert!(base_rep.restraint.is_fixed(squid_n_core::dof::Dof::Ux));
    assert!(base_rep.restraint.is_fixed(squid_n_core::dof::Dof::Uy));
    assert!(base_rep.restraint.is_fixed(squid_n_core::dof::Dof::Rz));
}

/// 支点ばねで水平に動ける基部では、代表節点を自由のままにする。
/// 基礎の質量が地盤ばねと連成して応答に効くようにするため。
#[test]
fn test_base_master_stays_free_when_supported_by_springs() {
    let mut model = two_story_model();
    // 柱脚を支点ばね支持へ変える（鉛直だけ拘束し、水平はばねで受ける）。
    let mut vertical_only = Dof6Mask::FREE;
    vertical_only.set_fixed(squid_n_core::dof::Dof::Uz);
    for n in model.nodes.iter_mut().take(2) {
        n.restraint = vertical_only;
        n.support_spring = Some([1.0e5, 1.0e5, 0.0, 0.0, 0.0, 0.0]);
    }

    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    let base_rep = &gen.rep_nodes[0];
    assert!(!base_rep.restraint.is_fixed(squid_n_core::dof::Dof::Ux));
    assert!(!base_rep.restraint.is_fixed(squid_n_core::dof::Dof::Uy));
    assert!(
        !base_rep.restraint.is_fixed(squid_n_core::dof::Dof::Rz),
        "並進が動ける階では面内回転も剛床が担う"
    );
}

#[test]
fn test_generate_single_level_is_error() {
    let mut model = Model::default();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    });
    assert!(generate_stories(&model, None).is_err());
}

/// 重量が非対称な 1 層モデル（自重なし・節点荷重のみで重みを制御）。
fn asymmetric_weight_model() -> Model {
    let mut model = Model::default();
    let coords = [
        [0.0, 0.0, 0.0],
        [4000.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [4000.0, 0.0, 3000.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if i < 2 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "DL".into(),
        nodal: vec![
            NodalLoad::manual(NodeId(2), [0.0, 0.0, -100000.0, 0.0, 0.0, 0.0]),
            NodalLoad::manual(NodeId(3), [0.0, 0.0, -300000.0, 0.0, 0.0, 0.0]),
        ],
        member: vec![],
    });
    model
}

#[test]
fn test_generate_weighted_centroid_matches_hand_calc() {
    let model = asymmetric_weight_model();
    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    // 基部の床 + 上の床の 2 階。対象は上の床（StoryId(1)）。
    assert_eq!(gen.stories.len(), 2);
    let story = &gen.stories[1];
    // 重量は自重なし・節点荷重のみ: 100kN + 300kN = 400kN
    assert_eq!(story.seismic_weight, Some(400000.0));
    assert_eq!(
        gen_slaves(&gen, StoryId(1)).len(),
        2,
        "床面上の既存節点は全てスレーブ"
    );

    // 手計算: Gx = Σ(iW·ix)/ΣiW = (100000*0 + 300000*4000) / 400000 = 3000
    assert_eq!(gen.rep_nodes.len(), 2);
    let rep = &gen.rep_nodes[1];
    assert!((rep.coord[0] - 3000.0).abs() < 1e-6, "Gx={}", rep.coord[0]);
    assert!((rep.coord[1] - 0.0).abs() < 1e-6, "Gy={}", rep.coord[1]);
    assert_eq!(rep.coord[2], 3000.0);
    // 自重を持つ要素がないモデルなので CorrectedLumped でも控除は発生せず、
    // 節点荷重の全量がそのまま質点質量になる。
    // mt = ΣiW/g = 400000/g、j = Σ(iW/g)·r² = (100000*3000² + 300000*1000²)/g
    let mass = rep
        .mass
        .expect("CorrectedLumped: 自重がないため全量が質点質量になる");
    let expected_mt = 400000.0 / GRAVITY_MM_S2;
    assert!(
        (mass[0] - expected_mt).abs() < 1e-9 * expected_mt,
        "mt={}",
        mass[0]
    );
    assert_eq!(mass[0], mass[1]);
    assert_eq!(mass[2], 0.0);
    assert_eq!(mass[3], 0.0);
    assert_eq!(mass[4], 0.0);
    let expected_j =
        (100000.0 * 3000.0_f64.powi(2) + 300000.0 * 1000.0_f64.powi(2)) / GRAVITY_MM_S2;
    assert!(
        (mass[5] - expected_j).abs() < 1e-9 * expected_j,
        "j={}",
        mass[5]
    );
    assert_eq!(rep.story, Some(StoryId(1)));
    assert!(rep.restraint.is_fixed(Dof::Uz));
    assert!(rep.restraint.is_fixed(Dof::Rx));
    assert!(rep.restraint.is_fixed(Dof::Ry));
    // このモデルは重心の手計算だけを見るため要素を持たず、全節点が非構造節点である。
    // 剛床を通じて写る剛性が無いので、代表節点は面内 3 成分とも拘束される
    // （可動性の判定規則は `master_restraint`）。上の階の代表節点が自由に残ること自体は
    // 要素を持つ `two_story_model` の `test_generate_two_stories` が確かめている。
    assert!(rep.restraint.is_fixed(Dof::Ux));
    assert!(rep.restraint.is_fixed(Dof::Uy));
    assert!(rep.restraint.is_fixed(Dof::Rz));
    // 既存節点数=4 の末尾連番で新規生成される（基部の床の分を含めて 2 つ）。
    assert_eq!(gen.generated_masters, vec![NodeId(4), NodeId(5)]);
}

#[test]
fn test_generate_zero_weight_falls_back_to_geometric_centroid() {
    let mut model = Model::default();
    // 幾何重心が非対称になるよう配置（自重・荷重ケースなし → 重量ゼロ）。
    let coords = [
        [0.0, 0.0, 0.0],
        [4000.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [6000.0, 0.0, 3000.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if i < 2 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    let gen = generate_stories(&model, None).unwrap();
    assert_eq!(gen.stories[1].seismic_weight, Some(0.0));
    let rep = &gen.rep_nodes[1];
    // 幾何重心(単純平均) = (0 + 6000) / 2 = 3000
    assert!((rep.coord[0] - 3000.0).abs() < 1e-6, "Gx={}", rep.coord[0]);
}

// ------------------------------------------------------------------
// §マスター節点の質点質量（MassMethod: CorrectedLumped / LumpedOnly）
// ------------------------------------------------------------------

/// 密度>0の柱2本（非対称断面配置ではなく非対称 DL 節点荷重）を持つ 1 層モデル。
/// マスターの質点質量算定（CorrectedLumped の控除／LumpedOnly の全量）を検証する
/// 共通土台。
fn two_columns_with_dl_model() -> Model {
    let mut model = Model::default();
    let coords = [
        [0.0, 0.0, 0.0],
        [4000.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [4000.0, 0.0, 3000.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if i < 2 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.sections.push(Section {
        id: SectionId(0),
        name: "COL".into(),
        area: 10000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "S".into(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: None,
    });
    for (i, (a, b)) in [(0u32, 2u32), (1, 3)].into_iter().enumerate() {
        model.elements.push(ElementData {
            id: ElemId(i as u32),
            kind: ElementKind::Beam,
            nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "DL".into(),
        nodal: vec![
            NodalLoad::manual(NodeId(2), [0.0, 0.0, -100000.0, 0.0, 0.0, 0.0]),
            NodalLoad::manual(NodeId(3), [0.0, 0.0, -300000.0, 0.0, 0.0, 0.0]),
        ],
        member: vec![],
    });
    model
}

#[test]
fn test_master_mass_corrected_lumped_deducts_density_self_weight() {
    let model = two_columns_with_dl_model();
    let gen =
        generate_stories_with_opts(&model, &[LoadCaseId(0)], true, MassMethod::CorrectedLumped)
            .unwrap();
    assert_eq!(gen.rep_nodes.len(), 2, "基部の床の分を含む");

    // 慣性力重心は設計地震用重量（設計単位体積重量 78.5e-6 N/mm³）で決まる。
    let sw_design: f64 = 78.5e-6 * 10000.0 * 3000.0;
    let sw_half = sw_design / 2.0;
    let w2 = sw_half + 100000.0; // NodeId(2), x=0
    let w3 = sw_half + 300000.0; // NodeId(3), x=4000
    let gx = (w2 * 0.0 + w3 * 4000.0) / (w2 + w3);

    let rep = &gen.rep_nodes[1];
    assert!((rep.coord[0] - gx).abs() < 1e-6, "Gx={}", rep.coord[0]);

    // CorrectedLumped: 柱の物理質量は解析の質量行列に部材密度質量として計上されるため
    // 控除され、net_i は DL 節点荷重分のみが残る（物理質量が各節点でちょうど相殺する）。
    let mass = rep
        .mass
        .expect("CorrectedLumped: DL節点荷重分の質点質量が設定される");
    let expected_mt = 400000.0 / GRAVITY_MM_S2;
    assert!(
        (mass[0] - expected_mt).abs() < 1e-9 * expected_mt,
        "mt={} expected={}",
        mass[0],
        expected_mt
    );
    assert_eq!(mass[0], mass[1], "並進質量 Ux=Uy");
    assert_eq!(mass[2], 0.0);
    assert_eq!(mass[3], 0.0);
    assert_eq!(mass[4], 0.0);

    let expected_j =
        (100000.0 * (0.0 - gx).powi(2) + 300000.0 * (4000.0 - gx).powi(2)) / GRAVITY_MM_S2;
    assert!(
        (mass[5] - expected_j).abs() < 1e-9 * expected_j,
        "j={} expected={}",
        mass[5],
        expected_j
    );
}

/// RC 柱 1 本（基部 z=0 固定・上端 z=3000 自由）と、柱脚節点に取り付く水平 RC 梁 1 本
/// （面積 0・せい 800）を持つ 1 層モデル。柱脚梁せい相当の追加自重だけが基部節点の質点
/// 質量に残ることの検証に使う。
fn rc_base_column_with_base_beam_model() -> Model {
    let mut model = Model::default();
    let coords = [[0.0, 0.0, 0.0], [4000.0, 0.0, 0.0], [0.0, 0.0, 3000.0]];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if i < 2 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.sections.push(Section {
        id: SectionId(0),
        name: "Col".into(),
        area: 90000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.sections.push(Section {
        id: SectionId(1),
        name: "Beam".into(),
        area: 0.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 800.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(2)].into_iter().collect(),
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    model.elements.push(ElementData {
        id: ElemId(1),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
        section: Some(SectionId(1)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    model
}

/// 補正質点方式の質量控除は通常分 `W_column` の上下 1/2 のみを控除し、柱脚梁せい相当の
/// 追加分 `W_extra`（下端節点の質点質量）は控除しないこと。基部階の `rep_nodes.mass` に
/// `W_extra / g` が残ることで確認する。
#[test]
fn test_master_mass_corrected_lumped_keeps_base_column_extra_bottom() {
    let model = rc_base_column_with_base_beam_model();
    let gen = generate_stories_with_opts(&model, &[], true, MassMethod::CorrectedLumped).unwrap();
    assert_eq!(gen.rep_nodes.len(), 2, "基部の床の分を含む");

    // 柱の単位長さ自重（RC・断面積 90000）と、柱脚梁の最大せい 800 分の追加自重。
    let per_mm = 2.4e-9 * 90000.0 * GRAVITY_MM_S2;
    let wextra = per_mm * 800.0;

    let rep = &gen.rep_nodes[0];
    let mass = rep
        .mass
        .expect("柱脚梁せい相当の追加自重が基部階の質点質量に残る");
    let expected_mt = wextra / GRAVITY_MM_S2;
    assert!(
        (mass[0] - expected_mt).abs() < 1e-9 * expected_mt,
        "mt={} expected={}",
        mass[0],
        expected_mt
    );
    assert_eq!(mass[0], mass[1], "並進質量 Ux=Uy");
}

#[test]
fn test_master_mass_lumped_only_keeps_density_self_weight() {
    let model = two_columns_with_dl_model();
    let gen =
        generate_stories_with_opts(&model, &[LoadCaseId(0)], true, MassMethod::LumpedOnly).unwrap();
    assert_eq!(gen.rep_nodes.len(), 2, "基部の床の分を含む");

    // 慣性力重心は設計地震用重量（設計単位体積重量 78.5e-6 N/mm³）で決まる。
    let sw_design: f64 = 78.5e-6 * 10000.0 * 3000.0;
    let gx = (100000.0 * 0.0 + (sw_design / 2.0 + 300000.0) * 4000.0)
        / (sw_design / 2.0 + 100000.0 + sw_design / 2.0 + 300000.0);

    // LumpedOnly の質点質量は物理質量相当（物理密度×g）。DL 節点荷重と合算する。
    let sw_per_column = 7.85e-9 * 10000.0 * 3000.0 * GRAVITY_MM_S2;
    let sw_half = sw_per_column / 2.0;
    let w2 = sw_half + 100000.0;
    let w3 = sw_half + 300000.0;

    let rep = &gen.rep_nodes[1];
    // LumpedOnly: 控除せず、物理質量相当の全量が質点質量になる。
    let mass = rep
        .mass
        .expect("LumpedOnly: 地震用重量の全量が質点質量になる");
    let expected_mt = (w2 + w3) / GRAVITY_MM_S2;
    assert!(
        (mass[0] - expected_mt).abs() < 1e-9 * expected_mt,
        "mt={} expected={}",
        mass[0],
        expected_mt
    );
    assert_eq!(mass[0], mass[1], "並進質量 Ux=Uy");

    let expected_j = (w2 * (0.0 - gx).powi(2) + w3 * (4000.0 - gx).powi(2)) / GRAVITY_MM_S2;
    assert!(
        (mass[5] - expected_j).abs() < 1e-9 * expected_j,
        "j={} expected={}",
        mass[5],
        expected_j
    );
}

// ------------------------------------------------------------------
// §設計重量（78.5）と物理質量（7.85）の分離
// ------------------------------------------------------------------

/// 鋼線材 1 本: 地震用重量（設計）は 78.5 kN/m³ ベース、動的質量は物理密度 7.85 t/m³ ベース。
#[test]
fn test_steel_line_design_weight_and_physical_mass_are_separated() {
    let (len, area) = (4000.0, 90000.0);
    let model = single_beam_model(len, 7.85e-9, area, None, RigidZone::default(), None);

    let corrected =
        generate_stories_with_opts(&model, &[], true, MassMethod::CorrectedLumped).unwrap();
    // 地震用重量（設計）: 78.5e-6 N/mm³ × A × L の上端半分。
    let design = 78.5e-6 * area * len / 2.0;
    assert!(
        (corrected.stories[1].seismic_weight.unwrap() - design).abs() < 1e-6,
        "設計重量が 78.5 ベースでない: {}",
        corrected.stories[1].seismic_weight.unwrap()
    );
    // CorrectedLumped は躯体分を質量行列が担うため、付加重量がなければ質点は 0。
    assert!(corrected.rep_nodes[1].mass.is_none());

    // LumpedOnly は物理質量（7.85 t/m³ × A × L × g）の上端半分を質点に持つ。
    let lumped = generate_stories_with_opts(&model, &[], true, MassMethod::LumpedOnly).unwrap();
    let physical = 7.85e-9 * area * len * GRAVITY_MM_S2 / 2.0;
    let m = lumped.rep_nodes[1].mass.expect("LumpedOnly は質点を持つ");
    assert!(
        (m[0] - physical / GRAVITY_MM_S2).abs() < 1e-9 * (physical / GRAVITY_MM_S2),
        "動的質量が 7.85 ベースでない: {}",
        m[0]
    );
    // 設計重量と物理質量は一致しない（地震用重量/g ≠ 動的質量）。
    assert!((corrected.stories[1].seismic_weight.unwrap() / GRAVITY_MM_S2 - m[0]).abs() > 1e-6);
}

/// `model` の主架構線材（ダンパー要素を除く）が解析の質量行列へ与える総質量相当の
/// 重量 [N]。実装（`analysis_mass_per_length`）と同じ [`Model::element_mass_properties`]
/// から求める。
fn main_frame_matrix_mass_equiv(model: &Model) -> f64 {
    let load_cfg = model.load_cfg.clone().unwrap_or_default();
    model
        .elements
        .iter()
        .filter(|e| {
            matches!(e.kind, ElementKind::Beam | ElementKind::Brace { .. })
                && e.nodes.len() >= 2
                && !load_cfg.dampers.iter().any(|d| d.elem == e.id)
                && model.element_section(e).is_some()
                && model.element_material(e).is_some()
        })
        .map(|e| {
            model.element_mass_properties(e).map_or(0.0, |p| {
                p.total_mass(model.member_length(e)) * GRAVITY_MM_S2
            })
        })
        .sum()
}

/// 両 MassMethod の公称並進総動的質量が一致することを確認する。
/// `CorrectedLumped` は「質点質量合計＋主架構の質量行列総和」、`LumpedOnly` は
/// 「質点質量合計」で総和が等しくなる。質量行列総和は実装と同じ
/// [`Model::element_mass_properties`] から求める。合計が一致することは、各節点で
/// クランプ `max(0, mass_equiv − matrix)` が発生していないことも意味する
/// （負の差が 1 つでもあれば合計は一致しない）。
fn assert_mass_methods_consistent(model: &Model) {
    let corrected =
        generate_stories_with_opts(model, &[], true, MassMethod::CorrectedLumped).unwrap();
    let lumped = generate_stories_with_opts(model, &[], true, MassMethod::LumpedOnly).unwrap();
    let sum_mass = |gen: &StoryGenResult| {
        gen.rep_nodes
            .iter()
            .filter_map(|n| n.mass)
            .map(|m| m[0])
            .sum::<f64>()
    };
    let matrix = main_frame_matrix_mass_equiv(model) / GRAVITY_MM_S2;
    let corrected_total = sum_mass(&corrected) + matrix;
    let lumped_total = sum_mass(&lumped);
    assert!(
        (corrected_total - lumped_total).abs() < 1e-9 * lumped_total.max(1.0),
        "両方式の総動的質量が一致しない: corrected={corrected_total} lumped={lumped_total}"
    );
}

/// 両 MassMethod の公称並進総動的質量が一致する（鋼材線材のモデル）。
#[test]
fn test_both_mass_methods_have_equal_total_dynamic_mass() {
    let (len, area) = (4000.0, 90000.0);
    let model = single_beam_model(len, 7.85e-9, area, None, RigidZone::default(), None);
    assert_mass_methods_consistent(&model);
}

/// 水平 RC 梁 1 本（スラブ厚控除が効く）と自重ゼロの RC 柱 2 本を持つ 1 層モデル。
fn rc_beam_with_slab_model() -> Model {
    let len = 6000.0;
    let mut model = Model::default();
    for (i, c) in [
        [0.0, 0.0, 0.0],
        [len, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [len, 0.0, 3000.0],
    ]
    .iter()
    .enumerate()
    {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if c[2] == 0.0 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    // 梁断面（b=400, D=700, A=280000）。スラブ厚 150 の控除で設計重量の断面積は 220000。
    model.sections.push(Section {
        id: SectionId(0),
        name: "RC梁".into(),
        area: 400.0 * 700.0,
        iy: 1.0e9,
        iz: 1.0e9,
        j: 1.0e9,
        depth: 700.0,
        width: 400.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    // 自重ゼロ（A=0）の柱断面。せい 800 で水平梁のフェイス控除 400×2 を作る。
    model.sections.push(Section {
        id: SectionId(1),
        name: "RC柱(重量なし)".into(),
        area: 0.0,
        iy: 1.0e9,
        iz: 1.0e9,
        j: 1.0e9,
        depth: 800.0,
        width: 800.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    for (id, a, b, sec, ref_vector) in [
        (0u32, 0u32, 2u32, 1u32, [1.0, 0.0, 0.0]),
        (1, 1, 3, 1, [1.0, 0.0, 0.0]),
        (2, 2, 3, 0, [0.0, 0.0, 1.0]),
    ] {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
            section: Some(SectionId(sec)),
            local_axis: LocalAxis { ref_vector },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
    }
    model.slab_thickness = 150.0;
    model.floor_regions.push(FloorRegion::new(
        FloorRegionId(0),
        vec![NodeId(2), NodeId(3)],
    ));
    model
}

/// RC 梁のスラブ厚控除があっても、物理質量相当は質量行列と同じ総断面・節点間長で
/// 算定されるため両方式が一致する。
#[test]
fn test_both_mass_methods_equal_for_rc_beam_with_slab_deduction() {
    assert_mass_methods_consistent(&rc_beam_with_slab_model());
}

/// 二次部材（質量行列に算入されない）とダンパー（断面自重を置換する）を含むモデルでも
/// 両方式の総動的質量が一致する。
#[test]
fn test_both_mass_methods_equal_with_secondary_member_and_damper() {
    assert_mass_methods_consistent(&secondary_joist_model());

    let damper = DamperSpec {
        elem: ElemId(0),
        device_weight: 20000.0,
        device_length: 1000.0,
        support_area: 5000.0,
    };
    let cfg = LoadCfg {
        dampers: vec![damper],
        ..Default::default()
    };
    let model = single_beam_model(
        4000.0,
        7.85e-9,
        90000.0,
        None,
        RigidZone::default(),
        Some(cfg),
    );
    assert_mass_methods_consistent(&model);
}

/// 仕上げ・付加線重量は質量行列に対応物がないため、CorrectedLumped でも質点に残る。
#[test]
fn test_line_finish_and_extra_weight_survive_corrected_lumped() {
    let (len, area) = (4000.0, 90000.0);
    let cfg = LoadCfg {
        extra_line_weight: vec![(ElemId(0), 5.0)],
        finish_area_weight: vec![(ElemId(0), 0.002)],
        ..Default::default()
    };
    let model = single_beam_model(len, 7.85e-9, area, None, RigidZone::default(), Some(cfg));
    let gen = generate_stories_with_opts(&model, &[], true, MassMethod::CorrectedLumped).unwrap();
    let top = gen.rep_nodes[1].mass.expect("付加重量が質点に残る");
    let phi = 2.0 * (300.0 + 300.0);
    let extra = (5.0 * len + 0.002 * phi * len) / 2.0;
    assert!(
        (top[0] - extra / GRAVITY_MM_S2).abs() < 1e-9 * (extra / GRAVITY_MM_S2),
        "付加重量が質点から消えた: {}",
        top[0]
    );
}

/// 鉄骨重量割増の増分（factor−1 相当）も質量行列に対応物がないため質点に残る。
/// 増分は設計重量 78.5 ベースではなく、質量行列（物理密度 7.85）に同じ factor を
/// 掛けた増分として残る。
#[test]
fn test_steel_weight_factor_increment_is_physical_based_in_corrected_lumped() {
    let (len, area) = (4000.0, 90000.0);
    let cfg = LoadCfg {
        steel_weight_factor: 1.3,
        ..Default::default()
    };
    let model = single_beam_model(len, 7.85e-9, area, None, RigidZone::default(), Some(cfg));
    let gen = generate_stories_with_opts(&model, &[], true, MassMethod::CorrectedLumped).unwrap();
    let top = gen.rep_nodes[1].mass.expect("割増増分が質点に残る");
    // 質点 = 質量行列（物理密度×g）× (factor − 1) の上端半分。
    let body_physical = 7.85e-9 * area * len * GRAVITY_MM_S2;
    let increment = body_physical * (1.3 - 1.0) / 2.0;
    assert!(
        (top[0] - increment / GRAVITY_MM_S2).abs() < 1e-9 * (increment / GRAVITY_MM_S2),
        "割増増分が物理質量ベースでない: {}",
        top[0]
    );
}

/// 鉄骨重量割増を掛けても両 MassMethod の総動的質量が一致し、`mass_equiv` が
/// 質量行列×factor（物理質量 7.85 ベース）になる。
#[test]
fn test_both_mass_methods_equal_with_steel_weight_factor() {
    let (len, area) = (4000.0, 90000.0);
    let cfg = LoadCfg {
        steel_weight_factor: 1.3,
        ..Default::default()
    };
    let model = single_beam_model(len, 7.85e-9, area, None, RigidZone::default(), Some(cfg));
    assert_mass_methods_consistent(&model);

    let lumped = generate_stories_with_opts(&model, &[], true, MassMethod::LumpedOnly).unwrap();
    let m = lumped.rep_nodes[1].mass.expect("質点質量");
    let physical_half = 7.85e-9 * area * len * GRAVITY_MM_S2 * 1.3 / 2.0;
    assert!(
        (m[0] - physical_half / GRAVITY_MM_S2).abs() < 1e-9 * (physical_half / GRAVITY_MM_S2),
        "動的質量に factor が物理ベースで掛かっていない: {}",
        m[0]
    );
}

/// ダンパー: 装置重量は設計・物理で同値、支持部だけが 78.5/7.85 で分離する。
#[test]
fn test_damper_device_weight_is_common_and_support_is_separated() {
    let len = 4000.0;
    let damper = DamperSpec {
        elem: ElemId(0),
        device_weight: 20000.0,
        device_length: 1000.0,
        support_area: 5000.0,
    };
    let cfg = LoadCfg {
        dampers: vec![damper],
        ..Default::default()
    };
    let model = single_beam_model(len, 7.85e-9, 90000.0, None, RigidZone::default(), Some(cfg));
    let support_len = (len - 1000.0).max(0.0);
    let gen = generate_stories(&model, None).unwrap();
    // 設計重量（地震用重量）: 装置＋支持部×78.5e-6。
    let design = 20000.0 + 5000.0 * support_len * 78.5e-6;
    assert!((gen.stories[1].seismic_weight.unwrap() - design / 2.0).abs() < 1e-6);
    // 物理質量（LumpedOnly）: 装置＋支持部×物理密度×g。装置は同値、支持部のみ小さい。
    let lumped = generate_stories_with_opts(&model, &[], true, MassMethod::LumpedOnly).unwrap();
    let mass_equiv = 20000.0 + 5000.0 * support_len * 7.85e-9 * GRAVITY_MM_S2;
    let m = lumped.rep_nodes[1].mass.expect("ダンパー質量が質点に残る");
    assert!(
        (m[0] - mass_equiv / 2.0 / GRAVITY_MM_S2).abs() < 1e-9 * (mass_equiv / 2.0 / GRAVITY_MM_S2),
        "ダンパー質量: {}",
        m[0]
    );
}

/// 自重同期ケース（[`generate_stories_with_synced_self_weight`]）と密度直接算入
/// （[`generate_stories_with_opts`] の `true`。密度経路）で動的質量が一致する。
///
/// 質点のみ方式で比較する。補正質点方式は鋼材のみのモデルでは両経路とも質点質量が
/// 0（`None`）になり、両経路の差を検出できないため。
#[test]
fn test_self_weight_via_case_matches_density_for_mass() {
    let mut model = two_story_model();
    model.load_cases.clear();
    let (nodal, member) = crate::self_weight::self_weight_case_content(&model, &LoadCfg::default());
    model.load_cases.push(LoadCase {
        kind: LoadCaseKind::Dead,
        id: LoadCaseId(0),
        name: "DL".into(),
        nodal,
        member,
    });
    let by_density = generate_stories_with_opts(&model, &[], true, MassMethod::LumpedOnly).unwrap();
    let by_case =
        generate_stories_with_synced_self_weight(&model, &[LoadCaseId(0)], MassMethod::LumpedOnly)
            .unwrap();
    let mut any_positive = false;
    for (a, b) in by_density.rep_nodes.iter().zip(by_case.rep_nodes.iter()) {
        let ma = a.mass.map(|m| m[0]).unwrap_or(0.0);
        let mb = b.mass.map(|m| m[0]).unwrap_or(0.0);
        any_positive |= ma > 0.0;
        assert!(
            (ma - mb).abs() < 1e-9 * ma.max(1.0),
            "true/false 経路で動的質量が一致しない: density={ma} case={mb}"
        );
    }
    assert!(
        any_positive,
        "質点質量が 0 のままでは密度直接算入と自重ケースの差を検出できない"
    );
}

/// 壁エレメント（仕上げ・開口あり）でも、密度直接算入（密度経路）と自重同期ケース
/// （[`generate_stories_with_synced_self_weight`]）の 2 経路で動的質量（質点のみ方式）が
/// 一致する。壁の質量行列分は [`Model::element_mass_properties`] からは求まらないため、
/// ここでは経路間の一致を見る。
#[test]
fn test_wall_mass_consistent_between_density_and_case_paths() {
    use squid_n_core::model::AreaLoad;

    let mut model = wall_model();
    model.wall_plates[0].loads = vec![AreaLoad {
        kind: "増打ち".into(),
        value: 5.0e-4,
    }];
    model.wall_plates[0].opening_area = 500_000.0;
    model.wall_plates[0].opening_weight = 3000.0;

    let by_density = generate_stories_with_opts(&model, &[], true, MassMethod::LumpedOnly).unwrap();

    let (nodal, member) = crate::self_weight::self_weight_case_content(&model, &LoadCfg::default());
    model.load_cases.clear();
    model.load_cases.push(LoadCase {
        kind: LoadCaseKind::Dead,
        id: LoadCaseId(0),
        name: "DL".into(),
        nodal,
        member,
    });
    let by_case =
        generate_stories_with_synced_self_weight(&model, &[LoadCaseId(0)], MassMethod::LumpedOnly)
            .unwrap();

    let masses = |gen: &StoryGenResult| {
        gen.rep_nodes
            .iter()
            .map(|n| n.mass.map(|m| m[0]).unwrap_or(0.0))
            .collect::<Vec<_>>()
    };
    let a = masses(&by_density);
    let b = masses(&by_case);
    assert_eq!(a.len(), b.len());
    assert!(
        a.iter().any(|m| *m > 0.0),
        "壁の動的質量が質点に残っていない"
    );
    for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
        assert!(
            (x - y).abs() < 1e-9 * x.max(1.0),
            "node {i}: density={x} case={y}"
        );
    }
}

/// 二次部材（小梁）1 本のみを持つ 1 層モデル（主架構要素なし）。
/// 二次部材の自重は解析の質量行列（部材密度質量）に算入されないことの確認用。
fn secondary_joist_model() -> Model {
    let mut model = Model::default();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [0.0, 0.0, 3000.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.nodes.push(Node {
        id: NodeId(2),
        coord: [2000.0, 0.0, 3000.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.sections.push(Section {
        id: SectionId(0),
        name: "JOIST".into(),
        area: 5000.0,
        iy: 1.0e7,
        iz: 1.0e7,
        j: 1.0e7,
        depth: 200.0,
        width: 100.0,
        as_y: 4000.0,
        as_z: 4000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "S".into(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: None,
    });
    let ends = squid_n_core::model::SecondaryMemberEnds::Detached([
        model.nodes[1].coord,
        model.nodes[2].coord,
    ]);
    model.unassigned_joists.push(SecondaryMember {
        gravity_end_shares: None,
        id: squid_n_core::ids::SecondaryMemberId(1),
        kind: SecondaryMemberKind::Joist,
        ends,
        section: Some(SectionId(0)),
        name: "G1".into(),
    });
    for top in [1, 2] {
        let mut column = two_story_model().elements[0].clone();
        column.id = ElemId(model.elements.len() as u32);
        column.nodes = [NodeId(0), NodeId(top)].into_iter().collect();
        model.elements.push(column);
    }
    model
}

#[test]
fn test_master_mass_corrected_lumped_does_not_deduct_secondary_member_self_weight() {
    let model = secondary_joist_model();
    let gen = generate_stories_with_opts(&model, &[], true, MassMethod::CorrectedLumped).unwrap();
    assert_eq!(gen.rep_nodes.len(), 2, "基部の床の分を含む");

    // 二次部材（小梁）の自重は主架構要素ではなく解析の質量行列（部材密度質量）に
    // 算入されないため、CorrectedLumped でも控除されずそのまま残る。
    let sw = 7.85e-9 * 5000.0 * 2000.0 * GRAVITY_MM_S2;
    let rep = &gen.rep_nodes[1];
    let mass = rep
        .mass
        .expect("二次部材の自重は控除されずそのまま質点質量になる");
    let expected_mt = sw / GRAVITY_MM_S2;
    assert!(
        (mass[0] - expected_mt).abs() < 1e-9 * expected_mt,
        "mt={} expected={}",
        mass[0],
        expected_mt
    );
}

/// 二次部材（鋼小梁）の設計重量・物理質量の両方に、主架構線材と同じ鉄骨重量割増が
/// 掛かる。設計は 78.5 kN/m³ ベース、物理質量は物理密度 7.85 t/m³ ベースで、同じ
/// `factor` を共有する。密度直接算入（story_gen）の経路で確認する。
///
/// 小梁の自重だけを見るため、支持柱は断面積 0（自重ゼロ）とする。
#[test]
fn test_secondary_joist_steel_weight_factor_applies_to_design_and_mass() {
    let mut model = Model::default();
    for (i, c) in [[0.0, 0.0, 0.0], [0.0, 0.0, 3000.0], [2000.0, 0.0, 3000.0]]
        .iter()
        .enumerate()
    {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if i == 0 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    let mk_section = |id: SectionId, name: &str, area: f64| Section {
        id,
        name: name.into(),
        area,
        iy: 1.0e7,
        iz: 1.0e7,
        j: 1.0e7,
        depth: 200.0,
        width: 100.0,
        as_y: 4000.0,
        as_z: 4000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    model
        .sections
        .push(mk_section(SectionId(0), "JOIST", 5000.0));
    model.sections.push(mk_section(SectionId(1), "ZERO", 0.0));
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "S".into(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: None,
    });
    for (id, a, b) in [(0u32, 0u32, 1u32), (1, 0, 2)] {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
            section: Some(SectionId(1)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
    }
    model.unassigned_joists.push(SecondaryMember {
        gravity_end_shares: None,
        id: squid_n_core::ids::SecondaryMemberId(1),
        kind: SecondaryMemberKind::Joist,
        ends: squid_n_core::model::SecondaryMemberEnds::Detached([
            model.nodes[1].coord,
            model.nodes[2].coord,
        ]),
        section: Some(SectionId(0)),
        name: "J".into(),
    });
    model.load_cfg = Some(LoadCfg {
        steel_weight_factor: 1.3,
        ..Default::default()
    });

    let (area, span, factor) = (5000.0, 2000.0, 1.3);
    let sm = &model.unassigned_joists[0];
    let design_udl = crate::floor::joist_self_weight_udl(&model, sm).expect("設計自重");
    let mass_udl = crate::floor::joist_mass_equiv_udl(&model, sm).expect("物理質量相当");
    assert!((design_udl - 78.5e-6 * area * factor).abs() < 1e-9 * design_udl);
    assert!((mass_udl - 7.85e-9 * area * GRAVITY_MM_S2 * factor).abs() < 1e-9 * mass_udl);

    let gen = generate_stories_with_opts(&model, &[], true, MassMethod::LumpedOnly).unwrap();
    let design = 78.5e-6 * area * factor * span;
    let sw = gen.stories[1].seismic_weight.expect("地震用重量");
    assert!(
        (sw - design).abs() < 1e-6 * design,
        "設計重量に factor が掛かっていない: {sw} expected={design}"
    );
    let physical_mass = 7.85e-9 * area * factor * span;
    let m = gen.rep_nodes[1].mass.expect("質点質量");
    assert!(
        (m[0] - physical_mass).abs() < 1e-9 * physical_mass,
        "物理質量に factor が掛かっていない: {} expected={physical_mass}",
        m[0]
    );
}

/// 2 本の並行大梁（いずれも材軸中間に節点を持たない 1 部材）を持ち、
/// その材軸位置 0.5 に小梁がアンカーするモデル。小梁の両端に一致する節点は無い。
fn secondary_joist_on_girder_midspan_model(with_joist: bool) -> Model {
    let mut model = Model::default();
    for (i, (x, y, z)) in [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 3000.0),
        (4000.0, 0.0, 3000.0),
        (0.0, 4000.0, 3000.0),
        (4000.0, 4000.0, 3000.0),
    ]
    .into_iter()
    .enumerate()
    {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: [x, y, z],
            restraint: if i == 0 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.sections.push(Section {
        id: SectionId(0),
        name: "G".into(),
        area: 8000.0,
        iy: 1.0e7,
        iz: 1.0e7,
        j: 1.0e7,
        depth: 300.0,
        width: 200.0,
        as_y: 4000.0,
        as_z: 4000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.sections.push(Section {
        id: SectionId(1),
        name: "J".into(),
        area: 5000.0,
        iy: 1.0e7,
        iz: 1.0e7,
        j: 1.0e7,
        depth: 200.0,
        width: 100.0,
        as_y: 4000.0,
        as_z: 4000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "S".into(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: None,
    });
    for (id, (a, b), sec) in [
        (0u32, (1u32, 2u32), 0u32),
        (1, (3, 4), 0),
        (2, (1, 3), 0),
        (3, (2, 4), 0),
    ] {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
            section: Some(SectionId(sec)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }
    if with_joist {
        model.unassigned_joists.push(SecondaryMember {
            gravity_end_shares: None,
            id: squid_n_core::ids::SecondaryMemberId(0),
            kind: SecondaryMemberKind::Joist,
            ends: squid_n_core::model::SecondaryMemberEnds::Supported([
                squid_n_core::model::SecondaryMemberAnchor {
                    support: squid_n_core::model::SupportMemberId::Primary(ElemId(0)),
                    position: 0.5,
                },
                squid_n_core::model::SecondaryMemberAnchor {
                    support: squid_n_core::model::SupportMemberId::Primary(ElemId(1)),
                    position: 0.5,
                },
            ]),
            section: Some(SectionId(1)),
            name: "J0".into(),
        });
    }
    model
}

/// 大梁の材軸中間へアンカーした小梁（両端に節点が無い）の自重が、DL ケースが
/// 無く密度から直接算入する経路でも階の地震用重量へ含まれること（欠落させない）。
#[test]
fn test_secondary_member_on_midspan_is_seismic_weight_in_density_path() {
    let with = secondary_joist_on_girder_midspan_model(true);
    let without = secondary_joist_on_girder_midspan_model(false);

    let sw = 78.5e-6 * 5000.0 * 4000.0;
    let gen_with = generate_stories_with_opts(&with, &[], true, MassMethod::default()).unwrap();
    let gen_without =
        generate_stories_with_opts(&without, &[], true, MassMethod::default()).unwrap();

    let upper = |gen: &StoryGenResult| gen.stories.last().unwrap().seismic_weight.unwrap();
    let delta = upper(&gen_with) - upper(&gen_without);
    assert!(
        (delta - sw).abs() < 1e-9 * sw.max(1.0),
        "小梁自重が階の地震用重量へ含まれない: delta={delta} expected={sw}"
    );
}

/// 基部(z=0, 固定)と上端(z=`len`, 自由)を結ぶ 1 部材の最小モデル。
/// 面控除・鉄骨割増率・付加線重量など「単一部材の自重」を検証する各テストの共通土台。
fn single_beam_model(
    len: f64,
    density: f64,
    area: f64,
    fc: Option<f64>,
    rigid_zone: RigidZone,
    load_cfg: Option<LoadCfg>,
) -> Model {
    let mut model = Model::default();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [0.0, 0.0, len],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.sections.push(Section {
        id: SectionId(0),
        name: "S".into(),
        area,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "M".into(),
        category: if fc.is_some() {
            MaterialCategory::Concrete
        } else {
            MaterialCategory::Steel
        },
        young: 205000.0,
        poisson: 0.3,
        density,
        shear: None,
        fc,
        fy: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone,
        plastic_zone: None,
        spring: None,
    });
    model.load_cfg = load_cfg;
    model
}

#[test]
fn test_static_reactions_point_load_hand_calc() {
    // 単純梁 L=4000, a=1000, p=800: Ri=p(L-a)/L=600, Rj=p*a/L=200
    let (ri, rj) = static_reactions(
        &MemberLoadKind::Point {
            a: 1000.0,
            p: 800.0,
        },
        4000.0,
    );
    assert!((ri - 600.0).abs() < 1e-9, "ri={}", ri);
    assert!((rj - 200.0).abs() < 1e-9, "rj={}", rj);
    assert!((ri + rj - 800.0).abs() < 1e-9);
}

#[test]
fn test_static_reactions_symmetric_distributed_is_half_half() {
    let (ri, rj) = static_reactions(
        &MemberLoadKind::Distributed {
            a: 0.0,
            b: 6000.0,
            w1: 10.0,
            w2: 10.0,
        },
        6000.0,
    );
    assert!((ri - 30000.0).abs() < 1e-9, "ri={}", ri);
    assert!((rj - 30000.0).abs() < 1e-9, "rj={}", rj);
}

#[test]
fn test_static_reactions_asymmetric_distributed_hand_calc() {
    // 三角形分布(w1=0→w2=20)、a=0,b=4000,L=4000。
    // W=(0+20)/2*4000=40000, xbar=4000*(0+40)/(3*20)=2666.666...,
    // Rj=W*xbar/L=26666.666..., Ri=W-Rj=13333.333...
    let (ri, rj) = static_reactions(
        &MemberLoadKind::Distributed {
            a: 0.0,
            b: 4000.0,
            w1: 0.0,
            w2: 20.0,
        },
        4000.0,
    );
    assert!((ri - 13333.333333333334).abs() < 1e-6, "ri={}", ri);
    assert!((rj - 26666.666666666668).abs() < 1e-6, "rj={}", rj);
    assert!((ri + rj - 40000.0).abs() < 1e-6);
}

#[test]
fn test_member_load_reaction_distribution_end_to_end() {
    // 自重を持たない(section/material 未設定)部材に非対称な三角形分布荷重を与え、
    // 剛床代表節点の重心が naive な 1/2-1/2 配分(x=2000)ではなく
    // 静定反力配分による偏った位置(x≈2666.67)になることを確認する（§1.4）。
    let mut model = Model::default();
    let coords = [
        [0.0, 0.0, 0.0],
        [4000.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [4000.0, 0.0, 3000.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if i < 2 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(2), NodeId(3)].into_iter().collect(),
        section: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "DL".into(),
        nodal: vec![],
        member: vec![MemberLoad::manual(
            ElemId(0),
            [0.0, 0.0, -1.0],
            MemberLoadKind::Distributed {
                a: 0.0,
                b: 4000.0,
                w1: 0.0,
                w2: 20.0,
            },
        )],
    });
    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    let rep = &gen.rep_nodes[1];
    assert!(
        (rep.coord[0] - 2666.666666666667).abs() < 1e-2,
        "Gx={}",
        rep.coord[0]
    );
}

#[test]
fn test_face_reduction_applies_only_to_concrete() {
    // §1.8: RC/SRC の柱（鉛直材）は床上面から床上面（＝節点間距離。フェイス控除
    // しない）、S 柱も節点間距離。single_beam_model は鉛直材（柱）なので、
    // fc の有無によらず全長で算定される。
    let len = 4000.0;
    let area = 90000.0;
    let density = 2.4e-9;
    let rz = RigidZone {
        face_i: Some(300.0),
        face_j: Some(300.0),
        ..Default::default()
    };

    let rc_model = single_beam_model(len, density, area, Some(24.0), rz, None);
    let rc = generate_stories(&rc_model, None).unwrap();
    let expected_rc = density * area * len * GRAVITY_MM_S2 / 2.0;
    assert!(
        (rc.stories[0].seismic_weight.unwrap() - expected_rc).abs() < 1e-6,
        "{}",
        rc.stories[0].seismic_weight.unwrap()
    );

    // 鋼材は標準の物理質量密度。設計用単位体積重量はこれに比例し、標準密度で 78.5 kN/m³。
    let steel_density = 7.85e-9;
    let s_model = single_beam_model(len, steel_density, area, None, rz, None);
    let s = generate_stories(&s_model, None).unwrap();
    let expected_s = 78.5e-6 * area * len / 2.0;
    assert!(
        (s.stories[0].seismic_weight.unwrap() - expected_s).abs() < 1e-6,
        "{}",
        s.stories[0].seismic_weight.unwrap()
    );
}

#[test]
fn test_face_reduction_applies_to_horizontal_concrete_beam() {
    // §1.8: RC/SRC の水平材（梁）は柱面間距離（len − face_i − face_j）で算定する。
    // 鉛直材（柱）は同じフェイス値でも控除しない（前テストで検証）。
    let len = 6000.0;
    let area = 400.0 * 700.0;
    let density = 2.4e-9;
    let mut model = Model::default();
    for (i, c) in [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [len, 0.0, 3000.0],
        [len, 0.0, 0.0],
    ]
    .iter()
    .enumerate()
    {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if c[2] == 0.0 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.sections.push(Section {
        id: SectionId(0),
        name: "RC".into(),
        area,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 700.0,
        width: 400.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        category: MaterialCategory::Concrete,
        young: 22000.0,
        poisson: 0.2,
        density,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    // 柱フェース距離は幾何（取り付く直交材のせい/2）から決まるため、せい 800 の
    // 柱を両端に立ててフェイス控除 400 を作る。柱の断面積は 0 にして自重を
    // 生じさせず、水平梁の自重だけを検証対象にする。
    model.sections.push(Section {
        id: SectionId(1),
        name: "柱(重量なし)".into(),
        area: 0.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 800.0,
        width: 800.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    // 水平梁（節点1→2）のみ断面・材料を持たせ、フェイス控除を検証する。
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(1), NodeId(2)].into_iter().collect(),
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    for (id, a, b) in [(1u32, 0u32, 1u32), (2, 3, 2)] {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
            section: Some(SectionId(1)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
    }

    let gen = generate_stories(&model, None).unwrap();
    let eff_len = len - 400.0 - 400.0;
    let expected = density * area * eff_len * GRAVITY_MM_S2;
    assert!(
        (gen.stories[1].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "w={} expected={}",
        gen.stories[1].seismic_weight.unwrap(),
        expected
    );
}

#[test]
fn test_steel_weight_factor_applies_only_to_steel() {
    let len = 4000.0;
    let area = 90000.0;
    let density = 7.85e-9;
    let cfg = LoadCfg {
        live_load_reduction: false,
        dampers: Vec::new(),
        finish_area_weight: Vec::new(),
        k_brace_rule: Default::default(),
        steel_weight_factor: 1.3,
        extra_line_weight: vec![],
    };

    let steel_model = single_beam_model(
        len,
        density,
        area,
        None,
        RigidZone::default(),
        Some(cfg.clone()),
    );
    let steel = generate_stories(&steel_model, None).unwrap();
    let expected_steel = 78.5e-6 * area * len * 1.3 / 2.0;
    assert!(
        (steel.stories[0].seismic_weight.unwrap() - expected_steel).abs() < 1e-6,
        "{}",
        steel.stories[0].seismic_weight.unwrap()
    );

    let rc_model = single_beam_model(
        len,
        density,
        area,
        Some(24.0),
        RigidZone::default(),
        Some(cfg),
    );
    let rc = generate_stories(&rc_model, None).unwrap();
    let expected_rc = density * area * len * GRAVITY_MM_S2 / 2.0;
    assert!(
        (rc.stories[0].seismic_weight.unwrap() - expected_rc).abs() < 1e-6,
        "割増率はコンクリート材に適用しない: {}",
        rc.stories[0].seismic_weight.unwrap()
    );
}

#[test]
fn test_extra_line_weight_adds_to_self_weight() {
    let len = 4000.0;
    let area = 90000.0;
    let density = 7.85e-9;
    let cfg = LoadCfg {
        live_load_reduction: false,
        dampers: Vec::new(),
        finish_area_weight: Vec::new(),
        k_brace_rule: Default::default(),
        steel_weight_factor: 1.0,
        extra_line_weight: vec![(ElemId(0), 5.0)],
    };
    let model = single_beam_model(len, density, area, None, RigidZone::default(), Some(cfg));
    let gen = generate_stories(&model, None).unwrap();
    let expected = (78.5e-6 * area * len + 5.0 * len) / 2.0;
    assert!(
        (gen.stories[1].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[1].seismic_weight.unwrap()
    );
}

/// 矩形壁(4000×3000, t=150)を上下 2 レベルの節点間に張った 1 層モデル。
fn wall_model() -> Model {
    let mut model = Model::default();
    let coords = [
        [0.0, 0.0, 0.0],
        [4000.0, 0.0, 0.0],
        [4000.0, 0.0, 3000.0],
        [0.0, 0.0, 3000.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if i < 2 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.sections.push(Section {
        id: SectionId(0),
        name: "Wall".into(),
        area: 0.0,
        iy: 1.0,
        iz: 1.0,
        j: 1.0,
        depth: 0.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: Some(150.0),
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    // 壁の解析要素は入力の正ではなく生成物（D5）のため、壁版（`WallPlate`）と
    // それが属する壁領域（`WallRegion`）を直接構築する。`enumerate_self_weight`
    // が内部で壁展開モデルを組み立て、そこから `ElementKind::Wall` を生成する。
    //
    // 囲まれた壁版の境界は割当領域（支持部材）が持つため、境界辺の支持部材を
    // 断面なしの梁として先に置く（テストが後から足す断面付きの部材と衝突しない
    // よう ID は 100 番台にする）。
    for (id, (a, b)) in [
        (100u32, (0u32, 1u32)),
        (101, (1, 2)),
        (102, (2, 3)),
        (103, (3, 0)),
    ] {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
            section: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }
    model.add_enclosed_wall_plate_from_nodes(
        &[NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        WallPlate {
            self_weight_shares: Vec::new(),
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed,
            section: Some(SectionId(0)),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![],
            slit: Default::default(),
        },
    );
    model.wall_regions.push(WallRegion {
        id: WallRegionId(0),
        name: String::new(),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        wall_plate_ids: vec![WallPlateId(0)],
        posts: Vec::new(),
    });
    model
}

#[test]
fn test_wall_self_weight_included_in_story_weight() {
    // §1.2: 壁自重 w=ρ·t·A·g を全頂点に等分配。
    // 基部(z=0)側 2 節点は基部の床に属し、層の重量には入らない。上の床
    // （層の上端）の地震用重量に算入されるのは上端 2 節点分(w/2)のみになる。
    let model = wall_model();
    let gen = generate_stories(&model, None).unwrap();
    assert_eq!(gen.stories.len(), 2, "基部の床 + 上の床");
    let area = 4000.0 * 3000.0;
    let w_total = 2.4e-9 * 150.0 * area * GRAVITY_MM_S2;
    let expected = w_total / 2.0;
    assert!(
        (gen.stories[1].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[1].seismic_weight.unwrap()
    );
}

/// 壁エレメントになる壁版の仕上げ・増打ちの面荷重も、階の地震用重量へ算入する。
///
/// 壁エレメントの自重は要素経由で算定するため、合成する `WallAttr` へ面荷重を
/// 写し忘れると**要素になる壁版だけ**この重さが黙って落ちる。要素にならない
/// 壁版は `Model::wall_plate_self_weight` を直接使うので落ちず、差が出るのが
/// 一部の壁だけという気づきにくい形になる。
#[test]
fn test_wall_finish_load_included_in_story_weight() {
    use squid_n_core::model::AreaLoad;

    let base = generate_stories(&wall_model(), None).unwrap();
    let mut model = wall_model();
    model.wall_plates[0].loads = vec![AreaLoad {
        kind: "増打ち".into(),
        value: 5.0e-4,
    }];
    let gen = generate_stories(&model, None).unwrap();

    // 壁の自重は四隅へ等分され、上端 2 節点分（1/2）だけが層の重量へ入る。
    // 面積は周辺柱梁の内法寸法で評価するため、増分そのものを式で書くかわりに
    // 「躯体自重に対する面荷重の比」で期待値を作る（内法係数が約分される）。
    let w0 = base.stories[1].seismic_weight.unwrap();
    let w1 = gen.stories[1].seismic_weight.unwrap();
    let ratio = 5.0e-4 / (2.4e-9 * 150.0 * GRAVITY_MM_S2);
    assert!(
        ((w1 - w0) / w0 - ratio).abs() < 1e-9,
        "増分比={} expected={ratio}",
        (w1 - w0) / w0
    );
}

/// 壁の仕上げ・増打ちの分は、`CorrectedLumped` の補正質点として残る。
///
/// 解析の質量行列は要素の**密度**からしか質量を作らないので、仕上げ・増打ちの
/// 面荷重はそこに現れない。補正質点の控除を総重量で行うと、控除だけされて分布質量
/// としては現れず、質量が黙って消える。控除は躯体（密度）分に限る必要がある。
#[test]
fn test_wall_finish_load_survives_corrected_lumped_mass() {
    use squid_n_core::model::AreaLoad;

    let base = generate_stories(&wall_model(), None).unwrap();
    let mut model = wall_model();
    model.wall_plates[0].loads = vec![AreaLoad {
        kind: "増打ち".into(),
        value: 5.0e-4,
    }];
    let gen = generate_stories(&model, None).unwrap();

    // 上の床の代表節点の質点質量。躯体分は控除されて 0 のままなので、増えた分は
    // そのまま仕上げ・増打ちの質量になる。
    let m0 = base.rep_nodes[1].mass.map(|m| m[0]).unwrap_or(0.0);
    let m1 = gen.rep_nodes[1].mass.map(|m| m[0]).unwrap_or(0.0);
    assert!(
        m1 > m0,
        "仕上げ・増打ちの質量が補正質点に残っていない: {m0} -> {m1}"
    );

    // 増分は「仕上げ分の重量の上端 2 節点ぶん ÷ g」に一致する。
    let dw = gen.stories[1].seismic_weight.unwrap() - base.stories[1].seismic_weight.unwrap();
    assert!(
        ((m1 - m0) - dw / GRAVITY_MM_S2).abs() / (dw / GRAVITY_MM_S2) < 1e-9,
        "質点質量の増分={} 期待={}",
        m1 - m0,
        dw / GRAVITY_MM_S2
    );
}

#[test]
fn test_wall_self_weight_uses_clear_dimensions_of_boundary_members() {
    // §壁自重: 耐震壁の重量は周辺の柱梁の内法寸法で計算する。
    // 側柱 500 角 ×2、上下梁 400×700 を壁の 4 辺に配置すると、
    // 内法係数 = (L−500/2×2)/L × (H−700/2×2)/H が芯々面積に乗じられる。
    let mut model = wall_model();
    // 側柱・上下梁用の断面（線材）。
    model.sections.push(Section {
        id: SectionId(1),
        name: "C500".into(),
        area: 0.0, // 自重 0（壁重量のみを観測するため）
        iy: 1.0,
        iz: 1.0,
        j: 1.0,
        depth: 500.0,
        width: 500.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.sections.push(Section {
        id: SectionId(2),
        name: "G400x700".into(),
        area: 0.0,
        iy: 1.0,
        iz: 1.0,
        j: 1.0,
        depth: 700.0,
        width: 400.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    let line = |id: u32, sec: u32, n0: u32, n1: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: [NodeId(n0), NodeId(n1)].into_iter().collect(),
        section: Some(SectionId(sec)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    // 壁節点順は [0,1,2,3] = 下辺(0-1)・右柱(1-2)・上辺(2-3)・左柱(3-0)。
    model.elements.push(line(1, 1, 1, 2)); // 右側柱
    model.elements.push(line(2, 1, 3, 0)); // 左側柱
    model.elements.push(line(3, 2, 0, 1)); // 下梁
    model.elements.push(line(4, 2, 2, 3)); // 上梁

    let gen = generate_stories(&model, None).unwrap();
    let (l, h) = (4000.0_f64, 3000.0_f64);
    let factor = ((l - 2.0 * 250.0) / l) * ((h - 2.0 * 350.0) / h);
    let w_total = 2.4e-9 * 150.0 * (l * h * factor) * GRAVITY_MM_S2;
    let expected = w_total / 2.0; // 上端2節点分のみ階重量に算入
    assert!(
        (gen.stories[1].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "got={}, expected={}",
        gen.stories[1].seismic_weight.unwrap(),
        expected
    );
}

#[test]
fn test_generate_stories_multi_sums_multiple_gravity_cases_and_dedupes() {
    let mut model = asymmetric_weight_model();
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(1),
        name: "LL".into(),
        nodal: vec![NodalLoad::manual(
            NodeId(2),
            [0.0, 0.0, -10000.0, 0.0, 0.0, 0.0],
        )],
        member: vec![],
    });

    // DL(400kN) + LL(10kN) = 410kN
    let gen = generate_stories_multi(&model, &[LoadCaseId(0), LoadCaseId(1)]).unwrap();
    assert_eq!(gen.stories[1].seismic_weight, Some(410000.0));

    // 重複 ID は 1 回だけ処理される（二重計上しない）
    let gen_dup =
        generate_stories_multi(&model, &[LoadCaseId(0), LoadCaseId(0), LoadCaseId(1)]).unwrap();
    assert_eq!(gen_dup.stories[1].seismic_weight, Some(410000.0));
}

/// `generate_stories_with_opts` の自重算入方法:
/// - `include_density_self_weight = false`（GravityCasesOnly）では密度からの自重
///   直接算入を行わず、重力ケースの内容だけを階重量へ算入する。
/// - 自重同期ケース（`self_weight_case_content`）を重力ケースとして渡した場合の
///   階重量が、密度直接算入（従来）の階重量と一致する
///   （自重の単一ソースオブトゥルース＝「DL」経由でも二重計上・欠落がない）。
#[test]
fn test_generate_stories_with_opts_self_weight_via_case_matches_density() {
    let mut model = two_story_model();

    // 従来: 密度から直接算入（重力ケースなし）。
    let by_density = generate_stories(&model, None).unwrap();

    // 自重をケース内容として与え、密度算入は無効化。
    // （two_story_model 組み込みの荷重ケースは重量比較の邪魔になるので除去）
    model.load_cases.clear();
    let (nodal, member) = crate::self_weight::self_weight_case_content(&model, &LoadCfg::default());
    model.load_cases.push(LoadCase {
        kind: LoadCaseKind::Dead,
        id: LoadCaseId(0),
        name: "DL".into(),
        nodal,
        member,
    });
    let by_case =
        generate_stories_with_opts(&model, &[LoadCaseId(0)], false, MassMethod::default()).unwrap();

    assert_eq!(by_density.stories.len(), by_case.stories.len());
    for (a, b) in by_density.stories.iter().zip(by_case.stories.iter()) {
        let (wa, wb) = (a.seismic_weight.unwrap(), b.seismic_weight.unwrap());
        assert!(
            (wa - wb).abs() < 1e-6 * wa.max(1.0),
            "story {} weight density={} case={}",
            a.name,
            wa,
            wb
        );
    }

    // include_density_self_weight = true のままケースも渡すと二重計上になる
    // （ガード側の except 選択が必要な旧構成の確認）。
    let doubled =
        generate_stories_with_opts(&model, &[LoadCaseId(0)], true, MassMethod::default()).unwrap();
    let w1 = by_density.stories[0].seismic_weight.unwrap();
    assert!(
        (doubled.stories[0].seismic_weight.unwrap() - 2.0 * w1).abs() < 1e-6 * w1,
        "自重をケースと密度の両方から算入すると 2 倍になるはず"
    );
}

/// `include_density_self_weight = false`（GravityCasesOnly）は重力ケースの内容だけを
/// 質点質量とし、モデル自重の置換も質量行列分の控除もしない。
#[test]
fn test_gravity_cases_only_uses_case_content_without_replacement() {
    let model = two_columns_with_dl_model();

    // 重力ケースなし: 質点質量は 0（None）。負の質量を作らない。
    for mm in [MassMethod::CorrectedLumped, MassMethod::LumpedOnly] {
        let gen = generate_stories_with_opts(&model, &[], false, mm).unwrap();
        for rep in &gen.rep_nodes {
            assert!(
                rep.mass.is_none_or(|m| m[0] >= 0.0),
                "重力ケースが無いのに質点質量が生じた: {:?}",
                rep.mass
            );
        }
        for s in &gen.stories {
            assert_eq!(s.seismic_weight, Some(0.0));
        }
    }

    // 手入力の DL 節点荷重のみ（自重を含まない）: 質点質量はそのケース分のみ。
    // 設計自重の置換（`node_weight − design_sw + physical_sw`）を行えば設計自重 > 物理質量
    // のため大幅に小さくなるが、GravityCasesOnly はその置換をしない。
    let gen = generate_stories_with_opts(&model, &[LoadCaseId(0)], false, MassMethod::LumpedOnly)
        .unwrap();
    let top = gen.rep_nodes[1].mass.expect("DL 節点荷重分の質点質量");
    let expected = 400000.0 / GRAVITY_MM_S2;
    assert!(
        (top[0] - expected).abs() < 1e-9 * expected,
        "モデル自重が差し引かれた: mt={} expected={}",
        top[0],
        expected
    );
    for rep in &gen.rep_nodes {
        if let Some(m) = rep.mass {
            assert!(m[0] >= 0.0, "負の質点質量: {}", m[0]);
        }
    }
}

/// 名前が「DL」で種別が固定荷重でも、内容が手入力（自動荷重による自重同期でない）なら
/// GravityCasesOnly は置換しない（ケース名・種別で自重を判別しない）。
#[test]
fn test_gravity_cases_only_does_not_replace_manual_dl_named_case() {
    let mut model = two_columns_with_dl_model();
    model.load_cases[0].kind = LoadCaseKind::Dead;
    model.load_cases[0].name = "DL".into();

    let gen = generate_stories_with_opts(&model, &[LoadCaseId(0)], false, MassMethod::LumpedOnly)
        .unwrap();
    let top = gen.rep_nodes[1].mass.expect("手入力節点荷重分の質点質量");
    let expected = 400000.0 / GRAVITY_MM_S2;
    assert!(
        (top[0] - expected).abs() < 1e-9 * expected,
        "名前だけ DL の手動ケースが置換された: mt={} expected={}",
        top[0],
        expected
    );
}

/// 自重同期済み DL（`self_weight_case_content`）を渡すと、両 MassMethod の総動的質量が
/// 一致する（CorrectedLumped は質点質量＋質量行列総和、LumpedOnly は質点質量合計）。
#[test]
fn test_synced_self_weight_mass_methods_agree() {
    let mut model = two_story_model();
    model.load_cases.clear();
    let (nodal, member) = crate::self_weight::self_weight_case_content(&model, &LoadCfg::default());
    model.load_cases.push(LoadCase {
        kind: LoadCaseKind::Dead,
        id: LoadCaseId(0),
        name: "DL".into(),
        nodal,
        member,
    });

    let corrected = generate_stories_with_synced_self_weight(
        &model,
        &[LoadCaseId(0)],
        MassMethod::CorrectedLumped,
    )
    .unwrap();
    let lumped =
        generate_stories_with_synced_self_weight(&model, &[LoadCaseId(0)], MassMethod::LumpedOnly)
            .unwrap();
    let sum_mass = |gen: &StoryGenResult| {
        gen.rep_nodes
            .iter()
            .filter_map(|n| n.mass)
            .map(|m| m[0])
            .sum::<f64>()
    };
    let matrix = main_frame_matrix_mass_equiv(&model) / GRAVITY_MM_S2;
    let corrected_total = sum_mass(&corrected) + matrix;
    let lumped_total = sum_mass(&lumped);
    assert!(
        (corrected_total - lumped_total).abs() < 1e-9 * lumped_total.max(1.0),
        "両方式の総動的質量が一致しない: corrected={corrected_total} lumped={lumped_total}"
    );
}

/// 自重同期済み DL を渡した LumpedOnly の質点質量は物理密度 7.85 t/m³ ベースになり、
/// 設計重量 78.5 kN/m³ ベースの地震用重量/g とは一致しない。
#[test]
fn test_synced_self_weight_steel_mass_is_physical() {
    let (len, area) = (4000.0, 90000.0);
    let mut model = single_beam_model(len, 7.85e-9, area, None, RigidZone::default(), None);
    let (nodal, member) = crate::self_weight::self_weight_case_content(&model, &LoadCfg::default());
    model.load_cases.push(LoadCase {
        kind: LoadCaseKind::Dead,
        id: LoadCaseId(0),
        name: "DL".into(),
        nodal,
        member,
    });

    let gen =
        generate_stories_with_synced_self_weight(&model, &[LoadCaseId(0)], MassMethod::LumpedOnly)
            .unwrap();
    // 上端節点の質点質量は物理質量の上端半分（7.85 t/m³ ベース）。
    let physical_half = 7.85e-9 * area * len * GRAVITY_MM_S2 / 2.0;
    let m = gen.rep_nodes[1].mass.expect("自重同期済み DL の質点質量");
    assert!(
        (m[0] - physical_half / GRAVITY_MM_S2).abs() < 1e-9 * (physical_half / GRAVITY_MM_S2),
        "動的質量が 7.85 ベースでない: {}",
        m[0]
    );
    // 地震用重量（設計 78.5 ベース）は物理質量より大きい。
    assert!(
        gen.stories[1].seismic_weight.unwrap() / GRAVITY_MM_S2 > m[0],
        "設計重量ベースの地震用重量が物理質量を下回らない"
    );
}

/// 自重を含まない重力ケース（空・手入力のみ）を SyncedGravityCases へ渡すと、
/// 置換量（設計自重）が含まれないためエラーになる（clamp で隠さない）。
#[test]
fn test_synced_self_weight_rejects_case_without_self_weight() {
    let mut model = two_columns_with_dl_model();
    // 自重を含まない（荷重が空の）DL ケース。
    model.load_cases.clear();
    model.load_cases.push(LoadCase {
        kind: LoadCaseKind::Dead,
        id: LoadCaseId(0),
        name: "DL".into(),
        nodal: vec![],
        member: vec![],
    });

    for mm in [MassMethod::CorrectedLumped, MassMethod::LumpedOnly] {
        let err =
            generate_stories_with_synced_self_weight(&model, &[LoadCaseId(0)], mm).unwrap_err();
        assert!(
            err.contains("重力ケースに想定した自重が含まれていません"),
            "{err}"
        );
    }

    // 空の gravity_lcs でも置換量が含まれないためエラーになる。
    assert!(generate_stories_with_synced_self_weight(&model, &[], MassMethod::LumpedOnly).is_err());
}

/// DL が無く密度から直接算入する経路でも、取り付く壁版の自重が階重量へ入る
/// （囲まれた壁・フレーム外雑壁と同じ。抜け落ちは危険側）。
#[test]
fn test_density_seismic_weight_includes_attached_wall_plate() {
    use squid_n_core::model::{LoadTransfer, RegionAnchor};

    let mut model = two_story_model();
    let baseline = generate_stories(&model, None).unwrap();

    model.sections.push(Section {
        id: SectionId(1),
        name: "壁 t150".into(),
        area: 0.0,
        iy: 1.0,
        iz: 1.0,
        j: 1.0,
        depth: 0.0,
        width: 0.0,
        as_y: 1.0,
        as_z: 1.0,
        floor: None,
        panel_thickness: None,
        thickness: Some(150.0),
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    let plate = WallPlate {
        self_weight_shares: Vec::new(),
        id: WallPlateId(0),
        shape: WallPlateShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(4), NodeId(5)],
                span: [0.0, 1.0],
                transfer: LoadTransfer::Anchor,
            },
            extent: Some([1000.0, 1000.0]),
        },
        section: Some(SectionId(1)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings: Vec::new(),
        loads: vec![],
        slit: Default::default(),
    };
    let expected = model
        .wall_plate_self_weight(&plate, &model)
        .expect("自重が求まる");
    model.wall_plates.push(plate);

    let with_wall = generate_stories(&model, None).unwrap();
    let w0 = baseline.stories[2].seismic_weight.unwrap();
    let w1 = with_wall.stories[2].seismic_weight.unwrap();
    assert!(
        ((w1 - w0) - expected).abs() / expected < 1e-6,
        "屋根階の増分={} expected={}",
        w1 - w0,
        expected
    );
}

// ------------------------------------------------------------------
// §壁開口・柱際スリット
// ------------------------------------------------------------------

#[test]
fn test_wall_opening_deduction_and_opening_weight() {
    let mut model = wall_model();
    model.wall_plates[0].opening_area = 1_000_000.0;
    model.wall_plates[0].opening_weight = 5000.0;
    let gen = generate_stories(&model, None).unwrap();
    let area = 4000.0 * 3000.0;
    let net_area = area - 1_000_000.0;
    let w_total = (2.4e-9 * 150.0 * net_area * GRAVITY_MM_S2 + 5000.0).max(0.0);
    let expected = w_total / 2.0; // 上端2節点分(4節点等分の半分)
    assert!(
        (gen.stories[1].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[1].seismic_weight.unwrap()
    );
}

#[test]
fn test_wall_opening_deduction_clamped_non_negative() {
    // 開口面積が壁面積を超える極端な入力でも自重が負にならない(clamp)。
    let mut model = wall_model();
    model.wall_plates[0].opening_area = 4000.0 * 3000.0 * 2.0; // 壁面積を超える
    let gen = generate_stories(&model, None).unwrap();
    assert_eq!(gen.stories[1].seismic_weight, Some(0.0));
}

#[test]
fn test_column_face_slit_does_not_change_self_weight_destination() {
    // §壁自重: 柱際スリットは要素壁の自重の行き先を変えない。行き先を変えるのは
    // 梁際のスリットだけである。
    let base = generate_stories(&wall_model(), None).unwrap().stories[1]
        .seismic_weight
        .unwrap();

    let mut model = wall_model();
    model.wall_plates[0].slit.column_face = [true, true];
    let slit = generate_stories(&model, None).unwrap().stories[1]
        .seismic_weight
        .unwrap();

    let area = 4000.0 * 3000.0;
    let w_total = 2.4e-9 * 150.0 * area * GRAVITY_MM_S2;
    // 上端 2 節点分（4 節点等分の半分）。
    assert!((base - w_total / 2.0).abs() < 1e-6, "{base}");
    assert!((slit - base).abs() < 1e-9, "{slit} != {base}");
}

/// §壁自重: 下辺の梁際スリットは、自重を全量上辺へ寄せる（三方スリットの垂れ壁型）。
///
/// `wall_model()` の頂点は下 2 節点・上 2 節点で、上位 2 節点はどちらも階に属する。
/// 通常配分（上端 2 節点で w/2）に対し、下辺が切れると階の地震用重量は w 全量になる。
#[test]
fn test_bottom_beam_face_slit_sends_self_weight_to_top() {
    let mut model = wall_model();
    model.wall_plates[0].slit.beam_face = [true, false];
    let got = generate_stories(&model, None).unwrap().stories[1]
        .seismic_weight
        .unwrap();

    let area = 4000.0 * 3000.0;
    let w_total = 2.4e-9 * 150.0 * area * GRAVITY_MM_S2;
    assert!((got - w_total).abs() < 1e-6, "{got}");
}

/// §壁自重: 上辺の梁際スリットは、自重を全量下辺へ寄せる（三方スリットの腰壁型）。
///
/// 下端 2 節点は柱脚（階に属さない基部）なので、階の地震用重量は 0 になる。
#[test]
fn test_top_beam_face_slit_sends_self_weight_to_bottom() {
    let mut model = wall_model();
    model.wall_plates[0].slit.beam_face = [false, true];
    let got = generate_stories(&model, None).unwrap().stories[1]
        .seismic_weight
        .unwrap();
    assert!(got.abs() < 1e-6, "{got}");
}

// ------------------------------------------------------------------
// §取り付く壁版（旧フレーム外雑壁の後継）
// ------------------------------------------------------------------

/// 柱 1 本（節点 0＝基部、節点 1＝頂部）と、その頂部へ取り付く壁版 1 枚のモデル。
/// 壁版の自重は「板厚 × 材料密度 × 重力加速度」で求まるので、期待値も同じ式で書ける。
fn single_column_with_attached_wall(transfer: LoadTransfer) -> (Model, f64) {
    use squid_n_core::model::RegionAnchor;

    let mut model = Model::default();
    for (id, z, restraint) in [(0, 0.0, Dof6Mask::FIXED), (1, 3000.0, Dof6Mask::FREE)] {
        model.nodes.push(Node {
            id: NodeId(id),
            coord: [0.0, 0.0, z],
            restraint,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    // 取付き線の両端は相異なる節点でなければならないため、頂部にもう 1 点置く。
    model.nodes.push(Node {
        id: NodeId(2),
        coord: [4000.0, 0.0, 3000.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.sections.push(Section {
        id: SectionId(0),
        name: "Col".into(),
        area: 0.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    // 壁版用の断面（板厚 120）。
    let mut wall_sec = model.sections[0].clone();
    wall_sec.id = SectionId(1);
    wall_sec.name = "Wall".into();
    wall_sec.thickness = Some(120.0);
    model.sections.push(wall_sec);
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    // 柱（節点 0-1）と頂部の梁（節点 1-2）。
    for (id, i, j) in [(0_u32, 0_u32, 1_u32), (1, 1, 2)] {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        });
    }
    model.wall_plates.push(WallPlate {
        self_weight_shares: Vec::new(),
        id: WallPlateId(0),
        shape: WallPlateShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(1), NodeId(2)],
                span: [0.0, 1.0],
                transfer,
            },
            extent: Some([200.0, 200.0]),
        },
        section: Some(SectionId(1)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings: vec![],
        loads: vec![],
        slit: Default::default(),
    });
    let total = 2.4e-9 * 120.0 * GRAVITY_MM_S2 * 4000.0 * 200.0;
    (model, total)
}

/// 取付き線へ分布させる壁版の重量は、全量がその階に残る。
#[test]
fn test_attached_wall_anchor_transfer_conserves_total_weight() {
    let (model, total) = single_column_with_attached_wall(LoadTransfer::Anchor);
    let gen = generate_stories(&model, None).unwrap();
    // 柱・梁の自重は断面積 0 なので 0。壁版の分だけが現れる。
    assert!(
        (gen.stories[1].seismic_weight.unwrap() - total).abs() < 1e-6,
        "{}",
        gen.stories[1].seismic_weight.unwrap()
    );
}

/// 両端の柱へ集中させる壁版でも、重量は取付き線の高さに残る。
///
/// 旧フレーム外雑壁の「柱」伝達は、最も近い柱要素の**上下 2 節点**へ 1/2 ずつ配って
/// いた。下端は基部でどの階にも属さないため、壁の重量の半分が階の地震用重量から
/// 黙って消えていた。上階の地震力を過小に見る危険側の挙動である。後継の
/// `LoadTransfer::Columns` は取付き線の両端へ集中するので、全量がその階に残る。
#[test]
fn test_attached_wall_columns_transfer_keeps_weight_at_anchor_line() {
    let (model, total) = single_column_with_attached_wall(LoadTransfer::Columns);
    let gen = generate_stories(&model, None).unwrap();
    assert!(
        (gen.stories[1].seismic_weight.unwrap() - total).abs() < 1e-6,
        "重量の一部が基部へ逃げている: {}",
        gen.stories[1].seismic_weight.unwrap()
    );
}

// ------------------------------------------------------------------
// §ダンパー自重
// ------------------------------------------------------------------

#[test]
fn test_damper_weight_replaces_section_self_weight() {
    let len = 4000.0;
    let damper = DamperSpec {
        elem: ElemId(0),
        device_weight: 20000.0,
        device_length: 1000.0,
        support_area: 5000.0,
    };
    let cfg = LoadCfg {
        dampers: vec![damper],
        ..Default::default()
    };
    let model = single_beam_model(len, 7.85e-9, 90000.0, None, RigidZone::default(), Some(cfg));
    let gen = generate_stories(&model, None).unwrap();
    let support_len = (len - 1000.0_f64).max(0.0);
    let w = 20000.0 + 5000.0 * support_len * 78.5e-6;
    let expected = w / 2.0;
    assert!(
        (gen.stories[1].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[1].seismic_weight.unwrap()
    );
}

#[test]
fn test_damper_zero_device_weight_counts_support_only() {
    // 「自重を考慮しない部材」: device_weight=0 かつ support_area>0 は支持部のみ算入。
    let len = 4000.0;
    let damper = DamperSpec {
        elem: ElemId(0),
        device_weight: 0.0,
        device_length: 500.0,
        support_area: 8000.0,
    };
    let cfg = LoadCfg {
        dampers: vec![damper],
        ..Default::default()
    };
    let model = single_beam_model(len, 7.85e-9, 90000.0, None, RigidZone::default(), Some(cfg));
    let gen = generate_stories(&model, None).unwrap();
    let support_len = (len - 500.0_f64).max(0.0);
    let w = 8000.0 * support_len * 78.5e-6;
    let expected = w / 2.0;
    assert!(
        (gen.stories[1].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[1].seismic_weight.unwrap()
    );
    // 断面自重(ρ·A·L·g)は使われない(桁違いに大きい値とは一致しない)。
    let naive = 7.85e-9 * 90000.0 * len * GRAVITY_MM_S2 / 2.0;
    assert!((gen.stories[1].seismic_weight.unwrap() - naive).abs() > 1.0);
}

// ------------------------------------------------------------------
// §仕上げ面重量の自動換算
// ------------------------------------------------------------------

#[test]
fn test_finish_area_weight_column_perimeter_four_side() {
    // single_beam_model は鉛直材(柱)。φ=2(b+D)、b=D=300 (helper内の断面固定値)。
    let len = 4000.0;
    let area = 90000.0;
    let density = 7.85e-9;
    let wf = 0.002;
    let cfg = LoadCfg {
        finish_area_weight: vec![(ElemId(0), wf)],
        ..Default::default()
    };
    let model = single_beam_model(len, density, area, None, RigidZone::default(), Some(cfg));
    let gen = generate_stories(&model, None).unwrap();
    let phi = 2.0 * (300.0 + 300.0);
    let expected = (78.5e-6 * area * len + wf * phi * len) / 2.0;
    assert!(
        (gen.stories[1].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[1].seismic_weight.unwrap()
    );
}

#[test]
fn test_finish_area_weight_beam_perimeter_three_side() {
    // 水平梁(非鉛直)。φ=b+2D の三面仕上げ。
    let mut model = Model::default();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [0.0, 0.0, 3000.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.nodes.push(Node {
        id: NodeId(2),
        coord: [6000.0, 0.0, 3000.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.sections.push(Section {
        id: SectionId(0),
        name: "Beam".into(),
        area: 90000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 600.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "S".into(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(1), NodeId(2)].into_iter().collect(),
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    let wf = 0.0015;
    model.load_cfg = Some(LoadCfg {
        finish_area_weight: vec![(ElemId(0), wf)],
        ..Default::default()
    });
    let gen = generate_stories(&model, None).unwrap();
    let len = 6000.0;
    let phi = 300.0 + 2.0 * 600.0;
    // 両端(node1, node2)とも z=3000 の同一階に属するため、全量がその階に現れる。
    let expected = 78.5e-6 * 90000.0 * len + wf * phi * len;
    assert!(
        (gen.stories[1].seismic_weight.unwrap() - expected).abs() < 1e-6,
        "{}",
        gen.stories[1].seismic_weight.unwrap()
    );
}

// ------------------------------------------------------------------
// §柱の長さ(下階柱なし時の柱脚梁せい付加)
// ------------------------------------------------------------------

#[test]
fn test_base_column_without_lower_column_adds_max_beam_depth() {
    let mut model = Model::default();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    }); // 柱脚(下階柱なし) & 梁の一端
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [0.0, 0.0, 3000.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    }); // 柱頭
    model.nodes.push(Node {
        id: NodeId(2),
        coord: [4000.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    }); // 梁の他端(基部)
    model.sections.push(Section {
        id: SectionId(0),
        name: "Col".into(),
        area: 90000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.sections.push(Section {
        id: SectionId(1),
        name: "Beam".into(),
        area: 0.0, // 自重寄与ゼロにして柱脚梁せい付加のみを検証する
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 600.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    model.elements.push(ElementData {
        id: ElemId(1),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(2)].into_iter().collect(),
        section: Some(SectionId(1)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    let gen = generate_stories(&model, None).unwrap();
    // 柱自重は節点伝達: 上端 = W_column/2、基部 = W_column/2 + Wextra
    // （Wextra は柱脚に取付く梁の最大せい 600 分の柱重量）。
    let w_col = 2.4e-9 * 90000.0 * 3000.0 * GRAVITY_MM_S2;
    let w_extra = 2.4e-9 * 90000.0 * 600.0 * GRAVITY_MM_S2;
    assert!(
        (gen.stories[1].seismic_weight.unwrap() - w_col / 2.0).abs() < 1e-6,
        "上端階 {}",
        gen.stories[1].seismic_weight.unwrap()
    );
    assert!(
        (gen.stories[0].seismic_weight.unwrap() - (w_col / 2.0 + w_extra)).abs() < 1e-6,
        "基部階 {}",
        gen.stories[0].seismic_weight.unwrap()
    );
}

/// 基部に**鉛直なブレース**が下階へ接続していても、ブレースは「下階の柱」とみなさない。
///
/// `is_column` は 2 節点の鉛直 `ElementKind::Beam` のみを柱とし、`has_column_below` も
/// `ElementKind::Beam` だけを下階柱として数える（`ElementKind::Brace` は除外）。したがって
/// 基部節点に下階へ伸びる鉛直ブレースが接続していても、その上の RC 柱には柱脚梁せい相当の
/// 追加自重 `W_extra` が加算される。ブレースが誤って下階柱とみなされれば期待値は
/// `W_column/2` となり、このテストが失敗する。
#[test]
fn test_base_column_with_lower_brace_still_adds_max_beam_depth() {
    let mut model = Model::default();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    }); // 柱脚(下階はブレースのみ) & 梁の一端 & ブレース上端
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [0.0, 0.0, 3000.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    }); // 柱頭
    model.nodes.push(Node {
        id: NodeId(2),
        coord: [4000.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    }); // 柱脚梁の他端(せい 600)
    model.nodes.push(Node {
        id: NodeId(3),
        coord: [0.0, 0.0, -3000.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    }); // 下階のブレース下端
    model.sections.push(Section {
        id: SectionId(0),
        name: "Col".into(),
        area: 90000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.sections.push(Section {
        id: SectionId(1),
        name: "Beam".into(),
        area: 0.0, // 自重寄与ゼロにして柱脚梁せい付加のみを検証する
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 600.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.sections.push(Section {
        id: SectionId(2),
        name: "Brace".into(),
        area: 0.0, // 自重は寄与させず、下階柱判定だけに効かせる
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 200.0,
        width: 200.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    // 検証対象の RC 柱(鉛直 Beam, node0-node1)。
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    // 柱脚に取付く水平梁(area=0、せい 600 が Wextra に効く)。
    model.elements.push(ElementData {
        id: ElemId(1),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(2)].into_iter().collect(),
        section: Some(SectionId(1)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    // 下階へ伸びる鉛直ブレース。柱と同様に鉛直だが `ElementKind::Brace` なので
    // `has_column_below` の下階柱には数えない。
    model.elements.push(ElementData {
        id: ElemId(2),
        kind: ElementKind::Brace {
            tension_only: false,
        },
        nodes: [NodeId(0), NodeId(3)].into_iter().collect(),
        section: Some(SectionId(2)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    let gen = generate_stories(&model, None).unwrap();
    // 階は標高 -3000 / 0 / 3000 の 3 つ。柱脚の基部は index 1、柱頭は index 2。
    // 柱自重は節点伝達: 上端 = W_column/2、基部 = W_column/2 + Wextra。
    // Wextra は柱脚に取付く梁の最大せい 600 分の柱重量。ブレースは下階の柱ではない。
    let w_col = 2.4e-9 * 90000.0 * 3000.0 * GRAVITY_MM_S2;
    let w_extra = 2.4e-9 * 90000.0 * 600.0 * GRAVITY_MM_S2;
    assert!(
        (gen.stories[2].seismic_weight.unwrap() - w_col / 2.0).abs() < 1e-6,
        "上端階 {}",
        gen.stories[2].seismic_weight.unwrap()
    );
    assert!(
        (gen.stories[1].seismic_weight.unwrap() - (w_col / 2.0 + w_extra)).abs() < 1e-6,
        "基部階 {} (ブレースを下階柱とみなして追加が抑制されていないこと)",
        gen.stories[1].seismic_weight.unwrap()
    );
}

/// 基部に**3 節点以上の鉛直な `ElementKind::Beam`**が下階へ接続していても、それを
/// 「下階の柱」とみなさない（下階柱は 2 節点の鉛直 `ElementKind::Beam` のみ）。
///
/// `is_column` は 2 節点の鉛直 `ElementKind::Beam` のみを柱とするため、`has_column_below`
/// も `e2.nodes.len() == 2` を要求する。したがって基部節点に下階へ伸びる 3 節点の鉛直
/// Beam が接続していても、その上の RC 柱には柱脚梁せい相当の追加自重 `W_extra` が加算される
/// （上端 = `W_column/2`、基部 = `W_column/2 + W_extra`）。この `nodes.len() == 2` を欠く
/// 実装では 3 節点 Beam の先頭 2 節点が下階柱と誤判定され `W_extra` が 0 になる。
#[test]
fn test_base_column_with_lower_three_node_vertical_beam_still_adds_max_beam_depth() {
    let mut model = Model::default();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    }); // 柱脚(下階柱なし) & 梁の一端 & 3節点鉛直Beamの始点
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [0.0, 0.0, 3000.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    }); // 柱頭
    model.nodes.push(Node {
        id: NodeId(2),
        coord: [4000.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    }); // 柱脚梁の他端(せい 600)
    model.nodes.push(Node {
        id: NodeId(3),
        coord: [0.0, 0.0, -3000.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    }); // 3節点鉛直Beamの2番目(基部より下)
    model.nodes.push(Node {
        id: NodeId(4),
        coord: [3000.0, 0.0, -3000.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    }); // 3節点鉛直Beamの3番目(下階の階を増やさない標高)
    model.sections.push(Section {
        id: SectionId(0),
        name: "Col".into(),
        area: 90000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.sections.push(Section {
        id: SectionId(1),
        name: "Beam".into(),
        area: 0.0, // 自重寄与ゼロにして柱脚梁せい付加のみを検証する
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 600.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.sections.push(Section {
        id: SectionId(2),
        name: "VBeam3".into(),
        area: 0.0, // 自重は寄与させず、下階柱判定だけに効かせる
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    // 検証対象の RC 柱(鉛直 Beam, node0-node1)。
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    // 柱脚に取付く水平梁(area=0、せい 600 が Wextra に効く)。
    model.elements.push(ElementData {
        id: ElemId(1),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(2)].into_iter().collect(),
        section: Some(SectionId(1)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    // 基部から下階へ伸びる 3 節点の鉛直 Beam。先頭 2 節点(node0-node3)が鉛直で
    // node3 が基部より下にあるが、2 節点でないため下階柱には数えない。
    model.elements.push(ElementData {
        id: ElemId(2),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(3), NodeId(4)].into_iter().collect(),
        section: Some(SectionId(2)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    let gen = generate_stories(&model, None).unwrap();
    // 階は標高 -3000 / 0 / 3000 の 3 つ。柱脚の基部は index 1、柱頭は index 2。
    // 柱自重は節点伝達: 上端 = W_column/2、基部 = W_column/2 + Wextra。
    // Wextra は柱脚に取付く梁の最大せい 600 分の柱重量。3 節点の鉛直 Beam は下階柱ではない。
    let w_col = 2.4e-9 * 90000.0 * 3000.0 * GRAVITY_MM_S2;
    let w_extra = 2.4e-9 * 90000.0 * 600.0 * GRAVITY_MM_S2;
    assert!(
        (gen.stories[2].seismic_weight.unwrap() - w_col / 2.0).abs() < 1e-6,
        "上端階 {}",
        gen.stories[2].seismic_weight.unwrap()
    );
    assert!(
        (gen.stories[1].seismic_weight.unwrap() - (w_col / 2.0 + w_extra)).abs() < 1e-6,
        "基部階 {} (3節点鉛直Beamを下階柱とみなして追加が抑制されていないこと)",
        gen.stories[1].seismic_weight.unwrap()
    );
}

#[test]
fn test_base_column_with_lower_column_does_not_add_beam_depth() {
    // 下階に柱がある場合は梁せいを付加しない(誤って常時付加しないことの回帰確認)。
    let mut model = Model::default();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, -3000.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    }); // 最下層(基部)
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    }); // 1F: 下階に柱(node0-node1)があるので梁せい付加なし
    model.nodes.push(Node {
        id: NodeId(2),
        coord: [0.0, 0.0, 3000.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    }); // 2F
    model.nodes.push(Node {
        id: NodeId(3),
        coord: [4000.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    }); // 1F 位置に取付く梁の他端
    model.sections.push(Section {
        id: SectionId(0),
        name: "Col".into(),
        area: 90000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.sections.push(Section {
        id: SectionId(1),
        name: "ColLower".into(),
        area: 0.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.sections.push(Section {
        id: SectionId(2),
        name: "Beam".into(),
        area: 0.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 600.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    // 下階の柱(node0-node1)。area=0 で自重寄与ゼロ(有無の判定のみに使う)。
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
        section: Some(SectionId(1)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    // 検証対象の柱(node1-node2)。
    model.elements.push(ElementData {
        id: ElemId(1),
        kind: ElementKind::Beam,
        nodes: [NodeId(1), NodeId(2)].into_iter().collect(),
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    // 1F(node1)に取付く梁(area=0、せい付加の誤検出があれば効いてしまう)。
    model.elements.push(ElementData {
        id: ElemId(2),
        kind: ElementKind::Beam,
        nodes: [NodeId(1), NodeId(3)].into_iter().collect(),
        section: Some(SectionId(2)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    let gen = generate_stories(&model, None).unwrap();
    // story0(1F, node1・node3) には下階柱(area0)/2 + 検証対象柱の下半分 + 梁(area0)/2。
    // 梁せい付加が誤って効いていれば eff_len が 3600 になり期待値からずれる。
    let w_upper = 2.4e-9 * 90000.0 * 3000.0 * GRAVITY_MM_S2; // 梁せい付加なし(eff_len=3000)
    let expected_story0 = w_upper / 2.0;
    assert!(
        (gen.stories[1].seismic_weight.unwrap() - expected_story0).abs() < 1e-6,
        "{}",
        gen.stories[1].seismic_weight.unwrap()
    );
}

// ------------------------------------------------------------------
// §K型ブレースの重量配分
// ------------------------------------------------------------------

/// K型ブレース: 基準節点(node2, node3)から内部節点(node4)へ2本のブレースが
/// 集まる形。ブレース断面積を非対称にして配分規則による重心の違いを検出する。
fn k_brace_model(rule: KBraceWeightRule) -> Model {
    let mut model = Model::default();
    let coords = [
        [0.0, 0.0, 0.0],
        [4000.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [4000.0, 0.0, 3000.0],
        [2000.0, 0.0, 3000.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if i < 2 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    // 柱(自重ゼロ、node2/node3 を「基準節点」化するために存在)
    model.sections.push(Section {
        id: SectionId(0),
        name: "Col".into(),
        area: 0.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 8000.0,
        as_z: 8000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    // ブレース1(node2-node4)
    model.sections.push(Section {
        id: SectionId(1),
        name: "Brace1".into(),
        area: 10000.0,
        iy: 1.0e6,
        iz: 1.0e6,
        j: 1.0e6,
        depth: 200.0,
        width: 200.0,
        as_y: 1000.0,
        as_z: 1000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    // ブレース2(node3-node4): 面積を2倍にして非対称にする
    model.sections.push(Section {
        id: SectionId(2),
        name: "Brace2".into(),
        area: 20000.0,
        iy: 1.0e6,
        iz: 1.0e6,
        j: 1.0e6,
        depth: 200.0,
        width: 200.0,
        as_y: 1000.0,
        as_z: 1000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "S".into(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: None,
    });
    let axis = LocalAxis {
        ref_vector: [0.0, 0.0, 1.0],
    };
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(2)].into_iter().collect(),
        section: Some(SectionId(0)),
        local_axis: axis,
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    model.elements.push(ElementData {
        id: ElemId(1),
        kind: ElementKind::Beam,
        nodes: [NodeId(1), NodeId(3)].into_iter().collect(),
        section: Some(SectionId(0)),
        local_axis: axis,
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    model.elements.push(ElementData {
        id: ElemId(2),
        kind: ElementKind::Brace {
            tension_only: false,
        },
        nodes: [NodeId(2), NodeId(4)].into_iter().collect(),
        section: Some(SectionId(1)),
        local_axis: axis,
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    model.elements.push(ElementData {
        id: ElemId(3),
        kind: ElementKind::Brace {
            tension_only: false,
        },
        nodes: [NodeId(3), NodeId(4)].into_iter().collect(),
        section: Some(SectionId(2)),
        local_axis: axis,
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    model.load_cfg = Some(LoadCfg {
        k_brace_rule: rule,
        ..Default::default()
    });
    model
}

#[test]
fn test_k_brace_base_nodes_only_shifts_centroid_toward_base_nodes() {
    let density = 7.85e-9;
    let len = 2000.0; // node2-node4, node3-node4 とも水平距離2000
    let w1 = density * 10000.0 * len * GRAVITY_MM_S2;
    let w2 = density * 20000.0 * len * GRAVITY_MM_S2;

    let internal = k_brace_model(KBraceWeightRule::InternalNodes);
    let gen_internal = generate_stories(&internal, None).unwrap();
    // 手計算: node2=w1/2(x=0), node3=w2/2(x=4000), node4=(w1+w2)/2(x=2000)
    let expected_internal = (1000.0 * w1 + 3000.0 * w2) / (w1 + w2);
    assert!(
        (gen_internal.rep_nodes[1].coord[0] - expected_internal).abs() < 1e-2,
        "{}",
        gen_internal.rep_nodes[1].coord[0]
    );

    let base_only = k_brace_model(KBraceWeightRule::BaseNodesOnly);
    let gen_base = generate_stories(&base_only, None).unwrap();
    // 手計算: node2=w1(x=0), node3=w2(x=4000), node4=0
    let expected_base = 4000.0 * w2 / (w1 + w2);
    assert!(
        (gen_base.rep_nodes[1].coord[0] - expected_base).abs() < 1e-2,
        "{}",
        gen_base.rep_nodes[1].coord[0]
    );

    // 両者は明確に異なる(基準節点側、より重いブレースが繋がる node3 側へ寄る)。
    assert!(gen_base.rep_nodes[1].coord[0] > gen_internal.rep_nodes[1].coord[0]);

    // 総重量(層重量)自体は配分規則によらず保存される。
    assert!(
        (gen_internal.stories[1].seismic_weight.unwrap()
            - gen_base.stories[1].seismic_weight.unwrap())
        .abs()
            < 1e-6
    );
}

#[test]
fn test_k_brace_internal_nodes_default_is_half_half() {
    // 既定(InternalNodes)は両端 1/2 ずつ(従来どおり)であることを回帰確認する。
    let mut model = k_brace_model(KBraceWeightRule::InternalNodes);
    model.load_cfg = None; // 既定値(LoadCfg::default())でも InternalNodes になることを確認
    let gen = generate_stories(&model, None).unwrap();
    let density = 7.85e-9;
    let len = 2000.0;
    let w1 = density * 10000.0 * len * GRAVITY_MM_S2;
    let w2 = density * 20000.0 * len * GRAVITY_MM_S2;
    let expected = (1000.0 * w1 + 3000.0 * w2) / (w1 + w2);
    assert!(
        (gen.rep_nodes[1].coord[0] - expected).abs() < 1e-2,
        "{}",
        gen.rep_nodes[1].coord[0]
    );
}

/// 断面形状と材料を階ごとに割り当てた [`two_story_model`]。
/// `lower`・`upper` はそれぞれ 1F・2F の柱梁に与える形状と材料の区分。
///
/// 階の構造種別は部材の構造種別から決まり、それは材料の区分で決まるため、
/// 形状だけでなく材料も階ごとに割り当てる。
fn two_story_model_with_shapes(
    lower: (squid_n_core::section_shape::SectionShape, MaterialCategory),
    upper: (squid_n_core::section_shape::SectionShape, MaterialCategory),
) -> Model {
    let mut model = two_story_model();
    model.sections = vec![
        lower.0.to_section(SectionId(0), "1F".into()),
        upper.0.to_section(SectionId(1), "2F".into()),
    ];
    // 既定の材料（MaterialId(0)）を 1F 用に置き換え、2F 用を追加する。
    model.materials[0].category = lower.1;
    let mut upper_mat = model.materials[0].clone();
    upper_mat.id = MaterialId(1);
    upper_mat.category = upper.1;
    model.materials.push(upper_mat);
    // 部材の所属階は「材端節点のうち最も高い節点」で決まる。
    // 節点 Z: 0/0/3500/3500/7000/7000 → 1F = 柱(0-2,1-3)・梁(2-3)、2F = 柱(2-4,3-5)・梁(4-5)。
    for e in &mut model.elements {
        let top_z = e
            .nodes
            .iter()
            .map(|n| model.nodes[n.index()].coord[2])
            .fold(f64::NEG_INFINITY, f64::max);
        let lower_story = top_z <= 3500.0;
        e.section = Some(if lower_story {
            SectionId(0)
        } else {
            SectionId(1)
        });
    }
    // 材料は断面が持つ。断面 0 = 1F、断面 1 = 2F にそれぞれ割り当てる。
    model.sections[0].material = Some(MaterialId(0));
    model.sections[1].material = Some(MaterialId(1));
    model
}

fn rc_rect_shape() -> squid_n_core::section_shape::SectionShape {
    use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};
    let bars = BarSet {
        count: 4,
        dia: 22.0,
        layers: 1,
    };
    squid_n_core::section_shape::SectionShape::RcRect {
        b: 600.0,
        d: 600.0,
        rebar: RcRebar {
            main_x: bars.clone(),
            main_y: bars,
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
            },
        },
    }
}

fn steel_h_shape() -> squid_n_core::section_shape::SectionShape {
    squid_n_core::section_shape::SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    }
}

/// 階の主要構造種別は、その階に属する柱・梁の構造種別から自動判定される
/// （下階 RC・上階 S の混合構造で階ごとに別々に判定されること）。
#[test]
fn test_generate_infers_story_structure_from_members() {
    use squid_n_core::model::StoryStructure;
    let model = two_story_model_with_shapes(
        (rc_rect_shape(), MaterialCategory::Concrete),
        (steel_h_shape(), MaterialCategory::Steel),
    );
    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    assert_eq!(gen.stories.len(), 3, "基部の床を含めて 3 階");
    // 構造種別は層の属性で、層の上端の階が持つ（下層 RC・上層 S）。
    assert_eq!(gen.stories[1].structure, StoryStructure::Rc);
    assert_eq!(gen.stories[2].structure, StoryStructure::S);
}

/// 構造種別は断面形状ではなく材料の区分で決まる。
/// H 形の断面でも材料がコンクリートなら、その階は RC になる。
#[test]
fn test_generate_story_structure_follows_material_not_shape() {
    use squid_n_core::model::StoryStructure;
    let model = two_story_model_with_shapes(
        (steel_h_shape(), MaterialCategory::Concrete),
        (steel_h_shape(), MaterialCategory::Steel),
    );
    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    assert_eq!(gen.stories[1].structure, StoryStructure::Rc);
    assert_eq!(gen.stories[2].structure, StoryStructure::S);
}

/// 形状定義を持たない断面（カタログ数値の直入力）でも材料の区分で判定できる。
#[test]
fn test_generate_story_structure_uses_material_without_shapes() {
    use squid_n_core::model::StoryStructure;
    let model = two_story_model(); // shape: None の断面＋鋼材の材料
    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    // 構造種別は層の属性で、層の上端の階が持つ。基部の床はどの層の上端でもない。
    assert!(gen.stories[1..]
        .iter()
        .all(|s| s.structure == StoryStructure::S));
}

/// 断面も材料も未割当の部材は種別を判定できないため集計から除く。
/// 対象部材が 1 本もない階は既定の RC になる（略算周期 α で S を算入しない安全側）。
#[test]
fn test_generate_story_structure_defaults_to_rc_without_section_and_material() {
    use squid_n_core::model::StoryStructure;
    let mut model = two_story_model();
    for e in &mut model.elements {
        e.section = None;
    }
    for s in &mut model.sections {
        s.material = None;
    }
    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    assert!(gen
        .stories
        .iter()
        .all(|s| s.structure == StoryStructure::Rc));
}

/// 階の再生成では、利用者が決める欄（階名・階レベル・階種別・地震用重量の手入力）を
/// 既存の階定義からそのまま引き継ぐ。所属節点・算定重量だけが更新される。
#[test]
fn test_regeneration_keeps_user_defined_story_fields() {
    use squid_n_core::model::StoryLevelKind;
    let mut model = two_story_model();
    // 1 回目の生成結果をモデルへ適用し、利用者の入力を加える。
    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    model.stories = gen.stories;
    model.stories[0].name = "1FL".into();
    model.stories[0].weight_override = Some(12345.0);
    model.stories[0].seismic_weight = Some(12345.0);
    model.stories[1].name = "PH".into();
    model.stories[1].level_kind = StoryLevelKind::Penthouse { k: 0.7 };

    let fresh = generate_stories(&model, Some(LoadCaseId(0)))
        .unwrap()
        .stories;
    assert_eq!(fresh[0].name, "1FL", "階名を引き継ぐ");
    assert_eq!(fresh[0].weight_override, Some(12345.0));
    assert_eq!(
        fresh[0].seismic_weight,
        Some(12345.0),
        "手入力の重量が算定値へ優先する"
    );
    assert_eq!(fresh[1].name, "PH");
    assert_eq!(
        fresh[1].level_kind,
        StoryLevelKind::Penthouse { k: 0.7 },
        "階の種別も引き継ぐ"
    );
    assert_eq!(fresh[1].weight_override, None, "手入力のない階はそのまま");
    assert!(
        fresh[1].seismic_weight.unwrap() > 50000.0,
        "手入力のない階は自動算定値"
    );
}

// ---- 階定義を正とする割り付け（階と剛床の分離） ----

/// 柱を中間高さで分割した 1 層モデル（各レベル 2 節点、中間に 2 節点）。
fn split_column_model() -> Model {
    let mut model = Model::default();
    let coords = [
        [0.0, 0.0, 0.0],
        [6000.0, 0.0, 0.0],
        [0.0, 0.0, 1750.0],
        [6000.0, 0.0, 1750.0],
        [0.0, 0.0, 3500.0],
        [6000.0, 0.0, 3500.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if i < 2 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.stories.push(Story {
        id: StoryId(0),
        name: "2FL".into(),
        elevation: 3500.0,
        node_ids: Vec::new(),
        seismic_weight: None,
        weight_override: None,
        structure: Default::default(),
        level_kind: Default::default(),
    });
    model
}

/// 中間高さの節点は階には属する（重量が階へ算入される）が、剛床のスレーブには
/// ならない。面内剛体として拘束してよいのは同一床面の節点だけであるため。
#[test]
fn test_mid_height_nodes_join_story_but_not_diaphragm() {
    let model = split_column_model();
    let gen = generate_stories(&model, None).unwrap();

    assert_eq!(gen.stories.len(), 2, "基部の床 + 上の床");
    // 中間 2 節点 + 床面 2 節点が上の床に属する（基部 2 節点は基部の床）。
    assert_eq!(gen.stories[1].node_ids.len(), 4);
    assert_eq!(
        gen.node_story[0],
        Some(StoryId(0)),
        "基部は基部の床に属する"
    );
    assert_eq!(gen.node_story[2], Some(StoryId(1)), "中間節点も階に属する");
    assert_eq!(gen.node_story[4], Some(StoryId(1)));

    // 剛床のスレーブは床面（z=3500）の 2 節点のみ。
    let slaves = gen_slaves(&gen, StoryId(1));
    assert_eq!(
        slaves,
        vec![NodeId(4), NodeId(5)],
        "スレーブは床面の節点だけ"
    );
}

/// 利用者が定義した階（階名・階レベル）を正として割り付ける。節点の Z を
/// クラスタリングし直して階を作り替えることはしない。
#[test]
fn test_predefined_stories_drive_the_assignment() {
    let mut model = two_story_model();
    // 2 レベル（3500・7000）あるが、階は 7000 の 1 つだけ定義する。
    model.stories.push(Story {
        id: StoryId(0),
        name: "RFL".into(),
        elevation: 7000.0,
        node_ids: Vec::new(),
        seismic_weight: None,
        weight_override: None,
        structure: Default::default(),
        level_kind: Default::default(),
    });

    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    // 定義した階はそのまま使い、階生成は基部の床だけを先頭に補う。
    assert_eq!(gen.stories.len(), 2, "定義した RFL + 補われた基部の床");
    assert_eq!(gen.stories[0].elevation, 0.0, "先頭は基部の床");
    assert_eq!(gen.stories[1].name, "RFL");
    assert_eq!(gen.stories[1].elevation, 7000.0);
    // 基部を除く 4 節点すべてが RFL の区間 (0, 7000] に入る。
    assert_eq!(gen.stories[1].node_ids.len(), 4);
    // 剛床のスレーブは床面（z=7000）の 2 節点のみ。
    assert_eq!(gen_slaves(&gen, StoryId(1)).len(), 2);
}

/// 床面（階のレベル）に節点がない階は剛床を持たない。階そのものは残り、
/// 区間に入る節点は所属するが、面内剛体として拘束する床面がないため
/// 剛床拘束も代表節点も作られない。
#[test]
fn test_story_without_floor_nodes_gets_no_diaphragm() {
    let mut model = two_story_model();
    model.stories.push(Story {
        id: StoryId(0),
        name: "3F".into(),
        elevation: 3500.0,
        node_ids: Vec::new(),
        seismic_weight: None,
        weight_override: None,
        structure: Default::default(),
        level_kind: Default::default(),
    });
    // レベル 10500 には節点がない（区間 (3500, 10500] には z=7000 の節点が入る）。
    model.stories.push(Story {
        id: StoryId(1),
        name: "4F".into(),
        elevation: 10500.0,
        node_ids: Vec::new(),
        seismic_weight: None,
        weight_override: None,
        structure: Default::default(),
        level_kind: Default::default(),
    });

    let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
    assert_eq!(gen.stories.len(), 3, "定義した 2 階 + 補われた基部の床");
    assert_eq!(gen.stories[2].name, "4F");
    assert_eq!(
        gen.stories[2].node_ids.len(),
        2,
        "区間に入る節点（z=7000）は階に属する"
    );
    assert!(
        gen.stories[2].seismic_weight.unwrap() > 0.0,
        "その重量も階へ算入される"
    );
    // 剛床は床面に節点がある階の分だけ（基部の床 z=0 と 3F の床 z=3500）。
    // 4F（z=10500）には節点がないため作られない。
    assert_eq!(gen.constraints.len(), 2);
    assert!(gen_slaves(&gen, StoryId(2)).is_empty());
    assert_eq!(gen.rep_nodes.len(), 2, "剛床のない階には代表節点も作らない");
}

/// 旧形式（層基準＝基部の階を持たない階列）のモデルを準備計算に通すと、
/// 階生成が基部の床を補って床基準へ揃える。**層の量は新形式と一致する**。
///
/// これがないと `Model::layers` が最下層を見落とし、層間変形角・層せん断力から
/// 静かに 1 層落ちる。移行コードを持たない代わりに、階生成が不変条件を作る
/// ことでこれを防いでいる（`dev_docs/handoff` の申し送り参照）。
#[test]
fn test_layer_quantities_match_between_legacy_and_floor_based_stories() {
    let base_model = two_story_model();

    // 旧形式: 層の上端の床だけを階として持つ（基部の階がない）。
    let mut legacy = base_model.clone();
    legacy.stories = vec![
        Story {
            id: StoryId(0),
            name: "2F".into(),
            elevation: 3500.0,
            node_ids: Vec::new(),
            seismic_weight: None,
            weight_override: None,
            structure: Default::default(),
            level_kind: Default::default(),
        },
        Story {
            id: StoryId(1),
            name: "3F".into(),
            elevation: 7000.0,
            node_ids: Vec::new(),
            seismic_weight: None,
            weight_override: None,
            structure: Default::default(),
            level_kind: Default::default(),
        },
    ];

    // 新形式: 基部の床を先頭に持つ。
    let mut modern = base_model.clone();
    modern.stories = vec![
        Story {
            id: StoryId(0),
            name: "1F".into(),
            elevation: 0.0,
            node_ids: Vec::new(),
            seismic_weight: None,
            weight_override: None,
            structure: Default::default(),
            level_kind: Default::default(),
        },
        legacy.stories[0].clone(),
        legacy.stories[1].clone(),
    ];
    modern.stories[1].id = StoryId(1);
    modern.stories[2].id = StoryId(2);

    let apply = |m: &Model| -> Model {
        let gen = generate_stories(m, Some(LoadCaseId(0))).unwrap();
        let mut out = m.clone();
        out.stories = gen.stories;
        out
    };
    let from_legacy = apply(&legacy);
    let from_modern = apply(&modern);

    // どちらも床基準（先頭が基部）へ揃う。
    assert_eq!(from_legacy.stories.len(), 3);
    assert_eq!(
        from_legacy.stories[0].elevation,
        from_legacy.base_elevation()
    );
    assert_eq!(
        from_modern.stories[0].elevation,
        from_modern.base_elevation()
    );

    // 層の量（名前・階高・重量）が一致する。
    let layers_of = |m: &Model| -> Vec<(String, f64, Option<f64>)> {
        m.layers()
            .into_iter()
            .map(|l| (l.name, l.height, l.weight))
            .collect()
    };
    let a = layers_of(&from_legacy);
    let b = layers_of(&from_modern);
    assert_eq!(a.len(), 2, "3500・7000 の 2 層");
    assert_eq!(a, b, "旧形式・新形式で層の量が一致する");
    // 最下層の階高は基部から数える（旧形式でも 0→3500）。
    assert_eq!(a[0].1, 3500.0);
    assert_eq!(a[1].1, 3500.0);
}
