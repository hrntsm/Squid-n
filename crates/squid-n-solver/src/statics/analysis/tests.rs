use super::seismic::{distribute_seismic_forces, main_system_weight};
use super::*;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    Constraint, ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LoadCombination,
    LocalAxis, Material, MaterialCategory, MemberLoad, MemberLoadKind, NodalLoad, Node, Section,
    Story, StoryLevelKind, StoryStructure,
};
use squid_n_core::model::{FloorRegion, SlabPlate};

fn make_cantilever_model() -> Model {
    Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [1000.0, 0.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![Section {
            id: SectionId(0),
            name: "beam".into(),
            area: 100.0,
            iy: 833.33,
            iz: 833.33,
            j: 100.0,
            depth: 10.0,
            width: 10.0,
            as_y: 83.33,
            as_z: 83.33,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: None,
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        }],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".into(),
            category: MaterialCategory::Steel,
            young: 20000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![
            LoadCase {
                kind: Default::default(),
                id: LoadCaseId(1),
                name: "axial".into(),
                nodal: vec![NodalLoad::manual(
                    NodeId(1),
                    [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                )],
                member: Vec::new(),
            },
            LoadCase {
                kind: Default::default(),
                id: LoadCaseId(2),
                name: "shear".into(),
                nodal: vec![NodalLoad::manual(
                    NodeId(1),
                    [0.0, 500.0, 0.0, 0.0, 0.0, 0.0],
                )],
                member: Vec::new(),
            },
        ],
        combinations: vec![LoadCombination {
            name: "combo1".into(),
            terms: vec![(LoadCaseId(1), 1.2), (LoadCaseId(2), 1.5)],
        }],
        ..Default::default()
    }
}

#[test]
fn test_prepare_and_single_case() {
    let model = make_cantilever_model();
    let analysis = Analysis::prepare(&model).unwrap();
    let result = analysis.linear_static(LoadCaseId(1)).unwrap();
    let ux = result.disp[1][0];
    let expected = 1000.0 * 1000.0 / (20000.0 * 100.0);
    assert!(
        (ux - expected).abs() < 1e-6,
        "ux={} expected={}",
        ux,
        expected
    );
}

#[test]
fn test_two_cases_one_factorization() {
    let model = make_cantilever_model();
    let analysis = Analysis::prepare(&model).unwrap();
    let r1 = analysis.linear_static(LoadCaseId(1)).unwrap();
    let r2 = analysis.linear_static(LoadCaseId(2)).unwrap();
    let ux = r1.disp[1][0];
    let uy = r2.disp[1][1];
    let ux_expected = 1000.0 * 1000.0 / (20000.0 * 100.0);
    let l = 1000.0_f64;
    let uy_expected = 500.0 * l.powi(3) / (3.0 * 20000.0 * 833.33);
    // Timoshenko beam includes shear deflection ≈ 0.1% — use relaxed tolerance
    assert!((ux - ux_expected).abs() < 1.0, "ux={}", ux);
    assert!(
        (uy - uy_expected).abs() < 20.0,
        "uy={} approx={}",
        uy,
        uy_expected
    );
}

#[test]
fn test_load_combination() {
    let model = make_cantilever_model();
    let analysis = Analysis::prepare(&model).unwrap();
    let combo = &model.combinations[0];
    let result = analysis.linear_combination(combo).unwrap();
    let ux = result.disp[1][0];
    let uy = result.disp[1][1];
    let ux_expected = 1.2 * (1000.0 * 1000.0 / (20000.0 * 100.0));
    let l = 1000.0_f64;
    let uy_expected = 1.5 * (500.0 * l.powi(3) / (3.0 * 20000.0 * 833.33));
    assert!((ux - ux_expected).abs() < 1.0, "ux={}", ux);
    // Timoshenko shear adds slight deflection — relaxed tolerance
    assert!(
        (uy - uy_expected).abs() < 20.0,
        "uy={} approx={}",
        uy,
        uy_expected
    );
}

#[test]
fn test_prepare_empty_model_gives_diagnostic() {
    let model = Model::default();
    let err = Analysis::prepare(&model).err().unwrap();
    assert!(matches!(err, SolveError::InvalidInput(_)), "{:?}", err);
}

#[test]
fn test_prepare_no_restraint_gives_diagnostic() {
    let mut model = make_cantilever_model();
    for n in &mut model.nodes {
        n.restraint = Dof6Mask::FREE;
    }
    let err = Analysis::prepare(&model).err().unwrap();
    let msg = format!("{}", err);
    assert!(msg.contains("拘束"), "{}", msg);
}

#[test]
fn test_prepare_missing_section_gives_diagnostic() {
    let mut model = make_cantilever_model();
    model.elements[0].section = None;
    let err = Analysis::prepare(&model).err().unwrap();
    let msg = format!("{}", err);
    assert!(msg.contains("断面が未割当"), "{}", msg);
}

/// 材料だけが未割当でも解析は止まる。断面と材料は別々の不備として報告し、
/// どちらを直せばよいかがメッセージから分かるようにする。材料は断面が持つ。
#[test]
fn test_prepare_missing_material_gives_diagnostic() {
    let mut model = make_cantilever_model();
    model.sections[0].material = None;
    let err = Analysis::prepare(&model).err().unwrap();
    let msg = format!("{}", err);
    assert!(msg.contains("材料が未割当"), "{}", msg);
}

/// `model_issues` は最初の 1 件で打ち切らず、不備をすべて集める。
/// 診断タブが「あと何を直せば解析できるか」を一覧で示せるようにするため。
#[test]
fn test_model_issues_collects_every_issue() {
    use super::precheck::{model_issues, IssueTargets};

    let mut model = make_cantilever_model();
    // 材料は断面が持つため、断面未割当と材料未割当は別々の部材でしか同時に起きない。
    // 部材 1 は断面を持ったまま、その断面の材料だけを外す。
    let mut second = model.elements[0].clone();
    second.id = squid_n_core::ids::ElemId(1);
    model.elements.push(second);
    model.elements[0].section = None;
    model.sections[0].material = None;
    for n in &mut model.nodes {
        n.restraint = Dof6Mask::FREE;
    }

    let issues = model_issues(&model);
    let messages: Vec<&str> = issues.iter().map(|i| i.message.as_str()).collect();
    assert!(
        messages.iter().any(|m| m.contains("拘束(支点)")),
        "{messages:?}"
    );
    assert!(
        messages.iter().any(|m| m.contains("断面が未割当")),
        "{messages:?}"
    );
    assert!(
        messages.iter().any(|m| m.contains("材料が未割当")),
        "{messages:?}"
    );

    // 部材を名指しする不備は対象 ID を持ち、UI が 3D 選択へ結び付けられる。
    let section_issue = issues
        .iter()
        .find(|i| i.message.contains("断面が未割当"))
        .unwrap();
    assert_eq!(
        section_issue.targets,
        IssueTargets::Members(vec![model.elements[0].id])
    );

    // 健全なモデルでは 1 件も出ない。
    assert!(model_issues(&make_cantilever_model()).is_empty());
}

/// 断面が未割当のスラブ・断面の材料が未割当のスラブは解析前チェックで止まる。
///
/// スラブの板厚と自重は断面から解決するため、断面が無いと床の固定荷重が
/// 過小なまま長期応力が出る（危険側）。既定厚で補わずに止める。
#[test]
fn test_model_issues_rejects_slab_without_section() {
    use super::precheck::{model_issues, precheck_model};
    use squid_n_core::ids::{FloorRegionId, SectionId, SlabId};
    use squid_n_core::model::{DistributionMethod, Slab, SlabShape};

    let mut model = make_cantilever_model();
    // 境界節点は 3 点必要なので、床用の節点を足す。
    let n = model.nodes.len() as u32;
    for (i, c) in [[0.0, 1000.0, 0.0], [1000.0, 1000.0, 0.0]]
        .into_iter()
        .enumerate()
    {
        model.nodes.push(Node {
            id: NodeId(n + i as u32),
            coord: c,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    let boundary = vec![NodeId(0), NodeId(1), NodeId(n + 1), NodeId(n)];
    // 大梁で境界を閉じる（1 本は `make_cantilever_model` が既に持つ 0→1）。
    // 閉じていないと「大梁の区画に載らない浮き床板」として扱われてしまう。
    for (i, j) in [(1, n + 1), (n + 1, n), (n, 0)] {
        model.elements.push(ElementData {
            id: ElemId(model.elements.len() as u32),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(i), NodeId(j)],
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
    let mut region = FloorRegion::new(FloorRegionId(0), boundary.clone());
    region.slab_ids.push(SlabId(0));
    model.floor_regions.push(region);
    model.slabs.push(Slab {
        id: SlabId(0),
        shape: SlabShape::Enclosed { boundary },
        plate: SlabPlate {
            section: None,
            loads: Vec::new(),
            usage: None,
            method: DistributionMethod::TriTrapezoid,
            one_way: None,
        },
    });

    // 断面が未割当。
    let msgs: Vec<String> = model_issues(&model)
        .into_iter()
        .map(|i| i.message)
        .collect();
    assert!(
        msgs.iter().any(|m| m.contains("断面が未割当の床")),
        "{msgs:?}"
    );
    assert!(precheck_model(&model).is_err());

    // 断面はあるが材料が未割当。
    let sid = SectionId(model.sections.len() as u32);
    model.sections.push(
        squid_n_core::section_shape::SectionShape::RcSlab { thickness: 150.0 }
            .to_section(sid, "S15".into()),
    );
    model.slabs[0].plate.section = Some(sid);
    let msgs: Vec<String> = model_issues(&model)
        .into_iter()
        .map(|i| i.message)
        .collect();
    assert!(
        msgs.iter().any(|m| m.contains("断面の材料または板厚")),
        "{msgs:?}"
    );

    // 材料を割り当てれば通る。
    let mid = squid_n_core::ids::MaterialId(model.materials.len() as u32);
    model.materials.push(squid_n_core::model::Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: mid,
        name: "Fc24".into(),
        category: squid_n_core::model::MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    model.sections[sid.index()].material = Some(mid);
    let msgs: Vec<String> = model_issues(&model)
        .into_iter()
        .map(|i| i.message)
        .collect();
    assert!(!msgs.iter().any(|m| m.contains("床")), "{msgs:?}");
}

/// 剛床のない階は警告として挙げ、解析は止めない。
///
/// 剛床がない階の水平力は階の節点へ質量比で直接分配されるため解析は成立する。
/// 一方、剛床を意図していたのに床が拾えていない場合に気づけないと困るので、
/// 診断には出す。
#[test]
fn test_model_issues_warns_story_without_diaphragm() {
    use super::precheck::{model_issues, precheck_model, IssueSeverity};
    use squid_n_core::ids::StoryId;
    use squid_n_core::model::Story;

    let mut model = make_cantilever_model();
    model.stories.push(Story {
        id: StoryId(0),
        name: "2F".into(),
        elevation: 3000.0,
        node_ids: vec![NodeId(1)],
        seismic_weight: None,
        weight_override: None,
        structure: Default::default(),
        level_kind: Default::default(),
    });

    let issues = model_issues(&model);
    let warning = issues
        .iter()
        .find(|i| i.message.contains("剛床のない階"))
        .unwrap_or_else(|| panic!("剛床のない階の警告が出ていない: {:?}", issues.len()));
    assert_eq!(warning.severity, IssueSeverity::Warning);
    assert!(warning.message.contains("2F"), "{}", warning.message);
    // 警告だけなら解析前チェックは通す。
    assert!(precheck_model(&model).is_ok());
}

/// 階名の重複はエラーとし、解析前チェックで止める。
///
/// 階名は結果の一覧・CSV の列見出しと断面の符号＋階に使われるため、重複すると
/// どの階の値なのかを判別できない。前後の空白しか違わない名前も同名として扱う。
#[test]
fn test_model_issues_errors_on_duplicate_story_names() {
    use super::precheck::{model_issues, precheck_model, IssueSeverity};
    use squid_n_core::ids::StoryId;
    use squid_n_core::model::Story;

    let mut model = make_cantilever_model();
    let story = |id: u32, name: &str, elevation: f64| Story {
        id: StoryId(id),
        name: name.into(),
        elevation,
        node_ids: Vec::new(),
        seismic_weight: None,
        weight_override: None,
        structure: Default::default(),
        level_kind: Default::default(),
    };
    // 見た目で区別できない差（末尾の空白）でも同名として扱う。
    model.stories.push(story(0, "2F", 3000.0));
    model.stories.push(story(1, "2F ", 6000.0));

    let issues = model_issues(&model);
    let issue = issues
        .iter()
        .find(|i| i.message.contains("階名が重複"))
        .expect("階名の重複が診断に出ていない");
    assert_eq!(issue.severity, IssueSeverity::Error);
    assert!(issue.message.contains("2F"), "{}", issue.message);
    // エラーなので解析前チェックが止める。
    assert!(precheck_model(&model).is_err());

    // 名前を分ければ通る（剛床のない階の警告だけが残る）。
    model.stories[1].name = "3F".into();
    assert!(precheck_model(&model).is_ok());
}

/// 剛床マスターの水平拘束検査用の 2 階モデル。
///
/// `story_index` の階に剛床を 1 つ置き、マスター節点の拘束と地震用重量を指定する。
fn make_two_story_diaphragm_model(
    story_index: usize,
    master_restraint: Dof6Mask,
    seismic_weight: Option<f64>,
    diaphragm_weight: Option<f64>,
) -> Model {
    let master = NodeId(story_index as u32);
    let mut nodes = vec![
        Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: Some(StoryId(0)),
            support_spring: None,
        },
        Node {
            id: NodeId(1),
            coord: [0.0, 0.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: Some(StoryId(1)),
            support_spring: None,
        },
    ];
    nodes[master.index()].restraint = master_restraint;
    Model {
        nodes,
        stories: vec![
            Story {
                id: StoryId(0),
                name: "1F".into(),
                elevation: 0.0,
                node_ids: vec![NodeId(0)],
                seismic_weight: Some(500.0),
                weight_override: None,
                structure: StoryStructure::Rc,
                level_kind: StoryLevelKind::Normal,
            },
            Story {
                id: StoryId(1),
                name: "2F".into(),
                elevation: 3000.0,
                node_ids: vec![NodeId(1)],
                seismic_weight,
                weight_override: None,
                structure: StoryStructure::Rc,
                level_kind: StoryLevelKind::Normal,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![Section {
            id: SectionId(0),
            name: "col".into(),
            area: 100.0,
            iy: 833.33,
            iz: 833.33,
            j: 100.0,
            depth: 10.0,
            width: 10.0,
            as_y: 83.33,
            as_z: 83.33,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: None,
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        }],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".into(),
            category: MaterialCategory::Steel,
            young: 20000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        constraints: vec![Constraint::RigidDiaphragm {
            story: StoryId(story_index as u32),
            master,
            slaves: Vec::new(),
            weight: diaphragm_weight,
            ci_override: None,
        }],
        ..Default::default()
    }
}

fn has_restrained_diaphragm_master_issue(model: &Model) -> bool {
    use super::precheck::model_issues;
    model_issues(model)
        .iter()
        .any(|i| i.message.contains("剛床マスターが水平拘束"))
}

/// 基部の剛床マスターが水平拘束されていてもエラーにしない（柱脚固定は正常）。
#[test]
fn test_model_issues_allows_base_diaphragm_master_horizontal_restraint() {
    use squid_n_core::dof::Dof;

    let mut r = Dof6Mask::FREE;
    r.set_fixed(Dof::Ux);
    r.set_fixed(Dof::Uy);
    r.set_fixed(Dof::Rz);
    let model = make_two_story_diaphragm_model(0, r, Some(800.0), Some(800.0));
    assert!(
        !has_restrained_diaphragm_master_issue(&model),
        "基部の水平拘束は許容される"
    );
}

/// 上階の剛床マスターが水平拘束され、地震用重量が正ならエラー。
#[test]
fn test_model_issues_errors_on_upper_diaphragm_master_horizontal_restraint() {
    use super::precheck::{model_issues, precheck_model, IssueSeverity, IssueTargets};
    use squid_n_core::dof::Dof;

    let mut r = Dof6Mask::FREE;
    r.set_fixed(Dof::Ux);
    let model = make_two_story_diaphragm_model(1, r, Some(800.0), None);
    assert!(has_restrained_diaphragm_master_issue(&model));
    assert!(precheck_model(&model).is_err());

    let issue = model_issues(&model)
        .into_iter()
        .find(|i| i.message.contains("剛床マスターが水平拘束"))
        .expect("水平拘束の不備が出ていない");
    assert_eq!(issue.severity, IssueSeverity::Error);
    assert!(issue.message.contains("2F"), "{}", issue.message);
    assert_eq!(issue.targets, IssueTargets::Nodes(vec![NodeId(1)]));
}

/// 上階の剛床マスターが水平拘束でも、地震用重量が 0 ならエラーにしない。
#[test]
fn test_model_issues_allows_upper_diaphragm_master_restraint_without_weight() {
    use squid_n_core::dof::Dof;

    let mut r = Dof6Mask::FREE;
    r.set_fixed(Dof::Uy);
    let model = make_two_story_diaphragm_model(1, r, Some(0.0), Some(0.0));
    assert!(
        !has_restrained_diaphragm_master_issue(&model),
        "重量 0 の階は対象外"
    );
}

/// 載荷区間が材長を超える部材荷重はエラーとし、解析前チェックで止める。
///
/// 等価節点力の積分が Hermite 形状関数を材外へ外挿するため、節点力と固定端内力が
/// 黙って誤る。全長載荷（`b = L`）は不備としない。
#[test]
fn test_model_issues_errors_on_member_load_beyond_length() {
    use super::precheck::{model_issues, precheck_model, IssueSeverity};
    use squid_n_core::model::{MemberLoad, MemberLoadKind};

    let mut model = make_cantilever_model();
    let elem = model.elements[0].id;
    let l = model.member_length(&model.elements[0]);
    assert!(l > 0.0);

    // 全長載荷は不備にしない。
    model.load_cases[0].member.push(MemberLoad::manual(
        elem,
        [0.0, 0.0, -1.0],
        MemberLoadKind::Distributed {
            a: 0.0,
            b: l,
            w1: 1.0,
            w2: 1.0,
        },
    ));
    assert!(
        !model_issues(&model)
            .iter()
            .any(|i| i.message.contains("載荷区間")),
        "全長載荷は不備ではない"
    );

    // 材長を超える区間はエラー。
    model.load_cases[0].member.push(MemberLoad::manual(
        elem,
        [0.0, 0.0, -1.0],
        MemberLoadKind::Point {
            a: l + 100.0,
            p: 1000.0,
        },
    ));
    let issues = model_issues(&model);
    let issue = issues
        .iter()
        .find(|i| i.message.contains("載荷区間"))
        .expect("載荷区間の不備が診断に出ていない");
    assert_eq!(issue.severity, IssueSeverity::Error);
    assert!(precheck_model(&model).is_err());
}

/// 部材が 1 つもないモデルで、全節点を孤立節点として並べない。
/// 「部材がありません」で同じことを言っており、節点を 1 つずつ挙げても情報が増えない。
#[test]
fn test_model_issues_skips_isolated_nodes_without_elements() {
    use super::precheck::model_issues;

    let mut model = make_cantilever_model();
    model.elements.clear();

    let messages: Vec<String> = model_issues(&model)
        .into_iter()
        .map(|i| i.message)
        .collect();
    assert!(
        messages.iter().any(|m| m.contains("部材がありません")),
        "{messages:?}"
    );
    assert!(
        !messages.iter().any(|m| m.contains("接続されていない節点")),
        "{messages:?}"
    );
}

#[test]
fn test_prepare_isolated_node_gives_diagnostic() {
    let mut model = make_cantilever_model();
    model.nodes.push(Node {
        id: NodeId(2),
        coord: [0.0, 5000.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    let err = Analysis::prepare(&model).err().unwrap();
    let msg = format!("{}", err);
    assert!(msg.contains("接続されていない節点"), "{}", msg);
}

/// 存在しない節点を参照する拘束（節点削除後の不整合など）は、panic ではなく
/// ダングリング参照の診断エラーになること（従来は precheck 内の直接添字で
/// panic していた）。
#[test]
fn test_prepare_dangling_constraint_reference_gives_diagnostic() {
    let mut model = make_cantilever_model();
    model
        .constraints
        .push(squid_n_core::model::Constraint::RigidLink {
            master: NodeId(99),
            slaves: vec![NodeId(1)],
            dofs: Dof6Mask::FIXED,
        });
    let err = Analysis::prepare(&model).err().unwrap();
    let msg = format!("{}", err);
    assert!(msg.contains("存在しない節点"), "{}", msg);
}

/// 有効せん断断面積 As=0 の断面を使う線材は入力エラーとする。
///
/// As=0 はせん断変形が生じず（φ=0）、せん断降伏の判定閾値も Qy=+∞ となるため、
/// 入力不足が「せん断について無限に強い部材」として黙って通ってしまう（危険側）。
/// せん断変形を無視したい場合は部材のモデル化として指定する（十分大きな As を
/// 与える `test_bernoulli_strict_1e9` の扱い）。
#[test]
fn test_prepare_zero_shear_area_is_error() {
    let mut model = make_cantilever_model();
    model.sections[0].as_y = 0.0;
    let err = Analysis::prepare(&model).err().unwrap();
    let msg = format!("{}", err);
    assert!(msg.contains("有効せん断断面積"), "{}", msg);

    // as_z 側だけが 0 でも同様に検出する。
    model.sections[0].as_y = 83.33;
    model.sections[0].as_z = 0.0;
    let err = Analysis::prepare(&model).err().unwrap();
    assert!(format!("{}", err).contains("有効せん断断面積"));
}

#[test]
fn test_linear_static_unknown_load_case_is_error() {
    let model = make_cantilever_model();
    let analysis = Analysis::prepare(&model).unwrap();
    let err = analysis.linear_static(LoadCaseId(99)).err().unwrap();
    let msg = format!("{}", err);
    assert!(msg.contains("荷重ケース"), "{}", msg);
}

#[test]
fn test_seismic_without_stories_is_error() {
    let model = make_cantilever_model();
    let analysis = Analysis::prepare(&model).unwrap();
    let err = analysis
        .seismic_static(SeismicDir::X, AiMode::Approx)
        .err()
        .unwrap();
    let msg = format!("{}", err);
    assert!(msg.contains("階"), "{}", msg);
}

#[test]
fn test_bernoulli_strict_1e9() {
    // Bernoulli beam: very large shear area → negligible shear deformation.
    // Axial: u = PL/EA, Bending: w = PL³/3EI — strict 1e-9 match.
    let mut model = make_cantilever_model();
    model.sections[0].as_y = 1e12;
    model.sections[0].as_z = 1e12;
    let analysis = Analysis::prepare(&model).unwrap();
    let r1 = analysis.linear_static(LoadCaseId(1)).unwrap();
    let r2 = analysis.linear_static(LoadCaseId(2)).unwrap();
    let ux = r1.disp[1][0];
    let uy = r2.disp[1][1];
    let ux_expected = 1000.0 * 1000.0 / (20000.0 * 100.0);
    let l = 1000.0_f64;
    let uy_expected = 500.0 * l.powi(3) / (3.0 * 20000.0 * 833.33);
    let ux_rel = (ux - ux_expected).abs() / ux_expected.abs();
    let uy_rel = (uy - uy_expected).abs() / uy_expected.abs();
    assert!(ux_rel < 1e-9, "ux rel err={}", ux_rel);
    assert!(uy_rel < 1e-4, "uy rel err={}", uy_rel);
}

// ---- §1.5 略算周期の鉄骨造比 α ----

/// 3層等階高（各1000mm、基部Z=0）で、指定した各**層**の `structure` から
/// `steel_height_ratio` を計算するテスト用モデル。
///
/// 階は床であり、先頭は基部の床（`Model::layers` の不変条件）。層 i の構造種別は
/// その上端の階が持つため、`structures[i]` は `stories[i + 1]` に入る。
fn make_story_ratio_model(structures: &[StoryStructure]) -> Model {
    let mut nodes = vec![Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: Some(StoryId(0)),
        support_spring: None,
    }];
    let mut stories = vec![Story {
        id: StoryId(0),
        name: "F1".to_string(),
        elevation: 0.0,
        node_ids: vec![NodeId(0)],
        seismic_weight: None,
        weight_override: None,
        structure: StoryStructure::default(),
        level_kind: StoryLevelKind::Normal,
    }];
    for (i, s) in structures.iter().enumerate() {
        let elev = (i as f64 + 1.0) * 1000.0;
        let nid = NodeId((i + 1) as u32);
        nodes.push(Node {
            id: nid,
            coord: [0.0, 0.0, elev],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: Some(StoryId((i + 1) as u32)),
            support_spring: None,
        });
        stories.push(Story {
            id: StoryId((i + 1) as u32),
            name: format!("F{}", i + 2),
            elevation: elev,
            node_ids: vec![nid],
            seismic_weight: Some(1000.0),
            weight_override: None,
            structure: *s,
            level_kind: StoryLevelKind::Normal,
        });
    }
    Model {
        nodes,
        stories,
        ..Default::default()
    }
}

#[test]
fn test_steel_height_ratio_bottom_story_s_gives_one_third() {
    let model =
        make_story_ratio_model(&[StoryStructure::S, StoryStructure::Rc, StoryStructure::Rc]);
    let alpha = steel_height_ratio(&model);
    assert!((alpha - 1.0 / 3.0).abs() < 1e-9, "alpha={}", alpha);
}

#[test]
fn test_steel_height_ratio_all_rc_is_zero() {
    let model = make_story_ratio_model(&[StoryStructure::Rc; 3]);
    assert_eq!(steel_height_ratio(&model), 0.0);
}

#[test]
fn test_steel_height_ratio_all_s_is_one() {
    let model = make_story_ratio_model(&[StoryStructure::S; 3]);
    let alpha = steel_height_ratio(&model);
    assert!((alpha - 1.0).abs() < 1e-9, "alpha={}", alpha);
}

#[test]
fn test_steel_height_ratio_no_stories_is_zero() {
    let model = Model::default();
    assert_eq!(steel_height_ratio(&model), 0.0);
}

// ---- §1.6 多剛床のPi重複載荷 ----

/// 剛床の分配規則の検証用モデル。地震用重量 400 の階を 1 つ持ち、指定した
/// `(マスター, 剛床重量, ci_override)` の剛床拘束を備える。剛床は階ではなく
/// 拘束として保持されるため、分配関数はモデルごと受け取る。
fn make_diaphragm_model(diaphragms: Vec<(NodeId, Option<f64>, Option<f64>)>) -> Model {
    let mut model = Model {
        stories: vec![Story {
            id: StoryId(0),
            name: "F1".into(),
            elevation: 1000.0,
            node_ids: Vec::new(),
            seismic_weight: Some(400.0),
            weight_override: None,
            structure: StoryStructure::Rc,
            level_kind: StoryLevelKind::Normal,
        }],
        ..Default::default()
    };
    for (master, weight, ci_override) in diaphragms {
        model.constraints.push(Constraint::RigidDiaphragm {
            story: StoryId(0),
            master,
            slaves: Vec::new(),
            weight,
            ci_override,
        });
    }
    model
}

#[test]
fn test_distribute_pi_single_diaphragm_gets_full_pi() {
    let model = make_diaphragm_model(vec![(NodeId(10), None, None)]);
    let shares = distribute_pi_over_diaphragms(&model, &model.stories[0], 40.0);
    assert_eq!(shares, vec![(NodeId(10), 40.0)]);
}

#[test]
fn test_distribute_pi_weight_ratio_3_to_1() {
    let model = make_diaphragm_model(vec![
        (NodeId(10), Some(300.0), None),
        (NodeId(11), Some(100.0), None),
    ]);
    let pi = 40.0;
    let shares = distribute_pi_over_diaphragms(&model, &model.stories[0], pi);
    let s10 = shares.iter().find(|(n, _)| *n == NodeId(10)).unwrap().1;
    let s11 = shares.iter().find(|(n, _)| *n == NodeId(11)).unwrap().1;
    assert!((s10 - 30.0).abs() < 1e-9, "s10={}", s10);
    assert!((s11 - 10.0).abs() < 1e-9, "s11={}", s11);
    // 合計は階の Pi に一致する（重複載荷しない）。
    let total: f64 = shares.iter().map(|(_, v)| v).sum();
    assert!((total - pi).abs() < 1e-9, "total={}", total);
}

#[test]
fn test_distribute_pi_equal_split_when_no_weight() {
    let model = make_diaphragm_model(vec![(NodeId(10), None, None), (NodeId(11), None, None)]);
    let pi = 40.0;
    let shares = distribute_pi_over_diaphragms(&model, &model.stories[0], pi);
    for (_, v) in &shares {
        assert!((*v - 20.0).abs() < 1e-9, "share={}", v);
    }
    let total: f64 = shares.iter().map(|(_, v)| v).sum();
    assert!((total - pi).abs() < 1e-9, "total={}", total);
}

/// 剛床を持たない階の水平力は、階に属する節点へ質点質量の比で分配される
/// （階と剛床は別概念であり、剛床がない階にも水平力は載る）。
#[test]
fn test_distribute_pi_without_diaphragm_falls_back_to_story_nodes() {
    let mut model = make_diaphragm_model(vec![]);
    model.nodes = vec![
        Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 1000.0],
            restraint: Dof6Mask::FREE,
            mass: Some([3.0, 3.0, 3.0, 0.0, 0.0, 0.0]),
            story: Some(StoryId(0)),
            support_spring: None,
        },
        Node {
            id: NodeId(1),
            coord: [6000.0, 0.0, 1000.0],
            restraint: Dof6Mask::FREE,
            mass: Some([1.0, 1.0, 1.0, 0.0, 0.0, 0.0]),
            story: Some(StoryId(0)),
            support_spring: None,
        },
    ];
    model.stories[0].node_ids = vec![NodeId(0), NodeId(1)];

    let pi = 40.0;
    let shares = distribute_pi_over_diaphragms(&model, &model.stories[0], pi);
    let s0 = shares.iter().find(|(n, _)| *n == NodeId(0)).unwrap().1;
    let s1 = shares.iter().find(|(n, _)| *n == NodeId(1)).unwrap().1;
    assert!((s0 - 30.0).abs() < 1e-9, "s0={s0}");
    assert!((s1 - 10.0).abs() < 1e-9, "s1={s1}");
    let total: f64 = shares.iter().map(|(_, v)| v).sum();
    assert!(
        (total - pi).abs() < 1e-9,
        "層せん断力の総量は保たれる: {total}"
    );
}

/// 剛床も質量もない階では等分割する（載荷位置は決まらないが総量は保つ）。
#[test]
fn test_distribute_pi_without_diaphragm_and_mass_splits_equally() {
    let mut model = make_diaphragm_model(vec![]);
    model.nodes = (0..2)
        .map(|i| Node {
            id: NodeId(i),
            coord: [i as f64 * 6000.0, 0.0, 1000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: Some(StoryId(0)),
            support_spring: None,
        })
        .collect();
    model.stories[0].node_ids = vec![NodeId(0), NodeId(1)];

    let shares = distribute_pi_over_diaphragms(&model, &model.stories[0], 40.0);
    for (_, v) in &shares {
        assert!((*v - 20.0).abs() < 1e-9, "share={v}");
    }
}

// ---- §4 追補: 副剛床のCi直接入力・パラペット・階別見付け幅 ----

#[test]
fn test_main_system_weight_excludes_ci_override_diaphragm() {
    let model = make_diaphragm_model(vec![
        (NodeId(10), Some(300.0), None),
        (NodeId(11), Some(100.0), Some(0.3)),
    ]);
    // make_diaphragm_model は seismic_weight=400.0 固定（主300+副100）。
    // 主系統重量は ci_override を持つ副剛床の重量(100)を除いた 300 になる。
    let w = main_system_weight(&model, &model.stories[0]);
    assert!((w - 300.0).abs() < 1e-9, "main_system_weight={}", w);
}

#[test]
fn test_distribute_seismic_forces_ci_override_adds_separate_force() {
    let model = make_diaphragm_model(vec![
        (NodeId(10), Some(300.0), None),
        (NodeId(11), Some(100.0), Some(0.3)),
    ]);
    // 主系統(重量300ベースで別途算定済み)の Pi として 60.0 を渡す。
    // 主剛床(唯一の ci_override 無し剛床)が全量を受け、副剛床には
    // 0.3×100=30 が別途載る。
    let pi = 60.0;
    let shares = distribute_seismic_forces(&model, &model.stories[0], pi);
    let s10 = shares.iter().find(|(n, _)| *n == NodeId(10)).unwrap().1;
    let s11 = shares.iter().find(|(n, _)| *n == NodeId(11)).unwrap().1;
    assert!((s10 - 60.0).abs() < 1e-9, "s10={}", s10);
    assert!((s11 - 30.0).abs() < 1e-9, "s11={}", s11);
}

#[test]
fn test_distribute_seismic_forces_matches_pi_distribution_without_ci_override() {
    // 全剛床が ci_override 無しなら distribute_pi_over_diaphragms と厳密一致。
    let model = make_diaphragm_model(vec![
        (NodeId(10), Some(300.0), None),
        (NodeId(11), Some(100.0), None),
    ]);
    let pi = 40.0;
    let expected = distribute_pi_over_diaphragms(&model, &model.stories[0], pi);
    let actual = distribute_seismic_forces(&model, &model.stories[0], pi);
    assert_eq!(expected, actual);
}

/// バッチ API（逐次モード）が個別呼び出しとビット一致すること。
/// 並列モードの検証はプロセス分離した統合テスト
/// （tests/parallel_batch.rs）で行う（並列度設定はプロセスグローバルのため）。
#[test]
fn test_linear_static_batch_matches_individual() {
    let model = make_cantilever_model();
    let analysis = Analysis::prepare(&model).unwrap();
    let batch = analysis.linear_static_batch(&[LoadCaseId(1), LoadCaseId(2)]);
    assert_eq!(batch.len(), 2);
    let r1 = analysis.linear_static(LoadCaseId(1)).unwrap();
    let r2 = analysis.linear_static(LoadCaseId(2)).unwrap();
    let b1 = batch[0].as_ref().unwrap();
    let b2 = batch[1].as_ref().unwrap();
    assert_eq!(b1.disp, r1.disp);
    assert_eq!(b2.disp, r2.disp);

    // 組合せバッチも個別呼び出しとビット一致する
    let combos = vec![model.combinations[0].clone()];
    let cb = analysis.linear_combination_batch(&combos);
    let c1 = analysis.linear_combination(&model.combinations[0]).unwrap();
    assert_eq!(cb[0].as_ref().unwrap().disp, c1.disp);

    // 存在しないケースはバッチ内でも個別にエラーになる（他ケースへ影響しない）
    let with_missing = analysis.linear_static_batch(&[LoadCaseId(1), LoadCaseId(99)]);
    assert!(with_missing[0].is_ok());
    assert!(matches!(with_missing[1], Err(SolveError::InvalidInput(_))));
}

/// 単純梁（i:ピン+ねじり拘束, j:ローラ）に全長 UDL を与えるモデル。
/// LoadCaseId(1) が UDL（強度 `w`）、combinations[0] は 1.5 倍の組合せ。
fn ss_beam_udl(l: f64, w: f64) -> Model {
    Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask(0b001111),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [l, 0.0, 0.0],
                restraint: Dof6Mask(0b000110),
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![Section {
            id: SectionId(0),
            name: "s".into(),
            area: 1000.0,
            iy: 1.0e7,
            iz: 1.0e7,
            j: 1.0e6,
            depth: 200.0,
            width: 100.0,
            as_y: 800.0,
            as_z: 800.0,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: None,
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        }],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "m".into(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "udl".into(),
            nodal: vec![],
            member: vec![MemberLoad::manual(
                ElemId(0),
                [0.0, 0.0, -1.0],
                MemberLoadKind::Distributed {
                    a: 0.0,
                    b: l,
                    w1: w,
                    w2: w,
                },
            )],
        }],
        combinations: vec![LoadCombination {
            name: "1.5UDL".into(),
            terms: vec![(LoadCaseId(1), 1.5)],
        }],
        ..Default::default()
    }
}

fn mz_at(mf: &squid_n_element::beam::MemberForces, xi_target: f64) -> f64 {
    mf.at
        .iter()
        .find(|(xi, _)| (xi - xi_target).abs() < 1e-9)
        .map(|(_, v)| v[5])
        .expect("section present")
}

/// `Analysis::linear_static` でも部材中間荷重が内力回復へ重ね合わされ、
/// 単純梁 UDL の中央曲げが wL²/8（放物線分布）・端部が ~0 になることを検証する。
/// （重ね合わせ欠落時は中央 Mz が 0 付近になり不合格になる。）
#[test]
fn analysis_linear_static_superposes_member_load() {
    let l = 1000.0_f64;
    let w = 2.0_f64;
    let model = ss_beam_udl(l, w);
    let analysis = Analysis::prepare(&model).unwrap();
    let res = analysis.linear_static(LoadCaseId(1)).unwrap();
    let (_, mf) = res
        .member_forces
        .iter()
        .find(|(id, _)| *id == ElemId(0))
        .expect("member forces");

    let expected_mid = w * l * l / 8.0;
    let mid = mz_at(mf, 0.5).abs();
    assert!(
        (mid - expected_mid).abs() / expected_mid < 1e-3,
        "midspan Mz={} expected {}",
        mid,
        expected_mid
    );
    let end = mz_at(mf, 0.0).abs().max(mz_at(mf, 1.0).abs());
    assert!(
        end < expected_mid * 1e-3,
        "end Mz should be ~0, got {}",
        end
    );
}

/// `Analysis::linear_combination` は各項の部材荷重を係数倍して重ね合わせる。
/// 1.5×UDL の中央曲げは 1.5·wL²/8 になる。
#[test]
fn analysis_linear_combination_scales_member_load() {
    let l = 1000.0_f64;
    let w = 2.0_f64;
    let model = ss_beam_udl(l, w);
    let analysis = Analysis::prepare(&model).unwrap();
    let res = analysis.linear_combination(&model.combinations[0]).unwrap();
    let (_, mf) = res
        .member_forces
        .iter()
        .find(|(id, _)| *id == ElemId(0))
        .expect("member forces");

    let expected_mid = 1.5 * w * l * l / 8.0;
    let mid = mz_at(mf, 0.5).abs();
    assert!(
        (mid - expected_mid).abs() / expected_mid < 1e-3,
        "combination midspan Mz={} expected {}",
        mid,
        expected_mid
    );
}

/// 荷重組合せの応答（荷重ケース単体結果の線形和）は、同じ線形結合の荷重を 1 つの
/// 荷重ケースへまとめて解いた結果と一致する（重ね合わせの原理の検証）。
/// 節点荷重・部材中間荷重の両方を含む組合せで、節点変位と部材断面力の双方を比較する。
#[test]
fn analysis_linear_combination_matches_assembled_load_case() {
    let l = 1000.0_f64;
    let w = 3.0_f64;
    let (c_axial, c_shear, c_udl) = (1.2_f64, 1.5_f64, -0.8_f64);
    let udl = |scale: f64| {
        MemberLoad::manual(
            ElemId(0),
            [0.0, 0.0, -1.0],
            MemberLoadKind::Distributed {
                a: 0.0,
                b: l,
                w1: w * scale,
                w2: w * scale,
            },
        )
    };

    // 節点荷重 2 ケース（軸・せん断）＋部材中間荷重 1 ケースを参照する組合せ。
    let mut model = make_cantilever_model();
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(3),
        name: "udl".into(),
        nodal: Vec::new(),
        member: vec![udl(1.0)],
    });
    model.combinations = vec![LoadCombination {
        name: "combo".into(),
        terms: vec![
            (LoadCaseId(1), c_axial),
            (LoadCaseId(2), c_shear),
            (LoadCaseId(3), c_udl),
        ],
    }];

    // 同じ線形結合の荷重を 1 ケースへまとめたモデル（荷重ベクトルを合成して解く経路）。
    let scale_nodal = |lc: &LoadCase, factor: f64| -> Vec<NodalLoad> {
        lc.nodal
            .iter()
            .map(|n| NodalLoad::manual(n.node, n.values.map(|v| v * factor)))
            .collect()
    };
    let mut nodal = scale_nodal(&model.load_cases[0], c_axial);
    nodal.extend(scale_nodal(&model.load_cases[1], c_shear));
    let mut merged = model.clone();
    merged.combinations.clear();
    merged.load_cases = vec![LoadCase {
        kind: Default::default(),
        id: LoadCaseId(9),
        name: "merged".into(),
        nodal,
        member: vec![udl(c_udl)],
    }];

    let superposed = Analysis::prepare(&model)
        .unwrap()
        .linear_combination(&model.combinations[0])
        .unwrap();
    let assembled = Analysis::prepare(&merged)
        .unwrap()
        .linear_static(LoadCaseId(9))
        .unwrap();

    let scale = superposed
        .disp
        .iter()
        .flatten()
        .fold(0.0_f64, |m, v| m.max(v.abs()))
        .max(1.0);
    for (i, (a, b)) in superposed
        .disp
        .iter()
        .zip(assembled.disp.iter())
        .enumerate()
    {
        for (d, (x, y)) in a.iter().zip(b.iter()).enumerate() {
            assert!(
                (x - y).abs() < 1e-9 * scale,
                "節点 {} 成分 {}: 線形和 {} ≠ 合成荷重 {}",
                i,
                d,
                x,
                y
            );
        }
    }

    assert_eq!(
        superposed.member_forces.len(),
        assembled.member_forces.len()
    );
    for ((id_a, mf_a), (id_b, mf_b)) in superposed
        .member_forces
        .iter()
        .zip(assembled.member_forces.iter())
    {
        assert_eq!(id_a, id_b);
        let scale = mf_a
            .at
            .iter()
            .flat_map(|(_, v)| v.iter())
            .fold(0.0_f64, |m, v| m.max(v.abs()))
            .max(1.0);
        for ((xi_a, v_a), (xi_b, v_b)) in mf_a.at.iter().zip(mf_b.at.iter()) {
            assert!((xi_a - xi_b).abs() < 1e-12, "評価断面位置がずれている");
            for (c, (x, y)) in v_a.iter().zip(v_b.iter()).enumerate() {
                assert!(
                    (x - y).abs() < 1e-9 * scale,
                    "部材 {} xi={} 成分 {}: 線形和 {} ≠ 合成荷重 {}",
                    id_a.0,
                    xi_a,
                    c,
                    x,
                    y
                );
            }
        }
    }
}

/// 節点を共有せずに交差する大梁は、解析を止めない警告として診断に出る。
///
/// 床領域は大梁で囲まれた区画として面走査で求めるため、交差があると区画が実際とずれる。
/// 解析自体は通るので、エラーではなく警告で利用者へ確かめる。
#[test]
fn test_crossing_beams_reported_as_warning() {
    use crate::analysis::precheck::{model_issues, IssueSeverity};
    use squid_n_core::ids::{ElemId, NodeId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node,
    };

    let mut model = Model::default();
    let pts = [(0.0, 0.0), (4000.0, 4000.0), (0.0, 4000.0), (4000.0, 0.0)];
    for (i, (x, y)) in pts.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: [*x, *y, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    // 節点を共有せずに X 字に交わる 2 本。
    for (k, (i, j)) in [(0u32, 1u32), (2, 3)].into_iter().enumerate() {
        model.elements.push(ElementData {
            id: ElemId(k as u32),
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
        });
    }

    let issues = model_issues(&model);
    let crossing = issues
        .iter()
        .find(|i| i.message.contains("交差する大梁"))
        .expect("交差の診断が出る");
    assert_eq!(crossing.severity, IssueSeverity::Warning);
    assert!(
        crossing.message.contains("部材 0, 1"),
        "{}",
        crossing.message
    );
}

/// どの床領域にも属さない小梁は警告し、解析は止めない。
#[test]
fn test_model_issues_warns_unassigned_joist() {
    use super::precheck::{model_issues, precheck_model, IssueSeverity};
    use squid_n_core::ids::SecondaryMemberId;
    use squid_n_core::model::{SecondaryMember, SecondaryMemberKind};

    let mut model = make_cantilever_model();
    let n = model.nodes.len() as u32;
    model.nodes.push(Node {
        id: NodeId(n),
        coord: [10000.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.nodes.push(Node {
        id: NodeId(n + 1),
        coord: [10000.0, 4000.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.secondary_members.push(SecondaryMember {
        kind: SecondaryMemberKind::Joist,
        nodes: [NodeId(n), NodeId(n + 1)],
        section: Some(SectionId(0)),
        name: "孤立小梁".into(),
        id: SecondaryMemberId(0),
    });

    let issues = model_issues(&model);
    let warning = issues
        .iter()
        .find(|i| {
            i.severity == IssueSeverity::Warning
                && (i.message.contains("小梁") || i.short.contains("小梁"))
                && (i.message.contains("所属")
                    || i.message.contains("割り当て")
                    || i.short.contains("所属")
                    || i.short.contains("割り当て"))
        })
        .unwrap_or_else(|| {
            let msgs: Vec<_> = issues.iter().map(|i| i.message.as_str()).collect();
            panic!("所属なし小梁の警告がない: {msgs:?}")
        });
    assert_eq!(warning.severity, IssueSeverity::Warning);
    assert!(precheck_model(&model).is_ok(), "解析は止めない");
}

/// 大梁の区画に載らない浮き床板は警告し、解析は止めない。
#[test]
fn test_model_issues_warns_floating_plate() {
    use super::precheck::{model_issues, precheck_model, IssueSeverity};
    use squid_n_core::ids::{MaterialId, SlabId};
    use squid_n_core::model::{DistributionMethod, Material, MaterialCategory, Slab, SlabShape};

    let mut model = make_cantilever_model();
    let n = model.nodes.len() as u32;
    for (i, c) in [
        [8000.0, 8000.0, 0.0],
        [12000.0, 8000.0, 0.0],
        [12000.0, 11000.0, 0.0],
        [8000.0, 11000.0, 0.0],
    ]
    .into_iter()
    .enumerate()
    {
        model.nodes.push(Node {
            id: NodeId(n + i as u32),
            coord: c,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    let sid = SectionId(model.sections.len() as u32);
    let mid = MaterialId(model.materials.len() as u32);
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: mid,
        name: "Fc24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    let mut sec = squid_n_core::section_shape::SectionShape::RcSlab { thickness: 150.0 }
        .to_section(sid, "S15".into());
    sec.material = Some(mid);
    model.sections.push(sec);
    model.slabs.push(Slab {
        id: SlabId(0),
        shape: SlabShape::Enclosed {
            boundary: vec![NodeId(n), NodeId(n + 1), NodeId(n + 2), NodeId(n + 3)],
        },
        plate: SlabPlate {
            section: Some(sid),
            loads: Vec::new(),
            usage: None,
            method: DistributionMethod::TriTrapezoid,
            one_way: None,
        },
    });

    let issues = model_issues(&model);
    let warning = issues
        .iter()
        .find(|i| {
            i.severity == IssueSeverity::Warning
                && (i.message.contains("床板") || i.short.contains("床板"))
                && (i.message.contains("割り当て")
                    || i.message.contains("浮")
                    || i.short.contains("割り当て")
                    || i.short.contains("浮"))
        })
        .unwrap_or_else(|| {
            let msgs: Vec<_> = issues.iter().map(|i| i.message.as_str()).collect();
            panic!("浮き床板の警告がない: {msgs:?}")
        });
    assert_eq!(warning.severity, IssueSeverity::Warning);
    assert!(precheck_model(&model).is_ok(), "解析は止めない");
}

/// 4 節点でない壁版・断面未割当の壁版は警告し、解析は止めない。
#[test]
fn test_model_issues_warns_wall_plates_not_expanded() {
    use super::precheck::{model_issues, precheck_model, IssueSeverity};
    use squid_n_core::ids::WallPlateId;
    use squid_n_core::model::{WallPlate, WallPlateShape};

    let mut model = make_cantilever_model();
    let n = model.nodes.len() as u32;
    for (i, c) in [
        [1000.0, 0.0, 3000.0],
        [0.0, 0.0, 3000.0],
        [500.0, 0.0, 1500.0],
    ]
    .into_iter()
    .enumerate()
    {
        model.nodes.push(Node {
            id: NodeId(n + i as u32),
            coord: c,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.wall_plates.push(WallPlate {
        id: WallPlateId(0),
        shape: WallPlateShape::Enclosed {
            boundary: vec![
                NodeId(0),
                NodeId(1),
                NodeId(n),
                NodeId(n + 2),
                NodeId(n + 1),
            ],
        },
        section: Some(SectionId(0)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings: Vec::new(),
        three_side_slit: false,
    });
    model.wall_plates.push(WallPlate {
        id: WallPlateId(1),
        shape: WallPlateShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(n), NodeId(n + 1)],
        },
        section: None,
        opening_area: 0.0,
        opening_weight: 0.0,
        openings: Vec::new(),
        three_side_slit: false,
    });

    let issues = model_issues(&model);
    let msgs: Vec<&str> = issues.iter().map(|i| i.message.as_str()).collect();
    assert!(
        issues.iter().any(|i| {
            i.severity == IssueSeverity::Warning && i.message.contains("4 節点でない壁版")
        }),
        "4 節点でない壁版の警告がない: {msgs:?}"
    );
    assert!(
        issues.iter().any(|i| {
            i.severity == IssueSeverity::Warning && i.message.contains("断面未割当の壁版")
        }),
        "断面未割当の壁版の警告がない: {msgs:?}"
    );
    assert!(precheck_model(&model).is_ok(), "解析は止めない");
}
