use super::*;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    ElementData, ElementKind, Haunch, JointKind, LocalAxis, MaterialCategory, MemberDetailAttr,
    MemberJoint, Node, Section,
};

fn sample_model() -> Model {
    Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: squid_n_core::dof::Dof6Mask::FIXED,
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: squid_n_core::dof::Dof6Mask::FREE,
                mass: None,
                story: Some(squid_n_core::ids::StoryId(0)),
                support_spring: None,
            },
        ],
        sections: vec![Section {
            id: SectionId(0),
            name: "H-400".to_string(),
            area: 100.0,
            iy: 1000.0,
            iz: 2000.0,
            j: 50.0,
            depth: 400.0,
            width: 200.0,
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
        }],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [
                squid_n_core::model::EndCondition::Fixed,
                squid_n_core::model::EndCondition::Fixed,
            ],
            force_regime: squid_n_core::model::ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        ..Default::default()
    }
}

#[test]
fn test_query_model_nodes() {
    let m = sample_model();
    let items = query_model(&m, "node", None);
    assert_eq!(items.len(), 2);
    assert_eq!(items[0]["id"], 0);
    assert_eq!(items[1]["story"], 0);
}

#[test]
fn test_query_model_elements_and_sections() {
    let m = sample_model();
    assert_eq!(query_model(&m, "member", None).len(), 1);
    let secs = query_model(&m, "section", None);
    assert_eq!(secs.len(), 1);
    assert_eq!(secs[0]["name"], "H-400");
}

/// 断面の問い合わせは材料 4 欄を出す。材料は断面が持ち、未割当は解析前チェックが
/// 止めるため、どの断面のどの欄が空かを問い合わせ側から追えるようにする。
#[test]
fn test_query_model_sections_expose_materials() {
    let mut m = sample_model();
    m.sections[0].rebar_material = None;
    let secs = query_model(&m, "section", None);
    assert_eq!(secs[0]["material"], 0, "主材料の ID を出す");
    assert!(
        secs[0]["rebar_material"].is_null(),
        "未割当の欄は null で見分けられる"
    );
    for key in ["floor", "shear_rebar_material", "steel_material"] {
        assert!(secs[0].get(key).is_some(), "{key} の欄がある");
    }
}

#[test]
fn test_query_model_filter() {
    let m = sample_model();
    // 名前で絞り込み（断面名 H-400 を含むものだけ）。
    assert_eq!(query_model(&m, "section", Some("H-400")).len(), 1);
    assert_eq!(query_model(&m, "section", Some("RC")).len(), 0);
}

#[test]
fn test_query_model_unknown_kind() {
    let m = sample_model();
    assert!(query_model(&m, "bogus", None).is_empty());
}

/// 部材付帯情報（ハンチ・継手位置）が登録された部材は、`query_model` の
/// member/elements 出力に `haunch_i`/`haunch_j`/`joints` が含まれる。
/// 付帯情報がない部材（本テストには含めない）は従来どおりのフィールドのみとなる
/// （`test_query_model_elements_and_sections` で確認済み）。
#[test]
fn test_query_model_elements_with_member_detail() {
    let mut m = sample_model();
    m.member_detail_attrs.push(MemberDetailAttr {
        elem: ElemId(0),
        haunch_i: Some(Haunch {
            length: 700.0,
            depth_increase: 200.0,
            width_increase: 0.0,
        }),
        haunch_j: Some(Haunch {
            length: 500.0,
            depth_increase: 150.0,
            width_increase: 50.0,
        }),
        joints: vec![MemberJoint {
            distance: 1000.0,
            kind: JointKind::Shop,
        }],
    });
    let items = query_model(&m, "elements", None);
    assert_eq!(items.len(), 1);
    let e = &items[0];
    assert_eq!(e["haunch_i"]["length"], 700.0);
    assert_eq!(e["haunch_i"]["depth_increase"], 200.0);
    assert_eq!(e["haunch_j"]["width_increase"], 50.0);
    let joints = e["joints"].as_array().expect("joints 配列");
    assert_eq!(joints.len(), 1);
    assert_eq!(joints[0]["distance"], 1000.0);
    assert_eq!(joints[0]["kind"], "Shop");
}

/// RC 矩形の片持ち柱モデル（終局検定ジョブ用）。長期荷重ケース 1 つ。
fn rc_column_model() -> Model {
    use squid_n_core::model::{LoadCase, Material, NodalLoad};
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    let rebar = RcRebar {
        main_x: BarSet {
            count: 8,
            dia: 25.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 8,
            dia: 25.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
        },
    };
    let shape = SectionShape::RcRect {
        b: 600.0,
        d: 600.0,
        rebar,
    };
    Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: squid_n_core::dof::Dof6Mask::FIXED,
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: squid_n_core::dof::Dof6Mask::FREE,
                mass: None,
                story: Some(squid_n_core::ids::StoryId(0)),
                support_spring: None,
            },
        ],
        // 材料は断面が持つ。RC 断面は主筋・せん断補強筋も要る。
        sections: vec![Section {
            material: Some(MaterialId(0)),
            rebar_material: Some(MaterialId(1)),
            shear_rebar_material: Some(MaterialId(1)),
            ..shape.to_section(SectionId(0), "C600".into())
        }],
        materials: vec![
            Material {
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
            },
            Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(1),
                name: "SD345".into(),
                category: MaterialCategory::Rebar,
                young: 205000.0,
                poisson: 0.3,
                density: 7.85e-9,
                shear: None,
                fc: None,
                fy: Some(345.0),
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [
                squid_n_core::model::EndCondition::Fixed,
                squid_n_core::model::EndCondition::Fixed,
            ],
            force_regime: squid_n_core::model::ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: squid_n_core::ids::LoadCaseId(0),
            name: "長期".into(),
            nodal: vec![NodalLoad::manual(
                NodeId(1),
                [0.0, 0.0, -500_000.0, 0.0, 0.0, 0.0],
            )],
            member: Vec::new(),
        }],
        ..Default::default()
    }
}

#[test]
fn test_compute_ultimate_check_job() {
    let model = rc_column_model();
    let outcome = compute_job(&model, JobKind::UltimateCheck, &JobParams::default())
        .expect("終局検定ジョブは成功するはず");
    match outcome {
        JobOutcome::UltimateCheck { summary } => {
            assert_eq!(summary["kind"], "UltimateCheck");
            assert_eq!(summary["n_checks"], 1);
            // 柱 1 本のせん断余裕度・耐力が算定されている。
            let members = summary["members"].as_array().expect("members 配列");
            assert_eq!(members.len(), 1);
            assert!(members[0]["qsu"].as_f64().unwrap() > 0.0);
            assert!(members[0]["shear_margin"].as_f64().unwrap() > 0.0);
            // CFT 集計キーが存在する（本モデルは CFT 柱なしなので 0）。
            assert_eq!(summary["n_cft_checks"], 0);
            assert!(summary["cft_members"].is_array());
        }
        _ => panic!("expected UltimateCheck outcome"),
    }
}

/// DesignCheck ジョブは既定では危険断面位置（柱フェイス [face=0 につき節点芯]・
/// 中央）の 3 断面のみを検定する（付帯情報なし）。
#[test]
fn test_compute_design_check_job_default_positions() {
    let model = rc_column_model();
    let outcome = compute_job(&model, JobKind::DesignCheck, &JobParams::default())
        .expect("断面検定ジョブは成功するはず");
    match outcome {
        JobOutcome::DesignCheck { summary, .. } => {
            assert_eq!(summary["kind"], "DesignCheck");
            assert_eq!(summary["n_checks"], 3);
        }
        _ => panic!("expected DesignCheck outcome"),
    }
}

/// 部材付帯情報（継手位置）が登録された部材は、継手位置でも断面力が評価され
/// （squid-n-element の `eval_sections` 拡張）、DesignCheck の検定位置にも
/// 継手位置が加わる（既定 3 断面 + 継手 1 = 4 検定）。
#[test]
fn test_compute_design_check_job_member_detail_joint() {
    let mut model = rc_column_model();
    // 節点間距離 3000mm の柱に、始端から 1000mm（正規化 1/3）の現場継手を追加する。
    model.member_detail_attrs.push(MemberDetailAttr {
        elem: ElemId(0),
        haunch_i: None,
        haunch_j: None,
        joints: vec![MemberJoint {
            distance: 1000.0,
            kind: JointKind::Site,
        }],
    });
    let outcome = compute_job(&model, JobKind::DesignCheck, &JobParams::default())
        .expect("断面検定ジョブは成功するはず");
    match outcome {
        JobOutcome::DesignCheck {
            member_force_rows,
            summary,
            ..
        } => {
            assert_eq!(summary["kind"], "DesignCheck");
            // 継手位置 1000/3000 の断面力行が追加されている。
            assert!(member_force_rows
                .iter()
                .any(|(_, pos, _)| (pos - 1000.0 / 3000.0).abs() < 1e-6));
            // 継手位置分だけ検定数が増える（3 -> 4）。
            assert_eq!(summary["n_checks"], 4);
        }
        _ => panic!("expected DesignCheck outcome"),
    }
}

#[test]
fn test_job_registry_lifecycle() {
    let mut reg = JobRegistry::new();
    let id = reg.register(JobKind::LinearStatic);
    assert!(matches!(reg.get(&id).unwrap().status, JobStatus::Queued));
    reg.update(&id, JobStatus::Running { progress: 0.5 });
    assert!(matches!(
        reg.get(&id).unwrap().status,
        JobStatus::Running { progress } if (progress - 0.5).abs() < 1e-6
    ));
    reg.update(
        &id,
        JobStatus::Done {
            result_ref: "r1".into(),
        },
    );
    assert!(matches!(
        &reg.get(&id).unwrap().status,
        JobStatus::Done { result_ref } if result_ref == "r1"
    ));
    // 異なる ID は別ジョブ。
    let id2 = reg.register(JobKind::Eigen);
    assert_ne!(id, id2);
    assert!(reg.get("nonexistent").is_none());
}

#[test]
fn test_quantity_takeoff_json_column() {
    let model = rc_column_model();
    // 部位別（既定）: RC 柱 1 本 → 0.6×0.6×3.0 = 1.08 m³。
    let v = quantity_takeoff_json(&model, None);
    let rows = v["rows"].as_array().unwrap();
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0]["category"], "柱");
    assert!((rows[0]["concrete_m3"].as_f64().unwrap() - 1.08).abs() < 1e-9);
    // 明細: 部材 1 件。合計と注記も返る。
    let detail = quantity_takeoff_json(&model, Some("detail"));
    assert_eq!(detail["rows"].as_array().unwrap().len(), 1);
    assert!(detail["totals"]["rebar_t"].as_f64().unwrap() > 0.0);
    assert!(!detail["notes"].as_array().unwrap().is_empty());
    // 鉄筋径別: D25（主筋）と D10（フープ）。
    let rebar = quantity_takeoff_json(&model, Some("rebar"));
    assert_eq!(rebar["rows"].as_array().unwrap().len(), 2);
}

#[test]
fn test_query_model_wall_plates() {
    use squid_n_core::model::{WallPlate, WallPlateShape};

    let mut m = sample_model();
    m.wall_plates.push(WallPlate {
        self_weight_shares: Vec::new(),
        id: squid_n_core::ids::WallPlateId(0),
        shape: WallPlateShape::Enclosed,
        section: Some(SectionId(0)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings: Vec::new(),
        loads: vec![],
        slit: Default::default(),
    });
    let items = query_model(&m, "wall_plate", None);
    assert_eq!(items.len(), 1);
    assert_eq!(items[0]["id"], 0);
    assert_eq!(items[0]["shape"]["kind"], "Enclosed");
    // 耐震スリットは辺ごとの真偽値として出す。
    assert_eq!(
        items[0]["slit"],
        serde_json::json!({ "column_face": [false, false], "beam_face": [false, false] })
    );
    // 壁エレメントになるかも出す（どの壁版が解析に効いているかを引けるようにする）。
    assert_eq!(items[0]["becomes_element"], serde_json::json!(false));
}

/// 耐震スリットは辺ごとに読み書きでき、省略時はどの辺も切れていない扱いになる。
#[test]
fn test_apply_edit_set_wall_plate_slit() {
    use squid_n_core::model::{WallPlate, WallPlateShape};

    let mut state = ServerState {
        model: sample_model(),
        undo: squid_n_edit::UndoStack::new(),
        jobs: JobRegistry::new(),
        results: squid_n_io::results::FsResultStore::open(
            std::env::temp_dir().join(format!("squid-n-test-{}/mcp_slit_test", std::process::id())),
        )
        .expect("temp store"),
    };
    state.model.wall_plates.push(WallPlate {
        self_weight_shares: Vec::new(),
        id: squid_n_core::ids::WallPlateId(0),
        shape: WallPlateShape::Enclosed,
        section: Some(SectionId(0)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings: Vec::new(),
        loads: vec![],
        slit: Default::default(),
    });

    // 三方スリット（柱際 2 辺 ＋ 下辺）を辺の組み合わせで指定する。
    let body = serde_json::json!({
        "command": "SetWallPlateAttrs",
        "id": 0,
        "slit": { "column_face": [true, true], "beam_face": [true, false] }
    });
    assert!(apply_edit(&mut state, &body).expect("apply").applied);
    assert_eq!(state.model.wall_plates[0].slit.column_face, [true, true]);
    assert_eq!(state.model.wall_plates[0].slit.beam_face, [true, false]);

    // 片方のキーだけでも指定できる。欠けた側は切れていない扱い。
    let body = serde_json::json!({
        "command": "SetWallPlateAttrs",
        "id": 0,
        "slit": { "beam_face": [false, true] }
    });
    assert!(apply_edit(&mut state, &body).expect("apply").applied);
    assert_eq!(state.model.wall_plates[0].slit.column_face, [false, false]);
    assert_eq!(state.model.wall_plates[0].slit.beam_face, [false, true]);

    // 省略すると「どの辺も切れていない」へ戻る（他の属性と同じ既定の扱い）。
    let body = serde_json::json!({ "command": "SetWallPlateAttrs", "id": 0 });
    assert!(apply_edit(&mut state, &body).expect("apply").applied);
    assert!(!state.model.wall_plates[0].slit.any());

    // 要素数が違う配列は誤りとして弾く（片側だけ指定して残りが既定になる、
    // という黙った解釈をしない）。
    let body = serde_json::json!({
        "command": "SetWallPlateAttrs",
        "id": 0,
        "slit": { "column_face": [true] }
    });
    assert!(apply_edit(&mut state, &body).is_err());
}

#[test]
fn test_apply_edit_wall_plate_region_assignment() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{NodeId, WallPlateId};
    use squid_n_core::model::{Node, WallPlate, WallPlateShape};

    let mut model = Model {
        nodes: (0..4)
            .map(|i| Node {
                id: NodeId(i),
                coord: match i {
                    0 => [0.0, 0.0, 0.0],
                    1 => [3000.0, 0.0, 0.0],
                    2 => [3000.0, 0.0, 3000.0],
                    _ => [0.0, 0.0, 3000.0],
                },
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            })
            .collect(),
        sections: sample_model().sections,
        ..Default::default()
    };
    let first = model.add_enclosed_wall_plate_from_nodes(
        &[NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        WallPlate {
            self_weight_shares: Vec::new(),
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed,
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![],
            slit: Default::default(),
        },
    );
    let region = model
        .wall_plate_assignment_region(first)
        .expect("割当領域")
        .id;
    let mut state = ServerState {
        model,
        undo: squid_n_edit::UndoStack::new(),
        jobs: JobRegistry::new(),
        results: squid_n_io::results::FsResultStore::open(std::env::temp_dir().join(format!(
            "squid-n-test-{}/mcp_wall_assign",
            std::process::id()
        )))
        .expect("temp store"),
    };

    // 未設定へ戻すと版も消える。
    let body = serde_json::json!({ "command": "UnsetWallPlateRegion", "region": region.0 });
    assert!(apply_edit(&mut state, &body).expect("apply").applied);
    assert!(state.model.wall_plates.is_empty());

    // 割り当て直すと生成される。
    let body = serde_json::json!({
        "command": "AssignWallPlateToRegion",
        "region": region.0,
        "section": null,
        "opening_area": 0.0,
        "opening_weight": 0.0
    });
    assert!(apply_edit(&mut state, &body).expect("apply").applied);
    assert_eq!(state.model.wall_plates.len(), 1);
    assert!(matches!(
        state.model.wall_plates[0].shape,
        WallPlateShape::Enclosed
    ));
    assert_eq!(query_model(&state.model, "wall_plate", None).len(), 1);

    // 版なしへ。
    let body = serde_json::json!({ "command": "SetWallPlateRegionNoPlate", "region": region.0 });
    assert!(apply_edit(&mut state, &body).expect("apply").applied);
    assert!(state.model.wall_plates.is_empty());

    // 領域不在は applied:false。
    let body = serde_json::json!({ "command": "AssignWallPlateToRegion", "region": 99 });
    assert!(!apply_edit(&mut state, &body).expect("parse ok").applied);
}

#[test]
fn test_apply_edit_noop_unknown_node() {
    let mut state = ServerState {
        model: sample_model(),
        undo: squid_n_edit::UndoStack::new(),
        jobs: JobRegistry::new(),
        results: squid_n_io::results::FsResultStore::open(
            std::env::temp_dir().join(format!("squid-n-test-{}/mcp_edit_noop", std::process::id())),
        )
        .expect("temp store"),
    };
    let body = serde_json::json!({
        "command": "AssignWallPlateToRegion",
        "region": 99
    });
    let result = apply_edit(&mut state, &body).expect("parse ok");
    assert!(!result.applied);
    assert!(state.model.wall_plates.is_empty());
}

#[test]
fn test_query_model_slabs_and_floor_regions() {
    use squid_n_core::ids::{FloorRegionId, NodeId, SlabId};
    use squid_n_core::model::{DistributionMethod, Slab, SlabPlate, SlabShape};

    let mut m = sample_model();
    m.slabs.push(Slab {
        id: SlabId(0),
        shape: SlabShape::Enclosed,
        plate: SlabPlate {
            section: Some(SectionId(0)),
            method: DistributionMethod::TriTrapezoid,
            ..Default::default()
        },
    });
    m.floor_regions.push(squid_n_core::model::FloorRegion {
        id: FloorRegionId(0),
        name: "R1".into(),
        boundary: vec![NodeId(0), NodeId(1)],
        secondary_joists: Vec::new(),
        slab_ids: vec![SlabId(0)],
    });
    assert_eq!(query_model(&m, "slab", None).len(), 1);
    let regions = query_model(&m, "floor_region", None);
    assert_eq!(regions.len(), 1);
    assert_eq!(regions[0]["name"], "R1");
}

#[test]
fn test_apply_edit_assign_slab_to_floor_plate_region() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, NodeId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node, PlateAssignment,
        SlabShape,
    };

    let mut model = Model {
        nodes: (0..4)
            .map(|i| Node {
                id: NodeId(i),
                coord: match i {
                    0 => [0.0, 0.0, 0.0],
                    1 => [3000.0, 0.0, 0.0],
                    2 => [3000.0, 3000.0, 0.0],
                    _ => [0.0, 3000.0, 0.0],
                },
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            })
            .collect(),
        sections: sample_model().sections,
        ..Default::default()
    };
    for (i, (a, b)) in [(0u32, 1u32), (1, 2), (2, 3), (3, 0)]
        .into_iter()
        .enumerate()
    {
        model.elements.push(ElementData {
            id: ElemId(i as u32),
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
    let report = model.rebuild_floor_assignment_regions();
    assert_eq!(report.regions, 1, "正方形の大梁で 1 面");
    let region = model.floor_assignment_regions.regions[0].id;

    let mut state = ServerState {
        model,
        undo: squid_n_edit::UndoStack::new(),
        jobs: JobRegistry::new(),
        results: squid_n_io::results::FsResultStore::open(std::env::temp_dir().join(format!(
            "squid-n-test-{}/mcp_edit_assign_slab",
            std::process::id()
        )))
        .expect("temp store"),
    };

    let body = serde_json::json!({
        "command": "AssignSlabToFloorPlateRegion",
        "region": region.0,
        "section": null,
        "method": "TriTrapezoid"
    });
    let result = apply_edit(&mut state, &body).expect("apply");
    assert!(result.applied);
    assert_eq!(state.model.slabs.len(), 1);
    assert!(matches!(state.model.slabs[0].shape, SlabShape::Enclosed));
    assert!(matches!(
        state.model.floor_assignment_regions.regions[0].assignment,
        PlateAssignment::Plate(_)
    ));
}

fn four_node_edit_state(name: &str) -> ServerState {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::NodeId;
    use squid_n_core::model::Node;

    ServerState {
        model: Model {
            nodes: (0..4)
                .map(|i| Node {
                    id: NodeId(i),
                    coord: match i {
                        0 => [0.0, 0.0, 0.0],
                        1 => [3000.0, 0.0, 0.0],
                        2 => [3000.0, 3000.0, 0.0],
                        _ => [0.0, 3000.0, 0.0],
                    },
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                    support_spring: None,
                })
                .collect(),
            sections: sample_model().sections,
            ..Default::default()
        },
        undo: squid_n_edit::UndoStack::new(),
        jobs: JobRegistry::new(),
        results: squid_n_io::results::FsResultStore::open(std::env::temp_dir().join(format!(
            "squid-n-test-{}/mcp_edit_{name}",
            std::process::id()
        )))
        .expect("temp store"),
    }
}

#[test]
fn test_apply_edit_nested_body_wrapper() {
    let mut state = four_node_edit_state("nested_body");
    let body = serde_json::json!({
        "body": {
            "command": "AddAttachedWallPlate",
            "anchor": {
                "Line": { "nodes": [0, 1], "span": [0.0, 1.0], "transfer": "Anchor" }
            },
            "extent": [900.0, 900.0],
            "section": null
        }
    });
    let result = apply_edit(&mut state, &body).expect("apply");
    assert!(result.applied);
    assert_eq!(state.model.wall_plates.len(), 1);
}

#[test]
fn test_apply_edit_add_attached_slab_flat_and_plate() {
    use squid_n_core::model::SlabShape;

    let mut state = four_node_edit_state("attached_slab_flat");
    let flat = serde_json::json!({
        "command": "AddAttachedSlab",
        "anchor": {
            "Line": {
                "nodes": [0, 1],
                "span": [0.0, 1.0],
                "transfer": "Anchor"
            }
        },
        "extent": [1000.0, 1000.0],
        "section": 0
    });
    let result = apply_edit(&mut state, &flat).expect("flat");
    assert!(
        result.applied,
        "フラット引数で AddAttachedSlab が適用される"
    );
    assert_eq!(state.model.slabs.len(), 1);
    assert!(matches!(
        state.model.slabs[0].shape,
        SlabShape::Attached { .. }
    ));
    assert_eq!(state.model.slabs[0].plate.section.map(|s| s.0), Some(0));

    let mut state = four_node_edit_state("attached_slab_plate");
    let nested_plate = serde_json::json!({
        "command": "AddAttachedSlab",
        "anchor": {
            "Line": {
                "nodes": [0, 1],
                "span": [0.0, 1.0],
                "transfer": "Anchor"
            }
        },
        "extent": [800.0, 800.0],
        "plate": {
            "section": 0,
            "loads": [],
            "method": "TriTrapezoid"
        }
    });
    let result = apply_edit(&mut state, &nested_plate).expect("plate");
    assert!(
        result.applied,
        "plate オブジェクトでも AddAttachedSlab が適用される"
    );
    assert_eq!(state.model.slabs.len(), 1);
}

#[test]
fn test_apply_edit_add_attached_wall_plate() {
    use squid_n_core::model::WallPlateShape;

    let mut state = four_node_edit_state("attached_wall");
    let body = serde_json::json!({
        "command": "AddAttachedWallPlate",
        "anchor": {
            "Line": {
                "nodes": [0, 1],
                "span": [0.0, 1.0],
                "transfer": "Anchor"
            }
        },
        "extent": [1200.0, 1200.0],
        "section": 0
    });
    let result = apply_edit(&mut state, &body).expect("apply");
    assert!(result.applied);
    assert_eq!(state.model.wall_plates.len(), 1);
    assert!(matches!(
        state.model.wall_plates[0].shape,
        WallPlateShape::Attached { .. }
    ));
}

#[test]
fn test_apply_edit_set_floor_region_name() {
    use squid_n_core::ids::{FloorRegionId, NodeId, SlabId};

    let mut state = four_node_edit_state("floor_region_name");
    state
        .model
        .floor_regions
        .push(squid_n_core::model::FloorRegion {
            id: FloorRegionId(0),
            name: "old".into(),
            boundary: vec![NodeId(0), NodeId(1)],
            secondary_joists: Vec::new(),
            slab_ids: vec![SlabId(0)],
        });
    let body = serde_json::json!({
        "command": "SetFloorRegionName",
        "id": 0,
        "name": "R1"
    });
    let result = apply_edit(&mut state, &body).expect("apply");
    assert!(result.applied);
    assert_eq!(state.model.floor_regions[0].name, "R1");
}

fn expect_parse_err(value: serde_json::Value) -> String {
    match parse_edit_command(&value) {
        Ok(_) => panic!("エラーになるはずだった: {value}"),
        Err(e) => e,
    }
}

#[test]
fn test_parse_rejects_obsolete_set_slab_secondary_joist_ids() {
    let err = expect_parse_err(serde_json::json!({
        "command": "SetSlabSecondaryJoistIds",
        "floor_region": 0,
        "secondary_joist_ids": [1, 2]
    }));
    assert!(err.contains("廃止"), "{err}");
}

#[test]
fn test_parse_requires_secondary_joists_array() {
    let err = expect_parse_err(serde_json::json!({
        "command": "SetFloorRegionSecondaryJoists",
        "floor_region": 0
    }));
    assert!(err.contains("secondary_joists"), "{err}");
}

#[test]
fn test_parse_rejects_legacy_secondary_joist_ids_key() {
    let err = expect_parse_err(serde_json::json!({
        "command": "SetFloorRegionSecondaryJoists",
        "floor_region": 0,
        "secondary_joist_ids": [1, 2]
    }));
    assert!(err.contains("廃止"), "{err}");
}

#[test]
fn test_parse_requires_wall_region_posts() {
    let err = expect_parse_err(serde_json::json!({
        "command": "SetWallRegionPosts",
        "wall_region": 0
    }));
    assert!(err.contains("posts"), "{err}");
}

#[test]
fn test_parse_set_secondary_member_end_support() {
    let cmd = parse_edit_command(&serde_json::json!({
        "command": "SetSecondaryMemberEndSupport",
        "member": 0,
        "end_support": ["Supported", "Free"]
    }))
    .expect("解析できる");
    assert_eq!(cmd.label(), "二次部材の端部支持条件変更");
}

#[test]
fn test_parse_requires_end_support() {
    let err = expect_parse_err(serde_json::json!({
        "command": "SetSecondaryMemberEndSupport",
        "member": 0
    }));
    assert!(err.contains("end_support"), "{err}");
}

/// 廃止した手入力小梁ラインのコマンドは、黙って無視せず明示エラーにする（§3.4 F1）。
#[test]
fn test_parse_rejects_obsolete_set_floor_region_joists() {
    let err = expect_parse_err(serde_json::json!({
        "command": "SetFloorRegionJoists",
        "id": 0,
        "joists": []
    }));
    assert!(err.contains("廃止"), "{err}");
}

#[test]
fn test_parse_requires_unassigned_joist_body() {
    let err = expect_parse_err(serde_json::json!({
        "command": "AddUnassignedJoist"
    }));
    assert!(err.contains("joist"), "{err}");
}

#[test]
fn test_parse_place_secondary_member() {
    let cmd = parse_edit_command(&serde_json::json!({
        "command": "PlaceSecondaryMember",
        "parent": "floor",
        "region": 0,
        "kind": "Joist",
        "ends": {"Supported": [
            {"support": {"Primary": 0}, "position": 0.5},
            {"support": {"Primary": 2}, "position": 0.5}
        ]},
        "name": "J0"
    }))
    .expect("解析できる");
    assert_eq!(cmd.label(), "二次部材配置");
}

#[test]
fn test_parse_requires_place_secondary_member_parent() {
    let err = expect_parse_err(serde_json::json!({
        "command": "PlaceSecondaryMember",
        "kind": "Joist",
        "ends": {"Detached": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]}
    }));
    assert!(err.contains("parent"), "{err}");
}

#[test]
fn test_parse_set_secondary_member_ends() {
    let cmd = parse_edit_command(&serde_json::json!({
        "command": "SetSecondaryMemberEnds",
        "member": 0,
        "ends": {"Supported": [
            {"support": {"Primary": 0}, "position": 0.25},
            {"support": {"Primary": 2}, "position": 0.75}
        ]}
    }))
    .expect("解析できる");
    assert_eq!(cmd.label(), "二次部材の端部移動");
}

/// 二次部材の配置は小梁を床領域へ入れ、割当領域を再構築する（MCP 経路）。
#[test]
fn test_apply_edit_place_secondary_member() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, FloorRegionId, NodeId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, FloorRegion, ForceRegime, LocalAxis, Node,
    };

    let mut state = four_node_edit_state("place_secondary_member");
    let mut model = Model::default();
    for i in 0..4u32 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: match i {
                0 => [0.0, 0.0, 0.0],
                1 => [3000.0, 0.0, 0.0],
                2 => [3000.0, 3000.0, 0.0],
                _ => [0.0, 3000.0, 0.0],
            },
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    for (i, (a, b)) in [(0u32, 1u32), (1, 2), (2, 3), (3, 0)]
        .into_iter()
        .enumerate()
    {
        model.elements.push(ElementData {
            id: ElemId(i as u32),
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
    model.floor_regions.push(FloorRegion {
        id: FloorRegionId(0),
        name: String::new(),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        secondary_joists: Vec::new(),
        slab_ids: Vec::new(),
    });
    model.rebuild_floor_assignment_regions();
    state.model = model;

    let body = serde_json::json!({
        "command": "PlaceSecondaryMember",
        "parent": "floor",
        "region": 0,
        "kind": "Joist",
        "ends": {"Supported": [
            {"support": {"Primary": 0}, "position": 0.5},
            {"support": {"Primary": 2}, "position": 0.5}
        ]},
        "name": "J0"
    });
    let result = apply_edit(&mut state, &body).expect("apply");
    assert!(result.applied);
    assert_eq!(state.model.joists().count(), 1);
    assert_eq!(state.model.floor_assignment_regions.regions.len(), 2);
    assert!(
        state.model.validate().is_ok(),
        "{:?}",
        state.model.validate()
    );
}

/// MCP 経由で間柱の端部負担率を指定できる。
#[test]
fn test_mcp_set_post_gravity_end_shares() {
    let mut model = sample_model();
    model
        .unassigned_posts
        .push(squid_n_core::model::SecondaryMember {
            id: squid_n_core::ids::SecondaryMemberId(0),
            gravity_end_shares: None,
            kind: squid_n_core::model::SecondaryMemberKind::Post,
            ends: squid_n_core::model::SecondaryMemberEnds::Detached([
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 3000.0],
            ]),
            section: None,
            name: "P1".into(),
        });
    let cmd = crate::edit::parse_edit_command(&serde_json::json!({
        "command": "SetPostGravityEndShares", "member": 0, "shares": [0.25, 0.75]
    }))
    .unwrap();
    cmd.apply(&mut model);
    assert_eq!(
        model.unassigned_posts[0].gravity_end_shares,
        Some([0.25, 0.75])
    );
    crate::edit::parse_edit_command(&serde_json::json!({
        "command": "SetPostGravityEndShares", "member": 0, "shares": null
    }))
    .unwrap()
    .apply(&mut model);
    assert_eq!(model.unassigned_posts[0].gravity_end_shares, None);
}
