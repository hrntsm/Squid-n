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
    use squid_n_core::ids::NodeId;
    use squid_n_core::model::{WallPlate, WallPlateShape};

    let mut m = sample_model();
    m.wall_plates.push(WallPlate {
        id: squid_n_core::ids::WallPlateId(0),
        shape: WallPlateShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(0), NodeId(1)],
        },
        section: Some(SectionId(0)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings: Vec::new(),
        three_side_slit: false,
    });
    let items = query_model(&m, "wall_plate", None);
    assert_eq!(items.len(), 1);
    assert_eq!(items[0]["id"], 0);
    assert_eq!(items[0]["shape"]["kind"], "Enclosed");
}

#[test]
fn test_apply_edit_add_enclosed_wall_plate() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::NodeId;
    use squid_n_core::model::{Node, WallPlateShape};

    let mut state = ServerState {
        model: Model {
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
        },
        undo: squid_n_edit::UndoStack::new(),
        jobs: JobRegistry::new(),
        results: squid_n_io::results::FsResultStore::open(
            std::env::temp_dir().join(format!("squid-n-test-{}/mcp_edit_test", std::process::id())),
        )
        .expect("temp store"),
    };

    let body = serde_json::json!({
        "command": "AddEnclosedWallPlate",
        "boundary": [0, 1, 2, 3],
        "section": null,
        "opening_area": 0.0,
        "opening_weight": 0.0
    });
    let result = apply_edit(&mut state, &body).expect("apply");
    assert!(result.applied);
    assert_eq!(state.model.wall_plates.len(), 1);
    assert!(matches!(
        state.model.wall_plates[0].shape,
        WallPlateShape::Enclosed { .. }
    ));
    assert!(state.model.elements.is_empty(), "Wall 要素を直接書かない");

    let items = query_model(&state.model, "wall_plate", None);
    assert_eq!(items.len(), 1);
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
        "command": "AddEnclosedWallPlate",
        "boundary": [0, 1, 2, 99]
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
        shape: SlabShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(0), NodeId(1)],
        },
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
        secondary_joist_ids: Vec::new(),
        slab_ids: vec![SlabId(0)],
        joists: Vec::new(),
    });
    assert_eq!(query_model(&m, "slab", None).len(), 1);
    let regions = query_model(&m, "floor_region", None);
    assert_eq!(regions.len(), 1);
    assert_eq!(regions[0]["name"], "R1");
}

#[test]
fn test_apply_edit_add_slab() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::NodeId;
    use squid_n_core::model::{Node, SlabShape};

    let mut state = ServerState {
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
        results: squid_n_io::results::FsResultStore::open(
            std::env::temp_dir().join(format!("squid-n-test-{}/mcp_edit_slab", std::process::id())),
        )
        .expect("temp store"),
    };

    let body = serde_json::json!({
        "command": "AddSlab",
        "boundary": [0, 1, 2, 3],
        "section": null,
        "method": "TriTrapezoid"
    });
    let result = apply_edit(&mut state, &body).expect("apply");
    assert!(result.applied);
    assert_eq!(state.model.slabs.len(), 1);
    assert!(matches!(
        state.model.slabs[0].shape,
        SlabShape::Enclosed { .. }
    ));
}
