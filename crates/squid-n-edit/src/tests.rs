use super::*;
use smallvec::smallvec;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::NodeId;
use squid_n_core::ids::*;
use squid_n_core::model::MaterialCategory;
use squid_n_core::model::{ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node};
use squid_n_core::model::{FloorRegion, Slab, SlabPlate, SlabShape};

fn empty_model() -> Model {
    Model::default()
}

/// 参照検証（`crate::refs`）を通すための下地モデル。
/// X 方向 1 m 間隔に節点を `n_nodes` 個並べ、隣どうしを梁 `n_elems` 本でつなぐ。
/// 節点・部材を指す編集コマンドのテストは、参照先が実在するこのモデルから始める。
fn seeded_model(n_nodes: u32, n_elems: u32) -> Model {
    let mut model = empty_model();
    for i in 0..n_nodes {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64 * 1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    for i in 0..n_elems {
        model.elements.push(ElementData {
            id: ElemId(i),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(i), NodeId(i + 1)],
            section: None,
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
    model
}

#[test]
fn test_set_node_coord_roundtrip() {
    let mut model = empty_model();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    let mut stack = UndoStack::new();

    let cmd = SetNodeCoord {
        node: NodeId(0),
        coord: [1000.0, 2000.0, 0.0],
    };
    stack.run(&mut model, Box::new(cmd));
    assert_eq!(model.nodes[0].coord, [1000.0, 2000.0, 0.0]);

    stack.undo(&mut model);
    assert_eq!(model.nodes[0].coord, [0.0, 0.0, 0.0]);

    stack.redo(&mut model);
    assert_eq!(model.nodes[0].coord, [1000.0, 2000.0, 0.0]);
}

#[test]
fn test_set_node_coord_invalid_id_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetNodeCoord {
            node: NodeId(99),
            coord: [1.0, 2.0, 3.0],
        }),
    );
    // 失敗したコマンド（Noop）は undo 履歴に積まれない。
    assert!(!stack.can_undo());
    assert!(model.nodes.is_empty());
}

#[test]
fn test_set_node_restraint_roundtrip() {
    let mut model = empty_model();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetNodeRestraint {
            node: NodeId(0),
            restraint: Dof6Mask::PINNED,
        }),
    );
    assert_eq!(model.nodes[0].restraint, Dof6Mask::PINNED);

    stack.undo(&mut model);
    assert_eq!(model.nodes[0].restraint, Dof6Mask::FREE);

    stack.redo(&mut model);
    assert_eq!(model.nodes[0].restraint, Dof6Mask::PINNED);
}

#[test]
fn test_set_node_restraint_invalid_id_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetNodeRestraint {
            node: NodeId(99),
            restraint: Dof6Mask::FIXED,
        }),
    );
    // 失敗したコマンド（Noop）は undo 履歴に積まれない。
    assert!(!stack.can_undo());
    assert!(model.nodes.is_empty());
}

#[test]
fn test_set_node_support_spring_roundtrip() {
    let mut model = empty_model();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask(0b111110), // Ux のみ自由
        mass: None,
        story: None,
        support_spring: None,
    });
    let mut stack = UndoStack::new();

    let spring = [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    stack.run(
        &mut model,
        Box::new(SetNodeSupportSpring {
            node: NodeId(0),
            spring: Some(spring),
        }),
    );
    assert_eq!(model.nodes[0].support_spring, Some(spring));

    stack.undo(&mut model);
    assert_eq!(model.nodes[0].support_spring, None);

    stack.redo(&mut model);
    assert_eq!(model.nodes[0].support_spring, Some(spring));

    // 解除（None へ）も同様に往復できる。
    stack.run(
        &mut model,
        Box::new(SetNodeSupportSpring {
            node: NodeId(0),
            spring: None,
        }),
    );
    assert_eq!(model.nodes[0].support_spring, None);
    stack.undo(&mut model);
    assert_eq!(model.nodes[0].support_spring, Some(spring));
}

/// 負のばね剛性は物理的に無意味なため 0 にクランプされること。
#[test]
fn test_set_node_support_spring_clamps_negative_to_zero() {
    let mut model = empty_model();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetNodeSupportSpring {
            node: NodeId(0),
            spring: Some([-1000.0, 500.0, -0.001, 0.0, -1.0, 1.0]),
        }),
    );
    assert_eq!(
        model.nodes[0].support_spring,
        Some([0.0, 500.0, 0.0, 0.0, 0.0, 1.0])
    );
}

#[test]
fn test_set_node_support_spring_invalid_id_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetNodeSupportSpring {
            node: NodeId(99),
            spring: Some([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        }),
    );
    // 失敗したコマンド（Noop）は undo 履歴に積まれない。
    assert!(!stack.can_undo());
    assert!(model.nodes.is_empty());
}

#[test]
fn test_add_node_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(AddNode {
            coord: [1000.0, 2000.0, 3000.0],
            restraint: Dof6Mask::FREE,
        }),
    );
    assert_eq!(model.nodes.len(), 1);
    assert_eq!(model.nodes[0].id, NodeId(0));
    assert_eq!(model.nodes[0].coord, [1000.0, 2000.0, 3000.0]);

    stack.undo(&mut model);
    assert_eq!(model.nodes.len(), 0);

    stack.redo(&mut model);
    assert_eq!(model.nodes.len(), 1);
    assert_eq!(model.nodes[0].id, NodeId(0));
}

#[test]
fn test_add_node_id_equals_index() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    for i in 0..3 {
        stack.run(
            &mut model,
            Box::new(AddNode {
                coord: [i as f64, 0.0, 0.0],
                restraint: Dof6Mask::FREE,
            }),
        );
    }
    for (i, node) in model.nodes.iter().enumerate() {
        assert_eq!(node.id, NodeId(i as u32));
    }
}

#[test]
fn test_delete_node_middle_renumbers_and_roundtrips() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    for i in 0..3 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    // 末尾の節点（N2）を使う部材を用意し、中間節点（N1）削除後に
    // 参照が N1 へ繰り上がることを確認する。
    model.elements.push(ElementData {
        id: squid_n_core::ids::ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(2)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });

    stack.run(&mut model, Box::new(DeleteNode { id: NodeId(1) }));
    assert_eq!(model.nodes.len(), 2);
    assert_eq!(model.nodes[0].id, NodeId(0));
    assert_eq!(model.nodes[1].id, NodeId(1));
    assert_eq!(model.nodes[1].coord, [2.0, 0.0, 0.0]);
    // 元 N2 だった部材参照は N1 に繰り上がる
    assert_eq!(model.elements[0].nodes[1], NodeId(1));
    assert!(model.validate().is_ok());

    stack.undo(&mut model);
    assert_eq!(model.nodes.len(), 3);
    for (i, node) in model.nodes.iter().enumerate() {
        assert_eq!(node.id, NodeId(i as u32));
        assert_eq!(node.coord, [i as f64, 0.0, 0.0]);
    }
    assert_eq!(model.elements[0].nodes.to_vec(), vec![NodeId(0), NodeId(2)]);
    assert!(model.validate().is_ok());

    stack.redo(&mut model);
    assert_eq!(model.nodes.len(), 2);
    assert_eq!(model.elements[0].nodes[1], NodeId(1));
    assert!(model.validate().is_ok());
}

#[test]
fn test_delete_node_in_use_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [1.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.elements.push(ElementData {
        id: squid_n_core::ids::ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(1)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });

    // 部材に使われている節点の削除は Noop（先に部材を削除する必要がある）
    stack.run(&mut model, Box::new(DeleteNode { id: NodeId(0) }));
    assert_eq!(model.nodes.len(), 2);
    assert!(model.validate().is_ok());
}

#[test]
fn test_add_delete_member_load_roundtrip() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::{LoadCase, MemberLoad, MemberLoadKind};
    let mut model = seeded_model(2, 1);
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "lc".into(),
        nodal: vec![],
        member: vec![],
    });
    let mut stack = UndoStack::new();
    let load = MemberLoad::manual(
        squid_n_core::ids::ElemId(0),
        [0.0, 0.0, -1.0],
        MemberLoadKind::Distributed {
            a: 0.0,
            b: 1000.0,
            w1: 2.0,
            w2: 2.0,
        },
    );
    stack.run(
        &mut model,
        Box::new(AddMemberLoad {
            lc: LoadCaseId(0),
            load: load.clone(),
        }),
    );
    assert_eq!(model.load_cases[0].member.len(), 1);
    assert_eq!(model.load_cases[0].member[0], load);

    stack.undo(&mut model);
    assert_eq!(model.load_cases[0].member.len(), 0);

    stack.redo(&mut model);
    assert_eq!(model.load_cases[0].member.len(), 1);

    // 削除と復元（位置保持）
    stack.run(
        &mut model,
        Box::new(DeleteMemberLoad {
            lc: LoadCaseId(0),
            index: 0,
        }),
    );
    assert_eq!(model.load_cases[0].member.len(), 0);
    stack.undo(&mut model);
    assert_eq!(model.load_cases[0].member.len(), 1);
    assert_eq!(model.load_cases[0].member[0], load);
}

#[test]
fn test_delete_member_undo_preserves_member_load_order() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::{LoadCase, MemberLoad, MemberLoadKind};
    let mk_elem = |id: u32| ElementData {
        id: squid_n_core::ids::ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(1)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let mk_load = |elem: u32, w: f64| {
        MemberLoad::manual(
            squid_n_core::ids::ElemId(elem),
            [0.0, 0.0, -1.0],
            MemberLoadKind::Distributed {
                a: 0.0,
                b: 1000.0,
                w1: w,
                w2: w,
            },
        )
    };
    let w1_of = |l: &MemberLoad| match l.kind {
        MemberLoadKind::Distributed { w1, .. } => w1,
        _ => 0.0,
    };

    let mut model = empty_model();
    model.elements = vec![mk_elem(0), mk_elem(1), mk_elem(2)];
    // 荷重順 [e0:1, e1:2, e2:3, e1:4]。elem1 に 2 つの荷重が離れて存在する。
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "lc".into(),
        nodal: vec![],
        member: vec![
            mk_load(0, 1.0),
            mk_load(1, 2.0),
            mk_load(2, 3.0),
            mk_load(1, 4.0),
        ],
    });

    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(DeleteMember { id: ElemId(1) }));
    // elem1 の荷重（w=2,4）が除去され、残りは [1,3]。
    let after: Vec<f64> = model.load_cases[0].member.iter().map(w1_of).collect();
    assert_eq!(after, vec![1.0, 3.0]);

    stack.undo(&mut model);
    // undo は削除前の順序 [1,2,3,4] を厳密に復元しなければならない
    // （前方順挿入では [1,2,4,3] と入れ替わっていた）。
    let restored: Vec<f64> = model.load_cases[0].member.iter().map(w1_of).collect();
    assert_eq!(
        restored,
        vec![1.0, 2.0, 3.0, 4.0],
        "undo は部材荷重の順序を厳密に復元する"
    );
}

#[test]
fn test_add_delete_member_roundtrip() {
    let mut model = seeded_model(2, 0);
    let mut stack = UndoStack::new();
    let elem = ElementData {
        id: squid_n_core::ids::ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(1)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    stack.run(&mut model, Box::new(AddMember { elem }));
    assert_eq!(model.elements.len(), 1);
    stack.undo(&mut model);
    assert_eq!(model.elements.len(), 0);
    stack.redo(&mut model);
    assert_eq!(model.elements.len(), 1);
}

#[test]
fn test_add_section_shape_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    let shape = squid_n_section::shape::SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let cmd = AddSectionShape {
        shape,
        new_id: SectionId(0),
        name: "H-300x300x10x15".into(),
        floor: None,
    };
    stack.run(&mut model, Box::new(cmd));
    assert_eq!(model.sections.len(), 1);
    assert_eq!(model.sections[0].id, SectionId(0));

    stack.undo(&mut model);
    assert_eq!(model.sections.len(), 0);

    stack.redo(&mut model);
    assert_eq!(model.sections.len(), 1);
    assert_eq!(model.sections[0].id, SectionId(0));
}

#[test]
fn test_edit_section_shape_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    let shape1 = squid_n_section::shape::SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let sec = shape1.to_section(SectionId(0), "H-300".into());
    let area_h = sec.area;
    model.sections.push(sec);

    let shape2 = squid_n_section::shape::SectionShape::SteelBox {
        height: 200.0,
        width: 200.0,
        thick: 12.0,
        corner_r: 0.0,
    };
    let cmd = EditSectionShape {
        section: SectionId(0),
        new_shape: shape2,
    };
    stack.run(&mut model, Box::new(cmd));
    assert!((model.sections[0].area - 9024.0).abs() < 1.0);

    stack.undo(&mut model);
    assert!((model.sections[0].area - area_h).abs() < 1.0);

    stack.redo(&mut model);
    assert!((model.sections[0].area - 9024.0).abs() < 1.0);
}

#[test]
fn test_duplicate_section_for_member_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    let shape = squid_n_section::shape::SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let sec = shape.to_section(SectionId(0), "H-300".into());
    model.sections.push(sec);

    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [1000.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(1)],
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

    let cmd = DuplicateSectionForMember { member: ElemId(0) };
    stack.run(&mut model, Box::new(cmd));
    assert_eq!(model.sections.len(), 2);
    assert_eq!(model.elements[0].section, Some(SectionId(1)));

    stack.undo(&mut model);
    assert_eq!(model.sections.len(), 1);
    assert_eq!(model.elements[0].section, Some(SectionId(0)));

    stack.redo(&mut model);
    assert_eq!(model.sections.len(), 2);
    assert_eq!(model.elements[0].section, Some(SectionId(1)));
}

#[test]
fn test_edit_section_shape_invalid_id_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    let shape = squid_n_section::shape::SectionShape::SteelBox {
        height: 200.0,
        width: 200.0,
        thick: 12.0,
        corner_r: 0.0,
    };
    let cmd = EditSectionShape {
        section: SectionId(99),
        new_shape: shape,
    };
    stack.run(&mut model, Box::new(cmd));
    // 失敗したコマンド（Noop）は undo 履歴に積まれない。
    assert!(!stack.can_undo());
    assert!(model.sections.is_empty());
}

#[test]
fn test_delete_add_section_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    let shape = squid_n_section::shape::SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let sec = shape.to_section(SectionId(0), "H-300".into());
    model.sections.push(sec);

    let cmd = DeleteSection { id: SectionId(0) };
    stack.run(&mut model, Box::new(cmd));
    assert_eq!(model.sections.len(), 0);

    stack.undo(&mut model);
    assert_eq!(model.sections.len(), 1);
    assert_eq!(model.sections[0].id, SectionId(0));

    stack.redo(&mut model);
    assert_eq!(model.sections.len(), 0);
}

#[test]
fn test_duplicate_section_no_section_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [1000.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(1)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });

    let cmd = DuplicateSectionForMember { member: ElemId(0) };
    stack.run(&mut model, Box::new(cmd));
    // 失敗したコマンド（Noop）は undo 履歴に積まれない。
    assert!(!stack.can_undo());
}

/// 2 節点 + 部材 2 本のモデル（部材削除・再採番テスト用）。
fn two_member_model() -> Model {
    let mut model = empty_model();
    for (i, x) in [0.0, 1000.0, 2000.0].iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: [*x, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    for i in 0..2u32 {
        model.elements.push(ElementData {
            id: ElemId(i),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(i), NodeId(i + 1)],
            section: None,
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
    model
}

/// 形状を持たない最小の断面（材料参照の検証用）。
fn bare_section(id: SectionId, material: Option<MaterialId>) -> squid_n_core::model::Section {
    squid_n_core::model::Section {
        id,
        name: format!("S{}", id.0),
        area: 100.0,
        iy: 1.0,
        iz: 1.0,
        j: 1.0,
        depth: 10.0,
        width: 10.0,
        as_y: 80.0,
        as_z: 80.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material,
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    }
}

#[test]
fn test_delete_member_middle_renumbers_and_roundtrips() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::{LoadCase, MemberLoad, MemberLoadKind};
    let mut model = two_member_model();
    // 両方の部材に部材荷重を付ける
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "lc".into(),
        nodal: vec![],
        member: vec![
            MemberLoad::manual(
                ElemId(0),
                [0.0, 0.0, -1.0],
                MemberLoadKind::Point { a: 500.0, p: 1.0 },
            ),
            MemberLoad::manual(
                ElemId(1),
                [0.0, 0.0, -1.0],
                MemberLoadKind::Point { a: 500.0, p: 2.0 },
            ),
        ],
    });
    let before = model.clone();
    let mut stack = UndoStack::new();

    // 先頭（中間）の部材を削除 → 後続 ID が繰り上がり、関連荷重も消える
    stack.run(&mut model, Box::new(DeleteMember { id: ElemId(0) }));
    assert_eq!(model.elements.len(), 1);
    assert_eq!(model.elements[0].id, ElemId(0));
    assert!(model.validate().is_ok());
    assert_eq!(model.load_cases[0].member.len(), 1);
    // 残った荷重は旧 ElemId(1) → 新 ElemId(0) を指す
    assert_eq!(model.load_cases[0].member[0].elem, ElemId(0));

    // undo で完全復元
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert!(model.validate().is_ok());
}

#[test]
fn test_delete_section_in_use_is_noop_and_renumbers() {
    use squid_n_core::model::Section;
    let mut model = two_member_model();
    for i in 0..2u32 {
        model.sections.push(Section {
            id: SectionId(i),
            name: format!("S{}", i),
            area: 100.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 10.0,
            width: 10.0,
            as_y: 80.0,
            as_z: 80.0,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: None,
            // 断面削除の検証だけを行うため、材料は割り当てない。
            material: None,
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        });
    }
    // 部材 0 に断面 1 を割当（断面 0 は未使用）
    model.elements[0].section = Some(SectionId(1));
    let mut stack = UndoStack::new();

    // 使用中の断面 1 は削除できない
    stack.run(&mut model, Box::new(DeleteSection { id: SectionId(1) }));
    assert_eq!(model.sections.len(), 2);

    // 未使用の断面 0 は削除でき、断面 1 → 0 に繰り上がり参照も追随
    let before = model.clone();
    stack.run(&mut model, Box::new(DeleteSection { id: SectionId(0) }));
    assert_eq!(model.sections.len(), 1);
    assert_eq!(model.sections[0].id, SectionId(0));
    assert_eq!(model.elements[0].section, Some(SectionId(0)));
    assert!(model.validate().is_ok());

    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
}

/// 小梁（JoistLine.section）が参照する断面は削除ガードで守られ、別断面の削除では
/// 参照が繰り上がって追随する（床設計用の断面参照が陳腐化しない。レビュー指摘）。
#[test]
fn test_delete_section_referenced_by_joist() {
    use squid_n_core::model::{JoistLine, Node, Section};
    let mut model = empty_model();
    for i in 0..4u32 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64 * 1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    for i in 0..2u32 {
        model.sections.push(Section {
            id: SectionId(i),
            name: format!("S{}", i),
            area: 100.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 10.0,
            width: 10.0,
            as_y: 80.0,
            as_z: 80.0,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: None,
            // 断面削除の検証だけを行うため、材料は割り当てない。
            material: None,
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        });
    }
    // 小梁ラインが断面 1 のみを参照する床領域（要素は断面を参照しない）。
    let mut region = squid_n_core::model::FloorRegion::new(
        FloorRegionId(0),
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
    );
    region.joists = vec![JoistLine {
        dir: [1.0, 0.0],
        spacing: 900.0,
        support: [NodeId(0), NodeId(1)],
        section: Some(SectionId(1)),
        pinned_onto: None,
    }];
    model.floor_regions.push(region);
    let mut stack = UndoStack::new();

    // 小梁が参照中の断面 1 は削除できない（要素だけでなく小梁も参照チェック対象）。
    stack.run(&mut model, Box::new(DeleteSection { id: SectionId(1) }));
    assert_eq!(model.sections.len(), 2, "小梁参照中の断面は削除されない");

    // 未参照の断面 0 を削除 → 断面 1 が 0 へ繰り上がり、小梁参照も追随する。
    stack.run(&mut model, Box::new(DeleteSection { id: SectionId(0) }));
    assert_eq!(model.sections.len(), 1);
    assert_eq!(
        model.floor_regions[0].joists[0].section,
        Some(SectionId(0)),
        "小梁の断面参照が繰り上がりに追随"
    );
    assert!(model.validate().is_ok());
}

#[test]
fn test_add_delete_material_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(AddMaterial {
            name: "SN400B".into(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            fc: None,
            fy: Some(235.0),
            strength_factor: None,
        }),
    );
    assert_eq!(model.materials.len(), 1);
    assert_eq!(model.materials[0].id, MaterialId(0));
    assert!(model.validate().is_ok());

    stack.undo(&mut model);
    assert_eq!(model.materials.len(), 0);
    stack.redo(&mut model);
    assert_eq!(model.materials.len(), 1);
}

#[test]
fn test_delete_material_in_use_is_noop() {
    let mut model = two_member_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(AddMaterial {
            name: "SN400B".into(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            fc: None,
            fy: Some(235.0),
            strength_factor: None,
        }),
    );
    // 材料は断面が持つ。参照元となる断面を 1 つ足して割り当てる。
    model
        .sections
        .push(bare_section(SectionId(0), Some(MaterialId(0))));
    stack.run(&mut model, Box::new(DeleteMaterial { id: MaterialId(0) }));
    assert_eq!(model.materials.len(), 1, "使用中の材料は削除できない");
}

#[test]
fn test_delete_material_middle_renumbers() {
    let mut model = two_member_model();
    let mut stack = UndoStack::new();
    for name in ["A", "B"] {
        stack.run(
            &mut model,
            Box::new(AddMaterial {
                name: name.into(),
                category: MaterialCategory::Steel,
                young: 1.0,
                poisson: 0.3,
                density: 0.0,
                fc: None,
                fy: None,
                strength_factor: None,
            }),
        );
    }
    model
        .sections
        .push(bare_section(SectionId(0), Some(MaterialId(1))));
    let before = model.clone();
    stack.run(&mut model, Box::new(DeleteMaterial { id: MaterialId(0) }));
    assert_eq!(model.materials.len(), 1);
    assert_eq!(model.materials[0].name, "B");
    assert_eq!(model.sections[0].material, Some(MaterialId(0)));
    assert!(model.validate().is_ok());
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
}

/// 断面の材料割当は 4 つの役割それぞれで往復し、undo で元へ戻る。
/// 材料は断面が持つため、この経路が材料割当の唯一の入口になる。
#[test]
fn test_set_section_material_roundtrip_for_every_role() {
    use crate::{SectionMaterialRole, SetSectionMaterial};

    let mut model = two_member_model();
    model.sections.push(bare_section(SectionId(0), None));
    for name in ["Fc24", "SD345"] {
        stack_add_material(&mut model, name);
    }
    let mut stack = UndoStack::new();

    let slot = |m: &squid_n_core::model::Model, role: SectionMaterialRole| match role {
        SectionMaterialRole::Main => m.sections[0].material,
        SectionMaterialRole::Rebar => m.sections[0].rebar_material,
        SectionMaterialRole::ShearRebar => m.sections[0].shear_rebar_material,
        SectionMaterialRole::Steel => m.sections[0].steel_material,
    };
    for role in [
        SectionMaterialRole::Main,
        SectionMaterialRole::Rebar,
        SectionMaterialRole::ShearRebar,
        SectionMaterialRole::Steel,
    ] {
        assert_eq!(slot(&model, role), None, "{role:?} の初期値は未割当");
        stack.run(
            &mut model,
            Box::new(SetSectionMaterial {
                section: SectionId(0),
                role,
                material: Some(MaterialId(1)),
            }),
        );
        assert_eq!(
            slot(&model, role),
            Some(MaterialId(1)),
            "{role:?} を割り当てる"
        );
        // ほかの欄は動かない（役割ごとに独立した欄であること）。
        for other in [
            SectionMaterialRole::Main,
            SectionMaterialRole::Rebar,
            SectionMaterialRole::ShearRebar,
            SectionMaterialRole::Steel,
        ] {
            if other != role {
                assert_eq!(slot(&model, other), None, "{other:?} は変わらない");
            }
        }
        assert!(model.validate().is_ok(), "{:?}", model.validate());
        stack.undo(&mut model);
        assert_eq!(slot(&model, role), None, "{role:?} の undo で未割当へ戻る");
    }
}

/// 存在しない断面を指す割当は Noop（モデルを壊さない）。
#[test]
fn test_set_section_material_on_missing_section_is_noop() {
    use crate::{SectionMaterialRole, SetSectionMaterial};

    let mut model = two_member_model();
    model.sections.push(bare_section(SectionId(0), None));
    stack_add_material(&mut model, "Fc24");
    let before = model.clone();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetSectionMaterial {
            section: SectionId(9),
            role: SectionMaterialRole::Main,
            material: Some(MaterialId(0)),
        }),
    );
    assert!(
        model.eq_ignoring_dofmap(&before),
        "存在しない断面への割当は無視する"
    );
}

/// 床への断面割当は往復し、実在しない断面の指定は Noop になる。
#[test]
fn test_set_slab_section_roundtrip() {
    use crate::{AddSlab, SetSlabSection};
    use squid_n_core::model::DistributionMethod;

    let mut model = seeded_model(4, 0);
    model.sections.push(bare_section(SectionId(0), None));
    let mut stack = UndoStack::new();
    assert!(stack.run(
        &mut model,
        Box::new(AddSlab {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            loads: Vec::new(),
            method: DistributionMethod::TriTrapezoid,
            usage: None,
            section: None,
        }),
    ));
    let slab = squid_n_core::ids::SlabId(0);

    assert!(stack.run(
        &mut model,
        Box::new(SetSlabSection {
            id: slab,
            section: Some(SectionId(0)),
        }),
    ));
    assert_eq!(model.slabs[0].section(), Some(SectionId(0)));
    stack.undo(&mut model);
    assert_eq!(model.slabs[0].section(), None, "undo で未割当へ戻る");

    // 実在しない断面の指定は Noop（モデルを壊さない）。
    assert!(!stack.run(
        &mut model,
        Box::new(SetSlabSection {
            id: slab,
            section: Some(SectionId(9)),
        }),
    ));
    assert_eq!(model.slabs[0].section(), None);
    assert!(model.validate().is_ok(), "{:?}", model.validate());
}

/// 床が参照する断面は削除できず、断面の削除で床の参照が繰り上がる。
///
/// 床は断面から板厚と自重を解決するため、断面が消えると床の重量が算定できなくなる。
/// 部材・小梁と同じく削除ガードと ID 繰り上げの対象にする。
#[test]
fn test_slab_section_reference_is_guarded_and_shifted() {
    use crate::{AddSlab, DeleteSection};
    use squid_n_core::model::DistributionMethod;

    let mut model = seeded_model(4, 0);
    for i in 0..2u32 {
        model.sections.push(bare_section(SectionId(i), None));
    }
    let mut stack = UndoStack::new();
    assert!(stack.run(
        &mut model,
        Box::new(AddSlab {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            loads: Vec::new(),
            method: DistributionMethod::TriTrapezoid,
            usage: None,
            section: Some(SectionId(1)),
        }),
    ));

    // 参照中の断面は削除できない。
    assert!(
        !stack.run(&mut model, Box::new(DeleteSection { id: SectionId(1) })),
        "床が参照する断面は削除できない"
    );

    // 参照されていない断面を消すと、後続の断面 ID が繰り上がり床の参照も追随する。
    assert!(stack.run(&mut model, Box::new(DeleteSection { id: SectionId(0) })));
    assert_eq!(
        model.slabs[0].section(),
        Some(SectionId(0)),
        "参照が繰り上がる"
    );
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    stack.undo(&mut model);
    assert_eq!(model.slabs[0].section(), Some(SectionId(1)), "undo で戻る");
    assert!(model.validate().is_ok(), "{:?}", model.validate());
}

/// 実在しない ID を指す割当は Noop（`Model::validate` が落ちるモデルを作らない）。
/// 参照の存在は書き込む側で確かめる（`crate::refs` の規約）。
#[test]
fn test_commands_reject_dangling_references() {
    use crate::{
        AddMember, AddMemberLoad, AddNodalLoad, AddSlab, SectionMaterialRole, SetElementSection,
        SetSectionMaterial,
    };
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::{
        DistributionMethod, LoadCase, LoadCaseKind, MemberLoad, MemberLoadKind, NodalLoad,
    };

    let mut model = two_member_model();
    model.sections.push(bare_section(SectionId(0), None));
    stack_add_material(&mut model, "Fc24");
    model.load_cases.push(LoadCase {
        id: LoadCaseId(0),
        name: "L".into(),
        kind: LoadCaseKind::Dead,
        nodal: Vec::new(),
        member: Vec::new(),
    });
    let before = model.clone();
    let mut stack = UndoStack::new();

    let new_elem = |id: u32, nodes: smallvec::SmallVec<[NodeId; 8]>, section| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes,
        section,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    let cases: Vec<(&str, Box<dyn crate::EditCommand>)> = vec![
        (
            "存在しない材料",
            Box::new(SetSectionMaterial {
                section: SectionId(0),
                role: SectionMaterialRole::Main,
                material: Some(MaterialId(9)),
            }),
        ),
        (
            "存在しない断面",
            Box::new(SetElementSection {
                elem: ElemId(0),
                section: Some(SectionId(9)),
            }),
        ),
        (
            "存在しない節点を端部に持つ部材",
            Box::new(AddMember {
                elem: new_elem(2, smallvec![NodeId(0), NodeId(9)], None),
            }),
        ),
        (
            "末尾でない ID の部材",
            Box::new(AddMember {
                elem: new_elem(7, smallvec![NodeId(0), NodeId(1)], None),
            }),
        ),
        (
            "存在しない断面を指す部材",
            Box::new(AddMember {
                elem: new_elem(2, smallvec![NodeId(0), NodeId(1)], Some(SectionId(9))),
            }),
        ),
        (
            "存在しない節点への節点荷重",
            Box::new(AddNodalLoad {
                lc: LoadCaseId(0),
                load: NodalLoad {
                    node: NodeId(9),
                    values: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    name: String::new(),
                    source: squid_n_core::model::LoadSource::Manual,
                },
            }),
        ),
        (
            "存在しない部材への部材荷重",
            Box::new(AddMemberLoad {
                lc: LoadCaseId(0),
                load: MemberLoad::manual(
                    ElemId(9),
                    [0.0, 0.0, -1.0],
                    MemberLoadKind::Distributed {
                        a: 0.0,
                        b: 1000.0,
                        w1: 1.0,
                        w2: 1.0,
                    },
                ),
            }),
        ),
        (
            "存在しない節点を境界に持つ床",
            Box::new(AddSlab {
                boundary: vec![NodeId(0), NodeId(1), NodeId(9)],
                loads: Vec::new(),
                method: DistributionMethod::TriTrapezoid,
                usage: None,
                section: None,
            }),
        ),
        (
            "存在しない断面を指す床",
            Box::new(AddSlab {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2)],
                loads: Vec::new(),
                method: DistributionMethod::TriTrapezoid,
                usage: None,
                section: Some(SectionId(9)),
            }),
        ),
    ];
    for (what, cmd) in cases {
        assert!(!stack.run(&mut model, cmd), "{what}: 適用されてしまった");
        assert!(
            model.eq_ignoring_dofmap(&before),
            "{what}: モデルが変更されている"
        );
    }
    assert!(model.validate().is_ok(), "{:?}", model.validate());
    assert!(!stack.can_undo(), "Noop は undo 履歴へ積まない");
}

/// テスト用: 名前だけを指定して材料を足す。
fn stack_add_material(model: &mut squid_n_core::model::Model, name: &str) {
    let id = MaterialId(model.materials.len() as u32);
    model.materials.push(squid_n_core::model::Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id,
        name: name.to_string(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: None,
    });
}

#[test]
fn test_set_material_field_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(AddMaterial {
            name: "Fc21".into(),
            category: MaterialCategory::Concrete,
            young: 21500.0,
            poisson: 0.2,
            density: 2.3e-9,
            fc: Some(21.0),
            fy: None,
            strength_factor: None,
        }),
    );
    stack.run(
        &mut model,
        Box::new(SetMaterialField {
            id: MaterialId(0),
            field: MaterialField::Fc,
            value: Some(24.0),
        }),
    );
    assert_eq!(model.materials[0].fc, Some(24.0));
    stack.undo(&mut model);
    assert_eq!(model.materials[0].fc, Some(21.0));
}

#[test]
fn test_set_material_strength_factor_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(AddMaterial {
            name: "SA440".into(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            fc: None,
            fy: Some(440.0),
            strength_factor: None,
        }),
    );
    assert_eq!(model.materials[0].strength_factor, None, "既定は自動判定");

    stack.run(
        &mut model,
        Box::new(SetMaterialField {
            id: MaterialId(0),
            field: MaterialField::StrengthFactor,
            value: Some(1.2),
        }),
    );
    assert_eq!(model.materials[0].strength_factor, Some(1.2));

    stack.undo(&mut model);
    assert_eq!(
        model.materials[0].strength_factor, None,
        "取り消しで自動判定に戻る"
    );

    stack.redo(&mut model);
    assert_eq!(model.materials[0].strength_factor, Some(1.2));
}

#[test]
fn test_add_delete_load_case_roundtrip() {
    use squid_n_core::ids::LoadCaseId;
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(AddLoadCase { name: "DL".into() }));
    stack.run(&mut model, Box::new(AddLoadCase { name: "LL".into() }));
    assert_eq!(model.load_cases.len(), 2);
    assert_eq!(model.load_cases[0].id, LoadCaseId(0));
    assert_eq!(model.load_cases[1].id, LoadCaseId(1));

    // 先頭を削除 → 後続 ID 繰り上がり
    let before = model.clone();
    stack.run(&mut model, Box::new(DeleteLoadCase { id: LoadCaseId(0) }));
    assert_eq!(model.load_cases.len(), 1);
    assert_eq!(model.load_cases[0].id, LoadCaseId(0));
    assert_eq!(model.load_cases[0].name, "LL");

    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
}

#[test]
fn test_delete_load_case_referenced_by_combo_is_noop() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::LoadCombination;
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(AddLoadCase { name: "DL".into() }));
    model.combinations.push(LoadCombination {
        name: "combo".into(),
        terms: vec![(LoadCaseId(0), 1.0)],
    });
    stack.run(&mut model, Box::new(DeleteLoadCase { id: LoadCaseId(0) }));
    assert_eq!(model.load_cases.len(), 1, "組合せ参照中のケースは削除不可");
}

#[test]
fn test_add_delete_combination_roundtrip() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::LoadCombination;
    let mut model = empty_model();
    model.combinations.push(LoadCombination {
        name: "既存".into(),
        terms: vec![(LoadCaseId(0), 1.0)],
    });
    let mut stack = UndoStack::new();

    let combo = LoadCombination {
        name: "1.0DL+1.0LL".into(),
        terms: vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), 1.0)],
    };
    stack.run(
        &mut model,
        Box::new(AddCombination {
            combo: combo.clone(),
        }),
    );
    assert_eq!(model.combinations.len(), 2);
    assert_eq!(model.combinations[1], combo);

    stack.undo(&mut model);
    assert_eq!(model.combinations.len(), 1);

    stack.redo(&mut model);
    assert_eq!(model.combinations.len(), 2);
    assert_eq!(model.combinations[1], combo);
}

#[test]
fn test_delete_combination_roundtrip_restores_position() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::LoadCombination;
    let mut model = empty_model();
    for (name, coef) in [("A", 1.0), ("B", 2.0), ("C", 3.0)] {
        model.combinations.push(LoadCombination {
            name: name.into(),
            terms: vec![(LoadCaseId(0), coef)],
        });
    }
    let before = model.clone();
    let mut stack = UndoStack::new();

    // 中間（B）を削除
    stack.run(&mut model, Box::new(DeleteCombination { index: 1 }));
    assert_eq!(model.combinations.len(), 2);
    assert_eq!(model.combinations[0].name, "A");
    assert_eq!(model.combinations[1].name, "C");

    // undo で元の位置（index 1）に復元
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert_eq!(model.combinations[1].name, "B");

    stack.redo(&mut model);
    assert_eq!(model.combinations.len(), 2);
    assert_eq!(model.combinations[0].name, "A");
    assert_eq!(model.combinations[1].name, "C");
}

#[test]
fn test_delete_combination_out_of_range_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(DeleteCombination { index: 0 }));
    assert!(model.combinations.is_empty());
    // 失敗したコマンド（Noop）は undo 履歴に積まれない。
    assert!(!stack.can_undo());
    assert!(model.combinations.is_empty());
}

#[test]
fn test_add_delete_slab_roundtrip() {
    use squid_n_core::model::DistributionMethod;
    let mut model = empty_model();
    for i in 0..4 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64 * 1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(AddSlab {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            loads: vec![],
            method: DistributionMethod::TriTrapezoid,
            usage: None,
            section: None,
        }),
    );
    assert_eq!(model.slabs.len(), 1);
    assert_eq!(model.slabs[0].id, SlabId(0));
    assert!(model.validate().is_ok());

    // 採番の確認：2 枚目は SlabId(1)
    stack.run(
        &mut model,
        Box::new(AddSlab {
            boundary: vec![NodeId(0), NodeId(1)],
            loads: vec![],
            method: DistributionMethod::OneWay,
            usage: None,
            section: None,
        }),
    );
    assert_eq!(model.slabs.len(), 2);
    assert_eq!(model.slabs[1].id, SlabId(1));

    stack.undo(&mut model);
    assert_eq!(model.slabs.len(), 1);
    assert_eq!(model.slabs[0].id, SlabId(0));

    stack.redo(&mut model);
    assert_eq!(model.slabs.len(), 2);
    assert_eq!(model.slabs[1].id, SlabId(1));
}

#[test]
fn test_delete_slab_middle_renumbers_and_roundtrips() {
    use squid_n_core::model::{AreaLoad, DistributionMethod};
    let mut model = empty_model();
    for i in 0..3 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64 * 1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    let mut stack = UndoStack::new();
    for (i, kind) in ["A", "B", "C"].iter().enumerate() {
        stack.run(
            &mut model,
            Box::new(AddSlab {
                boundary: vec![NodeId(i as u32)],
                loads: vec![AreaLoad {
                    kind: kind.to_string(),
                    value: 1.0,
                }],
                method: DistributionMethod::TributaryArea,
                usage: None,
                section: None,
            }),
        );
    }
    assert_eq!(model.slabs.len(), 3);
    let before = model.clone();

    // 中間（SlabId(1) = "B"）を削除 → 後続 ID が繰り上がる
    stack.run(&mut model, Box::new(DeleteSlab { id: SlabId(1) }));
    assert_eq!(model.slabs.len(), 2);
    assert_eq!(model.slabs[0].id, SlabId(0));
    assert_eq!(model.slabs[0].plate.loads[0].kind, "A");
    assert_eq!(model.slabs[1].id, SlabId(1));
    assert_eq!(model.slabs[1].plate.loads[0].kind, "C");
    assert!(model.validate().is_ok());

    // undo で完全復元
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert!(model.validate().is_ok());

    stack.redo(&mut model);
    assert_eq!(model.slabs.len(), 2);
    assert_eq!(model.slabs[1].plate.loads[0].kind, "C");
}

#[test]
fn test_delete_slab_out_of_range_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(DeleteSlab { id: SlabId(0) }));
    assert!(model.slabs.is_empty());
    // 失敗したコマンド（Noop）は undo 履歴に積まれない。
    assert!(!stack.can_undo());
    assert!(model.slabs.is_empty());
}

fn make_story(id: u32, weight: Option<f64>) -> squid_n_core::model::Story {
    squid_n_core::model::Story {
        level_kind: Default::default(),
        structure: Default::default(),
        id: StoryId(id),
        name: format!("{}F", id + 1),
        elevation: id as f64 * 3000.0,
        node_ids: vec![],
        seismic_weight: weight,
        weight_override: None,
    }
}

#[test]
fn test_set_story_weight_roundtrip() {
    let mut model = empty_model();
    model.stories.push(make_story(0, None));
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetStoryWeight {
            story: StoryId(0),
            weight: Some(1234.5),
        }),
    );
    // 手入力値は実効値（seismic_weight）へも反映される。
    assert_eq!(model.stories[0].weight_override, Some(1234.5));
    assert_eq!(model.stories[0].seismic_weight, Some(1234.5));

    stack.undo(&mut model);
    assert_eq!(model.stories[0].weight_override, None);
    assert_eq!(model.stories[0].seismic_weight, None);

    stack.redo(&mut model);
    assert_eq!(model.stories[0].weight_override, Some(1234.5));
    assert_eq!(model.stories[0].seismic_weight, Some(1234.5));
}

/// 手入力の解除（`weight: None`）では実効値の自動算定値は据え置き、
/// 手入力フラグのみ落ちる（次の準備計算で再算定される）。
#[test]
fn test_clear_story_weight_override_keeps_auto_value() {
    let mut model = empty_model();
    model.stories.push(make_story(0, Some(500.0)));
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetStoryWeight {
            story: StoryId(0),
            weight: Some(800.0),
        }),
    );
    stack.run(
        &mut model,
        Box::new(SetStoryWeight {
            story: StoryId(0),
            weight: None,
        }),
    );
    assert_eq!(model.stories[0].weight_override, None);
    assert_eq!(model.stories[0].seismic_weight, Some(800.0));

    stack.undo(&mut model);
    assert_eq!(model.stories[0].weight_override, Some(800.0));
}

#[test]
fn test_set_story_weight_invalid_id_is_noop() {
    let mut model = empty_model();
    model.stories.push(make_story(0, Some(999.0)));
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetStoryWeight {
            story: StoryId(99),
            weight: Some(1.0),
        }),
    );
    assert_eq!(model.stories[0].seismic_weight, Some(999.0));
    // 失敗したコマンド（Noop）は undo 履歴に積まれない。
    assert!(!stack.can_undo());
    assert_eq!(model.stories[0].seismic_weight, Some(999.0));
}

/// `ApplyStories` が剛床代表節点(`rep_nodes`/`generated_masters`)込みで適用され、
/// undo で節点数・`generated_masters`・stories・constraints が完全に元へ戻ること
/// （`eq_ignoring_dofmap` で比較）。redo も確認する。
#[test]
fn test_apply_stories_roundtrip_with_generated_masters() {
    use squid_n_core::dof::Dof;
    use squid_n_core::model::{Constraint, Story};

    let mut model = empty_model();
    for i in 0..2u32 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64 * 1000.0, 0.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    let before = model.clone();
    let mut stack = UndoStack::new();

    let mut rep_restraint = Dof6Mask::FREE;
    rep_restraint.set_fixed(Dof::Uz);
    rep_restraint.set_fixed(Dof::Rx);
    rep_restraint.set_fixed(Dof::Ry);
    let rep_node = Node {
        id: NodeId(2),
        coord: [500.0, 0.0, 3000.0],
        restraint: rep_restraint,
        mass: None,
        story: Some(StoryId(0)),
        support_spring: None,
    };
    let cmd = ApplyStories {
        stories: vec![Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(0),
            name: "1F".into(),
            elevation: 3000.0,
            node_ids: vec![NodeId(0), NodeId(1)],
            seismic_weight: Some(1000.0),
            weight_override: None,
        }],
        node_story: vec![Some(StoryId(0)), Some(StoryId(0))],
        constraints: vec![Constraint::rigid_diaphragm(
            StoryId(0),
            NodeId(2),
            vec![NodeId(0), NodeId(1)],
        )],
        rep_nodes: vec![rep_node],
        generated_masters: vec![NodeId(2)],
        mass_method: Default::default(),
    };

    stack.run(&mut model, Box::new(cmd));
    assert_eq!(model.nodes.len(), 3);
    assert_eq!(model.generated_masters, vec![NodeId(2)]);
    assert_eq!(model.stories.len(), 1);
    assert_eq!(model.constraints.len(), 1);
    assert!(model.validate().is_ok());

    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert!(model.generated_masters.is_empty());
    assert!(model.stories.is_empty());
    assert!(model.validate().is_ok());

    stack.redo(&mut model);
    assert_eq!(model.nodes.len(), 3);
    assert_eq!(model.generated_masters, vec![NodeId(2)]);
    assert_eq!(model.stories.len(), 1);
    assert_eq!(model.constraints.len(), 1);
}

/// `ApplyStories` が `model.mass_method` を設定し、undo で変更前の値へ復元されること。
#[test]
fn test_apply_stories_sets_and_restores_mass_method() {
    use squid_n_core::model::MassMethod;

    let mut model = empty_model();
    for i in 0..2u32 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64 * 1000.0, 0.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    // 変更前は既定（CorrectedLumped）。
    assert_eq!(model.mass_method, MassMethod::CorrectedLumped);
    let mut stack = UndoStack::new();

    let cmd = ApplyStories {
        stories: vec![],
        node_story: vec![None, None],
        constraints: vec![],
        rep_nodes: vec![],
        generated_masters: vec![],
        mass_method: MassMethod::LumpedOnly,
    };

    stack.run(&mut model, Box::new(cmd));
    assert_eq!(model.mass_method, MassMethod::LumpedOnly);

    stack.undo(&mut model);
    assert_eq!(model.mass_method, MassMethod::CorrectedLumped);

    stack.redo(&mut model);
    assert_eq!(model.mass_method, MassMethod::LumpedOnly);
}

/// 階数が減って不活性化された剛床代表節点（`generated_masters` には残るが
/// `restraint=FIXED`/`story=None`）の `DeleteNode` → undo(`InsertNode`) で
/// `generated_masters` が（ID 繰り上げ込みで）正しく維持されること。
#[test]
fn test_delete_leftover_generated_master_roundtrip() {
    let mut model = empty_model();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    });
    // 不活性化された旧代表節点(story_gen.rs の仕様どおり restraint=FIXED, story=None)。
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [3000.0, 0.0, 3000.0],
        restraint: Dof6Mask::FIXED,
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
    model.generated_masters = vec![NodeId(1)];
    model.elements.push(ElementData {
        id: squid_n_core::ids::ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(2)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });
    let before = model.clone();
    let mut stack = UndoStack::new();

    // 不活性節点は何にも参照されていないため削除できる。
    stack.run(&mut model, Box::new(DeleteNode { id: NodeId(1) }));
    assert_eq!(model.nodes.len(), 2);
    assert!(model.generated_masters.is_empty());
    // 旧 NodeId(2) だった部材参照は NodeId(1) に繰り上がる
    assert_eq!(model.elements[0].nodes[1], NodeId(1));
    assert!(model.validate().is_ok());

    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert_eq!(model.generated_masters, vec![NodeId(1)]);
    assert!(model.validate().is_ok());

    stack.redo(&mut model);
    assert_eq!(model.nodes.len(), 2);
    assert!(model.generated_masters.is_empty());
}

#[test]
fn test_set_load_case_kind_roundtrip() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::LoadCaseKind;
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(AddLoadCase { name: "DL".into() }));
    assert_eq!(model.load_cases[0].kind, LoadCaseKind::Other);

    stack.run(
        &mut model,
        Box::new(SetLoadCaseKind {
            id: LoadCaseId(0),
            kind: LoadCaseKind::Dead,
        }),
    );
    assert_eq!(model.load_cases[0].kind, LoadCaseKind::Dead);

    stack.undo(&mut model);
    assert_eq!(model.load_cases[0].kind, LoadCaseKind::Other);

    stack.redo(&mut model);
    assert_eq!(model.load_cases[0].kind, LoadCaseKind::Dead);
}

#[test]
fn test_set_load_case_kind_invalid_id_is_noop() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::LoadCaseKind;
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetLoadCaseKind {
            id: LoadCaseId(0),
            kind: LoadCaseKind::Dead,
        }),
    );
    assert!(model.load_cases.is_empty());
}

/// 1 つの節点に複数の節点荷重を定義でき、追加・変更・削除が添字で行える。
/// undo は追加した順序ごと元へ戻す。
#[test]
fn test_nodal_loads_are_indexed_and_allow_duplicates_per_node() {
    use squid_n_core::model::{LoadCaseKind, NodalLoad};
    let mut model = seeded_model(4, 0);
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(AddLoadCase { name: "LC0".into() }));

    let mut first = NodalLoad::manual(NodeId(3), [100.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    first.name = "機器荷重".into();
    let mut second = NodalLoad::manual(NodeId(3), [0.0, 0.0, -50.0, 0.0, 0.0, 0.0]);
    second.name = "吊り荷重".into();
    for load in [first.clone(), second.clone()] {
        stack.run(
            &mut model,
            Box::new(AddNodalLoad {
                lc: LoadCaseId(0),
                load,
            }),
        );
    }
    // 同じ節点でも 2 件が独立して並ぶ（旧仕様は節点キーの upsert で 1 件に潰れた）。
    assert_eq!(
        model.load_cases[0].nodal,
        vec![first.clone(), second.clone()]
    );

    // 添字指定の変更は対象 1 件だけを差し替える。
    let mut edited = second.clone();
    edited.values[2] = -80.0;
    stack.run(
        &mut model,
        Box::new(SetNodalLoad {
            lc: LoadCaseId(0),
            index: 1,
            load: edited.clone(),
        }),
    );
    assert_eq!(model.load_cases[0].nodal, vec![first.clone(), edited]);

    // 削除 → undo で同じ位置へ戻る。
    stack.run(
        &mut model,
        Box::new(DeleteNodalLoad {
            lc: LoadCaseId(0),
            index: 0,
        }),
    );
    assert_eq!(model.load_cases[0].nodal.len(), 1);
    stack.undo(&mut model);
    assert_eq!(model.load_cases[0].nodal[0], first);

    // 種別は触っていないので既定のまま。
    assert_eq!(model.load_cases[0].kind, LoadCaseKind::Other);
}

/// 準備計算が生成した荷重は編集・削除できない（コマンドが Noop を返す）。
/// UI 側でメニューを出さないだけでなく、コマンド層でも守る。
#[test]
fn test_auto_loads_reject_edit_and_delete() {
    use squid_n_core::model::NodalLoad;
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(AddLoadCase { name: "DL".into() }));
    let auto = NodalLoad::auto(NodeId(0), [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]);
    // 自動生成分は同期（`replace_auto_loads`）が入れるものなので、そちらで用意する。
    model.load_cases[0].replace_auto_loads(vec![auto.clone()], Vec::new());

    stack.run(
        &mut model,
        Box::new(SetNodalLoad {
            lc: LoadCaseId(0),
            index: 0,
            load: NodalLoad::manual(NodeId(0), [999.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        }),
    );
    assert_eq!(
        model.load_cases[0].nodal,
        vec![auto.clone()],
        "変更できない"
    );

    stack.run(
        &mut model,
        Box::new(DeleteNodalLoad {
            lc: LoadCaseId(0),
            index: 0,
        }),
    );
    assert_eq!(
        model.load_cases[0].nodal,
        vec![auto.clone()],
        "削除できない"
    );

    // 追加も拒む。ここで受け付けると、逆操作の削除が拒む側の規則と食い違い、
    // undo が無言で効かなくなる。
    let added = stack.run(
        &mut model,
        Box::new(AddNodalLoad {
            lc: LoadCaseId(0),
            load: NodalLoad::auto(NodeId(1), [0.0, 0.0, -1.0, 0.0, 0.0, 0.0]),
        }),
    );
    assert!(!added, "自動生成分は追加コマンドで積めない");
    assert_eq!(model.load_cases[0].nodal, vec![auto], "件数が増えない");
}

#[test]
fn test_sync_slab_loads_to_case_creates_new_case() {
    use squid_n_core::ids::{ElemId, LoadCaseId};
    use squid_n_core::model::{LoadCaseKind, MemberLoad, MemberLoadKind, NodalLoad};
    let mut model = seeded_model(2, 1);
    let mut stack = UndoStack::new();

    let member = vec![MemberLoad::auto(
        ElemId(0),
        [0.0, 0.0, -1.0],
        MemberLoadKind::Distributed {
            a: 0.0,
            b: 1000.0,
            w1: 1.0,
            w2: 1.0,
        },
    )];
    let nodal = vec![NodalLoad::auto(NodeId(0), [0.0, 0.0, -5.0, 0.0, 0.0, 0.0])];

    stack.run(
        &mut model,
        Box::new(SyncSlabLoadsToCase {
            name: "床荷重(自動)".into(),
            kind: LoadCaseKind::Dead,
            nodal: nodal.clone(),
            member: member.clone(),
        }),
    );
    assert_eq!(model.load_cases.len(), 1);
    assert_eq!(model.load_cases[0].id, LoadCaseId(0));
    assert_eq!(model.load_cases[0].name, "床荷重(自動)");
    assert_eq!(model.load_cases[0].kind, LoadCaseKind::Dead);
    assert_eq!(model.load_cases[0].member, member);
    assert_eq!(model.load_cases[0].nodal, nodal);

    // undo → 新規作成したケースごと消える(DeleteLoadCase を再利用した逆操作)。
    stack.undo(&mut model);
    assert!(model.load_cases.is_empty());

    stack.redo(&mut model);
    assert_eq!(model.load_cases.len(), 1);
}

#[test]
fn test_sync_slab_loads_to_case_keeps_manual_and_replaces_auto() {
    use squid_n_core::ids::ElemId;
    use squid_n_core::model::{LoadCaseKind, MemberLoad, MemberLoadKind};
    let mut model = seeded_model(3, 2);
    let mut stack = UndoStack::new();

    // 既存の同名ケースに、利用者が手で入れた荷重が 1 件ある状態。
    stack.run(
        &mut model,
        Box::new(AddLoadCase {
            name: "床荷重(自動)".into(),
        }),
    );
    let manual = MemberLoad::manual(
        ElemId(0),
        [0.0, 0.0, -1.0],
        MemberLoadKind::Distributed {
            a: 0.0,
            b: 500.0,
            w1: 9.0,
            w2: 9.0,
        },
    );
    stack.run(
        &mut model,
        Box::new(AddMemberLoad {
            lc: LoadCaseId(0),
            load: manual.clone(),
        }),
    );
    let before = model.clone();
    assert_eq!(model.load_cases[0].member.len(), 1);

    let auto_member = vec![MemberLoad::auto(
        ElemId(1),
        [0.0, 0.0, -1.0],
        MemberLoadKind::Distributed {
            a: 0.0,
            b: 2000.0,
            w1: 3.0,
            w2: 3.0,
        },
    )];
    stack.run(
        &mut model,
        Box::new(SyncSlabLoadsToCase {
            name: "床荷重(自動)".into(),
            kind: LoadCaseKind::Dead,
            nodal: Vec::new(),
            member: auto_member.clone(),
        }),
    );
    // 手入力は残り、自動生成分が後ろに付く。
    assert_eq!(model.load_cases.len(), 1);
    assert_eq!(
        model.load_cases[0].member,
        vec![manual.clone(), auto_member[0].clone()]
    );
    assert_eq!(model.load_cases[0].kind, LoadCaseKind::Dead);

    // 再同期しても自動生成分は置き換わるだけで増えない（冪等）。
    stack.run(
        &mut model,
        Box::new(SyncSlabLoadsToCase {
            name: "床荷重(自動)".into(),
            kind: LoadCaseKind::Dead,
            nodal: Vec::new(),
            member: auto_member.clone(),
        }),
    );
    assert_eq!(model.load_cases[0].member.len(), 2);
    assert_eq!(model.load_cases[0].member[0], manual);

    // undo を2回 → 元の手動入力だけの状態に戻る。
    stack.undo(&mut model);
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
}

/// 手入力扱いの荷重を同期内容として渡しても、自動生成分として積まれる
/// （手入力のまま積むと次の同期で消えず、実行のたびに荷重が増え続ける）。
#[test]
fn test_sync_slab_loads_marks_content_as_auto() {
    use squid_n_core::ids::ElemId;
    use squid_n_core::model::{LoadCaseKind, MemberLoad, MemberLoadKind};
    let mut model = seeded_model(2, 1);
    let mut stack = UndoStack::new();

    let content = vec![MemberLoad::manual(
        ElemId(0),
        [0.0, 0.0, -1.0],
        MemberLoadKind::Distributed {
            a: 0.0,
            b: 1000.0,
            w1: 2.0,
            w2: 2.0,
        },
    )];
    for _ in 0..3 {
        stack.run(
            &mut model,
            Box::new(SyncSlabLoadsToCase {
                name: "床荷重(自動)".into(),
                kind: LoadCaseKind::Dead,
                nodal: Vec::new(),
                member: content.clone(),
            }),
        );
    }
    assert_eq!(
        model.load_cases[0].member.len(),
        1,
        "同期のたびに増えてはいけない"
    );
    assert!(model.load_cases[0].member[0].source.is_auto());
}

#[test]
fn test_set_load_cfg_roundtrip() {
    use squid_n_core::model::LoadCfg;
    let mut model = empty_model();
    assert!(model.load_cfg.is_none());
    let mut stack = UndoStack::new();

    let cfg = LoadCfg {
        steel_weight_factor: 1.05,
        ..Default::default()
    };
    stack.run(
        &mut model,
        Box::new(SetLoadCfg {
            cfg: Some(cfg.clone()),
        }),
    );
    assert_eq!(model.load_cfg, Some(cfg));

    stack.undo(&mut model);
    assert!(model.load_cfg.is_none());

    stack.redo(&mut model);
    assert_eq!(model.load_cfg.as_ref().unwrap().steel_weight_factor, 1.05);
}

#[test]
fn test_set_wall_attr_add_replace_and_remove_roundtrip() {
    use squid_n_core::model::WallAttr;
    let mut model = seeded_model(2, 1);
    let mut stack = UndoStack::new();

    let attr1 = WallAttr {
        elem: ElemId(0),
        opening_area: 100.0,
        opening_weight: 50.0,
        three_side_slit: false,
        openings: vec![],
    };
    stack.run(
        &mut model,
        Box::new(SetWallAttr {
            attr: attr1.clone(),
        }),
    );
    assert_eq!(model.wall_attrs, vec![attr1.clone()]);

    // 既存エントリを置換
    let attr2 = WallAttr {
        elem: ElemId(0),
        opening_area: 200.0,
        opening_weight: 80.0,
        three_side_slit: true,
        openings: vec![],
    };
    stack.run(
        &mut model,
        Box::new(SetWallAttr {
            attr: attr2.clone(),
        }),
    );
    assert_eq!(model.wall_attrs, vec![attr2.clone()]);

    stack.undo(&mut model);
    assert_eq!(model.wall_attrs, vec![attr1.clone()]);

    // 削除
    stack.run(&mut model, Box::new(RemoveWallAttr { elem: ElemId(0) }));
    assert!(model.wall_attrs.is_empty());

    stack.undo(&mut model);
    assert_eq!(model.wall_attrs, vec![attr1]);
}

#[test]
fn test_remove_wall_attr_missing_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(RemoveWallAttr { elem: ElemId(0) }));
    assert!(model.wall_attrs.is_empty());
    // 失敗したコマンド（Noop）は undo 履歴に積まれない。
    assert!(!stack.can_undo());
    assert!(model.wall_attrs.is_empty());
}

fn sample_misc_wall(weight: f64) -> squid_n_core::model::MiscWall {
    squid_n_core::model::MiscWall {
        start: [0.0, 0.0, 0.0],
        end: [3000.0, 0.0, 0.0],
        height: 3000.0,
        weight_per_area: weight,
        transfer: Default::default(),
        thickness: None,
    }
}

#[test]
fn test_add_delete_misc_wall_roundtrip() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(AddMiscWall {
            wall: sample_misc_wall(1.0),
        }),
    );
    stack.run(
        &mut model,
        Box::new(AddMiscWall {
            wall: sample_misc_wall(2.0),
        }),
    );
    assert_eq!(model.misc_walls.len(), 2);

    let before = model.clone();
    stack.run(&mut model, Box::new(DeleteMiscWall { index: 0 }));
    assert_eq!(model.misc_walls.len(), 1);
    assert_eq!(model.misc_walls[0].weight_per_area, 2.0);

    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));

    stack.redo(&mut model);
    assert_eq!(model.misc_walls.len(), 1);
}

#[test]
fn test_set_misc_wall_roundtrip() {
    let mut model = empty_model();
    model.misc_walls.push(sample_misc_wall(1.0));
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetMiscWall {
            index: 0,
            wall: sample_misc_wall(9.0),
        }),
    );
    assert_eq!(model.misc_walls[0].weight_per_area, 9.0);

    stack.undo(&mut model);
    assert_eq!(model.misc_walls[0].weight_per_area, 1.0);
}

#[test]
fn test_set_misc_wall_out_of_range_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetMiscWall {
            index: 0,
            wall: sample_misc_wall(1.0),
        }),
    );
    assert!(model.misc_walls.is_empty());
}

#[test]
fn test_set_story_level_kind_roundtrip() {
    use squid_n_core::model::StoryLevelKind;
    let mut model = empty_model();
    model.stories.push(make_story(0, None));
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetStoryLevelKind {
            story: StoryId(0),
            level_kind: StoryLevelKind::Penthouse { k: 0.5 },
        }),
    );
    assert_eq!(
        model.stories[0].level_kind,
        StoryLevelKind::Penthouse { k: 0.5 }
    );

    stack.undo(&mut model);
    assert_eq!(model.stories[0].level_kind, StoryLevelKind::Normal);
}

#[test]
fn test_set_slab_joists_roundtrip() {
    use squid_n_core::model::JoistLine;
    let mut model = seeded_model(4, 0);
    model
        .floor_regions
        .push(squid_n_core::model::FloorRegion::new(
            FloorRegionId(0),
            vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        ));
    let mut stack = UndoStack::new();
    assert!(model.floor_regions[0].joists.is_empty());

    let joists = vec![JoistLine {
        dir: [0.0, 1.0],
        spacing: 900.0,
        support: [NodeId(0), NodeId(3)],
        section: None,
        pinned_onto: None,
    }];
    stack.run(
        &mut model,
        Box::new(SetFloorRegionJoists {
            id: FloorRegionId(0),
            joists: joists.clone(),
        }),
    );
    assert_eq!(model.floor_regions[0].joists, joists);

    // undo で元の空 joists に戻る（対称逆操作）。
    stack.undo(&mut model);
    assert!(model.floor_regions[0].joists.is_empty());
    stack.redo(&mut model);
    assert_eq!(model.floor_regions[0].joists, joists);

    // 存在しない FloorRegionId は Noop（モデル不変・undo スタックも安全）。
    stack.run(
        &mut model,
        Box::new(SetFloorRegionJoists {
            id: FloorRegionId(99),
            joists: vec![],
        }),
    );
    assert_eq!(model.floor_regions[0].joists, joists);
}

#[test]
fn test_materialize_slab_joists_creates_beams() {
    use squid_n_core::model::{ElementKind, EndCondition, JoistLine};
    let mut model = seeded_model(4, 0);
    let mut region = squid_n_core::model::FloorRegion::new(
        FloorRegionId(0),
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
    );
    region.joists = vec![JoistLine {
        dir: [0.0, 1.0],
        spacing: 900.0,
        support: [NodeId(0), NodeId(3)],
        section: None,
        pinned_onto: None,
    }];
    model.floor_regions.push(region);
    let mut stack = UndoStack::new();
    let before = model.elements.len();

    stack.run(
        &mut model,
        Box::new(MaterializeSlabJoists {
            slab: FloorRegionId(0),
        }),
    );
    assert_eq!(model.elements.len(), before + 1, "小梁1本が実部材化される");
    let beam = model.elements.last().unwrap();
    assert_eq!(beam.kind, ElementKind::Beam);
    assert_eq!(beam.nodes.len(), 2);
    assert!(
        (beam.nodes[0] == NodeId(0) && beam.nodes[1] == NodeId(3))
            || (beam.nodes[0] == NodeId(3) && beam.nodes[1] == NodeId(0)),
        "支持2節点を両端に持つ"
    );
    assert_eq!(
        beam.end_cond,
        [EndCondition::Pinned, EndCondition::Pinned],
        "小梁は両端ピン"
    );

    // 再実行しても既存の実部材があるので新規生成しない（冪等）。
    stack.run(
        &mut model,
        Box::new(MaterializeSlabJoists {
            slab: FloorRegionId(0),
        }),
    );
    assert_eq!(
        model.elements.len(),
        before + 1,
        "実部材化済みは重複生成しない"
    );

    // undo で生成した実部材が末尾から除去される。
    stack.undo(&mut model); // 2回目（冪等 no-op）を戻す
    assert_eq!(model.elements.len(), before + 1);
    stack.undo(&mut model); // 1回目の生成を戻す
    assert_eq!(model.elements.len(), before, "undo で実部材化を取り消す");
}

#[test]
fn test_set_multi_opening_mode_roundtrip() {
    use squid_n_core::model::MultiOpeningMode;
    let mut model = empty_model();
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Equivalent);
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetMultiOpeningMode {
            mode: MultiOpeningMode::Envelope,
        }),
    );
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Envelope);

    stack.run(
        &mut model,
        Box::new(SetMultiOpeningMode {
            mode: MultiOpeningMode::Auto,
        }),
    );
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Auto);

    stack.undo(&mut model);
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Envelope);
    stack.undo(&mut model);
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Equivalent);

    stack.redo(&mut model);
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Envelope);
    stack.redo(&mut model);
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Auto);
}

/// 同じモードへの再設定でも既存の値置換系コマンドと同様に処理される
/// （同値判定による分岐なし。undo すれば必ず変更前の値へ戻る）。
#[test]
fn test_set_multi_opening_mode_same_value_is_symmetric() {
    use squid_n_core::model::MultiOpeningMode;
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    stack.run(
        &mut model,
        Box::new(SetMultiOpeningMode {
            mode: MultiOpeningMode::Equivalent,
        }),
    );
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Equivalent);

    stack.undo(&mut model);
    assert_eq!(model.multi_opening_mode, MultiOpeningMode::Equivalent);
}

#[test]
fn test_set_member_hysteresis_roundtrip() {
    use squid_n_core::model::HysteresisModel;
    let mut model = empty_model();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0; 3],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [1000.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(1)],
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
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetMemberHysteresis {
            elem: ElemId(0),
            rule: HysteresisModel::Takeda,
        }),
    );
    assert_eq!(
        model.member_hysteresis(ElemId(0)),
        Some(HysteresisModel::Takeda)
    );
    stack.undo(&mut model);
    assert_eq!(model.member_hysteresis(ElemId(0)), None);
    stack.redo(&mut model);
    assert_eq!(
        model.member_hysteresis(ElemId(0)),
        Some(HysteresisModel::Takeda)
    );

    // 存在しない部材は Noop。
    let mut stack2 = UndoStack::new();
    stack2.run(
        &mut model,
        Box::new(SetMemberHysteresis {
            elem: ElemId(99),
            rule: HysteresisModel::Standard,
        }),
    );
    assert_eq!(model.member_hysteresis(ElemId(99)), None);
}

#[test]
fn test_set_member_hysteresis_th_roundtrip() {
    use squid_n_core::model::HysteresisModel;
    let mut model = two_member_model();
    let mut stack = UndoStack::new();

    // 増分用スロットを Takeda に設定しておき、時刻歴用スロットの操作で
    // 影響を受けないことを確認する。
    stack.run(
        &mut model,
        Box::new(SetMemberHysteresis {
            elem: ElemId(0),
            rule: HysteresisModel::Takeda,
        }),
    );
    assert_eq!(model.member_hysteresis_th_raw(ElemId(0)), None);

    stack.run(
        &mut model,
        Box::new(SetMemberHysteresisTh {
            elem: ElemId(0),
            rule_th: Some(HysteresisModel::KarsanJirsa),
        }),
    );
    assert_eq!(
        model.member_hysteresis_th_raw(ElemId(0)),
        Some(HysteresisModel::KarsanJirsa)
    );
    // 増分用スロットは影響を受けない。
    assert_eq!(
        model.member_hysteresis(ElemId(0)),
        Some(HysteresisModel::Takeda)
    );

    stack.undo(&mut model);
    assert_eq!(model.member_hysteresis_th_raw(ElemId(0)), None);
    assert_eq!(
        model.member_hysteresis(ElemId(0)),
        Some(HysteresisModel::Takeda)
    );

    stack.redo(&mut model);
    assert_eq!(
        model.member_hysteresis_th_raw(ElemId(0)),
        Some(HysteresisModel::KarsanJirsa)
    );

    // 存在しない部材は Noop。
    let mut stack2 = UndoStack::new();
    stack2.run(
        &mut model,
        Box::new(SetMemberHysteresisTh {
            elem: ElemId(99),
            rule_th: Some(HysteresisModel::OriginOriented),
        }),
    );
    assert_eq!(model.member_hysteresis_th_raw(ElemId(99)), None);
}

#[test]
fn test_add_damper_creates_element_and_attr_roundtrip() {
    use squid_n_core::model::{DamperKind, DamperProps};
    let mut model = two_member_model();
    let before = model.clone();
    let new_id = ElemId(model.elements.len() as u32);
    let props = DamperProps {
        kind: DamperKind::Maxwell,
        kd: 120_000.0,
        c0: 2_000.0,
        alpha: 0.5,
        ..Default::default()
    };
    let elem = ElementData {
        id: new_id,
        kind: ElementKind::Damper,
        nodes: smallvec![NodeId(0), NodeId(2)],
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
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(AddDamper { elem, props }));
    // 要素と特性が原子的に追加される。
    assert_eq!(model.elements.len(), 3);
    assert_eq!(model.elements[2].kind, ElementKind::Damper);
    assert_eq!(model.damper_props(new_id), Some(props));

    // undo で要素・特性ともに消える（完全復元）。
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert_eq!(model.damper_props(new_id), None);

    // redo で再生成。
    stack.redo(&mut model);
    assert_eq!(model.damper_props(new_id), Some(props));
}

#[test]
fn test_set_damper_props_roundtrip() {
    use squid_n_core::model::DamperProps;
    let mut model = two_member_model();
    let e = ElemId(1);
    let p1 = DamperProps {
        kd: 100.0,
        c0: 10.0,
        alpha: 1.0,
        ..Default::default()
    };
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(SetDamperProps {
            elem: e,
            props: Some(p1),
        }),
    );
    assert_eq!(model.damper_props(e), Some(p1));
    // 解除。
    stack.run(
        &mut model,
        Box::new(SetDamperProps {
            elem: e,
            props: None,
        }),
    );
    assert_eq!(model.damper_props(e), None);
    stack.undo(&mut model);
    assert_eq!(model.damper_props(e), Some(p1));
    stack.undo(&mut model);
    assert_eq!(model.damper_props(e), None);

    // 存在しない部材は Noop。
    let mut stack2 = UndoStack::new();
    stack2.run(
        &mut model,
        Box::new(SetDamperProps {
            elem: ElemId(99),
            props: Some(p1),
        }),
    );
    assert_eq!(model.damper_props(ElemId(99)), None);
}

#[test]
fn test_add_isolator_creates_element_and_attr_roundtrip() {
    use squid_n_core::model::{IsolatorKind, IsolatorProps};
    let mut model = two_member_model();
    let before = model.clone();
    let new_id = ElemId(model.elements.len() as u32);
    let props = IsolatorProps {
        kind: IsolatorKind::LeadRubber,
        k1: 2000.0,
        k2: 200.0,
        qd: 100_000.0,
        kv: 5_000_000.0,
        ..Default::default()
    };
    let elem = ElementData {
        id: new_id,
        kind: ElementKind::Isolator,
        nodes: smallvec![NodeId(0), NodeId(2)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(AddIsolator { elem, props }));
    // 要素と特性が原子的に追加される。
    assert_eq!(model.elements.len(), 3);
    assert_eq!(model.elements[2].kind, ElementKind::Isolator);
    assert_eq!(
        model
            .isolator_attrs
            .iter()
            .find(|a| a.elem == new_id)
            .map(|a| a.props),
        Some(props)
    );

    // undo で要素・特性ともに消える（完全復元）。
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert!(!model.isolator_attrs.iter().any(|a| a.elem == new_id));

    // redo で再生成。
    stack.redo(&mut model);
    assert_eq!(
        model
            .isolator_attrs
            .iter()
            .find(|a| a.elem == new_id)
            .map(|a| a.props),
        Some(props)
    );
}

/// `AddIsolator`: `elem.id` が `model.elements.len()`（末尾の次）と一致しない場合は
/// ID＝配列インデックスの不変条件を壊すため Noop になること。
#[test]
fn test_add_isolator_id_mismatch_is_noop() {
    use squid_n_core::model::{IsolatorKind, IsolatorProps};
    let mut model = two_member_model();
    let before = model.clone();
    let props = IsolatorProps {
        kind: IsolatorKind::LeadRubber,
        ..Default::default()
    };
    // 末尾の次（あるべき ID）ではなく、既存の 0 番を指定してしまったケース。
    let elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Isolator,
        nodes: smallvec![NodeId(0), NodeId(2)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(AddIsolator { elem, props }));
    assert!(
        model.eq_ignoring_dofmap(&before),
        "elem.id が末尾でない AddIsolator は Noop のはず"
    );
}

#[test]
fn test_place_support_isolator_roundtrip() {
    use squid_n_core::model::{ElementKind, IsolatorKind, IsolatorProps};
    let mut model = empty_model();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [1000.0, 2000.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    });
    let before = model.clone();

    let props = IsolatorProps {
        kind: IsolatorKind::LaminatedRubber,
        k1: 1500.0,
        k2: 150.0,
        qd: 80_000.0,
        kv: 3_000_000.0,
        ..Default::default()
    };
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(PlaceSupportIsolator {
            node: NodeId(0),
            props,
        }),
    );

    // 接地節点（FIXED、対象節点と同一座標）が新規作成される。
    assert_eq!(model.nodes.len(), 2);
    let ground = &model.nodes[1];
    assert_eq!(ground.restraint, Dof6Mask::FIXED);
    assert_eq!(ground.coord, [1000.0, 2000.0, 0.0]);
    // 対象節点の restraint は FREE に解放される。
    assert_eq!(model.nodes[0].restraint, Dof6Mask::FREE);
    // 零長 Isolator 要素が i端=接地節点・j端=対象節点で追加される。
    assert_eq!(model.elements.len(), 1);
    assert_eq!(model.elements[0].kind, ElementKind::Isolator);
    assert_eq!(model.elements[0].nodes[0], NodeId(1));
    assert_eq!(model.elements[0].nodes[1], NodeId(0));
    assert_eq!(
        model
            .isolator_attrs
            .iter()
            .find(|a| a.elem == ElemId(0))
            .map(|a| a.props),
        Some(props)
    );

    // undo で節点削除まで含めて完全に元へ戻ること。
    stack.undo(&mut model);
    assert!(
        model.eq_ignoring_dofmap(&before),
        "undo should fully restore the original model (including the ground node)"
    );

    // redo で再度設置される。
    stack.redo(&mut model);
    assert_eq!(model.nodes.len(), 2);
    assert_eq!(model.nodes[0].restraint, Dof6Mask::FREE);
    assert_eq!(model.elements.len(), 1);
}

/// 配置（`PlaceSupportIsolator`）→撤去（`RemoveSupportIsolator`）で、接地節点・
/// 要素・特性が完全に消え、対象節点の拘束が FIXED（撤去仕様。配置前拘束は
/// 記録しないため常に FIXED）へ戻り、配置前と `eq_ignoring_dofmap` で一致すること。
#[test]
fn test_remove_support_isolator_roundtrip() {
    use squid_n_core::model::{ElementKind, IsolatorKind, IsolatorProps};
    let mut model = empty_model();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [1000.0, 2000.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    });
    let before = model.clone();

    let props = IsolatorProps {
        kind: IsolatorKind::LaminatedRubber,
        k1: 1500.0,
        k2: 150.0,
        qd: 80_000.0,
        kv: 3_000_000.0,
        ..Default::default()
    };
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(PlaceSupportIsolator {
            node: NodeId(0),
            props,
        }),
    );
    let placed = model.clone();
    assert_eq!(model.nodes.len(), 2);
    assert_eq!(model.elements.len(), 1);

    // 撤去: 接地節点・要素・特性が消え、対象節点の拘束は FIXED（撤去仕様）へ戻る。
    stack.run(
        &mut model,
        Box::new(RemoveSupportIsolator { node: NodeId(0) }),
    );
    assert!(
        model.eq_ignoring_dofmap(&before),
        "撤去後は配置前と一致するはず（接地節点・要素・特性が完全に消えること）"
    );
    assert_eq!(model.nodes[0].restraint, Dof6Mask::FIXED);

    // 撤去→undo で配置状態に戻ること（拘束も FREE に戻る＝配置直後の状態と一致）。
    stack.undo(&mut model);
    assert!(
        model.eq_ignoring_dofmap(&placed),
        "撤去の undo は配置直後の状態に完全復元するはず"
    );
    assert_eq!(model.nodes[0].restraint, Dof6Mask::FREE);
    assert_eq!(model.elements[0].kind, ElementKind::Isolator);

    // 撤去→undo→redo で再度撤去された状態に戻ること。
    stack.redo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert_eq!(model.nodes[0].restraint, Dof6Mask::FIXED);
}

/// 通常の（支点ではない）免震要素・存在しない節点では `RemoveSupportIsolator` は
/// Noop になること（`DeleteMember` を使うべきケースを誤って壊さない）。
#[test]
fn test_remove_support_isolator_noop_when_not_support_isolator() {
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, IsolatorProps, LocalAxis,
    };
    let mut model = empty_model();
    model.nodes.push(Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    model.nodes.push(Node {
        id: NodeId(1),
        coord: [1000.0, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    // 通常の（2つの構造節点間の）免震要素。零長ではないため支点免震要素の形を満たさない。
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Isolator,
        nodes: smallvec![NodeId(0), NodeId(1)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });
    model
        .isolator_attrs
        .push(squid_n_core::model::IsolatorAttr {
            elem: ElemId(0),
            props: IsolatorProps::default(),
        });
    let before = model.clone();

    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(RemoveSupportIsolator { node: NodeId(0) }),
    );
    assert!(
        model.eq_ignoring_dofmap(&before),
        "通常の免震要素は Noop のはず"
    );

    // 存在しない節点も Noop。
    stack.run(
        &mut model,
        Box::new(RemoveSupportIsolator { node: NodeId(99) }),
    );
    assert!(model.eq_ignoring_dofmap(&before));
}

#[test]
fn test_place_support_isolator_invalid_node_is_noop() {
    use squid_n_core::model::IsolatorProps;
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(PlaceSupportIsolator {
            node: NodeId(99),
            props: IsolatorProps::default(),
        }),
    );
    assert!(model.nodes.is_empty());
    assert!(model.elements.is_empty());
    stack.undo(&mut model);
    assert!(model.nodes.is_empty());
}

#[test]
fn test_damper_def_add_update_remove_roundtrip() {
    use squid_n_core::model::{DamperDef, DamperKind, DamperProps};
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    let def1 = DamperDef {
        name: "オイルダンパー A".to_string(),
        props: DamperProps {
            kind: DamperKind::Maxwell,
            kd: 100_000.0,
            c0: 1_000.0,
            alpha: 1.0,
            ..Default::default()
        },
    };
    stack.run(&mut model, Box::new(AddDamperDef { def: def1.clone() }));
    assert_eq!(model.damper_defs.len(), 1);
    assert_eq!(model.damper_defs[0], def1);

    let def1_updated = DamperDef {
        name: "オイルダンパー A（改）".to_string(),
        props: DamperProps {
            kd: 120_000.0,
            ..def1.props
        },
    };
    stack.run(
        &mut model,
        Box::new(UpdateDamperDef {
            index: 0,
            def: def1_updated.clone(),
        }),
    );
    assert_eq!(model.damper_defs[0], def1_updated);

    stack.undo(&mut model);
    assert_eq!(model.damper_defs[0], def1);

    stack.redo(&mut model);
    assert_eq!(model.damper_defs[0], def1_updated);

    // 削除→undo で同じ位置へ復元。
    let before_remove = model.clone();
    stack.run(&mut model, Box::new(RemoveDamperDef { index: 0 }));
    assert!(model.damper_defs.is_empty());
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before_remove));
    assert_eq!(model.damper_defs[0], def1_updated);

    // 範囲外の index は Noop。
    let mut stack2 = UndoStack::new();
    stack2.run(
        &mut model,
        Box::new(UpdateDamperDef {
            index: 99,
            def: def1,
        }),
    );
    assert_eq!(model.damper_defs.len(), 1);
    stack2.run(&mut model, Box::new(RemoveDamperDef { index: 99 }));
    assert_eq!(model.damper_defs.len(), 1);
}

#[test]
fn test_damper_def_removal_does_not_affect_assigned_member() {
    // DamperDef は値コピーで割り当てるため、定義の削除は既存の割当済み部材
    // （Model::damper_attrs）に影響しない。
    use squid_n_core::model::{DamperDef, DamperKind, DamperProps};
    let mut model = two_member_model();
    let def = DamperDef {
        name: "テスト用".to_string(),
        props: DamperProps {
            kind: DamperKind::Maxwell,
            kd: 50_000.0,
            c0: 500.0,
            alpha: 1.0,
            ..Default::default()
        },
    };
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(AddDamperDef { def: def.clone() }));
    // 部材へは props の値コピーで割り当てる（参照は持たない）。
    stack.run(
        &mut model,
        Box::new(SetDamperProps {
            elem: ElemId(0),
            props: Some(def.props),
        }),
    );
    assert_eq!(model.damper_props(ElemId(0)), Some(def.props));

    stack.run(&mut model, Box::new(RemoveDamperDef { index: 0 }));
    assert!(model.damper_defs.is_empty());
    // 定義を削除しても、既に割り当て済みの部材の特性は壊れない。
    assert_eq!(model.damper_props(ElemId(0)), Some(def.props));
}

#[test]
fn test_set_member_detail_attr_add_replace_and_remove_roundtrip() {
    use squid_n_core::model::{Haunch, JointKind, MemberDetailAttr, MemberJoint};
    let mut model = seeded_model(2, 1);
    let mut stack = UndoStack::new();

    let attr1 = MemberDetailAttr {
        elem: ElemId(0),
        haunch_i: Some(Haunch {
            length: 700.0,
            depth_increase: 200.0,
            width_increase: 0.0,
        }),
        haunch_j: None,
        joints: vec![],
    };
    stack.run(
        &mut model,
        Box::new(SetMemberDetailAttr {
            attr: attr1.clone(),
        }),
    );
    assert_eq!(model.member_detail_attrs, vec![attr1.clone()]);

    stack.undo(&mut model);
    assert!(model.member_detail_attrs.is_empty());

    stack.redo(&mut model);
    assert_eq!(model.member_detail_attrs, vec![attr1.clone()]);

    // 既存エントリを置換
    let attr2 = MemberDetailAttr {
        elem: ElemId(0),
        haunch_i: None,
        haunch_j: Some(Haunch {
            length: 500.0,
            depth_increase: 150.0,
            width_increase: 0.0,
        }),
        joints: vec![MemberJoint {
            distance: 1000.0,
            kind: JointKind::Shop,
        }],
    };
    stack.run(
        &mut model,
        Box::new(SetMemberDetailAttr {
            attr: attr2.clone(),
        }),
    );
    assert_eq!(model.member_detail_attrs, vec![attr2.clone()]);

    stack.undo(&mut model);
    assert_eq!(model.member_detail_attrs, vec![attr1.clone()]);

    // 削除
    stack.run(
        &mut model,
        Box::new(RemoveMemberDetailAttr { elem: ElemId(0) }),
    );
    assert!(model.member_detail_attrs.is_empty());

    stack.undo(&mut model);
    assert_eq!(model.member_detail_attrs, vec![attr1]);
}

#[test]
fn test_remove_member_detail_attr_missing_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(RemoveMemberDetailAttr { elem: ElemId(0) }),
    );
    assert!(model.member_detail_attrs.is_empty());
    // 失敗したコマンド（Noop）は undo 履歴に積まれない。
    assert!(!stack.can_undo());
    assert!(model.member_detail_attrs.is_empty());
}

#[test]
fn test_set_steel_design_attr_add_replace_and_remove_roundtrip() {
    use squid_n_core::model::SteelDesignAttr;
    let mut model = seeded_model(3, 2);
    let mut stack = UndoStack::new();

    let attr1 = SteelDesignAttr {
        elem: ElemId(0),
        joint_flange_loss: 10.0,
        joint_web_loss: 0.0,
        scallop_web_loss: 0.0,
        lb_direct: None,
        lateral_brace_count: None,
        lk_y_direct: None,
        lk_z_direct: None,
        c_direct: None,
    };
    stack.run(
        &mut model,
        Box::new(SetSteelDesignAttr {
            attr: attr1.clone(),
        }),
    );
    assert_eq!(model.steel_design_attrs, vec![attr1.clone()]);

    stack.undo(&mut model);
    assert!(model.steel_design_attrs.is_empty());

    stack.redo(&mut model);
    assert_eq!(model.steel_design_attrs, vec![attr1.clone()]);

    // 既存エントリを置換（座屈長さの直接入力を追加）。
    let attr2 = SteelDesignAttr {
        elem: ElemId(0),
        joint_flange_loss: 0.0,
        joint_web_loss: 0.0,
        scallop_web_loss: 20.0,
        lb_direct: Some((1000.0, 2000.0, 3000.0)),
        lateral_brace_count: Some(3),
        lk_y_direct: Some(3500.0),
        lk_z_direct: Some(1750.0),
        c_direct: Some(1.5),
    };
    stack.run(
        &mut model,
        Box::new(SetSteelDesignAttr {
            attr: attr2.clone(),
        }),
    );
    assert_eq!(model.steel_design_attrs, vec![attr2.clone()]);

    stack.undo(&mut model);
    assert_eq!(model.steel_design_attrs, vec![attr1.clone()]);

    // 削除
    stack.run(
        &mut model,
        Box::new(RemoveSteelDesignAttr { elem: ElemId(0) }),
    );
    assert!(model.steel_design_attrs.is_empty());

    stack.undo(&mut model);
    assert_eq!(model.steel_design_attrs, vec![attr1]);
}

#[test]
fn test_remove_steel_design_attr_missing_is_noop() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(RemoveSteelDesignAttr { elem: ElemId(0) }),
    );
    assert!(model.steel_design_attrs.is_empty());
    // 失敗したコマンド（Noop）は undo 履歴に積まれない。
    assert!(!stack.can_undo());
    assert!(model.steel_design_attrs.is_empty());
}

#[test]
fn test_delete_member_shifts_and_restores_side_table_attrs() {
    use squid_n_core::model::{DamperProps, HysteresisModel};
    let mut model = two_member_model();
    // 部材0に履歴則、部材1にダンパー特性を付与。
    model.set_member_hysteresis(ElemId(0), HysteresisModel::Takeda);
    let props = DamperProps {
        kd: 90_000.0,
        c0: 1_500.0,
        alpha: 0.4,
        ..Default::default()
    };
    model.set_damper_props(ElemId(1), Some(props));
    let before = model.clone();

    let mut stack = UndoStack::new();
    // 部材0を削除 → 部材1が ElemId(0) へ繰り上がり、その側テーブル参照も追従する。
    stack.run(&mut model, Box::new(DeleteMember { id: ElemId(0) }));
    assert_eq!(model.elements.len(), 1);
    // 削除された部材0の履歴則は消える。
    assert_eq!(model.member_hysteresis(ElemId(0)), None);
    // 元・部材1のダンパー特性は新 ElemId(0) を指す（参照整合）。
    assert_eq!(model.damper_props(ElemId(0)), Some(props));
    assert!(model.validate().is_ok());

    // undo で側テーブル属性も含め完全復元。
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert_eq!(
        model.member_hysteresis(ElemId(0)),
        Some(HysteresisModel::Takeda)
    );
    assert_eq!(model.damper_props(ElemId(1)), Some(props));

    // redo で再削除・再整合。
    stack.redo(&mut model);
    assert_eq!(model.damper_props(ElemId(0)), Some(props));
    assert_eq!(model.member_hysteresis(ElemId(0)), None);
}

/// `DeleteMember` が `member_detail_attrs`（ハンチ・継手位置）も
/// `take_elem_attrs`/`restore_elem_attrs` 経由で退避・復元すること
/// （`ElemAttrs.detail` の配線の検証）。
#[test]
fn test_delete_member_restores_member_detail_attr() {
    use squid_n_core::model::{Haunch, MemberDetailAttr};
    let mut model = two_member_model();
    // 部材0にハンチ付帯情報を付与。
    let attr = MemberDetailAttr {
        elem: ElemId(0),
        haunch_i: Some(Haunch {
            length: 700.0,
            depth_increase: 200.0,
            width_increase: 0.0,
        }),
        haunch_j: None,
        joints: vec![],
    };
    model.member_detail_attrs.push(attr.clone());
    let before = model.clone();

    let mut stack = UndoStack::new();
    // 部材0を削除 → 付帯情報も連動して消える。
    stack.run(&mut model, Box::new(DeleteMember { id: ElemId(0) }));
    assert_eq!(model.elements.len(), 1);
    assert!(model.member_detail(ElemId(0)).is_none());
    assert!(model.validate().is_ok());

    // undo で付帯情報も含め完全復元。
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert_eq!(model.member_detail(ElemId(0)), Some(&attr));

    // redo で再削除。
    stack.redo(&mut model);
    assert!(model.member_detail(ElemId(0)).is_none());
}

/// `CompositeCommand`: ペースト相当（既存セルの `SetNodeCoord` ×2 ＋行追加の
/// `AddNode` ＋追加行への `SetNodeCoord`）を 1 undo で丸ごと復元すること。
#[test]
fn test_composite_paste_roundtrip() {
    let mut model = empty_model();
    for (i, x) in [0.0, 1000.0].iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: [*x, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    let before = model.clone();
    let mut stack = UndoStack::new();

    // 3 行 ×1 列のペースト相当: 既存 2 行の座標変更＋不足 1 行の追加＋追加行の座標設定。
    stack.run(
        &mut model,
        Box::new(CompositeCommand {
            label: "節点座標の貼り付け".to_string(),
            children: vec![
                Box::new(SetNodeCoord {
                    node: NodeId(0),
                    coord: [10.0, 20.0, 30.0],
                }),
                Box::new(SetNodeCoord {
                    node: NodeId(1),
                    coord: [40.0, 50.0, 60.0],
                }),
                Box::new(AddNode {
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FREE,
                }),
                // 直前の AddNode が作った節点を同一複合内で参照できること（逐次適用）。
                Box::new(SetNodeCoord {
                    node: NodeId(2),
                    coord: [70.0, 80.0, 90.0],
                }),
            ],
        }),
    );
    assert_eq!(model.nodes.len(), 3);
    assert_eq!(model.nodes[0].coord, [10.0, 20.0, 30.0]);
    assert_eq!(model.nodes[1].coord, [40.0, 50.0, 60.0]);
    assert_eq!(model.nodes[2].coord, [70.0, 80.0, 90.0]);
    assert_eq!(stack.undo_label(), Some("節点座標の貼り付け"));

    // undo 1 回で追加行の除去も含めて元通り。
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));

    // redo 1 回で再適用。
    stack.redo(&mut model);
    assert_eq!(model.nodes.len(), 3);
    assert_eq!(model.nodes[2].coord, [70.0, 80.0, 90.0]);
}

/// `CompositeCommand`: 行削除相当（`DeleteNode` を行番号の降順に並べた複合）の
/// undo 1 回で、ID の繰り上げ・部材参照の付け替えごと復元すること（§4.6 の前提）。
#[test]
fn test_composite_delete_nodes_descending_roundtrip() {
    let mut model = empty_model();
    for i in 0..5u32 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64 * 1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    // 削除対象外の節点 0-4 を結ぶ部材（削除で節点 4 の ID が繰り上がる）。
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(0), NodeId(4)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });
    let before = model.clone();
    let mut stack = UndoStack::new();

    // 節点 1・3 の行削除。降順に並べ、先行する削除で後続の ID がずれないようにする。
    stack.run(
        &mut model,
        Box::new(CompositeCommand {
            label: "選択した 2 行を削除".to_string(),
            children: vec![
                Box::new(DeleteNode { id: NodeId(3) }),
                Box::new(DeleteNode { id: NodeId(1) }),
            ],
        }),
    );
    assert_eq!(model.nodes.len(), 3);
    // 残った節点は旧 0・2・4 で、ID は 0・1・2 に詰められている。
    assert_eq!(model.nodes[1].coord, [2000.0, 0.0, 0.0]);
    assert_eq!(model.nodes[2].coord, [4000.0, 0.0, 0.0]);
    // 部材の参照も旧 4 → 新 2 へ繰り上がっている。
    assert_eq!(model.elements[0].nodes[1], NodeId(2));
    assert!(model.validate().is_ok());

    // undo 1 回で 5 節点・ID・部材参照ごと元通り。
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));

    // redo 1 回で再削除。
    stack.redo(&mut model);
    assert_eq!(model.nodes.len(), 3);
    assert_eq!(model.elements[0].nodes[1], NodeId(2));
}

// ============================================================================
// 二次部材（小梁・間柱）・一本部材指定（beam_groups）の参照整合
// ============================================================================

fn sample_secondary(id: u32, n0: u32, n1: u32) -> squid_n_core::model::SecondaryMember {
    squid_n_core::model::SecondaryMember {
        id: squid_n_core::ids::SecondaryMemberId(id),
        kind: squid_n_core::model::SecondaryMemberKind::Joist,
        nodes: [NodeId(n0), NodeId(n1)],
        section: None,
        name: "小梁".into(),
    }
}

/// 節点削除の ID 繰り上げが二次部材の節点参照にも波及すること。
/// 従来は `shift_node_ids` が `secondary_members` を走査しておらず、
/// 節点削除後に二次部材が別の節点へ張り付いていた（保存時の validate まで
/// 発覚しないダングリング）。
#[test]
fn test_delete_node_shifts_secondary_member_nodes() {
    let mut model = empty_model();
    for i in 0..3u32 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [f64::from(i) * 1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.secondary_members.push(sample_secondary(0, 1, 2));
    let before = model.clone();
    let mut stack = UndoStack::new();

    // 節点 0 はどこからも参照されていないので削除できる。
    stack.run(&mut model, Box::new(DeleteNode { id: NodeId(0) }));
    assert_eq!(model.nodes.len(), 2);
    // 二次部材の参照は旧 1→新 0、旧 2→新 1 へ繰り上がる。
    assert_eq!(model.secondary_members[0].nodes, [NodeId(0), NodeId(1)]);
    assert!(model.validate().is_ok());

    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
}

/// 二次部材の節点は「使用中」とみなされ、節点削除が Noop になること。
#[test]
fn test_delete_node_used_by_secondary_member_is_noop() {
    let mut model = empty_model();
    for i in 0..2u32 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [f64::from(i) * 1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.secondary_members.push(sample_secondary(0, 0, 1));
    let mut stack = UndoStack::new();
    stack.run(&mut model, Box::new(DeleteNode { id: NodeId(0) }));
    assert_eq!(model.nodes.len(), 2, "二次部材が参照する節点は削除できない");
}

/// 断面・材料削除の ID 繰り上げが二次部材の参照にも波及し、
/// 二次部材が参照中の断面・材料は削除ガードで Noop になること。
#[test]
fn test_delete_section_material_shift_and_guard_secondary_refs() {
    use squid_n_core::model::{Material, Section};
    let mut model = empty_model();
    for i in 0..2u32 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [f64::from(i) * 1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
        model.sections.push(Section {
            id: SectionId(i),
            name: format!("S{}", i),
            area: 100.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 10.0,
            width: 10.0,
            as_y: 80.0,
            as_z: 80.0,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: None,
            // 材料は断面が持つ。断面 i に材料 i を割り当てる。
            material: Some(MaterialId(i)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        });
        model.materials.push(Material {
            id: MaterialId(i),
            name: format!("M{}", i),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: None,
            concrete_class: Default::default(),
            strength_factor: None,
        });
    }
    let mut sm = sample_secondary(0, 0, 1);
    sm.section = Some(SectionId(1));
    model.secondary_members.push(sm);
    let mut stack = UndoStack::new();

    // 未使用の断面 0・材料 0 を削除 → 二次部材の断面参照と、その断面が持つ
    // 材料参照がどちらも 1→0 へ繰り上がる。
    stack.run(&mut model, Box::new(DeleteSection { id: SectionId(0) }));
    stack.run(&mut model, Box::new(DeleteMaterial { id: MaterialId(0) }));
    assert_eq!(model.secondary_members[0].section, Some(SectionId(0)));
    assert_eq!(model.sections[0].material, Some(MaterialId(0)));
    assert!(model.validate().is_ok());

    // 二次部材が参照中の断面と、その断面が参照中の材料は削除できない（Noop）。
    stack.run(&mut model, Box::new(DeleteSection { id: SectionId(0) }));
    stack.run(&mut model, Box::new(DeleteMaterial { id: MaterialId(0) }));
    assert_eq!(
        model.sections.len(),
        1,
        "二次部材が参照する断面は削除できない"
    );
    assert_eq!(model.materials.len(), 1, "断面が参照する材料は削除できない");
}

/// 部材削除が一本部材指定（beam_groups）から当該部材を連動削除し、
/// 残る参照は ID 繰り上げに追従し、undo で完全復元されること。
/// 従来は beam_groups が繰り上げの対象外で、部材削除後にグループが
/// 無関係な部材のモーメントを検定に合成していた。
#[test]
fn test_delete_member_cascades_beam_groups_and_restores() {
    let mut model = two_member_model();
    model.beam_groups = vec![vec![ElemId(0), ElemId(1)]];
    let before = model.clone();
    let mut stack = UndoStack::new();

    stack.run(&mut model, Box::new(DeleteMember { id: ElemId(0) }));
    // グループから削除部材が外れ、旧 ElemId(1) は新 ElemId(0) へ繰り上がる。
    assert_eq!(model.beam_groups, vec![vec![ElemId(0)]]);
    assert!(model.validate().is_ok());

    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert!(model.validate().is_ok());
}

/// `Model::validate` が beam_groups のダングリング参照を検出すること。
#[test]
fn test_validate_detects_dangling_beam_group() {
    let mut model = two_member_model();
    model.beam_groups = vec![vec![ElemId(5)]];
    assert!(model.validate().is_err());
}

/// 失敗したコマンドが redo 履歴を消さないこと。従来は失敗（Noop）でも
/// `undone.clear()` が走り、undo 直後に無効な操作をすると redo が失われていた。
#[test]
fn test_failed_command_keeps_redo_history() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(AddNode {
            coord: [1.0, 2.0, 3.0],
            restraint: Dof6Mask::FREE,
        }),
    );
    stack.undo(&mut model);
    assert!(stack.can_redo(), "undo 直後は redo できる");

    // 無効なコマンド（存在しない節点への座標設定）は適用されず、redo 履歴も残る。
    let applied = stack.run(
        &mut model,
        Box::new(SetNodeCoord {
            node: NodeId(99),
            coord: [0.0, 0.0, 0.0],
        }),
    );
    assert!(!applied, "失敗したコマンドは適用されない");
    assert!(stack.can_redo(), "失敗したコマンドで redo 履歴が消えない");
    stack.redo(&mut model);
    assert_eq!(model.nodes.len(), 1, "redo で節点追加が再適用される");
}

// ---------------------------------------------------------------------------
// 断面の同一性キー（符号＋階）の不変条件
// ---------------------------------------------------------------------------

fn h_shape() -> squid_n_section::shape::SectionShape {
    squid_n_section::shape::SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    }
}

/// 符号＋階が同じ断面は追加できない。符号か階のどちらかが違えば追加できる。
/// GUI 以外の呼び出し元（MCP など）に対しても不変条件を守るため、コマンド側で拒否する。
#[test]
fn test_add_section_shape_rejects_duplicate_key() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    let add = |name: &str, floor: Option<&str>, id: u32| AddSectionShape {
        shape: h_shape(),
        new_id: SectionId(id),
        name: name.into(),
        floor: floor.map(str::to_string),
    };

    stack.run(&mut model, Box::new(add("C1", Some("1"), 0)));
    assert_eq!(model.sections.len(), 1);

    // 符号＋階が同じなので追加されない。
    stack.run(&mut model, Box::new(add("C1", Some("1"), 1)));
    assert_eq!(model.sections.len(), 1, "符号＋階の重複は追加できない");

    // 階が違えば別断面として追加できる。
    stack.run(&mut model, Box::new(add("C1", Some("2"), 1)));
    assert_eq!(model.sections.len(), 2);
    assert_eq!(model.sections[1].floor.as_deref(), Some("2"));

    // 階なしも符号だけで一意判定する。
    stack.run(&mut model, Box::new(add("C1", None, 2)));
    assert_eq!(model.sections.len(), 3);
    stack.run(&mut model, Box::new(add("C1", None, 3)));
    assert_eq!(model.sections.len(), 3, "階なしどうしの符号重複も拒否する");
}

/// 符号・階の変更も、他の断面と衝突する場合は適用しない。自分自身は衝突判定から外す。
#[test]
fn test_set_section_name_rejects_duplicate_key() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    for (i, floor) in ["1", "2"].iter().enumerate() {
        stack.run(
            &mut model,
            Box::new(AddSectionShape {
                shape: h_shape(),
                new_id: SectionId(i as u32),
                name: "C1".into(),
                floor: Some((*floor).to_string()),
            }),
        );
    }

    // C1(2) を C1(1) にしようとしても適用されない。
    stack.run(
        &mut model,
        Box::new(SetSectionName {
            id: SectionId(1),
            name: "C1".into(),
            floor: Some("1".into()),
        }),
    );
    assert_eq!(model.sections[1].floor.as_deref(), Some("2"), "衝突は拒否");

    // 空いているキーへは変更でき、undo で戻る。
    stack.run(
        &mut model,
        Box::new(SetSectionName {
            id: SectionId(1),
            name: "C9".into(),
            floor: Some("3".into()),
        }),
    );
    assert_eq!(model.sections[1].name, "C9");
    assert_eq!(model.sections[1].floor.as_deref(), Some("3"));
    stack.undo(&mut model);
    assert_eq!(model.sections[1].name, "C1");
    assert_eq!(model.sections[1].floor.as_deref(), Some("2"));

    // 自分自身と同じキーへの変更は衝突扱いにしない（符号だけを直す操作が通る）。
    stack.run(
        &mut model,
        Box::new(SetSectionName {
            id: SectionId(0),
            name: "C1".into(),
            floor: Some("1".into()),
        }),
    );
    assert_eq!(model.sections[0].name, "C1");
}

/// 断面形状を変更しても符号と階は維持する（同一性キーが形状変更で消えない）。
#[test]
fn test_edit_section_shape_keeps_name_and_floor() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    stack.run(
        &mut model,
        Box::new(AddSectionShape {
            shape: h_shape(),
            new_id: SectionId(0),
            name: "C1".into(),
            floor: Some("1".into()),
        }),
    );
    stack.run(
        &mut model,
        Box::new(EditSectionShape {
            section: SectionId(0),
            new_shape: squid_n_section::shape::SectionShape::SteelBox {
                height: 300.0,
                width: 300.0,
                thick: 12.0,
                corner_r: 0.0,
            },
        }),
    );
    assert_eq!(model.sections[0].name, "C1");
    assert_eq!(model.sections[0].floor.as_deref(), Some("1"));
    assert!(matches!(
        model.sections[0].shape,
        Some(squid_n_section::shape::SectionShape::SteelBox { .. })
    ));
}

/// カタログ断面の追加も符号の重複を拒否する（同じカタログ断面を 2 回追加できない）。
#[test]
fn test_add_catalog_section_rejects_duplicate_key() {
    let mut model = empty_model();
    let mut stack = UndoStack::new();
    let sec = h_shape().to_section(SectionId(0), "H-300x150x6.5x9".into());

    stack.run(
        &mut model,
        Box::new(AddCatalogSection {
            section: sec.clone(),
        }),
    );
    assert_eq!(model.sections.len(), 1);
    stack.run(&mut model, Box::new(AddCatalogSection { section: sec }));
    assert_eq!(model.sections.len(), 1, "同じ符号は 2 回追加できない");

    stack.undo(&mut model);
    assert_eq!(model.sections.len(), 0);
}

// ---- 階定義の編集（階名・階レベル・追加・削除） ----

/// 階 `levels` と、標高 `zs` の節点を持つモデル。
fn story_edit_model(zs: &[f64], levels: &[(&str, f64)]) -> Model {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::model::{Node, Story};

    Model {
        nodes: zs
            .iter()
            .enumerate()
            .map(|(i, &z)| Node {
                id: NodeId(i as u32),
                coord: [0.0, 0.0, z],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            })
            .collect(),
        stories: levels
            .iter()
            .enumerate()
            .map(|(i, &(name, elevation))| Story {
                id: StoryId(i as u32),
                name: name.into(),
                elevation,
                node_ids: Vec::new(),
                seismic_weight: None,
                weight_override: None,
                structure: Default::default(),
                level_kind: Default::default(),
            })
            .collect(),
        ..Default::default()
    }
}

/// 階の追加は標高昇順の位置へ入り、以降の階の ID が繰り上がる。undo で元に戻る。
#[test]
fn test_add_story_inserts_in_elevation_order_and_renumbers() {
    let mut model = story_edit_model(&[0.0, 4000.0, 11000.0], &[("1F", 4000.0), ("3F", 11000.0)]);
    model.nodes[2].story = Some(StoryId(1));
    model
        .constraints
        .push(squid_n_core::model::Constraint::rigid_diaphragm(
            StoryId(1),
            NodeId(2),
            vec![],
        ));
    let mut undo = UndoStack::new();

    undo.run(
        &mut model,
        Box::new(AddStory {
            name: "2F".into(),
            elevation: 7500.0,
        }),
    );

    let names: Vec<&str> = model.stories.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names, vec!["1F", "2F", "3F"], "標高昇順に挿入される");
    assert!(
        model
            .stories
            .iter()
            .enumerate()
            .all(|(i, s)| s.id.index() == i),
        "StoryId ＝配列位置の不変条件が保たれる"
    );
    assert_eq!(
        model.nodes[2].story,
        Some(StoryId(2)),
        "3F の参照が繰り上がる"
    );
    assert_eq!(
        model.diaphragms_of(StoryId(2)).count(),
        1,
        "剛床の参照も繰り上がる"
    );
    assert!(model.validate().is_ok());

    undo.undo(&mut model);
    assert_eq!(model.stories.len(), 2);
    assert_eq!(model.nodes[2].story, Some(StoryId(1)));
    assert_eq!(model.diaphragms_of(StoryId(1)).count(), 1);
}

/// 階の削除は階定義だけを消し、節点・部材は残す。削除した階の剛床は取り除かれ、
/// その階に属していた節点は所属階を失う。上位階の ID は繰り下がる。
#[test]
fn test_delete_story_keeps_nodes_and_drops_its_diaphragm() {
    let mut model = story_edit_model(
        &[0.0, 4000.0, 7500.0, 11000.0],
        &[("1F", 4000.0), ("2F", 7500.0), ("3F", 11000.0)],
    );
    model.nodes[1].story = Some(StoryId(0));
    model.nodes[2].story = Some(StoryId(1));
    model.nodes[3].story = Some(StoryId(2));
    for (sid, nid) in [(0u32, 1u32), (1, 2), (2, 3)] {
        model
            .constraints
            .push(squid_n_core::model::Constraint::rigid_diaphragm(
                StoryId(sid),
                NodeId(nid),
                vec![],
            ));
    }
    let n_nodes = model.nodes.len();
    let mut undo = UndoStack::new();

    undo.run(&mut model, Box::new(DeleteStory { story: StoryId(1) }));

    assert_eq!(model.nodes.len(), n_nodes, "節点は消えない");
    let names: Vec<&str> = model.stories.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names, vec!["1F", "3F"]);
    assert_eq!(model.nodes[2].story, None, "削除した階の節点は所属階を失う");
    assert_eq!(
        model.nodes[3].story,
        Some(StoryId(1)),
        "上位階の参照が繰り下がる"
    );
    assert_eq!(model.constraints.len(), 2, "削除した階の剛床だけが消える");
    assert_eq!(model.diaphragms_of(StoryId(1)).count(), 1);
    assert!(model.validate().is_ok());

    undo.undo(&mut model);
    assert_eq!(model.stories.len(), 3);
    assert_eq!(model.nodes[2].story, Some(StoryId(1)));
    assert_eq!(model.constraints.len(), 3);
}

/// 階レベルの変更で並び順が入れ替わる場合も、標高昇順と ID＝配列位置を保つ。
#[test]
fn test_set_story_level_resorts_and_renumbers() {
    let mut model = story_edit_model(
        &[0.0, 4000.0, 7500.0],
        &[("1F", 0.0), ("2F", 4000.0), ("3F", 7500.0)],
    );
    model.nodes[1].story = Some(StoryId(1));
    model.nodes[2].story = Some(StoryId(2));
    let mut undo = UndoStack::new();

    // 2F のレベルを 3F より上へ動かす（入力ミスの是正など）。
    undo.run(
        &mut model,
        Box::new(SetStoryLevel {
            story: StoryId(1),
            name: "RF".into(),
            elevation: 9000.0,
        }),
    );

    let names: Vec<&str> = model.stories.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names, vec!["1F", "3F", "RF"], "標高昇順へ並べ替わる");
    assert!(model
        .stories
        .iter()
        .enumerate()
        .all(|(i, s)| s.id.index() == i));
    assert_eq!(
        model.nodes[1].story,
        Some(StoryId(2)),
        "旧 2F の節点は新 ID を指す"
    );
    assert_eq!(
        model.nodes[2].story,
        Some(StoryId(1)),
        "旧 3F の節点も追随する"
    );
    assert!(model.validate().is_ok());

    undo.undo(&mut model);
    let names: Vec<&str> = model.stories.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names, vec!["1F", "2F", "3F"]);
    assert_eq!(model.nodes[1].story, Some(StoryId(1)));
}

/// 基部の階（`StoryId(0)`）は標高を変えられず、削除もできない。
/// 階の列の先頭が基部であることは `Model::layers` が依拠する不変条件であり、
/// これが崩れると最下層が層の一覧から静かに落ちる。
#[test]
fn test_base_story_level_and_deletion_are_guarded() {
    let mut model = story_edit_model(
        &[0.0, 4000.0, 7500.0],
        &[("1F", 0.0), ("2F", 4000.0), ("3F", 7500.0)],
    );
    let mut undo = UndoStack::new();

    // 標高は据え置かれるが、階名の変更は通る。
    undo.run(
        &mut model,
        Box::new(SetStoryLevel {
            story: StoryId(0),
            name: "GL".into(),
            elevation: 2000.0,
        }),
    );
    assert_eq!(model.stories[0].elevation, 0.0, "基部の標高は変わらない");
    assert_eq!(model.stories[0].name, "GL", "階名は変えられる");
    assert_eq!(model.layer_count(), 2, "層は 2 つのまま");

    // 削除は Noop。
    assert!(
        !undo.run(&mut model, Box::new(DeleteStory { story: StoryId(0) })),
        "基部の階は削除できない"
    );
    assert_eq!(model.stories.len(), 3);
    assert_eq!(model.stories[0].elevation, model.base_elevation());
}

/// 適用待ちコマンドがモデルの変更（階の削除など）の後に残っても、index が
/// 範囲外なら Noop になり、誤った階へ適用されない。
/// 注意: `StoryId ＝配列位置`が不変条件のため、範囲内の古い index は常に
/// 「その位置に居る階」へ適用されてしまう。範囲外 Noop と UI 側の適用順序
/// （削除は最後、1 フレーム 1 コマンド）が実際の防御線である。
#[test]
fn test_stale_story_commands_are_noop() {
    let mut model = story_edit_model(
        &[0.0, 4000.0, 7500.0],
        &[("1F", 0.0), ("2F", 4000.0), ("3F", 7500.0)],
    );
    let mut undo = UndoStack::new();

    // 削除後に残った古いコマンド: 編集と削除の index はどちらも範囲外になる。
    let stale_level = Box::new(SetStoryLevel {
        story: StoryId(3),
        name: "RF".into(),
        elevation: 9000.0,
    });
    let stale_delete = Box::new(DeleteStory { story: StoryId(3) });

    // 先に 2F(=StoryId(1)) を削除すると階数が 2 になり、index 3 は範囲外。
    undo.run(&mut model, Box::new(DeleteStory { story: StoryId(1) }));
    assert_eq!(model.stories.len(), 2, "2F を削除すると残り 2 階");
    assert!(undo.can_undo());

    assert!(
        !undo.run(&mut model, stale_level),
        "範囲外の SetStoryLevel は Noop"
    );
    let names: Vec<&str> = model.stories.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names, vec!["1F", "3F"], "Noop はモデルを変えない");
    assert!(
        !undo.run(&mut model, stale_delete),
        "範囲外の DeleteStory は Noop（誤った階を消さない）"
    );
    assert_eq!(model.stories.len(), 2, "Noop はモデルを変えない");
    assert!(model.validate().is_ok());
}

/// 階への複製で使う、準備計算相当の所属階の割り当て（標高で階へ結び付ける）。
fn assign_node_stories(model: &mut Model) {
    let stories: Vec<(squid_n_core::ids::StoryId, f64)> =
        model.stories.iter().map(|s| (s.id, s.elevation)).collect();
    for n in &mut model.nodes {
        n.story = stories
            .iter()
            .find(|(_, z)| (n.coord[2] - z).abs() <= 1.0)
            .map(|(id, _)| *id);
    }
    for s in &mut model.stories {
        let ids: Vec<NodeId> = model
            .nodes
            .iter()
            .filter(|n| n.story == Some(s.id))
            .map(|n| n.id)
            .collect();
        s.node_ids = ids;
    }
}

/// 階への複製: 断面を複製先の階名で作り直して割り当て、undo で丸ごと戻す。
///
/// 階に属するかどうかは `Model::member_story`（材端節点のうちもっとも高い節点の
/// 所属階）で判定するため、階 `2F` には 1FL→2FL の柱と 2FL の大梁が属する。
#[test]
fn test_copy_story_assigns_sections_with_target_floor_name() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;

    let mut model = frame_model(&FrameSpec::default()).unwrap();
    assign_node_stories(&mut model);
    // 2F の部材へ、階を持つ断面 C1(2F) を割り当てる。
    let sec_id = SectionId(model.sections.len() as u32);
    let mut c1 = bare_section(sec_id, None);
    c1.name = "C1".into();
    c1.floor = Some("2F".into());
    model.sections.push(c1);
    let targets_2f: Vec<squid_n_core::ids::ElemId> = model
        .elements
        .iter()
        .filter(|e| model.member_story(e) == Some(StoryId(1)))
        .map(|e| e.id)
        .collect();
    assert!(!targets_2f.is_empty());
    for id in &targets_2f {
        model.elements[id.index()].section = Some(sec_id);
    }
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    let n_sections = model.sections.len();
    let cmd = CopyStory {
        from: StoryId(1),
        to: vec![StoryId(2)],
        targets: CopyTargets {
            sections: true,
            ..Default::default()
        },
        overwrite: true,
    };
    let report = cmd.preview(&model);
    assert_eq!(report.sections_created, 1, "3F 用に C1 を 1 枚だけ複製する");
    assert_eq!(report.created_sections, vec!["C1 (3F)".to_string()]);

    let mut stack = UndoStack::new();
    assert!(stack.run(&mut model, Box::new(cmd)));
    let copied = model
        .sections
        .iter()
        .find(|s| s.key() == ("C1", Some("3F")))
        .expect("C1 (3F) ができる");
    let assigned: Vec<&squid_n_core::model::ElementData> = model
        .elements
        .iter()
        .filter(|e| model.member_story(e) == Some(StoryId(2)))
        .collect();
    assert!(!assigned.is_empty());
    assert!(assigned.iter().all(|e| e.section == Some(copied.id)));
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    // undo でモデルが丸ごと戻る。
    stack.undo(&mut model);
    assert_eq!(model.sections.len(), n_sections, "複製した断面が消える");
    assert!(model
        .elements
        .iter()
        .filter(|e| model.member_story(e) == Some(StoryId(2)))
        .all(|e| e.section.is_none()));
}

/// 階への複製を 2 回実行しても、荷重が二重にならない（足すのではなく載せ替える）。
///
/// 足すだけにすると同じ部材へ同じ荷重が積み上がり、見た目では気づけないまま
/// 重い設計になる。複製先の手入力荷重を取り除いてから載せるため、2 回目は
/// 件数が変わらず `loads_replaced` に計上される。
#[test]
fn test_copy_story_replaces_loads_instead_of_stacking() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;
    use squid_n_core::model::{MemberLoad, MemberLoadKind};

    let mut model = frame_model(&FrameSpec::default()).unwrap();
    assign_node_stories(&mut model);
    // 2F の大梁を 1 本選び、手入力の等分布荷重を載せる。
    let girder = model
        .elements
        .iter()
        .find(|e| {
            model.member_story(e) == Some(StoryId(1))
                && model.nodes[e.nodes[0].index()].coord[2]
                    == model.nodes[e.nodes[1].index()].coord[2]
        })
        .map(|e| e.id)
        .expect("2F に大梁がある");
    model.load_cases[0].member.push(MemberLoad::manual(
        girder,
        [0.0, 0.0, -1.0],
        MemberLoadKind::Distributed {
            a: 0.0,
            b: 6000.0,
            w1: 10.0,
            w2: 10.0,
        },
    ));
    let base_loads = model.load_cases[0].member.len();

    let cmd = || CopyStory {
        from: StoryId(1),
        to: vec![StoryId(2)],
        targets: CopyTargets {
            loads: true,
            ..Default::default()
        },
        overwrite: true,
    };
    let mut stack = UndoStack::new();
    assert!(stack.run(&mut model, Box::new(cmd())));
    assert_eq!(model.load_cases[0].member.len(), base_loads + 1);

    // 2 回目は載せ替えになり、件数は増えない。
    let report = cmd().preview(&model);
    assert_eq!(report.loads_copied, 1);
    assert_eq!(report.loads_removed, 1, "複製先の手入力荷重を取り除く");
    assert!(stack.run(&mut model, Box::new(cmd())));
    assert_eq!(
        model.load_cases[0].member.len(),
        base_loads + 1,
        "2 回実行しても荷重は二重にならない"
    );
}

/// 床の複製で、同じ床を「新規」と「更新」に二重計上しない。
#[test]
fn test_copy_story_counts_new_slabs_once() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;
    use squid_n_core::model::{AreaLoad, SlabUsage};

    let mut model = frame_model(&FrameSpec::default()).unwrap();
    // 3F の床を消して、2F から配り直せる状態にする。
    model.slabs.retain(|sl| {
        let z = model.nodes[sl.boundary_nodes().unwrap()[0].index()].coord[2];
        !(7000.0..8000.0).contains(&z)
    });
    for (i, sl) in model.slabs.iter_mut().enumerate() {
        sl.id = squid_n_core::ids::SlabId(i as u32);
    }
    model.floor_regions.clear();
    assign_node_stories(&mut model);
    for sl in &mut model.slabs {
        sl.plate.usage = Some(SlabUsage::Office);
        sl.plate.loads = vec![AreaLoad {
            kind: "仕上".into(),
            value: 0.6,
        }];
    }
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    let cmd = CopyStory {
        from: StoryId(1),
        to: vec![StoryId(2)],
        targets: CopyTargets {
            slabs: true,
            loads: true,
            ..Default::default()
        },
        overwrite: true,
    };
    let report = cmd.preview(&model);
    assert_eq!(report.slabs_created, 2, "3F へ床を 2 枚作る");
    assert_eq!(report.slabs_updated, 0, "作ったばかりの床は更新に数えない");

    let mut stack = UndoStack::new();
    assert!(stack.run(&mut model, Box::new(cmd)));
    let new_slabs: Vec<&squid_n_core::model::Slab> = model
        .slabs
        .iter()
        .filter(|sl| {
            (model.nodes[sl.boundary_nodes().unwrap()[0].index()].coord[2] - 7500.0).abs() < 1.0
        })
        .collect();
    assert_eq!(new_slabs.len(), 2);
    assert!(new_slabs
        .iter()
        .all(|sl| sl.plate.usage == Some(SlabUsage::Office)));
    assert!(new_slabs.iter().all(|sl| sl.plate.loads.len() == 1));
    assert!(model.validate().is_ok(), "{:?}", model.validate());
}

/// 上書きが真なら、複製元の「無い」状態も写す（複製先の余分を削除・解除する）。
///
/// 複製は複製元の状態をそのまま写す操作なので、複製元に床が無い位置の床、
/// 複製元が断面を持たない相手の断面は、複製先から取り除く。
#[test]
fn test_copy_story_overwrite_mirrors_absence() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;

    let mut model = frame_model(&FrameSpec::default()).unwrap();
    assign_node_stories(&mut model);
    // 2F の床を 1 枚だけ消す（3F には両方ある状態にする）。
    let doomed = model
        .slabs
        .iter()
        .find(|sl| {
            (model.nodes[sl.boundary_nodes().unwrap()[0].index()].coord[2] - 4000.0).abs() < 1.0
        })
        .map(|sl| sl.id)
        .expect("2F に床がある");
    crate::DeleteSlab { id: doomed }.apply(&mut model);
    let slabs_3f = |m: &Model| {
        m.slabs
            .iter()
            .filter(|sl| {
                (m.nodes[sl.boundary_nodes().unwrap()[0].index()].coord[2] - 7500.0).abs() < 1.0
            })
            .count()
    };
    assert_eq!(slabs_3f(&model), 2, "3F には床が 2 枚ある");
    // 3F の部材へ断面を付ける（2F は未割当のまま）。
    let sec_id = SectionId(model.sections.len() as u32);
    let mut c1 = bare_section(sec_id, None);
    c1.name = "C1".into();
    c1.floor = Some("3F".into());
    model.sections.push(c1);
    let members_3f: Vec<squid_n_core::ids::ElemId> = model
        .elements
        .iter()
        .filter(|e| model.member_story(e) == Some(StoryId(2)))
        .map(|e| e.id)
        .collect();
    for id in &members_3f {
        model.elements[id.index()].section = Some(sec_id);
    }
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    let cmd = CopyStory {
        from: StoryId(1),
        to: vec![StoryId(2)],
        targets: CopyTargets {
            sections: true,
            slabs: true,
            ..Default::default()
        },
        overwrite: true,
    };
    let report = cmd.preview(&model);
    assert_eq!(report.slabs_deleted, 1, "2F に無い位置の 3F の床を消す");
    assert_eq!(
        report.sections_cleared,
        members_3f.len(),
        "2F が未割当なので 3F の断面を外す"
    );
    assert!(report.removes_input());

    let mut stack = UndoStack::new();
    assert!(stack.run(&mut model, Box::new(cmd)));
    assert_eq!(slabs_3f(&model), 1);
    assert!(members_3f
        .iter()
        .all(|id| model.elements[id.index()].section.is_none()));
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    // undo 1 回で削除・解除ごと戻る。
    stack.undo(&mut model);
    assert_eq!(slabs_3f(&model), 2);
    assert!(members_3f
        .iter()
        .all(|id| model.elements[id.index()].section == Some(sec_id)));
}

/// 上書きが偽なら、複製先の既存には触れない（削除も置換もしない）。
#[test]
fn test_copy_story_without_overwrite_keeps_existing() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;
    use squid_n_core::model::{MemberLoad, MemberLoadKind};

    let mut model = frame_model(&FrameSpec::default()).unwrap();
    assign_node_stories(&mut model);
    // 2F・3F の同じ位置の大梁へ、別々の手入力荷重を載せる。
    let girder_of = |m: &Model, story: StoryId| {
        m.elements
            .iter()
            .find(|e| {
                m.member_story(e) == Some(story)
                    && m.nodes[e.nodes[0].index()].coord[2] == m.nodes[e.nodes[1].index()].coord[2]
                    && m.nodes[e.nodes[0].index()].coord[1] == 0.0
                    && m.nodes[e.nodes[1].index()].coord[1] == 0.0
                    && m.nodes[e.nodes[0].index()].coord[0] == 0.0
            })
            .map(|e| e.id)
            .expect("大梁がある")
    };
    let load = |elem, w| {
        MemberLoad::manual(
            elem,
            [0.0, 0.0, -1.0],
            MemberLoadKind::Distributed {
                a: 0.0,
                b: 6000.0,
                w1: w,
                w2: w,
            },
        )
    };
    let g2 = girder_of(&model, StoryId(1));
    let g3 = girder_of(&model, StoryId(2));
    model.load_cases[0].member.push(load(g2, 10.0));
    model.load_cases[0].member.push(load(g3, 99.0));

    let cmd = CopyStory {
        from: StoryId(1),
        to: vec![StoryId(2)],
        targets: CopyTargets {
            loads: true,
            ..Default::default()
        },
        overwrite: false,
    };
    let report = cmd.preview(&model);
    assert_eq!(report.loads_removed, 0, "既存の荷重は消さない");
    assert_eq!(report.loads_copied, 0, "既に荷重がある相手へは載せない");

    // 3F の荷重は 99.0 のまま残る。
    let mut stack = UndoStack::new();
    assert!(!stack.run(&mut model, Box::new(cmd)), "変更がないので Noop");
    let kept = model.load_cases[0]
        .member
        .iter()
        .find(|l| l.elem == g3)
        .expect("3F の荷重が残る");
    assert!(matches!(
        kept.kind,
        MemberLoadKind::Distributed { w1, .. } if (w1 - 99.0).abs() < 1e-9
    ));
}

/// 上書きが真でも、複製元の平面の外にある複製先の床には手を触れない。
///
/// セットバックや張り出しのある建物で、複製元と関係のない場所が消えるのを防ぐ。
#[test]
fn test_copy_story_overwrite_keeps_slabs_outside_source_plan() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;
    use squid_n_core::model::Node;

    let mut model = frame_model(&FrameSpec {
        with_slabs: false,
        ..FrameSpec::default()
    })
    .unwrap();
    // 3F だけに張り出した平面（2F には節点が無い位置）を足し、そこへ床を張る。
    let mut extra = Vec::new();
    for (x, y) in [(0.0, -6000.0), (6000.0, -6000.0)] {
        let id = NodeId(model.nodes.len() as u32);
        model.nodes.push(Node {
            id,
            coord: [x, y, 7500.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
        extra.push(id);
    }
    // 既存の 3F の 2 節点と合わせて 4 角形にする。
    let corner = |x: f64, y: f64| {
        model
            .nodes
            .iter()
            .find(|n| {
                (n.coord[0] - x).abs() < 1.0
                    && (n.coord[1] - y).abs() < 1.0
                    && (n.coord[2] - 7500.0).abs() < 1.0
            })
            .map(|n| n.id)
            .expect("3F の節点がある")
    };
    model.slabs.push(Slab {
        id: SlabId(0),
        shape: SlabShape::Enclosed {
            boundary: vec![extra[0], extra[1], corner(6000.0, 0.0), corner(0.0, 0.0)],
        },
        plate: SlabPlate {
            section: None,
            loads: Vec::new(),
            usage: None,
            method: squid_n_core::model::DistributionMethod::TriTrapezoid,
            one_way: None,
        },
    });
    assign_node_stories(&mut model);
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    let cmd = CopyStory {
        from: StoryId(1),
        to: vec![StoryId(2)],
        targets: CopyTargets {
            slabs: true,
            ..Default::default()
        },
        overwrite: true,
    };
    let report = cmd.preview(&model);
    assert_eq!(
        report.slabs_deleted, 0,
        "複製元の平面の外にある床は消さない"
    );
}

/// 同じ符号＋階の断面が既にあれば使い回し、寸法が違えば名指しする。
#[test]
fn test_copy_story_reports_mismatched_existing_section() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;

    let mut model = frame_model(&FrameSpec::default()).unwrap();
    assign_node_stories(&mut model);
    // C1 (2F) と、寸法の違う C1 (3F) を用意する。
    let id_2f = SectionId(model.sections.len() as u32);
    let mut c1_2f = bare_section(id_2f, None);
    c1_2f.name = "C1".into();
    c1_2f.floor = Some("2F".into());
    c1_2f.area = 1000.0;
    model.sections.push(c1_2f);
    let id_3f = SectionId(model.sections.len() as u32);
    let mut c1_3f = bare_section(id_3f, None);
    c1_3f.name = "C1".into();
    c1_3f.floor = Some("3F".into());
    c1_3f.area = 500.0;
    model.sections.push(c1_3f);
    for e in model
        .elements
        .iter()
        .filter(|e| model.member_story(e) == Some(StoryId(1)))
        .map(|e| e.id)
        .collect::<Vec<_>>()
    {
        model.elements[e.index()].section = Some(id_2f);
    }

    let cmd = CopyStory {
        from: StoryId(1),
        to: vec![StoryId(2)],
        targets: CopyTargets {
            sections: true,
            ..Default::default()
        },
        overwrite: true,
    };
    let report = cmd.preview(&model);
    assert_eq!(report.sections_created, 0, "既存の C1 (3F) を使い回す");
    assert_eq!(report.sections_reused, 1);
    assert_eq!(report.mismatched_sections, vec!["C1 (3F)".to_string()]);

    // 中身は書き換えない（対象範囲の外の部材まで変わってしまうため）。
    let mut m = model.clone();
    cmd.apply(&mut m);
    assert_eq!(m.sections[id_3f.index()].area, 500.0);
}

/// 材長の違う相手（階高の違う柱）へ部材荷重を配るとき、載荷区間を材長へ合わせる。
///
/// 全長載荷は複製先の材長へ合わせ、部分載荷は位置をそのまま写す。収まらないものは
/// 配らずに数える。そのまま写すと `gauss_dist` が形状関数を材外へ外挿し、等価節点力が
/// 黙って誤る。
#[test]
fn test_copy_story_fits_member_load_to_target_length() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;
    use squid_n_core::model::{MemberLoad, MemberLoadKind};

    // 2F の階高 4000、3F の階高 3500。
    let mut model = frame_model(&FrameSpec::default()).unwrap();
    assign_node_stories(&mut model);
    let column_of = |m: &Model, story: StoryId| {
        m.elements
            .iter()
            .find(|e| {
                m.member_story(e) == Some(story)
                    && m.nodes[e.nodes[0].index()].coord[0] == 0.0
                    && m.nodes[e.nodes[0].index()].coord[1] == 0.0
                    && m.nodes[e.nodes[1].index()].coord[0] == 0.0
                    && m.nodes[e.nodes[1].index()].coord[1] == 0.0
            })
            .map(|e| e.id)
            .expect("柱がある")
    };
    let c2 = column_of(&model, StoryId(1));
    let c3 = column_of(&model, StoryId(2));
    assert_eq!(model.member_length(&model.elements[c2.index()]), 4000.0);
    assert_eq!(model.member_length(&model.elements[c3.index()]), 3500.0);

    // 全長載荷・収まる部分載荷・収まらない部分載荷の 3 つを載せる。
    let lc = &mut model.load_cases[0];
    lc.member.push(MemberLoad::manual(
        c2,
        [1.0, 0.0, 0.0],
        MemberLoadKind::Distributed {
            a: 0.0,
            b: 4000.0,
            w1: 1.0,
            w2: 1.0,
        },
    ));
    lc.member.push(MemberLoad::manual(
        c2,
        [1.0, 0.0, 0.0],
        MemberLoadKind::Point {
            a: 2000.0,
            p: 100.0,
        },
    ));
    lc.member.push(MemberLoad::manual(
        c2,
        [1.0, 0.0, 0.0],
        MemberLoadKind::Point {
            a: 3800.0,
            p: 100.0,
        },
    ));

    let cmd = CopyStory {
        from: StoryId(1),
        to: vec![StoryId(2)],
        targets: CopyTargets {
            loads: true,
            ..Default::default()
        },
        overwrite: true,
    };
    let report = cmd.preview(&model);
    assert_eq!(report.loads_copied, 2, "収まる 2 件だけ配る");
    assert_eq!(report.loads_unfit, 1, "材長に収まらない 1 件は配らない");

    let mut stack = UndoStack::new();
    assert!(stack.run(&mut model, Box::new(cmd)));
    let copied: Vec<&MemberLoad> = model.load_cases[0]
        .member
        .iter()
        .filter(|l| l.elem == c3)
        .collect();
    assert_eq!(copied.len(), 2);
    // 全長載荷は 3500 へ合う。
    assert!(copied.iter().any(|l| matches!(
        l.kind,
        MemberLoadKind::Distributed { a, b, .. } if a == 0.0 && (b - 3500.0).abs() < 1e-9
    )));
    // 部分載荷は位置をそのまま。
    assert!(copied
        .iter()
        .any(|l| matches!(l.kind, MemberLoadKind::Point { a, .. } if (a - 2000.0).abs() < 1e-9)));
    // 載荷区間が材長を超える荷重は残らない。
    assert!(copied.iter().all(|l| match l.kind {
        MemberLoadKind::Point { a, .. } => a <= 3500.0 + 1.0,
        MemberLoadKind::Distributed { b, .. } => b <= 3500.0 + 1.0,
    }));
}

/// 同じ構面のブレースと大梁を、材端の XY だけでは区別できない。
///
/// 階内の相対高さで区別するため、1FL→2FL のブレースは 2FL の大梁と別のキーになり、
/// 断面が誤って入れ替わらない。
#[test]
fn test_copy_story_distinguishes_brace_from_girder() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;
    use squid_n_core::model::{ElementKind, EndCondition, ForceRegime, LocalAxis};

    let mut model = frame_model(&FrameSpec::default()).unwrap();
    // 各階の同じ構面へブレースを 1 本ずつ足す（下階の隅 → 上階の隣の隅）。
    let node_at = |m: &Model, x: f64, z: f64| {
        m.nodes
            .iter()
            .find(|n| {
                (n.coord[0] - x).abs() < 1.0 && n.coord[1] == 0.0 && (n.coord[2] - z).abs() < 1.0
            })
            .map(|n| n.id)
            .expect("節点がある")
    };
    for (z0, z1) in [(0.0, 4000.0), (4000.0, 7500.0)] {
        let id = squid_n_core::ids::ElemId(model.elements.len() as u32);
        model.elements.push(ElementData {
            id,
            kind: ElementKind::Brace {
                tension_only: false,
            },
            nodes: smallvec![node_at(&model, 0.0, z0), node_at(&model, 6000.0, z1)],
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
    assign_node_stories(&mut model);
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    // 2F のブレースにだけ断面を付ける（大梁は未割当のまま）。
    let brace_2f = model
        .elements
        .iter()
        .find(|e| {
            matches!(e.kind, ElementKind::Brace { .. }) && model.member_story(e) == Some(StoryId(1))
        })
        .map(|e| e.id)
        .expect("2F のブレース");
    let sec = SectionId(model.sections.len() as u32);
    let mut br = bare_section(sec, None);
    br.name = "BR1".into();
    br.floor = Some("2F".into());
    model.sections.push(br);
    model.elements[brace_2f.index()].section = Some(sec);

    let cmd = CopyStory {
        from: StoryId(1),
        to: vec![StoryId(2)],
        targets: CopyTargets {
            sections: true,
            ..Default::default()
        },
        overwrite: true,
    };
    let mut stack = UndoStack::new();
    assert!(stack.run(&mut model, Box::new(cmd)));

    // 3F のブレースへ BR1 (3F) が付き、3F の大梁は未割当のまま。
    let brace_3f = model
        .elements
        .iter()
        .find(|e| {
            matches!(e.kind, ElementKind::Brace { .. }) && model.member_story(e) == Some(StoryId(2))
        })
        .expect("3F のブレース");
    let assigned = brace_3f.section.expect("ブレースへ断面が付く");
    assert_eq!(model.sections[assigned.index()].name, "BR1");
    assert_eq!(
        model.sections[assigned.index()].floor.as_deref(),
        Some("3F")
    );
    let girders_3f: Vec<&ElementData> = model
        .elements
        .iter()
        .filter(|e| {
            matches!(e.kind, ElementKind::Beam)
                && model.member_story(e) == Some(StoryId(2))
                && model.nodes[e.nodes[0].index()].coord[2]
                    == model.nodes[e.nodes[1].index()].coord[2]
        })
        .collect();
    assert!(!girders_3f.is_empty());
    assert!(
        girders_3f.iter().all(|e| e.section.is_none()),
        "大梁へブレースの断面が入らない"
    );
}

/// 床の断面参照が複製先の階の断面へ読み替わる（符号＋階の識別に合わせる）。
#[test]
fn test_copy_story_remaps_slab_section_to_target_floor() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;

    let mut model = frame_model(&FrameSpec::default()).unwrap();
    // 3F の床を消し、2F の床の断面へ階を持たせる。
    model.slabs.retain(|sl| {
        let z = model.nodes[sl.boundary_nodes().unwrap()[0].index()].coord[2];
        !(7000.0..8000.0).contains(&z)
    });
    for (i, sl) in model.slabs.iter_mut().enumerate() {
        sl.id = squid_n_core::ids::SlabId(i as u32);
    }
    model.floor_regions.clear();
    model.sections[0].floor = Some("2F".into());
    assign_node_stories(&mut model);
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    let cmd = CopyStory {
        from: StoryId(1),
        to: vec![StoryId(2)],
        targets: CopyTargets {
            slabs: true,
            ..Default::default()
        },
        overwrite: true,
    };
    let mut stack = UndoStack::new();
    assert!(stack.run(&mut model, Box::new(cmd)));

    let new_slabs: Vec<&squid_n_core::model::Slab> = model
        .slabs
        .iter()
        .filter(|sl| {
            (model.nodes[sl.boundary_nodes().unwrap()[0].index()].coord[2] - 7500.0).abs() < 1.0
        })
        .collect();
    assert_eq!(new_slabs.len(), 2);
    for sl in &new_slabs {
        let sec = sl.section().expect("床へ断面が付く");
        assert_eq!(model.sections[sec.index()].floor.as_deref(), Some("3F"));
        assert_eq!(model.sections[sec.index()].name, "S15");
    }
}

/// 床の削除を伴う回でも、新しく作った床を「新規」と「更新」で二重に数えない。
///
/// 床の削除は `FloorRegionId` を繰り上げるため、添字の閾値では新旧を見分けられない。
#[test]
fn test_copy_story_counts_new_slabs_once_with_deletion() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;
    use squid_n_core::model::{AreaLoad, SlabUsage};

    let mut model = frame_model(&FrameSpec::default()).unwrap();
    assign_node_stories(&mut model);
    // 床の平面位置（境界の最小 X）とレベルで 1 枚を選ぶ。
    let pick = |m: &Model, z: f64, x0: f64| {
        m.slabs
            .iter()
            .find(|sl| {
                let zs = m.nodes[sl.boundary_nodes().unwrap()[0].index()].coord[2];
                let xmin = sl
                    .boundary_nodes()
                    .unwrap()
                    .iter()
                    .map(|n| m.nodes[n.index()].coord[0])
                    .fold(f64::INFINITY, f64::min);
                (zs - z).abs() < 1.0 && (xmin - x0).abs() < 1.0
            })
            .map(|sl| sl.id)
            .expect("床がある")
    };
    // 2F は左の床板を、3F は右の床板を消す。複製で 3F の左が作られ、右が消える。
    let doomed = pick(&model, 4000.0, 0.0);
    crate::DeleteSlab { id: doomed }.apply(&mut model);
    let doomed3 = pick(&model, 7500.0, 6000.0);
    crate::DeleteSlab { id: doomed3 }.apply(&mut model);
    for sl in &mut model.slabs {
        sl.plate.usage = Some(SlabUsage::Office);
        sl.plate.loads = vec![AreaLoad {
            kind: "仕上".into(),
            value: 0.6,
        }];
    }

    let cmd = CopyStory {
        from: StoryId(1),
        to: vec![StoryId(2)],
        targets: CopyTargets {
            slabs: true,
            loads: true,
            ..Default::default()
        },
        overwrite: true,
    };
    let report = cmd.preview(&model);
    assert_eq!(report.slabs_deleted, 1, "2F に無い位置の 3F の床を消す");
    assert_eq!(report.slabs_created, 1, "2F にあって 3F に無い床を作る");
    assert_eq!(
        report.slabs_updated, 0,
        "作ったばかりの床は更新に数えない（削除で ID が繰り上がっても）"
    );
}

/// 座標が丸めの境目にあっても、許容差 1 mm 以内なら対応が取れる。
#[test]
fn test_copy_story_matches_across_rounding_boundary() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;

    let mut model = frame_model(&FrameSpec {
        with_slabs: false,
        ..FrameSpec::default()
    })
    .unwrap();
    // 3F の節点をわずかにずらす（丸めれば別バケット、実距離は 0.4 mm）。
    for n in &mut model.nodes {
        if (n.coord[2] - 7500.0).abs() < 1.0 && n.coord[0] == 6000.0 {
            n.coord[0] = 6000.4;
        }
    }
    for n in &mut model.nodes {
        if (n.coord[2] - 4000.0).abs() < 1.0 && n.coord[0] == 6000.0 {
            n.coord[0] = 5999.6;
        }
    }
    assign_node_stories(&mut model);

    let sec = SectionId(model.sections.len() as u32);
    let mut g1 = bare_section(sec, None);
    g1.name = "G1".into();
    g1.floor = Some("2F".into());
    model.sections.push(g1);
    let girder_2f = model
        .elements
        .iter()
        .find(|e| {
            model.member_story(e) == Some(StoryId(1))
                && model.nodes[e.nodes[0].index()].coord[2]
                    == model.nodes[e.nodes[1].index()].coord[2]
                && model.nodes[e.nodes[0].index()].coord[1] == 0.0
                && model.nodes[e.nodes[1].index()].coord[1] == 0.0
                && model.nodes[e.nodes[0].index()].coord[0] == 0.0
        })
        .map(|e| e.id)
        .expect("2F の大梁");
    model.elements[girder_2f.index()].section = Some(sec);

    let cmd = CopyStory {
        from: StoryId(1),
        to: vec![StoryId(2)],
        targets: CopyTargets {
            sections: true,
            ..Default::default()
        },
        overwrite: true,
    };
    let report = cmd.preview(&model);
    assert!(
        report.sections_assigned > 0,
        "0.4 mm のずれでも対応が取れる: {report:?}"
    );
}

// ─── 二次部材・壁領域のテスト ────────────────────────────────────────────────

/// 二次部材用の最小モデル（節点 2 個）を作る。
fn sm_base_model() -> Model {
    use squid_n_core::dof::Dof6Mask;
    let mut model = empty_model();
    for i in 0..2u32 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64 * 1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model
}

/// `kind` で小梁または間柱の `SecondaryMember` を生成する（テスト用ヘルパー）。
fn make_sm(
    id: u32,
    kind: squid_n_core::model::SecondaryMemberKind,
) -> squid_n_core::model::SecondaryMember {
    squid_n_core::model::SecondaryMember {
        id: SecondaryMemberId(id),
        kind,
        nodes: [NodeId(0), NodeId(1)],
        section: None,
        name: format!("SM{id}"),
    }
}

/// 二次部材の追加・削除・ID 繰り上げを確認する。
#[test]
fn add_delete_secondary_member() {
    use squid_n_core::model::SecondaryMemberKind;
    let mut model = sm_base_model();
    let mut stack = UndoStack::new();

    // 小梁 SM0・SM1 を追加する。
    stack.run(
        &mut model,
        Box::new(AddSecondaryMember {
            sm: make_sm(0, SecondaryMemberKind::Joist),
        }),
    );
    stack.run(
        &mut model,
        Box::new(AddSecondaryMember {
            sm: make_sm(1, SecondaryMemberKind::Joist),
        }),
    );
    assert_eq!(model.secondary_members.len(), 2);
    assert_eq!(model.secondary_members[0].id, SecondaryMemberId(0));
    assert_eq!(model.secondary_members[1].id, SecondaryMemberId(1));

    // 先頭（SM0）を削除 → 後続 SM1 が SM0 に繰り上がる。
    stack.run(
        &mut model,
        Box::new(DeleteSecondaryMember {
            id: SecondaryMemberId(0),
        }),
    );
    assert_eq!(model.secondary_members.len(), 1);
    assert_eq!(model.secondary_members[0].id, SecondaryMemberId(0));
    assert_eq!(model.secondary_members[0].name, "SM1");
    assert!(model.validate().is_ok());

    // undo で元の 2 本に戻る。
    stack.undo(&mut model);
    assert_eq!(model.secondary_members.len(), 2);
    assert_eq!(model.secondary_members[0].id, SecondaryMemberId(0));
    assert_eq!(model.secondary_members[1].id, SecondaryMemberId(1));
    assert!(model.validate().is_ok());
}

/// 二次部材削除時に `Slab.secondary_joist_ids` から除去されること。
#[test]
fn delete_secondary_member_cascade_slab() {
    use squid_n_core::model::SecondaryMemberKind;
    let mut model = sm_base_model();
    let mut stack = UndoStack::new();

    // SM0（Joist）を追加してスラブに登録する。
    stack.run(
        &mut model,
        Box::new(AddSecondaryMember {
            sm: make_sm(0, SecondaryMemberKind::Joist),
        }),
    );
    // 節点を 2 つ追加してスラブの境界とする。
    use squid_n_core::dof::Dof6Mask;
    for i in 2..4u32 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64 * 1000.0, 1000.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    // 二次部材が属す床領域（大梁の区画）。区画そのものは自動生成対象だが、
    // このテストは `secondary_joist_ids` の追随だけを見るため直接組み立てる。
    model.floor_regions.push(FloorRegion::new(
        FloorRegionId(0),
        vec![NodeId(0), NodeId(1), NodeId(3), NodeId(2)],
    ));
    stack.run(
        &mut model,
        Box::new(SetSlabSecondaryJoistIds {
            slab: FloorRegionId(0),
            ids: vec![SecondaryMemberId(0)],
        }),
    );
    assert_eq!(
        model.floor_regions[0].secondary_joist_ids,
        vec![SecondaryMemberId(0)]
    );

    // SM0 を削除 → スラブの secondary_joist_ids からも除去される。
    stack.run(
        &mut model,
        Box::new(DeleteSecondaryMember {
            id: SecondaryMemberId(0),
        }),
    );
    assert!(model.floor_regions[0].secondary_joist_ids.is_empty());
    assert!(model.validate().is_ok());
}

/// 二次部材削除時に `WallRegion.post_ids` から除去されること。
#[test]
fn delete_secondary_member_cascade_wall_region() {
    use squid_n_core::model::{SecondaryMemberKind, WallRegion};
    let mut model = sm_base_model();
    let mut stack = UndoStack::new();

    // 間柱 SM0 を追加して壁領域に登録する。
    stack.run(
        &mut model,
        Box::new(AddSecondaryMember {
            sm: make_sm(0, SecondaryMemberKind::Post),
        }),
    );
    stack.run(
        &mut model,
        Box::new(AddWallRegion {
            region: WallRegion {
                wall: None,
                post_ids: vec![SecondaryMemberId(0)],
            },
        }),
    );
    assert_eq!(model.wall_regions[0].post_ids, vec![SecondaryMemberId(0)]);

    // SM0 を削除 → 壁領域の post_ids からも除去される。
    stack.run(
        &mut model,
        Box::new(DeleteSecondaryMember {
            id: SecondaryMemberId(0),
        }),
    );
    assert!(model.wall_regions[0].post_ids.is_empty());
    assert!(model.validate().is_ok());
}

/// undo で削除した二次部材が元の位置・参照と共に復元されること。
#[test]
fn undo_delete_secondary_member() {
    use squid_n_core::model::{SecondaryMemberKind, WallRegion};
    let mut model = sm_base_model();
    let mut stack = UndoStack::new();

    // SM0（Joist）・SM1（Post）・SM2（Joist）を追加する。
    for (i, kind) in [
        SecondaryMemberKind::Joist,
        SecondaryMemberKind::Post,
        SecondaryMemberKind::Joist,
    ]
    .iter()
    .enumerate()
    {
        stack.run(
            &mut model,
            Box::new(AddSecondaryMember {
                sm: make_sm(i as u32, *kind),
            }),
        );
    }

    // スラブを追加して SM0・SM2 を登録する。
    use squid_n_core::dof::Dof6Mask;
    for i in 2..4u32 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64 * 1000.0, 1000.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.floor_regions.push(FloorRegion::new(
        FloorRegionId(0),
        vec![NodeId(0), NodeId(1), NodeId(3), NodeId(2)],
    ));
    stack.run(
        &mut model,
        Box::new(SetSlabSecondaryJoistIds {
            slab: FloorRegionId(0),
            ids: vec![SecondaryMemberId(0), SecondaryMemberId(2)],
        }),
    );

    // 壁領域を追加して SM1 を登録する。
    stack.run(
        &mut model,
        Box::new(AddWallRegion {
            region: WallRegion {
                wall: None,
                post_ids: vec![SecondaryMemberId(1)],
            },
        }),
    );

    let before = model.clone();

    // SM1（Post, index=1）を削除する。
    stack.run(
        &mut model,
        Box::new(DeleteSecondaryMember {
            id: SecondaryMemberId(1),
        }),
    );
    assert_eq!(model.secondary_members.len(), 2);
    // SM2 が SM1 に繰り上がり、スラブの secondary_joist_ids も追随する。
    assert_eq!(
        model.floor_regions[0].secondary_joist_ids,
        vec![SecondaryMemberId(0), SecondaryMemberId(1)]
    );
    // 壁領域の post_ids から SM1 が除去される。
    assert!(model.wall_regions[0].post_ids.is_empty());
    assert!(model.validate().is_ok());

    // undo で元の状態に完全復元される。
    stack.undo(&mut model);
    assert!(model.eq_ignoring_dofmap(&before));
    assert!(model.validate().is_ok());
}

/// 壁領域の追加・削除・undo を確認する。
#[test]
fn add_delete_wall_region() {
    use squid_n_core::model::WallRegion;
    let mut model = empty_model();
    let mut stack = UndoStack::new();

    let region = WallRegion {
        wall: None,
        post_ids: vec![],
    };

    // 追加
    stack.run(
        &mut model,
        Box::new(AddWallRegion {
            region: region.clone(),
        }),
    );
    assert_eq!(model.wall_regions.len(), 1);

    // 削除
    stack.run(&mut model, Box::new(DeleteWallRegion { index: 0 }));
    assert_eq!(model.wall_regions.len(), 0);
    assert!(model.validate().is_ok());

    // undo で復元
    stack.undo(&mut model);
    assert_eq!(model.wall_regions.len(), 1);
    assert!(model.validate().is_ok());
}

/// `SetSlabSecondaryJoistIds` で Joist でない ID を渡すとエラー（Noop）になること。
#[test]
fn set_slab_secondary_joist_ids_validation() {
    use squid_n_core::model::SecondaryMemberKind;
    let mut model = sm_base_model();
    let mut stack = UndoStack::new();

    // 間柱（Post）を追加する（Joist ではない）。
    stack.run(
        &mut model,
        Box::new(AddSecondaryMember {
            sm: make_sm(0, SecondaryMemberKind::Post),
        }),
    );

    // スラブを用意する。
    use squid_n_core::dof::Dof6Mask;
    for i in 2..4u32 {
        model.nodes.push(Node {
            id: NodeId(i),
            coord: [i as f64 * 1000.0, 1000.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.floor_regions.push(FloorRegion::new(
        FloorRegionId(0),
        vec![NodeId(0), NodeId(1), NodeId(3), NodeId(2)],
    ));

    // Post（SM0）を secondary_joist_ids に設定しようとしても Noop になる。
    let changed = stack.run(
        &mut model,
        Box::new(SetSlabSecondaryJoistIds {
            slab: FloorRegionId(0),
            ids: vec![SecondaryMemberId(0)],
        }),
    );
    assert!(!changed, "Joist でない ID を渡すと Noop");
    assert!(model.floor_regions[0].secondary_joist_ids.is_empty());
    assert!(model.validate().is_ok());
}

/// SetSecondaryMemberSection: 断面変更・undo・存在しない断面は Noop。
#[test]
fn test_set_secondary_member_section_roundtrip() {
    use squid_n_core::model::SecondaryMemberKind;
    let mut model = seeded_model(2, 0);
    model.sections.push(bare_section(SectionId(0), None));
    let sm = squid_n_core::model::SecondaryMember {
        id: squid_n_core::ids::SecondaryMemberId(0),
        kind: SecondaryMemberKind::Joist,
        nodes: [NodeId(0), NodeId(1)],
        section: None,
        name: "J1".into(),
    };
    model.secondary_members.push(sm);
    let mut stack = UndoStack::new();

    // 断面を割り当てる
    assert!(stack.run(
        &mut model,
        Box::new(SetSecondaryMemberSection {
            id: squid_n_core::ids::SecondaryMemberId(0),
            section: Some(SectionId(0)),
        }),
    ));
    assert_eq!(model.secondary_members[0].section, Some(SectionId(0)));

    // undo で元に戻る
    stack.undo(&mut model);
    assert_eq!(model.secondary_members[0].section, None);

    stack.redo(&mut model);
    assert_eq!(model.secondary_members[0].section, Some(SectionId(0)));

    // 実在しない断面は Noop
    let before = model.clone();
    assert!(!stack.run(
        &mut model,
        Box::new(SetSecondaryMemberSection {
            id: squid_n_core::ids::SecondaryMemberId(0),
            section: Some(SectionId(99)),
        }),
    ));
    assert!(model.eq_ignoring_dofmap(&before));
    assert!(model.validate().is_ok());
}

/// SetWallRegion: 全置換・undo・範囲外 index は Noop。
#[test]
fn test_set_wall_region_roundtrip() {
    use squid_n_core::model::{SecondaryMemberKind, WallRegion};
    let mut model = seeded_model(2, 1);
    // Post 種別の二次部材を追加
    model
        .secondary_members
        .push(squid_n_core::model::SecondaryMember {
            id: squid_n_core::ids::SecondaryMemberId(0),
            kind: SecondaryMemberKind::Post,
            nodes: [NodeId(0), NodeId(1)],
            section: None,
            name: "P1".into(),
        });
    // ElemId(0) を Wall 種別に変更する
    model.elements[0].kind = ElementKind::Wall;
    let region_a = WallRegion {
        wall: Some(ElemId(0)),
        post_ids: vec![],
    };
    let region_b = WallRegion {
        wall: Some(ElemId(0)),
        post_ids: vec![squid_n_core::ids::SecondaryMemberId(0)],
    };
    model.wall_regions.push(region_a.clone());
    let mut stack = UndoStack::new();

    // 置換
    assert!(stack.run(
        &mut model,
        Box::new(SetWallRegion {
            index: 0,
            region: region_b.clone(),
        }),
    ));
    assert_eq!(model.wall_regions[0], region_b);

    // undo
    stack.undo(&mut model);
    assert_eq!(model.wall_regions[0], region_a);

    // 範囲外 index は Noop
    assert!(!stack.run(
        &mut model,
        Box::new(SetWallRegion {
            index: 99,
            region: region_b,
        }),
    ));
    assert_eq!(model.wall_regions.len(), 1);
    assert!(model.validate().is_ok());
}

/// AddWallRegion: wall フィールドに Wall 種別でない要素 ID を渡すと Noop。
#[test]
fn test_add_wall_region_rejects_non_wall_elem() {
    use squid_n_core::model::WallRegion;
    // seeded_model(2,1) は Beam 要素を 1 本持つ
    let mut model = seeded_model(2, 1);
    let before = model.clone();
    let mut stack = UndoStack::new();

    // ElemId(0) は Beam なので Wall ではない → Noop
    assert!(!stack.run(
        &mut model,
        Box::new(AddWallRegion {
            region: WallRegion {
                wall: Some(ElemId(0)),
                post_ids: vec![],
            },
        }),
    ));
    assert!(
        model.eq_ignoring_dofmap(&before),
        "Beam を wall に指定した AddWallRegion は Noop のはず"
    );
    assert!(model.validate().is_ok(), "{:?}", model.validate());
}

/// SetSlabSecondaryJoistIds: undo で元リストに戻ること。
#[test]
fn test_set_slab_secondary_joist_ids_roundtrip() {
    use squid_n_core::model::SecondaryMemberKind;
    let mut model = seeded_model(4, 0);
    // 小梁（Joist）を 2 本追加
    for i in 0..2u32 {
        model
            .secondary_members
            .push(squid_n_core::model::SecondaryMember {
                id: squid_n_core::ids::SecondaryMemberId(i),
                kind: SecondaryMemberKind::Joist,
                nodes: [NodeId(0), NodeId(1)],
                section: None,
                name: format!("J{}", i),
            });
    }
    // 床領域を追加（secondary_joist_ids = [SmId(0)]）
    let mut region = FloorRegion::new(
        FloorRegionId(0),
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
    );
    region.secondary_joist_ids = vec![squid_n_core::ids::SecondaryMemberId(0)];
    model.floor_regions.push(region);
    let mut stack = UndoStack::new();

    // [SmId(1)] に置換
    assert!(stack.run(
        &mut model,
        Box::new(SetSlabSecondaryJoistIds {
            slab: squid_n_core::ids::FloorRegionId(0),
            ids: vec![squid_n_core::ids::SecondaryMemberId(1)],
        }),
    ));
    assert_eq!(
        model.floor_regions[0].secondary_joist_ids,
        vec![squid_n_core::ids::SecondaryMemberId(1)]
    );

    // undo で元の [SmId(0)] へ戻る
    stack.undo(&mut model);
    assert_eq!(
        model.floor_regions[0].secondary_joist_ids,
        vec![squid_n_core::ids::SecondaryMemberId(0)]
    );

    stack.redo(&mut model);
    assert_eq!(
        model.floor_regions[0].secondary_joist_ids,
        vec![squid_n_core::ids::SecondaryMemberId(1)]
    );
    assert!(model.validate().is_ok(), "{:?}", model.validate());
}

// ─── 階への複製・部材削除と二次部材／壁領域の参照整合 ──────────────────────────

/// 階への複製（二次部材）で `SecondaryMember.id == index` の不変条件が保たれること。
///
/// `..sm` で複製元の `id` を写すと複製先の部材が同じ ID を名乗り、`validate` が
/// `IndexMismatch` を返す（＝二次部材を持つモデルでは階への複製が通らない）。
#[test]
fn test_copy_story_secondary_keeps_id_index_invariant() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;

    let mut model = frame_model(&FrameSpec::default()).unwrap();
    assign_node_stories(&mut model);
    let n2f: Vec<NodeId> = model
        .nodes
        .iter()
        .filter(|n| n.story == Some(StoryId(1)))
        .map(|n| n.id)
        .collect();
    assert!(n2f.len() >= 2);
    model
        .secondary_members
        .push(squid_n_core::model::SecondaryMember {
            id: SecondaryMemberId(0),
            kind: squid_n_core::model::SecondaryMemberKind::Joist,
            nodes: [n2f[0], n2f[1]],
            section: None,
            name: "J1".into(),
        });
    assert!(model.validate().is_ok(), "前提: {:?}", model.validate());

    let mut stack = UndoStack::new();
    assert!(stack.run(
        &mut model,
        Box::new(CopyStory {
            from: StoryId(1),
            to: vec![StoryId(2)],
            targets: CopyTargets {
                secondary: true,
                ..Default::default()
            },
            overwrite: true,
        }),
    ));
    assert_eq!(model.secondary_members.len(), 2, "3F へ 1 本複製される");
    for (i, sm) in model.secondary_members.iter().enumerate() {
        assert_eq!(sm.id, SecondaryMemberId(i as u32), "id は添字と一致する");
    }
    assert!(
        model.validate().is_ok(),
        "複製後の validate: {:?}",
        model.validate()
    );

    stack.undo(&mut model);
    assert_eq!(model.secondary_members.len(), 1);
    assert!(model.validate().is_ok());
}

/// 階への複製（床）は床板（`Slab`）だけを写し、床領域（`FloorRegion`）の
/// 小梁登録には触れないこと。
///
/// 床領域は大梁の区画（`rebuild_floor_regions` が結び直す）であり、床板の
/// 複製操作の対象外。もし複製が床領域へ誤って波及すると、2F の小梁登録が
/// 3F の床領域にも紛れ込み、床荷重を二重に拾う。
#[test]
fn test_copy_story_slab_copy_does_not_touch_floor_regions() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;

    let mut model = frame_model(&FrameSpec::default()).unwrap();
    assign_node_stories(&mut model);
    let n2f: Vec<NodeId> = model
        .nodes
        .iter()
        .filter(|n| n.story == Some(StoryId(1)))
        .map(|n| n.id)
        .collect();
    model
        .secondary_members
        .push(squid_n_core::model::SecondaryMember {
            id: SecondaryMemberId(0),
            kind: squid_n_core::model::SecondaryMemberKind::Joist,
            nodes: [n2f[0], n2f[1]],
            section: None,
            name: "J1".into(),
        });
    // 2F の床領域へ小梁 SM0 を登録する。
    let src = model
        .floor_regions
        .iter()
        .position(|fr| fr.boundary.iter().all(|n| n2f.contains(n)))
        .expect("2F に床領域がある");
    model.floor_regions[src].secondary_joist_ids = vec![SecondaryMemberId(0)];
    // 3F の床板を消して、複製で「新規作成」が起きる状況にする
    // （`retain_slabs` で床領域の `slab_ids` からの参照も一緒に落とす）。
    let n3f: Vec<NodeId> = model
        .nodes
        .iter()
        .filter(|n| n.story == Some(StoryId(2)))
        .map(|n| n.id)
        .collect();
    model.retain_slabs(|sl| {
        !sl.boundary_nodes()
            .is_some_and(|b| b.iter().all(|n| n3f.contains(n)))
    });
    assert!(model.validate().is_ok(), "前提: {:?}", model.validate());
    let regions_before = model.floor_regions.clone();
    let slabs_before = model.slabs.len();

    let mut stack = UndoStack::new();
    assert!(stack.run(
        &mut model,
        Box::new(CopyStory {
            from: StoryId(1),
            to: vec![StoryId(2)],
            targets: CopyTargets {
                slabs: true,
                ..Default::default()
            },
            overwrite: true,
        }),
    ));
    assert!(model.slabs.len() > slabs_before, "3F の床板が作られる");
    assert_eq!(
        model.floor_regions, regions_before,
        "床板の複製は床領域（小梁登録を含む）に触れない"
    );
    assert!(model.validate().is_ok(), "{:?}", model.validate());
}

/// 壁部材を削除したとき、その壁版を指す壁領域が「版なし」へ戻り、
/// 繰り上げで別の壁へ黙って付け替わらないこと。undo で壁版が戻ること。
#[test]
fn test_delete_member_clears_wall_region_wall() {
    let mut model = seeded_model(4, 3);
    for e in &mut model.elements {
        e.kind = ElementKind::Wall;
    }
    // ElemId(1) を指す壁領域。削除で繰り上がると ElemId(2) だった壁を指してしまう。
    model.wall_regions.push(squid_n_core::model::WallRegion {
        wall: Some(ElemId(1)),
        post_ids: vec![],
    });
    assert!(model.validate().is_ok(), "前提: {:?}", model.validate());

    let mut stack = UndoStack::new();
    assert!(stack.run(&mut model, Box::new(DeleteMember { id: ElemId(1) })));
    assert_eq!(
        model.wall_regions[0].wall, None,
        "削除した壁を指す壁領域は版なしへ戻る"
    );
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    stack.undo(&mut model);
    assert_eq!(
        model.wall_regions[0].wall,
        Some(ElemId(1)),
        "undo で壁版の参照が戻る"
    );
    assert!(model.validate().is_ok(), "{:?}", model.validate());
}

/// 取り付く床板を追加し、undo で消える。取付き先が実在しない節点なら Noop。
#[test]
fn test_add_attached_slab_roundtrip() {
    use squid_n_core::ids::NodeId;
    use squid_n_core::model::{LoadTransfer, RegionAnchor, SlabPlate};

    let mut model = Model::default();
    for i in 0..2u32 {
        model.nodes.push(squid_n_core::model::Node {
            id: NodeId(i),
            coord: [i as f64 * 4000.0, 0.0, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    let mut undo = UndoStack::default();

    let cmd = crate::AddAttachedSlab {
        anchor: RegionAnchor::Line {
            nodes: [NodeId(0), NodeId(1)],
            span: [0.0, 1.0],
            transfer: LoadTransfer::Anchor,
        },
        extent: [1500.0, 1500.0],
        plate: SlabPlate::default(),
    };
    assert!(undo.run(&mut model, Box::new(cmd)));
    assert_eq!(model.slabs.len(), 1);
    assert!(model.slabs[0].is_attached());
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    undo.undo(&mut model);
    assert!(model.slabs.is_empty(), "undo で消える");

    // 実在しない節点を指す取り付く床板は作らない。
    let bad = crate::AddAttachedSlab {
        anchor: RegionAnchor::Point(NodeId(9)),
        extent: [1000.0, 1000.0],
        plate: SlabPlate::default(),
    };
    assert!(!undo.run(&mut model, Box::new(bad)));
    assert!(model.slabs.is_empty());

    // 部分区間（0.0 <= t_i < t_j <= 1.0）は受け付ける。
    let partial = crate::AddAttachedSlab {
        anchor: RegionAnchor::Line {
            nodes: [NodeId(0), NodeId(1)],
            span: [0.25, 0.75],
            transfer: LoadTransfer::Anchor,
        },
        extent: [1000.0, 1000.0],
        plate: SlabPlate::default(),
    };
    assert!(undo.run(&mut model, Box::new(partial)));
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    // 区間が逆順（始端 >= 終端）は受け付けない。
    let reversed = crate::AddAttachedSlab {
        anchor: RegionAnchor::Line {
            nodes: [NodeId(0), NodeId(1)],
            span: [0.75, 0.25],
            transfer: LoadTransfer::Anchor,
        },
        extent: [1000.0, 1000.0],
        plate: SlabPlate::default(),
    };
    let before = model.slabs.len();
    assert!(!undo.run(&mut model, Box::new(reversed)));
    assert_eq!(model.slabs.len(), before, "逆順の区間は追加されない");
}

/// 断面未割当、extent 1000,1000 の取り付く床板の追加が往復する。
#[test]
fn test_add_attached_slab_sectionless_extent_1000_roundtrip() {
    use squid_n_core::ids::NodeId;
    use squid_n_core::model::{LoadTransfer, RegionAnchor, SlabPlate, SlabShape};

    let mut model = Model::default();
    for i in 0..2u32 {
        model.nodes.push(squid_n_core::model::Node {
            id: NodeId(i),
            coord: [i as f64 * 4000.0, 0.0, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    let mut undo = UndoStack::default();
    assert!(undo.run(
        &mut model,
        Box::new(crate::AddAttachedSlab {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(0), NodeId(1)],
                span: [0.0, 1.0],
                transfer: LoadTransfer::Anchor,
            },
            extent: [1000.0, 1000.0],
            plate: SlabPlate::default(),
        })
    ));
    assert_eq!(model.slabs.len(), 1);
    assert!(model.slabs[0].plate.section.is_none());
    match &model.slabs[0].shape {
        SlabShape::Attached { extent, .. } => assert_eq!(*extent, [1000.0, 1000.0]),
        other => panic!("{other:?}"),
    }
    undo.undo(&mut model);
    assert!(model.slabs.is_empty());
}

/// SetAttachedExtent / SetAttachedAnchor の Noop。
#[test]
fn test_set_attached_extent_and_anchor_noop() {
    use squid_n_core::ids::{NodeId, SlabId};
    use squid_n_core::model::{LoadTransfer, RegionAnchor, Slab, SlabPlate, SlabShape};

    let mut model = seeded_model(4, 0);
    model.slabs.push(Slab {
        id: SlabId(0),
        shape: SlabShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        },
        plate: SlabPlate::default(),
    });
    let mut undo = UndoStack::default();
    assert!(
        !undo.run(
            &mut model,
            Box::new(crate::SetAttachedExtent {
                id: SlabId(0),
                extent: [1500.0, 1500.0],
            })
        ),
        "囲まれ床板への張り出し変更は Noop"
    );
    assert!(
        !undo.run(
            &mut model,
            Box::new(crate::SetAttachedExtent {
                id: SlabId(9),
                extent: [1500.0, 1500.0],
            })
        ),
        "存在しない ID は Noop"
    );

    assert!(undo.run(
        &mut model,
        Box::new(crate::AddAttachedSlab {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(0), NodeId(1)],
                span: [0.0, 1.0],
                transfer: LoadTransfer::Anchor,
            },
            extent: [1000.0, 1000.0],
            plate: SlabPlate::default(),
        })
    ));
    let aid = model.slabs.last().unwrap().id;
    assert!(
        !undo.run(
            &mut model,
            Box::new(crate::SetAttachedExtent {
                id: aid,
                extent: [f64::INFINITY, 1000.0],
            })
        ),
        "非有限の張り出しは Noop"
    );
    assert!(
        !undo.run(
            &mut model,
            Box::new(crate::SetAttachedAnchor {
                id: SlabId(0),
                anchor: RegionAnchor::Point(NodeId(0)),
            })
        ),
        "囲まれ床板への取付き先変更は Noop"
    );
    assert!(
        !undo.run(
            &mut model,
            Box::new(crate::SetAttachedAnchor {
                id: aid,
                anchor: RegionAnchor::Point(NodeId(99)),
            })
        ),
        "欠ける節点は Noop"
    );
    assert!(
        undo.run(
            &mut model,
            Box::new(crate::SetAttachedAnchor {
                id: aid,
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: [0.0, 0.5],
                    transfer: LoadTransfer::Anchor,
                },
            })
        ),
        "部分区間は受け付ける"
    );
    assert!(model.validate().is_ok(), "{:?}", model.validate());
    assert!(
        !undo.run(
            &mut model,
            Box::new(crate::SetAttachedAnchor {
                id: aid,
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: [0.5, 0.5],
                    transfer: LoadTransfer::Anchor,
                },
            })
        ),
        "始端 == 終端（幅0）は Noop"
    );
}

/// SetSlabSection は断面未割当の床板に断面を付け、None で断面を外す。
#[test]
fn test_set_slab_section_sets_and_clears_section() {
    use squid_n_core::ids::{SectionId, SlabId};
    use squid_n_core::model::{Slab, SlabPlate, SlabShape};

    let mut model = seeded_model(4, 0);
    model.sections.push(bare_section(SectionId(0), None));
    model.slabs.push(Slab {
        id: SlabId(0),
        shape: SlabShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        },
        plate: SlabPlate::default(),
    });
    assert!(model.slabs[0].plate.section.is_none());
    let mut undo = UndoStack::default();
    assert!(
        undo.run(
            &mut model,
            Box::new(crate::SetSlabSection {
                id: SlabId(0),
                section: Some(SectionId(0)),
            })
        ),
        "断面未割当へ断面を付ける"
    );
    assert_eq!(model.slabs[0].section(), Some(SectionId(0)));
    assert!(undo.run(
        &mut model,
        Box::new(crate::SetSlabSection {
            id: SlabId(0),
            section: None,
        })
    ));
    assert!(
        model.slabs[0].plate.section.is_none(),
        "None で断面が外れる"
    );
}

/// 床領域の名前を変更し、undo で戻る。同じ名前・存在しない ID は Noop。
#[test]
fn test_set_floor_region_name_roundtrip() {
    use squid_n_core::ids::FloorRegionId;
    use squid_n_core::model::FloorRegion;

    let mut model = Model::default();
    model
        .floor_regions
        .push(FloorRegion::new(FloorRegionId(0), Vec::new()));
    let mut undo = UndoStack::default();

    assert!(undo.run(
        &mut model,
        Box::new(crate::SetFloorRegionName {
            id: FloorRegionId(0),
            name: "階段室".into(),
        })
    ));
    assert_eq!(model.floor_regions[0].name, "階段室");
    undo.undo(&mut model);
    assert_eq!(model.floor_regions[0].name, "");

    // 同じ名前・存在しない ID は Noop。
    assert!(!undo.run(
        &mut model,
        Box::new(crate::SetFloorRegionName {
            id: FloorRegionId(0),
            name: String::new(),
        })
    ));
    assert!(!undo.run(
        &mut model,
        Box::new(crate::SetFloorRegionName {
            id: FloorRegionId(9),
            name: "x".into(),
        })
    ));
}

/// 階複製（床板）は取り付く床板を複製せず見送り件数へ数える。
///
/// 床領域（大梁の区画）は `CopyStory` の対象外（`rebuild_floor_regions` が
/// 別途結び直す）。ここでは床板（`Slab`）の複製だけを見る。
#[test]
fn test_copy_story_skips_attached_slabs() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;
    use squid_n_core::model::{LoadTransfer, RegionAnchor, SlabPlate};

    let mut model = frame_model(&FrameSpec::default()).unwrap();
    assign_node_stories(&mut model);

    // 2F に取り付く床板（バルコニー）を足す。
    let src = model
        .slabs
        .iter()
        .position(|sl| {
            let z = model.nodes[sl.boundary_nodes().unwrap()[0].index()].coord[2];
            (3000.0..5000.0).contains(&z)
        })
        .expect("2F の床板");
    let anchor_nodes = {
        let b = model.slabs[src].boundary_nodes().unwrap();
        [b[0], b[1]]
    };
    let mut undo = UndoStack::default();
    assert!(undo.run(
        &mut model,
        Box::new(crate::AddAttachedSlab {
            anchor: RegionAnchor::Line {
                nodes: anchor_nodes,
                span: [0.0, 1.0],
                transfer: LoadTransfer::Anchor,
            },
            extent: [1500.0, 1500.0],
            plate: SlabPlate::default(),
        })
    ));
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    let attached_before = model.slabs.iter().filter(|s| s.is_attached()).count();

    let cmd = CopyStory {
        from: StoryId(1),
        to: vec![StoryId(2)],
        targets: CopyTargets {
            slabs: true,
            ..Default::default()
        },
        overwrite: true,
    };
    let report = cmd.preview(&model);
    assert!(report.skipped > 0, "取り付く床板を見送り件数へ数える");

    undo.run(&mut model, Box::new(cmd));
    assert_eq!(
        model.slabs.iter().filter(|s| s.is_attached()).count(),
        attached_before,
        "取り付く床板は複製しない"
    );
    assert!(model.validate().is_ok(), "{:?}", model.validate());
}

/// 断面未割当の enclosed 床板は複製先でも断面未割当のまま。
#[test]
fn test_copy_story_keeps_sectionless_enclosed_slab() {
    use crate::{CopyStory, CopyTargets};
    use squid_n_core::frame_gen::{frame_model, FrameSpec};
    use squid_n_core::ids::StoryId;

    let mut model = frame_model(&FrameSpec::default()).unwrap();
    assign_node_stories(&mut model);
    let src_z = model.stories[1].elevation;
    let src = model
        .slabs
        .iter()
        .position(|sl| {
            sl.boundary_nodes()
                .is_some_and(|b| (model.nodes[b[0].index()].coord[2] - src_z).abs() < 1.0)
        })
        .expect("2F の床板");
    model.slabs[src].plate.section = None;
    let to_z = model.stories[2].elevation;
    let doomed: Vec<squid_n_core::ids::SlabId> = model
        .slabs
        .iter()
        .filter(|sl| {
            sl.boundary_nodes()
                .is_some_and(|b| (model.nodes[b[0].index()].coord[2] - to_z).abs() < 1.0)
        })
        .map(|sl| sl.id)
        .collect();
    for id in doomed.into_iter().rev() {
        crate::DeleteSlab { id }.apply(&mut model);
    }

    let mut undo = UndoStack::default();
    assert!(undo.run(
        &mut model,
        Box::new(CopyStory {
            from: StoryId(1),
            to: vec![StoryId(2)],
            targets: CopyTargets {
                slabs: true,
                ..Default::default()
            },
            overwrite: false,
        })
    ));
    let copied: Vec<_> = model
        .slabs
        .iter()
        .filter(|sl| {
            sl.boundary_nodes()
                .is_some_and(|b| (model.nodes[b[0].index()].coord[2] - to_z).abs() < 1.0)
        })
        .collect();
    assert!(copied.len() >= 2, "複製先にも床板ができる");
    assert_eq!(
        copied
            .iter()
            .filter(|sl| sl.plate.section.is_none())
            .count(),
        1,
        "断面未割当の enclosed 床板は断面が付かないまま複製される"
    );
}
