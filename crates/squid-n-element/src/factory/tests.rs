use super::*;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::AnalysisKind;
use squid_n_core::model::{EndCondition, LocalAxis, Material, MaterialCategory, Node, Section};

fn make_diaphragm_model() -> Model {
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
                coord: [5000.0, 0.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(2),
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        constraints: vec![squid_n_core::model::Constraint::rigid_diaphragm(
            squid_n_core::ids::StoryId(0),
            NodeId(2),
            vec![NodeId(1)],
        )],
        sections: vec![Section {
            id: SectionId(0),
            name: "sec".into(),
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
            fy: Some(1e20),
        }],
        ..Default::default()
    }
}

/// フォースレジームの解決: 明示指定はそのまま、Auto はトポロジ
/// （剛床所属 × 鉛直材か）から判定する。
#[test]
fn test_resolve_force_regime_explicit_and_auto() {
    let model = make_diaphragm_model();
    let make = |id: u32, nodes: [NodeId; 2], force_regime: ForceRegime| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![nodes[0], nodes[1]],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    let beam_explicit = make(0, [NodeId(0), NodeId(1)], ForceRegime::UniaxialBendingShear);
    assert!(matches!(
        resolve_force_regime(&beam_explicit, &model),
        ResolvedRegime::ConcentratedSpring
    ));
    let col_explicit = make(1, [NodeId(0), NodeId(2)], ForceRegime::AxialBendingInteract);
    assert!(matches!(
        resolve_force_regime(&col_explicit, &model),
        ResolvedRegime::Fiber
    ));
    // 柱トポロジ（Auto なら Fiber）でも明示指定が優先されること
    let col_explicit_spring = make(2, [NodeId(0), NodeId(2)], ForceRegime::UniaxialBendingShear);
    assert!(matches!(
        resolve_force_regime(&col_explicit_spring, &model),
        ResolvedRegime::ConcentratedSpring
    ));

    let beam_auto = make(0, [NodeId(0), NodeId(1)], ForceRegime::Auto);
    assert!(matches!(
        resolve_force_regime(&beam_auto, &model),
        ResolvedRegime::ConcentratedSpring
    ));
    let col_auto = make(1, [NodeId(0), NodeId(2)], ForceRegime::Auto);
    assert!(matches!(
        resolve_force_regime(&col_auto, &model),
        ResolvedRegime::Fiber
    ));
}

/// 線形弾性解析の要素生成は `ForceRegime` に依らず弾性 `BeamElement`。
///
/// 剛床に載る梁は `resolve_force_regime` では `ConcentratedSpring` に判定されるが、
/// これは非線形解析だけの振り分けである。線形側がこれに従うと
/// (1) 材端ばねが直列に入って弾性剛性が落ち、(2) 材端集中ばね梁は
/// `recover_forces` を持たないため部材内力が丸ごと欠落する。
#[test]
fn test_build_behavior_linear_is_always_elastic_beam() {
    let model = make_diaphragm_model();
    let make = |nodes: [NodeId; 2]| ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![nodes[0], nodes[1]],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let beam = make([NodeId(0), NodeId(1)]);
    let col = make([NodeId(0), NodeId(2)]);

    let ctx = crate::behavior::Ctx { model: &model };
    for (label, data) in [("集中ばね判定", &beam), ("ファイバー判定", &col)] {
        let behavior = build_behavior(data, &model);
        assert!(
            behavior.recover_forces(&[0.0; 12]).is_some(),
            "{label}: 線形解析の梁は内力を回収できる弾性 BeamElement であること"
        );
        let elastic = crate::frame::beam::BeamElement::new(data, &model);
        let k_ref = elastic.axis.to_global(&elastic.local_stiffness());
        let k = behavior.tangent_stiffness(&ctx);
        for i in 0..12 {
            for j in 0..12 {
                assert!(
                    (k.get(i, j) - k_ref.get(i, j)).abs() <= k_ref.get(i, j).abs() * 1e-12 + 1e-9,
                    "{label}: K[{i}][{j}] が弾性梁と一致しない: {} vs {}",
                    k.get(i, j),
                    k_ref.get(i, j)
                );
            }
        }
    }
}

#[test]
fn test_build_nonlinear_behavior_concentrated_spring_uses_spring_beam() {
    let model = make_diaphragm_model();
    let beam = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let behavior = build_nonlinear_behavior(
        &beam,
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    let snap = behavior.snapshot_state();
    let is_spring = snap
        .downcast_ref::<(
            Vec<Box<dyn squid_n_material::uniaxial::UniaxialMaterial>>,
            [f64; 4],
            [f64; 4],
            [f64; 12],
            [f64; 12],
        )>()
        .is_some();
    assert!(
        is_spring,
        "nonlinear ConcentratedSpring should be ConcentratedSpringBeam"
    );
}

#[test]
fn test_build_nonlinear_behavior_fiber_uses_fiber_beam() {
    let model = make_diaphragm_model();
    let col = ElementData {
        id: ElemId(1),
        kind: ElementKind::Fiber,
        nodes: smallvec::smallvec![NodeId(0), NodeId(2)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let behavior = build_nonlinear_behavior(
        &col,
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    let snap = behavior.snapshot_state();
    let is_fiber = snap
        .downcast_ref::<crate::frame::fiber::FiberBeamSnapshot>()
        .is_some();
    assert!(is_fiber, "nonlinear Fiber should be FiberBeam");
}

/// ブレース要素の生成モデル用（2 節点・軸方向 4000mm・断面積 2000mm2）。
fn make_brace_model(tension_only: bool) -> (Model, ElementData) {
    let model = Model {
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
                coord: [4000.0, 0.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        sections: vec![Section {
            id: SectionId(0),
            name: "brace".into(),
            area: 2000.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 100.0,
            width: 100.0,
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
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "steel".into(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(235.0),
        }],
        ..Default::default()
    };
    let elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Brace { tension_only },
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Pinned, EndCondition::Pinned],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    (model, elem)
}

/// ブレース: 線形・非線形のどちらの生成でも要素側の特別扱いはせず、全剛性
/// E·A/L の TrussElement を生成する。引張専用の圧縮側無効化は線形応力解析の
/// active-set 反復で扱う。
#[test]
fn test_build_behavior_brace_uses_full_truss_stiffness() {
    let ea_l = 205000.0 * 2000.0 / 4000.0;
    for tension_only in [false, true] {
        let (model, elem) = make_brace_model(tension_only);
        let ctx = crate::behavior::Ctx { model: &model };
        let k = build_behavior(&elem, &model).tangent_stiffness(&ctx);
        assert!(
            (k.get(0, 0) - ea_l).abs() < 1e-6,
            "線形 tension_only={tension_only}: k00={}",
            k.get(0, 0)
        );
    }

    let (model, elem) = make_brace_model(true);
    let ctx = crate::behavior::Ctx { model: &model };
    let k = build_nonlinear_behavior(
        &elem,
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    )
    .tangent_stiffness(&ctx);
    assert!(
        (k.get(0, 0) - ea_l).abs() < 1e-6,
        "非線形 tension_only: k00={}",
        k.get(0, 0)
    );
}

/// 壁要素の開口低減: wall_attrs の開口面積からせん断剛性が低減されること
/// （RC規準（耐震壁）の開口低減 r=1−1.25·√(開口面積/壁面積)）。
#[test]
fn test_build_behavior_wall_opening_reduces_shear_stiffness() {
    use squid_n_core::model::WallAttr;

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    };
    let mut model = Model {
        nodes: vec![
            make_node(0, [0.0, 0.0, 0.0]),
            make_node(1, [4000.0, 0.0, 0.0]),
            make_node(2, [4000.0, 0.0, 3000.0]),
            make_node(3, [0.0, 0.0, 3000.0]),
        ],
        sections: vec![Section {
            id: SectionId(0),
            name: "wall".into(),
            area: 150.0 * 1000.0,
            iy: 1.0e9,
            iz: 1.0e9,
            j: 1.0e9,
            depth: 1000.0,
            width: 150.0,
            as_y: 125_000.0,
            as_z: 125_000.0,
            floor: None,
            panel_thickness: None,
            thickness: Some(150.0),
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
            name: "FC24".into(),
            category: MaterialCategory::Concrete,
            young: 23000.0,
            poisson: 0.2,
            density: 0.0,
            shear: None,
            fc: Some(24.0),
            fy: None,
        }],
        ..Default::default()
    };
    let wall = ElementData {
        id: ElemId(0),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    let shear_pattern = |k: &crate::behavior::LocalMat| -> f64 {
        let mut u = [0.0; 24];
        u[2 * 6] = 1.0;
        u[3 * 6] = 1.0;
        let mut s = 0.0;
        for i in 0..24 {
            for j in 0..24 {
                s += u[i] * k.get(i, j) * u[j];
            }
        }
        s
    };
    let axial_pattern = |k: &crate::behavior::LocalMat| -> f64 {
        let mut u = [0.0; 24];
        u[2 * 6 + 2] = 1.0;
        u[3 * 6 + 2] = 1.0;
        let mut s = 0.0;
        for i in 0..24 {
            for j in 0..24 {
                s += u[i] * k.get(i, j) * u[j];
            }
        }
        s
    };

    let b_no = build_behavior(&wall, &model);
    let ctx = crate::behavior::Ctx { model: &model };
    let k_no = b_no.tangent_stiffness(&ctx);

    model.wall_attrs.push(WallAttr {
        elem: ElemId(0),
        opening_area: 1.2e6,
        opening_weight: 0.0,
        slit: Default::default(),
        openings: vec![],
        finish_intensity: 0.0,
    });
    let b_open = build_behavior(&wall, &model);
    let ctx2 = crate::behavior::Ctx { model: &model };
    let k_open = b_open.tangent_stiffness(&ctx2);

    model.wall_attrs[0] = WallAttr {
        elem: ElemId(0),
        opening_area: 1.0,
        opening_weight: 0.0,
        slit: Default::default(),
        finish_intensity: 0.0,
        openings: vec![
            squid_n_core::model::WallOpening {
                width: 1000.0,
                height: 800.0,
                offset: Some([0.0, 500.0]),
            },
            squid_n_core::model::WallOpening {
                width: 500.0,
                height: 800.0,
                offset: Some([1800.0, 500.0]),
            },
        ],
    };
    let b_dims = build_behavior(&wall, &model);
    let ctx3 = crate::behavior::Ctx { model: &model };
    let k_dims = b_dims.tangent_stiffness(&ctx3);
    assert!(
        (shear_pattern(&k_dims) - shear_pattern(&k_open)).abs() < 1e-6,
        "個別開口(Σ1.2e6)と面積のみ(1.2e6)の低減が一致しない: {} vs {}",
        shear_pattern(&k_dims),
        shear_pattern(&k_open)
    );

    model.multi_opening_mode = squid_n_core::model::MultiOpeningMode::Envelope;
    let b_env = build_behavior(&wall, &model);
    let ctx4 = crate::behavior::Ctx { model: &model };
    let k_env = b_env.tangent_stiffness(&ctx4);
    assert!(
        shear_pattern(&k_env) < shear_pattern(&k_dims) * 0.999,
        "包絡モードで低減が強まらない: env={} eq={}",
        shear_pattern(&k_env),
        shear_pattern(&k_dims)
    );
    model.multi_opening_mode = squid_n_core::model::MultiOpeningMode::Equivalent;

    assert!(
        shear_pattern(&k_open) < shear_pattern(&k_no) * 0.999,
        "shear open={} no={}",
        shear_pattern(&k_open),
        shear_pattern(&k_no)
    );
    assert!((axial_pattern(&k_open) - axial_pattern(&k_no)).abs() < 1e-6);
}

#[test]
fn test_resolve_member_hysteresis_and_flexural_springs() {
    use squid_n_core::model::HysteresisModel;
    use squid_n_core::section_shape::{RcBeamRebar, SectionShape};

    fn rebar() -> RcBeamRebar {
        use squid_n_core::section_shape::BeamStirrup;
        RcBeamRebar {
            main_dia: 22.0,
            top: vec![2],
            bottom: vec![2],
            cover: 50.0,
            stirrup: BeamStirrup {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
            },
        }
    }

    let mut model = make_diaphragm_model();
    let beam = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    model.elements.push(beam.clone());

    assert!(!is_rc_like_section(&beam, &model));
    assert_eq!(
        resolve_member_hysteresis(&beam, &model, AnalysisKind::Incremental),
        HysteresisModel::Standard
    );
    let (_i, _j, backbone) = build_flexural_springs(
        &beam,
        &model,
        HysteresisModel::Standard,
        StrengthBasis::Nominal,
    );
    assert!(backbone.use_mn);

    model.sections[0].shape = Some(SectionShape::RcBeamRect {
        b: 400.0,
        d: 700.0,
        rebar: rebar(),
    });
    model.sections[0].depth = 700.0;
    model.sections[0].width = 400.0;
    model.sections[0].iz = 400.0 * 700.0f64.powi(3) / 12.0;
    model.materials[0].fc = Some(24.0);
    model.materials[0].fy = Some(345.0);
    assert!(is_rc_like_section(&beam, &model));
    assert_eq!(
        resolve_member_hysteresis(&beam, &model, AnalysisKind::Incremental),
        HysteresisModel::Takeda
    );
    let (_i, _j, backbone) = build_flexural_springs(
        &beam,
        &model,
        HysteresisModel::Takeda,
        StrengthBasis::Nominal,
    );
    assert!(
        !backbone.use_mn,
        "武田型(履歴材料)は N-M 相関(set_yield)対象外"
    );

    model.sections[0].shape = Some(SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    });
    assert!(!is_rc_like_section(&beam, &model));
    assert_eq!(
        resolve_member_hysteresis(&beam, &model, AnalysisKind::Incremental),
        HysteresisModel::Standard
    );

    model.set_member_hysteresis(ElemId(0), HysteresisModel::MaxPointOriented);
    assert_eq!(
        resolve_member_hysteresis(&beam, &model, AnalysisKind::Incremental),
        HysteresisModel::MaxPointOriented
    );
    let (_i, _j, backbone) = build_flexural_springs(
        &beam,
        &model,
        HysteresisModel::MaxPointOriented,
        StrengthBasis::Nominal,
    );
    assert!(!backbone.use_mn);
}

/// 履歴則の 2 スロット（増分用／時刻歴用）の解決を検証する。
/// - 時刻歴用スロット未指定は増分用の指定に従う
/// - 時刻歴用スロット指定は時刻歴解決のみに効く
#[test]
fn test_resolve_member_hysteresis_two_slots() {
    use squid_n_core::model::HysteresisModel;

    let mut model = make_diaphragm_model();
    let beam = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    model.elements.push(beam.clone());

    for kind in [AnalysisKind::Incremental, AnalysisKind::TimeHistory] {
        assert_eq!(
            resolve_member_hysteresis(&beam, &model, kind),
            HysteresisModel::Standard
        );
    }

    model.set_member_hysteresis(ElemId(0), HysteresisModel::OriginOriented);
    assert_eq!(
        resolve_member_hysteresis(&beam, &model, AnalysisKind::TimeHistory),
        HysteresisModel::OriginOriented
    );

    model.set_member_hysteresis_th(ElemId(0), Some(HysteresisModel::MaxPointOriented));
    assert_eq!(
        resolve_member_hysteresis(&beam, &model, AnalysisKind::Incremental),
        HysteresisModel::OriginOriented
    );
    assert_eq!(
        resolve_member_hysteresis(&beam, &model, AnalysisKind::TimeHistory),
        HysteresisModel::MaxPointOriented
    );
}

/// ファイバー・MS のコンクリート除荷則の解決を検証する。
/// - 既定: 増分=逆行型、時刻歴=Karsan–Jirsa 型（壁は増分=原点指向型）
/// - コンクリート履歴として解釈できない指定（武田型等）は既定へフォールバック
#[test]
fn test_resolve_fiber_concrete_hysteresis_defaults_and_overrides() {
    use squid_n_core::model::HysteresisModel;

    let mut model = make_diaphragm_model();
    let col = ElementData {
        id: ElemId(0),
        kind: ElementKind::Fiber,
        nodes: smallvec::smallvec![NodeId(0), NodeId(2)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    model.elements.push(col.clone());

    assert_eq!(
        resolve_fiber_concrete_hysteresis(&col, &model, AnalysisKind::Incremental),
        HysteresisModel::Retrograde
    );
    assert_eq!(
        resolve_fiber_concrete_hysteresis(&col, &model, AnalysisKind::TimeHistory),
        HysteresisModel::KarsanJirsa
    );
    assert_eq!(
        resolve_wall_concrete_hysteresis(&col, &model, AnalysisKind::Incremental),
        HysteresisModel::OriginOriented
    );
    assert_eq!(
        resolve_wall_concrete_hysteresis(&col, &model, AnalysisKind::TimeHistory),
        HysteresisModel::KarsanJirsa
    );

    model.set_member_hysteresis(ElemId(0), HysteresisModel::Takeda);
    assert_eq!(
        resolve_fiber_concrete_hysteresis(&col, &model, AnalysisKind::Incremental),
        HysteresisModel::Retrograde
    );

    model.set_member_hysteresis(ElemId(0), HysteresisModel::OriginOriented);
    assert_eq!(
        resolve_fiber_concrete_hysteresis(&col, &model, AnalysisKind::Incremental),
        HysteresisModel::OriginOriented
    );
    assert_eq!(
        resolve_fiber_concrete_hysteresis(&col, &model, AnalysisKind::TimeHistory),
        HysteresisModel::OriginOriented
    );

    model.set_member_hysteresis_th(ElemId(0), Some(HysteresisModel::KarsanJirsa));
    assert_eq!(
        resolve_fiber_concrete_hysteresis(&col, &model, AnalysisKind::Incremental),
        HysteresisModel::OriginOriented
    );
    assert_eq!(
        resolve_fiber_concrete_hysteresis(&col, &model, AnalysisKind::TimeHistory),
        HysteresisModel::KarsanJirsa
    );
}

/// 耐震壁の面内せん断ばねの履歴則解決を検証する。
/// - 既定（Auto・コンクリート用指定）: 最大点指向型（増分・時刻歴共通）
/// - Q–δ 系の個別指定（標準型・武田型等）は尊重される
#[test]
fn test_resolve_wall_shear_hysteresis_defaults_and_overrides() {
    use squid_n_core::model::HysteresisModel;

    let mut model = make_diaphragm_model();
    let wall = ElementData {
        id: ElemId(0),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    model.elements.push(wall.clone());

    for kind in [AnalysisKind::Incremental, AnalysisKind::TimeHistory] {
        assert_eq!(
            resolve_wall_shear_hysteresis(&wall, &model, kind),
            HysteresisModel::MaxPointOriented
        );
    }

    model.set_member_hysteresis(ElemId(0), HysteresisModel::KarsanJirsa);
    assert_eq!(
        resolve_wall_shear_hysteresis(&wall, &model, AnalysisKind::Incremental),
        HysteresisModel::MaxPointOriented
    );
    assert_eq!(
        resolve_wall_concrete_hysteresis(&wall, &model, AnalysisKind::Incremental),
        HysteresisModel::KarsanJirsa
    );

    model.set_member_hysteresis(ElemId(0), HysteresisModel::Standard);
    assert_eq!(
        resolve_wall_shear_hysteresis(&wall, &model, AnalysisKind::Incremental),
        HysteresisModel::Standard
    );
}

/// 材端曲げバネの降伏時剛性低下率 αy。
/// - 形状未設定・非 RC 矩形・鉛直材（柱）は既定 0.3。
/// - RC 矩形の梁は菅野式。b=400・D=700・4-D22（1 段）・かぶり 50・帯筋 D10・
///   可撓長 5000・Ec=20000 のときの手計算値 αy≈0.19546
///   （pt=0.002715、a/D=3.571、d/D=629/700、n=10.25）。
#[test]
fn test_flexural_alpha_y_sugano_for_rc_beam() {
    use squid_n_core::section_shape::{RcBeamRebar, SectionShape};

    let mut model = make_diaphragm_model();
    let beam = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    model.elements.push(beam.clone());

    assert!((flexural_alpha_y(&beam, &model) - 0.3).abs() < 1e-12);

    let rebar = RcBeamRebar {
        main_dia: 22.0,
        top: vec![2],
        bottom: vec![2],
        cover: 50.0,
        stirrup: squid_n_core::section_shape::BeamStirrup {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
        },
    };
    model.sections[0].shape = Some(SectionShape::RcBeamRect {
        b: 400.0,
        d: 700.0,
        rebar,
    });
    let got = flexural_alpha_y(&beam, &model);
    assert!(
        (got - 0.195_459_048_474_485).abs() < 1e-12,
        "菅野式の αy: got={got}"
    );
    assert!(got > 0.0 && got < 1.0);
    assert!(
        (got - 0.3).abs() > 1e-3,
        "菅野式の値が既定 0.3 と区別できること（got={got}）"
    );

    let mut column = beam.clone();
    column.nodes = smallvec::smallvec![NodeId(0), NodeId(2)];
    assert!((flexural_alpha_y(&column, &model) - 0.3).abs() < 1e-12);
}

#[test]
fn test_rc_beam_flexural_spring_exhibits_takeda_degradation() {
    use squid_n_core::model::HysteresisModel;
    use squid_n_core::section_shape::{BeamStirrup, RcBeamRebar, SectionShape};

    let mut model = make_diaphragm_model();
    let beam = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    model.elements.push(beam.clone());
    model.sections[0].shape = Some(SectionShape::RcBeamRect {
        b: 400.0,
        d: 700.0,
        rebar: RcBeamRebar {
            main_dia: 22.0,
            top: vec![2],
            bottom: vec![2],
            cover: 50.0,
            stirrup: BeamStirrup {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
            },
        },
    });
    model.sections[0].depth = 700.0;
    model.sections[0].width = 400.0;
    model.sections[0].iz = 400.0 * 700.0f64.powi(3) / 12.0;
    model.materials[0].young = 25_000.0;
    model.materials[0].fc = Some(24.0);
    model.materials[0].fy = Some(345.0);

    let rule = resolve_member_hysteresis(&beam, &model, AnalysisKind::Incremental);
    assert_eq!(rule, HysteresisModel::Takeda);
    let (mut si, _sj, backbone) =
        build_flexural_springs(&beam, &model, rule, StrengthBasis::Nominal);
    assert!(!backbone.use_mn);

    let (_m0, k0) = si.trial(1e-8);
    si.commit();
    assert!(k0 > 0.0);

    let big = 0.02_f64;
    let (m_peak, _) = si.trial(big);
    si.commit();
    assert!(m_peak > 0.0, "should carry positive moment at peak");

    let (m1, _) = si.trial(big * 0.95);
    let (m2, _) = si.trial(big * 0.90);
    let ku = (m1 - m2) / (big * 0.05);
    assert!(
        ku < k0 * 0.999,
        "Takeda unloading stiffness ({ku}) must be below initial ({k0})"
    );
    assert!(ku > 0.0, "unloading stiffness must stay positive");
}

#[test]
fn test_steel_beam_flexural_spring_buckling_degrades() {
    use squid_n_core::model::HysteresisModel;
    use squid_n_core::section_shape::SectionShape;

    let mut model = make_diaphragm_model();
    let beam = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    model.elements.push(beam.clone());
    model.sections[0].shape = Some(SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    });
    model.sections[0].depth = 400.0;
    model.sections[0].width = 200.0;
    model.sections[0].iz = 200.0 * 400.0f64.powi(3) / 12.0;
    model.materials[0].fy = Some(325.0);
    model.set_member_hysteresis(ElemId(0), HysteresisModel::SteelBuckling);

    let rule = resolve_member_hysteresis(&beam, &model, AnalysisKind::Incremental);
    assert_eq!(rule, HysteresisModel::SteelBuckling);
    let (mut si, _sj, backbone) =
        build_flexural_springs(&beam, &model, rule, StrengthBasis::Nominal);
    assert!(
        backbone.use_mn,
        "座屈考慮型は set_yield 対応で N-M 相関適用可"
    );

    let (_m0, k0) = si.trial(1e-9);
    si.commit();
    assert!(k0 > 0.0);
    let theta_y = { 1e-3 };
    let mut m_max = 0.0_f64;
    let mut m_last = 0.0_f64;
    for i in 1..=200 {
        let th = theta_y * i as f64 * 0.5;
        let (m, _) = si.trial(th);
        si.commit();
        m_max = m_max.max(m);
        m_last = m;
    }
    assert!(m_max > 0.0);
    assert!(
        m_last < m_max * 0.999,
        "buckling degradation: last M ({m_last}) must fall below peak ({m_max})"
    );
}
