use super::*;
use crate::behavior::Ctx;
use crate::factory::StrengthBasis;
use approx::assert_relative_eq;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node, Section,
};

fn make_test_fiber_beam(shear_mod: Option<f64>) -> FiberBeam {
    let model = build_test_model(shear_mod);
    FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal)
}

fn make_test_beam_element(as_val: f64) -> crate::beam::BeamElement {
    crate::beam::BeamElement {
        id: ElemId(0),
        e: 205000.0,
        g: 78846.15,
        a: 20000.0,
        a_mass: 20000.0,
        // 要素座標系のフィールド値: せい 200（ローカル y 方向）× 幅 100 の矩形。
        // iz（Mz 面、∫y²dA）=強軸 100·200³/12、iy（My 面、∫z²dA）=弱軸 200·100³/12。
        iy: 16666666.66666667,
        iz: 66666666.66666667,
        j: 0.0,
        as_y: as_val,
        as_z: as_val,
        length: 3000.0,
        density: 0.0,
        nodes: [NodeId(0), NodeId(1)],
        axis: crate::transform::LocalFrame {
            rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        },
        rigid: squid_n_core::model::RigidZone::default(),
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        eval_sections: vec![],
        section: None,
        material: None,
        committed_disp: [0.0; 12],
        trial_disp: [0.0; 12],
    }
}

fn build_test_model(shear_mod: Option<f64>) -> Model {
    Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [3000.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Fiber,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![Section {
            id: SectionId(0),
            name: "test".to_string(),
            area: 20000.0,
            iy: 66666666.66666667,
            iz: 16666666.66666667,
            j: 0.0,
            depth: 200.0,
            width: 100.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "steel".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: shear_mod,
            fc: None,
            fy: None,
        }],
        ..Default::default()
    }
}

/// 指定した2節点座標・参照ベクトルで FiberBeam を生成するヘルパ（座標変換テスト用）。
fn make_oriented_fiber(p0: [f64; 3], p1: [f64; 3], ref_vec: [f64; 3]) -> FiberBeam {
    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: p0,
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: p1,
                restraint: Default::default(),
                mass: None,
                story: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Fiber,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: ref_vec,
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![Section {
            id: SectionId(0),
            name: "s".to_string(),
            area: 20000.0,
            iy: 66666666.66666667,
            iz: 16666666.66666667,
            j: 0.0,
            depth: 200.0,
            width: 100.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "steel".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: Some(0.0),
            fc: None,
            fy: None,
        }],
        ..Default::default()
    };
    FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal)
}

/// 降伏応力 fy を指定した鋼材ファイバ梁（X 整列・恒等フレーム）を生成するヘルパ。
fn make_steel_fiber_with_fy(fy: Option<f64>) -> FiberBeam {
    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [3000.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Fiber,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![Section {
            id: SectionId(0),
            name: "s".to_string(),
            area: 20000.0,
            iy: 66666666.66666667,
            iz: 16666666.66666667,
            j: 0.0,
            depth: 200.0,
            width: 100.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "steel".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: Some(0.0),
            fc: None,
            fy,
        }],
        ..Default::default()
    };
    FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal)
}

/// ねじり剛性テスト用の FiberBeam を生成する。
/// 既知の G, J, L で Saint-Venant ねじり剛性を検証するため。
fn make_torsion_fiber_beam(g: f64, j: f64) -> FiberBeam {
    let mut model = build_test_model(Some(g));
    model.sections[0].j = j;
    FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal)
}

/// 降伏データ検証: Material.fy を与えた鋼材ファイバは、同一の大曲率変形に対して
/// 弾性材（fy 無し＝1e20）より小さい曲げ内力を示す（＝実際に降伏している）。
#[test]
fn test_fiber_steel_yields_with_fy() {
    let ctx = Ctx {
        model: &Model::default(),
    };
    // 端部 ry に十分大きな逆対称回転を与え、曲げで降伏させる。
    // My 面の縁距離は幅/2=50mm（ファイバ座標は要素座標系: y=せい・z=幅）のため、
    // 降伏後モーメントが弾性値の 1/2 を明確に下回るだけの曲率倍率を確保する。
    let big = 0.2;
    let du = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, big, 0.0, 0.0, 0.0, 0.0, 0.0, -big, 0.0],
    };

    let mut yielding = make_steel_fiber_with_fy(Some(235.0));
    yielding.update_state(&du, true, &ctx);
    let f_y = yielding.internal_force(&ElemState::default(), &ctx);

    let mut elastic = make_steel_fiber_with_fy(None);
    elastic.update_state(&du, true, &ctx);
    let f_e = elastic.internal_force(&ElemState::default(), &ctx);

    // 曲げモーメント DOF(ry_i = index 4) で比較。降伏材は弾性材より明確に小さいこと。
    assert!(
        f_e.data[4].abs() > 1.0,
        "elastic bending moment must be non-trivial (test sanity): {}",
        f_e.data[4]
    );
    assert!(
        f_y.data[4].abs() < f_e.data[4].abs() * 0.5,
        "yielding moment {} should be well below elastic {} (fy plumbing inactive?)",
        f_y.data[4],
        f_e.data[4]
    );
}

/// 座標変換の検証: 軸方向（X 整列）と鉛直柱（Z 整列）でグローバル接線剛性を比較し、
/// 軸剛性・曲げ剛性が正しいグローバル DOF へ写像されることを確認する。
/// 回転変換が欠落していると鉛直柱の水平 DOF に軸剛性が誤って現れる。
#[test]
fn test_global_rotation_vertical_column() {
    let l = 3000.0;
    let ctx = Ctx {
        model: &Model::default(),
    };
    let zero_du = LocalVec {
        data: SmallVec::from_elem(0.0, 12),
    };
    // X 整列（ref [0,1,0] で恒等フレーム）: local x = global X(軸), local y = global Y(曲げ)
    let mut fx = make_oriented_fiber([0.0, 0.0, 0.0], [l, 0.0, 0.0], [0.0, 1.0, 0.0]);
    fx.update_state(&zero_du, false, &ctx); // 初期接線（弾性係数）をキャッシュへ
    let kx = fx.tangent_stiffness(&ElemState::default(), &ctx);
    // Z 整列（鉛直柱, ref [1,0,0]）: local x = global Z(軸), local y = global X(曲げ)
    let mut fz = make_oriented_fiber([0.0, 0.0, 0.0], [0.0, 0.0, l], [1.0, 0.0, 0.0]);
    fz.update_state(&zero_du, false, &ctx);
    let kz = fz.tangent_stiffness(&ElemState::default(), &ctx);

    // 軸剛性: X 整列の ux_i (DOF0) == Z 整列の uz_i (DOF2)
    assert_relative_eq!(kz.get(2, 2), kx.get(0, 0), epsilon = 1.0);
    // 曲げ剛性: X 整列の uy_i (DOF1, local 曲げ) == Z 整列の ux_i (DOF0, local 曲げ)
    assert_relative_eq!(kz.get(0, 0), kx.get(1, 1), epsilon = 1.0);
    // 鉛直柱の水平 DOF は曲げ剛性（小）であって軸剛性（大）ではないこと
    assert!(
        kz.get(0, 0) < kz.get(2, 2),
        "vertical column horizontal DOF must be bending (small), not axial (large): ux={}, uz={}",
        kz.get(0, 0),
        kz.get(2, 2)
    );
}

#[test]
fn test_elastic_stiffness_matches_beam() {
    let mut fiber = make_test_fiber_beam(Some(0.0));
    let beam = make_test_beam_element(1e30);

    let ctx = Ctx {
        model: &build_test_model(Some(0.0)),
    };
    let state = ElemState::default();

    let u = [
        1.0, 0.5, 0.3, 0.0, 0.001, 0.002, -0.5, 0.2, -0.1, 0.0, 0.003, -0.001,
    ];
    let du = LocalVec {
        data: SmallVec::from_slice(&u),
    };
    fiber.update_state(&du, true, &ctx);

    let k_fiber = fiber.tangent_stiffness(&state, &ctx);
    let k_beam = beam.local_stiffness_raw();

    for i in 0..12 {
        for j in 0..12 {
            let expected = k_beam.get(i, j);
            let actual = k_fiber.get(i, j);
            if expected.abs() > 1e-6 {
                assert_relative_eq!(actual, expected, max_relative = 0.01);
            } else {
                assert!(
                    actual.abs() < 1e-3,
                    "K[{i}][{j}] zero expected, got {actual}"
                );
            }
        }
    }
}

#[test]
fn test_elastic_stiffness_symmetric() {
    let mut fiber = make_test_fiber_beam(Some(0.0));
    let ctx = Ctx {
        model: &build_test_model(Some(0.0)),
    };
    let state = ElemState::default();

    let u = [
        1.0, 0.5, 0.3, 0.0, 0.001, 0.002, -0.5, 0.2, -0.1, 0.0, 0.003, -0.001,
    ];
    let du = LocalVec {
        data: SmallVec::from_slice(&u),
    };
    fiber.update_state(&du, true, &ctx);

    let k = fiber.tangent_stiffness(&state, &ctx);
    for i in 0..12 {
        for j in 0..12 {
            assert!(
                (k.get(i, j) - k.get(j, i)).abs() < 1e-9,
                "K[{i}][{j}] != K[{j}][{i}]: {} vs {}",
                k.get(i, j),
                k.get(j, i)
            );
        }
    }
}

#[test]
fn test_axial_response() {
    let mut fiber = make_test_fiber_beam(Some(0.0));
    let ctx = Ctx {
        model: &build_test_model(Some(0.0)),
    };
    let state = ElemState::default();

    let eps0 = 0.001;
    let du = LocalVec {
        data: SmallVec::from_slice(&[
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            eps0 * 3000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]),
    };
    fiber.update_state(&du, true, &ctx);

    let f = fiber.internal_force(&state, &ctx);
    let a_disc: f64 = fiber.gauss_points[0]
        .section
        .fibers
        .iter()
        .map(|f| f.area)
        .sum();
    let expected_n = eps0 * 205000.0 * a_disc;
    assert_relative_eq!(f.data[0], -expected_n, epsilon = 1.0);
    assert_relative_eq!(f.data[6], expected_n, epsilon = 1.0);
}

#[test]
fn test_pure_bending_mphi() {
    let mut fiber = make_test_fiber_beam(Some(0.0));
    let ctx = Ctx {
        model: &build_test_model(Some(0.0)),
    };
    let state = ElemState::default();

    let ky = 1e-6;
    let du = LocalVec {
        data: SmallVec::from_slice(&[
            0.0,
            0.0,
            0.0,
            0.0,
            ky * 3000.0 / 2.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            -ky * 3000.0 / 2.0,
            0.0,
        ]),
    };
    fiber.update_state(&du, true, &ctx);

    let f = fiber.internal_force(&state, &ctx);
    let iy_disc: f64 = fiber.gauss_points[0]
        .section
        .fibers
        .iter()
        .map(|f| f.area * f.z * f.z)
        .sum();
    let expected_my = ky * 205000.0 * iy_disc;
    assert_relative_eq!(f.data[4], expected_my, epsilon = 1.0);
    assert_relative_eq!(f.data[10], -expected_my, epsilon = 1.0);
}

#[test]
fn test_n_m_interaction() {
    let mut fiber = make_test_fiber_beam(Some(0.0));
    let ctx = Ctx {
        model: &build_test_model(Some(0.0)),
    };
    let state = ElemState::default();

    let eps0 = 0.0005;
    let ky = 1e-6;
    let du = LocalVec {
        data: SmallVec::from_slice(&[
            0.0,
            0.0,
            0.0,
            0.0,
            ky * 3000.0 / 2.0,
            0.0,
            eps0 * 3000.0,
            0.0,
            0.0,
            0.0,
            -ky * 3000.0 / 2.0,
            0.0,
        ]),
    };
    fiber.update_state(&du, true, &ctx);

    let f = fiber.internal_force(&state, &ctx);
    let a_disc: f64 = fiber.gauss_points[0]
        .section
        .fibers
        .iter()
        .map(|f| f.area)
        .sum();
    let iy_disc: f64 = fiber.gauss_points[0]
        .section
        .fibers
        .iter()
        .map(|f| f.area * f.z * f.z)
        .sum();
    let expected_n = eps0 * 205000.0 * a_disc;
    let expected_my = ky * 205000.0 * iy_disc;
    assert_relative_eq!(f.data[0], -expected_n, epsilon = 1.0);
    assert_relative_eq!(f.data[4], expected_my, epsilon = 1.0);
}

#[test]
fn test_yield_progression() {
    let mut fiber = {
        let model = Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [3000.0, 0.0, 0.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Fiber,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 1.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "yield_test".to_string(),
                area: 20000.0,
                iy: 66666666.66666667,
                iz: 16666666.66666667,
                j: 0.0,
                depth: 200.0,
                width: 100.0,
                as_y: 0.0,
                as_z: 0.0,
                panel_thickness: None,
                thickness: None,
                shape: None,
            }],
            materials: vec![Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "steel".to_string(),
                young: 205000.0,
                poisson: 0.3,
                density: 0.0,
                shear: Some(0.0),
                fc: None,
                // fy 未設定だと Bilinear の降伏点が 1e20 となり降伏しない
                // （テストが恒等比較になってしまう）ため明示する。
                fy: Some(235.0),
            }],
            ..Default::default()
        };
        FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal)
    };

    let ctx = Ctx {
        model: &Model::default(),
    };
    let state = ElemState::default();

    let eps_y = 235.0 / 205000.0;
    // My 面（κy）の縁距離はファイバ座標の |z| 最大 = 幅/2 = 50mm
    // （ファイバ座標は要素座標系: y=せい・z=幅）。
    let z_max = 50.0;
    let ky_y = eps_y / z_max;
    let ky_final = ky_y * 3.0;

    let mut last_my = 0.0;
    let n_steps = 50;
    let mut prev_ky = 0.0;
    for i in 1..=n_steps {
        let ky_curr = ky_final * (i as f64) / (n_steps as f64);
        let dky = ky_curr - prev_ky;
        prev_ky = ky_curr;
        let du = LocalVec {
            data: SmallVec::from_slice(&[
                0.0,
                0.0,
                0.0,
                0.0,
                dky * 3000.0 / 2.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -dky * 3000.0 / 2.0,
                0.0,
            ]),
        };
        fiber.update_state(&du, true, &ctx);

        let f = fiber.internal_force(&state, &ctx);
        last_my = f.data[4];
    }

    let iy_disc: f64 = fiber.gauss_points[0]
        .section
        .fibers
        .iter()
        .map(|f| f.area * f.z * f.z)
        .sum();
    let elastic_pred = ky_final * 205000.0 * iy_disc;
    assert!(
        last_my < elastic_pred,
        "post-yield My ({}) must be below elastic prediction ({})",
        last_my,
        elastic_pred
    );
}

#[test]
fn test_commit_revert() {
    let mut fiber = make_test_fiber_beam(Some(0.0));
    let ctx = Ctx {
        model: &build_test_model(Some(0.0)),
    };

    let du = LocalVec {
        data: SmallVec::from_slice(&[0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    };

    fiber.update_state(&du, false, &ctx);
    assert_relative_eq!(fiber.trial_disp[4], 0.001, epsilon = 1e-12);
    assert_relative_eq!(fiber.committed_disp[4], 0.0, epsilon = 1e-12);
    fiber.revert_state();
    assert_relative_eq!(fiber.trial_disp[4], 0.0, epsilon = 1e-12);
    assert_relative_eq!(fiber.committed_disp[4], 0.0, epsilon = 1e-12);

    fiber.update_state(&du, false, &ctx);
    fiber.commit_state();
    assert_relative_eq!(fiber.trial_disp[4], 0.001, epsilon = 1e-12);
    assert_relative_eq!(fiber.committed_disp[4], 0.001, epsilon = 1e-12);

    let du2 = LocalVec {
        data: SmallVec::from_slice(&[0.0, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    };
    fiber.update_state(&du2, false, &ctx);
    assert_relative_eq!(fiber.trial_disp[4], 0.003, epsilon = 1e-12);
    fiber.revert_state();
    assert_relative_eq!(fiber.trial_disp[4], 0.001, epsilon = 1e-12);
    assert_relative_eq!(fiber.committed_disp[4], 0.001, epsilon = 1e-12);
}

#[test]
fn test_snapshot_restore() {
    let mut fiber = make_test_fiber_beam(Some(0.0));
    let ctx = Ctx {
        model: &build_test_model(Some(0.0)),
    };

    let du = LocalVec {
        data: SmallVec::from_slice(&[0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    };
    fiber.update_state(&du, true, &ctx);
    let snap = fiber.snapshot_state();

    let du2 = LocalVec {
        data: SmallVec::from_slice(&[0.0, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    };
    fiber.update_state(&du2, false, &ctx);
    assert_relative_eq!(fiber.trial_disp[4], 0.003, epsilon = 1e-12);

    fiber.restore_state(&*snap);
    assert_relative_eq!(fiber.trial_disp[4], 0.001, epsilon = 1e-12);
    assert_relative_eq!(fiber.committed_disp[4], 0.001, epsilon = 1e-12);
}

#[test]
fn test_geometric_stiffness() {
    let fiber = make_test_fiber_beam(Some(0.0));
    let n = 100000.0;
    let kg = fiber.geometric_stiffness(n);
    let l = fiber.length;
    let c = n / l;
    assert_relative_eq!(kg.get(1, 1), c * 6.0 / 5.0, epsilon = 1e-9);
    assert_relative_eq!(kg.get(5, 5), c * 2.0 * l * l / 15.0, epsilon = 1e-9);
    assert_relative_eq!(kg.get(4, 4), c * 2.0 * l * l / 15.0, epsilon = 1e-9);
    assert_relative_eq!(kg.get(2, 4), -c * l / 10.0, epsilon = 1e-9);
}

#[test]
fn test_internal_force_zero_at_zero_disp() {
    let fiber = make_test_fiber_beam(None);
    let f = fiber.internal_force(
        &ElemState::default(),
        &Ctx {
            model: &Model::default(),
        },
    );
    for v in f.data.iter() {
        assert!(v.abs() < 1e-12, "zero disp should give zero force, got {v}");
    }
}

#[test]
fn test_fiber_section_area_matches_section() {
    let fiber = make_test_fiber_beam(None);
    let a_disc: f64 = fiber.gauss_points[0]
        .section
        .fibers
        .iter()
        .map(|f| f.area)
        .sum();
    let expected = 100.0 * 200.0;
    assert_relative_eq!(a_disc, expected, max_relative = 0.01);
}

#[test]
fn test_update_state_trial_stress_nonzero() {
    let mut fiber = make_test_fiber_beam(Some(0.0));
    let ctx = Ctx {
        model: &build_test_model(Some(0.0)),
    };

    let du = LocalVec {
        data: SmallVec::from_slice(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
    };
    fiber.update_state(&du, false, &ctx);

    for gp in &fiber.gauss_points {
        for &s in &gp.trial_stress {
            assert!(
                s.abs() > 0.0,
                "trial_stress should be nonzero after axial disp"
            );
        }
    }
}

#[test]
fn test_different_gp_have_independent_mats() {
    let fiber = make_test_fiber_beam(Some(0.0));
    let gp0_ptr = &fiber.gauss_points[0].mats[0] as *const _;
    let gp1_ptr = &fiber.gauss_points[1].mats[0] as *const _;
    assert_ne!(gp0_ptr, gp1_ptr, "GP mats must be independent instances");
}

#[test]
fn test_torsional_stiffness() {
    let g = 78846.0;
    let j = 1.0e6;
    let l = 3000.0;
    let expected_kt = g * j / l;

    let mut fiber = make_torsion_fiber_beam(g, j);
    let ctx = Ctx {
        model: &build_test_model(Some(g)),
    };
    // 接線キャッシュを初期化（ゼロ変位で update_state）
    let zero_du = LocalVec {
        data: SmallVec::from_elem(0.0, 12),
    };
    fiber.update_state(&zero_du, false, &ctx);

    let k = fiber.tangent_stiffness(&ElemState::default(), &ctx);
    assert!(
        (k.get(3, 3) - expected_kt).abs() < 1e-6 * expected_kt.max(1.0),
        "K[3][3] should be G*J/L: expected {}, got {}",
        expected_kt,
        k.get(3, 3)
    );
    assert!(
        (k.get(9, 9) - expected_kt).abs() < 1e-6 * expected_kt.max(1.0),
        "K[9][9] should be G*J/L: expected {}, got {}",
        expected_kt,
        k.get(9, 9)
    );
    assert!(
        (k.get(3, 9) + expected_kt).abs() < 1e-6 * expected_kt.max(1.0),
        "K[3][9] should be -G*J/L: expected {}, got {}",
        -expected_kt,
        k.get(3, 9)
    );
    assert!(
        (k.get(9, 3) + expected_kt).abs() < 1e-6 * expected_kt.max(1.0),
        "K[9][3] should be -G*J/L: expected {}, got {}",
        -expected_kt,
        k.get(9, 3)
    );
}

#[test]
fn test_torsional_internal_force() {
    let g = 78846.0;
    let j = 1.0e6;
    let l = 3000.0;
    let kt = g * j / l;

    let mut fiber = make_torsion_fiber_beam(g, j);
    let ctx = Ctx {
        model: &build_test_model(Some(g)),
    };
    let theta_i = 0.01;
    let theta_j = -0.005;
    let du = LocalVec {
        data: smallvec::smallvec![
            0.0, 0.0, 0.0, theta_i, 0.0, 0.0, 0.0, 0.0, 0.0, theta_j, 0.0, 0.0,
        ],
    };
    fiber.update_state(&du, true, &ctx);
    let f = fiber.internal_force(&ElemState::default(), &ctx);

    let expected_mx_i = kt * (theta_i - theta_j);
    assert!(
        (f.data[3] - expected_mx_i).abs() < 1e-6 * expected_mx_i.abs().max(1.0),
        "Mx_i should be kt*(θ_i - θ_j): expected {}, got {}",
        expected_mx_i,
        f.data[3]
    );
    assert!(
        (f.data[9] + expected_mx_i).abs() < 1e-6 * expected_mx_i.abs().max(1.0),
        "Mx_j should be -Mx_i: expected {}, got {}",
        -expected_mx_i,
        f.data[9]
    );
}

/// 鉛直柱（Z整列）でねじり剛性 GJ 追加後、グローバル rz DOF (index 5, 11) が
/// 特異でない（非ゼロの対角成分を持つ）ことを確認する回帰テスト。
/// 以前は rz 拘束が無いと特異化していた。
#[test]
fn test_vertical_column_rz_nonsingular() {
    let g = 78846.0;
    let j = 1.0e6;
    let l = 3000.0;
    let expected_kt = g * j / l;

    // Z 整列（鉛直柱）: local x = global Z
    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, l],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Fiber,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![Section {
            id: SectionId(0),
            name: "col".to_string(),
            area: 10000.0,
            iy: 8.333e6,
            iz: 8.333e6,
            j,
            depth: 100.0,
            width: 100.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "steel".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: Some(g),
            fc: None,
            fy: None,
        }],
        ..Default::default()
    };

    let mut fiber = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);
    let ctx = Ctx {
        model: &Model::default(),
    };
    let zero_du = LocalVec {
        data: SmallVec::from_elem(0.0, 12),
    };
    fiber.update_state(&zero_du, false, &ctx);

    let k = fiber.tangent_stiffness(&ElemState::default(), &ctx);
    // 鉛直柱では local rx が global rz に回転される。
    // global rz は節点自由度 index 5 (i端) と index 11 (j端)。
    let k55 = k.get(5, 5);
    let k11_11 = k.get(11, 11);
    assert!(
        k55 > 0.0,
        "global rz_i (k[5][5]) must be > 0 with torsion stiffness, got {}",
        k55
    );
    assert!(
        k11_11 > 0.0,
        "global rz_j (k[11][11]) must be > 0 with torsion stiffness, got {}",
        k11_11
    );
    // ねじり剛性が回転後も正しく伝わっていることの緩い確認
    let _ = expected_kt;
}

/// 回帰テスト: 剛体回転（両端に同じ回転角 θ、曲率ゼロ）だけを与えても
/// 内力が発生しないこと（客観性）。かつて曲げ剛性へ並列加算していた独立
/// せん断ばねは、端部並進差 uy_j−uy_i=θ・L を誤ってせん断変形とみなし
/// 偽の内力を出していた（GAs/L・θL のオーダー、有効せん断断面積が大きい
/// 断面ほど顕著）。
#[test]
fn test_fiber_rigid_rotation_produces_no_force() {
    // 有効せん断断面積を大きく取り、旧実装ならせん断ばね寄与が支配的に
    // なる条件（矩形断面 500x500 相当）で検証する。
    let mut model = build_test_model(Some(78846.15));
    model.sections[0].as_y = 208333.0;
    model.sections[0].as_z = 208333.0;
    model.sections[0].depth = 500.0;
    model.sections[0].width = 500.0;
    model.sections[0].area = 250000.0;
    model.sections[0].iy = 5.2083333e9;
    model.sections[0].iz = 5.2083333e9;

    let mut fiber = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);
    let ctx = Ctx { model: &model };

    let theta = 1.0e-4;
    let l = 3000.0;
    let du = LocalVec {
        data: SmallVec::from_slice(&[
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            theta,
            0.0,
            theta * l,
            0.0,
            0.0,
            0.0,
            theta,
        ]),
    };
    fiber.update_state(&du, false, &ctx);
    let f = fiber.internal_force(&ElemState::default(), &ctx);
    // 許容値 1.0 の根拠: 旧実装の偽せん断力は GAs/L・θL ≈ 1.6e6 N、正常時は
    // 丸め誤差（~1e-7）で、判定は 6 桁以上の余裕を持つ。並進 [N]・回転 [N·mm]
    // の単位混在は、双方とも「ほぼゼロ vs 1e6 以上」の判別であり問題にならない。
    for (i, v) in f.data.iter().enumerate() {
        assert!(
            v.abs() < 1.0,
            "剛体回転のみで内力が発生した（客観性違反）: dof {i} = {v}"
        );
    }
}

/// 回帰テスト: 弾性状態の初期横剛性が Timoshenko 理論値と一致すること。
/// かつての並列せん断ばね（GAs/L を並進 DOF へ直接加算）は片持ち先端剛性を
/// 理論値の数十倍にしていた。本テストは i 端固定の片持ち縮約剛性
/// k = 1/(L³/3EI + L/GAs)（先端モーメントフリー、曲げ＋せん断の直列）を
/// 照合し、GAs/L オーダーの過大剛性の再混入と、せん断柔性の欠落
/// （Euler 化 = 理論比 1+φ/... の過大）の両方を検出する。
#[test]
fn test_fiber_initial_lateral_stiffness_matches_timoshenko_theory() {
    let mut model = build_test_model(Some(78846.15));
    model.sections[0].as_y = 208333.0;
    model.sections[0].as_z = 208333.0;
    model.sections[0].depth = 500.0;
    model.sections[0].width = 500.0;
    model.sections[0].area = 250000.0;
    model.sections[0].iy = 5.2083333e9;
    model.sections[0].iz = 5.2083333e9;

    let mut fiber = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);
    let ctx = Ctx { model: &model };
    let zero = LocalVec {
        data: SmallVec::from_elem(0.0, 12),
    };
    fiber.update_state(&zero, false, &ctx); // 初期弾性接線をキャッシュへ
    let k = fiber.tangent_stiffness(&ElemState::default(), &ctx);

    // 片持ち（i端固定）の j 端 [uy, rz] 2x2 ブロックを縮約し、
    // 先端モーメントフリーの並進剛性 k_tip = det/K(rz,rz) を求める。
    let a = k.get(7, 7);
    let b = k.get(7, 11);
    let c = k.get(11, 11);
    let k_tip = (a * c - b * b) / c;

    let e = 205000.0;
    let g = 78846.15;
    let l: f64 = 3000.0;
    let ei = e * 5.2083333e9;
    let gas = g * 208333.0;
    let k_timo = 1.0 / (l.powi(3) / (3.0 * ei) + l / gas);
    // ファイバー離散化（12x20 格子の図心集中）による EI の僅かな目減り
    // （1−1/nd² ≈ 0.9975）を含めて 1% 以内で一致すること。
    // 旧実装の並列せん断ばね混入時は k_tip ≈ GAs/L ≈ 47×k_timo で大きく外れ、
    // せん断柔性の欠落（Euler 化）時は約 +9% 外れる（いずれも許容 1% 超）。
    approx::assert_relative_eq!(k_tip, k_timo, max_relative = 0.01);
}

/// 受け入れテスト（Timoshenko 適合内挿）: 弾性状態の 12×12 接線剛性が
/// 弾性 Timoshenko 梁 `BeamElement` と厳密一致すること。
/// **非対称断面**（幅 300×せい 600、as_y≠as_z）を用い、断面レイヤ→要素座標系の
/// クロス変換（強軸 (uy,rz) ← 断面 iy・as_z / 弱軸 (uz,ry) ← 断面 iz・as_y）の
/// 取り違えも検出する。
/// ファイバー格子は面積を図心集中させるため EI が僅かに目減りする
/// （格子回転後の要素座標系で、強軸 1−1/nd²、弱軸 1−1/nw²）。比較対象の
/// BeamElement には格子の離散 EI と同じ値（要素座標系）を与え、離散化誤差と
/// 定式化誤差を分離して定式化の厳密一致を検証する。許容値は max|K| を基準と
/// した絶対許容 1e-9·max|K|（実測差は ~1e-16·max|K| で機械精度一致）。
#[test]
fn test_fiber_elastic_stiffness_matches_timoshenko_beam_element() {
    let g = 78846.15;
    let (b_w, d_h): (f64, f64) = (300.0, 600.0);
    let (nw, nd) = (12.0, 20.0);
    let area = b_w * d_h;
    // 格子の離散断面二次モーメント（要素座標系。格子は 90° 回転され
    // 要素 y=せい方向・z=幅方向となるため、強軸＝要素 z 軸まわり（∫y²dA）は
    // せい方向分割 nd、弱軸＝要素 y 軸まわり（∫z²dA）は幅方向分割 nw が効く）
    let iz_elem = b_w * d_h.powi(3) / 12.0 * (1.0 - 1.0 / (nd * nd)); // 強軸 (uy,rz)
    let iy_elem = d_h * b_w.powi(3) / 12.0 * (1.0 - 1.0 / (nw * nw)); // 弱軸 (uz,ry)
                                                                      // 要素座標系のせん断有効断面積（意図的に非対称）
    let as_y_elem = 120000.0; // (uy,rz) 面
    let as_z_elem = 80000.0; // (uz,ry) 面
    let j = 1.0e6;

    let mut model = build_test_model(Some(g));
    model.sections[0].depth = d_h;
    model.sections[0].width = b_w;
    model.sections[0].area = area;
    // 断面レイヤ諸元（クロス変換で要素座標系に対応: iy_sec→要素(uy,rz)、
    // as_z_sec→要素(uy,rz)。BeamElement 側は要素座標系の値を直接持たせる）
    model.sections[0].iy = iz_elem;
    model.sections[0].iz = iy_elem;
    model.sections[0].as_z = as_y_elem;
    model.sections[0].as_y = as_z_elem;
    model.sections[0].j = j;

    let mut fiber = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);
    let ctx = Ctx { model: &model };
    let zero = LocalVec {
        data: SmallVec::from_elem(0.0, 12),
    };
    fiber.update_state(&zero, false, &ctx);
    let k_fb = fiber.tangent_stiffness(&ElemState::default(), &ctx);

    let mut be = make_test_beam_element(as_y_elem);
    be.a = area;
    be.a_mass = area;
    be.iy = iy_elem;
    be.iz = iz_elem;
    be.j = j;
    be.as_y = as_y_elem;
    be.as_z = as_z_elem;
    let k_be = be.tangent_stiffness(&ElemState::default(), &ctx);

    let kmax = (0..12)
        .flat_map(|i| (0..12).map(move |j| (i, j)))
        .map(|(i, j)| k_be.get(i, j).abs())
        .fold(0.0_f64, f64::max);
    for i in 0..12 {
        for j in 0..12 {
            let diff = (k_fb.get(i, j) - k_be.get(i, j)).abs();
            assert!(
                diff <= 1e-9 * kmax,
                "K({i},{j}) が Timoshenko 梁と不一致: fiber={}, beam={}, 差={diff:.3e}",
                k_fb.get(i, j),
                k_be.get(i, j)
            );
        }
    }
}

/// 500角・as_y/as_z 付き（φ>0）の断面パラメータをテストモデルへ設定する。
fn set_square500_shear_section(model: &mut Model) {
    model.sections[0].depth = 500.0;
    model.sections[0].width = 500.0;
    model.sections[0].area = 250000.0;
    model.sections[0].iy = 5.2083333e9;
    model.sections[0].iz = 5.2083333e9;
    model.sections[0].as_y = 208333.0;
    model.sections[0].as_z = 208333.0;
}

/// 塑性化域考慮モデルでも φ>0 の Timoshenko 適合内挿が機能すること:
/// (1) 剛体回転で内力ゼロ（客観性）、(2) 接線と内力の FD 整合、
/// (3) 片持ち先端剛性が Timoshenko 理論値の近傍にあること。
/// 端部を 1 点端点則で積分するため厳密一致はせず（曲げ剛性が数%過大）、
/// (3) は「理論値の 0.95〜1.15 倍」の帯で判定する（GAs/L 混入時は ~47 倍、
/// せん断柔性欠落（Euler 化）時は 1+φ/4 ≈ 1.09 倍＋端点則の過大が乗るため
/// 帯の上限は端点則ぶんを含む値とする）。
#[test]
fn test_plastic_zone_phi_positive_timoshenko_behavior() {
    let mut model = build_test_model(Some(78846.15));
    set_square500_shear_section(&mut model);
    model.elements[0].plastic_zone = Some(250.0);
    let ctx = Ctx { model: &model };
    let state = ElemState::default();
    let build =
        || FiberBeam::with_plastic_zone(&model.elements[0], &model, 250.0, StrengthBasis::Nominal);

    // (1) 剛体回転の客観性
    let theta = 1.0e-4;
    let l = 3000.0;
    let mut fb = build();
    let du = LocalVec {
        data: SmallVec::from_slice(&[
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            theta,
            0.0,
            theta * l,
            0.0,
            0.0,
            0.0,
            theta,
        ]),
    };
    fb.update_state(&du, false, &ctx);
    let f = fb.internal_force(&state, &ctx);
    for (i, v) in f.data.iter().enumerate() {
        assert!(v.abs() < 1.0, "塑性化域+φ>0 で客観性違反: dof {i} = {v}");
    }

    // (2) FD 整合（弾性域の代表変形状態）
    let h = 1e-6;
    let u0: [f64; 12] = [
        0.1, 0.2, -0.1, 0.0005, 0.001, -0.0005, -0.05, 0.15, 0.1, -0.0005, 0.0008, 0.0002,
    ];
    let mut b0 = build();
    b0.update_state(
        &LocalVec {
            data: SmallVec::from_slice(&u0),
        },
        false,
        &ctx,
    );
    let f0 = b0.internal_force(&state, &ctx);
    let k = b0.tangent_stiffness(&state, &ctx);
    let kmax = (0..12)
        .flat_map(|i| (0..12).map(move |j| (i, j)))
        .map(|(i, j)| k.get(i, j).abs())
        .fold(0.0_f64, f64::max);
    for j in 0..12 {
        let mut up = u0;
        up[j] += h;
        let mut bp = build();
        bp.update_state(
            &LocalVec {
                data: SmallVec::from_slice(&up),
            },
            false,
            &ctx,
        );
        let fp = bp.internal_force(&state, &ctx);
        for i in 0..12 {
            let fd = (fp.data[i] - f0.data[i]) / h;
            let err = (fd - k.get(i, j)).abs() / kmax;
            assert!(
                err < 1e-6,
                "塑性化域+φ>0 で K≠∂f/∂u: ({i},{j}) 誤差={err:.3e}"
            );
        }
    }

    // (3) 片持ち先端剛性が Timoshenko 理論値の近傍（端点則の過大を許容）
    let mut fb2 = build();
    let zero = LocalVec {
        data: SmallVec::from_elem(0.0, 12),
    };
    fb2.update_state(&zero, false, &ctx);
    let k2 = fb2.tangent_stiffness(&state, &ctx);
    let a = k2.get(7, 7);
    let b = k2.get(7, 11);
    let c = k2.get(11, 11);
    let k_tip = (a * c - b * b) / c;
    let ei = 205000.0 * 5.2083333e9;
    let gas = 78846.15 * 208333.0;
    let k_timo = 1.0 / (l.powi(3) / (3.0 * ei) + l / gas);
    let ratio = k_tip / k_timo;
    assert!(
        (0.95..1.15).contains(&ratio),
        "塑性化域+φ>0 の先端剛性が理論値帯を外れた: ratio={ratio}"
    );
}

/// 整合性テスト: 接線剛性 K が内力 f_int の微分 ∂f/∂u と一致すること
/// （有限差分照合）。K ≠ ∂f/∂u の要素が混ざると Newton 反復が二次収束せず
/// 幾何級数的収束（比一定）に退化するため、ソルバ収束性の前提として検証する。
/// trial は committed 状態から評価される（path 非依存）ため、摂動ごとに
/// 要素を作り直して評価する。
#[test]
fn test_fiber_tangent_consistent_with_internal_force() {
    let model = build_test_model(Some(78846.15));
    let ctx = Ctx { model: &model };
    let state = ElemState::default();
    let h = 1e-6;
    // 弾性域の代表的な変形状態（並進 [mm]・回転 [rad] 混在）
    let u0: [f64; 12] = [
        0.1, 0.2, -0.1, 0.0005, 0.001, -0.0005, -0.05, 0.15, 0.1, -0.0005, 0.0008, 0.0002,
    ];

    let mut b0 = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);
    b0.update_state(
        &LocalVec {
            data: SmallVec::from_slice(&u0),
        },
        false,
        &ctx,
    );
    let f0 = b0.internal_force(&state, &ctx);
    let k = b0.tangent_stiffness(&state, &ctx);
    let kmax = (0..12)
        .flat_map(|i| (0..12).map(move |j| (i, j)))
        .map(|(i, j)| k.get(i, j).abs())
        .fold(0.0_f64, f64::max);

    for j in 0..12 {
        let mut up = u0;
        up[j] += h;
        let mut bp = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);
        bp.update_state(
            &LocalVec {
                data: SmallVec::from_slice(&up),
            },
            false,
            &ctx,
        );
        let fp = bp.internal_force(&state, &ctx);
        for i in 0..12 {
            let fd = (fp.data[i] - f0.data[i]) / h;
            let err = (fd - k.get(i, j)).abs() / kmax;
            assert!(
                err < 1e-6,
                "K(i={i}, j={j}) が ∂f/∂u と不一致: K={}, FD={}, 相対誤差={err:.3e}",
                k.get(i, j),
                fd
            );
        }
    }
}

#[test]
fn test_fiber_beam_checkpoint_roundtrip() {
    let mut fiber = make_test_fiber_beam(Some(0.0));
    let ctx = Ctx {
        model: &build_test_model(Some(0.0)),
    };
    let du = LocalVec {
        data: SmallVec::from_slice(&[
            0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0005, 0.0,
        ]),
    };
    fiber.update_state(&du, true, &ctx);

    let snap_before = fiber.snapshot_state();
    let checkpoint = fiber.serialize_checkpoint();

    let mut restored = make_test_fiber_beam(Some(0.0));
    restored.deserialize_checkpoint(&checkpoint).unwrap();
    let snap_after = restored.snapshot_state();

    let before = snap_before
        .downcast_ref::<([f64; 12], [f64; 12], Vec<Vec<Box<dyn UniaxialMaterial>>>)>()
        .unwrap();
    let after = snap_after
        .downcast_ref::<([f64; 12], [f64; 12], Vec<Vec<Box<dyn UniaxialMaterial>>>)>()
        .unwrap();
    for i in 0..12 {
        assert_relative_eq!(before.0[i], after.0[i], epsilon = 1e-12);
        assert_relative_eq!(before.1[i], after.1[i], epsilon = 1e-12);
    }
}
/// plastic_zone 付きのテストモデルから塑性化域考慮 FiberBeam を生成する。
fn make_plastic_zone_fiber(lp: f64, fy: Option<f64>) -> FiberBeam {
    let mut model = build_test_model(Some(0.0));
    model.elements[0].plastic_zone = Some(lp);
    model.materials[0].fy = fy;
    FiberBeam::with_plastic_zone(&model.elements[0], &model, lp, StrengthBasis::Nominal)
}

#[test]
fn test_plastic_zone_axial_stiffness_exact() {
    // 軸剛性は端部ファイバ(2Lp) + 中央弾性(L-2Lp) の合成で EA/L に厳密一致する
    let fb = make_plastic_zone_fiber(300.0, None);
    let ctx = Ctx {
        model: &build_test_model(Some(0.0)),
    };
    let k = fb.tangent_stiffness(&ElemState::default(), &ctx);
    let ea_over_l = 205000.0 * 20000.0 / 3000.0;
    assert_relative_eq!(k.get(0, 0), ea_over_l, max_relative = 1e-9);
}

#[test]
fn test_plastic_zone_elastic_stiffness_close_to_full_fiber() {
    // Lp が小さければ弾性剛性は全長ファイバー積分（=弾性梁）に漸近する。
    // 端部の1点矩形則による誤差は O(Lp/L)（曲率分布の勾配×区間幅）で、
    // Lp = L/20 なら数%以内に収まる。
    let model = build_test_model(Some(0.0));
    let ctx = Ctx { model: &model };
    let full = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);
    let k_full = full.tangent_stiffness(&ElemState::default(), &ctx);

    let pz = make_plastic_zone_fiber(150.0, None); // Lp = L/20
    let k_pz = pz.tangent_stiffness(&ElemState::default(), &ctx);
    for (i, j) in [(1usize, 1usize), (2, 2), (4, 4), (5, 5), (1, 5), (2, 4)] {
        assert_relative_eq!(k_pz.get(i, j), k_full.get(i, j), max_relative = 5e-2);
    }
}

/// 塑性化域考慮モデルの中央弾性部 `k_mid` にも断面→要素座標系のクロス変換
/// （elem EIz←sec.iy）が効いていることの回帰テスト。
/// B マトリクスの (uy,rz)=Mz 面と (uz,ry)=My 面の係数は大きさが同一のため、
/// 積分方式に依らず k_mid(1,1)/k_mid(2,2) = EIz_elem/EIy_elem = sec.iy/sec.iz
/// （強軸/弱軸）が厳密に成り立つ。全長ファイバー積分との相対比較
/// （上のテスト）と異なり、断面値から独立に期待比を定めるため、
/// グリッド回転とクロス変換が同時に欠落しても検出できる。
#[test]
fn test_plastic_zone_k_mid_strong_axis_in_mz_plane() {
    let model = build_test_model(Some(0.0));
    let pz = make_plastic_zone_fiber(300.0, None);
    let k_mid = pz.k_mid.as_ref().expect("plastic zone model has k_mid");
    let sec = &model.sections[0];
    let ratio = k_mid.get(1, 1) / k_mid.get(2, 2);
    let expected = sec.iy / sec.iz; // 強軸（Mz 面）/ 弱軸（My 面）
    assert!(
        (ratio - expected).abs() / expected < 1e-12,
        "k_mid(1,1)/k_mid(2,2)={} expected sec.iy/sec.iz={}",
        ratio,
        expected
    );
    // 鉛直曲げ（Mz 面）の方が剛であること（せい 200 > 幅 100 の断面）
    assert!(k_mid.get(1, 1) > k_mid.get(2, 2));
}

#[test]
fn test_plastic_zone_yield_reduces_stiffness() {
    // 端部断面が降伏すると接線剛性が低下する（中央は弾性のまま）
    let mut fb = make_plastic_zone_fiber(300.0, Some(235.0));
    let model = build_test_model(Some(0.0));
    let ctx = Ctx { model: &model };
    let k0 = fb.tangent_stiffness(&ElemState::default(), &ctx);

    // i端に大回転 → 端部断面降伏
    let du = LocalVec {
        data: SmallVec::from_slice(&[0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    };
    fb.update_state(&du, false, &ctx);
    let k1 = fb.tangent_stiffness(&ElemState::default(), &ctx);
    assert!(
        k1.get(4, 4) < 0.9 * k0.get(4, 4),
        "降伏後の回転剛性は低下するはず: k0={}, k1={}",
        k0.get(4, 4),
        k1.get(4, 4)
    );
    // 中央弾性部があるため完全にゼロにはならない
    assert!(k1.get(4, 4) > 0.0);
}

#[test]
fn test_plastic_zone_checkpoint_roundtrip() {
    let mut fb = make_plastic_zone_fiber(300.0, Some(235.0));
    let model = build_test_model(Some(0.0));
    let ctx = Ctx { model: &model };
    let du = LocalVec {
        data: SmallVec::from_slice(&[0.0, 0.0, 0.0, 0.0, 0.02, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    };
    fb.update_state(&du, true, &ctx);
    let cp = fb.serialize_checkpoint();

    let mut fb2 = make_plastic_zone_fiber(300.0, Some(235.0));
    fb2.deserialize_checkpoint(&cp).unwrap();
    let du2 = LocalVec {
        data: SmallVec::from_slice(&[0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    };
    fb.update_state(&du2, false, &ctx);
    fb2.update_state(&du2, false, &ctx);
    let f1 = fb.internal_force(&ElemState::default(), &ctx);
    let f2 = fb2.internal_force(&ElemState::default(), &ctx);
    for i in 0..12 {
        assert_relative_eq!(f1.data[i], f2.data[i], epsilon = 1e-6);
    }
}

/// RC 断面（RcRect＋配筋）のファイバー柱は、コンクリート格子に加えて主筋が
/// 点ファイバーとして分離配置される（構造力学のファイバーモデルにおける鉄筋分離）。
/// 従来は均質コンクリート断面で引張鉄筋を無視していた。
#[test]
fn test_rc_fiber_section_includes_separated_rebar() {
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    let shape = SectionShape::RcRect {
        b: 500.0,
        d: 500.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 4,
                dia: 25.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 4,
                dia: 25.0,
                layers: 1,
            },
            cover: 50.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        },
    };
    let sec = shape.to_section(SectionId(0), "C500".into());
    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Fiber,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![sec],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "FC30".into(),
            young: 25000.0,
            poisson: 0.2,
            density: 0.0,
            shear: Some(0.0),
            fc: Some(30.0),
            fy: None,
        }],
        ..Default::default()
    };
    let fb = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);
    let gp = &fb.gauss_points[0];
    // コンクリート格子 12×20=240 に主筋（main_x 4×上下2=8 + main_y 4×側面2=8 = 16 本）が加算。
    assert!(
        gp.section.fibers.len() > 240,
        "主筋ファイバーが分離配置されていない: {}",
        gp.section.fibers.len()
    );
    let rebar_count = gp.section.fibers.iter().filter(|f| f.material == 1).count();
    assert_eq!(rebar_count, 16, "主筋本数（上下8＋側面8）: {rebar_count}");
    // 主筋は最外縁近く（かぶり50・径25 → z0=500/2-50-12.5=187.5）に配置される。
    let max_abs_z = gp
        .section
        .fibers
        .iter()
        .filter(|f| f.material == 1)
        .map(|f| f.z.abs())
        .fold(0.0_f64, f64::max);
    assert!(max_abs_z > 180.0, "主筋が最外縁近くにない: {max_abs_z}");
}

/// 降伏後の部材内力が「接線剛性 × 全変位」ではなく**ファイバー状態**から
/// 取り出されること（`state_member_forces`）。
///
/// 降伏させた要素で、(a) 端部の断面内力が復元力（`internal_force`）と一致する
/// （釣合いによる分配）、(b) 接線剛性 × 全変位で組んだ内力とは明確に異なる、
/// ことを確認する。(a) が成り立たないと、非線形解析の応力が降伏後に誤る。
#[test]
fn test_state_member_forces_uses_fiber_state_not_tangent() {
    let ctx = Ctx {
        model: &Model::default(),
    };
    // 端部 rz（強軸曲げ面）に大きな逆対称回転を与えて降伏させる。
    let big = 0.2;
    let du = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, big, 0.0, 0.0, 0.0, 0.0, 0.0, -big],
    };
    let mut elem = make_steel_fiber_with_fy(Some(235.0));
    elem.update_state(&du, true, &ctx);
    let state = ElemState::default();

    let mf = elem
        .state_member_forces(&state, &ctx)
        .expect("ファイバー梁は状態から内力を返す");
    // 評価断面は弾性梁と同じ規則（剛域なし → 節点芯・中央）。
    assert!(mf.at.iter().any(|(xi, _)| xi.abs() < 1e-12));
    assert!(mf.at.iter().any(|(xi, _)| (xi - 0.5).abs() < 1e-12));
    assert!(mf.at.iter().any(|(xi, _)| (xi - 1.0).abs() < 1e-12));

    // i 端（xi=0）の Mz は復元力の f[5] と符号反転で一致する（断面内力の規約）。
    let f = elem.internal_force(&state, &ctx);
    let mz_i = mf
        .at
        .iter()
        .find(|(xi, _)| xi.abs() < 1e-12)
        .map(|(_, v)| v[5])
        .unwrap();
    assert_relative_eq!(mz_i, -f.data[5], epsilon = 1e-6);
    assert!(f.data[5].abs() > 1.0, "前提: 曲げが有意であること");

    // 接線剛性 × 全変位で組んだ内力は降伏後に過小評価となり、状態由来の値と一致しない。
    let k = elem.tangent_stiffness(&state, &ctx);
    let mut f_tangent = 0.0;
    for j in 0..12 {
        f_tangent += k.get(5, j) * elem.axis.rotate_to_global(&elem.trial_disp)[j];
    }
    assert!(
        (f_tangent - f.data[5]).abs() > f.data[5].abs() * 0.1,
        "降伏後に接線剛性×全変位と状態由来の内力が一致してしまっている: {} vs {}",
        f_tangent,
        f.data[5]
    );
}

/// `state_member_forces` の内力場が連続・整合であること
/// （`BeamElement::recover_forces` と同じ規約: N/Qy/Qz/Mx は一定、
/// Mz/My は dMz/dx = Qy・dMy/dx = −Qz の線形場）。
///
/// 端部内力を釣合いでスパン内へ分配しているため、降伏後もこの関係が成り立つ。
#[test]
fn test_state_member_forces_field_is_continuous() {
    let ctx = Ctx {
        model: &Model::default(),
    };
    // 曲げ・軸・弱軸曲げが同時に生じる一般的な変位を与えて降伏させる。
    let du = LocalVec {
        data: smallvec::smallvec![
            0.5, 0.0, 0.0, 0.0, 0.05, 0.2, //
            -0.5, 0.0, 0.0, 0.0, -0.03, -0.1
        ],
    };
    let mut elem = make_steel_fiber_with_fy(Some(235.0));
    elem.eval_sections = vec![0.0, 0.25, 0.5, 0.75, 1.0];
    elem.update_state(&du, true, &ctx);

    let mf = elem
        .state_member_forces(&ElemState::default(), &ctx)
        .unwrap();
    let l = elem.length;
    let at = |xi: f64| -> [f64; 6] {
        mf.at
            .iter()
            .find(|(p, _)| (p - xi).abs() < 1e-12)
            .map(|(_, v)| *v)
            .unwrap()
    };

    let a = at(0.0);
    assert!(a[5].abs() > 1.0, "前提: 強軸曲げが有意であること");
    for &xi in &[0.25, 0.5, 0.75, 1.0] {
        let v = at(xi);
        // N・Qy・Qz・Mx は部材内で一定
        assert_relative_eq!(v[0], a[0], max_relative = 1e-9, epsilon = 1e-6);
        assert_relative_eq!(v[1], a[1], max_relative = 1e-9, epsilon = 1e-6);
        assert_relative_eq!(v[2], a[2], max_relative = 1e-9, epsilon = 1e-6);
        assert_relative_eq!(v[3], a[3], max_relative = 1e-9, epsilon = 1e-6);
        // dMz/dx = Qy, dMy/dx = -Qz（スパン内荷重なし）
        assert_relative_eq!(
            v[5],
            a[5] + a[1] * xi * l,
            max_relative = 1e-9,
            epsilon = 1e-6
        );
        assert_relative_eq!(
            v[4],
            a[4] - a[2] * xi * l,
            max_relative = 1e-9,
            epsilon = 1e-6
        );
    }
}
