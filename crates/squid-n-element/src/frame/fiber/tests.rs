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

    let before = snap_before.downcast_ref::<FiberBeamSnapshot>().unwrap();
    let after = snap_after.downcast_ref::<FiberBeamSnapshot>().unwrap();
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

/// 塑性増分ヒンジモデルの弾性剛性 `k_el` にも断面→要素座標系のクロス変換
/// （elem EIz←sec.iy）が効いていることの回帰テスト。
/// B マトリクスの (uy,rz)=Mz 面と (uz,ry)=My 面の係数は大きさが同一のため、
/// せん断剛性なし（G=0、φ=0）のモデルでは
/// k_el(1,1)/k_el(2,2) = EIz_elem/EIy_elem = sec.iy/sec.iz（強軸/弱軸）が
/// 厳密に成り立つ。断面値から独立に期待比を定めるため、
/// グリッド回転とクロス変換が同時に欠落しても検出できる。
#[test]
fn test_plastic_zone_k_el_strong_axis_in_mz_plane() {
    let model = build_test_model(Some(0.0));
    let pz = make_plastic_zone_fiber(300.0, None);
    let k_el = &pz
        .hinge
        .as_ref()
        .expect("plastic zone model has hinge")
        .k_el;
    let sec = &model.sections[0];
    let ratio = k_el.get(1, 1) / k_el.get(2, 2);
    let expected = sec.iy / sec.iz; // 強軸（Mz 面）/ 弱軸（My 面）
    assert!(
        (ratio - expected).abs() / expected < 1e-12,
        "k_el(1,1)/k_el(2,2)={} expected sec.iy/sec.iz={}",
        ratio,
        expected
    );
    // 鉛直曲げ（Mz 面）の方が剛であること（せい 200 > 幅 100 の断面）
    assert!(k_el.get(1, 1) > k_el.get(2, 2));
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
            main_grade: None,
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

// ===== 剛域（材端剛体アーム）=====

/// 剛域長 λi・λj を与えたテストモデル（節点間長 3000mm、500 角・せん断断面付き）。
fn build_rigid_zone_model(li: f64, lj: f64) -> Model {
    let mut model = build_test_model(Some(78846.15));
    set_square500_shear_section(&mut model);
    model.elements[0].rigid_zone = squid_n_core::model::RigidZone {
        length_i: li,
        length_j: lj,
        face_i: li,
        face_j: lj,
        ..Default::default()
    };
    model
}

/// 受け入れテスト: 剛域を与えた弾性状態の 12×12 接線剛性が、同じ剛域を与えた
/// 弾性 Timoshenko 梁 `BeamElement` と、**軸自由度を除いて厳密一致**すること。
///
/// 曲げ・せん断は可撓長で組んでから剛体アームで節点自由度へ写す扱いが両者で
/// 共通なので厳密に一致する。ねじりも節点間長基準 GJ/L で一致する。
/// 軸のみ、弾性梁が A·(L'/L) 補正で EA/L（節点間長基準）とするのに対し、
/// ファイバー要素は断面積分が軸力-曲げを連成させるため補正できず EA/L'
/// （剛域を軸方向にも剛とする扱い）になる。その比 L/L' も明示的に検証する。
#[test]
fn 剛域ありの弾性剛性は軸以外が弾性梁と厳密一致する() {
    let (li, lj) = (400.0, 250.0);
    let (l, l_flex) = (3000.0, 3000.0 - 400.0 - 250.0);
    let g = 78846.15;
    let (b_w, d_h): (f64, f64) = (500.0, 500.0);
    let (nw, nd) = (12.0, 20.0);
    let area = b_w * d_h;
    // ファイバー格子の離散 EI（要素座標系。図心集中による目減りを反映）
    let iz_elem = b_w * d_h.powi(3) / 12.0 * (1.0 - 1.0 / (nd * nd)); // 強軸 (uy,rz)
    let iy_elem = d_h * b_w.powi(3) / 12.0 * (1.0 - 1.0 / (nw * nw)); // 弱軸 (uz,ry)
    let as_y_elem = 208333.0;
    let as_z_elem = 150000.0;
    let j = 1.0e6;

    let mut model = build_rigid_zone_model(li, lj);
    model.sections[0].depth = d_h;
    model.sections[0].width = b_w;
    model.sections[0].area = area;
    model.sections[0].iy = iz_elem;
    model.sections[0].iz = iy_elem;
    model.sections[0].as_z = as_y_elem;
    model.sections[0].as_y = as_z_elem;
    model.sections[0].j = j;

    let ctx = Ctx { model: &model };
    let zero = LocalVec {
        data: SmallVec::from_elem(0.0, 12),
    };
    let mut fiber = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);
    assert_relative_eq!(fiber.flex_length, l_flex, max_relative = 1e-12);
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
    be.rigid = model.elements[0].rigid_zone;
    let k_be = be.tangent_stiffness(&ElemState::default(), &ctx);

    let kmax = (0..12)
        .flat_map(|i| (0..12).map(move |j| (i, j)))
        .map(|(i, j)| k_be.get(i, j).abs())
        .fold(0.0_f64, f64::max);
    // 軸自由度（0, 6）以外は厳密一致
    for i in 0..12 {
        for j in 0..12 {
            if [0, 6].contains(&i) || [0, 6].contains(&j) {
                continue;
            }
            let diff = (k_fb.get(i, j) - k_be.get(i, j)).abs();
            assert!(
                diff <= 1e-9 * kmax,
                "K({i},{j}) が剛域つき Timoshenko 梁と不一致: fiber={}, beam={}, 差={diff:.3e}",
                k_fb.get(i, j),
                k_be.get(i, j)
            );
        }
    }
    // 軸剛性: 弾性梁は EA/L、ファイバーは EA/L'
    let ea = 205000.0 * area;
    assert_relative_eq!(k_be.get(0, 0), ea / l, max_relative = 1e-9);
    assert_relative_eq!(k_fb.get(0, 0), ea / l_flex, max_relative = 1e-9);
    // ねじりは両者とも節点間長基準
    assert_relative_eq!(k_fb.get(3, 3), g * j / l, max_relative = 1e-9);
    assert_relative_eq!(k_be.get(3, 3), g * j / l, max_relative = 1e-9);
}

/// 剛域は曲げ剛性を増大させる（可撓長が短くなり、剛体アームが加わるため）。
/// 片持ち（i 端固定）の先端並進剛性で比較する。
#[test]
fn 剛域は片持ち先端の曲げ剛性を増大させる() {
    let ctx_model_none = {
        let mut m = build_test_model(Some(78846.15));
        set_square500_shear_section(&mut m);
        m
    };
    let model_rz = build_rigid_zone_model(400.0, 250.0);

    let tip_stiffness = |model: &Model| -> f64 {
        let ctx = Ctx { model };
        let mut fb = FiberBeam::new(&model.elements[0], model, StrengthBasis::Nominal);
        fb.update_state(
            &LocalVec {
                data: SmallVec::from_elem(0.0, 12),
            },
            false,
            &ctx,
        );
        let k = fb.tangent_stiffness(&ElemState::default(), &ctx);
        // j 端 [uy, rz] の 2×2 を縮約した先端モーメントフリー剛性
        let (a, b, c) = (k.get(7, 7), k.get(7, 11), k.get(11, 11));
        (a * c - b * b) / c
    };

    let k_none = tip_stiffness(&ctx_model_none);
    let k_rz = tip_stiffness(&model_rz);
    assert!(
        k_rz > k_none * 1.2,
        "剛域で曲げ剛性が十分に増大していない: 剛域なし={k_none:.3e}, 剛域あり={k_rz:.3e}"
    );
}

/// 剛域があっても剛体回転だけでは内力が発生しないこと（客観性）。
/// 剛体アームの運動学（`rigid_arm`）の符号を誤ると、可撓端に見かけの相対
/// たわみが生じて偽の内力が出る。
#[test]
fn 剛域ありでも剛体回転で内力が生じない() {
    let model = build_rigid_zone_model(400.0, 250.0);
    let ctx = Ctx { model: &model };
    let mut fiber = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);

    // 節点 i まわりの θz 剛体回転（節点自由度で与える）
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
    for (i, v) in f.data.iter().enumerate() {
        assert!(
            v.abs() < 1.0,
            "剛域つきの剛体回転で内力が発生した（客観性違反）: dof {i} = {v}"
        );
    }
}

/// 剛域があっても接線剛性が内力の厳密な勾配（∂f/∂u）であること。
/// 剛体アーム変換が剛性側（Trᵀ K Tr）と内力側（Trᵀ f）で整合していないと崩れる。
#[test]
fn 剛域ありでも接線剛性が内力の勾配と一致する() {
    let model = build_rigid_zone_model(400.0, 250.0);
    let ctx = Ctx { model: &model };
    let state = ElemState::default();
    let h = 1e-6;
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

/// 塑性化域考慮モデルでは、端部積分点（ξ=∓1）が剛域フェイスに置かれ、
/// その積分重み（＝塑性化域長 Lp）と中央弾性部が可撓長基準になること。
#[test]
fn 剛域ありの塑性化域は可撓長基準になる() {
    let (li, lj) = (400.0, 250.0);
    let l_flex = 3000.0 - li - lj;
    let lp = 300.0;
    let mut model = build_rigid_zone_model(li, lj);
    model.elements[0].plastic_zone = Some(lp);
    let fb = FiberBeam::with_plastic_zone(&model.elements[0], &model, lp, StrengthBasis::Nominal);

    assert_relative_eq!(fb.flex_length, l_flex, max_relative = 1e-12);
    assert_eq!(fb.gauss_points.len(), 2);
    for gp in &fb.gauss_points {
        assert_relative_eq!(gp.xi.abs(), 1.0, max_relative = 1e-12);
        // 重み w·(L'/2) = Lp → w = 2Lp/L'
        assert_relative_eq!(gp.weight, 2.0 * lp / l_flex, max_relative = 1e-12);
    }
    assert!(fb.hinge.is_some(), "塑性増分ヒンジが構築されていない");
}

/// 剛域長の合計が節点間長以上になる病的な入力は、剛域なしとして扱う
/// （可撓長ゼロで要素が退化するのを防ぐ）。
#[test]
fn 可撓長が残らない剛域は無視される() {
    let model = build_rigid_zone_model(2000.0, 1500.0); // 合計 3500 > L=3000
    let fb = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);
    assert_eq!(fb.rigid_i, 0.0);
    assert_eq!(fb.rigid_j, 0.0);
    assert_relative_eq!(fb.flex_length, fb.length, max_relative = 1e-12);
}

// ===== 材端解放（ピン・半剛）=====

/// 指定した端条件のテストモデル（節点間長 3000mm、500 角・せん断断面付き）。
fn build_release_model(end_cond: [EndCondition; 2]) -> Model {
    let mut model = build_test_model(Some(78846.15));
    set_square500_shear_section(&mut model);
    model.elements[0].end_cond = end_cond;
    model
}

/// 弾性状態で `FiberBeam` を組み、初期接線をキャッシュしたうえで返す。
fn elastic_fiber(model: &Model) -> FiberBeam {
    let ctx = Ctx { model };
    let mut fb = FiberBeam::new(&model.elements[0], model, StrengthBasis::Nominal);
    fb.update_state(
        &LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        },
        false,
        &ctx,
    );
    fb
}

/// 受け入れテスト: 材端ピンの弾性剛性が、同じ端条件の弾性 Timoshenko 梁
/// `BeamElement` と（軸自由度を除いて）厳密一致すること。
/// 材端解放の静縮約が弾性梁と同じ定式化で入っていることを担保する。
#[test]
fn 材端ピンの弾性剛性が弾性梁と一致する() {
    let (b_w, d_h): (f64, f64) = (500.0, 500.0);
    let (nw, nd) = (12.0, 20.0);
    let area = b_w * d_h;
    let iz_elem = b_w * d_h.powi(3) / 12.0 * (1.0 - 1.0 / (nd * nd));
    let iy_elem = d_h * b_w.powi(3) / 12.0 * (1.0 - 1.0 / (nw * nw));
    let as_y_elem = 208333.0;
    let as_z_elem = 150000.0;
    let j = 1.0e6;

    // i 端ピン・j 端剛接
    let mut model = build_release_model([EndCondition::Pinned, EndCondition::Fixed]);
    model.sections[0].depth = d_h;
    model.sections[0].width = b_w;
    model.sections[0].area = area;
    model.sections[0].iy = iz_elem;
    model.sections[0].iz = iy_elem;
    model.sections[0].as_z = as_y_elem;
    model.sections[0].as_y = as_z_elem;
    model.sections[0].j = j;

    let ctx = Ctx { model: &model };
    let fiber = elastic_fiber(&model);
    let k_fb = fiber.tangent_stiffness(&ElemState::default(), &ctx);

    let mut be = make_test_beam_element(as_y_elem);
    be.a = area;
    be.a_mass = area;
    be.iy = iy_elem;
    be.iz = iz_elem;
    be.j = j;
    be.as_y = as_y_elem;
    be.as_z = as_z_elem;
    be.end_cond = model.elements[0].end_cond;
    let k_be = be.tangent_stiffness(&ElemState::default(), &ctx);

    let kmax = (0..12)
        .flat_map(|i| (0..12).map(move |j| (i, j)))
        .map(|(i, j)| k_be.get(i, j).abs())
        .fold(0.0_f64, f64::max);
    for i in 0..12 {
        for j in 0..12 {
            if [0, 6].contains(&i) || [0, 6].contains(&j) {
                continue; // 軸は 4.9.5 のとおり弾性梁と定義が異なる（剛域なしなら一致）
            }
            let diff = (k_fb.get(i, j) - k_be.get(i, j)).abs();
            assert!(
                diff <= 1e-9 * kmax,
                "K({i},{j}) が材端ピンの弾性梁と不一致: fiber={}, beam={}, 差={diff:.3e}",
                k_fb.get(i, j),
                k_be.get(i, j)
            );
        }
        // ピン端（i 端）の回転自由度は剛性を持たない
        for r in [3usize, 4, 5] {
            assert!(
                k_fb.get(r, i).abs() < 1e-6 * kmax.max(1.0),
                "ピン端の回転自由度 {r} に剛性が残っている: K({r},{i})={}",
                k_fb.get(r, i)
            );
        }
    }
    // 剛接端（j 端）は曲げ剛性を持つ
    assert!(k_fb.get(11, 11) > 0.0);
}

/// ピン端では、その端に曲げモーメント内力が生じないこと（厳密なモーメント解放）。
/// 剛接端との比較で、解放が実際に効いていることを確認する。
#[test]
fn 材端ピンでは当該端の曲げモーメントがゼロになる() {
    let pinned = build_release_model([EndCondition::Pinned, EndCondition::Fixed]);
    let fixed = build_release_model([EndCondition::Fixed, EndCondition::Fixed]);
    // j 端に並進 uy を与える（片持ち的な変形）。
    let du = |uy: f64| LocalVec {
        data: SmallVec::from_slice(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, uy, 0.0, 0.0, 0.0, 0.0]),
    };

    let ctx_p = Ctx { model: &pinned };
    let mut fb_p = FiberBeam::new(&pinned.elements[0], &pinned, StrengthBasis::Nominal);
    fb_p.update_state(&du(1.0), false, &ctx_p);
    let f_p = fb_p.internal_force(&ElemState::default(), &ctx_p);

    let ctx_f = Ctx { model: &fixed };
    let mut fb_f = FiberBeam::new(&fixed.elements[0], &fixed, StrengthBasis::Nominal);
    fb_f.update_state(&du(1.0), false, &ctx_f);
    let f_f = fb_f.internal_force(&ElemState::default(), &ctx_f);

    // 剛接端は i 端に大きなモーメントを持つ
    assert!(
        f_f.data[5].abs() > 1.0e6,
        "剛接端の Mz が小さすぎる: {}",
        f_f.data[5]
    );
    // ピン端は Mz ≈ 0（剛接端の値に対して 1e-9 未満）
    assert!(
        f_p.data[5].abs() < 1e-9 * f_f.data[5].abs(),
        "ピン端に曲げモーメントが残っている: {}",
        f_p.data[5]
    );
    // ピンにより横剛性は下がる（片持ち: 3EI/L³ vs 12EI/L³ のオーダー）
    assert!(
        f_p.data[7].abs() < f_f.data[7].abs(),
        "ピン解放で横剛性が下がっていない"
    );
}

/// 半剛（回転ばね）は、剛接とピンの中間の剛性になること。
/// ばね剛性 →∞ で剛接、→0 でピンに漸近する。
#[test]
fn 半剛端は剛接とピンの中間になる() {
    // j 端（剛接）の回転剛性 K(rz_j, rz_j) を指標にする。i 端の条件により
    // 剛接なら 4EI/L'、ピンなら 3EI/L'（いずれもせん断補正を含む）へ変わる。
    let rot_stiffness = |end_cond: [EndCondition; 2]| -> f64 {
        let model = build_release_model(end_cond);
        let ctx = Ctx { model: &model };
        let fb = elastic_fiber(&model);
        fb.tangent_stiffness(&ElemState::default(), &ctx)
            .get(11, 11)
    };
    let k_fixed = rot_stiffness([EndCondition::Fixed, EndCondition::Fixed]);
    let k_pin = rot_stiffness([EndCondition::Pinned, EndCondition::Fixed]);
    // 6EI/L' 程度の中間的なばね剛性
    let k_theta = 6.0 * 205000.0 * 5.2083333e9 / 3000.0;
    let k_semi = rot_stiffness([EndCondition::SemiRigid { k_theta }, EndCondition::Fixed]);

    assert!(
        k_pin < k_semi && k_semi < k_fixed,
        "半剛が剛接とピンの中間になっていない: pin={k_pin:.4e}, semi={k_semi:.4e}, fixed={k_fixed:.4e}"
    );
    // i 端ピンは剛接の 3/4 倍（3EI/L' vs 4EI/L'）へ近い値になる
    assert!(
        (k_pin / k_fixed - 0.75).abs() < 0.05,
        "ピン端の回転剛性比が 3/4 から外れている: {:.4}",
        k_pin / k_fixed
    );
    // ばね剛性を十分大きく／小さくすると剛接／ピンへ漸近する
    let k_stiff = rot_stiffness([
        EndCondition::SemiRigid {
            k_theta: k_theta * 1.0e8,
        },
        EndCondition::Fixed,
    ]);
    assert_relative_eq!(k_stiff, k_fixed, max_relative = 1e-6);
    let k_soft = rot_stiffness([
        EndCondition::SemiRigid {
            k_theta: k_theta * 1.0e-8,
        },
        EndCondition::Fixed,
    ]);
    assert_relative_eq!(k_soft, k_pin, max_relative = 1e-6);
}

/// 材端解放があっても接線剛性が内力の厳密な勾配（∂f/∂u）であること。
/// 内部自由度の静縮約（剛性側）と内部釣合いの解（内力側）が整合していないと崩れる。
#[test]
fn 材端解放ありでも接線剛性が内力の勾配と一致する() {
    for end_cond in [
        [EndCondition::Pinned, EndCondition::Fixed],
        [EndCondition::Fixed, EndCondition::Pinned],
        [
            EndCondition::SemiRigid { k_theta: 2.0e12 },
            EndCondition::Pinned,
        ],
    ] {
        let mut model = build_release_model(end_cond);
        // 剛域も併用して、剛体アーム変換との整合も同時に検証する。
        model.elements[0].rigid_zone = squid_n_core::model::RigidZone {
            length_i: 400.0,
            length_j: 250.0,
            face_i: 400.0,
            face_j: 250.0,
            ..Default::default()
        };
        let ctx = Ctx { model: &model };
        let state = ElemState::default();
        let h = 1e-6;
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
                    "{end_cond:?}: K(i={i}, j={j}) が ∂f/∂u と不一致: K={}, FD={}, 相対誤差={err:.3e}",
                    k.get(i, j),
                    fd
                );
            }
        }
    }
}

/// 材端解放があっても剛体回転で内力が生じないこと（客観性）。
#[test]
fn 材端解放ありでも剛体回転で内力が生じない() {
    let model = build_release_model([EndCondition::Pinned, EndCondition::Fixed]);
    let ctx = Ctx { model: &model };
    let mut fiber = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);
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
    for (i, v) in f.data.iter().enumerate() {
        assert!(
            v.abs() < 1.0,
            "材端解放つきの剛体回転で内力が発生した（客観性違反）: dof {i} = {v}"
        );
    }
}

/// 降伏後（非線形域）でもピン端のモーメント解放が保たれること。
/// 内部自由度の内部釣合いを Newton で解いているため、材料が降伏しても
/// 「ピン端の要素モーメント = 0」が維持される。
#[test]
fn 降伏後もピン端のモーメント解放が保たれる() {
    let mut model = build_release_model([EndCondition::Pinned, EndCondition::Fixed]);
    model.materials[0].fy = Some(235.0);
    let ctx = Ctx { model: &model };
    let mut fb = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);

    // 段階的に大変形を与えて降伏させる。
    for _ in 0..40 {
        let du = LocalVec {
            data: SmallVec::from_slice(&[
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0,
            ]),
        };
        fb.update_state(&du, true, &ctx);
    }
    let f = fb.internal_force(&ElemState::default(), &ctx);
    // 剛接端（j 端）のモーメントを基準に、ピン端（i 端）は無視できる大きさ
    let m_fixed = f.data[11].abs().max(1.0);
    assert!(
        f.data[5].abs() < 1e-8 * m_fixed,
        "降伏後にピン端へモーメントが残った: Mz_i={}, Mz_j={}",
        f.data[5],
        f.data[11]
    );
    // 実際に降伏していること（弾性なら接線剛性が初期値のまま）
    let k = fb.tangent_stiffness(&ElemState::default(), &ctx);
    let k0 = elastic_fiber(&build_release_model([
        EndCondition::Pinned,
        EndCondition::Fixed,
    ]))
    .tangent_stiffness(&ElemState::default(), &ctx);
    assert!(
        k.get(7, 7) < 0.95 * k0.get(7, 7),
        "降伏していない（接線剛性が低下していない）: {} vs {}",
        k.get(7, 7),
        k0.get(7, 7)
    );
}

/// ねじり剛性を持たない部材（J=0）ではピン端でも rx を解放しない
/// （解放しても縮約行列が特異化するだけで意味がないため）。
#[test]
fn ねじり剛性が無い部材はrxを解放しない() {
    let mut model = build_release_model([EndCondition::Pinned, EndCondition::Pinned]);
    model.sections[0].j = 0.0;
    let fb = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);
    assert!(
        fb.releases.iter().all(|r| r.dof != 3 && r.dof != 9),
        "J=0 で rx が解放された: {:?}",
        fb.releases
    );
    // 曲げ回転（ry, rz）は両端とも解放される
    assert_eq!(fb.releases.len(), 4);

    // J>0 なら rx も解放される
    model.sections[0].j = 1.0e6;
    let fb = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);
    assert_eq!(fb.releases.len(), 6);
}

/// 材端解放の内部自由度がチェックポイント／スナップショットで往復すること。
#[test]
fn 材端解放の内部自由度がチェックポイントで往復する() {
    let model = build_release_model([EndCondition::Pinned, EndCondition::Fixed]);
    let ctx = Ctx { model: &model };
    let mut fb = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);
    fb.update_state(
        &LocalVec {
            data: SmallVec::from_slice(&[
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.001,
            ]),
        },
        true,
        &ctx,
    );
    assert!(
        fb.trial_int.iter().any(|v| v.abs() > 1e-12),
        "内部自由度が動いていない"
    );

    let checkpoint = fb.serialize_checkpoint();
    let mut restored = FiberBeam::new(&model.elements[0], &model, StrengthBasis::Nominal);
    restored.deserialize_checkpoint(&checkpoint).unwrap();
    for (a, b) in fb.trial_int.iter().zip(restored.trial_int.iter()) {
        assert_relative_eq!(a, b, epsilon = 1e-12);
    }
    for (a, b) in fb.committed_int.iter().zip(restored.committed_int.iter()) {
        assert_relative_eq!(a, b, epsilon = 1e-12);
    }
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

/// 角形鋼管（SteelBox、中空断面）のファイバ配置が管壁のみで、断面積・断面二次
/// モーメントが理論値と一致することを検証する回帰テスト。
///
/// 従来は形状によらず width×depth の中実矩形格子でファイバを生成しており、
/// □-400×400×12 では断面積を約 8.6 倍（160000/18624 mm²）に過大評価し、
/// 剛性・全塑性耐力も同様に過大だった（保有水平耐力の過大評価＝危険側）。
#[test]
fn test_steel_box_fibers_are_hollow() {
    let shape = squid_n_core::section_shape::SectionShape::SteelBox {
        height: 400.0,
        width: 400.0,
        thick: 12.0,
        corner_r: 0.0,
    };
    let (sec, mats) = build_gauss_fibers(
        400.0,
        400.0,
        12,
        20,
        Some(&shape),
        None,
        205000.0,
        Some(295.0),
        1.0,
        1.0,
    );
    assert_eq!(sec.fibers.len(), mats.len());

    // 断面積: A = 400² − 376²（角部直角）。板分割は端数なく厳密一致する。
    let a_sum: f64 = sec.fibers.iter().map(|f| f.area).sum();
    let a_exact = 400.0_f64 * 400.0 - 376.0 * 376.0;
    assert_relative_eq!(a_sum, a_exact, max_relative = 1e-9);

    // 断面二次モーメント（回転後座標: せい方向=y）: I = (400⁴ − 376⁴)/12。
    // ファイバ離散化（板厚 2 分割・板長 16 分割程度）の打切り誤差 2% 以内。
    let i_sum: f64 = sec.fibers.iter().map(|f| f.area * f.y * f.y).sum();
    let i_exact = (400.0_f64.powi(4) - 376.0_f64.powi(4)) / 12.0;
    assert_relative_eq!(i_sum, i_exact, max_relative = 0.02);

    // 材料区分はすべて鋼材（2）で、管内側（|y|,|z| < 376/2 の中央部）にファイバが無い。
    assert!(sec.fibers.iter().all(|f| f.material == 2));
    assert!(sec
        .fibers
        .iter()
        .all(|f| f.y.abs() > 376.0 / 2.0 - 1e-9 || f.z.abs() > 376.0 / 2.0 - 1e-9));
}

/// RC 円形断面のファイバ配置が円形（極座標リング）で、コンクリート断面積が
/// π·d²/4 と一致し、主筋が材料区分 1 で分離配置されることを検証する。
/// 従来は d×d の中実矩形格子で断面積を 4/π ≒ 1.27 倍に過大評価していた。
#[test]
fn test_rc_circle_fibers_match_circle_area() {
    let rebar = squid_n_core::section_shape::RcRebar {
        main_grade: None,
        main_x: squid_n_core::section_shape::BarSet {
            count: 4,
            dia: 22.0,
            layers: 1,
        },
        main_y: squid_n_core::section_shape::BarSet {
            count: 4,
            dia: 22.0,
            layers: 1,
        },
        cover: 40.0,
        shear: squid_n_core::section_shape::ShearBar {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
            grade: None,
        },
    };
    let shape = squid_n_core::section_shape::SectionShape::RcCircle { d: 600.0, rebar };
    let (sec, _mats) = build_gauss_fibers(
        600.0,
        600.0,
        12,
        20,
        Some(&shape),
        Some(24.0),
        22000.0,
        Some(345.0),
        1.0,
        1.0,
    );

    let conc_area: f64 = sec
        .fibers
        .iter()
        .filter(|f| f.material == 0)
        .map(|f| f.area)
        .sum();
    let circle = std::f64::consts::PI * 600.0_f64 * 600.0 / 4.0;
    assert_relative_eq!(conc_area, circle, max_relative = 1e-9);

    // 主筋 8 本が材料区分 1 の点ファイバとして分離されていること。
    let n_rebar = sec.fibers.iter().filter(|f| f.material == 1).count();
    assert_eq!(n_rebar, 8);
}
