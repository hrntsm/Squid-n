use super::*;
use crate::behavior::{Ctx, ElementBehavior};
use crate::factory::StrengthBasis;
use approx::assert_relative_eq;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    AnalysisKind, ElementData, ElementKind, EndCondition, ForceRegime, HysteresisModel, LocalAxis,
    Material, MaterialCategory, Model, Node, Section,
};

fn make_test_fiber_beam(shear_mod: Option<f64>) -> FiberBeam {
    let model = build_test_model(shear_mod);
    FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    )
}

fn make_test_beam_element(as_val: f64) -> crate::frame::beam::BeamElement {
    crate::frame::beam::BeamElement {
        id: ElemId(0),
        e: 205000.0,
        g: 78846.15,
        a: 20000.0,
        a_mass: 20000.0,
        iy: 16666666.66666667,
        iz: 66666666.66666667,
        j: 0.0,
        as_y: as_val,
        as_z: as_val,
        length: 3000.0,
        density: 0.0,
        mass_properties: squid_n_core::model::SectionMassProperties::default(),
        mass_properties_error: None,
        nodes: [NodeId(0), NodeId(1)],
        axis: crate::transform::LocalFrame {
            rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        },
        rigid: squid_n_core::model::RigidZone::default(),
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        torsion_release: [false, false],
        eval_sections: vec![],
        section: None,
        material: None,
        committed_disp: [0.0; 12],
        trial_disp: [0.0; 12],
        local_stiffness_cache: std::sync::OnceLock::new(),
    }
}

#[test]
fn beamとfiberは同じ断面なら整合質量が一致する() {
    let density = 2.4e-9;
    let as_val = 15000.0;
    let mut model = build_test_model(Some(78846.15));
    model.materials[0].density = density;
    model.sections[0].as_y = as_val;
    model.sections[0].as_z = as_val;
    let fiber = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );

    let mut beam = make_test_beam_element(as_val);
    beam.density = density;
    beam.iy = model.sections[0].iy;
    beam.iz = model.sections[0].iz;
    beam.mass_properties = squid_n_core::model::SectionMassProperties::uniform(
        density,
        model.sections[0].area,
        model.sections[0].iy,
        model.sections[0].iz,
    );
    let mass_beam = beam.mass_matrix(crate::behavior::MassOption::Consistent);
    let mass_fiber = fiber.mass_matrix(crate::behavior::MassOption::Consistent);
    for i in 0..12 {
        for j in 0..12 {
            assert!(
                (mass_beam.get(i, j) - mass_fiber.get(i, j)).abs() < 1e-10,
                "M({i},{j}) が Beam/Fiber で不一致: beam={}, fiber={}",
                mass_beam.get(i, j),
                mass_fiber.get(i, j)
            );
        }
    }
}

#[test]
fn 有効断面性能を使うbeamとfiberのphiと整合質量が一致する() {
    use squid_n_core::section_shape::SectionShape;

    let mut model = build_test_model(Some(78846.15));
    model.sections[0].shape = Some(SectionShape::CftBox {
        height: 260.0,
        width: 180.0,
        thick: 12.0,
    });
    model.materials[0].fc = Some(24.0);
    let beam = crate::frame::beam::BeamElement::new(&model.elements[0], &model);
    let fiber = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );

    let flex_length = fiber.flex_length;
    let expected_phi_y = 12.0 * beam.e * beam.iz / (beam.g * beam.as_y * flex_length.powi(2));
    let expected_phi_z = 12.0 * beam.e * beam.iy / (beam.g * beam.as_z * flex_length.powi(2));
    assert_relative_eq!(fiber.phi_y, expected_phi_y, epsilon = 1e-12);
    assert_relative_eq!(fiber.phi_z, expected_phi_z, epsilon = 1e-12);

    let mass_fiber = fiber.mass_matrix(crate::behavior::MassOption::Consistent);
    assert!(mass_fiber.get(1, 1).is_finite());
}

#[test]
fn rc_src_cftの材料領域質量はbeamとfiberの全成分で一致する() {
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    let rebar = RcRebar {
        main_x: BarSet {
            count: 4,
            dia: 19.0,
            layers: 2,
        },
        main_y: BarSet {
            count: 4,
            dia: 19.0,
            layers: 2,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 150.0,
            legs: 2,
        },
    };
    for (shape, main_category, with_rebar, with_steel) in [
        (
            SectionShape::RcRect {
                b: 500.0,
                d: 600.0,
                rebar: rebar.clone(),
            },
            MaterialCategory::Concrete,
            true,
            false,
        ),
        (
            SectionShape::SrcRect {
                b: 500.0,
                d: 600.0,
                rebar: rebar.clone(),
                steel_height: 400.0,
                steel_width: 200.0,
                steel_web_thick: 10.0,
                steel_flange_thick: 16.0,
            },
            MaterialCategory::Concrete,
            true,
            true,
        ),
        (
            SectionShape::CftBox {
                height: 400.0,
                width: 400.0,
                thick: 16.0,
            },
            MaterialCategory::Steel,
            false,
            false,
        ),
    ] {
        let mut model = build_test_model(Some(78846.15));
        model.sections[0].shape = Some(shape);
        model.sections[0].material =
            Some(MaterialId(if main_category == MaterialCategory::Steel {
                0
            } else {
                1
            }));
        model.sections[0].rebar_material = with_rebar.then_some(MaterialId(2));
        model.sections[0].shear_rebar_material = with_rebar.then_some(MaterialId(2));
        model.sections[0].steel_material = with_steel.then_some(MaterialId(3));
        model.materials[0].density = 2.4e-9;
        model.materials[0].category = main_category;
        model.materials[0].fc = (main_category == MaterialCategory::Steel).then_some(30.0);
        model.materials[0].young = 205000.0;
        model.materials.push(Material {
            density: 2.4e-9,
            category: MaterialCategory::Concrete,
            young: 25000.0,
            poisson: 0.2,
            fc: Some(24.0),
            ..model.materials[0].clone()
        });
        model.materials.push(Material {
            density: 7.8e-9,
            category: MaterialCategory::Rebar,
            young: 200000.0,
            poisson: 0.3,
            fy: Some(400.0),
            ..model.materials[0].clone()
        });
        model.materials.push(Material {
            density: 7.8e-9,
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            fy: Some(325.0),
            ..model.materials[0].clone()
        });

        let beam = crate::frame::beam::BeamElement::new(&model.elements[0], &model);
        let fiber = FiberBeam::new(
            &model.elements[0],
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
        );
        let mass_beam = beam.mass_matrix(crate::behavior::MassOption::Consistent);
        let mass_fiber = fiber.mass_matrix(crate::behavior::MassOption::Consistent);
        let lumped_fiber = fiber.mass_matrix(crate::behavior::MassOption::Lumped);
        let expected_lumped_mass = fiber.density
            * fiber.gauss_points[0]
                .section
                .fibers
                .iter()
                .map(|fiber| fiber.area)
                .sum::<f64>()
            * fiber.length;
        assert_relative_eq!(
            lumped_fiber.get(0, 0) * 2.0,
            expected_lumped_mass,
            epsilon = 1.0e-12
        );
        assert!(lumped_fiber.get(0, 0).is_finite() && lumped_fiber.get(3, 3) == 0.0);
        let expected_mass = model
            .element_mass_properties(&model.elements[0])
            .expect("テスト断面の質量特性を解決できる");
        assert_relative_eq!(
            expected_mass.mass_per_length * beam.length,
            beam.mass_properties.total_mass(beam.length),
            epsilon = 1.0e-10
        );
        for i in 0..12 {
            for j in 0..12 {
                assert!(
                    (mass_beam.get(i, j) - mass_fiber.get(i, j)).abs()
                        <= 1.0e-3 * (1.0 + mass_beam.get(i, j).abs() + mass_fiber.get(i, j).abs()),
                    "Beam/Fiber の全12x12質量が不一致: M({i},{j}) beam={} fiber={}",
                    mass_beam.get(i, j),
                    mass_fiber.get(i, j)
                );
            }
        }
    }
}

#[test]
fn 非対称断面のbeamとfiberは各曲げブロックが一致する() {
    let density = 2.4e-9;
    let mut model = build_test_model(Some(78846.15));
    model.materials[0].density = density;
    model.sections[0].as_y = 12000.0;
    model.sections[0].as_z = 18000.0;
    model.elements[0].rigid_zone.length_i = 300.0;
    model.elements[0].rigid_zone.length_j = 200.0;

    let fiber = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    let mut beam = make_test_beam_element(model.sections[0].as_z);
    beam.as_y = model.sections[0].as_y;
    beam.as_z = model.sections[0].as_z;
    beam.iy = model.sections[0].iy;
    beam.iz = model.sections[0].iz;
    beam.rigid = model.elements[0].rigid_zone;
    beam.density = density;
    beam.mass_properties = squid_n_core::model::SectionMassProperties::uniform(
        density,
        model.sections[0].area,
        model.sections[0].iy,
        model.sections[0].iz,
    );
    let mass_beam = beam.mass_matrix(crate::behavior::MassOption::Consistent);
    let mass_fiber = fiber.mass_matrix(crate::behavior::MassOption::Consistent);
    for indices in [[1usize, 5, 7, 11], [2, 4, 8, 10]] {
        for &i in &indices {
            for &j in &indices {
                assert!(
                    (mass_beam.get(i, j) - mass_fiber.get(i, j)).abs() < 1e-10,
                    "非対称断面の曲げブロック M({i},{j}) が Beam/Fiber で不一致: beam={}, fiber={}",
                    mass_beam.get(i, j),
                    mass_fiber.get(i, j)
                );
            }
        }
    }
}

#[test]
fn 端部解放質量はbeamとfiberで一致し剛体並進質量を保存する() {
    for end_condition in [
        EndCondition::Pinned,
        EndCondition::SemiRigid { k_theta: 2.0e8 },
    ] {
        let density = 2.4e-9;
        let mut model = build_test_model(Some(78846.15));
        model.materials[0].density = density;
        model.sections[0].j = 1.0e8;
        model.sections[0].as_y = 15000.0;
        model.sections[0].as_z = 15000.0;
        model.elements[0].end_cond = [end_condition, EndCondition::Fixed];
        model.elements[0].rigid_zone.length_i = 300.0;
        model.elements[0].rigid_zone.length_j = 200.0;
        let fiber = FiberBeam::new(
            &model.elements[0],
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
        );
        let beam = crate::frame::beam::BeamElement::new(&model.elements[0], &model);

        let mass_beam = beam.mass_matrix(crate::behavior::MassOption::Consistent);
        let mass_fiber = fiber.mass_matrix(crate::behavior::MassOption::Consistent);
        assert_mass_positive_semidefinite(&mass_beam, "Beam");
        assert_mass_positive_semidefinite(&mass_fiber, "Fiber");
        for i in 0..12 {
            assert!(mass_beam.get(i, i) >= -1.0e-12);
            assert!(mass_fiber.get(i, i) >= -1.0e-12);
            for j in 0..12 {
                assert!((mass_beam.get(i, j) - mass_beam.get(j, i)).abs() < 1.0e-9);
                assert!((mass_fiber.get(i, j) - mass_fiber.get(j, i)).abs() < 1.0e-9);
                assert!(mass_beam.get(i, j).is_finite());
                assert!(mass_fiber.get(i, j).is_finite());
            }
        }
        for translation in 0..3 {
            let mut u = [0.0; 12];
            u[translation] = 1.0;
            u[translation + 6] = 1.0;
            let total = (0..12)
                .flat_map(|i| (0..12).map(move |j| (i, j)))
                .map(|(i, j)| u[i] * mass_beam.get(i, j) * u[j])
                .sum::<f64>();
            assert!((total - beam.mass_properties.total_mass(beam.length)).abs() < 1.0e-8);
            let fiber_total = (0..12)
                .flat_map(|i| (0..12).map(move |j| (i, j)))
                .map(|(i, j)| u[i] * mass_fiber.get(i, j) * u[j])
                .sum::<f64>();
            assert!((fiber_total - fiber.mass_properties.total_mass(fiber.length)).abs() < 1.0e-8);
        }
        for i in [4, 5, 10, 11] {
            assert!(
                mass_fiber.get(i, i) > 1.0e-6,
                "Fiber の回転慣性対角 M({i},{i}) が期待値 1e-6 を下回る"
            );
        }
        for (i, j) in [(1, 5), (5, 7), (7, 11), (2, 4), (4, 8), (8, 10)] {
            assert!(
                mass_fiber.get(i, j).abs() > 1.0e-6,
                "Fiber の並進回転結合 M({i},{j}) が期待値 1e-6 を下回る"
            );
        }
    }
}

fn assert_mass_positive_semidefinite(matrix: &crate::behavior::LocalMat, name: &str) {
    let trace = (0..12).map(|i| matrix.get(i, i)).sum::<f64>();
    let vectors = [
        [1.0; 12],
        [
            1.0, -1.0, 2.0, -2.0, 3.0, -3.0, 4.0, -4.0, 5.0, -5.0, 6.0, -6.0,
        ],
        [
            2.0, 3.0, 5.0, 7.0, 11.0, 13.0, 17.0, 19.0, 23.0, 29.0, 31.0, 37.0,
        ],
        [
            0.0, 1.0, 0.0, -2.0, 3.0, 0.0, -5.0, 7.0, 0.0, -11.0, 13.0, 0.0,
        ],
    ];
    for (vector_index, u) in vectors.iter().enumerate() {
        let quadratic = (0..12)
            .flat_map(|i| (0..12).map(move |j| (i, j)))
            .map(|(i, j)| u[i] * matrix.get(i, j) * u[j])
            .sum::<f64>();
        let norm_squared = u.iter().map(|value| value * value).sum::<f64>();
        assert!(
            quadratic >= -1.0e-10 * trace * norm_squared,
            "{name} 端部解放質量のPSD検証に失敗: vector={vector_index}, uᵀMu={quadratic:e}, trace={trace:e}"
        );
    }
}

#[test]
fn 端部解放質量は独立縮約結果の全成分と一致する() {
    for end_condition in [
        EndCondition::Fixed,
        EndCondition::Pinned,
        EndCondition::SemiRigid { k_theta: 2.0e8 },
    ] {
        let density = 2.4e-9;
        let mut model = build_test_model(Some(78846.15));
        model.materials[0].density = density;
        model.sections[0].as_y = 15000.0;
        model.sections[0].as_z = 15000.0;
        model.elements[0].end_cond = [end_condition, EndCondition::Fixed];

        let beam = crate::frame::beam::BeamElement::new(&model.elements[0], &model);
        let fiber = FiberBeam::new(
            &model.elements[0],
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
        );
        let actual_beam = beam.mass_matrix(crate::behavior::MassOption::Consistent);
        let actual_fiber = fiber.mass_matrix(crate::behavior::MassOption::Consistent);

        let mut unreleased = model.elements[0].clone();
        unreleased.end_cond = [EndCondition::Fixed, EndCondition::Fixed];
        let base_beam = crate::frame::beam::BeamElement::new(&unreleased, &model);
        let base_mass = base_beam.mass_matrix(crate::behavior::MassOption::Consistent);
        let expected_beam =
            independent_released_mass(&base_beam.local_stiffness_raw(), &base_mass, end_condition);
        let base_fiber = FiberBeam::new(
            &unreleased,
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
        );
        let base_fiber_mass = base_fiber.mass_matrix(crate::behavior::MassOption::Consistent);
        let expected_fiber = independent_released_mass(
            &base_fiber.initial_elastic_stiffness,
            &base_fiber_mass,
            end_condition,
        );

        for i in 0..12 {
            for j in 0..12 {
                let expected_beam_ij = expected_beam[i][j];
                let expected_fiber_ij = expected_fiber[i][j];
                assert!(
                    (actual_beam.get(i, j) - expected_beam_ij).abs()
                        < 1e-8 * (1.0 + expected_beam_ij.abs()),
                    "beam M({i},{j}) actual={} expected={expected_beam_ij}",
                    actual_beam.get(i, j)
                );
                assert!(
                    (actual_fiber.get(i, j) - expected_fiber_ij).abs()
                        < 1e-8 * (1.0 + expected_fiber_ij.abs()),
                    "fiber M({i},{j}) actual={} expected={expected_fiber_ij}",
                    actual_fiber.get(i, j)
                );
            }
        }
    }
}

fn independent_released_mass(
    k_elem: &crate::behavior::LocalMat,
    m_elem: &crate::behavior::LocalMat,
    end_condition: EndCondition,
) -> [[f64; 12]; 12] {
    let Some(k_spring) = (match end_condition {
        EndCondition::Fixed => None,
        EndCondition::Pinned => Some(0.0),
        EndCondition::SemiRigid { k_theta } => Some(k_theta),
    }) else {
        return std::array::from_fn(|i| std::array::from_fn(|j| m_elem.get(i, j)));
    };

    let released = [4usize, 5usize];
    let mut expanded_k = [[0.0; 14]; 14];
    let mut expanded_m = [[0.0; 14]; 14];
    let expanded_index = |dof: usize| match released.iter().position(|&r| r == dof) {
        Some(index) => 12 + index,
        None => dof,
    };
    for i in 0..12 {
        for j in 0..12 {
            let a = expanded_index(i);
            let b = expanded_index(j);
            expanded_k[a][b] += k_elem.get(i, j);
            expanded_m[a][b] += m_elem.get(i, j);
        }
    }
    for (index, &dof) in released.iter().enumerate() {
        let internal = 12 + index;
        expanded_k[dof][dof] += k_spring;
        expanded_k[internal][internal] += k_spring;
        expanded_k[dof][internal] -= k_spring;
        expanded_k[internal][dof] -= k_spring;
    }

    let kbb = [
        [expanded_k[12][12], expanded_k[12][13]],
        [expanded_k[13][12], expanded_k[13][13]],
    ];
    let determinant = kbb[0][0] * kbb[1][1] - kbb[0][1] * kbb[1][0];
    assert!(determinant.abs() > 1e-12, "独立参照の Kbb が特異");
    let mut r = [[0.0; 12]; 14];
    for i in 0..12 {
        r[i][i] = 1.0;
    }
    for j in 0..12 {
        let rhs = [-expanded_k[12][j], -expanded_k[13][j]];
        r[12][j] = (rhs[0] * kbb[1][1] - kbb[0][1] * rhs[1]) / determinant;
        r[13][j] = (kbb[0][0] * rhs[1] - rhs[0] * kbb[1][0]) / determinant;
    }

    std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            (0..14)
                .flat_map(|a| (0..14).map(move |b| (a, b)))
                .map(|(a, b)| r[a][i] * expanded_m[a][b] * r[b][j])
                .sum()
        })
    })
}

#[test]
#[should_panic(expected = "BeamElement の端部解放質量を縮約できません")]
fn beamはkbb特異時に端部解放質量を明示的に失敗させる() {
    let mut beam = make_test_beam_element(15000.0);
    beam.e = 0.0;
    beam.end_cond = [EndCondition::Pinned, EndCondition::Fixed];
    beam.mass_properties =
        squid_n_core::model::SectionMassProperties::uniform(2.4e-9, 20000.0, beam.iz, beam.iy);
    beam.mass_matrix(crate::behavior::MassOption::Consistent);
}

#[test]
#[should_panic(expected = "FiberBeam の端部解放質量を縮約できません")]
fn fiberはkbb特異時に端部解放質量を明示的に失敗させる() {
    let mut model = build_test_model(Some(78846.15));
    model.materials[0].young = 0.0;
    model.elements[0].end_cond = [EndCondition::Pinned, EndCondition::Fixed];
    let fiber = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    fiber.mass_matrix(crate::behavior::MassOption::Consistent);
}

#[test]
fn fiberはkbb特異時に剛性をkaaへフォールバックする() {
    let mut model = build_test_model(Some(78846.15));
    model.materials[0].young = 0.0;
    model.elements[0].end_cond = [EndCondition::Pinned, EndCondition::Fixed];
    let fiber = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    let k = fiber.condense_releases(&LocalMat::zeros(12));
    assert!(k.data.iter().all(|v| v.is_finite()));
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
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [3000.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Fiber,
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
            name: "steel".to_string(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: shear_mod,
            fc: None,
            fy: Some(1e20),
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
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: p1,
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Fiber,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
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
            name: "steel".to_string(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: Some(0.0),
            fc: None,
            fy: Some(1e20),
        }],
        ..Default::default()
    };
    FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    )
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
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [3000.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Fiber,
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
            name: "steel".to_string(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: Some(0.0),
            fc: None,
            fy,
        }],
        ..Default::default()
    };
    FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    )
}

/// ねじり剛性テスト用の FiberBeam を生成する。
/// 既知の G, J, L で Saint-Venant ねじり剛性を検証するため。
fn make_torsion_fiber_beam(g: f64, j: f64) -> FiberBeam {
    let mut model = build_test_model(Some(g));
    model.sections[0].j = j;
    FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    )
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
    let mut fx = make_oriented_fiber([0.0, 0.0, 0.0], [l, 0.0, 0.0], [0.0, 1.0, 0.0]);
    fx.update_state(&zero_du, false, &ctx);
    let kx = fx.tangent_stiffness(&ctx);
    let mut fz = make_oriented_fiber([0.0, 0.0, 0.0], [0.0, 0.0, l], [1.0, 0.0, 0.0]);
    fz.update_state(&zero_du, false, &ctx);
    let kz = fz.tangent_stiffness(&ctx);

    assert_relative_eq!(kz.get(2, 2), kx.get(0, 0), epsilon = 1.0);
    assert_relative_eq!(kz.get(0, 0), kx.get(1, 1), epsilon = 1.0);
    assert!(
        kz.get(0, 0) < kz.get(2, 2),
        "vertical column horizontal DOF must be bending (small), not axial (large): ux={}, uz={}",
        kz.get(0, 0),
        kz.get(2, 2)
    );
}

#[test]
fn 任意方向材の整合質量は独立な座標変換と一致する() {
    let fiber = make_oriented_fiber([0.0, 0.0, 0.0], [3000.0, 1200.0, 2400.0], [0.0, 1.0, 0.0]);
    let local = crate::frame::prismatic::consistent_mass_timoshenko(
        fiber.mass_properties,
        fiber.flex_length,
        fiber.phi_z,
        fiber.phi_y,
    );
    let expected = {
        let mut result = LocalMat::zeros(12);
        for i in 0..12 {
            for j in 0..12 {
                let mut value = 0.0;
                for a in 0..12 {
                    for b in 0..12 {
                        let ra_i = fiber.axis.rot[a % 3][i % 3];
                        let rb_j = fiber.axis.rot[b % 3][j % 3];
                        if a / 3 == i / 3 && b / 3 == j / 3 {
                            value += ra_i * local.get(a, b) * rb_j;
                        }
                    }
                }
                result.set(i, j, value);
            }
        }
        result
    };
    let actual = fiber.mass_matrix(crate::behavior::MassOption::Consistent);
    for i in 0..12 {
        for j in 0..12 {
            assert_relative_eq!(actual.get(i, j), expected.get(i, j), epsilon = 1.0e-10);
        }
    }
}

#[test]
fn test_elastic_stiffness_matches_beam() {
    let mut fiber = make_test_fiber_beam(Some(0.0));
    let beam = make_test_beam_element(1e30);

    let ctx = Ctx {
        model: &build_test_model(Some(0.0)),
    };

    let u = [
        1.0, 0.5, 0.3, 0.0, 0.001, 0.002, -0.5, 0.2, -0.1, 0.0, 0.003, -0.001,
    ];
    let du = LocalVec {
        data: SmallVec::from_slice(&u),
    };
    fiber.update_state(&du, true, &ctx);

    let k_fiber = fiber.tangent_stiffness(&ctx);
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

    let u = [
        1.0, 0.5, 0.3, 0.0, 0.001, 0.002, -0.5, 0.2, -0.1, 0.0, 0.003, -0.001,
    ];
    let du = LocalVec {
        data: SmallVec::from_slice(&u),
    };
    fiber.update_state(&du, true, &ctx);

    let k = fiber.tangent_stiffness(&ctx);
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

/// 弾性応答の手計算照合: 軸力は N=E·A_disc·ε、曲げは M=E·I_disc·κ となり、
/// 軸と曲げを同時に与えても互いに連成しないこと（断面格子の図心・対称性）。
#[test]
fn test_elastic_force_matches_hand_calc() {
    let model = build_test_model(Some(0.0));
    let ctx = Ctx { model: &model };
    let new_fiber = || {
        FiberBeam::new(
            &model.elements[0],
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
        )
    };

    let sample = new_fiber();
    let a_disc: f64 = sample.gauss_points[0]
        .section
        .fibers
        .iter()
        .map(|f| f.area)
        .sum();
    let iy_disc: f64 = sample.gauss_points[0]
        .section
        .fibers
        .iter()
        .map(|f| f.area * f.z * f.z)
        .sum();

    let eps0 = 0.001;
    let mut axial = new_fiber();
    axial.update_state(
        &LocalVec {
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
        },
        true,
        &ctx,
    );
    let f = axial.internal_force(&ctx);
    let expected_n = eps0 * 205000.0 * a_disc;
    assert_relative_eq!(f.data[0], -expected_n, epsilon = 1.0);
    assert_relative_eq!(f.data[6], expected_n, epsilon = 1.0);

    let ky = 1e-6;
    let mut bending = new_fiber();
    bending.update_state(
        &LocalVec {
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
        },
        true,
        &ctx,
    );
    let f = bending.internal_force(&ctx);
    let expected_my = ky * 205000.0 * iy_disc;
    assert_relative_eq!(f.data[4], expected_my, epsilon = 1.0);
    assert_relative_eq!(f.data[10], -expected_my, epsilon = 1.0);

    let eps0_combined = 0.0005;
    let mut combined = new_fiber();
    combined.update_state(
        &LocalVec {
            data: SmallVec::from_slice(&[
                0.0,
                0.0,
                0.0,
                0.0,
                ky * 3000.0 / 2.0,
                0.0,
                eps0_combined * 3000.0,
                0.0,
                0.0,
                0.0,
                -ky * 3000.0 / 2.0,
                0.0,
            ]),
        },
        true,
        &ctx,
    );
    let f = combined.internal_force(&ctx);
    assert_relative_eq!(f.data[0], -eps0_combined * 205000.0 * a_disc, epsilon = 1.0);
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
                    support_spring: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [3000.0, 0.0, 0.0],
                    restraint: Default::default(),
                    mass: None,
                    story: None,
                    support_spring: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Fiber,
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
                name: "steel".to_string(),
                category: MaterialCategory::Steel,
                young: 205000.0,
                poisson: 0.3,
                density: 0.0,
                shear: Some(0.0),
                fc: None,
                fy: Some(235.0),
            }],
            ..Default::default()
        };
        FiberBeam::new(
            &model.elements[0],
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
        )
    };

    let ctx = Ctx {
        model: &Model::default(),
    };

    let eps_y = 235.0 / 205000.0;
    // My 面（κy）の縁距離はファイバ座標の |z| 最大 = 幅/2 = 50mm
    // （ファイバ格子は y=せい・z=幅で、断面格子を 90° 回転して配置している）。
    let z_max = 50.0;
    let ky_y = eps_y / z_max;

    let iy_disc: f64 = fiber.gauss_points[0]
        .section
        .fibers
        .iter()
        .map(|f| f.area * f.z * f.z)
        .sum();

    let mut prev_ky = 0.0;
    for ratio in [0.5, 1.0, 2.0, 3.0] {
        let ky = ky_y * ratio;
        let dky = ky - prev_ky;
        prev_ky = ky;
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

        let my = fiber.internal_force(&ctx).data[4];
        let elastic_pred = ky * 205000.0 * iy_disc;
        // 鋼ファイバはなめらか降伏（MenegottoPinto）のため、ratio=1.0 では
        // 最外縁が降伏ひずみの約 0.92 倍に達し、既に弾性予測を下回る。
        // 弾性とみなす境界は section 側（Bilinear の ratio<=1.0）より手前になる。
        if ratio < 1.0 {
            assert_relative_eq!(my, elastic_pred, max_relative = 1e-6);
        } else {
            assert!(
                my < elastic_pred,
                "post-yield My ({}) must be below elastic prediction ({})",
                my,
                elastic_pred
            );
        }
    }
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
fn test_different_gp_have_independent_mats() {
    let fiber = make_test_fiber_beam(Some(0.0));
    let gp0_ptr = &fiber.gauss_points[0].mats[0] as *const _;
    let gp1_ptr = &fiber.gauss_points[1].mats[0] as *const _;
    assert_ne!(gp0_ptr, gp1_ptr, "GP mats must be independent instances");
}

/// ねじり剛性が G·J/L、ねじり内力が Mx = (G·J/L)·(θi − θj) となること（手計算照合）。
#[test]
fn test_torsional_stiffness_and_internal_force() {
    let g = 78846.0;
    let j = 1.0e6;
    let l = 3000.0;
    let expected_kt = g * j / l;

    let mut fiber = make_torsion_fiber_beam(g, j);
    let ctx = Ctx {
        model: &build_test_model(Some(g)),
    };
    let zero_du = LocalVec {
        data: SmallVec::from_elem(0.0, 12),
    };
    fiber.update_state(&zero_du, false, &ctx);

    let k = fiber.tangent_stiffness(&ctx);
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

    let theta_i = 0.01;
    let theta_j = -0.005;
    let du = LocalVec {
        data: smallvec::smallvec![
            0.0, 0.0, 0.0, theta_i, 0.0, 0.0, 0.0, 0.0, 0.0, theta_j, 0.0, 0.0,
        ],
    };
    fiber.update_state(&du, true, &ctx);
    let f = fiber.internal_force(&ctx);

    let expected_mx_i = expected_kt * (theta_i - theta_j);
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

#[test]
fn 不正な質量特性でもfiberはlumpedで生成できconsistentで失敗する() {
    let mut model = build_test_model(Some(78846.15));
    model.sections[0].shape = Some(squid_n_core::section_shape::SectionShape::RcRect {
        b: 400.0,
        d: 400.0,
        rebar: squid_n_core::section_shape::RcRebar {
            main_x: squid_n_core::section_shape::BarSet {
                count: 0,
                dia: 0.0,
                layers: 0,
            },
            main_y: squid_n_core::section_shape::BarSet {
                count: 0,
                dia: 0.0,
                layers: 0,
            },
            cover: 0.0,
            shear: squid_n_core::section_shape::ShearBar {
                dia: 0.0,
                pitch: 0.0,
                legs: 0,
            },
        },
    });
    model.materials[0].fc = Some(30.0);
    let mut fiber = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    fiber.mass_properties_error = Some("断面形状が不正です".into());
    assert!(fiber
        .mass_matrix(crate::behavior::MassOption::Lumped)
        .data
        .iter()
        .all(|v| v.is_finite()));
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        fiber.mass_matrix(crate::behavior::MassOption::Consistent)
    }));
    assert!(result.is_err());
}

#[test]
fn fiberの純ねじり行列は質量極二次モーメントを使う() {
    let g = 78846.0;
    let j = 1.0e6;
    let iy = 2.0e8;
    let iz = 5.0e7;
    let density = 7.85e-9;
    let length: f64 = 3000.0;
    let mut model = build_test_model(Some(g));
    model.sections[0].j = j;
    model.sections[0].iy = iy;
    model.sections[0].iz = iz;
    model.materials[0].density = density;
    let mut fiber = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    let ctx = Ctx { model: &model };
    let zero_du = LocalVec {
        data: SmallVec::from_elem(0.0, 12),
    };
    fiber.update_state(&zero_du, false, &ctx);
    let stiffness = fiber.tangent_stiffness(&ctx);
    let mass = fiber.mass_matrix(crate::behavior::MassOption::Consistent);
    let expected = 3.0 * g * j / (density * (iy + iz) * length.powi(2));
    let actual = stiffness.get(9, 9) / mass.get(9, 9);
    assert_relative_eq!(actual, expected, max_relative = 1e-10);
}

/// 鉛直柱（Z整列）でねじり剛性 GJ 追加後、グローバル rz DOF (index 5, 11) が
/// 特異でない（非ゼロの対角成分を持つ）ことを確認する回帰テスト。
#[test]
fn test_vertical_column_rz_nonsingular() {
    let g = 78846.0;
    let j = 1.0e6;
    let l = 3000.0;
    let expected_kt = g * j / l;

    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, l],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Fiber,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
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
            name: "steel".to_string(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: Some(g),
            fc: None,
            fy: Some(1e20),
        }],
        ..Default::default()
    };

    let mut fiber = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    let ctx = Ctx {
        model: &Model::default(),
    };
    let zero_du = LocalVec {
        data: SmallVec::from_elem(0.0, 12),
    };
    fiber.update_state(&zero_du, false, &ctx);

    let k = fiber.tangent_stiffness(&ctx);
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
    let _ = expected_kt;
}

/// 回帰テスト: 剛体回転（両端に同じ回転角 θ、曲率ゼロ）だけを与えても
/// 内力が発生しないこと（客観性）。
#[test]
fn test_fiber_rigid_rotation_produces_no_force() {
    let mut model = build_test_model(Some(78846.15));
    model.sections[0].as_y = 208333.0;
    model.sections[0].as_z = 208333.0;
    model.sections[0].depth = 500.0;
    model.sections[0].width = 500.0;
    model.sections[0].area = 250000.0;
    model.sections[0].iy = 5.2083333e9;
    model.sections[0].iz = 5.2083333e9;

    let mut fiber = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
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
    let f = fiber.internal_force(&ctx);
    for (i, v) in f.data.iter().enumerate() {
        assert!(
            v.abs() < 1.0,
            "剛体回転のみで内力が発生した（客観性違反）: dof {i} = {v}"
        );
    }
}

/// 回帰テスト: 弾性状態の初期横剛性が Timoshenko 理論値と一致すること。
/// 本テストは i 端固定の片持ち縮約剛性
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

    let mut fiber = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    let ctx = Ctx { model: &model };
    let zero = LocalVec {
        data: SmallVec::from_elem(0.0, 12),
    };
    fiber.update_state(&zero, false, &ctx);
    let k = fiber.tangent_stiffness(&ctx);

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
    let iz_elem = b_w * d_h.powi(3) / 12.0 * (1.0 - 1.0 / (nd * nd));
    let iy_elem = d_h * b_w.powi(3) / 12.0 * (1.0 - 1.0 / (nw * nw));
    let as_y_elem = 120000.0;
    let as_z_elem = 80000.0;
    let j = 1.0e6;

    let mut model = build_test_model(Some(g));
    model.sections[0].depth = d_h;
    model.sections[0].width = b_w;
    model.sections[0].area = area;
    model.sections[0].iy = iz_elem;
    model.sections[0].iz = iy_elem;
    model.sections[0].as_z = as_y_elem;
    model.sections[0].as_y = as_z_elem;
    model.sections[0].j = j;

    let mut fiber = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    let ctx = Ctx { model: &model };
    let zero = LocalVec {
        data: SmallVec::from_elem(0.0, 12),
    };
    fiber.update_state(&zero, false, &ctx);
    let k_fb = fiber.tangent_stiffness(&ctx);

    let mut be = make_test_beam_element(as_y_elem);
    be.a = area;
    be.a_mass = area;
    be.iy = iy_elem;
    be.iz = iz_elem;
    be.j = j;
    be.as_y = as_y_elem;
    be.as_z = as_z_elem;
    let k_be = be.tangent_stiffness(&ctx);

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
    let build = || {
        FiberBeam::with_plastic_zone(
            &model.elements[0],
            &model,
            250.0,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
        )
    };

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
    let f = fb.internal_force(&ctx);
    for (i, v) in f.data.iter().enumerate() {
        assert!(v.abs() < 1.0, "塑性化域+φ>0 で客観性違反: dof {i} = {v}");
    }

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
    let f0 = b0.internal_force(&ctx);
    let k = b0.tangent_stiffness(&ctx);
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
        let fp = bp.internal_force(&ctx);
        for i in 0..12 {
            let fd = (fp.data[i] - f0.data[i]) / h;
            let err = (fd - k.get(i, j)).abs() / kmax;
            assert!(
                err < 1e-6,
                "塑性化域+φ>0 で K≠∂f/∂u: ({i},{j}) 誤差={err:.3e}"
            );
        }
    }

    let mut fb2 = build();
    let zero = LocalVec {
        data: SmallVec::from_elem(0.0, 12),
    };
    fb2.update_state(&zero, false, &ctx);
    let k2 = fb2.tangent_stiffness(&ctx);
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
    let h = 1e-6;
    let u0: [f64; 12] = [
        0.1, 0.2, -0.1, 0.0005, 0.001, -0.0005, -0.05, 0.15, 0.1, -0.0005, 0.0008, 0.0002,
    ];

    let mut b0 = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    b0.update_state(
        &LocalVec {
            data: SmallVec::from_slice(&u0),
        },
        false,
        &ctx,
    );
    let f0 = b0.internal_force(&ctx);
    let k = b0.tangent_stiffness(&ctx);
    let kmax = (0..12)
        .flat_map(|i| (0..12).map(move |j| (i, j)))
        .map(|(i, j)| k.get(i, j).abs())
        .fold(0.0_f64, f64::max);

    for j in 0..12 {
        let mut up = u0;
        up[j] += h;
        let mut bp = FiberBeam::new(
            &model.elements[0],
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
        );
        bp.update_state(
            &LocalVec {
                data: SmallVec::from_slice(&up),
            },
            false,
            &ctx,
        );
        let fp = bp.internal_force(&ctx);
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
    FiberBeam::with_plastic_zone(
        &model.elements[0],
        &model,
        lp,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    )
}

/// 塑性化域考慮モデルの弾性剛性は、軸が EA/L（厳密）、曲げ・せん断が
/// 全長ファイバー積分モデル（`FiberBeam`）に近いこと。
#[test]
fn test_plastic_zone_elastic_stiffness_matches_full_fiber_and_axial() {
    let model = build_test_model(Some(0.0));
    let ctx = Ctx { model: &model };
    let full = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    let k_full = full.tangent_stiffness(&ctx);

    let pz = make_plastic_zone_fiber(150.0, Some(1e20));
    let k_pz = pz.tangent_stiffness(&ctx);
    for (i, j) in [(1usize, 1usize), (2, 2), (4, 4), (5, 5), (1, 5), (2, 4)] {
        assert_relative_eq!(k_pz.get(i, j), k_full.get(i, j), max_relative = 5e-2);
    }

    let pz_axial = make_plastic_zone_fiber(300.0, Some(1e20));
    let k_axial = pz_axial.tangent_stiffness(&ctx);
    let ea_over_l = 205000.0 * 20000.0 / 3000.0;
    assert_relative_eq!(k_axial.get(0, 0), ea_over_l, max_relative = 1e-9);
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
    let pz = make_plastic_zone_fiber(300.0, Some(1e20));
    let k_el = &pz
        .hinge
        .as_ref()
        .expect("plastic zone model has hinge")
        .k_el;
    let sec = &model.sections[0];
    let ratio = k_el.get(1, 1) / k_el.get(2, 2);
    let expected = sec.iy / sec.iz;
    assert!(
        (ratio - expected).abs() / expected < 1e-12,
        "k_el(1,1)/k_el(2,2)={} expected sec.iy/sec.iz={}",
        ratio,
        expected
    );
    assert!(k_el.get(1, 1) > k_el.get(2, 2));
}

#[test]
fn test_plastic_zone_yield_reduces_stiffness() {
    let mut fb = make_plastic_zone_fiber(300.0, Some(235.0));
    let model = build_test_model(Some(0.0));
    let ctx = Ctx { model: &model };
    let k0 = fb.tangent_stiffness(&ctx);

    let du = LocalVec {
        data: SmallVec::from_slice(&[0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    };
    fb.update_state(&du, false, &ctx);
    let k1 = fb.tangent_stiffness(&ctx);
    assert!(
        k1.get(4, 4) < 0.9 * k0.get(4, 4),
        "降伏後の回転剛性は低下するはず: k0={}, k1={}",
        k0.get(4, 4),
        k1.get(4, 4)
    );
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
    let f1 = fb.internal_force(&ctx);
    let f2 = fb2.internal_force(&ctx);
    for i in 0..12 {
        assert_relative_eq!(f1.data[i], f2.data[i], epsilon = 1e-6);
    }
}

/// RC 断面（RcRect＋配筋）のファイバー柱は、コンクリート格子に加えて主筋が
/// 点ファイバーとして分離配置される。
/// RC 断面（RcRect＋配筋、500 角・Fc30）のファイバー柱モデル。
fn rc_fiber_model() -> Model {
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
            },
        },
    };
    let mut sec = shape.to_section(SectionId(0), "C500".into());
    sec.material = Some(MaterialId(0));
    sec.rebar_material = Some(MaterialId(1));
    sec.shear_rebar_material = Some(MaterialId(1));
    Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Fiber,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
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
        materials: vec![
            Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "FC30".into(),
                category: MaterialCategory::Concrete,
                young: 25000.0,
                poisson: 0.2,
                density: 0.0,
                shear: Some(0.0),
                fc: Some(30.0),
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
                density: 0.0,
                shear: None,
                fc: None,
                fy: Some(345.0),
            },
        ],
        ..Default::default()
    }
}

#[test]
fn test_rc_fiber_section_includes_separated_rebar() {
    let model = rc_fiber_model();
    let fb = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    let gp = &fb.gauss_points[0];
    assert!(
        gp.section.fibers.len() > 240,
        "主筋ファイバーが分離配置されていない: {}",
        gp.section.fibers.len()
    );
    let rebar_count = gp.section.fibers.iter().filter(|f| f.material == 1).count();
    assert_eq!(rebar_count, 16, "主筋本数（上下8＋側面8）: {rebar_count}");
    let max_abs_z = gp
        .section
        .fibers
        .iter()
        .filter(|f| f.material == 1)
        .map(|f| f.z.abs())
        .fold(0.0_f64, f64::max);
    assert!(max_abs_z > 180.0, "主筋が最外縁近くにない: {max_abs_z}");
}

/// ファイバー材料は、**未経験状態でひずみ 0 を与えたとき初期弾性係数を返す**
/// （[`squid_n_material::uniaxial::UniaxialMaterial::trial`] の共通要件）。
///
/// `GaussPoint::new` はこの値を断面の初期接線としてキャッシュし、塑性化域考慮
/// モデルはそこから「弾性状態でヒンジ回転 0」を成立させる基準剛性 `sec_ei` を採る。
/// 0 を返す材料が 1 つでも混ざると、その断面の弾性曲げ剛性が過小になって
/// 弾性域でも要素接線剛性が負になる。骨格式は原点で応力 0 になるため、終局域の
/// ゼロクランプに巻き込まれやすい箇所であり、**実際に使う全材料**を通しで確かめる。
#[test]
fn test_all_fiber_materials_return_initial_tangent_at_zero_strain() {
    use squid_n_core::model::HysteresisModel;

    for rule in [
        HysteresisModel::Retrograde,
        HysteresisModel::OriginOriented,
        HysteresisModel::KarsanJirsa,
    ] {
        for fc in [21.0, 60.0, 80.0] {
            let expected = if fc <= 60.0 {
                squid_n_material::newrc::NewRcEnvelope::new(fc).ec
            } else {
                2.0 * fc / 0.002
            };
            let mut m = concrete_fiber_material(Some(fc), rule);
            let (s, t) = m.trial(0.0);
            assert_eq!(s, 0.0, "rule={rule:?} fc={fc}: ひずみ 0 で応力が 0 でない");
            assert_relative_eq!(t, expected, max_relative = 1e-9);
        }
    }

    let mut steel = steel_fiber_material(205000.0, Some(345.0));
    let (s, t) = steel.trial(0.0);
    assert_eq!(s, 0.0);
    assert_relative_eq!(t, 205000.0, max_relative = 1e-9);
}

#[test]
#[should_panic(expected = "設計基準強度 Fc が未設定です")]
fn concrete_fiber_material_rejects_missing_fc() {
    concrete_fiber_material(None, HysteresisModel::Retrograde);
}

/// 塑性化域考慮ファイバー梁（RC 断面）は、**弾性域では接線剛性が正定値**である。
///
/// ヒンジ回転 γ は「断面の弾性線を超える塑性超過分」
/// \( \gamma = s L_p (\kappa - m_{sec}/EI_{sec}) \) として定義され、\( EI_{sec} \)
/// （`sec_ei`）は要素生成時の断面接線をそのまま基準にする。したがって
/// **ひずみ 0 の断面接線が実際の弾性接線と一致していなければならない**。
/// コンクリート材料がひずみ 0 で接線 0 を返すと \( EI_{sec} \) が主筋分だけになり、
/// 弾性状態でも \( \kappa - m_{sec}/EI_{sec} \neq 0 \) となって静縮約が破綻し、
/// 接線剛性の対角が負になる（増分解析が長期載荷の時点で解けなくなる）。
#[test]
fn test_rc_plastic_zone_fiber_tangent_stays_positive_in_elastic_range() {
    let model = rc_fiber_model();
    let ctx = Ctx { model: &model };
    let mut fb = FiberBeam::with_plastic_zone(
        &model.elements[0],
        &model,
        250.0,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );

    let h = fb.hinge.as_ref().expect("塑性化域ヒンジが構築されていない");
    let d0 = fb.gauss_points[0].cached_stiff;
    assert_relative_eq!(h.sec_ei[0][1], d0[2][2], max_relative = 1e-12);
    let nominal_eiz = 25000.0 * model.sections[0].iy;
    assert!(
        d0[2][2] > 0.5 * nominal_eiz && d0[2][2] < 2.0 * nominal_eiz,
        "初期断面剛性が公称 E·I とかけ離れている: EIz_sec={}, E·Iy={}",
        d0[2][2],
        nominal_eiz
    );

    let du = LocalVec {
        data: SmallVec::from_slice(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0, 0.0, 0.0]),
    };
    fb.update_state(&du, false, &ctx);
    let k = fb.tangent_stiffness(&ctx);
    for i in 0..12 {
        assert!(
            k.get(i, i) >= 0.0,
            "弾性域なのに接線剛性の対角が負: K[{i}][{i}]={}",
            k.get(i, i)
        );
    }
}

/// 剛域長 λi・λj を与えたテストモデル（節点間長 3000mm、500 角・せん断断面付き）。
fn build_rigid_zone_model(li: f64, lj: f64) -> Model {
    let mut model = build_test_model(Some(78846.15));
    set_square500_shear_section(&mut model);
    model.elements[0].rigid_zone = squid_n_core::model::RigidZone {
        length_i: li,
        length_j: lj,
        face_i: Some(li),
        face_j: Some(lj),
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
    let iz_elem = b_w * d_h.powi(3) / 12.0 * (1.0 - 1.0 / (nd * nd));
    let iy_elem = d_h * b_w.powi(3) / 12.0 * (1.0 - 1.0 / (nw * nw));
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
    let mut fiber = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    assert_relative_eq!(fiber.flex_length, l_flex, max_relative = 1e-12);
    fiber.update_state(&zero, false, &ctx);
    let k_fb = fiber.tangent_stiffness(&ctx);

    let mut be = make_test_beam_element(as_y_elem);
    be.a = area;
    be.a_mass = area;
    be.iy = iy_elem;
    be.iz = iz_elem;
    be.j = j;
    be.as_y = as_y_elem;
    be.as_z = as_z_elem;
    be.rigid = model.elements[0].rigid_zone;
    let k_be = be.tangent_stiffness(&ctx);

    let kmax = (0..12)
        .flat_map(|i| (0..12).map(move |j| (i, j)))
        .map(|(i, j)| k_be.get(i, j).abs())
        .fold(0.0_f64, f64::max);
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
    let ea = 205000.0 * area;
    assert_relative_eq!(k_be.get(0, 0), ea / l, max_relative = 1e-9);
    assert_relative_eq!(k_fb.get(0, 0), ea / l_flex, max_relative = 1e-9);
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
        let mut fb = FiberBeam::new(
            &model.elements[0],
            model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
        );
        fb.update_state(
            &LocalVec {
                data: SmallVec::from_elem(0.0, 12),
            },
            false,
            &ctx,
        );
        let k = fb.tangent_stiffness(&ctx);
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
    let mut fiber = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );

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
    let f = fiber.internal_force(&ctx);
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
    let h = 1e-6;
    let u0: [f64; 12] = [
        0.1, 0.2, -0.1, 0.0005, 0.001, -0.0005, -0.05, 0.15, 0.1, -0.0005, 0.0008, 0.0002,
    ];

    let mut b0 = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    b0.update_state(
        &LocalVec {
            data: SmallVec::from_slice(&u0),
        },
        false,
        &ctx,
    );
    let f0 = b0.internal_force(&ctx);
    let k = b0.tangent_stiffness(&ctx);
    let kmax = (0..12)
        .flat_map(|i| (0..12).map(move |j| (i, j)))
        .map(|(i, j)| k.get(i, j).abs())
        .fold(0.0_f64, f64::max);

    for j in 0..12 {
        let mut up = u0;
        up[j] += h;
        let mut bp = FiberBeam::new(
            &model.elements[0],
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
        );
        bp.update_state(
            &LocalVec {
                data: SmallVec::from_slice(&up),
            },
            false,
            &ctx,
        );
        let fp = bp.internal_force(&ctx);
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
    let fb = FiberBeam::with_plastic_zone(
        &model.elements[0],
        &model,
        lp,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );

    assert_relative_eq!(fb.flex_length, l_flex, max_relative = 1e-12);
    assert_eq!(fb.gauss_points.len(), 2);
    for gp in &fb.gauss_points {
        assert_relative_eq!(gp.xi.abs(), 1.0, max_relative = 1e-12);
        assert_relative_eq!(gp.weight, 2.0 * lp / l_flex, max_relative = 1e-12);
    }
    assert!(fb.hinge.is_some(), "塑性増分ヒンジが構築されていない");
}

/// 剛域長の合計が節点間長以上になる病的な入力は、剛域なしとして扱う
/// （可撓長ゼロで要素が退化するのを防ぐ）。
#[test]
fn 可撓長が残らない剛域は無視される() {
    let model = build_rigid_zone_model(2000.0, 1500.0);
    let fb = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    assert_eq!(fb.rigid_i, 0.0);
    assert_eq!(fb.rigid_j, 0.0);
    assert_relative_eq!(fb.flex_length, fb.length, max_relative = 1e-12);
}

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
    let mut fb = FiberBeam::new(
        &model.elements[0],
        model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
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
    let k_fb = fiber.tangent_stiffness(&ctx);

    let mut be = make_test_beam_element(as_y_elem);
    be.a = area;
    be.a_mass = area;
    be.iy = iy_elem;
    be.iz = iz_elem;
    be.j = j;
    be.as_y = as_y_elem;
    be.as_z = as_z_elem;
    be.end_cond = model.elements[0].end_cond;
    let k_be = be.tangent_stiffness(&ctx);

    let kmax = (0..12)
        .flat_map(|i| (0..12).map(move |j| (i, j)))
        .map(|(i, j)| k_be.get(i, j).abs())
        .fold(0.0_f64, f64::max);
    for i in 0..12 {
        for j in 0..12 {
            if [0, 6].contains(&i) || [0, 6].contains(&j) {
                continue;
            }
            let diff = (k_fb.get(i, j) - k_be.get(i, j)).abs();
            assert!(
                diff <= 1e-9 * kmax,
                "K({i},{j}) が材端ピンの弾性梁と不一致: fiber={}, beam={}, 差={diff:.3e}",
                k_fb.get(i, j),
                k_be.get(i, j)
            );
        }
        for r in [3usize, 4, 5] {
            assert!(
                k_fb.get(r, i).abs() < 1e-6 * kmax.max(1.0),
                "ピン端の回転自由度 {r} に剛性が残っている: K({r},{i})={}",
                k_fb.get(r, i)
            );
        }
    }
    assert!(k_fb.get(11, 11) > 0.0);
}

/// ピン端では、その端に曲げモーメント内力が生じないこと（厳密なモーメント解放）。
/// 剛接端との比較で、解放が実際に効いていることを確認する。
#[test]
fn 材端ピンでは当該端の曲げモーメントがゼロになる() {
    let pinned = build_release_model([EndCondition::Pinned, EndCondition::Fixed]);
    let fixed = build_release_model([EndCondition::Fixed, EndCondition::Fixed]);
    let du = |uy: f64| LocalVec {
        data: SmallVec::from_slice(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, uy, 0.0, 0.0, 0.0, 0.0]),
    };

    let ctx_p = Ctx { model: &pinned };
    let mut fb_p = FiberBeam::new(
        &pinned.elements[0],
        &pinned,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    fb_p.update_state(&du(1.0), false, &ctx_p);
    let f_p = fb_p.internal_force(&ctx_p);

    let ctx_f = Ctx { model: &fixed };
    let mut fb_f = FiberBeam::new(
        &fixed.elements[0],
        &fixed,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    fb_f.update_state(&du(1.0), false, &ctx_f);
    let f_f = fb_f.internal_force(&ctx_f);

    assert!(
        f_f.data[5].abs() > 1.0e6,
        "剛接端の Mz が小さすぎる: {}",
        f_f.data[5]
    );
    assert!(
        f_p.data[5].abs() < 1e-9 * f_f.data[5].abs(),
        "ピン端に曲げモーメントが残っている: {}",
        f_p.data[5]
    );
    assert!(
        f_p.data[7].abs() < f_f.data[7].abs(),
        "ピン解放で横剛性が下がっていない"
    );
}

/// 半剛（回転ばね）は、剛接とピンの中間の剛性になること。
/// ばね剛性 →∞ で剛接、→0 でピンに漸近する。
#[test]
fn 半剛端は剛接とピンの中間になる() {
    let rot_stiffness = |end_cond: [EndCondition; 2]| -> f64 {
        let model = build_release_model(end_cond);
        let ctx = Ctx { model: &model };
        let fb = elastic_fiber(&model);
        fb.tangent_stiffness(&ctx).get(11, 11)
    };
    let k_fixed = rot_stiffness([EndCondition::Fixed, EndCondition::Fixed]);
    let k_pin = rot_stiffness([EndCondition::Pinned, EndCondition::Fixed]);
    let k_theta = 6.0 * 205000.0 * 5.2083333e9 / 3000.0;
    let k_semi = rot_stiffness([EndCondition::SemiRigid { k_theta }, EndCondition::Fixed]);

    assert!(
        k_pin < k_semi && k_semi < k_fixed,
        "半剛が剛接とピンの中間になっていない: pin={k_pin:.4e}, semi={k_semi:.4e}, fixed={k_fixed:.4e}"
    );
    assert!(
        (k_pin / k_fixed - 0.75).abs() < 0.05,
        "ピン端の回転剛性比が 3/4 から外れている: {:.4}",
        k_pin / k_fixed
    );
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
/// i 端半剛・j 端ピンは両端に解放を持ち、ピンと回転ばねの双方を最も多く含む代表ケース。
#[test]
fn 材端解放ありでも接線剛性が内力の勾配と一致する() {
    let end_cond = [
        EndCondition::SemiRigid { k_theta: 2.0e12 },
        EndCondition::Pinned,
    ];
    let mut model = build_release_model(end_cond);
    model.elements[0].rigid_zone = squid_n_core::model::RigidZone {
        length_i: 400.0,
        length_j: 250.0,
        face_i: Some(400.0),
        face_j: Some(250.0),
        ..Default::default()
    };
    let ctx = Ctx { model: &model };
    let h = 1e-6;
    let u0: [f64; 12] = [
        0.1, 0.2, -0.1, 0.0005, 0.001, -0.0005, -0.05, 0.15, 0.1, -0.0005, 0.0008, 0.0002,
    ];

    let mut b0 = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    b0.update_state(
        &LocalVec {
            data: SmallVec::from_slice(&u0),
        },
        false,
        &ctx,
    );
    let f0 = b0.internal_force(&ctx);
    let k = b0.tangent_stiffness(&ctx);
    let kmax = (0..12)
        .flat_map(|i| (0..12).map(move |j| (i, j)))
        .map(|(i, j)| k.get(i, j).abs())
        .fold(0.0_f64, f64::max);

    for j in 0..12 {
        let mut up = u0;
        up[j] += h;
        let mut bp = FiberBeam::new(
            &model.elements[0],
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
        );
        bp.update_state(
            &LocalVec {
                data: SmallVec::from_slice(&up),
            },
            false,
            &ctx,
        );
        let fp = bp.internal_force(&ctx);
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

/// 材端解放があっても剛体回転で内力が生じないこと（客観性）。
#[test]
fn 材端解放ありでも剛体回転で内力が生じない() {
    let model = build_release_model([EndCondition::Pinned, EndCondition::Fixed]);
    let ctx = Ctx { model: &model };
    let mut fiber = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
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
    let f = fiber.internal_force(&ctx);
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
    let mut fb = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );

    for _ in 0..40 {
        let du = LocalVec {
            data: SmallVec::from_slice(&[
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0,
            ]),
        };
        fb.update_state(&du, true, &ctx);
    }
    let f = fb.internal_force(&ctx);
    let m_fixed = f.data[11].abs().max(1.0);
    assert!(
        f.data[5].abs() < 1e-8 * m_fixed,
        "降伏後にピン端へモーメントが残った: Mz_i={}, Mz_j={}",
        f.data[5],
        f.data[11]
    );
    let k = fb.tangent_stiffness(&ctx);
    let k0 = elastic_fiber(&build_release_model([
        EndCondition::Pinned,
        EndCondition::Fixed,
    ]))
    .tangent_stiffness(&ctx);
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
fn ねじり剛性がない部材はrxを解放しない() {
    let mut model = build_release_model([EndCondition::Pinned, EndCondition::Pinned]);
    model.sections[0].j = 0.0;
    let fb = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    assert!(
        fb.releases.iter().all(|r| r.dof != 3 && r.dof != 9),
        "J=0 で rx が解放された: {:?}",
        fb.releases
    );
    assert_eq!(fb.releases.len(), 4);

    model.sections[0].j = 1.0e6;
    let fb = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
    assert_eq!(fb.releases.len(), 6);
}

/// 材端解放の内部自由度がチェックポイント／スナップショットで往復すること。
#[test]
fn 材端解放の内部自由度がチェックポイントで往復する() {
    let model = build_release_model([EndCondition::Pinned, EndCondition::Fixed]);
    let ctx = Ctx { model: &model };
    let mut fb = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
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
    let mut restored = FiberBeam::new(
        &model.elements[0],
        &model,
        StrengthBasis::Nominal,
        AnalysisKind::Incremental,
    );
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
    let big = 0.2;
    let du = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, big, 0.0, 0.0, 0.0, 0.0, 0.0, -big],
    };
    let mut elem = make_steel_fiber_with_fy(Some(235.0));
    elem.update_state(&du, true, &ctx);

    let mf = elem
        .state_member_forces(&ctx)
        .expect("ファイバー梁は状態から内力を返す");
    assert!(mf.at.iter().any(|(xi, _)| xi.abs() < 1e-12));
    assert!(mf.at.iter().any(|(xi, _)| (xi - 0.5).abs() < 1e-12));
    assert!(mf.at.iter().any(|(xi, _)| (xi - 1.0).abs() < 1e-12));

    let f = elem.internal_force(&ctx);
    let mz_i = mf
        .at
        .iter()
        .find(|(xi, _)| xi.abs() < 1e-12)
        .map(|(_, v)| v[5])
        .unwrap();
    assert_relative_eq!(mz_i, -f.data[5], epsilon = 1e-6);
    assert!(f.data[5].abs() > 1.0, "前提: 曲げが有意であること");

    let k = elem.tangent_stiffness(&ctx);
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
    let du = LocalVec {
        data: smallvec::smallvec![0.5, 0.0, 0.0, 0.0, 0.05, 0.2, -0.5, 0.0, 0.0, 0.0, -0.03, -0.1],
    };
    let mut elem = make_steel_fiber_with_fy(Some(235.0));
    elem.eval_sections = vec![0.0, 0.25, 0.5, 0.75, 1.0];
    elem.update_state(&du, true, &ctx);

    let mf = elem.state_member_forces(&ctx).unwrap();
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
        assert_relative_eq!(v[0], a[0], max_relative = 1e-9, epsilon = 1e-6);
        assert_relative_eq!(v[1], a[1], max_relative = 1e-9, epsilon = 1e-6);
        assert_relative_eq!(v[2], a[2], max_relative = 1e-9, epsilon = 1e-6);
        assert_relative_eq!(v[3], a[3], max_relative = 1e-9, epsilon = 1e-6);
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
        FiberYield {
            main: Some(295.0),
            rebar: None,
            steel: Some(295.0),
        },
        1.0,
        1.0,
        StrengthParams {
            steel_fy: 295.0,
            rebar_fy: 295.0,
            concrete_fc: 24.0,
            steel_e: 205000.0,
        },
        HysteresisModel::Retrograde,
    )
    .expect("鋼断面は配筋不要");
    assert_eq!(sec.fibers.len(), mats.len());

    let a_sum: f64 = sec.fibers.iter().map(|f| f.area).sum();
    let a_exact = 400.0_f64 * 400.0 - 376.0 * 376.0;
    assert_relative_eq!(a_sum, a_exact, max_relative = 1e-9);

    let i_sum: f64 = sec.fibers.iter().map(|f| f.area * f.y * f.y).sum();
    let i_exact = (400.0_f64.powi(4) - 376.0_f64.powi(4)) / 12.0;
    assert_relative_eq!(i_sum, i_exact, max_relative = 0.02);

    assert!(sec.fibers.iter().all(|f| f.material == 2));
    assert!(sec
        .fibers
        .iter()
        .all(|f| f.y.abs() > 376.0 / 2.0 - 1e-9 || f.z.abs() > 376.0 / 2.0 - 1e-9));
}

/// RC 円形断面のファイバ配置が円形（極座標リング）で、コンクリート断面積が
/// π·d²/4 と一致し、主筋が材料区分 1 で分離配置されることを検証する。
#[test]
fn test_rc_circle_fibers_match_circle_area() {
    let rebar = squid_n_core::section_shape::RcRebar {
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
        FiberYield {
            main: Some(345.0),
            rebar: Some(345.0),
            steel: None,
        },
        1.0,
        1.0,
        StrengthParams {
            steel_fy: 235.0,
            rebar_fy: 345.0,
            concrete_fc: 24.0,
            steel_e: 22000.0,
        },
        HysteresisModel::Retrograde,
    )
    .expect("RC 円形断面の配筋は妥当");

    let conc_area: f64 = sec
        .fibers
        .iter()
        .filter(|f| f.material == 0)
        .map(|f| f.area)
        .sum();
    let circle = std::f64::consts::PI * 600.0_f64 * 600.0 / 4.0;
    assert_relative_eq!(conc_area, circle, max_relative = 1e-9);

    let n_rebar = sec.fibers.iter().filter(|f| f.material == 1).count();
    assert_eq!(n_rebar, 8);
}
