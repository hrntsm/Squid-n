//! 並列モード（`squid_n_math::parallelism`）の統合テスト。
//!
//! 並列度設定はプロセスグローバルのため、単一スレッド前提の決定性テスト
//! （unit テスト群）と干渉しないよう統合テスト（別プロセス）に分離している。
//! このファイル内のテストは同一プロセスで並行実行されるので、
//! すべて並列モードを前提とし、途中で `Deterministic` へ戻さない。

use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LoadCombination, LocalAxis,
    Material, MaterialCategory, Model, NodalLoad, Node, Section,
};
use squid_n_math::parallelism::{set_parallelism, Parallelism};
use squid_n_solver::statics::analysis::Analysis;

/// 複数の荷重ケースを持つ片持ち梁モデル（unit テストの
/// `make_cantilever_model` と同等。先端に方向違いの集中荷重を n ケース）。
fn make_model(n_cases: usize) -> Model {
    let load_cases: Vec<LoadCase> = (0..n_cases)
        .map(|i| {
            let mut values = [0.0; 6];
            // ケースごとに方向・大きさを変える（x/y/z の集中荷重）
            values[i % 3] = 100.0 * (i as f64 + 1.0);
            LoadCase {
                kind: Default::default(),
                id: LoadCaseId(i as u32 + 1),
                name: format!("case{}", i + 1),
                nodal: vec![NodalLoad::manual(NodeId(1), values)],
                member: Vec::new(),
            }
        })
        .collect();
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
        load_cases,
        combinations: vec![LoadCombination {
            name: "combo1".into(),
            terms: vec![(LoadCaseId(1), 1.2), (LoadCaseId(2), 1.5)],
        }],
        ..Default::default()
    }
}

/// 並列バッチの結果が、同じ並列設定での個別呼び出しと一致すること
/// （ケース間に共有可変状態がなく、実行順が結果に影響しないことの検証）。
#[test]
fn test_parallel_batch_matches_individual() {
    set_parallelism(Parallelism::Threads(2));
    let model = make_model(8);
    let analysis = Analysis::prepare(&model).unwrap();

    let lcs: Vec<LoadCaseId> = (1..=8).map(LoadCaseId).collect();
    let batch = analysis.linear_static_batch(&lcs);
    assert_eq!(batch.len(), 8);
    for (i, lc) in lcs.iter().enumerate() {
        let individual = analysis.linear_static(*lc).unwrap();
        let b = batch[i].as_ref().unwrap();
        assert_eq!(b.disp, individual.disp, "case {} disp mismatch", i + 1);
        assert_eq!(
            b.member_forces.len(),
            individual.member_forces.len(),
            "case {} member_forces length mismatch",
            i + 1
        );
    }
}

/// 並列モードでも解析値が物理的に正しいこと（軸引張の理論解と比較）。
#[test]
fn test_parallel_solve_value_correct() {
    set_parallelism(Parallelism::Auto);
    let model = make_model(3);
    let analysis = Analysis::prepare(&model).unwrap();
    let res = analysis.linear_static(LoadCaseId(1)).unwrap();
    // case1 は x 方向 100 の軸引張: u = PL/EA
    let expected = 100.0 * 1000.0 / (20000.0 * 100.0);
    let ux = res.disp[1][0];
    assert!(
        (ux - expected).abs() < 1e-9,
        "ux={} expected={}",
        ux,
        expected
    );
}

/// 並列組合せバッチが逐次の組合せ解と（値として）一致すること。
#[test]
fn test_parallel_combination_batch() {
    set_parallelism(Parallelism::Threads(2));
    let model = make_model(4);
    let analysis = Analysis::prepare(&model).unwrap();
    let combos = vec![
        model.combinations[0].clone(),
        LoadCombination {
            name: "combo2".into(),
            terms: vec![(LoadCaseId(3), 1.0), (LoadCaseId(4), -0.5)],
        },
    ];
    let batch = analysis.linear_combination_batch(&combos);
    for (i, combo) in combos.iter().enumerate() {
        let individual = analysis.linear_combination(combo).unwrap();
        assert_eq!(
            batch[i].as_ref().unwrap().disp,
            individual.disp,
            "combo {} mismatch",
            i
        );
    }
}
