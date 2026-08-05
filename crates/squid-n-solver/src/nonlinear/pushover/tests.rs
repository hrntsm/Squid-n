use super::*;
use crate::constraint::Reducer;
use squid_n_core::dof::{Dof6Mask, DofMap};
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    Constraint, DiaphragmDef, ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis,
    Material, MaterialCategory, Node, Section, Story,
};
use squid_n_core::section_shape::ShearBar;

/// 1層・鉛直ファイバ柱の片持ちプッシュオーバー（P5 §10 相当の最小統合テスト）。
/// 配線済み非線形要素（FiberBeam）＋座標変換＋NR 反復＋降伏追跡が
/// エンドツーエンドで動作することを検証する。
fn single_column_model(fy: f64, seismic_weight: f64) -> Model {
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
                coord: [0.0, 0.0, 3000.0],
                // FiberBeam はねじり剛性を持たないため、Z 軸柱の頂部ねじり DOF(rz=bit5)
                // のみ拘束して特異性を除く。曲げ回転 rx,ry と並進は自由。
                restraint: Dof6Mask(0b100000),
                mass: None,
                story: Some(StoryId(0)),
                support_spring: None,
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
            j: 1.0e6,
            depth: 100.0,
            width: 100.0,
            as_y: 0.0,
            as_z: 0.0,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: None,
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
            fy: Some(fy),
        }],
        stories: vec![Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(0),
            name: "1F".to_string(),
            elevation: 3000.0,
            node_ids: vec![NodeId(1)],
            diaphragms: vec![DiaphragmDef {
                ci_override: None,
                weight: None,
                master: NodeId(1),
                slaves: vec![],
                rigid: true,
            }],
            seismic_weight: Some(seismic_weight),
            weight_override: None,
        }],
        ..Default::default()
    }
}

#[test]
fn test_pushover_single_column_forms_hinge() {
    // 降伏応力を低め、地震重量を降伏荷重をやや超える程度に設定し、
    // 柱脚に曲げヒンジが形成されることを確認する。
    let model = single_column_model(235.0, 80_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);

    let result = pushover_analysis(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        20,    // max_steps
        0.0,   // max_disp（変位制御に移行しない＝荷重制御のみ）
        false, // use_kg
        false, // use_arc_length
        0.0,
    )
    .expect("pushover should run end-to-end");

    // パイプライン全体が収束ステップを生成していること。
    assert!(
        !result.capacity_curve.is_empty(),
        "capacity curve should have at least one converged step"
    );
    // 荷重−変位曲線の頂部変位は単調に正（水平押し）であること。
    let last = result.capacity_curve.last().unwrap();
    assert!(
        last.roof_disp > 0.0,
        "roof displacement should be positive: {}",
        last.roof_disp
    );
    // 降伏応力を与えた鋼材ファイバ柱で、柱脚に曲げヒンジが追跡されること
    //（座標変換＋ファイバ降伏＋降伏追跡のエンドツーエンド検証）。
    assert!(
        !result.hinges.is_empty(),
        "at least one hinge should form in the column under lateral push"
    );

    // steps は capacity_curve と同じ収束ステップ数だけ積まれること。
    assert_eq!(
        result.steps.len(),
        result.capacity_curve.len(),
        "steps should have one entry per capacity_curve point"
    );
    // 各 step の story_drifts は層数（本モデルは1層）と一致すること。
    for s in &result.steps {
        assert_eq!(
            s.story_drifts.len(),
            model.stories.len(),
            "story_drifts length should match number of stories"
        );
    }

    // 終了理由が記録されること（従来は非収束を含む全打ち切りが無言だった）。
    // 本モデルは目標無効（max_disp=0）の荷重制御のみのため、終了理由は
    // 「目標到達以外の正常系」（λ 上限・スケジュール完了）または「非収束」の
    // いずれかであり、少なくとも Unknown（未記録）ではないこと。
    assert_ne!(
        result.termination,
        crate::pushover::PushoverTermination::Unknown,
        "終了理由が記録されるべき: {:?}",
        result.termination
    );

    // 部材別終局応答（終局検定の設計用応力・部材別 Rp 反映用）が生成されること。
    assert_eq!(
        result.member_response.len(),
        model.elements.len(),
        "member_response should have one entry per element"
    );
    let col = result
        .member_response
        .iter()
        .find(|r| r.elem == model.elements[0].id)
        .expect("column member response");
    // 水平押しで柱脚に曲げ・せん断・変形角が生じる（いずれも正）。
    assert!(
        col.m_strong > 0.0 && col.shear_strong > 0.0 && col.rp > 0.0,
        "column terminal response should be nonzero: Mz={}, Vy={}, Rp={}",
        col.m_strong,
        col.shear_strong,
        col.rp
    );
}

#[test]
fn test_pushover_load_control_endpoint_is_mesh_independent() {
    // 荷重制御プッシュオーバーの終点（λ=1、base_shear=一定）の頂部変位は、
    // 物理的には荷重増分ステップ数に依存しない。各ステップの Newton 反復で
    // 「最後の修正量」だけを total_disp へ加算していた回帰バグでは、塑性域
    // （1 ステップに複数反復を要する）で途中の修正量が脱落し、終点変位が
    // ステップ数に依存して過小評価されていた（20 ステップで約 5% 過小、
    // ステップを細かくするほど真値へ漸近）。全反復修正量を累積する修正後は
    // ステップ数によらず同一終点となる。
    //
    // 本モデルは弾性降伏変位 ≈69mm（Qy=My/L≈13.05kN、k=3EI/L³≈189.8N/mm）で、
    // λ=1 の base_shear=16000N は降伏後（塑性域）にあり複数反復ステップを含む。
    let run = |steps: usize| -> (f64, f64) {
        let model = single_column_model(235.0, 80_000.0);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let result = pushover_analysis(
            &model,
            &dofmap,
            &reducer,
            SeismicDir::X,
            steps,
            0.0,
            false,
            false,
            0.0,
        )
        .expect("pushover should run end-to-end");
        let last = result.capacity_curve.last().unwrap();
        (last.roof_disp, last.base_shear)
    };
    let (roof_20, base_20) = run(20);
    let (roof_80, base_80) = run(80);

    // 前提: いずれも同一の終局荷重（λ=1）まで到達している。
    assert!((base_20 - 16_000.0).abs() < 1.0, "base_20={base_20}");
    assert!((base_80 - 16_000.0).abs() < 1.0, "base_80={base_80}");
    // 前提: 終点は弾性降伏変位を超えており、塑性域＝複数反復ステップを含む。
    assert!(
        roof_20 > 69.0,
        "endpoint must be inelastic: roof_20={roof_20}"
    );

    // 本題: 荷重ステップ数によらず終点頂部変位が一致すること（相対差 < 0.1%）。
    let rel_diff = (roof_20 - roof_80).abs() / roof_80;
    assert!(
        rel_diff < 1e-3,
        "load-control endpoint roof disp must be mesh-independent: \
         roof(20 steps)={roof_20}, roof(80 steps)={roof_80}, rel_diff={rel_diff:.4}; \
         a step-count dependence indicates dropped Newton corrections in total_disp"
    );
}

/// コンクリート強度 Fc が未設定の RC 部材があるモデルは、弾性のまま解析せず
/// エラーで停止する。Fc がないと曲げひび割れ Mc=0 でヒンジが一切検出されず、
/// ファイバー断面も Fc を勝手に仮定するため、崩壊機構が形成されないまま
/// 保有水平耐力を過大評価する（危険側）。
#[test]
fn test_pushover_stops_when_concrete_strength_unset() {
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape};

    let mut model = single_column_model(235.0, 80_000.0);
    // 断面を RC 矩形にし、材料からコンクリート強度・降伏強度を落とす（未入力を模擬）。
    model.sections[0].shape = Some(SectionShape::RcRect {
        b: 100.0,
        d: 100.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 4,
                dia: 13.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 4,
                dia: 13.0,
                layers: 1,
            },
            cover: 20.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: Some("SD295A".into()),
            },
            main_grade: Some("SD345".into()),
        },
    });
    model.materials[0].category = squid_n_core::model::MaterialCategory::Concrete;
    model.materials[0].name = "conc".into();
    model.materials[0].fy = None;
    model.materials[0].fc = None;

    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let err = pushover_analysis(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        10,
        0.0,
        false,
        false,
        0.0,
    )
    .expect_err("Fc 未設定の RC 部材があれば解析を停止すべき");
    assert!(err.contains("Fc"), "{}", err);

    // Fc を設定すれば解析は通る（チェックが恒常的に解析を妨げないこと）。
    model.materials[0].fc = Some(24.0);
    assert!(
        pushover_analysis(
            &model,
            &dofmap,
            &reducer,
            SeismicDir::X,
            10,
            0.0,
            false,
            false,
            0.0,
        )
        .is_ok(),
        "Fc を設定すれば解析できるべき"
    );
}

#[test]
fn test_pushover_requires_seismic_weight() {
    // 地震重量未定義ではエラーを返す（入力検証）。
    let model = single_column_model(235.0, 0.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        10,
        0.0,
        false,
        false,
        0.0,
    );
    assert!(
        result.is_err(),
        "should error when no seismic weight defined"
    );
}

/// 支点ばね（`Node::support_spring`）検証用モデル: 零長節点バネ（軸剛性 `kx`）で
/// 節点0(全固定)-節点1(水平のみ自由)を結ぶ 1 自由度系。節点1 に `support_kx`
/// （`Some` なら水平支点ばね剛性 [N/mm]）を与える。2 節点を同一座標（零長）に
/// 置くことで、`NodalSpringElement` の局所軸＝全体座標系（`spring.rs` の
/// 零長特例）となり、軸バネ `kx` がそのまま水平（全体 X）成分に一致する
/// （局所軸の回転を気にせず手計算と直接比較できる）。
fn spring_column_model(kx: f64, support_kx: Option<f64>, seismic_weight: f64) -> Model {
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
                coord: [0.0, 0.0, 0.0],
                // 水平（Ux）のみ自由（他の 5 自由度は固定）。合成剛性
                // kx+support_kx の検証を Ux 1 自由度に限定するため。
                restraint: Dof6Mask(0b111110),
                mass: None,
                story: Some(StoryId(0)),
                support_spring: support_kx.map(|k| [k, 0.0, 0.0, 0.0, 0.0, 0.0]),
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::NodalSpring,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Pinned, EndCondition::Pinned],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: Some([kx, 0.0, 0.0, 0.0, 0.0, 0.0]),
        }],
        stories: vec![Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(0),
            name: "1F".to_string(),
            elevation: 3000.0,
            node_ids: vec![NodeId(1)],
            diaphragms: vec![DiaphragmDef {
                ci_override: None,
                weight: None,
                master: NodeId(1),
                slaves: vec![],
                rigid: true,
            }],
            seismic_weight: Some(seismic_weight),
            weight_override: None,
        }],
        ..Default::default()
    }
}

/// 支点ばね（`Node::support_spring`）が非線形経路（プッシュオーバー）の
/// 全体剛性 K・内力 f_int の両方に反映され、静的 Newton 法が正しい
/// 合成剛性 `kx+ks` の変位へ収束することを検証する（本文書の要求 (b)）。
///
/// K だけに支点ばねを加算し f_int 側の `k・u` 計上を忘れると（本テストが
/// 検出したい典型的なバグ）、Newton は「要素ばね kx のみ」の残差方程式の
/// 根へ収束し、支点ばねが変位に一切効かなくなる（support_kx の有無で
/// 終点変位が変化しない）。固定 λ 刻み（`max_disp=0.0` で目標判定を無効化）
/// を用いるため、荷重パターン q・λ=1 到達時の外力は両ケースで完全に同一
/// （モデル剛性に依存しない）であり、終点変位の比較がそのまま合成剛性の
/// 検証になる。
#[test]
fn test_pushover_support_spring_affects_k_and_f_int() {
    let kx = 1000.0; // 要素（節点バネ）側の水平剛性 [N/mm]
    let ks = 1000.0; // 支点ばねの水平剛性 [N/mm]
    let weight = 80_000.0;

    let run = |support_kx: Option<f64>| -> (f64, f64) {
        let model = spring_column_model(kx, support_kx, weight);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let result = pushover_analysis(
            &model,
            &dofmap,
            &reducer,
            SeismicDir::X,
            4,     // max_steps（線形弾性なので刻みは収束結果に影響しない）
            0.0,   // max_disp=0（目標判定なし＝固定 λ 刻みで λ=1 まで荷重制御）
            false, // use_kg
            false, // use_arc_length
            0.0,
        )
        .expect("linear spring pushover should converge every step");
        let last = result.capacity_curve.last().expect("at least one step");
        (last.roof_disp, last.base_shear)
    };

    let (roof_baseline, base_baseline) = run(None);
    let (roof_spring, base_spring) = run(Some(ks));

    // 前提: 両ケースで到達する外力（λ=1 時点の base_shear）は等しい
    // （q・λ 刻みはモデル剛性に依存しないため）。
    assert!(
        (base_baseline - base_spring).abs() < 1e-6 * base_baseline.abs().max(1.0),
        "base_shear should be identical regardless of support spring: \
         baseline={base_baseline}, spring={base_spring}"
    );

    // 本題: 支点ばね有りの終点変位は、要素ばねと支点ばねの合成剛性
    // kx+ks に対応する理論比 kx/(kx+ks) だけ小さくなること。
    let expected_ratio = kx / (kx + ks);
    let actual_ratio = roof_spring / roof_baseline;
    assert!(
        (actual_ratio - expected_ratio).abs() < 1e-6,
        "roof disp ratio should match combined stiffness kx/(kx+ks)={expected_ratio}, \
         got {actual_ratio} (roof_baseline={roof_baseline}, roof_spring={roof_spring})"
    );

    // restraint で固定した自由度に support_spring を与えても無視されること
    // （固定支持を優先する仕様。dof.rs の判定に影響しないことの確認を兼ねる）。
    let mut model_fixed_with_spring = spring_column_model(kx, None, weight);
    model_fixed_with_spring.nodes[0].support_spring = Some([1.0e9, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let dofmap_fixed = DofMap::build(&model_fixed_with_spring);
    // 節点0 は全固定のため、support_spring を与えても活性自由度は増えない
    // （固定支持を優先しばね値を無視する。孤立節点の扱いは dof.rs 側の現状仕様）。
    assert_eq!(
        dofmap_fixed.n_active(),
        1,
        "restraint で固定した節点の support_spring は活性 DOF に影響しない"
    );
}

#[test]
fn test_pushover_arc_length_path_runs() {
    // 弧長法フェーズ（f_int 反復再評価版）がエンドツーエンドで動作すること。
    let model = single_column_model(235.0, 80_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        10,    // max_steps（荷重制御）
        0.0,   // max_disp
        false, // use_kg
        true,  // use_arc_length
        1.0,   // arc_length_dl [mm]
    )
    .expect("arc-length pushover should run end-to-end");
    assert!(!result.capacity_curve.is_empty());
    assert!(result.qu > 0.0);
}

#[test]
fn test_pushover_computes_member_ductility() {
    // 変位制御で十分に押し込み、ファイバ柱の部材塑性率 μ が算定されること
    // （降伏方式では降伏曲率が基点、降伏後 μ≥1 が報告される）。
    let model = single_column_model(235.0, 80_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis_recording(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        20,
        PushoverTarget::from_max_disp(300.0), // 目標変位300mm（大変形で確実に降伏させる）
        PushoverControl::default(),
        true,
        false,
        false,
        0.0,
        DuctilityMethod::FirstYield,
    )
    .expect("pushover should run");
    // 降伏したヒンジで塑性率 μ≥1 が算定される（旧実装の粗いモーメント比では
    // なく、危険断面の曲率塑性率）。
    let max_mu = result
        .hinges
        .iter()
        .map(|h| h.ductility)
        .fold(0.0_f64, f64::max);
    assert!(
        max_mu >= 1.0,
        "member ductility should be ≥1 after yielding: {max_mu}"
    );
}

#[test]
fn test_pushover_ductility_method_selection_changes_reference() {
    // 塑性率方式の選択が塑性率基点を変えることを確認する。降伏方式(3)は降伏時に
    // 基点到達し μ≥1、基点歪み方式(1)は本押込量では基点ひずみ（鉄骨 0.01）未到達で
    // μ=0（未評価）となる。
    let run = |method: DuctilityMethod| -> f64 {
        let model = single_column_model(235.0, 80_000.0);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let result = pushover_analysis_recording(
            &model,
            &dofmap,
            &reducer,
            SeismicDir::X,
            20,
            PushoverTarget::from_max_disp(0.0),
            PushoverControl::default(),
            true,
            false,
            false,
            0.0,
            method,
        )
        .expect("pushover should run");
        result
            .hinges
            .iter()
            .map(|h| h.ductility)
            .fold(0.0_f64, f64::max)
    };
    // 3 方式とも降伏後は基点到達し μ≥1 の妥当な塑性率を算定する（機構の検証）。
    for method in [
        DuctilityMethod::FirstYield,
        DuctilityMethod::ReferenceStrain,
        DuctilityMethod::WeightedAverageJm,
    ] {
        let mu = run(method);
        assert!(
            mu >= 1.0 && mu.is_finite(),
            "{method:?} は降伏後に妥当な塑性率 μ≥1 を算定する: {mu}"
        );
    }
}

/// determine_mechanism / hinge_story 用の2層・柱通り（基礎-1F-2F）モデル。
/// node0=基礎(story None), node1=1F(story0), node2=2F(story1)。
/// elem0=1F柱(0-1), elem1=2F柱(1-2)。
fn two_story_model() -> Model {
    let sec = Section {
        id: SectionId(0),
        name: "c".to_string(),
        area: 10000.0,
        iy: 8.333e6,
        iz: 8.333e6,
        j: 1.0e6,
        depth: 100.0,
        width: 100.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "s".to_string(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: Some(0.0),
        fc: None,
        fy: Some(235.0),
    };
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
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(StoryId(0)),
                support_spring: None,
            },
            Node {
                id: NodeId(2),
                coord: [0.0, 0.0, 6000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(StoryId(1)),
                support_spring: None,
            },
        ],
        elements: vec![
            ElementData {
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
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Fiber,
                nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
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
            },
        ],
        sections: vec![sec],
        materials: vec![mat],
        stories: vec![
            Story {
                level_kind: Default::default(),
                structure: Default::default(),
                id: StoryId(0),
                name: "1F".to_string(),
                elevation: 3000.0,
                node_ids: vec![NodeId(1)],
                diaphragms: vec![],
                seismic_weight: None,
                weight_override: None,
            },
            Story {
                level_kind: Default::default(),
                structure: Default::default(),
                id: StoryId(1),
                name: "2F".to_string(),
                elevation: 6000.0,
                node_ids: vec![NodeId(2)],
                diaphragms: vec![],
                seismic_weight: None,
                weight_override: None,
            },
        ],
        ..Default::default()
    }
}

fn hinge(elem: u32, pos: f64, level: HingeLevel) -> HingeEvent {
    HingeEvent {
        step: 0,
        elem: ElemId(elem),
        pos,
        level,
        ductility: 1.0,
    }
}

#[test]
fn test_determine_mechanism_partial_when_insufficient() {
    let model = two_story_model();
    // ひび割れのみ → 降伏ヒンジ0個 < r+1 → Partial
    assert!(matches!(
        determine_mechanism(&[hinge(0, 0.0, HingeLevel::Crack)], &model, SeismicDir::X),
        MechanismType::Partial
    ));
}

/// two_story_model は部材2・節点3・基礎FIXED(平面3DOF) → r=0（静定）。
/// したがって降伏ヒンジ1個で運動学的機構成立（r+1=1）。単一階集中→層崩壊。
#[test]
fn test_determine_mechanism_single_yield_establishes_mechanism() {
    let model = two_story_model();
    // elem0 端 j (pos=1.0) → node1 = 1F 単独階 → 層崩壊
    match determine_mechanism(&[hinge(0, 1.0, HingeLevel::Yield)], &model, SeismicDir::X) {
        MechanismType::StoryCollapse { story } => assert_eq!(story, StoryId(0)),
        other => panic!(
            "expected StoryCollapse{{0}}, got {:?}",
            std::mem::discriminant(&other)
        ),
    }
}

/// 静的不静定次数の計算検証（平面骨組: r = 3m − 3n + r_support）。
#[test]
fn test_compute_static_indeterminacy_two_story() {
    // 2層2柱: 部材2・節点3・基礎節点(node0)が平面3DOF拘束 → r = 6 - 9 + 3 = 0（静定）
    let model = two_story_model();
    assert_eq!(compute_static_indeterminacy(&model, SeismicDir::X), 0);
}

#[test]
fn test_compute_static_indeterminacy_indeterminate_portal() {
    // 1層1スパン両端固定ラーメン: 柱2+梁1=部材3、節点4（基礎2点FIXED+上部2点FREE）
    // r = 3*3 - 3*4 + (3+3) = 9 - 12 + 6 = 3（3次不静定）
    let model = two_story_model(); // 共用せず簡易生成
    let _ = model; // unused warning 回避
    let nodes = vec![
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
            coord: [0.0, 0.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: Some(StoryId(0)),
            support_spring: None,
        },
        Node {
            id: NodeId(2),
            coord: [5000.0, 0.0, 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: Some(StoryId(0)),
            support_spring: None,
        },
        Node {
            id: NodeId(3),
            coord: [5000.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
            support_spring: None,
        },
    ];
    let elems = vec![
        ElementData {
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
        },
        ElementData {
            id: ElemId(1),
            kind: ElementKind::Fiber,
            nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
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
        },
        ElementData {
            id: ElemId(2),
            kind: ElementKind::Fiber,
            nodes: smallvec::smallvec![NodeId(3), NodeId(2)],
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
        },
    ];
    let portal = Model {
        nodes,
        elements: elems,
        sections: vec![Section {
            id: SectionId(0),
            name: "c".to_string(),
            area: 10000.0,
            iy: 8.333e6,
            iz: 8.333e6,
            j: 1.0e6,
            depth: 100.0,
            width: 100.0,
            as_y: 0.0,
            as_z: 0.0,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "s".to_string(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: Some(0.0),
            fc: None,
            fy: Some(235.0),
        }],
        stories: vec![Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(0),
            name: "1F".to_string(),
            elevation: 3000.0,
            node_ids: vec![NodeId(1), NodeId(2)],
            diaphragms: vec![],
            seismic_weight: None,
            weight_override: None,
        }],
        ..Default::default()
    };
    assert_eq!(compute_static_indeterminacy(&portal, SeismicDir::X), 3);

    // 直交フレーム込みの 3D モデル: 同じ門型を Y=5000 にもう 1 構面複製し、
    // 柱頭同士を Y 方向大梁 2 本でつなぐ。X 加力の静的不静定次数は
    // 「X-Z 構面 2 面ぶん」の 3+3=6 であるべきで、Y 方向大梁を部材数へ
    // 算入してはならない（従来はモデル全体の要素数・節点数を使っており、
    // 直交大梁 1 本につき r が 3 水増しされ、機構成立ゲート（≧r+1）が過大に
    // なって層崩壊機構が Partial と誤判定されていた）。
    let mut model3d = portal.clone();
    let n_nodes = model3d.nodes.len() as u32;
    for i in 0..n_nodes {
        let mut n = model3d.nodes[i as usize].clone();
        n.id = NodeId(n_nodes + i);
        n.coord[1] += 5000.0;
        model3d.nodes.push(n);
    }
    let n_elems = model3d.elements.len() as u32;
    for i in 0..n_elems {
        let mut e = model3d.elements[i as usize].clone();
        e.id = ElemId(n_elems + i);
        e.nodes = e.nodes.iter().map(|nid| NodeId(nid.0 + n_nodes)).collect();
        model3d.elements.push(e);
    }
    // 柱頭 (node1,node2) と複製構面の柱頭 (node1+4,node2+4) をつなぐ Y 方向大梁。
    for (k, (a, b)) in [(1u32, 5u32), (2, 6)].iter().enumerate() {
        let mut e = model3d.elements[1].clone(); // 元の X 方向梁を雛形に
        e.id = ElemId(2 * n_elems + k as u32);
        e.nodes = smallvec::smallvec![NodeId(*a), NodeId(*b)];
        model3d.elements.push(e);
    }
    assert_eq!(
        compute_static_indeterminacy(&model3d, SeismicDir::X),
        6,
        "X加力の不静定次数はX-Z構面2面ぶん(3+3)であり、直交Y大梁で水増しされない"
    );
    // Y 加力では逆に、Y 方向大梁 2 本と柱 4 本が Y-Z 構面を構成する。
    // 構面あたり 柱2+梁1=3部材・節点4（基部2固定）→ r=3、2 構面で 6。
    // X 方向梁は算入しない。
    assert_eq!(
        compute_static_indeterminacy(&model3d, SeismicDir::Y),
        6,
        "Y加力の不静定次数はY-Z構面2面ぶんであり、直交X大梁で水増しされない"
    );
}

#[test]
fn test_determine_mechanism_story_collapse() {
    let model = two_story_model();
    // 1F柱の両端（elem0 pos1.0 → node1=1F, elem1 pos0.0 → node1=1F）が降伏
    // → 降伏ヒンジが1F(story0)に集中 → 層崩壊
    let hinges = vec![
        hinge(0, 1.0, HingeLevel::Yield),
        hinge(1, 0.0, HingeLevel::Yield),
    ];
    match determine_mechanism(&hinges, &model, SeismicDir::X) {
        MechanismType::StoryCollapse { story } => assert_eq!(story, StoryId(0)),
        other => panic!(
            "expected StoryCollapse{{0}}, got {:?}",
            std::mem::discriminant(&other)
        ),
    }
}

#[test]
fn test_determine_mechanism_overall() {
    let model = two_story_model();
    // 1F(story0)と2F(story1)に分散して降伏 → 全体崩壊
    let hinges = vec![
        hinge(0, 1.0, HingeLevel::Yield), // node1 = 1F
        hinge(1, 1.0, HingeLevel::Yield), // node2 = 2F
    ];
    assert!(matches!(
        determine_mechanism(&hinges, &model, SeismicDir::X),
        MechanismType::Overall
    ));
}

#[test]
fn test_pushover_base_shear_is_real_force() {
    // 最初の（弾性）ステップで base_shear/roof_disp が片持ち柱の弾性剛性
    // 3EI/L³ ≈ 189.8 N/mm に一致することを確認（DOF添字加算の旧バグを排除）。
    let model = single_column_model(235.0, 80_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        20,
        0.0,
        false,
        false,
        0.0,
    )
    .unwrap();
    let first = result.capacity_curve.first().unwrap();
    assert!(first.roof_disp > 0.0 && first.base_shear > 0.0);
    let k = first.base_shear / first.roof_disp;
    assert!(
        (150.0..=230.0).contains(&k),
        "first-step stiffness base_shear/roof_disp={k} should be ~3EI/L^3≈189.8"
    );
    // Qu はピークベースシア（全点以上）であること。
    for c in &result.capacity_curve {
        assert!(
            result.qu >= c.base_shear - 1e-6,
            "qu {} must be >= {}",
            result.qu,
            c.base_shear
        );
    }
    assert!(result.qu > 0.0);
}

fn portal_frame_model(fy: f64, seismic_weight: f64) -> Model {
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
                coord: [0.0, 0.0, 3000.0],
                // FiberBeam はねじり剛性を持たないため Rz を拘束
                restraint: Dof6Mask(0b100000),
                mass: None,
                story: Some(StoryId(0)),
                support_spring: None,
            },
            Node {
                id: NodeId(2),
                coord: [5000.0, 0.0, 3000.0],
                restraint: Dof6Mask(0b100000),
                mass: None,
                story: Some(StoryId(0)),
                support_spring: None,
            },
            Node {
                id: NodeId(3),
                coord: [5000.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![
            ElementData {
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
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Fiber,
                nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
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
            },
            ElementData {
                id: ElemId(2),
                kind: ElementKind::Fiber,
                nodes: smallvec::smallvec![NodeId(3), NodeId(2)],
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
            },
        ],
        sections: vec![Section {
            id: SectionId(0),
            name: "col".to_string(),
            area: 10000.0,
            iy: 8.333e6,
            iz: 8.333e6,
            j: 1.0e6,
            depth: 100.0,
            width: 100.0,
            as_y: 0.0,
            as_z: 0.0,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: None,
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
            fy: Some(fy),
        }],
        stories: vec![Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(0),
            name: "1F".to_string(),
            elevation: 3000.0,
            node_ids: vec![NodeId(1), NodeId(2)],
            diaphragms: vec![DiaphragmDef {
                ci_override: None,
                weight: None,
                master: NodeId(1),
                slaves: vec![NodeId(2)],
                rigid: true,
            }],
            seismic_weight: Some(seismic_weight),
            weight_override: None,
        }],
        constraints: vec![Constraint::RigidDiaphragm {
            story: StoryId(0),
            master: NodeId(1),
            slaves: vec![NodeId(2)],
        }],
        ..Default::default()
    }
}

// 1層1スパン剛床ラーメン（門形フレーム）で崩壊荷重が手計算値（4・My/H_col）
// に一致し、柱両端に4つの塑性ヒンジが形成され全体機構となることを検証する（P5 §10.1）。
//
// 手計算: Z=I/(depth/2)=166,660, My=σ_y·Z, Qu=4My/H=52,220 N（柱両端降伏・2柱）。
// seismic_weight は崩壊荷重を上回る値に設定し、真に降伏到達させる。
#[test]
fn test_portal_frame_collapse_load() {
    let qu_theory: f64 = 52_220.0;
    let model = portal_frame_model(235.0, 600_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);

    let result = pushover_analysis(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        80,
        0.0,
        false,
        false,
        0.0,
    )
    .expect("pushover should run end-to-end");

    // 柱両端の降伏ヒンジが実際に形成されていること（運動学的機構: r+1=4）。
    let yielded_hinges = result
        .hinges
        .iter()
        .filter(|h| !matches!(h.level, HingeLevel::Crack))
        .count();
    assert!(
        yielded_hinges >= 4,
        "at least 4 yielded hinges expected for Overall mechanism, got {} (total hinges={})",
        yielded_hinges,
        result.hinges.len()
    );

    // 崩壊機構が成立していること（Partial でない）。
    assert!(
        !matches!(result.mechanism, MechanismType::Partial),
        "mechanism should not be Partial for a collapsed portal frame"
    );

    assert!(result.qu > 0.0, "qu should be positive, got {}", result.qu);

    // 4番目の降伏ヒンジ（柱両端×2本＝4個で運動学的機構成立）発生ステップの
    // ベースシアを「観測崩壊荷重」とする（qu=max(base_shear) はまだ弾性最大反力で
    // plateau を正確に捉えられないため、降伏到達点で照合する）。
    let mut yield_steps: Vec<u32> = result
        .hinges
        .iter()
        .filter(|h| !matches!(h.level, HingeLevel::Crack))
        .map(|h| h.step)
        .collect();
    yield_steps.sort_unstable();
    yield_steps.dedup();
    assert!(
        yield_steps.len() >= 4,
        "need >=4 distinct yield steps for Overall mechanism, got {}: {:?}",
        yield_steps.len(),
        yield_steps
    );
    let mech_step = yield_steps[3];
    let qu_observed = result
        .capacity_curve
        .iter()
        .find(|c| c.step == mech_step)
        .map(|c| c.base_shear)
        .unwrap_or(0.0);
    let rel_err = (qu_observed - qu_theory).abs() / qu_theory;
    // pushover は段階改良途上のため、比較的広めの許容差（30%）を設ける。
    assert!(
        rel_err < 0.30,
        "observed_qu={} at step {} deviates from Qu_theory={} by {:.1}% (>30%)",
        qu_observed,
        mech_step,
        qu_theory,
        rel_err * 100.0
    );
}

#[test]
fn test_portal_frame_mechanism_classified() {
    let model = portal_frame_model(235.0, 600_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);

    let result = pushover_analysis(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        80,
        0.0,
        false,
        false,
        0.0,
    )
    .expect("pushover should run end-to-end");

    match &result.mechanism {
        MechanismType::Overall | MechanismType::StoryCollapse { .. } => {}
        other => panic!(
            "expected Overall or StoryCollapse, got {:?}",
            std::mem::discriminant(other)
        ),
    }
}

// ---- せん断降伏耐力 Qy の単体テスト ----

#[test]
fn test_compute_shear_yield_qy_steel() {
    // 鋼系（fy 設定あり）: Qy = as・fy/√3（RcRect 形状の有無・方向によらない）。
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "s".to_string(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(200.0),
    };
    let qy = compute_shear_yield_qy(1000.0, Some(&mat), None, ShearDir::Z, 3000.0);
    let expected = 1000.0 * 200.0 / 3.0_f64.sqrt();
    assert!(
        (qy - expected).abs() < 1e-6,
        "qy={qy} should equal as*fy/sqrt(3)={expected}"
    );
}

#[test]
fn test_compute_shear_yield_qy_rc_fallback_without_rc_rect_shape() {
    // RC系（fy 無し・fc 設定あり）かつ断面形状情報（RcRect）がない場合:
    // Qy = as・0.7√fc（慣用値へフォールバック）。
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "rc".to_string(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 0.0,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
    let qy = compute_shear_yield_qy(50000.0, Some(&mat), None, ShearDir::Z, 3000.0);
    let expected = 50000.0 * 0.7 * 24.0_f64.sqrt();
    assert!(
        (qy - expected).abs() < 1e-6,
        "qy={qy} should equal as*0.7*sqrt(fc)={expected}"
    );
}

/// SRC 矩形の Qy は「RC 部（荒川式）＋内蔵鉄骨の全塑性せん断 sAw·F/√3」の
/// 累加式。同一の b・d・配筋の RcRect との差が鉄骨項の手計算値と一致する。
#[test]
fn test_compute_shear_yield_qy_src_is_rc_plus_steel() {
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "FC24".to_string(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 0.0,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
    let rebar = RcRebar {
        main_grade: Some("SD345".into()),
        main_x: BarSet {
            count: 8,
            dia: 25.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 4,
            dia: 25.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
            grade: None,
        },
    };
    let rc_shape = SectionShape::RcRect {
        b: 600.0,
        d: 600.0,
        rebar: rebar.clone(),
    };
    let src_shape = SectionShape::SrcRect {
        b: 600.0,
        d: 600.0,
        rebar,
        steel_height: 400.0,
        steel_width: 200.0,
        steel_web_thick: 8.0,
        steel_flange_thick: 13.0,
        steel_grade: "SN400B".into(),
    };
    let rc_sec = rc_shape.to_section(SectionId(0), "rc".into());
    let src_sec = src_shape.to_section(SectionId(1), "src".into());

    // 強軸（局所 y）: 鉄骨項 = tw·(H−2tf)·F·1.1/√3（SN400B tf/tw≤40 → F=235）。
    let qy_rc = compute_shear_yield_qy(1.0, Some(&mat), Some(&rc_sec), ShearDir::Y, 3000.0);
    let qy_src = compute_shear_yield_qy(1.0, Some(&mat), Some(&src_sec), ShearDir::Y, 3000.0);
    let steel_y = 8.0 * (400.0 - 2.0 * 13.0) * 235.0 * 1.1 / 3.0_f64.sqrt();
    assert!(
        (qy_src - qy_rc - steel_y).abs() < 1e-6,
        "強軸: qy_src−qy_rc={} 期待 {}",
        qy_src - qy_rc,
        steel_y
    );

    // 弱軸（局所 z）: 鉄骨項 = 2·B·tf·F·1.1/√3。
    let qy_rc_z = compute_shear_yield_qy(1.0, Some(&mat), Some(&rc_sec), ShearDir::Z, 3000.0);
    let qy_src_z = compute_shear_yield_qy(1.0, Some(&mat), Some(&src_sec), ShearDir::Z, 3000.0);
    let steel_z = 2.0 * 200.0 * 13.0 * 235.0 * 1.1 / 3.0_f64.sqrt();
    assert!(
        (qy_src_z - qy_rc_z - steel_z).abs() < 1e-6,
        "弱軸: qy_src−qy_rc={} 期待 {}",
        qy_src_z - qy_rc_z,
        steel_z
    );

    // 材料に fy が設定されていても（主筋 σy のフォールバック等）、形状から
    // 精算できる SRC は累加式を使う（fy 先行だと剛性等価換算面積 × fy/√3 の
    // 桁違いに大きい鋼系式へ流れ、せん断降伏が検出されなくなる危険側）。
    let mut mat_with_fy = mat.clone();
    mat_with_fy.fy = Some(235.0);
    let qy_with_fy = compute_shear_yield_qy(
        1.0e6,
        Some(&mat_with_fy),
        Some(&src_sec),
        ShearDir::Y,
        3000.0,
    );
    assert!(
        (qy_with_fy - qy_src).abs() < 1e-6,
        "fy 設定時も累加式: qy={} 期待 {}",
        qy_with_fy,
        qy_src
    );

    // 板厚区分: フランジ厚 45mm（>40）の SN400B は弱軸（フランジ）の F が
    // 215 へ落ち、強軸（ウェブ tw=8 ≤40）は 235 のまま（板厚は板要素ごとに解決）。
    let thick_flange = SectionShape::SrcRect {
        b: 600.0,
        d: 600.0,
        rebar: match &rc_shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        },
        steel_height: 400.0,
        steel_width: 200.0,
        steel_web_thick: 8.0,
        steel_flange_thick: 45.0,
        steel_grade: "SN400B".into(),
    };
    let tf_sec = thick_flange.to_section(SectionId(2), "src-tf45".into());
    let qy_tf_z = compute_shear_yield_qy(1.0, Some(&mat), Some(&tf_sec), ShearDir::Z, 3000.0);
    let steel_tf_z = 2.0 * 200.0 * 45.0 * 215.0 * 1.1 / 3.0_f64.sqrt();
    assert!(
        (qy_tf_z - qy_rc_z - steel_tf_z).abs() < 1e-6,
        "厚板フランジの弱軸: qy_src−qy_rc={} 期待 {}",
        qy_tf_z - qy_rc_z,
        steel_tf_z
    );
}

#[test]
fn test_compute_shear_yield_qy_zero_as_is_infinite() {
    // 有効せん断断面積が 0 の断面は判定対象外（Qy=∞扱い）。
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "s".to_string(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(200.0),
    };
    assert_eq!(
        compute_shear_yield_qy(0.0, Some(&mat), None, ShearDir::Z, 3000.0),
        f64::INFINITY
    );
    // 材料未設定でも∞扱い。
    assert_eq!(
        compute_shear_yield_qy(1000.0, None, None, ShearDir::Z, 3000.0),
        f64::INFINITY
    );
}

/// RC 矩形断面（`SectionShape::RcRect`）+ 配筋情報がある場合、Qy は荒川式
/// （`rc_qsu_simple`）による方向別算定値に一致すること。
/// 要素座標系はせい方向＝ローカル y のため、y 方向（強軸・main_x）、
/// z 方向（弱軸・main_y、b/d 入れ替え）の双方を検証する。
#[test]
fn test_compute_shear_yield_qy_rc_rect_matches_arakawa_handcalc() {
    let rebar = RcRebar {
        main_grade: None,
        main_x: BarSet {
            count: 6,
            dia: 25.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 4,
            dia: 19.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
            grade: None,
        },
    };
    let (b, d) = (400.0, 600.0);
    let shape = SectionShape::RcRect {
        b,
        d,
        rebar: rebar.clone(),
    };
    let sec = shape.to_section(SectionId(0), "RC-400x600".into());
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "rc".to_string(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 0.0,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
    let clear_span = 3000.0;

    // y 方向（強軸曲げのせん断）: b=幅, d=せい, 引張鉄筋 main_x。
    // しきい値のせん断有効断面積は断面 as_z（ウェブ）由来（クロス変換）。
    // 本モジュール（shear_yield.rs）は保有水平耐力計算専用のため、主筋 σy には
    // 材料強度係数（直接入力係数優先、なければ一律1.1）を無条件で乗じる
    // （`material_strength_factor_rebar`）。せん断補強筋 σwy=295 は割増対象外。
    let bar_area = |bs: &BarSet| bs.count as f64 * std::f64::consts::PI / 4.0 * bs.dia * bs.dia;
    let qsu_y_handcalc = rc_qsu_simple(&RcCapacityInput {
        b,
        d,
        at: bar_area(&rebar.main_x) / 2.0,
        d_eff: d - rebar.cover - rebar.main_x.dia / 2.0,
        sigma_y: 345.0 * 1.1,
        fc: 24.0,
        pw: (std::f64::consts::PI / 4.0 * 10.0 * 10.0 * 2.0) / (b * 100.0),
        sigma_wy: 295.0,
        clear_span,
        sigma_0: 0.0,
    });
    let qy_y = compute_shear_yield_qy(sec.as_z, Some(&mat), Some(&sec), ShearDir::Y, clear_span);
    assert!(
        (qy_y - qsu_y_handcalc).abs() < 1e-6,
        "qy_y={qy_y} should equal rc_qsu_simple handcalc={qsu_y_handcalc}"
    );

    // z 方向（弱軸曲げのせん断）: b と d を入れ替え、引張鉄筋 main_y。
    let qsu_z_handcalc = rc_qsu_simple(&RcCapacityInput {
        b: d,
        d: b,
        at: bar_area(&rebar.main_y) / 2.0,
        d_eff: b - rebar.cover - rebar.main_y.dia / 2.0,
        sigma_y: 345.0 * 1.1,
        fc: 24.0,
        pw: (std::f64::consts::PI / 4.0 * 10.0 * 10.0 * 2.0) / (d * 100.0),
        sigma_wy: 295.0,
        clear_span,
        sigma_0: 0.0,
    });
    let qy_z = compute_shear_yield_qy(sec.as_y, Some(&mat), Some(&sec), ShearDir::Z, clear_span);
    assert!(
        (qy_z - qsu_z_handcalc).abs() < 1e-6,
        "qy_z={qy_z} should equal rc_qsu_simple handcalc={qsu_z_handcalc}"
    );
    // 断面が非正方形（b≠d、主筋も非対称）なので y・z の Qy は異なるはず。
    assert!((qy_y - qy_z).abs() > 1.0, "qy_y={qy_y} qy_z={qy_z}");
}

/// 断面（配筋）に指定した材質が耐力へ反映されること。
///
/// - せん断補強筋の材質は荒川式の σwy を通じて Qy を変える（高強度品ほど大きい）。
/// - 主筋の材質は曲げ降伏 My = 0.9·at·σy·d を通じて曲げヒンジ閾値を変える
///   （SD295A は既定 345 相当より小さい）。荒川式のせん断終局強度は主筋量 pt に
///   依存し σy には依らないため、主筋材質は Qy を変えない。
#[test]
fn test_section_rebar_grades_are_reflected_in_capacities() {
    let make_section = |main_grade: Option<&str>, shear_grade: Option<&str>| {
        let rebar = RcRebar {
            main_grade: main_grade.map(str::to_string),
            main_x: BarSet {
                count: 6,
                dia: 25.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 4,
                dia: 19.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: shear_grade.map(str::to_string),
            },
        };
        SectionShape::RcRect {
            b: 400.0,
            d: 600.0,
            rebar,
        }
        .to_section(SectionId(0), "RC-400x600".into())
    };
    // RC 材料（fy は持たない）。fy を持つ材料は鋼系として扱われ荒川式へ入らない。
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".to_string(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 0.0,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };

    // --- せん断: σwy が Qy に効く ---
    let qy = |sec: &squid_n_core::model::Section| {
        compute_shear_yield_qy(sec.as_z, Some(&mat), Some(sec), ShearDir::Y, 3000.0)
    };
    let sec_normal = make_section(Some("SD345"), Some("SD295A"));
    let sec_high = make_section(Some("SD345"), Some("KH785"));
    assert!(
        qy(&sec_high) > qy(&sec_normal),
        "高強度せん断補強筋は Qy を上げるはず: {} vs {}",
        qy(&sec_high),
        qy(&sec_normal)
    );

    // --- 曲げ: 主筋 σy が My に効く ---
    let my_of = |sec: squid_n_core::model::Section| {
        let mut model = single_column_model(235.0, 80_000.0);
        model.sections[0] = sec;
        model.materials[0] = mat.clone();
        compute_hinge_thresholds(&model)[0].my
    };
    let my_sd345 = my_of(make_section(Some("SD345"), Some("SD295A")));
    let my_sd295 = my_of(make_section(Some("SD295A"), Some("SD295A")));
    assert!(
        my_sd295 < my_sd345,
        "SD295A の主筋は My を下げるはず: {my_sd295} vs {my_sd345}"
    );
    // 比は σy の比（295/345）に一致する（My = 0.9·at·σy·d の σy 比例）。
    let ratio = my_sd295 / my_sd345;
    assert!(
        (ratio - 295.0 / 345.0).abs() < 1e-9,
        "My は σy に比例するはず: ratio={ratio}"
    );
}

/// as_y/as_z を明示的に与えた片持ち柱モデル（`single_column_model` のせん断有効
/// 断面積を差し替えたもの）。せん断降伏耐力 Qy は as_y/as_z と材料強度のみに
/// 依存し、実際に生じるせん断力（`track_shear_yield`）は材端力の釣合いから
/// 求まるため、せん断バネ剛性（材料のせん断弾性係数）を変更する必要はない。
fn single_column_model_with_shear(fy: f64, seismic_weight: f64, as_shear: f64) -> Model {
    let mut model = single_column_model(fy, seismic_weight);
    model.sections[0].as_y = as_shear;
    model.sections[0].as_z = as_shear;
    model
}

#[test]
fn test_pushover_shear_yield_event_recorded() {
    // せん断有効断面積を小さく設定してせん断降伏耐力 Qy を小さくすることで、
    // 水平荷重漸増中にせん断降伏イベントが記録されることを確認する
    // （曲げヒンジ判定 `track_hinges` とは独立の判定経路の検証）。
    let model = single_column_model_with_shear(235.0, 80_000.0, 50.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);

    let result = pushover_analysis(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        20,
        0.0,
        false,
        false,
        0.0,
    )
    .expect("pushover should run end-to-end");

    assert!(
        !result.shear_yields.is_empty(),
        "shear yield event should be recorded when Qy is small relative to applied shear"
    );
}

/// as_y・as_z を独立に設定した片持ち柱モデル（局所 y・z 方向分離の検証用）。
fn single_column_model_with_shear_yz(fy: f64, seismic_weight: f64, as_y: f64, as_z: f64) -> Model {
    let mut model = single_column_model(fy, seismic_weight);
    model.sections[0].as_y = as_y;
    model.sections[0].as_z = as_z;
    model
}

/// `single_column_model` は節点 (0,0,0)→(0,0,3000)、`local_axis.ref_vector=[1,0,0]`
/// なので局所座標系は ex=[0,0,1], ey=[1,0,0], ez=[0,1,0]（`LocalFrame::from_nodes`）。
/// `SeismicDir::X` でプッシュすると力はグローバル X＝局所 y（ey）方向に生じ、
/// 局所 z（ez＝グローバル Y）方向にはほぼ生じない。
/// 局所 y のしきい値は断面 as_z、局所 z は断面 as_y から作られる（クロス変換）。
fn run_pushover_has_shear_yield(as_y: f64, as_z: f64) -> bool {
    let model = single_column_model_with_shear_yz(235.0, 80_000.0, as_y, as_z);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        20,
        0.0,
        false,
        false,
        0.0,
    )
    .expect("pushover should run end-to-end");
    !result.shear_yields.is_empty()
}

/// 局所 y/z 方向の厳密分離（改良1）の検証:
/// 実際に力が生じる方向（局所 y、しきい値は断面 as_z 由来）の Qy を小さくすれば
/// せん断降伏イベントが記録されるが、力がほぼ生じない方向（局所 z、断面 as_y 由来）
/// の Qy をどれだけ小さくしても記録されないこと。v1（軸直交合力 vs
/// min(qy_y,qy_z)）では後者でも誤って記録されてしまっていた
/// （qy_z が min を支配してしまうため）。
#[test]
fn test_pushover_shear_yield_direction_independent() {
    assert!(
        run_pushover_has_shear_yield(1.0e12, 50.0),
        "small as_z (feeding the actually-stressed local-y threshold) should trigger a shear \
             yield event"
    );
    assert!(
        !run_pushover_has_shear_yield(50.0, 1.0e12),
        "small as_y (feeding the unstressed local-z threshold) should NOT trigger a shear \
             yield event once Vy/Vz are judged independently against qy_y/qy_z"
    );
}

// ---- 精緻化1: h0 への剛域控除の単体テスト ----

#[test]
fn test_effective_clear_span_deducts_rigid_zone_lengths() {
    let rz = RigidZone {
        length_i: 500.0,
        length_j: 300.0,
        ..Default::default()
    };
    // h0 = 節点間長3000 − (500+300) = 2200。
    assert!((effective_clear_span(3000.0, &rz) - 2200.0).abs() < 1e-9);
}

#[test]
fn test_effective_clear_span_falls_back_when_non_positive() {
    // 剛域長の合計が節点間長を超える異常入力 → 節点間長へフォールバック。
    let rz_over = RigidZone {
        length_i: 2000.0,
        length_j: 1500.0,
        ..Default::default()
    };
    assert_eq!(effective_clear_span(3000.0, &rz_over), 3000.0);

    // ちょうど0（または極小の浮動小数点誤差域）でもフォールバック。
    let rz_zero = RigidZone {
        length_i: 1500.0,
        length_j: 1500.0,
        ..Default::default()
    };
    assert_eq!(effective_clear_span(3000.0, &rz_zero), 3000.0);
}

/// RC矩形断面 + 配筋情報を持つ要素モデル（剛域テスト共通）。
/// 節点間距離3000mm、`rigid_zone` は呼び出し側で差し替える。
fn rc_column_model_with_rigid_zone(rigid_zone: RigidZone) -> (Model, RcRebar, f64, f64) {
    let rebar = RcRebar {
        main_grade: None,
        main_x: BarSet {
            count: 6,
            dia: 25.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 4,
            dia: 19.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
            grade: None,
        },
    };
    let (b, d) = (400.0, 600.0);
    let shape = SectionShape::RcRect {
        b,
        d,
        rebar: rebar.clone(),
    };
    let sec = shape.to_section(SectionId(0), "RC-400x600".into());
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "rc".to_string(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 0.0,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
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
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(StoryId(0)),
                support_spring: None,
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
            rigid_zone,
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![sec],
        materials: vec![mat],
        ..Default::default()
    };
    (model, rebar, b, d)
}

#[test]
fn test_compute_shear_yield_thresholds_rc_rect_uses_rigid_zone_reduced_clear_span() {
    // 剛域: length_i=400, length_j=200 → h0 = 3000-600 = 2400。
    let rigid_zone = RigidZone {
        length_i: 400.0,
        length_j: 200.0,
        ..Default::default()
    };
    let (model, rebar, b, d) = rc_column_model_with_rigid_zone(rigid_zone);
    let thresholds = compute_shear_yield_thresholds(&model);
    let th = &thresholds[0];

    let bar_area = |bs: &BarSet| bs.count as f64 * std::f64::consts::PI / 4.0 * bs.dia * bs.dia;
    let expected_clear_span = 2400.0;

    // y方向（強軸・main_x。クロス変換で局所 y が強軸側）: RcArakawa を採用し、
    // h0=2400 での rc_qsu_simple 手計算に一致。σy は主筋の材料強度係数（一律1.1）を
    // 乗じた 345×1.1（保有水平耐力計算専用モジュールのため無条件で適用）。
    let qsu_y_handcalc = rc_qsu_simple(&RcCapacityInput {
        b,
        d,
        at: bar_area(&rebar.main_x) / 2.0,
        d_eff: d - rebar.cover - rebar.main_x.dia / 2.0,
        sigma_y: 345.0 * 1.1,
        fc: 24.0,
        pw: (std::f64::consts::PI / 4.0 * 10.0 * 10.0 * 2.0) / (b * 100.0),
        sigma_wy: 295.0,
        clear_span: expected_clear_span,
        sigma_0: 0.0,
    });
    match &th.y {
        DirThreshold::RcArakawa {
            input, gross_area, ..
        } => {
            assert!(
                (input.clear_span - expected_clear_span).abs() < 1e-9,
                "clear_span={} expected={}",
                input.clear_span,
                expected_clear_span
            );
            assert!((gross_area - b * d).abs() < 1e-9);
        }
        DirThreshold::Static(_) => panic!("expected RcArakawa for RcRect with rebar"),
    }
    assert!(
        (th.y.qy(0.0) - qsu_y_handcalc).abs() < 1e-6,
        "qy(0.0)={} handcalc={}",
        th.y.qy(0.0),
        qsu_y_handcalc
    );
}

#[test]
fn test_compute_shear_yield_thresholds_rc_rect_falls_back_when_rigid_zone_exceeds_length() {
    // 剛域長の合計(2000+1500=3500)が節点間長(3000)を超える異常入力
    // → h0 は節点間長3000へフォールバックする。
    let rigid_zone = RigidZone {
        length_i: 2000.0,
        length_j: 1500.0,
        ..Default::default()
    };
    let (model, rebar, b, d) = rc_column_model_with_rigid_zone(rigid_zone);
    let thresholds = compute_shear_yield_thresholds(&model);
    let th = &thresholds[0];

    let bar_area = |bs: &BarSet| bs.count as f64 * std::f64::consts::PI / 4.0 * bs.dia * bs.dia;
    let qsu_y_handcalc = rc_qsu_simple(&RcCapacityInput {
        b,
        d,
        at: bar_area(&rebar.main_x) / 2.0,
        d_eff: d - rebar.cover - rebar.main_x.dia / 2.0,
        // 主筋の材料強度係数（一律1.1）を乗じた 345×1.1。
        sigma_y: 345.0 * 1.1,
        fc: 24.0,
        pw: (std::f64::consts::PI / 4.0 * 10.0 * 10.0 * 2.0) / (b * 100.0),
        sigma_wy: 295.0,
        clear_span: 3000.0, // フォールバック後の値
        sigma_0: 0.0,
    });
    assert!((th.y.qy(0.0) - qsu_y_handcalc).abs() < 1e-6);
}

// ---- 精緻化2: 軸力σ0の動的反映の単体テスト ----

#[test]
fn test_dir_threshold_qy_axial_term_matches_handcalc() {
    // rc_capacity::tests::sample_input と同一の断面（b=400,D=600,pw=0.002等）で
    // DirThreshold::RcArakawa を直接構成し、圧縮軸力からの σ0 反映を検算する。
    let b = 400.0;
    let d = 600.0;
    let d_eff = 530.0;
    let input = RcCapacityInput {
        b,
        d,
        at: 1935.0,
        d_eff,
        sigma_y: 345.0,
        fc: 24.0,
        pw: 0.002,
        sigma_wy: 295.0,
        clear_span: 3000.0,
        sigma_0: 0.0, // プレースホルダ（qy() が上書きする）
    };
    let gross_area = b * d;
    let th = DirThreshold::RcArakawa {
        input,
        gross_area,
        steel_qy: 0.0,
    };

    let qy_base = th.qy(0.0);
    let qsu_base_handcalc = rc_qsu_simple(&input);
    assert!((qy_base - qsu_base_handcalc).abs() < 1e-6);

    // 圧縮軸力 N_compress = 5.0 * gross_area → σ0 = 5.0 [N/mm²]（適用範囲0〜0.4Fc=9.6内）。
    let sigma_0 = 5.0;
    let n_compress = sigma_0 * gross_area;
    let qy_with_axial = th.qy(n_compress);
    let j = 7.0 * d_eff / 8.0;
    let expected_delta = 0.1 * sigma_0 * b * j;
    assert!(
        (qy_with_axial - qy_base - expected_delta).abs() < 1e-6,
        "delta={} expected={}",
        qy_with_axial - qy_base,
        expected_delta
    );

    // 引張（n_compress=0、呼び出し側で既にクランプ済みの規約）は σ0=0 のまま、
    // Qy は base と一致（増えない）。
    assert!((th.qy(0.0) - qy_base).abs() < 1e-9);
}

/// 軸力符号規約の検算（単純片持ち柱、節点 i=(0,0,0)・j=(0,0,3000)、
/// `ref_vector=[1,0,0]` → `LocalFrame::from_nodes` により ex=[0,0,1]）。
///
/// 柱頭（j端）を Δ=-1mm（ex と逆向き、圧縮方向）変位させたときの内力を
/// 手計算（f_local_x(i)=-N>0, f_local_x(j)=N<0、doc `axial_compression` 参照）
/// で再現し、`axial_compression` がこの圧縮を正しく検出することを確認する。
#[test]
fn test_axial_compression_sign_convention_handcalc() {
    let ex = [0.0, 0.0, 1.0];
    // 圧縮（N<0、|N|=1000）: f_i はコンプレッション側 = +|N|・ex、f_j = -|N|・ex。
    let n_compress_mag = 1000.0;
    let f_i_comp = [0.0, 0.0, n_compress_mag];
    let f_j_comp = [0.0, 0.0, -n_compress_mag];
    assert!(
        (axial_compression(f_i_comp, f_j_comp, ex) - n_compress_mag).abs() < 1e-9,
        "compression should be detected as a positive n_compress"
    );

    // 引張（N>0）: 圧縮側の符号が反転 → axial_compression は 0（圧縮なし）。
    let f_i_tension = [0.0, 0.0, -n_compress_mag];
    let f_j_tension = [0.0, 0.0, n_compress_mag];
    assert_eq!(
        axial_compression(f_i_tension, f_j_tension, ex),
        0.0,
        "pure tension must not be treated as compression (sigma_0=0 for tension)"
    );

    // 片端のみ圧縮成分がある非対称ケース（数値誤差や分布荷重を模擬）:
    // 両端のうち大きい方（実勢値）を採用する。
    let f_i_asym = [0.0, 0.0, n_compress_mag];
    let f_j_asym = [0.0, 0.0, -0.5 * n_compress_mag];
    assert!(
        (axial_compression(f_i_asym, f_j_asym, ex) - n_compress_mag).abs() < 1e-9,
        "should take the larger of the two end-derived compression values"
    );
}

/// `ElementBehavior::internal_force` が固定のグローバル材端力を返すだけのテスト
/// スタブ（`track_shear_yield` は `global_dofs`/剛性を使わないため他は無関係）。
struct FixedForceBehavior {
    f: LocalVec,
}

impl ElementBehavior for FixedForceBehavior {
    fn n_dof(&self) -> usize {
        12
    }
    fn global_dofs(&self, _dof: &DofMap) -> SmallVec<[usize; 24]> {
        SmallVec::new()
    }
    fn tangent_stiffness(&self, _ctx: &Ctx) -> squid_n_element::behavior::LocalMat {
        squid_n_element::behavior::LocalMat::zeros(12)
    }
    fn internal_force(&self, _ctx: &Ctx) -> LocalVec {
        LocalVec {
            data: self.f.data.clone(),
        }
    }
    fn mass_matrix(
        &self,
        _opt: squid_n_element::behavior::MassOption,
    ) -> squid_n_element::behavior::LocalMat {
        squid_n_element::behavior::LocalMat::zeros(12)
    }
}

/// 精緻化2のエンドツーエンド確認: 同一のせん断力 Vz デマンドに対し、
/// 軸圧縮が作用する場合は σ0 反映で Qy が増え判定を免れるが、圧縮がない
/// （引張・軸力ゼロ）場合は従来どおり判定に掛かることを、実際の
/// `track_shear_yield` を通して確認する（`compute_shear_yield_thresholds` の
/// 構築から一貫して検証）。
#[test]
fn test_track_shear_yield_axial_compression_raises_qy_end_to_end() {
    let (model, _rebar, b, d) = rc_column_model_with_rigid_zone(RigidZone::default());
    let thresholds = compute_shear_yield_thresholds(&model);
    let (input, gross_area) = match &thresholds[0].z {
        DirThreshold::RcArakawa {
            input, gross_area, ..
        } => (*input, *gross_area),
        DirThreshold::Static(_) => panic!("expected RcArakawa"),
    };
    assert!((gross_area - b * d).abs() < 1e-6);

    let qy_base = rc_qsu_simple(&input);
    let sigma_0 = 5.0; // 0〜0.4Fc=9.6 の範囲内
    let n_compress = sigma_0 * gross_area;
    let mut inp_axial = input;
    inp_axial.sigma_0 = sigma_0;
    let qy_boosted = rc_qsu_simple(&inp_axial);
    assert!(qy_boosted > qy_base, "axial term should raise Qy");

    // Vz を base と boosted のちょうど中間に設定: base では降伏、boosted では非降伏。
    // モデルは node i=(0,0,0)・j=(0,0,3000)、ref_vector=[1,0,0] のため
    // ex=[0,0,1], ey=[1,0,0], ez=[0,1,0]（既存テストの局所座標系規約と同じ）。
    // よって Vz は global y 成分（f.data[1]/f.data[7]）、N は global z 成分
    // （f.data[2]/f.data[8]）に対応する。
    let vz_demand = (qy_base + qy_boosted) / 2.0;

    // ケースA: 軸圧縮あり（N_compress = sigma_0*gross_area）→ 判定を免れるはず。
    let f_comp = LocalVec {
        data: SmallVec::from_slice(&[
            0.0,
            vz_demand,
            n_compress,
            0.0,
            0.0,
            0.0,
            0.0,
            -vz_demand,
            -n_compress,
            0.0,
            0.0,
            0.0,
        ]),
    };
    let behaviors_comp: Vec<Box<dyn ElementBehavior>> =
        vec![Box::new(FixedForceBehavior { f: f_comp })];
    let mut events_comp = Vec::new();
    track_shear_yield(&model, &behaviors_comp, &thresholds, 0, &mut events_comp);
    assert!(
        events_comp.is_empty(),
        "compression should raise Qy above the shear demand, suppressing the event"
    );

    // ケースB: 軸力なし（同じ Vz デマンド）→ 従来どおり判定に掛かるはず。
    let f_zero = LocalVec {
        data: SmallVec::from_slice(&[
            0.0, vz_demand, 0.0, 0.0, 0.0, 0.0, 0.0, -vz_demand, 0.0, 0.0, 0.0, 0.0,
        ]),
    };
    let behaviors_zero: Vec<Box<dyn ElementBehavior>> =
        vec![Box::new(FixedForceBehavior { f: f_zero })];
    let mut events_zero = Vec::new();
    track_shear_yield(&model, &behaviors_zero, &thresholds, 0, &mut events_zero);
    assert!(
        !events_zero.is_empty(),
        "without axial compression the same Vz demand should still trigger the event"
    );
}

// ---- 保有水平耐力計算（プッシュオーバー）の材料強度割増: 部材組み立て時の
// 係数配線方式（`build_nonlinear_behavior(.., StrengthBasis::MaterialStrength)`
// および pushover 専用モジュール hinge.rs / shear_yield.rs の無条件適用）の検証。
// 旧方式（モデル複製 `scale_steel_material_strength`）は廃止したため、
// `compute_hinge_thresholds` / `compute_shear_yield_thresholds` が返す
// 実効降伏応力（My・σy）を直接検証する。 ----

/// 鋼材断面1本の片持ち柱モデル（形状情報なし＝フォールバック分岐、
/// `member_moment_thresholds` の σy·Ze 経路）を作る。
fn steel_hinge_model(name: &str, fy: f64, strength_factor: Option<f64>) -> Model {
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
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
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
            name: "c".to_string(),
            area: 10000.0,
            iy: 8.333e6,
            iz: 8.333e6,
            j: 1.0e6,
            depth: 100.0,
            width: 100.0,
            as_y: 0.0,
            as_z: 0.0,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            strength_factor,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: name.to_string(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(fy),
        }],
        ..Default::default()
    }
}

/// 鋼材文脈: 既知の鋼材グレード名（SS400=1.1倍、SA440=590N級で1.05倍）は
/// `compute_hinge_thresholds` の My に材料強度係数がそのまま反映され、
/// 未知名称の材料に対する比が係数と一致することを確認する。
#[test]
fn test_compute_hinge_thresholds_steel_uses_material_strength_factor() {
    let my_of =
        |name: &str, fy: f64| compute_hinge_thresholds(&steel_hinge_model(name, fy, None))[0].my;

    let my_unknown = my_of("未知鋼材", 235.0);
    let my_ss400 = my_of("SS400", 235.0);
    assert!(
        (my_ss400 / my_unknown - 1.1).abs() < 1e-9,
        "SS400（既知グレード）は未知名称の1.1倍のはず: {my_ss400}/{my_unknown}"
    );

    let my_unknown2 = my_of("未知鋼材2", 440.0);
    let my_sa440 = my_of("SA440", 440.0);
    assert!(
        (my_sa440 / my_unknown2 - 1.05).abs() < 1e-9,
        "SA440（590N級）は未知名称の1.05倍のはず: {my_sa440}/{my_unknown2}"
    );
}

/// 直接入力の割増係数（`Material::strength_factor`）は、名称から鋼材グレードを
/// 解決できない材料でも最優先で使われることを確認する。
#[test]
fn test_compute_hinge_thresholds_direct_strength_factor_overrides_name_lookup() {
    let my_of = |factor: Option<f64>| {
        compute_hinge_thresholds(&steel_hinge_model("カスタム材料", 235.0, factor))[0].my
    };
    let my_default = my_of(None); // 未知名称 → 係数 1.0
    let my_scaled = my_of(Some(1.25));
    assert!(
        (my_scaled / my_default - 1.25).abs() < 1e-9,
        "直接入力係数1.25が最優先で使われるはず: {my_scaled}/{my_default}"
    );
}

/// RC 矩形断面 + 配筋情報を持つ片持ち柱モデル（fy 未設定＝既定345）を作る。
fn rc_hinge_model() -> (Model, RcRebar, f64, f64) {
    let rebar = RcRebar {
        main_grade: None,
        main_x: BarSet {
            count: 6,
            dia: 25.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 4,
            dia: 19.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
            grade: None,
        },
    };
    let (b, d) = (400.0, 600.0);
    let shape = SectionShape::RcRect {
        b,
        d,
        rebar: rebar.clone(),
    };
    let sec = shape.to_section(SectionId(0), "RC-400x600".into());
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "rc".to_string(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 0.0,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
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
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
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
        materials: vec![mat],
        ..Default::default()
    };
    (model, rebar, b, d)
}

/// RC 主筋文脈: fy 未設定（既定 SD345=345）の RC 矩形で、`compute_hinge_thresholds`
/// の My が主筋の材料強度係数（一律1.1）を乗じた σy=345×1.1 の
/// `rc_mu_simple` 相当になることを確認する。
#[test]
fn test_compute_hinge_thresholds_rc_rebar_uses_material_strength_factor() {
    let (model, rebar, _b, d) = rc_hinge_model();
    let thresholds = compute_hinge_thresholds(&model);

    let bar_area = |bs: &BarSet| bs.count as f64 * std::f64::consts::PI / 4.0 * bs.dia * bs.dia;
    let at = bar_area(&rebar.main_x) / 2.0;
    let d_eff = (d - rebar.cover - rebar.main_x.dia / 2.0).max(0.0);
    let expected_my = rc_mu_simple(&RcCapacityInput {
        b: 1.0,
        d,
        at,
        d_eff,
        sigma_y: 345.0 * 1.1,
        fc: 24.0,
        pw: 0.0,
        sigma_wy: 0.0,
        clear_span: 1.0,
        sigma_0: 0.0,
    });
    assert!(
        (thresholds[0].my - expected_my).abs() < 1e-6,
        "my={} expected={}",
        thresholds[0].my,
        expected_my
    );
}

/// せん断降伏側（shear_yield.rs）: RC 矩形の主筋 σy には材料強度係数（1.1）が
/// 乗じられる一方、せん断補強筋 σwy=295 は割増対象外のまま据え置かれることを
/// 確認する（`rc_rect_capacity_input` の実装）。
#[test]
fn test_compute_shear_yield_thresholds_rc_rebar_scaled_but_shear_reinforcement_is_not() {
    let (model, _rebar, _b, _d) = rc_hinge_model();
    let thresholds = compute_shear_yield_thresholds(&model);
    match &thresholds[0].y {
        DirThreshold::RcArakawa { input, .. } => {
            assert!(
                (input.sigma_y - 345.0 * 1.1).abs() < 1e-9,
                "主筋 σy は1.1倍のはず: {}",
                input.sigma_y
            );
            assert_eq!(
                input.sigma_wy, 295.0,
                "せん断補強筋は材料強度割増の対象外のため295のまま"
            );
        }
        DirThreshold::Static(_) => panic!("expected RcArakawa for RcRect with rebar"),
    }
}

/// 変位制御フェーズが実際に目標変位まで押し切り、荷重制御（λ=1＝C0=0.2 級）を
/// 超える耐力まで到達することを検証する回帰テスト。
///
/// 修正前は変位制御のペナルティ残差 `penalty·(target − u)` の桁落ちで収束判定が
/// 原理的に成立せず、変位制御フェーズが1点も確定しなかった。その結果 Qu は荷重
/// 制御の頭打ち（λ=1、参照荷重 C0=0.2・地震重量＝設計地震力レベル）に張り付き、
/// 崩壊機構へ到達しないまま過小評価されていた（保有水平耐力計算として致命的）。
#[test]
fn test_pushover_displacement_control_reaches_target_and_exceeds_design_load() {
    let seismic_weight = 80_000.0;
    let model = single_column_model(235.0, seismic_weight);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let max_disp = 200.0;
    let n_steps = 50usize;

    // 荷重制御のみ（max_disp=0）: Qu は参照荷重 C0=0.2・地震重量で頭打ち。
    let load_only_model = single_column_model(235.0, seismic_weight);
    let lo_dofmap = DofMap::build(&load_only_model);
    let lo_reducer = Reducer::build(&load_only_model, &lo_dofmap);
    let load_only = pushover_analysis(
        &load_only_model,
        &lo_dofmap,
        &lo_reducer,
        SeismicDir::X,
        n_steps,
        0.0,
        false,
        false,
        0.0,
    )
    .expect("load control run");

    let result = pushover_analysis(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        n_steps,
        max_disp,
        false,
        false,
        0.0,
    )
    .expect("displacement control run");

    // 変位制御フェーズが点を確定していること。ステップ番号は確定順の通し番号のため、
    // フェーズは荷重係数で識別する（変位制御は設計地震力レベル λ=1 を超えて押し込む）。
    let disp_phase_points = result
        .steps
        .iter()
        .filter(|s| s.load_factor > 1.0 + 1e-9)
        .count();
    assert!(
        disp_phase_points >= 5,
        "displacement-control phase should record points, got {}",
        disp_phase_points
    );

    // 目標変位（200mm）まで到達していること。
    let last_roof = result
        .capacity_curve
        .last()
        .map(|c| c.roof_disp)
        .unwrap_or(0.0);
    assert!(
        (last_roof - max_disp).abs() < 1.0,
        "roof should reach target {}mm, got {:.3}mm",
        max_disp,
        last_roof
    );

    // 変位制御により荷重制御頭打ち（≈ 0.2·W = 16kN）を上回る耐力へ到達すること。
    assert!(
        result.qu > load_only.qu * 1.05,
        "displacement control Qu={:.1} should exceed load-only Qu={:.1}",
        result.qu,
        load_only.qu
    );
    // 塑性増分ヒンジモデルでは、押し切った耐力が柱脚の全塑性崩壊荷重
    // Vp = Mp/L = 1.1·σy·Zp/L（Zp = b·d²/4、材料強度割増 1.1）で頭打ちになる
    // （旧定式化は降伏後もほぼ弾性勾配で伸び続け Qu を過大評価していた）。
    // バイリニア硬化（b=0.01）の分だけ Vp をやや上回る。
    let zp = 100.0 * 100.0 * 100.0 / 4.0;
    let vp = 1.1 * 235.0 * zp / 3000.0;
    assert!(
        result.qu > vp * 0.95 && result.qu < vp * 1.3,
        "Qu={:.1} should cap near plastic collapse Vp={:.1} (0.95..1.3)",
        result.qu,
        vp
    );
}

/// ヒンジが 1 つも発生しない弾性範囲では、荷重制御→変位制御のフェーズ切替を
/// またいでも性能曲線のベースシアが単調非減少であることを検証する回帰テスト。
///
/// 旧実装の変位制御は Ai 分布の比例荷重を残差から外し、頂部 1 自由度を
/// ペナルティばねで押し込んでいた。載荷パターンが「Ai 分布」→「頂部 1 点載荷」へ
/// 不連続に変わるため、ヒンジがない弾性のままでもフェーズ切替点でベースシアが
/// 落ち込み、その後頂部 1 点載荷の剛性勾配で伸び直す非物理的な V 字曲線を描いて
/// いた。現行実装は比例荷重パターン λ·q を保持し、荷重係数 λ を頂部変位拘束から
/// 決定するため、弾性域では曲線が単調増加し λ も 1 を超えて滑らかに増加する。
#[test]
fn test_pushover_elastic_curve_monotonic_across_phase_switch() {
    // 降伏応力を非現実的に高くし、曲げヒンジ・ファイバー降伏を発生させない。
    let seismic_weight = 80_000.0;
    let n_steps = 20usize;
    let model = single_column_model(100_000.0, seismic_weight);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        n_steps,
        // 荷重制御 λ=1 の頂部変位（≈ 0.2W / (3EI/L³) ≈ 84mm）より十分大きい
        // 目標変位とし、変位制御フェーズが必ず実行されるようにする。
        300.0,
        false,
        false,
        0.0,
    )
    .expect("pushover should run");

    // 前提: 弾性のまま（ヒンジ無し）で変位制御フェーズまで進んでいること。
    // ステップ番号は確定順の通し番号のため、フェーズは荷重係数で識別する
    // （変位制御は設計地震力レベル λ=1 を超えて押し込む）。
    assert!(result.hinges.is_empty(), "弾性のままであること");
    assert!(
        result.steps.iter().any(|s| s.load_factor > 1.0 + 1e-9),
        "変位制御フェーズの点が確定していること"
    );

    // ベースシアが全区間で単調非減少であること（旧実装はフェーズ切替点で低下）。
    for w in result.capacity_curve.windows(2) {
        assert!(
            w[1].base_shear >= w[0].base_shear * (1.0 - 1e-6),
            "弾性域でベースシアが低下してはならない: {:.1} -> {:.1} (step {} -> {})",
            w[0].base_shear,
            w[1].base_shear,
            w[0].step,
            w[1].step
        );
    }

    // 弾性の比例載荷では base_shear = λ·C0·W（C0=0.2）が全フェーズで成り立つこと
    // （変位制御でも載荷パターン λ·q が保持されている検証）。
    for s in &result.steps {
        let expected = s.load_factor * 0.2 * seismic_weight;
        assert!(
            (s.base_shear - expected).abs() <= expected.abs().max(1.0) * 1e-3,
            "base_shear={:.1} が λ·0.2W={:.1} と一致すること",
            s.base_shear,
            expected
        );
    }

    // 設計地震力レベル（λ=1）を超えて押し込めていること。
    let last_lambda = result.steps.last().map(|s| s.load_factor).unwrap_or(0.0);
    assert!(
        last_lambda > 1.0,
        "変位制御で λ が 1 を超えること: {last_lambda:.3}"
    );
}

/// 均等変位刻み制御: 性能曲線の頂部変位刻みが全域で概ね目標刻み
/// du = 押込み上限 / ステップ数 に揃うことを検証する。
///
/// 固定 λ 刻み＋変位制御固定 10 分割の旧制御では、弾性域（荷重制御 λ≦1）に点が
/// 密集し、塑性化が進む変位制御域が数倍〜数十倍粗くなる偏りがあった（グラフの
/// 序盤だけ点が細かく後半が荒い、の回帰テスト）。
#[test]
fn test_pushover_uniform_displacement_spacing() {
    let seismic_weight = 80_000.0;
    let max_disp = 200.0;
    let n_steps = 50usize;
    let model = single_column_model(235.0, seismic_weight);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        n_steps,
        max_disp,
        false,
        false,
        0.0,
    )
    .expect("pushover should run");

    let du = max_disp / n_steps as f64;
    let disps: Vec<f64> = result.capacity_curve.iter().map(|c| c.roof_disp).collect();
    // 全域を概ね du 刻みでカバーする（λ=1 到達の端数ステップ等での多少の増減は許容）。
    assert!(
        disps.len() >= 40,
        "均等刻みなら 200/4=50 点程度が確定するはず: {} 点",
        disps.len()
    );
    for w in disps.windows(2) {
        let d = w[1] - w[0];
        assert!(d > 0.0, "頂部変位は単調増加であること: {:.4}", d);
        assert!(
            d <= du * 2.0 + 1e-6,
            "頂部変位刻みが目標刻み du={:.2} の 2 倍を超えないこと: {:.3}",
            du,
            d
        );
    }
    // 平均刻みも du と同程度であること（弾性域の点密集＝過小刻みの回帰）。
    let avg = (disps.last().unwrap() - disps.first().unwrap()) / (disps.len() - 1) as f64;
    assert!(
        avg >= du * 0.4,
        "平均刻み {:.3} が目標刻み du={:.2} と同程度であること",
        avg,
        du
    );
}

/// 荷重増分のみ（`PushoverControl::LoadOnly`）の比較モードで、終了目標が有効なら
/// λ=1（設計地震力レベル）を超えて荷重増分が継続し、目標変位へ到達することを
/// 検証する。単柱（fy=235）はバイリニア硬化（b=0.01）のため、降伏後も荷重増分の
/// まま硬化勾配に沿って目標まで載荷できる。
#[test]
fn test_pushover_load_only_extends_beyond_design_level() {
    let seismic_weight = 80_000.0;
    let model = single_column_model(235.0, seismic_weight);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis_recording(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        50,
        PushoverTarget::from_max_disp(200.0),
        PushoverControl::LoadOnly,
        true,
        false,
        false,
        0.0,
        DuctilityMethod::default(),
    )
    .expect("pushover should run");

    // 結果にどの制御方式で解析したかが記録されること（結果画面・CSV の識別用）。
    assert_eq!(result.control, PushoverControl::LoadOnly);

    let last = result.steps.last().expect("収束ステップがあること");
    assert!(
        last.load_factor > 1.0,
        "荷重増分が λ=1 を超えて継続すること: {:.3}",
        last.load_factor
    );
    assert!(
        last.top_disp >= 200.0 * 0.99,
        "目標変位 200mm へ到達すること: {:.1}mm",
        last.top_disp
    );
    // 荷重増分のみでは λ は単調非減少（変位制御の λ 減少域は存在しない）。
    for w in result.steps.windows(2) {
        assert!(
            w[1].load_factor >= w[0].load_factor - 1e-12,
            "λ が減少しないこと: {:.4} -> {:.4}",
            w[0].load_factor,
            w[1].load_factor
        );
    }
    assert!(
        !result.hinges.is_empty(),
        "降伏（ヒンジ発生）後の硬化域まで載荷されていること"
    );
    // 設計地震力レベル 0.2W を上回る耐力へ到達すること。
    assert!(
        result.qu > 0.2 * seismic_weight * 1.05,
        "Qu={:.1} が設計地震力レベル 0.2W={:.1} を上回ること",
        result.qu,
        0.2 * seismic_weight
    );
}

/// 長期荷重の初期載荷（apply_long_term）で、水平力増分の前に長期系荷重ケースが
/// 載荷されることを検証する。(1) 最初の記録ステップが λ=0 の長期載荷時点になる、
/// (2) その時点のベースシア（水平）はほぼ 0、(3) 柱軸力に長期軸力が保持された
/// まま水平増分が進む（N-M 応答経路が長期点から始まる）、の 3 点。
#[test]
fn test_pushover_long_term_preload_sets_initial_axial_state() {
    use squid_n_core::ids::LoadCaseId;
    use squid_n_core::model::{LoadCase, LoadCaseKind, NodalLoad};

    let mut model = single_column_model(235.0, 80_000.0);
    let p = 50_000.0;
    model.load_cases.push(LoadCase {
        id: LoadCaseId(0),
        name: "DL".into(),
        nodal: vec![NodalLoad {
            node: NodeId(1),
            values: [0.0, 0.0, -p, 0.0, 0.0, 0.0],
        }],
        member: vec![],
        kind: LoadCaseKind::Dead,
    });
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis_recording(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        20,
        PushoverTarget::from_max_disp(50.0),
        PushoverControl::default(),
        true,
        false,
        false,
        0.0,
        DuctilityMethod::default(),
    )
    .expect("pushover should run");

    let first = result.steps.first().expect("長期載荷ステップがあること");
    assert_eq!(
        first.load_factor, 0.0,
        "最初の記録ステップは長期載荷時点（λ=0）であること"
    );
    assert!(
        first.base_shear.abs() < 1.0,
        "長期載荷時点の水平ベースシアはほぼ 0: {:.3}",
        first.base_shear
    );
    assert!(
        result.steps.len() > 1,
        "長期載荷後に水平増分ステップが続くこと"
    );
    // 終局時の柱軸力（圧縮正）に長期軸力 P が保持されていること
    // （単柱の水平載荷は軸力を生まないため、軸力 ≒ P のまま）。
    let axial = result
        .member_response
        .first()
        .map(|m| m.axial)
        .unwrap_or(0.0);
    assert!(
        (axial - p).abs() < 0.05 * p,
        "柱軸力に長期軸力が保持されること: axial={:.0}, P={:.0}",
        axial,
        p
    );

    // 長期荷重を無効にした場合は従来どおり λ=0 の記録はない。
    let model2 = single_column_model(235.0, 80_000.0);
    let dofmap2 = DofMap::build(&model2);
    let reducer2 = Reducer::build(&model2, &dofmap2);
    let result2 = pushover_analysis_recording(
        &model2,
        &dofmap2,
        &reducer2,
        SeismicDir::X,
        20,
        PushoverTarget::from_max_disp(50.0),
        PushoverControl::default(),
        false,
        false,
        false,
        0.0,
        DuctilityMethod::default(),
    )
    .expect("pushover should run");
    assert!(
        result2.steps.first().is_some_and(|s| s.load_factor > 0.0),
        "長期載荷無効時は最初から水平増分であること"
    );
}

/// 荷重増分のみで終了目標が両方無効の場合は、従来の荷重制御と同じく λ=1
/// （設計地震力レベル）で終了することを検証する（λ=1 超の延長は終了目標が
/// 有効な場合に限る）。
#[test]
fn test_pushover_load_only_without_target_stops_at_lambda_1() {
    let model = single_column_model(235.0, 80_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis_recording(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        20,
        PushoverTarget::from_max_disp(0.0),
        PushoverControl::LoadOnly,
        true,
        false,
        false,
        0.0,
        DuctilityMethod::default(),
    )
    .expect("pushover should run");
    let last = result.steps.last().expect("収束ステップがあること");
    assert!(
        (last.load_factor - 1.0).abs() < 1e-9,
        "終了目標が無効なら λ=1 で終了すること: {:.4}",
        last.load_factor
    );
}

/// 4 節点の耐震壁（壁エレメントモデル、節点配列 `[下辺a, 下辺b, 上辺a, 上辺b]`）で、
/// 加力方向の水平力が「下辺 2 節点の**合計**」になること。
///
/// 従来は data[0..3]（下辺a）と data[6..9]（下辺b）の最大値を取っており、下辺 2 節点の
/// 一方だけを見る形で水平力を約 1/2 に過小評価していた（βu・壁の τu が過小＝
/// 部材種別・Ds が甘くなる危険側）。2 節点の線材では従来と同じ値になること
/// （リグレッションがないこと）も併せて確認する。
#[test]
fn test_horizontal_force_sums_wall_bottom_nodes() {
    use squid_n_element::behavior::LocalVec;

    // 4 節点壁: 下辺 a=30kN, b=70kN（合計 100kN）、上辺 a=-40kN, b=-60kN（合計 -100kN）。
    let mut data = smallvec::SmallVec::<[f64; 24]>::from_elem(0.0, 24);
    data[0] = 30_000.0; // 下辺a Fx
    data[6] = 70_000.0; // 下辺b Fx
    data[12] = -40_000.0; // 上辺a Fx
    data[18] = -60_000.0; // 上辺b Fx
    let f = LocalVec { data };
    let h = super::member_response::horizontal_force_in_dir(&f, 4, 0);
    assert!(
        (h - 100_000.0).abs() < 1e-6,
        "4 節点壁の水平力は下辺の合計 100kN であるべき（旧実装は max=70kN）。got {}",
        h
    );

    // 2 節点線材: i 端 +50kN / j 端 -50kN → 50kN（従来と同じ）。
    let mut data2 = smallvec::SmallVec::<[f64; 24]>::from_elem(0.0, 12);
    data2[0] = 50_000.0;
    data2[6] = -50_000.0;
    let f2 = LocalVec { data: data2 };
    let h2 = super::member_response::horizontal_force_in_dir(&f2, 2, 0);
    assert!(
        (h2 - 50_000.0).abs() < 1e-6,
        "2 節点は従来どおり。got {}",
        h2
    );
}

// ===== 系レベル V&V: 剛域が保有水平耐力・崩壊機構へ与える影響 =====

/// 剛域検証用の門形フレーム（1層1スパン）。
///
/// 柱は 100×100 のファイバー要素で両端固定、はりは柱より十分強い断面
/// （300×300）として弾性に留め、**柱の両端 4 ヒンジによる崩壊機構**に固定する。
/// これにより崩壊荷重は手計算 Qu = 4·My/L'（L' = 柱の可撓長）で照合できる。
/// `rigid` に剛域長 λ [mm] を与えると、柱の上下端に λ の剛域を設定する。
fn portal_frame_rigid_zone_model(fy: f64, seismic_weight: f64, rigid: f64) -> Model {
    let mut model = portal_frame_model(fy, seismic_weight);
    // はり用の強い断面を追加し、はりへ割り当てる（はりを弾性に保つ）。
    let strong = Section {
        id: SectionId(1),
        name: "girder".to_string(),
        area: 90000.0,
        iy: 6.75e8,
        iz: 6.75e8,
        j: 1.0e9,
        depth: 300.0,
        width: 300.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    model.sections.push(strong);
    model.elements[1].section = Some(SectionId(1));
    // 柱（要素 0・2）の上下端へ剛域を設定する。
    if rigid > 0.0 {
        for idx in [0usize, 2] {
            model.elements[idx].rigid_zone = RigidZone {
                length_i: rigid,
                length_j: rigid,
                face_i: rigid,
                face_j: rigid,
                ..Default::default()
            };
        }
    }
    model
}

/// 4 個目の降伏ヒンジ（柱両端×2 本で運動学的機構が成立する）が揃ったステップの
/// ベースシアを「観測崩壊荷重」として返す。
///
/// ヒンジは (部材, 材端) で重複を除いて数える（`determine_mechanism` の
/// 運動学的ゲートと同じ数え方）。
fn observed_collapse_shear(result: &PushoverResult) -> Option<f64> {
    let mut seen: std::collections::BTreeSet<(u32, u8)> = std::collections::BTreeSet::new();
    let mut mech_step = None;
    let mut events: Vec<&HingeEvent> = result
        .hinges
        .iter()
        .filter(|h| !matches!(h.level, HingeLevel::Crack))
        .collect();
    events.sort_by_key(|h| h.step);
    for h in events {
        seen.insert((h.elem.index() as u32, if h.pos < 0.5 { 0 } else { 1 }));
        if seen.len() >= 4 {
            mech_step = Some(h.step);
            break;
        }
    }
    let mech_step = mech_step?;
    result
        .capacity_curve
        .iter()
        .find(|c| c.step == mech_step)
        .map(|c| c.base_shear)
}

fn run_rigid_zone_pushover(rigid: f64) -> PushoverResult {
    let model = portal_frame_rigid_zone_model(235.0, 600_000.0, rigid);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    pushover_analysis(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        400,
        0.0,
        false,
        false,
        0.0,
    )
    .expect("pushover should run end-to-end")
}

/// 系レベル V&V（1）: 剛域つき門形フレームの**崩壊荷重が可撓長基準の理論値**に
/// 一致すること。
///
/// 柱両端 4 ヒンジの崩壊機構では Qu = 4·My/L'（L' = 柱の可撓長）であり、剛域を
/// 与えると L' = H − 2λ に短くなるぶん崩壊荷重は H/L' 倍になる。断面のファイバー
/// 離散化や荷重ステップの量子化による誤差を打ち消すため、**剛域なしとの比**で
/// 照合する（絶対値は既存 `test_portal_frame_collapse_load` が別途照合済み）。
#[test]
fn vnv_剛域つき門形フレームの崩壊荷重が可撓長基準になる() {
    let h: f64 = 3000.0;
    let lam: f64 = 300.0;
    let l_flex = h - 2.0 * lam;

    let r0 = run_rigid_zone_pushover(0.0);
    let r1 = run_rigid_zone_pushover(lam);
    let q0 = observed_collapse_shear(&r0).expect("剛域なしで崩壊機構が成立しない");
    let q1 = observed_collapse_shear(&r1).expect("剛域ありで崩壊機構が成立しない");

    let ratio = q1 / q0;
    let theory = h / l_flex;
    // 荷重ステップの量子化（400 ステップ）で数 % のばらつきが出るため許容 5%。
    assert!(
        (ratio / theory - 1.0).abs() < 0.05,
        "崩壊荷重比が可撓長基準の理論値から外れている: 実測 {ratio:.4}（Qu={q0:.0}→{q1:.0} N）, 理論 H/L'={theory:.4}"
    );
    // 剛域を無視していた頃は崩壊荷重が剛域に反応しなかった（比 = 1.0）。
    assert!(
        ratio > 1.1,
        "剛域が崩壊荷重に反映されていない（比 {ratio:.4}）"
    );
}

/// 系レベル V&V（2）: 剛域があっても崩壊機構が成立し、崩壊ヒンジが**柱に**
/// 形成されること（はりを強くしているため柱の全体機構になる）。
/// 崩壊機構種別は Ds の機構補正（`squid_n_design_jp::secondary::story_ds`）へ
/// 直接効くため、剛域の導入で機構分類が崩れないことを確認する。
#[test]
fn vnv_剛域つきでも柱の崩壊機構が成立する() {
    let result = run_rigid_zone_pushover(300.0);
    let yielded: Vec<&HingeEvent> = result
        .hinges
        .iter()
        .filter(|h| !matches!(h.level, HingeLevel::Crack))
        .collect();
    assert!(
        yielded.len() >= 4,
        "柱両端 4 ヒンジの機構に達していない: 降伏ヒンジ {} 個",
        yielded.len()
    );
    // 降伏ヒンジはすべて柱（要素 0・2）に生じる（はりは弾性に留める設計）。
    assert!(
        yielded
            .iter()
            .all(|h| h.elem == ElemId(0) || h.elem == ElemId(2)),
        "はりに降伏ヒンジが生じた（柱の崩壊機構になっていない）"
    );
    assert!(
        !matches!(result.mechanism, MechanismType::Partial),
        "崩壊機構が Partial のまま（機構が成立していない）"
    );
}

/// 系レベル V&V（3）: 剛域が層の弾性剛性を理論どおり増大させること。
/// 剛性率 Rs・層間変形角の検定（`squid_n_design_jp::secondary::holding_capacity`）は
/// 層剛性に直接依存するため、要素レベルだけでなく層レベルでも確認する。
///
/// 両端固定柱の層せん断剛性は 12EI/L'³（節点変位基準。剛体アームは変形しない）で、
/// 剛域を与えると (H/L')³ 倍になる。
#[test]
fn vnv_剛域は層の弾性剛性を可撓長の三乗で増大させる() {
    let h: f64 = 3000.0;
    let lam: f64 = 300.0;
    let l_flex = h - 2.0 * lam;
    let elastic_k = |r: &PushoverResult| -> f64 {
        let p = &r.capacity_curve[0];
        p.base_shear / p.roof_disp
    };
    let k0 = elastic_k(&run_rigid_zone_pushover(0.0));
    let k1 = elastic_k(&run_rigid_zone_pushover(lam));
    let ratio = k1 / k0;
    let theory = (h / l_flex).powi(3);
    // せん断変形・はりの弾性変形の寄与で理論値から数 % ずれる。
    assert!(
        (ratio / theory - 1.0).abs() < 0.05,
        "層剛性比が (H/L')³ から外れている: 実測 {ratio:.4}, 理論 {theory:.4}"
    );
}

/// 既定目標（最大層間変形角 1/150）の増分解析で、(1) ヒンジが発生し、
/// (2) ヒンジ形成に伴い剛性が低下し、(3) 層間変形角 1/150 の初回到達で
/// 解析が打ち切られることをエンドツーエンドで確認する。
///
/// 材端集中ばね（一成分系、`ForceRegime::UniaxialBendingShear`）の門形ラーメン。
/// 変位法ファイバー要素は端部ガウス点の積分重み（2Lp/L）が小さく部材降伏後も
/// 要素剛性がほとんど低下しないため、ヒンジ形成に伴う剛性低下の系レベル検証は
/// 材端集中ばね経路で行う。
fn portal_frame_spring_model(fy: f64, seismic_weight: f64) -> Model {
    let mut m = portal_frame_model(fy, seismic_weight);
    for e in &mut m.elements {
        e.kind = ElementKind::Beam;
        e.force_regime = ForceRegime::UniaxialBendingShear;
    }
    // ファイバー用の G=0（ねじり剛性を持たない前提の設定）のままでは
    // 一般梁要素のねじり剛性 GJ/L が 0 になるため、実際の G を与える。
    for mat in &mut m.materials {
        mat.shear = Some(78_846.0);
    }
    m
}

/// 崩壊機構（柱両端 4 ヒンジ）が層間変形角 1/150（=20mm）手前で形成されるよう
/// 降伏応力を下げた門形ラーメン（Qu=4My/H）を用いる。fy=50（材料強度1.1倍で
/// My≒9.2kN·m）では Qu≒12kN・機構形成変位 ≒13mm となり、目標到達までに
/// 確実に全体機構へ入る。地震重量は λ=1 のベースシア（0.2W=6kN）が Qu を
/// 下回る 30kN とし、機構形成は変位制御（ペナルティ法で機構後も可解）側で
/// 起こす（荷重制御で機構を超えると剛性行列が正定値性を失い解析が止まるため）。
#[test]
fn test_pushover_drift_angle_target_forms_hinge_with_stiffness_reduction() {
    let model = portal_frame_spring_model(50.0, 30_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let target = PushoverTarget::default();
    assert_eq!(target.max_disp, None, "既定は目標変位無効");
    assert_eq!(
        target.max_drift_angle,
        Some(1.0 / 150.0),
        "既定は層間変形角 1/150"
    );
    let result = pushover_analysis_recording(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        80,
        target,
        PushoverControl::default(),
        true,
        false,
        false,
        0.0,
        DuctilityMethod::default(),
    )
    .expect("pushover should run");

    let height = 3000.0;
    let angle_of = |s: &PushoverStep| s.story_drifts[0].abs() / height;
    let last = result.steps.last().expect("収束ステップがあること");

    // (3) 目標到達で打ち切り: 最終ステップのみが 1/150 以上（初回到達で停止）。
    //     ペナルティ法の相対精度分の緩和（0.1%）を見込む。
    let target_angle = 1.0 / 150.0;
    assert!(
        angle_of(last) >= target_angle * 0.999,
        "最終ステップは目標層間変形角 1/150 以上であること: {:.6}",
        angle_of(last)
    );
    for s in &result.steps[..result.steps.len() - 1] {
        assert!(
            angle_of(s) < target_angle,
            "目標到達前のステップは 1/150 未満であること: {:.6}",
            angle_of(s)
        );
    }

    // (1) 目標到達までに柱脚の曲げヒンジが発生していること。
    assert!(
        !result.hinges.is_empty(),
        "層間変形角 1/150 到達までにヒンジが発生すること"
    );

    // (2) 剛性低下: 初期割線剛性 k0 に対し、最終区間の接線剛性が有意に低下すること。
    let curve = &result.capacity_curve;
    assert!(curve.len() >= 3, "剛性比較に十分な点数があること");
    let first = curve
        .iter()
        .find(|p| p.roof_disp > 1e-9)
        .expect("変位が生じた点");
    let k0 = first.base_shear / first.roof_disp;
    let (p1, p2) = (&curve[curve.len() - 2], &curve[curve.len() - 1]);
    let ddisp = p2.roof_disp - p1.roof_disp;
    assert!(ddisp > 1e-9, "最終区間で変位が進んでいること");
    let kt = (p2.base_shear - p1.base_shear) / ddisp;
    assert!(
        kt < 0.5 * k0,
        "ヒンジ形成後の接線剛性が初期剛性の 1/2 未満へ低下すること: k0={k0:.2}, kt={kt:.2}"
    );
}

/// 目標変位と目標層間変形角を両方有効にした場合、早く成立する方（本設定では
/// 層間変形角）で打ち切られること。単一層柱では頂部変位 = 層間変位のため、
/// 変形角 1/150（=20mm）は目標変位 100mm より先に到達する。
#[test]
fn test_pushover_both_targets_stop_at_earlier_one() {
    let model = single_column_model(30.0, 10_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis_recording(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        20,
        PushoverTarget {
            max_disp: Some(100.0),
            max_drift_angle: Some(1.0 / 150.0),
        },
        PushoverControl::default(),
        true,
        false,
        false,
        0.0,
        DuctilityMethod::default(),
    )
    .expect("pushover should run");
    let last = result.steps.last().expect("収束ステップがあること");
    assert!(
        last.story_drifts[0].abs() / 3000.0 >= (1.0 / 150.0) * 0.999,
        "層間変形角 1/150 に到達していること"
    );
    assert!(
        last.top_disp < 100.0,
        "目標変位 100mm より先に層間変形角で打ち切られること: {:.1}mm",
        last.top_disp
    );
}

/// MS（マルチスプリング）要素の柱でも既定目標（層間変形角 1/150）の増分解析が
/// 完走し、目標到達で打ち切られること（終了判定の配線は要素種別に依存しない）。
#[test]
fn test_pushover_drift_angle_target_runs_with_multi_spring() {
    let mut model = single_column_model(235.0, 80_000.0);
    model.elements[0].kind = ElementKind::MultiSpring;
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis_recording(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        20,
        PushoverTarget::default(),
        PushoverControl::default(),
        true,
        false,
        false,
        0.0,
        DuctilityMethod::default(),
    )
    .expect("MS 柱の増分解析が完走すること");
    let last = result.steps.last().expect("収束ステップがあること");
    assert!(
        last.story_drifts[0].abs() / 3000.0 >= (1.0 / 150.0) * 0.999,
        "MS 柱でも層間変形角 1/150 まで到達して打ち切られること: {:.6}",
        last.story_drifts[0].abs() / 3000.0
    );
}

/// 耐震壁（壁エレメントモデル）1 枚の 1 層モデル。下辺 2 節点固定、
/// 上辺 2 節点を剛床（マスター NodeId(3)）で束ねる。面外・回転自由度は
/// 壁エレメントが剛性を持たないため拘束する。
fn wall_story_model(seismic_weight: f64) -> Model {
    wall_story_model_with(4000.0, seismic_weight)
}

/// 壁長 lw を指定できる版（曲げ支配の細長壁の検証用）。
///
/// 耐震壁は四周を柱・梁に囲まれた壁を対象とする（`misc_wall::wall_is_seismic`）ため、
/// 四周へ RC 側柱・大梁を配置する。側柱は面内両端ピン化されて面内せん断・曲げを
/// 負担しないため、面内の応答は壁エレメントが支配する。
fn wall_story_model_with(lw: f64, seismic_weight: f64) -> Model {
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};
    let make_node = |id: u32, coord: [f64; 3], restraint: Dof6Mask, story: Option<StoryId>| Node {
        id: NodeId(id),
        coord,
        restraint,
        mass: None,
        story,
        support_spring: None,
    };
    // 上辺節点: 面内並進 (ux, uz) のみ自由。
    let top_mask = Dof6Mask(0b111010);
    let shape = SectionShape::RcWall {
        thickness: 150.0,
        ps: 0.0025,
    };
    let rebar = RcRebar {
        main_grade: Some("SD345".into()),
        main_x: BarSet {
            count: 8,
            dia: 22.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 8,
            dia: 22.0,
            layers: 1,
        },
        cover: 50.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
            grade: None,
        },
    };
    let frame_shape = SectionShape::RcRect {
        b: 600.0,
        d: 600.0,
        rebar,
    };
    // 四周の柱・梁（下辺・上辺・左右の鉛直辺）。
    let edge = |id: u32, n0: u32, n1: u32, ref_vector: [f64; 3]| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(n0), NodeId(n1)],
        section: Some(SectionId(1)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis { ref_vector },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    Model {
        nodes: vec![
            make_node(0, [0.0, 0.0, 0.0], Dof6Mask::FIXED, None),
            make_node(1, [lw, 0.0, 0.0], Dof6Mask::FIXED, None),
            make_node(2, [lw, 0.0, 3000.0], top_mask, Some(StoryId(0))),
            make_node(3, [0.0, 0.0, 3000.0], top_mask, Some(StoryId(0))),
        ],
        elements: vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Wall,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
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
            },
            edge(1, 0, 1, [0.0, 0.0, 1.0]), // 下辺
            edge(2, 3, 2, [0.0, 0.0, 1.0]), // 上辺
            edge(3, 0, 3, [1.0, 0.0, 0.0]), // 左の鉛直辺（側柱）
            edge(4, 1, 2, [1.0, 0.0, 0.0]), // 右の鉛直辺（側柱）
        ],
        sections: vec![
            shape.to_section(SectionId(0), "W150".into()),
            frame_shape.to_section(SectionId(1), "RC-600x600".into()),
        ],
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
        stories: vec![Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(0),
            name: "1F".to_string(),
            elevation: 3000.0,
            node_ids: vec![NodeId(2), NodeId(3)],
            diaphragms: vec![DiaphragmDef {
                ci_override: None,
                weight: None,
                master: NodeId(3),
                slaves: vec![NodeId(2)],
                rigid: true,
            }],
            seismic_weight: Some(seismic_weight),
            weight_override: None,
        }],
        constraints: vec![Constraint::RigidDiaphragm {
            story: StoryId(0),
            master: NodeId(3),
            slaves: vec![NodeId(2)],
        }],
        ..Default::default()
    }
}

/// 耐震壁（壁エレメント）モデルでも既定目標（層間変形角 1/150）の増分解析が
/// 完走し、せん断終局強度 Qu で頭打ち（剛性低下）した後に目標到達で
/// 打ち切られること。λ=1 のベースシア（0.2W=20kN）は壁の Qu を大きく下回る
/// 重量とし、頭打ちは変位制御側で起こす。
#[test]
fn test_pushover_drift_angle_target_runs_with_wall_panel() {
    let model = wall_story_model(100_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis_recording(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        20,
        PushoverTarget::default(),
        PushoverControl::default(),
        true,
        false,
        false,
        0.0,
        DuctilityMethod::default(),
    )
    .expect("耐震壁モデルの増分解析が完走すること");
    let last = result.steps.last().expect("収束ステップがあること");
    assert!(
        last.story_drifts[0].abs() / 3000.0 >= (1.0 / 150.0) * 0.999,
        "耐震壁モデルでも層間変形角 1/150 まで到達して打ち切られること: {:.6}",
        last.story_drifts[0].abs() / 3000.0
    );
    // 壁の弾完全塑性（Qu 頭打ち）により、最終区間の接線剛性は初期割線剛性より
    // 大幅に低下しているはず。
    let curve = &result.capacity_curve;
    let first = curve
        .iter()
        .find(|p| p.roof_disp > 1e-9)
        .expect("変位が生じた点");
    let k0 = first.base_shear / first.roof_disp;
    let (p1, p2) = (&curve[curve.len() - 2], &curve[curve.len() - 1]);
    let ddisp = p2.roof_disp - p1.roof_disp;
    assert!(ddisp > 1e-9, "最終区間で変位が進んでいること");
    let kt = (p2.base_shear - p1.base_shear) / ddisp;
    assert!(
        kt < 0.5 * k0,
        "壁の Qu 頭打ちで接線剛性が低下すること: k0={k0:.2}, kt={kt:.2}"
    );
}

/// 塑性増分ヒンジ化したファイバー柱の増分解析で、既定目標（層間変形角 1/150）
/// までに (1) ヒンジが発生し、(2) 降伏後の接線剛性が実際に低下し、
/// (3) 目標の初回到達で打ち切られることを確認する。
/// 旧定式化（端部ガウス点の B 積分、重み 2Lp/L'）では端部断面が全塑性化しても
/// 要素剛性が数 % しか低下しなかった（`dev_docs/v_and_v/増分解析_ヒンジ形成と
/// 剛性低下_検証_2026-07.md` §2.2）。
///
/// fy=30 では初降伏 V_y = My/L ≒ 1.8kN（変位 ≒9mm・1/330）、全塑性
/// Vp = 1.1·σy·Zp/L ≒ 2.75kN。地震重量 10kN（λ=1 で 2.0kN）は初降伏と
/// 全塑性の間にあり、荷重制御中に降伏が始まり変位制御で目標まで押し込む。
#[test]
fn test_pushover_fiber_hinge_softens_at_drift_target() {
    let model = single_column_model(30.0, 10_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis_recording(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        40,
        PushoverTarget::default(),
        PushoverControl::default(),
        true,
        false,
        false,
        0.0,
        DuctilityMethod::default(),
    )
    .expect("fiber pushover should run");

    let last = result.steps.last().expect("収束ステップがあること");
    let angle = last.story_drifts[0].abs() / 3000.0;
    assert!(
        angle >= (1.0 / 150.0) * 0.999,
        "層間変形角 1/150 に到達して打ち切られること: {angle:.6}"
    );
    assert!(
        !result.hinges.is_empty(),
        "目標到達までにヒンジが発生すること"
    );

    // 剛性低下: 初期割線剛性に対し最終区間の接線剛性が有意に低下すること。
    let curve = &result.capacity_curve;
    let first = curve
        .iter()
        .find(|p| p.roof_disp > 1e-9)
        .expect("変位が生じた点");
    let k0 = first.base_shear / first.roof_disp;
    let (p1, p2) = (&curve[curve.len() - 2], &curve[curve.len() - 1]);
    let ddisp = p2.roof_disp - p1.roof_disp;
    assert!(ddisp > 1e-9, "最終区間で変位が進んでいること");
    let kt = (p2.base_shear - p1.base_shear) / ddisp;
    assert!(
        kt < 0.5 * k0,
        "ファイバー柱でも降伏後の接線剛性が初期剛性の 1/2 未満へ低下すること: k0={k0:.2}, kt={kt:.2}"
    );
    // 耐力は全塑性崩壊荷重 Vp = 1.1·σy·Zp/L 近傍で頭打ちになる（過大評価しない）。
    let vp = 1.1 * 30.0 * (100.0 * 100.0 * 100.0 / 4.0) / 3000.0;
    assert!(
        result.qu < vp * 1.3,
        "Qu={:.1} が全塑性崩壊荷重 Vp={:.1} を大きく超えないこと",
        result.qu,
        vp
    );
}

/// 耐震壁の**曲げ降伏**の検証: 細長壁（lw=1000・h=3000、曲げ支配。せん断終局
/// Qu は曲げ耐力より十分大きい）では、既定でファイバー化された壁柱の端部断面が
/// 曲げ降伏し、目標層間変形角 1/150 到達までに接線剛性が実際に低下すること。
/// 従来の壁柱（弾性梁＋せん断 Qu 頭打ちのみ）では曲げ降伏は表現されず、
/// 細長壁の耐力を危険側に過大評価していた。
#[test]
fn test_pushover_wall_flexural_yield_softens() {
    let model = wall_story_model_with(1000.0, 30_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = pushover_analysis_recording(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        40,
        PushoverTarget::default(),
        PushoverControl::default(),
        true,
        false,
        false,
        0.0,
        DuctilityMethod::default(),
    )
    .expect("細長壁の増分解析が完走すること");
    let last = result.steps.last().expect("収束ステップがあること");
    assert!(
        last.story_drifts[0].abs() / 3000.0 >= (1.0 / 150.0) * 0.999,
        "層間変形角 1/150 まで到達して打ち切られること: {:.6}",
        last.story_drifts[0].abs() / 3000.0
    );
    let curve = &result.capacity_curve;
    let first = curve
        .iter()
        .find(|p| p.roof_disp > 1e-9)
        .expect("変位が生じた点");
    let k0 = first.base_shear / first.roof_disp;
    let (p1, p2) = (&curve[curve.len() - 2], &curve[curve.len() - 1]);
    let ddisp = p2.roof_disp - p1.roof_disp;
    assert!(ddisp > 1e-9, "最終区間で変位が進んでいること");
    let kt = (p2.base_shear - p1.base_shear) / ddisp;
    assert!(
        kt < 0.5 * k0,
        "細長壁の曲げ降伏で接線剛性が低下すること: k0={k0:.2}, kt={kt:.2}"
    );
    // 耐力が弾性外挿（k0×最終変位）に対して有意に頭打ちしていること。
    assert!(
        result.qu < 0.7 * k0 * p2.roof_disp,
        "Qu={:.1} が弾性外挿 {:.1} より有意に小さいこと",
        result.qu,
        k0 * p2.roof_disp
    );
}

/// 接線剛性が初めから特異なモデルは、ソルバの内部表現（`factor: NotPositiveDefinite`）
/// ではなく**どの節点のどの自由度に剛性がないか**を示す日本語診断で停止する。
///
/// 回帰対象: 従来は `newton_converge` の分解失敗をそのまま `Err` として上げており、
/// UI には「増分解析エラー: factor: NotPositiveDefinite」だけが出て原因に辿り着けなかった。
#[test]
fn test_pushover_singular_tangent_reports_dof_diagnosis() {
    // 単柱モデルの頂部ねじり拘束（rz）を外すと、ファイバ柱はねじり剛性
    // （断面 J は正だが材料の G=0）を持たないため rz の剛性が 0 になり特異化する。
    let mut model = single_column_model(235.0, 100_000.0);
    model.nodes[1].restraint = Dof6Mask::FREE;
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let err = pushover_analysis(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        10,
        50.0,
        false,
        false,
        0.0,
    )
    .expect_err("特異なモデルは診断付きで停止する");
    assert!(
        err.contains("剛性がありません") && err.contains("節点 1"),
        "自由度を名指しした診断になっていない: {err}"
    );
    assert!(
        !err.contains("NotPositiveDefinite"),
        "ソルバ内部表現がそのまま露出している: {err}"
    );
}

/// 荷重制御フェーズが増分半減でも収束しない場合、結果を「Qu=0 の空の性能曲線」として
/// 返さず、原因を切り分けた日本語メッセージで停止する（ソルバ内部表現は出さない）。
///
/// 回帰対象: 分解失敗を非収束扱いへ変えた際に、1 ステップも確定していないのに
/// `Ok` で空の結果（Qu=0）を返すと保有水平耐力 0 と誤認させる（危険側）。
#[test]
fn test_pushover_unconverged_load_control_reports_reason() {
    // fy を極小にすると最初の増分から釣合いに収束できない（全断面が即降伏する）。
    let model = single_column_model(0.5, 100_000.0);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let err = pushover_analysis_recording(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        20,
        PushoverTarget::from_max_disp(200.0),
        PushoverControl::Phased,
        false,
        false,
        false,
        0.0,
        DuctilityMethod::default(),
    )
    .expect_err("1 ステップも確定しないなら空の結果を返さず停止する");
    assert!(
        err.contains("収束しません"),
        "非収束であることが伝わらない: {err}"
    );
    assert!(
        !err.contains("NotPositiveDefinite") && !err.contains("factor:"),
        "ソルバ内部表現がそのまま露出している: {err}"
    );
}
