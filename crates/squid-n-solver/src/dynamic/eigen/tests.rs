use super::*;
use crate::constraint::Reducer;
use squid_n_core::dof::{Dof, Dof6Mask, DofMap};
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    Constraint, ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model,
    Node, Section,
};

/// Ux のみ自由（並進1方向）にするマスク。
const FREE_UX: Dof6Mask = Dof6Mask(0b111110);

/// 軸ばね 1 本（剛性 k=EA/L）＋先端質量 m の 1 自由度モデル。
/// node0 固定、node1 は Ux のみ自由で質量 m を持つ。
/// 理論固有周期 T = 2π√(m/k)。
fn make_1dof_spring_model() -> Model {
    let k = 1000.0_f64;
    let m = 1.0_f64;
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
                restraint: FREE_UX,
                mass: Some([m, 0.0, 0.0, 0.0, 0.0, 0.0]),
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(1),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
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
            name: "spring".into(),
            area: 1.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 1.0,
            width: 1.0,
            as_y: 1.0,
            as_z: 1.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".into(),
            young: k * 1000.0 / 1.0, // EA/L = young*1/1000 = k
            poisson: 0.0,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        ..Default::default()
    }
}

/// 2層等質量等剛性せん断モデル（軸ばね2本の直列）。
/// node0 固定、node1/node2 は Ux のみ自由で各質量 m。
/// K=[[2k,-k],[-k,k]], M=mI。λ=(k/m)(3∓√5)/2。
fn make_shear_2dof_model() -> Model {
    let k = 1000.0_f64;
    let m = 1.0_f64;
    let young = k * 1000.0; // EA/L = young*1/1000 = k
    let node = |id: u32, x: f64, restraint: Dof6Mask, mass: Option<[f64; 6]>| Node {
        id: NodeId(id),
        coord: [x, 0.0, 0.0],
        restraint,
        mass,
        story: None,
        support_spring: None,
    };
    let beam = |id: u32, a: u32, b: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(a), NodeId(b)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    Model {
        nodes: vec![
            node(0, 0.0, Dof6Mask::FIXED, None),
            node(1, 1000.0, FREE_UX, Some([m, 0.0, 0.0, 0.0, 0.0, 0.0])),
            node(2, 2000.0, FREE_UX, Some([m, 0.0, 0.0, 0.0, 0.0, 0.0])),
        ],
        elements: vec![beam(1, 0, 1), beam(2, 1, 2)],
        sections: vec![Section {
            id: SectionId(0),
            name: "spring".into(),
            area: 1.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 1.0,
            width: 1.0,
            as_y: 1.0,
            as_z: 1.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".into(),
            young,
            poisson: 0.0,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        ..Default::default()
    }
}

/// 門型ラーメン相当モデル（柱2本＋梁1本、柱脚固定、柱頭2節点は6自由度すべて自由）。
/// crates/squid-n-app の sample::portal_frame() を模したジオメトリだが、
/// 材料密度は 0 とし、質量は柱頭2節点の水平(Ux)自由度のみに集中質量として与える
/// （実務でよく使う「水平質点系」モデル化）。
/// 縮約後自由度は 12(=2節点×6) あるが、質量を持つ自由度はそのうち 2 つだけなので、
/// 縮約後質量行列 M_red のランクは厳密に 2 になる（10自由度は完全に質量ゼロ）。
fn make_portal_frame_like_model(top_mass: f64) -> Model {
    let coords = [
        [0.0, 0.0, 0.0],
        [6000.0, 0.0, 0.0],
        [0.0, 0.0, 3500.0],
        [6000.0, 0.0, 3500.0],
    ];
    let nodes = coords
        .iter()
        .enumerate()
        .map(|(i, c)| Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if i < 2 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: if i >= 2 && top_mass > 0.0 {
                Some([top_mass, 0.0, 0.0, 0.0, 0.0, 0.0])
            } else {
                None
            },
            story: None,
            support_spring: None,
        })
        .collect();
    let col_section = Section {
        id: SectionId(0),
        name: "col".into(),
        area: 11980.0,
        iy: 2.04e8,
        iz: 6.75e7,
        j: 3.54e6,
        depth: 300.0,
        width: 300.0,
        as_y: 6000.0,
        as_z: 6000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let beam_section = Section {
        id: SectionId(1),
        name: "beam".into(),
        area: 8337.0,
        iy: 2.37e8,
        iz: 2.62e7,
        j: 1.0e6,
        depth: 400.0,
        width: 200.0,
        as_y: 4000.0,
        as_z: 4000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let members = [(0u32, 1u32, 2u32, 0u32), (1, 1, 3, 0), (2, 2, 3, 1)];
    let elements = members
        .iter()
        .map(|&(id, i, j, sec)| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(i), NodeId(j)],
            section: Some(SectionId(sec)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: if sec == 0 {
                    [1.0, 0.0, 0.0]
                } else {
                    [0.0, 0.0, 1.0]
                },
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        })
        .collect();
    Model {
        nodes,
        elements,
        sections: vec![col_section, beam_section],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SN400B".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0, // 質量は集中質量のみで与える（水平質点系モデル化）
            shear: None,
            fc: None,
            fy: Some(235.0),
        }],
        ..Default::default()
    }
}

/// 再現テスト: 質量ランク(=2) が要求モード数(2)ちょうどでも、
/// 部分空間反復の作業次元 q(>2) の中で射影質量行列がランク落ちするため、
/// 修正前の実装は Cholesky 分解失敗 → 対角フォールバック(diag_fallback)に落ち、
/// 質量ゼロ方向の θ=k/m を素朴に計算して f64::MAX を混入させていた
/// （diag_fallback は projected な q×q 行列を「対角」とみなす近似で、
/// 非対角の結合を無視するため、q>質量ランクでは必ず不正確になる）。
/// 修正後は質量固有分解によりランク落ちを正しく分離し、2 つの有限な固有値が
/// 得られることを確認する。
#[test]
fn test_eigen_portal_frame_like_mass_rank_equals_n_modes() {
    let model = make_portal_frame_like_model(1.0e-3);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = solve_eigen(&model, &dofmap, &reducer, 2)
        .expect("質量ランクが要求モード数以上なら解けるべき");

    assert_eq!(result.omega2.len(), 2);
    for (i, &w2) in result.omega2.iter().enumerate() {
        assert!(
            w2.is_finite() && w2 > 0.0,
            "mode{}: omega2={} は有限な正値であるべき(f64::MAX 混入は禁止)",
            i,
            w2
        );
    }
    for (i, &t) in result.period.iter().enumerate() {
        assert!(
            t.is_finite() && t > 0.0,
            "mode{}: period={} は有限な正値であるべき",
            i,
            t
        );
    }
    // 周期は昇順のモードで降順（1次が最長周期）。
    assert!(
        result.period[0] > result.period[1],
        "T1={} T2={} は T1>T2 であるべき",
        result.period[0],
        result.period[1]
    );
}

/// 質量ランク不足(ランク1)で2モードを要求した場合は、f64::MAX を混ぜて返さず、
/// 日本語の明示エラーを返すことを確認する。
#[test]
fn test_eigen_mass_rank_deficient_returns_explicit_error() {
    let mut model = make_portal_frame_like_model(1.0e-3);
    // node3(柱頭2つ目)の質量を落とし、質量ランクを 1 にする。
    model.nodes[3].mass = None;
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = solve_eigen(&model, &dofmap, &reducer, 2);
    let err = match result {
        Err(e) => e,
        Ok(r) => panic!(
            "質量ランク(1) < 要求モード数(2) はエラーになるべきだが omega2={:?} が返った",
            r.omega2
        ),
    };
    let msg = err.to_string();
    assert!(
        msg.contains("質量"),
        "エラーメッセージは質量ランク不足を説明すべき: {}",
        msg
    );
}

/// 負の節点質量（符号誤りなどの入力不備）は、M の半正定値性が崩れて一般化
/// Jacobi の前提が破れ誤った固有値が「正常終了」で返るため、明示エラーになること。
#[test]
fn test_eigen_negative_mass_returns_explicit_error() {
    let mut model = make_portal_frame_like_model(1.0e-3);
    // 合計質量（トレース）は正のまま、対角に負が混じる状態を作る
    // （トレースまで非正だと既存の「質量ゼロ」検出が先に発火する）。
    model.nodes[2].mass = Some([-0.5e-3, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = solve_eigen(&model, &dofmap, &reducer, 1);
    let err = match result {
        Err(e) => e,
        Ok(r) => panic!(
            "負質量はエラーになるべきだが omega2={:?} が返った",
            r.omega2
        ),
    };
    let msg = err.to_string();
    assert!(msg.contains("負"), "エラーは負質量を説明すべき: {}", msg);
}

/// `gevd_jacobi` は M=I（単位行列）を渡すと標準固有値問題 K z = θ z に一致する。
#[test]
fn test_gevd_jacobi_2x2_identity_mass() {
    let k = vec![2.0, 1.0, 1.0, 3.0];
    let m = vec![1.0, 0.0, 0.0, 1.0];
    let (vals, vecs) = gevd_jacobi(&k, &m, 2);
    let expected_1 = (5.0 - 5.0_f64.sqrt()) / 2.0;
    let expected_2 = (5.0 + 5.0_f64.sqrt()) / 2.0;
    assert!((vals[0] - expected_1).abs() < 1e-10, "val0={}", vals[0]);
    assert!((vals[1] - expected_2).abs() < 1e-10, "val1={}", vals[1]);
    // M=I 正規直交（zᵀz=1）であることを確認する。
    for j in 0..2 {
        let mut norm = 0.0;
        for i in 0..2 {
            norm += vecs[i * 2 + j] * vecs[i * 2 + j];
        }
        assert!((norm - 1.0).abs() < 1e-10, "vec{} not normalized", j);
    }
}

#[test]
fn test_1dof_period() {
    let k = 1000.0_f64;
    let m = 1.0_f64;
    let expected_omega2 = k / m;
    let expected_t = 2.0 * std::f64::consts::PI / expected_omega2.sqrt();

    let model = make_1dof_spring_model();
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = solve_eigen(&model, &dofmap, &reducer, 1).unwrap();

    // 質量・自由度が正しく組まれていれば 1 モードが得られる。
    assert_eq!(result.omega2.len(), 1, "1 モードが解けていない");
    assert!(result.omega2[0] > 0.0, "omega2={}", result.omega2[0]);
    // 反復解法だが SPD 1 自由度なので高精度に収束する（理論一致・許容差）。
    assert!(
        (result.omega2[0] - expected_omega2).abs() / expected_omega2 < 1e-8,
        "omega2={} expected={}",
        result.omega2[0],
        expected_omega2
    );
    // 設計書 §7.2 の例: T = 0.198692 s
    assert!(
        (result.period[0] - expected_t).abs() / expected_t < 1e-8,
        "T={} expected={}",
        result.period[0],
        expected_t
    );
    assert!(
        (result.period[0] - 0.198692).abs() < 1e-5,
        "T={} 設計書例 0.198692 と不一致",
        result.period[0]
    );
}

/// 2層せん断モデル: T1=0.32150, T2=0.12280 へ収束し、
/// 2 モードで有効質量比合計 ≈100%（設計書 §7.2）。
#[test]
fn test_2dof_shear_period_and_mass() {
    let model = make_shear_2dof_model();
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = solve_eigen(&model, &dofmap, &reducer, 2).unwrap();

    assert_eq!(result.omega2.len(), 2);
    let k = 1000.0_f64;
    let m = 1.0_f64;
    let lam1 = (k / m) * (3.0 - 5.0_f64.sqrt()) / 2.0; // ≈382.0
    let lam2 = (k / m) * (3.0 + 5.0_f64.sqrt()) / 2.0; // ≈2618.0

    assert!(
        (result.omega2[0] - lam1).abs() / lam1 < 1e-6,
        "λ1={} expected={}",
        result.omega2[0],
        lam1
    );
    assert!(
        (result.omega2[1] - lam2).abs() / lam2 < 1e-6,
        "λ2={} expected={}",
        result.omega2[1],
        lam2
    );

    let t1 = 2.0 * std::f64::consts::PI / result.omega2[0].sqrt();
    let t2 = 2.0 * std::f64::consts::PI / result.omega2[1].sqrt();
    assert!((t1 - 0.32150).abs() < 1e-4, "T1={}", t1);
    assert!((t2 - 0.12280).abs() < 1e-4, "T2={}", t2);

    // X 方向有効質量の合計が全質量 2m に一致（有効質量比合計 ≈100%）。
    let total_mass = 2.0 * m;
    let eff_sum: f64 = result.effective_mass.iter().map(|e| e[0]).sum();
    assert!(
        (eff_sum - total_mass).abs() / total_mass < 1e-6,
        "有効質量合計={} 全質量={}",
        eff_sum,
        total_mass
    );
    // モード1 が支配的。理論値は閉形式から求める（このKでは ≈94.7%）。
    // 1次モード形 φ=[1, k/(k−λ1)] より Meff1 = (Σφ)²/(Σφ²)。
    let s = k / (k - lam1); // φ2/φ1
    let meff1_theory = (1.0 + s).powi(2) / (1.0 + s * s);
    let ratio1 = result.effective_mass[0][0] / total_mass;
    assert!(
        (ratio1 - meff1_theory / total_mass).abs() < 1e-6,
        "mode1 有効質量比={} 理論={}",
        ratio1,
        meff1_theory / total_mass
    );
}

/// crates/squid-n-app の sample::portal_frame() と等価なモデル
/// （柱2本＋梁1本、柱脚固定、H形断面、材料密度あり・節点集中質量なし）。
/// 断面性能は H-300x300x10x15（柱）・H-400x200x8x13（梁）の実断面計算値。
/// 質量は一貫質量行列(consistent mass)のみから生じ、並進DOFに比べ
/// 回転DOFの質量ははるかに小さい（質量行列が病的に悪条件）。
/// この状態で eigen(1)〜eigen(3) が f64::MAX を返さず、妥当な周期を
/// 返すことを確認する（実際に発生した不具合の再現・回帰テスト）。
fn make_portal_frame_density_mass_model() -> Model {
    let coords = [
        [0.0, 0.0, 0.0],
        [6000.0, 0.0, 0.0],
        [0.0, 0.0, 3500.0],
        [6000.0, 0.0, 3500.0],
    ];
    let nodes = coords
        .iter()
        .enumerate()
        .map(|(i, c)| Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if i < 2 {
                Dof6Mask::FIXED
            } else {
                Dof6Mask::FREE
            },
            mass: None,
            story: None,
            support_spring: None,
        })
        .collect();
    let col_section = Section {
        id: SectionId(0),
        name: "col".into(),
        area: 11700.0,
        iy: 1.993275e8,
        iz: 6.75225e7,
        j: 7.65e5,
        depth: 300.0,
        width: 300.0,
        as_y: 6000.0,
        as_z: 6000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let beam_section = Section {
        id: SectionId(1),
        name: "beam".into(),
        area: 8192.0,
        iy: 2.2965e8,
        iz: 1.735e7,
        j: 3.568e5,
        depth: 400.0,
        width: 200.0,
        as_y: 4000.0,
        as_z: 4000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let members = [(0u32, 1u32, 2u32, 0u32), (1, 1, 3, 0), (2, 2, 3, 1)];
    let elements = members
        .iter()
        .map(|&(id, i, j, sec)| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(i), NodeId(j)],
            section: Some(SectionId(sec)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: if sec == 0 {
                    [1.0, 0.0, 0.0]
                } else {
                    [0.0, 0.0, 1.0]
                },
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        })
        .collect();
    Model {
        nodes,
        elements,
        sections: vec![col_section, beam_section],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SN400B".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: Some(235.0),
        }],
        ..Default::default()
    }
}

#[test]
fn test_eigen_portal_frame_density_mass_two_modes() {
    let model = make_portal_frame_density_mass_model();
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);

    let result = solve_eigen(&model, &dofmap, &reducer, 2)
        .expect("密度由来の質量でも eigen(2) は解けるべき");
    assert_eq!(result.omega2.len(), 2);
    for (i, &w2) in result.omega2.iter().enumerate() {
        assert!(
            w2.is_finite() && w2 > 0.0,
            "mode{}: omega2={} は有限な正値であるべき(f64::MAX 混入は禁止)",
            i,
            w2
        );
    }
    assert!(
        result.period[0] > result.period[1],
        "T1={} T2={} は T1>T2 であるべき",
        result.period[0],
        result.period[1]
    );
    // ラーメン(高さ3.5m・スパン6mの鋼構造)としては数百ms〜1s台が妥当なオーダー。
    assert!(
        result.period[0] > 0.01 && result.period[0] < 5.0,
        "T1={} は非物理的なオーダー",
        result.period[0]
    );
}

/// 質量方式 LumpedOnly では部材密度による要素質量を質量行列に算入しないことを、
/// 固有周期で確認する回帰テスト。密度を持つ 1 自由度ばねモデルで、
/// 既定（CorrectedLumped: 要素質量＋節点質量）では周期が節点質量のみの
/// 理論値からずれ、LumpedOnly では理論値 T=2π√(m/k) に一致する。
#[test]
fn test_eigen_mass_method_lumped_only_skips_element_mass() {
    let mut model = make_1dof_spring_model();
    // 密度を与えて要素質量を発生させる（値は節点質量と同程度のオーダー）。
    model.materials[0].density = 1.0e-3;

    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let t_default = solve_eigen(&model, &dofmap, &reducer, 1).unwrap().period[0];

    model.mass_method = squid_n_core::model::MassMethod::LumpedOnly;
    let t_lumped = solve_eigen(&model, &dofmap, &reducer, 1).unwrap().period[0];

    let k = 1000.0_f64;
    let m = 1.0;
    let t_theory = 2.0 * std::f64::consts::PI * (m / k).sqrt();
    assert!(
        (t_lumped - t_theory).abs() < 1e-9 * t_theory,
        "LumpedOnly の周期 {} が節点質量のみの理論値 {} と不一致",
        t_lumped,
        t_theory
    );
    assert!(
        t_default > t_theory * 1.01,
        "既定方式の周期 {} は要素質量の分だけ理論値 {} より長くなるはず",
        t_default,
        t_theory
    );
}

/// 決定性テスト: 固有値解析を10回実行しビット一致を確認
#[test]
fn test_eigen_deterministic() {
    let model = make_1dof_spring_model();
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let first = solve_eigen(&model, &dofmap, &reducer, 1).unwrap();
    for _ in 0..9 {
        let cur = solve_eigen(&model, &dofmap, &reducer, 1).unwrap();
        assert_eq!(first.omega2.len(), cur.omega2.len());
        for (a, b) in first.omega2.iter().zip(cur.omega2.iter()) {
            assert_eq!(a.to_bits(), b.to_bits());
        }
        for (a, b) in first.period.iter().zip(cur.period.iter()) {
            assert_eq!(a.to_bits(), b.to_bits());
        }
        for (s_a, s_b) in first.shapes.iter().zip(cur.shapes.iter()) {
            assert_eq!(s_a.len(), s_b.len());
            for (va, vb) in s_a.iter().zip(s_b.iter()) {
                assert_eq!(va.to_bits(), vb.to_bits());
            }
        }
    }
}

/// 柱2本(柱脚とも同一節点で固定)＋剛床モデル。柱頭2節点(スレーブ)を
/// 浮遊マスター節点(Uz/Rx/Ry固定)へ RigidDiaphragm で従属させ、マスターには
/// 並進(Ux,Uy)と回転(Rz)の集中質量を与える（質量ランクはちょうど3＝Ux,Uy,Rz）。
/// `node_shapes` が縮約座標からの展開後も剛床の面内剛体変位の運動学
/// （ix = mx − θz·dy, iy = my + θz・dx）を満たすことを検証するためのモデル
/// （`constraint.rs` の `test_rigid_diaphragm_master_recovers_translation_and_torsion`
/// と同じ式）。
///
/// 本テストは `node_shapes` の展開処理そのものの正しさを検証することが目的の
/// ため、最小構成（縮約後独立自由度9・柱2本）を用いる。並進質量と回転慣性の
/// スケール差による質量ランク過少検出（かつて柱4本構成で顕在化していた）の
/// 回帰は `test_eigen_mass_rank_translation_rotation_scale_mix`（柱4本・
/// `make_four_column_diaphragm_model`）が担う。
fn make_diaphragm_columns_model(top_mass: f64, rot_mass: f64) -> Model {
    let col_section = Section {
        id: SectionId(0),
        name: "col".into(),
        area: 11980.0,
        iy: 2.04e8,
        iz: 6.75e7,
        j: 3.54e6,
        depth: 300.0,
        width: 300.0,
        as_y: 6000.0,
        as_z: 6000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mut master_restraint = Dof6Mask::FREE;
    master_restraint.set_fixed(Dof::Uz);
    master_restraint.set_fixed(Dof::Rx);
    master_restraint.set_fixed(Dof::Ry);

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
            // 柱頭(スレーブ1): X方向にオフセット
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3500.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            },
            // マスター(剛床代表節点): 面内(Ux,Uy,Rz)のみ自由
            Node {
                id: NodeId(2),
                coord: [1000.0, 0.0, 3500.0],
                restraint: master_restraint,
                mass: Some([top_mass, top_mass, 0.0, 0.0, 0.0, rot_mass]),
                story: None,
                support_spring: None,
            },
            // 柱頭(スレーブ2): Y方向にオフセット（マスターから見て非対称な配置）
            Node {
                id: NodeId(3),
                coord: [0.0, 1000.0, 3500.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
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
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
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
        sections: vec![col_section],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SN400B".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0, // 質量はマスター節点の集中質量のみ
            shear: None,
            fc: None,
            fy: Some(235.0),
        }],
        constraints: vec![Constraint::RigidDiaphragm {
            story: StoryId(0),
            master: NodeId(2),
            slaves: vec![NodeId(1), NodeId(3)],
        }],
        ..Default::default()
    }
}

/// 回帰テスト: `ModalResult::node_shapes`（`shapes` を節点×6成分へ展開したもの）
/// が、剛床(RigidDiaphragm)のスレーブ節点でも剛体変位の運動学
/// ix = mx − θz·(y_s−y_m), iy = my + θz·(x_s−x_m) を満たすこと、
/// 固定節点の成分が0であること、モード数・節点数が `shapes` と整合することを確認する。
#[test]
fn test_eigen_node_shapes_rigid_diaphragm_kinematics() {
    let top_mass = 1.0e-3;
    let rot_mass = 5.0e4;
    let model = make_diaphragm_columns_model(top_mass, rot_mass);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let n_modes = 3;
    let result = solve_eigen(&model, &dofmap, &reducer, n_modes)
        .expect("質量ランク(Ux,Uy,Rz=3)ちょうどの要求モード数(3)は解けるべき");

    // (a) モード数・節点数の整合
    assert_eq!(
        result.node_shapes.len(),
        result.shapes.len(),
        "node_shapes のモード数が shapes と不一致"
    );
    assert_eq!(result.node_shapes.len(), n_modes);
    for ns in &result.node_shapes {
        assert_eq!(ns.len(), model.nodes.len(), "node_shapes の節点数が不一致");
    }

    let master = 2usize;
    let master_coord = model.nodes[master].coord;
    let slaves = [1usize, 3];

    // (c) 全モードで水平成分が自明にゼロにならないことの確認用
    let mut max_horizontal: f64 = 0.0;
    // 回転成分についても同様に確認する（θz が全モードでゼロなら、
    // 剛床の回転項(θz·dx, θz·dy)を実質的に検証していないことになるため）。
    let mut max_rotation: f64 = 0.0;

    for ns in &result.node_shapes {
        let ux_m = ns[master][0];
        let uy_m = ns[master][1];
        let theta_z = ns[master][5];
        max_horizontal = max_horizontal.max(ux_m.abs()).max(uy_m.abs());
        max_rotation = max_rotation.max(theta_z.abs());

        // (b) 剛床スレーブの水平成分が剛体条件と整合すること。
        // 許容誤差は「相対1e-9」を基本としつつ、期待値がモード内の代表的な
        // 変位スケールに対してほぼ0になる場合でも誤って厳しくなりすぎない
        // よう、モード内の最大変位スケールに対する絶対誤差1e-9も許容する
        // （相対誤差1e-9 と 絶対誤差1e-9・(モード内最大スケール) の緩い方）。
        let mode_scale = ux_m
            .abs()
            .max(uy_m.abs())
            .max((theta_z * 1000.0_f64).abs())
            .max(1.0);
        for &s in &slaves {
            let dx = model.nodes[s].coord[0] - master_coord[0];
            let dy = model.nodes[s].coord[1] - master_coord[1];
            let expected_ux = ux_m - theta_z * dy;
            let expected_uy = uy_m + theta_z * dx;

            let tol_ux = 1e-9 * expected_ux.abs().max(mode_scale);
            let tol_uy = 1e-9 * expected_uy.abs().max(mode_scale);
            assert!(
                (ns[s][0] - expected_ux).abs() < tol_ux,
                "slave{} ux: got={} want={}",
                s,
                ns[s][0],
                expected_ux
            );
            assert!(
                (ns[s][1] - expected_uy).abs() < tol_uy,
                "slave{} uy: got={} want={}",
                s,
                ns[s][1],
                expected_uy
            );
        }

        // (d) 固定(柱脚)節点は全成分0
        for (comp, &v) in ns[0].iter().enumerate() {
            assert_eq!(v, 0.0, "固定節点0 成分{} は0であるべき: {}", comp, v);
        }
    }

    assert!(
        max_horizontal > 1e-6,
        "全モードで水平成分がほぼ0（自明に成立するだけの検証になっている）: max={}",
        max_horizontal
    );
    assert!(
        max_rotation > 1e-9,
        "全モードで回転成分θzがほぼ0（剛床の回転項を実質検証できていない）: max={}",
        max_rotation
    );
}

/// 柱4本（正方形配置・柱脚固定）＋剛床の1層モデル。柱頭4節点をスレーブとし、
/// 床重心の浮遊マスター節点（Uz/Rx/Ry固定）に並進(Ux,Uy)と回転(Rz)の集中質量を
/// 与える（質量ランクはちょうど3＝Ux,Uy,Rz）。剛床付き建物の最も標準的な
/// モデル化（各階を水平2並進＋回転1の質点で代表させる）に対応する。
fn make_four_column_diaphragm_model(top_mass: f64, rot_mass: f64) -> Model {
    let col_section = Section {
        id: SectionId(0),
        name: "col".into(),
        area: 11980.0,
        iy: 2.04e8,
        iz: 6.75e7,
        j: 3.54e6,
        depth: 300.0,
        width: 300.0,
        as_y: 6000.0,
        as_z: 6000.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let mut master_restraint = Dof6Mask::FREE;
    master_restraint.set_fixed(Dof::Uz);
    master_restraint.set_fixed(Dof::Rx);
    master_restraint.set_fixed(Dof::Ry);

    let bases = [[0.0, 0.0], [6000.0, 0.0], [0.0, 6000.0], [6000.0, 6000.0]];
    let mut nodes = Vec::new();
    let mut elements = Vec::new();
    let mut slaves = Vec::new();
    for (i, [x, y]) in bases.iter().enumerate() {
        let base_id = NodeId((i * 2) as u32);
        let top_id = NodeId((i * 2 + 1) as u32);
        nodes.push(Node {
            id: base_id,
            coord: [*x, *y, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
            support_spring: None,
        });
        nodes.push(Node {
            id: top_id,
            coord: [*x, *y, 3500.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
        elements.push(ElementData {
            id: ElemId(i as u32),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![base_id, top_id],
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
        });
        slaves.push(top_id);
    }
    let master_id = NodeId(8);
    nodes.push(Node {
        id: master_id,
        coord: [3000.0, 3000.0, 3500.0],
        restraint: master_restraint,
        mass: Some([top_mass, top_mass, 0.0, 0.0, 0.0, rot_mass]),
        story: None,
        support_spring: None,
    });

    Model {
        nodes,
        elements,
        sections: vec![col_section],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SN400B".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0, // 質量はマスター節点の集中質量のみ
            shear: None,
            fc: None,
            fy: Some(235.0),
        }],
        constraints: vec![Constraint::RigidDiaphragm {
            story: StoryId(0),
            master: master_id,
            slaves,
        }],
        ..Default::default()
    }
}

/// 回帰テスト: 並進質量(t オーダー)と回転慣性(t·mm² オーダー、単位系due to
/// 並進の 10^6〜10^8 倍のスケール)が混在する剛床モデルで、質量ランク判定が
/// スケール差により過少検出されないこと。
///
/// 従来の実装は射影質量行列 M̄ の固有値を「最大固有値との相対値」で
/// 切り捨てていたため、回転慣性の固有値が支配的になると並進方向の質量が
/// 「質量なし」と誤判定され、真の質量ランク(3)未満しか見つからず
/// InvalidInput エラーになっていた（剛床で縮約後の独立自由度が多い＝
/// 部分空間に質量ゼロ方向が多く混ざるほど顕在化する）。
#[test]
fn test_eigen_mass_rank_translation_rotation_scale_mix() {
    let top_mass = 1.0e-3;
    let rot_mass = 5.0e4;
    let model = make_four_column_diaphragm_model(top_mass, rot_mass);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = solve_eigen(&model, &dofmap, &reducer, 3)
        .expect("質量ランク3（マスターのUx,Uy,Rz）に対し3モードの要求は解けるべき");

    assert_eq!(result.period.len(), 3);
    for (i, &t) in result.period.iter().enumerate() {
        assert!(
            t.is_finite() && t > 0.0,
            "モード{}の周期が正の有限値でない: {}",
            i + 1,
            t
        );
    }

    // 正方形対称配置なので X/Y 並進は同一周期の縮退モード、回転(Rz)モードが
    // 別周期で現れる。有効質量は X・Y 各方向とも全質量（top_mass）が
    // 並進モードに現れるはず（合計で相対誤差1e-6以内）。
    for dir in 0..2 {
        let sum: f64 = result.effective_mass.iter().map(|em| em[dir]).sum();
        assert!(
            (sum - top_mass).abs() < 1e-6 * top_mass,
            "方向{}の有効質量合計 {} が全質量 {} と不一致",
            dir,
            sum,
            top_mass
        );
    }
}

/// 部分空間反復の質量重み付け（`K·y=M·x`）が正しく効いていることを、
/// 質量が非対称（m1≠m2）な2層軸ばねモデルの解析解と比較して確認する。
/// M∝I（等質量）の [`test_2dof_shear_period_and_mass`] では質量重み付けの
/// 有無が結果に影響しないため区別できない（この2つは相補的な回帰テスト）。
#[test]
fn test_2dof_shear_unequal_mass_matches_analytic() {
    let k = 1000.0_f64;
    let young = k * 1000.0;
    let node = |id: u32, x: f64, restraint: Dof6Mask, mass: Option<[f64; 6]>| Node {
        id: NodeId(id),
        coord: [x, 0.0, 0.0],
        restraint,
        mass,
        story: None,
        support_spring: None,
    };
    let beam = |id: u32, a: u32, b: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(a), NodeId(b)],
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let model = Model {
        nodes: vec![
            node(0, 0.0, Dof6Mask::FIXED, None),
            node(1, 1000.0, FREE_UX, Some([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])),
            node(2, 2000.0, FREE_UX, Some([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])),
        ],
        elements: vec![beam(1, 0, 1), beam(2, 1, 2)],
        sections: vec![Section {
            id: SectionId(0),
            name: "spring".into(),
            area: 1.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 1.0,
            width: 1.0,
            as_y: 1.0,
            as_z: 1.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".into(),
            young,
            poisson: 0.0,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        ..Default::default()
    };
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let result = solve_eigen(&model, &dofmap, &reducer, 2).unwrap();

    // 2λ² - 5000λ + 1e6 = 0 の根（K=[[2000,-1000],[-1000,1000]], M=diag(1,2)）
    let disc = (5000.0_f64.powi(2) - 4.0 * 2.0 * 1.0e6).sqrt();
    let lam1 = (5000.0 - disc) / 4.0;
    let lam2 = (5000.0 + disc) / 4.0;
    assert!(
        (result.omega2[0] - lam1).abs() / lam1 < 1e-6,
        "λ1計算値={} 理論値={}",
        result.omega2[0],
        lam1
    );
    assert!(
        (result.omega2[1] - lam2).abs() / lam2 < 1e-6,
        "λ2計算値={} 理論値={}",
        result.omega2[1],
        lam2
    );
}

#[test]
fn test_eigen_subspace_matches_dense_ground_truth_q_lt_n() {
    // n_modes=1 で q(=5) < n(=8) となる直列質点鎖を作り、部分空間反復の結果
    // solve_eigen(...,1) を、同じ K_red/M_red を dense 化して gevd_jacobi に
    // 直接渡した「厳密解（反復なし）」と比較する回帰テスト。
    // 質量を大小交互（1.0, 50.0, ...）にして質量分布を強く非一様にし、
    // 部分空間が n を張れない条件でも最低次モードへ正しく収束することを確認する。
    let k = 1000.0_f64;
    let young = k * 1000.0;
    let n_masses = 8usize;
    let mut nodes = vec![Node {
        id: NodeId(0),
        coord: [0.0, 0.0, 0.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
        support_spring: None,
    }];
    for i in 1..=n_masses {
        let m = if i % 2 == 1 { 1.0 } else { 50.0 };
        nodes.push(Node {
            id: NodeId(i as u32),
            coord: [1000.0 * i as f64, 0.0, 0.0],
            restraint: FREE_UX,
            mass: Some([m, 0.0, 0.0, 0.0, 0.0, 0.0]),
            story: None,
            support_spring: None,
        });
    }
    let elements = (0..n_masses)
        .map(|i| ElementData {
            id: ElemId(i as u32 + 1),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(i as u32), NodeId(i as u32 + 1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        })
        .collect();
    let model = Model {
        nodes,
        elements,
        sections: vec![Section {
            id: SectionId(0),
            name: "spring".into(),
            area: 1.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 1.0,
            width: 1.0,
            as_y: 1.0,
            as_z: 1.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".into(),
            young,
            poisson: 0.0,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        ..Default::default()
    };
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);

    let result = solve_eigen(&model, &dofmap, &reducer, 1).unwrap();

    // 同じ K_red/M_red を dense 化し、gevd_jacobi へ直接渡す（反復なしの厳密解）。
    let k_free = assemble_global_k(&model, &dofmap);
    let k_red = reducer.reduce_k(&k_free);
    let m_free = assemble_global_m(
        &model,
        &dofmap,
        squid_n_element::behavior::MassOption::Consistent,
    );
    let m_red = reducer.reduce_k(&m_free);
    let n = k_red.nrows();
    let mut k_dense = vec![0.0; n * n];
    let mut m_dense = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            k_dense[i * n + j] = k_red.get(i, j).copied().unwrap_or(0.0);
            m_dense[i * n + j] = m_red.get(i, j).copied().unwrap_or(0.0);
        }
    }
    let (exact_vals, _) = gevd_jacobi(&k_dense, &m_dense, n);

    assert!(
        (result.omega2[0] - exact_vals[0]).abs() / exact_vals[0] < 1e-6,
        "部分空間反復の結果 {} が厳密解 {} と不一致（q<n での収束先ずれの疑い）",
        result.omega2[0],
        exact_vals[0]
    );
}
