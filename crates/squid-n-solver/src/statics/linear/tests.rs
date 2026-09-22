use super::*;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, JointKind, LoadCase, LocalAxis, Material,
    MaterialCategory, MemberDetailAttr, MemberJoint, MemberLoad, MemberLoadKind, Model, NodalLoad,
    Node, Section,
};

/// 単純梁（i:ピン, j:ローラ）に等分布荷重 → 中央曲げ wL²/8、端部 0 を検証。
/// 曲げは静定なので EI に依らず厳密。組立（等価節点力）＋回復（重ね合わせ）の総合検証。
#[test]
fn simply_supported_udl_midspan_moment() {
    let l = 1000.0_f64;
    let w = 2.0_f64;
    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                // Ux,Uy,Uz,Rx 拘束（並進ピン＋ねじり剛体モード除去）
                restraint: Dof6Mask(0b001111),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [l, 0.0, 0.0],
                // Uy,Uz 拘束（ローラ。Ux 自由）
                restraint: Dof6Mask(0b000110),
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
            name: "s".into(),
            area: 1000.0,
            iy: 1.0e7,
            iz: 1.0e7,
            j: 1.0e6,
            depth: 200.0,
            width: 100.0,
            as_y: 800.0,
            as_z: 800.0,
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
            name: "m".into(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "udl".into(),
            nodal: vec![],
            member: vec![MemberLoad::manual(
                ElemId(0),
                [0.0, 0.0, -1.0],
                MemberLoadKind::Distributed {
                    a: 0.0,
                    b: l,
                    w1: w,
                    w2: w,
                },
            )],
        }],
        ..Default::default()
    };

    let res = linear_static_once(&model, LoadCaseId(1)).expect("solve");
    let (_, mf) = res
        .member_forces
        .iter()
        .find(|(id, _)| *id == ElemId(0))
        .expect("member forces for elem 0");

    let expected_mid = w * l * l / 8.0;
    let mut mid_mz = None;
    let mut end_mz_max = 0.0_f64;
    for (xi, vals) in &mf.at {
        let mz = vals[5];
        if (xi - 0.5).abs() < 1e-9 {
            mid_mz = Some(mz);
        }
        if (*xi < 1e-9) || ((xi - 1.0).abs() < 1e-9) {
            end_mz_max = end_mz_max.max(mz.abs());
        }
    }
    let mid = mid_mz.expect("midspan section present");
    assert!(
        (mid.abs() - expected_mid).abs() / expected_mid < 1e-3,
        "midspan Mz={} expected {}",
        mid,
        expected_mid
    );
    assert!(
        end_mz_max < expected_mid * 1e-3,
        "end Mz should be ~0, got {}",
        end_mz_max
    );
}

/// 単純梁モデル（長さ l、i:ピン+ねじり拘束, j:ローラ）を指定の部材荷重で作る。
fn ss_beam(l: f64, member: Vec<MemberLoad>) -> Model {
    Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask(0b001111),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [l, 0.0, 0.0],
                restraint: Dof6Mask(0b000110),
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
            name: "s".into(),
            area: 1000.0,
            iy: 1.0e7,
            iz: 1.0e7,
            j: 1.0e6,
            depth: 200.0,
            width: 100.0,
            as_y: 800.0,
            as_z: 800.0,
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
            name: "m".into(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "lc".into(),
            nodal: vec![],
            member,
        }],
        ..Default::default()
    }
}

fn value_at(mf: &squid_n_element::frame::beam::MemberForces, xi: f64, comp: usize) -> f64 {
    mf.at
        .iter()
        .find(|(x, _)| (x - xi).abs() < 1e-9)
        .map(|(_, v)| v[comp])
        .expect("section present")
}

fn mid_value(mf: &squid_n_element::frame::beam::MemberForces, comp: usize) -> f64 {
    value_at(mf, 0.5, comp)
}

/// 単純梁・材長 1/4 点の集中荷重 P の内力回復を、荷重より先の断面 x=3L/8 で
/// 検証する。x>a なので `fixed_internal_local` の Point 回復項（合力・
/// モーメント）が非ゼロで効く。
///
/// 静定梁の手計算: 支点反力 R_i = P·b/L（b = L−a）、M(x) = R_i·x − P·(x−a)、
/// Qy(x) = R_i − P。評価断面 x=3L/8 は既定の評価断面（0, L/2, L）に含まれない
/// ため、部材付帯情報の継手位置で追加する（剛性・応力解析には影響しない）。
#[test]
fn simply_supported_off_center_point_load_internal_force() {
    let l = 1000.0_f64;
    let p = 500.0_f64;
    let a = l / 4.0;
    let b = l - a;
    let r_i = p * b / l;
    let x = 3.0 * l / 8.0;
    let mut model = ss_beam(
        l,
        vec![MemberLoad::manual(
            ElemId(0),
            [0.0, 0.0, -1.0],
            MemberLoadKind::Point { a, p },
        )],
    );
    model.member_detail_attrs.push(MemberDetailAttr {
        elem: ElemId(0),
        haunch_i: None,
        haunch_j: None,
        joints: vec![MemberJoint {
            distance: x,
            kind: JointKind::Site,
        }],
    });
    let res = linear_static_once(&model, LoadCaseId(1)).expect("solve");
    let (_, mf) = res
        .member_forces
        .iter()
        .find(|(id, _)| *id == ElemId(0))
        .unwrap();
    let xi = x / l;
    let m = value_at(mf, xi, 5);
    let expected_m = r_i * x - p * (x - a);
    assert!(
        (m - expected_m).abs() / expected_m < 1e-3,
        "point Mz={} expected {}",
        m,
        expected_m
    );
    let shear = value_at(mf, xi, 1);
    let expected_shear = r_i - p;
    assert!(
        (shear - expected_shear).abs() / expected_shear.abs() < 1e-3,
        "point Qy={} expected {}",
        shear,
        expected_shear
    );
}

/// 単純梁・全体 Y 方向 UDL（ローカル z 面）→ 中央 My = wL²/8。z 面の符号検証。
#[test]
fn simply_supported_udl_zplane_moment() {
    let l = 1000.0_f64;
    let w = 1.5_f64;
    let model = ss_beam(
        l,
        vec![MemberLoad::manual(
            ElemId(0),
            [0.0, -1.0, 0.0],
            MemberLoadKind::Distributed {
                a: 0.0,
                b: l,
                w1: w,
                w2: w,
            },
        )],
    );
    let res = linear_static_once(&model, LoadCaseId(1)).expect("solve");
    let (_, mf) = res
        .member_forces
        .iter()
        .find(|(id, _)| *id == ElemId(0))
        .unwrap();
    let expected = w * l * l / 8.0;
    let mid = mid_value(mf, 4).abs();
    assert!(
        (mid - expected).abs() / expected < 1e-3,
        "zplane mid My={} expected {}",
        mid,
        expected
    );
    // ねじり・Mz は概ね 0
    assert!(mid_value(mf, 5).abs() < expected * 1e-3, "Mz leak");
}

fn make_axial_cantilever() -> Model {
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
            name: "sec".to_string(),
            area: 100.0,
            iy: 1000.0,
            iz: 1000.0,
            j: 100.0,
            depth: 100.0,
            width: 100.0,
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
            name: "mat".to_string(),
            category: MaterialCategory::Steel,
            young: 1000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "axial".to_string(),
            nodal: vec![NodalLoad::manual(
                NodeId(1),
                [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )],
            member: vec![],
        }],
        ..Default::default()
    }
}

#[test]
fn test_linear_static_axial_cantilever() {
    let model = make_axial_cantilever();
    let result = linear_static_once(&model, LoadCaseId(1)).unwrap();
    assert!(
        (result.disp[1][0] - 10.0).abs() < 1e-6,
        "ux={}",
        result.disp[1][0]
    );
    assert!(result.member_forces.len() == 1);
    let forces = &result.member_forces[0].1;
    // 軸力 N は部材内力（引張正）。先端 +1000N の引張で全断面 N=+1000。
    for (xi, vals) in &forces.at {
        assert!((vals[0] - 1000.0).abs() < 1e-6, "N(ξ={})={}", xi, vals[0]);
    }
}

#[test]
fn axial_distributed_gravity_recovers_full_base_compression() {
    let mut model = make_axial_cantilever();
    model.nodes[1].coord = [0.0, 0.0, 1000.0];
    model.elements[0].local_axis.ref_vector = [1.0, 0.0, 0.0];
    model.load_cases[0].nodal.clear();
    model.load_cases[0].member = vec![MemberLoad::manual(
        ElemId(0),
        [0.0, 0.0, -1.0],
        MemberLoadKind::Distributed {
            a: 0.0,
            b: 1000.0,
            w1: 2.0,
            w2: 2.0,
        },
    )];
    let result = linear_static_once(&model, LoadCaseId(1)).unwrap();
    // 上端自由の柱は、各断面より上の全重量を圧縮軸力として負担する。
    for (xi, values) in &result.member_forces[0].1.at {
        let expected = -2000.0 * (1.0 - xi);
        assert!(
            (values[0] - expected).abs() < 1e-6,
            "xi={xi}: N={}, expected={expected}",
            values[0]
        );
    }
}

/// X 軸上の片持ち梁に「グローバル Y 方向」の先端荷重をかける。
/// 参照ベクトル [0,0,1] では local y = global Z（鉛直上）、local z = global −Y と
/// なるので、水平（Y 方向）たわみは弱軸＝断面 **iz** で決まる（iy=強軸ではない）。
/// クロス変換（construct.rs）または to_global を欠くと iy を使ってしまい誤る。
/// よって iy≠iz の断面で、δ = PL³/(3E·iz) に一致することを確認する。
#[test]
fn test_beam_to_global_transverse_uses_correct_inertia() {
    // 現実的な鋼材大断面（iz=1e9 級）を用いる：to_global 修正の検証に加え、
    // 端ばね静縮約のペナルティが大断面でも非正定値化しないこと（堅牢性）も同時に確認。
    let e = 205000.0_f64;
    let l = 1000.0_f64;
    let iy = 2.0e9_f64;
    let iz = 1.0e9_f64;
    let p = 10000.0_f64;
    let mut model = make_axial_cantilever();
    model.materials[0].young = e;
    model.sections[0].iy = iy;
    model.sections[0].iz = iz;
    model.sections[0].as_y = 1.0e9;
    model.sections[0].as_z = 1.0e9;
    model.load_cases[0].nodal[0].values = [0.0, p, 0.0, 0.0, 0.0, 0.0];

    let result = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let uy = result.disp[1][1];
    let expected = p * l.powi(3) / (3.0 * e * iz);
    let buggy = p * l.powi(3) / (3.0 * e * iy);
    // iz ベースの値に一致し、iy ベース(1/2)を明確に排除する。
    assert!(
        (uy - expected).abs() / expected < 1e-3,
        "uy={} expected(iz)={} buggy(iy)={}",
        uy,
        expected,
        buggy
    );
}

/// 剛域がモデル→解析へ接続され、結果に効くことのエンドツーエンド確認。
/// 同一片持ち梁で、基部に大きな剛域（可とう長を短縮）を入れると、
/// 先端たわみが明確に小さく（剛く）なる。
#[test]
fn test_rigid_zone_affects_analysis() {
    let mut base = make_axial_cantilever();
    base.sections[0].iy = 1.0e7;
    base.sections[0].iz = 1.0e7;
    base.sections[0].as_y = 1.0e8;
    base.sections[0].as_z = 1.0e8;
    base.load_cases[0].nodal[0].values = [0.0, 0.0, 1000.0, 0.0, 0.0, 0.0];

    // 剛域なし
    let r0 = linear_static_once(&base, LoadCaseId(1)).unwrap();
    let uz0 = r0.disp[1][2];

    // 基部に剛域 λ_i=800（可とう長 200）
    let mut rigid = base.clone();
    rigid.elements[0].rigid_zone.length_i = 800.0;
    let r1 = linear_static_once(&rigid, LoadCaseId(1)).unwrap();
    let uz1 = r1.disp[1][2];

    assert!(
        uz0.abs() > 0.0 && uz1.abs() > 0.0,
        "uz0={} uz1={}",
        uz0,
        uz1
    );
    assert!(
        uz1.abs() < 0.5 * uz0.abs(),
        "剛域で剛くなるはず: uz_norigid={} uz_rigid={}",
        uz0,
        uz1
    );
}

#[test]
fn test_linear_static_vertical_cantilever_bending() {
    // 鉛直柱: (0,0,0)固定 → (0,0,1000)自由。頂部に水平荷重 P=1000 (global X)。
    // 座標変換が正しく適用されれば曲げ片持ち応答 δx ≈ PL³/3E·Iz + せん断 ≈ 333,364。
    // 回転変換が欠落していると軸剛性を誤用して δx≈10 になる（回帰防止）。
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
                coord: [0.0, 0.0, 1000.0],
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
            name: "sec".to_string(),
            area: 100.0,
            iy: 1000.0,
            iz: 1000.0,
            j: 100.0,
            depth: 100.0,
            width: 100.0,
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
            name: "mat".to_string(),
            category: MaterialCategory::Steel,
            young: 1000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "h".to_string(),
            nodal: vec![NodalLoad::manual(
                NodeId(1),
                [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )],
            member: vec![],
        }],
        ..Default::default()
    };
    let result = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let ux = result.disp[1][0];
    // 曲げ主成分 333,333 + せん断 ~31。軸剛性誤用(=10)を確実に弾く帯域で判定。
    assert!(
            (333_000.0..=334_000.0).contains(&ux),
            "vertical cantilever tip ux={ux} (expected ~333,364 bending; got axial ~10 means rotation missing)"
        );
}

#[test]
fn test_linear_static_deterministic() {
    let model = make_axial_cantilever();
    let first = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let second = linear_static_once(&model, LoadCaseId(1)).unwrap();
    assert_eq!(first.disp, second.disp);
    assert_eq!(first.member_forces.len(), second.member_forces.len());
    for (a, b) in first.member_forces.iter().zip(second.member_forces.iter()) {
        assert_eq!(a.0, b.0);
        assert_eq!(a.1.at, b.1.at);
    }
}

#[test]
fn test_shell_rigid_floor_membrane_off() {
    use squid_n_core::model::{Constraint, Story};

    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: Some(StoryId(0)),
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [100.0, 0.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(StoryId(0)),
                support_spring: None,
            },
            Node {
                id: NodeId(2),
                coord: [100.0, 100.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(StoryId(0)),
                support_spring: None,
            },
            Node {
                id: NodeId(3),
                coord: [0.0, 100.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: Some(StoryId(0)),
                support_spring: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Shell,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
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
            name: "shell".to_string(),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            floor: None,
            panel_thickness: None,
            thickness: Some(10.0),
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
            name: "mat".to_string(),
            category: MaterialCategory::Steel,
            young: 1000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        stories: vec![Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(0),
            name: "floor".to_string(),
            elevation: 0.0,
            node_ids: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            seismic_weight: None,
            weight_override: None,
        }],
        constraints: vec![Constraint::rigid_diaphragm(
            StoryId(0),
            NodeId(0),
            vec![NodeId(1), NodeId(2), NodeId(3)],
        )],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "load".to_string(),
            nodal: vec![NodalLoad::manual(NodeId(2), [0.0, 0.0, 1.0, 0.0, 0.0, 0.0])],
            member: vec![],
        }],
        ..Default::default()
    };

    let res = linear_static_once(&model, LoadCaseId(1)).unwrap();
    assert!(
        res.disp[1][0].abs() < 1e-12 && res.disp[1][1].abs() < 1e-12,
        "slave should not move in-plane: {:?}",
        [res.disp[1][0], res.disp[1][1]]
    );
    assert!(
        res.disp[2][2].abs() > 1e-12,
        "shell should deflect vertically: {}",
        res.disp[2][2]
    );
}

/// 単純支持正方形板（等分布荷重）の N×N メッシュモデルを作る。
/// 周辺=単純支持（Uz=0, 縁回転自由）。面内は全節点で固定（平板曲げ＝面内変位0）。
fn make_ss_plate(n: usize, a: f64, t: f64, e: f64, nu: f64, q: f64, clamped: bool) -> Model {
    let h = a / n as f64;
    let nn = n + 1;
    let idx = |ix: usize, iy: usize| (iy * nn + ix) as u32;
    let mut nodes = Vec::new();
    for iy in 0..nn {
        for ix in 0..nn {
            let on_boundary = ix == 0 || ix == n || iy == 0 || iy == n;
            // 常に Ux,Uy,Rz を固定（面内＋ドリリング）。周辺は Uz も固定。
            let mut mask = 0b100011u8;
            if on_boundary {
                mask |= 1 << 2;
                if clamped {
                    mask |= 1 << 3;
                    mask |= 1 << 4;
                }
            }
            nodes.push(Node {
                id: NodeId(idx(ix, iy)),
                coord: [ix as f64 * h, iy as f64 * h, 0.0],
                restraint: Dof6Mask(mask),
                mass: None,
                story: None,
                support_spring: None,
            });
        }
    }
    let mut elements = Vec::new();
    let mut eid = 0u32;
    for iy in 0..n {
        for ix in 0..n {
            elements.push(ElementData {
                id: ElemId(eid),
                kind: ElementKind::Shell,
                nodes: smallvec::smallvec![
                    NodeId(idx(ix, iy)),
                    NodeId(idx(ix + 1, iy)),
                    NodeId(idx(ix + 1, iy + 1)),
                    NodeId(idx(ix, iy + 1)),
                ],
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
            eid += 1;
        }
    }
    // 等分布荷重 q を負担面積で節点 Fz へ（周辺節点の荷重は支点が負担）。
    let mut nodal = Vec::new();
    for iy in 0..nn {
        for ix in 0..nn {
            let wx = if ix == 0 || ix == n { 0.5 } else { 1.0 };
            let wy = if iy == 0 || iy == n { 0.5 } else { 1.0 };
            let fz = q * (wx * h) * (wy * h);
            nodal.push(NodalLoad::manual(
                NodeId(idx(ix, iy)),
                [0.0, 0.0, fz, 0.0, 0.0, 0.0],
            ));
        }
    }
    Model {
        nodes,
        elements,
        sections: vec![Section {
            id: SectionId(0),
            name: "plate".into(),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            floor: None,
            panel_thickness: None,
            thickness: Some(t),
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
            name: "m".into(),
            category: MaterialCategory::Steel,
            young: e,
            poisson: nu,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "q".into(),
            nodal,
            member: vec![],
        }],
        ..Default::default()
    }
}

/// 正方形板（等分布荷重）の中央たわみが参照解 α·q·a⁴/D に収束すること。
///
/// ケース表で境界条件と α を切り替える。単純支持 α=0.00406、四辺固定（クランプ）
/// α=0.00126。各ケースで 4・8・16 分割の誤差が単調減少し、16×16 で参照解の
/// ±2% 以内（仕様 §9.3）に入る。
#[test]
fn test_plate_convergence() {
    let (a, t, e, nu, q) = (1000.0_f64, 10.0_f64, 200000.0_f64, 0.3_f64, 0.01_f64);
    let d = e * t.powi(3) / (12.0 * (1.0 - nu * nu));

    for (clamped, alpha, label) in [
        (false, 0.00406_f64, "単純支持"),
        (true, 0.00126_f64, "四辺固定"),
    ] {
        let ref_w = alpha * q * a.powi(4) / d;
        let center_w = |n: usize| -> f64 {
            let model = make_ss_plate(n, a, t, e, nu, q, clamped);
            let res = linear_static_once(&model, LoadCaseId(1)).unwrap();
            let nn = n + 1;
            let c = (n / 2) * nn + (n / 2);
            res.disp[c][2].abs()
        };

        let w4 = center_w(4);
        let w8 = center_w(8);
        let w16 = center_w(16);
        let e4 = (w4 - ref_w).abs();
        let e8 = (w8 - ref_w).abs();
        let e16 = (w16 - ref_w).abs();

        // 細分化で誤差が単調減少して参照解へ近づく
        assert!(
            e8 < e4 && e16 < e8,
            "{label}: 誤差が単調減少しない: e4={e4} e8={e8} e16={e16} (w4={w4} w8={w8} w16={w16} ref={ref_w})"
        );
        // 16×16 で参照解の ±2% 以内
        assert!(
            e16 / ref_w < 0.02,
            "{label}: 16x16 誤差 {:.2}% > 2% (w16={} ref={})",
            e16 / ref_w * 100.0,
            w16,
            ref_w
        );
    }
}

// 1 スパン・2 柱・頂部大梁・対角ブレース 1 本のモデル（ブレース付きラーメン）。
// 柱・大梁は Fixed-Fixed（曲げ骨組）、ブレースは Pinned-Pinned のトラス要素。
// 荷重ケースを 2 本（Dead=長期, Seismic=短期）用意し、同一の鉛直荷重を与える。
fn braced_frame(kind: squid_n_core::model::LoadCaseKind) -> Model {
    use squid_n_core::model::StressAnalysisCfg;

    let l = 4000.0_f64;
    let h = 3000.0_f64;
    let node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    };
    let sec = Section {
        id: SectionId(0),
        name: "steel".into(),
        area: 6000.0,
        iy: 8.0e7,
        iz: 8.0e7,
        j: 1.0e6,
        depth: 300.0,
        width: 300.0,
        as_y: 5000.0,
        as_z: 5000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let mat = Material {
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
    };
    Model {
        nodes: vec![
            {
                let mut n = node(0, [0.0, 0.0, 0.0]);
                n.restraint = Dof6Mask::FIXED;
                n
            },
            {
                let mut n = node(1, [l, 0.0, 0.0]);
                n.restraint = Dof6Mask::FIXED;
                n
            },
            node(2, [0.0, 0.0, h]),
            node(3, [l, 0.0, h]),
        ],
        elements: vec![
            // e0: 柱A（鉛直）
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(2)],
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            // e1: 柱B（鉛直）
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(1), NodeId(3)],
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            // e2: 頂部大梁（水平）
            ElementData {
                id: ElemId(2),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(2), NodeId(3)],
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            // e3: 対角ブレース（柱Aの脚部 → 柱Bの頂部）
            ElementData {
                id: ElemId(3),
                kind: ElementKind::Brace {
                    tension_only: false,
                },
                nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Pinned, EndCondition::Pinned],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![sec],
        materials: vec![mat],
        load_cases: vec![LoadCase {
            id: LoadCaseId(1),
            name: "gravity".into(),
            nodal: vec![NodalLoad::manual(
                NodeId(2),
                [0.0, 0.0, -1.0e5, 0.0, 0.0, 0.0],
            )],
            member: vec![],
            kind,
        }],
        stress_cfg: StressAnalysisCfg::default(),
        ..Default::default()
    }
}

// 柱の長期軸力無効化テスト用モデル。同一の2節点間に「柱」（`ElementKind::Beam`、
// 鉛直、Fixed-Fixed）と「鉛直ブレース」（`ElementKind::Brace`、同じ2節点）を
// 並列に配置する。荷重方向が部材軸と厳密に一致するため、柱の軸剛性が
// 支配的な baseline では柱・ブレースへほぼ等分に軸力を負担させつつ、
// 柱側だけ曲げ・せん断剛性（iy/iz/j/as_y/as_z）はそのまま健全に保つ。
// ブレース側は常に軸剛性のみを保つため、柱の軸力を無効化しても
// 機構化（特異行列）を起こさず、負担していた軸力がそのままブレースへ
// 移ることを明快に検証できる。
fn column_with_parallel_vertical_brace() -> Model {
    use squid_n_core::model::StressAnalysisCfg;

    let h = 3000.0_f64;
    let sec = Section {
        id: SectionId(0),
        name: "steel".into(),
        area: 6000.0,
        iy: 8.0e7,
        iz: 8.0e7,
        j: 1.0e6,
        depth: 300.0,
        width: 300.0,
        as_y: 5000.0,
        as_z: 5000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let mat = Material {
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
    };
    Model {
        nodes: vec![
            {
                let mut n = Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                    support_spring: None,
                };
                n.restraint = Dof6Mask::FIXED;
                n
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, h],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![
            // e0: 柱（鉛直）。no_long_axial_column の対象。
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
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
            },
            // e1: 同じ2節点間の鉛直ブレース（並列）。常に軸剛性を保ち、
            // 柱が負担しなくなった軸力の受け皿になる。
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Brace {
                    tension_only: false,
                },
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Pinned, EndCondition::Pinned],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![sec],
        materials: vec![mat],
        load_cases: vec![LoadCase {
            id: LoadCaseId(1),
            name: "gravity".into(),
            nodal: vec![NodalLoad::manual(
                NodeId(1),
                [0.0, 0.0, -1.0e5, 0.0, 0.0, 0.0],
            )],
            member: vec![],
            kind: squid_n_core::model::LoadCaseKind::Dead,
        }],
        stress_cfg: StressAnalysisCfg::default(),
        ..Default::default()
    }
}

fn axial_force(res: &StaticOnce, elem: ElemId) -> f64 {
    let (_, mf) = res
        .member_forces
        .iter()
        .find(|(id, _)| *id == elem)
        .unwrap_or_else(|| panic!("member forces for elem {:?} not found", elem));
    mf.at[0].1[0]
}

/// 検証1: `no_long_axial_brace=true` の長期ケースでは、ブレース軸力がほぼ0
/// （元の1e-3倍以下）になり、周囲の柱（柱A。ブレースが基部で直接取り付く側の
/// 柱で、荷重も直接負担する主経路）の軸力の絶対値が増えること
/// （一貫構造計算プログラムの実務慣行）。
#[test]
fn test_no_long_axial_brace_zeros_brace_force_and_increases_column() {
    let mut model = braced_frame(squid_n_core::model::LoadCaseKind::Dead);
    let base = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let base_brace = axial_force(&base, ElemId(3)).abs();
    let base_col_a = axial_force(&base, ElemId(0)).abs();
    assert!(
        base_brace > 1.0,
        "baseline brace force should be non-trivial: {base_brace}"
    );

    model.stress_cfg.no_long_axial_brace = true;
    let cut = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let cut_brace = axial_force(&cut, ElemId(3)).abs();
    let cut_col_a = axial_force(&cut, ElemId(0)).abs();

    assert!(
        cut_brace <= base_brace * 1e-3,
        "brace force should collapse to ~0: base={base_brace} cut={cut_brace}"
    );
    assert!(
            cut_col_a > base_col_a,
            "column A axial force should increase when brace unloaded: base={base_col_a} cut={cut_col_a}"
        );
}

/// 検証2: `no_long_axial_column=true` の長期ケースでは、柱の長期軸力が
/// ほぼ0（元の1e-3倍以下）になること。同一2節点間で柱と並列に鉛直
/// ブレースを置いたモデル（`column_with_parallel_vertical_brace`）を使い、
/// 柱が負担しなくなった軸力がブレース側で健全に負担される
/// （機構化せず数値的に安定に解ける）ことも合わせて確認する。
#[test]
fn test_no_long_axial_column_zeros_column_force() {
    let mut model = column_with_parallel_vertical_brace();
    let base = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let base_col = axial_force(&base, ElemId(0)).abs();
    let base_brace = axial_force(&base, ElemId(1)).abs();
    assert!(
        base_col > 1.0,
        "baseline column force should be non-trivial: {base_col}"
    );

    model.stress_cfg.no_long_axial_column = true;
    let cut = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let cut_col = axial_force(&cut, ElemId(0)).abs();
    let cut_brace = axial_force(&cut, ElemId(1)).abs();

    assert!(
        cut_col <= base_col * 1e-3,
        "column force should collapse to ~0: base={base_col} cut={cut_col}"
    );
    // 柱が負担しなくなった軸力は並列ブレースへ移り、荷重全体はブレース単体で
    // 健全に負担される（機構化していないことの確認）。
    assert!(
            cut_brace > base_brace,
            "brace should pick up the load the column no longer carries: base={base_brace} cut={cut_brace}"
        );
    assert!(
        (cut_brace - 1.0e5).abs() / 1.0e5 < 1e-3,
        "brace should carry ~all of the applied load once column axial is disabled: {cut_brace}"
    );
}

/// 検証3: 短期（Seismic）荷重ケースでは、`no_long_axial_brace=true` でも
/// 適用されない（ブレース軸力が長期無効化なしの基準値と同程度に残ること）。
#[test]
fn test_axial_cut_not_applied_to_short_term_case() {
    let mut model = braced_frame(squid_n_core::model::LoadCaseKind::Dead);
    // 同一の鉛直荷重を持つ短期（地震）荷重ケースを追加する。
    model.load_cases.push(LoadCase {
        id: LoadCaseId(2),
        name: "seismic_gravity_dummy".into(),
        nodal: vec![NodalLoad::manual(
            NodeId(2),
            [0.0, 0.0, -1.0e5, 0.0, 0.0, 0.0],
        )],
        member: vec![],
        kind: squid_n_core::model::LoadCaseKind::Seismic,
    });
    model.stress_cfg.no_long_axial_brace = true;

    let long_term = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let short_term = linear_static_once(&model, LoadCaseId(2)).unwrap();

    let brace_long = axial_force(&long_term, ElemId(3)).abs();
    let brace_short = axial_force(&short_term, ElemId(3)).abs();

    assert!(
        brace_long < 1.0,
        "長期ケースはブレース軸力が無効化されるはず: {brace_long}"
    );
    assert!(
        brace_short > 1.0,
        "短期ケースは無効化されず通常どおりブレース軸力を負担するはず: {brace_short}"
    );
}

/// SRC柱の軸剛性を低減しても、曲げ・せん断・ねじり剛性と質量は保持する。
#[test]
fn test_axial_cut_applies_to_composite_src_column() {
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    let mut model = column_with_parallel_vertical_brace();
    // 柱断面を SRC（shape あり・コンクリート材料 fc あり）へ差し替える。
    model.sections[0].shape = Some(SectionShape::SrcRect {
        b: 600.0,
        d: 600.0,
        rebar: RcRebar {
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
            },
        },
        steel_height: 400.0,
        steel_width: 200.0,
        steel_web_thick: 9.0,
        steel_flange_thick: 12.0,
    });
    model.materials[0].fc = Some(24.0);
    model.materials[0].young = 2.27e4;

    let elem = &model.elements[0];
    let normal = build_behavior_with_axial_factor(elem, &model, 1.0);
    let reduced = build_behavior_with_axial_factor(elem, &model, AXIAL_DISABLE_FACTOR);
    let ctx = Ctx { model: &model };
    let normal_k = normal.tangent_stiffness(&ctx);
    let reduced_k = reduced.tangent_stiffness(&ctx);
    for row in 0..12 {
        for col in 0..12 {
            let factor = if [2, 8].contains(&row) && [2, 8].contains(&col) {
                AXIAL_DISABLE_FACTOR
            } else {
                1.0
            };
            let expected = normal_k.get(row, col) * factor;
            assert!(
                (reduced_k.get(row, col) - expected).abs() <= expected.abs().max(1.0) * 1e-12,
                "剛性成分 ({row}, {col})"
            );
        }
    }
    for mass in [
        squid_n_element::behavior::MassOption::Lumped,
        squid_n_element::behavior::MassOption::Consistent,
    ] {
        assert_eq!(
            normal.mass_matrix(mass).data,
            reduced.mass_matrix(mass).data
        );
    }

    let base = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let base_col = axial_force(&base, ElemId(0)).abs();
    assert!(
        base_col > 1.0,
        "SRC柱の基準軸力が有意であること: {base_col}"
    );

    model.stress_cfg.no_long_axial_column = true;
    let cut = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let cut_col = axial_force(&cut, ElemId(0)).abs();
    let cut_brace = axial_force(&cut, ElemId(1)).abs();

    assert!(
        cut_col <= base_col * 1e-3,
        "SRC柱でも軸力が無効化されるはず: base={base_col} cut={cut_col}"
    );
    assert!(
        (cut_brace - 1.0e5).abs() / 1.0e5 < 1e-3,
        "荷重はブレースが全量負担するはず: {cut_brace}"
    );
}

// 引張専用ブレースの active-set 反復（真の引張専用解析）

/// 引張専用ブレース検証用の1スパン門型フレーム。
///
/// - N0[0,0,0]・N1[L,0,0]: 基部固定
/// - N2[0,0,H]・N3[L,0,H]: 頂部自由
/// - e0/e1: 柱（鉛直 Beam, Fixed-Fixed）… ブレース無効化時の水平抵抗（曲げ）経路
/// - e2: 頂部大梁（Beam）
/// - e3: 対角ブレース N0→N3（`tension_only` は引数指定）
///
/// 頂部2節点に水平力 `fx`（+x）を与える。ブレース軸 t=(L,0,H)/len に対し、
/// +x のスウェイで N3 が N0 から離れる → 軸伸び δ>0（引張）。-x なら δ<0（圧縮）。
fn tension_only_portal(fx: f64, tension_only: bool) -> Model {
    let l = 4000.0_f64;
    let h = 3000.0_f64;
    let node = |id: u32, coord: [f64; 3], fixed: bool| Node {
        id: NodeId(id),
        coord,
        restraint: if fixed {
            Dof6Mask::FIXED
        } else {
            Dof6Mask::FREE
        },
        mass: None,
        story: None,
        support_spring: None,
    };
    let sec = Section {
        id: SectionId(0),
        name: "steel".into(),
        area: 6000.0,
        iy: 8.0e7,
        iz: 8.0e7,
        j: 1.0e6,
        depth: 300.0,
        width: 300.0,
        as_y: 5000.0,
        as_z: 5000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let mat = Material {
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
    };
    let column = |id: u32, n0: u32, n1: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(n0), NodeId(n1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    Model {
        nodes: vec![
            node(0, [0.0, 0.0, 0.0], true),
            node(1, [l, 0.0, 0.0], true),
            node(2, [0.0, 0.0, h], false),
            node(3, [l, 0.0, h], false),
        ],
        elements: vec![
            column(0, 0, 2),
            column(1, 1, 3),
            // e2: 頂部大梁
            ElementData {
                id: ElemId(2),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(2), NodeId(3)],
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            // e3: 対角ブレース N0→N3
            ElementData {
                id: ElemId(3),
                kind: ElementKind::Brace { tension_only },
                nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Pinned, EndCondition::Pinned],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![sec],
        materials: vec![mat],
        load_cases: vec![LoadCase {
            id: LoadCaseId(1),
            name: "wind".into(),
            nodal: vec![
                NodalLoad::manual(NodeId(2), [fx, 0.0, 0.0, 0.0, 0.0, 0.0]),
                NodalLoad::manual(NodeId(3), [fx, 0.0, 0.0, 0.0, 0.0, 0.0]),
            ],
            member: vec![],
            kind: squid_n_core::model::LoadCaseKind::Seismic,
        }],
        stress_cfg: squid_n_core::model::StressAnalysisCfg::default(),
        ..Default::default()
    }
}

/// 引張側の荷重（+x スウェイ）では、反復 ON の引張専用ブレースが全剛性の
/// 一般ブレース（`tension_only: false`）と厳密に一致する軸力を負担すること。
/// active なブレースは E·A/L の一般ブレースそのものになる、という等価性の検証。
#[test]
fn test_tension_only_iteration_tension_side_matches_full_brace() {
    // 参照: 一般（全剛性）ブレース
    let full = tension_only_portal(1.0e4, false);
    let ref_res = linear_static_once(&full, LoadCaseId(1)).unwrap();
    let ref_brace = axial_force(&ref_res, ElemId(3));
    assert!(
        ref_brace.abs() > 1.0,
        "参照ブレース軸力が有意であること: {ref_brace}"
    );

    // 引張専用 + 反復 ON
    let mut model = tension_only_portal(1.0e4, true);
    model.stress_cfg.tension_only_iteration = true;
    let res = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let brace = axial_force(&res, ElemId(3));

    assert!(
        (brace - ref_brace).abs() / ref_brace.abs() < 1e-6,
        "引張側では全剛性ブレースと一致するはず: full={ref_brace} to={brace}"
    );
}

/// 圧縮側の荷重（−x スウェイ）では、反復 ON の引張専用ブレースが無効化され
/// 軸力がほぼ0になること。一方、反復 OFF（一括解析）では全剛性 E·A/L のまま
/// 圧縮の軸力を負担すること。
#[test]
fn test_tension_only_iteration_compression_side_is_slack() {
    // 反復 OFF（既定の一括解析）: 全剛性 E·A/L で圧縮を負担する
    let base = tension_only_portal(-1.0e4, true);
    assert!(!base.stress_cfg.tension_only_iteration);
    let base_res = linear_static_once(&base, LoadCaseId(1)).unwrap();
    let base_brace = axial_force(&base_res, ElemId(3));
    assert!(
        base_brace.abs() > 1.0,
        "反復 OFF では圧縮軸力が有意であること: {base_brace}"
    );
    assert!(base_brace < 0.0, "圧縮（負）であること: {base_brace}");

    // 反復 ON: 圧縮ブレースは無効化され軸力ほぼ0
    let mut model = tension_only_portal(-1.0e4, true);
    model.stress_cfg.tension_only_iteration = true;
    let res = linear_static_once(&model, LoadCaseId(1)).unwrap();
    let brace = axial_force(&res, ElemId(3));

    assert!(
        brace.abs() <= base_brace.abs() * 1e-3,
        "圧縮側では軸力が0へ落ちるはず: base={base_brace} to={brace}"
    );
}

/// 剛床に載る梁を持つ 1 層門型ラーメン（柱 ElemId 0,1／梁 ElemId 2）。
/// 梁の全長に等分布荷重 w=10 N/mm（鉛直下向き）をかける。
///
/// `with_rigid_floor` が true のとき、階（`Story`）に剛床を定義する。
/// 自由度の拘束（`constraints`）は付けないため解は変わらないはずで、
/// 剛床の有無で応力解析の結果が変わってはいけないことの検証に使う
/// （`ForceRegime::Auto` の剛床判定が線形解析の要素種別へ漏れない）。
fn rigid_floor_portal(with_rigid_floor: bool) -> Model {
    use squid_n_core::model::Story;
    let l = 6000.0_f64;
    let h = 3500.0_f64;
    let mut model = Model {
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
                coord: [l, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(2),
                coord: [0.0, 0.0, h],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(3),
                coord: [l, 0.0, h],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![],
        sections: vec![Section {
            id: SectionId(0),
            name: "s".into(),
            area: 1.0e4,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e6,
            depth: 400.0,
            width: 200.0,
            as_y: 4.0e3,
            as_z: 4.0e3,
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
            name: "SN400B".into(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(235.0),
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "udl".into(),
            nodal: vec![],
            member: vec![MemberLoad::manual(
                ElemId(2),
                [0.0, 0.0, -1.0],
                MemberLoadKind::Distributed {
                    a: 0.0,
                    b: l,
                    w1: 10.0,
                    w2: 10.0,
                },
            )],
        }],
        ..Default::default()
    };
    let mut push = |id: u32, i: u32, j: u32, ref_vector: [f64; 3]| {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(i), NodeId(j)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis { ref_vector },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    };
    push(0, 0, 2, [1.0, 0.0, 0.0]);
    push(1, 1, 3, [1.0, 0.0, 0.0]);
    push(2, 2, 3, [0.0, 0.0, 1.0]);

    if with_rigid_floor {
        // 梁の材端節点が剛床上にあることだけを表す剛床（スレーブなし）。
        // 自由度は何も拘束しないので、線形解析の結果は剛床の有無で変わらないはず
        // であり、要素種別が剛床判定に引きずられていないかを切り分けられる。
        model
            .constraints
            .push(squid_n_core::model::Constraint::rigid_diaphragm(
                StoryId(0),
                NodeId(2),
                Vec::new(),
            ));
        model.stories.push(Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(0),
            name: "2F".into(),
            elevation: h,
            node_ids: vec![NodeId(2), NodeId(3)],
            seismic_weight: None,
            weight_override: None,
        });
    }
    model
}

///
/// `ForceRegime::Auto` は「剛床に載る水平材」を材端集中ばね（非線形要素）へ
/// 振り分けるが、これは非線形解析だけの規則である。線形解析の要素生成が
/// この振り分けに従うと、材端集中ばね梁が `recover_forces` を持たないため
/// 当該梁が `member_forces` から丸ごと欠落し、応力図・検定比図に
/// 何も表示されなくなる（RC 基礎梁など剛床に載らない梁だけが残る）。
#[test]
fn test_rigid_floor_beam_member_forces_are_recovered() {
    let model = rigid_floor_portal(true);
    let res = linear_static_once(&model, LoadCaseId(1)).unwrap();

    for id in [ElemId(0), ElemId(1), ElemId(2)] {
        assert!(
            res.member_forces.iter().any(|(e, _)| *e == id),
            "部材 {:?} の内力が回収されていない: 回収済み={:?}",
            id,
            res.member_forces
                .iter()
                .map(|(e, _)| e.0)
                .collect::<Vec<_>>()
        );
    }

    // 梁の曲げは両端固定に近い分布（中央 sagging・端部 hogging）で、
    // 単純梁の中央曲げ wL²/8 を超えない有意な値になる。
    let beam = res
        .member_forces
        .iter()
        .find(|(e, _)| *e == ElemId(2))
        .map(|(_, mf)| mf)
        .expect("梁の内力");
    let m_mid = beam
        .at
        .iter()
        .find(|(xi, _)| (xi - 0.5).abs() < 1e-9)
        .map(|(_, f)| f[5])
        .expect("中央の内力");
    let w = 10.0_f64;
    let l = 6000.0_f64;
    let m_simple = w * l * l / 8.0;
    assert!(
        m_mid.abs() > m_simple * 0.1 && m_mid.abs() < m_simple,
        "梁中央の曲げが妥当な範囲にない: M={m_mid} (wL²/8={m_simple})"
    );
}

/// 自由度を拘束しない剛床（スレーブなしの `Constraint::RigidDiaphragm`）を加えても、
/// 線形解析の結果が一切変わらないこと。要素種別が `ForceRegime` の剛床判定に依存すると、
/// 材端ばねが直列に入って梁の曲げ剛性が落ち、応力・変形が変わってしまう。
#[test]
fn test_rigid_floor_definition_does_not_change_linear_result() {
    let without = linear_static_once(&rigid_floor_portal(false), LoadCaseId(1)).unwrap();
    let with = linear_static_once(&rigid_floor_portal(true), LoadCaseId(1)).unwrap();

    assert_eq!(without.member_forces.len(), with.member_forces.len());
    for ((id_a, mf_a), (id_b, mf_b)) in without.member_forces.iter().zip(&with.member_forces) {
        assert_eq!(id_a, id_b);
        assert_eq!(mf_a.at.len(), mf_b.at.len());
        for ((xi_a, fa), (xi_b, fb)) in mf_a.at.iter().zip(&mf_b.at) {
            assert!((xi_a - xi_b).abs() < 1e-12);
            for k in 0..6 {
                assert!(
                    (fa[k] - fb[k]).abs() <= fa[k].abs() * 1e-12 + 1e-6,
                    "部材 {:?} xi={xi_a} 成分{k} が剛床定義の有無で変化: {} vs {}",
                    id_a,
                    fa[k],
                    fb[k]
                );
            }
        }
    }
    for (ua, ub) in without.disp.iter().zip(&with.disp) {
        for k in 0..6 {
            assert!(
                (ua[k] - ub[k]).abs() <= ua[k].abs() * 1e-12 + 1e-12,
                "節点変位が剛床定義の有無で変化: {} vs {}",
                ua[k],
                ub[k]
            );
        }
    }
}

/// 線材の部材内力が回収できないモデルは、黙って結果から欠落させず解析エラーにする
/// （[`ensure_line_member_forces`]）。要素実装が `recover_forces` を持たないと、
/// 応力図・断面検定・接合部検定の入力から当該部材が無言で消えるため。
#[test]
fn test_ensure_line_member_forces_detects_missing() {
    let model = rigid_floor_portal(true);
    let res = linear_static_once(&model, LoadCaseId(1)).unwrap();

    // 全線材が揃っていれば OK。
    assert!(ensure_line_member_forces(&model, &res.member_forces).is_ok());

    // 梁 ElemId(2) の内力が欠落した状態を模擬するとエラーになり、
    // メッセージに欠落した部材 ID が含まれる。
    let missing: Vec<_> = res
        .member_forces
        .iter()
        .filter(|(id, _)| *id != ElemId(2))
        .cloned()
        .collect();
    let err = ensure_line_member_forces(&model, &missing)
        .expect_err("欠落を検出できていない")
        .to_string();
    assert!(err.contains('2'), "欠落した部材 ID が示されていない: {err}");
}
