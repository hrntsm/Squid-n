use super::*;
use approx::assert_relative_eq;
use squid_n_core::ids::{MaterialId, SectionId};
use squid_n_core::model::{Material, MaterialCategory, Section};
use squid_n_material::{Bilinear, Concrete, UniaxialMaterial};
use squid_n_section::fiber::rect_fiber_section;

fn make_section(w: f64, d: f64) -> Section {
    Section {
        id: SectionId(0),
        name: "test".into(),
        area: w * d,
        iy: w * d.powi(3) / 12.0,
        iz: d * w.powi(3) / 12.0,
        j: w.powi(3) * d / 3.0,
        depth: d,
        width: w,
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
    }
}

#[test]
fn test_member_skeleton_generic_basic() {
    let sec = make_section(100.0, 200.0);
    let mat_data = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: squid_n_core::ids::MaterialId(0),
        name: "steel".into(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: None,
    };
    let fibers = rect_fiber_section(100.0, 200.0, 10, 20, 0);
    let reinforcement = Reinforcement {
        main_bars: vec![],
        hoop_pitch: 100.0,
        hoop_area: 0.0,
    };
    let member = MemberData {
        section: &sec,
        reinforcement: &reinforcement,
        material: &mat_data,
        fibers: &fibers,
        span: 4000.0,
        inflection_ratio: 0.5,
    };
    let template = Bilinear::new(205000.0, 235.0, 0.01);
    let mut mats: Vec<Box<dyn UniaxialMaterial>> = (0..fibers.fibers.len())
        .map(|_| template.clone_box())
        .collect();
    let skeleton = build_member_skeleton(&member, 0.0, &mut mats, 0.4);
    assert!(!skeleton.points.is_empty());
    assert!(skeleton.points.last().unwrap().1 >= skeleton.points.first().unwrap().1);
}

#[test]
fn test_rc_skeleton_yield_matches_handcalc() {
    // 代表 RC 梁: b=300, D=500, 引張鉄筋 4-D19 (As≈4×283.5=1134 mm²), fy=345, E=200000
    // 手計算 My = at·σy·j, j=7d/8, d=D-cover-φ/2 = 500-50-9.5 = 440.5
    let b = 300.0;
    let d_total = 500.0;
    let cover = 50.0;
    let bar_dia: f64 = 19.0;
    let n_bars = 4;
    let as_bar: f64 = std::f64::consts::PI * (bar_dia / 2.0).powi(2);
    let at = n_bars as f64 * as_bar;
    let d = d_total - cover - bar_dia / 2.0;
    let j = 7.0 * d / 8.0;
    let fy = 345.0;
    let e_steel = 200000.0;
    let my_handcalc = at * fy * j; // [N·mm]

    let sec = make_section(b, d_total);
    // 引張鉄筋位置: 上端側 z = +(d - D/2) = +190.5（正曲率 ky>0 で上端が引張となる符号規約）
    let z_tension = d - d_total / 2.0;
    let rebar = Reinforcement {
        main_bars: (0..n_bars)
            .map(|i| {
                let y = (i as f64 - (n_bars as f64 - 1.0) / 2.0) * (b - 2.0 * cover)
                    / (n_bars as f64 - 1.0).max(1.0);
                (y, z_tension, as_bar)
            })
            .collect(),
        hoop_pitch: 100.0,
        hoop_area: 0.0,
    };
    let concrete = Concrete::new(30.0, 2.0);
    let steel = Bilinear::new(e_steel, fy, 0.01);
    let opts = SkeletonOptions {
        span: 4000.0,
        inflection_ratio: 0.5,
        n_axial: 0.0,
        alpha: 0.4,
    };
    let skeleton = build_rc_member_skeleton(
        &sec,
        &rebar,
        &concrete,
        &steel,
        &opts,
        &ShearContribution::none(),
        &PulloutContribution::none(),
    );

    // 降伏点のモーメント（points[2]）が手計算と概ね一致（離散化・j近似で 15% 以内）
    let my_fiber = skeleton.points.get(2).map(|p| p.1).unwrap_or(0.0);
    let ratio = my_fiber / my_handcalc;
    assert!(
        ratio > 0.85 && ratio < 1.15,
        "My fiber ({:.3} N·mm) vs handcalc ({:.3}): ratio={:.3}",
        my_fiber,
        my_handcalc,
        ratio
    );
}

#[test]
fn test_rc_skeleton_trilinear_shape() {
    let sec = make_section(300.0, 500.0);
    let rebar = Reinforcement {
        main_bars: vec![
            (0.0, 190.0, 283.5),
            (-90.0, 190.0, 283.5),
            (90.0, 190.0, 283.5),
        ],
        hoop_pitch: 100.0,
        hoop_area: 0.0,
    };
    let concrete = Concrete::new(30.0, 2.0);
    let steel = Bilinear::new(200000.0, 345.0, 0.01);
    let opts = SkeletonOptions {
        span: 4000.0,
        inflection_ratio: 0.5,
        n_axial: 0.0,
        alpha: 0.4,
    };
    let skeleton = build_rc_member_skeleton(
        &sec,
        &rebar,
        &concrete,
        &steel,
        &opts,
        &ShearContribution::none(),
        &PulloutContribution::none(),
    );

    // 4 点（原点+3折点）で昇順
    assert_eq!(skeleton.points.len(), 4);
    for w in skeleton.points.windows(2) {
        assert!(w[0].0 <= w[1].0, "theta must be ascending");
        assert!(w[0].1 <= w[1].1 + 1e-6, "M must be non-decreasing");
    }
    // ひび割れ < 降伏 < 終局
    assert!(skeleton.points[1].1 < skeleton.points[2].1);
    assert!(skeleton.points[2].1 < skeleton.points[3].1);
}

#[test]
fn test_rc_skeleton_axial_dependency() {
    let sec = make_section(300.0, 500.0);
    let rebar = Reinforcement {
        main_bars: vec![(0.0, 190.0, 283.5)],
        hoop_pitch: 100.0,
        hoop_area: 0.0,
    };
    let concrete = Concrete::new(30.0, 2.0);
    let steel = Bilinear::new(200000.0, 345.0, 0.01);
    let opts0 = SkeletonOptions {
        span: 4000.0,
        inflection_ratio: 0.5,
        n_axial: 0.0,
        alpha: 0.4,
    };
    let opts1 = SkeletonOptions {
        n_axial: -500_000.0, // 圧縮軸力
        ..opts0
    };
    let sk_n0 = build_rc_member_skeleton(
        &sec,
        &rebar,
        &concrete,
        &steel,
        &opts0,
        &ShearContribution::none(),
        &PulloutContribution::none(),
    );
    let sk_n1 = build_rc_member_skeleton(
        &sec,
        &rebar,
        &concrete,
        &steel,
        &opts1,
        &ShearContribution::none(),
        &PulloutContribution::none(),
    );
    // 軸力により降伏モーメントが変化する
    let my_n0 = sk_n0.points[2].1;
    let my_n1 = sk_n1.points[2].1;
    assert!(
        (my_n0 - my_n1).abs() / my_n0.max(1.0) > 1e-3,
        "axial force should change My: N0={}, N1={}",
        my_n0,
        my_n1
    );
}

#[test]
fn test_rc_skeleton_shear_contribution_increases_rotation() {
    // せん断変形を加えると降伏回転角 θy が増加する（M は同一）。
    let sec = make_section(300.0, 500.0);
    let rebar = Reinforcement {
        main_bars: vec![
            (0.0, 190.0, 283.5),
            (-90.0, 190.0, 283.5),
            (90.0, 190.0, 283.5),
        ],
        hoop_pitch: 100.0,
        hoop_area: 0.0,
    };
    let concrete = Concrete::new(30.0, 2.0);
    let steel = Bilinear::new(200000.0, 345.0, 0.01);
    let opts = SkeletonOptions {
        span: 4000.0,
        inflection_ratio: 0.5,
        n_axial: 0.0,
        alpha: 0.4,
    };
    let sk_no_shear = build_rc_member_skeleton(
        &sec,
        &rebar,
        &concrete,
        &steel,
        &opts,
        &ShearContribution::none(),
        &PulloutContribution::none(),
    );
    let sk_with_shear = build_rc_member_skeleton(
        &sec,
        &rebar,
        &concrete,
        &steel,
        &opts,
        &ShearContribution::rc_rect(300.0, 500.0, &concrete),
        &PulloutContribution::none(),
    );
    let theta_y_no = sk_no_shear.points[2].0;
    let theta_y_with = sk_with_shear.points[2].0;
    assert!(
        theta_y_with > theta_y_no,
        "shear contribution must increase θy: no={}, with={}",
        theta_y_no,
        theta_y_with
    );
    // M は同一（せん断は変形のみ加算）
    let my_no = sk_no_shear.points[2].1;
    let my_with = sk_with_shear.points[2].1;
    assert_relative_eq!(my_no, my_with, epsilon = 1e-3);
}

#[test]
fn test_rc_skeleton_pullout_contribution_increases_rotation() {
    // 鉄筋抜出しを加えると降伏回転角 θy が増加する。
    let sec = make_section(300.0, 500.0);
    let rebar = Reinforcement {
        main_bars: vec![
            (0.0, 190.0, 283.5),
            (-90.0, 190.0, 283.5),
            (90.0, 190.0, 283.5),
        ],
        hoop_pitch: 100.0,
        hoop_area: 0.0,
    };
    let concrete = Concrete::new(30.0, 2.0);
    let steel = Bilinear::new(200000.0, 345.0, 0.01);
    let opts = SkeletonOptions {
        span: 4000.0,
        inflection_ratio: 0.5,
        n_axial: 0.0,
        alpha: 0.4,
    };
    let pullout = PulloutContribution {
        bar_diameter: 19.0,
        e_s: 200000.0,
        fy: 345.0,
        bond_coeff: 9.0,
    };
    let sk_no = build_rc_member_skeleton(
        &sec,
        &rebar,
        &concrete,
        &steel,
        &opts,
        &ShearContribution::none(),
        &PulloutContribution::none(),
    );
    let sk_with = build_rc_member_skeleton(
        &sec,
        &rebar,
        &concrete,
        &steel,
        &opts,
        &ShearContribution::none(),
        &pullout,
    );
    assert!(
        sk_with.points[2].0 > sk_no.points[2].0,
        "pullout must increase θy: no={}, with={}",
        sk_no.points[2].0,
        sk_with.points[2].0
    );
}

#[test]
fn test_rc_skeleton_ultimate_matches_handcalc() {
    // 終局モーメント Mu が規準式 Mu ≈ a_t·σy·j（引張鉄筋降伏型、係数 0.9 系）と照合。
    // 降伏型破壊（a_t が少なめ）の RC 梁で Mu は My の 1.0〜1.2 倍程度。
    // 規準式: Mu = 0.9·a_t·σy·j （AIJ『非線形解析指針』等の簡易式）
    let b = 300.0;
    let d_total = 500.0;
    let cover = 50.0;
    let bar_dia: f64 = 19.0;
    let n_bars = 4;
    let as_bar: f64 = std::f64::consts::PI * (bar_dia / 2.0).powi(2);
    let at = n_bars as f64 * as_bar;
    let d = d_total - cover - bar_dia / 2.0;
    let j = 7.0 * d / 8.0;
    let fy = 345.0;
    let mu_handcalc = 0.9 * at * fy * j; // [N·mm]

    let sec = make_section(b, d_total);
    let z_tension = d - d_total / 2.0;
    let rebar = Reinforcement {
        main_bars: (0..n_bars)
            .map(|i| {
                let y = (i as f64 - (n_bars as f64 - 1.0) / 2.0) * (b - 2.0 * cover)
                    / (n_bars as f64 - 1.0).max(1.0);
                (y, z_tension, as_bar)
            })
            .collect(),
        hoop_pitch: 100.0,
        hoop_area: 0.0,
    };
    let concrete = Concrete::new(30.0, 2.0);
    let steel = Bilinear::new(200000.0, fy, 0.01);
    let opts = SkeletonOptions {
        span: 4000.0,
        inflection_ratio: 0.5,
        n_axial: 0.0,
        alpha: 0.4,
    };
    let skeleton = build_rc_member_skeleton(
        &sec,
        &rebar,
        &concrete,
        &steel,
        &opts,
        &ShearContribution::none(),
        &PulloutContribution::none(),
    );

    let mu_fiber = skeleton.points.get(3).map(|p| p.1).unwrap_or(0.0);
    let ratio = mu_fiber / mu_handcalc;
    // 0.9·a_t·σy·j は近似式。ファイバ積分は圧縮側コンクリートも寄与するため
    // Mu は My の 1.0〜1.3 倍程度。規準式との一致は 30% 以内を許容。
    assert!(
        ratio > 0.7 && ratio < 1.3,
        "Mu fiber ({:.3} N·mm) vs handcalc 0.9·at·σy·j ({:.3}): ratio={:.3}",
        mu_fiber,
        mu_handcalc,
        ratio
    );
}

#[test]
fn test_rc_skeleton_mu_greater_than_my() {
    // 降伏型 RC では Mu >= My（降伏後もわずかに耐力上昇）。
    let sec = make_section(300.0, 500.0);
    let rebar = Reinforcement {
        main_bars: vec![
            (0.0, 190.0, 283.5),
            (-90.0, 190.0, 283.5),
            (90.0, 190.0, 283.5),
        ],
        hoop_pitch: 100.0,
        hoop_area: 0.0,
    };
    let concrete = Concrete::new(30.0, 2.0);
    let steel = Bilinear::new(200000.0, 345.0, 0.01);
    let opts = SkeletonOptions {
        span: 4000.0,
        inflection_ratio: 0.5,
        n_axial: 0.0,
        alpha: 0.4,
    };
    let skeleton = build_rc_member_skeleton(
        &sec,
        &rebar,
        &concrete,
        &steel,
        &opts,
        &ShearContribution::none(),
        &PulloutContribution::none(),
    );
    let my = skeleton.points[2].1;
    let mu = skeleton.points[3].1;
    assert!(
        mu >= my - 1e-6,
        "Mu ({}) must be >= My ({}) for yield-type RC",
        mu,
        my
    );
}
