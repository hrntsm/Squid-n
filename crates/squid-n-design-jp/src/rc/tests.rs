use super::*;
use squid_n_core::ids::{MaterialId, SectionId};
use squid_n_core::model::MaterialCategory;
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

pub(crate) fn make_material(fc: f64, grade: &str) -> Material {
    Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: grade.to_string(),
        category: MaterialCategory::Concrete,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: Some(fc),
        fy: None,
    }
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn rc_rect_shape(
    b: f64,
    d: f64,
    main_count: u32,
    main_dia: f64,
    main_layers: u32,
    cover: f64,
    shear_dia: f64,
    shear_pitch: f64,
    shear_legs: u32,
) -> SectionShape {
    SectionShape::RcRect {
        b,
        d,
        rebar: RcRebar {
            main_x: BarSet {
                count: main_count,
                dia: main_dia,
                layers: main_layers,
            },
            main_y: BarSet {
                count: main_count,
                dia: main_dia,
                layers: main_layers,
            },
            cover,
            shear: ShearBar {
                dia: shear_dia,
                pitch: shear_pitch,
                legs: shear_legs,
            },
        },
    }
}

pub(crate) fn make_section(shape: SectionShape) -> Section {
    shape.to_section(SectionId(0), "test".to_string())
}

/// 鉄筋の材料（材料名がグレード名、`fy` が降伏点）。
pub(crate) fn make_rebar_material(grade: &str, fy: f64) -> Material {
    Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(1),
        name: grade.to_string(),
        category: MaterialCategory::Rebar,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(fy),
    }
}

/// RC 検定の標準の材料構成（主筋 SD345・せん断補強筋 SD295A）。
/// 材料は断面が持つため、検定コンテキストへ渡して解決させる。
fn ctx_materials(ctx: DesignCtx) -> DesignCtx {
    DesignCtx {
        rebar_material: Some(make_rebar_material("SD345", 345.0)),
        shear_rebar_material: Some(make_rebar_material("SD295A", 295.0)),
        ..ctx
    }
}

pub(crate) fn ctx_beam(term: LoadTerm) -> DesignCtx {
    ctx_materials(DesignCtx {
        term,
        kind: MemberKind::Beam,
        ..Default::default()
    })
}

pub(crate) fn ctx_column(term: LoadTerm) -> DesignCtx {
    ctx_materials(DesignCtx {
        term,
        kind: MemberKind::Column,
        ..Default::default()
    })
}

// 地震時短期の設計用せん断力 QD = min(QD1, QD2)

#[test]
fn test_seismic_design_shear_min_of_qd1_qd2() {
    use crate::{QdMethod, SeismicQd};
    let mut ctx = DesignCtx {
        seismic_qd: Some(SeismicQd {
            long_at: vec![(0.0, [0.0, 50_000.0, 0.0, 0.0, 0.0, 0.0])],
            n_factor: 1.5,
            n_mechanism: 1.0,
            q_simple: None, // 未算定 → QL で代替
            clear_length: 4000.0,
            method: QdMethod::Min,
        }),
        ..Default::default()
    };
    // 当該組合せ Q=150kN、QL=50kN → QE=100kN、QD2 = 50+1.5×100 = 200kN。
    // ΣMy=400kN·m、Q0 未算定 → 梁 QD1 = QL+ΣMy/l′ = 50+400e6/4000 = 150kN → min = 150kN。
    let q_beam = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 400.0e6, false);
    assert!((q_beam - 150_000.0).abs() < 1e-6, "q_beam={q_beam}");
    // Q0=80kN を与えると梁 QD1 = 80+100 = 180kN → min(180, 200) = 180kN。
    ctx.seismic_qd.as_mut().unwrap().q_simple = Some(80_000.0);
    let q_beam_q0 = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 400.0e6, false);
    assert!(
        (q_beam_q0 - 180_000.0).abs() < 1e-6,
        "q_beam_q0={q_beam_q0}"
    );
    ctx.seismic_qd.as_mut().unwrap().q_simple = None;
    // 柱 QD1 = ΣcMy/h′ = 100kN（Q0/QL を加算しない）→ min(100, 200) = 100kN。
    let q_col = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 400.0e6, true);
    assert!((q_col - 100_000.0).abs() < 1e-6, "q_col={q_col}");
    // QD2 単独選択。
    ctx.seismic_qd.as_mut().unwrap().method = QdMethod::Qd2;
    let q2 = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 400.0e6, false);
    assert!((q2 - 200_000.0).abs() < 1e-6, "q2={q2}");
    // ΣMy<=0（終局曲げ不明）のとき QD1 は無効で QD2 のみ。
    ctx.seismic_qd.as_mut().unwrap().method = QdMethod::Min;
    let q_no_mu = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 0.0, false);
    assert!((q_no_mu - 200_000.0).abs() < 1e-6);
    // 評価位置が長期内力にない場合・文脈なしの場合は解析値のまま。
    let q_missing = seismic_design_shear(&ctx, 0.5, 150_000.0, 1, 400.0e6, false);
    assert!((q_missing - 150_000.0).abs() < 1e-6);
    ctx.seismic_qd = None;
    let q_none = seismic_design_shear(&ctx, 0.0, 150_000.0, 1, 400.0e6, false);
    assert!((q_none - 150_000.0).abs() < 1e-6);
}

// 許容応力度（RC 造検定でのみ使う独自カバレッジ分。他は material_strength.rs 側で検証）

#[test]
fn test_rebar_allowable_tension_table() {
    assert!((rebar_allowable_tension("SR235", 16.0, true) - 155.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SR235", 16.0, false) - 235.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SR295", 16.0, true) - 155.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SR295", 16.0, false) - 295.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SD295A", 16.0, true) - 195.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SD390", 22.0, true) - 215.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SD390", 32.0, true) - 195.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SD390", 22.0, false) - 390.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("SD490", 22.0, false) - 490.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("UNKNOWN", 22.0, true) - 195.0).abs() < 1e-9);
    assert!((rebar_allowable_tension("UNKNOWN", 22.0, false) - 295.0).abs() < 1e-9);
}

#[test]
fn test_rebar_allowable_shear_table() {
    assert!((rebar_allowable_shear("SR235", true) - 155.0).abs() < 1e-9);
    // 短期は基準強度 F=235（令90条表）。フォールバック 295 に落ちて F 値を
    // 超過していた回帰の防止。
    assert!((rebar_allowable_shear("SR235", false) - 235.0).abs() < 1e-9);
    assert!((rebar_allowable_shear("SR295", false) - 295.0).abs() < 1e-9);
    assert!((rebar_allowable_shear("SD345", true) - 195.0).abs() < 1e-9);
    assert!((rebar_allowable_shear("SD295A", false) - 295.0).abs() < 1e-9);
    assert!((rebar_allowable_shear("SD345", false) - 345.0).abs() < 1e-9);
    assert!((rebar_allowable_shear("SD390", false) - 390.0).abs() < 1e-9);
    assert!((rebar_allowable_shear("SD490", false) - 390.0).abs() < 1e-9);
    assert!((rebar_allowable_shear("UNKNOWN", false) - 295.0).abs() < 1e-9);
}

// せん断スパン比 α・せん断耐力（普通強度せん断補強筋）

#[test]
fn test_shear_alpha_formula_and_clamps() {
    let d = 500.0;
    let q = 100_000.0;
    // (M/(Q・d), max_alpha, 期待 α)。M/(Q・d)=1 は上限、3 は下限に一致。
    let cases = [
        (1.0, 2.0, 2.0),
        (2.0, 2.0, 4.0 / 3.0),
        (3.0, 2.0, 1.0),
        (0.0, 2.0, 2.0),  // 素の 4.0 → 梁の上限 2.0
        (10.0, 2.0, 1.0), // 素の約 0.364 → 下限 1.0
        (0.0, 1.5, 1.5),  // 柱の上限 1.5
    ];
    for (mqd, max_alpha, expected) in cases {
        let alpha = shear_alpha(q * d * mqd, q, d, max_alpha);
        assert!(
            (alpha - expected).abs() < 1e-9,
            "M/(Q・d)={mqd}, max={max_alpha}: alpha={alpha}, expected={expected}"
        );
    }
}

#[test]
fn test_pw_ratio_capped_long_term() {
    // 過大なせん断補強筋比を作り、長期は 0.6% に制限されることを確認する。
    let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 13.0, 30.0, 4);
    let rebar = match &shape {
        SectionShape::RcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
    assert!(props.pw > 0.006, "テストの前提として pw > 0.6% が必要");

    let allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", true);
    let alpha = 1.5;
    let qa_capped = shear_capacity(&props, &allow, alpha, LoadTerm::Long, true, false);

    // 手計算: pw を 0.6% に制限した式と一致すること。
    let pw_term = 0.5 * allow.w_ft * (0.006 - 0.002);
    let expected = props.b * props.j * (alpha * allow.fs + pw_term);
    assert!((qa_capped - expected).abs() / expected < 1e-6);
}

#[test]
fn test_beam_shear_damage_control_vs_safety() {
    let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
    let rebar = match &shape {
        SectionShape::RcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
    let allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);
    let alpha = 1.4;

    let qa_damage = shear_capacity(&props, &allow, alpha, LoadTerm::Short, true, false);
    let qa_safety = shear_capacity(&props, &allow, alpha, LoadTerm::Short, false, false);

    let pw_term = if props.pw < 0.002 {
        0.0
    } else {
        0.5 * allow.w_ft * (props.pw.min(0.012) - 0.002)
    };
    let expected_damage = props.b * props.j * ((2.0 / 3.0) * alpha * allow.fs + pw_term);
    let expected_safety = props.b * props.j * (alpha * allow.fs + pw_term);

    assert!((qa_damage - expected_damage).abs() / expected_damage < 1e-6);
    assert!((qa_safety - expected_safety).abs() / expected_safety < 1e-6);
    assert!(
        qa_damage < qa_safety,
        "損傷制御式は安全確保式より小さいはず"
    );

    // shear_capacity_for は普通強度式へ委譲する（分岐反転の回帰防止）。
    // 実運用の既定は損傷制御（true）のため、true は損傷制御式
    // b・j・(2/3・α・fs + pw項)、false は安全確保式 b・j・(α・fs + pw項) と照合する。
    for (damage_control, expected) in [(true, expected_damage), (false, expected_safety)] {
        let qa_via_dispatch = shear_capacity_for(
            &props,
            &allow,
            alpha,
            LoadTerm::Short,
            damage_control,
            false,
        );
        assert!(
            (qa_via_dispatch - expected).abs() / expected < 1e-9,
            "damage_control={damage_control}: QA={qa_via_dispatch}, expected={expected}"
        );
    }
}

#[test]
fn test_column_safety_check_excludes_alpha() {
    let shape = rc_rect_shape(400.0, 400.0, 8, 22.0, 2, 40.0, 10.0, 100.0, 2);
    let rebar = match &shape {
        SectionShape::RcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = rect_axis_props_strong(&make_section(shape), &rebar);

    // 柱の「安全確保のための検討」式は α を含まない。普通強度せん断補強筋で、
    // 手計算の期待値 b・j・(fs + 0.5・w_ft・(pw − 0.002)) と照合する。
    // fs = min(24/30, 0.49+24/100)×1.5 = 1.095、pw ≈ 0.003927。
    let fs = 1.095;
    let pw = props.pw;
    assert!(pw > 0.002, "テストの前提として pw > 0.002 が必要: pw={pw}");
    let b_j = props.b * props.j;
    let mut allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);
    let w_ft = allow.w_ft;
    let pw_term = 0.5 * w_ft * (pw.min(0.012) - 0.002);
    let expected = b_j * (fs + pw_term);
    let qa_alpha_1 = shear_capacity_for(&props, &allow, 1.0, LoadTerm::Short, false, true);
    let qa_alpha_1_5 = shear_capacity_for(&props, &allow, 1.5, LoadTerm::Short, false, true);
    assert!(
        (qa_alpha_1 - expected).abs() / expected < 1e-9,
        "QA={qa_alpha_1}, expected={expected}"
    );
    assert!(
        (qa_alpha_1 - qa_alpha_1_5).abs() < 1e-6,
        "α を変えても安全確保式は変化しない"
    );

    // 損傷制御式は α に依存するため異なる値になる。
    allow.w_ft = w_ft;
    let qa_damage_1 = shear_capacity(&props, &allow, 1.0, LoadTerm::Short, true, true);
    let qa_damage_1_5 = shear_capacity(&props, &allow, 1.5, LoadTerm::Short, true, true);
    assert!((qa_damage_1 - qa_damage_1_5).abs() > 1e-6);
}

#[test]
fn test_column_long_term_shear_has_no_rebar_term() {
    let shape = rc_rect_shape(400.0, 400.0, 8, 22.0, 2, 40.0, 10.0, 60.0, 4);
    let rebar = match &shape {
        SectionShape::RcRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = rect_axis_props_strong(&make_section(shape), &rebar);
    let allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", true);
    let alpha = 1.3;
    let qal = shear_capacity(&props, &allow, alpha, LoadTerm::Long, true, true);
    let expected = props.b * props.j * alpha * allow.fs;
    assert!((qal - expected).abs() / expected < 1e-9);
}

// フォールバック・RcDesign 統合（振り分け全体の確認）

#[test]
fn test_fc_missing_fallback() {
    let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
    let sec = make_section(shape);
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SD345".to_string(),
        category: MaterialCategory::Rebar,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    };
    let ctx = ctx_beam(LoadTerm::Long);
    let forces = MemberForcesAt {
        pos: 0.0,
        n: 0.0,
        qy: 0.0,
        qz: 0.0,
        my: 0.0,
        mz: 0.0,
    };
    let design = RcDesign;
    let outcome = design.check(&forces, &sec, &mat, &ctx);
    match outcome {
        CheckOutcome::Skipped { reason } => assert!(reason.contains("Fc")),
        CheckOutcome::Checked(_) => panic!("Fc 未設定は検定不能(Skipped)のはず"),
    }
}

#[test]
fn test_shape_missing_fallback() {
    // shape を持たない Section（数値直入力等）。
    let sec = Section {
        id: SectionId(0),
        name: "no-shape".to_string(),
        area: 300.0 * 600.0,
        iy: 1.0,
        iz: 1.0,
        j: 1.0,
        depth: 600.0,
        width: 300.0,
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
    };
    let mat = make_material(24.0, "SD345");
    let ctx = ctx_beam(LoadTerm::Long);
    let forces = MemberForcesAt {
        pos: 0.0,
        n: 0.0,
        qy: 0.0,
        qz: 0.0,
        my: 0.0,
        mz: 0.0,
    };
    let design = RcDesign;
    let outcome = design.check(&forces, &sec, &mat, &ctx);
    match outcome {
        CheckOutcome::Skipped { reason } => assert!(reason.contains("配筋情報なし")),
        CheckOutcome::Checked(_) => panic!("配筋情報なしは検定不能(Skipped)のはず"),
    }
}

/// せん断補強筋に未対応グレード（KH785）を割り当てた RC 部材は、普通強度として
/// 検定せずに理由付きで検定不能とする。
#[test]
fn test_rc_unsupported_shear_rebar_grade_skip() {
    let sec = make_section(rc_rect_shape(
        300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2,
    ));
    let mat = make_material(24.0, "SD345");
    let forces = MemberForcesAt {
        pos: 0.0,
        n: 0.0,
        qy: 0.0,
        qz: 0.0,
        my: 0.0,
        mz: 0.0,
    };
    let ctx = DesignCtx {
        shear_rebar_material: Some(make_rebar_material("KH785", 785.0)),
        ..ctx_beam(LoadTerm::Short)
    };
    let design = RcDesign;
    let outcome = design.check(&forces, &sec, &mat, &ctx);
    match outcome {
        CheckOutcome::Skipped { reason } => {
            assert!(reason.contains("KH785"), "{reason}");
            assert!(
                reason.contains("SR235・SR295・SD295・SD345・SD390・SD490"),
                "{reason}"
            );
        }
        CheckOutcome::Checked(_) => panic!("未対応グレードは検定不能(Skipped)のはず"),
    }
}

/// 対応グレード（SR235）でもせん断補強筋の fy が未設定なら検定不能とする。
/// fy を設定すれば検定する。
#[test]
fn test_rc_supported_shear_rebar_fy_missing_skip() {
    let sec = make_section(rc_rect_shape(
        300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2,
    ));
    let mat = make_material(24.0, "SD345");
    let forces = MemberForcesAt {
        pos: 0.0,
        n: 0.0,
        qy: 0.0,
        qz: 0.0,
        my: 0.0,
        mz: 0.0,
    };
    let mut shear_mat = make_rebar_material("SR235", 235.0);
    shear_mat.fy = None;
    let ctx = DesignCtx {
        shear_rebar_material: Some(shear_mat),
        ..ctx_beam(LoadTerm::Short)
    };
    let design = RcDesign;
    match design.check(&forces, &sec, &mat, &ctx) {
        CheckOutcome::Skipped { reason } => {
            assert!(
                reason.contains("SR235") && reason.contains("fy"),
                "{reason}"
            );
        }
        CheckOutcome::Checked(_) => panic!("fy 未設定は検定不能(Skipped)のはず"),
    }

    let ctx = DesignCtx {
        shear_rebar_material: Some(make_rebar_material("SR235", 235.0)),
        ..ctx_beam(LoadTerm::Short)
    };
    match design.check(&forces, &sec, &mat, &ctx) {
        CheckOutcome::Checked(_) => {}
        CheckOutcome::Skipped { reason } => panic!("fy 設定時は検定するはず: {reason}"),
    }
}

#[test]
fn test_rc_circle_beam_and_column_smoke() {
    let shape = SectionShape::RcCircle {
        d: 600.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 12,
                dia: 22.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 12,
                dia: 22.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 1,
            },
        },
    };
    let sec = make_section(shape);
    let mat = make_material(24.0, "SD345");
    let design = RcDesign;

    let forces = MemberForcesAt {
        pos: 0.0,
        n: -200_000.0,
        qy: 30_000.0,
        qz: 20_000.0,
        my: 10_000_000.0,
        mz: 20_000_000.0,
    };

    let ctx_col = ctx_column(LoadTerm::Short);
    let r_col = design.check(&forces, &sec, &mat, &ctx_col).unwrap_checked();
    assert!(r_col.ratio().is_finite() && r_col.ratio() >= 0.0);
    assert!(r_col.basis.contains("円形柱"));

    let ctx_b = ctx_beam(LoadTerm::Short);
    let r_beam = design.check(&forces, &sec, &mat, &ctx_b).unwrap_checked();
    assert!(r_beam.ratio().is_finite() && r_beam.ratio() >= 0.0);
}
