use super::*;
use squid_n_core::ids::{MaterialId, SectionId};
use squid_n_core::model::MaterialCategory;
use squid_n_core::section_shape::{
    BeamStirrup, CircleColumnHoop, RcBeamRebar, RcCircleColumnRebar, RcRectColumnRebar,
    RectColumnHoop, SectionShape,
};

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
    SectionShape::RcColumnRect {
        b,
        d,
        rebar: RcRectColumnRebar {
            main_dia,
            x: vec![main_count / 2; main_layers as usize],
            y: vec![main_count / 2; main_layers as usize],
            cover,
            hoop: RectColumnHoop {
                dia: shear_dia,
                pitch: shear_pitch,
                legs_x: shear_legs,
                legs_y: shear_legs,
            },
        },
    }
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn rc_beam_shape(
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
    SectionShape::RcBeamRect {
        b,
        d,
        rebar: RcBeamRebar {
            main_dia,
            top: vec![main_count; main_layers as usize],
            bottom: vec![main_count; main_layers as usize],
            cover,
            stirrup: BeamStirrup {
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
    // SD490 の短期は基準強度 490。長期せん断補強筋は 195 の据え置き。
    assert!((rebar_allowable_shear("SD490", true) - 195.0).abs() < 1e-9);
    assert!((rebar_allowable_shear("SD490", false) - 490.0).abs() < 1e-9);
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
    let props = axis_props_from_shape(&shape, crate::ultimate::rc_props::RcDirection::Strong, true)
        .unwrap();
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
    let props = axis_props_from_shape(&shape, crate::ultimate::rc_props::RcDirection::Strong, true)
        .unwrap();
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
fn test_beam_shear_damage_control_wft_cap_sd490() {
    let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
    let props = axis_props_from_shape(&shape, crate::ultimate::rc_props::RcDirection::Strong, true)
        .unwrap();
    assert!(props.pw > 0.002, "テストの前提として pw > 0.002 が必要");

    // 材料表の w_ft は SD490 短期 = 490 のまま（クランプは検定時のみ）。
    let allow = rc_allow(24.0, ConcreteClass::Normal, "SD490", false);
    assert!((allow.w_ft - 490.0).abs() < 1e-9);

    let alpha = 1.4;
    let qa_damage = shear_capacity(&props, &allow, alpha, LoadTerm::Short, true, false);
    let qa_safety = shear_capacity(&props, &allow, alpha, LoadTerm::Short, false, false);

    // 損傷制御の短期のみ w_ft を 390 にクランプして手計算と照合する。
    let pw_term_damage = 0.5 * 390.0 * (props.pw.min(0.012) - 0.002);
    let expected_damage = props.b * props.j * ((2.0 / 3.0) * alpha * allow.fs + pw_term_damage);
    assert!((qa_damage - expected_damage).abs() / expected_damage < 1e-9);

    // 安全確保の短期は材料表の 490 をそのまま用いる。
    let pw_term_safety = 0.5 * 490.0 * (props.pw.min(0.012) - 0.002);
    let expected_safety = props.b * props.j * (alpha * allow.fs + pw_term_safety);
    assert!((qa_safety - expected_safety).abs() / expected_safety < 1e-9);

    assert!(
        qa_damage < qa_safety,
        "損傷制御（w_ft=390）は安全確保（w_ft=490）より小さいはず"
    );
}

#[test]
fn test_column_safety_check_excludes_alpha() {
    let shape = rc_rect_shape(400.0, 400.0, 8, 22.0, 2, 40.0, 10.0, 100.0, 2);
    let props = axis_props_from_shape(&shape, crate::ultimate::rc_props::RcDirection::Strong, true)
        .unwrap();

    // 柱の「安全確保のための検討」式は α を含まない。普通強度せん断補強筋で、
    // 手計算の期待値 b・j・(fs + 0.5・w_ft・(pw − 0.002)) と照合する。
    // fs = min(24/30, 0.5+24/100)×1.5 = 1.11、pw ≈ 0.003927。
    let fs = 1.11;
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
    let props = axis_props_from_shape(&shape, crate::ultimate::rc_props::RcDirection::Strong, true)
        .unwrap();
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
    let sec = make_section(rc_beam_shape(
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
    let shape = SectionShape::RcColumnCircle {
        d: 600.0,
        rebar: RcCircleColumnRebar {
            main_dia: 22.0,
            count: 12,
            cover: 40.0,
            hoop: CircleColumnHoop {
                dia: 10.0,
                pitch: 100.0,
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

    // 円形柱断面を梁部材に割り当てた場合は用途不一致として検定不能。
    let ctx_b = ctx_beam(LoadTerm::Short);
    match design.check(&forces, &sec, &mat, &ctx_b) {
        CheckOutcome::Skipped { reason } => assert!(reason.contains("用途不一致"), "{reason}"),
        CheckOutcome::Checked(_) => panic!("柱用断面の梁割当は検定不能(Skipped)のはず"),
    }
}

fn rc_beam_rect_shape() -> SectionShape {
    SectionShape::RcBeamRect {
        b: 400.0,
        d: 600.0,
        rebar: RcBeamRebar {
            main_dia: 22.0,
            top: vec![4, 2],
            bottom: vec![3, 2],
            cover: 40.0,
            stirrup: BeamStirrup {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
            },
        },
    }
}

pub(crate) fn rc_column_rect_shape() -> SectionShape {
    SectionShape::RcColumnRect {
        b: 400.0,
        d: 400.0,
        rebar: RcRectColumnRebar {
            main_dia: 22.0,
            x: vec![3],
            y: vec![3],
            cover: 40.0,
            hoop: RectColumnHoop {
                dia: 10.0,
                pitch: 100.0,
                legs_x: 2,
                legs_y: 2,
            },
        },
    }
}

fn rc_column_circle_shape() -> SectionShape {
    SectionShape::RcColumnCircle {
        d: 600.0,
        rebar: RcCircleColumnRebar {
            main_dia: 22.0,
            count: 8,
            cover: 40.0,
            hoop: CircleColumnHoop {
                dia: 10.0,
                pitch: 100.0,
            },
        },
    }
}

/// 新型 `RcColumnRect` の柱検定は `Checked` を返し、軸力+二軸曲げとせん断の
/// 内訳を持つ。
#[test]
fn test_rc_column_rect_check_components_axial_bending_and_shear() {
    let sec = make_section(rc_column_rect_shape());
    let mat = make_material(24.0, "SD345");
    let ctx = ctx_column(LoadTerm::Long);
    let forces = MemberForcesAt {
        pos: 0.0,
        n: -200_000.0,
        qy: 50_000.0,
        qz: 30_000.0,
        my: 10.0e6,
        mz: 20.0e6,
    };
    let result = RcDesign.check(&forces, &sec, &mat, &ctx).unwrap_checked();
    assert!(result
        .components
        .iter()
        .any(|c| c.kind == crate::CheckKind::AxialBending));
    assert!(result
        .components
        .iter()
        .any(|c| c.kind == crate::CheckKind::Shear));
    assert!(result.basis.contains("柱"), "basis={}", result.basis);
}

/// 新型 `RcColumnCircle` の柱検定は `Checked` を返す。
#[test]
fn test_rc_column_circle_check_checked() {
    let sec = make_section(rc_column_circle_shape());
    let mat = make_material(24.0, "SD345");
    let ctx = ctx_column(LoadTerm::Long);
    let forces = MemberForcesAt {
        pos: 0.0,
        n: -200_000.0,
        qy: 30_000.0,
        qz: 20_000.0,
        my: 10.0e6,
        mz: 20.0e6,
    };
    let result = RcDesign.check(&forces, &sec, &mat, &ctx).unwrap_checked();
    assert!(result
        .components
        .iter()
        .any(|c| c.kind == crate::CheckKind::AxialBending));
    assert!(result.basis.contains("円形柱"), "basis={}", result.basis);
}

/// 円形柱の帯筋（1 組 2 本）は構造規定の pw 警告を常時出さない。
#[test]
fn test_rc_column_circle_hoop_pw_provision_ok() {
    use crate::ultimate::rc_props::RcDirection;

    let shape = rc_column_circle_shape();
    let info = rebar_info_from_shape(&shape, true).unwrap();
    let props = axis_props_from_shape(&shape, RcDirection::Strong, true).unwrap();
    let d = 600.0;
    let gross = std::f64::consts::PI * d * d / 4.0;
    assert!(props.pw > 0.002, "pw={}", props.pw);

    let prov = super::provisions::column_provisions_info(
        &info,
        d,
        3000.0,
        ConcreteClass::Normal,
        false,
        gross,
        info.main_area,
        0.0,
        24.0,
        props.pw,
    );
    assert!(
        !prov.warnings.iter().any(|w| w.starts_with("pw=")),
        "pw 警告が出た: {:?}",
        prov.warnings
    );
}

/// 新型 `RcBeamRect` は mz の符号で引張側（上端/下端）が変わり、上下非対称
/// （上 4+2 / 下 3+2）のため曲げ検定比が変わる。`RcDesign.check` が Checked を返す。
#[test]
fn test_rc_beam_rect_bending_depends_on_tension_side() {
    let sec = make_section(rc_beam_rect_shape());
    let mat = make_material(24.0, "SD345");
    let ctx = ctx_beam(LoadTerm::Short);
    let forces_pos = MemberForcesAt {
        pos: 0.0,
        n: 0.0,
        qy: 0.0,
        qz: 0.0,
        my: 0.0,
        mz: 40_000_000.0,
    };
    let forces_neg = MemberForcesAt {
        pos: forces_pos.pos,
        n: forces_pos.n,
        qy: forces_pos.qy,
        qz: forces_pos.qz,
        my: forces_pos.my,
        mz: -40_000_000.0,
    };
    let r_pos = RcDesign
        .check(&forces_pos, &sec, &mat, &ctx)
        .unwrap_checked();
    let r_neg = RcDesign
        .check(&forces_neg, &sec, &mat, &ctx)
        .unwrap_checked();
    let bend = |r: &crate::CheckResult| {
        r.components
            .iter()
            .find(|c| c.kind == crate::CheckKind::Bending)
            .expect("Bending component")
            .ratio
    };
    assert!(
        (bend(&r_pos) - bend(&r_neg)).abs() > 1e-9,
        "mz の符号で引張側が変わり曲げ検定比が変わるはず: pos={}, neg={}",
        bend(&r_pos),
        bend(&r_neg)
    );
}

/// 新型 `RcColumnCircle` の柱検定は円形断面の曲げ耐力（N-M 相関）を用いる。
#[test]
fn test_rc_circle_beam_bending_matches_circle_axis_props() {
    let shape = SectionShape::RcColumnCircle {
        d: 600.0,
        rebar: RcCircleColumnRebar {
            main_dia: 22.0,
            count: 12,
            cover: 40.0,
            hoop: CircleColumnHoop {
                dia: 10.0,
                pitch: 100.0,
            },
        },
    };
    let sec = make_section(shape.clone());
    let mat = make_material(24.0, "SD345");
    let ctx = ctx_column(LoadTerm::Short);
    let forces_at = |mz: f64| MemberForcesAt {
        pos: 0.0,
        n: 0.0,
        qy: 0.0,
        qz: 0.0,
        my: 0.0,
        mz,
    };
    let axial_bending = |mz: f64| {
        RcDesign
            .check(&forces_at(mz), &sec, &mat, &ctx)
            .unwrap_checked()
            .components
            .iter()
            .find(|c| c.kind == crate::CheckKind::AxialBending)
            .expect("AxialBending component")
            .ratio
    };

    // N=0 の円形柱は軸力比 0・曲げ比 (mz/MA)^2 の2乗則に従う。
    let r20 = axial_bending(20_000_000.0);
    let r40 = axial_bending(40_000_000.0);
    assert!(r20 > 0.0 && r20.is_finite(), "r20={r20}");
    assert!(
        (r40 / r20 - 4.0).abs() < 1e-9,
        "2乗則のはず: r20={r20}, r40={r40}"
    );
}

/// 新型 `RcBeamRect` の配筋が未入力なら、諸元算定の expect で panic せず
/// Skipped（配筋が未入力）を返す。
#[test]
fn test_rc_beam_rect_unset_rebar_skipped() {
    let shape = SectionShape::RcBeamRect {
        b: 400.0,
        d: 600.0,
        rebar: RcBeamRebar {
            main_dia: 22.0,
            top: vec![],
            bottom: vec![],
            cover: 40.0,
            stirrup: BeamStirrup {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
            },
        },
    };
    let sec = make_section(shape);
    let mat = make_material(24.0, "SD345");
    let ctx = ctx_beam(LoadTerm::Short);
    let forces = MemberForcesAt {
        pos: 0.0,
        n: 0.0,
        qy: 0.0,
        qz: 0.0,
        my: 0.0,
        mz: 10_000_000.0,
    };
    match RcDesign.check(&forces, &sec, &mat, &ctx) {
        CheckOutcome::Skipped { reason } => assert!(reason.contains("配筋が未入力"), "{reason}"),
        CheckOutcome::Checked(_) => panic!("配筋未入力は検定不能(Skipped)のはず"),
    }
}

/// 実配筋が幾何的に成立しない新モデル断面は、諸元算定の expect で panic せず
/// Skipped（配筋が不整合）を返す。
#[test]
fn test_rc_inconsistent_column_rebar_skipped() {
    let shape = SectionShape::RcColumnRect {
        b: 600.0,
        d: 700.0,
        rebar: RcRectColumnRebar {
            main_dia: 22.0,
            x: vec![4, 2],
            y: vec![3],
            cover: 40.0,
            hoop: RectColumnHoop {
                dia: 10.0,
                pitch: 100.0,
                legs_x: 2,
                legs_y: 3,
            },
        },
    };
    let sec = make_section(shape);
    let mat = make_material(24.0, "SD345");
    let forces = MemberForcesAt {
        pos: 0.0,
        n: -100_000.0,
        qy: 0.0,
        qz: 0.0,
        my: 0.0,
        mz: 10_000_000.0,
    };
    match RcDesign.check(&forces, &sec, &mat, &ctx_column(LoadTerm::Short)) {
        CheckOutcome::Skipped { reason } => assert!(reason.contains("配筋が不整合"), "{reason}"),
        CheckOutcome::Checked(_) => panic!("配筋不整合は検定不能(Skipped)のはず"),
    }
}

/// 梁用断面を柱部材へ割り当てた場合は用途不一致として検定不能。
#[test]
fn test_rc_beam_shape_on_column_skipped() {
    let sec = make_section(rc_beam_rect_shape());
    let mat = make_material(24.0, "SD345");
    let forces = MemberForcesAt {
        pos: 0.0,
        n: -100_000.0,
        qy: 0.0,
        qz: 0.0,
        my: 0.0,
        mz: 10_000_000.0,
    };
    match RcDesign.check(&forces, &sec, &mat, &ctx_column(LoadTerm::Short)) {
        CheckOutcome::Skipped { reason } => {
            assert!(reason.contains("用途不一致"), "{reason}");
            assert!(reason.contains("梁用断面"), "{reason}");
        }
        CheckOutcome::Checked(_) => panic!("用途不一致は検定不能(Skipped)のはず"),
    }
}

/// 柱用断面を梁部材へ割り当てた場合は用途不一致として検定不能。
#[test]
fn test_rc_column_shape_on_beam_skipped() {
    let sec = make_section(rc_column_rect_shape());
    let mat = make_material(24.0, "SD345");
    let forces = MemberForcesAt {
        pos: 0.0,
        n: 0.0,
        qy: 0.0,
        qz: 0.0,
        my: 0.0,
        mz: 10_000_000.0,
    };
    match RcDesign.check(&forces, &sec, &mat, &ctx_beam(LoadTerm::Short)) {
        CheckOutcome::Skipped { reason } => {
            assert!(reason.contains("用途不一致"), "{reason}");
            assert!(reason.contains("柱用断面"), "{reason}");
        }
        CheckOutcome::Checked(_) => panic!("用途不一致は検定不能(Skipped)のはず"),
    }
}
