use super::*;
use crate::rc::{concrete_allowable_shear, rebar_allowable_shear};
use crate::steel::{steel_f_value_prefix, steel_fs, steel_ft};
use crate::{LoadTerm, SeismicQd};
use squid_n_core::ids::{MaterialId, SectionId};
use squid_n_core::model::MaterialCategory;
use squid_n_core::rc_capacity::{rc_mu_simple, RcCapacityInput};
use squid_n_core::section_shape::{
    BeamStirrup, RcBeamRebar, RcRectColumnRebar, RectColumnHoop, SectionShape,
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

pub(crate) fn make_material_no_fc(grade: &str) -> Material {
    Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: grade.to_string(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    }
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn src_rect_shape(
    b: f64,
    d: f64,
    main_count: u32,
    main_dia: f64,
    main_layers: u32,
    cover: f64,
    shear_dia: f64,
    shear_pitch: f64,
    shear_legs: u32,
    steel_height: f64,
    steel_width: f64,
    steel_web_thick: f64,
    steel_flange_thick: f64,
) -> SectionShape {
    SectionShape::SrcColumnRect {
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
        steel_height,
        steel_width,
        steel_web_thick,
        steel_flange_thick,
    }
}

/// SRC 柱の標準テスト断面（500x500、8-D22 主筋、内蔵鉄骨 300x200 H形）。
pub(crate) fn src_column_shape() -> SectionShape {
    src_rect_shape(
        500.0, 500.0, 8, 22.0, 2, 40.0, 10.0, 100.0, 2, 300.0, 200.0, 9.0, 14.0,
    )
}

/// 実配筋 SRC 梁のテスト断面（RC 部分は [`RcBeamRebar`]）。
#[allow(clippy::too_many_arguments)]
pub(crate) fn src_beam_rect_shape(
    b: f64,
    d: f64,
    main_dia: f64,
    top: Vec<u32>,
    bottom: Vec<u32>,
    cover: f64,
    shear_dia: f64,
    shear_pitch: f64,
    shear_legs: u32,
    steel_height: f64,
    steel_width: f64,
    steel_web_thick: f64,
    steel_flange_thick: f64,
) -> SectionShape {
    SectionShape::SrcBeamRect {
        b,
        d,
        rebar: RcBeamRebar {
            main_dia,
            top,
            bottom,
            cover,
            stirrup: BeamStirrup {
                dia: shear_dia,
                pitch: shear_pitch,
                legs: shear_legs,
            },
        },
        steel_height,
        steel_width,
        steel_web_thick,
        steel_flange_thick,
    }
}

/// 実配筋 SRC 矩形柱のテスト断面（RC 部分は [`RcRectColumnRebar`]）。
#[allow(clippy::too_many_arguments)]
pub(crate) fn src_column_rect_shape(
    b: f64,
    d: f64,
    main_dia: f64,
    x: Vec<u32>,
    y: Vec<u32>,
    cover: f64,
    hoop_dia: f64,
    hoop_pitch: f64,
    legs_x: u32,
    legs_y: u32,
    steel_height: f64,
    steel_width: f64,
    steel_web_thick: f64,
    steel_flange_thick: f64,
) -> SectionShape {
    SectionShape::SrcColumnRect {
        b,
        d,
        rebar: RcRectColumnRebar {
            main_dia,
            x,
            y,
            cover,
            hoop: RectColumnHoop {
                dia: hoop_dia,
                pitch: hoop_pitch,
                legs_x,
                legs_y,
            },
        },
        steel_height,
        steel_width,
        steel_web_thick,
        steel_flange_thick,
    }
}

pub(crate) fn make_section(shape: SectionShape) -> Section {
    shape.to_section(SectionId(0), "test".to_string())
}

pub(crate) fn zero_forces() -> MemberForcesAt {
    MemberForcesAt {
        pos: 0.0,
        n: 0.0,
        qy: 0.0,
        qz: 0.0,
        my: 0.0,
        mz: 0.0,
    }
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

/// 内蔵鉄骨の材料（材料名が鋼種名。F 値の解決に用いる）。
pub(crate) fn make_steel_material(grade: &str) -> Material {
    Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(2),
        name: grade.to_string(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    }
}

/// SRC 検定の標準の材料構成（主筋 SD345・せん断補強筋 SD295A・内蔵鉄骨 SN400B）。
/// 材料は断面が持つため、検定コンテキストへ渡して解決させる。
fn ctx_materials(ctx: DesignCtx) -> DesignCtx {
    DesignCtx {
        rebar_material: Some(make_rebar_material("SD345", 345.0)),
        shear_rebar_material: Some(make_rebar_material("SD295A", 295.0)),
        steel_material: Some(make_steel_material("SN400B")),
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

// ------------------------------------------------------------------
// せん断の鉄骨/RC 弾性分担（src_shear_check）
// ------------------------------------------------------------------

#[test]
fn test_src_beam_shear_split_handcalc() {
    let shape = src_beam_rect_shape(
        400.0,
        700.0,
        22.0,
        vec![6, 2],
        vec![6, 2],
        40.0,
        10.0,
        100.0,
        2,
        500.0,
        200.0,
        9.0,
        14.0,
    );
    let rebar = match &shape {
        SectionShape::SrcBeamRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = beam_axis_props(400.0, 700.0, &rebar, false);
    let (_sa, sz, _) = steel_h_props(500.0, 200.0, 9.0, 14.0);

    let q = 200_000.0;
    let expected_s_q = sz / (sz + props.at * props.j) * q;

    let fs = concrete_allowable_shear(24.0, true);
    let w_ft = rebar_allowable_shear("SD345", true);
    let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
    let s_fs = steel_fs(f_value, LoadTerm::Long);

    let ctx = ctx_beam(LoadTerm::Long);
    let seismic = SrcSeismicCtx {
        ctx: &ctx,
        pos: 0.0,
        q_index: 1,
        s_ft_short: 0.0,
        r_mu: 0.0,
    };
    let shear = src_shear_check(
        q,
        0.0,
        q,
        sz,
        props.at,
        props.j,
        props.d,
        props.b,
        200.0,
        props.pw,
        fs,
        w_ft,
        s_fs,
        9.0 * (500.0 - 2.0 * 14.0),
        2.0,
        &SrcShearMode::Beam,
        &seismic,
    );
    assert!(!shear.used_qd);
    assert!((shear.s_q - expected_s_q).abs() / expected_s_q < 1e-9);
    assert!((shear.s_q + shear.r_q - q).abs() < 1e-6);
}

/// SRC の pw 上限は SRC 規準1987 準拠で長短期とも 0.6%
/// （「pw が 0.6% を超える場合は 0.6% として算定する」）。
#[test]
fn test_src_shear_pw_capped_at_0_6_percent_both_terms() {
    // 過大なせん断補強筋比（pw > 0.6%）を与え、算定に使われる pw が
    // 0.6% に頭打ちされることを確認する。
    let shape = src_beam_rect_shape(
        400.0,
        700.0,
        22.0,
        vec![6, 2],
        vec![6, 2],
        40.0,
        13.0,
        30.0,
        4,
        500.0,
        200.0,
        9.0,
        14.0,
    );
    let rebar = match &shape {
        SectionShape::SrcBeamRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = beam_axis_props(400.0, 700.0, &rebar, false);
    assert!(props.pw > 0.006, "テストの前提として pw > 0.6% が必要");

    let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
    for long_term in [true, false] {
        let fs = concrete_allowable_shear(24.0, long_term);
        let w_ft = rebar_allowable_shear("SD345", long_term);
        let term = if long_term {
            LoadTerm::Long
        } else {
            LoadTerm::Short
        };
        let s_fs = steel_fs(f_value, term);
        let ctx = ctx_beam(term);
        let seismic = SrcSeismicCtx {
            ctx: &ctx,
            pos: 0.0,
            q_index: 1,
            s_ft_short: 0.0,
            r_mu: 0.0,
        };
        let shear = src_shear_check(
            100_000.0,
            0.0,
            100_000.0,
            0.0, // 鉄骨寄与を 0 として RC 側の pw の効果だけを見る
            props.at,
            props.j,
            props.d,
            props.b,
            200.0,
            props.pw,
            fs,
            w_ft,
            s_fs,
            0.0,
            2.0,
            &SrcShearMode::Beam,
            &seismic,
        );
        assert!(
            (shear.pw - 0.006).abs() < 1e-12,
            "long_term={long_term}: pw={} は 0.6% に頭打ちされるはず",
            shear.pw
        );
    }
}

/// SRC 柱の短期 RC 部許容せん断力 rQAS1 は α を含まない
/// （SRC規準。rQAS1 = b・rj・(fs + 0.5・pw・wft)）。
#[test]
fn test_src_column_short_rc_allowable_has_no_alpha() {
    let shape = src_rect_shape(
        400.0, 700.0, 6, 22.0, 2, 40.0, 13.0, 100.0, 2, 500.0, 200.0, 9.0, 14.0,
    );
    let rebar = match &shape {
        SectionShape::SrcColumnRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = column_axis_props(400.0, 700.0, &rebar, true);
    let fs = concrete_allowable_shear(24.0, false);
    let w_ft = rebar_allowable_shear("SD345", false);
    let ctx = ctx_column(LoadTerm::Short);
    let seismic = SrcSeismicCtx {
        ctx: &ctx,
        pos: 0.0,
        q_index: 1,
        s_ft_short: 0.0,
        r_mu: 0.0,
    };
    // m_for_alpha=0 → α は上限側（=2）に張り付く条件。α が式に入っていれば
    // rQA1 が 2 倍近く動くが、柱の短期 rQAS1 は α 非依存であること。
    let shear = src_shear_check(
        100_000.0,
        0.0,
        100_000.0,
        0.0, // 鉄骨寄与 0 で RC 側のみを見る
        props.at,
        props.j,
        props.d,
        props.b,
        390.0, // b′ を大きく取り rQA2 を支配させない
        props.pw,
        fs,
        w_ft,
        0.0,
        0.0,
        2.0,
        &SrcShearMode::Column { beta: 0.0 },
        &seismic,
    );
    let pw = props.pw.min(0.006);
    let expected_rqa1 = props.b * props.j * (fs + 0.5 * pw * w_ft);
    let expected_rqa2 = props.b * props.j * (2.0 * (390.0 / props.b) * fs + pw * w_ft);
    let expected = expected_rqa1.min(expected_rqa2);
    assert!(
        (shear.r_qa - expected).abs() / expected < 1e-9,
        "rQA={} expected={}（α を含まない短期式）",
        shear.r_qa,
        expected
    );
}

/// SRC 柱の長期は併用式 QA = (1+β)・b・rj・a′・fs を全せん断力と比較する
/// （SRC規準 P.96-97。a′ = rα（b′/b ≥ rα/3 のとき）または 3b′/b）。
#[test]
fn test_src_column_long_combined_formula() {
    let shape = src_rect_shape(
        400.0, 700.0, 6, 22.0, 2, 40.0, 13.0, 100.0, 2, 500.0, 200.0, 9.0, 14.0,
    );
    let rebar = match &shape {
        SectionShape::SrcColumnRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = column_axis_props(400.0, 700.0, &rebar, true);
    let fs = concrete_allowable_shear(24.0, true);
    let ctx = ctx_column(LoadTerm::Long);
    let seismic = SrcSeismicCtx {
        ctx: &ctx,
        pos: 0.0,
        q_index: 1,
        s_ft_short: 0.0,
        r_mu: 0.0,
    };
    let beta = 0.25;
    let q = 150_000.0;
    // m=0 → α=2（上限）。b′/b=0.5 < α/3=2/3 なので a′=3b′/b=1.5 が採用される。
    let b_prime = 0.5 * props.b;
    let shear = src_shear_check(
        q,
        0.0,
        q,
        1.0e6,
        props.at,
        props.j,
        props.d,
        props.b,
        b_prime,
        props.pw,
        fs,
        0.0,
        100.0,
        1000.0,
        2.0,
        &SrcShearMode::Column { beta },
        &seismic,
    );
    let a_prime = 1.5;
    let qa = (1.0 + beta) * props.b * props.j * a_prime * fs;
    assert!(
        (shear.r_qa - qa).abs() / qa < 1e-9,
        "QA={} expected={}",
        shear.r_qa,
        qa
    );
    assert!((shear.ratio - q / qa).abs() < 1e-9, "ratio={}", shear.ratio);
    assert!(!shear.used_qd);
}

// ------------------------------------------------------------------
// DesignCheck 振り分けの共通経路（Fc未設定・断面形状不一致）
// ------------------------------------------------------------------

#[test]
fn test_src_fc_missing_skip() {
    let shape = src_column_shape();
    let sec = make_section(shape);
    let mat = make_material_no_fc("SD345");
    let ctx = ctx_column(LoadTerm::Long);
    let design = SrcDesign;
    let outcome = design.check(&zero_forces(), &sec, &mat, &ctx);
    match outcome {
        CheckOutcome::Skipped { reason } => assert!(reason.contains("Fc")),
        CheckOutcome::Checked(_) => panic!("Fc 未設定は検定不能(Skipped)のはず"),
    }
}

#[test]
fn test_src_shape_mismatch_skip() {
    let sec = Section {
        id: SectionId(0),
        name: "no-shape".to_string(),
        area: 1.0,
        iy: 1.0,
        iz: 1.0,
        j: 1.0,
        depth: 500.0,
        width: 500.0,
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
    let ctx = ctx_column(LoadTerm::Long);
    let design = SrcDesign;
    let outcome = design.check(&zero_forces(), &sec, &mat, &ctx);
    match outcome {
        CheckOutcome::Skipped { reason } => assert!(reason.contains("断面形状不一致")),
        CheckOutcome::Checked(_) => panic!("断面形状不一致は検定不能(Skipped)のはず"),
    }
}

/// せん断補強筋に未対応グレード（KH785）を割り当てた SRC 部材は、普通強度として
/// 検定せずに理由付きで検定不能とする。
#[test]
fn test_src_unsupported_shear_rebar_grade_skip() {
    let sec = make_section(src_column_shape());
    let mat = make_material(24.0, "SD345");
    let ctx = DesignCtx {
        shear_rebar_material: Some(make_rebar_material("KH785", 785.0)),
        ..ctx_column(LoadTerm::Long)
    };
    let design = SrcDesign;
    let outcome = design.check(&zero_forces(), &sec, &mat, &ctx);
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
fn test_src_supported_shear_rebar_fy_missing_skip() {
    let sec = make_section(src_column_shape());
    let mat = make_material(24.0, "SD345");
    let mut shear_mat = make_rebar_material("SR235", 235.0);
    shear_mat.fy = None;
    let ctx = DesignCtx {
        shear_rebar_material: Some(shear_mat),
        ..ctx_column(LoadTerm::Long)
    };
    let design = SrcDesign;
    match design.check(&zero_forces(), &sec, &mat, &ctx) {
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
        ..ctx_column(LoadTerm::Long)
    };
    match design.check(&zero_forces(), &sec, &mat, &ctx) {
        CheckOutcome::Checked(_) => {}
        CheckOutcome::Skipped { reason } => panic!("fy 設定時は検定するはず: {reason}"),
    }
}

// ------------------------------------------------------------------
// 地震時短期の設計用せん断力（構造規定方式）: SRC 梁
// ------------------------------------------------------------------

/// rQD2 = max(0, n・(|Q|−sQD)) が支配するケース（rMu=0 で rQD1 を無効化し
/// QdMethod::Qd2 で明示的に検証する）。
#[test]
fn test_src_beam_seismic_qd2_handcalc() {
    use crate::QdMethod;
    let shape = src_beam_rect_shape(
        400.0,
        700.0,
        22.0,
        vec![6, 2],
        vec![6, 2],
        40.0,
        10.0,
        100.0,
        2,
        500.0,
        200.0,
        9.0,
        14.0,
    );
    let rebar = match &shape {
        SectionShape::SrcBeamRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = beam_axis_props(400.0, 700.0, &rebar, false);
    let (_sa, sz, _) = steel_h_props(500.0, 200.0, 9.0, 14.0);
    let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
    let s_ft_short = steel_ft(f_value, LoadTerm::Short);
    let s_fs = steel_fs(f_value, LoadTerm::Short);
    let fs = concrete_allowable_shear(24.0, false);
    let w_ft = rebar_allowable_shear("SD345", false);

    let ql = 50_000.0; // 長期せん断力
    let q = 200_000.0; // 当該組合せの短期せん断力
    let n_factor = 1.5;
    let clear_length = 4000.0;

    let ctx = DesignCtx {
        seismic_qd: Some(SeismicQd {
            long_at: vec![(0.0, [0.0, ql, 0.0, 0.0, 0.0, 0.0])],
            n_factor,
            n_mechanism: 1.0,
            q_simple: None,
            clear_length,
            method: QdMethod::Qd2,
        }),
        ..Default::default()
    };
    // r_mu=0 とすることで rQD1 を無効化し（doc 参照）、rQD2 のみを検証する。
    let seismic = SrcSeismicCtx {
        ctx: &ctx,
        pos: 0.0,
        q_index: 1,
        s_ft_short,
        r_mu: 0.0,
    };

    let shear = src_shear_check(
        q,
        0.0,
        q,
        sz,
        props.at,
        props.j,
        props.d,
        props.b,
        200.0,
        props.pw,
        fs,
        w_ft,
        s_fs,
        9.0 * (500.0 - 2.0 * 14.0),
        2.0,
        &SrcShearMode::Beam,
        &seismic,
    );

    let denom = sz + props.at * props.j;
    let share = sz / denom;
    let s_ql = share * ql;
    let sum_s_m = 2.0 * sz * s_ft_short;
    let s_qd_expected = s_ql + sum_s_m / clear_length;
    let r_qd2_expected = (n_factor * (q - s_qd_expected)).max(0.0);

    assert!(shear.used_qd);
    assert!(
        (shear.s_q - s_qd_expected).abs() / s_qd_expected < 1e-9,
        "sQD={}, expected={}",
        shear.s_q,
        s_qd_expected
    );
    assert!(
        (shear.r_q - r_qd2_expected).abs() / r_qd2_expected.max(1.0) < 1e-9,
        "rQD={}, expected(rQD2)={}",
        shear.r_q,
        r_qd2_expected
    );
}

/// rQD1 = rQL + (rMu1+rMu2)/l′ が支配するケース（QdMethod::Qd1 で
/// 明示的に検証する）。
#[test]
fn test_src_beam_seismic_qd1_handcalc() {
    use crate::QdMethod;
    let shape = src_beam_rect_shape(
        400.0,
        700.0,
        22.0,
        vec![6, 2],
        vec![6, 2],
        40.0,
        10.0,
        100.0,
        2,
        500.0,
        200.0,
        9.0,
        14.0,
    );
    let rebar = match &shape {
        SectionShape::SrcBeamRect { rebar, .. } => rebar.clone(),
        _ => unreachable!(),
    };
    let props = beam_axis_props(400.0, 700.0, &rebar, false);
    let (_sa, sz, _) = steel_h_props(500.0, 200.0, 9.0, 14.0);
    let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
    let s_ft_short = steel_ft(f_value, LoadTerm::Short);
    let s_fs = steel_fs(f_value, LoadTerm::Short);
    let fs = concrete_allowable_shear(24.0, false);
    let w_ft = rebar_allowable_shear("SD345", false);

    let ql = 50_000.0;
    let q = 200_000.0;
    let n_factor = 1.5;
    let clear_length = 4000.0;
    // rc_mu_simple で機械的に算定した rMu（部材端 1 箇所分）。
    let r_mu = rc_mu_simple(&RcCapacityInput {
        b: props.b,
        d: props.d_full,
        at: props.at,
        d_eff: props.d,
        sigma_y: 345.0,
        fc: 24.0,
        pw: props.pw,
        sigma_wy: 0.0,
        clear_span: 0.0,
        sigma_0: 0.0,
    });
    assert!(r_mu > 0.0, "テストの前提として rMu>0 が必要");

    let ctx = DesignCtx {
        seismic_qd: Some(SeismicQd {
            long_at: vec![(0.0, [0.0, ql, 0.0, 0.0, 0.0, 0.0])],
            n_factor,
            n_mechanism: 1.0,
            q_simple: None,
            clear_length,
            method: QdMethod::Qd1,
        }),
        ..Default::default()
    };
    let seismic = SrcSeismicCtx {
        ctx: &ctx,
        pos: 0.0,
        q_index: 1,
        s_ft_short,
        r_mu,
    };

    let shear = src_shear_check(
        q,
        0.0,
        q,
        sz,
        props.at,
        props.j,
        props.d,
        props.b,
        200.0,
        props.pw,
        fs,
        w_ft,
        s_fs,
        9.0 * (500.0 - 2.0 * 14.0),
        2.0,
        &SrcShearMode::Beam,
        &seismic,
    );

    let denom = sz + props.at * props.j;
    let share = sz / denom;
    let s_ql = share * ql;
    let r_ql = (ql - s_ql).max(0.0);
    let r_qd1_expected = r_ql + 2.0 * r_mu / clear_length;

    assert!(shear.used_qd);
    assert!(
        (shear.r_q - r_qd1_expected).abs() / r_qd1_expected < 1e-9,
        "rQD={}, expected(rQD1)={}",
        shear.r_q,
        r_qd1_expected
    );
}

// ------------------------------------------------------------------
// 実配筋モデル（SrcBeamRect / SrcColumnRect）の SRC 検定
// ------------------------------------------------------------------

/// 実配筋 SRC 梁で許容応力度検定が動き、鉄骨 sMo に RC 部分 rMA が累加される
/// （MA > sMo）。
#[test]
fn test_src_beam_rect_check_accumulates_steel_and_rc() {
    let shape = src_beam_rect_shape(
        400.0,
        700.0,
        22.0,
        vec![3],
        vec![3],
        40.0,
        10.0,
        100.0,
        2,
        500.0,
        200.0,
        9.0,
        14.0,
    );
    let sec = make_section(shape);
    let mat = make_material(24.0, "SD345");
    let ctx = ctx_beam(LoadTerm::Long);

    // mz>0 は下端引張。
    let forces = MemberForcesAt {
        mz: 1.0,
        ..zero_forces()
    };
    let r = SrcDesign.check(&forces, &sec, &mat, &ctx).unwrap_checked();
    assert!(
        r.ratio().is_finite() && r.ratio() > 0.0,
        "ratio={}",
        r.ratio()
    );

    let ma = 1.0 / r.ratio();
    let (_sa, sz, _) = steel_h_props(500.0, 200.0, 9.0, 14.0);
    let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
    let s_mo = sz * steel_ft(f_value, LoadTerm::Long);
    assert!(
        ma > s_mo * 1.0001,
        "RC 部分の累加で MA > sMo のはず: ma={ma}, sMo={s_mo}"
    );
}

/// 実配筋 SRC 柱で許容応力度検定が動き、鉄骨 sMo に RC 部分の曲げ耐力が
/// 累加される（N=0 の MA_z > sMo）。
#[test]
fn test_src_column_rect_check_accumulates_steel_and_rc() {
    let shape = src_column_rect_shape(
        500.0,
        500.0,
        22.0,
        vec![4],
        vec![4],
        40.0,
        10.0,
        100.0,
        2,
        2,
        300.0,
        200.0,
        9.0,
        14.0,
    );
    let sec = make_section(shape);
    let mat = make_material(24.0, "SD345");
    let ctx = ctx_column(LoadTerm::Long);

    let forces = MemberForcesAt {
        mz: 1.0,
        ..zero_forces()
    };
    let r = SrcDesign.check(&forces, &sec, &mat, &ctx).unwrap_checked();
    assert!(
        r.ratio().is_finite() && r.ratio() > 0.0,
        "ratio={}",
        r.ratio()
    );

    let ma_z = 1.0 / r.ratio();
    let (_sa, sz_z, _) = steel_h_props(300.0, 200.0, 9.0, 14.0);
    let f_value = steel_f_value_prefix("SN400B", 14.0).unwrap();
    let s_mo = sz_z * steel_ft(f_value, LoadTerm::Long);
    assert!(
        ma_z >= s_mo * 0.99,
        "RC 部分の累加で MA_z >= sMo のはず: ma_z={ma_z}, sMo={s_mo}"
    );
}

/// 実配筋 SRC 梁で地震時短期の構造規定方式（終局曲げ rMu を含む）が動き、
/// 設計用せん断力が弾性分担ではなく構造規定方式で算定される。
#[test]
fn test_src_beam_rect_seismic_qd_uses_ultimate_moment() {
    use crate::QdMethod;
    let shape = src_beam_rect_shape(
        400.0,
        700.0,
        22.0,
        vec![3],
        vec![3],
        40.0,
        10.0,
        100.0,
        2,
        500.0,
        200.0,
        9.0,
        14.0,
    );
    let sec = make_section(shape);
    let mat = make_material(24.0, "SD345");
    let ctx = DesignCtx {
        seismic_qd: Some(SeismicQd {
            long_at: vec![(0.0, [0.0, 50_000.0, 0.0, 0.0, 0.0, 0.0])],
            n_factor: 1.5,
            n_mechanism: 1.0,
            q_simple: None,
            clear_length: 4000.0,
            method: QdMethod::Qd1,
        }),
        ..ctx_beam(LoadTerm::Short)
    };
    let forces = MemberForcesAt {
        qy: 200_000.0,
        ..zero_forces()
    };
    let r = SrcDesign.check(&forces, &sec, &mat, &ctx).unwrap_checked();
    let shear = r
        .components
        .iter()
        .find(|c| c.kind == crate::CheckKind::Shear)
        .expect("せん断成分があるはず");
    assert!(
        shear.detail.contains("構造規定方式"),
        "終局 rMu を含む構造規定方式が使われるはず: {}",
        shear.detail
    );
}

/// 実配筋 SRC 柱・梁の未入力配筋は検定不能（Skipped）とし、panic しない。
#[test]
fn test_src_new_types_unset_rebar_skip() {
    let beam = src_beam_rect_shape(
        400.0,
        700.0,
        22.0,
        vec![],
        vec![],
        40.0,
        10.0,
        100.0,
        2,
        500.0,
        200.0,
        9.0,
        14.0,
    );
    let col = src_column_rect_shape(
        500.0,
        500.0,
        22.0,
        vec![],
        vec![],
        40.0,
        10.0,
        100.0,
        2,
        2,
        300.0,
        200.0,
        9.0,
        14.0,
    );
    let mat = make_material(24.0, "SD345");
    let beam_out = SrcDesign.check(
        &zero_forces(),
        &make_section(beam),
        &mat,
        &ctx_beam(LoadTerm::Long),
    );
    let col_out = SrcDesign.check(
        &zero_forces(),
        &make_section(col),
        &mat,
        &ctx_column(LoadTerm::Long),
    );
    match beam_out {
        CheckOutcome::Skipped { reason } => assert!(reason.contains("配筋"), "{reason}"),
        CheckOutcome::Checked(_) => panic!("未入力梁配筋は Skipped のはず"),
    }
    match col_out {
        CheckOutcome::Skipped { reason } => assert!(reason.contains("配筋"), "{reason}"),
        CheckOutcome::Checked(_) => panic!("未入力柱配筋は Skipped のはず"),
    }
}
