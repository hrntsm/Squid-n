//! 壁を含む最小モデルの統合テスト（壁領域再設計 Step 7+8 着手前の安全網）。
//!
//! # 目的
//!
//! `dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` §3.2 E8 で決めたとおり、
//! 実フィクスチャ（`tests/fixtures/model.stb`、`full_model.rs`）には壁要素が 0 件で、
//! 壁関連の機能（`WallAttr` の開口低減・自重、`OutOfFrameMiscWall` の 0.5m 分割集計）は
//! いずれも単体テストでしか検証されていない。`Model.floor_regions`（26 件）を
//! 巻き込まない**独立した壁専用フィクスチャ**として、耐震壁 1 パネル＋フレーム外雑壁
//! 1 本を含む最小の立体架構を用意し、`App` の全解析入口を通した代表スカラを
//! ピン止めする。`WallRegion`/`WallAttr`/`OutOfFrameMiscWall` の型を作り替える Step 7+8 の
//! 着手時、このテストの差分が「型変更が計算結果を変えていないか」の一次判定になる。
//!
//! # モデル
//!
//! 4m(X)×3m(Y)×3m(Z) の 1 スパン・1 層。柱 4 本（H-300x300 鋼）・梁 4 本（H-400x200 鋼、
//! 頂部で閉路）・耐震壁 1 枚（Y=0 面、RC t=150、開口 1 つ）・フレーム外雑壁 1 本
//! （Y=3000 面の梁上端に沿う想定、height=900・Column 伝達）。柱脚 4 節点は固定支点。
//! 荷重は ST-Bridge を経由しないため `App::run_preparation` の自動同期
//! （`sync_auto_load_cases_action`）が DL・LL・EX・EY を生成する。

use squid_n_app::app::{App, StaticCaseKey};
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, MaterialCategory,
    MiscWallTransfer, Model, Node, OutOfFrameMiscWall, RigidZone, Section, WallAttr, WallOpening,
};
use squid_n_section::shape::SectionShape;
use squid_n_solver::analysis::SeismicDir;

/// 有効数字 4 桁の指数表記（`full_model.rs::sig4` と同じ丸め規則。両ファイルの
/// スナップショットを比較できるよう合わせている）。
fn sig4(v: f64) -> String {
    if !v.is_finite() {
        return format!("{v}");
    }
    let v = if v == 0.0 { 0.0 } else { v };
    format!("{v:.3e}")
}

/// 指定した静的結果を取り出す（`full_model.rs::static_of` と同じ規則）。
fn static_res_for(app: &App, key: StaticCaseKey) -> &squid_n_solver::linear::StaticOnce {
    &app.results
        .as_ref()
        .expect("解析結果が格納されているはず")
        .statics
        .iter()
        .find(|(k, _)| *k == key)
        .expect("指定した荷重ケースの静的結果")
        .1
}

/// 壁 1 パネル＋雑壁 1 本を含む、柱 4 本・梁 4 本の 1 スパン立体架構。
fn wall_bay_model() -> Model {
    let mut model = Model::default();

    // 節点: 柱脚 4（固定）+ 柱頭 4（自由）。
    let base = [
        [0.0, 0.0, 0.0],
        [4000.0, 0.0, 0.0],
        [4000.0, 3000.0, 0.0],
        [0.0, 3000.0, 0.0],
    ];
    let top = [
        [0.0, 0.0, 3000.0],
        [4000.0, 0.0, 3000.0],
        [4000.0, 3000.0, 3000.0],
        [0.0, 3000.0, 3000.0],
    ];
    for (i, c) in base.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    for (i, c) in top.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(4 + i as u32),
            coord: *c,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }

    // 断面: 柱 H-300x300、梁 H-400x200（鋼、SN400B）。
    let col_shape = SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let beam_shape = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    };
    model
        .sections
        .push(col_shape.to_section(SectionId(0), "柱 H-300x300x10x15".into()));
    model
        .sections
        .push(beam_shape.to_section(SectionId(1), "梁 H-400x200x8x13".into()));
    // 断面: 耐震壁（RC t=150）。
    model.sections.push(Section {
        id: SectionId(2),
        name: "耐震壁 t150".into(),
        area: 150.0 * 3000.0,
        iy: 1.0e9,
        iz: 1.0e9,
        j: 1.0e9,
        depth: 3000.0,
        width: 150.0,
        as_y: 1.0e5,
        as_z: 1.0e5,
        floor: None,
        panel_thickness: None,
        thickness: Some(150.0),
        shape: None,
        material: Some(MaterialId(1)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });

    // 材料: 鋼 SN400B（柱・梁）、RC Fc24（耐震壁）。
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SN400B".into(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(235.0),
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(1),
        name: "Fc24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    for sec in model.sections.iter_mut().take(2) {
        sec.material = Some(MaterialId(0));
    }

    let beam = |id: u32, i: u32, j: u32, sec: u32, ref_v: [f64; 3]| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
        section: Some(SectionId(sec)),
        local_axis: LocalAxis { ref_vector: ref_v },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    };
    // 柱 4 本（id 0-3）。
    for i in 0..4u32 {
        model.elements.push(beam(i, i, 4 + i, 0, [1.0, 0.0, 0.0]));
    }
    // 梁 4 本（id 4-7。頂部で閉路: 4-5, 5-6, 6-7, 7-4）。
    let beam_pairs = [(4u32, 5u32), (5, 6), (6, 7), (7, 4)];
    for (k, (i, j)) in beam_pairs.iter().enumerate() {
        model
            .elements
            .push(beam(4 + k as u32, *i, *j, 1, [0.0, 0.0, 1.0]));
    }

    // 耐震壁 1 枚（Y=0 面: 節点 0,1,5,4 の矩形ループ）。開口あり。
    model.elements.push(ElementData {
        id: ElemId(8),
        kind: ElementKind::Wall,
        nodes: [NodeId(0), NodeId(1), NodeId(5), NodeId(4)]
            .into_iter()
            .collect(),
        section: Some(SectionId(2)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    });
    model.wall_attrs.push(WallAttr {
        elem: ElemId(8),
        opening_area: 0.0,
        opening_weight: 0.0,
        three_side_slit: false,
        openings: vec![WallOpening {
            width: 900.0,
            height: 1200.0,
            offset: Some([1550.0, 0.0]),
        }],
    });

    // フレーム外雑壁 1 本（Y=3000 面の梁 6-7 上端に沿うパラペット想定。
    // height=900・Column 伝達で節点 6・7 へ集計される）。
    model.misc_walls.push(OutOfFrameMiscWall {
        start: [4000.0, 3000.0, 3000.0],
        end: [0.0, 3000.0, 3000.0],
        height: 900.0,
        weight_per_area: 1.2e-3,
        transfer: MiscWallTransfer::Column,
        thickness: None,
    });

    model
}

fn wall_bay_app() -> App {
    let mut app = App::default();
    app.analysis_cfg.threads = 1;
    app.model = wall_bay_model();
    app
}

#[test]
fn test_wall_bay_model_is_valid() {
    let model = wall_bay_model();
    assert!(model.validate().is_ok(), "{:?}", model.validate());
    assert_eq!(model.nodes.len(), 8);
    assert_eq!(model.elements.len(), 9);
    assert_eq!(model.wall_attrs.len(), 1);
    assert_eq!(model.misc_walls.len(), 1);
}

#[test]
fn test_wall_bay_model_runs_full_pipeline() {
    let mut app = wall_bay_app();
    app.run_preparation();
    assert!(
        app.last_error.as_deref().unwrap_or("").is_empty()
            || app.last_error.as_deref().unwrap_or("").starts_with('⚠'),
        "準備計算でエラー: {:?}",
        app.last_error
    );
    app.last_error = None;

    app.run_static_all();
    assert!(
        app.last_error.is_none(),
        "静的解析でエラー: {:?}",
        app.last_error
    );

    app.run_eigen(app.analysis_cfg.n_modes);
    assert!(
        app.last_error.is_none(),
        "固有値解析でエラー: {:?}",
        app.last_error
    );

    assert!(app.results.is_some(), "解析結果が格納されているはず");
    assert!(
        app.preparation
            .as_ref()
            .unwrap()
            .summary
            .total_seismic_weight
            > 0.0,
        "地震用重量が正であること（壁・雑壁の自重を含む）"
    );
}

/// 代表スカラのスナップショット（有効数字 4 桁）。
///
/// `full_model.rs::snapshot_key_scalars` と同じ趣旨。壁関連の型（`WallRegion`・
/// `WallAttr`・`OutOfFrameMiscWall`）を作り替える際、この値の変化を V&V の対象とすること。
#[test]
fn snapshot_wall_bay_scalars() {
    let mut app = wall_bay_app();
    app.run_preparation();
    app.last_error = None;
    app.run_static_all();
    app.run_eigen(app.analysis_cfg.n_modes);

    let mut out = String::new();
    let mut line = |k: &str, v: String| {
        out.push_str(k);
        out.push_str(" = ");
        out.push_str(&v);
        out.push('\n');
    };

    let prep = app.preparation.as_ref().expect("準備計算の結果");
    line(
        "prep.total_seismic_weight",
        sig4(prep.summary.total_seismic_weight),
    );

    let modal = app
        .results
        .as_ref()
        .expect("解析結果")
        .modal
        .as_ref()
        .expect("固有値");
    for (i, t) in modal.period.iter().enumerate() {
        line(&format!("eigen.T[{i}]"), sig4(*t));
    }

    // 耐震壁要素（id=8）の材端力。`member_forces` は `Vec<(ElemId, MemberForces)>`
    // で位置は保証されないため、`ElemId(8)` で探す（Step 8 で要素生成の順序・件数が
    // 変われば `.get(8)` は無関係の要素を指しうる）。
    let wall_member_forces = |res: &squid_n_solver::linear::StaticOnce| {
        res.member_forces
            .iter()
            .find(|(id, _)| *id == ElemId(8))
            .expect(
                "耐震壁要素の材端力（Step 8 で参照先が壁領域 ID 起点へ変わったら、この \
                探索条件も追随させること）",
            )
            .1
            .at
            .clone()
    };

    // DL（鉛直荷重）は対称な架構では壁と柱の負担差が小さく、この壁単体では
    // 材端力がほぼゼロになる（剛性比の高い壁柱換算の軸力・曲げが小さいだけで、
    // 壁自体は剛性に確実に効いている。固有周期が壁の有無で 0.0797s→0.0522s と
    // 動くことは `test_wall_element_changes_eigen_period` で固定済み）。
    // **この 2 行は値の大きさではなく `member_forces` から壁要素が消えていないこと
    // 自体を監視する行**（Step 8 の参照張り替えで壁が結果から欠落する回帰を、
    // 上の `.expect()` パニックとして検知する）。壁の面内せん断・開口低減を
    // 実際に動かす荷重は地震（EX）のため、変化を追う代表値は EX 側に置く。
    let dl_key = app
        .model
        .load_cases
        .iter()
        .find(|c| c.kind == squid_n_core::model::LoadCaseKind::Dead)
        .map(|c| c.id)
        .expect("DL ケース");
    let dl_res = static_res_for(&app, StaticCaseKey::User(dl_key));
    let wall_forces = wall_member_forces(dl_res);
    line("static.DL.wall_forces_len", wall_forces.len().to_string());
    let dl_force_sum: f64 = wall_forces
        .iter()
        .flat_map(|(_, f)| f.iter())
        .map(|v| v.abs())
        .sum();
    line("static.DL.wall_forces_abs_sum", sig4(dl_force_sum));

    let ex_res = static_res_for(&app, StaticCaseKey::Seismic(SeismicDir::X));
    let ex_wall_forces = wall_member_forces(ex_res);
    let ex_force_sum: f64 = ex_wall_forces
        .iter()
        .flat_map(|(_, f)| f.iter())
        .map(|v| v.abs())
        .sum();
    line("static.EX.wall_forces_abs_sum", sig4(ex_force_sum));
    // 柱頭（節点 4、耐震壁側の柱）の水平変位。壁が面内せん断で剛性を
    // 持つ限り、壁なしのフレームより小さくなる（回帰時の一次判定に使える）。
    line("static.EX.top_disp_x[node4]", sig4(ex_res.disp[4][0]));

    insta::assert_snapshot!(out);
}

/// 壁要素が固有周期（＝剛性行列）に確実に寄与していることの確認。
///
/// `snapshot_wall_bay_scalars` の DL 側材端力がほぼゼロになる理由の裏付け
/// （壁が剛性に効いていないのではなく、この対称な架構・鉛直荷重条件では
/// 壁柱換算の軸力・曲げが小さいだけであること）を記録する。
#[test]
fn test_wall_element_changes_eigen_period() {
    let mut with_wall = wall_bay_app();
    with_wall.run_preparation();
    with_wall.last_error = None;
    with_wall.run_eigen(1);
    let t_with = with_wall
        .results
        .as_ref()
        .unwrap()
        .modal
        .as_ref()
        .unwrap()
        .period[0];

    let mut model_without = wall_bay_model();
    model_without
        .elements
        .retain(|e| e.kind != ElementKind::Wall);
    model_without.wall_attrs.clear();
    let mut without_wall = App::default();
    without_wall.analysis_cfg.threads = 1;
    without_wall.model = model_without;
    without_wall.run_preparation();
    without_wall.last_error = None;
    without_wall.run_eigen(1);
    let t_without = without_wall
        .results
        .as_ref()
        .unwrap()
        .modal
        .as_ref()
        .unwrap()
        .period[0];

    assert!(
        (t_with - t_without).abs() > t_without * 0.1,
        "壁の有無で 1 次固有周期が実質的に変わらない（壁が剛性に寄与していない）: \
         with={t_with}, without={t_without}"
    );
}
