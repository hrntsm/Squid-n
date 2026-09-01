//! 間柱で分割された壁版を持つモデルの統合テスト。
//!
//! # 目的
//!
//! 実フィクスチャ（`tests/fixtures/model.stb`）は壁 0 枚・間柱 0 本、壁フィクスチャ
//! （`wall_model.rs`）も間柱 0 本で、**壁版から間柱への荷重の分配を通すモデルが
//! 存在しなかった**。本ファイルはその経路を `App` の入口から固定する。
//!
//! `wall_model.rs` は「型の作り替えで計算結果が変わっていないか」を判定する基準と
//! いう役割を持つため、そこへ間柱を足すとその役割自体を壊す。分割壁版・間柱は
//! 独立したフィクスチャとして本ファイルに置く。
//!
//! # モデル
//!
//! 4m(X)×3m(Y)×3m(Z) の 1 スパン・1 層。柱 4 本・頂部大梁 4 本・基礎大梁 4 本
//! （すべて鋼）。柱脚 4 節点は固定支点。Y=0 面の壁領域を x=2000 の間柱 1 本で
//! 2 枚の壁版（RC t=150）へ分割している。
//!
//! # 何を固定するか
//!
//! - 分割された壁版は**壁エレメントにならない**（壁領域全体を覆っていないため）。
//! - 壁版の自重が失われない。間柱は左右の壁版から半分ずつ受け、逐次伝達で
//!   上下端の大梁へ渡す。柱側の鉛直辺は柱が受ける。
//! - 間柱が壁領域へ帰属し、解析前チェックのエラーにならない。
//! - 間柱は断面検定の対象外（軸力・面外曲げが未対応）だが、表から消えず
//!   「未」の行として残る。

use squid_n_app::app::App;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, WallPlateId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, MaterialCategory,
    Model, Node, RigidZone, SecondaryMember, SecondaryMemberKind, WallPlate, WallPlateShape,
};
use squid_n_core::section_shape::SectionShape;
use squid_n_core::wall_region_rebuild::rebuild_wall_regions;

const WALL_T: f64 = 150.0;
const RC_DENSITY: f64 = 2.4e-9;
/// 壁全体（4000×3000）の自重 [N]。開口はない。
fn wall_weight() -> f64 {
    4000.0 * 3000.0 * WALL_T * RC_DENSITY * squid_n_core::units::GRAVITY_MM_S2
}

fn wall_post_model() -> Model {
    let mut model = Model::default();

    // 節点: 柱脚 4（固定）・柱頭 4（自由）・間柱の上下端 2（要素非接続）。
    let plan = [[0.0, 0.0], [4000.0, 0.0], [4000.0, 3000.0], [0.0, 3000.0]];
    for (i, c) in plan.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: [c[0], c[1], 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    for (i, c) in plan.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(4 + i as u32),
            coord: [c[0], c[1], 3000.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    for (i, z) in [0.0_f64, 3000.0].into_iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(8 + i as u32),
            coord: [2000.0, 0.0, z],
            // 間柱の端部は支点ではない（要素が接続しない幾何節点）。
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }

    // 断面: 柱 H-300x300、梁 H-400x200（鋼）、壁 RC t=150、間柱 H-200x100。
    model.sections.push(
        SectionShape::SteelH {
            height: 300.0,
            width: 300.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        }
        .to_section(SectionId(0), "柱 H-300x300".into()),
    );
    model.sections.push(
        SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 8.0,
            flange_thick: 13.0,
        }
        .to_section(SectionId(1), "梁 H-400x200".into()),
    );
    let mut wall_sec = SectionShape::RcWall {
        thickness: WALL_T,
        ps: 0.0025,
    }
    .to_section(SectionId(2), "壁 t150".into());
    wall_sec.material = Some(MaterialId(1));
    model.sections.push(wall_sec);
    model.sections.push(
        SectionShape::SteelH {
            height: 200.0,
            width: 100.0,
            web_thick: 6.0,
            flange_thick: 8.0,
        }
        .to_section(SectionId(3), "間柱 H-200x100".into()),
    );
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
        young: 22000.0,
        poisson: 0.2,
        density: RC_DENSITY,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    for id in [0usize, 1, 3] {
        model.sections[id].material = Some(MaterialId(0));
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
    for i in 0..4u32 {
        model.elements.push(beam(i, i, 4 + i, 0, [1.0, 0.0, 0.0]));
    }
    // 頂部・柱脚とも閉路にする（`region_gen::wall` が 4 構面を検出するため）。
    // **間柱の位置では大梁を分割しない。** 分割すると壁領域の境界が 5 節点になる。
    for (k, (i, j)) in [(4u32, 5u32), (5, 6), (6, 7), (7, 4)].iter().enumerate() {
        model
            .elements
            .push(beam(4 + k as u32, *i, *j, 1, [0.0, 0.0, 1.0]));
    }
    for (k, (i, j)) in [(0u32, 1u32), (1, 2), (2, 3), (3, 0)].iter().enumerate() {
        model
            .elements
            .push(beam(8 + k as u32, *i, *j, 1, [0.0, 0.0, 1.0]));
    }

    // Y=0 面の壁を、間柱（節点 8-9）の位置で 2 枚へ分割する。
    for (id, boundary) in [(0u32, [0u32, 8, 9, 4]), (1, [8, 1, 5, 9])] {
        model.wall_plates.push(WallPlate {
            id: WallPlateId(id),
            shape: WallPlateShape::Enclosed {
                boundary: boundary.into_iter().map(NodeId).collect(),
            },
            section: Some(SectionId(2)),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            three_side_slit: false,
        });
    }
    model.unassigned_posts.push(SecondaryMember {
        kind: SecondaryMemberKind::Post,
        nodes: [NodeId(8), NodeId(9)],
        section: Some(SectionId(3)),
        name: "P1".into(),
    });

    let report = rebuild_wall_regions(&mut model);
    assert_eq!(report.regions, 4, "1 スパンの 4 鉛直構面が検出される");
    assert_eq!(
        report.wall_plates_assigned, 2,
        "壁版 2 枚が Y=0 面へ帰属する"
    );
    assert_eq!(report.unassigned_wall_plates, 0);
    assert_eq!(report.unassigned_posts, 0, "間柱が壁領域へ帰属する");

    model
}

fn wall_post_app() -> App {
    let mut app = App::default();
    app.analysis_cfg.threads = 1;
    app.model = wall_post_model();
    app
}

#[test]
fn test_model_is_valid() {
    let model = wall_post_model();
    assert!(model.validate().is_ok(), "{:?}", model.validate());
    assert_eq!(model.posts().count(), 1);
    assert_eq!(model.wall_plates.len(), 2);
}

/// 間柱で分割された壁版は壁エレメントにならない。壁柱が枚数分に割れることを
/// 避けるため、壁エレメントは壁領域全体を覆う 4 節点の壁版のときだけ作る。
#[test]
fn test_split_wall_plates_do_not_become_elements() {
    let model = wall_post_model();
    for plate in &model.wall_plates {
        assert!(
            !model.wall_plate_covers_region(plate),
            "分割された壁版は壁領域を覆わない"
        );
    }
    let (expanded, index, report) = squid_n_load::wall_expand::expand_wall_elements(&model);
    assert_eq!(report.generated, 0, "壁エレメントは作られない");
    assert_eq!(report.skipped_not_covering, 2);
    assert!(index.is_empty());
    assert!(expanded
        .elements
        .iter()
        .all(|e| e.kind != ElementKind::Wall));
}

/// 壁版の自重が、左右の鉛直辺（柱・間柱）へ半分ずつ配られる。
#[test]
fn test_wall_weight_is_split_between_columns_and_post() {
    let model = wall_post_model();
    let out = squid_n_load::wall_plate_load::distribute_enclosed_wall_plates(&model);

    let post = out
        .posts
        .get(&(NodeId(8), NodeId(9)))
        .expect("間柱が壁版から荷重を受ける");
    let post_total: f64 = post
        .member_loads
        .iter()
        .map(|l| match *l {
            squid_n_core::model::MemberLoadKind::Distributed { a, b, w1, w2 } => {
                (w1 + w2) / 2.0 * (b - a)
            }
            squid_n_core::model::MemberLoadKind::Point { p, .. } => p,
        })
        .sum();
    let expect = wall_weight() / 2.0;
    assert!(
        (post_total - expect).abs() / expect < 1e-6,
        "間柱は壁全体の 1/2（左右の壁版から 1/4 ずつ）を受ける: {post_total} / 期待 {expect}"
    );
    assert_eq!(out.primary.len(), 2, "柱側の鉛直辺 2 本が残りを受ける");
}

/// 準備計算・DL 同期・線形静解析まで通り、解析前チェックがエラーを出さない。
#[test]
fn test_runs_full_pipeline() {
    let mut app = wall_post_app();
    app.run_preparation();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
}

/// 壁版の自重が地震用重量へ算入される（要素にならなくても失われない）。
#[test]
fn test_wall_weight_reaches_story_seismic_weight() {
    let mut app = wall_post_app();
    app.run_preparation();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let total: f64 = app
        .model
        .stories
        .iter()
        .filter_map(|s| s.seismic_weight)
        .sum();
    assert!(
        total > wall_weight(),
        "層重量 {total} が壁自重 {} を含まない",
        wall_weight()
    );
}

/// 間柱は検定対象外だが、表から消さず「未」の行として残す。
#[test]
fn test_post_appears_as_unchecked_row() {
    let mut app = wall_post_app();
    app.run_preparation();
    app.run_linear_static(LoadCaseId(0));
    app.run_design_check();
    let results = app.results.as_ref().expect("解析結果");
    let post_rows: Vec<_> = results
        .joist_checks
        .iter()
        .filter(|(_, target, _)| {
            matches!(
                target,
                squid_n_app::app::JoistCheckTarget::SecondaryPost { .. }
            )
        })
        .collect();
    assert_eq!(post_rows.len(), 1, "間柱 1 本が表に出る");
    assert!(post_rows[0].2.unchecked, "判定は「未」");
}

/// 壁版の自重が固定荷重ケース「DL」へ届く。間柱が受けたぶんは逐次伝達が
/// 上下端の大梁の中間集中荷重へ変え、柱側の鉛直辺は柱が受ける。
///
/// 壁版を外したモデルとの差分で測る（躯体自重の算定式に依存しない）。
#[test]
fn test_wall_weight_reaches_dl_load_case() {
    let dl_total = |plates: bool| -> f64 {
        let mut app = wall_post_app();
        if !plates {
            app.model.wall_plates.clear();
            for r in &mut app.model.wall_regions {
                r.wall_plate_ids.clear();
            }
        }
        app.run_preparation();
        assert!(app.last_error.is_none(), "{:?}", app.last_error);
        let dl = app
            .model
            .load_cases
            .iter()
            .find(|lc| lc.kind == squid_n_core::model::LoadCaseKind::Dead)
            .expect("DL ケース");
        dl.member
            .iter()
            .map(|m| match m.kind {
                squid_n_core::model::MemberLoadKind::Distributed { a, b, w1, w2 } => {
                    (w1 + w2) / 2.0 * (b - a)
                }
                squid_n_core::model::MemberLoadKind::Point { p, .. } => p,
            })
            .sum::<f64>()
            + dl.nodal.iter().map(|nl| -nl.values[2]).sum::<f64>()
    };

    let diff = dl_total(true) - dl_total(false);
    assert!(
        (diff - wall_weight()).abs() / wall_weight() < 1e-6,
        "DL へ届いた壁自重 {diff} が壁版の自重 {} と一致しない",
        wall_weight()
    );
}

/// 要素にならない壁版の数量が数量拾いから落ちない。
///
/// 壁の数量は解析要素（`ElementKind::Wall`）経由でも数えるため、要素にならなく
/// なった壁版を壁版側で数え直さないと、数量が黙って消える。
#[test]
fn test_split_wall_plates_are_counted_in_quantity() {
    let model = wall_post_model();
    let takeoff = squid_n_design_jp::quantity::compute_quantity_takeoff(
        &model,
        &squid_n_design_jp::quantity::QuantityCfg::default(),
    );
    let wall_m3: f64 = takeoff
        .items
        .iter()
        .filter(|i| i.category == squid_n_design_jp::quantity::MemberCategory::MiscWall)
        .map(|i| i.concrete_m3)
        .sum();
    let expect = 4000.0 * 3000.0 * WALL_T * 1e-9;
    assert!(
        (wall_m3 - expect).abs() / expect < 1e-9,
        "壁 2 枚のコンクリート体積 {wall_m3} m3 が期待 {expect} m3 と一致しない"
    );
}
