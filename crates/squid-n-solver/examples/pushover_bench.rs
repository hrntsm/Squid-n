//! 増分解析（プッシュオーバー）のベンチマーク（高速化前後の比較計測用）。
//!
//! 5層×3スパン×3スパンのS造立体フレーム（柱・梁、剛床、階の地震用重量、
//! 長期荷重ケース）を solver 単体で生成し、以下 2 ケースの所要時間を計測する。
//!
//! - (a) 段階制御（荷重制御→変位制御。既定の `PushoverControl::Phased`、
//!   終了目標は最大層間変形角 1/50）
//! - (b) 荷重増分のみ（`PushoverControl::LoadOnly`、同じ終了目標）
//!
//! 各ケースは 3 回実行して最小値を採用する（OS ノイズの影響を減らすため）。
//! 数値結果の非退行確認用に、最終ステップの頂部変位・ベースシア・確定ステップ数・
//! ヒンジ数を毎回表示する（高速化前後で完全一致すべき値）。
//!
//! ```bash
//! cargo run -p squid-n-solver --example pushover_bench --release
//! ```

use std::time::Instant;

use squid_n_core::dof::{Dof6Mask, DofMap};
use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    Constraint, ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LoadCaseKind,
    LocalAxis, Material, MaterialCategory, MemberLoad, MemberLoadKind, Model, Node, Section, Story,
};
use squid_n_solver::analysis::SeismicDir;
use squid_n_solver::constraint::Reducer;
use squid_n_solver::pushover::{
    pushover_analysis_recording, DuctilityMethod, PushoverControl, PushoverResult, PushoverTarget,
};

/// nx×ny スパン・nz 層の S造立体ラーメン（柱・X/Y 大梁）を生成する。
/// 各階に剛床（隅節点をマスターとする）と地震用重量を与え、大梁に長期荷重
/// ケース（固定荷重、等分布荷重）を載荷する。
fn make_frame(nx: usize, ny: usize, nz: usize) -> Model {
    let span = 6000.0; // [mm]
    let height = 3500.0; // [mm]

    let node_id = |ix: usize, iy: usize, iz: usize| -> NodeId {
        NodeId((iz * (nx + 1) * (ny + 1) + iy * (nx + 1) + ix) as u32)
    };

    let mut nodes = Vec::new();
    for iz in 0..=nz {
        for iy in 0..=ny {
            for ix in 0..=nx {
                let is_base = iz == 0;
                nodes.push(Node {
                    id: node_id(ix, iy, iz),
                    coord: [ix as f64 * span, iy as f64 * span, iz as f64 * height],
                    restraint: if is_base {
                        Dof6Mask::FIXED
                    } else {
                        Dof6Mask::FREE
                    },
                    mass: None,
                    story: if is_base {
                        None
                    } else {
                        Some(StoryId((iz - 1) as u32))
                    },
                    support_spring: None,
                });
            }
        }
    }

    let mut elements = Vec::new();
    let mut beam_ids_for_load = Vec::new();
    let mut push_beam =
        |n0: NodeId, n1: NodeId, ref_vector: [f64; 3], section: SectionId| -> ElemId {
            let id = ElemId(elements.len() as u32);
            elements.push(ElementData {
                id,
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![n0, n1],
                section: Some(section),
                local_axis: LocalAxis { ref_vector },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
            id
        };

    // 柱（全層・全通り、断面 SectionId(0)）
    for iz in 0..nz {
        for iy in 0..=ny {
            for ix in 0..=nx {
                push_beam(
                    node_id(ix, iy, iz),
                    node_id(ix, iy, iz + 1),
                    [1.0, 0.0, 0.0],
                    SectionId(0),
                );
            }
        }
    }
    // 大梁（各階、X/Y 両方向、断面 SectionId(1)）。長期荷重を載荷する対象として ID を控える。
    for iz in 1..=nz {
        for iy in 0..=ny {
            for ix in 0..nx {
                let id = push_beam(
                    node_id(ix, iy, iz),
                    node_id(ix + 1, iy, iz),
                    [0.0, 0.0, 1.0],
                    SectionId(1),
                );
                beam_ids_for_load.push(id);
            }
        }
        for iy in 0..ny {
            for ix in 0..=nx {
                let id = push_beam(
                    node_id(ix, iy, iz),
                    node_id(ix, iy + 1, iz),
                    [0.0, 0.0, 1.0],
                    SectionId(1),
                );
                beam_ids_for_load.push(id);
            }
        }
    }

    // 長期荷重ケース（固定荷重）: 全大梁に等分布荷重 w=10 N/mm（鉛直下向き）。
    // プッシュオーバーの長期荷重初期載荷（apply_long_term）に使う。
    let member: Vec<MemberLoad> = beam_ids_for_load
        .iter()
        .map(|&elem| MemberLoad {
            elem,
            dir: [0.0, 0.0, -1.0],
            kind: MemberLoadKind::Distributed {
                a: 0.0,
                b: span,
                w1: 10.0,
                w2: 10.0,
            },
        })
        .collect();
    let load_cases = vec![LoadCase {
        kind: LoadCaseKind::Dead,
        id: LoadCaseId(0),
        name: "長期".into(),
        nodal: Vec::new(),
        member,
    }];

    // 階設定: 各階に剛床（隅節点をマスター、残りをスレーブ）と地震用重量を持たせる。
    // 地震用重量は大梁の長期荷重合計（w×スパン長×本数）相当の概算値。
    let beams_per_story = nx * (ny + 1) + ny * (nx + 1);
    let seismic_weight = 10.0 * span * beams_per_story as f64;
    let mut stories = Vec::new();
    let mut constraints = Vec::new();
    for iz in 1..=nz {
        let node_ids: Vec<NodeId> = (0..=ny)
            .flat_map(|iy| (0..=nx).map(move |ix| node_id(ix, iy, iz)))
            .collect();
        let master = node_ids[0];
        let slaves: Vec<NodeId> = node_ids[1..].to_vec();
        constraints.push(Constraint::rigid_diaphragm(
            StoryId((iz - 1) as u32),
            master,
            slaves,
        ));
        stories.push(Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId((iz - 1) as u32),
            name: format!("{iz}F"),
            elevation: iz as f64 * height,
            node_ids,
            seismic_weight: Some(seismic_weight),
            weight_override: None,
        });
    }

    Model {
        nodes,
        elements,
        sections: vec![
            Section {
                id: SectionId(0),
                name: "柱 H-400x400x13x21".into(),
                area: 21_870.0,
                iy: 6.6e8,
                iz: 6.6e8,
                j: 2.0e7,
                depth: 400.0,
                width: 400.0,
                as_y: 12_000.0,
                as_z: 12_000.0,
                floor: None,
                panel_thickness: None,
                thickness: None,
                shape: None,
                material: Some(MaterialId(0)),
                rebar_material: None,
                shear_rebar_material: None,
                steel_material: None,
            },
            Section {
                id: SectionId(1),
                name: "梁 H-400x200x8x13".into(),
                area: 8_412.0,
                iy: 2.34e8,
                iz: 2.34e8,
                j: 6.0e5,
                depth: 400.0,
                width: 200.0,
                as_y: 4_000.0,
                as_z: 4_000.0,
                floor: None,
                panel_thickness: None,
                thickness: None,
                shape: None,
                material: Some(MaterialId(0)),
                rebar_material: None,
                shear_rebar_material: None,
                steel_material: None,
            },
        ],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SN400".into(),
            category: MaterialCategory::Steel,
            young: 205_000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(235.0),
        }],
        load_cases,
        stories,
        constraints,
        ..Default::default()
    }
}

/// 計測1回分: プッシュオーバー解析を実行して所要時間と結果を返す。
fn run_case(control: PushoverControl) -> (f64, PushoverResult) {
    let model = make_frame(3, 3, 5);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let target = PushoverTarget {
        max_disp: None,
        max_drift_angle: Some(1.0 / 50.0),
    };
    let t0 = Instant::now();
    let result = pushover_analysis_recording(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        50, // max_steps
        target,
        control,
        true,  // apply_long_term
        false, // use_kg
        false, // use_arc_length
        0.0,
        DuctilityMethod::default(),
    )
    .expect("pushover should converge");
    (t0.elapsed().as_secs_f64(), result)
}

/// 3回実行して最小時間と最後の結果を返す。
fn bench(control: PushoverControl) -> (f64, PushoverResult) {
    let mut best = f64::INFINITY;
    let mut last = None;
    for _ in 0..3 {
        let (t, r) = run_case(control);
        best = best.min(t);
        last = Some(r);
    }
    (best, last.unwrap())
}

fn print_result(label: &str, secs: f64, r: &PushoverResult) {
    let last = r.capacity_curve.last();
    let roof = last.map_or(0.0, |p| p.roof_disp);
    let shear = last.map_or(0.0, |p| p.base_shear);
    println!(
        "{label}: {secs:.3} s  確定ステップ {}  頂部変位 {roof:.4} mm  ベースシア {shear:.4} N  ヒンジ {}  せん断降伏 {}",
        r.capacity_curve.len(),
        r.hinges.len(),
        r.shear_yields.len(),
    );
}

fn main() {
    let model = make_frame(3, 3, 5);
    let dofmap = DofMap::build(&model);
    println!("=== 増分解析（プッシュオーバー）ベンチマーク ===");
    println!(
        "モデル: 3x3スパン 5層  節点 {}  部材 {}  自由度数(縮約前) {}",
        model.nodes.len(),
        model.elements.len(),
        dofmap.n_active(),
    );
    println!();
    println!("--- 各ケース3回実行、最小値を採用 ---");

    let (t_phased, r_phased) = bench(PushoverControl::Phased);
    print_result("(a) 段階制御 Phased  ", t_phased, &r_phased);

    let (t_load, r_load) = bench(PushoverControl::LoadOnly);
    print_result("(b) 荷重増分 LoadOnly", t_load, &r_load);

    println!();
    println!("=== 計測完了 ===");
}
