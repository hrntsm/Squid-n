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

use squid_n_core::dof::DofMap;
use squid_n_core::ids::{LoadCaseId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    Constraint, LoadCase, LoadCaseKind, MemberLoad, MemberLoadKind, Model, Story,
};
use squid_n_solver::common::constraint::Reducer;
use squid_n_solver::nonlinear::pushover::{
    pushover_analysis_recording, DuctilityMethod, PushoverControl, PushoverResult, PushoverTarget,
};
use squid_n_solver::statics::analysis::SeismicDir;

#[path = "common/mod.rs"]
mod common;

/// nx×ny スパン・nz 層の S造立体ラーメン（柱・X/Y 大梁）を生成する。
/// 各階に剛床（隅節点をマスターとする）と地震用重量を与え、大梁に長期荷重
/// ケース（固定荷重、等分布荷重）を載荷する。
fn make_frame(nx: usize, ny: usize, nz: usize) -> Model {
    let grid = common::FrameGrid::new(nx, ny, nz);
    let (span, height) = (grid.span, grid.height);
    // 基部以外の節点は階へ帰属させる（剛床・地震用重量の集計対象）。
    let nodes = grid.build_nodes(|node, iz| {
        if iz > 0 {
            node.story = Some(StoryId((iz - 1) as u32));
        }
    });

    // 大梁は長期荷重を載荷する対象として ID を控える。
    let mut elements = Vec::new();
    let mut beam_ids_for_load = Vec::new();
    grid.push_frame_members(&mut elements, SectionId(0), SectionId(1), |id| {
        beam_ids_for_load.push(id)
    });

    // 長期荷重ケース（固定荷重）: 全大梁に等分布荷重 w=10 N/mm（鉛直下向き）。
    // プッシュオーバーの長期荷重初期載荷（apply_long_term）に使う。
    let member: Vec<MemberLoad> = beam_ids_for_load
        .iter()
        .map(|&elem| {
            MemberLoad::manual(
                elem,
                [0.0, 0.0, -1.0],
                MemberLoadKind::Distributed {
                    a: 0.0,
                    b: span,
                    w1: 10.0,
                    w2: 10.0,
                },
            )
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
            .flat_map(|iy| (0..=nx).map(move |ix| grid.node_id(ix, iy, iz)))
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
        sections: common::column_beam_sections(),
        materials: common::sn400_steel(),
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
