//! 固有値解析（部分空間反復）のベンチマーク（高速化前後の比較計測用）。
//!
//! 多層立体ラーメン（柱・X/Y 大梁、節点質量つき）を solver 単体で生成し、
//! モード数を変えて [`squid_n_solver::dynamic::eigen::solve_eigen`] の所要時間を計測する。
//!
//! 各ケースは 3 回実行して最小値を採用する（OS ノイズの影響を減らすため）。
//! 数値結果の非退行確認用に、1次固有周期・最終モードの周期・有効質量比合計を
//! 毎回表示する（高速化前後で完全一致すべき値）。
//!
//! ```bash
//! cargo run -p squid-n-solver --example eigen_bench --release
//! ```

use std::time::Instant;

use squid_n_core::dof::DofMap;
use squid_n_core::ids::SectionId;
use squid_n_core::model::Model;
use squid_n_solver::common::constraint::Reducer;
use squid_n_solver::dynamic::eigen::solve_eigen;

#[path = "common/mod.rs"]
mod common;

/// nx×ny スパン・nz 層の立体ラーメン（柱＋X/Y 大梁）を生成する。
/// 基部以外の全節点に並進・回転質量を与える。
fn make_frame(nx: usize, ny: usize, nz: usize) -> Model {
    let node_mass = 800.0; // [N・s²/mm]
    let node_mass_rot = node_mass * 1.0e-3;

    let grid = common::FrameGrid::new(nx, ny, nz);
    let nodes = grid.build_nodes(|node, iz| {
        if iz > 0 {
            node.mass = Some([
                node_mass,
                node_mass,
                node_mass,
                node_mass_rot,
                node_mass_rot,
                node_mass_rot,
            ]);
        }
    });

    let mut elements = Vec::new();
    grid.push_frame_members(&mut elements, SectionId(0), SectionId(1), |_| {});

    Model {
        nodes,
        elements,
        sections: common::column_beam_sections(),
        materials: common::sn400_steel(),
        ..Default::default()
    }
}

fn bench_case(model: &Model, dofmap: &DofMap, reducer: &Reducer, n_modes: usize) {
    let mut best = f64::INFINITY;
    let mut last = None;
    for _ in 0..3 {
        let t0 = Instant::now();
        let r = solve_eigen(model, dofmap, reducer, n_modes).expect("eigen should converge");
        best = best.min(t0.elapsed().as_secs_f64());
        last = Some(r);
    }
    let r = last.unwrap();
    let eff_sum: f64 = r.effective_mass.iter().map(|m| m[0]).sum();
    println!(
        "モード数 {n_modes:>2}: {best:.3} s  T1={:.6} s  T{}={:.6} s  有効質量比X合計={:.6}",
        r.period.first().copied().unwrap_or(0.0),
        r.period.len(),
        r.period.last().copied().unwrap_or(0.0),
        eff_sum,
    );
}

fn main() {
    // 中規模モデル（数千自由度）。固有値解析の実運用レンジを想定する。
    let model = make_frame(6, 6, 12);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    println!("=== 固有値解析ベンチマーク ===");
    println!(
        "モデル: 6x6スパン 12層  節点 {}  部材 {}  自由度数(縮約後) {}",
        model.nodes.len(),
        model.elements.len(),
        reducer.n_indep,
    );
    println!();
    println!("--- 各ケース3回実行、最小値を採用 ---");
    for n_modes in [1, 6, 15] {
        bench_case(&model, &dofmap, &reducer, n_modes);
    }
    println!();
    println!("=== 計測完了 ===");
}
