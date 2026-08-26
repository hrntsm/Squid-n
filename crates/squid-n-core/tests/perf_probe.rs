//! AD-HOC PERF PROBE -- measures the wall-clock cost of
//! `squid_n_core::region_gen::wall::wall_planes`（`generate_wall_region_boundaries`
//! を通じて計測する）on synthetic regular column-grid models.
//!
//! `dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` §5.7 参照。素朴な実装
//! （「候補ペアごとに既知の全直線と実距離で比較する」、外側ループ O(N²) × 内側の
//! 線形探索 O(L)）を実測したところ、格子状の柱配置では斜め方向の候補直線が
//! 柱本数の増加とともに大量に残るため L 自体が増え、900 本の柱（30×30 格子）で
//! 19 秒を超えた。グリッド索引（`LineIndex`）による重複判定の高速化後は同じ
//! ケースで約 1.8 秒まで縮んだが、依然として超線形に増える（本ファイルはこの
//! 実測値を得るためのものであり、更に最適化する場合の回帰検知にも使う）。
//!
//! Run (release, single-threaded so wall time is not muddied by test
//! parallelism; timings are printed via --nocapture):
//!
//!   cargo test --release -p squid-n-core --test perf_probe -- --nocapture --test-threads=1 --ignored

use std::time::Instant;

use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Model, Node,
};
use squid_n_core::region_gen::generate_wall_region_boundaries;

fn node(id: u32, x: f64, y: f64, z: f64) -> Node {
    Node {
        id: NodeId(id),
        coord: [x, y, z],
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    }
}

fn beam(id: u32, i: u32, j: u32) -> ElementData {
    ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
        section: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    }
}

/// nx × ny 本の通り芯 × 1 層の直交格子骨組（柱・梁とも RC 概算断面は不要、
/// `region_gen` は要素種別と節点座標しか見ないため断面・材料は付けない）。
fn build_grid_model(nx: usize, ny: usize) -> Model {
    let bay = 6000.0_f64;
    let story_h = 3500.0_f64;
    let mut model = Model::default();

    let node_id = |level: usize, i: usize, j: usize| -> u32 { ((level * ny + j) * nx + i) as u32 };
    for level in 0..=1 {
        for j in 0..ny {
            for i in 0..nx {
                model.nodes.push(node(
                    node_id(level, i, j),
                    i as f64 * bay,
                    j as f64 * bay,
                    level as f64 * story_h,
                ));
            }
        }
    }

    let mut eid = 0u32;
    // 柱。
    for j in 0..ny {
        for i in 0..nx {
            model
                .elements
                .push(beam(eid, node_id(0, i, j), node_id(1, i, j)));
            eid += 1;
        }
    }
    // 梁（X 方向・Y 方向、各階）。
    for level in 0..=1 {
        for j in 0..ny {
            for i in 0..nx - 1 {
                model
                    .elements
                    .push(beam(eid, node_id(level, i, j), node_id(level, i + 1, j)));
                eid += 1;
            }
        }
        for j in 0..ny - 1 {
            for i in 0..nx {
                model
                    .elements
                    .push(beam(eid, node_id(level, i, j), node_id(level, i, j + 1)));
                eid += 1;
            }
        }
    }
    model
}

fn run_case(label: &str, nx: usize, ny: usize) {
    let model = build_grid_model(nx, ny);
    let columns = nx * ny;
    let start = Instant::now();
    let boundaries = generate_wall_region_boundaries(&model);
    let elapsed = start.elapsed();
    println!(
        "{label}: {nx}x{ny} 柱={columns:>5} 要素={:>6} -> 壁境界={:>4} 個, {:>8.3}ms",
        model.elements.len(),
        boundaries.len(),
        elapsed.as_secs_f64() * 1000.0
    );
}

#[test]
#[ignore = "perf probe, not a correctness test -- run explicitly with --release --ignored"]
fn perf_probe_wall_planes() {
    run_case("A  5x5  ", 5, 5);
    run_case("B 10x10 ", 10, 10);
    run_case("C 15x15 ", 15, 15);
    run_case("D 20x20 ", 20, 20);
    run_case("E 30x30 ", 30, 30);
}
