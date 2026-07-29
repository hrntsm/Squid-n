//! 固有値解析（部分空間反復）のベンチマーク（高速化前後の比較計測用）。
//!
//! 多層立体ラーメン（柱・X/Y 大梁、節点質量つき）を solver 単体で生成し、
//! モード数を変えて [`squid_n_solver::eigen::solve_eigen`] の所要時間を計測する。
//!
//! 各ケースは 3 回実行して最小値を採用する（OS ノイズの影響を減らすため）。
//! 数値結果の非退行確認用に、1次固有周期・最終モードの周期・有効質量比合計を
//! 毎回表示する（高速化前後で完全一致すべき値）。
//!
//! ```bash
//! cargo run -p squid-n-solver --example eigen_bench --release
//! ```

use std::time::Instant;

use squid_n_core::dof::{Dof6Mask, DofMap};
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node, Section,
};
use squid_n_solver::constraint::Reducer;
use squid_n_solver::eigen::solve_eigen;

/// nx×ny スパン・nz 層の立体ラーメン（柱＋X/Y 大梁）を生成する。
/// 基部以外の全節点に並進・回転質量を与える。
fn make_frame(nx: usize, ny: usize, nz: usize) -> Model {
    let span = 6000.0; // [mm]
    let height = 3500.0; // [mm]
    let node_mass = 800.0; // [N・s²/mm]
    let node_mass_rot = node_mass * 1.0e-3;

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
                    mass: if is_base {
                        None
                    } else {
                        Some([
                            node_mass,
                            node_mass,
                            node_mass,
                            node_mass_rot,
                            node_mass_rot,
                            node_mass_rot,
                        ])
                    },
                    story: None,
                    support_spring: None,
                });
            }
        }
    }

    let mut elements = Vec::new();
    let mut push_beam = |n0: NodeId, n1: NodeId, ref_vector: [f64; 3], section: SectionId| {
        elements.push(ElementData {
            id: ElemId(elements.len() as u32),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![n0, n1],
            section: Some(section),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis { ref_vector },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    };

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
    for iz in 1..=nz {
        for iy in 0..=ny {
            for ix in 0..nx {
                push_beam(
                    node_id(ix, iy, iz),
                    node_id(ix + 1, iy, iz),
                    [0.0, 0.0, 1.0],
                    SectionId(1),
                );
            }
        }
        for iy in 0..ny {
            for ix in 0..=nx {
                push_beam(
                    node_id(ix, iy, iz),
                    node_id(ix, iy + 1, iz),
                    [0.0, 0.0, 1.0],
                    SectionId(1),
                );
            }
        }
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
                panel_thickness: None,
                thickness: None,
                shape: None,
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
                panel_thickness: None,
                thickness: None,
                shape: None,
            },
        ],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SN400".into(),
            young: 205_000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(235.0),
        }],
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
