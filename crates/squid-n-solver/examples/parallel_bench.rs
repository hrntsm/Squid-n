//! 並列計算のベンチマーク（速度確認用）。
//!
//! 多層立体ラーメンモデルを生成し、単一スレッド（Deterministic）と
//! 並列（Auto=全コア）で「解析準備（K 組立＋分解）」と「荷重組合せの
//! 一括解析（ケース並列）」の所要時間を比較する。
//!
//! ```bash
//! cargo run -p squid-n-solver --example parallel_bench --release
//! ```

use std::time::Instant;

use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LoadCombination, LocalAxis,
    Material, Model, NodalLoad, Node, Section,
};
use squid_n_math::parallelism::{set_parallelism, Parallelism};
use squid_n_solver::analysis::Analysis;

/// nx×ny スパン・nz 層の立体ラーメン（柱＋X/Y 大梁）を生成する。
fn make_frame(nx: usize, ny: usize, nz: usize, n_cases: usize) -> Model {
    let span = 6000.0; // [mm]
    let height = 3500.0; // [mm]
    let node_id = |ix: usize, iy: usize, iz: usize| -> NodeId {
        NodeId((iz * (nx + 1) * (ny + 1) + iy * (nx + 1) + ix) as u32)
    };

    let mut nodes = Vec::new();
    for iz in 0..=nz {
        for iy in 0..=ny {
            for ix in 0..=nx {
                nodes.push(Node {
                    id: node_id(ix, iy, iz),
                    coord: [ix as f64 * span, iy as f64 * span, iz as f64 * height],
                    restraint: if iz == 0 {
                        Dof6Mask::FIXED
                    } else {
                        Dof6Mask::FREE
                    },
                    mass: None,
                    story: None,
                    support_spring: None,
                });
            }
        }
    }

    let mut elements = Vec::new();
    let mut push_beam = |n0: NodeId, n1: NodeId, ref_vector: [f64; 3]| {
        elements.push(ElementData {
            id: ElemId(elements.len() as u32),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![n0, n1],
            section: Some(SectionId(0)),
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
                // 柱
                push_beam(
                    node_id(ix, iy, iz),
                    node_id(ix, iy, iz + 1),
                    [1.0, 0.0, 0.0],
                );
            }
        }
    }
    for iz in 1..=nz {
        for iy in 0..=ny {
            for ix in 0..nx {
                // X 方向大梁
                push_beam(
                    node_id(ix, iy, iz),
                    node_id(ix + 1, iy, iz),
                    [0.0, 0.0, 1.0],
                );
            }
        }
        for iy in 0..ny {
            for ix in 0..=nx {
                // Y 方向大梁
                push_beam(
                    node_id(ix, iy, iz),
                    node_id(ix, iy + 1, iz),
                    [0.0, 0.0, 1.0],
                );
            }
        }
    }

    // 荷重ケース: 最上層の全節点に方向・大きさ違いの集中荷重
    let top_nodes: Vec<NodeId> = (0..=ny)
        .flat_map(|iy| (0..=nx).map(move |ix| node_id(ix, iy, nz)))
        .collect();
    let load_cases: Vec<LoadCase> = (0..n_cases)
        .map(|i| {
            let mut values = [0.0; 6];
            values[i % 3] = 10_000.0 * ((i % 5) as f64 + 1.0);
            LoadCase {
                kind: Default::default(),
                id: LoadCaseId(i as u32 + 1),
                name: format!("case{}", i + 1),
                nodal: top_nodes
                    .iter()
                    .map(|&n| NodalLoad { node: n, values })
                    .collect(),
                member: Vec::new(),
            }
        })
        .collect();

    // 荷重組合せ: 隣接 2 ケースの線形和を n_cases 個
    let combinations: Vec<LoadCombination> = (0..n_cases)
        .map(|i| LoadCombination {
            name: format!("combo{}", i + 1),
            terms: vec![
                (LoadCaseId(i as u32 + 1), 1.0),
                (LoadCaseId(((i + 1) % n_cases) as u32 + 1), 0.5),
            ],
        })
        .collect();

    Model {
        nodes,
        elements,
        sections: vec![Section {
            id: SectionId(0),
            name: "H-400".into(),
            area: 8_400.0,
            iy: 2.3e8,
            iz: 2.3e8,
            j: 1.0e6,
            depth: 400.0,
            width: 200.0,
            as_y: 4_000.0,
            as_z: 4_000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
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
            fy: None,
        }],
        load_cases,
        combinations,
        ..Default::default()
    }
}

fn bench(label: &str, p: Parallelism, model: &Model) -> (f64, f64, f64) {
    set_parallelism(p);
    let t0 = Instant::now();
    let analysis = Analysis::prepare(model).expect("解析準備に失敗");
    let t_prepare = t0.elapsed().as_secs_f64();

    let t1 = Instant::now();
    let results = analysis.linear_combination_batch(&model.combinations);
    let t_batch = t1.elapsed().as_secs_f64();
    let n_ok = results.iter().filter(|r| r.is_ok()).count();

    // ケース数 < コア数のときの自動配分（余りコアを faer 内部並列へ）の確認用
    let t2 = Instant::now();
    let small = analysis.linear_combination_batch(&model.combinations[..2]);
    let t_small = t2.elapsed().as_secs_f64();
    let small_ok = small.iter().filter(|r| r.is_ok()).count();

    println!(
        "{label:<24} prepare(K組立+分解): {t_prepare:8.3}s  組合せ{}件一括: {t_batch:8.3}s (成功 {n_ok})  組合せ2件のみ: {t_small:8.3}s (成功 {small_ok})",
        results.len(),
    );
    (t_prepare, t_batch, t_small)
}

fn main() {
    let (nx, ny, nz, n_cases) = (14, 14, 24, 16);
    let model = make_frame(nx, ny, nz, n_cases);
    let n_dof = model.nodes.len() * 6;
    println!(
        "モデル: {}x{}スパン {}層  節点 {}  部材 {}  全DOF {}（基部固定を含む）",
        nx,
        ny,
        nz,
        model.nodes.len(),
        model.elements.len(),
        n_dof,
    );
    let threads = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1);
    println!("使用可能コア数: {threads}");
    println!();

    // ウォームアップ（ページキャッシュ・アロケータの影響を均す）
    let _ = bench("(ウォームアップ)", Parallelism::Deterministic, &model);
    println!();

    let (p_seq, b_seq, s_seq) = bench("単一スレッド(既定)", Parallelism::Deterministic, &model);
    let (p_par, b_par, s_par) = bench("並列(Auto=全コア)", Parallelism::Auto, &model);
    println!();
    println!(
        "速度比: prepare ×{:.2}  組合せ{}件一括 ×{:.2}  組合せ2件のみ ×{:.2}",
        p_seq / p_par,
        model.combinations.len(),
        b_seq / b_par,
        s_seq / s_par,
    );
}
