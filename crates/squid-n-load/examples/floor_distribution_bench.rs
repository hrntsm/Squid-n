//! 床荷重分配の所要時間を現行方式と基準資料方式で比較する（Release 実行用）。
//! 基準方式（一辺 100mm 以下の最近接辺格子・等距離等分）は本 example 内に自己完結で再現する。

use std::time::Instant;

use squid_n_core::geom::polygon as geom_polygon;
use squid_n_core::ids::NodeId;
use squid_n_core::model::{AreaLoad, DistributionMethod, Model, Node, Slab, SlabPlate};
use squid_n_load::floor::distribute_slab;

/// 基準資料方式の格子の最大辺長 [mm]。
const MAX_CELL_MM: f64 = 100.0;

/// 等距離とみなす許容差の、セル寸法に対する相対値。
const TIE_REL_TOL: f64 = 1e-9;

fn node(id: u32, x: f64, y: f64) -> Node {
    Node {
        id: NodeId(id),
        coord: [x, y, 0.0],
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    }
}

fn enclosed_slab_model(pts: &[(f64, f64)], method: DistributionMethod, w: f64) -> (Model, Slab) {
    let nodes: Vec<Node> = pts
        .iter()
        .enumerate()
        .map(|(i, (x, y))| node(i as u32, *x, *y))
        .collect();
    let boundary: Vec<NodeId> = (0..pts.len() as u32).map(NodeId).collect();
    let mut model = Model {
        nodes,
        ..Default::default()
    };
    let id = model.add_enclosed_slab_from_nodes(
        &boundary,
        SlabPlate {
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            method,
            ..Default::default()
        },
    );
    let slab = model.slabs[id.index()].clone();
    (model, slab)
}

/// 基準資料方式の各辺負担面積 [mm²] と格子内総面積 [mm²]。
fn reference_edge_areas(coords: &[[f64; 3]]) -> (Vec<f64>, f64) {
    let n = coords.len();
    let mut areas = vec![0.0_f64; n];
    if n < 3 {
        return (areas, 0.0);
    }
    let poly: Vec<[f64; 2]> = coords.iter().map(|c| [c[0], c[1]]).collect();
    let (lo, hi) = geom_polygon::bounding_box(&poly);
    let width = hi[0] - lo[0];
    let height = hi[1] - lo[1];
    if width <= 0.0 || height <= 0.0 {
        return (areas, 0.0);
    }
    let nx = (width / MAX_CELL_MM).ceil().max(1.0) as usize;
    let ny = (height / MAX_CELL_MM).ceil().max(1.0) as usize;
    let dx = width / nx as f64;
    let dy = height / ny as f64;
    let cell_area = dx * dy;
    let tie_tol = dx.max(dy) * TIE_REL_TOL;

    let mut dists = vec![0.0_f64; n];
    let mut sampled = 0.0_f64;
    for iy in 0..ny {
        let y = lo[1] + (iy as f64 + 0.5) * dy;
        for ix in 0..nx {
            let x = lo[0] + (ix as f64 + 0.5) * dx;
            let p = [x, y];
            if !geom_polygon::contains_by_ray_crossing(&poly, p) {
                continue;
            }
            sampled += cell_area;

            let mut d_min = f64::INFINITY;
            for (e, d) in dists.iter_mut().enumerate() {
                *d = geom_polygon::point_segment_dist(p, poly[e], poly[(e + 1) % n]);
                d_min = d_min.min(*d);
            }
            let ties: Vec<usize> = dists
                .iter()
                .enumerate()
                .filter(|(_, d)| **d - d_min <= tie_tol)
                .map(|(e, _)| e)
                .collect();
            let share = cell_area / ties.len() as f64;
            for e in ties {
                areas[e] += share;
            }
        }
    }
    (areas, sampled)
}

/// `f` を 1 回ウォームアップした後 `iters` 回実行し、所要時間 [ms] の中央値を返す。
fn bench<T>(iters: usize, mut f: impl FnMut() -> T) -> f64 {
    let warm = f();
    std::hint::black_box(&warm);
    let mut samples = Vec::with_capacity(iters);
    for _ in 0..iters {
        let t0 = Instant::now();
        let out = f();
        let dt = t0.elapsed().as_secs_f64() * 1000.0;
        std::hint::black_box(&out);
        samples.push(dt);
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    samples[samples.len() / 2]
}

struct Case {
    name: &'static str,
    pts: Vec<(f64, f64)>,
}

fn main() {
    let iters = 20;
    let w = 0.005_f64;
    let cases = vec![
        Case {
            name: "正方形 6000x6000",
            pts: vec![(0.0, 0.0), (6000.0, 0.0), (6000.0, 6000.0), (0.0, 6000.0)],
        },
        Case {
            name: "長方形 6000x4000",
            pts: vec![(0.0, 0.0), (6000.0, 0.0), (6000.0, 4000.0), (0.0, 4000.0)],
        },
        Case {
            name: "大きな床 30000x20000",
            pts: vec![
                (0.0, 0.0),
                (30000.0, 0.0),
                (30000.0, 20000.0),
                (0.0, 20000.0),
            ],
        },
        Case {
            name: "非矩形 L形 30000x20000",
            pts: vec![
                (0.0, 0.0),
                (30000.0, 0.0),
                (30000.0, 10000.0),
                (10000.0, 10000.0),
                (10000.0, 20000.0),
                (0.0, 20000.0),
            ],
        },
    ];

    println!("床荷重分配の所要時間（中央値, 各{iters}回, 面荷重 w={w}）");
    println!("ケース                       現行[ms]     基準[ms]   基準/現行");
    for case in &cases {
        let (model, slab) = enclosed_slab_model(&case.pts, DistributionMethod::TriTrapezoid, w);
        let coords = slab
            .boundary_coords(&model)
            .expect("境界座標")
            .iter()
            .map(|c| [c[0], c[1], 0.0])
            .collect::<Vec<[f64; 3]>>();

        let current = bench(iters, || distribute_slab(&model, &slab));
        let reference = bench(iters, || reference_edge_areas(&coords));
        println!(
            "{:<28} {current:9.4} {reference:11.4} {:10.2}",
            case.name,
            reference / current.max(1e-9),
        );
    }
}
