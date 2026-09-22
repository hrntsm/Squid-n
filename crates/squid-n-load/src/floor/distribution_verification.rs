//! 床荷重のXY両方向分配の検証: 基準資料方式を本モジュール内だけに再現し、現行方式と比較する。
//! 基準方式は「一辺 100mm 以下の格子で各セル重心を最近接辺へ帰属し、等距離の辺には
//! 等分する」ものとする。本番の分配ロジックは変更しない。

use super::polygon::polygon_edge_areas;
use super::tests::{make_rect_slab_model, make_square_slab_model, polygon_slab_model, total_load};
use super::*;
use squid_n_core::geom::polygon as geom_polygon;
use squid_n_core::geom::polygon::area_xy;
use squid_n_core::ids::{ElemId, FloorRegionId, NodeId, SecondaryMemberId, SlabId};
use squid_n_core::model::{
    AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, FloorRegion, ForceRegime,
    LocalAxis, Node, SecondaryMember, SecondaryMemberAnchor, SecondaryMemberEnds,
    SecondaryMemberKind, SlabPlate, SupportMemberId,
};

/// 基準資料方式の格子の最大辺長 [mm]。
const MAX_CELL_MM: f64 = 100.0;

/// 等距離とみなす許容差の、セル寸法に対する相対値。
const TIE_REL_TOL: f64 = 1e-9;

/// 基準資料方式の各辺負担面積 [mm²] と、格子内と判定した総面積 [mm²]。
///
/// バウンディングボックスを `ceil(幅/100) × ceil(高さ/100)` に分割し、各セル中心が
/// 多角形内部なら最近接辺へセル面積を加算する。`split_ties` が真なら最小距離に並ぶ
/// 複数の辺へ均等に、偽なら最初に見つかった最小距離の辺へ全量を加算する。
fn reference_edge_areas(coords: &[[f64; 3]], split_ties: bool) -> (Vec<f64>, f64) {
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

            if split_ties {
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
            } else {
                let best = dists
                    .iter()
                    .enumerate()
                    .min_by(|a, b| a.1.total_cmp(b.1))
                    .map(|(e, _)| e)
                    .unwrap_or(0);
                areas[best] += cell_area;
            }
        }
    }
    (areas, sampled)
}

/// 基準資料方式の辺ごとの負担荷重 [N]。辺インデックス順の `Vec<f64>` で返す。
fn reference_edge_loads(coords: &[[f64; 3]], w: f64, split_ties: bool) -> Vec<f64> {
    reference_edge_areas(coords, split_ties)
        .0
        .iter()
        .map(|a| w * a)
        .collect()
}

/// 現行 polygon と同じ 200×200 固定格子で、多角形内部と判定したセル面積の総和 [mm²] を
/// 独立に集計する。`polygon_edge_areas` の総和と比較することで、辺帰属による面積の
/// 取りこぼしがないこと（総和保存）を辺負担のベクタ自身ではなく独立値で検証する。
fn polygon_grid_sampled_area(coords: &[[f64; 3]]) -> f64 {
    let poly: Vec<[f64; 2]> = coords.iter().map(|c| [c[0], c[1]]).collect();
    let (lo, hi) = geom_polygon::bounding_box(&poly);
    let width = hi[0] - lo[0];
    let height = hi[1] - lo[1];
    if width <= 0.0 || height <= 0.0 {
        return 0.0;
    }
    const N_GRID: usize = 200;
    let dx = width / N_GRID as f64;
    let dy = height / N_GRID as f64;
    let cell_area = dx * dy;
    let mut sampled = 0.0_f64;
    for iy in 0..N_GRID {
        let y = lo[1] + (iy as f64 + 0.5) * dy;
        for ix in 0..N_GRID {
            let x = lo[0] + (ix as f64 + 0.5) * dx;
            if geom_polygon::contains_by_ray_crossing(&poly, [x, y]) {
                sampled += cell_area;
            }
        }
    }
    sampled
}

/// 現行 polygon と同じ 200×200 固定格子・最近接辺で、最小距離に並ぶ辺（等距離）へ
/// セル面積を均等に配る「等分版」の各辺負担面積 [mm²]。距離は [`polygon_edge_areas`] と
/// 同じ 2 乗距離尺度で評価し、等距離の許容差はセル寸法に対する相対長さを換算して用いる。
fn current_split_edge_areas(coords: &[[f64; 3]]) -> Vec<f64> {
    let n = coords.len();
    let mut areas = vec![0.0_f64; n];
    if n < 3 {
        return areas;
    }
    let poly: Vec<[f64; 2]> = coords.iter().map(|c| [c[0], c[1]]).collect();
    let (lo, hi) = geom_polygon::bounding_box(&poly);
    let width = hi[0] - lo[0];
    let height = hi[1] - lo[1];
    if width <= 0.0 || height <= 0.0 {
        return areas;
    }
    const N_GRID: usize = 200;
    let dx = width / N_GRID as f64;
    let dy = height / N_GRID as f64;
    let cell_area = dx * dy;
    let tie_tol = dx.max(dy) * TIE_REL_TOL;
    let mut dists_sq = vec![0.0_f64; n];
    for iy in 0..N_GRID {
        let y = lo[1] + (iy as f64 + 0.5) * dy;
        for ix in 0..N_GRID {
            let x = lo[0] + (ix as f64 + 0.5) * dx;
            let p = [x, y];
            if !geom_polygon::contains_by_ray_crossing(&poly, p) {
                continue;
            }
            let mut d_min_sq = f64::INFINITY;
            for (e, d2) in dists_sq.iter_mut().enumerate() {
                *d2 = geom_polygon::point_segment_dist_sq(p, poly[e], poly[(e + 1) % n]);
                d_min_sq = d_min_sq.min(*d2);
            }
            let tie_limit_sq = tie_tol * (2.0 * d_min_sq.sqrt() + tie_tol);
            let ties: Vec<usize> = dists_sq
                .iter()
                .enumerate()
                .filter(|(_, d2)| **d2 - d_min_sq <= tie_limit_sq)
                .map(|(e, _)| e)
                .collect();
            let share = cell_area / ties.len() as f64;
            for e in ties {
                areas[e] += share;
            }
        }
    }
    areas
}

/// 荷重分配結果から、境界辺 `Edge(e)` ごとの総荷重 [N] を集計する。
fn edge_totals(loads: &[BeamLoad], n: usize) -> Vec<f64> {
    let mut out = vec![0.0_f64; n];
    for bl in loads {
        if let LoadTarget::Edge(e) = bl.target {
            if e < n {
                out[e] += bl.cmq.q_i + bl.cmq.q_j;
            }
        }
    }
    out
}

/// 総和が期待値（`w × 面積`）に相対 1e-9 以内で一致することを確認する（総和保存）。
fn assert_total(label: &str, values: &[f64], expected: f64) {
    let sum: f64 = values.iter().sum();
    let rel = (sum - expected).abs() / expected.abs().max(1e-12);
    assert!(
        rel < 1e-9,
        "{label}: 総和={sum} 期待={expected} 相対誤差={rel:e}"
    );
}

/// 辺負担がすべて等しいこと（対称性が崩れていないこと）を確認する。
fn assert_symmetric(values: &[f64], label: &str) {
    let max = values.iter().cloned().fold(f64::MIN, f64::max);
    let min = values.iter().cloned().fold(f64::MAX, f64::min);
    let rel = (max - min).abs() / max.abs().max(1e-12);
    assert!(
        rel < 1e-9,
        "{label}: 対称性が崩れた max={max} min={min} 相対差={rel:e}"
    );
}

/// `pairs` で指定した辺どうしの負担が等しいことを確認する（長方形の対辺対称性）。
fn assert_pairs_equal(values: &[f64], pairs: &[(usize, usize)], label: &str) {
    for &(a, b) in pairs {
        let (va, vb) = (values[a], values[b]);
        let rel = (va - vb).abs() / va.abs().max(vb.abs()).max(1e-12);
        assert!(
            rel < 1e-9,
            "{label}: 辺{a}={va} と辺{b}={vb} が等しくない 相対差={rel:e}"
        );
    }
}

/// 基準方式の辺負担面積の総和が、格子内と判定した面積に厳密（相対 1e-9）に一致する
/// ことを確認する（等距離の等分で面積が落ちないこと）。
fn assert_reference_conserves(coords: &[[f64; 3]], label: &str) {
    let (areas, sampled) = reference_edge_areas(coords, true);
    let sum: f64 = areas.iter().sum();
    assert!(
        (sum - sampled).abs() <= 1e-9 * sampled.max(1.0),
        "{label}: 面積の総和={sum} 格子内面積={sampled}"
    );
}

/// 現行と基準の辺負担を表と総和で出力する。`true_area` は多角形の真の面積 [mm²]。
fn print_comparison(label: &str, w: f64, true_area: f64, current: &[f64], baseline: &[f64]) {
    let cur_total: f64 = current.iter().sum();
    let base_total: f64 = baseline.iter().sum();
    println!("--- {label} ---");
    println!("辺   現行[N]          基準[N]          差[%]      判定");
    for i in 0..current.len().max(baseline.len()) {
        let c = current.get(i).copied().unwrap_or(0.0);
        let b = baseline.get(i).copied().unwrap_or(0.0);
        let d = if b.abs() > 1e-12 {
            100.0 * (c - b) / b
        } else {
            0.0
        };
        let side = if (c - b).abs() <= b.abs() * 1e-9 {
            "一致"
        } else if c < b {
            "現行<基準(過小=危険側)"
        } else {
            "現行>基準(過大=安全側)"
        };
        println!("{i:<3} {c:15.3} {b:15.3} {d:9.4}  {side}");
    }
    let expected = w * true_area;
    println!(
        "総和: 現行={cur_total:.6} 基準={base_total:.6} 真値={expected:.6} 現行誤差={:.3e} 基準誤差={:.3e}",
        (cur_total - expected).abs() / expected.abs().max(1e-12),
        (base_total - expected).abs() / expected.abs().max(1e-12),
    );
}

/// ケース1: 正方形 6000×6000。TriTrapezoid と基準方式の各辺負担を比較する。
#[test]
fn case1_square_6000_tritrapezoid_vs_reference() {
    let w = 0.005_f64;
    let side = 6000.0_f64;
    let (model, slab) = make_square_slab_model(side, DistributionMethod::TriTrapezoid, w);
    let loads = distribute_slab(&model, &slab);
    let current = edge_totals(&loads, 4);
    let coords = slab.boundary_coords(&model).expect("境界座標");
    let baseline = reference_edge_loads(&coords, w, true);

    print_comparison(
        "ケース1: 正方形 6000x6000 TriTrapezoid vs 基準方式",
        w,
        side * side,
        &current,
        &baseline,
    );

    assert_total("現行TriTrapezoid", &current, w * side * side);
    assert_symmetric(&current, "現行TriTrapezoid");
    assert_symmetric(&baseline, "基準方式");
    assert_reference_conserves(&coords, "基準方式");
}

/// ケース2: 長方形 6000×4000。TriTrapezoid と基準方式の各辺負担を比較する。
#[test]
fn case2_rect_6000x4000_tritrapezoid_vs_reference() {
    let w = 0.005_f64;
    let (lx, ly) = (6000.0_f64, 4000.0_f64);
    let (model, slab) = make_rect_slab_model(lx, ly, DistributionMethod::TriTrapezoid, w);
    let loads = distribute_slab(&model, &slab);
    let current = edge_totals(&loads, 4);
    let coords = slab.boundary_coords(&model).expect("境界座標");
    let baseline = reference_edge_loads(&coords, w, true);

    print_comparison(
        "ケース2: 長方形 6000x4000 TriTrapezoid vs 基準方式",
        w,
        lx * ly,
        &current,
        &baseline,
    );

    assert_total("現行TriTrapezoid", &current, w * lx * ly);
    assert_pairs_equal(&baseline, &[(0, 2), (1, 3)], "基準方式");
    assert_reference_conserves(&coords, "基準方式");
}

/// ケース3: 大きな床 30000×20000。現行 polygon の固定 200×200（セル 150×100）と
/// 基準方式（100mm 以下）の各辺負担を比較し、セル寸法が 100mm を超える影響を見る。
#[test]
fn case3_large_floor_polygon200_vs_reference() {
    let w = 0.002_f64;
    let (lx, ly) = (30000.0_f64, 20000.0_f64);
    let coords: Vec<[f64; 3]> = vec![
        [0.0, 0.0, 0.0],
        [lx, 0.0, 0.0],
        [lx, ly, 0.0],
        [0.0, ly, 0.0],
    ];

    let poly_areas = polygon_edge_areas(&coords, &[0, 1, 2, 3]);
    let current: Vec<f64> = poly_areas.iter().map(|a| w * a).collect();
    let baseline = reference_edge_loads(&coords, w, true);

    print_comparison(
        "ケース3: 大きな床 30000x20000 polygon(200x200固定) vs 基準方式",
        w,
        lx * ly,
        &current,
        &baseline,
    );
    let poly_sampled: f64 = poly_areas.iter().sum();
    println!(
        "格子内面積: 現行polygon={poly_sampled:.1} 真値={:.1} セル寸法=150x100",
        lx * ly
    );

    let (model, slab) = make_rect_slab_model(lx, ly, DistributionMethod::TriTrapezoid, w);
    let tri = edge_totals(&distribute_slab(&model, &slab), 4);
    println!("参考 TriTrapezoid(現行矩形経路)[N]: {tri:?}");

    assert_total("現行polygon200", &current, w * lx * ly);
    assert_total("基準方式", &baseline, w * lx * ly);
    assert_pairs_equal(&baseline, &[(0, 2), (1, 3)], "基準方式");
    assert_reference_conserves(&coords, "基準方式");
}

/// 非矩形床の現行 polygon と基準方式を比較して出力する。
fn run_nonrect_case(label: &str, pts: &[(f64, f64)], w: f64) {
    let (model, slab) = polygon_slab_model(pts, DistributionMethod::TriTrapezoid, w);
    let loads = distribute_slab(&model, &slab);
    let n = pts.len();
    let current = edge_totals(&loads, n);
    let coords: Vec<[f64; 3]> = pts.iter().map(|(x, y)| [*x, *y, 0.0]).collect();
    let true_area = area_xy(&coords);
    let baseline = reference_edge_loads(&coords, w, true);

    print_comparison(label, w, true_area, &current, &baseline);

    let poly_sampled = polygon_grid_sampled_area(&coords);
    let (_, base_sampled) = reference_edge_areas(&coords, true);
    println!(
        "格子内面積: 現行polygon200={poly_sampled:.1} 基準={base_sampled:.1} 真値={true_area:.1}"
    );

    assert_total("現行polygon200", &current, w * poly_sampled);
    assert_reference_conserves(&coords, label);
}

/// ケース4: 非矩形床（台形・L形）。現行 polygon と基準方式の各辺負担を比較する。
#[test]
fn case4_nonrect_trapezoid_and_lshape() {
    let w = 0.003_f64;
    run_nonrect_case(
        "ケース4a: 台形 polygon vs 基準方式",
        &[
            (0.0, 0.0),
            (6000.0, 0.0),
            (4000.0, 3000.0),
            (1000.0, 3000.0),
        ],
        w,
    );
    run_nonrect_case(
        "ケース4b: L形 polygon vs 基準方式",
        &[
            (0.0, 0.0),
            (12000.0, 0.0),
            (12000.0, 6000.0),
            (6000.0, 6000.0),
            (6000.0, 12000.0),
            (0.0, 12000.0),
        ],
        w,
    );
}

/// ケース6: 等距離ケース。正方形 4900×4900（49×49 分割）で、等分あり／なしを比較する。
#[test]
fn case6_equal_distance_split_vs_unsplit() {
    let w = 0.005_f64;
    let side = 4900.0_f64;
    let coords: Vec<[f64; 3]> = vec![
        [0.0, 0.0, 0.0],
        [side, 0.0, 0.0],
        [side, side, 0.0],
        [0.0, side, 0.0],
    ];
    let split = reference_edge_loads(&coords, w, true);
    let unsplit = reference_edge_loads(&coords, w, false);

    println!("--- ケース6: 等距離ケース 正方形 4900x4900 (49x49分割) ---");
    println!("辺   等分あり[N]      等分なし[N]");
    for i in 0..4 {
        println!("{i:<3} {:15.3} {:15.3}", split[i], unsplit[i]);
    }
    println!(
        "等分あり: max-min={:.3e}  等分なし: max-min={:.3e}",
        split.iter().cloned().fold(f64::MIN, f64::max)
            - split.iter().cloned().fold(f64::MAX, f64::min),
        unsplit.iter().cloned().fold(f64::MIN, f64::max)
            - unsplit.iter().cloned().fold(f64::MAX, f64::min),
    );

    assert_total("等分あり", &split, w * side * side);
    assert_total("等分なし", &unsplit, w * side * side);
    assert_symmetric(&split, "等分あり");
    let unsplit_token = unsplit.iter().cloned().fold(f64::MIN, f64::max)
        - unsplit.iter().cloned().fold(f64::MAX, f64::min);
    assert!(
        unsplit_token > 0.0,
        "等分なしは対称形でも非対称になるはず: {unsplit:?}"
    );
    assert_reference_conserves(&coords, "等距離ケース");
}

/// ケース5の床モデルの大梁（辺→節点）。
const GIRDER_ENDS: [(usize, usize); 4] = [(0, 1), (1, 2), (2, 3), (3, 0)];

/// ケース5の中央小梁の材軸（X=2500 の鉛直線）。
const JOIST_A: [f64; 2] = [2500.0, 0.0];
/// ケース5の中央小梁の材軸（上端）。
const JOIST_B: [f64; 2] = [2500.0, 4000.0];

/// ケース5の小梁が境界として載る辺の分類。
enum EdgeClass {
    Girder(usize),
    Joist,
}

/// 点がどの大梁（`GIRDER_ENDS` の順）の材軸上にあるかを返す。
fn girder_of_point(model: &Model, p: [f64; 2]) -> Option<usize> {
    for (idx, &(i, j)) in GIRDER_ENDS.iter().enumerate() {
        let a = model.nodes.get(i)?.coord;
        let b = model.nodes.get(j)?.coord;
        let d2 = geom_polygon::point_segment_dist_sq(p, [a[0], a[1]], [b[0], b[1]]);
        if d2 <= 1.0 {
            return Some(idx);
        }
    }
    None
}

/// 床板の辺 `e` が大梁か中央小梁かを、辺中点の位置から判定する。
fn classify_edge(coords: &[[f64; 3]], e: usize, model: &Model) -> Option<EdgeClass> {
    let a = coords[e];
    let b = coords[(e + 1) % coords.len()];
    let mid = [0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])];
    if geom_polygon::point_segment_dist(mid, JOIST_A, JOIST_B) <= 1.0 {
        return Some(EdgeClass::Joist);
    }
    girder_of_point(model, mid).map(EdgeClass::Girder)
}

/// ケース5のモデル（6000×4000 の床領域と X=2500 の中央小梁）を作る。
fn joist_floor_model(w: f64) -> (Model, SlabId, SlabId) {
    let mk_node = |id: u32, x: f64, y: f64| Node {
        id: NodeId(id),
        coord: [x, y, 0.0],
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    };
    let mk_beam = |id: u32, i: u32, j: u32| ElementData {
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
    };
    let nodes = vec![
        mk_node(0, 0.0, 0.0),
        mk_node(1, 6000.0, 0.0),
        mk_node(2, 6000.0, 4000.0),
        mk_node(3, 0.0, 4000.0),
        mk_node(4, 2500.0, 0.0),
        mk_node(5, 2500.0, 4000.0),
    ];
    let elements = vec![
        mk_beam(0, 0, 1),
        mk_beam(1, 1, 2),
        mk_beam(2, 2, 3),
        mk_beam(3, 3, 0),
    ];
    let plate = SlabPlate {
        loads: vec![AreaLoad {
            kind: "DL".into(),
            value: w,
        }],
        method: DistributionMethod::TriTrapezoid,
        ..Default::default()
    };
    let mut region = FloorRegion::new(
        FloorRegionId(0),
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
    );
    region.secondary_joists = vec![SecondaryMember {
        id: SecondaryMemberId(4),
        gravity_end_shares: None,
        kind: SecondaryMemberKind::Joist,
        ends: SecondaryMemberEnds::Supported([
            SecondaryMemberAnchor {
                support: SupportMemberId::Primary(ElemId(0)),
                position: 2500.0 / 6000.0,
            },
            SecondaryMemberAnchor {
                support: SupportMemberId::Primary(ElemId(2)),
                position: 3500.0 / 6000.0,
            },
        ]),
        section: None,
        name: "J".into(),
    }];
    let mut model = Model {
        nodes,
        elements,
        floor_regions: vec![region],
        ..Default::default()
    };
    model.rebuild_floor_assignment_regions();
    let left = model
        .assign_enclosed_slab_to_matching_region(
            &[NodeId(0), NodeId(4), NodeId(5), NodeId(3)],
            plate.clone(),
        )
        .expect("左半分の割当領域");
    let right = model
        .assign_enclosed_slab_to_matching_region(
            &[NodeId(4), NodeId(1), NodeId(2), NodeId(5)],
            plate,
        )
        .expect("右半分の割当領域");
    model.floor_regions[0].slab_ids = vec![left, right];
    (model, left, right)
}

/// 現行カスケードの最終主架構反力 [N] を大梁 `GIRDER_ENDS` ごとに集計する。
fn current_primary_by_girder(
    model: &Model,
    transfer: &crate::cascade::SecondaryTransfer,
) -> [f64; 4] {
    let mut out = [0.0_f64; 4];
    for bl in &transfer.leftover_region_loads {
        add_to_girder(&mut out, bl.elem, bl.cmq.q_i + bl.cmq.q_j);
    }
    let (nodal, member) = transfer.primary_loads(model);
    for bl in &member {
        add_to_girder(&mut out, bl.elem, bl.cmq.q_i + bl.cmq.q_j);
    }
    for (node, r) in &nodal {
        let Some(n) = model.nodes.get(node.index()) else {
            continue;
        };
        if let Some(idx) = girder_of_point(model, [n.coord[0], n.coord[1]]) {
            out[idx] += *r;
        }
    }
    out
}

fn add_to_girder(out: &mut [f64; 4], elem: ElemId, total: f64) {
    let idx = elem.0 as usize;
    if idx < out.len() {
        out[idx] += total;
    }
}

/// 基準方式の辺負担を小梁の単純梁反力（両端等分）に換算して主架構ごとに集計する。
fn reference_primary_by_girder(model: &Model, slabs: &[SlabId], w: f64) -> [f64; 4] {
    let mut out = [0.0_f64; 4];
    let mut joist_total = 0.0_f64;
    for &sid in slabs {
        let Some(slab) = model.slab(sid) else {
            continue;
        };
        let Some(coords) = slab.boundary_coords(model) else {
            continue;
        };
        for (e, load) in reference_edge_loads(&coords, w, true).iter().enumerate() {
            match classify_edge(&coords, e, model) {
                Some(EdgeClass::Girder(i)) => out[i] += load,
                Some(EdgeClass::Joist) => joist_total += load,
                None => {}
            }
        }
    }
    out[0] += 0.5 * joist_total;
    out[2] += 0.5 * joist_total;
    out
}

/// ケース5: 小梁を含む床の最終主架構反力を、現行カスケードと基準方式で比較する。
#[test]
fn case5_joist_floor_primary_reactions_current_vs_reference() {
    let w = 0.005_f64;
    let area = 6000.0 * 4000.0;
    let (model, left, right) = joist_floor_model(w);

    let region_loads = distribute_region(&model, &model.floor_regions[0], |_| w);
    assert_total("distribute_region", &[total_load(&region_loads)], w * area);
    let (joist_map, _) = secondary_joist_distribution_split(&model, |_| w);
    let joist_load_count = joist_map
        .get(&SecondaryMemberId(4))
        .map(|e| e.member_loads.len())
        .unwrap_or(0);
    assert!(joist_load_count > 0, "小梁に分配荷重がある");

    let transfer = crate::cascade::solve(&model, |_| w, false);
    assert!(transfer.unresolved.is_empty(), "{:?}", transfer.unresolved);
    let current = current_primary_by_girder(&model, &transfer);
    let baseline = reference_primary_by_girder(&model, &[left, right], w);
    let current_total: f64 = current.iter().sum();
    let baseline_total: f64 = baseline.iter().sum();

    println!("--- ケース5: 小梁を含む床 6000x4000 (小梁 X=2500) ---");
    println!("主架構   現行[N]          基準[N]          差[%]      判定");
    for (i, name) in ["G0(下)", "G1(右)", "G2(上)", "G3(左)"].iter().enumerate() {
        let c = current[i];
        let b = baseline[i];
        let d = if b.abs() > 1e-12 {
            100.0 * (c - b) / b
        } else {
            0.0
        };
        let side = if (c - b).abs() <= b.abs() * 1e-9 {
            "一致"
        } else if c < b {
            "現行<基準(過小=危険側)"
        } else {
            "現行>基準(過大=安全側)"
        };
        println!("{name:<8} {c:15.3} {b:15.3} {d:9.4}  {side}");
    }
    println!(
        "合計: 現行={current_total:.6} 基準={baseline_total:.6} 真値={:.6}",
        w * area
    );

    assert_total("現行カスケード合計", &[current_total], w * area);
    assert_total("基準方式合計", &[baseline_total], w * area);
}

/// 2 値の相対差 [%]。両者がほぼ零なら 0 とし、分母は絶対値の大きい方を採る。
fn rel_diff_pct(a: f64, b: f64) -> f64 {
    let denom = a.abs().max(b.abs());
    if denom > 1e-12 {
        100.0 * (a - b) / denom
    } else {
        0.0
    }
}

/// `a` の `b` に対する相対誤差。
fn rel_err(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-12)
}

/// 現行・等分版・基準方式の各辺負担荷重 [N] と辺ごとの差 [N]/[%]、最大相対差、総和を出力する。
fn print_three_way(
    label: &str,
    w: f64,
    true_area: f64,
    current: &[f64],
    split: &[f64],
    baseline: &[f64],
) {
    println!("--- {label} ---");
    println!(
        "辺   現行[N]        等分[N]        基準[N]      現行-等分[N] 等分-基準[N] 現行-等分[%] 等分-基準[%]"
    );
    let mut max_current_split = 0.0_f64;
    let mut max_split_base = 0.0_f64;
    for (i, &c) in current.iter().enumerate() {
        let s = split.get(i).copied().unwrap_or(0.0);
        let b = baseline.get(i).copied().unwrap_or(0.0);
        let d_cs = rel_diff_pct(c, s);
        let d_sb = rel_diff_pct(s, b);
        max_current_split = max_current_split.max(d_cs.abs());
        max_split_base = max_split_base.max(d_sb.abs());
        println!(
            "{i:<3} {c:13.3} {s:13.3} {b:13.3} {:12.3} {:12.3} {d_cs:12.4} {d_sb:12.4}",
            c - s,
            s - b,
        );
    }
    let cur_total: f64 = current.iter().sum();
    let split_total: f64 = split.iter().sum();
    let base_total: f64 = baseline.iter().sum();
    let expected = w * true_area;
    println!("最大相対差: 現行vs等分={max_current_split:.4}%  等分vs基準={max_split_base:.4}%");
    println!(
        "総和[N]: 現行={cur_total:.6} 等分={split_total:.6} 基準={base_total:.6} 真値={expected:.6}"
    );
    println!(
        "総和誤差(真値比): 現行={:.3e} 等分={:.3e} 基準={:.3e}",
        rel_err(cur_total, expected),
        rel_err(split_total, expected),
        rel_err(base_total, expected),
    );
}

/// 3方式（現行 `polygon_edge_areas`・等分版 `current_split_edge_areas`・基準方式
/// `reference_edge_areas(split_ties=true)`）を比較して出力する。総和保存と、`symmetry_pairs`
/// で指定した対称辺の等分版の等値を厳格に検証する。
fn run_split_case(label: &str, pts: &[(f64, f64)], w: f64, symmetry_pairs: &[(usize, usize)]) {
    let n = pts.len();
    let coords: Vec<[f64; 3]> = pts.iter().map(|(x, y)| [*x, *y, 0.0]).collect();
    let true_area = area_xy(&coords);
    let candidate: Vec<usize> = (0..n).collect();
    let current_areas = polygon_edge_areas(&coords, &candidate);
    let split_areas = current_split_edge_areas(&coords);
    let (baseline_areas, baseline_sampled) = reference_edge_areas(&coords, true);
    let sampled = polygon_grid_sampled_area(&coords);

    let current: Vec<f64> = current_areas.iter().map(|a| w * a).collect();
    let split: Vec<f64> = split_areas.iter().map(|a| w * a).collect();
    let baseline: Vec<f64> = baseline_areas.iter().map(|a| w * a).collect();

    print_three_way(label, w, true_area, &current, &split, &baseline);
    println!(
        "格子内面積: 現行/等分200x200={sampled:.1} 基準={baseline_sampled:.1} 真値={true_area:.1}"
    );

    assert_total("現行polygon200", &current_areas, sampled);
    assert_total("等分版", &split_areas, sampled);
    assert_reference_conserves(&coords, label);

    if !symmetry_pairs.is_empty() {
        assert_pairs_equal(&split, symmetry_pairs, &format!("{label} 等分版"));
    }
}

/// ケース7: 現行に等距離の均等割りを足した「等分版」を、L形・T形・十字形・
/// 凸五角形・正八角形・U字形状で現行・基準方式と比較する。
#[test]
fn case7_split_ties_shape_matrix() {
    let w = 0.003_f64;

    // L形: 12000×12000 のうち右上 6000×6000 を欠く。対角線 y=x について対称。
    run_split_case(
        "ケース7a: L形 12000x12000（右上6000x6000欠）",
        &[
            (0.0, 0.0),
            (12000.0, 0.0),
            (12000.0, 6000.0),
            (6000.0, 6000.0),
            (6000.0, 12000.0),
            (0.0, 12000.0),
        ],
        w,
        &[(0, 5), (1, 4), (2, 3)],
    );

    // T形（凹形状）: 下辺 12000×6000 の上に 6000×6000 が中央に乗る。x=6000 について対称。
    run_split_case(
        "ケース7b: T形（凹形状）12000x12000",
        &[
            (0.0, 0.0),
            (12000.0, 0.0),
            (12000.0, 6000.0),
            (9000.0, 6000.0),
            (9000.0, 12000.0),
            (3000.0, 12000.0),
            (3000.0, 6000.0),
            (0.0, 6000.0),
        ],
        w,
        &[(1, 7), (2, 6), (3, 5)],
    );

    // 十字形: 12000×12000 から四隅 3000×3000 を欠く。凹角を4つ持つ。
    run_split_case(
        "ケース7c: 十字形 12000x12000（四隅3000x3000欠）",
        &[
            (3000.0, 0.0),
            (9000.0, 0.0),
            (9000.0, 3000.0),
            (12000.0, 3000.0),
            (12000.0, 9000.0),
            (9000.0, 9000.0),
            (9000.0, 12000.0),
            (3000.0, 12000.0),
            (3000.0, 9000.0),
            (0.0, 9000.0),
            (0.0, 3000.0),
            (3000.0, 3000.0),
        ],
        w,
        &[(1, 11), (2, 10), (3, 9), (4, 8), (5, 7)],
    );

    // 凸五角形: 既存 test_polygon_pentagon_conservation と同じ形状。対称性なし。
    run_split_case(
        "ケース7d: 凸五角形",
        &[
            (0.0, 0.0),
            (5000.0, 0.0),
            (6000.0, 3000.0),
            (2500.0, 5000.0),
            (-1000.0, 3000.0),
        ],
        w,
        &[],
    );

    // 正八角形: 半径 6000、頂点角 22.5°+45°k。45°回転対称は格子が保存しないため、
    // 90°回転で移る辺（0-2-4-6、1-3-5-7）のみ対称辺として検証する。
    let r = 6000.0_f64;
    let (c, s) = (22.5_f64.to_radians().cos(), 22.5_f64.to_radians().sin());
    let octagon = [
        (r * c, r * s),
        (r * s, r * c),
        (-r * s, r * c),
        (-r * c, r * s),
        (-r * c, -r * s),
        (-r * s, -r * c),
        (r * s, -r * c),
        (r * c, -r * s),
    ];
    run_split_case(
        "ケース7e: 正八角形（半径6000）",
        &octagon,
        w,
        &[(0, 2), (2, 4), (4, 6), (1, 3), (3, 5), (5, 7)],
    );

    // U字形状: 12000×12000 の上辺中央 6000×8000 を欠く。凹角を2つ持つ。
    run_split_case(
        "ケース7f: U字形状 12000x12000（上辺中央6000x8000欠）",
        &[
            (0.0, 0.0),
            (12000.0, 0.0),
            (12000.0, 12000.0),
            (9000.0, 12000.0),
            (9000.0, 4000.0),
            (3000.0, 4000.0),
            (3000.0, 12000.0),
            (0.0, 12000.0),
        ],
        w,
        &[(1, 7), (2, 6), (3, 5)],
    );
}
