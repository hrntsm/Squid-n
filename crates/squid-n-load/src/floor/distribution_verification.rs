//! 床荷重のXY両方向分配の検証: 有限線分拡張方式を本モジュール内だけに再現し、現行方式と比較する。
//! 有限線分拡張方式は「一辺 100mm 以下の格子で各セル重心を最近接辺へ帰属し、等距離の辺には
//! 等分する」ものとする。辺を有限線分とみなす距離解釈は原典（凸四辺形対象）を凹多角形へ
//! 拡張した Squid-n の仮定であり、無限直線方式は感度分析用に併記する。本番の分配ロジックは
//! 変更しない。

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

/// 格子の最大辺長 [mm]。
const MAX_CELL_MM: f64 = 100.0;

/// 等距離とみなす許容差の、セル寸法に対する相対値。
const TIE_REL_TOL: f64 = 1e-9;

/// 点 `p` から、辺 `a`–`b` を含む無限直線までの距離 [mm]。
///
/// 垂線の足が線分外にあっても端点へクランプせず、直線上の足までの距離
/// `|cross(ab, ap)| / |ab|` を返す。
fn point_supporting_line_dist(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let len = (ab[0] * ab[0] + ab[1] * ab[1]).sqrt();
    if len <= f64::EPSILON {
        return geom_polygon::point_segment_dist(p, a, b);
    }
    let ap = [p[0] - a[0], p[1] - a[1]];
    (ab[0] * ap[1] - ab[1] * ap[0]).abs() / len
}

/// `point_supporting_line_dist` の手計算検証。無限直線距離と有限線分距離
/// （`geom_polygon::point_segment_dist`）の違いを直接固定する。
#[test]
fn supporting_line_distance_hand_calc() {
    let a = [0.0, 0.0];
    let b = [10.0, 0.0];

    // 垂線の足が線分内: (5,3) の距離は 3。
    let d_inside = point_supporting_line_dist([5.0, 3.0], a, b);
    assert!((d_inside - 3.0).abs() < 1e-9, "{d_inside}");

    // 垂線の足が線分外（正側）: 無限直線は 4、有限線分は √(10²+4²)=√116。
    let d_out_pos = point_supporting_line_dist([20.0, 4.0], a, b);
    assert!((d_out_pos - 4.0).abs() < 1e-9, "{d_out_pos}");
    let seg_out_pos = geom_polygon::point_segment_dist([20.0, 4.0], a, b);
    assert!(
        (seg_out_pos - 116.0_f64.sqrt()).abs() < 1e-9,
        "{seg_out_pos}"
    );

    // 垂線の足が線分外（負側）: 無限直線は 2、有限線分は √(5²+2²)=√29。
    let d_out_neg = point_supporting_line_dist([-5.0, 2.0], a, b);
    assert!((d_out_neg - 2.0).abs() < 1e-9, "{d_out_neg}");
    let seg_out_neg = geom_polygon::point_segment_dist([-5.0, 2.0], a, b);
    assert!(
        (seg_out_neg - 29.0_f64.sqrt()).abs() < 1e-9,
        "{seg_out_neg}"
    );

    // 退化（線分長ゼロ）: 有限線分距離へフォールバックし (3,7) の距離は 4。
    let d_degenerate = point_supporting_line_dist([3.0, 7.0], [3.0, 3.0], [3.0, 3.0]);
    assert!((d_degenerate - 4.0).abs() < 1e-9, "{d_degenerate}");
}

/// 格子サンプリングによる各辺負担面積 [mm²] と、格子内と判定した総面積 [mm²]。
///
/// バウンディングボックスを `ceil(幅/100) × ceil(高さ/100)` に分割し、各セル中心が
/// 多角形内部なら最近接辺へセル面積を加算する。`split_ties` が真なら最小距離に並ぶ
/// 複数の辺へ均等に、偽なら最初に見つかった最小距離の辺へ全量を加算する。
/// `use_supporting_line` が真なら辺を無限直線とみなし、偽なら有限線分（端点クランプ）
/// として距離を測る。
fn grid_edge_areas(
    coords: &[[f64; 3]],
    split_ties: bool,
    use_supporting_line: bool,
) -> (Vec<f64>, f64) {
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
                let (a, b) = (poly[e], poly[(e + 1) % n]);
                *d = if use_supporting_line {
                    point_supporting_line_dist(p, a, b)
                } else {
                    geom_polygon::point_segment_dist(p, a, b)
                };
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

/// 凸四辺形スラブを対象とする手計算照合の各辺負担面積 [mm²] と、格子内総面積 [mm²]。
///
/// 原典（基準資料 2.2.2 (3)）が対象とする凸四辺形に限り、各辺の累積負担が有限線分距離と
/// 無限直線距離で一致することを内部で assert したうえで、有限線分距離・等分ありの結果を返す。
fn manual_quadrilateral_edge_areas(coords: &[[f64; 3]]) -> (Vec<f64>, f64) {
    assert_eq!(coords.len(), 4, "凸四辺形専用");
    let (segment, sampled) = grid_edge_areas(coords, true, false);
    let (line, _) = grid_edge_areas(coords, true, true);
    for (e, (a, b)) in segment.iter().zip(line.iter()).enumerate() {
        let rel = (a - b).abs() / a.abs().max(b.abs()).max(1e-12);
        assert!(
            rel < 1e-9,
            "凸四辺形では各辺の累積負担が有限線分距離と無限直線距離で一致するはず: 辺{e} 線分={a} 直線={b}"
        );
    }
    (segment, sampled)
}

/// 有限線分距離（端点クランプ）を凹多角形へ拡張した仮定での各辺負担面積 [mm²]。
///
/// `split_ties` が真なら等距離の辺へ均等割りし、偽なら先勝ちで 1 辺へ帰属する。
fn segment_extension_edge_areas(coords: &[[f64; 3]], split_ties: bool) -> (Vec<f64>, f64) {
    grid_edge_areas(coords, split_ties, false)
}

/// 辺を無限直線とみなす感度分析用の各辺負担面積 [mm²] と、格子内総面積 [mm²]。
fn supporting_line_sensitivity_edge_areas(
    coords: &[[f64; 3]],
    split_ties: bool,
) -> (Vec<f64>, f64) {
    grid_edge_areas(coords, split_ties, true)
}

/// 有限線分拡張方式の辺ごとの負担荷重 [N]。辺インデックス順の `Vec<f64>` で返す。
fn segment_extension_edge_loads(coords: &[[f64; 3]], w: f64, split_ties: bool) -> Vec<f64> {
    segment_extension_edge_areas(coords, split_ties)
        .0
        .iter()
        .map(|a| w * a)
        .collect()
}

/// 無限直線方式（感度分析用）の辺ごとの負担荷重 [N]。
fn supporting_line_edge_loads(coords: &[[f64; 3]], w: f64, split_ties: bool) -> Vec<f64> {
    supporting_line_sensitivity_edge_areas(coords, split_ties)
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

/// 有限線分拡張方式の辺負担面積の総和が、格子内と判定した面積に厳密（相対 1e-9）に
/// 一致することを確認する（等距離の等分で面積が落ちないこと）。
fn assert_reference_conserves(coords: &[[f64; 3]], label: &str) {
    let (areas, sampled) = segment_extension_edge_areas(coords, true);
    let sum: f64 = areas.iter().sum();
    assert!(
        (sum - sampled).abs() <= 1e-9 * sampled.max(1.0),
        "{label}: 面積の総和={sum} 格子内面積={sampled}"
    );
}

/// 現行と有限線分拡張方式の辺負担を表と総和で出力する。`true_area` は多角形の真の面積 [mm²]。
fn print_comparison(label: &str, w: f64, true_area: f64, current: &[f64], baseline: &[f64]) {
    let cur_total: f64 = current.iter().sum();
    let base_total: f64 = baseline.iter().sum();
    println!("--- {label} ---");
    println!("辺   現行[N]          拡張方式[N]      差[%]      判定");
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
            "現行<拡張(過小=危険側)"
        } else {
            "現行>拡張(過大=安全側)"
        };
        println!("{i:<3} {c:15.3} {b:15.3} {d:9.4}  {side}");
    }
    let expected = w * true_area;
    println!(
        "総和: 現行={cur_total:.6} 拡張={base_total:.6} 真値={expected:.6} 現行誤差={:.3e} 拡張誤差={:.3e}",
        (cur_total - expected).abs() / expected.abs().max(1e-12),
        (base_total - expected).abs() / expected.abs().max(1e-12),
    );
}

/// ケース1: 正方形 6000×6000。TriTrapezoid と有限線分拡張方式の各辺負担を比較する。
#[test]
fn case1_square_6000_tritrapezoid_vs_reference() {
    let w = 0.005_f64;
    let side = 6000.0_f64;
    let (model, slab) = make_square_slab_model(side, DistributionMethod::TriTrapezoid, w);
    let loads = distribute_slab(&model, &slab);
    let current = edge_totals(&loads, 4);
    let coords = slab.boundary_coords(&model).expect("境界座標");
    let (baseline_areas, _) = manual_quadrilateral_edge_areas(&coords);
    let baseline: Vec<f64> = baseline_areas.iter().map(|a| w * a).collect();

    print_comparison(
        "ケース1: 正方形 6000x6000 TriTrapezoid vs 有限線分拡張方式",
        w,
        side * side,
        &current,
        &baseline,
    );

    assert_total("現行TriTrapezoid", &current, w * side * side);
    assert_symmetric(&current, "現行TriTrapezoid");
    assert_symmetric(&baseline, "有限線分拡張方式");
    assert_reference_conserves(&coords, "有限線分拡張方式");
    assert_max_rel_diff_below(
        &current,
        &baseline,
        1e-9,
        "ケース1: 現行TriTrapezoid と有限線分拡張方式",
    );
}

/// ケース2: 長方形 6000×4000。TriTrapezoid と有限線分拡張方式の各辺負担を比較する。
#[test]
fn case2_rect_6000x4000_tritrapezoid_vs_reference() {
    let w = 0.005_f64;
    let (lx, ly) = (6000.0_f64, 4000.0_f64);
    let (model, slab) = make_rect_slab_model(lx, ly, DistributionMethod::TriTrapezoid, w);
    let loads = distribute_slab(&model, &slab);
    let current = edge_totals(&loads, 4);
    let coords = slab.boundary_coords(&model).expect("境界座標");
    let (baseline_areas, _) = manual_quadrilateral_edge_areas(&coords);
    let baseline: Vec<f64> = baseline_areas.iter().map(|a| w * a).collect();

    print_comparison(
        "ケース2: 長方形 6000x4000 TriTrapezoid vs 有限線分拡張方式",
        w,
        lx * ly,
        &current,
        &baseline,
    );

    assert_total("現行TriTrapezoid", &current, w * lx * ly);
    assert_pairs_equal(&baseline, &[(0, 2), (1, 3)], "有限線分拡張方式");
    assert_reference_conserves(&coords, "有限線分拡張方式");
    assert_max_rel_diff_below(
        &current,
        &baseline,
        1e-9,
        "ケース2: 現行TriTrapezoid と有限線分拡張方式",
    );
}

/// ケース3: 大きな床 30000×20000。現行 polygon の固定 200×200（セル 150×100）と
/// 有限線分拡張方式（100mm 以下）の各辺負担を比較し、セル寸法が 100mm を超える影響を見る。
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
    let baseline = segment_extension_edge_loads(&coords, w, true);

    print_comparison(
        "ケース3: 大きな床 30000x20000 polygon(200x200固定) vs 有限線分拡張方式",
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
    assert_total("有限線分拡張方式", &baseline, w * lx * ly);
    assert_pairs_equal(&baseline, &[(0, 2), (1, 3)], "有限線分拡張方式");
    assert_reference_conserves(&coords, "有限線分拡張方式");
}

/// 非矩形床の現行 polygon と有限線分拡張方式を比較して出力する。
fn run_nonrect_case(label: &str, pts: &[(f64, f64)], w: f64) {
    let (model, slab) = polygon_slab_model(pts, DistributionMethod::TriTrapezoid, w);
    let loads = distribute_slab(&model, &slab);
    let n = pts.len();
    let current = edge_totals(&loads, n);
    let coords: Vec<[f64; 3]> = pts.iter().map(|(x, y)| [*x, *y, 0.0]).collect();
    let true_area = area_xy(&coords);
    let baseline = segment_extension_edge_loads(&coords, w, true);

    print_comparison(label, w, true_area, &current, &baseline);

    let poly_sampled = polygon_grid_sampled_area(&coords);
    let (_, base_sampled) = segment_extension_edge_areas(&coords, true);
    println!(
        "格子内面積: 現行polygon200={poly_sampled:.1} 有限線分拡張方式={base_sampled:.1} 真値={true_area:.1}"
    );

    assert_total("現行polygon200", &current, w * poly_sampled);
    assert_reference_conserves(&coords, label);
}

/// ケース4: 非矩形床（台形・L形）。現行 polygon と有限線分拡張方式の各辺負担を比較する。
#[test]
fn case4_nonrect_trapezoid_and_lshape() {
    let w = 0.003_f64;
    run_nonrect_case(
        "ケース4a: 台形 polygon vs 有限線分拡張方式",
        &[
            (0.0, 0.0),
            (6000.0, 0.0),
            (4000.0, 3000.0),
            (1000.0, 3000.0),
        ],
        w,
    );
    run_nonrect_case(
        "ケース4b: L形 polygon vs 有限線分拡張方式",
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
    let split = segment_extension_edge_loads(&coords, w, true);
    let unsplit = segment_extension_edge_loads(&coords, w, false);

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

/// 有限線分拡張方式の辺負担を小梁の単純梁反力（両端等分）に換算して主架構ごとに集計する。
fn segment_extension_primary_by_girder(model: &Model, slabs: &[SlabId], w: f64) -> [f64; 4] {
    let mut out = [0.0_f64; 4];
    let mut joist_total = 0.0_f64;
    for &sid in slabs {
        let Some(slab) = model.slab(sid) else {
            continue;
        };
        let Some(coords) = slab.boundary_coords(model) else {
            continue;
        };
        for (e, load) in segment_extension_edge_loads(&coords, w, true)
            .iter()
            .enumerate()
        {
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

/// ケース5: 小梁を含む床の最終主架構反力を、現行カスケードと有限線分拡張方式で比較する。
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
    let baseline = segment_extension_primary_by_girder(&model, &[left, right], w);
    let current_total: f64 = current.iter().sum();
    let baseline_total: f64 = baseline.iter().sum();

    println!("--- ケース5: 小梁を含む床 6000x4000 (小梁 X=2500) ---");
    println!("主架構   現行[N]          有限線分拡張[N]  差[%]      判定");
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
            "現行<有限線分拡張(過小=危険側)"
        } else {
            "現行>有限線分拡張(過大=安全側)"
        };
        println!("{name:<8} {c:15.3} {b:15.3} {d:9.4}  {side}");
    }
    println!(
        "合計: 現行={current_total:.6} 有限線分拡張={baseline_total:.6} 真値={:.6}",
        w * area
    );

    assert_total("現行カスケード合計", &[current_total], w * area);
    assert_total("有限線分拡張方式合計", &[baseline_total], w * area);
    assert_max_rel_diff_below(
        &current,
        &baseline,
        0.01,
        "ケース5: 現行カスケード と 有限線分拡張方式（最終主架構反力）",
    );
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

/// 辺ごとの相対差の最大値 [%]。分母は `max(|a|,|b|)`。
fn max_rel_diff_pct(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b.iter())
        .map(|(&x, &y)| rel_diff_pct(x, y).abs())
        .fold(0.0_f64, f64::max)
}

/// `a` と `b` の辺ごとの相対差の最大値（比）が `limit` 未満であることを確認する。
fn assert_max_rel_diff_below(a: &[f64], b: &[f64], limit: f64, label: &str) {
    let m = a
        .iter()
        .zip(b.iter())
        .map(|(&x, &y)| {
            let denom = x.abs().max(y.abs());
            if denom > 1e-12 {
                (x - y).abs() / denom
            } else {
                0.0
            }
        })
        .fold(0.0_f64, f64::max);
    assert!(
        m < limit,
        "{label}: 最大相対差={:.4}% が閾値 {:.4}% 以上",
        100.0 * m,
        100.0 * limit
    );
}

/// 現行・等分版・有限線分拡張方式の各辺負担荷重 [N] と辺ごとの差 [N]/[%]、最大相対差、
/// 総和を出力する。
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
        "辺   現行[N]        等分[N]        有限線分拡張[N] 現行-等分[N] 等分-拡張[N] 現行-等分[%] 等分-拡張[%]"
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
    println!("最大相対差: 現行vs等分={max_current_split:.4}%  等分vs拡張={max_split_base:.4}%");
    println!(
        "総和[N]: 現行={cur_total:.6} 等分={split_total:.6} 拡張={base_total:.6} 真値={expected:.6}"
    );
    println!(
        "総和誤差(真値比): 現行={:.3e} 等分={:.3e} 拡張={:.3e}",
        rel_err(cur_total, expected),
        rel_err(split_total, expected),
        rel_err(base_total, expected),
    );
}

/// 3方式（現行 `polygon_edge_areas`・等分版 `current_split_edge_areas`・有限線分拡張方式
/// `segment_extension_edge_areas(split_ties=true)`）を比較して出力する。
/// `current_split_limit`・`split_baseline_limit` はそれぞれ現行対等分版、等分版対有限線分
/// 拡張方式の許容相対差（比）。総和保存と、`symmetry_pairs` で指定した対称辺の等分版の
/// 等値を厳格に検証する。
fn run_split_case(
    label: &str,
    pts: &[(f64, f64)],
    w: f64,
    symmetry_pairs: &[(usize, usize)],
    current_split_limit: Option<f64>,
    split_baseline_limit: Option<f64>,
) {
    let n = pts.len();
    let coords: Vec<[f64; 3]> = pts.iter().map(|(x, y)| [*x, *y, 0.0]).collect();
    let true_area = area_xy(&coords);
    let candidate: Vec<usize> = (0..n).collect();
    let current_areas = polygon_edge_areas(&coords, &candidate);
    let split_areas = current_split_edge_areas(&coords);
    let (baseline_areas, baseline_sampled) = segment_extension_edge_areas(&coords, true);
    let sampled = polygon_grid_sampled_area(&coords);

    let current: Vec<f64> = current_areas.iter().map(|a| w * a).collect();
    let split: Vec<f64> = split_areas.iter().map(|a| w * a).collect();
    let baseline: Vec<f64> = baseline_areas.iter().map(|a| w * a).collect();

    print_three_way(label, w, true_area, &current, &split, &baseline);
    println!(
        "格子内面積: 現行/等分200x200={sampled:.1} 有限線分拡張={baseline_sampled:.1} 真値={true_area:.1}"
    );

    assert_total("現行polygon200", &current_areas, sampled);
    assert_total("等分版", &split_areas, sampled);
    assert_reference_conserves(&coords, label);

    if !symmetry_pairs.is_empty() {
        assert_pairs_equal(&split, symmetry_pairs, &format!("{label} 等分版"));
    }
    if let Some(limit) = current_split_limit {
        assert_max_rel_diff_below(&current, &split, limit, &format!("{label}: 現行 と 等分版"));
    }
    if let Some(limit) = split_baseline_limit {
        assert_max_rel_diff_below(
            &split,
            &baseline,
            limit,
            &format!("{label}: 等分版 と 有限線分拡張方式"),
        );
    }
}

/// ケース7: 現行に等距離の均等割りを足した「等分版」を、L形・T形・十字形・
/// 凸五角形・正八角形・U字形状で現行・有限線分拡張方式と比較する。
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
        None,
        Some(0.01),
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
        None,
        Some(0.01),
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
        None,
        Some(0.01),
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
        Some(1e-9),
        None,
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
        Some(1e-9),
        None,
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
        None,
        Some(0.025),
    );
}

/// ケース7で使う凹形状のうち、ケース8・9で共通に使う代表形状。
const LSHAPE_PTS: [(f64, f64); 6] = [
    (0.0, 0.0),
    (12000.0, 0.0),
    (12000.0, 6000.0),
    (6000.0, 6000.0),
    (6000.0, 12000.0),
    (0.0, 12000.0),
];
const TSHAPE_PTS: [(f64, f64); 8] = [
    (0.0, 0.0),
    (12000.0, 0.0),
    (12000.0, 6000.0),
    (9000.0, 6000.0),
    (9000.0, 12000.0),
    (3000.0, 12000.0),
    (3000.0, 6000.0),
    (0.0, 6000.0),
];
const CROSS_PTS: [(f64, f64); 12] = [
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
];
const USHAPE_PTS: [(f64, f64); 8] = [
    (0.0, 0.0),
    (12000.0, 0.0),
    (12000.0, 12000.0),
    (9000.0, 12000.0),
    (9000.0, 4000.0),
    (3000.0, 4000.0),
    (3000.0, 12000.0),
    (0.0, 12000.0),
];

/// 凸四辺形（正方形・長方形・斜めの凸四辺形）で、各辺の累積負担が有限線分距離と無限直線
/// 距離で全辺相対 1e-9 以内に一致することを確認して数値を出力する。原典との直接照合の根拠となる。
fn check_convex_quad_distance_equivalence(label: &str, pts: &[(f64, f64)], w: f64) {
    let coords: Vec<[f64; 3]> = pts.iter().map(|(x, y)| [*x, *y, 0.0]).collect();
    let (segment, _) = segment_extension_edge_areas(&coords, true);
    let (line, _) = supporting_line_sensitivity_edge_areas(&coords, true);
    println!("--- ケース8: 凸四辺形 {label} 有限線分 vs 無限直線 ---");
    println!("辺   有限線分[N]    無限直線[N]    相対差");
    let mut max_rel = 0.0_f64;
    for e in 0..coords.len() {
        let s = w * segment[e];
        let l = w * line[e];
        let rel = (s - l).abs() / s.abs().max(l.abs()).max(1e-12);
        max_rel = max_rel.max(rel);
        println!("{e:<3} {s:13.3} {l:13.3} {rel:11.3e}");
    }
    println!("最大相対差: {max_rel:.3e}");
    assert!(
        max_rel < 1e-9,
        "{label}: 凸四辺形では各辺の累積負担が有限線分距離と無限直線距離で一致するはず（最大相対差={max_rel:e}）"
    );
}

/// 4 通りの距離・等分の組み合わせによる辺負担 [N] を出力する。
///
/// `line_split_min_rel` は「有限線分・等分」と「無限直線・等分」の最大相対差の下限、
/// `line_unsplit_zero_edge` は「無限直線・先勝」で 0 になるべき辺、
/// `seg_unsplit_min_rel` は「有限線分・等分」と「有限線分・先勝」の最大相対差の下限。
fn print_distance_modes(
    label: &str,
    pts: &[(f64, f64)],
    w: f64,
    line_split_min_rel: f64,
    line_unsplit_zero_edge: Option<usize>,
    seg_unsplit_min_rel: Option<f64>,
) {
    let coords: Vec<[f64; 3]> = pts.iter().map(|(x, y)| [*x, *y, 0.0]).collect();
    let seg_split = segment_extension_edge_loads(&coords, w, true);
    let seg_unsplit = segment_extension_edge_loads(&coords, w, false);
    let line_split = supporting_line_edge_loads(&coords, w, true);
    let line_unsplit = supporting_line_edge_loads(&coords, w, false);
    println!("--- ケース8: {label} 距離解釈の感度分析 ---");
    println!("辺   線分等分[N]    線分先勝[N]    直線等分[N]    直線先勝[N]");
    for e in 0..coords.len() {
        println!(
            "{e:<3} {:13.3} {:13.3} {:13.3} {:13.3}",
            seg_split[e], seg_unsplit[e], line_split[e], line_unsplit[e],
        );
    }
    println!(
        "最大相対差: 線分等分vs直線等分={:.4}%  線分等分vs線分先勝={:.4}%",
        max_rel_diff_pct(&seg_split, &line_split),
        max_rel_diff_pct(&seg_split, &seg_unsplit),
    );
    let area = area_xy(&coords);
    let line_split_rel = max_rel_diff_pct(&seg_split, &line_split) / 100.0;
    assert!(
        line_split_rel > line_split_min_rel,
        "{label}: 有限線分・等分 と 無限直線・等分 の最大相対差={:.4}% は {:.2}% を超えるはず",
        100.0 * line_split_rel,
        100.0 * line_split_min_rel,
    );
    if let Some(e) = line_unsplit_zero_edge {
        assert!(
            line_unsplit[e].abs() <= 1e-9 * w * area,
            "{label}: 無限直線・先勝では辺{e} が 0 になるはず: {:.6}",
            line_unsplit[e],
        );
    }
    if let Some(min_rel) = seg_unsplit_min_rel {
        let seg_unsplit_rel = max_rel_diff_pct(&seg_split, &seg_unsplit) / 100.0;
        assert!(
            seg_unsplit_rel > min_rel,
            "{label}: 有限線分・等分 と 有限線分・先勝 の最大相対差={:.4}% は {:.2}% を超えるはず",
            100.0 * seg_unsplit_rel,
            100.0 * min_rel,
        );
    }
    assert_total("線分等分", &seg_split, w * area);
    assert_total("線分先勝", &seg_unsplit, w * area);
    assert_total("直線等分", &line_split, w * area);
    assert_total("直線先勝", &line_unsplit, w * area);
    assert_reference_conserves(&coords, label);
}

/// ケース8: 距離解釈の感度分析。凸四辺形での両解釈の一致を assert し、凹形状
/// （L形・T形・十字形・U字）で有限線分と無限直線の 4 通りを比較出力する。凹形状では
/// 「有限線分・等分 と 無限直線・等分」の最大相対差が 10% を超えること、T形・U字では
/// 「無限直線・先勝」で同一支持直線上の離れた辺（辺6）が 0 になること、L形では
/// 「有限線分・等分 と 有限線分・先勝」の最大相対差が 10% を超えることを assert する。
#[test]
fn case8_distance_interpretation_sensitivity() {
    let w = 0.003_f64;

    check_convex_quad_distance_equivalence(
        "正方形 6000x6000",
        &[(0.0, 0.0), (6000.0, 0.0), (6000.0, 6000.0), (0.0, 6000.0)],
        w,
    );
    check_convex_quad_distance_equivalence(
        "長方形 6000x4000",
        &[(0.0, 0.0), (6000.0, 0.0), (6000.0, 4000.0), (0.0, 4000.0)],
        w,
    );
    check_convex_quad_distance_equivalence(
        "斜めの凸四辺形",
        &[
            (0.0, 0.0),
            (8000.0, 1000.0),
            (7000.0, 6000.0),
            (500.0, 5000.0),
        ],
        w,
    );

    print_distance_modes("L形 12000x12000", &LSHAPE_PTS, w, 0.10, None, Some(0.10));
    print_distance_modes("T形 12000x12000", &TSHAPE_PTS, w, 0.10, Some(6), None);
    print_distance_modes("十字形 12000x12000", &CROSS_PTS, w, 0.10, None, None);
    print_distance_modes("U字形状 12000x12000", &USHAPE_PTS, w, 0.10, Some(6), None);
}

/// 頂点列を `shift` 個循環移動し、`reverse` なら時計回りへ反転した座標列を返す。
fn transformed_cycle(pts: &[(f64, f64)], shift: usize, reverse: bool) -> Vec<(f64, f64)> {
    let n = pts.len();
    let mut out: Vec<(f64, f64)> = (0..n).map(|i| pts[(i + shift) % n]).collect();
    if reverse {
        out.reverse();
    }
    out
}

/// 辺 `e` の両端座標を昇順ソートしたキー。循環移動・反転で不変な物理辺の識別子。
fn edge_key(pts: &[(f64, f64)], e: usize) -> ([f64; 2], [f64; 2]) {
    let a = pts[e];
    let b = pts[(e + 1) % pts.len()];
    let (lo, hi) = if (a.0, a.1) <= (b.0, b.1) {
        (a, b)
    } else {
        (b, a)
    };
    ([lo.0, lo.1], [hi.0, hi.1])
}

/// 物理辺を表すキー（両端座標のビット列を昇順ソートしたもの）。
type EdgeKey = ([u64; 2], [u64; 2]);

/// 物理辺のキー→負担荷重 [N] のマップ。
type EdgeLoadMap = std::collections::BTreeMap<EdgeKey, f64>;

/// 物理辺のキー→負担荷重 [N] のマップを作る。
fn edge_load_map(pts: &[(f64, f64)], loads: &[f64]) -> EdgeLoadMap {
    let mut map = EdgeLoadMap::new();
    for (e, &load) in loads.iter().enumerate() {
        let (a, b) = edge_key(pts, e);
        map.insert(
            (
                [a[0].to_bits(), a[1].to_bits()],
                [b[0].to_bits(), b[1].to_bits()],
            ),
            load,
        );
    }
    map
}

/// ある計算方式について、各変換パターンの物理辺負担マップを返す。
fn physical_edge_loads(
    base: &[(f64, f64)],
    patterns: &[(usize, bool)],
    w: f64,
    mode: fn(&[[f64; 3]], f64, bool) -> Vec<f64>,
    split_ties: bool,
) -> Vec<EdgeLoadMap> {
    patterns
        .iter()
        .map(|&(shift, reverse)| {
            let pts = transformed_cycle(base, shift, reverse);
            let coords: Vec<[f64; 3]> = pts.iter().map(|(x, y)| [*x, *y, 0.0]).collect();
            let loads = mode(&coords, w, split_ties);
            edge_load_map(&pts, &loads)
        })
        .collect()
}

/// 複数の物理辺負担マップ間の最大相対差が `limit` 未満（不変）であることを確認する。
fn assert_maps_invariant(maps: &[EdgeLoadMap], limit: f64, label: &str) -> bool {
    let mut max_rel = 0.0_f64;
    let reference = &maps[0];
    for map in &maps[1..] {
        for (key, v) in reference {
            let other = map.get(key).copied().unwrap_or(f64::NAN);
            let rel = (v - other).abs() / v.abs().max(other.abs()).max(1e-12);
            max_rel = max_rel.max(rel);
        }
    }
    println!("{label}: 最大相対差={max_rel:.3e}");
    assert!(
        max_rel < limit,
        "{label}: 不変のはずが最大相対差={max_rel:e} >= {limit:e}"
    );
    true
}

/// 複数の物理辺負担マップに差がある（順序依存）ことを確認する。
fn assert_maps_differ(maps: &[EdgeLoadMap], label: &str) -> bool {
    let reference = &maps[0];
    let mut max_rel = 0.0_f64;
    for map in &maps[1..] {
        for (key, v) in reference {
            let other = map.get(key).copied().unwrap_or(f64::NAN);
            let rel = (v - other).abs() / v.abs().max(other.abs()).max(1e-12);
            max_rel = max_rel.max(rel);
        }
    }
    println!("{label}: 最大相対差={max_rel:.3e}");
    assert!(
        max_rel > 1e-6,
        "{label}: 順序依存で差が出るはずが最大相対差={max_rel:e}"
    );
    true
}

/// 一形状について、頂点順序の変換パターンごとに 3 方式の物理辺負担を比較する。
fn check_vertex_order(label: &str, pts: &[(f64, f64)], w: f64) {
    let patterns: Vec<(usize, bool)> = vec![
        (0, false),
        (1, false),
        (2, false),
        (3, false),
        (0, true),
        (1, true),
    ];
    let current = physical_edge_loads(pts, &patterns, w, polygon_current_edge_loads_ref, false);
    let split = physical_edge_loads(pts, &patterns, w, segment_extension_edge_loads_ref, true);
    let line_split = physical_edge_loads(pts, &patterns, w, supporting_line_edge_loads_ref, true);

    println!("--- ケース9: {label} 頂点順序の不変性 ---");
    assert_maps_differ(&current, &format!("{label} 現行（先勝ち）"));
    assert_maps_invariant(&split, 1e-9, &format!("{label} 等分版（線分・等分）"));
    assert_maps_invariant(&line_split, 1e-9, &format!("{label} 無限直線・等分版"));
}

/// `physical_edge_loads` へ渡す有限線分拡張方式の関数ポインタ相当。
fn segment_extension_edge_loads_ref(coords: &[[f64; 3]], w: f64, split_ties: bool) -> Vec<f64> {
    segment_extension_edge_loads(coords, w, split_ties)
}

/// `physical_edge_loads` へ渡す本番の非矩形経路 [`polygon_edge_areas`]（200×200 固定格子・
/// 先勝ち）の関数ポインタ相当。辺ごとの負担荷重 [N] を返す。`split_ties` は現行実装に
/// 等距離の均等割りがないため用いない。
fn polygon_current_edge_loads_ref(coords: &[[f64; 3]], w: f64, _split_ties: bool) -> Vec<f64> {
    let candidates: Vec<usize> = (0..coords.len()).collect();
    polygon_edge_areas(coords, &candidates)
        .iter()
        .map(|a| w * a)
        .collect()
}

/// `physical_edge_loads` へ渡す無限直線方式の関数ポインタ相当。
fn supporting_line_edge_loads_ref(coords: &[[f64; 3]], w: f64, split_ties: bool) -> Vec<f64> {
    supporting_line_edge_loads(coords, w, split_ties)
}

/// ケース9: L形と十字形について、頂点列の循環移動・反転で本番の `polygon_edge_areas`
/// （200×200 固定格子・先勝ち）の物理辺負担が変わること（L形 3.701e-1・十字形 6.755e-1）、
/// 等分版・無限直線等分版が変わらないことを assert する。
#[test]
fn case9_vertex_order_invariance() {
    let w = 0.003_f64;
    check_vertex_order("L形 12000x12000", &LSHAPE_PTS, w);
    check_vertex_order("十字形 12000x12000", &CROSS_PTS, w);
}
