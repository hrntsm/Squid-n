//! 多角形床の分配戦略（最近接辺グリッドサンプリングによる負担面積法）。
//!
//! - [`distribute_polygon`] — 矩形でない凸/凹多角形床の分配（全辺負担）

use squid_n_core::geom::polygon as geom_polygon;

use super::fem::fem_uniform;
use super::geometry::edge_len;
use super::types::{push_edge, BeamLoad, LoadShape};

const POLY_GRID_N: usize = 200;

/// 等距離とみなす許容差の、格子セル寸法に対する相対値。
const TIE_REL_TOL: f64 = 1e-9;

/// 矩形でない凸（または単純な凹）多角形床の分配（レビュー §1.13 ギャップ「多角形床組」対応）。
///
/// 45°法の一般化として「各点を最も近い辺に帰属させる」負担面積法を、多角形の
/// バウンディングボックスを `POLY_GRID_N × POLY_GRID_N`（200×200）に格子分割した
/// 決定的なサンプリングで近似する。各セル中心が多角形内部なら、その中心から最も近い
/// 辺（線分）へセル面積を加算する。最近接辺が等距離のときは、最小距離に並ぶ辺へ
/// セル面積を均等に配分する。辺ごとの負担面積が求まったら、等価等分布
/// `w_line = W_edge / L_edge`（`W_edge = w × 辺の負担面積`）として `LoadShape::Uniform` +
/// `fem_uniform` で返す。
///
/// 荷重保存は「サンプル点の全数帰属」により、格子内部と判定された点の面積の総和について
/// 厳密に成り立つ（Σ辺負担荷重 = w × Σ格子内サンプル面積）。格子内サンプル面積と真の
/// 多角形面積（[`geom_polygon::area_xy`]）との差は格子近似誤差のみで、十分細かい分割（200×200）で
/// 1%未満に収まる（凸多角形で確認）。
pub(crate) fn distribute_polygon(coords: &[[f64; 3]], w: f64, loads: &mut Vec<BeamLoad>) {
    let n = coords.len();
    if n < 3 {
        return;
    }
    let candidate_edges: Vec<usize> = (0..n).collect();
    let edge_area = polygon_edge_areas(coords, &candidate_edges);
    emit_edge_loads(coords, w, &edge_area, loads);
}

/// 多角形の各辺への負担面積を、格子サンプリングで求める（[`distribute_polygon`] と
/// 共通処理）。各セル中心が多角形内部なら、
/// `candidate_edges` の中で最も近い辺（線分）へセル面積を加算する。最近接辺が等距離の
/// ときは、最小距離に並ぶ辺へセル面積を均等に配分する。
/// `candidate_edges` に全辺（`0..n`）を渡せば [`distribute_polygon`] と同じ挙動になり、
/// 部分集合を渡せば非候補の辺には荷重が帰属しなくなる（取り付く床板の支持辺分配
/// [`super::cantilever::distribute_cantilever`] が使う）。
pub(crate) fn polygon_edge_areas(coords: &[[f64; 3]], candidate_edges: &[usize]) -> Vec<f64> {
    let n = coords.len();
    let mut edge_area = vec![0.0_f64; n];
    if candidate_edges.is_empty() {
        return edge_area;
    }
    let poly2: Vec<[f64; 2]> = coords.iter().map(|c| [c[0], c[1]]).collect();
    let (lo, hi) = geom_polygon::bounding_box(&poly2);
    let (min_x, max_x, min_y, max_y) = (lo[0], hi[0], lo[1], hi[1]);
    let dx = (max_x - min_x) / POLY_GRID_N as f64;
    let dy = (max_y - min_y) / POLY_GRID_N as f64;
    if dx <= 0.0 || dy <= 0.0 {
        return edge_area;
    }
    let cell_area = dx * dy;
    let tie_tol = dx.max(dy) * TIE_REL_TOL;
    let mut dists_sq = vec![0.0_f64; n];
    for iy in 0..POLY_GRID_N {
        let y = min_y + (iy as f64 + 0.5) * dy;
        for ix in 0..POLY_GRID_N {
            let x = min_x + (ix as f64 + 0.5) * dx;
            let p = [x, y];
            if !geom_polygon::contains_by_ray_crossing(&poly2, p) {
                continue;
            }
            let mut d_min_sq = f64::INFINITY;
            for &e in candidate_edges {
                let a = poly2[e];
                let b = poly2[(e + 1) % n];
                let d2 = geom_polygon::point_segment_dist_sq(p, a, b);
                dists_sq[e] = d2;
                d_min_sq = d_min_sq.min(d2);
            }
            let tie_limit_sq = tie_tol * (2.0 * d_min_sq.sqrt() + tie_tol);
            let tie_count = candidate_edges
                .iter()
                .filter(|&&e| dists_sq[e] <= d_min_sq + tie_limit_sq)
                .count();
            let share = cell_area / tie_count as f64;
            for &e in candidate_edges {
                if dists_sq[e] <= d_min_sq + tie_limit_sq {
                    edge_area[e] += share;
                }
            }
        }
    }
    edge_area
}

/// 辺ごとの負担面積 `edge_area`（[`polygon_edge_areas`] の出力）を、等価等分布
/// `w_line = W_edge / L_edge`（`W_edge = w × edge_area[e]`）の辺荷重として `loads` へ追加する。
fn emit_edge_loads(coords: &[[f64; 3]], w: f64, edge_area: &[f64], loads: &mut Vec<BeamLoad>) {
    for (e, &a_e) in edge_area.iter().enumerate() {
        if a_e <= 0.0 {
            continue;
        }
        let l_e = edge_len(coords, e);
        if l_e <= 1e-9 {
            continue;
        }
        let w_edge = w * a_e;
        let w_line = w_edge / l_e;
        push_edge(
            loads,
            e,
            LoadShape::Uniform { w: w_line },
            fem_uniform(w_line, l_e),
        );
    }
}
