//! 多角形床の分配戦略（最近接辺グリッドサンプリングによる負担面積法）。
//!
//! - [`distribute_polygon`] — 矩形でない凸/凹多角形床の分配（全辺負担）
//! - [`distribute_polygon_supported`] — 支持辺指定付きの分配（非支持辺は負担しない）

use super::fem::fem_uniform;
use super::geometry::edge_len;
use super::types::{push_edge, BeamLoad, LoadShape};

const POLY_GRID_N: usize = 200;

fn bbox2(poly: &[[f64; 2]]) -> (f64, f64, f64, f64) {
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for p in poly {
        min_x = min_x.min(p[0]);
        max_x = max_x.max(p[0]);
        min_y = min_y.min(p[1]);
        max_y = max_y.max(p[1]);
    }
    (min_x, max_x, min_y, max_y)
}

/// 辺上（頂点を含む）とみなす距離 [mm]。床レベルの公差と同じ 1 mm。
const ON_BOUNDARY_TOL_MM: f64 = 1.0;

/// 点が多角形内部にあるか（レイキャスト法／偶奇則）。XY 平面投影用。
///
/// 辺上の点は内側にならない。境界を含めるときは [`point_on_polygon_boundary`] と併用する。
pub(crate) fn point_in_polygon(p: [f64; 2], poly: &[[f64; 2]]) -> bool {
    let n = poly.len();
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (poly[i][0], poly[i][1]);
        let (xj, yj) = (poly[j][0], poly[j][1]);
        if (yi > p[1]) != (yj > p[1]) {
            let x_int = (xj - xi) * (p[1] - yi) / (yj - yi) + xi;
            if p[0] < x_int {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

/// 点が多角形の辺上（頂点を含む）にあるか。距離 [`ON_BOUNDARY_TOL_MM`] 以内を辺上とする。
pub(crate) fn point_on_polygon_boundary(p: [f64; 2], poly: &[[f64; 2]]) -> bool {
    let n = poly.len();
    if n < 2 {
        return false;
    }
    let tol2 = ON_BOUNDARY_TOL_MM * ON_BOUNDARY_TOL_MM;
    for i in 0..n {
        if point_segment_dist2(p, poly[i], poly[(i + 1) % n]) <= tol2 {
            return true;
        }
    }
    false
}

fn point_segment_dist2(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let ap = [p[0] - a[0], p[1] - a[1]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1];
    let t = if len2 > 1e-12 {
        ((ap[0] * ab[0] + ap[1] * ab[1]) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let proj = [a[0] + ab[0] * t, a[1] + ab[1] * t];
    let dx = p[0] - proj[0];
    let dy = p[1] - proj[1];
    dx * dx + dy * dy
}

/// 矩形でない凸（または単純な凹）多角形床の分配（レビュー §1.13 ギャップ「多角形床組」対応）。
///
/// 45°法の一般化として「各点を最も近い辺に帰属させる」負担面積法を、多角形の
/// バウンディングボックスを `POLY_GRID_N × POLY_GRID_N`（200×200）に格子分割した
/// 決定的なサンプリングで近似する。各セル中心が多角形内部なら、その中心から最も近い
/// 辺（線分）へセル面積を加算する。辺ごとの負担面積が求まったら、等価等分布
/// `w_line = W_edge / L_edge`（`W_edge = w × 辺の負担面積`）として `LoadShape::Uniform` +
/// `fem_uniform` で返す。
///
/// 荷重保存は「サンプル点の全数帰属」により、格子内部と判定された点の面積の総和について
/// 厳密に成り立つ（Σ辺負担荷重 = w × Σ格子内サンプル面積）。格子内サンプル面積と真の
/// 多角形面積（[`polygon_area`]）との差は格子近似誤差のみで、十分細かい分割（200×200）で
/// 1%未満に収まる（凸多角形で確認）。強く凹んだ（入隅の深い）多角形では近似精度が
/// 低下する可能性がある（未検証・残課題）。
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
/// [`distribute_polygon_supported`] の共通処理）。各セル中心が多角形内部なら、
/// `candidate_edges` の中で最も近い辺（線分）へセル面積を加算する。
/// `candidate_edges` に全辺（`0..n`）を渡せば [`distribute_polygon`] と同じ挙動になり、
/// 部分集合を渡せば非候補の辺には荷重が帰属しなくなる（[`distribute_polygon_supported`]）。
fn polygon_edge_areas(coords: &[[f64; 3]], candidate_edges: &[usize]) -> Vec<f64> {
    let n = coords.len();
    let mut edge_area = vec![0.0_f64; n];
    if candidate_edges.is_empty() {
        return edge_area;
    }
    let poly2: Vec<[f64; 2]> = coords.iter().map(|c| [c[0], c[1]]).collect();
    let (min_x, max_x, min_y, max_y) = bbox2(&poly2);
    let dx = (max_x - min_x) / POLY_GRID_N as f64;
    let dy = (max_y - min_y) / POLY_GRID_N as f64;
    if dx <= 0.0 || dy <= 0.0 {
        return edge_area;
    }
    let cell_area = dx * dy;
    for iy in 0..POLY_GRID_N {
        let y = min_y + (iy as f64 + 0.5) * dy;
        for ix in 0..POLY_GRID_N {
            let x = min_x + (ix as f64 + 0.5) * dx;
            let p = [x, y];
            if !point_in_polygon(p, &poly2) {
                continue;
            }
            let mut best_e = candidate_edges[0];
            let mut best_d2 = f64::INFINITY;
            for &e in candidate_edges {
                let a = poly2[e];
                let b = poly2[(e + 1) % n];
                let d2 = point_segment_dist2(p, a, b);
                if d2 < best_d2 {
                    best_d2 = d2;
                    best_e = e;
                }
            }
            edge_area[best_e] += cell_area;
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

/// 支持辺指定付きの最近接辺グリッドサンプリング帰属（レビュー残課題「片持ち梁・先端リブ
/// 小梁の分割伝達」「一般スラブの部分支持（開口際等）」対応。片持ち梁がある
/// 場合はスラブと同様のルールにより分割して荷重伝達する扱い）。
///
/// [`distribute_polygon`] と同じ格子サンプリング法（[`polygon_edge_areas`]）だが、各サンプル
/// 点を「支持辺（`supported[i] == true` の辺）のみ」の中から最近接の辺に帰属させる。
/// 非支持辺（`supported[i] == false`）には荷重が帰属しない。
///
/// 呼び出し元（[`distribute_slab`]）の用途は2通り:
/// - `SlabKind::Cantilever` + `edge_supported`: 取付き大梁（辺0）に加え、片持ち梁・先端
///   リブ小梁が取り付く辺を支持辺として指定する（例: 辺0・1・3 支持＝両側に片持ち梁、
///   辺2 も支持に含めれば先端リブ小梁あり）。
/// - `SlabKind::Interior` + `edge_supported`: 開口際などで一部の辺が大梁・小梁に
///   取り付かない一般スラブの分配に用いる（非支持辺には荷重を負担させない一般化）。
///
/// `supported` の長さが `coords.len()` と一致しない、または支持辺が1つもない
/// （全要素 `false`）場合は、指定が無意味なため安全側（総荷重を捨てない）に倒して
/// 全辺支持へフォールバックする（＝ [`distribute_polygon`] と同じ結果になる）。
pub(crate) fn distribute_polygon_supported(
    coords: &[[f64; 3]],
    w: f64,
    loads: &mut Vec<BeamLoad>,
    supported: &[bool],
) {
    let n = coords.len();
    if n < 3 {
        return;
    }
    let candidate_edges: Vec<usize> = if supported.len() == n && supported.iter().any(|&b| b) {
        (0..n).filter(|&i| supported[i]).collect()
    } else {
        (0..n).collect()
    };
    let edge_area = polygon_edge_areas(coords, &candidate_edges);
    emit_edge_loads(coords, w, &edge_area, loads);
}
