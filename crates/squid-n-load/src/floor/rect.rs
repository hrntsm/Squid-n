//! 矩形床の分配戦略（三角形・台形／一方向／負担面積）。
//!
//! - [`distribute_rect`] — 矩形床の分配（TriTrapezoid / OneWay / TributaryArea）
//! - [`distribute_one_way_dir`] — 一方向スラブの伝達方向指定に基づく分配

use squid_n_core::model::{DistributionMethod, OneWayDir, Slab};

use super::fem::{fem_trapezoid, fem_triangle, fem_uniform};
use super::geometry::edge_len;
use super::types::{push_edge, BeamLoad, LoadShape};

fn edge_dir2(coords: &[[f64; 3]], i: usize) -> [f64; 2] {
    let n = coords.len();
    let a = coords[i];
    let b = coords[(i + 1) % n];
    [b[0] - a[0], b[1] - a[1]]
}

/// 矩形の4辺を、与えられた2D軸ベクトルに対して「軸に平行な2辺 (0,2 または 1,3)」と
/// 「軸に直交する2辺」に分類する。矩形（辺0‖辺2、辺1‖辺3）を仮定する。
/// 戻り値は `(平行な2辺, 直交する2辺)`。
fn classify_rect_edges_by_axis(coords: &[[f64; 3]], axis: [f64; 2]) -> ([usize; 2], [usize; 2]) {
    let d0 = edge_dir2(coords, 0);
    let n0 = (d0[0] * d0[0] + d0[1] * d0[1]).sqrt().max(1e-12);
    let na = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt().max(1e-12);
    let cos0 = (d0[0] * axis[0] + d0[1] * axis[1]).abs() / (n0 * na);
    if cos0 >= 0.5 {
        ([0, 2], [1, 3])
    } else {
        ([1, 3], [0, 2])
    }
}

/// 矩形床の分配（三角形・台形（45°）／一方向／負担面積法）。`coords` は境界4頂点の座標。
pub(crate) fn distribute_rect(
    slab: &Slab,
    coords: &[[f64; 3]],
    lx: f64,
    ly: f64,
    w: f64,
    loads: &mut Vec<BeamLoad>,
) {
    match slab.method() {
        DistributionMethod::TriTrapezoid => {
            let is_square = (lx - ly).abs() < 1e-6;
            if is_square {
                let w0 = w * lx / 2.0;
                for i in 0..4 {
                    let l = if i % 2 == 0 { lx } else { ly };
                    push_edge(loads, i, LoadShape::Triangle { w0 }, fem_triangle(w0, l));
                }
            } else {
                let short = lx.min(ly);
                let long = lx.max(ly);
                let w0 = w * short / 2.0;
                let a = short / 2.0;
                let b = long - 2.0 * a;

                for i in 0..4 {
                    let l = if i % 2 == 0 { lx } else { ly };
                    let is_short_side = (l - short).abs() < 1e-6;
                    if is_short_side {
                        push_edge(loads, i, LoadShape::Triangle { w0 }, fem_triangle(w0, l));
                    } else {
                        push_edge(
                            loads,
                            i,
                            LoadShape::Trapezoid { w0, a, b },
                            fem_trapezoid(w0, a, b, l),
                        );
                    }
                }
            }
        }
        DistributionMethod::OneWay => {
            if let Some(dir) = slab.one_way() {
                distribute_one_way_dir(coords, dir, w, loads);
            } else {
                // 従来互換（`one_way` 未指定）: 辺0・2負担（＝辺1方向スパン）。
                let w_line = w * ly / 2.0;
                for i in 0..4 {
                    let l = if i % 2 == 0 { lx } else { ly };
                    if (l - lx).abs() < 1e-6 {
                        push_edge(
                            loads,
                            i,
                            LoadShape::Uniform { w: w_line },
                            fem_uniform(w_line, l),
                        );
                    }
                }
            }
        }
        DistributionMethod::TributaryArea => {
            // 45°負担面積を等価等分布へ換算（総和保存）。
            let short = lx.min(ly);
            let long = lx.max(ly);
            for i in 0..4 {
                let l = if i % 2 == 0 { lx } else { ly };
                let is_short_side = (l - short).abs() <= (l - long).abs();
                let w_line = if is_short_side {
                    w * short / 4.0
                } else {
                    w * (short * long / 2.0 - short * short / 4.0) / long
                };
                push_edge(
                    loads,
                    i,
                    LoadShape::Uniform { w: w_line },
                    fem_uniform(w_line, l),
                );
            }
        }
    }
}

/// 一方向スラブの荷重伝達方向指定（`region.one_way() = Some(dir)`）に基づく分配（レビュー §1.13）。
///
/// `dir` に対応する全体座標軸（X→[1,0]、Y→[0,1]）を「伝達方向」とし、伝達方向に
/// **直交する**2辺（＝伝達方向と垂直に走る2辺）を負担辺とする。負担辺の線荷重は
/// `w×(スパン長/2)`（スパン長＝伝達方向に平行な辺の長さ）。総和は
/// `2×w×(スパン長/2)×負担辺長 = w×面積` で保存される。
pub(crate) fn distribute_one_way_dir(
    coords: &[[f64; 3]],
    dir: OneWayDir,
    w: f64,
    loads: &mut Vec<BeamLoad>,
) {
    let axis = match dir {
        OneWayDir::X => [1.0, 0.0],
        OneWayDir::Y => [0.0, 1.0],
    };
    let (parallel, bearing) = classify_rect_edges_by_axis(coords, axis);
    let span_len = edge_len(coords, parallel[0]);
    let w_line = w * span_len / 2.0;
    for &e in &bearing {
        let l_e = edge_len(coords, e);
        push_edge(
            loads,
            e,
            LoadShape::Uniform { w: w_line },
            fem_uniform(w_line, l_e),
        );
    }
}
