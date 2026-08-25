//! 矩形床の分配戦略（三角形・台形／一方向／負担面積／小梁二段階伝達）。
//!
//! - [`distribute_rect`] — 矩形床の分配（TriTrapezoid / OneWay / TributaryArea）
//! - [`distribute_one_way_dir`] — 一方向スラブの伝達方向指定に基づく分配
//! - [`distribute_rect_with_joists`] — 小梁による二段階伝達（矩形スラブ）

use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{DistributionMethod, ElementKind, FloorRegion, Model, OneWayDir, Slab};

use super::fem::{fem_trapezoid, fem_triangle, fem_uniform};
use super::geometry::{dist3, edge_len};
use super::types::{push_edge, BeamLoad, Cmq, LoadShape, LoadTarget};

/// 節点 `a`↔`b` を両端に持つ実 `Beam` 要素がモデルに存在するか
/// （小梁が実部材化されているかの判定。ノード順は不問）。
fn beam_between(model: &Model, a: NodeId, b: NodeId) -> bool {
    model.elements.iter().any(|e| {
        e.kind == ElementKind::Beam
            && e.nodes.len() == 2
            && ((e.nodes[0] == a && e.nodes[1] == b) || (e.nodes[0] == b && e.nodes[1] == a))
    })
}

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

/// 小梁 (`region.joist_lines()`) による二段階伝達（矩形スラブのみ。仕様書 §5.1「2段階」）。
///
/// **簡易モデルの仮定**（本実装の割り切り。厳密な二方向効果は考慮しない）:
/// - 各 `JoistLine` は1本の物理的な小梁を表し、`dir` 方向に架かり、`spacing` 分の
///   負担幅（トリビュタリ幅）を一様に負担すると仮定する（面荷重×`spacing`＝小梁の等分布荷重）。
/// - 小梁は単純梁として振る舞い、両端 `support` 節点へ反力
///   `R = w·spacing·L_joist/2`（`L_joist` = support間距離）を返す
///   （`LoadTarget::Node(support)` の集中荷重、`LoadShape::Point`）。
/// - 小梁と平行な2辺（境界の大梁。`dir` に平行な辺）は、小梁群がカバーしない残りの幅
///   `remainder = width_total − Σspacing_k`（`width_total` = 小梁と直交する境界辺の長さ）を
///   両辺で折半して負担する（各辺 UDL = `w·remainder/2`）。これは「小梁が
///   `spacing` 間隔で並び、境界からも半間隔 `spacing/2` 離れて配置される」典型例のとき
///   厳密に `w·(spacing/2)` に一致する（境界梁が"仮想的な端の小梁"のように振る舞う）。
///   この remainder 方式により、`spacing` が不均一でも総和保存が厳密に成り立つ。
/// - 小梁位置・本数の実際の配置（`JoistLine.support` の座標）はモデルからそのまま用いる。
pub(crate) fn distribute_rect_with_joists(
    model: &Model,
    region: &FloorRegion,
    coords: &[[f64; 3]],
    w: f64,
    loads: &mut Vec<BeamLoad>,
) {
    if region.joist_lines().is_empty() {
        return;
    }
    let dir = region.joist_lines()[0].dir;
    let dn = (dir[0] * dir[0] + dir[1] * dir[1]).sqrt();
    let axis = if dn > 1e-12 {
        [dir[0] / dn, dir[1] / dn]
    } else {
        [0.0, 1.0]
    };
    let (parallel_edges, perp_edges) = classify_rect_edges_by_axis(coords, axis);
    let width_total = edge_len(coords, perp_edges[0]);

    let mut spacing_sum = 0.0;
    for j in region.joist_lines() {
        let (Some(n0), Some(n1)) = (
            model.nodes.get(j.support[0].index()),
            model.nodes.get(j.support[1].index()),
        ) else {
            continue;
        };
        let l_joist = dist3(n0.coord, n1.coord);
        if beam_between(model, j.support[0], j.support[1]) {
            // 実部材化された小梁: トリビュタリ等分布荷重 w·spacing を小梁自身へ載せ、
            // 小梁が FEM で両端支持へ伝達する（点反力は用いない＝二重計上しない）。
            let w_udl = w * j.spacing;
            loads.push(BeamLoad {
                elem: ElemId(u32::MAX),
                target: LoadTarget::Span([j.support[0], j.support[1]]),
                shape: LoadShape::Uniform { w: w_udl },
                cmq: fem_uniform(w_udl, l_joist),
            });
        } else {
            // 仮想小梁: 単純梁反力 R=w·spacing·L/2 を両端節点へ集中荷重として伝達する。
            let r = w * j.spacing * l_joist / 2.0;
            loads.push(BeamLoad {
                elem: ElemId(u32::MAX),
                target: LoadTarget::Node(j.support[0]),
                shape: LoadShape::Point { p: r, x: 0.0 },
                cmq: Cmq {
                    c_i: 0.0,
                    c_j: 0.0,
                    q_i: r,
                    q_j: 0.0,
                },
            });
            loads.push(BeamLoad {
                elem: ElemId(u32::MAX),
                target: LoadTarget::Node(j.support[1]),
                shape: LoadShape::Point { p: r, x: 0.0 },
                cmq: Cmq {
                    c_i: 0.0,
                    c_j: 0.0,
                    q_i: r,
                    q_j: 0.0,
                },
            });
        }
        spacing_sum += j.spacing;
    }

    let remainder = width_total - spacing_sum;
    let boundary_w_line = w * remainder / 2.0;
    for &e in &parallel_edges {
        let l_e = edge_len(coords, e);
        push_edge(
            loads,
            e,
            LoadShape::Uniform { w: boundary_w_line },
            fem_uniform(boundary_w_line, l_e),
        );
    }
}
