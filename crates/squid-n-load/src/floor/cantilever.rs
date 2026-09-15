//! 取り付く床板（片持ち・バルコニー・出隅）の分配戦略。
//!
//! - [`distribute_to_node`] — 取り付く床板の荷重を節点（柱）へ集中
//! - [`distribute_cantilever`] — 全長を覆う支持部材の辺へ最近接負担面積で分配

use squid_n_core::geom::polygon::area_xy;
use squid_n_core::geom::MEMBER_AXIS_TOL_MM;
use squid_n_core::ids::ElemId;
use squid_n_core::ids::NodeId;
use squid_n_core::ids::SecondaryMemberId;
use squid_n_core::model::Model;

use super::fem::fem_uniform;
use super::geometry::{dist3, edge_len};
use super::polygon::polygon_edge_areas;
use super::types::{push_edge, BeamLoad, Cmq, LoadShape, LoadTarget};
use crate::secondary::project_on_segment;
use squid_n_core::model::SecondaryJoistAxis;

/// 取り付く床板の荷重を節点（柱）へ集中させる分配。
///
/// 出隅の片持ちスラブは、荷重伝達方向および片持ち梁の取付きに関わらず、節点荷重として
/// すべて柱に伝達する。本実装ではこれに従い、全荷重 `W = w × 多角形面積`
/// （[`area_xy`]。構造芯から出隅先端までの長方形＝境界そのものの面積）に
/// `ratio` を掛けた分を、`node` への単一の集中荷重として返す。
/// 小梁反力や取り付く壁版の柱集中と同じ `LoadTarget::Node` + `LoadShape::Point`
/// （`q_i = W`、`q_j = 0`）の機構を再利用する。
///
/// `ratio` は呼び出し側が渡す按分比。
/// [`LoadTransfer::Columns`](squid_n_core::model::LoadTransfer::Columns) は取付き線の
/// 区間中点 `t_mid` から `1.0 - t_mid`／`t_mid` を渡す（全長 `[0, 1]` なら `t_mid = 0.5`
/// で両端とも 0.5）。出隅
/// （[`RegionAnchor::Point`](squid_n_core::model::RegionAnchor::Point)）は全荷重を
/// 渡すため 1.0 を用いる。
pub(crate) fn distribute_to_node(
    node: NodeId,
    coords: &[[f64; 3]],
    w: f64,
    ratio: f64,
    loads: &mut Vec<BeamLoad>,
) {
    let area = area_xy(coords);
    if area <= 0.0 {
        return;
    }
    let total = w * area * ratio;
    loads.push(BeamLoad {
        elem: ElemId(u32::MAX),
        target: LoadTarget::Node(node),
        shape: LoadShape::Point { p: total, x: 0.0 },
        cmq: Cmq {
            c_i: 0.0,
            c_j: 0.0,
            q_i: total,
            q_j: 0.0,
        },
    });
}

fn point_line_dist(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let ap = [p[0] - a[0], p[1] - a[1]];
    let len = (ab[0] * ab[0] + ab[1] * ab[1]).sqrt();
    if len < 1e-12 {
        return dist3(p, a);
    }
    (ap[0] * ab[1] - ap[1] * ab[0]).abs() / len
}

/// 取り付く床板の境界辺 k の支持部材。
enum EdgeSupport {
    /// 辺の全長を覆う実部材（複数要素に分かれうる）。
    Beams(Vec<crate::secondary::SegmentCoverage>),
    /// 辺の全長に材軸が載る二次部材小梁。
    Joist {
        member: SecondaryMemberId,
        t: [f64; 2],
    },
}

fn edge_coords(coords: &[[f64; 3]], k: usize) -> ([f64; 3], [f64; 3]) {
    (coords[k], coords[(k + 1) % coords.len()])
}

/// 辺 `p0`→`p1` の全長を覆う実部材を探す。覆えない場合は `None`。
fn covering_beams(
    candidates: &[crate::secondary::BeamSpanCandidate],
    p0: [f64; 3],
    p1: [f64; 3],
) -> Option<Vec<crate::secondary::SegmentCoverage>> {
    let len = dist3(p0, p1);
    if len <= 1e-9 {
        return None;
    }
    let cover = crate::secondary::beams_along_segment_with(candidates, p0, p1, MEMBER_AXIS_TOL_MM);
    if cover.is_empty() {
        return None;
    }
    crate::secondary::coverage_covers_full(&cover, len, MEMBER_AXIS_TOL_MM).then_some(cover)
}

/// 辺の全長に材軸が載る二次部材小梁を探す。無ければ `None`。
fn covering_joist(
    axes: &[SecondaryJoistAxis],
    p0: [f64; 3],
    p1: [f64; 3],
) -> Option<(SecondaryMemberId, [f64; 2])> {
    let mut best: Option<(SecondaryMemberId, [f64; 2], f64)> = None;
    for axis in axes {
        let (Some(s0), Some(s1)) = (
            project_on_segment(p0, axis.a, axis.b, MEMBER_AXIS_TOL_MM),
            project_on_segment(p1, axis.a, axis.b, MEMBER_AXIS_TOL_MM),
        ) else {
            continue;
        };
        let d = point_line_dist(p0, axis.a, axis.b).max(point_line_dist(p1, axis.a, axis.b));
        if best.is_none_or(|(_, _, bd)| d < bd) {
            best = Some((axis.member, [s0 / axis.len, s1 / axis.len], d));
        }
    }
    best.map(|(member, t, _)| (member, t))
}

fn edge_support(
    beams: &[crate::secondary::BeamSpanCandidate],
    joists: &[SecondaryJoistAxis],
    p0: [f64; 3],
    p1: [f64; 3],
) -> Option<EdgeSupport> {
    if let Some(cover) = covering_beams(beams, p0, p1) {
        return Some(EdgeSupport::Beams(cover));
    }
    covering_joist(joists, p0, p1).map(|(member, t)| EdgeSupport::Joist { member, t })
}

/// 支持辺の等分布荷重を、その辺を受ける部材へ分配結果として積む。
fn emit_support_load(
    model: &Model,
    sup: &EdgeSupport,
    w_line: f64,
    edge_len_mm: f64,
    loads: &mut Vec<BeamLoad>,
) {
    match sup {
        EdgeSupport::Beams(cover) => {
            for c in cover {
                let Some(elem) = model.element(c.elem) else {
                    continue;
                };
                if elem.nodes.len() != 2 {
                    continue;
                }
                let len_e = model.member_length(elem);
                if len_e <= 1e-9 {
                    continue;
                }
                let t = [
                    (c.elem_pos[0] / len_e).clamp(0.0, 1.0),
                    (c.elem_pos[1] / len_e).clamp(0.0, 1.0),
                ];
                let covered = (t[1] - t[0]).abs() * len_e;
                loads.push(BeamLoad {
                    elem: c.elem,
                    target: LoadTarget::Span {
                        nodes: [elem.nodes[0], elem.nodes[1]],
                        t,
                    },
                    shape: LoadShape::Uniform { w: w_line },
                    cmq: fem_uniform(w_line, covered.max(1e-9)),
                });
            }
        }
        EdgeSupport::Joist { member, t } => {
            loads.push(BeamLoad {
                elem: ElemId(u32::MAX),
                target: LoadTarget::Secondary {
                    member: *member,
                    t: *t,
                },
                shape: LoadShape::Uniform { w: w_line },
                cmq: fem_uniform(w_line, edge_len_mm.max(1e-9)),
            });
        }
    }
}

/// 取り付く床板の分配（取付き線へ分布）。
///
/// 境界辺のうち「全長を覆う実部材または二次部材小梁」がある辺（辺0 の取付き線は常に
/// 支持）を支持辺とし、最近接支持辺の負担面積で分配する。支持辺が取付き線だけの場合は
/// 先端まで一様なスラブの単純片持ち反力に相当する等分布 `w_line = w·d`（`d` は出し幅平均）
/// を取付き辺へ載せる。支持辺が他にもある場合は、辺ごとの負担面積 `W_edge = w × A_edge` を
/// 等価等分布 `w_line = W_edge / L_edge` として、実部材の辺はその要素へ、二次部材の辺は
/// `Span`（節点対＋区間）としてその材軸へ渡す。総和は支持辺の負担面積の合計に一致する。
pub(crate) fn distribute_cantilever(
    model: &Model,
    coords: &[[f64; 3]],
    w: f64,
    loads: &mut Vec<BeamLoad>,
) {
    if coords.len() < 4 {
        return;
    }
    let l_attach = edge_len(coords, 0);
    let d = 0.5
        * (point_line_dist(coords[2], coords[0], coords[1])
            + point_line_dist(coords[3], coords[0], coords[1]));
    if l_attach <= 1e-9 || d <= 1e-9 {
        return;
    }

    let beam_candidates = crate::secondary::beam_span_candidates(model);
    let joists = model.secondary_joist_axes();
    let mut supports: Vec<(usize, EdgeSupport)> = Vec::new();
    for k in 1..coords.len() {
        let (p0, p1) = edge_coords(coords, k);
        if let Some(sup) = edge_support(&beam_candidates, &joists, p0, p1) {
            supports.push((k, sup));
        }
    }

    if supports.is_empty() {
        let w_line = w * d;
        push_edge(
            loads,
            0,
            LoadShape::Uniform { w: w_line },
            fem_uniform(w_line, l_attach),
        );
        return;
    }

    let mut candidates = vec![0usize];
    candidates.extend(supports.iter().map(|(k, _)| *k));
    let mut areas = polygon_edge_areas(coords, &candidates);
    let sampled: f64 = areas.iter().sum();
    let true_area = area_xy(coords);
    if sampled > 1e-12 && true_area > 0.0 {
        let scale = true_area / sampled;
        for a in &mut areas {
            *a *= scale;
        }
    }

    for (e, &area) in areas.iter().enumerate() {
        if area <= 0.0 {
            continue;
        }
        let l_e = edge_len(coords, e);
        if l_e <= 1e-9 {
            continue;
        }
        let w_line = w * area / l_e;
        if e == 0 {
            push_edge(
                loads,
                0,
                LoadShape::Uniform { w: w_line },
                fem_uniform(w_line, l_e),
            );
        } else if let Some((_, sup)) = supports.iter().find(|(k, _)| *k == e) {
            emit_support_load(model, sup, w_line, l_e, loads);
        }
    }
}
