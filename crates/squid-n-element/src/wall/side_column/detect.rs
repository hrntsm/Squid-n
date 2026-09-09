//! 側柱判定（自部材が耐震壁の側柱かどうかの幾何判定）。
//!
//! 判定結果として解放すべき局所曲げ面（[`ReleaseAxis`]）を返す。

use super::ReleaseAxis;
use crate::transform::LocalFrame;
use squid_n_core::geom::vec3::{cross, dot, unit};
use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, ElementKind, Model};

/// 曲げを伝達する線材（柱・梁として扱う要素種別）か。
///
/// 大梁として壁の辺を構成しうるのは線材のみで、ブレース（軸材）・面要素・バネは
/// 曲げを伝達しないため除く。ファイバー梁・マルチスプリング梁は対象に含める。
pub fn is_line_member(kind: ElementKind) -> bool {
    matches!(
        kind,
        ElementKind::Beam | ElementKind::Fiber | ElementKind::MultiSpring
    )
}

/// 耐震壁の側柱として扱う要素種別か。
///
/// 線材（[`is_line_member`]）のうち `Beam` に限る。
pub fn is_side_column_member(kind: ElementKind) -> bool {
    matches!(kind, ElementKind::Beam)
}

/// 自部材（`data`）が耐震壁（壁エレメントモデル）の側柱（面内両端ピンの柱）かどうかを
/// 判定し、そうであれば解放すべき局所曲げ面を返す。
///
/// 条件:
/// 1. 自部材が側柱として扱う種別（[`is_side_column_member`]）かつ鉛直材であること
///    （dz が dx・dy に対して支配的）。
/// 2. `model.elements` 中に節点数4以上の `ElementKind::Wall` があり、それが耐震壁として
///    成立すること（[`crate::wall::misc_wall::wall_is_seismic`]）。
/// 3. その壁の四隅を z で下辺2・上辺2 に分け、下辺の軸方向への射影で上辺と対応付けた
///    とき、自部材の両端節点が「下辺a-上辺a」または「下辺b-上辺b」のいずれかの
///    鉛直辺の2節点と一致すること。
///
/// 解放曲げ面は、壁面法線（下辺方向×鉛直の外積）と柱の局所 ey・ez の内積絶対値が
/// 大きい方（＝回転軸が壁法線に平行な方）とする。
pub fn wall_side_column_release(data: &ElementData, model: &Model) -> Option<ReleaseAxis> {
    let (n0, n1, p0, p1) = side_column_candidate(data, model)?;

    for wall in &model.elements {
        if !matches!(wall.kind, ElementKind::Wall) || wall.nodes.len() < 4 {
            continue;
        }
        let contains = |nid: NodeId| wall.nodes.iter().take(4).any(|&x| x == nid);
        if !(contains(n0) && contains(n1)) {
            continue;
        }
        if !crate::wall::misc_wall::wall_is_seismic(wall, model) {
            continue;
        }
        let Some((sides, normal)) = wall_side_edges(wall, model) else {
            continue;
        };

        let matches_side = |side: (NodeId, NodeId)| -> bool {
            (side.0 == n0 && side.1 == n1) || (side.0 == n1 && side.1 == n0)
        };
        if !(matches_side(sides[0]) || matches_side(sides[1])) {
            continue;
        }

        return Some(release_axis_for_normal(data, p0, p1, normal));
    }
    None
}

/// 側柱候補の共通前提チェック（種別・2 節点・鉛直材）。満たせば両端の
/// 節点 ID と座標を返す。
fn side_column_candidate(
    data: &ElementData,
    model: &Model,
) -> Option<(NodeId, NodeId, [f64; 3], [f64; 3])> {
    if !is_side_column_member(data.kind) || data.nodes.len() < 2 {
        return None;
    }
    let n0 = data.nodes[0];
    let n1 = data.nodes[1];
    let node0 = model.nodes.get(n0.index())?;
    let node1 = model.nodes.get(n1.index())?;
    let (p0, p1) = (node0.coord, node1.coord);
    if !squid_n_core::geom::is_vertical_axis(p0, p1) {
        return None;
    }
    Some((n0, n1, p0, p1))
}

/// 壁の鉛直辺 2 本（下辺a-上辺a・下辺b-上辺b の節点対）と壁面法線。
type SideEdges = ([(NodeId, NodeId); 2], [f64; 3]);

/// 壁 1 枚の鉛直辺 2 本（下辺a-上辺a・下辺b-上辺b の節点対）と壁面法線を返す。
/// 退化した壁（節点欠落・辺長ゼロ・法線が定まらない）は `None`。
fn wall_side_edges(wall: &ElementData, model: &Model) -> Option<SideEdges> {
    let g = crate::wall::wall_element::wall_element_geometry(wall, model)?;
    let normal = unit(cross(g.ex_bottom, [0.0, 0.0, 1.0]))?;
    Some(([(g.bottom[0], g.top[0]), (g.bottom[1], g.top[1])], normal))
}

/// 壁面法線から解放すべき局所曲げ面を定める（回転軸が壁法線に平行な方）。
fn release_axis_for_normal(
    data: &ElementData,
    p0: [f64; 3],
    p1: [f64; 3],
    normal: [f64; 3],
) -> ReleaseAxis {
    let axis = LocalFrame::from_nodes(p0, p1, data.local_axis.ref_vector);
    let dot_ey = dot(axis.rot[1], normal).abs();
    let dot_ez = dot(axis.rot[2], normal).abs();
    if dot_ey >= dot_ez {
        ReleaseAxis::LocalY
    } else {
        ReleaseAxis::LocalZ
    }
}

/// 耐震壁の鉛直辺（節点対）→ 壁面法線の事前インデックス。
pub struct SideColumnEdges {
    /// 節点対（NodeId 昇順に正規化）→ 壁面法線。
    edges: std::collections::HashMap<(NodeId, NodeId), [f64; 3]>,
}

impl SideColumnEdges {
    fn key(a: NodeId, b: NodeId) -> (NodeId, NodeId) {
        if a.0 <= b.0 {
            (a, b)
        } else {
            (b, a)
        }
    }

    /// 耐震壁として成立する全壁の鉛直辺を収集する。
    pub fn build(model: &Model) -> Self {
        let mut edges = std::collections::HashMap::new();
        for wall in &model.elements {
            if !matches!(wall.kind, ElementKind::Wall) || wall.nodes.len() < 4 {
                continue;
            }
            if !crate::wall::misc_wall::wall_is_seismic(wall, model) {
                continue;
            }
            let Some((sides, normal)) = wall_side_edges(wall, model) else {
                continue;
            };
            for (a, b) in sides {
                edges.entry(Self::key(a, b)).or_insert(normal);
            }
        }
        Self { edges }
    }

    /// [`wall_side_column_release`] と同じ判定（構築済みインデックス版）。
    pub fn release_axis(&self, data: &ElementData, model: &Model) -> Option<ReleaseAxis> {
        let (n0, n1, p0, p1) = side_column_candidate(data, model)?;
        let normal = *self.edges.get(&Self::key(n0, n1))?;
        Some(release_axis_for_normal(data, p0, p1, normal))
    }
}
