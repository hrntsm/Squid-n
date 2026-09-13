//! 二次部材（小梁・間柱）。全体解析（剛性行列）には算入しない。

use super::*;

/// 二次部材の種別。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum SecondaryMemberKind {
    /// 小梁。
    Joist,
    /// 間柱。
    Post,
}

/// 二次部材の端部支持条件。
#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize,
)]
pub enum EndSupport {
    /// 主架構または他の二次部材に支持される（既定）。
    #[default]
    Supported,
    /// 自由端。荷重はこの端から出ていかない。
    Free,
}

/// 二次部材（小梁・間柱）。全体解析の対象外。
///
/// 実体は床領域（[`super::FloorRegion::secondary_joists`]）または壁領域
/// （[`super::WallRegion::posts`]）が保持する。所属未割当の小梁・間柱だけ
/// [`super::Model::unassigned_joists`] / [`super::Model::unassigned_posts`] に置く。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SecondaryMember {
    pub kind: SecondaryMemberKind,
    /// 両端節点（小梁: 始端→終端、間柱: 下端→上端の順を推奨。順序に依存しない）。
    pub nodes: [NodeId; 2],
    /// 断面参照。
    pub section: Option<SectionId>,
    /// 表示名。
    pub name: String,
    /// 端部支持条件（[`EndSupport`]）。`nodes` と同じ順。既定は両端 `Supported`。
    #[serde(default)]
    pub end_support: [EndSupport; 2],
}

impl SecondaryMember {
    /// 自由端の位置（`0` または `1`）。両端 `Supported` なら `None`。
    /// 片持ち小梁は片端だけが `Free` の状態を指す。
    pub fn free_end(&self) -> Option<usize> {
        match self.end_support {
            [EndSupport::Free, EndSupport::Supported] => Some(0),
            [EndSupport::Supported, EndSupport::Free] => Some(1),
            _ => None,
        }
    }

    /// 片持ち小梁か（片端だけが `Free`）。
    pub fn is_cantilever(&self) -> bool {
        self.free_end().is_some()
    }
}

/// 点 `p` が線分 `a`–`b` の材軸上（距離 `tol` [mm] 以内）にあり、両端から `tol` を
/// 超えた内法にあるか。
fn point_in_segment_interior(p: [f64; 3], a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    if len2 <= 1.0 {
        return false;
    }
    let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let t = (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2;
    let len = len2.sqrt();
    let s = t * len;
    if s <= tol || s >= len - tol {
        return false;
    }
    let proj = [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]];
    let d = ((p[0] - proj[0]).powi(2) + (p[1] - proj[1]).powi(2) + (p[2] - proj[2]).powi(2)).sqrt();
    d <= tol
}

/// 実部材化していない二次部材小梁の材軸。
#[derive(Clone, Copy, Debug)]
pub struct SecondaryJoistAxis {
    pub nodes: [NodeId; 2],
    /// 材軸始端の座標 [mm]。
    pub a: [f64; 3],
    /// 材軸終端の座標 [mm]。
    pub b: [f64; 3],
    /// 材軸長さ [mm]。
    pub len: f64,
    /// 端部支持条件（`nodes` と同じ順）。
    pub end_support: [EndSupport; 2],
}

impl Model {
    /// 節点 `node` が主架構の幾何にあるか（要素が接続する節点、または水平材の材軸上）。
    fn node_on_primary_geometry(&self, node: NodeId) -> bool {
        if self.elements.iter().any(|e| e.nodes.contains(&node)) {
            return true;
        }
        let Some(n) = self.nodes.get(node.index()) else {
            return false;
        };
        let p = n.coord;
        self.elements.iter().any(|e| {
            if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
                return false;
            }
            let (Some(a), Some(b)) = (
                self.nodes.get(e.nodes[0].index()),
                self.nodes.get(e.nodes[1].index()),
            ) else {
                return false;
            };
            point_in_segment_interior(p, a.coord, b.coord, crate::geom::MEMBER_AXIS_TOL_MM)
        })
    }

    /// 実部材化していない二次部材小梁の材軸を集める（両端に実 `Beam` 要素がある小梁は
    /// 実部材として扱うため除く）。
    pub fn secondary_joist_axes(&self) -> Vec<SecondaryJoistAxis> {
        let materialized: std::collections::HashSet<(u32, u32)> = self
            .elements
            .iter()
            .filter(|e| e.kind == ElementKind::Beam && e.nodes.len() == 2)
            .map(|e| {
                let (a, b) = (e.nodes[0].0, e.nodes[1].0);
                (a.min(b), a.max(b))
            })
            .collect();
        let mut out = Vec::new();
        for sm in self.joists() {
            if sm.kind != SecondaryMemberKind::Joist {
                continue;
            }
            let (n0, n1) = (sm.nodes[0], sm.nodes[1]);
            if n0 == n1 {
                continue;
            }
            let (lo, hi) = (n0.0.min(n1.0), n0.0.max(n1.0));
            if materialized.contains(&(lo, hi)) {
                continue;
            }
            let (Some(a), Some(b)) = (
                self.nodes.get(n0.index()).map(|n| n.coord),
                self.nodes.get(n1.index()).map(|n| n.coord),
            ) else {
                continue;
            };
            let len = crate::geom::vec3::dist(a, b);
            if len <= 1e-9 {
                continue;
            }
            out.push(SecondaryJoistAxis {
                nodes: [n0, n1],
                a,
                b,
                len,
                end_support: sm.end_support,
            });
        }
        out
    }

    /// 二次部材が実部材化済みか（両端節点を結ぶ 2 節点 `Beam` 要素がある）。
    pub fn secondary_member_materialized(&self, sm: &SecondaryMember) -> bool {
        let (a, b) = (sm.nodes[0], sm.nodes[1]);
        self.elements.iter().any(|e| {
            e.kind == ElementKind::Beam
                && e.nodes.len() == 2
                && ((e.nodes[0] == a && e.nodes[1] == b) || (e.nodes[0] == b && e.nodes[1] == a))
        })
    }

    /// 節点 `node` が主架構または他の二次部材の内法に幾何的に載るか。`exclude` の部材は除く。
    /// 他の二次部材の端点と一致するだけの端は支持に数えない。
    pub fn node_has_geometric_support(&self, node: NodeId, exclude: &SecondaryMember) -> bool {
        if self.node_on_primary_geometry(node) {
            return true;
        }
        let Some(p) = self.nodes.get(node.index()).map(|n| n.coord) else {
            return false;
        };
        let tol = crate::geom::MEMBER_AXIS_TOL_MM;
        let exclude_key = secondary_key(exclude);
        self.joists().chain(self.posts()).any(|other| {
            if secondary_key(other) == exclude_key || other.nodes.contains(&node) {
                return false;
            }
            let (Some(a), Some(b)) = (
                self.nodes.get(other.nodes[0].index()).map(|n| n.coord),
                self.nodes.get(other.nodes[1].index()).map(|n| n.coord),
            ) else {
                return false;
            };
            point_in_segment_interior(p, a, b, tol)
        })
    }

    /// 取り込み時に、幾何的に支持のない二次部材の端を `EndSupport::Free` に推定する。
    ///
    /// 端が主架構にも他の二次部材の内法にも載らず（[`Self::node_has_geometric_support`]）、
    /// 他の二次部材の端点とも一致しない場合に自由端とする。端点が一致する場合
    /// （先端リブの端が片持ち小梁の先端に載るなど）は、自分の反対側の端が主架構に載り、
    /// かつ相手の材軸が自分の材軸と連続していないときだけ自由端とする。材軸が連続する
    /// 端（分割された小梁の継ぎ目）はどちらも自由端にしない。両端が自由端になる推定も
    /// しない。推定して Free にした端を返す。
    pub fn infer_secondary_end_supports(&mut self) -> Vec<([NodeId; 2], usize)> {
        let members: Vec<SecondaryMember> = self.joists().chain(self.posts()).cloned().collect();
        let mut free_flags = vec![[false; 2]; members.len()];
        for (i, sm) in members.iter().enumerate() {
            for (end, flag) in free_flags[i].iter_mut().enumerate() {
                if self.node_has_geometric_support(sm.nodes[end], sm) {
                    continue;
                }
                let tied: Vec<usize> = members
                    .iter()
                    .enumerate()
                    .filter(|(j, other)| *j != i && other.nodes.contains(&sm.nodes[end]))
                    .map(|(j, _)| j)
                    .collect();
                let is_free = if tied.is_empty() {
                    true
                } else {
                    self.node_on_primary_geometry(sm.nodes[1 - end])
                        && !tied
                            .iter()
                            .any(|&j| axes_are_collinear(self, sm, &members[j]))
                };
                if is_free {
                    *flag = true;
                }
            }
        }
        for flags in &mut free_flags {
            if *flags == [true, true] {
                *flags = [false, false];
            }
        }

        let by_key: std::collections::HashMap<_, _> = members
            .iter()
            .zip(free_flags.iter())
            .map(|(sm, flags)| (secondary_key(sm), *flags))
            .collect();
        let apply = |sm: &mut SecondaryMember| {
            let flags = by_key
                .get(&secondary_key(sm))
                .copied()
                .unwrap_or([false; 2]);
            sm.end_support = flags.map(|f| {
                if f {
                    EndSupport::Free
                } else {
                    EndSupport::Supported
                }
            });
        };
        for sm in &mut self.unassigned_joists {
            apply(sm);
        }
        for r in &mut self.floor_regions {
            for sm in &mut r.secondary_joists {
                apply(sm);
            }
        }
        for sm in &mut self.unassigned_posts {
            apply(sm);
        }
        for r in &mut self.wall_regions {
            for sm in &mut r.posts {
                apply(sm);
            }
        }

        let mut inferred = Vec::new();
        for (sm, flags) in members.iter().zip(free_flags.iter()) {
            for (end, flag) in flags.iter().enumerate() {
                if *flag {
                    inferred.push((sm.nodes, end));
                }
            }
        }
        inferred
    }
}

fn secondary_key(sm: &SecondaryMember) -> (SecondaryMemberKind, NodeId, NodeId) {
    let (a, b) = (sm.nodes[0], sm.nodes[1]);
    let (lo, hi) = if a.0 <= b.0 { (a, b) } else { (b, a) };
    (sm.kind, lo, hi)
}

/// 2 本の二次部材の材軸が連続しているか。共有節点から遠い方の端点が、相手の
/// 材軸直線から [`crate::geom::MEMBER_AXIS_TOL_MM`] 以内にあることで判定する。
fn axes_are_collinear(model: &Model, a: &SecondaryMember, b: &SecondaryMember) -> bool {
    let coords = |sm: &SecondaryMember| {
        Some((
            model.nodes.get(sm.nodes[0].index())?.coord,
            model.nodes.get(sm.nodes[1].index())?.coord,
        ))
    };
    let (Some((a0, a1)), Some((b0, b1))) = (coords(a), coords(b)) else {
        return false;
    };
    let same = |p: [f64; 3], q: [f64; 3]| {
        (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2) <= 1.0
    };
    let b_far = if same(b0, a0) || same(b0, a1) { b1 } else { b0 };
    let d = [a1[0] - a0[0], a1[1] - a0[1], a1[2] - a0[2]];
    let len2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    if len2 <= 1.0 {
        return false;
    }
    let ap = [b_far[0] - a0[0], b_far[1] - a0[1], b_far[2] - a0[2]];
    let cross = [
        ap[1] * d[2] - ap[2] * d[1],
        ap[2] * d[0] - ap[0] * d[2],
        ap[0] * d[1] - ap[1] * d[0],
    ];
    let dist =
        (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt() / len2.sqrt();
    dist <= crate::geom::MEMBER_AXIS_TOL_MM
}
