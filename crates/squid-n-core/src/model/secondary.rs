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
///
/// 両端は [`SecondaryMemberEnds`] の支持部材アンカーで表し、モデル節点を持たない。
/// 既定の `ends` は生座標が原点の退化した [`SecondaryMemberEnds::Detached`]。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SecondaryMember {
    /// 安定 ID。モデル内の全二次部材で一意。
    pub id: SecondaryMemberId,
    pub kind: SecondaryMemberKind,
    /// 両端の支持部材への取付き位置（片持ちは支持端と自由端ベクトル）。
    pub ends: SecondaryMemberEnds,
    /// 断面参照。
    pub section: Option<SectionId>,
    /// 表示名。
    pub name: String,
}

impl Default for SecondaryMember {
    fn default() -> Self {
        Self {
            id: SecondaryMemberId(0),
            kind: SecondaryMemberKind::Joist,
            ends: SecondaryMemberEnds::Detached([[0.0; 3]; 2]),
            section: None,
            name: String::new(),
        }
    }
}

impl SecondaryMember {
    /// 片持ち（片端が自由端）か。
    pub fn is_cantilever(&self) -> bool {
        matches!(self.ends, SecondaryMemberEnds::Cantilever { .. })
    }

    /// 支持部材アンカーへ解決できなかった生座標表現か。
    pub fn is_detached(&self) -> bool {
        matches!(self.ends, SecondaryMemberEnds::Detached(_))
    }
}

/// 取付き位置表現へ解決する前の、モデル節点対で表した二次部材。
///
/// 取り込みと一括変換の入力専用。`Model` へ入る `SecondaryMember` は
/// `ends` を持つため、この型は `Model` の外にだけ現れる。
#[derive(Clone, Copy, Debug)]
pub struct SecondaryMemberNodes {
    pub id: SecondaryMemberId,
    pub kind: SecondaryMemberKind,
    pub nodes: [NodeId; 2],
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

/// 点 `p` が線分 `a`–`b` の材軸上（距離 `tol` [mm] 以内、端点を含む）にあるとき、
/// 材軸位置 `t`（0..1）と材軸からの距離 [mm] を返す。
fn point_on_axis(p: [f64; 3], a: [f64; 3], b: [f64; 3], tol: f64) -> Option<(f64, f64)> {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    if len2 <= 1.0 {
        return None;
    }
    let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let t = (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2;
    if !(-1e-9..=1.0 + 1e-9).contains(&t) {
        return None;
    }
    let proj = [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]];
    let d = ((p[0] - proj[0]).powi(2) + (p[1] - proj[1]).powi(2) + (p[2] - proj[2]).powi(2)).sqrt();
    (d <= tol).then_some((t.clamp(0.0, 1.0), d))
}

/// [`point_on_axis`] の両端 `tol` を超えた内法だけを返す版。
fn point_on_axis_interior(p: [f64; 3], a: [f64; 3], b: [f64; 3], tol: f64) -> Option<(f64, f64)> {
    let (t, d) = point_on_axis(p, a, b, tol)?;
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len = (ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]).sqrt();
    let s = t * len;
    (s > tol && s < len - tol).then_some((t, d))
}

/// 実部材化していない二次部材小梁の材軸。
#[derive(Clone, Copy, Debug)]
pub struct SecondaryJoistAxis {
    /// 対応する二次部材の安定 ID。
    pub member: SecondaryMemberId,
    /// 材軸始端の座標 [mm]。
    pub a: [f64; 3],
    /// 材軸終端の座標 [mm]。
    pub b: [f64; 3],
    /// 材軸長さ [mm]。
    pub len: f64,
    /// 端部支持条件（`a`→`b` の順）。
    pub end_support: [EndSupport; 2],
    /// 支持部材アンカーへ解決できていない（[`SecondaryMemberEnds::Detached`]）か。
    pub detached: bool,
}

/// 端点座標が一致するか（[`crate::geom::MEMBER_AXIS_TOL_MM`] 以内）。
fn points_equal(a: [f64; 3], b: [f64; 3]) -> bool {
    let delta = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]
        <= crate::geom::MEMBER_AXIS_TOL_MM * crate::geom::MEMBER_AXIS_TOL_MM
}

/// 片持ち以外は両端支持として扱う（片持ちの自由端は常に端番号 1）。
fn ends_end_support(ends: &SecondaryMemberEnds) -> [EndSupport; 2] {
    match ends {
        SecondaryMemberEnds::Cantilever { .. } => [EndSupport::Supported, EndSupport::Free],
        _ => [EndSupport::Supported, EndSupport::Supported],
    }
}

impl Model {
    /// 点 `p` が主架構の幾何にあるか（要素が接続する節点、または水平大梁の材軸上）。
    fn point_on_primary_geometry(&self, p: [f64; 3]) -> bool {
        if self.nodes.iter().any(|n| {
            points_equal(n.coord, p) && self.elements.iter().any(|e| e.nodes.contains(&n.id))
        }) {
            return true;
        }
        self.elements.iter().any(|e| {
            if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
                return false;
            }
            let (Some(a), Some(b)) = (self.node(e.nodes[0]), self.node(e.nodes[1])) else {
                return false;
            };
            point_in_segment_interior(p, a.coord, b.coord, crate::geom::MEMBER_AXIS_TOL_MM)
        })
    }

    /// 実部材化していない二次部材小梁の材軸を集める（両端に実 `Beam` 要素がある小梁は
    /// 実部材として扱うため除く）。
    pub fn secondary_joist_axes(&self) -> Vec<SecondaryJoistAxis> {
        let mut out = Vec::new();
        for sm in self.joists() {
            if sm.kind != SecondaryMemberKind::Joist {
                continue;
            }
            if self.secondary_member_materialized(sm) {
                continue;
            }
            let Some((a, b, len)) = self.secondary_member_axis(sm) else {
                continue;
            };
            out.push(SecondaryJoistAxis {
                member: sm.id,
                a,
                b,
                len,
                end_support: ends_end_support(&sm.ends),
                detached: sm.is_detached(),
            });
        }
        out
    }

    /// 二次部材の両端座標 [mm] を端番号順に返す。片持ちは支持端（0）と自由端（1）。
    /// 支持端アンカーや片持ち自由端の構面が解決できなければ `None`。
    pub fn secondary_member_end_points(
        &self,
        sm: &SecondaryMember,
    ) -> Option<([f64; 3], [f64; 3])> {
        match sm.ends {
            SecondaryMemberEnds::Supported([a, b]) => {
                Some((self.anchor_point(a)?, self.anchor_point(b)?))
            }
            SecondaryMemberEnds::Cantilever {
                support,
                free_end_vector,
            } => {
                let p = self.anchor_point(support)?;
                let q = self.cantilever_free_point(sm.kind, p, free_end_vector)?;
                Some((p, q))
            }
            SecondaryMemberEnds::Detached([p0, p1]) => Some((p0, p1)),
        }
    }

    /// 片持ち自由端を支持端基準の構面内ベクトル [mm] から 3 次元座標へ解決する。
    ///
    /// 小梁は水平面（全体 XY）、間柱は鉛直構面の局所座標 `(s, z)` を基底とする。
    /// 間柱は鉛直のため `s` 成分は 0 になる（0 でない場合は支持端を通る構面方向を
    /// [`crate::region_gen::wall::wall_planes`] から一意に取れるときに限り解決する）。
    pub(crate) fn cantilever_free_point(
        &self,
        kind: SecondaryMemberKind,
        support: [f64; 3],
        vector: [f64; 2],
    ) -> Option<[f64; 3]> {
        match kind {
            SecondaryMemberKind::Joist => {
                Some([support[0] + vector[0], support[1] + vector[1], support[2]])
            }
            SecondaryMemberKind::Post => {
                if vector[0].abs() <= crate::geom::MEMBER_AXIS_TOL_MM {
                    Some([support[0], support[1], support[2] + vector[1]])
                } else {
                    let direction = self.wall_plane_direction_at(support)?;
                    Some([
                        support[0] + vector[0] * direction[0],
                        support[1] + vector[0] * direction[1],
                        support[2] + vector[1],
                    ])
                }
            }
        }
    }

    /// 座標 `p` を通る鉛直構面の方向単位ベクトル [mm]。候補が 1 つに定まらない場合は
    /// `None`。
    fn wall_plane_direction_at(&self, p: [f64; 3]) -> Option<[f64; 2]> {
        let mut found = None;
        for (origin, direction) in crate::region_gen::wall::wall_planes(self) {
            let v = [p[0] - origin[0], p[1] - origin[1]];
            if (v[0] * direction[1] - v[1] * direction[0]).abs() <= crate::geom::MEMBER_AXIS_TOL_MM
            {
                if found.is_some() {
                    return None;
                }
                found = Some(direction);
            }
        }
        found
    }

    /// 二次部材の材軸 `(始端座標, 終端座標, 材軸長さ)` [mm] を返す。
    ///
    /// [`SecondaryMember::ends`] の支持部材アンカー（片持ちは支持端と自由端ベクトル）
    /// から解決する。
    pub fn secondary_member_axis(&self, sm: &SecondaryMember) -> Option<([f64; 3], [f64; 3], f64)> {
        let (p0, p1) = self.secondary_member_end_points(sm)?;
        let len = crate::geom::vec3::dist(p0, p1);
        (len > 1e-9).then_some((p0, p1, len))
    }

    /// 二次部材が実部材化済みか。両端座標に一致する節点を結ぶ 2 節点 `Beam` 要素の
    /// 有無で判定する。
    pub fn secondary_member_materialized(&self, sm: &SecondaryMember) -> bool {
        let Some((a, b)) = self.secondary_member_end_points(sm) else {
            return false;
        };
        let tol = crate::geom::MEMBER_AXIS_TOL_MM;
        let find = |p: [f64; 3]| {
            self.nodes
                .iter()
                .find(|n| crate::geom::vec3::dist(n.coord, p) <= tol)
                .map(|n| n.id)
        };
        let (Some(na), Some(nb)) = (find(a), find(b)) else {
            return false;
        };
        self.elements.iter().any(|e| {
            e.kind == ElementKind::Beam
                && e.nodes.len() == 2
                && ((e.nodes[0] == na && e.nodes[1] == nb)
                    || (e.nodes[0] == nb && e.nodes[1] == na))
        })
    }

    /// 点 `p` が主架構または他の二次部材の内法に幾何的に載るか。`exclude` の部材は除く。
    fn point_has_geometric_support(
        &self,
        p: [f64; 3],
        exclude: SecondaryMemberId,
        members: &[(SecondaryMemberId, SecondaryMemberKind, [f64; 3], [f64; 3])],
    ) -> bool {
        if self.point_on_primary_geometry(p) {
            return true;
        }
        let tol = crate::geom::MEMBER_AXIS_TOL_MM;
        members
            .iter()
            .any(|(id, _, a, b)| *id != exclude && point_in_segment_interior(p, *a, *b, tol))
    }

    /// 節点 `node` が主架構または他の二次部材の内法に幾何的に載るか。`exclude` の部材は除く。
    /// 他の二次部材の端点と一致するだけの端は支持に数えない。
    pub fn node_has_geometric_support(&self, node: NodeId, exclude: &SecondaryMember) -> bool {
        if self.elements.iter().any(|e| e.nodes.contains(&node)) {
            return true;
        }
        let Some(p) = self.nodes.get(node.index()).map(|n| n.coord) else {
            return false;
        };
        let members: Vec<_> = self
            .joists()
            .chain(self.posts())
            .filter_map(|sm| {
                let (a, b) = self.secondary_member_end_points(sm)?;
                Some((sm.id, sm.kind, a, b))
            })
            .collect();
        self.point_has_geometric_support(p, exclude.id, &members)
    }

    /// 幾何的に支持のない二次部材の端を自由端と推定する（`members` の並びで返す）。
    ///
    /// 端が主架構にも他の二次部材の内法にも載らず、他の二次部材の端点とも一致しない場合は
    /// 自由端とする。端点が一致する場合（先端リブの端が片持ち小梁の先端に載るなど）は、
    /// 自分の反対側の端が主架構に載り、かつ相手の材軸が自分の材軸と連続していないときだけ
    /// 自由端とする。材軸が連続する端（分割された小梁の継ぎ目）はどちらも自由端にしない。
    /// 両端が自由端になる推定もしない。
    fn infer_free_ends(
        &self,
        members: &[(SecondaryMemberId, SecondaryMemberKind, [f64; 3], [f64; 3])],
    ) -> Vec<[bool; 2]> {
        let mut free_flags = vec![[false; 2]; members.len()];
        for i in 0..members.len() {
            for (end, flag) in free_flags[i].iter_mut().enumerate() {
                let p = if end == 0 { members[i].2 } else { members[i].3 };
                if self.point_has_geometric_support(p, members[i].0, members) {
                    continue;
                }
                let tied: Vec<usize> = members
                    .iter()
                    .enumerate()
                    .filter(|(j, (_, _, a, b))| {
                        *j != i && (points_equal(*a, p) || points_equal(*b, p))
                    })
                    .map(|(j, _)| j)
                    .collect();
                let other_end = if end == 0 { members[i].3 } else { members[i].2 };
                let is_free = if tied.is_empty() {
                    true
                } else {
                    self.point_on_primary_geometry(other_end)
                        && !tied
                            .iter()
                            .any(|&j| axes_are_collinear(members[i], members[j]))
                };
                if is_free {
                    *flag = true;
                }
            }
            if free_flags[i] == [true, true] {
                free_flags[i] = [false, false];
            }
        }
        free_flags
    }

    /// 節点 `node` が取り付く支持部材と材軸位置を解決する。`exclude` の二次部材は除く。
    ///
    /// 主架構の 2 節点 `Beam` を要素 ID 昇順で調べ、材軸上（端点を含む）にあれば
    /// その要素を優先する。無ければ他の二次部材の内法に載るものを安定 ID 昇順で調べる。
    /// いずれも距離 `MEMBER_AXIS_TOL_MM` [mm] 以内で最も近い候補を採る。
    pub fn resolve_point_anchor(
        &self,
        p: [f64; 3],
        exclude: Option<SecondaryMemberId>,
    ) -> Option<SecondaryMemberAnchor> {
        let tol = crate::geom::MEMBER_AXIS_TOL_MM;
        let mut primary: Option<(f64, SecondaryMemberAnchor)> = None;
        for e in &self.elements {
            if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
                continue;
            }
            let (Some(a), Some(b)) = (self.node(e.nodes[0]), self.node(e.nodes[1])) else {
                continue;
            };
            let Some((t, d)) = point_on_axis(p, a.coord, b.coord, tol) else {
                continue;
            };
            if primary.map(|(best, _)| d < best).unwrap_or(true) {
                primary = Some((
                    d,
                    SecondaryMemberAnchor {
                        support: SupportMemberId::Primary(e.id),
                        position: t,
                    },
                ));
            }
        }
        if let Some((_, anchor)) = primary {
            return Some(anchor);
        }
        let mut secondary: Option<(f64, SecondaryMemberAnchor)> = None;
        for sm in self.joists().chain(self.posts()) {
            if Some(sm.id) == exclude {
                continue;
            }
            let Some((a, b)) = self.support_member_axis(SupportMemberId::Secondary(sm.id)) else {
                continue;
            };
            let Some((t, d)) = point_on_axis_interior(p, a, b, tol) else {
                continue;
            };
            let anchor = SecondaryMemberAnchor {
                support: SupportMemberId::Secondary(sm.id),
                position: t,
            };
            if secondary.map(|(best, _)| d < best).unwrap_or(true) {
                secondary = Some((d, anchor));
            }
        }
        secondary.map(|(_, anchor)| anchor)
    }

    /// 端点座標 [mm] と端部支持条件から、両端を取付き位置表現へ解決する。支持端は
    /// [`Self::resolve_point_anchor`] でアンカーへ、自由端は支持端を基準とした親構面内
    /// ベクトル（[`SecondaryMemberEnds::Cantilever`]）へ変換する。支持端のアンカーを
    /// 解決できない部材は生座標の [`SecondaryMemberEnds::Detached`] を返す
    /// （片持ちへの読み替えはしない）。
    pub fn secondary_ends_from_coords(
        &self,
        id: SecondaryMemberId,
        kind: SecondaryMemberKind,
        coords: [[f64; 3]; 2],
        supported: [bool; 2],
    ) -> SecondaryMemberEnds {
        let a0 = if supported[0] {
            self.resolve_point_anchor(coords[0], Some(id))
        } else {
            None
        };
        let a1 = if supported[1] {
            self.resolve_point_anchor(coords[1], Some(id))
        } else {
            None
        };
        let cantilever = |anchor: SecondaryMemberAnchor, free: [f64; 3]| {
            let support = self.anchor_point(anchor)?;
            let vector = self.free_end_vector(kind, support, free)?;
            Some(SecondaryMemberEnds::Cantilever {
                support: anchor,
                free_end_vector: vector,
            })
        };
        match (a0, a1) {
            (Some(a), Some(b)) if a != b => SecondaryMemberEnds::Supported([a, b]),
            (Some(a), None) if !supported[1] => {
                cantilever(a, coords[1]).unwrap_or(SecondaryMemberEnds::Detached(coords))
            }
            (None, Some(b)) if !supported[0] => {
                cantilever(b, coords[0]).unwrap_or(SecondaryMemberEnds::Detached(coords))
            }
            _ => SecondaryMemberEnds::Detached(coords),
        }
    }

    /// 支持端 `support` と自由端 `free` の座標 [mm] から、親構面内の自由端ベクトル [mm]
    /// を返す。小梁は水平面（XY）、間柱は鉛直構面の局所座標 `(s, z)`。
    fn free_end_vector(
        &self,
        kind: SecondaryMemberKind,
        support: [f64; 3],
        free: [f64; 3],
    ) -> Option<[f64; 2]> {
        let ds = [free[0] - support[0], free[1] - support[1]];
        match kind {
            SecondaryMemberKind::Joist => Some(ds),
            SecondaryMemberKind::Post => {
                let horiz = (ds[0] * ds[0] + ds[1] * ds[1]).sqrt();
                if horiz <= crate::geom::MEMBER_AXIS_TOL_MM {
                    Some([0.0, free[2] - support[2]])
                } else {
                    let direction = self.wall_plane_direction_at(support)?;
                    Some([
                        ds[0] * direction[0] + ds[1] * direction[1],
                        free[2] - support[2],
                    ])
                }
            }
        }
    }

    /// 全二次部材の両端を支持部材アンカー（片持ちは支持端＋自由端ベクトル）へ
    /// 一括で解決する。幾何的に支持のない端は自由端と推定する。アンカーへ解決できない
    /// 部材は生座標の [`SecondaryMemberEnds::Detached`] として残し、重量・荷重の欠落を
    /// 避けつつ解析前チェックでエラーにする（片持ちへの読み替えはしない）。
    pub fn anchorize_secondary_members(&mut self) -> SecondaryAnchorizeReport {
        let members: Vec<(SecondaryMemberId, SecondaryMemberKind, [f64; 3], [f64; 3])> = self
            .joists()
            .chain(self.posts())
            .filter_map(|sm| {
                let (a, b) = self.secondary_member_end_points(sm)?;
                Some((sm.id, sm.kind, a, b))
            })
            .collect();
        let free_flags = self.infer_free_ends(&members);

        let mut resolved: std::collections::HashMap<SecondaryMemberId, SecondaryMemberEnds> =
            std::collections::HashMap::new();
        let mut unresolved = Vec::new();
        let mut inferred_free_ends = Vec::new();
        for (i, (id, kind, a, b)) in members.iter().enumerate() {
            for (end, free) in free_flags[i].iter().enumerate() {
                if *free {
                    inferred_free_ends.push((*id, end));
                }
            }
            let supported = [!free_flags[i][0], !free_flags[i][1]];
            let ends = self.secondary_ends_from_coords(*id, *kind, [*a, *b], supported);
            if matches!(ends, SecondaryMemberEnds::Detached(_)) {
                unresolved.push(*id);
            }
            resolved.insert(*id, ends);
        }
        let apply = |sm: &mut SecondaryMember| {
            if let Some(ends) = resolved.get(&sm.id) {
                sm.ends = *ends;
            }
        };
        for sm in &mut self.unassigned_joists {
            apply(sm);
        }
        for sm in &mut self.unassigned_posts {
            apply(sm);
        }
        for region in &mut self.floor_regions {
            for sm in &mut region.secondary_joists {
                apply(sm);
            }
        }
        for region in &mut self.wall_regions {
            for sm in &mut region.posts {
                apply(sm);
            }
        }

        SecondaryAnchorizeReport {
            resolved: members.len() - unresolved.len(),
            inferred_free_ends,
            unresolved,
        }
    }
}

/// [`Model::anchorize_secondary_members`] の結果。
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct SecondaryAnchorizeReport {
    /// 支持部材アンカーへ解決できた部材数。
    pub resolved: usize,
    /// 自由端と推定した端 `(部材 ID, 端番号)`。
    pub inferred_free_ends: Vec<(SecondaryMemberId, usize)>,
    /// アンカーへ解決できなかった部材 ID（[`SecondaryMemberEnds::Detached`] として残る）。
    pub unresolved: Vec<SecondaryMemberId>,
}

/// 2 本の二次部材の材軸が連続しているか。共有端点から遠い方の端点が、相手の
/// 材軸直線から [`crate::geom::MEMBER_AXIS_TOL_MM`] 以内にあることで判定する。
fn axes_are_collinear(
    a: (SecondaryMemberId, SecondaryMemberKind, [f64; 3], [f64; 3]),
    b: (SecondaryMemberId, SecondaryMemberKind, [f64; 3], [f64; 3]),
) -> bool {
    let (a0, a1) = (a.2, a.3);
    let (b0, b1) = (b.2, b.3);
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

/// 二次部材の安定 ID 重複・アンカー参照・材軸位置範囲・支持グラフ循環・片持ち自由端を
/// 検証する。各端は [`SecondaryMemberEnds`] の支持部材アンカーまたは生座標で表す。
pub fn validate_secondary_members(
    members: &[&SecondaryMember],
) -> Result<(), crate::error::CoreError> {
    use crate::error::CoreError;
    let mut attest: Vec<(SecondaryMemberId, SecondaryMemberEnds)> = Vec::new();
    for sm in members {
        attest.push((sm.id, sm.ends));
    }
    let mut ids = std::collections::HashSet::new();
    for (id, ends) in &attest {
        if !ids.insert(*id) {
            return Err(CoreError::DuplicateId(format!(
                "SecondaryMemberId({})",
                id.0
            )));
        }
        match *ends {
            SecondaryMemberEnds::Supported(anchors) => {
                validate_anchor(anchors[0], *id)?;
                validate_anchor(anchors[1], *id)?;
                if anchors[0] == anchors[1] {
                    return Err(CoreError::DanglingRef(format!(
                        "SecondaryMember {} の両端が同じ取付き位置",
                        id.0
                    )));
                }
            }
            SecondaryMemberEnds::Cantilever {
                support,
                free_end_vector,
            } => {
                validate_anchor(support, *id)?;
                if !free_end_vector.iter().all(|v| v.is_finite()) || free_end_vector == [0.0, 0.0] {
                    return Err(CoreError::DanglingRef(format!(
                        "SecondaryMember {} の自由端ベクトルが不正",
                        id.0
                    )));
                }
            }
            SecondaryMemberEnds::Detached(coords) => {
                if !coords.iter().flatten().all(|v| v.is_finite()) {
                    return Err(CoreError::DanglingRef(format!(
                        "SecondaryMember {} の端点座標が不正",
                        id.0
                    )));
                }
            }
        }
    }
    for (id, ends) in &attest {
        for anchor in ends.anchors() {
            if let SupportMemberId::Secondary(target) = anchor.support {
                if target == *id || !ids.contains(&target) {
                    return Err(CoreError::DanglingRef(format!(
                        "SecondaryMember {} -> SecondaryMember {}",
                        id.0, target.0
                    )));
                }
            }
        }
    }
    reject_support_cycles(&attest)
}

/// 支持グラフ（二次部材 → 支持する二次部材）に閉路があれば拒否する。
fn reject_support_cycles(
    members: &[(SecondaryMemberId, SecondaryMemberEnds)],
) -> Result<(), crate::error::CoreError> {
    use crate::error::CoreError;
    #[derive(Clone, Copy, PartialEq)]
    enum Mark {
        Visiting,
        Done,
    }
    let index_of: std::collections::HashMap<SecondaryMemberId, usize> =
        members.iter().enumerate().map(|(i, m)| (m.0, i)).collect();
    fn visit(
        i: usize,
        members: &[(SecondaryMemberId, SecondaryMemberEnds)],
        index_of: &std::collections::HashMap<SecondaryMemberId, usize>,
        marks: &mut [Option<Mark>],
    ) -> Result<(), CoreError> {
        match marks[i] {
            Some(Mark::Done) => return Ok(()),
            Some(Mark::Visiting) => {
                return Err(CoreError::DanglingRef(format!(
                    "二次部材の支持グラフが循環しています（SecondaryMemberId({})）",
                    members[i].0 .0
                )));
            }
            None => {}
        }
        marks[i] = Some(Mark::Visiting);
        for anchor in members[i].1.anchors() {
            if let SupportMemberId::Secondary(id) = anchor.support {
                if let Some(&j) = index_of.get(&id) {
                    visit(j, members, index_of, marks)?;
                }
            }
        }
        marks[i] = Some(Mark::Done);
        Ok(())
    }
    let mut marks = vec![None; members.len()];
    for i in 0..members.len() {
        visit(i, members, &index_of, &mut marks)?;
    }
    Ok(())
}

fn validate_anchor(
    anchor: SecondaryMemberAnchor,
    member: SecondaryMemberId,
) -> Result<(), crate::error::CoreError> {
    if !anchor.position.is_finite() || !(0.0..=1.0).contains(&anchor.position) {
        return Err(crate::error::CoreError::DanglingRef(format!(
            "SecondaryMember {} の取付き位置が範囲外",
            member.0
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node, SecondaryMemberKind,
    };

    fn node(id: u32, coord: [f64; 3]) -> Node {
        Node {
            id: NodeId(id),
            coord,
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    fn beam(id: u32, a: u32, b: u32) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
            section: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    fn two_girder_model() -> Model {
        Model {
            nodes: vec![
                node(0, [0.0, 0.0, 0.0]),
                node(1, [4000.0, 0.0, 0.0]),
                node(2, [0.0, 4000.0, 0.0]),
                node(3, [4000.0, 4000.0, 0.0]),
                node(4, [2000.0, 0.0, 0.0]),
                node(5, [2000.0, 4000.0, 0.0]),
            ],
            elements: vec![beam(0, 0, 1), beam(1, 2, 3)],
            ..Default::default()
        }
    }

    fn joist(id: u32, coords: [[f64; 3]; 2]) -> SecondaryMember {
        SecondaryMember {
            id: SecondaryMemberId(id),
            kind: SecondaryMemberKind::Joist,
            ends: SecondaryMemberEnds::Detached(coords),
            section: None,
            name: format!("J{id}"),
        }
    }

    #[test]
    fn 節点を支持部材の材軸位置へ解決する() {
        let model = two_girder_model();
        let a = model
            .resolve_point_anchor([2000.0, 0.0, 0.0], None)
            .expect("大梁 0 の材軸上");
        assert_eq!(a.support, SupportMemberId::Primary(ElemId(0)));
        assert!((a.position - 0.5).abs() < 1e-9, "{a:?}");
        let b = model
            .resolve_point_anchor([2000.0, 4000.0, 0.0], None)
            .expect("大梁 1 の材軸上");
        assert_eq!(b.support, SupportMemberId::Primary(ElemId(1)));
    }

    #[test]
    fn 両端節点から両端アンカーを得る() {
        let mut model = two_girder_model();
        model
            .unassigned_joists
            .push(joist(0, [[2000.0, 0.0, 0.0], [2000.0, 4000.0, 0.0]]));

        let report = model.anchorize_secondary_members();
        assert_eq!((report.resolved, report.unresolved.len()), (1, 0));
        assert_eq!(
            model.unassigned_joists[0].ends,
            SecondaryMemberEnds::Supported([
                SecondaryMemberAnchor {
                    support: SupportMemberId::Primary(ElemId(0)),
                    position: 0.5,
                },
                SecondaryMemberAnchor {
                    support: SupportMemberId::Primary(ElemId(1)),
                    position: 0.5,
                },
            ])
        );
        assert!(model.validate().is_ok());
    }

    #[test]
    fn 自由端は支持端基準のベクトルになる() {
        let mut model = two_girder_model();
        model
            .unassigned_joists
            .push(joist(0, [[2000.0, 0.0, 0.0], [2000.0, 1000.0, 0.0]]));

        let report = model.anchorize_secondary_members();
        assert_eq!((report.resolved, report.unresolved.len()), (1, 0));
        assert_eq!(
            model.unassigned_joists[0].ends,
            SecondaryMemberEnds::Cantilever {
                support: SecondaryMemberAnchor {
                    support: SupportMemberId::Primary(ElemId(0)),
                    position: 0.5,
                },
                free_end_vector: [0.0, 1000.0],
            }
        );
        assert!(model.validate().is_ok());
    }

    #[test]
    fn 支持のない端は生座標のまま残す() {
        let mut model = two_girder_model();
        model.unassigned_posts.push(SecondaryMember {
            id: SecondaryMemberId(0),
            kind: SecondaryMemberKind::Post,
            ends: SecondaryMemberEnds::Detached([[0.0, 0.0, 3000.0], [2000.0, 0.0, 3000.0]]),
            section: None,
            name: "P0".into(),
        });

        let report = model.anchorize_secondary_members();
        assert_eq!(report.resolved, 0);
        assert_eq!(report.unresolved, vec![SecondaryMemberId(0)]);
        assert!(model.unassigned_posts[0].is_detached());
    }
}
