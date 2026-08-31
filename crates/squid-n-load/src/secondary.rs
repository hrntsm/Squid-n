//! 二次部材（小梁・間柱）経由の荷重を主架構へ変換する（CMQ 経路）。
//!
//! 二次部材の支持点や床板の角は、主架構の梁（大梁）のスパン中間に
//! 節点共有なしで載ることがある（ST-Bridge 取り込みモデルの典型）。
//! そのような「要素が接続しない節点」への集中荷重は解析に載らないため
//! （`DofMap::build` が自由度から除外する）、載っている梁の
//! **中間集中荷重**（`MemberLoadKind::Point`。大梁の CMQ）へ変換する。

use squid_n_core::ids::ElemId;
use squid_n_core::model::{ElementKind, MemberLoad, MemberLoadKind, Model, NodalLoad};

#[cfg(test)]
use squid_n_core::ids::NodeId;
#[cfg(test)]
use squid_n_core::model::SlabShape;

/// `beam_span_position`/`resolve_nodal_to_primary` が候補とする 2 節点 `Beam` 要素
/// （要素独立な幾何のみ。ダングリング参照は除外済み）。`resolve_nodal_to_primary` が
/// 荷重ごとに `model.elements` を走査し直す（かつ節点座標を都度引き直す）のを避け、
/// 呼び出し 1 回につき 1 度だけ構築して使い回すための中間表現。
pub struct BeamSpanCandidate {
    id: ElemId,
    a: [f64; 3],
    b: [f64; 3],
}

/// `model` から `beam_span_position` の対象となる 2 節点 `Beam` 要素の候補列
/// （端点座標つき）を集める。ダングリング参照（未検証モデルで節点が見つからない
/// 要素）はここで読み飛ばす（この要素だけを除外し、他要素の探索は継続する。
/// 元の `beam_span_position` の「1 要素の不整合で全体を打ち切らない」挙動を保つ）。
pub fn beam_span_candidates(model: &Model) -> Vec<BeamSpanCandidate> {
    let mut out = Vec::new();
    for e in &model.elements {
        if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
            continue;
        }
        let (Some(node_a), Some(node_b)) = (
            model.nodes.get(e.nodes[0].index()),
            model.nodes.get(e.nodes[1].index()),
        ) else {
            continue;
        };
        out.push(BeamSpanCandidate {
            id: e.id,
            a: node_a.coord,
            b: node_b.coord,
        });
    }
    out
}

/// `beam_span_position` の本体。事前構築済みの候補列 `candidates` から、`coord` が
/// スパン上（端点を除く、距離 `tol` [mm] 以内）にある最も近い梁を探す。
/// 複数の梁に載る場合は最も近いものを返す（`d < bd` の狭義比較により、同着なら
/// 候補列で先に見つかったものを保持する＝要素順の先勝ち）。
pub fn best_span_position(
    candidates: &[BeamSpanCandidate],
    coord: [f64; 3],
    tol: f64,
) -> Option<(ElemId, f64)> {
    let mut best: Option<(ElemId, f64, f64)> = None; // (elem, a, dist)
    for c in candidates {
        let (a, b) = (c.a, c.b);
        let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
        if len2 < 1.0 {
            continue;
        }
        let ap = [coord[0] - a[0], coord[1] - a[1], coord[2] - a[2]];
        let t = (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2;
        let len = len2.sqrt();
        // 端点そのもの（節点共有で解決すべき位置）は対象外。端点近傍 tol 以内は
        // スパン内側へ丸める。
        let a_pos = (t * len).clamp(0.0, len);
        if a_pos <= tol || a_pos >= len - tol {
            continue;
        }
        let proj = [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]];
        let d = ((coord[0] - proj[0]).powi(2)
            + (coord[1] - proj[1]).powi(2)
            + (coord[2] - proj[2]).powi(2))
        .sqrt();
        if d <= tol && best.map(|(_, _, bd)| d < bd).unwrap_or(true) {
            best = Some((c.id, a_pos, d));
        }
    }
    best.map(|(e, a, _)| (e, a))
}

/// 節点座標が梁要素のスパン上（端点を除く、距離 `tol` [mm] 以内）にあれば
/// `(要素 ID, i 端からの距離 a)` を返す。複数の梁に載る場合は最も近いものを返す。
///
/// 単発呼び出し向けの簡便版（候補列をこの呼び出し内だけで構築する）。
/// `resolve_nodal_to_primary` のように同一モデルに対して複数回（荷重ごとに）
/// 呼ぶ場合は、`beam_span_candidates` で事前構築した候補列を `best_span_position`
/// へ渡して使い回すこと（要素走査・節点座標引きの重複を避ける）。
pub fn beam_span_position(model: &Model, coord: [f64; 3], tol: f64) -> Option<(ElemId, f64)> {
    let candidates = beam_span_candidates(model);
    best_span_position(&candidates, coord, tol)
}

/// 線分が載る大梁の 1 区間（[`beams_along_segment`] の結果）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SegmentCoverage {
    /// 覆っている梁要素。
    pub elem: ElemId,
    /// 線分上の被覆区間（線分の始点からの距離 [mm]）。
    pub seg: [f64; 2],
    /// 梁の i 端からの距離で表した被覆区間 [mm]（`seg` と同じ向きに並ぶ）。
    pub elem_pos: [f64; 2],
}

/// 線分 `p0`→`p1` を覆う大梁の区間を、線分に沿った順で返す。
///
/// 取り付き領域の取付き線のように、**主架構の上に載る線分**を実際の梁へ割り付けるために使う。
/// 取付き線の両端は領域が参照する節点だが、その間の大梁が中間節点で分割されていると、
/// 節点対の完全一致では梁が見つからない。ここでは幾何で覆いを求めるため、分割の有無に依らない。
///
/// 判定は次のとおり。梁の両端が線分の直線上（直交距離 `tol` 以内）にあり、線分方向への
/// 射影が線分と重なる梁を集め、重なり区間を返す。線分に載っていない梁・重なりが
/// `tol` 以下の梁は返さない。返り値は線分の始点側から並ぶ。
///
/// **覆いが線分の全長に満たない場合もそのまま返す**（呼び出し側が全長を覆えたかを判断し、
/// 覆えないぶんの荷重の行き先を決める）。
pub fn beams_along_segment(
    model: &Model,
    p0: [f64; 3],
    p1: [f64; 3],
    tol: f64,
) -> Vec<SegmentCoverage> {
    let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if len <= tol {
        return Vec::new();
    }
    let u = [d[0] / len, d[1] / len, d[2] / len];
    // 線分の直線からの直交距離。
    let perp_dist = |q: [f64; 3]| -> f64 {
        let v = [q[0] - p0[0], q[1] - p0[1], q[2] - p0[2]];
        let t = v[0] * u[0] + v[1] * u[1] + v[2] * u[2];
        let w = [v[0] - t * u[0], v[1] - t * u[1], v[2] - t * u[2]];
        (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt()
    };
    let along = |q: [f64; 3]| -> f64 {
        (q[0] - p0[0]) * u[0] + (q[1] - p0[1]) * u[1] + (q[2] - p0[2]) * u[2]
    };

    let mut out = Vec::new();
    for c in beam_span_candidates(model) {
        if perp_dist(c.a) > tol || perp_dist(c.b) > tol {
            continue; // 線分の直線上にない梁。
        }
        let (ta, tb) = (along(c.a), along(c.b));
        let (lo, hi) = (ta.min(tb), ta.max(tb));
        let s0 = lo.max(0.0);
        let s1 = hi.min(len);
        if s1 - s0 <= tol {
            continue; // 重なっていない（端点で接するだけを含む）。
        }
        // 梁の i 端からの距離へ写す（梁が線分と逆向きでも `seg` と同じ並びにする）。
        let elem_len = (tb - ta).abs();
        let to_elem = |t: f64| -> f64 {
            if tb >= ta {
                (t - ta).clamp(0.0, elem_len)
            } else {
                (ta - t).clamp(0.0, elem_len)
            }
        };
        out.push(SegmentCoverage {
            elem: c.id,
            seg: [s0, s1],
            elem_pos: [to_elem(s0), to_elem(s1)],
        });
    }
    out.sort_by(|a, b| a.seg[0].total_cmp(&b.seg[0]));
    out
}

/// 小梁（両端 `a`・`b`）の負担幅 [mm] を、その辺に沿う床板の幾何から求める。
///
/// ST-Bridge 取り込みの床板は小梁で細分された小片として入ってくるため、小梁の
/// 辺は隣接する床板の境界辺と重なる。**同じレベル**の床板の中から、この辺に沿う
/// 境界辺を持つ床板を探し（T 字取り付き等で、隣の床板の境界がこの辺の途中の
/// 節点で分割されている場合も、節点対の完全一致ではなく直線への重なりで判定
/// するため取りこぼさない。レベルを見ないと、上下階で平面が繰り返される建物では
/// 別階の床板を拾ってしまう）、見つかった床板の直交方向の幅の合計を半分にして返す
/// （大梁-小梁-大梁の標準的な負担幅規則。片側 1 枚を代表に選ぶ・全体を
/// 床板の枚数で単純平均するといった近似はしない。「合計の半分」は、この
/// 小梁の左右どちらに何枚の床板があっても、側ごとの合計の半分を足し合わせた
/// ものと数学的に等しい）。
///
/// 小梁がどの床板の境界にも沿わない（床板の内部を通る、対応する床板がない、
/// 取り付く床板しかない等）場合は `None`。
///
/// **二次部材小梁の本番検定では使わない**（分配 `Span` 経路が正。§5.39 以降）。
/// §5.5 当時の幾何負担幅の回帰テスト専用。
#[cfg(test)]
fn joist_edge_tributary_width(model: &Model, a: NodeId, b: NodeId) -> Option<f64> {
    let na = model.nodes.get(a.index())?;
    let nb = model.nodes.get(b.index())?;
    let dx = nb.coord[0] - na.coord[0];
    let dy = nb.coord[1] - na.coord[1];
    let len = (dx * dx + dy * dy).sqrt();
    if len <= 1e-9 {
        return None;
    }
    let dir = [dx / len, dy / len];
    let perp = [-dir[1], dir[0]];
    let tol = SPAN_TOL_MM;
    let mid_z = (na.coord[2] + nb.coord[2]) / 2.0;

    let mut sum = 0.0_f64;
    let mut found = false;

    for slab in &model.slabs {
        let SlabShape::Enclosed { boundary } = &slab.shape else {
            continue; // 取り付く床板は境界辺を持たない。
        };
        let n = boundary.len();
        if n < 3 {
            continue;
        }
        // 同じレベルの床板だけを対象にする。XY だけで判定すると、上下階で
        // 平面が繰り返される建物では別階の床板を誤って拾う。
        if !slab
            .level(model)
            .is_some_and(|z| (z - mid_z).abs() <= squid_n_core::geom::LEVEL_TOL_MM)
        {
            continue;
        }
        let Some(coords) = slab.boundary_coords(model) else {
            continue;
        };
        // 境界辺のいずれかが、小梁の直線に載り（直交距離 tol 以内）、かつ辺自身が
        // 小梁の区間 [0, len] にほぼ収まるか（1 辺で全長を覆う必要はない。隣が
        // 複数枚に割れていれば、それぞれ収まるぶんで検出できればよい）。
        //
        // 「辺自身が収まる」まで求めるのは、同じ通り芯上にある無関係な遠くの辺
        // （この小梁の延長線上だが別スパンの床板の境界）を誤って拾わないため。
        // 区間の交差が非零というだけの判定では、小梁の [0, len] を大きく越えて
        // 延びる辺（無関係な床板）まで「重なりあり」になってしまう。
        let overlaps = (0..n).any(|k| {
            let p = coords[k];
            let q = coords[(k + 1) % n];
            let dp = (p[0] - na.coord[0]) * perp[0] + (p[1] - na.coord[1]) * perp[1];
            let dq = (q[0] - na.coord[0]) * perp[0] + (q[1] - na.coord[1]) * perp[1];
            if dp.abs() > tol || dq.abs() > tol {
                return false;
            }
            let tp = (p[0] - na.coord[0]) * dir[0] + (p[1] - na.coord[1]) * dir[1];
            let tq = (q[0] - na.coord[0]) * dir[0] + (q[1] - na.coord[1]) * dir[1];
            let edge_len = (tq - tp).abs();
            // 退化した（ほぼ長さ 0 の）辺は除く。`edge_len - tol` が負になり、
            // 区間 [0, len] の外（クランプで t0 == t1 になる位置）でも
            // 「収まっている」と誤判定してしまうため。
            if edge_len <= tol || edge_len > len + tol {
                return false; // 小梁より長い辺は別の床板の境界とみなす。
            }
            let t0 = tp.min(tq).clamp(0.0, len);
            let t1 = tp.max(tq).clamp(0.0, len);
            t1 - t0 >= edge_len - tol
        });
        if !overlaps {
            continue;
        }

        let width = if let Some((lx, ly)) = crate::floor::slab_dimensions(model, slab) {
            if dir[0].abs() >= dir[1].abs() {
                ly
            } else {
                lx
            }
        } else {
            let (mut mn, mut mx) = (f64::INFINITY, f64::NEG_INFINITY);
            for c in &coords {
                let t = c[0] * perp[0] + c[1] * perp[1];
                mn = mn.min(t);
                mx = mx.max(t);
            }
            mx - mn
        };

        found = true;
        sum += width;
    }

    found.then_some(sum / 2.0)
}

/// 各節点に要素（解析部材）が接続しているかを返す。
pub fn node_connected_flags(model: &Model) -> Vec<bool> {
    let mut connected = vec![false; model.nodes.len()];
    for e in &model.elements {
        for n in &e.nodes {
            if let Some(slot) = connected.get_mut(n.index()) {
                *slot = true;
            }
        }
    }
    connected
}

/// 「要素が接続しない節点」への節点荷重を、その節点が載っている主架構梁の
/// 中間集中荷重（CMQ）へ変換する。
///
/// - 要素が接続する節点への荷重: そのまま `NodalLoad` として返す。
/// - 接続しない節点で、力成分（並進）が非零・モーメント成分が零、かつ節点が
///   梁スパン上（±`tol`）にある: `MemberLoad`（`Point{a, p}`、`dir` = 力の方向）
///   へ変換する。
/// - 変換できない荷重（モーメント付き・どの梁にも載らない等）: `NodalLoad` の
///   まま返す（解析では零剛性節点として無視されるが、荷重タブでは見える）。
///
/// 変換は冪等（変換済みの出力を再度通しても変化しない）。
pub fn resolve_nodal_to_primary(
    model: &Model,
    nodal: Vec<NodalLoad>,
    tol: f64,
) -> (Vec<NodalLoad>, Vec<MemberLoad>) {
    let connected = node_connected_flags(model);
    // 要素独立な梁候補（節点対応付け前の端点座標）を 1 回だけ構築し、荷重ごとの
    // `beam_span_position` 呼び出し（= 全要素走査＋座標引き直し）を避ける
    // （性能。候補列・探索ロジックは `beam_span_position` と共通のため挙動は不変）。
    let candidates = beam_span_candidates(model);
    let mut out_nodal = Vec::new();
    let mut out_member = Vec::new();
    for nl in nodal {
        let ni = nl.node.index();
        if connected.get(ni).copied().unwrap_or(false) {
            out_nodal.push(nl);
            continue;
        }
        let f = [nl.values[0], nl.values[1], nl.values[2]];
        let p = (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt();
        let has_moment = nl.values[3..6].iter().any(|m| m.abs() > 1e-9);
        if p <= 1e-9 || has_moment {
            out_nodal.push(nl);
            continue;
        }
        let Some(node) = model.nodes.get(ni) else {
            out_nodal.push(nl);
            continue;
        };
        match best_span_position(&candidates, node.coord, tol) {
            // 変換後も元の荷重の素性（名称・生成元）を引き継ぐ。
            Some((elem, a)) => out_member.push(MemberLoad {
                elem,
                dir: [f[0] / p, f[1] / p, f[2] / p],
                kind: MemberLoadKind::Point { a, p },
                name: nl.name.clone(),
                source: nl.source,
            }),
            None => out_nodal.push(nl),
        }
    }
    (out_nodal, out_member)
}

/// 節点→梁スパン変換の既定許容差 [mm]（大梁芯からのずれの許容）。
///
/// 判定規則の情報源を 1 つに保つため、値は [`squid_n_core::geom::MEMBER_AXIS_TOL_MM`]
/// を用いる（節点を共有せずに交差・接触する梁の診断も同じ許容差で判定する）。
pub const SPAN_TOL_MM: f64 = squid_n_core::geom::MEMBER_AXIS_TOL_MM;

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, EndCondition, ForceRegime, LocalAxis, Node, SecondaryMember,
        SecondaryMemberKind,
    };

    fn node(id: u32, x: f64, y: f64, z: f64) -> Node {
        Node {
            id: NodeId(id),
            coord: [x, y, z],
            restraint: Dof6Mask::FREE,
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

    /// 大梁（0-1, 長さ6000）のスパン上 x=2000 にある非接続節点(2)への鉛直荷重が、
    /// 大梁の中間集中荷重 Point{a=2000} へ変換される。接続節点(0)への荷重はそのまま。
    #[test]
    fn test_resolve_nodal_to_primary_converts_span_node() {
        let model = Model {
            nodes: vec![
                node(0, 0.0, 0.0, 0.0),
                node(1, 6000.0, 0.0, 0.0),
                node(2, 2000.0, 0.0, 0.0),
            ],
            elements: vec![beam(0, 0, 1)],
            unassigned_joists: vec![SecondaryMember {
                kind: SecondaryMemberKind::Joist,
                nodes: [NodeId(2), NodeId(2)],
                section: Some(SectionId(0)),
                name: "b1".into(),
            }],
            ..Default::default()
        };
        let nodal = vec![
            NodalLoad::manual(NodeId(2), [0.0, 0.0, -5000.0, 0.0, 0.0, 0.0]),
            NodalLoad::manual(NodeId(0), [0.0, 0.0, -1000.0, 0.0, 0.0, 0.0]),
        ];
        let (out_nodal, out_member) = resolve_nodal_to_primary(&model, nodal, SPAN_TOL_MM);
        assert_eq!(out_nodal.len(), 1);
        assert_eq!(out_nodal[0].node, NodeId(0));
        assert_eq!(out_member.len(), 1);
        assert_eq!(out_member[0].elem, ElemId(0));
        assert_eq!(out_member[0].dir, [0.0, 0.0, -1.0]);
        match out_member[0].kind {
            MemberLoadKind::Point { a, p } => {
                assert!((a - 2000.0).abs() < 1e-9);
                assert!((p - 5000.0).abs() < 1e-9);
            }
            _ => panic!("Point になるはず"),
        }
        // 冪等: 変換済み nodal を再度通しても変化しない。
        let (again_nodal, again_member) =
            resolve_nodal_to_primary(&model, out_nodal.clone(), SPAN_TOL_MM);
        assert_eq!(again_nodal, out_nodal);
        assert!(again_member.is_empty());
    }

    /// ダングリング参照（未検証モデル）を持つ梁が先に走査されても、後続の
    /// 正しい梁でスパン位置が見つかる（1 要素のダングリング参照で関数全体が
    /// 空振りしないことの回帰テスト）。
    #[test]
    fn test_beam_span_position_skips_dangling_element() {
        let model = Model {
            nodes: vec![node(0, 0.0, 0.0, 0.0), node(1, 6000.0, 0.0, 0.0)],
            elements: vec![
                // 存在しない節点(99)を参照するダングリング要素を先頭に置く。
                beam(0, 99, 98),
                beam(1, 0, 1),
            ],
            ..Default::default()
        };
        let hit = beam_span_position(&model, [2000.0, 0.0, 0.0], SPAN_TOL_MM);
        assert_eq!(hit, Some((ElemId(1), 2000.0)));
    }

    /// どの梁にも載らない非接続節点への荷重は NodalLoad のまま返る。
    #[test]
    fn test_resolve_nodal_to_primary_keeps_unresolvable() {
        let model = Model {
            nodes: vec![
                node(0, 0.0, 0.0, 0.0),
                node(1, 6000.0, 0.0, 0.0),
                node(2, 3000.0, 5000.0, 0.0),
            ],
            elements: vec![beam(0, 0, 1)],
            ..Default::default()
        };
        let nodal = vec![NodalLoad::manual(
            NodeId(2),
            [0.0, 0.0, -5000.0, 0.0, 0.0, 0.0],
        )];
        let (out_nodal, out_member) = resolve_nodal_to_primary(&model, nodal, SPAN_TOL_MM);
        assert_eq!(out_nodal.len(), 1);
        assert!(out_member.is_empty());
    }
}

#[cfg(test)]
mod segment_tests {
    use super::*;
    use squid_n_core::ids::{NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node,
    };

    fn model_with(xs: &[f64], beams: &[(u32, u32)]) -> Model {
        let mut m = Model::default();
        for (i, x) in xs.iter().enumerate() {
            m.nodes.push(Node {
                id: NodeId(i as u32),
                coord: [*x, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            });
        }
        for (k, (i, j)) in beams.iter().enumerate() {
            m.elements.push(ElementData {
                id: ElemId(k as u32),
                kind: ElementKind::Beam,
                nodes: [NodeId(*i), NodeId(*j)].into_iter().collect(),
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
        }
        m
    }

    /// 分割された大梁（A—M—B）を線分 A→B が覆う。区間は線分の始点側から並ぶ。
    #[test]
    fn test_covers_subdivided_beams_in_order() {
        let m = model_with(&[0.0, 2000.0, 4000.0], &[(0, 1), (1, 2)]);
        let cover = beams_along_segment(&m, [0.0, 0.0, 0.0], [4000.0, 0.0, 0.0], SPAN_TOL_MM);
        assert_eq!(cover.len(), 2);
        assert_eq!(cover[0].elem, ElemId(0));
        assert_eq!(cover[0].seg, [0.0, 2000.0]);
        assert_eq!(cover[1].elem, ElemId(1));
        assert_eq!(cover[1].seg, [2000.0, 4000.0]);
        let covered: f64 = cover.iter().map(|c| c.seg[1] - c.seg[0]).sum();
        assert!((covered - 4000.0).abs() < 1e-9, "全長を覆う");
    }

    /// 梁が線分と逆向きでも、梁上の位置は線分の向きに合わせて返す。
    #[test]
    fn test_reversed_beam_positions_follow_segment() {
        let m = model_with(&[0.0, 4000.0], &[(1, 0)]);
        let cover = beams_along_segment(&m, [0.0, 0.0, 0.0], [4000.0, 0.0, 0.0], SPAN_TOL_MM);
        assert_eq!(cover.len(), 1);
        // 梁の i 端は x=4000 側なので、線分始点（x=0）は梁の 4000 の位置に当たる。
        assert!((cover[0].elem_pos[0] - 4000.0).abs() < 1e-9);
        assert!((cover[0].elem_pos[1] - 0.0).abs() < 1e-9);
    }

    /// 線分の一部にしか梁がない場合は、その区間だけを返す（覆えたかは呼び出し側が判断する）。
    #[test]
    fn test_partial_coverage_is_reported_as_is() {
        let m = model_with(&[0.0, 2000.0], &[(0, 1)]);
        let cover = beams_along_segment(&m, [0.0, 0.0, 0.0], [4000.0, 0.0, 0.0], SPAN_TOL_MM);
        assert_eq!(cover.len(), 1);
        assert_eq!(cover[0].seg, [0.0, 2000.0]);
    }

    /// 線分から外れた（平行だが離れた）梁は覆いに数えない。
    #[test]
    fn test_offset_beam_is_not_covered() {
        let mut m = model_with(&[0.0, 4000.0], &[(0, 1)]);
        for n in &mut m.nodes {
            n.coord[1] = 1000.0; // 線分から 1m ずらす
        }
        let cover = beams_along_segment(&m, [0.0, 0.0, 0.0], [4000.0, 0.0, 0.0], SPAN_TOL_MM);
        assert!(cover.is_empty());
    }
}

#[cfg(test)]
mod joist_tributary_tests {
    use super::*;
    use squid_n_core::ids::SlabId;
    use squid_n_core::model::{DistributionMethod, Node, Slab, SlabPlate};

    fn node(id: u32, x: f64, y: f64) -> Node {
        Node {
            id: NodeId(id),
            coord: [x, y, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    fn rect_slab(id: u32, corners: [u32; 4]) -> Slab {
        Slab {
            id: SlabId(id),
            shape: SlabShape::Enclosed {
                boundary: corners.into_iter().map(NodeId).collect(),
            },
            plate: SlabPlate {
                section: None,
                loads: Vec::new(),
                usage: None,
                method: DistributionMethod::TriTrapezoid,
                one_way: None,
            },
        }
    }

    /// 4000×3000 と 4000×3000 の 2 枚が joist（節点 3-2）を挟んで隣り合う。
    /// 負担幅は両側の半分の和＝(3000/2)+(3000/2)=3000（joist 1 本だけで
    /// 6000 幅の間を分け合う、単純梁の負担幅と同じ値）。
    #[test]
    fn test_two_sided_pieces_split_evenly() {
        let mut model = Model {
            nodes: vec![
                node(0, 0.0, 0.0),
                node(1, 4000.0, 0.0),
                node(2, 4000.0, 3000.0),
                node(3, 0.0, 3000.0),
                node(4, 4000.0, 6000.0),
                node(5, 0.0, 6000.0),
            ],
            ..Default::default()
        };
        model.slabs.push(rect_slab(0, [0, 1, 2, 3]));
        model.slabs.push(rect_slab(1, [3, 2, 4, 5]));

        let w = joist_edge_tributary_width(&model, NodeId(3), NodeId(2));
        assert!(matches!(w, Some(x) if (x - 3000.0).abs() < 1e-6), "{w:?}");
        // 端点の順序を入れ替えても同じ結果（辺の向きに依存しない）。
        let w_rev = joist_edge_tributary_width(&model, NodeId(2), NodeId(3));
        assert_eq!(w, w_rev);
    }

    /// 建物の外周に載る joist（片側にしか床板がない）は、その片側の幅の半分。
    #[test]
    fn test_perimeter_joist_uses_single_side_half_width() {
        let mut model = Model {
            nodes: vec![
                node(0, 0.0, 0.0),
                node(1, 4000.0, 0.0),
                node(2, 4000.0, 3000.0),
                node(3, 0.0, 3000.0),
            ],
            ..Default::default()
        };
        model.slabs.push(rect_slab(0, [0, 1, 2, 3]));

        let w = joist_edge_tributary_width(&model, NodeId(0), NodeId(1));
        assert!(matches!(w, Some(x) if (x - 1500.0).abs() < 1e-6), "{w:?}");
    }

    /// T 字取り付き: 片側は 1 枚（幅 3000）、反対側は途中の節点で 2 枚に割れている
    /// （幅 3000 の帯が 2 枚。それぞれの境界はこの小梁の一部区間としか重ならない）。
    /// 負担幅は「割れた側の合計幅の半分」＋「割れていない側の幅の半分」
    /// ＝ (3000+3000)/2 + 3000/2 = 4500（3 枚の単純平均 3000 ではない）。
    #[test]
    fn test_t_junction_sums_split_side_before_halving() {
        let mut model = Model {
            nodes: vec![
                node(0, 0.0, 0.0),
                node(1, 4000.0, 0.0),
                node(2, 4000.0, 3000.0),
                node(3, 0.0, 3000.0),
                node(4, 4000.0, 6000.0),
                node(5, 0.0, 6000.0),
                node(6, 2000.0, 3000.0), // 反対側を割る、joist 辺の途中の節点
                node(7, 2000.0, 6000.0),
            ],
            ..Default::default()
        };
        model.slabs.push(rect_slab(0, [0, 1, 2, 3])); // 割れていない側
        model.slabs.push(rect_slab(1, [3, 6, 7, 5])); // 割れた側・左半分
        model.slabs.push(rect_slab(2, [6, 2, 4, 7])); // 割れた側・右半分

        let w = joist_edge_tributary_width(&model, NodeId(3), NodeId(2));
        assert!(matches!(w, Some(x) if (x - 4500.0).abs() < 1e-6), "{w:?}");
    }

    /// どの床板の境界辺にも載らない（床板の内部を貫く）小梁は None。
    /// 呼び出し側はここで別の幾何近似へフォールバックすること。
    #[test]
    fn test_interior_joist_returns_none() {
        let mut model = Model {
            nodes: vec![
                node(0, 0.0, 0.0),
                node(1, 4000.0, 0.0),
                node(2, 4000.0, 6000.0),
                node(3, 0.0, 6000.0),
                node(4, 0.0, 3000.0),
                node(5, 4000.0, 3000.0),
            ],
            ..Default::default()
        };
        model.slabs.push(rect_slab(0, [0, 1, 2, 3]));

        let w = joist_edge_tributary_width(&model, NodeId(4), NodeId(5));
        assert_eq!(w, None);
    }

    /// 小梁の直線上（同じ Y）だが遠く離れた、退化した（ほぼ長さ 0 の）辺を持つ
    /// 無関係な床板は拾わない。クランプ後の区間が [len, len]（幅 0）になっても、
    /// 辺自身が短ければ「収まっている」と誤判定しないことの回帰テスト。
    #[test]
    fn test_far_degenerate_edge_on_same_line_is_not_matched() {
        let mut model = Model {
            nodes: vec![
                node(0, 0.0, 0.0),
                node(1, 4000.0, 0.0),
                node(2, 4000.0, 3000.0),
                node(3, 0.0, 3000.0),
                // 遠く離れた、無関係な床板（ほぼ長さ 0 の辺 4-5 を持つ）。
                node(4, 50000.0, 0.0),
                node(5, 50005.0, 0.0),
                node(6, 50005.0, 1000.0),
                node(7, 50000.0, 1000.0),
            ],
            ..Default::default()
        };
        model.slabs.push(rect_slab(1, [4, 5, 6, 7]));

        // 片側にしか床板がない小梁として、正しい答え（1500）だけが返ることを確認する。
        model.slabs.push(rect_slab(0, [0, 1, 2, 3]));
        let w = joist_edge_tributary_width(&model, NodeId(0), NodeId(1));
        assert!(matches!(w, Some(x) if (x - 1500.0).abs() < 1e-6), "{w:?}");
    }
}
