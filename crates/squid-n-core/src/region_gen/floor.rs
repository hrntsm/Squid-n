//! 主架構（水平な大梁）が囲む閉領域（床領域の境界）の検出。
//!
//! 床領域は「大梁で囲まれた領域ごとに 1 つ」と定める。本モジュールは、その閉領域を
//! 主架構のトポロジーから求める。壁領域（柱と梁が囲む鉛直構面内の閉領域）は
//! [`super::wall`] が同じ面走査エンジン（[`super::scan_faces`]）を使って求める。
//!
//! # 生成規則
//!
//! 1. **対象**は水平な 2 節点の梁要素（[`ElementKind::Beam`]）のみ。両端の Z 差が
//!    [`crate::geom::LEVEL_TOL_MM`] 以内のものを水平とみなす。柱・ブレース・壁・二次部材は対象にしない
//!    （床領域の境界は大梁であるという定義による）。
//! 2. **レベルごと**に分けて面を求める。同じ Z（[`crate::geom::LEVEL_TOL_MM`] 以内）の梁を 1 つの
//!    平面グラフとして扱う。
//! 3. **面走査**は半辺（有向辺）をたどる定型手法による（[`super::scan_faces`]）。
//! 4. **外周面は符号付き面積が負**になるため、これを捨てる。面積の大小では判別しない
//!    （中庭のある建物では、面積最大の内部面が外周面より大きくなりうる）。
//! 5. **行き止まりの辺**（片持ち梁など、片端が他の梁と繋がらない梁）は、面走査が
//!    往復してその場に戻るため面を作らない。往復ぶんは外周面に吸収される。
//!
//! # 平面グラフであることの前提
//!
//! 面走査は、辺どうしが節点でのみ接することを前提とする。節点を共有せずに交差する梁が
//! あると、検出される床領域は実際とずれるが、走査自体はエラーにならず黙って通る。
//! [`scan_region_boundaries`] はその組を [`RegionBoundaryScan::crossings`] として報告する
//! （検出は続行する。モデルの不備として利用者へ知らせるための情報である）。
//!
//! # 扱わないもの
//!
//! - **穴（中庭）を持つ面**。グラフが非連結で、ある閉路の内側に独立した閉路がある場合、
//!   外側の面は内側の存在を無視した多角形として返る。実建物では中庭のまわりに梁が
//!   通るため（＝連結する）、この形は稀である。
//! - **段差床**。レベルで分けるため、同じ階でもレベルが違えば別の平面グラフになる。
//!   段差部の梁は両方のレベルのどちらにも属さない（両端の Z が違うため水平とみなされない）。

use super::{point_segment_dist, polygon_contains_strict, scan_faces, signed_area, Edge};
use crate::geom::{vec3, LEVEL_TOL_MM, MEMBER_AXIS_TOL_MM};
use crate::ids::{ElemId, NodeId};
use crate::model::{ElementKind, Model};

/// 面走査の対象とする梁の最小長さ [mm]。これ未満は方位角が定まらないため除外する。
const MIN_EDGE_LEN_MM: f64 = 1.0;

/// 主架構が囲む閉領域（床領域の境界）1 つ。
#[derive(Clone, Debug, PartialEq)]
pub struct RegionBoundary {
    /// 面のレベル Z [mm]（構成する節点の Z の平均）。
    pub level: f64,
    /// 境界の節点列（反時計回り。始点は繰り返さない）。
    pub boundary: Vec<NodeId>,
    /// 境界をなす梁要素（`boundary` の辺と同順。辺 i は `boundary[i]`→`boundary[i+1]`）。
    pub edges: Vec<ElemId>,
}

impl RegionBoundary {
    /// 境界節点の XY 座標列。節点が引けない場合は `None`。
    fn polygon(&self, model: &Model) -> Option<Vec<[f64; 2]>> {
        self.boundary
            .iter()
            .map(|n| model.nodes.get(n.index()).map(|n| [n.coord[0], n.coord[1]]))
            .collect()
    }

    /// 平面多角形の面積 [mm²]（シューレース公式）。
    pub fn area(&self, model: &Model) -> f64 {
        self.polygon(model)
            .map(|pts| signed_area(&pts).abs())
            .unwrap_or(0.0)
    }

    /// 点 `p`（XY）がこの境界の内部にあるか。**辺上（[`super::BOUNDARY_TOL_MM`] 以内）は含めない。**
    ///
    /// 版や二次部材をこの境界へ割り当てる用途を想定する。辺上の点は隣接する境界の双方に
    /// 該当してしまうため含めない（所属を一意に決められるようにする）。
    ///
    /// レイキャストは辺上の点の扱いが定まらない（辺の向きしだいで内側にも外側にもなる）ため、
    /// 辺までの距離による判定を先に行う。
    pub fn contains(&self, model: &Model, p: [f64; 2]) -> bool {
        let Some(poly) = self.polygon(model) else {
            return false;
        };
        polygon_contains_strict(&poly, p)
    }

    /// この境界と同じレベルか（[`crate::geom::LEVEL_TOL_MM`] 以内）。
    pub fn is_same_level(&self, z: f64) -> bool {
        (self.level - z).abs() <= LEVEL_TOL_MM
    }
}

/// 面走査の結果。床領域の境界に加え、平面グラフとして矛盾がある兆候を持ち帰る。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RegionBoundaryScan {
    /// 検出した床領域の境界。レベルの昇順、同一レベル内は面積の降順。
    pub boundaries: Vec<RegionBoundary>,
    /// 節点を共有せずに交差している水平梁の組。
    ///
    /// 面走査は平面グラフ（辺どうしが節点でのみ接する）を前提とする。交差する梁があると
    /// 検出される境界は実際の床領域とずれるが、走査自体はエラーにならず黙って通る。
    /// **モデルの不備として利用者へ知らせるための情報であり、境界の検出は続行する。**
    pub crossings: Vec<(ElemId, ElemId)>,
    /// 閉じずに終わった面走査の数。
    ///
    /// 各半辺の後続は一意に定まるため、正しく組めていれば必ず 0 になる（不変条件の番人）。
    /// 0 でない場合は面走査の実装かグラフの構築に誤りがある。
    pub unclosed: usize,
}

/// 主架構（水平な大梁）が囲む閉領域を、レベルごとに検出する。
///
/// モデルは変更しない。生成規則はモジュールドキュメントを参照。
/// 平面グラフとして矛盾がある兆候（交差する梁など）も併せて返す。
pub fn scan_region_boundaries(model: &Model) -> RegionBoundaryScan {
    let mut scan = RegionBoundaryScan::default();
    for (level, edges) in horizontal_beams_by_level(model) {
        scan.crossings.extend(crossing_pairs(model, &edges));
        let (boundaries, unclosed) = faces_of_level(model, level, &edges);
        scan.boundaries.extend(boundaries);
        scan.unclosed += unclosed;
    }
    scan
}

/// [`scan_region_boundaries`] の境界だけを取り出す薄いラッパ。
pub fn generate_region_boundaries(model: &Model) -> Vec<RegionBoundary> {
    scan_region_boundaries(model).boundaries
}

/// 節点を共有せずに交差している水平大梁の組を、レベルごとに集めて返す。
///
/// 面走査は平面グラフ（辺どうしが節点でのみ接する）を前提とするため、交差する梁があると
/// 検出される床領域が実際とずれる。**モデルの不備として利用者へ知らせるための情報**であり、
/// 境界の検出自体は続行する（[`scan_region_boundaries`] 参照）。
///
/// 境界を組まずに交差だけを知りたい場合（診断など）は、面走査を伴わないこちらを使う。
pub fn crossing_beams(model: &Model) -> Vec<(ElemId, ElemId)> {
    let mut out = Vec::new();
    for (_, edges) in horizontal_beams_by_level(model) {
        out.extend(crossing_pairs(model, &edges));
    }
    out
}

/// 同一レベルの梁のうち、節点を共有せずに交差している組を返す。
///
/// 端点で接する（T 字・十字に節点を共有する）ものは交差としない。
/// 一方の端点が他方の内部に載る（節点を共有しない T 字）場合も交差として報告する。
/// 面走査がその節点を分岐として扱えず、床領域がつながってしまうためである。
fn crossing_pairs(model: &Model, edges: &[Edge]) -> Vec<(ElemId, ElemId)> {
    let coords = |n: NodeId| model.nodes.get(n.index()).map(|x| [x.coord[0], x.coord[1]]);

    // 総当たりは梁の本数の 2 乗になる（実測で 32,800 本・約 530ms）。準備計算と
    // 解析前チェックの両方から毎回呼ばれるため、まず境界矩形が重なる組だけに絞る。
    // X の下限で並べ、X 区間が離れた時点で内側の走査を打ち切る（走査線法）。
    struct Box {
        idx: usize,
        min: [f64; 2],
        max: [f64; 2],
    }
    let mut boxes: Vec<Box> = Vec::with_capacity(edges.len());
    for (idx, e) in edges.iter().enumerate() {
        let (Some(a), Some(b)) = (coords(e.a), coords(e.b)) else {
            continue;
        };
        boxes.push(Box {
            idx,
            min: [
                a[0].min(b[0]) - MEMBER_AXIS_TOL_MM,
                a[1].min(b[1]) - MEMBER_AXIS_TOL_MM,
            ],
            max: [
                a[0].max(b[0]) + MEMBER_AXIS_TOL_MM,
                a[1].max(b[1]) + MEMBER_AXIS_TOL_MM,
            ],
        });
    }
    boxes.sort_by(|x, y| x.min[0].total_cmp(&y.min[0]));

    let mut out = Vec::new();
    for (i, bi) in boxes.iter().enumerate() {
        for bj in boxes.iter().skip(i + 1) {
            if bj.min[0] > bi.max[0] {
                break; // 以降は X 区間が離れる（下限の昇順に並んでいる）。
            }
            if bj.min[1] > bi.max[1] || bi.min[1] > bj.max[1] {
                continue; // Y 区間が離れている。
            }
            let (e1, e2) = (&edges[bi.idx], &edges[bj.idx]);
            if e1.a == e2.a || e1.a == e2.b || e1.b == e2.a || e1.b == e2.b {
                continue; // 節点を共有する組は交差ではない。
            }
            let (Some(p1), Some(p2), Some(q1), Some(q2)) =
                (coords(e1.a), coords(e1.b), coords(e2.a), coords(e2.b))
            else {
                continue;
            };
            if segments_touch(p1, p2, q1, q2) {
                out.push((e1.elem, e2.elem));
            }
        }
    }
    out.sort_by_key(|(a, b)| (a.0, b.0));
    out
}

/// 2 線分が交わるか（端点どうしの共有は上位で除外済み）。
///
/// 一方の端点が相手の材軸上に載る（節点を共有しない T 字）判定には、
/// [`crate::geom::MEMBER_AXIS_TOL_MM`] を用いる。荷重を梁へ割り付ける側が拾う近さと
/// そろえるためで、外積のしきい値で見ると長い部材ほど厳しくなり、
/// 「荷重は載るのに診断には出ない」ずれが生じる。
fn segments_touch(p1: [f64; 2], p2: [f64; 2], q1: [f64; 2], q2: [f64; 2]) -> bool {
    let d = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    let (d1, d2, d3, d4) = (d(p1, p2, q1), d(p1, p2, q2), d(q1, q2, p1), d(q1, q2, p2));
    if ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0)) {
        return true;
    }
    // 端点が相手の線分上に載る（節点を共有しない T 字・重なり）。
    let on =
        |a: [f64; 2], b: [f64; 2], c: [f64; 2]| point_segment_dist(c, a, b) <= MEMBER_AXIS_TOL_MM;
    on(p1, p2, q1) || on(p1, p2, q2) || on(q1, q2, p1) || on(q1, q2, p2)
}

/// 水平な 2 節点梁を、レベルごとに集める。返り値はレベルの昇順。
fn horizontal_beams_by_level(model: &Model) -> Vec<(f64, Vec<Edge>)> {
    let mut buckets: Vec<(f64, Vec<Edge>)> = Vec::new();
    for e in &model.elements {
        if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
            continue;
        }
        let (Some(na), Some(nb)) = (
            model.nodes.get(e.nodes[0].index()),
            model.nodes.get(e.nodes[1].index()),
        ) else {
            continue;
        };
        if (na.coord[2] - nb.coord[2]).abs() > LEVEL_TOL_MM {
            continue; // 段差部の梁・傾斜梁は水平面に属さない。
        }
        if vec3::dist(na.coord, nb.coord) < MIN_EDGE_LEN_MM {
            continue;
        }
        let z = (na.coord[2] + nb.coord[2]) / 2.0;
        let edge = Edge {
            a: e.nodes[0],
            b: e.nodes[1],
            elem: e.id,
        };
        match buckets
            .iter_mut()
            .find(|(level, _)| (*level - z).abs() <= LEVEL_TOL_MM)
        {
            Some((_, list)) => list.push(edge),
            None => buckets.push((z, vec![edge])),
        }
    }
    buckets.sort_by(|a, b| a.0.total_cmp(&b.0));
    buckets
}

/// 1 レベルぶんの平面グラフから内部面を取り出す。返り値は（面, 閉じなかった走査の数）。
fn faces_of_level(model: &Model, level: f64, edges: &[Edge]) -> (Vec<RegionBoundary>, usize) {
    let proj = |n: NodeId| {
        model
            .nodes
            .get(n.index())
            .map(|nd| [nd.coord[0], nd.coord[1]])
    };
    let (faces, unclosed) = scan_faces(edges, proj);

    // 外周面は符号付き面積が負になる。行き止まりの辺だけを往復した閉路は面積 0。
    let mut boundaries: Vec<RegionBoundary> = faces
        .into_iter()
        .filter(|f| f.signed_area > 0.0)
        .map(|f| RegionBoundary {
            level,
            boundary: f.boundary,
            edges: f.edges,
        })
        .collect();

    boundaries.sort_by(|a, b| b.area(model).total_cmp(&a.area(model)));
    (boundaries, unclosed)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{ElementData, EndCondition, ForceRegime, LocalAxis, Node};

    fn node(id: u32, x: f64, y: f64, z: f64) -> Node {
        Node {
            id: NodeId(id),
            coord: [x, y, z],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    fn beam(id: u32, i: u32, j: u32) -> ElementData {
        ElementData {
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
        }
    }

    /// 節点を格子状に並べたモデル。`nx`×`ny` 個の格子区画の梁格子を張る。
    fn grid(nx: usize, ny: usize, pitch: f64, z: f64) -> Model {
        let mut model = Model::default();
        let idx = |ix: usize, iy: usize| (iy * (nx + 1) + ix) as u32;
        for iy in 0..=ny {
            for ix in 0..=nx {
                model
                    .nodes
                    .push(node(idx(ix, iy), ix as f64 * pitch, iy as f64 * pitch, z));
            }
        }
        let mut eid = 0;
        for iy in 0..=ny {
            for ix in 0..nx {
                model.elements.push(beam(eid, idx(ix, iy), idx(ix + 1, iy)));
                eid += 1;
            }
        }
        for ix in 0..=nx {
            for iy in 0..ny {
                model.elements.push(beam(eid, idx(ix, iy), idx(ix, iy + 1)));
                eid += 1;
            }
        }
        model
    }

    #[test]
    fn test_single_boundary() {
        let model = grid(1, 1, 4000.0, 0.0);
        let boundaries = generate_region_boundaries(&model);
        assert_eq!(boundaries.len(), 1, "1 床領域なら面は 1 つ");
        assert_eq!(boundaries[0].boundary.len(), 4);
        assert!((boundaries[0].area(&model) - 4000.0 * 4000.0).abs() < 1.0);
        assert!((boundaries[0].level - 0.0).abs() < 1e-9);
        assert_eq!(boundaries[0].edges.len(), 4, "境界の梁が辺と同数");
    }

    #[test]
    fn test_grid_boundaries() {
        let model = grid(3, 2, 3000.0, 4000.0);
        let boundaries = generate_region_boundaries(&model);
        assert_eq!(boundaries.len(), 6, "3×2 の格子は 6 面");
        for p in &boundaries {
            assert!((p.area(&model) - 3000.0 * 3000.0).abs() < 1.0);
            assert!((p.level - 4000.0).abs() < 1e-9);
        }
    }

    /// 面の頂点列は反時計回り（符号付き面積が正）で返る。
    #[test]
    fn test_boundary_is_counter_clockwise() {
        let model = grid(1, 1, 4000.0, 0.0);
        let p = &generate_region_boundaries(&model)[0];
        let pts: Vec<[f64; 2]> = p
            .boundary
            .iter()
            .map(|n| {
                let c = model.nodes[n.index()].coord;
                [c[0], c[1]]
            })
            .collect();
        assert!(signed_area(&pts) > 0.0, "反時計回りで返る");
    }

    /// 片持ち梁（行き止まりの辺）は面を作らず、囲まれた面の数も変えない。
    #[test]
    fn test_dangling_beam_makes_no_boundary() {
        let mut model = grid(1, 1, 4000.0, 0.0);
        model.nodes.push(node(4, 6000.0, 0.0, 0.0));
        let eid = model.elements.len() as u32;
        model.elements.push(beam(eid, 1, 4)); // 節点 1 から外へ跳ね出す片持ち梁
        let boundaries = generate_region_boundaries(&model);
        assert_eq!(boundaries.len(), 1, "片持ち梁は面を作らない");
        assert!((boundaries[0].area(&model) - 4000.0 * 4000.0).abs() < 1.0);
    }

    /// レベルが違う梁は別の平面グラフとして扱う。
    #[test]
    fn test_levels_are_separated() {
        let mut model = grid(1, 1, 4000.0, 0.0);
        let base_nodes = model.nodes.len() as u32;
        let base_elems = model.elements.len() as u32;
        let upper = grid(1, 1, 4000.0, 4000.0);
        for (k, n) in upper.nodes.iter().enumerate() {
            model.nodes.push(node(
                base_nodes + k as u32,
                n.coord[0],
                n.coord[1],
                n.coord[2],
            ));
        }
        for (k, e) in upper.elements.iter().enumerate() {
            model.elements.push(beam(
                base_elems + k as u32,
                base_nodes + e.nodes[0].0,
                base_nodes + e.nodes[1].0,
            ));
        }
        let boundaries = generate_region_boundaries(&model);
        assert_eq!(boundaries.len(), 2, "レベルごとに 1 面ずつ");
        assert!(
            (boundaries[0].level - 0.0).abs() < 1e-9,
            "レベルの昇順で返る"
        );
        assert!((boundaries[1].level - 4000.0).abs() < 1e-9);
    }

    /// 柱・ブレース・傾斜梁は境界に使わない。
    #[test]
    fn test_only_horizontal_beams_are_used() {
        let mut model = grid(1, 1, 4000.0, 0.0);
        // 節点 0 から立ち上がる柱。
        model.nodes.push(node(4, 0.0, 0.0, 4000.0));
        let eid = model.elements.len() as u32;
        model.elements.push(beam(eid, 0, 4));
        // 傾斜梁（両端の Z が違う）。
        model.nodes.push(node(5, 4000.0, 0.0, 2000.0));
        model.elements.push(beam(eid + 1, 1, 5));
        assert_eq!(generate_region_boundaries(&model).len(), 1);
    }

    /// 節点を共有せずに交差する梁は、モデルの不備として報告する。
    #[test]
    fn test_crossing_beams_are_reported() {
        let mut model = grid(1, 1, 4000.0, 0.0);
        // 床領域の内側を斜めに横切る 2 本。互いに交差し、外周とも節点を共有しない。
        model.nodes.push(node(4, 1000.0, 1000.0, 0.0));
        model.nodes.push(node(5, 3000.0, 3000.0, 0.0));
        model.nodes.push(node(6, 1000.0, 3000.0, 0.0));
        model.nodes.push(node(7, 3000.0, 1000.0, 0.0));
        let eid = model.elements.len() as u32;
        model.elements.push(beam(eid, 4, 5));
        model.elements.push(beam(eid + 1, 6, 7));

        let scan = scan_region_boundaries(&model);
        assert_eq!(scan.crossings.len(), 1, "交差する 1 組を報告する");
        assert_eq!(scan.unclosed, 0, "走査自体は閉じる");
        // 交差があっても境界の検出は続行する（外周の 1 面は取れる）。
        assert_eq!(scan.boundaries.len(), 1);
    }

    /// 節点を共有せずに一方の端点が他方の途中へ載る梁（T 字）も報告する。
    #[test]
    fn test_touching_beam_without_shared_node_is_reported() {
        let mut model = grid(1, 1, 4000.0, 0.0);
        // 辺 0-1（y=0）の中間へ、節点を共有せずに突き当たる梁。
        model.nodes.push(node(4, 2000.0, 0.0, 0.0));
        model.nodes.push(node(5, 2000.0, 2000.0, 0.0));
        let eid = model.elements.len() as u32;
        model.elements.push(beam(eid, 4, 5));

        let scan = scan_region_boundaries(&model);
        assert_eq!(scan.crossings.len(), 1);
    }

    /// 節点を共有する組（十字に交わる格子）は交差として報告しない。
    #[test]
    fn test_shared_node_is_not_a_crossing() {
        let model = grid(2, 2, 3000.0, 0.0);
        let scan = scan_region_boundaries(&model);
        assert!(scan.crossings.is_empty(), "{:?}", scan.crossings);
        assert_eq!(scan.boundaries.len(), 4);
        assert_eq!(scan.unclosed, 0);
    }

    /// 材軸からわずかにずれて突き当たる梁も、交差として報告する。
    ///
    /// 荷重の割り付けは `MEMBER_AXIS_TOL_MM` 以内のずれを「梁に載っている」と扱うため、
    /// 診断も同じ近さで知らせないと「荷重は載るのに診断には出ない」状態になる。
    #[test]
    fn test_near_collinear_touch_is_reported() {
        let mut model = grid(1, 1, 4000.0, 0.0);
        // 辺 0-1（y=0）の中間へ、5mm 手前で止まる梁（節点は共有しない）。
        model.nodes.push(node(4, 2000.0, 5.0, 0.0));
        model.nodes.push(node(5, 2000.0, 2000.0, 0.0));
        let eid = model.elements.len() as u32;
        model.elements.push(beam(eid, 4, 5));
        assert_eq!(
            scan_region_boundaries(&model).crossings.len(),
            1,
            "5mm のずれは交差として報告する"
        );

        // 100mm 離れていれば、突き当たっているとはみなさない。
        model.nodes[4].coord[1] = 100.0;
        assert!(scan_region_boundaries(&model).crossings.is_empty());
    }

    /// 境界の内外判定（辺上は内部に含めない）。
    #[test]
    fn test_region_boundary_contains() {
        let model = grid(1, 1, 4000.0, 0.0);
        let p = &generate_region_boundaries(&model)[0];
        assert!(p.contains(&model, [2000.0, 2000.0]), "内部");
        assert!(!p.contains(&model, [5000.0, 2000.0]), "外部");
        assert!(!p.contains(&model, [2000.0, 0.0]), "辺上は含めない");
        assert!(p.is_same_level(0.0));
        assert!(!p.is_same_level(4000.0));
    }

    /// L 形（凹多角形）の面も 1 つの面として取れる。
    #[test]
    fn test_concave_boundary() {
        // 2×2 の格子から 1 床領域ぶんの梁を落として L 形の面を作る。
        let mut model = Model::default();
        let pts = [
            (0.0, 0.0),
            (4000.0, 0.0),
            (8000.0, 0.0),
            (0.0, 4000.0),
            (4000.0, 4000.0),
            (8000.0, 4000.0),
            (0.0, 8000.0),
            (4000.0, 8000.0),
        ];
        for (i, (x, y)) in pts.iter().enumerate() {
            model.nodes.push(node(i as u32, *x, *y, 0.0));
        }
        // 外周: 0-1-2-5-4-7-6-0（L 形）
        let ring = [(0, 1), (1, 2), (2, 5), (5, 4), (4, 7), (7, 6), (6, 0)];
        for (k, (i, j)) in ring.iter().enumerate() {
            model.elements.push(beam(k as u32, *i, *j));
        }
        let boundaries = generate_region_boundaries(&model);
        assert_eq!(boundaries.len(), 1, "L 形でも面は 1 つ");
        // 面積 = 8000×4000 + 4000×4000 = 48,000,000 mm²
        assert!((boundaries[0].area(&model) - 48.0e6).abs() < 1.0);
        assert_eq!(boundaries[0].boundary.len(), 7);
    }
}
