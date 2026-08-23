//! 主架構が囲む閉領域（パネル）の検出。
//!
//! 床領域は「大梁で囲まれた領域ごとに 1 つ」と定める。本モジュールは、その閉領域を
//! 主架構のトポロジーから求める。壁領域（柱と梁が囲む構面内の閉領域）も同じ面走査で
//! 求められるが、構面の切り出し規則が未決のため、現時点では床（水平面）のみを扱う。
//!
//! # 生成規則
//!
//! 1. **対象**は水平な 2 節点の梁要素（[`ElementKind::Beam`]）のみ。両端の Z 差が
//!    [`crate::geom::LEVEL_TOL_MM`] 以内のものを水平とみなす。柱・ブレース・壁・二次部材は対象にしない
//!    （床領域の境界は大梁であるという定義による）。
//! 2. **レベルごと**に分けて面を求める。同じ Z（[`crate::geom::LEVEL_TOL_MM`] 以内）の梁を 1 つの
//!    平面グラフとして扱う。
//! 3. **面走査**は半辺（有向辺）をたどる定型手法による。各節点で接続辺を方位角順に並べ、
//!    半辺 `u→v` の次を「`v` のまわりで `v→u` の 1 つ手前（時計回りに次）」とすると、
//!    内部を左に見て一周する閉路が得られる。
//! 4. **外周面は符号付き面積が負**になるため、これを捨てる。面積の大小では判別しない
//!    （中庭のある建物では、面積最大の内部面が外周面より大きくなりうる）。
//! 5. **行き止まりの辺**（片持ち梁など、片端が他の梁と繋がらない梁）は、面走査が
//!    往復してその場に戻るため面を作らない。往復ぶんは外周面に吸収される。
//!
//! # 平面グラフであることの前提
//!
//! 面走査は、辺どうしが節点でのみ接することを前提とする。節点を共有せずに交差する梁が
//! あると、検出される区画は実際とずれるが、走査自体はエラーにならず黙って通る。
//! [`scan_floor_panels`] はその組を [`PanelScan::crossings`] として報告する
//! （検出は続行する。モデルの不備として利用者へ知らせるための情報である）。
//!
//! # 扱わないもの
//!
//! - **穴（中庭）を持つ面**。グラフが非連結で、ある閉路の内側に独立した閉路がある場合、
//!   外側の面は内側の存在を無視した多角形として返る。実建物では中庭のまわりに梁が
//!   通るため（＝連結する）、この形は稀である。
//! - **段差床**。レベルで分けるため、同じ階でもレベルが違えば別の平面グラフになる。
//!   段差部の梁は両方のレベルのどちらにも属さない（両端の Z が違うため水平とみなされない）。

use crate::geom::{vec3, LEVEL_TOL_MM};
use crate::ids::{ElemId, NodeId};
use crate::model::{ElementKind, Model};
use std::collections::HashMap;

/// 面走査の対象とする梁の最小長さ [mm]。これ未満は方位角が定まらないため除外する。
const MIN_EDGE_LEN_MM: f64 = 1.0;

/// 主架構が囲む閉領域（パネル）1 つ。
#[derive(Clone, Debug, PartialEq)]
pub struct Panel {
    /// 面のレベル Z [mm]（構成する節点の Z の平均）。
    pub level: f64,
    /// 境界の節点列（反時計回り。始点は繰り返さない）。
    pub boundary: Vec<NodeId>,
    /// 境界をなす梁要素（`boundary` の辺と同順。辺 i は `boundary[i]`→`boundary[i+1]`）。
    pub edges: Vec<ElemId>,
}

impl Panel {
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

    /// 点 `p`（XY）がこのパネルの内部にあるか。**辺上（[`BOUNDARY_TOL_MM`] 以内）は含めない。**
    ///
    /// 版や二次部材をパネルへ割り当てる用途を想定する。辺上の点は隣接パネルの双方に
    /// 該当してしまうため含めない（所属を一意に決められるようにする）。
    ///
    /// レイキャストは辺上の点の扱いが定まらない（辺の向きしだいで内側にも外側にもなる）ため、
    /// 辺までの距離による判定を先に行う。
    pub fn contains(&self, model: &Model, p: [f64; 2]) -> bool {
        let Some(poly) = self.polygon(model) else {
            return false;
        };
        let n = poly.len();
        if n < 3 {
            return false;
        }
        for i in 0..n {
            if point_segment_dist(p, poly[i], poly[(i + 1) % n]) <= BOUNDARY_TOL_MM {
                return false;
            }
        }
        let mut inside = false;
        let mut j = n - 1;
        for i in 0..n {
            let (a, b) = (poly[i], poly[j]);
            if (a[1] > p[1]) != (b[1] > p[1]) {
                let x = (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0];
                if p[0] < x {
                    inside = !inside;
                }
            }
            j = i;
        }
        inside
    }

    /// このパネルと同じレベルか（[`crate::geom::LEVEL_TOL_MM`] 以内）。
    pub fn is_same_level(&self, z: f64) -> bool {
        (self.level - z).abs() <= LEVEL_TOL_MM
    }
}

/// 辺上とみなす点から辺までの距離の上限 [mm]。
pub const BOUNDARY_TOL_MM: f64 = 1.0;

/// 点から線分までの距離 [mm]（XY 平面）。
fn point_segment_dist(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1];
    let t = if len2 <= f64::EPSILON {
        0.0
    } else {
        (((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / len2).clamp(0.0, 1.0)
    };
    let q = [a[0] + t * ab[0], a[1] + t * ab[1]];
    ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2)).sqrt()
}

/// 多角形の符号付き面積（反時計回りが正）。
fn signed_area(pts: &[[f64; 2]]) -> f64 {
    let n = pts.len();
    if n < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        sum += a[0] * b[1] - b[0] * a[1];
    }
    sum / 2.0
}

/// 水平な大梁の 1 区間（面走査の入力）。
struct Edge {
    a: NodeId,
    b: NodeId,
    elem: ElemId,
}

/// 面走査の結果。パネルに加え、平面グラフとして矛盾がある兆候を持ち帰る。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PanelScan {
    /// 検出したパネル。レベルの昇順、同一レベル内は面積の降順。
    pub panels: Vec<Panel>,
    /// 節点を共有せずに交差している水平梁の組。
    ///
    /// 面走査は平面グラフ（辺どうしが節点でのみ接する）を前提とする。交差する梁があると
    /// 検出されるパネルは実際の区画とずれるが、走査自体はエラーにならず黙って通る。
    /// **モデルの不備として利用者へ知らせるための情報であり、パネルの検出は続行する。**
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
pub fn scan_floor_panels(model: &Model) -> PanelScan {
    let mut scan = PanelScan::default();
    for (level, edges) in horizontal_beams_by_level(model) {
        scan.crossings.extend(crossing_pairs(model, &edges));
        let (panels, unclosed) = faces_of_level(model, level, &edges);
        scan.panels.extend(panels);
        scan.unclosed += unclosed;
    }
    scan
}

/// [`scan_floor_panels`] のパネルだけを取り出す薄いラッパ。
pub fn generate_floor_panels(model: &Model) -> Vec<Panel> {
    scan_floor_panels(model).panels
}

/// 節点を共有せずに交差している水平大梁の組を、レベルごとに集めて返す。
///
/// 面走査は平面グラフ（辺どうしが節点でのみ接する）を前提とするため、交差する梁があると
/// 検出される区画が実際とずれる。**モデルの不備として利用者へ知らせるための情報**であり、
/// パネルの検出自体は続行する（[`scan_floor_panels`] 参照）。
///
/// パネルを組まずに交差だけを知りたい場合（診断など）は、面走査を伴わないこちらを使う。
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
/// 面走査がその節点を分岐として扱えず、区画がつながってしまうためである。
fn crossing_pairs(model: &Model, edges: &[Edge]) -> Vec<(ElemId, ElemId)> {
    let coords = |n: NodeId| model.nodes.get(n.index()).map(|x| [x.coord[0], x.coord[1]]);
    let mut out = Vec::new();
    for (i, e1) in edges.iter().enumerate() {
        for e2 in edges.iter().skip(i + 1) {
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
    out
}

/// 2 線分が交わるか（端点どうしの共有は上位で除外済み）。
fn segments_touch(p1: [f64; 2], p2: [f64; 2], q1: [f64; 2], q2: [f64; 2]) -> bool {
    let d = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    let on = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        d(a, b, c).abs() <= AREA_EPS
            && c[0] >= a[0].min(b[0]) - LEVEL_TOL_MM
            && c[0] <= a[0].max(b[0]) + LEVEL_TOL_MM
            && c[1] >= a[1].min(b[1]) - LEVEL_TOL_MM
            && c[1] <= a[1].max(b[1]) + LEVEL_TOL_MM
    };
    let (d1, d2, d3, d4) = (d(p1, p2, q1), d(p1, p2, q2), d(q1, q2, p1), d(q1, q2, p2));
    if ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0)) {
        return true;
    }
    // 端点が相手の線分上に載る（節点を共有しない T 字・重なり）。
    on(p1, p2, q1) || on(p1, p2, q2) || on(q1, q2, p1) || on(q1, q2, p2)
}

/// 外積が 0 とみなせる上限 [mm²]。長さ 1mm の食い違いを許す幅とする。
const AREA_EPS: f64 = 1.0;

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
fn faces_of_level(model: &Model, level: f64, edges: &[Edge]) -> (Vec<Panel>, usize) {
    // 半辺（有向辺）の一覧。同じ節点対に複数の梁があっても、最初の 1 本だけを採る
    // （重複部材は面を増やさない）。
    let mut half: HashMap<(NodeId, NodeId), ElemId> = HashMap::new();
    for e in edges {
        if e.a == e.b {
            continue;
        }
        half.entry((e.a, e.b)).or_insert(e.elem);
        half.entry((e.b, e.a)).or_insert(e.elem);
    }

    // 各節点まわりの接続先を方位角の昇順に並べる。
    let mut around: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
    for &(from, to) in half.keys() {
        around.entry(from).or_default().push(to);
    }
    for (from, list) in around.iter_mut() {
        let Some(origin) = model.nodes.get(from.index()) else {
            continue;
        };
        list.sort_by(|x, y| {
            angle_at(model, origin.coord, *x).total_cmp(&angle_at(model, origin.coord, *y))
        });
        list.dedup();
    }

    let mut visited: HashMap<(NodeId, NodeId), bool> = HashMap::new();
    let mut panels = Vec::new();
    let mut unclosed = 0;
    let mut starts: Vec<(NodeId, NodeId)> = half.keys().copied().collect();
    // 走査順を安定させる（HashMap の反復順に依存しない）。
    starts.sort_by_key(|(a, b)| (a.0, b.0));

    for start in starts {
        if visited.contains_key(&start) {
            continue;
        }
        let mut boundary = Vec::new();
        let mut face_edges = Vec::new();
        let mut cur = start;
        let mut closed = false;
        // 閉路をたどる。半辺の総数を超えたら異常として打ち切る（無限ループ防止）。
        for _ in 0..=half.len() {
            if visited.insert(cur, true).is_some() {
                break;
            }
            boundary.push(cur.0);
            face_edges.push(half[&cur]);
            let Some(next) = next_half_edge(&around, cur) else {
                break;
            };
            cur = next;
            if cur == start {
                closed = true;
                break;
            }
        }
        if !closed {
            // 各半辺の後続は一意なので、ここへ来るのはグラフの構築に誤りがある場合だけ。
            unclosed += 1;
            continue;
        }
        if boundary.len() < 3 {
            continue;
        }
        let pts: Vec<[f64; 2]> = boundary
            .iter()
            .filter_map(|n| model.nodes.get(n.index()))
            .map(|n| [n.coord[0], n.coord[1]])
            .collect();
        if pts.len() != boundary.len() {
            continue;
        }
        // 外周面は符号付き面積が負になる。行き止まりの辺だけを往復した閉路は面積 0。
        if signed_area(&pts) <= 0.0 {
            continue;
        }
        panels.push(Panel {
            level,
            boundary,
            edges: face_edges,
        });
    }

    panels.sort_by(|a, b| b.area(model).total_cmp(&a.area(model)));
    (panels, unclosed)
}

/// `origin` から節点 `to` を見た方位角（-π..π）。節点が引けない場合は端に寄せる。
fn angle_at(model: &Model, origin: [f64; 3], to: NodeId) -> f64 {
    let Some(n) = model.nodes.get(to.index()) else {
        return f64::MAX;
    };
    (n.coord[1] - origin[1]).atan2(n.coord[0] - origin[0])
}

/// 半辺 `u→v` の次の半辺。`v` のまわりで `v→u` の 1 つ手前（時計回りに次）を選ぶと、
/// 内部を左に見て一周する閉路になる。
fn next_half_edge(
    around: &HashMap<NodeId, Vec<NodeId>>,
    (u, v): (NodeId, NodeId),
) -> Option<(NodeId, NodeId)> {
    let list = around.get(&v)?;
    let pos = list.iter().position(|&w| w == u)?;
    let prev = if pos == 0 { list.len() - 1 } else { pos - 1 };
    Some((v, list[prev]))
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

    /// 節点を格子状に並べたモデル。`nx`×`ny` 区画の梁格子を張る。
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
    fn test_single_panel() {
        let model = grid(1, 1, 4000.0, 0.0);
        let panels = generate_floor_panels(&model);
        assert_eq!(panels.len(), 1, "1 区画なら面は 1 つ");
        assert_eq!(panels[0].boundary.len(), 4);
        assert!((panels[0].area(&model) - 4000.0 * 4000.0).abs() < 1.0);
        assert!((panels[0].level - 0.0).abs() < 1e-9);
        assert_eq!(panels[0].edges.len(), 4, "境界の梁が辺と同数");
    }

    #[test]
    fn test_grid_panels() {
        let model = grid(3, 2, 3000.0, 4000.0);
        let panels = generate_floor_panels(&model);
        assert_eq!(panels.len(), 6, "3×2 の格子は 6 面");
        for p in &panels {
            assert!((p.area(&model) - 3000.0 * 3000.0).abs() < 1.0);
            assert!((p.level - 4000.0).abs() < 1e-9);
        }
    }

    /// 面の頂点列は反時計回り（符号付き面積が正）で返る。
    #[test]
    fn test_boundary_is_counter_clockwise() {
        let model = grid(1, 1, 4000.0, 0.0);
        let p = &generate_floor_panels(&model)[0];
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
    fn test_dangling_beam_makes_no_panel() {
        let mut model = grid(1, 1, 4000.0, 0.0);
        model.nodes.push(node(4, 6000.0, 0.0, 0.0));
        let eid = model.elements.len() as u32;
        model.elements.push(beam(eid, 1, 4)); // 節点 1 から外へ跳ね出す片持ち梁
        let panels = generate_floor_panels(&model);
        assert_eq!(panels.len(), 1, "片持ち梁は面を作らない");
        assert!((panels[0].area(&model) - 4000.0 * 4000.0).abs() < 1.0);
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
        let panels = generate_floor_panels(&model);
        assert_eq!(panels.len(), 2, "レベルごとに 1 面ずつ");
        assert!((panels[0].level - 0.0).abs() < 1e-9, "レベルの昇順で返る");
        assert!((panels[1].level - 4000.0).abs() < 1e-9);
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
        assert_eq!(generate_floor_panels(&model).len(), 1);
    }

    /// 節点を共有せずに交差する梁は、モデルの不備として報告する。
    #[test]
    fn test_crossing_beams_are_reported() {
        let mut model = grid(1, 1, 4000.0, 0.0);
        // 区画の内側を斜めに横切る 2 本。互いに交差し、外周とも節点を共有しない。
        model.nodes.push(node(4, 1000.0, 1000.0, 0.0));
        model.nodes.push(node(5, 3000.0, 3000.0, 0.0));
        model.nodes.push(node(6, 1000.0, 3000.0, 0.0));
        model.nodes.push(node(7, 3000.0, 1000.0, 0.0));
        let eid = model.elements.len() as u32;
        model.elements.push(beam(eid, 4, 5));
        model.elements.push(beam(eid + 1, 6, 7));

        let scan = scan_floor_panels(&model);
        assert_eq!(scan.crossings.len(), 1, "交差する 1 組を報告する");
        assert_eq!(scan.unclosed, 0, "走査自体は閉じる");
        // 交差があってもパネルの検出は続行する（外周の 1 面は取れる）。
        assert_eq!(scan.panels.len(), 1);
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

        let scan = scan_floor_panels(&model);
        assert_eq!(scan.crossings.len(), 1);
    }

    /// 節点を共有する組（十字に交わる格子）は交差として報告しない。
    #[test]
    fn test_shared_node_is_not_a_crossing() {
        let model = grid(2, 2, 3000.0, 0.0);
        let scan = scan_floor_panels(&model);
        assert!(scan.crossings.is_empty(), "{:?}", scan.crossings);
        assert_eq!(scan.panels.len(), 4);
        assert_eq!(scan.unclosed, 0);
    }

    /// パネルの内外判定（辺上は内部に含めない）。
    #[test]
    fn test_panel_contains() {
        let model = grid(1, 1, 4000.0, 0.0);
        let p = &generate_floor_panels(&model)[0];
        assert!(p.contains(&model, [2000.0, 2000.0]), "内部");
        assert!(!p.contains(&model, [5000.0, 2000.0]), "外部");
        assert!(!p.contains(&model, [2000.0, 0.0]), "辺上は含めない");
        assert!(p.is_same_level(0.0));
        assert!(!p.is_same_level(4000.0));
    }

    /// L 形（凹多角形）の面も 1 つの面として取れる。
    #[test]
    fn test_concave_panel() {
        // 2×2 の格子から 1 区画ぶんの梁を落として L 形の面を作る。
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
        let panels = generate_floor_panels(&model);
        assert_eq!(panels.len(), 1, "L 形でも面は 1 つ");
        // 面積 = 8000×4000 + 4000×4000 = 48,000,000 mm²
        assert!((panels[0].area(&model) - 48.0e6).abs() < 1.0);
        assert_eq!(panels[0].boundary.len(), 7);
    }
}
