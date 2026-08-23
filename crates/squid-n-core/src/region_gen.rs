//! 主架構が囲む閉領域（パネル）の検出。
//!
//! 床領域は「大梁で囲まれた領域ごとに 1 つ」と定める。本モジュールは、その閉領域を
//! 主架構のトポロジーから求める。壁領域（柱と梁が囲む構面内の閉領域）も同じ面走査で
//! 求められるが、構面の切り出し規則が未決のため、現時点では床（水平面）のみを扱う。
//!
//! # 生成規則
//!
//! 1. **対象**は水平な 2 節点の梁要素（[`ElementKind::Beam`]）のみ。両端の Z 差が
//!    [`LEVEL_TOL_MM`] 以内のものを水平とみなす。柱・ブレース・壁・二次部材は対象にしない
//!    （床領域の境界は大梁であるという定義による）。
//! 2. **レベルごと**に分けて面を求める。同じ Z（[`LEVEL_TOL_MM`] 以内）の梁を 1 つの
//!    平面グラフとして扱う。
//! 3. **面走査**は半辺（有向辺）をたどる定型手法による。各節点で接続辺を方位角順に並べ、
//!    半辺 `u→v` の次を「`v` のまわりで `v→u` の 1 つ手前（時計回りに次）」とすると、
//!    内部を左に見て一周する閉路が得られる。
//! 4. **外周面は符号付き面積が負**になるため、これを捨てる。面積の大小では判別しない
//!    （中庭のある建物では、面積最大の内部面が外周面より大きくなりうる）。
//! 5. **行き止まりの辺**（片持ち梁など、片端が他の梁と繋がらない梁）は、面走査が
//!    往復してその場に戻るため面を作らない。往復ぶんは外周面に吸収される。
//!
//! # 扱わないもの
//!
//! - **穴（中庭）を持つ面**。グラフが非連結で、ある閉路の内側に独立した閉路がある場合、
//!   外側の面は内側の存在を無視した多角形として返る。実建物では中庭のまわりに梁が
//!   通るため（＝連結する）、この形は稀である。
//! - **段差床**。レベルで分けるため、同じ階でもレベルが違えば別の平面グラフになる。
//!   段差部の梁は両方のレベルのどちらにも属さない（両端の Z が違うため水平とみなされない）。

use crate::geom::vec3;
use crate::ids::{ElemId, NodeId};
use crate::model::{ElementKind, Model};
use std::collections::HashMap;

/// 同じレベル・同じ高さとみなす Z の差 [mm]。
pub const LEVEL_TOL_MM: f64 = 1.0;

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
    /// 平面多角形の面積 [mm²]（シューレース公式）。
    pub fn area(&self, model: &Model) -> f64 {
        let pts: Vec<[f64; 2]> = self
            .boundary
            .iter()
            .filter_map(|n| model.nodes.get(n.index()))
            .map(|n| [n.coord[0], n.coord[1]])
            .collect();
        signed_area(&pts).abs()
    }
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

/// 主架構（水平な大梁）が囲む閉領域を、レベルごとに検出する。
///
/// モデルは変更しない。生成規則はモジュールドキュメントを参照。
/// 返り値はレベルの昇順、同一レベル内は面積の降順とする（呼び出しごとに順序が変わらない）。
pub fn generate_floor_panels(model: &Model) -> Vec<Panel> {
    let mut out = Vec::new();
    for (level, edges) in horizontal_beams_by_level(model) {
        out.extend(faces_of_level(model, level, &edges));
    }
    out
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

/// 1 レベルぶんの平面グラフから内部面を取り出す。
fn faces_of_level(model: &Model, level: f64, edges: &[Edge]) -> Vec<Panel> {
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
                break;
            }
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
    panels
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
