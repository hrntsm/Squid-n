//! 主架構が囲む閉領域（床領域・壁領域の境界）の検出。
//!
//! 床領域は「大梁で囲まれた領域ごとに 1 つ」、壁領域は「柱と梁が囲む鉛直構面内の
//! 閉領域ごとに 1 つ」と定める。本モジュールは、その閉領域を主架構のトポロジーから
//! 求める半辺（有向辺）面走査エンジン（[`scan_faces`]）を共有し、床（[`floor`]）と
//! 壁（[`wall`]）がそれぞれ「対象部材の絞り込み」と「2 次元への射影」を用意して
//! 同じ走査に載せる。
//!
//! 設計の経緯は `dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` を参照。
//!
//! # 面走査エンジン（[`scan_faces`]）
//!
//! 1. **半辺**（有向辺）の一覧を作る。同じ節点対に複数の部材があっても、最初の 1 本だけを採る
//!    （重複部材は面を増やさない）。
//! 2. 各節点まわりの接続先を、`proj`（節点 → 局所 2D 座標）が返す座標の方位角の昇順に並べる。
//! 3. 半辺 `u→v` の次を「`v` のまわりで `v→u` の 1 つ手前（時計回りに次）」とすると、
//!    内部を左に見て一周する閉路が得られる（定型の面走査）。
//! 4. 得られた閉路すべてを、符号付き面積つきで返す。**どれが外周面かの判別は呼び出し側が行う。**
//!    床は「符号付き面積が負」で判別できる（グローバル XY・+Z を上とする固定の向きで
//!    常に判定できるため）。壁は局所座標 `(s, z)` の水平方向 `s` が構面ごとに任意に決まり、
//!    床と同じ絶対符号の規則を機械的に流用できるとは限らないため、[`wall`] が自身で
//!    判別方法を持つ（[`wall`] のモジュールドキュメント参照）。
//!
//! `proj` が `None` を返す節点（節点が引けない陳腐化した参照）を含む閉路は捨てる。

use crate::ids::{ElemId, NodeId};
use std::collections::HashMap;

pub mod floor;
pub mod wall;

pub use floor::{
    crossing_beams, generate_region_boundaries, scan_region_boundaries, RegionBoundary,
    RegionBoundaryScan,
};
pub use wall::{
    generate_wall_region_boundaries, scan_wall_region_boundaries, WallRegionBoundary,
    WallRegionBoundaryScan,
};

/// 辺上とみなす点から辺までの距離の上限 [mm]。
pub const BOUNDARY_TOL_MM: f64 = 1.0;

/// 点 `p`（2D）が多角形の内部にあるか。**辺上（[`BOUNDARY_TOL_MM`] 以内）は含めない。**
///
/// 版や二次部材をこの境界へ割り当てる用途を想定する。辺上の点は隣接する境界の双方に
/// 該当してしまうため含めない（所属を一意に決められるようにする）。
///
/// レイキャストは辺上の点の扱いが定まらない（辺の向きしだいで内側にも外側にもなる）ため、
/// 辺までの距離による判定を先に行う。座標系は問わない（床は XY、壁は局所 `(s, z)` で使う）。
pub fn polygon_contains_strict(poly: &[[f64; 2]], p: [f64; 2]) -> bool {
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

/// 面走査の入力となる 1 本の部材（無向の辺。半辺は本モジュールが内部で作る）。
pub(crate) struct Edge {
    pub a: NodeId,
    pub b: NodeId,
    pub elem: ElemId,
}

/// 面走査で見つかった 1 つの閉路。
pub(crate) struct Face {
    pub boundary: Vec<NodeId>,
    pub edges: Vec<ElemId>,
    /// `proj` が返す 2D 座標系での符号付き面積（シューレース公式）。
    /// 外周面の判別は呼び出し側が行う（モジュールドキュメント参照）。
    pub signed_area: f64,
}

/// 半辺（有向辺）をたどる面走査エンジン。モジュールドキュメント参照。
///
/// 戻り値は `(検出した閉路の一覧, 閉じなかった走査の数)`。後者は各半辺の後続が
/// 一意に定まる不変条件の番人であり、0 でなければ呼び出し側のグラフ構築に誤りがある。
/// 閉路の順序は開始半辺の安定順（`(NodeId, NodeId)` の辞書順）。
pub(crate) fn scan_faces<P>(edges: &[Edge], proj: P) -> (Vec<Face>, usize)
where
    P: Fn(NodeId) -> Option<[f64; 2]>,
{
    // 半辺（有向辺）の一覧。同じ節点対に複数の部材があっても、最初の 1 本だけを採る
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
        let Some(origin) = proj(*from) else {
            continue;
        };
        list.sort_by(|x, y| angle_at(&proj, origin, *x).total_cmp(&angle_at(&proj, origin, *y)));
        list.dedup();
    }

    let mut visited: HashMap<(NodeId, NodeId), bool> = HashMap::new();
    let mut faces = Vec::new();
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
        let pts: Vec<[f64; 2]> = boundary.iter().filter_map(|&n| proj(n)).collect();
        if pts.len() != boundary.len() {
            continue;
        }
        faces.push(Face {
            signed_area: signed_area(&pts),
            boundary,
            edges: face_edges,
        });
    }

    (faces, unclosed)
}

/// `origin` から節点 `to` を `proj` で見た方位角（-π..π）。射影できない場合は端に寄せる。
fn angle_at<P>(proj: &P, origin: [f64; 2], to: NodeId) -> f64
where
    P: Fn(NodeId) -> Option<[f64; 2]>,
{
    match proj(to) {
        Some(c) => (c[1] - origin[1]).atan2(c[0] - origin[0]),
        None => f64::MAX,
    }
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

/// 多角形の XY 投影面積（絶対値。周りの向きに依らない）。
///
/// 床板の面積を「床の分配と同じ XY 投影」で測るための入口
/// （[`crate::model::Model::self_standing_wall_coverage`]）。
pub fn signed_area_abs(pts: &[[f64; 2]]) -> f64 {
    signed_area(pts).abs()
}

/// 多角形の符号付き面積（反時計回りが正）。
pub(crate) fn signed_area(pts: &[[f64; 2]]) -> f64 {
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

/// 点から線分までの距離 [mm]（2D 平面。座標系は問わない）。
pub(crate) fn point_segment_dist(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
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
