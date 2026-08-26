//! 柱・梁が囲む鉛直構面内の閉領域（壁領域の境界）の検出。
//!
//! 壁領域は「柱と梁が囲む鉛直構面内の閉領域ごとに 1 つ」と定める（D1）。床の面走査
//! （[`super::floor`]）は「同じレベル Z」で対象を絞り込んでから面走査にかけるが、
//! 壁は「同じ鉛直構面（＝ XY 平面上の同一直線の真上）」で絞り込む必要があり、
//! この絞り込み自体が新規の幾何判定になる。通り芯（`AxisGroup`。表示専用のデータで
//! 構造計算には一切使わない）には依存しない。直交グリッドか斜めかを問わず同じロジックで
//! 扱う（`dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` §3.2 E1）。
//!
//! # 生成規則
//!
//! 1. **鉛直構面の候補**は、全柱（`ElementKind::Beam` のうち
//!    [`crate::geom::is_vertical_pair`] を満たすもの）の柱脚位置（XY）から作る。
//!    柱位置を [`crate::geom::MEMBER_AXIS_TOL_MM`] 以内で重複排除したうえで全ペアから
//!    直線を作り、既に見つかっている直線と同一とみなせるものは統合する。
//!
//!    **同一直線とみなす判定は、角度の許容差ではなく、柱位置から直線までの実距離 [mm]
//!    で行う。** 角度の許容差は直線が長いほど遠方での位置ずれが拡大するため使わない
//!    （梁の交差診断を同じ理由で角度の外積しきい値から `MEMBER_AXIS_TOL_MM` へ
//!    統一した経緯（§8.3）と同じ判断）。例えば 0.5° の角度許容差は 50m 先で 436mm の
//!    ずれになり、10mm の許容差としては粗すぎる。
//! 2. **各直線に乗る部材**は、両端の XY 射影が直線から `MEMBER_AXIS_TOL_MM` 以内にある
//!    `ElementKind::Beam` の要素（柱・梁の区別、鉛直・水平・斜めの区別は問わない）。
//!    ブレース・壁要素・二次部材・パネルゾーンは対象にしない。
//! 3. **面走査**は直線ごとに、局所座標 `(s, z)`（s=直線に沿った位置、z=グローバル Z）へ
//!    射影して行う（[`super::scan_faces`]、床と共通のエンジン）。1 本の直線は建物の
//!    全高さをまとめて 1 つの平面グラフとして扱うため、複数階にまたがる壁面は
//!    階ごとの内部面として現れる。
//!
//! # 非平面な壁面は複数の境界へ自然に分解される
//!
//! 壁領域の境界検出は常に厳密な平面（1 つの直線の真上）を対象とする。芯ずれ・
//! 折れ曲がり・傾きがある壁面は、1 つの [`WallRegionBoundary`] にはならず、
//! 平面ごとに複数の境界へ自然に分解される。**この前提は緩めない**
//! （複数面の節点をまとめて 1 つの境界にする近道を取らないこと。§3.2 E2）。
//!
//! # 面積・自重の計算について
//!
//! [`WallRegionBoundary::area`] は、実座標からニューエルの公式で求めた 3 次元面積である。
//! トポロジー（面走査・外周面の判別）は直線への射影という近似で行ってよいが
//! （`MEMBER_AXIS_TOL_MM` 以内の芯ずれ・非平面性は面走査の結果を左右しない）、
//! 面積・荷重は理想平面へ投影せず実座標から直接求める（§3.2 E3）。芯ずれが
//! `MEMBER_AXIS_TOL_MM` 相当あると、投影面積は自重計算で無視できない誤差になりうるため。
//!
//! # 計算量について
//!
//! 柱脚位置の全ペアから直線候補を作るため、柱の本数を N とすると候補数は O(N²) になる。
//! 実建物の柱本数（数百〜数千本程度）では実用上問題にならないが、極端に柱本数が多い
//! モデルでは遅くなりうる。梁の交差診断（[`super::floor::crossing_beams`]）も当初は
//! 総当たりで実装し、実測して初めて走査線法へ最適化した経緯があり（§8.3）、
//! 本モジュールも同じ方針（先に正しさを固め、実測してから最適化する）を踏襲する。
//!
//! # 未結線
//!
//! 本モジュールは幾何の検出のみを提供する。`WallRegion`/`WallPlate` への統合、
//! モデルとの結線（要素生成・交差診断の接続）は別の作業（Step 7+8）で行う
//! （`dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` §9）。

use super::{scan_faces, Edge};
use crate::geom::{is_vertical_pair, MEMBER_AXIS_TOL_MM};
use crate::ids::{ElemId, NodeId};
use crate::model::{ElementKind, Model};
use std::collections::HashMap;

/// 局所座標 `(s, z)` での部材の最小長さ [mm]。3 次元長さではなく射影後の長さで判定する
/// （構面にほぼ垂直な短い部材が射影で長さ 0 に潰れ、方位角の並べ替えを乱すのを防ぐ）。
const MIN_PROJECTED_EDGE_LEN_MM: f64 = 1.0;

/// 柱・梁が囲む鉛直構面内の閉領域（壁領域の境界）1 つ。
#[derive(Clone, Debug, PartialEq)]
pub struct WallRegionBoundary {
    /// この境界が乗る構面（直線）の基準点（柱脚位置の XY 平面上）。
    pub plane_origin: [f64; 2],
    /// 構面の方向単位ベクトル（XY 平面上）。見出し角は `[0, π)` に正規化済み。
    pub plane_direction: [f64; 2],
    /// 境界の節点列（局所座標 `(s, z)` で反時計回り。始点は繰り返さない）。
    pub boundary: Vec<NodeId>,
    /// 境界をなす部材（`boundary` の辺と同順。辺 i は `boundary[i]`→`boundary[i+1]`）。
    pub edges: Vec<ElemId>,
}

impl WallRegionBoundary {
    /// 境界節点の実座標列 [mm]（3 次元）。節点が引けない場合は `None`。
    fn coords(&self, model: &Model) -> Option<Vec<[f64; 3]>> {
        self.boundary
            .iter()
            .map(|n| model.nodes.get(n.index()).map(|n| n.coord))
            .collect()
    }

    /// 面積 [mm²]（ニューエルの公式による 3 次元面積。理想平面への投影を経由しない。
    /// モジュールドキュメント参照）。節点が引けない場合は 0。
    pub fn area(&self, model: &Model) -> f64 {
        self.coords(model)
            .map(|pts| newell_area(&pts))
            .unwrap_or(0.0)
    }
}

/// 面走査の結果。壁領域の境界を持つ。
///
/// [`super::floor::RegionBoundaryScan::crossings`] に相当する交差診断はまだ持たない
/// （未着手。`dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` §9 残課題）。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct WallRegionBoundaryScan {
    /// 検出した壁領域の境界。構面（直線）ごと、同一構面内は面積の降順。
    pub boundaries: Vec<WallRegionBoundary>,
    /// 閉じずに終わった面走査の数（[`super::floor::RegionBoundaryScan::unclosed`] と同じ意味）。
    pub unclosed: usize,
}

/// 柱・梁（`ElementKind::Beam`）が囲む鉛直構面内の閉領域を検出する。
///
/// モデルは変更しない。生成規則はモジュールドキュメントを参照。
pub fn scan_wall_region_boundaries(model: &Model) -> WallRegionBoundaryScan {
    let mut scan = WallRegionBoundaryScan::default();
    for (origin, direction) in wall_planes(model) {
        let edges = members_on_plane(model, origin, direction);
        let proj = |n: NodeId| -> Option<[f64; 2]> {
            model
                .nodes
                .get(n.index())
                .map(|nd| project(origin, direction, nd.coord))
        };
        let (faces, unclosed) = scan_faces(&edges, proj);
        scan.unclosed += unclosed;

        let mut boundaries: Vec<WallRegionBoundary> = faces
            .into_iter()
            .filter(|f| f.signed_area > 0.0)
            .map(|f| WallRegionBoundary {
                plane_origin: origin,
                plane_direction: direction,
                boundary: f.boundary,
                edges: f.edges,
            })
            .collect();
        boundaries.sort_by(|a, b| b.area(model).total_cmp(&a.area(model)));
        scan.boundaries.extend(boundaries);
    }
    scan
}

/// [`scan_wall_region_boundaries`] の境界だけを取り出す薄いラッパ。
pub fn generate_wall_region_boundaries(model: &Model) -> Vec<WallRegionBoundary> {
    scan_wall_region_boundaries(model).boundaries
}

/// 柱脚位置（XY）から鉛直構面の候補直線を検出する。戻り値は `(origin, direction)`。
/// `direction` は見出し角 `[0, π)` に正規化した単位ベクトル。
///
/// **候補ペアの重複判定にはグリッド索引（[`LineIndex`]）を使う。** 候補ペアは
/// 柱本数 N に対して O(N²) 生じ、格子状の柱配置では格子点を結ぶ斜め方向の候補が
/// 大量に残るため、見つかっている直線の本数 L も N の増加とともに増える。
/// 素朴に「既存の全直線と実距離で比較する」実装（O(N²·L)）は、実測で
/// 900 本の柱を持つ 30×30 格子で 19 秒を超えた（`crates/squid-n-core/tests/perf_probe.rs`）。
/// グリッド索引は候補を粗く絞り込む役割のみを持ち、**最終的な同一直線の判定は
/// 必ず [`is_same_line`]（実距離）で行う**ため、索引の取りこぼし・衝突は
/// 正しさを損なわない（性能上のヒントにすぎない）。
fn wall_planes(model: &Model) -> Vec<([f64; 2], [f64; 2])> {
    let footprints = column_footprints(model);
    if footprints.len() < 2 {
        return Vec::new();
    }
    let mut lines: Vec<([f64; 2], [f64; 2])> = Vec::new();
    let mut index = LineIndex::new(&footprints);
    for i in 0..footprints.len() {
        for j in (i + 1)..footprints.len() {
            let (p, q) = (footprints[i], footprints[j]);
            let Some(direction) = canonical_direction(p, q) else {
                continue; // 重複点（距離ほぼ 0）は方向が定まらない。
            };
            let found = index
                .nearby(p, direction)
                .into_iter()
                .find(|&idx| is_same_line(lines[idx].0, lines[idx].1, &[p, q]));
            if found.is_some() {
                continue;
            }
            let idx = lines.len();
            lines.push((p, direction));
            index.insert(idx, p, direction);
        }
    }
    lines
}

/// [`wall_planes`] の候補直線をグリッドで粗く索引する構造体。
///
/// キーは `(2θ の単位円上の位置, 基準点からの符号なし距離)` を [`MEMBER_AXIS_TOL_MM`]
/// 相当の刻みで量子化したもの。`2θ`（見出し角の 2 倍）を使うのは、直線の向きには
/// ±180° の符号の曖昧さがあり、これを素朴な角度（mod π）でバケット化すると 0°/180°
/// の境界で不連続になる（179.99° と 0.01° はほぼ同じ直線だが、実数直線上のバケットでは
/// 両端に離れて割り当たる）ためである。角度を 2 倍すると、この符号の曖昧さがバケット
/// キーの計算自体に現れなくなり（`cos 2(θ+π) = cos 2θ`）、単位円上の連続な埋め込みとして
/// 素直にグリッド量子化できる。オフセット（[`point_to_line_dist`]）はもとから符号を
/// 持たないため、同様の問題は生じない。
struct LineIndex {
    /// 直線の代表点からの距離を測る基準点（柱脚位置の重心）。
    reference: [f64; 2],
    /// 見出し角 `θ` の量子化刻み（`2θ` 側は `2 * ang_res`）。
    ang_res: f64,
    buckets: HashMap<(i64, i64, i64), Vec<usize>>,
}

impl LineIndex {
    fn new(footprints: &[[f64; 2]]) -> Self {
        let reference = centroid(footprints);
        let r_max = footprints
            .iter()
            .map(|&p| dist2(p, reference).sqrt())
            .fold(0.0_f64, f64::max)
            .max(MEMBER_AXIS_TOL_MM);
        // 基準点から最も遠い柱脚位置でも、角度の量子化による位置ずれが
        // MEMBER_AXIS_TOL_MM 以下になるようにする。
        let ang_res = MEMBER_AXIS_TOL_MM / r_max;
        LineIndex {
            reference,
            ang_res,
            buckets: HashMap::new(),
        }
    }

    fn key(&self, line_point: [f64; 2], direction: [f64; 2]) -> (i64, i64, i64) {
        let theta = direction[1].atan2(direction[0]);
        let two_theta = 2.0 * theta;
        let cell = (2.0 * self.ang_res).max(1e-12);
        let bc = (two_theta.cos() / cell).floor() as i64;
        let bs = (two_theta.sin() / cell).floor() as i64;
        let offset = point_to_line_dist(line_point, direction, self.reference);
        let bo = (offset / MEMBER_AXIS_TOL_MM).floor() as i64;
        (bc, bs, bo)
    }

    /// このキーの近傍バケット（角度・オフセットとも隣接セルを含む）にある候補の
    /// 直線インデックス一覧。近傍を含めるのは、量子化の境界をまたぐ場合を拾うため。
    fn nearby(&self, line_point: [f64; 2], direction: [f64; 2]) -> Vec<usize> {
        let (bc, bs, bo) = self.key(line_point, direction);
        let mut out = Vec::new();
        for dc in -1..=1 {
            for ds in -1..=1 {
                for doff in -1..=1 {
                    if let Some(v) = self.buckets.get(&(bc + dc, bs + ds, bo + doff)) {
                        out.extend_from_slice(v);
                    }
                }
            }
        }
        out
    }

    fn insert(&mut self, idx: usize, line_point: [f64; 2], direction: [f64; 2]) {
        let key = self.key(line_point, direction);
        self.buckets.entry(key).or_default().push(idx);
    }
}

/// 点群の重心。
fn centroid(pts: &[[f64; 2]]) -> [f64; 2] {
    let n = pts.len().max(1) as f64;
    let sum = pts
        .iter()
        .fold([0.0, 0.0], |a, p| [a[0] + p[0], a[1] + p[1]]);
    [sum[0] / n, sum[1] / n]
}

/// 柱の柱脚位置（XY）を [`crate::geom::MEMBER_AXIS_TOL_MM`] 以内で重複排除して集める。
fn column_footprints(model: &Model) -> Vec<[f64; 2]> {
    let mut pts: Vec<[f64; 2]> = Vec::new();
    for e in &model.elements {
        if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
            continue;
        }
        let (Some(a), Some(b)) = (
            model.nodes.get(e.nodes[0].index()),
            model.nodes.get(e.nodes[1].index()),
        ) else {
            continue;
        };
        if !is_vertical_pair(a.coord, b.coord) {
            continue;
        }
        let p = [a.coord[0], a.coord[1]];
        let dup = pts
            .iter()
            .any(|&q| dist2(p, q) <= MEMBER_AXIS_TOL_MM * MEMBER_AXIS_TOL_MM);
        if !dup {
            pts.push(p);
        }
    }
    pts
}

/// 2 点から見出し角 `[0, π)` に正規化した単位方向を作る。距離がほぼ 0 なら `None`。
///
/// 角度の mod 演算（`atan2` の結果を `π` で割った余りを取る等）は 0°/180° の境界で
/// 不連続になるため使わない。ベクトルの符号反転による正規化は、この境界の前後で
/// 連続に振る舞う（179.99° 付近と 0.01° 付近はどちらも符号反転後にほぼ同じ方向になる）。
fn canonical_direction(p: [f64; 2], q: [f64; 2]) -> Option<[f64; 2]> {
    let d = [q[0] - p[0], q[1] - p[1]];
    let len = (d[0] * d[0] + d[1] * d[1]).sqrt();
    if len <= f64::EPSILON {
        return None;
    }
    let mut u = [d[0] / len, d[1] / len];
    if u[0] < 0.0 || (u[0] == 0.0 && u[1] < 0.0) {
        u = [-u[0], -u[1]];
    }
    Some(u)
}

/// 直線 `(origin, direction)` が、`defining_points`（新たな候補直線を定義した柱脚位置）と
/// 同一の構面とみなせるか。
///
/// 角度の許容差ではなく、各点から既存直線までの実距離 [mm] で判定する
/// （モジュールドキュメント参照）。
fn is_same_line(origin: [f64; 2], direction: [f64; 2], defining_points: &[[f64; 2]]) -> bool {
    defining_points
        .iter()
        .all(|&p| point_to_line_dist(origin, direction, p) <= MEMBER_AXIS_TOL_MM)
}

/// 点から直線（`origin` を通り `direction` 方向）までの距離 [mm]。
fn point_to_line_dist(origin: [f64; 2], direction: [f64; 2], p: [f64; 2]) -> f64 {
    let v = [p[0] - origin[0], p[1] - origin[1]];
    (v[0] * direction[1] - v[1] * direction[0]).abs()
}

fn dist2(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)
}

/// 直線 `(origin, direction)` に乗る `ElementKind::Beam` を辺として集める。
/// 両端の XY 射影が直線から `MEMBER_AXIS_TOL_MM` 以内にあるものを対象とする。
fn members_on_plane(model: &Model, origin: [f64; 2], direction: [f64; 2]) -> Vec<Edge> {
    let mut edges = Vec::new();
    for e in &model.elements {
        if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
            continue;
        }
        let (Some(a), Some(b)) = (
            model.nodes.get(e.nodes[0].index()),
            model.nodes.get(e.nodes[1].index()),
        ) else {
            continue;
        };
        let (pa, pb) = ([a.coord[0], a.coord[1]], [b.coord[0], b.coord[1]]);
        if point_to_line_dist(origin, direction, pa) > MEMBER_AXIS_TOL_MM
            || point_to_line_dist(origin, direction, pb) > MEMBER_AXIS_TOL_MM
        {
            continue;
        }
        let (sa, sb) = (
            project(origin, direction, a.coord),
            project(origin, direction, b.coord),
        );
        let proj_len = ((sa[0] - sb[0]).powi(2) + (sa[1] - sb[1]).powi(2)).sqrt();
        if proj_len < MIN_PROJECTED_EDGE_LEN_MM {
            continue; // 直線にほぼ垂直な短い部材は射影で潰れるため除外する。
        }
        edges.push(Edge {
            a: e.nodes[0],
            b: e.nodes[1],
            elem: e.id,
        });
    }
    edges
}

/// 実座標を局所座標 `(s, z)` へ射影する。s=`direction` に沿った `origin` からの距離、
/// z=グローバル Z。
fn project(origin: [f64; 2], direction: [f64; 2], coord: [f64; 3]) -> [f64; 2] {
    let v = [coord[0] - origin[0], coord[1] - origin[1]];
    let s = v[0] * direction[0] + v[1] * direction[1];
    [s, coord[2]]
}

/// 3 次元多角形の面積（ニューエルの公式）。理想平面への投影を経由しない
/// （モジュールドキュメント参照。芯ずれ・非平面性による誤差を面積計算に持ち込まない）。
fn newell_area(pts: &[[f64; 3]]) -> f64 {
    let n = pts.len();
    if n < 3 {
        return 0.0;
    }
    let mut normal = [0.0; 3];
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        normal[0] += (a[1] - b[1]) * (a[2] + b[2]);
        normal[1] += (a[2] - b[2]) * (a[0] + b[0]);
        normal[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    0.5 * (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt()
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

    /// [`add_plane_frame`] の仕様（引数が多くなるため 1 つにまとめる）。
    struct PlaneFrameSpec {
        /// XY 平面上の起点。
        origin: [f64; 2],
        /// 柱列の見出し角 [度]。
        heading_deg: f64,
        /// 柱の水平ピッチ [mm]。
        bay: f64,
        /// 階高 [mm]。
        story_h: f64,
        /// スパン数。
        n_bay: usize,
        /// 階数。
        n_story: usize,
    }

    /// `spec.n_bay`×`spec.n_story` の平面骨組を、XY 平面上の `spec.origin` を起点に
    /// 見出し角 `spec.heading_deg` 方向へ柱列を並べて作る。節点・要素は既存モデルへ追加する。
    fn add_plane_frame(model: &mut Model, next_id: &mut u32, spec: PlaneFrameSpec) {
        let PlaneFrameSpec {
            origin,
            heading_deg,
            bay,
            story_h,
            n_bay,
            n_story,
        } = spec;
        let theta = heading_deg.to_radians();
        let (dx, dy) = (theta.cos(), theta.sin());
        let base_node = model.nodes.len() as u32;
        let idx = |ix: usize, iz: usize| base_node + (iz * (n_bay + 1) + ix) as u32;
        for iz in 0..=n_story {
            for ix in 0..=n_bay {
                let s = ix as f64 * bay;
                model.nodes.push(node(
                    idx(ix, iz),
                    origin[0] + s * dx,
                    origin[1] + s * dy,
                    iz as f64 * story_h,
                ));
            }
        }
        // 柱（各 ix 列を iz 方向につなぐ）。
        for ix in 0..=n_bay {
            for iz in 0..n_story {
                model
                    .elements
                    .push(beam(*next_id, idx(ix, iz), idx(ix, iz + 1)));
                *next_id += 1;
            }
        }
        // 梁（各 iz レベルを ix 方向につなぐ）。
        for iz in 0..=n_story {
            for ix in 0..n_bay {
                model
                    .elements
                    .push(beam(*next_id, idx(ix, iz), idx(ix + 1, iz)));
                *next_id += 1;
            }
        }
    }

    /// X 方向（見出し角 0°）の壁面。床の格子と同じ形の骨組が壁でも同様に検出できる。
    #[test]
    fn test_x_direction_plane() {
        let mut model = Model::default();
        let mut next_id = 0u32;
        add_plane_frame(
            &mut model,
            &mut next_id,
            PlaneFrameSpec {
                origin: [0.0, 0.0],
                heading_deg: 0.0,
                bay: 4000.0,
                story_h: 3000.0,
                n_bay: 2,
                n_story: 2,
            },
        );
        let scan = scan_wall_region_boundaries(&model);
        assert_eq!(scan.unclosed, 0, "半辺の後続は一意に定まるはず");
        assert_eq!(scan.boundaries.len(), 4, "2×2 の壁構面は 4 面");
        for b in &scan.boundaries {
            assert!(
                (b.area(&model) - 4000.0 * 3000.0).abs() < 1.0,
                "面積 {}",
                b.area(&model)
            );
        }
    }

    /// Y 方向（見出し角 90°）の壁面。`canonical_direction` の境界ケース（dx=0）を含む。
    #[test]
    fn test_y_direction_plane() {
        let mut model = Model::default();
        let mut next_id = 0u32;
        add_plane_frame(
            &mut model,
            &mut next_id,
            PlaneFrameSpec {
                origin: [0.0, 0.0],
                heading_deg: 90.0,
                bay: 4000.0,
                story_h: 3000.0,
                n_bay: 2,
                n_story: 2,
            },
        );
        let scan = scan_wall_region_boundaries(&model);
        assert_eq!(scan.unclosed, 0, "半辺の後続は一意に定まるはず");
        assert_eq!(scan.boundaries.len(), 4, "2×2 の壁構面は 4 面");
        for b in &scan.boundaries {
            assert!(
                (b.area(&model) - 4000.0 * 3000.0).abs() < 1.0,
                "面積 {}",
                b.area(&model)
            );
        }
    }

    /// 斜め構面（見出し角 30°）。直交グリッドでない構面も同じロジックで検出できる。
    #[test]
    fn test_oblique_plane() {
        let mut model = Model::default();
        let mut next_id = 0u32;
        add_plane_frame(
            &mut model,
            &mut next_id,
            PlaneFrameSpec {
                origin: [0.0, 0.0],
                heading_deg: 30.0,
                bay: 4000.0,
                story_h: 3000.0,
                n_bay: 2,
                n_story: 2,
            },
        );
        let scan = scan_wall_region_boundaries(&model);
        assert_eq!(scan.unclosed, 0, "半辺の後続は一意に定まるはず");
        assert_eq!(scan.boundaries.len(), 4, "2×2 の壁構面は 4 面（斜め）");
        for b in &scan.boundaries {
            assert!(
                (b.area(&model) - 4000.0 * 3000.0).abs() < 1.0,
                "面積 {}",
                b.area(&model)
            );
        }
    }

    /// 面の頂点列は正しく閉じ、境界の梁が辺と同数になる。
    #[test]
    fn test_single_boundary_shape() {
        let mut model = Model::default();
        let mut next_id = 0u32;
        add_plane_frame(
            &mut model,
            &mut next_id,
            PlaneFrameSpec {
                origin: [0.0, 0.0],
                heading_deg: 0.0,
                bay: 4000.0,
                story_h: 3000.0,
                n_bay: 1,
                n_story: 1,
            },
        );
        let boundaries = generate_wall_region_boundaries(&model);
        assert_eq!(boundaries.len(), 1);
        assert_eq!(boundaries[0].boundary.len(), 4);
        assert_eq!(boundaries[0].edges.len(), 4);
    }

    /// 同一直線とみなす判定は角度ではなく実距離で行う。50m の直線に対し、
    /// 遠端で 9mm ずれた点は同一直線、11mm ずれた点は別の直線とみなす。
    #[test]
    fn test_is_same_line_uses_absolute_distance_not_angle() {
        let origin = [0.0, 0.0];
        let direction = [1.0, 0.0];
        let near = [50_000.0, 9.0];
        let far = [50_000.0, 11.0];
        assert!(is_same_line(origin, direction, &[near]), "9mm は同一直線");
        assert!(!is_same_line(origin, direction, &[far]), "11mm は別の直線");
    }

    /// 180°付近の見出し角も、点の与え方（順序）によらず同じ正規化方向になる
    /// （0°/180°の境界で不連続にならないことの確認）。
    #[test]
    fn test_canonical_direction_is_order_independent_near_seam() {
        let p = [0.0, 0.0];
        let q = [-1000.0, 0.2]; // 見出し角 ~179.99°
        let d1 = canonical_direction(p, q).expect("方向が求まる");
        let d2 = canonical_direction(q, p).expect("方向が求まる");
        assert!((d1[0] - d2[0]).abs() < 1e-9 && (d1[1] - d2[1]).abs() < 1e-9);
    }

    /// 構面にほぼ垂直に取り付く部材があっても、面の数は変わらない
    /// （両端が構面から離れるため対象部材の絞り込みで除外される）。
    #[test]
    fn test_perpendicular_beam_does_not_change_face_count() {
        let mut model = Model::default();
        let mut next_id = 0u32;
        add_plane_frame(
            &mut model,
            &mut next_id,
            PlaneFrameSpec {
                origin: [0.0, 0.0],
                heading_deg: 0.0,
                bay: 4000.0,
                story_h: 3000.0,
                n_bay: 1,
                n_story: 1,
            },
        );
        // 境界節点 (4000,0,3000) から構面に垂直（Y方向）へ張り出す梁。
        let extra_node = model.nodes.len() as u32;
        model.nodes.push(node(extra_node, 4000.0, 2000.0, 3000.0));
        model.elements.push(beam(next_id, 3, extra_node));
        let boundaries = generate_wall_region_boundaries(&model);
        assert_eq!(boundaries.len(), 1, "構面に垂直な部材は面の数を変えない");
    }

    /// 平面図形の面積は、ニューエルの公式による 3 次元面積で求まり、投影面積とは異なる
    /// （境界上の 1 節点が理想平面から外れている場合。§3.2 E3 の確認）。
    ///
    /// 2 スパン・1 層の骨組で、中央の柱（ix=1）の柱頭だけを Y 方向へ 5mm ずらして
    /// 非平面な境界を作る。**柱自身をそろってずらすと（柱脚・柱頭とも同じだけ動かすと）、
    /// 2 本の鉛直線が張る面は常に平面になり非平面を作れない**（鉛直な 2 直線は
    /// オフセットの大小によらず必ず 1 つの平面を張るため）。柱頭・柱脚のどちらか
    /// 片方だけをずらして初めて非平面になる。
    #[test]
    fn test_area_uses_3d_newell_not_projected_area() {
        let mut model = Model::default();
        let mut next_id = 0u32;
        add_plane_frame(
            &mut model,
            &mut next_id,
            PlaneFrameSpec {
                origin: [0.0, 0.0],
                heading_deg: 0.0,
                bay: 4000.0,
                story_h: 3000.0,
                n_bay: 2,
                n_story: 1,
            },
        );
        let boundaries = generate_wall_region_boundaries(&model);
        assert_eq!(boundaries.len(), 2, "2 スパンの壁構面は 2 面");
        for b in &boundaries {
            assert!(
                (b.area(&model) - 4000.0 * 3000.0).abs() < 1.0,
                "平面のときは投影面積と一致: {}",
                b.area(&model)
            );
        }

        // 中央柱（ix=1）の柱頭ノードだけを Y+5mm ずらす（柱自身の鉛直判定
        // `VERTICAL_TOL_MM`=1mm は超えるため footprint の抽出元からは外れるが、
        // 構面の許容差 `MEMBER_AXIS_TOL_MM`=10mm には収まるため部材としては引き続き拾われる）。
        let mid_top = 4; // idx(1, 1) = 1*(n_bay+1)+1 = 1*3+1
        model.nodes[mid_top].coord[1] += 5.0;
        let boundaries = generate_wall_region_boundaries(&model);
        assert_eq!(boundaries.len(), 2, "5mm のずれは面走査の結果を左右しない");
        let newell_areas: Vec<f64> = boundaries.iter().map(|b| b.area(&model)).collect();
        assert!(
            newell_areas
                .iter()
                .any(|a| (a - 4000.0 * 3000.0).abs() > 1.0),
            "少なくとも 1 面はニューエル面積が投影面積からずれる: {newell_areas:?}"
        );
    }
}
