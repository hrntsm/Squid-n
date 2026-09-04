//! 多角形の幾何演算（面積・重心・点と辺の距離・内包判定）。
//!
//! 床領域・壁領域の検出とリビルド（[`crate::region_gen`]・[`crate::region_rebuild`]・
//! [`crate::wall_region_rebuild`]）、床の荷重分配（`squid_n_load::floor`）、
//! 部材数量の拾い出しが同じ式を必要とするため、ここを唯一の実装とする。
//!
//! 2 次元の関数は座標系を問わない（床は全体 XY、壁は構面の局所 `(s, z)`、
//! 断面は局所 `(y, z)` で使う）。長さは mm、面積は mm² を単位とする。
//!
//! # 面積の 3 種
//!
//! 多角形の面積には**投影面積と真の面積**があり、用途で使い分ける。
//!
//! - [`signed_area`]・[`area`] — 2 次元多角形のシューレース公式。
//! - [`area_xy`] — 3 次元の頂点を全体 XY 平面へ投影した面積。床は水平面内にあるため、
//!   床の荷重分配はこちらを使う（分配が見る面積と一致させる）。
//! - [`area_3d`] — 3 次元の頂点が張る平面上での真の面積（Newell の公式）。
//!   傾いた壁・シェルの自重はこちらを使う。投影面積は傾きのぶんだけ小さく出るため、
//!   自重に投影面積を用いると重量を過小評価する**危険側**になる。

/// 辺上とみなす、点から辺までの距離の上限 \[mm\]。
///
/// [`contains_excluding_boundary`]・[`contains_including_boundary`] が共有する。
/// 床領域・壁領域への帰属判定と、床の荷重分配が同じ幅で辺上を判断する
/// （判定規則の情報源を 1 つに保つ）。
pub const BOUNDARY_TOL_MM: f64 = 1.0;

/// 縮退とみなす、境界ボックスの最大辺長の 2 乗に対する面積の相対上限
/// （根拠は [`centroid`] のドキュメントを参照）。
const DEGENERATE_AREA_REL: f64 = 1e-12;

/// 多角形の符号付き面積 \[mm²\]（シューレース公式。反時計回りが正）。頂点が 3 個未満なら 0。
///
/// 符号は面走査（[`crate::region_gen`]）が閉路の向きから外周面を判別するのに使う。
/// 向きを問わない面積は [`area`] を使う。
pub fn signed_area(pts: &[[f64; 2]]) -> f64 {
    shoelace(pts.len(), |i| pts[i])
}

/// シューレース公式の本体。`xy(i)` は i 番目の頂点の `(x, y)` を返す。
///
/// 2 次元の点列と、3 次元の点列を XY へ投影したものの両方から呼ぶため、
/// 頂点の取り出し方を引数に切り出している（[`area_xy`] が同じ式を持ち直さずに済む）。
fn shoelace(n: usize, xy: impl Fn(usize) -> [f64; 2]) -> f64 {
    if n < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..n {
        let a = xy(i);
        let b = xy((i + 1) % n);
        sum += a[0] * b[1] - b[0] * a[1];
    }
    sum / 2.0
}

/// 多角形の面積 \[mm²\]（絶対値。頂点の周り方に依らない）。
pub fn area(pts: &[[f64; 2]]) -> f64 {
    signed_area(pts).abs()
}

/// 3 次元の頂点を全体 XY 平面へ投影した多角形の面積 \[mm²\]。
///
/// 床は水平面内にあるという前提で、床板の面積と荷重分配が見る面積をそろえる。
/// 傾いた面の真の面積が要るときは [`area_3d`] を使う。
pub fn area_xy(pts: &[[f64; 3]]) -> f64 {
    shoelace(pts.len(), |i| [pts[i][0], pts[i][1]]).abs()
}

/// 平面多角形（3 次元座標、頂点が同一平面上と仮定）の面積 \[mm²\]。
///
/// Newell の公式 `N = 1/2 Σ(Vi × Vi+1)`, `Area = |N|` による。凸・非凸いずれも、
/// 頂点が境界を一周する順序で与えられていれば成立する。頂点が 3 個未満なら 0。
///
/// 壁・シェル要素の自重算定とスラブ・壁の数量拾いが共通で用いる。
pub fn area_3d(pts: &[[f64; 3]]) -> f64 {
    if pts.len() < 3 {
        return 0.0;
    }
    let n = pts.len();
    let mut normal = [0.0_f64; 3];
    for i in 0..n {
        let (p0, p1) = (pts[i], pts[(i + 1) % n]);
        let c = super::vec3::cross(p0, p1);
        normal = super::vec3::add(normal, c);
    }
    0.5 * super::vec3::norm(normal)
}

/// 多角形の面積重心。
///
/// 縮退した多角形（頂点が 3 個未満、または一直線に並ぶもの）は面積重心が定まらないため、
/// 頂点の単純平均へフォールバックする。
///
/// # 縮退の判定を絶対値で行わない理由
///
/// 一直線に並んだ頂点の符号付き面積は厳密には 0 だが、建物座標（~1e4 mm）では
/// シューレース和の各項が ~1e8 になるため、丸め誤差で |面積| ~1e-8 mm² が残る。
/// これを 1e-9 や [`f64::EPSILON`] のような絶対値のしきい値と比べても縮退と判定できず、
/// 残差が `6A` の除算へ入って重心が発散する。
///
/// そこで面積を**境界ボックスの最大辺長の 2 乗**と比べる。比は無次元なので、
/// 建物の平面（~1e4 mm）でも断面の輪郭（~1e2 mm）でも同じ判定が使える。
/// 境界ボックスの「面積」を基準に採らないのは、縮退した多角形ではそれ自体が 0 へ
/// 潰れて基準にならないためである。シューレース和の相対丸め誤差は頂点数 n に対し
/// 概ね `n · f64::EPSILON`（~1e-15）で、しきい値 1e-12 はそれに 3 桁の余裕を持つ。
pub fn centroid(pts: &[[f64; 2]]) -> [f64; 2] {
    let vertex_mean = || {
        if pts.is_empty() {
            return [0.0, 0.0];
        }
        let n = pts.len() as f64;
        [
            pts.iter().map(|p| p[0]).sum::<f64>() / n,
            pts.iter().map(|p| p[1]).sum::<f64>() / n,
        ]
    };
    if pts.len() < 3 {
        return vertex_mean();
    }
    let a = signed_area(pts);
    if a.abs() <= max_extent_sq(pts) * DEGENERATE_AREA_REL {
        return vertex_mean();
    }
    let mut cx = 0.0;
    let mut cy = 0.0;
    for (i, p0) in pts.iter().enumerate() {
        let p1 = pts[(i + 1) % pts.len()];
        let cross = p0[0] * p1[1] - p1[0] * p0[1];
        cx += (p0[0] + p1[0]) * cross;
        cy += (p0[1] + p1[1]) * cross;
    }
    let six_a = 6.0 * a;
    [cx / six_a, cy / six_a]
}

/// 点群の境界ボックス `(最小の角, 最大の角)`。空の点群は原点に潰れた箱を返す。
pub fn bounding_box(pts: &[[f64; 2]]) -> ([f64; 2], [f64; 2]) {
    if pts.is_empty() {
        return ([0.0, 0.0], [0.0, 0.0]);
    }
    let (mut min_x, mut max_x) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut min_y, mut max_y) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in pts {
        min_x = min_x.min(p[0]);
        max_x = max_x.max(p[0]);
        min_y = min_y.min(p[1]);
        max_y = max_y.max(p[1]);
    }
    ([min_x, min_y], [max_x, max_y])
}

/// 境界ボックスの長いほうの辺長の 2 乗 \[mm²\]。面積と同じ次元を持つ縮退判定の基準。
fn max_extent_sq(pts: &[[f64; 2]]) -> f64 {
    let (lo, hi) = bounding_box(pts);
    let extent = (hi[0] - lo[0]).max(hi[1] - lo[1]);
    extent * extent
}

/// 点 `p` から線分 `a`–`b` までの距離 \[mm\]。
pub fn point_segment_dist(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    point_segment_dist_sq(p, a, b).sqrt()
}

/// 点 `p` から線分 `a`–`b` までの距離の 2 乗 \[mm²\]。
///
/// 距離どうしを比べるだけなら平方根を省ける（比較のしきい値も 2 乗して渡す）。
pub fn point_segment_dist_sq(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1];
    let t = if len2 <= f64::EPSILON {
        0.0
    } else {
        (((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / len2).clamp(0.0, 1.0)
    };
    let q = [a[0] + t * ab[0], a[1] + t * ab[1]];
    let (dx, dy) = (p[0] - q[0], p[1] - q[1]);
    dx * dx + dy * dy
}

/// 点 `p` が多角形の辺上（頂点を含む）にあるか。辺までの距離が
/// [`BOUNDARY_TOL_MM`] 以内を辺上とする。頂点が 2 個未満なら偽。
pub fn on_boundary(poly: &[[f64; 2]], p: [f64; 2]) -> bool {
    let n = poly.len();
    if n < 2 {
        return false;
    }
    let tol2 = BOUNDARY_TOL_MM * BOUNDARY_TOL_MM;
    (0..n).any(|i| point_segment_dist_sq(p, poly[i], poly[(i + 1) % n]) <= tol2)
}

/// 点 `p` が多角形の内部にあるか。**辺上（[`BOUNDARY_TOL_MM`] 以内）は内部に含めない。**
///
/// 版や二次部材をどの領域へ帰属させるか決める用途に使う。辺上の点は隣接する領域の
/// 双方に該当してしまうため、含めると帰属を一意に決められない。
///
/// 辺上を含めたいときは [`contains_including_boundary`] を使う。**どちらを選ぶかは
/// 「辺上の点を落としたいか拾いたいか」で決まり、取り違えると帰属の重複か脱落を招く**
/// ため、名前で区別できる 2 つの関数に分けている。
pub fn contains_excluding_boundary(poly: &[[f64; 2]], p: [f64; 2]) -> bool {
    if poly.len() < 3 || on_boundary(poly, p) {
        return false;
    }
    ray_crossing(poly, p)
}

/// 点 `p` が多角形の内部または辺上（[`BOUNDARY_TOL_MM`] 以内）にあるか。
///
/// 床板の縁に載る小梁のように、辺上の点を対象から落としたくない用途に使う。
/// 辺上を除きたいときは [`contains_excluding_boundary`] を使う。
pub fn contains_including_boundary(poly: &[[f64; 2]], p: [f64; 2]) -> bool {
    if poly.len() < 3 {
        return false;
    }
    on_boundary(poly, p) || ray_crossing(poly, p)
}

/// レイキャスト（偶奇則）だけで内包を判定する。**[`BOUNDARY_TOL_MM`] の帯を一切見ない。**
///
/// 辺上ちょうどの点の扱いは辺の向きしだいで内側にも外側にもなるため、**点を
/// どの領域に帰属させるかを決める用途に使ってはならない**（[`contains_excluding_boundary`]
/// か [`contains_including_boundary`] を使う）。
///
/// 本関数は、床の荷重分配が格子のセル中心を「多角形の中か外か」で数え上げる走査
/// （`squid_n_load::floor`）のように、**点に帰属先を与えるのではなく面積を積む**
/// 用途のためにある。この走査に公差の帯を持ち込むと、辺から 1 mm 以内に中心を持つ
/// セルが内外どちらの判定でも数から漏れ、荷重面積が落ちる（**危険側**）。
pub fn contains_by_ray_crossing(poly: &[[f64; 2]], p: [f64; 2]) -> bool {
    if poly.len() < 3 {
        return false;
    }
    ray_crossing(poly, p)
}

/// レイキャスト（偶奇則）の本体。頂点が 3 個以上あることを前提とする。
fn ray_crossing(poly: &[[f64; 2]], p: [f64; 2]) -> bool {
    let n = poly.len();
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

#[cfg(test)]
mod tests {
    use super::*;

    /// 反時計回りの正方形。
    fn ccw_square(side: f64) -> Vec<[f64; 2]> {
        vec![[0.0, 0.0], [side, 0.0], [side, side], [0.0, side]]
    }

    #[test]
    fn signed_area_is_positive_for_ccw_and_negative_for_cw() {
        let ccw = ccw_square(10.0);
        let mut cw = ccw.clone();
        cw.reverse();
        assert_eq!(signed_area(&ccw), 100.0);
        assert_eq!(signed_area(&cw), -100.0);
        assert_eq!(area(&cw), 100.0);
    }

    #[test]
    fn signed_area_of_degenerate_input_is_zero() {
        assert_eq!(signed_area(&[]), 0.0);
        assert_eq!(signed_area(&[[0.0, 0.0], [1.0, 1.0]]), 0.0);
    }

    #[test]
    fn area_xy_projects_and_area_3d_keeps_true_area() {
        // XZ 平面に立つ 1000×1000 の正方形。XY へ投影すると潰れて 0 になる。
        let pts = [
            [0.0, 0.0, 0.0],
            [1000.0, 0.0, 0.0],
            [1000.0, 0.0, 1000.0],
            [0.0, 0.0, 1000.0],
        ];
        assert_eq!(area_xy(&pts), 0.0);
        assert!((area_3d(&pts) - 1_000_000.0).abs() < 1e-6);
    }

    #[test]
    fn centroid_of_offset_rectangle_is_its_center() {
        let rect = [
            [100.0, 200.0],
            [300.0, 200.0],
            [300.0, 600.0],
            [100.0, 600.0],
        ];
        let c = centroid(&rect);
        assert!((c[0] - 200.0).abs() < 1e-9);
        assert!((c[1] - 400.0).abs() < 1e-9);
    }

    /// 面積重心は凹多角形でも頂点の単純平均とは一致しない（L 形で確認する）。
    #[test]
    fn centroid_of_l_shape_differs_from_vertex_mean() {
        let l = [
            [0.0, 0.0],
            [200.0, 0.0],
            [200.0, 100.0],
            [100.0, 100.0],
            [100.0, 200.0],
            [0.0, 200.0],
        ];
        let c = centroid(&l);
        // 面積 30000、面積重心は手計算で (250/3, 250/3)。
        assert!((c[0] - 250.0 / 3.0).abs() < 1e-9, "{c:?}");
        assert!((c[1] - 250.0 / 3.0).abs() < 1e-9, "{c:?}");
        let mean_x = l.iter().map(|p| p[0]).sum::<f64>() / l.len() as f64;
        assert!((c[0] - mean_x).abs() > 1.0, "単純平均と一致してはならない");
    }

    /// 建物座標の尺度で一直線に並んだ頂点は、丸め誤差で符号付き面積が厳密な 0 に
    /// ならないことがある。絶対値のしきい値ではこれをすり抜けて重心が発散するため、
    /// 境界ボックス相対で縮退と判定できることを確かめる。
    #[test]
    fn centroid_of_collinear_points_falls_back_to_vertex_mean() {
        let collinear = [
            [1234.5, 6789.0],
            [11234.5, 6789.0],
            [21234.5, 6789.0],
            [31234.5, 6789.0],
        ];
        let c = centroid(&collinear);
        let mean_x = collinear.iter().map(|p| p[0]).sum::<f64>() / 4.0;
        assert!(
            c[0].is_finite() && c[1].is_finite(),
            "発散してはならない: {c:?}"
        );
        assert!((c[0] - mean_x).abs() < 1e-6, "{c:?}");
        assert!((c[1] - 6789.0).abs() < 1e-6, "{c:?}");
    }

    #[test]
    fn centroid_of_fewer_than_three_points_is_vertex_mean() {
        assert_eq!(centroid(&[]), [0.0, 0.0]);
        assert_eq!(centroid(&[[2.0, 4.0]]), [2.0, 4.0]);
        assert_eq!(centroid(&[[0.0, 0.0], [2.0, 4.0]]), [1.0, 2.0]);
    }

    #[test]
    fn point_segment_dist_clamps_to_the_segment_ends() {
        let (a, b) = ([0.0, 0.0], [10.0, 0.0]);
        assert!((point_segment_dist([5.0, 3.0], a, b) - 3.0).abs() < 1e-12);
        // 線分の外側へ出た点は端点までの距離になる。
        assert!((point_segment_dist([14.0, 3.0], a, b) - 5.0).abs() < 1e-12);
        // 長さ 0 の線分は始点までの距離。
        assert!((point_segment_dist([3.0, 4.0], a, a) - 5.0).abs() < 1e-12);
    }

    #[test]
    fn point_segment_dist_sq_is_the_square_of_the_distance() {
        let (p, a, b) = ([5.0, 3.0], [0.0, 0.0], [10.0, 0.0]);
        let d = point_segment_dist(p, a, b);
        assert!((point_segment_dist_sq(p, a, b) - d * d).abs() < 1e-12);
    }

    /// 辺上の点を落とすか拾うかが、3 つの内包判定を分ける唯一の違いである。
    #[test]
    fn the_three_containment_rules_differ_only_on_the_boundary_band() {
        let sq = ccw_square(1000.0);
        let inner = [500.0, 500.0];
        let outer = [1500.0, 500.0];
        // 辺から 0.5mm 内側（BOUNDARY_TOL_MM = 1.0 の帯の中）。
        let on_band = [500.0, 0.5];

        for p in [inner, outer, on_band] {
            let excl = contains_excluding_boundary(&sq, p);
            let incl = contains_including_boundary(&sq, p);
            let ray = contains_by_ray_crossing(&sq, p);
            match p {
                p if p == inner => assert!(excl && incl && ray, "内部は 3 つとも真"),
                p if p == outer => assert!(!excl && !incl && !ray, "外部は 3 つとも偽"),
                _ => {
                    assert!(!excl, "辺上は除外側で偽");
                    assert!(incl, "辺上は包含側で真");
                    assert!(ray, "レイキャストは帯を見ないので内部として真");
                }
            }
        }
    }

    #[test]
    fn containment_of_fewer_than_three_points_is_false() {
        let seg = [[0.0, 0.0], [10.0, 0.0]];
        assert!(!contains_excluding_boundary(&seg, [5.0, 0.0]));
        assert!(!contains_including_boundary(&seg, [5.0, 0.0]));
        assert!(!contains_by_ray_crossing(&seg, [5.0, 0.0]));
    }

    #[test]
    fn bounding_box_spans_the_point_cloud() {
        let pts = [[3.0, -1.0], [-2.0, 5.0], [1.0, 2.0]];
        assert_eq!(bounding_box(&pts), ([-2.0, -1.0], [3.0, 5.0]));
        assert_eq!(bounding_box(&[]), ([0.0, 0.0], [0.0, 0.0]));
    }

    #[test]
    fn on_boundary_covers_vertices_and_edges() {
        let sq = ccw_square(1000.0);
        assert!(on_boundary(&sq, [0.0, 0.0]), "頂点");
        assert!(on_boundary(&sq, [500.0, 0.0]), "辺上");
        assert!(on_boundary(&sq, [500.0, 0.9]), "帯の内側");
        assert!(!on_boundary(&sq, [500.0, 1.1]), "帯の外側");
        assert!(!on_boundary(&[[0.0, 0.0]], [0.0, 0.0]), "頂点 2 個未満は偽");
    }
}
