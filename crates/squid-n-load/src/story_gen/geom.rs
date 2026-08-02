//! 幾何ユーティリティ（座標計算）。
//!
//! - [`polygon_area_3d`] — 平面多角形（3D座標）の面積（Newell の公式）
//! - [`dist3`] — 2 点間の 3D 距離
//! - `is_vertical_pair` — 両端が鉛直材（柱）かの判定
//!   （[`squid_n_core::geom::is_vertical_pair`] の再エクスポート）

/// 平面多角形（3D座標、頂点が同一平面上と仮定）の面積。
///
/// Newell の公式 `N = 1/2 Σ(Vi × Vi+1)`, `Area = |N|` による。
/// 凸・非凸いずれも、頂点が境界を一周する順序で与えられていれば成立する。
/// 壁・シェル要素の自重（§1.2）算定に用いる。
pub(super) fn polygon_area_3d(pts: &[[f64; 3]]) -> f64 {
    if pts.len() < 3 {
        return 0.0;
    }
    let n = pts.len();
    let (mut nx, mut ny, mut nz) = (0.0, 0.0, 0.0);
    for i in 0..n {
        let p0 = pts[i];
        let p1 = pts[(i + 1) % n];
        nx += p0[1] * p1[2] - p0[2] * p1[1];
        ny += p0[2] * p1[0] - p0[0] * p1[2];
        nz += p0[0] * p1[1] - p0[1] * p1[0];
    }
    0.5 * (nx * nx + ny * ny + nz * nz).sqrt()
}

/// 2 点間の 3D 距離 [mm]。
pub(super) fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// 「鉛直材（柱）」判定。仕上げ周長式・雑壁の柱探索・柱脚梁せい付加・通り芯の
/// 自動生成が共通で用いるため、判定規則は `squid-n-core` を情報源とする。
pub(super) use squid_n_core::geom::is_vertical_pair;
