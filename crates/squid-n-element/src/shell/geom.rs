//! 要素の幾何量。
//!
//! - [`element_area`] — 四辺形を2三角形に分割した面積

use squid_n_core::geom::vec3;

/// 三角形 (a, b, c) の面積。2 辺の外積の大きさの半分。
fn triangle_area(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    0.5 * vec3::norm(vec3::cross(vec3::sub(b, a), vec3::sub(c, a)))
}

pub(crate) fn element_area(coords: &[[f64; 3]; 4]) -> f64 {
    // 四辺形を三角形 0-1-2 と 1-2-3 の 2 つへ分けて足す。
    triangle_area(coords[0], coords[1], coords[2]) + triangle_area(coords[1], coords[2], coords[3])
}
