//! 3 次元ベクトルの基本演算。
//!
//! 節点座標・部材軸・面法線の扱いはクレートを横断して現れるため、内積・外積・
//! ノルム・単位化といった最小限の演算をここに集約する（各クレートが同じ数式を
//! 私的なヘルパとして持ち直すのを防ぎ、零ベクトル判定の閾値も 1 か所に保つ）。
//!
//! 座標系は常に mm。ベクトルは `[f64; 3]` の値型で受け渡す（要素数 3 の
//! 固定長配列はコピーが安く、参照を持ち回るより呼び出し側が読みやすい）。

/// 数値的に零ベクトルとみなすノルムの上限 [mm]。
///
/// 座標が mm であるため、これを下回る長さの部材軸・面法線は縮退している
/// （方向を定義できない）とみなす。
pub const ZERO_TOL: f64 = 1e-9;

/// 差 `a − b`。
pub fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// 和 `a + b`。
pub fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// スカラ倍 `s・a`。
pub fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

/// 2 点の中点。
pub fn midpoint(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        0.5 * (a[0] + b[0]),
        0.5 * (a[1] + b[1]),
        0.5 * (a[2] + b[2]),
    ]
}

/// 内積。
pub fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// 外積。
pub fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// ノルム（長さ）。
pub fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// 2 点間の距離。
pub fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    norm(sub(a, b))
}

/// 単位ベクトル。長さが [`ZERO_TOL`] 以下（縮退）なら `None`。
pub fn unit(a: [f64; 3]) -> Option<[f64; 3]> {
    let l = norm(a);
    (l > ZERO_TOL).then(|| [a[0] / l, a[1] / l, a[2] / l])
}

/// `a` から `b` へ向かう単位ベクトル。2 点が一致（縮退）なら `None`。
pub fn unit_from(a: [f64; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    unit(sub(b, a))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dot_cross_norm_の基本則() {
        let a = [1.0, 0.0, 0.0];
        let b = [0.0, 2.0, 0.0];
        assert_eq!(dot(a, b), 0.0);
        assert_eq!(cross(a, b), [0.0, 0.0, 2.0]);
        assert_eq!(norm(b), 2.0);
        // 外積は両ベクトルに直交する。
        let c = cross(a, b);
        assert_eq!(dot(c, a), 0.0);
        assert_eq!(dot(c, b), 0.0);
    }

    #[test]
    fn unit_は縮退ベクトルで_none_を返す() {
        assert_eq!(unit([0.0, 0.0, 0.0]), None);
        assert_eq!(unit([ZERO_TOL, 0.0, 0.0]), None);
        assert_eq!(unit([0.0, 3.0, 4.0]), Some([0.0, 0.6, 0.8]));
    }

    #[test]
    fn dist_と_midpoint() {
        let a = [0.0, 0.0, 0.0];
        let b = [3.0, 4.0, 0.0];
        assert_eq!(dist(a, b), 5.0);
        assert_eq!(midpoint(a, b), [1.5, 2.0, 0.0]);
        assert_eq!(unit_from(a, b), Some([0.6, 0.8, 0.0]));
        assert_eq!(unit_from(a, a), None);
    }
}
