//! 幾何判定のテスト。

use super::*;

/// 法線が期待方向と（符号を問わず）平行か。
fn assert_parallel(got: [f64; 3], want: [f64; 3], label: &str) {
    let dot = got[0] * want[0] + got[1] * want[1] + got[2] * want[2];
    assert!(
        (dot.abs() - 1.0).abs() < 1e-6,
        "{label}: got={got:?} want=±{want:?} (dot={dot})"
    );
}

/// 鉛直面（X = 一定）の点群から、X 方向の法線が求まる。
///
/// 通常の線形回帰（z = ax + by + c）では表せない配置であり、主成分分析でしか
/// 解けないことを確認する（同じ (x,y) に z 違いの点が複数ある）。
#[test]
fn fits_vertical_plane() {
    let pts = vec![
        [3000.0, 0.0, 0.0],
        [3000.0, 0.0, 4000.0],
        [3000.0, 6000.0, 0.0],
        [3000.0, 6000.0, 4000.0],
        [3000.0, 12000.0, 8000.0],
    ];
    let n = best_fit_plane_normal(&pts).expect("法線");
    assert_parallel(n, [1.0, 0.0, 0.0], "X=一定の鉛直面");
}

/// 水平面（Z = 一定）の点群からは Z 方向の法線が求まる。
#[test]
fn fits_horizontal_plane() {
    let pts = vec![
        [0.0, 0.0, 4000.0],
        [6000.0, 0.0, 4000.0],
        [0.0, 5000.0, 4000.0],
        [6000.0, 5000.0, 4000.0],
    ];
    let n = best_fit_plane_normal(&pts).expect("法線");
    assert_parallel(n, [0.0, 0.0, 1.0], "Z=一定の水平面");
}

/// 斜めの鉛直面（X-Y 平面内で 45° 振れた構面）でも法線が求まる。
#[test]
fn fits_skewed_vertical_plane() {
    // x = y の鉛直面 → 法線は (1,-1,0)/√2。
    let pts = vec![
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 4000.0],
        [5000.0, 5000.0, 0.0],
        [5000.0, 5000.0, 4000.0],
        [10000.0, 10000.0, 8000.0],
    ];
    let n = best_fit_plane_normal(&pts).expect("法線");
    let s = 1.0 / 2.0_f64.sqrt();
    assert_parallel(n, [s, -s, 0.0], "x=y の鉛直面");
}

/// 点が一直線（水平）に並ぶ場合は平面が定まらないため、その直線と鉛直軸を
/// 含む平面の法線を返す。
#[test]
fn collinear_horizontal_points_give_vertical_plane() {
    let pts = vec![
        [0.0, 0.0, 4000.0],
        [3000.0, 0.0, 4000.0],
        [6000.0, 0.0, 4000.0],
    ];
    let n = best_fit_plane_normal(&pts).expect("法線");
    // 直線は X 方向 → 直線と Z を含む平面（XZ 平面）の法線は Y。
    assert_parallel(n, [0.0, 1.0, 0.0], "X 方向に並ぶ点");
    // 法線が鉛直でない（＝構面が鉛直面として読める）ことも確かめる。
    assert!(n[2].abs() < 1e-9, "法線は水平: {n:?}");
}

/// 点が鉛直な一直線（1 本の柱に積まれた節点）に並ぶ場合は、直線と鉛直軸が
/// 一致して平面がまったく定まらないため、X 方向を法線とする。
#[test]
fn collinear_vertical_points_fall_back_to_x_normal() {
    let pts = vec![
        [1000.0, 2000.0, 0.0],
        [1000.0, 2000.0, 4000.0],
        [1000.0, 2000.0, 8000.0],
    ];
    let n = best_fit_plane_normal(&pts).expect("法線");
    assert_parallel(n, [1.0, 0.0, 0.0], "鉛直に並ぶ点");
}

/// 点が 3 個未満、または全点が同一の場合は法線を決められない。
#[test]
fn degenerate_inputs_return_none() {
    assert!(best_fit_plane_normal(&[]).is_none());
    assert!(best_fit_plane_normal(&[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]).is_none());
    let same = [[5.0, 5.0, 5.0]; 4];
    assert!(best_fit_plane_normal(&same).is_none(), "全点が同一");
}

/// 平面から少しばらついた点群でも、最も当てはまる平面の法線が求まる。
#[test]
fn tolerates_scatter_around_plane() {
    let pts = vec![
        [3000.0, 0.0, 0.0],
        [3001.0, 0.0, 4000.0],
        [2999.0, 6000.0, 0.0],
        [3000.5, 6000.0, 4000.0],
        [2999.5, 12000.0, 8000.0],
    ];
    let n = best_fit_plane_normal(&pts).expect("法線");
    // ばらつきは 1mm 程度、面の広がりは数 m のため、法線はほぼ X 方向。
    assert!(n[0].abs() > 0.999, "法線はほぼ X 方向: {n:?}");
}

#[test]
fn vertical_pair_uses_horizontal_distance() {
    assert!(is_vertical_pair([0.0, 0.0, 0.0], [0.0, 0.0, 3000.0]));
    assert!(is_vertical_pair([0.0, 0.0, 0.0], [0.5, 0.5, 3000.0]));
    assert!(!is_vertical_pair([0.0, 0.0, 0.0], [10.0, 0.0, 3000.0]));
}
