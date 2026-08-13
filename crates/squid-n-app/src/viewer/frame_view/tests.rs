//! 構面表示の視線方向のテスト。

use super::*;

/// 視線方向へスナップしたカメラで、グローバル軸が画面のどちらへ写るかを返す。
/// `(右方向成分, 上方向成分)` をグローバル X・Y・Z について返す。
fn screen_axes(dir: [f32; 3]) -> [(f32, f32); 3] {
    let mut cam = CameraState::default();
    cam.snap_to_direction(dir);
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]].map(|w| {
        let r = super::super::camera::q_rotate(cam.rot, w);
        (r[0], r[1])
    })
}

/// X 通りの軸組図: 画面右が +Y、画面上が +Z になる。
#[test]
fn x_axis_frame_faces_with_y_right_and_z_up() {
    let dir = view_direction([1.0, 0.0, 0.0]);
    let [x, y, z] = screen_axes(dir);
    assert!(x.0.abs() < 1e-5 && x.1.abs() < 1e-5, "X は視線方向: {x:?}");
    assert!(y.0 > 0.99, "画面右が +Y: {y:?}");
    assert!(z.1 > 0.99, "画面上が +Z: {z:?}");
}

/// Y 通りの軸組図: 画面右が +X、画面上が +Z になる。
///
/// 法線 +Y のまま見ると X が画面左になり図が左右反転するため、−Y 側から見る向きが
/// 選ばれることを確かめる。
#[test]
fn y_axis_frame_faces_with_x_right_and_z_up() {
    let dir = view_direction([0.0, 1.0, 0.0]);
    assert!(dir[1] < 0.0, "−Y 側から見る: {dir:?}");
    let [x, y, z] = screen_axes(dir);
    assert!(x.0 > 0.99, "画面右が +X: {x:?}");
    assert!(y.0.abs() < 1e-5 && y.1.abs() < 1e-5, "Y は視線方向: {y:?}");
    assert!(z.1 > 0.99, "画面上が +Z: {z:?}");
}

/// 伏図: 画面右が +X、画面上が +Y になる（真上から見下ろす）。
#[test]
fn story_frame_faces_from_above() {
    let dir = view_direction([0.0, 0.0, 1.0]);
    assert!(dir[2] > 0.0, "真上から見下ろす: {dir:?}");
    let [x, y, _] = screen_axes(dir);
    assert!(x.0 > 0.99, "画面右が +X: {x:?}");
    assert!(y.1 > 0.99, "画面上が +Y: {y:?}");
}

/// 斜めの構面（x = y の鉛直面）でも、画面上は +Z のままになる。
#[test]
fn skewed_frame_keeps_z_up() {
    let s = 1.0 / 2.0_f64.sqrt();
    let dir = view_direction([s, -s, 0.0]);
    let [_, _, z] = screen_axes(dir);
    assert!(z.1 > 0.99, "画面上が +Z: {z:?}");
}
