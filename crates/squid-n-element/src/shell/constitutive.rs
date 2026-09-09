//! 等方性材料の構成則行列（平面応力・曲げ・せん断）。
//!
//! - [`d_membrane`] — 膜剛性 D 行列（厚さ積分済み）
//! - [`d_bending`] — 曲げ剛性 D 行列
//! - [`d_shear`] — 横せん断剛性 D 行列（せん断補正係数 5/6）

pub(crate) fn d_membrane(e: f64, nu: f64, t: f64) -> [[f64; 3]; 3] {
    let c = e * t / (1.0 - nu * nu);
    [
        [c, c * nu, 0.0],
        [c * nu, c, 0.0],
        [0.0, 0.0, c * (1.0 - nu) / 2.0],
    ]
}

pub(crate) fn d_bending(e: f64, nu: f64, t: f64) -> [[f64; 3]; 3] {
    let d0 = e * t * t * t / (12.0 * (1.0 - nu * nu));
    [
        [d0, d0 * nu, 0.0],
        [d0 * nu, d0, 0.0],
        [0.0, 0.0, d0 * (1.0 - nu) / 2.0],
    ]
}

pub(crate) fn d_shear(e: f64, nu: f64, t: f64) -> [[f64; 2]; 2] {
    let g = e / (2.0 * (1.0 + nu));
    let c = g * t * 5.0 / 6.0;
    [[c, 0.0], [0.0, c]]
}
