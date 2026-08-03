//! シェル断面力のデータ構造。
//!
//! - [`ShellResultants`] — 単位幅あたりの断面力（膜・曲げ・せん断）

/// Shell resultants per unit width at a point.
#[derive(Clone, Debug, PartialEq)]
pub struct ShellResultants {
    pub nx: f64,
    pub ny: f64,
    pub nxy: f64,
    pub mx: f64,
    pub my: f64,
    pub mxy: f64,
    pub qx: f64,
    pub qy: f64,
}
