//! 4節点シェル要素（MITC4）。責務ごとにサブモジュール分割。
//!
//! - [`resultants`] — 断面力・コンター用データ構造
//! - [`frame`] — 要素ローカル直交フレームと回転変換
//! - [`shape`] — 形状関数・ヤコビアン・ガウス積分点
//! - [`constitutive`] — 構成則 D 行列（膜・曲げ・せん断）
//! - [`geom`] — 要素面積などの幾何量
//! - [`element`] — [`ShellElement`] 本体と生成・ローカル座標
//! - [`bmatrix`] — B 行列（膜・曲げ・MITC4 せん断）
//! - [`stiffness`] — 剛性行列とドリリング安定化
//! - [`recovery`] — 断面力回復・コンター外挿
//! - [`element_behavior`] — [`ElementBehavior`](crate::behavior::ElementBehavior) 実装

#[cfg(test)]
use crate::behavior::LocalMat;
#[cfg(test)]
use squid_n_core::ids::NodeId;

pub const DEFAULT_DRILLING_FACTOR: f64 = 1.0e-3;
pub const N_GAUSS: usize = 2;

mod bmatrix;
mod constitutive;
mod element;
mod element_behavior;
mod frame;
mod geom;
mod recovery;
mod resultants;
mod shape;
mod stiffness;

pub use element::ShellElement;
pub use frame::ShellFrame;
pub use resultants::ShellResultants;

// tests（shell::tests）が `use super::*` から直接参照する自由関数を供給。
#[cfg(test)]
pub(crate) use shape::{dshape_cart, shape_2d};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
#[allow(non_snake_case)]
mod tests;
