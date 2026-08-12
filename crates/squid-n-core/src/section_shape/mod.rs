//! パラメトリック断面形状 [`SectionShape`] と派生断面性能。
//!
//! 責務ごとにサブモジュールへ分割している。
//!
//! - [`types`] — 型定義（BarSet, ShearBar, RcRebar, SectionShape, bar_set_area）
//! - [`constants`] — 材料・換算定数
//! - [`material`] — 材料換算関数（Ec, 壁せん断形状係数）
//! - [`geometry`] — 断面幾何量のヘルパ
//! - [`properties`] — 基本断面性能（A, Iy, Iz, J, ...）
//! - [`composite`] — SRC/CFT の複合換算断面性能
//! - [`builder`] — Section 生成
//! - [`label`] — 形状と寸法の表記（`H-500x250x9x16` 等）

mod builder;
mod composite;
mod constants;
mod geometry;
mod label;
mod material;
mod properties;
mod types;

#[cfg(test)]
mod tests;

// tests は `use super::*` 経由で `SectionId(0)` を参照する（非テストビルドでは
// 未使用となるため cfg(test) でゲートする）。
#[cfg(test)]
use crate::ids::SectionId;

pub use composite::{CftCoreProps, CompositeProps};
pub use constants::{E_STEEL, KAPPA_RC, N_S_EQ};
pub use material::{
    concrete_young_modulus, concrete_young_modulus_gamma, wall_shear_shape_factor,
    wall_shear_shape_factor_isection,
};
pub use types::{bar_set_area, BarSet, RcRebar, SectionShape, ShearBar};
