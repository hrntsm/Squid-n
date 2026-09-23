//! パラメトリック断面形状 [`SectionShape`] と派生断面性能。
//!
//! 責務ごとにサブモジュールへ分割している。
//!
//! - [`types`] — 型定義（BarSet, ShearBar, RcRebar, RcBeamRebar, RcRectColumnRebar, RcCircleColumnRebar, RebarPoint, SectionShape, one_bar_area, bar_set_area, shear_legs_area）
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
mod shear;
mod types;

#[cfg(test)]
mod tests;

#[cfg(test)]
use crate::ids::SectionId;

pub use composite::{CftCoreProps, CompositeProps};
pub use constants::{E_STEEL, KAPPA_RC, N_S_EQ};
pub use material::{
    concrete_young_modulus, concrete_young_modulus_gamma, wall_shear_shape_factor_isection,
};
pub use shear::{
    material_strip_section_properties, strip_section_properties,
    wall_rectangular_section_properties, MaterialSectionStrip, MaterialStripSectionProperties,
    StripSectionProperties,
};
pub use types::{
    bar_set_area, one_bar_area, shear_legs_area, BarSet, BeamStirrup, CircleColumnHoop,
    RcBeamRebar, RcCircleColumnRebar, RcRebar, RcRectColumnRebar, RebarPoint, RectColumnHoop,
    SectionShape, ShearBar,
};
