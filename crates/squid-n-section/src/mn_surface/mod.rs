//! 3次元 M-N 相関曲面（降伏曲面）の算定。
//! 単位: 長さ [mm], 応力 [N/mm²], 軸力 [N], モーメント [N·mm]。

pub mod fibers;
pub mod m_phi;
pub mod plastic;
pub mod surface;
pub mod types;

pub use fibers::{max_dimension, plastic_fibers, plastic_fibers_at, AnnulusRes};
pub use m_phi::{m_phi_curve, m_theta_curve, MPhiCurve};
pub use plastic::{axial_capacity, plastic_moment_at_n, plastic_point, slice_at_n};
pub use surface::{build_simple_spring_surface, build_surface, MnSurface};
pub use types::{concrete_young, FiberRegion, PlasticFiber, StrengthParams, YieldModelKind};

#[cfg(test)]
mod tests;
