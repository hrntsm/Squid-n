//! 弾性梁要素（剛域・端条件・SRC 等価換算を含む）。
//!
//! 責務ごとにサブモジュールへ分割する:
//! - [`element`] — データ型（`BeamElement`・`MemberForces`）
//! - [`construct`] — モデルデータからの `BeamElement` 構築
//! - [`stiffness_factors`] — スラブ協力幅・合成梁・壁エレメント上下大梁の剛性倍率
//! - [`stiffness`] — 弾性剛性行列 12×12 の構築（剛域変換・端部ばね静縮約）
//! - [`forces`] — 節点変位からの部材内力復元
//! - [`behavior`] — `ElementBehavior` トレイト実装
//! - [`linalg`] — 小行列の逆行列（汎用数値ヘルパ）
//! - [`rigid_zone`] — 剛域の自動算定

mod behavior;
mod construct;
mod element;
mod forces;
mod linalg;
mod rigid_zone;
mod stiffness;
mod stiffness_factors;

pub use element::{BeamElement, MemberForces};
pub use rigid_zone::{
    apply_auto_rigid_zones, auto_rigid_zones, recompute_auto_zones, RigidZoneRule,
};
pub use stiffness_factors::{
    composite_props_of, stiffness_breakdown, StiffnessBreakdown, WALL_GIRDER_STIFF_FACTOR,
};

pub(crate) use linalg::invert_small;

#[cfg(test)]
mod tests;
