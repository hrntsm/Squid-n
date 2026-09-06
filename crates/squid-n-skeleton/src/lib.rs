//! 部材スケルトン曲線（トリリニア）の自動算定。

mod builder;
mod deformation;
mod fiber_model;
mod types;

pub use builder::{build_member_skeleton, build_rc_member_skeleton};
pub use deformation::{PulloutContribution, ShearContribution};
pub use types::{AxialInteraction, MemberData, MemberSkeleton, Reinforcement, SkeletonOptions};

#[cfg(test)]
mod tests;
