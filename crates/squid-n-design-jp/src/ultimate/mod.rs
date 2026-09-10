//! **終局検定（保有水平耐力計算における部材の終局強度検定）**。
//!
//! 崩壊機構形成後の各部材について、終局せん断強度・付着割裂耐力に対する
//! 余裕度（せん断・付着が曲げに先行して破壊しないこと）を検定する。
//! 両端ヒンジを仮定した終局せん断応力を設計用せん断力とする。
//!
//! 検定対象は矩形 RC 断面のみ。CFT 柱の軸終局耐力は [`cft`]、柱梁接合部の
//! 終局耐力は [`joint`] を参照。主筋は上下対称配筋を仮定する。

#[cfg(test)]
use crate::MemberKind;
#[cfg(test)]
use squid_n_core::model::Model;

pub mod cft;
pub mod cft_nm;
pub mod joint;
pub mod rc_axial;
pub mod rc_shear;
pub mod rc_shear_ductility;

mod cft_check;
mod geometry;
mod options;
mod rc_check;
mod rc_strength;

pub use cft::{
    cft_axial_ultimate, cft_column_class, cft_concrete_buckling_axial,
    cft_concrete_buckling_stress, cft_concrete_slenderness, cft_ncu1, CftAxialInput,
    CftAxialUltimate, CftColumnClass,
};
pub use cft_nm::{
    cft_long_medium_column_mu, cft_nk, cft_short_column_mu, CftBendingInput, CftLongMediumInput,
};
pub use joint::{
    joint_fj, joint_kappa, rc_joint_ultimate, RcJointUltimateInput, RcJointUltimateResult,
};
pub use rc_axial::{rc_axial_margin, rc_column_axial_ultimate, RcAxialUltimate};
pub use rc_shear::{
    bond_reliable_strength_deformed, bond_split_ratio, plastic_cot_phi, plastic_k1, plastic_k2,
    plastic_nu, plastic_nu0, rc_shear_qbu_bond, rc_shear_qsu_plastic, BondStrengthInput,
    RcBondSplitInput, RcPlasticShearInput,
};
pub use rc_shear_ductility::{
    arch_tan_theta, bond_force_tx, ductility_mu, ductility_nu, rc_shear_vbu_ductility,
    rc_shear_vu_ductility, truss_lambda, RcDuctilityShearInput, RcVbuInput,
};

pub use cft_check::{cft_mu_nm, collect_cft_ultimate_checks, CftUltimateCheck};
pub use options::{MemberDemand, ShearMethod, UltimateShearOptions};
pub use rc_check::{collect_rc_ultimate_checks, UltimateCheck};
pub use rc_strength::biaxial_margin;

#[cfg(test)]
mod tests;
