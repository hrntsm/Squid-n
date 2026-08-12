//! **終局検定（保有水平耐力計算における部材の終局強度検定）**。
//!
//! 非線形解析（荷重増分解析）で崩壊機構が形成された後、各部材が終局せん断強度・
//! 付着割裂耐力に対して十分な余裕（せん断・付着が曲げに先行して破壊しないこと）を
//! 持つかを検定する。本実装で「終局強度型設計指針」を選択した場合の
//! 塑性理論式（[`rc_shear`]）と、柱の軸終局耐力（[`rc_axial`]）を実装する。
//!
//! # 検定の考え方（採用応力・RC 柱の余裕度）
//! - 両端ヒンジを仮定した終局せん断応力 `Qmu = 上限強度倍率·(Mu上+Mu下)/内法` を
//!   設計用せん断力とし、終局せん断強度 `Qsu`（塑性理論式）・付着割裂耐力 `Qbu`
//!   との比（余裕度 `Qsu/Qmu`, `Qbu/Qmu`）を算定する。
//! - 余裕度 ≥ 1.0（せん断・付着が曲げ降伏に先行しない）で OK。
//!
//! # 曲げ終局強度 Mu
//! 梁は [`squid_n_core::rc_capacity::rc_mu_simple`]（構造規定 at 式）を用いる。
//! 柱は軸力を考慮した構造規定 at 式
//! （[`squid_n_core::rc_capacity::rc_column_mu_simple`]）を用いる。
//!
//! # 適用範囲・簡略化（doc 兼申し送り）
//! - RC 部材の検定対象は `SectionShape::RcRect`（矩形 RC 断面）のみ。円形柱・SRC・鋼は
//!   別途（本モジュールの RC 経路の対象外）。CFT 柱の軸終局耐力は [`cft`]、柱梁接合部の
//!   終局耐力は [`joint`] を参照。
//! - せん断・付着は強軸（せい方向主筋 main_x）を基本とし、柱は指定により 2 軸せん断
//!   （[`biaxial_margin`]）も検定できる。終局せん断強度は [`ShearMethod`] により
//!   塑性理論式（[`rc_shear`]）または靭性指針式 Vu（[`rc_shear_ductility`]）を選択できる。
//! - 主筋は上下対称配筋を仮定し、引張側主筋量は main_x の総断面積の半分とする。

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
mod rc_section;
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
