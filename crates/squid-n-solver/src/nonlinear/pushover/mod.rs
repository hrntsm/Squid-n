//! プッシュオーバー解析（P5 §7）。責務ごとにサブモジュールへ分割する。
//!
//! - [`types`] — 解析結果・イベントの型定義
//! - [`assembly`] — 剛性行列の組立・内力ベクトルの算定（`dynamic`/`timehistory` 共有）
//! - [`diagnosis`] — 接線剛性が特異・非正定値になったときの診断メッセージ
//! - [`response`] — ベースシア・層せん断・層間変位・屋根変位の算定
//! - [`driver`] — 荷重制御・変位制御・弧長法を統括する司令塔
//! - [`hinge`] — 曲げヒンジの閾値算定と発生追跡
//! - [`ductility`] — 部材塑性率の追跡
//! - [`shear_yield`] — せん断降伏耐力 Qy の算定と降伏イベント追跡
//! - [`member_response`] — 終局時の部材別応答の算定
//! - [`mechanism`] — 崩壊機構の判定
//! - [`geom`] — 幾何ヘルパ（内積・軸圧縮力）

mod assembly;
mod diagnosis;
mod driver;
mod ductility;
mod geom;
mod hinge;
mod mechanism;
pub(crate) mod member_response;
mod response;
mod shear_yield;
mod types;

pub use driver::{pushover_analysis, pushover_analysis_recording};
pub use response::story_reference_node;
pub use types::{
    CapacityPoint, DuctilityMethod, HingeEvent, HingeLevel, MechanismType, MemberHistory,
    MemberStepState, PushoverControl, PushoverMemberResponse, PushoverResult, PushoverStep,
    PushoverTarget, PushoverTermination, ShearYieldEvent,
};

// dynamic/timehistory が `crate::nonlinear::pushover::{add_support_spring_f_int, assemble_k,
// assemble_k_cached_ref, compute_f_int}` で参照する。
pub(crate) use assembly::{
    add_support_spring_f_int, assemble_k, assemble_k_cached_ref, compute_f_int,
};

// tests（`use super::*`）が参照する非公開項目・外部名を供給する
// （非テストビルドでは持ち込まない）。
#[cfg(test)]
use crate::statics::analysis::SeismicDir;
#[cfg(test)]
use geom::axial_compression;
#[cfg(test)]
use hinge::compute_hinge_thresholds;
#[cfg(test)]
use mechanism::{compute_static_indeterminacy, determine_mechanism};
#[cfg(test)]
use shear_yield::{
    compute_shear_yield_qy, compute_shear_yield_thresholds, effective_clear_span,
    track_shear_yield, DirThreshold, SecMaterials, ShearDir,
};
#[cfg(test)]
use smallvec::SmallVec;
#[cfg(test)]
use squid_n_core::model::{Model, RigidZone};
#[cfg(test)]
use squid_n_core::rc_capacity::{rc_mu_simple, rc_qsu_simple, RcCapacityInput};
#[cfg(test)]
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape};
#[cfg(test)]
use squid_n_element::behavior::{Ctx, ElementBehavior, LocalVec};

#[cfg(test)]
mod tests;
