//! 解析ジョブ（モデルの解析前処理・解析条件・各解析の純粋計算）。
//! GUI（`squid-n-app`）と MCP サーバ（`squid-n-mcp`）の共通下層。
//!
//! - [`auto_loads`] — 荷重ケースの自動生成（DL/LL/EX/EY。モデルは書き換えない）
//! - [`prepare`] — 解析前処理（剛域・仕口パネル・荷重ケースの自動同期）
//! - [`settings`] — 解析条件（[`settings::AnalysisSettings`]）
//! - [`compute`] — 各解析の純粋計算（所有モデル＋解析条件 → 結果）
//! - [`sample_wave`] — 正弦減衰のサンプル地震波（[`sample_wave::sample_ground_motion`]）
//! - [`ultimate_demand`] — 終局検定用の部材需要組み立て（[`ultimate_demand::member_demand_from_static_forces`] 等）
//! - [`error`] — ジョブのエラー型（[`error::JobError`]）
//!
//! 結果の整形（MCP の JSON・GUI の表示）は各クレートに残す。

pub mod auto_loads;
pub mod compute;
pub mod design_q0;
pub mod error;
pub mod lumped_mass;
pub mod prepare;
pub mod sample_wave;
pub mod settings;
pub mod ultimate_demand;

pub use auto_loads::{
    apply_auto_load_cases, compute_auto_load_cases, compute_dl_beam_loads,
    compute_gravity_auto_load_cases, compute_seismic_auto_load_cases, AutoLoadCaseContent,
    AutoLoadComputeResult,
};
pub use design_q0::{
    gravity_case_ids_for_seismic_weight, simple_beam_q0_by_elem, simple_beam_q0_by_gravity_cases,
    sum_analyzed_gravity_member_forces, sum_member_forces_lists,
};
pub use error::{JobError, JobResult};
pub use lumped_mass::{build_lumped_mass, LumpedMassBuildInput};
pub use prepare::{apply_rigid_zones_and_panels, prepare_model_for_analysis, PrepareReport};
pub use sample_wave::{
    build_ground_motion, lumped_accel_from_wave, sample_ground_motion, sample_lumped_ground_motion,
};
pub use settings::AnalysisSettings;
pub use ultimate_demand::{
    member_demand_from_pushover, member_demand_from_static_forces, q_long_map_from_member_forces,
};
