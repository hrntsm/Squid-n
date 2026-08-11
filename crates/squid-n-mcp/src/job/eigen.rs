//! 固有値解析ジョブの純粋計算。
//!
//! - [`compute_eigen_job`] — Eigen ジョブの純粋計算部分。

use super::{model_with_auto_rigid_zones, JobOutcome};
use squid_n_core::model::Model;

/// Eigen ジョブの純粋計算部分。
pub(crate) fn compute_eigen_job(model: &Model, n_modes: usize) -> Result<JobOutcome, String> {
    let model = model_with_auto_rigid_zones(model);
    // 解析の実体は GUI と共通（`squid-n-job`）。
    let modal = squid_n_job::compute::compute_eigen(model, n_modes).map_err(|e| e.to_string())?;
    let summary = serde_json::json!({
        "kind": "Eigen",
        "n_modes": modal.period.len(),
        "period": modal.period,
    });
    Ok(JobOutcome::Eigen {
        period: modal.period,
        omega2: modal.omega2,
        participation: modal.participation,
        effective_mass: modal.effective_mass,
        summary,
    })
}
