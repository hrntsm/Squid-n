//! 固有値解析ジョブの純粋計算。
//!
//! - [`compute_eigen_job`] — Eigen ジョブの純粋計算部分。

use super::{attach_prepare_notices, model_prepared_for_analysis, JobOutcome, JobParams};
use squid_n_core::model::Model;
use squid_n_job::JobError;

/// Eigen ジョブの純粋計算部分。
pub(crate) fn compute_eigen_job(model: &Model, params: &JobParams) -> Result<JobOutcome, JobError> {
    let (model, notices) = model_prepared_for_analysis(model, params);
    // 解析の実体は GUI と共通（`squid-n-job`）。
    let modal = squid_n_job::compute::compute_eigen(model, params.n_modes)?;
    let mut summary = serde_json::json!({
        "kind": "Eigen",
        "n_modes": modal.period.len(),
        "period": modal.period,
    });
    attach_prepare_notices(&mut summary, notices);
    Ok(JobOutcome::Eigen {
        period: modal.period,
        omega2: modal.omega2,
        participation: modal.participation,
        effective_mass: modal.effective_mass,
        summary,
    })
}
