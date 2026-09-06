//! 時刻歴応答解析ジョブの純粋計算。
//!
//! - [`compute_time_history_job`] — TimeHistory ジョブの純粋計算部分。

use super::{attach_prepare_notices, model_prepared_for_analysis, JobDir, JobOutcome, JobParams};
use squid_n_core::model::Model;
use squid_n_job::{settings::ThDir, JobError};

/// TimeHistory ジョブの純粋計算部分。
/// MCP からは減衰モデル・積分法・非線形の切り替えを受け付けていない。
pub(crate) fn compute_time_history_job(
    model: &Model,
    params: &JobParams,
) -> Result<JobOutcome, JobError> {
    let (work, notices) = model_prepared_for_analysis(model, params);

    let cfg = squid_n_job::AnalysisSettings {
        th_dt: params.dt,
        th_duration: params.duration,
        th_period: params.period,
        th_amp: params.amp,
        th_dir: match params.dir {
            JobDir::X => ThDir::X,
            JobDir::Y => ThDir::Y,
        },
        ..Default::default()
    };
    let wave = squid_n_job::sample_ground_motion(&cfg);

    let result = squid_n_job::compute::compute_time_history(work, cfg, wave)?;

    let peak_disp = result
        .history
        .node_disp
        .iter()
        .fold(0.0_f64, |m, v| m.max(v.abs()));
    let mut summary = serde_json::json!({
        "kind": "TimeHistory",
        "peak_disp": peak_disp,
        "record_dir_y": result.history.record_dir_y,
        "n_steps": result.time.len(),
    });
    attach_prepare_notices(&mut summary, notices);
    Ok(JobOutcome::TimeHistory { summary })
}
