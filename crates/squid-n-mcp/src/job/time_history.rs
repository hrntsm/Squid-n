//! 時刻歴応答解析ジョブの純粋計算。
//!
//! - [`compute_time_history_job`] — TimeHistory ジョブの純粋計算部分。

use super::{model_with_auto_rigid_zones, JobDir, JobOutcome};
use squid_n_core::model::Model;
use squid_n_job::JobError;

/// TimeHistory ジョブの純粋計算部分。
/// 前処理・解析条件・解析の実体は `squid-n-job` の共通実装で、**GUI と同一**である。
/// 解析条件は `AnalysisSettings` の既定（剛性比例減衰 h=0.02・Newmark-β・線形）を
/// 用いる。MCP からは減衰モデル・積分法・非線形の切り替えを受け付けていない。
///
/// サンプル波の生成式は squid-n-app の `App::sample_wave` と同一
/// （squid-n-mcp は squid-n-app に依存しないため複製している）。
pub(crate) fn compute_time_history_job(
    model: &Model,
    dir: JobDir,
    dt: f64,
    duration: f64,
    period: f64,
    amp: f64,
) -> Result<JobOutcome, JobError> {
    let work = model_with_auto_rigid_zones(model);

    let n = ((duration / dt).ceil() as usize).max(2);
    let omega = 2.0 * std::f64::consts::PI / period.max(1e-6);
    let accel: Vec<f64> = (0..n)
        .map(|i| {
            let t = i as f64 * dt;
            amp * (omega * t).sin() * (-0.3 * t).exp()
        })
        .collect();
    let wave = match dir {
        JobDir::X => squid_n_solver::timehistory::GroundMotion {
            dt,
            accel_x: accel,
            accel_y: None,
            accel_theta: None,
        },
        JobDir::Y => {
            let n = accel.len();
            squid_n_solver::timehistory::GroundMotion {
                dt,
                accel_x: vec![0.0; n],
                accel_y: Some(accel),
                accel_theta: None,
            }
        }
    };

    // 解析条件は GUI と同じ `AnalysisSettings` で与える。既定は剛性比例減衰
    // h=0.02・Newmark-β・線形で、従来 MCP 側に固定値で書かれていた条件と一致する。
    let cfg = squid_n_job::AnalysisSettings::default();
    let result = squid_n_job::compute::compute_time_history(work, cfg, wave)?;

    let peak_disp = result
        .history
        .node_disp
        .iter()
        .fold(0.0_f64, |m, v| m.max(v.abs()));
    let summary = serde_json::json!({
        "kind": "TimeHistory",
        "peak_disp": peak_disp,
        "record_dir_y": result.history.record_dir_y,
        "n_steps": result.time.len(),
    });
    Ok(JobOutcome::TimeHistory { summary })
}
