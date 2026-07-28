//! 時刻歴応答解析ジョブの純粋計算。
//!
//! - [`compute_time_history_job`] — TimeHistory ジョブの純粋計算部分。

use super::{model_with_auto_rigid_zones, JobDir, JobOutcome};
use squid_n_core::model::Model;

/// TimeHistory ジョブの純粋計算部分。
/// サンプル波の生成式は squid-n-app の `App::sample_wave`/`build_ground_motion`
/// （app.rs）と同一（squid-n-mcp は squid-n-app に依存しないため複製している）。
/// 減衰は剛性比例減衰 h=0.02（1次固有円振動数を使用）固定
/// （`App::compute_time_history` の `ThDampingModel::StiffnessProportional` 経路と同じ）。
pub(crate) fn compute_time_history_job(
    model: &Model,
    dir: JobDir,
    dt: f64,
    duration: f64,
    period: f64,
    amp: f64,
) -> Result<JobOutcome, String> {
    let model = model_with_auto_rigid_zones(model);
    let model = &model;
    let analysis = squid_n_solver::analysis::Analysis::prepare(model)
        .map_err(|e| format!("解析準備エラー: {e}"))?;

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

    let omega1 = match analysis.eigen(1) {
        Ok(modal) => match modal.omega2.first() {
            Some(&w2) if w2 > 0.0 => w2.sqrt(),
            _ => return Err("固有値が得られず減衰を設定できません。".to_string()),
        },
        Err(e) => return Err(format!("固有値解析エラー: {e}")),
    };
    let damping = squid_n_solver::damping::Damping::StiffnessProportional {
        h: 0.02,
        omega: omega1,
        basis: squid_n_solver::damping::StiffnessKind::Initial,
    };
    let newmark = squid_n_solver::timehistory::NewmarkCfg::average_accel();
    // record_every は squid-n-mcp からは未指定（None=自動決定）。UI（squid-n-app）側の
    // 設定は本ジョブでは扱っていない。
    let result = analysis
        .time_history(&wave, newmark, damping, None)
        .map_err(|e| format!("時刻歴解析エラー: {e}"))?;

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
