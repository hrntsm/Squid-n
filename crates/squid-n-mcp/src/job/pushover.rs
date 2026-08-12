//! 増分解析（プッシュオーバー解析）ジョブの純粋計算。
//!
//! - [`compute_pushover_job`] — Pushover ジョブの純粋計算部分。

use super::{JobDir, JobOutcome};
use squid_n_core::model::Model;
use squid_n_job::JobError;
use squid_n_solver::pushover::PushoverTarget;

/// Pushover ジョブの純粋計算部分。
/// 前処理・解析条件・純粋計算はいずれも `squid-n-job` の共通実装で、
/// **GUI と同一**である。モデルは所有権を取って複製したものを渡す前提
/// （増分解析は非線形状態を模型に書き戻すため）。
pub(crate) fn compute_pushover_job(
    model: Model,
    dir: JobDir,
    steps: usize,
    target: PushoverTarget,
) -> Result<JobOutcome, JobError> {
    // 解析前処理（剛域＋仕口パネル＋荷重自動同期）は GUI と同一の実装を通す。
    let mut work = model;
    let settings = squid_n_job::AnalysisSettings::default();
    squid_n_job::prepare::prepare_model_for_analysis(&mut work, &settings, None);
    // 解析条件は GUI と同じ `AnalysisSettings` を組み立てて共通の純粋計算へ渡す。
    let cfg = squid_n_job::AnalysisSettings {
        push_dir: match dir {
            JobDir::X => squid_n_solver::analysis::SeismicDir::X,
            JobDir::Y => squid_n_solver::analysis::SeismicDir::Y,
        },
        push_steps: steps,
        push_use_max_disp: target.max_disp.is_some(),
        push_max_disp: target.max_disp.unwrap_or_default(),
        push_use_drift_angle: target.max_drift_angle.is_some(),
        push_drift_denom: target
            .max_drift_angle
            .map(|a| 1.0 / a.max(f64::MIN_POSITIVE))
            .unwrap_or(200.0),
        ..Default::default()
    };
    let result = squid_n_job::compute::compute_pushover(work, cfg)?;

    let mechanism = match result.mechanism {
        squid_n_solver::pushover::MechanismType::Overall => "Overall".to_string(),
        squid_n_solver::pushover::MechanismType::StoryCollapse { layer } => {
            format!("StoryCollapse(layer={layer})")
        }
        squid_n_solver::pushover::MechanismType::Partial => "Partial".to_string(),
    };
    // qu は N 単位（squid_n_solver::pushover::PushoverResult）。GUI(app.rs/summary.rs)と
    // 同様に kN 表示にするため /1000.0 する。
    let summary = serde_json::json!({
        "kind": "Pushover",
        "qu_kN": result.qu / 1000.0,
        "mechanism": mechanism,
        "n_steps": result.steps.len(),
    });
    Ok(JobOutcome::Pushover { summary })
}
