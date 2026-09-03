//! 増分解析（プッシュオーバー解析）ジョブの純粋計算。
//!
//! - [`compute_pushover_job`] — Pushover ジョブの純粋計算部分。

use super::{attach_prepare_notices, JobDir, JobOutcome, JobParams};
use squid_n_core::model::Model;
use squid_n_core::units::to_display::force_kn;
use squid_n_job::JobError;

/// Pushover ジョブの純粋計算部分。
/// 前処理・解析条件・純粋計算はいずれも `squid-n-job` の共通実装で、
/// **GUI と同一**である。モデルは所有権を取って複製したものを渡す前提
/// （増分解析は非線形状態を模型に書き戻すため）。
///
/// なお増分解析の水平載荷パターン（Ai 分布）は solver 側で略算 T・Z=1・地盤 II・
/// C0=0.2 に固定されている。`z`/`soil`/`c0`/`ai_mode`/`design_period` は解析前の
/// 荷重自動同期（階重量・EX/EY）に効くが、載荷 Ai そのものには使われない。
pub(crate) fn compute_pushover_job(
    model: Model,
    params: &JobParams,
) -> Result<JobOutcome, JobError> {
    // 解析前処理（剛域＋仕口パネル＋荷重自動同期）は GUI と同一の実装を通す。
    let mut work = model;
    let prepare_settings = params.analysis_settings_for_prepare();
    let prepare_report = squid_n_job::prepare::prepare_model_for_analysis(
        &mut work,
        &prepare_settings,
        params.design_period,
    );
    // 解析条件は GUI と同じ `AnalysisSettings` を組み立てて共通の純粋計算へ渡す。
    let target = params.pushover_target();
    let cfg = squid_n_job::AnalysisSettings {
        push_dir: match params.dir {
            JobDir::X => squid_n_solver::statics::analysis::SeismicDir::X,
            JobDir::Y => squid_n_solver::statics::analysis::SeismicDir::Y,
        },
        push_steps: params.steps,
        push_use_max_disp: target.max_disp.is_some(),
        push_max_disp: target.max_disp.unwrap_or_default(),
        push_use_drift_angle: target.max_drift_angle.is_some(),
        push_drift_denom: target
            .max_drift_angle
            .map(|a| 1.0 / a.max(f64::MIN_POSITIVE))
            .unwrap_or(200.0),
        ..prepare_settings
    };
    let result = squid_n_job::compute::compute_pushover(work, cfg)?;

    let mechanism = match result.mechanism {
        squid_n_solver::nonlinear::pushover::MechanismType::Overall => "Overall".to_string(),
        squid_n_solver::nonlinear::pushover::MechanismType::StoryCollapse { layer } => {
            format!("StoryCollapse(layer={layer})")
        }
        squid_n_solver::nonlinear::pushover::MechanismType::Partial => "Partial".to_string(),
    };
    let mut summary = serde_json::json!({
        "kind": "Pushover",
        "qu_kN": force_kn(result.qu),
        "mechanism": mechanism,
        "n_steps": result.steps.len(),
    });
    attach_prepare_notices(&mut summary, prepare_report.notices);
    Ok(JobOutcome::Pushover { summary })
}
