//! 増分解析（プッシュオーバー解析）ジョブの純粋計算。
//!
//! - [`compute_pushover_job`] — Pushover ジョブの純粋計算部分。

use super::{JobDir, JobOutcome};
use squid_n_core::model::Model;
use squid_n_solver::pushover::PushoverTarget;

/// Pushover ジョブの純粋計算部分。
/// squid-n-app の `App::compute_pushover`（app.rs）と同じ流れ
/// （squid-n-mcp は squid-n-app に依存しないため複製している）。
/// モデルは所有権を取って複製したものを渡す前提
/// （増分解析は非線形状態を模型に書き戻すため）。
pub(crate) fn compute_pushover_job(
    model: Model,
    dir: JobDir,
    steps: usize,
    target: PushoverTarget,
) -> Result<JobOutcome, String> {
    let mut work = model;
    // 解析前に剛域を自動算定してモデルへ反映する（設計書 §6.2.1、標準実装）。
    squid_n_element::beam::apply_auto_rigid_zones(
        &mut work,
        &squid_n_element::beam::RigidZoneRule::default(),
    );
    squid_n_solver::analysis::Analysis::prepare(&work)
        .map_err(|e| format!("解析準備エラー: {e}"))?;
    let dofmap = squid_n_core::dof::DofMap::build(&work);
    let reducer = squid_n_solver::constraint::Reducer::build(&work, &dofmap);
    let seismic_dir = match dir {
        JobDir::X => squid_n_solver::analysis::SeismicDir::X,
        JobDir::Y => squid_n_solver::analysis::SeismicDir::Y,
    };
    let result = squid_n_solver::pushover::pushover_analysis_recording(
        &work,
        &dofmap,
        &reducer,
        seismic_dir,
        steps,
        target,
        squid_n_solver::pushover::PushoverControl::default(),
        true,
        false,
        false,
        0.0,
        squid_n_solver::pushover::DuctilityMethod::default(),
    )
    .map_err(|e| format!("増分解析エラー: {e}"))?;

    let mechanism = match result.mechanism {
        squid_n_solver::pushover::MechanismType::Overall => "Overall".to_string(),
        squid_n_solver::pushover::MechanismType::StoryCollapse { story } => {
            format!("StoryCollapse(story={})", story.0)
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
