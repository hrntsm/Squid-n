//! 断面検定・接合部検定ジョブの純粋計算。
//!
//! - [`compute_design_check_job`] — DesignCheck ジョブの純粋計算部分。

use super::{
    flatten_member_force_rows, model_prepared_for_analysis, resolve_load_case, JobOutcome,
};
use squid_n_core::model::Model;
use squid_n_job::JobError;

/// DesignCheck ジョブの純粋計算部分。
/// 指定/先頭の荷重ケースで線形静的解析を行い、断面力に対して
/// [`squid_n_design_jp::run_member_design_checks`] による許容応力度検定を行う。
/// 検定条件（長期/短期）は既定で長期（`LoadTerm::Long`）とする。
pub(crate) fn compute_design_check_job(
    model: &Model,
    load_case: Option<u32>,
) -> Result<JobOutcome, JobError> {
    // 剛域自動算定は face_i/face_j による危険断面位置（§6.2.3）の算定にも使う。
    let work = model_prepared_for_analysis(model);
    let lc_id = resolve_load_case(&work, load_case)?.id;
    // 解析の実体は GUI と共通（`squid-n-job`）。エラー文言も共通の `JobError`。
    let result = squid_n_job::compute::compute_linear_static(work.clone(), lc_id)?;
    let model = &work;
    let lc_id = lc_id.0;

    let member_force_rows = flatten_member_force_rows(&result.member_forces);

    let report = squid_n_design_jp::run_member_design_checks(
        model,
        &result.member_forces,
        &result.panel_moments,
        &squid_n_design_jp::MemberDesignCheckOptions::default(),
    );

    let mut n_checks = 0usize;
    let mut n_ng = 0usize;
    let mut n_skipped = 0usize;
    let mut max_ratio = 0.0_f64;

    for (_, _, outcome) in &report.member_checks {
        n_checks += 1;
        match outcome {
            squid_n_design_jp::CheckOutcome::Checked(cr) => {
                if !cr.ok() {
                    n_ng += 1;
                }
                if cr.ratio() > max_ratio {
                    max_ratio = cr.ratio();
                }
            }
            squid_n_design_jp::CheckOutcome::Skipped { .. } => {
                n_skipped += 1;
            }
        }
    }

    let n_joint_checks = report.joint_checks.len();
    let n_joint_ng = report
        .joint_checks
        .iter()
        .filter(|(_, _, cr)| !cr.ok())
        .count();
    for (_, _, cr) in &report.joint_checks {
        if cr.ratio() > max_ratio {
            max_ratio = cr.ratio();
        }
    }

    let summary = serde_json::json!({
        "kind": "DesignCheck",
        "case": lc_id,
        "n_checks": n_checks,
        "n_ng": n_ng,
        "n_skipped": n_skipped,
        "n_joint_checks": n_joint_checks,
        "n_joint_ng": n_joint_ng,
        "max_ratio": max_ratio,
    });
    Ok(JobOutcome::DesignCheck {
        case: lc_id,
        member_force_rows,
        summary,
    })
}
