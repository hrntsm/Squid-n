//! 線形静的解析ジョブの純粋計算。
//!
//! - [`compute_linear_static_job`] — LinearStatic ジョブの純粋計算部分。

use super::{
    flatten_member_force_rows, model_prepared_for_analysis, resolve_load_case, JobOutcome,
};
use squid_n_core::model::Model;
use squid_n_job::JobError;

/// LinearStatic ジョブの純粋計算部分。
pub(crate) fn compute_linear_static_job(
    model: &Model,
    load_case: Option<u32>,
) -> Result<JobOutcome, JobError> {
    let work = model_prepared_for_analysis(model);
    let lc_id = resolve_load_case(&work, load_case)?.id;
    // 解析の実体は GUI と共通（`squid-n-job`）。
    let result = squid_n_job::compute::compute_linear_static(work.clone(), lc_id)?;
    let model = &work;
    let lc_id = lc_id.0;

    let node_ids: Vec<u32> = model.nodes.iter().map(|n| n.id.0).collect();
    let member_force_rows = flatten_member_force_rows(&result.member_forces);
    let max_abs_disp = result
        .disp
        .iter()
        .flat_map(|d| d.iter())
        .fold(0.0_f64, |m, v| m.max(v.abs()));

    let summary = serde_json::json!({
        "kind": "LinearStatic",
        "case": lc_id,
        "n_nodes": node_ids.len(),
        "n_member_force_rows": member_force_rows.len(),
        "max_abs_disp": max_abs_disp,
    });
    Ok(JobOutcome::LinearStatic {
        case: lc_id,
        node_ids,
        disp: result.disp,
        member_force_rows,
        summary,
    })
}
