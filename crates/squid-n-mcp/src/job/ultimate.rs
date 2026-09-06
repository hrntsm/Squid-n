//! 終局検定ジョブの純粋計算。
//!
//! - [`compute_ultimate_check_job`] — 終局検定ジョブ（靭性保証型耐震設計指針）。

use super::{
    attach_prepare_notices, model_prepared_for_analysis, resolve_load_case, JobOutcome, JobParams,
};
use squid_n_core::model::Model;
use squid_n_job::JobError;

/// 終局検定ジョブ（靭性保証型耐震設計指針）。
/// 設計軸力は `load_case`（未指定なら先頭ケース）の線形静的解析の軸力を用いる。
pub(crate) fn compute_ultimate_check_job(
    model: &Model,
    params: &JobParams,
) -> Result<JobOutcome, JobError> {
    let (work, notices) = model_prepared_for_analysis(model, params);
    let lc_id = resolve_load_case(&work, params.load_case)?.id;
    let result = squid_n_job::compute::compute_linear_static(work.clone(), lc_id)?;
    let model = &work;
    let lc_id = lc_id.0;

    let demand = squid_n_job::member_demand_from_static_forces(&result.member_forces, None, None);
    let axial: Vec<(squid_n_core::ids::ElemId, f64)> =
        demand.iter().map(|(id, d)| (*id, d.n_axial)).collect();

    let opts = squid_n_design_jp::ultimate::UltimateShearOptions::default();
    let checks = squid_n_design_jp::ultimate::collect_rc_ultimate_checks(model, &demand, &opts);
    let cft_checks = squid_n_design_jp::ultimate::collect_cft_ultimate_checks(model, &axial);

    let n_checks = checks.len();
    let n_ng = checks.iter().filter(|c| !c.ok).count();
    let min_shear_margin = checks
        .iter()
        .map(|c| c.shear_margin)
        .filter(|m| m.is_finite())
        .fold(f64::INFINITY, f64::min);
    let members: Vec<serde_json::Value> = checks
        .iter()
        .map(|c| {
            serde_json::json!({
                "elem": c.elem.0,
                "kind": format!("{:?}", c.kind),
                "mu": c.mu,
                "qmu": c.qmu,
                "qsu": c.qsu,
                "qbu": c.qbu,
                "shear_margin": c.shear_margin,
                "bond_margin": c.bond_margin,
                "ok": c.ok,
            })
        })
        .collect();

    let cft_members: Vec<serde_json::Value> = cft_checks
        .iter()
        .map(|c| {
            serde_json::json!({
                "elem": c.elem.0,
                "class": format!("{:?}", c.class),
                "ncu": c.ncu,
                "ntu": c.ntu,
                "mu_nm": c.mu_nm,
                "n_design": c.n_design,
                "axial_margin": c.axial_margin,
                "ok": c.ok,
            })
        })
        .collect();

    let mut summary = serde_json::json!({
        "kind": "UltimateCheck",
        "case": lc_id,
        "n_checks": n_checks,
        "n_ng": n_ng,
        "min_shear_margin": if min_shear_margin.is_finite() { serde_json::json!(min_shear_margin) } else { serde_json::Value::Null },
        "members": members,
        "n_cft_checks": cft_checks.len(),
        "n_cft_ng": cft_checks.iter().filter(|c| !c.ok).count(),
        "cft_members": cft_members,
    });
    attach_prepare_notices(&mut summary, notices);
    Ok(JobOutcome::UltimateCheck { summary })
}
