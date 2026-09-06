//! 断面検定・接合部検定ジョブの純粋計算。
//!
//! - [`compute_design_check_job`] — DesignCheck ジョブの純粋計算部分。

use super::{
    attach_prepare_notices, flatten_member_force_rows, model_prepared_for_analysis,
    resolve_load_case, JobOutcome, JobParams,
};
use squid_n_core::model::{LoadCaseKind, Model};
use squid_n_design_jp::{BondMethod, LoadTerm, MemberDesignCheckOptions, QdMethod};
use squid_n_job::JobError;

/// DesignCheck ジョブの純粋計算部分。
/// 検定条件は荷重ケース種別で決める。地震時短期では重力ケースを別途解析して組合せ内力で検定する。
/// 重力ケースの再解析が一部失敗してもジョブ全体は落とさない。
pub(crate) fn compute_design_check_job(
    model: &Model,
    params: &JobParams,
) -> Result<JobOutcome, JobError> {
    let (work, notices) = model_prepared_for_analysis(model, params);
    let lc = resolve_load_case(&work, params.load_case)?;
    let lc_id = lc.id;
    let result = squid_n_job::compute::compute_linear_static(work.clone(), lc_id)?;
    let expanded_storage;
    let model: &Model = if squid_n_load::wall_expand::model_has_wall_plates_to_expand(&work) {
        let (expanded, _wall_index, _wall_report) =
            squid_n_load::wall_expand::expand_wall_elements(&work);
        expanded_storage = expanded;
        &expanded_storage
    } else {
        &work
    };
    let lc_id_u32 = lc_id.0;

    let term = match lc.kind {
        LoadCaseKind::Seismic | LoadCaseKind::Wind => LoadTerm::Short,
        _ => LoadTerm::Long,
    };

    let mut gravity_failed = 0usize;
    let (long_member_forces, q0_by_elem, check_forces) = if lc.kind == LoadCaseKind::Seismic {
        let gravity_ids = squid_n_job::gravity_case_ids_for_seismic_weight(model);
        let mut gravity_results = Vec::new();
        for gid in &gravity_ids {
            if *gid == lc_id {
                continue;
            }
            match squid_n_job::compute::compute_linear_static(work.clone(), *gid) {
                Ok(g) => gravity_results.push(g.member_forces),
                Err(_) => gravity_failed += 1,
            }
        }
        let long = if gravity_results.is_empty() {
            None
        } else {
            Some(squid_n_job::sum_member_forces_lists(&gravity_results))
        };
        let q0 = squid_n_job::simple_beam_q0_by_gravity_cases(model);
        let combo = if let Some(ref lf) = long {
            squid_n_job::sum_member_forces_lists(&[lf.clone(), result.member_forces.clone()])
        } else {
            result.member_forces.clone()
        };
        (long, q0, combo)
    } else {
        (None, Default::default(), result.member_forces.clone())
    };

    let member_force_rows = flatten_member_force_rows(&check_forces);

    let report = squid_n_design_jp::run_member_design_checks(
        model,
        &check_forces,
        &result.panel_moments,
        &MemberDesignCheckOptions {
            term,
            rc_damage_control: true,
            bond_method: BondMethod::default(),
            qd_method: QdMethod::default(),
            long_member_forces: long_member_forces.as_deref(),
            q_simple_by_elem: Some(&q0_by_elem),
            beam_group_overrides: None,
        },
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

    let mut summary = serde_json::json!({
        "kind": "DesignCheck",
        "case": lc_id_u32,
        "term": match term {
            LoadTerm::Long => "long",
            LoadTerm::Short => "short",
        },
        "n_checks": n_checks,
        "n_ng": n_ng,
        "n_skipped": n_skipped,
        "n_joint_checks": n_joint_checks,
        "n_joint_ng": n_joint_ng,
        "max_ratio": max_ratio,
        "qd_wired": long_member_forces.is_some(),
        "gravity_failed": gravity_failed,
    });
    attach_prepare_notices(&mut summary, notices);
    Ok(JobOutcome::DesignCheck {
        case: lc_id_u32,
        member_force_rows,
        summary,
    })
}
