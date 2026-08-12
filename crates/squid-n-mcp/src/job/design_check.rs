//! 断面検定・接合部検定ジョブの純粋計算。
//!
//! - [`compute_design_check_job`] — DesignCheck ジョブの純粋計算部分。

use super::{
    flatten_member_force_rows, model_prepared_for_analysis, resolve_load_case, JobOutcome,
};
use squid_n_core::model::{LoadCaseKind, Model};
use squid_n_design_jp::{LoadTerm, MemberDesignCheckOptions};
use squid_n_job::JobError;

/// DesignCheck ジョブの純粋計算部分。
/// 指定/先頭の荷重ケースで線形静的解析を行い、断面力に対して
/// [`squid_n_design_jp::run_member_design_checks`] による許容応力度検定を行う。
///
/// 検定条件は荷重ケース種別から決める（`Seismic`/`Wind` → 短期、それ以外 → 長期）。
/// 地震時短期では重力ケースを別途解析して QD / 柱メカニズム用の長期内力と Q0 を渡す。
pub(crate) fn compute_design_check_job(
    model: &Model,
    load_case: Option<u32>,
) -> Result<JobOutcome, JobError> {
    // 剛域自動算定は face_i/face_j による危険断面位置（§6.2.3）の算定にも使う。
    let work = model_prepared_for_analysis(model);
    let lc = resolve_load_case(&work, load_case)?;
    let lc_id = lc.id;
    // 解析の実体は GUI と共通（`squid-n-job`）。エラー文言も共通の `JobError`。
    let result = squid_n_job::compute::compute_linear_static(work.clone(), lc_id)?;
    let model = &work;
    let lc_id_u32 = lc_id.0;

    let term = match lc.kind {
        LoadCaseKind::Seismic | LoadCaseKind::Wind => LoadTerm::Short,
        _ => LoadTerm::Long,
    };

    // 地震時短期: 重力ケースを線形解析して長期内力を重ね、Q0 も算定する。
    // 風時短期は QD 割増の対象外（長期内力は渡さない）。
    let (long_member_forces, q0_by_elem) = if lc.kind == LoadCaseKind::Seismic {
        let gravity_ids = squid_n_job::gravity_case_ids_for_seismic_weight(model);
        let mut gravity_results = Vec::new();
        for gid in &gravity_ids {
            if *gid == lc_id {
                // 対象ケース自身が重力のときは、その結果を長期として流用する。
                gravity_results.push(result.member_forces.clone());
                continue;
            }
            let g = squid_n_job::compute::compute_linear_static(work.clone(), *gid)?;
            gravity_results.push(g.member_forces);
        }
        let long = if gravity_results.is_empty() {
            None
        } else {
            Some(squid_n_job::sum_member_forces_lists(&gravity_results))
        };
        let q0 = squid_n_job::simple_beam_q0_by_gravity_cases(model);
        (long, q0)
    } else {
        (None, Default::default())
    };

    let member_force_rows = flatten_member_force_rows(&result.member_forces);

    let report = squid_n_design_jp::run_member_design_checks(
        model,
        &result.member_forces,
        &result.panel_moments,
        &MemberDesignCheckOptions {
            term,
            rc_damage_control: true,
            bond_method: Default::default(),
            qd_method: Default::default(),
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

    let summary = serde_json::json!({
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
    });
    Ok(JobOutcome::DesignCheck {
        case: lc_id_u32,
        member_force_rows,
        summary,
    })
}
