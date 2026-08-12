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
/// 指定/先頭の荷重ケースで線形静的解析を行い、断面力に対して
/// [`squid_n_design_jp::run_member_design_checks`] による許容応力度検定を行う。
///
/// 検定条件は荷重ケース種別から決める（`Seismic`/`Wind` → 短期、それ以外 → 長期）。
/// 地震時短期では重力ケースを別途解析し、長期内力と地震ケース内力を線形加算した
/// 組合せ内力で検定する（GUI の `DL+LL±EX` に相当）。QD / 柱メカニズム用の
/// 長期内力と Q0 も同時に渡す。
///
/// 重力ケースの再解析が一部失敗してもジョブ全体は落とさず、成功分だけで長期を組む
/// （`summary.gravity_failed` に失敗件数を載せる）。
pub(crate) fn compute_design_check_job(
    model: &Model,
    params: &JobParams,
) -> Result<JobOutcome, JobError> {
    // 剛域自動算定は face_i/face_j による危険断面位置（§6.2.3）の算定にも使う。
    let (work, notices) = model_prepared_for_analysis(model, params);
    let lc = resolve_load_case(&work, params.load_case)?;
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
    let mut gravity_failed = 0usize;
    let (long_member_forces, q0_by_elem, check_forces) = if lc.kind == LoadCaseKind::Seismic {
        let gravity_ids = squid_n_job::gravity_case_ids_for_seismic_weight(model);
        let mut gravity_results = Vec::new();
        for gid in &gravity_ids {
            // 重力 ID が対象 Seismic と一致することは通常ない（到達時はスキップ）。
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
        // 検定に渡す内力は「長期 ⊕ 地震」の組合せ（QE = |Q − QL| が成立するため）。
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

    // MCP は GUI の analysis_cfg を持たないため、付着・QD 方式はクレート既定
    // （BondMethod::Rc1999、QdMethod 既定）を明示する。一本部材は未配線。
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
