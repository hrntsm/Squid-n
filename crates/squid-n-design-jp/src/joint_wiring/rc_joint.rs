//! RC 柱梁接合部（許容応力度・終局）のせん断検定配線。

use super::common::{rc_dt, MemberInfo};
use crate::rc::joint::{rc_joint_shear_check, JointShape, RcJointInput};
use crate::{CheckComponent, CheckKind, CheckResult};
use squid_n_core::ids::NodeId;
use squid_n_core::section_shape::SectionShape;

/// RC 柱梁接合部（許容応力度・終局）の検定を `out` へ追加する。
pub(super) fn check_rc_joint(
    cols: &[&MemberInfo<'_>],
    beams: &[&MemberInfo<'_>],
    nid: NodeId,
    out: &mut Vec<(NodeId, String, CheckResult)>,
) {
    let rc_col = cols.iter().find(|c| {
        matches!(c.sec.shape, Some(SectionShape::RcRect { .. })) && c.mat.fc.unwrap_or(0.0) > 0.0
    });
    let rc_beams: Vec<&&MemberInfo> = beams
        .iter()
        .filter(|b| matches!(b.sec.shape, Some(SectionShape::RcRect { .. })))
        .collect();
    if let (Some(col), false) = (rc_col, rc_beams.is_empty()) {
        let shape = match (cols.len() >= 2, rc_beams.len() >= 2) {
            (true, true) => JointShape::Cross,
            (false, true) => JointShape::Tee,
            (true, false) => JointShape::Knee,
            (false, false) => JointShape::Corner,
        };
        let Some(SectionShape::RcRect { .. }) = col.sec.shape else {
            unreachable!()
        };
        let beam0 = rc_beams[0];
        let beam_j = if let Some(SectionShape::RcRect { d, ref rebar, .. }) = beam0.sec.shape {
            7.0 / 8.0 * (d - rc_dt(rebar))
        } else {
            0.8 * beam0.sec.depth
        };
        let sum_beam_moments: f64 = rc_beams
            .iter()
            .map(|b| {
                if let Some(SectionShape::RcRect {
                    b: bw,
                    d,
                    ref rebar,
                    ..
                }) = b.sec.shape
                {
                    let at = squid_n_core::section_shape::bar_set_area(&rebar.main_x) / 2.0;
                    let dt = rc_dt(rebar);
                    let mu_inp = squid_n_core::rc_capacity::RcCapacityInput {
                        b: bw,
                        d,
                        at,
                        d_eff: d - dt,
                        sigma_y: crate::material_strength::rebar_sigma_y_of(b.rebar_mat),
                        fc: b.mat.fc.unwrap_or(0.0),
                        pw: 0.0,
                        sigma_wy: 0.0,
                        clear_span: 0.0,
                        sigma_0: 0.0,
                    };
                    squid_n_core::rc_capacity::rc_mu_simple(&mu_inp)
                } else {
                    b.end_forces(nid).map(|f| f[5].abs()).unwrap_or(0.0)
                }
            })
            .sum();
        let col_shear = cols
            .iter()
            .filter_map(|c| c.end_forces(nid))
            .map(|f| f[1].abs().max(f[2].abs()))
            .fold(0.0, f64::max);
        let col_height = cols.iter().map(|c| c.length).sum::<f64>() / cols.len() as f64;
        let beam_span = rc_beams.iter().map(|b| b.length).sum::<f64>() / rc_beams.len() as f64;
        let inp = RcJointInput {
            shape,
            fc: col.mat.fc.unwrap_or(0.0),
            col_depth: col.sec.depth,
            col_width: col.sec.width,
            beam_width: beam0.sec.width,
            beam_j,
            sum_beam_moments,
            col_shear,
            col_height,
            beam_span,
        };
        out.push((nid, "接合部(RC)".to_string(), rc_joint_shear_check(&inp)));

        let bi = (col.sec.width - beam0.sec.width) / 2.0;
        let bai = (bi / 2.0).min(col.sec.depth / 4.0).max(0.0);
        let bj = beam0.sec.width + 2.0 * bai;
        let (t_top, t_bottom) = if let Some(SectionShape::RcRect { rebar, .. }) = &beam0.sec.shape {
            let half_area = squid_n_core::section_shape::bar_set_area(&rebar.main_x) / 2.0;
            let sigma_y = crate::material_strength::rebar_sigma_y_of(beam0.rebar_mat);
            (half_area * sigma_y, half_area * sigma_y)
        } else {
            (0.0, 0.0)
        };
        let col_shears: Vec<f64> = cols
            .iter()
            .filter_map(|c| c.end_forces(nid))
            .map(|f| f[1].abs().max(f[2].abs()))
            .collect();
        let qcu = if col_shears.is_empty() {
            0.0
        } else {
            col_shears.iter().sum::<f64>() / col_shears.len() as f64
        };
        let phi = if beams.len() >= 4 { 1.0 } else { 0.85 };
        let u = crate::ultimate::rc_joint_ultimate(&crate::ultimate::RcJointUltimateInput {
            shape,
            phi,
            fc: col.mat.fc.unwrap_or(0.0),
            bj,
            dj: col.sec.depth,
            t_top,
            t_bottom,
            qcu,
            alpha: 1.0,
        });
        let ratio = if u.vju > 0.0 {
            u.qdu / u.vju
        } else {
            f64::INFINITY
        };
        out.push((
                nid,
                "接合部終局(RC)".to_string(),
                CheckResult {
                    basis: "靭性保証型指針 柱梁接合部終局(Vju=κ·φ·Fj·bj·Dj)".to_string(),
                    detail: String::new(),
                    components: vec![CheckComponent {
                        kind: CheckKind::Shear,
                        ratio,
                        detail: format!(
                            "κ={:.2}, φ={:.2}, Fj={:.3} N/mm², bj={:.1} mm, Dj={:.1} mm, \
                             Vju={:.1} N, T={:.1} N, T′={:.1} N, Qcu={:.1} N, Qdu={:.1} N, 余裕率={:.3}",
                            u.kappa, phi, u.fj, bj, col.sec.depth, u.vju, t_top, t_bottom, qcu, u.qdu, u.margin
                        ),
                    }],
                },
            ));
    }
}
