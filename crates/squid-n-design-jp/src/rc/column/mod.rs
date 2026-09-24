//! 鉄筋コンクリート造柱の断面検定（RC 規準14条: 軸力・軸力+曲げ・せん断）。

use super::{
    axis_props_from_shape, main_rebar_grade, rc_allow, rebar_allowable_tension,
    rebar_info_from_shape, rebar_sigma_y_of, seismic_design_shear, shear_alpha, shear_capacity_for,
    shear_rebar_grade, AxisProps,
};
use crate::ultimate::rc_props::RcDirection;
use crate::{CheckComponent, CheckKind, CheckResult, DesignCtx, LoadTerm, MemberForcesAt};
use squid_n_core::model::{Material, Section};
use squid_n_core::section_shape::SectionShape;

mod nm_interaction;

pub(crate) use nm_interaction::interp_ma;
use nm_interaction::*;

/// 矩形柱の直交方向主筋総断面積 at_perp [mm²]。
///
/// 全主筋 ag から検討方向の引張側・圧縮側最外段 at・ac を除いた残り（負にならない）。
fn rect_column_at_perp(ag_mm2: f64, at_mm2: f64, ac_mm2: f64) -> f64 {
    (ag_mm2 - at_mm2 - ac_mm2).max(0.0)
}

/// 柱の断面検定（RC 規準 14条）。
pub(crate) fn column_check(
    forces: &MemberForcesAt,
    sec: &Section,
    mat: &Material,
    ctx: &DesignCtx,
    shape: &SectionShape,
    fc_raw: f64,
) -> CheckResult {
    let long_term = ctx.term == LoadTerm::Long;
    let grade = main_rebar_grade(ctx.rebar_material.as_ref());
    let allow = rc_allow(
        fc_raw,
        mat.concrete_class,
        shear_rebar_grade(ctx.shear_rebar_material.as_ref()),
        long_term,
    );

    let n_design = -forces.n;

    if let SectionShape::RcColumnCircle { d, .. } = shape {
        let damage_control = ctx.rc_damage_control;
        let d_full = *d;
        let props = axis_props_from_shape(shape, RcDirection::Strong, true)
            .expect("円形柱の断面諸元を算定できる形状のみ来る");
        let info =
            rebar_info_from_shape(shape, true).expect("円形柱の鉄筋情報を算定できる形状のみ来る");
        let ft = rebar_allowable_tension(grade, info.main_dia, long_term);

        let gross_area = std::f64::consts::PI * d_full * d_full / 4.0;
        let as_total = info.main_area;
        let na = column_axial_capacity(gross_area, as_total, allow.fc, ft, allow.n_ratio);

        let axis = ColumnAxis {
            props,
            at_perp: 0.0,
            ft,
        };
        let curve = column_nm_curve(&axis, &allow, na);
        let ma = interp_ma(&curve, n_design);

        let ratio_axial = if forces.n < 0.0 && na > 0.0 {
            (-forces.n) / na
        } else {
            0.0
        };
        let ratio_moment = if ma > 0.0 {
            (forces.mz / ma).powi(2) + (forces.my / ma).powi(2)
        } else {
            0.0
        };

        let (m_for_alpha_y, q_for_alpha_y) =
            ctx.shear_span.unwrap_or((forces.mz.abs(), forces.qy.abs()));
        let alpha_y = shear_alpha(m_for_alpha_y, q_for_alpha_y, axis.props.d, 1.5);
        let qay = shear_capacity_for(&axis.props, &allow, alpha_y, ctx.term, damage_control, true);
        let (q_design_y, q_design_z) = if ctx.seismic_qd.is_some() {
            let mu_inp = squid_n_core::rc_capacity::RcCapacityInput {
                b: gross_area / d_full,
                d: d_full,
                at: axis.props.at,
                d_eff: axis.props.d,
                sigma_y: rebar_sigma_y_of(ctx.rebar_material.as_ref()),
                fc: fc_raw,
                pw: axis.props.pw,
                sigma_wy: 0.0,
                clear_span: 0.0,
                sigma_0: 0.0,
            };
            let sum_mu_fallback =
                2.0 * squid_n_core::rc_capacity::rc_column_mu_simple(&mu_inp, as_total, n_design);
            let (sum_mu_z, sum_mu_y) = match ctx.column_sum_my {
                Some((sz, sy)) => (sz.unwrap_or(sum_mu_fallback), sy.unwrap_or(sum_mu_fallback)),
                None => (sum_mu_fallback, sum_mu_fallback),
            };
            (
                seismic_design_shear(ctx, forces.pos, forces.qy, 1, sum_mu_z, true),
                seismic_design_shear(ctx, forces.pos, forces.qz, 2, sum_mu_y, true),
            )
        } else {
            (forces.qy.abs(), forces.qz.abs())
        };
        let ratio_qy = if qay > 0.0 { q_design_y / qay } else { 0.0 };

        let (m_for_alpha_z, q_for_alpha_z) = ctx
            .shear_span_y
            .unwrap_or((forces.my.abs(), forces.qz.abs()));
        let alpha_z = shear_alpha(m_for_alpha_z, q_for_alpha_z, axis.props.d, 1.5);
        let qaz = shear_capacity_for(&axis.props, &allow, alpha_z, ctx.term, damage_control, true);
        let ratio_qz = if qaz > 0.0 { q_design_z / qaz } else { 0.0 };

        let basis = "RC 規準14条（円形柱、等価矩形近似）".to_string();
        let axial_bending_detail = format!(
            "NA={:.1} N, N={:.1} N, MA={:.1} N·mm（等価矩形近似）, mz={:.1} N·mm, my={:.1} N·mm",
            na, n_design, ma, forces.mz, forces.my,
        );
        let shear_detail = format!(
            "QAy={:.1} N, QAz={:.1} N, αy={:.3}, αz={:.3}, pw={:.5}",
            qay, qaz, alpha_y, alpha_z, axis.props.pw,
        );
        let mut detail = format!("at={:.1} mm², d={:.1} mm", axis.props.at, axis.props.d);

        let mut components = vec![
            CheckComponent {
                kind: CheckKind::AxialBending,
                ratio: ratio_axial.max(ratio_moment),
                detail: axial_bending_detail,
            },
            CheckComponent {
                kind: CheckKind::Shear,
                ratio: ratio_qy.max(ratio_qz),
                detail: shear_detail,
            },
        ];

        if crate::rc::provisions::is_member_level_station(forces.pos) {
            let prov = crate::rc::provisions::column_provisions_info(
                &rebar_info_from_shape(shape, true).unwrap(),
                d_full,
                ctx.clear_length.filter(|&l| l > 1e-9).unwrap_or(ctx.length),
                mat.concrete_class,
                long_term,
                gross_area,
                as_total,
                n_design.max(0.0),
                fc_raw,
                props.pw,
            );
            if let Some(c) = prov.provision_component() {
                components.push(c);
            }
            detail.push_str(&prov.warning_suffix());
        }

        return CheckResult {
            basis,
            detail,
            components,
        };
    }

    if let SectionShape::RcColumnRect { b, d, .. } = shape {
        let damage_control = ctx.rc_damage_control;

        let props_z = axis_props_from_shape(shape, RcDirection::Strong, true)
            .expect("矩形柱の強軸断面諸元を算定できる形状のみ来る");
        let props_y = axis_props_from_shape(shape, RcDirection::Weak, true)
            .expect("矩形柱の弱軸断面諸元を算定できる形状のみ来る");
        let info =
            rebar_info_from_shape(shape, true).expect("矩形柱の鉄筋情報を算定できる形状のみ来る");
        let ft_z = rebar_allowable_tension(grade, info.main_dia, long_term);
        let ft_y = ft_z;

        let gross_area = b * d;
        let as_total = info.main_area;
        let na = column_axial_capacity(gross_area, as_total, allow.fc, ft_z, allow.n_ratio);

        let at_perp_for_z = rect_column_at_perp(as_total, props_z.at, props_z.ac);
        let at_perp_for_y = rect_column_at_perp(as_total, props_y.at, props_y.ac);

        let axis_z = ColumnAxis {
            props: props_z,
            at_perp: at_perp_for_z,
            ft: ft_z,
        };
        let axis_y = ColumnAxis {
            props: props_y,
            at_perp: at_perp_for_y,
            ft: ft_y,
        };

        let curve_z = column_nm_curve(&axis_z, &allow, na);
        let curve_y = column_nm_curve(&axis_y, &allow, na);
        let ma_z = interp_ma(&curve_z, n_design);
        let ma_y = interp_ma(&curve_y, n_design);

        let ratio_axial = if forces.n < 0.0 && na > 0.0 {
            (-forces.n) / na
        } else {
            0.0
        };
        let ratio_z = if ma_z > 0.0 {
            forces.mz.abs() / ma_z
        } else {
            0.0
        };
        let ratio_y = if ma_y > 0.0 {
            forces.my.abs() / ma_y
        } else {
            0.0
        };
        let ratio_moment = ratio_z + ratio_y;

        let (m_for_alpha_y, q_for_alpha_y) =
            ctx.shear_span.unwrap_or((forces.mz.abs(), forces.qy.abs()));
        let alpha_y = shear_alpha(m_for_alpha_y, q_for_alpha_y, axis_z.props.d, 1.5);
        let qay = shear_capacity_for(
            &axis_z.props,
            &allow,
            alpha_y,
            ctx.term,
            damage_control,
            true,
        );
        let (q_design_y, q_design_z) = if ctx.seismic_qd.is_some() {
            let sigma_y = rebar_sigma_y_of(ctx.rebar_material.as_ref());
            let mu_of = |b_dir: f64, d_dir: f64, props: &AxisProps| {
                let mu_inp = squid_n_core::rc_capacity::RcCapacityInput {
                    b: b_dir,
                    d: d_dir,
                    at: props.at,
                    d_eff: props.d,
                    sigma_y,
                    fc: fc_raw,
                    pw: props.pw,
                    sigma_wy: 0.0,
                    clear_span: 0.0,
                    sigma_0: 0.0,
                };
                squid_n_core::rc_capacity::rc_column_mu_simple(&mu_inp, as_total, n_design)
            };
            let fallback_z = 2.0 * mu_of(*b, *d, &axis_z.props);
            let fallback_y = 2.0 * mu_of(*d, *b, &axis_y.props);
            let (sum_mu_z, sum_mu_y) = match ctx.column_sum_my {
                Some((sz, sy)) => (sz.unwrap_or(fallback_z), sy.unwrap_or(fallback_y)),
                None => (fallback_z, fallback_y),
            };
            (
                seismic_design_shear(ctx, forces.pos, forces.qy, 1, sum_mu_z, true),
                seismic_design_shear(ctx, forces.pos, forces.qz, 2, sum_mu_y, true),
            )
        } else {
            (forces.qy.abs(), forces.qz.abs())
        };
        let ratio_qy = if qay > 0.0 { q_design_y / qay } else { 0.0 };

        let (m_for_alpha_z, q_for_alpha_z) = ctx
            .shear_span_y
            .unwrap_or((forces.my.abs(), forces.qz.abs()));
        let alpha_z = shear_alpha(m_for_alpha_z, q_for_alpha_z, axis_y.props.d, 1.5);
        let qaz = shear_capacity_for(
            &axis_y.props,
            &allow,
            alpha_z,
            ctx.term,
            damage_control,
            true,
        );
        let ratio_qz = if qaz > 0.0 { q_design_z / qaz } else { 0.0 };

        let basis = "RC 規準14条（柱、軸力+二軸曲げ+せん断）".to_string();
        let axial_bending_detail = format!(
            "NA={:.1} N, N={:.1} N, MA_z={:.1} N·mm, MA_y={:.1} N·mm, mz={:.1} N·mm, my={:.1} N·mm",
            na, n_design, ma_z, ma_y, forces.mz, forces.my,
        );
        let shear_detail = format!(
            "QAy={:.1} N, QAz={:.1} N, αy={:.3}, αz={:.3}, pw_z={:.5}, pw_y={:.5}",
            qay, qaz, alpha_y, alpha_z, axis_z.props.pw, axis_y.props.pw
        );
        let mut detail = String::new();

        let mut components = vec![
            CheckComponent {
                kind: CheckKind::AxialBending,
                ratio: ratio_axial.max(ratio_moment),
                detail: axial_bending_detail,
            },
            CheckComponent {
                kind: CheckKind::Shear,
                ratio: ratio_qy.max(ratio_qz),
                detail: shear_detail,
            },
        ];

        if crate::rc::provisions::is_member_level_station(forces.pos) {
            let d_min = b.min(*d);
            let prov = crate::rc::provisions::column_provisions_info(
                &rebar_info_from_shape(shape, true).unwrap(),
                d_min,
                ctx.clear_length.filter(|&l| l > 1e-9).unwrap_or(ctx.length),
                mat.concrete_class,
                long_term,
                gross_area,
                as_total,
                n_design.max(0.0),
                fc_raw,
                props_z.pw.min(props_y.pw),
            );
            if let Some(c) = prov.provision_component() {
                components.push(c);
            }
            detail.push_str(&prov.warning_suffix());
        }

        return CheckResult {
            basis,
            detail,
            components,
        };
    }

    if let SectionShape::RcColumnCircle { d, rebar } = shape {
        let damage_control = ctx.rc_damage_control;
        let d_full = *d;
        let props = axis_props_from_shape(shape, RcDirection::Strong, true).unwrap();
        let ft = rebar_allowable_tension(grade, rebar.main_dia, long_term);

        let gross_area = std::f64::consts::PI * d_full * d_full / 4.0;
        let as_total = rebar.total_main_area();
        let na = column_axial_capacity(gross_area, as_total, allow.fc, ft, allow.n_ratio);

        let axis = ColumnAxis {
            props,
            at_perp: 0.0,
            ft,
        };
        let curve = column_nm_curve(&axis, &allow, na);
        let ma = interp_ma(&curve, n_design);

        let ratio_axial = if forces.n < 0.0 && na > 0.0 {
            (-forces.n) / na
        } else {
            0.0
        };
        let ratio_moment = if ma > 0.0 {
            (forces.mz / ma).powi(2) + (forces.my / ma).powi(2)
        } else {
            0.0
        };

        let (m_for_alpha_y, q_for_alpha_y) =
            ctx.shear_span.unwrap_or((forces.mz.abs(), forces.qy.abs()));
        let alpha_y = shear_alpha(m_for_alpha_y, q_for_alpha_y, axis.props.d, 1.5);
        let qay = shear_capacity_for(&axis.props, &allow, alpha_y, ctx.term, damage_control, true);
        let (q_design_y, q_design_z) = if ctx.seismic_qd.is_some() {
            let mu_inp = squid_n_core::rc_capacity::RcCapacityInput {
                b: gross_area / d_full,
                d: d_full,
                at: axis.props.at,
                d_eff: axis.props.d,
                sigma_y: rebar_sigma_y_of(ctx.rebar_material.as_ref()),
                fc: fc_raw,
                pw: axis.props.pw,
                sigma_wy: 0.0,
                clear_span: 0.0,
                sigma_0: 0.0,
            };
            let sum_mu_fallback =
                2.0 * squid_n_core::rc_capacity::rc_column_mu_simple(&mu_inp, as_total, n_design);
            let (sum_mu_z, sum_mu_y) = match ctx.column_sum_my {
                Some((sz, sy)) => (sz.unwrap_or(sum_mu_fallback), sy.unwrap_or(sum_mu_fallback)),
                None => (sum_mu_fallback, sum_mu_fallback),
            };
            (
                seismic_design_shear(ctx, forces.pos, forces.qy, 1, sum_mu_z, true),
                seismic_design_shear(ctx, forces.pos, forces.qz, 2, sum_mu_y, true),
            )
        } else {
            (forces.qy.abs(), forces.qz.abs())
        };
        let ratio_qy = if qay > 0.0 { q_design_y / qay } else { 0.0 };

        let (m_for_alpha_z, q_for_alpha_z) = ctx
            .shear_span_y
            .unwrap_or((forces.my.abs(), forces.qz.abs()));
        let alpha_z = shear_alpha(m_for_alpha_z, q_for_alpha_z, axis.props.d, 1.5);
        let qaz = shear_capacity_for(&axis.props, &allow, alpha_z, ctx.term, damage_control, true);
        let ratio_qz = if qaz > 0.0 { q_design_z / qaz } else { 0.0 };

        let basis = "RC 規準14条（円形柱、等価矩形近似）".to_string();
        let axial_bending_detail = format!(
            "NA={:.1} N, N={:.1} N, MA={:.1} N·mm（等価矩形近似）, mz={:.1} N·mm, my={:.1} N·mm",
            na, n_design, ma, forces.mz, forces.my,
        );
        let shear_detail = format!(
            "QAy={:.1} N, QAz={:.1} N, αy={:.3}, αz={:.3}, pw={:.5}",
            qay, qaz, alpha_y, alpha_z, axis.props.pw,
        );
        let mut detail = format!("at={:.1} mm², d={:.1} mm", axis.props.at, axis.props.d);

        let mut components = vec![
            CheckComponent {
                kind: CheckKind::AxialBending,
                ratio: ratio_axial.max(ratio_moment),
                detail: axial_bending_detail,
            },
            CheckComponent {
                kind: CheckKind::Shear,
                ratio: ratio_qy.max(ratio_qz),
                detail: shear_detail,
            },
        ];

        if crate::rc::provisions::is_member_level_station(forces.pos) {
            let prov = crate::rc::provisions::column_provisions_info(
                &rebar_info_from_shape(shape, true).unwrap(),
                d_full,
                ctx.clear_length.filter(|&l| l > 1e-9).unwrap_or(ctx.length),
                mat.concrete_class,
                long_term,
                gross_area,
                as_total,
                n_design.max(0.0),
                fc_raw,
                axis.props.pw,
            );
            if let Some(c) = prov.provision_component() {
                components.push(c);
            }
            detail.push_str(&prov.warning_suffix());
        }

        return CheckResult {
            basis,
            detail,
            components,
        };
    }

    let rebar = match shape {
        SectionShape::RcColumnRect { rebar, .. } => rebar,
        _ => unreachable!(),
    };
    let damage_control = ctx.rc_damage_control;

    let props_z = axis_props_from_shape(shape, RcDirection::Strong, true).unwrap();
    let props_y = axis_props_from_shape(shape, RcDirection::Weak, true).unwrap();
    let ft_z = rebar_allowable_tension(grade, rebar.main_dia, long_term);
    let ft_y = ft_z;

    let gross_area = sec.width * sec.depth;
    let info = rebar_info_from_shape(shape, true).unwrap();
    let as_total = info.main_area;
    let ft_axial = rebar_allowable_tension(grade, rebar.main_dia, long_term);
    let na = column_axial_capacity(gross_area, as_total, allow.fc, ft_axial, allow.n_ratio);

    let at_perp_for_z = rect_column_at_perp(as_total, props_z.at, props_z.ac);
    let at_perp_for_y = rect_column_at_perp(as_total, props_y.at, props_y.ac);

    let axis_z = ColumnAxis {
        props: props_z,
        at_perp: at_perp_for_z,
        ft: ft_z,
    };
    let axis_y = ColumnAxis {
        props: props_y,
        at_perp: at_perp_for_y,
        ft: ft_y,
    };

    let curve_z = column_nm_curve(&axis_z, &allow, na);
    let curve_y = column_nm_curve(&axis_y, &allow, na);
    let ma_z = interp_ma(&curve_z, n_design);
    let ma_y = interp_ma(&curve_y, n_design);

    let ratio_axial = if forces.n < 0.0 && na > 0.0 {
        (-forces.n) / na
    } else {
        0.0
    };
    let ratio_z = if ma_z > 0.0 {
        forces.mz.abs() / ma_z
    } else {
        0.0
    };
    let ratio_y = if ma_y > 0.0 {
        forces.my.abs() / ma_y
    } else {
        0.0
    };
    let ratio_moment = ratio_z + ratio_y;

    let (m_for_alpha_y, q_for_alpha_y) =
        ctx.shear_span.unwrap_or((forces.mz.abs(), forces.qy.abs()));
    let alpha_y = shear_alpha(m_for_alpha_y, q_for_alpha_y, axis_z.props.d, 1.5);
    let qay = shear_capacity_for(
        &axis_z.props,
        &allow,
        alpha_y,
        ctx.term,
        damage_control,
        true,
    );
    let (q_design_y, q_design_z) = if ctx.seismic_qd.is_some() {
        let sigma_y = rebar_sigma_y_of(ctx.rebar_material.as_ref());
        let mu_of = |b: f64, d_full: f64, props: &AxisProps| {
            let mu_inp = squid_n_core::rc_capacity::RcCapacityInput {
                b,
                d: d_full,
                at: props.at,
                d_eff: props.d,
                sigma_y,
                fc: fc_raw,
                pw: props.pw,
                sigma_wy: 0.0,
                clear_span: 0.0,
                sigma_0: 0.0,
            };
            squid_n_core::rc_capacity::rc_column_mu_simple(&mu_inp, as_total, n_design)
        };
        let fallback_z = 2.0 * mu_of(sec.width, sec.depth, &axis_z.props);
        let fallback_y = 2.0 * mu_of(sec.depth, sec.width, &axis_y.props);
        let (sum_mu_z, sum_mu_y) = match ctx.column_sum_my {
            Some((sz, sy)) => (sz.unwrap_or(fallback_z), sy.unwrap_or(fallback_y)),
            None => (fallback_z, fallback_y),
        };
        (
            seismic_design_shear(ctx, forces.pos, forces.qy, 1, sum_mu_z, true),
            seismic_design_shear(ctx, forces.pos, forces.qz, 2, sum_mu_y, true),
        )
    } else {
        (forces.qy.abs(), forces.qz.abs())
    };
    let ratio_qy = if qay > 0.0 { q_design_y / qay } else { 0.0 };

    let (m_for_alpha_z, q_for_alpha_z) = ctx
        .shear_span_y
        .unwrap_or((forces.my.abs(), forces.qz.abs()));
    let alpha_z = shear_alpha(m_for_alpha_z, q_for_alpha_z, axis_y.props.d, 1.5);
    let qaz = shear_capacity_for(
        &axis_y.props,
        &allow,
        alpha_z,
        ctx.term,
        damage_control,
        true,
    );
    let ratio_qz = if qaz > 0.0 { q_design_z / qaz } else { 0.0 };

    let basis = "RC 規準14条（柱、軸力+二軸曲げ+せん断）".to_string();
    let axial_bending_detail = format!(
        "NA={:.1} N, N={:.1} N, MA_z={:.1} N·mm, MA_y={:.1} N·mm, mz={:.1} N·mm, my={:.1} N·mm",
        na, n_design, ma_z, ma_y, forces.mz, forces.my,
    );
    let shear_detail = format!(
        "QAy={:.1} N, QAz={:.1} N, αy={:.3}, αz={:.3}, pw_z={:.5}, pw_y={:.5}",
        qay, qaz, alpha_y, alpha_z, axis_z.props.pw, axis_y.props.pw
    );
    let mut detail = String::new();

    let mut components = vec![
        CheckComponent {
            kind: CheckKind::AxialBending,
            ratio: ratio_axial.max(ratio_moment),
            detail: axial_bending_detail,
        },
        CheckComponent {
            kind: CheckKind::Shear,
            ratio: ratio_qy.max(ratio_qz),
            detail: shear_detail,
        },
    ];

    if crate::rc::provisions::is_member_level_station(forces.pos) {
        let d_min = sec.width.min(sec.depth);
        let prov = crate::rc::provisions::column_provisions_info(
            &info,
            d_min,
            ctx.clear_length.filter(|&l| l > 1e-9).unwrap_or(ctx.length),
            mat.concrete_class,
            long_term,
            gross_area,
            as_total,
            n_design.max(0.0),
            fc_raw,
            axis_z.props.pw.min(axis_y.props.pw),
        );
        if let Some(c) = prov.provision_component() {
            components.push(c);
        }
        detail.push_str(&prov.warning_suffix());
    }

    CheckResult {
        basis,
        detail,
        components,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rc::beam::beam_moment_capacity;
    use crate::rc::tests::{
        ctx_column, make_material, make_section, rc_column_rect_shape, rc_rect_shape,
    };
    use crate::DesignCheck;

    #[test]
    fn test_column_axial_capacity_handcalc() {
        let fc = 8.0; // 長期許容圧縮（Fc=24 なら 8.0）
        let ft = 215.0;
        let n_ratio = 15.0;
        let gross = 400.0 * 400.0;
        let as_total = 8.0 * std::f64::consts::PI * (22.0 / 2.0f64).powi(2);
        let na = column_axial_capacity(gross, as_total, fc, ft, n_ratio);

        let ae = gross + (n_ratio - 1.0) * as_total;
        let expected = (fc * ae).min(ft * ae / n_ratio);
        assert!((na - expected).abs() < 1e-6);
    }

    #[test]
    fn test_column_n0_moment_close_to_beam_ma_t() {
        // N=0 のときの柱 MA が、対応する梁の MA_t とおおむね一致すること
        // （j≒7d/8 の近似差程度、20% 程度の許容）を確認する。
        let b = 400.0;
        let d_full = 400.0;
        let shape = rc_rect_shape(b, d_full, 8, 22.0, 2, 40.0, 10.0, 100.0, 2);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let sec = make_section(shape);

        let allow = rc_allow(
            24.0,
            squid_n_core::units::ConcreteClass::Normal,
            "SD345",
            true,
        );
        let ft = rebar_allowable_tension("SD345", 22.0, true);

        let props_z = rect_axis_props_strong(&sec, &rebar);
        let bm = beam_moment_capacity(&props_z, ft, allow.fc, allow.n_ratio);

        let gross_area = sec.width * sec.depth;
        let as_total = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
        let na = column_axial_capacity(gross_area, as_total, allow.fc, ft, allow.n_ratio);

        let axis_z = ColumnAxis {
            props: props_z,
            at_perp: bar_set_area(&rebar.main_y),
            ft,
        };
        let curve = column_nm_curve(&axis_z, &allow, na);
        let ma_at_n0 = interp_ma(&curve, 0.0);

        let rel_diff = (ma_at_n0 - bm.ma).abs() / bm.ma;
        assert!(
            rel_diff < 0.2,
            "N=0 の柱 MA={ma_at_n0} が梁 MA={} と 20% 以上乖離",
            bm.ma
        );
    }

    #[test]
    fn test_column_moment_increases_then_decreases_with_compression() {
        // 軽配筋（N=0 では引張鉄筋支配）の断面を用いる。RC 規準14条の N-M
        // 相関曲線は一般に「引張支配の隅（大きな引張軸力・小さな M）→
        // 釣合点（M最大）→ 全断面圧縮の隅（N=NA, M=0）」という山型になる。
        // 釣合点（ピーク）の位置は配筋量に依存し、既に N=0 でコンクリート縁
        // 応力が支配する（過大配筋の）断面ではピークが引張側にずれることも
        // あるため、ここではピークが正の圧縮軸力側に来る軽配筋断面で検証する
        // （`test_beam_moment_heavy_reinforcement_compression_governs` が過大
        // 配筋側の挙動を別途カバーする）。
        let b = 400.0;
        let d_full = 400.0;
        let shape = rc_rect_shape(b, d_full, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let sec = make_section(shape);

        let allow = rc_allow(
            24.0,
            squid_n_core::units::ConcreteClass::Normal,
            "SD345",
            true,
        );
        let ft = rebar_allowable_tension("SD345", 19.0, true);
        let props_z = rect_axis_props_strong(&sec, &rebar);
        let gross_area = sec.width * sec.depth;
        let as_total = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
        let na = column_axial_capacity(gross_area, as_total, allow.fc, ft, allow.n_ratio);

        let axis_z = ColumnAxis {
            props: props_z,
            at_perp: bar_set_area(&rebar.main_y),
            ft,
        };
        let curve = column_nm_curve(&axis_z, &allow, na);

        let m_at_0 = interp_ma(&curve, 0.0);
        let m_at_mid = interp_ma(&curve, na * 0.3);
        let m_at_near_na = interp_ma(&curve, na * 0.98);

        assert!(
            m_at_mid > m_at_0,
            "圧縮軸力の増加で MA は一旦増加するはず: m0={m_at_0}, mid={m_at_mid}"
        );
        assert!(
            m_at_near_na < m_at_mid,
            "軸力が NA に近づくと MA は減少するはず: mid={m_at_mid}, near_na={m_at_near_na}"
        );
    }

    #[test]
    fn test_column_biaxial_linear_sum() {
        let b = 400.0;
        let d_full = 400.0;
        let shape = rc_rect_shape(b, d_full, 8, 22.0, 2, 40.0, 10.0, 100.0, 2);
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);

        // まず微小な mz を与えて ratio から MA_z を逆算する。
        let forces_z_only = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 1.0,
        };
        let design = crate::rc::RcDesign;
        let r0 = design
            .check(&forces_z_only, &sec, &mat, &ctx)
            .unwrap_checked();
        let ma_z_approx = 1.0 / r0.ratio().max(1e-30);

        let mz_test = ma_z_approx * 0.3;
        let forces = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: mz_test,
        };
        let r = design.check(&forces, &sec, &mat, &ctx).unwrap_checked();
        assert!(
            (r.ratio() - 0.3).abs() < 0.05,
            "mz 単独 0.3 割合のとき ratio ≒ 0.3 のはず: ratio={}",
            r.ratio()
        );
    }

    /// 矩形柱: components に AxialBending・Shear が含まれ、その最大値が
    /// ratio と一致することを確認する。
    #[test]
    fn test_column_check_components_axial_bending_and_shear() {
        let b = 400.0;
        let d_full = 400.0;
        let shape = rc_rect_shape(b, d_full, 8, 22.0, 2, 40.0, 10.0, 100.0, 2);
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);
        let forces = MemberForcesAt {
            pos: 0.0,
            n: -200_000.0,
            qy: 50_000.0,
            qz: 30_000.0,
            my: 10.0e6,
            mz: 20.0e6,
        };
        let design = crate::rc::RcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx).unwrap_checked();
        assert_eq!(result.components.len(), 2);
        assert!(result
            .components
            .iter()
            .any(|c| c.kind == crate::CheckKind::AxialBending));
        assert!(result
            .components
            .iter()
            .any(|c| c.kind == crate::CheckKind::Shear));
    }

    /// メカニズム方向欠落（内側 None）は外側未算定と同じく 2·Mu フォールバック。
    #[test]
    fn test_column_sum_my_missing_direction_uses_2mu_fallback() {
        use crate::{QdMethod, SeismicQd};
        let shape = rc_rect_shape(400.0, 400.0, 8, 22.0, 2, 40.0, 10.0, 100.0, 2);
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let forces = MemberForcesAt {
            pos: 0.0,
            n: -500_000.0,
            qy: 150_000.0,
            qz: 80_000.0,
            my: 40.0e6,
            mz: 60.0e6,
        };
        let mut ctx_outer_none = ctx_column(LoadTerm::Short);
        ctx_outer_none.seismic_qd = Some(SeismicQd {
            long_at: vec![(0.0, [forces.n, 40_000.0, 20_000.0, 0.0, 0.0, 0.0])],
            n_factor: 1.5,
            n_mechanism: 1.0,
            q_simple: None,
            clear_length: 3500.0,
            method: QdMethod::Qd1,
        });
        ctx_outer_none.column_sum_my = None;
        let mut ctx_dirs_missing = ctx_column(LoadTerm::Short);
        ctx_dirs_missing.seismic_qd = Some(SeismicQd {
            long_at: vec![(0.0, [forces.n, 40_000.0, 20_000.0, 0.0, 0.0, 0.0])],
            n_factor: 1.5,
            n_mechanism: 1.0,
            q_simple: None,
            clear_length: 3500.0,
            method: QdMethod::Qd1,
        });
        ctx_dirs_missing.column_sum_my = Some((None, None));

        let design = crate::rc::RcDesign;
        let r0 = design
            .check(&forces, &sec, &mat, &ctx_outer_none)
            .unwrap_checked();
        let r1 = design
            .check(&forces, &sec, &mat, &ctx_dirs_missing)
            .unwrap_checked();
        let shear = |r: &crate::CheckResult| {
            r.components
                .iter()
                .find(|c| c.kind == crate::CheckKind::Shear)
                .map(|c| c.ratio)
                .unwrap()
        };
        assert!(
            (shear(&r0) - shear(&r1)).abs() < 1e-9,
            "外側 None と内側 (None,None) は同じ 2·Mu: {} vs {}",
            shear(&r0),
            shear(&r1)
        );
    }

    /// 旧 `RcRect` 柱の検定値が、`axis_props_from_shape` 経由で諸元を引いた
    /// 場合と一致すること（新型移行後も旧柱の値を変えない回帰）。
    #[test]
    fn test_old_rect_column_check_matches_axis_props_from_shape() {
        let shape = rc_rect_shape(400.0, 400.0, 8, 22.0, 1, 40.0, 10.0, 100.0, 2);
        let sec = make_section(shape.clone());
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Short);
        let forces = MemberForcesAt {
            pos: 0.0,
            n: -300_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 40_000_000.0,
        };
        let r = crate::rc::RcDesign
            .check(&forces, &sec, &mat, &ctx)
            .unwrap_checked();

        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar,
            _ => unreachable!(),
        };
        let allow = rc_allow(
            24.0,
            squid_n_core::units::ConcreteClass::Normal,
            "SD345",
            false,
        );
        let props_z = axis_props_from_shape(&shape, RcDirection::Strong, true).unwrap();
        let ft = rebar_allowable_tension("SD345", rebar.main_x.dia, false);
        let ft_axial =
            rebar_allowable_tension("SD345", rebar.main_x.dia.max(rebar.main_y.dia), false);
        let gross_area = sec.width * sec.depth;
        let as_total = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
        let na = column_axial_capacity(gross_area, as_total, allow.fc, ft_axial, allow.n_ratio);
        let axis_z = ColumnAxis {
            props: props_z,
            at_perp: bar_set_area(&rebar.main_y),
            ft,
        };
        let ma_z = interp_ma(&column_nm_curve(&axis_z, &allow, na), -forces.n);
        let expected = ((-forces.n) / na).max(forces.mz.abs() / ma_z);

        assert!(
            (r.ratio() - expected).abs() < 1e-9,
            "旧 RcRect 柱の検定比が axis_props_from_shape 経由と一致しない: {} vs {}",
            r.ratio(),
            expected
        );
    }

    /// 新 `RcColumnRect` の許容 N-M 用 at_perp は、全主筋 ag から検討方向の
    /// 引張側・圧縮側最外段 at・ac を除いた残りであり、交点筋を二重計上しない。
    #[test]
    fn test_rc_column_rect_at_perp_excludes_intersection_bars() {
        let shape = rc_column_rect_shape();
        let rebar = match &shape {
            SectionShape::RcColumnRect { rebar, .. } => rebar,
            _ => unreachable!(),
        };
        let ag = rebar.total_main_area();
        let props_z = axis_props_from_shape(&shape, RcDirection::Strong, true).unwrap();
        let props_y = axis_props_from_shape(&shape, RcDirection::Weak, true).unwrap();

        let at_perp_z = rect_column_at_perp(ag, props_z.at, props_z.ac);
        let at_perp_y = rect_column_at_perp(ag, props_y.at, props_y.ac);

        assert!(
            (props_z.at + props_z.ac + at_perp_z - ag).abs() < 1e-9,
            "強軸: at+ac+at_perp が全主筋 ag に一致しない: {} + {} + {} != {}",
            props_z.at,
            props_z.ac,
            at_perp_z,
            ag
        );
        assert!(
            (props_y.at + props_y.ac + at_perp_y - ag).abs() < 1e-9,
            "弱軸: at+ac+at_perp が全主筋 ag に一致しない: {} + {} + {} != {}",
            props_y.at,
            props_y.ac,
            at_perp_y,
            ag
        );
        assert!(
            (at_perp_z - rebar.y_direction_area_mm2()).abs() > 1e-9,
            "強軸 at_perp が Y 方向全筋（交点筋を二重計上）になっている: {}",
            at_perp_z
        );
        assert!(
            (at_perp_y - rebar.x_direction_area_mm2()).abs() > 1e-9,
            "弱軸 at_perp が X 方向全筋（交点筋を二重計上）になっている: {}",
            at_perp_y
        );
    }
}
