//! 柱の N-M 相関カーネル（RC 規準 14条: 軸力・軸力+曲げ耐力）。

use crate::rc::{AxisProps, RcAllow};

/// 許容軸力 NA（M=0）[N]。`NA = min(fc・Ae, ft・Ae/n)`。
pub(super) fn column_axial_capacity(
    gross_area: f64,
    as_total: f64,
    fc: f64,
    ft: f64,
    n_ratio: f64,
) -> f64 {
    let ae = gross_area + (n_ratio - 1.0) * as_total;
    (fc * ae).min(ft * ae / n_ratio)
}

/// N-M 相関曲線を構成する 1 軸分の状態。
pub(super) struct ColumnAxis {
    pub(super) props: AxisProps,
    /// 直交方向の主筋総断面積（断面中央 D/2 に集約）。
    pub(super) at_perp: f64,
    /// 当該軸の主筋径に応じた許容引張・圧縮応力度 ft(=r_fc)。
    pub(super) ft: f64,
}

/// 中立軸位置 xn における (N_allow, |M_allow|) を求める。
fn column_nm_at_xn(axis: &ColumnAxis, allow: &RcAllow, xn: f64) -> Option<(f64, f64)> {
    if xn <= 0.0 {
        return None;
    }
    let p = &axis.props;
    let d_full = p.d_full;
    let b = p.b;
    let n_ratio = allow.n_ratio;
    let fc = allow.fc;
    let r_fc = axis.ft;
    let ft = axis.ft;

    let s_bar = |y: f64, area: f64| -> f64 {
        if area <= 0.0 {
            return f64::INFINITY;
        }
        let diff = xn - y;
        if diff.abs() < 1e-9 {
            return f64::INFINITY;
        }
        if diff > 0.0 {
            r_fc / (n_ratio * diff)
        } else {
            ft / (n_ratio * (-diff))
        }
    };

    let s1 = fc / xn;
    let s2 = s_bar(p.dt, p.ac);
    let s3 = s_bar(d_full - p.dt, p.at);
    let s = s1.min(s2).min(s3);
    if !s.is_finite() || s <= 0.0 {
        return None;
    }

    let xc = xn.min(d_full);
    if xc <= 0.0 {
        return None;
    }

    let nc = b * s * (xn * xc - xc * xc / 2.0);
    let mc =
        b * s * (xn * (d_full / 2.0) * xc - (xn + d_full / 2.0) * xc * xc / 2.0 + xc.powi(3) / 3.0);

    let bar_contrib = |y: f64, area: f64| -> (f64, f64) {
        if area <= 0.0 {
            return (0.0, 0.0);
        }
        let mult = if y <= xn { n_ratio - 1.0 } else { n_ratio };
        let force = area * mult * s * (xn - y);
        let moment = force * (d_full / 2.0 - y);
        (force, moment)
    };

    let (n_c, m_c) = bar_contrib(p.dt, p.ac);
    let (n_t, m_t) = bar_contrib(d_full - p.dt, p.at);
    let (n_p, m_p) = bar_contrib(d_full / 2.0, axis.at_perp);

    let n_total = nc + n_c + n_t + n_p;
    let m_total = mc + m_c + m_t + m_p;
    Some((n_total, m_total.abs()))
}

const XN_SCAN_POINTS: usize = 400;
const XN_RATIO_MIN: f64 = 0.02;
const XN_RATIO_MAX: f64 = 10.0;

/// N-M 相関曲線を構成する。`xn/D = 0.02〜10` を対数的にスキャンする。
pub(super) fn column_nm_curve(
    axis: &ColumnAxis,
    allow: &RcAllow,
    na_point: f64,
) -> Vec<(f64, f64)> {
    let mut pts = Vec::with_capacity(XN_SCAN_POINTS + 1);
    let log_min = XN_RATIO_MIN.ln();
    let log_max = XN_RATIO_MAX.ln();
    for i in 0..XN_SCAN_POINTS {
        let t = i as f64 / (XN_SCAN_POINTS as f64 - 1.0);
        let ratio = (log_min + t * (log_max - log_min)).exp();
        let xn = axis.props.d_full * ratio;
        if let Some(pt) = column_nm_at_xn(axis, allow, xn) {
            if pt.0.is_finite() && pt.1.is_finite() {
                pts.push(pt);
            }
        }
    }
    pts.push((na_point, 0.0));
    pts.sort_by(|a, b| a.0.total_cmp(&b.0));
    pts
}

/// N-M 相関曲線から設計軸力に対する許容曲げモーメント MA を線形補間で求める。
/// 範囲外は端点値でクランプする。
pub(crate) fn interp_ma(points: &[(f64, f64)], n_design: f64) -> f64 {
    if points.is_empty() {
        return 0.0;
    }
    if n_design <= points[0].0 {
        return points[0].1;
    }
    let last = points.len() - 1;
    if n_design >= points[last].0 {
        return points[last].1;
    }
    for w in points.windows(2) {
        let (n0, m0) = w[0];
        let (n1, m1) = w[1];
        if n_design >= n0 && n_design <= n1 {
            if (n1 - n0).abs() < 1e-9 {
                return m0.max(m1);
            }
            let t = (n_design - n0) / (n1 - n0);
            return m0 + t * (m1 - m0);
        }
    }
    points[last].1
}
