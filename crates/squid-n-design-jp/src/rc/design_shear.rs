//! 地震時短期の設計用せん断力 QD = min(QD1, QD2)。

use crate::DesignCtx;

/// 地震時短期の設計用せん断力 QD [N]。`QD = min(QD1, QD2)`。
/// `ctx.seismic_qd` が None の場合は解析せん断力 `|q_signed|` を返す。
/// `q_index`: 長期内力配列のせん断成分位置（qy=1, qz=2）。
pub(crate) fn seismic_design_shear(
    ctx: &DesignCtx,
    pos: f64,
    q_signed: f64,
    q_index: usize,
    sum_mu: f64,
    is_column: bool,
) -> f64 {
    let Some(qd) = &ctx.seismic_qd else {
        return q_signed.abs();
    };
    let Some(ql_signed) = qd
        .long_at
        .iter()
        .find(|(p, _)| (p - pos).abs() < 1e-6)
        .map(|(_, f)| f[q_index])
    else {
        return q_signed.abs();
    };
    let ql = ql_signed.abs();
    let qe = (q_signed - ql_signed).abs();
    let qd2 = ql + qd.n_factor * qe;
    let n_mech = if qd.n_mechanism > 0.0 {
        qd.n_mechanism
    } else {
        1.0
    };
    let qd1 = if qd.clear_length > 0.0 && sum_mu > 0.0 {
        let mech = n_mech * sum_mu / qd.clear_length;
        if is_column {
            mech
        } else {
            let q0 = qd.q_simple.unwrap_or(ql).abs();
            q0 + mech
        }
    } else {
        f64::INFINITY
    };
    qd.method.resolve(qd1, qd2)
}
