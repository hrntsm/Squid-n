//! 地震時短期の設計用せん断力 QD = min(QD1, QD2)（梁/柱の設計用せん断力。RC 規準）。
//!
//! [`seismic_design_shear`] — 地震時短期の設計用せん断力 QD。

use crate::DesignCtx;

/// 地震時短期の設計用せん断力 QD [N]。
///
/// - 梁: `QD1 = Q0 + n_mech・ΣBMy/l′`（`Q0` 未算定時は `QL` で代替）、
///   柱: `QD1 = n_mech・ΣcMy/h′`
/// - `QD2 = QL + n・QE`（`QE` = 当該組合せのせん断力 − 長期せん断力）
/// - `QD = min(QD1, QD2)`（[`crate::QdMethod`] により QD1/QD2 単独も選択可）
///
/// `ctx.seismic_qd` が None（長期・積雪時・暴風時）、または長期内力に同一
/// 評価位置が見つからない場合は、解析せん断力 `|q_signed|` をそのまま返す
/// （積雪時・暴風時の `QD = QL + Qsn／QL + Qw` は組合せの弾性せん断力に一致）。
///
/// `q_index`: 長期内力配列 `[N,Qy,Qz,Mx,My,Mz]` のせん断成分位置（qy=1, qz=2）。
/// `sum_mu`: 部材両端の終局曲げモーメントの絶対値和 ΣMy [N·mm]。0 以下または
/// `clear_length` が 0 以下の場合、QD1 は無効（QD2 のみ）とする。
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
            // 梁: Q0（両端支持せん断）+ n_mech·ΣMy/L0。Q0 未算定時は QL で代替。
            let q0 = qd.q_simple.unwrap_or(ql).abs();
            q0 + mech
        }
    } else {
        f64::INFINITY
    };
    qd.method.resolve(qd1, qd2)
}
