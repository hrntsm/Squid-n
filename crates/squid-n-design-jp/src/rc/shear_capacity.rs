//! せん断スパン比 α と許容せん断力 QA（普通強度せん断補強筋）。
//!
//! [`shear_alpha`] — せん断スパン比による割増係数 α。
//! [`shear_capacity`] — 許容せん断力 QA（普通強度）。
//! [`shear_capacity_generic`] — 許容せん断力 QA の汎用式。
//! [`shear_capacity_for`] — 許容せん断力 QA の入口。

use super::allowable::*;
use super::section_props::*;
pub(crate) use crate::LoadTerm;

/// せん断スパン比による割増係数 α = 4/(M/(Q・d)+1)。`max_alpha` でクランプ
/// （梁 2.0、柱 1.5）。下限は共通で 1.0。
///
/// 退化時の規約: Q≈0 かつ M>0 は M/(Q・d)→∞ すなわち α→下限 1.0 を返す。
/// M も Q も 0（無応力）または d≤0 は中立な α=1.0（割増なし）とする。
pub(crate) fn shear_alpha(m: f64, q: f64, d: f64, max_alpha: f64) -> f64 {
    if q.abs() < 1e-9 || d <= 0.0 {
        return 1.0;
    }
    let mqd = m.abs() / (q.abs() * d);
    let alpha = 4.0 / (mqd + 1.0);
    alpha.clamp(1.0, max_alpha)
}

/// 許容せん断力 QA [N]。
///
/// 梁（`is_column=false`）:
/// - 長期  `QAL = b・j・(α・fs + 0.5・w_ft・(pw-0.002))`（pw は 0.6% 上限）
/// - 短期・損傷制御 `QAS = b・j・(2/3・α・fs + 0.5・w_ft・(pw-0.002))`
/// - 短期・安全確保 `QAS = b・j・(α・fs + 0.5・w_ft・(pw-0.002))`（pw は 1.2% 上限）
///
/// 柱（`is_column=true`）:
/// - 長期  `QAL = b・j・α・fs`（補強筋項なし）
/// - 短期・損傷制御 `QAS = b・j・(2/3・α・fs + 0.5・w_ft・(pw-0.002))`
/// - 短期・安全確保 `QAS = b・j・(fs + 0.5・w_ft・(pw-0.002))`（**α を含まない**）
///
/// いずれも pw<0.002 のときせん断補強筋項は 0（マイナスにしない）。
pub(crate) fn shear_capacity(
    props: &AxisProps,
    allow: &RcAllow,
    alpha: f64,
    term: LoadTerm,
    damage_control: bool,
    is_column: bool,
) -> f64 {
    let pw_cap = if term == LoadTerm::Long { 0.006 } else { 0.012 };
    shear_capacity_generic(
        props,
        allow,
        alpha,
        term,
        damage_control,
        is_column,
        pw_cap,
        0.002,
    )
}

/// 許容せん断力 QA の汎用式。`pw_cap`（pw の上限値）・`pw_offset`
/// （せん断補強筋項のオフセット、通常は 0.002）を外部から指定できる。
/// [`shear_capacity`] はこの関数をオフセット 0.002 固定で呼び出すラッパーである。
#[allow(clippy::too_many_arguments)]
pub(crate) fn shear_capacity_generic(
    props: &AxisProps,
    allow: &RcAllow,
    alpha: f64,
    term: LoadTerm,
    damage_control: bool,
    is_column: bool,
    pw_cap: f64,
    pw_offset: f64,
) -> f64 {
    let pw = props.pw.min(pw_cap);
    let pw_term = if props.pw < pw_offset {
        0.0
    } else {
        0.5 * allow.w_ft * (pw - pw_offset)
    };

    match term {
        LoadTerm::Long => {
            if is_column {
                props.b * props.j * alpha * allow.fs
            } else {
                props.b * props.j * (alpha * allow.fs + pw_term)
            }
        }
        LoadTerm::Short => {
            if damage_control {
                props.b * props.j * ((2.0 / 3.0) * alpha * allow.fs + pw_term)
            } else if is_column {
                props.b * props.j * (allow.fs + pw_term)
            } else {
                props.b * props.j * (alpha * allow.fs + pw_term)
            }
        }
    }
}

/// 許容せん断力 QA の入口。せん断補強筋は対応グレードのみを対象とするため、
/// 普通強度式 [`shear_capacity`] を呼ぶ。
pub(crate) fn shear_capacity_for(
    props: &AxisProps,
    allow: &RcAllow,
    alpha: f64,
    term: LoadTerm,
    damage_control: bool,
    is_column: bool,
) -> f64 {
    shear_capacity(props, allow, alpha, term, damage_control, is_column)
}
