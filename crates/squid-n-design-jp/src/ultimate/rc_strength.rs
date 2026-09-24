//! RC 部材の終局強度（曲げ・せん断）ヘルパ群。
//!
//! - [`biaxial_margin`] — 2 軸相互作用の余裕度。
//! - [`column_axis_shear`] — 指定方向の柱の Qsu・Qmu（2 軸せん断用）。
//! - [`column_mu`] — 柱の曲げ終局強度 Mu（構造規定 at 式）。
//! - [`member_shear_strength`] — 選択式に応じた終局せん断強度 Qsu/Vu。
//! - [`ductility_be_ns`] — 靭性指針式のトラス機構有効幅 be・中子筋本数 Ns。

use super::options::{ShearMethod, UltimateShearOptions};
use super::rc_props::RcBarProps;
use super::rc_shear::{rc_shear_qsu_plastic, RcPlasticShearInput};
use super::rc_shear_ductility::{rc_shear_vu_ductility, RcDuctilityShearInput};
use squid_n_core::rc_capacity::{rc_column_mu_simple, RcCapacityInput};
use squid_n_core::section_shape::{one_bar_area, RcRebar};

/// 2 軸相互作用の余裕度 `1/((rx)^α + (ry)^α)^(1/α)`（採用応力）。
///
/// `rx`,`ry` は各軸の「需要/耐力」比（例: `Qmx/Qux`, `Qmy/Quy`）、`alpha` は相互作用の
/// 指数（RC 柱は 2.0）。ここでは αx=αy=α と等しく扱う。両比が 0 のとき（需要ゼロ）は
/// `f64::INFINITY` を返す。`alpha ≤ 0` の不正入力も `f64::INFINITY`。
pub fn biaxial_margin(rx: f64, ry: f64, alpha: f64) -> f64 {
    if alpha <= 0.0 {
        return f64::INFINITY;
    }
    let rx = rx.max(0.0);
    let ry = ry.max(0.0);
    let s = rx.powf(alpha) + ry.powf(alpha);
    if s <= 0.0 {
        f64::INFINITY
    } else {
        1.0 / s.powf(1.0 / alpha)
    }
}

/// 指定方向の諸元 `props` から柱の終局せん断強度 `Qsu`（塑性理論式）と両端ヒンジ時
/// せん断力 `Qmu` を算定する（2 軸せん断用）。有効せいが 0 以下なら `(0.0, 0.0)`。
pub(super) fn column_axis_shear(
    props: &RcBarProps,
    fc: f64,
    sigma_y: f64,
    n_axial: f64,
    l_clear: f64,
    opts: &UltimateShearOptions,
) -> (f64, f64) {
    if props.d_eff <= 0.0 {
        return (0.0, 0.0);
    }
    let cap = RcCapacityInput {
        b: props.b_dir,
        d: props.d_dir,
        at: props.at,
        d_eff: props.d_eff,
        sigma_y,
        fc,
        pw: props.pw,
        sigma_wy: opts.sigma_wy,
        clear_span: l_clear.max(1.0),
        sigma_0: 0.0,
    };
    let qsu = member_shear_strength(props, fc, n_axial, l_clear, opts);
    let mu = rc_column_mu_simple(&cap, props.ag, n_axial);
    let qmu = if l_clear > 0.0 {
        opts.upper_strength_factor * 2.0 * mu / l_clear
    } else {
        0.0
    };
    (qsu, qmu)
}

/// 柱の曲げ終局強度 Mu [N·mm]（構造規定 at 式、軸力考慮）。
/// `b_dir`=幅, `d_dir`=せい, `dt`=引張縁〜引張筋距離, `at`=引張側主筋, `ag`=全主筋。
#[allow(clippy::too_many_arguments)]
pub(super) fn column_mu(
    b_dir: f64,
    d_dir: f64,
    dt: f64,
    at: f64,
    ag: f64,
    sigma_y: f64,
    fc: f64,
    n_axial: f64,
) -> f64 {
    let cap = RcCapacityInput {
        b: b_dir,
        d: d_dir,
        at,
        d_eff: (d_dir - dt).max(1.0),
        sigma_y,
        fc,
        pw: 0.0,
        sigma_wy: 0.0,
        clear_span: 1.0,
        sigma_0: 0.0,
    };
    rc_column_mu_simple(&cap, ag, n_axial)
}

/// 靭性指針式による終局せん断信頼強度 `Vu` [N]（[`rc_shear_ductility`]）を
/// 指定方向の諸元 `props` から算定する。`je` はトラス機構有効せい（`jt` を用いる）。
fn member_vu_ductility(
    props: &RcBarProps,
    je: f64,
    fc: f64,
    n_axial: f64,
    l_clear: f64,
    sigma_wy: f64,
    opts: &UltimateShearOptions,
) -> f64 {
    let s = props.shear_pitch;
    let aw = props.shear_legs as f64 * one_bar_area(props.shear_dia);
    let pwe = if s > 0.0 { aw / (props.be * s) } else { 0.0 };
    rc_shear_vu_ductility(&RcDuctilityShearInput {
        b: props.b_dir,
        d_full: props.d_dir,
        be: props.be,
        je,
        pwe,
        sigma_wy,
        s,
        n_s: props.n_s,
        l_clear,
        fc,
        rp: opts.rp,
        tensile_axial: n_axial < 0.0,
        lightweight: opts.lightweight,
    })
}

/// 靭性指針式のトラス機構有効幅 `be`（外側横補強筋の芯々間隔近似）と中子筋本数 `Ns`
/// （`legs/2 − 1` 近似）を断面諸元から求める（[`member_vu_ductility`]・Vbu で共用）。
#[allow(dead_code)]
pub(super) fn ductility_be_ns(b_dir: f64, rebar: &RcRebar) -> (f64, u32) {
    let be = (b_dir - 2.0 * (rebar.cover + rebar.shear.dia / 2.0)).max(1.0);
    let n_s = (rebar.shear.legs / 2).saturating_sub(1);
    (be, n_s)
}

/// 選択された [`ShearMethod`] に応じた終局せん断強度 `Qsu`/`Vu` [N]。
pub(super) fn member_shear_strength(
    props: &RcBarProps,
    fc: f64,
    n_axial: f64,
    l_clear: f64,
    opts: &UltimateShearOptions,
) -> f64 {
    let jt = 7.0 * props.d_eff / 8.0;
    match opts.shear_method {
        ShearMethod::Plastic => rc_shear_qsu_plastic(&RcPlasticShearInput {
            b: props.b_dir,
            d_full: props.d_dir,
            jt,
            pw: props.pw,
            sigma_wy: opts.sigma_wy,
            l_clear,
            fc,
            rp: opts.rp,
            lightweight: opts.lightweight,
        }),
        ShearMethod::Ductility => {
            member_vu_ductility(props, jt, fc, n_axial, l_clear, opts.sigma_wy, opts)
        }
    }
}
