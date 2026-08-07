//! RC 部材の終局強度（曲げ・せん断）ヘルパ群。
//!
//! - [`biaxial_margin`] — 2 軸相互作用の余裕度。
//! - [`column_axis_shear`] — 指定軸方向の柱の Qsu・Qmu（2 軸せん断用）。
//! - [`column_mu`] — 柱の曲げ終局強度 Mu（at 式 / ACI）。
//! - [`member_shear_strength`] — 選択式に応じた終局せん断強度 Qsu/Vu。
//! - [`ductility_be_ns`] — 靭性指針式のトラス機構有効幅 be・中子筋本数 Ns。

use super::options::{MuMethod, ShearMethod, UltimateShearOptions};
use super::rc_column_aci::{rc_column_mu_aci, AciColumnInput};
use super::rc_section::{bar_set_area, hoop_pw};
use super::rc_shear::{rc_shear_qsu_plastic, RcPlasticShearInput};
use super::rc_shear_ductility::{rc_shear_vu_ductility, RcDuctilityShearInput};
use squid_n_core::rc_capacity::{rc_column_mu_simple, RcCapacityInput};
use squid_n_core::section_shape::{BarSet, RcRebar};

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

/// 指定方向（`b_dir`=幅, `d_dir`=せい, `main`=当該方向主筋）の柱の終局せん断強度
/// `Qsu`（塑性理論式）と両端ヒンジ時せん断力 `Qmu` を算定する（2 軸せん断用）。
#[allow(clippy::too_many_arguments)]
pub(super) fn column_axis_shear(
    b_dir: f64,
    d_dir: f64,
    main: &BarSet,
    rebar: &RcRebar,
    fc: f64,
    sigma_y: f64,
    ag: f64,
    n_axial: f64,
    l_clear: f64,
    opts: &UltimateShearOptions,
) -> (f64, f64) {
    let dt = rebar.cover + rebar.shear.dia + main.dia / 2.0;
    let d_eff = d_dir - dt;
    if d_eff <= 0.0 {
        return (0.0, 0.0);
    }
    let jt = 7.0 * d_eff / 8.0;
    let at = bar_set_area(main) / 2.0;
    let pw = hoop_pw(rebar, b_dir);
    let qsu = member_shear_strength(
        b_dir, d_dir, jt, pw, rebar, fc, n_axial, l_clear, true, opts,
    );
    let cap = RcCapacityInput {
        b: b_dir,
        d: d_dir,
        at,
        d_eff,
        sigma_y,
        fc,
        pw,
        sigma_wy: opts.sigma_wy,
        clear_span: l_clear.max(1.0),
        sigma_0: 0.0,
    };
    let mu = rc_column_mu_simple(&cap, ag, n_axial);
    let qmu = if l_clear > 0.0 {
        opts.upper_strength_factor * 2.0 * mu / l_clear
    } else {
        0.0
    };
    (qsu, qmu)
}

/// 柱の曲げ終局強度 Mu [N·mm]（`mu_method` に応じて at 式 / ACI 平面保持）。
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
    mu_method: MuMethod,
) -> f64 {
    match mu_method {
        MuMethod::Aci => {
            let layers = [(dt, at), (d_dir - dt, at)];
            rc_column_mu_aci(
                &AciColumnInput {
                    b: b_dir,
                    d_full: d_dir,
                    fc,
                    sigma_y,
                    es: 205000.0,
                },
                &layers,
                n_axial,
            )
        }
        MuMethod::AtFormula => {
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
    }
}

/// 靭性指針式による終局せん断信頼強度 `Vu` [N]（[`rc_shear_ductility`]）を断面諸元から
/// 算定する。`b_dir`=幅, `d_dir`=せい, `je`=トラス機構有効せい（`jt` を用いる）。
///
/// # 簡略化（doc 兼申し送り）
/// 靭性指針の `be`（トラス機構有効幅＝外側横補強筋の芯々間隔）・`Ns`（中子筋本数）は
/// モデルに直接保持されないため、以下で近似する:
/// - `be = 幅 − 2·(かぶり + 補強筋径/2)`（せん断補強筋のコア芯々幅）。
/// - `pwe = aw/(be·s)`（`aw`＝1 組の補強筋断面積、`s`＝ピッチ）。
/// - `Ns = legs/2 − 1`（2 本脚→Ns=0、4 本脚→Ns=1、…）。
/// - 引張軸力（`n_axial < 0`）の柱は `tanθ=0`（アーチ機構無効）。
#[allow(clippy::too_many_arguments)]
fn member_vu_ductility(
    b_dir: f64,
    d_dir: f64,
    je: f64,
    rebar: &RcRebar,
    fc: f64,
    n_axial: f64,
    l_clear: f64,
    sigma_wy: f64,
    opts: &UltimateShearOptions,
) -> f64 {
    let (be, n_s) = ductility_be_ns(b_dir, rebar);
    let s = rebar.shear.pitch;
    let aw =
        rebar.shear.legs as f64 * std::f64::consts::PI / 4.0 * rebar.shear.dia * rebar.shear.dia;
    let pwe = if s > 0.0 { aw / (be * s) } else { 0.0 };
    rc_shear_vu_ductility(&RcDuctilityShearInput {
        b: b_dir,
        d_full: d_dir,
        be,
        je,
        pwe,
        sigma_wy,
        s,
        n_s,
        l_clear,
        fc,
        rp: opts.rp,
        tensile_axial: n_axial < 0.0,
        lightweight: opts.lightweight,
    })
}

/// 靭性指針式のトラス機構有効幅 `be`（外側横補強筋の芯々間隔近似）と中子筋本数 `Ns`
/// （`legs/2 − 1` 近似）を断面諸元から求める（[`member_vu_ductility`]・Vbu で共用）。
pub(super) fn ductility_be_ns(b_dir: f64, rebar: &RcRebar) -> (f64, u32) {
    let be = (b_dir - 2.0 * (rebar.cover + rebar.shear.dia / 2.0)).max(1.0);
    let n_s = (rebar.shear.legs / 2).saturating_sub(1);
    (be, n_s)
}

/// 部材のせん断補強筋の終局検定用 σwy・ν0 上書き・上限適用後 pw を解決する。
///
/// せん断補強筋の材質（`UltimateShearOptions::shear_grade`。断面の
/// `shear_rebar_material` の名前）が高強度せん断補強筋の既知製品の場合、製品別の
/// σwy（min(25·Fc, 上限) 等）・ν0（1275 級 0.7(1.0−Fc/140)、785/685 級
/// 0.7(0.7−Fc/200)）・pw 上限（1.2%、1275 級の柱かつ Fc＜27 は 0.8%）を適用する
/// （[`crate::material_strength::ultimate_hoop_sigma_wy`] ほか）。
/// 普通強度・判別不能な製品名は (opts.sigma_wy, None, pw) のまま。
fn resolve_hoop_ultimate(
    fc: f64,
    pw: f64,
    is_column: bool,
    opts: &UltimateShearOptions,
) -> (f64, Option<f64>, f64) {
    use crate::material_strength::{
        ultimate_hoop_nu0, ultimate_hoop_pw_cap, ultimate_hoop_sigma_wy,
    };
    let grade = opts.shear_grade.as_deref();
    // 高強度品は製品表（Fc 依存の頭打ち込み）、普通強度（SD*/SR*）は規格の基準強度、
    // 材質未設定のみ既定値へ落とす。
    let sigma_wy = grade
        .and_then(|g| ultimate_hoop_sigma_wy(g, fc))
        .or_else(|| {
            grade
                .filter(|g| !crate::material_strength::is_high_strength_shear_grade(g))
                .and_then(squid_n_core::material_grade::rebar_grade_f_value)
        })
        .unwrap_or(opts.sigma_wy);
    let nu0_override = grade.and_then(|g| ultimate_hoop_nu0(g, fc));
    let pw_capped = match grade.and_then(|g| ultimate_hoop_pw_cap(g, fc, is_column)) {
        Some(cap) => pw.min(cap),
        None => pw,
    };
    (sigma_wy, nu0_override, pw_capped)
}

/// 選択された [`ShearMethod`] に応じた終局せん断強度 `Qsu`/`Vu` [N]。
///
/// 高強度せん断補強筋（`ShearBar.grade`）使用時は製品別の σwy・ν0・pw 上限を
/// 適用する（[`resolve_hoop_ultimate`]）。靭性指針式（Vu）には製品別 σwy のみ
/// 適用し、ν は指針の標準式のまま（製品別 ν は塑性理論式の表による規定のため）。
#[allow(clippy::too_many_arguments)]
pub(super) fn member_shear_strength(
    b_dir: f64,
    d_dir: f64,
    jt: f64,
    pw: f64,
    rebar: &RcRebar,
    fc: f64,
    n_axial: f64,
    l_clear: f64,
    is_column: bool,
    opts: &UltimateShearOptions,
) -> f64 {
    let (sigma_wy, nu0_override, pw) = resolve_hoop_ultimate(fc, pw, is_column, opts);
    match opts.shear_method {
        ShearMethod::Plastic => rc_shear_qsu_plastic(&RcPlasticShearInput {
            b: b_dir,
            d_full: d_dir,
            jt,
            pw,
            sigma_wy,
            l_clear,
            fc,
            rp: opts.rp,
            lightweight: opts.lightweight,
            nu0_override,
        }),
        ShearMethod::Ductility => member_vu_ductility(
            b_dir, d_dir, jt, rebar, fc, n_axial, l_clear, sigma_wy, opts,
        ),
    }
}
