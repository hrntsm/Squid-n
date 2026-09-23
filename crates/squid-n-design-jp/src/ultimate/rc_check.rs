//! RC 矩形部材の終局検定ドライバ。
//!
//! - [`UltimateCheck`] — 1 部材分の終局検定結果。
//! - [`collect_rc_ultimate_checks`] — モデルの RC 矩形部材を一括検定する。

use crate::MemberKind;
use squid_n_core::ids::ElemId;
use squid_n_core::model::{ElementData, Material, Model, Section};
use squid_n_core::rc_capacity::{rc_mu_simple, RcCapacityInput};
use squid_n_core::section_shape::{one_bar_area, SectionShape};

use super::geometry::clear_span;
use super::options::{MemberDemand, ShearMethod, UltimateShearOptions};
use super::rc_axial::{rc_column_axial_ultimate, RcAxialUltimate};
use super::rc_props::{rc_bar_props, RcDirection};
use super::rc_shear::{
    bond_reliable_strength_deformed, rc_shear_qbu_bond, BondStrengthInput, RcBondSplitInput,
};
use super::rc_shear_ductility::{rc_shear_vbu_ductility, RcVbuInput};
use super::rc_strength::{biaxial_margin, column_axis_shear, column_mu, member_shear_strength};

/// 1 部材分の終局検定結果。
#[derive(Clone, Debug)]
pub struct UltimateCheck {
    /// 部材 ID。
    pub elem: ElemId,
    /// 部材種別（梁/柱）。
    pub kind: MemberKind,
    /// 曲げ終局強度 Mu [N·mm]。
    pub mu: f64,
    /// 両端ヒンジ時せん断力 Qmu = 上限強度倍率·2·Mu/内法 [N]。
    pub qmu: f64,
    /// 塑性理論式による終局せん断強度 Qsu [N]。
    pub qsu: f64,
    /// 付着割裂による終局せん断耐力 Qbu [N]（`include_bond=false` なら 0）。
    pub qbu: f64,
    /// せん断余裕度 Qsu/Qmu（強軸）。
    pub shear_margin: f64,
    /// 2 軸せん断余裕度（柱かつ `biaxial_shear=true` のとき Some）。
    /// `1/((Qmx/Qsux)^2+(Qmy/Qsuy)^2)^(1/2)`。
    pub biaxial_shear_margin: Option<f64>,
    /// 2 軸曲げ余裕度（柱かつ `biaxial_bending=true` のとき Some）。
    /// `1/((Mmx/Mux)^2+(Mmy/Muy)^2)^(1/2)`。設計用曲げ需要が 0 なら `f64::INFINITY`。
    pub biaxial_bending_margin: Option<f64>,
    /// 付着余裕度 Qbu/Qmu（`include_bond=false` なら `f64::INFINITY`）。
    pub bond_margin: f64,
    /// 軸終局耐力（柱のみ Some）。
    pub axial: Option<RcAxialUltimate>,
    /// 判定（せん断余裕度・付着余裕度が共に 1.0 以上で true）。
    pub ok: bool,
    /// 根拠（表示用）。
    pub basis: String,
    /// 詳細（表示用）。
    pub detail: String,
}

/// 強軸曲げの引張側。`mz > 0` は下端引張、`mz < 0` は上端引張、`mz == 0` は
/// 上下の引張鉄筋量 `at` が小さい側を引張とする。
fn strong_tension_is_top(shape: &SectionShape, mz: f64, need_be: bool) -> bool {
    if mz < 0.0 {
        return true;
    }
    if mz > 0.0 {
        return false;
    }
    let at_top = rc_bar_props(shape, RcDirection::Strong, true, need_be).map(|p| p.at);
    let at_bottom = rc_bar_props(shape, RcDirection::Strong, false, need_be).map(|p| p.at);
    match (at_top, at_bottom) {
        (Some(t), Some(b)) => t < b,
        (Some(_), None) => true,
        _ => false,
    }
}

/// 1 部材の終局検定を実行する（対応断面以外・Fc 未設定は `Ok(None)`、
/// せん断補強筋に未対応グレードまたは `fy` 未設定がある場合は `Err`）。
fn check_member(
    elem: &ElementData,
    sec: &Section,
    mat: &Material,
    model: &Model,
    demand: MemberDemand,
    opts: &UltimateShearOptions,
) -> Result<Option<UltimateCheck>, String> {
    let Some(shape) = sec.shape.as_ref() else {
        return Ok(None);
    };
    let (b, d, kind) = match shape {
        SectionShape::RcBeamRect { b, d, .. } => (*b, *d, MemberKind::Beam),
        SectionShape::RcColumnRect { b, d, .. } => (*b, *d, MemberKind::Column),
        SectionShape::RcColumnCircle { d, .. } => (*d, *d, MemberKind::Column),
        SectionShape::RcRect { b, d, .. } => (*b, *d, MemberKind::of_element(elem, model)),
        _ => return Ok(None),
    };
    let Some(fc) = mat.fc else {
        return Ok(None);
    };
    if fc <= 0.0 || b <= 0.0 || d <= 0.0 {
        return Ok(None);
    }
    if let Some(msg) = squid_n_core::material_grade::shear_rebar_material_issue(
        model.element_shear_rebar_material(elem),
    ) {
        return Err(format!("部材 ID {} の{}", elem.id.0, msg));
    }
    let opts_owned = UltimateShearOptions {
        rp: demand.rp.map(|rp| rp.max(0.0)).unwrap_or(opts.rp),
        sigma_wy: squid_n_core::material_grade::shear_rebar_yield_strength(
            model.element_shear_rebar_material(elem),
        )
        .unwrap_or(opts.sigma_wy),
        ..opts.clone()
    };
    let opts = &opts_owned;
    let Some(sigma_y) =
        squid_n_core::material_grade::rebar_yield_strength(model.element_rebar_material(elem))
    else {
        return Ok(None);
    };
    let l_clear = clear_span(elem, model);

    let need_be = opts.shear_method == ShearMethod::Ductility;
    let tension_is_top = strong_tension_is_top(shape, demand.mz, need_be);
    let Some(p) = rc_bar_props(shape, RcDirection::Strong, tension_is_top, need_be) else {
        return Ok(None);
    };
    let jt = 7.0 * p.d_eff / 8.0;
    let at = p.at;
    let ag = p.ag;
    let pw = p.pw;
    let n_axial = demand.n_axial;

    let cap = RcCapacityInput {
        b: p.b_dir,
        d: p.d_dir,
        at,
        d_eff: p.d_eff,
        sigma_y,
        fc,
        pw,
        sigma_wy: opts.sigma_wy,
        clear_span: l_clear.max(1.0),
        sigma_0: 0.0,
    };
    let mu = match kind {
        MemberKind::Column => column_mu(p.b_dir, p.d_dir, p.dt, at, ag, sigma_y, fc, n_axial),
        _ => rc_mu_simple(&cap),
    };

    let qmu = match demand.shear {
        Some(qm) => opts.upper_strength_factor * qm.abs(),
        None => {
            if l_clear > 0.0 {
                opts.upper_strength_factor * 2.0 * mu / l_clear
            } else {
                0.0
            }
        }
    };

    let qsu = member_shear_strength(&p, fc, n_axial, l_clear, opts);

    let (qbu, tau_bu) = if opts.include_bond {
        let tau_bu = bond_reliable_strength_deformed(&BondStrengthInput {
            fc,
            b: p.b_dir,
            db1: p.main_dia,
            n_bars: p.n_tension,
            cover_side: p.cover,
            cover_bottom: p.cover,
            hoop_area: p.shear_legs as f64 * one_bar_area(p.shear_dia),
            hoop_pitch: p.shear_pitch,
            pw,
            top_bar: false,
        });
        let sum_phi = p.n_tension as f64 * std::f64::consts::PI * p.main_dia;
        let qbu = match opts.shear_method {
            ShearMethod::Plastic => rc_shear_qbu_bond(&RcBondSplitInput {
                b: p.b_dir,
                d_full: p.d_dir,
                jt,
                tau_bu,
                sum_phi,
                l_clear,
                fc,
                rp: opts.rp,
                lightweight: opts.lightweight,
            }),
            ShearMethod::Ductility => rc_shear_vbu_ductility(&RcVbuInput {
                b: p.b_dir,
                d_full: p.d_dir,
                be: p.be,
                je: jt,
                tau_bu,
                sum_phi1: sum_phi,
                tau_bu2: 0.0,
                sum_phi2: 0.0,
                s: p.shear_pitch,
                n_s: p.n_s,
                l_clear,
                fc,
                rp: opts.rp,
                tensile_axial: n_axial < 0.0,
                yield_hinge: opts.rp > 0.0,
                lightweight: opts.lightweight,
            }),
        };
        (qbu, tau_bu)
    } else {
        (0.0, 0.0)
    };

    let ql = demand.q_long.map(|q| q.abs()).unwrap_or(0.0);
    let shear_margin = if qmu > 0.0 {
        ((qsu - ql).max(0.0)) / qmu
    } else {
        f64::INFINITY
    };
    let bond_margin = if !opts.include_bond {
        f64::INFINITY
    } else if qmu > 0.0 {
        ((qbu - ql).max(0.0)) / qmu
    } else {
        f64::INFINITY
    };

    let biaxial_shear_margin = if kind == MemberKind::Column && opts.biaxial_shear {
        rc_bar_props(shape, RcDirection::Weak, tension_is_top, need_be).map(|p_y| {
            let (qsu_y, qmu_y_hinge) =
                column_axis_shear(&p_y, fc, sigma_y, n_axial, l_clear, opts);
            let qmu_y = match demand.shear_weak {
                Some(qmy) => opts.upper_strength_factor * qmy.abs(),
                None => qmu_y_hinge,
            };
            let rx = if qsu > 0.0 { qmu / qsu } else { f64::INFINITY };
            let ry = if qsu_y > 0.0 {
                qmu_y / qsu_y
            } else {
                f64::INFINITY
            };
            biaxial_margin(rx, ry, 2.0)
        })
    } else {
        None
    };

    let biaxial_bending_margin = if kind == MemberKind::Column && opts.biaxial_bending {
        rc_bar_props(shape, RcDirection::Weak, tension_is_top, need_be).map(|p_y| {
            let mux = mu;
            let muy =
                column_mu(p_y.b_dir, p_y.d_dir, p_y.dt, p_y.at, p_y.ag, sigma_y, fc, n_axial);
            let rx = if mux > 0.0 {
                demand.mz.abs() / mux
            } else if demand.mz.abs() > 0.0 {
                f64::INFINITY
            } else {
                0.0
            };
            let ry = if muy > 0.0 {
                demand.my.abs() / muy
            } else if demand.my.abs() > 0.0 {
                f64::INFINITY
            } else {
                0.0
            };
            biaxial_margin(rx, ry, 2.0)
        })
    } else {
        None
    };

    let axial = if kind == MemberKind::Column {
        Some(rc_column_axial_ultimate(p.b_dir, p.d_dir, fc, ag, sigma_y))
    } else {
        None
    };

    let effective_shear_ok = match biaxial_shear_margin {
        Some(m) => m >= 1.0,
        None => shear_margin >= 1.0,
    };
    let bending_ok = biaxial_bending_margin.map(|m| m >= 1.0).unwrap_or(true);
    let ok = effective_shear_ok && bond_margin >= 1.0 && bending_ok;

    let shear_label = match opts.shear_method {
        ShearMethod::Plastic => "塑性理論式 Qsu",
        ShearMethod::Ductility => "靭性指針式 Vu",
    };
    let basis = match kind {
        MemberKind::Column => format!("RC柱 終局検定（{shear_label}/Qbu）"),
        _ => format!("RC梁 終局検定（{shear_label}/Qbu）"),
    };
    let biaxial_str = match biaxial_shear_margin {
        Some(m) => format!(", 2軸せん断余裕度={m:.3}"),
        None => String::new(),
    };
    let bend_str = match biaxial_bending_margin {
        Some(m) => format!(", 2軸曲げ余裕度={m:.3}"),
        None => String::new(),
    };
    let detail = format!(
        "Mu={:.0} N·mm, Qmu={:.0} N, Qsu={:.0} N, Qbu={:.0} N, τbu={:.3} N/mm², \
         Qsu/Qmu={:.3}, Qbu/Qmu={:.3}{}{}, pw={:.5}, jt={:.1} mm, L={:.0} mm, Rp={:.4}",
        mu,
        qmu,
        qsu,
        qbu,
        tau_bu,
        shear_margin,
        bond_margin,
        biaxial_str,
        bend_str,
        pw,
        jt,
        l_clear,
        opts.rp
    );

    Ok(Some(UltimateCheck {
        elem: elem.id,
        kind,
        mu,
        qmu,
        qsu,
        qbu,
        shear_margin,
        biaxial_shear_margin,
        biaxial_bending_margin,
        bond_margin,
        axial,
        ok,
        basis,
        detail,
    }))
}

/// モデルの RC 矩形部材について終局検定（塑性理論式）を一括実行する。
///
/// - `demand_by_elem`: 部材の設計用需要（[`MemberDemand`]：圧縮正の軸力と強軸/弱軸の
///   設計用曲げモーメント）。柱の Mu・軸余裕度・2 軸曲げ余裕度に用いる。該当 ID がない
///   部材は需要 0（安全側）で評価する。軸力は長期（G+P）静的、曲げ需要は当該組合せの
///   応答値を渡すことを想定する。
/// - 対象外（`RcRect` 以外・断面/材料未解決・Fc 未設定・有効せい ≤ 0）の部材は
///   結果に含めない。
/// - 検定対象の部材でせん断補強筋に未対応グレードまたは `fy` 未設定がある場合は、
///   部材 ID と是正内容を含む理由を `Err` で返す。
pub fn collect_rc_ultimate_checks(
    model: &Model,
    demand_by_elem: &[(ElemId, MemberDemand)],
    opts: &UltimateShearOptions,
) -> Result<Vec<UltimateCheck>, String> {
    let mut out = Vec::new();
    for elem in &model.elements {
        let Some(sec) = elem.section.and_then(|sid| model.sections.get(sid.index())) else {
            continue;
        };
        let Some(mat) = model.element_material(elem) else {
            continue;
        };
        let demand = demand_by_elem
            .iter()
            .find(|(id, _)| *id == elem.id)
            .map(|(_, d)| *d)
            .unwrap_or_default();
        if let Some(check) = check_member(elem, sec, mat, model, demand, opts)? {
            out.push(check);
        }
    }
    Ok(out)
}
