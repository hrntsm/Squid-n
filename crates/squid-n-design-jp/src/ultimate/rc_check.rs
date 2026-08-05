//! RC 矩形部材の終局検定ドライバ。
//!
//! - [`UltimateCheck`] — 1 部材分の終局検定結果。
//! - [`collect_rc_ultimate_checks`] — モデルの RC 矩形部材を一括検定する。

use crate::MemberKind;
use squid_n_core::ids::ElemId;
use squid_n_core::model::{ElementData, Material, Model, Section};
use squid_n_core::rc_capacity::{rc_mu_simple, RcCapacityInput};
use squid_n_core::section_shape::SectionShape;

use super::geometry::{clear_span, member_kind};
use super::options::{MemberDemand, ShearMethod, UltimateShearOptions};
use super::rc_axial::{rc_column_axial_ultimate, RcAxialUltimate};
use super::rc_section::{bar_set_area, hoop_pw};
use super::rc_shear::{
    bond_reliable_strength_deformed, rc_shear_qbu_bond, BondStrengthInput, RcBondSplitInput,
};
use super::rc_shear_ductility::{rc_shear_vbu_ductility, RcVbuInput};
use super::rc_strength::{
    biaxial_margin, column_axis_shear, column_mu, ductility_be_ns, member_shear_strength,
};

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

/// 1 部材の終局検定を実行する（`RcRect` 以外・Fc 未設定は `None`）。
fn check_member(
    elem: &ElementData,
    sec: &Section,
    mat: &Material,
    model: &Model,
    demand: MemberDemand,
    opts: &UltimateShearOptions,
) -> Option<UltimateCheck> {
    let SectionShape::RcRect { b, d, rebar } = sec.shape.as_ref()? else {
        return None;
    };
    let (b, d) = (*b, *d);
    let fc = mat.fc?;
    if fc <= 0.0 || b <= 0.0 || d <= 0.0 {
        return None;
    }
    // 部材別 Rp（プッシュオーバー応答からの直接反映）が与えられていれば UI 一律 Rp を
    // 置き換える。以降 opts.rp を参照する全経路（ν・cotφ・μ・tanθ）に効く。
    let opts_owned;
    let opts = if let Some(rp) = demand.rp {
        opts_owned = UltimateShearOptions {
            rp: rp.max(0.0),
            ..*opts
        };
        &opts_owned
    } else {
        opts
    };
    let kind = member_kind(elem, model);
    let sigma_y = mat.fy.unwrap_or(345.0);
    let l_clear = clear_span(elem, model);

    // 断面諸元（強軸＝せい方向主筋 main_x）。
    let dt = rebar.cover + rebar.shear.dia + rebar.main_x.dia / 2.0;
    let d_eff = d - dt;
    if d_eff <= 0.0 {
        return None;
    }
    let jt = 7.0 * d_eff / 8.0;
    let at = bar_set_area(&rebar.main_x) / 2.0;
    let ag = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
    let pw = hoop_pw(rebar, b);
    let n_axial = demand.n_axial;

    // 曲げ終局強度 Mu（柱は軸力考慮・mu_method 対応、梁は軸力なし）。
    let cap = RcCapacityInput {
        b,
        d,
        at,
        d_eff,
        sigma_y,
        fc,
        pw,
        sigma_wy: opts.sigma_wy,
        clear_span: l_clear.max(1.0),
        sigma_0: 0.0,
    };
    let mu = match kind {
        MemberKind::Column => column_mu(b, d, dt, at, ag, sigma_y, fc, n_axial, opts.mu_method),
        _ => rc_mu_simple(&cap),
    };

    // 設計用せん断力 Qmu。プッシュオーバー応答の設計用せん断が与えられていれば
    // それを直接反映（上限強度倍率を乗じる）、なければ両端ヒンジ略算 2·Mu/内法。
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

    // 終局せん断強度 Qsu（塑性理論式）または Vu（靭性指針式）。
    let qsu = member_shear_strength(
        b,
        d,
        jt,
        pw,
        rebar,
        fc,
        n_axial,
        l_clear,
        matches!(kind, MemberKind::Column),
        opts,
    );

    // 付着割裂耐力 Qbu。
    let (qbu, tau_bu) = if opts.include_bond {
        // 引張側主筋本数（対称配筋の半分、外側一列を代表）。
        let n_tension = (rebar.main_x.count as f64 / 2.0).max(1.0);
        let tau_bu = bond_reliable_strength_deformed(&BondStrengthInput {
            fc,
            b,
            db1: rebar.main_x.dia,
            n_bars: n_tension.round() as u32,
            cover_side: rebar.cover,
            cover_bottom: rebar.cover,
            hoop_area: rebar.shear.legs as f64 * std::f64::consts::PI / 4.0
                * rebar.shear.dia
                * rebar.shear.dia,
            hoop_pitch: rebar.shear.pitch,
            pw,
            top_bar: false,
        });
        let sum_phi = n_tension * std::f64::consts::PI * rebar.main_x.dia;
        // 塑性理論式は付着割裂耐力 Qbu、靭性指針式は付着考慮せん断信頼強度 Vbu を用いる。
        let qbu = match opts.shear_method {
            ShearMethod::Plastic => rc_shear_qbu_bond(&RcBondSplitInput {
                b,
                d_full: d,
                jt,
                tau_bu,
                sum_phi,
                l_clear,
                fc,
                rp: opts.rp,
                lightweight: opts.lightweight,
            }),
            ShearMethod::Ductility => {
                let (be, n_s) = ductility_be_ns(b, rebar);
                rc_shear_vbu_ductility(&RcVbuInput {
                    b,
                    d_full: d,
                    be,
                    je: jt,
                    tau_bu,
                    sum_phi1: sum_phi,
                    // モデルは 1 段配筋を仮定するため 2 段目主筋（τbu2・Σφ2）は 0。
                    tau_bu2: 0.0,
                    sum_phi2: 0.0,
                    s: rebar.shear.pitch,
                    n_s,
                    l_clear,
                    fc,
                    rp: opts.rp,
                    tensile_axial: n_axial < 0.0,
                    // Rp>0（ヒンジ回転を指定）を降伏ヒンジ計画部材とみなす（6.8.16b）。
                    yield_hinge: opts.rp > 0.0,
                    lightweight: opts.lightweight,
                })
            }
        };
        (qbu, tau_bu)
    } else {
        (0.0, 0.0)
    };

    // 梁の余裕率は分子から長期せん断力 QL を控除する
    // （(Qsu−QL)/Qmu・(Qbu−QL)/Qmu ≥ 1.0。QL 未指定は 0 扱い＝従来動作）。
    // せん断補強筋が MK785/SPR785/SPR685 の場合は QL=Q0（長期荷重による
    // 単純梁せん断力）と読み替える（各製品の技術評定の規定。Q0 未算定時は QL）。
    let use_q_simple = rebar
        .shear
        .grade
        .as_deref()
        .map(|g| {
            let g = g.trim().to_uppercase();
            ["MK785", "SPR785", "SPR685"]
                .iter()
                .any(|p| g.starts_with(p))
        })
        .unwrap_or(false);
    let ql = if use_q_simple {
        demand
            .q_simple
            .or(demand.q_long)
            .map(|q| q.abs())
            .unwrap_or(0.0)
    } else {
        demand.q_long.map(|q| q.abs()).unwrap_or(0.0)
    };
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

    // 2 軸せん断余裕度（柱のみ、指定時）。弱軸（main_y、b↔D 入替）の Qsu/Qmu を
    // 算定し、相互作用式 1/((Qmx/Qsux)^2+(Qmy/Qsuy)^2)^(1/2)（RC は α=2.0）で合成する。
    let biaxial_shear_margin = if kind == MemberKind::Column && opts.biaxial_shear {
        let (qsu_y, qmu_y_hinge) = column_axis_shear(
            d,
            b,
            &rebar.main_y,
            rebar,
            fc,
            sigma_y,
            ag,
            n_axial,
            l_clear,
            opts,
        );
        // 弱軸設計用せん断 Qmuy。プッシュオーバー応答の弱軸せん断が与えられていれば
        // それを直接反映（上限強度倍率を乗じる）、なければ両端ヒンジ略算 2·Muy/内法。
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
        Some(biaxial_margin(rx, ry, 2.0))
    } else {
        None
    };

    // 2 軸曲げ余裕度（柱のみ、指定時）。強軸 Mux（=mu）・弱軸 Muy（main_y, b↔D 入替）の
    // 終局曲げ強度と設計用曲げ需要 Mmx=|mz|, Mmy=|my| を相互作用式で合成する。
    let biaxial_bending_margin = if kind == MemberKind::Column && opts.biaxial_bending {
        let dt_y = rebar.cover + rebar.shear.dia + rebar.main_y.dia / 2.0;
        let at_y = bar_set_area(&rebar.main_y) / 2.0;
        let mux = mu;
        let muy = column_mu(d, b, dt_y, at_y, ag, sigma_y, fc, n_axial, opts.mu_method);
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
        Some(biaxial_margin(rx, ry, 2.0))
    } else {
        None
    };

    let axial = if kind == MemberKind::Column {
        Some(rc_column_axial_ultimate(b, d, fc, ag, sigma_y))
    } else {
        None
    };

    // せん断判定は 2 軸指定時は 2 軸余裕度、そうでなければ強軸せん断余裕度を用いる。
    let effective_shear_ok = match biaxial_shear_margin {
        Some(m) => m >= 1.0,
        None => shear_margin >= 1.0,
    };
    // 2 軸曲げ指定時は曲げ余裕度も判定に加える。
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

    Some(UltimateCheck {
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
    })
}

/// モデルの RC 矩形部材について終局検定（塑性理論式）を一括実行する。
///
/// - `demand_by_elem`: 部材の設計用需要（[`MemberDemand`]：圧縮正の軸力と強軸/弱軸の
///   設計用曲げモーメント）。柱の Mu・軸余裕度・2 軸曲げ余裕度に用いる。該当 ID がない
///   部材は需要 0（安全側）で評価する。軸力は長期（G+P）静的、曲げ需要は当該組合せの
///   応答値を渡すことを想定する。
/// - 対象外（`RcRect` 以外・断面/材料未解決・Fc 未設定・有効せい ≤ 0）の部材は
///   結果に含めない。
pub fn collect_rc_ultimate_checks(
    model: &Model,
    demand_by_elem: &[(ElemId, MemberDemand)],
    opts: &UltimateShearOptions,
) -> Vec<UltimateCheck> {
    let mut out = Vec::new();
    for elem in &model.elements {
        let Some(sec) = elem.section.and_then(|sid| model.sections.get(sid.index())) else {
            continue;
        };
        let Some(mat) = elem
            .material
            .and_then(|mid| model.materials.get(mid.index()))
        else {
            continue;
        };
        let demand = demand_by_elem
            .iter()
            .find(|(id, _)| *id == elem.id)
            .map(|(_, d)| *d)
            .unwrap_or_default();
        if let Some(check) = check_member(elem, sec, mat, model, demand, opts) {
            out.push(check);
        }
    }
    out
}
