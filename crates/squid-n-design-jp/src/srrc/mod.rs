//! SRC 造の断面検定（許容応力度検定。SRC 規準 1987 の
//! SRC 梁・SRC 柱部分に準拠）。
//!
//! - [`beam`][]: 鉄骨鉄筋コンクリート造梁の断面検定（累加強度式 MA=sMo+rMA）。
//! - [`column`][]: 鉄骨鉄筋コンクリート造柱の断面検定（累加強度式・fc′低減）。
//! - [`panel_zone`][]: SRC 造柱梁接合部（パネルゾーン）の断面検定（SRC 規準）。

use crate::{CheckOutcome, DesignCheck, DesignCtx, LoadTerm, MemberForcesAt, MemberKind};
use squid_n_core::model::{Material, Section};
use squid_n_core::rc_rebar_geom::RectEdge;
use squid_n_core::section_shape::{RcBeamRebar, RcRectColumnRebar, SectionShape};

mod beam;
/// 鉄骨鉄筋コンクリート造梁のせん断終局強度（非線形解析のせん断ばね終局耐力）。
/// SRC 梁せん断復元力特性（構造関係技術基準解説書・SRC 規準）。
pub mod beam_nonlinear;
mod column;
pub mod panel_zone;

pub(crate) use crate::ratio_or_large;
pub(crate) use crate::rc::{
    bar_set_area, rect_axis_props as src_rect_axis_props, shear_alpha, AxisProps as SrcAxisProps,
};

/// 内蔵鋼材の断面積・断面係数を [`SectionShape`] の断面性能計算を借りて
/// 求める（H 形鋼: `sA`, 強軸 `sZ`, 弱軸 `sZ`）。
fn steel_h_props(height: f64, width: f64, web_thick: f64, flange_thick: f64) -> (f64, f64, f64) {
    let shape = SectionShape::SteelH {
        height,
        width,
        web_thick,
        flange_thick,
    };
    let a = shape.calc_area();
    let iy = shape.calc_iy();
    let iz = shape.calc_iz();
    let sz_strong = if height > 0.0 { iy * 2.0 / height } else { 0.0 };
    let sz_weak = if width > 0.0 { iz * 2.0 / width } else { 0.0 };
    (a, sz_strong, sz_weak)
}

/// 実配筋梁（[`RcBeamRebar`]）の指定引張側における、RC 部分 1 軸分の断面諸元。
///
/// `tension_is_top` は上端側を引張とするか。応力中心間距離は `j = 7d/8`。
fn beam_axis_props(b: f64, d_full: f64, rebar: &RcBeamRebar, tension_is_top: bool) -> SrcAxisProps {
    let bending = rebar.bending_steel(d_full, tension_is_top);
    let tension = bending.tension;
    SrcAxisProps {
        b,
        d_full,
        dt: tension.centroid_from_edge_mm,
        d: tension.effective_depth_mm,
        at: tension.area_mm2,
        ac: bending.compression.area_mm2,
        j: 7.0 * tension.effective_depth_mm / 8.0,
        pw: rebar.pw(b),
    }
}

/// 実配筋矩形柱（[`RcRectColumnRebar`]）の強軸/弱軸における、RC 部分 1 軸分の断面諸元。
///
/// 強軸は上下辺、弱軸は左右辺の最外段 1 列を引張側とする。応力中心間距離は `j = 7d/8`。
fn column_axis_props(b: f64, d_full: f64, rebar: &RcRectColumnRebar, strong: bool) -> SrcAxisProps {
    let (b_dir, d_dir, edge, aw) = if strong {
        (b, d_full, RectEdge::Top, rebar.aw_x_mm2())
    } else {
        (d_full, b, RectEdge::Left, rebar.aw_y_mm2())
    };
    let steel = rebar.edge_steel(edge, b, d_full);
    let pw = if rebar.hoop.pitch > 0.0 {
        aw / (b_dir * rebar.hoop.pitch)
    } else {
        0.0
    };
    SrcAxisProps {
        b: b_dir,
        d_full: d_dir,
        dt: steel.centroid_from_edge_mm,
        d: steel.effective_depth_mm,
        at: steel.area_mm2,
        ac: steel.area_mm2,
        j: 7.0 * steel.effective_depth_mm / 8.0,
        pw,
    }
}

/// 実配筋梁の曲げ引張側。`mz>0` は下端引張、`mz<0` は上端引張とし、
/// `mz=0` は上下の引張鉄筋量が小さい側を引張とする。
fn beam_tension_is_top(forces: &MemberForcesAt, d_full: f64, rebar: &RcBeamRebar) -> bool {
    if forces.mz < 0.0 {
        true
    } else if forces.mz > 0.0 {
        false
    } else {
        rebar.bending_steel(d_full, true).tension.area_mm2
            < rebar.bending_steel(d_full, false).tension.area_mm2
    }
}

struct SrcShearResult {
    ratio: f64,
    s_q: f64,
    r_q: f64,
    s_qa: f64,
    r_qa: f64,
    alpha: f64,
    pw: f64,
    /// 地震時短期の構造規定方式（[`src_seismic_qd`]）で `s_q`/`r_q` を
    /// 算定したか（false の場合は弾性分担）。
    used_qd: bool,
}

/// SRC 梁・柱の地震時短期の設計用せん断力（構造規定方式）算定に必要な
/// 追加入力（[`src_seismic_qd`] 参照）。
struct SrcSeismicCtx<'a> {
    ctx: &'a DesignCtx,
    /// 評価位置（`ctx.seismic_qd.long_at` 検索用）。
    pos: f64,
    /// 長期内力配列 `[N,Qy,Qz,Mx,My,Mz]` のせん断成分位置（qy=1, qz=2）。
    q_index: usize,
    /// 鋼材の短期許容引張応力度 sft [N/mm²]
    /// （`steel_ft(f_value, LoadTerm::Short)`。長短期どちらの検定でも
    /// QD 割増自体は短期時のみ発動するため、常に短期値を用いる）。
    s_ft_short: f64,
    /// RC 部分の終局曲げモーメント rMu（部材端 1 箇所分）[N·mm]
    /// （[`squid_n_core::rc_capacity::rc_mu_simple`]/
    /// [`squid_n_core::rc_capacity::rc_column_mu_simple`] で算定）。
    /// 0 以下なら rQD1 は無効（rQD2 のみ）とする。
    r_mu: f64,
}

/// SRC 梁・柱の地震時短期の設計用せん断力（構造規定方式、SRC 規準 1987）。
/// `seismic.ctx.seismic_qd` が None、または長期内力に
/// 同一評価位置が見つからない場合は None を返す（呼び出し側は弾性
/// 分担にフォールバックする）。
///
/// - `sQD = sQL + (sM1+sM2)/l′`（`sQL = share・|QL|`、`sM1+sM2 = 2・sZ・sft`）
/// - `rQD1 = rQL + (rMu1+rMu2)/l′`（`rQL = |QL| − sQL`、`rMu1+rMu2 = 2・rMu`）
/// - `rQD2 = max(0, n・(|Q| − sQD))`
///   （SRC規準1987 の `rQD2 = n・(QL+QE−sQD)` を、`QL+QE` = 当該組合せの
///   全せん断力 `|Q|` と読んだもの。`QE` は水平力分のせん断力増分であり、
///   `QL+QE` はその和として組合せ後の全せん断力に一致するとみなした）
/// - `rQD = min(rQD1, rQD2)`（[`crate::QdMethod::Qd1`]/[`crate::QdMethod::Qd2`]
///   選択時はそれぞれ単独。`Qd1` で rQD1 が無効な場合は rQD2 で代替する）
/// - 戻り値は `(sQD, rQD)`。
fn src_seismic_qd(
    seismic: &SrcSeismicCtx,
    q_signed: f64,
    share: f64,
    sz: f64,
) -> Option<(f64, f64)> {
    let qd = seismic.ctx.seismic_qd.as_ref()?;
    let ql_signed = qd
        .long_at
        .iter()
        .find(|(p, _)| (p - seismic.pos).abs() < 1e-6)
        .map(|(_, f)| f[seismic.q_index])?;

    let ql = ql_signed.abs();
    let q = q_signed.abs();

    let s_ql = share * ql;
    let sum_s_m = 2.0 * sz * seismic.s_ft_short;
    let s_qd = if qd.clear_length > 0.0 {
        s_ql + sum_s_m / qd.clear_length
    } else {
        s_ql
    };

    let r_ql = (ql - s_ql).max(0.0);
    let r_qd1 = if qd.clear_length > 0.0 && seismic.r_mu > 0.0 {
        let sum_r_mu = 2.0 * seismic.r_mu;
        r_ql + sum_r_mu / qd.clear_length
    } else {
        f64::INFINITY
    };
    let r_qd2 = (qd.n_factor * (q - s_qd)).max(0.0);

    let r_qd = qd.method.resolve(r_qd1, r_qd2);

    Some((s_qd, r_qd))
}

/// せん断検定の部材モード（SRC規準 1987 の梁・柱の式の切替）。
enum SrcShearMode {
    /// 梁: 鉄骨・RC の分担検定（長短共通）。
    /// `rQA1 = b·rj·(rα·fs + 0.5·pw·wft)`、`rQA2 = b·rj·(2(b′/b)·fs + pw·wft)`。
    Beam,
    /// 柱（SRC規準 P.96-97）:
    /// - 長期は併用式 `QA = (1+β)·b·rj·a′·fs` を全せん断力と比較する
    ///   （`a′ = rα`（`b′/b ≥ rα/3` のとき）または `3b′/b`、`1 ≤ rα ≤ 2`）。
    /// - 短期は鉄骨部 `sQA`（強軸 `dw·tw·sfs`／弱軸 `(4/3)·bf·tf·sfs`）と
    ///   RC 部 `rQAS1 = b·rj·(fs + 0.5·pw·wft)`（**α を含まない**）・
    ///   `rQAS2 = b·rj·(2(b′/b)·fs + pw·wft)` を分担 `sQD`/`rQD` と比較する。
    ///
    /// `beta` は鉄骨ウェブの形式と寸法による係数
    /// （充腹 `β = n·tw·sd/(b·rj)`、弱軸・非充腹 `β = 1.33·n·bf·tf/(b·rj)`）。
    Column { beta: f64 },
}

/// 全せん断力を鉄骨部分・RC 部分に分担させ、それぞれの許容せん断力と比較する
/// （梁・柱の両方向で共通利用。式の切替は [`SrcShearMode`]）。
/// `seismic.ctx.seismic_qd` が Some で当該評価位置の長期内力が見つかる場合は
/// 地震時短期の構造規定方式（[`src_seismic_qd`]）による設計用せん断力
/// `sQD`/`rQD` を用い、それ以外は弾性分担 `share = sz/(sz+at・rj)` を当該
/// 組合せの全せん断力にそのまま適用して代替する。
#[allow(clippy::too_many_arguments)]
fn src_shear_check(
    q_signed: f64,
    m_for_alpha: f64,
    q_for_alpha: f64,
    sz: f64,
    at: f64,
    rj: f64,
    rd: f64,
    b: f64,
    b_prime: f64,
    pw_raw: f64,
    fs: f64,
    w_ft: f64,
    s_fs: f64,
    steel_shear_area: f64,
    alpha_max: f64,
    mode: &SrcShearMode,
    seismic: &SrcSeismicCtx,
) -> SrcShearResult {
    let alpha = shear_alpha(m_for_alpha, q_for_alpha, rd, alpha_max);
    let q = q_signed.abs();

    let pw_cap = 0.006;
    let pw = pw_raw.min(pw_cap);

    let b_ratio = if b > 1e-9 {
        (b_prime / b).max(0.0)
    } else {
        0.0
    };

    if let SrcShearMode::Column { beta } = mode {
        if seismic.ctx.term == LoadTerm::Long {
            let a_prime = if b_ratio >= alpha / 3.0 {
                alpha
            } else {
                3.0 * b_ratio
            };
            let qa = (1.0 + beta) * b * rj * a_prime * fs;
            let ratio = if qa > 1e-9 { q / qa } else { 0.0 };
            return SrcShearResult {
                ratio,
                s_q: 0.0,
                r_q: q,
                s_qa: steel_shear_area * s_fs,
                r_qa: qa,
                alpha,
                pw,
                used_qd: false,
            };
        }
    }

    let denom = sz + at * rj;
    let share = if denom > 1e-12 { sz / denom } else { 1.0 };

    let (s_q, r_q, used_qd) = match src_seismic_qd(seismic, q_signed, share, sz) {
        Some((s_qd, r_qd)) => (s_qd, r_qd, true),
        None => {
            let s_q = share * q;
            (s_q, (q - s_q).max(0.0), false)
        }
    };

    let s_qa = steel_shear_area * s_fs;

    let r_qa1 = match mode {
        SrcShearMode::Beam => b * rj * (alpha * fs + 0.5 * pw * w_ft),
        SrcShearMode::Column { .. } => b * rj * (fs + 0.5 * pw * w_ft),
    };
    let r_qa2 = b * rj * (2.0 * b_ratio * fs + pw * w_ft);
    let r_qa = r_qa1.min(r_qa2);

    let ratio_s = if s_qa > 1e-9 { s_q / s_qa } else { 0.0 };
    let ratio_r = if r_qa > 1e-9 { r_q / r_qa } else { 0.0 };

    SrcShearResult {
        ratio: ratio_s.max(ratio_r),
        s_q,
        r_q,
        s_qa,
        r_qa,
        alpha,
        pw,
        used_qd,
    }
}

/// SRC 梁・SRC 柱の断面検定
/// （`SectionShape::SrcRect`・`SrcBeamRect`・`SrcColumnRect` を対象とする）。
pub struct SrcDesign;

impl DesignCheck for SrcDesign {
    fn check(
        &self,
        forces: &MemberForcesAt,
        sec: &Section,
        mat: &Material,
        ctx: &DesignCtx,
    ) -> CheckOutcome {
        let fc_raw = mat.fc.unwrap_or(0.0);
        if fc_raw <= 0.0 {
            return CheckOutcome::Skipped {
                reason: "SRC検定: Fc未設定（Material.fc が None/0 です）".to_string(),
            };
        }

        let shape = match &sec.shape {
            Some(
                s @ (SectionShape::SrcRect { .. }
                | SectionShape::SrcBeamRect { .. }
                | SectionShape::SrcColumnRect { .. }),
            ) => s,
            _ => {
                return CheckOutcome::Skipped {
                    reason: "SRC検定: 断面形状不一致（Section.shape が SrcRect/SrcBeamRect/\
                             SrcColumnRect ではありません）"
                        .to_string(),
                };
            }
        };

        if ctx.rebar_material.is_none() {
            return CheckOutcome::Skipped {
                reason: "SRC検定: 主筋の材料が未割当（断面タブで主筋の材料を割り当ててください）"
                    .to_string(),
            };
        }
        if ctx.shear_rebar_material.is_none() {
            return CheckOutcome::Skipped {
                reason: "SRC検定: せん断補強筋の材料が未割当\
                         （断面タブでせん断補強筋の材料を割り当ててください）"
                    .to_string(),
            };
        }
        if let Some(msg) = squid_n_core::material_grade::shear_rebar_material_issue(
            ctx.shear_rebar_material.as_ref(),
        ) {
            return CheckOutcome::Skipped {
                reason: format!("SRC検定: {msg}"),
            };
        }
        let Some(steel_mat) = ctx.steel_material.as_ref() else {
            return CheckOutcome::Skipped {
                reason: "SRC検定: 内蔵鉄骨の材料が未割当\
                         （断面タブで内蔵鉄骨の材料を割り当ててください）"
                    .to_string(),
            };
        };
        let steel_grade = steel_mat.name.as_str();

        let cr = match shape {
            SectionShape::SrcRect {
                b,
                d,
                rebar,
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
            } => match ctx.kind {
                MemberKind::Beam | MemberKind::Brace => {
                    let props = src_rect_axis_props(*b, *d, &rebar.main_x, rebar);
                    beam::src_beam_check(
                        forces,
                        mat,
                        ctx,
                        props,
                        props,
                        rebar.main_x.dia,
                        *steel_height,
                        *steel_width,
                        *steel_web_thick,
                        *steel_flange_thick,
                        steel_grade,
                        fc_raw,
                    )
                }
                MemberKind::Column => {
                    let props_z = src_rect_axis_props(*b, *d, &rebar.main_x, rebar);
                    let props_y = src_rect_axis_props(*d, *b, &rebar.main_y, rebar);
                    let as_x = bar_set_area(&rebar.main_x);
                    let as_y = bar_set_area(&rebar.main_y);
                    column::src_column_check(
                        forces,
                        mat,
                        ctx,
                        props_z,
                        props_y,
                        as_y,
                        as_x,
                        as_x + as_y,
                        rebar.main_x.dia,
                        rebar.main_y.dia,
                        *steel_height,
                        *steel_width,
                        *steel_web_thick,
                        *steel_flange_thick,
                        steel_grade,
                        fc_raw,
                    )
                }
            },
            SectionShape::SrcBeamRect {
                b,
                d,
                rebar,
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
            } => {
                if ctx.kind == MemberKind::Column {
                    return CheckOutcome::Skipped {
                        reason: "SRC検定: 梁用断面を柱部材に割り当てています（用途不一致）"
                            .to_string(),
                    };
                }
                if rebar.is_unset() {
                    return CheckOutcome::Skipped {
                        reason: "SRC検定: 配筋が未入力です".to_string(),
                    };
                }
                if let Err(e) = rebar.validate(*b, *d) {
                    return CheckOutcome::Skipped {
                        reason: format!("SRC検定: 配筋が不整合です（{e}）"),
                    };
                }
                let tension_is_top = beam_tension_is_top(forces, *d, rebar);
                let props = beam_axis_props(*b, *d, rebar, tension_is_top);
                let props_other = beam_axis_props(*b, *d, rebar, !tension_is_top);
                beam::src_beam_check(
                    forces,
                    mat,
                    ctx,
                    props,
                    props_other,
                    rebar.main_dia,
                    *steel_height,
                    *steel_width,
                    *steel_web_thick,
                    *steel_flange_thick,
                    steel_grade,
                    fc_raw,
                )
            }
            SectionShape::SrcColumnRect {
                b,
                d,
                rebar,
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
            } => {
                if matches!(ctx.kind, MemberKind::Beam | MemberKind::Brace) {
                    return CheckOutcome::Skipped {
                        reason: "SRC検定: 柱用断面を梁部材に割り当てています（用途不一致）"
                            .to_string(),
                    };
                }
                if rebar.is_unset() {
                    return CheckOutcome::Skipped {
                        reason: "SRC検定: 配筋が未入力です".to_string(),
                    };
                }
                if let Err(e) = rebar.validate(*b, *d) {
                    return CheckOutcome::Skipped {
                        reason: format!("SRC検定: 配筋が不整合です（{e}）"),
                    };
                }
                let props_z = column_axis_props(*b, *d, rebar, true);
                let props_y = column_axis_props(*b, *d, rebar, false);
                column::src_column_check(
                    forces,
                    mat,
                    ctx,
                    props_z,
                    props_y,
                    rebar.y_direction_area_mm2(),
                    rebar.x_direction_area_mm2(),
                    rebar.total_main_area(),
                    rebar.main_dia,
                    rebar.main_dia,
                    *steel_height,
                    *steel_width,
                    *steel_web_thick,
                    *steel_flange_thick,
                    steel_grade,
                    fc_raw,
                )
            }
            _ => unreachable!(),
        };
        CheckOutcome::Checked(cr)
    }
}

#[cfg(test)]
pub(crate) mod tests;
