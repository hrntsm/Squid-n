//! CFT 造の断面検定（許容応力度検定）。SRC 規準の累加強度式を
//! CFT 柱（コンクリート充填鋼管）に準用する（相互拘束効果による
//! コンクリート強度割増しは考慮しない）。
//!
//! コンクリート強度未設定の断面は検定をスキップし、内蔵鉄骨の鋼種を
//! 解決できない場合は SS400 相当（F=235）で検定する。
//!
//! # モジュール構成
//! CFT はトップレベルの単一モジュール（`crate::cft`、本ファイル）とし、
//! [`crate::srrc`] の共通ヘルパ（断面諸元抽出等）には依存しない
//! （断面形状・N-M 相関の算定方法が SRC 矩形断面と異なるため）。

use crate::rc::concrete_allowable_compression_class;
use crate::steel::{steel_f_value_prefix, steel_fc, steel_fs, steel_ft};
use crate::{
    effective_slenderness, CheckComponent, CheckKind, CheckOutcome, CheckResult, DesignCheck,
    DesignCtx, LoadTerm, MemberForcesAt,
};
use squid_n_core::model::{Material, Section};
use squid_n_core::section_shape::SectionShape;

use crate::ratio_or_large;

/// 矩形充填コンクリート部分の (cN, cM) を弾性三角形応力分布の閉形式で求める。
/// `xn`: 中立軸位置（圧縮縁からの距離）[mm]、`cb`/`cd`: 検討方向の充填断面
/// 幅・せい [mm]。
fn cft_rect_cn_cm(cb: f64, cd: f64, fc: f64, xn: f64) -> (f64, f64) {
    if cb <= 0.0 || cd <= 0.0 || fc <= 0.0 || xn <= 0.0 {
        return (0.0, 0.0);
    }
    let xr = xn / cd;
    if xr <= 1.0 {
        let cn = cb * cd * fc * (xr / 2.0);
        let cm = cb * cd * cd * fc * (xr * (3.0 - 2.0 * xr) / 12.0);
        (cn, cm)
    } else {
        let cn = cb * cd * fc * (1.0 - 1.0 / (2.0 * xr));
        let cm = cb * cd * cd * fc * (1.0 / (12.0 * xr));
        (cn, cm)
    }
}

/// 設計軸力 `n_design`（0≤N<cNc）に対する矩形充填コンクリート部分の cM を、
/// 閉形式の逆算（cN(xn)=N となる xn を解く）で求める。
fn cft_rect_ma(cb: f64, cd: f64, fc: f64, n_design: f64) -> f64 {
    let cnc = cb * cd * fc;
    if cnc <= 0.0 || n_design <= 0.0 {
        return 0.0;
    }
    let ratio = (n_design / cnc).clamp(0.0, 1.0 - 1e-9);
    let xr = if ratio <= 0.5 {
        2.0 * ratio
    } else {
        1.0 / (2.0 * (1.0 - ratio))
    };
    let xn = xr * cd;
    let (_, cm) = cft_rect_cn_cm(cb, cd, fc, xn);
    cm
}

/// 縁応力 `fc` 一定・線形分布（コンクリート引張無視）を断面内で数値積分し、
/// 任意断面形状の (cN, cM) を求める汎用ヘルパ。`width_fn(y)` は圧縮縁からの
/// 距離 `y` [mm] における断面幅 [mm] を返す。
fn numeric_cn_cm(cd: f64, fc: f64, xn: f64, width_fn: impl Fn(f64) -> f64) -> (f64, f64) {
    if cd <= 0.0 || fc <= 0.0 || xn <= 0.0 {
        return (0.0, 0.0);
    }
    let y_max = xn.min(cd);
    if y_max <= 0.0 {
        return (0.0, 0.0);
    }
    let n_steps = 400usize;
    let dy = y_max / n_steps as f64;
    let center = cd / 2.0;
    let mut cn = 0.0;
    let mut cm = 0.0;
    for i in 0..n_steps {
        let y = (i as f64 + 0.5) * dy;
        let sigma = (fc * (1.0 - y / xn)).max(0.0);
        let width = width_fn(y);
        let df = sigma * width * dy;
        cn += df;
        cm += df * (center - y);
    }
    (cn, cm.abs())
}

/// 円形充填コンクリート部分の (cN, cM)（数値積分、矩形と同じ弾性仮定）。
/// `dc`: 充填部直径 [mm]。
fn cft_circle_cn_cm(dc: f64, fc: f64, xn: f64) -> (f64, f64) {
    numeric_cn_cm(dc, fc, xn, |y| {
        let r = dc / 2.0;
        2.0 * (r * r - (y - r).powi(2)).max(0.0).sqrt()
    })
}

/// 設計軸力 `n_design`（0≤N<cNc）に対する円形充填コンクリート部分の cM を、
/// 二分法で cN(xn)=N となる xn を求めて算定する。
fn cft_circle_ma(dc: f64, fc: f64, n_design: f64) -> f64 {
    if dc <= 0.0 || fc <= 0.0 || n_design <= 0.0 {
        return 0.0;
    }
    let cnc = std::f64::consts::PI * dc * dc / 4.0 * fc;
    if cnc <= 0.0 {
        return 0.0;
    }
    let target = n_design.min(cnc * (1.0 - 1e-6));
    let mut lo = 1e-6 * dc;
    let mut hi = 50.0 * dc;
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        let (cn, _) = cft_circle_cn_cm(dc, fc, mid);
        if cn < target {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let xn = 0.5 * (lo + hi);
    let (_, cm) = cft_circle_cn_cm(dc, fc, xn);
    cm
}

/// CFT 柱 1 軸分の許容曲げモーメント MA(N)。累加強度式による 3 分岐
/// （コンクリート+鋼管累加 / 圧縮超過で鋼管のみ / 引張で鋼管のみ）を実装する。
/// `cm_fn`: 0≤N≤cNc の範囲でコンクリート部分の cM(N) を返す関数
/// （矩形は [`cft_rect_ma`]、円形は [`cft_circle_ma`]）。
fn cft_axis_capacity(
    n_design: f64,
    cnc: f64,
    sa: f64,
    s_ft: f64,
    s_fc: f64,
    sz: f64,
    cm_fn: impl Fn(f64) -> f64,
) -> f64 {
    if n_design < 0.0 {
        (sz * (s_ft - (-n_design) / sa)).max(0.0)
    } else if n_design <= cnc {
        sz * s_ft + cm_fn(n_design)
    } else {
        let sn = n_design - cnc;
        (sz * (s_fc - sn / sa)).max(0.0)
    }
}

/// CFT 柱の鋼管部分の許容応力度 (s_ft, s_fs, s_fc)。
///
/// s_fc は座屈を考慮する（SRC 規準準用。細長比の記号
/// λ=Lk/i・Lk/D に対応）。細長比 λ は**鋼管単体**の断面二次半径で評価する
/// （充填コンクリートの曲げ剛性寄与を無視するため実際より λ が大きく
/// 算定され、安全側）。λ=0（座屈長さ 0）のとき s_fc は長期 F/1.5（=s_ft）
/// に一致する。
fn cft_common_steel(f_value: f64, term: LoadTerm, lambda: f64) -> (f64, f64, f64) {
    let s_ft = steel_ft(f_value, term);
    let s_fs = steel_fs(f_value, term);
    let s_fc = steel_fc(f_value, lambda, term);
    (s_ft, s_fs, s_fc)
}

fn cft_box_steel_props(height: f64, width: f64, thick: f64) -> (f64, f64, f64) {
    let shape = SectionShape::CftBox {
        height,
        width,
        thick,
    };
    let a = shape.calc_area();
    let iy = shape.calc_iy();
    let iz = shape.calc_iz();
    let sz_mz = if height > 0.0 { iy * 2.0 / height } else { 0.0 };
    let sz_my = if width > 0.0 { iz * 2.0 / width } else { 0.0 };
    (a, sz_mz, sz_my)
}

fn cft_pipe_steel_props(outer_dia: f64, thick: f64) -> (f64, f64) {
    let shape = SectionShape::CftPipe { outer_dia, thick };
    let a = shape.calc_area();
    let iy = shape.calc_iy();
    let sz = if outer_dia > 0.0 {
        iy * 2.0 / outer_dia
    } else {
        0.0
    };
    (a, sz)
}

/// CFT 柱の設計用せん断力 `QD`（SRC 規準準用）。
///
/// - `QD1 = ΣcMy/h′`。cMy は CFT 指針の N-M 相互作用による終局曲げ耐力
///   Mu(N)（[`crate::ultimate::cft_mu_nm`]、柱分類対応）とし、柱頭・柱脚
///   同一断面の仮定で `ΣcMy = 2·Mu(N)` とする（RC 柱の QD1 と同じ扱い）。
/// - `QD2 = |QL| + n・|Q−QL|`。
/// - `ctx.seismic_qd.method`（QD1/QD2/min）の選択は RC と共通の
///   [`crate::rc::seismic_design_shear`] に委譲する。
///
/// `ctx.seismic_qd` が None、または長期内力に同一評価位置が見つからない
/// 場合は解析せん断力 `|q_signed|` をそのまま返す。
fn cft_q_design(ctx: &DesignCtx, pos: f64, q_signed: f64, q_index: usize, sum_c_my: f64) -> f64 {
    crate::rc::seismic_design_shear(ctx, pos, q_signed, q_index, sum_c_my, true)
}

fn cft_box_check(
    forces: &MemberForcesAt,
    mat: &Material,
    ctx: &DesignCtx,
    height: f64,
    width: f64,
    thick: f64,
    fc_raw: f64,
) -> CheckResult {
    let long_term = ctx.term == LoadTerm::Long;
    let fc_allow = concrete_allowable_compression_class(fc_raw, mat.concrete_class, long_term);

    let f_value = steel_f_value_prefix(&mat.name, thick)
        .or(mat.fy)
        .unwrap_or(235.0);
    let (sa, sz_z, sz_y) = cft_box_steel_props(height, width, thick);
    let shape = SectionShape::CftBox {
        height,
        width,
        thick,
    };
    let lambda = effective_slenderness(
        shape.calc_iy(),
        shape.calc_iz(),
        sa,
        ctx.length,
        ctx.lk_y,
        ctx.lk_z,
    );
    let (s_ft, s_fs, s_fc) = cft_common_steel(f_value, ctx.term, lambda);
    let s_nt = sa * s_ft;
    let s_nc = sa * s_fc;

    let c_b_z = (width - 2.0 * thick).max(0.0);
    let c_d_z = (height - 2.0 * thick).max(0.0);
    let c_b_y = c_d_z;
    let c_d_y = c_b_z;

    let c_area = c_b_z * c_d_z;
    let cnc = c_area * fc_allow;

    let n_design = -forces.n;

    let ma_z = cft_axis_capacity(n_design, cnc, sa, s_ft, s_fc, sz_z, |n| {
        cft_rect_ma(c_b_z, c_d_z, fc_allow, n)
    });
    let ma_y = cft_axis_capacity(n_design, cnc, sa, s_ft, s_fc, sz_y, |n| {
        cft_rect_ma(c_b_y, c_d_y, fc_allow, n)
    });

    let ratio_z = ratio_or_large(forces.mz, ma_z);
    let ratio_y = ratio_or_large(forces.my, ma_y);
    let ratio_biaxial = ratio_z + ratio_y;

    let ratio_axial = if n_design > cnc + s_nc {
        n_design / (cnc + s_nc)
    } else if n_design < 0.0 && (-n_design) > s_nt {
        (-n_design) / s_nt
    } else {
        0.0
    };

    let s_aw_y = 2.0 * thick * (height - 2.0 * thick).max(0.0);
    let s_aw_z = 2.0 * thick * (width - 2.0 * thick).max(0.0);
    let s_qa_y = s_aw_y * s_fs;
    let s_qa_z = s_aw_z * s_fs;
    let (sum_c_my_z, sum_c_my_y) = if ctx.seismic_qd.is_some() {
        let shape = SectionShape::CftBox {
            height,
            width,
            thick,
        };
        let lk_y = ctx.lk_y.unwrap_or(ctx.length);
        let lk_z = ctx.lk_z.unwrap_or(ctx.length);
        let mu_z = crate::ultimate::cft_mu_nm(&shape, fc_raw, f_value, n_design, lk_y, false)
            .unwrap_or(0.0);
        let mu_y = crate::ultimate::cft_mu_nm(&shape, fc_raw, f_value, n_design, lk_z, true)
            .unwrap_or(0.0);
        (2.0 * mu_z, 2.0 * mu_y)
    } else {
        (0.0, 0.0)
    };
    let q_design_y = cft_q_design(ctx, forces.pos, forces.qy, 1, sum_c_my_z);
    let q_design_z = cft_q_design(ctx, forces.pos, forces.qz, 2, sum_c_my_y);
    let ratio_shear_y = if s_qa_y > 1e-9 {
        q_design_y / s_qa_y
    } else {
        0.0
    };
    let ratio_shear_z = if s_qa_z > 1e-9 {
        q_design_z / s_qa_z
    } else {
        0.0
    };
    let ratio_shear = ratio_shear_y.max(ratio_shear_z);

    let basis = "CFT柱(角形): SRC規準に基づく累加強度式".to_string();
    let axial_bending_detail = format!(
        "cNc={:.1} N, sNc={:.1} N, sNt={:.1} N, N={:.1} N, MAz={:.1} N·mm, MAy={:.1} N·mm, \
         mz={:.1} N·mm, my={:.1} N·mm",
        cnc, s_nc, s_nt, n_design, ma_z, ma_y, forces.mz, forces.my,
    );
    let shear_detail = format!(
        "sQAy={:.1} N, sQAz={:.1} N, qy={:.1} N, qz={:.1} N",
        s_qa_y, s_qa_z, forces.qy, forces.qz
    );
    let detail = String::new();

    let components = vec![
        CheckComponent {
            kind: CheckKind::AxialBending,
            ratio: ratio_axial.max(ratio_biaxial),
            detail: axial_bending_detail,
        },
        CheckComponent {
            kind: CheckKind::Shear,
            ratio: ratio_shear,
            detail: shear_detail,
        },
    ];

    CheckResult {
        basis,
        detail,
        components,
    }
}

fn cft_pipe_check(
    forces: &MemberForcesAt,
    mat: &Material,
    ctx: &DesignCtx,
    outer_dia: f64,
    thick: f64,
    fc_raw: f64,
) -> CheckResult {
    let long_term = ctx.term == LoadTerm::Long;
    let fc_allow = concrete_allowable_compression_class(fc_raw, mat.concrete_class, long_term);

    let f_value = steel_f_value_prefix(&mat.name, thick)
        .or(mat.fy)
        .unwrap_or(235.0);
    let (sa, sz) = cft_pipe_steel_props(outer_dia, thick);
    let shape = SectionShape::CftPipe { outer_dia, thick };
    let iy = shape.calc_iy();
    let lambda = effective_slenderness(iy, iy, sa, ctx.length, ctx.lk_y, ctx.lk_z);
    let (s_ft, s_fs, s_fc) = cft_common_steel(f_value, ctx.term, lambda);
    let s_nt = sa * s_ft;
    let s_nc = sa * s_fc;

    let dc = (outer_dia - 2.0 * thick).max(0.0);
    let c_area = std::f64::consts::PI * dc * dc / 4.0;
    let cnc = c_area * fc_allow;

    let n_design = -forces.n;

    let ma = cft_axis_capacity(n_design, cnc, sa, s_ft, s_fc, sz, |n| {
        cft_circle_ma(dc, fc_allow, n)
    });

    let ratio_z = ratio_or_large(forces.mz, ma);
    let ratio_y = ratio_or_large(forces.my, ma);
    let ratio_biaxial = ratio_z + ratio_y;

    let ratio_axial = if n_design > cnc + s_nc {
        n_design / (cnc + s_nc)
    } else if n_design < 0.0 && (-n_design) > s_nt {
        (-n_design) / s_nt
    } else {
        0.0
    };

    let s_aw = sa / 2.0;
    let s_qa = s_aw * s_fs;
    let sum_c_my = if ctx.seismic_qd.is_some() {
        let shape = SectionShape::CftPipe { outer_dia, thick };
        let lk = ctx
            .lk_y
            .unwrap_or(ctx.length)
            .max(ctx.lk_z.unwrap_or(ctx.length));
        2.0 * crate::ultimate::cft_mu_nm(&shape, fc_raw, f_value, n_design, lk, false)
            .unwrap_or(0.0)
    } else {
        0.0
    };
    let q_design_y = cft_q_design(ctx, forces.pos, forces.qy, 1, sum_c_my);
    let q_design_z = cft_q_design(ctx, forces.pos, forces.qz, 2, sum_c_my);
    let q_res = (q_design_y.powi(2) + q_design_z.powi(2)).sqrt();
    let ratio_shear = if s_qa > 1e-9 { q_res / s_qa } else { 0.0 };

    let basis = "CFT柱(円形): SRC規準に基づく累加強度式".to_string();
    let axial_bending_detail = format!(
        "cNc={:.1} N, sNc={:.1} N, sNt={:.1} N, N={:.1} N, MA={:.1} N·mm, mz={:.1} N·mm, \
         my={:.1} N·mm",
        cnc, s_nc, s_nt, n_design, ma, forces.mz, forces.my,
    );
    let shear_detail = format!(
        "sQA={:.1} N, qy={:.1} N, qz={:.1} N",
        s_qa, forces.qy, forces.qz
    );
    let detail = String::new();

    let components = vec![
        CheckComponent {
            kind: CheckKind::AxialBending,
            ratio: ratio_axial.max(ratio_biaxial),
            detail: axial_bending_detail,
        },
        CheckComponent {
            kind: CheckKind::Shear,
            ratio: ratio_shear,
            detail: shear_detail,
        },
    ];

    CheckResult {
        basis,
        detail,
        components,
    }
}

/// CFT 柱の断面検定（`SectionShape::CftBox`/`CftPipe` を対象とする）。
/// 準拠規準に CFT 梁の規定はないため、`ctx.kind` に依らず柱の検定式を
/// 適用する。
pub struct CftDesign;

impl DesignCheck for CftDesign {
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
                reason: "CFT検定: Fc未設定（Material.fc が None/0 です）".to_string(),
            };
        }

        let cr = match &sec.shape {
            Some(SectionShape::CftBox {
                height,
                width,
                thick,
            }) => cft_box_check(forces, mat, ctx, *height, *width, *thick, fc_raw),
            Some(SectionShape::CftPipe { outer_dia, thick }) => {
                cft_pipe_check(forces, mat, ctx, *outer_dia, *thick, fc_raw)
            }
            _ => {
                return CheckOutcome::Skipped {
                    reason:
                        "CFT検定: 断面形状不一致（Section.shape が CftBox/CftPipe ではありません）"
                            .to_string(),
                };
            }
        };
        CheckOutcome::Checked(cr)
    }
}

#[cfg(test)]
mod tests;
