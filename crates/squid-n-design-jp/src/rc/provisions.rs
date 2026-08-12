//! RC 部材の構造規定（マニュアル 2.5.2 / 2.5.3）と梁の長期たわみ略算。
//!
//! - **入力エラー** → [`CheckKind::Provision`]（検定比 > 1 で NG）
//! - **警告** → 共通 detail へ付記（検定比には載せない）
//!
//! 計算ルート依存の項目（割増率下限・pw のルート別下限）はルート設定が未実装のため
//! 対象外（pw はルート1/3 相当の 0.2% 下限のみ警告）。

use squid_n_core::section_shape::{RcRebar, SectionShape};
use squid_n_core::units::ConcreteClass;

use super::section_props::{pw_ratio, AxisProps};
use crate::{CheckComponent, CheckKind, DesignCtx, LoadTerm};

/// 構造規定の判定結果。
pub(crate) struct ProvisionCheck {
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
}

impl ProvisionCheck {
    pub(crate) fn provision_component(&self) -> Option<CheckComponent> {
        if self.errors.is_empty() {
            return None;
        }
        Some(CheckComponent {
            kind: CheckKind::Provision,
            ratio: 1.0 + self.errors.len() as f64 * 0.01,
            detail: format!("構造規定エラー: {}", self.errors.join(" / ")),
        })
    }

    pub(crate) fn warning_suffix(&self) -> String {
        if self.warnings.is_empty() {
            String::new()
        } else {
            format!("; ⚠ {}", self.warnings.join(" / "))
        }
    }
}

/// 部材レベルの項目を付記する位置か。
///
/// 危険断面の既定は柱フェイスと中央であり、剛域ありでは節点芯（pos=0）は
/// 検定対象外になる。中央（pos≈0.5）は常に危険断面に含まれるため、ここで
/// 一度だけ付記する（重複を避ける）。
pub(crate) fn is_member_level_station(pos: f64) -> bool {
    (pos - 0.5).abs() < 1e-6
}

/// RC 梁の構造規定（マニュアル 2.5.2(5)）。
pub(crate) fn beam_provisions(
    d_full: f64,
    props: &AxisProps,
    rebar: &RcRebar,
    long_term: bool,
    mz_abs: f64,
    ft: f64,
) -> ProvisionCheck {
    let mut errors = Vec::new();
    let mut warnings = Vec::new();

    if rebar.cover + 1e-9 < 30.0 {
        errors.push(format!(
            "かぶり厚 {:.0} mm < 30 mm（入力エラー）",
            rebar.cover
        ));
    }
    let pitch = rebar.shear.pitch;
    if pitch > 0.0 {
        let lim_err = (0.75 * d_full).min(300.0);
        if pitch > lim_err + 1e-9 {
            errors.push(format!(
                "あばら間隔 {:.0} mm > min(3/4·D, 300)={:.0} mm（入力エラー）",
                pitch, lim_err
            ));
        }
        let lim_warn = (0.5 * d_full).min(250.0);
        if pitch > lim_warn + 1e-9 {
            warnings.push(format!(
                "あばら間隔 {:.0} mm > min(D/2, 250)={:.0} mm",
                pitch, lim_warn
            ));
        }
    }
    if rebar.main_x.dia + 1e-9 < 13.0 {
        warnings.push(format!("主筋径 D{:.0} < D13", rebar.main_x.dia));
    }
    if props.pw + 1e-12 < 0.002 {
        warnings.push(format!("pw={:.4} < 0.2%", props.pw));
    }
    if long_term && ft > 0.0 && props.j > 0.0 && props.d > 0.0 && props.b > 0.0 {
        let at_min_geom = 0.004 * props.b * props.d;
        let at_req = if mz_abs > 0.0 {
            mz_abs / (ft * props.j)
        } else {
            0.0
        };
        let at_need = if at_req > 0.0 {
            at_min_geom.min((4.0 / 3.0) * at_req)
        } else {
            at_min_geom
        };
        if props.at + 1e-9 < at_need {
            warnings.push(format!(
                "引張鉄筋 at={:.0} mm² < min(0.004bd, 4/3·必要量)={:.0} mm²",
                props.at, at_need
            ));
        }
    }

    ProvisionCheck { errors, warnings }
}

/// RC 柱の構造規定（マニュアル 2.5.3(5)）。
///
/// `length` は支点間距離 [mm]。呼び出し側は内法（`DesignCtx.clear_length`）を
/// 優先して渡す。
#[allow(clippy::too_many_arguments)]
pub(crate) fn column_provisions(
    shape: &SectionShape,
    rebar: &RcRebar,
    d_min: f64,
    length: f64,
    concrete_class: ConcreteClass,
    long_term: bool,
    ag: f64,
    as_total: f64,
    n_short_comp: f64,
    fc_raw: f64,
) -> ProvisionCheck {
    let mut errors = Vec::new();
    let mut warnings = Vec::new();

    if rebar.cover + 1e-9 < 30.0 {
        errors.push(format!(
            "かぶり厚 {:.0} mm < 30 mm（入力エラー）",
            rebar.cover
        ));
    }

    let (min_bars, main_count) = match shape {
        SectionShape::RcCircle { .. } => (8u32, rebar.main_x.count),
        _ => (4u32, rebar.main_x.count + rebar.main_y.count),
    };
    if main_count < min_bars {
        errors.push(format!(
            "主筋全本数 {main_count} < 最小 {min_bars}（入力エラー）"
        ));
    }

    if rebar.shear.pitch > 100.0 + 1e-9 {
        errors.push(format!(
            "帯筋間隔 {:.0} mm > 100 mm（入力エラー）",
            rebar.shear.pitch
        ));
    }

    if d_min > 0.0 && length > 0.0 {
        let ratio = d_min / length;
        let lim = match concrete_class {
            ConcreteClass::Normal => 1.0 / 15.0,
            ConcreteClass::Lightweight1 | ConcreteClass::Lightweight2 => 1.0 / 10.0,
        };
        if ratio + 1e-12 < lim {
            warnings.push(format!("最小径/支点間距離={:.4} < {:.4}", ratio, lim));
        }
    }

    if ag > 0.0 {
        let pg = as_total / ag;
        if pg + 1e-12 < 0.008 {
            warnings.push(format!("主筋比 pg={:.3}% < 0.8%", pg * 100.0));
        }
        if !long_term && fc_raw > 0.0 && n_short_comp > 0.0 {
            let sigma = n_short_comp / ag;
            let lim = fc_raw / 3.0;
            if sigma + 1e-9 < lim {
                warnings.push(format!("短期軸応力度 {:.2} < Fc/3={:.2} N/mm²", sigma, lim));
            }
        }
    }

    let b_for_pw_a = d_min.max(1.0);
    // 矩形は両方向の幅で pw を算定し、小さい方（厳しい側）で下限を見る。
    // 円形は等価幅 1 本。
    let pw = match shape {
        SectionShape::RcCircle { .. } => pw_ratio(&rebar.shear, b_for_pw_a),
        SectionShape::RcRect { b, d, .. } | SectionShape::SrcRect { b, d, .. } => {
            let pw_b = pw_ratio(&rebar.shear, (*b).max(1.0));
            let pw_d = pw_ratio(&rebar.shear, (*d).max(1.0));
            pw_b.min(pw_d)
        }
        _ => {
            // 呼び出し側が矩形・円形以外を渡すことは想定しないが、安全側に d_min。
            pw_ratio(&rebar.shear, b_for_pw_a)
        }
    };
    if pw + 1e-12 < 0.002 {
        warnings.push(format!("pw={:.4} < 0.2%（ルート未連動・下限 0.2%）", pw));
    }

    ProvisionCheck { errors, warnings }
}

/// 矩形梁の長期たわみ δ [mm]（マニュアル略算・鋼梁と同形）。
///
/// \\( M_0 = |M_c| + (|M_i|+|M_j|)/2 \\) とし、
/// \\( \delta = 5 M_0 L^2/(48 E I) - (|M_i|+|M_j|) L^2/(16 E I) \\)。
/// 等分布荷重のとき \\( w=(|Q_i|+|Q_j|)/L \\) 形と等価。\\( I = b D^3/12 \\)。
pub(crate) fn beam_deflection_approx(
    b: f64,
    d_full: f64,
    length: f64,
    e: f64,
    m_i: f64,
    m_j: f64,
    m_c: f64,
) -> Option<f64> {
    if length <= 1e-9 || e <= 1e-9 || b <= 0.0 || d_full <= 0.0 {
        return None;
    }
    let i = b * d_full.powi(3) / 12.0;
    if i <= 1e-9 {
        return None;
    }
    let m_l = m_i.abs();
    let m_r = m_j.abs();
    let m0 = m_c.abs() + (m_l + m_r) / 2.0;
    let l2 = length * length;
    let delta = 5.0 * m0 * l2 / (48.0 * e * i) - (m_l + m_r) * l2 / (16.0 * e * i);
    Some(delta.max(0.0))
}

/// たわみ検定比 = (α·δ/L) / (1/250)。α 既定 8（平12 告示第1459号）。
pub(crate) fn beam_deflection_ratio(delta: f64, length: f64, alpha: f64) -> f64 {
    if length <= 1e-9 {
        return 0.0;
    }
    250.0 * alpha * delta / length
}

/// 長期たわみの CheckComponent。端部・中央モーメントが揃うときのみ。
///
/// スパン L は [`DesignCtx::clear_length`]（内法）を優先し、無ければ `length`。
pub(crate) fn beam_deflection_component(
    ctx: &DesignCtx,
    b: f64,
    d_full: f64,
    e: f64,
) -> Option<CheckComponent> {
    if ctx.term != LoadTerm::Long {
        return None;
    }
    let (m_i, m_j) = ctx.end_moments_z?;
    let m_c = ctx.mid_moment_z?;
    let span = ctx.clear_length.filter(|&l| l > 1e-9).unwrap_or(ctx.length);
    let alpha = 8.0;
    let delta = beam_deflection_approx(b, d_full, span, e, m_i, m_j, m_c)?;
    let ratio = beam_deflection_ratio(delta, span, alpha);
    Some(CheckComponent {
        kind: CheckKind::Deflection,
        ratio,
        detail: format!(
            "δ={:.2} mm, L={:.0} mm（内法優先）, α={:.0}, α·δ/L={:.5}, 制限 1/250, 検定比={:.3}",
            delta,
            span,
            alpha,
            alpha * delta / span,
            ratio
        ),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::section_shape::{BarSet, ShearBar};

    fn sample_rebar(cover: f64, pitch: f64, dia: f64, count: u32) -> RcRebar {
        RcRebar {
            cover,
            main_x: BarSet {
                dia,
                count,
                layers: 1,
            },
            main_y: BarSet {
                dia,
                count: 0,
                layers: 1,
            },
            shear: ShearBar {
                dia: 10.0,
                pitch,
                legs: 2,
            },
        }
    }

    #[test]
    fn beam_cover_error_and_pitch_error() {
        let rebar = sample_rebar(20.0, 400.0, 19.0, 4);
        let props = AxisProps {
            b: 300.0,
            d_full: 600.0,
            dt: 50.0,
            d: 550.0,
            at: 1000.0,
            ac: 1000.0,
            j: 7.0 * 550.0 / 8.0,
            pw: 0.003,
        };
        let p = beam_provisions(600.0, &props, &rebar, true, 0.0, 195.0);
        assert!(p.errors.iter().any(|e| e.contains("かぶり")));
        assert!(p.errors.iter().any(|e| e.contains("あばら間隔")));
    }

    #[test]
    fn deflection_uniform_simple_beam() {
        let b = 300.0;
        let d = 600.0;
        let l = 6000.0;
        let e = 21_000.0;
        let w = 10.0;
        let m0 = w * l * l / 8.0;
        let delta = beam_deflection_approx(b, d, l, e, 0.0, 0.0, m0).unwrap();
        let i = b * d.powi(3) / 12.0;
        let expect = 5.0 * w * l.powi(4) / (384.0 * e * i);
        assert!((delta - expect).abs() / expect < 1e-9);
    }

    #[test]
    fn column_pw_uses_min_of_both_widths() {
        // 帯筋 Aw=2·78.5、ピッチ 100 → pw(b)=Aw/(b·s)。
        // b=800, d=400 → pw_d < pw_b。pw_d = 157/(400·100)=0.003925 ≥ 0.2%。
        // ピッチを大きくして pw_d だけ 0.2% 未満にする。
        let rebar = sample_rebar(40.0, 250.0, 22.0, 4);
        let shape = SectionShape::RcRect {
            b: 800.0,
            d: 400.0,
            rebar: rebar.clone(),
        };
        let p = column_provisions(
            &shape,
            &rebar,
            400.0,
            4000.0,
            ConcreteClass::Normal,
            true,
            800.0 * 400.0,
            4.0 * std::f64::consts::PI * 11.0 * 11.0,
            0.0,
            24.0,
        );
        // pw_d = 2·(π·5²)/(400·250) ≈ 0.00157 < 0.002
        assert!(
            p.warnings
                .iter()
                .any(|w| w.contains("pw=") && w.contains("0.2%")),
            "warnings={:?}",
            p.warnings
        );
    }

    #[test]
    fn deflection_prefers_clear_length() {
        let mut ctx = DesignCtx {
            term: LoadTerm::Long,
            length: 8000.0,
            clear_length: Some(6000.0),
            end_moments_z: Some((0.0, 0.0)),
            mid_moment_z: Some(1.0e8),
            ..Default::default()
        };
        let c = beam_deflection_component(&ctx, 300.0, 600.0, 21_000.0).unwrap();
        assert!(c.detail.contains("L=6000"), "detail={}", c.detail);
        ctx.clear_length = None;
        let c2 = beam_deflection_component(&ctx, 300.0, 600.0, 21_000.0).unwrap();
        assert!(c2.detail.contains("L=8000"), "detail={}", c2.detail);
    }

    #[test]
    fn column_short_axial_warns_below_fc_over_3() {
        let rebar = sample_rebar(40.0, 100.0, 22.0, 4);
        let shape = SectionShape::RcRect {
            b: 400.0,
            d: 400.0,
            rebar: rebar.clone(),
        };
        let ag = 400.0 * 400.0;
        let as_total = 4.0 * std::f64::consts::PI * 11.0 * 11.0;
        let fc = 24.0;
        // σ = 1.0 < Fc/3=8 → 警告
        let low = column_provisions(
            &shape,
            &rebar,
            400.0,
            4000.0,
            ConcreteClass::Normal,
            false,
            ag,
            as_total,
            1.0 * ag,
            fc,
        );
        assert!(
            low.warnings
                .iter()
                .any(|w| w.contains("短期軸応力度") && w.contains("Fc/3")),
            "warnings={:?}",
            low.warnings
        );
        // σ = 10.0 > Fc/3=8 → 警告なし
        let ok = column_provisions(
            &shape,
            &rebar,
            400.0,
            4000.0,
            ConcreteClass::Normal,
            false,
            ag,
            as_total,
            10.0 * ag,
            fc,
        );
        assert!(
            ok.warnings.iter().all(|w| !w.contains("短期軸応力度")),
            "warnings={:?}",
            ok.warnings
        );
    }

    #[test]
    fn column_slenderness_uses_passed_span_length() {
        // d_min/L: 400/7000 < 1/15 → 警告、400/5000 > 1/15 → なし
        let rebar = sample_rebar(40.0, 100.0, 22.0, 8);
        let shape = SectionShape::RcRect {
            b: 400.0,
            d: 400.0,
            rebar: rebar.clone(),
        };
        let ag = 400.0 * 400.0;
        let as_total = 8.0 * std::f64::consts::PI * 11.0 * 11.0;
        let slender = column_provisions(
            &shape,
            &rebar,
            400.0,
            7000.0,
            ConcreteClass::Normal,
            true,
            ag,
            as_total,
            0.0,
            24.0,
        );
        assert!(
            slender
                .warnings
                .iter()
                .any(|w| w.contains("最小径/支点間距離")),
            "warnings={:?}",
            slender.warnings
        );
        let ok = column_provisions(
            &shape,
            &rebar,
            400.0,
            5000.0,
            ConcreteClass::Normal,
            true,
            ag,
            as_total,
            0.0,
            24.0,
        );
        assert!(
            ok.warnings.iter().all(|w| !w.contains("最小径/支点間距離")),
            "warnings={:?}",
            ok.warnings
        );
    }
}
