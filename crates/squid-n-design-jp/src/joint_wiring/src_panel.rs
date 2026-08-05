//! SRC 造柱梁接合部（パネルゾーン）の検定配線。

use super::common::{rc_dt, MemberInfo};
use crate::rc::joint::JointShape;
use crate::srrc::panel_zone::{src_panel_zone_check, SrcPanelInput};
use crate::{CheckResult, LoadTerm};
use squid_n_core::ids::NodeId;
use squid_n_core::section_shape::SectionShape;

/// SRC 造柱梁接合部（パネルゾーン）の検定を `out` へ追加する。
pub(super) fn check_src_panel(
    cols: &[&MemberInfo<'_>],
    beams: &[&MemberInfo<'_>],
    nid: NodeId,
    term: LoadTerm,
    out: &mut Vec<(NodeId, String, CheckResult)>,
) {
    // ── SRC 造柱梁接合部（パネルゾーン） ─────────────────────
    let src_col = cols.iter().find(|c| {
        matches!(c.sec.shape, Some(SectionShape::SrcRect { .. })) && c.mat.fc.unwrap_or(0.0) > 0.0
    });
    if let Some(col) = src_col {
        if let Some(SectionShape::SrcRect {
            ref rebar,
            steel_height,
            steel_web_thick,
            steel_flange_thick,
            ..
        }) = col.sec.shape
        {
            let fc = col.mat.fc.unwrap_or(0.0);
            // mCd（柱の左右主筋間距離）の近似: 柱幅 − 2・rc_dt(rebar)
            // （冒頭 doc 参照。既存 RC 接合部配線の beam_j 近似に合わせる）。
            let m_cd = (col.sec.width - 2.0 * rc_dt(rebar)).max(0.0);
            let s_cd = (steel_height - steel_flange_thick).max(0.0);
            let j_tw = steel_web_thick;

            let beam0 = beams[0];
            let beam_is_steel = beam0.kind.is_steel_like();
            let m_bd = if beam_is_steel {
                // 梁が S 造の場合は mBd の代わりに sBd（フランジ板厚中心間
                // 距離）を渡す（S パネルゾーンの db 算定と同じ近似）。
                match beam0.sec.shape {
                    Some(SectionShape::SteelH { flange_thick, .. }) => {
                        beam0.sec.depth - flange_thick
                    }
                    _ => 0.9 * beam0.sec.depth,
                }
            } else {
                match beam0.sec.shape {
                    Some(SectionShape::RcRect { ref rebar, .. })
                    | Some(SectionShape::SrcRect { ref rebar, .. }) => {
                        (beam0.sec.depth - 2.0 * rc_dt(rebar)).max(0.0)
                    }
                    _ => 0.8 * beam0.sec.depth,
                }
            };

            // 接合部形状（RC 接合部配線と同じ判定: 柱2本以上×取り付く梁
            // 2本以上で十字形、以下同様）。
            let shape = match (cols.len() >= 2, beams.len() >= 2) {
                (true, true) => JointShape::Cross,
                (false, true) => JointShape::Tee,
                (true, false) => JointShape::Knee,
                (false, false) => JointShape::Corner,
            };

            let sum_beam_moments: f64 = beams
                .iter()
                .filter_map(|b| b.end_forces(nid))
                .map(|f| f[5].abs())
                .sum();

            let inp = SrcPanelInput {
                shape,
                fc,
                long_term: term == LoadTerm::Long,
                col_width: col.sec.width,
                beam_width: beam0.sec.width,
                m_bd,
                m_cd,
                j_tw,
                s_cd,
                beam_is_steel,
                n_ratio: crate::rc::young_ratio_n(fc),
                // h′/h（内法階高/階高比、原典図 2026-07-11）は情報がないため 1.0 固定（暫定、
                // 冒頭 doc 参照）。
                h_ratio: 1.0,
                sum_beam_moments,
            };
            out.push((
                nid,
                "柱梁接合部(SRC)".to_string(),
                src_panel_zone_check(&inp),
            ));
        }
    }
}
