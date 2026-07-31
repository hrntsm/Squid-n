//! S 造パネルゾーンのせん断検定配線。

use super::common::{is_steel, MemberInfo};
use crate::steel::panel_zone::{s_panel_zone_check, SPanelInput};
use crate::CheckResult;
use squid_n_core::ids::NodeId;
use squid_n_core::panel_zone::{beam_panel_depth, PanelGeometry};

/// S 造パネルゾーンの検定を `out` へ追加する。
///
/// `panel_moment` は仕口パネルをモデル化した接合部で解析が出力した設計用パネル
/// モーメント `pM` [N·mm]。`None`（モデル化していない接合部）のときは従来どおり
/// 梁端モーメント・柱せん断から `pM` を組み立てる。
pub(super) fn check_s_panel(
    cols: &[&MemberInfo<'_>],
    beams: &[&MemberInfo<'_>],
    nid: NodeId,
    panel_moment: Option<f64>,
    out: &mut Vec<(NodeId, String, CheckResult)>,
) {
    // ── S 造パネルゾーン ─────────────────────────────────────
    let s_col = cols.iter().find(|c| is_steel(&c.mat.name));
    let s_beams: Vec<&&MemberInfo> = beams.iter().filter(|b| is_steel(&b.mat.name)).collect();
    if let (Some(col), false) = (s_col, s_beams.is_empty()) {
        // パネル諸元（dc・tp・断面区分）はモデル化側と共通の解決規則を用いる。
        // CFT はモデル化の対象外だが、断面検定では鋼管部を S 造と同じ式で評価する
        // ため、ここでは対象に含める（`PanelGeometry::is_modeling_target` は見ない）。
        let panel = PanelGeometry::from_column(col.sec);
        if let Some(panel) = panel {
            // 梁フランジ板厚中心間距離 db（最大せいの梁を採用）。
            let db = s_beams
                .iter()
                .map(|b| beam_panel_depth(b.sec))
                .fold(0.0, f64::max);
            // プリセット外の直接入力材料は fy を基準強度として用いる（それも無ければ 235）。
            let t = crate::steel::steel_f_value_prefix(&col.mat.name, 40.0);
            let fy = t.or(col.mat.fy).unwrap_or(235.0);
            // 軸力比 n = 圧縮軸力/(F·A)（当該ケースの軸力。引張は 0）。
            let n_axial = col
                .end_forces(nid)
                .map(|f| (-f[0]).max(0.0) / (fy * col.sec.area.max(1e-9)))
                .unwrap_or(0.0);
            let m_left = s_beams
                .first()
                .and_then(|b| b.end_forces(nid))
                .map(|f| f[5].abs())
                .unwrap_or(0.0);
            let m_right = s_beams
                .get(1)
                .and_then(|b| b.end_forces(nid))
                .map(|f| f[5].abs())
                .unwrap_or(0.0);
            let mut col_qs: Vec<f64> = cols
                .iter()
                .filter(|c| is_steel(&c.mat.name))
                .filter_map(|c| c.end_forces(nid))
                .map(|f| f[1].abs().max(f[2].abs()))
                .collect();
            col_qs.resize(2, 0.0);
            let inp = SPanelInput {
                geometry: panel,
                db,
                fy,
                axial_ratio: n_axial,
                beam_moment_left: m_left,
                beam_moment_right: m_right,
                col_shear_upper: col_qs[0],
                col_shear_lower: col_qs[1],
                design_moment: panel_moment,
            };
            out.push((nid, "パネルゾーン(S)".to_string(), s_panel_zone_check(&inp)));
        }
    }
}
