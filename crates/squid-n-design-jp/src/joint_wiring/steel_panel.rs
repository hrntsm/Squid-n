//! S 造パネルゾーンのせん断検定配線。

use super::common::MemberInfo;
use crate::steel::panel_zone::{s_panel_zone_check, SPanelInput};
use crate::CheckResult;
use squid_n_core::ids::NodeId;
use squid_n_core::model::Model;
use squid_n_core::panel_zone::resolve_panel_joint;

/// S 造パネルゾーンの検定を `out` へ追加する。
///
/// 検定する接合部の判定は [`resolve_panel_joint`] に委ねる。仕口パネルの
/// モデル化と**同じ規則**で、取り付く柱・はりがすべて S/CFT 系の接合部のみを
/// 対象とする。RC/SRC が 1 本でも混じる接合部は、コンクリートが接合部を拘束して
/// 挙動が変わるため対象外とし、RC 柱梁接合部・SRC パネルゾーンの検定が担う。
///
/// モデル化と唯一異なるのは CFT の扱いで、鋼管部を S 造と同じ式で評価できるため
/// 検定は対象に含める（モデル化は接合部を剛節点として扱う）。
///
/// `panel_moment` は仕口パネルをモデル化した接合部で解析が出力した設計用パネル
/// モーメント `pM` [N·mm]。`None`（モデル化していない接合部）のときは梁端
/// モーメント・柱せん断から `pM` を組み立てる。
pub(super) fn check_s_panel(
    model: &Model,
    cols: &[&MemberInfo<'_>],
    beams: &[&MemberInfo<'_>],
    nid: NodeId,
    panel_moment: Option<f64>,
    out: &mut Vec<(NodeId, String, CheckResult)>,
) {
    let Some(joint) = resolve_panel_joint(model, nid, &model.elements) else {
        return;
    };
    // 諸元は `Ve` が最小の柱から採る。軸力比 n と基準強度 F もその柱の値を用いる。
    let Some(col) = cols.iter().find(|c| c.elem.id == joint.column) else {
        return;
    };

    // プリセット外の直接入力材料は fy を基準強度として用いる（それも無ければ 235）。
    let t = crate::steel::steel_f_value_prefix(&col.mat.name, 40.0);
    let fy = t.or(col.mat.fy).unwrap_or(235.0);
    // 軸力比 n = 圧縮軸力/(F·A)（当該ケースの軸力。引張は 0）。
    let n_axial = col
        .end_forces(nid)
        .map(|f| (-f[0]).max(0.0) / (fy * col.sec.area.max(1e-9)))
        .unwrap_or(0.0);
    let m_left = beams
        .first()
        .and_then(|b| b.end_forces(nid))
        .map(|f| f[5].abs())
        .unwrap_or(0.0);
    let m_right = beams
        .get(1)
        .and_then(|b| b.end_forces(nid))
        .map(|f| f[5].abs())
        .unwrap_or(0.0);
    let mut col_qs: Vec<f64> = cols
        .iter()
        .filter_map(|c| c.end_forces(nid))
        .map(|f| f[1].abs().max(f[2].abs()))
        .collect();
    col_qs.resize(2, 0.0);
    let inp = SPanelInput {
        geometry: joint.geometry,
        db: joint.db,
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
