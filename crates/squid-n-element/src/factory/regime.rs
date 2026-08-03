//! フォースレジーム（`ForceRegime`）判定（P5 §5）。
//!
//! - [`resolve_force_regime`] — `ForceRegime::Auto` をトポロジから判定
//! - [`ResolvedRegime`] — 判定結果（集中ばね / ファイバー）
//! - [`is_vertical_member`] — 鉛直材（柱）かどうか
//! - [`is_on_rigid_diaphragm`] — 剛床に所属するか

use squid_n_core::model::{ElementData, ForceRegime, Model};

/// ForceRegime の自動選択結果（P5 §5）
pub enum ResolvedRegime {
    ConcentratedSpring,
    Fiber,
}

/// ForceRegime::Auto をトポロジから判定する（P5 §5）
/// 剛床所属の階かつ梁で軸力変動が小 → ConcentratedSpring
/// それ以外 → Fiber
pub fn resolve_force_regime(data: &ElementData, model: &Model) -> ResolvedRegime {
    if data.force_regime != ForceRegime::Auto {
        return match data.force_regime {
            ForceRegime::UniaxialBendingShear => ResolvedRegime::ConcentratedSpring,
            ForceRegime::AxialBendingInteract => ResolvedRegime::Fiber,
            ForceRegime::Auto => unreachable!(),
        };
    }

    // Auto の判定ロジック（ヒューリスティック）
    // 剛床に所属する梁（= 鉛直軸でない部材）は集中ばね
    let is_vertical = is_vertical_member(data, model);
    let on_rigid_diaphragm = is_on_rigid_diaphragm(data, model);

    if on_rigid_diaphragm && !is_vertical {
        ResolvedRegime::ConcentratedSpring
    } else {
        ResolvedRegime::Fiber
    }
}

pub(super) fn is_vertical_member(data: &ElementData, model: &Model) -> bool {
    if data.nodes.len() < 2 {
        return false;
    }
    let n0 = &model.nodes.get(data.nodes[0].index());
    let n1 = &model.nodes.get(data.nodes[1].index());
    match (n0, n1) {
        // 全クレート共通の 45° 余弦基準（|ez| > 0.707）。層せん断集計・
        // 変形角定義と同じ規約で柱系/梁系を分ける。
        (Some(n0), Some(n1)) => squid_n_core::geom::is_vertical_axis(n0.coord, n1.coord),
        _ => false,
    }
}

fn is_on_rigid_diaphragm(data: &ElementData, model: &Model) -> bool {
    let elem_nodes: Vec<squid_n_core::ids::NodeId> = data.nodes.iter().copied().collect();
    for story in &model.stories {
        for dia in &story.diaphragms {
            if elem_nodes
                .iter()
                .any(|n| *n == dia.master || dia.slaves.contains(n))
            {
                return true;
            }
        }
    }
    for c in &model.constraints {
        if let squid_n_core::model::Constraint::RigidDiaphragm { master, slaves, .. } = c {
            if elem_nodes
                .iter()
                .any(|n| *n == *master || slaves.contains(n))
            {
                return true;
            }
        }
    }
    false
}
