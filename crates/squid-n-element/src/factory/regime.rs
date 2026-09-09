//! フォースレジーム（`ForceRegime`）判定。
//!
//! - [`resolve_force_regime`] — `ForceRegime::Auto` をトポロジから判定
//! - [`ResolvedRegime`] — 判定結果（集中ばね / ファイバー）
//! - [`is_vertical_member`] — 鉛直材（柱）かどうか
//! - [`is_on_rigid_diaphragm`] — 剛床に所属するか

use squid_n_core::model::{ElementData, ForceRegime, Model};

/// ForceRegime の自動選択結果
pub enum ResolvedRegime {
    ConcentratedSpring,
    Fiber,
}

/// ForceRegime::Auto をトポロジから判定する。
/// 剛床所属の梁 → ConcentratedSpring、それ以外 → Fiber
pub fn resolve_force_regime(data: &ElementData, model: &Model) -> ResolvedRegime {
    if data.force_regime != ForceRegime::Auto {
        return match data.force_regime {
            ForceRegime::UniaxialBendingShear => ResolvedRegime::ConcentratedSpring,
            ForceRegime::AxialBendingInteract => ResolvedRegime::Fiber,
            ForceRegime::Auto => unreachable!(),
        };
    }

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
        (Some(n0), Some(n1)) => squid_n_core::geom::is_vertical_axis(n0.coord, n1.coord),
        _ => false,
    }
}

fn is_on_rigid_diaphragm(data: &ElementData, model: &Model) -> bool {
    data.nodes.iter().any(|&n| model.node_on_rigid_diaphragm(n))
}
