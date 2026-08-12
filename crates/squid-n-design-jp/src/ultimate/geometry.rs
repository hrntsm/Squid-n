//! 部材の幾何量（保有水平耐力計算の終局検定で共用する部材形状の判定）。
//!
//! - [`clear_span`] — 剛域（フェイス距離）控除後の内法長さ。
//!
//! 部材種別の判定は [`crate::MemberKind::of_element`]、部材両端節点間の幾何長は
//! [`Model::member_length`] を直接用いる（判定・算定の情報源を 1 つに保つ）。

use squid_n_core::model::{ElementData, Model};

/// 内法長さ [mm] = 幾何長 − 両端フェイス距離。フェイス合計が幾何長以上の
/// 不整合入力では幾何長のままとする（app の rank-auto と同規則）。
///
/// フェース距離が未算定（`RigidZone::face_i/face_j` が `None`）の場合も幾何長を
/// 返す。終局耐力は解析後に算定するため、その時点では剛域算定が済んでいる。
pub(super) fn clear_span(elem: &ElementData, model: &Model) -> f64 {
    let geom = model.member_length(elem);
    match elem.rigid_zone.clear_span_from(geom) {
        Some(lo) if lo > 0.0 => lo,
        _ => geom,
    }
}
