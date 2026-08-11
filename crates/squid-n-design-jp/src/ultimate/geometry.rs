//! 部材の幾何量（保有水平耐力計算の終局検定で共用する部材形状の判定）。
//!
//! - [`member_kind`] — 部材軸の鉛直成分から部材種別（梁/柱/ブレース）を判定する。
//! - [`geometric_length`] — 部材両端節点間の幾何長。
//! - [`clear_span`] — 剛域（フェイス距離）控除後の内法長さ。

use crate::MemberKind;
use squid_n_core::model::{ElementData, Model};

/// 部材軸の鉛直成分 |ez| から部材種別を判定する（app の `member_kind_of` と同規則）。
pub(super) fn member_kind(elem: &ElementData, model: &Model) -> MemberKind {
    let coords: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .take(2)
        .collect();
    let (Some(p0), Some(p1)) = (coords.first(), coords.get(1)) else {
        return MemberKind::Beam;
    };
    let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if len < 1e-9 {
        return MemberKind::Beam;
    }
    let ez = (d[2] / len).abs();
    if ez >= 0.8 {
        MemberKind::Column
    } else if ez <= 0.2 {
        MemberKind::Beam
    } else {
        MemberKind::Brace
    }
}

/// 部材両端節点間の幾何長 [mm]。
pub(super) fn geometric_length(elem: &ElementData, model: &Model) -> f64 {
    let coords: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .take(2)
        .collect();
    let (Some(p0), Some(p1)) = (coords.first(), coords.get(1)) else {
        return 0.0;
    };
    let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// 内法長さ [mm] = 幾何長 − 両端フェイス距離。フェイス合計が幾何長以上の
/// 不整合入力では幾何長のままとする（app の rank-auto と同規則）。
///
/// フェース距離が未算定（`RigidZone::face_i/face_j` が `None`）の場合も幾何長を
/// 返す。終局耐力は解析後に算定するため、その時点では剛域算定が済んでいる。
pub(super) fn clear_span(elem: &ElementData, model: &Model) -> f64 {
    let geom = geometric_length(elem, model);
    match elem.rigid_zone.clear_span_from(geom) {
        Some(lo) if lo > 0.0 => lo,
        _ => geom,
    }
}
