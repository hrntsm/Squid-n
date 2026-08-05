//! 剛域（rigid zone）の自動算定。
//!
//! モデルのトポロジ（部材種別・接続断面）から各部材端の剛域長を算定し、
//! `ElementData::rigid_zone` へ反映する前処理を提供する。剛性・内力を計算する
//! [`BeamElement`](super::BeamElement) とは独立しており、解析前に一度だけ適用する。

use squid_n_core::adjacency::NodeAdjacency;
use squid_n_core::model::{Model, RigidZone, ZoneSource};
use squid_n_core::structure_kind::member_structure_kind;

pub struct RigidZoneRule {
    pub reduction: f64,
}

impl Default for RigidZoneRule {
    fn default() -> Self {
        Self { reduction: 1.0 }
    }
}

fn elem_axis(model: &Model, e: &squid_n_core::model::ElementData) -> [f64; 3] {
    if e.nodes.len() < 2 {
        return [0.0, 0.0, 0.0];
    }
    let p0 = model.nodes[e.nodes[0].index()].coord;
    let p1 = model.nodes[e.nodes[1].index()].coord;
    let dx = p1[0] - p0[0];
    let dy = p1[1] - p0[1];
    let dz = p1[2] - p0[2];
    let l = (dx * dx + dy * dy + dz * dz).sqrt();
    if l < 1e-12 {
        [0.0, 0.0, 0.0]
    } else {
        [dx / l, dy / l, dz / l]
    }
}

/// `only_rc_src` を true にすると、RC/SRC 系の直交 Beam 要素だけを対象に最大せいを探す
/// （剛域長 λ 用。仕口部に接続する柱(梁)がすべてＳの場合、剛域長さは0
/// ＝ S 系直交材は無視することで自然に d_max=0 となる）。false なら種別を問わず全直交
/// Beam 要素が対象（危険断面位置 face 用。§6.2.3 は幾何量であり種別を区別しない）。
fn max_orth_depth(
    model: &Model,
    node: squid_n_core::ids::NodeId,
    target_axis: [f64; 3],
    target_elem_idx: usize,
    adjacency: &NodeAdjacency,
    only_rc_src: bool,
) -> f64 {
    let mut d_max = 0.0;
    for &ei in adjacency.indices_at(node) {
        if ei == target_elem_idx {
            continue;
        }
        let e = &model.elements[ei];
        if only_rc_src && member_structure_kind(model, e).is_steel_like() {
            continue;
        }
        let axis = elem_axis(model, e);
        let dot =
            (axis[0] * target_axis[0] + axis[1] * target_axis[1] + axis[2] * target_axis[2]).abs();
        if dot < 0.707 {
            // 概ね直交（45°以上）
            if let Some(sec) = e.section.and_then(|sid| model.sections.get(sid.index())) {
                if sec.depth > d_max {
                    d_max = sec.depth;
                }
            }
        }
    }
    d_max
}

/// 剛域算定の本体（隣接マップは呼び出し側が構築して共有する）。
/// `target_elem_idx` は `model.elements` 内の対象要素の添字。
fn rigid_zone_with_adjacency(
    model: &Model,
    target_elem_idx: usize,
    adjacency: &NodeAdjacency,
    rule: &RigidZoneRule,
) -> RigidZone {
    let elem = &model.elements[target_elem_idx];
    let nodes = &elem.nodes;
    if nodes.len() < 2 {
        return RigidZone {
            reduction: rule.reduction,
            ..Default::default()
        };
    }

    let self_sec = elem.section.and_then(|sid| model.sections.get(sid.index()));
    let d_self = self_sec.map(|s| s.depth).unwrap_or(0.0);

    let target_axis = elem_axis(model, elem);

    // face 用: 種別を問わない直交 Beam 要素の最大せい（従来どおりの幾何量）。
    let d_orth_face_i = max_orth_depth(
        model,
        nodes[0],
        target_axis,
        target_elem_idx,
        adjacency,
        false,
    );
    let d_orth_face_j = max_orth_depth(
        model,
        nodes[nodes.len() - 1],
        target_axis,
        target_elem_idx,
        adjacency,
        false,
    );
    // λ 用: RC/SRC 系の直交 Beam 要素だけの最大せい。
    let d_orth_rc_i = max_orth_depth(
        model,
        nodes[0],
        target_axis,
        target_elem_idx,
        adjacency,
        true,
    );
    let d_orth_rc_j = max_orth_depth(
        model,
        nodes[nodes.len() - 1],
        target_axis,
        target_elem_idx,
        adjacency,
        true,
    );

    // 剛域長 λ は自部材の構造種別で式を切り替える（技術基準解説書「剛域の計算」）。
    // - RC/SRC 造: λ = reduction·(D_orth_rc/2 − D_self/4)（従来式。負は 0 クランプ）。
    // - Ｓ・ＣＦＴ造: λ = D_orth_rc/2（D_self/4 の控除なし・reduction も掛けない。
    //   RC/SRC 大梁のうち最大せいの梁フェイスまでの長さ＝仕口部を除いた長さ）。
    //   直交する RC/SRC 系の梁（柱）がなければ D_orth_rc=0 なので λ=0
    //   （Ｓ造の剛域長さは0となる）。
    let self_kind = member_structure_kind(model, elem);
    let lambda = |d_orth_rc: f64| -> f64 {
        if self_kind.is_steel_like() {
            d_orth_rc / 2.0
        } else {
            (rule.reduction * (d_orth_rc / 2.0 - d_self / 4.0)).max(0.0)
        }
    };
    // フェイス距離 = D_orth/2 は剛性用剛域の低減率（慣用調整）と無関係な幾何量なので
    // reduction を掛けない（設計書 §6.2.1「設計位置との区別」）。
    // λ が負→0 にクランプされる場合でも face はそのまま D_orth/2 を保持する。
    let face = |d_orth: f64| -> f64 { d_orth / 2.0 };

    RigidZone {
        length_i: lambda(d_orth_rc_i),
        length_j: lambda(d_orth_rc_j),
        source_i: ZoneSource::Auto,
        source_j: ZoneSource::Auto,
        reduction: rule.reduction,
        face_i: face(d_orth_face_i),
        face_j: face(d_orth_face_j),
        // パネル分のオフセットは剛域算定の対象外（`panel_gen` が別途書き込む）。
        // ここで既定値へ落としても、`recompute_auto_zones` が反映しないため
        // モデル側の値は保たれる。
        panel_offset_i: 0.0,
        panel_offset_j: 0.0,
    }
}

/// 単一要素の剛域を算定する（テスト・単発利用向け）。
/// 内部で隣接マップを構築するため O(E)。全要素へ適用する場合は
/// [`apply_auto_rigid_zones`]（隣接マップを 1 回だけ構築して共有）を使うこと。
pub fn auto_rigid_zones(
    model: &squid_n_core::model::Model,
    elem_id: squid_n_core::ids::ElemId,
    rule: &RigidZoneRule,
) -> RigidZone {
    let Some(target_elem_idx) = model.elements.iter().position(|e| e.id == elem_id) else {
        return RigidZone {
            reduction: rule.reduction,
            ..Default::default()
        };
    };
    let adjacency = NodeAdjacency::build(model);
    rigid_zone_with_adjacency(model, target_elem_idx, &adjacency, rule)
}

pub fn recompute_auto_zones(zone: &mut RigidZone, recomputed: &RigidZone) {
    if matches!(zone.source_i, ZoneSource::Auto) {
        zone.length_i = recomputed.length_i;
    }
    if matches!(zone.source_j, ZoneSource::Auto) {
        zone.length_j = recomputed.length_j;
    }
    // 仕口パネル分のオフセット（`panel_offset_i/j`）は**触らない**。剛域算定は
    // 単独で走る経路（増分解析・時刻歴・MCP のジョブ）があり、ここで初期化すると
    // パネル要素が残ったままオフセットだけが消えたモデルで解析が走る。
    // オフセットの更新は `springs::panel_gen::apply_auto_panel_zones` の責務とする。

    // フェイス距離は剛域長の Manual/Auto フラグとは独立な幾何量（接続関係から
    // 一意に決まる §6.2.1）。手動で剛域長を保護しているときも、モデルの接続情報
    // が変われば危険断面位置は追従すべきなので、Manual 保護の対象外として常に
    // 再算定値で更新する。
    zone.face_i = recomputed.face_i;
    zone.face_j = recomputed.face_j;
}

/// モデル全要素の剛域を自動算定し、`ElementData::rigid_zone` を更新する前処理。
/// `source` が `Auto` の端のみ更新し、`Manual` 端は保護する（設計書 §6.2.1）。
/// 解析前に1回呼ぶことで剛域が組立に反映される（既定では剛域長 0 のまま
/// ＝呼ばなければ従来挙動。明示的に有効化する設計）。
///
/// 隣接マップ（節点 → 接続 Beam 要素）を 1 回だけ構築して全要素で共有するため
/// O(E)（辺数比例）で完了する。
pub fn apply_auto_rigid_zones(model: &mut Model, rule: &RigidZoneRule) {
    let adjacency = NodeAdjacency::build(model);
    let recomputed: Vec<(usize, RigidZone)> = model
        .elements
        .iter()
        .enumerate()
        .filter(|(_, e)| matches!(e.kind, squid_n_core::model::ElementKind::Beam))
        .map(|(i, _)| (i, rigid_zone_with_adjacency(model, i, &adjacency, rule)))
        .collect();

    for (i, rz) in recomputed {
        let zone = &mut model.elements[i].rigid_zone;
        recompute_auto_zones(zone, &rz);
        // reduction も Auto 算定値を反映（手動端の length は保持済み）。
        zone.reduction = rz.reduction;
    }
}
