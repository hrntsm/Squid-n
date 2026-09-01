//! 剛域（rigid zone）の自動算定。
//!
//! モデルのトポロジ（部材種別・接続断面）から各部材端の剛域長を算定し、
//! `ElementData::rigid_zone` へ反映する前処理を提供する。剛性・内力を計算する
//! [`BeamElement`](super::BeamElement) とは独立しており、解析前に一度だけ適用する。

use squid_n_core::adjacency::NodeAdjacency;
use squid_n_core::model::{Model, RigidZone, ZoneSource};
use squid_n_core::structure_kind::member_structure_kind;

pub struct RigidZoneRule {
    /// 部材フェース・部材せいに、取り付く壁を考慮するか（技術基準。既定は考慮する）。
    ///
    /// `false` にすると部材の原断面だけで算定する。対象となる壁は
    /// [`crate::wall::misc_wall::collect_rigid_zone_walls`]（現場打ちコンクリート壁で
    /// 厚さ 100 mm 以上。耐震壁・雑壁を問わない）。
    pub consider_walls: bool,
}

impl Default for RigidZoneRule {
    fn default() -> Self {
        Self {
            consider_walls: true,
        }
    }
}

/// 部材に取り付く壁が、部材フェースから張り出す長さ [mm]。
///
/// 柱（鉛直材）には袖壁が材軸に直交する水平方向へ、梁（水平材）には腰壁・垂壁が
/// 鉛直方向へ張り出す。壁の長さ（袖壁長さ・腰壁/垂壁高さ）は部材芯からの距離で
/// 与えられるため、部材フェースからの張り出しは「壁の長さ − 部材せい/2」となる。
///
/// `toward` を与えると、その向きへ張り出す壁だけを対象にする（部材フェース距離
/// Lf 用。柱の片側だけに袖壁があるとき、フェースが伸びるのは袖壁が張り出して
/// いる側に取り付く梁だけで、反対側の梁は伸びない）。`None` なら向きを問わず
/// 最大を採る（部材せい D 用。「両側に取り付く壁の長さが異なる場合は長い方の壁を
/// 基準にする」）。
fn wall_protrusion(
    model: &Model,
    elem: &squid_n_core::model::ElementData,
    depth: f64,
    walls: &[crate::wall::misc_wall::InFrameMiscWallGeometry],
    toward: Option<[f64; 3]>,
) -> f64 {
    if walls.is_empty() || elem.nodes.len() < 2 {
        return 0.0;
    }
    let (n0, n1) = (elem.nodes[0], elem.nodes[elem.nodes.len() - 1]);
    let same_pair = |a: squid_n_core::ids::NodeId, b: squid_n_core::ids::NodeId| -> bool {
        (n0 == a && n1 == b) || (n0 == b && n1 == a)
    };
    // 向きの一致判定（`toward` が None なら常に採用）。
    let accepts = |dir: [f64; 3]| -> bool {
        match toward {
            None => true,
            Some(t) => dir[0] * t[0] + dir[1] * t[1] + dir[2] * t[2] > 1e-9,
        }
    };

    let axis = elem_axis(model, elem);
    // 全クレート共通の 45° 余弦基準（|ez| > 0.707）で柱系/梁系を分ける。
    let is_vertical = axis[2].abs() > squid_n_core::geom::VERTICAL_COS_TOL;
    let mut out = 0.0_f64;

    for w in walls {
        if is_vertical {
            // 柱: 壁の鉛直辺（bottom_pair[s]–top_pair[s]）が自部材と一致する側の袖壁。
            // 上下いずれかの辺に主架構が無い壁（取り付く壁版）は鉛直辺が柱と
            // 一致しえないため、袖壁としては効かない。
            let (Some(bottom), Some(top)) = (w.bottom_pair, w.top_pair) else {
                continue;
            };
            for s in 0..2 {
                // 柱際が切れている辺は柱と一体でないため、剛域を作らない。
                if w.column_face_slit[s] {
                    continue;
                }
                if !same_pair(bottom[s], top[s]) {
                    continue;
                }
                // 壁は s=0 側の柱からは +e_wall 方向へ、s=1 側の柱からは −e_wall 方向へ伸びる。
                let e_wall = w.bottom_dir;
                let sign = if s == 0 { 1.0 } else { -1.0 };
                let dir = [e_wall[0] * sign, e_wall[1] * sign, 0.0];
                if !accepts(dir) {
                    continue;
                }
                out = out.max((w.wing_length(s) - depth / 2.0).clamp(0.0, w.lw));
            }
        } else {
            // 梁: 下辺が自部材なら壁は上に載る（腰壁）、上辺が自部材なら下に垂れる（垂壁）。
            for (matched, extent, dir) in [
                (
                    w.bottom_pair.is_some_and(|p| same_pair(p[0], p[1])),
                    w.strip_height(false),
                    [0.0, 0.0, 1.0],
                ),
                (
                    w.top_pair.is_some_and(|p| same_pair(p[0], p[1])),
                    w.strip_height(true),
                    [0.0, 0.0, -1.0],
                ),
            ] {
                if !matched || !accepts(dir) {
                    continue;
                }
                out = out.max((extent - depth / 2.0).clamp(0.0, w.h));
            }
        }
    }
    out
}

/// 壁を考慮した部材せい D [mm]（原断面せい ＋ 壁の張り出し）。
fn depth_with_walls(
    model: &Model,
    elem: &squid_n_core::model::ElementData,
    walls: &[crate::wall::misc_wall::InFrameMiscWallGeometry],
) -> f64 {
    let depth = elem
        .section
        .and_then(|sid| model.sections.get(sid.index()))
        .map(|s| s.depth)
        .unwrap_or(0.0);
    depth + wall_protrusion(model, elem, depth, walls, None)
}

use squid_n_core::geom::element_axis as elem_axis;

/// 節点 `node` から部材フェースまでの距離 Lf [mm]（剛域長 λ 用。壁を含む）。
///
/// 対象部材と概ね直交する柱・大梁の最大せいの半分に、その部材へ取り付く壁の
/// 張り出しを加えた値。構造種別は問わない（剛域を設けるか否かの判定は
/// [`all_rc_src_at`] が別に行う）。
///
/// 直交材として数える範囲は、壁を含めない幾何量を求める
/// [`squid_n_core::face_distance`] と一致させること（柱・大梁 = `ElementKind::Beam`
/// のみ）。ここだけ壁要素や `PanelZone` を数えると、同じ「部材フェース」を指す 2 つの値が
/// 食い違う。
fn max_orth_face(
    model: &Model,
    node: squid_n_core::ids::NodeId,
    target_axis: [f64; 3],
    target_elem_idx: usize,
    adjacency: &NodeAdjacency,
    walls: &[crate::wall::misc_wall::InFrameMiscWallGeometry],
    toward: [f64; 3],
) -> f64 {
    let mut lf_max = 0.0_f64;
    for &ei in adjacency.indices_at(node) {
        if ei == target_elem_idx {
            continue;
        }
        let e = &model.elements[ei];
        if e.kind != squid_n_core::model::ElementKind::Beam {
            continue;
        }
        let axis = elem_axis(model, e);
        if squid_n_core::geom::vec3::dot(axis, target_axis).abs()
            >= squid_n_core::geom::ORTHOGONAL_DOT_MAX
        {
            // 概ね平行（45°未満）。直交材ではないので対象外。
            continue;
        }
        let Some(sec) = e.section.and_then(|sid| model.sections.get(sid.index())) else {
            continue;
        };
        // 節点→部材フェース = 直交部材せい/2 ＋ 対象部材の側へ張り出す壁の長さ。
        let lf = sec.depth / 2.0 + wall_protrusion(model, e, sec.depth, walls, Some(toward));
        lf_max = lf_max.max(lf);
    }
    lf_max
}

/// 節点 `node` に集合する柱・大梁（`ElementKind::Beam`）がすべて RC/SRC 系か。
///
/// 剛域を設けるのはこの条件が成り立つ節点だけとする（技術基準。1 本でも S 系の
/// 柱・大梁が集まる仕口では剛域を設けない）。対象部材自身もその節点に集合する
/// 部材の 1 つなので判定に含める。柱・大梁以外（ブレース・壁・仕口パネル）と、
/// 解析要素ではない二次部材（小梁・間柱）は判定の対象外。
fn all_rc_src_at(
    model: &Model,
    node: squid_n_core::ids::NodeId,
    adjacency: &NodeAdjacency,
) -> bool {
    adjacency.indices_at(node).iter().all(|&ei| {
        let e = &model.elements[ei];
        e.kind != squid_n_core::model::ElementKind::Beam
            || !member_structure_kind(model, e).is_steel_like()
    })
}

/// 剛域算定の本体（隣接マップは呼び出し側が構築して共有する）。
/// `target_elem_idx` は `model.elements` 内の対象要素の添字。
fn rigid_zone_with_adjacency(
    model: &Model,
    target_elem_idx: usize,
    adjacency: &NodeAdjacency,
    rule: &RigidZoneRule,
    walls: &[crate::wall::misc_wall::InFrameMiscWallGeometry],
    face: [f64; 2],
) -> RigidZone {
    let elem = &model.elements[target_elem_idx];
    let nodes = &elem.nodes;
    if nodes.len() < 2 {
        return RigidZone::default();
    }

    let target_axis = elem_axis(model, elem);

    let node_i = nodes[0];
    let node_j = nodes[nodes.len() - 1];
    let (ci, cj) = (
        model.nodes[node_i.index()].coord,
        model.nodes[node_j.index()].coord,
    );

    // 壁を考慮した部材せい D（設定で原断面に切り替えられる）。
    let d_self = if rule.consider_walls {
        depth_with_walls(model, elem, walls)
    } else {
        elem.section
            .and_then(|sid| model.sections.get(sid.index()))
            .map(|s| s.depth)
            .unwrap_or(0.0)
    };

    // 節点から部材フェースまでの距離 Lf。壁は向きを持つため、各端で
    // 「その節点から自部材が伸びる向き」を渡す。
    let dir_i = [cj[0] - ci[0], cj[1] - ci[1], cj[2] - ci[2]];
    let dir_j = [-dir_i[0], -dir_i[1], -dir_i[2]];
    let lf = |node, toward, walls: &[crate::wall::misc_wall::InFrameMiscWallGeometry]| {
        max_orth_face(
            model,
            node,
            target_axis,
            target_elem_idx,
            adjacency,
            walls,
            toward,
        )
    };
    // 剛域長 λ 用は壁込み（技術基準「剛域の計算」）。
    let lf_i = lf(node_i, dir_i, walls);
    let lf_j = lf(node_j, dir_j, walls);
    // 危険断面位置 face は壁を含めない幾何量で、算定は core の単一実装
    // （`squid_n_core::face_distance`）に一元化してある。壁の考慮は剛域の規定で
    // あって、危険断面位置は柱フェース（＝直交部材せい/2）で決まる（設計書 §6.2.3）。
    // face は RC/SRC 梁の自重の内法長にも使われるため、ここに壁を混ぜると
    // 壁の張り出し分だけ梁の自重が過小になる。
    let [face_i, face_j] = face;

    // 剛域長 λ = Lf − D_自身/4（技術基準「剛域の計算」）。
    // 剛域を設けるのは、その節点に集合する柱・大梁がすべて RC/SRC のときだけで、
    // 1 本でも S 系があればその端の剛域は 0 とする。S 造の仕口は剛域ではなく
    // 仕口パネル（`panel_offset_i/j`）でモデル化する。
    let lambda = |lf: f64, all_rc_src: bool| -> f64 {
        if !all_rc_src {
            return 0.0;
        }
        (lf - d_self / 4.0).max(0.0)
    };

    let (mut length_i, mut length_j) = (
        lambda(lf_i, all_rc_src_at(model, node_i, adjacency)),
        lambda(lf_j, all_rc_src_at(model, node_j, adjacency)),
    );

    // 両端の剛域長の合計が材長以上になる場合（壁の張り出しが大きい・短スパンの梁）は、
    // 材長の中点から部材せいの 1/4 の距離までを剛域とする
    // （技術基準「剛域長が重なる場合」）。これを行わないと可撓長が 0 以下になり、
    // 要素が剛性ゼロに退化する。
    let len = ((cj[0] - ci[0]).powi(2) + (cj[1] - ci[1]).powi(2) + (cj[2] - ci[2]).powi(2)).sqrt();
    if len > 0.0 && length_i + length_j >= len {
        let half = (len / 2.0 - d_self / 4.0).max(0.0);
        length_i = half;
        length_j = half;
    }

    RigidZone {
        length_i,
        length_j,
        source_i: ZoneSource::Auto,
        source_j: ZoneSource::Auto,
        // 剛域を設けない仕口（S 系を含む節点）でも危険断面位置は必要なので、
        // フェース距離は λ とは独立に常に持つ。
        face_i: Some(face_i),
        face_j: Some(face_j),
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
        return RigidZone::default();
    };
    let adjacency = NodeAdjacency::build(model);
    let walls = if rule.consider_walls {
        crate::wall::misc_wall::collect_rigid_zone_walls(model)
    } else {
        Vec::new()
    };
    let face = squid_n_core::face_distance::face_distances(model)[target_elem_idx];
    rigid_zone_with_adjacency(model, target_elem_idx, &adjacency, rule, &walls, face)
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
    // 壁の収集はモデル全体で 1 回だけ（要素ごとに走査すると O(E·W) になる）。
    let walls = if rule.consider_walls {
        crate::wall::misc_wall::collect_rigid_zone_walls(model)
    } else {
        Vec::new()
    };
    // 危険断面位置のフェース距離は core の単一実装から一括で得る（O(要素数)）。
    let faces = squid_n_core::face_distance::face_distances(model);
    let recomputed: Vec<(usize, RigidZone)> = model
        .elements
        .iter()
        .enumerate()
        .filter(|(_, e)| matches!(e.kind, squid_n_core::model::ElementKind::Beam))
        .map(|(i, _)| {
            (
                i,
                rigid_zone_with_adjacency(model, i, &adjacency, rule, &walls, faces[i]),
            )
        })
        .collect();

    for (i, rz) in recomputed {
        recompute_auto_zones(&mut model.elements[i].rigid_zone, &rz);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wall::misc_wall::InFrameMiscWallGeometry;
    use squid_n_core::ids::{ElemId, NodeId, SectionId, WallPlateId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node,
    };

    fn node(id: u32, coord: [f64; 3]) -> Node {
        Node {
            id: NodeId(id),
            coord,
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    /// 左辺（節点 0-3）の柱と、それに取り付く壁 1 枚を持つモデル。
    fn column_with_wall() -> (Model, ElementData) {
        let model = Model {
            nodes: vec![
                node(0, [0.0, 0.0, 0.0]),
                node(1, [4000.0, 0.0, 0.0]),
                node(2, [4000.0, 0.0, 3000.0]),
                node(3, [0.0, 0.0, 3000.0]),
            ],
            ..Default::default()
        };
        let column = ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        (model, column)
    }

    fn wall_geometry(column_face_slit: [bool; 2]) -> InFrameMiscWallGeometry {
        InFrameMiscWallGeometry {
            t: 150.0,
            lw: 4000.0,
            h: 3000.0,
            // 下辺は節点 0→1、上辺は節点 3→2。柱（0-3）は s=0 側にあたる。
            bottom_pair: Some([NodeId(0), NodeId(1)]),
            top_pair: Some([NodeId(3), NodeId(2)]),
            bottom_dir: [1.0, 0.0, 0.0],
            envelope: None,
            plate: WallPlateId(0),
            column_face_slit,
        }
    }

    /// 柱際が切れている側の柱には、袖壁による剛域の張り出しを与えない。
    ///
    /// 柱際スリットは柱との縁を切る指定なので、その柱の接合部が壁のぶん大きく
    /// なることはない。梁への腰壁・垂れ壁は別途残る（[`super::tests`] の
    /// 剛性側テストと同じ規約）。
    #[test]
    fn 柱際スリットのある側は剛域の袖壁張り出しを持たない() {
        let (model, column) = column_with_wall();
        let depth = 300.0;

        // スリット無し: 袖壁 lw/2=2000 から柱せいの半分を引いた 1850 が張り出す。
        let walls = [wall_geometry([false, false])];
        let plain = wall_protrusion(&model, &column, depth, &walls, None);
        assert!((plain - 1850.0).abs() < 1e-9, "{plain}");

        // s=0 側（柱 0-3 の側）を切ると張り出さない。
        let walls = [wall_geometry([true, false])];
        let slit = wall_protrusion(&model, &column, depth, &walls, None);
        assert!((slit - 0.0).abs() < 1e-9, "{slit}");

        // 反対側だけを切っても、この柱には効かない（左右独立であることの確認）。
        let walls = [wall_geometry([false, true])];
        let other = wall_protrusion(&model, &column, depth, &walls, None);
        assert!((other - 1850.0).abs() < 1e-9, "{other}");
    }
}
