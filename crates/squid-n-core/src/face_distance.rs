//! 柱フェース距離（節点から部材フェースまでの距離）の算定。
//!
//! フェース距離は「その端で直交する部材の最大せいの半分」で、接合関係と断面せい
//! だけから一意に決まる**幾何量**である。剛域長のようなモデル化の設定には
//! 左右されない。危険断面位置・RC/SRC 梁の自重の内法長・数量積算の鉄筋長さなど、
//! 剛域とは無関係な用途がこの値を読む。
//!
//! # なぜ core にあるか
//!
//! 以前はこの算定が `squid_n_element` の剛域算定（`apply_auto_rigid_zones`）の
//! 中だけにあり、結果を `RigidZone::face_i/face_j` へキャッシュしていた。
//! そのため「剛域を算定する前に読むと 0 になる」という順序依存があり、
//! 実際に固定荷重が 9.6% 過大になる不具合を生んだ（`dev_docs/handoff/`
//! 「実モデル統合テスト」4.1 節）。
//!
//! 幾何量は幾何から求めれば順序に依存しない。そこで算定を core へ置き、
//! 上位クレート（`squid_n_load` の自重算定など）がキャッシュを当てにせず
//! [`face_distances`] で直接求められるようにしている。

use crate::adjacency::NodeAdjacency;
use crate::geom::{element_axis as elem_axis, vec3, ORTHOGONAL_DOT_MAX};
use crate::model::{ElementKind, Model};

/// 節点 `node` で対象部材と概ね直交する Beam 要素の最大せいの半分 [mm]。
/// 直交材がない端は 0.0。構造種別は問わない（幾何量のため）。
fn face_at(
    model: &Model,
    node: crate::ids::NodeId,
    target_axis: [f64; 3],
    target_elem_idx: usize,
    adjacency: &NodeAdjacency,
) -> f64 {
    let mut d_max = 0.0_f64;
    for &ei in adjacency.indices_at(node) {
        if ei == target_elem_idx {
            continue;
        }
        let e = &model.elements[ei];
        if e.kind != ElementKind::Beam {
            continue;
        }
        let axis = elem_axis(model, e);
        if vec3::dot(axis, target_axis).abs() >= ORTHOGONAL_DOT_MAX {
            continue;
        }
        if let Some(sec) = e.section.and_then(|sid| model.sections.get(sid.index())) {
            d_max = d_max.max(sec.depth);
        }
    }
    d_max / 2.0
}

/// モデルの全要素について、両端の柱フェース距離 `[i 端, j 端]` [mm] を求める。
///
/// 添字は `model.elements` の並びと一致する。Beam 以外の要素と、節点が 2 つ
/// 未満の要素は `[0.0, 0.0]`。計算量は O(要素数)。
///
/// キャッシュ（`RigidZone::face_i/face_j`）を当てにできない場所から使う。
pub fn face_distances(model: &Model) -> Vec<[f64; 2]> {
    let adjacency = NodeAdjacency::build(model);
    model
        .elements
        .iter()
        .enumerate()
        .map(|(i, e)| {
            if e.kind != ElementKind::Beam || e.nodes.len() < 2 {
                return [0.0, 0.0];
            }
            let axis = elem_axis(model, e);
            let ni = e.nodes[0];
            let nj = e.nodes[e.nodes.len() - 1];
            [
                face_at(model, ni, axis, i, &adjacency),
                face_at(model, nj, axis, i, &adjacency),
            ]
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{ElemId, MaterialId, NodeId, SectionId};
    use crate::model::{
        ElementData, EndCondition, ForceRegime, LocalAxis, Node, RigidZone, Section,
    };

    fn node(id: u32, c: [f64; 3]) -> Node {
        Node {
            id: NodeId(id),
            coord: c,
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    fn section(id: u32, depth: f64) -> Section {
        Section {
            id: SectionId(id),
            name: String::new(),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: None,
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        }
    }

    fn elem(id: u32, kind: ElementKind, a: u32, b: u32, sec: u32) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind,
            nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
            section: Some(SectionId(sec)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    /// 柱（せい 600）が取り付く端のフェース距離は柱せいの半分、直交材がない端は 0。
    #[test]
    fn 直交材のせいの半分をフェース距離とする() {
        let model = Model {
            nodes: vec![
                node(0, [0.0, 0.0, 0.0]),
                node(1, [0.0, 0.0, 3000.0]),
                node(2, [4000.0, 0.0, 3000.0]),
            ],
            elements: vec![
                elem(0, ElementKind::Beam, 0, 1, 0),
                elem(1, ElementKind::Beam, 1, 2, 1),
            ],
            sections: vec![section(0, 600.0), section(1, 700.0)],
            ..Default::default()
        };
        let f = face_distances(&model);
        // 梁（要素 1）: i 端に柱が取り付くので 600/2、j 端は直交材なしで 0。
        assert_eq!(f[1], [300.0, 0.0]);
        // 柱（要素 0）: 上端に梁が取り付くので 700/2、下端は直交材なしで 0。
        assert_eq!(f[0], [0.0, 350.0]);
    }

    /// フェース距離を決めるのは柱・大梁だけで、壁は数えない。
    ///
    /// 壁を数えると、剛域長を求めるときの「部材フェース」と食い違う。
    #[test]
    fn 壁はフェース距離に数えない() {
        let mut model = Model {
            nodes: vec![
                node(0, [0.0, 0.0, 0.0]),
                node(1, [0.0, 0.0, 3000.0]),
                node(2, [4000.0, 0.0, 3000.0]),
            ],
            elements: vec![elem(0, ElementKind::Beam, 1, 2, 0)],
            sections: vec![section(0, 700.0), section(1, 9999.0)],
            ..Default::default()
        };
        assert_eq!(face_distances(&model)[0], [0.0, 0.0]);

        // 梁の i 端に直交する壁を足しても変わらない。
        model.elements.push(elem(1, ElementKind::Wall, 0, 1, 1));
        assert_eq!(face_distances(&model)[0], [0.0, 0.0]);

        // 同じ位置に柱（Beam）を足すと、そのせいの半分が効く。
        model.elements.push(elem(2, ElementKind::Beam, 0, 1, 1));
        assert_eq!(face_distances(&model)[0], [4999.5, 0.0]);
    }
}
