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
use crate::model::{ElementKind, Model};

/// 直交とみなす軸内積の上限（概ね 45°以上で直交扱い）。
const ORTHOGONAL_DOT_MAX: f64 = 0.707;

/// 部材の単位軸ベクトル。長さ 0 の部材は零ベクトルを返す。
fn elem_axis(model: &Model, e: &crate::model::ElementData) -> [f64; 3] {
    if e.nodes.len() < 2 {
        return [0.0, 0.0, 0.0];
    }
    let (Some(n0), Some(n1)) = (
        model.nodes.get(e.nodes[0].index()),
        model.nodes.get(e.nodes[e.nodes.len() - 1].index()),
    ) else {
        return [0.0, 0.0, 0.0];
    };
    let d = [
        n1.coord[0] - n0.coord[0],
        n1.coord[1] - n0.coord[1],
        n1.coord[2] - n0.coord[2],
    ];
    let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if l < 1e-12 {
        [0.0, 0.0, 0.0]
    } else {
        [d[0] / l, d[1] / l, d[2] / l]
    }
}

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
        let dot =
            (axis[0] * target_axis[0] + axis[1] * target_axis[1] + axis[2] * target_axis[2]).abs();
        if dot >= ORTHOGONAL_DOT_MAX {
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

/// 算定した柱フェース距離を `RigidZone::face_i/face_j` へ反映する（冪等）。
///
/// 剛域長は変更しない。剛域長もあわせて算定する場合は
/// `squid_n_element::beam::apply_auto_rigid_zones` を使う。
pub fn apply_face_distances(model: &mut Model) {
    let faces = face_distances(model);
    for (e, f) in model.elements.iter_mut().zip(faces) {
        if e.kind != ElementKind::Beam {
            continue;
        }
        e.rigid_zone.face_i = Some(f[0]);
        e.rigid_zone.face_j = Some(f[1]);
    }
}
