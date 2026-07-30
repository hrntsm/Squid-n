//! 梁のねじり剛性の既定モデル化（i 端ねじれ解放）。
//!
//! 日本の一貫計算プログラムでは、床スラブと一体になる大梁のねじり剛性を
//! 設計上期待しないのが通例である（曲げ・せん断で抵抗させ、ねじりモーメントは
//! 負担させない）。本実装もこれに倣い、**水平材（梁）の i 端のねじれ回転
//! （材軸まわり回転）を既定でピン（解放）**として要素剛性を組む。解放した端の
//! ねじりモーメントは 0 となり、ねじりは材長方向に一定であるため部材全長で
//! \\( M_x = 0 \\) となる（＝ねじり剛性を期待しないモデル化）。
//!
//! ただしねじりを解放すると、その部材はねじれ回転に一切剛性を持たなくなるため、
//! **材軸まわり回転を他に拘束するものが無い節点では全体剛性行列が特異になる**。
//! 典型は「柱が無く、一直線に並ぶ梁だけが集まる節点」（大梁を中間で分割した点）で、
//! 直交部材の曲げも支点拘束も効かないため、その節点の材軸まわり回転が浮いてしまう。
//! そこで [`i_end_torsion_release`] は、解放しても特異にならないことを節点ごとに
//! 確かめてから解放を許す（確かめられない部材は従来どおり \\( GJ/L \\) を保持する。
//! 安全側のフォールバック）。
//!
//! 判定は「その部材の材軸 \\( e_x \\) まわりの回転が、当該部材のねじり以外の経路で
//! 拘束されるか」を両端節点について見る。回転剛性を与える経路は次のいずれか。
//!
//! - **非平行な線材の曲げ**: 線材は材軸に直交する 2 方向まわりの曲げ剛性を持つため、
//!   互いに非平行な線材が 2 方向以上集まる節点は、すべての回転軸が曲げで拘束される。
//! - **線材以外の要素**（壁・シェル・パネルゾーン・ばね・免震・ダンパー）: 節点に
//!   回転剛性を与え得るため、拘束ありとみなす。
//! - **支点拘束**（`Node::restraint`）: 材軸成分を持つ回転自由度がすべて拘束されている。
//! - **支点ばね**（`Node::support_spring`）の回転成分が正。

use squid_n_core::dof::Dof;
use squid_n_core::model::{BeamTorsionMode, ElementData, ElementKind, Model};

/// 水平材（梁）と見なす勾配のしきい値。水平投影長に対する高低差の比で、
/// 断面性能の割増し判定（`stiffness_factors`）と同じ 5% を用いる。
const HORIZONTAL_SLOPE_TOL: f64 = 0.05;

/// 線材（材軸まわりのねじりと、それに直交する 2 軸まわりの曲げを持つ要素）か。
fn is_line_member(kind: ElementKind) -> bool {
    matches!(
        kind,
        ElementKind::Beam
            | ElementKind::Fiber
            | ElementKind::MultiSpring
            | ElementKind::Brace { .. }
    )
}

/// 要素の材軸単位ベクトル（2 節点未満・退化長さは None）。
fn axis_of(data: &ElementData, model: &Model) -> Option<[f64; 3]> {
    if data.nodes.len() < 2 {
        return None;
    }
    let p0 = model.nodes.get(data.nodes[0].index())?.coord;
    let p1 = model.nodes.get(data.nodes[1].index())?.coord;
    let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    (l > 1e-9).then(|| [d[0] / l, d[1] / l, d[2] / l])
}

/// 材軸が水平（勾配 5% 以下）か。
fn is_horizontal(axis: [f64; 3]) -> bool {
    let lp = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
    lp > 1e-9 && axis[2].abs() <= HORIZONTAL_SLOPE_TOL * lp
}

/// 単位ベクトル 2 本が平行（向きの反転を含む）か。
fn is_parallel(a: [f64; 3], b: [f64; 3]) -> bool {
    let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    dot.abs() > 1.0 - 1e-6
}

/// 節点 `node` において、軸 `axis` まわりの回転が「部材 `elem` のねじり以外」で
/// 拘束されるか。判定規則はモジュールドキュメント参照。
fn rotation_restrained_elsewhere(
    model: &Model,
    node: squid_n_core::ids::NodeId,
    axis: [f64; 3],
    elem: squid_n_core::ids::ElemId,
) -> bool {
    // 支点拘束: 軸成分を持つ回転自由度がすべて拘束されていれば、その軸まわりの
    // 回転は支点で止まる。
    if let Some(n) = model.nodes.get(node.index()) {
        let rot_dofs = [Dof::Rx, Dof::Ry, Dof::Rz];
        let needed: Vec<Dof> = rot_dofs
            .iter()
            .enumerate()
            .filter(|(i, _)| axis[*i].abs() > 1e-9)
            .map(|(_, d)| *d)
            .collect();
        if !needed.is_empty() && needed.iter().all(|d| n.restraint.is_fixed(*d)) {
            return true;
        }
        // 支点ばね [kx, ky, kz, krx, kry, krz] の回転成分。
        if let Some(k) = n.support_spring {
            let ok = (0..3)
                .filter(|i| axis[*i].abs() > 1e-9)
                .all(|i| k[3 + i] > 0.0);
            if ok {
                return true;
            }
        }
    }
    for other in &model.elements {
        if other.id == elem || !other.nodes.contains(&node) {
            continue;
        }
        if !is_line_member(other.kind) {
            // 壁・シェル・パネルゾーン・ばね類は回転剛性を与え得るため拘束ありとみなす。
            return true;
        }
        match axis_of(other, model) {
            // 非平行な線材の曲げが軸まわりの回転を拘束する。
            Some(a) if !is_parallel(a, axis) => return true,
            // 平行な線材はねじりでしか軸まわりを拘束できず、そのねじりも
            // 同じ既定で解放され得るため、拘束ありとはみなさない。
            _ => {}
        }
    }
    false
}

/// 部材 `data` の i 端ねじれを既定モデル化として解放してよいか。
///
/// 対象は水平材（梁）のみで、柱（鉛直材）・ブレース等のねじりは保持する。
/// 解放によって材軸まわり回転が浮く節点が生じる場合は解放しない
/// （モジュールドキュメントの判定規則を参照）。
/// モデルが `BeamTorsionMode::Keep`（ねじり剛性を保持するモデル化。床小梁の
/// 格子解析など）の場合は常に解放しない。
pub fn i_end_torsion_release(data: &ElementData, model: &Model) -> bool {
    if model.beam_torsion != BeamTorsionMode::ReleaseIEnd {
        return false;
    }
    if !is_line_member(data.kind) {
        return false;
    }
    let Some(axis) = axis_of(data, model) else {
        return false;
    };
    if !is_horizontal(axis) {
        return false;
    }
    data.nodes
        .iter()
        .take(2)
        .all(|n| rotation_restrained_elsewhere(model, *n, axis, data.id))
}
