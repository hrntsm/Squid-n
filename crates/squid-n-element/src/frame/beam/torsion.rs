//! 部材のねじり剛性の既定モデル化（i 端ねじれ解放）。
//!
//! 日本の一貫計算プログラムでは、床スラブと一体になる大梁のねじり剛性を
//! 設計上期待しないのが通例である（曲げ・せん断で抵抗させ、ねじりモーメントは
//! 負担させない）。本実装もこれに倣い、**線材（梁・柱）の i 端のねじれ回転
//! （材軸まわり回転）を既定でピン（解放）**として要素剛性を組む。解放した端の
//! ねじりモーメントは 0 となり、ねじりは材長方向に一定であるため部材全長で
//! \\( M_x = 0 \\) となる（＝ねじり剛性を期待しないモデル化）。
//!
//! 建物一律の切替は `Model::beam_torsion`（[`BeamTorsionMode`]）で、既定は
//! `ReleaseIEnd`。`Keep` にすると全部材でねじり剛性 \\( GJ/L \\) を保持する。
//!
//! ただしねじりを解放すると、その部材はねじれ回転に一切剛性を持たなくなるため、
//! **材軸まわり回転を他に拘束するものがない節点では全体剛性行列が特異になる**。
//! 典型は「一直線に並ぶ線材だけが集まる節点」（大梁・柱を中間で分割した点）で、
//! 直交部材の曲げも支点拘束も効かないため、その節点の材軸まわり回転が浮いてしまう。
//! そこで [`i_end_torsion_release`] は、解放しても特異にならないことを節点ごとに
//! 確かめてから解放を許す（確かめられない部材は従来どおり \\( GJ/L \\) を保持する。
//! 安全側のフォールバック）。理由付きの判定結果は [`i_end_torsion_release_skip`]
//! が返し、準備計算の「ねじり解放の対象外部材」表に用いる。
//!
//! 判定は「その部材の材軸 \\( e_x \\) まわりの回転が、当該部材のねじり以外の経路で
//! 拘束されるか」を両端節点について見る。回転剛性を与える経路は次のいずれか。
//!
//! - **非平行な線材の曲げ**: 線材は材軸に直交する 2 方向まわりの曲げ剛性を持つため、
//!   互いに非平行な線材が 2 方向以上集まる節点は、すべての回転軸が曲げで拘束される。
//! - **線材以外の要素**（壁・シェル・ばね・免震・ダンパー）: 節点に
//!   回転剛性を与え得るため、拘束ありとみなす。
//! - **支点拘束**（`Node::restraint`）: 材軸成分を持つ回転自由度がすべて拘束されている。
//! - **支点ばね**（`Node::support_spring`）の回転成分が正。
//!
//! 仕口パネル（`ElementKind::PanelZone`）は判定に含めない。パネルが剛性を与える
//! のは接合部のせん断変形角（節点の標準 6 自由度とは別枠の追加自由度）だけで、
//! 節点の回転自由度には一切寄与しないため、拘束ありとみなすと材軸まわり回転が
//! 浮いた節点を見逃す。
//!
//! 剛床（`Constraint::RigidDiaphragm`）は判定に含めない。剛床が拘束するのは面内
//! 3 成分（Ux・Uy・Rz）で、スレーブ節点の Rz が独立自由度でなくなる一方、その
//! 拘束はマスター節点の Rz へ集約されるだけである。「柱 1 本だけが載る剛床」では
//! マスターの Rz を止めるのがその柱のねじりだけになるため、剛床を「拘束あり」と
//! みなすと特異化を見逃す。安全側に倒して考慮しない。

use squid_n_core::dof::Dof;
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{BeamTorsionMode, ElementData, ElementKind, Model};

/// i 端ねじれを解放しない理由。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TorsionReleaseSkip {
    /// 建物一律の設定が `BeamTorsionMode::Keep`（ねじり剛性を保持する）。
    ModeKeep,
    /// 線材ではない（壁・シェル・ばね類）。ねじり自由度の概念を持たない。
    NotLineMember,
    /// 材軸が定まらない（2 節点未満・退化長さ）。
    DegenerateAxis,
    /// 材軸まわり回転を拘束するものがない節点がある（解放すると特異化する）。
    /// `node` はその節点。
    UnrestrainedRotation { node: NodeId },
}

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

/// 単位ベクトル 2 本が平行（向きの反転を含む）か。
fn is_parallel(a: [f64; 3], b: [f64; 3]) -> bool {
    let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    dot.abs() > 1.0 - 1e-6
}

/// 節点 `node` において、軸 `axis` まわりの回転が「部材 `elem` のねじり以外」で
/// 拘束されるか。判定規則はモジュールドキュメント参照。
fn rotation_restrained_elsewhere(
    model: &Model,
    node: NodeId,
    axis: [f64; 3],
    elem: ElemId,
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
        // 仕口パネルは接合部のせん断変形角（節点の標準 6 自由度とは別枠の追加
        // 自由度）にのみ剛性を与え、節点の回転自由度には一切寄与しない。
        // `ElementData::nodes` には描画用に接続部材の節点も並ぶが、それらの
        // 節点に対しても剛性を与えないため、拘束ありとみなしてはならない
        // （みなすと材軸まわり回転が浮いた節点を見逃し、全体剛性行列が特異になる）。
        if matches!(other.kind, ElementKind::PanelZone) {
            continue;
        }
        if !is_line_member(other.kind) {
            // 壁・シェル・ばね類は回転剛性を与え得るため拘束ありとみなす。
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

/// 部材 `data` の i 端ねじれを解放しない理由（解放してよければ `None`）。
///
/// 準備計算の「ねじり解放の対象外部材」表は、この戻り値のうち
/// [`TorsionReleaseSkip::UnrestrainedRotation`]（＝ねじり剛性が残る部材）だけを
/// 一覧する。
pub fn i_end_torsion_release_skip(data: &ElementData, model: &Model) -> Option<TorsionReleaseSkip> {
    if model.beam_torsion != BeamTorsionMode::ReleaseIEnd {
        return Some(TorsionReleaseSkip::ModeKeep);
    }
    if !is_line_member(data.kind) {
        return Some(TorsionReleaseSkip::NotLineMember);
    }
    let Some(axis) = axis_of(data, model) else {
        return Some(TorsionReleaseSkip::DegenerateAxis);
    };
    for node in data.nodes.iter().take(2) {
        if !rotation_restrained_elsewhere(model, *node, axis, data.id) {
            return Some(TorsionReleaseSkip::UnrestrainedRotation { node: *node });
        }
    }
    None
}

/// 部材 `data` の i 端ねじれを既定モデル化として解放してよいか。
///
/// 対象は線材（梁・柱・ブレース）で、解放によって材軸まわり回転が浮く節点が
/// 生じる場合は解放しない（モジュールドキュメントの判定規則を参照）。
/// モデルが `BeamTorsionMode::Keep`（ねじり剛性を保持するモデル化。床小梁の
/// 格子解析など）の場合は常に解放しない。
pub fn i_end_torsion_release(data: &ElementData, model: &Model) -> bool {
    i_end_torsion_release_skip(data, model).is_none()
}
