//! 通り芯（[`AxisGroup`]）の自動生成。
//!
//! 柱が立つ平面位置から X 方向・Y 方向の通り芯を作る。通り芯は構造計算に用いない
//! 識別用のデータであり、生成は準備計算の一部ではなく利用者が明示的に実行する
//! 操作（モデルタブの「柱位置から自動生成」）である。
//!
//! 生成規則:
//!
//! 1. **対象**は柱（線材のうち両端の水平距離が [`crate::geom::VERTICAL_TOL_MM`]
//!    未満のもの＝[`crate::geom::is_vertical_pair`]）の材端節点のみ。梁・ブレース・
//!    二次部材・壁は対象にしない（通り芯を「柱が立つ位置」と定義する）。剛床代表節点は
//!    要素が接続しないため自然に対象外となる。
//! 2. **追加先グループ**は幾何で突き合わせる。離れを測る向きがグローバル X 軸に沿う
//!    平行芯グループ（[`AxisGroupKind::plan_dir`] が [`AxisPlanDir::X`]）があれば
//!    そこへ追加し、無ければ `X` グループ（原点 `(0,0)`・方向角 270°）を新設する。
//!    Y 方向も同様（新設時の方向角は 0°）。離れはそのグループの原点・方向角で測るため、
//!    取り込んだグループの符号規約にそのまま従う。
//! 3. **クラスタリング**は離れの昇順に [`AXIS_TOL_MM`] の許容差で行う。
//! 4. **既存優先**。同一グループ内に許容差以内の通りが既にあれば、その位置には
//!    新しい通りを作らない（[`AxisSource::Manual`] の通り＝手動作成・ST-Bridge
//!    取り込み・利用者が改名したものを保護する）。
//! 5. **採番**は離れの昇順に、そのグループ内で未使用の最小の正整数 `n` を用いて
//!    `{グループ名}{n}` とする。
//!
//! 既存の [`AxisSource::Auto`] の通りは毎回破棄して作り直す。
//! [`AxisGroupKind::Other`] のグループ（円弧芯・放射芯・作図芯）は対象外で、
//! そのまま保持する。

use crate::geom::is_vertical_pair;
use crate::ids::NodeId;
use crate::model::{Axis, AxisGroup, AxisGroupKind, AxisPlanDir, AxisSource, ElementKind, Model};

/// 同一の通りとみなす離れの差 [mm]。
pub const AXIS_TOL_MM: f64 = 1.0;

/// 新設する X 方向グループの名前と方向角 [度]（芯線は Y 軸に平行、離れは +X 向き）。
const X_GROUP: (&str, f64) = ("X", 270.0);
/// 新設する Y 方向グループの名前と方向角 [度]（芯線は X 軸に平行、離れは +Y 向き）。
const Y_GROUP: (&str, f64) = ("Y", 0.0);

/// 柱位置から通り芯を自動生成し、適用後の [`Model::axes`] の全量を返す。
///
/// モデルは変更しない（呼び出し側が編集コマンド経由で適用する）。
/// 生成規則はモジュールドキュメントを参照。
pub fn generate_axes(model: &Model) -> Vec<AxisGroup> {
    // 柱の材端節点（平面位置つき）。同じ位置に上下階の柱が積まれるため節点は重複し得る。
    let mut column_nodes: Vec<NodeId> = Vec::new();
    for e in &model.elements {
        if !matches!(e.kind, ElementKind::Beam) || e.nodes.len() != 2 {
            continue;
        }
        let (Some(a), Some(b)) = (
            model.nodes.get(e.nodes[0].index()),
            model.nodes.get(e.nodes[1].index()),
        ) else {
            continue;
        };
        if !is_vertical_pair(a.coord, b.coord) {
            continue;
        }
        column_nodes.push(a.id);
        column_nodes.push(b.id);
    }
    column_nodes.sort();
    column_nodes.dedup();

    // 既存の自動生成分を捨てる（手動・取り込み由来は保持）。
    let mut groups: Vec<AxisGroup> = model.axes.clone();
    for g in &mut groups {
        g.axes.retain(|a| a.source == AxisSource::Manual);
    }

    for (dir, (default_name, default_angle)) in
        [(AxisPlanDir::X, X_GROUP), (AxisPlanDir::Y, Y_GROUP)]
    {
        let gi = match groups.iter().position(|g| g.kind.plan_dir() == Some(dir)) {
            Some(i) => i,
            None => {
                if column_nodes.is_empty() {
                    continue;
                }
                groups.push(AxisGroup {
                    name: unused_group_name(&groups, default_name),
                    kind: AxisGroupKind::Parallel {
                        origin: [0.0, 0.0],
                        angle_deg: default_angle,
                    },
                    axes: Vec::new(),
                });
                groups.len() - 1
            }
        };
        add_generated_axes(&mut groups[gi], model, &column_nodes);
    }

    groups
}

/// 既存グループと重複しないグループ名を選ぶ（`X` が使われていれば `X2`, `X3`…）。
fn unused_group_name(groups: &[AxisGroup], base: &str) -> String {
    if !groups.iter().any(|g| g.name == base) {
        return base.to_string();
    }
    (2..)
        .map(|n| format!("{base}{n}"))
        .find(|name| !groups.iter().any(|g| &g.name == name))
        .expect("無限イテレータから必ず見つかる")
}

/// 1 つのグループへ、柱位置から作った通りを追加する。
fn add_generated_axes(group: &mut AxisGroup, model: &Model, column_nodes: &[NodeId]) {
    // 柱の材端節点を、このグループの原点・方向角で測った離れへ写す。
    let mut by_distance: Vec<(f64, NodeId)> = column_nodes
        .iter()
        .filter_map(|&id| {
            let n = model.nodes.get(id.index())?;
            let d = group.kind.distance_of(n.coord[0], n.coord[1])?;
            Some((d, id))
        })
        .collect();
    by_distance.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));

    // 離れの昇順にクラスタリングする（代表値はクラスタ先頭の離れ。
    // 階生成の Z レベル判定と同じく、代表値との差で判定して連鎖的な流れを防ぐ）。
    let mut clusters: Vec<(f64, Vec<NodeId>)> = Vec::new();
    for (d, id) in by_distance {
        match clusters.last_mut() {
            Some((rep, nodes)) if (d - *rep).abs() <= AXIS_TOL_MM => nodes.push(id),
            _ => clusters.push((d, vec![id])),
        }
    }

    // 既存の通り（Manual）と同じ位置は作らない（既存優先）。
    clusters.retain(|(d, _)| {
        !group
            .axes
            .iter()
            .any(|a| a.distance.is_some_and(|ad| (ad - d).abs() <= AXIS_TOL_MM))
    });

    for (d, nodes) in clusters {
        let name = unused_axis_name(group);
        group.axes.push(Axis {
            name,
            distance: Some(d),
            nodes,
            source: AxisSource::Auto,
        });
    }
    group.sort_axes();
}

/// グループ内で未使用の最小の正整数 `n` による通り名 `{グループ名}{n}`。
fn unused_axis_name(group: &AxisGroup) -> String {
    (1..)
        .map(|n| format!("{}{n}", group.name))
        .find(|name| !group.axes.iter().any(|a| &a.name == name))
        .expect("無限イテレータから必ず見つかる")
}

#[cfg(test)]
mod tests;
