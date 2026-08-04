//! フレーム外雑壁（部材化しない壁）の重量集計。
//!
//! - [`misc_wall_weight_shares`] — 雑壁重量の近傍節点への配分（`model.nodes` 添字 → [N]）
//! - [`accumulate_misc_wall_weight`] — 配分結果を節点重量へ加算する
//! - [`nearest_node`] — 指定点に最も近い節点
//! - [`nearest_column_element`] — 指定点に最も近い柱要素

use super::geom::{dist3, is_vertical_pair};
use super::*;

/// モデル全節点のうち、指定点に最も近い節点。
fn nearest_node(model: &Model, pt: [f64; 3]) -> Option<NodeId> {
    model
        .nodes
        .iter()
        .min_by(|a, b| dist3(a.coord, pt).total_cmp(&dist3(b.coord, pt)))
        .map(|n| n.id)
}

/// 「柱要素」候補（鉛直な `Beam` 要素、要素本体＋両端座標）。`misc_wall_weight_shares`
/// が壁を 500mm ごとに区分し、区分ごとに最近接柱を探す際、種別・節点数判定と
/// 鉛直判定・座標引きを毎回繰り返さないよう、壁ループの前に 1 回だけ構築して
/// 使い回す（性能。走査順・最近接判定ロジックは変更しない）。
struct ColumnCandidate<'a> {
    elem: &'a ElementData,
    a: [f64; 3],
    b: [f64; 3],
}

/// `model` から「柱要素」（鉛直な `Beam` 要素）候補（両端座標つき）を集める。
fn column_candidates(model: &Model) -> Vec<ColumnCandidate<'_>> {
    let mut out = Vec::new();
    for e in &model.elements {
        if e.kind != ElementKind::Beam || e.nodes.len() < 2 {
            continue;
        }
        let (a, b) = (
            model.nodes[e.nodes[0].index()].coord,
            model.nodes[e.nodes[1].index()].coord,
        );
        if !is_vertical_pair(a, b) {
            continue;
        }
        out.push(ColumnCandidate { elem: e, a, b });
    }
    out
}

/// 「柱要素」候補のうち、指定点に最も近い節点を持つ要素。
/// §フレーム外雑壁の「柱」伝達タイプに用いる（最近接の柱節点→その柱要素の上下節点）。
fn nearest_column_element<'a>(
    candidates: &[ColumnCandidate<'a>],
    pt: [f64; 3],
) -> Option<&'a ElementData> {
    let mut best: Option<(&ElementData, f64)> = None;
    for c in candidates {
        let d = dist3(c.a, pt).min(dist3(c.b, pt));
        if best.map(|(_, bd)| d < bd).unwrap_or(true) {
            best = Some((c.elem, d));
        }
    }
    best.map(|(e, _)| e)
}

/// フレーム外雑壁（`Model.misc_walls`）の重量を近傍節点へ集計する（§フレーム外雑壁）。
///
/// 壁を長さ 0.5m（端数含む）ごとの領域に分割し、各領域重量
/// `weight_per_area × height × 領域長` を、領域中心位置（下端線分上の点、
/// 高さ方向に `height/2` を加えた 3D 点）から近い節点へ `transfer` の規則で伝達する。
/// - `Column`: 最も近い柱要素（鉛直 `Beam`）の上下 2 節点へ 1/2 ずつ
///   （柱が見つからない場合はモデル全節点中の最近接節点へ全量）
/// - `Beam`: モデル全節点中の最近接節点へ全量集中
/// - `SelfStanding`: 自立壁の簡易扱い。解析用の専用節点を新設する代わりに
///   モデル全節点中の最近接節点（配置階の代表的な節点）へ全量を伝達する
///   （配置階の剛床へ伝達する扱いの簡易近似）。
pub(super) fn accumulate_misc_wall_weight(model: &Model, node_weight: &mut [f64]) {
    for (i, w) in misc_wall_weight_shares(model) {
        node_weight[i] += w;
    }
}

/// [`accumulate_misc_wall_weight`] の配分内容（`model.nodes` 添字 → [N]）を返す。
/// 地震用重量の集計と自重(自動)荷重ケース（[`crate::self_weight`]）で共有する。
pub(crate) fn misc_wall_weight_shares(model: &Model) -> Vec<(usize, f64)> {
    const SEGMENT_LEN: f64 = 500.0;
    let mut shares: Vec<(usize, f64)> = Vec::new();
    // 「柱」伝達タイプの区分ごとの最近接探索が使う候補列を 1 回だけ構築する
    // （性能。`nearest_node` は節点数分の走査のままとし、タイブレーク挙動を
    // 変えるおそれのある空間索引は導入しない）。
    let columns = column_candidates(model);
    for wall in &model.misc_walls {
        let (s, e) = (wall.start, wall.end);
        let (dx, dy, dz) = (e[0] - s[0], e[1] - s[1], e[2] - s[2]);
        let total_len = (dx * dx + dy * dy + dz * dz).sqrt();
        if total_len <= 0.0 {
            continue;
        }
        let mut offset = 0.0;
        while offset < total_len - 1e-9 {
            let seg_len = SEGMENT_LEN.min(total_len - offset);
            let t_center = (offset + seg_len / 2.0) / total_len;
            let center = [
                s[0] + dx * t_center,
                s[1] + dy * t_center,
                s[2] + dz * t_center + wall.height / 2.0,
            ];
            let seg_weight = wall.weight_per_area * wall.height * seg_len;

            match wall.transfer {
                MiscWallTransfer::Column => {
                    if let Some(col) = nearest_column_element(&columns, center) {
                        let ni = col.nodes[0].index();
                        let nj = col.nodes[1].index();
                        shares.push((ni, seg_weight / 2.0));
                        shares.push((nj, seg_weight / 2.0));
                    } else if let Some(n) = nearest_node(model, center) {
                        shares.push((n.index(), seg_weight));
                    }
                }
                MiscWallTransfer::Beam | MiscWallTransfer::SelfStanding => {
                    if let Some(n) = nearest_node(model, center) {
                        shares.push((n.index(), seg_weight));
                    }
                }
            }

            offset += SEGMENT_LEN;
        }
    }
    shares
}
