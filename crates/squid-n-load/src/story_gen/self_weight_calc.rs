//! 自重（線材・壁・シェル・ダンパー）の列挙と算定。
//!
//! - [`SelfWeightItem`] — 自重 1 件分の重量と帰属の中間表現
//! - [`enumerate_self_weight`] — モデル全要素の自重を列挙する
//! - [`steel_density_ton_mm3`] — 鋼材の質量密度 [ton/mm³]
//! - [`finish_perimeter`] — 仕上げ周長 φ
//! - [`wall_clear_area_factor`] — 耐震壁の内法係数

use std::collections::HashMap;

use super::geom::{dist3, is_vertical_pair, polygon_area_3d};
use super::*;

/// 節点対の順不同キー（`(min,max)`）。`node_adjacency`/`beam_pair_map` で
/// ノード順に依存しない同じキーを使うための共通ヘルパー。
fn ordered_pair(a: NodeId, b: NodeId) -> (NodeId, NodeId) {
    if a.0 <= b.0 {
        (a, b)
    } else {
        (b, a)
    }
}

/// 節点 → その節点に取り付く線材（`Beam`/`Brace`、2 節点以上）の `model.elements`
/// 添字一覧。`has_column_below`/`max_depth`（柱脚に接続する部材の探索）を
/// O(柱数×要素数) から O(1) 参照へ落とすための事前索引（`enumerate_self_weight`
/// の主ループ前に 1 回だけ構築する）。
fn node_adjacency(model: &Model) -> HashMap<NodeId, Vec<usize>> {
    let mut adj: HashMap<NodeId, Vec<usize>> = HashMap::new();
    for (idx, e) in model.elements.iter().enumerate() {
        if matches!(e.kind, ElementKind::Beam | ElementKind::Brace { .. }) && e.nodes.len() >= 2 {
            for &n in &e.nodes {
                adj.entry(n).or_default().push(idx);
            }
        }
    }
    adj
}

/// 節点対 (min,max)（辺の両端。線材が3節点以上でも先頭・末尾を辺とみなす。
/// `wall_clear_area_factor` の元の照合と同じ）→ `Beam` 要素の `model.elements`
/// 添字。壁の各辺→柱梁対応付けを O(壁の辺数×要素数) から O(1) 参照へ落とすための
/// 事前索引（`enumerate_self_weight` で1回構築し使い回す）。同一節点対に複数の
/// 候補がある場合は要素順で最初に見つかったものを採用する（元の
/// `elements.iter().find` の挙動を保つため `entry().or_insert` を使う）。
fn beam_pair_map(model: &Model) -> HashMap<(NodeId, NodeId), usize> {
    let mut map = HashMap::new();
    for (idx, e) in model.elements.iter().enumerate() {
        if e.kind == ElementKind::Beam && e.nodes.len() >= 2 {
            let (a, b) = (e.nodes[0], e.nodes[e.nodes.len() - 1]);
            map.entry(ordered_pair(a, b)).or_insert(idx);
        }
    }
    map
}

/// 鋼材単位体積重量（γs=77kN/m³、実務慣用値）を内部単位系の質量密度
/// [ton/mm³] に換算した値（≈7.85e-9）。ダンパー支持部重量（§ダンパー自重）に用いる。
/// `squid-n-core::units` の単一ソースオブトゥルースから導出する（レビュー §1.11 と同じ方針）。
pub(crate) fn steel_density_ton_mm3() -> f64 {
    squid_n_core::units::to_internal::mass_density_from_unit_weight_kn_m3(
        squid_n_core::units::STEEL_UNIT_WEIGHT_KN_M3,
    )
}

/// 仕上げ周長 φ（柱梁自重の仕上げ荷重）。
/// 鉛直材（柱）は四周仕上げ `2(b+D)`、それ以外（梁）は三面仕上げ `b+2D`。
/// 断面の `width`/`depth` のいずれかが 0 以下の場合は 0（換算対象外）とする。
fn finish_perimeter(width: f64, depth: f64, is_vertical: bool) -> f64 {
    if width <= 0.0 || depth <= 0.0 {
        return 0.0;
    }
    if is_vertical {
        2.0 * (width + depth)
    } else {
        width + 2.0 * depth
    }
}

/// 自重 1 件分の重量とその帰属。
///
/// 地震用重量の節点集計（[`generate_stories_multi`]）と、長期応力解析用の
/// 自重(自動)荷重ケース（[`crate::self_weight`]）が同じ算定を共有するための
/// 中間表現。重量の算定規則（自重算定長・スラブ厚控除・仕上げ・ダンパー置換等）は
/// [`enumerate_self_weight`] に一元化する。
pub(crate) enum SelfWeightItem {
    /// 線材（柱・梁・ブレース）の自重（総量 [N]）。`elem_idx` は `model.elements` の添字。
    Line { elem_idx: usize, total: f64 },
    /// ダンパー装置＋支持部の重量（総量 [N]）。両端節点（`model.nodes` 添字）へ 1/2 ずつ。
    Damper { ni: usize, nj: usize, total: f64 },
    /// 壁・シェルの自重の頂点配分（`model.nodes` 添字 → [N]）。
    Panel { shares: Vec<(usize, f64)> },
    /// 二次部材（小梁・間柱）の自重（総量 [N]）。両端節点（`model.nodes` 添字）へ
    /// 1/2 ずつ。要素ではないため部材荷重にはならず、節点荷重（→ 主架構梁上の
    /// 節点なら CMQ 変換）として扱う。
    SecondaryLine { ni: usize, nj: usize, total: f64 },
}

/// モデル全要素の自重を列挙する（§柱梁自重・§壁自重・§ダンパー自重）。
///
/// - 線材（柱・梁・ブレース, `ElementKind::Beam`/`Brace`）: ρ·A·L·g。
///   §1.8: 自重算定長 L は、コンクリート材（`mat.fc` あり = RC/SRC）の水平材（梁）は
///   柱面間距離（`len` から両端の柱フェース距離を引いた、負にならない範囲）、鉛直材（柱）は
///   床上面から床上面まで（＝節点間距離。フェイス控除しない）、鋼材（S 梁・柱）は
///   節点間距離（RC/SRC 大梁は柱面間距離、
///   RC/SRC 柱は床上面から床上面、S 梁・柱は節点間距離）。
///   §1.9: RC/SRC 梁の断面積は梁上部のスラブ厚分 b·t を控除する
///   （w_c = γ·b(D−t)+…。スラブ重量は構造芯間の面積で別途計上されるため、
///   控除しないと梁幅×スラブ厚の体積が二重計上になる）。スラブが定義されて
///   いないモデル（純フレーム等）では控除しない。
///   §柱の長さ: コンクリート柱（鉛直材）で下端節点に別の柱（鉛直 Beam/Brace）が
///   下から接続していない場合、下端節点に取り付く梁（非鉛直 Beam）の最大せいを
///   自重算定長へ加算する（下階に柱がない場合、柱脚に取付く梁の最大せいの
///   長さを柱長さに付加する扱い）。
///   ギャップ対応: 鋼材のみ `load_cfg.effective_steel_factor()`（鉄骨重量割増率）を乗じ、
///   `load_cfg.extra_line_weight`（耐火被覆等の付加線重量 [N/mm]）・
///   `load_cfg.finish_area_weight`（仕上げ面重量 w_f、周長 φ から自動換算）が
///   あれば自重算定長を掛けて加算する。
/// - 壁・シェル（`ElementKind::Wall`/`Shell`, 節点数3以上）: ρ·t·(A−開口面積)·g＋開口重量
///   （§壁自重）を全頂点に等分配。三方スリット壁は最上位標高の頂点へ全量集中
///   （壁に三方スリットが指定されている場合、壁荷重は全て上部の大梁に伝達する扱い）。
///   §1.2: 壁の重量を階高の中央で上下階の節点に分配する扱いに対応
///   （矩形壁なら上下2節点ずつに1/4ずつ配分される）。
///   §壁自重: 4 節点の耐震壁は「周辺の柱梁の内法寸法」で面積を評価する
///   （[`wall_clear_area_factor`]。芯々面積に内法係数を乗じる。控除相手の
///   柱・梁が見つからない辺は控除なし＝芯々のまま保守側）。
/// - ダンパー（`load_cfg.dampers` に登録された Beam/Brace 要素）: 断面自重
///   （ρ·A·L·g）は使わず、装置重量＋支持部重量に置き換える（§ダンパー自重。
///   `device_weight=0` かつ `support_area>0` の場合は支持部のみが算入され、
///   自重を考慮しない部材に相当する）。
pub(crate) fn enumerate_self_weight(model: &Model, load_cfg: &LoadCfg) -> Vec<SelfWeightItem> {
    let mut items = Vec::new();
    // 柱脚探索（`has_column_below`/`max_depth`）・壁の辺→柱梁対応付け
    // （`wall_clear_area_factor`）を要素数分の走査から索引参照へ落とすため、
    // 主ループの前に 1 回だけ構築する（性能。走査順・判定条件は変更しない）。
    let node_adj = node_adjacency(model);
    let beam_pairs = beam_pair_map(model);
    // 柱フェース距離は `RigidZone::face_i/face_j` のキャッシュを当てにせず、
    // 幾何から直接求める。キャッシュは剛域の自動算定が走るまで未算定（None）で、
    // 以前はそれを 0 とみなして節点間距離で自重を算定していたため、準備計算の
    // 1 回目だけ固定荷重が 9.6% 過大になっていた（申し送り「実モデル統合テスト」4.1）。
    let faces = squid_n_core::face_distance::face_distances(model);
    for (elem_idx, elem) in model.elements.iter().enumerate() {
        // ダンパー自重（§ダンパー自重）: 対象部材は断面からの自重計算をスキップし、
        // 装置重量＋支持部断面積×(節点間距離−装置長さ)×鋼材単位体積重量で置き換える。
        if matches!(elem.kind, ElementKind::Beam | ElementKind::Brace { .. })
            && elem.nodes.len() >= 2
        {
            if let Some(damper) = load_cfg.dampers.iter().find(|d| d.elem == elem.id) {
                let ni = elem.nodes[0].index();
                let nj = elem.nodes[1].index();
                let len = dist3(model.nodes[ni].coord, model.nodes[nj].coord);
                let support_len = (len - damper.device_length).max(0.0);
                let w = damper.device_weight
                    + damper.support_area * support_len * steel_density_ton_mm3() * GRAVITY_MM_S2;
                items.push(SelfWeightItem::Damper { ni, nj, total: w });
                continue;
            }
        }

        // 材料は断面が持つ（`Model::element_material`）。
        let (Some(sec), Some(mat)) = (model.element_section(elem), model.element_material(elem))
        else {
            continue;
        };

        match elem.kind {
            ElementKind::Beam | ElementKind::Brace { .. } if elem.nodes.len() >= 2 => {
                let ni = elem.nodes[0].index();
                let nj = elem.nodes[1].index();
                let (ci, cj) = (model.nodes[ni].coord, model.nodes[nj].coord);
                let len = dist3(ci, cj);
                let is_vertical = is_vertical_pair(ci, cj);
                let is_concrete = mat.fc.is_some();
                // §1.8: 柱面間距離の控除は水平材（梁）のみ。鉛直材（柱）は
                // 床上面から床上面（＝節点間距離）で算定する。
                let mut eff_len = if is_concrete && !is_vertical {
                    let [fi, fj] = faces[elem_idx];
                    (len - fi - fj).max(0.0)
                } else {
                    len
                };

                // §柱の長さ: コンクリート造の柱で、下端節点から下に続く柱がない場合、
                // 下端節点に取り付く梁（非鉛直 Beam）の最大せいを自重算定長へ加算する。
                if is_concrete && is_vertical {
                    let bottom_local = if ci[2] <= cj[2] { 0 } else { 1 };
                    let bottom_id = elem.nodes[bottom_local];
                    let bottom_z = model.nodes[bottom_id.index()].coord[2];
                    let adj_at_bottom = node_adj
                        .get(&bottom_id)
                        .map(|v| v.as_slice())
                        .unwrap_or(&[]);
                    let has_column_below = adj_at_bottom.iter().any(|&idx| {
                        let e2 = &model.elements[idx];
                        e2.id != elem.id && {
                            let (a, b) = (
                                model.nodes[e2.nodes[0].index()].coord,
                                model.nodes[e2.nodes[1].index()].coord,
                            );
                            is_vertical_pair(a, b) && {
                                let other = if e2.nodes[0] == bottom_id { b } else { a };
                                other[2] < bottom_z - LEVEL_TOL_MM
                            }
                        }
                    });
                    if !has_column_below {
                        let max_depth = adj_at_bottom
                            .iter()
                            .filter_map(|&idx| {
                                let e2 = &model.elements[idx];
                                if e2.kind != ElementKind::Beam || e2.id == elem.id {
                                    return None;
                                }
                                let (a, b) = (
                                    model.nodes[e2.nodes[0].index()].coord,
                                    model.nodes[e2.nodes[1].index()].coord,
                                );
                                if is_vertical_pair(a, b) {
                                    None
                                } else {
                                    e2.section
                                        .and_then(|sid| model.sections.get(sid.index()))
                                        .map(|s| s.depth)
                                }
                            })
                            .fold(0.0_f64, f64::max);
                        eff_len += max_depth;
                    }
                }

                let factor = if is_concrete {
                    1.0
                } else {
                    load_cfg.effective_steel_factor()
                };
                // §1.9: RC/SRC 梁（水平材）はスラブ厚分の断面積 b·t を控除する
                // （w_c = γ·b(D−t)+…。スラブ重量が構造芯間の面積で別途計上される
                // ための二重計上防止）。スラブがないモデルでは控除しない。
                let self_weight_area = if is_concrete
                    && !is_vertical
                    && model.slab_thickness > 0.0
                    && !model.slabs.is_empty()
                {
                    (sec.area - sec.width * model.slab_thickness.min(sec.depth)).max(0.0)
                } else {
                    sec.area
                };
                let mut w = mat.density * self_weight_area * eff_len * GRAVITY_MM_S2 * factor;
                if let Some(&(_, lw)) = load_cfg
                    .extra_line_weight
                    .iter()
                    .find(|(id, _)| *id == elem.id)
                {
                    w += lw * eff_len;
                }
                // §仕上げ荷重の自動換算: w_f × 仕上げ周長 φ を自重算定長に乗じて加算する。
                if let Some(&(_, wf)) = load_cfg
                    .finish_area_weight
                    .iter()
                    .find(|(id, _)| *id == elem.id)
                {
                    let phi = finish_perimeter(sec.width, sec.depth, is_vertical);
                    w += wf * phi * eff_len;
                }

                items.push(SelfWeightItem::Line { elem_idx, total: w });
            }
            ElementKind::Wall | ElementKind::Shell if elem.nodes.len() >= 3 => {
                let Some(t) = sec.thickness else {
                    continue;
                };
                let pts: Vec<[f64; 3]> = elem
                    .nodes
                    .iter()
                    .map(|n| model.nodes[n.index()].coord)
                    .collect();
                // §壁自重: 耐震壁は周辺柱梁の内法寸法で面積を評価する。
                let area =
                    polygon_area_3d(&pts) * wall_clear_area_factor(model, elem, &pts, &beam_pairs);

                // §壁自重: 開口控除・開口重量。三方スリットは全量を最上位標高の頂点へ。
                let attr = model.wall_attrs.iter().find(|a| a.elem == elem.id);
                let opening_area = attr.map(|a| a.total_opening_area()).unwrap_or(0.0);
                let opening_weight = attr.map(|a| a.opening_weight).unwrap_or(0.0);
                let three_side_slit = attr.map(|a| a.three_side_slit).unwrap_or(false);
                let net_area = (area - opening_area).max(0.0);
                let w = (mat.density * t * net_area * GRAVITY_MM_S2 + opening_weight).max(0.0);

                let shares = if three_side_slit {
                    // 壁荷重は全て上部の節点（頂点のうち標高最大のもの。同率上位は等分）へ。
                    let max_z = pts.iter().map(|p| p[2]).fold(f64::MIN, f64::max);
                    let top_indices: Vec<usize> = pts
                        .iter()
                        .enumerate()
                        .filter(|(_, p)| (p[2] - max_z).abs() < LEVEL_TOL_MM)
                        .map(|(i, _)| i)
                        .collect();
                    let share = w / top_indices.len() as f64;
                    top_indices
                        .into_iter()
                        .map(|i| (elem.nodes[i].index(), share))
                        .collect()
                } else {
                    let share = w / pts.len() as f64;
                    elem.nodes.iter().map(|n| (n.index(), share)).collect()
                };
                items.push(SelfWeightItem::Panel { shares });
            }
            _ => {}
        }
    }

    // 二次部材（小梁・間柱）の自重: ρ·A·L·g（節点間距離。解析部材ではないため
    // 柱面間控除・スラブ厚控除は行わない簡易則）。鋼材は鉄骨重量割増率を乗じる。
    // 両端節点へ 1/2 ずつ帰属する（[`SelfWeightItem::SecondaryLine`]）。
    for sm in &model.secondary_members {
        let (Some(sec_id), Some(mat)) = (sm.section, model.secondary_material(sm)) else {
            continue;
        };
        let Some(sec) = model.sections.get(sec_id.index()) else {
            continue;
        };
        let ni = sm.nodes[0].index();
        let nj = sm.nodes[1].index();
        let (Some(n0), Some(n1)) = (model.nodes.get(ni), model.nodes.get(nj)) else {
            continue;
        };
        let len = dist3(n0.coord, n1.coord);
        let factor = if mat.fc.is_some() {
            1.0
        } else {
            load_cfg.effective_steel_factor()
        };
        let total = mat.density * sec.area * len * GRAVITY_MM_S2 * factor;
        if total > 0.0 {
            items.push(SelfWeightItem::SecondaryLine { ni, nj, total });
        }
    }

    items
}

/// 耐震壁の自重面積算定用の**内法係数**（芯々面積に乗じる係数、(0,1]）。
///
/// 耐震壁の重量は周辺の柱梁の内法寸法で計算する扱いに対応。
/// 対象は 4 節点の `ElementKind::Wall` のみ（シェル床・多角形壁は 1.0）。
/// 各辺を鉛直辺（側柱候補）・水平辺（上下梁候補）に分類し、辺の節点対に一致する
/// 線材（`ElementKind::Beam`）の断面寸法の半分を芯々寸法から控除する:
/// - 水平辺（上下梁）: 梁せい `sec.depth` の半分を高さから控除
/// - 鉛直辺（側柱）: 平面内の向きが特定できないため `min(width, depth)` の半分を
///   長さから控除（控除を小さくとる保守側の近似）
///
/// ハンチ・セットバック等で柱梁が斜めの場合の個別考慮は行わない。控除相手の
/// 部材が見つからない辺は控除なし（芯々のまま＝保守側）。
///
/// `beam_pairs` は `beam_pair_map` が返す節点対→`Beam` 要素添字の索引（呼び出し側
/// `enumerate_self_weight` が壁ループの前に 1 回だけ構築したものを使い回す）。
fn wall_clear_area_factor(
    model: &Model,
    elem: &ElementData,
    pts: &[[f64; 3]],
    beam_pairs: &HashMap<(NodeId, NodeId), usize>,
) -> f64 {
    if elem.kind != ElementKind::Wall || elem.nodes.len() != 4 || pts.len() != 4 {
        return 1.0;
    }
    let n = 4usize;
    let mut l_len = 0.0; // 水平辺（芯々長さ）の合計
    let mut l_cnt = 0u32;
    let mut h_len = 0.0; // 鉛直辺（芯々高さ）の合計
    let mut h_cnt = 0u32;
    let mut l_deduct = 0.0; // 側柱の半幅の和（長さ方向の控除）
    let mut h_deduct = 0.0; // 上下梁の半せいの和（高さ方向の控除）
    for i in 0..n {
        let (a, b) = (elem.nodes[i], elem.nodes[(i + 1) % n]);
        let (pa, pb) = (pts[i], pts[(i + 1) % n]);
        let dz = (pb[2] - pa[2]).abs();
        let dh = ((pb[0] - pa[0]).powi(2) + (pb[1] - pa[1]).powi(2)).sqrt();
        let len = (dz * dz + dh * dh).sqrt();
        if len <= 0.0 {
            continue;
        }
        // 辺の節点対に一致する線材（柱・梁）の断面。
        let member_sec = beam_pairs
            .get(&ordered_pair(a, b))
            .and_then(|&idx| model.elements[idx].section)
            .and_then(|sid| model.sections.get(sid.index()));
        if dz > dh {
            // 鉛直辺 = 側柱候補
            h_len += len;
            h_cnt += 1;
            if let Some(sec) = member_sec {
                l_deduct += sec.width.min(sec.depth).max(0.0) / 2.0;
            }
        } else {
            // 水平辺 = 上下梁候補
            l_len += len;
            l_cnt += 1;
            if let Some(sec) = member_sec {
                h_deduct += sec.depth.max(0.0) / 2.0;
            }
        }
    }
    if l_cnt == 0 || h_cnt == 0 {
        return 1.0;
    }
    let l = l_len / l_cnt as f64;
    let h = h_len / h_cnt as f64;
    if l <= 0.0 || h <= 0.0 {
        return 1.0;
    }
    let fl = ((l - l_deduct) / l).clamp(0.0, 1.0);
    let fh = ((h - h_deduct) / h).clamp(0.0, 1.0);
    (fl * fh).clamp(0.0, 1.0)
}
