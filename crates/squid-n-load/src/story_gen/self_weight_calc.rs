//! 自重（線材・壁・シェル・ダンパー）の列挙と算定。
//!
//! - [`SelfWeightItem`] — 自重 1 件分の設計重量・物理質量相当と帰属の中間表現
//! - [`enumerate_self_weight`] — モデル全要素の自重を列挙する
//! - [`steel_density_ton_mm3`] — 鋼材の物理質量密度 [ton/mm³]
//! - [`finish_perimeter`] — 仕上げ周長 φ
//! - [`wall_clear_area`] — 耐震壁の内法面積

use std::collections::HashMap;

use squid_n_core::section_shape::SectionShape;

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
/// 候補がある場合は断面を持つ要素を優先し、どちらも同じなら要素順で最初に
/// 見つかったものを採用する（断面未割当の仮の支持部材が実際の柱梁の内法控除を
/// 打ち消さないようにするため）。
fn beam_pair_map(model: &Model) -> HashMap<(NodeId, NodeId), usize> {
    let mut map = HashMap::new();
    for (idx, e) in model.elements.iter().enumerate() {
        if e.kind == ElementKind::Beam && e.nodes.len() >= 2 {
            let (a, b) = (e.nodes[0], e.nodes[e.nodes.len() - 1]);
            match map.entry(ordered_pair(a, b)) {
                std::collections::hash_map::Entry::Vacant(v) => {
                    v.insert(idx);
                }
                std::collections::hash_map::Entry::Occupied(mut o) => {
                    let prev = &model.elements[*o.get()];
                    if prev.section.is_none() && e.section.is_some() {
                        o.insert(idx);
                    }
                }
            }
        }
    }
    map
}

/// 鋼材の物理質量密度 [ton/mm³]（質量行列・動的解析用）。ダンパー支持部の
/// 質量相当重量の算定に用いる。設計用単位体積重量からは導出しない。
pub(crate) fn steel_density_ton_mm3() -> f64 {
    squid_n_core::units::STEEL_MASS_DENSITY_TON_MM3
}

/// 鋼材の設計用単位体積重量 [N/mm³]（DL・地震用重量用）。ダンパー支持部の
/// 設計重量の算定に用いる。
pub(crate) fn steel_design_unit_weight_n_per_mm3() -> f64 {
    squid_n_core::units::to_internal::unit_weight_kn_per_m3(
        squid_n_core::units::STEEL_UNIT_WEIGHT_KN_M3,
    )
}

/// 解析の質量行列が線材へ与える単位長さ当たり質量 [t/mm]。
/// 質量行列の組み立てと同じ [`Model::element_mass_properties`] から求める。
fn analysis_mass_per_length(model: &Model, elem: &ElementData) -> f64 {
    model
        .element_mass_properties(elem)
        .map_or(0.0, |properties| properties.mass_per_length)
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
    /// 線材（柱・梁・ブレース）の自重。`elem_idx` は `model.elements` の添字。
    ///
    /// - `load` は通常部の設計重量 [N]（DL・地震用重量）。鉄骨重量割増 `factor`・
    ///   付加線重量・仕上げを含む。
    /// - `extra_bottom_load` は下端節点だけへ加算する設計重量 [N]（通常部の外側。
    ///   柱以外・S 柱・下階柱ありは 0）。
    /// - `mass_equiv` は通常部の物理質量相当の重量 [N]。躯体分だけを設計単位体積重量から
    ///   物理密度（×g）へ置換し、付加重量（仕上げ・付加線重量・割増増分）はそのまま残す。
    /// - `extra_bottom_mass_equiv` は `extra_bottom_load` に対応する物理質量相当 [N]。
    /// - `matrix_mass_equiv` は通常部のうち解析の質量行列（部材密度質量）が受け持つ分 [N]。
    ///   付加重量・下端付加分は質量行列に対応物がないため含めない。
    /// - `is_column` は 2 節点の鉛直 `ElementKind::Beam`（ブレースは false）。
    Line {
        elem_idx: usize,
        load: f64,
        mass_equiv: f64,
        matrix_mass_equiv: f64,
        extra_bottom_load: f64,
        extra_bottom_mass_equiv: f64,
        is_column: bool,
    },
    /// ダンパー装置＋支持部の重量。両端節点（`model.nodes` 添字）へ 1/2 ずつ。
    /// `load` は設計重量、`mass_equiv` は物理質量相当（装置重量はそのまま）。
    Damper {
        ni: usize,
        nj: usize,
        load: f64,
        mass_equiv: f64,
    },
    /// 壁・シェルの自重の頂点配分（`model.nodes` 添字 → [N]）。
    ///
    /// 質量方式ごとに基準が異なるため 3 値を持つ。
    /// `load_shares` は設計重量（設計躯体 ＋ 仕上げ・増打ち ＋ 開口重量）、
    /// `mass_equiv_shares` は物理密度の躯体 ＋ 仕上げ・増打ち ＋ 開口重量、
    /// `matrix_shares` は物理密度の躯体 ＋ 開口重量（解析の質量行列が受け持つ分）。
    /// 仕上げ・増打ちは質量行列に対応物がないため、`mass_equiv_shares` と
    /// `matrix_shares` の差として補正質点に残す。
    Panel {
        load_shares: Vec<(usize, f64)>,
        mass_equiv_shares: Vec<(usize, f64)>,
        matrix_shares: Vec<(usize, f64)>,
    },
}

/// モデル全要素の自重を列挙する（§柱梁自重・§壁自重・§ダンパー自重）。
///
/// - 線材（柱・梁・ブレース, `ElementKind::Beam`/`Brace`）: 設計重量（設計単位体積
///   重量×A×L。付加線重量・仕上げを含む）と物理質量相当（物理密度×A×L×g に付加重量を
///   足したもの）を別々に持つ。
///   §1.8: 自重算定長 L は、コンクリート材（`mat.fc` あり = RC/SRC）の水平材（梁）は
///   柱面間距離（`len` から両端の柱フェース距離を引いた、負にならない範囲）、鉛直材（柱）は
///   床上面から床上面まで（＝節点間距離。フェイス控除しない）、鋼材（S 梁・柱）は
///   節点間距離（RC/SRC 大梁は柱面間距離、
///   RC/SRC 柱は床上面から床上面、S 梁・柱は節点間距離）。
///   §1.9: RC/SRC 梁の断面積は梁上部のスラブ厚分 b·t を控除する
///   （w_c = γ·b(D−t)+…。スラブ重量は構造芯間の面積で別途計上されるため、
///   控除しないと梁幅×スラブ厚の体積が二重計上になる）。スラブが定義されて
///   いないモデル（純フレーム等）では控除しない。
///   §柱の長さ: コンクリート柱（2 節点の鉛直 `ElementKind::Beam`）で下端へ別の柱
///   （同じく 2 節点の鉛直 `ElementKind::Beam`）が接続しない場合、下端節点に取り付く
///   非鉛直 Beam の最大せい [mm] に相当する重量を
///   追加自重として下端節点だけへ加算する（通常自重は上下へ 1/2 ずつ。柱以外・S 柱・
///   下階柱ありは 0。ブレースは柱とみなさない）。総重量は `w·(L+Dmax)` で保存する。
///   ギャップ対応: 鋼材のみ `load_cfg.effective_steel_factor()`（鉄骨重量割増率）を乗じ、
///   `load_cfg.extra_line_weight`（耐火被覆等の付加線重量 [N/mm]）・
///   `load_cfg.finish_area_weight`（仕上げ面重量 w_f、周長 φ から自動換算）が
///   あれば自重算定長を掛けて加算する。
/// - 壁・シェル（`ElementKind::Wall`/`Shell`, 節点数3以上）: 設計重量（設計躯体＋
///   仕上げ・増打ち＋開口重量）・物理質量相当（物理密度の躯体＋仕上げ・増打ち＋
///   開口重量）・質量行列が受け持つ分（物理密度の躯体＋開口重量）を別々に
///   全頂点へ等分配（§壁自重）。要素になる壁版は上下の梁と一体なので、行き先を
///   上下どちらかへ寄せる扱いはしない（上下いずれかの梁との縁切りは壁版の形が表し、
///   取り付く壁版として `crate::wall_attached` が受け持つ）。
///   §1.2: 壁の重量を階高の中央で上下階の節点に分配する扱いに対応
///   （矩形壁なら上下2節点ずつに1/4ずつ配分される）。
///   §壁自重: 4 節点の耐震壁は「周辺の柱梁の内法寸法」で面積を評価する
///   （[`wall_clear_area`]。芯々面積に内法係数を乗じる。控除相手の
///   柱・梁が見つからない辺は控除なし＝芯々のまま保守側）。
/// - ダンパー（`load_cfg.dampers` に登録された Beam/Brace 要素）: 断面自重は使わず、
///   設計重量は装置重量＋支持部×設計単位体積重量、物理質量相当は装置重量＋支持部×
///   物理密度×g に置き換える（§ダンパー自重。装置重量は両者で同値。
///   `device_weight=0` かつ `support_area>0` の場合は支持部のみが算入され、
///   自重を考慮しない部材に相当する）。
///
/// 壁の解析要素（`ElementKind::Wall`）は入力の正である `model` には存在しない
/// 生成物（D5）のため、本関数は内部で壁展開モデル
/// （[`crate::wall_expand::expand_wall_elements`]）を組み立てて壁を検出する
/// （呼び出し元に展開を要求しない。忘れると壁の自重が静かに消えるため）。
/// 返す [`SelfWeightItem::Line`] の `elem_idx` は柱・梁・ブレースのみを指し、
/// 生成要素は展開モデルの末尾へ追加されるだけなので、`model.elements` に対する
/// 添字としてもそのまま有効。壁の開口は、壁展開モデルに合成される
/// `wall_attrs`（[`crate::wall_expand::expand_wall_elements`] が壁版から複製する。
/// モジュール doc 参照）から今までどおり読む。
pub(crate) fn enumerate_self_weight(model: &Model, load_cfg: &LoadCfg) -> Vec<SelfWeightItem> {
    let (expanded, _wall_index, _wall_expand_report) =
        crate::wall_expand::expand_wall_elements(model);
    let model = &expanded;
    let mut items = Vec::new();
    let node_adj = node_adjacency(model);
    let beam_pairs = beam_pair_map(model);
    let faces = squid_n_core::face_distance::face_distances(model);
    for (elem_idx, elem) in model.elements.iter().enumerate() {
        if matches!(elem.kind, ElementKind::Beam | ElementKind::Brace { .. })
            && elem.nodes.len() >= 2
        {
            if let Some(damper) = load_cfg.dampers.iter().find(|d| d.elem == elem.id) {
                let ni = elem.nodes[0].index();
                let nj = elem.nodes[1].index();
                let len = dist3(model.nodes[ni].coord, model.nodes[nj].coord);
                let support_len = (len - damper.device_length).max(0.0);
                let load = damper.device_weight
                    + damper.support_area * support_len * steel_design_unit_weight_n_per_mm3();
                let mass_equiv = damper.device_weight
                    + damper.support_area * support_len * steel_density_ton_mm3() * GRAVITY_MM_S2;
                items.push(SelfWeightItem::Damper {
                    ni,
                    nj,
                    load,
                    mass_equiv,
                });
                continue;
            }
        }

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
                let is_column =
                    elem.kind == ElementKind::Beam && elem.nodes.len() == 2 && is_vertical;
                let is_concrete = mat.fc.is_some();
                let eff_len = if is_concrete && !is_vertical {
                    let [fi, fj] = faces[elem_idx];
                    (len - fi - fj).max(0.0)
                } else {
                    len
                };

                let mut max_depth = 0.0;
                if is_column && is_concrete {
                    let bottom_local = if ci[2] <= cj[2] { 0 } else { 1 };
                    let bottom_id = elem.nodes[bottom_local];
                    let bottom_z = model.nodes[bottom_id.index()].coord[2];
                    let adj_at_bottom = node_adj
                        .get(&bottom_id)
                        .map(|v| v.as_slice())
                        .unwrap_or(&[]);
                    let has_column_below = adj_at_bottom.iter().any(|&idx| {
                        let e2 = &model.elements[idx];
                        e2.id != elem.id && e2.kind == ElementKind::Beam && e2.nodes.len() == 2 && {
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
                        max_depth = adj_at_bottom
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
                    }
                }

                let factor = if is_concrete {
                    1.0
                } else {
                    load_cfg.effective_steel_factor()
                };
                let self_weight_area = if is_concrete
                    && !is_vertical
                    && model.slab_thickness > 0.0
                    && !model.floor_regions.is_empty()
                {
                    (sec.area - sec.width * model.slab_thickness.min(sec.depth)).max(0.0)
                } else {
                    sec.area
                };
                let mut design_per_length =
                    mat.design_unit_weight_n_per_mm3() * self_weight_area * factor;
                let mut physical_per_length =
                    mat.density * self_weight_area * GRAVITY_MM_S2 * factor;
                if let Some(&(_, lw)) = load_cfg
                    .extra_line_weight
                    .iter()
                    .find(|(id, _)| *id == elem.id)
                {
                    design_per_length += lw;
                    physical_per_length += lw;
                }
                if let Some(&(_, wf)) = load_cfg
                    .finish_area_weight
                    .iter()
                    .find(|(id, _)| *id == elem.id)
                {
                    let phi = finish_perimeter(sec.width, sec.depth, is_vertical);
                    design_per_length += wf * phi;
                    physical_per_length += wf * phi;
                }
                let load = design_per_length * eff_len;
                let body_design = mat.design_unit_weight_n_per_mm3() * self_weight_area * eff_len;
                let matrix_mass_equiv = analysis_mass_per_length(model, elem) * len * GRAVITY_MM_S2;
                let is_cft = matches!(
                    sec.shape,
                    Some(SectionShape::CftBox { .. } | SectionShape::CftPipe { .. })
                );
                let body_physical = if is_cft {
                    matrix_mass_equiv
                } else {
                    mat.density * self_weight_area * eff_len * GRAVITY_MM_S2
                };
                let mass_equiv = load - body_design + body_physical;

                let extra_bottom_load = design_per_length * max_depth;
                let extra_bottom_mass_equiv = physical_per_length * max_depth;
                items.push(SelfWeightItem::Line {
                    elem_idx,
                    load,
                    mass_equiv,
                    matrix_mass_equiv,
                    extra_bottom_load,
                    extra_bottom_mass_equiv,
                    is_column,
                });
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
                let area = wall_clear_area(model, elem, &pts, &beam_pairs);

                let attr = model.wall_attrs.iter().find(|a| a.elem == elem.id);
                let opening_area = attr.map(|a| a.total_opening_area()).unwrap_or(0.0);
                let opening_weight = attr.map(|a| a.opening_weight).unwrap_or(0.0);
                let net_area = (area - opening_area).max(0.0);
                let finish = attr.map(|a| a.finish_intensity).unwrap_or(0.0);
                let w_load = ((mat.design_unit_weight_n_per_mm3() * t + finish) * net_area
                    + opening_weight)
                    .max(0.0);

                let slit = attr.map(|a| a.slit).unwrap_or_default();
                let load_shares = wall_corner_shares(elem, &pts, w_load, slit);
                let w_matrix =
                    (mat.density * t * GRAVITY_MM_S2 * net_area + opening_weight).max(0.0);
                let matrix_shares = wall_corner_shares(elem, &pts, w_matrix, slit);
                let w_mass_equiv = ((mat.density * t * GRAVITY_MM_S2 + finish) * net_area
                    + opening_weight)
                    .max(0.0);
                let mass_equiv_shares = wall_corner_shares(elem, &pts, w_mass_equiv, slit);
                items.push(SelfWeightItem::Panel {
                    load_shares,
                    mass_equiv_shares,
                    matrix_shares,
                });
            }
            _ => {}
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
/// 壁エレメントの自重を頂点へ配る（§壁自重）。
///
/// 縁が切れていない梁際の辺へ伝える。上下とも一体なら四隅へ等分し、片側だけ切れて
/// いれば反対側の 2 節点へ全量を寄せる。上下とも切れた壁は四隅へ等分して重量を落とさず、
/// 解析前チェックのエラーに委ねる。
///
/// 上下の別は標高で決める。同じ標高の節点はまとめて 1 つの辺として扱う
/// （台形壁で上辺の 2 節点の標高がわずかに異なる場合も、[`LEVEL_TOL_MM`] 以内なら
/// 同じ辺とみなす）。
fn wall_corner_shares(
    elem: &squid_n_core::model::ElementData,
    pts: &[[f64; 3]],
    w: f64,
    slit: squid_n_core::model::WallSlit,
) -> Vec<(usize, f64)> {
    let equal = |idx: Vec<usize>| -> Vec<(usize, f64)> {
        let share = w / idx.len() as f64;
        idx.into_iter()
            .map(|i| (elem.nodes[i].index(), share))
            .collect()
    };
    let all: Vec<usize> = (0..pts.len()).collect();
    let (bottom_slit, top_slit) = (slit.beam_face[0], slit.beam_face[1]);
    if bottom_slit == top_slit {
        return equal(all);
    }
    let level = |take_max: bool| -> Vec<usize> {
        let z =
            pts.iter()
                .map(|p| p[2])
                .fold(if take_max { f64::MIN } else { f64::MAX }, |acc, v| {
                    if take_max {
                        acc.max(v)
                    } else {
                        acc.min(v)
                    }
                });
        (0..pts.len())
            .filter(|&i| (pts[i][2] - z).abs() < LEVEL_TOL_MM)
            .collect()
    };
    let idx = level(bottom_slit);
    if idx.is_empty() {
        return equal(all);
    }
    equal(idx)
}

fn wall_clear_area(
    model: &Model,
    elem: &ElementData,
    pts: &[[f64; 3]],
    beam_pairs: &HashMap<(NodeId, NodeId), usize>,
) -> f64 {
    if elem.kind != ElementKind::Wall || elem.nodes.len() != 4 {
        return polygon_area_3d(pts);
    }
    let Some(geom) = squid_n_core::model::wall_element_geometry(elem, model) else {
        return 0.0;
    };
    let boundary = [geom.bottom[0], geom.bottom[1], geom.top[1], geom.top[0]];
    let points = boundary.map(|node| model.nodes[node.index()].coord);
    let dimensions = std::array::from_fn(|i| {
        beam_pairs
            .get(&ordered_pair(boundary[i], boundary[(i + 1) % 4]))
            .and_then(|&idx| model.element_section(&model.elements[idx]))
            .map(|sec| [sec.width, sec.depth])
    });
    polygon_area_3d(&points) * squid_n_core::model::wall_clear_area_factor(&points, &dimensions)
}
