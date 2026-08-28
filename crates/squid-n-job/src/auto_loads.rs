//! 重力（DL/LL）・地震（EX/EY）荷重ケースの自動生成（モデルは書き換えない）。
//!
//! GUI は [`compute_auto_load_cases`] で内容を求め、undo 付きの
//! `SyncSlabLoadsToCase` で書き込む。MCP 等のエフェメラルな作業コピーは
//! [`apply_auto_load_cases`] で直接反映する。

use std::collections::HashMap;

use squid_n_core::ids::{ElemId, FloorRegionId, LoadCaseId, NodeId};
use squid_n_core::model::{
    ElementKind, FloorRegion, LoadCase, LoadCaseKind, LoadPurpose, MemberLoad, MemberLoadKind,
    Model, NodalLoad, Slab, DL_CASE_NAME, EX_CASE_NAME, EY_CASE_NAME, LL_FRAME_CASE_NAME,
    LL_SEISMIC_CASE_NAME,
};
use squid_n_load::floor::{self, BeamLoad, LoadShape, LoadTarget};
use squid_n_load::secondary::{beam_span_position, resolve_nodal_to_primary, SPAN_TOL_MM};
use squid_n_solver::analysis::{self, AiMode, SeismicDir};

use crate::settings::AnalysisSettings;

/// 自動同期対象の 1 荷重ケース分の内容（書き込み前）。
pub struct AutoLoadCaseContent {
    pub name: &'static str,
    pub kind: LoadCaseKind,
    pub nodal: Vec<NodalLoad>,
    pub member: Vec<MemberLoad>,
}

/// [`compute_auto_load_cases`] の戻り値。
pub struct AutoLoadComputeResult {
    pub cases: Vec<AutoLoadCaseContent>,
    pub notices: Vec<String>,
}

/// 節点対の順不同キー（`(min,max)`）。
fn beam_key(a: NodeId, b: NodeId) -> (NodeId, NodeId) {
    if a.0 <= b.0 {
        (a, b)
    } else {
        (b, a)
    }
}

/// 節点対 (min,max) → 実 `Beam`（2 節点）要素の `ElemId` を引く索引。
pub fn beam_elem_map(model: &Model) -> HashMap<(NodeId, NodeId), ElemId> {
    let mut map = HashMap::new();
    for e in &model.elements {
        if e.kind == ElementKind::Beam && e.nodes.len() == 2 {
            map.entry(beam_key(e.nodes[0], e.nodes[1])).or_insert(e.id);
        }
    }
    map
}

/// 交差小梁スラブについて、床格子サブモデルの支点反力を大梁接続点への集中荷重として返す。
pub fn slab_grillage_node_reactions(
    model: &Model,
    slab: &FloorRegion,
    w: f64,
    beam_map: &HashMap<(NodeId, NodeId), ElemId>,
) -> Option<Vec<(NodeId, f64)>> {
    if !floor::uses_joist_distribution(model, slab) {
        return None;
    }
    if slab
        .joist_lines()
        .iter()
        .any(|j| beam_map.contains_key(&beam_key(j.support[0], j.support[1])))
    {
        return None;
    }
    let g = crate::floor_grillage::build_slab_grillage(model, slab, w)?;
    let sol = crate::floor_grillage::solve_grillage(&g.model, LoadCaseId(0)).ok()?;
    Some(
        g.support_origin
            .iter()
            .map(|(n, id)| (*id, sol.reactions[*n][2]))
            .collect(),
    )
}

fn slab_grillage_unit_reactions(
    model: &Model,
    beam_map: &HashMap<(NodeId, NodeId), ElemId>,
) -> HashMap<FloorRegionId, Vec<(NodeId, f64)>> {
    let mut out = HashMap::new();
    for slab in &model.floor_regions {
        if let Some(reactions) = slab_grillage_node_reactions(model, slab, 1.0, beam_map) {
            out.insert(slab.id, reactions);
        }
    }
    out
}

/// `distribute_region`/`distribute_slab_resolved` が返す `BeamLoad`（`Node`/`Span` のみ）を
/// 実部材へ解決して `out` へ積む。`grillage_reactions` が `Some` の間は `Node` を素通しせず
/// 格子反力（後段で別途積む）に譲る（二重計上を防ぐ）。
fn push_resolved_loads(
    loads: Vec<BeamLoad>,
    grillage_reactions: &Option<Vec<(NodeId, f64)>>,
    beam_map: &HashMap<(NodeId, NodeId), ElemId>,
    out: &mut Vec<BeamLoad>,
) {
    let find_beam =
        |n0: NodeId, n1: NodeId| -> Option<ElemId> { beam_map.get(&beam_key(n0, n1)).copied() };
    for mut bl in loads {
        match bl.target {
            LoadTarget::Node(_) => {
                if grillage_reactions.is_none() {
                    out.push(bl);
                }
            }
            LoadTarget::Edge(_) => {
                // `distribute_region`/`distribute_slab_resolved` は Edge を残さず Span へ解決済み。
                continue;
            }
            LoadTarget::Span {
                nodes: [n0, n1], ..
            } => {
                if let Some(elem) = find_beam(n0, n1) {
                    bl.elem = elem;
                }
                out.push(bl);
            }
        }
    }
}

/// 各床領域について面荷重強度 `w_of(slab)` を境界へ分配し、`BeamLoad` 列を返す。
/// `w_of` は床板ごとの面荷重強度 [N/mm²]（DL・LL を分けるため床板単位で渡す）。
///
/// 床領域に帰属する床板（`region.slab_ids`）は床領域ごとにまとめて分配し、
/// どの床領域からも参照されない床板（片持ち・バルコニー・出隅、または帰属先が
/// 見つからない浮き床板）は個別に分配する（荷重を取りこぼさないため）。
pub fn slab_beam_loads_with(
    model: &Model,
    w_of: impl Fn(&Slab) -> f64,
    unit_reactions: &HashMap<FloorRegionId, Vec<(NodeId, f64)>>,
    beam_map: &HashMap<(NodeId, NodeId), ElemId>,
) -> Vec<BeamLoad> {
    let mut beam_loads = Vec::new();
    let mut referenced = std::collections::HashSet::new();
    for region in &model.floor_regions {
        referenced.extend(region.slab_ids.iter().copied());
        // 格子反力は代表床板（`region.slab_ids` の先頭）の強度で判定する
        // （`uses_joist_distribution` が要求する形。§floor::mod ドキュメント参照）。
        let w = region
            .slab_ids
            .first()
            .and_then(|&id| model.slab(id))
            .map(&w_of)
            .unwrap_or(0.0);
        let grillage_reactions: Option<Vec<(NodeId, f64)>> = unit_reactions
            .get(&region.id)
            .map(|rs| rs.iter().map(|(node, r)| (*node, r * w)).collect());
        push_resolved_loads(
            floor::distribute_region(model, region, &w_of),
            &grillage_reactions,
            beam_map,
            &mut beam_loads,
        );
        if let Some(reactions) = grillage_reactions {
            for (node, r) in reactions {
                if r.abs() <= 1e-9 {
                    continue;
                }
                beam_loads.push(BeamLoad {
                    elem: ElemId(u32::MAX),
                    target: LoadTarget::Node(node),
                    shape: LoadShape::Point { p: r, x: 0.0 },
                    cmq: floor::Cmq {
                        c_i: 0.0,
                        c_j: 0.0,
                        q_i: r,
                        q_j: 0.0,
                    },
                });
            }
        }
    }
    for slab in &model.slabs {
        if referenced.contains(&slab.id) {
            continue;
        }
        let w = w_of(slab);
        push_resolved_loads(
            floor::distribute_slab_resolved(model, slab, w),
            &None,
            beam_map,
            &mut beam_loads,
        );
    }
    beam_loads
}

/// `BeamLoad` 列を荷重ケースへ書き込める `NodalLoad`/`MemberLoad` へ変換する。
pub fn slab_load_case_content(
    model: &Model,
    beam_loads: &[BeamLoad],
) -> (Vec<NodalLoad>, Vec<MemberLoad>) {
    const DIR: [f64; 3] = [0.0, 0.0, -1.0];
    let mut nodal = Vec::new();
    let mut member = Vec::new();

    fn push_dist(member: &mut Vec<MemberLoad>, elem: ElemId, a: f64, b: f64, w1: f64, w2: f64) {
        if b - a <= 1e-9 {
            return;
        }
        member.push(MemberLoad::auto(
            elem,
            DIR,
            MemberLoadKind::Distributed { a, b, w1, w2 },
        ));
    }

    fn emit_shape(
        member: &mut Vec<MemberLoad>,
        elem: ElemId,
        a0: f64,
        len_e: f64,
        flip: bool,
        shape: &LoadShape,
    ) {
        match *shape {
            LoadShape::Uniform { w } => push_dist(member, elem, a0, a0 + len_e, w, w),
            LoadShape::Linear { w_i, w_j } => {
                let (w1, w2) = if flip { (w_j, w_i) } else { (w_i, w_j) };
                push_dist(member, elem, a0, a0 + len_e, w1, w2);
            }
            LoadShape::Triangle { w0 } => {
                let mid = len_e / 2.0;
                push_dist(member, elem, a0, a0 + mid, 0.0, w0);
                push_dist(member, elem, a0 + mid, a0 + len_e, w0, 0.0);
            }
            LoadShape::Trapezoid { w0, a, b } => {
                push_dist(member, elem, a0, a0 + a, 0.0, w0);
                push_dist(member, elem, a0 + a, a0 + a + b, w0, w0);
                push_dist(member, elem, a0 + a + b, a0 + len_e, w0, 0.0);
            }
            LoadShape::Point { p, x } => {
                let xx = if flip { len_e - x } else { x };
                member.push(MemberLoad::auto(
                    elem,
                    DIR,
                    MemberLoadKind::Point { a: a0 + xx, p },
                ));
            }
        }
    }

    /// 線分に沿った荷重強度 [N/mm]（線分始点からの距離 `x`）。`Point` は強度を持たない。
    fn line_intensity(shape: &LoadShape, len: f64, x: f64) -> f64 {
        match *shape {
            LoadShape::Uniform { w } => w,
            LoadShape::Linear { w_i, w_j } => {
                if len <= 1e-9 {
                    w_i
                } else {
                    w_i + (w_j - w_i) * (x / len)
                }
            }
            LoadShape::Triangle { w0 } => {
                let mid = len / 2.0;
                if x <= mid {
                    if mid <= 1e-9 {
                        0.0
                    } else {
                        w0 * (x / mid)
                    }
                } else if len - mid <= 1e-9 {
                    0.0
                } else {
                    w0 * ((len - x) / (len - mid))
                }
            }
            LoadShape::Trapezoid { w0, a, b } => {
                if x < a {
                    if a <= 1e-9 {
                        w0
                    } else {
                        w0 * (x / a)
                    }
                } else if x <= a + b {
                    w0
                } else {
                    let tail = len - (a + b);
                    if tail <= 1e-9 {
                        w0
                    } else {
                        w0 * ((len - x) / tail)
                    }
                }
            }
            LoadShape::Point { .. } => 0.0,
        }
    }

    /// 荷重強度が折れる位置（線分始点からの距離）。区間をここで割ると各片が線形になる。
    fn shape_breakpoints(shape: &LoadShape, len: f64) -> Vec<f64> {
        match *shape {
            LoadShape::Uniform { .. } | LoadShape::Linear { .. } => Vec::new(),
            LoadShape::Triangle { .. } => vec![len / 2.0],
            LoadShape::Trapezoid { a, b, .. } => vec![a, a + b],
            LoadShape::Point { x, .. } => vec![x],
        }
    }

    /// 線分 `p0`→`p1` に載る荷重を、それを覆う大梁へ割り付ける。
    ///
    /// 取付き線が張る大梁が中間節点で分割されていても、幾何で覆いを求めるため外れない。
    /// 線分の全長を覆えなかった場合は `false` を返し、呼び出し側の従来経路（両端節点への
    /// 振り分け）へ委ねる（覆えないぶんの荷重を落とさないため）。
    fn emit_along_segment(
        model: &Model,
        member: &mut Vec<MemberLoad>,
        p0: [f64; 3],
        p1: [f64; 3],
        shape: &LoadShape,
    ) -> bool {
        let len = {
            let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
            (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
        };
        if len <= 1e-9 {
            return false;
        }
        let cover = squid_n_load::secondary::beams_along_segment(model, p0, p1, SPAN_TOL_MM);
        if cover.is_empty() {
            return false;
        }
        // 覆いが全長に届かない（隙間がある・端が余る）場合は割り付けない。
        // 被覆区間どうしが重なっている場合（重複部材・部分的に重なる梁）も割り付けない。
        // 単純な合計で判定すると、重なりのぶんだけ長さが水増しされ、隙間があっても
        // 「全長を覆えた」と誤判定する（前半へ二重に載り、後半が無荷重になる）。
        let mut union = 0.0_f64;
        let mut reach = 0.0_f64;
        let mut sum = 0.0_f64;
        for c in &cover {
            sum += c.seg[1] - c.seg[0];
            union += (c.seg[1] - c.seg[0].max(reach)).max(0.0);
            reach = reach.max(c.seg[1]);
        }
        if (len - union).abs() > SPAN_TOL_MM || (sum - union).abs() > SPAN_TOL_MM {
            return false;
        }

        let mut breaks = shape_breakpoints(shape, len);
        breaks.retain(|x| *x > 0.0 && *x < len);
        // 集中荷重が被覆区間の境目にちょうど載る場合、両側の区間が同じ位置を含むため、
        // 1 度載せたら以降の区間では載せない（二重計上を防ぐ）。
        let mut point_placed = false;
        for c in &cover {
            // 覆い区間を折れ点で割り、各片を線形分布として梁へ載せる。
            let mut cuts = vec![c.seg[0], c.seg[1]];
            cuts.extend(
                breaks
                    .iter()
                    .copied()
                    .filter(|x| *x > c.seg[0] && *x < c.seg[1]),
            );
            cuts.sort_by(f64::total_cmp);
            let span = c.seg[1] - c.seg[0];
            let to_elem = |x: f64| -> f64 {
                if span <= 1e-9 {
                    c.elem_pos[0]
                } else {
                    c.elem_pos[0] + (x - c.seg[0]) / span * (c.elem_pos[1] - c.elem_pos[0])
                }
            };
            if let LoadShape::Point { p, x } = *shape {
                if !point_placed && x >= c.seg[0] && x <= c.seg[1] {
                    member.push(MemberLoad::auto(
                        c.elem,
                        DIR,
                        MemberLoadKind::Point { a: to_elem(x), p },
                    ));
                    point_placed = true;
                }
                continue;
            }
            for w in cuts.windows(2) {
                let (x0, x1) = (w[0], w[1]);
                if x1 - x0 <= 1e-9 {
                    continue;
                }
                let (mut a, mut b) = (to_elem(x0), to_elem(x1));
                let (mut w1, mut w2) = (
                    line_intensity(shape, len, x0),
                    line_intensity(shape, len, x1),
                );
                if a > b {
                    std::mem::swap(&mut a, &mut b);
                    std::mem::swap(&mut w1, &mut w2);
                }
                push_dist(member, c.elem, a, b, w1, w2);
            }
        }
        true
    }

    fn simple_reactions(shape: &LoadShape, len: f64) -> (f64, f64) {
        match *shape {
            LoadShape::Uniform { w } => {
                let total = w * len;
                (total / 2.0, total / 2.0)
            }
            LoadShape::Linear { w_i, w_j } => {
                // 単純梁反力。台形の面積重心按分と一致する: R_i = L(2w_i+w_j)/6。
                (len * (2.0 * w_i + w_j) / 6.0, len * (w_i + 2.0 * w_j) / 6.0)
            }
            LoadShape::Triangle { w0 } => {
                let total = w0 * len / 2.0;
                (total / 2.0, total / 2.0)
            }
            LoadShape::Trapezoid { w0, a, b } => {
                let total = w0 * (a + b);
                (total / 2.0, total / 2.0)
            }
            LoadShape::Point { p, x } => {
                if len <= 1e-9 {
                    (p / 2.0, p / 2.0)
                } else {
                    let t = (x / len).clamp(0.0, 1.0);
                    (p * (1.0 - t), p * t)
                }
            }
        }
    }

    for bl in beam_loads {
        match bl.target {
            LoadTarget::Node(n) => {
                let LoadShape::Point { p, .. } = bl.shape else {
                    continue;
                };
                nodal.push(NodalLoad::auto(n, [0.0, 0.0, -p, 0.0, 0.0, 0.0]));
            }
            LoadTarget::Edge(_) => {
                let Some(elem) = model.element(bl.elem) else {
                    continue;
                };
                let l = model.member_length(elem);
                if l <= 1e-9 {
                    continue;
                }
                emit_shape(&mut member, elem.id, 0.0, l, false, &bl.shape);
            }
            LoadTarget::Span { nodes: [n0, n1], t } => {
                // `t == [0, 1]`（既定。荷重が nodes 間の全長を覆う）は従来どおり
                // `bl.elem`（`push_resolved_loads` が直接ヒットさせた要素）への
                // 高速経路を使う。部分区間（取付き線の一部だけに載る取り付き領域）は
                // `bl.elem` の局所 x 軸の向きを検証していないため、この経路には乗せず
                // 下の座標ベースの幾何解決へ回す（件数が少なく性能への影響もない）。
                let full = (t[0] - 0.0).abs() <= 1e-9 && (t[1] - 1.0).abs() <= 1e-9;
                if full {
                    if let Some(elem) = model.element(bl.elem) {
                        let l = model.member_length(elem);
                        if l > 1e-9 {
                            emit_shape(&mut member, elem.id, 0.0, l, false, &bl.shape);
                        }
                        continue;
                    }
                }
                let (Some(node0), Some(node1)) =
                    (model.nodes.get(n0.index()), model.nodes.get(n1.index()))
                else {
                    continue;
                };
                // 荷重が実際に載る区間の端点。全長（t=[0,1]）なら nodes そのもの、
                // 部分区間なら nodes 間を t で線形補間した点（実節点ではない）。
                let lerp = |c0: [f64; 3], c1: [f64; 3], f: f64| {
                    [
                        c0[0] + (c1[0] - c0[0]) * f,
                        c0[1] + (c1[1] - c0[1]) * f,
                        c0[2] + (c1[2] - c0[2]) * f,
                    ]
                };
                let (p0, p1) = if full {
                    (node0.coord, node1.coord)
                } else {
                    (
                        lerp(node0.coord, node1.coord, t[0]),
                        lerp(node0.coord, node1.coord, t[1]),
                    )
                };
                let hit0 = beam_span_position(model, p0, SPAN_TOL_MM);
                let hit1 = beam_span_position(model, p1, SPAN_TOL_MM);
                if let (Some((e0, a0)), Some((e1, a1))) = (hit0, hit1) {
                    if e0 == e1 {
                        let start = a0.min(a1);
                        let len_e = (a1 - a0).abs();
                        if len_e > 1e-9 {
                            emit_shape(&mut member, e0, start, len_e, a0 > a1, &bl.shape);
                        }
                        continue;
                    }
                }
                // 線分が複数の大梁にまたがる場合（取付き線の下の大梁が中間節点で
                // 分割されている等）は、覆っている梁へ幾何で割り付ける。節点対の
                // 完全一致に頼ると、この場合に両端節点への振り分けへ落ちてしまい、
                // 等分布が集中荷重に化けて梁のモーメントが過小になる。
                if emit_along_segment(model, &mut member, p0, p1, &bl.shape) {
                    continue;
                }
                // どの大梁も区間を覆えなかった場合、荷重を落とさず nodes（取付き線の
                // 実節点＝柱）へ単純梁反力として振り分ける。部分区間ではその反力は
                // 本来 p0・p1（nodes 間の内側）に生じるが、そこに実節点はないため、
                // 総和を保つ最善の代替として nodes 自身へ寄せる（位置はわずかにずれる）。
                let len = {
                    let (a, b) = (p0, p1);
                    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
                };
                let (r0, r1) = simple_reactions(&bl.shape, len);
                for (n, r) in [(n0, r0), (n1, r1)] {
                    if r.abs() > 1e-9 {
                        nodal.push(NodalLoad::auto(n, [0.0, 0.0, -r, 0.0, 0.0, 0.0]));
                    }
                }
            }
        }
    }

    let (nodal, extra_member) = resolve_nodal_to_primary(model, nodal, SPAN_TOL_MM);
    member.extend(extra_member);
    (nodal, member)
}

/// 床の DL 分配 `BeamLoad` 列（スラブ固定荷重＋自立壁の等価面荷重）。
///
/// 現状の CMQ 図ソースでもあるが、荷重ケースの全部材荷重ではない
/// （梁自重・取り付く壁版の線アンカーは [`compute_gravity_auto_load_cases`] 側で
/// 「DL」へ加算する。CMQ 図への反映は
/// `dev_docs/handoff/CMQ図を荷重ケースの全荷重へ_申し送り.md`）。
pub fn compute_dl_beam_loads(model: &Model) -> Vec<BeamLoad> {
    let beam_map = beam_elem_map(model);
    let unit_reactions = slab_grillage_unit_reactions(model, &beam_map);
    let extra_intensity = squid_n_load::wall_attached::floor_region_wall_extra_intensity(model);
    slab_beam_loads_with(
        model,
        |slab| {
            model.slab_dead_intensity(slab) + extra_intensity.get(&slab.id).copied().unwrap_or(0.0)
        },
        &unit_reactions,
        &beam_map,
    )
}

/// 重力系（DL・LL(架構用)・LL(地震用)）の自動生成内容を計算する。
pub fn compute_gravity_auto_load_cases(model: &Model) -> AutoLoadComputeResult {
    let beam_map = beam_elem_map(model);
    let unit_reactions = slab_grillage_unit_reactions(model, &beam_map);
    let dl_beam_loads = compute_dl_beam_loads(model);

    let (mut dl_nodal, mut dl_member) = slab_load_case_content(model, &dl_beam_loads);
    let load_cfg = model.load_cfg.clone().unwrap_or_default();
    let (sw_nodal, sw_member) =
        squid_n_load::self_weight::self_weight_case_content(model, &load_cfg);
    dl_nodal.extend(sw_nodal);
    dl_member.extend(sw_member);
    // 「線」アンカーの取り付く壁版（パラペット・腰壁・垂れ壁）の自重。床板分配と同じ
    // 幾何解決（`slab_load_case_content` の `LoadTarget::Span`）へ合流させることで、
    // 取付き線の部分区間（`span`）を梁全長へ薄めず正確に扱う（`wall_attached` 参照）。
    let attached_wall_loads = squid_n_load::wall_attached::attached_wall_beam_loads(model);
    let (aw_nodal, aw_member) = slab_load_case_content(model, &attached_wall_loads);
    dl_nodal.extend(aw_nodal);
    dl_member.extend(aw_member);
    let (dl_nodal, extra_member) = resolve_nodal_to_primary(model, dl_nodal, SPAN_TOL_MM);
    dl_member.extend(extra_member);

    let ll_beam_loads = slab_beam_loads_with(
        model,
        |slab| slab.live_intensity(LoadPurpose::Frame),
        &unit_reactions,
        &beam_map,
    );
    let (ll_nodal, ll_member) = slab_load_case_content(model, &ll_beam_loads);

    let ls_beam_loads = slab_beam_loads_with(
        model,
        |slab| slab.live_intensity(LoadPurpose::Seismic),
        &unit_reactions,
        &beam_map,
    );
    let (ls_nodal, ls_member) = slab_load_case_content(model, &ls_beam_loads);

    AutoLoadComputeResult {
        cases: vec![
            AutoLoadCaseContent {
                name: DL_CASE_NAME,
                kind: LoadCaseKind::Dead,
                nodal: dl_nodal,
                member: dl_member,
            },
            AutoLoadCaseContent {
                name: LL_FRAME_CASE_NAME,
                kind: LoadCaseKind::Live,
                nodal: ll_nodal,
                member: ll_member,
            },
            AutoLoadCaseContent {
                name: LL_SEISMIC_CASE_NAME,
                kind: LoadCaseKind::LiveSeismic,
                nodal: ls_nodal,
                member: ls_member,
            },
        ],
        notices: Vec::new(),
    }
}

/// 地震系（EX・EY）の自動生成内容を計算する。
pub fn compute_seismic_auto_load_cases(
    model: &Model,
    settings: &AnalysisSettings,
    design_period: Option<f64>,
) -> AutoLoadComputeResult {
    let mut notices = Vec::new();
    let mut cases = Vec::new();

    if model.stories.is_empty() {
        return AutoLoadComputeResult { cases, notices };
    }

    let t = match settings.ai_mode {
        AiMode::Approx => {
            let height_m = analysis::building_height_mm(model) / 1000.0;
            let steel_ratio = analysis::steel_height_ratio(model);
            Some(squid_n_load::ai::approx_t(height_m, steel_ratio))
        }
        AiMode::SemiPrecise => design_period,
    };

    let Some(t) = t else {
        notices.push(
            "精算周期(固有値解析)が選択されていますが固有値解析が未実行です。\
             解析タブの固有値解析を先に実行してください\
             (EX/EY の地震荷重は更新されません)。"
                .to_string(),
        );
        return AutoLoadComputeResult { cases, notices };
    };

    for (dir, name) in [(SeismicDir::X, EX_CASE_NAME), (SeismicDir::Y, EY_CASE_NAME)] {
        let cfg = analysis::SeismicCfg {
            dir,
            mode: settings.ai_mode,
            z: settings.z,
            soil: settings.soil,
            c0: settings.c0,
        };
        if let Ok(lc) = analysis::build_seismic_load_case_from_model(model, cfg, t) {
            cases.push(AutoLoadCaseContent {
                name,
                kind: LoadCaseKind::Seismic,
                nodal: lc.nodal,
                member: lc.member,
            });
        }
    }

    AutoLoadComputeResult { cases, notices }
}

/// 重力(DL/LL/LL地震用)＋地震(EX/EY)の自動生成内容を計算する（モデルは書き換えない）。
pub fn compute_auto_load_cases(
    model: &Model,
    settings: &AnalysisSettings,
    design_period: Option<f64>,
) -> AutoLoadComputeResult {
    let gravity = compute_gravity_auto_load_cases(model);
    let seismic = compute_seismic_auto_load_cases(model, settings, design_period);
    AutoLoadComputeResult {
        cases: gravity.cases.into_iter().chain(seismic.cases).collect(),
        notices: seismic.notices,
    }
}

fn node_exists(model: &Model, id: NodeId) -> bool {
    id.index() < model.nodes.len()
}

fn elem_exists(model: &Model, id: ElemId) -> bool {
    model.element(id).is_some()
}

/// `cases` の内容を model の同名ケースへ Auto 分として書き込む（無ければ作成）。undo なし。
pub fn apply_auto_load_cases(model: &mut Model, cases: &[AutoLoadCaseContent]) {
    use squid_n_core::ids::LoadCaseId;

    for case in cases {
        if !case.nodal.iter().all(|l| node_exists(model, l.node))
            || !case.member.iter().all(|l| elem_exists(model, l.elem))
        {
            continue;
        }

        let empty = case.nodal.is_empty() && case.member.is_empty();
        if let Some(idx) = model.load_cases.iter().position(|lc| lc.name == case.name) {
            model.load_cases[idx].kind = case.kind;
            model.load_cases[idx].replace_auto_loads(case.nodal.clone(), case.member.clone());
        } else if !empty {
            let new_id = LoadCaseId(model.load_cases.len() as u32);
            let mut lc = LoadCase {
                id: new_id,
                name: case.name.to_string(),
                kind: case.kind,
                nodal: Vec::new(),
                member: Vec::new(),
            };
            lc.replace_auto_loads(case.nodal.clone(), case.member.clone());
            model.load_cases.push(lc);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::{FloorRegionId, NodeId, SlabId};
    use squid_n_core::model::{
        AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime,
        LocalAxis, Node,
    };
    use squid_n_core::model::{Slab, SlabPlate, SlabShape};

    fn make_square_slab_model() -> Model {
        let mk_node = |id: u32, x: f64, y: f64| Node {
            id: NodeId(id),
            coord: [x, y, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        };
        let nodes = vec![
            mk_node(0, 0.0, 0.0),
            mk_node(1, 4000.0, 0.0),
            mk_node(2, 4000.0, 4000.0),
            mk_node(3, 0.0, 4000.0),
        ];
        let mk_beam = |id: u32, i: u32, j: u32| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        let elements = vec![
            mk_beam(0, 0, 1),
            mk_beam(1, 1, 2),
            mk_beam(2, 2, 3),
            mk_beam(3, 3, 0),
        ];
        let boundary = vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)];
        let slab = Slab {
            id: SlabId(0),
            shape: SlabShape::Enclosed {
                boundary: boundary.clone(),
            },
            plate: SlabPlate {
                section: None,
                loads: vec![AreaLoad {
                    kind: "DL".into(),
                    value: 0.005,
                }],
                usage: None,
                method: DistributionMethod::TriTrapezoid,
                one_way: None,
            },
        };
        let mut region = FloorRegion::new(FloorRegionId(0), boundary);
        region.slab_ids.push(slab.id);
        Model {
            nodes,
            elements,
            floor_regions: vec![region],
            slabs: vec![slab],
            ..Default::default()
        }
    }

    #[test]
    fn compute_gravity_creates_dl_with_slab_loads() {
        let model = make_square_slab_model();
        model.validate().expect("valid model");
        let result = compute_gravity_auto_load_cases(&model);
        assert_eq!(result.cases.len(), 3);
        let dl = result
            .cases
            .iter()
            .find(|c| c.name == DL_CASE_NAME)
            .expect("DL");
        assert_eq!(dl.kind, LoadCaseKind::Dead);
        assert_eq!(dl.member.len(), 8, "4辺 × 三角形2区間");
    }

    #[test]
    fn apply_auto_load_cases_creates_dl_case() {
        let mut model = make_square_slab_model();
        model.validate().expect("valid model");
        let computed = compute_gravity_auto_load_cases(&model);
        apply_auto_load_cases(&mut model, &computed.cases);
        let dl = model
            .load_cases
            .iter()
            .find(|lc| lc.name == DL_CASE_NAME)
            .expect("DL case exists");
        assert_eq!(dl.member.len(), 8);
        assert!(dl.nodal.iter().all(|nl| nl.source.is_auto()));
    }

    /// 取り付く壁版（パラペット等）の自重が「DL」ケースへ合流すること
    /// （`squid_n_load::wall_attached` の結線。dig 2026-08-27 Q2=A・Q3=B）。
    #[test]
    fn compute_gravity_includes_attached_wall_plate_self_weight() {
        use squid_n_core::ids::{MaterialId, SectionId, WallPlateId};
        use squid_n_core::model::{
            LoadTransfer, Material, MaterialCategory, RegionAnchor, Section, WallPlate,
            WallPlateShape,
        };

        let mut model = make_square_slab_model();
        model.materials.push(Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Fc24".into(),
            category: MaterialCategory::Concrete,
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        });
        model.sections.push(Section {
            id: SectionId(0),
            name: "壁 t150".into(),
            area: 0.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 0.0,
            width: 0.0,
            as_y: 1.0,
            as_z: 1.0,
            floor: None,
            panel_thickness: None,
            thickness: Some(150.0),
            shape: None,
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        });
        // 辺0（節点0-1、大梁として実在）に全長載るパラペット（立ち上がり500mm）。
        let plate = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: [0.0, 1.0],
                    transfer: LoadTransfer::Anchor,
                },
                extent: [500.0, 500.0],
            },
            section: Some(SectionId(0)),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            three_side_slit: false,
        };
        let expected = model
            .wall_plate_self_weight(&plate, &model)
            .expect("自重が求まる");
        model.wall_plates.push(plate);
        model.validate().expect("valid model");

        let baseline = compute_gravity_auto_load_cases(&make_square_slab_model());
        let baseline_dl = baseline
            .cases
            .iter()
            .find(|c| c.name == DL_CASE_NAME)
            .unwrap();

        let result = compute_gravity_auto_load_cases(&model);
        let dl = result
            .cases
            .iter()
            .find(|c| c.name == DL_CASE_NAME)
            .expect("DL");
        assert_eq!(
            dl.member.len(),
            baseline_dl.member.len() + 1,
            "辺0（節点0-1）へ壁の等分布荷重が1件加わるはず"
        );

        // 辺0（節点0-1）に載る分布荷重の合計force（w×区間長の総和）を、壁版追加の
        // 前後で比較する。増分が壁の自重総量と一致すること（総和保存）。
        let beam0 = beam_elem_map(&model)[&beam_key(NodeId(0), NodeId(1))];
        let member_total_on = |members: &[squid_n_core::model::MemberLoad]| -> f64 {
            members
                .iter()
                .filter(|m| m.elem == beam0)
                .map(|m| match m.kind {
                    squid_n_core::model::MemberLoadKind::Distributed { a, b, w1, w2 } => {
                        0.5 * (w1 + w2) * (b - a)
                    }
                    _ => 0.0,
                })
                .sum()
        };
        let added = member_total_on(&dl.member) - member_total_on(&baseline_dl.member);
        assert!(
            (added - expected).abs() / expected < 1e-6,
            "added={added} expected={expected}"
        );
    }

    /// 台形の取り付く壁版が DL の部材荷重まで線形変化として届き、強度比と総和が保存される。
    #[test]
    fn compute_gravity_preserves_trapezoid_linear_shape_on_beam() {
        use squid_n_core::ids::{MaterialId, SectionId, WallPlateId};
        use squid_n_core::model::{
            LoadTransfer, Material, MaterialCategory, RegionAnchor, Section, WallPlate,
            WallPlateShape,
        };

        let mut model = make_square_slab_model();
        model.materials.push(Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Fc24".into(),
            category: MaterialCategory::Concrete,
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        });
        model.sections.push(Section {
            id: SectionId(0),
            name: "壁 t150".into(),
            area: 0.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 0.0,
            width: 0.0,
            as_y: 1.0,
            as_z: 1.0,
            floor: None,
            panel_thickness: None,
            thickness: Some(150.0),
            shape: None,
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        });
        let plate = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: [0.0, 1.0],
                    transfer: LoadTransfer::Anchor,
                },
                extent: [500.0, 1500.0],
            },
            section: Some(SectionId(0)),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            three_side_slit: false,
        };
        let expected = model
            .wall_plate_self_weight(&plate, &model)
            .expect("自重が求まる");
        model.wall_plates.push(plate);
        model.validate().expect("valid model");

        let result = compute_gravity_auto_load_cases(&model);
        let dl = result
            .cases
            .iter()
            .find(|c| c.name == DL_CASE_NAME)
            .expect("DL");
        let beam0 = beam_elem_map(&model)[&beam_key(NodeId(0), NodeId(1))];
        let wall_loads: Vec<_> = dl
            .member
            .iter()
            .filter(|m| m.elem == beam0)
            .filter_map(|m| match m.kind {
                squid_n_core::model::MemberLoadKind::Distributed { a, b, w1, w2 } => {
                    Some((a, b, w1, w2))
                }
                _ => None,
            })
            .collect();
        // 床分配の等分布に、壁の線形変化が 1 件乗る。
        let linear = wall_loads
            .iter()
            .copied()
            .find(|&(_, _, w1, w2)| w1 > 1e-12 && w2 > 1e-12 && (w1 - w2).abs() > 1e-12)
            .expect("台形壁の線形変化（両端とも正で不等）が部材荷重に載るはず");
        let (a, b, w1, w2) = linear;
        let h0 = 500.0;
        let h1 = 1500.0;
        assert!(
            ((w1 / w2) - (h0 / h1)).abs() < 1e-9,
            "w1/w2={} h0/h1={}",
            w1 / w2,
            h0 / h1
        );
        let added = 0.5 * (w1 + w2) * (b - a);
        assert!(
            (added - expected).abs() / expected < 1e-6,
            "added={added} expected={expected}"
        );
    }

    /// 自立壁（床領域アンカー）の自重が、床の固定荷重分配経由で DL の総和に乗る。
    #[test]
    fn compute_gravity_includes_floor_region_attached_wall_as_slab_extra() {
        use squid_n_core::ids::{MaterialId, SectionId, WallPlateId};
        use squid_n_core::model::{
            Material, MaterialCategory, RegionAnchor, Section, WallPlate, WallPlateShape,
        };

        let mut model = make_square_slab_model();
        model.materials.push(Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Fc24".into(),
            category: MaterialCategory::Concrete,
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        });
        model.sections.push(Section {
            id: SectionId(0),
            name: "壁 t150".into(),
            area: 0.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 0.0,
            width: 0.0,
            as_y: 1.0,
            as_z: 1.0,
            floor: None,
            panel_thickness: None,
            thickness: Some(150.0),
            shape: None,
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        });
        // 自立壁は床領域の**内側**に置く（境界の辺上に置くと、厳密内包の判定で
        // 「床に載っていない」＝解析前チェックのエラー対象になる）。
        let n = model.nodes.len() as u32;
        for (i, x) in [1000.0f64, 3000.0].into_iter().enumerate() {
            model.nodes.push(squid_n_core::model::Node {
                id: NodeId(n + i as u32),
                coord: [x, 2000.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            });
        }
        let plate = WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Attached {
                anchor: RegionAnchor::FloorRegion {
                    nodes: [NodeId(n), NodeId(n + 1)],
                },
                extent: [500.0, 500.0],
            },
            section: Some(SectionId(0)),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            three_side_slit: false,
        };
        let expected = model
            .wall_plate_self_weight(&plate, &model)
            .expect("自重が求まる");
        model.wall_plates.push(plate);
        model.validate().expect("valid model");

        let member_total = |members: &[squid_n_core::model::MemberLoad]| -> f64 {
            members
                .iter()
                .map(|m| match m.kind {
                    squid_n_core::model::MemberLoadKind::Distributed { a, b, w1, w2 } => {
                        0.5 * (w1 + w2) * (b - a)
                    }
                    squid_n_core::model::MemberLoadKind::Point { p, .. } => p,
                })
                .sum()
        };
        let nodal_total = |nodal: &[squid_n_core::model::NodalLoad]| -> f64 {
            nodal.iter().map(|nl| -nl.values[2]).sum()
        };

        let baseline = compute_gravity_auto_load_cases(&make_square_slab_model());
        let baseline_dl = baseline
            .cases
            .iter()
            .find(|c| c.name == DL_CASE_NAME)
            .unwrap();
        let result = compute_gravity_auto_load_cases(&model);
        let dl = result
            .cases
            .iter()
            .find(|c| c.name == DL_CASE_NAME)
            .expect("DL");
        let added = member_total(&dl.member) + nodal_total(&dl.nodal)
            - member_total(&baseline_dl.member)
            - nodal_total(&baseline_dl.nodal);
        assert!(
            (added - expected).abs() / expected < 1e-6,
            "added={added} expected={expected}"
        );
    }

    #[test]
    fn compute_seismic_skips_when_semiprecise_without_period() {
        use squid_n_core::ids::StoryId;
        use squid_n_core::model::Story;

        let mut model = Model::default();
        model.stories.push(Story {
            id: StoryId(0),
            name: "1F".into(),
            elevation: 0.0,
            node_ids: vec![],
            seismic_weight: Some(1.0e6),
            weight_override: None,
            structure: Default::default(),
            level_kind: Default::default(),
        });
        let settings = AnalysisSettings {
            ai_mode: AiMode::SemiPrecise,
            ..Default::default()
        };
        let result = compute_seismic_auto_load_cases(&model, &settings, None);
        assert!(result.cases.is_empty());
        assert_eq!(result.notices.len(), 1);
    }
}

#[cfg(test)]
mod attached_anchor_tests {
    //! 取り付き領域の取付き線が、分割された大梁に載る場合の荷重の割り付け。

    use super::*;
    use squid_n_core::ids::{SectionId, SlabId};
    use squid_n_core::model::{
        AreaLoad, ElementData, ElementKind, EndCondition, ForceRegime, LoadTransfer, LocalAxis,
        MemberLoadKind, Node, RegionAnchor, Slab, SlabPlate, SlabShape,
    };

    fn node(id: u32, x: f64) -> Node {
        Node {
            id: NodeId(id),
            coord: [x, 0.0, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    fn beam(id: u32, i: u32, j: u32) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    /// 取付き線 A—B の下の大梁が A—M—B の 2 本に分割されていても、片持ちの等分布は
    /// 両方の梁へ分布荷重として載る（節点への集中荷重に化けない）。
    #[test]
    fn test_attached_anchor_spans_subdivided_beams() {
        const L: f64 = 4000.0;
        const D: f64 = 1500.0;
        const W: f64 = 0.003;

        let mut model = Model {
            nodes: vec![node(0, 0.0), node(1, L / 2.0), node(2, L)],
            elements: vec![beam(0, 0, 1), beam(1, 1, 2)],
            ..Default::default()
        };
        model.slabs = vec![Slab {
            id: SlabId(0),
            shape: SlabShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(2)],
                    span: [0.0, 1.0],
                    transfer: LoadTransfer::Anchor,
                },
                extent: [D, D],
            },
            plate: SlabPlate {
                loads: vec![AreaLoad {
                    kind: "DL".into(),
                    value: W,
                }],
                ..Default::default()
            },
        }];

        let beam_map = beam_elem_map(&model);
        let beam_loads = slab_beam_loads_with(
            &model,
            |s| model.slab_dead_intensity(s),
            &Default::default(),
            &beam_map,
        );
        let (nodal, member) = slab_load_case_content(&model, &beam_loads);

        assert!(nodal.is_empty(), "節点荷重へ落ちない: {nodal:?}");
        assert_eq!(member.len(), 2, "2 本の梁へ分布荷重が載る: {member:?}");

        let mut total = 0.0;
        for ml in &member {
            let MemberLoadKind::Distributed { a, b, w1, w2 } = ml.kind else {
                panic!("分布荷重でない: {ml:?}");
            };
            // 片持ちの等分布なので、どの区間も強度は w×跳ね出し量で一定。
            assert!((w1 - W * D).abs() < 1e-12 && (w2 - W * D).abs() < 1e-12);
            assert!((b - a - L / 2.0).abs() < 1e-9, "梁の全長に載る");
            total += (b - a) * (w1 + w2) / 2.0;
        }
        // 総和保存: w × 取付き長さ × 跳ね出し量。
        assert!((total - W * L * D).abs() / (W * L * D) < 1e-9, "{total}");
    }

    /// 取付き線の部分区間（`span != [0, 1]`）は、区間が覆う長さだけへ載る
    /// （取付き線の全長ではない）。区間の両端は実節点ではなく大梁のスパン中間に
    /// 幾何解決される。
    #[test]
    fn test_attached_anchor_partial_span_loads_only_covered_segment() {
        const L: f64 = 4000.0;
        const D: f64 = 1500.0;
        const W: f64 = 0.003;
        const T: [f64; 2] = [0.25, 0.75]; // 覆う区間: x=1000..3000

        let mut model = Model {
            nodes: vec![node(0, 0.0), node(1, L)],
            elements: vec![beam(0, 0, 1)],
            ..Default::default()
        };
        model.slabs = vec![Slab {
            id: SlabId(0),
            shape: SlabShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: T,
                    transfer: LoadTransfer::Anchor,
                },
                extent: [D, D],
            },
            plate: SlabPlate {
                loads: vec![AreaLoad {
                    kind: "DL".into(),
                    value: W,
                }],
                ..Default::default()
            },
        }];

        let beam_map = beam_elem_map(&model);
        let beam_loads = slab_beam_loads_with(
            &model,
            |s| model.slab_dead_intensity(s),
            &Default::default(),
            &beam_map,
        );
        let (nodal, member) = slab_load_case_content(&model, &beam_loads);

        assert!(nodal.is_empty(), "節点荷重へ落ちない: {nodal:?}");
        assert_eq!(member.len(), 1, "区間全体が1本の梁に載る: {member:?}");

        let MemberLoadKind::Distributed { a, b, w1, w2 } = member[0].kind else {
            panic!("分布荷重でない: {:?}", member[0]);
        };
        let (a_expected, b_expected) = (L * T[0], L * T[1]);
        assert!((a - a_expected).abs() < 1e-6, "a={a} expected={a_expected}");
        assert!((b - b_expected).abs() < 1e-6, "b={b} expected={b_expected}");
        assert!((w1 - W * D).abs() < 1e-12 && (w2 - W * D).abs() < 1e-12);

        // 総和保存: w × 覆う区間の長さ × 跳ね出し量（取付き線の全長ではない）。
        let covered_len = b_expected - a_expected;
        let total = (b - a) * (w1 + w2) / 2.0;
        assert!(
            (total - W * covered_len * D).abs() / (W * covered_len * D) < 1e-9,
            "{total}"
        );
    }

    /// 区間の始端が取付き線の始端節点にちょうど一致する場合（`t = [0.0, 0.5]`）。
    /// `beam_span_position` は節点近傍（`tol` 以内）の点を「梁のスパン中間」から除外するため、
    /// 区間の始端では `hit0` が得られず、`emit_along_segment` の幾何割付側へ落ちる。
    /// その経路でも正しい区間へ載ることを確認する（節点ちょうどに一致するケースの回帰）。
    #[test]
    fn test_attached_anchor_partial_span_touches_start_node() {
        const L: f64 = 4000.0;
        const D: f64 = 1500.0;
        const W: f64 = 0.003;
        const T: [f64; 2] = [0.0, 0.5]; // 覆う区間: x=0..2000（始端が節点0に一致）

        let mut model = Model {
            nodes: vec![node(0, 0.0), node(1, L)],
            elements: vec![beam(0, 0, 1)],
            ..Default::default()
        };
        model.slabs = vec![Slab {
            id: SlabId(0),
            shape: SlabShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: T,
                    transfer: LoadTransfer::Anchor,
                },
                extent: [D, D],
            },
            plate: SlabPlate {
                loads: vec![AreaLoad {
                    kind: "DL".into(),
                    value: W,
                }],
                ..Default::default()
            },
        }];

        let beam_map = beam_elem_map(&model);
        let beam_loads = slab_beam_loads_with(
            &model,
            |s| model.slab_dead_intensity(s),
            &Default::default(),
            &beam_map,
        );
        let (nodal, member) = slab_load_case_content(&model, &beam_loads);

        assert!(nodal.is_empty(), "節点荷重へ落ちない: {nodal:?}");
        assert_eq!(member.len(), 1, "区間全体が1本の梁に載る: {member:?}");

        let MemberLoadKind::Distributed { a, b, w1, w2 } = member[0].kind else {
            panic!("分布荷重でない: {:?}", member[0]);
        };
        let (a_expected, b_expected) = (L * T[0], L * T[1]);
        assert!((a - a_expected).abs() < 1e-6, "a={a} expected={a_expected}");
        assert!((b - b_expected).abs() < 1e-6, "b={b} expected={b_expected}");
        assert!((w1 - W * D).abs() < 1e-12 && (w2 - W * D).abs() < 1e-12);

        let covered_len = b_expected - a_expected;
        let total = (b - a) * (w1 + w2) / 2.0;
        assert!(
            (total - W * covered_len * D).abs() / (W * covered_len * D) < 1e-9,
            "{total}"
        );
    }

    /// 区間の終端が取付き線の終端節点にちょうど一致する場合（`t = [0.5, 1.0]`）。
    /// `test_attached_anchor_partial_span_touches_start_node` の対称ケース。処理コードは
    /// `p0`／`p1` を対称に扱うため通るはずだが、始端側しか確認していなかったため追加する。
    #[test]
    fn test_attached_anchor_partial_span_touches_end_node() {
        const L: f64 = 4000.0;
        const D: f64 = 1500.0;
        const W: f64 = 0.003;
        const T: [f64; 2] = [0.5, 1.0]; // 覆う区間: x=2000..4000（終端が節点1に一致）

        let mut model = Model {
            nodes: vec![node(0, 0.0), node(1, L)],
            elements: vec![beam(0, 0, 1)],
            ..Default::default()
        };
        model.slabs = vec![Slab {
            id: SlabId(0),
            shape: SlabShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: T,
                    transfer: LoadTransfer::Anchor,
                },
                extent: [D, D],
            },
            plate: SlabPlate {
                loads: vec![AreaLoad {
                    kind: "DL".into(),
                    value: W,
                }],
                ..Default::default()
            },
        }];

        let beam_map = beam_elem_map(&model);
        let beam_loads = slab_beam_loads_with(
            &model,
            |s| model.slab_dead_intensity(s),
            &Default::default(),
            &beam_map,
        );
        let (nodal, member) = slab_load_case_content(&model, &beam_loads);

        assert!(nodal.is_empty(), "節点荷重へ落ちない: {nodal:?}");
        assert_eq!(member.len(), 1, "区間全体が1本の梁に載る: {member:?}");

        let MemberLoadKind::Distributed { a, b, w1, w2 } = member[0].kind else {
            panic!("分布荷重でない: {:?}", member[0]);
        };
        let (a_expected, b_expected) = (L * T[0], L * T[1]);
        assert!((a - a_expected).abs() < 1e-6, "a={a} expected={a_expected}");
        assert!((b - b_expected).abs() < 1e-6, "b={b} expected={b_expected}");
        assert!((w1 - W * D).abs() < 1e-12 && (w2 - W * D).abs() < 1e-12);

        let covered_len = b_expected - a_expected;
        let total = (b - a) * (w1 + w2) / 2.0;
        assert!(
            (total - W * covered_len * D).abs() / (W * covered_len * D) < 1e-9,
            "{total}"
        );
    }

    /// `LoadTransfer::Columns`（出隅・雑壁の柱伝達に相当）で部分区間を使う場合、
    /// 総荷重の作用点は区間の中点（無次元位置 `t_mid`）にあるため、単純梁の
    /// 集中荷重反力公式（R0 = P(1-t)、R1 = P・t）で両端の柱へ按分する。
    /// 全長（`t = [0, 1]`、`t_mid = 0.5`）なら従来どおり半分ずつになる。
    #[test]
    fn test_attached_anchor_columns_transfer_partial_span_splits_by_lever_arm() {
        const L: f64 = 4000.0;
        const D: f64 = 1500.0;
        const W: f64 = 0.003;
        const T: [f64; 2] = [0.75, 1.0]; // 覆う区間: x=3000..4000、区間中点 t_mid=0.875

        let mut model = Model {
            nodes: vec![node(0, 0.0), node(1, L)],
            elements: vec![beam(0, 0, 1)],
            ..Default::default()
        };
        model.slabs = vec![Slab {
            id: SlabId(0),
            shape: SlabShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: T,
                    transfer: LoadTransfer::Columns,
                },
                extent: [D, D],
            },
            plate: SlabPlate {
                loads: vec![AreaLoad {
                    kind: "DL".into(),
                    value: W,
                }],
                ..Default::default()
            },
        }];

        let beam_map = beam_elem_map(&model);
        let beam_loads = slab_beam_loads_with(
            &model,
            |s| model.slab_dead_intensity(s),
            &Default::default(),
            &beam_map,
        );
        let (nodal, member) = slab_load_case_content(&model, &beam_loads);

        assert!(
            member.is_empty(),
            "柱集中荷重は分布荷重を生じない: {member:?}"
        );
        assert_eq!(nodal.len(), 2, "両端の柱へ集中: {nodal:?}");

        let covered_len = L * (T[1] - T[0]);
        let total_w = W * D * covered_len;
        let t_mid = 0.5 * (T[0] + T[1]);
        let r0 = nodal
            .iter()
            .find(|nl| nl.node == NodeId(0))
            .map(|nl| -nl.values[2])
            .unwrap_or(0.0);
        let r1 = nodal
            .iter()
            .find(|nl| nl.node == NodeId(1))
            .map(|nl| -nl.values[2])
            .unwrap_or(0.0);
        assert!(
            (r0 - total_w * (1.0 - t_mid)).abs() / total_w < 1e-9,
            "r0={r0}"
        );
        assert!((r1 - total_w * t_mid).abs() / total_w < 1e-9, "r1={r1}");
        assert!(
            ((r0 + r1) - total_w).abs() / total_w < 1e-9,
            "総和保存: {}",
            r0 + r1
        );
    }

    /// 取付き線の部分区間が、下の大梁の分割点（中間節点）をまたいでいても、
    /// 覆っている両方の梁へ正しく分割して載る。
    #[test]
    fn test_attached_anchor_partial_span_crosses_beam_split() {
        const L: f64 = 4000.0;
        const D: f64 = 1500.0;
        const W: f64 = 0.003;
        const T: [f64; 2] = [0.25, 0.75]; // 覆う区間: x=1000..3000（分割点 x=2000 をまたぐ）

        let mut model = Model {
            nodes: vec![node(0, 0.0), node(1, L / 2.0), node(2, L)],
            elements: vec![beam(0, 0, 1), beam(1, 1, 2)],
            ..Default::default()
        };
        model.slabs = vec![Slab {
            id: SlabId(0),
            shape: SlabShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(2)],
                    span: T,
                    transfer: LoadTransfer::Anchor,
                },
                extent: [D, D],
            },
            plate: SlabPlate {
                loads: vec![AreaLoad {
                    kind: "DL".into(),
                    value: W,
                }],
                ..Default::default()
            },
        }];

        let beam_map = beam_elem_map(&model);
        let beam_loads = slab_beam_loads_with(
            &model,
            |s| model.slab_dead_intensity(s),
            &Default::default(),
            &beam_map,
        );
        let (nodal, member) = slab_load_case_content(&model, &beam_loads);

        assert!(nodal.is_empty(), "節点荷重へ落ちない: {nodal:?}");
        assert_eq!(member.len(), 2, "分割点の両側それぞれへ載る: {member:?}");

        let mut total = 0.0;
        for ml in &member {
            let MemberLoadKind::Distributed { a, b, w1, w2 } = ml.kind else {
                panic!("分布荷重でない: {ml:?}");
            };
            assert!((w1 - W * D).abs() < 1e-12 && (w2 - W * D).abs() < 1e-12);
            total += (b - a) * (w1 + w2) / 2.0;
        }
        let covered_len = L * (T[1] - T[0]);
        assert!(
            (total - W * covered_len * D).abs() / (W * covered_len * D) < 1e-9,
            "{total}"
        );
    }

    /// 重複部材（同じ区間を覆う梁が 2 本）があるときは幾何割付を使わない。
    ///
    /// 被覆長の単純合計で判定すると、重なりのぶんだけ長さが水増しされ、隙間があっても
    /// 全長を覆えたと誤判定する。前半へ二重に載り、後半が無荷重になるため、
    /// 従来どおり両端節点へ振り分ける。
    #[test]
    fn test_overlapping_beams_fall_back_to_nodes() {
        const L: f64 = 4000.0;
        const D: f64 = 1500.0;
        const W: f64 = 0.003;

        let mut model = Model {
            // 0—1（半分だけ）に重複した梁 2 本。1—2 には梁が無い。
            nodes: vec![node(0, 0.0), node(1, L / 2.0), node(2, L)],
            elements: vec![beam(0, 0, 1), beam(1, 0, 1)],
            ..Default::default()
        };
        model.slabs = vec![Slab {
            id: SlabId(0),
            shape: SlabShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(2)],
                    span: [0.0, 1.0],
                    transfer: LoadTransfer::Anchor,
                },
                extent: [D, D],
            },
            plate: SlabPlate {
                loads: vec![AreaLoad {
                    kind: "DL".into(),
                    value: W,
                }],
                ..Default::default()
            },
        }];

        let beam_map = beam_elem_map(&model);
        let beam_loads = slab_beam_loads_with(
            &model,
            |s| model.slab_dead_intensity(s),
            &Default::default(),
            &beam_map,
        );
        let (nodal, member) = slab_load_case_content(&model, &beam_loads);
        assert!(
            member.is_empty(),
            "重なりがあるときは梁へ割り付けない: {member:?}"
        );
        let total: f64 = nodal.iter().map(|nl| -nl.values[2]).sum();
        assert!(
            (total - W * L * D).abs() / (W * L * D) < 1e-9,
            "総和は保たれる: {total}"
        );
    }

    /// 集中荷重が被覆区間の境目に載っても、二重に計上しない。
    #[test]
    fn test_point_load_on_coverage_boundary_is_placed_once() {
        const L: f64 = 4000.0;
        let model = Model {
            nodes: vec![node(0, 0.0), node(1, L / 2.0), node(2, L)],
            elements: vec![beam(0, 0, 1), beam(1, 1, 2)],
            ..Default::default()
        };
        // 2 本の梁の境目（線分の中央）へ集中荷重を載せる。
        let bl = squid_n_load::floor::BeamLoad {
            elem: ElemId(u32::MAX),
            target: LoadTarget::Span {
                nodes: [NodeId(0), NodeId(2)],
                t: [0.0, 1.0],
            },
            shape: squid_n_load::floor::LoadShape::Point {
                p: 1000.0,
                x: L / 2.0,
            },
            cmq: squid_n_load::floor::Cmq {
                c_i: 0.0,
                c_j: 0.0,
                q_i: 0.0,
                q_j: 0.0,
            },
        };
        let (nodal, member) = slab_load_case_content(&model, &[bl]);
        assert!(nodal.is_empty());
        assert_eq!(member.len(), 1, "1 本の梁へ 1 度だけ載る: {member:?}");
    }
}
