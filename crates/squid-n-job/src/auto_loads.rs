//! 重力（DL/LL）・地震（EX/EY）荷重ケースの自動生成（モデルは書き換えない）。
//!
//! GUI は [`compute_auto_load_cases`] で内容を求め、undo 付きの
//! `SyncSlabLoadsToCase` で書き込む。MCP 等のエフェメラルな作業コピーは
//! [`apply_auto_load_cases`] で直接反映する。

use std::collections::HashMap;

use squid_n_core::ids::{ElemId, FloorRegionId, LoadCaseId, NodeId};
use squid_n_core::model::{
    ElementKind, FloorRegion, LoadCase, LoadCaseKind, LoadPurpose, MemberLoad, MemberLoadKind,
    Model, NodalLoad, DL_CASE_NAME, EX_CASE_NAME, EY_CASE_NAME, LL_FRAME_CASE_NAME,
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
    /// DL 分配の `BeamLoad`（GUI の CMQ 図用）。
    pub dl_beam_loads: Vec<BeamLoad>,
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

/// 各スラブについて面荷重強度 `w_of(slab)` を境界へ分配し、`BeamLoad` 列を返す。
pub fn slab_beam_loads_with(
    model: &Model,
    w_of: impl Fn(&FloorRegion) -> f64,
    unit_reactions: &HashMap<FloorRegionId, Vec<(NodeId, f64)>>,
    beam_map: &HashMap<(NodeId, NodeId), ElemId>,
) -> Vec<BeamLoad> {
    let mut beam_loads = Vec::new();
    for slab in &model.floor_regions {
        let find_beam =
            |n0: NodeId, n1: NodeId| -> Option<ElemId> { beam_map.get(&beam_key(n0, n1)).copied() };
        let w = w_of(slab);
        let grillage_reactions: Option<Vec<(NodeId, f64)>> = unit_reactions
            .get(&slab.id)
            .map(|rs| rs.iter().map(|(node, r)| (*node, r * w)).collect());
        for mut bl in floor::distribute_slab_w(model, slab, w) {
            match bl.target {
                LoadTarget::Node(_) => {
                    if grillage_reactions.is_none() {
                        beam_loads.push(bl);
                    }
                }
                LoadTarget::Edge(k) => {
                    // 辺 k の両端節点。取り付き領域は辺 0（取付き線）だけが受け手を持つ。
                    let Some([n0, n1]) = slab.edge_nodes(k) else {
                        continue;
                    };
                    match find_beam(n0, n1) {
                        Some(elem) => {
                            bl.elem = elem;
                            beam_loads.push(bl);
                        }
                        None => {
                            bl.elem = ElemId(u32::MAX);
                            bl.target = LoadTarget::Span([n0, n1]);
                            beam_loads.push(bl);
                        }
                    }
                }
                LoadTarget::Span([n0, n1]) => {
                    if let Some(elem) = find_beam(n0, n1) {
                        bl.elem = elem;
                    }
                    beam_loads.push(bl);
                }
            }
        }
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
            LoadShape::Uniform { .. } => Vec::new(),
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
            LoadTarget::Span([n0, n1]) => {
                if let Some(elem) = model.element(bl.elem) {
                    let l = model.member_length(elem);
                    if l > 1e-9 {
                        emit_shape(&mut member, elem.id, 0.0, l, false, &bl.shape);
                    }
                    continue;
                }
                let (Some(node0), Some(node1)) =
                    (model.nodes.get(n0.index()), model.nodes.get(n1.index()))
                else {
                    continue;
                };
                let hit0 = beam_span_position(model, node0.coord, SPAN_TOL_MM);
                let hit1 = beam_span_position(model, node1.coord, SPAN_TOL_MM);
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
                if emit_along_segment(model, &mut member, node0.coord, node1.coord, &bl.shape) {
                    continue;
                }
                let len = {
                    let (a, b) = (node0.coord, node1.coord);
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

/// CMQ 図用の DL スラブ分配 `BeamLoad` 列（自重・LL は含まない）。
pub fn compute_dl_beam_loads(model: &Model) -> Vec<BeamLoad> {
    let beam_map = beam_elem_map(model);
    let unit_reactions = slab_grillage_unit_reactions(model, &beam_map);
    slab_beam_loads_with(
        model,
        |slab| model.region_dead_intensity(slab),
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
        dl_beam_loads,
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
        return AutoLoadComputeResult {
            cases,
            dl_beam_loads: Vec::new(),
            notices,
        };
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
        return AutoLoadComputeResult {
            cases,
            dl_beam_loads: Vec::new(),
            notices,
        };
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

    AutoLoadComputeResult {
        cases,
        dl_beam_loads: Vec::new(),
        notices,
    }
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
        dl_beam_loads: gravity.dl_beam_loads,
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
    use squid_n_core::ids::{FloorRegionId, NodeId};
    use squid_n_core::model::{
        AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime,
        LocalAxis, Node,
    };
    use squid_n_core::model::{RegionShape, SlabPlate};

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
        let slab = FloorRegion {
            id: FloorRegionId(0),
            name: String::new(),
            shape: RegionShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            plate: Some(SlabPlate {
                section: None,
                loads: vec![AreaLoad {
                    kind: "DL".into(),
                    value: 0.005,
                }],
                usage: None,
                method: DistributionMethod::TriTrapezoid,
                one_way: None,
                joists: vec![],
            }),
            secondary_joist_ids: vec![],
        };
        Model {
            nodes,
            elements,
            floor_regions: vec![slab],
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
        assert!(!result.dl_beam_loads.is_empty());
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
    use squid_n_core::ids::{FloorRegionId, SectionId};
    use squid_n_core::model::{
        AreaLoad, ElementData, ElementKind, EndCondition, FloorRegion, ForceRegime, LoadTransfer,
        LocalAxis, MemberLoadKind, Node, RegionAnchor, RegionShape, SlabPlate,
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
        model.floor_regions = vec![FloorRegion {
            id: FloorRegionId(0),
            name: String::new(),
            shape: RegionShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(2)],
                    span: [0.0, 1.0],
                },
                extent: [D, D],
                transfer: LoadTransfer::Anchor,
            },
            plate: Some(SlabPlate {
                loads: vec![AreaLoad {
                    kind: "DL".into(),
                    value: W,
                }],
                ..Default::default()
            }),
            secondary_joist_ids: vec![],
        }];

        let beam_map = beam_elem_map(&model);
        let beam_loads = slab_beam_loads_with(
            &model,
            |r| model.region_dead_intensity(r),
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
        model.floor_regions = vec![FloorRegion {
            id: FloorRegionId(0),
            name: String::new(),
            shape: RegionShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(2)],
                    span: [0.0, 1.0],
                },
                extent: [D, D],
                transfer: LoadTransfer::Anchor,
            },
            plate: Some(SlabPlate {
                loads: vec![AreaLoad {
                    kind: "DL".into(),
                    value: W,
                }],
                ..Default::default()
            }),
            secondary_joist_ids: vec![],
        }];

        let beam_map = beam_elem_map(&model);
        let beam_loads = slab_beam_loads_with(
            &model,
            |r| model.region_dead_intensity(r),
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
            target: LoadTarget::Span([NodeId(0), NodeId(2)]),
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
