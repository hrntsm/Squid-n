//! 重力（DL/LL）・地震（EX/EY）荷重ケースの自動生成（モデルは書き換えない）。
//!
//! GUI は [`compute_auto_load_cases`] で内容を求め、undo 付きの
//! `SyncSlabLoadsToCase` で書き込む。MCP 等のエフェメラルな作業コピーは
//! [`apply_auto_load_cases`] で直接反映する。

use std::collections::HashMap;

use squid_n_core::ids::{ElemId, LoadCaseId, NodeId, SlabId};
use squid_n_core::model::{
    ElementKind, LoadCase, LoadCaseKind, LoadPurpose, MemberLoad, MemberLoadKind, Model, NodalLoad,
    Slab, DL_CASE_NAME, EX_CASE_NAME, EY_CASE_NAME, LL_FRAME_CASE_NAME, LL_SEISMIC_CASE_NAME,
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
    slab: &Slab,
    w: f64,
    beam_map: &HashMap<(NodeId, NodeId), ElemId>,
) -> Option<Vec<(NodeId, f64)>> {
    if !floor::uses_joist_distribution(model, slab) {
        return None;
    }
    if slab
        .joists
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
) -> HashMap<SlabId, Vec<(NodeId, f64)>> {
    let mut out = HashMap::new();
    for slab in &model.slabs {
        if let Some(reactions) = slab_grillage_node_reactions(model, slab, 1.0, beam_map) {
            out.insert(slab.id, reactions);
        }
    }
    out
}

/// 各スラブについて面荷重強度 `w_of(slab)` を境界へ分配し、`BeamLoad` 列を返す。
pub fn slab_beam_loads_with(
    model: &Model,
    w_of: impl Fn(&Slab) -> f64,
    unit_reactions: &HashMap<SlabId, Vec<(NodeId, f64)>>,
    beam_map: &HashMap<(NodeId, NodeId), ElemId>,
) -> Vec<BeamLoad> {
    let mut beam_loads = Vec::new();
    for slab in &model.slabs {
        let n = slab.boundary.len();
        if n < 3 {
            continue;
        }
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
                    if k >= n {
                        continue;
                    }
                    let n0 = slab.boundary[k];
                    let n1 = slab.boundary[(k + 1) % n];
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
        |slab| model.slab_dead_intensity(slab),
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
    use squid_n_core::ids::{NodeId, SlabId};
    use squid_n_core::model::{
        AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime,
        LocalAxis, Node, Slab,
    };

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
        let slab = Slab {
            usage: None,
            edge_supported: None,
            section: None,
            kind: Default::default(),
            one_way: None,
            id: SlabId(0),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists: vec![],
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: 0.005,
            }],
            method: DistributionMethod::TriTrapezoid,
            secondary_joist_ids: vec![],
        };
        Model {
            nodes,
            elements,
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
