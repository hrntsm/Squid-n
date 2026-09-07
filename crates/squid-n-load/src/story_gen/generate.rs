//! 階（Story）生成の本体。

use super::reactions::static_reactions;
use super::*;

/// 生成結果。[`Model`] へ適用するのは呼び出し側（EditCommand 経由）。
#[derive(Clone, Debug, PartialEq)]
pub struct StoryGenResult {
    /// 下から順の階（基部レベルは含まない）。
    pub stories: Vec<Story>,
    /// 各節点の所属階（`model.nodes` と同順。基部レベルは None）。
    /// 長さは `model.nodes.len()`（新規に生成される代表節点は含まない。
    /// 代表節点の所属階は `rep_nodes` 側の `story` フィールドが正）。
    pub node_story: Vec<Option<StoryId>>,
    /// 各階の剛床拘束（`Reducer` が読む `model.constraints` 用）。
    pub constraints: Vec<Constraint>,
    /// 生成・更新される剛床代表節点（座標＝慣性力重心、拘束・所属階設定済み）。
    /// ID が既存の `model.nodes` 範囲内なら置換（再利用）、範囲外なら新規追加。
    pub rep_nodes: Vec<Node>,
    /// 適用後の `model.generated_masters` の全量。
    pub generated_masters: Vec<NodeId>,
}

/// 階へ節点・剛床・地震用重量を割り付ける（複数の重力荷重ケースを地震用重量に
/// 算入する版）。
///
/// **階レベルの正は [`Model::stories`] の `elevation`**（利用者が定義する）であり、
/// 本関数はそれを書き換えない。階が 1 つも定義されていないモデルに限り、節点の
/// Z 座標をクラスタリングして階レベルを初期化する。階名・階種別・地震用重量の
/// 手入力も既存の階定義からそのまま引き継ぐ。
///
/// ただし**基部レベルの階（基部の床）が無ければ先頭に補う**。階が床レベル列であり
/// その先頭が基部であることは [`Model::layers`] が依拠する不変条件であり、
/// それを成立させるのは本関数の責務である。
///
/// - **階への帰属は区間**（直下階のレベル超〜当該階のレベル以下）。中間高さの
///   節点も区間に入る階へ属し、その重量は階の地震用重量へ算入される。
///   最下階（基部の床）だけは下端を含む点区間で、柱脚・基礎梁の節点が属する
/// - **剛床のスレーブは床面のみ**（階のレベル ±[`DIAPHRAGM_LEVEL_TOL_MM`]）。
///   床面に節点がない階は剛床を持たない
/// - 代表節点の拘束は [`master_restraint`] がスレーブの可動性から決める
/// - 前回生成した剛床代表節点（`model.generated_masters`）は構造節点の
///   クラスタリング対象から除外する（再生成時に過去の代表節点を混ぜない）
/// - 各階の剛床代表節点は、その階の全構造節点の慣性力重心（重量重み付き重心）に
///   新規生成する（既存の `generated_masters` があれば座標・拘束・所属階を更新して再利用）
/// - `gravity_lcs` に指定した各ケースの鉛直下向き荷重を地震重量に算入する
///   （自重は材料密度から常に算入）。重複 ID は 1 回だけ処理する
///   （固定荷重＋地震用積載荷重など複数ケースの合算に対応する下準備）。
///
/// 自重が「DL」ケースへ自動同期されるモデル（標準構成）では、密度からの
/// 自重直接算入と二重計上になるため [`generate_stories_with_opts`] を
/// `include_density_self_weight = false` で使うこと。
pub fn generate_stories_multi(
    model: &Model,
    gravity_lcs: &[LoadCaseId],
) -> Result<StoryGenResult, String> {
    generate_stories_with_opts(model, gravity_lcs, true, MassMethod::default())
}

/// 剛床代表節点（マスター）の拘束を、スレーブが実際にその方向へ動けるかで決める。
/// 構造節点のスレーブだけを可動性の判定対象とする。Rz は並進が 1 つでも自由なときだけ自由にする。
fn master_restraint(
    model: &Model,
    common: Dof6Mask,
    slaves: &[NodeId],
    structural: &[bool],
) -> Dof6Mask {
    let free_slave = |dof: Dof| {
        slaves.iter().any(|n| {
            structural.get(n.index()).copied().unwrap_or(false)
                && model
                    .nodes
                    .get(n.index())
                    .is_some_and(|s| !s.restraint.is_fixed(dof))
        })
    };
    let mut m = common;
    let (ux_free, uy_free) = (free_slave(Dof::Ux), free_slave(Dof::Uy));
    if !ux_free {
        m.set_fixed(Dof::Ux);
    }
    if !uy_free {
        m.set_fixed(Dof::Uy);
    }
    if !ux_free && !uy_free {
        m.set_fixed(Dof::Rz);
    }
    m
}

/// [`generate_stories_multi`] の自重算入方法を選べる版。
/// `include_density_self_weight=false` は密度からの直接算入を行わず、
/// `gravity_lcs` のケース内容だけを算入する（直接算入すると二重計上になる）。
pub fn generate_stories_with_opts(
    model: &Model,
    gravity_lcs: &[LoadCaseId],
    include_density_self_weight: bool,
    mass_method: MassMethod,
) -> Result<StoryGenResult, String> {
    if model.nodes.is_empty() {
        return Err("節点がありません".into());
    }

    let generated: std::collections::HashSet<NodeId> =
        model.generated_masters.iter().copied().collect();
    let struct_nodes: Vec<&Node> = model
        .nodes
        .iter()
        .filter(|n| !generated.contains(&n.id))
        .collect();
    if struct_nodes.is_empty() {
        return Err("節点がありません".into());
    }

    let structural = squid_n_core::dof::structural_nodes(model);

    let base = model.base_elevation();
    let mut story_levels: Vec<f64> = if model.stories.is_empty() {
        let mut zs: Vec<f64> = struct_nodes.iter().map(|n| n.coord[2]).collect();
        zs.sort_by(|a, b| a.total_cmp(b));
        let mut levels: Vec<f64> = Vec::new();
        for z in zs {
            match levels.last() {
                Some(&last) if (z - last).abs() <= LEVEL_TOL_MM => {}
                _ => levels.push(z),
            }
        }
        if levels.len() < 2 {
            return Err(
                "階が定義されておらず、節点の標高(Z)も 1 レベルしかありません。\
                 階を定義するか、2 レベル以上の節点を作成してください。"
                    .into(),
            );
        }
        levels
    } else {
        model.stories.iter().map(|s| s.elevation).collect()
    };
    if story_levels
        .first()
        .is_none_or(|&z| z > base + LEVEL_TOL_MM)
    {
        story_levels.insert(0, base);
    }

    let spans: Vec<(f64, f64)> = story_levels
        .iter()
        .enumerate()
        .map(|(i, &top)| {
            let bottom = if i == 0 { top } else { story_levels[i - 1] };
            (bottom, top)
        })
        .collect();

    let mut nodes_by_story: Vec<Vec<NodeId>> = vec![Vec::new(); story_levels.len()];
    for n in &struct_nodes {
        let z = n.coord[2];
        let hit = spans
            .iter()
            .enumerate()
            .position(|(i, &(b, t))| if i == 0 { z >= b } else { z > b } && z <= t);
        if let Some(i) = hit {
            nodes_by_story[i].push(n.id);
        }
    }

    let mut node_weight = vec![0.0f64; model.nodes.len()];
    let load_cfg = model.load_cfg.clone().unwrap_or_default();

    let mut is_base_node = vec![false; model.nodes.len()];
    for e in &model.elements {
        if !matches!(e.kind, ElementKind::Brace { .. }) {
            for n in &e.nodes {
                if let Some(slot) = is_base_node.get_mut(n.index()) {
                    *slot = true;
                }
            }
        }
    }

    let k_brace_redistribute =
        |node_weight: &mut Vec<f64>, ni: usize, nj: usize, wi: f64, wj: f64| match (
            is_base_node[ni],
            is_base_node[nj],
        ) {
            (true, false) => node_weight[ni] += wi + wj,
            (false, true) => node_weight[nj] += wi + wj,
            (true, true) | (false, false) => {
                node_weight[ni] += wi;
                node_weight[nj] += wj;
            }
        };

    let self_weight_items = enumerate_self_weight(model, &load_cfg);

    let distribute_line_panel =
        |target: &mut Vec<f64>, item: &SelfWeightItem, panel_density_only: bool| match item {
            SelfWeightItem::Line { elem_idx, total } => {
                let elem = &model.elements[*elem_idx];
                let ni = elem.nodes[0].index();
                let nj = elem.nodes[1].index();
                if matches!(elem.kind, ElementKind::Brace { .. })
                    && load_cfg.k_brace_rule == KBraceWeightRule::BaseNodesOnly
                {
                    k_brace_redistribute(target, ni, nj, *total / 2.0, *total / 2.0);
                } else {
                    target[ni] += *total / 2.0;
                    target[nj] += *total / 2.0;
                }
            }
            SelfWeightItem::Panel {
                shares,
                density_shares,
            } => {
                let src = if panel_density_only {
                    density_shares
                } else {
                    shares
                };
                for &(i, w) in src {
                    target[i] += w;
                }
            }
            SelfWeightItem::Damper { .. } | SelfWeightItem::SecondaryLine { .. } => {}
        };

    let mut node_self_weight = vec![0.0f64; model.nodes.len()];
    for item in &self_weight_items {
        distribute_line_panel(&mut node_self_weight, item, true);
    }

    if include_density_self_weight {
        for item in &self_weight_items {
            match item {
                SelfWeightItem::Damper { ni, nj, total } => {
                    node_weight[*ni] += total / 2.0;
                    node_weight[*nj] += total / 2.0;
                }
                SelfWeightItem::SecondaryLine { ni, nj, total } => {
                    node_weight[*ni] += total / 2.0;
                    node_weight[*nj] += total / 2.0;
                }
                SelfWeightItem::Line { .. } | SelfWeightItem::Panel { .. } => {
                    distribute_line_panel(&mut node_weight, item, false);
                }
            }
        }

        crate::wall_attached::accumulate_attached_wall_seismic_weight(model, &mut node_weight);
        crate::wall_plate_load::accumulate_enclosed_wall_seismic_weight(model, &mut node_weight);
    }

    let elem_idx_by_id: std::collections::HashMap<squid_n_core::ids::ElemId, usize> = model
        .elements
        .iter()
        .enumerate()
        .map(|(i, e)| (e.id, i))
        .collect();
    let mut seen_lcs: std::collections::HashSet<LoadCaseId> = std::collections::HashSet::new();
    for &lc_id in gravity_lcs {
        if !seen_lcs.insert(lc_id) {
            continue;
        }
        let Some(lc) = model.load_cases.iter().find(|c| c.id == lc_id) else {
            continue;
        };
        for nl in &lc.nodal {
            if nl.values[2] < 0.0 {
                node_weight[nl.node.index()] += -nl.values[2];
            }
        }
        for ml in &lc.member {
            let Some(elem) = elem_idx_by_id
                .get(&ml.elem)
                .map(|&i| &model.elements[i])
                .filter(|e| e.nodes.len() >= 2)
            else {
                continue;
            };
            let dz = ml.dir[2];
            if dz >= 0.0 {
                continue;
            }
            let ni = elem.nodes[0].index();
            let nj = elem.nodes[1].index();
            let (ci, cj) = (model.nodes[ni].coord, model.nodes[nj].coord);
            let len = ((cj[0] - ci[0]).powi(2) + (cj[1] - ci[1]).powi(2) + (cj[2] - ci[2]).powi(2))
                .sqrt();
            let (ri, rj) = static_reactions(&ml.kind, len);
            let scale = -dz;
            if matches!(elem.kind, ElementKind::Brace { .. })
                && load_cfg.k_brace_rule == KBraceWeightRule::BaseNodesOnly
            {
                k_brace_redistribute(&mut node_weight, ni, nj, ri * scale, rj * scale);
            } else {
                node_weight[ni] += ri * scale;
                node_weight[nj] += rj * scale;
            }
        }
    }

    let mut stories = Vec::new();
    let mut node_story = vec![None; model.nodes.len()];
    let mut constraints = Vec::new();
    let mut rep_nodes: Vec<Node> = Vec::new();
    let mut generated_masters: Vec<NodeId> = Vec::new();

    let mut reuse_masters = model.generated_masters.iter().copied();
    let mut next_new_id = model.nodes.len() as u32;

    let mut rep_restraint_base = Dof6Mask::FREE;
    rep_restraint_base.set_fixed(Dof::Uz);
    rep_restraint_base.set_fixed(Dof::Rx);
    rep_restraint_base.set_fixed(Dof::Ry);

    for (si, &elev) in story_levels.iter().enumerate() {
        let story_id = StoryId(si as u32);
        let node_ids: Vec<NodeId> = std::mem::take(&mut nodes_by_story[si]);

        let prev = model
            .stories
            .iter()
            .find(|s| (s.elevation - elev).abs() <= LEVEL_TOL_MM);
        let name = prev
            .map(|s| s.name.clone())
            .unwrap_or_else(|| squid_n_core::model::default_story_name(si));
        let level_kind = prev.map(|s| s.level_kind).unwrap_or_default();
        let weight_override = prev.and_then(|s| s.weight_override);

        let weight: f64 = node_ids.iter().map(|n| node_weight[n.index()]).sum();

        let slaves: Vec<NodeId> = node_ids
            .iter()
            .copied()
            .filter(|n| (model.nodes[n.index()].coord[2] - elev).abs() <= DIAPHRAGM_LEVEL_TOL_MM)
            .collect();

        for n in &node_ids {
            node_story[n.index()] = Some(story_id);
        }

        if !slaves.is_empty() {
            let (gx, gy) = if weight > 0.0 {
                let gx = node_ids
                    .iter()
                    .map(|n| node_weight[n.index()] * model.nodes[n.index()].coord[0])
                    .sum::<f64>()
                    / weight;
                let gy = node_ids
                    .iter()
                    .map(|n| node_weight[n.index()] * model.nodes[n.index()].coord[1])
                    .sum::<f64>()
                    / weight;
                (gx, gy)
            } else {
                let gx = node_ids
                    .iter()
                    .map(|n| model.nodes[n.index()].coord[0])
                    .sum::<f64>()
                    / node_ids.len() as f64;
                let gy = node_ids
                    .iter()
                    .map(|n| model.nodes[n.index()].coord[1])
                    .sum::<f64>()
                    / node_ids.len() as f64;
                (gx, gy)
            };

            let master = reuse_masters.next().unwrap_or_else(|| {
                let id = NodeId(next_new_id);
                next_new_id += 1;
                id
            });

            let net_i = |idx: usize| -> f64 {
                match mass_method {
                    MassMethod::CorrectedLumped => {
                        (node_weight[idx] - node_self_weight[idx]).max(0.0)
                    }
                    MassMethod::LumpedOnly => node_weight[idx],
                }
            };
            let mt_weight: f64 = node_ids.iter().map(|n| net_i(n.index())).sum();
            let mass = if mt_weight > 0.0 {
                let mt = squid_n_core::units::to_internal::weight_n_to_mass(mt_weight);
                let j: f64 = node_ids
                    .iter()
                    .map(|n| {
                        let idx = n.index();
                        let mi = squid_n_core::units::to_internal::weight_n_to_mass(net_i(idx));
                        let dx = model.nodes[idx].coord[0] - gx;
                        let dy = model.nodes[idx].coord[1] - gy;
                        mi * (dx * dx + dy * dy)
                    })
                    .sum();
                Some([mt, mt, 0.0, 0.0, 0.0, j])
            } else {
                None
            };

            rep_nodes.push(Node {
                id: master,
                coord: [gx, gy, elev],
                restraint: master_restraint(model, rep_restraint_base, &slaves, &structural),
                mass,
                story: Some(story_id),
                support_spring: None,
            });
            generated_masters.push(master);

            constraints.push(Constraint::RigidDiaphragm {
                story: story_id,
                master,
                slaves,
                weight: Some(weight),
                ci_override: None,
            });
        }

        stories.push(Story {
            id: story_id,
            name,
            elevation: elev,
            node_ids,
            seismic_weight: Some(weight_override.unwrap_or(weight)),
            weight_override,
            structure: Default::default(),
            level_kind,
        });
    }

    if stories.is_empty() {
        return Err("階を構成する節点が見つかりませんでした。".into());
    }

    assign_story_structures(model, &node_story, &mut stories);

    for id in reuse_masters {
        rep_nodes.push(Node {
            id,
            coord: model.nodes[id.index()].coord,
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
            support_spring: None,
        });
        generated_masters.push(id);
    }

    Ok(StoryGenResult {
        stories,
        node_story,
        constraints,
        rep_nodes,
        generated_masters,
    })
}

/// 各階の主要構造種別（[`StoryStructure`]）を、その階に属する柱・梁の断面形状から
/// 判定して `stories` へ書き込む。
///
/// 略算周期 T = h(0.02+0.01α) の α は「柱及び梁の大部分が鉄骨造である階の高さの
/// 合計の比」（令88条・昭55建告1793号）であり、階ごとに柱・梁の構造種別が分かれば
/// 決まる。したがって利用者の入力ではなく断面形状から判定する。
///
/// - 対象部材: 線材（柱・梁・ブレース相当の [`ElementKind::Beam`]）のうち断面形状を
///   持つもの。形状定義のない断面（カタログ数値の直入力）は種別を決められないため
///   除外する。
/// - 部材の所属階: 材端節点のうち標高が最も高い節点の所属階
///   （階 i の階高は elevation_{i-1}〜elevation_i なので、その範囲の柱は上端が階 i に
///   属し、階 i のレベルにある梁も階 i に属する）。
/// - 階の種別: 対象部材の種別ごとの本数の最多（[`StoryStructure::majority`]）。
fn assign_story_structures(model: &Model, node_story: &[Option<StoryId>], stories: &mut [Story]) {
    use squid_n_core::model::StoryStructure;
    let mut counts: std::collections::HashMap<StoryId, (usize, usize, usize)> =
        std::collections::HashMap::with_capacity(stories.len());
    for e in &model.elements {
        if !matches!(e.kind, ElementKind::Beam) || e.nodes.len() < 2 {
            continue;
        }
        if e.section.is_none() {
            continue;
        }
        let top = e
            .nodes
            .iter()
            .filter_map(|nid| model.nodes.get(nid.index()))
            .max_by(|a, b| a.coord[2].total_cmp(&b.coord[2]));
        let Some(story) = top.and_then(|n| node_story.get(n.id.index()).copied().flatten()) else {
            continue;
        };
        let slot = counts.entry(story).or_default();
        match StoryStructure::of_structure_kind(
            squid_n_core::structure_kind::member_structure_kind(model, e),
        ) {
            StoryStructure::Rc => slot.0 += 1,
            StoryStructure::S => slot.1 += 1,
            StoryStructure::Src => slot.2 += 1,
        }
    }
    for story in stories.iter_mut() {
        let (n_rc, n_s, n_src) = counts.get(&story.id).copied().unwrap_or_default();
        story.structure = StoryStructure::majority(n_rc, n_s, n_src);
    }
}

/// 節点 Z 座標から階を自動生成する。
/// `gravity_lc` を渡した場合はその 1 ケースのみを地震用重量に算入する。
pub fn generate_stories(
    model: &Model,
    gravity_lc: Option<LoadCaseId>,
) -> Result<StoryGenResult, String> {
    let lcs: Vec<LoadCaseId> = gravity_lc.into_iter().collect();
    generate_stories_multi(model, &lcs)
}
