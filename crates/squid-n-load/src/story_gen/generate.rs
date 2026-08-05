//! 階（Story）生成の本体。
//!
//! - [`StoryGenResult`] — 生成結果（呼び出し側が [`Model`] へ適用する）
//! - [`generate_stories_multi`] — 複数重力荷重ケースを地震用重量に算入する階生成
//! - [`generate_stories`] — 単一ケース指定の従来互換ラッパー

use super::misc_wall::accumulate_misc_wall_weight;
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

/// 節点 Z 座標から階を自動生成する（複数の重力荷重ケースを地震用重量に算入する版）。
///
/// - 最下レベルは基部(支点レベル)とみなし階に含めない
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

/// [`generate_stories_multi`] の自重算入方法を選べる版。
///
/// `include_density_self_weight`:
/// - `true`: 従来どおり自重（柱梁・壁・ダンパー・フレーム外雑壁）を材料密度から
///   直接算入する（自重が重力ケースに含まれないモデル向け）。
/// - `false`: 密度からの自重・フレーム外雑壁の直接算入を行わず、`gravity_lcs` の
///   ケース内容だけを算入する。自重同期ケース
///   （[`crate::self_weight::self_weight_case_content`] の内容を含む「DL」）が
///   重力ケースに含まれるモデル向け（自重・雑壁ともケース側に含まれるため、
///   直接算入すると二重計上になる）。
///
/// `mass_method`: 剛床代表節点（マスター）へ与える質点質量（[`Node::mass`]）の
/// 算定方式（[`MassMethod`]）。`include_density_self_weight` の真偽によらず、
/// `CorrectedLumped` は解析の質量行列に部材密度質量として計上される自重
/// （主架構線材・壁パネル）を地震用重量から控除した残りを、`LumpedOnly` は
/// 地震用重量の全量をマスターの質点質量とする。
pub fn generate_stories_with_opts(
    model: &Model,
    gravity_lcs: &[LoadCaseId],
    include_density_self_weight: bool,
    mass_method: MassMethod,
) -> Result<StoryGenResult, String> {
    if model.nodes.is_empty() {
        return Err("節点がありません".into());
    }

    // --- 0. 構造節点の抽出（前回生成分の剛床代表節点を除外） ---
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

    // --- 1. Z レベルのクラスタリング（構造節点のみ） ---
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
            "節点の標高(Z)が 1 レベルしかありません。階を生成するには 2 レベル以上必要です。"
                .into(),
        );
    }

    let level_of = |z: f64| -> usize {
        levels
            .iter()
            .position(|&l| (z - l).abs() <= LEVEL_TOL_MM)
            .unwrap_or(0)
    };

    // 各レベルの所属節点を 1 パスでグルーピングする（階ごとに全節点を
    // 走査し直すと O(節点数×レベル数²) になるため）。
    let mut nodes_by_level: Vec<Vec<NodeId>> = vec![Vec::new(); levels.len()];
    for n in &struct_nodes {
        nodes_by_level[level_of(n.coord[2])].push(n.id);
    }

    // --- 2. 節点の重量配分 ---
    let mut node_weight = vec![0.0f64; model.nodes.len()];
    let load_cfg = model.load_cfg.clone().unwrap_or_default();

    // K型ブレースの重量配分（§K型ブレース）に用いる「基準節点」判定。
    // 基準節点＝ Brace 以外の要素が 1 つでも接続する節点。それ以外は「内部節点」。
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

    // §K型ブレース（BaseNodesOnly）: 総重量（または部材荷重の両端反力）を
    // 基準節点のみへ再配分する共通則。両端とも基準節点は各自の分をそのまま、
    // 片端が内部節点ならその分も基準節点側へ全量、両端とも内部節点は
    // フォールバックで元の配分のまま。
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

    // 自重（算定規則は enumerate_self_weight に一元化。§柱梁自重・§壁自重・§ダンパー自重）。
    // - 線材: 総重量を両端に半分ずつ（対称等分布荷重の静定反力。
    //   K型ブレースは §K型ブレースの規則で再配分）。
    // - ダンパー: 両端節点へ 1/2 ずつ（鉛直配置は上下階へ、水平配置は同一階の
    //   両節点へ、が節点標高から自然に成立する）。
    // - 壁・シェル: 頂点配分（三方スリットは最上位標高の頂点へ全量）。
    //
    // CorrectedLumped のマスター補正質点算定（後段）が「解析の質量行列に部材密度
    // 質量として計上される自重（線材・壁パネル）」の節点配分を必要とするため、
    // `include_density_self_weight` の真偽によらず列挙自体は常に行う。
    let self_weight_items = enumerate_self_weight(model, &load_cfg);

    // 線材（柱梁・ブレース）・壁パネルの自重を対象 Vec へ配分する（K型ブレースの
    // 再配分規則込み）。node_weight（地震用重量の合算）と node_self_weight
    // （解析質量行列に部材密度質量として計上される自重の控除用）の双方で使う。
    let distribute_line_panel = |target: &mut Vec<f64>, item: &SelfWeightItem| match item {
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
        SelfWeightItem::Panel { shares } => {
            for &(i, w) in shares {
                target[i] += w;
            }
        }
        SelfWeightItem::Damper { .. } | SelfWeightItem::SecondaryLine { .. } => {}
    };

    // §CorrectedLumped の控除対象: 解析の質量行列に部材密度質量として計上される
    // 要素（主架構の線材・壁パネル）の自重のみ。ダンパー・二次部材（小梁・間柱）・
    // フレーム外雑壁は解析質量に算入されない（assemble_global_m がダンパーの
    // mass_matrix を零で返し、二次部材・雑壁は model.elements にすら現れない）
    // ため控除しない。
    let mut node_self_weight = vec![0.0f64; model.nodes.len()];
    for item in &self_weight_items {
        distribute_line_panel(&mut node_self_weight, item);
    }

    // 自重が重力ケース（「DL」自動同期）側に含まれる場合は、地震用重量の合算
    // (node_weight) への直接算入を行わない（include_density_self_weight = false）。
    if include_density_self_weight {
        for item in &self_weight_items {
            match item {
                SelfWeightItem::Damper { ni, nj, total } => {
                    node_weight[*ni] += total / 2.0;
                    node_weight[*nj] += total / 2.0;
                }
                // 二次部材（小梁・間柱）: 両端節点へ 1/2 ずつ（節点は所属階の
                // レベルでクラスタリングされるため、階重量へ自然に算入される）。
                SelfWeightItem::SecondaryLine { ni, nj, total } => {
                    node_weight[*ni] += total / 2.0;
                    node_weight[*nj] += total / 2.0;
                }
                SelfWeightItem::Line { .. } | SelfWeightItem::Panel { .. } => {
                    distribute_line_panel(&mut node_weight, item);
                }
            }
        }

        // §フレーム外雑壁: 部材としてモデル化しない壁の重量を近傍節点へ集計する。
        // （false の場合は自重同期ケースの節点荷重に雑壁分が含まれるため行わない）
        accumulate_misc_wall_weight(model, &mut node_weight);
    }

    // 指定荷重ケース（複数可）の鉛直下向き成分。
    // §1.4: 部材荷重は単純梁の静定反力（`static_reactions`）で両端に配分する
    // （令88条の実務的取扱い: 地震用節点重量 = 大梁の CMoQo 計算による梁せん断力 Q0）。
    // 部材荷重 → 要素の解決は ID 添字マップで行う（荷重ごとの線形探索は
    // O(部材荷重数×要素数) になり、DL 自動同期モデルでは要素数の 2 乗で悪化する）。
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
            // 全体座標系の作用方向（正規化済み）の鉛直下向き成分
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
            // ブレースに載る鉛直荷重（自重同期ケースの等分布など）にも
            // §K型ブレースの配分規則を適用する（密度直接算入と同じ規約）。
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

    // --- 3. 階の構築（レベル 1 以上、下から順） ---
    let mut stories = Vec::new();
    let mut node_story = vec![None; model.nodes.len()];
    let mut constraints = Vec::new();
    let mut rep_nodes: Vec<Node> = Vec::new();
    let mut generated_masters: Vec<NodeId> = Vec::new();

    // 既存の代表節点は昇順（下の階から順）に再利用し、足りない分は末尾連番で新規生成する。
    let mut reuse_masters = model.generated_masters.iter().copied();
    let mut next_new_id = model.nodes.len() as u32;

    // 剛床代表節点の拘束: 要素が接続しない浮遊節点のため、剛床が拘束しない
    // 3 自由度（Uz, Rx, Ry）を固定しないと特異行列になる。Ux, Uy, Rz は自由。
    let mut rep_restraint = Dof6Mask::FREE;
    rep_restraint.set_fixed(Dof::Uz);
    rep_restraint.set_fixed(Dof::Rx);
    rep_restraint.set_fixed(Dof::Ry);

    for (si, &elev) in levels.iter().enumerate().skip(1) {
        let story_id = StoryId((si - 1) as u32);
        let node_ids: Vec<NodeId> = std::mem::take(&mut nodes_by_level[si]);
        if node_ids.is_empty() {
            continue;
        }

        let weight: f64 = node_ids.iter().map(|n| node_weight[n.index()]).sum();

        // 慣性力重心（重量重み付き重心）。重量が算定できない場合は幾何重心へフォールバック。
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

        // 剛床代表節点（慣性力重心に置く専用の仮想節点）の生成/再利用。
        let master = reuse_masters.next().unwrap_or_else(|| {
            let id = NodeId(next_new_id);
            next_new_id += 1;
            id
        });

        // マスターへ与える質点質量（mass_method による。§CorrectedLumped/LumpedOnly）。
        // 控除後重量 net_i:
        // - CorrectedLumped: 地震用重量から、解析の質量行列に部材密度質量として
        //   計上される自重（線材・壁パネル）を控除した残り（負にはしない）。
        // - LumpedOnly: 控除せず地震用重量そのもの。
        let net_i = |idx: usize| -> f64 {
            match mass_method {
                MassMethod::CorrectedLumped => (node_weight[idx] - node_self_weight[idx]).max(0.0),
                MassMethod::LumpedOnly => node_weight[idx],
            }
        };
        let mt_weight: f64 = node_ids.iter().map(|n| net_i(n.index())).sum();
        // 質点質量が算定できる階のみ設定する（Σnet_i ≦ 0 は None のまま）。
        let mass = if mt_weight > 0.0 {
            let mt = squid_n_core::units::to_internal::weight_n_to_mass(mt_weight);
            // 回転慣性 j = Σ(net_i/g)·r_i²（r_i はマスター座標 (gx,gy) からの平面距離）。
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
            restraint: rep_restraint,
            mass,
            story: Some(story_id),
            support_spring: None,
        });
        generated_masters.push(master);

        // 当該階の全構造節点がスレーブ（マスターは専用節点のため、既存節点は 1 点でも全てスレーブ）。
        let slaves = node_ids.clone();

        for n in &node_ids {
            node_story[n.index()] = Some(story_id);
        }
        constraints.push(Constraint::RigidDiaphragm {
            story: story_id,
            master,
            slaves: slaves.clone(),
        });
        stories.push(Story {
            id: story_id,
            name: format!("{}F", si),
            elevation: elev,
            node_ids,
            diaphragms: vec![DiaphragmDef {
                ci_override: None,
                master,
                slaves,
                rigid: true,
                weight: Some(weight),
            }],
            seismic_weight: Some(weight),
            weight_override: None,
            structure: Default::default(),
            level_kind: Default::default(),
        });
    }

    if stories.is_empty() {
        return Err("階を構成する節点が見つかりませんでした。".into());
    }

    // 主要構造種別は断面形状から自動判定する（利用者の入力項目ではない）。
    assign_story_structures(model, &node_story, &mut stories);

    // 階数が減って余った旧代表節点は不活性化する（拘束固定・所属階なし）が、
    // `generated_masters` には残して次回再生成時に再利用できるようにする。
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
    // 階ごとの (RC, S, SRC) 本数。`node_story` が持つのは `StoryId` であり、
    // それが `stories` 内の位置と一致する保証（`Model::validate` の
    // 「id == 添字」不変条件）に依存しないよう、`StoryId` をキーに集計する。
    let mut counts: std::collections::HashMap<StoryId, (usize, usize, usize)> =
        std::collections::HashMap::with_capacity(stories.len());
    for e in &model.elements {
        if !matches!(e.kind, ElementKind::Beam) || e.nodes.len() < 2 {
            continue;
        }
        // 断面も材料も未割当の部材は構造種別を判定できないため集計から除く。
        if e.section.is_none() && e.material.is_none() {
            continue;
        }
        // 材端節点のうち最も高い節点の所属階へ計上する。
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

/// 階の再生成で失われる利用者の手入力を、標高が一致する旧階から引き継ぐ。
///
/// 準備計算は毎回階を再生成する（節点・断面・荷重の変更を階の重量へ反映するため）
/// が、地震用重量の手入力（[`Story::weight_override`]）と階の種別
/// （[`squid_n_core::model::StoryLevelKind`]。PH・地下の指定は自動判定できない）は
/// 利用者の入力であり、再生成で失ってはならない。標高差が [`LEVEL_TOL_MM`] 以内の
/// 旧階を同じ階とみなして引き継ぐ（階数が変わっても残った階の入力は保たれる）。
///
/// 手入力の重量は `seismic_weight` へも反映するため、解析・設計側は
/// `seismic_weight` だけを読めばよい。
pub fn carry_over_manual_story_settings(prev: &[Story], stories: &mut [Story]) {
    for story in stories.iter_mut() {
        let Some(old) = prev
            .iter()
            .find(|o| (o.elevation - story.elevation).abs() <= LEVEL_TOL_MM)
        else {
            continue;
        };
        story.level_kind = old.level_kind;
        story.weight_override = old.weight_override;
        if let Some(w) = old.weight_override {
            story.seismic_weight = Some(w);
        }
    }
}

/// 節点 Z 座標から階を自動生成する（重力荷重ケース単一指定・従来互換の薄いラッパー）。
///
/// 詳細は [`generate_stories_multi`] を参照。`gravity_lc` を `Some` で渡した場合は
/// その 1 ケースのみを地震用重量に算入する（`None` は自重のみ）。
pub fn generate_stories(
    model: &Model,
    gravity_lc: Option<LoadCaseId>,
) -> Result<StoryGenResult, String> {
    let lcs: Vec<LoadCaseId> = gravity_lc.into_iter().collect();
    generate_stories_multi(model, &lcs)
}
