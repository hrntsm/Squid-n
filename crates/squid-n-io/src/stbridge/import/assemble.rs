//! パース済み中間表現から内部モデルを組み立てる（Import の後段）。
//!
//! [`StbParser`] が集めた file id 付きの中間表現（`Raw*` / `Pending*`）を受け取り、
//! id の 0 始まり連番への正規化・材料/断面の解決・部材/スラブ/壁/荷重ケースの構築・
//! 支点の自動設定を行い、モデルと [`ImportReport`] を返す。

use super::super::StbError;
use super::parser::{RawSlabSection, StbParser};
use super::{
    material_std, ImportReport, PendingMember, PendingMemberKind, PendingSec, PendingSecKind,
    PendingSecondary, RawAxisGroup, RawLoadCase, RawMaterial, RawNode, RawSlab, RawStory, RawWall,
    SecMatRef,
};
use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, SlabId, StoryId};
use squid_n_core::model::{
    DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis,
    Material, MaterialCategory, Model, NodalLoad, Node, Section, Slab, Story,
};
use squid_n_core::section_shape::SectionShape;
use std::collections::HashMap;

/// パース済み中間表現からモデルと取り込み報告を組み立てる（後段のエントリポイント）。
pub(super) fn assemble(parsed: StbParser) -> Result<(Model, ImportReport), StbError> {
    let StbParser {
        mut warnings,
        unsupported,
        attr_usage,
        raw_nodes,
        raw_stories,
        raw_materials,
        raw_load_cases,
        pending_secs,
        pending_members,
        pending_secondaries,
        steel_lib,
        raw_slabs,
        slab_secs,
        raw_walls,
        wall_sec_thickness,
        raw_axis_groups,
        ..
    } = parsed;

    let mut model = Model::default();

    // 各 id 空間を file id 昇順の 0 始まり連番へ正規化する（内部モデルの不変条件
    // 「配列添字 == id.index()」を満たすため）。返り値は file id → 新 index。
    let node_index = build_index(raw_nodes.iter().map(|n| n.file_id));
    let story_index = build_index(raw_stories.iter().map(|s| s.file_id));
    let material_index = build_index(raw_materials.iter().map(|m| m.file_id));

    // file id の一意性を検証する（fail-loud）。build_index は id を重複排除するが、
    // raw_* は排除しないまま push されるため、重複 id があると model.nodes 等の
    // 配列長が index 数を超え「配列添字 == id.index()」の不変条件が壊れ、部材が
    // 別実体の節点/断面/材料を無言で参照する（ジオメトリ破損）。重複はエラーとする。
    check_unique_ids("StbNode", raw_nodes.iter().map(|n| n.file_id), &node_index)?;
    check_unique_ids(
        "StbStory",
        raw_stories.iter().map(|s| s.file_id),
        &story_index,
    )?;
    check_unique_ids(
        "StbMaterial/StbSecColumn_S ほか材料",
        raw_materials.iter().map(|m| m.file_id),
        &material_index,
    )?;

    build_nodes_and_stories(&mut model, raw_nodes, raw_stories, &node_index);
    build_axes(&mut model, raw_axis_groups, &node_index);

    // 区分をグレード名から決められず、物性から推定した材料の名前。
    // 推定は外れることがあるため、取込後に notes で利用者へ通知する。
    let mut guessed_categories: Vec<String> = Vec::new();
    build_materials(
        &mut model,
        raw_materials,
        &material_index,
        &pending_secs,
        &mut guessed_categories,
    );

    let mut notes: Vec<String> = Vec::new();

    // 断面 id を整列・連番へ再割当てし、形鋼名を解決してモデルへ格納する。
    // 符号＋階が重複する定義の統合・改番もここで行う。
    let section_index = build_sections(
        &mut model,
        pending_secs,
        &steel_lib,
        &material_index,
        &mut warnings,
        &mut notes,
    );

    let mut stats = LinkStats::default();
    build_members(
        &mut model,
        pending_members,
        &node_index,
        &section_index,
        &material_index,
        &mut stats,
    );
    let (n_joists, n_posts) = build_secondaries(
        &mut model,
        pending_secondaries,
        &node_index,
        &section_index,
        &material_index,
        &mut stats,
    );
    stats.push_warnings(&mut warnings);

    let slab_section_count = build_slabs(
        &mut model,
        raw_slabs,
        &slab_secs,
        &node_index,
        &mut warnings,
    );
    build_walls(
        &mut model,
        raw_walls,
        &wall_sec_thickness,
        &node_index,
        &material_index,
        &mut warnings,
    );
    build_load_cases(&mut model, raw_load_cases, &node_index, &mut warnings);
    warn_unsupported(&unsupported, &mut warnings);

    push_import_notes(
        &mut notes,
        guessed_categories,
        n_joists,
        n_posts,
        slab_section_count,
    );
    auto_assign_supports(&mut model, &mut notes);

    let attributes = attr_dispositions(attr_usage);
    Ok((
        model,
        ImportReport {
            warnings,
            notes,
            attributes,
        },
    ))
}

/// 属性の扱いの集計を、要素名・属性名の昇順に整列した報告へ変換する。
/// 整列は出力の決定性のため（`HashMap` の走査順は不定）。
fn attr_dispositions(
    usage: HashMap<(String, String), super::parser::AttrCount>,
) -> Vec<super::AttrDisposition> {
    let mut v: Vec<super::AttrDisposition> = usage
        .into_iter()
        .map(|((element, attribute), c)| super::AttrDisposition {
            element,
            attribute,
            count: c.total,
            imported: c.imported,
        })
        .collect();
    v.sort_by(|a, b| {
        a.element
            .cmp(&b.element)
            .then_with(|| a.attribute.cmp(&b.attribute))
    });
    v
}

/// 節点と階を id 正規化してモデルへ格納する（階所属の解決・補完を含む）。
fn build_nodes_and_stories(
    model: &mut Model,
    mut raw_nodes: Vec<RawNode>,
    mut raw_stories: Vec<RawStory>,
    node_index: &HashMap<u32, u32>,
) {
    // 実 ST-Bridge の階所属（StbStory/StbNodeIdList）から file node id → file story id を作る。
    // 節点の所属階は、まず節点自身の `story` 属性（Squid 方言）を優先し、なければこの表を引く。
    let node_story_from_list: HashMap<u32, u32> = raw_stories
        .iter()
        .flat_map(|s| {
            let sid = s.file_id;
            s.node_ids.iter().map(move |&nid| (nid, sid))
        })
        .collect();

    // `Model::stories` は標高（`elevation`）の昇順という不変条件を持つ（階への帰属区間が
    // 直下階のレベルで決まるため、並びが崩れると帰属が壊れる）。ST-Bridge の `StbStory` は
    // 標高順に並んでいる保証がないため、ここで並べ替えたうえで `StoryId` を振り直す。
    // 標高が同じ階は file id の昇順で安定させる。
    raw_stories.sort_by(|a, b| {
        a.elevation
            .total_cmp(&b.elevation)
            .then(a.file_id.cmp(&b.file_id))
    });
    // file id → 標高昇順での位置（`StoryId` ＝配列位置の不変条件を満たす）。
    let story_rank: HashMap<u32, u32> = raw_stories
        .iter()
        .enumerate()
        .map(|(i, s)| (s.file_id, i as u32))
        .collect();
    for s in raw_stories {
        // 階の所属節点を正規化後の NodeId へ解決する（存在しない節点は除外）。
        let node_ids = s
            .node_ids
            .iter()
            .filter_map(|fid| node_index.get(fid).copied().map(NodeId))
            .collect();
        model.stories.push(Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(story_rank[&s.file_id]),
            name: s.name,
            elevation: s.elevation,
            node_ids,
            seismic_weight: None,
            weight_override: None,
        });
    }

    raw_nodes.sort_by_key(|n| n.file_id);
    for n in raw_nodes {
        model.nodes.push(Node {
            id: NodeId(node_index[&n.file_id]),
            coord: n.coord,
            restraint: squid_n_core::dof::Dof6Mask::FREE,
            mass: None,
            // 節点の所属階は `StbStory/StbNodeIdList` から引く（標準スキーマ）。
            story: node_story_from_list
                .get(&n.file_id)
                .and_then(|sfid| story_rank.get(sfid).copied())
                .map(StoryId),
            support_spring: None,
        });
    }

    // 念のため、節点の story から Story.node_ids を補完する
    // （StbNodeIdList 由来との重複は除く）。
    for node in &model.nodes {
        if let Some(sid) = node.story {
            let list = &mut model.stories[sid.index()].node_ids;
            if !list.contains(&node.id) {
                list.push(node.id);
            }
        }
    }
}

/// 通り芯グループを id 正規化してモデルへ格納する（`Manual` 扱い・離れの昇順に整列）。
fn build_axes(
    model: &mut Model,
    raw_axis_groups: Vec<RawAxisGroup>,
    node_index: &HashMap<u32, u32>,
) {
    // 通り芯。所属節点の file id を正規化後の NodeId へ張り替える。取り込んだ通りは
    // 利用者の入力と同格の `Manual` とし、柱位置からの自動生成で作り直さない
    // （通り名・芯ずれした所属は自動生成では復元できないため）。
    for g in raw_axis_groups {
        let axes: Vec<squid_n_core::model::Axis> = g
            .axes
            .into_iter()
            .map(|ax| {
                let mut nodes: Vec<NodeId> = ax
                    .node_ids
                    .iter()
                    .filter_map(|fid| node_index.get(fid).copied().map(NodeId))
                    .collect();
                nodes.sort();
                nodes.dedup();
                squid_n_core::model::Axis {
                    name: ax.name,
                    distance: ax.distance,
                    nodes,
                    source: squid_n_core::model::AxisSource::Manual,
                }
            })
            .collect();
        model.axes.push(squid_n_core::model::AxisGroup {
            name: g.name,
            kind: g.kind,
            axes,
        });
    }
    // 平行芯グループは離れの昇順に保つ（一覧表示・書き出しが座標順になる）。
    for group in &mut model.axes {
        group.sort_axes();
    }
}

/// 材料を id 正規化して格納し、断面のグレード名参照を標準材料表から材料として補う。
fn build_materials(
    model: &mut Model,
    mut raw_materials: Vec<RawMaterial>,
    material_index: &HashMap<u32, u32>,
    pending_secs: &[PendingSec],
    guessed_categories: &mut Vec<String>,
) {
    raw_materials.sort_by_key(|m| m.file_id);
    for m in raw_materials {
        let category = resolve_material_category(&m.name, m.fc, m.fy, guessed_categories);
        model.materials.push(Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(material_index[&m.file_id]),
            name: m.name,
            category,
            young: m.young,
            poisson: m.poisson,
            density: m.density,
            shear: m.shear,
            fc: m.fc,
            fy: m.fy,
        });
    }

    // ST-Bridge 2.0 の StbModel は材料テーブル（E・ν・密度）を持たず、材料は断面に付く
    // グレード名（コンクリート `Fc21`、鋼種 `SN400B`、鉄筋 `SD345` 等）で表す。日本の
    // 構造材料は規格化されており名前が物性を一意に定めるため、断面が参照するグレード名を
    // 標準材料表で物性へ解決し、同名の材料がまだなければ材料として追加する。
    {
        use std::collections::HashSet;
        let mut existing: HashSet<String> =
            model.materials.iter().map(|m| m.name.clone()).collect();
        // 文書順で決定的に列挙し、重複名は最初の 1 回だけ追加する。
        let mut grades: Vec<&str> = Vec::new();
        for p in pending_secs {
            if let Some(SecMatRef::Grade(name)) = &p.mat {
                if !name.is_empty() && !grades.contains(&name.as_str()) {
                    grades.push(name.as_str());
                }
            }
        }
        for name in grades {
            if existing.contains(name) {
                continue;
            }
            if let Some(std) = material_std::resolve_grade(name) {
                let id = MaterialId(model.materials.len() as u32);
                let category = resolve_material_category(name, std.fc, std.fy, guessed_categories);
                model.materials.push(Material {
                    strength_factor: None,
                    concrete_class: Default::default(),
                    id,
                    name: name.to_string(),
                    category,
                    young: std.young,
                    poisson: std.poisson,
                    density: std.density,
                    shear: None,
                    fc: std.fc,
                    fy: std.fy,
                });
                existing.insert(name.to_string());
            }
        }
    }
}

/// 断面側の材料参照を、部材への伝播用に file id → 正規化後 material index へ解決する。
///
/// ST-Bridge は材料を断面に持つため、部材が id_material を持たない（実 STB 相当の）
/// 場合に断面の材料を部材へ伝播する。数値 id は material_index、鋼のグレード名は
/// 同名の材料へ突き合わせる（同名複数は最初の一致）。
/// 部材・二次部材の参照解決の失敗件数（まとめて警告へ変換するための集計）。
#[derive(Default)]
struct LinkStats {
    /// 存在しない節点を参照しスキップした部材数。
    skipped_members: u32,
    /// 存在しない断面を参照し断面リンクを外した部材数。
    dangling_section: u32,
    /// 存在しない材料を参照し材料リンクを外した部材数。
    dangling_material: u32,
    /// 断面に既に付いている材料と違う材料を指した部材数（先に付いた方を採る）。
    conflicting_material: u32,
}

impl LinkStats {
    /// 集計した参照解決の失敗を警告メッセージへ変換する。
    fn push_warnings(&self, warnings: &mut Vec<String>) {
        if self.skipped_members > 0 {
            warnings.push(format!(
                "存在しない節点を参照する部材を {} 件スキップしました",
                self.skipped_members
            ));
        }
        if self.dangling_section > 0 {
            warnings.push(format!(
                "存在しない断面を参照する部材が {} 件あり、断面リンクを外しました",
                self.dangling_section
            ));
        }
        if self.dangling_material > 0 {
            warnings.push(format!(
                "存在しない材料を参照する部材が {} 件あり、材料リンクを外しました",
                self.dangling_material
            ));
        }
        if self.conflicting_material > 0 {
            warnings.push(format!(
                "同じ断面を参照する部材が別々の材料を指しています（{} 件）。\
                 材料は断面が持つため先に解決した材料を採りました。\
                 違う材料を使う部材は断面を分けてください",
                self.conflicting_material
            ));
        }
    }
}

/// 部材（柱・大梁・ブレース）を格納する（節点・断面・材料の参照を正規化後の index に
/// 張り替える）。参照先が存在しない部材はスキップし、断面/材料の欠落は None にして
/// ダングリングを防ぐ。
fn build_members(
    model: &mut Model,
    pending_members: Vec<PendingMember>,
    node_index: &HashMap<u32, u32>,
    section_index: &HashMap<u32, u32>,
    material_index: &HashMap<u32, u32>,
    stats: &mut LinkStats,
) {
    for m in pending_members {
        let (Some(&ni), Some(&nj)) = (node_index.get(&m.n_i), node_index.get(&m.n_j)) else {
            stats.skipped_members += 1;
            continue;
        };
        // 断面参照: 実在しない id を指していれば警告して None にする（ダングリング防止）。
        let section = m.section.and_then(|fid| match section_index.get(&fid) {
            Some(&idx) => Some(SectionId(idx)),
            None => {
                stats.dangling_section += 1;
                None
            }
        });
        // 材料は断面が持つため、部材の `id_material` は断面へ移す。断面がグレード名を
        // 持たず部材だけが材料を指すファイル（実 ST-Bridge に多い）でも材料が失われない。
        // 同じ断面を指す部材が別々の材料を指す場合は、最初の 1 件を採る。
        if let Some(fid) = m.material {
            match material_index.get(&fid) {
                Some(&idx) => {
                    if let Some(sec) = section.and_then(|sid| model.sections.get_mut(sid.index())) {
                        // 既に別の材料が付いている断面は上書きしない（先勝ち）。
                        // 黙って捨てると材料の食い違いに気づけないため件数を数える。
                        match sec.material {
                            Some(existing) if existing != MaterialId(idx) => {
                                stats.conflicting_material += 1;
                            }
                            Some(_) => {}
                            None => sec.material = Some(MaterialId(idx)),
                        }
                    }
                }
                None => stats.dangling_material += 1,
            }
        }
        let id = ElemId(model.elements.len() as u32);
        // 梁・柱は端部接合条件（`condition_*`）を尊重し、ブレースは軸材なので両端ピン。
        let (kind, end_cond) = match m.kind {
            PendingMemberKind::Beam => (ElementKind::Beam, m.end_cond),
            PendingMemberKind::Brace { tension_only } => (
                ElementKind::Brace { tension_only },
                [EndCondition::Pinned, EndCondition::Pinned],
            ),
        };
        // ref_vector は部材軸（節点座標）と `rotate` から算出する。
        let ref_vector = ref_vector_from_rotate(
            model.nodes[ni as usize].coord,
            model.nodes[nj as usize].coord,
            m.rotate,
        );
        model.elements.push(ElementData {
            id,
            kind,
            nodes: smallvec::smallvec![NodeId(ni), NodeId(nj)],
            section,
            local_axis: LocalAxis { ref_vector },
            end_cond,
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }
}

/// 二次部材（小梁・間柱）を格納する（参照の正規化・伝播は部材と同じ規則）。
/// 全体解析の対象外（CMQ 用）のため `model.elements` には入れず
/// `model.secondary_members` に入れる。返り値は（小梁数, 間柱数）。
fn build_secondaries(
    model: &mut Model,
    pending_secondaries: Vec<PendingSecondary>,
    node_index: &HashMap<u32, u32>,
    section_index: &HashMap<u32, u32>,
    material_index: &HashMap<u32, u32>,
    stats: &mut LinkStats,
) -> (usize, usize) {
    let mut n_joists = 0usize;
    let mut n_posts = 0usize;
    for s in pending_secondaries {
        let (Some(&ni), Some(&nj)) = (node_index.get(&s.n_i), node_index.get(&s.n_j)) else {
            stats.skipped_members += 1;
            continue;
        };
        let section = s.section.and_then(|fid| match section_index.get(&fid) {
            Some(&idx) => Some(SectionId(idx)),
            None => {
                stats.dangling_section += 1;
                None
            }
        });
        // 材料は断面が持つ。二次部材の `id_material` も部材と同じく断面へ移す。
        if let Some(fid) = s.material {
            match material_index.get(&fid) {
                Some(&idx) => {
                    if let Some(sec) = section.and_then(|sid| model.sections.get_mut(sid.index())) {
                        match sec.material {
                            Some(existing) if existing != MaterialId(idx) => {
                                stats.conflicting_material += 1;
                            }
                            Some(_) => {}
                            None => sec.material = Some(MaterialId(idx)),
                        }
                    }
                }
                None => stats.dangling_material += 1,
            }
        }
        match s.kind {
            squid_n_core::model::SecondaryMemberKind::Joist => n_joists += 1,
            squid_n_core::model::SecondaryMemberKind::Post => n_posts += 1,
        }
        model
            .secondary_members
            .push(squid_n_core::model::SecondaryMember {
                id: squid_n_core::ids::SecondaryMemberId(model.secondary_members.len() as u32),
                kind: s.kind,
                nodes: [NodeId(ni), NodeId(nj)],
                section,
                name: s.name,
            });
    }
    (n_joists, n_posts)
}

/// スラブ（StbSlab）を格納する（境界の正規化・厚さ解決・自重の自動設定）。
/// 返り値は自重を設定したスラブ数（notes 通知用）。
///
/// 3 頂点未満・存在しない節点を含むスラブはスキップして報告する。
///
/// スラブ断面（`StbSecSlab_RC` / `StbSecSlabDeck`）は符号・階・板厚・コンクリート
/// のグレード名を持つため、内部の [`Section`] として組み立ててスラブへ割り当てる。
/// **自重は面荷重へ焼き込まない**。板厚と材料が断面にそろっているので、自重は
/// 使うたびに `Model::slab_self_weight_intensity` が算定する（板厚や材料を
/// 変えたときに自重が追随しない食い違いを作らないため）。
/// 仕上げ荷重・用途（積載）は ST-Bridge が持たないため、荷重タブでの設定が要る。
fn build_slabs(
    model: &mut Model,
    raw_slabs: Vec<RawSlab>,
    slab_secs: &HashMap<u32, RawSlabSection>,
    node_index: &HashMap<u32, u32>,
    warnings: &mut Vec<String>,
) -> usize {
    let mut skipped_slabs = 0u32;
    // ST-Bridge の XML には「スラブと小梁の親子関係」を明示する要素がない。
    // そのため `Slab.secondary_joist_ids` は現行 importer では空配列のままにし、
    // ここでは関連付けを行わない（旧スキーマの自動補正と区別してスキップ）。
    // ST-Bridge の将来拡張で、親子関係が明示されればそのときに収集する。
    // ST-Bridge の断面 file id → 内部の断面 ID。同じ断面を指すスラブで使い回す。
    let mut sec_of_file: HashMap<u32, SectionId> = HashMap::new();
    let mut slab_section_count = 0usize;
    for rs in raw_slabs {
        let mut boundary = Vec::with_capacity(rs.boundary.len());
        let mut resolved = true;
        for fid in &rs.boundary {
            match node_index.get(fid) {
                Some(&ni) => boundary.push(NodeId(ni)),
                None => {
                    resolved = false;
                    break;
                }
            }
        }
        if !resolved || boundary.len() < 3 {
            skipped_slabs += 1;
            continue;
        }
        let section = rs.section_fid.and_then(|fid| {
            let raw = slab_secs.get(&fid)?;
            if raw.thickness <= 0.0 {
                return None;
            }
            if let Some(&sid) = sec_of_file.get(&fid) {
                return Some(sid);
            }
            let sid = push_slab_section(model, fid, raw);
            sec_of_file.insert(fid, sid);
            slab_section_count += 1;
            Some(sid)
        });
        let new_id = SlabId(model.slabs.len() as u32);
        model.slabs.push(Slab {
            id: new_id,
            boundary,
            joists: Vec::new(),
            loads: Vec::new(),
            method: DistributionMethod::TriTrapezoid,
            kind: Default::default(),
            one_way: None,
            edge_supported: None,
            usage: None,
            section,
            secondary_joist_ids: Vec::new(),
        });
    }
    if skipped_slabs > 0 {
        warnings.push(format!(
            "境界節点が解決できない、または頂点数が不足するスラブを {skipped_slabs} 件スキップしました"
        ));
    }
    slab_section_count
}

/// 取り込んだスラブ断面を内部の [`Section`] として末尾へ追加し、その ID を返す。
///
/// 符号は `name` 属性、無ければ `S{file_id}`。階は `floor` 属性をそのまま持つ
/// （断面の同一性は符号＋階のため）。符号＋階が既存の断面と衝突する場合は
/// 空いた符号まで連番を送る。コンクリートは `strength_concrete` のグレード名から
/// 材料を引き当て、無ければ標準材料表から起こして追加する。
fn push_slab_section(model: &mut Model, file_id: u32, raw: &RawSlabSection) -> SectionId {
    use squid_n_core::section_shape::SectionShape;

    let base = raw
        .name
        .clone()
        .filter(|n| !n.is_empty())
        .unwrap_or_else(|| format!("S{file_id}"));
    let floor = raw.floor.clone();
    let mut name = base.clone();
    let mut n = 2u32;
    while squid_n_core::model::section_key_taken(&model.sections, (&name, floor.as_deref()), None) {
        name = format!("{base}#{n}");
        n += 1;
    }
    let sid = SectionId(model.sections.len() as u32);
    let mut sec = SectionShape::RcSlab {
        thickness: raw.thickness,
    }
    .to_section(sid, name);
    sec.floor = floor;
    sec.material = raw
        .concrete
        .as_deref()
        .and_then(|grade| ensure_material_by_grade(model, grade));
    model.sections.push(sec);
    sid
}

/// グレード名の材料を探し、無ければ標準材料表から起こして追加する。
/// 標準表にも無い名前（`Fc21` のような規格名でないもの）は `None`。
fn ensure_material_by_grade(model: &mut Model, grade: &str) -> Option<MaterialId> {
    if let Some(m) = model.materials.iter().find(|m| m.name == grade) {
        return Some(m.id);
    }
    let std = material_std::resolve_grade(grade)?;
    let id = MaterialId(model.materials.len() as u32);
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id,
        name: grade.to_string(),
        category: squid_n_core::material_grade::category_of_grade(grade)
            .unwrap_or(MaterialCategory::Concrete),
        young: std.young,
        poisson: std.poisson,
        density: std.density,
        shear: None,
        fc: std.fc,
        fy: std.fy,
    });
    Some(id)
}

/// 壁（StbWall）を壁要素（`ElementKind::Wall`）として格納する。
///
/// 厚さ（StbSecWall_RC）は t>0 のとき厚さ専用の Section を末尾に追加して参照する
/// （壁自重は section.thickness を用いるため）。断面は**厚さごとに 1 件**とし、同じ厚さの
/// 壁が何枚あっても使い回す（断面の同一性キー「符号＋階」を壁でも一意に保つため。
/// 壁ごとに断面を作ると `Wall t180` が枚数分並び、断面一覧が実態と合わなくなる）。
/// 3頂点未満・存在しない節点を含む壁はスキップして報告する。
fn build_walls(
    model: &mut Model,
    raw_walls: Vec<RawWall>,
    wall_sec_thickness: &HashMap<u32, f64>,
    node_index: &HashMap<u32, u32>,
    material_index: &HashMap<u32, u32>,
    warnings: &mut Vec<String>,
) {
    // ST-Bridge の XML には「壁と間柱の親子関係」を明示する要素がない。
    // 壁要素単体では `WallRegion` の `wall` と `post_ids` を紐づけられず、
    // そのため `wall_regions` は空配列のままにする（現行 importer ではスキップ）。
    // 将来、親子関係を表す要素が見つかればここで `WallRegion` を生成する。
    let mut skipped_walls = 0u32;
    // 壁厚 → 生成済みの厚さ専用断面。同じ厚さの壁で断面を使い回すための索引。
    // f64 は Hash を持たないため、符号（`Wall t180`）そのものをキーにする。
    let mut wall_sections: HashMap<String, SectionId> = HashMap::new();
    for rw in raw_walls {
        let mut boundary: smallvec::SmallVec<[NodeId; 8]> =
            smallvec::SmallVec::with_capacity(rw.boundary.len());
        let mut resolved = true;
        for fid in &rw.boundary {
            match node_index.get(fid) {
                Some(&ni) => boundary.push(NodeId(ni)),
                None => {
                    resolved = false;
                    break;
                }
            }
        }
        if !resolved || boundary.len() < 3 {
            skipped_walls += 1;
            continue;
        }
        // 厚さ >0 のときのみ厚さ専用断面を参照する（同じ厚さなら既存の断面を使い回す）。
        let section = rw
            .section_fid
            .and_then(|fid| wall_sec_thickness.get(&fid).copied())
            .filter(|t| *t > 0.0)
            .map(|t| {
                let base = format!("Wall t{}", t);
                if let Some(&sid) = wall_sections.get(&base) {
                    return sid;
                }
                // 断面定義側に同名の断面があるとキーが衝突するため、空いた符号まで送る
                // （壁の厚さ断面は階を持たないので、符号だけで一意になればよい）。
                let mut name = base.clone();
                let mut n = 2u32;
                while squid_n_core::model::section_key_taken(&model.sections, (&name, None), None) {
                    name = format!("{base}#{n}");
                    n += 1;
                }
                let sid = SectionId(model.sections.len() as u32);
                model.sections.push(Section {
                    id: sid,
                    name: name.clone(),
                    area: 0.0,
                    iy: 0.0,
                    iz: 0.0,
                    j: 0.0,
                    depth: 0.0,
                    width: 0.0,
                    as_y: 0.0,
                    as_z: 0.0,
                    floor: None,
                    panel_thickness: None,
                    thickness: Some(t),
                    shape: None,
                    material: None,
                    rebar_material: None,
                    shear_rebar_material: None,
                    steel_material: None,
                });
                wall_sections.insert(base, sid);
                sid
            });
        // 壁の材料も断面が持つ（要素は持たない）。断面側が未設定なら補う。
        if let Some(mid) = rw
            .material_fid
            .and_then(|fid| material_index.get(&fid).copied())
            .map(MaterialId)
        {
            if let Some(sec) = section.and_then(|sid| model.sections.get_mut(sid.index())) {
                if sec.material.is_none() {
                    sec.material = Some(mid);
                }
            }
        }
        let id = ElemId(model.elements.len() as u32);
        model.elements.push(ElementData {
            id,
            kind: ElementKind::Wall,
            nodes: boundary,
            section,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }
    if skipped_walls > 0 {
        warnings.push(format!(
            "境界節点が解決できない、または頂点数が不足する壁を {skipped_walls} 件スキップしました"
        ));
    }
}

/// 荷重ケースを格納する（節点参照を正規化。存在しない節点への荷重は破棄して報告）。
fn build_load_cases(
    model: &mut Model,
    raw_load_cases: Vec<RawLoadCase>,
    node_index: &HashMap<u32, u32>,
    warnings: &mut Vec<String>,
) {
    let mut dropped_loads = 0u32;
    for (i, lc) in raw_load_cases.into_iter().enumerate() {
        let nodal = lc
            .nodal
            .into_iter()
            .filter_map(|(fid, values)| match node_index.get(&fid) {
                // 取り込んだ荷重は利用者の入力として扱う（準備計算の同期対象に
                // しない。`StbNodalLoad` に名称属性はないため名称は空）。
                Some(&ni) => Some(NodalLoad::manual(NodeId(ni), values)),
                None => {
                    dropped_loads += 1;
                    None
                }
            })
            .collect();
        model.load_cases.push(LoadCase {
            kind: Default::default(),
            id: LoadCaseId(i as u32),
            name: lc.name,
            nodal,
            member: vec![],
        });
    }
    if dropped_loads > 0 {
        warnings.push(format!(
            "存在しない節点への節点荷重を {dropped_loads} 件破棄しました"
        ));
    }
}

/// 未対応要素の集計を 1 行の警告にまとめる（タグ名昇順で決定的に）。
fn warn_unsupported(unsupported: &HashMap<String, u32>, warnings: &mut Vec<String>) {
    if !unsupported.is_empty() {
        let mut items: Vec<(&String, &u32)> = unsupported.iter().collect();
        items.sort_by(|a, b| a.0.cmp(b.0));
        let list = items
            .iter()
            .map(|(tag, n)| format!("{tag}×{n}"))
            .collect::<Vec<_>>()
            .join(", ");
        warnings.push(format!("未対応の要素をスキップしました: {list}"));
    }
}

/// 取り込み時の自動補完・二次部材化の通知（notes）を積む。
fn push_import_notes(
    notes: &mut Vec<String>,
    mut guessed_categories: Vec<String>,
    n_joists: usize,
    n_posts: usize,
    slab_section_count: usize,
) {
    if !guessed_categories.is_empty() {
        guessed_categories.sort();
        guessed_categories.dedup();
        notes.push(format!(
            "材料 {} の区分をグレード名から決められないため、物性から推定しました\
            （区分は構造種別の判定に使います。「材料」タブで確認してください）",
            guessed_categories.join("・")
        ));
    }
    if n_joists + n_posts > 0 {
        notes.push(format!(
            "小梁 {n_joists} 本・間柱 {n_posts} 本を二次部材として取り込みました\
            （全体解析の対象外。床荷重・自重は大梁への集中荷重（CMQ）として伝達します）"
        ));
    }
    if slab_section_count > 0 {
        notes.push(format!(
            "スラブ断面 {slab_section_count} 件を取り込み、床へ割り当てました\
            （自重は断面の板厚と材料から算定します。仕上げ荷重・用途（積載）は\
            ST-Bridge に含まれないため、荷重タブで設定してください）"
        ));
    }
}

/// 支点の自動設定: ST-Bridge は境界条件（支点）を持たないため、支点が 1 つも
/// ないモデルは最下レベル（Z 最小、許容差 1mm）で柱脚を持つ節点をピン支点
/// （並進固定・回転自由）に設定する（柱脚ピンの仮定＝基礎の回転拘束を
/// 期待しない安全側の既定。解析可能な出発点にする）。
///
/// 柱が取り付かず梁だけが取り付く最下レベル節点（地中梁の中間節点など）は
/// 支点にしない。仮定した内容は notes で通知する。拘束を 1 つでも持つモデル
/// （将来の方言拡張等で取り込んだ場合）はそのまま尊重して何もしない。
fn auto_assign_supports(model: &mut Model, notes: &mut Vec<String>) {
    use squid_n_core::dof::Dof6Mask;
    if !model.nodes.is_empty() && model.nodes.iter().all(|n| n.restraint == Dof6Mask::FREE) {
        const BASE_LEVEL_TOL_MM: f64 = 1.0;
        let z_min = model
            .nodes
            .iter()
            .map(|n| n.coord[2])
            .fold(f64::INFINITY, f64::min);

        // 柱脚が取り付く節点の集合を求める。柱＝鉛直な 2 節点 Beam 要素
        // （全クレート共通の 45° 余弦基準 `squid_n_core::geom::is_vertical_axis`）。
        // その下端節点（Z が小さい方）を柱脚候補とする。
        let mut column_base: std::collections::HashSet<usize> = std::collections::HashSet::new();
        for elem in &model.elements {
            if elem.kind != ElementKind::Beam || elem.nodes.len() != 2 {
                continue;
            }
            let (a, b) = (elem.nodes[0].index(), elem.nodes[1].index());
            if a >= model.nodes.len() || b >= model.nodes.len() {
                continue;
            }
            let (pa, pb) = (model.nodes[a].coord, model.nodes[b].coord);
            if !squid_n_core::geom::is_vertical_axis(pa, pb) {
                continue; // 長さ 0 または水平材（梁）は柱ではない
            }
            let bottom = if pa[2] <= pb[2] { a } else { b };
            column_base.insert(bottom);
        }

        // 最下レベルかつ柱脚が取り付く節点だけをピン支点にする。
        let mut fixed = 0usize;
        for (i, n) in model.nodes.iter_mut().enumerate() {
            if (n.coord[2] - z_min).abs() <= BASE_LEVEL_TOL_MM && column_base.contains(&i) {
                n.restraint = Dof6Mask::PINNED;
                fixed += 1;
            }
        }

        if fixed > 0 {
            notes.push(format!(
                "支点情報がないため、最下レベル（Z={z_min:.0} mm）で柱が取り付く節点 {fixed} 箇所をピン支点に設定しました（モデルタブ→境界条件で変更できます）"
            ));
        } else {
            // 最下レベルに柱脚が 1 つもない（柱が全くない／柱脚が最下レベルに
            // 達しない）場合は、解析可能性を優先して従来どおり最下レベルの全節点を
            // ピン支点にフォールバックする。
            for n in &mut model.nodes {
                if (n.coord[2] - z_min).abs() <= BASE_LEVEL_TOL_MM {
                    n.restraint = Dof6Mask::PINNED;
                    fixed += 1;
                }
            }
            if fixed > 0 {
                notes.push(format!(
                    "支点情報がないため、最下レベル（Z={z_min:.0} mm）の節点 {fixed} 箇所をピン支点に設定しました（柱脚が特定できなかったため全節点。モデルタブ→境界条件で変更できます）"
                ));
            }
        }
    }
}

/// file id が一意であることを検証する（fail-loud）。要素数が重複排除後の
/// index 数を超えていれば重複 id ありとしてエラーを返す。
fn check_unique_ids(
    kind: &str,
    ids: impl Iterator<Item = u32>,
    index: &HashMap<u32, u32>,
) -> Result<(), StbError> {
    let count = ids.count();
    if count > index.len() {
        return Err(StbError::Parse(format!(
            "{kind} の file id が重複しています（{count} 要素に対し一意 id は {} 個）。\
             id は一意である必要があります。",
            index.len()
        )));
    }
    Ok(())
}

/// ST-Bridge の材料の区分を決める。
///
/// ST-Bridge はグレード名で材料を表すため、区分はまずグレード名から決める。
/// 名前から決まらない場合は物性から推定し、推定した材料の名前を `guessed` へ
/// 積む（取込後に notes で利用者へ通知するため）。
///
/// 推定は `Fc` を持つものをコンクリート、`Fy` だけを持つものを鋼材とし、
/// どちらもなければコンクリートとする。区分を誤って鋼材にすると RC 部材が
/// 鋼の検定式・鋼の Mp 式で評価されて危険側になるため、判断がつかない場合は
/// コンクリート側へ寄せる。
fn resolve_material_category(
    name: &str,
    fc: Option<f64>,
    fy: Option<f64>,
    guessed: &mut Vec<String>,
) -> MaterialCategory {
    if let Some(category) = squid_n_core::material_grade::category_of_grade(name) {
        return category;
    }
    guessed.push(name.to_string());
    match (fc, fy) {
        (Some(_), _) => MaterialCategory::Concrete,
        (None, Some(_)) => MaterialCategory::Steel,
        (None, None) => MaterialCategory::Concrete,
    }
}

/// file id の集合を昇順・重複排除して 0 始まり連番へ写像する（file id → 新 index）。
fn build_index(ids: impl Iterator<Item = u32>) -> HashMap<u32, u32> {
    let mut sorted: Vec<u32> = ids.collect();
    sorted.sort_unstable();
    sorted.dedup();
    sorted
        .into_iter()
        .enumerate()
        .map(|(i, id)| (id, i as u32))
        .collect()
}

/// 保留していた断面を id 昇順に整列・連番へ再割当てし、形鋼名を解決して
/// `model.sections` を構築する。返り値は 元の file id → 再割当て後 index のマップ。
///
/// ST-Bridge は断面の一意キーが `guid` で、同じ符号の断面を階ごとに別定義として持つ。
/// Squid-n の断面は**符号＋階**が一意キーなので、キーが衝突した定義はここで解決する。
///
/// - 断面性能・形状が完全に一致するものは 1 件へ統合し、参照していた file id を
///   同じ index へ写す（統合件数は `notes` で通知する）
/// - 一致しないものは符号へ連番を付けて（`b3` → `b3#2`）別断面として残す。
///   キーを一意にしたうえで定義を 1 件も捨てないための扱いで、`warnings` で通知する
fn build_sections(
    model: &mut Model,
    mut pending: Vec<PendingSec>,
    steel_lib: &HashMap<String, SectionShape>,
    material_index: &HashMap<u32, u32>,
    warnings: &mut Vec<String>,
    notes: &mut Vec<String>,
) -> HashMap<u32, u32> {
    // file id 昇順で整列（Standard 書き出しは分割断面を文書順に整列させないため）。
    pending.sort_by_key(|s| s.file_id);

    let mut index_map: HashMap<u32, u32> = HashMap::new();
    // 符号＋階 → model.sections の添字。衝突の検出と統合先の解決に使う。
    let mut by_key: HashMap<(String, Option<String>), usize> = HashMap::new();
    let mut merged = 0u32;
    let mut renamed: Vec<String> = Vec::new();
    for ps in pending.into_iter() {
        let file_id = ps.file_id;
        let floor = ps.floor.clone();
        // 統合・改番で最終的な添字が決まるまで id は仮置きする（下で確定させる）。
        let new_id = SectionId(model.sections.len() as u32);
        let section = match ps.kind {
            PendingSecKind::Raw {
                area,
                iy,
                iz,
                j,
                depth,
                width,
            } => Section {
                id: new_id,
                name: ps.name,
                area,
                iy,
                iz,
                j,
                depth,
                width,
                as_y: 0.0,
                as_z: 0.0,
                floor: None,
                panel_thickness: None,
                thickness: None,
                shape: None,
                material: None,
                rebar_material: None,
                shear_rebar_material: None,
                steel_material: None,
            },
            PendingSecKind::Shape(shape) => shape.to_section(new_id, ps.name),
            PendingSecKind::SteelRef(shape_name) => {
                // 形鋼ライブラリに定義がない参照は物性ゼロの断面として残す
                // （参照する部材の断面リンクを保つため。解析前に要確認）。
                match shape_name.and_then(|nm| steel_lib.get(&nm).cloned()) {
                    Some(shape) => shape.to_section(new_id, ps.name),
                    None => {
                        warnings.push(format!(
                            "鋼断面 (name=\"{}\") の形鋼参照を解決できず物性ゼロで取り込みました",
                            ps.name
                        ));
                        zero_section(new_id, ps.name)
                    }
                }
            }
            PendingSecKind::CftRef(steel_name) => {
                // 充填鋼管の形鋼（BOX/Pipe）を CFT 形状へ読み替える。
                let cft = steel_name
                    .and_then(|nm| steel_lib.get(&nm).cloned())
                    .and_then(|s| match s {
                        SectionShape::SteelBox {
                            height,
                            width,
                            thick,
                            ..
                        } => Some(SectionShape::CftBox {
                            height,
                            width,
                            thick,
                        }),
                        SectionShape::SteelPipe { outer_dia, thick } => {
                            Some(SectionShape::CftPipe { outer_dia, thick })
                        }
                        _ => None,
                    });
                match cft {
                    Some(shape) => shape.to_section(new_id, ps.name),
                    None => {
                        warnings.push(format!(
                            "CFT 断面 (name=\"{}\") の充填鋼管参照を解決できず物性ゼロで取り込みました",
                            ps.name
                        ));
                        zero_section(new_id, ps.name)
                    }
                }
            }
            PendingSecKind::SrcRef {
                b,
                d,
                rebar,
                steel_name,
            } => {
                // 内蔵鉄骨（H 形鋼）の寸法を解決する。未解決なら 0 とし、形状は保持する。
                let steel_dims = steel_name
                    .and_then(|nm| steel_lib.get(&nm).cloned())
                    .and_then(|s| match s {
                        SectionShape::SteelH {
                            height,
                            width,
                            web_thick,
                            flange_thick,
                        } => Some((height, width, web_thick, flange_thick)),
                        _ => None,
                    });
                if steel_dims.is_none() {
                    warnings.push(format!(
                        "SRC 断面 (name=\"{}\") の内蔵鉄骨参照を解決できず鉄骨寸法ゼロで取り込みました",
                        ps.name
                    ));
                }
                let (sh, sw, sweb, sfl) = steel_dims.unwrap_or((0.0, 0.0, 0.0, 0.0));
                SectionShape::SrcRect {
                    b,
                    d,
                    rebar,
                    steel_height: sh,
                    steel_width: sw,
                    steel_web_thick: sweb,
                    steel_flange_thick: sfl,
                }
                .to_section(new_id, ps.name)
            }
        };
        let mut section = section;
        section.floor = floor;
        // 材料は断面が持つ。主材料は断面側の参照（数値 id またはグレード名）から、
        // 配筋・内蔵鉄骨の材質はグレード名から材料テーブルへ解決して結ぶ。
        section.material = ps
            .mat
            .as_ref()
            .and_then(|r| resolve_sec_material(model, r, material_index));
        section.rebar_material =
            ps.grades.main_rebar.as_deref().and_then(|g| {
                find_or_create_bar_material(model, g, MaterialCategory::Rebar, notes)
            });
        section.shear_rebar_material =
            ps.grades.shear_rebar.as_deref().and_then(|g| {
                find_or_create_bar_material(model, g, MaterialCategory::Rebar, notes)
            });
        section.steel_material =
            ps.grades.steel.as_deref().and_then(|g| {
                find_or_create_bar_material(model, g, MaterialCategory::Steel, notes)
            });

        // 符号＋階の衝突を解決してから格納する。
        let idx = match by_key.get(&(section.name.clone(), section.floor.clone())) {
            // 完全に同じ断面の重複定義。統合し、参照だけを既存の断面へ向ける。
            Some(&existing) if model.sections[existing].properties_eq(&section) => {
                merged += 1;
                existing
            }
            // 同じ符号＋階で中身が違う。定義を捨てないよう符号へ連番を付けて残す。
            Some(_) => {
                let original = section.name.clone();
                let mut n = 2u32;
                while by_key.contains_key(&(format!("{original}#{n}"), section.floor.clone())) {
                    n += 1;
                }
                section.name = format!("{original}#{n}");
                renamed.push(format!("{} → {}", original, section.name));
                push_section(model, &mut by_key, section)
            }
            None => push_section(model, &mut by_key, section),
        };
        index_map.insert(file_id, idx as u32);
    }
    if merged > 0 {
        notes.push(format!(
            "符号＋階と断面性能が同一の断面定義 {merged} 件を既存の断面へ統合しました"
        ));
    }
    if !renamed.is_empty() {
        // 衝突が多いファイルで警告 1 行が際限なく伸びないよう、列挙は先頭のみに留める。
        const MAX_LISTED: usize = 10;
        let listed = renamed
            .iter()
            .take(MAX_LISTED)
            .cloned()
            .collect::<Vec<_>>()
            .join("、");
        let rest = renamed.len().saturating_sub(MAX_LISTED);
        let tail = if rest > 0 {
            format!("（ほか {rest} 件）")
        } else {
            String::new()
        };
        warnings.push(format!(
            "符号＋階が重複し断面性能が異なる断面が {} 件あったため、符号に連番を付けて取り込みました: {listed}{tail}",
            renamed.len()
        ));
    }
    index_map
}

/// 断面を末尾へ追加し、`id`（＝配列添字）を確定して符号＋階の索引へ登録する。
fn push_section(
    model: &mut Model,
    by_key: &mut HashMap<(String, Option<String>), usize>,
    mut section: Section,
) -> usize {
    let idx = model.sections.len();
    section.id = SectionId(idx as u32);
    by_key.insert((section.name.clone(), section.floor.clone()), idx);
    model.sections.push(section);
    idx
}

/// 物性ゼロ・形状なしの断面（形鋼名未解決などのフォールバック。解析前に要確認）。
fn zero_section(id: SectionId, name: String) -> Section {
    Section {
        id,
        name,
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 0.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: None,
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    }
}

/// 部材軸（`p_i`→`p_j`）まわりに断面回転角 `rotate` [deg] を適用した ref_vector を返す。
/// `rotate=0` の基準は、水平材は鉛直上（グローバル Z）、鉛直材はグローバル X 方向。
/// これは従来の既定 ref_vector と同一の局所座標系を与える（水平材で [0,0,1]）。
fn ref_vector_from_rotate(p_i: [f64; 3], p_j: [f64; 3], rotate_deg: f64) -> [f64; 3] {
    let axis = {
        let d = [p_j[0] - p_i[0], p_j[1] - p_i[1], p_j[2] - p_i[2]];
        let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if l < 1e-9 {
            [0.0, 0.0, 1.0]
        } else {
            [d[0] / l, d[1] / l, d[2] / l]
        }
    };
    // 軸が鉛直に近ければ基準を X、そうでなければ Z にとる。
    let base = if axis[2].abs() > 0.99 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    };
    // base を軸に直交化して rotate=0 の基準 ref0 を得る。
    let bdot = base[0] * axis[0] + base[1] * axis[1] + base[2] * axis[2];
    let ref0 = {
        let r = [
            base[0] - bdot * axis[0],
            base[1] - bdot * axis[1],
            base[2] - bdot * axis[2],
        ];
        let l = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
        if l < 1e-9 {
            base
        } else {
            [r[0] / l, r[1] / l, r[2] / l]
        }
    };
    if rotate_deg.abs() < 1e-9 {
        return ref0;
    }
    // ref0 を軸まわりに rotate 回転（ロドリゲスの回転公式。ref0⊥axis なので簡約形）。
    let th = rotate_deg.to_radians();
    let (s, c) = (th.sin(), th.cos());
    let cross = [
        axis[1] * ref0[2] - axis[2] * ref0[1],
        axis[2] * ref0[0] - axis[0] * ref0[2],
        axis[0] * ref0[1] - axis[1] * ref0[0],
    ];
    [
        ref0[0] * c + cross[0] * s,
        ref0[1] * c + cross[1] * s,
        ref0[2] * c + cross[2] * s,
    ]
}

/// 断面側の材料参照（数値 id またはグレード名）を `MaterialId` へ解決する。
fn resolve_sec_material(
    model: &Model,
    r: &SecMatRef,
    material_index: &HashMap<u32, u32>,
) -> Option<MaterialId> {
    match r {
        SecMatRef::Id(mid) => material_index.get(mid).copied().map(MaterialId),
        SecMatRef::Grade(name) => model
            .materials
            .iter()
            .find(|m| m.name == *name)
            .map(|m| m.id),
    }
}

/// 鉄筋・鉄骨のグレード名から材料を引き、無ければ作る（取り込みの境界）。
///
/// ST-Bridge は鉄筋・内蔵鉄骨の材質を**グレード名**で持つのに対し、内部モデルは
/// 材料テーブルへの参照で持つ。名前が一致する材料があればそれを使い、無ければ
/// グレード名から基準強度を推定して 1 行作る。推定に用いる規則
/// （[`squid_n_core::material_grade::rebar_grade_f_value`] ほか）はここでしか
/// 使わず、内部の強度解決は作られた材料の `fy` だけを見る。
fn find_or_create_bar_material(
    model: &mut Model,
    grade: &str,
    category: MaterialCategory,
    notes: &mut Vec<String>,
) -> Option<MaterialId> {
    let grade = grade.trim();
    if grade.is_empty() {
        return None;
    }
    if let Some(m) = model.materials.iter().find(|m| m.name == grade) {
        return Some(m.id);
    }
    let fy = match category {
        MaterialCategory::Rebar => squid_n_core::material_grade::rebar_grade_f_value(grade),
        // 鋼材は板厚区分を持つが、断面側の板厚はここでは解決できないため
        // 40mm 以下の基準強度で作る（板厚別の低減は検定側が板厚から解決する）。
        _ => squid_n_core::material_grade::steel_f_value_prefix(grade, 40.0),
    };
    if fy.is_none() {
        notes.push(format!(
            "材質「{grade}」の基準強度を名称から解決できませんでした。材料タブで fy を設定してください"
        ));
    }
    let id = MaterialId(model.materials.len() as u32);
    // 規格値はプリセット表から引く（名前が一致するものだけ。無ければ鋼系の一般値）。
    let preset = squid_n_core::material_grade::material_presets()
        .into_iter()
        .find(|p| p.name == grade);
    model.materials.push(Material {
        id,
        name: grade.to_string(),
        category,
        young: preset.as_ref().map(|p| p.young).unwrap_or(205_000.0),
        poisson: preset.as_ref().map(|p| p.poisson).unwrap_or(0.3),
        density: preset.as_ref().map(|p| p.density).unwrap_or(0.0),
        shear: None,
        fc: None,
        fy: fy.or_else(|| preset.as_ref().and_then(|p| p.fy)),
        concrete_class: Default::default(),
        strength_factor: None,
    });
    Some(id)
}
