//! 階への複製。ある階の入力を、同じ平面位置の相手へ上階（下階）へ配る。
//!
//! 実務では下の階で断面・床・荷重を決め、上の階へ同じ設定を配ってから階ごとに
//! 差分を直す。その「配る」操作をまとめたものが [`CopyStory`] である。
//!
//! # 何を対象にするか
//!
//! 階に属するかどうかは [`Model::member_story`]（材端節点のうちもっとも高い節点の
//! 所属階）で判定する。判定の情報源を 1 つに保つため、複製だけ別の規則は持たない。
//! 床基準の階では、階 `2F` に属するのは 1FL→2FL の柱と 2FL の大梁・床になる。
//!
//! # 対応付け
//!
//! **平面位置**で突き合わせる。柱は 1 点、大梁・小梁は両端の 2 点、壁・床は頂点の
//! 並びを使い、いずれも階のレベル差を引いた XY 座標が [`PLAN_TOL_MM`] 以内なら
//! 同じものとみなす。節点 ID では対応が取れない（階ごとに別の節点である）ため、
//! 座標で突き合わせるほかない。
//!
//! # 作るもの・作らないもの
//!
//! **部材と節点は作らない。** 架構そのものは作成ウィザードか手作業で先に用意されて
//! いる前提で、この機能の役目は断面・荷重・床割りを配ることである。相手が見つからな
//! かった分は件数を [`CopyStoryReport`] で返し、呼び出し側が利用者へ示す。
//!
//! 床と二次部材だけは、必要な節点がそろっていれば新しく作る（複製元にしかない床を
//! 配れないと、床割りを配るという目的が果たせないため）。すでにある場合は属性を
//! 上書きする。

use super::*;
use squid_n_core::ids::{ElemId, NodeId, SectionId, SlabId, StoryId};
use squid_n_core::model::{Section, Slab, SlabUsage};
use std::collections::{HashMap, HashSet};

/// 同じ平面位置とみなす座標差 [mm]。
pub const PLAN_TOL_MM: f64 = 1.0;

/// 複製する対象。ダイアログのチェックボックスに対応する。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CopyTargets {
    /// 部材の断面割当。複製先の階名で断面を複製してから割り当てる。
    pub sections: bool,
    /// 荷重（利用者が入れた節点荷重・部材荷重と、床の面荷重・用途）。
    pub loads: bool,
    /// 床（境界の形）。面荷重・用途は `loads` 側で写す。
    pub slabs: bool,
    /// 二次部材（小梁・間柱）。
    pub secondary: bool,
}

impl CopyTargets {
    pub fn any(self) -> bool {
        self.sections || self.loads || self.slabs || self.secondary
    }
}

/// 複製の結果。ダイアログへ出して、何が配られて何が配られなかったかを示す。
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CopyStoryReport {
    /// 断面を割り当てた部材の数。
    pub sections_assigned: usize,
    /// 新しく作った断面の数。
    pub sections_created: usize,
    /// 既にあった符号＋階の断面をそのまま使った数。
    pub sections_reused: usize,
    /// 複製した荷重の件数（節点荷重＋部材荷重）。
    pub loads_copied: usize,
    /// 複製先にあって載せ替えた荷重の件数（同上）。
    pub loads_replaced: usize,
    /// 新しく作った床の数。
    pub slabs_created: usize,
    /// 既にある床へ属性を上書きした数。
    pub slabs_updated: usize,
    /// 新しく作った二次部材の数。
    pub secondary_created: usize,
    /// 既にある二次部材へ属性を上書きした数。
    pub secondary_updated: usize,
    /// 複製先に相手が見つからず飛ばした数（部材・床・二次部材の合計）。
    pub skipped: usize,
}

impl CopyStoryReport {
    /// 何か 1 つでも複製したか。
    pub fn changed(&self) -> bool {
        self.sections_assigned > 0
            || self.loads_copied > 0
            || self.loads_replaced > 0
            || self.slabs_created > 0
            || self.slabs_updated > 0
            || self.secondary_created > 0
            || self.secondary_updated > 0
    }

    /// 利用者へ出す 1 行の要約。
    pub fn summary(&self) -> String {
        let mut parts = Vec::new();
        if self.sections_assigned > 0 {
            parts.push(format!(
                "断面 {} 部材（新規 {} ・既存 {}）",
                self.sections_assigned, self.sections_created, self.sections_reused
            ));
        }
        if self.loads_copied > 0 {
            let mut s = format!("荷重 {} 件", self.loads_copied);
            if self.loads_replaced > 0 {
                s.push_str(&format!("（{} 件を載せ替え）", self.loads_replaced));
            }
            parts.push(s);
        }
        if self.slabs_created + self.slabs_updated > 0 {
            parts.push(format!(
                "床 {} 枚（新規 {} ・更新 {}）",
                self.slabs_created + self.slabs_updated,
                self.slabs_created,
                self.slabs_updated
            ));
        }
        if self.secondary_created + self.secondary_updated > 0 {
            parts.push(format!(
                "二次部材 {} 本（新規 {} ・更新 {}）",
                self.secondary_created + self.secondary_updated,
                self.secondary_created,
                self.secondary_updated
            ));
        }
        if parts.is_empty() {
            parts.push("複製したものはありません".to_string());
        }
        let mut s = parts.join("、");
        if self.skipped > 0 {
            s.push_str(&format!(
                "。相手が見つからず {} 件を飛ばしました",
                self.skipped
            ));
        }
        s
    }
}

/// 平面位置のキー（[`PLAN_TOL_MM`] で丸めた XY 座標）。
type PlanKey = (i64, i64);

fn plan_key(coord: [f64; 3]) -> PlanKey {
    let q = |v: f64| (v / PLAN_TOL_MM).round() as i64;
    (q(coord[0]), q(coord[1]))
}

/// 複製元の階から複製先の階へ、選んだ対象を配る。
///
/// 複製先は複数指定できる（2 階の設定を 3・4・5 階へ一度に配る）。
/// 逆操作はモデル全体の復元とする。断面の追加・床の追加・荷重の追加が絡み合い、
/// 個別の逆コマンドを組むと順序の取り違えで壊れやすいためである
/// （[`RestoreStories`] と同じ、丸ごと戻す対称パターン）。
pub struct CopyStory {
    pub from: StoryId,
    pub to: Vec<StoryId>,
    pub targets: CopyTargets,
}

impl CopyStory {
    /// 複製を試したときの結果を、モデルを変えずに求める（ダイアログの事前表示用）。
    pub fn preview(&self, model: &Model) -> CopyStoryReport {
        let mut probe = model.clone();
        copy_into(&mut probe, self)
    }

    /// 複製で新しく作ることになる断面の符号＋階（ダイアログの事前表示用）。
    pub fn new_section_labels(&self, model: &Model) -> Vec<String> {
        let mut probe = model.clone();
        let before = probe.sections.len();
        copy_into(&mut probe, self);
        probe.sections[before..]
            .iter()
            .map(|s| s.display_name())
            .collect()
    }
}

impl EditCommand for CopyStory {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if !self.targets.any() {
            return Box::new(Noop);
        }
        let before = model.clone();
        let report = copy_into(model, self);
        if !report.changed() {
            *model = before;
            return Box::new(Noop);
        }
        Box::new(RestoreModel { old: before })
    }

    fn label(&self) -> &str {
        "階への複製"
    }
}

/// モデル全体を復元する逆操作（[`CopyStory`] 専用）。
pub struct RestoreModel {
    pub old: Model,
}

impl EditCommand for RestoreModel {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let replaced = std::mem::replace(model, self.old.clone());
        Box::new(RestoreModel { old: replaced })
    }

    fn label(&self) -> &str {
        "階への複製の復元"
    }
}

/// 複製の本体。`model` を書き換えて結果を返す。
fn copy_into(model: &mut Model, cmd: &CopyStory) -> CopyStoryReport {
    let mut report = CopyStoryReport::default();
    for &to in &cmd.to {
        if to == cmd.from {
            continue;
        }
        copy_one(model, cmd.from, to, cmd.targets, &mut report);
    }
    report
}

fn copy_one(
    model: &mut Model,
    from: StoryId,
    to: StoryId,
    targets: CopyTargets,
    report: &mut CopyStoryReport,
) {
    let (Some(src_story), Some(dst_story)) = (
        model.stories.get(from.index()).cloned(),
        model.stories.get(to.index()).cloned(),
    ) else {
        return;
    };
    let dz = dst_story.elevation - src_story.elevation;

    if targets.sections {
        copy_sections(model, from, to, &dst_story.name, report);
    }
    // この位置より後ろの床は、直後の `copy_slabs` が作った新しい床である。
    // 面荷重を載せたときに「新規」と「更新」を二重に数えないよう境目を控える。
    let slab_base = model.slabs.len();
    if targets.slabs {
        copy_slabs(model, from, to, dz, report);
    }
    if targets.secondary {
        copy_secondary(model, from, to, dz, report);
    }
    if targets.loads {
        copy_slab_loads(model, from, to, slab_base, report);
        copy_case_loads(model, from, to, dz, report);
    }
}

/// 平面位置の並び（部材・床の対応付けキー）。節点順の違いを吸収するため整列する。
fn plan_keys(model: &Model, nodes: &[NodeId]) -> Option<Vec<PlanKey>> {
    let mut keys: Vec<PlanKey> = nodes
        .iter()
        .map(|n| model.nodes.get(n.index()).map(|nd| plan_key(nd.coord)))
        .collect::<Option<Vec<_>>>()?;
    keys.sort_unstable();
    Some(keys)
}

/// 階に属する部材を、平面位置の並びで引ける索引にする。
fn members_by_plan(model: &Model, story: StoryId) -> HashMap<Vec<PlanKey>, ElemId> {
    let mut out = HashMap::new();
    for e in &model.elements {
        if model.member_story(e) != Some(story) {
            continue;
        }
        if let Some(k) = plan_keys(model, &e.nodes) {
            out.entry(k).or_insert(e.id);
        }
    }
    out
}

/// 断面の割当を配る。複製先の階名で断面を複製してから割り当てる。
fn copy_sections(
    model: &mut Model,
    from: StoryId,
    to: StoryId,
    dst_story_name: &str,
    report: &mut CopyStoryReport,
) {
    let dst = members_by_plan(model, to);
    // 複製元の断面 → 複製先の断面。同じ組は 1 回だけ作る。
    let mut mapped: HashMap<SectionId, SectionId> = HashMap::new();
    let src: Vec<(Vec<PlanKey>, SectionId)> = model
        .elements
        .iter()
        .filter(|e| model.member_story(e) == Some(from))
        .filter_map(|e| Some((plan_keys(model, &e.nodes)?, e.section?)))
        .collect();

    for (key, src_sec) in src {
        let Some(&elem) = dst.get(&key) else {
            report.skipped += 1;
            continue;
        };
        let dst_sec = match mapped.get(&src_sec) {
            Some(&s) => s,
            None => {
                let s = section_for_story(model, src_sec, dst_story_name, report);
                mapped.insert(src_sec, s);
                s
            }
        };
        if let Some(e) = model.elements.get_mut(elem.index()) {
            if e.section != Some(dst_sec) {
                e.section = Some(dst_sec);
                report.sections_assigned += 1;
            }
        }
    }
}

/// 複製先の階名を持つ断面を返す。無ければ複製元の断面を複製して作る。
///
/// 階を持たない断面（`floor` が `None`）は階に紐づかないため、そのまま共有する。
/// 同じ符号＋階の断面が既にあれば、寸法が違ってもそれを使う（利用者がすでに
/// 上階の断面を決めているときに、複製が黙って寸法を戻すのを避ける）。
fn section_for_story(
    model: &mut Model,
    src: SectionId,
    dst_story_name: &str,
    report: &mut CopyStoryReport,
) -> SectionId {
    let Some(src_sec) = model.sections.get(src.index()).cloned() else {
        return src;
    };
    if src_sec.floor.is_none() {
        return src;
    }
    let key = (src_sec.name.as_str(), Some(dst_story_name));
    if let Some(found) = model.sections.iter().find(|s| s.key() == key) {
        report.sections_reused += 1;
        return found.id;
    }
    let id = SectionId(model.sections.len() as u32);
    model.sections.push(Section {
        id,
        floor: Some(dst_story_name.to_string()),
        ..src_sec
    });
    report.sections_created += 1;
    id
}

/// 階の床を、平面位置の並びで引ける索引にする。
fn slabs_by_plan(model: &Model, story: StoryId) -> HashMap<Vec<PlanKey>, SlabId> {
    let mut out = HashMap::new();
    for sl in &model.slabs {
        if slab_story(model, sl) != Some(story) {
            continue;
        }
        if let Some(k) = plan_keys(model, &sl.boundary) {
            out.entry(k).or_insert(sl.id);
        }
    }
    out
}

/// 床の所属階（境界節点のうちもっとも高い節点の所属階。部材と同じ規則）。
fn slab_story(model: &Model, slab: &Slab) -> Option<StoryId> {
    slab.boundary
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .max_by(|a, b| a.coord[2].total_cmp(&b.coord[2]))
        .and_then(|n| n.story)
}

/// 複製元の節点に対応する複製先の節点を、平面位置と標高差から引く。
fn mapped_node(model: &Model, src: NodeId, dz: f64) -> Option<NodeId> {
    let c = model.nodes.get(src.index())?.coord;
    let want = [c[0], c[1], c[2] + dz];
    model
        .nodes
        .iter()
        .find(|n| {
            (n.coord[0] - want[0]).abs() <= PLAN_TOL_MM
                && (n.coord[1] - want[1]).abs() <= PLAN_TOL_MM
                && (n.coord[2] - want[2]).abs() <= PLAN_TOL_MM
        })
        .map(|n| n.id)
}

/// 床（境界の形）を配る。すでに同じ位置に床があれば境界はそのままにする。
fn copy_slabs(
    model: &mut Model,
    from: StoryId,
    to: StoryId,
    dz: f64,
    report: &mut CopyStoryReport,
) {
    let existing = slabs_by_plan(model, to);
    let src: Vec<Slab> = model
        .slabs
        .iter()
        .filter(|sl| slab_story(model, sl) == Some(from))
        .cloned()
        .collect();
    for sl in src {
        let Some(key) = plan_keys(model, &sl.boundary) else {
            report.skipped += 1;
            continue;
        };
        if existing.contains_key(&key) {
            continue;
        }
        let Some(boundary) = sl
            .boundary
            .iter()
            .map(|n| mapped_node(model, *n, dz))
            .collect::<Option<Vec<_>>>()
        else {
            report.skipped += 1;
            continue;
        };
        let id = SlabId(model.slabs.len() as u32);
        model.slabs.push(Slab {
            id,
            boundary,
            loads: Vec::new(),
            usage: None,
            joists: Vec::new(),
            ..sl
        });
        report.slabs_created += 1;
    }
}

/// 床の面荷重・用途を配る（「荷重」の対象。床の形は `copy_slabs` が受け持つ）。
///
/// 複製先の床は 1 段上にあるだけなので、平面位置だけで突き合わせられる。
/// `slab_base` より後ろの床は同じ操作で作ったばかりの床のため、「更新」には数えない
/// （数えると 1 枚の床が「新規」と「更新」で二重に報告される）。
fn copy_slab_loads(
    model: &mut Model,
    from: StoryId,
    to: StoryId,
    slab_base: usize,
    report: &mut CopyStoryReport,
) {
    let dst = slabs_by_plan(model, to);
    let src: Vec<(
        Vec<PlanKey>,
        Vec<squid_n_core::model::AreaLoad>,
        Option<SlabUsage>,
    )> = model
        .slabs
        .iter()
        .filter(|sl| slab_story(model, sl) == Some(from))
        .filter_map(|sl| Some((plan_keys(model, &sl.boundary)?, sl.loads.clone(), sl.usage)))
        .collect();
    for (key, loads, usage) in src {
        let Some(&sid) = dst.get(&key) else {
            report.skipped += 1;
            continue;
        };
        let is_new = sid.index() >= slab_base;
        if let Some(sl) = model.slabs.get_mut(sid.index()) {
            let changed = sl.loads != loads || sl.usage != usage;
            sl.loads = loads;
            sl.usage = usage;
            if changed && !is_new {
                report.slabs_updated += 1;
            }
        }
    }
}

/// 二次部材（小梁・間柱）を配る。
fn copy_secondary(
    model: &mut Model,
    from: StoryId,
    to: StoryId,
    dz: f64,
    report: &mut CopyStoryReport,
) {
    let member_story_of = |model: &Model, nodes: &[NodeId; 2]| -> Option<StoryId> {
        nodes
            .iter()
            .filter_map(|nid| model.nodes.get(nid.index()))
            .max_by(|a, b| a.coord[2].total_cmp(&b.coord[2]))
            .and_then(|n| n.story)
    };
    let existing: HashMap<Vec<PlanKey>, usize> = model
        .secondary_members
        .iter()
        .enumerate()
        .filter(|(_, sm)| member_story_of(model, &sm.nodes) == Some(to))
        .filter_map(|(i, sm)| Some((plan_keys(model, &sm.nodes)?, i)))
        .collect();
    let src: Vec<squid_n_core::model::SecondaryMember> = model
        .secondary_members
        .iter()
        .filter(|sm| member_story_of(model, &sm.nodes) == Some(from))
        .cloned()
        .collect();
    for sm in src {
        let Some(key) = plan_keys(model, &sm.nodes) else {
            report.skipped += 1;
            continue;
        };
        if let Some(&i) = existing.get(&key) {
            model.secondary_members[i].section = sm.section;
            report.secondary_updated += 1;
            continue;
        }
        let (Some(a), Some(b)) = (
            mapped_node(model, sm.nodes[0], dz),
            mapped_node(model, sm.nodes[1], dz),
        ) else {
            report.skipped += 1;
            continue;
        };
        model
            .secondary_members
            .push(squid_n_core::model::SecondaryMember {
                nodes: [a, b],
                ..sm
            });
        report.secondary_created += 1;
    }
}

/// 荷重ケースの節点荷重・部材荷重を配る。
///
/// 対象は利用者が入れた分（`LoadSource::Manual`）だけとする。準備計算・荷重同期が
/// 作る分は同期のたびに全件作り直されるため、複製しても次の同期で消える。
///
/// 複製先の節点・部材にすでに載っている手入力荷重は、**取り除いてから載せ替える**。
/// 足すだけにすると、同じ複製を 2 回実行したときに荷重が二重になり、
/// 見た目では気づけないまま重い設計になってしまうためである。載せ替えた件数は
/// [`CopyStoryReport::loads_replaced`] で返し、何が失われたかを利用者へ示す。
/// 相手が見つからなかった節点・部材の荷重には手を触れない。
fn copy_case_loads(
    model: &mut Model,
    from: StoryId,
    to: StoryId,
    dz: f64,
    report: &mut CopyStoryReport,
) {
    let dst_members = members_by_plan(model, to);
    // 複製元の節点 → 複製先の節点。所属階の判定は部材・床と同じく `Node::story`
    // （準備計算が付ける）に従う。
    let src_nodes: Vec<NodeId> = model
        .nodes
        .iter()
        .filter(|n| n.story == Some(from))
        .map(|n| n.id)
        .collect();
    let node_map: HashMap<NodeId, NodeId> = src_nodes
        .into_iter()
        .filter_map(|n| Some((n, mapped_node(model, n, dz)?)))
        .collect();
    // 複製元の部材 → 複製先の部材。
    let elem_map: HashMap<ElemId, ElemId> = model
        .elements
        .iter()
        .filter(|e| model.member_story(e) == Some(from))
        .filter_map(|e| {
            let k = plan_keys(model, &e.nodes)?;
            Some((e.id, *dst_members.get(&k)?))
        })
        .collect();
    // 載せ替えの対象（複製元に相手がある複製先の節点・部材）。
    let dst_nodes: HashSet<NodeId> = node_map.values().copied().collect();
    let dst_elems: HashSet<ElemId> = elem_map.values().copied().collect();

    for lc in &mut model.load_cases {
        let mut add_nodal = Vec::new();
        for nl in lc.nodal.iter().filter(|l| !l.source.is_auto()) {
            if let Some(&n) = node_map.get(&nl.node) {
                add_nodal.push(squid_n_core::model::NodalLoad {
                    node: n,
                    ..nl.clone()
                });
            }
        }
        let mut add_member = Vec::new();
        for ml in lc.member.iter().filter(|l| !l.source.is_auto()) {
            if let Some(&e) = elem_map.get(&ml.elem) {
                add_member.push(squid_n_core::model::MemberLoad {
                    elem: e,
                    ..ml.clone()
                });
            }
        }
        // 先に複製先の手入力荷重を取り除いてから足す（二重載荷を防ぐ）。
        let before = lc.nodal.len() + lc.member.len();
        lc.nodal
            .retain(|l| l.source.is_auto() || !dst_nodes.contains(&l.node));
        lc.member
            .retain(|l| l.source.is_auto() || !dst_elems.contains(&l.elem));
        report.loads_replaced += before - (lc.nodal.len() + lc.member.len());
        report.loads_copied += add_nodal.len() + add_member.len();
        lc.nodal.extend(add_nodal);
        lc.member.extend(add_member);
    }
}
