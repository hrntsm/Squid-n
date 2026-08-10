//! 階への複製。ある階の入力を、同じ平面位置の相手へ配る。
//!
//! 実務では下の階で断面・床・荷重を決め、上の階へ同じ設定を配ってから階ごとに
//! 差分を直す。その「配る」操作をまとめたものが [`CopyStory`] である。
//!
//! # 複製の意味
//!
//! 複製は**複製元の状態をそのまま写す**。複製元に「無い」という状態も写すため、
//! [`CopyStory::overwrite`] が真なら複製先の余分（複製元で断面が未割当・床が無い・
//! 荷重が無い相手）は解除・削除される。偽なら複製先が空いているところにだけ入れ、
//! 既存には触れない。どちらでも 2 回実行した結果は 1 回と同じになる（冪等）。
//!
//! # 何を対象にするか
//!
//! 階に属するかどうかは [`Model::member_story`]（材端節点のうちもっとも高い節点の
//! 所属階）で判定する。判定の情報源を 1 つに保つため、複製だけ別の規則は持たない。
//! 床基準の階では、階 `2F` に属するのは 1FL→2FL の柱と 2FL の大梁・床になる。
//!
//! # 対応付け
//!
//! **平面位置**で突き合わせる。柱は 1 点、大梁・小梁は両端の 2 点、床は頂点の
//! 並びを使い、いずれも XY 座標が [`PLAN_TOL_MM`] 以内なら同じものとみなす。
//! 節点 ID では対応が取れない（階ごとに別の節点である）ため、座標で突き合わせる
//! ほかない。
//!
//! **手を触れるのは、複製元に対応する相手が見つかったものだけである。** 複製元の
//! 平面の外にある複製先の部材・床・二次部材・荷重には、上書きが真でも触れない。
//! セットバックや張り出しのある建物で、複製元と関係のない場所が消えるのを防ぐ。
//!
//! # 作るもの・作らないもの
//!
//! **部材と節点は作らない。** 架構そのものは作成ウィザードか手作業で先に用意されて
//! いる前提で、この機能の役目は断面・荷重・床割りを配ることである。相手が見つから
//! なかった分は件数を [`CopyStoryReport`] で返し、呼び出し側が利用者へ示す。
//!
//! 床と二次部材だけは、必要な節点がそろっていれば新しく作る（複製元にしかない床を
//! 配れないと、床割りを配るという目的が果たせないため）。
//!
//! **断面の中身（寸法・材料）は書き換えない。** 複製先の階名を持つ同じ符号の断面が
//! 既にあれば、寸法が違ってもそれを使う。断面はどの階の部材からでも参照できるため、
//! 中身を書き換えると複製の対象範囲の外にある部材まで変わってしまう。寸法が食い違う
//! 場合は [`CopyStoryReport::mismatched_sections`] で名指しし、利用者が判断する。

use super::*;
use squid_n_core::ids::{ElemId, NodeId, SectionId, SlabId, StoryId};
use squid_n_core::model::{SecondaryMember, Section, Slab, SlabUsage};
use std::collections::{HashMap, HashSet};

/// 同じ平面位置とみなす座標差 [mm]。
pub const PLAN_TOL_MM: f64 = 1.0;

/// 複製する対象。ダイアログのチェックボックスに対応する。
///
/// 既定はすべて偽とする。複製は削除・解除も行うため、何を配るかは利用者が
/// 必ず選ぶ形にする。
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
    /// 断面の割当を解除した部材の数（複製元が未割当のため）。
    pub sections_cleared: usize,
    /// 新しく作った断面の数。
    pub sections_created: usize,
    /// 既にあった符号＋階の断面をそのまま使った数。
    pub sections_reused: usize,
    /// 使い回した既存断面のうち、寸法・材料が複製元と違うものの符号＋階。
    /// 中身は書き換えないため、食い違いを利用者へ示すために持つ。
    pub mismatched_sections: Vec<String>,
    /// 複製した荷重の件数（節点荷重＋部材荷重）。
    pub loads_copied: usize,
    /// 複製先から取り除いた荷重の件数（同上）。
    pub loads_removed: usize,
    /// 新しく作った床の数。
    pub slabs_created: usize,
    /// 既にある床へ属性を上書きした数。
    pub slabs_updated: usize,
    /// 削除した床の数（複製元に同じ位置の床が無いため）。
    pub slabs_deleted: usize,
    /// 新しく作った二次部材の数。
    pub secondary_created: usize,
    /// 既にある二次部材へ属性を上書きした数。
    pub secondary_updated: usize,
    /// 削除した二次部材の数（複製元に同じ位置の二次部材が無いため）。
    pub secondary_deleted: usize,
    /// 複製先に相手が見つからず飛ばした数（部材・床・二次部材の合計）。
    pub skipped: usize,
}

impl CopyStoryReport {
    /// 何か 1 つでも複製したか。
    pub fn changed(&self) -> bool {
        self.sections_assigned > 0
            || self.sections_cleared > 0
            || self.loads_copied > 0
            || self.loads_removed > 0
            || self.slabs_created > 0
            || self.slabs_updated > 0
            || self.slabs_deleted > 0
            || self.secondary_created > 0
            || self.secondary_updated > 0
            || self.secondary_deleted > 0
    }

    /// 入力が減る変更を含むか（削除・解除）。実行前の確認で強調するために使う。
    pub fn removes_input(&self) -> bool {
        self.sections_cleared > 0
            || self.loads_removed > 0
            || self.slabs_deleted > 0
            || self.secondary_deleted > 0
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
        if self.sections_cleared > 0 {
            parts.push(format!("断面の解除 {} 部材", self.sections_cleared));
        }
        if self.loads_copied > 0 {
            parts.push(format!("荷重 {} 件", self.loads_copied));
        }
        if self.loads_removed > 0 {
            parts.push(format!("荷重の削除 {} 件", self.loads_removed));
        }
        if self.slabs_created + self.slabs_updated > 0 {
            parts.push(format!(
                "床 {} 枚（新規 {} ・更新 {}）",
                self.slabs_created + self.slabs_updated,
                self.slabs_created,
                self.slabs_updated
            ));
        }
        if self.slabs_deleted > 0 {
            parts.push(format!("床の削除 {} 枚", self.slabs_deleted));
        }
        if self.secondary_created + self.secondary_updated > 0 {
            parts.push(format!(
                "二次部材 {} 本（新規 {} ・更新 {}）",
                self.secondary_created + self.secondary_updated,
                self.secondary_created,
                self.secondary_updated
            ));
        }
        if self.secondary_deleted > 0 {
            parts.push(format!("二次部材の削除 {} 本", self.secondary_deleted));
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
/// 逆操作はモデル全体の復元とする。断面の追加・床の追加削除・荷重の載せ替えが
/// 絡み合い、個別の逆コマンドを組むと順序の取り違えで壊れやすいためである
/// （[`RestoreStories`] と同じ、丸ごと戻す対称パターン）。
pub struct CopyStory {
    pub from: StoryId,
    pub to: Vec<StoryId>,
    pub targets: CopyTargets,
    /// 複製先の既存を上書きするか。
    ///
    /// 真なら複製元の状態をそのまま写す（置換・追加に加え、複製元に無いものの
    /// 解除・削除も行う）。偽なら複製先が空いているところにだけ入れ、既存には
    /// 触れない。
    pub overwrite: bool,
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
        copy_one(model, cmd, to, &mut report);
    }
    report.mismatched_sections.sort();
    report.mismatched_sections.dedup();
    report
}

fn copy_one(model: &mut Model, cmd: &CopyStory, to: StoryId, report: &mut CopyStoryReport) {
    let from = cmd.from;
    let (Some(src_story), Some(dst_story)) = (
        model.stories.get(from.index()).cloned(),
        model.stories.get(to.index()).cloned(),
    ) else {
        return;
    };
    let dz = dst_story.elevation - src_story.elevation;

    if cmd.targets.sections {
        copy_sections(model, cmd, to, &dst_story.name, report);
    }
    // この位置より後ろの床は、直後の `copy_slabs` が作った新しい床である。
    // 面荷重を載せたときに「新規」と「更新」を二重に数えないよう境目を控える。
    let slab_base = model.slabs.len();
    if cmd.targets.slabs {
        copy_slabs(model, cmd, to, dz, report);
    }
    if cmd.targets.secondary {
        copy_secondary(model, cmd, to, dz, report);
    }
    if cmd.targets.loads {
        copy_slab_loads(model, cmd, to, slab_base, report);
        copy_case_loads(model, cmd, to, dz, report);
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
///
/// 上書きが真のときは、複製元の部材が断面を持たない相手の割当を解除する。
/// 複製元に相手がいない部材（複製元の平面の外）には触れない。
fn copy_sections(
    model: &mut Model,
    cmd: &CopyStory,
    to: StoryId,
    dst_story_name: &str,
    report: &mut CopyStoryReport,
) {
    let dst = members_by_plan(model, to);
    // 複製元の断面 → 複製先の断面。同じ組は 1 回だけ作る。
    let mut mapped: HashMap<SectionId, SectionId> = HashMap::new();
    // 複製元の部材（断面の有無を問わず、平面位置と断面を控える）。
    let src: Vec<(Vec<PlanKey>, Option<SectionId>)> = model
        .elements
        .iter()
        .filter(|e| model.member_story(e) == Some(cmd.from))
        .filter_map(|e| Some((plan_keys(model, &e.nodes)?, e.section)))
        .collect();

    for (key, src_sec) in src {
        let Some(&elem) = dst.get(&key) else {
            report.skipped += 1;
            continue;
        };
        let Some(src_sec) = src_sec else {
            // 複製元が未割当。上書きなら複製先も未割当へそろえる。
            if cmd.overwrite {
                if let Some(e) = model.elements.get_mut(elem.index()) {
                    if e.section.is_some() {
                        e.section = None;
                        report.sections_cleared += 1;
                    }
                }
            }
            continue;
        };
        // 上書きしない設定では、既に断面が付いている部材には触れない。
        if !cmd.overwrite
            && model
                .elements
                .get(elem.index())
                .is_some_and(|e| e.section.is_some())
        {
            continue;
        }
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
/// 同じ符号＋階の断面が既にあれば、寸法が違ってもそれを使う。断面はどの階の部材
/// からでも参照できるため、中身を書き換えると複製の対象範囲の外にある部材まで
/// 変わってしまう。食い違いは [`CopyStoryReport::mismatched_sections`] で示す。
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
        // 符号＋階が同じでも中身が違えば、複製しても寸法はそろわない。
        let same_content = Section {
            id: found.id,
            floor: found.floor.clone(),
            ..src_sec.clone()
        } == *found;
        if !same_content {
            report.mismatched_sections.push(found.display_name());
        }
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

/// 二次部材の所属階（材端節点のうちもっとも高い節点の所属階）。
fn secondary_story(model: &Model, sm: &SecondaryMember) -> Option<StoryId> {
    sm.nodes
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

/// 複製先の節点が、複製元の階にも対応する節点を持つか。
///
/// 削除の判断対象を「複製元の平面の内側」に限るために使う。複製元の平面の外に
/// ある床・二次部材は、複製元に「無い」のではなく複製元の範囲外なので消さない。
fn maps_back(model: &Model, dst: NodeId, dz: f64) -> bool {
    mapped_node(model, dst, -dz).is_some()
}

/// 床（境界の形）を配る。
///
/// 上書きが真のときは、複製元に同じ位置の床が無い複製先の床を削除する。ただし
/// 境界節点すべてが複製元の階へ対応するものに限る（[`maps_back`]）。
fn copy_slabs(
    model: &mut Model,
    cmd: &CopyStory,
    to: StoryId,
    dz: f64,
    report: &mut CopyStoryReport,
) {
    let src_keys: HashSet<Vec<PlanKey>> = model
        .slabs
        .iter()
        .filter(|sl| slab_story(model, sl) == Some(cmd.from))
        .filter_map(|sl| plan_keys(model, &sl.boundary))
        .collect();

    // 先に削除する（複製元に無い床を消してから、複製元の床を作る）。
    if cmd.overwrite {
        let doomed: Vec<SlabId> = model
            .slabs
            .iter()
            .filter(|sl| slab_story(model, sl) == Some(to))
            .filter(|sl| {
                plan_keys(model, &sl.boundary).is_some_and(|k| !src_keys.contains(&k))
                    && sl.boundary.iter().all(|&n| maps_back(model, n, dz))
            })
            .map(|sl| sl.id)
            .collect();
        // 床の削除は後続の `SlabId` を繰り上げるため、降順に消す
        // （先に消した床より小さい ID は動かないので、控えた ID がずれない）。
        for id in doomed.into_iter().rev() {
            let inverse = crate::DeleteSlab { id }.apply(model);
            if !inverse.is_noop() {
                report.slabs_deleted += 1;
            }
        }
    }

    let existing = slabs_by_plan(model, to);
    let src: Vec<Slab> = model
        .slabs
        .iter()
        .filter(|sl| slab_story(model, sl) == Some(cmd.from))
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
    cmd: &CopyStory,
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
        .filter(|sl| slab_story(model, sl) == Some(cmd.from))
        .filter_map(|sl| Some((plan_keys(model, &sl.boundary)?, sl.loads.clone(), sl.usage)))
        .collect();
    for (key, loads, usage) in src {
        let Some(&sid) = dst.get(&key) else {
            report.skipped += 1;
            continue;
        };
        let is_new = sid.index() >= slab_base;
        let Some(sl) = model.slabs.get_mut(sid.index()) else {
            continue;
        };
        // 上書きしない設定では、既に面荷重・用途が入っている床には触れない。
        if !cmd.overwrite && !is_new && (!sl.loads.is_empty() || sl.usage.is_some()) {
            continue;
        }
        let changed = sl.loads != loads || sl.usage != usage;
        sl.loads = loads;
        sl.usage = usage;
        if changed && !is_new {
            report.slabs_updated += 1;
        }
    }
}

/// 二次部材（小梁・間柱）を配る。
///
/// 上書きが真のときは、複製元に同じ位置の二次部材が無い複製先の二次部材を削除する
/// （材端節点が複製元の階へ対応するものに限る）。
fn copy_secondary(
    model: &mut Model,
    cmd: &CopyStory,
    to: StoryId,
    dz: f64,
    report: &mut CopyStoryReport,
) {
    let src_keys: HashSet<Vec<PlanKey>> = model
        .secondary_members
        .iter()
        .filter(|sm| secondary_story(model, sm) == Some(cmd.from))
        .filter_map(|sm| plan_keys(model, &sm.nodes))
        .collect();

    if cmd.overwrite {
        let before = model.secondary_members.len();
        let keep: Vec<SecondaryMember> = model
            .secondary_members
            .iter()
            .filter(|sm| {
                if secondary_story(model, sm) != Some(to) {
                    return true;
                }
                let unmatched = plan_keys(model, &sm.nodes).is_some_and(|k| !src_keys.contains(&k));
                let in_src_plan = sm.nodes.iter().all(|&n| maps_back(model, n, dz));
                !(unmatched && in_src_plan)
            })
            .cloned()
            .collect();
        report.secondary_deleted += before - keep.len();
        model.secondary_members = keep;
    }

    let existing: HashMap<Vec<PlanKey>, usize> = model
        .secondary_members
        .iter()
        .enumerate()
        .filter(|(_, sm)| secondary_story(model, sm) == Some(to))
        .filter_map(|(i, sm)| Some((plan_keys(model, &sm.nodes)?, i)))
        .collect();
    let src: Vec<SecondaryMember> = model
        .secondary_members
        .iter()
        .filter(|sm| secondary_story(model, sm) == Some(cmd.from))
        .cloned()
        .collect();
    for sm in src {
        let Some(key) = plan_keys(model, &sm.nodes) else {
            report.skipped += 1;
            continue;
        };
        if let Some(&i) = existing.get(&key) {
            // 上書きしない設定では、既にある二次部材には触れない。
            if !cmd.overwrite {
                continue;
            }
            if model.secondary_members[i].section != sm.section {
                model.secondary_members[i].section = sm.section;
                report.secondary_updated += 1;
            }
            continue;
        }
        let (Some(a), Some(b)) = (
            mapped_node(model, sm.nodes[0], dz),
            mapped_node(model, sm.nodes[1], dz),
        ) else {
            report.skipped += 1;
            continue;
        };
        model.secondary_members.push(SecondaryMember {
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
/// 上書きが真のときは、複製元に相手がある複製先の節点・部材から手入力荷重を
/// 取り除いてから複製元の分を載せる。複製元がその相手に荷重を持たない場合も
/// 取り除いたままにする（「無い」という状態を写す）。相手が見つからなかった
/// 節点・部材の荷重には手を触れない。
fn copy_case_loads(
    model: &mut Model,
    cmd: &CopyStory,
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
        .filter(|n| n.story == Some(cmd.from))
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
        .filter(|e| model.member_story(e) == Some(cmd.from))
        .filter_map(|e| {
            let k = plan_keys(model, &e.nodes)?;
            Some((e.id, *dst_members.get(&k)?))
        })
        .collect();
    // 手を触れてよい複製先（複製元に相手がある節点・部材）。
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
        if cmd.overwrite {
            // 複製元に相手がある複製先の手入力荷重を取り除いてから載せる。
            let before = lc.nodal.len() + lc.member.len();
            lc.nodal
                .retain(|l| l.source.is_auto() || !dst_nodes.contains(&l.node));
            lc.member
                .retain(|l| l.source.is_auto() || !dst_elems.contains(&l.elem));
            report.loads_removed += before - (lc.nodal.len() + lc.member.len());
        } else {
            // 上書きしない設定では、既に手入力荷重が載っている相手へは載せない。
            let busy_nodes: HashSet<NodeId> = lc
                .nodal
                .iter()
                .filter(|l| !l.source.is_auto())
                .map(|l| l.node)
                .collect();
            let busy_elems: HashSet<ElemId> = lc
                .member
                .iter()
                .filter(|l| !l.source.is_auto())
                .map(|l| l.elem)
                .collect();
            add_nodal.retain(|l| !busy_nodes.contains(&l.node));
            add_member.retain(|l| !busy_elems.contains(&l.elem));
        }
        report.loads_copied += add_nodal.len() + add_member.len();
        lc.nodal.extend(add_nodal);
        lc.member.extend(add_member);
    }
}
