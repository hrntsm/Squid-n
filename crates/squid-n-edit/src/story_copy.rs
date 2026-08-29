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
use squid_n_core::model::{
    SecondaryMember, SecondaryMemberKind, Section, Slab, SlabPlate, SlabShape, SlabUsage,
};
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
    /// 新しく作った断面の符号＋階（実行前の見込みで一覧に出す）。
    pub created_sections: Vec<String>,
    /// 既にあった符号＋階の断面をそのまま使った数。
    pub sections_reused: usize,
    /// 使い回した既存断面のうち、寸法・材料が複製元と違うものの符号＋階。
    /// 中身は書き換えないため、食い違いを利用者へ示すために持つ。
    pub mismatched_sections: Vec<String>,
    /// 複製した荷重の件数（節点荷重＋部材荷重）。
    pub loads_copied: usize,
    /// 複製先から取り除いた荷重の件数（同上）。
    pub loads_removed: usize,
    /// 載荷区間が複製先の材長に収まらず配れなかった部材荷重の件数。
    pub loads_unfit: usize,
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
        if self.loads_unfit > 0 {
            parts.push(format!(
                "材長に収まらず配れなかった荷重 {} 件",
                self.loads_unfit
            ));
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

/// 1 方向の座標を、許容差以内で 1 つへ畳んだ代表値の列（昇順）。
///
/// 座標を [`PLAN_TOL_MM`] で丸めて整数キーにすると、丸めの境目をまたぐ 2 点が
/// 0.001 mm しか離れていなくても別のキーになる。取り込んだモデルの小数の揺れで
/// 対応が取れなくなるため、先に座標そのものを代表値へ寄せてからキーを作る。
/// 畳み方は通り芯の重複の畳み込み（[`squid_n_core::axis_gen`]）と同じで、
/// 整列してから許容差以内の連なりを 1 つにまとめる。
struct Axis1d {
    reps: Vec<f64>,
}

impl Axis1d {
    fn build(values: impl Iterator<Item = f64>) -> Self {
        let mut v: Vec<f64> = values.collect();
        v.sort_by(f64::total_cmp);
        let mut reps: Vec<f64> = Vec::new();
        for x in v {
            if reps.last().is_none_or(|r| (x - r).abs() > PLAN_TOL_MM) {
                reps.push(x);
            }
        }
        Self { reps }
    }

    /// 値が属する代表の番号（許容差以内に代表が無ければ `None`）。
    fn find(&self, v: f64) -> Option<usize> {
        if self.reps.is_empty() {
            return None;
        }
        let i = self.reps.partition_point(|r| *r < v);
        // 挿入位置の前後だけが候補になる（代表は昇順）。
        [i.checked_sub(1), Some(i)]
            .into_iter()
            .flatten()
            .filter(|&k| k < self.reps.len())
            .filter(|&k| (self.reps[k] - v).abs() <= PLAN_TOL_MM)
            .min_by(|&a, &b| {
                (self.reps[a] - v)
                    .abs()
                    .total_cmp(&(self.reps[b] - v).abs())
            })
    }
}

/// 節点座標の正規化と、正規化キーからの逆引き。
///
/// 複製は「複製元の節点に対応する複製先の節点」を何度も引くため、そのたびに全節点を
/// 走査すると節点数の 2 乗に比例する。正規化した座標をキーにした逆引きを 1 度だけ
/// 作り、以降は定数時間で引く。
struct CoordIndex {
    /// 標高の代表値。複製先の節点を標高差から引くときに、値がどの代表へ属するかを
    /// 探す必要があるため、Z だけは代表列を持ち続ける。
    z: Axis1d,
    /// 節点ごとの正規化キー（`model.nodes` と同順）。
    keys: Vec<(usize, usize, usize)>,
    /// 正規化キー → 節点（同じ位置に節点が複数あれば先に現れたもの）。
    by_key: HashMap<(usize, usize, usize), NodeId>,
}

impl CoordIndex {
    fn build(model: &Model) -> Self {
        let x = Axis1d::build(model.nodes.iter().map(|n| n.coord[0]));
        let y = Axis1d::build(model.nodes.iter().map(|n| n.coord[1]));
        let z = Axis1d::build(model.nodes.iter().map(|n| n.coord[2]));
        let mut keys = Vec::with_capacity(model.nodes.len());
        let mut by_key = HashMap::new();
        for n in &model.nodes {
            // 代表列は全節点の座標から作ったので、必ず見つかる。
            let k = (
                x.find(n.coord[0]).unwrap_or(0),
                y.find(n.coord[1]).unwrap_or(0),
                z.find(n.coord[2]).unwrap_or(0),
            );
            keys.push(k);
            by_key.entry(k).or_insert(n.id);
        }
        Self { z, keys, by_key }
    }

    /// 節点の平面位置（正規化した XY の番号）。
    fn plan(&self, n: NodeId) -> Option<(usize, usize)> {
        self.keys.get(n.index()).map(|&(x, y, _)| (x, y))
    }

    /// 同じ平面位置で標高が `z` の節点。
    fn at(&self, plan: (usize, usize), z: f64) -> Option<NodeId> {
        let zi = self.z.find(z)?;
        self.by_key.get(&(plan.0, plan.1, zi)).copied()
    }
}

/// 材端・頂点 1 点分のキー。正規化した XY の番号と、階の中での高さの位置。
type PointKey = (usize, usize, i64);

/// 部材・床・二次部材の対応付けキー（材端／頂点の並び。順序の違いを吸収するため整列）。
type PlanKey = Vec<PointKey>;

/// 階の中での高さの位置。直下階のレベルを 0、当該階のレベルを 1000 とし、
/// あいだは階高に対する比を 1/1000 で量子化する。
///
/// 材端の XY だけでキーを作ると、同じ構面に投影される部材を区別できない。
/// 1FL→2FL のブレースと 2FL の大梁はどちらも XY が `[(0,0), (6000,0)]` になり、
/// 同じ構面の X ブレース 2 本も互いに区別できない。高さの位置を足すと、
/// 大梁は両端が 1000、ブレースは 0 と 1000 になり、X ブレースは XY との組が
/// 入れ替わるため区別できる。
///
/// 絶対の高さではなく比で持つのは、階高の違う階へも対応を取るためである。
fn level_tag(z: f64, bottom: f64, top: f64) -> i64 {
    if (z - bottom).abs() <= PLAN_TOL_MM {
        return 0;
    }
    if (z - top).abs() <= PLAN_TOL_MM {
        return 1000;
    }
    let h = top - bottom;
    if h.abs() < 1e-9 {
        return 500;
    }
    (((z - bottom) / h) * 1000.0).round().clamp(1.0, 999.0) as i64
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
    ///
    /// 新しく作る断面の一覧も同じ結果（[`CopyStoryReport::created_sections`]）へ含める。
    /// モデルを丸ごと複製して試算するため、一覧のためだけにもう一度走らせない。
    pub fn preview(&self, model: &Model) -> CopyStoryReport {
        let mut probe = model.clone();
        copy_into(&mut probe, self)
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

/// 対応付けの索引。同じキーの相手が 2 つ以上あるキーは、どれを選ぶべきか決められない
/// ため索引から外し、`ambiguous` へ入れる。
///
/// 先に見つかったものを採ると、どれが選ばれるかは要素の並び順しだいになる。
/// 誤って別の部材へ断面や荷重を配るより、飛ばして件数を報告するほうが安全である。
struct PlanIndex<T> {
    map: HashMap<PlanKey, T>,
    ambiguous: HashSet<PlanKey>,
}

impl<T> PlanIndex<T> {
    fn build(items: impl Iterator<Item = (PlanKey, T)>) -> Self {
        let mut map: HashMap<PlanKey, T> = HashMap::new();
        let mut ambiguous = HashSet::new();
        for (k, v) in items {
            if map.remove(&k).is_some() || ambiguous.contains(&k) {
                ambiguous.insert(k);
                continue;
            }
            map.insert(k, v);
        }
        Self { map, ambiguous }
    }

    /// キーに対応する相手。曖昧なキーは `None`（呼び出し側は飛ばして数える）。
    fn get(&self, k: &PlanKey) -> Option<&T> {
        self.map.get(k)
    }

    fn is_ambiguous(&self, k: &PlanKey) -> bool {
        self.ambiguous.contains(k)
    }
}

/// 複製 1 回のあいだ使い回す文脈（座標の索引と階の区間）。
struct Ctx {
    coords: CoordIndex,
    /// 階ごとの帰属区間 `(下端, 上端)`（`model.stories` と同順）。
    spans: Vec<(f64, f64)>,
}

impl Ctx {
    fn build(model: &Model) -> Self {
        Self {
            coords: CoordIndex::build(model),
            spans: model.story_spans(),
        }
    }

    /// 材端・頂点の並びから対応付けキーを作る（順序の違いを吸収するため整列）。
    fn key(&self, model: &Model, story: StoryId, nodes: &[NodeId]) -> Option<PlanKey> {
        let (bottom, top) = *self.spans.get(story.index())?;
        let mut keys: Vec<PointKey> = nodes
            .iter()
            .map(|n| {
                let plan = self.coords.plan(*n)?;
                let z = model.nodes.get(n.index())?.coord[2];
                Some((plan.0, plan.1, level_tag(z, bottom, top)))
            })
            .collect::<Option<Vec<_>>>()?;
        keys.sort_unstable();
        Some(keys)
    }

    /// 複製元の節点に対応する複製先の節点（同じ平面位置で標高差 `dz`）。
    fn mapped_node(&self, model: &Model, src: NodeId, dz: f64) -> Option<NodeId> {
        let plan = self.coords.plan(src)?;
        let z = model.nodes.get(src.index())?.coord[2] + dz;
        self.coords.at(plan, z)
    }

    /// 複製先の節点が、複製元の階にも対応する節点を持つか。
    ///
    /// 削除の判断対象を「複製元の平面の内側」に限るために使う。複製元の平面の外に
    /// ある床・二次部材は、複製元に「無い」のではなく複製元の範囲外なので消さない。
    fn maps_back(&self, model: &Model, dst: NodeId, dz: f64) -> bool {
        self.mapped_node(model, dst, -dz).is_some()
    }
}

/// 複製の本体。`model` を書き換えて結果を返す。
fn copy_into(model: &mut Model, cmd: &CopyStory) -> CopyStoryReport {
    let mut report = CopyStoryReport::default();
    // 複製は節点を作らず消さないため、座標の索引は 1 度だけ作れば足りる。
    let ctx = Ctx::build(model);
    for &to in &cmd.to {
        if to == cmd.from {
            continue;
        }
        copy_one(model, &ctx, cmd, to, &mut report);
    }
    report.mismatched_sections.sort();
    report.mismatched_sections.dedup();
    report.created_sections.dedup();
    report
}

fn copy_one(
    model: &mut Model,
    ctx: &Ctx,
    cmd: &CopyStory,
    to: StoryId,
    report: &mut CopyStoryReport,
) {
    let from = cmd.from;
    let (Some(src_story), Some(dst_story)) = (
        model.stories.get(from.index()).cloned(),
        model.stories.get(to.index()).cloned(),
    ) else {
        return;
    };
    let dz = dst_story.elevation - src_story.elevation;

    // 形（床・二次部材）を先に整えてから、断面の割当と荷重を配る。断面の割当は
    // 部材だけでなく床・二次部材も受け持つため、対象がそろってから走らせる。
    let created_slabs = if cmd.targets.slabs {
        copy_slabs(model, ctx, cmd, to, dz, &dst_story.name, report)
    } else {
        Vec::new()
    };
    if cmd.targets.secondary {
        copy_secondary(model, ctx, cmd, to, dz, &dst_story.name, report);
    }
    if cmd.targets.sections {
        copy_sections(model, ctx, cmd, to, &dst_story.name, report);
    }
    if cmd.targets.loads {
        copy_slab_loads(model, ctx, cmd, to, &created_slabs, report);
        copy_case_loads(model, ctx, cmd, to, dz, report);
    }
}

/// 階に属する部材を、対応付けキーで引ける索引にする。
fn members_by_plan(model: &Model, ctx: &Ctx, story: StoryId) -> PlanIndex<ElemId> {
    PlanIndex::build(model.elements.iter().filter_map(|e| {
        (model.member_story(e) == Some(story))
            .then(|| Some((ctx.key(model, story, &e.nodes)?, e.id)))
            .flatten()
    }))
}

/// 階の床板を、対応付けキーで引ける索引にする。
fn slabs_by_plan(model: &Model, ctx: &Ctx, story: StoryId) -> PlanIndex<SlabId> {
    PlanIndex::build(model.slabs.iter().filter_map(|sl| {
        (slab_story(model, sl) == Some(story))
            .then(|| Some((ctx.key(model, story, sl.boundary_nodes()?)?, sl.id)))
            .flatten()
    }))
}

/// 二次部材の格納位置（D6）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum SecondarySlot {
    UnassignedJoist(usize),
    UnassignedPost(usize),
    FloorJoist { region: usize, index: usize },
    WallPost { region: usize, index: usize },
}

fn secondary_at(model: &Model, slot: SecondarySlot) -> Option<&SecondaryMember> {
    match slot {
        SecondarySlot::UnassignedJoist(i) => model.unassigned_joists.get(i),
        SecondarySlot::UnassignedPost(i) => model.unassigned_posts.get(i),
        SecondarySlot::FloorJoist { region, index } => {
            model.floor_regions.get(region)?.secondary_joists.get(index)
        }
        SecondarySlot::WallPost { region, index } => {
            model.wall_regions.get(region)?.posts.get(index)
        }
    }
}

fn secondary_at_mut(model: &mut Model, slot: SecondarySlot) -> Option<&mut SecondaryMember> {
    match slot {
        SecondarySlot::UnassignedJoist(i) => model.unassigned_joists.get_mut(i),
        SecondarySlot::UnassignedPost(i) => model.unassigned_posts.get_mut(i),
        SecondarySlot::FloorJoist { region, index } => model
            .floor_regions
            .get_mut(region)?
            .secondary_joists
            .get_mut(index),
        SecondarySlot::WallPost { region, index } => {
            model.wall_regions.get_mut(region)?.posts.get_mut(index)
        }
    }
}

fn all_secondary_slots(model: &Model) -> Vec<SecondarySlot> {
    let mut out = Vec::new();
    for i in 0..model.unassigned_joists.len() {
        out.push(SecondarySlot::UnassignedJoist(i));
    }
    for i in 0..model.unassigned_posts.len() {
        out.push(SecondarySlot::UnassignedPost(i));
    }
    for (ri, region) in model.floor_regions.iter().enumerate() {
        for ji in 0..region.secondary_joists.len() {
            out.push(SecondarySlot::FloorJoist {
                region: ri,
                index: ji,
            });
        }
    }
    for (ri, region) in model.wall_regions.iter().enumerate() {
        for pi in 0..region.posts.len() {
            out.push(SecondarySlot::WallPost {
                region: ri,
                index: pi,
            });
        }
    }
    out
}

fn secondary_count(model: &Model) -> usize {
    model.unassigned_joists.len()
        + model.unassigned_posts.len()
        + model
            .floor_regions
            .iter()
            .map(|r| r.secondary_joists.len())
            .sum::<usize>()
        + model
            .wall_regions
            .iter()
            .map(|r| r.posts.len())
            .sum::<usize>()
}

/// 階の二次部材を、対応付けキーで引ける索引にする。
fn secondary_by_plan(model: &Model, ctx: &Ctx, story: StoryId) -> PlanIndex<SecondarySlot> {
    PlanIndex::build(all_secondary_slots(model).into_iter().filter_map(|slot| {
        let sm = secondary_at(model, slot)?;
        (secondary_story(model, sm) == Some(story))
            .then(|| ctx.key(model, story, &sm.nodes).map(|k| (k, slot)))
            .flatten()
    }))
}

/// 床板の所属階（参照する節点のうちもっとも高い節点の所属階。部材と同じ規則）。
///
/// 取り付く床板は自由端に節点を持たないため、取付き先の節点で判定する。
/// ここで `None` を返すと複製の対象からも見送り件数からも外れ、利用者からは
/// 「複製したのに床が足りない」理由が見えなくなる。
fn slab_story(model: &Model, slab: &Slab) -> Option<StoryId> {
    let refs: Vec<squid_n_core::ids::NodeId> = match slab.boundary_nodes() {
        Some(b) => b.to_vec(),
        None => slab.reference_node().into_iter().collect(),
    };
    refs.iter()
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

/// 断面の割当を配る（部材・床・二次部材）。
///
/// 上書きが真のときは、複製元が断面を持たない相手の割当を解除する。
/// 複製元に相手がいないもの（複製元の平面の外）には触れない。
fn copy_sections(
    model: &mut Model,
    ctx: &Ctx,
    cmd: &CopyStory,
    to: StoryId,
    dst_story_name: &str,
    report: &mut CopyStoryReport,
) {
    // 複製元の断面 → 複製先の断面。同じ組は 1 回だけ作る。
    let mut mapped: HashMap<SectionId, SectionId> = HashMap::new();

    // --- 部材 ---
    let dst = members_by_plan(model, ctx, to);
    let src: Vec<(PlanKey, Option<SectionId>)> = model
        .elements
        .iter()
        .filter(|e| model.member_story(e) == Some(cmd.from))
        .filter_map(|e| Some((ctx.key(model, cmd.from, &e.nodes)?, e.section)))
        .collect();
    for (key, src_sec) in src {
        let Some(&elem) = dst.get(&key) else {
            report.skipped += 1;
            continue;
        };
        let current = model.elements.get(elem.index()).and_then(|e| e.section);
        let Some(next) = resolve_section(
            model,
            cmd,
            &mut mapped,
            src_sec,
            current,
            dst_story_name,
            report,
        ) else {
            continue;
        };
        if let Some(e) = model.elements.get_mut(elem.index()) {
            if e.section != next {
                count_section_change(e.section, next, report);
                e.section = next;
            }
        }
    }

    // --- 床 ---
    let dst_slabs = slabs_by_plan(model, ctx, to);
    let src: Vec<(PlanKey, Option<SectionId>)> = model
        .slabs
        .iter()
        .filter(|sl| slab_story(model, sl) == Some(cmd.from))
        .filter_map(|sl| {
            Some((
                ctx.key(model, cmd.from, sl.boundary_nodes()?)?,
                sl.section(),
            ))
        })
        .collect();
    for (key, src_sec) in src {
        let Some(&sid) = dst_slabs.get(&key) else {
            continue;
        };
        let current = model.slabs.get(sid.index()).and_then(|sl| sl.section());
        let Some(next) = resolve_section(
            model,
            cmd,
            &mut mapped,
            src_sec,
            current,
            dst_story_name,
            report,
        ) else {
            continue;
        };
        if let Some(sl) = model.slabs.get_mut(sid.index()) {
            if sl.plate.section != next {
                count_section_change(sl.plate.section, next, report);
                sl.plate.section = next;
            }
        }
    }

    // --- 二次部材 ---
    let dst_sec = secondary_by_plan(model, ctx, to);
    let src: Vec<(PlanKey, Option<SectionId>)> = all_secondary_slots(model)
        .into_iter()
        .filter_map(|slot| {
            let sm = secondary_at(model, slot)?;
            (secondary_story(model, sm) == Some(cmd.from))
                .then(|| Some((ctx.key(model, cmd.from, &sm.nodes)?, sm.section)))
                .flatten()
        })
        .collect();
    for (key, src_sec) in src {
        let Some(&slot) = dst_sec.get(&key) else {
            continue;
        };
        let current = secondary_at(model, slot).and_then(|sm| sm.section);
        let Some(next) = resolve_section(
            model,
            cmd,
            &mut mapped,
            src_sec,
            current,
            dst_story_name,
            report,
        ) else {
            continue;
        };
        let Some(sm) = secondary_at_mut(model, slot) else {
            continue;
        };
        if sm.section != next {
            count_section_change(sm.section, next, report);
            sm.section = next;
        }
    }
}

/// 複製元の断面参照を、複製先へ割り当てる参照へ読み替える。
///
/// 上書きしない設定で複製先に既に断面が付いている場合は `None`（触れない）。
fn resolve_section(
    model: &mut Model,
    cmd: &CopyStory,
    mapped: &mut HashMap<SectionId, SectionId>,
    src_sec: Option<SectionId>,
    current: Option<SectionId>,
    dst_story_name: &str,
    report: &mut CopyStoryReport,
) -> Option<Option<SectionId>> {
    if !cmd.overwrite && current.is_some() {
        return None;
    }
    let Some(src_sec) = src_sec else {
        // 複製元が未割当。上書きなら複製先も未割当へそろえる。
        return cmd.overwrite.then_some(None);
    };
    let dst = match mapped.get(&src_sec) {
        Some(&s) => s,
        None => {
            let s = section_for_story(model, src_sec, dst_story_name, report);
            mapped.insert(src_sec, s);
            s
        }
    };
    Some(Some(dst))
}

/// 断面の割当・解除の件数を数える。
fn count_section_change(
    before: Option<SectionId>,
    after: Option<SectionId>,
    report: &mut CopyStoryReport,
) {
    match after {
        Some(_) => report.sections_assigned += 1,
        None if before.is_some() => report.sections_cleared += 1,
        None => {}
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
    let created = Section {
        id,
        floor: Some(dst_story_name.to_string()),
        ..src_sec
    };
    report.created_sections.push(created.display_name());
    model.sections.push(created);
    report.sections_created += 1;
    id
}

/// 床板（境界の形）を配る。新しく作った床板の ID を返す。
///
/// 上書きが真のときは、複製元に同じ位置の床板が無い複製先の床板を削除する。ただし
/// 境界節点すべてが複製元の階へ対応するものに限る（[`Ctx::maps_back`]）。
/// 所属する床領域（大梁の区画）は次の準備計算（`rebuild_floor_regions`）が
/// 自動で結びつけるため、ここでは床板だけを配る。
fn copy_slabs(
    model: &mut Model,
    ctx: &Ctx,
    cmd: &CopyStory,
    to: StoryId,
    dz: f64,
    dst_story_name: &str,
    report: &mut CopyStoryReport,
) -> Vec<SlabId> {
    let src_keys: HashSet<PlanKey> = model
        .slabs
        .iter()
        .filter(|sl| slab_story(model, sl) == Some(cmd.from))
        .filter_map(|sl| ctx.key(model, cmd.from, sl.boundary_nodes()?))
        .collect();

    // 先に削除する（複製元に無い床を消してから、複製元の床を作る）。
    if cmd.overwrite {
        let doomed: Vec<SlabId> = model
            .slabs
            .iter()
            .filter(|sl| slab_story(model, sl) == Some(to))
            .filter(|sl| {
                let Some(boundary) = sl.boundary_nodes() else {
                    return false; // 取り付く床板は平面キーで対応付けない（複製の対象外）。
                };
                ctx.key(model, to, boundary)
                    .is_some_and(|k| !src_keys.contains(&k))
                    && boundary.iter().all(|&n| ctx.maps_back(model, n, dz))
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

    let existing = slabs_by_plan(model, ctx, to);
    let src: Vec<Slab> = model
        .slabs
        .iter()
        .filter(|sl| slab_story(model, sl) == Some(cmd.from))
        .cloned()
        .collect();
    let mut mapped: HashMap<SectionId, SectionId> = HashMap::new();
    let mut created = Vec::new();
    for sl in src {
        let Some(src_boundary) = sl.boundary_nodes().map(|b| b.to_vec()) else {
            // 取り付く床板（片持ち・バルコニー・出隅）は複製しない。
            //
            // 対応付けは境界節点の平面キーで行うが、取り付く床板は自由端に節点を持たず、
            // 取付き先の節点だけでは同じ形の床板を作れない（張り出し量・区間・荷重の
            // 出口まで写す必要がある）。**黙って落とすと複製先で床荷重が欠ける**ため、
            // 見送った件数として報告へ数える。
            report.skipped += 1;
            continue;
        };
        let Some(key) = ctx.key(model, cmd.from, &src_boundary) else {
            report.skipped += 1;
            continue;
        };
        if existing.get(&key).is_some() {
            continue;
        }
        if existing.is_ambiguous(&key) {
            report.skipped += 1;
            continue;
        }
        let Some(boundary) = src_boundary
            .iter()
            .map(|n| ctx.mapped_node(model, *n, dz))
            .collect::<Option<Vec<_>>>()
        else {
            report.skipped += 1;
            continue;
        };
        // 断面の参照は複製先の階の断面へ読み替える（符号＋階の識別に合わせる）。
        let section = sl.section().map(|s| match mapped.get(&s) {
            Some(&d) => d,
            None => {
                let d = section_for_story(model, s, dst_story_name, report);
                mapped.insert(s, d);
                d
            }
        });
        let id = SlabId(model.slabs.len() as u32);
        model.slabs.push(Slab {
            id,
            shape: SlabShape::Enclosed { boundary },
            plate: SlabPlate {
                section,
                method: sl.method(),
                one_way: sl.one_way(),
                ..Default::default()
            },
        });
        created.push(id);
        report.slabs_created += 1;
    }
    created
}

/// 床の面荷重・用途を配る（「荷重」の対象。床板の形は `copy_slabs` が受け持つ）。
///
/// `created` は同じ操作で作ったばかりの床板のため、「更新」には数えない（数えると
/// 1 枚の床板が「新規」と「更新」で二重に報告される）。床板の削除が `SlabId` を
/// 繰り上げるため、添字の閾値ではなく ID の集合で見分ける。
fn copy_slab_loads(
    model: &mut Model,
    ctx: &Ctx,
    cmd: &CopyStory,
    to: StoryId,
    created: &[SlabId],
    report: &mut CopyStoryReport,
) {
    let dst = slabs_by_plan(model, ctx, to);
    let src: Vec<(
        PlanKey,
        Vec<squid_n_core::model::AreaLoad>,
        Option<SlabUsage>,
    )> = model
        .slabs
        .iter()
        .filter(|sl| slab_story(model, sl) == Some(cmd.from))
        .filter_map(|sl| {
            Some((
                ctx.key(model, cmd.from, sl.boundary_nodes()?)?,
                sl.plate.loads.clone(),
                sl.plate.usage,
            ))
        })
        .collect();
    for (key, loads, usage) in src {
        let Some(&sid) = dst.get(&key) else {
            report.skipped += 1;
            continue;
        };
        let is_new = created.contains(&sid);
        let Some(sl) = model.slabs.get_mut(sid.index()) else {
            continue;
        };
        let plate = &mut sl.plate;
        // 上書きしない設定では、既に面荷重・用途が入っている床には触れない。
        if !cmd.overwrite && !is_new && (!plate.loads.is_empty() || plate.usage.is_some()) {
            continue;
        }
        let changed = plate.loads != loads || plate.usage != usage;
        plate.loads = loads;
        plate.usage = usage;
        if changed && !is_new {
            report.slabs_updated += 1;
        }
    }
}

fn should_delete_copied_secondary(
    model: &Model,
    ctx: &Ctx,
    _cmd: &CopyStory,
    to: StoryId,
    dz: f64,
    sm: &SecondaryMember,
    src_keys: &HashSet<PlanKey>,
) -> bool {
    if secondary_story(model, sm) != Some(to) {
        return false;
    }
    let unmatched = ctx
        .key(model, to, &sm.nodes)
        .is_some_and(|k| !src_keys.contains(&k));
    let in_src_plan = sm.nodes.iter().all(|&n| ctx.maps_back(model, n, dz));
    unmatched && in_src_plan
}

/// 二次部材（小梁・間柱）の形を配る。断面は `copy_sections` が受け持つ。
///
/// 上書きが真のときは、複製元に同じ位置の二次部材が無い複製先の二次部材を削除する
/// （材端節点が複製元の階へ対応するものに限る）。新規複製分は未割当リストへ入れ、
/// 次回の領域リビルドで D7 により帰属が決まる。
fn copy_secondary(
    model: &mut Model,
    ctx: &Ctx,
    cmd: &CopyStory,
    to: StoryId,
    dz: f64,
    dst_story_name: &str,
    report: &mut CopyStoryReport,
) {
    let src_keys: HashSet<PlanKey> = all_secondary_slots(model)
        .into_iter()
        .filter_map(|slot| {
            let sm = secondary_at(model, slot)?;
            (secondary_story(model, sm) == Some(cmd.from))
                .then(|| ctx.key(model, cmd.from, &sm.nodes))
                .flatten()
        })
        .collect();

    if cmd.overwrite {
        let before = secondary_count(model);
        let delete_joists: Vec<bool> = model
            .unassigned_joists
            .iter()
            .map(|sm| should_delete_copied_secondary(model, ctx, cmd, to, dz, sm, &src_keys))
            .collect();
        let mut ji = 0usize;
        model.unassigned_joists.retain(|_| {
            let keep = !delete_joists[ji];
            ji += 1;
            keep
        });
        let delete_posts: Vec<bool> = model
            .unassigned_posts
            .iter()
            .map(|sm| should_delete_copied_secondary(model, ctx, cmd, to, dz, sm, &src_keys))
            .collect();
        let mut pi = 0usize;
        model.unassigned_posts.retain(|_| {
            let keep = !delete_posts[pi];
            pi += 1;
            keep
        });
        let region_joist_deletes: Vec<Vec<bool>> = model
            .floor_regions
            .iter()
            .map(|region| {
                region
                    .secondary_joists
                    .iter()
                    .map(|sm| {
                        should_delete_copied_secondary(model, ctx, cmd, to, dz, sm, &src_keys)
                    })
                    .collect()
            })
            .collect();
        for (region, delete) in model.floor_regions.iter_mut().zip(region_joist_deletes) {
            let mut k = 0usize;
            region.secondary_joists.retain(|_| {
                let keep = !delete[k];
                k += 1;
                keep
            });
        }
        let region_post_deletes: Vec<Vec<bool>> = model
            .wall_regions
            .iter()
            .map(|region| {
                region
                    .posts
                    .iter()
                    .map(|sm| {
                        should_delete_copied_secondary(model, ctx, cmd, to, dz, sm, &src_keys)
                    })
                    .collect()
            })
            .collect();
        for (region, delete) in model.wall_regions.iter_mut().zip(region_post_deletes) {
            let mut k = 0usize;
            region.posts.retain(|_| {
                let keep = !delete[k];
                k += 1;
                keep
            });
        }
        report.secondary_deleted += before - secondary_count(model);
    }

    let existing = secondary_by_plan(model, ctx, to);
    let src: Vec<SecondaryMember> = all_secondary_slots(model)
        .into_iter()
        .filter_map(|slot| secondary_at(model, slot).cloned())
        .filter(|sm| secondary_story(model, sm) == Some(cmd.from))
        .collect();
    let mut mapped: HashMap<SectionId, SectionId> = HashMap::new();
    for sm in src {
        let Some(key) = ctx.key(model, cmd.from, &sm.nodes) else {
            report.skipped += 1;
            continue;
        };
        if existing.get(&key).is_some() {
            continue;
        }
        if existing.is_ambiguous(&key) {
            report.skipped += 1;
            continue;
        }
        let (Some(a), Some(b)) = (
            ctx.mapped_node(model, sm.nodes[0], dz),
            ctx.mapped_node(model, sm.nodes[1], dz),
        ) else {
            report.skipped += 1;
            continue;
        };
        let section = sm.section.map(|s| match mapped.get(&s) {
            Some(&d) => d,
            None => {
                let d = section_for_story(model, s, dst_story_name, report);
                mapped.insert(s, d);
                d
            }
        });
        let new_sm = SecondaryMember {
            kind: sm.kind,
            nodes: [a, b],
            section,
            name: sm.name.clone(),
        };
        match sm.kind {
            SecondaryMemberKind::Joist => model.unassigned_joists.push(new_sm),
            SecondaryMemberKind::Post => model.unassigned_posts.push(new_sm),
        }
        report.secondary_created += 1;
    }
}

/// 部材荷重を複製先の材長へ合わせる。合わせられない場合は `None`（配らない）。
///
/// 載荷位置は i 端からの mm の絶対位置である。同じ平面位置で突き合わせるため大梁の
/// 材長は一致するが、柱は階高が違えば材長も違う。そのまま写すと載荷区間が材長を
/// 超え、等価節点力の積分が形状関数を材外へ外挿して結果が黙って誤る。
///
/// - **全長載荷**（`a≈0` かつ `b≈L`）は複製先の材長へ合わせる。外壁荷重のような
///   全長等分布は「材長いっぱい」という意図が明確なため。
/// - **部分載荷・集中荷重**は位置をそのまま写す。i 端から 2 m といった位置には
///   絶対の意味があり、材長比で按分すると利用者の意図から外れる。
/// - 新しい材長に**収まらないもの**は配らない。縮めると区間長が黙って変わる。
fn fit_member_load(
    kind: squid_n_core::model::MemberLoadKind,
    src_len: f64,
    dst_len: f64,
) -> Option<squid_n_core::model::MemberLoadKind> {
    use squid_n_core::model::MemberLoadKind;
    match kind {
        MemberLoadKind::Point { a, p } => {
            (a <= dst_len + PLAN_TOL_MM).then_some(MemberLoadKind::Point { a, p })
        }
        MemberLoadKind::Distributed { a, b, w1, w2 } => {
            if a.abs() <= PLAN_TOL_MM && (b - src_len).abs() <= PLAN_TOL_MM {
                return Some(MemberLoadKind::Distributed {
                    a: 0.0,
                    b: dst_len,
                    w1,
                    w2,
                });
            }
            (b <= dst_len + PLAN_TOL_MM).then_some(MemberLoadKind::Distributed { a, b, w1, w2 })
        }
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
    ctx: &Ctx,
    cmd: &CopyStory,
    to: StoryId,
    dz: f64,
    report: &mut CopyStoryReport,
) {
    let dst_members = members_by_plan(model, ctx, to);
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
        .filter_map(|n| Some((n, ctx.mapped_node(model, n, dz)?)))
        .collect();
    // 複製元の部材 → (複製先の部材, 複製元の材長, 複製先の材長)。
    let elem_map: HashMap<ElemId, (ElemId, f64, f64)> = model
        .elements
        .iter()
        .filter(|e| model.member_story(e) == Some(cmd.from))
        .filter_map(|e| {
            let k = ctx.key(model, cmd.from, &e.nodes)?;
            let &dst = dst_members.get(&k)?;
            let dst_elem = model.elements.get(dst.index())?;
            Some((
                e.id,
                (dst, model.member_length(e), model.member_length(dst_elem)),
            ))
        })
        .collect();
    // 手を触れてよい複製先（複製元に相手がある節点・部材）。
    let dst_nodes: HashSet<NodeId> = node_map.values().copied().collect();
    let dst_elems: HashSet<ElemId> = elem_map.values().map(|(e, _, _)| *e).collect();

    let mut unfit = 0usize;
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
            let Some(&(e, src_len, dst_len)) = elem_map.get(&ml.elem) else {
                continue;
            };
            match fit_member_load(ml.kind.clone(), src_len, dst_len) {
                Some(kind) => add_member.push(squid_n_core::model::MemberLoad {
                    elem: e,
                    kind,
                    ..ml.clone()
                }),
                None => unfit += 1,
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
    report.loads_unfit += unfit;
}
