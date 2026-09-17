//! 二次部材（小梁・間柱）と壁領域の編集コマンド（D6: 領域内実体）。

use super::*;
use squid_n_core::ids::*;
use squid_n_core::model::{
    EndSupport, SecondaryMember, SecondaryMemberAnchor, SecondaryMemberEnds, SecondaryMemberKind,
};
use std::collections::HashSet;

fn secondary_member_ok(model: &Model, sm: &SecondaryMember) -> bool {
    crate::refs::section_ref_ok(model, sm.section)
        && model
            .secondary_member_axis(sm)
            .is_some_and(|(_, _, len)| len > 1e-9)
}

fn joists_ok(joists: &[SecondaryMember]) -> bool {
    joists
        .iter()
        .all(|sm| sm.kind == SecondaryMemberKind::Joist)
}

fn posts_ok(posts: &[SecondaryMember]) -> bool {
    posts.iter().all(|sm| sm.kind == SecondaryMemberKind::Post)
}

fn unique_ids(sms: &[SecondaryMember]) -> bool {
    let mut seen = HashSet::new();
    sms.iter().all(|sm| seen.insert(sm.id))
}

fn joist_id_in_other_regions(model: &Model, id: SecondaryMemberId, skip: FloorRegionId) -> bool {
    model
        .floor_regions
        .iter()
        .any(|r| r.id != skip && r.secondary_joists.iter().any(|sm| sm.id == id))
}

fn post_id_in_other_regions(model: &Model, id: SecondaryMemberId, skip: WallRegionId) -> bool {
    model
        .wall_regions
        .iter()
        .any(|r| r.id != skip && r.posts.iter().any(|sm| sm.id == id))
}

fn relocate_removed(
    old: &[SecondaryMember],
    new_ids: &HashSet<SecondaryMemberId>,
    unassigned: &mut Vec<SecondaryMember>,
) {
    for sm in old {
        if new_ids.contains(&sm.id) {
            continue;
        }
        if unassigned.iter().any(|u| u.id == sm.id) {
            continue;
        }
        unassigned.push(sm.clone());
    }
}

fn take_from_unassigned(
    unassigned: &mut Vec<SecondaryMember>,
    new_ids: &HashSet<SecondaryMemberId>,
) {
    unassigned.retain(|sm| !new_ids.contains(&sm.id));
}

/// 未割当小梁を末尾へ追加する。逆操作は [`DeleteUnassignedJoist`]。
pub struct AddUnassignedJoist {
    pub sm: SecondaryMember,
}

impl EditCommand for AddUnassignedJoist {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.sm.kind != SecondaryMemberKind::Joist || !secondary_member_ok(model, &self.sm) {
            return Box::new(Noop);
        }
        if model.joists().any(|sm| sm.id == self.sm.id) {
            return Box::new(Noop);
        }
        let index = model.unassigned_joists.len();
        model.unassigned_joists.push(self.sm.clone());
        Box::new(DeleteUnassignedJoist { index })
    }

    fn label(&self) -> &str {
        "未割当小梁追加"
    }
}

/// 未割当小梁を削除する。逆操作は [`InsertUnassignedJoist`]。
pub struct DeleteUnassignedJoist {
    pub index: usize,
}

impl EditCommand for DeleteUnassignedJoist {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index >= model.unassigned_joists.len() {
            return Box::new(Noop);
        }
        let removed = model.unassigned_joists.remove(self.index);
        Box::new(InsertUnassignedJoist {
            index: self.index,
            sm: removed,
        })
    }

    fn label(&self) -> &str {
        "未割当小梁削除"
    }
}

/// [`DeleteUnassignedJoist`] の逆操作。
pub struct InsertUnassignedJoist {
    pub index: usize,
    pub sm: SecondaryMember,
}

impl EditCommand for InsertUnassignedJoist {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.unassigned_joists.len() {
            return Box::new(Noop);
        }
        model.unassigned_joists.insert(self.index, self.sm.clone());
        Box::new(DeleteUnassignedJoist { index: self.index })
    }

    fn label(&self) -> &str {
        "未割当小梁削除の取り消し"
    }
}

/// 未割当間柱を末尾へ追加する。
pub struct AddUnassignedPost {
    pub sm: SecondaryMember,
}

impl EditCommand for AddUnassignedPost {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.sm.kind != SecondaryMemberKind::Post || !secondary_member_ok(model, &self.sm) {
            return Box::new(Noop);
        }
        if model.posts().any(|sm| sm.id == self.sm.id) {
            return Box::new(Noop);
        }
        let index = model.unassigned_posts.len();
        model.unassigned_posts.push(self.sm.clone());
        Box::new(DeleteUnassignedPost { index })
    }

    fn label(&self) -> &str {
        "未割当間柱追加"
    }
}

/// 未割当間柱を削除する。
pub struct DeleteUnassignedPost {
    pub index: usize,
}

impl EditCommand for DeleteUnassignedPost {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index >= model.unassigned_posts.len() {
            return Box::new(Noop);
        }
        let removed = model.unassigned_posts.remove(self.index);
        Box::new(InsertUnassignedPost {
            index: self.index,
            sm: removed,
        })
    }

    fn label(&self) -> &str {
        "未割当間柱削除"
    }
}

/// [`DeleteUnassignedPost`] の逆操作。
pub struct InsertUnassignedPost {
    pub index: usize,
    pub sm: SecondaryMember,
}

impl EditCommand for InsertUnassignedPost {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.unassigned_posts.len() {
            return Box::new(Noop);
        }
        model.unassigned_posts.insert(self.index, self.sm.clone());
        Box::new(DeleteUnassignedPost { index: self.index })
    }

    fn label(&self) -> &str {
        "未割当間柱削除の取り消し"
    }
}

/// 床領域の小梁リスト（`secondary_joists`）を全置換する。
///
/// 新しいリストに無い旧所属は未割当へ移す（実体を消さない）。
/// 未割当にあった同じ端点は領域側へ移す。他領域との端点重複は Noop。
/// 次回の準備計算（`rebuild_floor_regions`）で D7 により幾何から入れ直される。
pub struct SetFloorRegionSecondaryJoists {
    pub region: FloorRegionId,
    pub joists: Vec<SecondaryMember>,
}

struct RestoreFloorRegionSecondaryJoists {
    region: FloorRegionId,
    joists: Vec<SecondaryMember>,
    unassigned: Vec<SecondaryMember>,
}

impl EditCommand for RestoreFloorRegionSecondaryJoists {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.region.index();
        if idx >= model.floor_regions.len() || model.floor_regions[idx].id != self.region {
            return Box::new(Noop);
        }
        let old_joists = std::mem::replace(
            &mut model.floor_regions[idx].secondary_joists,
            self.joists.clone(),
        );
        let old_unassigned =
            std::mem::replace(&mut model.unassigned_joists, self.unassigned.clone());
        Box::new(RestoreFloorRegionSecondaryJoists {
            region: self.region,
            joists: old_joists,
            unassigned: old_unassigned,
        })
    }

    fn label(&self) -> &str {
        "床領域小梁リスト変更の取り消し"
    }
}

impl EditCommand for SetFloorRegionSecondaryJoists {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.region.index();
        if idx >= model.floor_regions.len() || model.floor_regions[idx].id != self.region {
            return Box::new(Noop);
        }
        if !joists_ok(&self.joists)
            || !self.joists.iter().all(|sm| secondary_member_ok(model, sm))
            || !unique_ids(&self.joists)
        {
            return Box::new(Noop);
        }
        if self
            .joists
            .iter()
            .any(|sm| joist_id_in_other_regions(model, sm.id, self.region))
        {
            return Box::new(Noop);
        }
        let new_keys: HashSet<_> = self.joists.iter().map(|sm| sm.id).collect();
        let old_joists = std::mem::replace(
            &mut model.floor_regions[idx].secondary_joists,
            self.joists.clone(),
        );
        let old_unassigned = model.unassigned_joists.clone();
        take_from_unassigned(&mut model.unassigned_joists, &new_keys);
        relocate_removed(&old_joists, &new_keys, &mut model.unassigned_joists);
        Box::new(RestoreFloorRegionSecondaryJoists {
            region: self.region,
            joists: old_joists,
            unassigned: old_unassigned,
        })
    }

    fn label(&self) -> &str {
        "床領域小梁リスト変更"
    }
}

/// 床領域内小梁の断面を変更する。
pub struct SetFloorRegionJoistSection {
    pub region: FloorRegionId,
    pub index: usize,
    pub section: Option<SectionId>,
}

impl EditCommand for SetFloorRegionJoistSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let ri = self.region.index();
        if ri >= model.floor_regions.len() || model.floor_regions[ri].id != self.region {
            return Box::new(Noop);
        }
        if self.index >= model.floor_regions[ri].secondary_joists.len() {
            return Box::new(Noop);
        }
        if !crate::refs::section_ref_ok(model, self.section) {
            return Box::new(Noop);
        }
        let old = model.floor_regions[ri].secondary_joists[self.index].section;
        model.floor_regions[ri].secondary_joists[self.index].section = self.section;
        Box::new(SetFloorRegionJoistSection {
            region: self.region,
            index: self.index,
            section: old,
        })
    }

    fn label(&self) -> &str {
        "床領域小梁断面変更"
    }
}

/// 壁領域の表示名変更。
pub struct SetWallRegionName {
    pub id: WallRegionId,
    pub name: String,
}

impl EditCommand for SetWallRegionName {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.wall_regions.len() || model.wall_regions[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut model.wall_regions[idx].name, self.name.clone());
        if old == self.name {
            return Box::new(Noop);
        }
        Box::new(SetWallRegionName {
            id: self.id,
            name: old,
        })
    }

    fn label(&self) -> &str {
        "壁領域名変更"
    }
}

/// 壁領域の間柱リスト（`posts`）を全置換する。
///
/// 新しいリストに無い旧所属は未割当へ移す。他領域との端点重複は Noop。
pub struct SetWallRegionPosts {
    pub region: WallRegionId,
    pub posts: Vec<SecondaryMember>,
}

struct RestoreWallRegionPosts {
    region: WallRegionId,
    posts: Vec<SecondaryMember>,
    unassigned: Vec<SecondaryMember>,
}

impl EditCommand for RestoreWallRegionPosts {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.region.index();
        if idx >= model.wall_regions.len() || model.wall_regions[idx].id != self.region {
            return Box::new(Noop);
        }
        let old_posts = std::mem::replace(&mut model.wall_regions[idx].posts, self.posts.clone());
        let old_unassigned =
            std::mem::replace(&mut model.unassigned_posts, self.unassigned.clone());
        Box::new(RestoreWallRegionPosts {
            region: self.region,
            posts: old_posts,
            unassigned: old_unassigned,
        })
    }

    fn label(&self) -> &str {
        "壁領域間柱リスト変更の取り消し"
    }
}

impl EditCommand for SetWallRegionPosts {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.region.index();
        if idx >= model.wall_regions.len() || model.wall_regions[idx].id != self.region {
            return Box::new(Noop);
        }
        if !posts_ok(&self.posts)
            || !self.posts.iter().all(|sm| secondary_member_ok(model, sm))
            || !unique_ids(&self.posts)
        {
            return Box::new(Noop);
        }
        if self
            .posts
            .iter()
            .any(|sm| post_id_in_other_regions(model, sm.id, self.region))
        {
            return Box::new(Noop);
        }
        let new_keys: HashSet<_> = self.posts.iter().map(|sm| sm.id).collect();
        let old_posts = std::mem::replace(&mut model.wall_regions[idx].posts, self.posts.clone());
        let old_unassigned = model.unassigned_posts.clone();
        take_from_unassigned(&mut model.unassigned_posts, &new_keys);
        relocate_removed(&old_posts, &new_keys, &mut model.unassigned_posts);
        Box::new(RestoreWallRegionPosts {
            region: self.region,
            posts: old_posts,
            unassigned: old_unassigned,
        })
    }

    fn label(&self) -> &str {
        "壁領域間柱リスト変更"
    }
}

/// 壁領域内間柱の断面を変更する。
pub struct SetWallRegionPostSection {
    pub region: WallRegionId,
    pub index: usize,
    pub section: Option<SectionId>,
}

impl EditCommand for SetWallRegionPostSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let ri = self.region.index();
        if ri >= model.wall_regions.len() || model.wall_regions[ri].id != self.region {
            return Box::new(Noop);
        }
        if self.index >= model.wall_regions[ri].posts.len() {
            return Box::new(Noop);
        }
        if !crate::refs::section_ref_ok(model, self.section) {
            return Box::new(Noop);
        }
        let old = model.wall_regions[ri].posts[self.index].section;
        model.wall_regions[ri].posts[self.index].section = self.section;
        Box::new(SetWallRegionPostSection {
            region: self.region,
            index: self.index,
            section: old,
        })
    }

    fn label(&self) -> &str {
        "壁領域間柱断面変更"
    }
}

/// 間柱の端部負担率を変更する。安定 ID で対象を探し、負担率は端の並びで指定する。
pub struct SetPostGravityEndShares {
    pub member: SecondaryMemberId,
    pub shares: Option<[f64; 2]>,
}

impl EditCommand for SetPostGravityEndShares {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let Some(post) = find_secondary_mut(model, self.member) else {
            return Box::new(Noop);
        };
        let old = std::mem::replace(&mut post.gravity_end_shares, self.shares);
        Box::new(Self {
            member: self.member,
            shares: old,
        })
    }

    fn label(&self) -> &str {
        "間柱端部負担率変更"
    }
}

/// 二次部材（小梁・間柱）の端部支持条件を変更する。
///
/// 安定 ID で対象を探し、床領域内・壁領域内・未割当のいずれにあっても設定する。
/// 支持端は現在の端部座標から支持部材アンカーへ再解決し、自由端は片持ちへ変換する
/// （片持ちへの読み替えは、利用者が自由端を指定したときだけ行う）。同じ条件の場合は
/// Noop。
pub struct SetSecondaryMemberEndSupport {
    pub member: SecondaryMemberId,
    pub end_support: [EndSupport; 2],
}

fn find_secondary_mut(model: &mut Model, id: SecondaryMemberId) -> Option<&mut SecondaryMember> {
    if let Some(sm) = model.unassigned_joists.iter_mut().find(|sm| sm.id == id) {
        return Some(sm);
    }
    if let Some(sm) = model.unassigned_posts.iter_mut().find(|sm| sm.id == id) {
        return Some(sm);
    }
    for region in &mut model.floor_regions {
        if let Some(sm) = region.secondary_joists.iter_mut().find(|sm| sm.id == id) {
            return Some(sm);
        }
    }
    for region in &mut model.wall_regions {
        if let Some(sm) = region.posts.iter_mut().find(|sm| sm.id == id) {
            return Some(sm);
        }
    }
    None
}

/// `ends` が表す端部支持条件（片持ちの自由端は端番号 1）。支持未解決の `Detached` は
/// 支持条件が確定していないため `None` とし、「同条件」として扱わない。
fn ends_supported(ends: &SecondaryMemberEnds) -> Option<[bool; 2]> {
    match ends {
        SecondaryMemberEnds::Cantilever { .. } => Some([true, false]),
        SecondaryMemberEnds::Supported(_) => Some([true, true]),
        SecondaryMemberEnds::Detached(_) => None,
    }
}

impl EditCommand for SetSecondaryMemberEndSupport {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let new_supported = [
            self.end_support[0] == EndSupport::Supported,
            self.end_support[1] == EndSupport::Supported,
        ];
        let Some(sm) = model.secondary_member(self.member) else {
            return Box::new(Noop);
        };
        let old_ends = sm.ends;
        let kind = sm.kind;
        if ends_supported(&old_ends) == Some(new_supported) {
            return Box::new(Noop);
        }
        let Some((a, b)) = model.secondary_member_end_points(sm) else {
            return Box::new(Noop);
        };
        let ends = model.secondary_ends_from_coords(self.member, kind, [a, b], new_supported);
        let candidate = SecondaryMember {
            id: self.member,
            gravity_end_shares: None,
            kind,
            ends,
            section: sm.section,
            name: sm.name.clone(),
        };
        if !secondary_ends_ok(model, &candidate) {
            return Box::new(Noop);
        }
        let snapshot = snapshot_secondary(model);
        let action = SecondaryAction::SetEnds(self.member, ends);
        apply_secondary_action(model, &action);
        model.rebuild_assignment_regions_dropping_orphan_plates();
        Box::new(RestoreSecondarySnapshot {
            snapshot,
            redo: action,
        })
    }

    fn label(&self) -> &str {
        "二次部材の端部支持条件変更"
    }
}

/// 二次部材を新規に配置する親（作業範囲）。
///
/// [`SecondaryParent::Unassigned`] はどの床領域・壁領域にも入れず未割当へ置く。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SecondaryParent {
    Floor(FloorRegionId),
    Wall(WallRegionId),
    Unassigned,
}

impl SecondaryParent {
    fn accepts(self, kind: SecondaryMemberKind) -> bool {
        match self {
            SecondaryParent::Floor(_) => kind == SecondaryMemberKind::Joist,
            SecondaryParent::Wall(_) => kind == SecondaryMemberKind::Post,
            SecondaryParent::Unassigned => true,
        }
    }
}

fn anchor_ok(model: &Model, anchor: &SecondaryMemberAnchor) -> bool {
    anchor.position.is_finite()
        && (0.0..=1.0).contains(&anchor.position)
        && model.support_member_axis(anchor.support).is_some()
}

/// 点 `p` [mm] が親領域（床領域・壁領域）の内部または境界上にあるか。
/// 親を持たない [`SecondaryParent::Unassigned`] は常に `true`。
fn parent_contains_point(model: &Model, parent: SecondaryParent, p: [f64; 3]) -> bool {
    match parent {
        SecondaryParent::Floor(id) => model.floor_region_contains_point_including_boundary(id, p),
        SecondaryParent::Wall(id) => model.wall_region_contains_point_including_boundary(id, p),
        SecondaryParent::Unassigned => true,
    }
}

/// 支持端が親領域（作業範囲）の内部または境界上にあるか。
///
/// 対象は [`SecondaryMemberEnds::Supported`] の 2 端と [`SecondaryMemberEnds::Cantilever`] の
/// 支持端で、支持端を持たない [`SecondaryMemberEnds::Detached`] は `false`。片持ちの
/// 自由端は親領域の内外を判定しない。
fn support_ends_in_parent(
    model: &Model,
    parent: SecondaryParent,
    ends: &SecondaryMemberEnds,
) -> bool {
    match ends {
        SecondaryMemberEnds::Supported([a, b]) => {
            let (Some(p0), Some(p1)) = (model.anchor_point(*a), model.anchor_point(*b)) else {
                return false;
            };
            parent_contains_point(model, parent, p0) && parent_contains_point(model, parent, p1)
        }
        SecondaryMemberEnds::Cantilever { support, .. } => model
            .anchor_point(*support)
            .is_some_and(|p| parent_contains_point(model, parent, p)),
        SecondaryMemberEnds::Detached(_) => false,
    }
}

/// 安定 ID で二次部材が属する親領域。どの領域にも属さなければ
/// [`SecondaryParent::Unassigned`]。
fn secondary_parent_of(model: &Model, id: SecondaryMemberId) -> SecondaryParent {
    if let Some(region) = model.floor_region_of_joist(id) {
        return SecondaryParent::Floor(region.id);
    }
    if let Some(region) = model
        .wall_regions
        .iter()
        .find(|r| r.posts.iter().any(|sm| sm.id == id))
    {
        return SecondaryParent::Wall(region.id);
    }
    SecondaryParent::Unassigned
}

/// 取付き位置表現が `Model::validate` を通るか。`Detached`（支持未解決）は配置を
/// 拒否する対象なので常に不適とする。
fn secondary_ends_ok(model: &Model, candidate: &SecondaryMember) -> bool {
    if matches!(candidate.ends, SecondaryMemberEnds::Detached(_)) {
        return false;
    }
    if !candidate.ends.anchors().iter().all(|a| anchor_ok(model, a)) {
        return false;
    }
    let mut all: Vec<&SecondaryMember> = model
        .joists()
        .chain(model.posts())
        .filter(|sm| sm.id != candidate.id)
        .collect();
    all.push(candidate);
    squid_n_core::model::validate_secondary_members(&all).is_ok()
}

/// 二次部材を 1 本追加する。端部は支持部材アンカーで与える。
///
/// 親領域（[`SecondaryParent`]）へ追加したうえで割当領域を再構築する。境界が
/// 変わって参照先を失った囲まれた版は取り除かれ、新領域は未設定になる。
/// 追加・再構築・版の除去は 1 つの Undo 単位で、取り消すと適用前へ戻る。
/// 支持が決まらない端（[`SecondaryMemberEnds::Detached`]）、種別に合わない親、
/// 支持端が親領域の内側・境界上にない場合は Noop とし、片持ちへの読み替えはしない。
pub struct PlaceSecondaryMember {
    pub parent: SecondaryParent,
    pub kind: SecondaryMemberKind,
    pub ends: SecondaryMemberEnds,
    pub section: Option<SectionId>,
    pub name: String,
}

impl EditCommand for PlaceSecondaryMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if !self.parent.accepts(self.kind) || !crate::refs::section_ref_ok(model, self.section) {
            return Box::new(Noop);
        }
        let candidate = SecondaryMember {
            id: SecondaryMemberId(u32::MAX),
            gravity_end_shares: None,
            kind: self.kind,
            ends: self.ends,
            section: self.section,
            name: self.name.clone(),
        };
        if !secondary_ends_ok(model, &candidate)
            || !support_ends_in_parent(model, self.parent, &self.ends)
        {
            return Box::new(Noop);
        }
        let snapshot = snapshot_secondary(model);
        let id = model.alloc_secondary_member_id();
        let action = SecondaryAction::Place {
            parent: self.parent,
            id,
            kind: self.kind,
            ends: self.ends,
            section: self.section,
            name: self.name.clone(),
        };
        if !apply_secondary_action(model, &action) {
            return Box::new(Noop);
        }
        model.rebuild_assignment_regions_dropping_orphan_plates();
        Box::new(RestoreSecondarySnapshot {
            snapshot,
            redo: action,
        })
    }

    fn label(&self) -> &str {
        "二次部材配置"
    }
}

/// 安定 ID で二次部材を 1 本削除する。割当領域を再構築し、参照先を失った版を
/// 取り除くまでを 1 つの Undo 単位に含める。
pub struct DeleteSecondaryMember {
    pub member: SecondaryMemberId,
}

impl EditCommand for DeleteSecondaryMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if model.secondary_member(self.member).is_none() {
            return Box::new(Noop);
        }
        let snapshot = snapshot_secondary(model);
        let action = SecondaryAction::Delete(self.member);
        if !apply_secondary_action(model, &action) {
            return Box::new(Noop);
        }
        model.rebuild_assignment_regions_dropping_orphan_plates();
        Box::new(RestoreSecondarySnapshot {
            snapshot,
            redo: action,
        })
    }

    fn label(&self) -> &str {
        "二次部材削除"
    }
}

/// 安定 ID で二次部材の両端（取付き位置）を置き換える。
///
/// 端の移動は `ends` の差し替えで表す。取付き位置表現が不正、支持が決まらない端
/// （[`SecondaryMemberEnds::Detached`]）、または移動先の支持端が現在の所属領域の内側・
/// 境界上にない場合は Noop。配置と同様に割当領域を再構築する。
pub struct SetSecondaryMemberEnds {
    pub member: SecondaryMemberId,
    pub ends: SecondaryMemberEnds,
}

impl EditCommand for SetSecondaryMemberEnds {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let Some(sm) = model.secondary_member(self.member) else {
            return Box::new(Noop);
        };
        if sm.ends == self.ends {
            return Box::new(Noop);
        }
        let candidate = SecondaryMember {
            id: self.member,
            gravity_end_shares: None,
            kind: sm.kind,
            ends: self.ends,
            section: sm.section,
            name: sm.name.clone(),
        };
        let parent = secondary_parent_of(model, self.member);
        if !secondary_ends_ok(model, &candidate)
            || !support_ends_in_parent(model, parent, &self.ends)
        {
            return Box::new(Noop);
        }
        let snapshot = snapshot_secondary(model);
        let action = SecondaryAction::SetEnds(self.member, self.ends);
        if !apply_secondary_action(model, &action) {
            return Box::new(Noop);
        }
        model.rebuild_assignment_regions_dropping_orphan_plates();
        Box::new(RestoreSecondarySnapshot {
            snapshot,
            redo: action,
        })
    }

    fn label(&self) -> &str {
        "二次部材の端部移動"
    }
}

/// 二次部材の配置・削除・端部移動のうち、再適用（redo）で繰り返す操作。
#[derive(Clone)]
enum SecondaryAction {
    Place {
        parent: SecondaryParent,
        id: SecondaryMemberId,
        kind: SecondaryMemberKind,
        ends: SecondaryMemberEnds,
        section: Option<SectionId>,
        name: String,
    },
    Delete(SecondaryMemberId),
    SetEnds(SecondaryMemberId, SecondaryMemberEnds),
}

/// 割当領域の再構築と孤児版の除去で変化するモデル部分（undo 用）。
#[derive(Clone)]
struct SecondarySnapshot {
    floor_regions: Vec<squid_n_core::model::FloorRegion>,
    wall_regions: Vec<squid_n_core::model::WallRegion>,
    unassigned_joists: Vec<SecondaryMember>,
    unassigned_posts: Vec<SecondaryMember>,
    floor_assignment_regions: squid_n_core::model::FloorPlateAssignmentRegions,
    wall_assignment_regions: squid_n_core::model::WallPlateAssignmentRegions,
    slabs: Vec<squid_n_core::model::Slab>,
    wall_plates: Vec<squid_n_core::model::WallPlate>,
    next_secondary_member_id: u32,
}

fn snapshot_secondary(model: &Model) -> SecondarySnapshot {
    SecondarySnapshot {
        floor_regions: model.floor_regions.clone(),
        wall_regions: model.wall_regions.clone(),
        unassigned_joists: model.unassigned_joists.clone(),
        unassigned_posts: model.unassigned_posts.clone(),
        floor_assignment_regions: model.floor_assignment_regions.clone(),
        wall_assignment_regions: model.wall_assignment_regions.clone(),
        slabs: model.slabs.clone(),
        wall_plates: model.wall_plates.clone(),
        next_secondary_member_id: model.next_secondary_member_id,
    }
}

fn restore_secondary(model: &mut Model, snapshot: SecondarySnapshot) {
    model.floor_regions = snapshot.floor_regions;
    model.wall_regions = snapshot.wall_regions;
    model.unassigned_joists = snapshot.unassigned_joists;
    model.unassigned_posts = snapshot.unassigned_posts;
    model.floor_assignment_regions = snapshot.floor_assignment_regions;
    model.wall_assignment_regions = snapshot.wall_assignment_regions;
    model.slabs = snapshot.slabs;
    model.wall_plates = snapshot.wall_plates;
    model.next_secondary_member_id = snapshot.next_secondary_member_id;
}

fn remove_secondary(model: &mut Model, id: SecondaryMemberId) -> bool {
    if let Some(pos) = model.unassigned_joists.iter().position(|sm| sm.id == id) {
        model.unassigned_joists.remove(pos);
        return true;
    }
    if let Some(pos) = model.unassigned_posts.iter().position(|sm| sm.id == id) {
        model.unassigned_posts.remove(pos);
        return true;
    }
    for region in &mut model.floor_regions {
        if let Some(pos) = region.secondary_joists.iter().position(|sm| sm.id == id) {
            region.secondary_joists.remove(pos);
            return true;
        }
    }
    for region in &mut model.wall_regions {
        if let Some(pos) = region.posts.iter().position(|sm| sm.id == id) {
            region.posts.remove(pos);
            return true;
        }
    }
    false
}

fn apply_secondary_action(model: &mut Model, action: &SecondaryAction) -> bool {
    match action {
        SecondaryAction::Place {
            parent,
            id,
            kind,
            ends,
            section,
            name,
        } => {
            if model.secondary_member(*id).is_some() {
                return false;
            }
            let member = SecondaryMember {
                id: *id,
                gravity_end_shares: None,
                kind: *kind,
                ends: *ends,
                section: *section,
                name: name.clone(),
            };
            match parent {
                SecondaryParent::Floor(region) => {
                    let idx = region.index();
                    match model.floor_regions.get_mut(idx) {
                        Some(r) if r.id == *region => {
                            r.secondary_joists.push(member);
                            true
                        }
                        _ => false,
                    }
                }
                SecondaryParent::Wall(region) => {
                    let idx = region.index();
                    match model.wall_regions.get_mut(idx) {
                        Some(r) if r.id == *region => {
                            r.posts.push(member);
                            true
                        }
                        _ => false,
                    }
                }
                SecondaryParent::Unassigned => {
                    if member.kind == SecondaryMemberKind::Joist {
                        model.unassigned_joists.push(member);
                    } else {
                        model.unassigned_posts.push(member);
                    }
                    true
                }
            }
        }
        SecondaryAction::Delete(id) => remove_secondary(model, *id),
        SecondaryAction::SetEnds(id, ends) => match find_secondary_mut(model, *id) {
            Some(sm) => {
                sm.ends = *ends;
                true
            }
            None => false,
        },
    }
}

/// [`SecondarySnapshot`] を復元し、redo で操作を再適用できるようにする逆操作。
struct RestoreSecondarySnapshot {
    snapshot: SecondarySnapshot,
    redo: SecondaryAction,
}

impl EditCommand for RestoreSecondarySnapshot {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        restore_secondary(model, self.snapshot.clone());
        Box::new(ApplySecondaryAction {
            action: self.redo.clone(),
        })
    }

    fn label(&self) -> &str {
        "二次部材編集の取り消し"
    }
}

/// [`RestoreSecondarySnapshot`] の逆操作。操作を再適用して割当領域を再構築する。
struct ApplySecondaryAction {
    action: SecondaryAction,
}

impl EditCommand for ApplySecondaryAction {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let snapshot = snapshot_secondary(model);
        if !apply_secondary_action(model, &self.action) {
            return Box::new(Noop);
        }
        model.rebuild_assignment_regions_dropping_orphan_plates();
        Box::new(RestoreSecondarySnapshot {
            snapshot,
            redo: self.action.clone(),
        })
    }

    fn label(&self) -> &str {
        "二次部材編集の再適用"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::{ElemId, FloorRegionId, NodeId, WallRegionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, FloorRegion, ForceRegime, LocalAxis, Node,
        SlabPlate, SupportMemberId, WallRegion,
    };

    /// 4 辺を柱・梁で囲んだ 1 壁領域のモデル（XY ではなく XZ 面内）。
    fn wall_model() -> Model {
        let mut model = Model::default();
        for (i, (x, z)) in [(0.0, 0.0), (4000.0, 0.0), (4000.0, 3000.0), (0.0, 3000.0)]
            .into_iter()
            .enumerate()
        {
            model.nodes.push(node(i as u32, [x, 0.0, z]));
        }
        for (i, (a, b)) in [(0u32, 3u32), (1, 2), (3, 2), (0, 1)]
            .into_iter()
            .enumerate()
        {
            model.elements.push(beam(i as u32, a, b));
        }
        model.wall_regions.push(WallRegion::new(
            WallRegionId(0),
            vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        ));
        model
    }

    fn node(id: u32, coord: [f64; 3]) -> Node {
        Node {
            id: NodeId(id),
            coord,
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    fn beam(id: u32, a: u32, b: u32) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
            section: None,
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

    fn anchor(elem: u32, position: f64) -> SecondaryMemberAnchor {
        SecondaryMemberAnchor {
            support: SupportMemberId::Primary(ElemId(elem)),
            position,
        }
    }

    /// 4 辺を大梁で囲んだ 1 床領域のモデル。割当領域は 1 面。
    fn square_model() -> Model {
        let mut model = Model::default();
        for (i, (x, y)) in [(0.0, 0.0), (4000.0, 0.0), (4000.0, 4000.0), (0.0, 4000.0)]
            .into_iter()
            .enumerate()
        {
            model.nodes.push(node(i as u32, [x, y, 0.0]));
        }
        for (i, (a, b)) in [(0u32, 1u32), (1, 2), (2, 3), (3, 0)]
            .into_iter()
            .enumerate()
        {
            model.elements.push(beam(i as u32, a, b));
        }
        model.floor_regions.push(FloorRegion::new(
            FloorRegionId(0),
            vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        ));
        model.rebuild_floor_assignment_regions();
        model
    }

    fn place_joist(model: &mut Model, undo: &mut crate::UndoStack) -> bool {
        undo.run(
            model,
            Box::new(PlaceSecondaryMember {
                parent: SecondaryParent::Floor(FloorRegionId(0)),
                kind: SecondaryMemberKind::Joist,
                ends: SecondaryMemberEnds::Supported([anchor(0, 0.5), anchor(2, 0.5)]),
                section: None,
                name: "J0".into(),
            }),
        )
    }

    #[test]
    fn 床領域へ小梁を配置すると割当領域が分割される() {
        let mut model = square_model();
        let mut undo = crate::UndoStack::new();
        assert_eq!(model.floor_assignment_regions.regions.len(), 1);

        assert!(place_joist(&mut model, &mut undo));
        assert_eq!(model.joists().count(), 1);
        assert_eq!(model.joists().next().unwrap().id, SecondaryMemberId(0));
        assert_eq!(
            model.floor_assignment_regions.regions.len(),
            2,
            "小梁で 2 面"
        );
        assert!(model
            .floor_assignment_regions
            .regions
            .iter()
            .all(|r| r.assignment.is_unset()));
        assert!(model.validate().is_ok(), "{:?}", model.validate());

        undo.undo(&mut model);
        assert_eq!(model.joists().count(), 0);
        assert_eq!(model.floor_assignment_regions.regions.len(), 1);
        assert!(model.validate().is_ok());

        undo.redo(&mut model);
        assert_eq!(model.joists().count(), 1);
        assert_eq!(
            model.joists().next().unwrap().id,
            SecondaryMemberId(0),
            "redo でも同じ安定 ID"
        );
        assert!(model.validate().is_ok());
    }

    #[test]
    fn 支持が決まらない配置は拒否する() {
        let mut model = square_model();
        let mut undo = crate::UndoStack::new();
        let applied = undo.run(
            &mut model,
            Box::new(PlaceSecondaryMember {
                parent: SecondaryParent::Floor(FloorRegionId(0)),
                kind: SecondaryMemberKind::Joist,
                ends: SecondaryMemberEnds::Detached([[0.0, 0.0, 0.0], [4000.0, 4000.0, 0.0]]),
                section: None,
                name: String::new(),
            }),
        );
        assert!(!applied, "Detached への読み替えはしない");
        assert_eq!(model.joists().count(), 0);
    }

    #[test]
    fn 存在しない支持部材を指す配置は拒否する() {
        let mut model = square_model();
        let mut undo = crate::UndoStack::new();
        let applied = undo.run(
            &mut model,
            Box::new(PlaceSecondaryMember {
                parent: SecondaryParent::Floor(FloorRegionId(0)),
                kind: SecondaryMemberKind::Joist,
                ends: SecondaryMemberEnds::Supported([anchor(99, 0.5), anchor(2, 0.5)]),
                section: None,
                name: String::new(),
            }),
        );
        assert!(!applied);
        assert_eq!(model.joists().count(), 0);
    }

    #[test]
    fn 版あり領域の分割で版が除去され新領域は未設定() {
        let mut model = square_model();
        let slab = model
            .assign_enclosed_slab_to_matching_region(
                &[NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
                SlabPlate::default(),
            )
            .expect("1 面へ割当");
        assert_eq!(model.slabs.len(), 1);

        let mut undo = crate::UndoStack::new();
        assert!(place_joist(&mut model, &mut undo));
        assert_eq!(model.slabs.len(), 0, "参照先を失った囲まれた床板は取り除く");
        assert!(model
            .floor_assignment_regions
            .regions
            .iter()
            .all(|r| r.assignment.is_unset()));
        assert!(model.unset_plate_assignment_regions().0.len() == 2);
        assert!(model.validate().is_ok(), "{:?}", model.validate());

        undo.undo(&mut model);
        assert_eq!(model.slabs.len(), 1, "undo で版が戻る");
        assert!(model.slab_assignment_region(slab).is_some());
        assert!(model.joists().count() == 0);
        assert!(model.validate().is_ok());
    }

    #[test]
    fn 二次部材の削除で領域が統合される() {
        let mut model = square_model();
        let mut undo = crate::UndoStack::new();
        assert!(place_joist(&mut model, &mut undo));
        let id = model.joists().next().unwrap().id;

        assert!(undo.run(&mut model, Box::new(DeleteSecondaryMember { member: id })));
        assert_eq!(model.joists().count(), 0);
        assert_eq!(model.floor_assignment_regions.regions.len(), 1);
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    #[test]
    fn 二次部材の端部を移動すると割当領域が再構築される() {
        let mut model = square_model();
        let mut undo = crate::UndoStack::new();
        assert!(place_joist(&mut model, &mut undo));
        let id = model.joists().next().unwrap().id;

        let ends = SecondaryMemberEnds::Supported([anchor(0, 0.25), anchor(2, 0.5)]);
        assert!(undo.run(
            &mut model,
            Box::new(SetSecondaryMemberEnds { member: id, ends })
        ));
        assert_eq!(model.joists().next().unwrap().ends, ends);
        assert_eq!(model.floor_assignment_regions.regions.len(), 2);
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    /// 床領域の外側に、アンカー先になるだけの大梁を 1 本足す。
    fn add_outside_floor_beam(model: &mut Model) {
        let base = model.nodes.len() as u32;
        model.nodes.push(node(base, [-4000.0, 0.0, 0.0]));
        model.nodes.push(node(base + 1, [-4000.0, 4000.0, 0.0]));
        model.elements.push(beam(base, base, base + 1));
    }

    #[test]
    fn 親領域外の支持部材へアンカーした配置は拒否する() {
        let mut model = square_model();
        add_outside_floor_beam(&mut model);
        let mut undo = crate::UndoStack::new();
        let applied = undo.run(
            &mut model,
            Box::new(PlaceSecondaryMember {
                parent: SecondaryParent::Floor(FloorRegionId(0)),
                kind: SecondaryMemberKind::Joist,
                ends: SecondaryMemberEnds::Supported([anchor(4, 0.5), anchor(0, 0.5)]),
                section: None,
                name: "J-out".into(),
            }),
        );
        assert!(!applied, "支持端が親領域の外にある配置は拒否する");
        assert_eq!(model.joists().count(), 0);
    }

    #[test]
    fn 親領域内の支持部材へアンカーした配置は成功する() {
        let mut model = square_model();
        let mut undo = crate::UndoStack::new();
        assert!(place_joist(&mut model, &mut undo));
        let j0 = model.joists().next().unwrap().id;

        let applied = undo.run(
            &mut model,
            Box::new(PlaceSecondaryMember {
                parent: SecondaryParent::Floor(FloorRegionId(0)),
                kind: SecondaryMemberKind::Joist,
                ends: SecondaryMemberEnds::Supported([
                    SecondaryMemberAnchor {
                        support: SupportMemberId::Secondary(j0),
                        position: 0.5,
                    },
                    anchor(1, 0.5),
                ]),
                section: None,
                name: "J1".into(),
            }),
        );
        assert!(applied, "支持端が親領域の内側にある配置は成功する");
        assert_eq!(model.joists().count(), 2);
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    #[test]
    fn 親領域外へ支持端を動かす端部移動は拒否する() {
        let mut model = square_model();
        let mut undo = crate::UndoStack::new();
        assert!(place_joist(&mut model, &mut undo));
        let id = model.joists().next().unwrap().id;
        let before = model.joists().next().unwrap().ends;

        add_outside_floor_beam(&mut model);
        let moved = SecondaryMemberEnds::Supported([anchor(4, 0.5), anchor(2, 0.5)]);
        let applied = undo.run(
            &mut model,
            Box::new(SetSecondaryMemberEnds {
                member: id,
                ends: moved,
            }),
        );
        assert!(!applied, "所属床領域の外へ支持端を動かす端部移動は拒否する");
        assert_eq!(model.joists().next().unwrap().ends, before);
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    #[test]
    fn 壁領域外の支持部材へアンカーした間柱配置は拒否する() {
        let mut model = wall_model();
        let base = model.nodes.len() as u32;
        model.nodes.push(node(base, [-4000.0, 0.0, 0.0]));
        model.nodes.push(node(base + 1, [-4000.0, 0.0, 3000.0]));
        model.elements.push(beam(base, base, base + 1));
        let mut undo = crate::UndoStack::new();
        let applied = undo.run(
            &mut model,
            Box::new(PlaceSecondaryMember {
                parent: SecondaryParent::Wall(WallRegionId(0)),
                kind: SecondaryMemberKind::Post,
                ends: SecondaryMemberEnds::Supported([anchor(base, 0.5), anchor(3, 0.5)]),
                section: None,
                name: "P-out".into(),
            }),
        );
        assert!(!applied, "支持端が壁領域の外にある配置は拒否する");
        assert_eq!(model.posts().count(), 0);
    }

    #[test]
    fn 壁領域内の支持部材へアンカーした間柱配置は成功する() {
        let mut model = wall_model();
        let mut undo = crate::UndoStack::new();
        let applied = undo.run(
            &mut model,
            Box::new(PlaceSecondaryMember {
                parent: SecondaryParent::Wall(WallRegionId(0)),
                kind: SecondaryMemberKind::Post,
                ends: SecondaryMemberEnds::Supported([anchor(3, 0.5), anchor(2, 0.5)]),
                section: None,
                name: "P-in".into(),
            }),
        );
        assert!(applied, "支持端が壁領域の内側・境界上にある配置は成功する");
        assert_eq!(model.posts().count(), 1);
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    #[test]
    fn 片持ち間柱は自由端が解決できなくても支持端が親領域にあれば配置できる() {
        let mut model = wall_model();
        // 構面を一意に決められないよう、鉛直材を 1 本だけ残す。
        model.elements.retain(|e| e.id != ElemId(1));
        let mut undo = crate::UndoStack::new();
        let applied = undo.run(
            &mut model,
            Box::new(PlaceSecondaryMember {
                parent: SecondaryParent::Wall(WallRegionId(0)),
                kind: SecondaryMemberKind::Post,
                ends: SecondaryMemberEnds::Cantilever {
                    support: SecondaryMemberAnchor {
                        support: SupportMemberId::Primary(ElemId(0)),
                        position: 0.5,
                    },
                    free_end_vector: [1000.0, 0.0],
                },
                section: None,
                name: "Pc".into(),
            }),
        );
        assert!(applied, "支持端が壁領域の境界上なら配置できる");
        assert_eq!(model.posts().count(), 1);
        let post = model.posts().next().expect("配置した間柱");
        assert!(
            model.secondary_member_end_points(post).is_none(),
            "構面が一意に決まらず自由端は解決できない"
        );
    }

    /// 大梁（主架構要素）の削除で割当領域の境界が変わり、そこだけに載っていた
    /// 囲まれた床板が孤児化しても、削除から戻せるまで（`DeleteMember` 直後に
    /// `Model::validate` が通る）を固定する。孤児版を残すと validate が必ず落ちる。
    #[test]
    fn 大梁削除で孤児化した床板は取り除かれvalidateが通る() {
        let mut model = square_model();
        let slab = model
            .assign_enclosed_slab_to_matching_region(
                &[NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
                SlabPlate::default(),
            )
            .expect("1 面へ割当");
        assert_eq!(model.slabs.len(), 1);
        assert!(model.validate().is_ok(), "{:?}", model.validate());

        let mut undo = crate::UndoStack::new();
        assert!(undo.run(&mut model, Box::new(crate::DeleteMember { id: ElemId(0) })));
        assert_eq!(model.slabs.len(), 0, "参照先を失った囲まれた床板は取り除く");
        assert!(model.floor_assignment_regions.regions.is_empty());
        assert!(model.validate().is_ok(), "{:?}", model.validate());

        undo.undo(&mut model);
        assert_eq!(model.slabs.len(), 1, "undo で版が戻る");
        assert!(model.slab_assignment_region(slab).is_some());
        assert_eq!(model.elements.len(), 4, "undo で大梁が戻る");
        assert!(model.validate().is_ok(), "{:?}", model.validate());

        undo.redo(&mut model);
        assert_eq!(model.slabs.len(), 0);
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }
}
