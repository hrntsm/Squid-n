//! 二次部材（小梁・間柱）と壁領域の編集コマンド（D6: 領域内実体）。

use super::*;
use squid_n_core::ids::*;
use squid_n_core::model::{SecondaryMember, SecondaryMemberKind};
use std::collections::HashSet;

// ─── バリデーション ─────────────────────────────────────

fn secondary_member_ok(model: &Model, sm: &SecondaryMember) -> bool {
    sm.nodes.iter().all(|&n| crate::refs::node_exists(model, n))
        && crate::refs::section_ref_ok(model, sm.section)
}

fn joists_ok(joists: &[SecondaryMember]) -> bool {
    joists
        .iter()
        .all(|sm| sm.kind == SecondaryMemberKind::Joist)
}

fn posts_ok(posts: &[SecondaryMember]) -> bool {
    posts.iter().all(|sm| sm.kind == SecondaryMemberKind::Post)
}

fn endpoint_key(sm: &SecondaryMember) -> (SecondaryMemberKind, u32, u32) {
    let a = sm.nodes[0].0.min(sm.nodes[1].0);
    let b = sm.nodes[0].0.max(sm.nodes[1].0);
    (sm.kind, a, b)
}

fn unique_endpoints(sms: &[SecondaryMember]) -> bool {
    let mut seen = HashSet::new();
    sms.iter().all(|sm| seen.insert(endpoint_key(sm)))
}

fn joist_key_in_other_regions(
    model: &Model,
    key: (SecondaryMemberKind, u32, u32),
    skip: FloorRegionId,
) -> bool {
    model
        .floor_regions
        .iter()
        .any(|r| r.id != skip && r.secondary_joists.iter().any(|sm| endpoint_key(sm) == key))
}

fn post_key_in_other_regions(
    model: &Model,
    key: (SecondaryMemberKind, u32, u32),
    skip: WallRegionId,
) -> bool {
    model
        .wall_regions
        .iter()
        .any(|r| r.id != skip && r.posts.iter().any(|sm| endpoint_key(sm) == key))
}

fn relocate_removed(
    old: &[SecondaryMember],
    new_keys: &HashSet<(SecondaryMemberKind, u32, u32)>,
    unassigned: &mut Vec<SecondaryMember>,
) {
    for sm in old {
        let key = endpoint_key(sm);
        if new_keys.contains(&key) {
            continue;
        }
        if unassigned.iter().any(|u| endpoint_key(u) == key) {
            continue;
        }
        unassigned.push(sm.clone());
    }
}

fn take_from_unassigned(
    unassigned: &mut Vec<SecondaryMember>,
    new_keys: &HashSet<(SecondaryMemberKind, u32, u32)>,
) {
    unassigned.retain(|sm| !new_keys.contains(&endpoint_key(sm)));
}

// ─── 未割当小梁 ─────────────────────────────────────────

/// 未割当小梁を末尾へ追加する。逆操作は [`DeleteUnassignedJoist`]。
pub struct AddUnassignedJoist {
    pub sm: SecondaryMember,
}

impl EditCommand for AddUnassignedJoist {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.sm.kind != SecondaryMemberKind::Joist || !secondary_member_ok(model, &self.sm) {
            return Box::new(Noop);
        }
        let key = endpoint_key(&self.sm);
        if model.joists().any(|sm| endpoint_key(sm) == key) {
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

// ─── 未割当間柱 ─────────────────────────────────────────

/// 未割当間柱を末尾へ追加する。
pub struct AddUnassignedPost {
    pub sm: SecondaryMember,
}

impl EditCommand for AddUnassignedPost {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.sm.kind != SecondaryMemberKind::Post || !secondary_member_ok(model, &self.sm) {
            return Box::new(Noop);
        }
        let key = endpoint_key(&self.sm);
        if model.posts().any(|sm| endpoint_key(sm) == key) {
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

// ─── 床領域の小梁 ───────────────────────────────────────

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
            || !unique_endpoints(&self.joists)
        {
            return Box::new(Noop);
        }
        if self
            .joists
            .iter()
            .any(|sm| joist_key_in_other_regions(model, endpoint_key(sm), self.region))
        {
            return Box::new(Noop);
        }
        let new_keys: HashSet<_> = self.joists.iter().map(endpoint_key).collect();
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

// ─── 壁領域 ────────────────────────────────────────────

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
            || !unique_endpoints(&self.posts)
        {
            return Box::new(Noop);
        }
        if self
            .posts
            .iter()
            .any(|sm| post_key_in_other_regions(model, endpoint_key(sm), self.region))
        {
            return Box::new(Noop);
        }
        let new_keys: HashSet<_> = self.posts.iter().map(endpoint_key).collect();
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
