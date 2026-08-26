//! 二次部材（小梁・間柱）と壁領域の編集コマンド。

use super::*;
use squid_n_core::ids::*;
use squid_n_core::model::{SecondaryMember, SecondaryMemberKind};

// ─── ヘルパー関数 ─────────────────────────────────────

/// モデル内の全 `SecondaryMemberId` 参照に `f` を適用する。
/// `DeleteSecondaryMember` / `InsertSecondaryMember` の ID 繰り上げ・繰り下げで共用する。
/// 走査本体は core 側（`Model::visit_secondary_member_ids`）が単一情報源。
fn shift_secondary_member_ids(model: &mut Model, f: impl FnMut(&mut SecondaryMemberId)) {
    model.visit_secondary_member_ids(f);
}

// ─── グループA: 二次部材の CRUD ──────────────────────────

/// 二次部材追加。末尾に `SecondaryMemberId(len)` で追加する。
///
/// バリデーション:
/// - `sm.id` が末尾添字と一致すること
/// - 参照する節点が実在すること
/// - `section` を指定する場合は実在すること
///
/// 逆操作は末尾の二次部材削除（`DeleteSecondaryMember`）。
pub struct AddSecondaryMember {
    pub sm: SecondaryMember,
}

impl EditCommand for AddSecondaryMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let expected_id = SecondaryMemberId(model.secondary_members.len() as u32);
        if self.sm.id != expected_id {
            return Box::new(Noop);
        }
        // 参照節点の存在チェック
        if !self
            .sm
            .nodes
            .iter()
            .all(|&n| crate::refs::node_exists(model, n))
        {
            return Box::new(Noop);
        }
        // 断面の存在チェック
        if !crate::refs::section_ref_ok(model, self.sm.section) {
            return Box::new(Noop);
        }
        model.secondary_members.push(self.sm.clone());
        Box::new(DeleteSecondaryMember { id: self.sm.id })
    }

    fn label(&self) -> &str {
        "二次部材追加"
    }
}

/// 二次部材削除（中間の部材も可）。逆操作は [`InsertSecondaryMember`]。
///
/// 削除後は後続 ID を繰り上げ、`Slab.secondary_joist_ids` と
/// `WallRegion.post_ids` からも該当 ID を除去する（カスケード削除）。
pub struct DeleteSecondaryMember {
    pub id: SecondaryMemberId,
}

impl EditCommand for DeleteSecondaryMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.secondary_members.len() || model.secondary_members[idx].id != self.id {
            return Box::new(Noop);
        }

        // カスケード: スラブの secondary_joist_ids から除去し、位置を退避
        // (スラブ添字, リスト内位置) を昇順で記録する（InsertSecondaryMember での復元用）。
        let mut slab_refs: Vec<(usize, usize)> = Vec::new();
        for (si, slab) in model.floor_regions.iter_mut().enumerate() {
            let mut pos = 0;
            while pos < slab.secondary_joist_ids.len() {
                if slab.secondary_joist_ids[pos] == self.id {
                    slab.secondary_joist_ids.remove(pos);
                    slab_refs.push((si, pos));
                } else {
                    pos += 1;
                }
            }
        }

        // カスケード: 壁領域の post_ids から除去し、位置を退避
        let mut region_refs: Vec<(usize, usize)> = Vec::new();
        for (ri, region) in model.wall_regions.iter_mut().enumerate() {
            let mut pos = 0;
            while pos < region.post_ids.len() {
                if region.post_ids[pos] == self.id {
                    region.post_ids.remove(pos);
                    region_refs.push((ri, pos));
                } else {
                    pos += 1;
                }
            }
        }

        let removed = model.secondary_members.remove(idx);
        let target = self.id.0;
        // 後続 ID の繰り上げ
        shift_secondary_member_ids(model, |id| {
            if id.0 > target {
                id.0 -= 1;
            }
        });

        Box::new(InsertSecondaryMember {
            index: idx,
            sm: removed,
            slab_refs,
            region_refs,
        })
    }

    fn label(&self) -> &str {
        "二次部材削除"
    }
}

/// 指定インデックスへ二次部材を再挿入し、後続 ID と参照を繰り下げ、
/// スラブ・壁領域への参照も元の位置へ復元する
/// （[`DeleteSecondaryMember`] の逆操作専用）。
pub struct InsertSecondaryMember {
    pub index: usize,
    pub sm: SecondaryMember,
    /// 削除時にスラブの `secondary_joist_ids` から除去した参照の
    /// (スラブ添字, リスト内位置)。昇順で記録されているため逆順で挿入して元の並びを復元する。
    pub slab_refs: Vec<(usize, usize)>,
    /// 削除時に壁領域の `post_ids` から除去した参照の
    /// (壁領域添字, リスト内位置)。昇順で記録されているため逆順で挿入して元の並びを復元する。
    pub region_refs: Vec<(usize, usize)>,
}

impl EditCommand for InsertSecondaryMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.secondary_members.len() {
            return Box::new(Noop);
        }
        let id = SecondaryMemberId(self.index as u32);
        // 後続 ID の繰り下げ（挿入位置以降を +1）
        shift_secondary_member_ids(model, |x| {
            if x.0 >= id.0 {
                x.0 += 1;
            }
        });
        let mut sm = self.sm.clone();
        sm.id = id;
        model.secondary_members.insert(self.index, sm);

        // スラブの secondary_joist_ids を元の位置へ復元（逆順挿入で昇順復元）
        for &(si, pos) in self.slab_refs.iter().rev() {
            if let Some(slab) = model.floor_regions.get_mut(si) {
                let insert_pos = pos.min(slab.secondary_joist_ids.len());
                slab.secondary_joist_ids.insert(insert_pos, id);
            }
        }

        // 壁領域の post_ids を元の位置へ復元
        for &(ri, pos) in self.region_refs.iter().rev() {
            if let Some(region) = model.wall_regions.get_mut(ri) {
                let insert_pos = pos.min(region.post_ids.len());
                region.post_ids.insert(insert_pos, id);
            }
        }

        Box::new(DeleteSecondaryMember { id })
    }

    fn label(&self) -> &str {
        "二次部材削除の取り消し"
    }
}

/// 二次部材の断面（`section`）変更。逆操作は変更前の断面への復元。
/// 存在しない `SecondaryMemberId`、および実在しない断面を指す割当は Noop。
pub struct SetSecondaryMemberSection {
    pub id: SecondaryMemberId,
    pub section: Option<SectionId>,
}

impl EditCommand for SetSecondaryMemberSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.secondary_members.len() || model.secondary_members[idx].id != self.id {
            return Box::new(Noop);
        }
        if !crate::refs::section_ref_ok(model, self.section) {
            return Box::new(Noop);
        }
        let old = model.secondary_members[idx].section;
        model.secondary_members[idx].section = self.section;
        Box::new(SetSecondaryMemberSection {
            id: self.id,
            section: old,
        })
    }

    fn label(&self) -> &str {
        "二次部材断面変更"
    }
}

// ─── グループB: 床の小梁 ID リスト操作 ──────────────────────

/// 指定スラブの `secondary_joist_ids` を全置換する。
///
/// バリデーション:
/// - スラブが実在すること
/// - 全 ID が `secondary_members` に実在し、かつ `kind == Joist` であること
/// - ID に重複がないこと
///
/// 逆操作は元のリストへ戻す `SetSlabSecondaryJoistIds`。
pub struct SetSlabSecondaryJoistIds {
    pub slab: FloorRegionId,
    pub ids: Vec<SecondaryMemberId>,
}

impl EditCommand for SetSlabSecondaryJoistIds {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.slab.index();
        if idx >= model.floor_regions.len() || model.floor_regions[idx].id != self.slab {
            return Box::new(Noop);
        }
        // 全 ID が実在し Joist 種別であること、重複がないことを確認する。
        if !secondary_joist_ids_ok(model, &self.ids) {
            return Box::new(Noop);
        }
        let old = std::mem::replace(
            &mut model.floor_regions[idx].secondary_joist_ids,
            self.ids.clone(),
        );
        Box::new(SetSlabSecondaryJoistIds {
            slab: self.slab,
            ids: old,
        })
    }

    fn label(&self) -> &str {
        "スラブ小梁IDリスト変更"
    }
}

// ─── グループC: 壁領域 ────────────────────────────────────
//
// 壁領域（`WallRegion`）は床領域（`FloorRegion`）と同じく主架構から再検出・
// 再構成される派生的な入力の持ち主であり（D10）、利用者が任意の境界を与えて
// 追加・削除する対象ではない（`region_gen::wall`/`wall_region_rebuild` が組み立てる）。
// そのため床領域と同じく、利用者が変更できるのは表示名と間柱の割当だけである
// （`FloorRegion` に `AddFloorRegion`/`DeleteFloorRegion` が存在しないのと同じ理由）。

/// 壁領域の表示名変更。逆操作は変更前の名前への復元。
/// 対象が存在しない、または変更前後で同じ名前なら Noop。
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

/// 壁領域に属する間柱（`post_ids`）の一括変更。
///
/// バリデーション:
/// - 壁領域が実在すること
/// - 全 ID が `secondary_members` に実在し、かつ `kind == Post` であること
/// - ID に重複がないこと
///
/// 逆操作は元のリストへ戻す `SetWallRegionPostIds`。
pub struct SetWallRegionPostIds {
    pub region: WallRegionId,
    pub ids: Vec<SecondaryMemberId>,
}

impl EditCommand for SetWallRegionPostIds {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.region.index();
        if idx >= model.wall_regions.len() || model.wall_regions[idx].id != self.region {
            return Box::new(Noop);
        }
        if !secondary_post_ids_ok(model, &self.ids) {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut model.wall_regions[idx].post_ids, self.ids.clone());
        Box::new(SetWallRegionPostIds {
            region: self.region,
            ids: old,
        })
    }

    fn label(&self) -> &str {
        "壁領域間柱IDリスト変更"
    }
}

// ─── バリデーション用ヘルパー ─────────────────────────────

/// 二次部材小梁 ID リストが妥当か確認する。
/// 全 ID が実在し `kind == Joist` であること、重複がないことを確認する。
fn secondary_joist_ids_ok(model: &Model, ids: &[SecondaryMemberId]) -> bool {
    // 重複チェック
    let mut seen = std::collections::HashSet::new();
    for &id in ids {
        if !seen.insert(id) {
            return false;
        }
        let idx = id.index();
        match model.secondary_members.get(idx) {
            Some(sm) if sm.id == id && sm.kind == SecondaryMemberKind::Joist => {}
            _ => return false,
        }
    }
    true
}

/// 間柱 ID リストが妥当か確認する（[`secondary_joist_ids_ok`] の Post 版）。
fn secondary_post_ids_ok(model: &Model, ids: &[SecondaryMemberId]) -> bool {
    let mut seen = std::collections::HashSet::new();
    for &id in ids {
        if !seen.insert(id) {
            return false;
        }
        let idx = id.index();
        match model.secondary_members.get(idx) {
            Some(sm) if sm.id == id && sm.kind == SecondaryMemberKind::Post => {}
            _ => return false,
        }
    }
    true
}
