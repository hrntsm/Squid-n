//! 階（[`Story`]）の編集コマンド。
//!
//! 階は利用者が定義するデータであり（`squid_n_core::model::story`）、
//! 階名・階レベルの変更、階の追加・削除をここで行う。準備計算が埋める欄
//! （所属節点・地震用重量・主要構造種別）と、剛床（`Constraint::RigidDiaphragm`）は
//! 触らない。階の追加・削除の直後はそれらが古い状態のまま残るため、呼び出し側は
//! 続けて階生成（準備計算）を実行して整合させる。
//!
//! `model.stories` は **`elevation` の昇順**かつ **`StoryId` ＝配列位置**という
//! 2 つの不変条件を持つ。追加・削除はこの両方を保つため、挿入位置を標高から決め、
//! 以降の階の ID を [`Model::visit_story_ids`] で繰り上げる。

use crate::EditCommand;
use squid_n_core::ids::StoryId;
use squid_n_core::model::{Model, Story, StoryLevelKind};

/// 階の階名と階レベルを設定する（階種別は [`crate::SetStoryLevelKind`]）。
///
/// 標高を変えると並び順の不変条件が崩れうるため、適用後に標高昇順へ並べ替え、
/// ID を振り直す。逆操作は「変更前の階定義の復元」とする
/// （並べ替えで他階の ID も動きうるため、1 階分の差分では戻せない）。
pub struct SetStoryLevel {
    pub story: StoryId,
    pub name: String,
    pub elevation: f64,
}

/// 階の利用者定義欄の一括復元（[`SetStoryLevel`]・[`AddStory`]・[`DeleteStory`] の逆操作）。
///
/// `model.stories` を丸ごと差し替え、`StoryId` の参照も復元前の対応へ戻す。
pub struct RestoreStoryDefs {
    pub stories: Vec<Story>,
    /// 復元後の各節点の所属階（`model.nodes` と同順）。
    pub node_story: Vec<Option<StoryId>>,
    /// 復元後の拘束（剛床の `story` 参照を含む）。
    pub constraints: Vec<squid_n_core::model::Constraint>,
}

/// 現在の階定義・階参照のスナップショットを撮る。
fn snapshot(model: &Model) -> RestoreStoryDefs {
    RestoreStoryDefs {
        stories: model.stories.clone(),
        node_story: model.nodes.iter().map(|n| n.story).collect(),
        constraints: model.constraints.clone(),
    }
}

/// 標高昇順へ並べ替え、`StoryId` ＝配列位置になるよう全参照を振り直す。
///
/// 並べ替え前の ID から並べ替え後の ID への対応表を作り、
/// [`Model::visit_story_ids`] で一括置換する。
fn resort_and_renumber(model: &mut Model) {
    // 旧 ID（＝現在の配列位置）を保持したまま標高昇順へ並べる。
    let mut order: Vec<usize> = (0..model.stories.len()).collect();
    order.sort_by(|&a, &b| {
        model.stories[a]
            .elevation
            .total_cmp(&model.stories[b].elevation)
    });
    // old_id → new_id（添字は並べ替え前の `Story::id`＝配列位置）
    let mut remap = vec![StoryId(0); model.stories.len()];
    for (new_idx, &old_idx) in order.iter().enumerate() {
        if let Some(slot) = remap.get_mut(model.stories[old_idx].id.index()) {
            *slot = StoryId(new_idx as u32);
        }
    }
    model.visit_story_ids(|sid| {
        if let Some(&new) = remap.get(sid.index()) {
            *sid = new;
        }
    });
    // 参照を書き換えたあとに実体を並べ替える（並べ替えを先にすると
    // `visit_story_ids` が拾う `Story::id` の対応が崩れる）。
    model.stories.sort_by_key(|s| s.id.0);
}

impl EditCommand for SetStoryLevel {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let before = snapshot(model);
        let Some(story) = model.stories.get_mut(self.story.index()) else {
            return Box::new(crate::Noop);
        };
        story.name = self.name.clone();
        story.elevation = self.elevation;
        resort_and_renumber(model);
        Box::new(before)
    }

    fn label(&self) -> &str {
        "階の設定変更"
    }
}

/// 階を追加する。標高の昇順を保つ位置へ挿入し、以降の階の ID を繰り上げる。
///
/// 所属節点・地震用重量は空のまま追加する（準備計算が埋める）。
pub struct AddStory {
    pub name: String,
    pub elevation: f64,
}

impl EditCommand for AddStory {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let before = snapshot(model);
        // 末尾へ加えてから並べ替える（挿入位置の計算と ID 繰り上げを
        // `resort_and_renumber` の 1 箇所に集約する）。
        model.stories.push(Story {
            id: StoryId(model.stories.len() as u32),
            name: self.name.clone(),
            elevation: self.elevation,
            node_ids: Vec::new(),
            seismic_weight: None,
            weight_override: None,
            structure: Default::default(),
            level_kind: StoryLevelKind::default(),
        });
        resort_and_renumber(model);
        Box::new(before)
    }

    fn label(&self) -> &str {
        "階の追加"
    }
}

/// 階を削除する。**階の定義だけを消し、節点・部材は残す**。
///
/// 階は法規上の層の定義であって部材の入れ物ではないため、削除しても構造は
/// 変わらない。削除した階に属していた節点は所属階を失い、次の階生成で
/// 直下階の区間へ吸収される。その階の剛床拘束は意味を失うため取り除く。
pub struct DeleteStory {
    pub story: StoryId,
}

impl EditCommand for DeleteStory {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if model.stories.get(self.story.index()).is_none() {
            return Box::new(crate::Noop);
        }
        let before = snapshot(model);
        model.stories.remove(self.story.index());
        // 削除した階の剛床は載る先を失うため取り除く。
        model.constraints.retain(|c| {
            !matches!(
                c,
                squid_n_core::model::Constraint::RigidDiaphragm { story, .. } if *story == self.story
            )
        });
        // 削除した階に属していた節点の所属階を外し、以降の階の ID を繰り上げる。
        let removed = self.story;
        for node in &mut model.nodes {
            match node.story {
                Some(s) if s == removed => node.story = None,
                Some(s) if s.0 > removed.0 => node.story = Some(StoryId(s.0 - 1)),
                _ => {}
            }
        }
        for c in &mut model.constraints {
            if let squid_n_core::model::Constraint::RigidDiaphragm { story, .. } = c {
                if story.0 > removed.0 {
                    *story = StoryId(story.0 - 1);
                }
            }
        }
        for (i, story) in model.stories.iter_mut().enumerate() {
            story.id = StoryId(i as u32);
        }
        Box::new(before)
    }

    fn label(&self) -> &str {
        "階の削除"
    }
}

impl EditCommand for RestoreStoryDefs {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let redo = snapshot(model);
        model.stories = self.stories.clone();
        for (node, story) in model.nodes.iter_mut().zip(self.node_story.iter()) {
            node.story = *story;
        }
        model.constraints = self.constraints.clone();
        Box::new(redo)
    }

    fn label(&self) -> &str {
        "階定義の復元"
    }
}
