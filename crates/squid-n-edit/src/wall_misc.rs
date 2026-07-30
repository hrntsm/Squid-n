//! 壁（雑壁・壁属性）およびその他の編集コマンド。

use super::*;
use squid_n_core::ids::*;

/// 階の地震用重量の手入力（`weight_override`）変更。設定値は実効値
/// （`seismic_weight`）へも反映するため、解析・設計側は `seismic_weight` だけを
/// 読めばよい。`weight` に `None` を渡すと手入力を解除する（`seismic_weight` は
/// 次の準備計算で自動算定値へ戻る）。逆操作は変更前の両値への復元。
/// 存在しない `StoryId` は Noop。
pub struct SetStoryWeight {
    pub story: StoryId,
    pub weight: Option<f64>,
}

impl EditCommand for SetStoryWeight {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.story.index();
        if idx >= model.stories.len() || model.stories[idx].id != self.story {
            return Box::new(Noop);
        }
        let story = &mut model.stories[idx];
        let old = RestoreStoryWeight {
            story: self.story,
            weight_override: story.weight_override,
            seismic_weight: story.seismic_weight,
        };
        story.weight_override = self.weight;
        if let Some(w) = self.weight {
            story.seismic_weight = Some(w);
        }
        Box::new(old)
    }

    fn label(&self) -> &str {
        "階地震重量変更"
    }
}

/// [`SetStoryWeight`] の逆操作。手入力値と実効値の両方を変更前へ戻す。
pub struct RestoreStoryWeight {
    pub story: StoryId,
    pub weight_override: Option<f64>,
    pub seismic_weight: Option<f64>,
}

impl EditCommand for RestoreStoryWeight {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.story.index();
        if idx >= model.stories.len() || model.stories[idx].id != self.story {
            return Box::new(Noop);
        }
        let story = &mut model.stories[idx];
        let old = RestoreStoryWeight {
            story: self.story,
            weight_override: story.weight_override,
            seismic_weight: story.seismic_weight,
        };
        story.weight_override = self.weight_override;
        story.seismic_weight = self.seismic_weight;
        Box::new(old)
    }

    fn label(&self) -> &str {
        "階地震重量変更の取り消し"
    }
}

/// 荷重ケース種別（`LoadCaseKind`）変更（レビュー §1.7: 地震用重量に使う
/// 荷重ケースを並び順ではなく種別で明示的に選べるようにする）。
/// 存在しない `LoadCaseId` は Noop。
pub struct SetLoadCaseKind {
    pub id: LoadCaseId,
    pub kind: squid_n_core::model::LoadCaseKind,
}

impl EditCommand for SetLoadCaseKind {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.load_cases.len() || model.load_cases[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = model.load_cases[idx].kind;
        model.load_cases[idx].kind = self.kind;
        Box::new(SetLoadCaseKind {
            id: self.id,
            kind: old,
        })
    }

    fn label(&self) -> &str {
        "荷重ケース種別変更"
    }
}

/// スラブ荷重を専用の荷重ケースへ全置換で同期する（レビュー §1.1: 面荷重→大梁
/// 分配の結果を応力解析の荷重ケースへ接続する）。
///
/// `name` で既存ケースを探し、見つかれば `kind` を指定値に固定した上で
/// `nodal`/`member` を丸ごと置き換える（逆操作は置換前の `LoadCase` 全体の
/// 復元、[`RestoreLoadCaseContent`]）。見つからなければ [`AddLoadCase`] と同じ
/// 「末尾に `LoadCaseId(len)`」の規則で新規ケースを追加する（逆操作は
/// 既存の [`DeleteLoadCase`] をそのまま再利用できる）。
///
/// `kind` は同期先ケースの種別を指定する（床固定荷重・自重は `Dead`、
/// 床積載荷重は `Live` など。令85条1項の DL/LL 分離に用いる）。
///
/// 呼び出し側（`squid-n-app::App::sync_gravity_load_cases_action`）は、計算結果が
/// 既存ケースの内容と変わらない場合はこのコマンドを発行しない（undo 履歴を
/// 汚さないための冪等性は呼び出し側の責務）。
pub struct SyncSlabLoadsToCase {
    pub name: String,
    pub kind: squid_n_core::model::LoadCaseKind,
    pub nodal: Vec<squid_n_core::model::NodalLoad>,
    pub member: Vec<squid_n_core::model::MemberLoad>,
}

impl EditCommand for SyncSlabLoadsToCase {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        use squid_n_core::model::LoadCase;

        if let Some(idx) = model.load_cases.iter().position(|lc| lc.name == self.name) {
            let old = model.load_cases[idx].clone();
            model.load_cases[idx].kind = self.kind;
            model.load_cases[idx].nodal = self.nodal.clone();
            model.load_cases[idx].member = self.member.clone();
            Box::new(RestoreLoadCaseContent { old })
        } else {
            let new_id = LoadCaseId(model.load_cases.len() as u32);
            model.load_cases.push(LoadCase {
                id: new_id,
                name: self.name.clone(),
                kind: self.kind,
                nodal: self.nodal.clone(),
                member: self.member.clone(),
            });
            Box::new(DeleteLoadCase { id: new_id })
        }
    }

    fn label(&self) -> &str {
        "荷重ケースの同期"
    }
}

/// [`SyncSlabLoadsToCase`] が既存ケースを置換したときの逆操作。
/// 置換前の `LoadCase` を丸ごと復元する（[`RestoreSection`]・[`RestoreStories`]
/// と同様、自身を逆操作として返す対称パターン）。`id` が指す位置が
/// ずれている（他の操作で荷重ケースが削除された等）場合は Noop。
pub struct RestoreLoadCaseContent {
    pub old: squid_n_core::model::LoadCase,
}

impl EditCommand for RestoreLoadCaseContent {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.old.id.index();
        if idx >= model.load_cases.len() || model.load_cases[idx].id != self.old.id {
            return Box::new(Noop);
        }
        let replaced = std::mem::replace(&mut model.load_cases[idx], self.old.clone());
        Box::new(RestoreLoadCaseContent { old: replaced })
    }

    fn label(&self) -> &str {
        "荷重ケース内容の復元"
    }
}

/// 荷重計算条件（`LoadCfg`）を全置換する。`None` は「既定値扱い」を意味する
/// （`Model.load_cfg` の規約どおり）。逆操作は置換前の値への復元。
pub struct SetLoadCfg {
    pub cfg: Option<squid_n_core::model::LoadCfg>,
}

impl EditCommand for SetLoadCfg {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let old = std::mem::replace(&mut model.load_cfg, self.cfg.clone());
        Box::new(SetLoadCfg { cfg: old })
    }

    fn label(&self) -> &str {
        "荷重計算条件変更"
    }
}

/// 複数開口の取り扱い（`Model::multi_opening_mode`）を建物一律に変更する。
/// 逆操作は変更前のモードへの [`SetMultiOpeningMode`] 再実行（[`SetLoadCfg`]
/// と同様の対称パターン）。値が変化しない場合も同じ型を返す（Noop 相当。
/// 既存の値置換系コマンドの慣習どおり、同値判定による分岐は行わない）。
pub struct SetMultiOpeningMode {
    pub mode: squid_n_core::model::MultiOpeningMode,
}

impl EditCommand for SetMultiOpeningMode {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let old = std::mem::replace(&mut model.multi_opening_mode, self.mode);
        Box::new(SetMultiOpeningMode { mode: old })
    }

    fn label(&self) -> &str {
        "複数開口の取り扱い変更"
    }
}

/// 部材のねじり剛性の扱い（`Model::beam_torsion`）を建物一律に変更する。
///
/// 既定は「線材（梁・柱）の i 端ねじれをピン（解放）」で、`Keep` にすると
/// 全部材でねじり剛性 GJ/L を保持する。剛性そのものが変わるため、呼び出し側は
/// 実行後に結果を陳腐化させること（UI は `staleness.mark_edited`）。
/// 逆操作は変更前のモードへの再実行（[`SetMultiOpeningMode`] と同じ対称パターン）。
pub struct SetBeamTorsion {
    pub mode: squid_n_core::model::BeamTorsionMode,
}

impl EditCommand for SetBeamTorsion {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let old = std::mem::replace(&mut model.beam_torsion, self.mode);
        Box::new(SetBeamTorsion { mode: old })
    }

    fn label(&self) -> &str {
        "部材ねじり剛性の扱い変更"
    }
}

/// 壁要素（`ElementKind::Wall`/`Shell`）の自重算定属性（`WallAttr`）を
/// 追加/更新する。`attr.elem` に一致する既存エントリがあれば置換し、
/// 無ければ末尾に追加する。逆操作は変更前の状態への復元
/// （既存エントリの置換なら変更前の `WallAttr` で [`SetWallAttr`] を再実行、
/// 新規追加なら [`RemoveWallAttr`] で取り消す）。
pub struct SetWallAttr {
    pub attr: squid_n_core::model::WallAttr,
}

impl EditCommand for SetWallAttr {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if let Some(pos) = model
            .wall_attrs
            .iter()
            .position(|a| a.elem == self.attr.elem)
        {
            let old = model.wall_attrs[pos].clone();
            model.wall_attrs[pos] = self.attr.clone();
            Box::new(SetWallAttr { attr: old })
        } else {
            model.wall_attrs.push(self.attr.clone());
            Box::new(RemoveWallAttr {
                elem: self.attr.elem,
            })
        }
    }

    fn label(&self) -> &str {
        "壁属性変更"
    }
}

/// 壁属性エントリを削除する（`elem` に一致するものを削除）。一致するエントリが
/// 無ければ Noop。逆操作は削除前の値を復元する [`SetWallAttr`]
/// （このエントリの `elem` は削除時点で存在しないため、`SetWallAttr` は
/// 「既存エントリなし→末尾追加」の枝を通り、元の位置には戻らないが、
/// `wall_attrs` は `ElemId` をキーとする集合的なデータであり配列順に意味は
/// 無いため問題ない）。
pub struct RemoveWallAttr {
    pub elem: ElemId,
}

impl EditCommand for RemoveWallAttr {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if let Some(pos) = model.wall_attrs.iter().position(|a| a.elem == self.elem) {
            let old = model.wall_attrs.remove(pos);
            Box::new(SetWallAttr { attr: old })
        } else {
            Box::new(Noop)
        }
    }

    fn label(&self) -> &str {
        "壁属性削除"
    }
}

/// フレーム外雑壁（`MiscWall`）を追加。末尾に追加する。逆操作は末尾の雑壁削除。
pub struct AddMiscWall {
    pub wall: squid_n_core::model::MiscWall,
}

impl EditCommand for AddMiscWall {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        model.misc_walls.push(self.wall.clone());
        let index = model.misc_walls.len() - 1;
        Box::new(DeleteMiscWall { index })
    }

    fn label(&self) -> &str {
        "雑壁追加"
    }
}

/// 雑壁を index 指定で削除。逆操作は [`InsertMiscWall`]（同じ位置への復元）。
/// `MiscWall` は他データから参照されないため ID 再採番は不要。index が範囲外なら Noop。
pub struct DeleteMiscWall {
    pub index: usize,
}

impl EditCommand for DeleteMiscWall {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index >= model.misc_walls.len() {
            return Box::new(Noop);
        }
        let removed = model.misc_walls.remove(self.index);
        Box::new(InsertMiscWall {
            index: self.index,
            wall: removed,
        })
    }

    fn label(&self) -> &str {
        "雑壁削除"
    }
}

/// 指定インデックスへ雑壁を再挿入する（[`DeleteMiscWall`] の逆操作専用）。
pub struct InsertMiscWall {
    pub index: usize,
    pub wall: squid_n_core::model::MiscWall,
}

impl EditCommand for InsertMiscWall {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.misc_walls.len() {
            return Box::new(Noop);
        }
        model.misc_walls.insert(self.index, self.wall.clone());
        Box::new(DeleteMiscWall { index: self.index })
    }

    fn label(&self) -> &str {
        "雑壁削除の取り消し"
    }
}

/// 雑壁の内容を index 指定で置換する（フィールド編集用）。逆操作は変更前の
/// 内容への復元。index が範囲外なら Noop。
pub struct SetMiscWall {
    pub index: usize,
    pub wall: squid_n_core::model::MiscWall,
}

impl EditCommand for SetMiscWall {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index >= model.misc_walls.len() {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut model.misc_walls[self.index], self.wall.clone());
        Box::new(SetMiscWall {
            index: self.index,
            wall: old,
        })
    }

    fn label(&self) -> &str {
        "雑壁変更"
    }
}

/// 階の種別（一般/PH/地下、`StoryLevelKind`）変更。逆操作は変更前の値への復元。
/// 存在しない `StoryId` は Noop。
pub struct SetStoryLevelKind {
    pub story: StoryId,
    pub level_kind: squid_n_core::model::StoryLevelKind,
}

impl EditCommand for SetStoryLevelKind {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.story.index();
        if idx >= model.stories.len() || model.stories[idx].id != self.story {
            return Box::new(Noop);
        }
        let old = model.stories[idx].level_kind;
        model.stories[idx].level_kind = self.level_kind;
        Box::new(SetStoryLevelKind {
            story: self.story,
            level_kind: old,
        })
    }

    fn label(&self) -> &str {
        "階種別変更"
    }
}

/// スラブ種別（`SlabKind`: 一般/片持ち/出隅）変更。逆操作は変更前の値への復元。
/// 存在しない `SlabId` は Noop。
pub struct SetSlabKind {
    pub id: SlabId,
    pub kind: squid_n_core::model::SlabKind,
}

impl EditCommand for SetSlabKind {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.slabs.len() || model.slabs[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = model.slabs[idx].kind;
        model.slabs[idx].kind = self.kind;
        Box::new(SetSlabKind {
            id: self.id,
            kind: old,
        })
    }

    fn label(&self) -> &str {
        "スラブ種別変更"
    }
}

/// スラブの一方向伝達方向（`one_way`）変更。逆操作は変更前の値への復元。
/// 存在しない `SlabId` は Noop。
pub struct SetSlabOneWay {
    pub id: SlabId,
    pub one_way: Option<squid_n_core::model::OneWayDir>,
}

impl EditCommand for SetSlabOneWay {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.slabs.len() || model.slabs[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = model.slabs[idx].one_way;
        model.slabs[idx].one_way = self.one_way;
        Box::new(SetSlabOneWay {
            id: self.id,
            one_way: old,
        })
    }

    fn label(&self) -> &str {
        "スラブ伝達方向変更"
    }
}

/// スラブの用途（`usage`。積載荷重プリセット）変更。逆操作は変更前の値への復元。
/// 存在しない `SlabId` は Noop。
pub struct SetSlabUsage {
    pub id: SlabId,
    pub usage: Option<squid_n_core::model::SlabUsage>,
}

impl EditCommand for SetSlabUsage {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.slabs.len() || model.slabs[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = model.slabs[idx].usage;
        model.slabs[idx].usage = self.usage;
        Box::new(SetSlabUsage {
            id: self.id,
            usage: old,
        })
    }

    fn label(&self) -> &str {
        "スラブ用途変更"
    }
}

/// スラブの小梁（`joists`。二段階伝達の小梁ライン）を全置換する。逆操作は
/// 変更前の `joists` への復元（`SetLoadCfg` と同様の値置換パターン）。
/// 存在しない `SlabId` は Noop。
pub struct SetSlabJoists {
    pub id: SlabId,
    pub joists: Vec<squid_n_core::model::JoistLine>,
}

impl EditCommand for SetSlabJoists {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.slabs.len() || model.slabs[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut model.slabs[idx].joists, self.joists.clone());
        Box::new(SetSlabJoists {
            id: self.id,
            joists: old,
        })
    }

    fn label(&self) -> &str {
        "スラブ小梁変更"
    }
}
