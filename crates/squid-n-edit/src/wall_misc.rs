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

/// スラブ荷重を専用の荷重ケースへ同期する（レビュー §1.1: 面荷重→大梁
/// 分配の結果を応力解析の荷重ケースへ接続する）。
///
/// `name` で既存ケースを探し、見つかれば `kind` を指定値に固定した上で
/// **自動生成分（`LoadSource::Auto`）だけを** `nodal`/`member` で置き換える。
/// 利用者が同じケースへ手入力した荷重（`LoadSource::Manual`）は順序を保って
/// 残る（逆操作は置換前の `LoadCase` 全体の
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

        // 分配結果が実在しない節点・部材を指す場合は同期しない（crate::refs の規約）。
        if !self
            .nodal
            .iter()
            .all(|l| crate::refs::node_exists(model, l.node))
            || !self
                .member
                .iter()
                .all(|l| crate::refs::elem_exists(model, l.elem))
        {
            return Box::new(Noop);
        }
        if let Some(idx) = model.load_cases.iter().position(|lc| lc.name == self.name) {
            let old = model.load_cases[idx].clone();
            model.load_cases[idx].kind = self.kind;
            model.load_cases[idx].replace_auto_loads(self.nodal.clone(), self.member.clone());
            Box::new(RestoreLoadCaseContent { old })
        } else {
            let new_id = LoadCaseId(model.load_cases.len() as u32);
            let mut case = LoadCase {
                id: new_id,
                name: self.name.clone(),
                kind: self.kind,
                nodal: Vec::new(),
                member: Vec::new(),
            };
            // 新規作成でも `replace_auto_loads` を通し、内容を自動生成分として
            // 積む（直接代入すると手入力扱いのまま残り、次回の同期で消えずに増える）。
            case.replace_auto_loads(self.nodal.clone(), self.member.clone());
            model.load_cases.push(case);
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

/// 仕口パネルのモデル化（`Model::panel_zone`）を建物一律に変更する。
///
/// 既定は「モデル化する」で、`None` にすると接合部を剛節点として扱う従来の
/// モデル化へ戻る。パネル要素の生成有無と接合部の剛性が変わるため、呼び出し側は
/// 実行後に結果を陳腐化させること（UI は `staleness.mark_edited`）。
/// 逆操作は変更前のモードへの再実行（[`SetBeamTorsion`] と同じ対称パターン）。
pub struct SetPanelZoneMode {
    pub mode: squid_n_core::model::PanelZoneMode,
}

impl EditCommand for SetPanelZoneMode {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let old = std::mem::replace(&mut model.panel_zone, self.mode);
        Box::new(SetPanelZoneMode { mode: old })
    }

    fn label(&self) -> &str {
        "仕口パネルのモデル化変更"
    }
}

/// フレーム外雑壁（`OutOfFrameMiscWall`）を追加。末尾に追加する。逆操作は末尾の雑壁削除。
pub struct AddMiscWall {
    pub wall: squid_n_core::model::OutOfFrameMiscWall,
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

indexed_delete_insert!(
    /// 雑壁を index 指定で削除。逆操作は [`InsertMiscWall`]（同じ位置への復元）。
    /// `OutOfFrameMiscWall` は他データから参照されないため ID 再採番は不要。index が範囲外なら Noop。
    DeleteMiscWall,
    /// 指定インデックスへ雑壁を再挿入する（[`DeleteMiscWall`] の逆操作専用）。
    InsertMiscWall,
    entity = squid_n_core::model::OutOfFrameMiscWall,
    vec = misc_walls,
    field = wall,
    del_label = "雑壁削除",
    ins_label = "雑壁削除の取り消し",
);

/// 雑壁の内容を index 指定で置換する（フィールド編集用）。逆操作は変更前の
/// 内容への復元。index が範囲外なら Noop。
pub struct SetMiscWall {
    pub index: usize,
    pub wall: squid_n_core::model::OutOfFrameMiscWall,
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

/// 床板の一方向伝達方向（`one_way`）変更。逆操作は変更前の値への復元。
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
        let old = model.slabs[idx].plate.one_way;
        model.slabs[idx].plate.one_way = self.one_way;
        Box::new(SetSlabOneWay {
            id: self.id,
            one_way: old,
        })
    }

    fn label(&self) -> &str {
        "床板伝達方向変更"
    }
}

/// 床領域の表示名変更。逆操作は変更前の名前への復元。
/// 存在しない `FloorRegionId` は Noop。
pub struct SetFloorRegionName {
    pub id: FloorRegionId,
    pub name: String,
}

impl EditCommand for SetFloorRegionName {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.floor_regions.len() || model.floor_regions[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut model.floor_regions[idx].name, self.name.clone());
        if old == self.name {
            return Box::new(Noop);
        }
        Box::new(SetFloorRegionName {
            id: self.id,
            name: old,
        })
    }

    fn label(&self) -> &str {
        "床領域名変更"
    }
}

/// 取り付く床板（片持ちスラブ・バルコニー・出隅）の追加。末尾に追加する。
/// 逆操作は末尾の床板削除（[`DeleteSlabEntity`]）。
///
/// 取付き線・取付き点が実在しない節点を指す場合は Noop。取付き線の張り出し量は
/// 符号つきで、取付き線 `nodes[0]`→`nodes[1]` の左側を正とする。
pub struct AddAttachedSlab {
    pub anchor: squid_n_core::model::RegionAnchor,
    pub extent: [f64; 2],
    pub plate: squid_n_core::model::SlabPlate,
}

impl EditCommand for AddAttachedSlab {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        use squid_n_core::model::{RegionAnchor, Slab, SlabShape};

        let nodes_ok = match self.anchor {
            RegionAnchor::Line { nodes, span, .. } => {
                // span の範囲は `Model::validate` と同じ規約（0.0 <= t_i < t_j <= 1.0）。
                let span_ok = span[0].is_finite()
                    && span[1].is_finite()
                    && span[0] >= -1e-9
                    && span[1] <= 1.0 + 1e-9
                    && span[1] - span[0] > 1e-9;
                if !span_ok {
                    return Box::new(Noop);
                }
                nodes[0] != nodes[1] && nodes.iter().all(|&n| crate::refs::node_exists(model, n))
            }
            RegionAnchor::Point(n) => crate::refs::node_exists(model, n),
            // 床板の取付き先には使わない（`RegionAnchor::FloorRegion` のドキュメント
            // 参照。壁側〔自立壁〕専用のアンカーであり、床板では常に不正）。
            RegionAnchor::FloorRegion { .. } => false,
        };
        if !nodes_ok {
            return Box::new(Noop);
        }
        if !self.extent[0].is_finite() || !self.extent[1].is_finite() {
            return Box::new(Noop);
        }
        if !crate::refs::section_ref_ok(model, self.plate.section) {
            return Box::new(Noop);
        }
        let id = SlabId(model.slabs.len() as u32);
        model.slabs.push(Slab {
            id,
            shape: SlabShape::Attached {
                anchor: self.anchor,
                extent: self.extent,
            },
            plate: self.plate.clone(),
        });
        Box::new(crate::DeleteSlab { id })
    }

    fn label(&self) -> &str {
        "取り付く床板追加"
    }
}

/// 取り付く床板の張り出し量（`extent`）変更。逆操作は変更前の値への復元。
/// 対象が取り付く床板でない、存在しない `SlabId`、または非有限の張り出し量は Noop。
pub struct SetAttachedExtent {
    pub id: SlabId,
    pub extent: [f64; 2],
}

impl EditCommand for SetAttachedExtent {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        use squid_n_core::model::SlabShape;

        let idx = self.id.index();
        if idx >= model.slabs.len() || model.slabs[idx].id != self.id {
            return Box::new(Noop);
        }
        if !self.extent[0].is_finite() || !self.extent[1].is_finite() {
            return Box::new(Noop);
        }
        let SlabShape::Attached { extent, .. } = &mut model.slabs[idx].shape else {
            return Box::new(Noop);
        };
        let old = *extent;
        *extent = self.extent;
        Box::new(SetAttachedExtent {
            id: self.id,
            extent: old,
        })
    }

    fn label(&self) -> &str {
        "取り付き張り出し量変更"
    }
}

/// 取り付く床板の取付き先（`anchor`）変更。逆操作は変更前の値への復元。
/// 対象が取り付く床板でない、存在しない `SlabId`、および
/// [`AddAttachedSlab`] と同じ取付き先検証に落ちる場合は Noop。
pub struct SetAttachedAnchor {
    pub id: SlabId,
    pub anchor: squid_n_core::model::RegionAnchor,
}

impl EditCommand for SetAttachedAnchor {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        use squid_n_core::model::{RegionAnchor, SlabShape};

        let idx = self.id.index();
        if idx >= model.slabs.len() || model.slabs[idx].id != self.id {
            return Box::new(Noop);
        }
        if !matches!(model.slabs[idx].shape, SlabShape::Attached { .. }) {
            return Box::new(Noop);
        }
        let nodes_ok = match self.anchor {
            RegionAnchor::Line { nodes, span, .. } => {
                // span の範囲は `Model::validate` と同じ規約（0.0 <= t_i < t_j <= 1.0）。
                let span_ok = span[0].is_finite()
                    && span[1].is_finite()
                    && span[0] >= -1e-9
                    && span[1] <= 1.0 + 1e-9
                    && span[1] - span[0] > 1e-9;
                if !span_ok {
                    return Box::new(Noop);
                }
                nodes[0] != nodes[1] && nodes.iter().all(|&n| crate::refs::node_exists(model, n))
            }
            RegionAnchor::Point(n) => crate::refs::node_exists(model, n),
            // 床板の取付き先には使わない（`RegionAnchor::FloorRegion` のドキュメント
            // 参照。壁側〔自立壁〕専用のアンカーであり、床板では常に不正）。
            RegionAnchor::FloorRegion { .. } => false,
        };
        if !nodes_ok {
            return Box::new(Noop);
        }
        let SlabShape::Attached { anchor, .. } = &mut model.slabs[idx].shape else {
            return Box::new(Noop);
        };
        let old = *anchor;
        *anchor = self.anchor;
        Box::new(SetAttachedAnchor {
            id: self.id,
            anchor: old,
        })
    }

    fn label(&self) -> &str {
        "取り付き先変更"
    }
}

// ─── 取り付く壁版（パラペット・腰壁・垂れ壁・自立壁） ────────────────
//
// 壁版（`WallPlate`）は柱・梁で囲まれた領域（`Enclosed`）が主で、これは
// ST-Bridge 取り込み・`rebuild_wall_regions`（床側 D20 相当。
// `dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` §9）が組み立てる派生的な
// 入力である（`secondary.rs` の「グループC」コメント参照）。取り付く壁版
// （`Attached`）だけは、ST-Bridge から自動検出できない自立壁を含め、利用者が
// 直接作る対象になりうるため、床側の `AddAttachedSlab` と同じ位置づけで
// 追加・削除コマンドを持つ。

/// 取り付く壁版の取付き先（`RegionAnchor`）が妥当か。
///
/// 壁の取付き先としては `Line`（梁に載るパラペット・腰壁・垂れ壁）と
/// `FloorRegion`（自立壁）のみを認める。`Point` は壁の取付き先としては使わない
/// （`WallPlateShape::Attached` のドキュメント参照。出隅スラブ専用）。
fn wall_anchor_ok(model: &Model, anchor: &squid_n_core::model::RegionAnchor) -> bool {
    use squid_n_core::model::RegionAnchor;

    match *anchor {
        RegionAnchor::Line { nodes, span, .. } => {
            // span の範囲は `Model::validate` と同じ規約（0.0 <= t_i < t_j <= 1.0）。
            let span_ok = span[0].is_finite()
                && span[1].is_finite()
                && span[0] >= -1e-9
                && span[1] <= 1.0 + 1e-9
                && span[1] - span[0] > 1e-9;
            span_ok
                && nodes[0] != nodes[1]
                && nodes.iter().all(|&n| crate::refs::node_exists(model, n))
        }
        // 自立壁が荷重を渡す床領域は保存しないため、検証する床領域参照は無い
        // （`RegionAnchor::FloorRegion` のドキュメント）。荷重を流せる床領域に
        // 載っているかは幾何の問題で、解析前チェック（`model_issues`）が見る。
        // ここで幾何判定まで行うと、同じ判定が 2 か所に分かれる。
        RegionAnchor::FloorRegion { nodes } => {
            nodes[0] != nodes[1] && nodes.iter().all(|&n| crate::refs::node_exists(model, n))
        }
        RegionAnchor::Point(_) => false,
    }
}

/// 壁版の削除（中間の壁版も可）。逆操作は [`InsertWallPlate`]。
///
/// ID＝配列インデックスの不変条件を保つため、削除後は当該壁版より後ろの
/// ID を 1 つずつ繰り上げる。削除前に `WallRegion.wall_plate_ids` から該当 ID を
/// 除去する（カスケード削除。[`crate::DeleteSlab`] と同じ理由）。
pub struct DeleteWallPlate {
    pub id: WallPlateId,
}

impl EditCommand for DeleteWallPlate {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.wall_plates.len() || model.wall_plates[idx].id != self.id {
            return Box::new(Noop);
        }

        // カスケード: 壁領域の wall_plate_ids から除去し、位置を退避
        // (壁領域添字, リスト内位置) を昇順で記録する（InsertWallPlate での復元用）。
        let mut region_refs: Vec<(usize, usize)> = Vec::new();
        for (ri, region) in model.wall_regions.iter_mut().enumerate() {
            let mut pos = 0;
            while pos < region.wall_plate_ids.len() {
                if region.wall_plate_ids[pos] == self.id {
                    region.wall_plate_ids.remove(pos);
                    region_refs.push((ri, pos));
                } else {
                    pos += 1;
                }
            }
        }

        let removed = model.wall_plates.remove(idx);
        let target = self.id.0;
        model.visit_wall_plate_ids(|id| {
            if id.0 > target {
                id.0 -= 1;
            }
        });

        Box::new(InsertWallPlate {
            index: idx,
            plate: removed,
            region_refs,
        })
    }

    fn label(&self) -> &str {
        "壁版削除"
    }
}

/// [`DeleteWallPlate`] の取り消し。
pub struct InsertWallPlate {
    pub index: usize,
    pub plate: squid_n_core::model::WallPlate,
    /// 削除時に壁領域の `wall_plate_ids` から除去した参照の (壁領域添字, リスト内位置)。
    /// 昇順で記録されているため逆順で挿入して元の並びを復元する。
    pub region_refs: Vec<(usize, usize)>,
}

impl EditCommand for InsertWallPlate {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.wall_plates.len() {
            return Box::new(Noop);
        }
        let id = WallPlateId(self.index as u32);
        model.visit_wall_plate_ids(|x| {
            if x.0 >= id.0 {
                x.0 += 1;
            }
        });
        let mut plate = self.plate.clone();
        plate.id = id;
        model.wall_plates.insert(self.index, plate);

        // 壁領域の wall_plate_ids を元の位置へ復元（逆順挿入で昇順復元）。
        for &(ri, pos) in self.region_refs.iter().rev() {
            if let Some(region) = model.wall_regions.get_mut(ri) {
                let insert_pos = pos.min(region.wall_plate_ids.len());
                region.wall_plate_ids.insert(insert_pos, id);
            }
        }

        Box::new(DeleteWallPlate { id })
    }

    fn label(&self) -> &str {
        "壁版削除の取り消し"
    }
}

/// 取り付く壁版（パラペット・腰壁・垂れ壁・自立壁）の追加。末尾に追加する。
/// 逆操作は末尾の壁版削除（[`DeleteWallPlate`]）。
///
/// 取付き先が実在しない節点・床領域を指す場合、および壁の取付き先として
/// 使わない `RegionAnchor::Point` を渡した場合は Noop（[`wall_anchor_ok`]）。
/// 張り出し量 `extent` は鉛直上向きが正、符号つきで負なら下向き
/// （垂れ壁）に張り出す。
pub struct AddAttachedWallPlate {
    pub anchor: squid_n_core::model::RegionAnchor,
    pub extent: [f64; 2],
    pub section: Option<SectionId>,
    pub opening_area: f64,
    pub opening_weight: f64,
}

impl EditCommand for AddAttachedWallPlate {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        use squid_n_core::model::{WallPlate, WallPlateShape};

        if !wall_anchor_ok(model, &self.anchor) {
            return Box::new(Noop);
        }
        if !self.extent[0].is_finite() || !self.extent[1].is_finite() {
            return Box::new(Noop);
        }
        if !crate::refs::section_ref_ok(model, self.section) {
            return Box::new(Noop);
        }
        let id = WallPlateId(model.wall_plates.len() as u32);
        model.wall_plates.push(WallPlate {
            id,
            shape: WallPlateShape::Attached {
                anchor: self.anchor,
                extent: self.extent,
            },
            section: self.section,
            opening_area: self.opening_area,
            opening_weight: self.opening_weight,
            openings: Vec::new(),
            three_side_slit: false,
        });
        Box::new(DeleteWallPlate { id })
    }

    fn label(&self) -> &str {
        "取り付く壁版追加"
    }
}

/// 柱・梁で囲まれた壁版（`WallPlateShape::Enclosed`）の追加。末尾に追加する。
/// 逆操作は末尾の壁版削除（[`DeleteWallPlate`]）。
///
/// ここで作るのは**柱・梁が囲む鉛直構面内の壁版**である。所属する壁領域は
/// 次の準備計算（`rebuild_wall_regions`）が自動で結びつける。
/// 取り付く壁版の追加コマンドは [`AddAttachedWallPlate`]。
///
/// 境界が実在しない節点を指す場合、境界が空、または実在しない断面を指す
/// 割当は Noop（[`AddSlab`] と同じ規約）。
pub struct AddEnclosedWallPlate {
    pub boundary: Vec<NodeId>,
    pub section: Option<SectionId>,
    pub opening_area: f64,
    pub opening_weight: f64,
}

impl EditCommand for AddEnclosedWallPlate {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        use squid_n_core::model::{WallPlate, WallPlateShape};

        if self.boundary.is_empty()
            || !self
                .boundary
                .iter()
                .all(|&n| crate::refs::node_exists(model, n))
            || !crate::refs::section_ref_ok(model, self.section)
        {
            return Box::new(Noop);
        }
        let id = WallPlateId(model.wall_plates.len() as u32);
        model.wall_plates.push(WallPlate {
            id,
            shape: WallPlateShape::Enclosed {
                boundary: self.boundary.clone(),
            },
            section: self.section,
            opening_area: self.opening_area,
            opening_weight: self.opening_weight,
            openings: Vec::new(),
            three_side_slit: false,
        });
        Box::new(DeleteWallPlate { id })
    }

    fn label(&self) -> &str {
        "囲まれた壁版追加"
    }
}

/// 取り付く壁版の張り出し量（`extent`）変更。逆操作は変更前の値への復元。
/// 対象が取り付く壁版でない、存在しない `WallPlateId`、または非有限の
/// 張り出し量は Noop。
pub struct SetAttachedWallPlateExtent {
    pub id: WallPlateId,
    pub extent: [f64; 2],
}

impl EditCommand for SetAttachedWallPlateExtent {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        use squid_n_core::model::WallPlateShape;

        let idx = self.id.index();
        if idx >= model.wall_plates.len() || model.wall_plates[idx].id != self.id {
            return Box::new(Noop);
        }
        if !self.extent[0].is_finite() || !self.extent[1].is_finite() {
            return Box::new(Noop);
        }
        let WallPlateShape::Attached { extent, .. } = &mut model.wall_plates[idx].shape else {
            return Box::new(Noop);
        };
        let old = *extent;
        *extent = self.extent;
        Box::new(SetAttachedWallPlateExtent {
            id: self.id,
            extent: old,
        })
    }

    fn label(&self) -> &str {
        "壁版取り付き張り出し量変更"
    }
}

/// 取り付く壁版の取付き先（`anchor`）変更。逆操作は変更前の値への復元。
/// 対象が取り付く壁版でない、存在しない `WallPlateId`、および [`wall_anchor_ok`]
/// に落ちる場合は Noop。
pub struct SetAttachedWallPlateAnchor {
    pub id: WallPlateId,
    pub anchor: squid_n_core::model::RegionAnchor,
}

impl EditCommand for SetAttachedWallPlateAnchor {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        use squid_n_core::model::WallPlateShape;

        let idx = self.id.index();
        if idx >= model.wall_plates.len() || model.wall_plates[idx].id != self.id {
            return Box::new(Noop);
        }
        if !matches!(
            model.wall_plates[idx].shape,
            WallPlateShape::Attached { .. }
        ) {
            return Box::new(Noop);
        }
        if !wall_anchor_ok(model, &self.anchor) {
            return Box::new(Noop);
        }
        let WallPlateShape::Attached { anchor, .. } = &mut model.wall_plates[idx].shape else {
            return Box::new(Noop);
        };
        let old = *anchor;
        *anchor = self.anchor;
        Box::new(SetAttachedWallPlateAnchor {
            id: self.id,
            anchor: old,
        })
    }

    fn label(&self) -> &str {
        "壁版取り付き先変更"
    }
}

/// 壁版の断面（`section`。板厚・材料を持つ断面）変更。逆操作は変更前の値への
/// 復元。存在しない `WallPlateId`、および実在しない断面を指す割当は Noop
/// （[`SetSlabSection`] と同じ規約）。
///
/// 断面は壁版の自重（板厚 × 材料密度）と、囲まれた壁版では解析要素の生成可否
/// （`squid_n_load::wall_expand` は断面未割当の壁版から要素を作らない）を
/// 決めるため、囲まれた壁版・取り付く壁版のどちらにも意味を持つ。
pub struct SetWallPlateSection {
    pub id: WallPlateId,
    pub section: Option<SectionId>,
}

impl EditCommand for SetWallPlateSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.wall_plates.len() || model.wall_plates[idx].id != self.id {
            return Box::new(Noop);
        }
        if !crate::refs::section_ref_ok(model, self.section) {
            return Box::new(Noop);
        }
        let old = model.wall_plates[idx].section;
        model.wall_plates[idx].section = self.section;
        Box::new(SetWallPlateSection {
            id: self.id,
            section: old,
        })
    }

    fn label(&self) -> &str {
        "壁版断面変更"
    }
}

/// 壁版の自重算定属性（開口面積・個別開口・開口部重量・三方スリット）を
/// 一括変更する。逆操作は変更前の値への復元。存在しない `WallPlateId` は Noop。
///
/// 開口の 4 つの値をひとまとめにするのは、`openings`（個別開口）が非空のとき
/// `opening_area` が無視されるという相互依存があるためである
/// （[`squid_n_core::model::WallPlate::total_opening_area`]）。別々のコマンドに
/// 分けると、undo の途中に「個別開口だけ戻って面積が残る」という、利用者が
/// 一度も入力していない組み合わせが現れうる。
///
/// `three_side_slit` は囲まれた壁版（`Enclosed`）が解析要素として生成される
/// ときにだけ効く（自重を上下に分けず頂部へ寄せる指定。
/// `squid_n_load::story_gen::self_weight_calc`）。取り付く壁版
/// （`Attached`。腰壁・垂れ壁・パラペット・自立壁）には自重を分ける相手方の
/// 下端がそもそも無く、`squid_n_load::wall_attached` はこのフィールドを読まない。
/// GUI は取り付く壁版でこの入力欄自体を出さないが、コマンドは形によらず値を
/// そのまま保存する（`WallPlate` が形によらず同じフィールドを持つ設計〔D3〕を
/// コマンド側で崩さないため）。
pub struct SetWallPlateAttrs {
    pub id: WallPlateId,
    pub opening_area: f64,
    pub opening_weight: f64,
    pub openings: Vec<squid_n_core::model::WallOpening>,
    pub three_side_slit: bool,
}

impl EditCommand for SetWallPlateAttrs {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.wall_plates.len() || model.wall_plates[idx].id != self.id {
            return Box::new(Noop);
        }
        let plate = &mut model.wall_plates[idx];
        let old = SetWallPlateAttrs {
            id: self.id,
            opening_area: plate.opening_area,
            opening_weight: plate.opening_weight,
            openings: plate.openings.clone(),
            three_side_slit: plate.three_side_slit,
        };
        plate.opening_area = self.opening_area;
        plate.opening_weight = self.opening_weight;
        plate.openings = self.openings.clone();
        plate.three_side_slit = self.three_side_slit;
        Box::new(old)
    }

    fn label(&self) -> &str {
        "壁版属性変更"
    }
}

/// 床板の用途（`usage`。積載荷重プリセット）変更。逆操作は変更前の値への復元。
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
        let old = model.slabs[idx].plate.usage;
        model.slabs[idx].plate.usage = self.usage;
        Box::new(SetSlabUsage {
            id: self.id,
            usage: old,
        })
    }

    fn label(&self) -> &str {
        "床板用途変更"
    }
}

/// 床板の断面（`section`。板厚・コンクリート材料を持つ断面）変更。
/// 逆操作は変更前の値への復元。存在しない `SlabId`、および実在しない断面を
/// 指す割当は Noop（crate::refs の規約）。
pub struct SetSlabSection {
    pub id: SlabId,
    pub section: Option<SectionId>,
}

impl EditCommand for SetSlabSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.slabs.len() || model.slabs[idx].id != self.id {
            return Box::new(Noop);
        }
        if !crate::refs::section_ref_ok(model, self.section) {
            return Box::new(Noop);
        }
        let old = model.slabs[idx].plate.section;
        model.slabs[idx].plate.section = self.section;
        Box::new(SetSlabSection {
            id: self.id,
            section: old,
        })
    }

    fn label(&self) -> &str {
        "床板断面変更"
    }
}
