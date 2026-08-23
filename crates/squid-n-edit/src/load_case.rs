//! 荷重ケース・荷重組み合わせ・階・スラブの編集コマンド。

use super::*;
use squid_n_core::ids::*;

/// 荷重ケース追加。末尾に `LoadCaseId(len)` で追加する。逆操作は荷重ケース削除。
pub struct AddLoadCase {
    pub name: String,
}

impl EditCommand for AddLoadCase {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let new_id = LoadCaseId(model.load_cases.len() as u32);
        model.load_cases.push(squid_n_core::model::LoadCase {
            kind: Default::default(),
            id: new_id,
            name: self.name.clone(),
            nodal: Vec::new(),
            member: Vec::new(),
        });
        Box::new(DeleteLoadCase { id: new_id })
    }

    fn label(&self) -> &str {
        "荷重ケース追加"
    }
}

id_indexed_delete_insert!(
    /// 荷重ケース削除（中身の節点荷重・部材荷重ごと削除し、undo で復元する）。
    /// 荷重組合せから参照中のケースは Noop。
    /// ID＝配列インデックスの不変条件を保つため、後続のケース ID と組合せからの参照を繰り上げる。
    DeleteLoadCase,
    /// 指定インデックスへ荷重ケースを再挿入する（[`DeleteLoadCase`] の逆操作専用）。
    InsertLoadCase,
    id = LoadCaseId,
    entity = squid_n_core::model::LoadCase,
    vec = load_cases,
    shift = shift_load_case_ids,
    guard = load_case_in_use,
    del_label = "荷重ケース削除",
    ins_label = "荷重ケース削除の取り消し",
);

/// 指定荷重ケースを参照している荷重組合せが存在するか（削除ガード用）。
fn load_case_in_use(model: &Model, id: LoadCaseId) -> bool {
    model
        .combinations
        .iter()
        .any(|c| c.terms.iter().any(|(lc, _)| *lc == id))
}

/// モデル内の全ての `LoadCaseId` 参照（ケース自身の ID を含む）に `f` を適用する。
fn shift_load_case_ids(model: &mut Model, mut f: impl FnMut(&mut LoadCaseId)) {
    for lc in &mut model.load_cases {
        f(&mut lc.id);
    }
    for combo in &mut model.combinations {
        for (lcid, _) in &mut combo.terms {
            f(lcid);
        }
    }
}

/// 荷重組合せ追加。末尾に追加する。逆操作は末尾の組合せ削除。
///
/// `LoadCombination` は ID を持たず配列インデックスのみで管理されるため、
/// 他の追加系コマンド（[`AddLoadCase`] 等）と異なり ID 採番は発生しない。
/// 参照する `LoadCaseId` の存在チェックは行わない（[`Model::validate`] も
/// 組合せの `LoadCaseId` 参照はダングリングチェックの対象外であり、既存の
/// [`DeleteLoadCase`] が参照側で削除を防ぐことで整合性を保っている）。
pub struct AddCombination {
    pub combo: squid_n_core::model::LoadCombination,
}

impl EditCommand for AddCombination {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        model.combinations.push(self.combo.clone());
        let index = model.combinations.len() - 1;
        Box::new(DeleteCombination { index })
    }

    fn label(&self) -> &str {
        "荷重組合せ追加"
    }
}

indexed_delete_insert!(
    /// 荷重組合せを index 指定で削除。逆操作は [`InsertCombination`]（同じ位置への復元）。
    /// 組合せは他のデータから参照されないため ID 再採番は不要。index が範囲外なら Noop。
    DeleteCombination,
    /// 指定インデックスへ荷重組合せを再挿入する（[`DeleteCombination`] の逆操作専用）。
    InsertCombination,
    entity = squid_n_core::model::LoadCombination,
    vec = combinations,
    field = combo,
    del_label = "荷重組合せ削除",
    ins_label = "荷重組合せ削除の取り消し",
);

/// 階定義の一括適用（階自動生成の結果を反映する）。
///
/// `model.stories`・各節点の所属階・剛床拘束(`Constraint::RigidDiaphragm`)を
/// まとめて差し替える。既存の RigidDiaphragm 拘束は除去し、Mpc / RigidLink は
/// 保持する。逆操作は差し替え前の状態の復元。
pub struct ApplyStories {
    pub stories: Vec<squid_n_core::model::Story>,
    /// `model.nodes` と同順の所属階。長さが合わない分は無視する。
    pub node_story: Vec<Option<squid_n_core::ids::StoryId>>,
    /// 追加する剛床拘束（既存の RigidDiaphragm と置換）。
    pub constraints: Vec<squid_n_core::model::Constraint>,
    /// 剛床代表節点。ID が既存範囲内なら置換（再利用）、範囲外（＝末尾連番）なら追加。
    pub rep_nodes: Vec<squid_n_core::model::Node>,
    /// 適用後の `model.generated_masters` の全量。
    pub generated_masters: Vec<NodeId>,
    /// 適用する動的解析の質量方式（[`squid_n_core::model::MassMethod`]）。
    /// `rep_nodes` の質点質量はこの方式で算定済みの前提（呼び出し側の責務）。
    pub mass_method: squid_n_core::model::MassMethod,
}

impl EditCommand for ApplyStories {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        use squid_n_core::model::Constraint;
        // 変更前の全量スナップショット（rep_nodes の置換/追加も含めて丸ごと復元できるようにする）。
        let old_nodes = model.nodes.clone();
        let old_generated_masters = model.generated_masters.clone();
        let old_mass_method = model.mass_method;

        let old_stories = std::mem::replace(&mut model.stories, self.stories.clone());
        for (node, st) in model.nodes.iter_mut().zip(self.node_story.iter()) {
            node.story = *st;
        }
        // RigidDiaphragm のみ差し替え、それ以外の拘束は保持
        let old_constraints = model.constraints.clone();
        model
            .constraints
            .retain(|c| !matches!(c, Constraint::RigidDiaphragm { .. }));
        model.constraints.extend(self.constraints.iter().cloned());

        // 剛床代表節点：ID＝配列インデックス不変条件を保って置換 or 追加する。
        for rn in &self.rep_nodes {
            let idx = rn.id.index();
            if idx < model.nodes.len() {
                model.nodes[idx] = rn.clone();
            } else {
                debug_assert_eq!(idx, model.nodes.len(), "rep_nodes は昇順の連番である前提");
                model.nodes.push(rn.clone());
            }
        }
        model.generated_masters = self.generated_masters.clone();
        model.mass_method = self.mass_method;

        Box::new(RestoreStories {
            stories: old_stories,
            nodes: old_nodes,
            constraints: old_constraints,
            generated_masters: old_generated_masters,
            mass_method: old_mass_method,
        })
    }

    fn label(&self) -> &str {
        "階定義の適用"
    }
}

/// [`ApplyStories`] の逆操作。`model.nodes` を丸ごと復元することで、
/// 追加された剛床代表節点の除去（truncate）や既存節点の置換をまとめて元に戻す。
pub struct RestoreStories {
    pub stories: Vec<squid_n_core::model::Story>,
    pub nodes: Vec<squid_n_core::model::Node>,
    pub constraints: Vec<squid_n_core::model::Constraint>,
    pub generated_masters: Vec<NodeId>,
    pub mass_method: squid_n_core::model::MassMethod,
}

impl EditCommand for RestoreStories {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let new_stories = std::mem::replace(&mut model.stories, self.stories.clone());
        let new_nodes = std::mem::replace(&mut model.nodes, self.nodes.clone());
        let new_constraints = std::mem::replace(&mut model.constraints, self.constraints.clone());
        let new_generated_masters =
            std::mem::replace(&mut model.generated_masters, self.generated_masters.clone());
        let new_mass_method = std::mem::replace(&mut model.mass_method, self.mass_method);
        Box::new(RestoreStories {
            stories: new_stories,
            nodes: new_nodes,
            constraints: new_constraints,
            generated_masters: new_generated_masters,
            mass_method: new_mass_method,
        })
    }

    fn label(&self) -> &str {
        "階定義の復元"
    }
}

/// 床領域の追加。末尾に `FloorRegionId(len)` で追加する（ID＝配列インデックスの不変条件を維持）。
/// 逆操作は床領域の削除。
///
/// ここで作るのは**囲まれた領域**（大梁が囲むパネル）である。取り付き領域
/// （片持ち・バルコニー・出隅）の追加コマンドは、UI の入力経路とあわせて別途用意する。
pub struct AddSlab {
    pub boundary: Vec<NodeId>,
    pub joists: Vec<squid_n_core::model::JoistLine>,
    pub loads: Vec<squid_n_core::model::AreaLoad>,
    pub method: squid_n_core::model::DistributionMethod,
    /// 室用途（積載荷重プリセット。`None` は積載寄与なし）。
    pub usage: Option<squid_n_core::model::SlabUsage>,
    /// スラブ断面（板厚・コンクリート材料を持つ断面）。`None` は未割当。
    pub section: Option<SectionId>,
}

impl EditCommand for AddSlab {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        // 境界・小梁が実在しない節点や断面を指す床は作らない（crate::refs の規約）。
        if !self
            .boundary
            .iter()
            .all(|&n| crate::refs::node_exists(model, n))
            || !crate::refs::joists_ok(model, &self.joists)
            || !crate::refs::section_ref_ok(model, self.section)
        {
            return Box::new(Noop);
        }
        let new_id = FloorRegionId(model.floor_regions.len() as u32);
        model.floor_regions.push(
            squid_n_core::model::FloorRegion::enclosed(new_id, self.boundary.clone()).with_plate(
                squid_n_core::model::SlabPlate {
                    section: self.section,
                    loads: self.loads.clone(),
                    usage: self.usage,
                    method: self.method,
                    one_way: None,
                    joists: self.joists.clone(),
                },
            ),
        );
        Box::new(DeleteSlab { id: new_id })
    }

    fn label(&self) -> &str {
        "床追加"
    }
}

id_indexed_delete_insert!(
    /// 床領域の削除（中間の領域も可）。逆操作は [`InsertSlab`]。
    ///
    /// ID＝配列インデックスの不変条件を保つため、削除後は当該領域より後ろの
    /// ID を 1 つずつ繰り上げる。`FloorRegionId` は領域自身の ID 以外からは
    /// 参照されない（`crates` 全体で grep 済み）ため、他データへの追従は不要。
    DeleteSlab,
    /// 指定インデックスへ床領域を再挿入する（[`DeleteSlab`] の逆操作専用）。
    InsertSlab,
    id = FloorRegionId,
    entity = squid_n_core::model::FloorRegion,
    vec = floor_regions,
    shift = shift_slab_ids,
    guard = |_: &Model, _| false,
    del_label = "床削除",
    ins_label = "床削除の取り消し",
);

/// モデル内の全ての `FloorRegionId` 参照（領域自身の ID を含む）に `f` を適用する。
fn shift_slab_ids(model: &mut Model, mut f: impl FnMut(&mut FloorRegionId)) {
    for slab in &mut model.floor_regions {
        f(&mut slab.id);
    }
}
