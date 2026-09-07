//! 節点・部材の編集コマンド。

use super::*;
use squid_n_core::ids::*;

/// 節点座標変更。
pub struct SetNodeCoord {
    pub node: NodeId,
    pub coord: [f64; 3],
}

impl EditCommand for SetNodeCoord {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.node.index();
        if idx >= model.nodes.len() || model.nodes[idx].id != self.node {
            return Box::new(Noop);
        }
        let old_coord = model.nodes[idx].coord;
        model.nodes[idx].coord = self.coord;
        Box::new(SetNodeCoord {
            node: self.node,
            coord: old_coord,
        })
    }

    fn label(&self) -> &str {
        "節点座標変更"
    }
}

/// 節点拘束（支点条件）変更。逆操作は変更前マスクへの復元。
pub struct SetNodeRestraint {
    pub node: NodeId,
    pub restraint: squid_n_core::dof::Dof6Mask,
}

impl EditCommand for SetNodeRestraint {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.node.index();
        if idx >= model.nodes.len() || model.nodes[idx].id != self.node {
            return Box::new(Noop);
        }
        let old = model.nodes[idx].restraint;
        model.nodes[idx].restraint = self.restraint;
        Box::new(SetNodeRestraint {
            node: self.node,
            restraint: old,
        })
    }

    fn label(&self) -> &str {
        "節点拘束変更"
    }
}

/// 節点の支点ばね変更。逆操作は変更前の指定への復元。
///
/// `restraint` で固定されている自由度のばね値は解析側（ソルバー）で無視される
/// （`Node::support_spring` の仕様）。本コマンドは restraint との整合チェックは
/// 行わない（先に固定を解除してからばねを設定する、または逆でもよい）。
/// 負のばね剛性は物理的に無意味なため 0 にクランプする。
pub struct SetNodeSupportSpring {
    pub node: NodeId,
    pub spring: Option<[f64; 6]>,
}

impl EditCommand for SetNodeSupportSpring {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.node.index();
        if idx >= model.nodes.len() || model.nodes[idx].id != self.node {
            return Box::new(Noop);
        }
        let old = model.nodes[idx].support_spring;
        let clamped = self.spring.map(|s| s.map(|v| v.max(0.0)));
        model.nodes[idx].support_spring = clamped;
        Box::new(SetNodeSupportSpring {
            node: self.node,
            spring: old,
        })
    }

    fn label(&self) -> &str {
        "支点ばね変更"
    }
}

/// 節点追加。末尾に `NodeId(len)` で追加する（ID＝配列インデックスの不変条件を維持）。
/// 逆操作は節点削除。
pub struct AddNode {
    pub coord: [f64; 3],
    pub restraint: squid_n_core::dof::Dof6Mask,
}

impl EditCommand for AddNode {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let new_id = NodeId(model.nodes.len() as u32);
        model.nodes.push(squid_n_core::model::Node {
            id: new_id,
            coord: self.coord,
            restraint: self.restraint,
            mass: None,
            story: None,
            support_spring: None,
        });
        Box::new(DeleteNode { id: new_id })
    }

    fn label(&self) -> &str {
        "節点追加"
    }
}

/// 節点削除（末尾以外の中間節点も可）。逆操作は [`InsertNode`]。
/// 削除後は ID を繰り上げる。参照されている節点は Noop とする。
pub struct DeleteNode {
    pub id: NodeId,
}

impl EditCommand for DeleteNode {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.nodes.len() || model.nodes[idx].id != self.id {
            return Box::new(Noop);
        }
        if model.node_in_use(self.id) {
            return Box::new(Noop);
        }
        let generated_master =
            if let Some(pos) = model.generated_masters.iter().position(|n| *n == self.id) {
                model.generated_masters.remove(pos);
                true
            } else {
                false
            };
        let mut axis_membership = Vec::new();
        for (gi, group) in model.axes.iter_mut().enumerate() {
            for (ai, axis) in group.axes.iter_mut().enumerate() {
                if let Some(pos) = axis.nodes.iter().position(|n| *n == self.id) {
                    axis.nodes.remove(pos);
                    axis_membership.push((gi, ai));
                }
            }
        }
        let removed = model.nodes.remove(idx);
        shift_node_ids(model, |id| {
            if id.0 > self.id.0 {
                id.0 -= 1;
            }
        });
        Box::new(InsertNode {
            index: idx,
            coord: removed.coord,
            restraint: removed.restraint,
            mass: removed.mass,
            story: removed.story,
            support_spring: removed.support_spring,
            generated_master,
            axis_membership,
        })
    }

    fn label(&self) -> &str {
        "節点削除"
    }
}

/// 指定インデックスへ節点を再挿入する（[`DeleteNode`] の逆操作専用）。
pub struct InsertNode {
    pub index: usize,
    pub coord: [f64; 3],
    pub restraint: squid_n_core::dof::Dof6Mask,
    pub mass: Option<[f64; 6]>,
    pub story: Option<squid_n_core::ids::StoryId>,
    pub support_spring: Option<[f64; 6]>,
    pub generated_master: bool,
    pub axis_membership: Vec<(usize, usize)>,
}

impl EditCommand for InsertNode {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let id = NodeId(self.index as u32);
        shift_node_ids(model, |nid| {
            if nid.0 >= id.0 {
                nid.0 += 1;
            }
        });
        model.nodes.insert(
            self.index,
            squid_n_core::model::Node {
                id,
                coord: self.coord,
                restraint: self.restraint,
                mass: self.mass,
                story: self.story,
                support_spring: self.support_spring,
            },
        );
        if self.generated_master {
            model.generated_masters.push(id);
            model.generated_masters.sort();
        }
        for &(gi, ai) in &self.axis_membership {
            if let Some(axis) = model.axes.get_mut(gi).and_then(|g| g.axes.get_mut(ai)) {
                let pos = axis.nodes.partition_point(|n| *n < id);
                axis.nodes.insert(pos, id);
            }
        }
        Box::new(DeleteNode { id })
    }

    fn label(&self) -> &str {
        "節点削除の取り消し"
    }
}

/// モデル内の全ての `NodeId` 参照（節点自身の ID を含む）に `f` を適用する。
/// [`DeleteNode`]／[`InsertNode`] の ID 繰り上げ・繰り下げで共用する。
/// 走査そのものはフィールド定義と同じ core 側（[`Model::visit_node_ids`]）が
/// 単一情報源として持つ（新フィールド追加時の追随漏れを防ぐ）。
fn shift_node_ids(model: &mut Model, f: impl FnMut(&mut NodeId)) {
    model.visit_node_ids(f);
}

/// 部材追加。逆操作は部材削除。
///
/// `elem.id` は `ElemId(model.elements.len())`（末尾の次の添字）と一致し、参照する
/// 節点・断面が実在していること（crate::refs の規約）。満たさない場合は `Noop`。
pub struct AddMember {
    pub elem: squid_n_core::model::ElementData,
}

impl EditCommand for AddMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if !crate::refs::new_elem_ok(model, &self.elem) {
            return Box::new(Noop);
        }
        model.elements.push(self.elem.clone());
        Box::new(DeleteMember { id: self.elem.id })
    }

    fn label(&self) -> &str {
        "部材追加"
    }
}

/// モデル末尾の部材を除去する（部材を末尾へ追加するコマンドの逆操作）。
/// `elems` の件数分だけ末尾から取り除く（生成直後の undo を想定し、末尾＝生成分）。
/// 逆操作は [`PushTailMembers`]（同じ部材の末尾再追加）。
pub struct PopTailMembers {
    pub elems: Vec<squid_n_core::model::ElementData>,
}

impl EditCommand for PopTailMembers {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let k = self.elems.len();
        let start = model.elements.len().saturating_sub(k);
        let removed: Vec<_> = model.elements.split_off(start);
        Box::new(PushTailMembers { elems: removed })
    }

    fn label(&self) -> &str {
        "実部材化の取り消し"
    }
}

/// モデル末尾へ部材を再追加する（[`PopTailMembers`] の逆操作）。
pub struct PushTailMembers {
    pub elems: Vec<squid_n_core::model::ElementData>,
}

impl EditCommand for PushTailMembers {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        for e in &self.elems {
            model.elements.push(e.clone());
        }
        Box::new(PopTailMembers {
            elems: self.elems.clone(),
        })
    }

    fn label(&self) -> &str {
        "実部材化の再適用"
    }
}

/// 制振ダンパー要素の追加（制振部材の力学モデル: Maxwell モデル等）。
/// 要素（`ElementKind::Damper`）と特性（`Model::damper_attrs`）を原子的に追加する。
/// 逆操作は部材削除（`DeleteMember` が側テーブル属性も退避・復元する）。
///
/// `elem` の ID・節点・断面の要件は [`AddMember`] と同じ（crate::refs の規約）。
pub struct AddDamper {
    pub elem: squid_n_core::model::ElementData,
    pub props: squid_n_core::model::DamperProps,
}

impl EditCommand for AddDamper {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if !crate::refs::new_elem_ok(model, &self.elem) {
            return Box::new(Noop);
        }
        let id = self.elem.id;
        model.elements.push(self.elem.clone());
        model.set_damper_props(id, Some(self.props));
        Box::new(DeleteMember { id })
    }

    fn label(&self) -> &str {
        "制振ダンパー追加"
    }
}

/// 免震支承材要素の追加（各免震部材指針）。
/// 要素（`ElementKind::Isolator`）と特性（`Model::isolator_attrs`）を原子的に追加する。
/// 逆操作は部材削除（`DeleteMember` が側テーブル属性も退避・復元する）。
///
/// `elem` の ID・節点・断面の要件は [`AddMember`] と同じ（crate::refs の規約）。
pub struct AddIsolator {
    pub elem: squid_n_core::model::ElementData,
    pub props: squid_n_core::model::IsolatorProps,
}

impl EditCommand for AddIsolator {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if !crate::refs::new_elem_ok(model, &self.elem) {
            return Box::new(Noop);
        }
        let id = self.elem.id;
        model.elements.push(self.elem.clone());
        model
            .isolator_attrs
            .push(squid_n_core::model::IsolatorAttr {
                elem: id,
                props: self.props,
            });
        Box::new(DeleteMember { id })
    }

    fn label(&self) -> &str {
        "免震支承材追加"
    }
}

/// 支点への免震装置の設置（既存の運用: 基礎節点↔上部節点間に零長 Isolator 要素）。
///
/// 対象節点 `node` と同一座標に接地節点（`restraint=FIXED`）を新規作成し、
/// その2節点間に零長 [`ElementKind::Isolator`](squid_n_core::model::ElementKind::Isolator)
/// 要素＋ [`IsolatorAttr`](squid_n_core::model::IsolatorAttr) を追加した上で、対象節点
/// 自身の `restraint` を `FREE` に変更する（免震装置を介して支持されるため、
/// 対象節点はもはや直接の固定支点ではない）。
///
/// 要素の節点順は `[接地節点, 対象節点]`（i端=接地/下端、j端=対象/上端）とする。
/// `element/src/springs/isolator.rs` の零長特例（2節点が同一座標の場合、局所 x 軸＝
/// 全体座標系の鉛直方向、節点0→節点1 の向き）に整合する。
///
/// 逆操作（[`UndoPlaceSupportIsolator`]）は生成した接地節点・Isolator 要素（＋属性）を
/// 削除し、対象節点の `restraint` を元へ戻す。要素削除を節点削除より先に行う
/// （`node_in_use` は要素が参照している間、節点の削除を拒否するため）。
pub struct PlaceSupportIsolator {
    pub node: NodeId,
    pub props: squid_n_core::model::IsolatorProps,
}

impl EditCommand for PlaceSupportIsolator {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.node.index();
        if idx >= model.nodes.len() || model.nodes[idx].id != self.node {
            return Box::new(Noop);
        }
        let coord = model.nodes[idx].coord;
        let old_restraint = model.nodes[idx].restraint;

        let ground_id = NodeId(model.nodes.len() as u32);
        model.nodes.push(squid_n_core::model::Node {
            id: ground_id,
            coord,
            restraint: squid_n_core::dof::Dof6Mask::FIXED,
            mass: None,
            story: None,
            support_spring: None,
        });

        let elem_id = ElemId(model.elements.len() as u32);
        model.elements.push(squid_n_core::model::ElementData {
            id: elem_id,
            kind: squid_n_core::model::ElementKind::Isolator,
            nodes: [ground_id, self.node].into_iter().collect(),
            section: None,
            local_axis: squid_n_core::model::LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [
                squid_n_core::model::EndCondition::Fixed,
                squid_n_core::model::EndCondition::Fixed,
            ],
            force_regime: squid_n_core::model::ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
        model
            .isolator_attrs
            .push(squid_n_core::model::IsolatorAttr {
                elem: elem_id,
                props: self.props,
            });

        model.nodes[idx].restraint = squid_n_core::dof::Dof6Mask::FREE;

        Box::new(UndoPlaceSupportIsolator {
            node: self.node,
            props: self.props,
            old_restraint,
            ground_node: ground_id,
            elem: elem_id,
        })
    }

    fn label(&self) -> &str {
        "支点免震装置の設置"
    }
}

/// [`PlaceSupportIsolator`] の逆操作。
pub struct UndoPlaceSupportIsolator {
    node: NodeId,
    props: squid_n_core::model::IsolatorProps,
    old_restraint: squid_n_core::dof::Dof6Mask,
    ground_node: NodeId,
    elem: ElemId,
}

impl EditCommand for UndoPlaceSupportIsolator {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.node.index();
        if idx >= model.nodes.len() || model.nodes[idx].id != self.node {
            return Box::new(Noop);
        }
        model.nodes[idx].restraint = self.old_restraint;
        DeleteMember { id: self.elem }.apply(model);
        DeleteNode {
            id: self.ground_node,
        }
        .apply(model);
        Box::new(PlaceSupportIsolator {
            node: self.node,
            props: self.props,
        })
    }

    fn label(&self) -> &str {
        "支点免震装置の設置の取り消し"
    }
}

/// [`PlaceSupportIsolator`] で配置した支点免震要素の撤去（単体削除）。
/// 撤去後の拘束は常に `FIXED` に統一する（設置前の拘束は復元しない）。
pub struct RemoveSupportIsolator {
    pub node: NodeId,
}

impl EditCommand for RemoveSupportIsolator {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let found = model.elements.iter().find_map(|e| {
            model
                .support_isolator_ends(e.id)
                .filter(|(upper, _)| *upper == self.node)
                .map(|(_, ground)| (e.id, ground))
        });
        let Some((elem_id, ground)) = found else {
            return Box::new(Noop);
        };
        let idx = self.node.index();
        if idx >= model.nodes.len() || model.nodes[idx].id != self.node {
            return Box::new(Noop);
        }
        let old_restraint = model.nodes[idx].restraint;

        let undo_member = DeleteMember { id: elem_id }.apply(model);
        let undo_node = DeleteNode { id: ground }.apply(model);

        let idx = self.node.index();
        if idx < model.nodes.len() && model.nodes[idx].id == self.node {
            model.nodes[idx].restraint = squid_n_core::dof::Dof6Mask::FIXED;
        }

        Box::new(UndoRemoveSupportIsolator {
            node: self.node,
            old_restraint,
            undo_node,
            undo_member,
        })
    }

    fn label(&self) -> &str {
        "支点免震装置の撤去"
    }
}

/// [`RemoveSupportIsolator`] の逆操作。
struct UndoRemoveSupportIsolator {
    node: NodeId,
    old_restraint: squid_n_core::dof::Dof6Mask,
    undo_node: Box<dyn EditCommand>,
    undo_member: Box<dyn EditCommand>,
}

impl EditCommand for UndoRemoveSupportIsolator {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        self.undo_node.apply(model);
        self.undo_member.apply(model);
        let idx = self.node.index();
        if idx < model.nodes.len() && model.nodes[idx].id == self.node {
            model.nodes[idx].restraint = self.old_restraint;
        }
        Box::new(RemoveSupportIsolator { node: self.node })
    }

    fn label(&self) -> &str {
        "支点免震装置の撤去の取り消し"
    }
}

/// 部材削除（中間の部材も可）。逆操作は [`InsertMember`]。
pub struct DeleteMember {
    pub id: ElemId,
}

impl EditCommand for DeleteMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.elements.len() || model.elements[idx].id != self.id {
            return Box::new(Noop);
        }
        let mut removed_loads = Vec::new();
        for (lci, lc) in model.load_cases.iter_mut().enumerate() {
            let mut li = 0;
            while li < lc.member.len() {
                if lc.member[li].elem == self.id {
                    removed_loads.push((lci, li, lc.member.remove(li)));
                } else {
                    li += 1;
                }
            }
        }
        let removed_attrs = model.take_elem_attrs(self.id);
        let mut removed_group_refs = Vec::new();
        for (gi, group) in model.beam_groups.iter_mut().enumerate() {
            let mut pos = 0;
            while pos < group.len() {
                if group[pos] == self.id {
                    group.remove(pos);
                    removed_group_refs.push((gi, pos));
                } else {
                    pos += 1;
                }
            }
        }
        let removed = model.elements.remove(idx);
        shift_elem_ids(model, |id| {
            if id.0 > self.id.0 {
                id.0 -= 1;
            }
        });
        Box::new(InsertMember {
            index: idx,
            elem: removed,
            member_loads: removed_loads,
            elem_attrs: removed_attrs,
            beam_group_refs: removed_group_refs,
        })
    }

    fn label(&self) -> &str {
        "部材削除"
    }
}

/// 指定インデックスへ部材を再挿入する（[`DeleteMember`] の逆操作専用）。
pub struct InsertMember {
    pub index: usize,
    pub elem: squid_n_core::model::ElementData,
    /// (荷重ケース index, 荷重 index, 内容)
    pub member_loads: Vec<(usize, usize, squid_n_core::model::MemberLoad)>,
    /// 削除時に退避した側テーブル属性。
    pub elem_attrs: squid_n_core::model::ElemAttrs,
    /// 削除時に一本部材指定から外した参照の (グループ index, グループ内位置)。
    pub beam_group_refs: Vec<(usize, usize)>,
}

impl EditCommand for InsertMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.elements.len() {
            return Box::new(Noop);
        }
        let id = ElemId(self.index as u32);
        shift_elem_ids(model, |eid| {
            if eid.0 >= id.0 {
                eid.0 += 1;
            }
        });
        let mut elem = self.elem.clone();
        elem.id = id;
        model.elements.insert(self.index, elem);
        for (lci, li, load) in self.member_loads.iter().rev() {
            if let Some(lc) = model.load_cases.get_mut(*lci) {
                let pos = (*li).min(lc.member.len());
                lc.member.insert(pos, load.clone());
            }
        }
        model.restore_elem_attrs(id, self.elem_attrs.clone());
        for &(gi, pos) in self.beam_group_refs.iter().rev() {
            if let Some(group) = model.beam_groups.get_mut(gi) {
                group.insert(pos.min(group.len()), id);
            }
        }
        Box::new(DeleteMember { id })
    }

    fn label(&self) -> &str {
        "部材削除の取り消し"
    }
}

/// モデル内の全ての `ElemId` 参照に `f` を適用する。
fn shift_elem_ids(model: &mut Model, f: impl FnMut(&mut ElemId)) {
    model.visit_elem_ids(f);
}

/// 何もしないコマンド。
pub struct Noop;

impl EditCommand for Noop {
    fn apply(&self, _model: &mut Model) -> Box<dyn EditCommand> {
        Box::new(Noop)
    }

    fn label(&self) -> &str {
        "Noop"
    }

    fn is_noop(&self) -> bool {
        true
    }
}
