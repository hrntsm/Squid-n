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

/// 節点削除（末尾以外の中間節点も可）。逆操作は [`InsertNode`]（元の位置に再挿入し、
/// 繰り上がった ID・参照を元に戻す）。
///
/// ID＝配列インデックスの不変条件を保つため、削除後は当該節点より後ろの
/// 節点 ID と、それを参照する全ての箇所（部材・節点荷重・階・床・拘束）を
/// 1 つずつ繰り上げる。部材などからまだ参照されている節点は削除すると
/// 参照が壊れるため Noop とする（先に参照を解消する必要がある）。
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
        // 剛床代表節点かどうかを退避し、リストからは先に除去してから ID を繰り上げる。
        let generated_master =
            if let Some(pos) = model.generated_masters.iter().position(|n| *n == self.id) {
                model.generated_masters.remove(pos);
                true
            } else {
                false
            };
        // 通り芯の所属は退避してから外す。通り芯は計算に用いない呼称であり、
        // `node_in_use` には数えない（＝節点削除を妨げない）ため、ここで参照を
        // 解消しないと `validate` の DanglingRef になる。
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

/// 指定インデックスへ節点を再挿入し、以降の節点 ID・参照を 1 つ繰り下げる
/// （[`DeleteNode`] の逆操作専用。新規追加は [`AddNode`] を使うこと）。
pub struct InsertNode {
    pub index: usize,
    pub coord: [f64; 3],
    pub restraint: squid_n_core::dof::Dof6Mask,
    pub mass: Option<[f64; 6]>,
    pub story: Option<squid_n_core::ids::StoryId>,
    /// 支点ばね（[`DeleteNode`] で退避した値。省略時は `None`）。
    pub support_spring: Option<[f64; 6]>,
    /// 削除された節点が `generated_masters`（剛床代表節点）に含まれていたか。
    /// 含まれていた場合、再挿入後の ID を `generated_masters` へ戻す。
    pub generated_master: bool,
    /// 削除された節点が属していた通り芯の位置（`model.axes` のグループ添字, 通り添字）。
    /// 再挿入後、同じ通りへ所属を戻す。
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
        // 通り芯の所属を戻す（所属リストは節点 ID 昇順に保つ）。
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

/// スラブの小梁（`JoistLine`）を実部材化する。各小梁について支持2節点を両端に持つ
/// 実 `Beam` 要素が未生成なら新規に生成（末尾に追加。断面未割当・両端ピン）する。
/// 実部材化された小梁には床分配が点反力ではなく等分布荷重を載せる（分配エンジンが
/// 実部材の有無で自動的に切り替える）。これにより小梁が応力解析に参加し、断面検定・
/// たわみ検定の対象となる。逆操作は生成した部材の末尾からの除去
/// （[`PopTailMembers`]。生成直後の undo のため末尾＝生成分）。
pub struct MaterializeSlabJoists {
    pub slab: FloorRegionId,
}

impl EditCommand for MaterializeSlabJoists {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        use squid_n_core::model::{ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis};
        let Some(slab) = model
            .floor_regions
            .get(self.slab.index())
            .filter(|s| s.id == self.slab)
        else {
            return Box::new(Noop);
        };
        // 支持節点対は借用を切るため先に複製する。
        let supports: Vec<[NodeId; 2]> = slab.joist_lines().iter().map(|j| j.support).collect();

        let beam_exists = |model: &Model, created: &[ElementData], a: NodeId, b: NodeId| -> bool {
            model.elements.iter().chain(created.iter()).any(|e| {
                e.kind == ElementKind::Beam
                    && e.nodes.len() == 2
                    && ((e.nodes[0] == a && e.nodes[1] == b)
                        || (e.nodes[0] == b && e.nodes[1] == a))
            })
        };

        let mut created: Vec<ElementData> = Vec::new();
        let mut next_id = model.elements.len() as u32;
        for sp in supports {
            let (a, b) = (sp[0], sp[1]);
            if a == b || beam_exists(model, &created, a, b) {
                continue;
            }
            created.push(ElementData {
                id: ElemId(next_id),
                kind: ElementKind::Beam,
                nodes: [a, b].into_iter().collect(),
                section: None,
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                // 小梁は大梁へピン接合（単純梁）とみなす。
                end_cond: [EndCondition::Pinned, EndCondition::Pinned],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
            next_id += 1;
        }
        for e in &created {
            model.elements.push(e.clone());
        }
        Box::new(PopTailMembers { elems: created })
    }

    fn label(&self) -> &str {
        "小梁の実部材化"
    }
}

/// モデル末尾の部材を除去する（[`MaterializeSlabJoists`] 等の逆操作）。
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

        // 1) 接地節点を末尾に追加（restraint=FIXED、対象節点と同一座標＝零長要素）。
        let ground_id = NodeId(model.nodes.len() as u32);
        model.nodes.push(squid_n_core::model::Node {
            id: ground_id,
            coord,
            restraint: squid_n_core::dof::Dof6Mask::FIXED,
            mass: None,
            story: None,
            support_spring: None,
        });

        // 2) 零長 Isolator 要素を末尾に追加（i端=接地節点、j端=対象節点）。
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

        // 3) 対象節点は免震装置を介して支持されるため、restraint を解放する。
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
        // 要素（Isolator＋属性）を先に削除してから接地節点を削除する
        // （node_in_use は要素が参照している間、節点の削除を拒否するため）。
        DeleteMember { id: self.elem }.apply(model);
        DeleteNode {
            id: self.ground_node,
        }
        .apply(model);
        // redo は同一パラメータで PlaceSupportIsolator を再適用する。LIFO の
        // undo/redo 前提の下、生成される接地節点・要素 ID は元と同じ値に戻る
        // （直前に削除した分だけ model.nodes/elements の末尾が縮んでいるため）。
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
///
/// [`Model::support_isolator_ends`] で対象節点 `node` に接続する支点免震要素
/// （零長 `Isolator` 要素・他端が孤立した `restraint=FIXED` の接地節点）を特定し、
/// その要素（＋免震特性）と接地節点を削除した上で、対象節点の `restraint` を
/// `FIXED` へ戻す複合コマンド。対象が支点免震要素の形を満たさない場合は Noop
/// （通常の〔支点ではない〕免震要素は [`DeleteMember`] を使うこと）。
///
/// **配置前の拘束は復元しない仕様**: `PlaceSupportIsolator` は設置前の `restraint`
/// を記録していないため（対象節点の元の拘束を覚えていない）、本コマンドは
/// 撤去後の拘束を常に `FIXED` に統一する。設置前がピン支点等だった場合でも
/// `FIXED` に戻る点に注意（必要なら撤去後に境界条件パネルで再設定する）。
///
/// 逆操作（undo）は接地節点・要素を元の位置・ID・特性で完全に復元し、対象節点の
/// 拘束も本コマンド実行直前の値（＝撤去前の値。通常は `PlaceSupportIsolator` が
/// 解放した `FREE`）へ戻す。
pub struct RemoveSupportIsolator {
    pub node: NodeId,
}

impl EditCommand for RemoveSupportIsolator {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        // 対象節点が「上部節点」側になっている支点免震要素を探す
        // （接地節点＝FIXED側を選んだ場合はヒットしない。support_isolator_ends 参照）。
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

        // 要素（Isolator＋属性）を先に削除してから接地節点を削除する
        // （node_in_use は要素が参照している間、節点の削除を拒否するため。
        // PlaceSupportIsolator の逆操作 UndoPlaceSupportIsolator と同じ順序）。
        let undo_member = DeleteMember { id: elem_id }.apply(model);
        let undo_node = DeleteNode { id: ground }.apply(model);

        // 対象節点は撤去後、免震装置を介さない直接支点になる。
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
    /// 接地節点の再挿入（[`DeleteNode`] が返した `InsertNode`）。
    undo_node: Box<dyn EditCommand>,
    /// 免震要素（＋属性）の再挿入（[`DeleteMember`] が返した `InsertMember`）。
    undo_member: Box<dyn EditCommand>,
}

impl EditCommand for UndoRemoveSupportIsolator {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        // 復元は削除と逆順: 先に接地節点を再挿入し、次に免震要素を再挿入する
        // （PlaceSupportIsolator と同じ生成順）。再挿入は元の位置・ID を復元するため、
        // 再挿入後は本コマンド実行前と同一の状態に戻る＝RemoveSupportIsolator を
        // 再実行すれば同じ結果になる（redo はそれをそのまま使う）。
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
///
/// ID＝配列インデックスの不変条件を保つため、削除後は当該部材より後ろの
/// 部材 ID と、それを参照する部材荷重の `elem` を 1 つずつ繰り上げる。
/// 当該部材を参照する部材荷重は連動して削除し、undo で復元する。
pub struct DeleteMember {
    pub id: ElemId,
}

impl EditCommand for DeleteMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.elements.len() || model.elements[idx].id != self.id {
            return Box::new(Noop);
        }
        // 当該部材を参照する部材荷重を (荷重ケース index, 荷重 index, 内容) で退避してから削除
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
        // 側テーブル属性（履歴則・ダンパー・免震等）を退避してから削除（残余は shift で繰上げ）。
        let removed_attrs = model.take_elem_attrs(self.id);
        // 一本部材指定（beam_groups）から当該部材を連動削除する（残すと shift 後に
        // 別部材を指し、検定の採用応力が無関係な部材と合成される）。undo 用に
        // (グループ index, グループ内位置) を退避する。
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
        // 当該部材を壁版に持つ壁領域を「版なし」へ戻す（`beam_groups` と同じ理由で、
        // 残すと繰り上げ後に別の要素を指す。隣が壁だと検証も通ってしまい、
        // 壁領域が黙って別の壁に付け替わる）。undo 用に壁領域の添字を退避する。
        let mut removed_wall_region_refs = Vec::new();
        for (ri, region) in model.wall_regions.iter_mut().enumerate() {
            if region.wall == Some(self.id) {
                region.wall = None;
                removed_wall_region_refs.push(ri);
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
            wall_region_refs: removed_wall_region_refs,
        })
    }

    fn label(&self) -> &str {
        "部材削除"
    }
}

/// 指定インデックスへ部材を再挿入し、以降の部材 ID・参照を 1 つ繰り下げ、
/// 連動削除された部材荷重を復元する（[`DeleteMember`] の逆操作専用）。
pub struct InsertMember {
    pub index: usize,
    pub elem: squid_n_core::model::ElementData,
    /// (荷重ケース index, 荷重 index, 内容)
    pub member_loads: Vec<(usize, usize, squid_n_core::model::MemberLoad)>,
    /// 削除時に退避した側テーブル属性（履歴則・ダンパー・免震等）。
    pub elem_attrs: squid_n_core::model::ElemAttrs,
    /// 削除時に一本部材指定（beam_groups）から外した参照の
    /// (グループ index, グループ内位置)。undo で同じ位置へ復元する。
    pub beam_group_refs: Vec<(usize, usize)>,
    /// 削除時に「版なし」へ戻した壁領域の添字。undo で壁版の参照を復元する。
    pub wall_region_refs: Vec<usize>,
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
        // 部材荷重は削除時に「縮んでいく配列でのインデックス」を昇順で記録している。
        // 正しく復元するには逆順（最後に削除したものから）で挿入する必要がある。
        // 従来は前方順に挿入しており、同一部材を参照する複数荷重の順序が入れ替わり、
        // undo が削除前の状態を正確に復元できていなかった。
        for (lci, li, load) in self.member_loads.iter().rev() {
            if let Some(lc) = model.load_cases.get_mut(*lci) {
                let pos = (*li).min(lc.member.len());
                lc.member.insert(pos, load.clone());
            }
        }
        // 退避した側テーブル属性を再挿入 ID へ紐づけ直して復元。
        model.restore_elem_attrs(id, self.elem_attrs.clone());
        // 一本部材指定（beam_groups）から外した参照を元の位置へ復元する。
        // 削除時は「縮んでいく配列での位置」を昇順で記録しているため、
        // 部材荷重と同様に逆順で挿入すると削除前の並びに戻る。
        for &(gi, pos) in self.beam_group_refs.iter().rev() {
            if let Some(group) = model.beam_groups.get_mut(gi) {
                group.insert(pos.min(group.len()), id);
            }
        }
        // 「版なし」へ戻した壁領域へ、再挿入した ID で壁版を復元する。
        for &ri in &self.wall_region_refs {
            if let Some(region) = model.wall_regions.get_mut(ri) {
                region.wall = Some(id);
            }
        }
        Box::new(DeleteMember { id })
    }

    fn label(&self) -> &str {
        "部材削除の取り消し"
    }
}

/// モデル内の全ての `ElemId` 参照（部材自身の ID・部材荷重・要素キー付き側テーブル・
/// 一本部材指定）に `f` を適用する。要素の削除・挿入に伴う ID 繰上げ／繰下げで
/// 参照整合を保つ。走査は core 側（[`Model::visit_elem_ids`]）が単一情報源として持つ。
fn shift_elem_ids(model: &mut Model, f: impl FnMut(&mut ElemId)) {
    model.visit_elem_ids(f);
}

/// 何もしないコマンド（参照不正時の安全なフォールバック）。
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
