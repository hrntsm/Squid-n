//! 拘束条件の型。
//!
//! - [`Constraint`] — 剛床・MPC・剛リンクの拘束定義。

use super::*;
use crate::dof::Dof;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Constraint {
    /// 剛床（面内剛体ダイアフラム）。**剛床の唯一の情報源**であり、階
    /// （[`Story`]）は剛床を保持しない（階と剛床は別概念。`model::story` の
    /// モジュールドキュメント参照）。階から剛床を辿るときは
    /// [`Model::diaphragms_of`] を使う。
    ///
    /// この拘束が存在すること自体が「面内剛体である」ことを意味する。
    /// スレーブ節点は階のレベル上にある節点に限る
    /// （[`Model::on_diaphragm_level`]）。
    RigidDiaphragm {
        /// この剛床が属する階。1 つの階が複数の剛床を持つことがある（段差床）。
        story: StoryId,
        master: NodeId,
        slaves: Vec<NodeId>,
        /// この剛床が負担する地震用重量 [N]。多剛床の階では層の水平力 Pi を
        /// 剛床ごとの重量比で分配するために用いる（多剛床の設計用せん断力。
        /// 令88条・昭55建告1793号）。None は未算定（階に単一剛床なら層重量全量）。
        #[serde(default)]
        weight: Option<f64>,
        /// 副剛床の層せん断力係数 Ci の直接入力（令88条・昭55建告1793号の
        /// 層せん断力係数）。Some の剛床は主系統の Ai 分布から除外され、
        /// 水平力 = ci_override × 剛床重量（等価震度扱い。上階に同一系統の
        /// 剛床が積み上がらない副剛床を想定）として作用する。
        /// None は主系統（Ai 分布）。
        #[serde(default)]
        ci_override: Option<f64>,
    },
    Mpc {
        master: NodeId,
        terms: Vec<(NodeId, Dof, f64)>,
    },
    RigidLink {
        master: NodeId,
        slaves: Vec<NodeId>,
        dofs: Dof6Mask,
    },
}

impl Constraint {
    /// 重量・Ci 指定を持たない剛床拘束を作る。
    ///
    /// 階に剛床が 1 つだけの場合、水平力はその剛床へ全量載るため重量比の按分が
    /// 不要になる（[`Model::diaphragms_of`] を用いる分配側の規則）。多剛床の階を
    /// 作るときだけ `weight`・`ci_override` を明示的に設定する。
    pub fn rigid_diaphragm(story: StoryId, master: NodeId, slaves: Vec<NodeId>) -> Self {
        Constraint::RigidDiaphragm {
            story,
            master,
            slaves,
            weight: None,
            ci_override: None,
        }
    }
}
