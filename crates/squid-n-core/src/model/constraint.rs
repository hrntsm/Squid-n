//! 拘束条件の型。

use super::*;
use crate::dof::Dof;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Constraint {
    /// 剛床（面内剛体ダイアフラム）。スレーブ節点は階のレベル上にある節点に限る
    /// （[`Model::on_diaphragm_level`]）。
    RigidDiaphragm {
        /// この剛床が属する階。1 つの階が複数の剛床を持つことがある（段差床）。
        story: StoryId,
        master: NodeId,
        slaves: Vec<NodeId>,
        /// この剛床が負担する地震用重量 [N]。None は未算定。
        #[serde(default)]
        weight: Option<f64>,
        /// 副剛床の層せん断力係数 Ci の直接入力。水平力 = ci_override × 剛床重量として作用する。
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
