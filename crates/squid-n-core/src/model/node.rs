//! 節点の型。
//!
//! - [`Node`] — 節点（座標・拘束・質量・所属階・支点ばね）。

use super::*;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Node {
    pub id: NodeId,
    pub coord: [f64; 3],
    pub restraint: Dof6Mask,
    pub mass: Option<[f64; 6]>,
    pub story: Option<StoryId>,
    /// 支点ばね（全体座標系の各自由度ばね剛性）
    /// `[kx, ky, kz, krx, kry, krz]`（並進[N/mm]・回転[N·mm/rad]）。
    /// `restraint` で固定（`Dof6Mask::is_fixed`）されている自由度の値は無視される
    /// （固定を優先。ばねと固定支持の二重定義を避ける）。`None` はばね支持なし
    /// （従来どおり自由 or 固定の二値）。旧スキーマ（本フィールド無し）は
    /// `None` で補完される。
    #[serde(default)]
    pub support_spring: Option<[f64; 6]>,
}
