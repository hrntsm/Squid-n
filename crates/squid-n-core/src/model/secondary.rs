//! 二次部材（小梁・間柱）。全体解析（剛性行列）には算入しない。

use super::*;

/// 二次部材の種別。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum SecondaryMemberKind {
    /// 小梁。
    Joist,
    /// 間柱。
    Post,
}

/// 二次部材（小梁・間柱）。全体解析の対象外。
///
/// 実体は床領域（[`super::FloorRegion::secondary_joists`]）または壁領域
/// （[`super::WallRegion::posts`]）が保持する。所属未割当の小梁・間柱だけ
/// [`super::Model::unassigned_joists`] / [`super::Model::unassigned_posts`] に置く。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SecondaryMember {
    pub kind: SecondaryMemberKind,
    /// 両端節点（小梁: 始端→終端、間柱: 下端→上端の順を推奨。順序に依存しない）。
    pub nodes: [NodeId; 2],
    /// 断面参照。
    pub section: Option<SectionId>,
    /// 表示名。
    pub name: String,
}
