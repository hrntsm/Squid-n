//! 二次部材（小梁・間柱）。
//!
//! 全体解析（剛性行列）には算入しない部材で、床荷重・自重を主架構
//! （大梁・柱）への荷重（CMQ: 中間集中荷重・部分分布荷重）として伝達する
//! ために保持する。ST-Bridge の `StbBeam`（小梁）・`StbPost`（間柱）が
//! 取り込み時にここへ入る。
//!
//! - 端部節点はモデルの節点（`Model.nodes`）を参照するが、要素が接続しない
//!   節点は解析自由度から自動的に除外される（`DofMap::build`）。
//! - 床荷重の分配・自重の荷重ケース同期が、二次部材経由の荷重を主架構の
//!   梁への集中荷重へ変換する（`squid_n_load::secondary`）。

use super::*;

/// 二次部材の種別。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SecondaryMemberKind {
    /// 小梁（床荷重を大梁へ伝える。単純梁として両端反力を返す）
    Joist,
    /// 間柱（壁荷重・自重を上下の梁へ伝える）
    Post,
}

/// 二次部材（小梁・間柱）。全体解析の対象外（[`crate::model::secondary`] モジュール
/// ドキュメント参照）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SecondaryMember {
    pub kind: SecondaryMemberKind,
    /// 両端節点（小梁: 始端→終端、間柱: 下端→上端の順を推奨。順序に依存しない）。
    pub nodes: [NodeId; 2],
    /// 断面参照（自重算定・将来の小梁/間柱断面算定に用いる）。
    pub section: Option<SectionId>,
    /// 表示名（ST-Bridge の name 等）。
    pub name: String,
}
