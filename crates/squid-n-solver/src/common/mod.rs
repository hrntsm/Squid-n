//! 解析共通の基盤モジュール。
//!
//! - [`assemble`] —    全体剛性・質量・荷重ベクトルの組み立て
//! - [`constraint`] —  拘束条件（自由度縮約）
//! - [`transaction`] — 全要素確定状態のスナップショット
//! - [`csc_cache`] —   CSC 疎行列組立てのキャッシュ（時刻歴応答解析の Newton 反復向け）
//! - [`newton`] —      Newton 反復の共通足場（収束規約・相対残差判定）
//! - [`tangent`] —     非線形反復の接線剛性・内力の組み立て（要素状態から）
//! - [`elem_loop`] —   要素ループの並列/逐次足場（順序保証付き map・可変 for_each）
pub mod assemble;
pub mod constraint;
pub mod csc_cache;
pub(crate) mod elem_loop;
pub mod newton;
pub(crate) mod tangent;
pub mod transaction;
