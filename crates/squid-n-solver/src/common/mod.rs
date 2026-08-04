//! 解析共通の基盤モジュール。
//!
//! - [`assemble`] —    全体剛性・質量・荷重ベクトルの組み立て
//! - [`constraint`] —  拘束条件（自由度縮約）
//! - [`transaction`] — 全要素確定状態のスナップショット
//! - [`csc_cache`] —   CSC 疎行列組立てのキャッシュ（時刻歴応答解析の Newton 反復向け）
//! - [`newton`] —      Newton 反復の共通足場（収束規約・相対残差判定）
pub mod assemble;
pub mod constraint;
pub mod csc_cache;
pub mod newton;
pub mod transaction;
