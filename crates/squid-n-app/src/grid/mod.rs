//! グリッド操作レイヤ（テーブルのスプレッドシート的編集）。
//!
//! 二層構成（dev_docs/specs/グリッド操作.md §3.1）:
//! - `core`: 純ロジック（egui 非依存・ヘッドレステスト可能）。選択状態機械・
//!   TSV 変換・タイル展開・ペースト検証。テーブル横断で共有する中核。
//! - `widget`: egui グルー（描画・入力処理、gui feature 限定）。
//!
//! グリッド操作レイヤ自体はテーブル固有の知識を持たず、テーブルごとの差は
//! すべてアダプタ（§3.4）に隔離する。

mod core;
#[cfg(feature = "gui")]
mod widget;

pub use self::core::*;
#[cfg(feature = "gui")]
pub use self::widget::*;

#[cfg(test)]
mod tests;
