//! 解析ジョブ（モデルの解析前処理・解析条件・各解析の純粋計算）。
//!
//! GUI（`squid-n-app`）と MCP サーバ（`squid-n-mcp`）は同じ解析を別々の入口から
//! 実行する。両者が `squid-n-app` に依存できない関係にあるため、かつては MCP 側が
//! 前処理と解析の配線を複製しており、**前処理の内容が食い違っていた**
//! （MCP は剛域のみを適用し、仕口パネルの生成と DL/LL/EX/EY の自動同期を経ない
//! モデルを解いていた）。
//!
//! 本クレートは両者の共通下層として、
//!
//! - [`prepare`] — 解析前処理（剛域・仕口パネル・荷重ケースの自動同期）
//! - [`settings`] — 解析条件（[`settings::AnalysisSettings`]）
//! - [`compute`] — 各解析の純粋計算（所有モデル＋解析条件 → 結果）
//! - [`error`] — ジョブのエラー型（[`error::JobError`]）
//!
//! を持つ。結果の整形（MCP の JSON・GUI の表示）は各クレートに残す
//! （両者の要件が結合するのを避けるため）。

pub mod compute;
pub mod error;
pub mod prepare;
pub mod settings;

pub use error::{JobError, JobResult};
pub use settings::AnalysisSettings;
