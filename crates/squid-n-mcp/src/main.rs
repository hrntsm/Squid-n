//! Squid-N MCP サーバの起動バイナリ（stdio トランスポート）。
//!
//! `--features mcp` 付きでのみビルドされる（Cargo.toml の `required-features`）。
//!
//! 使い方:
//!   squid-n-mcp [MODEL.scz]
//!
//! 引数にモデルファイル（.scz）を渡すと起動時に読み込む。省略時は空モデルで
//! 起動する。解析結果ストアの場所は環境変数 `SQUID_N_RESULT_DIR` で指定でき、
//! 未設定なら OS 一時ディレクトリ配下の `squid-n-mcp-results` を使う。
//!
//! stdout は MCP の JSON-RPC トランスポートそのものなので、ログや診断は
//! 一切 stdout に書かないこと（壊れたフレームとしてクライアントが切断する）。

use squid_n_core::model::Model;
use squid_n_mcp::server::run_stdio_server;
use squid_n_mcp::{default_result_dir, ServerState};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let model = match std::env::args().nth(1) {
        // 準備計算の結果・解析結果は MCP サーバでは使わないため読み捨てる。
        Some(path) => squid_n_io::scz::load_scz(std::path::Path::new(&path))?.model,
        None => Model::default(),
    };
    let state = ServerState::with_fs_store(model, default_result_dir())?;
    run_stdio_server(state).await
}
