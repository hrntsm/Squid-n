//! Squid-n MCP サーバの起動バイナリ（stdio トランスポート）。
//!
//! 使い方:
//!   squid-n-mcp [MODEL.scz]
//!
//! stdout は MCP の JSON-RPC トランスポートそのものなので、ログや診断は
//! 一切 stdout に書かないこと。

use squid_n_core::model::Model;
use squid_n_mcp::server::run_stdio_server;
use squid_n_mcp::{default_result_dir, ServerState};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let model = match std::env::args().nth(1) {
        Some(path) => squid_n_io::scz::load_scz(std::path::Path::new(&path))?.model,
        None => Model::default(),
    };
    let state = ServerState::with_fs_store(model, default_result_dir())?;
    run_stdio_server(state).await
}
