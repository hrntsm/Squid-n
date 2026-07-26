# アーキテクチャ

Squid-N は 15 のクレートから成る階層型アーキテクチャで構成されています。

```
Layer 0: squid-n-core（基本データ構造・DOF管理）、squid-n-math（疎行列・ソルバ）
Layer 1: squid-n-material（一軸材料履歴則）
Layer 2: squid-n-section（断面性能算定）
Layer 3: squid-n-element（梁・板・パネルゾーン要素）
Layer 4: squid-n-skeleton（スケルトン曲線）、squid-n-load（Ai分布、床荷重、荷重組合せ）、squid-n-solver（各種解析）
Layer 5: squid-n-design-jp（日本仕様設計計算）、squid-n-io（結果I/O）、squid-n-edit（編集トランザクション）
Layer 6: squid-n-mcp（MCP サーバ）、squid-n-app（GUI アプリケーション）
```

依存方向は上層から下層のみです。
循環依存は次のコマンドで検出します。

```bash
cargo run -p xtask -- check-deps
```

## クレート一覧

| クレート | 役割 |
|----------|------|
| `squid-n-core` | 基本データ構造・DOF 管理 |
| `squid-n-math` | 疎行列・ソルバ |
| `squid-n-material` | 一軸材料履歴則 |
| `squid-n-section` | 断面性能算定 |
| `squid-n-element` | 梁・板・パネルゾーン要素 |
| `squid-n-skeleton` | スケルトン曲線 |
| `squid-n-load` | Ai 分布・床荷重・荷重組合せ |
| `squid-n-solver` | 各種解析 |
| `squid-n-design-jp` | 日本仕様設計計算 |
| `squid-n-io` | 結果 I/O |
| `squid-n-edit` | 編集トランザクション |
| `squid-n-gpu` | GPU 高速化 |
| `squid-n-ml` | ML 断面提案 |
| `squid-n-mcp` | MCP サーバ |
| `squid-n-app` | GUI アプリケーション |

## API リファレンス

各クレートの API ドキュメント（rustdoc）は、CI で `cargo doc` から生成され、このサイトの [`api/`](./api/squid_n_core/index.html) 以下に併設されます。
