# アーキテクチャ

Squid-n は 14 のクレートから成る階層型アーキテクチャで構成されています。

```
Layer 0: squid-n-core（基本データ構造・DOF 管理・荷重組合せ）、squid-n-math（疎行列・ソルバ）、
         squid-n-material（一軸材料履歴則）
Layer 1: squid-n-section（断面性能算定）、squid-n-load（Ai 分布・床荷重）
Layer 2: squid-n-edit（編集トランザクション）、squid-n-skeleton（スケルトン曲線）
Layer 3: squid-n-element（梁・板・パネルゾーン要素）
Layer 4: squid-n-solver（各種解析）、squid-n-io（結果 I/O）
Layer 5: squid-n-design-jp（日本仕様設計計算）
Layer 6: squid-n-job（解析前処理・解析条件・解析の純粋計算）
Layer 7: squid-n-mcp（MCP サーバ）、squid-n-app（GUI アプリケーション）
```

`squid-n-job` は GUI と MCP サーバの共通下層です。
両者は同じ解析を別々の入口から実行するため、前処理（剛域・仕口パネル）と解析条件をここに集約し、**同じモデルに対して同じ結果を返す**ことを保証しています。

依存方向は上層から下層のみと定めているため、循環依存が生じていないかを次のコマンドで検出します。

```bash
cargo run -p xtask -- check-deps
```

## クレート一覧

| クレート | 役割 |
|----------|------|
| `squid-n-core` | 基本データ構造・DOF 管理・荷重組合せ |
| `squid-n-math` | 疎行列・ソルバ |
| `squid-n-material` | 一軸材料履歴則 |
| `squid-n-section` | 断面性能算定 |
| `squid-n-element` | 梁・板・パネルゾーン要素 |
| `squid-n-skeleton` | スケルトン曲線 |
| `squid-n-load` | Ai 分布・床荷重 |
| `squid-n-solver` | 各種解析 |
| `squid-n-design-jp` | 日本仕様設計計算 |
| `squid-n-io` | 結果 I/O |
| `squid-n-edit` | 編集トランザクション |
| `squid-n-job` | 解析前処理・解析条件・解析の純粋計算（GUI と MCP の共通下層） |
| `squid-n-mcp` | MCP サーバ |
| `squid-n-app` | GUI アプリケーション |

## API リファレンス

各クレートの API ドキュメント（rustdoc）は、CI で `cargo doc` から生成され、このサイトの [`api/`](./api/squid_n_core/index.html) 以下に併設されます。
