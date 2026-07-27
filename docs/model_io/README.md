# モデル入出力（ファイル形式）

Squid-n は構造モデルを 2 つのファイル形式で入出力します。

| 形式 | 拡張子 | 用途 | 往復精度 |
|---|---|---|---|
| Squid-n プロジェクト | `.scz` | Squid-n ネイティブの保存形式。モデルを欠損なく保存・読込する | 完全一致 |
| [ST-Bridge](https://www.building-smart.or.jp/meeting/buildingsmart/st-bridge/) | `.stb` / `.xml` | 他社の一貫計算プログラムや BIM ツールとのモデル受け渡し | サブセットのみ意味的一致 |

いずれも GUI アプリ（`squid-n-app`）のファイルメニューから利用でき、実装は `squid-n-io` クレートにあります。**日常的な保存・読込は `.scz`**、**他ツール連携は `.stb`** と使い分けます。

- 実装: `.scz` は `squid-n-io::scz`（`save_scz` / `load_scz`）、ST-Bridge は `squid-n-io::stbridge`（`export_stbridge` / `import_stbridge`）。

## この章の内容

- [Squid-n プロジェクト形式（.scz）](./01_プロジェクト形式_scz.md)
- [ST-Bridge 形式（.stb / .xml）](./02_ST-Bridge_形式.md)
- [ST-Bridge 要素別 変換状況一覧](./03_ST-Bridge_要素別変換状況.md)

## MCP サーバでのモデル入力

MCP サーバ（`squid-n-mcp`）は起動時の第 1 引数でモデルファイルを読み込みます。**現状は `.scz` のみ**に対応しており、ST-Bridge ファイルの直接指定には対応していません。ST-Bridge から取り込む場合は、いったん GUI で読み込んで `.scz` として保存し、その `.scz` を MCP サーバに渡します。詳細は [MCP サーバ](../mcp_server/README.md)を参照してください。
