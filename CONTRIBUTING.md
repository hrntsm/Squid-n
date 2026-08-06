# コントリビューションガイド

Squid-n の開発に参加いただきありがとうございます。本書はビルド・テスト・静的解析・
ドキュメントの手順と、開発上の約束事をまとめたものです。

日本での利用を想定したプロジェクトです。**コミットメッセージ・コード中のコメント・
Issue / PR のやりとりは日本語**で行ってください。

## 前提ツール

- [Rust ツールチェイン](https://rustup.rs/)（stable）。`cargo` / `rustc` が使えること。
  CI は毎回その時点の最新 stable を取得するため、手元も `rustup update stable` で最新に
  保ってください（「[静的解析](#静的解析)」の「ツールチェインのバージョンを合わせる」）
- ドキュメントをローカルで確認する場合は [mdBook](https://rust-lang.github.io/mdBook/)

## ビルド

```bash
# ワークスペース全体ビルド
cargo build --workspace

# リリースビルド
cargo build --workspace --release
```

### 機能フラグ

| フラグ | 対象クレート | 内容 |
|--------|-------------|------|
| `gui` | squid-n-app | GUI（egui/eframe） |
| `mcp` | squid-n-mcp | MCP サーバ |
| `gpu` | squid-n-gpu | GPU 行列演算（P10、実装中） |
| `ml` | squid-n-ml | ML 断面提案（P11、未実装） |
| `p7` | squid-n-design-jp | 二次設計（Ds、偏心率、保有耐力、パネルせん断）。既定で有効 |

GPU や ML を無効化しても解析機能は CPU で動作する。

非デフォルトの機能フラグは `--workspace` ビルドでは検証されないため、
対象クレートを `-p` で指定して有効化する（ワークスペースルートで
`cargo build -p squid-n-app --features mcp` のように**フラグを持たない
クレートを指定するとエラーになる**ことに注意）。

```bash
# MCP サーバのビルド・テスト（bin ターゲット含む）
cargo build -p squid-n-mcp --features mcp
cargo test  -p squid-n-mcp --features mcp

# MCP サーバの起動（stdio。使い方は docs/mcp_server/ を参照）
cargo run -p squid-n-mcp --features mcp
```

## テスト

```bash
# 全テスト実行
cargo test --workspace

# 決定性テスト（100回ビット一致確認を含む）
cargo test --workspace deterministic

# 依存方向チェック（循環依存の検出）
cargo run -p xtask -- check-deps
```

依存方向は上層から下層のみです。新しいクレート間依存を追加した場合は
`check-deps` が通ることを必ず確認してください。

## 静的解析

**コミット前には必ず確認してください。** CI と同条件で実行します
（`--all-targets` がないとテストコードが clippy の対象外になります）。

```bash
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo clippy -p squid-n-app --all-targets --features gui --locked -- -D warnings
cargo clippy -p squid-n-mcp --all-targets --features mcp --locked -- -D warnings
cargo fmt --all -- --check
```

`cargo fmt --all` で自動整形できます。

**フラグ付きの 2 行を省略しないでください。** `gui`・`mcp` は既定で無効な
フィーチャフラグのため、1 行目のワークスペース全体の実行だけでは
`cfg(feature = "gui")` 配下のコード（GUI のビュー・テーブル・3D 表示のほぼ全体）が
コンパイルすらされません。フラグ付きでしか現れないビルドエラー・警告があります。

テストも同様に、フラグ付きの実行が必要です。

```bash
cargo test --workspace --locked
cargo test -p squid-n-app --features gui
cargo test -p squid-n-mcp --features mcp
```

### ツールチェインのバージョンを合わせる

**静的解析を実行する前に、ツールチェインを最新の stable へ更新してください。**

```bash
rustup update stable
cargo clippy --version
```

CI は `dtolnay/rust-toolchain@stable` で**その時点の最新 stable** を取得します。clippy の
lint は stable の更新で追加・拡張されるため、手元のツールチェインが古いと手元では通って
CI だけが落ちます。実際に、手元の 1.94 では通ったコードが CI の 1.97 で
`explicit_counter_loop` に引っかかった例があります。

`--locked` が固定するのは `Cargo.lock` による依存の解決だけで、ツールチェインの
バージョンは固定されません。既存の開発環境や、ツールチェインが同梱されたコンテナを
使う場合は、そこに入っているものが最新の stable とは限らないため特に注意してください。

## ドキュメントサイト（mdBook）

ドキュメントサイトは**アプリケーション利用者向け**（計算根拠・理論・出典）です。
[mdBook](https://rust-lang.github.io/mdBook/) で構築しています。
開発者向けドキュメント（設計仕様・検証記録・申し送り・ロードマップ）は `dev_docs/` に集約しており、
サイトには含めません。構成は [dev_docs/README.md](dev_docs/README.md) を参照してください。

```bash
# mdBook の導入（初回のみ）
cargo install mdbook

# ローカルでプレビュー（http://localhost:3000、変更を自動リロード）
mdbook serve --open

# 静的 HTML をビルド（出力先: book/）
mdbook build
```

- ソース: `docs/`（利用者向けコンテンツのみを置く）
- 目次: `docs/SUMMARY.md`（ページを追加・削除したらここも更新する）
- 設定: `book.toml`（数式は `mathjax-support` により `\\(...\\)`／`\\[...\\]` で記述）
- 各章は**章ディレクトリ＋小項目ページ**の構成です（`docs/calc_basis/` のほか、
  `docs/model_io/`・`docs/mcp_server/`・`docs/preparation/`・`docs/result_view/`）。
  小項目を追加したら章の `README.md` の一覧と `docs/SUMMARY.md` に追記してください
- 章トップの `README.md` は `index.html` として出力される一方、リンクは `README.html` の
  まま解決されるため、章ディレクトリを追加したら `book.toml` の
  `[output.html.redirect]` にも 1 行追加してください
- 全ページ末尾のフッター（MIT License と無保証の注意書き）は `theme/footer.js`・`theme/footer.css`
  で差し込んでいます。リンク先は `docs/introduction.md` の「ライセンスと免責事項」のため、
  この見出しを変更する場合は `theme/footer.js` のアンカーも合わせて更新してください

`main` への push で GitHub Pages に自動デプロイされます
（`.github/workflows/docs.yml`）。API リファレンス（rustdoc）も同時に生成され、
`/api/` 以下に併設されます。

## CI

PR を作成すると以下が自動実行されます（`.github/workflows/ci.yml`）。
ローカルで上記の静的解析・テストを通しておくと手戻りが減ります。

- テスト（`cargo test --workspace`）
- Clippy 静的解析
- フォーマットチェック
- 脆弱性確認（cargo audit）
- 依存性チェック（cargo-deny）

## プルリクエスト

1. `main` から作業ブランチを作成する
2. 変更を加え、上記のビルド・テスト・静的解析が通ることを確認する
3. 日本語でコミットメッセージを記述する
4. `main` 向けに PR を作成する
