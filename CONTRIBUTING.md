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

### 実モデルの統合テスト

`crates/squid-n-app/tests/full_model.rs` は、実建物の ST-Bridge
（`crates/squid-n-app/tests/fixtures/model.stb`。4 層＋PH の S 造・一部 RC、
節点 166・解析要素 115・小梁 56。ST-Bridge 上のスラブ片 82 枚は取り込み時に大梁の区画（床領域）26 へ帰属を割り当てる）を読み込み、GUI のボタンが呼ぶのと
同じ入口（`App` の `run_*` / `compute_*`）で全解析を通します。手組みの小規模
モデルでは現れない、実建物特有の構成（剛床・二次部材・多数のスラブ）に起因する
退行を検出することが目的です。

```bash
cargo test -p squid-n-app --test full_model

# 既知の不具合として #[ignore] にしているテストを実行する
cargo test -p squid-n-app --test full_model -- --ignored

# スナップショット（代表スカラ）の差分を承認する
cargo insta review
```

**`App` に解析エントリ（`run_*` / `compute_*`）を追加したら、このファイルにも
テストを追加してください。** 追加を怠ると、その機能だけが回帰検出の対象外に
なります。既定で無効な機能や利用頻度の低い経路ほど、この漏れによって静かに
壊れます。

現在 `#[ignore]` にしている既知の不具合と、その原因・再開手順は
[dev_docs/handoff/実モデル統合テスト_申し送り.md](dev_docs/handoff/実モデル統合テスト_申し送り.md)
にまとめています。

`crates/squid-n-app/tests/wall_model.rs` は、`full_model.rs` のフィクスチャに壁要素が
含まれていないことを補う、壁（耐震壁・フレーム外雑壁）専用の最小フィクスチャです。
`full_model.rs` の床領域（26 件）を巻き込まずに壁関連の代表スカラを独立してスナップショット
します。壁の型（`WallAttr`・`MiscWall` 等）を変更したときはこちらも実行してください。

```bash
cargo test -p squid-n-app --test wall_model
```

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
`explicit_counter_loop` に引っかかった例があります。また、手元の 1.96/1.97 では通ったコードが
CI の 1.98 で `needless_late_init` に引っかかった例もあります。**2026-08-22 時点で
CI が使う stable は 1.98 以上のため、手元のツールチェインは少なくとも 1.98 以上に
更新してください。**

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

PR の説明は、レビュアーが「何を、なぜ変えたか」「どのように確認したか」「既存の挙動にどんな影響があるか」を追えることを目的とします。PR 本文には `.github/pull_request_template.md` を使用します。

### 手順

1. `main` から作業ブランチを作成する
2. 変更を加え、上記のビルド・テスト・静的解析が通ることを確認する
3. 日本語でコミットメッセージを記述する（プレフィックス規約は「Git コミットルール」を参照）
4. `main` 向けに PR を作成し、本文を `.github/pull_request_template.md` の構成に従って記述する

### タイトル

PR タイトルはコミットメッセージと同じプレフィックス規約に従います。

```text
<プレフィックス>: <簡潔な変更内容>
```

例:

```text
feat: 荷重ケースのツリー編集に対応する
fix: 材料未割当の診断漏れを修正する
docs: 計算根拠の説明を追加する
```

### 本文

以下を具体的に記載してください。

- **概要**: この PR の目的と、なぜこの変更が必要なのか
- **変更内容**: 主な変更を、必要なら機能・モデル・UI・テスト・ドキュメントなどに分けて説明
- **影響・注意事項**: 利用者から見える挙動、データ形式、既存プロジェクトファイル、後方互換性などへの影響
- **検証**: 実行したテスト・静的解析・ビルドなどと結果
- **ドキュメント**: `docs/` / `dev_docs/` を更新した場合は対象と理由
- **残課題**: 後続 PR に回した事項や既知の制約
- **レビューで確認してほしい点**: 特にレビューしてほしい設計・仕様・実装上のポイント（なければ省略可）

大きな変更では、変更前後の挙動や設計上の判断理由を表や箇条書きで整理するとレビューしやすくなります。レビュー中に検出して修正した不具合や回帰テストを追加した場合は、その内容も記載してください。

### 検証

変更内容に応じて、必要な検証を行ってください。通常は以下を使用します。

```bash
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo clippy -p squid-n-app --all-targets --features gui --locked -- -D warnings
cargo clippy -p squid-n-mcp --all-targets --features mcp --locked -- -D warnings
cargo fmt --all -- --check
cargo test --workspace --locked
cargo test -p squid-n-app --features gui
cargo test -p squid-n-mcp --features mcp
cargo run -p xtask -- check-deps
mdbook build
```

すべてを実行しない場合は、変更範囲に応じて必要なものを選び、PR 本文の「検証」に実行結果を記載してください。CI で実行される検証と、ローカルで実行した検証を混同しないようにしてください。

### ドキュメント

`docs/` は利用者向けの現在の仕様・計算根拠、`dev_docs/` は開発経緯・V&V・申し送り・ロードマップです。変更内容に対応する文書を更新してください。既定値を変更した場合は、利用者が結果の根拠を追えるよう `docs/` に既定値を反映します。

### dev_docs（申し送り・V&V）

実装・修正の申し送りや V&V レポートを追加・更新したら、**集約一覧も合わせて更新**してください。
詳細な索引構成は [dev_docs/README.md](dev_docs/README.md) を参照。

| 操作 | 更新するファイル |
|------|------------------|
| 申し送りを新規作成 | [`dev_docs/handoff/README.md`](dev_docs/handoff/README.md) のカテゴリ表・**時系列**表に 1 行ずつ追加（ファイル先頭に `作成日:` を記載） |
| 残課題がある | [`dev_docs/handoff/残課題一覧.md`](dev_docs/handoff/残課題一覧.md) に 1 行追加 |
| 残課題が解消した | 上記 `残課題一覧.md` から該当行を削除 |
| V&V レポートを追加・更新 | [`dev_docs/v_and_v/README.md`](dev_docs/v_and_v/README.md) のレポート目録と索引 #N の状態（✅/🔶/❌）を更新 |
| 未検証・一部が残る | [`dev_docs/v_and_v/未検証一覧.md`](dev_docs/v_and_v/未検証一覧.md) に 1 行追加 |

申し送りの詳細は個別ファイル、横断的な未完了は [`残課題一覧.md`](dev_docs/handoff/残課題一覧.md)、
V&V の未完了は [`未検証一覧.md`](dev_docs/v_and_v/未検証一覧.md) が入口です。
PR 本文の「残課題」欄と内容が食い違わないよう、PR 作成前に一覧の更新を済ませてください。
