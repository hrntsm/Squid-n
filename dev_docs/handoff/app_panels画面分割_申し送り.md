# app `panels` の画面単位分割 — 申し送り

作成日: 2026-08-13
対象コード: `crates/squid-n-app/src/app/panels/`

`panels.rs` が 3000 行に肥大し、ナビ・準備計算・解析実行・結果・インスペクタ・
ステータスバーが 1 ファイルに同居していた。後続の `viewer/mod.rs` ハブ薄化と
`actions` 解析ジョブ分割の前に、画面境界だけをファイルへ切り出した。
アルゴリズムの統合や挙動の変更は行っていない。

## 背景

GUI の `impl App` 描画メソッドが 1 ファイルに集まっていると、画面ごとの変更理由が
混ざり、レビュー単位も後続の構造 PR の移動単位も読めない。
`actions` は 2026-08-12 に機能境界で分割済みだが、`panels` は未着手のままだった。

分割の単位は「1 型 1 ファイル」ではなく **利用者向けの画面** とする。
`actions`（解析種別）と軸を揃えない。解析実行 UI の静的 / 固有値 / 増分 / 時刻歴は
同一画面の節なので切らない。

## 変更内容

`panels.rs` をディレクトリ化し、次の構成にした。

```
panels/
  mod.rs          — ファイルダイアログ、薄いタブスイッチャー、
                    ①②切替、結果表示対象の選択肢収集
  navigator.rs    — 左ドック ナビ
  draw_tools.rs   — 左ドック 作成パレット
  preparation.rs  — 右ドック ①準備計算
  analysis.rs     — 右ドック ②解析（実行 UI。節は切らない）
  results.rs      — 結果タブ（スイッチャー＋増分結果＋質点系）
  inspector.rs    — 右ドック インスペクタ
  status_bar.rs   — ステータスバー
```

メソッド本体は移動のみで、シグネチャと描画内容は変えていない。
可視性の引き上げは不要だった。`right_panel_switcher` と
`result_display_options` は親モジュールの private メソッドとして子から見える。
`toggle_dock_icon` は `status_bar` だけが使うため同ファイルの private 関数にした。

`app/mod.rs` の `mod panels;` はディレクトリ化してもそのまま解決する。

## 設計判断

- **実体のある画面だけファイル化した。** `model_tab` / `design_tab` / `report_tab` は
  本体が `tables/` や `design_view` 等への委譲なのでハブに残した。
- **結果タブの子ビューは `results.rs` に同居した。** 増分結果・質点系は結果画面の
  表示モードであり、解析実行画面の節を切らない判断と揃える。
- **ファイルダイアログはハブに残した。** 画面ではなく、入出力の薄い UI 入口である。

## 意図的にやらないこと（残課題）

本波（GUI ファイル分割）の残り:

- `viewer/mod.rs` のハブ薄化（`viewer_panel` は入口として残し、カメラ・ピック・
  CMQ・変形などを既存の `viewer/*.rs` へ移す）
- `actions/mod.rs` に残る解析ジョブの分割（共有基盤はハブ、静的・固有値・増分・
  時刻歴は兄弟ファイル。`design/` と同じ型）

本波の対象外:

- `AppCore` / `UiState` への型分離（専用ブランチ）
- app／MCP の設計入口を `squid-n-job` へ寄せる（機能統合）
- 巨大テストファイルの分割
- 単一機能の厚い計算コア（`wall_panel.rs` 等）の行数分割

## 検証

- `cargo fmt -p squid-n-app`
- `cargo check -p squid-n-app --features gui`
- `cargo clippy -p squid-n-app --all-targets --features gui --locked -- -D warnings`
