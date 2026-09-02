# App の AppCore / UiState 分離 — 申し送り

作成日: 2026-09-02
対象コード: `crates/squid-n-app/src/app/mod.rs`、`crates/squid-n-app/src/app/actions/io.rs`、`crates/squid-n-app/src/mn_view.rs`

この申し送りでは、124 フィールドの `App` 構造体を `AppCore` と `UiState` へ分けた経緯と、その過程で見つかった 2 件の危険側の誤表示を記録します。

## 背景

`AppCore` / `UiState` の分離は、[`実装レビューと重複統合_申し送り.md`](実装レビューと重複統合_申し送り.md) §2.2 が「専用ブランチで一括実施」として残していた項目です。
直前の [`時刻歴ソルバの重複統合とHHT削除_申し送り.md`](時刻歴ソルバの重複統合とHHT削除_申し送り.md) でも、書き換え箇所が多く検証の大半がコンパイラ頼みになることを理由に見送っています。
本ブランチはその専用ブランチにあたります。

分離の目的は、フィールド数を減らすことではありません。
`load_model` が抱えていた「リセット対象を手で列挙する」構造をなくすことです。

`load_model` はモデルを丸ごと差し替える入口で、新規作成・サンプル読込・ST-Bridge 取り込み・架構ウィザード・プロジェクト読込のすべてが通ります。
ここでは旧モデルの id や添字を握った状態を捨てなければなりませんが、その対象は約 50 行の代入として手で並べられていました。
フィールドを増やすたびにここへ書き足す必要があり、書き足さなくてもコンパイルは通り、テストも落ちません。

実際、着手時点で 8 件の書き漏らしが残っていました。
そのうち 2 件は、利用者が実際より大きい耐力を読み取りうる危険側の誤りでした。

## 決めたこと

着手前に dig で 9 問を確認し、次の形に決めました。

| 問い | 決定 |
| --- | --- |
| 主対象 | `App` の完全な `AppCore` / `UiState` 二分割（約 3400 参照） |
| リセット対象の表現 | 型で分ける。`reset` は `scoped` への `Default::default()` 代入 |
| `AppCore` の内部 | 産物側だけ入れ子にする。解析条件・設計条件は直下に残す |
| `UiState` の gui ゲート | 丸ごとは囲まない。個々のフィールドの `cfg` は中で維持する |
| 見つかった漏れ | 同一ブランチで是正する。移設と是正はコミットを分ける |
| `MnCache` のキー | 断面形状を鍵に加える（根本原因のため） |
| 型の命名 | リセット意味論で対称に（`ModelScoped` / `UiModelScoped`） |
| メソッドの帰属 | 225 個とも `impl App` に残す |
| コミット | 移設・リセット是正・キー是正・文書の 4 つ |

分割の軸は「現在のモデルに紐づくか」の一点です。
`AppCore` と `UiState` の両方に同じ軸で `scoped` を置いたので、フィールドを追加するときの問いは「モデルを差し替えたら捨てるか」だけになります。

境界を gui フィーチャで引かなかったのは、それが実装の都合だからです。
`nav`（98 参照）と `camera` はどちらも画面の見え方ですが、前者は gui ゲートされていません。
フィーチャで割ると同じ性質のものが別の構造体へ散り、次に足す人が置き場所を決められません。

`AppCore` の条件側を入れ子にしなかったのは、保証が必要なのがリセットされる側だけだからです。
`analysis_cfg` は 260 参照あり、`self.core.conditions.analysis_cfg` と 4 段にしても得るものがありません。

## 変更内容

### 構造

```
App { core: AppCore, ui: UiState }

AppCore { model, log, analysis_cfg, design_term, …, scoped: ModelScoped }
UiState { scoped: UiModelScoped, view: UiViewState }
```

| 構造体 | フィールド数 | 内容 | モデル差し替え |
| --- | --- | --- | --- |
| `AppCore` 直下 | 16 | モデル本体・イベントログ・解析条件・設計条件 | 持ち越す |
| `ModelScoped` | 25 | 解析結果・準備計算・診断・stale 状態・実行中ジョブ・プロジェクトパス・波形選択・直近の報告 | 捨てる |
| `UiModelScoped` | 41 | 選択・ナビゲータの注目対象・入力途中のドラフト・詳細窓の選択部材とキャッシュ・作成モード | 捨てる |
| `UiViewState` | 42 | ドックの開閉と表示パネル・工程タブ・カメラ・配色・表示トグル・架構ウィザード | 持ち越す |

`load_model` の約 50 行は次の 4 行になりました。

```rust
model.migrate_legacy_auto_load_cases();
self.core.model = model;
self.core.scoped = ModelScoped::default();
self.ui.scoped = UiModelScoped::default();
self.sync_node_edit();
```

リセットの理由は各フィールドの doc へ移しています。
`job` を捨てる理由（旧モデルで計算中の結果が完了時に新モデルへ「最新結果」として適用される）のように、読む価値のある記述はフィールドの隣にあるほうが見つかります。

### 見つかった書き漏らし

| フィールド | 何を握っていたか | 影響 |
| --- | --- | --- |
| `mn_view` | 断面添字とその断面の曲面キャッシュ | **別断面の MN 相関曲面を黙って表示する** |
| `lumped_wave_library_selection` / `_sha256` | 質点系時刻歴の波形選択 | 一度も選んでいない波形が新しいプロジェクトの `.scz` へ記録される |
| `load_editor` | 荷重ケース・節点・部材の id | 旧モデル向けの入力内容が新モデルの画面に残る |
| `node_grid` | 表の選択矩形 | 旧モデルの行・列が選択されたまま残る |
| `view_mode_idx` | モード形の表示番号 | 存在しない次数を指したまま残る |
| `pending_duplicate_node_coord` | 節点追加の保留座標 | 旧モデル向けの確認ダイアログが残る |
| `node_draft` | 節点追加フォームの入力途中の座標 | 入力途中の値が残る |
| `project_path` | プロジェクトファイルのパス | 呼び出し元 5 箇所が個別に `None` を代入して辛うじて保たれていた |

`ds_beta_u_by_story` 群（保有水平耐力の警告表示）も列挙になく、`ModelScoped` へ置いたことで捨てられるようになりました。
ただしこちらは表示側が `displayed_pushover()` の内側にあり、`results` が捨てられる以上は画面に出ないので、実害はありませんでした。

`load_editor` も、`commit_load_editor` が現在のモデルに対して id を検証してエラーを返すため、誤った荷重が入ることはありませんでした。

`project_path` は `ModelScoped` へ置いたことで `load_model` が捨てるようになったので、呼び出し元 4 箇所の代入を削除しています。

### MN 相関曲面のキャッシュキー

`mn_view` をリセットしても、危険側の誤表示は半分しか塞げませんでした。

`MnCache` の鍵は `section_idx`（`usize`）と `strength` だけで、断面形状そのものを含んでいませんでした。
曲面は断面形状から算定するため、同じ添字の断面が別物へ変わってもキャッシュを有効と判定します。
モデルの差し替えはリセットで塞げますが、**同一モデル内で断面寸法を編集する経路はリセットでは塞げません**。
設計中に断面寸法を触るのは日常操作なので、こちらのほうが発生頻度は高いはずです。

そこで `MnCache` と `MThetaKey` の鍵へ `SectionShape` を加えました。
あわせて塑性化領域長さ Lp の自動設定（0.5D）の判定も、添字だけでなく形状の変化で行うようにしています。
断面せいを編集したときに Lp が旧断面の値のまま残ると、M-θ 骨格曲線が断面と食い違うためです。

## 設計判断

- **`UiState::reset_for_new_model()` は `*self = Default::default()` にできない。** gui 系 76 のうち `load_model` が捨てるのは 32 だけで、残る 44（ドック配置・カメラ・配色・表示トグル）は利用者の作業環境として持ち越す。全部捨てると、モデルを開くたびに画面が初期化されて作業にならない
- **`UiState` をフラットにして `reset` へ列挙する案は採らない。** それは `App` にあった書き漏らしの構造を `UiState` の中へ引っ越すだけで、目的を達成しない
- **メソッドは `impl App` に残す。** `impl AppCore` への移設は状態分割とは独立した変更で、同じ差分に混ぜるとレビューが成立しない。ただし `last_error`・`last_notice`・`combo_error` を `ModelScoped` へ、`log` を `AppCore` 直下へ置いており、後日の移設に必要な配置は済んでいる
- **`pending_wave_register` は `UiViewState` へ置く。** 波形ライブラリへの登録確認はモデルに紐づかないので、モデルを差し替えても有効なまま残るのが正しい

## 意図的にやらないこと（残課題）

- 225 個の `impl App` メソッドの `impl AppCore` への移設。GUI 側が `&mut app.core` と `&app.ui` を同時に借用できるようになるが、状態分割とは別の変更のため分ける
- 巨大テストファイルの分割。`app/tests.rs` は 9209 行ある
- `lib.rs` のフラット再エクスポート整理（[`実装レビューと重複統合_申し送り.md`](実装レビューと重複統合_申し送り.md) §2.3）

## 検証

- `cargo clippy --workspace --all-targets --locked -- -D warnings`
- `cargo clippy -p squid-n-app --all-targets --features gui --locked -- -D warnings`
- `cargo clippy -p squid-n-mcp --all-targets --features mcp --locked -- -D warnings`
- `cargo fmt --all -- --check`
- `cargo test --workspace --locked`（0 failed）
- `cargo test -p squid-n-app --features gui`（551 + 実モデル 27 + 壁モデル 9 + 壁間柱モデル 10、0 failed）
- `cargo test -p squid-n-mcp --features mcp`（37 passed）
- `cargo run -p xtask -- check-deps`（50 upstream checks OK）

移設で挙動が変わっていないことは、次の 2 つで確かめています。

- 移設コミットの時点で `full_model` の代表スカラのスナップショットが承認なしで一致した。実建物の ST-Bridge（4 層＋PH）を GUI と同じ入口で全解析まで通すテストのため、`App` 経由の解析結果が変わっていれば差分が出る
- 移設は機械的な置換で行い、置換対象を `app` 系の受け手と `impl App` ブロック内の `self` に限った。受け手の候補は全 124 フィールド名について実際に出現するものを列挙して確認し、`App` ではないもの（`view`・`contents`・`result`・`undo`・`saved` 等）を除いている

リセットの是正とキャッシュキーの是正には、次の回帰テストを追加しました。

- `test_load_model_resets_model_bound_ui_state` — `mn_view`・`load_editor`・`view_mode_idx`・`pending_duplicate_node_coord`・`node_draft` が捨てられる
- `test_load_model_resets_model_bound_core_state` — 質点系の波形選択・`project_path`・`ds_*` 群が捨てられる
- `test_load_model_keeps_view_settings` — ドック配置・タブ・配色・カメラ・解析条件は持ち越す
- `mn_view::tests::cache_is_rebuilt_when_shape_changes_at_same_index` — 添字も強度も同じまま断面せいを 600 mm から 900 mm へ変えると、軸圧縮耐力が増えた新しい曲面へ作り直される
- `mn_view::tests::cache_is_reused_when_nothing_changes` — 何も変わらなければ作り直さない（毎フレームの再計算を避ける本来の目的を壊していない）

利用者向けドキュメント（`docs/`）は変更していません。
計算根拠と既定値は変えておらず、MN 相関曲面ビューのキャッシュについて `docs/` に記述がないためです。
