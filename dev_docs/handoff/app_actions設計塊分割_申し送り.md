# app `actions` の構造分割 — 申し送り

作成日: 2026-08-12
対象コード: `crates/squid-n-app/src/app/actions/`

`actions.rs` が 3000 行超に肥大し、解析実行・入出力・荷重同期・設計入口が
1 ファイルに同居していた。後続で設計入口を `squid-n-job` へ寄せる予定があるため、
先に責務境界だけをファイルへ切り出した。アルゴリズムの統合や挙動の変更は行っていない。

## 背景

GUI と MCP で同じ設計・解析を別入口から呼ぶ構成では、入口の配線がクレートをまたいで
複製されやすい。共通化の受け皿として `squid-n-job` はあるが、app 側の入口が
巨大な `actions.rs` に埋もれたままだと、どのメソッドを移すかの単位が読めない。

このため、機能統合の前に **構造だけを分ける** 方針とした。
式や判定の単一化（壁幾何、`RcCapacityInput` 組み立てなど）は別 PR とする。

## 変更内容

`actions.rs` をディレクトリ化し、次の構成にした。

```
actions/
  mod.rs          — 解析ジョブ基盤・静的／増分／時刻歴・診断など（残り）
  io.rs           — プロジェクト／ST-Bridge 入出力とモデル読込
  loads.rs        — 荷重ケース自動同期・床荷重分配・CMQ 表示
  design/
    mod.rs
    holding.rs    — 保有水平耐力
    ultimate.rs   — 終局検定と需要組み立て
    check.rs      — 許容応力度検定・床内検定
    period.rs     — 設計用固有周期
```

メソッド本体は移動のみで、シグネチャと計算内容は変えていない。
分割に伴い、親モジュールから呼ばれる private メソッド
（`empty_lateral_case_in_combo` / `compute_auto_load_sync_hash`）は `pub(crate)` にした。
可視性を上げないと、別ファイルの `impl App` から見えなくなるためである。

`needs_recording_confirm` と `SAVE_RECORDING_CONFIRM_BYTES` は `io` に置き、
テストから従来どおり `actions::` 経由で参照できるよう `#[cfg(test)]` で再エクスポートした。
本番ビルドでは未使用 import になるのを避けるためである。

`app/mod.rs` にある設計ヘルパ（`rc_capacity_input_from_rect` など）は動かしていない。
入口の境界だけ先に作る意図で、入力組み立ての統合は後続とする。

## 設計判断

- **設計入口を最初に切った。** 保有・終局・許容・床検定は後続の job 寄せの主戦場であり、
  ここだけ先に境界を作っておくと移動単位が明確になる。
- **`design_seismic_period` も設計側へ置いた。** 呼び出しの多くは荷重同期だが、
  「設計用周期の決定」という責務は設計に寄せたほうが、周期の情報源が 1 か所に見える。
- **I/O と荷重同期も同波で切った。** 当初は設計塊だけを想定していたが、
  `actions` 残部がなお 2000 行超あるとレビュー単位として大きいため、
  境界がはっきりしている入出力と荷重同期まで広げた。
- **解析ジョブ残部（静的・増分・時刻歴）はまだ `mod.rs` に残している。**
  相互呼び出しが多く、一度に切ると差分が追いづらいため、次の構造 PR に回す。

## 意図的にやらないこと（残課題）

- app／MCP の設計入口を `squid-n-job` へ寄せる（機能統合）
- 壁幾何や耐力入力組み立てなど、クレート横断の式の単一化
- `viewer/mod.rs` / `panels.rs` の分割
- `actions/mod.rs` に残る解析ジョブのさらなる分割
- 薄い再エクスポートの削除、`floor_grillage` の所属移動

## 検証

- `cargo fmt -p squid-n-app`
- `cargo check -p squid-n-app --features gui`
- `cargo clippy -p squid-n-app --all-targets --features gui --locked -- -D warnings`
