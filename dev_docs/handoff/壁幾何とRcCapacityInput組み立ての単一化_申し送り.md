# 壁幾何と RcCapacityInput 組み立ての単一化 — 申し送り

作成日: 2026-08-12
対象コード:
`crates/squid-n-element/src/wall/misc_wall.rs`、
`crates/squid-n-core/src/rc_capacity.rs`、
`crates/squid-n-solver/src/nonlinear/pushover/shear_yield.rs`、
`crates/squid-n-app/src/app/mod.rs`

## 背景

同じ壁・同じ配筋に対して、寸法や入力の組み立て方が経路ごとに分かれていると、
後から読む人がどちらが正かを追えない。
今回は、すでに両方に実装がある次の 2 点だけを下層へ寄せた。

- 耐震壁の開口周比に使う壁長・高さ
- RC 矩形の配筋から `RcCapacityInput` を作る変換

保有水平耐力や床検定を `job` へ移す話、MCP 側を厚くする話は、入口がまだ片方にしかないため対象外とした。

## 変更内容

### 1. 耐震壁の開口周比寸法 → `wall_panel_geometry`

`wall_is_seismic` の r0 は、これまで `wall_extent`（節点間の最大水平距離 × 鉛直高さ）を見ていた。
四周判定はすでに `wall_panel_geometry` を使っており、寸法の情報源が 2 系統あった。

四周判定と r0 で同じ幾何を共有するようにし、`wall_extent` は削除した。
台形壁では旧包絡と上下辺平均長が食い違うため、その分岐を回帰テストで残した。
矩形壁の既存判定は、数値を動かさない前提で通している。

### 2. `RcCapacityInput` 組み立て → `squid_n_core::rc_capacity`

app と solver が、同じ規約をコメントで指しながら別実装を持っていた。
`rc_capacity_input_from_rect(b, d, main, rebar, …)` を core に置き、変換本体を 1 か所にした。

σy の材料強度割増は共有関数には入れない。
保有水平耐力専用の solver 経路だけが、呼び出し後に割増を掛ける。
app は強軸（`main_x`）を渡す薄い委譲に留めている。

## 意図的にやらないこと

- 保有水平耐力・床検定・`design_seismic_period` の job 化
- MCP を app 並みに厚くすること
- `one_bar_area` など、ワークスペース全体の小規模コピペ一掃
- 薄い再エクスポート削除や `viewer` / `panels` の分割

## 検証

- `cargo test -p squid-n-element misc_wall --lib`
- `cargo test -p squid-n-core rc_capacity --lib`
- `cargo test -p squid-n-solver shear_yield --lib`（関連）
- `cargo test -p squid-n-app --features gui test_rc_capacity_input_from_rect`
- （コミット前）対象クレートの clippy（`--all-targets`）、`cargo fmt`
