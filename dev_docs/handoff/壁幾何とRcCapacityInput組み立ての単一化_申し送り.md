# 壁幾何と RcCapacityInput 組み立ての単一化 — 申し送り

作成日: 2026-08-12
対象コード:
`crates/squid-n-element/src/wall/misc_wall.rs`、
`crates/squid-n-core/src/rc_capacity.rs`、
`crates/squid-n-solver/src/nonlinear/pushover/shear_yield.rs`、
`crates/squid-n-app/src/app/mod.rs`

機能の二重実装のうち、いま両方に実体があるものだけを潰した。
保有・床・周期の job 化や MCP への機能追加は対象外。

## 変更内容

### 1. 耐震壁の開口周比寸法 → `wall_panel_geometry`

`wall_is_seismic` の r0 が使っていた `wall_extent`（節点間の最大水平距離×鉛直高さ）を削除し、
四周判定と同じ `wall_panel_geometry` の `lw` / `h` を共有する。
台形壁では旧包絡と上下辺平均長が食い違うため、その分岐を回帰テストで明文化した。
矩形壁の既存判定は数値不変を維持する。

### 2. `RcCapacityInput` 組み立て → `squid_n_core::rc_capacity`

`rc_capacity_input_from_rect(b, d, main, rebar, …)` を core に置く。
σy に材料強度割増は掛けない。solver の保有経路だけ後掛けする。
app は強軸（`main_x`）を渡す薄い委譲に留める。

## 意図的にやらないこと

- 保有水平耐力・床検定・`design_seismic_period` の job 化
- MCP を app 並みに厚くすること
- `one_bar_area` 等のワークスペース全体の小規模コピペ一掃
- 薄い再エクスポート削除・`viewer`/`panels` 分割

## 検証

- `cargo test -p squid-n-element misc_wall --lib`
- `cargo test -p squid-n-core rc_capacity --lib`
- `cargo test -p squid-n-solver shear_yield --lib`（関連）
- `cargo test -p squid-n-app --features gui test_rc_capacity_input_from_rect`
- （コミット前）対象クレートの clippy（`--all-targets`）、`cargo fmt`
