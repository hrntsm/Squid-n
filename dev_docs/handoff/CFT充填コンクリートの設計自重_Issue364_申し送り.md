作成日: 2026-09-23

# CFT 充填コンクリートの設計自重（Issue #364）申し送り

## 概要

CFT 線材の設計重量（DL・地震用重量）に充填コンクリート分が計上されておらず、
DL・地震用重量が過小になる危険側の差異を解消した。設計重量を
`w = γs·As + γC·Ac`（As=鋼管断面積、Ac=充填コンクリート断面積）とし、鋼管部には
通常の S 造と同じ鉄骨重量割増 `factor` を掛け、充填部には掛けない。物理質量（質量行列）
は従来どおり鋼管と充填を別領域で計上し、二重計上しない。

設計重量の単位体積重量は、鋼管部 `γs = 78.5 kN/m³`・充填部 `γC`（無筋コンクリート。
Fc と種類で決まる気乾単位体積重量。例 Fc36 普通 → 23.0 kN/m³）。

## 実装の要点

- `squid-n-core`（`model/material.rs`）: 充填部の単位体積重量の表引きを 1 か所
  （`Material::cft_filling_unit_weight_kn_m3`、非公開）へ集約し、公開メソッド
  `Material::cft_core_design_unit_weight_n_per_mm3`（γC を N/mm³ へ換算）と
  `Material::cft_core_mass_density`（γC を質量密度 t/mm³ へ換算）を追加。既存の
  `mass.rs` の `cft_core_density` はこのメソッドへ委譲する。
- `squid-n-load`（`story_gen/self_weight_calc.rs`）: 線材ループで
  `is_cft = matches!(sec.shape, CftBox | CftPipe)` を判定し、
  - 設計重量: `mat.design_unit_weight_n_per_mm3()·self_weight_area·factor + mat.cft_core_design_unit_weight_n_per_mm3()·core_area + extras`
  - 物理質量: 質量行列の単位長さ質量から充填部を引いた鋼管分に `factor` を掛け、
    充填部はそのまま足す（`core_area = sec.shape.cft_core_props().area`）。
  - `matrix_mass_equiv`（Basis::Matrix）は解析質量行列そのまま（factor なし）。
  - `factor` の判定のみ CFT を鋼材扱いへ変更し、`is_concrete` による `eff_len`・
    `max_depth`・`self_weight_area` は現状維持。
- 断面形状 `CftBox`/`CftPipe` 以外は `core_area = 0` となり現行式に一致する。
- 二次部材（小梁・間柱）には CFT を用いない前提とし、入力検証で弾くようにした。
  解析前チェック（`crates/squid-n-solver/src/statics/analysis/precheck.rs` の
  `model_issues`）が CFT 断面を割り当てられた小梁・間柱をエラーにし、解析を止める。
  編集コマンド（`crates/squid-n-edit/src/secondary.rs`。`AddUnassignedJoist`・
  `AddUnassignedPost`・`SetFloorRegionSecondaryJoists`・`SetWallRegionPosts`・
  `SetFloorRegionJoistSection`・`SetWallRegionPostSection`・`PlaceSecondaryMember`）は
  CFT 断面の指定を Noop で拒否する。二次部材の自重式は充填コンクリートを扱わないため、
  CFT を割り当てると充填分の自重が欠落して危険側になることへの対処である。
- 下階柱がなく柱脚に水平梁が接続する CFT 柱でも、`is_concrete` による基部梁せい付加
  （`max_depth`）の経路は RC/SRC と同じ条件で働き、付加される設計重量・物理質量相当に
  充填コンクリート分（γC·Ac·Dmax）と鋼管部への `effective_steel_factor` が反映される。
- 利用者向け計算根拠（[地震用重量・層データの生成](../../docs/calc_basis/01_荷重/05_地震用重量_層データの生成.md)・
  [標準材料一覧](../../docs/calc_basis/02_材料/04_標準材料一覧.md)）と
  [ADR-0033](../adr/0033-steel-design-weight-vs-physical-mass.md) を更新した。
  解析前チェックの一覧（[準備計算](../../docs/preparation/README.md)）にも
  二次部材の CFT 禁止を追記した。

## 検証

- `cargo test -p squid-n-core -p squid-n-load`（CFT の設計自重・地震用重量・両 MassMethod
  一致・factor の鋼管限定を追加テストで確認）
- `cargo clippy -p squid-n-load -p squid-n-core --all-targets --locked -- -D warnings`
- `cargo test -p squid-n-app`（`wall_model.rs` の CFT 側柱テスト、スナップショット）
- `cargo fmt --all -- --check`
- `cargo run -p xtask -- check-docs`

## 残課題

- ST-Bridge 取込の CFT は `StbSecColumn_CFT` の `strength_concrete` が断面の主材料に
  解決されるため、主材料がコンクリート区分になる。`SectionMassProperties` は CFT の
  主材料を「鋼材区分＋fc」に限定して検証するため、取込直後の CFT は質量特性の解決に
  失敗し、物理質量が 0 に落ちる。本修正後の設計重量も、この状態では鋼管部へコンクリート
  区分の密度由来の重量を掛けてしまう。ADR 0030 の記載（主材料＝充填コンクリート）とも
  関係する。外部 ST-Bridge の CFT 取込はテストされていない。別 Issue で追跡する。
