作成日: 2026-09-22

# 鋼材の設計重量と物理質量の分離（Issue #333）申し送り

## 概要

鋼材の固定荷重用単位体積重量を基準資料どおり 78.5 kN/m³ に整合させ、あわせて
Squid-n の設計判断として設計重量（DL・地震用重量）と物理質量（質量行列・動的解析）を
分離した。`CorrectedLumped` と `LumpedOnly` の公称並進総動的質量を一致させる。

- 設計重量（鋼材）: γs = 78.5 kN/m³（基準資料 p.11 表2.1.1-1 由来の**固定値**。密度 > 0 の
  鋼材は保存密度に依存せず 78.5 固定。質量密度 0 以下は自重 0）
- 物理質量（鋼材）: ρ = 7.85 t/m³（Squid-n 側の設計判断）
- 数量積算: 7.85 t/m³（不変。物理質量と値は同じだが用途は独立）
- 動的質量は `Σ mass_equiv / g` に一致し、地震用重量/g とは一致しない

設計判断は [ADR-0033](../adr/0033-steel-design-weight-vs-physical-mass.md) を正とする。

## 実装の要点

- `units.rs`: `STEEL_UNIT_WEIGHT_KN_M3 = 78.5`、`STEEL_MASS_DENSITY_T_M3 = 7.85`
  （`STEEL_MASS_DENSITY_TON_MM3 = 7.85e-9`）、`STEEL_UNIT_WEIGHT_TAKEOFF_T_M3 = 7.85` を
  独立定数として維持。
- `Material::design_unit_weight_n_per_mm3`: 材料区分から設計単位体積重量を解決
  （`Steel` かつ密度 > 0 → 78.5 kN/m³ 固定、密度 0 以下 → 0、その他 → 密度×g）。
  `fc.is_some()` では判定しない。鉄筋は Steel に含めない。
- `SelfWeightItem`: `Line { load, mass_equiv, matrix_mass_equiv, extra_bottom_load,
  extra_bottom_mass_equiv, is_column }`、`Damper { load, mass_equiv }`、
  `Panel { load_shares, mass_equiv_shares, matrix_shares }`。`Line` の `mass_equiv` の
  躯体分は質量行列と同じ幾何（総断面・節点間長）で物理密度から算定し、鉄骨重量割増
  `factor` を乗じる。付加線重量・仕上げは割増の対象外としてそのまま加算する。
- `generate.rs`: 節点ごとに設計地震用重量 `node_weight`、物理質量相当 `node_mass_equiv`、
  質量行列に入る分 `node_matrix_mass_equiv` を別配列で集計。内部で `SelfWeightMode`
  （`Density`／`GravityCasesOnly`／`SyncedGravityCases`）を切り替える。
  - `generate_stories_multi`・`generate_stories_with_opts(..., true, ...)` → `Density`
  - `generate_stories_with_opts(..., false, ...)` → `GravityCasesOnly`。重力ケースの内容
    だけを算入し、`node_mass_equiv = node_weight`、`node_matrix_mass_equiv = 0`
    （モデル自重の置換・質量行列分の控除をしない）
  - `generate_stories_with_synced_self_weight`（新規公開入口）→ `SyncedGravityCases`。
    自重同期済み DL を前提に、DL の設計自重を質量のみ物理質量へ置換する。節点ごとの
    `node_mass_equiv` が有限かつ非負（許容差内）であることを検査し、負ならエラー。
- `wall_plate_load.rs`: 置換量を重力ケースと同じ帰属で求めるため、二次部材端の反力を
  `resolve_nodal_to_primary` で主架構へ解決してから集計する `*_resolved` 版を追加。
  設計重量（地震用重量）の節点集計は従来どおり二次部材端へ配る版を維持する。
- `cascade.rs` に `SelfWeightBasis` を追加し、二次部材の自重を設計用と質量用で
  同じ支持経路・端部負担率により流す。質量用（`MassEquiv`）でも鋼材には設計と同じ
  鉄骨重量割増 `factor` を掛ける（主架構線材と同一規則）。
- プリセット・ST-Bridge 取込・UI 既定の鋼材・鉄筋密度は 7.85e-9 t/mm³。
- アプリ `generate_stories_action` は、DL がないモデルでは `Density`、DL があるモデルでは
  直前に `sync_gravity_load_cases_action` で DL を自動同期済みのため
  `generate_stories_with_synced_self_weight` を呼ぶ。

## 検証

- `cargo test -p squid-n-core -p squid-n-load -p squid-n-io -p squid-n-design-jp -p squid-n-app --locked`
- `cargo clippy`（変更クレート、`--all-targets --locked -- -D warnings`）
- `cargo fmt --all -- --check`
- スナップショット（`full_model__snapshot_key_scalars`）は、二次部材・壁版の自重の
  動的質量が設計重量ベースから物理質量ベースへ是正され、固有周期・応答がわずかに
  変化する方向であることを確認して更新。`wall_model` 系は変化なし。

## 残課題

- **D8（対応済み）**: 質点系解析 `crates/squid-n-job/src/lumped_mass.rs` の層質量は
  `seismic_weight/g`（設計重量ベース）であったが、[ADR-0034](../adr/0034-lumped-mass-physical-mass.md)
  と [`質点系の物理質量化_Issue365_申し送り.md`](質点系の物理質量化_Issue365_申し送り.md) により、
  階生成が `Story::dynamic_mass`（物理質量相当）へ保存した値へ統一した
  （GitHub Issue #365）。[残課題一覧](../handoff/残課題一覧.md) の該当行は削除済み。
- **高密度カスタム鋼材**: 設計重量を 78.5 kN/m³ 固定としたため、78.5/g ≈ 8.005 t/m³ を
  超える密度を入力した鋼材では設計重量が物理重量を下回る（DL・地震用重量を過小評価する
  危険側）。ADR-0033 に留保として明記。入力制限・警告を設けるかは別途判断。
- 二次部材・取り付く壁版の物理質量相当は、壁版がコンクリートで設計＝物理のため
  設計値と同値。二次部材（鋼）は物理質量で流す。
- RC/SRC 梁の設計重量（DL・地震用重量）は従来どおりスラブ厚を控除した断面積で算定する。
  物理質量相当は質量行列と同じ総断面・節点間長へ統一したため、`CorrectedLumped` の
  クランプ（`max(0, mass_equiv − matrix)`）は発生せず両方式が一致する。
- **CFT**: 線材の設計重量（DL・地震用重量）は鋼管断面×78.5 のみで、充填コンクリート分を
  欠く（質量行列は `element_mass_properties` でコアを含むため両方式が一致しない）。
  **危険側**。要検討。[GitHub Issue #364](https://github.com/hrntsm/Squid-n/issues/364)、
  [残課題一覧](../handoff/残課題一覧.md) に記載。
