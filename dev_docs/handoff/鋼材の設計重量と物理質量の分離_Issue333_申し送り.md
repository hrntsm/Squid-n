作成日: 2026-09-22

# 鋼材の設計重量と物理質量の分離（Issue #333）申し送り

## 概要

鋼材の固定荷重用単位体積重量を基準資料どおり 78.5 kN/m³ に整合させ、あわせて
Squid-n の設計判断として設計重量（DL・地震用重量）と物理質量（質量行列・動的解析）を
分離した。`CorrectedLumped` と `LumpedOnly` の公称並進総動的質量を一致させる。

- 設計重量（鋼材）: γs = 78.5 kN/m³（基準資料 p.11 表2.1.1-1 由来）
- 物理質量（鋼材）: ρ = 7.85 t/m³（Squid-n 側の設計判断）
- 数量積算: 7.85 t/m³（不変。物理質量と値は同じだが用途は独立）
- 動的質量は `Σ mass_equiv / g` に一致し、地震用重量/g とは一致しない

設計判断は [ADR-0032](../adr/0032-steel-design-weight-vs-physical-mass.md) を正とする。

## 実装の要点

- `units.rs`: `STEEL_UNIT_WEIGHT_KN_M3 = 78.5`、`STEEL_MASS_DENSITY_T_M3 = 7.85`
  （`STEEL_MASS_DENSITY_TON_MM3 = 7.85e-9`）、`STEEL_UNIT_WEIGHT_TAKEOFF_T_M3 = 7.85` を
  独立定数として維持。
- `Material::design_unit_weight_n_per_mm3`: 材料区分から設計単位体積重量を解決
  （`Steel`→78.5、その他→密度×g）。`fc.is_some()` では判定しない。鉄筋は Steel に含めない。
- `SelfWeightItem`: `Line { load, mass_equiv, matrix_mass_equiv, extra_bottom_load,
  extra_bottom_mass_equiv, is_column }`、`Damper { load, mass_equiv }`、
  `Panel { load_shares, mass_equiv_shares, matrix_shares }`。`Line` の `mass_equiv` の
  躯体分は質量行列と同じ幾何（総断面・節点間長）で物理密度から算定する。
- `generate.rs`: 節点ごとに設計地震用重量 `node_weight`、物理質量相当 `node_mass_equiv`、
  質量行列に入る分 `node_matrix_mass_equiv` を別配列で集計。
  `CorrectedLumped` は `net = mass_equiv − matrix`、`LumpedOnly` は `net = mass_equiv`。
- `cascade.rs` に `SelfWeightBasis` を追加し、二次部材の自重を設計用と質量用で
  同じ支持経路・端部負担率により流す。
- プリセット・ST-Bridge 取込・UI 既定の鋼材・鉄筋密度は 7.85e-9 t/mm³。

## 検証

- `cargo test -p squid-n-core -p squid-n-load -p squid-n-io -p squid-n-design-jp -p squid-n-app --locked`
- `cargo clippy`（変更クレート、`--all-targets --locked -- -D warnings`）
- `cargo fmt --all -- --check`
- スナップショット（`full_model__snapshot_key_scalars`、`wall_model__snapshot_wall_bay_scalars`、
  `wall_model__snapshot_wall_ds_group_and_holding_capacity`）は、地震用重量が設計 78.5 で
  増える方向、固有周期・変形がわずかに変化する方向であることを確認して承認。

## 残課題

- **D8**: 質点系解析 `crates/squid-n-job/src/lumped_mass.rs` は `seismic_weight/g` を
  用いており、設計重量ベースの質量のまま。物理質量へ切り替えるかは別途判断。
  [残課題一覧](../handoff/残課題一覧.md) に記載。
- 二次部材・取り付く壁版の物理質量相当は、壁版がコンクリートで設計＝物理のため
  設計値と同値。二次部材（鋼）は物理質量で流す。
- RC/SRC 梁の設計重量（DL・地震用重量）は従来どおりスラブ厚を控除した断面積で算定する。
  物理質量相当は質量行列と同じ総断面・節点間長へ統一したため、`CorrectedLumped` の
  クランプ（`max(0, mass_equiv − matrix)`）は発生せず両方式が一致する。
- **CFT**: 線材の設計重量（DL・地震用重量）は鋼管断面×78.5 のみで、充填コンクリート分を
  欠く（質量行列は `element_mass_properties` でコアを含むため両方式が一致しない）。
  **危険側**。要検討。[残課題一覧](../handoff/残課題一覧.md) に記載。
