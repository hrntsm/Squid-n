作成日: 2026-09-23

# 質点系の物理質量化（Issue #365）申し送り

## 概要

質点系（串団子）の層質量を、設計地震用重量ベース（`seismic_weight/g`。鋼材 78.5 kN/m³）
から物理質量相当（鋼材 7.85 t/m³）へ切り替え、フレームの動的解析と同じ質量源に統一した。
設計判断は [ADR-0034](../adr/0034-lumped-mass-physical-mass.md) を正とする。

## 実装の要点

- **保存フィールド**（`squid-n-core/src/model/story.rs`）:
  `StoryDynamicMass { mass_equiv_weight_n [N], center_xy_mm [mm], inertia_t_mm2 [t·mm²] }`。
  `Story::dynamic_mass: Option<StoryDynamicMass>` を `Story` の**末尾フィールド**として追加
  （msgpack の位置ずれ防止）。`Layer::dynamic_mass` は上端床の値を写す導出値。
- **算定規則**（`squid-n-load/src/story_gen/generate.rs`）: 階ループで節点の物理質量相当重量
  `node_mass_equiv` の**全量** \\( \sum w\_{\text{mass},i} \\) を集計する。`w_sum > 0` なら
  質量重心 \\( \sum(w x)/w\_sum \\)・\\( \sum(w y)/w\_sum \\) と
  \\( J = \sum (w/g)\lvert r-G\rvert^2 \\) を算定。`w_sum ≤ 0` は幾何重心・`J = 0` として
  `Some` を入れる（`None`＝未算定と質量 0 を区別するため）。`weight_override` には依存させない。
- **未算定エラー**: `squid-n-job::lumped_mass` は `None` を `JobError::InvalidInput`、
  `squid-n-solver::build_lumped_mass_model` は `SolveError::InvalidInput` として止める。
  設計重量へのフォールバックはしない。同関数の戻り値は `Result<LumpedMassModel, SolveError>`
  へ変更した。
- **J の直接算定**: 3 次元の `StorySpatial.j` は `inertia_t_mm2`、`mass_xy` は
  `center_xy_mm` をそのまま使う。**廃止した比例換算**: 剛床マスター節点の RZ 質量を並進質量に
  合わせて拡げる旧 `floor_j`（`j * story_mass / mt`）は削除した。比例換算が安全側か危険側か
  自明でなく、質量分布の異なる回転半径保存の前提も崩れるため。
- **経路統一**: 質点系の線形 2D/3D・非線形はいずれも同じ `Story::dynamic_mass` を単一情報源と
  する。質量方式（`CorrectedLumped` / `LumpedOnly`）に依らず全量を使う。剛床マスター質量は
  `CorrectedLumped` では `net` 質量（分布質量控除後）のため質点系には使わない。
- `gravity_lcs` の内容だけを質量相当値とする `GravityCasesOnly`（`generate_stories_with_opts`
  の `include_density_self_weight = false`）は契約どおりケース内容を質量相当値とするため、
  常に 7.85 t/m³ へ補正されるわけではない。標準構成では使わない（テスト専用）。

## 検証

- `cargo test -p squid-n-core -p squid-n-load --locked`
- `cargo test -p squid-n-solver -p squid-n-job --locked`
- `cargo test --workspace --locked`
- `cargo clippy --workspace --all-targets --locked -- -D warnings`
- `cargo fmt --all -- --check`
- 追加テスト:
  - `squid-n-load`: 物理質量相当重量が設計重量より小さいこと、質量方式間の一致、
    密度経路と自重同期経路の一致、`weight_override` 不変、3 次元 J の手計算照合。
  - `squid-n-solver`: 未算定時にエラー、`dynamic_mass` 設定時の質量算定。
  - `squid-n-job`: 層質量が `mass_equiv_weight_n/g` になること、未算定エラー、
    3 次元の `mass_xy`・`j` が保存値と一致すること。

## 残課題

- 準備計算前のモデルでは `dynamic_mass` が未算定となり質点系を実行できない（意図どおり）。
  GUI/MCP の導線で階生成を必須化するかは別課題。
- `GravityCasesOnly` は契約どおりケース内容を質量相当値とするため、常に 7.85 t/m³ へ
  補正されるわけではない（本番未使用・テストのみ）。
- CFT 線材の設計重量（DL・地震用重量）が充填コンクリート分を欠く問題は別課題
  （[GitHub Issue #364](https://github.com/hrntsm/Squid-n/issues/364)）。
