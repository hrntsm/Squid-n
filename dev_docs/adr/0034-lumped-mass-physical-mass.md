# 質点系の層質量を物理質量相当へ統一する

Status: accepted

## 決定

質点系（串団子）の層質量・質量重心・回転慣性 J は、階生成が節点の物理質量相当重量
`node_mass_equiv`（質量方式に依らない**全量**）から算定して
`squid_n_core::model::Story::dynamic_mass` に保存した値を**単一情報源**とする。

- 層質量は `dynamic_mass.mass_equiv_weight_n / g`（`g` は重力加速度）とする。
- 3 次元の質量重心は `dynamic_mass.center_xy_mm`、回転慣性は
  `dynamic_mass.inertia_t_mm2`（質量重心まわり `Σ (w_i/g)|r_i − G|²`）とする。
- 未算定（`None`）は明示エラーとし、設計重量 `seismic_weight` へ**フォールバックしない**。
- 設計重量ベースへ切り替える入口は設けない。

## 背景と理由

従来、質点系の層質量は `Story::seismic_weight / g`（設計地震用重量）で算定していた。
設計地震用重量は鋼材 78.5 kN/m³ ベース（[ADR-0033](0033-steel-design-weight-vs-physical-mass.md)）
であり、フレームの動的解析が用いる物理質量（鋼材 7.85 t/m³ ベース）と食い違っていた。
同じ建物のフレーム固有値と質点系固有値で質量源が異なる状態を解消する。

`Story::dynamic_mass` を階生成（準備計算）が節点の物理質量相当重量から一括算定して
モデルへ保存する。串団子は部材分布質量を持たないため、層質量は質量方式
（`CorrectedLumped` / `LumpedOnly`）に依らず**全量** `Σ node_mass_equiv` を用いる。

既定の `CorrectedLumped` で剛床マスターに与える質点質量は、解析の質量行列へ計上される
分布質量分を控除した `net` 質量である（二重計上防止）。フレームの固有値では控除分が
質量行列から戻るため総動的質量は方式に依らず一致するが、分布質量を持たない質点系で
`net` 質量を使うと総質量を過小評価する。したがって質点系用には全量を別途保存する。

3 次元の回転慣性 J は、従来は剛床マスター節点の RZ 質量を並進質量が地震用重量と
一致するよう回転半径を保って比例換算していた。この換算が安全側か危険側かは自明で
なく、質量の異なる分布を使って回転半径を保つ前提も崩れるため廃止し、質量と同じ
`node_mass_equiv` 分布から質量重心まわりに一貫して直接算定する。

## 実装

- `squid-n-core`: `StoryDynamicMass { mass_equiv_weight_n [N], center_xy_mm [mm],
  inertia_t_mm2 [t·mm²] }` を追加し、`Story::dynamic_mass: Option<StoryDynamicMass>`
  として保持する。`Layer::dynamic_mass` は上端床の値から導出する。`Story` の新フィールドは
  **末尾**に置く（msgpack の位置ずれ防止）。
- `squid-n-load::story_gen::generate`: 階ループで `node_mass_equiv` の全量を集計し、
  `w_sum > 0` なら重心 `Σ(w·x)/w_sum` と `J = Σ(w/g)|r−G|²` を算定する。`w_sum ≤ 0` は
  幾何重心・`J = 0` として `Some` を入れる（`None`＝未算定と質量 0 を区別する）。
  `weight_override` には依存させない。
- `squid-n-job::lumped_mass`: `layer_mass`・`layer_inertia` が `layer.dynamic_mass` を読む。
  未算定は `JobError::InvalidInput`。`build_spatial` の `mass_xy` は `center_xy_mm`、
  `j` は `inertia_t_mm2`。`floor_j`（マスター RZ 質量の比例換算）を削除。
- `squid-n-solver::dynamic::lumped_mass::build_lumped_mass_model`: 層質量を
  `dynamic_mass.mass_equiv_weight_n / g` とし、戻り値を
  `Result<LumpedMassModel, SolveError>` へ変更する。未算定は `SolveError::InvalidInput`。

## 影響

- 質点系の層質量が設計重量ベースから物理質量ベースへ変わる（鋼材で約 2% 減）。
  固有周期はわずかに短く、応答も変化する。フレームの動的質量と整合する。
- 設計地震用重量 `seismic_weight` と質点系の質量は一致しない（用途分離は ADR-0033 と同じ）。
- 準備計算（階生成）を通していないモデルは `dynamic_mass` が未算定となり、質点系を
  実行できない（明示エラー）。設計重量での代用はしない。
- 手入力の地震用重量 `Story::weight_override` は設計地震力のみを変え、質点系の質量は
  変えない。
- 3 次元の J は質量重心まわりに直接算定した値になり、剛床マスターの有無に依存しない。
  従来の `J ≤ 0` で 3 次元質点系を実行できない制約は、質量 0 の階がある場合を除き解消する。

## 関連

- [ADR-0033](0033-steel-design-weight-vs-physical-mass.md) — 鋼材の設計重量と物理質量の分離
- `crates/squid-n-core/src/model/story.rs`
- `crates/squid-n-load/src/story_gen/generate.rs`
- `crates/squid-n-job/src/lumped_mass.rs`
- `crates/squid-n-solver/src/dynamic/lumped_mass/model.rs`
- `docs/calc_basis/05_構造解析/10_質点系解析.md`
- `dev_docs/handoff/質点系の物理質量化_Issue365_申し送り.md`
