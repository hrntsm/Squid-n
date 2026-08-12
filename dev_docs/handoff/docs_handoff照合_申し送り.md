# docs / handoff 照合で判明した実装残課題 申し送り

作成日: 2026-08-12
対象: 利用者向け `docs/` と `dev_docs/handoff/` を実装コードと突合した結果のうち、
**ドキュメント修正では解消せず実装側の是正が必要な項目**。

本申し送りは監査スナップショットである。既存申し送りと重複する項目は出典を併記し、
新規に明示した項目だけを「新規」とする。

## 1. 新規に明示した実装課題

### 1.1 保有水平耐力 τu への開口低減 r2 未配線（危険側）— **修正済み**

- **内容**: `holding.rs` が `area = thickness * wall_len * r2` で τu を算定する。
  r2 は `WallPanelElement::opening_strength_reduction`（Qu と同一の開口寸法）。
- **出典**: [耐震壁_保有水平耐力連携_申し送り.md](耐震壁_保有水平耐力連携_申し送り.md) §1。
- **利用者 docs**: `docs/calc_basis/07_二次設計/03_部材ランク.md` を現仕様に更新済み。

### 1.2 壁式構造列の情報源が二重

- **内容**: 設計タブ `App::wall_structure` と、準備計算が集計する `Story::structure` が
  別系統。保有水平耐力は前者のみ参照。
- **出典**: 同申し送り §2（更新済み）。

### 1.3 ST-Bridge 取り込み後の `Slab::joists` 未設定 — **修正済み**

- **内容**: 小梁要素自体は取り込むが、床スラブの `joists` メタデータは空のまま
  （床荷重二重計上を避ける）。`floor_design_checks` が `secondary_members` の
  小梁を断面検定するよう修正した。
- **根拠**: `crates/squid-n-app/tests/full_model.rs` の
  `joist_design_checks_cover_imported_secondary_members`。
- **出典**: [実モデル統合テスト_申し送り.md](実モデル統合テスト_申し送り.md) §4.3。
- **利用者 docs**: `docs/model_io/03_ST-Bridge_要素別変換状況.md` を現仕様に更新済み。

### 1.4 準備計算 CSV と画面の差

- **内容**: `PreparationResult` に `torsion_skipped`・`panels` があるが、
  `build_preparation_csv` の出力対象外。
- **対応方針候補**: CSV 拡張、または画面専用であることの docs 明記（後者は
  `docs/preparation/11_出力と保存.md` で対応済み）。実装拡張は任意。

### 1.5 診断 stale と準備計算サマリ件数 — **修正済み**

- **内容**: `refresh_preparation` は `diagnostics_stale` に依らず `run_diagnostics` を呼ぶ。
  準備計算タブの診断件数は常に再集計される。

## 2. 既存申し送りどおり未着手（監査で再確認）

| 項目 | 出典 |
|------|------|
| 壁式構造列の情報源が二重 | 耐震壁 §2 |
| ST-Bridge `Node::story` が準備計算で上書き | 階を床レベル基準 §残 |
| 壁 Qu の σwh=295 固定 | 材料を断面へ移す §残（実装はせん断補強筋材料を参照。未割当時のみ 295） |
| `SetSectionMaterial` の材料存在検証なし | 同（`material_ref_ok` で検証済み） |
| `Model::slab_thickness` 廃止 | スラブへ断面 §残 |
| CMQ 図のデータ供給（`beam_loads` 未接続） | 申し送り.md / 残課題一覧 |
| H 形 κ 不一致・shear_area_2d・山形鋼 Iyz・SRC mCd | クレート横断 §残 |

## 3. ドキュメント側で今回対応したもの（実装変更なし）

- 計算根拠の誤パス・陳腐化行番号（`steel_height_ratio`、`material_strength/`、履歴則、減衰既定など）
- 主筋「SD345 既定」の誤記を 6.1 と整合
- 準備計算ファイル番号の重複（`06_` が二つ）を解消
- 耐力壁 τu / 壁式構造の現状を利用者 docs に明記
- handoff 索引の残課題区分（非線形 TH・剛域長）を本文に合わせて更新

## 参照

- 一次監査: Composer 分担（calc_basis / UI・IO docs / handoff）
- 二次レビュー: 同セッション内の再照合
