# 鋼材の設計重量と物理質量を分離する

Status: accepted

## 決定

鋼材の単位体積重量を用途で分離する。

- **設計重量（DL 荷重・地震用重量）**: 設計用単位体積重量 γs = 78.5 kN/m³。
  基準資料の単位体積重量表（鉄骨）による。
- **物理質量（質量行列・動的解析）**: 物理質量密度 ρ = 7.85 t/m³。
  質量密度 `Material.density` はこの物理質量密度のまま（鋼材 7.85e-9 t/mm³）とし、
  設計重量を密度から導出しない。
- **数量積算**: 積算の慣用値 7.85 t/m³。物理質量密度と値は同じだが用途は独立で、
  独立の定数として扱う。

`CorrectedLumped` と `LumpedOnly` の公称並進総動的質量が一致するよう、各自重は
「設計重量 `load`」と「物理質量相当の重量 `mass_equiv`」の 2 値を持つ。動的質量は
`Σ mass_equiv / g` に一致し、地震用重量（設計重量）とは一致しない。

壁・二次部材・床のコンクリートは設計単位体積重量が密度×g に等しいため、設計重量と
物理質量相当は一致する。鉄骨重量割増の増分・仕上げ・付加線重量など質量行列に対応物が
ない付加重量は、質量の 2 値でも同値として質点に残す。

## 背景と理由

基準資料（PDF p.11 表2.1.1-1）は鉄骨の単位重量 γ = 78.5 kN/m³ を与えるのみで、
質量・密度・g 換算には言及しない。従来は固定荷重の γs = 77 kN/m³ から質量密度を
g で除して導出していたため、基準資料の 78.5 と一致せず、また設計重量と動的質量が
暗黙に同じ値で扱われていた。

78.5 kN/m³ を g で換算すると約 8.005 t/m³ となり、物理質量密度 7.85 t/m³ とは
一致しない。設計重量を過小評価しないこと（安全側）と、物理的に正しい質量で動的解析を
行うことを両立するため、用途を分離する。78.5 は基準資料由来、7.85 による動的質量と
両質量方式の一致は Squid-n 側の設計判断である。

鋼材の設計単位体積重量は材料の区分（`MaterialCategory::Steel`）から解決し、
`Material.fc` の有無では判定しない（RC/SRC の鉄骨内蔵材など、fc の有無と区分が
一致しない場合があるため）。鉄筋は鋼材に含めない。RC/SRC の主材料はコンクリートで
あり、鉄筋の自重は γRC/γSRC に内包されるため別加算しない。

## 影響

- 鋼材を含む DL 荷重・地震用重量が増える（78.5/77.0 ≈ +1.9%。従来の質量換算比
  7.85 t/m³ との比では 78.5/76.98 ≈ +2.0%）。
- 固有値解析の動的質量は物理密度 7.85 t/m³ ベースのままで、`CorrectedLumped` と
  `LumpedOnly` の総動的質量が一致する。
- 設計重量と動的質量は一致しない。利用者向け計算根拠に両者の用途分離を明記する。
- 数量積算（7.85 t/m³）は不変。
- プリセット・ST-Bridge 取込・UI 既定の鋼材・鉄筋密度は 7.85e-9 t/mm³ になる。
- 質点系解析 `crates/squid-n-job/src/lumped_mass.rs`（`seismic_weight/g` を使用）は
  本決定の対象外であり、物理質量へ切り替えていない（残課題）。

## 関連

- `crates/squid-n-core/src/units.rs`
- `crates/squid-n-core/src/model/material.rs`
- `crates/squid-n-load/src/story_gen/self_weight_calc.rs`
- `crates/squid-n-load/src/story_gen/generate.rs`
- `crates/squid-n-load/src/cascade.rs`
- `crates/squid-n-load/src/wall_plate_load.rs`
- `crates/squid-n-load/src/floor/joist_design.rs`
- `docs/calc_basis/01_荷重/05_地震用重量_層データの生成.md`
- `docs/calc_basis/02_材料/04_標準材料一覧.md`
- `docs/calc_basis/05_構造解析/03_固有値解析.md`
- `docs/calc_basis/10_数量積算/01_数量積算の算定式.md`
