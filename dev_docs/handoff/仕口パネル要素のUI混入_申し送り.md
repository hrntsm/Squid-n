# 自動生成した仕口パネル要素の UI 混入 申し送り

作成日: 2026-08-05
対象コード:
`crates/squid-n-core/src/model/element.rs`、
`crates/squid-n-solver/src/statics/analysis/precheck.rs`、
`crates/squid-n-app/src/app/actions.rs`、`crates/squid-n-app/src/app/mod.rs`、
`crates/squid-n-app/src/app/panels.rs`、`crates/squid-n-app/src/viewer/mod.rs`、
`docs/preparation/06_仕口パネル.md`、`docs/calc_basis/05_構造解析/02_線形静解析.md`

他社（Super Build/SS7 出力）の ST-Bridge を取り込むと、断面が正しく割り当たっている
純 S 造モデルで「断面が未割当です」の警告が多数出て、ナビゲータでも RC 部材が実在数より
多く計上される、という報告を受けて調査した。

## 調査結果: ST-Bridge 取り込みは正常

報告に使われたファイル（S 造 3 層＋PH、角形鋼管柱 □-400x400x16x56 / □-300x300x12x42、
STKR400）を `import_stbridge_with_report` へ通した結果は次のとおりで、取り込み側に
欠落は無かった。

| 項目 | 結果 |
| --- | --- |
| 要素 | 115 本（柱 40・大梁 75）。断面・材料の未割当 0 |
| 断面 | 30。角形鋼管は `SteelBox`、H 形は `SteelH` として解決 |
| 二次部材・スラブ | 小梁 56 本・スラブ 82 枚。いずれも断面／厚さの欠落なし |
| `ImportReport.warnings` | 0 件 |
| 構造種別 | 鉛直材 40 本すべて `S`（RC 誤判定は無い） |

つまり「読み込めていない」のではなく、**準備計算が自動生成する仕口パネル要素
（`ElementKind::PanelZone`）を、実部材向けの UI がそのまま拾っていた**のが原因。
このモデルでは 42 箇所にパネルが生成され、要素 ID #115〜#156 として要素配列の末尾に
追加される。報告時のスクリーンショットの「診断 (E0/W42)」「部材 #115〜」と一致する。

## 症状と対応（3 か所）

### 1. 「診断」タブの断面未割当警告（`run_diagnostics`）

- **変更前**: 要素種別で絞らず `e.section.is_none()` を拾っていたため、生成された
  パネル 42 本がそのまま警告になった。すべての部材に断面が割り当たっているのに
  警告が 42 件並び、本当に割当が漏れた部材があっても埋もれる。
- **変更後**: `ElementKind::requires_section_and_material()` で対象を絞る。

### 2. ナビゲータの部材グループ（`navigator_panel`）

- **変更前**: `elem_is_steel` の真偽で 2 分し、偽をすべて「RC部材」としていた。
  パネルは材料を持たず `member_structure_kind` が RC を返すため、S 造の建物で
  「鋼材部材 (98) / RC部材 (59)」と表示された（59 = RC 基礎大梁 17 ＋ パネル 42）。
- **変更後**: 振り分けを `App::member_material_groups` へ切り出し、材料を持つ要素
  （同じ `requires_section_and_material()`）だけを対象にした。

### 3. 3D ビューの選択ハイライト

- **変更前**: 選択要素の先頭 2 節点を無条件に赤線で結んでいた。パネルの節点列は
  「接合部の節点 ＋ 取り付く部材の他端」なので、赤線が取り付く柱・梁とまったく同じ
  線分になる。「RC部材」を選ぶと全柱が赤くなり、**柱が RC と誤判定されているように
  見えた**（実際の判定は S で正しい）。
- **変更後**: 描き方を `viewer::element_draw_shape`（`DrawShape::Line` / `Polygon` /
  `None`）へ集約した。線材＝材軸の線分、面要素（壁・シェル）＝節点列を閉じた輪郭、
  仕口パネル＝描かない。面要素は変更前まで先頭 2 節点の 1 辺だけが光っていたが、
  輪郭表示にして選択範囲がわかるようにした。
  なお 3D ビューには「部材線を描くか（`draws_as_line`）」「面要素をポリゴンで描くか
  （インラインの `matches!(Wall | Shell)`）」という同じ分類が別々に存在していたため、
  これらも `element_draw_shape` から導く形に統一した（`draws_as_line` は
  `== DrawShape::Line` の薄いラッパ）。

## 判定の単一情報源化

「断面・材料の割当が必須な要素種別か」は `precheck_model` がインラインの `matches!` で
持つだけで、診断・ナビゲータは各々の書き方をしていた。`ElementKind` へ
`requires_section_and_material()` を追加し、`precheck_model` を含む全箇所がこれを呼ぶ形に
統一した。要素種別の追加時に扱いを決め忘れないよう、実装はワイルドカードを使わない
網羅 `match` とし、新種別はコンパイルエラーになる。

| 種別 | 断面・材料 | 理由 |
| --- | --- | --- |
| `Beam` / `Fiber` / `MultiSpring` / `Brace` / `Shell` / `Wall` | 必須 | 断面諸元と材料定数から剛性を作る |
| `PanelZone` | 不要 | 取り付く柱・梁の断面から求めた実効体積 Ve による |
| `NodalSpring` | 不要 | `ElementData::spring`（局所軸 6 成分） |
| `Isolator` / `Damper` | 不要 | `Model::isolator_attrs` / `Model::damper_attrs` |

## 回帰テスト

| テスト | 内容 |
| --- | --- |
| `model::tests::test_requires_section_and_material_covers_line_and_area_elements` | 線材・面材が真 |
| `model::tests::test_requires_section_and_material_excludes_property_driven_elements` | パネル・バネ・免震・ダンパーが偽 |
| `app::tests::test_run_diagnostics_ignores_generated_panel_zones` | 準備計算を通した S 造モデルで未割当警告が出ない |
| `app::tests::test_member_material_groups_excludes_generated_panel_zones` | パネル生成後も RC 部材が 0 のまま |
| `viewer::tests::要素の描き方は種別ごとに一意に決まる` | 部材線・ハイライトの描き方の対応表 |

いずれも準備計算（`ensure_preparation`）を実際に通してパネルを生成させたうえで確認して
おり、パネルが 1 つも生成されないモデルで素通りしないよう、生成されたことも表明している。

## 残課題

- 部材タブの部材一覧はパネルを含む全要素を並べ、種別を `{:?}`（`PanelZone`）で表示する。
  自動生成要素であることが一覧からはわからないため、表示名の和訳・自動生成要素の
  折りたたみは今後の検討事項。
