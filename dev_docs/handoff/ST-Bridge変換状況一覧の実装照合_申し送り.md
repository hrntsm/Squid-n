# ST-Bridge 要素別 変換状況一覧を実装と照合した件 申し送り

## 1. 目的

利用者ドキュメントの [ST-Bridge 要素別 変換状況一覧](../../docs/model_io/03_ST-Bridge_要素別変換状況.md)
が実装より古くなっていたため、`squid-n-io::stbridge` の実装を読み直して全面的に照合した。

照合したのは以下のファイルで、表の各行はこれらの分岐と対応している。

- `import/parser.rs`（受け付ける要素・属性、未対応要素の警告）
- `import/assemble.rs`（モデル構築、既定値、警告・通知の文面）
- `import/steel.rs`・`import/rebar.rs`・`import/material_std.rs`（形鋼・配筋・グレード名）
- `export.rs`・`section_std.rs`（書き出す要素、断面の型分けとフォールバック）

## 2. 一覧の記述と実装が食い違っていた箇所

| 項目 | 一覧の旧記述 | 実装 |
| --- | --- | --- |
| 断面の表現モード | 「断面形状モード（`Standard`）」「物性モード（`Raw`）」の 2 モードがある前提 | モードはなく、標準スキーマ準拠の 1 形式のみ。`StbSecRaw` は標準要素で表せない断面のフォールバック |
| `StbNode` の座標 | 小文字 `x`/`y`/`z` も受ける | 大文字 `X`/`Y`/`Z` のみ |
| 階の所属節点 | 書き出しは `StbNode@story`（方言）で表す | `StbStory/StbNodeIdList/StbNodeId` で書き出す（方言は廃止） |
| `StbMaterial` | 取り込み・書き出しとも対応 | 取り込みのみ。書き出しは断面のグレード名だけで表す |
| `StbBeam`（小梁） | `StbGirder` と同じ行で大梁として扱う | 二次部材（`SecondaryMemberKind::Joist`）として取り込み、`StbBeam` として書き出す |
| `StbPost`（間柱） | 梁部材として取り込む（情報一部欠落） | 二次部材（`SecondaryMemberKind::Post`）として往復する |
| `StbSlab` | 断面を持たず板厚のみ | 符号・階・板厚・`strength_concrete` を断面として取り込み、床へ割り当てる |
| `StbLoadCase`・`StbNodalLoad` | 取り込み・書き出しとも非対応 | 取り込みは対応（ケース名＋節点荷重 6 成分）。書き出しは非対応 |
| 非一様な鋼断面（`_Taper` / `_Joint`） | 図形を復元できず断面欠落として警告 | `shape_start` などから始端の形鋼を採り、一様断面として近似する |
| `StbSecWall_RC` | 取り込み・書き出しとも対応 | 取り込むのは厚さのみ（符号・階・`strength_concrete` は読まない） |
| 角形鋼管の角部外半径 | 記載なし | `r` 属性を `SteelBox::corner_r` として往復する |

併せて [ST-Bridge 形式（.stb / .xml）](../../docs/model_io/02_ST-Bridge_形式.md) 側も、
階の種別（`kind`）が書き出し専用であること、小梁が対応範囲に入ること、
荷重は取り込みのみ対応であることを直した。

## 3. 一覧に「往復しない」と明記した非対称な箇所

いずれも実装として意図的にそうなっている、または実害が小さいと判断して残しているもの。
利用者から見える挙動なので、一覧の備考へ書いて隠さない方針とした。

- 部材の符号（`name`）: 取り込みで保つのは二次部材のみ。書き出しは `C1`・`G1` の自動命名
- 壁断面の符号・階・コンクリート材料: 書き出しは `strength_concrete` を出すが取り込みで読まない
- 階の種別（`kind`）・スラブの `kind_slab`: 書き出しのみ（取り込みは `kind_slab` を使わず、スラブは常に囲まれた領域にする）
- スラブ断面 `StbSecSlab_RC`: 書き出すのは `StbSlab` を出した領域が参照する断面だけ。版なし床・取り付き領域だけが指す孤立断面は出さない
- 非一様な鋼断面: 始端の形鋼で一様断面に近似するため、中間・終端の形鋼は失われる

## 4. 照合中に見つかった要確認事項

一覧の修正では扱わず、別途判断する。

- **`StbNodalLoad` の属性名**: パーサは小文字の `fx`〜`mz` のみを読む。実 ST-Bridge が
  大文字（`FX` 等）を使うのであれば、他社ファイルの節点荷重が値ゼロで取り込まれる。
  スキーマ（`STBridge_v202.xsd`）で属性名を確認したうえで、必要なら候補キーを増やす。
- **形鋼ライブラリの並び順**: `section_std::steel_rank` の判定文字列が
  `<StbSecFlatBar` / `<StbSecRoundBar` になっているが、実際に出力するタグは
  `<StbSecRoll-FlatBar` / `<StbSecRoll-RoundBar` である。順位が既定の 99 へ落ちるため、
  平鋼と中実丸鋼が混在するモデルでは `StbSecSteel` の子要素がスキーマの sequence 順に
  ならない可能性がある。
- **`import/parser.rs` のコメント**と `import/assemble.rs` の `build_nodes_and_stories` に、
  節点の `story` 属性（Squid 方言）を優先する旨の記述が残っているが、現在の実装は
  `StbStory/StbNodeIdList` だけを見る。
- `stbridge/tests.rs` の `test_standard_mode_steel_column` に「Raw モード（既定）」という
  モード前提のコメントが残っている。

## 5. 一覧を保守するときの見どころ

- 取り込みの可否は `parser.rs` の `start_*` メソッド群が担当タグを返すかどうかで決まる。
  どの分岐にも該当しない要素は `record_unsupported` が拾い、部材・断面・荷重の直属子で
  あれば未知の要素でも警告に出る（fail-loud）。
- 書き出しの可否は `export.rs` の `members_body` と `section_std.rs` の
  `standard_sections` を見る。標準要素へ落ちない断面は `raw()` に行き着く。
- 既定値は `assemble.rs`（支点の自動設定・材料区分の推定）と `rebar.rs`（無筋相当の既定配筋）、
  `parser.rs`（端部接合条件・`rotate`・`feature_brace`）に散っている。
  `docs/` は既定値をすべて書く方針なので、この 3 ファイルを変えたら一覧の
  「取り込み時の既定値」を見直す。
