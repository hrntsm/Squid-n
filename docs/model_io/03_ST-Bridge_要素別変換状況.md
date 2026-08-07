# ST-Bridge 要素別 変換状況一覧

ST-Bridge の主要要素ごとの変換状況です。凡例: **✅ 対応** ／ **⚠️ 一部・近似** ／ **❌ 非対応**。
「取り込み」は他社ファイルを読めるか、「書き出し」は Squid-n が出力するか、「往復・備考」は
`import→export→再import` での保存性と注意点を示します。断面（形鋼・RC 等）は**断面形状モード**（`Standard`）での状況ですが、物性モード（`Raw`）は全断面が `StbSecRaw` で完全一致往復します。

対応範囲の考え方と非対応項目の扱いは [ST-Bridge 形式（.stb / .xml）](./02_ST-Bridge_形式.md)を参照してください。
本表は要素の単位でまとめたものです。属性の単位では、取り込んだファイルに存在した属性の扱いを
すべて取り込み時に報告します（[属性の扱いの報告](./02_ST-Bridge_形式.md#属性の扱いの報告)）。

## 節点・層・材料

| ST-Bridge 要素 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| `StbNode`（座標・所属層） | ✅ | ✅ | 座標（小文字 `x/y/z`・大文字 `X/Y/Z` 双方可）。拘束・質量は対象外 |
| `StbStory`（名称・標高） | ✅ | ✅ | — |
| `StbStory/StbNodeIdList/StbNodeId`（階の所属節点） | ✅ | ⚠️ | 取り込みで `Node.story`・`Story.node_ids` へ反映。書き出しは `StbNode@story` 属性（方言）で表現 |
| `StbMaterial`（E・ν・密度・Fc・Fy） | ✅ | ✅ | 材料の種別・規格名は `name` のみ（型分けは非対応） |
| 部材の `id_material`（材料参照） | ⚠️ | ❌ | 材料は断面が持つため、取り込みでは参照先の断面へ移す（同じ断面を指す部材が別々の材料を指す場合は最初の 1 件）。書き出しは断面のグレード名で表す |

## 部材

| ST-Bridge 要素 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| `StbColumn`（柱） | ✅ | ✅ | 鉛直材として往復。端部の偏心 `offset_*` は対象外 |
| `StbGirder` / `StbBeam`（大梁・小梁） | ✅ | ✅ | 水平材として往復。端部の偏心 `offset_*` は対象外 |
| `StbPost`（間柱） | ⚠️ | ⚠️ | 梁部材として取り込む（間柱の別種別がなく情報一部欠落） |
| `StbBrace`（ブレース） | ✅ | ✅ | `tension_only` 含む。取り込み時は両端ピン既定 |
| `StbSlab`（スラブ） | ✅ | ✅ | 境界節点ループ（`StbNodeIdOrder`・子要素 `StbNodeId`・CDATA 可）＋厚さ。荷重・用途・分配法は対象外 |
| `StbWall`（壁） | ✅ | ✅ | 境界節点ループ＋厚さ＋材料。開口（`StbOpen`）は対象外 |
| `StbFooting` / `StbPile` / `StbFoundationColumn` / `StbStripFooting`（基礎系） | ❌ | ❌ | 取り込み時に警告 |
| `StbParapet` / `StbOpen`（パラペット・開口） | ❌ | ❌ | 取り込み時に警告 |

## 断面 — 鋼（形鋼ライブラリ `StbSecSteel`）

`StbSecColumn_S` / `StbSecBeam_S` / `StbSecBrace_S` が形鋼図形を参照します。

| 形鋼要素 | 取り込み | 書き出し | 内部形状・備考 |
|---|:--:|:--:|---|
| `StbSecRoll-H` / `StbSecBuild-H` | ✅ | ✅ | H 形鋼（`SteelH`） |
| `StbSecBuild-H`（上下フランジ相違） | ✅ | ✅ | 非対称組立 H（`SteelBuiltH`）。下フランジは方言属性 `B2`/`t2_lower`。第三者は上フランジの対称 H として読む |
| `StbSecRoll-BOX` / `StbSecBuild-BOX` | ✅ | ✅ | 角形鋼管（`SteelBox`） |
| `StbSecPipe` / `StbSecRoll-Pipe` / `StbSecBuild-Pipe` | ✅ | ✅ | 鋼管（`SteelPipe`）。書き出しは `StbSecPipe` |
| `StbSecRoll-L` | ✅ | ✅ | 山形鋼（`SteelAngle`） |
| `StbSecRoll-C` | ✅ | ✅ | 溝形鋼（`SteelChannel`） |
| `StbSecRoll-T` / `StbSecBuild-T` | ✅ | ✅ | T 形鋼（`SteelTee`） |
| `StbSecRoll-FlatBar` | ✅ | ✅ | 平鋼・鋼板（`SteelFlatBar`） |
| `StbSecRoll-RoundBar` | ✅ | ✅ | 中実丸鋼（`SteelRoundBar`） |
| `StbSecRoll-LipC` | ✅ | ✅ | リップ溝形鋼（`SteelLipChannel`）。幅厚比・部材ランク検定は対象外 |
| 組立断面（2L・2C・十字）・リップ Z・その他軽量形鋼 | ❌ | ❌ | 未対応。参照解決できず物性ゼロ／断面欠落として警告 |
| テーパ・非一様鋼（`_NotSame` / `_Taper` / `_Joint`） | ❌ | ❌ | 図形を復元できず断面欠落として警告 |

## 断面 — RC・SRC・CFT

| ST-Bridge 要素 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| `StbSecColumn_RC`（`_Rect` / `_Circle`） | ✅ | ✅ | RC 矩形・円形柱（`RcRect`/`RcCircle`）＋配筋 |
| `StbSecBeam_RC`（`_Straight`） | ✅ | ✅ | RC 矩形梁＋配筋。円形梁は ST-Bridge に図形がなく物性へフォールバック |
| `StbSecBarArrangement*`（配筋） | ⚠️ | ✅ | 主筋（本数・径・段数、段別本数の合算）・帯筋・かぶりを best-effort。呼び名径 `D22` 可。実スキーマ完全準拠は今後の課題 |
| `StbSecColumn_CFT`（＋充填鋼管） | ✅ | ⚠️ | CFT 角形・円形（`CftBox`/`CftPipe`）。**柱のみ**。梁に使うと物性（`StbSecRaw`）へ |
| `StbSecColumn_SRC` / `StbSecBeam_SRC` | ✅ | ✅ | SRC 矩形（`SrcRect`）＋内蔵鉄骨＋配筋＋鋼種 `strength_steel` |
| RC の T 形・L 形梁、テーパ・ハンチ | ❌ | ❌ | 図形を復元できず断面欠落として警告 |
| `StbSecFoundation_RC` / `StbSecPile_*` / `StbSecParapet_RC` / `StbSecOpen_RC` | ❌ | ❌ | 取り込み時に警告 |

## 断面 — スラブ・壁

| ST-Bridge 要素 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| `StbSecSlab_RC`（厚さ） | ✅ | ✅ | 図形要素（`StbSecSlab_RC_Straight` 等）から厚さ（`depth`）を取得 |
| `StbSecSlabDeck`（デッキ合成） | ✅ | ⚠️ | 図形（`StbSecSlabDeckStraight`）からコンクリート部せいを厚さとして取得。書き出しは `StbSecSlab_RC` 相当 |
| `StbSecWall_RC`（厚さ） | ✅ | ✅ | 同上 |
| `StbSecSlab_S`（鋼スラブ） | ❌ | ❌ | 取り込み時に警告 |

## 荷重・その他

| ST-Bridge 要素 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| `StbLoadCase` / `StbNodalLoad`（節点荷重） | ❌ | ❌ | 荷重は `StbModel`（幾何）ではなく `StbCalData`/`StbAnaModels` に属するため対象外 |
| `StbAxes` > `StbParallelAxes`（平行芯） | ✅ | ✅ | グループの原点・方向角、通り名・離れ・所属節点が往復（[通り芯](../model_edit/01_通り芯.md)） |
| `StbAxes` > 円弧芯・放射芯・作図芯 | 🔶 | ❌ | 通り名と所属節点のみ取り込む（幾何は保持しない）。書き出しは平行芯のみで、除いた旨を通知 |
| 拘束条件（支点）・質量 | ❌ | ❌ | ST-Bridge の幾何スコープ外 |
