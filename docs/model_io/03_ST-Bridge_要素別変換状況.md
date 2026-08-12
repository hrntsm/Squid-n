# ST-Bridge 要素別 変換状況一覧

ST-Bridge の主要要素ごとの変換状況です。
凡例: **✅ 対応** ／ **⚠️ 一部・近似** ／ **❌ 非対応**。
「取り込み」は他社ファイルを読めるか、「書き出し」は Squid-n が出力するか、「往復・備考」は
`import→export→再import` での保存性と注意点を示します。

対応範囲の考え方と非対応項目の扱いは [ST-Bridge 形式（.stb / .xml）](./02_ST-Bridge_形式.md)を参照してください。
本表は要素の単位でまとめたものです。
属性の単位では、取り込んだファイルに存在した属性の扱いを
すべて取り込み時に報告します（[属性の扱いの報告](./02_ST-Bridge_形式.md#属性の扱いの報告)）。

書き出しは ST-Bridge 2.0.2 標準スキーマ準拠の 1 形式だけで、断面の表現を選ぶモードはありません。
標準要素で表せない断面だけが、物性を直接持つ拡張要素 `StbSecRaw` へ落ちます。

## 節点・階・材料

| ST-Bridge 要素 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| `StbNode`（座標） | ✅ | ✅ | 座標は標準スキーマの大文字 `X`/`Y`/`Z`。書き出しの `kind` は `ON_GRID` 固定（所属部材から種別を決められないため）。拘束・質量は対象外 |
| `StbStory`（名称・標高） | ✅ | ✅ | 取り込みでは標高（`height`）の昇順へ並べ替えて階を作る。階の種別（`kind`）は書き出しのみで、取り込みでは読まないため往復しない |
| `StbStory/StbNodeIdList/StbNodeId`（階の所属節点） | ✅ | ✅ | 標準スキーマのまま往復する |
| 断面のグレード名（`strength_main`・`strength_concrete`・`strength_steel` ほか） | ✅ | ✅ | 材料の実体はこれで往復する。取り込みでは名前から[標準材料一覧](../calc_basis/02_材料/04_標準材料一覧.md)で E・ν・密度・Fc・Fy を復元する |
| `StbMaterial`（E・ν・密度・fc・fy） | ✅ | ❌ | ST-Bridge 2.0 の `StbModel` は材料表を持たないため、書き出しでは出力しない。材料表を持つファイルを読んだときに物性を捨てないよう、取り込みだけ受け付ける |
| 部材の `id_material`（材料参照） | ⚠️ | ❌ | 材料は断面が持つため、取り込みでは参照先の断面へ移す（同じ断面を指す部材が別々の材料を指す場合は最初の 1 件を採り、件数を警告へ出す）。書き出しは断面のグレード名で表す |

## 部材

| ST-Bridge 要素 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| `StbColumn`（柱） | ✅ | ✅ | 鉛直材として往復。`rotate`・`condition_bottom`/`_top` を読む。端部の偏心 `offset_*` は対象外 |
| `StbGirder`（大梁） | ✅ | ✅ | 水平材として往復。`rotate`・`condition_start`/`_end` を読む。端部の偏心 `offset_*` は対象外 |
| `StbBeam`（小梁） | ✅ | ✅ | 二次部材として往復する。全体解析の対象外で、床荷重と自重は大梁への集中荷重（CMQ）として伝える。床スラブの小梁一覧には載せない。断面検定は [1.4.3 交差小梁の床格子](../calc_basis/01_荷重/03_床荷重の分配.md#143-交差小梁の床格子サブストラクチャ解析床-phase-f) |
| `StbPost`（間柱） | ✅ | ✅ | 二次部材の間柱として往復。節点は `id_node_bottom`/`_top`（`id_node_start`/`_end` も可） |
| `StbBrace`（ブレース） | ✅ | ✅ | `feature_brace` を読み、`TENSIONANDCOMPRESSION` 以外は引張専用とする。両端ピンで取り込む |
| `StbSlab`（スラブ） | ✅ | ✅ | 境界節点ループ（`StbNodeIdOrder` のテキスト・CDATA・子要素 `StbNodeId` のいずれも可）＋断面参照。`kind_slab` は書き出しのみ。仕上げ荷重・用途（積載）・分配法は対象外 |
| `StbWall`（壁） | ✅ | ✅ | 境界節点ループ＋断面参照（厚さ）。`id_material` は断面へ移す。開口（`StbOpen`）は対象外 |
| 部材の符号（`name`） | ⚠️ | ⚠️ | 取り込みで符号を保つのは二次部材（小梁・間柱）のみ。書き出しは `C1`・`G1`・`BR1` のような自動命名になるため、符号は往復しない |
| `StbFooting` / `StbPile` / `StbFoundationColumn` / `StbStripFooting`（基礎系） | ❌ | ❌ | 取り込み時に警告 |
| `StbParapet` / `StbOpen`（パラペット・開口） | ❌ | ❌ | 取り込み時に警告 |

境界節点が解決できないスラブ・壁と、頂点が 3 つに満たないスラブ・壁は、取り込まずに件数を警告へ出します。

## 断面 — 鋼（形鋼ライブラリ `StbSecSteel`）

`StbSecColumn_S` / `StbSecBeam_S` / `StbSecBrace_S` が形鋼図形を参照します。
書き出しでは、柱の断面が `StbSecColumn_S`、大梁とブレースの断面が `StbSecBeam_S` になります。
ブレース専用の型を使わないのは、断面の内容が梁用と同じで、型を分けても情報が増えないためです。
二次部材（小梁・間柱）だけが使う断面と、どの部材からも参照されない断面は、柱用として書き出します。

| 形鋼要素 | 取り込み | 書き出し | 内部形状・備考 |
|---|:--:|:--:|---|
| `StbSecRoll-H` / `StbSecBuild-H` | ✅ | ✅ | H 形鋼（`SteelH`）。書き出しは `StbSecRoll-H` |
| `StbSecBuild-H`（上下フランジ相違） | ✅ | ✅ | 非対称組立 H（`SteelBuiltH`）。下フランジは方言属性 `B2`/`t2_lower`。第三者は上フランジの対称 H として読む |
| `StbSecRoll-BOX` / `StbSecBuild-BOX` | ✅ | ✅ | 角形鋼管（`SteelBox`）。角部外半径 `r` も往復する（属性がなければ角部直角として 0） |
| `StbSecPipe` / `StbSecRoll-Pipe` / `StbSecBuild-Pipe` | ✅ | ✅ | 鋼管（`SteelPipe`）。書き出しは `StbSecPipe` |
| `StbSecRoll-L` | ✅ | ✅ | 山形鋼（`SteelAngle`） |
| `StbSecRoll-C` | ✅ | ✅ | 溝形鋼（`SteelChannel`） |
| `StbSecRoll-T` / `StbSecBuild-T` | ✅ | ✅ | T 形鋼（`SteelTee`） |
| `StbSecRoll-FlatBar` | ✅ | ✅ | 平鋼・鋼板（`SteelFlatBar`） |
| `StbSecRoll-RoundBar` | ✅ | ✅ | 中実丸鋼（`SteelRoundBar`）。直径 `D` がなければ半径 `R` を 2 倍する |
| `StbSecRoll-LipC` | ✅ | ✅ | リップ溝形鋼（`SteelLipChannel`）。幅厚比・部材ランク検定は対象外 |
| 組立断面（2L・2C・十字）・リップ Z・その他軽量形鋼 | ❌ | ❌ | 形鋼参照を解決できず、断面性能ゼロの断面として警告する |

鋼断面の図形参照（`StbSecSteelColumn_S_Same`・`StbSecSteelBeam_S_Straight` など）は、
`shape`・`shape_start`・`shape_center`・`shape_main` の順に見て、最初に見つかった形鋼名を採ります。

| 図形参照 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| 一様断面（`*_Same` / `*_Straight`） | ✅ | ✅ | 書き出しはこの形だけを使う |
| テーパ・継手（`*_Taper` / `*_Joint` ほか始端を持つ図形） | ⚠️ | ❌ | 始端の形鋼を採り、材長方向に一様な断面として近似する。中間・終端の形鋼は取り込まない |
| 上記 4 つの属性をいずれも持たない図形（柱の `*_NotSame` など） | ❌ | ❌ | 形鋼名を取れず、断面性能ゼロの断面として警告する |

## 断面 — RC・SRC・CFT

| ST-Bridge 要素 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| `StbSecColumn_RC`（`_Rect` / `_Circle`） | ✅ | ✅ | RC 矩形・円形柱（`RcRect`/`RcCircle`）＋配筋 |
| `StbSecBeam_RC`（`_Straight`） | ✅ | ✅ | RC 矩形梁＋配筋。円形梁は ST-Bridge に図形がなく `StbSecRaw` へフォールバック |
| `StbSecBarArrangement*`（配筋） | ⚠️ | ✅ | 主筋（本数・径・段数）・帯筋・あばら筋・かぶりを best-effort で取り込む。詳細は下記 |
| `StbSecColumn_CFT`（＋充填鋼管） | ✅ | ⚠️ | CFT 角形・円形（`CftBox`/`CftPipe`）。**柱のみ**。梁に使うと `StbSecRaw` へ |
| `StbSecColumn_SRC` / `StbSecBeam_SRC` | ✅ | ✅ | SRC 矩形（`SrcRect`）＋内蔵鉄骨（H 形鋼）＋配筋＋鋼種 `strength_steel` |
| `StbSecRaw`（物性直持ちの拡張要素） | ✅ | ✅ | 標準要素で表せない断面のフォールバック。他ソフトは解釈できないが、参照する部材の断面リンクは保たれる |
| RC・SRC のテーパ・ハンチ等（矩形・円形以外の図形） | ❌ | ❌ | 図形を認識できず、断面を取り込めなかったものとして警告する |
| `StbSecFoundation_RC` / `StbSecPile_*` / `StbSecParapet_RC` / `StbSecOpen_RC` | ❌ | ❌ | 取り込み時に警告 |

配筋の取り込みは、実ファイルで使われる属性名の揺れを吸収するようにしています。

- 主筋の本数は、段別の本数（`N_main_X_1st`・`N_main_top_1st` など 1〜3 段目）を合算して総本数とする
- 段数は明示の属性があればそれを採り、なければ本数が 0 でない段を数える
- 主筋の径は `D_main` などから採り、`D22` のような呼び名も数値 22 として読む
- かぶりは配筋の子要素になければ、配置コンテナ（`StbSecBarArrangement*`）の `depth_cover_*` を採る
- 材質は主筋が `strength_main`、せん断補強筋が `strength_band`・`strength_stirrup`

ST-Bridge の主筋径は `D_main` の 1 種類だけなので、X 方向と Y 方向で径を変えた配筋は往復しません。
書き出しでは 1 段の配筋へ丸めるため、多段配筋も段数を保てません。

## 断面 — スラブ・壁

| ST-Bridge 要素 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| `StbSecSlab_RC`（厚さ・符号・階・コンクリート） | ✅ | ✅ | 厚さ（`depth`）は図形要素（`StbSecFigureSlab_RC` > `StbSecSlab_RC_Straight`）から取る。符号 `name`・階 `floor`・`strength_concrete` も断面として取り込む |
| `StbSecSlabDeck`（デッキ合成） | ✅ | ⚠️ | 図形（`StbSecSlabDeckStraight`）からコンクリート部せいを厚さとして取る。書き出しは `StbSecSlab_RC` 相当 |
| `StbSecWall_RC`（厚さ） | ⚠️ | ✅ | 取り込むのは厚さのみで、符号・階・`strength_concrete` は読まない。書き出しは壁要素ごとに 1 件、符号は `W1` のような自動命名 |
| `StbSecSlab_S`（鋼スラブ） | ❌ | ❌ | 取り込み時に警告 |

スラブの断面は符号＋階で識別するため、同じ断面を参照する床が何枚あっても断面は 1 件です。
床の自重は面荷重へ焼き込まず、断面の板厚とコンクリート材料から使うたびに算定します
（[床の断面と自重](../calc_basis/01_荷重/08_床の断面と自重.md)）。

壁の断面は厚さごとに 1 件だけ作り、符号は `Wall t180` のように厚さから決めます。
壁の材料は取り込みでは `StbWall` の `id_material` から解決するため、書き出した
`strength_concrete` を読み戻す経路はなく、材料は往復しません。

## 荷重・その他

| ST-Bridge 要素 | 取り込み | 書き出し | 往復・備考 |
|---|:--:|:--:|---|
| `StbLoadCase` / `StbNodalLoad`（荷重ケース・節点荷重） | ✅ | ❌ | 荷重ケースの名称と、節点荷重の 6 成分（`fx`・`fy`・`fz`・`mx`・`my`・`mz`。欠けている成分は 0）を取り込む。荷重は `StbModel`（幾何）の外なので書き出さない |
| `StbLoadCase` 直下のほかの荷重要素（`StbLoadMember` ほか） | ❌ | ❌ | 取り込み時に警告 |
| `StbAxes` > `StbParallelAxes`（平行芯） | ✅ | ✅ | グループの原点・方向角、通り名・離れ・所属節点が往復（[通り芯](../model_edit/01_通り芯.md)） |
| `StbAxes` > 円弧芯・放射芯・作図芯 | ⚠️ | ❌ | 通り名と所属節点のみ取り込む（幾何は保持しない）。書き出しは平行芯のみで、除いた旨を通知 |
| 拘束条件（支点） | ❌ | ❌ | ST-Bridge の幾何スコープ外。支点を持たないモデルは取り込み時に自動設定する（下記） |
| 質量 | ❌ | ❌ | ST-Bridge の幾何スコープ外 |
| `StbCommon` | ❌ | ✅ | 書き出しのみ。プロジェクト名・アプリ名は `Squid-n` 固定 |
| `StbJoints` | ❌ | ✅ | 接合部は扱わないため、書き出しでは空要素だけを出す |

## 取り込み時の既定値

ファイルに属性がないときの扱いをまとめます。
解析結果に直接効く最後の 2 つは、既定を採ったことを取り込み報告の警告・通知でも知らせます。

- 部材端の接合条件（`condition_*`）がない場合は剛接合（`FIX`）とする
- 断面の回転角（`rotate`）がない場合は 0 とする
- ブレースの `feature_brace` がない場合は引張専用とする
- 断面参照（`id_section`）が `-1` または未指定の場合は、断面を持たない部材として取り込む
- 配筋を持たない RC・SRC 断面は、本数・径・ピッチをすべて 0 とした無筋相当の配筋で補う
- 形鋼参照を解決できない鋼・CFT・SRC 断面は、断面性能を 0 とした断面として残す
- 支点を 1 つも持たないモデルは、最下レベルで柱脚が付く節点をピン支点にする
  （[支点の自動設定](./02_ST-Bridge_形式.md#支点の自動設定取り込み時)）
