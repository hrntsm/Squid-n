# ST-Bridge 形式（.stb / .xml）

[ST-Bridge](https://www.building-smart.or.jp/meeting/buildingsmart/st-bridge/)（XML, 2.0 系）で**読み込み・書き出し**ができ、他社の一貫計算プログラムや BIM ツールとモデルを受け渡すための入出力経路として機能します。

<div class="impl-ref">

**実装参照**：`squid_n_io::stbridge::{import_stbridge_with_report, export_stbridge}`（`crates/squid-n-io/src/stbridge/`）が取り込み・書き出しの入口です。

</div>

## GUI からの操作

| メニュー | 動作 |
|---|---|
| 📥 **ST-Bridge 読込…** | `.stb`（または `.xml`）ファイルを選び、内部モデルへ取り込みます。取り込んだモデルは検証（`validate`）を通ってから現在のモデルと差し替わります |
| 📤 **ST-Bridge 書出…** | 標準 ST-Bridge 2.0.2（`StbSecColumn_S` 等＋形鋼ライブラリ）で書き出す。BIM・他ソフト向け |

- ファイル選択ダイアログの拡張子フィルタは `.stb` / `.xml`。
- ST-Bridge 読込は `.scz` プロジェクトとは別系統であり、読み込んでもプロジェクトの保存先パスは設定されません（新規モデルとして開く扱い）。上書き保存するとネイティブの `.scz` として保存されます。
- 書き出しは **ST-Bridge 2.0.2 標準スキーマ準拠**の 1 形式のみです（他ソフトとの相互運用が目的のため独自方言は用いません）。完全一致の保存にはネイティブの `.scz` を使います。

<div class="impl-ref">

**実装参照**：メニューは `squid_n_app`（`crates/squid-n-app/src/app/mod.rs`）が表示し、取り込みは `squid_n_app::app::App::import_stbridge_from`、書き出しは `squid_n_app::app::App::export_stbridge_to`（`crates/squid-n-app/src/app/actions/io.rs`）が担います。

</div>

## 対応バージョン

- **ST-Bridge 2.0 系のみ**を受け付けます（ルート要素 `ST_BRIDGE` の `version` 属性が `2.` で始まること）。
- 1.x 系や ST-Bridge でない XML は読み込みエラーになります。
- ファイルの文字コードは、BOM 付き UTF-8・UTF-8・Shift_JIS（Windows-31J）の順に判定します。

<div class="impl-ref">

**実装参照**：文字コードは `squid_n_io::stbridge::read_stbridge_file`（`crates/squid-n-io/src/stbridge/import/mod.rs`）が判定し、`version` 属性の検証は `squid_n_io::stbridge::import::parser::parse`（`crates/squid-n-io/src/stbridge/import/parser.rs`）が行います。

</div>

## 対応範囲（意味的往復を保証するサブセット）

読み込み・書き出しの対象は、`import → export → 再 import` でモデルが**意味的に一致する**範囲に限定しています。

要素ごとの詳細な変換状況（取り込み／書き出し／往復・備考）は、
[ST-Bridge 要素別 変換状況一覧](./03_ST-Bridge_要素別変換状況.md)を参照してください。

<div class="impl-ref">

**実装参照**：取り込みと書き出しは `squid_n_io::stbridge::{import, export}`（`crates/squid-n-io/src/stbridge/`）が担い、床板・壁版の床領域／壁領域への付け直しは `squid_n_core::{region_rebuild, wall_region_rebuild}` が行います。

</div>

## 非対応（対象外）

非対応の要素・属性や、Squid-n 独自の解析・設計情報は
入出力の対象に含まれず、読み込み後は既定値になります。ただし、`StbModel`（幾何）の外にある `StbMaterial`、
`StbLoadCase`、`StbNodalLoad` は例外として取り込みます（書き出しは対象外です）。要素ごとの非対応項目と、
取り込み・書き出し・往復の扱いは、[ST-Bridge 要素別変換状況一覧](./03_ST-Bridge_要素別変換状況.md)を参照してください。

- 剛域・製作情報などの詳細属性。

取り込み時にデータを欠落させる要素・属性は、取りこぼしを無言で捨てず `ImportReport` の警告として
通知します。手動リストにない新要素・ベンダー拡張も、部材グループ・断面の直属子であれば要素名で
通知します。GUI の「ST-Bridge 読込」は警告があれば「⚠️ 取り込み時の注意」として表示します。

<div class="impl-ref">

**実装参照**：未対応要素の収集と警告への変換は `squid_n_io::stbridge::import`（`crates/squid-n-io/src/stbridge/import/{parser.rs, assemble.rs}`）が担い、`squid_n_io::stbridge::ImportReport` として返ります。

</div>

### 属性の扱いの報告

要素だけでなく**属性の単位でも、ファイルに存在したものをどう扱ったかをすべて報告します**。
取り込みの報告 `ImportReport` は `attributes` に、要素名・属性名ごとの出現件数と、そのうち
取り込んだ件数を持ちます。

無視リストは持ちません。
`guid` や `app_name` のように解析へ用いない属性も「取り込まなかった属性」として現れますが、
どの属性がどう扱われたかを利用者が漏れなく追えることを優先しています。
リストを持たない副次的な利点として、取り込みの実装を増やせば報告も自動的に追随するため、
一覧の更新漏れが起きません。

GUI の「ST-Bridge 読込」では、扱いの全量を下ドックの「ログ」タブへ出します。
読込直後のメッセージには「取り込まなかった属性が N 種類あります」という要約だけを出します。
実ファイルでは属性の種類が数十になり、そのまま並べると読めなくなるためです。

同じ属性でも文脈により扱いが分かれることがあります。
たとえば 1 つの断面に図形要素が複数ある場合、最初の図形だけを採るため 2 つ目以降の寸法属性は
参照しません。
このときログには「N 件中 M 件を取り込み」と出ます。

<div class="impl-ref">

**実装参照**：属性の出現・取り込み件数は `squid_n_io::stbridge::AttrDisposition` として集計し（`crates/squid-n-io/src/stbridge/import/{parser.rs, assemble.rs}`）、GUI のログ出力は `squid_n_app::app::App::log_attribute_dispositions`（`crates/squid-n-app/src/app/actions/io.rs`）が担います。

</div>

### 支点の自動設定（取り込み時）

ST-Bridge は境界条件（支点）を持たないため、支点が 1 つもないモデルを取り込んだ
ときは、そのままでは解析できません。そこで取り込み時に、**最下レベル（Z 最小、
許容差 1 mm）で柱脚が取り付く節点**をピン支点（並進 3 自由度を拘束・回転自由）に
自動設定します（柱脚ピンの仮定。基礎の回転拘束を期待しない安全側の既定）。柱の判定は鉛直な 2 節点線材（部材軸の鉛直成分が大きい部材）で行い、
柱が取り付かず梁だけが取り付く最下レベル節点（地中梁の中間節点など）は支点にしません。
最下レベルに柱脚が特定できない場合に限り、解析可能性を優先して最下レベルの全節点を
ピン支点にフォールバックします。設定した内容は `ImportReport` の通知（notes）で知らせ、
モデルタブ→境界条件で変更できます。すでに拘束を持つモデルはそのまま尊重し、何もしません。

<div class="impl-ref">

**実装参照**：`squid_n_io::stbridge::import::assemble::auto_assign_supports`（`crates/squid-n-io/src/stbridge/import/assemble.rs`）が判定・設定し、通知は `squid_n_io::stbridge::ImportReport` の notes に積みます。

</div>

## 断面の符号と階（取り込み）

ST-Bridge は断面を `guid` で識別しており、**同じ符号の断面を階ごとに別の定義として持ちます**。
そのため 1 つのファイルに `name="C1"` の `StbSecColumn_S` が、`floor="1"`・`floor="2"`・`floor="3"`
と 3 つ並ぶことは正常な状態です。

Squid-n の断面は[符号と階の組](../model_edit/02_断面の符号と階.md)で識別するため、この構造を
そのまま取り込めます。
断面の `floor` 属性は文字列としてそのまま保持し、階（`StbStory`）への参照としては扱いません。
ST-Bridge の `floor` は自由文字列で、階への id 参照を持たないためです。
実際、断面の階名が `1`・`R`・`PHR` で `StbStory` の名称が `1FL`・`RFL`・`PHRFL` というように
食い違うファイルや、断面の階名に対応する `StbStory` が存在しないファイルがあります。

符号＋階が重複する断面定義は、取り込みの段階で次のように解決します。

- 断面性能・断面形状・**材料**が完全に一致する定義は、1 件へ統合する。統合した件数は
  `ImportReport` の通知（notes）で知らせる
- いずれかが食い違う定義は、符号へ連番を付けて（`b3` → `b3#2`）別の断面として残す。
  符号を変えた内容は `ImportReport` の警告で知らせる

材料も一致の判定に含めるのは、材料は断面が持ち、違う材料を割り当てるならそれは
別の断面だからです。材料だけが違う定義を 1 件へまとめると、片方の材料が無言で消えます。

食い違う定義を捨てずに残すのは、利用者が書いた断面定義を無言で消さないためです。
連番を付けた時点で符号＋階は一意になるので、モデル側の制約とも矛盾しません。

階を持たない断面定義（`floor` 属性のない `StbSecBeam_S` など）が複数あり、内容も同一である場合は
1 件へ統合されます。
小梁の断面定義が同じ内容で何件も並ぶファイルは実際にあるため、この統合で断面一覧が実態に近づきます。

<div class="impl-ref">

**実装参照**：重複の統合・連番付与・材料を含む一致判定は `squid_n_io::stbridge::import::assemble::build_sections`（`crates/squid-n-io/src/stbridge/import/assemble.rs`）が担います。

</div>

## 断面表現（書き出し）

書き出しは **ST-Bridge 2.0.2 標準スキーマ準拠**の 1 形式のみです。内部モデルのパラメトリック断面形状
（`Section.shape`）を対応する標準要素＋形鋼ライブラリ（`StbSecSteel`）へ写像します。

断面ごとの書き出し先と、取り込み・書き出し・往復の詳細は、[ST-Bridge 要素別 変換状況一覧](./03_ST-Bridge_要素別変換状況.md)を参照してください。

Squid-n 固有の解析・設計属性や材料の物性値まで含めた完全一致での往復が必要な場合は
[`.scz`](./01_プロジェクト形式_scz.md) を使います。

<div class="impl-ref">

**実装参照**：断面の写像は `squid_n_io::stbridge::section_std`（`crates/squid-n-io/src/stbridge/section_std.rs`）が、XML 全体の組み立ては `squid_n_io::stbridge::export`（`crates/squid-n-io/src/stbridge/export.rs`）が担います。

</div>

## 開発者向け: ライブラリ API（Rust）

<details>
<summary>Rust コード例（開発者向け）</summary>

```rust
use squid_n_io::stbridge::{import_stbridge, import_stbridge_with_report, export_stbridge};

// 読み込み: ST-Bridge XML 文字列 → 内部モデル
let xml = std::fs::read_to_string("model.stb")?;
let model = import_stbridge(&xml)?;

// 読み込み（欠落の報告つき）: 未対応要素のスキップ・断面欠落・参照解決失敗などを警告として得る
let (model, report) = import_stbridge_with_report(&xml)?;
if !report.is_clean() {
    for w in &report.warnings {
        eprintln!("警告: {w}");
    }
}

// 書き出し: 内部モデル → 標準 ST-Bridge 2.0.2 XML 文字列
let xml = export_stbridge(&model)?;
std::fs::write("model.stb", xml)?;
```

</details>
