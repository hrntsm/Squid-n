# SRC/CFT 等価断面の ns フォールバック — 2026-09

## 対象

SRC/CFT の要素剛性に用いる等価断面性能（ヤング係数比 \\( n\_s \\)）について、材料由来の値を算定する通常経路、材料を知らない `Section` 形状レベルの暫定値、算定できない場合の既定値フォールバック、およびフォールバック時の利用者通知を実装と照合する。

現在仕様の正本は [SRC / CFT の等価断面性能](../../docs/calc_basis/03_断面性能/06_SRC_CFT_の等価断面性能.md) と [部材剛性](../../docs/preparation/09_部材剛性.md) とし、本レポートは照合結果と背景を扱う。

## 背景

docs は `N_S_EQ = 15` を ns の既定として説明していたが、実装の通常経路は材料のヤング係数比 \\( E\_{\text{steel}}/E\_c \\) を算定していた。15 の位置付けが文書と実装で食い違っていたため、Issue #306 で両者を照合し、15 を「材料から算定できない場合のフォールバック」と確定した。

## 検証対象の要約

要素剛性に用いる SRC/CFT 等価断面性能について、通常の算定経路、`Section` 形状レベルの暫定値、材料から算定できない場合の既定値フォールバック、およびフォールバック時の利用者通知を、現行仕様（[SRC / CFT の等価断面性能](../../docs/calc_basis/03_断面性能/06_SRC_CFT_の等価断面性能.md)・[部材剛性](../../docs/preparation/09_部材剛性.md)）と照合した。

## フォールバック経路の特定

| 経路 | 実装 | 条件 |
|---|---|---|
| 通常経路 | `SectionShape::src_equivalent_props` / `cft_equivalent_props`（`crates/squid-n-core/src/section_shape/composite.rs`）を `squid_n_element::frame::beam::composite_props_of` が呼ぶ | 主材料の `fc` があり、ヤング係数が正、形状が対象 |
| `Section` 形状レベルの暫定値 | `SectionShape::to_section` の `iy`・`iz`・`as_y`・`as_z`、`calc_axial_stiffness_area`（SRC は `N_S_EQ = 15`、CFT は鋼管のみ） | 形状のみから生成。要素構築時に通常経路の値で上書き |
| フォールバック（SRC） | `composite_props_of` が `None` → `N_S_EQ = 15` | 主材料の `fc` 未設定、ヤング係数が 0 以下 |
| フォールバック（CFT） | `composite_props_of` が `None` → 鋼管のみ | 主材料の `fc` 未設定、ヤング係数が 0 以下、CFT 充填部の内法が 0 になる形状 |

`Section` の断面レベル `iy`・`iz` も形状レベルで生成されるため、フォールバック時の曲げ・せん断剛性は形状レベルの値がそのまま使われる（SRC は `N_S_EQ = 15` の暫定値、CFT は充填コンクリートを含めない鋼管のみの値）。

## 安全側の判定

`N_S_EQ = 15` は典型的な材料由来値（\\( F\_c = 21 \\) で約 9.5、\\( F\_c = 60 \\) で約 6.1）より常に大きく、鉄骨の等価換算を過大にする。これは全体変位を小さく見せる（非保守）一方、着目部材の応力は大きく見せる（保守）ため、一方向に定まらない。**安全側とは言い切れない。** 値の妥当性を含めて原典照合を要する。

## 実装した通知

フォールバック時は解析を止めず、次を通知する。

- 診断（解析前チェック）: 見出し「材料由来の等価断面性能を算定できない SRC/CFT 断面を使う部材があります」／短句「等価断面性能を算定できません」／是正「断面タブで主材料のコンクリート Fc とヤング係数を設定してください。CFT では鋼管の板厚・外径（充填部の内法が正の値か）も確認してください。未設定・不成立の間は、SRC は N_S_EQ=15、CFT は鋼管のみで剛性を評価します。」（`ModelIssue::warn`、`crates/squid-n-solver/src/statics/analysis/precheck.rs`）
- 準備計算「部材剛性」表: フォールバック行を例外として載せ、断面名に SRC は「（既定値 15）」、CFT は「（鋼管のみ）」を付ける（`crates/squid-n-app/src/app/preparation.rs`、`crates/squid-n-app/src/prep_view.rs`）。CFT の hover は、材料条件だけでなく鋼管の板厚・外径（充填部の内法が正の値か）の確認も促す

## 検証

| 検証 | 内容 | 結果 |
|---|---|---|
| `test_model_issues_warns_src_composite_fallback` | SRC で `fc` 未設定のとき、警告（`IssueSeverity::Warning`）が出て解析は止まらない | ✅ |
| `test_model_issues_warns_cft_composite_fallback` | CFT で `fc` 未設定のとき、同じく警告が出る | ✅ |
| `test_model_issues_warns_cft_composite_fallback_for_zero_core` | CFT で `fc`・ヤング係数は正常だが充填部の内法が 0 になる形状（板厚過大）のとき、形状原因でも警告が出る | ✅ |
| `test_model_issues_no_composite_fallback_warning_with_fc` | `fc` があるときは警告が出ない | ✅ |
| `test_preparation_member_stiffness_reports_src_fallback_without_fc` | SRC のフォールバック行が準備表に載り、種別が `SrcNsDefault` になる | ✅ |
| `test_preparation_member_stiffness_reports_cft_fallback_without_fc` | CFT のフォールバック行が準備表に載り、種別が `CftSteelOnly` になる | ✅ |
| `test_preparation_member_stiffness_reports_cft_fallback_for_zero_core` | CFT で内法が 0 になる形状でも準備表の種別が `CftSteelOnly` になる | ✅ |

## 実行結果（2026-09-22）

- `cargo test -p squid-n-solver --lib composite_fallback`：3 成功、0 失敗
- `cargo test -p squid-n-app --features gui --lib fallback_without_fc`：2 成功、0 失敗

GUI の実画面の操作・目視は未実施。

## 残課題

- `N_S_EQ = 15` の原典照合。値の根拠（慣用値の出典）を一次資料で確認する。
- 15 が安全側と言い切れないことの利用者への開示方法（計算根拠での注意喚起の妥当性）。
