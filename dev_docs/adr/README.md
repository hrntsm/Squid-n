# 設計判断（ADR）

Squid-n の重要な設計判断の正本。用語の意味はルートの [CONTEXT.md](../../CONTEXT.md) が持つ。

## 一覧

| # | Status | タイトル |
|---|--------|----------|
| [0001](0001-internal-units.md) | accepted | 内部単位系は N・mm・s とする |
| [0002](0002-determinism.md) | accepted | 決定性は単一スレッドのビット一致を保証する |
| [0003](0003-single-writer-edit-commands.md) | accepted | モデル所有権は単一ライタとし、編集はコマンド経由に限る |
| [0004](0004-story-and-diaphragm-separation.md) | accepted | 階と剛床を分離し、階は床レベル基準の利用者定義とする |
| [0005](0005-materials-on-sections.md) | accepted | 材料は断面が持つ |
| [0006](0006-wall-elements-generated-from-wall-plates.md) | accepted | 壁エレメントは壁版から都度生成する派生物とする |
| [0007](0007-floor-regions-slabs-secondary-members.md) | accepted | 床は床領域・床板・二次部材で表し、解析要素は柱と大梁までとする |
| [0008](0008-secondary-member-load-cascade.md) | accepted | 二次部材の荷重は支持相手へ逐次伝達し、交点は常にピンとする |
| [0009](0009-fail-loud-unassigned-inputs.md) | accepted | 未設定の材料・断面を既定値で埋めない |
| 0010 | — | 欠番（HHT-α 廃止は handoff と現行仕様で足りるため ADR 化しない） |
| [0011](0011-no-gpu-reserved-crates.md) | accepted | GPU は恒久的に実装せず、予約クレートを置かない |
| [0012](0012-stbridge-partial-roundtrip.md) | accepted | ST-Bridge は完全往復を保証しない |
| [0013](0013-adopt-wall-element-model.md) | accepted | 耐震壁は壁エレメント置換モデルを採用し、TVLEM は採用しない |
| [0014](0014-schema-compatibility.md) | accepted | スキーマ互換は初回リリースまで維持しない |

## 規約

- ADR にするのは「後から覆すコストが高い」「背景を知らないと意外に見える」「実際のトレードオフがあった」の 3 条件をすべて満たす**現在有効な判断**だけ。単なる実装手順・容易に変えられる選択・未実装の計画・自明な判断は書かない。
- 番号は本ディレクトリ内の新規連番。欠番は許容し、削除した番号を再利用しない。本文で D 番号（文書ローカルな識別子）を引用するときは「文書パス＋定義節＋番号」で書く。
- 判断を覆すときは、新しい ADR を作成し、旧 ADR の Status を `superseded by ADR-XXXX` に更新する。
- Legacy source には移行元の一次資料を残す。移行元を現行仕様へ書き換えた場合は、改訂前の要点を handoff に転記してから参照する。
- 追加・更新したら、この一覧にも 1 行追記する。
