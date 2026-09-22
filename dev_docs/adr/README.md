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
| [0008](0008-secondary-member-load-cascade.md) | superseded by ADR-0018 | 二次部材の荷重は支持相手へ逐次伝達し、交点は常にピンとする |
| [0009](0009-fail-loud-unassigned-inputs.md) | accepted | 未設定の材料・断面を既定値で埋めない |
| 0010 | — | 欠番（HHT-α 廃止は handoff と現行仕様で足りるため ADR 化しない） |
| [0011](0011-no-gpu-reserved-crates.md) | accepted | GPU は恒久的に実装せず、予約クレートを置かない |
| [0012](0012-stbridge-partial-roundtrip.md) | accepted | ST-Bridge は完全往復を保証しない |
| [0013](0013-adopt-wall-element-model.md) | accepted | 耐震壁は壁エレメント置換モデルを採用し、TVLEM は採用しない |
| [0014](0014-schema-compatibility.md) | accepted | スキーマ互換は初回リリースまで維持しない |
| [0015](0015-cantilever-secondary-members.md) | accepted | 片持ち小梁は支持条件で表し、基端モーメントは伝達しない |
| [0016](0016-attached-slabs-between-members.md) | accepted | 取り付く床板は支持部材の間ごとに表す |
| [0017](0017-attached-slab-support-edges.md) | accepted | 取り付く床板の荷重は、全長を覆う支持部材の辺へ最近接負担面積で分配する |
| [0018](0018-explicit-gravity-supports.md) | accepted | 壁自重の支持辺と鉛直二次部材の端部負担率を明示する |
| [0019](0019-material-wall-section-and-directional-strength.md) | accepted | 壁の材料別実断面と正負別せん断耐力を採用する |
| [0020](0020-assign-plates-to-member-bounded-regions.md) | accepted | 囲まれた床板・壁版は支持部材で分割された領域へ割り当てる |
| [0021](0021-same-position-support-members.md) | accepted | 同じ位置に支持部材が重なる場合は同種をエラー、異種は主架構優先とする |
| [0022](0022-unmapped-slit-specification-warning.md) | accepted | 耐震スリット指定が辺へ反映できない場合は警告する |
| [0023](0023-hinge-detail-single-source-analysis-models.md) | accepted | ヒンジ詳細の表示は解析要素と同じ非線形モデルを単一情報源とする |
| [0024](0024-separate-panel-element-and-joint-check.md) | accepted | 仕口パネル解析要素と柱梁接合部の断面算定を分離する |
| [0025](0025-consistent-mass-section-properties.md) | accepted | Beam と Fiber の整合質量は共通の断面質量特性から算定する |
| [0026](0026-current-spec-single-source-docs.md) | accepted | 現在仕様の正本は docs とし、フェーズ実装仕様書を置かない |
| [0027](0027-src-cft-ns-material-or-fallback.md) | accepted | SRC/CFT の等価断面剛性は材料由来 ns を通常とし、算定不能時は N_S_EQ=15 へフォールバックして通知する |

## 規約

- ADR にするのは「後から覆すコストが高い」「背景を知らないと意外に見える」「実際のトレードオフがあった」の 3 条件をすべて満たす**現在有効な判断**だけ。単なる実装手順・容易に変えられる選択・未実装の計画・自明な判断は書かない。
- 番号は本ディレクトリ内の新規連番。欠番は許容し、削除した番号を再利用しない。本文で D 番号（文書ローカルな識別子）を引用するときは「文書パス＋定義節＋番号」で書く。
- 判断を覆すときは、新しい ADR を作成し、旧 ADR の Status を `superseded by ADR-XXXX` に更新する。
- Legacy source には移行元の一次資料を残す。移行元を現行仕様へ書き換えた場合は、改訂前の要点を handoff に転記してから参照する。
- 追加・更新したら、この一覧にも 1 行追記する。
