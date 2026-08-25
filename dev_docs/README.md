# dev_docs — 開発者向けドキュメント

Squid-n の開発者向けドキュメントを集約したディレクトリ。
**利用者向けの計算根拠は `docs/`（mdBook サイト）** にあり、本ディレクトリはサイトには含めない。

## パッと見る一覧

| 一覧 | 内容 |
|------|------|
| [**handoff/残課題一覧.md**](handoff/残課題一覧.md) | 実装残りの集約チェックリスト |
| [**v_and_v/未検証一覧.md**](v_and_v/未検証一覧.md) | V&V 未完了（❌/🔶）の集約チェックリスト |
| [**specs/用語集.md**](specs/用語集.md) | 実装に登場する概念と呼び名の集約（確立語も含む、利用者とエージェント間の共通語彙）。命名前に確認する |

## 構成

| ディレクトリ / ファイル | 内容 |
| --- | --- |
| [`specs/`](specs/README.md) | 実装仕様書・設計書・原典（法令・規準）照合リスト |
| [`v_and_v/`](v_and_v/README.md) | Verification & Validation（参照実装照合・監査・レビュー記録） |
| [`handoff/`](handoff/README.md) | 申し送り・開発運用ドキュメント（実装内容と残課題） |

### handoff（申し送り）

- [**README.md**](handoff/README.md) — 全申し送りファイルの目録（**時系列**・カテゴリ別）
- [**残課題一覧.md**](handoff/残課題一覧.md) — 未完了項目の集約チェックリスト
- [`ROADMAP.md`](handoff/ROADMAP.md) — 動作達成ロードマップ（2026-07 完了済み・歴史的記録）
- [`申し送り.md`](handoff/申し送り.md) — 初期ロードマップ実装の横断申し送り

### v_and_v（検証記録）

- [**README.md**](v_and_v/README.md) — レポート目録・要素→テスト索引
- [**未検証一覧.md**](v_and_v/未検証一覧.md) — 未検証・一部項目の集約チェックリスト
- [`pending_items.md`](v_and_v/pending_items.md) — P9 仕様乖離の歴史的記録
