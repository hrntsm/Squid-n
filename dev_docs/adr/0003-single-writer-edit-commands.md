# モデル所有権は単一ライタとし、編集はコマンド経由に限る

Status: accepted

モデルの編集は GUI・MCP とも編集トランザクション（コマンド）経由に直列化し、直接書き換えを許さない。同じモデルに対して GUI と MCP が同じ解析結果を返すこと、Undo と将来の外部連携（Grasshopper Sync）が同じ経路に乗ることを保証するためである。

Legacy source:
- `dev_docs/specs/構造計算一貫プログラム_実装設計書.md` R29・§14.2
- `dev_docs/specs/P13_Grasshopper連携.md` §1 D3・D22・D23
