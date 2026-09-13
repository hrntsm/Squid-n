# ST-Bridge は完全往復を保証しない

Status: accepted

ST-Bridge の取り込みは幾何・断面・部材・荷重までを対象とし、書き出しは荷重を含まない（解析結果・独自属性も対象外）。完全往復は保証しない。取り込みは「ST-Bridge の構造を写す」処理ではなく「正しい内部モデルへ変換する」処理と位置づけ、往復互換性が下がっても内部モデルの正しさを優先する。

Legacy source:
- `dev_docs/specs/構造計算一貫プログラム_実装設計書.md` §12.5
- `docs/model_io/02_ST-Bridge_形式.md`（荷重は取り込みのみ）
- `dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` §3 D12
