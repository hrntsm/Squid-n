# 耐震壁は壁エレメント置換モデルを採用し、TVLEM は採用しない

Status: accepted

耐震壁の解析モデルは、壁柱と剛梁からなる 4 節点 24 自由度の壁エレメント置換モデル（実務の一貫構造計算で標準的に用いられるモデル）を採用する。設計書 §6.6・P5.5 で計画していた壁谷澤 TVLEM は採用しない。TVLEM は採用文献の確定が前提で文献間の定義揺れがあり、実験照合（妥当性確認）が必須である一方、現行モデルは実装・GUI・入出力・数量積算・V&V の経路が揃っているためである。

Legacy source:
- `docs/calc_basis/04_要素剛性/05_壁エレメントモデル.md`
- `dev_docs/specs/P5.5_壁とMS.md`
- `dev_docs/v_and_v/README.md` 索引 #16（TVLEM は対象外）・#29・#30
- `dev_docs/v_and_v/未検証一覧.md` §3（耐震壁・壁エレメントの残る検証）
- `dev_docs/handoff/DomainDocs基盤導入_申し送り.md` §4（撤回の記録）
