# GPU は恒久的に実装せず、予約クレートを置かない

Status: accepted

GPU 高速化は恒久的に実装しない。実装の入らない予約クレートや feature（`gpu`）を置かず、解は CPU で確定する。GPU 経路の f32 概算モードは持たない（反復ソルバ PCG の f32 演算は GPU とは別の既存経路）。

Legacy source:
- `dev_docs/handoff/GPUとMLクレートの削除_申し送り.md`
- 初回設計書（#313 で廃止。git 履歴参照）§15・§16
