# はじめに

**Squid-n** は、Rust で実装された日本の建築構造計算一貫プログラムです。

このサイトは、Squid-n を利用する方に向けたドキュメントです。

- **[アーキテクチャ](./architecture.md)**：15 クレートから成る階層型構成の全体像
- **[モデル入出力（ファイル形式）](./model_io/README.md)**：ネイティブの `.scz` 形式と
  ST-Bridge（`.stb`）形式でモデルを保存・読込・書出する入出力経路と、その対応範囲
- **[MCP サーバ](./mcp_server/README.md)**：AI エージェントからモデル照会・解析実行・結果取得を行う
  MCP（Model Context Protocol）サーバのビルド・起動・ツール一覧
- **[準備計算（解析前の確認）](./preparation/README.md)**：応力解析に先立って階・剛域・断面性能・
  地震力・荷重を確定させる計算と、その確認画面の見方
- **[結果の表示（3D ビューア）](./result_view/README.md)**：変形図の表示倍率、床・二次部材の変形追従、
  検定比図・モデル化図など、3D ビューアの表示仕様
- **[計算根拠（理論・出典）](./calc_basis/README.md)**：各計算が「何という基準・法令の、何という式で」
  算定されているかを、告示・施行令の条／式番号と実装位置まで対応づけた根拠ドキュメント。
  荷重・材料・断面性能・構造解析・一次設計・二次設計・部材終局耐力・免震制振の
  各章を、算定項目ごとのページに分けて収録しています

## ライセンスと免責事項

Squid-n は MIT License のもとで公開しているソフトウェアです。
ライセンスの全文はリポジトリの [LICENSE](https://github.com/hrntsm/squid-n/blob/main/LICENSE) にあります。

```
MIT License

Copyright (c) 2026 Hiroaki NATSUME
```

### 無保証（AS IS）

MIT License は、ソフトウェアを「現状のまま（AS IS）」提供し、明示・黙示を問わずいかなる保証も行わないことを条件としています。

> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
> IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
> FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.

この無保証は、Squid-n が出力する応力、変形、検定比、保有水平耐力などの計算結果にもそのまま及びます。
つまり Squid-n は、計算結果の正確性、妥当性、および特定の目的への適合性を、いずれも保証しません。
また、Squid-n の使用または使用不能によって生じたいかなる損害についても、著作権者および貢献者は責任を負いません。

本ドキュメントについても同じです。
このサイトは、各計算がどの法令・規準のどの式に基づいて算定されているかを説明するものであり、個々の建築物についてその結果が正しいことや、適法であることを保証するものではありません。

### 利用にあたって

Squid-n は、国土交通大臣の認定を受けた構造計算プログラム（いわゆる大臣認定プログラム）ではありません。
そのため、確認申請における認定プログラムとしての取扱い（大臣認定プログラムを用いた場合の審査の特例）は受けられません。

入力データの妥当性、モデル化の適否、そして出力された結果の検証は、いずれも利用者の責任に属します。
計算結果は、手計算や理論解、ほかのプログラムとの突合などによる利用者自身の検証を経たうえで使用してください。
設計における式の適用可否（適用範囲や前提条件）の判断も、設計者（構造設計一級建築士等）の責任に属します。

## 開発者向け資料

設計仕様・検証記録・開発運用ドキュメントは開発者向けのため本サイトには含めていません。
これらは [dev_docs/](https://github.com/hrntsm/squid-n/tree/main/dev_docs) に集約しており、リポジトリの以下を参照してください。

- [dev_docs/specs/](https://github.com/hrntsm/squid-n/tree/main/dev_docs/specs)：フェーズ単位の実装仕様書と原典（法令・規準）照合リスト
- [dev_docs/v_and_v/](https://github.com/hrntsm/squid-n/tree/main/dev_docs/v_and_v)：各要素・各設計式の Verification & Validation レポート
- [dev_docs/handoff/ROADMAP.md](https://github.com/hrntsm/squid-n/blob/main/dev_docs/handoff/ROADMAP.md)・[dev_docs/handoff/](https://github.com/hrntsm/squid-n/tree/main/dev_docs/handoff)：完了済みロードマップ、申し送り、UI 関連ドキュメント

## リポジトリ

ソースコードは [github.com/hrntsm/squid-n](https://github.com/hrntsm/squid-n) にあります。

ビルド・テスト・静的解析の手順はリポジトリの `README.md` を参照してください。
