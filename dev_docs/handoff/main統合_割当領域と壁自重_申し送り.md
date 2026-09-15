# main 統合（割当領域と壁自重・壁断面）の申し送り

作成日: 2026-09-15

## 目的

本作業ブランチ（ADR 0020「囲まれた床板・壁版を支持部材で分割された領域へ割り当てる」の実装）へ
`origin/main` の 9 コミットを取り込み、コンフリクトを解消した。取り込んだ main の主な変更は次のとおり。

- 壁自重の支持辺と鉛直二次部材の端部負担率の明示（main の ADR 0018）
- 壁の材料別実断面と正負別せん断耐力（main の ADR 0019）
- 冗長なテストの削除・統合とレビュー指摘に基づく保証の復元
- 開発ループ（Clippy・test）の高速化、文書整備

## ADR 番号の再編

main が ADR 0018（`0018-explicit-gravity-supports`）と ADR 0019
（`0019-material-wall-section-and-directional-strength`）を追加していたため、本作業の ADR を
`0020-assign-plates-to-member-bounded-regions` へ改番した。本文・リンク・索引を更新済み。

## 競合解消の要点

- **ADR 0020 を優先**した。任意節点境界による `AddSlab` / `AddEnclosedWallPlate` は復活させず、
  割当領域経由の `Assign*ToRegion` / `Set*RegionNoPlate` / `Unset*Region` を正とした。
- **二次部材**は本作業の `id` ＋ `ends` を正とし、main が追加した `gravity_end_shares` を
  `SecondaryMember` に残した（`end_support` は本作業で廃止済みのため採用しない）。
  `SetPostGravityEndShares` は端点対ではなく安定 ID（`member`）で対象を指定する形へ適応した。
  縦材は `gravity_end_shares` が未指定・不正だと逐次伝達の対象から外れる（main の挙動）。
- **壁版の `self_weight_shares`**: main の ADR 0018 を正とし、`edge_shares_with` は明示負担率
  （`has_valid_self_weight_shares` が真）の辺へだけ配る。未指定・不正値のときは幾何から推定せず
  `wall_plates_without_load_path` の解析前エラーとし、別の辺へ振り替えない。当初の統合で入れた
  幾何フォールバック（鉛直支持辺が 2 辺ある場合は等分、無ければ最も低い支持辺 1 辺へ全量）は
  下梁で受ける壁の梁応力を過小評価しうる危険側のため削除した（`has_valid_self_weight_shares` の
  `&Model` 化は、境界を壁版割当領域から解決する本作業の型変更として維持する）。
- **ST-Bridge 取り込み**は割当領域で `StbSlab` を分割・按分し、未帰属面積があれば取り込み全体を
  失敗させる本作業の挙動を正とした。main の旧テストは支持部材の無い面に版を置いていたため、
  その面を囲む大梁をテストへ追加して成立させた。

## main 側テストの扱い

本作業の API 変更（`SecondaryMember.nodes`・`end_support` の廃止、`WallPlateShape::Enclosed` の
境界廃止、`SlabShape::Enclosed` の境界廃止）と両立できない main 側テストは、本作業版の等価テストへ
置き換えた。主な対象は `squid-n-core/model/tests.rs`、`squid-n-edit/src/tests.rs`、
`squid-n-mcp/src/tests.rs`、`squid-n-load/{cascade,wall_plate_load,story_gen}/tests.rs`。
個別に移植した main の機能テストは `SetPostGravityEndShares`（`squid-n-edit`・`squid-n-mcp`）と、
壁自重の明示負担率・間柱端部負担率の挙動を確認するテストである。

## 検証

`cargo build --workspace --locked`、`cargo fmt --all -- --check`、
`cargo test --workspace --locked`、`cargo test -p squid-n-app --features gui`、
`cargo test -p squid-n-mcp --features mcp`、clippy 3 本（`-D warnings`）、
`cargo run -p xtask --locked -- check-deps`、`mdbook build` はすべて成功。

## ADR 0018 との整合の是正（2026-09-15）

当初の統合で入れた幾何フォールバックは main の ADR 0018 に反する危険側の挙動だったため、
明示負担率必須・未指定／不正は入力エラー・振り替えなしの契約へ戻した。

- `edge_shares_with` の `geometric_edge_shares` 呼び出しを削除し、未指定・不正・指定辺の
  支持欠落では空を返す（`wall_plates_without_load_path` が解析前エラーにする）。
- `squid-n-load` の回帰テストを追加・復元した。明示負担率どおりの配分、負担率の未指定・
  不正値・支持区間重複での診断、指定辺がスリットで切れた場合に振り替えないこと、
  鉛直支持辺があっても幾何フォールバックしないこと、間柱の端部負担率どおりの配分と
  未指定・不正時の逐次伝達除外を固定する。
- `wall_post_model` の壁版へ割当領域境界の辺順に負担率を明示した。

## 残課題

- なし（ADR 0018 の契約へ整合済み）。
