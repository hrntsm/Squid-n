<!--
このテンプレートは、レビュアーが「何を、なぜ変えたか」「どのように確認したか」
「既存の挙動にどんな影響があるか」を短時間で把握できることを目的としています。
不要な節は削除して構いませんが、概要・変更内容・検証は具体的に記載してください。
-->

## 概要

<!-- この PR の目的と、なぜこの変更が必要なのかを簡潔に記載してください。 -->

## 変更内容

<!-- 主な変更を箇条書きで記載してください。大きな変更は機能・モデル・UI・テスト・ドキュメントなどに分けてください。 -->

- 

## 影響・注意事項

<!--
利用者から見える挙動、データ形式、既存プロジェクトファイル、後方互換性などへの影響を記載してください。
影響がない場合は「なし」と記載してください。
-->

- 

## 検証

<!-- 実行したコマンドと結果を記載してください。 -->

- [ ] `cargo clippy --workspace --all-targets --locked -- -D warnings`
- [ ] `cargo clippy -p squid-n-app --all-targets --features gui --locked -- -D warnings`
- [ ] `cargo clippy -p squid-n-mcp --all-targets --features mcp --locked -- -D warnings`
- [ ] `cargo fmt --all -- --check`
- [ ] `cargo test --workspace --locked`
- [ ] `cargo test -p squid-n-app --features gui`
- [ ] `cargo test -p squid-n-mcp --features mcp`
- [ ] `cargo run -p xtask -- check-deps`
- [ ] `mdbook build`

<!-- 実行していない項目はそのままで構いません。変更範囲に応じて必要な検証を行い、結果や補足を以下に記載してください。 -->

### 検証結果

<!-- 例: cargo test -p squid-n-app --features gui: 475 passed / 0 failed -->

## ドキュメント

<!--
docs/ / dev_docs/ を更新した場合は、対象と理由を記載してください。
不要な場合は「変更なし」と記載してください。
-->

- 

## 残課題

<!-- 後続 PR に回した事項、既知の制約、今後対応する事項を記載してください。なければ「なし」と記載してください。 -->

- 

## レビューで確認してほしい点

<!--
特にレビューしてほしい設計・仕様・実装上のポイントがあれば記載してください。
なければこの節を削除して構いません。
-->

-
