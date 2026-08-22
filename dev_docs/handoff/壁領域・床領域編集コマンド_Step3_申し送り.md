# 壁領域・床領域 編集コマンド追加（Step 3）申し送り

作成日: 2026-08-22  
対象クレート: `squid-n-edit`

---

## 概要

`squid-n-edit` クレートに、二次部材（小梁・間柱）と壁領域を操作する編集コマンドを追加した。
undo/redo 対応・バリデーション・カスケード削除を含む 9 コマンドを実装した。

あわせて、この構造を入れたことで壊れた既存経路（階への複製・部材削除）をレビューで洗い出して修正している。
詳細は下記「レビューで見つかった不具合」を参照すること。
データモデルの設計意図は [`specs/床領域と壁領域.md`](../specs/床領域と壁領域.md) にまとめた。

---

## 追加ファイル

### `crates/squid-n-edit/src/secondary.rs`（新規）

以下のコマンドを実装した。

| コマンド | 概要 |
|----------|------|
| `AddSecondaryMember` | 末尾に二次部材を追加。節点・断面の存在チェック込み |
| `DeleteSecondaryMember` | ID 指定で削除。後続 ID 繰り上げ・カスケード削除（スラブ/壁領域） |
| `InsertSecondaryMember` | DeleteSecondaryMember の逆操作専用。元の位置・参照を完全復元 |
| `SetSecondaryMemberSection` | 断面変更。存在しない断面は Noop |
| `SetSlabSecondaryJoistIds` | スラブの secondary_joist_ids 全置換。Joist 種別・重複チェック込み |
| `AddWallRegion` | 壁領域を末尾に追加。wall 部材・post_ids の存在チェック込み |
| `DeleteWallRegion` | index 指定で壁領域削除 |
| `InsertWallRegion` | DeleteWallRegion の逆操作専用 |
| `SetWallRegion` | 壁領域内容の全置換。バリデーション込み |

---

## 実装上の判断

### `id_indexed_delete_insert!` マクロを使わなかった理由

`DeleteSecondaryMember` はカスケード削除（スラブ・壁領域から ID を除去）と
退避データ（`slab_refs` / `region_refs`）を持つため、マクロに収まらない個別実装とした。

`WallRegion` は他データから参照されないため `indexed_delete_insert!` マクロを
使えるが、`AddWallRegion` 側でバリデーション（`wall` 部材・`post_ids` の存在確認）が
必要なため、Delete/Insert のみマクロ化を検討した結果、全て手書きのほうが一貫性が
高いと判断し個別実装とした。

### ID 繰り上げ処理

`shift_secondary_member_ids` は `Model::visit_secondary_member_ids` を呼ぶだけの
薄いラッパーとした（`shift_node_ids` / `shift_elem_ids` と同パターン）。
`visit_secondary_member_ids` が `secondary_members.id` ・スラブ `secondary_joist_ids` ・
壁領域 `post_ids` を一括走査するため、追加先が増えても core 側の更新だけで追随できる。

### カスケード削除の退避順序

削除時は「縮んでいく配列で昇順」に `(添字, 位置)` を記録し、
復元（`InsertSecondaryMember`）時は逆順（`iter().rev()`）で挿入する。
これは `InsertMember` / `InsertNode` の部材荷重復元と同じパターンで、
複数箇所に同一 ID が入っている場合でも削除前の並びを厳密に再現できる。

---

## レビューで見つかった不具合

`SecondaryMember.id == 配列添字` という不変条件と、その `validate` 検証を新たに入れた。
ところが追随を確認したのは新規コマンドだけで、既存の `secondary_members` 変更箇所を見落としていた。
そのため次の 3 件が壊れており、いずれも再現テストを添えて修正した。

| 箇所 | 症状 |
|------|------|
| `story_copy.rs` `copy_secondary` | `..sm` が複製元の `id` まで写し、`validate` が `IndexMismatch`。二次部材を持つモデルで階への複製が通らない |
| `story_copy.rs` `copy_slabs` | `..sl` が `secondary_joist_ids` を写し、複製先の床が複製元の小梁を子として抱える。`validate` は床をまたいだ重複を見ないため黙って通る |
| `node_member.rs` `DeleteMember` | 削除した壁を指す `WallRegion.wall` を放置。ID 繰り上げ後に隣の要素を指し、それが壁だと検証も通って別の壁へ付け替わる |

`copy_secondary` の上書き経路にあった `model.secondary_members = keep;` も、
ID の再採番と領域側の参照の張り替えを行う `Model::retain_secondary_members` へ置き換えた。

**教訓として、`id == index` のような不変条件を新設したときは、その型を変更する既存箇所を全数で洗うこと。**
今回は `grep` で `secondary_members` の変更箇所（`push` / `remove` / `retain` / 代入）を数えたところ、
非テストの箇所は 4 つしかなく、そのうち 2 つが壊れていた。

なお、`cargo clippy -p squid-n-app --features gui` を通していなかったため、
`viewer/deform.rs` のテストヘルパーが `id` 未指定でコンパイルできない状態のままだった。
CONTRIBUTING.md の静的解析コマンドは、既定機能だけでなく `gui` / `mcp` も必ず実行すること。

## テスト

`crates/squid-n-edit/src/tests.rs` に 13 件、`crates/squid-n-core/src/model/tests.rs` に 3 件追加した。

| テスト名 | 確認内容 |
|----------|----------|
| `add_delete_secondary_member` | 追加・削除・ID 繰り上げ・undo |
| `delete_secondary_member_cascade_slab` | 削除時に Slab.secondary_joist_ids から除去 |
| `delete_secondary_member_cascade_wall_region` | 削除時に WallRegion.post_ids から除去 |
| `undo_delete_secondary_member` | undo で元位置・参照（slab_refs / region_refs）が復元される |
| `add_delete_wall_region` | 壁領域の追加・削除・undo |
| `set_slab_secondary_joist_ids_validation` | Joist でない ID を渡すと Noop（バリデーション確認） |
| `test_set_secondary_member_section_roundtrip` | 断面変更・undo・redo・実在しない断面は Noop |
| `test_set_wall_region_roundtrip` | 壁領域の全置換・undo・範囲外 index は Noop |
| `test_add_wall_region_rejects_non_wall_elem` | Wall でない要素を `wall` に指定すると Noop |
| `test_set_slab_secondary_joist_ids_roundtrip` | 小梁 ID リストの置換・undo・redo |
| `test_copy_story_secondary_keeps_id_index_invariant` | 階への複製で `id == index` が保たれる（再現テスト） |
| `test_copy_story_slab_does_not_inherit_source_joist_ids` | 複製先の床が複製元の小梁を抱えない（再現テスト） |
| `test_delete_member_clears_wall_region_wall` | 壁削除で壁領域が版なしへ戻り、undo で復元される（再現テスト） |
| `test_validate_duplicate_slab_secondary_joist_ids` | 床の小梁 ID の重複を弾く |
| `test_validate_duplicate_wall_region_post_ids` | 壁領域の間柱 ID の重複を弾く |
| `test_retain_secondary_members_remaps_region_refs` | 間引き時の ID 再採番と領域側の参照の張り替え |

---

## 残課題

| 状態 | 項目 |
|------|------|
| ☐ | `squid-n-app` のナビゲータへ二次部材・壁領域の編集 UI を接続する |
| ☐ | `squid-n-mcp` に二次部材・壁領域操作のツールを追加する |
| ☐ | 床領域・壁領域を「版を任意、小梁／間柱は領域の子」とするナビ階層化（詳細は[残課題一覧 §2](残課題一覧.md)） |
| ☐ | 荷重分配・CMQ 変換を所属関係に対応させ、同時に床をまたいだ小梁の重複検証を追加する |
| ☐ | 利用者向け `docs/` への節の追加（UI と荷重分配ができてから。設計は[specs/床領域と壁領域](../specs/床領域と壁領域.md)） |

---

## 次ステップへの申し送り

### squid-n-app ナビゲータ更新

`squid-n-app` の二次部材パネルが現在どのように実装されているかを確認した上で、
以下のコマンドを UI アクションから呼ぶ接続を実装する。

- 追加ボタン → `AddSecondaryMember`
- 削除ボタン → `DeleteSecondaryMember`
- 断面変更 → `SetSecondaryMemberSection`
- スラブ小梁 ID リスト編集 → `SetSlabSecondaryJoistIds`
- 壁領域の追加/削除/編集 → `AddWallRegion` / `DeleteWallRegion` / `SetWallRegion`

`UndoStack::run` の戻り値（`bool`）で変更有無を検出し、
`App::staleness.mark_edited()` を適切に呼ぶこと（他コマンドと同パターン）。
