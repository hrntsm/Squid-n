# 壁領域・床領域 編集コマンド追加（Step 3）申し送り

作成日: 2026-08-22  
対象クレート: `squid-n-edit`

---

## 概要

`squid-n-edit` クレートに、二次部材（小梁・間柱）と壁領域を操作する編集コマンドを追加した。
undo/redo 対応・バリデーション・カスケード削除を含む 9 コマンドを実装し、テスト 6 件を追加している。

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

## テスト

`crates/squid-n-edit/src/tests.rs` に 6 件追加した。

| テスト名 | 確認内容 |
|----------|----------|
| `add_delete_secondary_member` | 追加・削除・ID 繰り上げ・undo |
| `delete_secondary_member_cascade_slab` | 削除時に Slab.secondary_joist_ids から除去 |
| `delete_secondary_member_cascade_wall_region` | 削除時に WallRegion.post_ids から除去 |
| `undo_delete_secondary_member` | undo で元位置・参照（slab_refs / region_refs）が復元される |
| `add_delete_wall_region` | 壁領域の追加・削除・undo |
| `set_slab_secondary_joist_ids_validation` | Joist でない ID を渡すと Noop（バリデーション確認） |

---

## 残課題

| 状態 | 項目 |
|------|------|
| ☐ | `squid-n-app` のナビゲータへ二次部材・壁領域の編集 UI を接続する |
| ☐ | `squid-n-mcp` に二次部材・壁領域操作のツールを追加する |
| ☐ | 床領域・壁領域を「版を任意、小梁／間柱は領域の子」とするナビ階層化（詳細は[残課題一覧 §2](残課題一覧.md)） |

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
