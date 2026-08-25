# 床領域の荷重分配を作り替える／`FloorRegion` と `Slab` を再分離する（2026-08, Step 4）

**対象**: 床領域内の荷重分配を小梁で再分割する経路への作り替え（申し送り Step 4）。
着手時に判明した設計修正（`FloorRegion`/`Slab` の再分離、D3 の修正）を含む
**区分**: データモデルの是正・荷重分配の作り替えと、その影響の記録
**判定**: 🔶（荷重分配の作り替えと型の再分離は実施済み。小梁設計を分配結果から出すのは Step 5 に残る。
**加えて** `static.DL.sum_base_axial` が Step 3 から約 7.7% 変化した原因を特定できておらず、
未解決のまま残る〔§「代表スカラの変化」・§「残す限界」参照〕）

---

## 対象

申し送り [床領域・壁領域の再設計](../handoff/床領域・壁領域の再設計_申し送り.md) の Step 4（D9）。
あわせて、Step 4 着手時に D3（領域が版の仕様を直接持つ）の前提が誤っていたことが判明し、
`FloorRegion`（床領域＝大梁の1区画）と `Slab`（床板＝区画内の版。大梁または小梁で囲まれる）を
再び別の型へ分離した（同申し送り §3.1）。

分配の入口は `squid_n_load::floor::distribute_region(model, region, w_of)`。
1 つの `FloorRegion` が複数の `Slab`（`region.slab_ids`）を持ちうるため、手入力小梁ライン
（`region.joists`）による二段階伝達がある区画は代表床板（`slab_ids` の先頭）で判定し、
それ以外は各 `Slab` を独立に境界（大梁または小梁）へ分配する。

## 入力モデル

`crates/squid-n-app/tests/fixtures/model.stb`（4 層＋PH の S 造。取り込み後は
節点 166・解析要素 115・二次部材 56・床領域 26・床板 82）。

## 実装中に見つけて直した既存バグ 4 件

型の再分離とは独立に見つかった不具合。詳細は申し送り §5.4 を参照。

1. `squid_n_load::floor::resolve_edges_to_span` が `LoadTarget::Edge → Span` へ変換する際、
   辺インデックスを一時的に入れていた `elem` フィールドを番兵 `ElemId(u32::MAX)` へ戻し忘れており、
   偶然実在する `ElemId` と衝突して無関係な部材へ全荷重が載る場合があった
   （`test_attached_anchor_spans_subdivided_beams`・`test_overlapping_beams_fall_back_to_nodes` で発覚）
2. `squid_n_edit::DeleteSlab` が汎用マクロ（ID を単純に繰り上げるだけ）を使っており、
   `FloorRegion.slab_ids` からの参照除去（カスケード）をしていなかった。削除後に別の床板の
   ID と衝突し、`validate` が「複数の床領域から参照されている」を返す状態を作れた
   （`test_copy_story_overwrite_mirrors_absence` で発覚。`DeleteSecondaryMember` と同じ
   手書きのカスケード削除に書き直した）
3. **`squid_n_element::beam::stiffness_factors::slab_cooperating_width` がスラブ協力幅・
   合成梁剛性を丸ごと算定しなくなっていた（本 Step 4 の再分離自体が生んだ回帰）。**
   判定が「梁の両端節点をともに含む**床板（`Slab`）**の境界」を条件としていたため、
   区画が小梁で複数の床板へ細分されると、区画の外周を走る大梁の両端が別々の床板へ
   分かれてしまい、どちらの床板の境界にも「両端を含む」が成立しなくなる。フィクスチャ
   （区画 26・床板 82。区画あたり平均 3 枚強）で確認したところ、修正前は
   `build_prep_member_stiffness` の表が 0 行（対象候補 115 本中 0）になっていた。
   判定を**床板ではなく床領域の境界（`FloorRegion::boundary`。小梁の分割によらず常に
   大梁の1区画を表す）優先**に直し、板厚は区画内の床板（`region.slab_ids`）の
   `slab_plate_thickness` の最大を用いるよう修正した（どの床領域にも属さない床板は
   従来どおり床板自身の境界で判定する）。回帰の再発を防ぐ単体テスト
   `test_beam_new_slab_cooperation_width_survives_joist_subdivided_region`
   （`crates/squid-n-element/src/frame/beam/tests.rs`）を追加した。
   あわせて `build_prep_member_stiffness`（`crates/squid-n-app/src/app/preparation.rs`）の
   事前判定が建物一律 `Model::slab_thickness`（既定 0）だけを見ており、個々の床板に
   板厚があっても表が空になる別のバグも直した（判定を「個々の床板の板厚が 1 枚でもある」
   OR「建物一律あり」に修正）。
4. `squid_n_load::floor::distribute_region` の手入力小梁ライン経路（`uses_joist_distribution`）が、
   区画の代表床板（`region.slab_ids` の先頭）1 枚だけを見て二段階伝達へ回しており、区画が
   床板を 2 枚以上持つ場合に代表以外の床板の面積・荷重を無視していた（総和保存が崩れる）。
   フィクスチャでは手入力小梁ラインが import 後に残らないため顕在化しないが、複数の床板
   （打設単位）と手入力小梁ラインが両方ある区画（この Step 4 が想定する典型例そのもの）で
   静かに荷重が失われる経路だった。`uses_joist_distribution` に `region.slab_ids.len() != 1`
   を追加し、床板が 2 枚以上ある区画はこの経路を拒否して通常経路（各床板を独立に分配）へ
   落とすよう修正した。再発防止テスト
   `test_uses_joist_distribution_false_for_multiple_slabs`
   （`crates/squid-n-load/src/floor/tests.rs`）を追加した。

この 3 件目は、`snapshot_key_scalars` の初回計測（本レポート初版）で
`design.slab_checks`（25→81）以外の値がすべて動いたことの主因だった。特に剛性が
下がった影響で `eigen.T[0]` が 5.023e-1 まで伸び、`pushover.Qu` が 2.274e6 まで
落ちていた。3 件目の修正後は Step 3 の値にほぼ戻っている（下表）。

## 代表スカラの変化（`snapshot_key_scalars`）

小梁が実際に分配へ参加するようになった（Step 3 まではパネル統合により、境界に取り込まれた
小梁の曲げが分配上は無視されていた）ため、応力解析全般・保有水平耐力・時刻歴応答が動いた。
値は上記バグ 3 件をすべて直した後の最終値。

| 項目 | Step 3（統合直後） | Step 4（本記録・最終） |
|---|---|---|
| `model.floor_regions` | 26 | 26（不変。区画数は変わらない） |
| `design.slab_checks` | 25 | **81**（区画内の複数床板が個別に検定されるため） |
| `design.joist_checks` | 56 | 56（不変。対象本数は変わらない） |
| `design.joist_max_ratio` | 4.508e-1 | 3.097e-1 |
| `design.max_ratio` | 6.133e-1 | 6.352e-1 |
| `eigen.T[0]` | 4.851e-1 | 4.849e-1 |
| `eigen.T[1]` | 4.517e-1 | 4.516e-1 |
| `eigen.T[2]` | 4.096e-1 | 4.042e-1 |
| `static.DL.min_uz` [mm] | −9.289e-1 | −8.685e-1 |
| `static.DL.sum_base_axial` | −3.135e6 | −2.895e6 |
| `static.EX.story[PHRFL].max_ux` | 1.744e1 | 1.744e1 |
| `metrics[1FL].drift_angle` | 1.359e-3 | 1.359e-3 |
| `metrics[3FL].drift_angle` | 8.759e-4 | 8.759e-4 |
| `pushover.Qu` | 2.513e6 | 2.509e6 |
| `pushover.steps` | 43 | 43 |
| `holding[0].Qu` | 2.513e6 | 2.509e6 |
| `th.peak_ux` | 4.054e1 | 4.055e1 |
| `th_nl.peak_ux` | 7.130e1 | 7.173e1 |

`design.slab_checks` が 25→81 に増えたのは不具合ではなく設計どおりの変化である。
Step 3 まではパネル 1 枚＝床設計 1 件だったが、Step 4 で `Slab` 単位（区画内の打設単位ごと）に
版設計を出すようにしたため、区画あたり複数枚に分かれている床板がそれぞれ 1 件として数えられる。

`static.DL.sum_base_axial`（−3.135e6→−2.895e6、約 7.7% 減）は原因を特定できていない
（残課題）。分配経路の変化では説明できない（荷重分配は現行コードで厳密に総和保存する
ことを直接検算済み。下記「検算」参照）。厚みの相違で説明できるかも検証したが、
フィクスチャの全 82 床板は板厚が一致しており（区画をまたいでも同一）、Step 3 の
「区画（パネル）1 つを 1 枚の版として統合」が実際より重い版厚を生んでいた、という
仮説も棄却された。取り付く床板（D20 変換によるカンチレバー化）もフィクスチャには
0 枚で、そちらの面積計算の相違でも説明できない。`base_column_axials`（本レポートの
測定関数）は鉛直部材（柱）に限った軸力の合計であり、モデル全体の反力総和と一致する
という保証はない（フィクスチャにはブレース・壁要素はなく `Beam` と `PanelZone` のみ
のため、他経路への迂回という説明も考えにくいが、確証は取れていない）。Step 3 の
コードは既にこのセッションの作業対象外（本記録作成時点で参照できない）であるため、
両者の直接比較による原因特定はできなかった。数値自体は符号反転やオーダー変化はなく、
実務モデルとの照合も含めて別途検証が必要な残課題として扱う。

他の値（固有周期・変位・保有水平耐力・時刻歴）はバグ 3 件をすべて直した後は Step 3 と
ほぼ一致しており、小梁が三角形・台形分配へ実際に参加したことによる差は小さい
（符号反転・オーダー変化はない）。

## 検算: 荷重分配の総和保存

`squid_n_job::auto_loads::compute_dl_beam_loads(&model)` が返す `BeamLoad` 列の
`cmq.q_i + cmq.q_j`（＝各荷重の等価節点力の総和。単純梁反力の和は元の荷重の総量に一致する）
の総和が、`Σ over model.slabs of model.slab_dead_intensity(slab) × polygon_area(slab)` と
一致することをフィクスチャで直接検算した（`prepared()` 適用後、比 0.999999999999999）。
床領域単位の分配（`distribute_region`）・床板単独の分配（`distribute_slab_resolved`）の
いずれの経路でも、DL の総量は床板の面荷重強度×面積の総和と厳密に一致し、取りこぼしはない。

## 残す限界

- 小梁設計（`check.rs::floor_design_checks`）は依然として `design_joist_simple`/
  `design_joist_from_forces` による独自の単純梁計算であり、`distribute_region` の分配結果
  から導いてはいない（Step 5 で解消予定）
- 3 枚の床板に載る小梁（T 字取り付き）の負担幅は、載っている床板の幅の平均という近似のまま
  （`check.rs::design_secondary_joist_checks` を `model.slabs` 基準へ retarget したのみで、
  近似そのものの解消は Step 5 待ち）
- `distribute_polygon_supported` 相当（支持辺の分割伝達）の書き直しは未着手
- 取付き線の部分区間（`span != [0, 1]`）への対応は未着手。`validate` が引き続き弾く
- `static.DL.sum_base_axial` が Step 3 から約 7.7% 減った原因は未特定（上記参照）。
  厚みの相違・取り付く床板の面積相違・荷重の取りこぼしのいずれも棄却したが、
  Step 3 のコードと直接比較できないため確定できていない
- **利用者から見える機能の後退**: `Slab` は `name` を持たず、取り付く床板（片持ち・
  バルコニー等）はどの `FloorRegion` にも属さないため、取り付く床板に名前を付ける手段が
  なくなった（`squid_n_edit::AddAttachedSlab` に `name` 引数がなく、
  `tables/slabs.rs` の `SlabDraft` からも `attached_name` を削除済み）。以前は
  `AddAttachedFloorRegion`（PR #218 時点の「`FloorRegion` ＝版そのもの」設計）が
  取り付く領域の名前を受け取っていた。バルコニー・片持ちスラブを名前で識別する
  運用があるなら、`Slab` へ名前を持たせるか、取り付く床板も何らかの領域へ
  帰属させる設計を検討すること
