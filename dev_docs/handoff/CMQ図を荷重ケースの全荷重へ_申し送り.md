# CMQ 図を荷重ケースの全荷重へ揃える

- 作成日: 2026-08-27
- 残課題: —

## 現状（改修前）

CMQ 図（`ViewMode::Cmq`）の描画ソースは、選択中の荷重ケースではなく、床荷重分配の
`BeamLoad` 列だった。

1. `App::refresh_beam_loads` / `sync_gravity_load_cases_action` が
   `compute_dl_beam_loads` の結果を `app.beam_loads` に書く
2. `App::cmq_display_member_loads` がそれを `slab_load_case_content` で
   `MemberLoad` へ変換する
3. `viewer/cmq.rs` が大梁ごとに C/M/Q を描く

`compute_dl_beam_loads` に入るのは、スラブ固定荷重の分配と、自立壁
（床領域アンカー）の等価面荷重（床分配へ上乗せ）だけだった。解析の「DL」ケースへは
別経路で、梁・柱・囲まれた壁の自重（`self_weight_case_content`）と、取り付く壁版の
線アンカー分布（`attached_wall_beam_loads`）が加算される。そのため、取り付く壁版の
分布荷重は応力解析の DL には載るが、CMQ 図には出ない、という不整合があった。

## 改修内容（2026-08-27、dig による設計確定）

利用者との dig で、単に「ソースを付け替える」だけでなく、応力図と同じ「軸」の
区別を CMQ 図にも持たせる設計へ広げた（下記 Q3 参照）。

### ソースの付け替え

CMQ 図のソースを `app.beam_loads`（DL 専用の中間表現）から、**表示中荷重ケースの
`member`（`LoadCase.member`）そのもの**へ付け替えた。

- 表示中ケースは `nav.focus_load_case`（荷重タブ・ナビゲータの荷重ケース一覧が
  既に持っている選択状態）をそのまま使う。応力図の `nav.focus_result` は使わない
  ── これは `app.results`（解析実行結果）からしか作られず解析未実行時は空になるが、
  CMQ 図は解析未実行でも使える診断図という現状の性質を保つため
  （`App::cmq_display_load_case`）
- 未選択時は静解析の対象決定（`resolved_analysis_target`）と同じ規約で先頭の
  荷重ケースへフォールバックする
- CMQ 図の画面には新しいケース選択 UI を追加しない。ナビゲータの荷重ケース一覧が
  既にその役目を果たしているため（現在のケース名は案内ラベルとして表示する）
- `LoadCase.member` は既に `sync_gravity_load_cases_action` が床分配・自重・取り付く
  壁版の線アンカー・（自動同期対象外の）手入力荷重をすべて合成した内容を持つため、
  ソースをそのまま読むだけで「あるべき姿」（床分配・梁自重・取り付く壁版の線アンカー・
  手入力の部材荷重を同じ図に重ねる）を満たす

### 軸（強軸 ey・弱軸 ez）の追加

**当初案（CMQ 図は強軸曲げのみを表示する）は、利用者からの指摘で撤回した。**
部材の局所軸は必ずしも「ey が鉛直・ez が水平」ではない（斜め柱、ひねりのある
断面の柱など）。この場合、鉛直下向きの荷重でも局所 ey 面・ez 面の両方に投影成分が
生じ、これは近似の誤差ではなく物理的に正しい現象である。強軸のみに絞ると、
そうしたモデルで弱軸側の荷重伝達が CMQ 図から欠落する。

そこで応力図（`ForceComponent::plane`）と同じ「面」の区別を CMQ 図にも導入した。

- `MemberLoad.dir`（世界座標の作用方向）を、要素の局所軸（`LocalFrame::from_nodes`
  が返す `rot[1]`=ey・`rot[2]`=ez。ビューアの `diagram_offset_dir` と同じ計算）へ
  投影してから C/M/Q を計算する（`project_load`）
- 「軸:」チェックボックスで強軸・弱軸を独立にON/OFFできる。既定は強軸のみ
  （直交グリッド・ひねりのない部材が大半で、弱軸はほぼ0になるモデルが多いため）
- 両軸を同時表示するときは同一スケールで重ねて描く（応力図の同単位成分の
  共有スケールと同じ規約）

### 死コードの一掃

CMQ 図の描画ソース付け替えにより、`app.beam_loads`／`refresh_beam_loads`／
`cmq_display_member_loads`／`AutoLoadComputeResult.dl_beam_loads` は他に消費者が
なくなったため削除した（`compute_dl_beam_loads` 関数自体は `compute_gravity_auto_load_cases`
の内部処理として存続）。対応する既存テスト
（`test_refresh_beam_loads_*`・`test_sync_gravity_invalidates_beam_loads_hash`）も削除した。

## 残課題

なし。実装・回帰テストとも完了。

関連: [床領域・壁領域の再設計 §5.23・§5.26](床領域・壁領域の再設計_申し送り.md)、
`crates/squid-n-app/src/app/actions/loads.rs`、`crates/squid-n-app/src/viewer/cmq.rs`、
`crates/squid-n-app/src/viewer/mod.rs`、`crates/squid-n-job/src/auto_loads.rs`。
