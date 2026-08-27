# CMQ 図を荷重ケースの全荷重へ揃える

- 作成日: 2026-08-27
- 残課題: **あり**

## 現状

CMQ 図（`ViewMode::Cmq`）の描画ソースは、選択中の荷重ケースではなく、床荷重分配の
`BeamLoad` 列である。

1. `App::refresh_beam_loads` / `sync_gravity_load_cases_action` が
   `compute_dl_beam_loads` の結果を `app.beam_loads` に書く
2. `App::cmq_display_member_loads` がそれを `slab_load_case_content` で
   `MemberLoad` へ変換する
3. `viewer/cmq.rs` が大梁ごとに C/M/Q を描く

`compute_dl_beam_loads` に入るのは、スラブ固定荷重の分配と、自立壁
（床領域アンカー）の等価面荷重（床分配へ上乗せ）だけである。解析の「DL」ケースへは
別経路で、梁・柱・囲まれた壁の自重（`self_weight_case_content`）と、取り付く壁版の
線アンカー分布（`attached_wall_beam_loads`）が加算される。そのため、取り付く壁版の
分布荷重は応力解析の DL には載るが、CMQ 図には出ない。

これは意図した設計ではない。床分配の検証図として先に配線した名残であり、
CMQ 図の改修として残している。

## あるべき姿

表示中の荷重ケースごとに、そのケースに載っている荷重をすべて反映した CMQ を描く。

- ソースはケースの `member`（および梁に載るよう変換済みの分布・集中）とする
- 床分配・梁自重・取り付く壁版の線アンカー・手入力の部材荷重を同じ図に重ねる
- ケースを切り替えたら（DL / LL(架構用) / 手入力ケースなど）図もそのケースに追従する
- 柱への節点集中だけ（`LoadTransfer::Columns`、柱軸力）は梁の C/M/Q 図には出ない
  （梁に載らない荷重なので、梁図から外れること自体は正しい）

## 残課題

| 状態 | 項目 |
|------|------|
| ☐ | CMQ 図のソースを `compute_dl_beam_loads` から、表示中荷重ケースの全部材荷重へ付け替える |
| ☐ | 荷重ケース切替（ナビ／コンボ）に CMQ 図を追従させる |
| ☐ | 取り付く壁版の線アンカー分布（等分布・線形変化）が DL の CMQ に載ることを回帰する |
| ☐ | 梁自重など、床分配以外の自動部材荷重も同じ図に載ることを回帰する |

関連: [床領域・壁領域の再設計 §5.23・§5.26](床領域・壁領域の再設計_申し送り.md)、
`crates/squid-n-app/src/app/actions/loads.rs`、`crates/squid-n-app/src/viewer/cmq.rs`、
`crates/squid-n-job/src/auto_loads.rs`。
