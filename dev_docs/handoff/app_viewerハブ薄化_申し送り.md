# app `viewer` の親モジュール分割 — 申し送り

作成日: 2026-08-13
対象コード: `crates/squid-n-app/src/viewer/`

この申し送りでは、3D ビューアの親モジュールを薄くした経緯を記録します。

`viewer/mod.rs` は 4344 行あり、画面の入口と描画の実体が同居していました。
カメラ、支持記号、CMQ 図、ピック、変形表示が、同じファイルに残っていたためです。
サブモジュールへの切り出しは始まっていました。
ですが、親が描画の振り分けを抱えたままだったため、ファイルを増やしても追いやすくはなっていませんでした。

今回は `viewer_panel` を入口として残し、別の変更理由を持つ描画だけを兄弟ファイルへ移しました。
計算内容と表示の挙動は変えていません。

## 背景

前の波で、`panels` は画面単位に分けました。
詳細は [`app_panels画面分割_申し送り.md`](app_panels画面分割_申し送り.md) を参照してください。

ビューアは画面としては 1 つなので、モード切替のような短い UI の節まで切ると、30 行程度のファイルが増えるだけになります。
親に残っていたのは、すでに独立しているはずの描画機能の取り残しです。

N/Q/M の応力図は `diagram.rs` に、支点ばねと免震の記号は `support_symbols.rs` にあります。
CMQ 図と、矢印・円弧の従来の支持記号は、これらとは直す理由が違うため、既存ファイルには混ぜませんでした。

## 変更内容

`viewer/mod.rs` から、次の兄弟ファイルへ描画を移しました。

```
viewer/
  mod.rs          — 表示モードの型、FrameFilter、Projector、viewer_panel
  camera.rs       — クォータニオンと CameraState
  support.rs      — 従来の支持記号（矢印・円弧・凡例）
  cmq.rs          — CMQ 図
  pick.rs         — ノードと部材のピック
  deform.rs       — 変形表示、スケール、未参照節点の変位補間
  playback.rs     — 時刻歴再生の時刻とフレーム
  scene.rs        — 部材の描き方、スラブと壁、グリッド、軸ガジェット
```

`mod.rs` は 4344 行から 1877 行になりました。
残しているのは、表示モードの型、`FrameFilter`、`Projector`、`viewer_panel`、再生 UI の `frame_range_controls` です。
`viewer_panel` と再生 UI は、画面の入口の一部として親に置いています。

`crate::viewer::ViewMode` や `CameraState`、`ForceComponents`、`TimeHistoryScaleCache` といった公開パスは変えていません。
親が再エクスポートするのは、クレート外から名前で触る `CameraState` と `TimeHistoryScaleCache` の 2 型だけです。
兄弟どうしは、実モジュールから import します。
たとえば ViewCube は `super::camera::q_rotate` を、格子スナップは `super::pick::pick_nearest_node` を直接指します。
部材長は `squid_n_core::geom::vec3::dist` を各ファイルが別名します。
親を経由する必要がないためです。

別ファイルへ移した private 関数は、親や兄弟から呼ぶために `pub(super)` へ上げました。
計算内容は変えていません。
`mn_view` が使う `CameraState::turntable_drag` と `project` だけは、従来どおり `pub(crate)` です。

上げた対象は次のとおりです。

- `support.rs`：`SupportKind`、`support_kind`、`draw_arrow`、`axis_basis`、`draw_rotation_arc`、`draw_support_symbol`、`draw_support_legend`
- `scene.rs`：`diagram_offset_dir`、`in_plane_offset_dir`、`draws_as_line`、`DrawShape`、`element_draw_shape`、`draw_slabs_and_joists`、`order_wall_nodes`、`draw_grid_and_axes`、`draw_axis_gadget`
- `pick.rs`：`pick_nearest_node`、`pick_nearest_member`、`member_load_pickable`
- `playback.rs`：`advance_play_time`、`frame_at_time`
- `cmq.rs`：`paint_diagram_polygon`、`draw_cmq_diagram`
- `deform.rs`：`BeamDeflection`、`display_disp`、`deform_display_scale`、`time_history_deform_scale`、`model_bbox` など

クォータニオンの合成（`q_axis_angle` / `q_mul` / `q_norm`）は `camera.rs` の非公開関数です。
回転の適用 `q_rotate` だけを兄弟へ出しています。
`mn_view` は `CameraState` と `project` だけを使うため、合成関数をクレートへ公開する理由がありません。

カメラのテストと再生のテストは、それぞれの機能ファイルへ移しました。
`FrameFilter` と `in_plane_offset_dir` のテストは、入口の絞り込み API の確認として親に残しています。

## 設計判断

- **入口は 1 つのままにする。** 表示モードごとにディレクトリを切ると、モード追加のたびに階層が増える
- **CMQ は `diagram.rs` に混ぜない。** N/Q/M の応力図と CMQ 図では、直す理由が違う
- **従来の支持記号は `support_symbols.rs` に混ぜない。** 後者は支点ばねと免震の専用記号
- **テストは機能ファイルへ移す。** カメラと再生は移動し、構面フィルタは親に残す

表示モードでディレクトリを切ると、`actions` の解析種別と軸を揃えたくなるかもしれません。
ですが、ビューアの変更理由は「どの図を直すか」であり、「どの解析を実行するか」ではないため、軸は揃えませんでした。

## 意図的にやらないこと（残課題）

本波（GUI のファイル分割）の対象だった 3 点は、これで揃いました。
解析ジョブの分割は [`app_actions解析種別分割_申し送り.md`](app_actions解析種別分割_申し送り.md) を参照してください。

本波の対象外は、次のとおりです。

- `AppCore` / `UiState` への型分離（専用ブランチ）
- app／MCP の設計入口を `squid-n-job` へ寄せること（機能統合）
- 巨大テストファイルの分割
- `hinge.rs` / `modeling.rs` / `check_ratio.rs` など、すでに 1 機能で太いファイルの再分割

## まとめ

親モジュールを薄くしても、画面の入口は 1 ファイルに残しています。
描画の実体は、変更理由ごとに兄弟ファイルへ出しています。
解析ジョブの分割は [`app_actions解析種別分割_申し送り.md`](app_actions解析種別分割_申し送り.md) です。

## 検証

- `cargo fmt -p squid-n-app`
- `cargo check -p squid-n-app --features gui`
- `cargo clippy -p squid-n-app --all-targets --features gui --locked -- -D warnings`
- `cargo test -p squid-n-app --features gui --lib viewer`（183 passed）
