//! ビューアの視点操作（ドラッグ・ズーム・構面正対・ViewCube）。
//!
//! 描画領域のポインタ入力を、カメラ状態とモデルの選択・編集へ反映する部分を
//! ここへ集める。視点（[`interact_camera`]・[`interact_viewcube`]）は投影より前に
//! 確定させる必要があり、クリック処理（[`handle_click`]）は投影後の点列が要る。
//! ホバー時の強調表示は描画と一体のため、[`super::viewer_panel`] 側に残している。

use squid_n_core::frame::Frame;

use crate::app::App;

use super::camera::CameraState;
use super::pick::{member_load_pickable, pick_nearest_member, pick_nearest_node};
use super::scene::order_wall_nodes;
use super::wall_expanded_view_model;
use super::{frame_view, space_grid, viewcube, FrameFilter, Projector, ViewMode};

/// ViewCube（右上に描く方位キューブ）の当たり判定結果。
pub(super) struct ViewCubeState {
    pub layout: viewcube::Layout,
    /// 構面表示中は視点が固定のため出さない。
    pub visible: bool,
    pub hover: Option<viewcube::Hit>,
    /// キューブ上のクリックはピック処理へ流さないため、呼び出し側が参照する。
    pub clicked: bool,
}

/// ポインタ入力（[`CameraState::apply_pointer_input`]）と構面正対を反映した
/// カメラを返す。
///
/// 構面表示中は回転を禁じたうえで、その構面の法線方向へ毎フレーム正対させる。
/// 正対はこのビュー固有の扱いのため、操作の共通部分とは分けてここに置く。
pub(super) fn interact_camera(
    ui: &egui::Ui,
    response: &egui::Response,
    frame: Option<&Frame>,
    base: &CameraState,
) -> CameraState {
    let mut cam = base.clone();
    // 構面表示中は回転を禁じる（正対が崩れ、構面内に描く基準線も傾くため）。
    cam.apply_pointer_input(ui, response, frame.is_none());

    // 構面表示中は、その構面の法線方向へ毎フレーム正対させる（回転操作は上で禁じて
    // いるが、全体表示から切り替えた直後の向きもここで確定する）。
    if let Some(f) = frame {
        cam.snap_to_direction(frame_view::view_direction(f.normal));
    }
    cam
}

/// ViewCube の当たり判定と、クリックによる視点スナップ。
///
/// 面クリック＝標準ビュー／コーナークリック＝アイソメへ即時スナップする。
/// モデルより手前の固定 UI のため、当たり判定を部材ピックより先に行い、
/// キューブ上のクリックはピック処理へ流さない。
pub(super) fn interact_viewcube(
    ui: &egui::Ui,
    response: &egui::Response,
    rect: egui::Rect,
    visible: bool,
    cam: &mut CameraState,
) -> ViewCubeState {
    let layout = viewcube::Layout {
        center: egui::pos2(rect.max.x - 55.0, rect.min.y + 55.0),
        scale: 22.0,
    };
    let hover = visible
        .then(|| {
            response
                .hover_pos()
                .and_then(|p| viewcube::hit_test(cam, &layout, p))
        })
        .flatten();
    if hover.is_some() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
    }
    let mut clicked = false;
    if visible && response.clicked() {
        if let Some(pos) = response.interact_pointer_pos() {
            if let Some(hit) = viewcube::hit_test(cam, &layout, pos) {
                cam.snap_to_direction(viewcube::hit_direction(hit));
                clicked = true;
            }
        }
    }
    ViewCubeState {
        layout,
        visible,
        hover,
        clicked,
    }
}

/// クリック処理が要る描画側の文脈（投影結果と絞り込み条件）。
///
/// `pts` は全節点をこのフレームの投影で写した画面座標、`node_visible` はその
/// 表示可否（解析対象外の節点を作成モードのピック対象から外す）。
pub(super) struct ClickContext<'a> {
    pub pts: &'a [egui::Pos2],
    pub node_visible: &'a [bool],
    pub filter: FrameFilter<'a>,
    pub proj: &'a Projector<'a>,
    pub frame: Option<&'a Frame>,
    pub mode: ViewMode,
}

/// 描画領域のクリックを処理する（荷重の対象ピック・作成モードの節点選び・
/// 通常の選択）。ViewCube 上のクリックは呼び出し側で除外済み。
///
/// 呼び出し側の描画用モデル（`wall_expanded_view_model` の結果）を作る前に呼ぶ。
/// 梁・壁作成モードが `app.core.model` を可変借用するためである。
/// 通常モードの部材ピックだけは壁を展開したモデルが要るため、この中で作り直す
/// （§5.17 残→§5.31 の多角形ピック）。
pub(super) fn handle_click(app: &mut App, response: &egui::Response, ctx: ClickContext<'_>) {
    let ClickContext {
        pts,
        node_visible,
        filter,
        proj,
        frame,
        mode,
    } = ctx;
    if let Some(click_pos) = response.interact_pointer_pos() {
        if app.load_pick_active() {
            // 荷重の対象ピック待ち：節点荷重なら節点、部材荷重なら部材を仮選択する
            // （確定は Enter。案内バーは `crate::load_editor`）。
            let picks_node = app
                .ui
                .scoped
                .load_editor
                .as_ref()
                .is_some_and(|e| e.picks_node());
            if picks_node {
                // 節点ピッキング許容距離（px）
                const NODE_PICK_THRESHOLD: f32 = 10.0;
                if let Some((i, d)) = pick_nearest_node(pts, node_visible, click_pos) {
                    if d <= NODE_PICK_THRESHOLD {
                        let node_id = app.core.model.nodes[i].id;
                        if let Some(editor) = app.ui.scoped.load_editor.as_mut() {
                            editor.set_picked_node(node_id);
                        }
                        app.ui.scoped.nav.focus_node = Some(node_id);
                        app.ui.scoped.selection.nodes = vec![node_id];
                    }
                }
            } else {
                // 部材ピッキング許容距離（px）
                const PICK_THRESHOLD: f32 = 8.0;
                if let Some((id, d)) = pick_nearest_member(&app.core.model, pts, click_pos, filter)
                {
                    if d <= PICK_THRESHOLD {
                        // 壁・スラブ等の非線材には部材荷重を載せられない
                        // （`is_member_load_target` と同じ集合に限る）。
                        if member_load_pickable(&app.core.model, id) {
                            // モデルの不変借用はここで終える（`set_picked_member`
                            // へはブレースか否かの判定結果だけを渡す）。
                            let is_brace = crate::load_editor::is_brace(&app.core.model, id);
                            if let Some(editor) = app.ui.scoped.load_editor.as_mut() {
                                editor.set_picked_member(id, is_brace);
                            }
                            app.ui.scoped.nav.focus_member = Some(id);
                            app.ui.scoped.selection.members = vec![id];
                        }
                    }
                }
            }
        } else if app.ui.scoped.beam_draw_mode {
            // 梁作成モード：クリック位置を既存節点または格子点へスナップする。
            // グリッド表示が OFF のときは、見えていない格子点を拾わないよう
            // 既存節点だけを対象にする。
            // 構面表示中は格子を描かないため、スナップの対象からも外す
            // （正射影で重なった別構面の格子点を拾い、見ていない構面へ
            // 節点と梁を作ってしまう）。
            let picked = if app.ui.view.show_space_grid && frame.is_none() {
                space_grid::pick(&app.core.model, proj, pts, node_visible, click_pos)
            } else {
                // 節点ピッキング許容距離（px）
                const NODE_PICK_THRESHOLD: f32 = 10.0;
                pick_nearest_node(pts, node_visible, click_pos)
                    .filter(|(_, d)| *d <= NODE_PICK_THRESHOLD)
                    .map(|(i, _)| space_grid::SnapPoint::Node(app.core.model.nodes[i].id))
            };
            if let Some(point) = picked {
                match app.ui.scoped.beam_draw_first {
                    None => {
                        // 1 点目：始点として記憶（この時点ではモデルを変更しない）
                        app.ui.scoped.beam_draw_first = Some(point);
                    }
                    Some(first) => {
                        // 2 点目：始点と異なれば梁を生成。節点のない格子点は
                        // 節点追加とあわせて 1 回の undo にまとめる。
                        if let Some((cmd, new_id)) =
                            space_grid::beam_command(&app.core.model, first, point)
                        {
                            app.core.scoped.undo.run(&mut app.core.model, Box::new(cmd));
                            app.core.scoped.staleness.mark_edited();
                            app.ui.scoped.nav.focus_member = Some(new_id);
                        }
                        // 次の梁に備えて始点をリセット
                        app.ui.scoped.beam_draw_first = None;
                    }
                }
            }
        } else if app.ui.scoped.wall_draw_mode {
            // 壁作成モード：クリック位置に最も近い節点を選び、4 点そろったら
            // 囲まれた壁版（`AddEnclosedWallPlate`）として追加する。
            let best = pick_nearest_node(pts, node_visible, click_pos);
            // 節点ピッキング許容距離（px）
            const NODE_PICK_THRESHOLD: f32 = 10.0;
            if let Some((i, d)) = best {
                if d <= NODE_PICK_THRESHOLD {
                    let node_id = app.core.model.nodes[i].id;
                    // 同一節点の重複選択は無視
                    if !app.ui.scoped.wall_draw_nodes.contains(&node_id) {
                        app.ui.scoped.wall_draw_nodes.push(node_id);
                    }
                    // 4 点そろったら壁版を生成
                    if app.ui.scoped.wall_draw_nodes.len() == 4 {
                        let ordered =
                            order_wall_nodes(&app.core.model, &app.ui.scoped.wall_draw_nodes);
                        let mut dedup = ordered.clone();
                        dedup.sort_by_key(|n| n.0);
                        dedup.dedup();
                        if dedup.len() == 4 {
                            let section = app
                                .ui
                                .scoped
                                .wall_plate_draft
                                .add_enclosed_section
                                .filter(|sid| {
                                    app.core
                                        .model
                                        .sections
                                        .get(sid.index())
                                        .is_some_and(|s| s.thickness.is_some_and(|t| t > 0.0))
                                });
                            if app.core.scoped.undo.run(
                                &mut app.core.model,
                                Box::new(squid_n_edit::AddEnclosedWallPlate {
                                    boundary: ordered,
                                    section,
                                    opening_area: 0.0,
                                    opening_weight: 0.0,
                                }),
                            ) {
                                // 所属壁領域への結びつきは `rebuild_wall_regions` が行う。
                                // 呼ばないと壁展開の対象にならず、3D・部材表に現れない
                                // （旧 `AddMember`+`Wall` は要素へ直書きしていたため即時見えた）。
                                squid_n_core::wall_region_rebuild::rebuild_wall_regions(
                                    &mut app.core.model,
                                );
                                app.core.scoped.staleness.mark_edited();
                            }
                        }
                        app.ui.scoped.wall_draw_nodes.clear();
                    }
                }
            }
        } else if app.ui.scoped.slab_draw_mode {
            // スラブ作成モード：クリック位置に最も近い節点を外周順に追加する。
            let best = pick_nearest_node(pts, node_visible, click_pos);
            // 節点ピッキング許容距離（px）
            const NODE_PICK_THRESHOLD: f32 = 10.0;
            if let Some((i, d)) = best {
                if d <= NODE_PICK_THRESHOLD {
                    let node_id = app.core.model.nodes[i].id;
                    // 同一節点の重複選択は無視（外周は各節点1回）。
                    if !app.ui.scoped.slab_draw_nodes.contains(&node_id) {
                        app.ui.scoped.slab_draw_nodes.push(node_id);
                    }
                }
            }
        } else {
            // 通常モード：クリック位置に最も近い部材を選び、閾値内なら選択。
            // 壁は `display_model` 上の多角形ピック（§5.17 残→§5.31）。
            // ピッキング許容距離（px）
            const PICK_THRESHOLD: f32 = 8.0;
            let display_model = wall_expanded_view_model(&app.core.model);
            let frame_for_pick = app
                .ui
                .scoped
                .frame_target
                .and_then(|t| squid_n_core::frame::build_frame(display_model.as_ref(), t));
            let filter_pick = FrameFilter::new(frame_for_pick.as_ref());
            match pick_nearest_member(display_model.as_ref(), pts, click_pos, filter_pick) {
                Some((id, d)) if d <= PICK_THRESHOLD => {
                    app.ui.scoped.selection.members = vec![id];
                    app.ui.scoped.nav.focus_member = Some(id);
                    if mode == ViewMode::Hinge {
                        app.ui.scoped.hinge_detail_elem = Some(id);
                    }
                    if mode == ViewMode::TimeHistory && !app.core.scoped.staleness.results_stale {
                        app.ui.scoped.th_detail_elem = Some(id);
                    }
                }
                _ => {
                    app.ui.scoped.selection.members.clear();
                }
            }
        }
    }
}
