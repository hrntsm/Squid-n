use crate::app::node_grid::NodeGridAdapter;
use crate::app::{App, LogLevel};
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::NodeId;
use squid_n_edit::{AddNode, SetNodeRestraint};

pub fn nodes_table(ui: &mut egui::Ui, app: &mut App) {
    // 節点追加フォーム（座標のみを扱う。境界条件は別パネルで編集する）。
    // 座標を入力してから「追加」を押すことで、その座標を持つ節点を作成する。
    ui.group(|ui| {
        ui.strong("節点を追加");
        // 左パネルが狭い場合でも「追加」ボタンが見切れないよう折り返す
        ui.horizontal_wrapped(|ui| {
            for (label, k) in [("X", 0), ("Y", 1), ("Z", 2)] {
                ui.label(label);
                let slot = &mut app.node_draft[k];
                let resp = ui.add(
                    egui::TextEdit::singleline(slot)
                        .desired_width(70.0)
                        .clip_text(false),
                );
                if slot.trim().parse::<f64>().is_err() {
                    ui.painter().rect_filled(
                        resp.rect,
                        0.0,
                        crate::theme::translucent(crate::theme::ERROR_RED, 60),
                    );
                }
            }
            if ui.button("+ 追加").clicked() {
                let mut coord = [0.0; 3];
                for (k, slot) in app.node_draft.iter().enumerate() {
                    coord[k] = slot.trim().parse::<f64>().unwrap_or(0.0);
                }
                // 同一座標の既存節点がある場合は確認ダイアログを挟む
                // （同じ座標の節点を重複して作成してよいかユーザに確認する）
                const COORD_TOL: f64 = 1e-9;
                let dup = app.model.nodes.iter().any(|n| {
                    (n.coord[0] - coord[0]).abs() < COORD_TOL
                        && (n.coord[1] - coord[1]).abs() < COORD_TOL
                        && (n.coord[2] - coord[2]).abs() < COORD_TOL
                });
                if dup {
                    app.pending_duplicate_node_coord = Some(coord);
                } else {
                    app.undo.run(
                        &mut app.model,
                        Box::new(AddNode {
                            coord,
                            restraint: Dof6Mask::FREE,
                        }),
                    );
                    // model.nodes が +1 されたので node_edit の長さを再同期
                    // （同期しないと body.rows が新しい行数で描画し node_edit[i] が範囲外になる）
                    app.sync_node_edit();
                    app.staleness.mark_edited();
                }
            }
        });
    });
    ui.separator();

    // 座標 3 列はグリッド操作レイヤ（スプレッドシート的編集。T4 パイロット）。
    // 矩形選択・Excel 相互 TSV コピペ・新規行プレースホルダ・行削除に対応し、
    // モデル編集はアダプタが squid-n-edit の複合コマンドへ落とす（undo 1 回で復元）。
    let edited = {
        let mut adapter = NodeGridAdapter {
            model: &mut app.model,
            undo: &mut app.undo,
            edited: false,
        };
        // 既存の 🗑 ボタン（1 行削除）はグリッドの末尾列として維持する
        app.node_grid.delete_buttons = true;
        app.node_grid.show(ui, &mut adapter, &["X", "Y", "Z"]);
        adapter.edited
    };
    for (msg, is_err) in app.node_grid.take_log() {
        app.log.push(
            if is_err {
                LogLevel::Error
            } else {
                LogLevel::Info
            },
            msg,
        );
    }
    if edited {
        // 編集があった場合は下流（結果・設計）を stale にする（UI設計 §5）
        app.staleness.mark_edited();
        app.sync_node_edit();
    }
    // 行選択に合わせてナビゲータのフォーカス節点を同期する
    // （境界条件タブ・3D ビューの強調表示が選択行を追う）
    if app.node_grid.grid.active {
        let r = app.node_grid.grid.anchor.row;
        if let Some(node) = app.model.nodes.get(r) {
            app.nav.focus_node = Some(node.id);
        }
    }

    // 重複座標の節点追加確認ダイアログ
    // （追加ボタン押下時に同一座標の既存節点が見つかった場合、ここで確認を取る）
    if app.pending_duplicate_node_coord.is_some() {
        let mut do_add = false;
        let mut do_cancel = false;
        let mut open = true;
        egui::Window::new("節点座標の重複")
            .title_bar(true)
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .open(&mut open)
            .show(ui.ctx(), |ui| {
                if let Some(coord) = app.pending_duplicate_node_coord {
                    ui.label(format!(
                        "({:.3}, {:.3}, {:.3}) と同じ座標の節点がすでに存在します。",
                        coord[0], coord[1], coord[2]
                    ));
                }
                ui.label("本当にこの節点を追加しますか？");
                ui.horizontal(|ui| {
                    if ui.button("追加する").clicked() {
                        do_add = true;
                    }
                    if ui.button("キャンセル").clicked() {
                        do_cancel = true;
                    }
                });
            });
        // 閉じるボタン（×）またはキャンセルで保留を破棄
        if !open || do_cancel {
            app.pending_duplicate_node_coord = None;
        }
        // 追加確定
        if do_add {
            if let Some(coord) = app.pending_duplicate_node_coord.take() {
                app.undo.run(
                    &mut app.model,
                    Box::new(AddNode {
                        coord,
                        restraint: Dof6Mask::FREE,
                    }),
                );
                app.sync_node_edit();
                app.staleness.mark_edited();
            }
        }
    }
}

/// 境界条件（拘束）タブ：節点一覧・追加フォームとは別の独立したサブタブ。
/// 節点を選んでから 自由／ピン／固定 やチェックボックスで拘束成分を設定する。
pub fn boundary_condition_panel(ui: &mut egui::Ui, app: &mut App) {
    if app.model.nodes.is_empty() {
        ui.label("節点がありません（先に「節点」タブで節点を追加してください）");
        return;
    }

    let node_ids: Vec<NodeId> = app.model.nodes.iter().map(|n| n.id).collect();
    let selected = app
        .nav
        .focus_node
        .filter(|id| node_ids.contains(id))
        .unwrap_or(node_ids[0]);
    app.nav.focus_node = Some(selected);

    ui.horizontal(|ui| {
        ui.label("対象節点:");
        egui::ComboBox::from_id_salt("bc_node_select")
            .selected_text(format!("N{}", selected.0))
            .show_ui(ui, |ui| {
                for id in &node_ids {
                    if ui
                        .selectable_label(selected == *id, format!("N{}", id.0))
                        .clicked()
                    {
                        app.nav.focus_node = Some(*id);
                    }
                }
            });
    });
    ui.separator();

    let selected = app.nav.focus_node.unwrap_or(selected);
    let Some(node) = app.model.nodes.iter().find(|n| n.id == selected) else {
        return;
    };
    let r = node.restraint;
    let mut pending_restraint: Option<Dof6Mask> = None;

    ui.horizontal(|ui| {
        // プリセットボタン（自由／ピン／固定）
        if ui.small_button("自由").clicked() {
            pending_restraint = Some(Dof6Mask::FREE);
        }
        if ui.small_button("ピン").clicked() {
            pending_restraint = Some(Dof6Mask::PINNED);
        }
        if ui.small_button("固定").clicked() {
            pending_restraint = Some(Dof6Mask::FIXED);
        }
    });
    ui.horizontal_wrapped(|ui| {
        // 各成分チェックボックス
        use squid_n_core::dof::Dof;
        for (d, lbl) in [
            (Dof::Ux, "X"),
            (Dof::Uy, "Y"),
            (Dof::Uz, "Z"),
            (Dof::Rx, "RX"),
            (Dof::Ry, "RY"),
            (Dof::Rz, "RZ"),
        ] {
            let mut on = r.is_fixed(d);
            if ui.checkbox(&mut on, lbl).changed() {
                let mut new_mask = r;
                new_mask.set(d, on);
                pending_restraint = Some(new_mask);
            }
        }
    });

    if let Some(mask) = pending_restraint {
        app.undo.run(
            &mut app.model,
            Box::new(SetNodeRestraint {
                node: selected,
                restraint: mask,
            }),
        );
        app.staleness.mark_edited();
    }
}
