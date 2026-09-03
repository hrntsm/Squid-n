use crate::app::node_grid::NodeGridAdapter;
use crate::app::{App, LogLevel};
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::NodeId;
use squid_n_core::model::{IsolatorKind, IsolatorProps};
use squid_n_core::units::to_display::{force_kn, stiffness_kn_per_mm};
use squid_n_core::units::to_internal;
use squid_n_edit::{
    AddNode, PlaceSupportIsolator, RemoveSupportIsolator, SetNodeRestraint, SetNodeSupportSpring,
};

/// 免震支承の配置フォーム（境界条件パネル）のドラフト状態。
/// `PlaceSupportIsolator` へ渡す諸元をフォーム上で保持する（節点非依存。
/// どの節点を選んでいても同じ入力中の諸元を使い回す「作成フォーム」）。
#[derive(Clone, Debug, Default)]
pub struct IsolatorSupportDraft {
    pub props: IsolatorProps,
}

/// 免震支承種別の日本語表示名（各免震部材指針の呼称）。
pub fn isolator_kind_label(kind: IsolatorKind) -> &'static str {
    match kind {
        IsolatorKind::LaminatedRubber => "天然ゴム系積層ゴム",
        IsolatorKind::LeadRubber => "鉛プラグ入り積層ゴム(LRB)",
        IsolatorKind::HighDampingRubber => "高減衰ゴム系積層ゴム(HDR)",
        IsolatorKind::ElasticSliding => "弾性すべり支承",
    }
}

/// 免震支承種別セレクタ（4種別をボタン列で選択）。
pub fn isolator_kind_selector(ui: &mut egui::Ui, kind: &mut IsolatorKind) {
    ui.horizontal_wrapped(|ui| {
        ui.label("支承種別:");
        for k in [
            IsolatorKind::LaminatedRubber,
            IsolatorKind::LeadRubber,
            IsolatorKind::HighDampingRubber,
            IsolatorKind::ElasticSliding,
        ] {
            if ui
                .selectable_label(*kind == k, isolator_kind_label(k))
                .clicked()
            {
                *kind = k;
            }
        }
    });
}

/// `IsolatorProps` の諸元入力。種別に応じて関係するフィールドのみ表示する
/// （すべり支承: K1・μ・N長期軸力・Kv／積層ゴム系: K1・K2・Qd・Kv・本数・
/// ゴム総厚＋任意の歪依存係数）。`id_source` は CollapsingHeader の id 衝突回避用
/// （同じ関数が複数箇所〔境界条件パネル・部材タブの免震支承追加フォーム〕から
/// 呼ばれるため）。
///
/// 入力表示は K1/K2/Kv=kN/mm・Qd=kN（免震一覧 `tables::members::isolators_table`
/// と統一）。`IsolatorProps` 自体は N/mm・N 単位で保持するため、
/// `to_display` / `to_internal` で換算する。
pub fn isolator_props_fields(ui: &mut egui::Ui, id_source: &str, props: &mut IsolatorProps) {
    ui.horizontal_wrapped(|ui| {
        ui.label("Kv 鉛直剛性[kN/mm]:");
        let mut kv_kn = stiffness_kn_per_mm(props.kv);
        if ui
            .add(
                egui::DragValue::new(&mut kv_kn)
                    .speed(1.0)
                    .range(0.0..=1.0e9),
            )
            .changed()
        {
            props.kv = to_internal::stiffness_kn_per_mm(kv_kn);
        }
    });
    match props.kind {
        IsolatorKind::ElasticSliding => {
            ui.horizontal_wrapped(|ui| {
                ui.label("K1 すべり前剛性[kN/mm]:");
                let mut k1_kn = stiffness_kn_per_mm(props.k1);
                if ui
                    .add(
                        egui::DragValue::new(&mut k1_kn)
                            .speed(1.0)
                            .range(0.0..=1.0e6),
                    )
                    .changed()
                {
                    props.k1 = to_internal::stiffness_kn_per_mm(k1_kn);
                }
                ui.label("μ 摩擦係数:");
                ui.add(
                    egui::DragValue::new(&mut props.mu)
                        .speed(0.01)
                        .range(0.0..=2.0),
                );
                ui.label("N 長期軸力[kN]（圧縮正、摩擦力算定用）:");
                let mut n_kn = force_kn(props.n_long);
                if ui
                    .add(
                        egui::DragValue::new(&mut n_kn)
                            .speed(1.0)
                            .range(0.0..=1.0e7),
                    )
                    .changed()
                {
                    props.n_long = to_internal::force_kn(n_kn);
                }
            });
        }
        IsolatorKind::LaminatedRubber
        | IsolatorKind::LeadRubber
        | IsolatorKind::HighDampingRubber => {
            ui.horizontal_wrapped(|ui| {
                ui.label("K1 初期(弾性)剛性[kN/mm]:");
                let mut k1_kn = stiffness_kn_per_mm(props.k1);
                if ui
                    .add(
                        egui::DragValue::new(&mut k1_kn)
                            .speed(1.0)
                            .range(0.0..=1.0e6),
                    )
                    .changed()
                {
                    props.k1 = to_internal::stiffness_kn_per_mm(k1_kn);
                }
                ui.label("K2 二次剛性[kN/mm]:");
                let mut k2_kn = stiffness_kn_per_mm(props.k2);
                if ui
                    .add(
                        egui::DragValue::new(&mut k2_kn)
                            .speed(0.1)
                            .range(0.0..=1.0e6),
                    )
                    .changed()
                {
                    props.k2 = to_internal::stiffness_kn_per_mm(k2_kn);
                }
                ui.label("Qd 特性耐力[kN]:");
                let mut qd_kn = force_kn(props.qd);
                if ui
                    .add(
                        egui::DragValue::new(&mut qd_kn)
                            .speed(1.0)
                            .range(0.0..=1.0e6),
                    )
                    .changed()
                {
                    props.qd = to_internal::force_kn(qd_kn);
                }
            });
            ui.horizontal_wrapped(|ui| {
                ui.label("マルチシアスプリング本数 n:");
                let mut n = props.n_springs;
                if ui.add(egui::DragValue::new(&mut n).range(1..=64)).changed() {
                    props.n_springs = n;
                }
                ui.label("ゴム総厚 H[mm]（歪依存判定用。0で歪依存を無効化）:");
                ui.add(
                    egui::DragValue::new(&mut props.total_rubber_thickness)
                        .speed(1.0)
                        .range(0.0..=10000.0),
                );
            });
            if props.total_rubber_thickness > 0.0 {
                egui::CollapsingHeader::new("歪依存係数（任意・詳細）")
                    .default_open(false)
                    .id_salt((id_source, "isolator_strain_dep"))
                    .show(ui, |ui| {
                        ui.label(
                            "CKd(γ)=c0+c1・γ+c2・γ²（二次剛性K2の歪依存）／\
                             CQd(γ)=c0+c1・γ+c2・γ²（特性耐力Qdの歪依存）。\
                             γ=δ/H（各免震部材の製品技術資料）。既定[1,0,0]は歪依存なし。",
                        );
                        ui.horizontal_wrapped(|ui| {
                            ui.label("CKd c0,c1,c2:");
                            for v in &mut props.ckd_gamma {
                                ui.add(egui::DragValue::new(v).speed(0.01));
                            }
                        });
                        ui.horizontal_wrapped(|ui| {
                            ui.label("CQd c0,c1,c2:");
                            for v in &mut props.cqd_gamma {
                                ui.add(egui::DragValue::new(v).speed(0.01));
                            }
                        });
                    });
            }
        }
    }
}

pub fn nodes_table(ui: &mut egui::Ui, app: &mut App) {
    // 節点追加フォーム（座標のみを扱う。境界条件は別パネルで編集する）。
    // 座標を入力してから「追加」を押すことで、その座標を持つ節点を作成する。
    ui.group(|ui| {
        ui.strong("節点を追加");
        // 左パネルが狭い場合でも「追加」ボタンが見切れないよう折り返す
        ui.horizontal_wrapped(|ui| {
            for (label, k) in [("X", 0), ("Y", 1), ("Z", 2)] {
                ui.label(label);
                let slot = &mut app.ui.scoped.node_draft[k];
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
                for (k, slot) in app.ui.scoped.node_draft.iter().enumerate() {
                    coord[k] = slot.trim().parse::<f64>().unwrap_or(0.0);
                }
                // 同一座標の既存節点がある場合は確認ダイアログを挟む
                // （同じ座標の節点を重複して作成してよいかユーザに確認する）
                const COORD_TOL: f64 = 1e-9;
                let dup = app.core.model.nodes.iter().any(|n| {
                    (n.coord[0] - coord[0]).abs() < COORD_TOL
                        && (n.coord[1] - coord[1]).abs() < COORD_TOL
                        && (n.coord[2] - coord[2]).abs() < COORD_TOL
                });
                if dup {
                    app.ui.scoped.pending_duplicate_node_coord = Some(coord);
                } else {
                    app.core.scoped.undo.run(
                        &mut app.core.model,
                        Box::new(AddNode {
                            coord,
                            restraint: Dof6Mask::FREE,
                        }),
                    );
                    // model.nodes が +1 されたので node_edit の長さを再同期
                    // （同期しないと body.rows が新しい行数で描画し node_edit[i] が範囲外になる）
                    app.sync_node_edit();
                    app.core.scoped.staleness.mark_edited();
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
            model: &mut app.core.model,
            undo: &mut app.core.scoped.undo,
            edited: false,
        };
        // 既存の 🗑 ボタン（1 行削除）はグリッドの末尾列として維持する
        app.ui.scoped.node_grid.delete_buttons = true;
        app.ui
            .scoped
            .node_grid
            .show(ui, &mut adapter, &["X", "Y", "Z"]);
        adapter.edited
    };
    for (msg, is_err) in app.ui.scoped.node_grid.take_log() {
        app.core.log.push(
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
        app.core.scoped.staleness.mark_edited();
        app.sync_node_edit();
    }
    // 行選択に合わせてナビゲータのフォーカス節点を同期する
    // （境界条件タブ・3D ビューの強調表示が選択行を追う）
    if app.ui.scoped.node_grid.grid.active {
        let r = app.ui.scoped.node_grid.grid.anchor.row;
        if let Some(node) = app.core.model.nodes.get(r) {
            app.ui.scoped.nav.focus_node = Some(node.id);
        }
    }

    // 重複座標の節点追加確認ダイアログ
    // （追加ボタン押下時に同一座標の既存節点が見つかった場合、ここで確認を取る）
    if app.ui.scoped.pending_duplicate_node_coord.is_some() {
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
                if let Some(coord) = app.ui.scoped.pending_duplicate_node_coord {
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
            app.ui.scoped.pending_duplicate_node_coord = None;
        }
        // 追加確定
        if do_add {
            if let Some(coord) = app.ui.scoped.pending_duplicate_node_coord.take() {
                app.core.scoped.undo.run(
                    &mut app.core.model,
                    Box::new(AddNode {
                        coord,
                        restraint: Dof6Mask::FREE,
                    }),
                );
                app.sync_node_edit();
                app.core.scoped.staleness.mark_edited();
            }
        }
    }
}

/// 境界条件（拘束）タブ：節点一覧・追加フォームとは別の独立したサブタブ。
/// 節点を選んでから 自由／ピン／固定 やチェックボックスで拘束成分を設定する。
pub fn boundary_condition_panel(ui: &mut egui::Ui, app: &mut App) {
    if app.core.model.nodes.is_empty() {
        ui.label("節点がありません（先に「節点」タブで節点を追加してください）");
        return;
    }

    let node_ids: Vec<NodeId> = app.core.model.nodes.iter().map(|n| n.id).collect();
    let selected = app
        .ui
        .scoped
        .nav
        .focus_node
        .filter(|id| node_ids.contains(id))
        .unwrap_or(node_ids[0]);
    app.ui.scoped.nav.focus_node = Some(selected);

    // ノード表示ラベル（ばね支持中の節点には「🌀ばね」バッジを付ける）
    let node_label = |id: NodeId| -> String {
        let has_spring = app
            .core
            .model
            .node(id)
            .is_some_and(|n| n.support_spring.is_some());
        if has_spring {
            format!("N{} 🌀ばね", id.0)
        } else {
            format!("N{}", id.0)
        }
    };

    ui.horizontal(|ui| {
        ui.label("対象節点:");
        egui::ComboBox::from_id_salt("bc_node_select")
            .selected_text(node_label(selected))
            .show_ui(ui, |ui| {
                for id in &node_ids {
                    if ui
                        .selectable_label(selected == *id, node_label(*id))
                        .clicked()
                    {
                        app.ui.scoped.nav.focus_node = Some(*id);
                    }
                }
            });
    });
    ui.separator();

    let selected = app.ui.scoped.nav.focus_node.unwrap_or(selected);
    let Some(node) = app.core.model.node(selected) else {
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
        app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(SetNodeRestraint {
                node: selected,
                restraint: mask,
            }),
        );
        app.core.scoped.staleness.mark_edited();
    }

    ui.separator();
    support_spring_section(ui, app, selected);
    ui.separator();
    isolator_support_section(ui, app, selected);
}

/// 「ばね支持」節：対象節点の支点ばね（全体座標系6成分）を編集する。
/// 拘束（`restraint`）で固定済みの成分は入力を無効化し「(固定)」と表示する
/// （`Node::support_spring` の仕様：固定成分のばね値は解析側で無視されるため）。
fn support_spring_section(ui: &mut egui::Ui, app: &mut App, node_id: NodeId) {
    egui::CollapsingHeader::new("ばね支持")
        .default_open(false)
        .id_salt("bc_spring_section")
        .show(ui, |ui| {
            let Some(node) = app.core.model.node(node_id) else {
                return;
            };
            let restraint = node.restraint;
            let mut enabled = node.support_spring.is_some();
            let mut spring = node.support_spring.unwrap_or([0.0; 6]);

            if ui.checkbox(&mut enabled, "ばね支持を有効化").changed() {
                let new_spring = if enabled { Some(spring) } else { None };
                app.core.scoped.undo.run(
                    &mut app.core.model,
                    Box::new(SetNodeSupportSpring {
                        node: node_id,
                        spring: new_spring,
                    }),
                );
                app.core.scoped.staleness.mark_edited();
                return;
            }
            if !enabled {
                ui.colored_label(crate::theme::GRAY_600, "無効（自由 or 固定のみ）");
                return;
            }

            use squid_n_core::dof::Dof;
            // ドラッグ中は毎フレーム `changed()` が真になるため、フレームごとに
            // コマンドを発行すると undo スタックを大量消費する。ドラッグ終了
            // （またはテキスト入力後のフォーカス喪失）で確定する。
            let mut commit = false;
            ui.horizontal_wrapped(|ui| {
                for (i, (d, label)) in [
                    (Dof::Ux, "Kx[N/mm]"),
                    (Dof::Uy, "Ky[N/mm]"),
                    (Dof::Uz, "Kz[N/mm]"),
                    (Dof::Rx, "KRx[N·mm/rad]"),
                    (Dof::Ry, "KRy[N·mm/rad]"),
                    (Dof::Rz, "KRz[N·mm/rad]"),
                ]
                .into_iter()
                .enumerate()
                {
                    let fixed = restraint.is_fixed(d);
                    ui.label(label);
                    let resp = ui.add_enabled(
                        !fixed,
                        egui::DragValue::new(&mut spring[i])
                            .speed(10.0)
                            .range(0.0..=1.0e12),
                    );
                    if fixed {
                        ui.colored_label(crate::theme::GRAY_600, "(固定)");
                    }
                    if resp.drag_stopped() || resp.lost_focus() {
                        commit = true;
                    }
                }
            });
            if commit {
                app.core.scoped.undo.run(
                    &mut app.core.model,
                    Box::new(SetNodeSupportSpring {
                        node: node_id,
                        spring: Some(spring),
                    }),
                );
                app.core.scoped.staleness.mark_edited();
            }
        });
}

/// 「免震支承の配置」節：対象節点に零長 Isolator 要素＋接地節点を設置する
/// （`PlaceSupportIsolator`）。既に配置済み（対象節点に接続する零長 Isolator
/// 要素がある）場合は諸元の要約のみ表示する（多重設置を避けるため入力フォームは
/// 出さない。取り消しは undo で行う）。
fn isolator_support_section(ui: &mut egui::Ui, app: &mut App, node_id: NodeId) {
    egui::CollapsingHeader::new("免震支承の配置")
        .default_open(false)
        .id_salt("bc_isolator_section")
        .show(ui, |ui| {
            let existing_elem = find_support_isolator(&app.core.model, node_id);

            if let Some(elem_id) = existing_elem {
                let props = app
                    .core.model
                    .isolator_attrs
                    .iter()
                    .find(|a| a.elem == elem_id)
                    .map(|a| a.props);
                match props {
                    Some(p) => {
                        ui.colored_label(
                            crate::theme::GOOD_GREEN,
                            format!(
                                "配置済み（要素#{}）: {} K1={:.0}kN/mm K2={:.0}kN/mm \
                                 Qd={:.1}kN Kv={:.0}kN/mm μ={:.3}",
                                elem_id.0,
                                isolator_kind_label(p.kind),
                                stiffness_kn_per_mm(p.k1),
                                stiffness_kn_per_mm(p.k2),
                                force_kn(p.qd),
                                stiffness_kn_per_mm(p.kv),
                                p.mu
                            ),
                        );
                        ui.label(
                            "諸元の変更は「部材」タブの免震支承一覧から行ってください。",
                        );
                        if ui
                            .button("撤去")
                            .on_hover_text(
                                "接地節点・免震支承要素を削除し、対象節点を直接支点（拘束固定）へ戻します（undo可）",
                            )
                            .clicked()
                        {
                            app.core.scoped.undo.run(
                                &mut app.core.model,
                                Box::new(RemoveSupportIsolator { node: node_id }),
                            );
                            app.core.scoped.staleness.mark_edited();
                        }
                    }
                    None => {
                        ui.colored_label(crate::theme::ERROR_RED, "免震支承の諸元が見つかりません");
                    }
                }
                return;
            }

            ui.label(
                "この節点を免震支承で支持します（同一座標に接地節点を新規作成し、\
                 零長の免震支承要素を設置。対象節点の拘束は自動的に解放されます）。",
            );
            isolator_kind_selector(ui, &mut app.ui.scoped.isolator_support_draft.props.kind);
            isolator_props_fields(
                ui,
                "bc_isolator_support",
                &mut app.ui.scoped.isolator_support_draft.props,
            );
            if ui
                .button("この支点に免震支承を配置")
                .on_hover_text(
                    "接地節点＋零長の免震支承要素を追加し、対象節点の拘束を解放します（undo可）",
                )
                .clicked()
            {
                app.core.scoped.undo.run(
                    &mut app.core.model,
                    Box::new(PlaceSupportIsolator {
                        node: node_id,
                        props: app.ui.scoped.isolator_support_draft.props,
                    }),
                );
                app.core.scoped.staleness.mark_edited();
            }
        });
}

/// 対象節点 `node_id` に設置済みの支点免震支承（零長 Isolator 要素）を探す。
/// `PlaceSupportIsolator` が生成する要素の形（対象節点と同一座標の接地節点との
/// 2節点、零長、接地節点は `restraint=FIXED` かつ孤立）を満たす `Isolator` 要素が
/// あればその `ElemId` を返す（純関数。`Model::support_isolator_ends` に委譲）。
///
/// `node_id` が接地節点（FIXED側）自身の場合は `None` を返す（接地節点を選んでも
/// 「配置済み」とは表示しない。上部節点＝対象節点側を選んだ場合のみヒットする）。
pub fn find_support_isolator(
    model: &squid_n_core::model::Model,
    node_id: NodeId,
) -> Option<squid_n_core::ids::ElemId> {
    model
        .elements
        .iter()
        .find(|e| {
            model
                .support_isolator_ends(e.id)
                .is_some_and(|(upper, _ground)| upper == node_id)
        })
        .map(|e| e.id)
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::model::Model;
    use squid_n_edit::UndoStack;

    /// `find_support_isolator`: 未設置の節点では `None` を返す。
    #[test]
    fn test_find_support_isolator_none_when_not_placed() {
        let mut model = Model::default();
        model.nodes.push(squid_n_core::model::Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
            support_spring: None,
        });
        assert_eq!(find_support_isolator(&model, NodeId(0)), None);
    }

    /// `PlaceSupportIsolator` 実行後は当該節点に接続する零長 Isolator 要素が
    /// `find_support_isolator` で見つかり、対象節点の拘束は解放（FREE）される。
    #[test]
    fn test_place_support_isolator_then_find_support_isolator() {
        let mut model = Model::default();
        model.nodes.push(squid_n_core::model::Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
            support_spring: None,
        });
        let mut undo = UndoStack::new();
        let props = IsolatorProps {
            kind: IsolatorKind::LeadRubber,
            ..IsolatorProps::default()
        };
        undo.run(
            &mut model,
            Box::new(PlaceSupportIsolator {
                node: NodeId(0),
                props,
            }),
        );

        assert_eq!(model.nodes[0].restraint, Dof6Mask::FREE);
        let found = find_support_isolator(&model, NodeId(0));
        assert!(found.is_some());
        let elem_id = found.unwrap();
        let attr_props = model
            .isolator_attrs
            .iter()
            .find(|a| a.elem == elem_id)
            .map(|a| a.props);
        assert_eq!(attr_props, Some(props));

        // undo で接地節点・要素が消え、拘束も元（FIXED）に戻る。
        undo.undo(&mut model);
        assert_eq!(model.nodes.len(), 1);
        assert_eq!(model.elements.len(), 0);
        assert_eq!(model.nodes[0].restraint, Dof6Mask::FIXED);
    }

    /// `find_support_isolator`: 接地節点（FIXED側）を選んだ場合は「配置済み」と
    /// 誤表示しないよう `None` を返す（対象節点＝上部節点側を選んだ場合のみ
    /// `Some` を返す）。
    #[test]
    fn test_find_support_isolator_none_when_ground_node_selected() {
        let mut model = Model::default();
        model.nodes.push(squid_n_core::model::Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
            support_spring: None,
        });
        let mut undo = UndoStack::new();
        undo.run(
            &mut model,
            Box::new(PlaceSupportIsolator {
                node: NodeId(0),
                props: IsolatorProps::default(),
            }),
        );
        let ground_id = NodeId(1);
        assert_eq!(model.nodes[ground_id.index()].restraint, Dof6Mask::FIXED);
        // 上部節点（対象節点）側では見つかる。
        assert!(find_support_isolator(&model, NodeId(0)).is_some());
        // 接地節点側では見つからない。
        assert_eq!(find_support_isolator(&model, ground_id), None);
    }

    /// 境界条件パネルの「撤去」ボタン相当（`RemoveSupportIsolator`）: 配置→撤去で
    /// 接地節点・要素が消え、対象節点の拘束が FIXED へ戻ること。
    #[test]
    fn test_isolator_support_section_remove_button_command() {
        let mut model = Model::default();
        model.nodes.push(squid_n_core::model::Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
            support_spring: None,
        });
        let before = model.clone();
        let mut undo = UndoStack::new();
        undo.run(
            &mut model,
            Box::new(PlaceSupportIsolator {
                node: NodeId(0),
                props: IsolatorProps::default(),
            }),
        );
        assert!(find_support_isolator(&model, NodeId(0)).is_some());

        undo.run(
            &mut model,
            Box::new(RemoveSupportIsolator { node: NodeId(0) }),
        );
        assert!(find_support_isolator(&model, NodeId(0)).is_none());
        assert!(model.eq_ignoring_dofmap(&before));
        assert_eq!(model.nodes[0].restraint, Dof6Mask::FIXED);
    }

    /// `SetNodeSupportSpring`: 固定されていない自由度にばね値を設定し、undo で解除できる。
    #[test]
    fn test_set_node_support_spring_via_undo() {
        let mut model = Model::default();
        model.nodes.push(squid_n_core::model::Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
        let mut undo = UndoStack::new();
        let spring = [1.0e5, 1.0e5, 2.0e5, 1.0e9, 1.0e9, 1.0e9];
        undo.run(
            &mut model,
            Box::new(SetNodeSupportSpring {
                node: NodeId(0),
                spring: Some(spring),
            }),
        );
        assert_eq!(model.nodes[0].support_spring, Some(spring));

        undo.undo(&mut model);
        assert_eq!(model.nodes[0].support_spring, None);
    }
}
