use crate::app::App;
use crate::tables::nodes::{isolator_kind_label, isolator_kind_selector, isolator_props_fields};
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    DamperDef, DamperKind, DamperProps, ElementData, ElementKind, EndCondition, ForceRegime,
    HysteresisModel, IsolatorProps, LocalAxis,
};
use squid_n_edit::{
    AddDamper, AddIsolator, AddMember, DeleteMember, EditCommand, RemoveSupportIsolator,
    SetDamperProps, SetElementMaterial, SetElementSection, SetMemberHysteresis,
    SetMemberHysteresisTh,
};

/// 「+ 免震支承材追加」フォームのドラフト状態（`AddIsolator` の諸元）。
/// 2節点間へ免震支承材要素を作成する独立フォーム用（境界条件タブの
/// 「支点への配置」〔`PlaceSupportIsolator`〕とは別導線）。
#[derive(Clone, Debug, Default)]
pub struct IsolatorMemberDraft {
    pub props: IsolatorProps,
}

/// 「制振ダンパーの定義から選択」の選択中インデックスを、定義削除後も選択中の
/// 定義名で追従させる（純関数。テスト容易）。`stored` は前回選択していた
/// `(インデックス, その時点の定義名)`。そのインデックスの位置にある定義の名前が
/// 変わっていなければそのまま、変わっていれば同名の定義を探して追従し、
/// 見つからなければ選択解除（`None`）する。
fn resolve_damper_def_selection(
    defs: &[DamperDef],
    stored: Option<(usize, String)>,
) -> Option<(usize, String)> {
    let (i, name) = stored?;
    if defs.get(i).map(|d| d.name.as_str()) == Some(name.as_str()) {
        return Some((i, name));
    }
    defs.iter().position(|d| d.name == name).map(|i| (i, name))
}

/// 「自動」解決表示用に、要素種別に応じた履歴則解決関数を使い分ける。
/// 梁=材端曲げバネの履歴則、ファイバー柱・MS・壁=コンクリート除荷則。
fn resolve_member_hysteresis_for_kind(
    elem: &ElementData,
    model: &squid_n_core::model::Model,
    kind: squid_n_core::model::AnalysisKind,
) -> HysteresisModel {
    match elem.kind {
        ElementKind::Fiber | ElementKind::MultiSpring => {
            squid_n_element::factory::resolve_fiber_concrete_hysteresis(elem, model, kind)
        }
        // 耐震壁は面内せん断ばね（支配的挙動）の解決結果を表示する。
        // 壁柱ファイバのコンクリート除荷則は resolve_wall_concrete_hysteresis で
        // 別途解決される（同じ指定をそれぞれ解釈可能な範囲で適用）。
        ElementKind::Wall => {
            squid_n_element::factory::resolve_wall_shear_hysteresis(elem, model, kind)
        }
        _ => squid_n_element::factory::resolve_member_hysteresis(elem, model, kind),
    }
}

pub fn members_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    // ── 梁追加フォーム ──────────────────────────────────────────
    if app.model.nodes.len() < 2 {
        ui.label("梁を追加するには節点が2つ以上必要です");
    } else {
        let id_i = egui::Id::new("add_member_sel_i");
        let id_j = egui::Id::new("add_member_sel_j");

        // egui 一時メモリから選択済み節点IDを取得。未設定なら先頭/2番目の節点で初期化。
        let mut sel_i: Option<NodeId> = ui
            .data(|d| d.get_temp::<Option<NodeId>>(id_i))
            .flatten()
            .or_else(|| app.model.nodes.first().map(|n| n.id));
        let mut sel_j: Option<NodeId> = ui
            .data(|d| d.get_temp::<Option<NodeId>>(id_j))
            .flatten()
            .or_else(|| app.model.nodes.get(1).map(|n| n.id));

        // 制振ダンパー追加時に使う「定義から選択」の選択中インデックス。
        // （インデックス, 選択時点の定義名）を egui 一時メモリに保持し（App
        // フィールドは増やさない）、定義削除で他の定義がずれても選択中の定義名で
        // 追従できるようにする（`resolve_damper_def_selection` 参照）。
        let id_damper_def_sel = egui::Id::new("add_member_damper_def_sel");
        let stored_damper_def_sel =
            ui.data(|d| d.get_temp::<Option<(usize, String)>>(id_damper_def_sel));
        let mut damper_def_sel: Option<usize> =
            resolve_damper_def_selection(&app.model.damper_defs, stored_damper_def_sel.flatten())
                .map(|(i, _)| i);

        let mut do_add = false;
        // 免震支承材の作成（2節点＋種別＋諸元。下の折りたたみフォームで諸元を編集）。
        let mut do_add_isolator = false;
        // 制振ダンパー（マクスウェル要素等）の追加（下部の一覧で編集する）。
        let mut do_add_damper = false;

        ui.horizontal(|ui| {
            ui.label("梁追加:");

            // i 節点 ComboBox
            let i_text = sel_i
                .map(|n| format!("N{}", n.0))
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt("add_member_i")
                .selected_text(i_text)
                .show_ui(ui, |ui| {
                    for node in &app.model.nodes {
                        let label = format!("N{}", node.id.0);
                        if ui
                            .selectable_label(sel_i == Some(node.id), &label)
                            .clicked()
                        {
                            sel_i = Some(node.id);
                        }
                    }
                });

            // j 節点 ComboBox
            let j_text = sel_j
                .map(|n| format!("N{}", n.0))
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt("add_member_j")
                .selected_text(j_text)
                .show_ui(ui, |ui| {
                    for node in &app.model.nodes {
                        let label = format!("N{}", node.id.0);
                        if ui
                            .selectable_label(sel_j == Some(node.id), &label)
                            .clicked()
                        {
                            sel_j = Some(node.id);
                        }
                    }
                });

            // i != j のときのみ追加ボタンを有効化
            let enabled = matches!((sel_i, sel_j), (Some(i), Some(j)) if i != j);
            if ui
                .add_enabled(enabled, egui::Button::new("+ 部材追加"))
                .clicked()
            {
                do_add = true;
            }

            ui.separator();
            // 免震支承材の作成は下の折りたたみフォーム（諸元入力）から実行する。
            if ui
                .add_enabled(enabled, egui::Button::new("+ 免震支承材追加"))
                .on_hover_text(
                    "下の「免震支承材を追加」フォームで種別・諸元を編集してから追加します",
                )
                .clicked()
            {
                do_add_isolator = true;
            }
            // 制振ダンパー（マクスウェル要素等）を選択2節点間に追加（下部一覧で編集）。
            // 「定義から選択」で選んだプリセットがあればその諸元を初期値にする。
            if ui
                .add_enabled(enabled, egui::Button::new("+ 制振ダンパー追加"))
                .on_hover_text(
                    "選択中の定義（未選択なら既定諸元）で制振ダンパーを追加（諸元は下部の一覧で編集）",
                )
                .clicked()
            {
                do_add_damper = true;
            }
        });

        // クロージャ終了後に一時メモリ更新（借用の競合を避ける）
        ui.data_mut(|d| d.insert_temp(id_i, sel_i));
        ui.data_mut(|d| d.insert_temp(id_j, sel_j));

        // ── 制振ダンパー「定義から選択」（damper_defs から選ぶと諸元に反映） ──
        ui.horizontal(|ui| {
            ui.label("制振ダンパーの定義:");
            let text = damper_def_sel
                .and_then(|i| app.model.damper_defs.get(i))
                .map(|d| d.name.clone())
                .unwrap_or_else(|| "（既定諸元）".to_string());
            egui::ComboBox::from_id_salt("add_member_damper_def")
                .selected_text(text)
                .show_ui(ui, |ui| {
                    if ui
                        .selectable_label(damper_def_sel.is_none(), "（既定諸元）")
                        .clicked()
                    {
                        damper_def_sel = None;
                    }
                    for (i, def) in app.model.damper_defs.iter().enumerate() {
                        if ui
                            .selectable_label(damper_def_sel == Some(i), &def.name)
                            .clicked()
                        {
                            damper_def_sel = Some(i);
                        }
                    }
                });
            if app.model.damper_defs.is_empty() {
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "（定義がありません。「断面」タブの「制振要素」パネルで作成できます）",
                );
            }
        });
        let to_store =
            damper_def_sel.and_then(|i| app.model.damper_defs.get(i).map(|d| (i, d.name.clone())));
        ui.data_mut(|d| d.insert_temp(id_damper_def_sel, to_store));

        // ── 免震支承材を追加（諸元フォーム） ─────────────────────
        ui.separator();
        egui::CollapsingHeader::new("免震支承材を追加")
            .default_open(false)
            .id_salt("add_isolator_section")
            .show(ui, |ui| {
                ui.label(format!(
                    "対象（上の梁追加と共通の2節点）: {} → {}",
                    sel_i
                        .map(|n| format!("N{}", n.0))
                        .unwrap_or_else(|| "―".to_string()),
                    sel_j
                        .map(|n| format!("N{}", n.0))
                        .unwrap_or_else(|| "―".to_string()),
                ));
                isolator_kind_selector(ui, &mut app.isolator_member_draft.props.kind);
                isolator_props_fields(
                    ui,
                    "members_add_isolator",
                    &mut app.isolator_member_draft.props,
                );
            });

        // 追加実行（クロージャ外で app の可変借用を使う）
        if do_add {
            if let (Some(i_node), Some(j_node)) = (sel_i, sel_j) {
                let new_id = ElemId(app.model.elements.len() as u32);
                let elem = ElementData {
                    id: new_id,
                    kind: ElementKind::Beam,
                    nodes: [i_node, j_node].into_iter().collect(),
                    section: None,
                    material: None,
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                };
                app.undo.run(&mut app.model, Box::new(AddMember { elem }));
                app.staleness.mark_edited();
            }
        }

        // 免震支承材追加（要素＋既定諸元を原子的に作成）。
        if do_add_isolator {
            if let (Some(i_node), Some(j_node)) = (sel_i, sel_j) {
                let new_id = ElemId(app.model.elements.len() as u32);
                let elem = ElementData {
                    id: new_id,
                    kind: ElementKind::Isolator,
                    nodes: [i_node, j_node].into_iter().collect(),
                    section: None,
                    material: None,
                    local_axis: LocalAxis {
                        ref_vector: [1.0, 0.0, 0.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                };
                app.undo.run(
                    &mut app.model,
                    Box::new(AddIsolator {
                        elem,
                        props: app.isolator_member_draft.props,
                    }),
                );
                app.nav.focus_member = Some(new_id);
                app.staleness.mark_edited();
            }
        }

        // 制振ダンパー追加（要素＋諸元を原子的に作成。「定義から選択」があればその諸元を使う）。
        if do_add_damper {
            if let (Some(i_node), Some(j_node)) = (sel_i, sel_j) {
                let new_id = ElemId(app.model.elements.len() as u32);
                let elem = ElementData {
                    id: new_id,
                    kind: ElementKind::Damper,
                    nodes: [i_node, j_node].into_iter().collect(),
                    section: None,
                    material: None,
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                };
                let props = damper_def_sel
                    .and_then(|i| app.model.damper_defs.get(i))
                    .map(|d| d.props)
                    .unwrap_or_default();
                app.undo
                    .run(&mut app.model, Box::new(AddDamper { elem, props }));
                app.nav.focus_member = Some(new_id);
                app.staleness.mark_edited();
            }
        }
    }
    ui.separator();
    // ── ここまで梁追加フォーム ────────────────────────────────────

    let n = app.model.elements.len();
    let mut pending_section: Vec<(usize, u32)> = Vec::new();
    let mut pending_material: Vec<(usize, u32)> = Vec::new();
    let mut pending_hysteresis: Vec<(usize, HysteresisModel)> = Vec::new();
    let mut pending_hysteresis_th: Vec<(usize, Option<HysteresisModel>)> = Vec::new();
    let mut pending_delete: Option<ElemId> = None;

    let row_h = crate::theme::table_row_height(ui);
    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::auto())
        .column(Column::initial(70.0))
        .column(Column::initial(80.0))
        .column(Column::initial(90.0))
        .column(Column::initial(120.0))
        .column(Column::initial(120.0))
        .column(Column::auto())
        .header(row_h, |mut h| {
            for t in &[
                "ID",
                "種別",
                "節点",
                "断面",
                "材料",
                "履歴則(増分)",
                "履歴則(時刻歴)",
                "",
            ] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(row_h, n, |mut row| {
                let i = row.index();
                let elem = &app.model.elements[i];
                let is_focus = app.nav.focus_member == Some(elem.id);
                row.col(|ui| {
                    let text = elem.id.0.to_string();
                    // 選択行は blue-500 背景になるため文字は白、非選択は既定色
                    let rich = egui::RichText::new(text).color(if is_focus {
                        crate::theme::WHITE
                    } else {
                        egui::Color32::PLACEHOLDER
                    });
                    if ui.selectable_label(is_focus, rich).clicked() {
                        app.nav.focus_member = Some(elem.id);
                    }
                });
                row.col(|ui| {
                    ui.label(format!("{:?}", elem.kind));
                });
                row.col(|ui| {
                    let ids: Vec<String> = elem.nodes.iter().map(|n| n.0.to_string()).collect();
                    ui.label(ids.join(","));
                });
                row.col(|ui| {
                    let selected = elem.section.map(|s| s.0).unwrap_or(u32::MAX);
                    let combo = egui::ComboBox::from_id_salt(format!("elem_sec_{}", i))
                        .selected_text(
                            elem.section
                                .map(|s| format!("S{}", s.0))
                                .unwrap_or_else(|| "―".to_string()),
                        );
                    combo.show_ui(ui, |ui| {
                        if ui.selectable_label(selected == u32::MAX, "―").clicked() {
                            pending_section.push((i, u32::MAX));
                        }
                        for sec in &app.model.sections {
                            if ui
                                .selectable_label(
                                    selected == sec.id.0,
                                    format!("S{} {}", sec.id.0, sec.name),
                                )
                                .clicked()
                            {
                                pending_section.push((i, sec.id.0));
                            }
                        }
                    });
                });
                row.col(|ui| {
                    let selected = elem.material.map(|m| m.0).unwrap_or(u32::MAX);
                    let combo = egui::ComboBox::from_id_salt(format!("elem_mat_{}", i))
                        .selected_text(
                            elem.material
                                .and_then(|m| app.model.materials.get(m.index()))
                                .map(|m| m.name.clone())
                                .unwrap_or_else(|| "―".to_string()),
                        );
                    combo.show_ui(ui, |ui| {
                        if ui.selectable_label(selected == u32::MAX, "―").clicked() {
                            pending_material.push((i, u32::MAX));
                        }
                        for mat in &app.model.materials {
                            if ui
                                .selectable_label(selected == mat.id.0, &mat.name)
                                .clicked()
                            {
                                pending_material.push((i, mat.id.0));
                            }
                        }
                    });
                });
                row.col(|ui| {
                    // 履歴則（復元力特性、増分解析用）。非線形解析の材端履歴則。
                    // 梁=材端曲げバネ、柱（ファイバー）・MS・壁=コンクリート除荷則へ反映。
                    let current = app.model.member_hysteresis(elem.id);
                    let selected_text = match current {
                        Some(r) => r.label().to_string(),
                        None => {
                            let eff = resolve_member_hysteresis_for_kind(
                                elem,
                                &app.model,
                                squid_n_core::model::AnalysisKind::Incremental,
                            );
                            format!("自動（{}）", eff.label())
                        }
                    };
                    let enabled = matches!(
                        elem.kind,
                        ElementKind::Beam
                            | ElementKind::Fiber
                            | ElementKind::MultiSpring
                            | ElementKind::Wall
                    );
                    ui.add_enabled_ui(enabled, |ui| {
                        egui::ComboBox::from_id_salt(format!("elem_hyst_{}", i))
                            .selected_text(selected_text)
                            .show_ui(ui, |ui| {
                                for m in HysteresisModel::ALL {
                                    let is_sel = match current {
                                        Some(c) => m == c,
                                        None => m == HysteresisModel::Auto,
                                    };
                                    if ui.selectable_label(is_sel, m.label()).clicked() {
                                        pending_hysteresis.push((i, m));
                                    }
                                }
                            })
                            .response
                            .on_hover_text(
                                "増分解析（保有水平耐力計算）の履歴則。\
                                 梁=材端曲げバネ（自動: RC/SRC/CFT=武田型、S=標準型）。\
                                 柱（ファイバー）・MS・壁=コンクリート除荷則\
                                 （逆行型／原点指向型／Karsan-Jirsa型のみ有効。\
                                 自動: 柱・MS=逆行型、壁=原点指向型）",
                            );
                    });
                });
                row.col(|ui| {
                    // 履歴則（時刻歴応答解析用スロット）。`None`＝増分用の指定に従う。
                    let current_th_raw = app.model.member_hysteresis_th_raw(elem.id);
                    let selected_text = match current_th_raw {
                        None => "増分と同じ".to_string(),
                        Some(HysteresisModel::Auto) => {
                            let eff = resolve_member_hysteresis_for_kind(
                                elem,
                                &app.model,
                                squid_n_core::model::AnalysisKind::TimeHistory,
                            );
                            format!("自動（{}）", eff.label())
                        }
                        Some(r) => r.label().to_string(),
                    };
                    let enabled = matches!(
                        elem.kind,
                        ElementKind::Beam
                            | ElementKind::Fiber
                            | ElementKind::MultiSpring
                            | ElementKind::Wall
                    );
                    ui.add_enabled_ui(enabled, |ui| {
                        egui::ComboBox::from_id_salt(format!("elem_hyst_th_{}", i))
                            .selected_text(selected_text)
                            .show_ui(ui, |ui| {
                                if ui
                                    .selectable_label(current_th_raw.is_none(), "増分と同じ")
                                    .clicked()
                                {
                                    pending_hysteresis_th.push((i, None));
                                }
                                for m in HysteresisModel::ALL {
                                    let is_sel = current_th_raw == Some(m);
                                    if ui.selectable_label(is_sel, m.label()).clicked() {
                                        pending_hysteresis_th.push((i, Some(m)));
                                    }
                                }
                            })
                            .response
                            .on_hover_text(
                                "時刻歴応答解析の履歴則。「増分と同じ」で増分用の指定に従う。\
                                 自動: 梁=増分と同じ既定、柱（ファイバー）・MS・壁の\
                                 コンクリートは Karsan-Jirsa型",
                            );
                    });
                });
                row.col(|ui| {
                    if ui
                        .button("🗑")
                        .on_hover_text("部材を削除（関連する部材荷重も削除されます）")
                        .clicked()
                    {
                        pending_delete = Some(elem.id);
                    }
                });
            });
        });

    // 確定処理
    let had_pending = !pending_section.is_empty()
        || !pending_material.is_empty()
        || !pending_hysteresis.is_empty()
        || !pending_hysteresis_th.is_empty()
        || pending_delete.is_some();
    for (i, sec_id) in pending_section {
        let elem_id = app.model.elements[i].id;
        let section = if sec_id == u32::MAX {
            None
        } else {
            // 参照先が存在するか確認
            let sid = SectionId(sec_id);
            if app.model.sections.iter().any(|s| s.id == sid) {
                Some(sid)
            } else {
                None
            }
        };
        app.undo.run(
            &mut app.model,
            Box::new(SetElementSection {
                elem: elem_id,
                section,
            }),
        );
    }
    for (i, mat_id) in pending_material {
        let elem_id = app.model.elements[i].id;
        let material = if mat_id == u32::MAX {
            None
        } else {
            let mid = MaterialId(mat_id);
            if app.model.materials.iter().any(|m| m.id == mid) {
                Some(mid)
            } else {
                None
            }
        };
        app.undo.run(
            &mut app.model,
            Box::new(SetElementMaterial {
                elem: elem_id,
                material,
            }),
        );
    }
    for (i, rule) in pending_hysteresis {
        let elem_id = app.model.elements[i].id;
        app.undo.run(
            &mut app.model,
            Box::new(SetMemberHysteresis {
                elem: elem_id,
                rule,
            }),
        );
    }
    for (i, rule_th) in pending_hysteresis_th {
        let elem_id = app.model.elements[i].id;
        app.undo.run(
            &mut app.model,
            Box::new(SetMemberHysteresisTh {
                elem: elem_id,
                rule_th,
            }),
        );
    }
    if let Some(elem_id) = pending_delete {
        app.undo
            .run(&mut app.model, Box::new(DeleteMember { id: elem_id }));
        if app.nav.focus_member == Some(elem_id) {
            app.nav.focus_member = None;
        }
    }

    // 編集があった場合は下流（結果・設計）を stale にする（UI設計 §5）
    if had_pending {
        app.staleness.mark_edited();
    }

    // ── 制振ダンパー一覧（Kd/C0/α の編集・削除）─────────────────────
    dampers_table(ui, app);
    // ── 免震支承材一覧（諸元編集・削除）───────────────────────────
    isolators_table(ui, app);
}

/// 制振ダンパー要素（`ElementKind::Damper`）の諸元編集・削除の一覧
/// （非線形動的解析の制振要素）。種別（マクスウェル＝速度依存型／
/// 履歴型バイリニア＝鋼材系）を選択し、種別に応じた諸元を編集する。
fn dampers_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    let dampers: Vec<(ElemId, DamperProps)> = app
        .model
        .elements
        .iter()
        .filter(|e| e.kind == ElementKind::Damper)
        .map(|e| (e.id, app.model.damper_props(e.id).unwrap_or_default()))
        .collect();
    if dampers.is_empty() {
        return;
    }

    ui.separator();
    ui.strong("制振ダンパー");
    ui.label(
        egui::RichText::new(
            "マクスウェル（速度依存）: Kd[kN/mm]・C0[kN·(s/mm)^α]・α。\
             履歴型バイリニア（鋼材系）: Kd=k1[kN/mm]・Qy[kN]・k2/k1。",
        )
        .color(crate::theme::GRAY_600)
        .small(),
    );

    // 変更・削除は借用衝突を避けて確定処理へ回す。
    let mut pending_props: Vec<(ElemId, DamperProps)> = Vec::new();
    let mut pending_del: Option<ElemId> = None;

    // ── 定義から一覧の全ダンパーへ一括適用 ─────────────────────
    if !app.model.damper_defs.is_empty() {
        let id_bulk_sel = egui::Id::new("dampers_table_bulk_def_sel");
        let mut bulk_sel: Option<usize> = ui
            .data(|d| d.get_temp::<Option<usize>>(id_bulk_sel))
            .flatten()
            .filter(|i| *i < app.model.damper_defs.len());
        ui.horizontal(|ui| {
            ui.label("一覧の全ダンパーへ一括適用:");
            let text = bulk_sel
                .and_then(|i| app.model.damper_defs.get(i))
                .map(|d| d.name.clone())
                .unwrap_or_else(|| "（定義を選択）".to_string());
            egui::ComboBox::from_id_salt("dampers_table_bulk_def")
                .selected_text(text)
                .show_ui(ui, |ui| {
                    for (i, def) in app.model.damper_defs.iter().enumerate() {
                        if ui
                            .selectable_label(bulk_sel == Some(i), &def.name)
                            .clicked()
                        {
                            bulk_sel = Some(i);
                        }
                    }
                });
            if ui
                .add_enabled(
                    bulk_sel.is_some(),
                    egui::Button::new("この一覧の全ダンパーへ適用"),
                )
                .on_hover_text(
                    "選択中の定義の諸元を、上の一覧に表示されている全ダンパーへコピーします",
                )
                .clicked()
            {
                if let Some(props) = bulk_sel
                    .and_then(|i| app.model.damper_defs.get(i))
                    .map(|d| d.props)
                {
                    for (elem_id, _) in &dampers {
                        pending_props.push((*elem_id, props));
                    }
                }
            }
        });
        ui.data_mut(|d| d.insert_temp(id_bulk_sel, bulk_sel));
    }

    let row_h = crate::theme::table_row_height(ui);
    TableBuilder::new(ui)
        .id_salt("dampers_table")
        .striped(true)
        .column(Column::auto())
        .column(Column::auto())
        .column(Column::initial(120.0))
        .column(Column::initial(80.0))
        .column(Column::initial(80.0))
        .column(Column::initial(64.0))
        .column(Column::initial(80.0))
        .column(Column::initial(64.0))
        .column(Column::auto())
        .header(row_h, |mut h| {
            for t in &["ID", "節点", "種別", "Kd", "C0", "α", "Qy", "k2/k1", ""] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|mut body| {
            for (elem_id, props) in &dampers {
                let elem_id = *elem_id;
                let mut props = *props;
                let is_maxwell = props.kind == DamperKind::Maxwell;
                body.row(row_h, |mut row| {
                    row.col(|ui| {
                        ui.label(elem_id.0.to_string());
                    });
                    row.col(|ui| {
                        let nodes = app
                            .model
                            .elements
                            .iter()
                            .find(|e| e.id == elem_id)
                            .map(|e| {
                                e.nodes
                                    .iter()
                                    .map(|n| n.0.to_string())
                                    .collect::<Vec<_>>()
                                    .join(",")
                            })
                            .unwrap_or_default();
                        ui.label(nodes);
                    });
                    // 種別セレクタ。
                    row.col(|ui| {
                        let label = match props.kind {
                            DamperKind::Maxwell => "マクスウェル",
                            DamperKind::HystereticBilinear => "履歴型ﾊﾞｲﾘﾆｱ",
                        };
                        egui::ComboBox::from_id_salt(format!("damper_kind_{}", elem_id.0))
                            .selected_text(label)
                            .show_ui(ui, |ui| {
                                for k in [DamperKind::Maxwell, DamperKind::HystereticBilinear] {
                                    let l = match k {
                                        DamperKind::Maxwell => "マクスウェル",
                                        DamperKind::HystereticBilinear => "履歴型ﾊﾞｲﾘﾆｱ",
                                    };
                                    if ui.selectable_label(props.kind == k, l).clicked()
                                        && props.kind != k
                                    {
                                        props.kind = k;
                                        pending_props.push((elem_id, props));
                                    }
                                }
                            });
                    });
                    // Kd（両種別で使用。kN/mm 単位で編集）。
                    row.col(|ui| {
                        let mut kd_kn = props.kd / 1000.0;
                        if ui
                            .add(
                                egui::DragValue::new(&mut kd_kn)
                                    .speed(1.0)
                                    .range(0.0..=1.0e9),
                            )
                            .changed()
                        {
                            props.kd = kd_kn * 1000.0;
                            pending_props.push((elem_id, props));
                        }
                    });
                    // C0（マクスウェルのみ）。
                    row.col(|ui| {
                        let mut c0_kn = props.c0 / 1000.0;
                        let resp = ui.add_enabled(
                            is_maxwell,
                            egui::DragValue::new(&mut c0_kn)
                                .speed(0.1)
                                .range(0.0..=1.0e9),
                        );
                        if resp.changed() {
                            props.c0 = c0_kn * 1000.0;
                            pending_props.push((elem_id, props));
                        }
                    });
                    // α（マクスウェルのみ）。
                    row.col(|ui| {
                        let resp = ui.add_enabled(
                            is_maxwell,
                            egui::DragValue::new(&mut props.alpha)
                                .speed(0.01)
                                .range(0.05..=2.0),
                        );
                        if resp.changed() {
                            pending_props.push((elem_id, props));
                        }
                    });
                    // Qy（履歴型のみ。kN 単位）。
                    row.col(|ui| {
                        let mut qy_kn = props.qy / 1000.0;
                        let resp = ui.add_enabled(
                            !is_maxwell,
                            egui::DragValue::new(&mut qy_kn)
                                .speed(1.0)
                                .range(0.0..=1.0e9),
                        );
                        if resp.changed() {
                            props.qy = qy_kn * 1000.0;
                            pending_props.push((elem_id, props));
                        }
                    });
                    // k2/k1（履歴型のみ）。
                    row.col(|ui| {
                        let resp = ui.add_enabled(
                            !is_maxwell,
                            egui::DragValue::new(&mut props.k2_ratio)
                                .speed(0.005)
                                .range(0.0..=0.99),
                        );
                        if resp.changed() {
                            pending_props.push((elem_id, props));
                        }
                    });
                    row.col(|ui| {
                        if ui.button("🗑").on_hover_text("制振ダンパーを削除").clicked()
                        {
                            pending_del = Some(elem_id);
                        }
                    });
                });
            }
        });

    let mut changed = false;
    for (elem_id, props) in pending_props {
        app.undo.run(
            &mut app.model,
            Box::new(SetDamperProps {
                elem: elem_id,
                props: Some(props),
            }),
        );
        changed = true;
    }
    if let Some(elem_id) = pending_del {
        app.undo
            .run(&mut app.model, Box::new(DeleteMember { id: elem_id }));
        if app.nav.focus_member == Some(elem_id) {
            app.nav.focus_member = None;
        }
        changed = true;
    }
    if changed {
        app.staleness.mark_edited();
    }
}

/// 免震支承材要素（`ElementKind::Isolator`）の諸元編集・削除の一覧。
/// 支点への設置（`PlaceSupportIsolator`、境界条件タブ）で生成された零長要素も
/// ここに一覧される（削除もここから行える。境界条件タブ側は要約表示のみ）。
fn isolators_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    let isolators: Vec<(ElemId, IsolatorProps)> = app
        .model
        .elements
        .iter()
        .filter(|e| e.kind == ElementKind::Isolator)
        .filter_map(|e| {
            app.model
                .isolator_attrs
                .iter()
                .find(|a| a.elem == e.id)
                .map(|a| (e.id, a.props))
        })
        .collect();
    if isolators.is_empty() {
        return;
    }

    ui.separator();
    ui.strong("免震支承材");
    ui.label(
        egui::RichText::new(
            "K1[kN/mm]・K2[kN/mm]・Qd[kN]（積層ゴム系のバイリニア。すべり支承は K2=Qd=0）・\
             Kv[kN/mm]（鉛直剛性）・μ（すべり支承の摩擦係数）。",
        )
        .color(crate::theme::GRAY_600)
        .small(),
    );

    let mut pending_props: Vec<(ElemId, IsolatorProps)> = Vec::new();
    let mut pending_del: Option<ElemId> = None;

    let row_h = crate::theme::table_row_height(ui);
    TableBuilder::new(ui)
        .id_salt("isolators_table")
        .striped(true)
        .column(Column::auto())
        .column(Column::auto())
        .column(Column::initial(150.0))
        .column(Column::initial(80.0))
        .column(Column::initial(80.0))
        .column(Column::initial(80.0))
        .column(Column::initial(80.0))
        .column(Column::initial(64.0))
        .column(Column::auto())
        .header(row_h, |mut h| {
            for t in &["ID", "節点", "種別", "K1", "K2", "Qd", "Kv", "μ", ""] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|mut body| {
            for (elem_id, props) in &isolators {
                let elem_id = *elem_id;
                let mut props = *props;
                let is_sliding = props.kind == squid_n_core::model::IsolatorKind::ElasticSliding;
                body.row(row_h, |mut row| {
                    row.col(|ui| {
                        ui.label(elem_id.0.to_string());
                    });
                    row.col(|ui| {
                        let nodes = app
                            .model
                            .elements
                            .iter()
                            .find(|e| e.id == elem_id)
                            .map(|e| {
                                e.nodes
                                    .iter()
                                    .map(|n| n.0.to_string())
                                    .collect::<Vec<_>>()
                                    .join(",")
                            })
                            .unwrap_or_default();
                        ui.label(nodes);
                    });
                    // 種別セレクタ。
                    row.col(|ui| {
                        egui::ComboBox::from_id_salt(format!("isolator_kind_{}", elem_id.0))
                            .selected_text(isolator_kind_label(props.kind))
                            .show_ui(ui, |ui| {
                                for k in [
                                    squid_n_core::model::IsolatorKind::LaminatedRubber,
                                    squid_n_core::model::IsolatorKind::LeadRubber,
                                    squid_n_core::model::IsolatorKind::HighDampingRubber,
                                    squid_n_core::model::IsolatorKind::ElasticSliding,
                                ] {
                                    if ui
                                        .selectable_label(props.kind == k, isolator_kind_label(k))
                                        .clicked()
                                        && props.kind != k
                                    {
                                        props.kind = k;
                                        pending_props.push((elem_id, props));
                                    }
                                }
                            });
                    });
                    // K1（両種別で使用。kN/mm 単位で編集）。
                    // ドラッグ中は毎フレーム changed() が真になるため、コマンド発行は
                    // ドラッグ終了（またはフォーカス喪失）まで遅らせる（undo スタックの
                    // 大量消費防止）。表示用の変換自体は毎フレーム行い、ドラッグ中の
                    // ライブ表示は維持する。
                    row.col(|ui| {
                        let mut k1_kn = props.k1 / 1000.0;
                        let resp = ui.add(
                            egui::DragValue::new(&mut k1_kn)
                                .speed(1.0)
                                .range(0.0..=1.0e6),
                        );
                        props.k1 = k1_kn * 1000.0;
                        if resp.drag_stopped() || resp.lost_focus() {
                            pending_props.push((elem_id, props));
                        }
                    });
                    // K2（積層ゴム系のみ）。
                    row.col(|ui| {
                        let mut k2_kn = props.k2 / 1000.0;
                        let resp = ui.add_enabled(
                            !is_sliding,
                            egui::DragValue::new(&mut k2_kn)
                                .speed(1.0)
                                .range(0.0..=1.0e6),
                        );
                        props.k2 = k2_kn * 1000.0;
                        if resp.drag_stopped() || resp.lost_focus() {
                            pending_props.push((elem_id, props));
                        }
                    });
                    // Qd（積層ゴム系のみ。kN 単位）。
                    row.col(|ui| {
                        let mut qd_kn = props.qd / 1000.0;
                        let resp = ui.add_enabled(
                            !is_sliding,
                            egui::DragValue::new(&mut qd_kn)
                                .speed(1.0)
                                .range(0.0..=1.0e6),
                        );
                        props.qd = qd_kn * 1000.0;
                        if resp.drag_stopped() || resp.lost_focus() {
                            pending_props.push((elem_id, props));
                        }
                    });
                    // Kv（両種別で使用。kN/mm 単位）。
                    row.col(|ui| {
                        let mut kv_kn = props.kv / 1000.0;
                        let resp = ui.add(
                            egui::DragValue::new(&mut kv_kn)
                                .speed(10.0)
                                .range(0.0..=1.0e9),
                        );
                        props.kv = kv_kn * 1000.0;
                        if resp.drag_stopped() || resp.lost_focus() {
                            pending_props.push((elem_id, props));
                        }
                    });
                    // μ（すべり支承のみ）。
                    row.col(|ui| {
                        let resp = ui.add_enabled(
                            is_sliding,
                            egui::DragValue::new(&mut props.mu)
                                .speed(0.005)
                                .range(0.0..=2.0),
                        );
                        if resp.drag_stopped() || resp.lost_focus() {
                            pending_props.push((elem_id, props));
                        }
                    });
                    row.col(|ui| {
                        if ui.button("🗑").on_hover_text("免震支承材を削除").clicked() {
                            pending_del = Some(elem_id);
                        }
                    });
                });
            }
        });

    let mut changed = false;
    for (elem_id, props) in pending_props {
        app.undo.run(
            &mut app.model,
            Box::new(SetIsolatorPropsLocal {
                elem: elem_id,
                props: Some(props),
            }),
        );
        changed = true;
    }
    if let Some(elem_id) = pending_del {
        // 対象が支点免震（零長＋接地節点）であれば RemoveSupportIsolator で
        // 接地節点まで含めて撤去する（通常の DeleteMember だと接地節点だけが
        // ゴミとして残ってしまうため）。通常の免震要素は従来どおり DeleteMember。
        match app.model.support_isolator_ends(elem_id) {
            Some((upper, _ground)) => {
                app.undo.run(
                    &mut app.model,
                    Box::new(RemoveSupportIsolator { node: upper }),
                );
            }
            None => {
                app.undo
                    .run(&mut app.model, Box::new(DeleteMember { id: elem_id }));
            }
        }
        if app.nav.focus_member == Some(elem_id) {
            app.nav.focus_member = None;
        }
        changed = true;
    }
    if changed {
        app.staleness.mark_edited();
    }
}

/// 免震支承材の特性（`IsolatorProps`）変更。
///
/// `squid-n-edit` には制振ダンパーの `SetDamperProps` に相当する免震支承材用の
/// コマンドが無いため（今回の作業範囲は `squid-n-edit` を読み取り専用とする方針）、
/// `Model::isolator_attrs` が公開フィールドであることを利用してこの UI 層で
/// `EditCommand` を実装する。`SetDamperProps` と同様、`props=None` で指定解除。
///
/// `props=Some` の場合は既存エントリを `iter_mut().find()` でインプレース書換え
/// する（retain+push だと undo 後に再適用した際、当該エントリが `isolator_attrs`
/// の末尾へ移動してしまい並び順が変わる）。`props=None`（解除）の場合のみ
/// `retain` で取り除く。
struct SetIsolatorPropsLocal {
    elem: ElemId,
    props: Option<IsolatorProps>,
}

impl EditCommand for SetIsolatorPropsLocal {
    fn apply(&self, model: &mut squid_n_core::model::Model) -> Box<dyn EditCommand> {
        let old = model
            .isolator_attrs
            .iter()
            .find(|a| a.elem == self.elem)
            .map(|a| a.props);
        match self.props {
            Some(p) => {
                if let Some(a) = model
                    .isolator_attrs
                    .iter_mut()
                    .find(|a| a.elem == self.elem)
                {
                    a.props = p;
                } else {
                    model
                        .isolator_attrs
                        .push(squid_n_core::model::IsolatorAttr {
                            elem: self.elem,
                            props: p,
                        });
                }
            }
            None => {
                model.isolator_attrs.retain(|a| a.elem != self.elem);
            }
        }
        Box::new(SetIsolatorPropsLocal {
            elem: self.elem,
            props: old,
        })
    }

    fn label(&self) -> &str {
        "免震支承材特性変更"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn def(name: &str) -> DamperDef {
        DamperDef {
            name: name.to_string(),
            props: DamperProps::default(),
        }
    }

    /// 選択中の定義がまだ同じ位置にあれば、そのまま維持する。
    #[test]
    fn test_resolve_damper_def_selection_unchanged_when_still_valid() {
        let defs = vec![def("A"), def("B")];
        let got = resolve_damper_def_selection(&defs, Some((1, "B".to_string())));
        assert_eq!(got, Some((1, "B".to_string())));
    }

    /// 選択中の定義より前の定義が削除され、位置がずれた場合は名前で追従する。
    #[test]
    fn test_resolve_damper_def_selection_follows_by_name_after_earlier_delete() {
        // 元は [A, B]（選択中は index=1 の B）。A が削除されて [B] になった。
        let defs = vec![def("B")];
        let got = resolve_damper_def_selection(&defs, Some((1, "B".to_string())));
        assert_eq!(got, Some((0, "B".to_string())));
    }

    /// 選択中だった定義自体が削除された場合は選択解除する。
    #[test]
    fn test_resolve_damper_def_selection_clears_when_deleted() {
        let defs = vec![def("A")];
        let got = resolve_damper_def_selection(&defs, Some((1, "B".to_string())));
        assert_eq!(got, None);
    }

    /// 未選択（既定諸元）はそのまま未選択を維持する。
    #[test]
    fn test_resolve_damper_def_selection_none_stays_none() {
        let defs = vec![def("A")];
        assert_eq!(resolve_damper_def_selection(&defs, None), None);
    }
}
