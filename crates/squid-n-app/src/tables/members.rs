use crate::app::App;
use crate::tables::nodes::{isolator_kind_label, isolator_kind_selector, isolator_props_fields};
use squid_n_core::ids::{ElemId, NodeId, SectionId};
use squid_n_core::model::{
    DamperDef, DamperKind, DamperProps, ElementData, ElementKind, EndCondition, ForceRegime,
    HysteresisModel, IsolatorProps, LocalAxis, Model,
};
use squid_n_core::units::to_display::{force_kn, stiffness_kn_per_mm, viscous_c0_kn};
use squid_n_core::units::to_internal;
use squid_n_edit::{
    AddDamper, AddIsolator, AddMember, DeleteMember, DeleteWallPlate, EditCommand,
    RemoveSupportIsolator, SetDamperProps, SetElementSection, SetMemberHysteresis,
    SetMemberHysteresisTh, SetWallPlateSection,
};
use squid_n_load::wall_expand::{self, WallExpansionIndex};
use std::borrow::Cow;

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

/// 部材表の履歴則コンボを編集できるか。
///
/// 生成壁の `ElemId` は入力モデルの `elements` に存在しないため、
/// [`SetMemberHysteresis`] は存在検証で Noop になる。履歴則は壁版のフィールドでもない。
fn member_hysteresis_editable(is_generated_wall: bool, kind: ElementKind) -> bool {
    !is_generated_wall
        && matches!(
            kind,
            ElementKind::Beam | ElementKind::Fiber | ElementKind::MultiSpring | ElementKind::Wall
        )
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

/// 部材表の表示用モデル。壁版から生成された `ElementKind::Wall` を含めるため、
/// 壁版を持つモデルでは [`wall_expand::expand_wall_elements`] の結果を使う。
struct MembersTableView<'a> {
    model: Cow<'a, Model>,
    wall_index: Option<WallExpansionIndex>,
}

fn members_table_view<'a>(model: &'a Model) -> MembersTableView<'a> {
    if wall_expand::model_has_wall_plates_to_expand(model) {
        let (expanded, index, _) = wall_expand::expand_wall_elements(model);
        MembersTableView {
            model: Cow::Owned(expanded),
            wall_index: Some(index),
        }
    } else {
        MembersTableView {
            model: Cow::Borrowed(model),
            wall_index: None,
        }
    }
}

pub fn members_table(ui: &mut egui::Ui, app: &mut App) {
    use crate::table_util::{self, Col};

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

    let (wall_index, pending_section, pending_hysteresis, pending_hysteresis_th, pending_delete) = {
        let view = members_table_view(&app.model);
        if view
            .wall_index
            .as_ref()
            .is_some_and(|index| !index.is_empty())
        {
            ui.label(
                egui::RichText::new(
                    "耐震壁（Wall）は壁版から生成された解析要素です。断面の変更は「壁版」タブ、\
                     削除も壁版の削除で行います（履歴則はこの一覧から編集できます）。",
                )
                .color(crate::theme::GRAY_600)
                .small(),
            );
            ui.separator();
        }

        // 本表は解析要素の一覧なので、解析要素にならない壁版（腰壁・垂壁・間柱で
        // 分割された壁等）は行として現れない。壁版の一覧は「壁版」タブが持つため、
        // そこへ導線を出す（「入力したはずの壁が見当たらない」を防ぐ）。
        if app
            .model
            .wall_plates
            .iter()
            .any(|p| !app.model.wall_plate_becomes_element(p))
        {
            ui.label(
                egui::RichText::new(
                    "解析要素にならない壁版（腰壁・垂壁・パラペット・自立壁・間柱で分割\
                     された壁など）は、本表には現れません。「壁版」タブで一覧・編集できます。",
                )
                .color(crate::theme::GRAY_600)
                .small(),
            );
            ui.separator();
        }

        let n = view.model.elements.len();
        let mut pending_section: Vec<(ElemId, u32)> = Vec::new();
        let mut pending_hysteresis: Vec<(ElemId, HysteresisModel)> = Vec::new();
        let mut pending_hysteresis_th: Vec<(ElemId, Option<HysteresisModel>)> = Vec::new();
        let mut pending_delete: Option<ElemId> = None;

        table_util::standard_table(
            ui,
            "members_tbl",
            &[
                Col::id(),
                Col::label("種別"),
                Col::wide_num("節点"),
                Col::name("断面"),
                Col::name("材料"),
                Col::text("履歴則(増分)"),
                Col::text("履歴則(時刻歴)"),
                Col::actions(),
            ],
            n,
            |row| {
                let i = row.index();
                let elem = &view.model.elements[i];
                let generated_wall = view
                    .wall_index
                    .as_ref()
                    .and_then(|index| index.plate_of(elem.id));
                let is_focus = app.nav.focus_member == Some(elem.id);
                row.col(|ui| {
                    if table_util::id_cell(ui, is_focus, elem.id.0, "クリックで部材を選択")
                    {
                        app.nav.focus_member = Some(elem.id);
                    }
                });
                row.col(|ui| {
                    let label = if generated_wall.is_some() {
                        "Wall(壁版)"
                    } else {
                        match elem.kind {
                            ElementKind::Beam => "Beam",
                            ElementKind::Fiber => "Fiber",
                            ElementKind::MultiSpring => "MultiSpring",
                            ElementKind::Wall => "Wall",
                            ElementKind::Shell => "Shell",
                            ElementKind::Brace { .. } => "Brace",
                            ElementKind::Damper => "Damper",
                            ElementKind::Isolator => "Isolator",
                            ElementKind::NodalSpring => "NodalSpring",
                            ElementKind::PanelZone => "PanelZone",
                        }
                    };
                    ui.label(label).on_hover_text(if generated_wall.is_some() {
                        "壁版から生成された耐震壁要素（解析専用）"
                    } else {
                        ""
                    });
                });
                row.col(|ui| {
                    let ids: Vec<String> = elem.nodes.iter().map(|n| n.0.to_string()).collect();
                    table_util::text_cell(ui, &ids.join(","));
                });
                row.col(|ui| {
                    let selected = elem.section.map(|s| s.0).unwrap_or(u32::MAX);
                    let hover = if generated_wall.is_some() {
                        "壁版由来の耐震壁です。断面は壁版タブでも編集できます"
                    } else {
                        ""
                    };
                    table_util::cell_combo(
                        ui,
                        format!("elem_sec_{}", i),
                        elem.section
                            .map(|s| format!("S{}", s.0))
                            .unwrap_or_else(|| "―".to_string()),
                        |ui| {
                            if ui.selectable_label(selected == u32::MAX, "―").clicked() {
                                pending_section.push((elem.id, u32::MAX));
                            }
                            for sec in &app.model.sections {
                                if ui
                                    .selectable_label(
                                        selected == sec.id.0,
                                        format!("S{} {}", sec.id.0, sec.name),
                                    )
                                    .clicked()
                                {
                                    pending_section.push((elem.id, sec.id.0));
                                }
                            }
                        },
                    )
                    .response
                    .on_hover_text(hover);
                });
                row.col(|ui| {
                    // 材料は断面が持つため、この欄は断面から引いた表示のみとする
                    // （割り当ては断面テーブルで行う）。
                    match view.model.element_material(elem) {
                        Some(m) => {
                            let name = m.name.clone();
                            ui.label(&name).on_hover_text(format!(
                                "{name}（断面が持つ材料です。変更は断面テーブルで行ってください）"
                            ));
                        }
                        None => table_util::muted_cell(
                            ui,
                            "―",
                            "断面に材料が割り当てられていません（断面テーブルで割り当てます）",
                        ),
                    }
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
                                view.model.as_ref(),
                                squid_n_core::model::AnalysisKind::Incremental,
                            );
                            format!("自動（{}）", eff.label())
                        }
                    };
                    let enabled = member_hysteresis_editable(generated_wall.is_some(), elem.kind);
                    ui.add_enabled_ui(enabled, |ui| {
                        table_util::cell_combo(
                            ui,
                            format!("elem_hyst_{}", i),
                            selected_text,
                            |ui| {
                                for m in HysteresisModel::ALL {
                                    let is_sel = match current {
                                        Some(c) => m == c,
                                        None => m == HysteresisModel::Auto,
                                    };
                                    if ui.selectable_label(is_sel, m.label()).clicked() {
                                        pending_hysteresis.push((elem.id, m));
                                    }
                                }
                            },
                        )
                        .response
                        .on_hover_text(if generated_wall.is_some() {
                            "壁版由来の耐震壁は入力モデルに要素として存在しないため、\
                             履歴則の個別指定はできません（解析時は壁の既定＝原点指向型）"
                        } else {
                            "増分解析（保有水平耐力計算）の履歴則。\
                                 梁=材端曲げバネ（自動: RC/SRC/CFT=武田型、S=標準型）。\
                                 柱（ファイバー）・MS・壁=コンクリート除荷則\
                                 （逆行型／原点指向型／Karsan-Jirsa型のみ有効。\
                                 自動: 柱・MS=逆行型、壁=原点指向型）"
                        });
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
                                view.model.as_ref(),
                                squid_n_core::model::AnalysisKind::TimeHistory,
                            );
                            format!("自動（{}）", eff.label())
                        }
                        Some(r) => r.label().to_string(),
                    };
                    let enabled = member_hysteresis_editable(generated_wall.is_some(), elem.kind);
                    ui.add_enabled_ui(enabled, |ui| {
                        table_util::cell_combo(
                            ui,
                            format!("elem_hyst_th_{}", i),
                            selected_text,
                            |ui| {
                                if ui
                                    .selectable_label(current_th_raw.is_none(), "増分と同じ")
                                    .clicked()
                                {
                                    pending_hysteresis_th.push((elem.id, None));
                                }
                                for m in HysteresisModel::ALL {
                                    let is_sel = current_th_raw == Some(m);
                                    if ui.selectable_label(is_sel, m.label()).clicked() {
                                        pending_hysteresis_th.push((elem.id, Some(m)));
                                    }
                                }
                            },
                        )
                        .response
                        .on_hover_text(if generated_wall.is_some() {
                            "壁版由来の耐震壁は入力モデルに要素として存在しないため、\
                             履歴則の個別指定はできません（解析時は壁の既定）"
                        } else {
                            "時刻歴応答解析の履歴則。「増分と同じ」で増分用の指定に従う。\
                                 自動: 梁=増分と同じ既定、柱（ファイバー）・MS・壁の\
                                 コンクリートは Karsan-Jirsa型"
                        });
                    });
                });
                row.col(|ui| {
                    let delete_hover = if generated_wall.is_some() {
                        "壁版由来の耐震壁を削除します（対応する壁版も削除されます）"
                    } else {
                        "部材を削除（関連する部材荷重も削除されます）"
                    };
                    if table_util::delete_cell(ui, delete_hover, None) {
                        pending_delete = Some(elem.id);
                    }
                });
            },
        );

        (
            view.wall_index.clone(),
            pending_section,
            pending_hysteresis,
            pending_hysteresis_th,
            pending_delete,
        )
    };

    // 確定処理
    let had_pending = !pending_section.is_empty()
        || !pending_hysteresis.is_empty()
        || !pending_hysteresis_th.is_empty()
        || pending_delete.is_some();
    for (elem_id, sec_id) in pending_section {
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
        if let Some(plate_id) = wall_index
            .as_ref()
            .and_then(|index| index.plate_of(elem_id))
        {
            app.undo.run(
                &mut app.model,
                Box::new(SetWallPlateSection {
                    id: plate_id,
                    section,
                }),
            );
        } else {
            app.undo.run(
                &mut app.model,
                Box::new(SetElementSection {
                    elem: elem_id,
                    section,
                }),
            );
        }
    }
    for (elem_id, rule) in pending_hysteresis {
        app.undo.run(
            &mut app.model,
            Box::new(SetMemberHysteresis {
                elem: elem_id,
                rule,
            }),
        );
    }
    for (elem_id, rule_th) in pending_hysteresis_th {
        app.undo.run(
            &mut app.model,
            Box::new(SetMemberHysteresisTh {
                elem: elem_id,
                rule_th,
            }),
        );
    }
    if let Some(elem_id) = pending_delete {
        if let Some(plate_id) = wall_index
            .as_ref()
            .and_then(|index| index.plate_of(elem_id))
        {
            app.undo
                .run(&mut app.model, Box::new(DeleteWallPlate { id: plate_id }));
        } else {
            app.undo
                .run(&mut app.model, Box::new(DeleteMember { id: elem_id }));
        }
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
    use crate::table_util::{self, Col};

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

    table_util::standard_table(
        ui,
        "dampers_table",
        &[
            Col::id(),
            Col::wide_num("節点"),
            Col::name("種別"),
            Col::num("Kd"),
            Col::num("C0"),
            Col::num("α"),
            Col::num("Qy"),
            Col::num("k2/k1"),
            Col::actions(),
        ],
        dampers.len(),
        |row| {
            let (elem_id, props) = dampers[row.index()];
            let mut props = props;
            let is_maxwell = props.kind == DamperKind::Maxwell;
            row.col(|ui| {
                table_util::id_label(ui, elem_id.0);
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
                table_util::text_cell(ui, &nodes);
            });
            // 種別セレクタ。
            row.col(|ui| {
                let label = match props.kind {
                    DamperKind::Maxwell => "マクスウェル",
                    DamperKind::HystereticBilinear => "履歴型ﾊﾞｲﾘﾆｱ",
                };
                table_util::cell_combo(ui, format!("damper_kind_{}", elem_id.0), label, |ui| {
                    for k in [DamperKind::Maxwell, DamperKind::HystereticBilinear] {
                        let l = match k {
                            DamperKind::Maxwell => "マクスウェル",
                            DamperKind::HystereticBilinear => "履歴型ﾊﾞｲﾘﾆｱ",
                        };
                        if ui.selectable_label(props.kind == k, l).clicked() && props.kind != k {
                            props.kind = k;
                            pending_props.push((elem_id, props));
                        }
                    }
                });
            });
            // Kd（両種別で使用。kN/mm 単位で編集）。
            row.col(|ui| {
                let mut kd_kn = stiffness_kn_per_mm(props.kd);
                let resp = table_util::cell_drag_value(
                    ui,
                    true,
                    egui::DragValue::new(&mut kd_kn)
                        .speed(1.0)
                        .range(0.0..=1.0e9),
                );
                if resp.changed() {
                    props.kd = to_internal::stiffness_kn_per_mm(kd_kn);
                    pending_props.push((elem_id, props));
                }
            });
            // C0（マクスウェルのみ）。
            row.col(|ui| {
                let mut c0_kn = viscous_c0_kn(props.c0);
                let resp = table_util::cell_drag_value(
                    ui,
                    is_maxwell,
                    egui::DragValue::new(&mut c0_kn)
                        .speed(0.1)
                        .range(0.0..=1.0e9),
                );
                if resp.changed() {
                    props.c0 = to_internal::viscous_c0_kn(c0_kn);
                    pending_props.push((elem_id, props));
                }
            });
            // α（マクスウェルのみ）。
            row.col(|ui| {
                let resp = table_util::cell_drag_value(
                    ui,
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
                let mut qy_kn = force_kn(props.qy);
                let resp = table_util::cell_drag_value(
                    ui,
                    !is_maxwell,
                    egui::DragValue::new(&mut qy_kn)
                        .speed(1.0)
                        .range(0.0..=1.0e9),
                );
                if resp.changed() {
                    props.qy = to_internal::force_kn(qy_kn);
                    pending_props.push((elem_id, props));
                }
            });
            // k2/k1（履歴型のみ）。
            row.col(|ui| {
                let resp = table_util::cell_drag_value(
                    ui,
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
                if table_util::delete_cell(ui, "制振ダンパーを削除", None) {
                    pending_del = Some(elem_id);
                }
            });
        },
    );

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
    use crate::table_util::{self, Col};

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

    table_util::standard_table(
        ui,
        "isolators_table",
        &[
            Col::id(),
            Col::wide_num("節点"),
            Col::text("種別"),
            Col::num("K1"),
            Col::num("K2"),
            Col::num("Qd"),
            Col::num("Kv"),
            Col::num("μ"),
            Col::actions(),
        ],
        isolators.len(),
        |row| {
            let (elem_id, props) = isolators[row.index()];
            let mut props = props;
            let is_sliding = props.kind == squid_n_core::model::IsolatorKind::ElasticSliding;
            row.col(|ui| {
                table_util::id_label(ui, elem_id.0);
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
                table_util::text_cell(ui, &nodes);
            });
            // 種別セレクタ。
            row.col(|ui| {
                table_util::cell_combo(
                    ui,
                    format!("isolator_kind_{}", elem_id.0),
                    isolator_kind_label(props.kind),
                    |ui| {
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
                    },
                );
            });
            // K1（両種別で使用。kN/mm 単位で編集）。
            // ドラッグ中は毎フレーム changed() が真になるため、コマンド発行は
            // ドラッグ終了（またはフォーカス喪失）まで遅らせる（undo スタックの
            // 大量消費防止）。表示用の変換自体は毎フレーム行い、ドラッグ中の
            // ライブ表示は維持する。
            row.col(|ui| {
                let mut k1_kn = stiffness_kn_per_mm(props.k1);
                let resp = table_util::cell_drag_value(
                    ui,
                    true,
                    egui::DragValue::new(&mut k1_kn)
                        .speed(1.0)
                        .range(0.0..=1.0e6),
                );
                props.k1 = to_internal::stiffness_kn_per_mm(k1_kn);
                if resp.drag_stopped() || resp.lost_focus() {
                    pending_props.push((elem_id, props));
                }
            });
            // K2（積層ゴム系のみ）。
            row.col(|ui| {
                let mut k2_kn = stiffness_kn_per_mm(props.k2);
                let resp = table_util::cell_drag_value(
                    ui,
                    !is_sliding,
                    egui::DragValue::new(&mut k2_kn)
                        .speed(1.0)
                        .range(0.0..=1.0e6),
                );
                props.k2 = to_internal::stiffness_kn_per_mm(k2_kn);
                if resp.drag_stopped() || resp.lost_focus() {
                    pending_props.push((elem_id, props));
                }
            });
            // Qd（積層ゴム系のみ。kN 単位）。
            row.col(|ui| {
                let mut qd_kn = force_kn(props.qd);
                let resp = table_util::cell_drag_value(
                    ui,
                    !is_sliding,
                    egui::DragValue::new(&mut qd_kn)
                        .speed(1.0)
                        .range(0.0..=1.0e6),
                );
                props.qd = to_internal::force_kn(qd_kn);
                if resp.drag_stopped() || resp.lost_focus() {
                    pending_props.push((elem_id, props));
                }
            });
            // Kv（両種別で使用。kN/mm 単位）。
            row.col(|ui| {
                let mut kv_kn = stiffness_kn_per_mm(props.kv);
                let resp = table_util::cell_drag_value(
                    ui,
                    true,
                    egui::DragValue::new(&mut kv_kn)
                        .speed(10.0)
                        .range(0.0..=1.0e9),
                );
                props.kv = to_internal::stiffness_kn_per_mm(kv_kn);
                if resp.drag_stopped() || resp.lost_focus() {
                    pending_props.push((elem_id, props));
                }
            });
            // μ（すべり支承のみ）。
            row.col(|ui| {
                let resp = table_util::cell_drag_value(
                    ui,
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
                if table_util::delete_cell(ui, "免震支承材を削除", None) {
                    pending_del = Some(elem_id);
                }
            });
        },
    );

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
/// コマンドがないため（今回の作業範囲は `squid-n-edit` を読み取り専用とする方針）、
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

    /// 壁版を持つモデルでは部材表用ビューに生成壁が含まれる。
    #[test]
    fn members_table_view_includes_generated_walls() {
        use squid_n_core::dof::Dof6Mask;
        use squid_n_core::ids::{NodeId, SectionId, WallPlateId, WallRegionId};
        use squid_n_core::model::{
            ElementKind, Model, Node, Section, WallPlate, WallPlateShape, WallRegion,
        };

        fn node(id: u32, x: f64, y: f64, z: f64) -> Node {
            Node {
                id: NodeId(id),
                coord: [x, y, z],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            }
        }

        let mut model = Model::default();
        for (id, (x, y, z)) in [
            (0, (0.0, 0.0, 0.0)),
            (1, (4000.0, 0.0, 0.0)),
            (2, (4000.0, 0.0, 3000.0)),
            (3, (0.0, 0.0, 3000.0)),
        ] {
            model.nodes.push(node(id, x, y, z));
        }
        model.sections.push(Section {
            id: SectionId(0),
            name: "壁 t150".into(),
            area: 150.0 * 3000.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 3000.0,
            width: 150.0,
            as_y: 1.0,
            as_z: 1.0,
            floor: None,
            panel_thickness: None,
            thickness: Some(150.0),
            shape: None,
            material: None,
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        });
        model.wall_plates.push(WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            section: Some(SectionId(0)),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            column_face_slit: [false, false],
        });
        model.wall_regions.push(WallRegion {
            id: WallRegionId(0),
            name: String::new(),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            wall_plate_ids: vec![WallPlateId(0)],
            posts: Vec::new(),
        });

        let base = model.elements.len();
        let view = members_table_view(&model);
        assert_eq!(view.model.elements.len(), base + 1);
        assert!(
            view.model
                .elements
                .iter()
                .any(|e| e.kind == ElementKind::Wall),
            "生成壁が部材表ビューに含まれる"
        );
        let wall = view
            .model
            .elements
            .iter()
            .find(|e| e.kind == ElementKind::Wall)
            .expect("wall");
        assert_eq!(
            view.wall_index.as_ref().and_then(|i| i.plate_of(wall.id)),
            Some(WallPlateId(0))
        );
    }

    /// 壁版を持たないモデルでは入力モデルを借用する。
    #[test]
    fn members_table_view_borrows_without_wall_plates() {
        let model = Model::default();
        let view = members_table_view(&model);
        assert!(matches!(view.model, Cow::Borrowed(_)));
        assert!(view.wall_index.is_none());
    }

    /// 生成壁の履歴則コンボは編集不可（`SetMemberHysteresis` が入力モデル上で Noop）。
    #[test]
    fn generated_wall_hysteresis_is_not_editable() {
        assert!(!member_hysteresis_editable(true, ElementKind::Wall));
        assert!(member_hysteresis_editable(false, ElementKind::Beam));
        assert!(member_hysteresis_editable(false, ElementKind::Wall));
        assert!(!member_hysteresis_editable(false, ElementKind::Damper));
    }
}
