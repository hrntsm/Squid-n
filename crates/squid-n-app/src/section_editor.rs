//! UI-3: 断面作成UI（パラメトリック SectionShape）。
//!
//! モデルタブの ModelsTab::Sections で表示される下部パネル。
//! 鋼 H / 箱 / L / C / T / 丸、RC 矩形 / 丸 の寸法を入力すると
//! `SectionShape::to_section` を呼んで `Section` を `model.sections`
//! に新規追加する。インスペクタ内寸法プレビューは後続 UI-4 で統合。

use crate::app::App;
use squid_n_core::ids::SectionId;
use squid_n_edit::{
    AddCatalogSection, AddSectionShape, EditSectionShape, SectionField, SetSectionField,
    SetSectionName,
};
use squid_n_section::catalog::CatalogShape;
use squid_n_section::shape::{
    BeamStirrup, CircleColumnHoop, RcBeamRebar, RcCircleColumnRebar, RcRectColumnRebar,
    RectColumnHoop, SectionShape,
};

/// 断面作成UIのドラフト状態。App に保持して UI を跨いで維持。
#[derive(Debug, Clone)]
pub struct SectionEditorDraft {
    pub kind: ShapeKind,
    /// 断面符号。階と組で断面の同一性キーになる。
    pub name: String,
    /// 階（空欄は「階の指定なし」）。ST-Bridge 由来の断面は階を持つため、
    /// 既存断面の符号・階を直すときにもこの欄を使う。
    pub floor: String,
    /// 符号・階の欄へ内容を読み込み済みの断面。断面テーブルで別の断面を選ぶたびに
    /// その断面の符号・階を欄へ写すための記録で、同じ断面を選び直しても
    /// 編集途中の入力を上書きしないようにする。
    pub synced_focus: Option<SectionId>,
    pub h: f64,
    pub b: f64,
    pub tw: f64,
    pub tf: f64,
    pub t: f64,
    /// 角形鋼管の角部外半径 r [mm]（0 は角部を直角とみなす）。
    pub r: f64,
    pub lip: f64,
    pub upper_width: f64,
    pub upper_thick: f64,
    pub lower_width: f64,
    pub lower_thick: f64,
    pub leg_a: f64,
    pub leg_b: f64,
    pub leg_thick: f64,
    pub outer_dia: f64,
    pub thick: f64,
    pub rc_b: f64,
    pub rc_d: f64,
    pub main_dia: f64,
    pub main_top: Vec<u32>,
    pub main_bottom: Vec<u32>,
    pub main_x: Vec<u32>,
    pub main_y: Vec<u32>,
    pub circle_count: u32,
    pub cover: f64,
    pub shear_dia: f64,
    pub shear_pitch: f64,
    pub shear_legs: u32,
    pub shear_legs_x: u32,
    pub shear_legs_y: u32,
}

impl Default for SectionEditorDraft {
    fn default() -> Self {
        Self {
            kind: ShapeKind::SteelH,
            name: "断面1".to_string(),
            floor: String::new(),
            synced_focus: None,
            h: 400.0,
            b: 200.0,
            tw: 8.0,
            tf: 12.0,
            t: 12.0,
            r: 0.0,
            lip: 20.0,
            upper_width: 200.0,
            upper_thick: 12.0,
            lower_width: 300.0,
            lower_thick: 16.0,
            leg_a: 75.0,
            leg_b: 75.0,
            leg_thick: 9.0,
            outer_dia: 216.0,
            thick: 8.0,
            rc_b: 400.0,
            rc_d: 600.0,
            main_dia: 19.0,
            main_top: vec![3],
            main_bottom: vec![3],
            main_x: vec![3],
            main_y: vec![3],
            circle_count: 8,
            cover: 40.0,
            shear_dia: 13.0,
            shear_pitch: 100.0,
            shear_legs: 2,
            shear_legs_x: 2,
            shear_legs_y: 2,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShapeKind {
    SteelH,
    SteelBox,
    SteelAngle,
    SteelChannel,
    SteelTee,
    SteelPipe,
    SteelFlatBar,
    SteelRoundBar,
    SteelLipChannel,
    SteelBuiltH,
    RcBeamRect,
    RcColumnRect,
    RcColumnCircle,
    RcSlab,
}

impl ShapeKind {
    pub fn label(self) -> &'static str {
        match self {
            ShapeKind::SteelH => "鋼 H形",
            ShapeKind::SteelBox => "鋼 箱形",
            ShapeKind::SteelAngle => "鋼 L形",
            ShapeKind::SteelChannel => "鋼 C形",
            ShapeKind::SteelTee => "鋼 T形",
            ShapeKind::SteelPipe => "鋼 丸鋼管",
            ShapeKind::SteelFlatBar => "鋼 平鋼",
            ShapeKind::SteelRoundBar => "鋼 中実丸鋼",
            ShapeKind::SteelLipChannel => "鋼 リップ溝形",
            ShapeKind::SteelBuiltH => "鋼 非対称組立H形",
            ShapeKind::RcBeamRect => "RC 矩形梁",
            ShapeKind::RcColumnRect => "RC 矩形柱",
            ShapeKind::RcColumnCircle => "RC 円形柱",
            ShapeKind::RcSlab => "RC スラブ",
        }
    }
    pub const ALL: [ShapeKind; 14] = [
        ShapeKind::SteelH,
        ShapeKind::SteelBox,
        ShapeKind::SteelAngle,
        ShapeKind::SteelChannel,
        ShapeKind::SteelTee,
        ShapeKind::SteelPipe,
        ShapeKind::SteelFlatBar,
        ShapeKind::SteelRoundBar,
        ShapeKind::SteelLipChannel,
        ShapeKind::SteelBuiltH,
        ShapeKind::RcBeamRect,
        ShapeKind::RcColumnRect,
        ShapeKind::RcColumnCircle,
        ShapeKind::RcSlab,
    ];
}

/// カタログ選択UIの選択状態（Shape→Family→Name の3段階）。
#[derive(Debug, Clone)]
pub struct CatalogDraft {
    pub shape: CatalogShape,
    pub family: Option<String>,
    pub name: Option<String>,
}

impl Default for CatalogDraft {
    fn default() -> Self {
        Self {
            shape: CatalogShape::H,
            family: None,
            name: None,
        }
    }
}

/// 断面カタログ選択パネル（日本国内規格）。Shape → Family → Name の順に絞り込み、
/// 選んだ断面をそのまま `Section` として追加する。パラメトリック作成（[`section_editor_panel`]）
/// とは別の独立した UI。
pub fn catalog_section_panel(ui: &mut egui::Ui, app: &mut App) {
    ui.group(|ui| {
        ui.strong("断面カタログから選択（日本国内規格）");
        ui.separator();

        ui.horizontal(|ui| {
            ui.label("Shape:");
            for s in CatalogShape::ALL {
                let cur = app.ui.scoped.catalog_draft.shape;
                if ui.selectable_label(cur == s, s.label()).clicked() && cur != s {
                    app.ui.scoped.catalog_draft.shape = s;
                    app.ui.scoped.catalog_draft.family = None;
                    app.ui.scoped.catalog_draft.name = None;
                }
            }
        });

        let families = squid_n_section::catalog::families(app.ui.scoped.catalog_draft.shape);
        if families.is_empty() {
            ui.label("該当する断面がありません");
            return;
        }
        let family = app
            .ui
            .scoped
            .catalog_draft
            .family
            .clone()
            .filter(|f| families.contains(&f.as_str()))
            .unwrap_or_else(|| families[0].to_string());
        if app.ui.scoped.catalog_draft.family.as_deref() != Some(family.as_str()) {
            app.ui.scoped.catalog_draft.family = Some(family.clone());
            app.ui.scoped.catalog_draft.name = None;
        }

        ui.horizontal(|ui| {
            ui.label("Family:");
            egui::ComboBox::from_id_salt("catalog_family_select")
                .selected_text(&family)
                .show_ui(ui, |ui| {
                    for f in &families {
                        if ui.selectable_label(family == *f, *f).clicked() {
                            app.ui.scoped.catalog_draft.family = Some((*f).to_string());
                            app.ui.scoped.catalog_draft.name = None;
                        }
                    }
                });
        });

        let entries =
            squid_n_section::catalog::entries_in(app.ui.scoped.catalog_draft.shape, &family);
        if entries.is_empty() {
            return;
        }
        let name = app
            .ui
            .scoped
            .catalog_draft
            .name
            .clone()
            .filter(|n| entries.iter().any(|e| &e.name == n))
            .unwrap_or_else(|| entries[0].name.clone());
        if app.ui.scoped.catalog_draft.name.as_deref() != Some(name.as_str()) {
            app.ui.scoped.catalog_draft.name = Some(name.clone());
        }

        ui.horizontal(|ui| {
            ui.label("Name:");
            egui::ComboBox::from_id_salt("catalog_name_select")
                .selected_text(&name)
                .show_ui(ui, |ui| {
                    for e in &entries {
                        if ui.selectable_label(name == e.name, &e.name).clicked() {
                            app.ui.scoped.catalog_draft.name = Some(e.name.clone());
                        }
                    }
                });
        });

        let Some(entry) = entries.iter().find(|e| e.name == name) else {
            return;
        };
        ui.separator();
        ui.label(format!(
            "算定: A = {:.3e} mm²   Iy = {:.3e} mm⁴   Iz = {:.3e} mm⁴   J = {:.3e} mm⁴",
            entry.area, entry.iy, entry.iz, entry.j
        ));

        let new_id = SectionId(app.core.model.sections.len() as u32);
        let sec = squid_n_section::catalog::to_section(entry, new_id);
        let taken =
            squid_n_core::model::section_key_taken(&app.core.model.sections, sec.key(), None);
        let add_resp = ui.add_enabled(!taken, egui::Button::new("+ 追加"));
        if taken {
            add_resp.on_hover_text(format!(
                "符号「{}」の断面が既にあります（符号は断面作成パネルで変更できます）",
                sec.name
            ));
        } else if add_resp.clicked() {
            app.core.scoped.undo.run(
                &mut app.core.model,
                Box::new(AddCatalogSection { section: sec }),
            );
            app.core.scoped.staleness.mark_edited();
        }
    });
}

/// 断面作成パネル。モデルタブの断面サブタブに併置。
pub fn section_editor_panel(ui: &mut egui::Ui, app: &mut App) {
    let mut pending_tp: Option<(SectionId, f64)> = None;

    let focused = focused_section_index(app.ui.scoped.nav.focus_section, &app.core.model.sections)
        .map(|idx| &app.core.model.sections[idx]);
    if let Some(sec) = focused {
        if app.ui.scoped.section_draft.synced_focus != Some(sec.id) {
            app.ui.scoped.section_draft.name = sec.name.clone();
            app.ui.scoped.section_draft.floor = sec.floor.clone().unwrap_or_default();
            app.ui.scoped.section_draft.synced_focus = Some(sec.id);
        }
    } else {
        app.ui.scoped.section_draft.synced_focus = None;
    }

    let draft = &mut app.ui.scoped.section_draft;

    ui.group(|ui| {
        ui.strong("断面作成");
        ui.separator();

        ui.horizontal(|ui| {
            ui.label("種別:");
            for k in ShapeKind::ALL {
                if ui.selectable_label(draft.kind == k, k.label()).clicked() {
                    draft.kind = k;
                }
            }
        });

        ui.horizontal(|ui| {
            ui.label("符号:");
            ui.add(egui::TextEdit::singleline(&mut draft.name).desired_width(120.0));
            ui.label("階:");
            ui.add(egui::TextEdit::singleline(&mut draft.floor).desired_width(80.0))
                .on_hover_text(
                    "同じ符号の断面を階ごとに分けて持つための欄です。\
                     空欄なら階の指定なしとして扱います",
                );
        });
        ui.separator();

        let mut predicted_id = SectionId(app.core.model.sections.len() as u32);

        match draft.kind {
            ShapeKind::SteelH => {
                steel_h_fields(ui, draft);
            }
            ShapeKind::SteelBox => {
                steel_box_fields(ui, draft);
            }
            ShapeKind::SteelAngle => {
                steel_angle_fields(ui, draft);
            }
            ShapeKind::SteelChannel => {
                steel_channel_fields(ui, draft);
            }
            ShapeKind::SteelTee => {
                steel_tee_fields(ui, draft);
            }
            ShapeKind::SteelPipe => {
                steel_pipe_fields(ui, draft);
            }
            ShapeKind::SteelFlatBar => {
                steel_flat_bar_fields(ui, draft);
            }
            ShapeKind::SteelRoundBar => {
                steel_round_bar_fields(ui, draft);
            }
            ShapeKind::SteelLipChannel => {
                steel_lip_channel_fields(ui, draft);
            }
            ShapeKind::SteelBuiltH => {
                steel_built_h_fields(ui, draft);
            }
            ShapeKind::RcBeamRect => {
                rc_beam_fields(ui, draft);
            }
            ShapeKind::RcColumnRect => {
                rc_rect_column_fields(ui, draft);
            }
            ShapeKind::RcColumnCircle => {
                rc_circle_column_fields(ui, draft);
            }
            ShapeKind::RcSlab => {
                ui.horizontal(|ui| {
                    ui.label("板厚 t [mm]");
                    ui.add(egui::DragValue::new(&mut draft.thick).speed(1.0));
                });
            }
        }

        ui.separator();

        let shape = build_shape(draft);
        let rebar_validation = shape.validate_rebar();
        if let Err(error) = &rebar_validation {
            ui.colored_label(egui::Color32::RED, format!("配筋エラー: {error}"));
        }
        let sec = shape.to_section(
            SectionId(app.core.model.sections.len() as u32),
            draft.name.clone(),
        );
        ui.label(format!(
            "算定: A = {:.3e} mm²   Iy = {:.3e} mm⁴   Iz = {:.3e} mm⁴   J = {:.3e} mm⁴",
            sec.area, sec.iy, sec.iz, sec.j
        ));

        ui.separator();

        let draft_floor = non_empty(&draft.floor);
        let focus = focused_section_index(app.ui.scoped.nav.focus_section, &app.core.model.sections);
        let key_free_for_add = !squid_n_core::model::section_key_taken(
            &app.core.model.sections,
            (draft.name.as_str(), draft_floor.as_deref()),
            None,
        );
        let key_free_for_rename = !squid_n_core::model::section_key_taken(
            &app.core.model.sections,
            (draft.name.as_str(), draft_floor.as_deref()),
            focus,
        );
        let key_label = match &draft_floor {
            Some(f) => format!("{} ({})", draft.name, f),
            None => draft.name.clone(),
        };

        ui.horizontal(|ui| {
            let can_add = key_free_for_add && !draft.name.trim().is_empty() && rebar_validation.is_ok();
            let add_resp = ui.add_enabled(can_add, egui::Button::new("+ 追加"));
            if !can_add {
                add_resp.on_hover_text(if draft.name.trim().is_empty() {
                    "符号を入力してください".to_string()
                } else {
                    format!("符号＋階「{key_label}」の断面が既にあります。符号か階を変えてください")
                });
            } else if add_resp.clicked() {
                predicted_id = SectionId(app.core.model.sections.len() as u32);
                app.core.scoped.undo.run(
                    &mut app.core.model,
                    Box::new(AddSectionShape {
                        shape: shape.clone(),
                        new_id: predicted_id,
                        name: draft.name.clone(),
                        floor: draft_floor.clone(),
                    }),
                );
                app.core.scoped.staleness.mark_edited();
                let n = app.core.model.sections.len();
                draft.name = format!("断面{}", n + 1);
            }
            ui.separator();

            let apply_resp = ui.add_enabled(focus.is_some() && rebar_validation.is_ok(), egui::Button::new("✏ 選択断面へ適用"));
            match focus {
                Some(idx) => {
                    let sid = app.core.model.sections[idx].id;
                    let name = app.core.model.sections[idx].display_name();
                    let used = app
                        .core.model
                        .elements
                        .iter()
                        .filter(|e| e.section == Some(sid))
                        .count();
                    let apply_resp = apply_resp.on_hover_text(format!(
                        "現在のフォーム内容で断面 {name} の形状を再定義します\
（この断面を使う全 {used} 部材に波及。符号と階は維持されます）"
                    ));
                    if apply_resp.clicked() {
                        app.core.scoped.undo.run(
                            &mut app.core.model,
                            Box::new(EditSectionShape {
                                section: sid,
                                new_shape: shape.clone(),
                            }),
                        );
                        app.core.scoped.staleness.mark_edited();
                    }
                    let can_rename = key_free_for_rename && !draft.name.trim().is_empty();
                    let rename_resp =
                        ui.add_enabled(can_rename, egui::Button::new("✏ 符号・階を変更"));
                    if !can_rename {
                        rename_resp.on_hover_text(if draft.name.trim().is_empty() {
                            "符号を入力してください".to_string()
                        } else {
                            format!("符号＋階「{key_label}」の断面が既にあります")
                        });
                    } else if rename_resp
                        .on_hover_text(format!(
                            "選択中の断面 {name} の符号・階をフォームの内容（{key_label}）へ変更します"
                        ))
                        .clicked()
                    {
                        app.core.scoped.undo.run(
                            &mut app.core.model,
                            Box::new(SetSectionName {
                                id: sid,
                                name: draft.name.clone(),
                                floor: draft_floor.clone(),
                            }),
                        );
                        app.core.scoped.staleness.mark_edited();
                    }
                    let current_tp = app.core.model.sections[idx].panel_thickness.unwrap_or(0.0);
                    panel_thickness_field(ui, sid, current_tp, &mut pending_tp);
                }
                None => {
                    apply_resp.on_hover_text("断面テーブルで対象断面を選択してください");
                }
            }

            ui.separator();
            ui.label(format!("現在: {}/セクション", app.core.model.sections.len()));
        });
    });

    if let Some((id, value)) = pending_tp {
        app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(SetSectionField {
                id,
                field: SectionField::PanelThickness,
                value,
            }),
        );
        app.core.scoped.staleness.mark_edited();
    }
}

/// `focus_section`（ナビゲータで選択中の断面）が現在も存在するか確認し、
/// 存在すれば `sections` 内のインデックスを返す。断面テーブル側での削除等で
/// 参照が古くなっている場合は `None`（ボタン無効化用）。
/// `App` 全体ではなく個別フィールドを引数に取ることで、呼び出し側の
/// `ui.group` クロージャが `app.ui.scoped.section_draft` と `app.core.model` を disjoint に
/// 借用できるようにしている。
fn focused_section_index(
    focus_section: Option<SectionId>,
    sections: &[squid_n_core::model::Section],
) -> Option<usize> {
    let sid = focus_section?;
    let idx = sid.index();
    if idx < sections.len() && sections[idx].id == sid {
        Some(idx)
    } else {
        None
    }
}

fn steel_h_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("H せい:");
        num_field(ui, &mut d.h);
        ui.label("B 幅:");
        num_field(ui, &mut d.b);
        ui.label("tw:");
        num_field(ui, &mut d.tw);
        ui.label("tf:");
        num_field(ui, &mut d.tf);
    });
}

fn steel_box_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("H せい:");
        num_field(ui, &mut d.h);
        ui.label("B 幅:");
        num_field(ui, &mut d.b);
        ui.label("t 板厚:");
        num_field(ui, &mut d.t);
        ui.label("r 角部半径:");
        num_field(ui, &mut d.r);
    });
}

fn steel_angle_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("A 脚長:");
        num_field(ui, &mut d.leg_a);
        ui.label("B 脚長:");
        num_field(ui, &mut d.leg_b);
        ui.label("t 厚:");
        num_field(ui, &mut d.leg_thick);
    });
}

fn steel_channel_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("H せい:");
        num_field(ui, &mut d.h);
        ui.label("B 幅:");
        num_field(ui, &mut d.b);
        ui.label("tw:");
        num_field(ui, &mut d.tw);
        ui.label("tf:");
        num_field(ui, &mut d.tf);
    });
}

fn steel_tee_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("H せい:");
        num_field(ui, &mut d.h);
        ui.label("B 幅:");
        num_field(ui, &mut d.b);
        ui.label("tw:");
        num_field(ui, &mut d.tw);
        ui.label("tf:");
        num_field(ui, &mut d.tf);
    });
}

fn steel_pipe_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("D 外径:");
        num_field(ui, &mut d.outer_dia);
        ui.label("t 板厚:");
        num_field(ui, &mut d.thick);
    });
}

fn steel_flat_bar_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("B 幅:");
        num_field(ui, &mut d.b);
        ui.label("t 板厚:");
        num_field(ui, &mut d.t);
    });
}

fn steel_round_bar_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("D 径:");
        num_field(ui, &mut d.outer_dia);
    });
}

fn steel_lip_channel_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("H せい:");
        num_field(ui, &mut d.h);
        ui.label("B 幅:");
        num_field(ui, &mut d.b);
        ui.label("C リップ:");
        num_field(ui, &mut d.lip);
        ui.label("t 板厚:");
        num_field(ui, &mut d.t);
    });
}

fn steel_built_h_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("H せい:");
        num_field(ui, &mut d.h);
        ui.label("tw ウェブ厚:");
        num_field(ui, &mut d.tw);
    });
    ui.horizontal(|ui| {
        ui.label("上フランジ 幅:");
        num_field(ui, &mut d.upper_width);
        ui.label("厚:");
        num_field(ui, &mut d.upper_thick);
    });
    ui.horizontal(|ui| {
        ui.label("下フランジ 幅:");
        num_field(ui, &mut d.lower_width);
        ui.label("厚:");
        num_field(ui, &mut d.lower_thick);
    });
}

fn rc_rect_dimensions(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("B 幅:");
        num_field(ui, &mut d.rc_b);
        ui.label("D せい:");
        num_field(ui, &mut d.rc_d);
    });
}

fn rc_beam_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("B 幅:");
        num_field(ui, &mut d.rc_b);
        ui.label("D せい:");
        num_field(ui, &mut d.rc_d);
    });
    rc_common_fields(ui, d);
    rebar_layers(ui, "上端筋", &mut d.main_top);
    rebar_layers(ui, "下端筋", &mut d.main_bottom);
    ui.horizontal(|ui| {
        ui.label("あばら筋 径:");
        size_field(ui, "sec_beam_stirrup_dia", &mut d.shear_dia);
        ui.label("ピッチ:");
        num_field(ui, &mut d.shear_pitch);
        ui.label("脚数:");
        int_field(ui, &mut d.shear_legs);
    });
}

fn rc_rect_column_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    rc_rect_dimensions(ui, d);
    rc_common_fields(ui, d);
    rebar_layers(ui, "X 主筋", &mut d.main_x);
    rebar_layers(ui, "Y 主筋", &mut d.main_y);
    ui.horizontal(|ui| {
        ui.label("帯筋 径:");
        size_field(ui, "sec_column_hoop_dia", &mut d.shear_dia);
        ui.label("ピッチ:");
        num_field(ui, &mut d.shear_pitch);
        ui.label("X 脚数:");
        int_field(ui, &mut d.shear_legs_x);
        ui.label("Y 脚数:");
        int_field(ui, &mut d.shear_legs_y);
    });
}

fn rc_circle_column_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.horizontal(|ui| {
        ui.label("D 径:");
        num_field(ui, &mut d.rc_d);
    });
    rc_common_fields(ui, d);
    ui.horizontal(|ui| {
        ui.label("主筋 総本数:");
        int_field(ui, &mut d.circle_count);
        ui.label("帯筋 径:");
        size_field(ui, "sec_circle_hoop_dia", &mut d.shear_dia);
        ui.label("ピッチ:");
        num_field(ui, &mut d.shear_pitch);
    });
}

fn rc_common_fields(ui: &mut egui::Ui, d: &mut SectionEditorDraft) {
    ui.separator();
    ui.strong("配筋");
    ui.label("鉄筋・せん断補強筋の材料は断面テーブルで割り当てます");
    ui.horizontal(|ui| {
        ui.label("主筋径:");
        size_field(ui, "sec_main_dia", &mut d.main_dia);
        ui.label("かぶり:");
        num_field(ui, &mut d.cover);
    });
}

fn rebar_layers(ui: &mut egui::Ui, label: &str, layers: &mut Vec<u32>) {
    ui.strong(label);
    let mut remove = None;
    for (index, count) in layers.iter_mut().enumerate() {
        ui.horizontal(|ui| {
            ui.label(format!("段{} 本数:", index + 1));
            int_field(ui, count);
            if ui.button("削除").clicked() {
                remove = Some(index);
            }
        });
    }
    if let Some(index) = remove {
        layers.remove(index);
    }
    if ui.button("段を追加").clicked() {
        layers.push(3);
    }
}

/// 鉄筋の呼び名サイズ入力（`D10`〜`D41`）。値は呼び名の数値で保持する。
fn size_field(ui: &mut egui::Ui, id: &str, val: &mut f64) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(format!("D{}", *val as i64))
        .show_ui(ui, |ui| {
            for &s in squid_n_core::material_grade::REBAR_NOMINAL_SIZES {
                if ui
                    .selectable_label((*val - s).abs() < 1e-9, format!("D{}", s as i64))
                    .clicked()
                {
                    *val = s;
                }
            }
        });
}

fn num_field(ui: &mut egui::Ui, val: &mut f64) {
    ui.add(
        egui::DragValue::new(val)
            .speed(1.0)
            .range(0.0..=1e6)
            .max_decimals(1),
    );
}

fn int_field(ui: &mut egui::Ui, val: &mut u32) {
    let mut tmp = *val as f64;
    let resp = ui.add(
        egui::DragValue::new(&mut tmp)
            .speed(1.0)
            .range(0.0..=10_000.0)
            .max_decimals(0),
    );
    if resp.changed() {
        let n = tmp.round() as u32;
        if n != *val {
            *val = n;
        }
    }
}

/// 空白のみの入力は「未設定」として `None` にする。
fn non_empty(s: &str) -> Option<String> {
    let t = s.trim();
    (!t.is_empty()).then(|| t.to_string())
}

fn build_shape(d: &SectionEditorDraft) -> SectionShape {
    match d.kind {
        ShapeKind::SteelH => SectionShape::SteelH {
            height: d.h,
            width: d.b,
            web_thick: d.tw,
            flange_thick: d.tf,
        },
        ShapeKind::SteelBox => SectionShape::SteelBox {
            height: d.h,
            width: d.b,
            thick: d.t,
            corner_r: d.r,
        },
        ShapeKind::SteelAngle => SectionShape::SteelAngle {
            leg_a: d.leg_a,
            leg_b: d.leg_b,
            thick: d.leg_thick,
        },
        ShapeKind::SteelChannel => SectionShape::SteelChannel {
            height: d.h,
            width: d.b,
            web_thick: d.tw,
            flange_thick: d.tf,
        },
        ShapeKind::SteelTee => SectionShape::SteelTee {
            height: d.h,
            width: d.b,
            web_thick: d.tw,
            flange_thick: d.tf,
        },
        ShapeKind::SteelPipe => SectionShape::SteelPipe {
            outer_dia: d.outer_dia,
            thick: d.thick,
        },
        ShapeKind::SteelFlatBar => SectionShape::SteelFlatBar {
            width: d.b,
            thick: d.t,
        },
        ShapeKind::SteelRoundBar => SectionShape::SteelRoundBar { dia: d.outer_dia },
        ShapeKind::SteelLipChannel => SectionShape::SteelLipChannel {
            height: d.h,
            width: d.b,
            lip: d.lip,
            thick: d.t,
        },
        ShapeKind::SteelBuiltH => SectionShape::SteelBuiltH {
            height: d.h,
            upper_width: d.upper_width,
            upper_thick: d.upper_thick,
            lower_width: d.lower_width,
            lower_thick: d.lower_thick,
            web_thick: d.tw,
        },
        ShapeKind::RcBeamRect => SectionShape::RcBeamRect {
            b: d.rc_b,
            d: d.rc_d,
            rebar: RcBeamRebar {
                main_dia: d.main_dia,
                top: nonzero_layers(&d.main_top),
                bottom: nonzero_layers(&d.main_bottom),
                cover: d.cover,
                stirrup: BeamStirrup {
                    dia: d.shear_dia,
                    pitch: d.shear_pitch,
                    legs: d.shear_legs,
                },
            },
        },
        ShapeKind::RcColumnRect => SectionShape::RcColumnRect {
            b: d.rc_b,
            d: d.rc_d,
            rebar: RcRectColumnRebar {
                main_dia: d.main_dia,
                x: nonzero_layers(&d.main_x),
                y: nonzero_layers(&d.main_y),
                cover: d.cover,
                hoop: RectColumnHoop {
                    dia: d.shear_dia,
                    pitch: d.shear_pitch,
                    legs_x: d.shear_legs_x,
                    legs_y: d.shear_legs_y,
                },
            },
        },
        ShapeKind::RcColumnCircle => SectionShape::RcColumnCircle {
            d: d.rc_d,
            rebar: RcCircleColumnRebar {
                main_dia: d.main_dia,
                count: d.circle_count,
                cover: d.cover,
                hoop: CircleColumnHoop {
                    dia: d.shear_dia,
                    pitch: d.shear_pitch,
                },
            },
        },
        ShapeKind::RcSlab => SectionShape::RcSlab { thickness: d.thick },
    }
}

fn nonzero_layers(layers: &[u32]) -> Vec<u32> {
    let result: Vec<_> = layers.iter().copied().filter(|count| *count > 0).collect();
    if result.is_empty() {
        vec![3]
    } else {
        result
    }
}

/// 選択中の断面へ仕口パネルの板厚を入力する欄。
///
/// ダイアフラム補強・ダブラープレートで接合部の板厚を増した場合に、柱の断面形状から
/// 算定される値（H 形＝ウェブ厚、角形・円形＝板厚）を上書きする。空欄・0 は未入力で、
/// 断面形状からの算定値を用いる）。
///
/// 仕口パネルのモデル化と S 造パネルゾーンの断面算定の双方がこの値を使う。
fn panel_thickness_field(
    ui: &mut egui::Ui,
    sid: SectionId,
    current: f64,
    pending: &mut Option<(SectionId, f64)>,
) {
    let id_buf = ui.id().with(("panel_thickness", sid.0));
    let mut buf: String = ui.data_mut(|d| d.get_temp(id_buf)).unwrap_or_else(|| {
        if current > 0.0 {
            format!("{current:.1}")
        } else {
            String::new()
        }
    });

    ui.horizontal(|ui| {
        ui.label("パネル板厚 tp [mm]");
        let resp = ui.add(egui::TextEdit::singleline(&mut buf).desired_width(60.0));
        let resp = resp.on_hover_text(
            "柱梁接合部の仕口パネルの板厚。ダイアフラム補強・ダブラープレートで\
             増厚した場合に入力します。空欄なら柱の断面形状から算定します\
             （H 形＝ウェブ厚、角形・円形＝板厚）",
        );
        if resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
            let value = buf.trim().parse::<f64>().unwrap_or(0.0).max(0.0);
            if (value - current).abs() > 1e-9 {
                *pending = Some((sid, value));
            }
        }
        if current > 0.0 {
            ui.label("（形状からの算定値を上書き中）");
        }
    });

    ui.data_mut(|d| d.insert_temp(id_buf, buf));
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_build_shape_steel_h_uses_draft_fields() {
        let d = SectionEditorDraft {
            h: 500.0,
            b: 250.0,
            tw: 10.0,
            tf: 15.0,
            ..SectionEditorDraft::default()
        };
        let s = build_shape(&d);
        if let SectionShape::SteelH {
            height,
            width,
            web_thick,
            flange_thick,
        } = s
        {
            assert_eq!(height, 500.0);
            assert_eq!(width, 250.0);
            assert_eq!(web_thick, 10.0);
            assert_eq!(flange_thick, 15.0);
        } else {
            panic!("expected SteelH");
        }
    }

    /// 角形鋼管の角部外半径 r は draft.r から SectionShape::SteelBox.corner_r
    /// へそのまま配線される（未入力時は既定値 0 で角部直角扱い）。
    #[test]
    fn test_build_shape_steel_box_wires_corner_r() {
        let d = SectionEditorDraft {
            kind: ShapeKind::SteelBox,
            h: 300.0,
            b: 300.0,
            t: 12.0,
            r: 30.0,
            ..SectionEditorDraft::default()
        };
        let s = build_shape(&d);
        if let SectionShape::SteelBox {
            height,
            width,
            thick,
            corner_r,
        } = s
        {
            assert_eq!(height, 300.0);
            assert_eq!(width, 300.0);
            assert_eq!(thick, 12.0);
            assert_eq!(corner_r, 30.0);
        } else {
            panic!("expected SteelBox");
        }
        assert_eq!(SectionEditorDraft::default().r, 0.0, "r の既定値は 0");
    }

    #[test]
    fn test_build_shape_rc_kinds_keep_real_rebar() {
        let d = SectionEditorDraft {
            kind: ShapeKind::RcBeamRect,
            rc_b: 400.0,
            rc_d: 800.0,
            main_top: vec![4, 2],
            main_bottom: vec![3, 3],
            ..SectionEditorDraft::default()
        };
        let SectionShape::RcBeamRect { b, d: depth, rebar } = build_shape(&d) else {
            panic!("RcBeamRect");
        };
        assert_eq!((b, depth), (400.0, 800.0));
        assert_eq!(rebar.top, vec![4, 2]);
        assert_eq!(rebar.bottom, vec![3, 3]);
        let mut layers = rebar.top;
        layers.push(5);
        layers.remove(1);
        assert_eq!(layers, vec![4, 5]);

        let d = SectionEditorDraft {
            kind: ShapeKind::RcColumnRect,
            main_x: vec![4, 2],
            main_y: vec![3],
            shear_legs_x: 3,
            shear_legs_y: 4,
            ..SectionEditorDraft::default()
        };
        let SectionShape::RcColumnRect { rebar, .. } = build_shape(&d) else {
            panic!("RcColumnRect");
        };
        assert_eq!(rebar.x, vec![4, 2]);
        assert_eq!(rebar.y, vec![3]);
        assert_eq!((rebar.hoop.legs_x, rebar.hoop.legs_y), (3, 4));
        assert!(SectionShape::RcColumnRect {
            b: 400.0,
            d: 600.0,
            rebar
        }
        .validate_rebar()
        .is_err());

        let d = SectionEditorDraft {
            kind: ShapeKind::RcColumnCircle,
            circle_count: 12,
            ..SectionEditorDraft::default()
        };
        let SectionShape::RcColumnCircle { rebar, .. } = build_shape(&d) else {
            panic!("RcColumnCircle");
        };
        assert_eq!(rebar.count, 12);
        assert!(!ShapeKind::ALL
            .iter()
            .any(|kind| kind.label().contains("円形梁")));
    }

    #[test]
    fn test_to_section_preview_matches_drafted_shape() {
        let d = SectionEditorDraft::default();
        let s = build_shape(&d);
        let sec = s.to_section(SectionId(0), "test".into());
        // H 400x200x8x12 の A は閉形式
        let expected = 2.0 * 200.0 * 12.0 + (400.0 - 24.0) * 8.0;
        assert!((sec.area - expected).abs() < 1e-9);
    }

    /// 「選択断面へ適用」ボタンが発行する `EditSectionShape` を undo.run 経由で
    /// 適用すると断面性能（A 等）が再算定され、undo で元の断面形状に戻ることを確認する。
    /// GUI（egui）非依存で、draft→shape 構築 (`build_shape`) と undo スタックのみで検証する。
    #[test]
    fn test_edit_section_shape_via_undo_recomputes_and_reverts() {
        use squid_n_core::model::Model;
        use squid_n_edit::UndoStack;

        // 既存断面（H 400x200x8x12）を用意
        let old_draft = SectionEditorDraft::default();
        let old_shape = build_shape(&old_draft);
        let sid = SectionId(0);
        let old_sec = old_shape.to_section(sid, "既存断面".to_string());
        let old_area = old_sec.area;

        let mut model = Model::default();
        model.sections.push(old_sec);

        // フォーム（draft）で寸法を変更 → 断面編集パネルの「適用」と同じ経路で shape を構築
        let new_draft = SectionEditorDraft {
            h: 500.0,
            b: 250.0,
            tw: 10.0,
            tf: 15.0,
            ..SectionEditorDraft::default()
        };
        let new_shape = build_shape(&new_draft);
        let new_area_expected = new_shape.to_section(sid, "既存断面".into()).area;
        assert!((new_area_expected - old_area).abs() > 1e-6);

        let mut undo = UndoStack::new();
        undo.run(
            &mut model,
            Box::new(EditSectionShape {
                section: sid,
                new_shape,
            }),
        );

        // 再算定された断面性能が反映され、名称は維持される
        assert!((model.sections[0].area - new_area_expected).abs() < 1e-6);
        assert_eq!(model.sections[0].name, "既存断面");

        // undo で元の形状・断面性能に戻る
        undo.undo(&mut model);
        assert!((model.sections[0].area - old_area).abs() < 1e-6);
        assert_eq!(model.sections[0].name, "既存断面");
    }
}
