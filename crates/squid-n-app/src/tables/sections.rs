//! モデルタブ「断面」の一覧表。
//!
//! 断面は符号＋階で一意に定まり、断面形状・断面性能はそこから導かれる結果なので、
//! 断面性能の欄は読み取り専用にしている。断面を作る・形状を変える・符号や階を
//! 直すのは断面作成パネル（[`crate::section_editor`]）が担う。
//!
//! 材料は断面が持つため、割り当てはこの表で行う。主材料に加えて、RC・SRC 断面では
//! 主筋・せん断補強筋・内蔵鉄骨の材料も個別に指定できる。断面形状から使わないと
//! わかる欄（鋼断面の主筋など）は淡色の「—」にして選べないようにしている。
//!
//! 断面性能は cm 系で表示する（内部は mm 系で保持。準備計算の断面性能表と同じ表記）。

use crate::app::App;
use squid_n_core::ids::{MaterialId, SectionId};
use squid_n_core::model::Model;
use squid_n_core::section_shape::SectionShape;
use squid_n_edit::{DeleteSection, SectionMaterialRole, SetSectionMaterial};

/// 断面性能の表示単位（cm 系）への換算。慣例の情報源は `squid-n-core`。
use squid_n_core::units::to_display::{area_cm2 as to_cm2, inertia_cm4 as to_cm4};

/// 材料の役割ごとに、その断面形状で使う欄かどうかを返す。
///
/// 形状定義を持たない断面（断面性能の数値直入力）は鉄筋量も内蔵鉄骨も持たない
/// ため、主材料の欄だけを有効にする。
fn role_applies(shape: Option<&SectionShape>, role: SectionMaterialRole) -> bool {
    let Some(shape) = shape else {
        return role == SectionMaterialRole::Main;
    };
    match role {
        SectionMaterialRole::Main => true,
        // 主筋・せん断補強筋は配筋を持つ断面のみ。
        SectionMaterialRole::Rebar | SectionMaterialRole::ShearRebar => matches!(
            shape,
            SectionShape::RcRect { .. }
                | SectionShape::RcCircle { .. }
                | SectionShape::SrcRect { .. }
                | SectionShape::RcWall { .. }
        ),
        // 内蔵鉄骨は SRC のみ。スラブは配筋も内蔵鉄骨も持たないため、
        // 主材料（コンクリート）の欄だけが有効になる。
        SectionMaterialRole::Steel => matches!(shape, SectionShape::SrcRect { .. }),
    }
}

/// 断面の役割別材料を返す。
fn role_material(
    sec: &squid_n_core::model::Section,
    role: SectionMaterialRole,
) -> Option<MaterialId> {
    match role {
        SectionMaterialRole::Main => sec.material,
        SectionMaterialRole::Rebar => sec.rebar_material,
        SectionMaterialRole::ShearRebar => sec.shear_rebar_material,
        SectionMaterialRole::Steel => sec.steel_material,
    }
}

/// 材料割り当てセル。選択されたら `pending` に積む（描画中はモデルを変えない）。
fn material_cell(
    ui: &mut egui::Ui,
    model: &Model,
    sec: &squid_n_core::model::Section,
    role: SectionMaterialRole,
    id_salt: &str,
    pending: &mut Vec<(SectionId, SectionMaterialRole, Option<MaterialId>)>,
) {
    use crate::table_util;

    if !role_applies(sec.shape.as_ref(), role) {
        table_util::muted_cell(ui, "—", "この断面形状では使いません");
        return;
    }
    let current = role_material(sec, role);
    let label = current
        .and_then(|mid| model.materials.get(mid.index()))
        .map(|m| m.name.clone())
        .unwrap_or_else(|| "―".to_string());
    table_util::cell_combo(ui, format!("{id_salt}_{}", sec.id.0), label, |ui| {
        if ui.selectable_label(current.is_none(), "―").clicked() {
            pending.push((sec.id, role, None));
        }
        for mat in &model.materials {
            if ui
                .selectable_label(current == Some(mat.id), &mat.name)
                .clicked()
            {
                pending.push((sec.id, role, Some(mat.id)));
            }
        }
    });
}

pub fn sections_table(ui: &mut egui::Ui, app: &mut App) {
    use crate::table_util::{self, Col};

    let n = app.model.sections.len();
    let mut pending_delete: Option<SectionId> = None;
    let mut pending_focus: Option<SectionId> = None;
    let mut pending_material: Vec<(SectionId, SectionMaterialRole, Option<MaterialId>)> =
        Vec::new();

    // 断面ごとの参照数。行ごとに全部材を走査すると O(断面数×部材数) になるため、
    // 表の描画前に 1 回だけ数える。数える対象は削除ガード
    // （`squid_n_edit` の `section_in_use`）と揃える必要がある。ここでの 0 が
    // そのまま「削除できる」の判定になるため、片方だけ数え漏らすと削除ボタンが
    // 押せるのにコマンドが Noop になり、無反応に見えてしまう。
    let mut n_elements: Vec<usize> = vec![0; n];
    let count = |sid: Option<SectionId>, n_elements: &mut [usize]| {
        if let Some(sid) = sid {
            if let Some(c) = n_elements.get_mut(sid.index()) {
                *c += 1;
            }
        }
    };
    for e in &app.model.elements {
        count(e.section, &mut n_elements);
    }
    for s in &app.model.floor_regions {
        // 床も断面を参照する（板厚・自重の情報源）。削除ガードが数える対象と
        // そろえないと、使用部材数 0 の行で削除ボタンが押せるのに Noop になる。
        count(s.section(), &mut n_elements);
        for j in s.joist_lines() {
            count(j.section, &mut n_elements);
        }
    }
    for sm in &app.model.secondary_members {
        count(sm.section, &mut n_elements);
    }

    table_util::standard_table(
        ui,
        "sections_tbl_0",
        &[
            Col::id(),
            Col::name("符号"),
            Col::label("階"),
            Col::text("断面形状"),
            Col::name("材料"),
            Col::name("主筋"),
            Col::name("せん断補強筋"),
            Col::name("内蔵鉄骨"),
            Col::num("部材数"),
            Col::wide_num("D×B [mm]"),
            Col::num("A [cm²]"),
            Col::num("Iy [cm⁴]"),
            Col::num("Iz [cm⁴]"),
            Col::num("J [cm⁴]"),
            Col::wide_num("Asy/Asz [cm²]"),
            Col::actions(),
        ],
        n,
        |row| {
            let i = row.index();
            let sec = &app.model.sections[i];
            row.col(|ui| {
                let sid = sec.id;
                let is_sel = app.nav.focus_section == Some(sid);
                if table_util::id_cell(ui, is_sel, sid.0, "クリックでインスペクタに断面詳細を表示")
                {
                    pending_focus = Some(sid);
                }
            });
            row.col(|ui| {
                table_util::text_cell(ui, &sec.name);
            });
            row.col(|ui| {
                // 階を持たない断面（アプリ内で作成した断面など）は符号だけが同一性キー。
                match &sec.floor {
                    Some(f) => table_util::text_cell(ui, f),
                    None => table_util::muted_cell(ui, "—", "階が設定されていません"),
                }
            });
            row.col(|ui| {
                match &sec.shape {
                    Some(shape) => table_util::text_cell(ui, &shape.dimension_label()),
                    // 形状定義を持たない断面は剛性増大率・幅厚比・終局耐力の
                    // 算定対象外になるため、数値直入力であることを示す。
                    None => table_util::muted_cell(
                        ui,
                        "—",
                        "形状定義がありません（断面性能の数値直入力）",
                    ),
                }
            });
            row.col(|ui| {
                material_cell(
                    ui,
                    &app.model,
                    sec,
                    SectionMaterialRole::Main,
                    "sec_mat",
                    &mut pending_material,
                );
            });
            row.col(|ui| {
                material_cell(
                    ui,
                    &app.model,
                    sec,
                    SectionMaterialRole::Rebar,
                    "sec_rebar_mat",
                    &mut pending_material,
                );
            });
            row.col(|ui| {
                material_cell(
                    ui,
                    &app.model,
                    sec,
                    SectionMaterialRole::ShearRebar,
                    "sec_shear_mat",
                    &mut pending_material,
                );
            });
            row.col(|ui| {
                material_cell(
                    ui,
                    &app.model,
                    sec,
                    SectionMaterialRole::Steel,
                    "sec_steel_mat",
                    &mut pending_material,
                );
            });
            row.col(|ui| {
                // どの部材にも使われていない断面は入力漏れ・不要断面の目印。
                if n_elements[i] == 0 {
                    table_util::muted_cell(ui, "0", "どの部材からも参照されていません");
                } else {
                    ui.label(format!("{}", n_elements[i]));
                }
            });
            row.col(|ui| {
                ui.label(format!("{:.0} × {:.0}", sec.depth, sec.width));
            });
            row.col(|ui| {
                ui.label(table_util::fmt_section_prop(to_cm2(sec.area)));
            });
            row.col(|ui| {
                ui.label(table_util::fmt_section_prop(to_cm4(sec.iy)));
            });
            row.col(|ui| {
                ui.label(table_util::fmt_section_prop(to_cm4(sec.iz)));
            });
            row.col(|ui| {
                ui.label(table_util::fmt_section_prop(to_cm4(sec.j)));
            });
            row.col(|ui| {
                ui.label(format!(
                    "{} / {}",
                    table_util::fmt_section_prop(to_cm2(sec.as_y)),
                    table_util::fmt_section_prop(to_cm2(sec.as_z))
                ));
            });
            row.col(|ui| {
                let sec_id = sec.id;
                let blocked = (n_elements[i] > 0)
                    .then_some("部材・小梁・二次部材から参照中のため削除できません");
                if table_util::delete_cell(ui, "この断面を削除", blocked) {
                    pending_delete = Some(sec_id);
                }
            });
        },
    );

    for (section, role, material) in pending_material {
        app.undo.run(
            &mut app.model,
            Box::new(SetSectionMaterial {
                section,
                role,
                material,
            }),
        );
        app.staleness.mark_edited();
    }
    if let Some(sid) = pending_delete {
        app.undo
            .run(&mut app.model, Box::new(DeleteSection { id: sid }));
        if app.nav.focus_section == Some(sid) {
            app.nav.focus_section = None;
        }
        app.staleness.mark_edited();
    }
    if let Some(sid) = pending_focus {
        app.nav.focus_section = Some(sid);
    }
}
