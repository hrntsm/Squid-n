//! モデルタブ「断面」の一覧表。
//!
//! 断面は符号＋階で一意に定まり、断面形状・断面性能はそこから導かれる結果なので、
//! この表は読み取り専用にしている（行の選択と削除のみ操作できる）。断面を作る・
//! 形状を変える・符号や階を直すのは断面作成パネル（[`crate::section_editor`]）が担う。
//!
//! 断面性能は cm 系で表示する（内部は mm 系で保持。準備計算の断面性能表と同じ表記）。

use crate::app::App;
use squid_n_core::ids::SectionId;
use squid_n_edit::DeleteSection;

/// mm² → cm²。
fn to_cm2(mm2: f64) -> f64 {
    mm2 * 1e-2
}

/// mm⁴ → cm⁴。
fn to_cm4(mm4: f64) -> f64 {
    mm4 * 1e-4
}

pub fn sections_table(ui: &mut egui::Ui, app: &mut App) {
    use crate::table_util::{self, Col};

    let n = app.model.sections.len();
    let mut pending_delete: Option<SectionId> = None;
    let mut pending_focus: Option<SectionId> = None;

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
    for s in &app.model.slabs {
        for j in &s.joists {
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
                ui.label(format!("{:.1}", to_cm2(sec.area)));
            });
            row.col(|ui| {
                ui.label(format!("{:.0}", to_cm4(sec.iy)));
            });
            row.col(|ui| {
                ui.label(format!("{:.0}", to_cm4(sec.iz)));
            });
            row.col(|ui| {
                ui.label(format!("{:.0}", to_cm4(sec.j)));
            });
            row.col(|ui| {
                ui.label(format!("{:.1} / {:.1}", to_cm2(sec.as_y), to_cm2(sec.as_z)));
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
