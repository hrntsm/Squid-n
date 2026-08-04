use crate::app::App;
use squid_n_core::ids::SectionId;
use squid_n_edit::{DeleteSection, SectionField, SetSectionField, SetSectionName};

pub fn sections_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::Column;

    let n = app.model.sections.len();
    let mut pending_name: Vec<(usize, String)> = Vec::new();
    let mut pending_field: Vec<(usize, SectionField, f64)> = Vec::new();
    let mut pending_delete: Option<SectionId> = None;
    let mut pending_focus: Option<SectionId> = None;

    // 編集バッファ（名称）
    let mut name_buf: Vec<String> = app.model.sections.iter().map(|s| s.name.clone()).collect();
    // 編集バッファ（数値フィールド）
    let mut num_bufs: Vec<[String; 7]> = app
        .model
        .sections
        .iter()
        .map(|s| {
            [
                format!("{:.1}", s.area),
                format!("{:.1}", s.iy),
                format!("{:.1}", s.iz),
                format!("{:.1}", s.j),
                format!("{:.1}", s.depth),
                format!("{:.1}", s.width),
                format!("{:.1}", s.as_y),
            ]
        })
        .collect();

    crate::table_util::standard_table(
        ui,
        "sections_tbl_0",
        &[
            Column::auto(),
            Column::initial(100.0),
            Column::initial(80.0),
            Column::initial(80.0),
            Column::initial(80.0),
            Column::initial(80.0),
            Column::auto(),
        ],
        &["ID", "名称", "A", "Iy", "Iz", "J", ""],
        n,
        |row| {
            let i = row.index();
            let sec = &app.model.sections[i];
            row.col(|ui| {
                let sid = sec.id;
                let is_sel = app.nav.focus_section == Some(sid);
                if ui
                    .add(egui::Button::selectable(is_sel, sid.0.to_string()))
                    .on_hover_text("クリックでインスペクタに断面詳細を表示")
                    .clicked()
                {
                    pending_focus = Some(sid);
                }
            });
            row.col(|ui| {
                let resp = ui.add(
                    egui::TextEdit::singleline(&mut name_buf[i])
                        .desired_width(90.0)
                        .clip_text(false),
                );
                if resp.lost_focus() && resp.changed() {
                    let trimmed = name_buf[i].trim().to_string();
                    if trimmed != sec.name && !trimmed.is_empty() {
                        pending_name.push((i, trimmed));
                    }
                }
            });
            // A, Iy, Iz, J
            let fields = [
                SectionField::Area,
                SectionField::Iy,
                SectionField::Iz,
                SectionField::J,
            ];
            for (k, field) in fields.iter().enumerate() {
                row.col(|ui| {
                    let buf = &mut num_bufs[i][k];
                    let resp = ui.add(
                        egui::TextEdit::singleline(buf)
                            .desired_width(70.0)
                            .clip_text(false),
                    );
                    if resp.lost_focus() && resp.changed() {
                        if let Ok(val) = buf.trim().parse::<f64>() {
                            let old = match field {
                                SectionField::Area => sec.area,
                                SectionField::Iy => sec.iy,
                                SectionField::Iz => sec.iz,
                                SectionField::J => sec.j,
                                _ => 0.0,
                            };
                            if (val - old).abs() > 1e-9 {
                                pending_field.push((i, *field, val));
                            }
                        }
                    }
                    if buf.trim().parse::<f64>().is_err() {
                        ui.painter().rect_filled(
                            resp.rect,
                            0.0,
                            crate::theme::translucent(crate::theme::ERROR_RED, 60),
                        );
                    }
                });
            }
            row.col(|ui| {
                let sec_id = app.model.sections[i].id;
                let in_use = app.model.elements.iter().any(|e| e.section == Some(sec_id))
                    || app
                        .model
                        .slabs
                        .iter()
                        .any(|s| s.joists.iter().any(|j| j.section == Some(sec_id)));
                let btn = ui.add_enabled(!in_use, egui::Button::new("🗑"));
                if in_use {
                    btn.on_hover_text("部材・小梁から参照中のため削除できません");
                } else if btn.clicked() {
                    pending_delete = Some(sec_id);
                }
            });
        },
    );

    // 確定処理
    let (had_name, had_field) = (!pending_name.is_empty(), !pending_field.is_empty());
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
    for (i, name) in pending_name {
        let sid = SectionId(app.model.sections[i].id.0);
        app.undo
            .run(&mut app.model, Box::new(SetSectionName { id: sid, name }));
    }
    for (i, field, val) in pending_field {
        let sid = SectionId(app.model.sections[i].id.0);
        app.undo.run(
            &mut app.model,
            Box::new(SetSectionField {
                id: sid,
                field,
                value: val,
            }),
        );
    }

    // 編集があった場合は下流（結果・設計）を stale にする（UI設計 §5）
    if had_name || had_field {
        app.staleness.mark_edited();
    }
}
