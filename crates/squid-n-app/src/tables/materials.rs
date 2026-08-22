use crate::app::App;
use squid_n_core::material_grade::{material_presets, MaterialPreset};
use squid_n_core::model::MaterialCategory;
use squid_n_core::units::{
    concrete_unit_weight_kn_m3, to_internal::mass_density_from_unit_weight_kn_m3, ConcreteClass,
    ConcreteComposition,
};
use squid_n_edit::{
    AddMaterial, DeleteMaterial, MaterialField, SetMaterialCategory, SetMaterialField,
    SetMaterialName,
};

/// プリセット追加 UI の選択状態（区分・グレード名・SRC造トグル）。
/// `ui.data`/`data_mut` の temp storage に保持する。
#[derive(Clone, Debug)]
struct PresetDraft {
    category: MaterialCategory,
    name: String,
    /// コンクリート区分のみ有効。ON のとき密度を γSRC 由来に差し替える。
    src: bool,
}

impl PresetDraft {
    fn new(presets: &[MaterialPreset], category: MaterialCategory) -> Self {
        Self {
            category,
            name: first_name_in(presets, category),
            src: false,
        }
    }
}

fn first_name_in(presets: &[MaterialPreset], category: MaterialCategory) -> String {
    presets
        .iter()
        .find(|p| p.category == category)
        .map(|p| p.name.to_string())
        .unwrap_or_default()
}

/// プリセットのグレード選択に添えるホバーテキスト（主要値の要約）。
fn preset_hover_text(p: &MaterialPreset) -> String {
    match p.category {
        MaterialCategory::Steel => format!("F={} (t≤40)", p.fy.unwrap_or_default()),
        MaterialCategory::Rebar => format!("降伏点 {}", p.fy.unwrap_or_default()),
        MaterialCategory::Concrete => format!("Fc={}, Ec={:.0}", p.fc.unwrap_or_default(), p.young),
    }
}

/// SRC造（鉄骨鉄筋コンクリート）トグル適用時の材料名・密度を計算する。
///
/// `fc` は元プリセットのコンクリート設計基準強度。密度は単位体積重量表の
/// γSRC（鉄骨鉄筋込み。普通コンクリート・Fc≤36 帯で 25.0 kN/m³）から導出する。
fn apply_src_toggle(name: &str, fc: f64) -> (String, f64) {
    let gamma = concrete_unit_weight_kn_m3(fc, ConcreteClass::Normal, ConcreteComposition::Src);
    let rho = mass_density_from_unit_weight_kn_m3(gamma);
    (format!("{name}(SRC)"), rho)
}

/// 材料タブ：プリセット追加・カスタム追加・一覧編集・削除。
pub fn materials_table(ui: &mut egui::Ui, app: &mut App) {
    use crate::table_util::{self, Col};

    // ── プリセット追加 ─────────────────────────────────────────
    let presets = material_presets();
    let id_preset_draft = egui::Id::new("material_preset_draft");
    let mut draft = ui
        .data(|d| d.get_temp::<PresetDraft>(id_preset_draft))
        .unwrap_or_else(|| PresetDraft::new(&presets, MaterialCategory::Steel));

    ui.horizontal(|ui| {
        ui.label("プリセット追加:");
        for cat in [
            MaterialCategory::Steel,
            MaterialCategory::Rebar,
            MaterialCategory::Concrete,
        ] {
            if ui
                .selectable_label(draft.category == cat, cat.label())
                .clicked()
                && draft.category != cat
            {
                draft.category = cat;
                draft.name = first_name_in(&presets, cat);
                draft.src = false;
            }
        }
    });

    let grades: Vec<&MaterialPreset> = presets
        .iter()
        .filter(|p| p.category == draft.category)
        .collect();
    if !grades.iter().any(|p| p.name == draft.name) {
        draft.name = grades
            .first()
            .map(|p| p.name.to_string())
            .unwrap_or_default();
    }

    ui.horizontal(|ui| {
        ui.label("グレード:");
        egui::ComboBox::from_id_salt("material_preset_select")
            .selected_text(&draft.name)
            .show_ui(ui, |ui| {
                for p in &grades {
                    let hover = preset_hover_text(p);
                    if ui
                        .selectable_label(draft.name == p.name, p.name)
                        .on_hover_text(hover)
                        .clicked()
                    {
                        draft.name = p.name.to_string();
                    }
                }
            });
        if draft.category == MaterialCategory::Concrete {
            ui.checkbox(&mut draft.src, "SRC造(γSRC)");
        }

        let selected = grades.iter().find(|p| p.name == draft.name).copied();
        if let Some(preset) = selected {
            let (name, density) = if draft.category == MaterialCategory::Concrete && draft.src {
                apply_src_toggle(preset.name, preset.fc.unwrap_or_default())
            } else {
                (preset.name.to_string(), preset.density)
            };
            if ui.button("+ 追加").clicked() {
                app.undo.run(
                    &mut app.model,
                    Box::new(AddMaterial {
                        name,
                        category: draft.category,
                        young: preset.young,
                        poisson: preset.poisson,
                        density,
                        fc: preset.fc,
                        fy: preset.fy,
                        strength_factor: None,
                    }),
                );
                app.staleness.mark_edited();
            }
        }
    });
    ui.data_mut(|d| d.insert_temp(id_preset_draft, draft));

    // ── 直接入力（カスタム）フォーム ─────────────────────────────
    // プリセットにない材料は直接入力する。
    let id_draft = egui::Id::new("material_custom_draft");
    // (名称, E, ν, 密度, Fc, Fy, 強度割増係数) の文字列ドラフト
    let mut draft: [String; 7] = ui
        .data(|d| d.get_temp::<[String; 7]>(id_draft))
        .unwrap_or_else(|| {
            [
                "新規材料".into(),
                "205000".into(),
                "0.3".into(),
                // 鋼材の γs=77 kN/m³ を質量密度へ換算した既定値（プリセットと同一）。
                format!(
                    "{:.4e}",
                    mass_density_from_unit_weight_kn_m3(
                        squid_n_core::units::STEEL_UNIT_WEIGHT_KN_M3
                    )
                ),
                String::new(),
                String::new(),
                String::new(),
            ]
        });
    let mut do_add_custom = false;
    // 直接入力材料の区分。部材が S 造か RC 造かはこの値で決まるため、
    // E・ν・ρ と同じく入力が必須の項目として扱う。
    let id_cat = ui.id().with("custom_material_category");
    let mut custom_category: MaterialCategory = ui
        .data_mut(|d| d.get_temp(id_cat))
        .unwrap_or(MaterialCategory::Steel);
    ui.horizontal(|ui| {
        ui.label("直接入力:");
        ui.add(egui::TextEdit::singleline(&mut draft[0]).desired_width(80.0))
            .on_hover_text("名称");
        egui::ComboBox::from_id_salt(id_cat)
            .selected_text(custom_category.label())
            .width(100.0)
            .show_ui(ui, |ui| {
                for cat in [
                    MaterialCategory::Steel,
                    MaterialCategory::Rebar,
                    MaterialCategory::Concrete,
                ] {
                    ui.selectable_value(&mut custom_category, cat, cat.label());
                }
            })
            .response
            .on_hover_text("区分。S 造 / RC 造の判定と検定式の選択に用います");
        for (k, label) in [(1, "E"), (2, "ν"), (3, "ρ"), (4, "Fc"), (5, "Fy")] {
            ui.label(label);
            ui.add(egui::TextEdit::singleline(&mut draft[k]).desired_width(60.0));
        }
        ui.label("割増");
        ui.add(egui::TextEdit::singleline(&mut draft[6]).desired_width(50.0))
            .on_hover_text(
                "保有水平耐力計算（増分解析）の材料強度割増係数。\
                 空欄=自動（鋼材1.1、590N級1.05、RC主筋1.1）",
            );
        let parsed_e = draft[1].parse::<f64>();
        let parsed_nu = draft[2].parse::<f64>();
        let parsed_rho = draft[3].parse::<f64>();
        let ok = parsed_e.is_ok() && parsed_nu.is_ok() && parsed_rho.is_ok();
        if ui
            .add_enabled(ok, egui::Button::new("+ 追加"))
            .on_hover_text("E・ν・ρ は必須。Fc・Fy・割増は空欄可")
            .clicked()
        {
            do_add_custom = true;
        }
    });
    if do_add_custom {
        let fc = draft[4].parse::<f64>().ok();
        let fy = draft[5].parse::<f64>().ok();
        let strength_factor = draft[6].parse::<f64>().ok();
        if let (Ok(e), Ok(nu), Ok(rho)) = (
            draft[1].parse::<f64>(),
            draft[2].parse::<f64>(),
            draft[3].parse::<f64>(),
        ) {
            app.undo.run(
                &mut app.model,
                Box::new(AddMaterial {
                    name: draft[0].clone(),
                    category: custom_category,
                    young: e,
                    poisson: nu,
                    density: rho,
                    fc,
                    fy,
                    strength_factor,
                }),
            );
            app.staleness.mark_edited();
        }
    }
    ui.data_mut(|d| d.insert_temp(id_cat, custom_category));
    ui.data_mut(|d| d.insert_temp(id_draft, draft));
    ui.separator();

    // ── 一覧テーブル（編集・削除） ──────────────────────────────
    let n = app.model.materials.len();
    ui.label(format!("材料一覧（{} 件）", n));
    let mut pending_name: Option<(u32, String)> = None;
    let mut pending_category: Option<(u32, MaterialCategory)> = None;
    let mut pending_field: Option<(u32, MaterialField, Option<f64>)> = None;
    let mut pending_delete: Option<u32> = None;
    let mut pending_focus: Option<squid_n_core::ids::MaterialId> = None;

    table_util::standard_table(
        ui,
        "materials_tbl",
        &[
            Col::id(),
            Col::name("名称"),
            Col::name("区分").hover(
                "S 造 / RC 造の判定に用います。剛域長・仕口パネルの対象・\
                 断面検定の式・数量集計がこの値で変わります",
            ),
            Col::num("E [N/mm²]"),
            Col::num("ν"),
            Col::num("ρ [t/mm³]"),
            Col::num("Fc"),
            Col::num("Fy"),
            Col::num("割増").hover(
                "保有水平耐力計算（増分解析）の材料強度割増係数。\
                 空欄=自動（鋼材1.1、590N級1.05、RC主筋1.1）",
            ),
            Col::actions(),
        ],
        n,
        |row| {
            let idx = row.index();
            let mat = &app.model.materials[idx];
            let mat_id = mat.id;
            row.col(|ui| {
                let is_sel = app.nav.focus_material == Some(mat_id);
                if table_util::id_cell(ui, is_sel, mat_id.0, "クリックで選択") {
                    pending_focus = Some(mat_id);
                }
            });
            row.col(|ui| {
                let mut name = mat.name.clone();
                if table_util::cell_text_edit(ui, &mut name).lost_focus() && name != mat.name {
                    pending_name = Some((mat_id.0, name));
                }
            });
            row.col(|ui| {
                let mut category = mat.category;
                table_util::cell_combo(ui, ("mat_category", mat_id.0), category.label(), |ui| {
                    for cat in [
                        MaterialCategory::Steel,
                        MaterialCategory::Rebar,
                        MaterialCategory::Concrete,
                    ] {
                        ui.selectable_value(&mut category, cat, cat.label());
                    }
                });
                if category != mat.category {
                    pending_category = Some((mat_id.0, category));
                }
            });
            // 数値セル: フォーカス喪失時に確定
            let cells: [(MaterialField, String, bool); 6] = [
                (MaterialField::Young, format!("{}", mat.young), true),
                (MaterialField::Poisson, format!("{}", mat.poisson), true),
                (MaterialField::Density, format!("{:.3e}", mat.density), true),
                (
                    MaterialField::Fc,
                    mat.fc.map(|v| format!("{}", v)).unwrap_or_default(),
                    false,
                ),
                (
                    MaterialField::Fy,
                    mat.fy.map(|v| format!("{}", v)).unwrap_or_default(),
                    false,
                ),
                (
                    MaterialField::StrengthFactor,
                    mat.strength_factor
                        .map(|v| format!("{}", v))
                        .unwrap_or_default(),
                    false,
                ),
            ];
            for (field, current, required) in cells {
                row.col(|ui| {
                    let cell_id = egui::Id::new(("mat_cell", mat_id.0, field as u8));
                    let mut buf = ui
                        .data(|d| d.get_temp::<String>(cell_id))
                        .unwrap_or_else(|| current.clone());
                    let resp = table_util::cell_text_edit(ui, &mut buf);
                    if resp.lost_focus() {
                        let parsed = buf.trim().parse::<f64>().ok();
                        let changed = buf.trim() != current.trim();
                        if changed && (parsed.is_some() || !required) {
                            pending_field = Some((mat_id.0, field, parsed));
                        }
                        ui.data_mut(|d| d.remove::<String>(cell_id));
                    } else if resp.has_focus() {
                        ui.data_mut(|d| d.insert_temp(cell_id, buf));
                    }
                });
            }
            row.col(|ui| {
                // 削除ガード（`squid_n_edit` の `material_in_use`）と数える対象を
                // 揃える。材料は断面が持つため、断面の 4 つの欄すべてを見る。
                let in_use = app.model.sections.iter().any(|s| {
                    [
                        s.material,
                        s.rebar_material,
                        s.shear_rebar_material,
                        s.steel_material,
                    ]
                    .contains(&Some(mat_id))
                });
                let blocked = in_use.then_some("断面から参照中のため削除できません");
                if table_util::delete_cell(ui, "この材料を削除", blocked) {
                    pending_delete = Some(mat_id.0);
                }
            });
        },
    );

    // 確定処理（テーブル描画後に model を可変借用）
    let mut edited = false;
    if let Some((id, name)) = pending_name {
        app.undo.run(
            &mut app.model,
            Box::new(SetMaterialName {
                id: squid_n_core::ids::MaterialId(id),
                name,
            }),
        );
        edited = true;
    }
    if let Some((id, category)) = pending_category {
        app.undo.run(
            &mut app.model,
            Box::new(SetMaterialCategory {
                id: squid_n_core::ids::MaterialId(id),
                category,
            }),
        );
        edited = true;
    }
    if let Some((id, field, value)) = pending_field {
        app.undo.run(
            &mut app.model,
            Box::new(SetMaterialField {
                id: squid_n_core::ids::MaterialId(id),
                field,
                value,
            }),
        );
        edited = true;
    }
    if let Some(id) = pending_delete {
        app.undo.run(
            &mut app.model,
            Box::new(DeleteMaterial {
                id: squid_n_core::ids::MaterialId(id),
            }),
        );
        if app.nav.focus_material == Some(squid_n_core::ids::MaterialId(id)) {
            app.nav.focus_material = None;
        }
        edited = true;
    }
    if let Some(mid) = pending_focus {
        app.nav.focus_material = Some(mid);
    }
    if edited {
        app.staleness.mark_edited();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// コンクリートプリセット（Fc≤36 帯）の密度が γRC=24.0 kN/m³ 由来であることを確認する。
    #[test]
    fn test_concrete_presets_match_unit_weight_table() {
        let presets = material_presets();
        let rc_density = mass_density_from_unit_weight_kn_m3(24.0);
        for name in ["Fc18", "Fc21", "Fc24", "Fc27", "Fc30", "Fc33", "Fc36"] {
            let p = presets
                .iter()
                .find(|p| p.name == name)
                .unwrap_or_else(|| panic!("preset {name} not found"));
            assert_eq!(p.category, MaterialCategory::Concrete);
            assert!(
                (p.density - rc_density).abs() < 1e-18,
                "{name}: density={} expected={}",
                p.density,
                rc_density
            );
        }
    }

    /// SRC造トグル適用時の密度が γSRC=25.0 kN/m³ 由来であることを確認する
    /// （Fc≤36 帯。`apply_src_toggle` に切り出したロジックを直接検証する）。
    #[test]
    fn test_apply_src_toggle_uses_gamma_src() {
        let src_density = mass_density_from_unit_weight_kn_m3(25.0);
        let (name, density) = apply_src_toggle("Fc24", 24.0);
        assert_eq!(name, "Fc24(SRC)");
        assert!(
            (density - src_density).abs() < 1e-18,
            "density={density} expected={src_density}"
        );
    }

    /// 鋼材プリセットの密度が γs=77 kN/m³ 由来であることを確認する
    /// （旧実装の 7.85e-9 ハードコードとは異なる値になる点に注意）。
    #[test]
    fn test_steel_presets_match_unit_weight_table() {
        let presets = material_presets();
        let steel_density = mass_density_from_unit_weight_kn_m3(77.0);
        let ss400 = presets
            .iter()
            .find(|p| p.name == "SS400")
            .expect("preset SS400 not found");
        assert_eq!(ss400.category, MaterialCategory::Steel);
        assert!((ss400.density - steel_density).abs() < 1e-18);
        // 旧実装の固定値 7.85e-9 とは厳密には一致しない（77/9.80665 が真値）。
        assert!((steel_density - 7.85e-9).abs() < 1e-11);
        assert_ne!(steel_density, 7.85e-9);
    }
}
