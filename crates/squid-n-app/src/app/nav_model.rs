//! ナビゲータ（左パネル）の断面・材料一覧ツリー。
//!
//! 断面は階を親ノードとしたグループ構造で表示し、材料は区分ごとのグループ構造で
//! 表示する。描画には UI の状態を持ち込まず、グループの組み立てを純関数に切り出して
//! いるため、テスト可能なままナビゲータへ反映できる。

use std::collections::{HashMap, HashSet};

#[cfg(feature = "gui")]
use super::*;
use squid_n_core::ids::{MaterialId, SectionId};
use squid_n_core::model::{Material, MaterialCategory, Section, Story};

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum SectionGroupKey {
    Floor(String),
    NoFloor,
}

/// 断面を階グループごとにまとめる。
///
/// まず `stories` に載る階名と一致する断面を上階から下階へ順に並べる。
/// 次に、階定義にない `floor: Some(name)` の断面を、`sections` の並び順で初登場した
/// 階名ごとに独立したグループへまとめる。`floor: None` の断面だけを末尾の
/// 「（階なし）」グループへ集約する。
pub(crate) fn section_floor_groups(
    stories: &[Story],
    sections: &[Section],
) -> Vec<(SectionGroupKey, Vec<SectionId>)> {
    let mut groups = Vec::new();
    let mut matched_section_ids = HashSet::new();

    for story in stories.iter().rev() {
        let ids: Vec<_> = sections
            .iter()
            .filter(|sec| sec.floor.as_deref() == Some(story.name.as_str()))
            .map(|sec| {
                matched_section_ids.insert(sec.id);
                sec.id
            })
            .collect();
        if !ids.is_empty() {
            groups.push((SectionGroupKey::Floor(story.name.clone()), ids));
        }
    }

    let mut unmatched_floor_index: HashMap<String, usize> = HashMap::new();
    let mut unmatched_floor_groups: Vec<(String, Vec<SectionId>)> = Vec::new();

    for sec in sections {
        let Some(floor_name) = &sec.floor else {
            continue;
        };
        if matched_section_ids.contains(&sec.id) {
            continue;
        }

        let index = *unmatched_floor_index
            .entry(floor_name.clone())
            .or_insert_with(|| {
                unmatched_floor_groups.push((floor_name.clone(), Vec::new()));
                unmatched_floor_groups.len() - 1
            });
        unmatched_floor_groups[index].1.push(sec.id);
    }

    for (floor_name, ids) in unmatched_floor_groups {
        groups.push((SectionGroupKey::Floor(floor_name), ids));
    }

    let no_floor: Vec<_> = sections
        .iter()
        .filter(|sec| sec.floor.is_none())
        .map(|sec| sec.id)
        .collect();
    if !no_floor.is_empty() {
        groups.push((SectionGroupKey::NoFloor, no_floor));
    }

    groups
}

/// 材料区分の UI 表示順。`MaterialCategory` の列挙順とは独立で、
/// 「鋼材 → コンクリート → 鉄筋」に対応する。バリアント追加時はここを更新しないと
/// コンパイルできない。
fn material_category_ui_rank(category: MaterialCategory) -> u8 {
    match category {
        MaterialCategory::Steel => 0,
        MaterialCategory::Concrete => 1,
        MaterialCategory::Rebar => 2,
    }
}

/// 材料区分の表示順。
///
/// UI では「鋼材 → コンクリート → 鉄筋」の順に並べる。`MaterialCategory` の列挙順とは
/// 異なるため、表示上の意図を明確にするために固定順序でまとめる。
pub(crate) fn material_category_groups(
    materials: &[Material],
) -> Vec<(MaterialCategory, Vec<MaterialId>)> {
    let mut groups: Vec<(MaterialCategory, Vec<MaterialId>)> = Vec::new();
    for mat in materials {
        if let Some((_, ids)) = groups.iter_mut().find(|(c, _)| *c == mat.category) {
            ids.push(mat.id);
        } else {
            groups.push((mat.category, vec![mat.id]));
        }
    }
    groups.sort_by_key(|(category, _)| material_category_ui_rank(*category));
    groups
}

#[cfg(feature = "gui")]
impl App {
    /// 断面一覧ツリー（階ごとにグループ化）。
    pub(crate) fn nav_sections(&mut self, ui: &mut egui::Ui) {
        let groups = section_floor_groups(&self.model.stories, &self.model.sections);
        let mut jump: Option<SectionId> = None;

        let header = egui::CollapsingHeader::new("断面一覧")
            .default_open(false)
            .id_salt("nav_sections");
        let _ = header.show(ui, |ui| {
            if self.model.sections.is_empty() {
                ui.colored_label(crate::theme::GRAY_600, "断面がありません");
                return;
            }

            for (group, ids) in &groups {
                let title = match group {
                    SectionGroupKey::Floor(name) => name.clone(),
                    SectionGroupKey::NoFloor => "（階なし）".to_string(),
                };
                let group_header = egui::CollapsingHeader::new(&title)
                    .default_open(false)
                    .id_salt(("nav_sections_group", title.clone()));
                group_header.show(ui, |ui| {
                    for sid in ids {
                        let sec = &self.model.sections[sid.index()];
                        let is_focus = self.nav.focus_section == Some(*sid);
                        let resp = ui
                            .selectable_label(is_focus, format!("[{}] {}", sec.id.0, sec.name))
                            .on_hover_text("クリックで断面テーブルへ移動");
                        if resp.clicked() {
                            jump = Some(*sid);
                        }
                    }
                });
            }
        });

        if let Some(sid) = jump {
            self.active_tab = Tab::Model;
            self.bottom_dock_open = true;
            self.bottom_tab = BottomTab::Model;
            self.model_tab = ModelTab::Sections;
            self.nav.focus_section = Some(sid);
        }
    }

    /// 材料一覧ツリー（材料区分ごとにグループ化）。
    pub(crate) fn nav_materials(&mut self, ui: &mut egui::Ui) {
        let groups = material_category_groups(&self.model.materials);
        let mut jump: Option<MaterialId> = None;

        let header = egui::CollapsingHeader::new("材料一覧")
            .default_open(false)
            .id_salt("nav_materials");
        let _ = header.show(ui, |ui| {
            if self.model.materials.is_empty() {
                ui.colored_label(crate::theme::GRAY_600, "材料がありません");
                return;
            }

            for (category, ids) in &groups {
                let group_header = egui::CollapsingHeader::new(category.label())
                    .default_open(false)
                    .id_salt(("nav_material_group", category.label()));
                group_header.show(ui, |ui| {
                    for mid in ids {
                        let mat = &self.model.materials[mid.index()];
                        let is_focus = self.nav.focus_material == Some(*mid);
                        let resp = ui
                            .selectable_label(is_focus, format!("[{}] {}", mat.id.0, mat.name))
                            .on_hover_text("クリックで材料テーブルへ移動");
                        if resp.clicked() {
                            jump = Some(*mid);
                        }
                    }
                });
            }
        });

        if let Some(mid) = jump {
            self.active_tab = Tab::Model;
            self.bottom_dock_open = true;
            self.bottom_tab = BottomTab::Model;
            self.model_tab = ModelTab::Materials;
            self.nav.focus_material = Some(mid);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::{MaterialId, SectionId, StoryId};

    fn story(name: &str) -> Story {
        Story {
            id: StoryId(0),
            name: name.to_string(),
            elevation: 0.0,
            node_ids: Vec::new(),
            seismic_weight: None,
            weight_override: None,
            structure: squid_n_core::model::StoryStructure::default(),
            level_kind: squid_n_core::model::StoryLevelKind::default(),
        }
    }

    fn section(id: u32, name: &str, floor: Option<&str>) -> Section {
        Section {
            id: SectionId(id),
            name: name.to_string(),
            floor: floor.map(str::to_string),
            area: 0.0,
            iy: 0.0,
            iz: 0.0,
            j: 0.0,
            depth: 0.0,
            width: 0.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
            material: None,
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        }
    }

    fn material(id: u32, name: &str, category: MaterialCategory) -> Material {
        Material {
            id: MaterialId(id),
            name: name.to_string(),
            category,
            young: 0.0,
            poisson: 0.0,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
            concrete_class: squid_n_core::units::ConcreteClass::Normal,
            strength_factor: None,
        }
    }

    #[test]
    fn section_floor_groups_collects_story_and_unmatched_floors() {
        let stories = vec![story("3F"), story("1F")];
        let sections = vec![
            section(0, "A", Some("3F")),
            section(1, "B", Some("2F")),
            section(2, "C", None),
        ];

        let groups = section_floor_groups(&stories, &sections);
        assert_eq!(groups.len(), 3);
        assert_eq!(groups[0].0, SectionGroupKey::Floor("3F".to_string()));
        assert_eq!(groups[0].1, vec![SectionId(0)]);
        assert_eq!(groups[1].0, SectionGroupKey::Floor("2F".to_string()));
        assert_eq!(groups[1].1, vec![SectionId(1)]);
        assert_eq!(groups[2].0, SectionGroupKey::NoFloor);
        assert_eq!(groups[2].1, vec![SectionId(2)]);
    }

    #[test]
    fn section_floor_groups_orders_story_floors_top_to_bottom() {
        // 階定義は下階→上階で格納し、表示は上階→下階。
        let stories = vec![story("1F"), story("2F"), story("3F")];
        let sections = vec![
            section(0, "A", Some("1F")),
            section(1, "B", Some("2F")),
            section(2, "C", Some("3F")),
        ];

        let groups = section_floor_groups(&stories, &sections);
        assert_eq!(
            groups,
            vec![
                (SectionGroupKey::Floor("3F".to_string()), vec![SectionId(2)]),
                (SectionGroupKey::Floor("2F".to_string()), vec![SectionId(1)]),
                (SectionGroupKey::Floor("1F".to_string()), vec![SectionId(0)]),
            ]
        );
    }

    #[test]
    fn section_floor_groups_keeps_unmatched_floors_in_first_seen_order() {
        let stories = vec![story("3F"), story("1F")];
        let sections = vec![
            section(0, "A", Some("2F")),
            section(1, "B", Some("4F")),
            section(2, "C", Some("2F")),
            section(3, "D", None),
        ];

        let groups = section_floor_groups(&stories, &sections);
        assert_eq!(groups.len(), 3);
        assert_eq!(groups[0].0, SectionGroupKey::Floor("2F".to_string()));
        assert_eq!(groups[0].1, vec![SectionId(0), SectionId(2)]);
        assert_eq!(groups[1].0, SectionGroupKey::Floor("4F".to_string()));
        assert_eq!(groups[1].1, vec![SectionId(1)]);
        assert_eq!(groups[2].0, SectionGroupKey::NoFloor);
        assert_eq!(groups[2].1, vec![SectionId(3)]);
    }

    #[test]
    fn section_floor_groups_omits_empty_story_groups() {
        let stories = vec![story("1F"), story("2F")];
        let sections = vec![section(0, "A", Some("1F"))];

        let groups = section_floor_groups(&stories, &sections);
        assert_eq!(groups.len(), 1);
        assert_eq!(groups[0].0, SectionGroupKey::Floor("1F".to_string()));
        assert_eq!(groups[0].1, vec![SectionId(0)]);
    }

    #[test]
    fn material_category_groups_uses_fixed_ui_order() {
        // 入力順は列挙順（鋼材→鉄筋→コンクリート）と食い違わせ、UI 順を固定する。
        let materials = vec![
            material(0, "R1", MaterialCategory::Rebar),
            material(1, "S1", MaterialCategory::Steel),
            material(2, "C1", MaterialCategory::Concrete),
            material(3, "S2", MaterialCategory::Steel),
        ];

        let groups = material_category_groups(&materials);
        assert_eq!(
            groups,
            vec![
                (MaterialCategory::Steel, vec![MaterialId(1), MaterialId(3)]),
                (MaterialCategory::Concrete, vec![MaterialId(2)]),
                (MaterialCategory::Rebar, vec![MaterialId(0)]),
            ]
        );
    }

    #[test]
    fn material_category_ui_rank_covers_all_variants() {
        // 新しい区分を足したら `material_category_ui_rank` の match がコンパイルエラーになる。
        assert_eq!(material_category_ui_rank(MaterialCategory::Steel), 0);
        assert_eq!(material_category_ui_rank(MaterialCategory::Concrete), 1);
        assert_eq!(material_category_ui_rank(MaterialCategory::Rebar), 2);
    }
}
