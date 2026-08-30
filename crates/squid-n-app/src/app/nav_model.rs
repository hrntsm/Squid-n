//! ナビゲータ（左パネル）の断面・材料一覧と、床領域・壁領域・未割当二次部材のツリー。
//!
//! 断面は階を親ノードとしたグループ構造で表示し、材料は区分ごとのグループ構造で
//! 表示する。床領域の子は所属小梁、壁領域の子は間柱。描画には UI の状態を持ち込まず、
//! グループの組み立てを純関数に切り出しているため、テスト可能なままナビゲータへ反映できる。

use std::collections::{HashMap, HashSet};

#[cfg(feature = "gui")]
use super::*;
use squid_n_core::ids::{FloorRegionId, MaterialId, NodeId, SectionId, WallRegionId};
use squid_n_core::model::{
    FloorRegion, Material, MaterialCategory, Model, SecondaryMember, SecondaryMemberKind, Section,
    Slab, Story, WallRegion,
};

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum SectionGroupKey {
    Floor(String),
    Secondary,
    NoFloor,
}

/// 断面を階グループごとにまとめる。
///
/// まず `stories` に載る階名と一致する断面を上階から下階へ順に並べる。
/// 次に、階定義にない `floor: Some(name)` の断面を、`sections` の並び順で初登場した
/// 階名ごとに独立したグループへまとめる。`floor: None` の断面は、二次部材・床板・
/// 床領域の小梁ラインが参照する断面を `Secondary` グループにまとめ、残りを末尾の
/// 「（階なし）」グループへ集約する。階名がある断面は二次部材・床板から参照されて
/// いても階グループに残す。
pub(crate) fn section_floor_groups(
    stories: &[Story],
    sections: &[Section],
    joists_and_posts: &[SecondaryMember],
    slabs: &[Slab],
    floor_regions: &[FloorRegion],
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

    // 小梁・間柱（領域内＋未割当）と手入力小梁ライン・床板が参照する断面を二次部材グループへ。
    let secondary_referenced_ids: HashSet<_> = joists_and_posts
        .iter()
        .filter_map(|member| member.section)
        .chain(slabs.iter().filter_map(|slab| slab.section()))
        .chain(floor_regions.iter().flat_map(|region| {
            region
                .joist_lines()
                .iter()
                .filter_map(|joist| joist.section)
        }))
        .collect();

    let secondary_no_floor: Vec<_> = sections
        .iter()
        .filter(|sec| sec.floor.is_none() && secondary_referenced_ids.contains(&sec.id))
        .map(|sec| sec.id)
        .collect();
    if !secondary_no_floor.is_empty() {
        groups.push((SectionGroupKey::Secondary, secondary_no_floor));
    }

    let no_floor: Vec<_> = sections
        .iter()
        .filter(|sec| sec.floor.is_none() && !secondary_referenced_ids.contains(&sec.id))
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

/// 床領域・壁領域の表示名（空なら `床領域 {id}` / `壁領域 {id}`）。
pub(crate) fn floor_region_display_name(region: &FloorRegion) -> String {
    if region.name.is_empty() {
        format!("床領域 {}", region.id.0)
    } else {
        region.name.clone()
    }
}

pub(crate) fn wall_region_display_name(region: &WallRegion) -> String {
    if region.name.is_empty() {
        format!("壁領域 {}", region.id.0)
    } else {
        region.name.clone()
    }
}

/// 二次部材のナビ表示ラベル（例: `小梁 n1–n2`）。
pub(crate) fn secondary_member_nav_label(sm: &SecondaryMember) -> String {
    let kind = match sm.kind {
        SecondaryMemberKind::Joist => "小梁",
        SecondaryMemberKind::Post => "間柱",
    };
    let (a, b) = (sm.nodes[0].0, sm.nodes[1].0);
    let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
    format!("{kind} n{lo}–n{hi}")
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct FloorRegionNavRow {
    pub id: FloorRegionId,
    pub title: String,
    pub joist_labels: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct WallRegionNavRow {
    pub id: WallRegionId,
    pub title: String,
    pub post_labels: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct UnassignedNavRows {
    pub joist_labels: Vec<String>,
    pub post_labels: Vec<String>,
}

pub(crate) fn floor_region_nav_rows(floor_regions: &[FloorRegion]) -> Vec<FloorRegionNavRow> {
    floor_regions
        .iter()
        .map(|region| FloorRegionNavRow {
            id: region.id,
            title: floor_region_display_name(region),
            joist_labels: region
                .secondary_joists
                .iter()
                .map(secondary_member_nav_label)
                .collect(),
        })
        .collect()
}

pub(crate) fn wall_region_nav_rows(wall_regions: &[WallRegion]) -> Vec<WallRegionNavRow> {
    wall_regions
        .iter()
        .map(|region| WallRegionNavRow {
            id: region.id,
            title: wall_region_display_name(region),
            post_labels: region
                .posts
                .iter()
                .map(secondary_member_nav_label)
                .collect(),
        })
        .collect()
}

pub(crate) fn unassigned_nav_rows(model: &Model) -> UnassignedNavRows {
    UnassignedNavRows {
        joist_labels: model
            .unassigned_joists
            .iter()
            .map(secondary_member_nav_label)
            .collect(),
        post_labels: model
            .unassigned_posts
            .iter()
            .map(secondary_member_nav_label)
            .collect(),
    }
}

#[cfg(feature = "gui")]
fn sanitize_region_focus(model: &Model, nav: &mut Navigator) {
    if let Some(id) = nav.focus_floor_region {
        let ok = model
            .floor_regions
            .get(id.index())
            .is_some_and(|r| r.id == id);
        if !ok {
            nav.focus_floor_region = None;
        }
    }
    if let Some(id) = nav.focus_wall_region {
        let ok = model
            .wall_regions
            .get(id.index())
            .is_some_and(|r| r.id == id);
        if !ok {
            nav.focus_wall_region = None;
        }
    }
}

#[cfg(feature = "gui")]
enum RegionNavAction {
    JumpFloorRegion(FloorRegionId),
    JumpWallRegion(WallRegionId),
    SelectSecondaryNodes([NodeId; 2]),
    RemoveFloorJoist(FloorRegionId, usize),
    RemoveWallPost(WallRegionId, usize),
    DeleteUnassignedJoist(usize),
    DeleteUnassignedPost(usize),
    AddUnassigned(SecondaryMember),
}

#[cfg(feature = "gui")]
impl App {
    /// 床領域・壁領域・未割当二次部材のナビゲータセクション。
    pub(crate) fn nav_regions(&mut self, ui: &mut egui::Ui) {
        sanitize_region_focus(&self.model, &mut self.nav);
        let mut action: Option<RegionNavAction> = None;
        self.nav_floor_regions(ui, &mut action);
        self.nav_wall_regions(ui, &mut action);
        self.nav_unassigned_secondary(ui, &mut action);
        if let Some(action) = action {
            self.apply_region_nav_action(action);
        }
    }

    fn nav_floor_regions(&mut self, ui: &mut egui::Ui, action: &mut Option<RegionNavAction>) {
        let rows = floor_region_nav_rows(&self.model.floor_regions);
        let header = egui::CollapsingHeader::new("床領域")
            .default_open(false)
            .id_salt("nav_floor_regions");
        let _ = header.show(ui, |ui| {
            if rows.is_empty() {
                ui.colored_label(crate::theme::GRAY_600, "床領域がありません");
                return;
            }
            for (row_index, row) in rows.iter().enumerate() {
                let region = &self.model.floor_regions[row_index];
                let is_focus = self.nav.focus_floor_region == Some(row.id);
                let region_header = egui::CollapsingHeader::new(&row.title)
                    .default_open(false)
                    .id_salt(("nav_floor_region", row.id.0));
                let resp = region_header.show(ui, |ui| {
                    for (ji, sm) in region.secondary_joists.iter().enumerate() {
                        ui.horizontal(|ui| {
                            let label = secondary_member_nav_label(sm);
                            let item_resp = ui
                                .selectable_label(false, &label)
                                .on_hover_text("クリックで両端節点を3D選択");
                            if item_resp.clicked() {
                                *action = Some(RegionNavAction::SelectSecondaryNodes(sm.nodes));
                            }
                            if ui
                                .small_button("🗑")
                                .on_hover_text("領域から外す（未割当へ移動）")
                                .clicked()
                            {
                                *action = Some(RegionNavAction::RemoveFloorJoist(row.id, ji));
                            }
                        });
                    }
                    if region.secondary_joists.is_empty() {
                        ui.colored_label(crate::theme::GRAY_600, "小梁がありません");
                    }
                });
                if resp.header_response.clicked() {
                    *action = Some(RegionNavAction::JumpFloorRegion(row.id));
                }
                if is_focus {
                    let rect = resp.header_response.rect;
                    ui.painter().line_segment(
                        [rect.left_bottom(), rect.right_bottom()],
                        egui::Stroke::new(2.0_f32, crate::theme::DATA_BLUE),
                    );
                }
            }
        });
    }

    fn nav_wall_regions(&mut self, ui: &mut egui::Ui, action: &mut Option<RegionNavAction>) {
        let rows = wall_region_nav_rows(&self.model.wall_regions);
        let header = egui::CollapsingHeader::new("壁領域")
            .default_open(false)
            .id_salt("nav_wall_regions");
        let _ = header.show(ui, |ui| {
            if rows.is_empty() {
                ui.colored_label(crate::theme::GRAY_600, "壁領域がありません");
                return;
            }
            for (row_index, row) in rows.iter().enumerate() {
                let region = &self.model.wall_regions[row_index];
                let is_focus = self.nav.focus_wall_region == Some(row.id);
                let region_header = egui::CollapsingHeader::new(&row.title)
                    .default_open(false)
                    .id_salt(("nav_wall_region", row.id.0));
                let resp = region_header.show(ui, |ui| {
                    for (pi, sm) in region.posts.iter().enumerate() {
                        ui.horizontal(|ui| {
                            let label = secondary_member_nav_label(sm);
                            let item_resp = ui
                                .selectable_label(false, &label)
                                .on_hover_text("クリックで両端節点を3D選択");
                            if item_resp.clicked() {
                                *action = Some(RegionNavAction::SelectSecondaryNodes(sm.nodes));
                            }
                            if ui
                                .small_button("🗑")
                                .on_hover_text("領域から外す（未割当へ移動）")
                                .clicked()
                            {
                                *action = Some(RegionNavAction::RemoveWallPost(row.id, pi));
                            }
                        });
                    }
                    if region.posts.is_empty() {
                        ui.colored_label(crate::theme::GRAY_600, "間柱がありません");
                    }
                });
                if resp.header_response.clicked() {
                    *action = Some(RegionNavAction::JumpWallRegion(row.id));
                }
                if is_focus {
                    let rect = resp.header_response.rect;
                    ui.painter().line_segment(
                        [rect.left_bottom(), rect.right_bottom()],
                        egui::Stroke::new(2.0_f32, crate::theme::DATA_BLUE),
                    );
                }
            }
        });
    }

    fn nav_unassigned_secondary(
        &mut self,
        ui: &mut egui::Ui,
        action: &mut Option<RegionNavAction>,
    ) {
        let rows = unassigned_nav_rows(&self.model);
        let header = egui::CollapsingHeader::new("未割当")
            .default_open(false)
            .id_salt("nav_unassigned_secondary");
        let _ = header.show(ui, |ui| {
            if rows.joist_labels.is_empty() && rows.post_labels.is_empty() {
                ui.colored_label(crate::theme::GRAY_600, "未割当の二次部材はありません");
            }
            for (index, sm) in self.model.unassigned_joists.iter().enumerate() {
                ui.horizontal(|ui| {
                    let label = secondary_member_nav_label(sm);
                    if ui
                        .selectable_label(false, &label)
                        .on_hover_text("クリックで両端節点を3D選択")
                        .clicked()
                    {
                        *action = Some(RegionNavAction::SelectSecondaryNodes(sm.nodes));
                    }
                    if ui.small_button("🗑").on_hover_text("削除").clicked() {
                        *action = Some(RegionNavAction::DeleteUnassignedJoist(index));
                    }
                });
            }
            for (index, sm) in self.model.unassigned_posts.iter().enumerate() {
                ui.horizontal(|ui| {
                    let label = secondary_member_nav_label(sm);
                    if ui
                        .selectable_label(false, &label)
                        .on_hover_text("クリックで両端節点を3D選択")
                        .clicked()
                    {
                        *action = Some(RegionNavAction::SelectSecondaryNodes(sm.nodes));
                    }
                    if ui.small_button("🗑").on_hover_text("削除").clicked() {
                        *action = Some(RegionNavAction::DeleteUnassignedPost(index));
                    }
                });
            }

            ui.separator();
            ui.label("未割当へ追加");
            ui.horizontal(|ui| {
                ui.label("種別");
                egui::ComboBox::from_id_salt("nav_unassigned_kind")
                    .selected_text(if self.nav.unassigned_add_post {
                        "間柱"
                    } else {
                        "小梁"
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.nav.unassigned_add_post, false, "小梁");
                        ui.selectable_value(&mut self.nav.unassigned_add_post, true, "間柱");
                    });
            });
            ui.horizontal(|ui| {
                ui.label("節点");
                ui.add(
                    egui::DragValue::new(&mut self.nav.unassigned_add_node_a)
                        .speed(1)
                        .prefix("N"),
                );
                ui.label("–");
                ui.add(
                    egui::DragValue::new(&mut self.nav.unassigned_add_node_b)
                        .speed(1)
                        .prefix("N"),
                );
            });
            ui.horizontal(|ui| {
                ui.label("断面");
                let selected = self
                    .nav
                    .unassigned_add_section
                    .and_then(|sid| self.model.sections.get(sid.index()))
                    .map(|sec| format!("[{}] {}", sec.id.0, sec.name))
                    .unwrap_or_else(|| "（なし）".to_string());
                egui::ComboBox::from_id_salt("nav_unassigned_section")
                    .selected_text(selected)
                    .show_ui(ui, |ui| {
                        if ui
                            .selectable_label(self.nav.unassigned_add_section.is_none(), "（なし）")
                            .clicked()
                        {
                            self.nav.unassigned_add_section = None;
                        }
                        for sec in &self.model.sections {
                            let label = format!("[{}] {}", sec.id.0, sec.name);
                            if ui
                                .selectable_label(
                                    self.nav.unassigned_add_section == Some(sec.id),
                                    label,
                                )
                                .clicked()
                            {
                                self.nav.unassigned_add_section = Some(sec.id);
                            }
                        }
                    });
            });
            if ui.button("+ 追加").clicked() {
                let kind = if self.nav.unassigned_add_post {
                    SecondaryMemberKind::Post
                } else {
                    SecondaryMemberKind::Joist
                };
                let sm = SecondaryMember {
                    kind,
                    nodes: [
                        NodeId(self.nav.unassigned_add_node_a),
                        NodeId(self.nav.unassigned_add_node_b),
                    ],
                    section: self.nav.unassigned_add_section,
                    name: String::new(),
                };
                *action = Some(RegionNavAction::AddUnassigned(sm));
            }
        });
    }

    fn apply_region_nav_action(&mut self, action: RegionNavAction) {
        use squid_n_edit::{
            AddUnassignedJoist, AddUnassignedPost, DeleteUnassignedJoist, DeleteUnassignedPost,
            SetFloorRegionSecondaryJoists, SetWallRegionPosts,
        };

        match action {
            RegionNavAction::JumpFloorRegion(id) => {
                self.active_tab = Tab::Model;
                self.bottom_dock_open = true;
                self.bottom_tab = BottomTab::Model;
                self.model_tab = ModelTab::Slabs;
                self.nav.focus_floor_region = Some(id);
            }
            RegionNavAction::JumpWallRegion(id) => {
                self.active_tab = Tab::Model;
                self.bottom_dock_open = true;
                self.bottom_tab = BottomTab::Model;
                self.model_tab = ModelTab::WallPlates;
                self.nav.focus_wall_region = Some(id);
            }
            RegionNavAction::SelectSecondaryNodes(nodes) => {
                self.selection.nodes = nodes.to_vec();
                self.selection.members.clear();
            }
            RegionNavAction::RemoveFloorJoist(region_id, index) => {
                let Some(region) = self.model.floor_regions.iter().find(|r| r.id == region_id)
                else {
                    return;
                };
                let mut joists = region.secondary_joists.clone();
                if index >= joists.len() {
                    return;
                }
                joists.remove(index);
                if self.undo.run(
                    &mut self.model,
                    Box::new(SetFloorRegionSecondaryJoists {
                        region: region_id,
                        joists,
                    }),
                ) {
                    self.staleness.mark_edited();
                }
            }
            RegionNavAction::RemoveWallPost(region_id, index) => {
                let Some(region) = self.model.wall_regions.iter().find(|r| r.id == region_id)
                else {
                    return;
                };
                let mut posts = region.posts.clone();
                if index >= posts.len() {
                    return;
                }
                posts.remove(index);
                if self.undo.run(
                    &mut self.model,
                    Box::new(SetWallRegionPosts {
                        region: region_id,
                        posts,
                    }),
                ) {
                    self.staleness.mark_edited();
                }
            }
            RegionNavAction::DeleteUnassignedJoist(index) => {
                if self
                    .undo
                    .run(&mut self.model, Box::new(DeleteUnassignedJoist { index }))
                {
                    self.staleness.mark_edited();
                }
            }
            RegionNavAction::DeleteUnassignedPost(index) => {
                if self
                    .undo
                    .run(&mut self.model, Box::new(DeleteUnassignedPost { index }))
                {
                    self.staleness.mark_edited();
                }
            }
            RegionNavAction::AddUnassigned(sm) => {
                let changed = match sm.kind {
                    SecondaryMemberKind::Joist => self
                        .undo
                        .run(&mut self.model, Box::new(AddUnassignedJoist { sm })),
                    SecondaryMemberKind::Post => self
                        .undo
                        .run(&mut self.model, Box::new(AddUnassignedPost { sm })),
                };
                if changed {
                    self.staleness.mark_edited();
                }
            }
        }
    }

    /// 断面一覧ツリー（階ごとにグループ化）。
    pub(crate) fn nav_sections(&mut self, ui: &mut egui::Ui) {
        let secondary: Vec<_> = self
            .model
            .joists()
            .chain(self.model.posts())
            .cloned()
            .collect();
        let groups = section_floor_groups(
            &self.model.stories,
            &self.model.sections,
            &secondary,
            &self.model.slabs,
            &self.model.floor_regions,
        );
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
                    SectionGroupKey::Secondary => "二次部材".to_string(),
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
    use squid_n_core::ids::{FloorRegionId, MaterialId, NodeId, SectionId, StoryId, WallRegionId};
    use squid_n_core::model::{SecondaryMember, SecondaryMemberKind, WallRegion};

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

    fn secondary_member(id: u32, section: Option<SectionId>) -> SecondaryMember {
        SecondaryMember {
            kind: SecondaryMemberKind::Joist,
            nodes: [squid_n_core::ids::NodeId(0), squid_n_core::ids::NodeId(1)],
            section,
            name: format!("secondary-{id}"),
        }
    }

    fn slab(id: u32, section: Option<SectionId>) -> Slab {
        Slab {
            id: squid_n_core::ids::SlabId(id),
            shape: squid_n_core::model::SlabShape::Enclosed { boundary: vec![] },
            plate: squid_n_core::model::SlabPlate {
                section,
                method: squid_n_core::model::DistributionMethod::OneWay,
                ..Default::default()
            },
        }
    }

    /// 手入力の小梁ライン（`FloorRegion::joists`）だけを持つ床領域。
    fn floor_region_with_joists(
        id: u32,
        joists: Vec<squid_n_core::model::JoistLine>,
    ) -> FloorRegion {
        let mut region = FloorRegion::new(squid_n_core::ids::FloorRegionId(id), vec![]);
        region.joists = joists;
        region
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

        let groups = section_floor_groups(&stories, &sections, &[], &[], &[]);
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

        let groups = section_floor_groups(&stories, &sections, &[], &[], &[]);
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

        let groups = section_floor_groups(&stories, &sections, &[], &[], &[]);
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

        let groups = section_floor_groups(&stories, &sections, &[], &[], &[]);
        assert_eq!(groups.len(), 1);
        assert_eq!(groups[0].0, SectionGroupKey::Floor("1F".to_string()));
        assert_eq!(groups[0].1, vec![SectionId(0)]);
    }

    #[test]
    fn secondary_member_sections_go_to_secondary_group() {
        let stories = vec![];
        let sections = vec![section(0, "A", None), section(1, "B", None)];
        let secondary_members = vec![secondary_member(0, Some(SectionId(0)))];

        let groups = section_floor_groups(&stories, &sections, &secondary_members, &[], &[]);
        assert_eq!(
            groups,
            vec![
                (SectionGroupKey::Secondary, vec![SectionId(0)]),
                (SectionGroupKey::NoFloor, vec![SectionId(1)]),
            ]
        );
    }

    #[test]
    fn slab_sections_go_to_secondary_group() {
        let stories = vec![];
        let sections = vec![section(0, "A", None), section(1, "B", None)];
        let slabs = vec![slab(0, Some(SectionId(0)))];

        let groups = section_floor_groups(&stories, &sections, &[], &slabs, &[]);
        assert_eq!(
            groups,
            vec![
                (SectionGroupKey::Secondary, vec![SectionId(0)]),
                (SectionGroupKey::NoFloor, vec![SectionId(1)]),
            ]
        );
    }

    #[test]
    fn secondary_group_comes_before_no_floor_group() {
        let stories = vec![story("1F")];
        let sections = vec![
            section(0, "A", Some("1F")),
            section(1, "B", None),
            section(2, "C", None),
        ];
        let secondary_members = vec![secondary_member(0, Some(SectionId(1)))];

        let groups = section_floor_groups(&stories, &sections, &secondary_members, &[], &[]);
        assert_eq!(
            groups,
            vec![
                (SectionGroupKey::Floor("1F".to_string()), vec![SectionId(0)]),
                (SectionGroupKey::Secondary, vec![SectionId(1)]),
                (SectionGroupKey::NoFloor, vec![SectionId(2)]),
            ]
        );
    }

    #[test]
    fn floored_secondary_sections_stay_in_floor_group() {
        let stories = vec![story("1F")];
        let sections = vec![section(0, "A", Some("1F")), section(1, "B", None)];
        let secondary_members = vec![secondary_member(0, Some(SectionId(0)))];

        let groups = section_floor_groups(&stories, &sections, &secondary_members, &[], &[]);
        assert_eq!(
            groups,
            vec![
                (SectionGroupKey::Floor("1F".to_string()), vec![SectionId(0)]),
                (SectionGroupKey::NoFloor, vec![SectionId(1)]),
            ]
        );
    }

    #[test]
    fn joist_sections_go_to_secondary_group() {
        let stories = vec![];
        let sections = vec![section(0, "A", None), section(1, "B", None)];
        let floor_regions = vec![floor_region_with_joists(
            0,
            vec![squid_n_core::model::JoistLine {
                dir: [1.0, 0.0],
                spacing: 1.0,
                support: [squid_n_core::ids::NodeId(0), squid_n_core::ids::NodeId(1)],
                section: Some(SectionId(0)),
                pinned_onto: None,
            }],
        )];

        let groups = section_floor_groups(&stories, &sections, &[], &[], &floor_regions);
        assert_eq!(
            groups,
            vec![
                (SectionGroupKey::Secondary, vec![SectionId(0)]),
                (SectionGroupKey::NoFloor, vec![SectionId(1)]),
            ]
        );
    }

    #[test]
    fn post_sections_go_to_secondary_group() {
        let stories = vec![];
        let sections = vec![section(0, "A", None), section(1, "B", None)];
        let secondary_members = vec![SecondaryMember {
            kind: SecondaryMemberKind::Post,
            nodes: [squid_n_core::ids::NodeId(0), squid_n_core::ids::NodeId(1)],
            section: Some(SectionId(0)),
            name: "post-0".to_string(),
        }];

        let groups = section_floor_groups(&stories, &sections, &secondary_members, &[], &[]);
        assert_eq!(
            groups,
            vec![
                (SectionGroupKey::Secondary, vec![SectionId(0)]),
                (SectionGroupKey::NoFloor, vec![SectionId(1)]),
            ]
        );
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

    #[test]
    fn secondary_member_nav_label_orders_node_ids() {
        let sm = SecondaryMember {
            kind: SecondaryMemberKind::Joist,
            nodes: [NodeId(5), NodeId(2)],
            section: None,
            name: String::new(),
        };
        assert_eq!(secondary_member_nav_label(&sm), "小梁 n2–n5");
    }

    #[test]
    fn floor_region_nav_rows_lists_secondary_joists() {
        let mut region = FloorRegion::new(FloorRegionId(0), vec![]);
        region.name = "A区画".to_string();
        region.secondary_joists.push(SecondaryMember {
            kind: SecondaryMemberKind::Joist,
            nodes: [NodeId(1), NodeId(2)],
            section: None,
            name: String::new(),
        });
        let rows = floor_region_nav_rows(&[region]);
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].title, "A区画");
        assert_eq!(rows[0].joist_labels, vec!["小梁 n1–n2".to_string()]);
    }

    #[test]
    fn wall_region_nav_rows_use_default_title() {
        let mut region = WallRegion::new(WallRegionId(3), vec![]);
        region.posts.push(SecondaryMember {
            kind: SecondaryMemberKind::Post,
            nodes: [NodeId(4), NodeId(7)],
            section: None,
            name: String::new(),
        });
        let rows = wall_region_nav_rows(&[region]);
        assert_eq!(rows[0].title, "壁領域 3");
        assert_eq!(rows[0].post_labels, vec!["間柱 n4–n7".to_string()]);
    }

    #[test]
    fn unassigned_nav_rows_lists_joists_and_posts() {
        let mut model = Model::default();
        model.unassigned_joists.push(SecondaryMember {
            kind: SecondaryMemberKind::Joist,
            nodes: [NodeId(0), NodeId(1)],
            section: None,
            name: String::new(),
        });
        model.unassigned_posts.push(SecondaryMember {
            kind: SecondaryMemberKind::Post,
            nodes: [NodeId(2), NodeId(3)],
            section: None,
            name: String::new(),
        });
        let rows = unassigned_nav_rows(&model);
        assert_eq!(rows.joist_labels, vec!["小梁 n0–n1".to_string()]);
        assert_eq!(rows.post_labels, vec!["間柱 n2–n3".to_string()]);
    }

    #[test]
    fn region_nav_rows_empty_model() {
        let model = Model::default();
        assert!(floor_region_nav_rows(&model.floor_regions).is_empty());
        assert!(wall_region_nav_rows(&model.wall_regions).is_empty());
        let unassigned = unassigned_nav_rows(&model);
        assert!(unassigned.joist_labels.is_empty());
        assert!(unassigned.post_labels.is_empty());
    }
}
