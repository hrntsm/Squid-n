//! 二次部材（小梁・間柱）の配置フォームと一覧。床タブ・壁版タブで共用する。

use crate::app::App;
use crate::table_util::{self, Col};
use squid_n_core::ids::*;
use squid_n_core::model::{
    ElementKind, EndSupport, Model, SecondaryMember, SecondaryMemberAnchor, SecondaryMemberEnds,
    SecondaryMemberKind, SupportMemberId,
};
use squid_n_edit::{
    DeleteSecondaryMember, PlaceSecondaryMember, SecondaryParent, SetSecondaryMemberEndSupport,
};

/// 二次部材の配置フォームのドラフト（GUI 専用）。
///
/// 親領域は小梁なら床領域、間柱なら壁領域。`unassigned` は未割当へ置く。
#[derive(Clone, Debug)]
pub struct SecondaryMemberDraft {
    pub unassigned: bool,
    /// 親領域 ID の生値（小梁は床領域、間柱は壁領域）。未選択は `None`。
    pub parent: Option<u32>,
    pub support_a: Option<SupportMemberId>,
    /// 端 A の材軸位置（0..1 の文字列）。
    pub pos_a: String,
    pub support_b: Option<SupportMemberId>,
    /// 端 B の材軸位置（0..1 の文字列）。
    pub pos_b: String,
    /// 端 B を自由端（片持ち）にするか。
    pub free_end: bool,
    /// 片持ちの自由端ベクトル。小梁は `[dx, dy]`、間柱は `[構面内 s, z]` [mm]。
    pub free_vector: [String; 2],
    pub section: Option<SectionId>,
    pub name: String,
}

impl Default for SecondaryMemberDraft {
    fn default() -> Self {
        Self {
            unassigned: false,
            parent: None,
            support_a: None,
            pos_a: "0.5".to_string(),
            support_b: None,
            pos_b: "0.5".to_string(),
            free_end: false,
            free_vector: ["0".to_string(), "0".to_string()],
            section: None,
            name: String::new(),
        }
    }
}

/// 端部支持条件の表示・選択で、実際の `ends` と候補が同じ状態を表すか。
///
/// モデルは片持ちを支持端アンカーと自由端ベクトルで表すため、`[Supported, Free]` と
/// `[Free, Supported]` はどちらも `Cantilever` へ正規化される。両者を同じものとして
/// 一致判定する。支持未解決の `Detached` はいずれの組合せにも一致させない。
fn ends_matches(ends: &SecondaryMemberEnds, candidate: [EndSupport; 2]) -> bool {
    let supported = [
        candidate[0] == EndSupport::Supported,
        candidate[1] == EndSupport::Supported,
    ];
    match ends {
        SecondaryMemberEnds::Supported(_) => supported == [true, true],
        SecondaryMemberEnds::Cantilever { .. } => {
            supported == [true, false] || supported == [false, true]
        }
        SecondaryMemberEnds::Detached(_) => false,
    }
}

fn ends_support_label(ends: &SecondaryMemberEnds) -> &'static str {
    match ends {
        SecondaryMemberEnds::Supported(_) => "支持-支持",
        SecondaryMemberEnds::Cantilever { .. } => "支持-自由（片持ち）",
        SecondaryMemberEnds::Detached(_) => "未解決",
    }
}

fn end_support_label(end_support: &[EndSupport; 2]) -> &'static str {
    match end_support {
        [EndSupport::Supported, EndSupport::Supported] => "支持-支持",
        [EndSupport::Supported, EndSupport::Free] => "支持-自由（片持ち）",
        [EndSupport::Free, EndSupport::Supported] => "自由-支持（片持ち）",
        [EndSupport::Free, EndSupport::Free] => "自由-自由",
    }
}

pub(crate) fn support_label(model: &Model, support: SupportMemberId) -> String {
    match support {
        SupportMemberId::Primary(id) => {
            let kind = model
                .element(id)
                .map(|e| {
                    if e.kind == ElementKind::Beam {
                        "大梁"
                    } else {
                        "部材"
                    }
                })
                .unwrap_or("部材");
            format!("{kind} E{}", id.0)
        }
        SupportMemberId::Secondary(id) => {
            let kind = model
                .secondary_member(id)
                .map(|sm| match sm.kind {
                    SecondaryMemberKind::Joist => "小梁",
                    SecondaryMemberKind::Post => "間柱",
                })
                .unwrap_or("二次部材");
            format!("{kind} SM{}", id.0)
        }
    }
}

/// 配置先に選べる支持部材。主架構の 2 節点梁と、同じ種別の既存二次部材のうち、
/// 材軸の両端が親領域（作業範囲）の内側または境界上にあるもの。
fn support_candidates(
    model: &Model,
    kind: SecondaryMemberKind,
    parent: SecondaryParent,
) -> Vec<SupportMemberId> {
    let mut out = Vec::new();
    for e in &model.elements {
        if e.kind == ElementKind::Beam && e.nodes.len() == 2 {
            out.push(SupportMemberId::Primary(e.id));
        }
    }
    match kind {
        SecondaryMemberKind::Joist => {
            out.extend(model.joists().map(|sm| SupportMemberId::Secondary(sm.id)));
        }
        SecondaryMemberKind::Post => {
            out.extend(model.posts().map(|sm| SupportMemberId::Secondary(sm.id)));
        }
    }
    out.retain(|support| support_axis_in_parent(model, *support, parent));
    out
}

/// 支持部材の材軸が親領域の内側または境界上にあるか。材軸を解決できない場合は `false`。
fn support_axis_in_parent(
    model: &Model,
    support: SupportMemberId,
    parent: SecondaryParent,
) -> bool {
    let Some((a, b)) = model.support_member_axis(support) else {
        return false;
    };
    [a, b].iter().all(|p| parent_contains(model, parent, *p))
}

/// 点 `p` [mm] が親領域（床領域・壁領域）の内側または境界上にあるか。
fn parent_contains(model: &Model, parent: SecondaryParent, p: [f64; 3]) -> bool {
    match parent {
        SecondaryParent::Floor(id) => model.floor_region_contains_point_including_boundary(id, p),
        SecondaryParent::Wall(id) => model.wall_region_contains_point_including_boundary(id, p),
        SecondaryParent::Unassigned => true,
    }
}

fn parse_position(text: &str) -> Option<f64> {
    let value = text.trim().parse::<f64>().ok()?;
    (value.is_finite() && (0.0..=1.0).contains(&value)).then_some(value)
}

/// フォームの入力から取付き位置表現を組み立てる。入力不足は `None`。
fn build_ends(draft: &SecondaryMemberDraft) -> Option<SecondaryMemberEnds> {
    let a = SecondaryMemberAnchor {
        support: draft.support_a?,
        position: parse_position(&draft.pos_a)?,
    };
    if draft.free_end {
        let vector = [
            draft.free_vector[0].trim().parse::<f64>().ok()?,
            draft.free_vector[1].trim().parse::<f64>().ok()?,
        ];
        if !vector.iter().all(|v| v.is_finite()) || vector == [0.0, 0.0] {
            return None;
        }
        Some(SecondaryMemberEnds::Cantilever {
            support: a,
            free_end_vector: vector,
        })
    } else {
        let b = SecondaryMemberAnchor {
            support: draft.support_b?,
            position: parse_position(&draft.pos_b)?,
        };
        (a != b).then_some(SecondaryMemberEnds::Supported([a, b]))
    }
}

fn parent_exists(model: &Model, kind: SecondaryMemberKind, raw: u32) -> bool {
    match kind {
        SecondaryMemberKind::Joist => model
            .floor_regions
            .get(raw as usize)
            .is_some_and(|r| r.id == FloorRegionId(raw)),
        SecondaryMemberKind::Post => model
            .wall_regions
            .get(raw as usize)
            .is_some_and(|r| r.id == WallRegionId(raw)),
    }
}

fn parent_options(app: &App, kind: SecondaryMemberKind) -> Vec<(u32, String)> {
    match kind {
        SecondaryMemberKind::Joist => app
            .core
            .model
            .floor_regions
            .iter()
            .map(|r| {
                let label = if r.name.is_empty() {
                    format!("床領域 #{}", r.id.0)
                } else {
                    r.name.clone()
                };
                (r.id.0, label)
            })
            .collect(),
        SecondaryMemberKind::Post => app
            .core
            .model
            .wall_regions
            .iter()
            .map(|r| {
                let label = if r.name.is_empty() {
                    format!("壁領域 #{}", r.id.0)
                } else {
                    r.name.clone()
                };
                (r.id.0, label)
            })
            .collect(),
    }
}

fn support_combo(
    ui: &mut egui::Ui,
    id_salt: impl std::hash::Hash,
    selected: &mut Option<SupportMemberId>,
    candidates: &[SupportMemberId],
    model: &Model,
) {
    let current = selected
        .map(|s| support_label(model, s))
        .unwrap_or_else(|| "―".to_string());
    egui::ComboBox::from_id_salt(id_salt)
        .selected_text(current)
        .show_ui(ui, |ui| {
            for candidate in candidates {
                if ui
                    .selectable_label(
                        *selected == Some(*candidate),
                        support_label(model, *candidate),
                    )
                    .clicked()
                {
                    *selected = Some(*candidate);
                }
            }
        });
}

/// 二次部材の配置フォーム。親領域（作業範囲）と両端の支持部材・材軸位置を指定する。
pub(crate) fn secondary_member_placement_form(
    app: &mut App,
    ui: &mut egui::Ui,
    kind: SecondaryMemberKind,
) {
    let title = match kind {
        SecondaryMemberKind::Joist => "小梁を配置",
        SecondaryMemberKind::Post => "間柱を配置",
    };
    ui.strong(title);
    ui.label(
        "親領域（小梁は床領域、間柱は壁領域）と、両端が取り付く支持部材・材軸位置\
         （0〜1）を指定します。端 B を「自由端」にすると片持ちになります。支持が\
         決まらない端は配置できません（片持ちへの読み替えはしません）。",
    );

    let parents = parent_options(app, kind);
    {
        let draft = &mut app.ui.scoped.secondary_draft;
        if !draft.unassigned
            && !draft
                .parent
                .is_some_and(|id| parents.iter().any(|(pid, _)| *pid == id))
        {
            draft.parent = parents.first().map(|(id, _)| *id);
        }
    }
    let parent = {
        let draft = &app.ui.scoped.secondary_draft;
        if draft.unassigned {
            SecondaryParent::Unassigned
        } else {
            match (kind, draft.parent) {
                (SecondaryMemberKind::Joist, Some(id)) => SecondaryParent::Floor(FloorRegionId(id)),
                (SecondaryMemberKind::Post, Some(id)) => SecondaryParent::Wall(WallRegionId(id)),
                _ => SecondaryParent::Unassigned,
            }
        }
    };
    let candidates = support_candidates(&app.core.model, kind, parent);
    {
        let draft = &mut app.ui.scoped.secondary_draft;
        if !draft.support_a.is_some_and(|s| candidates.contains(&s)) {
            draft.support_a = candidates.first().copied();
        }
        if !draft.free_end && !draft.support_b.is_some_and(|s| candidates.contains(&s)) {
            draft.support_b = candidates.get(1).or(candidates.first()).copied();
        }
    }

    let selected_parent_text = {
        let draft = &app.ui.scoped.secondary_draft;
        match (draft.unassigned, draft.parent) {
            (true, _) => "未割当".to_string(),
            (false, Some(id)) => parents
                .iter()
                .find(|(pid, _)| *pid == id)
                .map(|(_, label)| label.clone())
                .unwrap_or_else(|| format!("#{id}")),
            (false, None) => "―".to_string(),
        }
    };
    let mut parent_changed: Option<Option<u32>> = None;
    ui.horizontal(|ui| {
        ui.label("親領域:");
        egui::ComboBox::from_id_salt(("secondary_parent", kind as u8))
            .selected_text(selected_parent_text)
            .show_ui(ui, |ui| {
                if ui
                    .selectable_label(app.ui.scoped.secondary_draft.unassigned, "未割当")
                    .clicked()
                {
                    parent_changed = Some(None);
                }
                for (id, label) in &parents {
                    let selected = !app.ui.scoped.secondary_draft.unassigned
                        && app.ui.scoped.secondary_draft.parent == Some(*id);
                    if ui.selectable_label(selected, label).clicked() {
                        parent_changed = Some(Some(*id));
                    }
                }
            });
    });
    if let Some(choice) = parent_changed {
        let draft = &mut app.ui.scoped.secondary_draft;
        draft.unassigned = choice.is_none();
        if choice.is_some() {
            draft.parent = choice;
        }
    }

    ui.horizontal(|ui| {
        ui.label("端 A:");
        support_combo(
            ui,
            ("secondary_support_a", kind as u8),
            &mut app.ui.scoped.secondary_draft.support_a,
            &candidates,
            &app.core.model,
        );
        ui.label("材軸位置:");
        ui.add(
            egui::TextEdit::singleline(&mut app.ui.scoped.secondary_draft.pos_a)
                .desired_width(50.0),
        );
    });
    ui.horizontal(|ui| {
        ui.label("端 B:");
        if app.ui.scoped.secondary_draft.free_end {
            ui.label("自由端");
            ui.label("ベクトル:");
            ui.add(
                egui::TextEdit::singleline(&mut app.ui.scoped.secondary_draft.free_vector[0])
                    .desired_width(60.0),
            );
            ui.add(
                egui::TextEdit::singleline(&mut app.ui.scoped.secondary_draft.free_vector[1])
                    .desired_width(60.0),
            );
            ui.label("[mm]");
        } else {
            support_combo(
                ui,
                ("secondary_support_b", kind as u8),
                &mut app.ui.scoped.secondary_draft.support_b,
                &candidates,
                &app.core.model,
            );
            ui.label("材軸位置:");
            ui.add(
                egui::TextEdit::singleline(&mut app.ui.scoped.secondary_draft.pos_b)
                    .desired_width(50.0),
            );
        }
    });
    ui.horizontal(|ui| {
        ui.checkbox(
            &mut app.ui.scoped.secondary_draft.free_end,
            "端 B を自由端（片持ち）にする",
        );
        if app.ui.scoped.secondary_draft.free_end {
            let hint = match kind {
                SecondaryMemberKind::Joist => "ベクトル [dx, dy] [mm]",
                SecondaryMemberKind::Post => "ベクトル [構面内 s, z] [mm]",
            };
            ui.label(hint);
        }
    });

    ui.horizontal(|ui| {
        ui.label("断面:");
        let label = app
            .ui
            .scoped
            .secondary_draft
            .section
            .and_then(|sid| app.core.model.sections.get(sid.index()))
            .map(|sec| sec.display_name())
            .unwrap_or_else(|| "―".to_string());
        egui::ComboBox::from_id_salt(("secondary_section", kind as u8))
            .selected_text(label)
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut app.ui.scoped.secondary_draft.section, None, "―");
                for sec in &app.core.model.sections {
                    ui.selectable_value(
                        &mut app.ui.scoped.secondary_draft.section,
                        Some(sec.id),
                        sec.display_name(),
                    );
                }
            });
        ui.label("名前:");
        ui.add(
            egui::TextEdit::singleline(&mut app.ui.scoped.secondary_draft.name).desired_width(80.0),
        );
    });

    let draft = app.ui.scoped.secondary_draft.clone();
    let can_place = build_ends(&draft).is_some()
        && (draft.unassigned
            || draft
                .parent
                .is_some_and(|id| parent_exists(&app.core.model, kind, id)));
    if ui
        .add_enabled(can_place, egui::Button::new("配置"))
        .on_hover_text("両端の支持部材と材軸位置が解決できるときだけ配置できます")
        .clicked()
    {
        let ends = build_ends(&draft).expect("can_place で確認済み");
        let parent = if draft.unassigned {
            SecondaryParent::Unassigned
        } else {
            match kind {
                SecondaryMemberKind::Joist => {
                    SecondaryParent::Floor(FloorRegionId(draft.parent.expect("確認済み")))
                }
                SecondaryMemberKind::Post => {
                    SecondaryParent::Wall(WallRegionId(draft.parent.expect("確認済み")))
                }
            }
        };
        let applied = app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(PlaceSecondaryMember {
                parent,
                kind,
                ends,
                section: draft.section,
                name: draft.name.clone(),
            }),
        );
        if applied {
            app.core.scoped.staleness.mark_edited();
        } else {
            app.core.scoped.last_notice = Some(
                "支持端が親領域の内側・境界上にないため配置しませんでした。\
                 親領域の内側で支持部材を選んでください。"
                    .to_string(),
            );
        }
    }
}

/// 二次部材の一覧。所属・ID・断面・支持条件の変更・削除。
pub(crate) fn secondary_member_list(app: &mut App, ui: &mut egui::Ui, kind: SecondaryMemberKind) {
    let members: Vec<SecondaryMember> = match kind {
        SecondaryMemberKind::Joist => app
            .core
            .model
            .joists()
            .filter(|sm| !app.core.model.secondary_member_materialized(sm))
            .cloned()
            .collect(),
        SecondaryMemberKind::Post => app
            .core
            .model
            .posts()
            .filter(|sm| !app.core.model.secondary_member_materialized(sm))
            .cloned()
            .collect(),
    };
    let mut pending_end_support: Vec<(SecondaryMemberId, [EndSupport; 2])> = Vec::new();
    let mut pending_delete: Vec<SecondaryMemberId> = Vec::new();
    let owner_of = |app: &App, id: SecondaryMemberId| -> Option<String> {
        match kind {
            SecondaryMemberKind::Joist => app
                .core
                .model
                .floor_regions
                .iter()
                .find(|r| r.secondary_joists.iter().any(|j| j.id == id))
                .map(|r| {
                    if r.name.is_empty() {
                        format!("#{}", r.id.0)
                    } else {
                        r.name.clone()
                    }
                }),
            SecondaryMemberKind::Post => app
                .core
                .model
                .wall_regions
                .iter()
                .find(|r| r.posts.iter().any(|p| p.id == id))
                .map(|r| {
                    if r.name.is_empty() {
                        format!("#{}", r.id.0)
                    } else {
                        r.name.clone()
                    }
                }),
        }
    };
    let table_id = match kind {
        SecondaryMemberKind::Joist => "secondary_joists_tbl",
        SecondaryMemberKind::Post => "secondary_posts_tbl",
    };
    table_util::standard_table(
        ui,
        table_id,
        &[
            Col::text("所属"),
            Col::text("ID"),
            Col::text("断面"),
            Col::name("支持条件"),
            Col::text("操作"),
        ],
        members.len(),
        |row| {
            let i = row.index();
            let sm = &members[i];
            row.col(|ui| match owner_of(app, sm.id) {
                Some(name) => table_util::text_cell(ui, &name),
                None => table_util::muted_cell(ui, "未割当", "どの領域にも所属していません"),
            });
            row.col(|ui| {
                table_util::text_cell(ui, &format!("SM{}", sm.id.0));
            });
            row.col(|ui| {
                let label = sm
                    .section
                    .and_then(|sid| app.core.model.sections.get(sid.index()))
                    .map(|sec| sec.display_name())
                    .unwrap_or_else(|| "―".to_string());
                table_util::text_cell(ui, &label);
            });
            row.col(|ui| {
                table_util::cell_combo(
                    ui,
                    (table_id, sm.id.0),
                    ends_support_label(&sm.ends),
                    |ui| {
                        for candidate in [
                            [EndSupport::Supported, EndSupport::Supported],
                            [EndSupport::Supported, EndSupport::Free],
                            [EndSupport::Free, EndSupport::Supported],
                        ] {
                            if ui
                                .selectable_label(
                                    ends_matches(&sm.ends, candidate),
                                    end_support_label(&candidate),
                                )
                                .clicked()
                                && !ends_matches(&sm.ends, candidate)
                            {
                                pending_end_support.push((sm.id, candidate));
                            }
                        }
                    },
                );
            });
            row.col(|ui| {
                if ui.button("削除").clicked() {
                    pending_delete.push(sm.id);
                }
            });
        },
    );
    let mut edited = false;
    for (member, end_support) in pending_end_support {
        edited |= app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(SetSecondaryMemberEndSupport {
                member,
                end_support,
            }),
        );
    }
    for member in pending_delete {
        edited |= app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(DeleteSecondaryMember { member }),
        );
    }
    if edited {
        app.core.scoped.staleness.mark_edited();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn anchor() -> SecondaryMemberAnchor {
        SecondaryMemberAnchor {
            support: SupportMemberId::Primary(ElemId(0)),
            position: 0.5,
        }
    }

    #[test]
    fn 支持未解決はどの端部候補にも一致しない() {
        let ends = SecondaryMemberEnds::Detached([[0.0, 0.0, 0.0], [1000.0, 0.0, 0.0]]);
        for candidate in [
            [EndSupport::Supported, EndSupport::Supported],
            [EndSupport::Supported, EndSupport::Free],
            [EndSupport::Free, EndSupport::Supported],
        ] {
            assert!(!ends_matches(&ends, candidate), "{candidate:?}");
        }
    }

    #[test]
    fn 片持ちは正規化後の組合せの両方に一致する() {
        let ends = SecondaryMemberEnds::Cantilever {
            support: anchor(),
            free_end_vector: [0.0, 1000.0],
        };
        assert!(ends_matches(
            &ends,
            [EndSupport::Supported, EndSupport::Free]
        ));
        assert!(ends_matches(
            &ends,
            [EndSupport::Free, EndSupport::Supported]
        ));
        assert!(!ends_matches(
            &ends,
            [EndSupport::Supported, EndSupport::Supported]
        ));
    }

    #[test]
    fn 両端支持は支持支持にのみ一致する() {
        let ends = SecondaryMemberEnds::Supported([anchor(), anchor()]);
        assert!(ends_matches(
            &ends,
            [EndSupport::Supported, EndSupport::Supported]
        ));
        assert!(!ends_matches(
            &ends,
            [EndSupport::Supported, EndSupport::Free]
        ));
        assert!(!ends_matches(
            &ends,
            [EndSupport::Free, EndSupport::Supported]
        ));
    }

    #[test]
    fn 支持部材候補は親領域内の材軸に絞る() {
        use squid_n_core::model::{
            ElementData, EndCondition, FloorRegion, ForceRegime, LocalAxis, Node,
        };

        fn beam(id: u32, a: u32, b: u32) -> ElementData {
            ElementData {
                id: ElemId(id),
                kind: ElementKind::Beam,
                nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
                section: None,
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            }
        }

        let mut model = Model::default();
        for (i, (x, y)) in [(0.0, 0.0), (4000.0, 0.0), (4000.0, 4000.0), (0.0, 4000.0)]
            .into_iter()
            .enumerate()
        {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: [x, y, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            });
        }
        for (i, (a, b)) in [(0u32, 1u32), (1, 2), (2, 3), (3, 0)]
            .into_iter()
            .enumerate()
        {
            model.elements.push(beam(i as u32, a, b));
        }
        model.nodes.push(Node {
            id: NodeId(4),
            coord: [-4000.0, 0.0, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
        model.nodes.push(Node {
            id: NodeId(5),
            coord: [-4000.0, 4000.0, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
        model.elements.push(beam(4, 4, 5));
        model.floor_regions.push(FloorRegion::new(
            FloorRegionId(0),
            vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        ));

        let parent = SecondaryParent::Floor(FloorRegionId(0));
        let candidates = support_candidates(&model, SecondaryMemberKind::Joist, parent);
        assert!(candidates.contains(&SupportMemberId::Primary(ElemId(0))));
        assert!(
            !candidates.contains(&SupportMemberId::Primary(ElemId(4))),
            "親領域の外にある大梁は候補にしない"
        );
    }
}
