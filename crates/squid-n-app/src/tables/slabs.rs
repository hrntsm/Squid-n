use crate::app::App;
use squid_n_core::ids::{FloorRegionId, NodeId};
use squid_n_core::model::{AreaLoad, DistributionMethod, JoistLine, OneWayDir, SlabUsage};
use squid_n_core::model::{RegionAnchor, RegionShape};
use squid_n_core::units::to_display::area_load_kn_per_m2;
use squid_n_core::units::to_internal;
use squid_n_edit::{
    AddSlab, DeleteSlab, SetAttachedAnchor, SetAttachedExtent, SetFloorRegionName, SetSlabJoists,
    SetSlabOneWay, SetSlabUsage,
};

/// スラブ追加フォームのドラフト状態（GUI 専用）。
/// `nodes` は境界4節点（頂点0→1→2→3→0 の順で外周を辿る）の選択状態。
#[derive(Clone, Debug)]
pub struct SlabDraft {
    /// 境界節点スロット（外周順。3〜N 個、可変長）。
    pub nodes: Vec<Option<NodeId>>,
    /// 荷重種別（既定 "DL"）
    pub load_kind: String,
    /// 荷重値の入力文字列。**UI 表示は kN/m²**（内部は `to_internal::area_load_kn_per_m2`）。
    pub load_value: String,
    pub method: DistributionMethod,
    /// スラブ用途（積載荷重プリセット。`None` は積載寄与なし）。
    pub usage: Option<SlabUsage>,
    /// スラブ断面（板厚・コンクリート材料を持つ断面。`None` は未割当）。
    pub section: Option<squid_n_core::ids::SectionId>,
    /// 小梁入力の対象スラブ（小梁編集セクション用）。
    pub joist_target: Option<FloorRegionId>,
    /// 小梁の支持節点（両端。小梁が架かる2節点）。
    pub joist_supports: [Option<NodeId>; 2],
    /// 小梁の負担幅 spacing の入力文字列（**UI 表示は mm**、内部も mm）。
    pub joist_spacing: String,
    /// 小梁の断面（床の中での小梁設計用。`None` は断面未割当）。
    pub joist_section: Option<squid_n_core::ids::SectionId>,
    /// 取り付き領域の名前。
    pub attached_name: String,
    /// 取り付き領域の取付き先の節点（線なら両端、点なら 1 つ目だけを使う）。
    pub attached_nodes: [Option<NodeId>; 2],
    /// 取り付き先を点（柱）にするか。false は線（取付き線）。
    pub attached_point: bool,
    /// 張り出し量の入力文字列 [mm]（線: 始端側・終端側、点: X 方向・Y 方向）。
    pub attached_extent: [String; 2],
    /// 取付き線に載る領域の荷重の出口。
    pub attached_transfer: squid_n_core::model::LoadTransfer,
}

impl Default for SlabDraft {
    fn default() -> Self {
        Self {
            nodes: vec![None; 4],
            load_kind: "DL".to_string(),
            load_value: "0".to_string(),
            method: DistributionMethod::TriTrapezoid,
            usage: None,
            section: None,
            joist_target: None,
            joist_supports: [None; 2],
            joist_spacing: "0".to_string(),
            joist_section: None,
            attached_name: String::new(),
            attached_nodes: [None; 2],
            attached_point: false,
            attached_extent: ["1000".to_string(), "1000".to_string()],
            attached_transfer: squid_n_core::model::LoadTransfer::Anchor,
        }
    }
}

/// 用途選択で提示するプリセット（令別表第1／国交省営繕基準・令和3年度版）。
/// `None` は「なし（積載寄与なし）」。`Custom` は UI からは扱わない
/// （モデル/シリアライズでは利用可）。並びは国交省営繕基準の表に概ね沿う。
const USAGE_PRESETS: &[Option<SlabUsage>] = &[
    None,
    Some(SlabUsage::Residential),
    Some(SlabUsage::Office),
    Some(SlabUsage::ResearchRoom),
    Some(SlabUsage::Classroom),
    Some(SlabUsage::Store),
    Some(SlabUsage::AssemblyFixed),
    Some(SlabUsage::AssemblyOther),
    Some(SlabUsage::Corridor),
    Some(SlabUsage::RegistryArchive),
    Some(SlabUsage::GeneralArchive),
    Some(SlabUsage::MobileArchive),
    Some(SlabUsage::LabChemistry),
    Some(SlabUsage::LabPhysics),
    Some(SlabUsage::ComputerRoom),
    Some(SlabUsage::MachineRoom),
    Some(SlabUsage::Gymnasium),
    Some(SlabUsage::Garage),
    Some(SlabUsage::Balcony),
    Some(SlabUsage::RoofResidential),
    Some(SlabUsage::RoofStore),
    Some(SlabUsage::RoofUnused),
    Some(SlabUsage::RoofSteelGym),
];

fn usage_label(u: Option<SlabUsage>) -> &'static str {
    match u {
        None => "なし",
        Some(SlabUsage::Residential) => "住宅の居室・寝室・病室",
        Some(SlabUsage::Office) => "事務室・会議室・食堂",
        Some(SlabUsage::ResearchRoom) => "研究室",
        Some(SlabUsage::Classroom) => "教室",
        Some(SlabUsage::Store) => "百貨店・店舗の売場",
        Some(SlabUsage::AssemblyFixed) => "集会室・客席（固定席）",
        Some(SlabUsage::AssemblyOther) => "集会室・客席（その他）",
        Some(SlabUsage::Corridor) => "廊下・玄関・階段",
        Some(SlabUsage::RegistryArchive) => "法務局登記書庫",
        Some(SlabUsage::GeneralArchive) => "一般書庫・倉庫等",
        Some(SlabUsage::MobileArchive) => "移動書架書庫・電算室空調機室・用具庫等",
        Some(SlabUsage::LabChemistry) => "一般実験室（化学系）",
        Some(SlabUsage::LabPhysics) => "一般実験室（物理系）",
        Some(SlabUsage::ComputerRoom) => "電算室",
        Some(SlabUsage::MachineRoom) => "機械室",
        Some(SlabUsage::Gymnasium) => "体育館・武道場等",
        Some(SlabUsage::Garage) => "自動車車庫・通路",
        Some(SlabUsage::Balcony) => "片持バルコニー・庇等",
        Some(SlabUsage::RoofResidential) => "屋上（学校・百貨店の類を除く）",
        Some(SlabUsage::RoofStore) => "屋上（学校・百貨店の類）",
        Some(SlabUsage::RoofUnused) => "屋上（通常人が使用しない）",
        Some(SlabUsage::RoofSteelGym) => "屋上（鉄骨造体育館・武道場等／短期）",
        Some(SlabUsage::Custom { .. }) => "任意入力",
    }
}

fn method_label(m: DistributionMethod) -> &'static str {
    match m {
        DistributionMethod::TriTrapezoid => "三角/台形(45°法)",
        DistributionMethod::OneWay => "一方向",
        DistributionMethod::TributaryArea => "負担面積",
    }
}

fn kind_label(region: &squid_n_core::model::FloorRegion) -> &'static str {
    if region.is_attached() {
        "取り付き"
    } else {
        "囲まれ"
    }
}

fn one_way_label(o: Option<OneWayDir>) -> &'static str {
    match o {
        None => "なし",
        Some(OneWayDir::X) => "X",
        Some(OneWayDir::Y) => "Y",
    }
}

pub fn slabs_table(ui: &mut egui::Ui, app: &mut App) {
    use crate::table_util::{self, Col};

    ui.label(
        "床領域は、主架構が囲む閉領域（境界節点）と、主架構に取り付く領域（取付き線または点と張り出し量）の 2 種です。版（スラブ断面）は任意です。版がなければ床荷重・スラブ検定・協力幅は生じません（結果タブ/モデルタブの3Dビューで表示モード「CMQ図」を選ぶと分配結果を確認できます）。",
    );
    ui.separator();

    // ── 一覧表 ──────────────────────────────────────────
    let n = app.model.floor_regions.len();
    let mut pending_delete: Option<FloorRegionId> = None;
    let mut pending_name: Vec<(FloorRegionId, String)> = Vec::new();
    let mut pending_one_way: Vec<(FloorRegionId, Option<OneWayDir>)> = Vec::new();
    let mut pending_usage: Vec<(FloorRegionId, Option<SlabUsage>)> = Vec::new();
    let mut pending_section: Vec<(FloorRegionId, Option<squid_n_core::ids::SectionId>)> =
        Vec::new();
    let mut pending_extent: Vec<(FloorRegionId, [f64; 2])> = Vec::new();
    let mut pending_anchor: Vec<(FloorRegionId, RegionAnchor)> = Vec::new();
    let node_ids: Vec<NodeId> = app.model.nodes.iter().map(|n| n.id).collect();
    // 板状の断面（板厚を持つ断面）だけを候補にする。板厚が無い断面を割り当てても
    // 自重・数量が算定できないため、選ばせない。
    let slab_sections: Vec<(squid_n_core::ids::SectionId, String)> = app
        .model
        .sections
        .iter()
        .filter(|sec| sec.thickness.is_some_and(|t| t > 0.0))
        .map(|sec| (sec.id, sec.display_name()))
        .collect();

    table_util::standard_table(
        ui,
        "slabs_tbl",
        &[
            Col::id(),
            Col::name("名前"),
            Col::text("境界節点"),
            Col::text("荷重"),
            Col::name("分配法"),
            Col::name("種別"),
            Col::name("一方向"),
            Col::text("用途"),
            Col::text("断面"),
            Col::label("小梁"),
            Col::actions(),
        ],
        n,
        |row| {
            let i = row.index();
            let slab = &app.model.floor_regions[i];
            row.col(|ui| {
                table_util::id_label(ui, slab.id.0);
            });
            row.col(|ui| {
                // 名前は領域を指し示すための表示用（「階段室」「吹抜け」など）。
                // 空欄のままでも構わないので、未設定は淡色のプレースホルダにする。
                let mut name = slab.name.clone();
                let resp = table_util::cell_text_edit(ui, &mut name);
                if resp.changed() {
                    pending_name.push((slab.id, name));
                }
            });
            row.col(|ui| match &slab.shape {
                RegionShape::Enclosed { boundary } => {
                    let s = boundary
                        .iter()
                        .map(|n| n.0.to_string())
                        .collect::<Vec<_>>()
                        .join("-");
                    table_util::text_cell(ui, &s);
                }
                RegionShape::Attached { anchor, extent } => {
                    attached_boundary_cell(
                        ui,
                        slab.id,
                        *anchor,
                        *extent,
                        &node_ids,
                        &mut pending_extent,
                        &mut pending_anchor,
                    );
                }
            });
            row.col(|ui| {
                let s = slab
                    .plate
                    .as_ref()
                    .map(|p| p.loads.as_slice())
                    .unwrap_or_default()
                    .iter()
                    .map(|l| format!("{} {:.2}kN/m²", l.kind, area_load_kn_per_m2(l.value)))
                    .collect::<Vec<_>>()
                    .join(", ");
                if s.is_empty() {
                    table_util::muted_cell(ui, "―", "床荷重が登録されていません");
                } else {
                    table_util::text_cell(ui, &s);
                }
            });
            row.col(|ui| {
                table_util::text_cell(ui, method_label(slab.method()));
            });
            row.col(|ui| {
                // 領域の種別（囲まれた領域か取り付き領域か）は形そのものなので、
                // 表からは変更しない（作図・取り込みで決まる）。
                table_util::text_cell(ui, kind_label(slab));
            });
            row.col(|ui| {
                table_util::cell_combo(
                    ui,
                    ("slab_one_way", slab.id.0),
                    one_way_label(slab.one_way()),
                    |ui| {
                        for ow in [None, Some(OneWayDir::X), Some(OneWayDir::Y)] {
                            if ui
                                .selectable_label(slab.one_way() == ow, one_way_label(ow))
                                .clicked()
                                && slab.one_way() != ow
                            {
                                pending_one_way.push((slab.id, ow));
                            }
                        }
                    },
                );
            });
            row.col(|ui| {
                table_util::cell_combo(
                    ui,
                    ("slab_usage", slab.id.0),
                    usage_label(slab.usage()),
                    |ui| {
                        for &u in USAGE_PRESETS {
                            if ui
                                .selectable_label(slab.usage() == u, usage_label(u))
                                .clicked()
                                && slab.usage() != u
                            {
                                pending_usage.push((slab.id, u));
                            }
                        }
                    },
                );
            });
            row.col(|ui| {
                let label = app
                    .model
                    .region_section(slab)
                    .map(|sec| sec.display_name())
                    .unwrap_or_else(|| "―".to_string());
                table_util::cell_combo(ui, ("slab_section", slab.id.0), &label, |ui| {
                    if ui.selectable_label(slab.section().is_none(), "―").clicked()
                        && slab.section().is_some()
                    {
                        pending_section.push((slab.id, None));
                    }
                    for (sid, name) in &slab_sections {
                        if ui
                            .selectable_label(slab.section() == Some(*sid), name)
                            .clicked()
                            && slab.section() != Some(*sid)
                        {
                            pending_section.push((slab.id, Some(*sid)));
                        }
                    }
                });
            });
            row.col(|ui| {
                let cnt = slab.joist_lines().len();
                if cnt == 0 {
                    table_util::muted_cell(ui, "―", "小梁が配置されていません");
                } else {
                    ui.label(format!("{cnt}本"));
                }
            });
            row.col(|ui| {
                if table_util::delete_cell(ui, "このスラブを削除", None) {
                    pending_delete = Some(slab.id);
                }
            });
        },
    );

    let had_pending = !pending_name.is_empty()
        || !pending_one_way.is_empty()
        || !pending_usage.is_empty()
        || !pending_section.is_empty()
        || !pending_extent.is_empty()
        || !pending_anchor.is_empty()
        || pending_delete.is_some();
    for (id, name) in pending_name {
        app.undo
            .run(&mut app.model, Box::new(SetFloorRegionName { id, name }));
    }
    for (id, one_way) in pending_one_way {
        app.undo
            .run(&mut app.model, Box::new(SetSlabOneWay { id, one_way }));
    }
    for (id, usage) in pending_usage {
        app.undo
            .run(&mut app.model, Box::new(SetSlabUsage { id, usage }));
    }
    for (id, section) in pending_section {
        app.undo.run(
            &mut app.model,
            Box::new(squid_n_edit::SetSlabSection { id, section }),
        );
    }
    for (id, extent) in pending_extent {
        app.undo
            .run(&mut app.model, Box::new(SetAttachedExtent { id, extent }));
    }
    for (id, anchor) in pending_anchor {
        app.undo
            .run(&mut app.model, Box::new(SetAttachedAnchor { id, anchor }));
    }
    if let Some(id) = pending_delete {
        app.undo.run(&mut app.model, Box::new(DeleteSlab { id }));
    }
    if had_pending {
        app.staleness.mark_edited();
    }

    ui.separator();
    // ── スラブ追加フォーム ──────────────────────────────────
    ui.strong("スラブを追加");

    if app.model.nodes.len() < 3 {
        ui.label("スラブを追加するには節点が3つ以上必要です");
        return;
    }

    // 借用衝突を避けるため、節点一覧は先にローカルへ複製しておく
    // （app.model への参照を保持したまま app.slab_draft を可変参照しないため）。
    let node_ids: Vec<NodeId> = app.model.nodes.iter().map(|n| n.id).collect();

    // 境界頂点は 3〜N の可変長。スロット数は +/− ボタンで調整する。
    if app.slab_draft.nodes.len() < 3 {
        app.slab_draft.nodes.resize(3, None);
    }
    ui.label(
        "境界節点（頂点0→1→2→…→0 の順で外周を辿り、その辺 i=節点i→節点i+1 を持つ梁を検索します。3〜N 節点対応）:",
    );
    ui.horizontal(|ui| {
        if ui.button("+ 頂点を追加").clicked() {
            app.slab_draft.nodes.push(None);
        }
        if ui
            .add_enabled(
                app.slab_draft.nodes.len() > 3,
                egui::Button::new("− 頂点を削除"),
            )
            .on_hover_text("末尾の頂点スロットを削除（最小3）")
            .clicked()
        {
            app.slab_draft.nodes.pop();
        }
        ui.label(format!("頂点数: {}", app.slab_draft.nodes.len()));
    });
    ui.horizontal_wrapped(|ui| {
        let n_slots = app.slab_draft.nodes.len();
        for k in 0..n_slots {
            let text = app.slab_draft.nodes[k]
                .map(|n| format!("N{}", n.0))
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt(format!("slab_draft_node_{}", k))
                .selected_text(format!("頂点{}: {}", k, text))
                .show_ui(ui, |ui| {
                    for &nid in &node_ids {
                        let label = format!("N{}", nid.0);
                        if ui
                            .selectable_label(app.slab_draft.nodes[k] == Some(nid), &label)
                            .clicked()
                        {
                            app.slab_draft.nodes[k] = Some(nid);
                        }
                    }
                });
        }
    });

    ui.horizontal(|ui| {
        ui.label("荷重種別:");
        ui.add(egui::TextEdit::singleline(&mut app.slab_draft.load_kind).desired_width(60.0));
        ui.label("荷重 [kN/m²]:");
        ui.add(egui::TextEdit::singleline(&mut app.slab_draft.load_value).desired_width(80.0));
    });

    ui.horizontal(|ui| {
        // 断面（板厚・コンクリート材料）。板厚を持つ断面だけを候補にする。
        ui.horizontal(|ui| {
            ui.label("断面:");
            // 下書きの断面が消えている（削除・ID 繰り上げ）場合は未割当へ戻す。
            // 残したままだと `AddSlab` が参照検証で Noop になり、「追加」を押しても
            // 何も起きない状態になる。
            let resolved = app
                .slab_draft
                .section
                .and_then(|sid| app.model.sections.get(sid.index()))
                .filter(|sec| sec.thickness.is_some_and(|t| t > 0.0));
            if resolved.is_none() {
                app.slab_draft.section = None;
            }
            let label = resolved
                .map(|sec| sec.display_name())
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt("slab_draft_section")
                .selected_text(label)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut app.slab_draft.section, None, "―");
                    for sec in &app.model.sections {
                        if sec.thickness.is_some_and(|t| t > 0.0) {
                            ui.selectable_value(
                                &mut app.slab_draft.section,
                                Some(sec.id),
                                sec.display_name(),
                            );
                        }
                    }
                });
        })
        .response
        .on_hover_text(
            "床の板厚と自重は断面から決まります。断面が未割当の床は解析前チェックで止まります",
        );
        ui.label("用途（積載荷重）:")
            .on_hover_text("令別表第1 の積載荷重（骨組用）を「LL(架構用)」ケースへ分配します");
        egui::ComboBox::from_id_salt("slab_draft_usage")
            .selected_text(usage_label(app.slab_draft.usage))
            .show_ui(ui, |ui| {
                for &u in USAGE_PRESETS {
                    ui.selectable_value(&mut app.slab_draft.usage, u, usage_label(u));
                }
            });
        if let Some(u) = app.slab_draft.usage {
            use squid_n_core::model::LoadPurpose;
            // 表示は kN/m²。
            ui.label(format!(
                "床用 {:.2} / 骨組用 {:.2} / 地震用 {:.2} kN/m²",
                area_load_kn_per_m2(u.live_load(LoadPurpose::Floor)),
                area_load_kn_per_m2(u.live_load(LoadPurpose::Frame)),
                area_load_kn_per_m2(u.live_load(LoadPurpose::Seismic)),
            ));
        }
    });

    ui.horizontal(|ui| {
        ui.label("分配法:");
        ui.selectable_value(
            &mut app.slab_draft.method,
            DistributionMethod::TriTrapezoid,
            "三角/台形(45°法)",
        );
        ui.selectable_value(
            &mut app.slab_draft.method,
            DistributionMethod::OneWay,
            "一方向",
        );
        ui.selectable_value(
            &mut app.slab_draft.method,
            DistributionMethod::TributaryArea,
            "負担面積",
        );
    });

    let selected: Vec<NodeId> = app.slab_draft.nodes.iter().filter_map(|n| *n).collect();
    let mut dedup = selected.clone();
    dedup.sort_by_key(|n| n.0);
    dedup.dedup();
    // 全スロットが埋まり（selected.len == slots）、3頂点以上、重複がないこと。
    let n_slots = app.slab_draft.nodes.len();
    let can_add = selected.len() == n_slots && n_slots >= 3 && dedup.len() == n_slots;

    if ui
        .add_enabled(can_add, egui::Button::new("+ 追加"))
        .on_hover_text("境界節点が3つ以上すべて選択され、かつ重複がない場合に追加できます")
        .clicked()
    {
        let boundary: Vec<NodeId> = app
            .slab_draft
            .nodes
            .iter()
            .map(|n| n.expect("can_add で全スロット Some を確認済み"))
            .collect();
        let value_kn_m2 = app
            .slab_draft
            .load_value
            .trim()
            .parse::<f64>()
            .unwrap_or(0.0);
        let value = to_internal::area_load_kn_per_m2(value_kn_m2);
        let kind = app.slab_draft.load_kind.trim();
        let kind = if kind.is_empty() { "DL" } else { kind }.to_string();
        app.undo.run(
            &mut app.model,
            Box::new(AddSlab {
                boundary,
                joists: Vec::new(),
                loads: vec![AreaLoad { kind, value }],
                method: app.slab_draft.method,
                usage: app.slab_draft.usage,
                section: app.slab_draft.section,
            }),
        );
        app.staleness.mark_edited();
    }

    attached_section(ui, app);
    joists_section(ui, app);
}

/// 取り付き領域（片持ちスラブ・バルコニー・出隅）の入力セクション。
///
/// 主架構に囲まれていない床は、囲まれた領域（大梁が囲むパネル）と違って境界を
/// 節点で描けない。取付き先（大梁の 2 節点、または柱の 1 節点）と張り出し量で作る。
/// 張り出し量の符号は、線なら取付き線 1→2 の**左側が正**、点なら全体座標の X/Y の向き。
fn attached_section(ui: &mut egui::Ui, app: &mut App) {
    use squid_n_core::model::{LoadTransfer, RegionAnchor};

    ui.separator();
    ui.strong("取り付き領域を追加（片持ち・バルコニー・出隅）");
    ui.label(
        "主架構に囲まれない床です。取付き先（大梁の2節点、または柱の1節点）と張り出し量で作ります。張り出し量の符号は、線なら取付き線 1→2 の左が正、点なら全体座標 X/Y の向きです。",
    );

    ui.horizontal(|ui| {
        ui.label("名前:");
        ui.add(egui::TextEdit::singleline(&mut app.slab_draft.attached_name).desired_width(120.0));
        ui.label("取付き先:");
        ui.selectable_value(&mut app.slab_draft.attached_point, false, "線（大梁）");
        ui.selectable_value(&mut app.slab_draft.attached_point, true, "点（柱）");
    });

    let node_ids: Vec<NodeId> = app.model.nodes.iter().map(|n| n.id).collect();
    let n_slots = if app.slab_draft.attached_point { 1 } else { 2 };
    ui.horizontal(|ui| {
        for k in 0..n_slots {
            ui.label(format!("節点{}:", k + 1));
            let label = app.slab_draft.attached_nodes[k]
                .map(|n| n.0.to_string())
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt(("attached_node", k))
                .selected_text(label)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut app.slab_draft.attached_nodes[k], None, "―");
                    for id in &node_ids {
                        ui.selectable_value(
                            &mut app.slab_draft.attached_nodes[k],
                            Some(*id),
                            id.0.to_string(),
                        );
                    }
                });
        }
    });

    ui.horizontal(|ui| {
        let labels = if app.slab_draft.attached_point {
            ["X 方向 [mm]:", "Y 方向 [mm]:"]
        } else {
            ["始端側 [mm]:", "終端側 [mm]:"]
        };
        for (label, value) in labels.iter().zip(app.slab_draft.attached_extent.iter_mut()) {
            ui.label(*label);
            ui.add(egui::TextEdit::singleline(value).desired_width(70.0));
        }
    });

    if !app.slab_draft.attached_point {
        ui.horizontal(|ui| {
            ui.label("荷重の出口:");
            ui.selectable_value(
                &mut app.slab_draft.attached_transfer,
                LoadTransfer::Anchor,
                "取付き線へ分布",
            );
            ui.selectable_value(
                &mut app.slab_draft.attached_transfer,
                LoadTransfer::Columns,
                "両端の柱へ集中",
            );
        });
    }

    let extent: Option<[f64; 2]> = {
        let a = app.slab_draft.attached_extent[0].trim().parse::<f64>().ok();
        let b = app.slab_draft.attached_extent[1].trim().parse::<f64>().ok();
        a.zip(b).map(|(a, b)| [a, b])
    };
    let anchor: Option<RegionAnchor> = if app.slab_draft.attached_point {
        app.slab_draft.attached_nodes[0].map(RegionAnchor::Point)
    } else {
        match (
            app.slab_draft.attached_nodes[0],
            app.slab_draft.attached_nodes[1],
        ) {
            (Some(a), Some(b)) if a != b => Some(RegionAnchor::Line {
                nodes: [a, b],
                span: [0.0, 1.0],
                transfer: app.slab_draft.attached_transfer,
            }),
            _ => None,
        }
    };

    let ready = anchor.is_some() && extent.is_some();
    if !ready {
        ui.label("取付き先の節点と張り出し量を指定してください");
    }
    if ui
        .add_enabled(ready, egui::Button::new("取り付き領域を追加"))
        .clicked()
    {
        if let (Some(anchor), Some(extent)) = (anchor, extent) {
            app.undo.run(
                &mut app.model,
                Box::new(squid_n_edit::AddAttachedFloorRegion {
                    name: app.slab_draft.attached_name.trim().to_string(),
                    anchor,
                    extent,
                    // 版の仕様（断面・仕上荷重・室用途）は、追加後に一覧表から与える。
                    plate: None,
                }),
            );
            app.staleness.mark_edited();
        }
    }
}

/// 小梁（`JoistLine`）の入力セクション。対象スラブを選び、支持2節点＋負担幅
/// `spacing` で小梁を追加/削除する。小梁は矩形スラブの二段階伝達
/// （`distribute_rect_with_joists`）でのみ使われ、分配法が「三角/台形」または
/// 「一方向」のとき有効になる（それ以外の分配法では無視される）。
///
/// 小梁の架かる方向 `dir` は支持2節点の平面（XY）ベクトルから自動算定する。
fn joists_section(ui: &mut egui::Ui, app: &mut App) {
    ui.separator();
    ui.strong("小梁を入力（矩形スラブの二段階伝達）");
    ui.label(
        "対象スラブを選び、小梁が架かる支持2節点と負担幅を指定します。分配法が「三角/台形」または「一方向」のときに有効です。",
    );

    if app.model.floor_regions.is_empty() {
        ui.label("スラブがありません");
        return;
    }

    // 対象スラブ選択。
    // 小梁 UI の対象は囲まれた領域のみ（取り付きは二段階伝達の対象外）。
    let slab_ids: Vec<FloorRegionId> = app
        .model
        .floor_regions
        .iter()
        .filter(|s| !s.is_attached())
        .map(|s| s.id)
        .collect();
    if slab_ids.is_empty() {
        ui.label("囲まれたスラブがありません（小梁は囲まれた領域のみ）");
        return;
    }
    if app
        .slab_draft
        .joist_target
        .is_none_or(|t| !slab_ids.contains(&t))
    {
        app.slab_draft.joist_target = slab_ids.first().copied();
    }
    egui::ComboBox::from_id_salt("joist_target_slab")
        .selected_text(
            app.slab_draft
                .joist_target
                .map(|t| format!("スラブ #{}", t.0))
                .unwrap_or_else(|| "―".to_string()),
        )
        .show_ui(ui, |ui| {
            for &sid in &slab_ids {
                ui.selectable_value(
                    &mut app.slab_draft.joist_target,
                    Some(sid),
                    format!("スラブ #{}", sid.0),
                );
            }
        });

    let Some(target) = app.slab_draft.joist_target else {
        return;
    };
    let Some(slab_idx) = app.model.floor_regions.iter().position(|s| s.id == target) else {
        return;
    };

    // 変更は借用衝突を避けるため、UI 走査後に SetSlabJoists で一括反映する。
    let mut new_joists: Option<Vec<JoistLine>> = None;

    // 既存小梁の一覧（削除ボタン付き）。
    let joists = app.model.floor_regions[slab_idx].joist_lines().to_vec();
    if joists.is_empty() {
        ui.label("この床には小梁がありません");
    } else {
        for (k, j) in joists.iter().enumerate() {
            ui.horizontal(|ui| {
                let sec = j
                    .section
                    .map(|s| format!("S{}", s.0))
                    .unwrap_or_else(|| "断面なし".to_string());
                ui.label(format!(
                    "小梁{}: 支持 N{}–N{}, 負担幅 {:.0} mm, 断面 {}",
                    k, j.support[0].0, j.support[1].0, j.spacing, sec
                ));
                // 交差接合の指定: 剛接十字（既定）か、他の小梁への受け/架け（ピン）か。
                // 「受け:小梁c」を選ぶとこの小梁が架け梁となり、交点で小梁c にピン接合で
                // 載る（曲げは伝えず鉛直せん断のみ。交差しない相手を選んでも無効）。
                let cur = match j.pinned_onto {
                    Some(c) => format!("受け:小梁{c}"),
                    None => "剛接十字".to_string(),
                };
                egui::ComboBox::from_id_salt(format!("joist_pin_{k}"))
                    .selected_text(cur)
                    .show_ui(ui, |ui| {
                        let mut sel = j.pinned_onto;
                        ui.selectable_value(&mut sel, None, "剛接十字");
                        for c in 0..joists.len() {
                            if c == k {
                                continue;
                            }
                            ui.selectable_value(&mut sel, Some(c), format!("受け:小梁{c}"));
                        }
                        if sel != j.pinned_onto {
                            let mut v = joists.clone();
                            v[k].pinned_onto = sel;
                            new_joists = Some(v);
                        }
                    })
                    .response
                    .on_hover_text(
                        "剛接十字＝交点で二方向曲げ連続（たわみ抑制）。受け/架け＝架け梁が受け梁にピンで載る（鉛直せん断のみ伝達）。",
                    );
                if ui.button("🗑").on_hover_text("この小梁を削除").clicked() {
                    let mut v = joists.clone();
                    v.remove(k);
                    // 削除で小梁インデックスがずれるため、pinned_onto を補正する。
                    for jj in v.iter_mut() {
                        match jj.pinned_onto {
                            Some(c) if c == k => jj.pinned_onto = None,
                            Some(c) if c > k => jj.pinned_onto = Some(c - 1),
                            _ => {}
                        }
                    }
                    new_joists = Some(v);
                }
            });
        }
    }

    // 小梁の実部材化（実 Beam 要素を生成し、応力解析・断面検定の対象にする）。
    if !joists.is_empty() {
        // 実 Beam が未生成の小梁本数を数える。
        let beam_exists = |a: NodeId, b: NodeId| -> bool {
            app.model.elements.iter().any(|e| {
                e.kind == squid_n_core::model::ElementKind::Beam
                    && e.nodes.len() == 2
                    && ((e.nodes[0] == a && e.nodes[1] == b)
                        || (e.nodes[0] == b && e.nodes[1] == a))
            })
        };
        let unmaterialized = joists
            .iter()
            .filter(|j| j.support[0] != j.support[1] && !beam_exists(j.support[0], j.support[1]))
            .count();
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    unmaterialized > 0,
                    egui::Button::new("小梁を実部材化"),
                )
                .on_hover_text(
                    "各小梁の支持2節点に実 Beam 要素を生成します。実部材化した小梁には床荷重が等分布荷重として載り、応力解析・断面検定の対象になります。",
                )
                .clicked()
            {
                app.undo.run(
                    &mut app.model,
                    Box::new(squid_n_edit::MaterializeSlabJoists { slab: target }),
                );
                app.staleness.mark_edited();
            }
            ui.label(if unmaterialized > 0 {
                format!("未実部材化: {unmaterialized}本")
            } else {
                "すべて実部材化済み".to_string()
            });
        });
    }

    // 小梁の追加フォーム。
    let node_ids: Vec<NodeId> = app.model.nodes.iter().map(|n| n.id).collect();
    ui.horizontal(|ui| {
        for e in 0..2 {
            let text = app.slab_draft.joist_supports[e]
                .map(|n| format!("N{}", n.0))
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt(format!("joist_support_{e}"))
                .selected_text(format!("支持{e}: {text}"))
                .show_ui(ui, |ui| {
                    for &nid in &node_ids {
                        if ui
                            .selectable_label(
                                app.slab_draft.joist_supports[e] == Some(nid),
                                format!("N{}", nid.0),
                            )
                            .clicked()
                        {
                            app.slab_draft.joist_supports[e] = Some(nid);
                        }
                    }
                });
        }
        ui.label("負担幅 [mm]:");
        ui.add(egui::TextEdit::singleline(&mut app.slab_draft.joist_spacing).desired_width(80.0));
        ui.label("断面:")
            .on_hover_text("床の中での小梁設計（単純支持梁の曲げ・たわみ検定）に用いる断面");
        let sec_text = app
            .slab_draft
            .joist_section
            .map(|s| format!("S{}", s.0))
            .unwrap_or_else(|| "―".to_string());
        egui::ComboBox::from_id_salt("joist_section")
            .selected_text(sec_text)
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut app.slab_draft.joist_section, None, "―");
                for sec in &app.model.sections {
                    ui.selectable_value(
                        &mut app.slab_draft.joist_section,
                        Some(sec.id),
                        format!("S{} {}", sec.id.0, sec.name),
                    );
                }
            });
    });

    let s0 = app.slab_draft.joist_supports[0];
    let s1 = app.slab_draft.joist_supports[1];
    let spacing = app
        .slab_draft
        .joist_spacing
        .trim()
        .parse::<f64>()
        .unwrap_or(0.0);
    // 追加可能な小梁を安全に構成する。両支持節点が現存し（節点削除でドラフトが
    // 陳腐化しても out-of-bounds しないよう `nodes.get` で確認）、平面（XY）方向に
    // 有意な離間がある（`dir≈[0,0]` は分配エンジンが Y 軸へ暗黙フォールバックし
    // 誤分配となるため弾く）場合のみ Some を返す。
    let addable_joist: Option<JoistLine> = (|| {
        let (a, b) = (s0?, s1?);
        if a == b || spacing <= 0.0 {
            return None;
        }
        let ca = app.model.nodes.get(a.index())?.coord;
        let cb = app.model.nodes.get(b.index())?.coord;
        let dir = [cb[0] - ca[0], cb[1] - ca[1]];
        if dir[0].hypot(dir[1]) <= 1e-9 {
            return None; // 平面上で重なる2節点（鉛直に積層等）は小梁として無効。
        }
        Some(JoistLine {
            dir,
            spacing,
            support: [a, b],
            section: app.slab_draft.joist_section,
            pinned_onto: None,
        })
    })();

    if ui
        .add_enabled(addable_joist.is_some(), egui::Button::new("+ 小梁を追加"))
        .on_hover_text(
            "現存する異なる支持2節点（平面上で離れている）と正の負担幅を指定してください",
        )
        .clicked()
    {
        if let Some(joist) = addable_joist {
            let mut v = joists.clone();
            v.push(joist);
            new_joists = Some(v);
        }
    }

    if let Some(v) = new_joists {
        app.undo.run(
            &mut app.model,
            Box::new(SetSlabJoists {
                id: target,
                joists: v,
            }),
        );
        app.staleness.mark_edited();
    }
}

fn attached_boundary_cell(
    ui: &mut egui::Ui,
    id: FloorRegionId,
    anchor: RegionAnchor,
    extent: [f64; 2],
    node_ids: &[NodeId],
    pending_extent: &mut Vec<(FloorRegionId, [f64; 2])>,
    pending_anchor: &mut Vec<(FloorRegionId, RegionAnchor)>,
) {
    ui.vertical(|ui| {
        match anchor {
            RegionAnchor::Line {
                nodes,
                span,
                transfer,
            } => {
                ui.horizontal(|ui| {
                    for k in 0..2 {
                        let mut sel = nodes[k];
                        egui::ComboBox::from_id_salt(("att_anc", id.0, k))
                            .selected_text(format!("N{}", sel.0))
                            .show_ui(ui, |ui| {
                                for &nid in node_ids {
                                    ui.selectable_value(&mut sel, nid, format!("N{}", nid.0));
                                }
                            });
                        if sel != nodes[k] && sel != nodes[1 - k] {
                            let mut n = nodes;
                            n[k] = sel;
                            pending_anchor.push((
                                id,
                                RegionAnchor::Line {
                                    nodes: n,
                                    span,
                                    transfer,
                                },
                            ));
                        }
                    }
                });
            }
            RegionAnchor::Point(n) => {
                let mut sel = n;
                egui::ComboBox::from_id_salt(("att_pt", id.0))
                    .selected_text(format!("N{}", sel.0))
                    .show_ui(ui, |ui| {
                        for &nid in node_ids {
                            ui.selectable_value(&mut sel, nid, format!("N{}", nid.0));
                        }
                    });
                if sel != n {
                    pending_anchor.push((id, RegionAnchor::Point(sel)));
                }
            }
        }
        ui.horizontal(|ui| {
            let mut e = extent;
            ui.add(egui::DragValue::new(&mut e[0]).suffix(" mm"));
            ui.add(egui::DragValue::new(&mut e[1]).suffix(" mm"));
            if e != extent && e[0].is_finite() && e[1].is_finite() {
                pending_extent.push((id, e));
            }
        });
    });
}
