use crate::app::App;
use squid_n_core::ids::{FloorRegionId, NodeId, SlabId};
use squid_n_core::model::{AreaLoad, DistributionMethod, OneWayDir, SlabUsage};
use squid_n_core::model::{RegionAnchor, SlabShape};
use squid_n_core::units::to_display::area_load_kn_per_m2;
use squid_n_core::units::to_internal;
use squid_n_edit::{
    AddSlab, DeleteSlab, SetAttachedAnchor, SetAttachedExtent, SetFloorRegionName, SetSlabOneWay,
    SetSlabUsage,
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
    /// 取り付き領域の取付き先の節点（線なら両端、点なら 1 つ目だけを使う）。
    pub attached_nodes: [Option<NodeId>; 2],
    /// 取り付き先を点（柱）にするか。false は線（取付き線）。
    pub attached_point: bool,
    /// 張り出し量の入力文字列 [mm]（線: 始端側・終端側、点: X 方向・Y 方向）。
    pub attached_extent: [String; 2],
    /// 取付き線に載る領域の荷重の出口。
    pub attached_transfer: squid_n_core::model::LoadTransfer,
    /// 取付き線上の無次元区間 `[t_i, t_j]`（0.0〜1.0）。全長は `[0.0, 1.0]`。点取付きでは使わない。
    pub attached_span: [f64; 2],
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
            attached_nodes: [None; 2],
            attached_point: false,
            attached_extent: ["1000".to_string(), "1000".to_string()],
            attached_transfer: squid_n_core::model::LoadTransfer::Anchor,
            attached_span: [0.0, 1.0],
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

fn kind_label(slab: &squid_n_core::model::Slab) -> &'static str {
    if slab.is_attached() {
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
        "床領域は大梁が囲む1区画です（名前・所属する小梁・所属する床板を持ちます）。床板（スラブ）は、\
         大梁または小梁で囲まれた版、または主架構に取り付く版（片持ち・バルコニー・出隅）です。\
         版の仕様（断面・荷重・用途・分配法）は床板が持ちます。断面が未割当の床板には床荷重・\
         スラブ検定・協力幅は生じません（結果タブ/モデルタブの3Dビューで表示モード「CMQ図」を\
         選ぶと分配結果を確認できます）。",
    );
    ui.separator();

    // ── 床領域一覧（表示名の編集のみ） ──────────────────────
    ui.strong("床領域（大梁の区画）");
    let mut pending_region_name: Vec<(FloorRegionId, String)> = Vec::new();
    table_util::standard_table(
        ui,
        "floor_regions_tbl",
        &[
            Col::id(),
            Col::name("名前"),
            Col::text("境界節点"),
            Col::label("床板"),
            Col::label("小梁"),
        ],
        app.core.model.floor_regions.len(),
        |row| {
            let i = row.index();
            let region = &app.core.model.floor_regions[i];
            row.col(|ui| {
                table_util::id_label(ui, region.id.0);
            });
            row.col(|ui| {
                let mut name = region.name.clone();
                let resp = table_util::cell_text_edit(ui, &mut name);
                if resp.changed() {
                    pending_region_name.push((region.id, name));
                }
            });
            row.col(|ui| {
                let s = region
                    .boundary
                    .iter()
                    .map(|n| n.0.to_string())
                    .collect::<Vec<_>>()
                    .join("-");
                table_util::text_cell(ui, &s);
            });
            row.col(|ui| {
                let cnt = region.slab_ids.len();
                if cnt == 0 {
                    table_util::muted_cell(ui, "―", "床板が割り当たっていません");
                } else {
                    ui.label(format!("{cnt}枚"));
                }
            });
            row.col(|ui| {
                let cnt = region.secondary_joists.len();
                if cnt == 0 {
                    table_util::muted_cell(ui, "―", "小梁が配置されていません");
                } else {
                    ui.label(format!("{cnt}本"));
                }
            });
        },
    );
    for (id, name) in pending_region_name {
        app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(SetFloorRegionName { id, name }),
        );
        app.core.scoped.staleness.mark_edited();
    }

    ui.add_space(8.0);
    ui.strong("床板（スラブ）");

    // ── 一覧表 ──────────────────────────────────────────
    let n = app.core.model.slabs.len();
    let mut pending_delete: Option<SlabId> = None;
    let mut pending_one_way: Vec<(SlabId, Option<OneWayDir>)> = Vec::new();
    let mut pending_usage: Vec<(SlabId, Option<SlabUsage>)> = Vec::new();
    let mut pending_section: Vec<(SlabId, Option<squid_n_core::ids::SectionId>)> = Vec::new();
    let mut pending_extent: Vec<(SlabId, [f64; 2])> = Vec::new();
    let mut pending_anchor: Vec<(SlabId, RegionAnchor)> = Vec::new();
    let node_ids: Vec<NodeId> = app.core.model.nodes.iter().map(|n| n.id).collect();
    // 板状の断面（板厚を持つ断面）だけを候補にする。板厚が無い断面を割り当てても
    // 自重・数量が算定できないため、選ばせない。
    let slab_sections: Vec<(squid_n_core::ids::SectionId, String)> = app
        .core
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
            Col::text("所属床領域"),
            Col::text("境界節点"),
            Col::text("荷重"),
            Col::name("分配法"),
            Col::name("種別"),
            Col::name("一方向"),
            Col::text("用途"),
            Col::text("断面"),
            Col::actions(),
        ],
        n,
        |row| {
            let i = row.index();
            let slab = &app.core.model.slabs[i];
            row.col(|ui| {
                table_util::id_label(ui, slab.id.0);
            });
            row.col(|ui| {
                // どの床領域（大梁の区画）に属するかは `slab_ids` から逆引きする
                // （取り付く床板・浮き床板はどの床領域からも参照されない）。
                let owner = app
                    .core
                    .model
                    .floor_regions
                    .iter()
                    .find(|r| r.slab_ids.contains(&slab.id));
                match owner {
                    Some(r) if !r.name.is_empty() => table_util::text_cell(ui, &r.name),
                    Some(r) => table_util::text_cell(ui, &format!("#{}", r.id.0)),
                    None => table_util::muted_cell(ui, "―", "どの床領域からも参照されていません"),
                }
            });
            row.col(|ui| match &slab.shape {
                SlabShape::Enclosed { boundary } => {
                    let s = boundary
                        .iter()
                        .map(|n| n.0.to_string())
                        .collect::<Vec<_>>()
                        .join("-");
                    table_util::text_cell(ui, &s);
                }
                SlabShape::Attached { anchor, extent } => {
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
                    .loads
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
                // 床板の種別（囲まれた床板か取り付く床板か）は形そのものなので、
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
                    .core
                    .model
                    .slab_section(slab)
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
                if table_util::delete_cell(ui, "この床板を削除", None) {
                    pending_delete = Some(slab.id);
                }
            });
        },
    );

    let had_pending = !pending_one_way.is_empty()
        || !pending_usage.is_empty()
        || !pending_section.is_empty()
        || !pending_extent.is_empty()
        || !pending_anchor.is_empty()
        || pending_delete.is_some();
    for (id, one_way) in pending_one_way {
        app.core
            .scoped
            .undo
            .run(&mut app.core.model, Box::new(SetSlabOneWay { id, one_way }));
    }
    for (id, usage) in pending_usage {
        app.core
            .scoped
            .undo
            .run(&mut app.core.model, Box::new(SetSlabUsage { id, usage }));
    }
    for (id, section) in pending_section {
        app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(squid_n_edit::SetSlabSection { id, section }),
        );
    }
    for (id, extent) in pending_extent {
        app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(SetAttachedExtent { id, extent }),
        );
    }
    for (id, anchor) in pending_anchor {
        app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(SetAttachedAnchor { id, anchor }),
        );
    }
    if let Some(id) = pending_delete {
        app.core
            .scoped
            .undo
            .run(&mut app.core.model, Box::new(DeleteSlab { id }));
    }
    if had_pending {
        app.core.scoped.staleness.mark_edited();
    }

    ui.separator();
    // ── 床板追加フォーム ──────────────────────────────────
    ui.strong("床板を追加");

    if app.core.model.nodes.len() < 3 {
        ui.label("床板を追加するには節点が3つ以上必要です");
        return;
    }

    // 借用衝突を避けるため、節点一覧は先にローカルへ複製しておく
    // （app.core.model への参照を保持したまま app.ui.scoped.slab_draft を可変参照しないため）。
    let node_ids: Vec<NodeId> = app.core.model.nodes.iter().map(|n| n.id).collect();

    // 境界頂点は 3〜N の可変長。スロット数は +/− ボタンで調整する。
    if app.ui.scoped.slab_draft.nodes.len() < 3 {
        app.ui.scoped.slab_draft.nodes.resize(3, None);
    }
    ui.label(
        "境界節点（頂点0→1→2→…→0 の順で外周を辿り、その辺 i=節点i→節点i+1 を持つ梁を検索します。3〜N 節点対応）:",
    );
    ui.horizontal(|ui| {
        if ui.button("+ 頂点を追加").clicked() {
            app.ui.scoped.slab_draft.nodes.push(None);
        }
        if ui
            .add_enabled(
                app.ui.scoped.slab_draft.nodes.len() > 3,
                egui::Button::new("− 頂点を削除"),
            )
            .on_hover_text("末尾の頂点スロットを削除（最小3）")
            .clicked()
        {
            app.ui.scoped.slab_draft.nodes.pop();
        }
        ui.label(format!("頂点数: {}", app.ui.scoped.slab_draft.nodes.len()));
    });
    ui.horizontal_wrapped(|ui| {
        let n_slots = app.ui.scoped.slab_draft.nodes.len();
        for k in 0..n_slots {
            let text = app.ui.scoped.slab_draft.nodes[k]
                .map(|n| format!("N{}", n.0))
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt(format!("slab_draft_node_{}", k))
                .selected_text(format!("頂点{}: {}", k, text))
                .show_ui(ui, |ui| {
                    for &nid in &node_ids {
                        let label = format!("N{}", nid.0);
                        if ui
                            .selectable_label(
                                app.ui.scoped.slab_draft.nodes[k] == Some(nid),
                                &label,
                            )
                            .clicked()
                        {
                            app.ui.scoped.slab_draft.nodes[k] = Some(nid);
                        }
                    }
                });
        }
    });

    ui.horizontal(|ui| {
        ui.label("荷重種別:");
        ui.add(
            egui::TextEdit::singleline(&mut app.ui.scoped.slab_draft.load_kind).desired_width(60.0),
        );
        ui.label("荷重 [kN/m²]:");
        ui.add(
            egui::TextEdit::singleline(&mut app.ui.scoped.slab_draft.load_value)
                .desired_width(80.0),
        );
    });

    ui.horizontal(|ui| {
        // 断面（板厚・コンクリート材料）。板厚を持つ断面だけを候補にする。
        ui.horizontal(|ui| {
            ui.label("断面:");
            // 下書きの断面が消えている（削除・ID 繰り上げ）場合は未割当へ戻す。
            // 残したままだと `AddSlab` が参照検証で Noop になり、「追加」を押しても
            // 何も起きない状態になる。
            let resolved = app
                .ui
                .scoped
                .slab_draft
                .section
                .and_then(|sid| app.core.model.sections.get(sid.index()))
                .filter(|sec| sec.thickness.is_some_and(|t| t > 0.0));
            if resolved.is_none() {
                app.ui.scoped.slab_draft.section = None;
            }
            let label = resolved
                .map(|sec| sec.display_name())
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt("slab_draft_section")
                .selected_text(label)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut app.ui.scoped.slab_draft.section, None, "―");
                    for sec in &app.core.model.sections {
                        if sec.thickness.is_some_and(|t| t > 0.0) {
                            ui.selectable_value(
                                &mut app.ui.scoped.slab_draft.section,
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
            .selected_text(usage_label(app.ui.scoped.slab_draft.usage))
            .show_ui(ui, |ui| {
                for &u in USAGE_PRESETS {
                    ui.selectable_value(&mut app.ui.scoped.slab_draft.usage, u, usage_label(u));
                }
            });
        if let Some(u) = app.ui.scoped.slab_draft.usage {
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
            &mut app.ui.scoped.slab_draft.method,
            DistributionMethod::TriTrapezoid,
            "三角/台形(45°法)",
        );
        ui.selectable_value(
            &mut app.ui.scoped.slab_draft.method,
            DistributionMethod::OneWay,
            "一方向",
        );
        ui.selectable_value(
            &mut app.ui.scoped.slab_draft.method,
            DistributionMethod::TributaryArea,
            "負担面積",
        );
    });

    let selected: Vec<NodeId> = app
        .ui
        .scoped
        .slab_draft
        .nodes
        .iter()
        .filter_map(|n| *n)
        .collect();
    let mut dedup = selected.clone();
    dedup.sort_by_key(|n| n.0);
    dedup.dedup();
    // 全スロットが埋まり（selected.len == slots）、3頂点以上、重複がないこと。
    let n_slots = app.ui.scoped.slab_draft.nodes.len();
    let can_add = selected.len() == n_slots && n_slots >= 3 && dedup.len() == n_slots;

    if ui
        .add_enabled(can_add, egui::Button::new("+ 追加"))
        .on_hover_text("境界節点が3つ以上すべて選択され、かつ重複がない場合に追加できます")
        .clicked()
    {
        let boundary: Vec<NodeId> = app
            .ui
            .scoped
            .slab_draft
            .nodes
            .iter()
            .map(|n| n.expect("can_add で全スロット Some を確認済み"))
            .collect();
        let value_kn_m2 = app
            .ui
            .scoped
            .slab_draft
            .load_value
            .trim()
            .parse::<f64>()
            .unwrap_or(0.0);
        let value = to_internal::area_load_kn_per_m2(value_kn_m2);
        let kind = app.ui.scoped.slab_draft.load_kind.trim();
        let kind = if kind.is_empty() { "DL" } else { kind }.to_string();
        app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(AddSlab {
                boundary,
                loads: vec![AreaLoad { kind, value }],
                method: app.ui.scoped.slab_draft.method,
                usage: app.ui.scoped.slab_draft.usage,
                section: app.ui.scoped.slab_draft.section,
            }),
        );
        app.core.scoped.staleness.mark_edited();
    }

    attached_section(ui, app);
}

/// 取り付き領域（片持ちスラブ・バルコニー・出隅）の入力セクション。
///
/// 主架構に囲まれていない床板は、囲まれた床板と違って境界を節点で描けない。
/// 取付き先（大梁の 2 節点、または柱の 1 節点）と張り出し量で作る。取り付く床板は
/// どの床領域からも参照されない独立した床板であり、名前は持たない。
/// 張り出し量の符号は、線なら取付き線 1→2 の**左側が正**、点なら全体座標の X/Y の向き。
fn attached_section(ui: &mut egui::Ui, app: &mut App) {
    use squid_n_core::model::{LoadTransfer, RegionAnchor};

    ui.separator();
    ui.strong("取り付く床板を追加（片持ち・バルコニー・出隅）");
    ui.label(
        "主架構に囲まれない床板です。取付き先（大梁の2節点、または柱の1節点）と張り出し量で作ります。張り出し量の符号は、線なら取付き線 1→2 の左が正、点なら全体座標 X/Y の向きです。",
    );

    ui.horizontal(|ui| {
        ui.label("取付き先:");
        ui.selectable_value(
            &mut app.ui.scoped.slab_draft.attached_point,
            false,
            "線（大梁）",
        );
        ui.selectable_value(
            &mut app.ui.scoped.slab_draft.attached_point,
            true,
            "点（柱）",
        );
    });

    let node_ids: Vec<NodeId> = app.core.model.nodes.iter().map(|n| n.id).collect();
    let n_slots = if app.ui.scoped.slab_draft.attached_point {
        1
    } else {
        2
    };
    ui.horizontal(|ui| {
        for k in 0..n_slots {
            ui.label(format!("節点{}:", k + 1));
            let label = app.ui.scoped.slab_draft.attached_nodes[k]
                .map(|n| n.0.to_string())
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt(("attached_node", k))
                .selected_text(label)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut app.ui.scoped.slab_draft.attached_nodes[k], None, "―");
                    for id in &node_ids {
                        ui.selectable_value(
                            &mut app.ui.scoped.slab_draft.attached_nodes[k],
                            Some(*id),
                            id.0.to_string(),
                        );
                    }
                });
        }
    });

    ui.horizontal(|ui| {
        let labels = if app.ui.scoped.slab_draft.attached_point {
            ["X 方向 [mm]:", "Y 方向 [mm]:"]
        } else {
            ["始端側 [mm]:", "終端側 [mm]:"]
        };
        for (label, value) in labels
            .iter()
            .zip(app.ui.scoped.slab_draft.attached_extent.iter_mut())
        {
            ui.label(*label);
            ui.add(egui::TextEdit::singleline(value).desired_width(70.0));
        }
    });

    if !app.ui.scoped.slab_draft.attached_point {
        ui.horizontal(|ui| {
            ui.label("荷重の出口:");
            ui.selectable_value(
                &mut app.ui.scoped.slab_draft.attached_transfer,
                LoadTransfer::Anchor,
                "取付き線へ分布",
            );
            ui.selectable_value(
                &mut app.ui.scoped.slab_draft.attached_transfer,
                LoadTransfer::Columns,
                "両端の柱へ集中",
            );
        });
        ui.horizontal(|ui| {
            ui.label("取付き線の区間 [0, 1]（既定は全長）:");
            ui.add(
                egui::DragValue::new(&mut app.ui.scoped.slab_draft.attached_span[0])
                    .range(0.0..=1.0)
                    .speed(0.01),
            );
            ui.label("〜");
            ui.add(
                egui::DragValue::new(&mut app.ui.scoped.slab_draft.attached_span[1])
                    .range(0.0..=1.0)
                    .speed(0.01),
            );
        });
    }

    let extent: Option<[f64; 2]> = {
        let a = app.ui.scoped.slab_draft.attached_extent[0]
            .trim()
            .parse::<f64>()
            .ok();
        let b = app.ui.scoped.slab_draft.attached_extent[1]
            .trim()
            .parse::<f64>()
            .ok();
        a.zip(b).map(|(a, b)| [a, b])
    };
    let span = app.ui.scoped.slab_draft.attached_span;
    // `Model::validate`（squid-n-core）・`AddAttachedSlab`（squid-n-edit）と同じ範囲。
    let span_ok = span[0].is_finite()
        && span[1].is_finite()
        && span[0] >= 0.0
        && span[1] <= 1.0
        && span[1] - span[0] > 1e-9;
    let anchor: Option<RegionAnchor> = if app.ui.scoped.slab_draft.attached_point {
        app.ui.scoped.slab_draft.attached_nodes[0].map(RegionAnchor::Point)
    } else {
        match (
            app.ui.scoped.slab_draft.attached_nodes[0],
            app.ui.scoped.slab_draft.attached_nodes[1],
        ) {
            (Some(a), Some(b)) if a != b && span_ok => Some(RegionAnchor::Line {
                nodes: [a, b],
                span,
                transfer: app.ui.scoped.slab_draft.attached_transfer,
            }),
            _ => None,
        }
    };

    if !app.ui.scoped.slab_draft.attached_point && !span_ok {
        ui.label("取付き線の区間は始端 < 終端にしてください");
    }
    let ready = anchor.is_some() && extent.is_some();
    if !ready {
        ui.label("取付き先の節点と張り出し量を指定してください");
    }
    if ui
        .add_enabled(ready, egui::Button::new("取り付く床板を追加"))
        .clicked()
    {
        if let (Some(anchor), Some(extent)) = (anchor, extent) {
            app.core.scoped.undo.run(
                &mut app.core.model,
                Box::new(squid_n_edit::AddAttachedSlab {
                    anchor,
                    extent,
                    // 版の仕様（断面・仕上荷重・室用途）は、追加後に一覧表から与える。
                    plate: squid_n_core::model::SlabPlate::default(),
                }),
            );
            app.core.scoped.staleness.mark_edited();
        }
    }
}

fn attached_boundary_cell(
    ui: &mut egui::Ui,
    id: SlabId,
    anchor: RegionAnchor,
    extent: [f64; 2],
    node_ids: &[NodeId],
    pending_extent: &mut Vec<(SlabId, [f64; 2])>,
    pending_anchor: &mut Vec<(SlabId, RegionAnchor)>,
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
                ui.horizontal(|ui| {
                    ui.label("区間:");
                    let mut s = span;
                    ui.add(egui::DragValue::new(&mut s[0]).range(0.0..=1.0).speed(0.01));
                    ui.label("〜");
                    ui.add(egui::DragValue::new(&mut s[1]).range(0.0..=1.0).speed(0.01));
                    // `Model::validate`（squid-n-core）・`SetAttachedAnchor`（squid-n-edit）と同じ範囲。
                    let s_ok = s[0].is_finite()
                        && s[1].is_finite()
                        && s[0] >= 0.0
                        && s[1] <= 1.0
                        && s[1] - s[0] > 1e-9;
                    if s != span && s_ok {
                        pending_anchor.push((
                            id,
                            RegionAnchor::Line {
                                nodes,
                                span: s,
                                transfer,
                            },
                        ));
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
            // 床板の取付き先には使わない（`RegionAnchor::FloorRegion` のドキュメント
            // 参照。壁側〔自立壁〕専用のアンカーであり、床板の編集 UI では到達しない）。
            RegionAnchor::FloorRegion { .. } => {}
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
