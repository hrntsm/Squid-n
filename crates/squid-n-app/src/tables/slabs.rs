use crate::app::App;
use squid_n_core::ids::{FloorPlateAssignmentRegionId, FloorRegionId, NodeId, SlabId};
use squid_n_core::model::{AreaLoad, DistributionMethod, LoadPurpose, OneWayDir, SlabUsage};
use squid_n_core::model::{RegionAnchor, SlabShape};
use squid_n_core::units::to_display::area_load_kn_per_m2;
use squid_n_core::units::to_internal;
use squid_n_edit::{
    AssignSlabToFloorPlateRegion, DeleteSlab, SetAttachedAnchor, SetAttachedExtent,
    SetFloorPlateRegionNoPlate, SetFloorRegionName, SetSlabOneWay, SetSlabUsage,
    UnsetFloorPlateRegion,
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
    /// 任意入力の積載荷重 [kN/m²]（床用・小梁用・大梁用・地震用の順）。
    pub custom_live_kn_m2: [String; 4],
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
            custom_live_kn_m2: std::array::from_fn(|_| "0".to_string()),
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
/// `None` は「なし（積載寄与なし）」。任意入力（`Custom`）は値を保持するため
/// ここには入れず、コンボ内の特別な項目から初期化する。並びは国交省営繕基準の表に概ね沿う。
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

/// 用途の実効 4 値 [N/mm²]（床用・小梁用・大梁用・地震用）。`None` はすべて 0。
fn usage_custom_values(u: Option<SlabUsage>) -> [f64; 4] {
    match u {
        Some(SlabUsage::Custom {
            floor,
            joist,
            frame,
            seismic,
        }) => [floor, joist, frame, seismic],
        Some(u) => [
            u.live_load(LoadPurpose::Floor),
            u.live_load(LoadPurpose::Joist),
            u.live_load(LoadPurpose::Frame),
            u.live_load(LoadPurpose::Seismic),
        ],
        None => [0.0; 4],
    }
}

/// 4 値 [N/mm²] を任意入力の積載荷重へまとめる。
fn custom_usage(values: [f64; 4]) -> SlabUsage {
    SlabUsage::Custom {
        floor: values[0],
        joist: values[1],
        frame: values[2],
        seismic: values[3],
    }
}

/// 用途の 4 値を kN/m² で並べた 1 行。
fn usage_values_text(u: SlabUsage) -> String {
    let [floor, joist, frame, seismic] = usage_custom_values(Some(u));
    format!(
        "床 {:.2} / 小梁 {:.2} / 大梁 {:.2} / 地震 {:.2} kN/m²",
        area_load_kn_per_m2(floor),
        area_load_kn_per_m2(joist),
        area_load_kn_per_m2(frame),
        area_load_kn_per_m2(seismic),
    )
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

    let n = app.core.model.slabs.len();
    let mut pending_delete: Option<SlabId> = None;
    let mut pending_one_way: Vec<(SlabId, Option<OneWayDir>)> = Vec::new();
    let mut pending_usage: Vec<(SlabId, Option<SlabUsage>)> = Vec::new();
    let mut pending_section: Vec<(SlabId, Option<squid_n_core::ids::SectionId>)> = Vec::new();
    let mut pending_extent: Vec<(SlabId, [f64; 2])> = Vec::new();
    let mut pending_anchor: Vec<(SlabId, RegionAnchor)> = Vec::new();
    let node_ids: Vec<NodeId> = app.core.model.nodes.iter().map(|n| n.id).collect();
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
                SlabShape::Enclosed => match app.core.model.slab_assignment_region(slab.id) {
                    Some(region) => {
                        table_util::text_cell(
                            ui,
                            &format!("R{}（{}辺）", region.id.0, region.boundary.len()),
                        );
                    }
                    None => table_util::muted_cell(ui, "―", "割当領域に属していません"),
                },
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
                ui.vertical(|ui| {
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
                            ui.separator();
                            let is_custom = matches!(slab.usage(), Some(SlabUsage::Custom { .. }));
                            if ui.selectable_label(is_custom, "任意入力").clicked() && !is_custom
                            {
                                let v = usage_custom_values(slab.usage());
                                pending_usage.push((slab.id, Some(custom_usage(v))));
                            }
                        },
                    );
                    match slab.usage() {
                        Some(SlabUsage::Custom {
                            floor,
                            joist,
                            frame,
                            seismic,
                        }) => {
                            let mut values = [floor, joist, frame, seismic];
                            let mut changed = false;
                            ui.horizontal_wrapped(|ui| {
                                for (label, value) in
                                    ["床", "小梁", "大梁", "地震"].iter().zip(values.iter_mut())
                                {
                                    ui.label(*label);
                                    let mut kn = area_load_kn_per_m2(*value);
                                    if ui
                                        .add(
                                            egui::DragValue::new(&mut kn)
                                                .speed(0.01)
                                                .suffix(" kN/m²"),
                                        )
                                        .changed()
                                    {
                                        *value = to_internal::area_load_kn_per_m2(kn);
                                        changed = true;
                                    }
                                }
                            });
                            if changed {
                                pending_usage.push((slab.id, Some(custom_usage(values))));
                            }
                        }
                        Some(u) => {
                            table_util::text_cell(ui, &usage_values_text(u));
                        }
                        None => {}
                    }
                });
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

    ui.add_space(8.0);
    ui.strong("小梁（二次部材）");
    ui.label(
        "小梁は解析要素ではなく、床板から受けた荷重を大梁へ伝えます。端部支持条件「自由」は\
         片持ち小梁（基端支持・先端自由）を表し、荷重は基端の鉛直反力として伝達します。",
    );
    crate::tables::secondary::secondary_member_placement_form(
        app,
        ui,
        squid_n_core::model::SecondaryMemberKind::Joist,
    );
    ui.add_space(4.0);
    crate::tables::secondary::secondary_member_list(
        app,
        ui,
        squid_n_core::model::SecondaryMemberKind::Joist,
    );

    ui.separator();
    ui.strong("床板の割当");
    ui.label(
        "囲まれた床板は、大梁・小梁で分割された床板割当領域へ1枚ずつ割り当てます。\
         任意の節点境界からは作成できません。",
    );

    let region_ids: Vec<FloorPlateAssignmentRegionId> = app
        .core
        .model
        .floor_assignment_regions
        .regions
        .iter()
        .map(|r| r.id)
        .collect();
    if region_ids.is_empty() {
        ui.label(
            "床板割当領域がありません。解析前処理（割当領域の再構築）を実行すると、\
             大梁・小梁で分割された領域が作られます。",
        );
    }
    table_util::standard_table(
        ui,
        "floor_assignment_regions_tbl",
        &[
            Col::id(),
            Col::text("所属床領域"),
            Col::text("状態"),
            Col::text("床板"),
            Col::text("境界支持部材"),
        ],
        region_ids.len(),
        |row| {
            let id = region_ids[row.index()];
            let region = app
                .core
                .model
                .floor_assignment_region(id)
                .expect("region_ids は同じモデルから取得");
            row.col(|ui| {
                table_util::id_label(ui, id.0);
            });
            row.col(|ui| {
                let owner = region.assignment.plate().and_then(|sid| {
                    app.core
                        .model
                        .floor_regions
                        .iter()
                        .find(|r| r.slab_ids.contains(&sid))
                });
                match owner {
                    Some(r) if !r.name.is_empty() => table_util::text_cell(ui, &r.name),
                    Some(r) => table_util::text_cell(ui, &format!("#{}", r.id.0)),
                    None => table_util::muted_cell(ui, "―", "未所属"),
                }
            });
            row.col(|ui| {
                let text = match region.assignment {
                    squid_n_core::model::PlateAssignment::Unset => "未設定",
                    squid_n_core::model::PlateAssignment::NoPlate => "版なし",
                    squid_n_core::model::PlateAssignment::Plate(_) => "版あり",
                };
                table_util::text_cell(ui, text);
            });
            row.col(|ui| match region.assignment.plate() {
                Some(sid) => table_util::text_cell(ui, &format!("#{}", sid.0)),
                None => table_util::muted_cell(ui, "―", "床板が割り当てられていません"),
            });
            row.col(|ui| {
                table_util::text_cell(ui, &format!("{}辺", region.boundary.len()));
            });
        },
    );

    ui.label(
        "下の「割り当てる床板の仕様」を設定し、未設定・版なしの領域へ割り当てます。\
         版ありの領域は上の床板一覧で編集してください。",
    );

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
        ui.horizontal(|ui| {
            ui.label("断面:");
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
            .on_hover_text("令別表第1 の積載荷重（大梁用）を「LL(架構用)」ケースへ分配します");
        egui::ComboBox::from_id_salt("slab_draft_usage")
            .selected_text(usage_label(app.ui.scoped.slab_draft.usage))
            .show_ui(ui, |ui| {
                for &u in USAGE_PRESETS {
                    ui.selectable_value(&mut app.ui.scoped.slab_draft.usage, u, usage_label(u));
                }
                ui.separator();
                let is_custom = matches!(
                    app.ui.scoped.slab_draft.usage,
                    Some(SlabUsage::Custom { .. })
                );
                if ui.selectable_label(is_custom, "任意入力").clicked() && !is_custom {
                    let v = usage_custom_values(app.ui.scoped.slab_draft.usage);
                    for (slot, value) in
                        app.ui.scoped.slab_draft.custom_live_kn_m2.iter_mut().zip(v)
                    {
                        *slot = format!("{:.2}", area_load_kn_per_m2(value));
                    }
                    app.ui.scoped.slab_draft.usage = Some(custom_usage(v));
                }
            });
        if let Some(SlabUsage::Custom { .. }) = app.ui.scoped.slab_draft.usage {
            ui.horizontal(|ui| {
                for (label, slot) in ["床用", "小梁用", "大梁用", "地震用"]
                    .iter()
                    .zip(app.ui.scoped.slab_draft.custom_live_kn_m2.iter_mut())
                {
                    ui.label(*label);
                    ui.add(egui::TextEdit::singleline(slot).desired_width(55.0));
                }
            });
            let v: [f64; 4] = std::array::from_fn(|i| {
                to_internal::area_load_kn_per_m2(
                    app.ui.scoped.slab_draft.custom_live_kn_m2[i]
                        .trim()
                        .parse::<f64>()
                        .unwrap_or(0.0),
                )
            });
            app.ui.scoped.slab_draft.usage = Some(custom_usage(v));
        } else if let Some(u) = app.ui.scoped.slab_draft.usage {
            ui.label(usage_values_text(u));
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
    let plate = squid_n_core::model::SlabPlate {
        section: app.ui.scoped.slab_draft.section,
        loads: vec![AreaLoad { kind, value }],
        usage: app.ui.scoped.slab_draft.usage,
        method: app.ui.scoped.slab_draft.method,
        one_way: None,
    };

    let mut pending_assign: Vec<FloorPlateAssignmentRegionId> = Vec::new();
    let mut pending_no_plate: Vec<FloorPlateAssignmentRegionId> = Vec::new();
    let mut pending_unset: Vec<FloorPlateAssignmentRegionId> = Vec::new();
    ui.horizontal_wrapped(|ui| {
        for &region_id in &region_ids {
            let Some(state) = app
                .core
                .model
                .floor_assignment_region(region_id)
                .map(|r| r.assignment)
            else {
                continue;
            };
            ui.group(|ui| {
                ui.label(format!("領域 R{}", region_id.0));
                if let squid_n_core::model::PlateAssignment::Plate(sid) = state {
                    ui.label(format!("床板 #{}", sid.0));
                } else if ui.button("この仕様で割当").clicked() {
                    pending_assign.push(region_id);
                }
                if !state.is_no_plate() && ui.button("版なし").clicked() {
                    pending_no_plate.push(region_id);
                }
                if !state.is_unset() && ui.button("未設定へ戻す").clicked() {
                    pending_unset.push(region_id);
                }
            });
        }
    });
    let pending =
        !pending_assign.is_empty() || !pending_no_plate.is_empty() || !pending_unset.is_empty();
    for &region in &pending_assign {
        app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(AssignSlabToFloorPlateRegion {
                region,
                plate: plate.clone(),
            }),
        );
    }
    for &region in &pending_no_plate {
        app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(SetFloorPlateRegionNoPlate { region }),
        );
    }
    for &region in &pending_unset {
        app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(UnsetFloorPlateRegion { region }),
        );
    }
    if pending {
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
    let span_ok = squid_n_core::model::span_is_valid(span);
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
                    if s != span && squid_n_core::model::span_is_valid(s) {
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
