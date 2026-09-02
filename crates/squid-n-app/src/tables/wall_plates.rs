//! 壁版（`Model.wall_plates` = [`WallPlate`]: 断面・開口・耐震スリット・取付き先）
//! の編集 UI。
//!
//! 壁の入力の正は壁版であり、解析用の壁要素（`ElementKind::Wall`）はモデルに
//! 保存せず準備計算・解析の直前に生成する（D5）。したがってこのタブは
//! `Model.elements` ではなく `Model.wall_plates` を対象にする。
//!
//! 編集は `squid_n_edit::{SetWallPlateSection, SetWallPlateAttrs,
//! SetAttachedWallPlateAnchor, SetAttachedWallPlateExtent, AddAttachedWallPlate,
//! AddEnclosedWallPlate, DeleteWallPlate}` 経由（undo 対応）。
//! 併せて、建物一律の複数開口の取り扱い（`Model.multi_opening_mode`）を
//! `squid_n_edit::SetMultiOpeningMode` 経由で編集する（undo 対応）。
//!
//! # 断面は即時反映、開口はフォーム＋適用
//!
//! 断面の割当は選択肢からの単純な差し替えなので、床板（[`crate::tables::slabs`]）
//! と同じく一覧のセルで即時反映する。開口（`opening_area` と `openings` は
//! 相互依存があり、入力途中に不正な文字列を経由する）だけはフォームへ集め、
//! 「適用」で 1 つの `SetWallPlateAttrs` として発行する。

use crate::app::App;
use squid_n_core::ids::{NodeId, SectionId, WallPlateId};
use squid_n_core::model::{
    AreaLoad, LoadTransfer, MultiOpeningMode, RegionAnchor, WallOpening, WallPlate, WallPlateShape,
};
use squid_n_core::units::{to_display, to_internal};
use squid_n_edit::{
    AddAttachedWallPlate, AddEnclosedWallPlate, DeleteWallPlate, SetAttachedWallPlateAnchor,
    SetAttachedWallPlateExtent, SetMultiOpeningMode, SetWallPlateAttrs, SetWallPlateSection,
};

/// 複数開口の取り扱い（`MultiOpeningMode`）の選択肢一覧（UI 表示順）。
const MULTI_OPENING_MODES: [MultiOpeningMode; 3] = [
    MultiOpeningMode::Equivalent,
    MultiOpeningMode::Envelope,
    MultiOpeningMode::Auto,
];

/// `MultiOpeningMode` の表示ラベル（RC規準「耐震壁の開口」の用語）。
fn multi_opening_mode_label(mode: MultiOpeningMode) -> &'static str {
    match mode {
        MultiOpeningMode::Equivalent => "等価開口とする",
        MultiOpeningMode::Envelope => "包絡する",
        MultiOpeningMode::Auto => "包絡開口・等価開口自動判定",
    }
}

/// 壁版フォームのドラフト状態（GUI 専用）。
///
/// 上段は既存の壁版を編集するフォーム（`target` を選ぶと `synced_for` の壁版の
/// 現在値でバッファを初期化し、「適用」で `SetWallPlateAttrs` を発行する）。
/// 下段の `add_*` は「取り付く壁版を追加」フォームの入力欄。
#[derive(Clone, Debug, Default)]
pub struct WallPlateDraft {
    /// 編集対象の壁版。
    pub target: Option<WallPlateId>,
    /// バッファを初期化した対象（`target` と異なれば model 値で再同期する）。
    pub synced_for: Option<WallPlateId>,
    /// 開口面積 [mm²] の入力バッファ（`openings` が空の場合のみ有効）。
    pub opening_area: String,
    /// 開口部重量 [N] の入力バッファ。
    pub opening_weight: String,
    /// 耐震スリット（4 辺）。柱際の添字は `WallPlate::column_face_nodes` の並び。
    pub slit: squid_n_core::model::WallSlit,
    /// 個別開口寸法の入力バッファ。1行1開口または「,」区切りで
    /// `幅x高さ` または `幅x高さ@x,z`（位置指定付き）を入力する。
    /// 空文字列は「個別開口なし（`opening_area` を使用）」を表す。
    pub openings: String,
    /// 仕上げ・増打ちの面荷重 [kN/m²] の入力バッファ。
    pub load_value: String,
    /// 仕上げ・増打ちの面荷重の名称（空欄は「仕上げ」）。
    pub load_kind: String,

    /// 追加フォーム: 取付き先を床領域（自立壁）にするか。false は線（大梁・柱頭）。
    pub add_to_floor_region: bool,
    /// 追加フォーム: 取付き線の両端、または自立壁の始点・終点。
    pub add_nodes: [Option<NodeId>; 2],
    /// 追加フォーム: 立ち上がり高さ [mm]（始端側・終端側。負なら垂れ壁）。
    pub add_extent: [String; 2],
    /// 追加フォーム: 高さを階高いっぱいにするか（自立壁のみ）。
    pub add_story_height: bool,
    /// 追加フォーム: 取付き線に載る壁版の荷重の出口。
    pub add_transfer: LoadTransfer,
    /// 追加フォーム: 取付き線上の無次元区間 `[t_i, t_j]`。床領域アンカーでは使わない。
    pub add_span: [f64; 2],
    /// 追加フォーム: 断面（板厚・材料）。
    pub add_section: Option<SectionId>,

    /// 囲まれた壁版追加フォーム: 境界 4 節点（反時計回り）。
    pub add_enclosed_nodes: [Option<NodeId>; 4],
    /// 囲まれた壁版追加フォーム: 断面。
    pub add_enclosed_section: Option<SectionId>,
}

/// 個別開口の入力バッファ1件分の書式エラー。
/// `parse_openings` が返すメッセージには不正箇所の文字列を含める。
fn parse_single_opening(entry: &str) -> Result<WallOpening, String> {
    let (dims, offset) = match entry.split_once('@') {
        Some((d, o)) => (d, Some(o)),
        None => (entry, None),
    };
    let (w_str, h_str) = dims
        .split_once(['x', 'X'])
        .ok_or_else(|| format!("不正な開口指定「{entry}」: '幅x高さ' 形式で入力してください"))?;
    let width: f64 = w_str
        .trim()
        .parse()
        .map_err(|_| format!("不正な幅「{}」: 数値ではありません", w_str.trim()))?;
    let height: f64 = h_str
        .trim()
        .parse()
        .map_err(|_| format!("不正な高さ「{}」: 数値ではありません", h_str.trim()))?;
    if width <= 0.0 || height <= 0.0 {
        return Err(format!(
            "開口寸法「{entry}」: 幅・高さは正の値で入力してください"
        ));
    }
    let offset = match offset {
        Some(o) => {
            let (x_str, z_str) = o
                .split_once(',')
                .ok_or_else(|| format!("不正な位置指定「{o}」: 'x,z' 形式で入力してください"))?;
            let x: f64 = x_str
                .trim()
                .parse()
                .map_err(|_| format!("不正な位置x「{}」: 数値ではありません", x_str.trim()))?;
            let z: f64 = z_str
                .trim()
                .parse()
                .map_err(|_| format!("不正な位置z「{}」: 数値ではありません", z_str.trim()))?;
            Some([x, z])
        }
        None => None,
    };
    Ok(WallOpening {
        width,
        height,
        offset,
    })
}

/// 個別開口入力バッファ（1行1開口または「,」区切り、`幅x高さ` / `幅x高さ@x,z`）を
/// パースする（egui 非依存の純関数）。
///
/// 開口の位置指定 `@x,z` 自体がカンマを含むため、単純な「,」split だけでは
/// 「幅x高さ」を含まないトークン（位置の z 座標）を直前のトークンへ結合することで
/// 「,」区切りの開口列と「@x,z」内の「,」を区別する。
pub fn parse_openings(s: &str) -> Result<Vec<WallOpening>, String> {
    let normalized = s.replace('\n', ",");
    let mut entries: Vec<String> = Vec::new();
    for token in normalized.split(',') {
        let token = token.trim();
        if token.is_empty() {
            continue;
        }
        if token.contains(['x', 'X']) {
            entries.push(token.to_string());
        } else {
            match entries.last_mut() {
                Some(last) => {
                    last.push(',');
                    last.push_str(token);
                }
                None => {
                    return Err(format!(
                        "不正な開口指定「{token}」: '幅x高さ' 形式で入力してください"
                    ));
                }
            }
        }
    }
    entries.iter().map(|e| parse_single_opening(e)).collect()
}

/// 個別開口リストを入力バッファ書式（1行1開口、`幅x高さ` または `幅x高さ@x,z`）へ
/// 整形する（`parse_openings` の逆変換）。既存値をフォームへ読み込む際に使用する。
pub fn format_openings(openings: &[WallOpening]) -> String {
    openings
        .iter()
        .map(|o| match o.offset {
            Some([x, z]) => format!("{}x{}@{},{}", o.width, o.height, x, z),
            None => format!("{}x{}", o.width, o.height),
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// 壁版の種別ラベル。
fn shape_label(plate: &WallPlate) -> &'static str {
    match plate.shape {
        WallPlateShape::Enclosed { .. } => "囲まれた",
        WallPlateShape::Attached { .. } => "取り付く",
    }
}

/// 耐震スリットの一覧表示（切れている辺の名前）。切れていなければ `None`。
///
/// 柱際はどちら側かを節点番号で示す。壁版は左右を持つが、図を見ずに「1 側／2 側」
/// とだけ書かれても利用者はどちらか判別できないためである。梁際は下辺・上辺と示す。
fn slit_label(plate: &WallPlate, model: &squid_n_core::model::Model) -> Option<String> {
    if !plate.slit.any() {
        return None;
    }
    let faces = plate.column_face_nodes(model);
    let mut names: Vec<String> = (0..2)
        .filter(|&k| plate.slit.column_face[k])
        .map(|k| match faces {
            Some(nodes) => format!("柱際N{}", nodes[k].0),
            // 境界が 4 節点でない等で節点を引けない場合は添字だけ示す。
            None => format!("柱際{}", k + 1),
        })
        .collect();
    for (k, name) in ["下辺", "上辺"].into_iter().enumerate() {
        if plate.slit.beam_face[k] {
            names.push(name.to_string());
        }
    }
    Some(format!("あり({})", names.join("・")))
}

/// 壁エレメントが生成されない理由（「解析要素」列のホバー）。
///
/// 生成の可否そのものは `Model::wall_plate_becomes_element` が単独で決める。ここは
/// **理由の言い換えだけ**を担い、可否の判定を組み立て直さない（同じ問いを 2 か所で
/// 実装すると、列の表示と実際の生成が食い違う）。
fn not_element_reason(plate: &WallPlate, model: &squid_n_core::model::Model) -> &'static str {
    if plate.is_attached() {
        return "取り付く壁版（腰壁・垂壁・パラペット・自立壁）は解析要素にしません。\
                自重の分配と周辺部材への剛性算入は行います";
    }
    if !model
        .wall_regions
        .iter()
        .any(|r| r.wall_plate_ids.contains(&plate.id))
    {
        return "どの壁領域にも帰属していません。準備計算が構面を検出できていないか、\
                壁版が構面の平面から外れています。自重だけを持つ壁版になります";
    }
    if !model.wall_plate_covers_region(plate) {
        return "壁領域全体を覆う 4 節点ではありません（間柱で分割されている、\
                または壁領域の境界が 5 節点以上）。自重だけを持つ壁版になります";
    }
    "断面が割り当たっていません。板厚・材料が引けないため要素を組み立てられません"
}

/// 開口の要約文字列（一覧表示用）。
fn opening_summary(plate: &WallPlate) -> String {
    if plate.openings.is_empty() {
        if plate.opening_area > 0.0 {
            format!("{:.3e} mm²", plate.opening_area)
        } else {
            "―".to_string()
        }
    } else {
        format!(
            "{}個 Σ{:.3e} mm²",
            plate.openings.len(),
            plate.total_opening_area()
        )
    }
}

pub fn wall_plates_table(ui: &mut egui::Ui, app: &mut App) {
    ui.label(
        "壁版は、柱・梁が囲む鉛直構面内の版（囲まれた壁版）、または主架構・床領域に\
         取り付く版（取り付く壁版＝パラペット・腰壁・垂れ壁・自立壁）です。板厚と材料は\
         断面から決まり、断面が未割当の壁版は自重が 0 になります（解析前チェックが止めます）。\
         解析用の壁要素はモデルに保存せず、準備計算・解析の直前に壁版から生成します。",
    );
    ui.separator();

    // ── 複数開口の取り扱い（建物一律） ─────────────────────────
    ui.horizontal(|ui| {
        ui.label("複数開口の取り扱い(建物一律):");
        let current = app.core.model.multi_opening_mode;
        let combo = egui::ComboBox::from_id_salt("multi_opening_mode")
            .selected_text(multi_opening_mode_label(current))
            .show_ui(ui, |ui| {
                for mode in MULTI_OPENING_MODES {
                    if ui
                        .selectable_label(current == mode, multi_opening_mode_label(mode))
                        .clicked()
                        && current != mode
                    {
                        app.core.scoped.undo
                            .run(&mut app.core.model, Box::new(SetMultiOpeningMode { mode }));
                        app.core.scoped.staleness.mark_edited();
                    }
                }
            });
        combo
            .response
            .on_hover_text("自動判定は開口間距離 l が l<1.5h または l<1m のとき包絡開口とみなします(h: 包絡開口とした場合の高さ。RC規準「耐震壁の開口」)。");
    });
    ui.label(
        "このモードは剛性の開口低減・耐震壁判定・検定の開口評価に適用されます\
         （自重控除は常に実開口面積を用います）。",
    );
    ui.separator();

    wall_plates_list(ui, app);
    attrs_form(ui, app);
    add_enclosed_form(ui, app);
    add_attached_form(ui, app);
}

/// 壁版一覧。断面と取付き先（取り付く壁版のみ）はセル内で即時反映する。
fn wall_plates_list(ui: &mut egui::Ui, app: &mut App) {
    use crate::table_util::{self, Col};

    ui.strong("壁版");
    if app.core.model.wall_plates.is_empty() {
        ui.label(
            "壁版がありません（ST-Bridge の取り込み、または下の「囲まれた壁版を追加」\
             「取り付く壁版を追加」で作ります）。",
        );
        return;
    }

    let mut pending_section: Vec<(WallPlateId, Option<SectionId>)> = Vec::new();
    let mut pending_extent: Vec<(WallPlateId, Option<[f64; 2]>)> = Vec::new();
    let mut pending_anchor: Vec<(WallPlateId, RegionAnchor)> = Vec::new();
    let mut pending_delete: Option<WallPlateId> = None;
    let node_ids: Vec<NodeId> = app.core.model.nodes.iter().map(|n| n.id).collect();
    // 板状の断面（板厚を持つ断面）だけを候補にする。板厚が無い断面を割り当てても
    // 自重・数量が算定できないため、選ばせない（床板の断面欄と同じ規約）。
    let wall_sections: Vec<(SectionId, String)> = app
        .core
        .model
        .sections
        .iter()
        .filter(|sec| sec.thickness.is_some_and(|t| t > 0.0))
        .map(|sec| (sec.id, sec.display_name()))
        .collect();

    table_util::standard_table(
        ui,
        "wall_plates_tbl",
        &[
            Col::id(),
            Col::label("種別"),
            Col::label("解析要素").hover(
                "壁エレメント（耐震壁）が生成される壁版か。生成されない壁版は自重だけを持つ",
            ),
            Col::text("所属壁領域"),
            Col::text("境界 / 取付き先"),
            Col::text("断面"),
            Col::num("面積[m²]").hover("境界多角形の生の面積（開口・柱梁の内法による低減前）"),
            Col::text("開口"),
            Col::text("仕上げ・増打ち").hover(
                "壁の重さに加える面荷重。躯体の自重は断面から別に求める。増打ちは構造厚に含めないため剛性・耐力には算入しない",
            ),
            Col::label("スリット").hover(
                "縁を切った辺。切れている辺の部材へは剛性算入せず自重も伝えない。4 辺すべてが一体でなければ耐震壁として成立しない",
            ),
            Col::actions(),
        ],
        app.core.model.wall_plates.len(),
        |row| {
            let i = row.index();
            let plate = &app.core.model.wall_plates[i];
            row.col(|ui| {
                table_util::id_label(ui, plate.id.0);
            });
            row.col(|ui| {
                table_util::text_cell(ui, shape_label(plate));
            });
            row.col(|ui| {
                if app.core.model.wall_plate_becomes_element(plate) {
                    ui.label("壁エレメント").on_hover_text(
                        "壁領域全体を覆う 4 節点で断面も割り当たっているため、解析要素\
                         （耐震壁）を生成します",
                    );
                } else {
                    table_util::muted_cell(ui, "―", not_element_reason(plate, &app.core.model));
                }
            });
            row.col(|ui| {
                // どの壁領域（柱・梁の区画）に属するかは `wall_plate_ids` から逆引きする
                // （取り付く壁版はどの壁領域からも参照されない）。
                let owner = app
                    .core.model
                    .wall_regions
                    .iter()
                    .find(|r| r.wall_plate_ids.contains(&plate.id));
                match owner {
                    Some(r) if !r.name.is_empty() => table_util::text_cell(ui, &r.name),
                    Some(r) => table_util::text_cell(ui, &format!("#{}", r.id.0)),
                    None => table_util::muted_cell(
                        ui,
                        "―",
                        "どの壁領域からも参照されていません（取り付く壁版、または帰属なし）",
                    ),
                }
            });
            // 「階高いっぱい」の壁版の高さは、階レベルから解決した値を見せる。
            let resolved_extent = app.core.model.wall_plate_extent(plate);
            row.col(|ui| match &plate.shape {
                WallPlateShape::Enclosed { boundary } => {
                    let s = boundary
                        .iter()
                        .map(|n| n.0.to_string())
                        .collect::<Vec<_>>()
                        .join("-");
                    table_util::text_cell(ui, &s);
                }
                WallPlateShape::Attached { anchor, extent } => {
                    attached_anchor_cell(
                        ui,
                        plate.id,
                        *anchor,
                        *extent,
                        resolved_extent,
                        &node_ids,
                        &mut pending_extent,
                        &mut pending_anchor,
                    );
                }
            });
            row.col(|ui| {
                let label = app
                    .core.model
                    .wall_plate_section(plate)
                    .map(|sec| sec.display_name())
                    .unwrap_or_else(|| "―".to_string());
                table_util::cell_combo(ui, ("wall_plate_section", plate.id.0), &label, |ui| {
                    if ui.selectable_label(plate.section.is_none(), "―").clicked()
                        && plate.section.is_some()
                    {
                        pending_section.push((plate.id, None));
                    }
                    for (sid, name) in &wall_sections {
                        if ui
                            .selectable_label(plate.section == Some(*sid), name)
                            .clicked()
                            && plate.section != Some(*sid)
                        {
                            pending_section.push((plate.id, Some(*sid)));
                        }
                    }
                });
            });
            row.col(|ui| {
                // 面積は m² 表示（mm² のままでは桁が読めない）。
                ui.label(format!("{:.2}", plate.area(&app.core.model) / 1.0e6));
            });
            row.col(|ui| {
                table_util::text_cell(ui, &opening_summary(plate));
            });
            row.col(|ui| {
                let s = plate
                    .loads
                    .iter()
                    .map(|l| {
                        format!("{} {:.2}kN/m²", l.kind, to_display::area_load_kn_per_m2(l.value))
                    })
                    .collect::<Vec<_>>()
                    .join(", ");
                if s.is_empty() {
                    table_util::muted_cell(ui, "―", "仕上げ・増打ちの面荷重が登録されていません");
                } else {
                    table_util::text_cell(ui, &s);
                }
            });
            row.col(|ui| {
                // 耐震スリットは、柱・梁と接する 4 辺を持つ囲まれた壁版でだけ意味を持つ。
                // 取り付く壁版（腰壁・垂れ壁・パラペット・自立壁）はその辺を持たない
                // ため、値の有無を問わず「―」とする。
                if plate.is_attached() {
                    table_util::muted_cell(
                        ui,
                        "―",
                        "取り付く壁版（腰壁・垂れ壁・パラペット・自立壁）は、\
                         柱・梁と接する 4 辺を持たないため耐震スリットの指定はありません",
                    );
                } else {
                    match slit_label(plate, &app.core.model) {
                        Some(label) => {
                            ui.label(label);
                        }
                        None => table_util::text_cell(ui, "なし"),
                    }
                }
            });
            row.col(|ui| {
                if table_util::delete_cell(ui, "この壁版を削除", None) {
                    pending_delete = Some(plate.id);
                }
            });
        },
    );

    let had_pending = !pending_section.is_empty()
        || !pending_extent.is_empty()
        || !pending_anchor.is_empty()
        || pending_delete.is_some();
    for (id, section) in pending_section {
        app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(SetWallPlateSection { id, section }),
        );
    }
    for (id, extent) in pending_extent {
        app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(SetAttachedWallPlateExtent { id, extent }),
        );
    }
    for (id, anchor) in pending_anchor {
        app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(SetAttachedWallPlateAnchor { id, anchor }),
        );
    }
    if let Some(id) = pending_delete {
        app.core
            .scoped
            .undo
            .run(&mut app.core.model, Box::new(DeleteWallPlate { id }));
        // 削除は後続の壁版 ID を 1 つずつ繰り上げるため、フォームの対象を
        // そのまま残すと「別の壁版を編集していた」ことになる。対象を外す。
        app.ui.scoped.wall_plate_draft.target = None;
        app.ui.scoped.wall_plate_draft.synced_for = None;
    }
    if had_pending {
        app.core.scoped.staleness.mark_edited();
    }
}

/// 取り付く壁版の取付き先・立ち上がり高さのセル（一覧内で即時反映）。
/// 床板の `slabs::attached_boundary_cell` と同じ流儀。
#[allow(clippy::too_many_arguments)]
fn attached_anchor_cell(
    ui: &mut egui::Ui,
    id: WallPlateId,
    anchor: RegionAnchor,
    extent: Option<[f64; 2]>,
    resolved_extent: Option<[f64; 2]>,
    node_ids: &[NodeId],
    pending_extent: &mut Vec<(WallPlateId, Option<[f64; 2]>)>,
    pending_anchor: &mut Vec<(WallPlateId, RegionAnchor)>,
) {
    ui.horizontal(|ui| {
        match anchor {
            RegionAnchor::Line {
                nodes,
                span,
                transfer,
            } => {
                for k in 0..2 {
                    let mut sel = nodes[k];
                    egui::ComboBox::from_id_salt(("wp_anc", id.0, k))
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
                ui.label("区間:");
                let mut s = span;
                ui.add(egui::DragValue::new(&mut s[0]).range(0.0..=1.0).speed(0.01));
                ui.label("〜");
                ui.add(egui::DragValue::new(&mut s[1]).range(0.0..=1.0).speed(0.01));
                // `Model::validate`（squid-n-core）・`wall_anchor_ok`（squid-n-edit）と同じ範囲。
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
                let mut t = transfer;
                egui::ComboBox::from_id_salt(("wp_tr", id.0))
                    .selected_text(transfer_label(transfer))
                    .show_ui(ui, |ui| {
                        for cand in [LoadTransfer::Anchor, LoadTransfer::Columns] {
                            ui.selectable_value(&mut t, cand, transfer_label(cand));
                        }
                    });
                if t != transfer {
                    pending_anchor.push((
                        id,
                        RegionAnchor::Line {
                            nodes,
                            span,
                            transfer: t,
                        },
                    ));
                }
            }
            // 自立壁。荷重を渡す床領域は保存せず壁の位置から都度求めるため、
            // ここで編集するのは壁自身の始点・終点だけである
            // （`RegionAnchor::FloorRegion` のドキュメント）。
            RegionAnchor::FloorRegion { nodes } => {
                ui.label("自立(始点/終点):");
                for k in 0..2 {
                    let mut sel = nodes[k];
                    egui::ComboBox::from_id_salt(("wp_fr_n", id.0, k))
                        .selected_text(format!("N{}", sel.0))
                        .show_ui(ui, |ui| {
                            for &nid in node_ids {
                                ui.selectable_value(&mut sel, nid, format!("N{}", nid.0));
                            }
                        });
                    if sel != nodes[k] && sel != nodes[1 - k] {
                        let mut n = nodes;
                        n[k] = sel;
                        pending_anchor.push((id, RegionAnchor::FloorRegion { nodes: n }));
                    }
                }
            }
            // 壁の取付き先には使わない（`RegionAnchor::Point` は出隅スラブ専用。
            // `squid-n-edit::wall_anchor_ok` が弾くため、この分岐へは到達しない）。
            RegionAnchor::Point(_) => {
                ui.label("(未対応の取付き先)");
            }
        }
        height_cell(ui, id, anchor, extent, resolved_extent, pending_extent);
    });
}

/// 立ち上がり高さの入力欄。
///
/// 自立壁だけは「階高」（`extent = None`）を選べる。階高は設計中に何度も変わるので、
/// 全高の壁を絶対寸法で書くと変更に追随せず、上階との間に隙間が残るためである。
/// 取付き線の壁版で階高を許すと、階高分の腰壁せいが取付き先の梁へ丸ごと算入されて
/// 剛性を過大に見るので、チェックボックス自体を出さない
/// （`WallPlateShape::Attached` のドキュメント）。
fn height_cell(
    ui: &mut egui::Ui,
    id: WallPlateId,
    anchor: RegionAnchor,
    extent: Option<[f64; 2]>,
    resolved_extent: Option<[f64; 2]>,
    pending_extent: &mut Vec<(WallPlateId, Option<[f64; 2]>)>,
) {
    ui.label("高さ:");
    if matches!(anchor, RegionAnchor::FloorRegion { .. }) {
        let mut story_height = extent.is_none();
        if ui
            .checkbox(&mut story_height, "階高")
            .on_hover_text("壁の下端から直上の階レベルまでを高さにする。階高を変えても追随する")
            .changed()
        {
            // 階高を外したときは、直前まで表示していた解決後の高さを初期値にする
            // （0 へ落とすと壁が消えたように見える）。
            let seed = resolved_extent.unwrap_or([0.0, 0.0]);
            pending_extent.push((id, (!story_height).then_some(seed)));
        }
        if story_height {
            match resolved_extent {
                Some(e) => {
                    ui.label(format!("{:.0} mm", e[0]));
                }
                // 直上に階が無い壁。解析前チェックがエラーで止める。
                None => {
                    ui.colored_label(crate::theme::WARN_TEXT, "階高が引けません");
                }
            }
            return;
        }
    }
    let Some(mut e) = extent else {
        return;
    };
    let before = e;
    ui.add(egui::DragValue::new(&mut e[0]).suffix(" mm"));
    ui.add(egui::DragValue::new(&mut e[1]).suffix(" mm"));
    if e != before && e[0].is_finite() && e[1].is_finite() {
        pending_extent.push((id, Some(e)));
    }
}

/// 仕上げ・増打ちの面荷重 [kN/m²] の入力を読む。空欄は 0、数値でなければ `None`。
fn parse_area_load_kn_per_m2(text: &str) -> Option<f64> {
    let t = text.trim();
    if t.is_empty() {
        return Some(0.0);
    }
    t.parse::<f64>().ok().filter(|v| v.is_finite())
}

fn transfer_label(t: LoadTransfer) -> &'static str {
    match t {
        LoadTransfer::Anchor => "取付き線へ分布",
        LoadTransfer::Columns => "両端の柱へ集中",
    }
}

/// 開口・耐震スリットの編集フォーム（対象を選び、「適用」で 1 コマンド発行）。
fn attrs_form(ui: &mut egui::Ui, app: &mut App) {
    ui.separator();
    ui.strong("開口・耐震スリットを設定");

    if app.core.model.wall_plates.is_empty() {
        return;
    }

    let plate_ids: Vec<WallPlateId> = app.core.model.wall_plates.iter().map(|p| p.id).collect();
    // 対象が消えている（削除・ID 繰り上げ）場合は未選択へ戻す。
    if app
        .ui
        .scoped
        .wall_plate_draft
        .target
        .is_some_and(|t| !plate_ids.contains(&t))
    {
        app.ui.scoped.wall_plate_draft.target = None;
        app.ui.scoped.wall_plate_draft.synced_for = None;
    }

    ui.horizontal(|ui| {
        ui.label("対象の壁版:");
        let text = app
            .ui
            .scoped
            .wall_plate_draft
            .target
            .map(|p| format!("壁版#{}", p.0))
            .unwrap_or_else(|| "―".to_string());
        egui::ComboBox::from_id_salt("wall_plate_target")
            .selected_text(text)
            .show_ui(ui, |ui| {
                for &pid in &plate_ids {
                    let kind = app
                        .core
                        .model
                        .wall_plate(pid)
                        .map(shape_label)
                        .unwrap_or_default();
                    if ui
                        .selectable_label(
                            app.ui.scoped.wall_plate_draft.target == Some(pid),
                            format!("壁版#{}（{kind}）", pid.0),
                        )
                        .clicked()
                    {
                        app.ui.scoped.wall_plate_draft.target = Some(pid);
                    }
                }
            });
    });

    // 対象が変わったら model の現在値でバッファを再同期する。
    if app.ui.scoped.wall_plate_draft.target != app.ui.scoped.wall_plate_draft.synced_for {
        if let Some(plate) = app
            .ui
            .scoped
            .wall_plate_draft
            .target
            .and_then(|pid| app.core.model.wall_plate(pid))
        {
            let (area, weight, slit, openings) = (
                plate.opening_area,
                plate.opening_weight,
                plate.slit,
                plate.openings.clone(),
            );
            app.ui.scoped.wall_plate_draft.opening_area = format!("{area:.0}");
            app.ui.scoped.wall_plate_draft.opening_weight = format!("{weight:.0}");
            app.ui.scoped.wall_plate_draft.slit = slit;
            app.ui.scoped.wall_plate_draft.openings = format_openings(&openings);
            // 面荷重は 1 件だけ扱う（床板の追加フォームと同じ簡略化）。複数件を
            // 持つ壁版は合計値を見せ、適用すると 1 件へまとめる。
            let total = plate.finish_intensity();
            app.ui.scoped.wall_plate_draft.load_value =
                format!("{:.3}", to_display::area_load_kn_per_m2(total));
            app.ui.scoped.wall_plate_draft.load_kind = plate
                .loads
                .first()
                .map(|l| l.kind.clone())
                .unwrap_or_else(|| "仕上げ".to_string());
            app.ui.scoped.wall_plate_draft.synced_for = app.ui.scoped.wall_plate_draft.target;
        }
    }

    let Some(target) = app.ui.scoped.wall_plate_draft.target else {
        ui.label("編集する壁版を選んでください。");
        return;
    };
    let is_attached = app
        .core
        .model
        .wall_plate(target)
        .is_some_and(|p| p.is_attached());

    ui.horizontal(|ui| {
        ui.label("開口面積[mm²]:");
        ui.add(
            egui::TextEdit::singleline(&mut app.ui.scoped.wall_plate_draft.opening_area)
                .desired_width(90.0),
        );
        ui.label("開口部重量[N]:");
        ui.add(
            egui::TextEdit::singleline(&mut app.ui.scoped.wall_plate_draft.opening_weight)
                .desired_width(90.0),
        );
    });

    ui.horizontal(|ui| {
        ui.label("仕上げ・増打ち:");
        ui.add(egui::TextEdit::singleline(&mut app.ui.scoped.wall_plate_draft.load_kind).desired_width(80.0));
        ui.add(
            egui::TextEdit::singleline(&mut app.ui.scoped.wall_plate_draft.load_value).desired_width(70.0),
        );
        ui.label("kN/m²");
    })
    .response
    .on_hover_text(
        "コンクリートの増打ち・仕上げの重さを面荷重として加えます。躯体の自重は         断面の板厚と材料から別に求めるため、ここには含めません。増打ちは構造厚に         含めない扱いなので、剛性・耐力には算入しません。",
    );

    // 耐震スリットは囲まれた壁版にしか意味がない（一覧の同名列と同じ理由）。
    // 取り付く壁版では入力欄自体を出さない。左右どちらの柱際かは節点番号で示す。
    if !is_attached {
        // スリットは辺の役割（柱際か梁際か、下辺か上辺か）を決められる 4 節点の
        // 壁版でのみ扱える。効かない壁版では入力を出さず、理由を示す。
        let quad = app
            .core
            .model
            .wall_plate(target)
            .is_some_and(|p| p.has_quad_boundary());
        let faces = app
            .core
            .model
            .wall_plate(target)
            .and_then(|p| p.column_face_nodes(&app.core.model));
        ui.horizontal(|ui| {
            ui.label("耐震スリット:");
            // 4 節点でない壁版では指定しても効かないため、入力を無効化する。
            ui.add_enabled_ui(quad, |ui| {
                for k in 0..2 {
                    let label = match faces {
                        Some(nodes) => format!("柱際 N{}", nodes[k].0),
                        None => format!("柱際 {}", k + 1),
                    };
                    ui.checkbox(
                        &mut app.ui.scoped.wall_plate_draft.slit.column_face[k],
                        label,
                    )
                    .on_hover_text(
                        "その柱との縁を切ります。切れている側へは袖壁として剛性算入しません",
                    );
                }
                for (k, label) in ["下辺", "上辺"].into_iter().enumerate() {
                    ui.checkbox(&mut app.ui.scoped.wall_plate_draft.slit.beam_face[k], label)
                        .on_hover_text(
                            "その梁との縁を切ります。切れている側へは腰壁・垂れ壁として\
                             剛性算入せず、自重も伝えません",
                        );
                }
            });
        });
        if !quad {
            ui.label(
                egui::RichText::new(
                    "耐震スリットは境界が 4 節点の壁版でのみ指定できます\
                     （辺が柱際か梁際かを決められないため）。",
                )
                .color(crate::theme::GRAY_600)
                .small(),
            );
        }
        if app.ui.scoped.wall_plate_draft.slit.both_beam_faces() {
            ui.colored_label(
                crate::theme::ERROR_RED,
                "上下の梁際をともに切ると自重の伝達先がなくなります（解析前チェックが止めます）",
            );
        }
        ui.label(
            egui::RichText::new(
                "三方スリットは柱際 2 辺と上下いずれか 1 辺、完全スリットは 4 辺を切った状態です。\
                 4 辺すべてが一体でなければ耐震壁として成立しません。",
            )
            .color(crate::theme::GRAY_600)
            .small(),
        );
    }

    ui.label("個別開口（任意・耐震壁判定/剛性計算の複数開口寸法）:");
    ui.add(
        egui::TextEdit::multiline(&mut app.ui.scoped.wall_plate_draft.openings)
            .desired_rows(3)
            .desired_width(320.0)
            .hint_text(
                "幅x高さ または 幅x高さ@x,z（位置指定）\n\
                 複数開口は改行または「,」区切りで入力\n\
                 例: 1000x2000, 800x900@3000,500",
            ),
    )
    .on_hover_text(
        "1行1開口または「,」区切りで '幅x高さ' もしくは位置付き '幅x高さ@x,z' を入力します。\
         空欄の場合は開口面積[mm²]の入力値がそのまま使われます。",
    );

    let parsed_openings = parse_openings(&app.ui.scoped.wall_plate_draft.openings);
    match &parsed_openings {
        Ok(openings) if !openings.is_empty() => {
            let sum_area: f64 = openings.iter().map(WallOpening::area).sum();
            ui.label(format!(
                "個別開口 {}個 Σ{:.2e} mm²（開口面積[mm²]の入力値は無視され、\
                 個別開口の面積和が優先されます）",
                openings.len(),
                sum_area
            ));
        }
        Ok(_) => {}
        Err(e) => {
            ui.colored_label(
                crate::theme::ERROR_RED,
                format!("個別開口の書式エラー: {e}"),
            );
        }
    }

    let parsed_area = app
        .ui
        .scoped
        .wall_plate_draft
        .opening_area
        .trim()
        .parse::<f64>();
    let parsed_weight = app
        .ui
        .scoped
        .wall_plate_draft
        .opening_weight
        .trim()
        .parse::<f64>();
    let parsed_load = parse_area_load_kn_per_m2(&app.ui.scoped.wall_plate_draft.load_value);
    if parsed_load.is_none() {
        ui.colored_label(
            crate::theme::ERROR_RED,
            "仕上げ・増打ちの面荷重は数値で入力してください（空欄は 0）",
        );
    }
    let can_apply = parsed_area.is_ok()
        && parsed_weight.is_ok()
        && parsed_openings.is_ok()
        && parsed_load.is_some();
    if ui
        .add_enabled(can_apply, egui::Button::new("✔ 適用"))
        .on_hover_text("選択した壁版に開口・スリットを設定します（undo可）")
        .clicked()
    {
        if let (Ok(opening_area), Ok(opening_weight), Ok(openings)) =
            (parsed_area, parsed_weight, parsed_openings)
        {
            // 取り付く壁版では入力欄を出さないため、既存値をそのまま書き戻す。
            let slit = if is_attached {
                app.core
                    .model
                    .wall_plate(target)
                    .map(|p| p.slit)
                    .unwrap_or_default()
            } else {
                app.ui.scoped.wall_plate_draft.slit
            };
            // 0 の面荷重は項目自体を持たせない（一覧が「―」になり、空の
            // 「仕上げ 0.00kN/m²」が並ばない）。
            let value = to_internal::area_load_kn_per_m2(parsed_load.unwrap_or(0.0));
            let kind = app.ui.scoped.wall_plate_draft.load_kind.trim();
            let kind = if kind.is_empty() { "仕上げ" } else { kind }.to_string();
            let loads = if value == 0.0 {
                Vec::new()
            } else {
                vec![AreaLoad { kind, value }]
            };
            app.core.scoped.undo.run(
                &mut app.core.model,
                Box::new(SetWallPlateAttrs {
                    id: target,
                    opening_area,
                    opening_weight,
                    openings,
                    loads,
                    slit,
                }),
            );
            app.core.scoped.staleness.mark_edited();
        }
    }
}

/// 柱・梁で囲まれた壁版の追加フォーム。
///
/// 主架構に囲まれた壁版は、取り付く壁版と違って境界を節点 4 点で指定する。
/// 所属する壁領域は準備計算（`rebuild_wall_regions`）が自動で結びつける。
fn add_enclosed_form(ui: &mut egui::Ui, app: &mut App) {
    ui.separator();
    ui.strong("囲まれた壁版を追加（柱・梁で囲まれた構面内の版）");
    ui.label(
        "柱・梁が囲む鉛直構面内の壁版です。境界は 4 節点を反時計回りに指定します。\
         所属する壁領域は準備計算が自動で結びつけます。",
    );

    if app.core.model.nodes.len() < 4 {
        ui.label("囲まれた壁版を追加するには節点が4つ以上必要です");
        return;
    }

    let node_ids: Vec<NodeId> = app.core.model.nodes.iter().map(|n| n.id).collect();

    ui.label("境界節点（反時計回り。下辺 2 節点 → 上辺 2 節点の順を推奨）:");
    ui.horizontal_wrapped(|ui| {
        for (k, slot) in app
            .ui
            .scoped
            .wall_plate_draft
            .add_enclosed_nodes
            .iter_mut()
            .enumerate()
        {
            let text = slot
                .map(|n| format!("N{}", n.0))
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt(("wp_enc_node", k))
                .selected_text(format!("節点{}: {}", k, text))
                .show_ui(ui, |ui| {
                    ui.selectable_value(slot, None, "―");
                    for &nid in &node_ids {
                        ui.selectable_value(slot, Some(nid), format!("N{}", nid.0));
                    }
                });
        }
    });

    ui.horizontal(|ui| {
        ui.label("断面:");
        let resolved = app
            .ui
            .scoped
            .wall_plate_draft
            .add_enclosed_section
            .and_then(|sid| app.core.model.sections.get(sid.index()))
            .filter(|sec| sec.thickness.is_some_and(|t| t > 0.0));
        if resolved.is_none() {
            app.ui.scoped.wall_plate_draft.add_enclosed_section = None;
        }
        let label = resolved
            .map(|sec| sec.display_name())
            .unwrap_or_else(|| "―".to_string());
        egui::ComboBox::from_id_salt("wp_add_enclosed_section")
            .selected_text(label)
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    &mut app.ui.scoped.wall_plate_draft.add_enclosed_section,
                    None,
                    "―",
                );
                for sec in &app.core.model.sections {
                    if sec.thickness.is_some_and(|t| t > 0.0) {
                        ui.selectable_value(
                            &mut app.ui.scoped.wall_plate_draft.add_enclosed_section,
                            Some(sec.id),
                            sec.display_name(),
                        );
                    }
                }
            });
    })
    .response
    .on_hover_text("壁の板厚と自重は断面から決まります。断面が未割当の壁版は自重が 0 になります");

    let selected: Vec<NodeId> = app
        .ui
        .scoped
        .wall_plate_draft
        .add_enclosed_nodes
        .iter()
        .filter_map(|n| *n)
        .collect();
    let mut dedup = selected.clone();
    dedup.sort_by_key(|n| n.0);
    dedup.dedup();
    let can_add = selected.len() == 4 && dedup.len() == 4;

    if !can_add {
        ui.label("境界節点 4 点をすべて選び、重複がないようにしてください");
    }
    if ui
        .add_enabled(can_add, egui::Button::new("+ 囲まれた壁版を追加"))
        .on_hover_text("4 節点すべてが選択され、重複がない場合に追加できます")
        .clicked()
    {
        let boundary: Vec<NodeId> = app
            .ui
            .scoped
            .wall_plate_draft
            .add_enclosed_nodes
            .iter()
            .map(|n| n.expect("can_add で全スロット Some を確認済み"))
            .collect();
        app.core.scoped.undo.run(
            &mut app.core.model,
            Box::new(AddEnclosedWallPlate {
                boundary,
                section: app.ui.scoped.wall_plate_draft.add_enclosed_section,
                opening_area: 0.0,
                opening_weight: 0.0,
            }),
        );
        app.core.scoped.staleness.mark_edited();
    }
}

/// 取り付く壁版（パラペット・腰壁・垂れ壁・自立壁）の追加フォーム。
///
/// 主架構に囲まれない壁版は、囲まれた壁版と違って境界を節点で描けない。
/// 取付き先（大梁・柱頭の 2 節点、または床領域＋壁自身の始終点）と
/// 立ち上がり高さで作る。高さの符号は鉛直上向きが正で、負なら垂れ壁になる。
fn add_attached_form(ui: &mut egui::Ui, app: &mut App) {
    ui.separator();
    ui.strong("取り付く壁版を追加（パラペット・腰壁・垂れ壁・自立壁）");
    ui.label(
        "主架構に囲まれない壁版です。取付き先と立ち上がり高さで作ります。高さは鉛直上向きが正で、\
         負の値にすると垂れ壁（下向きの張り出し）になります。",
    );

    if app.core.model.nodes.len() < 2 {
        ui.label("取り付く壁版を追加するには節点が2つ以上必要です");
        return;
    }

    ui.horizontal(|ui| {
        ui.label("取付き先:");
        ui.selectable_value(
            &mut app.ui.scoped.wall_plate_draft.add_to_floor_region,
            false,
            "線（大梁・柱頭）",
        );
        ui.selectable_value(
            &mut app.ui.scoped.wall_plate_draft.add_to_floor_region,
            true,
            "床領域（自立壁）",
        )
        .on_hover_text(
            "床の上に立つ間仕切り等。自重は壁が載っている床領域（位置から都度解決）の床板へ等価な面荷重としてならします",
        );
    });

    let node_ids: Vec<NodeId> = app.core.model.nodes.iter().map(|n| n.id).collect();
    let to_region = app.ui.scoped.wall_plate_draft.add_to_floor_region;

    if to_region {
        if app.core.model.floor_regions.is_empty() {
            ui.label("床領域がありません（準備計算を実行すると主架構から生成されます）");
            return;
        }
        // 荷重を渡す床領域は選ばせない。壁の位置から都度求めるため保存しない
        // （`RegionAnchor::FloorRegion` のドキュメント）。床領域をまたぐ壁は
        // 内部で分割して配り、床に載らない壁は解析前チェックが止める。
        ui.label(
            "荷重は壁が載っている床領域の床板へ等価な面荷重としてならします（床領域をまたぐ壁は\
             境界で分割して配ります）。荷重を流せる床の上に無い壁は解析前チェックが止めます。",
        );
    }

    ui.horizontal(|ui| {
        let label = if to_region {
            ["壁の始点:", "壁の終点:"]
        } else {
            ["節点1:", "節点2:"]
        };
        for (k, text_label) in label.iter().enumerate() {
            ui.label(*text_label);
            let text = app.ui.scoped.wall_plate_draft.add_nodes[k]
                .map(|n| format!("N{}", n.0))
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt(("wp_add_node", k))
                .selected_text(text)
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        &mut app.ui.scoped.wall_plate_draft.add_nodes[k],
                        None,
                        "―",
                    );
                    for &nid in &node_ids {
                        ui.selectable_value(
                            &mut app.ui.scoped.wall_plate_draft.add_nodes[k],
                            Some(nid),
                            format!("N{}", nid.0),
                        );
                    }
                });
        }
    });

    // 「階高いっぱい」を選べるのは自立壁だけである（`WallPlateShape::Attached` の
    // ドキュメント）。取付き線に取り付く全高の壁は囲まれた壁版として入力する。
    let use_story_height = to_region && app.ui.scoped.wall_plate_draft.add_story_height;
    ui.horizontal(|ui| {
        if to_region {
            ui.checkbox(
                &mut app.ui.scoped.wall_plate_draft.add_story_height,
                "高さは階高",
            )
            .on_hover_text("壁の下端から直上の階レベルまでを高さにする。階高を変えても追随する");
        }
        ui.add_enabled_ui(!use_story_height, |ui| {
            ui.label("立ち上がり高さ 始端側 [mm]:");
            ui.add(
                egui::TextEdit::singleline(&mut app.ui.scoped.wall_plate_draft.add_extent[0])
                    .desired_width(70.0),
            );
            ui.label("終端側 [mm]:");
            ui.add(
                egui::TextEdit::singleline(&mut app.ui.scoped.wall_plate_draft.add_extent[1])
                    .desired_width(70.0),
            );
        });
    });

    if !to_region {
        ui.horizontal(|ui| {
            ui.label("荷重の出口:");
            ui.selectable_value(
                &mut app.ui.scoped.wall_plate_draft.add_transfer,
                LoadTransfer::Anchor,
                transfer_label(LoadTransfer::Anchor),
            );
            ui.selectable_value(
                &mut app.ui.scoped.wall_plate_draft.add_transfer,
                LoadTransfer::Columns,
                transfer_label(LoadTransfer::Columns),
            );
        });
        ui.horizontal(|ui| {
            ui.label("取付き線の区間 [0, 1]（既定は全長）:");
            ui.add(
                egui::DragValue::new(&mut app.ui.scoped.wall_plate_draft.add_span[0])
                    .range(0.0..=1.0)
                    .speed(0.01),
            );
            ui.label("〜");
            ui.add(
                egui::DragValue::new(&mut app.ui.scoped.wall_plate_draft.add_span[1])
                    .range(0.0..=1.0)
                    .speed(0.01),
            );
        });
    }

    ui.horizontal(|ui| {
        ui.label("断面:");
        // 下書きの断面が消えている（削除・ID 繰り上げ）場合は未割当へ戻す。
        // 残したままだと `AddAttachedWallPlate` が参照検証で Noop になり、
        // 「追加」を押しても何も起きない状態になる。
        let resolved = app
            .ui
            .scoped
            .wall_plate_draft
            .add_section
            .and_then(|sid| app.core.model.sections.get(sid.index()))
            .filter(|sec| sec.thickness.is_some_and(|t| t > 0.0));
        if resolved.is_none() {
            app.ui.scoped.wall_plate_draft.add_section = None;
        }
        let label = resolved
            .map(|sec| sec.display_name())
            .unwrap_or_else(|| "―".to_string());
        egui::ComboBox::from_id_salt("wp_add_section")
            .selected_text(label)
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut app.ui.scoped.wall_plate_draft.add_section, None, "―");
                for sec in &app.core.model.sections {
                    if sec.thickness.is_some_and(|t| t > 0.0) {
                        ui.selectable_value(
                            &mut app.ui.scoped.wall_plate_draft.add_section,
                            Some(sec.id),
                            sec.display_name(),
                        );
                    }
                }
            });
    })
    .response
    .on_hover_text("壁の板厚と自重は断面から決まります。断面が未割当の壁版は自重が 0 になります");

    // `AddAttachedWallPlate` は非有限の高さを Noop で弾く。GUI 側でも同じ条件で
    // 「追加」を無効にしないと、"inf"・"NaN"（`parse::<f64>()` を通る）を入れたとき
    // ボタンだけ押せて何も起きない状態になる。
    let parsed_extent: Option<[f64; 2]> = {
        let a = app.ui.scoped.wall_plate_draft.add_extent[0]
            .trim()
            .parse::<f64>()
            .ok()
            .filter(|v| v.is_finite());
        let b = app.ui.scoped.wall_plate_draft.add_extent[1]
            .trim()
            .parse::<f64>()
            .ok()
            .filter(|v| v.is_finite());
        a.zip(b).map(|(a, b)| [a, b])
    };
    // 階高いっぱいの壁は高さを持たない（`extent = None`）。「未入力」と区別する
    // ため、追加可否の判定は別のフラグで持つ。
    let extent: Option<[f64; 2]> = if use_story_height {
        None
    } else {
        parsed_extent
    };
    let extent_ok = use_story_height || parsed_extent.is_some();
    let span = app.ui.scoped.wall_plate_draft.add_span;
    // `Model::validate`（squid-n-core）・`wall_anchor_ok`（squid-n-edit）と同じ範囲。
    let span_ok = span[0].is_finite()
        && span[1].is_finite()
        && span[0] >= 0.0
        && span[1] <= 1.0
        && span[1] - span[0] > 1e-9;
    let anchor: Option<RegionAnchor> = match (
        app.ui.scoped.wall_plate_draft.add_nodes[0],
        app.ui.scoped.wall_plate_draft.add_nodes[1],
    ) {
        (Some(a), Some(b)) if a != b => {
            if to_region {
                Some(RegionAnchor::FloorRegion { nodes: [a, b] })
            } else if span_ok {
                Some(RegionAnchor::Line {
                    nodes: [a, b],
                    span,
                    transfer: app.ui.scoped.wall_plate_draft.add_transfer,
                })
            } else {
                None
            }
        }
        _ => None,
    };

    if !to_region && !span_ok {
        ui.label("取付き線の区間は始端 < 終端にしてください");
    }
    let ready = anchor.is_some() && extent_ok;
    if !ready {
        ui.label("取付き先の節点（相異なる2点）と立ち上がり高さを指定してください");
    }
    if ui
        .add_enabled(ready, egui::Button::new("+ 取り付く壁版を追加"))
        .clicked()
    {
        if let Some(anchor) = anchor {
            app.core.scoped.undo.run(
                &mut app.core.model,
                Box::new(AddAttachedWallPlate {
                    anchor,
                    extent,
                    section: app.ui.scoped.wall_plate_draft.add_section,
                    // 開口は追加後に上の「開口・耐震スリットを設定」で与える
                    // （床板の追加フォームが版の仕様を後から与えるのと同じ流儀）。
                    opening_area: 0.0,
                    opening_weight: 0.0,
                }),
            );
            app.core.scoped.staleness.mark_edited();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plate_model() -> (squid_n_core::model::Model, Vec<NodeId>) {
        use squid_n_core::model::Node;
        let mut m = squid_n_core::model::Model::default();
        for (i, c) in [
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [4000.0, 0.0, 3000.0],
            [0.0, 0.0, 3000.0],
        ]
        .iter()
        .enumerate()
        {
            m.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            });
        }
        let ids: Vec<NodeId> = m.nodes.iter().map(|n| n.id).collect();
        (m, ids)
    }

    fn enclosed(id: u32, boundary: Vec<NodeId>, section: Option<SectionId>) -> WallPlate {
        WallPlate {
            id: WallPlateId(id),
            shape: WallPlateShape::Enclosed { boundary },
            section,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![],
            slit: Default::default(),
        }
    }

    /// 「解析要素」列のホバーは、要素にならない理由を実際の状態どおりに述べる。
    ///
    /// 理由の言い分けを誤ると、利用者は直しようのない案内を読むことになる。
    /// とくに「どの壁領域にも帰属していない」を「4 節点でない」と言ってしまうと、
    /// 境界の節点数を疑って時間を使わせる。
    #[test]
    fn test_not_element_reason_distinguishes_causes() {
        let (mut m, ids) = plate_model();

        // どの壁領域からも参照されていない。
        m.wall_plates
            .push(enclosed(0, ids.clone(), Some(SectionId(0))));
        assert!(
            not_element_reason(&m.wall_plates[0], &m).contains("どの壁領域にも帰属していません")
        );

        // 壁領域はあるが、覆っていない（境界が 3 節点）。
        m.wall_regions.push(squid_n_core::model::WallRegion {
            id: squid_n_core::ids::WallRegionId(0),
            name: String::new(),
            boundary: ids.clone(),
            wall_plate_ids: vec![WallPlateId(0)],
            posts: Vec::new(),
        });
        m.wall_plates[0].shape = WallPlateShape::Enclosed {
            boundary: ids[..3].to_vec(),
        };
        assert!(not_element_reason(&m.wall_plates[0], &m).contains("4 節点ではありません"));

        // 覆っているが断面が無い。
        m.wall_plates[0].shape = WallPlateShape::Enclosed {
            boundary: ids.clone(),
        };
        m.wall_plates[0].section = None;
        assert!(not_element_reason(&m.wall_plates[0], &m).contains("断面が割り当たっていません"));

        // 取り付く壁版。
        let mut attached = enclosed(1, Vec::new(), None);
        attached.shape = WallPlateShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [ids[0], ids[1]],
                span: [0.0, 1.0],
                transfer: Default::default(),
            },
            extent: Some([900.0, 900.0]),
        };
        assert!(not_element_reason(&attached, &m).contains("取り付く壁版"));
    }

    /// 空文字列は「個別開口なし」を表し、空の Vec になること。
    #[test]
    fn test_parse_openings_empty_is_empty_vec() {
        assert_eq!(parse_openings("").unwrap(), Vec::new());
        assert_eq!(parse_openings("   ").unwrap(), Vec::new());
    }

    /// 課題例の「,」区切り＋位置指定付き開口が正しくパースされること
    /// （offset のカンマと開口区切りのカンマの曖昧性を解消できているか）。
    #[test]
    fn test_parse_openings_comma_separated_with_offset() {
        let openings = parse_openings("1000x2000, 800x900@3000,500").unwrap();
        assert_eq!(
            openings,
            vec![
                WallOpening {
                    width: 1000.0,
                    height: 2000.0,
                    offset: None,
                },
                WallOpening {
                    width: 800.0,
                    height: 900.0,
                    offset: Some([3000.0, 500.0]),
                },
            ]
        );
    }

    /// 改行区切り（1行1開口）でも同じ結果になること。
    #[test]
    fn test_parse_openings_newline_separated() {
        let openings = parse_openings("1000x2000\n800x900@3000,500").unwrap();
        assert_eq!(
            openings,
            vec![
                WallOpening {
                    width: 1000.0,
                    height: 2000.0,
                    offset: None,
                },
                WallOpening {
                    width: 800.0,
                    height: 900.0,
                    offset: Some([3000.0, 500.0]),
                },
            ]
        );
    }

    /// 大文字 'X' や前後の空白を許容すること。
    #[test]
    fn test_parse_openings_tolerates_whitespace_and_uppercase_x() {
        let openings = parse_openings("  1000 X 2000  ").unwrap();
        assert_eq!(
            openings,
            vec![WallOpening {
                width: 1000.0,
                height: 2000.0,
                offset: None
            }]
        );
    }

    /// 'x' を含まない不正な書式はエラーになること。
    #[test]
    fn test_parse_openings_rejects_missing_x_separator() {
        let err = parse_openings("1000,2000").unwrap_err();
        assert!(err.contains("1000"), "err={err}");
    }

    /// 数値でない幅・高さはエラーになること。
    #[test]
    fn test_parse_openings_rejects_non_numeric() {
        assert!(parse_openings("abcxdef").is_err());
    }

    /// 幅・高さが 0 以下はエラーになること。
    #[test]
    fn test_parse_openings_rejects_non_positive_dims() {
        assert!(parse_openings("0x2000").is_err());
        assert!(parse_openings("1000x-5").is_err());
    }

    /// 位置指定の書式が 'x,z' でない場合はエラーになること。
    #[test]
    fn test_parse_openings_rejects_malformed_offset() {
        assert!(parse_openings("1000x2000@3000").is_err());
    }

    /// format_openings は parse_openings の逆変換になっていること（往復一致）。
    #[test]
    fn test_format_openings_roundtrip() {
        let openings = vec![
            WallOpening {
                width: 1000.0,
                height: 2000.0,
                offset: None,
            },
            WallOpening {
                width: 800.0,
                height: 900.0,
                offset: Some([3000.0, 500.0]),
            },
        ];
        let formatted = format_openings(&openings);
        assert_eq!(parse_openings(&formatted).unwrap(), openings);
    }

    /// 空リストは空文字列へ整形されること。
    #[test]
    fn test_format_openings_empty() {
        assert_eq!(format_openings(&[]), "");
    }
}
