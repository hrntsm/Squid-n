use crate::app::App;
use squid_n_core::ids::LoadCaseId;
use squid_n_core::model::{LoadCaseKind, MemberLoad, MemberLoadKind};
use squid_n_edit::{
    AddCombination, AddLoadCase, DeleteCombination, DeleteMemberLoad, DeleteNodalLoad,
    SetLoadCaseKind, SetLoadCaseName, SetNodalLoad,
};

/// `LoadCaseKind` の全種別（UI の選択肢一覧・順序を1箇所に集約する）。
const LOAD_CASE_KINDS: [LoadCaseKind; 7] = [
    LoadCaseKind::Dead,
    LoadCaseKind::Live,
    LoadCaseKind::LiveSeismic,
    LoadCaseKind::Snow,
    LoadCaseKind::Wind,
    LoadCaseKind::Seismic,
    LoadCaseKind::Other,
];

use crate::app::load_case_kind_label;

pub fn loads_table(ui: &mut egui::Ui, app: &mut App) {
    use crate::table_util::{self, Col};

    // --- スラブ荷重（床荷重）への案内 ---
    ui.label(format!(
        "スラブ: {} 枚（モデルタブの「スラブ」で床荷重を追加できます。分配結果は結果タブ/モデルタブの3Dビューで表示モード「CMQ図」を選ぶと確認できます）",
        app.model.slabs.len()
    ));
    ui.add_space(4.0);

    // --- 荷重ケース一覧（名称編集・追加・削除・編集対象の選択） ---
    ui.horizontal(|ui| {
        ui.strong("荷重ケース");
        if ui
            .button("+ ケース追加")
            .on_hover_text("新しい荷重ケースを追加します")
            .clicked()
        {
            let name = format!("LC{}", app.model.load_cases.len());
            app.undo.run(&mut app.model, Box::new(AddLoadCase { name }));
            // 追加したケースを編集対象として選択
            app.nav.focus_load_case = app.model.load_cases.last().map(|lc| lc.id);
            app.staleness.mark_edited();
        }
    });
    let n_lc = app.model.load_cases.len();
    let mut pending_name: Vec<(usize, String)> = Vec::new();
    let mut pending_kind: Vec<(LoadCaseId, LoadCaseKind)> = Vec::new();
    let mut pending_delete: Option<LoadCaseId> = None;
    let mut name_bufs: Vec<String> = app
        .model
        .load_cases
        .iter()
        .map(|lc| lc.name.clone())
        .collect();

    table_util::standard_table(
        ui,
        "load_cases_tbl",
        &[
            Col::id(),
            Col::name("名称"),
            Col::name("種別"),
            Col::num("荷重数"),
            Col::actions(),
        ],
        n_lc,
        |row| {
            let i = row.index();
            let lc = &app.model.load_cases[i];
            let is_sel = app.nav.focus_load_case == Some(lc.id);
            row.col(|ui| {
                if table_util::id_cell(ui, is_sel, lc.id.0, "クリックで下の荷重編集の対象にする")
                {
                    app.nav.focus_load_case = Some(lc.id);
                }
            });
            row.col(|ui| {
                if i < name_bufs.len() {
                    let resp = table_util::cell_text_edit(ui, &mut name_bufs[i]);
                    if resp.lost_focus() && resp.changed() {
                        let trimmed = name_bufs[i].trim().to_string();
                        if trimmed != lc.name && !trimmed.is_empty() {
                            pending_name.push((i, trimmed));
                        }
                    }
                }
            });
            row.col(|ui| {
                table_util::cell_combo(
                    ui,
                    ("load_case_kind", lc.id.0),
                    load_case_kind_label(lc.kind),
                    |ui| {
                        for kind in LOAD_CASE_KINDS {
                            if ui
                                .selectable_label(lc.kind == kind, load_case_kind_label(kind))
                                .clicked()
                                && lc.kind != kind
                            {
                                pending_kind.push((lc.id, kind));
                            }
                        }
                    },
                );
            });
            row.col(|ui| {
                ui.label((lc.nodal.len() + lc.member.len()).to_string());
            });
            row.col(|ui| {
                let referenced = app
                    .model
                    .combinations
                    .iter()
                    .any(|c| c.terms.iter().any(|(id, _)| *id == lc.id));
                let blocked = referenced.then_some("荷重組合せから参照中のため削除できません");
                if table_util::delete_cell(ui, "ケースと中身の荷重をまとめて削除", blocked)
                {
                    pending_delete = Some(lc.id);
                }
            });
        },
    );

    // 削除は `delete_load_case_action` が自前で陳腐化を扱うため、ここには含めない
    // （組合せから参照中で削除が拒まれた場合に、変更もないのに陳腐化してしまう）。
    let had_name = !pending_name.is_empty() || !pending_kind.is_empty();
    for (i, name) in pending_name {
        let lc_id = LoadCaseId(app.model.load_cases[i].id.0);
        app.undo.run(
            &mut app.model,
            Box::new(SetLoadCaseName { id: lc_id, name }),
        );
    }
    for (id, kind) in pending_kind {
        app.undo
            .run(&mut app.model, Box::new(SetLoadCaseKind { id, kind }));
    }
    if let Some(lc_id) = pending_delete {
        // 削除は後続の LoadCaseId を繰り上げるため、開いたままの荷重モーダルを
        // 閉じる必要がある（`App::delete_load_case_action` に一元化）。
        app.delete_load_case_action(lc_id);
    }
    if had_name {
        app.staleness.mark_edited();
    }

    ui.add_space(8.0);
    combinations_section(ui, app);

    ui.add_space(8.0);
    egui::CollapsingHeader::new("荷重計算条件")
        .id_salt("load_cfg_section")
        .default_open(false)
        .show(ui, |ui| {
            crate::tables::load_cfg::load_cfg_panel(ui, app);
        });

    ui.add_space(8.0);

    // --- 節点荷重詳細（選択中の荷重ケース） ---
    ui.strong("節点荷重");
    if app.model.load_cases.is_empty() {
        ui.label("荷重ケースがありません。「+ ケース追加」で作成してください。");
        return;
    }
    // 編集対象: ナビゲータ/上表で選択したケース → 最後に実行したケース → 先頭
    let lc_idx = app
        .nav
        .focus_load_case
        .and_then(|id| app.model.load_cases.iter().position(|lc| lc.id == id))
        .or_else(|| {
            app.last_static.and_then(|key| match key {
                // 地震静的(Seismic)はユーザー荷重ケースに対応しないため
                // None（呼び出し元のフォールバックで先頭ケースが選ばれる）。
                crate::app::StaticKey::Case(crate::app::StaticCaseKey::User(id)) => {
                    app.model.load_cases.iter().position(|lc| lc.id == id)
                }
                crate::app::StaticKey::Case(crate::app::StaticCaseKey::Seismic(_))
                | crate::app::StaticKey::Combo(_) => None,
            })
        })
        .unwrap_or(0);
    let lc_id = app.model.load_cases[lc_idx].id;
    ui.label(format!(
        "ケース: {} ({})",
        lc_id.0, app.model.load_cases[lc_idx].name
    ));

    let nodal_count = app.model.load_cases[lc_idx].nodal.len();
    // 準備計算が生成した荷重は同期のたびに作り直されるため編集・削除できない。
    // 表には残す（この画面は準備計算の結果を確認する場でもある）。
    let mut pending_load: Vec<(usize, [f64; 6])> = Vec::new();
    let mut pending_name_edit: Vec<(usize, String)> = Vec::new();
    let mut pending_nodal_delete: Option<usize> = None;
    let mut value_bufs: Vec<[String; 6]> = app.model.load_cases[lc_idx]
        .nodal
        .iter()
        .map(|n| n.values.map(|v| format!("{:.2}", v)))
        .collect();
    let mut nodal_name_bufs: Vec<String> = app.model.load_cases[lc_idx]
        .nodal
        .iter()
        .map(|n| n.name.clone())
        .collect();

    table_util::standard_table(
        ui,
        "nodal_loads_tbl",
        &[
            Col::name("名称"),
            Col::id_named("節点"),
            Col::num("Fx"),
            Col::num("Fy"),
            Col::num("Fz"),
            Col::num("Mx"),
            Col::num("My"),
            Col::num("Mz"),
            Col::actions(),
        ],
        nodal_count,
        |row| {
            let i = row.index();
            let nodal = &app.model.load_cases[lc_idx].nodal[i];
            let is_auto = nodal.source.is_auto();
            row.col(|ui| {
                if is_auto {
                    table_util::muted_cell(ui, AUTO_LOAD_LABEL, AUTO_LOAD_HOVER);
                } else {
                    let resp = table_util::cell_text_edit(ui, &mut nodal_name_bufs[i]);
                    if resp.lost_focus() && resp.changed() {
                        pending_name_edit.push((i, nodal_name_bufs[i].trim().to_string()));
                    }
                }
            });
            row.col(|ui| {
                table_util::id_label(ui, nodal.node.0);
            });
            for k in 0..6 {
                row.col(|ui| {
                    if is_auto {
                        ui.label(format!("{:.2}", nodal.values[k]));
                        return;
                    }
                    let buf = &mut value_bufs[i][k];
                    let resp = table_util::cell_text_edit(ui, buf);
                    if resp.lost_focus() && resp.changed() {
                        if let Ok(val) = buf.trim().parse::<f64>() {
                            if (val - nodal.values[k]).abs() > 1e-9 {
                                let mut new_vals = nodal.values;
                                new_vals[k] = val;
                                pending_load.push((i, new_vals));
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
                let blocked = is_auto.then_some(AUTO_LOAD_HOVER);
                if table_util::delete_cell(ui, "この節点荷重を削除", blocked) {
                    pending_nodal_delete = Some(i);
                }
            });
        },
    );

    // モデルが実際に変わったときだけ解析結果を陳腐化させる。`UndoStack::run` は
    // コマンドが Noop だった場合に false を返すため、その戻り値で判定する
    // （入力欄で打ち直して元の値に戻した場合など、値が変わらない確定操作で
    // 解析結果を無効にしない）。
    let mut had_load = false;
    // 値・名称の変更は `SetNodalLoad`（要素まるごと差し替え）で行うため、
    // 変更前の内容を読んでから 1 件ずつ発行する。
    for (index, values) in pending_load {
        let mut load = app.model.load_cases[lc_idx].nodal[index].clone();
        load.values = values;
        had_load |= app.undo.run(
            &mut app.model,
            Box::new(SetNodalLoad {
                lc: lc_id,
                index,
                load,
            }),
        );
    }
    for (index, name) in pending_name_edit {
        let mut load = app.model.load_cases[lc_idx].nodal[index].clone();
        if load.name == name {
            continue;
        }
        load.name = name;
        had_load |= app.undo.run(
            &mut app.model,
            Box::new(SetNodalLoad {
                lc: lc_id,
                index,
                load,
            }),
        );
    }
    if let Some(index) = pending_nodal_delete {
        had_load |= app.undo.run(
            &mut app.model,
            Box::new(DeleteNodalLoad { lc: lc_id, index }),
        );
    }
    if had_load {
        app.staleness.mark_edited();
    }

    // --- 部材荷重セクション ---
    ui.add_space(8.0);
    ui.strong("部材荷重");

    let mut pending_delete: Option<usize> = None;
    {
        let member_loads = &app.model.load_cases[lc_idx].member;
        if member_loads.is_empty() {
            ui.label("部材荷重なし");
        } else {
            for (i, ml) in member_loads.iter().enumerate() {
                ui.horizontal(|ui| {
                    let is_auto = ml.source.is_auto();
                    let label = format!(
                        "{} / 部材#{} / {} / {}",
                        if is_auto {
                            AUTO_LOAD_LABEL.to_string()
                        } else {
                            member_load_display_name(ml)
                        },
                        ml.elem.0,
                        member_load_kind_text(&ml.kind),
                        format_args!("dir=({:.1},{:.1},{:.1})", ml.dir[0], ml.dir[1], ml.dir[2]),
                    );
                    if is_auto {
                        ui.colored_label(crate::theme::GRAY_600, label)
                            .on_hover_text(AUTO_LOAD_HOVER);
                    } else {
                        ui.label(label);
                    }
                    let btn = ui.add_enabled(!is_auto, egui::Button::new("削除"));
                    if is_auto {
                        btn.on_disabled_hover_text(AUTO_LOAD_HOVER);
                    } else if btn.clicked() {
                        pending_delete = Some(i);
                    }
                });
            }
        }
    }
    if let Some(index) = pending_delete {
        app.undo.run(
            &mut app.model,
            Box::new(DeleteMemberLoad { lc: lc_id, index }),
        );
        app.staleness.mark_edited();
    }

    ui.add_space(4.0);
    ui.colored_label(
        crate::theme::GRAY_600,
        "荷重の追加は左パネルのナビゲータで、荷重ケースを右クリックして行います\
         （対象の節点・部材を 3D ビューで選べます）。",
    );
}

/// 自動生成された荷重であることを示す表示名。
pub(crate) const AUTO_LOAD_LABEL: &str = "（自動計算）";

/// 自動生成された荷重を編集・削除できない理由。
pub(crate) const AUTO_LOAD_HOVER: &str =
    "準備計算が生成した荷重です。実行のたびに作り直されるため編集・削除できません";

/// 部材荷重の種別・諸元の表示文字列。
pub(crate) fn member_load_kind_text(kind: &MemberLoadKind) -> String {
    match *kind {
        MemberLoadKind::Point { a, p } => format!("中間集中 a={:.0} P={:.1}", a, p),
        MemberLoadKind::Distributed { a, b, w1, w2 } => {
            format!("分布 [{:.0},{:.0}] w1={:.2} w2={:.2}", a, b, w1, w2)
        }
    }
}

/// 部材荷重の表示名。名称が未入力なら種別から自動ラベルを作る。
pub(crate) fn member_load_display_name(ml: &MemberLoad) -> String {
    if ml.name.trim().is_empty() {
        member_load_kind_text(&ml.kind)
    } else {
        ml.name.clone()
    }
}

/// 節点荷重の表示名。名称が未入力なら零でない成分から自動ラベルを作る。
pub(crate) fn nodal_load_display_name(nl: &squid_n_core::model::NodalLoad) -> String {
    if !nl.name.trim().is_empty() {
        return nl.name.clone();
    }
    const COMPONENTS: [&str; 6] = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"];
    let parts: Vec<String> = nl
        .values
        .iter()
        .enumerate()
        .filter(|(_, v)| v.abs() > 1e-9)
        .map(|(k, v)| format!("{}={:.1}", COMPONENTS[k], v))
        .collect();
    if parts.is_empty() {
        "0".to_string()
    } else {
        parts.join(" ")
    }
}

/// 荷重ケース名を引く（見つからなければ空文字）。
fn load_case_name(model: &squid_n_core::model::Model, id: LoadCaseId) -> String {
    model
        .load_cases
        .iter()
        .find(|lc| lc.id == id)
        .map(|lc| lc.name.clone())
        .unwrap_or_default()
}

/// 荷重ケース選択 ComboBox。`allow_none` の場合のみ「（なし）」を選択できる。
fn combo_case_selector(
    ui: &mut egui::Ui,
    id_salt: &str,
    label: &str,
    model: &squid_n_core::model::Model,
    selected: &mut Option<LoadCaseId>,
    allow_none: bool,
) {
    ui.horizontal(|ui| {
        ui.label(label);
        let text = selected
            .and_then(|id| model.load_cases.iter().find(|lc| lc.id == id))
            .map(|lc| format!("[{}] {}", lc.id.0, lc.name))
            .unwrap_or_else(|| "（なし）".to_string());
        egui::ComboBox::from_id_salt(id_salt)
            .selected_text(text)
            .show_ui(ui, |ui| {
                if allow_none
                    && ui
                        .selectable_label(selected.is_none(), "（なし）")
                        .clicked()
                {
                    *selected = None;
                }
                for lc in &model.load_cases {
                    if ui
                        .selectable_label(
                            *selected == Some(lc.id),
                            format!("[{}] {}", lc.id.0, lc.name),
                        )
                        .clicked()
                    {
                        *selected = Some(lc.id);
                    }
                }
            });
    });
}

/// 荷重組合せセクション：既存組合せの一覧・削除と、標準組合せの自動生成 UI。
fn combinations_section(ui: &mut egui::Ui, app: &mut App) {
    ui.strong("荷重組合せ");

    if app.model.load_cases.is_empty() {
        ui.label("荷重ケースがありません。組合せを作成するにはまず荷重ケースを追加してください。");
        return;
    }

    // --- 既存組合せの一覧（内訳表示・削除） ---
    let mut pending_delete: Option<usize> = None;
    if app.model.combinations.is_empty() {
        ui.label("組合せがありません。下の「自動生成」で作成できます。");
    } else {
        for (i, combo) in app.model.combinations.iter().enumerate() {
            ui.horizontal(|ui| {
                let terms_str = combo
                    .terms
                    .iter()
                    .map(|(id, factor)| {
                        format!(
                            "{:.2}×[{}]{}",
                            factor,
                            id.0,
                            load_case_name(&app.model, *id)
                        )
                    })
                    .collect::<Vec<_>>()
                    .join(" + ");
                ui.label(format!("{}: {}", combo.name, terms_str));
                if ui
                    .button("🗑")
                    .on_hover_text("この荷重組合せを削除")
                    .clicked()
                {
                    pending_delete = Some(i);
                }
            });
        }
    }
    if let Some(idx) = pending_delete {
        app.undo
            .run(&mut app.model, Box::new(DeleteCombination { index: idx }));
        app.staleness.mark_edited();
    }

    // --- 自動生成 ---
    ui.add_space(4.0);
    ui.strong("自動生成");
    combo_case_selector(
        ui,
        "combo_draft_dl",
        "DL用:",
        &app.model,
        &mut app.combo_draft.dl,
        false,
    );
    combo_case_selector(
        ui,
        "combo_draft_ll",
        "LL用:",
        &app.model,
        &mut app.combo_draft.ll,
        false,
    );
    combo_case_selector(
        ui,
        "combo_draft_seismic_x",
        "地震X用:",
        &app.model,
        &mut app.combo_draft.seismic_x,
        true,
    );
    combo_case_selector(
        ui,
        "combo_draft_seismic_y",
        "地震Y用:",
        &app.model,
        &mut app.combo_draft.seismic_y,
        true,
    );
    combo_case_selector(
        ui,
        "combo_draft_snow",
        "積雪用:",
        &app.model,
        &mut app.combo_draft.snow,
        true,
    );

    ui.checkbox(&mut app.analysis_cfg.heavy_snow_zone, "多雪区域")
        .on_hover_text("有効にすると長期 G+P+δ1・S、短期地震 G+P+δ3・S±K の組合せも生成します（施行令86条・82条）");
    if app.analysis_cfg.heavy_snow_zone {
        ui.horizontal(|ui| {
            ui.label("積雪低減係数:").on_hover_text(
                "多雪区域の積雪荷重低減係数（平12建告1455号。既定 δ1=0.7、δ3=0.35）",
            );
            ui.label("δ1(長期)");
            ui.add(
                egui::DragValue::new(&mut app.analysis_cfg.snow_delta1)
                    .speed(0.01)
                    .range(0.0..=1.0),
            );
            ui.label("δ3(地震時)");
            ui.add(
                egui::DragValue::new(&mut app.analysis_cfg.snow_delta3)
                    .speed(0.01)
                    .range(0.0..=1.0),
            );
        });
    }

    let can_generate = app.combo_draft.dl.is_some() && app.combo_draft.ll.is_some();
    ui.horizontal(|ui| {
        if ui
            .add_enabled(can_generate, egui::Button::new("⚙ 標準組合せを生成"))
            .on_hover_text(
                "DL/LL（必須）と地震X/Y・積雪（任意）から長期・短期の標準組合せを生成します",
            )
            .clicked()
        {
            if let (Some(dl), Some(ll)) = (app.combo_draft.dl, app.combo_draft.ll) {
                let input = squid_n_load::combo::ComboInput {
                    dl,
                    ll,
                    seismic_x: app.combo_draft.seismic_x,
                    seismic_y: app.combo_draft.seismic_y,
                    snow: app.combo_draft.snow,
                    heavy_snow_zone: app.analysis_cfg.heavy_snow_zone,
                    snow_factors: Some(squid_n_load::combo::SnowFactors {
                        delta1: app.analysis_cfg.snow_delta1,
                        delta3: app.analysis_cfg.snow_delta3,
                    }),
                };
                let combos = squid_n_load::combo::standard_combinations(&input);
                for combo in combos {
                    app.undo
                        .run(&mut app.model, Box::new(AddCombination { combo }));
                }
                app.staleness.mark_edited();
                // 別経路で組合せを作れたため、自動生成の失敗表示は解消する
                // （残すと解決済みのエラーが欄に出続ける）。
                app.combo_error = None;
            }
        }
        if ui
            .button("⚙ 種別から自動生成")
            .on_hover_text(
                "荷重ケースの種別から固定(必須)・積載(必須)・積雪・風を各先頭1件選んで標準組合せを生成します。\
                 種別が特定できない場合はエラーになります",
            )
            .clicked()
        {
            app.auto_generate_combinations_action();
        }
    });
    // 組合せ生成に固有のエラーのみ表示する（`last_error` は共用スロットのため、
    // ここへ出すと他の操作のエラーが無関係な欄に現れる）。
    if let Some(err) = &app.combo_error {
        ui.colored_label(crate::theme::ERROR_RED, err);
    }
}
