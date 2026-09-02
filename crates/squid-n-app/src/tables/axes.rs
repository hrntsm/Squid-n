//! 通り芯（`Model.axes`）の一覧・編集 UI。
//!
//! 通り芯は日本の構造設計で各通りを識別するための呼称であり、**構造計算には
//! 用いない**。そのため準備計算の成果ではなくモデルデータとして扱い、生成は
//! 準備計算ではなくこのタブの「柱位置から自動生成」で明示的に実行する
//! （生成規則は `squid_n_core::axis_gen`）。編集も解析結果を陳腐化させない。
//!
//! 編集は `squid_n_edit::{ReplaceAxes, RenameAxis}` 経由（undo 対応）。

use crate::app::App;
use crate::table_util::Col;
use squid_n_core::model::{AxisGroupKind, AxisSource};
use squid_n_edit::RenameAxis;

/// 通り名の編集中の状態（GUI 専用）。`(グループ添字, 通り添字, 入力中の名前)`。
#[derive(Clone, Debug, Default)]
pub struct AxisNameDraft {
    pub editing: Option<(usize, usize)>,
    pub name: String,
}

/// グループの幾何の説明文。
fn kind_label(kind: AxisGroupKind) -> String {
    match kind {
        AxisGroupKind::Parallel { origin, angle_deg } => format!(
            "平行芯（原点 ({:.0}, {:.0}) ・方向角 {:.0}°）",
            origin[0], origin[1], angle_deg
        ),
        AxisGroupKind::Other => "平行芯以外（円弧・放射・作図。幾何は保持しない）".to_string(),
    }
}

fn source_label(source: AxisSource) -> &'static str {
    match source {
        AxisSource::Auto => "自動",
        AxisSource::Manual => "手動",
    }
}

pub fn axes_table(ui: &mut egui::Ui, app: &mut App) {
    ui.horizontal(|ui| {
        if ui
            .button("🛠 柱位置から自動生成")
            .on_hover_text(
                "柱（両端の水平距離が 1mm 未満の線材）が立つ位置に、X 方向・Y 方向の\
                 通り芯を作ります。座標の昇順に、そのグループで未使用の最小番号で\
                 X1・X2… と名付けます。\
                 手動で作った通り・ST-Bridge から取り込んだ通り・名前を変更した通りは\
                 保護され、同じ位置には新しい通りを作りません。\
                 通り芯は構造計算に用いないため、生成しても解析結果は陳腐化しません。",
            )
            .clicked()
        {
            app.generate_axes_action();
        }
        ui.colored_label(
            crate::theme::GRAY_600,
            "通り芯は各通りを識別するための呼称です。構造計算には用いません。",
        );
    });
    ui.separator();

    if app.core.model.axes.is_empty() {
        ui.colored_label(
            crate::theme::GRAY_600,
            "通り芯はありません。「柱位置から自動生成」で作成するか、\
             ST-Bridge ファイル（StbAxes）を読み込むと取り込まれます。",
        );
        return;
    }

    let mut pending_edit: Option<(usize, usize)> = None;
    let mut pending_commit: Option<(usize, usize, String)> = None;
    let mut pending_cancel = false;

    for (gi, group) in app.core.model.axes.iter().enumerate() {
        ui.strong(format!("{} — {}", group.name, kind_label(group.kind)));
        if group.axes.is_empty() {
            ui.colored_label(crate::theme::GRAY_600, "  （通りなし）");
            continue;
        }
        let draft = &mut app.ui.scoped.axis_name_draft;
        let model = &app.core.model;
        crate::table_util::standard_table(
            ui,
            &format!("axes_tbl_{gi}"),
            &[
                Col::name("通り名"),
                Col::num("離れ [mm]"),
                Col::num("所属節点"),
                Col::num("所属要素"),
                Col::label("出所"),
                Col::actions_n(2),
            ],
            group.axes.len(),
            |row| {
                let ai = row.index();
                let axis = &group.axes[ai];
                let editing = draft.editing == Some((gi, ai));
                row.col(|ui| {
                    if editing {
                        let resp = crate::table_util::cell_text_edit(ui, &mut draft.name);
                        if resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                            pending_commit = Some((gi, ai, draft.name.trim().to_string()));
                        }
                    } else {
                        crate::table_util::text_cell(ui, &axis.name);
                    }
                });
                row.col(|ui| match axis.distance {
                    Some(d) => {
                        ui.label(format!("{d:.0}"));
                    }
                    None => {
                        crate::table_util::muted_cell(ui, "―", "通り芯の離れが設定されていません")
                    }
                });
                row.col(|ui| {
                    ui.label(format!("{}", axis.nodes.len()));
                });
                row.col(|ui| {
                    ui.label(format!("{}", model.axis_elements(axis).len()));
                });
                row.col(|ui| {
                    ui.label(source_label(axis.source));
                });
                row.col(|ui| {
                    // セルの既定レイアウトは親（縦並びのパネル）を引き継ぐため、
                    // ボタンを 2 つ置く場合は横並びに明示しないと行高からはみ出す。
                    ui.horizontal(|ui| {
                        if editing {
                            if ui.button("✔").on_hover_text("名前を確定").clicked() {
                                pending_commit = Some((gi, ai, draft.name.trim().to_string()));
                            }
                            if ui.button("✖").on_hover_text("取り消し").clicked() {
                                pending_cancel = true;
                            }
                        } else if ui.button("✏").on_hover_text("通り名を変更").clicked() {
                            pending_edit = Some((gi, ai));
                        }
                    });
                });
            },
        );
        ui.add_space(6.0);
    }

    ui.colored_label(
        crate::theme::GRAY_600,
        "出所が「自動」の通りは自動生成のたびに作り直されます。名前を変更した通りは\
         「手動」に変わり、以後の自動生成で保護されます。",
    );

    if let Some((gi, ai)) = pending_edit {
        app.ui.scoped.axis_name_draft.name = app.core.model.axes[gi].axes[ai].name.clone();
        app.ui.scoped.axis_name_draft.editing = Some((gi, ai));
    }
    if pending_cancel {
        app.ui.scoped.axis_name_draft.editing = None;
    }
    if let Some((gi, ai, name)) = pending_commit {
        app.ui.scoped.axis_name_draft.editing = None;
        // 空欄は無視する（名前のない通りは識別札として意味を成さない）。
        if !name.is_empty() && app.core.model.axes[gi].axes[ai].name != name {
            app.core.scoped.undo.run(
                &mut app.core.model,
                Box::new(RenameAxis {
                    group: gi,
                    axis: ai,
                    name,
                }),
            );
            app.core.scoped.staleness.mark_non_calc_edited();
        }
    }
}
