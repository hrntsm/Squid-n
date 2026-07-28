//! 制振要素（ダンパー定義、`Model::damper_defs`）の編集UI。
//!
//! 断面カタログ（`section_editor.rs`）と同じ「あらかじめ諸元を登録したプリセットを
//! 選んで部材に割り当てる」UX を制振ダンパーにも提供する。本パネルは
//! `ModelTab::Sections`（断面タブ）の断面作成パネル群と並べて表示する
//! （`app/panels.rs` の Sections 表示箇所から呼ぶ）。
//!
//! 定義は `ElemId` を持たない名前付きプリセットであり、部材への割当は
//! `DamperProps` の値コピーで行うため（`squid_n_core::model::DamperDef` の docs参照）、
//! ここでの追加・編集・削除は既に割当済みの部材には影響しない。

use crate::app::App;
use squid_n_core::model::{DamperDef, DamperKind, DamperProps};
use squid_n_edit::{AddDamperDef, RemoveDamperDef, UpdateDamperDef};

/// 制振要素定義フォームのドラフト状態。
#[derive(Clone, Debug)]
pub struct DamperDefDraft {
    pub name: String,
    pub props: DamperProps,
    /// 編集対象の定義インデックス（`model.damper_defs`）。`None` は新規追加。
    pub edit_index: Option<usize>,
}

impl Default for DamperDefDraft {
    fn default() -> Self {
        Self {
            name: "ダンパー定義1".to_string(),
            props: DamperProps::default(),
            edit_index: None,
        }
    }
}

/// `DamperKind` の日本語表示名。
pub fn damper_kind_label(kind: DamperKind) -> &'static str {
    match kind {
        DamperKind::Maxwell => "オイルダンパー(Maxwell)",
        DamperKind::HystereticBilinear => "履歴型(バイリニア)",
    }
}

/// 一覧表示用の諸元サマリ（種別ごとに関係する値のみ）。純関数（テスト容易）。
pub fn damper_summary(props: &DamperProps) -> String {
    match props.kind {
        DamperKind::Maxwell => {
            let relief = match (props.relief_velocity, props.c2_ratio) {
                (Some(vr), Some(c2)) => {
                    format!(" / リリーフ Vr={:.0}mm/s C2/C1={:.2}", vr, c2)
                }
                _ => String::new(),
            };
            format!(
                "Kd={:.0}kN/mm C0={:.1}kN·(s/mm)^α α={:.2}{}",
                props.kd / 1000.0,
                props.c0 / 1000.0,
                props.alpha,
                relief
            )
        }
        DamperKind::HystereticBilinear => format!(
            "Kd={:.0}kN/mm Qy={:.0}kN k2/k1={:.3}",
            props.kd / 1000.0,
            props.qy / 1000.0,
            props.k2_ratio
        ),
    }
}

/// 制振要素（ダンパー定義）パネル。`ModelTab::Sections` の断面パネル群と並置する。
pub fn damper_def_panel(ui: &mut egui::Ui, app: &mut App) {
    ui.group(|ui| {
        ui.strong("制振要素（ダンパー定義プリセット）");
        ui.label(
            egui::RichText::new(
                "断面のように、あらかじめ諸元を登録した制振ダンパーの定義（製品プリセット）を、\
                 「部材」タブの「制振ダンパー追加」フォームで選んで部材に割り当てられます。",
            )
            .color(crate::theme::GRAY_600)
            .small(),
        );
        ui.separator();

        // ── 一覧 ──────────────────────────────────────────
        let mut pending_edit: Option<usize> = None;
        let mut pending_delete: Option<usize> = None;
        if app.model.damper_defs.is_empty() {
            ui.label("登録済みの制振要素定義はありません。");
        } else {
            for (i, def) in app.model.damper_defs.iter().enumerate() {
                ui.horizontal(|ui| {
                    ui.label(format!("[{}] {}", i, def.name));
                    ui.label(damper_kind_label(def.props.kind));
                    ui.label(
                        egui::RichText::new(damper_summary(&def.props))
                            .color(crate::theme::GRAY_600)
                            .small(),
                    );
                    if ui
                        .button("✏")
                        .on_hover_text("フォームへ読み込んで編集")
                        .clicked()
                    {
                        pending_edit = Some(i);
                    }
                    if ui
                        .button("🗑")
                        .on_hover_text(
                            "この定義を削除します（既に割り当て済みの部材には影響しません）",
                        )
                        .clicked()
                    {
                        pending_delete = Some(i);
                    }
                });
            }
        }
        if let Some(i) = pending_edit {
            if let Some(def) = app.model.damper_defs.get(i) {
                app.damper_def_draft.name = def.name.clone();
                app.damper_def_draft.props = def.props;
                app.damper_def_draft.edit_index = Some(i);
            }
        }
        if let Some(i) = pending_delete {
            app.undo
                .run(&mut app.model, Box::new(RemoveDamperDef { index: i }));
            app.staleness.mark_edited();
            if app.damper_def_draft.edit_index == Some(i) {
                app.damper_def_draft.edit_index = None;
            }
        }

        ui.separator();
        ui.strong("定義を作成・編集");
        ui.horizontal(|ui| {
            ui.label("名称:");
            ui.text_edit_singleline(&mut app.damper_def_draft.name);
        });
        ui.horizontal(|ui| {
            ui.label("種別:");
            for k in [DamperKind::Maxwell, DamperKind::HystereticBilinear] {
                if ui
                    .selectable_label(app.damper_def_draft.props.kind == k, damper_kind_label(k))
                    .clicked()
                {
                    app.damper_def_draft.props.kind = k;
                }
            }
        });

        match app.damper_def_draft.props.kind {
            DamperKind::Maxwell => maxwell_fields(ui, &mut app.damper_def_draft.props),
            DamperKind::HystereticBilinear => {
                hysteretic_fields(ui, &mut app.damper_def_draft.props)
            }
        }

        ui.separator();
        ui.horizontal(|ui| {
            let can_add = !app.damper_def_draft.name.trim().is_empty();
            if ui
                .add_enabled(can_add, egui::Button::new("+ 追加"))
                .clicked()
            {
                app.undo.run(
                    &mut app.model,
                    Box::new(AddDamperDef {
                        def: DamperDef {
                            name: app.damper_def_draft.name.clone(),
                            props: app.damper_def_draft.props,
                        },
                    }),
                );
                app.staleness.mark_edited();
                app.damper_def_draft.edit_index = None;
            }

            let can_update = app
                .damper_def_draft
                .edit_index
                .is_some_and(|i| i < app.model.damper_defs.len());
            if ui
                .add_enabled(can_update, egui::Button::new("✏ 選択定義へ適用"))
                .on_hover_text("フォームの内容で読み込み元の定義を上書きします")
                .clicked()
            {
                if let Some(i) = app.damper_def_draft.edit_index {
                    app.undo.run(
                        &mut app.model,
                        Box::new(UpdateDamperDef {
                            index: i,
                            def: DamperDef {
                                name: app.damper_def_draft.name.clone(),
                                props: app.damper_def_draft.props,
                            },
                        }),
                    );
                    app.staleness.mark_edited();
                }
            }

            if app.damper_def_draft.edit_index.is_some() && ui.button("新規作成に戻す").clicked()
            {
                app.damper_def_draft.edit_index = None;
            }
        });
    });
}

/// マクスウェル（速度依存型）の諸元入力。リリーフ特性はチェックで有効化する
/// オプション入力（`relief_velocity`・`c2_ratio` は `Some`/`None` が対）。
fn maxwell_fields(ui: &mut egui::Ui, props: &mut DamperProps) {
    ui.horizontal_wrapped(|ui| {
        ui.label("Kd[kN/mm]（バネ剛性）:");
        let mut kd_kn = props.kd / 1000.0;
        if ui
            .add(
                egui::DragValue::new(&mut kd_kn)
                    .speed(1.0)
                    .range(0.0..=1.0e6),
            )
            .changed()
        {
            props.kd = kd_kn * 1000.0;
        }
        ui.label("C0[kN·(s/mm)^α]（粘性係数）:");
        let mut c0_kn = props.c0 / 1000.0;
        if ui
            .add(
                egui::DragValue::new(&mut c0_kn)
                    .speed(0.1)
                    .range(0.0..=1.0e6),
            )
            .changed()
        {
            props.c0 = c0_kn * 1000.0;
        }
        ui.label("α（速度指数。1.0で線形粘性）:");
        ui.add(
            egui::DragValue::new(&mut props.alpha)
                .speed(0.01)
                .range(0.05..=2.0),
        );
    });

    let mut relief_enabled = props.relief_velocity.is_some();
    if ui
        .checkbox(
            &mut relief_enabled,
            "リリーフ特性を有効化（オイルダンパーのバイパス弁による頭打ち特性）",
        )
        .changed()
    {
        if relief_enabled {
            props.relief_velocity = Some(props.relief_velocity.unwrap_or(200.0));
            props.c2_ratio = Some(props.c2_ratio.unwrap_or(0.1));
        } else {
            props.relief_velocity = None;
            props.c2_ratio = None;
        }
    }
    if relief_enabled {
        ui.horizontal_wrapped(|ui| {
            ui.label("Vr[mm/s]（リリーフ速度）:");
            let mut vr = props.relief_velocity.unwrap_or(200.0);
            if ui
                .add(egui::DragValue::new(&mut vr).speed(1.0).range(0.0..=1.0e5))
                .changed()
            {
                props.relief_velocity = Some(vr);
            }
            ui.label("C2/C1（リリーフ後の減衰係数比）:");
            let mut c2 = props.c2_ratio.unwrap_or(0.1);
            if ui
                .add(egui::DragValue::new(&mut c2).speed(0.01).range(0.0..=1.0))
                .changed()
            {
                props.c2_ratio = Some(c2);
            }
        });
    }
}

/// 履歴型バイリニアの諸元入力（鋼材系ダンパー）。
fn hysteretic_fields(ui: &mut egui::Ui, props: &mut DamperProps) {
    ui.horizontal_wrapped(|ui| {
        ui.label("Kd=k1[kN/mm]（初期軸剛性）:");
        let mut kd_kn = props.kd / 1000.0;
        if ui
            .add(
                egui::DragValue::new(&mut kd_kn)
                    .speed(1.0)
                    .range(0.0..=1.0e6),
            )
            .changed()
        {
            props.kd = kd_kn * 1000.0;
        }
        ui.label("Qy[kN]（降伏軸力）:");
        let mut qy_kn = props.qy / 1000.0;
        if ui
            .add(
                egui::DragValue::new(&mut qy_kn)
                    .speed(1.0)
                    .range(0.0..=1.0e6),
            )
            .changed()
        {
            props.qy = qy_kn * 1000.0;
        }
        ui.label("k2/k1（第2剛性比）:");
        ui.add(
            egui::DragValue::new(&mut props.k2_ratio)
                .speed(0.005)
                .range(0.0..=0.99),
        );
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_damper_summary_maxwell_without_relief() {
        let props = DamperProps {
            kind: DamperKind::Maxwell,
            kd: 100_000.0,
            c0: 2_000.0,
            alpha: 1.0,
            ..DamperProps::default()
        };
        let s = damper_summary(&props);
        assert!(s.contains("Kd=100kN/mm"), "s={s}");
        assert!(s.contains("C0=2.0kN"), "s={s}");
        assert!(!s.contains("リリーフ"), "s={s}");
    }

    #[test]
    fn test_damper_summary_maxwell_with_relief() {
        let props = DamperProps {
            kind: DamperKind::Maxwell,
            relief_velocity: Some(150.0),
            c2_ratio: Some(0.2),
            ..DamperProps::default()
        };
        let s = damper_summary(&props);
        assert!(s.contains("リリーフ"), "s={s}");
        assert!(s.contains("Vr=150"), "s={s}");
        assert!(s.contains("C2/C1=0.20"), "s={s}");
    }

    #[test]
    fn test_damper_summary_hysteretic() {
        let props = DamperProps {
            kind: DamperKind::HystereticBilinear,
            kd: 50_000.0,
            qy: 30_000.0,
            k2_ratio: 0.02,
            ..DamperProps::default()
        };
        let s = damper_summary(&props);
        assert!(s.contains("Kd=50kN/mm"), "s={s}");
        assert!(s.contains("Qy=30kN"), "s={s}");
        assert!(s.contains("k2/k1=0.020"), "s={s}");
    }

    #[test]
    fn test_damper_kind_label() {
        assert_eq!(
            damper_kind_label(DamperKind::Maxwell),
            "オイルダンパー(Maxwell)"
        );
        assert_eq!(
            damper_kind_label(DamperKind::HystereticBilinear),
            "履歴型(バイリニア)"
        );
    }

    #[test]
    fn test_damper_def_draft_default_is_maxwell() {
        let d = DamperDefDraft::default();
        assert_eq!(d.props.kind, DamperKind::Maxwell);
        assert!(d.edit_index.is_none());
        assert!(!d.name.trim().is_empty());
    }
}
