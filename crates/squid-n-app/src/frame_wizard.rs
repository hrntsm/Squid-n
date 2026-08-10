//! 架構作成ウィザード。スパンと階高を入力して、柱・大梁・柱脚支点・通り芯・階・床を
//! 一括生成する（`ファイル > 新規（架構ウィザード）…`）。
//!
//! **新規モデルを作る操作**であり、現在のモデルを置き換える。既存モデルへ架構を
//! 足す使い方は想定していない（既存節点との突き合わせ規則を決める必要があり、
//! 3D ビューの格子点スナップで代替できる）。
//!
//! 生成の規則は [`squid_n_core::frame_gen`] が持つ。ここは入力欄と確認の表示だけを
//! 受け持ち、寸法から架構を組み立てる判断はコアへ委ねる。

use crate::app::App;
use squid_n_core::frame_gen::{BaseSupport, FrameSpec};
use squid_n_core::model::SlabUsage;

/// ウィザードの入力状態。`App` が保持し、ウィンドウを閉じても内容を保つ。
#[derive(Debug, Clone)]
pub struct FrameWizardState {
    pub open: bool,
    pub spec: FrameSpec,
    /// 等スパン一括入力の「本数」と「スパン [mm]」。
    pub bulk_x: (usize, f64),
    pub bulk_y: (usize, f64),
    /// 等階高一括入力の「階数」と「階高 [mm]」。
    pub bulk_story: (usize, f64),
}

impl Default for FrameWizardState {
    fn default() -> Self {
        Self {
            open: false,
            spec: FrameSpec::default(),
            bulk_x: (2, 6000.0),
            bulk_y: (1, 6000.0),
            bulk_story: (3, 3500.0),
        }
    }
}

/// 用途選択で提示するプリセット（床タブと同じ並びの抜粋）。
const USAGE_PRESETS: &[(Option<SlabUsage>, &str)] = &[
    (None, "なし"),
    (Some(SlabUsage::Office), "事務室"),
    (Some(SlabUsage::Residential), "住宅の居室"),
    (Some(SlabUsage::Classroom), "教室"),
    (Some(SlabUsage::Store), "店舗の売場"),
    (Some(SlabUsage::Corridor), "廊下・階段"),
    (Some(SlabUsage::RoofUnused), "屋上（通常人が使用しない）"),
];

/// ウィザードのウィンドウを描く。生成が確定したら現在のモデルを置き換える。
pub fn frame_wizard_window(ctx: &egui::Context, app: &mut App) {
    if !app.frame_wizard.open {
        return;
    }
    let mut open = true;
    let mut generate = false;
    egui::Window::new("新規（架構ウィザード）")
        .open(&mut open)
        .resizable(true)
        .default_width(520.0)
        .show(ctx, |ui| {
            ui.label(
                "スパンと階高を入力すると、節点・柱・大梁・柱脚支点・通り芯・階・床を\
                 まとめて作ります。断面と材料は作りませんので、生成後に断面タブで\
                 割り当ててください。",
            );
            ui.colored_label(
                crate::theme::BEST_YELLOW,
                "⚠ 現在のモデルを置き換えます（undo できません）。",
            );
            ui.separator();

            let w = &mut app.frame_wizard;
            spans_section(ui, "X 方向", &mut w.spec.x_spans, &mut w.bulk_x, "wiz_x");
            ui.add_space(4.0);
            spans_section(ui, "Y 方向", &mut w.spec.y_spans, &mut w.bulk_y, "wiz_y");
            ui.add_space(4.0);
            stories_section(ui, w);
            ui.add_space(4.0);
            options_section(ui, w);

            ui.separator();
            let counts = w.spec.counts();
            match w.spec.validate() {
                Some(msg) => {
                    ui.colored_label(crate::theme::ERROR_RED, msg);
                }
                None => {
                    ui.label(format!(
                        "生成: 節点 {} ・柱 {} 本 ・大梁 {} 本 ・床 {} 枚",
                        counts.nodes, counts.columns, counts.girders, counts.slabs
                    ));
                    if ui
                        .button("✅ この内容で作成")
                        .on_hover_text("現在のモデルを置き換えます")
                        .clicked()
                    {
                        generate = true;
                    }
                }
            }
        });

    if generate {
        match squid_n_core::frame_gen::frame_model(&app.frame_wizard.spec) {
            Ok(model) => {
                app.load_model(model);
                app.project_path = None;
                app.frame_wizard.open = false;
                app.report_notice("架構を作成しました。断面タブで断面を割り当ててください");
            }
            Err(e) => app.report_error(e),
        }
        return;
    }
    if !open {
        app.frame_wizard.open = false;
    }
}

/// スパンの入力欄（個別の表 ＋ 等スパン一括入力）。
fn spans_section(
    ui: &mut egui::Ui,
    label: &str,
    spans: &mut Vec<f64>,
    bulk: &mut (usize, f64),
    salt: &str,
) {
    ui.group(|ui| {
        ui.horizontal(|ui| {
            ui.strong(format!("{label}のスパン [mm]"));
            ui.label(format!("（通り {} 本）", spans.len() + 1));
        });
        ui.horizontal_wrapped(|ui| {
            let mut remove = None;
            for (i, s) in spans.iter_mut().enumerate() {
                ui.push_id((salt, i), |ui| {
                    ui.add(egui::DragValue::new(s).speed(100.0).range(1.0..=1.0e5));
                    if ui
                        .small_button("✖")
                        .on_hover_text("このスパンを削除")
                        .clicked()
                    {
                        remove = Some(i);
                    }
                });
            }
            if ui
                .small_button("＋")
                .on_hover_text("スパンを追加")
                .clicked()
            {
                spans.push(bulk.1);
            }
            if let Some(i) = remove {
                spans.remove(i);
            }
        });
        ui.horizontal(|ui| {
            ui.label("等スパン一括入力:");
            ui.add(egui::DragValue::new(&mut bulk.0).range(0..=50));
            ui.label("スパン ×");
            ui.add(
                egui::DragValue::new(&mut bulk.1)
                    .speed(100.0)
                    .range(1.0..=1.0e5),
            );
            ui.label("mm");
            if ui.button("適用").clicked() {
                *spans = vec![bulk.1; bulk.0];
            }
        });
    });
}

/// 階高と階名の入力欄。
fn stories_section(ui: &mut egui::Ui, w: &mut FrameWizardState) {
    ui.group(|ui| {
        ui.strong("階");
        ui.horizontal(|ui| {
            ui.label("等階高一括入力:");
            ui.add(egui::DragValue::new(&mut w.bulk_story.0).range(1..=60));
            ui.label("階 ×");
            ui.add(
                egui::DragValue::new(&mut w.bulk_story.1)
                    .speed(100.0)
                    .range(1.0..=1.0e5),
            );
            ui.label("mm");
            if ui.button("適用").clicked() {
                w.spec.story_heights = vec![w.bulk_story.1; w.bulk_story.0];
                w.spec.story_names.clear();
            }
        });
        // 階名の既定は `default_story_name`（床基準の連番）。最上階も数字で通す。
        let n = w.spec.story_heights.len();
        w.spec.story_names.resize(n, String::new());
        egui::Grid::new("wiz_stories")
            .num_columns(3)
            .show(ui, |ui| {
                ui.label("階");
                ui.label("階高 [mm]");
                ui.label("階名");
                ui.end_row();
                for i in (0..n).rev() {
                    ui.label(format!("{}", i + 1));
                    ui.add(
                        egui::DragValue::new(&mut w.spec.story_heights[i])
                            .speed(100.0)
                            .range(1.0..=1.0e5),
                    );
                    let hint = squid_n_core::model::default_story_name(i);
                    ui.add(
                        egui::TextEdit::singleline(&mut w.spec.story_names[i])
                            .hint_text(hint)
                            .desired_width(80.0),
                    );
                    ui.end_row();
                }
            });
    });
}

/// 柱脚・大梁・床の設定。
fn options_section(ui: &mut egui::Ui, w: &mut FrameWizardState) {
    ui.group(|ui| {
        ui.horizontal(|ui| {
            ui.strong("柱脚:");
            ui.selectable_value(&mut w.spec.base_support, BaseSupport::Fixed, "固定");
            ui.selectable_value(&mut w.spec.base_support, BaseSupport::Pinned, "ピン");
        });
        ui.checkbox(&mut w.spec.with_girders, "大梁を作る");
        ui.checkbox(&mut w.spec.with_slabs, "床を作る")
            .on_hover_text("各階の各格子パネルに 1 枚ずつ。板厚の断面もあわせて作ります");
        if w.spec.with_slabs {
            ui.horizontal(|ui| {
                ui.label("板厚 [mm]:");
                ui.add(
                    egui::DragValue::new(&mut w.spec.slab_thickness)
                        .speed(5.0)
                        .range(1.0..=1000.0),
                );
                ui.label("用途:");
                let cur = USAGE_PRESETS
                    .iter()
                    .find(|(u, _)| *u == w.spec.slab_usage)
                    .map(|(_, l)| *l)
                    .unwrap_or("なし");
                egui::ComboBox::from_id_salt("wiz_slab_usage")
                    .selected_text(cur)
                    .show_ui(ui, |ui| {
                        for (u, label) in USAGE_PRESETS {
                            ui.selectable_value(&mut w.spec.slab_usage, *u, *label);
                        }
                    });
            });
            ui.label(
                egui::RichText::new(
                    "床の材料（コンクリート）は割り当てません。断面タブで割り当てるまで\
                     床の自重は 0 になり、解析前チェックが止めます",
                )
                .size(11.0),
            );
        }
    });
}
