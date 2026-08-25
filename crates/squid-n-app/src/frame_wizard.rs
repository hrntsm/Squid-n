//! 架構作成ウィザード。スパンと階高を入力して、柱・大梁・柱脚支点・通り芯・階・床を
//! 一括生成する（`ファイル > 新規（架構ウィザード）…`）。
//!
//! 床のコンクリートだけは材料も作る。床は解析対象外の二次部材で、材料は自重を決める
//! 入力にすぎないためである（柱・大梁の断面・材料は利用者が決める）。
//!
//! **新規モデルを作る操作**であり、現在のモデルを置き換える。既存モデルへ架構を
//! 足す使い方は想定していない（既存節点との突き合わせ規則を決める必要があり、
//! 3D ビューの格子点スナップで代替できる）。
//!
//! 生成の規則は [`squid_n_core::frame_gen`] が持つ。ここは入力欄と確認の表示だけを
//! 受け持ち、寸法から架構を組み立てる判断はコアへ委ねる。

use crate::app::App;
use squid_n_core::frame_gen::{BaseSupport, FrameSpec};
use squid_n_core::material_grade::material_presets;
use squid_n_core::model::{MaterialCategory, SlabUsage};

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

/// 階の一覧をスクロールに切り替える高さ [px]。1 行およそ 24px で、見出し行を含めて
/// 10 階前後までがスクロールなしで収まる。
const STORY_ROWS_MAX_HEIGHT: f32 = 260.0;

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
    // 各セクションを個別にスクロールさせても合計の高さは小さい画面を超えうるため、
    // 中身全体も高さで頭打ちにし、あふれた分はここで拾う。これが無いと下端の
    // 「この内容で作成」が画面外へ出て押せない。
    //
    // `Window::vscroll` は使わない。ウィンドウ側のスクロールは縦に縮まない設定
    // （`auto_shrink(false)`）で作られるため、既定の 3 階でもウィンドウが規定の高さまで
    // 広がって余白とスクロールバーが出てしまう。ここで `auto_shrink` の縦を有効にした
    // 自前のスクロールを置けば、中身が収まるうちはウィンドウが中身どおりの高さになる。
    //
    // 残りの 15% はタイトルバーとウィンドウの余白の分。
    //
    // 同じ高さを `default_height` にも渡す。スクロールを挟むとウィンドウは中身ではなく
    // 割り当てられた高さまでしか広がらないため、既定の 420px のままでは画面が広くても
    // そこで頭打ちになってしまう。ウィンドウ自体は中身の高さで描かれるので、中身が短い
    // ときにここが余白になることはない。
    let body_max_height = ctx.content_rect().height() * 0.85;
    egui::Window::new("新規（架構ウィザード）")
        .open(&mut open)
        .resizable(true)
        .default_width(520.0)
        .default_height(body_max_height)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical()
                .id_salt("wiz_body")
                .max_height(body_max_height)
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    ui.label(
                        "スパンと階高を入力すると、節点・柱・大梁・柱脚支点・通り芯・階・床を\
                 まとめて作ります。柱・大梁の断面と材料は作りませんので、生成後に\
                 断面タブで割り当ててください。",
                    );
                    ui.colored_label(
                        crate::theme::WARN_TEXT,
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
                                "生成: 節点 {} ・柱 {} 本 ・梁 {} 本（基礎梁を含む）・床 {} 枚",
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
        // 「＋」はスパン全体の本数を変える操作で、個々のスパンに属する「✖」とは役割が違う。
        // 列の末尾に置くとスパンが増えるほど右へ流れて押しにくいため、ヘッダ側に固定する。
        let mut added = false;
        ui.horizontal(|ui| {
            ui.strong(format!("{label}のスパン [mm]"));
            ui.label(format!("（通り {} 本）", spans.len() + 1));
            if ui
                .small_button("＋")
                .on_hover_text("スパンを追加")
                .clicked()
            {
                spans.push(bulk.1);
                added = true;
            }
        });
        // 折り返さずに 1 列へ並べ、横スクロールで奥を見る。折り返すとウィンドウ幅で段の
        // 位置が変わり、左から i 番目という並び順と通り番号の対応が読み取りにくくなる。
        egui::ScrollArea::horizontal()
            .id_salt(salt)
            .auto_shrink([false, true])
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    let mut remove = None;
                    let last = spans.len().saturating_sub(1);
                    for (i, s) in spans.iter_mut().enumerate() {
                        ui.push_id((salt, i), |ui| {
                            let resp =
                                ui.add(egui::DragValue::new(s).speed(100.0).range(1.0..=1.0e5));
                            // 追加した欄は列の右端に現れるので、その回だけ右端まで送る。
                            // `stick_to_right` では利用者が途中までスクロールしている間
                            // 追従しないため、追加した欄を名指しで送る。縦の指示も同時に
                            // 立つが、この ScrollArea が両方向とも回収するので外側の
                            // 縦スクロールは動かない。
                            if added && i == last {
                                resp.scroll_to_me(Some(egui::Align::Max));
                            }
                            if ui
                                .small_button("✖")
                                .on_hover_text("このスパンを削除")
                                .clicked()
                            {
                                remove = Some(i);
                            }
                        });
                    }
                    if let Some(i) = remove {
                        spans.remove(i);
                    }
                });
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
        // 階（床）は層より 1 つ多い。行は床ごとに並べ、階高は「その床とすぐ下の床の
        // 間」＝層の高さなので、基部の床の行には階高欄を置かない。
        // 階名の既定は `default_story_name`（床基準の連番）。最上階も数字で通す。
        let n = w.spec.story_heights.len();
        w.spec.story_names.resize(n + 1, String::new());
        // 階は最大 60 まで増やせる。約 10 行分で打ち切り、それを超える分はスクロールで
        // 見る。階数が少ないうちは縦に縮ませたいので auto_shrink の縦は true。
        egui::ScrollArea::vertical()
            .id_salt("wiz_stories_scroll")
            .max_height(STORY_ROWS_MAX_HEIGHT)
            .auto_shrink([false, true])
            .show(ui, |ui| {
                egui::Grid::new("wiz_stories")
                    .num_columns(3)
                    .show(ui, |ui| {
                        ui.label("階");
                        ui.label("階高 [mm]");
                        ui.label("階名");
                        ui.end_row();
                        // 床 fi（0 = 基部）を上から順に描く。床 fi の階高は
                        // 層 fi-1 の高さ（`story_heights[fi - 1]`）。
                        for fi in (0..=n).rev() {
                            ui.label(format!("{}", fi + 1));
                            match fi.checked_sub(1) {
                                Some(li) => {
                                    ui.add(
                                        egui::DragValue::new(&mut w.spec.story_heights[li])
                                            .speed(100.0)
                                            .range(1.0..=1.0e5),
                                    );
                                }
                                None => {
                                    ui.label("—").on_hover_text(
                                        "基部（柱脚・基礎梁のレベル）。階高は上の階が持ちます",
                                    );
                                }
                            }
                            let hint = squid_n_core::model::default_story_name(fi);
                            ui.add(
                                egui::TextEdit::singleline(&mut w.spec.story_names[fi])
                                    .hint_text(hint)
                                    .desired_width(80.0),
                            );
                            ui.end_row();
                        }
                    });
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
            .on_hover_text(
                "各階の各格子区画に 1 枚ずつ。板厚の断面とコンクリートもあわせて作ります",
            );
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
            ui.horizontal(|ui| {
                ui.label("コンクリート:");
                // 選択肢は材料タブと同じ標準材料プリセット（Fc18〜Fc60）。床の自重は
                // この材料の密度から決まるため、割り当てない選択肢は設けない。
                egui::ComboBox::from_id_salt("wiz_slab_concrete")
                    .selected_text(&w.spec.slab_concrete)
                    .show_ui(ui, |ui| {
                        for p in material_presets()
                            .iter()
                            .filter(|p| p.category == MaterialCategory::Concrete)
                        {
                            ui.selectable_value(
                                &mut w.spec.slab_concrete,
                                p.name.to_string(),
                                p.name,
                            );
                        }
                    });
                ui.label(egui::RichText::new("この材料を作って床の断面へ割り当てます").size(11.0));
            });
        }
    });
}
