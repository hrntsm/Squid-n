//! 階への複製のダイアログ（`① 準備計算 > 階の定義 > ⧉`）。
//!
//! ある階で決めた断面・荷重・床割り・二次部材を、同じ平面位置の相手へ配る。
//! 実務では下の階で決めてから上の階へ配り、階ごとに差分を直すという進め方をとるため、
//! その「配る」操作をひとまとめにしたものである。
//!
//! 複製の規則は [`squid_n_edit::CopyStory`] が持つ。ここは複製元・複製先・対象の
//! 選択と、実行前の見込み表示だけを受け持つ。
//!
//! # 実行前に見せるもの
//!
//! 複製は断面を新しく作り、床を増やし、荷重を載せ替える。取り消せるとはいえ影響が
//! 広いため、[`CopyStory::preview`] でモデルを変えずに試した結果を先に見せる。
//! 新しく作る断面の符号＋階も列挙し、断面表がどう増えるかを実行前に確かめられる。

use crate::app::App;
use squid_n_core::ids::StoryId;
use squid_n_edit::{CopyStory, CopyStoryReport, CopyTargets};

/// 事前表示が依存するもの。モデルの版（[`squid_n_edit::UndoStack::revision`]）を
/// 含めるため、複製元・複製先・対象が同じでもモデルを編集すれば計算し直される。
type PreviewKey = (StoryId, Vec<StoryId>, CopyTargets, u64);

/// 事前表示のキャッシュ。
///
/// [`CopyStory::preview`] はモデルを丸ごと複製して試算するため、毎フレーム呼ぶと
/// モデルの大きさに比例した複製が 60 回／秒で走る。選択とモデルの版が変わったときだけ
/// 計算し直す。
#[derive(Debug, Clone)]
struct PreviewCache {
    key: PreviewKey,
    report: CopyStoryReport,
    new_sections: Vec<String>,
}

/// ダイアログの入力状態。`App` が保持し、ウィンドウを閉じても内容を保つ。
#[derive(Debug, Clone, Default)]
pub struct StoryCopyState {
    pub open: bool,
    /// 複製元の階。階の削除で消えることがあるため、毎フレーム実在を確かめる。
    pub from: Option<StoryId>,
    /// 複製先の選択（`model.stories` と同順・同数）。
    pub to: Vec<bool>,
    pub targets: CopyTargets,
    /// 直前の実行結果（実行後もダイアログへ残して結果を確認できるようにする）。
    pub report: Option<CopyStoryReport>,
    /// 事前表示のキャッシュ。
    preview: Option<PreviewCache>,
}

impl StoryCopyState {
    /// 選択中の複製先を [`StoryId`] の並びにする。
    fn targets_to(&self) -> Vec<StoryId> {
        self.to
            .iter()
            .enumerate()
            .filter(|(_, on)| **on)
            .map(|(i, _)| StoryId(i as u32))
            .collect()
    }
}

/// ダイアログを開く（複製元を選び直し、前回の結果を消す）。
///
/// 対象の既定は全部入りとする。一部だけ配りたい場合に外す使い方を想定している。
pub fn open(app: &mut App, from: StoryId) {
    app.story_copy.open = true;
    app.story_copy.from = Some(from);
    app.story_copy.report = None;
    app.story_copy.preview = None;
    app.story_copy.to = vec![false; app.model.stories.len()];
    app.story_copy.targets = CopyTargets {
        sections: true,
        loads: true,
        slabs: true,
        secondary: true,
    };
}

/// ダイアログのウィンドウを描く。
pub fn story_copy_window(ctx: &egui::Context, app: &mut App) {
    if !app.story_copy.open {
        return;
    }
    // 階の追加・削除で並びが変わるため、選択の長さを毎フレームそろえる。
    let n = app.model.stories.len();
    app.story_copy.to.resize(n, false);
    if app.story_copy.from.is_some_and(|s| s.index() >= n) {
        app.story_copy.from = None;
    }

    let mut open = true;
    let mut run = false;
    egui::Window::new("階への複製")
        .open(&mut open)
        .resizable(true)
        .default_width(460.0)
        .show(ctx, |ui| {
            if n == 0 {
                ui.label(
                    "階が定義されていません。先に階を追加するか、準備計算を実行してください。",
                );
                return;
            }
            // 所属階は準備計算が付けるため、未実行だと配る相手を 1 つも見つけられない。
            if !app.model.nodes.iter().any(|nd| nd.story.is_some()) {
                ui.colored_label(
                    crate::theme::BEST_YELLOW,
                    "⚠ 節点の所属階が未設定です。先に準備計算を実行してください\
                     （所属階が無いと複製する相手を見つけられません）。",
                );
                ui.separator();
            }

            from_section(ui, app);
            ui.add_space(4.0);
            to_section(ui, app);
            ui.add_space(4.0);
            targets_section(ui, app);
            ui.separator();
            run = preview_section(ui, app);
        });

    // 実行ボタンは事前表示が埋まっているときだけ出るため、ここでは両方そろう。
    if let (true, Some(from), Some(cache)) =
        (run, app.story_copy.from, app.story_copy.preview.clone())
    {
        let cmd = CopyStory {
            from,
            to: app.story_copy.targets_to(),
            targets: app.story_copy.targets,
        };
        if app.undo.run(&mut app.model, Box::new(cmd)) {
            app.staleness.mark_edited();
            app.report_notice(format!("階へ複製しました: {}", cache.report.summary()));
            app.story_copy.report = Some(cache.report);
        } else {
            app.report_notice("複製するものがありませんでした");
        }
    }
    if !open {
        app.story_copy.open = false;
    }
}

/// 複製元の選択。
fn from_section(ui: &mut egui::Ui, app: &mut App) {
    let names: Vec<(StoryId, String)> = app
        .model
        .stories
        .iter()
        .map(|s| (s.id, s.name.clone()))
        .collect();
    ui.horizontal(|ui| {
        ui.strong("複製元:");
        let current = app
            .story_copy
            .from
            .and_then(|f| names.iter().find(|(id, _)| *id == f))
            .map(|(_, n)| n.as_str())
            .unwrap_or("（選択してください）");
        egui::ComboBox::from_id_salt("story_copy_from")
            .selected_text(current)
            .show_ui(ui, |ui| {
                // 上階から順に並べる（階の一覧と同じ見え方にする）。
                for (id, name) in names.iter().rev() {
                    ui.selectable_value(&mut app.story_copy.from, Some(*id), name);
                }
            });
    });
    // 複製元へ配る意味はないため、複製先の選択からは外す。
    if let Some(f) = app.story_copy.from {
        if let Some(on) = app.story_copy.to.get_mut(f.index()) {
            *on = false;
        }
    }
}

/// 複製先の選択（複数可）。
fn to_section(ui: &mut egui::Ui, app: &mut App) {
    let names: Vec<String> = app.model.stories.iter().map(|s| s.name.clone()).collect();
    let from = app.story_copy.from;
    ui.group(|ui| {
        ui.horizontal(|ui| {
            ui.strong("複製先:");
            if ui.small_button("すべて選択").clicked() {
                for (i, on) in app.story_copy.to.iter_mut().enumerate() {
                    *on = from != Some(StoryId(i as u32));
                }
            }
            if ui.small_button("すべて解除").clicked() {
                app.story_copy.to.fill(false);
            }
        });
        ui.horizontal_wrapped(|ui| {
            for (i, name) in names.iter().enumerate().rev() {
                let id = StoryId(i as u32);
                ui.add_enabled_ui(from != Some(id), |ui| {
                    ui.checkbox(&mut app.story_copy.to[i], name);
                });
            }
        });
    });
}

/// 複製する対象の選択。
fn targets_section(ui: &mut egui::Ui, app: &mut App) {
    let t = &mut app.story_copy.targets;
    ui.group(|ui| {
        ui.strong("複製する対象");
        ui.checkbox(&mut t.sections, "断面の割当")
            .on_hover_text("複製先の階名で断面を複製してから割り当てます（符号は変えません）");
        ui.checkbox(&mut t.loads, "荷重").on_hover_text(
            "手入力の節点荷重・部材荷重と、床の面荷重・用途を配ります。\
             複製先の手入力荷重は取り除いてから載せ替えます",
        );
        ui.checkbox(&mut t.slabs, "床")
            .on_hover_text("複製先に同じ位置の床が無ければ作ります（既にあれば形はそのまま）");
        ui.checkbox(&mut t.secondary, "二次部材")
            .on_hover_text("小梁・間柱を配ります");
    });
}

/// 事前表示を必要なときだけ作り直す。
fn refresh_preview(app: &mut App, from: StoryId, to: Vec<StoryId>) -> &PreviewCache {
    let key: PreviewKey = (
        from,
        to.clone(),
        app.story_copy.targets,
        app.undo.revision(),
    );
    if app.story_copy.preview.as_ref().is_none_or(|c| c.key != key) {
        let cmd = CopyStory {
            from,
            to,
            targets: app.story_copy.targets,
        };
        app.story_copy.preview = Some(PreviewCache {
            key,
            report: cmd.preview(&app.model),
            new_sections: cmd.new_section_labels(&app.model),
        });
    }
    app.story_copy
        .preview
        .as_ref()
        .expect("直前に埋めているので必ずある")
}

/// 実行前の見込みと実行ボタン。押されたら真を返す。
#[must_use]
fn preview_section(ui: &mut egui::Ui, app: &mut App) -> bool {
    let Some(from) = app.story_copy.from else {
        ui.colored_label(crate::theme::GRAY_600, "複製元の階を選んでください。");
        return false;
    };
    let to = app.story_copy.targets_to();
    if to.is_empty() {
        ui.colored_label(crate::theme::GRAY_600, "複製先の階を選んでください。");
        return false;
    }
    if !app.story_copy.targets.any() {
        ui.colored_label(crate::theme::GRAY_600, "複製する対象を選んでください。");
        return false;
    }

    let cache = refresh_preview(app, from, to).clone();
    ui.label(format!("見込み: {}", cache.report.summary()));
    if !cache.new_sections.is_empty() {
        ui.collapsing(
            format!("新しく作る断面 {} 件", cache.new_sections.len()),
            |ui| {
                for label in &cache.new_sections {
                    ui.label(label);
                }
            },
        );
    }

    let clicked = ui
        .add_enabled(
            cache.report.changed(),
            egui::Button::new("✅ この内容で複製"),
        )
        .on_hover_text("undo 1 回で元に戻せます")
        .on_disabled_hover_text("この組み合わせで配れるものがありません")
        .clicked();

    if let Some(done) = &app.story_copy.report {
        ui.separator();
        ui.colored_label(
            crate::theme::GOOD_GREEN,
            format!("実行結果: {}", done.summary()),
        );
    }
    clicked
}
