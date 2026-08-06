//! 荷重の追加・編集モーダルと、対象を 3D ビューで選ぶピックモード。
//!
//! ナビゲータ（左パネル）の荷重ツリーを右クリックして開く。対象の節点・部材は
//! 数が多く ID を並べても選べないため、3D ビューでのクリック選択を既定とする。
//!
//! モーダルは対象選択の間だけ閉じる。「3D で選択」を押すと入力内容を保持したまま
//! [`LoadEditor::picking`] を立ててモーダルを閉じ、3D クリックで仮選択、Enter で
//! 確定してモーダルへ戻る（Esc は選び直しを取り消して元の対象へ戻す）。
//! ピック待ちの間はアプリ全体を操作できるため、確定時に対象の存在と、
//! 編集の場合は開いた時点の内容との一致を検証する。

use squid_n_core::ids::{ElemId, LoadCaseId, NodeId};
use squid_n_core::model::{ElementKind, MemberLoad, MemberLoadKind, Model, NodalLoad};

use crate::app::App;

/// 部材荷重の作用方向の選択肢（全体座標）。
const DIR_CHOICES: [(&str, [f64; 3]); 6] = [
    ("鉛直下(-Z)", [0.0, 0.0, -1.0]),
    ("鉛直上(+Z)", [0.0, 0.0, 1.0]),
    ("X+", [1.0, 0.0, 0.0]),
    ("X-", [-1.0, 0.0, 0.0]),
    ("Y+", [0.0, 1.0, 0.0]),
    ("Y-", [0.0, -1.0, 0.0]),
];

/// 部材荷重の作用方向が材軸方向であることを示す選択肢の番号
/// （ブレースはこの選択肢しか選べない。[`brace_axis_dir`] を参照）。
const DIR_ALONG_AXIS: usize = DIR_CHOICES.len();

/// 荷重の種類。ツリーのどのグループから開いたかで決まり、モーダルの間は変わらない。
#[derive(Clone, Debug, PartialEq)]
pub enum LoadDraft {
    Nodal(NodalDraft),
    Member(MemberDraft),
}

/// 節点荷重の入力内容。成分は文字列で保持し、確定時に解釈する
/// （入力途中の `-` や空文字で値が飛ばないようにする）。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NodalDraft {
    pub name: String,
    pub node: Option<NodeId>,
    pub values: [String; 6],
}

/// 部材荷重の入力内容。
#[derive(Clone, Debug, PartialEq)]
pub struct MemberDraft {
    pub name: String,
    pub elem: Option<ElemId>,
    /// 0=中間集中、1=等分布、2=台形。
    pub kind: u8,
    /// [`DIR_CHOICES`] の番号、または [`DIR_ALONG_AXIS`]（材軸方向）。
    pub dir: usize,
    pub a: String,
    pub b: String,
    pub w1: String,
    pub w2: String,
    pub p: String,
}

impl Default for MemberDraft {
    fn default() -> Self {
        Self {
            name: String::new(),
            elem: None,
            kind: 1,
            dir: 0,
            a: "0".into(),
            b: "0".into(),
            w1: "0".into(),
            w2: "0".into(),
            p: "0".into(),
        }
    }
}

/// モーダルが編集している対象。
#[derive(Clone, Debug, PartialEq)]
pub enum LoadEditTarget {
    /// 新規追加。
    New,
    /// 既存荷重の編集。`index` は荷重ケース内の添字、`snapshot` は開いた時点の内容。
    /// ピック待ちの間に他の操作で添字がずれても取り違えないよう、確定時に照合する。
    ExistingNodal { index: usize, snapshot: NodalLoad },
    /// 既存の部材荷重の編集（[`LoadEditTarget::ExistingNodal`] と同じ規約）。
    ExistingMember { index: usize, snapshot: MemberLoad },
}

/// 荷重の追加・編集モーダルの状態。
#[derive(Clone, Debug, PartialEq)]
pub struct LoadEditor {
    /// 対象の荷重ケース。
    pub lc: LoadCaseId,
    pub target: LoadEditTarget,
    pub draft: LoadDraft,
    /// 3D ピック待ち（true の間モーダルは閉じている）。
    pub picking: bool,
    /// ピック待ちに入る直前の対象。Esc で元へ戻すために保持する。
    pick_backup: Option<PickBackup>,
    /// 直前の確定操作で生じたエラー（対象が消えた・内容が変わった等）。
    pub error: Option<String>,
}

/// ピック待ちに入る前の対象（Esc の復元用）。
#[derive(Clone, Copy, Debug, PartialEq)]
enum PickBackup {
    Node(Option<NodeId>),
    Member(Option<ElemId>),
}

impl LoadEditor {
    /// 節点荷重を新規追加するモーダルを開く。`focus_node` が指す節点を初期値にする。
    pub fn new_nodal(lc: LoadCaseId, focus_node: Option<NodeId>) -> Self {
        Self {
            lc,
            target: LoadEditTarget::New,
            draft: LoadDraft::Nodal(NodalDraft {
                node: focus_node,
                values: std::array::from_fn(|_| "0".to_string()),
                ..Default::default()
            }),
            picking: false,
            pick_backup: None,
            error: None,
        }
    }

    /// 部材荷重を新規追加するモーダルを開く。`focus_member` が指す部材を初期値にする。
    pub fn new_member(lc: LoadCaseId, focus_member: Option<ElemId>) -> Self {
        Self {
            lc,
            target: LoadEditTarget::New,
            draft: LoadDraft::Member(MemberDraft {
                elem: focus_member,
                ..Default::default()
            }),
            picking: false,
            pick_backup: None,
            error: None,
        }
    }

    /// 既存の節点荷重を編集するモーダルを開く。
    pub fn edit_nodal(lc: LoadCaseId, index: usize, load: &NodalLoad) -> Self {
        Self {
            lc,
            target: LoadEditTarget::ExistingNodal {
                index,
                snapshot: load.clone(),
            },
            draft: LoadDraft::Nodal(NodalDraft {
                name: load.name.clone(),
                node: Some(load.node),
                values: load.values.map(|v| format!("{}", v)),
            }),
            picking: false,
            pick_backup: None,
            error: None,
        }
    }

    /// 既存の部材荷重を編集するモーダルを開く。
    pub fn edit_member(lc: LoadCaseId, index: usize, load: &MemberLoad, model: &Model) -> Self {
        let (kind, a, b, w1, w2, p) = match load.kind {
            MemberLoadKind::Point { a, p } => (
                0u8,
                format!("{}", a),
                "0".into(),
                "0".into(),
                "0".into(),
                format!("{}", p),
            ),
            MemberLoadKind::Distributed { a, b, w1, w2 } => {
                // 全長かつ強度一定なら等分布、それ以外は台形として開く。
                let full = model
                    .elements
                    .iter()
                    .find(|e| e.id == load.elem)
                    .map(|e| elem_length(model, e))
                    .unwrap_or(0.0);
                let uniform = (w1 - w2).abs() < 1e-9 && a.abs() < 1e-6 && (b - full).abs() < 1e-6;
                (
                    if uniform { 1 } else { 2 },
                    format!("{}", a),
                    format!("{}", b),
                    format!("{}", w1),
                    format!("{}", w2),
                    "0".into(),
                )
            }
        };
        Self {
            lc,
            target: LoadEditTarget::ExistingMember {
                index,
                snapshot: load.clone(),
            },
            draft: LoadDraft::Member(MemberDraft {
                name: load.name.clone(),
                elem: Some(load.elem),
                kind,
                dir: dir_choice_of(load, model),
                a,
                b,
                w1,
                w2,
                p,
            }),
            picking: false,
            pick_backup: None,
            error: None,
        }
    }

    /// ピック待ちへ入る（モーダルを閉じる）。現在の対象を復元用に控える。
    pub fn begin_pick(&mut self) {
        self.pick_backup = Some(match &self.draft {
            LoadDraft::Nodal(d) => PickBackup::Node(d.node),
            LoadDraft::Member(d) => PickBackup::Member(d.elem),
        });
        self.picking = true;
        self.error = None;
    }

    /// ピックを確定してモーダルへ戻る。
    pub fn confirm_pick(&mut self) {
        self.picking = false;
        self.pick_backup = None;
    }

    /// ピックを取り消し、対象を元へ戻してモーダルへ戻る。
    pub fn cancel_pick(&mut self) {
        match (self.pick_backup.take(), &mut self.draft) {
            (Some(PickBackup::Node(n)), LoadDraft::Nodal(d)) => d.node = n,
            (Some(PickBackup::Member(e)), LoadDraft::Member(d)) => d.elem = e,
            _ => {}
        }
        self.picking = false;
    }

    /// 3D でピックした節点を仮選択として反映する（節点荷重のときのみ）。
    pub fn set_picked_node(&mut self, node: NodeId) {
        if let LoadDraft::Nodal(d) = &mut self.draft {
            d.node = Some(node);
        }
    }

    /// 3D でピックした部材を仮選択として反映する（部材荷重のときのみ）。
    /// ブレースを選んだ場合、材軸直交方向の入力は意味を持たないため
    /// 方向を材軸方向へ切り替える（[`brace_axis_dir`] を参照）。
    pub fn set_picked_member(&mut self, elem: ElemId, model: &Model) {
        if let LoadDraft::Member(d) = &mut self.draft {
            d.elem = Some(elem);
            if is_brace(model, elem) {
                d.dir = DIR_ALONG_AXIS;
            } else if d.dir == DIR_ALONG_AXIS {
                d.dir = 0;
            }
        }
    }

    /// ピックの対象が節点か（false なら部材）。
    pub fn picks_node(&self) -> bool {
        matches!(self.draft, LoadDraft::Nodal(_))
    }
}

/// 部材の材端間距離 [mm]。
fn elem_length(model: &Model, elem: &squid_n_core::model::ElementData) -> f64 {
    if elem.nodes.len() < 2 {
        return 0.0;
    }
    let (Some(i), Some(j)) = (
        model.nodes.get(elem.nodes[0].index()),
        model.nodes.get(elem.nodes[1].index()),
    ) else {
        return 0.0;
    };
    let (a, b) = (i.coord, j.coord);
    ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt()
}

/// 指定部材がブレース（トラス要素）か。
pub fn is_brace(model: &Model, elem: ElemId) -> bool {
    model
        .elements
        .iter()
        .any(|e| e.id == elem && matches!(e.kind, ElementKind::Brace { .. }))
}

/// ブレースの材軸方向（i→j の単位ベクトル）。求まらない場合は鉛直下向き。
fn brace_axis_dir(model: &Model, elem: ElemId) -> [f64; 3] {
    let Some(e) = model.elements.iter().find(|e| e.id == elem) else {
        return [0.0, 0.0, -1.0];
    };
    if e.nodes.len() < 2 {
        return [0.0, 0.0, -1.0];
    }
    let (Some(i), Some(j)) = (
        model.nodes.get(e.nodes[0].index()),
        model.nodes.get(e.nodes[1].index()),
    ) else {
        return [0.0, 0.0, -1.0];
    };
    let d = [
        j.coord[0] - i.coord[0],
        j.coord[1] - i.coord[1],
        j.coord[2] - i.coord[2],
    ];
    let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if n < 1e-9 {
        [0.0, 0.0, -1.0]
    } else {
        [d[0] / n, d[1] / n, d[2] / n]
    }
}

/// 既存の部材荷重の方向が、どの選択肢に当たるかを引く。
/// 一致するものがなければ材軸方向として扱う（ブレースの荷重を開いた場合）。
fn dir_choice_of(load: &MemberLoad, model: &Model) -> usize {
    let same = |a: [f64; 3], b: [f64; 3]| {
        (a[0] - b[0]).abs() < 1e-6 && (a[1] - b[1]).abs() < 1e-6 && (a[2] - b[2]).abs() < 1e-6
    };
    DIR_CHOICES
        .iter()
        .position(|(_, d)| same(*d, load.dir))
        .filter(|_| !is_brace(model, load.elem))
        .unwrap_or(DIR_ALONG_AXIS)
}

/// 入力欄の文字列を数値へ。空欄・解釈できない文字列は 0 とする。
fn parse(s: &str) -> f64 {
    s.trim().parse::<f64>().unwrap_or(0.0)
}

impl App {
    /// 荷重の 3D ピック待ちか（ビューアのクリック処理・他の作成モードとの排他判定）。
    pub(crate) fn load_pick_active(&self) -> bool {
        self.load_editor.as_ref().is_some_and(|e| e.picking)
    }

    /// 荷重モーダル・ピックモードの毎フレーム処理。
    /// ビューアより先に呼ぶと、ピック確定のキー入力を 3D クリックと同じフレームで
    /// 拾ってしまうため、中央パネルの描画後に呼ぶ。
    pub(crate) fn load_editor_ui(&mut self, ctx: &egui::Context) {
        if self.load_editor.is_none() {
            return;
        }
        if self.load_pick_active() {
            self.load_pick_bar(ctx);
            return;
        }
        self.load_editor_modal(ctx);
    }

    /// ピック待ち中の案内バー（画面上端）。モーダルは閉じているため、
    /// いま何を求められているか・どう抜けるかをここだけが示す。
    fn load_pick_bar(&mut self, ctx: &egui::Context) {
        let Some(editor) = self.load_editor.as_ref() else {
            return;
        };
        let picks_node = editor.picks_node();
        let current = match &editor.draft {
            LoadDraft::Nodal(d) => d.node.map(|n| format!("節点 N{}", n.0)),
            LoadDraft::Member(d) => d.elem.map(|e| format!("部材 #{}", e.0)),
        };
        let mut confirm = false;
        let mut cancel = false;
        // 3D ビューを覆わないよう画面上端に固定する（移動・折り畳み不可）。
        egui::Window::new("load_pick_bar")
            .title_bar(false)
            .resizable(false)
            .movable(false)
            .anchor(egui::Align2::CENTER_TOP, [0.0, 8.0])
            .show(ctx, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.colored_label(
                        crate::theme::BEST_YELLOW,
                        if picks_node {
                            "荷重の対象を選択中：3D ビューで節点をクリック"
                        } else {
                            "荷重の対象を選択中：3D ビューで部材をクリック"
                        },
                    );
                    match &current {
                        Some(label) => {
                            ui.label(format!("選択中: {}", label));
                        }
                        None => {
                            ui.colored_label(crate::theme::GRAY_600, "未選択");
                        }
                    }
                    confirm = ui
                        .add_enabled(current.is_some(), egui::Button::new("確定 (Enter)"))
                        .clicked();
                    cancel = ui.button("取消 (Esc)").clicked();
                });
            });

        // キー入力は 3D ビューにフォーカスがなくても効くよう ctx から直接読む。
        if ctx.input(|i| i.key_pressed(egui::Key::Enter)) && current.is_some() {
            confirm = true;
        }
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            cancel = true;
        }

        if let Some(editor) = self.load_editor.as_mut() {
            if confirm {
                editor.confirm_pick();
            } else if cancel {
                editor.cancel_pick();
            }
        }
    }

    /// 荷重の追加・編集モーダル本体。
    fn load_editor_modal(&mut self, ctx: &egui::Context) {
        let Some(editor) = self.load_editor.take() else {
            return;
        };
        let mut editor = editor;
        let mut close = false;
        let mut commit = false;
        let mut begin_pick = false;

        let title = match (&editor.target, &editor.draft) {
            (LoadEditTarget::New, LoadDraft::Nodal(_)) => "節点荷重の追加",
            (LoadEditTarget::New, LoadDraft::Member(_)) => "部材荷重の追加",
            (_, LoadDraft::Nodal(_)) => "節点荷重の編集",
            (_, LoadDraft::Member(_)) => "部材荷重の編集",
        };
        let case_label = self
            .model
            .load_cases
            .iter()
            .find(|lc| lc.id == editor.lc)
            .map(|lc| format!("[{}] {}", lc.id.0, lc.name))
            .unwrap_or_else(|| "（不明な荷重ケース）".to_string());

        egui::Modal::new(egui::Id::new("load_editor_modal")).show(ctx, |ui| {
            ui.set_width(420.0);
            ui.heading(title);
            ui.label(format!("荷重ケース: {}", case_label));
            ui.separator();

            match &mut editor.draft {
                LoadDraft::Nodal(d) => {
                    ui.horizontal(|ui| {
                        ui.label("名称:");
                        ui.add(
                            egui::TextEdit::singleline(&mut d.name)
                                .hint_text("未入力可（成分から自動表示）")
                                .desired_width(260.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("対象節点:");
                        match d.node {
                            Some(n) => {
                                let coord = self
                                    .model
                                    .nodes
                                    .get(n.index())
                                    .map(|nd| {
                                        format!(
                                            "N{} ({:.0}, {:.0}, {:.0})",
                                            n.0, nd.coord[0], nd.coord[1], nd.coord[2]
                                        )
                                    })
                                    .unwrap_or_else(|| format!("N{}（存在しません）", n.0));
                                ui.label(coord);
                            }
                            None => {
                                ui.colored_label(crate::theme::BEST_YELLOW, "未選択");
                            }
                        }
                        begin_pick |= ui
                            .button("3D で選択")
                            .on_hover_text(
                                "入力内容を保ったままモーダルを閉じ、3D ビューで節点を選びます",
                            )
                            .clicked();
                    });
                    ui.add_space(4.0);
                    ui.label("荷重成分（力 [N]・モーメント [N·mm]）");
                    egui::Grid::new("load_editor_nodal_values")
                        .num_columns(4)
                        .spacing([8.0, 4.0])
                        .show(ui, |ui| {
                            for (k, label) in
                                ["Fx", "Fy", "Fz", "Mx", "My", "Mz"].into_iter().enumerate()
                            {
                                ui.label(label);
                                ui.add(
                                    egui::TextEdit::singleline(&mut d.values[k])
                                        .desired_width(110.0),
                                );
                                if k % 2 == 1 {
                                    ui.end_row();
                                }
                            }
                        });
                }
                LoadDraft::Member(d) => {
                    ui.horizontal(|ui| {
                        ui.label("名称:");
                        ui.add(
                            egui::TextEdit::singleline(&mut d.name)
                                .hint_text("未入力可（種別から自動表示）")
                                .desired_width(260.0),
                        );
                    });
                    let brace = d.elem.is_some_and(|e| is_brace(&self.model, e));
                    ui.horizontal(|ui| {
                        ui.label("対象部材:");
                        match d.elem {
                            Some(e) => {
                                let kind = self
                                    .model
                                    .elements
                                    .iter()
                                    .find(|el| el.id == e)
                                    .map(|el| format!("{:?}", el.kind))
                                    .unwrap_or_else(|| "存在しません".to_string());
                                ui.label(format!("#{} ({})", e.0, kind));
                            }
                            None => {
                                ui.colored_label(crate::theme::BEST_YELLOW, "未選択");
                            }
                        }
                        begin_pick |= ui
                            .button("3D で選択")
                            .on_hover_text(
                                "入力内容を保ったままモーダルを閉じ、3D ビューで部材を選びます",
                            )
                            .clicked();
                    });
                    if brace {
                        ui.colored_label(
                            crate::theme::GRAY_600,
                            "ブレースは軸剛性のみを持つため、荷重は材軸方向のみ指定できます。\
                             材軸直交方向の荷重は両端の節点へ静定分配されます。",
                        );
                    }
                    ui.horizontal(|ui| {
                        ui.label("種別:");
                        ui.selectable_value(&mut d.kind, 0u8, "中間集中");
                        ui.selectable_value(&mut d.kind, 1u8, "等分布");
                        ui.selectable_value(&mut d.kind, 2u8, "台形");
                    });
                    ui.horizontal(|ui| {
                        ui.label("方向:");
                        if brace {
                            d.dir = DIR_ALONG_AXIS;
                            ui.label("材軸方向");
                        } else {
                            let current = DIR_CHOICES
                                .get(d.dir)
                                .map(|(l, _)| *l)
                                .unwrap_or("鉛直下(-Z)");
                            egui::ComboBox::from_id_salt("load_editor_member_dir")
                                .selected_text(current)
                                .show_ui(ui, |ui| {
                                    for (idx, (label, _)) in DIR_CHOICES.iter().enumerate() {
                                        ui.selectable_value(&mut d.dir, idx, *label);
                                    }
                                });
                        }
                    });
                    match d.kind {
                        0 => {
                            ui.horizontal(|ui| {
                                ui.label("a [mm]:");
                                ui.add(egui::TextEdit::singleline(&mut d.a).desired_width(90.0));
                                ui.label("P [N]:");
                                ui.add(egui::TextEdit::singleline(&mut d.p).desired_width(90.0));
                            });
                        }
                        1 => {
                            ui.horizontal(|ui| {
                                ui.label("w [N/mm]:");
                                ui.add(egui::TextEdit::singleline(&mut d.w1).desired_width(90.0));
                                ui.colored_label(crate::theme::GRAY_600, "材長全体に等分布");
                            });
                        }
                        _ => {
                            ui.horizontal(|ui| {
                                ui.label("a [mm]:");
                                ui.add(egui::TextEdit::singleline(&mut d.a).desired_width(90.0));
                                ui.label("b [mm]:");
                                ui.add(egui::TextEdit::singleline(&mut d.b).desired_width(90.0));
                            });
                            ui.horizontal(|ui| {
                                ui.label("w1 [N/mm]:");
                                ui.add(egui::TextEdit::singleline(&mut d.w1).desired_width(90.0));
                                ui.label("w2 [N/mm]:");
                                ui.add(egui::TextEdit::singleline(&mut d.w2).desired_width(90.0));
                            });
                        }
                    }
                }
            }

            if let Some(err) = &editor.error {
                ui.add_space(4.0);
                ui.colored_label(crate::theme::ERROR_RED, err);
            }

            ui.separator();
            ui.horizontal(|ui| {
                let has_target = match &editor.draft {
                    LoadDraft::Nodal(d) => d.node.is_some(),
                    LoadDraft::Member(d) => d.elem.is_some(),
                };
                let ok_label = if matches!(editor.target, LoadEditTarget::New) {
                    "追加"
                } else {
                    "更新"
                };
                commit = ui
                    .add_enabled(has_target, egui::Button::new(ok_label))
                    .on_disabled_hover_text("対象の節点／部材を選んでください")
                    .clicked();
                close = ui.button("キャンセル").clicked();
            });
        });

        if begin_pick {
            editor.begin_pick();
            self.load_editor = Some(editor);
            return;
        }
        if close {
            return; // editor は take 済みなので、戻さなければ閉じる
        }
        if commit {
            match self.commit_load_editor(&editor) {
                Ok(()) => return,
                Err(msg) => editor.error = Some(msg),
            }
        }
        self.load_editor = Some(editor);
    }

    /// モーダルの入力内容を編集コマンドとして発行する。
    /// 対象が消えている・編集対象の内容が開いた時点と変わっている場合はエラーを返す
    /// （ピック待ちの間にモデルが編集され、添字が別の荷重を指している可能性がある）。
    fn commit_load_editor(&mut self, editor: &LoadEditor) -> Result<(), String> {
        let lc = editor.lc;
        if !self.model.load_cases.iter().any(|c| c.id == lc) {
            return Err("荷重ケースが見つかりません".to_string());
        }
        match &editor.draft {
            LoadDraft::Nodal(d) => {
                let Some(node) = d.node else {
                    return Err("対象の節点が選ばれていません".to_string());
                };
                if node.index() >= self.model.nodes.len() {
                    return Err(format!("節点 N{} は存在しません", node.0));
                }
                let load = NodalLoad {
                    node,
                    values: std::array::from_fn(|k| parse(&d.values[k])),
                    name: d.name.trim().to_string(),
                    source: squid_n_core::model::LoadSource::Manual,
                };
                match &editor.target {
                    LoadEditTarget::New => {
                        self.undo.run(
                            &mut self.model,
                            Box::new(squid_n_edit::AddNodalLoad { lc, load }),
                        );
                    }
                    LoadEditTarget::ExistingNodal { index, snapshot } => {
                        self.verify_nodal_snapshot(lc, *index, snapshot)?;
                        self.undo.run(
                            &mut self.model,
                            Box::new(squid_n_edit::SetNodalLoad {
                                lc,
                                index: *index,
                                load,
                            }),
                        );
                    }
                    LoadEditTarget::ExistingMember { .. } => {
                        return Err("編集対象の種類が一致しません".to_string())
                    }
                }
            }
            LoadDraft::Member(d) => {
                let Some(elem) = d.elem else {
                    return Err("対象の部材が選ばれていません".to_string());
                };
                let Some(element) = self.model.elements.iter().find(|e| e.id == elem) else {
                    return Err(format!("部材 #{} は存在しません", elem.0));
                };
                let length = elem_length(&self.model, element);
                if length <= 1e-9 {
                    return Err(format!("部材 #{} の材長が 0 です", elem.0));
                }
                let dir = if d.dir == DIR_ALONG_AXIS {
                    brace_axis_dir(&self.model, elem)
                } else {
                    DIR_CHOICES[d.dir].1
                };
                let kind = match d.kind {
                    0 => MemberLoadKind::Point {
                        a: parse(&d.a),
                        p: parse(&d.p),
                    },
                    1 => MemberLoadKind::Distributed {
                        a: 0.0,
                        b: length,
                        w1: parse(&d.w1),
                        w2: parse(&d.w1),
                    },
                    _ => MemberLoadKind::Distributed {
                        a: parse(&d.a),
                        b: parse(&d.b),
                        w1: parse(&d.w1),
                        w2: parse(&d.w2),
                    },
                };
                if let MemberLoadKind::Distributed { a, b, .. } = kind {
                    if b <= a {
                        return Err("分布区間は b > a となるように入力してください".to_string());
                    }
                }
                let load = MemberLoad {
                    elem,
                    dir,
                    kind,
                    name: d.name.trim().to_string(),
                    source: squid_n_core::model::LoadSource::Manual,
                };
                match &editor.target {
                    LoadEditTarget::New => {
                        self.undo.run(
                            &mut self.model,
                            Box::new(squid_n_edit::AddMemberLoad { lc, load }),
                        );
                    }
                    LoadEditTarget::ExistingMember { index, snapshot } => {
                        self.verify_member_snapshot(lc, *index, snapshot)?;
                        self.undo.run(
                            &mut self.model,
                            Box::new(squid_n_edit::SetMemberLoad {
                                lc,
                                index: *index,
                                load,
                            }),
                        );
                    }
                    LoadEditTarget::ExistingNodal { .. } => {
                        return Err("編集対象の種類が一致しません".to_string())
                    }
                }
            }
        }
        self.staleness.mark_edited();
        Ok(())
    }

    /// 編集対象の節点荷重が、モーダルを開いた時点の内容のままか確認する。
    fn verify_nodal_snapshot(
        &self,
        lc: LoadCaseId,
        index: usize,
        snapshot: &NodalLoad,
    ) -> Result<(), String> {
        let case = self
            .model
            .load_cases
            .iter()
            .find(|c| c.id == lc)
            .ok_or_else(|| "荷重ケースが見つかりません".to_string())?;
        match case.nodal.get(index) {
            Some(cur) if cur == snapshot => Ok(()),
            _ => Err(STALE_TARGET_MESSAGE.to_string()),
        }
    }

    /// 編集対象の部材荷重が、モーダルを開いた時点の内容のままか確認する。
    fn verify_member_snapshot(
        &self,
        lc: LoadCaseId,
        index: usize,
        snapshot: &MemberLoad,
    ) -> Result<(), String> {
        let case = self
            .model
            .load_cases
            .iter()
            .find(|c| c.id == lc)
            .ok_or_else(|| "荷重ケースが見つかりません".to_string())?;
        match case.member.get(index) {
            Some(cur) if cur == snapshot => Ok(()),
            _ => Err(STALE_TARGET_MESSAGE.to_string()),
        }
    }
}

/// 編集対象が入れ替わっていたときの案内。
const STALE_TARGET_MESSAGE: &str =
    "編集中に対象の荷重が変更・削除されました。閉じてから選び直してください";
