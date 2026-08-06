//! ナビゲータ（左パネル）の荷重ケースツリー。
//!
//! 「荷重ケース → 種別グループ（節点荷重／部材荷重）→ 個別の荷重」の 3 階層で、
//! 右クリックのコンテキストメニューから荷重の追加・編集・削除を行う。
//! 追加・編集は [`crate::load_editor`] のモーダルへ渡す。
//!
//! ツリーに並べるのは**利用者が入力した荷重だけ**で、準備計算が生成した荷重
//! （床荷重の分配・自重・Ai 分布の水平力）は表示しない。自動生成分は同期のたびに
//! 作り直されるため利用者が触る余地がなく、ここに出すと編集できない行が大量に
//! 並んで手入力の荷重が埋もれる。内容は下ドックの「荷重」タブで確認できる。

use super::*;
use crate::load_editor::LoadEditor;
use crate::tables::loads::{member_load_display_name, nodal_load_display_name};

/// ツリーの右クリックメニューが要求した操作。
/// メニューのクロージャ内では `self` を可変借用できないため、一旦ここへ退避して
/// クロージャの外で適用する。
enum LoadTreeAction {
    AddCase,
    AddNodal(LoadCaseId),
    AddMember(LoadCaseId),
    EditNodal(LoadCaseId, usize),
    EditMember(LoadCaseId, usize),
    DeleteNodal(LoadCaseId, usize),
    DeleteMember(LoadCaseId, usize),
    DeleteCase(LoadCaseId),
}

impl App {
    /// 荷重ケースツリー（ナビゲータ内の 1 セクション）。
    pub(crate) fn nav_load_cases(&mut self, ui: &mut egui::Ui) {
        let mut action: Option<LoadTreeAction> = None;

        let header = egui::CollapsingHeader::new("荷重ケース")
            .default_open(true)
            .id_salt("nav_load_cases");
        let resp = header.show(ui, |ui| {
            if self.model.load_cases.is_empty() {
                ui.colored_label(crate::theme::GRAY_600, "荷重ケースがありません");
            }
            for i in 0..self.model.load_cases.len() {
                self.nav_load_case_node(ui, i, &mut action);
            }
        });
        // セクション見出しの右クリック＝どのケースにも属さない操作（ケース追加）。
        resp.header_response.context_menu(|ui| {
            if ui.button("荷重ケースを追加").clicked() {
                action = Some(LoadTreeAction::AddCase);
                ui.close();
            }
        });

        if let Some(action) = action {
            self.apply_load_tree_action(action);
        }
    }

    /// 荷重ケース 1 件のノードと、その配下の種別グループ。
    fn nav_load_case_node(
        &mut self,
        ui: &mut egui::Ui,
        case_index: usize,
        action: &mut Option<LoadTreeAction>,
    ) {
        let lc_id = self.model.load_cases[case_index].id;
        let label = format!(
            "[{}] {}",
            case_index, self.model.load_cases[case_index].name
        );
        let is_sel = self.nav.focus_load_case == Some(lc_id);
        let referenced = self.load_case_referenced_by_combination(lc_id);

        let header = egui::CollapsingHeader::new(egui::RichText::new(label).strong())
            .default_open(false)
            .id_salt(("nav_lc", lc_id.0));
        let resp = header.show(ui, |ui| {
            self.nav_load_group(ui, case_index, LoadGroup::Nodal, action);
            self.nav_load_group(ui, case_index, LoadGroup::Member, action);
        });

        if resp.header_response.clicked() {
            self.nav.focus_load_case = Some(lc_id);
        }
        if is_sel {
            // 選択中のケースは見出しの下に細い下線を引く（CollapsingHeader は
            // selectable_label と違い選択状態を持たないため）。
            let rect = resp.header_response.rect;
            ui.painter().line_segment(
                [rect.left_bottom(), rect.right_bottom()],
                egui::Stroke::new(2.0_f32, crate::theme::DATA_BLUE),
            );
        }
        resp.header_response.context_menu(|ui| {
            if ui.button("節点荷重を追加").clicked() {
                *action = Some(LoadTreeAction::AddNodal(lc_id));
                ui.close();
            }
            if ui.button("部材荷重を追加").clicked() {
                *action = Some(LoadTreeAction::AddMember(lc_id));
                ui.close();
            }
            ui.separator();
            let del = ui.add_enabled(!referenced, egui::Button::new("この荷重ケースを削除"));
            if referenced {
                del.on_disabled_hover_text("荷重組合せから参照中のため削除できません");
            } else if del.clicked() {
                *action = Some(LoadTreeAction::DeleteCase(lc_id));
                ui.close();
            }
        });
    }

    /// 種別グループ（節点荷重／部材荷重）のノードと、その配下の個別荷重。
    fn nav_load_group(
        &mut self,
        ui: &mut egui::Ui,
        case_index: usize,
        group: LoadGroup,
        action: &mut Option<LoadTreeAction>,
    ) {
        let lc_id = self.model.load_cases[case_index].id;
        // 表示するのは手入力分のみ。添字は編集・削除コマンドが使うため一緒に持つ。
        let entries: Vec<(usize, String)> = match group {
            LoadGroup::Nodal => self.model.load_cases[case_index]
                .manual_nodal()
                .map(|(i, nl)| {
                    (
                        i,
                        format!("N{}  {}", nl.node.0, nodal_load_display_name(nl)),
                    )
                })
                .collect(),
            LoadGroup::Member => self.model.load_cases[case_index]
                .manual_member()
                .map(|(i, ml)| {
                    (
                        i,
                        format!("#{}  {}", ml.elem.0, member_load_display_name(ml)),
                    )
                })
                .collect(),
        };

        let header = egui::CollapsingHeader::new(format!("{} ({})", group.label(), entries.len()))
            .default_open(false)
            .id_salt(("nav_lc_group", lc_id.0, group.salt()));
        let resp = header.show(ui, |ui| {
            if entries.is_empty() {
                ui.colored_label(crate::theme::GRAY_600, "（なし）");
            }
            for (index, label) in &entries {
                let leaf = ui
                    .selectable_label(false, label)
                    .on_hover_text("クリックで対象を 3D ビューで選択／右クリックで編集・削除");
                if leaf.clicked() {
                    self.focus_load_target(case_index, group, *index);
                }
                leaf.context_menu(|ui| {
                    if ui.button("値を編集").clicked() {
                        *action = Some(match group {
                            LoadGroup::Nodal => LoadTreeAction::EditNodal(lc_id, *index),
                            LoadGroup::Member => LoadTreeAction::EditMember(lc_id, *index),
                        });
                        ui.close();
                    }
                    if ui.button("この荷重を削除").clicked() {
                        *action = Some(match group {
                            LoadGroup::Nodal => LoadTreeAction::DeleteNodal(lc_id, *index),
                            LoadGroup::Member => LoadTreeAction::DeleteMember(lc_id, *index),
                        });
                        ui.close();
                    }
                });
            }
        });
        resp.header_response.context_menu(|ui| {
            if ui.button(group.add_label()).clicked() {
                *action = Some(match group {
                    LoadGroup::Nodal => LoadTreeAction::AddNodal(lc_id),
                    LoadGroup::Member => LoadTreeAction::AddMember(lc_id),
                });
                ui.close();
            }
        });
    }

    /// 個別荷重の左クリック：荷重が載っている節点・部材を 3D の注目対象にする。
    fn focus_load_target(&mut self, case_index: usize, group: LoadGroup, index: usize) {
        match group {
            LoadGroup::Nodal => {
                if let Some(nl) = self.model.load_cases[case_index].nodal.get(index) {
                    self.nav.focus_node = Some(nl.node);
                    self.selection.nodes = vec![nl.node];
                    self.selection.members.clear();
                }
            }
            LoadGroup::Member => {
                if let Some(ml) = self.model.load_cases[case_index].member.get(index) {
                    self.nav.focus_member = Some(ml.elem);
                    self.selection.members = vec![ml.elem];
                    self.selection.nodes.clear();
                }
            }
        }
    }

    /// 指定の荷重ケースを参照している荷重組合せがあるか（ケース削除の可否）。
    fn load_case_referenced_by_combination(&self, lc: LoadCaseId) -> bool {
        self.model
            .combinations
            .iter()
            .any(|c| c.terms.iter().any(|(id, _)| *id == lc))
    }

    /// ツリーのメニューが要求した操作を適用する。
    fn apply_load_tree_action(&mut self, action: LoadTreeAction) {
        match action {
            LoadTreeAction::AddCase => {
                let name = format!("LC{}", self.model.load_cases.len());
                self.undo.run(
                    &mut self.model,
                    Box::new(squid_n_edit::AddLoadCase { name }),
                );
                self.nav.focus_load_case = self.model.load_cases.last().map(|lc| lc.id);
                self.staleness.mark_edited();
            }
            LoadTreeAction::AddNodal(lc) => {
                self.open_load_editor(LoadEditor::new_nodal(lc, self.nav.focus_node));
            }
            LoadTreeAction::AddMember(lc) => {
                self.open_load_editor(LoadEditor::new_member(lc, self.nav.focus_member));
            }
            LoadTreeAction::EditNodal(lc, index) => {
                let Some(load) = self
                    .model
                    .load_cases
                    .iter()
                    .find(|c| c.id == lc)
                    .and_then(|c| c.nodal.get(index))
                    .cloned()
                else {
                    return;
                };
                self.open_load_editor(LoadEditor::edit_nodal(lc, index, &load));
            }
            LoadTreeAction::EditMember(lc, index) => {
                let Some(load) = self
                    .model
                    .load_cases
                    .iter()
                    .find(|c| c.id == lc)
                    .and_then(|c| c.member.get(index))
                    .cloned()
                else {
                    return;
                };
                let editor = LoadEditor::edit_member(lc, index, &load, &self.model);
                self.open_load_editor(editor);
            }
            LoadTreeAction::DeleteNodal(lc, index) => {
                self.undo.run(
                    &mut self.model,
                    Box::new(squid_n_edit::DeleteNodalLoad { lc, index }),
                );
                self.staleness.mark_edited();
            }
            LoadTreeAction::DeleteMember(lc, index) => {
                self.undo.run(
                    &mut self.model,
                    Box::new(squid_n_edit::DeleteMemberLoad { lc, index }),
                );
                self.staleness.mark_edited();
            }
            LoadTreeAction::DeleteCase(lc) => {
                self.undo.run(
                    &mut self.model,
                    Box::new(squid_n_edit::DeleteLoadCase { id: lc }),
                );
                if self.nav.focus_load_case == Some(lc) {
                    self.nav.focus_load_case = None;
                }
                if self.last_static == Some(StaticKey::Case(StaticCaseKey::User(lc))) {
                    self.last_static = None;
                }
                self.staleness.mark_edited();
            }
        }
    }

    /// 荷重モーダルを開く。3D ビューが出ないタブにいる場合はモデルタブへ移す
    /// （対象の節点・部材は 3D クリックで選ぶため、ビューが必要）。
    fn open_load_editor(&mut self, editor: LoadEditor) {
        if !matches!(self.active_tab, Tab::Model | Tab::Loads | Tab::Analysis) {
            self.active_tab = Tab::Model;
        }
        self.nav.focus_load_case = Some(editor.lc);
        self.load_editor = Some(editor);
    }
}

/// 荷重ケース配下の種別グループ。
#[derive(Clone, Copy, PartialEq, Eq)]
enum LoadGroup {
    Nodal,
    Member,
}

impl LoadGroup {
    fn label(self) -> &'static str {
        match self {
            LoadGroup::Nodal => "節点荷重",
            LoadGroup::Member => "部材荷重",
        }
    }

    fn add_label(self) -> &'static str {
        match self {
            LoadGroup::Nodal => "節点荷重を追加",
            LoadGroup::Member => "部材荷重を追加",
        }
    }

    /// `CollapsingHeader` の id_salt に混ぜる識別子。
    fn salt(self) -> u8 {
        match self {
            LoadGroup::Nodal => 0,
            LoadGroup::Member => 1,
        }
    }
}
