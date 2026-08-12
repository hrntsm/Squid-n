//! `App` の egui パネル描画メソッド。

use super::*;
use crate::table_util::fmt_section_prop;
use crate::table_util::Col;
use squid_n_core::units::to_display::{area_cm2, inertia_cm4};

/// ステータスバーのドック/パネル切替アイコンの共通クリック挙動（Zed 風）。
/// 対象ドックが開いていて対象パネルが既にアクティブなら閉じて `false` を返す。
/// それ以外はドックを開いて `true` を返す（呼び出し側は `true` のときのみ
/// 対象パネル/タブをアクティブにする）。
fn toggle_dock_icon(dock_open: &mut bool, is_active: bool) -> bool {
    if *dock_open && is_active {
        *dock_open = false;
        false
    } else {
        *dock_open = true;
        true
    }
}

impl App {
    /// 「開く…」ダイアログを表示して読み込む。
    pub(crate) fn open_project_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("Squid-n プロジェクト", &["scz"])
            .pick_file()
        {
            self.open_project_from(path);
        }
    }

    /// 保存する。`force_ask` またはパス未設定時はダイアログで保存先を尋ねる。
    pub(crate) fn save_project_dialog(&mut self, force_ask: bool) {
        let path = if force_ask {
            None
        } else {
            self.project_path.clone()
        };
        let path = path.or_else(|| {
            rfd::FileDialog::new()
                .add_filter("Squid-n プロジェクト", &["scz"])
                .set_file_name("model.scz")
                .save_file()
        });
        if let Some(path) = path {
            self.save_project_to(path);
        }
    }

    /// 「ST-Bridge 読込…」ダイアログを表示して読み込む。
    pub(crate) fn import_stbridge_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("ST-Bridge", &["stb", "xml"])
            .pick_file()
        {
            self.import_stbridge_from(path);
        }
    }

    /// 「ST-Bridge 書出…」ダイアログを表示して保存先を尋ね、標準 ST-Bridge で書き出す。
    pub(crate) fn export_stbridge_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("ST-Bridge", &["stb", "xml"])
            .set_file_name("model.stb")
            .save_file()
        {
            self.export_stbridge_to(path);
        }
    }

    /// 左ペイン：ナビゲータ（階/部材群/荷重ケース/結果ケースのツリー）。
    pub(crate) fn navigator_panel(&mut self, ui: &mut egui::Ui) {
        ui.group(|ui| {
            ui.strong("ナビゲータ");
            ui.separator();

            // 部材グループ（簡易: 材種ごと）
            let header = egui::CollapsingHeader::new("部材グループ")
                .default_open(true)
                .id_salt("nav_groups");
            header.show(ui, |ui| {
                // 鋼系（S・CFT）とそれ以外へ 1 回の走査で分ける
                // （振り分けの規約は `member_material_groups`）。
                let (steel_ids, rc_ids) = member_material_groups(&self.model);
                // selected 表示は簡易判定（先頭要素が当該グループに属するか）。
                let is_steel_sel = self
                    .selection
                    .members
                    .first()
                    .map(|id| steel_ids.contains(id))
                    .unwrap_or(false);
                if ui
                    .selectable_label(is_steel_sel, format!("鋼材部材 ({})", steel_ids.len()))
                    .on_hover_text("クリックで3Dビューにハイライト")
                    .clicked()
                {
                    self.selection.members = steel_ids.clone();
                }
                let is_rc_sel = self
                    .selection
                    .members
                    .first()
                    .map(|id| rc_ids.contains(id))
                    .unwrap_or(false);
                if ui
                    .selectable_label(is_rc_sel, format!("RC部材 ({})", rc_ids.len()))
                    .on_hover_text("クリックで3Dビューにハイライト")
                    .clicked()
                {
                    self.selection.members = rc_ids.clone();
                }
            });

            self.nav_load_cases(ui);

            // 部材リスト（クリックで focus_member を更新 → テーブル/インスペクタに連動）
            let header = egui::CollapsingHeader::new("部材一覧")
                .default_open(false)
                .id_salt("nav_members");
            header.show(ui, |ui| {
                use crate::table_util::{self, Col};
                let n = self.model.elements.len();
                table_util::standard_table(
                    ui,
                    "nav_members_tbl",
                    &[Col::id(), Col::label("種別")],
                    n,
                    |row| {
                        let idx = row.index();
                        let elem = self.model.elements[idx].clone();
                        let is_focus = self.nav.focus_member == Some(elem.id);
                        row.col(|ui| {
                            if table_util::id_cell(ui, is_focus, elem.id.0, "クリックで部材を選択")
                            {
                                self.nav.focus_member = Some(elem.id);
                            }
                        });
                        row.col(|ui| {
                            ui.label(format!("{:?}", elem.kind));
                        });
                    },
                );
            });

            // 結果ケース：静的解析結果／荷重組合せ結果をクリックで表示対象に選択できる。
            // 選択は変位図だけでなく応力図・断面検定（長期/短期）まで切り替える
            // （`select_displayed_result`）。クロージャ内では self を可変借用できないため、
            // クリックされたキーを一旦退避し、クロージャの外で適用する。
            let mut nav_selected: Option<StaticKey> = None;
            let header = egui::CollapsingHeader::new("結果ケース")
                .default_open(true)
                .id_salt("nav_result_cases");
            header.show(ui, |ui| {
                if let Some(r) = &self.results {
                    if r.statics.is_empty() && r.combos.is_empty() && r.modal.is_none() {
                        ui.label("（未実行）");
                    } else {
                        for (key, _) in r.statics.iter() {
                            let label = match key {
                                StaticCaseKey::User(id) => {
                                    let lc_name = self
                                        .model
                                        .load_cases
                                        .iter()
                                        .find(|lc| lc.id == *id)
                                        .map(|lc| lc.name.as_str())
                                        .unwrap_or("");
                                    format!("静的 LC {} {}", id.0, lc_name)
                                }
                                StaticCaseKey::Seismic(SeismicDir::X) => {
                                    "地震静的 (X方向)".to_string()
                                }
                                StaticCaseKey::Seismic(SeismicDir::Y) => {
                                    "地震静的 (Y方向)".to_string()
                                }
                            };
                            let is_sel = self.nav.focus_result == Some(StaticKey::Case(*key));
                            if ui.selectable_label(is_sel, label).clicked() {
                                nav_selected = Some(StaticKey::Case(*key));
                            }
                        }
                        for (i, (name, _)) in r.combos.iter().enumerate() {
                            let is_sel = self.nav.focus_result == Some(StaticKey::Combo(i));
                            if ui
                                .selectable_label(is_sel, format!("組合せ {}", name))
                                .clicked()
                            {
                                nav_selected = Some(StaticKey::Combo(i));
                            }
                        }
                        if r.modal.is_some() {
                            ui.label("固有値");
                        }
                    }
                } else {
                    ui.label("（未実行）");
                }
            });
            if let Some(key) = nav_selected {
                self.select_displayed_result(key);
            }

            // 階/レベル（準備計算が生成した階を上階→下階順に表示）
            let _ = ui.collapsing("階/レベル", |ui| {
                if self.model.stories.is_empty() {
                    ui.colored_label(crate::theme::GRAY_600, "未定義");
                    if ui.small_button("🏢 解析タブで自動生成").clicked() {
                        self.active_tab = Tab::Analysis;
                    }
                } else {
                    for s in self.model.stories.iter().rev() {
                        ui.label(format!(
                            "{}  Z={:.0}mm  W={:.1}kN",
                            s.name,
                            s.elevation,
                            s.seismic_weight.unwrap_or(0.0) / 1000.0
                        ));
                    }
                }
            });
        });
    }

    /// 左ドック「作成」パネル：梁・壁・スラブ作成モードの切替と断面割当 UI。
    /// いずれもビューア（3D クリック）と連動する状態（`beam_draw_mode` 等）を操作する。
    pub(crate) fn draw_tools_panel(&mut self, ui: &mut egui::Ui) {
        ui.strong("作成");
        ui.separator();

        // 荷重の対象ピック中は 3D のクリックをそちらが受け取るため、作成モードを
        // ON にできると「切り替えたのに反応しない」状態になる。パネルごと無効にする。
        if self.load_pick_active() {
            ui.colored_label(
                crate::theme::BEST_YELLOW,
                "荷重の対象を選択中は作成モードを使えません。\
                 3D ビューで対象を選ぶか、Esc で選択を取り消してください。",
            );
            return;
        }

        // --- 梁作成モード ---
        // ON 中はクリックで節点を選び、2 点目で梁を生成する（OFF 中は部材クリック=断面割当）。
        ui.horizontal(|ui| {
            let beam_was_on = self.beam_draw_mode;
            ui.toggle_value(&mut self.beam_draw_mode, "梁作成モード");
            // 梁作成を ON にしたら壁・スラブ作成は OFF（排他）
            if self.beam_draw_mode && !beam_was_on {
                self.wall_draw_mode = false;
                self.slab_draw_mode = false;
            }
            if self.beam_draw_mode {
                match self.beam_draw_first {
                    None => {
                        ui.label("始点をクリック");
                    }
                    Some(first) => {
                        ui.label(format!("始点 {} 選択中 → 終点をクリック", first.label()));
                        if ui.button("キャンセル").clicked() {
                            self.beam_draw_first = None;
                        }
                    }
                }
            }
        });
        // モード OFF 時は始点選択をクリア
        if !self.beam_draw_mode {
            self.beam_draw_first = None;
        }

        // --- 壁作成モード ---
        // ON 中はクリックで柱・梁に囲まれた 4 節点を順に選び、4 点目で壁を生成する。
        ui.horizontal(|ui| {
            let wall_was_on = self.wall_draw_mode;
            ui.toggle_value(&mut self.wall_draw_mode, "壁作成モード");
            // 壁作成を ON にしたら梁・スラブ作成は OFF（排他）
            if self.wall_draw_mode && !wall_was_on {
                self.beam_draw_mode = false;
                self.slab_draw_mode = false;
            }
            if self.wall_draw_mode {
                let picked: Vec<String> = self
                    .wall_draw_nodes
                    .iter()
                    .map(|n| format!("N{}", n.0))
                    .collect();
                ui.label(format!(
                    "節点を4つクリック ({}/4){}",
                    self.wall_draw_nodes.len(),
                    if picked.is_empty() {
                        String::new()
                    } else {
                        format!(": {}", picked.join(", "))
                    }
                ));
                if !self.wall_draw_nodes.is_empty() && ui.button("キャンセル").clicked() {
                    self.wall_draw_nodes.clear();
                }
            }
        });
        // モード OFF 時は選択をクリア
        if !self.wall_draw_mode {
            self.wall_draw_nodes.clear();
        }

        // --- スラブ作成モード ---
        // ON 中はクリックで境界節点を外周順に選び、3〜N 節点そろったら「確定」で生成する。
        ui.horizontal(|ui| {
            let slab_was_on = self.slab_draw_mode;
            ui.toggle_value(&mut self.slab_draw_mode, "スラブ作成モード");
            // スラブ作成を ON にしたら梁・壁作成は OFF（排他）
            if self.slab_draw_mode && !slab_was_on {
                self.beam_draw_mode = false;
                self.wall_draw_mode = false;
            }
            if self.slab_draw_mode {
                // 節点削除などで陳腐化した参照（範囲外 id）を毎フレーム除去し、
                // 存在しない節点を境界に含むスラブの生成を防ぐ。
                let node_count = self.model.nodes.len() as u32;
                self.slab_draw_nodes.retain(|n| n.0 < node_count);
                let picked: Vec<String> = self
                    .slab_draw_nodes
                    .iter()
                    .map(|n| format!("N{}", n.0))
                    .collect();
                ui.label(format!(
                    "境界節点を外周順にクリック ({}){}",
                    self.slab_draw_nodes.len(),
                    if picked.is_empty() {
                        String::new()
                    } else {
                        format!(": {}", picked.join(", "))
                    }
                ));
                if self.slab_draw_nodes.len() >= 3 && ui.button("確定").clicked() {
                    let boundary = self.slab_draw_nodes.clone();
                    // 床タブの追加フォームと同じ下書きの断面を使う。消えた断面を
                    // 指したままだと `AddSlab` が参照検証で Noop になり無反応に
                    // 見えるため、解決できない下書きは未割当として渡す。
                    let draft_section = self.slab_draft.section.filter(|sid| {
                        self.model
                            .sections
                            .get(sid.index())
                            .is_some_and(|s| s.thickness.is_some_and(|t| t > 0.0))
                    });
                    self.undo.run(
                        &mut self.model,
                        Box::new(squid_n_edit::AddSlab {
                            boundary,
                            joists: Vec::new(),
                            loads: Vec::new(),
                            method: squid_n_core::model::DistributionMethod::TriTrapezoid,
                            usage: self.slab_draft.usage,
                            section: draft_section,
                        }),
                    );
                    self.staleness.mark_edited();
                    self.slab_draw_nodes.clear();
                }
                if !self.slab_draw_nodes.is_empty() && ui.button("キャンセル").clicked() {
                    self.slab_draw_nodes.clear();
                }
            }
        });
        // モード OFF 時は選択をクリア
        if !self.slab_draw_mode {
            self.slab_draw_nodes.clear();
        }

        // --- 断面割当 UI ---
        // focus_member を先にコピーして、後段の可変借用と競合しないようにする
        let focus_id: Option<squid_n_core::ids::ElemId> = self.nav.focus_member;
        // 存在確認もここで行い、ローカルに有効性と現在断面を取得
        let elem_info: Option<(squid_n_core::ids::ElemId, Option<SectionId>)> =
            focus_id.and_then(|eid| {
                self.model
                    .elements
                    .iter()
                    .find(|e| e.id == eid)
                    .map(|e| (e.id, e.section))
            });

        let mut pending_assign: Option<Option<SectionId>> = None;

        if let Some((elem_id, current_section)) = elem_info {
            ui.horizontal(|ui| {
                ui.label(format!("選択中の梁 #{}", elem_id.0));
                ui.label("断面:");
                let selected_text = current_section
                    .map(|sid| format!("S{}", sid.0))
                    .unwrap_or_else(|| "―".to_string());
                egui::ComboBox::from_id_salt("viewer_assign_section")
                    .selected_text(selected_text)
                    .show_ui(ui, |ui| {
                        if ui
                            .selectable_label(current_section.is_none(), "―")
                            .clicked()
                        {
                            pending_assign = Some(None);
                        }
                        for sec in &self.model.sections {
                            if ui
                                .selectable_label(
                                    current_section == Some(sec.id),
                                    format!("S{}", sec.id.0),
                                )
                                .clicked()
                            {
                                pending_assign = Some(Some(sec.id));
                            }
                        }
                    });
            });
            // クロージャ外で発行（借用ルール）
            if let Some(section) = pending_assign {
                self.undo.run(
                    &mut self.model,
                    Box::new(squid_n_edit::SetElementSection {
                        elem: elem_id,
                        section,
                    }),
                );
                self.staleness.mark_edited();
            }
        } else {
            ui.label("ビューアで梁をクリックすると選択できます");
        }
    }

    /// モデルタブ：サブタブ切替で節点/部材/断面/材料を編集するテーブルを表示。
    pub(crate) fn model_tab_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            let subs = [
                ("節点", ModelTab::Nodes),
                ("境界条件", ModelTab::BoundaryConditions),
                ("部材", ModelTab::Members),
                ("断面", ModelTab::Sections),
                ("材料", ModelTab::Materials),
                ("スラブ", ModelTab::Slabs),
                ("壁属性", ModelTab::WallAttrs),
                ("雑壁", ModelTab::MiscWalls),
                ("部材付帯情報", ModelTab::MemberDetails),
                ("S造検定属性", ModelTab::SteelAttrs),
                ("通り芯", ModelTab::Axes),
            ];
            for (label, sub) in &subs {
                let sel = self.model_tab == *sub;
                if ui.selectable_label(sel, *label).clicked() {
                    self.model_tab = *sub;
                }
            }
        });
        ui.separator();
        match self.model_tab {
            ModelTab::Nodes => crate::tables::nodes::nodes_table(ui, self),
            ModelTab::BoundaryConditions => {
                crate::tables::nodes::boundary_condition_panel(ui, self)
            }
            ModelTab::Members => crate::tables::members::members_table(ui, self),
            ModelTab::Sections => {
                crate::tables::sections::sections_table(ui, self);
                ui.add_space(8.0);
                crate::section_editor::catalog_section_panel(ui, self);
                ui.add_space(8.0);
                crate::section_editor::section_editor_panel(ui, self);
                ui.add_space(8.0);
                crate::damper_def_editor::damper_def_panel(ui, self);
            }
            ModelTab::Materials => crate::tables::materials::materials_table(ui, self),
            ModelTab::Slabs => crate::tables::slabs::slabs_table(ui, self),
            ModelTab::WallAttrs => crate::tables::wall_attrs::wall_attrs_table(ui, self),
            ModelTab::MiscWalls => crate::tables::misc_walls::misc_walls_table(ui, self),
            ModelTab::MemberDetails => {
                crate::tables::member_details::member_details_table(ui, self)
            }
            ModelTab::SteelAttrs => crate::tables::steel_attrs::steel_attrs_table(ui, self),
            ModelTab::Axes => crate::tables::axes::axes_table(ui, self),
        }
    }

    /// 右ドック「① 準備計算」パネル：解析条件の入力・階の定義と、準備計算の実行。
    ///
    /// 一貫計算の手順（① 解析入力を確定 → ② 解く）のうち ① を担う。実行すると
    /// 階の定義・剛域・荷重ケース（DL/LL/EX/EY/WX/WY）が確定する。② は
    /// [`App::analysis_panel`]。
    pub(crate) fn preparation_panel(&mut self, ui: &mut egui::Ui) {
        self.right_panel_switcher(ui);
        ui.heading("① 準備計算");
        ui.separator();

        // バックグラウンドジョブ実行中は実行ボタンを無効化する（P8 §5）。
        let running = self.job.is_some();
        self.preparation_section(ui, running);
    }

    /// 右ドック「② 解析」パネル：確定した荷重ケース・荷重組合せを解く。
    ///
    /// 地震力も EX/EY の荷重ケースとして扱うため、専用の実行導線は
    /// 設けない。① は [`App::preparation_panel`]。
    pub(crate) fn analysis_panel(&mut self, ui: &mut egui::Ui) {
        self.right_panel_switcher(ui);
        ui.heading("② 解析");
        ui.separator();

        // バックグラウンドジョブ実行中は全解析ボタンを無効化する（P8 §5）。
        let running = self.job.is_some();

        if let Some(when) = self.staleness.last_run {
            if let Ok(dur) = when.elapsed() {
                ui.label(format!("最終実行: {:.0} 秒前", dur.as_secs_f64()));
            } else {
                ui.label("最終実行: 不明");
            }
        } else {
            ui.label("最終実行: なし");
        }
        if self.staleness.results_stale {
            ui.colored_label(
                crate::theme::BEST_YELLOW,
                "⚠ モデルが編集されました。結果は再計算が必要です。",
            );
        }
        if self.staleness.preparation_stale {
            ui.colored_label(
                crate::theme::BEST_YELLOW,
                "⚠ 準備計算が未実行、またはモデル編集により古くなっています\
                 （解析の実行時に自動で最新化されます）。",
            );
        }
        ui.separator();

        self.analysis_run_sections(ui, running);
    }

    /// 「① 準備計算」「② 解析」を行き来する切替行（両パネルの先頭に置く）。
    ///
    /// ステータスバーのアイコンからも切り替えられるが、① と ② は一貫計算の手順として
    /// 連続しているため、パネル内からも直接移動できるようにする。
    fn right_panel_switcher(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            if ui
                .selectable_label(self.right_panel == RightPanel::Preparation, "① 準備計算")
                .clicked()
            {
                self.right_panel = RightPanel::Preparation;
            }
            if ui
                .selectable_label(self.right_panel == RightPanel::Analysis, "② 解析")
                .clicked()
            {
                self.right_panel = RightPanel::Analysis;
            }
        });
        ui.add_space(2.0);
    }

    /// 準備計算（① 解析前の前処理）のセクション一式。
    ///
    /// 実行ボタン・結果ステータスに続けて、準備計算が使う入力
    /// （地震力の算定諸元・計算条件）と、その成果である
    /// 階の定義を並べる。地震力の諸元をここへ置くのは、これが
    /// EX/EY の荷重ケースを決める準備計算の入力だからである。
    fn preparation_section(&mut self, ui: &mut egui::Ui, running: bool) {
        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(!running, egui::Button::new("🛠 準備計算 実行"))
                .on_hover_text(
                    "解析前の前処理（階の定義・剛域の算定・床荷重/自重/積載の集計・\
                     地震力(Ai分布)の算定・荷重ケース DL/LL/EX/EY の生成・\
                     モデル整合性チェック）を実行し、結果を下ドック「準備計算」タブに表示します",
                )
                .clicked()
            {
                self.run_preparation();
                self.bottom_dock_open = true;
                self.bottom_tab = BottomTab::Preparation;
            }
            if ui
                .button("📋 結果を表示")
                .on_hover_text("下ドックの「準備計算」タブを開きます")
                .clicked()
            {
                self.bottom_dock_open = true;
                self.bottom_tab = BottomTab::Preparation;
            }
        });
        match self.preparation.as_ref() {
            _ if self.staleness.preparation_stale => ui.colored_label(
                crate::theme::BEST_YELLOW,
                "⚠ 準備計算が未実行、またはモデル編集により古くなっています。",
            ),
            Some(p) if !p.is_ready() => ui.colored_label(
                crate::theme::ERROR_RED,
                format!(
                    "⛔ 準備計算: 整合性チェックにエラー {} 件（解析前に解消してください）",
                    p.diag_errors
                ),
            ),
            Some(p) => ui.colored_label(
                crate::theme::GOOD_GREEN,
                format!(
                    "✅ 準備計算 済（階 {} ・剛域 {} 部材・警告 {} 件）",
                    p.stories.len(),
                    p.rigid_zones.len(),
                    p.diag_warnings
                ),
            ),
            None => ui.colored_label(crate::theme::GRAY_600, "準備計算: 未実行"),
        };
        ui.add_space(6.0);

        self.seismic_condition_section(ui);
        ui.add_space(6.0);
        self.member_modeling_section(ui);
        ui.add_space(6.0);
        self.calc_condition_section(ui);
        ui.add_space(6.0);
        self.stories_section(ui);
    }

    /// 部材のモデル化（建物一律）。解析条件ではなく「部材をどう解くか」の設定で、
    /// 変更した時点でモデルへ反映される（準備計算の実行を待たない）。剛性が変わる
    /// ため、変更すると結果は陳腐化する（`staleness.mark_edited`）。
    fn member_modeling_section(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("部材のモデル化")
            .default_open(false)
            .id_salt("as_member_modeling")
            .show(ui, |ui| {
                use squid_n_core::model::BeamTorsionMode;
                let mut release = self.model.beam_torsion == BeamTorsionMode::ReleaseIEnd;
                let resp = ui
                    .checkbox(&mut release, "部材 i 端のねじりをピン（梁・柱）")
                    .on_hover_text(
                        "日本の一貫計算の通例に合わせ、線材（梁要素。柱を含む）の i 端の\
                     ねじれ回転をピンとしてモデル化します。ねじりは材長方向に一定のため、\
                     解放した部材は全長で Mx=0 になります。\
                     ただし、ねじりを解放すると材軸まわりの回転が拘束されなくなる節点を\
                     持つ部材（一直線に並ぶ部材だけが集まる中間節点・片持ち先端など）は、\
                     剛性行列が特異になるため自動的に対象外とし、ねじり剛性を保持します。\
                     対象外になった部材は準備計算の結果タブで確認できます。",
                    );
                if resp.changed() {
                    let mode = if release {
                        BeamTorsionMode::ReleaseIEnd
                    } else {
                        BeamTorsionMode::Keep
                    };
                    self.undo.run(
                        &mut self.model,
                        Box::new(squid_n_edit::SetBeamTorsion { mode }),
                    );
                    self.staleness.mark_edited();
                }
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "OFF にすると全部材でねじり剛性 GJ/L を保持します。\
                     床小梁の格子解析は、交差する小梁が両端を大梁にねじれ止めされた\
                     一本材であるため、この設定によらず常にねじり剛性を保持します。",
                );

                ui.add_space(6.0);
                use squid_n_core::model::PanelZoneMode;
                let mut panel = self.model.panel_zone.is_enabled();
                let resp = ui
                    .checkbox(&mut panel, "仕口パネルをモデル化（柱梁接合部）")
                    .on_hover_text(
                        "S 造（CFT を除く）の柱梁接合部に仕口パネルを設け、接合部の\
                     せん断変形を解析へ反映します。パネルを設けた節点はせん断変形角\
                     γX・γY の 2 自由度を追加で持ち、取り付く部材はパネル寸法分だけ\
                     離れた位置（柱フェース・梁フェース）で接合します。\
                     RC・SRC・CFT の接合部は対象外で、従来どおり剛域で有限寸法を評価します。\
                     生成されたパネルは準備計算の結果タブで確認できます。",
                    );
                if resp.changed() {
                    let mode = if panel {
                        PanelZoneMode::Model
                    } else {
                        PanelZoneMode::None
                    };
                    self.undo.run(
                        &mut self.model,
                        Box::new(squid_n_edit::SetPanelZoneMode { mode }),
                    );
                    self.staleness.mark_edited();
                }
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "OFF にすると接合部を剛節点として扱います（パネルのせん断変形を\
                     考慮しません）。柱梁接合部の断面算定は、この設定によらず常に行います。",
                );

                ui.add_space(6.0);
                let mut consider = self.model.stress_cfg.rigid_zone_consider_walls;
                let resp = ui
                    .checkbox(&mut consider, "剛域の算定で壁を考慮する")
                    .on_hover_text(
                        "剛域長 λ = 節点から部材フェースまでの距離 − 部材せい/4 の\
                     「部材フェース」「部材せい」に、取り付く壁を含めた寸法を用います\
                     （柱には袖壁、梁には腰壁・垂壁）。両側に取り付く壁の長さが異なる\
                     場合は長い方を基準にします。対象は現場打ちコンクリート壁で厚さ\
                     100mm 以上のもので、耐震壁・雑壁を問いません。",
                    );
                if resp.changed() {
                    self.model.stress_cfg.rigid_zone_consider_walls = consider;
                    self.staleness.mark_edited();
                }
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "OFF にすると部材の原断面だけで剛域を算定します。\
                     剛域を設けるのは、その節点に集まる柱・大梁がすべて RC/SRC の\
                     ときだけです（S 造の仕口は仕口パネルでモデル化します）。",
                );
            });
    }

    /// 地震力（Ai 分布）の算定諸元。準備計算が EX/EY 荷重ケースを組み立てる入力。
    fn seismic_condition_section(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("地震力の条件 (Ai 分布)")
            .default_open(true)
            .id_salt("as_seismic_cfg")
            .show(ui, |ui| {
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "準備計算で水平力を算定し、荷重ケース EX・EY へ反映します。",
                );
                ui.horizontal_wrapped(|ui| {
                    ui.label("T算定:");
                    ui.selectable_value(
                        &mut self.analysis_cfg.ai_mode,
                        AiMode::SemiPrecise,
                        "固有値",
                    )
                    .on_hover_text("固有値解析による 1 次周期（先に固有値解析の実行が必要）");
                    ui.selectable_value(&mut self.analysis_cfg.ai_mode, AiMode::Approx, "略算")
                        .on_hover_text("T = h(0.02 + 0.01α) の略算式");
                });
                ui.horizontal_wrapped(|ui| {
                    ui.label("Z:");
                    ui.add(
                        egui::DragValue::new(&mut self.analysis_cfg.z)
                            .speed(0.05)
                            .range(0.7..=1.0),
                    )
                    .on_hover_text(
                        "地震地域係数 Z（昭55建告1793号 別表第2）。建設地の値を入力します",
                    );
                    ui.label("地盤:");
                    use squid_n_load::ai::SoilClass;
                    for (label, soil) in [
                        ("第一種", SoilClass::I),
                        ("第二種", SoilClass::II),
                        ("第三種", SoilClass::III),
                    ] {
                        ui.selectable_value(&mut self.analysis_cfg.soil, soil, label);
                    }
                    ui.label("C0:");
                    ui.add(
                        egui::DragValue::new(&mut self.analysis_cfg.c0)
                            .speed(0.05)
                            .range(0.05..=1.0),
                    );
                });
            });
    }

    /// 計算条件（質量方式・並列スレッド数）。いずれも準備計算の実行時に
    /// モデルへ反映される、または解析全体に共通で効く設定。
    fn calc_condition_section(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("計算条件")
            .default_open(false)
            .id_salt("as_calc_cfg")
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    use squid_n_core::model::MassMethod;
                    ui.label("質量方式:");
                    egui::ComboBox::from_id_salt("mass_method")
                        .selected_text(match self.analysis_cfg.mass_method {
                            MassMethod::CorrectedLumped => "補正質点（既定）",
                            MassMethod::LumpedOnly => "質点のみ",
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut self.analysis_cfg.mass_method,
                                MassMethod::CorrectedLumped,
                                "補正質点（既定）",
                            );
                            ui.selectable_value(
                                &mut self.analysis_cfg.mass_method,
                                MassMethod::LumpedOnly,
                                "質点のみ",
                            );
                        })
                        .response
                        .on_hover_text(
                            "準備計算の実行時にモデルへ反映される。\
                             固有値・時刻歴・精算周期の質量に共通で効く。",
                        );
                });
                ui.horizontal_wrapped(|ui| {
                    ui.label("並列スレッド数:");
                    ui.add(egui::DragValue::new(&mut self.analysis_cfg.threads).range(0..=256));
                });
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "0=自動(全コア) / 1=単一スレッド(結果の完全再現) / n=固定",
                );
            });
    }

    /// 階の追加フォーム。階名と階レベルだけを与えて階を作る（所属節点・重量は
    /// 準備計算が埋める）。既定の階レベルは最上階の 1 つ上を階高 3500mm で見込む。
    fn story_add_row(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("階名:");
            ui.add(
                egui::TextEdit::singleline(&mut self.new_story_draft.0)
                    .desired_width(60.0)
                    .hint_text("3F"),
            );
            ui.label("レベル[mm]:");
            ui.add(
                egui::DragValue::new(&mut self.new_story_draft.1)
                    .speed(50.0)
                    .range(-1.0e6..=1.0e6),
            );
            let name = self.new_story_draft.0.trim().to_string();
            let can_add = !name.is_empty();
            if ui
                .add_enabled(can_add, egui::Button::new("➕ 階を追加"))
                .on_hover_text(
                    "階名とレベルだけを定義します。所属節点・剛床・地震用重量は\
                     次の準備計算で算定されます。",
                )
                .on_disabled_hover_text("階名を入力してください")
                .clicked()
            {
                let elevation = self.new_story_draft.1;
                self.undo.run(
                    &mut self.model,
                    Box::new(squid_n_edit::AddStory { name, elevation }),
                );
                self.staleness.mark_edited();
                self.new_story_draft.0.clear();
                self.new_story_draft.1 = elevation + 3500.0;
            }
        });
        ui.separator();
    }

    /// 階の定義。**階名・階レベル・階種別・地震用重量の手入力は利用者が決める**
    /// データで、ここで編集する。節点数・主要構造種別は準備計算が埋める
    /// 派生値のため表示のみとする。
    ///
    /// 表の行は上階→下階の順に並べる（伏図・階の分布タブと同じ向き）。
    /// 各セルで確定した編集は適用待ちキューへ積まれ、描画ループを抜けてから
    /// 1 フレーム 1 コマンドずつ適用される（階の追加・削除・レベル変更は
    /// `StoryId` の繰り上げ・並べ替えを伴うため、モデルを書き換えたあと
    /// 残りの行が古い ID を指さないようにする）。
    fn stories_section(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("階の定義")
            .default_open(false)
            .id_salt("as_stories_table")
            .show(ui, |ui| {
                self.story_add_row(ui);
                if self.model.stories.is_empty() {
                    ui.colored_label(
                        crate::theme::GRAY_600,
                        "未定義です。上の「階を追加」で定義するか、準備計算を実行すると\
                         節点の標高から生成されます。",
                    );
                    return;
                }
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "階名・レベル・重量・種別は利用者が決めます。節点数・構造種別は\
                     準備計算が算定します（構造種別は柱・梁の断面から判定）。",
                );

                use squid_n_core::model::StoryLevelKind;

                // model.stories を借用したまま undo.run（model の削除）ができないため、
                // 行データを先に複製してから描画・編集確定を行う。並びは伏図・
                // 階の分布タブと同じ上階→下階の順にする（model.stories は下から上）。
                #[allow(clippy::type_complexity)]
                let story_rows: Vec<(
                    squid_n_core::ids::StoryId,
                    String,
                    f64,
                    usize,
                    Option<f64>,
                    Option<f64>,
                    squid_n_core::model::StoryStructure,
                    StoryLevelKind,
                )> = self
                    .model
                    .stories
                    .iter()
                    .rev()
                    .map(|s| {
                        (
                            s.id,
                            s.name.clone(),
                            s.elevation,
                            s.node_ids.len(),
                            s.seismic_weight,
                            s.weight_override,
                            s.structure,
                            s.level_kind,
                        )
                    })
                    .collect();

                // 確定待ちの編集コマンド。表ループの途中で model を書き換えると
                // 残りの行が古い ID を指すため、ここでは集めるだけに留める。
                // 適用はループを抜けた後にキューへ積み、この関数の先頭で
                // 1 フレーム 1 コマンドずつ行う。同一フレームで複数セルが確定
                // しても破棄されない（確定 → 適用に 1 フレームの遅延が付く）。
                let mut pending_delete: Option<squid_n_core::ids::StoryId> = None;
                // 階への複製ダイアログを開く階（ダイアログは `self` を要するため、
                // 表ループを抜けてから開く）。
                let mut pending_copy: Option<squid_n_core::ids::StoryId> = None;
                let mut pending_level_kind: Option<(squid_n_core::ids::StoryId, StoryLevelKind)> =
                    None;
                let mut pending_weight: Option<(squid_n_core::ids::StoryId, Option<f64>)> = None;
                let mut pending_name_elev: Option<(
                    squid_n_core::ids::StoryId,
                    String,
                    f64,
                )> = None;

                crate::table_util::standard_table(
                    ui,
                    "prep_story_def",
                    &[
                        Col::label("階名"),
                        Col::num("レベル [mm]"),
                        Col::num("節点数"),
                        Col::label("構造"),
                        Col::wide_num("W [kN]").hover(
                            "地震用重量。編集すると確定値として固定され、\
                             準備計算で再生成しても上書きされません（undo 可）",
                        ),
                        Col::wide_num("種別"),
                        // 階への複製（⧉）と削除（🗑）の 2 つ。
                        Col::actions_n(2),
                    ],
                    story_rows.len(),
                    |row| {
                        let (
                            story,
                            name,
                            elevation,
                            n_nodes,
                            weight,
                            weight_override,
                            structure,
                            level_kind,
                        ) = &story_rows[row.index()];
                        let story = *story;
                        // 基部の階（床レベル列の先頭）。標高の変更・削除を禁じ、
                        // 層の属性（種別・重量）は上端の階が持つため編集させない。
                        let is_base = story.index() == 0;

                        // 階名（編集可）。空文字は無視する（確定はフォーカス喪失時）。
                        row.col(|ui| {
                            let cell_id = egui::Id::new(("story_name", story.0));
                            let mut buf = ui
                                .data(|d| d.get_temp::<String>(cell_id))
                                .unwrap_or_else(|| name.clone());
                            let resp = ui.add(
                                egui::TextEdit::singleline(&mut buf)
                                    .desired_width(ui.available_width()),
                            );
                            if resp.has_focus() {
                                ui.data_mut(|d| d.insert_temp(cell_id, buf));
                            } else if resp.lost_focus() {
                                let trimmed = buf.trim().to_string();
                                if !trimmed.is_empty() && trimmed != *name {
                                    pending_name_elev = Some((story, trimmed, *elevation));
                                }
                                ui.data_mut(|d| d.remove::<String>(cell_id));
                            }
                        });
                        // レベル（編集可。ドラッグ中は行ごとの一時値を持ち、終了時に確定）。
                        // 基部の階だけは表示のみ。基部の標高は構造の最下端そのもので
                        // あり、階の列の先頭が基部であることは層の算定が依拠する
                        // 不変条件のため（`squid_n_core::model::story`）。
                        row.col(|ui| {
                            if is_base {
                                ui.label(format!("{elevation:.0}")).on_hover_text(
                                    "基部（柱脚・基礎梁のレベル）。構造の最下端そのものなので変更できません",
                                );
                                return;
                            }
                            let cell_id = egui::Id::new(("story_elevation", story.0));
                            let mut v = ui
                                .data(|d| d.get_temp::<f64>(cell_id))
                                .unwrap_or(*elevation);
                            let resp = crate::table_util::cell_drag_value(
                                ui,
                                true,
                                egui::DragValue::new(&mut v)
                                    .speed(50.0)
                                    .range(-1.0e6..=1.0e6),
                            );
                            if resp.has_focus() || resp.dragged() {
                                ui.data_mut(|d| d.insert_temp(cell_id, v));
                            } else if resp.drag_stopped() || resp.lost_focus() {
                                if (v - *elevation).abs() > 1e-6 {
                                    pending_name_elev = Some((story, name.clone(), v));
                                }
                                ui.data_mut(|d| d.remove::<f64>(cell_id));
                            }
                        });
                        // 節点数（準備計算で決まる導出値。表示のみ）。
                        row.col(|ui| {
                            ui.label(format!("{n_nodes}"));
                        });
                        // 構造（準備計算で決まる導出値。表示のみ）。
                        row.col(|ui| {
                            ui.label(crate::app::preparation::story_structure_label(*structure));
                        });
                        // 地震用重量 W。手入力すると確定値として固定され、自動は予想値で示す。
                        row.col(|ui| {
                            let cell_id = egui::Id::new(("story_weight", story.0));
                            let mut w = ui
                                .data(|d| d.get_temp::<f64>(cell_id))
                                .unwrap_or(weight.unwrap_or(0.0) / 1000.0);
                            ui.horizontal_wrapped(|ui| {
                                let resp = ui
                                    .add(
                                        egui::DragValue::new(&mut w)
                                            .speed(1.0)
                                            .range(0.0..=1.0e9),
                                    )
                                    .on_hover_text(
                                        "重量。手動で確定値を固定できます。解除ボタンで自動へ戻します",
                                    );
                                if resp.has_focus() || resp.dragged() {
                                    ui.data_mut(|d| d.insert_temp(cell_id, w));
                                } else if resp.drag_stopped() || resp.lost_focus() {
                                    let new_weight = w * 1000.0;
                                    if (new_weight - weight.unwrap_or(0.0)).abs() > 1e-6 {
                                        pending_weight = Some((story, Some(new_weight)));
                                    }
                                    ui.data_mut(|d| d.remove::<f64>(cell_id));
                                }
                                if weight_override.is_some() {
                                    if ui
                                        .small_button("解除")
                                        .on_hover_text(
                                            "確定値を解除し、準備計算が自動で進めるように戻します",
                                        )
                                        .clicked()
                                    {
                                        pending_weight = Some((story, None));
                                    }
                                } else {
                                    ui.colored_label(crate::theme::GRAY_600, "自動");
                                }
                            });
                        });
                        // 種別（変更可）。PH の k と地下の深さは数値編集で確定する。
                        // 種別は**層**の属性で、層の上端の階が持つ。基部の階は
                        // どの層の上端でもないため編集させない（`Layer` 参照）。
                        row.col(|ui| {
                            if is_base {
                                ui.colored_label(crate::theme::GRAY_600, "—").on_hover_text(
                                    "階種別は層の属性で、層の上端の階が持ちます。\
                                     基部の階はどの層の上端でもないため設定しません",
                                );
                                return;
                            }
                            let mut new_level_kind: Option<StoryLevelKind> = None;
                            let label = match level_kind {
                                StoryLevelKind::Normal => "一般".to_string(),
                                StoryLevelKind::Penthouse { k } => format!("PH(k={k:.2})"),
                                StoryLevelKind::Basement { depth_m } => {
                                    format!("地下(H={depth_m:.1}m)")
                                }
                            };
                            egui::ComboBox::from_id_salt(("story_level_kind", story.0))
                                .selected_text(label)
                                .width(ui.available_width())
                                .show_ui(ui, |ui| {
                                    if ui
                                        .selectable_label(
                                            matches!(level_kind, StoryLevelKind::Normal),
                                            "一般",
                                        )
                                        .clicked()
                                    {
                                        new_level_kind = Some(StoryLevelKind::Normal);
                                    }
                                    if ui
                                        .selectable_label(
                                            matches!(level_kind, StoryLevelKind::Penthouse { .. }),
                                            "PH（塔屋）",
                                        )
                                        .clicked()
                                    {
                                        let k = if let StoryLevelKind::Penthouse { k } = level_kind
                                        {
                                            *k
                                        } else {
                                            0.5
                                        };
                                        new_level_kind = Some(StoryLevelKind::Penthouse { k });
                                    }
                                    if ui
                                        .selectable_label(
                                            matches!(level_kind, StoryLevelKind::Basement { .. }),
                                            "地下",
                                        )
                                        .clicked()
                                    {
                                        let depth_m =
                                            if let StoryLevelKind::Basement { depth_m } = level_kind
                                            {
                                                *depth_m
                                            } else {
                                                3.0
                                            };
                                        new_level_kind =
                                            Some(StoryLevelKind::Basement { depth_m });
                                    }
                                });
                            if let StoryLevelKind::Penthouse { k } = level_kind {
                                let cell_id = egui::Id::new(("story_ph_k", story.0));
                                let mut kv = ui
                                    .data(|d| d.get_temp::<f64>(cell_id))
                                    .unwrap_or(*k);
                                let resp = ui.add(
                                    egui::DragValue::new(&mut kv)
                                        .speed(0.05)
                                        .range(0.0..=2.0)
                                        .prefix("k="),
                                );
                                if resp.has_focus() || resp.dragged() {
                                    ui.data_mut(|d| d.insert_temp(cell_id, kv));
                                } else if resp.drag_stopped() || resp.lost_focus() {
                                    if (kv - *k).abs() > 1e-9 {
                                        new_level_kind =
                                            Some(StoryLevelKind::Penthouse { k: kv });
                                    }
                                    ui.data_mut(|d| d.remove::<f64>(cell_id));
                                }
                            }
                            if let StoryLevelKind::Basement { depth_m } = level_kind {
                                let cell_id = egui::Id::new(("story_bs_d", story.0));
                                let mut dv = ui
                                    .data(|d| d.get_temp::<f64>(cell_id))
                                    .unwrap_or(*depth_m);
                                let resp = ui.add(
                                    egui::DragValue::new(&mut dv)
                                        .speed(0.1)
                                        .range(0.0..=100.0)
                                        .suffix("m"),
                                );
                                if resp.has_focus() || resp.dragged() {
                                    ui.data_mut(|d| d.insert_temp(cell_id, dv));
                                } else if resp.drag_stopped() || resp.lost_focus() {
                                    if (dv - *depth_m).abs() > 1e-9 {
                                        new_level_kind =
                                            Some(StoryLevelKind::Basement { depth_m: dv });
                                    }
                                    ui.data_mut(|d| d.remove::<f64>(cell_id));
                                }
                            }
                            if let Some(level_kind_new) = new_level_kind {
                                pending_level_kind = Some((story, level_kind_new));
                            }
                        });
                        // 操作（階への複製・行削除）。
                        row.col(|ui| {
                            if ui
                                .small_button("⧉")
                                .on_hover_text(
                                    "この階の断面・荷重・床・二次部材を、ほかの階へ\
                                     複製します（同じ平面位置の相手へ配ります）",
                                )
                                .clicked()
                            {
                                pending_copy = Some(story);
                            }
                            // 基部の階は削除できない（階の列の先頭が基部であることが
                            // 層の算定の不変条件。消すと最下層が落ちる）。
                            if is_base {
                                ui.add_enabled(false, egui::Button::new("🗑").small())
                                    .on_disabled_hover_text(
                                        "基部の階は削除できません（最下層の下端がなくなるため）",
                                    );
                            } else if crate::table_util::delete_cell(
                                ui,
                                "この階を削除します。所属節点は所属階を失い、次の階生成で\
                                 直下階の区間へ吸収されます（undo 可）",
                                None,
                            ) {
                                pending_delete = Some(story);
                            }
                        });
                    },
                );
                // 描画ループの後で、確定した編集コマンドを適用待ちキューへ積む。
                // 積む順序は「StoryId を変えない操作を先に、削除を最後」とする。
                // （`SetStoryLevelKind`・`SetStoryWeight` は ID を変えず、
                // `SetStoryLevel` は標高の変更時のみ並べ替える。`DeleteStory` が
                // 後を観るコマンドの ID を古くしないよう、削除は最後に積む。）
                if let Some((story, level_kind)) = pending_level_kind {
                    self.pending_story_cmds.push_back(Box::new(
                        squid_n_edit::SetStoryLevelKind { story, level_kind },
                    ));
                }
                if let Some((story, weight)) = pending_weight {
                    self.pending_story_cmds
                        .push_back(Box::new(squid_n_edit::SetStoryWeight { story, weight }));
                }
                if let Some((story, name, elevation)) = pending_name_elev {
                    self.pending_story_cmds.push_back(Box::new(squid_n_edit::SetStoryLevel {
                        story,
                        name,
                        elevation,
                    }));
                }
                if let Some(story) = pending_delete {
                    self.pending_story_cmds
                        .push_back(Box::new(squid_n_edit::DeleteStory { story }));
                }
                if let Some(story) = pending_copy {
                    crate::story_copy_view::open(self, story);
                }
            });
    }

    /// 解析（② 実行）のセクション一式。静的解析（荷重ケース・荷重組合せ）・
    /// 固有値・増分解析・時刻歴応答。
    fn analysis_run_sections(&mut self, ui: &mut egui::Ui, running: bool) {
        self.static_analysis_section(ui, running);
        ui.add_space(6.0);
        self.eigen_section(ui, running);
        ui.add_space(6.0);
        self.pushover_section(ui, running);
        ui.add_space(6.0);
        self.time_history_section(ui, running);
    }

    /// 静的解析（荷重ケース単体・荷重組合せ）の実行。
    ///
    /// 実行導線は「単体実行」と「一括解析」の 2 つに統一する。求解の最小単位は
    /// 荷重ケース単体で、荷重組合せはその結果の線形和として組み立てるため
    /// （重ね合わせの原理。`Analysis::linear_combination`）、荷重ケースと荷重組合せを
    /// 別の実行導線に分ける理由がない。
    ///
    /// - **単体実行**: 選択した 1 件（荷重ケースまたは荷重組合せ）を解く。
    /// - **一括解析**: 全荷重ケースを解き、全荷重組合せをその線形和で求める。
    ///
    /// 地震力（EX/EY）・風圧力（WX/WY）も準備計算が生成する荷重ケースなので、
    /// ここから同じ導線で実行する（[`App::start_load_case_job`] が方向別の結果キーへ
    /// 振り分ける）。
    fn static_analysis_section(&mut self, ui: &mut egui::Ui, running: bool) {
        egui::CollapsingHeader::new("静的解析")
            .default_open(true)
            .id_salt("as_static")
            .show(ui, |ui| {
                let target = self.resolved_analysis_target();
                ui.horizontal_wrapped(|ui| {
                    ui.label("対象:");
                    let text = target
                        .and_then(|t| self.static_target_label(t))
                        .unwrap_or_else(|| "（なし）".to_string());
                    egui::ComboBox::from_id_salt("analysis_target")
                        .selected_text(text)
                        .show_ui(ui, |ui| {
                            // 荷重ケースと荷重組合せを 1 つの一覧に並べる（見出しで区切る）。
                            ui.label(
                                egui::RichText::new("荷重ケース").color(crate::theme::GRAY_600),
                            );
                            let cases: Vec<(LoadCaseId, String)> = self
                                .model
                                .load_cases
                                .iter()
                                .map(|c| (c.id, format!("[{}] {}", c.id.0, c.name)))
                                .collect();
                            for (id, label) in cases {
                                let t = StaticTarget::Case(id);
                                if ui.selectable_label(target == Some(t), label).clicked() {
                                    self.analysis_target = Some(t);
                                    self.nav.focus_load_case = Some(id);
                                }
                            }
                            if !self.model.combinations.is_empty() {
                                ui.separator();
                                ui.label(
                                    egui::RichText::new("荷重組合せ").color(crate::theme::GRAY_600),
                                );
                            }
                            let combos: Vec<String> = self
                                .model
                                .combinations
                                .iter()
                                .map(|c| c.name.clone())
                                .collect();
                            for (i, name) in combos.into_iter().enumerate() {
                                let t = StaticTarget::Combo(i);
                                if ui.selectable_label(target == Some(t), name).clicked() {
                                    self.analysis_target = Some(t);
                                }
                            }
                        });
                    if ui
                        .add_enabled(
                            target.is_some() && !running,
                            egui::Button::new("▶ 単体実行"),
                        )
                        .on_hover_text(
                            "選択した荷重ケース／荷重組合せを 1 件だけ解析します\
                             （組合せは参照する荷重ケースを解いてから線形和で求めます）",
                        )
                        .clicked()
                    {
                        if let Some(t) = target {
                            self.start_static_target_job(t);
                            if self.last_error.is_none() {
                                self.active_tab = Tab::Results;
                                self.results_view = ResultsView::Spatial;
                            }
                        }
                    }
                });
                ui.horizontal_wrapped(|ui| {
                    if ui
                        .add_enabled(!running, egui::Button::new("▶▶ 一括解析"))
                        .on_hover_text(
                            "全ての荷重ケースを解析し、全ての荷重組合せをその結果の線形和で\
                             求めます（並列スレッド数設定を使用）",
                        )
                        .clicked()
                    {
                        self.start_static_all_job();
                        if self.last_error.is_none() {
                            self.active_tab = Tab::Results;
                            self.results_view = ResultsView::Spatial;
                        }
                    }
                });
                if self.model.load_cases.is_empty() {
                    ui.colored_label(
                        crate::theme::GRAY_600,
                        "荷重ケースがありません。荷重タブで作成してください。",
                    );
                } else {
                    ui.colored_label(
                        crate::theme::GRAY_600,
                        "EX/EY（地震力）は準備計算が自動生成します。\
                         荷重組合せは荷重ケース単体の解析結果の線形和として求めます。",
                    );
                }
            });
    }

    /// 静的解析の実行対象（「対象」ドロップダウンの選択）を解決する。
    ///
    /// 優先順は「選択中の対象（モデル編集で失効していないもの）」→
    /// 「ナビゲータ／荷重表で選択中の荷重ケース」→「荷重ケースの先頭」。
    fn resolved_analysis_target(&self) -> Option<StaticTarget> {
        let valid = |t: StaticTarget| match t {
            StaticTarget::Case(id) => self.model.load_cases.iter().any(|c| c.id == id),
            StaticTarget::Combo(i) => i < self.model.combinations.len(),
        };
        self.analysis_target
            .filter(|t| valid(*t))
            .or_else(|| {
                self.nav
                    .focus_load_case
                    .filter(|id| self.model.load_cases.iter().any(|c| c.id == *id))
                    .map(StaticTarget::Case)
            })
            .or_else(|| {
                self.model
                    .load_cases
                    .first()
                    .map(|c| StaticTarget::Case(c.id))
            })
    }

    /// 静的解析の実行対象の表示名（ドロップダウンの選択表示）。対象が失効している
    /// 場合は `None`。
    fn static_target_label(&self, target: StaticTarget) -> Option<String> {
        match target {
            StaticTarget::Case(id) => self
                .model
                .load_cases
                .iter()
                .find(|c| c.id == id)
                .map(|c| format!("[{}] {}", c.id.0, c.name)),
            StaticTarget::Combo(i) => self.model.combinations.get(i).map(|c| c.name.clone()),
        }
    }

    /// 固有値解析。
    fn eigen_section(&mut self, ui: &mut egui::Ui, running: bool) {
        egui::CollapsingHeader::new("固有値")
            .default_open(false)
            .id_salt("as_eigen")
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.label("モード数:");
                    let mut n = self.analysis_cfg.n_modes;
                    ui.add(egui::DragValue::new(&mut n).range(1..=30));
                    self.analysis_cfg.n_modes = n;
                    if ui
                        .add_enabled(!running, egui::Button::new("▶ 実行"))
                        .clicked()
                    {
                        // UI スレッドをブロックしないようバックグラウンドで実行する
                        // （他の解析と同じジョブ経路）。
                        self.start_eigen_job(self.analysis_cfg.n_modes);
                    }
                });
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "質量方式は準備計算の「計算条件」で設定します。",
                );
            });
    }

    /// 増分解析（プッシュオーバー）。
    fn pushover_section(&mut self, ui: &mut egui::Ui, running: bool) {
        egui::CollapsingHeader::new("増分解析")
            .default_open(false)
            .id_salt("as_pushover")
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.label("方向:");
                    ui.selectable_value(&mut self.analysis_cfg.push_dir, SeismicDir::X, "X");
                    ui.selectable_value(&mut self.analysis_cfg.push_dir, SeismicDir::Y, "Y");
                    ui.label("ステップ:");
                    ui.add(egui::DragValue::new(&mut self.analysis_cfg.push_steps).range(1..=100));
                });
                // 終了目標（いずれかへの到達で解析を打ち切る）。両方とも無効なら
                // 荷重制御 λ=1 まで解析する（solver 側 PushoverTarget の既定挙動）。
                ui.horizontal_wrapped(|ui| {
                    ui.checkbox(
                        &mut self.analysis_cfg.push_use_drift_angle,
                        "目標層間変形角:",
                    )
                    .on_hover_text("全層の層間変形角がこの値に達した時点で解析を打ち切ります");
                    ui.label("1/");
                    ui.add_enabled(
                        self.analysis_cfg.push_use_drift_angle,
                        egui::DragValue::new(&mut self.analysis_cfg.push_drift_denom)
                            .speed(10.0)
                            .range(50.0..=1000.0),
                    );
                    ui.separator();
                    ui.checkbox(&mut self.analysis_cfg.push_use_max_disp, "目標変位[mm]:")
                        .on_hover_text("頂部変位がこの値に達した時点で解析を打ち切ります");
                    ui.add_enabled(
                        self.analysis_cfg.push_use_max_disp,
                        egui::DragValue::new(&mut self.analysis_cfg.push_max_disp)
                            .speed(10.0)
                            .range(1.0..=10000.0),
                    );
                });
                if !self.analysis_cfg.push_use_max_disp && !self.analysis_cfg.push_use_drift_angle {
                    ui.colored_label(
                        crate::theme::GRAY_600,
                        "目標未設定: 荷重制御(λ=1)までで終了します。",
                    );
                }
                ui.horizontal_wrapped(|ui| {
                    use squid_n_solver::pushover::DuctilityMethod;
                    ui.label("塑性率方式:")
                        .on_hover_text("ファイバーモデルの塑性率（構造力学）");
                    egui::ComboBox::from_id_salt("ductility_method")
                        .selected_text(match self.analysis_cfg.ductility_method {
                            DuctilityMethod::ReferenceStrain => "基点歪み",
                            DuctilityMethod::WeightedAverageJm => "重み付け平均Jm",
                            DuctilityMethod::FirstYield => "降伏時",
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut self.analysis_cfg.ductility_method,
                                DuctilityMethod::ReferenceStrain,
                                "基点歪み（RC:引張0.01/圧縮0.005・鉄骨0.01）",
                            );
                            ui.selectable_value(
                                &mut self.analysis_cfg.ductility_method,
                                DuctilityMethod::WeightedAverageJm,
                                "重み付け平均塑性率 Jm≥1",
                            );
                            ui.selectable_value(
                                &mut self.analysis_cfg.ductility_method,
                                DuctilityMethod::FirstYield,
                                "降伏発生時（塑性率1）",
                            );
                        });
                });
                ui.horizontal_wrapped(|ui| {
                    ui.checkbox(
                        &mut self.analysis_cfg.push_apply_long_term,
                        "長期荷重を初期載荷",
                    )
                    .on_hover_text(
                        "長期系荷重ケース（固定・積載等）を水平力増分の前に載荷し、\
                         その応力状態を初期条件とします。長期荷重ケースがない場合は\
                         無視されます。",
                    );
                });
                ui.horizontal_wrapped(|ui| {
                    use squid_n_solver::pushover::PushoverControl;
                    ui.label("増分方式:").on_hover_text(
                        "荷重増分のみは比較検証用。変位制御へ移行せず、終了目標が有効な場合は\
                         λ=1 を超えて荷重を増分し、収束しなくなった時点（耐力ピーク近傍）で\
                         打ち切ります。耐力低下域は追跡できません。",
                    );
                    egui::ComboBox::from_id_salt("push_control")
                        .selected_text(match self.analysis_cfg.push_control {
                            PushoverControl::Phased => "段階制御（荷重→変位）",
                            PushoverControl::LoadOnly => "荷重増分のみ",
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut self.analysis_cfg.push_control,
                                PushoverControl::Phased,
                                "段階制御（荷重→変位）",
                            );
                            ui.selectable_value(
                                &mut self.analysis_cfg.push_control,
                                PushoverControl::LoadOnly,
                                "荷重増分のみ",
                            )
                            .on_hover_text(
                                "比較検証用。変位制御へ移行せず、終了目標が有効な場合は\
                                 λ=1 を超えて荷重を増分し、収束しなくなった時点（耐力ピーク近傍）\
                                 で打ち切ります。耐力低下域は追跡できません。",
                            );
                        });
                });
                ui.horizontal_wrapped(|ui| {
                    if ui
                        .add_enabled(!running, egui::Button::new("▶ 実行"))
                        .clicked()
                    {
                        self.start_pushover_job();
                    }
                    if self.job.as_ref().is_some_and(|j| j.label == "増分解析") {
                        ui.spinner();
                    }
                });
            });
    }

    /// 時刻歴応答解析（線形／非線形）。
    fn time_history_section(&mut self, ui: &mut egui::Ui, running: bool) {
        egui::CollapsingHeader::new("時刻歴応答")
            .default_open(false)
            .id_salt("as_time_history")
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.label("方向:");
                    ui.selectable_value(&mut self.analysis_cfg.th_dir, ThDir::X, "X");
                    ui.selectable_value(&mut self.analysis_cfg.th_dir, ThDir::Y, "Y");
                    ui.selectable_value(&mut self.analysis_cfg.th_dir, ThDir::Xy, "X+Y")
                        .on_hover_text("同一波形を両方向へ同時入力(CSV は2列)");
                    ui.separator();
                    ui.checkbox(
                        &mut self.analysis_cfg.th_nonlinear,
                        "非線形(復元力特性を考慮)",
                    )
                    .on_hover_text(
                        "各部材の復元力特性（ひび割れ・降伏等）を考慮し、\
                         各時刻ステップを Newton 反復で解く時刻歴応答解析。\
                         積分法は Newmark-β 固定になります。",
                    );
                });
                ui.horizontal_wrapped(|ui| {
                    ui.label("積分法:");
                    // 低: 非線形 ON でも Newmark-β は選択状態のまま有効表示にする
                    // （非線形時刻歴は常に Newmark-β 相当で解くため、選択自体は無効化する
                    // 理由がない）。HHT-α のみ無効化し、hover で理由を示す。
                    ui.selectable_value(
                        &mut self.analysis_cfg.th_integrator,
                        ThIntegrator::NewmarkBeta,
                        "Newmark-β",
                    );
                    ui.add_enabled_ui(!self.analysis_cfg.th_nonlinear, |ui| {
                        ui.selectable_value(
                            &mut self.analysis_cfg.th_integrator,
                            ThIntegrator::HhtAlpha,
                            "HHT-α(α=-0.1)",
                        )
                        .on_disabled_hover_text(
                            "非線形時刻歴は Newmark-β 固定です（HHT-α は線形専用）。",
                        );
                    });
                });
                if self.analysis_cfg.th_nonlinear {
                    ui.horizontal_wrapped(|ui| {
                        ui.label("Newton反復: 最大回数");
                        ui.add(
                            egui::DragValue::new(&mut self.analysis_cfg.th_max_iter).range(1..=500),
                        );
                        ui.label("収束許容誤差(相対):");
                        ui.add(
                            egui::DragValue::new(&mut self.analysis_cfg.th_tol)
                                .speed(1e-7)
                                .range(1e-9..=1e-2),
                        );
                    });
                }
                ui.horizontal_wrapped(|ui| {
                    ui.label("記録間引き(0=自動):");
                    ui.add(
                        egui::DragValue::new(&mut self.analysis_cfg.th_record_every)
                            .range(0..=100000),
                    )
                    .on_hover_text(
                        "3D アニメーション・層応答グラフ・部材履歴用の詳細記録\
                         （ThRecording）を N ステップごとに 1 フレーム記録します。\
                         0 なら記録フレーム数が概ね 1000 になるよう自動決定します\
                         （線形・HHT-α・非線形の 3 経路とも共通）。\
                         ピーク値（最大変位・最大内力・層せん断力係数の最大値）は\
                         間引かず全ステップで更新するため、この値は精度ではなく\
                         アニメーション・履歴グラフの解像度とメモリ使用量に影響します。",
                    );
                });
                ui.horizontal_wrapped(|ui| {
                    ui.add_enabled(
                        self.analysis_cfg.th_nonlinear,
                        egui::Checkbox::new(
                            &mut self.analysis_cfg.th_apply_long_term,
                            "長期荷重を初期状態として考慮",
                        ),
                    )
                    .on_hover_text(
                        "長期系荷重ケース（固定・積載等）を時刻歴開始前に静的載荷し、\
                         その応力状態を初期条件とします。長期荷重ケースがない場合は\
                         無視されます。",
                    )
                    .on_disabled_hover_text(
                        "線形時刻歴は重ね合わせ運用のため対象外です\
                         （「非線形」をONにすると使用できます）。",
                    );
                });
                ui.horizontal_wrapped(|ui| {
                    ui.label("減衰:");
                    ui.selectable_value(
                        &mut self.analysis_cfg.th_damping_model,
                        ThDampingModel::StiffnessProportional,
                        "剛性比例",
                    );
                    ui.selectable_value(
                        &mut self.analysis_cfg.th_damping_model,
                        ThDampingModel::Rayleigh,
                        "Rayleigh",
                    );
                    ui.selectable_value(
                        &mut self.analysis_cfg.th_damping_model,
                        ThDampingModel::Modal,
                        "モード別",
                    )
                    .on_hover_text("各モードに減衰比 h を与える（非線形は初期剛性モード）");
                    ui.selectable_value(
                        &mut self.analysis_cfg.th_damping_model,
                        ThDampingModel::TangentAlpha1,
                        "接線(α1一定)",
                    )
                    .on_hover_text("瞬間剛性比例。C=2h/ω1e·Kt を毎ステップ再構成");
                    ui.selectable_value(
                        &mut self.analysis_cfg.th_damping_model,
                        ThDampingModel::TangentH1,
                        "接線(h1一定)",
                    )
                    .on_hover_text("瞬間剛性比例。ω1 を毎ステップ更新し減衰比 h1 を保つ");
                    ui.separator();
                    ui.label(match self.analysis_cfg.th_damping_model {
                        ThDampingModel::StiffnessProportional
                        | ThDampingModel::TangentAlpha1
                        | ThDampingModel::TangentH1 => "減衰比 h:",
                        ThDampingModel::Modal => "減衰比 h(全モード):",
                        ThDampingModel::Rayleigh => "h1(1次):",
                    });
                    ui.add(
                        egui::DragValue::new(&mut self.analysis_cfg.th_damping)
                            .speed(0.005)
                            .range(0.0..=0.3),
                    );
                    if self.analysis_cfg.th_damping_model == ThDampingModel::Rayleigh {
                        ui.label("h2(2次):");
                        ui.add(
                            egui::DragValue::new(&mut self.analysis_cfg.th_h2)
                                .speed(0.005)
                                .range(0.0..=0.3),
                        );
                    }
                });
                ui.horizontal_wrapped(|ui| {
                    ui.label("サンプル波: dt[s]");
                    ui.add(
                        egui::DragValue::new(&mut self.analysis_cfg.th_dt)
                            .speed(0.001)
                            .range(0.001..=0.1),
                    );
                    ui.label("継続[s]");
                    ui.add(
                        egui::DragValue::new(&mut self.analysis_cfg.th_duration)
                            .speed(0.5)
                            .range(1.0..=120.0),
                    );
                    ui.label("周期[s]");
                    ui.add(
                        egui::DragValue::new(&mut self.analysis_cfg.th_period)
                            .speed(0.05)
                            .range(0.05..=5.0),
                    );
                    ui.label("振幅[mm/s²]");
                    ui.add(
                        egui::DragValue::new(&mut self.analysis_cfg.th_amp)
                            .speed(50.0)
                            .range(10.0..=10000.0),
                    );
                });
                // 位相差入力（ねじれ加振）。構造動力学の位相差入力解析 t=(L·sinθ)/Vs。
                ui.horizontal_wrapped(|ui| {
                    ui.checkbox(&mut self.analysis_cfg.phase_diff_enabled, "位相差入力")
                        .on_hover_text(
                            "見かけ速度で地震動が矩形基礎を通過する位相差からねじれ加振を生成",
                        );
                    ui.add_enabled_ui(self.analysis_cfg.phase_diff_enabled, |ui| {
                        ui.label("Vs[m/s]");
                        ui.add(
                            egui::DragValue::new(&mut self.analysis_cfg.phase_diff_vs)
                                .speed(10.0)
                                .range(50.0..=2000.0),
                        );
                        ui.label("L[m]");
                        ui.add(
                            egui::DragValue::new(&mut self.analysis_cfg.phase_diff_length_m)
                                .speed(1.0)
                                .range(1.0..=500.0),
                        );
                        ui.label("θ[°]");
                        ui.add(
                            egui::DragValue::new(&mut self.analysis_cfg.phase_diff_incidence_deg)
                                .speed(1.0)
                                .range(0.0..=90.0),
                        );
                        ui.selectable_value(&mut self.analysis_cfg.phase_diff_dir_y, false, "X");
                        ui.selectable_value(&mut self.analysis_cfg.phase_diff_dir_y, true, "Y");
                        let lag = squid_n_solver::phase_diff::phase_lag_time(
                            self.analysis_cfg.phase_diff_length_m,
                            self.analysis_cfg.phase_diff_incidence_deg,
                            self.analysis_cfg.phase_diff_vs,
                        );
                        ui.label(format!("位相遅れ {:.4}s", lag));
                    });
                });
                ui.horizontal_wrapped(|ui| {
                    if ui
                        .add_enabled(!running, egui::Button::new("▶ サンプル波で実行"))
                        .on_hover_text("正弦減衰波を生成して時刻歴解析を実行します")
                        .clicked()
                    {
                        let wave = Self::sample_wave(&self.analysis_cfg);
                        self.start_time_history_job(wave);
                    }
                    if ui
                        .add_enabled(!running, egui::Button::new("📂 波形CSVを開いて実行…"))
                        .on_hover_text(
                            "1 行 1 値(加速度 gal)の CSV/テキスト。dt は上の設定値を使用します",
                        )
                        .clicked()
                    {
                        self.run_time_history_from_csv();
                    }
                    if self
                        .job
                        .as_ref()
                        .is_some_and(|j| j.label.starts_with("時刻歴応答"))
                    {
                        ui.spinner();
                    }
                });
                ui.label(
                    egui::RichText::new("応答グラフは入力の大きい方向を記録")
                        .small()
                        .color(crate::theme::GRAY_600),
                );
            });
    }

    /// 波形 CSV（X/Y: 1 行 1 値、X+Y: 1 行 2 列、いずれも gal 単位）を選択して
    /// 時刻歴解析をジョブ実行する。
    pub(crate) fn run_time_history_from_csv(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("波形 (CSV/テキスト)", &["csv", "txt", "dat"])
            .pick_file()
        else {
            return;
        };
        let content = match std::fs::read_to_string(&path) {
            Ok(c) => c,
            Err(e) => {
                self.report_error(format!("波形読込エラー: {}", e));
                return;
            }
        };
        let dir = self.analysis_cfg.th_dir;
        let (col1, col2) = match parse_wave_csv(&content, dir) {
            Ok(v) => v,
            Err(e) => {
                self.report_error(e);
                return;
            }
        };
        let wave = match dir {
            // X/Y は単一列を方向へ振り分ける（従来仕様、job::build_ground_motion 共用）。
            ThDir::X | ThDir::Y => {
                squid_n_job::build_ground_motion(self.analysis_cfg.th_dt, dir, col1)
            }
            // X+Y は CSV の 2 列がそのまま X・Y の入力になる
            // （build_ground_motion の Xy 分岐は「同一波形を複製」する仕様のため、
            // 別波形の 2 列読込はここで直接 GroundMotion を組み立てる）。
            ThDir::Xy => squid_n_solver::timehistory::GroundMotion {
                dt: self.analysis_cfg.th_dt,
                accel_x: col1,
                accel_y: col2,
                accel_theta: None,
            },
        };
        self.start_time_history_job(wave);
    }

    /// 結果タブの「表示対象」ドロップダウン用の選択肢（キーと表示名）を収集する。
    /// 静的ケース（ユーザー荷重・地震静的）に続けて荷重組合せを並べる。
    fn result_display_options(&self) -> Vec<(StaticKey, String)> {
        let mut opts = Vec::new();
        if let Some(r) = &self.results {
            for (key, _) in r.statics.iter() {
                let label = match key {
                    StaticCaseKey::User(id) => {
                        let nm = self
                            .model
                            .load_cases
                            .iter()
                            .find(|lc| lc.id == *id)
                            .map(|lc| lc.name.as_str())
                            .unwrap_or("");
                        format!("LC {} {}", id.0, nm)
                    }
                    StaticCaseKey::Seismic(SeismicDir::X) => "地震静的 (X方向)".to_string(),
                    StaticCaseKey::Seismic(SeismicDir::Y) => "地震静的 (Y方向)".to_string(),
                };
                opts.push((StaticKey::Case(*key), label));
            }
            for (i, (name, _)) in r.combos.iter().enumerate() {
                opts.push((StaticKey::Combo(i), name.clone()));
            }
        }
        opts
    }

    /// 結果タブ：3Dビューア と 時刻歴グラフを切替。
    pub(crate) fn results_tab_panel(&mut self, ui: &mut egui::Ui) {
        // 表示対象（荷重ケース／組合せ）の選択肢を先に収集する
        // （クロージャ内で self を可変借用しないため）。current_key は現在の表示対象。
        let result_options = self.result_display_options();
        let current_key = self.nav.focus_result.or(self.last_static);
        let mut selected_result: Option<StaticKey> = None;
        ui.horizontal(|ui| {
            let sel_spatial = self.results_view == ResultsView::Spatial;
            let sel_th = self.results_view == ResultsView::TimeHistory;
            let sel_po = self.results_view == ResultsView::Pushover;
            let sel_lm = self.results_view == ResultsView::LumpedMass;
            if ui.selectable_label(sel_spatial, "3D/応力図").clicked() {
                self.results_view = ResultsView::Spatial;
            }
            if ui.selectable_label(sel_th, "時刻歴").clicked() {
                self.results_view = ResultsView::TimeHistory;
            }
            if ui.selectable_label(sel_po, "増分解析").clicked() {
                self.results_view = ResultsView::Pushover;
            }
            if ui.selectable_label(sel_lm, "質点系モデル").clicked() {
                self.results_view = ResultsView::LumpedMass;
            }
            ui.separator();
            // 結果サマリ
            if let Some(r) = &self.results {
                ui.label(format!("静的ケース数: {}", r.statics.len()));
                if let Some(m) = &r.modal {
                    let t1 = m.period.first().copied().unwrap_or(0.0);
                    ui.label(format!("固有周期 T1: {:.3} s", t1));
                }
                let n_checks: usize = r.member_checks.iter().map(|m| m.positions.len()).sum();
                ui.label(format!("検定結果数: {}", n_checks));
            } else {
                ui.colored_label(crate::theme::GRAY_600, "▷ 未実行");
            }
            // 表示対象（荷重ケース／組合せ）の選択。変位図に加え、応力図・断面検定
            // （その組合せの長期/短期）まで切り替える。
            if !result_options.is_empty() {
                ui.separator();
                ui.label("表示対象:");
                let cur_label = current_key
                    .and_then(|k| result_options.iter().find(|(o, _)| *o == k))
                    .map(|(_, l)| l.clone())
                    .unwrap_or_else(|| "（選択）".to_string());
                egui::ComboBox::from_id_salt("results_display_selector")
                    .selected_text(cur_label)
                    .show_ui(ui, |ui| {
                        for (opt_key, label) in &result_options {
                            if ui
                                .selectable_label(current_key == Some(*opt_key), label)
                                .clicked()
                            {
                                selected_result = Some(*opt_key);
                            }
                        }
                    });
            }
        });
        if let Some(key) = selected_result {
            self.select_displayed_result(key);
        }
        ui.separator();
        match self.results_view {
            ResultsView::Spatial => crate::viewer::viewer_panel(ui, self),
            ResultsView::TimeHistory => crate::time_history_view::time_history_panel(ui, self),
            ResultsView::Pushover => self.pushover_panel(ui),
            ResultsView::LumpedMass => self.lumped_mass_panel(ui),
        }
    }

    /// 設計タブ：検定表（許容応力度・保有水平耐力）と MN 相関曲面ビューを切り替える。
    pub(crate) fn design_tab_panel(&mut self, ui: &mut egui::Ui) {
        // 断面算定の対象荷重（ケース／組合せ）を選ぶドロップダウン用の選択肢。
        // 長期/短期区分は選んだ組合せ名から自動判定され（令82条の荷重組合せ:
        // G+P=長期、地震・積雪・風入り=短期）、対象荷重の右に読み取り専用で表示する。
        let result_options = self.result_display_options();
        let current_key = self.nav.focus_result.or(self.last_static);
        let mut selected_result: Option<StaticKey> = None;
        ui.horizontal(|ui| {
            let sel_table = self.design_view == DesignView::Table;
            let sel_ult = self.design_view == DesignView::Ultimate;
            let sel_mn = self.design_view == DesignView::MnSurface;
            let sel_qty = self.design_view == DesignView::Quantities;
            if ui.selectable_label(sel_table, "検定表").clicked() {
                self.design_view = DesignView::Table;
            }
            if ui.selectable_label(sel_ult, "終局検定").clicked() {
                self.design_view = DesignView::Ultimate;
            }
            if ui.selectable_label(sel_mn, "MN相関曲面").clicked() {
                self.design_view = DesignView::MnSurface;
            }
            if ui.selectable_label(sel_qty, "数量積算").clicked() {
                self.design_view = DesignView::Quantities;
            }
            // 対象荷重の選択。選ぶとその組合せの内力・長期/短期で断面算定が再実行される。
            if !result_options.is_empty() {
                ui.separator();
                ui.label("対象荷重:");
                let cur_label = current_key
                    .and_then(|k| result_options.iter().find(|(o, _)| *o == k))
                    .map(|(_, l)| l.clone())
                    .unwrap_or_else(|| "（選択）".to_string());
                egui::ComboBox::from_id_salt("design_display_selector")
                    .selected_text(cur_label)
                    .show_ui(ui, |ui| {
                        for (opt_key, label) in &result_options {
                            if ui
                                .selectable_label(current_key == Some(*opt_key), label)
                                .clicked()
                            {
                                selected_result = Some(*opt_key);
                            }
                        }
                    });
                // 荷重継続性区分（許容応力度の長期/短期）。対象荷重から自動判定した
                // 結果の表示のみで、ここでの手動切替は行わない。
                let term_label = match self.design_term {
                    LoadTerm::Long => "長期",
                    LoadTerm::Short => "短期",
                };
                ui.label(format!("許容応力度: {term_label}")).on_hover_text(
                    "対象荷重（組合せ）の内容から自動判定します（令82条: G+P=長期、\
                         地震・積雪・風を含む組合せ=短期）。",
                );
            }
        });
        if let Some(key) = selected_result {
            self.select_displayed_result(key);
        }
        ui.separator();
        match self.design_view {
            DesignView::Table => crate::design_view::design_table(ui, self),
            DesignView::Ultimate => crate::ultimate_view::ultimate_table(ui, self),
            DesignView::MnSurface => crate::mn_view::mn_surface_panel(ui, self),
            DesignView::Quantities => crate::quantity_view::quantity_panel(ui, self),
        }
    }

    /// 増分解析結果（性能曲線・ヒンジ・崩壊機構）の表示。
    pub(crate) fn pushover_panel(&mut self, ui: &mut egui::Ui) {
        if self
            .results
            .as_ref()
            .and_then(|r| r.pushover.as_ref())
            .is_none()
        {
            ui.colored_label(
                crate::theme::GRAY_600,
                "増分解析結果がありません。解析タブから実行してください。",
            );
            return;
        }

        // 必要保有水平耐力の総合判定（Qu ≥ Qun = Ds·Fes·Qud）を先に算定する。
        // 実行ボタン→結果画面でそのまま OK/NG を確認できるよう、性能曲線より前に
        // バナー表示する。`compute_holding_capacity` は &mut self を要するため、
        // 以降の `po` 借用より前にここで所有権付きの結果へ落とす。
        let hc_verdict = self.compute_holding_capacity().ok();

        let po = self
            .results
            .as_ref()
            .and_then(|r| r.pushover.as_ref())
            .expect("checked above");

        // ── 必要保有水平耐力 判定バナー ──────────────────────────────
        match &hc_verdict {
            Some((res, _)) if !res.stories.is_empty() => {
                let ng = res.stories.iter().filter(|s| !s.ok).count();
                if ng == 0 {
                    ui.colored_label(
                        crate::theme::GOOD_GREEN,
                        format!(
                            "✔ 必要保有水平耐力を満足: 全 {} 層で Qu ≥ Qun（Qun = Ds·Fes·Qud）",
                            res.stories.len()
                        ),
                    );
                } else {
                    ui.colored_label(
                        crate::theme::ERROR_RED,
                        format!(
                            "✘ 必要保有水平耐力が不足: {} / {} 層で Qu < Qun。設計タブ「保有水平耐力」で詳細を確認してください。",
                            ng,
                            res.stories.len()
                        ),
                    );
                }
            }
            _ => {
                ui.colored_label(
                    crate::theme::GRAY_600,
                    "必要保有水平耐力の判定には荷重ケース EX／EY（地震力）の実行が必要です（解析タブ）。",
                );
            }
        }
        // 崩壊機構が未形成（部分崩壊形）の警告。崩壊機構が確定しない限り Ds・
        // 目標未到達のまま打ち切られた解析（非収束・特異化）は Qu が過小評価の
        // 可能性があるため、終了理由を警告として明示する。
        if po.termination.is_premature() {
            ui.colored_label(
                crate::theme::SECONDARY_AMBER,
                format!(
                    "⚠ 増分解析は目標到達前に打ち切られました（{}）。性能曲線が途中で\
                     途切れており、Qu はその時点までの最大値です。",
                    po.termination.describe()
                ),
            );
        }
        // 必要保有水平耐力は暫定値であることを明示する（日本の慣行: 崩壊機構の確定が
        // 必要保有水平耐力算定の前提）。
        if matches!(
            po.mechanism,
            squid_n_solver::pushover::MechanismType::Partial
        ) {
            ui.colored_label(
                crate::theme::SECONDARY_AMBER,
                "⚠ 崩壊機構が未形成（部分崩壊形）です。目標変位を増やして再実行するか設計を\
                 見直してください。崩壊機構が確定するまで Ds・必要保有水平耐力は暫定値です。",
            );
        }
        ui.separator();

        ui.horizontal(|ui| {
            ui.label(format!("保有水平耐力 Qu = {:.1} kN", po.qu / 1000.0));
            ui.separator();
            let mech = match &po.mechanism {
                squid_n_solver::pushover::MechanismType::Overall => "全体崩壊形".to_string(),
                squid_n_solver::pushover::MechanismType::StoryCollapse { layer } => {
                    // 層の呼び名は下端の階名（法令の「i 階」）。
                    let name = self
                        .model
                        .layers()
                        .get(*layer)
                        .map(|l| l.name.clone())
                        .unwrap_or_else(|| format!("{}", layer + 1));
                    format!("層崩壊形 ({name})")
                }
                squid_n_solver::pushover::MechanismType::Partial => "部分崩壊形".to_string(),
            };
            ui.label(format!("崩壊機構: {}", mech));
            ui.separator();
            ui.label(format!("ヒンジ発生 {} 件", po.hinges.len()));
            ui.separator();
            let control = match po.control {
                squid_n_solver::pushover::PushoverControl::Phased => "段階制御",
                squid_n_solver::pushover::PushoverControl::LoadOnly => "荷重増分のみ",
            };
            ui.label(format!("増分方式: {}", control));
        });
        // 塑性率（構造力学）の方式と最大値。
        ui.horizontal(|ui| {
            use squid_n_solver::pushover::DuctilityMethod;
            let method = match self.analysis_cfg.ductility_method {
                DuctilityMethod::ReferenceStrain => "基点歪み",
                DuctilityMethod::WeightedAverageJm => "重み付け平均Jm",
                DuctilityMethod::FirstYield => "降伏時",
            };
            let max_mu = po
                .hinges
                .iter()
                .map(|h| h.ductility)
                .fold(0.0_f64, f64::max);
            ui.label(format!("塑性率方式: {method}"));
            ui.separator();
            ui.label(format!("最大部材塑性率 μmax = {:.2}", max_mu));
        });

        // 層別の保有水平耐力（性能曲線・層別ピーク層せん断力）。加力方向により
        // 符号を持ちうるため絶対値を取ってから最大値を求める
        // （crates/squid-n-app/src/app/actions.rs の `story_qu` 算定と同じ着眼＝
        // capacity_curve 全点にわたる層せん断力の最大値／βu の分母）。
        let layers = self.model.layers();
        let n_stories = layers.len();
        let story_name = |i: usize| -> String {
            layers
                .get(i)
                .map(|l| l.name.clone())
                .unwrap_or_else(|| squid_n_core::model::default_story_name(i))
        };
        let story_qu_kn: Vec<f64> = (0..n_stories)
            .map(|i| {
                po.capacity_curve
                    .iter()
                    .filter_map(|p| p.story_shear.get(i).copied())
                    .map(f64::abs)
                    .fold(0.0_f64, f64::max)
                    / 1000.0
            })
            .collect();
        if !story_qu_kn.is_empty() {
            let line = story_qu_kn
                .iter()
                .enumerate()
                .map(|(i, q)| format!("{} {:.1} kN", story_name(i), q))
                .collect::<Vec<_>>()
                .join(" / ");
            ui.label(format!("層別 Qu: {line}"));
        }

        // 性能曲線（層別: 層間変位 - 層せん断力）。層ごとに 1 本の折れ線を描く
        // （既存の色（`crate::theme` のデータ系色）を層番号で巡回して使用）。
        const STORY_COLORS: [egui::Color32; 8] = [
            crate::theme::DATA_BLUE,
            crate::theme::GOOD_GREEN,
            crate::theme::PARETO_RED,
            crate::theme::BEST_YELLOW,
            crate::theme::HILITE_PURPLE,
            crate::theme::SECONDARY_AMBER,
            crate::theme::BLUE_600,
            crate::theme::GREEN_600,
        ];
        egui_plot::Plot::new("pushover_curve")
            .x_axis_label("層間変位 [mm]")
            .y_axis_label("層せん断力 [kN]")
            .legend(egui_plot::Legend::default())
            .height(ui.available_height() * 0.6)
            .show(ui, |plot_ui| {
                for i in 0..n_stories {
                    let points: Vec<[f64; 2]> = po
                        .capacity_curve
                        .iter()
                        .map(|p| {
                            let drift = p.story_drift.get(i).copied().unwrap_or(0.0).abs();
                            let shear = p.story_shear.get(i).copied().unwrap_or(0.0).abs() / 1000.0;
                            [drift, shear]
                        })
                        .collect();
                    let color = STORY_COLORS[i % STORY_COLORS.len()];
                    plot_ui.line(
                        egui_plot::Line::new(
                            story_name(i),
                            egui_plot::PlotPoints::from(points.clone()),
                        )
                        .color(color)
                        .width(2.0_f32),
                    );
                    // 実際に釣合いを解いて確定した増分ステップの点をマーカーで示す。
                    // 点間を結ぶ折れ線は単なる補間であり計算結果ではないため、
                    // どこが計算点かをマーカーで判別できるようにする（同名で登録し
                    // 凡例のエントリは折れ線と共有する）。
                    plot_ui.points(
                        egui_plot::Points::new(story_name(i), egui_plot::PlotPoints::from(points))
                            .color(color)
                            .radius(3.0_f32)
                            .shape(egui_plot::MarkerShape::Circle),
                    );
                }
            });

        // ヒンジ発生履歴（先頭 20 件）
        ui.separator();
        ui.strong("ヒンジ発生履歴");
        egui::ScrollArea::vertical().show(ui, |ui| {
            for h in po.hinges.iter().take(20) {
                let level = match h.level {
                    squid_n_solver::pushover::HingeLevel::Crack => "ひび割れ",
                    squid_n_solver::pushover::HingeLevel::Yield => "降伏",
                    squid_n_solver::pushover::HingeLevel::Ultimate => "終局",
                };
                ui.label(format!(
                    "step {}: 部材 {} pos={:.2} {} (μ={:.2})",
                    h.step, h.elem.0, h.pos, level, h.ductility
                ));
            }
            if po.hinges.len() > 20 {
                ui.label(format!("... 他 {} 件", po.hinges.len() - 20));
            }
        });
    }

    /// 質点系（串団子）モデルの表示。増分解析結果から層 Q-δ を
    /// トリリニア縮約し、層ごとの質量・階高・復元力特性を一覧する
    /// （構造動力学の質点系解析モデル）。
    pub(crate) fn lumped_mass_panel(&mut self, ui: &mut egui::Ui) {
        use squid_n_solver::lumped_mass::{build_lumped_mass_model, LumpedMassType};

        let Some(po) = self.results.as_ref().and_then(|r| r.pushover.as_ref()) else {
            ui.colored_label(
                crate::theme::GRAY_600,
                "増分解析結果がありません。質点系モデルは\
                 増分解析結果から生成します。解析タブから実行してください。",
            );
            return;
        };

        // モデル化タイプ・第1折点判定の割線剛性比を選択。
        ui.horizontal(|ui| {
            ui.label("モデル化タイプ:");
            let cur = self.analysis_cfg.lumped_mass_type;
            egui::ComboBox::from_id_salt("lumped_mass_type")
                .selected_text(cur.label())
                .show_ui(ui, |ui| {
                    for t in [
                        LumpedMassType::EquivalentShear,
                        LumpedMassType::EquivalentBendingShear,
                        LumpedMassType::BendingShearSeparated,
                    ] {
                        ui.selectable_value(&mut self.analysis_cfg.lumped_mass_type, t, t.label());
                    }
                });
            ui.separator();
            ui.label("第1折点 割線比:");
            ui.add(
                egui::DragValue::new(&mut self.analysis_cfg.lumped_secant_ratio)
                    .speed(0.01)
                    .range(0.3..=0.95),
            );
        });
        ui.separator();

        // 増分解析から串団子モデルを生成（軽量なので毎フレーム再構成）。
        let lm = build_lumped_mass_model(
            &self.model,
            po,
            self.analysis_cfg.lumped_mass_type,
            self.analysis_cfg.lumped_secant_ratio,
        );

        let total_mass: f64 = lm.stories.iter().map(|s| s.mass).sum();
        ui.horizontal(|ui| {
            ui.label(format!("質点数: {}", lm.stories.len()));
            ui.separator();
            ui.label(format!("総質量: {:.1} t", total_mass));
            ui.separator();
            ui.label(format!("モデル: {}", lm.model_type.label()));
        });
        ui.separator();

        // 層ごとの質点・復元力特性（トリリニア）を一覧。上層から順に表示。
        egui::ScrollArea::vertical().show(ui, |ui| {
            // model.stories と stick は同順（build_lumped_mass_model が順に生成）。
            // 上層から順に見せるため、表示順は逆順にした索引で引く。
            let order: Vec<usize> = (0..lm.stories.len()).rev().collect();
            crate::table_util::standard_table(
                ui,
                "lumped_mass_stories",
                &[
                    Col::label("階"),
                    Col::num("質量[t]"),
                    Col::num("階高[mm]"),
                    Col::num("K1[kN/mm]"),
                    Col::num("K2[kN/mm]"),
                    Col::num("K3[kN/mm]"),
                    Col::wide_num("第1折点 δ1/Q1"),
                    Col::wide_num("第2折点 δ2/Q2"),
                    Col::wide_num("第3折点 δ3/Q3"),
                ],
                order.len(),
                |row| {
                    let i = order[row.index()];
                    let stick = &lm.stories[i];
                    let name = self
                        .model
                        .stories
                        .get(i)
                        .map(|s| s.name.as_str())
                        .unwrap_or("-");
                    let sk = &stick.skeleton;
                    row.col(|ui| {
                        crate::table_util::text_cell(ui, name);
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.2}", stick.mass));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.0}", stick.height));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.1}", sk.k1 / 1000.0));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.1}", sk.k2() / 1000.0));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.1}", sk.k3() / 1000.0));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.2} / {:.0}", sk.d1, sk.q1 / 1000.0));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.2} / {:.0}", sk.d2, sk.q2 / 1000.0));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.2} / {:.0}", sk.d3, sk.q3 / 1000.0));
                    });
                },
            );

            ui.add_space(6.0);
            ui.colored_label(
                crate::theme::GRAY_600,
                "K は [kN/mm]、Q は [kN]、δ は [mm]。骨格は増分解析の層 Q-δ を\
                 等包絡面積則でトリリニア縮約したもの。",
            );
        });

        // ── 質点系（せん断型）時刻歴応答解析 ──────────────────────────
        ui.separator();
        let mut run_stick = false;
        let mut clear_stick = false;
        ui.horizontal(|ui| {
            if ui
                .button("▶ 質点系時刻歴を実行")
                .on_hover_text(
                    "サンプル波（「時刻歴応答」の dt/継続/周期/振幅・減衰比）で串団子モデルの\
                     非線形時刻歴（Newmark-β、各層トリリニア）を実行します",
                )
                .clicked()
            {
                run_stick = true;
            }
            if self.stick_response.is_some() && ui.button("結果クリア").clicked() {
                clear_stick = true;
            }
        });
        if run_stick {
            let accel = Self::sample_wave(&self.analysis_cfg).accel_x;
            let res = squid_n_solver::lumped_mass::lumped_mass_time_history(
                &lm,
                &accel,
                self.analysis_cfg.th_dt,
                self.analysis_cfg.th_damping,
            );
            self.stick_response = Some(res);
        }
        if clear_stick {
            self.stick_response = None;
        }
        if let Some(res) = &self.stick_response {
            let roof_peak = res
                .roof_disp
                .iter()
                .cloned()
                .fold(0.0f64, |m, v| m.max(v.abs()));
            let mu_max = res.story_ductility.iter().cloned().fold(0.0f64, f64::max);
            ui.horizontal(|ui| {
                ui.label(format!("頂部最大変位: {:.2} mm", roof_peak));
                ui.separator();
                ui.label(format!("最大層塑性率 μ: {:.2}", mu_max));
            });
            if res.non_converged_steps > 0 {
                ui.colored_label(
                    crate::theme::SECONDARY_AMBER,
                    format!(
                        "⚠ Newton 反復が {} ステップで収束しませんでした。\
                         応答値は参考値です（dt を小さくすると改善する場合があります）。",
                        res.non_converged_steps
                    ),
                );
            }
            // 上層から順に見せるため、表示順は逆順にした索引で引く。
            let order: Vec<usize> = (0..res.story_peak_drift.len()).rev().collect();
            crate::table_util::standard_table(
                ui,
                "stick_th_result",
                &[
                    Col::label("階"),
                    Col::num("最大層間変形[mm]"),
                    Col::num("最大層せん断[kN]"),
                    Col::num("塑性率μ"),
                ],
                order.len(),
                |row| {
                    let i = order[row.index()];
                    let name = self
                        .model
                        .stories
                        .get(i)
                        .map(|s| s.name.as_str())
                        .unwrap_or("-");
                    row.col(|ui| {
                        crate::table_util::text_cell(ui, name);
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.2}", res.story_peak_drift[i]));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.0}", res.story_peak_shear[i] / 1000.0));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.2}", res.story_ductility[i]));
                    });
                },
            );
            let pts: Vec<[f64; 2]> = res
                .time
                .iter()
                .zip(res.roof_disp.iter())
                .map(|(&t, &d)| [t, d])
                .collect();
            egui_plot::Plot::new("stick_roof_plot")
                .height(160.0)
                .x_axis_label("時間[s]")
                .y_axis_label("頂部変位[mm]")
                .show(ui, |pu| {
                    pu.line(
                        egui_plot::Line::new("roof", egui_plot::PlotPoints::from(pts))
                            .color(crate::theme::DATA_BLUE),
                    );
                });
        }
    }

    /// レポートタブ：CSV レポートのプレビューとエクスポート。
    pub(crate) fn report_tab_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("レポート");
        if !crate::summary::has_report_content(&self.results) {
            ui.colored_label(
                crate::theme::GRAY_600,
                "解析結果がありません。解析タブから実行するとレポートを生成できます。",
            );
            return;
        }
        let csv = crate::summary::build_report_csv(self);
        ui.horizontal(|ui| {
            if ui.button("💾 CSV エクスポート…").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("CSV", &["csv"])
                    .set_file_name("report.csv")
                    .save_file()
                {
                    if let Err(e) = std::fs::write(&path, &csv) {
                        self.report_error(format!("レポート保存エラー: {}", e));
                    }
                }
            }
            if ui.button("📋 クリップボードへコピー").clicked() {
                ui.ctx().copy_text(csv.clone());
            }
        });
        ui.separator();
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.add(
                egui::TextEdit::multiline(&mut csv.as_str())
                    .font(egui::TextStyle::Monospace)
                    .desired_width(f32::INFINITY),
            );
        });
    }

    /// 右ペイン：選択要素のインスペクタ。
    /// 3D/ナビゲータ/テーブルの選択（現時点では focus_*）を表示。断面編集は UI-4 で拡充。
    pub(crate) fn inspector_panel(&mut self, ui: &mut egui::Ui) {
        // 遅延アクション（借用チェーン回避：UI 内で self.model を immutable borrow 中に
        // mut borrow できないため、複製ボタンクリックは一旦 here に保存）
        let mut duplicate_member = None;
        let mut highlight_section_members: Option<Vec<ElemId>> = None;
        ui.group(|ui| {
            ui.strong("インスペクタ");
            ui.separator();

            // 選択された部材の諸元
            if let Some(elem_id) = self.nav.focus_member {
                if let Some(e) = self.model.element(elem_id) {
                    ui.label(format!("部材 ID: {}", e.id.0));
                    let n0 = e.nodes.first().map(|n| n.0).unwrap_or(0);
                    let n1 = e.nodes.get(1).map(|n| n.0).unwrap_or(0);
                    ui.label(format!("節点 I/J: {} / {}", n0, n1));
                    if let Some(sec_id) = e.section {
                        if let Some(sec) = self
                            .model
                            .sections
                            .get(sec_id.index())
                            .filter(|s| s.id == sec_id)
                        {
                            ui.label(format!("断面: {} ({})", sec.name, sec_id.0));
                            ui.label(format!(
                                "  A = {} cm²",
                                fmt_section_prop(area_cm2(sec.area))
                            ));
                            ui.label(format!(
                                "  Iy= {} cm⁴",
                                fmt_section_prop(inertia_cm4(sec.iy))
                            ));
                            ui.label(format!(
                                "  Iz= {} cm⁴",
                                fmt_section_prop(inertia_cm4(sec.iz))
                            ));
                            // 影響数: 同一断面を使う部材数
                            let n_used = self
                                .model
                                .elements
                                .iter()
                                .filter(|o| o.section == Some(sec_id))
                                .count();
                            ui.colored_label(
                                crate::theme::BLUE_500,
                                format!("この断面を使う {} 部材に影響", n_used),
                            );
                            // UI-4: 複製ボタン（UI設計 §3）。同断面を新規IDで複製し、
                            // 当該部材のみ新断面に割当。
                            if ui.button("📋 複製してこの部材だけ別断面に").clicked()
                            {
                                duplicate_member = Some(elem_id);
                            }
                        }
                    } else {
                        ui.label("断面: 未割当");
                    }
                    // 材料は断面が持つ。ここでは断面から引いた実効値を表示する。
                    if let Some(mat) = self.model.element_material(e) {
                        ui.label(format!("材料: {} ({})", mat.name, mat.id.0));
                        ui.label(format!("  E = {:.1} N/mm²", mat.young));
                        if let Some(fc) = mat.fc {
                            ui.label(format!("  Fc = {:.1} N/mm²", fc));
                        }
                    }
                    ui.separator();
                    // 検定結果サマリ（同一部材）
                    if let Some(r) = &self.results {
                        let positions = r
                            .member_checks
                            .iter()
                            .find(|m| m.elem == elem_id)
                            .map(|m| m.positions.as_slice())
                            .unwrap_or(&[]);
                        ui.label(format!("検定結果（{} 位置）", positions.len()));
                        for p in positions.iter().take(8) {
                            match &p.outcome {
                                squid_n_design_jp::CheckOutcome::Checked(cr) => {
                                    let ratio = cr.ratio();
                                    let color = crate::theme::status_color(ratio);
                                    ui.colored_label(
                                        color,
                                        format!("  pos={:.2} 検定比={:.3}", p.xi, ratio),
                                    );
                                }
                                squid_n_design_jp::CheckOutcome::Skipped { reason } => {
                                    ui.colored_label(
                                        crate::theme::GRAY_600,
                                        format!("  pos={:.2} 検定不能（{reason}）", p.xi),
                                    );
                                }
                            }
                        }
                        if positions.len() > 8 {
                            ui.label(format!("  ... 他 {} 件", positions.len() - 8));
                        }
                    }
                } else {
                    ui.colored_label(crate::theme::GRAY_600, "部材を選択してください");
                }
            } else {
                ui.colored_label(
                    egui::Color32::from_rgb(150, 150, 150),
                    "部材を選択してください",
                );
            }

            // 選択された断面の諸元（断面テーブルの行選択と連動）
            if let Some(sec_id) = self.nav.focus_section {
                if let Some(sec) = self.model.sections.iter().find(|s| s.id == sec_id) {
                    ui.separator();
                    ui.strong("断面（選択中）");
                    ui.label(format!("名前: {} ({})", sec.name, sec_id.0));
                    ui.label(format!(
                        "  A = {} cm²",
                        fmt_section_prop(area_cm2(sec.area))
                    ));
                    ui.label(format!(
                        "  Iy= {} cm⁴",
                        fmt_section_prop(inertia_cm4(sec.iy))
                    ));
                    ui.label(format!(
                        "  Iz= {} cm⁴",
                        fmt_section_prop(inertia_cm4(sec.iz))
                    ));
                    let used: Vec<ElemId> = self
                        .model
                        .elements
                        .iter()
                        .filter(|e| e.section == Some(sec_id))
                        .map(|e| e.id)
                        .collect();
                    ui.label(format!("使用部材数: {}", used.len()));
                    if ui.button("🔍 使用部材を3Dハイライト").clicked() {
                        highlight_section_members = Some(used);
                    }
                }
            }

            ui.separator();
            // 選択された節点の諸元
            if let Some(node_id) = self.nav.focus_node {
                if let Some(node) = self.model.node(node_id) {
                    ui.label(format!("節点 ID: {}", node.id.0));
                    ui.label(format!(
                        "座標: ({:.3}, {:.3}, {:.3})",
                        node.coord[0], node.coord[1], node.coord[2]
                    ));
                    // 拘束情報
                    let is_fixed = node.restraint.0 != 0;
                    if is_fixed {
                        ui.label("拘束: あり");
                    } else {
                        ui.label("拘束: なし");
                    }
                }
            }
        });

        // 遅延実行: 複製ボタンが押されていたら EditCommand を叩く
        if let Some(member) = duplicate_member {
            self.undo.run(
                &mut self.model,
                Box::new(squid_n_edit::DuplicateSectionForMember { member }),
            );
            self.staleness.mark_edited();
        }
        // 遅延実行: 断面の使用部材ハイライトボタン
        if let Some(members) = highlight_section_members {
            self.selection.members = members;
        }
    }

    /// 下部ステータスバー。
    ///
    /// 部材/節点/断面サマリは常に右端に見えている必要がある一方、左側（ファイル名・
    /// 解析状況・エラーメッセージ）はいくらでも長くなりうる（特に ST-Bridge 取込警告は
    /// 複数件を連結した長文）。`ui.horizontal` 1本に全部を並べると、horizontal レイアウトの
    /// 子 `Ui` は主軸方向の幅を事実上無制限に確保するため、エラーメッセージに
    /// `Label::truncate()` を付けても truncate の基準となる幅がなく効かず、右側のサマリと
    /// 重なって表示されてしまう。そのためサマリの表示幅を先に採寸し、行を「左ゾーン
    /// （明示的に幅を制限する）」と「右ゾーン（サマリ専用）」へ矩形分割してから描画する。
    pub(crate) fn status_bar(&mut self, ui: &mut egui::Ui) {
        let summary = format!(
            "部材 {}. 節点 {}. 断面 {}.",
            self.model.elements.len(),
            self.model.nodes.len(),
            self.model.sections.len()
        );
        let body_font = egui::TextStyle::Body.resolve(ui.style());
        let summary_width = ui
            .painter()
            .layout_no_wrap(summary.clone(), body_font.clone(), crate::theme::GRAY_700)
            .size()
            .x;
        // 右ゾーンはサマリに加えて右ドックのパネル切替アイコン（🔍・⚙）も描くため、
        // アイコン2個分の幅＋ボタン余白＋アイコン間の間隔ぶんを確保幅に含める
        // （不足すると左ゾーンと重なる）。
        let icon_width = ui
            .painter()
            .layout_no_wrap("🔍".to_string(), body_font, crate::theme::GRAY_700)
            .size()
            .x
            + ui.spacing().button_padding.x * 2.0;
        let toggle_width = icon_width * 2.0 + ui.spacing().item_spacing.x;

        let row_rect = ui.available_rect_before_wrap();
        let gap = ui.spacing().item_spacing.x;
        let right_width = summary_width + gap + toggle_width;
        let right_rect = egui::Rect::from_min_max(
            egui::pos2(
                (row_rect.max.x - right_width - gap).max(row_rect.min.x),
                row_rect.min.y,
            ),
            row_rect.max,
        );
        let left_rect = egui::Rect::from_min_max(
            row_rect.min,
            egui::pos2((right_rect.min.x - gap).max(row_rect.min.x), row_rect.max.y),
        );

        #[allow(deprecated)]
        ui.allocate_ui_at_rect(left_rect, |ui| {
            ui.horizontal(|ui| {
                // ドック/パネル切替アイコン（Zed 風）。対象ドックが開いていて対象パネルが
                // アクティブなら閉じる。それ以外は開いてそのパネルをアクティブにする。
                let is_nav_active = self.left_dock_open && self.left_panel == LeftPanel::Navigator;
                if ui
                    .selectable_label(is_nav_active, "🗂")
                    .on_hover_text("ナビゲータ")
                    .clicked()
                    && toggle_dock_icon(&mut self.left_dock_open, is_nav_active)
                {
                    self.left_panel = LeftPanel::Navigator;
                }
                let is_draw_active = self.left_dock_open && self.left_panel == LeftPanel::DrawTools;
                if ui
                    .selectable_label(is_draw_active, "✏")
                    .on_hover_text("作成パレット")
                    .clicked()
                    && toggle_dock_icon(&mut self.left_dock_open, is_draw_active)
                {
                    self.left_panel = LeftPanel::DrawTools;
                }
                // 左ドック用と下ドック用のアイコン群の間に区切りを入れ、
                // どのアイコンがどの領域を操作するのかを見分けられるようにする。
                ui.separator();
                let is_log_active = self.bottom_dock_open && self.bottom_tab == BottomTab::Log;
                if ui
                    .selectable_label(is_log_active, "📜")
                    .on_hover_text("ログ")
                    .clicked()
                    && toggle_dock_icon(&mut self.bottom_dock_open, is_log_active)
                {
                    self.bottom_tab = BottomTab::Log;
                }
                let is_model_active = self.bottom_dock_open && self.bottom_tab == BottomTab::Model;
                if ui
                    .selectable_label(is_model_active, "📋")
                    .on_hover_text("モデル表")
                    .clicked()
                    && toggle_dock_icon(&mut self.bottom_dock_open, is_model_active)
                {
                    self.bottom_tab = BottomTab::Model;
                }
                let is_loads_active = self.bottom_dock_open && self.bottom_tab == BottomTab::Loads;
                if ui
                    .selectable_label(is_loads_active, "⚡")
                    .on_hover_text("荷重表")
                    .clicked()
                    && toggle_dock_icon(&mut self.bottom_dock_open, is_loads_active)
                {
                    self.bottom_tab = BottomTab::Loads;
                }
                let is_prep_active =
                    self.bottom_dock_open && self.bottom_tab == BottomTab::Preparation;
                if ui
                    .selectable_label(is_prep_active, "🛠")
                    .on_hover_text("準備計算の結果")
                    .clicked()
                    && toggle_dock_icon(&mut self.bottom_dock_open, is_prep_active)
                {
                    self.bottom_tab = BottomTab::Preparation;
                }
                let is_diag_active =
                    self.bottom_dock_open && self.bottom_tab == BottomTab::Diagnostics;
                if ui
                    .selectable_label(is_diag_active, "⚠")
                    .on_hover_text("診断")
                    .clicked()
                    && toggle_dock_icon(&mut self.bottom_dock_open, is_diag_active)
                {
                    self.bottom_tab = BottomTab::Diagnostics;
                }
                ui.separator();
                // プロジェクトファイル名 + 未保存マーカー
                let file_label = self
                    .project_path
                    .as_ref()
                    .and_then(|p| p.file_name())
                    .map(|n| n.to_string_lossy().to_string())
                    .unwrap_or_else(|| "(未保存プロジェクト)".to_string());
                let marker = if self.staleness.unsaved_changes {
                    " ●"
                } else {
                    ""
                };
                ui.label(format!("{}{}", file_label, marker));
                ui.separator();
                // バックグラウンド解析ジョブの実行状況
                if let Some(job) = &self.job {
                    let elapsed = job.started.elapsed().unwrap_or_default().as_secs_f64();
                    ui.colored_label(
                        crate::theme::GOOD_GREEN,
                        format!("⏳ {} 実行中… {:.0}s", job.label, elapsed),
                    );
                    ui.separator();
                }
                // stale アイコン
                if self.staleness.results_stale {
                    ui.colored_label(crate::theme::BEST_YELLOW, "⚠ stale");
                } else if self.results.is_some() {
                    ui.colored_label(crate::theme::GOOD_GREEN, "✓ 最新");
                } else {
                    ui.colored_label(crate::theme::GRAY_600, "▷ 未実行");
                }
                if let Some(err) = &self.last_error {
                    ui.separator();
                    // ST-Bridge 取込警告（複数件を \n 区切りで連結）など改行を含む
                    // メッセージは1行に畳んでから truncate する（\n はレイアウト上
                    // 明示的な改行として扱われ、行の高さ・幅の見積りが崩れるため）。
                    // 全文はホバーで表示する。クリックでログパネルを開けるようにする
                    // （エラーの詳細な経緯はログに残っているため）。
                    let one_line = err.replace('\n', " ");
                    let clicked = ui
                        .add(
                            egui::Label::new(
                                egui::RichText::new(format!("⚠ {}", one_line))
                                    .color(crate::theme::ERROR_RED),
                            )
                            .truncate()
                            .sense(egui::Sense::click()),
                        )
                        .on_hover_text(format!("{}\n\nクリックでログを開く", err))
                        .clicked();
                    if clicked {
                        self.bottom_dock_open = true;
                    }
                }
                // last_error（赤・処理を止める）とは別枠の注意事項（例: 精算周期
                // (SemiPrecise)選択時に固有値解析が未実行で EX/EY が未更新である旨）。
                // 情報色（BEST_YELLOW）で表示し、解析自体は継続してよいことを示す。
                if let Some(notice) = &self.last_notice {
                    ui.separator();
                    let one_line = notice.replace('\n', " ");
                    ui.add(
                        egui::Label::new(
                            egui::RichText::new(format!("ℹ {}", one_line))
                                .color(crate::theme::BEST_YELLOW),
                        )
                        .truncate(),
                    )
                    .on_hover_text(notice);
                }
            });
        });

        #[allow(deprecated)]
        ui.allocate_ui_at_rect(right_rect, |ui| {
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // サマリの右に配置（right_to_left のため先に追加する）。左ゾーンと同じ
                // toggle_dock_icon 方式（アクティブなら閉じる／それ以外は開いてそのパネルを
                // アクティブにする）で右ドックのパネルを切り替える。
                let is_analysis_active =
                    self.right_dock_open && self.right_panel == RightPanel::Analysis;
                if ui
                    .selectable_label(is_analysis_active, "⚙")
                    .on_hover_text("② 解析（実行）")
                    .clicked()
                    && toggle_dock_icon(&mut self.right_dock_open, is_analysis_active)
                {
                    self.right_panel = RightPanel::Analysis;
                }
                let is_prep_panel_active =
                    self.right_dock_open && self.right_panel == RightPanel::Preparation;
                if ui
                    .selectable_label(is_prep_panel_active, "🛠")
                    .on_hover_text("① 準備計算（解析条件の入力・実行）")
                    .clicked()
                    && toggle_dock_icon(&mut self.right_dock_open, is_prep_panel_active)
                {
                    self.right_panel = RightPanel::Preparation;
                }
                let is_inspector_active =
                    self.right_dock_open && self.right_panel == RightPanel::Inspector;
                if ui
                    .selectable_label(is_inspector_active, "🔍")
                    .on_hover_text("インスペクタ")
                    .clicked()
                    && toggle_dock_icon(&mut self.right_dock_open, is_inspector_active)
                {
                    self.right_panel = RightPanel::Inspector;
                }
                ui.label(summary);
            });
        });
    }
}
