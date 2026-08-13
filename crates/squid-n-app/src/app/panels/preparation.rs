//! 右ドック ①準備計算。
//!
//! `panels` からの構造分割。アルゴリズム変更は行わない。

use super::*;
use crate::table_util::Col;
use squid_n_core::units::to_display::force_kn;
use squid_n_core::units::to_internal;

impl App {
    /// 右ドック「① 準備計算」パネル：解析条件の入力・階の定義と、準備計算の実行。
    ///
    /// 一貫計算の手順（① 解析入力を確定 → ② 解く）のうち ① を担う。実行すると
    /// 階の定義・剛域・荷重ケース（DL/LL/EX/EY/WX/WY）が確定する。② は
    /// [`App::static_panel`] ほか各解析パネル。
    pub(crate) fn preparation_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("① 準備計算");
        ui.separator();

        // バックグラウンドジョブ実行中は実行ボタンを無効化する（P8 §5）。
        let running = self.job.is_some();
        self.preparation_section(ui, running);
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
                crate::theme::WARN_TEXT,
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
                                .unwrap_or(force_kn(weight.unwrap_or(0.0)));
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
                                    let new_weight = to_internal::force_kn(w);
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
}
