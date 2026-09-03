//! 準備計算ビュー（下ドック「準備計算」タブ）。
//!
//! [`crate::app::PreparationResult`] を表として表示し、解析前に階の分布・剛域・
//! Ai 分布・荷重集計を確認できるようにする。CSV エクスポート
//! （[`crate::summary::build_preparation_csv`]）にも対応する。

use crate::table_util::Col;

use squid_n_core::units::to_display::{area_cm2, force_kn, inertia_cm4, length_m, moment_kn_m};

use crate::app::{
    ai_mode_label, load_case_kind_label, member_kind_label, member_rank_label, soil_class_label,
    steel_member_use_label, story_level_kind_label, story_structure_label, zone_source_label, App,
    PreparationResult, RIGID_ZONE_RATIO_WARN,
};

/// 準備計算ビュー内の表示切替。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum PrepView {
    /// 建物概要と階の分布。
    #[default]
    Stories,
    /// 地震力（Ai 分布）。
    Seismic,
    /// 剛域。
    RigidZone,
    /// ねじり解放（i 端ねじれピン）の対象外部材。
    Torsion,
    /// 仕口パネル（柱梁接合部パネル）。
    PanelZone,
    /// 断面性能（断面諸量）。
    Sections,
    /// 鋼断面の幅厚比・部材ランク。
    WidthThickness,
    /// 部材単位の剛性割増し・SRC/CFT 等価断面。
    MemberStiffness,
    /// 荷重ケースの集計。
    Loads,
}

/// 準備計算ビューの状態。
#[derive(Clone, Copy, Debug, Default)]
pub struct PrepViewState {
    pub view: PrepView,
}

/// 準備計算パネルの描画（下ドック）。
pub fn preparation_panel(ui: &mut egui::Ui, app: &mut App) {
    ui.horizontal(|ui| {
        if ui
            .button("▶ 準備計算 実行")
            .on_hover_text(
                "剛域の算定・床荷重/自重/積載の集計・地震力(Ai分布)の算定・\
                 モデル整合性チェックをまとめて実行します（階が未定義なら自動生成します）",
            )
            .clicked()
        {
            app.run_preparation();
        }
        if app.core.scoped.staleness.preparation_stale {
            ui.colored_label(
                crate::theme::WARN_TEXT,
                "⚠ モデルが編集されました。準備計算は再実行が必要です。",
            );
        } else if let Some(elapsed) = app
            .core
            .scoped
            .preparation
            .as_ref()
            .and_then(|p| p.computed_at.elapsed().ok())
        {
            ui.colored_label(
                crate::theme::GRAY_600,
                format!("最終実行: {:.0} 秒前", elapsed.as_secs_f64()),
            );
        }
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if app.core.scoped.preparation.is_some() && ui.button("📋 CSV をコピー").clicked()
            {
                ui.ctx()
                    .copy_text(crate::summary::build_preparation_csv(app));
            }
        });
    });
    ui.separator();

    let Some(prep) = app.core.scoped.preparation.as_ref() else {
        ui.colored_label(
            crate::theme::GRAY_600,
            "準備計算が未実行です。「▶ 準備計算 実行」を押すと、\
             階の分布・剛域・Ai 分布・荷重集計を確認できます。",
        );
        return;
    };

    // 整合性チェックの要約（エラーがあれば解析前に解消する必要がある）。
    if !prep.is_ready() {
        ui.colored_label(
            crate::theme::ERROR_RED,
            format!(
                "⛔ 整合性チェック: エラー {} 件・警告 {} 件（下ドック「診断」タブで内容を確認できます）",
                prep.diag_errors, prep.diag_warnings
            ),
        );
    } else if prep.diag_warnings > 0 {
        ui.colored_label(
            crate::theme::WARN_TEXT,
            format!("⚠ 整合性チェック: 警告 {} 件", prep.diag_warnings),
        );
    } else {
        ui.colored_label(crate::theme::GOOD_GREEN, "✅ 整合性チェック: 問題なし");
    }

    let view = &mut app.ui.view.prep_view.view;
    ui.horizontal(|ui| {
        for (v, label) in [
            (PrepView::Stories, "階の分布"),
            (PrepView::Seismic, "地震力 (Ai 分布)"),
            (PrepView::RigidZone, "剛域"),
            (PrepView::Torsion, "ねじり解放"),
            (PrepView::PanelZone, "仕口パネル"),
            (PrepView::Sections, "断面性能"),
            (PrepView::WidthThickness, "幅厚比"),
            (PrepView::MemberStiffness, "部材剛性"),
            (PrepView::Loads, "荷重集計"),
        ] {
            if ui.selectable_label(*view == v, label).clicked() {
                *view = v;
            }
        }
    });
    ui.separator();

    let view = app.ui.view.prep_view.view;
    // 横スクロールは表ごとに `table_util::standard_table` が持つため、ここは縦のみ。
    // 外側にも横スクロールを置くと、表の横スクロールと二重になって操作が定まらない。
    egui::ScrollArea::vertical()
        .id_salt("prep_view")
        .auto_shrink([false, false])
        .show(ui, |ui| match view {
            PrepView::Stories => stories_section(ui, prep),
            PrepView::Seismic => seismic_section(ui, prep),
            PrepView::RigidZone => rigid_zone_section(ui, prep),
            PrepView::Torsion => torsion_section(ui, prep),
            PrepView::PanelZone => panel_zone_section(ui, prep),
            PrepView::Sections => sections_section(ui, prep),
            PrepView::WidthThickness => width_thickness_section(ui, prep),
            PrepView::MemberStiffness => member_stiffness_section(ui, prep),
            PrepView::Loads => loads_section(ui, prep),
        });
}

/// 建物概要と階の分布。
fn stories_section(ui: &mut egui::Ui, prep: &PreparationResult) {
    let s = &prep.summary;
    egui::Grid::new("prep_summary")
        .num_columns(4)
        .spacing([16.0, 2.0])
        .show(ui, |ui| {
            ui.label("節点／部材");
            ui.label(format!("{} / {}", s.n_nodes, s.n_elements));
            ui.label("支点");
            ui.label(format!("{}", s.n_supports));
            ui.end_row();
            ui.label("階数／剛床数");
            ui.label(format!("{} / {}", s.n_stories, s.n_diaphragms));
            ui.label("地盤面 GL [mm]");
            ui.label(format!("{:.0}", s.ground_elevation));
            ui.end_row();
            ui.label("建物高さ h [m]");
            ui.label(format!("{:.2}", length_m(s.height_mm)));
            ui.label("鉄骨造高さ比 α");
            ui.label(format!("{:.3}", s.steel_height_ratio));
            ui.end_row();
            ui.label("地震用重量 ΣW [kN]");
            ui.label(format!("{:.1}", force_kn(s.total_seismic_weight)));
            ui.label("質量モデル");
            ui.label(match s.mass_method {
                squid_n_core::model::MassMethod::CorrectedLumped => "補正質点",
                squid_n_core::model::MassMethod::LumpedOnly => "質点のみ",
            });
            ui.end_row();
        });
    ui.add_space(6.0);

    if prep.stories.is_empty() {
        ui.colored_label(
            crate::theme::GRAY_600,
            "階が定義されていません（地震力・増分解析には階の定義が必要です）",
        );
        return;
    }

    // 上階→下階の順で並べる（伏図・軸組図と同じ見え方にする）。
    let rows: Vec<_> = prep.stories.iter().rev().collect();
    crate::table_util::standard_table(
        ui,
        "prep_stories",
        &[
            Col::label("階"),
            Col::num("床レベル [mm]"),
            Col::num("階高 [mm]"),
            Col::num("節点数"),
            Col::num("剛床数"),
            Col::num("地震用重量 Wi [kN]"),
            Col::num("累積 ΣWj [kN]"),
            Col::label("構造"),
            Col::label("種別"),
        ],
        rows.len(),
        |row| {
            let r = rows[row.index()];
            row.col(|ui| {
                crate::table_util::text_cell(ui, &r.name);
            });
            row.col(|ui| {
                ui.label(format!("{:.0}", r.elevation));
            });
            row.col(|ui| {
                ui.label(format!("{:.0}", r.height));
            });
            row.col(|ui| {
                ui.label(format!("{}", r.n_nodes));
            });
            row.col(|ui| {
                // 剛床がない階の水平力は、その階の節点へ質量比で直接分配される。
                // 解析は通るため、意図した入力かどうかを確かめられるよう強調する
                // （診断タブにも警告として出る）。
                if r.n_diaphragms == 0 {
                    ui.colored_label(crate::theme::BEST_YELLOW, "0");
                } else {
                    ui.label(format!("{}", r.n_diaphragms));
                }
            });
            row.col(|ui| {
                if r.weight <= 0.0 {
                    ui.colored_label(crate::theme::BEST_YELLOW, "0.0");
                } else {
                    ui.label(format!("{:.1}", force_kn(r.weight)));
                }
            });
            row.col(|ui| {
                ui.label(format!("{:.1}", force_kn(r.cumulative_weight)));
            });
            row.col(|ui| {
                ui.label(story_structure_label(r.structure));
            });
            row.col(|ui| {
                ui.label(story_level_kind_label(r.level_kind));
            });
        },
    );
}

/// 地震力（Ai 分布）。
fn seismic_section(ui: &mut egui::Ui, prep: &PreparationResult) {
    let Some(sm) = prep.seismic.as_ref() else {
        ui.colored_label(
            crate::theme::WARN_TEXT,
            prep.seismic_note
                .clone()
                .unwrap_or_else(|| "地震力(Ai分布)を算定できませんでした".to_string()),
        );
        return;
    };

    egui::Grid::new("prep_seismic_cfg")
        .num_columns(4)
        .spacing([16.0, 2.0])
        .show(ui, |ui| {
            ui.label("設計用固有周期 T [s]");
            ui.label(format!("{:.3}", sm.t));
            ui.label("T の算定法");
            ui.label(ai_mode_label(sm.t_mode));
            ui.end_row();
            ui.label("地盤種別 / Tc [s]");
            ui.label(format!("{} / {:.1}", soil_class_label(sm.soil), sm.tc));
            ui.label("振動特性係数 Rt");
            ui.label(format!("{:.3}", sm.rt));
            ui.end_row();
            ui.label("地域係数 Z");
            ui.label(format!("{:.2}", sm.z));
            ui.label("標準せん断力係数 C0");
            ui.label(format!("{:.2}", sm.c0));
            ui.end_row();
            ui.label("基部せん断力 Q1 [kN]");
            ui.label(format!("{:.1}", force_kn(sm.base_shear)));
            ui.label("ベースシア係数 Q1/ΣW");
            let total = prep.summary.total_seismic_weight;
            ui.label(if total > 0.0 {
                format!("{:.4}", sm.base_shear / total)
            } else {
                "—".to_string()
            });
            ui.end_row();
        });
    if sm.clamped_negative_pi {
        ui.colored_label(
            crate::theme::ERROR_RED,
            "⚠ 層の水平外力 Pi に負値が現れ 0 へクランプしました。\
             階の地震用重量 Wi の並び（上階ほど軽くなっているか）を確認してください。",
        );
    }
    ui.add_space(6.0);

    let rows: Vec<_> = sm.rows.iter().rev().collect();
    crate::table_util::standard_table(
        ui,
        "prep_seismic",
        &[
            Col::label("階"),
            Col::num("Wi [kN]"),
            Col::num("ΣWj [kN]"),
            Col::num("αi"),
            Col::num("Ai"),
            Col::num("Ci"),
            Col::num("Qi [kN]"),
            Col::num("Pi [kN]"),
            Col::label("種別"),
        ],
        rows.len(),
        |row| {
            let r = rows[row.index()];
            // αi・Ai は一般階のみ意味を持つ（PH 階・地下階は別式）。
            let normal = matches!(r.level_kind, squid_n_core::model::StoryLevelKind::Normal);
            row.col(|ui| {
                crate::table_util::text_cell(ui, &r.name);
            });
            row.col(|ui| {
                ui.label(format!("{:.1}", force_kn(r.weight)));
            });
            row.col(|ui| {
                ui.label(format!("{:.1}", force_kn(r.cumulative_weight)));
            });
            row.col(|ui| {
                ui.label(if normal {
                    format!("{:.3}", r.alpha)
                } else {
                    "—".to_string()
                });
            });
            row.col(|ui| {
                ui.label(if normal {
                    format!("{:.3}", r.ai)
                } else {
                    "—".to_string()
                });
            });
            row.col(|ui| {
                ui.label(format!("{:.4}", r.ci));
            });
            row.col(|ui| {
                ui.label(format!("{:.1}", force_kn(r.qi)));
            });
            row.col(|ui| {
                ui.label(format!("{:.1}", force_kn(r.pi)));
            });
            row.col(|ui| {
                ui.label(story_level_kind_label(r.level_kind));
            });
        },
    );
    ui.add_space(4.0);
    ui.colored_label(
        crate::theme::GRAY_600,
        "Ci は一般階が層せん断力係数、PH 階は震度 k、地下階は水平震度 K を表します。\
         αi・Ai は一般階のみ算定します（令88条・昭55建告1793号）。",
    );
}

/// ねじり解放（i 端ねじれピン）の対象外部材。
///
/// 既定では線材（梁・柱）の i 端ねじれを解放し、部材全長で Mx=0 とする。
/// ただし解放すると材軸まわりの回転を拘束するものがない節点が生じる部材は、
/// 剛性行列が特異になるため自動的に対象外とし、ねじり剛性 GJ/L を保持する。
/// この表は「想定と違ってねじり剛性が残っている部材」を見つけるためのもので、
/// ねじり剛性をもともと持たない部材（断面の J≤0・材料の G≤0）は含めない。
fn torsion_section(ui: &mut egui::Ui, prep: &PreparationResult) {
    if !prep.torsion_release_enabled {
        ui.colored_label(
            crate::theme::GRAY_600,
            "「部材 i 端のねじりをピン（梁・柱）」が OFF のため、全部材でねじり剛性 GJ/L を             保持しています（準備計算パネルの「部材のモデル化」で切り替えます）。",
        );
        return;
    }
    ui.label(format!(
        "ねじり解放の対象外部材: {} 本",
        prep.torsion_skipped.len()
    ));
    ui.colored_label(
        crate::theme::GRAY_600,
        "既定では線材（梁・柱）の i 端ねじれをピンとし、部材全長で Mx=0 とします。         ただし、ねじりを解放すると材軸まわりの回転を拘束するものがなくなる節点を持つ部材は、         剛性行列が特異になるため自動的に対象外とし、ねじり剛性 GJ/L を保持します。         材軸まわりの回転は「非平行な線材の曲げ」「線材以外の要素（壁・シェル・ばね類）」         「支点拘束」「支点ばねの回転成分」のいずれかで拘束されている必要があります。         仕口パネルは接合部のせん断変形角にのみ剛性を与え、節点の回転自由度には         寄与しないため、この判定には含めません。",
    );
    ui.add_space(6.0);

    if prep.torsion_skipped.is_empty() {
        ui.colored_label(
            crate::theme::GOOD_GREEN,
            "✅ 対象外の部材はありません（ねじり剛性を持つ全ての線材で i 端ねじれを解放しています）",
        );
        return;
    }

    let rows = &prep.torsion_skipped;
    crate::table_util::standard_table(
        ui,
        "prep_torsion",
        &[
            Col::id_named("部材"),
            Col::label("種別"),
            Col::num("節点"),
            Col::text("理由"),
        ],
        rows.len(),
        |row| {
            let r = &rows[row.index()];
            row.col(|ui| {
                ui.label(format!("#{}", r.elem.0));
            });
            row.col(|ui| {
                ui.label(member_kind_label(r.kind));
            });
            row.col(|ui| {
                ui.label(format!("{}", r.node.0));
            });
            row.col(|ui| {
                crate::table_util::text_cell(
                    ui,
                    "この節点の材軸まわり回転を拘束する部材・支点がない",
                );
            });
        },
    );
}

/// 仕口パネル（柱梁接合部パネル）。
///
/// S 造（CFT を除く）の柱梁接合節点に生成したパネルの寸法とせん断剛性を一覧する。
/// パネルを設けた節点はせん断変形角 γX・γY の 2 自由度を追加で持ち、取り付く部材は
/// パネル寸法分だけ離れた位置（柱フェース・梁フェース）で接合する。
fn panel_zone_section(ui: &mut egui::Ui, prep: &PreparationResult) {
    if !prep.panel_modeling_enabled {
        ui.colored_label(
            crate::theme::GRAY_600,
            "「仕口パネルをモデル化（柱梁接合部）」が OFF のため、接合部を剛節点として\
             扱っています（準備計算パネルの「部材のモデル化」で切り替えます）。\
             柱梁接合部の断面算定は、この設定によらず常に行います。",
        );
        return;
    }
    ui.label(format!("仕口パネル: {} 箇所", prep.panels.len()));
    ui.colored_label(
        crate::theme::GRAY_600,
        "せん断剛性は Kxp = Kyp = G・Ve です。実効体積 Ve は H 形柱で dc・db・tp、\
         角形・円形鋼管柱で 2・dc・db・tp とし、断面検定の降伏モーメント \
         pMy = (Ve/κ)・√(1−n²)・Fy/√3 と同じ体積を用います。板厚 tp は柱断面形状から\
         算出し、断面に「パネル板厚」が入力されていればそちらを優先します。\
         RC・SRC・CFT の接合部はモデル化の対象外で、従来どおり剛域で有限寸法を評価します\
         （CFT の断面算定は従来どおり行います）。",
    );
    ui.add_space(6.0);

    if prep.panels.is_empty() {
        ui.colored_label(
            crate::theme::GRAY_600,
            "生成されたパネルはありません（対象となる S 造の柱梁接合部がない、または\
             柱・梁の断面が未割当です）。",
        );
        return;
    }

    let rows = &prep.panels;
    crate::table_util::standard_table(
        ui,
        "prep_panels",
        &[
            Col::id_named("節点"),
            Col::num("dc [mm]"),
            Col::num("db [mm]"),
            Col::num("tp [mm]"),
            Col::num("Ve [mm³]"),
            Col::num("Kxp=Kyp [kN·m/rad]"),
        ],
        rows.len(),
        |row| {
            let r = &rows[row.index()];
            row.col(|ui| {
                ui.label(format!("{}", r.node.0));
            });
            row.col(|ui| {
                ui.label(format!("{:.1}", r.dc));
            });
            row.col(|ui| {
                ui.label(format!("{:.1}", r.db));
            });
            row.col(|ui| {
                ui.label(format!("{:.1}", r.tp));
            });
            row.col(|ui| {
                ui.label(format!("{:.3e}", r.ve));
            });
            row.col(|ui| {
                // N·mm/rad → kN·m/rad（回転剛性。換算係数はモーメント表示と同じ）
                ui.label(format!("{:.3e}", moment_kn_m(r.k_panel)));
            });
        },
    );
}

/// 剛域。
fn rigid_zone_section(ui: &mut egui::Ui, prep: &PreparationResult) {
    ui.horizontal_wrapped(|ui| {
        ui.label(format!(
            "剛域・危険断面位置を持つ部材: {} / 梁要素 {}",
            prep.rigid_zones.len(),
            prep.rigid_zone_candidates
        ));
        ui.separator();
        ui.colored_label(
            crate::theme::GRAY_600,
            "3D の形状はモデル化ビュー（結果タブ「モデル化」）でも確認できます",
        );
    });
    ui.colored_label(
        crate::theme::GRAY_600,
        "剛域長 λ は剛性計算に、フェース距離は断面算定の危険断面位置に用います\
         （S 造の仕口では λ = 0 でもフェース距離は付きます）。",
    );
    ui.add_space(6.0);

    if prep.rigid_zones.is_empty() {
        ui.colored_label(
            crate::theme::GRAY_600,
            "剛域・危険断面位置を持つ部材がありません（節点に直交部材が\
             接続していない場合、剛域長 λ もフェース距離も 0 になります）",
        );
        return;
    }

    let rows = &prep.rigid_zones;
    crate::table_util::standard_table(
        ui,
        "prep_rigid_zones",
        &[
            Col::id_named("部材"),
            Col::label("種別"),
            Col::wide_num("節点 i–j"),
            Col::num("材長 L [mm]"),
            Col::wide_num("λi [mm]"),
            Col::wide_num("λj [mm]"),
            Col::wide_num("パネル i/j [mm]"),
            Col::num("可とう長 L' [mm]"),
            Col::wide_num("フェース i/j [mm]"),
            Col::num("剛域比"),
        ],
        rows.len(),
        |row| {
            let r = &rows[row.index()];
            row.col(|ui| {
                ui.label(format!("#{}", r.elem.0));
            });
            row.col(|ui| {
                ui.label(member_kind_label(r.kind));
            });
            row.col(|ui| {
                ui.label(format!("{}–{}", r.node_i.0, r.node_j.0));
            });
            row.col(|ui| {
                ui.label(format!("{:.0}", r.length));
            });
            row.col(|ui| {
                ui.label(format!(
                    "{:.0} ({})",
                    r.zone_i,
                    zone_source_label(r.source_i)
                ));
            });
            row.col(|ui| {
                ui.label(format!(
                    "{:.0} ({})",
                    r.zone_j,
                    zone_source_label(r.source_j)
                ));
            });
            row.col(|ui| {
                // 仕口パネル分のオフセット。剛域長とは別の量で、剛体アーム長は
                // 両者の大きい方になる。
                ui.label(format!("{:.0} / {:.0}", r.panel_offset_i, r.panel_offset_j));
            });
            row.col(|ui| {
                // 可とう長が 0 以下だと剛性・応力が算定できない（入力異常）。
                if r.clear_length <= 0.0 {
                    ui.colored_label(crate::theme::ERROR_RED, format!("{:.0}", r.clear_length));
                } else {
                    ui.label(format!("{:.0}", r.clear_length));
                }
            });
            row.col(|ui| {
                ui.label(format!("{:.0} / {:.0}", r.face_i, r.face_j));
            });
            row.col(|ui| {
                if r.ratio > RIGID_ZONE_RATIO_WARN {
                    ui.colored_label(crate::theme::BEST_YELLOW, format!("{:.3}", r.ratio));
                } else {
                    ui.label(format!("{:.3}", r.ratio));
                }
            });
        },
    );
}

/// 断面性能（断面諸量）。
fn sections_section(ui: &mut egui::Ui, prep: &PreparationResult) {
    if prep.sections.is_empty() {
        ui.colored_label(crate::theme::GRAY_600, "断面が定義されていません");
        return;
    }
    let rows = &prep.sections;
    crate::table_util::standard_table(
        ui,
        "prep_sections",
        &[
            Col::id(),
            Col::name("符号"),
            Col::label("階"),
            Col::text("形状"),
            Col::num("部材数"),
            Col::wide_num("D×B [mm]"),
            Col::num("A [cm²]"),
            Col::num("Iy [cm⁴]"),
            Col::num("Iz [cm⁴]"),
            Col::num("J [cm⁴]"),
            Col::wide_num("Asy/Asz [cm²]"),
            Col::wide_num("iy/iz [mm]"),
            Col::text("材料 (E [N/mm²])"),
        ],
        rows.len(),
        |row| {
            let r = &rows[row.index()];
            row.col(|ui| {
                ui.label(format!("{}", r.section.0));
            });
            row.col(|ui| {
                crate::table_util::text_cell(ui, &r.name);
            });
            row.col(|ui| {
                // 同じ符号の断面を階で見分けられるようにする（断面の同一性は符号＋階）。
                match r.floor.as_deref() {
                    Some(f) => crate::table_util::text_cell(ui, f),
                    None => crate::table_util::muted_cell(ui, "—", "階が設定されていません"),
                }
            });
            row.col(|ui| {
                match r.shape_label.as_deref() {
                    Some(l) => crate::table_util::text_cell(ui, l),
                    // 形状定義を持たない断面は剛性増大率・幅厚比・終局耐力の
                    // 算定対象外になるため、数値直入力であることを示す。
                    None => crate::table_util::muted_cell(
                        ui,
                        "数値直入力",
                        "形状定義がありません（断面性能の数値直入力）",
                    ),
                }
            });
            row.col(|ui| {
                // どの部材にも使われていない断面は入力漏れ・不要断面の目印。
                if r.n_elements == 0 {
                    ui.colored_label(crate::theme::GRAY_600, "0");
                } else {
                    ui.label(format!("{}", r.n_elements));
                }
            });
            row.col(|ui| {
                ui.label(format!("{:.0} × {:.0}", r.depth, r.width));
            });
            // 断面性能は cm 系で表示する（慣例の情報源は `squid_n_core::units`）。
            row.col(|ui| {
                ui.label(crate::table_util::fmt_section_prop(area_cm2(r.area)));
            });
            row.col(|ui| {
                ui.label(crate::table_util::fmt_section_prop(inertia_cm4(r.iy)));
            });
            row.col(|ui| {
                ui.label(crate::table_util::fmt_section_prop(inertia_cm4(r.iz)));
            });
            row.col(|ui| {
                ui.label(crate::table_util::fmt_section_prop(inertia_cm4(r.j)));
            });
            row.col(|ui| {
                ui.label(format!(
                    "{} / {}",
                    crate::table_util::fmt_section_prop(area_cm2(r.as_y)),
                    crate::table_util::fmt_section_prop(area_cm2(r.as_z))
                ));
            });
            row.col(|ui| {
                ui.label(format!("{:.1} / {:.1}", r.ry, r.rz));
            });
            row.col(|ui| match (&r.material, r.young) {
                (Some(m), Some(e)) => {
                    crate::table_util::text_cell(ui, &format!("{} ({:.0})", m, e))
                }
                (Some(m), None) => crate::table_util::text_cell(ui, m),
                _ => crate::table_util::muted_cell(ui, "未割当", "材料が割り当てられていません"),
            });
        },
    );
    ui.add_space(4.0);
    ui.colored_label(
        crate::theme::GRAY_600,
        "A・I・J・As は弾性解析に用いる断面諸量です。断面二次半径 i = √(I/A) は\
         座屈長さ比・細長比の確認用に併記しています。",
    );
}

/// 鋼断面の幅厚比・部材ランク。
fn width_thickness_section(ui: &mut egui::Ui, prep: &PreparationResult) {
    if prep.width_thickness.is_empty() {
        ui.colored_label(
            crate::theme::GRAY_600,
            "対象となる鋼部材がありません（形状定義を持つ断面が割り当てられた\
             鋼材の部材が対象です）",
        );
        return;
    }
    let rows = &prep.width_thickness;
    crate::table_util::standard_table(
        ui,
        "prep_width_thickness",
        &[
            Col::text("断面"),
            Col::label("用途"),
            Col::name("材料"),
            Col::num("部材数"),
            Col::num("最大幅厚比"),
            Col::label("ランク"),
        ],
        rows.len(),
        |row| {
            let r = &rows[row.index()];
            row.col(|ui| {
                crate::table_util::text_cell(ui, &r.section_name);
            });
            row.col(|ui| {
                ui.label(steel_member_use_label(r.member_use));
            });
            row.col(|ui| {
                crate::table_util::text_cell(ui, &r.material);
            });
            row.col(|ui| {
                ui.label(format!("{}", r.n_elements));
            });
            row.col(|ui| {
                match r.max_ratio {
                    Some(v) => ui.label(format!("{:.1}", v)),
                    None => ui.colored_label(crate::theme::GRAY_600, "—"),
                };
            });
            row.col(|ui| {
                use squid_n_design_jp::secondary::holding_capacity::MemberRank;
                match r.rank {
                    // FD は Ds を最も不利にする（幅厚比の入力確認を促す）。
                    Some(rank @ MemberRank::FD) => {
                        ui.colored_label(crate::theme::ERROR_RED, member_rank_label(rank))
                    }
                    Some(rank @ MemberRank::FC) => {
                        ui.colored_label(crate::theme::BEST_YELLOW, member_rank_label(rank))
                    }
                    Some(rank) => ui.label(member_rank_label(rank)),
                    None => ui.colored_label(crate::theme::GRAY_600, "判定不可"),
                };
            });
        },
    );
    ui.add_space(4.0);
    ui.colored_label(
        crate::theme::GRAY_600,
        "断面・用途（柱／梁）・鋼種が同じ部材は 1 行にまとめています。ランクは\
         保有水平耐力の Ds 算定に用いるものと同じ判定です（円形鋼管は径厚比の\
         体系が異なるため「判定不可」）。筋かいは有効細長比で種別（BA〜BC）を\
         定めるため本表の対象外です。",
    );
}

/// 部材単位の剛性割増し・SRC/CFT 等価断面。
fn member_stiffness_section(ui: &mut egui::Ui, prep: &PreparationResult) {
    ui.label(format!(
        "剛性の割増し・等価換算がある部材: {} / 梁要素 {}",
        prep.member_stiffness.len(),
        prep.member_stiffness_candidates
    ));
    ui.add_space(6.0);

    if prep.member_stiffness.is_empty() {
        ui.colored_label(
            crate::theme::GRAY_600,
            "該当する部材がありません（スラブ協力幅は「剛性計算用のスラブ厚」が\
             0 のとき無効、壁エレメント上下大梁は耐震壁が成立した壁がある場合、\
             等価換算は SRC/CFT 断面がある場合に生じます）",
        );
        return;
    }

    let rows = &prep.member_stiffness;
    crate::table_util::standard_table(
        ui,
        "prep_member_stiffness",
        &[
            Col::id_named("部材"),
            Col::label("種別"),
            Col::text("断面"),
            Col::name("材料"),
            Col::num("スラブ"),
            Col::num("壁上下梁"),
            Col::num("元 Iy [cm⁴]"),
            Col::num("実効 Iy [cm⁴]"),
            Col::num("総増大率"),
        ],
        rows.len(),
        |row| {
            let r = &rows[row.index()];
            row.col(|ui| {
                ui.label(format!("#{}", r.elem.0));
            });
            row.col(|ui| {
                ui.label(member_kind_label(r.kind));
            });
            row.col(|ui| {
                // SRC/CFT は等価換算後の値を使うことが分かるよう印を付ける。
                let text = if r.composite.is_some() {
                    format!("{}（等価換算）", r.section_name)
                } else {
                    r.section_name.clone()
                };
                ui.label(text).on_hover_text(match &r.composite {
                    Some(c) => format!(
                        "SRC/CFT 等価断面: A={:.1} cm², Iy={:.0} cm⁴, Iz={:.0} cm⁴,\n\
                             J={:.0} cm⁴, Asy={:.1} cm², Asz={:.1} cm²",
                        c.area_ax * 1e-2,
                        c.iy * 1e-4,
                        c.iz * 1e-4,
                        c.j * 1e-4,
                        c.as_y * 1e-2,
                        c.as_z * 1e-2
                    ),
                    None => "等価換算なし".to_string(),
                });
            });
            row.col(|ui| {
                crate::table_util::text_cell(ui, &r.material);
            });
            row.col(|ui| {
                label_factor(ui, r.slab_factor);
            });
            row.col(|ui| {
                label_factor(ui, r.wall_girder_factor);
            });
            row.col(|ui| {
                ui.label(format!("{:.0}", r.section_iy * 1e-4));
            });
            row.col(|ui| {
                ui.label(format!("{:.0}", r.effective_iy * 1e-4));
            });
            row.col(|ui| {
                if r.section_iy > 0.0 {
                    ui.label(format!("{:.2} 倍", r.effective_iy / r.section_iy));
                } else {
                    ui.colored_label(crate::theme::GRAY_600, "—");
                }
            });
        },
    );
    ui.add_space(4.0);
    ui.colored_label(
        crate::theme::GRAY_600,
        "「スラブ」は RC 矩形梁のスラブ協力幅（T 形断面）・H 形鋼梁の合成梁による\
         強軸曲げの増大率、「壁上下梁」は壁エレメント上下大梁の一律倍率です。\
         「実効 Iy」は等価換算とこれらをすべて適用した強軸曲げ剛性用の値で、\
         フレーム内雑壁（腰壁・垂壁・袖壁）の算入分は含みません。",
    );
}

/// 増大率のセル。1.0（割増しなし）は淡色、大きい倍率は強調する。
fn label_factor(ui: &mut egui::Ui, factor: f64) {
    if factor <= 1.0 {
        ui.colored_label(crate::theme::GRAY_600, "1.00");
    } else if factor >= 10.0 {
        ui.colored_label(crate::theme::BEST_YELLOW, format!("{:.0} 倍", factor));
    } else {
        ui.label(format!("{:.3}", factor));
    }
}

/// 荷重ケースの集計。
fn loads_section(ui: &mut egui::Ui, prep: &PreparationResult) {
    if prep.load_cases.is_empty() {
        ui.colored_label(crate::theme::GRAY_600, "荷重ケースがありません");
        return;
    }
    let rows = &prep.load_cases;
    crate::table_util::standard_table(
        ui,
        "prep_load_cases",
        &[
            Col::name("荷重ケース"),
            Col::label("種別"),
            Col::num("節点荷重数"),
            Col::num("部材荷重数"),
            Col::num("ΣFx [kN]"),
            Col::num("ΣFy [kN]"),
            Col::num("ΣFz [kN]"),
        ],
        rows.len(),
        |row| {
            let r = &rows[row.index()];
            let empty = r.n_nodal == 0 && r.n_member == 0;
            row.col(|ui| {
                crate::table_util::text_cell(ui, &r.name);
            });
            row.col(|ui| {
                ui.label(load_case_kind_label(r.kind));
            });
            row.col(|ui| {
                ui.label(format!("{}", r.n_nodal));
            });
            row.col(|ui| {
                ui.label(format!("{}", r.n_member));
            });
            for k in 0..3 {
                row.col(|ui| {
                    let text = format!("{:.1}", force_kn(r.sum_force[k]));
                    if empty {
                        ui.colored_label(crate::theme::GRAY_600, text);
                    } else {
                        ui.label(text);
                    }
                });
            }
        },
    );
    ui.add_space(4.0);
    ui.colored_label(
        crate::theme::GRAY_600,
        "ΣF は節点荷重と部材荷重（分布荷重は合力）を全体座標系で積算した外力の総和です。\
         鉛直下向きが負のため、重力系のケースでは ΣFz が負値になります。",
    );
}
