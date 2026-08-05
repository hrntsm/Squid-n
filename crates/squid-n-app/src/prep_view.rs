//! 準備計算ビュー（下ドック「準備計算」タブ）。
//!
//! [`crate::app::PreparationResult`] を表として表示し、解析前に階の分布・剛域・
//! Ai 分布・風圧力・荷重集計を確認できるようにする。CSV エクスポート
//! （[`crate::summary::build_preparation_csv`]）にも対応する。

use egui_extras::{Column, TableBuilder};

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
    /// 風圧力。
    Wind,
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

/// N → kN。表は kN 表示に統一する（モデル内部の力の単位は N）。
fn kn(n: f64) -> f64 {
    n / 1000.0
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
        if app.staleness.preparation_stale {
            ui.colored_label(
                crate::theme::BEST_YELLOW,
                "⚠ モデルが編集されました。準備計算は再実行が必要です。",
            );
        } else if let Some(elapsed) = app
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
            if app.preparation.is_some() && ui.button("📋 CSV をコピー").clicked() {
                ui.ctx()
                    .copy_text(crate::summary::build_preparation_csv(app));
            }
        });
    });
    ui.separator();

    let Some(prep) = app.preparation.as_ref() else {
        ui.colored_label(
            crate::theme::GRAY_600,
            "準備計算が未実行です。「▶ 準備計算 実行」を押すと、\
             階の分布・剛域・Ai 分布・風圧力・荷重集計を確認できます。",
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
            crate::theme::BEST_YELLOW,
            format!("⚠ 整合性チェック: 警告 {} 件", prep.diag_warnings),
        );
    } else {
        ui.colored_label(crate::theme::GOOD_GREEN, "✅ 整合性チェック: 問題なし");
    }

    let view = &mut app.prep_view.view;
    ui.horizontal(|ui| {
        for (v, label) in [
            (PrepView::Stories, "階の分布"),
            (PrepView::Seismic, "地震力 (Ai 分布)"),
            (PrepView::Wind, "風圧力"),
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

    let view = app.prep_view.view;
    egui::ScrollArea::both()
        .id_salt("prep_view")
        .auto_shrink([false, false])
        .show(ui, |ui| match view {
            PrepView::Stories => stories_section(ui, prep),
            PrepView::Seismic => seismic_section(ui, prep),
            PrepView::Wind => wind_section(ui, prep),
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
            ui.label(format!("{:.2}", s.height_mm / 1000.0));
            ui.label("鉄骨造高さ比 α");
            ui.label(format!("{:.3}", s.steel_height_ratio));
            ui.end_row();
            ui.label("地震用重量 ΣW [kN]");
            ui.label(format!("{:.1}", kn(s.total_seismic_weight)));
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
            "階が定義されていません（地震力・風圧力・増分解析には階の定義が必要です）",
        );
        return;
    }

    // 上階→下階の順で並べる（伏図・軸組図と同じ見え方にする）。
    let rows: Vec<_> = prep.stories.iter().rev().collect();
    crate::table_util::standard_table(
        ui,
        "prep_stories",
        &[
            Column::initial(90.0),
            Column::initial(90.0),
            Column::initial(90.0),
            Column::initial(70.0),
            Column::initial(70.0),
            Column::initial(110.0),
            Column::initial(110.0),
            Column::initial(60.0),
            Column::initial(110.0),
        ],
        &[
            "階",
            "床レベル [mm]",
            "階高 [mm]",
            "節点数",
            "剛床数",
            "地震用重量 Wi [kN]",
            "累積 ΣWj [kN]",
            "構造",
            "種別",
        ],
        rows.len(),
        |row| {
            let r = rows[row.index()];
            row.col(|ui| {
                ui.label(&r.name);
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
                // 剛床がない階には地震力・風荷重を載荷できない。
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
                    ui.label(format!("{:.1}", kn(r.weight)));
                }
            });
            row.col(|ui| {
                ui.label(format!("{:.1}", kn(r.cumulative_weight)));
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
            crate::theme::BEST_YELLOW,
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
            ui.label(format!("{:.1}", kn(sm.base_shear)));
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
            Column::initial(90.0),
            Column::initial(110.0),
            Column::initial(110.0),
            Column::initial(70.0),
            Column::initial(70.0),
            Column::initial(70.0),
            Column::initial(100.0),
            Column::initial(100.0),
            Column::initial(110.0),
        ],
        &[
            "階",
            "Wi [kN]",
            "ΣWj [kN]",
            "αi",
            "Ai",
            "Ci",
            "Qi [kN]",
            "Pi [kN]",
            "種別",
        ],
        rows.len(),
        |row| {
            let r = rows[row.index()];
            // αi・Ai は一般階のみ意味を持つ（PH 階・地下階は別式）。
            let normal = matches!(r.level_kind, squid_n_core::model::StoryLevelKind::Normal);
            row.col(|ui| {
                ui.label(&r.name);
            });
            row.col(|ui| {
                ui.label(format!("{:.1}", kn(r.weight)));
            });
            row.col(|ui| {
                ui.label(format!("{:.1}", kn(r.cumulative_weight)));
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
                ui.label(format!("{:.1}", kn(r.qi)));
            });
            row.col(|ui| {
                ui.label(format!("{:.1}", kn(r.pi)));
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

/// 風圧力（X・Y の両方向）。速度圧など方向によらない諸元は 1 度だけ表示し、
/// 見付面積・層水平力は方向ごとに表を分ける。
fn wind_section(ui: &mut egui::Ui, prep: &PreparationResult) {
    let Some(first) = prep.wind.first() else {
        ui.colored_label(
            crate::theme::BEST_YELLOW,
            prep.wind_note
                .clone()
                .unwrap_or_else(|| "風圧力を算定できませんでした".to_string()),
        );
        return;
    };

    egui::Grid::new("prep_wind_cfg")
        .num_columns(4)
        .spacing([16.0, 2.0])
        .show(ui, |ui| {
            ui.label("建物高さ H [m]");
            ui.label(format!("{:.2}", first.h_mm / 1000.0));
            ui.label("基準風速 V0 [m/s]");
            ui.label(format!("{:.1}", first.v0));
            ui.end_row();
            ui.label("地表面粗度区分");
            ui.label(format!("{:?}", first.roughness));
            ui.label("速度圧 q [N/m²]");
            ui.label(format!("{:.1}", first.q));
            ui.end_row();
            ui.label("Er / Gf / E");
            ui.label(format!(
                "{:.3} / {:.3} / {:.3}",
                first.er, first.gf, first.e
            ));
            ui.label("");
            ui.label("");
            ui.end_row();
        });
    ui.add_space(6.0);

    if let Some(note) = prep.wind_note.as_ref() {
        ui.colored_label(crate::theme::BEST_YELLOW, note);
    }

    for w in &prep.wind {
        ui.strong(format!(
            "{:?} 方向（基部せん断力 {:.1} kN）",
            w.dir,
            kn(w.base_shear)
        ));
        wind_table(ui, w);
        ui.add_space(6.0);
    }
}

/// 風圧力の層別表（1 方向分）。
fn wind_table(ui: &mut egui::Ui, w: &crate::app::PrepWind) {
    let rows: Vec<_> = w.rows.iter().rev().collect();
    crate::table_util::standard_table(
        ui,
        &format!("prep_wind_table_{:?}", w.dir),
        &[
            Column::initial(90.0),
            Column::initial(140.0),
            Column::initial(100.0),
            Column::initial(110.0),
            Column::initial(70.0),
            Column::initial(110.0),
            Column::initial(100.0),
        ],
        &[
            "階",
            "負担高さ [mm]",
            "見付幅 [mm]",
            "見付面積 [m²]",
            "Kz",
            "風圧力 [N/m²]",
            "層水平力 [kN]",
        ],
        rows.len(),
        |row| {
            let r = rows[row.index()];
            row.col(|ui| {
                ui.label(&r.name);
            });
            row.col(|ui| {
                ui.label(format!("{:.0} 〜 {:.0}", r.z_bottom, r.z_top));
            });
            row.col(|ui| {
                ui.label(format!("{:.0}", r.width));
            });
            row.col(|ui| {
                ui.label(format!("{:.2}", r.area * 1e-6));
            });
            row.col(|ui| {
                ui.label(format!("{:.3}", r.kz));
            });
            row.col(|ui| {
                ui.label(format!("{:.1}", r.pressure));
            });
            row.col(|ui| {
                ui.label(format!("{:.1}", kn(r.force)));
            });
        },
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
            Column::initial(70.0),
            Column::initial(70.0),
            Column::initial(80.0),
            Column::remainder(),
        ],
        &["部材", "種別", "節点", "理由"],
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
                ui.label("この節点の材軸まわり回転を拘束する部材・支点がない");
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
            Column::initial(70.0),
            Column::initial(90.0),
            Column::initial(90.0),
            Column::initial(80.0),
            Column::initial(120.0),
            Column::remainder(),
        ],
        &[
            "節点",
            "dc [mm]",
            "db [mm]",
            "tp [mm]",
            "Ve [mm³]",
            "Kxp=Kyp [kN·m/rad]",
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
                // N·mm/rad → kN·m/rad
                ui.label(format!("{:.3e}", r.k_panel / 1.0e6));
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
            Column::initial(70.0),
            Column::initial(70.0),
            Column::initial(100.0),
            Column::initial(90.0),
            Column::initial(110.0),
            Column::initial(110.0),
            Column::initial(110.0),
            Column::initial(110.0),
            Column::initial(110.0),
            Column::initial(80.0),
        ],
        &[
            "部材",
            "種別",
            "節点 i–j",
            "材長 L [mm]",
            "λi [mm]",
            "λj [mm]",
            "パネル i/j [mm]",
            "可とう長 L' [mm]",
            "フェース i/j [mm]",
            "剛域比",
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
    let row_h = crate::theme::table_row_height(ui);
    let rows = &prep.sections;
    TableBuilder::new(ui)
        .striped(true)
        .id_salt("prep_sections")
        .column(Column::initial(50.0))
        .column(Column::initial(180.0))
        .column(Column::initial(50.0))
        .column(Column::initial(100.0))
        .column(Column::initial(60.0))
        .column(Column::initial(110.0))
        .column(Column::initial(100.0))
        .column(Column::initial(100.0))
        .column(Column::initial(100.0))
        .column(Column::initial(100.0))
        .column(Column::initial(110.0))
        .column(Column::initial(110.0))
        .column(Column::initial(140.0))
        .header(row_h, |mut h| {
            for t in &[
                "ID",
                "符号",
                "階",
                "形状",
                "部材数",
                "D×B [mm]",
                "A [cm²]",
                "Iy [cm⁴]",
                "Iz [cm⁴]",
                "J [cm⁴]",
                "Asy/Asz [cm²]",
                "iy/iz [mm]",
                "材料 (E [N/mm²])",
            ] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(row_h, rows.len(), |mut row| {
                let r = &rows[row.index()];
                row.col(|ui| {
                    ui.label(format!("{}", r.section.0));
                });
                row.col(|ui| {
                    ui.label(&r.name);
                });
                row.col(|ui| {
                    // 同じ符号の断面を階で見分けられるようにする（断面の同一性は符号＋階）。
                    match r.floor.as_deref() {
                        Some(f) => ui.label(f),
                        None => ui.colored_label(crate::theme::GRAY_600, "—"),
                    };
                });
                row.col(|ui| {
                    match r.shape_label.as_deref() {
                        Some(l) => ui.label(l),
                        // 形状定義を持たない断面は剛性増大率・幅厚比・終局耐力の
                        // 算定対象外になるため、数値直入力であることを示す。
                        None => ui.colored_label(crate::theme::GRAY_600, "数値直入力"),
                    };
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
                // cm 系へ換算して表示する（mm 系のままでは桁が大きく比較しづらい）。
                row.col(|ui| {
                    ui.label(format!("{:.1}", r.area * 1e-2));
                });
                row.col(|ui| {
                    ui.label(format!("{:.0}", r.iy * 1e-4));
                });
                row.col(|ui| {
                    ui.label(format!("{:.0}", r.iz * 1e-4));
                });
                row.col(|ui| {
                    ui.label(format!("{:.0}", r.j * 1e-4));
                });
                row.col(|ui| {
                    ui.label(format!("{:.1} / {:.1}", r.as_y * 1e-2, r.as_z * 1e-2));
                });
                row.col(|ui| {
                    ui.label(format!("{:.1} / {:.1}", r.ry, r.rz));
                });
                row.col(|ui| {
                    match (&r.material, r.young) {
                        (Some(m), Some(e)) => ui.label(format!("{} ({:.0})", m, e)),
                        (Some(m), None) => ui.label(m.clone()),
                        _ => ui.colored_label(crate::theme::GRAY_600, "未割当"),
                    };
                });
            });
        });
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
    let row_h = crate::theme::table_row_height(ui);
    let rows = &prep.width_thickness;
    TableBuilder::new(ui)
        .striped(true)
        .id_salt("prep_width_thickness")
        .column(Column::initial(180.0))
        .column(Column::initial(60.0))
        .column(Column::initial(110.0))
        .column(Column::initial(70.0))
        .column(Column::initial(110.0))
        .column(Column::initial(80.0))
        .header(row_h, |mut h| {
            for t in &["断面", "用途", "材料", "部材数", "最大幅厚比", "ランク"] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(row_h, rows.len(), |mut row| {
                let r = &rows[row.index()];
                row.col(|ui| {
                    ui.label(&r.section_name);
                });
                row.col(|ui| {
                    ui.label(steel_member_use_label(r.member_use));
                });
                row.col(|ui| {
                    ui.label(&r.material);
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
            });
        });
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

    let row_h = crate::theme::table_row_height(ui);
    let rows = &prep.member_stiffness;
    TableBuilder::new(ui)
        .striped(true)
        .id_salt("prep_member_stiffness")
        .column(Column::initial(70.0))
        .column(Column::initial(70.0))
        .column(Column::initial(160.0))
        .column(Column::initial(100.0))
        .column(Column::initial(100.0))
        .column(Column::initial(100.0))
        .column(Column::initial(120.0))
        .column(Column::initial(120.0))
        .column(Column::initial(100.0))
        .header(row_h, |mut h| {
            for t in &[
                "部材",
                "種別",
                "断面",
                "材料",
                "スラブ",
                "壁上下梁",
                "元 Iy [cm⁴]",
                "実効 Iy [cm⁴]",
                "総増大率",
            ] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(row_h, rows.len(), |mut row| {
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
                    ui.label(&r.material);
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
            });
        });
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
    let row_h = crate::theme::table_row_height(ui);
    let rows = &prep.load_cases;
    TableBuilder::new(ui)
        .striped(true)
        .column(Column::initial(140.0))
        .column(Column::initial(110.0))
        .column(Column::initial(90.0))
        .column(Column::initial(90.0))
        .column(Column::initial(110.0))
        .column(Column::initial(110.0))
        .column(Column::initial(110.0))
        .header(row_h, |mut h| {
            for t in &[
                "荷重ケース",
                "種別",
                "節点荷重数",
                "部材荷重数",
                "ΣFx [kN]",
                "ΣFy [kN]",
                "ΣFz [kN]",
            ] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(row_h, rows.len(), |mut row| {
                let r = &rows[row.index()];
                let empty = r.n_nodal == 0 && r.n_member == 0;
                row.col(|ui| {
                    ui.label(&r.name);
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
                        let text = format!("{:.1}", kn(r.sum_force[k]));
                        if empty {
                            ui.colored_label(crate::theme::GRAY_600, text);
                        } else {
                            ui.label(text);
                        }
                    });
                }
            });
        });
    ui.add_space(4.0);
    ui.colored_label(
        crate::theme::GRAY_600,
        "ΣF は節点荷重と部材荷重（分布荷重は合力）を全体座標系で積算した外力の総和です。\
         鉛直下向きが負のため、重力系のケースでは ΣFz が負値になります。",
    );
}
