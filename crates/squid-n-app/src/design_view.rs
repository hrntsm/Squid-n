use crate::app::App;
use squid_n_core::units::to_display::{
    area_load_kn_per_m2, force_kn, moment_kn_m, moment_kn_m_per_m, stiffness_kn_per_mm,
};

/// 柱の積載荷重低減（令85条2項）の参考表示。
/// `Model.load_cfg.live_load_reduction == true` のときのみ表示する。
/// 支持床数・低減率の集計は `crate::app::column_live_load_factors` による。
/// **検定の長期軸力への実適用は残課題**（表示のみ。荷重計算条件のツールチップにも明記）。
fn live_load_reduction_section(ui: &mut egui::Ui, app: &App) {
    if !app
        .core
        .model
        .load_cfg
        .as_ref()
        .is_some_and(|c| c.live_load_reduction)
    {
        return;
    }
    egui::CollapsingHeader::new("柱の積載荷重低減（令85条2項・参考表示）")
        .id_salt("live_load_reduction_section")
        .default_open(true)
        .show(ui, |ui| {
            ui.colored_label(
                crate::theme::GRAY_600,
                "支える床の数に応じた低減率の集計値です。断面検定の長期軸力への実適用は未対応（残課題）。",
            );
            let factors = crate::app::column_live_load_factors(&app.core.model);
            if factors.is_empty() {
                ui.label("柱要素（鉛直材）がありません。準備計算で階が生成され所属階が設定されると床数を集計できます。");
                return;
            }
            for (elem, floors, factor) in factors {
                ui.label(format!(
                    "柱#{}: 支持床数 {} → 低減率 {:.2}",
                    elem.0, floors, factor
                ));
            }
        });
    ui.add_space(6.0);
}

pub fn design_table(ui: &mut egui::Ui, app: &mut App) {
    use crate::table_util::Col;

    live_load_reduction_section(ui, app);

    // ── 一次設計: 部材検定表 ─────────────────────────────────────
    ui.strong("部材検定（許容応力度）");
    // 断面算定条件（許容応力度設計・令82条）。
    ui.horizontal(|ui| {
        let mut changed = false;
        changed |= ui
            .checkbox(
                &mut app.core.analysis_cfg.rc_damage_control,
                "RC短期せん断: 損傷制御",
            )
            .on_hover_text(
                "ON: 損傷制御のための検討（2/3・α・fs）、OFF: 安全確保のための検討。\
                 軽量コンクリート×高強度せん断補強筋は常に安全確保式（RC規準）",
            )
            .changed();
        ui.label("QD:");
        for (m, label) in [
            (squid_n_design_jp::QdMethod::Min, "min(QD1,QD2)"),
            (squid_n_design_jp::QdMethod::Qd1, "QD1"),
            (squid_n_design_jp::QdMethod::Qd2, "QD2"),
        ] {
            if ui
                .selectable_label(app.core.analysis_cfg.qd_method == m, label)
                .on_hover_text(
                    "地震時短期の設計用せん断力の決定方法（QD1=終局曲げベース、\
                     QD2=QL+n・QE。長期組合せ(G+P)を先に解析している場合のみ有効）",
                )
                .clicked()
            {
                app.core.analysis_cfg.qd_method = m;
                changed = true;
            }
        }
        ui.label("付着:");
        for (m, label) in [
            (squid_n_design_jp::BondMethod::Rc1999, "1999"),
            (squid_n_design_jp::BondMethod::Rc1991, "1991"),
        ] {
            if ui
                .selectable_label(app.core.analysis_cfg.bond_method == m, label)
                .on_hover_text(
                    "RC 梁付着検定の方式。1999=必要付着長さ、1991=τa=Q/(ψ·j)。既定は 1999",
                )
                .clicked()
            {
                app.core.analysis_cfg.bond_method = m;
                changed = true;
            }
        }
        if changed {
            app.run_design_check();
        }
    });
    if app.core.scoped.staleness.design_stale {
        ui.colored_label(
            crate::theme::WARN_TEXT,
            "⚠ モデルが編集されました。解析を再実行してください。",
        );
    }
    // 1 行 = 1 検定位置。検定不能（Skipped）は ratio/ok を持たないため
    // `Option` にし、根拠セルには reason を表示する（判定は「検定不能」灰色）。
    struct CheckRow {
        elem: squid_n_core::ids::ElemId,
        pos: f64,
        ratio: Option<f64>,
        ok: Option<bool>,
        basis: String,
        /// 全検定式に共通の数値根拠（根拠セルのホバー表示用）。
        detail: String,
        components: Vec<squid_n_design_jp::CheckComponent>,
    }
    let checks: Vec<CheckRow> = app
        .core
        .scoped
        .results
        .as_ref()
        .map(|r| {
            r.member_checks
                .iter()
                .flat_map(|m| {
                    m.positions.iter().map(move |p| match &p.outcome {
                        squid_n_design_jp::CheckOutcome::Checked(cr) => CheckRow {
                            elem: m.elem,
                            pos: p.xi,
                            ratio: Some(cr.ratio()),
                            ok: Some(cr.ok()),
                            basis: cr.basis.clone(),
                            detail: cr.detail.clone(),
                            components: cr.components.clone(),
                        },
                        squid_n_design_jp::CheckOutcome::Skipped { reason } => CheckRow {
                            elem: m.elem,
                            pos: p.xi,
                            ratio: None,
                            ok: None,
                            basis: reason.clone(),
                            detail: String::new(),
                            components: Vec::new(),
                        },
                    })
                })
                .collect()
        })
        .unwrap_or_default();
    // 各行の部材に割り当てられている断面（NG部材→断面編集への遷移用）。
    let section_of: Vec<Option<(squid_n_core::ids::SectionId, String)>> = checks
        .iter()
        .map(|row| {
            app.core
                .model
                .elements
                .iter()
                .find(|e| e.id == row.elem)
                .and_then(|e| e.section)
                .and_then(|sid| {
                    app.core
                        .model
                        .sections
                        .iter()
                        .find(|s| s.id == sid)
                        .map(|s| (sid, s.name.clone()))
                })
        })
        .collect();

    if checks.is_empty() {
        ui.colored_label(
            crate::theme::GRAY_600,
            "検定結果がありません。解析タブから静的解析を実行してください（部材に断面と材料の割当が必要です）。",
        );
    } else {
        // NG 件数集計には検定不能（ok=None）を含めない。
        let ng_count = checks.iter().filter(|row| row.ok == Some(false)).count();
        ui.label(format!(
            "{} 位置を検定、NG {} 件（部材IDクリックで 3D ビューにハイライト）",
            checks.len(),
            ng_count
        ));
    }

    let n = checks.len();
    let mut focus: Option<squid_n_core::ids::ElemId> = None;
    let mut jump_to_section: Option<(squid_n_core::ids::SectionId, squid_n_core::ids::ElemId)> =
        None;
    crate::table_util::standard_table(
        ui,
        "design_member_checks",
        &[
            Col::id_named("部材"),
            Col::num("位置"),
            Col::num("検定比"),
            Col::label("判定"),
            Col::text("根拠"),
            Col::text("内訳"),
            Col::name("断面"),
        ],
        n,
        |row| {
            let i = row.index();
            let r = &checks[i];
            let is_focus = app.ui.scoped.nav.focus_member == Some(r.elem);
            row.col(|ui| {
                if crate::table_util::id_cell(
                    ui,
                    is_focus,
                    r.elem.0,
                    "クリックで部材を選択（結果タブの3Dビューで確認できます）",
                ) {
                    focus = Some(r.elem);
                }
            });
            row.col(|ui| {
                ui.label(format!("{:.3}", r.pos));
            });
            row.col(|ui| match r.ratio {
                Some(ratio) => {
                    ui.colored_label(crate::theme::status_color(ratio), format!("{:.4}", ratio));
                }
                None => {
                    ui.label("-");
                }
            });
            row.col(|ui| match r.ok {
                Some(true) => {
                    ui.label("OK");
                }
                Some(false) => {
                    ui.colored_label(crate::theme::ERROR_RED, "NG");
                }
                None => {
                    ui.colored_label(crate::theme::GRAY_600, "検定不能");
                }
            });
            row.col(|ui| {
                if r.detail.is_empty() {
                    crate::table_util::text_cell(ui, &r.basis);
                } else {
                    ui.label(&r.basis).on_hover_text(&r.detail);
                }
            });
            row.col(|ui| {
                if r.components.is_empty() {
                    ui.label("-");
                } else {
                    ui.horizontal(|ui| {
                        for (idx, c) in r.components.iter().enumerate() {
                            if idx > 0 {
                                ui.label("／");
                            }
                            ui.colored_label(
                                crate::theme::status_color(c.ratio),
                                format!("{} {:.2}", c.kind.label(), c.ratio),
                            )
                            .on_hover_text(&c.detail);
                        }
                    });
                }
            });
            row.col(|ui| match &section_of[i] {
                Some((sid, name)) => {
                    if ui
                        .button(name)
                        .on_hover_text("クリックでモデルタブの断面編集へ移動")
                        .clicked()
                    {
                        jump_to_section = Some((*sid, r.elem));
                    }
                }
                None => {
                    ui.label("-");
                }
            });
        },
    );
    if let Some(id) = focus {
        app.ui.scoped.nav.focus_member = Some(id);
    }
    if let Some((sid, eid)) = jump_to_section {
        app.ui.view.active_tab = crate::app::Tab::Model;
        app.ui.view.model_tab = crate::app::ModelTab::Sections;
        app.ui.scoped.nav.focus_section = Some(sid);
        app.ui.scoped.nav.focus_member = Some(eid);
    }

    // ── 一次設計: 節点単位の検定（柱梁接合部・パネルゾーン・冷間耐力比・耐震壁） ──
    struct JointCheckRow {
        node: squid_n_core::ids::NodeId,
        label: String,
        ratio: Option<f64>,
        ok: Option<bool>,
        basis: String,
        /// 根拠セルのホバー表示用（単一式なので component の detail。
        /// 共通 detail がある場合はその後に連結する）。
        detail: String,
    }
    let joint_checks: Vec<JointCheckRow> = app
        .core
        .scoped
        .results
        .as_ref()
        .map(|r| {
            r.joint_checks
                .iter()
                .map(|j| match &j.outcome {
                    squid_n_design_jp::CheckOutcome::Checked(cr) => {
                        let mut detail = cr.detail.clone();
                        if let Some(c) = cr.components.first() {
                            if !detail.is_empty() {
                                detail.push_str(", ");
                            }
                            detail.push_str(&c.detail);
                        }
                        JointCheckRow {
                            node: j.node,
                            label: j.label.clone(),
                            ratio: Some(cr.ratio()),
                            ok: Some(cr.ok()),
                            basis: cr.basis.clone(),
                            detail,
                        }
                    }
                    squid_n_design_jp::CheckOutcome::Skipped { reason } => JointCheckRow {
                        node: j.node,
                        label: j.label.clone(),
                        ratio: None,
                        ok: None,
                        basis: reason.clone(),
                        detail: String::new(),
                    },
                })
                .collect()
        })
        .unwrap_or_default();
    if !joint_checks.is_empty() {
        ui.add_space(12.0);
        ui.strong("接合部・耐震壁の検定");
        // NG 件数集計には検定不能（ok=None）を含めない。
        let ng = joint_checks.iter().filter(|j| j.ok == Some(false)).count();
        ui.label(format!("{} 箇所を検定、NG {} 件", joint_checks.len(), ng));
        crate::table_util::standard_table(
            ui,
            "joint_checks",
            &[
                Col::id_named("節点"),
                Col::name("種別"),
                Col::num("検定比"),
                Col::label("判定"),
                Col::text("根拠"),
            ],
            joint_checks.len(),
            |row| {
                let j = &joint_checks[row.index()];
                row.col(|ui| {
                    crate::table_util::id_label(ui, j.node.0);
                });
                row.col(|ui| {
                    crate::table_util::text_cell(ui, &j.label);
                });
                row.col(|ui| match j.ratio {
                    Some(ratio) => {
                        ui.colored_label(
                            crate::theme::status_color(ratio),
                            format!("{:.4}", ratio),
                        );
                    }
                    None => {
                        ui.label("-");
                    }
                });
                row.col(|ui| match j.ok {
                    Some(true) => {
                        ui.label("OK");
                    }
                    Some(false) => {
                        ui.colored_label(crate::theme::ERROR_RED, "NG");
                    }
                    None => {
                        ui.colored_label(crate::theme::GRAY_600, "検定不能");
                    }
                });
                row.col(|ui| {
                    if j.detail.is_empty() {
                        crate::table_util::text_cell(ui, &j.basis);
                    } else {
                        ui.label(&j.basis).on_hover_text(&j.detail);
                    }
                });
            },
        );
    }

    // ── 免震支承材の非線形特性 ────────────
    if !app.core.model.isolator_attrs.is_empty() {
        ui.add_space(12.0);
        ui.strong("免震支承材の非線形特性");
        for a in &app.core.model.isolator_attrs {
            let p = a.props;
            let ks = squid_n_design_jp::isolator::multi_shear_stiffness_reduction(p.n_springs);
            let qs = squid_n_design_jp::isolator::multi_shear_strength_reduction(p.n_springs);
            use squid_n_core::model::IsolatorKind;
            let desc = match p.kind {
                IsolatorKind::LaminatedRubber
                | IsolatorKind::LeadRubber
                | IsolatorKind::HighDampingRubber => {
                    // 等価水平剛性 keq・等価粘性減衰定数 Heq を設計変位 200mm（参考）で算定
                    // （LRB 統一型 keq=Qd/δ+Kd、Heq=(2/π)Qd(δ−Qd/((β−1)Kd))/(keq·δ²)）。
                    let disp = 200.0;
                    let keq = squid_n_design_jp::isolator::equivalent_stiffness(p.k2, p.qd, disp);
                    let heq =
                        squid_n_design_jp::isolator::equivalent_damping(p.k1, p.k2, p.qd, disp);
                    let kind_label = match p.kind {
                        IsolatorKind::LeadRubber => "鉛プラグ積層ゴム(LRB)",
                        IsolatorKind::HighDampingRubber => "高減衰ゴム(HDR)",
                        _ => "天然ゴム積層ゴム",
                    };
                    let strain_dep = if p.total_rubber_thickness > 0.0
                        && (p.ckd_gamma != [1.0, 0.0, 0.0] || p.cqd_gamma != [1.0, 0.0, 0.0])
                    {
                        format!("／ 歪依存 H={:.0}mm", p.total_rubber_thickness)
                    } else {
                        String::new()
                    };
                    format!(
                        "{} K1={:.0}kN/mm K2={:.0}kN/mm Qd={:.0}kN Kv={:.0}kN/mm ／ δ=200mm時 keq={:.1}kN/mm Heq={:.3} {}",
                        kind_label,
                        stiffness_kn_per_mm(p.k1),
                        stiffness_kn_per_mm(p.k2),
                        force_kn(p.qd),
                        stiffness_kn_per_mm(p.kv),
                        stiffness_kn_per_mm(keq),
                        heq,
                        strain_dep
                    )
                }
                IsolatorKind::ElasticSliding => format!(
                    "弾性すべり μ={:.3} N={:.0}kN Qmax={:.0}kN Kv={:.0}kN/mm",
                    p.mu,
                    force_kn(p.n_long),
                    force_kn(squid_n_design_jp::isolator::friction_max_force(
                        p.mu, p.n_long
                    )),
                    stiffness_kn_per_mm(p.kv)
                ),
            };
            ui.label(format!(
                "部材{}: {} ／ マルチシア n={} 剛性低減={:.4} 耐力低減={:.4}",
                a.elem.0, desc, p.n_springs, ks, qs
            ));
        }
    }

    // ── 制振ダンパーの非線形特性 ──
    if !app.core.model.damper_attrs.is_empty() {
        ui.add_space(12.0);
        ui.strong("制振ダンパーの非線形特性");
        for a in &app.core.model.damper_attrs {
            let p = a.props;
            match p.kind {
                squid_n_core::model::DamperKind::Maxwell => {
                    // 緩和時間 τ=C0/Kd。線形マクスウェルの損失は ωτ≈1 で最大。
                    let tau = if p.kd > 0.0 { p.c0 / p.kd } else { 0.0 };
                    ui.label(format!(
                        "部材{}: マクスウェル Kd={:.0} C0={:.0} α={:.2} ／ 緩和時間 τ={:.3}s（時刻歴で作用）",
                        a.elem.0, p.kd, p.c0, p.alpha, tau
                    ));
                }
                squid_n_core::model::DamperKind::HystereticBilinear => {
                    // 降伏変位 δy=Qy/k1。
                    let dy = if p.kd > 0.0 { p.qy / p.kd } else { 0.0 };
                    ui.label(format!(
                        "部材{}: 履歴型ﾊﾞｲﾘﾆｱ k1={:.0} Qy={:.0} k2/k1={:.3} ／ 降伏変位 δy={:.2}mm（静的・動的で作用）",
                        a.elem.0, p.kd, p.qy, p.k2_ratio, dy
                    ));
                }
            }
        }
    }

    // ── 非線形解析の材端履歴則 ──
    ui.add_space(12.0);
    egui::CollapsingHeader::new("非線形解析の材端履歴則(増分)")
        .default_open(false)
        .show(ui, |ui| {
            ui.label(
                "増分解析（保有水平耐力計算）の材端曲げバネの復元力履歴則。\
                 既定は RC/SRC/CFT 梁=武田型、S 梁=標準型（部材表で個別指定可）。",
            );
            use std::collections::BTreeMap;
            let mut counts: BTreeMap<&'static str, u32> = BTreeMap::new();
            let mut overrides: Vec<String> = Vec::new();
            for e in &app.core.model.elements {
                if e.kind != squid_n_core::model::ElementKind::Beam {
                    continue;
                }
                let eff = squid_n_element::factory::resolve_member_hysteresis(
                    e,
                    &app.core.model,
                    squid_n_core::model::AnalysisKind::Incremental,
                );
                *counts.entry(eff.label()).or_default() += 1;
                if let Some(r) = app.core.model.member_hysteresis(e.id) {
                    let mut line = format!("部材{}: {}", e.id.0, r.label());
                    if let Some(r_th) = app.core.model.member_hysteresis_th_raw(e.id) {
                        line.push_str(&format!("(時刻歴: {})", r_th.label()));
                    }
                    overrides.push(line);
                }
            }
            if counts.is_empty() {
                ui.label("梁部材がありません。");
            } else {
                for (label, cnt) in &counts {
                    ui.label(format!("{}: {} 部材", label, cnt));
                }
            }
            if !overrides.is_empty() {
                ui.label(format!("個別指定: {}", overrides.join(", ")));
            }
        });

    // ── 二次設計: 層指標（層間変形角・剛性率・偏心率） ────────────
    ui.add_space(12.0);
    ui.strong("層指標（二次設計: 層間変形角・剛性率・偏心率）");
    // 層間変形角・剛性率・偏心率と必要保有水平耐力の判定は、いずれも加力方向ごとに
    // 評価する（令82条の2・平19国交告594号）。評価方向は解析の実行条件ではなく
    // 判定の条件なので、設計タブのこの位置で選ぶ。
    ui.horizontal(|ui| {
        use squid_n_solver::analysis::SeismicDir;
        ui.label("加力方向:").on_hover_text(
            "層指標と必要保有水平耐力の判定を評価する方向。\
             剛心の精算には対応する向きの EX／EY の解析結果を用いる",
        );
        ui.selectable_value(&mut app.core.analysis_cfg.seismic_dir, SeismicDir::X, "X");
        ui.selectable_value(&mut app.core.analysis_cfg.seismic_dir, SeismicDir::Y, "Y");
    });
    if app.core.model.stories.is_empty() {
        ui.colored_label(
            crate::theme::GRAY_600,
            "階が未定義です。解析タブの「準備計算 実行」を行ってください。",
        );
    } else if let Some(st) = app.current_static() {
        // 表示対象はナビゲータの結果ケース選択（→最後に実行した結果）に追従する。
        let ctx = crate::summary::metrics_ctx_from_results(app.core.scoped.results.as_ref());
        let metrics = crate::summary::compute_story_metrics_with(
            &app.core.model,
            &st.disp,
            app.core.analysis_cfg.seismic_dir,
            &ctx,
        );

        // 変形角の制限値は計算条件（令82条の2: 原則 1/200、緩和時 1/120）に追従する。
        let denom = metrics
            .first()
            .map(|m| m.drift_limit_denom)
            .unwrap_or(app.core.model.stress_cfg.drift_limit_denom);
        let drift_label = format!("変形角(1/{:.0})", denom);
        crate::table_util::standard_table(
            ui,
            "design_story_metrics",
            &[
                Col::label("階"),
                Col::num("階高[mm]"),
                Col::num("層間変位[mm]"),
                Col::num(drift_label.as_str()),
                Col::num("剛性率Rs(≥0.6)"),
                Col::num("偏心率Re(≤0.15)"),
                Col::num("Fes"),
            ],
            metrics.len(),
            |row| {
                let m = &metrics[row.index()];
                row.col(|ui| {
                    crate::table_util::text_cell(ui, &m.name);
                });
                row.col(|ui| {
                    ui.label(format!("{:.0}", m.height));
                });
                row.col(|ui| {
                    ui.label(format!("{:.3}", m.drift));
                });
                row.col(|ui| {
                    let txt = if m.drift_angle > 1e-12 {
                        format!("1/{:.0}", 1.0 / m.drift_angle)
                    } else {
                        "0".to_string()
                    };
                    if m.drift_ok {
                        ui.colored_label(crate::theme::GOOD_GREEN, txt);
                    } else {
                        ui.colored_label(crate::theme::ERROR_RED, format!("{} NG", txt));
                    }
                });
                row.col(|ui| {
                    let txt = format!("{:.3}", m.rs);
                    if m.rs_ok {
                        ui.colored_label(crate::theme::GOOD_GREEN, txt);
                    } else {
                        ui.colored_label(crate::theme::ERROR_RED, format!("{} NG", txt));
                    }
                });
                row.col(|ui| {
                    let txt = format!("{:.3}", m.re);
                    if m.re_ok {
                        ui.colored_label(crate::theme::GOOD_GREEN, txt);
                    } else {
                        ui.colored_label(crate::theme::ERROR_RED, format!("{} NG", txt));
                    }
                });
                row.col(|ui| {
                    ui.label(format!("{:.3}", m.fes));
                });
            },
        );
    } else {
        ui.colored_label(
            crate::theme::GRAY_600,
            "静的解析結果がありません。荷重ケース EX／EY（地震力）を実行すると層指標を評価できます。",
        );
    }

    // ── 二次設計: 保有水平耐力（ルート3） ──────────────────────
    ui.add_space(12.0);
    ui.strong("保有水平耐力（ルート3）");
    ui.horizontal(|ui| {
        use squid_n_design_jp::secondary::holding_capacity::FrameType;
        ui.label("架構種別:");
        ui.selectable_value(&mut app.core.design_frame, FrameType::RcFrame, "RCラーメン");
        ui.selectable_value(&mut app.core.design_frame, FrameType::RcWall, "RC壁式");
        ui.selectable_value(
            &mut app.core.design_frame,
            FrameType::SteelFrame,
            "Sラーメン",
        );
        ui.selectable_value(
            &mut app.core.design_frame,
            FrameType::SteelBrace,
            "Sブレース",
        );
    });
    ui.horizontal(|ui| {
        ui.checkbox(
            &mut app.core.design_rank_auto,
            "自動判定（鋼=幅厚比・RC矩形=Qsu/Qmu）",
        )
        .on_hover_text(
            "鋼部材(断面形状を持つもの)は幅厚比から、RC矩形部材(断面形状 RcRect かつ\
                 コンクリート強度Fc設定済みの材料)はせん断余裕度 Qsu/Qmu の略算から\
                 部材ランクを層ごとに自動判定します。断面形状未設定の部材・幅厚比の対象外\
                 形状(円形鋼管等)・RC円形・Fc未設定材料はスキップされ、1 本も算定できなかった\
                 層は下記の選択値にフォールバックします。RC の Qsu の軸力項に用いる軸力は\
                 先頭荷重ケース（長期相当）の結果を優先し、なければ最後に実行した\
                 静的解析結果を使用する簡易運用です。",
        );
    });
    ui.horizontal(|ui| {
        use squid_n_design_jp::secondary::holding_capacity::MemberRank;
        ui.label(if app.core.design_rank_auto {
            "部材ランク（フォールバック用）:"
        } else {
            "部材ランク:"
        });
        ui.selectable_value(&mut app.core.design_rank, MemberRank::FA, "FA");
        ui.selectable_value(&mut app.core.design_rank, MemberRank::FB, "FB");
        ui.selectable_value(&mut app.core.design_rank, MemberRank::FC, "FC");
        ui.selectable_value(&mut app.core.design_rank, MemberRank::FD, "FD");
    });
    ui.horizontal(|ui| {
        ui.checkbox(&mut app.core.wall_structure, "壁式構造")
            .on_hover_text(
                "耐力壁の種別（WA〜WD）判定に壁式構造の列を用います。告示「耐力壁の種別」表は\
                 壁式構造で限界値が厳しく（τu/Fc: WA 0.1・WB 0.125・WC 0.15）、\
                 壁式構造以外（WA 0.20・WB 0.25）とは別の列になります。",
            );
    });
    if !app.core.design_rank_auto {
        let ds = squid_n_design_jp::secondary::holding_capacity::ds_value(
            app.core.design_frame,
            app.core.design_rank,
        );
        ui.label(format!("Ds = {:.2}（部材ランク選択値による簡易運用）", ds));
    }

    match app.compute_holding_capacity() {
        Err(msg) => {
            ui.colored_label(crate::theme::GRAY_600, &msg);
            let needs_analysis =
                msg.contains("増分解析") || msg.contains("地震静的") || msg.contains("階");
            if needs_analysis && ui.button("▶ 解析タブへ").clicked() {
                app.ui.view.active_tab = crate::app::Tab::Analysis;
            }
        }
        Ok((result, story_ranks)) => {
            crate::table_util::standard_table(
                ui,
                "design_holding_capacity",
                &[
                    Col::label("階"),
                    Col::num("Qu[kN]"),
                    Col::num("Qud[kN]"),
                    Col::num("Ds"),
                    Col::num("Fes"),
                    Col::num("Qun[kN]"),
                    Col::label("判定"),
                    Col::label("採用ランク"),
                ],
                result.stories.len(),
                |row| {
                    let i = row.index();
                    let s = &result.stories[i];
                    // 層の呼び名は下端の階名（法令の「i 階」）。
                    let name = app
                        .core
                        .model
                        .layers()
                        .get(i)
                        .map(|l| l.name.clone())
                        .unwrap_or_else(|| format!("{}", s.story.0));
                    row.col(|ui| {
                        crate::table_util::text_cell(ui, &name);
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.1}", force_kn(s.qu)));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.1}", force_kn(s.qud)));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.2}", s.ds));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.2}", s.fes));
                    });
                    row.col(|ui| {
                        ui.label(format!("{:.1}", force_kn(s.qun)));
                    });
                    row.col(|ui| {
                        if s.ok {
                            ui.colored_label(crate::theme::GOOD_GREEN, "OK");
                        } else {
                            ui.colored_label(crate::theme::ERROR_RED, "NG");
                        }
                    });
                    row.col(|ui| {
                        ui.label(story_ranks.get(i).map(|r| rank_label(*r)).unwrap_or("-"));
                    });
                },
            );
            // 崩壊機構（増分解析判定）を表示する。Ds は部材ランクに加えて
            // この崩壊機構を層別に反映する（層崩壊形の層は1段階不利、部分崩壊形は
            // 機構未確定として補正なし＝暫定値、全体崩壊形は標準）。
            if let Some(po) = app.displayed_pushover() {
                use squid_n_solver::pushover::MechanismType;
                let (mech, warn) = match &po.mechanism {
                    MechanismType::Overall => ("全体崩壊形".to_string(), false),
                    MechanismType::StoryCollapse { layer } => {
                        // 層の呼び名は下端の階名（法令の「i 階」）。
                        let name = app
                            .core
                            .model
                            .layers()
                            .get(*layer)
                            .map(|l| l.name.clone())
                            .unwrap_or_else(|| format!("{}", layer + 1));
                        (format!("層崩壊形 ({name})"), false)
                    }
                    MechanismType::Partial => ("部分崩壊形（機構未形成）".to_string(), true),
                };
                ui.colored_label(
                    if warn {
                        crate::theme::SECONDARY_AMBER
                    } else {
                        crate::theme::GRAY_600
                    },
                    format!(
                        "崩壊機構: {}（Ds へ層別に反映{}）",
                        mech,
                        if warn {
                            "。機構が確定するまで Ds・Qun は暫定値です"
                        } else {
                            ""
                        }
                    ),
                );
            }
            // βu（耐力壁・筋かいの水平耐力比）の算定状況。Ds 表の行選択に直結するため
            // 算定値、または算定できなかった旨を明示する。
            if app.core.scoped.ds_beta_u_unavailable {
                ui.colored_label(
                    crate::theme::SECONDARY_AMBER,
                    "⚠ 架構種別が耐力壁付き／筋かい付きですが、耐力壁・筋かい部材を検出\
                     できなかったため βu を算定できません。架構種別別の Ds 表で代用して\
                     います（告示の βu 別の表は適用されていません）。",
                );
            } else if !app.core.scoped.ds_beta_u_by_story.is_empty()
                && app.core.scoped.ds_beta_u_by_story.iter().any(|b| *b > 0.0)
            {
                let list = app
                    .core
                    .scoped
                    .ds_beta_u_by_story
                    .iter()
                    .map(|b| format!("{:.2}", b))
                    .collect::<Vec<_>>()
                    .join(", ");
                ui.colored_label(
                    crate::theme::GRAY_600,
                    format!("βu（耐力壁・筋かいの水平耐力比、下階→上階）: {}", list),
                );
            }
            // rank-auto で 1 本も算定できず選択ランクへフォールバックした層の警告。
            // 幅厚比表の対象外形状（円形鋼管等）・形状未設定などの層は選択ランク
            // （既定 FA）のまま Ds が決まり、実状より甘いと危険側になるため明示する。
            if app.core.design_rank_auto && !app.core.scoped.ds_rank_fallback_stories.is_empty() {
                ui.colored_label(
                    crate::theme::SECONDARY_AMBER,
                    format!(
                        "⚠ 部材ランクを 1 本も算定できず、選択ランク {:?} を適用した層があります\
                         （{}）。幅厚比表の対象外形状・断面形状未設定・Fc 未設定などが原因です。\
                         選択ランクが実状より甘いと Ds を過小評価するため、該当層の部材種別を\
                         確認してください。",
                        app.core.design_rank,
                        app.core.scoped.ds_rank_fallback_stories.join("、"),
                    ),
                );
            }
            let note = if app.core.design_rank_auto {
                "Qu は増分解析性能曲線上の層別ピーク層せん断力（崩壊機構形成時の耐力）。\
                 Ds は部材ランク自動判定（鋼=幅厚比、RC矩形=せん断余裕度 Qsu/Qmu の略算。柱は\
                 軸力考慮の曲げ終局から Qmu を算定）×崩壊機構。形状未設定・RC円形・Fc未設定材料は\
                 選択値フォールバック。"
            } else {
                "Qu は増分解析性能曲線上の層別ピーク層せん断力。Ds は選択ランク×崩壊機構\
                 （部材ランク自動判定OFF）。"
            };
            ui.colored_label(crate::theme::GRAY_600, note);
        }
    }

    floor_design_section(ui, app);
}

/// 床の中での小梁・スラブ設計の表示（`ResultsBundle.joist_checks`/`slab_checks`）。
/// 小梁は単純支持梁として曲げ・たわみを検定し、スラブは一方向版として設計曲げ
/// モーメント・必要鉄筋量を表示する（いずれも全体 FEM から独立）。
fn floor_design_section(ui: &mut egui::Ui, app: &App) {
    use crate::table_util::Col;
    let Some(r) = app.core.scoped.results.as_ref() else {
        return;
    };
    if r.joist_checks.is_empty() && r.slab_checks.is_empty() {
        return;
    }

    ui.add_space(12.0);
    ui.strong("小梁・床の設計（床の中で・単純支持／一方向）");
    ui.colored_label(
        crate::theme::GRAY_600,
        "小梁は大梁を分割せず、床の中で単純支持梁として曲げ・たわみを検定します（反力は\
         大梁へ CMQ として伝達）。スラブは一方向版として設計曲げと必要鉄筋量を算定します。\
         鋼小梁の E・長期 ft は断面材料（未設定時 E=205000・F=235）。鉄筋は SD295（長期 ft=195）です。",
    );

    if !r.joist_checks.is_empty() {
        ui.label("小梁（単純支持梁）:");
        crate::table_util::standard_table(
            ui,
            "joist_design_table",
            &[
                Col::id_named("スラブ"),
                Col::id_named("二次部材"),
                Col::num("スパン[mm]"),
                Col::num("M[kN·m]"),
                Col::num("Q[kN]"),
                Col::wide_num("δ[mm]"),
                Col::num("検定比"),
                Col::label("判定"),
            ],
            r.joist_checks.len(),
            |row| {
                let (sid, ji, jr) = &r.joist_checks[row.index()];
                row.col(|ui| {
                    // 間柱には床板が無い。小梁でも所属床領域が床板を持たなければ空になる。
                    ui.label(match sid {
                        Some(id) => format!("#{}", id.0),
                        None => "—".into(),
                    });
                });
                row.col(|ui| {
                    let label = match ji {
                        crate::app::JoistCheckTarget::SecondaryJoist { nodes } => {
                            secondary_label(app.core.model.joists(), *nodes)
                        }
                        crate::app::JoistCheckTarget::SecondaryPost { nodes } => {
                            format!(
                                "（間柱）{}",
                                secondary_label(app.core.model.posts(), *nodes)
                            )
                        }
                    };
                    ui.label(label);
                });
                row.col(|ui| {
                    ui.label(format!("{:.0}", jr.span));
                });
                row.col(|ui| {
                    ui.label(format!("{:.2}", moment_kn_m(jr.m_max)));
                });
                row.col(|ui| {
                    ui.label(format!("{:.2}", force_kn(jr.q_max)));
                });
                row.col(|ui| {
                    ui.label(format!("{:.2} (δ/L=1/{:.0})", jr.deflection, {
                        if jr.deflection_span_ratio > 0.0 {
                            1.0 / jr.deflection_span_ratio
                        } else {
                            f64::INFINITY
                        }
                    }));
                });
                row.col(|ui| {
                    ui.colored_label(
                        crate::theme::status_color(jr.ratio),
                        format!("{:.2}", jr.ratio),
                    );
                });
                row.col(|ui| {
                    if jr.unchecked {
                        ui.label("未");
                    } else if jr.ok {
                        ui.colored_label(crate::theme::GOOD_GREEN, "OK");
                    } else {
                        ui.colored_label(crate::theme::ERROR_RED, "NG");
                    }
                });
            },
        );
    }

    if !r.slab_checks.is_empty() {
        ui.add_space(6.0);
        ui.label("スラブ（一方向版）:");
        crate::table_util::standard_table(
            ui,
            "slab_design_table",
            &[
                Col::id_named("スラブ"),
                Col::num("スパン[mm]"),
                Col::num("w[kN/m²]"),
                Col::num("M[kN·m/m]"),
                Col::num("t[mm]"),
                Col::num("必要As[mm²/m]"),
            ],
            r.slab_checks.len(),
            |row| {
                let (sid, sr) = &r.slab_checks[row.index()];
                row.col(|ui| {
                    ui.label(format!("#{}", sid.0));
                });
                row.col(|ui| {
                    ui.label(format!("{:.0}", sr.span));
                });
                row.col(|ui| {
                    ui.label(format!("{:.2}", area_load_kn_per_m2(sr.w)));
                });
                row.col(|ui| {
                    ui.label(format!("{:.2}", moment_kn_m_per_m(sr.moment)));
                });
                row.col(|ui| {
                    ui.label(format!("{:.0}", sr.thickness));
                });
                row.col(|ui| {
                    ui.label(format!("{:.0}", sr.as_req_per_m));
                });
            },
        );
    }
}

/// `MemberRank` の表示名（FA〜FD）。
fn rank_label(r: squid_n_design_jp::secondary::holding_capacity::MemberRank) -> &'static str {
    use squid_n_design_jp::secondary::holding_capacity::MemberRank;
    match r {
        MemberRank::FA => "FA",
        MemberRank::FB => "FB",
        MemberRank::FC => "FC",
        MemberRank::FD => "FD",
    }
}

/// 二次部材の表示名（名前が空なら端点の節点対から作る）。
fn secondary_label<'a>(
    members: impl Iterator<Item = &'a squid_n_core::model::SecondaryMember>,
    nodes: [squid_n_core::ids::NodeId; 2],
) -> String {
    let key = (nodes[0].0.min(nodes[1].0), nodes[0].0.max(nodes[1].0));
    members
        .filter(|sm| {
            (
                sm.nodes[0].0.min(sm.nodes[1].0),
                sm.nodes[0].0.max(sm.nodes[1].0),
            ) == key
        })
        .find_map(|sm| (!sm.name.is_empty()).then(|| sm.name.clone()))
        .unwrap_or_else(|| format!("SM{}-{}", nodes[0].0, nodes[1].0))
}
