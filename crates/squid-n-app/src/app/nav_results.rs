//! ナビゲータ（左パネル）の解析結果ツリー。
//!
//! 解析種別で階層化した結果ツリーを表示し、葉ノードのクリックで結果タブの
//! 表示対象を切り替える。ツリー構築は [`build_result_tree`] に集約し、
//! 画面切替は [`apply_result_tree_action`] に集約する。

use super::*;
use squid_n_core::ids::{LumpedVibrationCaseId, VibrationCaseId};
use squid_n_core::model::{Model, EX_CASE_NAME, EY_CASE_NAME};
use squid_n_solver::analysis::SeismicDir;

/// ナビゲータに並べる解析結果ツリー（純データ）。
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub(crate) struct ResultTree {
    pub static_leaves: Vec<(StaticKey, String)>,
    pub eigen_modes: Vec<(usize, String)>,
    pub pushover_dirs: Vec<SeismicDir>,
    pub time_history_cases: Vec<(VibrationCaseId, String)>,
    pub lumped_eigen_modes: Vec<(usize, String)>,
    pub lumped_th_cases: Vec<(LumpedVibrationCaseId, String)>,
}

impl ResultTree {
    pub fn is_empty(&self) -> bool {
        self.static_leaves.is_empty()
            && self.eigen_modes.is_empty()
            && self.pushover_dirs.is_empty()
            && self.time_history_cases.is_empty()
            && self.lumped_eigen_modes.is_empty()
            && self.lumped_th_cases.is_empty()
    }

    pub fn has_lumped_section(&self) -> bool {
        !self.lumped_eigen_modes.is_empty() || !self.lumped_th_cases.is_empty()
    }
}

pub(crate) fn user_static_label(name: &str) -> String {
    name.to_string()
}

pub(crate) fn seismic_static_label(dir: SeismicDir) -> &'static str {
    match dir {
        SeismicDir::X => EX_CASE_NAME,
        SeismicDir::Y => EY_CASE_NAME,
    }
}

pub(crate) fn eigen_mode_label(mode_index: usize, period_s: f64) -> String {
    format!(
        "{}次 (T={} s)",
        mode_index + 1,
        format_eigen_period(period_s)
    )
}

fn format_eigen_period(t: f64) -> String {
    let s = format!("{t:.2}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

pub(crate) fn pushover_dir_label(dir: SeismicDir) -> &'static str {
    match dir {
        SeismicDir::X => "X方向",
        SeismicDir::Y => "Y方向",
    }
}

fn vibration_case_label(model: &Model, id: VibrationCaseId) -> String {
    model
        .vibration_cases
        .iter()
        .find(|c| c.id == id)
        .map(|c| c.name.clone())
        .unwrap_or_else(|| format!("ケース {}", id.0))
}

fn lumped_vibration_case_label(model: &Model, id: LumpedVibrationCaseId) -> String {
    model
        .lumped_vibration_cases
        .iter()
        .find(|c| c.id == id)
        .map(|c| c.name.clone())
        .unwrap_or_else(|| format!("ケース {}", id.0))
}

/// `ResultsBundle` とモデルからナビの解析結果ツリーを構築する。
pub(crate) fn build_result_tree(
    results: Option<&ResultsBundle>,
    model: &Model,
    load_case_name: impl Fn(LoadCaseId) -> String,
) -> ResultTree {
    let Some(r) = results else {
        return ResultTree::default();
    };

    let mut static_leaves = Vec::new();
    for (key, _) in &r.statics {
        let sk = StaticKey::Case(*key);
        let label = match key {
            StaticCaseKey::User(id) => user_static_label(&load_case_name(*id)),
            StaticCaseKey::Seismic(dir) => seismic_static_label(*dir).to_string(),
        };
        static_leaves.push((sk, label));
    }
    for (i, (name, _)) in r.combos.iter().enumerate() {
        static_leaves.push((StaticKey::Combo(i), name.clone()));
    }

    let eigen_modes = r
        .modal
        .as_ref()
        .map(|m| {
            m.period
                .iter()
                .enumerate()
                .map(|(i, t)| (i, eigen_mode_label(i, *t)))
                .collect()
        })
        .unwrap_or_default();

    let mut pushover_dirs = Vec::new();
    if r.pushover_x.is_some() {
        pushover_dirs.push(SeismicDir::X);
    }
    if r.pushover_y.is_some() {
        pushover_dirs.push(SeismicDir::Y);
    }
    if pushover_dirs.is_empty() && r.pushover.is_some() {
        pushover_dirs.push(r.infer_pushover_view_dir(SeismicDir::X));
    }

    let time_history_cases: Vec<_> = r
        .time_histories
        .iter()
        .map(|(id, _)| (*id, vibration_case_label(model, *id)))
        .collect();

    let lumped_eigen_modes = r
        .lumped
        .as_ref()
        .map(|l| {
            l.modal
                .period
                .iter()
                .enumerate()
                .map(|(i, t)| (i, eigen_mode_label(i, *t)))
                .collect()
        })
        .unwrap_or_default();

    let lumped_th_cases: Vec<_> = r
        .lumped_results
        .iter()
        .map(|(id, _)| (*id, lumped_vibration_case_label(model, *id)))
        .collect();

    ResultTree {
        static_leaves,
        eigen_modes,
        pushover_dirs,
        time_history_cases,
        lumped_eigen_modes,
        lumped_th_cases,
    }
}

#[cfg(feature = "gui")]
enum ResultTreeAction {
    Static(StaticKey),
    EigenMode(usize),
    Pushover(SeismicDir),
    TimeHistoryCase(VibrationCaseId),
    LumpedEigenMode(usize),
    LumpedTimeHistoryCase(LumpedVibrationCaseId),
}

#[cfg(feature = "gui")]
fn is_static_selected(app: &App, key: StaticKey) -> bool {
    app.nav.focus_result == Some(key)
}

#[cfg(feature = "gui")]
fn is_eigen_mode_selected(app: &App, mode_idx: usize) -> bool {
    app.results_view == ResultsView::Spatial
        && app.view_mode == crate::viewer::ViewMode::Mode
        && app.view_mode_idx == mode_idx
}

#[cfg(feature = "gui")]
fn is_pushover_selected(app: &App, dir: SeismicDir) -> bool {
    app.results_view == ResultsView::Pushover && app.pushover_view_dir == dir
}

#[cfg(feature = "gui")]
fn is_time_history_case_selected(app: &App, id: VibrationCaseId) -> bool {
    app.results_view == ResultsView::TimeHistory && app.view_vibration_case == Some(id)
}

#[cfg(feature = "gui")]
fn is_lumped_eigen_mode_selected(app: &App, mode_idx: usize) -> bool {
    app.results_view == ResultsView::Spatial
        && app.view_mode == crate::viewer::ViewMode::LumpedMode
        && app.view_mode_idx == mode_idx
}

#[cfg(feature = "gui")]
fn is_lumped_th_case_selected(app: &App, id: LumpedVibrationCaseId) -> bool {
    app.results_view == ResultsView::LumpedMass && app.view_lumped_vibration_case == Some(id)
}

#[cfg(feature = "gui")]
fn draw_header_underline(ui: &egui::Ui, rect: egui::Rect) {
    ui.painter().line_segment(
        [rect.left_bottom(), rect.right_bottom()],
        egui::Stroke::new(2.0_f32, crate::theme::DATA_BLUE),
    );
}

#[cfg(feature = "gui")]
impl App {
    pub(crate) fn nav_result_cases(&mut self, ui: &mut egui::Ui) {
        let tree = build_result_tree(self.results.as_ref(), &self.model, |id| {
            self.model
                .load_cases
                .iter()
                .find(|lc| lc.id == id)
                .map(|lc| lc.name.clone())
                .unwrap_or_default()
        });

        let mut action: Option<ResultTreeAction> = None;

        let header = egui::CollapsingHeader::new("解析結果")
            .default_open(true)
            .id_salt("nav_result_cases");
        header.show(ui, |ui| {
            if tree.is_empty() {
                ui.label("（未実行）");
                return;
            }

            if !tree.static_leaves.is_empty() {
                let static_header_sel = self.nav.focus_result.is_some();
                let h = egui::CollapsingHeader::new("静的解析")
                    .default_open(true)
                    .id_salt("nav_result_static");
                let s_resp = h.show(ui, |ui| {
                    for (key, label) in &tree.static_leaves {
                        if ui
                            .selectable_label(is_static_selected(self, *key), label)
                            .clicked()
                        {
                            action = Some(ResultTreeAction::Static(*key));
                        }
                    }
                });
                if static_header_sel {
                    draw_header_underline(ui, s_resp.header_response.rect);
                }
            }

            if !tree.eigen_modes.is_empty() {
                let eigen_sel = tree
                    .eigen_modes
                    .iter()
                    .any(|(i, _)| is_eigen_mode_selected(self, *i));
                let h = egui::CollapsingHeader::new("固有値解析")
                    .default_open(false)
                    .id_salt("nav_result_eigen");
                let e_resp = h.show(ui, |ui| {
                    for (i, label) in &tree.eigen_modes {
                        if ui
                            .selectable_label(is_eigen_mode_selected(self, *i), label)
                            .clicked()
                        {
                            action = Some(ResultTreeAction::EigenMode(*i));
                        }
                    }
                });
                if eigen_sel {
                    draw_header_underline(ui, e_resp.header_response.rect);
                }
            }

            if !tree.pushover_dirs.is_empty() {
                let po_sel = tree
                    .pushover_dirs
                    .iter()
                    .any(|d| is_pushover_selected(self, *d));
                let h = egui::CollapsingHeader::new("増分解析")
                    .default_open(false)
                    .id_salt("nav_result_pushover");
                let p_resp = h.show(ui, |ui| {
                    for dir in &tree.pushover_dirs {
                        let label = pushover_dir_label(*dir);
                        if ui
                            .selectable_label(is_pushover_selected(self, *dir), label)
                            .clicked()
                        {
                            action = Some(ResultTreeAction::Pushover(*dir));
                        }
                    }
                });
                if po_sel {
                    draw_header_underline(ui, p_resp.header_response.rect);
                }
            }

            if !tree.time_history_cases.is_empty() {
                let th_sel = tree
                    .time_history_cases
                    .iter()
                    .any(|(id, _)| is_time_history_case_selected(self, *id));
                let h = egui::CollapsingHeader::new("時刻歴応答解析")
                    .default_open(false)
                    .id_salt("nav_result_time_history");
                let t_resp = h.show(ui, |ui| {
                    for (id, label) in &tree.time_history_cases {
                        if ui
                            .selectable_label(is_time_history_case_selected(self, *id), label)
                            .clicked()
                        {
                            action = Some(ResultTreeAction::TimeHistoryCase(*id));
                        }
                    }
                });
                if th_sel {
                    draw_header_underline(ui, t_resp.header_response.rect);
                }
            }

            if tree.has_lumped_section() {
                let lumped_sel = tree
                    .lumped_eigen_modes
                    .iter()
                    .any(|(i, _)| is_lumped_eigen_mode_selected(self, *i))
                    || tree
                        .lumped_th_cases
                        .iter()
                        .any(|(id, _)| is_lumped_th_case_selected(self, *id));
                let h = egui::CollapsingHeader::new("質点系")
                    .default_open(false)
                    .id_salt("nav_result_lumped");
                let l_resp = h.show(ui, |ui| {
                    if !tree.lumped_eigen_modes.is_empty() {
                        let eh = egui::CollapsingHeader::new("固有値")
                            .default_open(false)
                            .id_salt("nav_result_lumped_eigen");
                        eh.show(ui, |ui| {
                            for (i, label) in &tree.lumped_eigen_modes {
                                if ui
                                    .selectable_label(
                                        is_lumped_eigen_mode_selected(self, *i),
                                        label,
                                    )
                                    .clicked()
                                {
                                    action = Some(ResultTreeAction::LumpedEigenMode(*i));
                                }
                            }
                        });
                    }
                    if !tree.lumped_th_cases.is_empty() {
                        let th = egui::CollapsingHeader::new("時刻歴")
                            .default_open(false)
                            .id_salt("nav_result_lumped_th");
                        th.show(ui, |ui| {
                            for (id, label) in &tree.lumped_th_cases {
                                if ui
                                    .selectable_label(is_lumped_th_case_selected(self, *id), label)
                                    .clicked()
                                {
                                    action = Some(ResultTreeAction::LumpedTimeHistoryCase(*id));
                                }
                            }
                        });
                    }
                });
                if lumped_sel {
                    draw_header_underline(ui, l_resp.header_response.rect);
                }
            }
        });

        if let Some(action) = action {
            self.apply_result_tree_action(action);
        }
    }

    fn apply_result_tree_action(&mut self, action: ResultTreeAction) {
        match action {
            ResultTreeAction::Static(key) => {
                self.select_displayed_result(key);
                self.active_tab = Tab::Results;
                self.results_view = ResultsView::Spatial;
            }
            ResultTreeAction::EigenMode(i) => {
                self.nav.focus_result = None;
                self.active_tab = Tab::Results;
                self.results_view = ResultsView::Spatial;
                self.view_mode = crate::viewer::ViewMode::Mode;
                self.view_mode_idx = i;
            }
            ResultTreeAction::Pushover(dir) => {
                self.nav.focus_result = None;
                self.set_pushover_view_dir(dir);
                self.active_tab = Tab::Results;
                self.results_view = ResultsView::Pushover;
            }
            ResultTreeAction::TimeHistoryCase(id) => {
                if let Some(res) = self
                    .results
                    .as_ref()
                    .and_then(|b| b.time_history_for(id))
                    .cloned()
                {
                    self.nav.focus_result = None;
                    self.set_spatial_time_history_view(id, &res);
                    self.active_tab = Tab::Results;
                    self.results_view = ResultsView::TimeHistory;
                    self.view_mode = crate::viewer::ViewMode::TimeHistory;
                }
            }
            ResultTreeAction::LumpedEigenMode(i) => {
                self.select_lumped_eigen_mode(i);
            }
            ResultTreeAction::LumpedTimeHistoryCase(id) => {
                if let Some(res) = self
                    .results
                    .as_ref()
                    .and_then(|b| b.lumped_result_for(id))
                    .cloned()
                {
                    self.nav.focus_result = None;
                    self.set_lumped_mass_view(Some(id), &res);
                    self.active_tab = Tab::Results;
                    self.results_view = ResultsView::LumpedMass;
                    self.view_mode = crate::viewer::ViewMode::LumpedTimeHistory;
                }
            }
        }
    }
}
