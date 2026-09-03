//! 振動荷重ケースと結果スロットの upsert・表示切替。

use super::*;
use squid_n_core::ids::{LumpedVibrationCaseId, VibrationCaseId};
use squid_n_core::model::{LumpedVibrationDim, LumpedVibrationDir, VibrationThDir};
use squid_n_solver::lumped_mass::StickDim;

pub(crate) fn vibration_th_dir_from_th(dir: ThDir) -> VibrationThDir {
    match dir {
        ThDir::X => VibrationThDir::X,
        ThDir::Y => VibrationThDir::Y,
        ThDir::Xy => VibrationThDir::Xy,
    }
}

pub(crate) fn lumped_vibration_dir_from_seismic(dir: SeismicDir) -> LumpedVibrationDir {
    match dir {
        SeismicDir::X => LumpedVibrationDir::X,
        SeismicDir::Y => LumpedVibrationDir::Y,
    }
}

pub(crate) fn lumped_vibration_dim_from_stick(dim: StickDim) -> LumpedVibrationDim {
    match dim {
        StickDim::Planar => LumpedVibrationDim::Planar,
        StickDim::Spatial => LumpedVibrationDim::Spatial,
    }
}

impl App {
    /// 立体時刻歴実行時の波形表示名（サンプル波は「サンプル」）。
    pub(crate) fn spatial_th_wave_name(&self) -> String {
        self.core
            .scoped
            .wave_library_selection
            .clone()
            .unwrap_or_else(|| "サンプル".to_string())
    }

    /// 質点系時刻歴実行時の波形表示名。
    pub(crate) fn lumped_th_wave_name(&self) -> String {
        self.core
            .scoped
            .lumped_wave_library_selection
            .clone()
            .unwrap_or_else(|| "サンプル".to_string())
    }

    /// 立体時刻歴の表示窓口を切り替える（`time_history_data` も更新）。
    pub(crate) fn set_spatial_time_history_view(
        &mut self,
        case_id: VibrationCaseId,
        res: &squid_n_solver::timehistory::ResponseResult,
    ) {
        self.core.scoped.view_vibration_case = Some(case_id);
        if let Some(bundle) = &mut self.core.scoped.results {
            bundle.time_history = Some(res.clone());
        }
        self.fill_time_history_data(res);
    }

    fn fill_time_history_data(&mut self, res: &squid_n_solver::timehistory::ResponseResult) {
        #[cfg(feature = "gui")]
        {
            self.ui.scoped.time_history_data = crate::time_history_view::TimeHistoryData {
                time: res.time.clone(),
                node_disp: res.history.node_disp.clone(),
                story_shear: res.history.base_shear.clone(),
                story_drift_angle: res.history.top_drift_angle.clone(),
                node: res.history.node,
            };
        }
        #[cfg(not(feature = "gui"))]
        let _ = res;
    }

    fn fill_time_history_from_window(&mut self) {
        let Some(res) = self
            .core
            .scoped
            .results
            .as_ref()
            .and_then(|b| b.time_history.clone())
        else {
            #[cfg(feature = "gui")]
            {
                self.ui.scoped.time_history_data =
                    crate::time_history_view::TimeHistoryData::default();
            }
            return;
        };
        self.fill_time_history_data(&res);
    }

    /// 保存済みの表示ケース ID から、時刻歴グラフと質点系窓口を復元する。
    pub(crate) fn hydrate_saved_vibration_views(
        &mut self,
        view_vibration_case: Option<VibrationCaseId>,
        view_lumped_vibration_case: Option<LumpedVibrationCaseId>,
    ) {
        match view_vibration_case {
            Some(id) => {
                if let Some(res) = self
                    .core
                    .scoped
                    .results
                    .as_ref()
                    .and_then(|b| b.time_history_for(id))
                    .cloned()
                {
                    self.set_spatial_time_history_view(id, &res);
                } else {
                    self.core.scoped.view_vibration_case = None;
                    self.fill_time_history_from_window();
                }
            }
            None => self.fill_time_history_from_window(),
        }

        match view_lumped_vibration_case {
            Some(id) => {
                if let Some(res) = self
                    .core
                    .scoped
                    .results
                    .as_ref()
                    .and_then(|b| b.lumped_result_for(id))
                    .cloned()
                {
                    self.set_lumped_mass_view(Some(id), &res);
                } else if let Some(res) = self
                    .core
                    .scoped
                    .results
                    .as_ref()
                    .and_then(|b| b.lumped.clone())
                {
                    self.set_lumped_mass_view(None, &res);
                } else {
                    self.core.scoped.view_lumped_vibration_case = None;
                    self.core.scoped.stick_response = None;
                }
            }
            None => {
                if let Some(res) = self
                    .core
                    .scoped
                    .results
                    .as_ref()
                    .and_then(|b| b.lumped.clone())
                {
                    self.set_lumped_mass_view(None, &res);
                }
            }
        }
    }

    /// 結果スロットが無い振動ケースをモデルから除く。
    ///
    /// 解析結果を同梱せずに保存した `.scz` を開いたとき、ナビに空ケースが
    /// 残らないようにする。未実行の空ケースは作らない、という規約に合わせる。
    pub(crate) fn prune_orphan_vibration_cases(&mut self) {
        let spatial: std::collections::HashSet<_> = self
            .core
            .scoped
            .results
            .as_ref()
            .map(|b| b.time_histories.iter().map(|(id, _)| *id).collect())
            .unwrap_or_default();
        let lumped: std::collections::HashSet<_> = self
            .core
            .scoped
            .results
            .as_ref()
            .map(|b| b.lumped_results.iter().map(|(id, _)| *id).collect())
            .unwrap_or_default();
        self.core
            .model
            .vibration_cases
            .retain(|c| spatial.contains(&c.id));
        self.core
            .model
            .lumped_vibration_cases
            .retain(|c| lumped.contains(&c.id));
    }

    /// 質点系結果の表示窓口を切り替える。
    pub(crate) fn set_lumped_mass_view(
        &mut self,
        case_id: Option<LumpedVibrationCaseId>,
        result: &squid_n_solver::lumped_mass::LumpedMassResult,
    ) {
        self.core.scoped.view_lumped_vibration_case = case_id;
        self.core.scoped.stick_response = result.response.clone();
        if let Some(bundle) = &mut self.core.scoped.results {
            bundle.lumped = Some(result.clone());
        }
    }

    /// 立体振動ケースを選択（ナビ「振動荷重ケース」用）。
    #[cfg(feature = "gui")]
    pub(crate) fn focus_spatial_vibration_case(&mut self, id: VibrationCaseId) {
        self.ui.scoped.nav.focus_vibration_case = Some(id);
        self.ui.scoped.nav.focus_lumped_vibration_case = None;
        self.ui.scoped.nav.focus_load_case = None;
        if let Some(res) = self
            .core
            .scoped
            .results
            .as_ref()
            .and_then(|b| b.time_history_for(id))
            .cloned()
        {
            self.set_spatial_time_history_view(id, &res);
            self.ui.view.active_tab = Tab::Results;
            #[cfg(feature = "gui")]
            {
                self.ui.view.results_view = ResultsView::TimeHistory;
                self.ui.view.view_mode = crate::viewer::ViewMode::TimeHistory;
            }
        }
    }

    /// 質点系振動ケースを選択（ナビ「振動荷重ケース」用）。
    #[cfg(feature = "gui")]
    pub(crate) fn focus_lumped_vibration_case(&mut self, id: LumpedVibrationCaseId) {
        self.ui.scoped.nav.focus_lumped_vibration_case = Some(id);
        self.ui.scoped.nav.focus_vibration_case = None;
        self.ui.scoped.nav.focus_load_case = None;
        if let Some(res) = self
            .core
            .scoped
            .results
            .as_ref()
            .and_then(|b| b.lumped_result_for(id))
            .cloned()
        {
            self.set_lumped_mass_view(Some(id), &res);
            self.ui.view.active_tab = Tab::Results;
            #[cfg(feature = "gui")]
            {
                self.ui.view.results_view = ResultsView::LumpedMass;
            }
        }
    }
}

/// 除外保存のために一時的に取り出した時刻歴の詳細記録。
pub(crate) struct TakenThRecordings {
    window: Option<squid_n_solver::timehistory::ThRecording>,
    slots: Vec<(usize, squid_n_solver::timehistory::ThRecording)>,
}

impl ResultsBundle {
    /// 立体時刻歴結果をケース ID で upsert する。
    pub fn upsert_time_history(
        &mut self,
        id: VibrationCaseId,
        result: squid_n_solver::timehistory::ResponseResult,
    ) {
        if let Some(pos) = self.time_histories.iter().position(|(i, _)| *i == id) {
            self.time_histories[pos].1 = result;
        } else {
            self.time_histories.push((id, result));
        }
    }

    pub fn time_history_for(
        &self,
        id: VibrationCaseId,
    ) -> Option<&squid_n_solver::timehistory::ResponseResult> {
        self.time_histories
            .iter()
            .find(|(i, _)| *i == id)
            .map(|(_, r)| r)
    }

    /// 質点系結果をケース ID で upsert する。
    pub fn upsert_lumped_result(
        &mut self,
        id: LumpedVibrationCaseId,
        result: squid_n_solver::lumped_mass::LumpedMassResult,
    ) {
        if let Some(pos) = self.lumped_results.iter().position(|(i, _)| *i == id) {
            self.lumped_results[pos].1 = result;
        } else {
            self.lumped_results.push((id, result));
        }
    }

    pub fn lumped_result_for(
        &self,
        id: LumpedVibrationCaseId,
    ) -> Option<&squid_n_solver::lumped_mass::LumpedMassResult> {
        self.lumped_results
            .iter()
            .find(|(i, _)| *i == id)
            .map(|(_, r)| r)
    }

    /// 表示窓口またはケース別スロットに時刻歴の詳細記録があるか。
    pub(crate) fn has_th_recording(&self) -> bool {
        self.time_history
            .as_ref()
            .is_some_and(|th| th.recording.is_some())
            || self
                .time_histories
                .iter()
                .any(|(_, th)| th.recording.is_some())
    }

    /// 詳細記録を一時的に外す（除外保存用。直後に [`Self::restore_th_recordings`]）。
    pub(crate) fn take_th_recordings(&mut self) -> TakenThRecordings {
        let window = self
            .time_history
            .as_mut()
            .and_then(|th| th.recording.take());
        let slots = self
            .time_histories
            .iter_mut()
            .enumerate()
            .filter_map(|(i, (_, th))| th.recording.take().map(|r| (i, r)))
            .collect();
        TakenThRecordings { window, slots }
    }

    pub(crate) fn restore_th_recordings(&mut self, taken: TakenThRecordings) {
        if let Some(rec) = taken.window {
            if let Some(th) = self.time_history.as_mut() {
                th.recording = Some(rec);
            }
        }
        for (i, rec) in taken.slots {
            if let Some((_, th)) = self.time_histories.get_mut(i) {
                th.recording = Some(rec);
            }
        }
    }

    /// 旧 `.scz`（`time_history` のみ）を振動ケース＋スロットへ移す。
    pub fn migrate_legacy_time_history(
        &mut self,
        model: &mut squid_n_core::model::Model,
        wave_name: &str,
        dir: VibrationThDir,
        nonlinear: bool,
    ) {
        if !self.time_histories.is_empty() {
            if self.time_history.is_none() {
                if let Some((_, r)) = self.time_histories.last() {
                    self.time_history = Some(r.clone());
                }
            }
            return;
        }
        let Some(th) = self.time_history.take() else {
            return;
        };
        let id = model.upsert_vibration_case(wave_name.to_string(), dir, nonlinear);
        self.time_histories.push((id, th.clone()));
        self.time_history = Some(th);
    }

    /// 旧 `.scz`（質点系 `lumped` のみ・時刻歴あり）を振動ケース＋スロットへ移す。
    pub fn migrate_legacy_lumped(
        &mut self,
        model: &mut squid_n_core::model::Model,
        wave_name: &str,
        dir: LumpedVibrationDir,
        nonlinear: bool,
        dim: LumpedVibrationDim,
    ) {
        if !self.lumped_results.is_empty() {
            if self.lumped.is_none() {
                if let Some((_, r)) = self.lumped_results.last() {
                    self.lumped = Some(r.clone());
                }
            }
            return;
        }
        let Some(lumped) = self.lumped.take() else {
            return;
        };
        if lumped.response.is_some() {
            let id = model.upsert_lumped_vibration_case(wave_name.to_string(), dir, nonlinear, dim);
            self.lumped_results.push((id, lumped.clone()));
            self.lumped = Some(lumped);
        } else {
            self.lumped = Some(lumped);
        }
    }
}
