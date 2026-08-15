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
        self.wave_library_selection
            .clone()
            .unwrap_or_else(|| "サンプル".to_string())
    }

    /// 質点系時刻歴実行時の波形表示名。
    pub(crate) fn lumped_th_wave_name(&self) -> String {
        self.lumped_wave_library_selection
            .clone()
            .unwrap_or_else(|| "サンプル".to_string())
    }

    /// 立体時刻歴の表示窓口を切り替える（`time_history_data` も更新）。
    pub(crate) fn set_spatial_time_history_view(
        &mut self,
        case_id: VibrationCaseId,
        res: &squid_n_solver::timehistory::ResponseResult,
    ) {
        self.view_vibration_case = Some(case_id);
        if let Some(bundle) = &mut self.results {
            bundle.time_history = Some(res.clone());
        }
        #[cfg(feature = "gui")]
        {
            self.time_history_data = crate::time_history_view::TimeHistoryData {
                time: res.time.clone(),
                node_disp: res.history.node_disp.clone(),
                story_shear: res.history.base_shear.clone(),
                story_drift_angle: res.history.top_drift_angle.clone(),
                node: res.history.node,
            };
        }
    }

    /// 質点系結果の表示窓口を切り替える。
    pub(crate) fn set_lumped_mass_view(
        &mut self,
        case_id: Option<LumpedVibrationCaseId>,
        result: &squid_n_solver::lumped_mass::LumpedMassResult,
    ) {
        self.view_lumped_vibration_case = case_id;
        self.stick_response = result.response.clone();
        if let Some(bundle) = &mut self.results {
            bundle.lumped = Some(result.clone());
        }
    }

    /// 立体振動ケースを選択（ナビ「振動荷重ケース」用）。
    #[cfg(feature = "gui")]
    pub(crate) fn focus_spatial_vibration_case(&mut self, id: VibrationCaseId) {
        self.nav.focus_vibration_case = Some(id);
        self.nav.focus_lumped_vibration_case = None;
        self.nav.focus_load_case = None;
        if let Some(res) = self
            .results
            .as_ref()
            .and_then(|b| b.time_history_for(id))
            .cloned()
        {
            self.set_spatial_time_history_view(id, &res);
            self.active_tab = Tab::Results;
            #[cfg(feature = "gui")]
            {
                self.results_view = ResultsView::TimeHistory;
                self.view_mode = crate::viewer::ViewMode::TimeHistory;
            }
        }
    }

    /// 質点系振動ケースを選択（ナビ「振動荷重ケース」用）。
    #[cfg(feature = "gui")]
    pub(crate) fn focus_lumped_vibration_case(&mut self, id: LumpedVibrationCaseId) {
        self.nav.focus_lumped_vibration_case = Some(id);
        self.nav.focus_vibration_case = None;
        self.nav.focus_load_case = None;
        if let Some(res) = self
            .results
            .as_ref()
            .and_then(|b| b.lumped_result_for(id))
            .cloned()
        {
            self.set_lumped_mass_view(Some(id), &res);
            self.active_tab = Tab::Results;
            #[cfg(feature = "gui")]
            {
                self.results_view = ResultsView::LumpedMass;
            }
        }
    }
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

    /// 旧 `.scz`（`time_history` のみ）を振動ケース＋スロットへ移す。
    pub fn migrate_legacy_time_history(
        &mut self,
        model: &mut squid_n_core::model::Model,
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
        let id = model.upsert_vibration_case("サンプル".into(), dir, nonlinear);
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
