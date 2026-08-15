//! 質点系解析。

use super::*;
use squid_n_solver::lumped_mass::LumpedMassResult;

impl App {
    fn lumped_static_and_pushover(
        &self,
    ) -> (
        Option<squid_n_solver::linear::StaticOnce>,
        Option<squid_n_solver::linear::StaticOnce>,
        Option<squid_n_solver::pushover::PushoverResult>,
        Option<squid_n_solver::pushover::PushoverResult>,
    ) {
        let res_x = self
            .results
            .as_ref()
            .and_then(|r| r.seismic(SeismicDir::X))
            .cloned();
        let res_y = self
            .results
            .as_ref()
            .and_then(|r| r.seismic(SeismicDir::Y))
            .cloned();
        let po_x = self.results.as_ref().and_then(|r| r.pushover_x.clone());
        let po_y = self.results.as_ref().and_then(|r| r.pushover_y.clone());
        (res_x, res_y, po_x, po_y)
    }

    /// 質点系固有値（時刻歴なし）をバックグラウンドで実行する。
    pub fn start_lumped_mass_eigen_job(&mut self) {
        self.start_lumped_mass_job(None, "質点系固有値");
    }

    /// 質点系時刻歴をバックグラウンドで実行する（固有値も同時に算定する）。
    pub fn start_lumped_mass_th_job(&mut self, accel: Vec<f64>) {
        self.start_lumped_mass_job(Some(accel), "質点系時刻歴");
    }

    fn start_lumped_mass_job(&mut self, accel: Option<Vec<f64>>, label: &'static str) {
        if !self.begin_analysis_job() {
            return;
        }
        let model = self.model.clone();
        let cfg = self.analysis_cfg;
        let (res_x, res_y, po_x, po_y) = self.lumped_static_and_pushover();
        self.spawn_analysis_job(label, move || {
            JobResult::LumpedMass(Box::new(Self::run_compute(|| {
                squid_n_job::compute::compute_lumped_mass(
                    model,
                    cfg,
                    res_x,
                    res_y,
                    po_x,
                    po_y,
                    accel.as_deref(),
                )
                .map_err(|e| e.to_string())
            })))
        });
        #[cfg(feature = "gui")]
        if let Some(job) = self.job.as_mut() {
            job.jump_on_success = Some((Tab::Results, ResultsView::LumpedMass));
        }
    }

    pub(super) fn apply_lumped_mass_result(&mut self, res: Result<LumpedMassResult, String>) {
        match res {
            Ok(result) => {
                self.stick_response = result.response.clone();
                let mut bundle = self.results.take().unwrap_or_default();
                bundle.lumped = Some(result);
                self.results = Some(bundle);
                self.staleness.last_run = Some(SystemTime::now());
            }
            Err(e) => self.report_error(e),
        }
    }

    /// 質点系サンプル波で時刻歴を開始する。
    pub fn start_lumped_mass_sample_th_job(&mut self) {
        let wave = squid_n_job::sample_lumped_ground_motion(&self.analysis_cfg);
        match squid_n_job::lumped_accel_from_wave(&wave, self.analysis_cfg.lumped_dir) {
            Ok(accel) => self.start_lumped_mass_th_job(accel),
            Err(e) => self.report_error(e),
        }
    }

    /// 波形ライブラリの選択波形で質点系時刻歴を開始する。
    #[cfg(feature = "gui")]
    pub fn start_lumped_mass_library_th_job(&mut self) {
        let Some(name) = self.lumped_wave_library_selection.clone() else {
            return;
        };
        let Some(dir) = squid_n_io::wave_library::wave_library_dir() else {
            self.report_error("波形ライブラリの保存先を特定できませんでした。".to_string());
            return;
        };
        let content = match std::fs::read_to_string(dir.join(&name)) {
            Ok(c) => c,
            Err(e) => {
                self.report_error(format!("波形読込エラー: {e}"));
                return;
            }
        };
        let mut cfg = self.analysis_cfg;
        cfg.th_dt = cfg.lumped_th_dt;
        cfg.th_dir = match cfg.lumped_dir {
            SeismicDir::X => ThDir::X,
            SeismicDir::Y => ThDir::Y,
        };
        let wave = match ground_motion_from_wave_content(&cfg, &content) {
            Ok(w) => w,
            Err(e) => {
                self.report_error(e);
                return;
            }
        };
        self.lumped_wave_library_selected_sha256 =
            squid_n_io::wave_library::wave_sha256(&dir, &name).ok();
        match squid_n_job::lumped_accel_from_wave(&wave, self.analysis_cfg.lumped_dir) {
            Ok(accel) => self.start_lumped_mass_th_job(accel),
            Err(e) => self.report_error(e),
        }
    }

    #[cfg(feature = "gui")]
    pub(crate) fn set_lumped_wave_library_selection(&mut self, name: Option<String>) {
        if name != self.lumped_wave_library_selection {
            self.lumped_wave_library_selection = name;
            self.lumped_wave_library_selected_sha256 = None;
        }
    }
}
