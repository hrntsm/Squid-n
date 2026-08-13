//! 固有値解析。
//!
//! `actions` からの構造分割。アルゴリズム変更は行わない。

use super::*;

impl App {
    /// T3: 固有値解析を実行し、結果を `self.results` に格納する（同期）。
    pub fn run_eigen(&mut self, n_modes: usize) {
        self.begin_analysis();
        let res = squid_n_job::compute::compute_eigen(self.model.clone(), n_modes)
            .map_err(|e| e.to_string());
        self.apply_eigen_result(res);
    }

    /// 固有値解析をバックグラウンドスレッドで実行する（解析パネル「▶ 実行」の
    /// 入口）。かつて固有値だけは UI スレッドで同期実行しており、モード数の
    /// 多い固有値解析中にアプリが無応答になっていた。
    pub fn start_eigen_job(&mut self, n_modes: usize) {
        if !self.begin_analysis_job() {
            return;
        }
        let model = self.model.clone();
        self.spawn_analysis_job("固有値解析", move || {
            JobResult::Modal(Self::run_compute(|| {
                squid_n_job::compute::compute_eigen(model, n_modes).map_err(|e| e.to_string())
            }))
        });
    }

    /// `compute_eigen` の結果を適用する（bundle 格納・最終実行時刻更新）。
    pub(super) fn apply_eigen_result(
        &mut self,
        res: Result<squid_n_solver::eigen::ModalResult, String>,
    ) {
        match res {
            Ok(modal) => {
                let mut bundle = self.results.take().unwrap_or_default();
                bundle.modal = Some(modal);
                self.results = Some(bundle);
                // 固有値のみの更新では設計は更新されないが、最新実行時刻は更新
                self.staleness.last_run = Some(SystemTime::now());
            }
            Err(e) => self.report_error(e),
        }
    }
}
