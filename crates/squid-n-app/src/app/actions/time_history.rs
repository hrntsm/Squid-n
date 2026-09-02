//! 時刻歴応答解析。
//!
//! `actions` からの構造分割。アルゴリズム変更は行わない。

use super::*;
use crate::app::vibration::vibration_th_dir_from_th;

impl App {
    /// `compute_time_history` の結果を適用する
    /// （振動ケース upsert・結果スロット更新・表示窓口更新）。
    pub(super) fn apply_time_history_result(
        &mut self,
        res: Result<squid_n_solver::timehistory::ResponseResult, String>,
    ) {
        match res {
            Ok(res) => {
                let wave_name = self.spatial_th_wave_name();
                let dir = vibration_th_dir_from_th(self.core.analysis_cfg.th_dir);
                let nonlinear = self.core.analysis_cfg.th_nonlinear;
                let case_id = self
                    .core
                    .model
                    .upsert_vibration_case(wave_name, dir, nonlinear);
                let mut bundle = self.core.scoped.results.take().unwrap_or_default();
                bundle.upsert_time_history(case_id, res.clone());
                self.core.scoped.results = Some(bundle);
                self.set_spatial_time_history_view(case_id, &res);
                self.core.scoped.staleness.mark_non_calc_edited();
                self.core.scoped.staleness.mark_fresh();
                self.core.scoped.last_error = None;
            }
            Err(e) => self.report_error(e),
        }
    }

    /// 線形時刻歴応答解析を実行する。減衰モデル・積分法は `analysis_cfg` に従う
    /// （剛性比例／Rayleigh、Newmark-β）。
    pub fn run_time_history(&mut self, wave: squid_n_solver::timehistory::GroundMotion) {
        self.begin_analysis();
        let res = squid_n_job::compute::compute_time_history(
            self.core.model.clone(),
            self.core.analysis_cfg,
            wave,
        )
        .map_err(|e| e.to_string());
        self.apply_time_history_result(res);
    }

    /// 時刻歴応答解析をバックグラウンドスレッドで実行する（P8 §5、残課題1）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_time_history_job(&mut self, wave: squid_n_solver::timehistory::GroundMotion) {
        if !self.begin_analysis_job() {
            return;
        }
        let model = self.core.model.clone();
        let cfg = self.core.analysis_cfg;
        // 非線形／線形の別をジョブラベル・完了ログへ出す（実行中の判別・履歴の両方で有用）。
        let label = if cfg.th_nonlinear {
            "時刻歴応答(非線形)"
        } else {
            "時刻歴応答(線形)"
        };
        self.spawn_analysis_job(label, move || {
            JobResult::TimeHistory(Box::new(Self::run_compute(|| {
                squid_n_job::compute::compute_time_history(model, cfg, wave)
                    .map_err(|e| e.to_string())
            })))
        });
        #[cfg(feature = "gui")]
        if let Some(job) = self.core.scoped.job.as_mut() {
            job.jump_on_success = Some((Tab::Results, ResultsView::TimeHistory));
        }
    }

    /// 正弦減衰のサンプル地震波を `cfg` から組み立てる
    /// （外部波形ファイルなしで機能を試せる導線。同期実行・ジョブ実行の双方で使う）。
    pub(crate) fn sample_wave(cfg: &AnalysisSettings) -> squid_n_solver::timehistory::GroundMotion {
        squid_n_job::sample_ground_motion(cfg)
    }

    /// 正弦減衰のサンプル地震波を生成して時刻歴解析を実行する（同期）。
    pub fn run_time_history_sample(&mut self) {
        self.apply_parallelism_setting();
        let wave = Self::sample_wave(&self.core.analysis_cfg);
        self.run_time_history(wave);
    }
}
