//! 時刻歴応答解析。
//!
//! `actions` からの構造分割。アルゴリズム変更は行わない。

use super::*;

impl App {
    /// `compute_time_history` の結果を適用する
    /// （bundle 格納・time_history_data 更新(gui)・最終実行時刻更新・エラー設定）。
    pub(super) fn apply_time_history_result(
        &mut self,
        res: Result<squid_n_solver::timehistory::ResponseResult, String>,
    ) {
        match res {
            Ok(res) => {
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
                let mut bundle = self.results.take().unwrap_or_default();
                bundle.time_history = Some(res);
                self.results = Some(bundle);
                // mark_fresh で stale を解消する（`apply_pushover_result` と同じ理由。
                // last_run の更新だけでは、編集後に時刻歴だけを実行しても
                // アニメーション・部材クリック・詳細ウィンドウが stale 判定で
                // 無効化されたまま復帰しなかった）。
                self.staleness.mark_fresh();
                self.last_error = None;
            }
            Err(e) => self.report_error(e),
        }
    }

    /// 線形時刻歴応答解析を実行する。減衰モデル・積分法は `analysis_cfg` に従う
    /// （剛性比例／Rayleigh、Newmark-β／HHT-α）。
    pub fn run_time_history(&mut self, wave: squid_n_solver::timehistory::GroundMotion) {
        self.begin_analysis();
        let res =
            squid_n_job::compute::compute_time_history(self.model.clone(), self.analysis_cfg, wave)
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
        let model = self.model.clone();
        let cfg = self.analysis_cfg;
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
        if let Some(job) = self.job.as_mut() {
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
        let wave = Self::sample_wave(&self.analysis_cfg);
        self.run_time_history(wave);
    }
}
