//! 増分解析（プッシュオーバー）。
//!
//! `actions` からの構造分割。アルゴリズム変更は行わない。

use super::*;

impl App {
    /// 方向別スロットの増分解析結果を返す。
    pub fn pushover_for(
        &self,
        dir: SeismicDir,
    ) -> Option<&squid_n_solver::nonlinear::pushover::PushoverResult> {
        let results = self.core.scoped.results.as_ref()?;
        results.pushover_for_dir(dir)
    }

    /// 結果タブ・設計タブで表示中の増分解析結果を返す。
    pub fn displayed_pushover(
        &self,
    ) -> Option<&squid_n_solver::nonlinear::pushover::PushoverResult> {
        let view_dir = self.core.scoped.pushover_view_dir;
        let results = self.core.scoped.results.as_ref()?;
        results
            .pushover_for_dir(view_dir)
            .or(results.pushover.as_ref())
    }

    /// 結果タブの増分解析表示方向を切り替え、`pushover` 窓口も同期する。
    #[cfg(any(test, feature = "gui"))]
    pub(crate) fn set_pushover_view_dir(&mut self, dir: SeismicDir) {
        if self.core.scoped.pushover_view_dir == dir {
            return;
        }
        self.core.scoped.pushover_view_dir = dir;
        if let Some(bundle) = self.core.scoped.results.as_mut() {
            if let Some(po) = bundle.pushover_for_dir(dir).cloned() {
                bundle.pushover = Some(po);
            }
        }
    }

    /// 保存直前に `pushover` 窓口を表示中方向へ同期する。
    pub(crate) fn sync_pushover_for_save(&mut self) {
        if let Some(bundle) = self.core.scoped.results.as_mut() {
            bundle.pushover = bundle
                .pushover_for_dir(self.core.scoped.pushover_view_dir)
                .cloned();
        }
    }

    /// `compute_pushover` の結果を適用する（bundle 格納・最終実行時刻更新・エラー設定）。
    pub(super) fn apply_pushover_result(
        &mut self,
        res: Result<squid_n_solver::nonlinear::pushover::PushoverResult, String>,
    ) {
        match res {
            Ok(result) => {
                // 目標未到達の打ち切り（非収束・特異化）は Qu が過小評価の可能性が
                // あるため、結果画面の警告に加えてログにも残す。
                if result.termination.is_premature() {
                    self.report_notice(format!(
                        "⚠ 増分解析は目標到達前に打ち切られました（{}）。Qu はその時点までの最大値です。",
                        result.termination.describe()
                    ));
                }
                let dir = self.core.analysis_cfg.push_dir;
                let mut bundle = self.core.scoped.results.take().unwrap_or_default();
                match dir {
                    SeismicDir::X => bundle.pushover_x = Some(result.clone()),
                    SeismicDir::Y => bundle.pushover_y = Some(result.clone()),
                }
                bundle.pushover = Some(result);
                self.core.scoped.results = Some(bundle);
                self.core.scoped.pushover_view_dir = dir;
                // mark_fresh で stale を解消する（`apply_static_case_result` と同じ扱い）。
                // last_run の更新だけでは results_stale が立ったままになり、編集後に
                // 増分解析だけを実行してもビューアが「再実行してください」表示のまま
                // 復帰しなかった。
                self.core.scoped.staleness.mark_fresh();
                self.core.scoped.last_error = None;
            }
            Err(e) => self.report_error(e),
        }
    }

    /// 増分解析（プッシュオーバー）を実行する。モデルは複製の上で解析する
    /// （非線形状態の副作用を GUI 上のモデルへ残さないため）。
    /// 鋼板耐震壁を含むモデルの増分解析で、せん断座屈を考慮していない旨を知らせる。
    ///
    /// 鋼板耐震壁の面内せん断終局強度は鋼板のせん断降伏 Qy=t·lw·F/√3 で評価している
    /// （`squid_n_element::wall::wall_element::WallElement::steel_shear_capacity_of`）。
    /// 幅厚比の大きい無補剛の鋼板は降伏前に面外へせん断座屈するため、その場合は
    /// 耐力を過大評価する（危険側）。解析は継続してよい事項のため注意事項として扱う。
    fn notice_steel_seismic_walls(&mut self) {
        let n = self
            .core
            .model
            .elements
            .iter()
            .filter(|e| {
                matches!(e.kind, squid_n_core::model::ElementKind::Wall)
                    && squid_n_element::wall::misc_wall::wall_is_seismic(e, &self.core.model)
                    && !squid_n_element::wall::misc_wall::is_rc_wall(e, &self.core.model)
            })
            .count();
        if n == 0 {
            return;
        }
        self.report_notice(format!(
            "鋼板耐震壁 {} 枚の面内せん断終局強度を、鋼板のせん断降伏 Qy=t·lw·F/√3 で評価します。\
             せん断座屈は考慮していないため、幅厚比が大きく補剛のない鋼板では耐力を過大評価します。",
            n
        ));
    }

    pub fn run_pushover(&mut self) {
        self.begin_analysis();
        self.notice_steel_seismic_walls();
        let res =
            squid_n_job::compute::compute_pushover(self.core.model.clone(), self.core.analysis_cfg)
                .map_err(|e| e.to_string());
        self.apply_pushover_result(res);
    }

    /// 増分解析（プッシュオーバー）をバックグラウンドスレッドで実行する（P8 §5、残課題1）。
    /// UI スレッドをブロックしないよう重い解析を逃がす。
    /// 既にジョブが実行中の場合は何もしない（last_error に案内文を設定）。
    pub fn start_pushover_job(&mut self) {
        if !self.begin_analysis_job() {
            return;
        }
        self.notice_steel_seismic_walls();
        let model = self.core.model.clone();
        let cfg = self.core.analysis_cfg;
        self.spawn_analysis_job("増分解析", move || {
            JobResult::Pushover(Self::run_compute(|| {
                squid_n_job::compute::compute_pushover(model, cfg).map_err(|e| e.to_string())
            }))
        });
        #[cfg(feature = "gui")]
        if let Some(job) = self.core.scoped.job.as_mut() {
            job.jump_on_success = Some((Tab::Results, ResultsView::Pushover));
        }
    }
}
