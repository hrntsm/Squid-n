use super::super::*;
use squid_n_core::units::to_display::length_m;

impl App {
    /// 地震荷重(Ai分布)の設計用固有周期 T[s] を、暗黙の解析なしで決定する。
    ///
    /// - `AiMode::Approx`: 略算式 T = h(0.02+0.01α)（令88条・昭和55年建設省
    ///   告示第1793号）。即時計算で解析は不要。
    /// - `AiMode::SemiPrecise`: 明示実行済みの固有値解析結果
    ///   （`self.core.scoped.results` の `modal`）の1次周期（`ModalResult::period[0]`）を
    ///   再利用する。固有値解析が未実行（`results` がない・`modal` がない・
    ///   `period` が空のいずれか）の場合は `Err`（実行を促す日本語メッセージ）。
    ///
    /// `Analysis::prepare`（剛性行列組立+Cholesky分解）や固有値解析を新たに
    /// 実行することはない（暗黙の重い解析を避けるための入口）。
    pub(crate) fn design_seismic_period(&self) -> Result<f64, String> {
        match self.core.analysis_cfg.ai_mode {
            AiMode::Approx => {
                let height_m = length_m(squid_n_solver::statics::analysis::building_height_mm(
                    &self.core.model,
                ));
                let steel_ratio =
                    squid_n_solver::statics::analysis::steel_height_ratio(&self.core.model);
                Ok(squid_n_load::ai::approx_t(height_m, steel_ratio))
            }
            AiMode::SemiPrecise => self
                .core
                .scoped
                .results
                .as_ref()
                .and_then(|r| r.modal.as_ref())
                .and_then(|m| m.period.first().copied())
                .ok_or_else(|| {
                    "精算周期(固有値解析)が選択されていますが固有値解析が未実行です。\
                     解析タブの固有値解析を先に実行してください\
                     (EX/EY の地震荷重は更新されません)。"
                        .to_string()
                }),
        }
    }
}
