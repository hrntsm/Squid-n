//! ジョブ実行（線形静的・固有値・増分解析・時刻歴・断面算定）関数。

use super::*;
use squid_n_job::{AnalysisSettings, JobError};
use squid_n_load::ai::SoilClass;
use squid_n_solver::statics::analysis::AiMode;

mod design_check;
mod eigen;
mod linear_static;
mod pushover;
mod time_history;
mod ultimate;

use design_check::compute_design_check_job;
use eigen::compute_eigen_job;
use linear_static::compute_linear_static_job;
use pushover::compute_pushover_job;
use time_history::compute_time_history_job;
use ultimate::compute_ultimate_check_job;

/// `analysis_run` の任意パラメータの解決後の値。
/// `duration` の既定は GUI（10.0 秒）より短い 2.0 秒とする。
#[derive(Debug, Clone, Copy)]
pub struct JobParams {
    /// 対象荷重ケース ID（未指定なら先頭ケース）。
    pub load_case: Option<u32>,
    /// モード数。
    pub n_modes: usize,
    /// 加力・入力方向。
    pub dir: JobDir,
    /// 最大ステップ数。
    pub steps: usize,
    /// 目標変位 [mm]。
    pub max_disp: Option<f64>,
    /// 目標最大層間変形角の分母 n（既定 150）。
    pub max_drift_denom: Option<f64>,
    /// サンプル波の時間刻み [s]。
    pub dt: f64,
    /// サンプル波の継続時間 [s]。
    pub duration: f64,
    /// サンプル波の周期 [s]。
    pub period: f64,
    /// サンプル波の振幅 [mm/s²]。
    pub amp: f64,
    /// 地域係数 Z。
    pub z: f64,
    /// 地盤種別。
    pub soil: SoilClass,
    /// 標準せん断力係数 C0。
    pub c0: f64,
    /// Ai 算定法。
    pub ai_mode: AiMode,
    /// 精算時の設計用基本周期 T [s]。
    pub design_period: Option<f64>,
}

impl Default for JobParams {
    fn default() -> Self {
        let s = AnalysisSettings::default();
        Self {
            load_case: None,
            n_modes: s.n_modes,
            dir: JobDir::X,
            steps: s.push_steps,
            max_disp: None,
            max_drift_denom: None,
            dt: s.th_dt,
            duration: 2.0,
            period: s.th_period,
            amp: s.th_amp,
            z: s.z,
            soil: s.soil,
            c0: s.c0,
            ai_mode: s.ai_mode,
            design_period: None,
        }
    }
}

impl JobParams {
    /// 解析前処理（荷重自動同期）に渡す `AnalysisSettings` を組み立てる。
    pub(crate) fn analysis_settings_for_prepare(&self) -> AnalysisSettings {
        AnalysisSettings {
            ai_mode: self.ai_mode,
            z: self.z,
            soil: self.soil,
            c0: self.c0,
            ..Default::default()
        }
    }

    /// `max_disp`/`max_drift_denom` から終了目標を組み立てる。
    /// 両方未指定なら層間変形角 1/150 のみ。
    pub(crate) fn pushover_target(&self) -> squid_n_solver::nonlinear::pushover::PushoverTarget {
        match (self.max_disp, self.max_drift_denom) {
            (None, None) => squid_n_solver::nonlinear::pushover::PushoverTarget::default(),
            (Some(max_disp), None) => {
                squid_n_solver::nonlinear::pushover::PushoverTarget::from_max_disp(max_disp)
            }
            (None, Some(denom)) => squid_n_solver::nonlinear::pushover::PushoverTarget {
                max_disp: None,
                max_drift_angle: Some(1.0 / denom.max(1.0)),
            },
            (Some(max_disp), Some(denom)) => squid_n_solver::nonlinear::pushover::PushoverTarget {
                max_disp: Some(max_disp),
                max_drift_angle: Some(1.0 / denom.max(1.0)),
            },
        }
    }
}

/// Pushover/TimeHistory の方向（"X"/"Y" のみ）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JobDir {
    X,
    Y,
}

/// 各 JobKind の compute 結果（結果ストア用の生データとサマリ JSON）。
pub enum JobOutcome {
    LinearStatic {
        case: u32,
        node_ids: Vec<u32>,
        disp: Vec<[f64; 6]>,
        member_force_rows: Vec<(u32, f64, [f64; 6])>,
        summary: serde_json::Value,
    },
    Eigen {
        period: Vec<f64>,
        omega2: Vec<f64>,
        participation: Vec<[f64; 3]>,
        effective_mass: Vec<[f64; 3]>,
        summary: serde_json::Value,
    },
    Pushover {
        summary: serde_json::Value,
    },
    TimeHistory {
        summary: serde_json::Value,
    },
    DesignCheck {
        case: u32,
        member_force_rows: Vec<(u32, f64, [f64; 6])>,
        summary: serde_json::Value,
    },
    UltimateCheck {
        summary: serde_json::Value,
    },
}

/// `kind` に応じて対応する compute_* 関数へ振り分ける。
pub fn compute_job(
    model: &Model,
    kind: JobKind,
    params: &JobParams,
) -> Result<JobOutcome, JobError> {
    match kind {
        JobKind::LinearStatic => compute_linear_static_job(model, params),
        JobKind::Eigen => compute_eigen_job(model, params),
        JobKind::Pushover => compute_pushover_job(model.clone(), params),
        JobKind::TimeHistory => compute_time_history_job(model, params),
        JobKind::DesignCheck => compute_design_check_job(model, params),
        JobKind::UltimateCheck => compute_ultimate_check_job(model, params),
    }
}

/// 線形静的解析結果の部材力を `(elem_id, pos, forces)` 行へ平坦化する。
pub(crate) fn flatten_member_force_rows(
    member_forces: &[(
        squid_n_core::ids::ElemId,
        squid_n_element::frame::beam::MemberForces,
    )],
) -> Vec<(u32, f64, [f64; 6])> {
    let mut rows = Vec::new();
    for (elem_id, mf) in member_forces {
        for (pos, forces) in &mf.at {
            rows.push((elem_id.0, *pos, *forces));
        }
    }
    rows
}

/// `load_case` 指定があればそれを、なければ先頭の荷重ケースを返す。
pub(crate) fn resolve_load_case(
    model: &Model,
    load_case: Option<u32>,
) -> Result<&squid_n_core::model::LoadCase, JobError> {
    match load_case {
        Some(id) => model
            .load_cases
            .iter()
            .find(|c| c.id.0 == id)
            .ok_or_else(|| JobError::LoadCaseNotFound(format!("{id} が存在しません"))),
        None => model
            .load_cases
            .first()
            .ok_or_else(|| JobError::LoadCaseNotFound("モデルに 1 つもありません".to_string())),
    }
}

/// モデルを複製し、解析前処理（剛域・仕口パネル・荷重自動同期）を反映して返す。
/// 戻り値の第 2 要素は前処理の注意事項（サマリ JSON の `notices` へ載せる）。
pub(crate) fn model_prepared_for_analysis(
    model: &Model,
    params: &JobParams,
) -> (Model, Vec<String>) {
    let mut model = model.clone();
    let settings = params.analysis_settings_for_prepare();
    let report = squid_n_job::prepare::prepare_model_for_analysis(
        &mut model,
        &settings,
        params.design_period,
    );
    (model, report.notices)
}

/// サマリ JSON へ前処理の注意事項を載せる（空なら何もしない）。
pub(crate) fn attach_prepare_notices(summary: &mut serde_json::Value, notices: Vec<String>) {
    if notices.is_empty() {
        return;
    }
    if let serde_json::Value::Object(map) = summary {
        map.insert("notices".to_string(), serde_json::json!(notices));
    }
}
