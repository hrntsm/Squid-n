//! ジョブ実行（線形静的・固有値・増分解析・時刻歴・断面算定）関数。
//!
//! # モジュール構成（1 ファイル 1 責務）
//! - [`linear_static`] — LinearStatic ジョブの純粋計算部分。
//! - [`eigen`] — Eigen ジョブの純粋計算部分。
//! - [`pushover`] — Pushover ジョブの純粋計算部分。
//! - [`time_history`] — TimeHistory ジョブの純粋計算部分。
//! - [`design_check`] — DesignCheck ジョブ（断面検定・接合部検定）の純粋計算部分。
//! - [`ultimate`] — UltimateCheck ジョブ（終局検定）の純粋計算部分。

use super::*;

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

/// `analysis_run` の任意パラメータの解決後の値（`AnalysisRunArgs` から変換する）。
/// 既定値は GUI (`squid_n_app::app::AnalysisSettings`) の既定に合わせる。
/// ただし `duration` は GUI 既定の 10.0 秒だと MCP 経由の応答待ちが長くなるため、
/// 動作確認がしやすい 2.0 秒を既定とする（呼び出し側で明示すれば変更可）。
#[derive(Debug, Clone, Copy)]
pub struct JobParams {
    /// LinearStatic/DesignCheck: 対象荷重ケース ID（未指定なら先頭ケース）。
    pub load_case: Option<u32>,
    /// Eigen: モード数。
    pub n_modes: usize,
    /// Pushover/TimeHistory: 加力・入力方向。
    pub dir: JobDir,
    /// Pushover: 最大ステップ数。
    pub steps: usize,
    /// Pushover: 目標変位 [mm]。未指定なら層間変形角のみで判定。
    pub max_disp: Option<f64>,
    /// Pushover: 目標最大層間変形角の分母 n（角度 1/n rad）。既定 150。
    pub max_drift_denom: Option<f64>,
    /// TimeHistory: サンプル波の時間刻み [s]。
    pub dt: f64,
    /// TimeHistory: サンプル波の継続時間 [s]。
    pub duration: f64,
    /// TimeHistory: サンプル波の周期 [s]。
    pub period: f64,
    /// TimeHistory: サンプル波の振幅 [mm/s²]。
    pub amp: f64,
}

impl Default for JobParams {
    fn default() -> Self {
        Self {
            load_case: None,
            n_modes: 3,
            dir: JobDir::X,
            steps: 50,
            max_disp: None,
            max_drift_denom: None,
            dt: 0.01,
            duration: 2.0,
            period: 0.5,
            amp: 1000.0,
        }
    }
}

impl JobParams {
    /// `max_disp`/`max_drift_denom` から `squid_n_solver::pushover::PushoverTarget` を
    /// 組み立てる。
    ///
    /// - 両方未指定 → `PushoverTarget::default()`（層間変形角 1/150 のみ。GUI 既定と同じ）。
    /// - `max_disp` のみ指定 → 目標変位のみ有効。
    /// - `max_drift_denom` のみ指定 → 目標層間変形角 1/n のみ有効。
    /// - 両方指定 → 両方有効（早く達した方で終了）。
    pub(crate) fn pushover_target(&self) -> squid_n_solver::pushover::PushoverTarget {
        match (self.max_disp, self.max_drift_denom) {
            (None, None) => squid_n_solver::pushover::PushoverTarget::default(),
            (Some(max_disp), None) => {
                squid_n_solver::pushover::PushoverTarget::from_max_disp(max_disp)
            }
            (None, Some(denom)) => squid_n_solver::pushover::PushoverTarget {
                max_disp: None,
                max_drift_angle: Some(1.0 / denom.max(1.0)),
            },
            (Some(max_disp), Some(denom)) => squid_n_solver::pushover::PushoverTarget {
                max_disp: Some(max_disp),
                max_drift_angle: Some(1.0 / denom.max(1.0)),
            },
        }
    }
}

/// Pushover/TimeHistory の方向（"X"/"Y"）。X+Y 同時入力（GUI の `ThDir::Xy`）は
/// MCP 経由では対応しない（仕様どおり "X"/"Y" のみ）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JobDir {
    X,
    Y,
}

/// 各 JobKind の compute 結果。結果ストアへ書くべき生データ（あれば）とサマリ
/// （`JobStatus::Done::result_ref` に格納する JSON）の両方を保持する。
/// ストアへの書き込みは `persist_job_outcome` が担う（`ServerState` のロック内で
/// 呼ぶ必要があるため、compute 側とは分離している）。
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
pub fn compute_job(model: &Model, kind: JobKind, params: &JobParams) -> Result<JobOutcome, String> {
    match kind {
        JobKind::LinearStatic => compute_linear_static_job(model, params.load_case),
        JobKind::Eigen => compute_eigen_job(model, params.n_modes),
        JobKind::Pushover => compute_pushover_job(
            model.clone(),
            params.dir,
            params.steps,
            params.pushover_target(),
        ),
        JobKind::TimeHistory => compute_time_history_job(
            model,
            params.dir,
            params.dt,
            params.duration,
            params.period,
            params.amp,
        ),
        JobKind::DesignCheck => compute_design_check_job(model, params.load_case),
        JobKind::UltimateCheck => compute_ultimate_check_job(model, params.load_case),
    }
}

/// `load_case` 指定があればそれを、無ければ先頭の荷重ケースを返す。
/// 荷重ケースが1つも無いモデルでは "no load cases" を返す
/// （既存の `analyze_model` と同じ文言。P8 のテストが this を確認している）。
pub(crate) fn resolve_load_case(
    model: &Model,
    load_case: Option<u32>,
) -> Result<&squid_n_core::model::LoadCase, String> {
    match load_case {
        Some(id) => model
            .load_cases
            .iter()
            .find(|c| c.id.0 == id)
            .ok_or_else(|| format!("荷重ケース {id} が存在しません")),
        None => model
            .load_cases
            .first()
            .ok_or_else(|| "no load cases".to_string()),
    }
}

/// モデルを複製し、標準の自動剛域（設計書 §6.2.1）を反映して返す。
/// clone → apply_auto_rigid_zones(default) の定型を集約する。`Analysis` は
/// モデルを借用するため、準備は呼出側で `Analysis::prepare(&model)` を行う。
pub(crate) fn model_with_auto_rigid_zones(model: &Model) -> Model {
    let mut model = model.clone();
    squid_n_element::beam::apply_auto_rigid_zones(
        &mut model,
        &squid_n_element::beam::RigidZoneRule::default(),
    );
    model
}
