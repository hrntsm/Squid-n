//! 各解析の純粋計算（所有モデル＋解析条件 → 結果）。
//!
//! いずれも `&self` を取らない自由関数で、GUI の状態に触れない。GUI
//! （`squid-n-app`）はバックグラウンドスレッドから、MCP サーバ（`squid-n-mcp`）は
//! ジョブから、**同じ関数**を同じ解析条件で呼ぶ。
//!
//! 渡すモデルは [`crate::prepare`] で前処理済みであること（剛域・仕口パネル・
//! 荷重ケースの自動同期）。前処理を経ないモデルを解くと、仕口パネルのない
//! 剛性で解いたり、地震力ゼロで増分解析したりすることになる。

use crate::error::{JobError, JobResult};
use crate::settings::{AnalysisSettings, ThDampingModel, ThIntegrator};
use squid_n_core::ids::LoadCaseId;
use squid_n_solver::analysis::Analysis;

/// 線形静的解析。前処理（[`crate::prepare`]）を通したモデルを渡す前提で、
/// ここでは再適用しない（二重適用を避ける）。
pub fn compute_linear_static(
    model: squid_n_core::model::Model,
    lc: LoadCaseId,
) -> JobResult<squid_n_solver::linear::StaticOnce> {
    match Analysis::prepare(&model) {
        Ok(analysis) => analysis
            .linear_static(lc)
            .map_err(|e| JobError::Solve(format!("{e:?}"))),
        Err(e) => Err(JobError::Prepare(format!("{e:?}"))),
    }
}

/// 固有値解析。前処理を通したモデルを渡す前提。
pub fn compute_eigen(
    model: squid_n_core::model::Model,
    n_modes: usize,
) -> JobResult<squid_n_solver::eigen::ModalResult> {
    match Analysis::prepare(&model) {
        Ok(analysis) => analysis
            .eigen(n_modes)
            .map_err(|e| JobError::Solve(format!("{e:?}"))),
        Err(e) => Err(JobError::Prepare(format!("{e:?}"))),
    }
}

/// 地震静的（Ai 分布）解析。前処理を通したモデルを渡す前提。
pub fn compute_seismic(
    model: squid_n_core::model::Model,
    cfg: squid_n_solver::analysis::SeismicCfg,
    t: f64,
) -> JobResult<squid_n_solver::linear::StaticOnce> {
    match Analysis::prepare(&model) {
        Ok(analysis) => analysis
            .seismic_static_with_period(cfg, t)
            .map_err(|e| JobError::Solve(format!("{e:?}"))),
        Err(e) => Err(JobError::Prepare(format!("{e:?}"))),
    }
}

/// 位相差入力（ねじれ加振）を `wave` へ付加する（構造動力学の位相差入力解析）。
/// `phase_diff_enabled` が false なら `wave` をそのまま返す。位相遅れ時間
/// `t=(L·sinθ)/Vs` を求め、位相遅れ方向の並進波からねじれ地動加速度を生成する。
fn apply_phase_diff(
    cfg: &AnalysisSettings,
    mut wave: squid_n_solver::timehistory::GroundMotion,
) -> squid_n_solver::timehistory::GroundMotion {
    if !cfg.phase_diff_enabled {
        return wave;
    }
    use squid_n_solver::phase_diff::{phase_lag_time, torsional_accel_series};
    let lag = phase_lag_time(
        cfg.phase_diff_length_m,
        cfg.phase_diff_incidence_deg,
        cfg.phase_diff_vs,
    );
    // 位相遅れ方向の並進加速度を基準波とする。
    let base: Vec<f64> = if cfg.phase_diff_dir_y {
        wave.accel_y.clone().unwrap_or_else(|| wave.accel_x.clone())
    } else {
        wave.accel_x.clone()
    };
    let l_mm = (cfg.phase_diff_length_m * 1000.0).max(1.0);
    let theta = torsional_accel_series(&base, wave.dt, lag, l_mm);
    wave.accel_theta = Some(theta);
    wave
}

/// 増分解析（プッシュオーバー）。モデルは呼び出し側で複製したものを渡す
/// （非線形状態の副作用を呼び出し側のモデルへ残さないため）。
///
/// 剛域・仕口パネル・荷重同期は [`crate::prepare`] で適用済みのモデルを渡す前提
/// （静的解析と同じ規約。かつては剛域だけをインライン適用する経路があり、
/// 仕口パネルの生成が省かれて静的解析と剛性の異なるモデルを解いていた）。
pub fn compute_pushover(
    model: squid_n_core::model::Model,
    cfg: AnalysisSettings,
) -> JobResult<squid_n_solver::pushover::PushoverResult> {
    let work = model;
    Analysis::prepare(&work).map_err(|e| JobError::Prepare(e.to_string()))?;
    let dofmap = squid_n_core::dof::DofMap::build(&work);
    let reducer = squid_n_solver::constraint::Reducer::build(&work, &dofmap);
    // 終了目標（目標変位・目標最大層間変形角）。両方無効なら荷重制御 λ=1 まで解析する。
    let target = squid_n_solver::pushover::PushoverTarget {
        max_disp: cfg.push_use_max_disp.then_some(cfg.push_max_disp),
        max_drift_angle: cfg
            .push_use_drift_angle
            .then_some(1.0 / cfg.push_drift_denom.max(1.0)),
    };
    squid_n_solver::pushover::pushover_analysis_recording(
        &work,
        &dofmap,
        &reducer,
        cfg.push_dir,
        cfg.push_steps,
        target,
        cfg.push_control,
        cfg.push_apply_long_term,
        false,
        false,
        0.0,
        cfg.ductility_method,
    )
    .map_err(|e| JobError::Convergence(e.to_string()))
}

/// 時刻歴応答解析。減衰モデル・積分法は `cfg` に従う（剛性比例／Rayleigh／
/// モード別／接線剛性比例、Newmark-β／HHT-α）。前処理を通したモデルを渡す前提。
pub fn compute_time_history(
    model: squid_n_core::model::Model,
    cfg: AnalysisSettings,
    wave: squid_n_solver::timehistory::GroundMotion,
) -> JobResult<squid_n_solver::timehistory::ResponseResult> {
    // 位相差入力（ねじれ加振）を指定時に付加する（構造動力学の位相差入力解析）。
    let wave = apply_phase_diff(&cfg, wave);
    let analysis = Analysis::prepare(&model).map_err(|e| JobError::Prepare(e.to_string()))?;
    let damping = match cfg.th_damping_model {
        ThDampingModel::StiffnessProportional => {
            // 1 次固有円振動数（減衰の基準）
            let omega1 = match analysis.eigen(1) {
                Ok(modal) => match modal.omega2.first() {
                    Some(&w2) if w2 > 0.0 => w2.sqrt(),
                    _ => {
                        return Err(JobError::InvalidInput(
                            "固有値が得られず減衰を設定できません。".to_string(),
                        ))
                    }
                },
                Err(e) => return Err(JobError::Solve(e.to_string())),
            };
            squid_n_solver::damping::Damping::StiffnessProportional {
                h: cfg.th_damping,
                omega: omega1,
                basis: squid_n_solver::damping::StiffnessKind::Initial,
            }
        }
        ThDampingModel::Rayleigh => {
            // 1次・2次の固有円振動数（Rayleigh 減衰の基準）
            let modal = match analysis.eigen(2) {
                Ok(m) => m,
                Err(e) => return Err(JobError::Solve(e.to_string())),
            };
            let (w1, w2) = match (modal.omega2.first(), modal.omega2.get(1)) {
                (Some(&a), Some(&b)) if a > 0.0 && b > 0.0 => (a.sqrt(), b.sqrt()),
                _ => {
                    return Err(JobError::InvalidInput(
                        "Rayleigh 減衰には 2 次までの固有値が必要です（モード数を確保できませんでした）。"
                            .to_string(),
                    ));
                }
            };
            squid_n_solver::damping::Damping::Rayleigh {
                h1: cfg.th_damping,
                w1,
                h2: cfg.th_h2,
                w2,
            }
        }
        ThDampingModel::Modal => {
            // モード別減衰: 得られる低次モードに一律の減衰比 h を与える。
            // 要求モード数はモデルの質量ランクに合わせ 6→1 の順に試行する。
            let mut modal = None;
            for k in (1..=6).rev() {
                if let Ok(m) = analysis.eigen(k) {
                    if !m.shapes.is_empty() {
                        modal = Some(m);
                        break;
                    }
                }
            }
            let modal = modal.ok_or_else(|| {
                JobError::InvalidInput("固有値が得られず減衰を設定できません。".to_string())
            })?;
            let omegas: Vec<f64> = modal
                .omega2
                .iter()
                .map(|&w2| if w2 > 0.0 { w2.sqrt() } else { 0.0 })
                .collect();
            let ratios = vec![cfg.th_damping; modal.shapes.len()];
            squid_n_solver::damping::Damping::modal(&modal.shapes, &omegas, &ratios)
        }
        ThDampingModel::TangentAlpha1 | ThDampingModel::TangentH1 => {
            // 瞬間（接線）剛性比例。基準は初期剛性の 1 次固有円振動数。
            let omega1 = match analysis.eigen(1) {
                Ok(modal) => match modal.omega2.first() {
                    Some(&w2) if w2 > 0.0 => w2.sqrt(),
                    _ => {
                        return Err(JobError::InvalidInput(
                            "固有値が得られず減衰を設定できません。".to_string(),
                        ))
                    }
                },
                Err(e) => return Err(JobError::Solve(e.to_string())),
            };
            if cfg.th_damping_model == ThDampingModel::TangentAlpha1 {
                squid_n_solver::damping::Damping::StiffnessProportional {
                    h: cfg.th_damping,
                    omega: omega1,
                    basis: squid_n_solver::damping::StiffnessKind::Tangent,
                }
            } else {
                squid_n_solver::damping::Damping::TangentStiffnessConstantH {
                    h1: cfg.th_damping,
                    omega1e: omega1,
                }
            }
        }
    };
    // 非線形時刻歴は `model` への可変借用が要る（Newton 反復で要素状態を
    // commit/rollback するため）。`analysis` はここまでで最後の利用（damping の
    // 固有値算定）なので、以降は使わず `model` への不変借用を終わらせる。
    if cfg.th_nonlinear {
        return compute_nonlinear_time_history(model, cfg, wave, damping);
    }
    // 0 は「自動決定」の意（`ThRecorder`/`recording.rs::auto_record_every` に委ねる）。
    let record_every = (cfg.th_record_every > 0).then_some(cfg.th_record_every);
    let result = match cfg.th_integrator {
        ThIntegrator::NewmarkBeta => {
            let newmark = squid_n_solver::timehistory::NewmarkCfg::average_accel();
            analysis.time_history(&wave, newmark, damping, record_every)
        }
        ThIntegrator::HhtAlpha => {
            let hht = squid_n_solver::timehistory::HhtCfg::new(wave.dt);
            analysis.time_history_hht(&wave, hht, damping, record_every)
        }
    };
    result.map_err(|e| JobError::Solve(e.to_string()))
}

/// 非線形時刻歴応答解析（[`compute_time_history`] の非線形分岐）。
/// dofmap/reducer の組み立ては [`compute_pushover`] と同じ経路
/// （`Analysis::prepare` は damping 算定用の固有値解析のみに使い、解の本体は
/// `DofMap::build` / `Reducer::build` を直接呼んで組み立てる）。
/// `use_kg`（幾何剛性）は増分解析 UI に対応する設定がないため、増分解析の
/// 既定と同じ `false` を用いる。減衰の累積方式（`DampingAccumulation`）も
/// UI 設定がないため既定（非累積型）を用いる。
fn compute_nonlinear_time_history(
    model: squid_n_core::model::Model,
    cfg: AnalysisSettings,
    wave: squid_n_solver::timehistory::GroundMotion,
    damping: squid_n_solver::damping::Damping,
) -> JobResult<squid_n_solver::timehistory::ResponseResult> {
    let model = model;
    squid_n_element::factory::ensure_nonlinear_input(&model).map_err(|e| {
        JobError::InvalidInput(format!("非線形時刻歴（部材耐力を算定できません）:\n{e}"))
    })?;
    let dofmap = squid_n_core::dof::DofMap::build(&model);
    let reducer = squid_n_solver::constraint::Reducer::build(&model, &dofmap);
    let n_indep = reducer.n_indep;
    let init = vec![0.0; n_indep];
    let newmark = squid_n_solver::timehistory::NewmarkCfg::average_accel();
    // 0 は「自動決定」の意（`ThRecorder`/`recording.rs::auto_record_every` に委ねる）。
    let record_every = (cfg.th_record_every > 0).then_some(cfg.th_record_every);
    let nl_cfg = squid_n_solver::timehistory::NonlinearThCfg {
        newton: squid_n_solver::newton::NewtonCriteria::new(cfg.th_max_iter, cfg.th_tol),
        use_kg: false,
        apply_long_term: cfg.th_apply_long_term,
        record_every,
    };
    squid_n_solver::timehistory::nonlinear_time_history_analysis(
        &model,
        &dofmap,
        &reducer,
        &wave,
        &newmark,
        &damping,
        squid_n_solver::damping::DampingAccumulation::default(),
        &init,
        &init,
        nl_cfg,
    )
    .map_err(|e| JobError::Convergence(e.to_string()))
}
