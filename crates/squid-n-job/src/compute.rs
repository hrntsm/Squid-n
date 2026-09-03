//! 各解析の純粋計算（所有モデル＋解析条件 → 結果）。
//!
//! いずれも `&self` を取らない自由関数で、GUI の状態に触れない。GUI
//! （`squid-n-app`）はバックグラウンドスレッドから、MCP サーバ（`squid-n-mcp`）は
//! ジョブから、**同じ関数**を同じ解析条件で呼ぶ。
//!
//! 渡すモデルは [`crate::prepare::prepare_model_for_analysis`] または
//! GUI の `ensure_preparation` で前処理済み（剛域・仕口パネル・DL/LL/EX/EY 同期）
//! であること。荷重同期だけが必要な場合は [`crate::auto_loads`] を直接使う。
//!
//! **壁展開について**: 壁の解析要素（`ElementKind::Wall`）は入力の正である
//! `Model` には存在しない生成物（D5）のため、本モジュールの各公開関数は
//! 受け取った `model` を [`squid_n_load::wall_expand::expand_wall_elements`]
//! で壁展開してから解く（呼び出し元〔GUI・MCP・job〕に展開を要求しない。
//! 忘れると壁がソルバから消えるため）。壁展開モデルは呼び出しのたびに
//! 都度組み立て、キャッシュしない（`dev_docs/handoff/
//! 床領域・壁領域の再設計_申し送り.md` dig Q2=A）。

use crate::error::{JobError, JobResult};
use crate::settings::{AnalysisSettings, ThDampingModel};
use squid_n_core::ids::LoadCaseId;
use squid_n_solver::statics::analysis::Analysis;

/// 壁展開モデルを組み立てる（本モジュール共通のエントリポイント。モジュール doc
/// 「壁展開について」参照）。展開の索引・件数報告は解析の入口では使わないため捨てる。
fn expand_walls(model: squid_n_core::model::Model) -> squid_n_core::model::Model {
    squid_n_load::wall_expand::expand_wall_elements_owned(model).0
}

/// 線形静的解析。前処理（[`crate::prepare`]）を通したモデルを渡す前提で、
/// ここでは再適用しない（二重適用を避ける）。
pub fn compute_linear_static(
    model: squid_n_core::model::Model,
    lc: LoadCaseId,
) -> JobResult<squid_n_solver::statics::linear::StaticOnce> {
    let model = expand_walls(model);
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
) -> JobResult<squid_n_solver::dynamic::eigen::ModalResult> {
    let model = expand_walls(model);
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
    cfg: squid_n_solver::statics::analysis::SeismicCfg,
    t: f64,
) -> JobResult<squid_n_solver::statics::linear::StaticOnce> {
    let model = expand_walls(model);
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
    mut wave: squid_n_solver::dynamic::timehistory::GroundMotion,
) -> squid_n_solver::dynamic::timehistory::GroundMotion {
    if !cfg.phase_diff_enabled {
        return wave;
    }
    use squid_n_solver::dynamic::phase_diff::{phase_lag_time, torsional_accel_series};
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
/// 剛域・仕口パネルは [`crate::prepare`] で適用済みのモデルを渡す前提
/// （静的解析と同じ規約。かつては剛域だけをインライン適用する経路があり、
/// 仕口パネルの生成が省かれて静的解析と剛性の異なるモデルを解いていた）。
pub fn compute_pushover(
    model: squid_n_core::model::Model,
    cfg: AnalysisSettings,
) -> JobResult<squid_n_solver::nonlinear::pushover::PushoverResult> {
    let work = expand_walls(model);
    Analysis::prepare(&work).map_err(|e| JobError::Prepare(e.to_string()))?;
    let dofmap = squid_n_core::dof::DofMap::build(&work);
    let reducer = squid_n_solver::common::constraint::Reducer::build(&work, &dofmap);
    // 終了目標（目標変位・目標最大層間変形角）。両方無効なら荷重制御 λ=1 まで解析する。
    let target = squid_n_solver::nonlinear::pushover::PushoverTarget {
        max_disp: cfg.push_use_max_disp.then_some(cfg.push_max_disp),
        max_drift_angle: cfg
            .push_use_drift_angle
            .then_some(1.0 / cfg.push_drift_denom.max(1.0)),
    };
    squid_n_solver::nonlinear::pushover::pushover_analysis_recording(
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
/// モード別／接線剛性比例、Newmark-β）。前処理を通したモデルを渡す前提。
pub fn compute_time_history(
    model: squid_n_core::model::Model,
    cfg: AnalysisSettings,
    wave: squid_n_solver::dynamic::timehistory::GroundMotion,
) -> JobResult<squid_n_solver::dynamic::timehistory::ResponseResult> {
    // 位相差入力（ねじれ加振）を指定時に付加する（構造動力学の位相差入力解析）。
    let wave = apply_phase_diff(&cfg, wave);
    let model = expand_walls(model);
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
            squid_n_solver::dynamic::damping::Damping::StiffnessProportional {
                h: cfg.th_damping,
                omega: omega1,
                basis: squid_n_solver::dynamic::damping::StiffnessKind::Initial,
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
            squid_n_solver::dynamic::damping::Damping::Rayleigh {
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
            squid_n_solver::dynamic::damping::Damping::modal(&modal.shapes, &omegas, &ratios)
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
                squid_n_solver::dynamic::damping::Damping::StiffnessProportional {
                    h: cfg.th_damping,
                    omega: omega1,
                    basis: squid_n_solver::dynamic::damping::StiffnessKind::Tangent,
                }
            } else {
                squid_n_solver::dynamic::damping::Damping::TangentStiffnessConstantH {
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
    let newmark = squid_n_solver::dynamic::timehistory::NewmarkCfg::average_accel();
    analysis
        .time_history(&wave, newmark, damping, record_every)
        .map_err(|e| JobError::Solve(e.to_string()))
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
    wave: squid_n_solver::dynamic::timehistory::GroundMotion,
    damping: squid_n_solver::dynamic::damping::Damping,
) -> JobResult<squid_n_solver::dynamic::timehistory::ResponseResult> {
    let model = model;
    squid_n_element::factory::ensure_nonlinear_input(&model).map_err(|e| {
        JobError::InvalidInput(format!("非線形時刻歴（部材耐力を算定できません）:\n{e}"))
    })?;
    let dofmap = squid_n_core::dof::DofMap::build(&model);
    let reducer = squid_n_solver::common::constraint::Reducer::build(&model, &dofmap);
    let n_indep = reducer.n_indep;
    let init = vec![0.0; n_indep];
    let newmark = squid_n_solver::dynamic::timehistory::NewmarkCfg::average_accel();
    // 0 は「自動決定」の意（`ThRecorder`/`recording.rs::auto_record_every` に委ねる）。
    let record_every = (cfg.th_record_every > 0).then_some(cfg.th_record_every);
    let nl_cfg = squid_n_solver::dynamic::timehistory::NonlinearThCfg {
        newton: squid_n_solver::common::newton::NewtonCriteria::new(cfg.th_max_iter, cfg.th_tol),
        use_kg: false,
        apply_long_term: cfg.th_apply_long_term,
        record_every,
    };
    squid_n_solver::dynamic::timehistory::nonlinear_time_history_analysis(
        &model,
        &dofmap,
        &reducer,
        &wave,
        &newmark,
        &damping,
        squid_n_solver::dynamic::damping::DampingAccumulation::default(),
        &init,
        &init,
        nl_cfg,
    )
    .map_err(|e| JobError::Convergence(e.to_string()))
}

/// 質点系（固有値。`accel` があれば時刻歴も）。
///
/// モデル生成は [`crate::lumped_mass::build_lumped_mass`]。線形は地震静的 EX/EY、
/// 非線形は増分解析結果が前提。3 次元は EX と EY の両方が必須。
pub fn compute_lumped_mass(
    model: squid_n_core::model::Model,
    cfg: AnalysisSettings,
    res_x: Option<squid_n_solver::statics::linear::StaticOnce>,
    res_y: Option<squid_n_solver::statics::linear::StaticOnce>,
    po_x: Option<squid_n_solver::nonlinear::pushover::PushoverResult>,
    po_y: Option<squid_n_solver::nonlinear::pushover::PushoverResult>,
    accel: Option<&[f64]>,
) -> JobResult<squid_n_solver::dynamic::lumped_mass::LumpedMassResult> {
    let model = expand_walls(model);
    let lm = crate::lumped_mass::build_lumped_mass(crate::lumped_mass::LumpedMassBuildInput {
        model: &model,
        dim: cfg.lumped_dim,
        source: cfg.lumped_stiffness,
        dir: cfg.lumped_dir,
        nonlinear: cfg.lumped_nonlinear,
        secant_ratio: cfg.lumped_secant_ratio,
        res_x: res_x.as_ref(),
        res_y: res_y.as_ref(),
        po_x: po_x.as_ref(),
        po_y: po_y.as_ref(),
    })?;
    let n_modes = cfg.lumped_n_modes.max(1);
    let modal = squid_n_solver::dynamic::lumped_mass::lumped_mass_eigen(&lm, n_modes)
        .map_err(|e| JobError::Solve(e.to_string()))?;
    let response = if let Some(a) = accel {
        if a.is_empty() {
            return Err(JobError::InvalidInput(
                "質点系時刻歴の地動加速度が空です".into(),
            ));
        }
        if cfg.lumped_th_dt <= 0.0 {
            return Err(JobError::InvalidInput(
                "質点系時刻歴の時間刻み dt が 0 以下です".into(),
            ));
        }
        let resp = squid_n_solver::dynamic::lumped_mass::lumped_mass_time_history(
            &lm,
            a,
            cfg.lumped_th_dt,
            cfg.lumped_th_damping,
        );
        if !lm.stories.is_empty() && resp.time.is_empty() {
            return Err(JobError::Solve(
                "質点系時刻歴を解けませんでした。質量または回転慣性を確認してください".into(),
            ));
        }
        Some(resp)
    } else {
        None
    };
    Ok(squid_n_solver::dynamic::lumped_mass::LumpedMassResult {
        model: lm,
        modal,
        response,
    })
}
