//! 非線形時刻歴応答解析（Newmark-β + Newton 反復 + commit/rollback）。
//!
//! - [`NonlinearThCfg`] — 解析設定（Newton 収束条件・幾何剛性・長期荷重初期化・記録間引き）
//! - [`nonlinear_time_history_analysis`] — 非線形時刻歴応答解析

use super::common::{
    accel_dir_at, accel_xy_at, empty_response, expand_peak_disp, mass_accel_free_into,
    reduced_vec_from, resolve_dt, solve_initial_accel, sparse_matvec_into, GroundInfluence,
    NewmarkCoeffs,
};
use super::config::{GroundMotion, NewmarkCfg};
use super::history::{
    choose_record_dir_y, pick_record_node, record_history_step, total_mass, update_story_drift,
};
use super::recording::{member_forces_nonlinear, ThRecorder};
use super::result::{ResponseHistory, ResponseResult};
use crate::common::assemble::assemble_global_m;
use crate::common::constraint::Reducer;
use crate::common::csc_cache::{CscCache, WeightedSumGuard};
use crate::common::elem_loop::apply_du_trial;
use crate::common::newton::{l2_norm, NewtonCriteria, STATIC_NEWTON};
use crate::common::tangent::{
    add_support_spring_f_int, assemble_k, assemble_k_cached_ref, compute_f_int,
};
use crate::common::transaction::{revert_all, StateSnapshot};
use crate::dynamic::damping::{Damping, DampingAccumulation};
use squid_n_core::dof::DofMap;
use squid_n_core::model::Model;
use squid_n_element::behavior::{ElementBehavior, MassOption};
use squid_n_element::factory::{build_nonlinear_behavior, StrengthBasis};
use squid_n_math::solver::{make_solver, LinearSolver, SolveError, SolverBackend};
use squid_n_math::sparse::{sparse_matvec, Triplet};

/// 非線形時刻歴応答解析の設定（Newton 収束条件・幾何剛性・長期荷重初期化・記録間引き）。
#[derive(Clone, Copy, Debug)]
pub struct NonlinearThCfg {
    /// 各時刻ステップの Newton 反復の収束規約（反復上限・相対許容誤差）。
    /// 基準ノルムは長期荷重を除いた動的外力ノルムと 1.0 の大きい方。
    pub newton: NewtonCriteria,
    /// 幾何剛性（P-Δ、Kg）を接線剛性に含めるか。
    pub use_kg: bool,
    /// 時刻歴開始前に長期荷重（固定・積載等、`LoadCaseKind::is_long_term`）を
    /// 静的 Newton 反復で載荷し、その変位・応力状態を時刻歴の初期条件とするか
    /// （プッシュオーバーの長期載荷フェーズと同じ考え方）。長期系荷重ケースが
    /// ないモデルでは何もしない。
    pub apply_long_term: bool,
    /// 詳細記録（[`super::ThRecording`]）の間引き係数。`None` は自動決定
    /// （記録フレーム数が概ね 1000 になるよう [`super::recording::auto_record_every`] で調整）。
    pub record_every: Option<usize>,
}

impl NonlinearThCfg {
    /// 既定値: 長期荷重初期化あり・幾何剛性なし・記録間引きは自動決定。
    /// Newton 反復の上限・相対許容誤差のみ呼び出し側で指定する。
    pub fn new(max_iter: usize, tol: f64) -> Self {
        Self {
            newton: NewtonCriteria::new(max_iter, tol),
            use_kg: false,
            apply_long_term: true,
            record_every: None,
        }
    }
}

impl Default for NonlinearThCfg {
    /// Newton 反復 20 回・相対許容誤差 1e-6・長期荷重初期化あり・幾何剛性なし・
    /// 記録間引きは自動決定。
    fn default() -> Self {
        Self::new(20, 1e-6)
    }
}

/// 非線形時刻歴応答解析（Newmark-β + Newton反復 + commit/rollback）。
///
/// 各時刻ステップで Newton 反復により内力（非線形復元力）と慣性力・減衰力・
/// 地震外力の動的釣合いを満たす解を求める。収束時は要素状態を commit、
/// 不収束時は Step 開始時の状態へ rollback する。
/// `cfg.apply_long_term` が真の場合、時刻歴開始前に長期荷重を静的載荷し
/// （プッシュオーバーの長期載荷フェーズと同じ経路）、その変位・応力状態を初期条件とする。
/// `initial_disp`/`initial_vel` は「長期解からの増分」としての動的初期条件で、
/// 記録される変位は長期変位を含む全変位である。
#[allow(clippy::too_many_arguments)]
pub fn nonlinear_time_history_analysis(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    wave: &GroundMotion,
    newmark: &NewmarkCfg,
    damping: &Damping,
    accumulation: DampingAccumulation,
    initial_disp: &[f64],
    initial_vel: &[f64],
    cfg: NonlinearThCfg,
) -> Result<ResponseResult, SolveError> {
    squid_n_math::parallelism::apply_to_faer();

    squid_n_element::factory::ensure_nonlinear_input(model).map_err(SolveError::InvalidInput)?;

    let dt = resolve_dt(newmark.dt, wave)?;

    let n_indep = reducer.n_indep;
    if n_indep == 0 {
        return Ok(empty_response(model, true, cfg.apply_long_term));
    }

    let mut behaviors = build_behaviors(model);
    for b in behaviors.iter_mut() {
        b.set_time_step(dt);
    }
    let mut non_converged_steps = 0usize;
    let mut peak_force_scale = 0.0_f64;
    let mut mu_hist: Vec<Vec<f64>> = vec![Vec::new(); model.elements.len()];

    let m_free = assemble_global_m(model, dofmap, MassOption::Consistent);
    let m_red = reducer.reduce_k(&m_free);

    let n_free = dofmap.n_active();
    let infl = GroundInfluence::build(model, dofmap, &m_free);
    let m_r_x: &[f64] = &infl.m_r_x;
    let m_r_y: &[f64] = &infl.m_r_y;

    let NewmarkCoeffs {
        c1,
        c2,
        c3,
        c4,
        c5,
        c6,
        ..
    } = NewmarkCoeffs::new(newmark, dt);

    let f0_free: Vec<f64> = if cfg.apply_long_term {
        let mut f = vec![0.0; n_free];
        for lc in model.load_cases.iter().filter(|l| l.kind.is_long_term()) {
            let flc = crate::common::assemble::assemble_global_f(model, dofmap, lc.id);
            for (acc, v) in f.iter_mut().zip(flc) {
                *acc += v;
            }
        }
        f
    } else {
        vec![0.0; n_free]
    };
    let f0_red = reducer.reduce_f(&f0_free);
    let has_long_term = f0_red.iter().any(|&v| v.abs() > 0.0);

    let mut u = vec![0.0; n_indep];
    if has_long_term {
        apply_long_term_static(
            model,
            dofmap,
            reducer,
            &mut behaviors,
            &f0_red,
            cfg.use_kg,
            &mut u,
        )?;
    }

    let k_free = assemble_k(model, dofmap, &behaviors, cfg.use_kg);
    let k_red = reducer.reduce_k(&k_free);
    let c_red = damping.assemble_c(&m_red, &k_red);

    {
        let du_dyn = reduced_vec_from(n_indep, initial_disp);
        for i in 0..n_indep {
            u[i] += du_dyn[i];
        }
        let du_free = reducer.expand_u(&du_dyn);
        apply_du_trial(model, dofmap, &mut behaviors, &du_free);
        for b in behaviors.iter_mut() {
            b.commit_state();
        }
    }

    let mut v = reduced_vec_from(n_indep, initial_vel);

    let mut f_damp = sparse_matvec(&c_red, &v);
    let mut c_v_last = vec![0.0; n_indep];

    let u_mode1: Vec<f64> = if matches!(damping, Damping::TangentStiffnessConstantH { .. }) {
        crate::dynamic::eigen::solve_eigen(model, dofmap, reducer, 1)
            .ok()
            .and_then(|modal| modal.shapes.into_iter().next())
            .filter(|s| s.len() == n_indep)
            .unwrap_or_else(|| vec![0.0; n_indep])
    } else {
        vec![0.0; n_indep]
    };

    let p_red_0 = reducer.reduce_f(&infl.force_at(wave, 0));

    let mut f_int0_free = compute_f_int(model, dofmap, &behaviors);
    {
        let u0_free = reducer.expand_u(&u);
        add_support_spring_f_int(model, dofmap, &u0_free, &mut f_int0_free);
    }
    let f_int0_red = reducer.reduce_f(&f_int0_free);
    let cv0 = sparse_matvec(&c_red, &v);
    let mut rhs_a0 = vec![0.0; n_indep];
    for i in 0..n_indep {
        rhs_a0[i] = p_red_0[i] + f0_red[i] - cv0[i] - f_int0_red[i];
    }
    let mut a = solve_initial_accel(&m_red, &rhs_a0, n_indep)?;

    let n_steps = wave.accel_x.len();
    let mut u_free = vec![0.0f64; n_free];
    let mut v_free = vec![0.0f64; n_free];
    let mut a_free = vec![0.0f64; n_free];
    reducer.expand_u_into(&u, &mut u_free);
    reducer.expand_u_into(&v, &mut v_free);
    reducer.expand_u_into(&a, &mut a_free);

    let mut peak_disp_free = vec![0.0f64; n_free];
    for i in 0..n_free {
        peak_disp_free[i] = peak_disp_free[i].max(u_free[i].abs());
    }
    let mut story_drift_angle = vec![0.0f64; model.layer_count()];
    update_story_drift(model, dofmap, &u_free, &mut story_drift_angle);

    let record_dir_y = choose_record_dir_y(wave);
    let dir_idx = if record_dir_y { 1 } else { 0 };
    let m_r_record: &[f64] = if record_dir_y { m_r_y } else { m_r_x };
    let mut history = ResponseHistory {
        node: pick_record_node(model, dofmap, dir_idx),
        record_dir_y,
        ..Default::default()
    };
    let rmr_record = total_mass(m_r_record, dofmap, model.nodes.len(), dir_idx);
    let mut ma_free = vec![0.0f64; n_free];
    mass_accel_free_into(&m_free, &a_free, &mut ma_free);
    {
        let xg_init = if record_dir_y {
            wave.accel_y
                .as_ref()
                .and_then(|a| a.first().copied())
                .unwrap_or(0.0)
        } else {
            wave.accel_x.first().copied().unwrap_or(0.0)
        };
        record_history_step(
            &mut history,
            model,
            dofmap,
            dir_idx,
            rmr_record,
            &u_free,
            &ma_free,
            xg_init,
        );
    }
    let mut time = Vec::with_capacity(n_steps + 1);
    time.push(0.0);

    let mut recorder = ThRecorder::new(
        model,
        dofmap,
        n_steps,
        model.elements.len(),
        cfg.record_every,
    );
    {
        let xg_x_init = wave.accel_x.first().copied().unwrap_or(0.0);
        let xg_y_init = wave
            .accel_y
            .as_ref()
            .and_then(|a| a.first().copied())
            .unwrap_or(0.0);
        let mf_init = member_forces_nonlinear(model, &behaviors);
        super::recording::ensure_line_member_forces_nonlinear(model, &mf_init)?;
        recorder.record_step(
            0, 0.0, model, dofmap, m_r_x, m_r_y, &ma_free, &u_free, &v_free, &a_free, xg_x_init,
            xg_y_init, &mf_init,
        );
    }

    let mut p_free_buf = vec![0.0f64; n_free];
    let mut p_dyn_red_buf = vec![0.0f64; n_indep];
    let mut p_red_buf = vec![0.0f64; n_indep];
    let mut u_trial_red_buf = vec![0.0f64; n_indep];
    let mut u_trial_free_buf = vec![0.0f64; n_free];
    let mut f_int_red_buf = vec![0.0f64; n_indep];
    let mut du_free_buf = vec![0.0f64; n_free];
    let mut dv_buf = vec![0.0f64; n_indep];
    let mut c_dv_buf = vec![0.0f64; n_indep];
    let mut c_v_red_buf = vec![0.0f64; n_indep];
    let mut m_a_red_buf = vec![0.0f64; n_indep];
    let mut r_red_buf = vec![0.0f64; n_indep];
    let mut du_red_buf = vec![0.0f64; n_indep];

    let mut k_eff_solver: Box<dyn LinearSolver> = make_solver(SolverBackend::DirectSparseCholesky);
    let mut k_t_free_cache = CscCache::new();
    let mut k_t_red_cache = CscCache::new();
    let mut k_eff_cache = WeightedSumGuard::new();
    let mut k_t_free_triplets_buf: Vec<Triplet> = Vec::new();
    let mut k_t_red_triplets_buf: Vec<Triplet> = Vec::new();

    for n in 0..n_steps {
        let t_next = (n + 1) as f64 * dt;

        infl.force_at_into(wave, n, &mut p_free_buf);
        reducer.reduce_f_into(&p_free_buf, &mut p_dyn_red_buf);
        let p_dyn_red = &p_dyn_red_buf;
        for i in 0..n_indep {
            p_red_buf[i] = p_dyn_red[i] + f0_red[i];
        }
        let p_red = &p_red_buf;

        let mut a_trial = vec![0.0; n_indep];
        let mut v_trial = vec![0.0; n_indep];
        for i in 0..n_indep {
            a_trial[i] = -c3 * v[i] - c4 * a[i];
            v_trial[i] = -c5 * v[i] - c6 * a[i];
        }

        let mut du_total = vec![0.0; n_indep];
        let mut converged = false;

        for _iter in cfg.newton.iters() {
            let (c_tan, k_t_red_precomputed) = if damping.is_tangent_based() {
                let k_t_free = assemble_k_cached_ref(
                    model,
                    dofmap,
                    &behaviors,
                    cfg.use_kg,
                    &mut k_t_free_cache,
                    &mut k_t_free_triplets_buf,
                );
                let k_t_red = reducer.reduce_k_cached_ref(
                    k_t_free,
                    &mut k_t_red_cache,
                    &mut k_t_red_triplets_buf,
                );
                let c = damping.assemble_c_tangent(&m_red, k_t_red, &k_red, &u_mode1);
                (Some(c), Some(k_t_red))
            } else {
                (None, None)
            };
            let c_cur = c_tan.as_ref().unwrap_or(&c_red);

            let mut f_int_free = compute_f_int(model, dofmap, &behaviors);
            {
                for i in 0..n_indep {
                    u_trial_red_buf[i] = u[i] + du_total[i];
                }
                reducer.expand_u_into(&u_trial_red_buf, &mut u_trial_free_buf);
                add_support_spring_f_int(model, dofmap, &u_trial_free_buf, &mut f_int_free);
            }
            reducer.reduce_f_into(&f_int_free, &mut f_int_red_buf);
            let f_int_red = &f_int_red_buf;

            match accumulation {
                DampingAccumulation::NonCumulative => {
                    sparse_matvec_into(c_cur, &v_trial, &mut c_v_red_buf)
                }
                DampingAccumulation::Cumulative => {
                    for i in 0..n_indep {
                        dv_buf[i] = v_trial[i] - v[i];
                    }
                    sparse_matvec_into(c_cur, &dv_buf, &mut c_dv_buf);
                    for i in 0..n_indep {
                        c_v_red_buf[i] = f_damp[i] + c_dv_buf[i];
                    }
                }
            };
            let c_v_red = &c_v_red_buf;
            c_v_last.clone_from(c_v_red);
            sparse_matvec_into(&m_red, &a_trial, &mut m_a_red_buf);
            let m_a_red = &m_a_red_buf;

            for i in 0..n_indep {
                r_red_buf[i] = p_red[i] - f_int_red[i] - c_v_red[i] - m_a_red[i];
            }
            let r_red = &r_red_buf;

            let r_norm = l2_norm(r_red);
            let scale = crate::common::newton::dynamic_force_scale(p_dyn_red, m_a_red, c_v_red);
            peak_force_scale = peak_force_scale.max(scale);
            let ref_norm = crate::common::newton::dynamic_reference_norm(scale, peak_force_scale);
            if cfg.newton.converged(r_norm, ref_norm) {
                converged = true;
                break;
            }

            let k_t_red = match k_t_red_precomputed {
                Some(k) => k,
                None => {
                    let k_t_free = assemble_k_cached_ref(
                        model,
                        dofmap,
                        &behaviors,
                        cfg.use_kg,
                        &mut k_t_free_cache,
                        &mut k_t_free_triplets_buf,
                    );
                    reducer.reduce_k_cached_ref(
                        k_t_free,
                        &mut k_t_red_cache,
                        &mut k_t_red_triplets_buf,
                    )
                }
            };
            let k_eff =
                k_eff_cache.combine_ref(n_indep, &[(1.0, k_t_red), (c2, c_cur), (c1, &m_red)]);
            k_eff_solver
                .factorize(k_eff)
                .map_err(|e| SolveError::Backend(format!("factor: {:?}", e)))?;
            k_eff_solver.solve_into(r_red, &mut du_red_buf)?;
            let du_red = &du_red_buf;
            reducer.expand_u_into(du_red, &mut du_free_buf);
            let du_free = &du_free_buf;

            for i in 0..n_indep {
                a_trial[i] += c1 * du_red[i];
                v_trial[i] += c2 * du_red[i];
                du_total[i] += du_red[i];
            }

            apply_du_trial(model, dofmap, &mut behaviors, du_free);
        }

        if !converged {
            if !du_total.iter().all(|x| x.is_finite()) {
                revert_all(&mut behaviors);
                return Err(SolveError::Backend(format!(
                    "nonlinear time history: step {} で応答が発散しました（変位が有限値ではありません）。                     時間刻み dt を小さくするか、減衰・復元力特性の設定を見直してください",
                    n
                )));
            }
            non_converged_steps += 1;
        }

        {
            for i in 0..n_indep {
                u[i] += du_total[i];
            }
            if accumulation == DampingAccumulation::Cumulative {
                f_damp.clone_from(&c_v_last);
            }
            v.copy_from_slice(&v_trial);
            a.copy_from_slice(&a_trial);

            for b in behaviors.iter_mut() {
                b.commit_state();
            }

            for (i, b) in behaviors.iter().enumerate() {
                if let Some(p) = b.ductility_probe() {
                    mu_hist[i].push(p.max_yield_ratio);
                }
            }

            time.push(t_next);

            reducer.expand_u_into(&u, &mut u_free);
            for i in 0..n_free {
                peak_disp_free[i] = peak_disp_free[i].max(u_free[i].abs());
            }
            update_story_drift(model, dofmap, &u_free, &mut story_drift_angle);
            reducer.expand_u_into(&v, &mut v_free);
            reducer.expand_u_into(&a, &mut a_free);
            mass_accel_free_into(&m_free, &a_free, &mut ma_free);
            let xg_next = accel_dir_at(wave, n + 1, record_dir_y);
            record_history_step(
                &mut history,
                model,
                dofmap,
                dir_idx,
                rmr_record,
                &u_free,
                &ma_free,
                xg_next,
            );

            let (xg_x_next, xg_y_next) = accel_xy_at(wave, n + 1);
            let mf_now = member_forces_nonlinear(model, &behaviors);
            recorder.record_step(
                (n + 1) as u64,
                t_next,
                model,
                dofmap,
                m_r_x,
                m_r_y,
                &ma_free,
                &u_free,
                &v_free,
                &a_free,
                xg_x_next,
                xg_y_next,
                &mf_now,
            );
        }
    }

    let peak_disp = expand_peak_disp(model, dofmap, &peak_disp_free);

    let fatigue = crate::damage::FatigueParams::default();
    let cumulative_ductility: Vec<f64> = mu_hist
        .iter()
        .map(|series| crate::damage::cumulative_damage_rainflow(series, fatigue))
        .collect();

    Ok(ResponseResult {
        non_converged_steps,
        time,
        peak_disp,
        story_drift_angle,
        cumulative_ductility,
        history,
        recording: Some(recorder.finish()),
        nonlinear: true,
        applied_long_term: cfg.apply_long_term,
    })
}

/// 長期漸増載荷の載荷率（0〜1）を追跡する状態機械。
///
/// 基準増分ごとに漸増し、収束失敗時は増分半減で再試行する。
#[derive(Debug, Clone, Copy)]
struct LoadFractionState {
    /// 収束が確定した載荷率。
    applied: f64,
    /// 失敗時に半減する前の基準増分（`1/n_grav`）。成功のたびにこれへ戻す。
    base_increment: f64,
    /// 次回試行する増分（半減リトライ中は `base_increment` より小さい）。
    increment: f64,
    /// 現在の載荷率に対する連続失敗回数。
    attempts: usize,
    /// 連続失敗の許容回数（これを超えたら `record_failure` が `false` を返す）。
    max_attempts: usize,
}

impl LoadFractionState {
    fn new(n_grav: usize) -> Self {
        let base_increment = 1.0 / n_grav.max(1) as f64;
        Self {
            applied: 0.0,
            base_increment,
            increment: base_increment,
            attempts: 0,
            max_attempts: 5,
        }
    }

    /// 次に試行すべき載荷率。100% に達していれば `None`。
    fn next_target(&self) -> Option<f64> {
        if self.applied >= 1.0 - 1e-9 {
            None
        } else {
            Some((self.applied + self.increment).min(1.0))
        }
    }

    /// 直前の [`Self::next_target`] の載荷率が収束した。載荷率を確定し、
    /// 次回の増分を基準増分へ戻す（半減リトライ後も残増分を基準ペースで継続する）。
    fn record_success(&mut self, mu_target: f64) {
        self.applied = mu_target;
        self.increment = self.base_increment;
        self.attempts = 0;
    }

    /// 直前の載荷率が収束しなかった。増分を半減して再試行する。
    /// 連続失敗が `max_attempts` に達したら `false`（呼び出し側は Err とする）。
    fn record_failure(&mut self) -> bool {
        self.attempts += 1;
        if self.attempts >= self.max_attempts {
            return false;
        }
        self.increment *= 0.5;
        true
    }
}

/// 長期荷重（`f0_red`、縮約空間）を静的 Newton 反復で載荷する。
/// 収束したステップごとに要素状態を commit し、載荷完了時の変位（縮約空間）を `u_out` に加算する。
fn apply_long_term_static(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    behaviors: &mut [Box<dyn ElementBehavior>],
    f0_red: &[f64],
    use_kg: bool,
    u_out: &mut [f64],
) -> Result<(), SolveError> {
    let n_indep = reducer.n_indep;
    let mut state = LoadFractionState::new(5);
    let mut solver: Box<dyn LinearSolver> = make_solver(SolverBackend::DirectSparseCholesky);
    let mut k_free_cache = CscCache::new();
    let mut k_red_cache = CscCache::new();
    let mut du_red_buf = vec![0.0f64; n_indep];
    let mut k_free_triplets_buf: Vec<Triplet> = Vec::new();
    let mut k_red_triplets_buf: Vec<Triplet> = Vec::new();
    while let Some(mu_target) = state.next_target() {
        let snap = StateSnapshot::capture(behaviors);
        let f_target: Vec<f64> = f0_red.iter().map(|&v| v * mu_target).collect();
        match newton_static_converge(
            model,
            dofmap,
            reducer,
            behaviors,
            &f_target,
            use_kg,
            n_indep,
            &*u_out,
            &mut solver,
            &mut k_free_cache,
            &mut k_red_cache,
            &mut du_red_buf,
            &mut k_free_triplets_buf,
            &mut k_red_triplets_buf,
        )? {
            Some(du) => {
                for b in behaviors.iter_mut() {
                    b.commit_state();
                }
                for i in 0..n_indep {
                    u_out[i] += du[i];
                }
                state.record_success(mu_target);
            }
            None => {
                snap.restore(behaviors);
                if !state.record_failure() {
                    return Err(SolveError::InvalidInput(
                        "長期荷重の初期載荷が収束しません（長期荷重に対して構造が不安定な可能性）"
                            .into(),
                    ));
                }
            }
        }
    }
    Ok(())
}

/// 固定外力 `f_target_red`（縮約空間）に対する静的 Newton 反復。
/// 収束時はステップ内の全修正量の累積を `Some` で返し、要素状態はトライアル反映済み・未確定のまま戻す。
/// 収束しなければ `Ok(None)`。
#[allow(clippy::too_many_arguments)]
fn newton_static_converge(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    behaviors: &mut [Box<dyn ElementBehavior>],
    f_target_red: &[f64],
    use_kg: bool,
    n_indep: usize,
    u_base_red: &[f64],
    solver: &mut Box<dyn LinearSolver>,
    k_free_cache: &mut CscCache,
    k_red_cache: &mut CscCache,
    du_red_buf: &mut Vec<f64>,
    k_free_triplets_buf: &mut Vec<Triplet>,
    k_red_triplets_buf: &mut Vec<Triplet>,
) -> Result<Option<Vec<f64>>, SolveError> {
    let mut du_total = vec![0.0; n_indep];
    for _iter in STATIC_NEWTON.iters() {
        let k_free = assemble_k_cached_ref(
            model,
            dofmap,
            behaviors,
            use_kg,
            k_free_cache,
            k_free_triplets_buf,
        );
        let k_red = reducer.reduce_k_cached_ref(k_free, k_red_cache, k_red_triplets_buf);
        let mut f_int_free = compute_f_int(model, dofmap, behaviors);
        {
            let u_trial_red: Vec<f64> = (0..n_indep).map(|i| u_base_red[i] + du_total[i]).collect();
            let u_trial_free = reducer.expand_u(&u_trial_red);
            add_support_spring_f_int(model, dofmap, &u_trial_free, &mut f_int_free);
        }
        let f_int_red = reducer.reduce_f(&f_int_free);
        let mut r_red = vec![0.0; n_indep];
        for i in 0..n_indep {
            r_red[i] = f_target_red[i] - f_int_red[i];
        }
        let r_norm = l2_norm(&r_red);
        let f_norm = l2_norm(f_target_red);
        if STATIC_NEWTON.converged(r_norm, f_norm.max(1.0)) {
            return Ok(Some(du_total));
        }
        solver
            .factorize(k_red)
            .map_err(|e| SolveError::Backend(format!("factor: {:?}", e)))?;
        solver.solve_into(&r_red, du_red_buf)?;
        let du_free = reducer.expand_u(du_red_buf.as_slice());
        for i in 0..n_indep {
            du_total[i] += du_red_buf[i];
        }
        apply_du_trial(model, dofmap, behaviors, &du_free);
    }
    Ok(None)
}

fn build_behaviors(model: &Model) -> Vec<Box<dyn squid_n_element::behavior::ElementBehavior>> {
    let mut behaviors = Vec::new();
    for elem in &model.elements {
        let b = build_nonlinear_behavior(
            elem,
            model,
            StrengthBasis::Nominal,
            squid_n_element::factory::AnalysisKind::TimeHistory,
        );
        behaviors.push(b);
    }
    behaviors
}

#[cfg(test)]
mod load_fraction_tests {
    use super::LoadFractionState;

    /// 全ステップ成功する場合、基準増分（1/5=0.2）刻みで 5 回で 100% へ到達する。
    #[test]
    fn all_success_reaches_full_load_in_five_steps() {
        let mut state = LoadFractionState::new(5);
        let mut targets = Vec::new();
        while let Some(mu) = state.next_target() {
            targets.push(mu);
            state.record_success(mu);
        }
        assert_eq!(targets.len(), 5);
        assert!((targets.last().copied().unwrap() - 1.0).abs() < 1e-12);
        assert!((state.applied - 1.0).abs() < 1e-12);
    }

    /// 途中で失敗→増分半減で成功するパスでも、最終的に載荷率は 100% に到達すること。
    #[test]
    fn half_step_success_still_reaches_full_load() {
        let mut state = LoadFractionState::new(5);
        let mut n_targets = 0usize;
        while let Some(mu) = state.next_target() {
            n_targets += 1;
            assert!(n_targets < 1000, "無限ループの疑い");
            // 最初の載荷率（1回目の目標=0.2）だけ 1 回失敗させ、以降は毎回成功させる。
            if n_targets == 1 {
                assert!(state.record_failure());
                continue;
            }
            state.record_success(mu);
        }
        assert!((state.applied - 1.0).abs() < 1e-9);
        // 100% に到達するまで、半減で増えた分の目標を含め複数回試行しているはず。
        assert!(n_targets > 5);
    }

    /// 最大試行回数を超えて失敗し続けると `record_failure` が false を返す
    /// （呼び出し側はこれを Err に変換する）。
    #[test]
    fn repeated_failure_exceeds_max_attempts() {
        let mut state = LoadFractionState::new(5);
        let mut ok = true;
        for _ in 0..10 {
            if state.next_target().is_none() {
                break;
            }
            ok = state.record_failure();
            if !ok {
                break;
            }
        }
        assert!(!ok);
    }
}
