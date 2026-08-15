//! 質点系（串団子）時刻歴応答（せん断型多質点系、構造力学）。
//!
//! 各層トリリニアの最大点指向型履歴で、Newmark-β 平均加速度法・Newton-Raphson により
//! 非線形時刻歴応答を解く。減衰は初期剛性比例。
//!
//! - [`StickResponse`] — 質点系（せん断型）時刻歴応答解析の結果。
//! - [`StickDirPeaks`] — 層ピークの方向内訳（X / Y / 45°）。
//! - [`lumped_mass_time_history`] — 質点系モデルの非線形時刻歴応答解析。

use super::model::{LumpedMassModel, StoryTrilinear};
use crate::common::newton::NewtonCriteria;
use squid_n_material::{HysteresisMaterial, HysteresisRule, UniaxialMaterial};

/// 層ピークの方向内訳（下層→上層）。長さは層数。旧プロジェクトでは空。
///
/// - **X / Y**: 剛心位置の並進成分の時刻歴最大絶対値
/// - **45°**: 単位ベクトル `(1,1)/√2` と `(1,−1)/√2`（135°）への投影絶対値の大きい方
///
/// 水平合成 √(vx²+vy²) の最大は [`StickResponse::story_peak_drift`] 等に入れる。
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct StickDirPeaks {
    #[serde(default)]
    pub x: Vec<f64>,
    #[serde(default)]
    pub y: Vec<f64>,
    #[serde(default)]
    pub deg45: Vec<f64>,
}

impl StickDirPeaks {
    pub fn zeros(n: usize) -> Self {
        Self {
            x: vec![0.0; n],
            y: vec![0.0; n],
            deg45: vec![0.0; n],
        }
    }

    /// 方向内訳がある（現行の解析結果）なら真。旧 `.scz` は空。
    pub fn has_values(&self) -> bool {
        !self.x.is_empty()
    }

    /// `(vx, vy)` の X / Y / 45° ピークを更新し、水平合成 √(vx²+vy²) を返す。
    pub(crate) fn accumulate(&mut self, i: usize, vx: f64, vy: f64) -> f64 {
        self.x[i] = self.x[i].max(vx.abs());
        self.y[i] = self.y[i].max(vy.abs());
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let p45 = ((vx + vy) * s).abs();
        let p135 = ((vx - vy) * s).abs();
        self.deg45[i] = self.deg45[i].max(p45).max(p135);
        (vx * vx + vy * vy).sqrt()
    }
}

fn peak_ratio(num: f64, den: f64) -> f64 {
    if den > 1e-9 {
        num / den
    } else {
        0.0
    }
}

/// 層間ピークから方向別塑性率を作る。欠けた方向の δ1 は `+∞` で μ=0。
/// 45° の δ1 は軸平行の降伏矩形に沿う斜め距離 `√2 · min(δ1x, δ1y)`。
pub(crate) fn dir_ductility(drift: &StickDirPeaks, d1x: &[f64], d1y: &[f64]) -> StickDirPeaks {
    let n = drift.x.len();
    let mut out = StickDirPeaks::zeros(n);
    for i in 0..n {
        let dx1 = d1x.get(i).copied().unwrap_or(f64::INFINITY);
        let dy1 = d1y.get(i).copied().unwrap_or(f64::INFINITY);
        out.x[i] = peak_ratio(drift.x[i], dx1);
        out.y[i] = peak_ratio(drift.y[i], dy1);
        let d1_45 = std::f64::consts::SQRT_2 * dx1.min(dy1);
        out.deg45[i] = peak_ratio(drift.deg45[i], d1_45);
    }
    out
}

/// 質点系（せん断型）時刻歴応答解析の結果。
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct StickResponse {
    /// 時刻列 [s]。
    pub time: Vec<f64>,
    /// 最上階（頂部）質点の加振方向変位時刻歴 [mm]（質量重心）。
    pub roof_disp: Vec<f64>,
    /// 各層の最大層間変形 [mm]（下層→上層）。水平合成 √(δx²+δy²) の時刻歴最大。
    pub story_peak_drift: Vec<f64>,
    /// 各層の最大層せん断力 [N]。水平合成 √(Qx²+Qy²) の時刻歴最大。
    pub story_peak_shear: Vec<f64>,
    /// 各層の最大塑性率。並進ばねが経験した `max(μx, μy)`（δ1≤0 の方向は 0）。
    pub story_ductility: Vec<f64>,
    /// Newton 反復が上限（[`STICK_NEWTON`]）内に収束しなかった時刻ステップ数。
    pub non_converged_steps: usize,
    /// 各時刻の層変位 [Ux, Uy, θz]（下層→上層）。3D 再生と 2D の加力方向変位に使う。
    #[serde(default)]
    pub floor_disp: Vec<Vec<[f64; 3]>>,
    /// 層間変形の方向内訳 [mm]。
    #[serde(default)]
    pub drift_dir: StickDirPeaks,
    /// 層せん断の方向内訳 [N]。
    #[serde(default)]
    pub shear_dir: StickDirPeaks,
    /// 塑性率の方向内訳。
    #[serde(default)]
    pub ductility_dir: StickDirPeaks,
}

impl StickResponse {
    pub(crate) fn empty(n: usize) -> Self {
        Self {
            time: Vec::new(),
            roof_disp: Vec::new(),
            story_peak_drift: vec![0.0; n],
            story_peak_shear: vec![0.0; n],
            story_ductility: vec![0.0; n],
            non_converged_steps: 0,
            floor_disp: Vec::new(),
            drift_dir: StickDirPeaks::zeros(n),
            shear_dir: StickDirPeaks::zeros(n),
            ductility_dir: StickDirPeaks::zeros(n),
        }
    }

    /// 頂部（最上階質量重心）の方向別最大変位 [mm]。
    /// `floor_disp` が空の旧結果では、加振方向時刻歴の絶対最大だけを「最大」に入れる。
    pub fn roof_dir_peaks(&self) -> (f64, f64, f64, f64) {
        let mut p = StickDirPeaks::zeros(1);
        let mut max = 0.0_f64;
        let mut any = false;
        for frame in &self.floor_disp {
            if let Some(u) = frame.last() {
                any = true;
                max = max.max(p.accumulate(0, u[0], u[1]));
            }
        }
        if any {
            (p.x[0], p.y[0], p.deg45[0], max)
        } else {
            let r = self.roof_disp.iter().fold(0.0f64, |m, v| m.max(v.abs()));
            (0.0, 0.0, 0.0, r)
        }
    }
}

/// 各時刻ステップの Newton 反復の収束規約。基準ノルムは動的釣り合いの各項の最大
/// （[`crate::newton::dynamic_reference_norm`]）。地動がゼロの時刻でも慣性力・
/// 減衰力が基準を支えるため、分母が退化しない。
pub const STICK_NEWTON: NewtonCriteria = NewtonCriteria::new(30, 1e-6);

/// 三重対角系 `A·x=b` を Thomas 法で解く（`a`=下副対角, `b_diag`=主対角, `c`=上副対角）。
pub(crate) fn solve_tridiagonal(a: &[f64], b_diag: &[f64], c: &[f64], d: &[f64]) -> Vec<f64> {
    let n = b_diag.len();
    if n == 0 {
        return Vec::new();
    }
    let mut cp = vec![0.0; n];
    let mut dp = vec![0.0; n];
    cp[0] = c[0] / b_diag[0];
    dp[0] = d[0] / b_diag[0];
    for i in 1..n {
        let m = b_diag[i] - a[i] * cp[i - 1];
        let m = if m.abs() < 1e-30 { 1e-30 } else { m };
        cp[i] = c[i] / m;
        dp[i] = (d[i] - a[i] * dp[i - 1]) / m;
    }
    let mut x = vec![0.0; n];
    x[n - 1] = dp[n - 1];
    for i in (0..n - 1).rev() {
        x[i] = dp[i] - cp[i] * x[i + 1];
    }
    x
}

/// 初期剛性の三重対角せん断系の 1 次固有円振動数 ω1 を逆反復で求める。
/// `m`=質量, `k`=各層初期剛性（`k[i]`=層 i の K1）。
pub(crate) fn fundamental_omega(m: &[f64], k: &[f64]) -> f64 {
    let n = m.len();
    if n == 0 {
        return 0.0;
    }
    // 初期剛性の三重対角 K（せん断型: K[i][i]=k_i+k_{i+1}, 副対角=−k_{i+1}）。
    let mut diag = vec![0.0; n];
    let mut lower = vec![0.0; n];
    let mut upper = vec![0.0; n];
    for i in 0..n {
        let ki = k[i];
        let ki1 = if i + 1 < n { k[i + 1] } else { 0.0 };
        diag[i] = ki + ki1;
        if i + 1 < n {
            upper[i] = -ki1;
            lower[i + 1] = -ki1;
        }
    }
    // 逆反復: K x = M x_prev。
    let mut x = vec![1.0; n];
    let mut omega2 = 0.0;
    for _ in 0..50 {
        let b: Vec<f64> = (0..n).map(|i| m[i] * x[i]).collect();
        let y = solve_tridiagonal(&lower, &diag, &upper, &b);
        // 正規化（M ノルム）。
        let ynorm: f64 = (0..n).map(|i| m[i] * y[i] * y[i]).sum::<f64>().sqrt();
        if ynorm < 1e-30 {
            break;
        }
        let xn: Vec<f64> = y.iter().map(|v| v / ynorm).collect();
        // Rayleigh 商 ω² = xᵀKx / xᵀMx。
        let kx_diag: Vec<f64> = (0..n)
            .map(|i| {
                let mut s = diag[i] * xn[i];
                if i > 0 {
                    s += lower[i] * xn[i - 1];
                }
                if i + 1 < n {
                    s += upper[i] * xn[i + 1];
                }
                s
            })
            .collect();
        let num: f64 = (0..n).map(|i| xn[i] * kx_diag[i]).sum();
        let den: f64 = (0..n).map(|i| m[i] * xn[i] * xn[i]).sum();
        omega2 = if den > 0.0 { num / den } else { 0.0 };
        x = xn;
    }
    omega2.max(0.0).sqrt()
}

/// 層のトリリニア骨格から最大点指向型（Clough 系トリリニア）の履歴材料を作る。
pub(crate) fn story_spring(sk: &StoryTrilinear) -> HysteresisMaterial {
    HysteresisMaterial::new(HysteresisRule::MaxPointOriented {
        crack: (sk.q1.max(1.0), sk.d1.max(1e-6)),
        yield_point: (sk.q2.max(sk.q1 + 1.0), sk.d2.max(sk.d1 * 1.0001)),
        ultimate: (sk.q3.max(sk.q2 + 1.0), sk.d3.max(sk.d2 * 1.0001)),
    })
}

/// 質点系（せん断型）モデルの非線形時刻歴応答解析（Newmark-β 平均加速度法・
/// Newton-Raphson）。せん断型多質点系の復元力特性（各層トリリニア、構造力学）。
///
/// - `lm`: 串団子モデル（各層の質量・トリリニア骨格）。
/// - `accel`: 地動加速度 [mm/s²]。`dt`: 刻み [s]。`h`: 減衰定数（初期剛性比例）。
///
/// 各層の復元力は最大点指向型トリリニア。減衰は初期剛性比例
/// `C=(2h/ω1)·K_init`（ω1=1 次固有円振動数）。
pub fn lumped_mass_time_history(
    lm: &LumpedMassModel,
    accel: &[f64],
    dt: f64,
    h: f64,
) -> StickResponse {
    let n = lm.stories.len();
    if n == 0 || dt <= 0.0 || accel.is_empty() {
        return StickResponse::empty(n);
    }
    if lm.is_spatial() {
        return super::spatial::lumped_mass_time_history_spatial(lm, accel, dt, h);
    }
    let mass: Vec<f64> = lm.stories.iter().map(|s| s.mass.max(1e-9)).collect();
    let k_init: Vec<f64> = lm.stories.iter().map(|s| s.skeleton.k1.max(1e-9)).collect();
    let mut springs: Vec<HysteresisMaterial> = lm
        .stories
        .iter()
        .map(|s| story_spring(&s.skeleton))
        .collect();

    // 初期剛性比例減衰係数 a1=2h/ω1。ω1 は固有値解析（faer、
    // `super::eigen::lumped_mass_eigen`）の1次モードを優先し、失敗時のみ
    // 本関数（逆反復法）へフォールバックする（`super::eigen::stick_omega1`）。
    let omega1 = super::eigen::stick_omega1(lm);
    let a1 = if omega1 > 0.0 { 2.0 * h / omega1 } else { 0.0 };

    // Newmark 平均加速度（β=1/4, γ=1/2）。
    let beta = 0.25;
    let gamma = 0.5;
    let c1 = 1.0 / (beta * dt * dt); // a = c1·Δu − ...
    let c2 = gamma / (beta * dt); // v = c2·Δu − ...

    let mut u = vec![0.0; n];
    let mut v = vec![0.0; n];
    let mut a = vec![0.0; n];

    let mut time: Vec<f64> = Vec::with_capacity(accel.len());
    let mut roof: Vec<f64> = Vec::with_capacity(accel.len());
    let mut floor_disp: Vec<Vec<[f64; 3]>> = Vec::with_capacity(accel.len());
    let mut peak_drift: Vec<f64> = vec![0.0; n];
    let mut peak_shear: Vec<f64> = vec![0.0; n];
    let mut drift_dir = StickDirPeaks::zeros(n);
    let mut shear_dir = StickDirPeaks::zeros(n);

    // 層ドリフト δ_i = u_i − u_{i-1}（u_0=base=0）。
    let drift = |u: &[f64], i: usize| if i == 0 { u[0] } else { u[i] - u[i - 1] };

    let mut non_converged_steps = 0usize;
    // 収束判定の基準ノルムの下限に使う、解析中に観測した力のスケールの最大値。
    let mut peak_force_scale = 0.0_f64;

    for (step, &ag) in accel.iter().enumerate() {
        // 外力（地動慣性力）。
        let p: Vec<f64> = mass.iter().map(|&mi| -mi * ag).collect();
        // 予測子（変位一定, du=0 から Newton）。
        let u_prev = u.clone();
        let v_prev = v.clone();
        let a_prev = a.clone();
        let mut u_tr = u_prev.clone();
        let mut step_converged = false;

        for _iter in STICK_NEWTON.iters() {
            // 層せん断・接線（各 spring を drift で試行）。
            let mut q = vec![0.0; n];
            let mut kt = vec![0.0; n];
            for i in 0..n {
                let (qi, ki) = springs[i].trial(drift(&u_tr, i));
                q[i] = qi;
                kt[i] = ki.max(1e-6);
            }
            // 内力 f_int[i]=Q_i−Q_{i+1}。
            let mut f_int = vec![0.0; n];
            for i in 0..n {
                let q_above = if i + 1 < n { q[i + 1] } else { 0.0 };
                f_int[i] = q[i] - q_above;
            }
            // Newmark の a, v（u_tr に対応）。
            let a_tr: Vec<f64> = (0..n)
                .map(|i| {
                    c1 * (u_tr[i] - u_prev[i])
                        - (1.0 / (beta * dt)) * v_prev[i]
                        - (1.0 / (2.0 * beta) - 1.0) * a_prev[i]
                })
                .collect();
            let v_tr: Vec<f64> = (0..n)
                .map(|i| v_prev[i] + dt * ((1.0 - gamma) * a_prev[i] + gamma * a_tr[i]))
                .collect();
            // 減衰力 C·v、C=a1·K_init（初期剛性比例・一定）。C·v を初期層剛性から
            // 直接計算する。従来は接線剛性 kt を用いており、降伏で層剛性が低下すると
            // 減衰も比例して失われる接線剛性比例減衰になっていた（docstring の
            // 初期剛性比例 C=(2h/ω1)·K_init と不整合。非弾性応答を過大評価する非安全側）。
            let cv = tridiag_stiffness_matvec(&k_init, &v_tr, a1);
            // 残差 r = p − M·a − C·v − f_int。
            let mut r = vec![0.0; n];
            let mut rnorm = 0.0;
            for i in 0..n {
                r[i] = p[i] - mass[i] * a_tr[i] - cv[i] - f_int[i];
                rnorm += r[i] * r[i];
            }
            // 基準ノルムは動的釣り合いの各項（外力・慣性力・減衰力）の最大とする。
            // 外力だけを基準にすると、地動加速度がゼロを横切る時刻で基準が消えて
            // 床の 1.0（N）まで落ち、判定が絶対値判定に化けて到達不能になる
            // （立体モデルの非線形時刻歴で実際に不収束を起こした。
            // `dev_docs/handoff/非線形時刻歴の収束_申し送り.md`）。
            let ma: Vec<f64> = (0..n).map(|i| mass[i] * a_tr[i]).collect();
            let scale = crate::newton::dynamic_force_scale(&p, &ma, &cv);
            peak_force_scale = peak_force_scale.max(scale);
            let ref_norm = crate::newton::dynamic_reference_norm(scale, peak_force_scale);
            if STICK_NEWTON.converged(rnorm.sqrt(), ref_norm) {
                step_converged = true;
                break;
            }
            // 有効接線 Keff = c1·M + c2·C + K_t（三重対角）。
            // 接線 K_t は kt、減衰 C=a1·K_init は初期剛性 k_init から組む。
            let (low, diag, up) = effective_tridiagonal(&mass, &kt, &k_init, a1, c1, c2);
            let du = solve_tridiagonal(&low, &diag, &up, &r);
            for i in 0..n {
                u_tr[i] += du[i];
            }
        }

        // 確定。非収束のまま反復上限へ達した場合も従来どおりトライアル状態を
        // 確定する（数値挙動は不変）が、無音にはせずステップ数を数えて結果へ
        // 明示する（プッシュオーバーの打ち切り明示と同じ方針）。
        if !step_converged {
            non_converged_steps += 1;
        }
        for s in springs.iter_mut() {
            s.commit();
        }
        // a, v を確定値へ更新。
        let a_new: Vec<f64> = (0..n)
            .map(|i| {
                c1 * (u_tr[i] - u_prev[i])
                    - (1.0 / (beta * dt)) * v_prev[i]
                    - (1.0 / (2.0 * beta) - 1.0) * a_prev[i]
            })
            .collect();
        let v_new: Vec<f64> = (0..n)
            .map(|i| v_prev[i] + dt * ((1.0 - gamma) * a_prev[i] + gamma * a_new[i]))
            .collect();
        u = u_tr;
        v = v_new;
        a = a_new;

        // 応答の記録。2 次元は加振方向成分だけが非ゼロ。
        for i in 0..n {
            let d_signed = drift(&u, i);
            let (qi, _) = {
                let mut sp = springs[i].clone();
                sp.trial(d_signed)
            };
            let (dx, dy, qx, qy) = match lm.dir {
                crate::analysis::SeismicDir::X => (d_signed, 0.0, qi, 0.0),
                crate::analysis::SeismicDir::Y => (0.0, d_signed, 0.0, qi),
            };
            peak_drift[i] = peak_drift[i].max(drift_dir.accumulate(i, dx, dy));
            peak_shear[i] = peak_shear[i].max(shear_dir.accumulate(i, qx, qy));
        }
        time.push(step as f64 * dt);
        roof.push(u[n - 1]);
        let mut frame = vec![[0.0; 3]; n];
        for (i, slot) in frame.iter_mut().enumerate() {
            match lm.dir {
                crate::analysis::SeismicDir::X => slot[0] = u[i],
                crate::analysis::SeismicDir::Y => slot[1] = u[i],
            }
        }
        floor_disp.push(frame);
    }

    let (d1x, d1y): (Vec<f64>, Vec<f64>) = match lm.dir {
        crate::analysis::SeismicDir::X => (
            lm.stories.iter().map(|s| s.skeleton.d1).collect(),
            vec![f64::INFINITY; n],
        ),
        crate::analysis::SeismicDir::Y => (
            vec![f64::INFINITY; n],
            lm.stories.iter().map(|s| s.skeleton.d1).collect(),
        ),
    };
    let ductility_dir = dir_ductility(&drift_dir, &d1x, &d1y);
    let ductility: Vec<f64> = (0..n)
        .map(|i| ductility_dir.x[i].max(ductility_dir.y[i]))
        .collect();

    StickResponse {
        time,
        roof_disp: roof,
        story_peak_drift: peak_drift,
        story_peak_shear: peak_shear,
        story_ductility: ductility,
        non_converged_steps,
        floor_disp,
        drift_dir,
        shear_dir,
        ductility_dir,
    }
}

/// せん断型三重対角剛性 `K(kt)` と `a1·K` の和は使わず、`(scale·K)·x` を直接計算する。
/// `scale` は減衰係数 a1。せん断型: `K[i][i]=kt_i+kt_{i+1}`, 副対角 `−kt_{i+1}`。
fn tridiag_stiffness_matvec(kt: &[f64], x: &[f64], scale: f64) -> Vec<f64> {
    let n = kt.len();
    let mut y = vec![0.0; n];
    for i in 0..n {
        let ki = kt[i];
        let ki1 = if i + 1 < n { kt[i + 1] } else { 0.0 };
        let mut s = (ki + ki1) * x[i];
        if i > 0 {
            s += -ki * x[i - 1];
        }
        if i + 1 < n {
            s += -ki1 * x[i + 1];
        }
        y[i] = scale * s;
    }
    y
}

/// 有効接線 `Keff = c1·M + c2·C + K_t` の三重対角成分（下・主・上）。
/// 接線剛性 `kt` は復元力 K_t（係数 1）に、初期剛性 `k_damp`(=k_init) は
/// 初期剛性比例減衰 `C=a1·K_init`（係数 c2）に用いる。両者は降伏後に異なる
/// （従来は両方に kt を用いており接線剛性比例減衰になっていた）。
fn effective_tridiagonal(
    mass: &[f64],
    kt: &[f64],
    k_damp: &[f64],
    a1: f64,
    c1: f64,
    c2: f64,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let n = kt.len();
    let mut low = vec![0.0; n];
    let mut diag = vec![0.0; n];
    let mut up = vec![0.0; n];
    // 剛性倍率: 接線 K_t は係数 1、減衰 C=a1·K_init は係数 c2·a1。
    let cd = c2 * a1;
    for i in 0..n {
        let kti = kt[i];
        let kti1 = if i + 1 < n { kt[i + 1] } else { 0.0 };
        let kdi = k_damp[i];
        let kdi1 = if i + 1 < n { k_damp[i + 1] } else { 0.0 };
        diag[i] = c1 * mass[i] + (kti + kti1) + cd * (kdi + kdi1);
        if i + 1 < n {
            let off = kti1 + cd * kdi1;
            up[i] = -off;
            low[i + 1] = -off;
        }
    }
    (low, diag, up)
}

#[cfg(test)]
mod damping_tests {
    use super::*;

    /// 有効接線の減衰項は初期剛性 k_init から組む（接線 kt ではない）。
    /// 降伏後（kt ≪ k_init）で両者は大きく異なるため、分離を検証する。
    #[test]
    fn test_effective_tridiagonal_damping_uses_initial_stiffness() {
        // 1 質点、降伏後: 接線 kt=1、初期 k_init=100。
        let mass = [2.0];
        let kt = [1.0];
        let k_init = [100.0];
        let (a1, c1, c2) = (0.1, 4.0, 2.0);
        let cd = c2 * a1; // 0.2

        let (_low, diag, _up) = effective_tridiagonal(&mass, &kt, &k_init, a1, c1, c2);
        // Keff = c1·M + K_t + (c2·a1)·K_init = 8 + 1 + 0.2·100 = 29。
        let expected = c1 * mass[0] + kt[0] + cd * k_init[0];
        assert!(
            (diag[0] - expected).abs() < 1e-12,
            "diag={} expected={} (減衰は初期剛性ベース)",
            diag[0],
            expected
        );
        // 接線剛性でしか組まない旧実装は 8 + (1+c2·a1)·1 = 9.2 で明確に異なる。
        let buggy = c1 * mass[0] + (1.0 + cd) * kt[0];
        assert!((diag[0] - buggy).abs() > 10.0);
    }

    /// 減衰力 C·v も初期剛性から評価される（tridiag_stiffness_matvec の直接検証）。
    #[test]
    fn test_damping_force_matvec_scales_stiffness() {
        // 2 質点 K=[1,1] の三重対角 [[2,-1],[-1,1]]·v をスケール a1 倍。
        let k = [1.0, 1.0];
        let v = [1.0, 0.0];
        let a1 = 0.5;
        let cv = tridiag_stiffness_matvec(&k, &v, a1);
        // 行0: a1·((1+1)·1 − 1·0) = 0.5·2 = 1.0、行1: a1·(−1·1) = −0.5。
        assert!((cv[0] - 1.0).abs() < 1e-12 && (cv[1] + 0.5).abs() < 1e-12);
    }
}

#[cfg(test)]
mod dir_peaks_tests {
    use super::{dir_ductility, StickDirPeaks};

    #[test]
    fn accumulate_splits_xy_diagonal_and_resultant() {
        let mut p = StickDirPeaks::zeros(1);
        let max = p.accumulate(0, 3.0, 4.0);
        assert!((max - 5.0).abs() < 1e-12);
        assert!((p.x[0] - 3.0).abs() < 1e-12);
        assert!((p.y[0] - 4.0).abs() < 1e-12);
        let want45 = 7.0 * std::f64::consts::FRAC_1_SQRT_2;
        assert!((p.deg45[0] - want45).abs() < 1e-12);
        // 135° 側の軌道でも 45° 欄は同じ斜め包絡になる。
        let mut q = StickDirPeaks::zeros(1);
        q.accumulate(0, 3.0, -4.0);
        assert!((q.deg45[0] - want45).abs() < 1e-12);
    }

    #[test]
    fn dir_ductility_uses_axis_and_diagonal_yield() {
        let mut drift = StickDirPeaks::zeros(1);
        drift.accumulate(0, 3.0, 4.0);
        let mu = dir_ductility(&drift, &[1.0], &[1.0]);
        assert!((mu.x[0] - 3.0).abs() < 1e-12);
        assert!((mu.y[0] - 4.0).abs() < 1e-12);
        // δ45 / (√2 · min δ1) = (7/√2) / √2 = 3.5
        assert!((mu.deg45[0] - 3.5).abs() < 1e-12);
    }
}
