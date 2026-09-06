//! 鉄骨大梁の座屈を考慮した履歴則（井戸田ほか 2015）とその耐力比算定。

use crate::state_serde::impl_material_serde;
use crate::uniaxial::UniaxialMaterial;

/// 横座屈で耐力が決まる H 形鋼梁の最大曲げ耐力比 `Mu/Mp`（井戸田ほか 2015 の式）。
/// 係数は原典既定: `cres=0.0`、`f=1.0`、`kres=0.3`、`kdef=1.0`。
pub fn lateral_buckling_mu_ratio(lambda_b: f64, kappa: f64, w_f: f64, e_lambda_b: f64) -> f64 {
    let lambda_b = lambda_b.max(1e-6);
    let e_lambda_b = e_lambda_b.max(1e-6);
    let kappa = kappa.clamp(-1.0, 1.0);
    const C_RES: f64 = 0.0;
    const K_DEF: f64 = 1.0;
    let q_kappa = if kappa <= 0.0 {
        -0.1 * kappa + 0.065
    } else {
        0.065
    };
    let r = if kappa <= 0.0 { 0.5 * kappa + 1.0 } else { 1.0 };
    let alpha_lambda = -0.2 * kappa - 0.25;
    let lambda_c = ((lambda_b / e_lambda_b) + w_f.powi(3)).max(0.0).cbrt();
    let h0 = if lambda_c <= 1.25 {
        alpha_lambda * (lambda_c - 1.25) + 1.0
    } else {
        1.0
    };
    let c_def = q_kappa * K_DEF.powf(r);
    let a = 1.0 + c_def * lambda_b + (1.0 + C_RES) * lambda_b * lambda_b;
    let disc = (a * a - 4.0 * lambda_b * lambda_b * (1.0 + C_RES * lambda_b * lambda_b)).max(0.0);
    let denom = a + disc.sqrt();
    if denom <= 1e-12 {
        1.0
    } else {
        (2.0 * h0 / denom).clamp(0.05, 5.0)
    }
}

/// 鉄骨大梁の座屈を考慮した履歴則（井戸田ほか 2015）。
/// 除荷は孟・大井・高梨の RO モデル（γ=5, Φ=0.5）。再載荷は経験最大点指向。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SteelBuckling {
    /// 初期（弾性）剛性 k1 [力/回転角]。
    pub k1: f64,
    /// 全塑性耐力 Mp。
    pub mp: f64,
    /// 最大耐力 Mu = mu_ratio·Mp（mu_ratio≥1）。
    pub mu: f64,
    /// 最大耐力に至る回転角 θu。
    pub theta_u: f64,
    /// 耐力劣化開始の回転角 θ_static（≥θu）。
    pub theta_static: f64,
    /// 残留耐力に至る回転角 θ_res（>θ_static）。
    pub theta_res: f64,
    /// 残留耐力比（Mu に対する。0<mu_res≤1）。
    pub mu_res: f64,
    committed: SbState,
    trial: SbState,
}

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
struct SbState {
    theta: f64,
    m: f64,
    tangent: f64,
    /// 経験最大回転角（正）。
    theta_max_pos: f64,
    /// 経験最大回転角（負）。
    theta_max_neg: f64,
    /// 直近の反転点。
    theta_r: f64,
    m_r: f64,
    /// 直近の進行方向（+1/-1/0）。
    dir: f64,
    /// 骨格上にいるか。
    on_backbone: bool,
}

impl SteelBuckling {
    /// RO モデル諸元。
    const RO_GAMMA: f64 = 5.0;
    const RO_PHI: f64 = 0.5;

    /// 詳細指定のコンストラクタ。
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        k1: f64,
        mp: f64,
        mu_ratio: f64,
        theta_u: f64,
        theta_static: f64,
        theta_res: f64,
        mu_res: f64,
    ) -> Self {
        let k1 = k1.max(1e-9);
        let mp = mp.max(1e-9);
        let mu = mp * mu_ratio.max(1.0);
        let theta_y = mp / k1;
        let theta_u = theta_u.max(theta_y * 1.0001);
        let theta_static = theta_static.max(theta_u);
        let theta_res = theta_res.max(theta_static * 1.0001);
        let init = SbState {
            tangent: k1,
            on_backbone: true,
            ..Default::default()
        };
        Self {
            k1,
            mp,
            mu,
            theta_u,
            theta_static,
            theta_res,
            mu_res: mu_res.clamp(0.05, 1.0),
            committed: init,
            trial: init,
        }
    }

    /// 既定諸元。
    pub fn with_defaults(k1: f64, mp: f64, mu_ratio: f64) -> Self {
        let theta_y = mp.max(1e-9) / k1.max(1e-9);
        Self::new(
            k1,
            mp,
            mu_ratio,
            2.0 * theta_y,
            4.0 * theta_y,
            10.0 * theta_y,
            0.5,
        )
    }

    /// 骨格（奇対称）。
    fn envelope(&self, theta: f64) -> (f64, f64) {
        let s = theta.signum();
        let t = theta.abs();
        let theta_y = self.mp / self.k1;
        let (m, k) = if t <= theta_y {
            (self.k1 * t, self.k1)
        } else if t <= self.theta_u {
            let kh = (self.mu - self.mp) / (self.theta_u - theta_y);
            (self.mp + kh * (t - theta_y), kh)
        } else if t <= self.theta_static {
            (self.mu, 0.0)
        } else if t <= self.theta_res {
            let kdeg = (self.mu_res * self.mu - self.mu) / (self.theta_res - self.theta_static);
            (self.mu + kdeg * (t - self.theta_static), kdeg)
        } else {
            (self.mu_res * self.mu, 0.0)
        };
        (s * m, k)
    }

    /// 反転点からの RO 除荷枝。
    /// RO: Δθ = (ΔM/k1)·(1 + Φ·|ΔM/Mp|^(γ−1))。ΔM を Newton で解く。
    fn ro_branch(&self, theta: f64, theta_r: f64, m_r: f64) -> (f64, f64) {
        let dtheta = theta - theta_r;
        let mut dm = self.k1 * dtheta;
        let g = Self::RO_GAMMA;
        let phi = Self::RO_PHI;
        for _ in 0..30 {
            let ratio = (dm / self.mp).abs();
            let f = (dm / self.k1) * (1.0 + phi * ratio.powf(g - 1.0)) - dtheta;
            let fp = (1.0 / self.k1) * (1.0 + phi * g * ratio.powf(g - 1.0));
            if fp.abs() < 1e-30 {
                break;
            }
            let step = f / fp;
            dm -= step;
            if step.abs() < 1e-9 * self.mp.max(1.0) {
                break;
            }
        }
        let ratio = (dm / self.mp).abs();
        let tangent = self.k1 / (1.0 + phi * g * ratio.powf(g - 1.0));
        (m_r + dm, tangent.max(1e-6))
    }

    fn eval_state(&self, theta: f64) -> SbState {
        let c = self.committed;
        let dir = (theta - c.theta).signum();
        if dir == 0.0 {
            return c;
        }
        let beyond_pos = dir > 0.0 && theta >= c.theta_max_pos;
        let beyond_neg = dir < 0.0 && theta <= c.theta_max_neg;
        let mut st = c;
        st.dir = dir;
        if beyond_pos || beyond_neg {
            let (m, k) = self.envelope(theta);
            st.m = m;
            st.tangent = k;
            st.theta = theta;
            st.on_backbone = true;
            st.theta_max_pos = st.theta_max_pos.max(theta);
            st.theta_max_neg = st.theta_max_neg.min(theta);
            return st;
        }
        if c.on_backbone || (c.dir != 0.0 && dir != c.dir) {
            st.theta_r = c.theta;
            st.m_r = c.m;
        }
        st.on_backbone = false;
        let (m, k) = self.ro_branch(theta, st.theta_r, st.m_r);
        st.m = m;
        st.tangent = k;
        st.theta = theta;
        st
    }
}

impl UniaxialMaterial for SteelBuckling {
    fn set_yield(&mut self, fy: f64) {
        let ratio = if self.mp > 0.0 {
            self.mu / self.mp
        } else {
            1.0
        };
        self.mp = fy.max(1e-9);
        self.mu = self.mp * ratio;
    }

    fn reference_stress(&self) -> f64 {
        self.mp
    }

    fn reference_strain(&self) -> f64 {
        if self.k1 > 0.0 {
            self.mp / self.k1
        } else {
            0.0
        }
    }

    fn trial(&mut self, theta: f64) -> (f64, f64) {
        let st = self.eval_state(theta);
        self.trial = st;
        (st.m, st.tangent)
    }

    fn probe(&self, theta: f64) -> (f64, f64) {
        let st = self.eval_state(theta);
        (st.m, st.tangent)
    }

    fn commit(&mut self) {
        self.committed = self.trial;
    }

    fn revert(&mut self) {
        self.trial = self.committed;
    }

    impl_material_serde!();
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_lateral_buckling_mu_ratio_slender_reduces() {
        // 細長比が大きいほど Mu/Mp は小さくなる（横座屈で耐力低下）。
        let stocky = lateral_buckling_mu_ratio(0.3, 0.0, 0.5, 0.3);
        let slender = lateral_buckling_mu_ratio(1.5, 0.0, 0.5, 0.3);
        assert!(stocky > slender, "stocky={stocky}, slender={slender}");
        assert!(slender > 0.0 && stocky <= 5.0);
    }

    #[test]
    fn test_steel_buckling_backbone_peak_then_degrade() {
        // 骨格: 弾性→硬化→最大耐力 Mu→劣化。単調載荷で Mu 到達後に耐力低下。
        let k1 = 1000.0;
        let mp = 100.0;
        let mut m = SteelBuckling::with_defaults(k1, mp, 1.3);
        let theta_y = mp / k1;
        // 弾性点。
        let (m_el, _) = m.trial(0.5 * theta_y);
        m.commit();
        assert_relative_eq!(m_el, 0.5 * mp, epsilon = 1e-6);
        // ピーク（θu=2θy 付近）。
        let (m_peak, _) = m.trial(2.0 * theta_y);
        m.commit();
        assert_relative_eq!(m_peak, 1.3 * mp, epsilon = 1e-3);
        // 劣化域（θ_res=10θy 手前）。耐力が Mu より低下。
        let (m_deg, _) = m.trial(8.0 * theta_y);
        m.commit();
        assert!(
            m_deg < m_peak && m_deg > 0.5 * 1.3 * mp * 0.99,
            "degradation: peak={m_peak}, deg={m_deg}"
        );
    }

    #[test]
    fn test_steel_buckling_ro_unload_initial_stiffness() {
        // RO 除荷は反転点で初期剛性 k1 から立ち上がる。
        let k1 = 1000.0;
        let mp = 100.0;
        let mut m = SteelBuckling::with_defaults(k1, mp, 1.2);
        let theta_y = mp / k1;
        m.trial(3.0 * theta_y);
        m.commit();
        // 反転直後の微小除荷: 接線 ≈ k1。
        let (_, k) = m.trial(3.0 * theta_y - 1e-6 * theta_y);
        assert_relative_eq!(k, k1, epsilon = k1 * 0.02);
    }

    #[test]
    fn test_steel_buckling_hysteretic_energy_positive() {
        // 1 サイクルで履歴ループ面積（散逸エネルギー）が正。
        let k1 = 1000.0;
        let mp = 100.0;
        let mut m = SteelBuckling::with_defaults(k1, mp, 1.2);
        let theta_y = mp / k1;
        let amp = 3.0 * theta_y;
        let path: Vec<f64> = (0..=80)
            .map(|i| {
                let phase = i as f64 / 20.0 * std::f64::consts::PI;
                amp * phase.sin()
            })
            .collect();
        let mut energy = 0.0;
        let mut prev = (0.0, 0.0);
        for &th in &path {
            let (mm, _) = m.trial(th);
            m.commit();
            energy += 0.5 * (prev.1 + mm) * (th - prev.0);
            prev = (th, mm);
        }
        assert!(
            energy > 0.0,
            "dissipated energy should be positive: {energy}"
        );
    }

    #[test]
    fn test_probe_matches_trial_without_mutating_state() {
        // probe は trial と数学的に同一の結果を返し、状態を書き換えない
        // （RO 除荷枝を経た状態で確認）。
        let mut m = SteelBuckling::with_defaults(1000.0, 100.0, 1.2);
        let theta_y = 100.0 / 1000.0;
        m.trial(3.0 * theta_y);
        m.commit();

        let probe_theta = 2.5 * theta_y; // 除荷（RO 枝）
        let before = m.probe(probe_theta);
        assert_eq!(before, m.probe(probe_theta));

        let mut clone_for_trial = m.clone();
        let via_trial = clone_for_trial.trial(probe_theta);
        assert_eq!(before, via_trial, "probe は trial と完全一致すること");

        let after_probe = m.trial(probe_theta);
        assert_eq!(after_probe, via_trial);
    }

    #[test]
    fn test_steel_buckling_commit_revert() {
        // 硬化域（θy=0.1 < θ < θu=0.2）で単調に耐力増加する区間を用いる。
        let mut m = SteelBuckling::with_defaults(1000.0, 100.0, 1.2);
        m.trial(0.12);
        m.commit();
        let (s1, _) = m.trial(0.18);
        m.revert();
        let (s2, _) = m.trial(0.13);
        assert!(
            s2 < s1,
            "revert then smaller θ → smaller M: s1={s1}, s2={s2}"
        );
    }
}
