//! コンクリートの一軸履歴モデル（設計書 §7）。

use crate::state_serde::impl_material_serde;
use crate::uniaxial::UniaxialMaterial;

/// コンクリートの一軸履歴モデル。
/// 除荷・再載荷は原点指向。単位: fc/ft [N/mm²], ec0/ecu [ひずみ(負)]。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Concrete {
    pub fc: f64,
    pub ec0: f64,
    pub ecu: f64,
    pub ft: f64,
    pub tension_stiffening: f64,
    pub residual_ratio: f64,
    committed: ConcreteState,
    trial: ConcreteState,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[allow(dead_code)]
struct ConcreteState {
    strain: f64,
    stress: f64,
    tangent: f64,
    /// 最大経験圧縮ひずみ。
    max_comp_strain: f64,
    /// 最大経験引張ひずみ。
    max_tens_strain: f64,
    is_cracked: bool,
}

impl Concrete {
    /// AIJ 系の代表値で生成する。
    pub fn new(fc: f64, ft: f64) -> Self {
        Self::with_params(fc, -0.002, -0.0035, ft, 0.5, 0.0)
    }

    pub fn with_params(
        fc: f64,
        ec0: f64,
        ecu: f64,
        ft: f64,
        tension_stiffening: f64,
        residual_ratio: f64,
    ) -> Self {
        let init = ConcreteState {
            strain: 0.0,
            stress: 0.0,
            tangent: 0.0,
            max_comp_strain: 0.0,
            max_tens_strain: 0.0,
            is_cracked: false,
        };
        Self {
            fc,
            ec0,
            ecu,
            ft,
            tension_stiffening,
            residual_ratio,
            committed: init.clone(),
            trial: init,
        }
    }

    /// 初期接線剛性 E0 = 2·fc/|ec0|。
    pub fn e0(&self) -> f64 {
        2.0 * self.fc / self.ec0.abs()
    }

    /// せん断弾性率 G0 = E0/(2(1+ν))。ν=0.2。
    pub fn e0_shear(&self) -> f64 {
        self.e0() / (2.0 * (1.0 + 0.2))
    }

    /// 圧縮包絡線。
    fn envelope_compression(&self, strain: f64) -> (f64, f64) {
        if strain >= self.ec0 {
            let r = strain / self.ec0;
            let stress = -self.fc * (2.0 * r - r * r);
            let tangent = -self.fc * (2.0 - 2.0 * r) / self.ec0;
            (stress, tangent)
        } else if strain >= self.ecu {
            let slope = self.fc * (1.0 - self.residual_ratio) / (self.ecu - self.ec0);
            let stress = -self.fc + slope * (strain - self.ec0);
            (stress, slope)
        } else {
            (-self.fc * self.residual_ratio, 0.0)
        }
    }

    /// 引張包絡線。
    fn envelope_tension(&self, strain: f64) -> (f64, f64) {
        let e0 = self.e0();
        let eps_cr = self.ft / e0;
        if strain <= eps_cr {
            (e0 * strain, e0)
        } else {
            let arg = -self.tension_stiffening * (strain / eps_cr - 1.0);
            let stress = self.ft * arg.exp();
            let tangent = -self.ft * (self.tension_stiffening / eps_cr) * arg.exp();
            (stress, tangent)
        }
    }

    fn eval_state(&self, strain: f64) -> ConcreteState {
        let c = &self.committed;
        let e0 = self.e0();
        let eps_cr = self.ft / e0;

        let (stress, tangent, max_comp, max_tens, is_cracked) = if strain <= 0.0 {
            let mut max_comp = c.max_comp_strain;
            let (s, t) = if strain < c.max_comp_strain {
                max_comp = strain;
                self.envelope_compression(strain)
            } else {
                if c.max_comp_strain < 0.0 {
                    let (sig_m, _) = self.envelope_compression(c.max_comp_strain);
                    let ku = sig_m / c.max_comp_strain;
                    (ku * strain, ku)
                } else {
                    (e0 * strain, e0)
                }
            };
            (s, t, max_comp, c.max_tens_strain, c.is_cracked)
        } else {
            let mut max_tens = c.max_tens_strain;
            let mut cracked = c.is_cracked;
            let (s, t) = if !cracked && strain <= eps_cr {
                (e0 * strain, e0)
            } else if !cracked && strain > eps_cr {
                cracked = true;
                max_tens = strain;
                self.envelope_tension(strain)
            } else {
                if strain > c.max_tens_strain {
                    max_tens = strain;
                    self.envelope_tension(strain)
                } else {
                    if c.max_tens_strain > 0.0 {
                        let (sig_m, _) = self.envelope_tension(c.max_tens_strain);
                        let kt = sig_m / c.max_tens_strain;
                        (kt * strain, kt)
                    } else {
                        (0.0, 0.0)
                    }
                }
            };
            (s, t, c.max_comp_strain, max_tens, cracked)
        };

        ConcreteState {
            strain,
            stress,
            tangent,
            max_comp_strain: max_comp,
            max_tens_strain: max_tens,
            is_cracked,
        }
    }
}

impl UniaxialMaterial for Concrete {
    fn reference_stress(&self) -> f64 {
        self.fc
    }

    fn reference_strain(&self) -> f64 {
        self.ec0.abs()
    }

    fn trial(&mut self, strain: f64) -> (f64, f64) {
        self.trial = self.eval_state(strain);
        (self.trial.stress, self.trial.tangent)
    }

    fn probe(&self, strain: f64) -> (f64, f64) {
        let s = self.eval_state(strain);
        (s.stress, s.tangent)
    }

    fn commit(&mut self) {
        self.committed = self.trial.clone();
    }

    fn revert(&mut self) {
        self.trial = self.committed.clone();
    }

    impl_material_serde!();
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_concrete_compression_peak() {
        let mut c = Concrete::new(30.0, 2.0);
        let (stress, t) = c.trial(-0.002);
        assert_relative_eq!(stress, -30.0, epsilon = 1e-6);
        assert_relative_eq!(t, 0.0, epsilon = 1e-3);
    }

    #[test]
    fn test_concrete_compression_initial_tangent_positive() {
        let mut c = Concrete::new(30.0, 2.0);
        let (_, t) = c.trial(-1e-9);
        let e0 = 2.0 * 30.0 / 0.002;
        assert_relative_eq!(t, e0, epsilon = 1.0);
    }

    #[test]
    fn test_concrete_softening_direction() {
        // ピーク後ひずみ進行で応力は 0 側へ（|σ| は減少）
        let mut c = Concrete::new(30.0, 2.0);
        c.trial(-0.002);
        c.commit();
        let (s_mid, _) = c.trial(-0.003);
        let (s_u, _) = c.trial(-0.0035);
        assert!(
            s_mid > -30.0 && s_u > s_mid,
            "softening must reduce |stress|: mid={}, u={}",
            s_mid,
            s_u
        );
        assert_relative_eq!(s_u, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_concrete_continuity_at_ecu() {
        let c = Concrete::new(30.0, 2.0);
        let (s_soft, _) = c.envelope_compression(-0.0035);
        let (s_res, _) = c.envelope_compression(-0.0036);
        assert_relative_eq!(s_soft, s_res, epsilon = 1e-6);
    }

    #[test]
    fn test_concrete_tension_crack_detection() {
        let mut c = Concrete::new(30.0, 2.0);
        let e0 = 2.0 * 30.0 / 0.002;
        let eps_cr = 2.0 / e0;
        // ひび割れ前: 弾性
        let (s_el, _) = c.trial(eps_cr * 0.5);
        assert_relative_eq!(s_el, e0 * eps_cr * 0.5, epsilon = 1e-9);
        // ひび割れ点: σ = ft
        let (s_cr, _) = c.trial(eps_cr);
        assert_relative_eq!(s_cr, 2.0, epsilon = 1e-6);
        // ひび割れ後: テンションスティフニング（ft から指数減衰、ただし ft 未満）
        c.commit();
        let (s_ts, _) = c.trial(eps_cr * 3.0);
        assert!(s_ts > 0.0 && s_ts < 2.0, "tension stiffening: {}", s_ts);
    }

    #[test]
    fn test_probe_matches_trial_without_mutating_state() {
        // probe は trial と数学的に同一の結果を返し、状態を書き換えない
        // （除荷・再載荷の割線分岐を経た状態で確認）。
        let mut c = Concrete::new(30.0, 2.0);
        c.trial(-0.0025);
        c.commit();

        let probe_strain = -0.001; // 除荷側（割線剛性の分岐）
        let before = c.probe(probe_strain);
        assert_eq!(before, c.probe(probe_strain));

        let mut clone_for_trial = c.clone();
        let via_trial = clone_for_trial.trial(probe_strain);
        assert_eq!(before, via_trial, "probe は trial と完全一致すること");

        let after_probe = c.trial(probe_strain);
        assert_eq!(after_probe, via_trial);
    }

    #[test]
    fn test_concrete_commit_revert() {
        let mut c = Concrete::new(30.0, 2.0);
        c.trial(-0.001);
        c.commit();
        c.trial(-0.002);
        c.revert();
        let (stress, _) = c.trial(-0.0005);
        assert!(stress < 0.0);
    }
}
