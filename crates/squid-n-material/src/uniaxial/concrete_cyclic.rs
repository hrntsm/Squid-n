//! コンクリートの繰返し履歴モデル（Yassin 1994 / Concrete02 系）。
//! 単位: 応力 [N/mm²]、ひずみ無次元。構築パラメータは大きさ（正値）で与え、
//! 応答は通常の符号規約（圧縮負・引張正）で返す。

use crate::state_serde::impl_material_serde;
use crate::uniaxial::mander;
use crate::uniaxial::UniaxialMaterial;

/// 圧縮側の包絡線（骨格曲線）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ConcreteEnvelope {
    /// 修正 Kent–Park。
    KentPark {
        /// 圧縮強度 fc（正）。
        fc: f64,
        /// ピークひずみ εc0（正）。
        eps_c0: f64,
        /// 残留圧縮強度 fcu（正）。
        fcu: f64,
        /// 残留開始ひずみ εcu（正）。
        eps_cu: f64,
    },
    /// Mander (1988) の Popovics 型連続曲線。
    Mander {
        /// 圧縮強度 f'cc（正）。
        fcc: f64,
        /// f'cc 時ひずみ εcc（正）。
        eps_cc: f64,
        /// 初期弾性係数 Ec。
        ec: f64,
        /// 終局ひずみ εcu（正）。
        eps_cu: f64,
    },
    /// NewRC 式。
    NewRc {
        /// NewRC 式の圧縮包絡線。
        envelope: crate::newrc::NewRcEnvelope,
        /// 終局ひずみ εcu（正）。
        eps_cu: f64,
    },
}

impl ConcreteEnvelope {
    /// 初期接線剛性 E0。
    pub fn initial_tangent(&self) -> f64 {
        match *self {
            ConcreteEnvelope::KentPark { fc, eps_c0, .. } => 2.0 * fc / eps_c0,
            ConcreteEnvelope::Mander { ec, .. } => ec,
            ConcreteEnvelope::NewRc { envelope, .. } => envelope.ec,
        }
    }

    /// ピーク強度（大きさ）。
    pub fn peak_stress(&self) -> f64 {
        match *self {
            ConcreteEnvelope::KentPark { fc, .. } => fc,
            ConcreteEnvelope::Mander { fcc, .. } => fcc,
            ConcreteEnvelope::NewRc { envelope, .. } => envelope.fc,
        }
    }

    /// ピークひずみ（大きさ）。
    pub fn peak_strain(&self) -> f64 {
        match *self {
            ConcreteEnvelope::KentPark { eps_c0, .. } => eps_c0,
            ConcreteEnvelope::Mander { eps_cc, .. } => eps_cc,
            ConcreteEnvelope::NewRc { envelope, .. } => envelope.eps_c0,
        }
    }

    /// 圧縮包絡線の評価。`x` は圧縮ひずみの大きさ。
    fn compression(&self, x: f64) -> (f64, f64) {
        match *self {
            ConcreteEnvelope::KentPark {
                fc,
                eps_c0,
                fcu,
                eps_cu,
            } => {
                if x <= eps_c0 {
                    let r = x / eps_c0;
                    (fc * (2.0 * r - r * r), fc * (2.0 - 2.0 * r) / eps_c0)
                } else if x <= eps_cu {
                    let slope = (fcu - fc) / (eps_cu - eps_c0);
                    (fc + slope * (x - eps_c0), slope)
                } else {
                    (fcu, 0.0)
                }
            }
            ConcreteEnvelope::Mander {
                fcc,
                eps_cc,
                ec,
                eps_cu,
            } => {
                if x <= eps_cu {
                    mander::popovics_envelope(fcc, eps_cc, ec, x)
                } else {
                    let (s, _) = mander::popovics_envelope(fcc, eps_cc, ec, eps_cu);
                    (s, 0.0)
                }
            }
            ConcreteEnvelope::NewRc { envelope, eps_cu } => {
                if x <= eps_cu {
                    envelope.compression(x)
                } else {
                    let (s, _) = envelope.compression(eps_cu);
                    (s, 0.0)
                }
            }
        }
    }
}

/// コンクリート繰返し履歴モデル。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ConcreteCyclic {
    /// 圧縮側包絡線。
    pub envelope: ConcreteEnvelope,
    /// 引張強度 ft（正）。
    pub ft: f64,
    /// 引張軟化勾配 Ets（正の大きさ）。
    pub ets: f64,
    committed: CcState,
    trial: CcState,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct CcState {
    strain: f64,
    stress: f64,
    tangent: f64,
    /// 最大経験圧縮ひずみ。
    eps_min: f64,
    /// 引張側の最大経験ひずみ。
    eps_t_max: f64,
}

impl ConcreteCyclic {
    /// 修正 Kent–Park 骨格。
    pub fn kent_park(fc: f64, eps_c0: f64, fcu: f64, eps_cu: f64, ft: f64, ets: f64) -> Self {
        Self::with_envelope(
            ConcreteEnvelope::KentPark {
                fc,
                eps_c0,
                fcu,
                eps_cu,
            },
            ft,
            ets,
        )
    }

    /// Mander 骨格。
    pub fn mander(fcc: f64, eps_cc: f64, ec: f64, eps_cu: f64, ft: f64, ets: f64) -> Self {
        Self::with_envelope(
            ConcreteEnvelope::Mander {
                fcc,
                eps_cc,
                ec,
                eps_cu,
            },
            ft,
            ets,
        )
    }

    /// NewRC 骨格 + Yassin 履歴。
    pub fn newrc(fc: f64, eps_cu: f64, ft: f64, ets: f64) -> Self {
        let envelope = crate::newrc::NewRcEnvelope::new(fc);
        Self::with_envelope(ConcreteEnvelope::NewRc { envelope, eps_cu }, ft, ets)
    }

    /// Mander 骨格（拘束後パラメータを算定）。
    #[allow(clippy::too_many_arguments)]
    pub fn mander_confined(
        fco: f64,
        eps_co: f64,
        ec: f64,
        fl_eff: f64,
        eps_cu: f64,
        ft: f64,
        ets: f64,
    ) -> Self {
        let p = mander::confined_params(fco, eps_co, fl_eff);
        Self::mander(p.fcc, p.eps_cc, ec, eps_cu, ft, ets)
    }

    fn with_envelope(envelope: ConcreteEnvelope, ft: f64, ets: f64) -> Self {
        let init = CcState {
            strain: 0.0,
            stress: 0.0,
            tangent: envelope.initial_tangent(),
            eps_min: 0.0,
            eps_t_max: 0.0,
        };
        Self {
            envelope,
            ft,
            ets,
            committed: init.clone(),
            trial: init,
        }
    }

    /// Karsan–Jirsa の残留塑性ひずみ εp。
    fn plastic_strain(&self, eps_min: f64) -> f64 {
        if eps_min >= 0.0 {
            return 0.0;
        }
        let eps_c0 = self.envelope.peak_strain();
        let eta = -eps_min / eps_c0;
        let eta_p = if eta < 2.0 {
            0.145 * eta * eta + 0.13 * eta
        } else {
            0.707 * (eta - 2.0) + 0.834
        };
        -eta_p * eps_c0
    }

    /// 引張包絡線。
    fn tension_envelope(&self, e_rel: f64) -> (f64, f64) {
        let e0 = self.envelope.initial_tangent();
        if self.ft <= 0.0 || e0 <= 0.0 {
            return (0.0, 0.0);
        }
        let eps_t0 = self.ft / e0;
        if e_rel <= eps_t0 {
            (e0 * e_rel, e0)
        } else {
            let s = self.ft - self.ets * (e_rel - eps_t0);
            if s > 0.0 {
                (s, -self.ets)
            } else {
                (0.0, 0.0)
            }
        }
    }

    fn eval_state(&self, strain: f64) -> CcState {
        let c = &self.committed;
        let mut eps_min = c.eps_min;
        let mut eps_t_max = c.eps_t_max;

        let (stress, tangent) = if strain < eps_min {
            eps_min = strain;
            let (smag, dmag) = self.envelope.compression(-strain);
            (-smag, dmag)
        } else {
            let eps_p = self.plastic_strain(eps_min);
            if strain <= eps_p {
                let (smag, _) = self.envelope.compression(-eps_min);
                let sig_un = -smag;
                if eps_min < eps_p {
                    let er = sig_un / (eps_min - eps_p);
                    (er * (strain - eps_p), er)
                } else {
                    let e0 = self.envelope.initial_tangent();
                    (e0 * strain, e0)
                }
            } else {
                let e_rel = strain - eps_p;
                if e_rel >= eps_t_max {
                    eps_t_max = e_rel;
                    self.tension_envelope(e_rel)
                } else {
                    let (s_max, _) = self.tension_envelope(eps_t_max);
                    if eps_t_max > 0.0 && s_max > 0.0 {
                        let et = s_max / eps_t_max;
                        (et * e_rel, et)
                    } else {
                        (0.0, 0.0)
                    }
                }
            }
        };

        CcState {
            strain,
            stress,
            tangent,
            eps_min,
            eps_t_max,
        }
    }
}

impl UniaxialMaterial for ConcreteCyclic {
    fn reference_stress(&self) -> f64 {
        self.envelope.peak_stress()
    }

    fn reference_strain(&self) -> f64 {
        self.envelope.peak_strain()
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

    fn kp30() -> ConcreteCyclic {
        // fc=30, εc0=0.002, fcu=6 (0.2fc), εcu=0.008, ft=2, Ets=1000
        ConcreteCyclic::kent_park(30.0, 0.002, 6.0, 0.008, 2.0, 1000.0)
    }

    #[test]
    fn test_kent_park_envelope_peak_and_continuity() {
        let c = kp30();
        let (s_peak, t_peak) = c.envelope.compression(0.002);
        assert_relative_eq!(s_peak, 30.0, epsilon = 1e-9);
        assert_relative_eq!(t_peak, 0.0, epsilon = 1e-9);
        // εcu での連続性（軟化直線の終点 = 残留）。
        let (s_a, _) = c.envelope.compression(0.008 - 1e-12);
        let (s_b, _) = c.envelope.compression(0.008 + 1e-12);
        assert_relative_eq!(s_a, s_b, epsilon = 1e-6);
        assert_relative_eq!(s_b, 6.0, epsilon = 1e-6);
    }

    #[test]
    fn test_karsan_jirsa_unloading_reaches_zero_at_plastic_strain() {
        let mut c = kp30();
        // η = 2（εmin = −0.004）まで圧縮 → εp/εc0 = 0.834。
        c.trial(-0.004);
        c.commit();
        let eps_p = -0.834 * 0.002;
        let (s_at_p, _) = c.trial(eps_p);
        assert_relative_eq!(s_at_p, 0.0, epsilon = 1e-9);
        // εp と εmin の中点では圧縮応力（除荷直線上）。
        let mid = (eps_p + (-0.004)) / 2.0;
        let (s_mid, t_mid) = c.probe(mid);
        let (smag, _) = c.envelope.compression(0.004);
        assert_relative_eq!(s_mid, -smag / 2.0, max_relative = 1e-9);
        assert!(t_mid > 0.0);
    }

    #[test]
    fn test_karsan_jirsa_small_amplitude_branch() {
        // η < 2 の分岐: εmin = −0.002（η=1）→ εp/εc0 = 0.145+0.13 = 0.275。
        let mut c = kp30();
        c.trial(-0.002);
        c.commit();
        let eps_p = -0.275 * 0.002;
        let (s_at_p, _) = c.trial(eps_p);
        assert_relative_eq!(s_at_p, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn test_reloading_returns_to_envelope() {
        let mut c = kp30();
        c.trial(-0.003);
        c.commit();
        // いったん完全除荷。
        c.trial(0.0);
        c.commit();
        // 再載荷で εmin に達すると包絡線上の応力に戻る。
        let (s_re, _) = c.trial(-0.003);
        let (smag, _) = c.envelope.compression(0.003);
        assert_relative_eq!(s_re, -smag, max_relative = 1e-9);
        c.commit();
        // さらに進むと包絡線を辿る。
        let (s_more, _) = c.trial(-0.004);
        let (smag2, _) = c.envelope.compression(0.004);
        assert_relative_eq!(s_more, -smag2, max_relative = 1e-9);
    }

    #[test]
    fn test_crack_closure_recovers_compression() {
        let mut c = kp30();
        // 圧縮 → 引張（ひび割れ）→ 再圧縮。
        c.trial(-0.003);
        c.commit();
        let eps_p = c.plastic_strain(-0.003);
        // 引張域まで除荷してひび割れを進行させる。
        c.trial(eps_p + 0.001);
        c.commit();
        // ひび割れ閉鎖点 εp では応力ゼロ、そこを下回ると圧縮を伝達する。
        let (s_at_p, _) = c.probe(eps_p);
        assert_relative_eq!(s_at_p, 0.0, epsilon = 1e-9);
        let (s_comp, _) = c.probe(eps_p - 0.0005);
        assert!(s_comp < 0.0, "ひび割れ閉鎖後は圧縮を伝達: {s_comp}");
    }

    #[test]
    fn test_tension_softening_and_stiffness_degradation() {
        let mut c = kp30();
        let e0 = c.envelope.initial_tangent();
        let eps_t0 = 2.0 / e0;
        // ひび割れ点で σ = ft。
        let (s_cr, _) = c.trial(eps_t0);
        assert_relative_eq!(s_cr, 2.0, max_relative = 1e-9);
        c.commit();
        // 軟化: σ = ft − Ets·Δε。
        let (s_soft, t_soft) = c.trial(eps_t0 + 0.001);
        assert_relative_eq!(s_soft, 2.0 - 1000.0 * 0.001, max_relative = 1e-9);
        assert_relative_eq!(t_soft, -1000.0, max_relative = 1e-9);
        c.commit();
        // 除荷は原点への割線（剛性劣化: E0 より小さい）。
        let (_, t_unload) = c.trial((eps_t0 + 0.001) * 0.5);
        assert!(t_unload > 0.0 && t_unload < e0);
        // 完全軟化後は応力ゼロ。
        let mut c2 = kp30();
        let (s_zero, _) = c2.trial(eps_t0 + 0.003);
        assert_relative_eq!(s_zero, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn test_mander_envelope_peak_via_popovics() {
        // Mander 骨格: ピークで σ = f'cc（Popovics の恒等式）。
        let mut c = ConcreteCyclic::mander(46.95, 0.00765, 25000.0, 0.02, 2.0, 1000.0);
        let (s_peak, _) = c.trial(-0.00765);
        assert_relative_eq!(s_peak, -46.95, max_relative = 1e-9);
    }

    #[test]
    fn test_mander_confined_stronger_than_unconfined() {
        // 有効拘束圧があると強度・靱性が上がる。
        let mut un = ConcreteCyclic::mander_confined(30.0, 0.002, 25000.0, 0.0, 0.01, 2.0, 1000.0);
        let mut co = ConcreteCyclic::mander_confined(30.0, 0.002, 25000.0, 3.0, 0.01, 2.0, 1000.0);
        let (s_un, _) = un.trial(-0.004);
        let (s_co, _) = co.trial(-0.004);
        assert!(
            s_co < s_un,
            "拘束ありの方が高応力（負に大きい）: un={s_un}, co={s_co}"
        );
    }

    #[test]
    fn test_newrc_envelope_peak_stress() {
        // NewRC 骨格: ピークひずみで σ = −fc。
        let mut c = ConcreteCyclic::newrc(30.0, 0.01, 2.0, 1000.0);
        let eps_c0 = c.envelope.peak_strain();
        let (s_peak, _) = c.trial(-eps_c0);
        assert_relative_eq!(s_peak, -30.0, max_relative = 1e-6);
    }

    #[test]
    fn test_newrc_karsan_jirsa_unloading_reaches_zero_at_plastic_strain() {
        // η = 2（εmin = −2·εc0）まで圧縮 → εp/εc0 = 0.834（Yassin 1994）。
        let mut c = ConcreteCyclic::newrc(30.0, 0.01, 2.0, 1000.0);
        let eps_c0 = c.envelope.peak_strain();
        c.trial(-2.0 * eps_c0);
        c.commit();
        let eps_p = -0.834 * eps_c0;
        let (s_at_p, _) = c.trial(eps_p);
        assert_relative_eq!(s_at_p, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn test_newrc_probe_matches_trial_without_mutating_state() {
        // probe は trial と数学的に同一の結果を返し、状態を書き換えない
        // （NewRC 骨格 + Yassin 履歴でも Kent–Park と同様に成立すること）。
        let mut c = ConcreteCyclic::newrc(30.0, 0.01, 2.0, 1000.0);
        let eps_c0 = c.envelope.peak_strain();
        c.trial(-2.0 * eps_c0);
        c.commit();
        c.trial(eps_c0 * 0.5);
        c.commit();

        for &probe_strain in &[-eps_c0, -eps_c0 * 0.5, eps_c0 * 0.2] {
            let before = c.probe(probe_strain);
            assert_eq!(before, c.probe(probe_strain));
            let mut clone_for_trial = c.clone();
            let via_trial = clone_for_trial.trial(probe_strain);
            assert_eq!(before, via_trial, "probe は trial と完全一致すること");
        }
    }

    #[test]
    fn test_probe_matches_trial_without_mutating_state() {
        // probe は trial と数学的に同一の結果を返し、状態を書き換えない
        // （Karsan–Jirsa 除荷・引張剛性劣化の分岐を経た状態で確認）。
        let mut c = kp30();
        c.trial(-0.004);
        c.commit();
        c.trial(0.001);
        c.commit();

        for &probe_strain in &[-0.002, -0.0005, 0.0005] {
            let before = c.probe(probe_strain);
            assert_eq!(before, c.probe(probe_strain));
            let mut clone_for_trial = c.clone();
            let via_trial = clone_for_trial.trial(probe_strain);
            assert_eq!(before, via_trial, "probe は trial と完全一致すること");
        }
    }

    #[test]
    fn test_commit_revert() {
        let mut c = kp30();
        c.trial(-0.002);
        c.commit();
        c.trial(-0.004);
        c.revert();
        // revert 後は εmin = −0.002 のまま（除荷は −0.002 からの割線）。
        let eps_p = c.plastic_strain(-0.002);
        let (s, _) = c.trial(eps_p);
        assert_relative_eq!(s, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn test_tangent_matches_finite_difference_on_smooth_segments() {
        let mut c = kp30();
        c.trial(-0.004);
        c.commit();
        c.trial(0.0005);
        c.commit();
        // 各分岐の内部（境界を跨がない点）で接線 = 数値微分。
        for &eps in &[-0.0045, -0.003, -0.0008, 0.0002] {
            let (_, tan) = c.probe(eps);
            let h = 1e-9;
            let (s1, _) = c.probe(eps + h);
            let (s0, _) = c.probe(eps - h);
            let fd = (s1 - s0) / (2.0 * h);
            assert_relative_eq!(tan, fd, max_relative = 1e-3, epsilon = 1e-3);
        }
    }
}
