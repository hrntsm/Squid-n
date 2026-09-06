//! NewRC コンクリート構成則（NewRC 式）。
use crate::state_serde::impl_material_serde;
use crate::uniaxial::UniaxialMaterial;

/// N/mm² → kgf/cm² の換算係数。
const NMM2_TO_KGFCM2: f64 = 1.0 / 0.0980665;

/// NewRC 式の圧縮包絡線（有理式）。単位: fc [N/mm²]。圧縮を正の大きさで評価する。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct NewRcEnvelope {
    /// コンクリート強度 Fc [N/mm²]。
    pub fc: f64,
    /// 初期接線 Ec [N/mm²]。
    pub ec: f64,
    /// 圧縮強度時ひずみ εc0（正）。
    pub eps_c0: f64,
    /// NewRC 係数 A（無次元）。
    a: f64,
    /// NewRC 係数 D（無次元）。
    d_coef: f64,
}

impl NewRcEnvelope {
    /// `fc` [N/mm²]、気乾単位体積重量 γ=2.4 [t/m³] で評価する。
    pub fn new(fc: f64) -> Self {
        Self::with_gamma(fc, 2.4)
    }

    /// `fc` [N/mm²]、`gamma` は気乾単位体積重量 [t/m³]。
    pub fn with_gamma(fc: f64, gamma: f64) -> Self {
        let sigma_b = fc.max(1e-6) * NMM2_TO_KGFCM2;
        let eps_c0 = 0.5243 * sigma_b.powf(0.25) * 1e-3;
        let ec_kgf = 4.0 * 1.0 * (sigma_b / 1000.0).powf(1.0 / 3.0) * 1e5 * (gamma / 2.4).powi(2);
        let sigma_cb = sigma_b;
        let a = ec_kgf * eps_c0 / sigma_cb;
        let d_coef = 1.50 + 1.68e-3 * sigma_b;
        let ec = ec_kgf * 0.0980665;
        Self {
            fc,
            ec,
            eps_c0,
            a,
            d_coef,
        }
    }

    /// NewRC 圧縮包絡線の応力比とその微分（正規化ひずみ X に対して）。
    fn ratio(&self, capital_x: f64) -> (f64, f64) {
        let a = self.a;
        let d = self.d_coef;
        let num = a * capital_x + (d - 1.0) * capital_x * capital_x;
        let den = 1.0 + (a - 2.0) * capital_x + d * capital_x * capital_x;
        let ratio = num / den;
        let num_p = a + 2.0 * (d - 1.0) * capital_x;
        let den_p = (a - 2.0) + 2.0 * d * capital_x;
        let dratio = (num_p * den - num * den_p) / (den * den);
        (ratio, dratio)
    }

    /// 圧縮包絡線の評価。`x` は圧縮ひずみの大きさ。
    /// 戻り値は (応力の大きさ, 接線)。
    ///
    /// 原点以下では σ=0・接線 = Ec を返す。終局域のゼロクランプと区別するため。
    pub fn compression(&self, x: f64) -> (f64, f64) {
        if x <= 0.0 {
            return (0.0, self.ec);
        }
        let capital_x = x / self.eps_c0;
        let (ratio, dratio) = self.ratio(capital_x);
        let stress = ratio * self.fc;
        if stress <= 0.0 {
            (0.0, 0.0)
        } else {
            let tangent = (self.fc / self.eps_c0) * dratio;
            (stress, tangent.max(0.0))
        }
    }
}

/// コンクリート履歴の除荷則（静的=逆行型／動的=原点指向型）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum ConcreteHysteresis {
    /// 原点指向型。既定。
    #[default]
    OriginOriented,
    /// 逆行型。
    Retrace,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[allow(dead_code)]
struct NewRcState {
    strain: f64,
    stress: f64,
    tangent: f64,
    max_comp_strain: f64,
    max_tens_strain: f64,
    is_cracked: bool,
}

/// NewRC コンクリート構成則。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ConcreteNewRc {
    /// コンクリート強度 Fc [N/mm²]。
    pub fc: f64,
    /// 引張強度 ft [N/mm²]。
    pub ft: f64,
    /// 初期接線 Ec [N/mm²]。
    pub ec: f64,
    /// 圧縮強度時ひずみ εc0（正）。
    eps_c0: f64,
    /// 圧縮包絡線。
    envelope: NewRcEnvelope,
    /// 除荷則。
    #[serde(default)]
    hysteresis: ConcreteHysteresis,
    committed: NewRcState,
    trial: NewRcState,
}

impl ConcreteNewRc {
    /// `fc`,`ft` [N/mm²]。
    pub fn new(fc: f64, ft: f64) -> Self {
        Self::with_gamma(fc, ft, 2.4)
    }

    pub fn with_gamma(fc: f64, ft: f64, gamma: f64) -> Self {
        let envelope = NewRcEnvelope::with_gamma(fc, gamma);
        let ec = envelope.ec;
        let eps_c0 = envelope.eps_c0;
        let init = NewRcState {
            strain: 0.0,
            stress: 0.0,
            tangent: ec,
            max_comp_strain: 0.0,
            max_tens_strain: 0.0,
            is_cracked: false,
        };
        Self {
            fc,
            ft,
            ec,
            eps_c0,
            envelope,
            hysteresis: ConcreteHysteresis::default(),
            committed: init.clone(),
            trial: init,
        }
    }

    /// 圧縮包絡線。
    fn envelope_compression(&self, strain: f64) -> (f64, f64) {
        let (smag, tmag) = self.envelope.compression(-strain);
        (-smag, tmag)
    }

    fn eps_cr(&self) -> f64 {
        if self.ec > 0.0 {
            self.ft / self.ec
        } else {
            0.0
        }
    }

    fn eval_state(&self, strain: f64) -> NewRcState {
        let c = &self.committed;
        let (stress, tangent, max_comp, max_tens, cracked) = if strain <= 0.0 {
            let mut max_comp = c.max_comp_strain;
            let (s, t) = if self.hysteresis == ConcreteHysteresis::Retrace {
                if strain < max_comp {
                    max_comp = strain;
                }
                self.envelope_compression(strain)
            } else if strain < c.max_comp_strain {
                max_comp = strain;
                self.envelope_compression(strain)
            } else if c.max_comp_strain < 0.0 {
                let (sig_m, _) = self.envelope_compression(c.max_comp_strain);
                let ku = sig_m / c.max_comp_strain;
                (ku * strain, ku)
            } else {
                (self.ec * strain, self.ec)
            };
            (s, t, max_comp, c.max_tens_strain, c.is_cracked)
        } else {
            let eps_cr = self.eps_cr();
            let mut cracked = c.is_cracked;
            let mut max_tens = c.max_tens_strain;
            let (s, t) = if !cracked && strain <= eps_cr {
                (self.ec * strain, self.ec)
            } else {
                cracked = true;
                max_tens = max_tens.max(strain);
                (0.0, 0.0)
            };
            (s, t, c.max_comp_strain, max_tens, cracked)
        };
        NewRcState {
            strain,
            stress,
            tangent,
            max_comp_strain: max_comp,
            max_tens_strain: max_tens,
            is_cracked: cracked,
        }
    }
}

impl UniaxialMaterial for ConcreteNewRc {
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

    fn reference_stress(&self) -> f64 {
        self.fc
    }

    fn reference_strain(&self) -> f64 {
        self.eps_c0
    }

    fn set_concrete_hysteresis(&mut self, dynamic: bool) {
        self.hysteresis = if dynamic {
            ConcreteHysteresis::OriginOriented
        } else {
            ConcreteHysteresis::Retrace
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_newrc_peak_at_ec0() {
        let c = ConcreteNewRc::new(30.0, 2.0);
        // ピーク（εc0）で σ=-fc（比=1）。
        let (stress, _) = c.envelope_compression(-c.eps_c0);
        assert_relative_eq!(stress, -30.0, max_relative = 1e-6);
    }

    #[test]
    fn test_newrc_initial_tangent_is_ec() {
        let c = ConcreteNewRc::new(30.0, 2.0);
        let (_, t) = c.envelope_compression(-1e-9);
        // ε=0 近傍の接線は Ec。
        assert_relative_eq!(t, c.ec, max_relative = 1e-3);
        // Ec は常識的な範囲（普通コンクリート 2〜3×10⁴ N/mm² 程度）。
        assert!(c.ec > 2.0e4 && c.ec < 3.5e4, "Ec={}", c.ec);
    }

    /// **ひずみちょうど 0** の接線が Ec であること（逆行型・原点指向型とも）。
    ///
    /// ひずみ 0 の接線はファイバー断面の初期弾性剛性としてそのまま使われる
    /// （ファイバー梁は生成時に `trial(0.0)` を呼んで初期接線をキャッシュする）。
    /// ここが 0 になると RC 断面の弾性曲げ剛性が主筋分だけに縮み、塑性化域考慮
    /// ファイバー梁の「弾性状態ではヒンジ回転 γ=0」という整合が破れて接線剛性が
    /// 負になる（増分解析が長期載荷の時点で解けなくなる）。ε≒0（-1e-9）ではなく
    /// **厳密な 0** で確かめる（有理式は x=0 で σ=0 となり、終局域の
    /// 「σ≤0 なら接線 0」クランプに掛かり得るため）。
    #[test]
    fn test_newrc_tangent_at_exactly_zero_strain_is_ec() {
        for dynamic in [false, true] {
            let mut c = ConcreteNewRc::new(21.0, 2.0);
            c.set_concrete_hysteresis(dynamic);
            let (s, t) = c.trial(0.0);
            assert_relative_eq!(s, 0.0, epsilon = 1e-12);
            assert_relative_eq!(t, c.ec, max_relative = 1e-12);
            // probe（非破壊評価）も同じ値を返す。
            let (ps, pt) = c.probe(0.0);
            assert_relative_eq!(ps, 0.0, epsilon = 1e-12);
            assert_relative_eq!(pt, c.ec, max_relative = 1e-12);
        }
    }

    /// 圧縮包絡線の原点は σ=0・接線 Ec（終局域クランプの巻き添えにしない）。
    #[test]
    fn test_newrc_envelope_compression_at_origin() {
        let e = NewRcEnvelope::new(21.0);
        let (s, t) = e.compression(0.0);
        assert_relative_eq!(s, 0.0, epsilon = 1e-12);
        assert_relative_eq!(t, e.ec, max_relative = 1e-12);
    }

    #[test]
    fn test_newrc_eps_c0_reasonable() {
        let c = ConcreteNewRc::new(30.0, 2.0);
        // εc0 は 0.002 前後（普通強度コンクリート）。
        assert!(c.eps_c0 > 0.0015 && c.eps_c0 < 0.0030, "εc0={}", c.eps_c0);
    }

    #[test]
    fn test_newrc_softening_after_peak() {
        let mut c = ConcreteNewRc::new(30.0, 2.0);
        let (s_peak, _) = c.trial(-c.eps_c0);
        c.commit();
        let (s_post, _) = c.trial(-2.0 * c.eps_c0);
        // ピーク後は |σ| が低下（軟化）。
        assert!(
            s_post > s_peak,
            "post-peak stress should reduce magnitude: peak={s_peak}, post={s_post}"
        );
    }

    #[test]
    fn test_newrc_tension_cracks() {
        let mut c = ConcreteNewRc::new(30.0, 2.0);
        let eps_cr = c.eps_cr();
        let (s_el, _) = c.trial(eps_cr * 0.5);
        assert!(s_el > 0.0);
        c.commit();
        let (s_cr, _) = c.trial(eps_cr * 2.0);
        // ひび割れ後は応力ゼロ（脆性）。
        assert_relative_eq!(s_cr, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn test_probe_matches_trial_without_mutating_state() {
        // probe は trial と数学的に同一の結果を返し、状態を書き換えない。
        let mut c = ConcreteNewRc::new(30.0, 2.0);
        c.trial(-c.eps_c0 * 1.2);
        c.commit();

        let probe_strain = -0.0008; // 除荷側（割線剛性の分岐）
        let before = c.probe(probe_strain);
        assert_eq!(before, c.probe(probe_strain));

        let mut clone_for_trial = c.clone();
        let via_trial = clone_for_trial.trial(probe_strain);
        assert_eq!(before, via_trial, "probe は trial と完全一致すること");

        let after_probe = c.trial(probe_strain);
        assert_eq!(after_probe, via_trial);
    }

    #[test]
    fn test_newrc_commit_revert() {
        let mut c = ConcreteNewRc::new(30.0, 2.0);
        c.trial(-0.001);
        c.commit();
        c.trial(-0.003);
        c.revert();
        let (stress, _) = c.trial(-0.0005);
        assert!(stress < 0.0);
    }

    #[test]
    fn test_newrc_reference_values() {
        let c = ConcreteNewRc::new(30.0, 2.0);
        assert_eq!(c.reference_stress(), 30.0);
        assert_relative_eq!(c.reference_strain(), c.eps_c0);
    }

    #[test]
    fn test_newrc_envelope_peak_at_ec0() {
        // NewRcEnvelope 単体のピーク: compression(εc0) == (fc, ~0)。
        let env = NewRcEnvelope::new(30.0);
        let (stress, tangent) = env.compression(env.eps_c0);
        assert_relative_eq!(stress, 30.0, max_relative = 1e-6);
        assert_relative_eq!(tangent, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_newrc_envelope_initial_tangent_is_ec() {
        // NewRcEnvelope 単体の初期接線: x→0 で接線 ≈ Ec。
        let env = NewRcEnvelope::new(30.0);
        let (_, tangent) = env.compression(1e-9);
        assert_relative_eq!(tangent, env.ec, max_relative = 1e-3);
    }

    #[test]
    fn test_newrc_refactor_matches_known_values() {
        // リファクタ前の実装（工学単位系 kg/cm² 換算の有理式）を Python で再現し
        // 得た既知値と、NewRcEnvelope 経由の ConcreteNewRc の応答が一致することを確認する。
        // fc=30.0, gamma=2.4（既定）のとき εc0 ≈ 0.0021927039678952937。
        let cases: [(f64, f64); 3] = [
            (-0.0005, -13.585629463966358),
            (-0.0021927039678952937, -30.0),
            (-0.004385407935790587, -26.63656156909651),
        ];
        for (strain, expected_stress) in cases {
            let mut c = ConcreteNewRc::new(30.0, 2.0);
            let (stress, _) = c.trial(strain);
            assert_relative_eq!(stress, expected_stress, max_relative = 1e-9);
        }
    }
}
