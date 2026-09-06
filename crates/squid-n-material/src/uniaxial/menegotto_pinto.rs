//! Menegotto–Pinto モデル（バウシンガー効果を滑らかに表現する鉄筋・鋼材履歴）。
//!
//! 基本式は Menegotto & Pinto (1973) の 2 漸近線を結ぶ滑らかな遷移曲線、
//! 履歴反転時の漸近線交点の更新則・曲率パラメータ R の劣化則・等方硬化則は
//! Filippou, Popov & Bertero (1983, EERC 83-19) に従う（OpenSees Steel02 と
//! 同一のアルゴリズム）。

use crate::state_serde::impl_material_serde;
use crate::uniaxial::UniaxialMaterial;

/// 載荷方向の状態。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
enum LoadingBranch {
    /// 未載荷。
    Initial,
    /// 引張方向へ載荷中。
    TowardTension,
    /// 圧縮方向へ載荷中。
    TowardCompression,
}

/// Menegotto–Pinto モデル。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct MenegottoPinto {
    /// 初期弾性係数 E [N/mm²]。
    pub e: f64,
    /// 降伏強度 fy [N/mm²]。
    pub fy: f64,
    /// 降伏後剛性比 b。
    pub b: f64,
    /// 初期曲率パラメータ R0。
    pub r0: f64,
    /// R 劣化則の係数 a1。
    pub a1: f64,
    /// R 劣化則の係数 a2。
    pub a2: f64,
    /// 等方硬化係数 a3（0 で等方硬化なし）。
    pub a3: f64,
    /// 等方硬化係数 a4。
    pub a4: f64,
    committed: MpState,
    trial: MpState,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct MpState {
    strain: f64,
    stress: f64,
    tangent: f64,
    /// 直前の反転点 (εr, σr)。
    eps_r: f64,
    sig_r: f64,
    /// 現在の分枝の漸近線交点 (ε0, σ0)。
    eps_0: f64,
    sig_0: f64,
    /// 経験した最大ひずみ。
    eps_max: f64,
    /// 経験した最小ひずみ。
    eps_min: f64,
    /// 直前の反転側の極値ひずみ。
    eps_pl: f64,
    /// 載荷方向。
    branch: LoadingBranch,
}

impl MenegottoPinto {
    /// 既定パラメータで生成する。
    pub fn new(e: f64, fy: f64) -> Self {
        Self::with_params(e, fy, 0.01, 20.0, 18.5, 0.15)
    }

    /// 等方硬化なし（a3=0, a4=1）でパラメータ指定。
    pub fn with_params(e: f64, fy: f64, b: f64, r0: f64, a1: f64, a2: f64) -> Self {
        Self::with_isotropic(e, fy, b, r0, a1, a2, 0.0, 1.0)
    }

    /// 等方硬化（a3, a4）を含む全パラメータ指定。
    #[allow(clippy::too_many_arguments)]
    pub fn with_isotropic(
        e: f64,
        fy: f64,
        b: f64,
        r0: f64,
        a1: f64,
        a2: f64,
        a3: f64,
        a4: f64,
    ) -> Self {
        let eps_y = if e > 0.0 { fy / e } else { 0.0 };
        let init = MpState {
            strain: 0.0,
            stress: 0.0,
            tangent: e,
            eps_r: 0.0,
            sig_r: 0.0,
            eps_0: eps_y,
            sig_0: fy,
            eps_max: eps_y,
            eps_min: -eps_y,
            eps_pl: eps_y,
            branch: LoadingBranch::Initial,
        };
        Self {
            e,
            fy,
            b,
            r0,
            a1,
            a2,
            a3,
            a4,
            committed: init.clone(),
            trial: init,
        }
    }

    fn eps_y(&self) -> f64 {
        if self.e > 0.0 {
            self.fy / self.e
        } else {
            0.0
        }
    }

    /// 等方硬化による降伏漸近線のシフト率。
    fn isotropic_shift(&self, state: &MpState) -> f64 {
        let eps_y = self.eps_y();
        if self.a3 == 0.0 || eps_y <= 0.0 {
            return 1.0;
        }
        let d1 = (state.eps_max - state.eps_min) / (2.0 * self.a4 * eps_y);
        1.0 + self.a3 * d1.max(0.0).powf(0.8)
    }

    /// 引張方向分枝の開始。
    fn start_tension_branch(&self, state: &mut MpState) {
        let eps_y = self.eps_y();
        let esh = self.b * self.e;
        let shft = self.isotropic_shift(state);
        let denom = self.e - esh;
        if denom.abs() < 1e-15 {
            return;
        }
        state.eps_0 =
            (self.fy * shft - esh * eps_y * shft - state.sig_r + self.e * state.eps_r) / denom;
        state.sig_0 = self.fy * shft + esh * (state.eps_0 - eps_y * shft);
        state.branch = LoadingBranch::TowardTension;
    }

    /// 圧縮方向分枝の開始。
    fn start_compression_branch(&self, state: &mut MpState) {
        let eps_y = self.eps_y();
        let esh = self.b * self.e;
        let shft = self.isotropic_shift(state);
        let denom = self.e - esh;
        if denom.abs() < 1e-15 {
            return;
        }
        state.eps_0 =
            (-self.fy * shft + esh * eps_y * shft - state.sig_r + self.e * state.eps_r) / denom;
        state.sig_0 = -self.fy * shft + esh * (state.eps_0 + eps_y * shft);
        state.branch = LoadingBranch::TowardCompression;
    }

    fn eval(&self, strain: f64) -> MpState {
        let c = &self.committed;
        let deps = strain - c.strain;
        let mut w = c.clone();

        if deps == 0.0 {
            w.strain = strain;
            return w;
        }

        match c.branch {
            LoadingBranch::Initial => {
                w.eps_r = 0.0;
                w.sig_r = 0.0;
                if deps > 0.0 {
                    self.start_tension_branch(&mut w);
                    w.eps_pl = w.eps_max;
                } else {
                    self.start_compression_branch(&mut w);
                    w.eps_pl = w.eps_min;
                }
            }
            LoadingBranch::TowardCompression if deps > 0.0 => {
                w.eps_r = c.strain;
                w.sig_r = c.stress;
                w.eps_min = w.eps_min.min(c.strain);
                self.start_tension_branch(&mut w);
                w.eps_pl = w.eps_max;
            }
            LoadingBranch::TowardTension if deps < 0.0 => {
                w.eps_r = c.strain;
                w.sig_r = c.stress;
                w.eps_max = w.eps_max.max(c.strain);
                self.start_compression_branch(&mut w);
                w.eps_pl = w.eps_min;
            }
            _ => {}
        }

        let deps0 = w.eps_0 - w.eps_r;
        let dsig0 = w.sig_0 - w.sig_r;
        if deps0.abs() < 1e-15 {
            w.strain = strain;
            w.stress = w.sig_r + self.e * (strain - w.eps_r);
            w.tangent = self.e;
            return w;
        }

        let eps_y = self.eps_y();
        let xi = if eps_y > 0.0 {
            ((w.eps_pl - w.eps_0) / eps_y).abs()
        } else {
            0.0
        };
        let r = (self.r0 - self.a1 * xi / (self.a2 + xi)).max(1.0);

        let eps_star = (strain - w.eps_r) / deps0;
        let dum1 = 1.0 + eps_star.abs().powf(r);
        let dum2 = dum1.powf(1.0 / r);
        let sig_star = self.b * eps_star + (1.0 - self.b) * eps_star / dum2;
        let dsig_star = self.b + (1.0 - self.b) / (dum1 * dum2);

        w.strain = strain;
        w.stress = w.sig_r + dsig0 * sig_star;
        w.tangent = dsig_star * dsig0 / deps0;
        w
    }
}

impl UniaxialMaterial for MenegottoPinto {
    fn reference_stress(&self) -> f64 {
        self.fy
    }

    fn reference_strain(&self) -> f64 {
        self.eps_y()
    }

    fn trial(&mut self, strain: f64) -> (f64, f64) {
        let working = self.eval(strain);
        self.trial = working;
        (self.trial.stress, self.trial.tangent)
    }

    fn probe(&self, strain: f64) -> (f64, f64) {
        let working = self.eval(strain);
        (working.stress, working.tangent)
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
    fn test_menegotto_pinto_elastic() {
        let mut mp = MenegottoPinto::new(205000.0, 235.0);
        let (stress, _) = mp.trial(0.001);
        assert_relative_eq!(stress, 205.0, epsilon = 5.0);
    }

    #[test]
    fn test_monotonic_approaches_hardening_asymptote() {
        // 単調引張で応力は降伏漸近線 σ = fy + b·E·(ε − εy) に漸近する。
        let e = 205000.0;
        let fy = 235.0;
        let b = 0.01;
        let mut mp = MenegottoPinto::with_params(e, fy, b, 20.0, 18.5, 0.15);
        let eps_y = fy / e;
        let eps = eps_y * 10.0;
        let (sig, _) = mp.trial(eps);
        let asymptote = fy + b * e * (eps - eps_y);
        assert_relative_eq!(sig, asymptote, max_relative = 0.01);
    }

    #[test]
    fn test_reversal_targets_shifted_asymptote_intersection() {
        // Filippou の反転則: 反転後の漸近線交点 (ε0,σ0) は「反転点を通る勾配 E の
        // 直線」と「反対側の降伏漸近線」の交点。+4εy から反転した直後の応力は
        // 弾性除荷 σr − E·Δε にほぼ一致する（交点までの遷移曲線初期勾配 ≈ E）。
        let e = 205000.0;
        let fy = 235.0;
        let mut mp = MenegottoPinto::new(e, fy);
        let eps_y = fy / e;
        let eps_top = eps_y * 4.0;
        mp.trial(eps_top);
        mp.commit();
        let sig_top = mp.probe(eps_top).0;
        let d = eps_y * 0.2;
        let (sig, _) = mp.trial(eps_top - d);
        assert_relative_eq!(sig, sig_top - e * d, max_relative = 0.02);
    }

    #[test]
    fn test_probe_matches_trial_without_mutating_state() {
        // probe は trial と数学的に同一の結果を返し、状態を書き換えない
        // （反転履歴を経た後の非弾性域で確認。反転検知ロジックの再現性が要点）。
        let mut mp = MenegottoPinto::new(205000.0, 235.0);
        let eps_y = 235.0 / 205000.0;
        for &target in &[eps_y * 4.0, -eps_y * 4.0] {
            mp.trial(target);
            mp.commit();
        }

        let probe_strain = eps_y * 2.0;
        let before = mp.probe(probe_strain);
        assert_eq!(before, mp.probe(probe_strain));

        let mut clone_for_trial = mp.clone();
        let via_trial = clone_for_trial.trial(probe_strain);
        assert_eq!(before, via_trial, "probe は trial と完全一致すること");

        let after_probe = mp.trial(probe_strain);
        assert_eq!(after_probe, via_trial);
    }

    #[test]
    fn test_menegotto_pinto_bauschinger_loop() {
        // 繰り返し履歴でバウシンガー効果（反転後の丸み）を確認
        let mut mp = MenegottoPinto::new(205000.0, 235.0);
        let eps_y = 235.0 / 205000.0;
        let mut peak = 0.0f64;
        // +4εy → -4εy → +4εy の履歴
        for &target in &[eps_y * 4.0, -eps_y * 4.0, eps_y * 4.0] {
            let n = 20;
            for i in 1..=n {
                let eps = target * (i as f64) / (n as f64);
                let (sig, _) = mp.trial(eps);
                mp.commit();
                peak = peak.max(sig.abs());
            }
        }
        // 反転後の曲率 R は ξ 増加で小さくなり、ループは漸近線に近づく。
        // ピーク応力は fy+硬化成分 に漸近し、fy を超えること（弾完全塑性ではない）
        assert!(peak > 235.0, "MP peak should exceed fy due to hardening");
    }

    #[test]
    fn test_r_degrades_after_plastic_excursion() {
        // 大振幅の反転後は R が小さくなり、遷移曲線が丸くなる。
        // 丸みの指標として「反転から 1εy 戻った点の応力の弾性除荷直線からの乖離」を
        // 小振幅反転の場合と比較する。
        let e = 205000.0;
        let fy = 235.0;
        let eps_y = fy / e;

        let departure = |amp: f64| -> f64 {
            let mut mp = MenegottoPinto::new(e, fy);
            mp.trial(amp);
            mp.commit();
            let sig_top = mp.probe(amp).0;
            let d = eps_y * 1.0;
            let (sig, _) = mp.trial(amp - d);
            (sig_top - e * d - sig).abs()
        };

        let small = departure(eps_y * 1.5);
        let large = departure(eps_y * 8.0);
        assert!(
            large > small,
            "大振幅後の反転はより丸い曲線になる: small={small}, large={large}"
        );
    }

    #[test]
    fn test_isotropic_hardening_grows_cyclic_peaks() {
        // a3 > 0 で対称サイクルのピーク応力がサイクルごとに成長する（等方硬化）。
        let e = 205000.0;
        let fy = 235.0;
        let eps_y = fy / e;
        let run_peaks = |a3: f64| -> Vec<f64> {
            let mut mp = MenegottoPinto::with_isotropic(e, fy, 0.01, 20.0, 18.5, 0.15, a3, 1.0);
            let mut peaks = Vec::new();
            for _cycle in 0..3 {
                for &target in &[eps_y * 3.0, -eps_y * 3.0] {
                    let n = 30;
                    for i in 1..=n {
                        let prev = mp.committed.strain;
                        let eps = prev + (target - prev) * (i as f64) / (n as f64);
                        mp.trial(eps);
                        mp.commit();
                    }
                    peaks.push(mp.committed.stress.abs());
                }
            }
            peaks
        };
        let peaks_iso = run_peaks(0.03);
        let peaks_kin = run_peaks(0.0);
        // 等方硬化ありは、履歴が深まるほど降伏漸近線がシフトし、同一履歴の
        // 移動硬化のみの場合よりサイクル後のピーク応力が大きくなる。
        assert!(
            peaks_iso.last().unwrap() > peaks_kin.last().unwrap(),
            "iso={peaks_iso:?} kin={peaks_kin:?}"
        );
        // シフト量は履歴極値の拡大とともに単調に効くため、等方硬化ありの
        // ピーク増分（対 移動硬化のみ）は最終サイクルほど大きい。
        let gain_first = peaks_iso[1] - peaks_kin[1];
        let gain_last = peaks_iso.last().unwrap() - peaks_kin.last().unwrap();
        assert!(
            gain_last >= gain_first,
            "iso gain should not shrink: first={gain_first}, last={gain_last}"
        );
    }

    #[test]
    fn test_tangent_matches_finite_difference() {
        // 接線剛性が数値微分と一致する（反転後の遷移曲線上で確認）。
        let e = 205000.0;
        let fy = 235.0;
        let eps_y = fy / e;
        let mut mp = MenegottoPinto::new(e, fy);
        mp.trial(eps_y * 4.0);
        mp.commit();
        for &eps in &[eps_y * 3.0, eps_y * 1.0, -eps_y * 1.0, -eps_y * 3.0] {
            let (_, tan) = mp.probe(eps);
            let h = eps_y * 1e-5;
            let (s1, _) = mp.probe(eps + h);
            let (s0, _) = mp.probe(eps - h);
            let fd = (s1 - s0) / (2.0 * h);
            assert_relative_eq!(tan, fd, max_relative = 1e-4);
        }
    }
}
