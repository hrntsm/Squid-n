//! Menegotto–Pinto モデル（バウシンガー効果を滑らかに表現する鉄筋履歴）。

use crate::state_serde::impl_material_serde;
use crate::uniaxial::UniaxialMaterial;

/// Menegotto–Pinto モデル（バウシンガー効果を滑らかに表現）。
/// 仕様書 §4 の正規化形。反転点 (εr,σr)・漸近線交点 (ε0,σ0)・ξ を状態に保持。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct MenegottoPinto {
    pub e: f64,
    pub fy: f64,
    pub b: f64,
    pub r0: f64,
    pub a1: f64,
    pub a2: f64,
    committed: MpState,
    trial: MpState,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct MpState {
    strain: f64,
    stress: f64,
    tangent: f64,
    /// 直前の反転点 (εr, σr)
    eps_r: f64,
    sig_r: f64,
    /// 漸近線の交点 (ε0, σ0)
    eps_0: f64,
    sig_0: f64,
    /// 反転後の塑性ひずみ振幅パラメータ ξ
    xi: f64,
    /// 直前の trial の進行方向（+1/-1/0）。反転検知に使う。
    direction: f64,
}

impl MenegottoPinto {
    pub fn new(e: f64, fy: f64) -> Self {
        Self::with_params(e, fy, 0.01, 20.0, 18.5, 0.15)
    }

    pub fn with_params(e: f64, fy: f64, b: f64, r0: f64, a1: f64, a2: f64) -> Self {
        let init = MpState {
            strain: 0.0,
            stress: 0.0,
            tangent: e,
            eps_r: 0.0,
            sig_r: 0.0,
            // 初期漸近線交点は弾性直線と降伏後直線の交点 = (εy, fy)
            eps_0: fy / e,
            sig_0: fy,
            xi: 0.0,
            direction: 0.0,
        };
        Self {
            e,
            fy,
            b,
            r0,
            a1,
            a2,
            committed: init.clone(),
            trial: init,
        }
    }

    /// 反転点 (εr,σr) と漸近線交点 (ε0,σ0) を更新する（標準 MP 手順）。
    /// 反転点 = 直前のコミット点。新しい漸近線交点は反転点を元の交点に対して鏡映。
    fn update_reversal(&self, state: &mut MpState, prev_strain: f64, prev_stress: f64) {
        let new_eps_r = prev_strain;
        let new_sig_r = prev_stress;
        // 元の漸近線交点に対する鏡映: (ε0',σ0') = 2·(εr,σr) - (ε0,σ0)
        let new_eps_0 = 2.0 * new_eps_r - state.eps_0;
        let new_sig_0 = 2.0 * new_sig_r - state.sig_0;
        // ξ: 反転後の塑性ひずみ振幅（Chang & Mander 2004 系の簡易形）。
        //   ξ = |εr - εr_prev| / εy,  εy = fy/E。単調増加を保証。
        let eps_y = self.fy / self.e;
        let xi_new = if eps_y.abs() > 1e-15 {
            ((new_eps_r - state.eps_r).abs() / eps_y).max(state.xi)
        } else {
            state.xi
        };
        state.eps_r = new_eps_r;
        state.sig_r = new_sig_r;
        state.eps_0 = new_eps_0;
        state.sig_0 = new_sig_0;
        state.xi = xi_new;
    }

    /// committed 状態から strain における状態を評価する（trial・probe 共通の
    /// 内部処理）。committed 状態のみを参照し、self.trial は書き換えない。
    fn eval(&self, strain: f64) -> MpState {
        let c = &self.committed;
        // 進行方向の判定と反転検知（trial 内で行う＝標準 MP）
        let dir_new = (strain - c.strain).signum();
        let mut working = c.clone();
        if dir_new != 0.0 && c.direction != 0.0 && dir_new != c.direction {
            // 反転: 直前のコミット点を新しい反転点として採用
            self.update_reversal(&mut working, c.strain, c.stress);
            working.direction = dir_new;
        } else if dir_new != 0.0 {
            working.direction = dir_new;
        }

        let deps = working.eps_0 - working.eps_r;
        let dsig = working.sig_0 - working.sig_r;
        if deps.abs() < 1e-15 {
            // 漸近線が縮退: 弾性直線で評価
            let stress = working.sig_r + self.e * (strain - working.eps_r);
            working.strain = strain;
            working.stress = stress;
            working.tangent = self.e;
            return working;
        }
        let eps_star = (strain - working.eps_r) / deps;
        let r = (self.r0 - self.a1 * working.xi / (self.a2 + working.xi)).max(1.0);
        let eps_star_abs = eps_star.abs();
        let denom = (1.0 + eps_star_abs.powf(r)).powf(1.0 / r);
        let sig_star = self.b * eps_star + (1.0 - self.b) * eps_star / denom;
        let stress = working.sig_r + dsig * sig_star;
        // dσ*/dε* = b + (1-b)·(1/denom - ε*·ddenom)
        let ddenom = if eps_star_abs > 1e-15 {
            eps_star_abs.powf(r - 1.0) * eps_star.signum()
                / (1.0 + eps_star_abs.powf(r)).powf(1.0 + 1.0 / r)
        } else {
            0.0
        };
        let dsig_star = self.b + (1.0 - self.b) * (1.0 / denom - eps_star * ddenom);
        let tangent = (dsig / deps) * dsig_star;
        let tangent = tangent.clamp(self.b * self.e, self.e);

        working.strain = strain;
        working.stress = stress;
        working.tangent = tangent;
        working
    }
}

impl UniaxialMaterial for MenegottoPinto {
    fn reference_stress(&self) -> f64 {
        self.fy
    }

    fn reference_strain(&self) -> f64 {
        if self.e > 0.0 {
            self.fy / self.e
        } else {
            0.0
        }
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
}
