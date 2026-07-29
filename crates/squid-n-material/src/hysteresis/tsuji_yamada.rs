//! 辻・山田モデル（混合硬化則）。

use crate::state_serde::impl_material_serde;
use crate::uniaxial::UniaxialMaterial;

/// 辻・山田モデル（辻・山田による混合硬化則）。
/// バイリニア骨格 + β による等方硬化/移動硬化の混合硬化則。
///
/// 塑性増分応力 Δσ を等方硬化 `Δσ̄ = β|Δσ|`（降伏幅の膨張）と移動硬化
/// `Δᾱ = (1−β)|Δσ|`（降伏幅中心の移動）へ配分する。`β=1` で等方硬化（降伏耐力が
/// 正負同時に膨張）、`β=0` で移動硬化（標準型と同等のバウシンガー効果）となる。
///
/// 単位規約は他の `UniaxialMaterial` と同じ（変形＝ひずみ or 回転、力＝応力 or
/// モーメント、剛性＝力/変形）。JFE 二重鋼管座屈補剛ブレース等で用いられる。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct TsujiYamada {
    /// 初期剛性 K1（力/変形）。
    pub k1: f64,
    /// 降伏耐力 Qy（初期降伏面の半径）。
    pub qy: f64,
    /// 第2剛性 K2（降伏後接線。0 ≤ K2 < K1）。
    pub k2: f64,
    /// 移動/等方硬化の配分 β（0..1）。
    pub beta: f64,
    committed: TyState,
    trial: TyState,
}

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
struct TyState {
    strain: f64,
    stress: f64,
    tangent: f64,
    /// 塑性変形 dp。
    dp: f64,
    /// 背応力（移動硬化の降伏面中心）α。
    alpha: f64,
    /// 等方硬化による降伏面半径の増分 Riso（R = Qy + Riso）。
    r_iso: f64,
}

impl TsujiYamada {
    pub fn new(k1: f64, qy: f64, k2: f64, beta: f64) -> Self {
        // K2 は 0 ≤ K2 < K1 にクランプ（K2≥K1 は硬化係数 H が非有限になるため）。
        let k2 = k2.clamp(0.0, k1 * 0.999);
        let init = TyState {
            tangent: k1,
            ..Default::default()
        };
        Self {
            k1,
            qy: qy.max(1e-9),
            k2,
            beta: beta.clamp(0.0, 1.0),
            committed: init,
            trial: init,
        }
    }

    /// 硬化係数 H（塑性接線）: K2 = K1·H/(K1+H) より H = K1·K2/(K1−K2)。
    fn hardening(&self) -> f64 {
        let d = self.k1 - self.k2;
        if d <= 1e-12 {
            0.0
        } else {
            self.k1 * self.k2 / d
        }
    }

    /// committed 状態から strain における状態を評価する（trial・probe 共通の
    /// 内部処理）。committed 状態のみを参照する（self.trial は書き換えない）。
    fn eval_state(&self, strain: f64) -> TyState {
        let c = self.committed;
        let h = self.hardening();
        let q_tr = self.k1 * (strain - c.dp);
        let r = self.qy + c.r_iso;
        let f = (q_tr - c.alpha).abs() - r;
        if f <= 0.0 {
            TyState {
                strain,
                stress: q_tr,
                tangent: self.k1,
                ..c
            }
        } else {
            let s = (q_tr - c.alpha).signum();
            let d_dp = f / (self.k1 + h);
            let dp_new = c.dp + s * d_dp;
            // 移動硬化（背応力）と等方硬化（降伏面膨張）へ配分。
            let alpha_new = c.alpha + (1.0 - self.beta) * h * s * d_dp;
            let r_iso_new = c.r_iso + self.beta * h * d_dp;
            let stress = self.k1 * (strain - dp_new);
            let tangent = self.k1 * h / (self.k1 + h);
            TyState {
                strain,
                stress,
                tangent,
                dp: dp_new,
                alpha: alpha_new,
                r_iso: r_iso_new,
            }
        }
    }
}

impl UniaxialMaterial for TsujiYamada {
    fn set_yield(&mut self, fy: f64) {
        self.qy = fy.max(1e-9);
    }

    fn reference_stress(&self) -> f64 {
        self.qy
    }

    fn reference_strain(&self) -> f64 {
        if self.k1 > 0.0 {
            self.qy / self.k1
        } else {
            0.0
        }
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
    fn test_tsuji_yamada_monotonic_bilinear() {
        // 単調載荷は K2 のバイリニア骨格を辿る。K1=1000, Qy=100, K2=100, δy=0.1。
        let mut m = TsujiYamada::new(1000.0, 100.0, 100.0, 0.5);
        let (s_el, t_el) = m.trial(0.05);
        assert_relative_eq!(s_el, 50.0, epsilon = 1e-6);
        assert_relative_eq!(t_el, 1000.0, epsilon = 1e-6);
        m.commit();
        let (s, t) = m.trial(0.3);
        // Qy + K2·(δ − δy) = 100 + 100·0.2 = 120。
        assert_relative_eq!(s, 120.0, epsilon = 1e-6);
        assert_relative_eq!(t, 100.0, epsilon = 1e-6);
    }

    #[test]
    fn test_tsuji_yamada_isotropic_grows_beta1() {
        // β=1（等方硬化）: 大振幅を経験すると降伏耐力が正負同時に膨張し、
        // 同一変形での再載荷応力が初回より増大する。
        let mut m = TsujiYamada::new(1000.0, 100.0, 100.0, 1.0);
        let (s1, _) = m.trial(0.3);
        m.commit();
        for &d in &[0.0, -0.3, 0.0] {
            m.trial(d);
            m.commit();
        }
        let (s2, _) = m.trial(0.3);
        assert!(
            s2 > s1 + 10.0,
            "isotropic hardening should raise reload force: first={s1}, second={s2}"
        );
    }

    #[test]
    fn test_tsuji_yamada_kinematic_bauschinger_beta0() {
        // β=0（移動硬化）: 降伏面は膨張せず中心（背応力 α）が移動する。
        // +方向降伏（δ=0.3, α=20）後、除荷は弾性域 2Qy/K1=0.20 を経て δ=0.10 で
        // 逆方向降伏する（バウシンガー効果）。
        let mut m = TsujiYamada::new(1000.0, 100.0, 100.0, 0.0);
        m.trial(0.3);
        m.commit();
        // δ=0.15 はまだ弾性（初期剛性）。
        let (_s, t_mid) = m.trial(0.15);
        assert_relative_eq!(t_mid, 1000.0, epsilon = 1e-6);
        // δ=0.05 では逆方向に塑性化（第2剛性・圧縮応力）。
        let (s_rev, t_rev) = m.trial(0.05);
        assert_relative_eq!(t_rev, 100.0, epsilon = 1e-6);
        assert!(s_rev < 0.0, "reverse plastic in compression: {s_rev}");
    }

    #[test]
    fn test_probe_matches_trial_without_mutating_state() {
        // probe は trial と数学的に同一の結果を返し、状態を書き換えない
        // （塑性化・除荷後の状態で確認）。
        let mut m = TsujiYamada::new(1000.0, 100.0, 100.0, 0.5);
        m.trial(0.3);
        m.commit();

        let probe_strain = 0.1; // 除荷側
        let before = m.probe(probe_strain);
        assert_eq!(before, m.probe(probe_strain));

        let mut clone_for_trial = m.clone();
        let via_trial = clone_for_trial.trial(probe_strain);
        assert_eq!(before, via_trial, "probe は trial と完全一致すること");

        let after_probe = m.trial(probe_strain);
        assert_eq!(after_probe, via_trial);
    }

    #[test]
    fn test_tsuji_yamada_commit_revert() {
        let mut m = TsujiYamada::new(1000.0, 100.0, 100.0, 0.5);
        m.trial(0.2);
        m.commit();
        let (s1, _) = m.trial(0.4);
        m.revert();
        let (s2, _) = m.trial(0.25);
        assert!(s2 < s1);
    }
}
