//! バイリニア鋼材（弾性＋線形硬化＝kinematic hardening）。

use crate::state_serde::impl_material_serde;
use crate::uniaxial::UniaxialMaterial;

/// バイリニア鋼材（弾性＋線形硬化＝kinematic hardening）。
/// 降伏点 fy [N/mm²]、ヤング率 e [N/mm²]、hardening = ひずみ硬化比（降伏後接線 = hardening·e）。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Bilinear {
    pub e: f64,
    pub fy: f64,
    pub hardening: f64,
    committed: BilinearState,
    trial: BilinearState,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[allow(dead_code)]
struct BilinearState {
    strain: f64,
    stress: f64,
    tangent: f64,
    plastic_strain: f64,
}

impl Bilinear {
    pub fn new(e: f64, fy: f64, hardening: f64) -> Self {
        let init = BilinearState {
            strain: 0.0,
            stress: 0.0,
            tangent: e,
            plastic_strain: 0.0,
        };
        Self {
            e,
            fy,
            hardening,
            committed: init.clone(),
            trial: init,
        }
    }

    /// 1D 線形 kinematic hardening の塑性係数 Hp。
    /// 降伏後の接線 dσ/dε = e·Hp/(e+Hp) = hardening·e となるよう Hp を定める。
    fn hp(&self) -> f64 {
        if self.hardening >= 1.0 {
            f64::INFINITY
        } else {
            self.hardening * self.e / (1.0 - self.hardening)
        }
    }

    /// 塑性ひずみ ep（通常は committed 値）と strain から状態を評価する
    /// （trial・probe 共通の内部処理）。
    fn eval_state(&self, ep: f64, strain: f64) -> BilinearState {
        let hp = self.hp();
        // 弾性予測（塑性ひずみは前回コミット値で固定）
        let sigma_tr = self.e * (strain - ep);
        // 降伏関数 f = |σ - α| - fy, α = Hp·ep（kinematic hardening の背応力）
        let alpha = hp * ep;
        let f_tr = (sigma_tr - alpha).abs() - self.fy;
        if f_tr <= 0.0 {
            // 弾性
            BilinearState {
                strain,
                stress: sigma_tr,
                tangent: self.e,
                plastic_strain: ep,
            }
        } else {
            // 塑性戻し写像: Δep = f_tr/(e+Hp), 方向 = sign(σ_tr - α)
            let sgn = (sigma_tr - alpha).signum();
            let dep = f_tr / (self.e + hp);
            let ep_new = ep + sgn * dep;
            let stress = self.e * (strain - ep_new);
            let tangent = if hp.is_infinite() {
                0.0
            } else {
                self.e * hp / (self.e + hp)
            };
            BilinearState {
                strain,
                stress,
                tangent,
                plastic_strain: ep_new,
            }
        }
    }
}

impl UniaxialMaterial for Bilinear {
    fn set_yield(&mut self, fy: f64) {
        // kinematic hardening の背応力・塑性ひずみは維持したまま降伏面半径のみ更新
        self.fy = fy.max(1e-9);
    }

    fn reference_stress(&self) -> f64 {
        // 実質弾性（fy を極端に大きく設定した鋼材ダミー）は塑性率評価の対象外。
        if self.fy >= 1e18 {
            0.0
        } else {
            self.fy
        }
    }

    fn reference_strain(&self) -> f64 {
        if self.fy >= 1e18 || self.e <= 0.0 {
            0.0
        } else {
            self.fy / self.e
        }
    }

    fn trial(&mut self, strain: f64) -> (f64, f64) {
        let ep = self.committed.plastic_strain;
        self.trial = self.eval_state(ep, strain);
        (self.trial.stress, self.trial.tangent)
    }

    fn probe(&self, strain: f64) -> (f64, f64) {
        let ep = self.committed.plastic_strain;
        let s = self.eval_state(ep, strain);
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
    use crate::MaterialStateError;
    use approx::assert_relative_eq;

    #[test]
    fn test_bilinear_elastic() {
        let mut s = Bilinear::new(205000.0, 235.0, 0.01);
        let (stress, t) = s.trial(0.001);
        assert_relative_eq!(stress, 205.0, epsilon = 1.0);
        assert_relative_eq!(t, 205000.0, epsilon = 1.0);
    }

    #[test]
    fn test_bilinear_yield_monotonic() {
        let mut s = Bilinear::new(205000.0, 235.0, 0.01);
        let eps_y = 235.0 / 205000.0;
        let (stress, t) = s.trial(eps_y * 5.0);
        // 降伏後: stress = fy + hardening·e·(ε - εy)
        let expected = 235.0 + 0.01 * 205000.0 * (eps_y * 5.0 - eps_y);
        assert_relative_eq!(stress, expected, epsilon = 1.0);
        assert_relative_eq!(t, 0.01 * 205000.0, epsilon = 1.0);
    }

    #[test]
    fn test_bilinear_unload_elastic() {
        let mut s = Bilinear::new(205000.0, 235.0, 0.01);
        let eps_y = 235.0 / 205000.0;
        s.trial(eps_y * 5.0);
        s.commit();
        // 除荷: 弾性勾配 E
        let (stress, t) = s.trial(eps_y * 4.0);
        assert_relative_eq!(t, 205000.0, epsilon = 1.0);
        // 応力は直前コミット点から E·Δε だけ戻る
        let prev = 235.0 + 0.01 * 205000.0 * (eps_y * 5.0 - eps_y);
        assert_relative_eq!(stress, prev - 205000.0 * eps_y, epsilon = 1.0);
    }

    #[test]
    fn test_bilinear_kinematic_bauschinger() {
        // 引張降伏→圧縮再載荷で降伏点が負側へ移動（kinematic hardening）
        let mut s = Bilinear::new(205000.0, 235.0, 0.01);
        let eps_y = 235.0 / 205000.0;
        s.trial(eps_y * 5.0);
        s.commit();
        s.trial(-eps_y * 0.5);
        s.commit();
        let (stress, _) = s.trial(-eps_y * 5.0);
        // kinematic hardening では |σ| が fy を超えてから塑性。完全弾性戻りではない。
        assert!(stress.abs() > 235.0 * 0.9, "kinematic shift expected");
    }

    #[test]
    fn test_bilinear_commit_revert() {
        let mut s = Bilinear::new(205000.0, 235.0, 0.01);
        let (s1, _) = s.trial(0.001);
        s.commit();
        let (s2, _) = s.trial(0.002);
        assert!(s2.abs() > s1.abs());
        s.revert();
        let (s3, _) = s.trial(0.0005);
        assert!(s3.abs() < s1.abs());
    }

    #[test]
    fn test_probe_matches_trial_without_mutating_state() {
        // probe は trial と数学的に同一の結果を返し、かつ状態を書き換えない
        // （clone_box()+trial() の置換として使えることの検証）。
        let mut m = Bilinear::new(205000.0, 235.0, 0.01);
        let eps_y = 235.0 / 205000.0;
        m.trial(eps_y * 3.0);
        m.commit();

        let probe_strain = eps_y * 10.0;
        let before = m.probe(probe_strain);
        // probe を何度呼んでも同じ結果（状態不変の確認）。
        assert_eq!(before, m.probe(probe_strain));

        let mut clone_for_trial = m.clone();
        let via_trial = clone_for_trial.trial(probe_strain);
        assert_eq!(before, via_trial, "probe は trial と完全一致すること");

        // probe 呼び出し後も committed 状態から通常どおり trial できること。
        let after_probe = m.trial(probe_strain);
        assert_eq!(after_probe, via_trial);
    }

    #[test]
    fn test_serialize_state_roundtrip() {
        let mut a = Bilinear::new(205000.0, 235.0, 0.01);
        a.trial(235.0 / 205000.0 * 3.0);
        a.commit();
        let bytes = a.serialize_state();
        let mut b = Bilinear::new(1.0, 1.0, 0.0);
        b.deserialize_state(&bytes).expect("valid state restores");
        assert_eq!(a.serialize_state(), b.serialize_state());
    }

    #[test]
    fn test_deserialize_state_rejects_corrupt_bytes() {
        // 従来は復元失敗を黙って握り潰していた。現在は Err を返し、
        // 状態は据え置かれる（破損チェックポイントを検出できる）。
        let mut m = Bilinear::new(205000.0, 235.0, 0.01);
        let before = m.serialize_state();
        let err = m.deserialize_state(&[0xff, 0x00, 0x01]);
        assert!(matches!(err, Err(MaterialStateError::Decode(_))));
        assert_eq!(m.serialize_state(), before, "失敗時は状態を変えない");
    }
}
