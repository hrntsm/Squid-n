//! M–φ（モーメント–曲率）から M–θ（モーメント–部材角）への変換。

use squid_n_material::Concrete;

const CURVATURE_EPS_INV_MM: f64 = 1e-15;

/// M–φ の 1 点を M–θ の 1 点に変換する。
#[allow(clippy::too_many_arguments)]
pub(crate) fn mphi_to_mtheta(
    curvature_inv_mm: f64,
    moment_n_mm: f64,
    ky_yield: Option<f64>,
    span_mm: f64,
    inflection_ratio: f64,
    plastic_hinge_length_mm: f64,
    shear_add: ShearContribution,
    pullout_add: PulloutContribution,
) -> (f64, f64) {
    if curvature_inv_mm.abs() < CURVATURE_EPS_INV_MM {
        return (0.0, 0.0);
    }
    let l = span_mm * inflection_ratio;
    let theta_f = match ky_yield {
        Some(ky_y) if curvature_inv_mm > ky_y => {
            ky_y * l / 3.0 + (curvature_inv_mm - ky_y) * plastic_hinge_length_mm
        }
        _ => curvature_inv_mm * l / 3.0,
    };
    let theta_s = shear_add.rotation(moment_n_mm, l);
    let theta_p = pullout_add.rotation(curvature_inv_mm, ky_yield);
    (theta_f + theta_s + theta_p, moment_n_mm)
}

/// せん断変形の寄与（M-θ への加算分）。
#[derive(Clone, Copy, Debug)]
pub struct ShearContribution {
    /// 等価せん断剛性 [N]。0 なら寄与なし。
    pub k_s: f64,
}

impl ShearContribution {
    pub fn none() -> Self {
        Self { k_s: 0.0 }
    }
    pub fn rc_rect(width: f64, depth: f64, concrete: &Concrete) -> Self {
        let g = concrete.e0_shear() / (2.0 * (1.0 + 0.2));
        let a_w = 5.0 / 6.0 * width * depth;
        Self { k_s: g * a_w }
    }
    fn rotation(&self, m: f64, l: f64) -> f64 {
        if self.k_s.abs() < 1e-12 || l.abs() < 1e-12 {
            return 0.0;
        }
        m / self.k_s
    }
}

/// 鉄筋抜出しの寄与（M-θ への加算分）。
#[derive(Clone, Copy, Debug)]
pub struct PulloutContribution {
    /// 鉄筋径 d_b [mm]
    pub bar_diameter: f64,
    /// 鉄筋ヤング率 E_s [N/mm²]
    pub e_s: f64,
    /// 降伏強度 f_y [N/mm²]
    pub fy: f64,
    /// 定着区の平均結合応力係数 ξ
    pub bond_coeff: f64,
}

impl PulloutContribution {
    pub fn none() -> Self {
        Self {
            bar_diameter: 0.0,
            e_s: 0.0,
            fy: 1.0,
            bond_coeff: 1.0,
        }
    }
    fn rotation(&self, ky: f64, ky_yield: Option<f64>) -> f64 {
        if self.bar_diameter < 1e-12 || self.e_s < 1e-12 || self.bond_coeff < 1e-12 {
            return 0.0;
        }
        let sigma_s = match ky_yield {
            Some(ky_y) if ky_y.abs() > CURVATURE_EPS_INV_MM => {
                if ky.abs() > ky_y.abs() {
                    self.fy
                } else {
                    (ky / ky_y).abs().min(1.0) * self.fy
                }
            }
            _ => self.fy * 0.5,
        };
        sigma_s * self.bar_diameter / (self.e_s * self.bond_coeff)
    }
}
