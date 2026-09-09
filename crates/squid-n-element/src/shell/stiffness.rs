//! 要素剛性行列の構成（膜・曲げ・MITC4 せん断・ドリリング安定化）。
//!
//! - [`ShellElement::local_stiffness`] — 2×2 ガウス積分によるローカル剛性 (24×24)
//! - [`ShellElement::add_drilling`] — ドリリング自由度の安定化
//! - [`ShellElement::apply_rigid_floor_membrane_off`] — 剛床時に膜自由度を除去

use super::constitutive::{d_bending, d_membrane, d_shear};
use super::element::ShellElement;
use super::geom::element_area;
use super::shape::{dshape_cart, jacobian, jacobian_det, GAUSS_PTS_2};
use crate::behavior::LocalMat;

impl ShellElement {
    pub(crate) fn add_drilling(&self, k: &mut LocalMat) {
        let gamma = self.drilling_factor;
        let g_mod = self.e / (2.0 * (1.0 + self.nu));
        let area = element_area(&self.coords);
        let scale = gamma * g_mod * self.t * area;

        let q_diag = 0.75 * scale;
        let q_off = -0.25 * scale;

        for i in 0..4 {
            let ri = i * 6 + 5;
            for j in 0..4 {
                let rj = j * 6 + 5;
                let val = if i == j { q_diag } else { q_off };
                k.set(ri, rj, k.get(ri, rj) + val);
            }
        }
    }

    #[allow(non_snake_case)]
    pub fn local_stiffness(&self) -> LocalMat {
        let n = 24;
        let mut k = LocalMat::zeros(n);
        let lc = self.local_coords();

        for gi in 0..2 {
            for gj in 0..2 {
                let gp = gi * 2 + gj;
                let xi = GAUSS_PTS_2[gp].0;
                let eta = GAUSS_PTS_2[gp].1;
                let det_j = jacobian_det(&jacobian(xi, eta, &lc));
                if det_j.abs() < 1e-30 {
                    continue;
                }
                let weight = det_j; // product of weights = 1*1 = 1

                let dNc = dshape_cart(xi, eta, &lc);

                if self.membrane_active {
                    let bm = self.membrane_b(xi, eta, &dNc);
                    let dm = d_membrane(self.e, self.nu, self.t);
                    let mut btd = vec![0.0; 24 * 3];
                    for i in 0..24 {
                        for r in 0..3 {
                            let mut s = 0.0;
                            for c in 0..3 {
                                s += bm[c * 24 + i] * dm[r][c];
                            }
                            btd[i * 3 + r] = s;
                        }
                    }
                    for i in 0..24 {
                        for j in 0..24 {
                            let mut s = 0.0;
                            for r in 0..3 {
                                s += btd[i * 3 + r] * bm[r * 24 + j];
                            }
                            k.set(i, j, k.get(i, j) + s * weight);
                        }
                    }
                }

                {
                    let bb = self.bending_b(xi, eta, &dNc);
                    let db = d_bending(self.e, self.nu, self.t);
                    let mut btd = vec![0.0; 24 * 3];
                    for i in 0..24 {
                        for r in 0..3 {
                            let mut s = 0.0;
                            for c in 0..3 {
                                s += bb[c * 24 + i] * db[r][c];
                            }
                            btd[i * 3 + r] = s;
                        }
                    }
                    for i in 0..24 {
                        for j in 0..24 {
                            let mut s = 0.0;
                            for r in 0..3 {
                                s += btd[i * 3 + r] * bb[r * 24 + j];
                            }
                            k.set(i, j, k.get(i, j) + s * weight);
                        }
                    }
                }

                {
                    let bs = self.shear_b_mitc4(xi, eta, &lc);
                    let ds = d_shear(self.e, self.nu, self.t);
                    let mut btd = vec![0.0; 24 * 2];
                    for i in 0..24 {
                        for r in 0..2 {
                            let mut s = 0.0;
                            for c in 0..2 {
                                s += bs[c * 24 + i] * ds[r][c];
                            }
                            btd[i * 2 + r] = s;
                        }
                    }
                    for i in 0..24 {
                        for j in 0..24 {
                            let mut s = 0.0;
                            for r in 0..2 {
                                s += btd[i * 2 + r] * bs[r * 24 + j];
                            }
                            k.set(i, j, k.get(i, j) + s * weight);
                        }
                    }
                }
            }
        }

        self.add_drilling(&mut k);

        k
    }

    pub fn apply_rigid_floor_membrane_off(&self, k: &mut LocalMat) {
        if !self.membrane_active {
            let n = 24;
            let mut mask = vec![true; n];
            for i in 0..4 {
                let bo = i * 6;
                mask[bo] = false;
                mask[bo + 1] = false;
                mask[bo + 5] = false;
            }
            for i in 0..n {
                if !mask[i] {
                    for j in 0..n {
                        k.set(i, j, 0.0);
                        k.set(j, i, 0.0);
                    }
                    k.set(i, i, 1.0);
                }
            }
        }
    }
}
