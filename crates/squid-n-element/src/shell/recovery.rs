//! 断面力の回復。
//!
//! - [`ShellElement::recover_resultants`] — 2×2 ガウス点で断面力を回復

use super::constitutive::{d_bending, d_membrane, d_shear};
use super::element::ShellElement;
use super::resultants::ShellResultants;
use super::shape::{dshape_cart, shape_2d, GAUSS_PTS_2};

impl ShellElement {
    #[allow(non_snake_case)]
    pub fn recover_resultants(
        &self,
        u_elem_global: &[f64; 24],
    ) -> Vec<([f64; 2], ShellResultants)> {
        let u_local = self.frame.rotate_to_local_24(u_elem_global);
        let lc = self.local_coords();
        let mut results = Vec::with_capacity(4);

        for gi in 0..2 {
            for gj in 0..2 {
                let gp = gi * 2 + gj;
                let xi = GAUSS_PTS_2[gp].0;
                let eta = GAUSS_PTS_2[gp].1;
                let dNc = dshape_cart(xi, eta, &lc);

                let bm = self.membrane_b(xi, eta, &dNc);
                let bb = self.bending_b(xi, eta, &dNc);
                let bs = self.shear_b_mitc4(xi, eta, &lc);

                let mut eps_m = [0.0; 3];
                let mut eps_b = [0.0; 3];
                let mut eps_s = [0.0; 2];

                for j in 0..24 {
                    for r in 0..3 {
                        eps_m[r] += bm[r * 24 + j] * u_local[j];
                        eps_b[r] += bb[r * 24 + j] * u_local[j];
                    }
                    for r in 0..2 {
                        eps_s[r] += bs[r * 24 + j] * u_local[j];
                    }
                }

                let dm = d_membrane(self.e, self.nu, self.t);
                let db = d_bending(self.e, self.nu, self.t);
                let ds = d_shear(self.e, self.nu, self.t);

                let nx = dm[0][0] * eps_m[0] + dm[0][1] * eps_m[1];
                let ny = dm[1][0] * eps_m[0] + dm[1][1] * eps_m[1];
                let nxy = dm[2][2] * eps_m[2];
                let mx = db[0][0] * eps_b[0] + db[0][1] * eps_b[1];
                let my = db[1][0] * eps_b[0] + db[1][1] * eps_b[1];
                let mxy = db[2][2] * eps_b[2];
                let qx = ds[0][0] * eps_s[0];
                let qy = ds[1][1] * eps_s[1];

                let N = shape_2d(xi, eta);
                let mut x = 0.0;
                let mut y = 0.0;
                for i in 0..4 {
                    x += N[i] * lc[i][0];
                    y += N[i] * lc[i][1];
                }

                results.push((
                    [x, y],
                    ShellResultants {
                        nx,
                        ny,
                        nxy,
                        mx,
                        my,
                        mxy,
                        qx,
                        qy,
                    },
                ));
            }
        }

        results
    }
}
