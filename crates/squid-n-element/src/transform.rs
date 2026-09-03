use crate::behavior::LocalMat;

#[derive(Clone, Copy, Debug)]
pub struct LocalFrame {
    pub rot: [[f64; 3]; 3],
}

impl LocalFrame {
    pub fn from_nodes(p_i: [f64; 3], p_j: [f64; 3], ref_vec: [f64; 3]) -> Self {
        let d = squid_n_core::geom::vec3::sub(p_j, p_i);
        // 零長要素（2 節点が同一座標）は材軸方向を定義できないため、長さ 1 の
        // 退化しないスケールに置き換えて ex を全体 X 方向へ倒す。
        let l = squid_n_core::geom::vec3::norm(d);
        let l = if l < 1e-12 { 1.0 } else { l };

        let ex = squid_n_core::geom::vec3::scale(d, 1.0 / l);

        let rdot = ref_vec[0] * ex[0] + ref_vec[1] * ex[1] + ref_vec[2] * ex[2];
        let mut ey = [
            ref_vec[0] - rdot * ex[0],
            ref_vec[1] - rdot * ex[1],
            ref_vec[2] - rdot * ex[2],
        ];
        let eyl = (ey[0] * ey[0] + ey[1] * ey[1] + ey[2] * ey[2]).sqrt();
        if eyl > 1e-12 {
            ey = [ey[0] / eyl, ey[1] / eyl, ey[2] / eyl];
        } else {
            let mut alt = if ex[0].abs() < 0.9 {
                [1.0, 0.0, 0.0]
            } else {
                [0.0, 1.0, 0.0]
            };
            let rdot2 = alt[0] * ex[0] + alt[1] * ex[1] + alt[2] * ex[2];
            alt = [
                alt[0] - rdot2 * ex[0],
                alt[1] - rdot2 * ex[1],
                alt[2] - rdot2 * ex[2],
            ];
            let altl = (alt[0] * alt[0] + alt[1] * alt[1] + alt[2] * alt[2]).sqrt();
            ey = if altl > 1e-12 {
                [alt[0] / altl, alt[1] / altl, alt[2] / altl]
            } else {
                [0.0, 1.0, 0.0]
            };
        }

        let ez = [
            ex[1] * ey[2] - ex[2] * ey[1],
            ex[2] * ey[0] - ex[0] * ey[2],
            ex[0] * ey[1] - ex[1] * ey[0],
        ];

        Self { rot: [ex, ey, ez] }
    }

    fn make_r12(&self) -> Vec<f64> {
        let n = 12;
        let mut r = vec![0.0; n * n];
        for b in 0..4 {
            let base = b * 3;
            for i in 0..3 {
                for j in 0..3 {
                    r[(base + i) * n + (base + j)] = self.rot[i][j];
                }
            }
        }
        r
    }

    fn make_r12_transpose(&self) -> Vec<f64> {
        let n = 12;
        let mut rt = vec![0.0; n * n];
        for b in 0..4 {
            let base = b * 3;
            for i in 0..3 {
                for j in 0..3 {
                    rt[(base + i) * n + (base + j)] = self.rot[j][i];
                }
            }
        }
        rt
    }

    pub fn to_global(&self, k_local: &LocalMat) -> LocalMat {
        let n = 12;
        let rt = self.make_r12_transpose();
        let r = self.make_r12();
        // K_global = R^T * K_local * R
        let mut tmp = vec![0.0; n * n];
        for i in 0..n {
            for j in 0..n {
                let mut s = 0.0;
                for k in 0..n {
                    s += k_local.get(i, k) * r[k * n + j];
                }
                tmp[i * n + j] = s;
            }
        }
        let mut kg = LocalMat::zeros(n);
        for i in 0..n {
            for j in 0..n {
                let mut s = 0.0;
                for k in 0..n {
                    s += rt[i * n + k] * tmp[k * n + j];
                }
                kg.set(i, j, s);
            }
        }
        kg
    }

    pub fn rotate_to_global(&self, v_local: &[f64; 12]) -> [f64; 12] {
        let rt = self.make_r12_transpose();
        let mut vg = [0.0; 12];
        for i in 0..12 {
            let mut s = 0.0;
            for j in 0..12 {
                s += rt[i * 12 + j] * v_local[j];
            }
            vg[i] = s;
        }
        vg
    }

    pub fn rotate_to_local(&self, v_global: &[f64; 12]) -> [f64; 12] {
        let r = self.make_r12();
        let mut vl = [0.0; 12];
        for i in 0..12 {
            let mut s = 0.0;
            for j in 0..12 {
                s += r[i * 12 + j] * v_global[j];
            }
            vl[i] = s;
        }
        vl
    }
}
