//! 要素ローカル直交フレームと 24 自由度の回転変換。
//!
//! - [`ShellFrame`] — 4節点シェルのローカル基底 (e1, e2, n)
//! - [`ShellFrame::from_nodes`] — 節点座標から正規直交フレームを構築
//! - [`ShellFrame::to_global`] — ローカル剛性 → グローバル剛性 (K_g = R·K_l·Rᵀ)
//! - [`ShellFrame::rotate_to_local_24`] — 24 ベクトル回転

use crate::behavior::LocalMat;

/// Element-local orthonormal frame for a 4-node shell.
#[derive(Clone, Copy)]
pub struct ShellFrame {
    pub e1: [f64; 3],
    pub e2: [f64; 3],
    pub n: [f64; 3],
}

impl ShellFrame {
    pub fn from_nodes(p: [[f64; 3]; 4]) -> Self {
        use squid_n_core::geom::vec3;

        // 面法線は対角線の外積で採る（平面でない 4 節点でも中立的な向きになる）。
        // 正規化の除算を `vec3::unit` へ寄せないのは、`unit` の縮退判定が mm 座標
        // 前提の `ZERO_TOL`（1e-9）で、ここで測る対角線外積・辺ベクトルとは
        // 判定値 1e-12 の意味が違うためである。
        let n = vec3::cross(vec3::sub(p[2], p[0]), vec3::sub(p[3], p[1]));
        let nl = vec3::norm(n);
        let n = if nl > 1e-12 {
            [n[0] / nl, n[1] / nl, n[2] / nl]
        } else {
            [0.0, 0.0, 1.0]
        };

        let e1 = vec3::sub(p[1], p[0]);
        let e1l = vec3::norm(e1);
        let e1 = if e1l > 1e-12 {
            [e1[0] / e1l, e1[1] / e1l, e1[2] / e1l]
        } else {
            [1.0, 0.0, 0.0]
        };

        let e2 = vec3::cross(n, e1);

        Self { e1, e2, n }
    }

    fn rot_6x6(&self) -> [f64; 36] {
        let mut r = [0.0; 36];
        for i in 0..3 {
            r[i * 6] = self.e1[i];
            r[i * 6 + 1] = self.e2[i];
            r[i * 6 + 2] = self.n[i];
            r[(i + 3) * 6 + 3] = self.e1[i];
            r[(i + 3) * 6 + 4] = self.e2[i];
            r[(i + 3) * 6 + 5] = self.n[i];
        }
        r
    }

    fn rot_6x6_transpose(&self) -> [f64; 36] {
        let mut rt = [0.0; 36];
        for i in 0..3 {
            rt[i] = self.e1[i];
            rt[6 + i] = self.e2[i];
            rt[12 + i] = self.n[i];
            rt[3 * 6 + (i + 3)] = self.e1[i];
            rt[4 * 6 + (i + 3)] = self.e2[i];
            rt[5 * 6 + (i + 3)] = self.n[i];
        }
        rt
    }

    pub fn to_global(&self, k_local: &LocalMat) -> LocalMat {
        let n = 24;
        let r = self.rot_6x6();
        let rt = self.rot_6x6_transpose();
        let mut r_block = vec![0.0; n * n];
        for b in 0..4 {
            let bo = b * 6;
            for i in 0..6 {
                for j in 0..6 {
                    r_block[(bo + i) * n + (bo + j)] = r[i * 6 + j];
                }
            }
        }
        let mut rt_block = vec![0.0; n * n];
        for b in 0..4 {
            let bo = b * 6;
            for i in 0..6 {
                for j in 0..6 {
                    rt_block[(bo + i) * n + (bo + j)] = rt[i * 6 + j];
                }
            }
        }
        // 標準規約: R=[e1 e2 n]（列＝ローカル基底）が local→global。
        // K_global = R · K_local · Rᵀ。
        let mut tmp = vec![0.0; n * n];
        for i in 0..n {
            for j in 0..n {
                let mut s = 0.0;
                for k in 0..n {
                    s += k_local.get(i, k) * rt_block[k * n + j];
                }
                tmp[i * n + j] = s;
            }
        }
        let mut kg = LocalMat::zeros(n);
        for i in 0..n {
            for j in 0..n {
                let mut s = 0.0;
                for k in 0..n {
                    s += r_block[i * n + k] * tmp[k * n + j];
                }
                kg.set(i, j, s);
            }
        }
        kg
    }

    /// Rotate a 24-vector from local to global: v_g = R v_l（R=[e1 e2 n]列）。
    pub fn rotate_to_local_24(&self, v_global: &[f64; 24]) -> [f64; 24] {
        let rt = self.rot_6x6_transpose();
        let n = 24;
        let mut rt_block = vec![0.0; n * n];
        for b in 0..4 {
            let bo = b * 6;
            for i in 0..6 {
                for j in 0..6 {
                    rt_block[(bo + i) * n + (bo + j)] = rt[i * 6 + j];
                }
            }
        }
        let mut vl = [0.0; 24];
        for i in 0..24 {
            let mut s = 0.0;
            for j in 0..24 {
                s += rt_block[i * 24 + j] * v_global[j];
            }
            vl[i] = s;
        }
        vl
    }
}
