use crate::behavior::LocalMat;
use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, Model};

/// 2 節点線材の端部幾何（節点 ID・節点座標・節点間距離）。
///
/// 線材要素の構築は例外なく「2 節点を引く → 座標を得る → 節点間距離を測る →
/// 局所座標系を組む」という順で始まる。前半 3 つは要素種別に依らないためここへ
/// 集約する。局所座標系の組み方は零長要素の扱いが要素ごとに異なる（節点バネは
/// 単位回転、免震支承は鉛直を既定とする）ため、[`Self::local_frame`] を使うか
/// 各要素で `LocalFrame::from_nodes` を呼ぶかは呼び出し側が決める。
///
/// 節点 ID が範囲外のときは原点へ落とす。要素が壊れたモデルでも構築自体は通し、
/// 検出は解析前チェックに委ねる（断面・材料の引き当てと同じ方針）。
#[derive(Clone, Copy, Debug)]
pub(crate) struct EndGeometry {
    /// 両端の節点 ID（i 端, j 端）。
    pub(crate) nodes: [NodeId; 2],
    /// 両端の節点座標 \[mm\]（i 端, j 端）。
    pub(crate) coords: [[f64; 3]; 2],
    /// 節点間距離（芯々長さ）\[mm\]。
    pub(crate) length: f64,
}

impl EndGeometry {
    /// 要素データとモデルから端部幾何を求める。
    pub(crate) fn of_element(data: &ElementData, model: &Model) -> Self {
        let nodes = [data.nodes[0], data.nodes[1]];
        let coords = nodes.map(|n| {
            model
                .nodes
                .get(n.index())
                .map(|node| node.coord)
                .unwrap_or([0.0; 3])
        });
        Self {
            nodes,
            coords,
            length: squid_n_core::geom::vec3::dist(coords[0], coords[1]),
        }
    }

    /// i 端から j 端へ向かう局所座標系。零長要素を特別扱いしない要素向け
    /// （`LocalFrame::from_nodes` が長さ 1 へ退避させる既定の扱いに従う）。
    pub(crate) fn local_frame(&self, ref_vec: [f64; 3]) -> LocalFrame {
        LocalFrame::from_nodes(self.coords[0], self.coords[1], ref_vec)
    }
}

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
