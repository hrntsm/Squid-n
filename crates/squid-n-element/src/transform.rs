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
        use squid_n_core::geom::vec3;

        let d = vec3::sub(p_j, p_i);
        // 零長要素（2 節点が同一座標）は材軸方向を定義できないため、長さ 1 の
        // 退化しないスケールに置き換えて ex を全体 X 方向へ倒す。
        let l = vec3::norm(d);
        let l = if l < 1e-12 { 1.0 } else { l };

        let ex = vec3::scale(d, 1.0 / l);

        // ref_vec から材軸成分を抜いた残差（グラム・シュミット）。ex は単位ベクトル
        // なので、内積を係数にそのまま引けばよい。
        let reject_ex = |v: [f64; 3]| vec3::sub(v, vec3::scale(ex, vec3::dot(v, ex)));

        // 正規化の除算は `vec3::unit` へ寄せない。`unit` の縮退判定は mm 座標を
        // 前提とした `ZERO_TOL`（1e-9）だが、ここで測るのは無次元の残差ベクトルで
        // あり、判定値 1e-12 の意味が違う。
        let mut ey = reject_ex(ref_vec);
        let eyl = vec3::norm(ey);
        if eyl > 1e-12 {
            ey = [ey[0] / eyl, ey[1] / eyl, ey[2] / eyl];
        } else {
            // ref_vec が材軸と平行で残差が消えた場合は、材軸から最も傾いた
            // 全体軸を代わりの基準に採る。
            let alt = reject_ex(if ex[0].abs() < 0.9 {
                [1.0, 0.0, 0.0]
            } else {
                [0.0, 1.0, 0.0]
            });
            let altl = vec3::norm(alt);
            ey = if altl > 1e-12 {
                [alt[0] / altl, alt[1] / altl, alt[2] / altl]
            } else {
                [0.0, 1.0, 0.0]
            };
        }

        let ez = vec3::cross(ex, ey);

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

    /// 全体座標系の要素変位から、局所座標系の材端力 `f_local = K_local·(R·u)` を返す。
    ///
    /// 弾性要素（[`crate::springs::spring`] の節点バネ・[`crate::frame::truss`] の
    /// トラス）が `recover_forces` で断面力を組み立てる前半部分。要素が返す断面力の
    /// 形（バネは 6 成分すべて・トラスは軸力のみ）は要素ごとに違うため、共有できるのは
    /// ここまでで、符号規約の適用は各要素に残る。
    ///
    /// `u_elem` が 12 成分に満たない場合は `None`（要素の自由度が揃っていない）。
    pub(crate) fn local_end_forces(&self, k_local: &LocalMat, u_elem: &[f64]) -> Option<[f64; 12]> {
        if u_elem.len() < 12 {
            return None;
        }
        let mut arr = [0.0; 12];
        arr.copy_from_slice(&u_elem[..12]);
        let u_local = self.rotate_to_local(&arr);
        let mut f_local = [0.0; 12];
        for (i, fi) in f_local.iter_mut().enumerate() {
            let mut s = 0.0;
            for (j, &uj) in u_local.iter().enumerate() {
                s += k_local.get(i, j) * uj;
            }
            *fi = s;
        }
        Some(f_local)
    }
}
