//! 弾性剛性行列 12×12 の構築。
//!
//! Timoshenko 梁の raw 剛性、剛域変換、端部回転ばねの静縮約を経て節点自由度の
//! 局所剛性 [`BeamElement::local_stiffness`] を組み立てる。

use super::element::BeamElement;
use super::linalg::invert_small;
use crate::behavior::LocalMat;
use squid_n_core::model::EndCondition;

impl BeamElement {
    pub fn local_stiffness_raw(&self) -> LocalMat {
        let (e, g, a, iy, iz, jj, l) = (
            self.e,
            self.g,
            self.a,
            self.iy,
            self.iz,
            self.j,
            self.length,
        );
        if l < 1e-12 {
            return LocalMat::zeros(12);
        }
        let phiz = 12.0 * e * iz / (g * self.as_y * l * l);
        let phiy = 12.0 * e * iy / (g * self.as_z * l * l);
        let az = e * iz / ((1.0 + phiz) * l * l * l);
        let ay = e * iy / ((1.0 + phiy) * l * l * l);

        let mut k = LocalMat::zeros(12);
        let mut s = |i: usize, j: usize, v: f64| {
            k.set(i, j, v);
            if i != j {
                k.set(j, i, v);
            }
        };

        s(0, 0, e * a / l);
        s(6, 6, e * a / l);
        s(0, 6, -e * a / l);
        s(3, 3, g * jj / l);
        s(9, 9, g * jj / l);
        s(3, 9, -g * jj / l);

        s(1, 1, 12.0 * az);
        s(7, 7, 12.0 * az);
        s(1, 7, -12.0 * az);
        s(1, 5, 6.0 * az * l);
        s(1, 11, 6.0 * az * l);
        s(5, 7, -6.0 * az * l);
        s(7, 11, -6.0 * az * l);
        s(5, 5, (4.0 + phiz) * az * l * l);
        s(11, 11, (4.0 + phiz) * az * l * l);
        s(5, 11, (2.0 - phiz) * az * l * l);

        s(2, 2, 12.0 * ay);
        s(8, 8, 12.0 * ay);
        s(2, 8, -12.0 * ay);
        s(2, 4, -6.0 * ay * l);
        s(2, 10, -6.0 * ay * l);
        s(4, 8, 6.0 * ay * l);
        s(8, 10, 6.0 * ay * l);
        s(4, 4, (4.0 + phiy) * ay * l * l);
        s(10, 10, (4.0 + phiy) * ay * l * l);
        s(4, 10, (2.0 - phiy) * ay * l * l);

        k
    }

    pub(crate) fn apply_rigid_zone_transform(
        &self,
        k_flex: &LocalMat,
        li: f64,
        lj: f64,
    ) -> LocalMat {
        if li.abs() < 1e-12 && lj.abs() < 1e-12 {
            return LocalMat {
                n: k_flex.n,
                data: k_flex.data.clone(),
            };
        }
        // Tr: 12×12 — flex端自由度(i', j') → 節点自由度(i, j)
        // 剛域腕の運動学 u_flex = u_node + θ_node × r。可撓端は i 端で節点から
        // 内側へ +li·ex（j 端で −lj·ex）にあるため r_i=(+li,0,0), r_j=(−lj,0,0)。
        // θ×r を成分展開すると（ex は局所 x）:
        //   i端: ux_i'=ux_i, uy_i'=uy_i+li*rz_i, uz_i'=uz_i-li*ry_i, 回転は不変
        //   j端: ux_j'=ux_j, uy_j'=uy_j-lj*rz_j, uz_j'=uz_j+lj*ry_j, 回転は不変
        // （剛体回転で K·u≈0 となる符号。test_rigid_zone_preserves_rigid_body_rotation）
        let mut tr = LocalMat::zeros(12);
        for i in 0..12 {
            tr.set(i, i, 1.0);
        }
        // i端 (index 0..5): uy(1)←rz(5), uz(2)←ry(4)
        tr.set(1, 5, li);
        tr.set(2, 4, -li);
        // j端 (index 6..11): uy(7)←rz(11), uz(8)←ry(10)
        tr.set(7, 11, -lj);
        tr.set(8, 10, lj);

        // K_node = Tr^T * K_flex * Tr
        let mut tmp = LocalMat::zeros(12);
        for i in 0..12 {
            for j in 0..12 {
                let mut s = 0.0;
                for k in 0..12 {
                    s += k_flex.get(i, k) * tr.get(k, j);
                }
                tmp.set(i, j, s);
            }
        }
        let mut kn = LocalMat::zeros(12);
        for i in 0..12 {
            for j in 0..12 {
                let mut s = 0.0;
                for k in 0..12 {
                    s += tr.get(k, i) * tmp.get(k, j);
                }
                kn.set(i, j, s);
            }
        }
        kn
    }

    /// 端部条件（剛接・ピン・半剛）を要素剛性へ反映し、12×12（節点自由度のみ）を返す。
    ///
    /// 剛接（Fixed）は「節点回転 = 要素端回転」を厳密に満たすため、要素端回転を
    /// そのまま節点回転自由度に残す（内部自由度・ばね不要）。全端剛接なら raw を
    /// そのまま返す（ペナルティ近似を用いない厳密な扱い）。
    ///
    /// ピン・半剛の端のみ、要素端回転を内部自由度へ分離し、節点回転との間に
    /// 回転ばね k_s を挟んで静縮約する（K* = Kaa − Kab·Kbb⁻¹·Kba）。
    ///   - ピン    (k_s = 0):        要素端回転が自由 → 厳密なモーメント解放
    ///   - 半剛    (k_s = k_theta):  有限の接合部回転剛性 [N·mm/rad]
    ///
    /// 内部並びは [外部 0..11（節点 ux,uy,uz,rx,ry,rz ×2）, 内部 12..（解放した
    /// 要素端回転を出現順に並べる）]。
    fn condense_end_springs(&self, k_elem: &LocalMat) -> LocalMat {
        // 回転自由度（局所 index）と所属端（0=i, 1=j）
        const ROT_DOFS: [(usize, usize); 6] = [(3, 0), (4, 0), (5, 0), (9, 1), (10, 1), (11, 1)];

        // Fixed は内部自由度を作らず節点回転に残す（厳密剛接）。
        // Pinned/SemiRigid のみ内部自由度＋回転ばねを導入する。
        let released_spring = |cond: &EndCondition| -> Option<f64> {
            match cond {
                EndCondition::Fixed => None,
                EndCondition::Pinned => Some(0.0),
                EndCondition::SemiRigid { k_theta } => Some(*k_theta),
            }
        };

        // 解放（非剛接）する回転自由度: (要素回転 DOF, ばね剛性 k_s)
        let mut released: Vec<(usize, f64)> = Vec::new();
        for &(r, end) in ROT_DOFS.iter() {
            if let Some(ks) = released_spring(&self.end_cond[end]) {
                released.push((r, ks));
            }
        }

        // 全端剛接: raw をそのまま返す（厳密）
        if released.is_empty() {
            return LocalMat {
                n: 12,
                data: k_elem.data.clone(),
            };
        }

        // 12（節点）＋ 解放数（内部の要素端回転）の系を組む
        let n_int = released.len();
        let n = 12 + n_int;
        let mut k = vec![0.0; n * n];

        // 要素 DOF → 組立 DOF 写像。解放回転は内部（12..）へ、それ以外は同位置。
        let mut map = [0usize; 12];
        for (i, m) in map.iter_mut().enumerate() {
            *m = i;
        }
        for (idx, &(r, _)) in released.iter().enumerate() {
            map[r] = 12 + idx;
        }

        // 要素剛性を配置（解放回転の行・列は内部自由度へ移る）
        for i in 0..12 {
            for j in 0..12 {
                k[map[i] * n + map[j]] += k_elem.get(i, j);
            }
        }

        // 回転ばね: 節点回転 r ↔ 内部の要素端回転 (12+idx)
        for (idx, &(r, ks)) in released.iter().enumerate() {
            let ir = 12 + idx;
            k[r * n + r] += ks;
            k[ir * n + ir] += ks;
            k[r * n + ir] -= ks;
            k[ir * n + r] -= ks;
        }

        // 内部 DOF (12..n) を静縮約: K* = Kaa − Kab·Kbb⁻¹·Kba
        let na = 12;
        let nb = n_int;
        let mut kaa = vec![0.0; na * na];
        let mut kab = vec![0.0; na * nb];
        let mut kba = vec![0.0; nb * na];
        let mut kbb = vec![0.0; nb * nb];

        for i in 0..na {
            for j in 0..na {
                kaa[i * na + j] = k[i * n + j];
            }
            for j in 0..nb {
                kab[i * nb + j] = k[i * n + (na + j)];
                kba[j * na + i] = k[(na + j) * n + i];
            }
        }
        for i in 0..nb {
            for j in 0..nb {
                kbb[i * nb + j] = k[(na + i) * n + (na + j)];
            }
        }

        let kbb_inv = invert_small(&kbb, nb);

        // kab_kbbinv = Kab * Kbb^-1
        let mut kab_kbbinv = vec![0.0; na * nb];
        for i in 0..na {
            for j in 0..nb {
                let mut s = 0.0;
                for l in 0..nb {
                    s += kab[i * nb + l] * kbb_inv[l * nb + j];
                }
                kab_kbbinv[i * nb + j] = s;
            }
        }

        let mut kstar = LocalMat::zeros(na);
        for i in 0..na {
            for j in 0..na {
                let mut s = kaa[i * na + j];
                for l in 0..nb {
                    s -= kab_kbbinv[i * nb + l] * kba[l * na + j];
                }
                kstar.set(i, j, s);
            }
        }
        kstar
    }

    /// 可撓部（剛域を除いた部分）の局所剛性 12×12。剛域変換の**手前**の状態で、
    /// 端部条件（ピン・半剛）の静縮約まで済ませたもの。
    ///
    /// [`Self::local_stiffness`] はこれに剛域変換を掛けたものであり、材端集中ばね梁
    /// （`concentrated::compute_kstar`）は「これ → 材端塑性ばねの静縮約 → 剛域変換」
    /// の順で組む。両者が同じ土台を共有することで、剛域・端部条件の扱いが
    /// 線形要素と非線形要素で食い違わないようにしている。
    pub(crate) fn local_stiffness_flex(&self) -> LocalMat {
        let l_flex = self.length - self.rigid.length_i - self.rigid.length_j;
        let k_raw = if l_flex > 1e-12 {
            let mut beam = self.clone();
            beam.length = l_flex;
            beam.end_cond = [EndCondition::Fixed, EndCondition::Fixed];
            // 軸・ねじり剛性は剛域で増大させない（断面性能の l0/l 補正）。可撓長 l0 で
            // 組み立てるため A·(l0/l)・J·(l0/l) を用いると EA/l・GJ/l（いずれも節点間長
            // 基準）となり、曲げ（せん断を含む）のみ剛域変換で剛とする扱いに揃う。
            // 剛域なしでは補正 1。
            beam.a = self.a * (l_flex / self.length);
            beam.j = self.j * (l_flex / self.length);
            beam.local_stiffness_raw()
        } else {
            LocalMat::zeros(12)
        };

        // 剛域を持たない可とう部で端部ばね静縮約 → 12×12
        self.condense_end_springs(&k_raw)
    }

    pub fn local_stiffness(&self) -> LocalMat {
        // 剛域変換で節点自由度へ
        let li = self.rigid.length_i;
        let lj = self.rigid.length_j;
        self.apply_rigid_zone_transform(&self.local_stiffness_flex(), li, lj)
    }
}
