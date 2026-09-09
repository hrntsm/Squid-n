//! 弾性剛性行列 12×12 の構築。

use super::element::BeamElement;
use crate::behavior::LocalMat;
use smallvec::SmallVec;
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
        crate::frame::rigid_arm::transform_stiffness(k_flex, li, lj)
    }

    /// 端部条件を要素剛性へ反映し、12×12（節点自由度のみ）を返す。
    ///
    /// ピン・半剛の端のみ、要素端回転を内部自由度へ分離し、節点回転との間に
    /// 回転ばね k_s を挟んで静縮約する。
    ///   - ピン    (k_s = 0)
    ///   - 半剛    (k_s = k_theta [N·mm/rad])
    ///
    /// 内部並びは [外部 0..11（節点 ux,uy,uz,rx,ry,rz ×2）, 内部 12..（解放した
    /// 要素端回転を出現順に並べる）]。
    fn condense_end_springs(&self, k_elem: &LocalMat) -> LocalMat {
        const ROT_DOFS: [(usize, usize); 6] = [(3, 0), (4, 0), (5, 0), (9, 1), (10, 1), (11, 1)];

        let released_spring = |cond: &EndCondition| -> Option<f64> {
            match cond {
                EndCondition::Fixed => None,
                EndCondition::Pinned => Some(0.0),
                EndCondition::SemiRigid { k_theta } => Some(*k_theta),
            }
        };

        let has_torsion = self.j > 0.0 && self.g > 0.0;

        let mut released: SmallVec<[(usize, f64); 6]> = SmallVec::new();
        for &(r, end) in ROT_DOFS.iter() {
            let is_torsion = r == 3 || r == 9;
            let spring = match released_spring(&self.end_cond[end]) {
                Some(ks) => Some(ks),
                None if is_torsion && self.torsion_release[end] => Some(0.0),
                None => None,
            };
            let Some(ks) = spring else { continue };
            if is_torsion && !has_torsion {
                continue;
            }
            released.push((r, ks));
        }

        crate::frame::prismatic::condense_end_releases(k_elem, &released)
    }

    /// 剛域長を可撓長が正に残る範囲へ解決した値 (λi, λj)。
    ///
    /// 合計が部材長以上になる場合は剛域なしとして扱う。
    pub(crate) fn rigid_lengths(&self) -> (f64, f64) {
        crate::frame::rigid_arm::resolve_lengths(
            self.rigid.rigid_length_i(),
            self.rigid.rigid_length_j(),
            self.length,
        )
    }

    pub(crate) fn local_stiffness_flex(&self) -> LocalMat {
        let (li, lj) = self.rigid_lengths();
        let l_flex = self.length - li - lj;
        let k_raw = if l_flex > 1e-12 {
            let mut beam = self.clone();
            beam.length = l_flex;
            beam.end_cond = [EndCondition::Fixed, EndCondition::Fixed];
            beam.a = self.a * (l_flex / self.length);
            beam.j = self.j * (l_flex / self.length);
            beam.local_stiffness_raw()
        } else {
            LocalMat::zeros(12)
        };

        self.condense_end_springs(&k_raw)
    }

    /// 節点自由度ベースの局所剛性 12×12（剛域変換・端部条件を適用済み）。
    pub fn local_stiffness(&self) -> LocalMat {
        self.local_stiffness_cache
            .get_or_init(|| {
                let (li, lj) = self.rigid_lengths();
                self.apply_rigid_zone_transform(&self.local_stiffness_flex(), li, lj)
            })
            .clone()
    }
}
