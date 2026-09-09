//! 節点変位からの部材内力復元。

use super::element::{BeamElement, MemberForces};

/// 局所座標の端部節点力 12 成分から、評価断面 `eval_sections` の断面内力を組み立てる。
///
/// `f_local` は `[Ni,Qyi,Qzi,Mxi,Myi,Mzi, Nj,Qyj,Qzj,Mxj,Myj,Mzj]` の並びである。
/// 戻り値は部材全長で連続な断面内力（軸力は引張正）。
///
/// i 端側（xi<0.5）は節点力 f0..f5 の符号を断面内力へ反転し、j 端側は f6..f11 を
/// そのまま用いる。
pub(crate) fn member_forces_from_end_forces(
    f_local: &[f64; 12],
    length: f64,
    eval_sections: &[f64],
) -> MemberForces {
    let mut at = Vec::with_capacity(eval_sections.len());
    for &xi in eval_sections {
        let (n, qy, qz, mx, my, mz) = if xi < 0.5 {
            let n = -f_local[0];
            let qy = f_local[1];
            let qz = f_local[2];
            let mx = -f_local[3];
            let my = -f_local[4] - f_local[2] * xi * length;
            let mz = -f_local[5] + f_local[1] * xi * length;
            (n, qy, qz, mx, my, mz)
        } else {
            let n = f_local[6];
            let qy = -f_local[7];
            let qz = -f_local[8];
            let mx = f_local[9];
            let my = f_local[10] - f_local[8] * (1.0 - xi) * length;
            let mz = f_local[11] + f_local[7] * (1.0 - xi) * length;
            (n, qy, qz, mx, my, mz)
        };
        at.push((xi, [n, qy, qz, mx, my, mz]));
    }

    MemberForces { at }
}

impl BeamElement {
    pub fn recover_forces(&self, u_elem_global: &[f64; 12]) -> MemberForces {
        let u_local = self.axis.rotate_to_local(u_elem_global);
        let k_local = self.local_stiffness();
        let mut f_local = [0.0; 12];
        for (i, fi) in f_local.iter_mut().enumerate() {
            let mut s = 0.0;
            for (j, &uj) in u_local.iter().enumerate() {
                s += k_local.get(i, j) * uj;
            }
            *fi = s;
        }

        member_forces_from_end_forces(&f_local, self.length, &self.eval_sections)
    }
}
