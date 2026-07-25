//! 節点変位からの部材内力復元。
//!
//! 局所剛性 × 局所変位で端部節点力を求め、評価断面（危険断面）ごとに
//! N/Qy/Qz/Mx/My/Mz を分布させて [`MemberForces`] を組み立てる。

use super::element::{BeamElement, MemberForces};

/// 局所座標の端部節点力 12 成分から、評価断面 `eval_sections` の断面内力を組み立てる。
///
/// `f_local` は `[Ni,Qyi,Qzi,Mxi,Myi,Mzi, Nj,Qyj,Qzj,Mxj,Myj,Mzj]` の並びで、
/// **部材が節点へ及ぼす力**（＝局所剛性 × 局所変位、または要素の復元力）である。
/// 戻り値は部材全長で連続な断面内力（切断法。軸力は引張正）。
///
/// 端部節点力さえ与えられれば断面内力は静力学（釣合い）だけで定まるため、
/// 弾性梁の `K·u` でも弾塑性要素の状態依存な復元力でも同じ式が使える。材端集中
/// ばね梁・ファイバー梁など、剛性と内力の関係が線形でない要素の内力分布も
/// これを共有して組み立てる（[`crate::behavior::ElementBehavior::state_member_forces`]）。
///
/// i 端側（xi<0.5）は節点力 f0..f5 の符号を断面内力へ反転し、j 端側は f6..f11 を
/// そのまま用いる。両者は連続な内力場（dMz/dx = Qy・dMy/dx = −Qz）を与える。
pub(crate) fn member_forces_from_end_forces(
    f_local: &[f64; 12],
    length: f64,
    eval_sections: &[f64],
) -> MemberForces {
    let mut at = Vec::with_capacity(eval_sections.len());
    for &xi in eval_sections {
        // 軸力 N は部材内力（引張正）。スパン内軸方向荷重が無い限り一定で、
        // i 端側は節点力 f_local[0]（引張時に -N）、j 端側は f_local[6]（+N）。
        // 旧実装の f0·(1-ξ)+f6·ξ は両端で符号が逆の節点力を線形補間しており、
        // 中央で N=0 となる誤りだったため、せん断と同じ端別採用に修正。
        //
        // モーメント Mx/My/Mz も同様に断面内力（j 端側の式と連続な内力場）で
        // 統一する。i 端側では節点モーメント f3/f4/f5 は断面内力と符号が
        // 逆のため反転して用いる（反転しないと ξ=0.5 で図がジャンプする）。
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
        // f_local = K_local * u_local (in local coords, at node ends)
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
