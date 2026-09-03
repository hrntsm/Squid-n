//! [`ElementBehavior`](crate::behavior::ElementBehavior) 実装。
//!
//! - 自由度写像・接線剛性・内力・質量行列・断面力回復を要素インターフェースへ接続

use super::element::ShellElement;
use super::geom::element_area;
use super::shape::{jacobian, jacobian_det, shape_2d, GAUSS_PTS_2};
use crate::behavior::{LocalMat, MassOption};
use smallvec::SmallVec;
use squid_n_core::dof::DofMap;

// ---------------------------------------------------------------------------
// ElementBehavior implementation
// ---------------------------------------------------------------------------
impl crate::behavior::ElementBehavior for ShellElement {
    fn n_dof(&self) -> usize {
        24
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        crate::behavior::node_global_dofs(&self.nodes, dof)
    }

    /// 剛床所属（`membrane_active == false`）の面内成分は、厳密なゼロにはならない。
    /// `apply_rigid_floor_membrane_off` が特異回避のため対角へ入れるダミー剛性 1.0 に
    /// 由来する微小な寄生剛性が残る（実剛性より約 10 桁小さく実害はない）。
    ///
    /// `internal_force`（[`crate::behavior::elastic_disp_behavior`] が生成）はこの行列をそのまま
    /// 使うため、寄生分は剛性と内力で同じ値になり、数学的には整合する。
    fn tangent_stiffness(&self, _ctx: &crate::behavior::Ctx) -> LocalMat {
        let mut k_local = self.local_stiffness();
        self.apply_rigid_floor_membrane_off(&mut k_local);
        self.frame.to_global(&k_local)
    }

    crate::behavior::elastic_disp_behavior!(ShellElement, 24);

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        let area = element_area(&self.coords);
        let m_total = self.density * self.t * area;
        let mut mm = LocalMat::zeros(24);
        match opt {
            MassOption::Lumped => {
                let m_node = m_total / 4.0;
                for i in 0..4 {
                    let bo = i * 6;
                    mm.set(bo, bo, m_node);
                    mm.set(bo + 1, bo + 1, m_node);
                    mm.set(bo + 2, bo + 2, m_node);
                }
            }
            MassOption::Consistent => {
                // Consistent mass uses 2×2 Gauss integration of NᵀρtN
                let lc = self.local_coords();
                for gi in 0..2 {
                    for gj in 0..2 {
                        let gp = gi * 2 + gj;
                        let xi = GAUSS_PTS_2[gp].0;
                        let eta = GAUSS_PTS_2[gp].1;
                        let det_j = jacobian_det(&jacobian(xi, eta, &lc));
                        let weight = det_j;
                        let n = shape_2d(xi, eta);
                        let rho_t = self.density * self.t;
                        for a in 0..4 {
                            let bo_a = a * 6;
                            let na = n[a];
                            for b in 0..4 {
                                let bo_b = b * 6;
                                let nb = n[b];
                                let contrib = na * nb * rho_t * weight;
                                for d in 0..3 {
                                    let ia = bo_a + d;
                                    let ib = bo_b + d;
                                    mm.set(ia, ib, mm.get(ia, ib) + contrib);
                                }
                            }
                        }
                    }
                }
            }
        }
        mm
    }

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::frame::beam::MemberForces> {
        if u_elem.len() < 24 {
            return None;
        }
        let mut arr = [0.0; 24];
        arr.copy_from_slice(&u_elem[..24]);
        let resultants = self.recover_resultants(&arr);
        let data: Vec<(f64, [f64; 6])> = resultants
            .into_iter()
            .map(|(pt, r)| (pt[0], [r.nx, r.ny, r.nxy, r.mx, r.my, r.mxy]))
            .collect();
        Some(crate::frame::beam::MemberForces { at: data })
    }
}
