//! [`ElementBehavior`] トレイト実装（自由度写像・接線/幾何剛性・内力・質量行列）。

use super::element::BeamElement;
use crate::behavior::{Ctx, ElementBehavior, LocalMat, MassOption};
use smallvec::SmallVec;
use squid_n_core::dof::DofMap;

impl ElementBehavior for BeamElement {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        crate::behavior::node_global_dofs(&self.nodes, dof)
    }

    fn tangent_stiffness(&self, _ctx: &Ctx) -> LocalMat {
        self.axis.to_global(&self.local_stiffness())
    }

    fn geometric_stiffness(&self, n: f64) -> LocalMat {
        let (li, lj) = self.rigid_lengths();
        let kg_node =
            crate::frame::prismatic::geometric_stiffness(n, self.length - li - lj, li, lj);
        self.axis.to_global(&kg_node)
    }

    crate::behavior::elastic_disp_behavior!(BeamElement, 12);

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        match opt {
            MassOption::Lumped => {
                crate::frame::prismatic::lumped_mass(self.density * self.a_mass * self.length)
            }
            MassOption::Consistent => {
                let mass_properties = if self.mass_properties != Default::default() {
                    self.mass_properties
                } else {
                    (self.mass_properties_resolver)()
                        .unwrap_or_else(|error| panic!("質量特性を解決できません: {error}"))
                };
                let (li, lj) = self.rigid_lengths();
                let flex_length = self.length - li - lj;
                let phi_y = if flex_length > 0.0 && self.g > 0.0 && self.as_z > 0.0 {
                    12.0 * self.e * self.iy / (self.g * self.as_z * flex_length.powi(2))
                } else {
                    0.0
                };
                let phi_z = if flex_length > 0.0 && self.g > 0.0 && self.as_y > 0.0 {
                    12.0 * self.e * self.iz / (self.g * self.as_y * flex_length.powi(2))
                } else {
                    0.0
                };
                let flex = crate::frame::prismatic::consistent_mass_timoshenko(
                    mass_properties,
                    flex_length,
                    phi_z,
                    phi_y,
                );
                let k_flex = self.local_stiffness_flex_raw();
                let releases = self.end_releases();
                let mm = crate::frame::prismatic::condense_end_releases_with_mass(
                    &k_flex,
                    &flex,
                    mass_properties,
                    li,
                    lj,
                    &releases,
                )
                .unwrap_or_else(|| {
                    panic!(
                        "BeamElement の端部解放質量を縮約できません: Kbb が特異です（解放条件を確認してください）"
                    )
                });
                self.axis.to_global(&mm)
            }
        }
    }

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::frame::beam::MemberForces> {
        if u_elem.len() < 12 {
            return None;
        }
        let mut arr = [0.0; 12];
        arr.copy_from_slice(&u_elem[..12]);
        Some(self.recover_forces(&arr))
    }

    fn state_member_forces(&self, _ctx: &Ctx) -> Option<crate::frame::beam::MemberForces> {
        Some(self.recover_forces(&self.trial_disp))
    }
}
