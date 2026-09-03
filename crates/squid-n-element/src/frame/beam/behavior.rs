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
        // 要素ローカルの 12×12 を全体系へ回す（K_global = Rᵀ K_local R）。
        // ElementBehavior::tangent_stiffness は全体系を返す契約（シェルと同じ）。
        // これを欠くと、ローカル系とグローバル系が一致しない部材（鉛直柱・
        // 任意方向材・非対称断面 iy≠iz）で組立 K が誤る。
        self.axis.to_global(&self.local_stiffness())
    }

    fn geometric_stiffness(&self, n: f64) -> LocalMat {
        let (li, lj) = self.rigid_lengths();
        let kg_node =
            crate::frame::prismatic::geometric_stiffness(n, self.length - li - lj, li, lj);
        // P-Δ を組立系で正しく加算するため全体系へ回す。
        self.axis.to_global(&kg_node)
    }

    crate::behavior::elastic_disp_behavior!(BeamElement, 12);

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        let m = self.density * self.a_mass * self.length;
        match opt {
            MassOption::Lumped => crate::frame::prismatic::lumped_mass(m),
            MassOption::Consistent => {
                // 部材軸まわりの回転慣性 ρ·J·l/6 を持つ。
                let ct = self.density * self.j * self.length / 6.0;
                let mm = crate::frame::prismatic::consistent_mass(m, self.length, ct);
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

    /// 弾性材は常に線形なので、蓄積した trial 変位からの復元でよい
    /// （非線形解析中の弾性材＝`recover_forces` と同じ結果）。
    fn state_member_forces(&self, _ctx: &Ctx) -> Option<crate::frame::beam::MemberForces> {
        Some(self.recover_forces(&self.trial_disp))
    }
}
