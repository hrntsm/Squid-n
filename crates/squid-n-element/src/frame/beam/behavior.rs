//! [`ElementBehavior`] トレイト実装（自由度写像・接線/幾何剛性・内力・質量行列）。

use super::element::BeamElement;
use crate::behavior::{Ctx, ElementBehavior, LocalMat, LocalVec, MassOption};
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};

impl ElementBehavior for BeamElement {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &nid in &self.nodes {
            let ni = nid.index();
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                if let Some(active) = dof.active(g) {
                    gdofs.push(active as usize);
                } else {
                    gdofs.push(usize::MAX);
                }
            }
        }
        gdofs
    }

    fn tangent_stiffness(&self, _ctx: &Ctx) -> LocalMat {
        // 要素ローカルの 12×12 を全体系へ回す（K_global = Rᵀ K_local R）。
        // ElementBehavior::tangent_stiffness は全体系を返す契約（シェルと同じ）。
        // これを欠くと、ローカル系とグローバル系が一致しない部材（鉛直柱・
        // 任意方向材・非対称断面 iy≠iz）で組立 K が誤る。
        self.axis.to_global(&self.local_stiffness())
    }

    fn geometric_stiffness(&self, n: f64) -> LocalMat {
        // 幾何剛性も弾性剛性と整合させる: 可撓長で組み、剛域変換で節点自由度へ写す。
        // 剛域があれば P-δ は可撓部でのみ生じ、剛域は剛体アームとして働く。
        // 剛域なし（li=lj=0）では従来どおり全長 L の幾何剛性に一致する。
        let (li, lj) = self.rigid_lengths();
        let l = self.length - li - lj;
        if l < 1e-12 {
            return LocalMat::zeros(12);
        }
        let c = n / l;
        let mut kg = LocalMat::zeros(12);
        let mut s = |i: usize, j: usize, v: f64| {
            kg.set(i, j, v);
            if i != j {
                kg.set(j, i, v);
            }
        };
        // xy面（uy=1,rz=5 / uy_j=7,rz_j=11）
        s(1, 1, c * 6.0 / 5.0);
        s(7, 7, c * 6.0 / 5.0);
        s(1, 7, -c * 6.0 / 5.0);
        s(1, 5, c * l / 10.0);
        s(1, 11, c * l / 10.0);
        s(5, 7, -c * l / 10.0);
        s(7, 11, -c * l / 10.0);
        s(5, 5, c * 2.0 * l * l / 15.0);
        s(11, 11, c * 2.0 * l * l / 15.0);
        s(5, 11, -c * l * l / 30.0);
        // xz面（uz=2,ry=4 / uz_j=8,ry_j=10）§4.1 規約で並進-回転結合項の符号が逆（ry の向き）
        s(2, 2, c * 6.0 / 5.0);
        s(8, 8, c * 6.0 / 5.0);
        s(2, 8, -c * 6.0 / 5.0);
        s(2, 4, -c * l / 10.0);
        s(2, 10, -c * l / 10.0);
        s(4, 8, c * l / 10.0);
        s(8, 10, c * l / 10.0);
        s(4, 4, c * 2.0 * l * l / 15.0);
        s(10, 10, c * 2.0 * l * l / 15.0);
        s(4, 10, -c * l * l / 30.0);
        // 剛域変換 → 全体系（P-Δ を組立系で正しく加算するため）
        let kg_node = self.apply_rigid_zone_transform(&kg, li, lj);
        self.axis.to_global(&kg_node)
    }

    fn internal_force(&self, _ctx: &Ctx) -> LocalVec {
        // trial_disp はグローバル系で蓄積されるため、グローバル剛性で内力を評価する。
        // f_global = (R^T·K_local·R)·u_global
        // Newton 反復中の未確定変位も反映する（トライアル追従。committed のみを
        // 参照すると反復中に内力が凍結し、収束が準ニュートンに劣化する）。
        let k = self.axis.to_global(&self.local_stiffness());
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        for i in 0..12 {
            let mut s = 0.0;
            for j in 0..12 {
                s += k.get(i, j) * self.trial_disp[j];
            }
            f.data[i] = s;
        }
        f
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        for i in 0..12 {
            self.trial_disp[i] += du.data[i];
        }
        if commit {
            self.committed_disp = self.trial_disp;
        }
    }

    fn commit_state(&mut self) {
        self.committed_disp = self.trial_disp;
    }

    fn revert_state(&mut self) {
        self.trial_disp = self.committed_disp;
    }

    fn snapshot_state(&self) -> Box<dyn std::any::Any> {
        Box::new((self.committed_disp, self.trial_disp))
    }

    fn restore_state(&mut self, state: &dyn std::any::Any) {
        let (committed, trial) =
            crate::behavior::downcast_snapshot::<([f64; 12], [f64; 12])>("BeamElement", state);
        self.committed_disp = *committed;
        self.trial_disp = *trial;
    }

    fn serialize_checkpoint(&self) -> Vec<u8> {
        // トライアル追従化により変位が蓄積されるようになったため、
        // チェックポイントに committed/trial の両変位を含める（レジューム時に
        // 変位 0 から再計算されて内力が不整合になるのを防ぐ）。
        bincode::serialize(&(self.committed_disp, self.trial_disp)).expect("serialize checkpoint")
    }

    fn deserialize_checkpoint(
        &mut self,
        data: &[u8],
    ) -> Result<(), crate::behavior::CheckpointError> {
        // 旧チェックポイント（変位未収録・空バイト列）は「状態なし」として許容する。
        if data.is_empty() {
            return Ok(());
        }
        let (committed, trial): ([f64; 12], [f64; 12]) = bincode::deserialize(data)
            .map_err(|e| crate::behavior::CheckpointError::Decode(e.to_string()))?;
        self.committed_disp = committed;
        self.trial_disp = trial;
        Ok(())
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        let m = self.density * self.a_mass * self.length;
        let mut mm = LocalMat::zeros(12);
        match opt {
            MassOption::Lumped => {
                // 並進 3 成分が等しい対角行列は回転不変（Rᵀ·(m/2)I·R = (m/2)I）の
                // ため、全体系変換は不要。
                for d in [0, 1, 2, 6, 7, 8] {
                    mm.set(d, d, m / 2.0);
                }
                mm
            }
            MassOption::Consistent => {
                let c1 = m / 6.0;
                let c2 = m / 420.0;
                let l = self.length;
                let l2 = l * l;
                // Axial (Ux):  indices 0,6
                mm.set(0, 0, 2.0 * c1);
                mm.set(0, 6, 1.0 * c1);
                mm.set(6, 0, 1.0 * c1);
                mm.set(6, 6, 2.0 * c1);
                // Torsion (Rx): indices 3,9
                let ct = self.density * self.j * l / 6.0;
                mm.set(3, 3, 2.0 * ct);
                mm.set(3, 9, 1.0 * ct);
                mm.set(9, 3, 1.0 * ct);
                mm.set(9, 9, 2.0 * ct);
                // Bending: Hermite 梁の一貫質量（4x4 ブロック）。
                // DOF は連続ではないためインデックス配列で指定する。
                //   Uy-Rz 面: [Uy_i=1, Rz_i=5, Uy_j=7, Rz_j=11]
                //   Uz-Ry 面: [Uz_i=2, Ry_i=4, Uz_j=8, Ry_j=10]（回転符号は逆）
                let b4 = |mm: &mut LocalMat, idx: [usize; 4], sign: f64| {
                    let [d0, r0, d1, r1] = idx;
                    // 並進-並進
                    mm.set(d0, d0, 156.0 * c2);
                    mm.set(d0, d1, 54.0 * c2);
                    mm.set(d1, d0, 54.0 * c2);
                    mm.set(d1, d1, 156.0 * c2);
                    // 並進-回転
                    mm.set(d0, r0, 22.0 * l * c2 * sign);
                    mm.set(r0, d0, 22.0 * l * c2 * sign);
                    mm.set(d0, r1, -13.0 * l * c2 * sign);
                    mm.set(r1, d0, -13.0 * l * c2 * sign);
                    mm.set(d1, r0, 13.0 * l * c2 * sign);
                    mm.set(r0, d1, 13.0 * l * c2 * sign);
                    mm.set(d1, r1, -22.0 * l * c2 * sign);
                    mm.set(r1, d1, -22.0 * l * c2 * sign);
                    // 回転-回転
                    mm.set(r0, r0, 4.0 * l2 * c2);
                    mm.set(r0, r1, -3.0 * l2 * c2);
                    mm.set(r1, r0, -3.0 * l2 * c2);
                    mm.set(r1, r1, 4.0 * l2 * c2);
                };
                b4(&mut mm, [1, 5, 7, 11], 1.0);
                b4(&mut mm, [2, 4, 8, 10], -1.0);
                // 整合質量は軸方向（m/3 系）と曲げ方向（156m/420 系）で係数が異なり
                // 回転不変ではないため、剛性行列と同様に要素局所系から全体系へ回す
                // （M_global = Rᵀ M_local R）。これを欠くと鉛直柱・斜材で質量が
                // 誤った全体軸へ配分される。
                self.axis.to_global(&mm)
            }
        }
    }

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        if u_elem.len() < 12 {
            return None;
        }
        let mut arr = [0.0; 12];
        arr.copy_from_slice(&u_elem[..12]);
        Some(self.recover_forces(&arr))
    }

    /// 弾性材は常に線形なので、蓄積した trial 変位からの復元でよい
    /// （非線形解析中の弾性材＝`recover_forces` と同じ結果）。
    fn state_member_forces(&self, _ctx: &Ctx) -> Option<crate::beam::MemberForces> {
        Some(self.recover_forces(&self.trial_disp))
    }
}
