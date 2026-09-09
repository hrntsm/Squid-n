use crate::behavior::{Ctx, ElementBehavior, LocalMat, LocalVec, MassOption};
use squid_n_core::dof::DofMap;

use smallvec::SmallVec;
use squid_n_material::uniaxial::UniaxialMaterial;
use std::any::Any;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SpringModel {
    OneComponent,
    TwoComponent,
}

/// 端バネの N-M 相関パラメータ。
#[derive(Clone, Copy, Debug)]
pub struct MnInteraction {
    /// N=0 での降伏モーメント [N·mm]。
    pub my0: f64,
    /// 軸許容耐力 [N]（正値）。
    pub n_allow: f64,
}

/// 材端集中ばね梁。
pub struct ConcentratedSpringBeam {
    pub elastic: crate::frame::beam::BeamElement,
    pub spring_i: Box<dyn UniaxialMaterial>,
    pub spring_j: Box<dyn UniaxialMaterial>,
    pub model: SpringModel,
    /// N-M 相関。
    pub mn: Option<MnInteraction>,
    /// ばね変形の確定値。
    rot_i: f64,
    rot_j: f64,
    /// ばね変形のトライアル値。
    trial_rot_i: f64,
    trial_rot_j: f64,
    /// 可撓端回転の確定値。
    thb_i: f64,
    thb_j: f64,
    /// 可撓端回転のトライアル値。
    trial_thb_i: f64,
    trial_thb_j: f64,
    flex_stiffness_cache: std::sync::OnceLock<LocalMat>,
}

impl ConcentratedSpringBeam {
    pub fn new(
        elastic: crate::frame::beam::BeamElement,
        spring_i: Box<dyn UniaxialMaterial>,
        spring_j: Box<dyn UniaxialMaterial>,
        model: SpringModel,
    ) -> Self {
        Self {
            elastic,
            spring_i,
            spring_j,
            model,
            mn: None,
            rot_i: 0.0,
            rot_j: 0.0,
            trial_rot_i: 0.0,
            trial_rot_j: 0.0,
            thb_i: 0.0,
            thb_j: 0.0,
            trial_thb_i: 0.0,
            trial_thb_j: 0.0,
            flex_stiffness_cache: std::sync::OnceLock::new(),
        }
    }

    pub fn new_one_component(
        elastic: crate::frame::beam::BeamElement,
        spring_i: Box<dyn UniaxialMaterial>,
        spring_j: Box<dyn UniaxialMaterial>,
    ) -> Self {
        Self::new(elastic, spring_i, spring_j, SpringModel::OneComponent)
    }

    pub fn with_mn_interaction(mut self, my0: f64, n_allow: f64) -> Self {
        self.mn = Some(MnInteraction {
            my0,
            n_allow: n_allow.max(1.0),
        });
        self
    }

    /// 現在の軸力 [N]（引張正）。
    fn current_axial_force(&self, du_local: Option<&[f64; 12]>) -> f64 {
        let ul = self.elastic.axis.rotate_to_local(&self.elastic.trial_disp);
        let mut d = ul[6] - ul[0];
        if let Some(du) = du_local {
            d += du[6] - du[0];
        }
        self.elastic.e * self.elastic.a / self.elastic.length.max(1.0) * d
    }

    fn apply_mn_interaction(&mut self, du_local: Option<&[f64; 12]>) {
        let Some(mn) = self.mn else {
            return;
        };
        let n = self.current_axial_force(du_local);
        let m_lim = (mn.my0 * (1.0 - n.abs() / mn.n_allow)).max(0.02 * mn.my0);
        self.spring_i.set_yield(m_lim);
        self.spring_j.set_yield(m_lim);
    }

    fn k_flex(&self) -> &LocalMat {
        self.flex_stiffness_cache
            .get_or_init(|| self.elastic.local_stiffness_flex())
    }

    fn u_flex_local(&self) -> [f64; 12] {
        let u_local = self.elastic.axis.rotate_to_local(&self.elastic.trial_disp);
        let (li, lj) = self.elastic.rigid_lengths();
        crate::frame::rigid_arm::to_flex_disp(&u_local, li, lj)
    }

    fn solve_internal_equilibrium(&mut self) {
        let k_flex = self.k_flex();
        let u_flex = self.u_flex_local();
        let er = SPRING_ROT_DOFS;
        let thn = [u_flex[er[0]], u_flex[er[1]]];
        let mut thb = [self.trial_thb_i, self.trial_thb_j];

        for _ in 0..50 {
            let mut uh = u_flex;
            uh[er[0]] = thb[0];
            uh[er[1]] = thb[1];
            let mut mb = [0.0_f64; 2];
            for (k, &e) in er.iter().enumerate() {
                let mut s = 0.0;
                for (j, &u) in uh.iter().enumerate() {
                    s += k_flex.get(e, j) * u;
                }
                mb[k] = s;
            }
            let g = [thn[0] - thb[0], thn[1] - thb[1]];
            let (ms_i, kt_i) = self.spring_i.probe(g[0]);
            let (ms_j, kt_j) = self.spring_j.probe(g[1]);
            let r = [mb[0] - ms_i, mb[1] - ms_j];
            let scale = mb[0]
                .abs()
                .max(mb[1].abs())
                .max(ms_i.abs())
                .max(ms_j.abs())
                .max(1.0);
            if r[0].abs().max(r[1].abs()) < 1e-9 * scale {
                break;
            }
            let j00 = k_flex.get(er[0], er[0]) + kt_i;
            let j01 = k_flex.get(er[0], er[1]);
            let j10 = k_flex.get(er[1], er[0]);
            let j11 = k_flex.get(er[1], er[1]) + kt_j;
            let det = j00 * j11 - j01 * j10;
            if det.abs() < 1e-30 {
                break;
            }
            thb[0] -= (j11 * r[0] - j01 * r[1]) / det;
            thb[1] -= (-j10 * r[0] + j00 * r[1]) / det;
        }

        self.trial_thb_i = thb[0];
        self.trial_thb_j = thb[1];
        self.trial_rot_i = thn[0] - thb[0];
        self.trial_rot_j = thn[1] - thb[1];
        self.spring_i.trial(self.trial_rot_i);
        self.spring_j.trial(self.trial_rot_j);
    }
}

/// 材端曲げばねが作用する局所回転自由度（局所 DOF 5・11）。
const SPRING_ROT_DOFS: [usize; 2] = [5, 11];

fn condense_springs(k_elem: &LocalMat, k_i: f64, k_j: f64) -> LocalMat {
    let releases = [(SPRING_ROT_DOFS[0], k_i), (SPRING_ROT_DOFS[1], k_j)];
    crate::frame::prismatic::condense_end_releases(k_elem, &releases)
}

fn compute_kstar(
    elastic: &crate::frame::beam::BeamElement,
    k_flex: &LocalMat,
    kti: f64,
    ktj: f64,
) -> LocalMat {
    let k_end = condense_springs(k_flex, kti, ktj);
    let (li, lj) = elastic.rigid_lengths();
    elastic.apply_rigid_zone_transform(&k_end, li, lj)
}

impl ElementBehavior for ConcentratedSpringBeam {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        crate::behavior::node_global_dofs(&self.elastic.nodes, dof)
    }

    fn tangent_stiffness(&self, _ctx: &Ctx) -> LocalMat {
        let kti = self.spring_i.probe(self.trial_rot_i).1;
        let ktj = self.spring_j.probe(self.trial_rot_j).1;

        let k_local = match self.model {
            SpringModel::OneComponent => compute_kstar(&self.elastic, self.k_flex(), kti, ktj),
            SpringModel::TwoComponent => unimplemented!(
                "TwoComponent spring model is not yet implemented (P5 §3). Use OneComponent."
            ),
        };
        self.elastic.axis.to_global(&k_local)
    }

    fn internal_force(&self, _ctx: &Ctx) -> LocalVec {
        let k_flex = self.k_flex();
        let u_flex = self.u_flex_local();
        let er = SPRING_ROT_DOFS;
        let mut uh = u_flex;
        uh[er[0]] = self.trial_thb_i;
        uh[er[1]] = self.trial_thb_j;

        let mut f_flex = [0.0_f64; 12];
        for (i, f) in f_flex.iter_mut().enumerate() {
            let mut s = 0.0;
            for (j, &u) in uh.iter().enumerate() {
                s += k_flex.get(i, j) * u;
            }
            *f = s;
        }
        let ms_i = self.spring_i.probe(self.trial_rot_i).0;
        let ms_j = self.spring_j.probe(self.trial_rot_j).0;
        f_flex[er[0]] = ms_i;
        f_flex[er[1]] = ms_j;

        let (li, lj) = self.elastic.rigid_lengths();
        let f_node = crate::frame::rigid_arm::to_node_force(&f_flex, li, lj);
        let f_global = self.elastic.axis.rotate_to_global(&f_node);
        LocalVec {
            data: SmallVec::from_slice(&f_global),
        }
    }

    fn state_member_forces(&self, ctx: &Ctx) -> Option<crate::frame::beam::MemberForces> {
        let f_global = self.internal_force(ctx);
        let arr: [f64; 12] = std::array::from_fn(|i| f_global.data[i]);
        let f_local = self.elastic.axis.rotate_to_local(&arr);
        Some(crate::frame::beam::member_forces_from_end_forces(
            &f_local,
            self.elastic.length,
            &self.elastic.eval_sections,
        ))
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        let du_global: [f64; 12] = std::array::from_fn(|i| du.data[i]);
        let du_local = self.elastic.axis.rotate_to_local(&du_global);
        self.apply_mn_interaction(Some(&du_local));
        self.elastic.update_state(du, commit, _ctx);
        self.solve_internal_equilibrium();
        if commit {
            self.spring_i.commit();
            self.spring_j.commit();
            self.rot_i = self.trial_rot_i;
            self.rot_j = self.trial_rot_j;
            self.thb_i = self.trial_thb_i;
            self.thb_j = self.trial_thb_j;
        }
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        self.elastic.mass_matrix(opt)
    }

    fn geometric_stiffness(&self, n: f64) -> LocalMat {
        self.elastic.geometric_stiffness(n)
    }

    fn snapshot_state(&self) -> Box<dyn Any> {
        let materials: Vec<Box<dyn UniaxialMaterial>> =
            vec![self.spring_i.clone_box(), self.spring_j.clone_box()];
        Box::new((
            materials,
            [self.rot_i, self.rot_j, self.trial_rot_i, self.trial_rot_j],
            [self.thb_i, self.thb_j, self.trial_thb_i, self.trial_thb_j],
            self.elastic.committed_disp,
            self.elastic.trial_disp,
        ))
    }

    fn restore_state(&mut self, state: &dyn Any) {
        type Snapshot = (
            Vec<Box<dyn UniaxialMaterial>>,
            [f64; 4],
            [f64; 4],
            [f64; 12],
            [f64; 12],
        );
        let snapshot =
            crate::behavior::downcast_snapshot::<Snapshot>("ConcentratedSpringBeam", state);
        if snapshot.0.len() == 2 {
            self.spring_i = snapshot.0[0].clone_box();
            self.spring_j = snapshot.0[1].clone_box();
        }
        [self.rot_i, self.rot_j, self.trial_rot_i, self.trial_rot_j] = snapshot.1;
        [self.thb_i, self.thb_j, self.trial_thb_i, self.trial_thb_j] = snapshot.2;
        self.elastic.committed_disp = snapshot.3;
        self.elastic.trial_disp = snapshot.4;
    }

    fn commit_state(&mut self) {
        self.elastic.commit_state();
        self.spring_i.commit();
        self.spring_j.commit();
        self.rot_i = self.trial_rot_i;
        self.rot_j = self.trial_rot_j;
        self.thb_i = self.trial_thb_i;
        self.thb_j = self.trial_thb_j;
    }

    fn revert_state(&mut self) {
        self.elastic.revert_state();
        self.spring_i.revert();
        self.spring_j.revert();
        self.trial_rot_i = self.rot_i;
        self.trial_rot_j = self.rot_j;
        self.trial_thb_i = self.thb_i;
        self.trial_thb_j = self.thb_j;
    }

    fn serialize_checkpoint(&self) -> Vec<u8> {
        let cp = ConcentratedSpringCheckpoint {
            rot_i: self.rot_i,
            rot_j: self.rot_j,
            trial_rot_i: self.trial_rot_i,
            trial_rot_j: self.trial_rot_j,
            thb_i: self.thb_i,
            thb_j: self.thb_j,
            trial_thb_i: self.trial_thb_i,
            trial_thb_j: self.trial_thb_j,
            spring_i: self.spring_i.serialize_state(),
            spring_j: self.spring_j.serialize_state(),
            elastic_committed_disp: self.elastic.committed_disp,
            elastic_trial_disp: self.elastic.trial_disp,
        };
        bincode::serialize(&cp).expect("serialize checkpoint")
    }

    fn deserialize_checkpoint(
        &mut self,
        data: &[u8],
    ) -> Result<(), crate::behavior::CheckpointError> {
        let cp: ConcentratedSpringCheckpoint = bincode::deserialize(data)
            .map_err(|e| crate::behavior::CheckpointError::Decode(e.to_string()))?;
        self.rot_i = cp.rot_i;
        self.rot_j = cp.rot_j;
        self.trial_rot_i = cp.trial_rot_i;
        self.trial_rot_j = cp.trial_rot_j;
        self.thb_i = cp.thb_i;
        self.thb_j = cp.thb_j;
        self.trial_thb_i = cp.trial_thb_i;
        self.trial_thb_j = cp.trial_thb_j;
        self.spring_i.deserialize_state(&cp.spring_i)?;
        self.spring_j.deserialize_state(&cp.spring_j)?;
        self.elastic.committed_disp = cp.elastic_committed_disp;
        self.elastic.trial_disp = cp.elastic_trial_disp;
        Ok(())
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
struct ConcentratedSpringCheckpoint {
    rot_i: f64,
    rot_j: f64,
    trial_rot_i: f64,
    trial_rot_j: f64,
    thb_i: f64,
    thb_j: f64,
    trial_thb_i: f64,
    trial_thb_j: f64,
    spring_i: Vec<u8>,
    spring_j: Vec<u8>,
    elastic_committed_disp: [f64; 12],
    elastic_trial_disp: [f64; 12],
}

#[cfg(test)]
mod tests;
