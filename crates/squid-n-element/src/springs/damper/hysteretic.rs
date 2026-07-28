//! 履歴型（弾塑性バイリニア）ダンパー要素の要素本体（鋼材系ダンパー）。
//!
//! 弾塑性バイリニア軸ばねの `ElementBehavior` 実装。変位依存のため静的・動的
//! いずれの解析でも作用する（`dt` 不要）。

use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::transform::LocalFrame;
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, Model};
use std::any::Any;

/// 履歴型（弾塑性バイリニア）ダンパー要素（2 節点・軸方向）。
/// 鋼材系ダンパー（SUB／アンボンドブレース／二重鋼管座屈補剛ブレース／鉛／U 型等、
/// 制振部材の標準的な弾塑性バイリニア）。初期軸剛性 `k1`・降伏軸力 `qy`・第2剛性 `k2`
/// の弾塑性軸ばね。変位依存のため静的・動的いずれの解析でも作用する（`dt` 不要）。
#[derive(Clone)]
pub struct HystereticDamperElement {
    pub nodes: [NodeId; 2],
    pub axis: LocalFrame,
    /// 軸力–伸び関係の弾塑性材料（Bilinear を力–変位として流用）。
    mat: squid_n_material::Bilinear,
    committed_elong: f64,
    trial_elong: f64,
}

impl HystereticDamperElement {
    pub fn new(data: &ElementData, model: &Model) -> Self {
        let n0 = data.nodes[0];
        let n1 = data.nodes[1];
        let p0 = model
            .nodes
            .get(n0.index())
            .map(|n| n.coord)
            .unwrap_or([0.0; 3]);
        let p1 = model
            .nodes
            .get(n1.index())
            .map(|n| n.coord)
            .unwrap_or([0.0; 3]);
        let axis = LocalFrame::from_nodes(p0, p1, data.local_axis.ref_vector);
        let props = model.damper_props(data.id).unwrap_or_default();
        let k1 = props.kd.max(1e-9);
        let qy = props.qy.max(1e-9);
        let hardening = props.k2_ratio.clamp(0.0, 0.999);
        Self {
            nodes: [n0, n1],
            axis,
            // Bilinear(e=k1, fy=qy, hardening=k2/k1) を軸力–伸びの弾塑性則として用いる。
            mat: squid_n_material::Bilinear::new(k1, qy, hardening),
            committed_elong: 0.0,
            trial_elong: 0.0,
        }
    }

    fn axial(&mut self, elong: f64) -> (f64, f64) {
        // Bilinear::trial は (応力, 接線) を返すが、ここでは (軸力, 軸剛性) に相当。
        use squid_n_material::UniaxialMaterial;
        self.mat.trial(elong)
    }

    fn local_stiffness(&self, ka: f64) -> LocalMat {
        let mut k = LocalMat::zeros(12);
        k.set(0, 0, ka);
        k.set(6, 6, ka);
        k.set(0, 6, -ka);
        k.set(6, 0, -ka);
        k
    }
}

impl ElementBehavior for HystereticDamperElement {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &nid in &self.nodes {
            let ni = nid.index();
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                gdofs.push(dof.active(g).map(|a| a as usize).unwrap_or(usize::MAX));
            }
        }
        gdofs
    }

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        // trial を汚さないよう複製して接線のみ取得。
        use squid_n_material::UniaxialMaterial;
        let mut m = self.mat.clone();
        let (_f, k) = m.trial(self.trial_elong);
        self.axis.to_global(&self.local_stiffness(k))
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        use squid_n_material::UniaxialMaterial;
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        let mut m = self.mat.clone();
        let (n, _k) = m.trial(self.trial_elong);
        let t = self.axis.rot[0];
        for k in 0..3 {
            f.data[k] = -n * t[k];
            f.data[6 + k] = n * t[k];
        }
        f
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        let du_global: [f64; 12] = std::array::from_fn(|i| du.data[i]);
        let du_local = self.axis.rotate_to_local(&du_global);
        let delong = du_local[6] - du_local[0];
        self.trial_elong = self.committed_elong + delong;
        let _ = self.axial(self.trial_elong);
        if commit {
            use squid_n_material::UniaxialMaterial;
            self.mat.commit();
            self.committed_elong = self.trial_elong;
        }
    }

    fn mass_matrix(&self, _opt: MassOption) -> LocalMat {
        LocalMat::zeros(12)
    }

    fn state_member_forces(
        &self,
        _state: &ElemState,
        _ctx: &Ctx,
    ) -> Option<crate::beam::MemberForces> {
        use squid_n_material::UniaxialMaterial;
        // trial を汚さないよう複製し、現在状態の復元力（引張正）を取り出す。
        let mut m = self.mat.clone();
        let (n, _k) = m.trial(self.trial_elong);
        let v = [n, 0.0, 0.0, 0.0, 0.0, 0.0];
        Some(crate::beam::MemberForces {
            at: vec![(0.0, v), (1.0, v)],
        })
    }

    fn commit_state(&mut self) {
        use squid_n_material::UniaxialMaterial;
        let _ = self.axial(self.trial_elong);
        self.mat.commit();
        self.committed_elong = self.trial_elong;
    }

    fn revert_state(&mut self) {
        use squid_n_material::UniaxialMaterial;
        self.mat.revert();
        self.trial_elong = self.committed_elong;
    }

    fn snapshot_state(&self) -> Box<dyn Any> {
        use squid_n_material::UniaxialMaterial;
        Box::new((
            self.committed_elong,
            self.trial_elong,
            self.mat.serialize_state(),
        ))
    }

    fn restore_state(&mut self, state: &dyn Any) {
        use squid_n_material::UniaxialMaterial;
        if let Some((ce, te, ms)) = state.downcast_ref::<(f64, f64, Vec<u8>)>() {
            self.committed_elong = *ce;
            self.trial_elong = *te;
            // 同一プロセス内で serialize_state した信頼済みバイト列のため、
            // 復元失敗は起こらない想定。トランザクション巻き戻しでの panic を
            // 避けるため失敗時は状態を据え置く。
            let _ = self.mat.deserialize_state(ms);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hyst_damper(k1: f64, qy: f64, k2ratio: f64) -> HystereticDamperElement {
        HystereticDamperElement {
            nodes: [NodeId(0), NodeId(1)],
            axis: LocalFrame::from_nodes([0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
            mat: squid_n_material::Bilinear::new(k1, qy, k2ratio),
            committed_elong: 0.0,
            trial_elong: 0.0,
        }
    }

    /// 節点1 の軸方向（グローバル X）へ伸び elong を与えて commit。
    fn drive(d: &mut HystereticDamperElement, elong: f64, model: &Model) {
        let prev = d.committed_elong;
        let mut du = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        du.data[6] = elong - prev;
        let ctx = Ctx { model };
        d.update_state(&du, true, &ctx);
    }

    #[test]
    fn test_hysteretic_bilinear_elastic_then_yield() {
        // 弾性域: N=k1·elong。降伏後: N≈qy+k2·(elong−δy)。
        let k1 = 1000.0;
        let qy = 100.0;
        let mut d = hyst_damper(k1, qy, 0.02);
        let model = Model::default();
        let dy = qy / k1; // 0.1
        drive(&mut d, 0.5 * dy, &model);
        let ctx = Ctx { model: &model };
        let f_el = d.internal_force(&ElemState {}, &ctx).data[6];
        assert!((f_el - k1 * 0.5 * dy).abs() < 1e-6, "elastic: {f_el}");
        // 降伏後（5δy）。
        drive(&mut d, 5.0 * dy, &model);
        let f_pl = d.internal_force(&ElemState {}, &ctx).data[6];
        let expect = qy + 0.02 * k1 * (5.0 * dy - dy);
        assert!(
            (f_pl - expect).abs() < 1.0,
            "plastic: {f_pl}, expect={expect}"
        );
    }

    #[test]
    fn test_hysteretic_active_in_static_no_dt() {
        // 履歴型は dt 不要（set_time_step を呼ばずとも力を発生＝静的で作用）。
        let mut d = hyst_damper(1000.0, 100.0, 0.02);
        let model = Model::default();
        let ctx = Ctx { model: &model };
        drive(&mut d, 0.05, &model);
        let n = d.internal_force(&ElemState {}, &ctx).data[6];
        assert!(n > 0.0, "hysteretic damper must be active in static: {n}");
        let kt = d.tangent_stiffness(&ElemState {}, &ctx).get(6, 6);
        assert!(kt > 0.0);
    }

    #[test]
    fn test_hysteretic_dissipates_energy() {
        // 1 サイクルの履歴ループ面積（散逸エネルギー）が正。
        let k1 = 1000.0;
        let qy = 100.0;
        let mut d = hyst_damper(k1, qy, 0.02);
        let model = Model::default();
        let ctx = Ctx { model: &model };
        let dy = qy / k1;
        let amp = 4.0 * dy;
        let mut energy = 0.0;
        let mut prev = (0.0, 0.0);
        for i in 0..=80 {
            let phase = i as f64 / 20.0 * std::f64::consts::PI;
            let elong = amp * phase.sin();
            drive(&mut d, elong, &model);
            let n = d.internal_force(&ElemState {}, &ctx).data[6];
            energy += 0.5 * (prev.1 + n) * (elong - prev.0);
            prev = (elong, n);
        }
        assert!(
            energy > 0.0,
            "hysteretic loop must dissipate energy: {energy}"
        );
    }
}
