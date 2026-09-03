//! # マクスウェル要素
//! バネ剛性 `Kd` と粘性ダッシュポット（力 `Fc = C0·sign(V)·|V|^α`）を直列に接続した
//! 2 節点軸方向要素。連結点変位 `Ud` を挟み、要素力 = バネ力 `Fk = Kd(Uij − Ud)` が
//! ダッシュポット力と釣り合う（`Fk = Fc`）。時刻歴では後退 Euler で `Ud` を毎ステップ
//! 更新し、`V = (Ud − Ud_前) / Δt` として釣合いを解く。
//!
//! - 線形（α=1、リリーフなし）: `Ud = (C0·Ud_前 + Δt·Kd·Uij) / (C0 + Δt·Kd)`（閉形式）。
//! - それ以外（α≠1、またはリリーフ有効）: 上式を初期値としてスカラー Newton 法で
//!   `Ud` を求める。
//!
//! ## リリーフ特性（オイルダンパー、`relief_velocity`/`c2_ratio`）
//! `DamperProps::relief_velocity`（リリーフ速度 Vr）が指定されると、実オイル
//! ダンパーのバイパス弁による頭打ち特性を、ダッシュポット力則の折れ線近似で
//! 表現する:
//!
//! - `|V| ≤ Vr`: 従来どおり `Fc = C0·sign(V)·|V|^α`。
//! - `|V| > Vr`: `Fc = sign(V)·(C0·Vr^α + C2·(|V|−Vr))`。
//!   `C2 = c2_ratio·C1`、`C1 = C0·α·Vr^(α−1)`（リリーフ点 Vr における
//!   リリーフ前の接線減衰係数）。`c2_ratio`（既定なら 0 扱い＝完全頭打ち、
//!   1 に近いほどリリーフによる頭打ちが弱い）が「リリーフ後の減衰係数比 C2/C1」
//!   を表す。この定義により、リリーフ点 Vr で力・勾配ともに連続な折れ線
//!   （力は C1 一致で連続、勾配は C1→C2 に不連続に低下）となる。
//!
//! `relief_velocity` が `None` の場合は従来どおり全域で `Fc = C0·sign(V)·|V|^α`。
//!
//! 減衰要素の要素力は節点力として運動方程式へ与えられる（構造動力学）。本実装は
//! 収束用に整合接線 `∂Fk/∂Uij` を接線剛性へ与えるが、収束解は要素力の釣合いに
//! 一致するため結果は原典と等価。`Δt<=0`（静的・線形解析）では不活性（力・剛性 0）。

use crate::behavior::{Ctx, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::transform::LocalFrame;
use smallvec::SmallVec;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, Model};
use std::any::Any;

/// マクスウェルダンパー要素（2 節点・軸方向）。
#[derive(Clone)]
pub struct MaxwellDamperElement {
    pub nodes: [NodeId; 2],
    pub axis: LocalFrame,
    /// バネ剛性 Kd [N/mm]。
    pub kd: f64,
    /// 粘性係数 C0 [N·(s/mm)^α]。
    pub c0: f64,
    /// 速度指数 α。
    pub alpha: f64,
    /// リリーフ速度 Vr [mm/s]（`None` はリリーフなし）。
    pub relief_velocity: Option<f64>,
    /// リリーフ後の減衰係数比 C2/C1（`relief_velocity` が `None` の場合は未使用）。
    pub c2_ratio: Option<f64>,
    /// 時間刻み Δt [s]（0 以下で不活性）。
    dt: f64,
    /// 確定軸伸び Uij [mm]（引張正）。
    committed_elong: f64,
    /// 試行軸伸び Uij [mm]。
    trial_elong: f64,
    /// 確定連結点変位 Ud [mm]。
    committed_ud: f64,
    /// commit 直前（`committed_ud` 更新前）に評価した軸力 N [N]（引張正）の
    /// キャッシュ。`state_member_forces` が commit 直後（Newton 反復に入る前、
    /// `trial_elong == committed_elong`）に呼ばれた場合はこの値を返す
    /// （中-2: commit 後に `axial_force` を再評価すると、`solve_ud` が
    /// 既に更新済みの `committed_ud` を初期値に使うため、後退 Euler の
    /// ダッシュポットが追加で緩和し軸力が過小評価される）。
    committed_force: f64,
}

impl MaxwellDamperElement {
    pub fn new(data: &ElementData, model: &Model) -> Self {
        let geom = crate::transform::EndGeometry::of_element(data, model);
        let [n0, n1] = geom.nodes;
        let axis = geom.local_frame(data.local_axis.ref_vector);
        let props = model.damper_props(data.id).unwrap_or_default();
        Self {
            nodes: [n0, n1],
            axis,
            kd: props.kd.max(0.0),
            c0: props.c0.max(0.0),
            alpha: if props.alpha > 0.0 { props.alpha } else { 1.0 },
            relief_velocity: props.relief_velocity.filter(|&vr| vr > 0.0),
            c2_ratio: props.c2_ratio,
            dt: 0.0,
            committed_elong: 0.0,
            trial_elong: 0.0,
            committed_ud: 0.0,
            committed_force: 0.0,
        }
    }

    /// ダッシュポット力則 `Fc(V)` と、その速度に関する接線 `dFc/dV` を返す。
    ///
    /// `relief_velocity=None`（またはリリーフ速度以下）では従来どおり
    /// `Fc=C0·sign(V)·|V|^α`、`dFc/dV=C0·α·|V|^(α−1)`。リリーフ有効時に
    /// `|V|` がリリーフ速度 Vr を超えると、モジュール docs の折れ線式
    /// （`Fc=sign(V)·(C0·Vr^α+C2·(|V|−Vr))`、`dFc/dV=C2`）に切り替わる。
    /// `|V|` が 0 近傍かつ α<1 のときの特異点回避のため `|V|` は `1e-12` を下限に
    /// クランプする（既存の `axial_tangent` と同じ安全策）。
    fn dashpot(&self, v: f64) -> (f64, f64) {
        let av = v.abs().max(1e-12);
        match self.relief_velocity {
            Some(vr) if av > vr => {
                // リリーフ点 Vr における「リリーフ前」の接線減衰係数 C1。
                let c1 = self.c0 * self.alpha * vr.powf(self.alpha - 1.0);
                let c2 = self.c2_ratio.unwrap_or(0.0).max(0.0) * c1;
                let fc_at_vr = self.c0 * vr.powf(self.alpha);
                let fc = v.signum() * (fc_at_vr + c2 * (av - vr));
                (fc, c2)
            }
            _ => {
                let fc = self.c0 * v.signum() * av.powf(self.alpha);
                let dfc = self.c0 * self.alpha * av.powf(self.alpha - 1.0);
                (fc, dfc)
            }
        }
    }

    /// 与えた軸伸び `elong` に対する連結点変位 `Ud` を後退 Euler で解く。
    /// `Δt<=0` では `Ud=elong`（バネ力 0 = 不活性）。
    fn solve_ud(&self, elong: f64) -> f64 {
        if self.dt <= 0.0 || self.kd <= 0.0 {
            return elong;
        }
        let ud0 = self.committed_ud;
        // 線形（α=1）閉形式を初期値に（リリーフ有効時も Newton 反復の初期値としては
        // そのまま使える。反復自体はリリーフの折れ線を正しく解く）。
        let mut ud = (self.c0 * ud0 + self.dt * self.kd * elong) / (self.c0 + self.dt * self.kd);
        // リリーフなし・α=1（線形）は閉形式が厳密解のため反復不要。
        let linear_exact = self.relief_velocity.is_none() && (self.alpha - 1.0).abs() < 1e-9;
        if linear_exact || self.c0 <= 0.0 {
            return ud;
        }
        // Newton 法: g(Ud) = Kd(elong−Ud) − Fc(V) = 0、V=(Ud−ud0)/Δt。
        for _ in 0..30 {
            let v = (ud - ud0) / self.dt;
            let (fc, dfc_dv) = self.dashpot(v);
            let g = self.kd * (elong - ud) - fc;
            // g'(Ud) = −Kd − (dFc/dV)/Δt
            let gp = -self.kd - dfc_dv / self.dt;
            if gp.abs() < 1e-30 {
                break;
            }
            let step = g / gp;
            ud -= step;
            if step.abs() < 1e-12 * (1.0 + ud.abs()) {
                break;
            }
        }
        ud
    }

    /// 現在の軸力 N [N]（引張正）= Kd(Uij − Ud)。`Δt<=0` は 0（不活性）。
    fn axial_force(&self, elong: f64) -> f64 {
        if self.dt <= 0.0 {
            return 0.0;
        }
        self.kd * (elong - self.solve_ud(elong))
    }

    /// 整合接線軸剛性 K_eff = Kd·C'/(Δt·Kd + C')、C'=dFc/dV（現在速度で評価。
    /// リリーフ無効時は `C0·α·|V|^(α−1)`、リリーフ有効かつ |V|>Vr のときは `C2`）。
    /// `Δt<=0` は 0（不活性）。
    fn axial_tangent(&self) -> f64 {
        if self.dt <= 0.0 || self.kd <= 0.0 {
            return 0.0;
        }
        let ud = self.solve_ud(self.trial_elong);
        let v = (ud - self.committed_ud) / self.dt;
        let (_, c_prime) = self.dashpot(v);
        let denom = self.dt * self.kd + c_prime;
        if denom <= 0.0 {
            0.0
        } else {
            self.kd * c_prime / denom
        }
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

impl ElementBehavior for MaxwellDamperElement {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        crate::behavior::node_global_dofs(&self.nodes, dof)
    }

    fn tangent_stiffness(&self, _ctx: &Ctx) -> LocalMat {
        self.axis
            .to_global(&self.local_stiffness(self.axial_tangent()))
    }

    fn internal_force(&self, _ctx: &Ctx) -> LocalVec {
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        let n = self.axial_force(self.trial_elong);
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
        if commit {
            let elong = self.committed_elong + delong;
            // committed_ud を更新する前（＝現在の後退 Euler 履歴に基づく）軸力を
            // キャッシュする（中-2: 更新後に axial_force を再評価すると
            // ダッシュポットが余分に緩和し過小評価になるため）。
            self.committed_force = self.axial_force(elong);
            self.committed_ud = self.solve_ud(elong);
            self.committed_elong = elong;
            self.trial_elong = elong;
        } else {
            self.trial_elong = self.committed_elong + delong;
        }
    }

    fn mass_matrix(&self, _opt: MassOption) -> LocalMat {
        // ダンパー要素は構造質量を持たない（自重はダンパー諸元・荷重側で扱う）。
        LocalMat::zeros(12)
    }

    fn state_member_forces(&self, _ctx: &Ctx) -> Option<crate::frame::beam::MemberForces> {
        // 現在状態の軸力（引張正）を両評価点一定で返す。時刻歴・増分解析の
        // 部材内力記録（N-δ 履歴ループ表示など）に用いる。
        //
        // commit 直後（trial_elong == committed_elong）は、`commit_state`/
        // `update_state(commit=true)` が更新前の履歴で評価しキャッシュした
        // `committed_force` を返す（中-2）。`axial_force(trial_elong)` を
        // 再評価すると、`solve_ud` の初期値 `committed_ud` が既にこの elong に
        // 対する解へ更新済みのため V≈0 となり、ダッシュポット力が過小評価される。
        // Newton 反復中（trial_elong != committed_elong）は従来どおり
        // 現在の試行伸びで再評価する。
        let n = if self.trial_elong == self.committed_elong {
            self.committed_force
        } else {
            self.axial_force(self.trial_elong)
        };
        let v = [n, 0.0, 0.0, 0.0, 0.0, 0.0];
        Some(crate::frame::beam::MemberForces {
            at: vec![(0.0, v), (1.0, v)],
        })
    }

    fn commit_state(&mut self) {
        // committed_ud を更新する前（＝現在の後退 Euler 履歴に基づく）軸力を
        // キャッシュする（中-2、`update_state(commit=true)` と同じ理由）。
        self.committed_force = self.axial_force(self.trial_elong);
        self.committed_ud = self.solve_ud(self.trial_elong);
        self.committed_elong = self.trial_elong;
    }

    fn revert_state(&mut self) {
        self.trial_elong = self.committed_elong;
    }

    fn set_time_step(&mut self, dt: f64) {
        self.dt = dt;
    }

    fn snapshot_state(&self) -> Box<dyn Any> {
        Box::new((
            self.committed_elong,
            self.committed_ud,
            self.trial_elong,
            self.committed_force,
        ))
    }

    fn restore_state(&mut self, state: &dyn Any) {
        let &(ce, cud, te, cf) = crate::behavior::downcast_snapshot::<(f64, f64, f64, f64)>(
            "MaxwellDamperElement",
            state,
        );
        self.committed_elong = ce;
        self.committed_ud = cud;
        self.trial_elong = te;
        self.committed_force = cf;
    }

    fn serialize_checkpoint(&self) -> Vec<u8> {
        // 後退 Euler の履歴状態（committed_elong・committed_ud）と、
        // commit 直後の軸力キャッシュ（committed_force、中-2）をチェックポイントへ
        // 含める。これらを欠くとレジューム時にダッシュポットの緩和状態が失われ、
        // 直後の軸力が不整合になる（`springs/spring.rs` と同じ考え方）。
        bincode::serialize(&(
            self.committed_elong,
            self.committed_ud,
            self.trial_elong,
            self.committed_force,
        ))
        .expect("serialize checkpoint")
    }

    fn deserialize_checkpoint(
        &mut self,
        data: &[u8],
    ) -> Result<(), crate::behavior::CheckpointError> {
        // 旧チェックポイント（本状態未収録・空バイト列）は「状態なし」として許容する。
        if data.is_empty() {
            return Ok(());
        }
        let (ce, cud, te, cf): (f64, f64, f64, f64) = bincode::deserialize(data)
            .map_err(|e| crate::behavior::CheckpointError::Decode(e.to_string()))?;
        self.committed_elong = ce;
        self.committed_ud = cud;
        self.trial_elong = te;
        self.committed_force = cf;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn damper(kd: f64, c0: f64, alpha: f64, dt: f64) -> MaxwellDamperElement {
        MaxwellDamperElement {
            nodes: [NodeId(0), NodeId(1)],
            axis: LocalFrame::from_nodes([0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
            kd,
            c0,
            alpha,
            relief_velocity: None,
            c2_ratio: None,
            dt,
            committed_elong: 0.0,
            trial_elong: 0.0,
            committed_ud: 0.0,
            committed_force: 0.0,
        }
    }

    #[test]
    fn test_maxwell_inert_when_dt_zero() {
        // Δt<=0（静的・線形）は不活性（力・剛性 0）。
        let d = damper(100.0, 1000.0, 1.0, 0.0);
        assert_eq!(d.axial_force(5.0), 0.0);
        assert_eq!(d.axial_tangent(), 0.0);
    }

    #[test]
    fn test_maxwell_state_member_forces_matches_axial_force() {
        // 部材内力記録（state_member_forces）は現在状態の軸力（引張正）を
        // 両評価点一定で返す。
        let mut d = damper(100.0, 1000.0, 1.0, 0.01);
        d.trial_elong = 1.0;
        let n = d.axial_force(1.0);
        assert!(n > 0.0);
        let mf = d
            .state_member_forces(&Ctx {
                model: &squid_n_core::model::Model::default(),
            })
            .expect("マクスウェルダンパーは状態から内力を返す");
        assert_eq!(mf.at.len(), 2);
        for (_, v) in &mf.at {
            assert!((v[0] - n).abs() < 1e-12);
            assert_eq!(v[5], 0.0);
        }
    }

    /// 中-2: commit 直後（`trial_elong == committed_elong`）の `state_member_forces`
    /// が「commit 直前（committed_ud 更新前）の `axial_force`」と一致すること。
    /// 過渡状態（ダッシュポットが定常に達していない各ステップ）では、
    /// commit 後に素朴に `axial_force` を再評価した値（`solve_ud` の初期値
    /// `committed_ud` が既に今回の解へ更新済み）とは明確に異なることも確認する。
    #[test]
    fn test_maxwell_state_member_forces_after_commit_matches_pre_commit_axial_force() {
        let mut d = damper(100.0, 1000.0, 1.0, 0.01);
        let mut du = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        du.data[6] = 0.3; // 各ステップ 0.3mm ずつ伸ばす（過渡状態を維持）。
        let model = Model::default();
        let ctx = Ctx { model: &model };

        for _ in 0..4 {
            let pre = d.clone(); // commit 直前（committed_ud 更新前）の状態。
            d.update_state(&du, true, &ctx);
            let elong = d.committed_elong;
            // 「commit 直前の axial_force」＝更新前の committed_ud を初期値に
            // 評価した軸力（committed_force はこれをキャッシュしているはず）。
            let expected = pre.axial_force(elong);
            assert!(
                (d.committed_force - expected).abs() < 1e-9 * expected.abs().max(1.0),
                "committed_force should match pre-commit axial_force: {} vs {}",
                d.committed_force,
                expected
            );

            // state_member_forces（commit 直後、trial_elong==committed_elong）は
            // キャッシュ値 committed_force を返す。
            let mf = d
                .state_member_forces(&ctx)
                .expect("マクスウェルダンパーは状態から内力を返す");
            assert!((mf.at[0].1[0] - d.committed_force).abs() < 1e-12);

            // 修正前の（バグのある）計算方法＝ commit 後に committed_ud 更新済みの
            // 状態で axial_force を再評価した値とは、（この過渡状態では）明確に
            // 異なる値になる（浮動小数点誤差では説明できない差）。
            let naive_post_commit = d.axial_force(elong);
            assert!(
                (d.committed_force - naive_post_commit).abs() > 1e-6 * expected.abs().max(1.0),
                "過渡状態では commit 直後の素朴な再評価はキャッシュ値と異なるはず: \
                 cached={}, naive={}",
                d.committed_force,
                naive_post_commit
            );
        }
    }

    #[test]
    fn test_maxwell_locks_at_fast_step_then_relaxes() {
        // 速い載荷（1 ステップ）ではダッシュポットがほぼロックしバネ力 ≈ Kd·Uij。
        // 変形一定で時間が進むとダッシュポットが緩和し力 → 0。
        let mut d = damper(100.0, 1000.0, 1.0, 0.01);
        d.trial_elong = 1.0;
        let f0 = d.axial_force(1.0);
        assert!(
            f0 > 90.0,
            "fast step should be near-locked (Kd·Uij), got {f0}"
        );
        // 変形を 1.0 に保ったまま多数ステップ commit（緩和）。
        for _ in 0..5000 {
            d.commit_state();
        }
        let f1 = d.axial_force(1.0);
        assert!(
            f1 < f0 * 0.1,
            "dashpot should relax force toward 0: f0={f0}, f1={f1}"
        );
    }

    #[test]
    fn test_maxwell_tangent_matches_finite_difference() {
        // 整合接線が有限差分と一致（α=1）。K_eff = Kd·C0/(C0+Δt·Kd)。
        let mut d = damper(100.0, 1000.0, 1.0, 0.01);
        d.trial_elong = 0.5;
        let kt = d.axial_tangent();
        let h = 1e-6;
        let f1 = d.axial_force(0.5 + h);
        let f2 = d.axial_force(0.5 - h);
        let fd = (f1 - f2) / (2.0 * h);
        assert!((kt - fd).abs() < 1e-3 * kt.max(1.0), "kt={kt}, fd={fd}");
        let expect = 100.0 * 1000.0 / (1000.0 + 0.01 * 100.0);
        assert!((kt - expect).abs() < 1e-6, "kt={kt}, expect={expect}");
    }

    #[test]
    fn test_maxwell_nonlinear_alpha_solves() {
        // α≠1（非線形粘性）でも Ud が釣合い（Kd(elong−Ud)=Fc(V)）を満たす。
        let mut d = damper(100.0, 500.0, 0.5, 0.02);
        d.committed_ud = 0.0;
        let elong = 2.0;
        let ud = d.solve_ud(elong);
        let v = (ud - 0.0) / d.dt;
        let fc = d.c0 * v.signum() * v.abs().powf(d.alpha);
        let fk = d.kd * (elong - ud);
        assert!(
            (fk - fc).abs() < 1e-6 * fk.abs().max(1.0),
            "fk={fk}, fc={fc}"
        );
    }

    #[test]
    fn test_maxwell_element_drives_axial_force() {
        // update_state で節点変位を与え、internal_force が軸方向へ力を返す。
        let mut d = damper(100.0, 1000.0, 1.0, 0.01);
        // 節点1 の ux に 1.0mm（軸方向 = グローバル X）。
        let mut du = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        du.data[6] = 1.0;
        let model = Model::default();
        let ctx = Ctx { model: &model };
        d.update_state(&du, false, &ctx);
        assert!((d.trial_elong - 1.0).abs() < 1e-9);
        let f = d.internal_force(&ctx);
        let n = d.axial_force(1.0);
        assert!(n > 0.0);
        assert!((f.data[0] + n).abs() < 1e-9); // 節点0 ux = −N
        assert!((f.data[6] - n).abs() < 1e-9); // 節点1 ux = +N
    }

    /// リリーフ特性: リリーフ速度 Vr の前後でダッシュポット力則が連続であり、
    /// リリーフ後の勾配が `C2 = c2_ratio・C1`（`C1=C0・α・Vr^(α-1)`）に一致すること。
    #[test]
    fn test_relief_dashpot_continuous_and_slope_matches_c2() {
        let c0 = 1000.0;
        let alpha = 1.0;
        let vr = 50.0;
        let c2_ratio = 0.1;
        let mut d = damper(500.0, c0, alpha, 0.01);
        d.relief_velocity = Some(vr);
        d.c2_ratio = Some(c2_ratio);

        // リリーフ点の直前・直後で力が連続（跳びがない）こと。
        // （±eps の評価点そのものにも C0・eps オーダーの傾き分の差が乗るため、
        // 許容値は eps=1e-6・(C0+C2) 程度を見込んだ 1e-2 とする。）
        let (fc_before, _) = d.dashpot(vr - 1e-6);
        let (fc_after, _) = d.dashpot(vr + 1e-6);
        assert!(
            (fc_before - fc_after).abs() < 1e-2,
            "force should be continuous at Vr: before={fc_before}, after={fc_after}"
        );
        // リリーフ点そのものでの値（sign(v)・C0・Vr^α）とも一致すること。
        let (fc_at, _) = d.dashpot(vr);
        let expected_at_vr = c0 * vr.powf(alpha);
        assert!(
            (fc_at - expected_at_vr).abs() < 1e-6,
            "fc_at={fc_at}, expected={expected_at_vr}"
        );

        // リリーフ前（|V|<Vr）の勾配は従来どおり C1=C0・α・Vr^(α-1)（α=1 なら C0）。
        let c1 = c0 * alpha * vr.powf(alpha - 1.0);
        let (_, dfc_pre) = d.dashpot(vr * 0.5);
        assert!((dfc_pre - c1).abs() < 1e-9, "dfc_pre={dfc_pre}, C1={c1}");

        // リリーフ後（|V|>Vr）の勾配は C2=c2_ratio・C1 に一致すること。
        let expected_c2 = c2_ratio * c1;
        let (_, dfc_post) = d.dashpot(vr * 10.0);
        assert!(
            (dfc_post - expected_c2).abs() < 1e-9,
            "dfc_post={dfc_post}, expected C2={expected_c2}"
        );

        // 負方向（V<0）でも符号反転した同じ折れ線であること。
        let (fc_neg, dfc_neg) = d.dashpot(-vr * 10.0);
        assert!(fc_neg < 0.0, "fc_neg should be negative: {fc_neg}");
        assert!(
            (dfc_neg - expected_c2).abs() < 1e-9,
            "勾配の大きさは符号によらず C2: dfc_neg={dfc_neg}"
        );
    }

    /// リリーフ有効時、Vr 以下の速度域では従来（リリーフなし）と完全に一致すること
    /// （既存挙動を壊さない回帰確認）。
    #[test]
    fn test_relief_matches_baseline_below_relief_velocity() {
        let mut base = damper(500.0, 1000.0, 1.0, 0.01);
        let mut relief = damper(500.0, 1000.0, 1.0, 0.01);
        relief.relief_velocity = Some(1000.0); // 本試験の速度域では届かない大きめの Vr
        relief.c2_ratio = Some(0.1);

        let elong = 0.05; // 小変位（低速度、Vr 未満）
        let f_base = base.axial_force(elong);
        let f_relief = relief.axial_force(elong);
        assert!(
            (f_base - f_relief).abs() < 1e-9 * f_base.abs().max(1.0),
            "below Vr, relief should not change the force: base={f_base}, relief={f_relief}"
        );
        let kt_base = {
            base.trial_elong = elong;
            base.axial_tangent()
        };
        let kt_relief = {
            relief.trial_elong = elong;
            relief.axial_tangent()
        };
        assert!(
            (kt_base - kt_relief).abs() < 1e-9 * kt_base.abs().max(1.0),
            "below Vr, tangent should match: base={kt_base}, relief={kt_relief}"
        );
    }

    /// リリーフ有効時、高速域（|V|≫Vr）ではリリーフなしより力の伸びが緩やかになる
    /// （c2_ratio<1 のため、頭打ち特性が効いていることの端到端（要素レベル）確認）。
    #[test]
    fn test_relief_caps_force_growth_at_high_velocity() {
        let kd = 1.0e7; // 十分剛なバネ（Ud がほぼ elong に追従し、V が明確に Vr を超える）
        let c0 = 1000.0;
        let alpha = 1.0;
        let dt = 0.001;
        let vr = 10.0;
        let c2_ratio = 0.05;

        let no_relief = damper(kd, c0, alpha, dt);
        let mut with_relief = damper(kd, c0, alpha, dt);
        with_relief.relief_velocity = Some(vr);
        with_relief.c2_ratio = Some(c2_ratio);

        // 1 ステップで大変位を与え、V≈elong/dt≫vr となる高速載荷を作る。
        let elong = 1.0;
        let f_no_relief = no_relief.axial_force(elong);
        let f_with_relief = with_relief.axial_force(elong);

        assert!(
            f_with_relief > 0.0 && f_with_relief < f_no_relief,
            "relief should cap force growth at high velocity: \
             no_relief={f_no_relief}, with_relief={f_with_relief}"
        );
    }
}
