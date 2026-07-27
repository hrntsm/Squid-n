use crate::beam::invert_small;
use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use squid_n_core::dof::{DofMap, DOF_PER_NODE};

use smallvec::SmallVec;
use squid_n_material::uniaxial::UniaxialMaterial;
use std::any::Any;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SpringModel {
    OneComponent,
    TwoComponent,
}

/// 端バネの N-M 相関パラメータ（2バネ連成の線形相関）。
/// 現在軸力 N に応じて回転バネの降伏モーメントを
/// M_lim = my0 · (1 − |N|/n_allow) で更新する（下限 0.02·my0）。
#[derive(Clone, Copy, Debug)]
pub struct MnInteraction {
    /// N=0 での降伏モーメント My0 [N·mm]
    pub my0: f64,
    /// 軸許容耐力 [N]（正値。引張・圧縮共通）
    pub n_allow: f64,
}

/// 材端集中ばね梁（one-component モデル）。
///
/// 節点回転 θn と可撓端回転 θb（内部自由度）を材端曲げばねが接続し、
/// ばね変形（相対回転）γ = θn − θb に履歴則を適用する。各トライアルで
/// 「可撓部端モーメント（`K_flex·û` の回転行）＝ばねモーメント M_s(γ)」の
/// 内部平衡を要素内 Newton（2 自由度）で解き、復元力はばねの履歴力と
/// 弾性可撓部の `K_flex·û` から経路整合に評価する。
///
/// 従来は (1) 復元力を「接線剛性 × 全変位」で評価し降伏後に履歴力と乖離する、
/// (2) ばね変形量に節点回転そのものを用いるため固定端（節点回転 0）で
/// 柱脚ヒンジが形成されず、回転する接合部ではモーメントと無関係に降伏扱いに
/// なる、という定式化上の欠陥があり、増分解析で剛性低下が生じなかった
/// （`dev_docs/v_and_v/` の該当項目参照）。
pub struct ConcentratedSpringBeam {
    pub elastic: crate::beam::BeamElement,
    pub spring_i: Box<dyn UniaxialMaterial>,
    pub spring_j: Box<dyn UniaxialMaterial>,
    pub model: SpringModel,
    /// N-M 相関（None = 従来どおり降伏モーメント一定）
    pub mn: Option<MnInteraction>,
    /// ばね変形（相対回転 γ = 節点回転 − 可撓端回転）の確定値。
    rot_i: f64,
    rot_j: f64,
    /// ばね変形のトライアル値（内部平衡の解）。
    trial_rot_i: f64,
    trial_rot_j: f64,
    /// 可撓端回転（内部自由度）の確定値。
    thb_i: f64,
    thb_j: f64,
    /// 可撓端回転のトライアル値。
    trial_thb_i: f64,
    trial_thb_j: f64,
}

impl ConcentratedSpringBeam {
    pub fn new(
        elastic: crate::beam::BeamElement,
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
        }
    }

    pub fn new_one_component(
        elastic: crate::beam::BeamElement,
        spring_i: Box<dyn UniaxialMaterial>,
        spring_j: Box<dyn UniaxialMaterial>,
    ) -> Self {
        Self::new(elastic, spring_i, spring_j, SpringModel::OneComponent)
    }

    /// N-M 相関を有効化する（ビルダー）。
    pub fn with_mn_interaction(mut self, my0: f64, n_allow: f64) -> Self {
        self.mn = Some(MnInteraction {
            my0,
            n_allow: n_allow.max(1.0),
        });
        self
    }

    /// 現在の軸力 [N]（引張正）。トライアル変位（＋任意の増分）から
    /// 弾性部の軸ひずみを取り出して評価する。Newton 反復中の累積修正量も
    /// 反映される（トライアル追従）。
    fn current_axial_force(&self, du_local: Option<&[f64; 12]>) -> f64 {
        let ul = self.elastic.axis.rotate_to_local(&self.elastic.trial_disp);
        let mut d = ul[6] - ul[0];
        if let Some(du) = du_local {
            d += du[6] - du[0];
        }
        self.elastic.e * self.elastic.a / self.elastic.length.max(1.0) * d
    }

    /// N-M 相関が有効なら、現在軸力に応じて両端バネの降伏モーメントを更新する。
    fn apply_mn_interaction(&mut self, du_local: Option<&[f64; 12]>) {
        let Some(mn) = self.mn else {
            return;
        };
        let n = self.current_axial_force(du_local);
        let m_lim = (mn.my0 * (1.0 - n.abs() / mn.n_allow)).max(0.02 * mn.my0);
        self.spring_i.set_yield(m_lim);
        self.spring_j.set_yield(m_lim);
    }

    /// 現在のトライアル節点変位を可撓端系の局所変位へ写す
    /// （グローバル→局所回転→剛域変換。回転成分は剛域で変わらない）。
    fn u_flex_local(&self) -> [f64; 12] {
        let u_local = self.elastic.axis.rotate_to_local(&self.elastic.trial_disp);
        let (li, lj) = self.elastic.rigid_lengths();
        crate::rigid_arm::to_flex_disp(&u_local, li, lj)
    }

    /// 内部平衡（可撓端回転 θb）を解き、トライアルばね変形・可撓端回転を更新する。
    ///
    /// 一成分系: 各端で「可撓部端モーメント（`K_flex·û` の回転行）＝ばねモーメント
    /// M_s(γ)、γ = θn − θb」を満たす θb を要素内 Newton で求める（2 自由度連成、
    /// 履歴則は区分線形のため通常数回で収束する）。ばねはトライアル状態
    /// （確定状態からの trial 評価）を保持したまま返す。
    fn solve_internal_equilibrium(&mut self) {
        let k_flex = self.elastic.local_stiffness_flex();
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
            let (ms_i, kt_i) = {
                let mut m = self.spring_i.clone_box();
                m.trial(g[0])
            };
            let (ms_j, kt_j) = {
                let mut m = self.spring_j.clone_box();
                m.trial(g[1])
            };
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
            // J = d r / d θb = [[K55+kt_i, K5,11], [K11,5, K11,11+kt_j]]
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

/// 材端曲げばねが作用する局所回転自由度（i 端, j 端）。
///
/// 要素局所系は「y 軸＝断面のせい方向」を規約とするため、強軸曲げ（せい方向の
/// 曲げ。梁の鉛直曲げ）は **z 軸まわり＝`rz`（局所 DOF 5・11）** に対応する
/// （`beam/construct.rs` の断面レイヤ→要素座標系クロス変換）。材端集中ばねは
/// 一軸曲げ（`ForceRegime::UniaxialBendingShear`）のモデルであり、骨格の降伏
/// モーメント（`flexural_yield_moment`＝強軸 Zp·σy）と初期剛性（6EI/l、
/// I は強軸）も強軸で与えるため、ばねは `rz` へ入れる。
const SPRING_ROT_DOFS: [usize; 2] = [5, 11];

fn condense_springs(k_elem: &LocalMat, k_i: f64, k_j: f64) -> LocalMat {
    let n = 14;
    let mut k = vec![0.0; n * n];

    // ばねを入れる回転自由度（i 端 → 内部 12、j 端 → 内部 13）だけを内部自由度へ移す。
    let map14 = |i: usize| -> usize {
        if i == SPRING_ROT_DOFS[0] {
            12
        } else if i == SPRING_ROT_DOFS[1] {
            13
        } else {
            i
        }
    };
    for i in 0..12 {
        for j in 0..12 {
            k[map14(i) * n + map14(j)] = k_elem.get(i, j);
        }
    }

    let ext_rot = SPRING_ROT_DOFS;
    let int_rot = [12usize, 13];
    for (idx, &er) in ext_rot.iter().enumerate() {
        let ir = int_rot[idx];
        let ks = if idx == 0 { k_i } else { k_j };
        k[er * n + er] += ks;
        k[ir * n + ir] += ks;
        k[er * n + ir] -= ks;
        k[ir * n + er] -= ks;
    }

    let na = 12;
    let nb = 2;
    let mut kaa = vec![0.0; na * na];
    let mut kab = vec![0.0; na * nb];
    let mut kba = vec![0.0; nb * na];
    let mut kbb = vec![0.0; nb * nb];

    for i in 0..na {
        for j in 0..na {
            kaa[i * na + j] = k[i * n + j];
        }
        for j in 0..nb {
            kab[i * nb + j] = k[i * n + (na + j)];
            kba[j * na + i] = k[(na + j) * n + i];
        }
    }
    for i in 0..nb {
        for j in 0..nb {
            kbb[i * nb + j] = k[(na + i) * n + (na + j)];
        }
    }

    let kbb_inv = invert_small(&kbb, nb);

    let mut kab_kbbinv = vec![0.0; na * nb];
    for i in 0..na {
        for j in 0..nb {
            let mut s = 0.0;
            for l in 0..nb {
                s += kab[i * nb + l] * kbb_inv[l * nb + j];
            }
            kab_kbbinv[i * nb + j] = s;
        }
    }

    let mut kstar = LocalMat::zeros(na);
    for i in 0..na {
        for j in 0..na {
            let mut s = kaa[i * na + j];
            for l in 0..nb {
                s -= kab_kbbinv[i * nb + l] * kba[l * na + j];
            }
            kstar.set(i, j, s);
        }
    }
    kstar
}

/// 材端曲げばねを直列接続した局所剛性（節点自由度 12×12）。
///
/// 組み立ての順序は弾性梁 [`crate::beam::BeamElement::local_stiffness`] と同じ土台を
/// 共有する:
///
/// 1. **可撓部**（剛域を除いた長さ `l − li − lj`）で生剛性を組み、端部条件
///    （ピン・半剛）を静縮約する（`local_stiffness_flex`）。
/// 2. 材端曲げばね（塑性ヒンジ）を強軸回転自由度へ直列に入れて静縮約する。
/// 3. 剛域変換で節点自由度へ移す。
///
/// 従来は 1. を省いて節点間全長の生剛性へ直接 2.・3. を掛けており、
/// (a) 剛域を持つ部材の可撓長が全長のままになる、(b) `end_cond` のピン・半剛が
/// 反映されず両端剛接として解かれる（剛性の過大評価）という食い違いがあった。
/// ばねは直列なので 1. の接合部ばねと 2. の塑性ばねの順序は結果に影響しない。
fn compute_kstar(elastic: &crate::beam::BeamElement, kti: f64, ktj: f64) -> LocalMat {
    let k_flex = elastic.local_stiffness_flex();
    let k_end = condense_springs(&k_flex, kti, ktj);
    // 剛域長は `local_stiffness_flex` と同じ規則で解決する（可撓長が残らない
    // 病的な入力は剛域なし扱い。`BeamElement::rigid_lengths`）。
    let (li, lj) = elastic.rigid_lengths();
    elastic.apply_rigid_zone_transform(&k_end, li, lj)
}

impl ElementBehavior for ConcentratedSpringBeam {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &nid in &self.elastic.nodes {
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

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        let kti = {
            let mut m = self.spring_i.clone_box();
            m.trial(self.trial_rot_i).1
        };
        let ktj = {
            let mut m = self.spring_j.clone_box();
            m.trial(self.trial_rot_j).1
        };

        let k_local = match self.model {
            SpringModel::OneComponent => compute_kstar(&self.elastic, kti, ktj),
            SpringModel::TwoComponent => unimplemented!(
                "TwoComponent spring model is not yet implemented (P5 §3). Use OneComponent."
            ),
        };
        // 静縮約済みローカル剛性をグローバル節点系へ回転
        self.elastic.axis.to_global(&k_local)
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        // 復元力は「弾性可撓部の K_flex·û（回転スロットは可撓端回転 θb）」と
        // 「ばねの履歴モーメント M_s(γ)」から経路整合に評価する（トライアル追従。
        // Newton 反復中の未確定変位も反映する）。節点の回転自由度にはばねを介して
        // モーメントが伝わるため、回転スロットはばね側の履歴力で置き換える
        // （内部平衡の解では両者は一致する）。
        let k_flex = self.elastic.local_stiffness_flex();
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
        let ms_i = {
            let mut m = self.spring_i.clone_box();
            m.trial(self.trial_rot_i).0
        };
        let ms_j = {
            let mut m = self.spring_j.clone_box();
            m.trial(self.trial_rot_j).0
        };
        f_flex[er[0]] = ms_i;
        f_flex[er[1]] = ms_j;

        let (li, lj) = self.elastic.rigid_lengths();
        let f_node = crate::rigid_arm::to_node_force(&f_flex, li, lj);
        let f_global = self.elastic.axis.rotate_to_global(&f_node);
        LocalVec {
            data: SmallVec::from_slice(&f_global),
        }
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        let du_global: [f64; 12] = std::array::from_fn(|i| du.data[i]);
        let du_local = self.elastic.axis.rotate_to_local(&du_global);
        // N-M 相関: バネの trial より先に現在軸力で降伏モーメントを更新する
        self.apply_mn_interaction(Some(&du_local));
        self.elastic.update_state(du, commit, _ctx);
        // 節点変位のトライアル更新後に内部平衡（可撓端回転）を解き直し、
        // ばね変形（相対回転）のトライアル状態を確定する。
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
        // 弾性梁部分の変位状態（committed/trial）もスナップショットへ含める。
        // これを欠くと、非収束ステップのロールバック（restore_state）後に
        // 弾性部のトライアル変位だけが失敗した反復の値のまま残ってしまう。
        // 可撓端回転（内部自由度）も同じ理由で含める。
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
        if let Some(snapshot) = state.downcast_ref::<Snapshot>() {
            if snapshot.0.len() == 2 {
                self.spring_i = snapshot.0[0].clone_box();
                self.spring_j = snapshot.0[1].clone_box();
            }
            [self.rot_i, self.rot_j, self.trial_rot_i, self.trial_rot_j] = snapshot.1;
            [self.thb_i, self.thb_j, self.trial_thb_i, self.trial_thb_j] = snapshot.2;
            self.elastic.committed_disp = snapshot.3;
            self.elastic.trial_disp = snapshot.4;
        }
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
        // 弾性梁部分の変位（committed/trial）もチェックポイントへ含める。
        // これを欠くと、レジューム時に弾性部の内力が変位 0 から再計算されて
        // 不整合になる（snapshot_state と同じ理由。FiberBeam の直列化と同規約）。
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

/// [`ConcentratedSpringBeam`] のチェックポイント形式（serialize/deserialize 共用）。
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
