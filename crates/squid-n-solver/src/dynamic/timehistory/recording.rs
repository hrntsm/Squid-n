//! 時刻歴応答の詳細記録（[`ThRecording`]）を組み立てる補助。
//!
//! - [`auto_record_every`] — 記録フレーム数が概ね 1000 になるよう `record_every` を自動決定
//! - [`ThRecorder`] — 記録の状態機械（層別集計・部材内力包絡・フレーム間引き）
//! - [`member_forces_linear`] / [`member_forces_nonlinear`] — 部材端力分布の算定
//!   （線形は `recover_forces`、非線形は `state_member_forces`）

use super::result::{StoryResponse, ThRecording};
use crate::constraint::Reducer;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::model::Model;
use squid_n_element::beam::MemberForces;
use squid_n_element::behavior::{Ctx, ElemState, ElementBehavior};

/// 記録フレーム数が概ね 1000 になるよう `record_every`（間引き係数）を自動決定する。
/// `n_steps` はステップ数（フレーム数は `n_steps+1`）。
pub(crate) fn auto_record_every(n_steps: usize) -> usize {
    (n_steps / 1000).max(1)
}

/// 線形解析用: 全自由節点変位 `u_free` から各要素の部材内力分布を復元する
/// （`model.elements` と同じ並び。`behaviors` は事前に `build_behavior` で
/// 構築済みのもの）。
pub(crate) fn member_forces_linear(
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    u_free: &[f64],
) -> Vec<Option<MemberForces>> {
    behaviors
        .iter()
        .map(|b| {
            let gdofs = b.global_dofs(dofmap);
            let mut u_elem = vec![0.0; gdofs.len()];
            for (k, &g) in gdofs.iter().enumerate() {
                if g != usize::MAX && g < u_free.len() {
                    u_elem[k] = u_free[g];
                }
            }
            b.recover_forces(&u_elem)
        })
        .collect()
}

/// 非線形解析用: 現在の要素状態（committed / trial）から部材内力分布を取り出す
/// （`model.elements` と同じ並び）。
pub(crate) fn member_forces_nonlinear(
    model: &Model,
    behaviors: &[Box<dyn ElementBehavior>],
) -> Vec<Option<MemberForces>> {
    let ctx = Ctx { model };
    let state = ElemState::default();
    behaviors
        .iter()
        .map(|b| b.state_member_forces(&state, &ctx))
        .collect()
}

/// 部材端力の包絡を更新する（各成分・各評価位置の絶対値最大。符号は極値の符号を保持）。
/// 評価位置数が一致しない場合（要素構成が変わることは実運用上ないが防御的に）は
/// 短い方の長さまでのみ更新する。
fn merge_peak_member_forces(peak: &mut [Option<MemberForces>], current: &[Option<MemberForces>]) {
    for (p, c) in peak.iter_mut().zip(current.iter()) {
        match (p.as_mut(), c) {
            (Some(pf), Some(cf)) => {
                let n = pf.at.len().min(cf.at.len());
                for i in 0..n {
                    for k in 0..6 {
                        if cf.at[i].1[k].abs() > pf.at[i].1[k].abs() {
                            pf.at[i].1[k] = cf.at[i].1[k];
                        }
                    }
                }
            }
            (None, Some(cf)) => *p = Some(cf.clone()),
            _ => {}
        }
    }
}

/// 節点順の全自由度変位（拘束・従属自由度を含む）を組み立てる。`u_free` は
/// 自由 DOF 空間（`dofmap` のアクティブ添字順）の展開済みベクトル。
fn expand_node_disp(model: &Model, dofmap: &DofMap, u_free: &[f64]) -> Vec<[f64; 6]> {
    let mut out = vec![[0.0f64; 6]; model.nodes.len()];
    for ni in 0..model.nodes.len() {
        for d in 0..DOF_PER_NODE {
            let g = ni * DOF_PER_NODE + d;
            if let Some(a) = dofmap.active(g) {
                out[ni][d] = u_free[a as usize];
            }
        }
    }
    out
}

/// 階に属する節点の (自由 DOF 添字, 質量重み) の一覧。質量重みは `Node::mass` の
/// 当該方向成分（無ければ 0。フロア応答の質量加重平均、無ければ単純平均にフォールバック）。
type StoryDofGroup = Vec<(usize, f64)>;

/// 時刻歴応答の詳細記録（[`ThRecording`]）を組み立てる状態機械。
///
/// 呼び出し側（`linear`/`hht`/`nonlinear` の各時刻歴ループ）は、各ステップ確定後に
/// [`Self::record_step`] を呼ぶ。フレーム間引き・部材内力包絡・層別集計をここに集約し、
/// 3 つの積分スキームで同一のロジックを共有する。
pub(crate) struct ThRecorder {
    record_every: usize,
    n_steps: u64,
    story_ids: Vec<squid_n_core::ids::StoryId>,
    weights: Vec<f64>,
    groups_x: Vec<StoryDofGroup>,
    groups_y: Vec<StoryDofGroup>,

    frame_time: Vec<f64>,
    node_disp: Vec<Vec<[f64; 6]>>,
    member_forces: Vec<Vec<Option<MemberForces>>>,
    peak_member_forces: Vec<Option<MemberForces>>,

    story_shear_x: Vec<Vec<f64>>,
    story_shear_y: Vec<Vec<f64>>,
    floor_accel_x: Vec<Vec<f64>>,
    floor_accel_y: Vec<Vec<f64>>,
    floor_vel_x: Vec<Vec<f64>>,
    floor_vel_y: Vec<Vec<f64>>,
    floor_disp_x: Vec<Vec<f64>>,
    floor_disp_y: Vec<Vec<f64>>,
    peak_shear_coeff_x: Vec<f64>,
    peak_shear_coeff_y: Vec<f64>,
}

impl ThRecorder {
    /// `n_steps` は解析の全ステップ数（地震波のサンプル数）。`record_every` が
    /// `None` の場合は [`auto_record_every`] で自動決定する。`n_elems` は
    /// `model.elements.len()`（部材内力包絡の初期化用）。
    pub(crate) fn new(
        model: &Model,
        dofmap: &DofMap,
        n_steps: usize,
        n_elems: usize,
        record_every: Option<usize>,
    ) -> Self {
        let record_every = record_every
            .unwrap_or_else(|| auto_record_every(n_steps))
            .max(1);
        let n_story = model.stories.len();

        let build_groups = |dir_idx: usize| -> Vec<StoryDofGroup> {
            model
                .stories
                .iter()
                .map(|story| {
                    story
                        .node_ids
                        .iter()
                        .filter_map(|&nid| {
                            let ni = nid.index();
                            let node = model.nodes.get(ni)?;
                            let g = ni * DOF_PER_NODE + dir_idx;
                            let a = dofmap.active(g)?;
                            let w = node.mass.map(|m| m[dir_idx]).unwrap_or(0.0).max(0.0);
                            Some((a as usize, w))
                        })
                        .collect()
                })
                .collect()
        };

        Self {
            record_every,
            n_steps: n_steps as u64,
            story_ids: model.stories.iter().map(|s| s.id).collect(),
            weights: model
                .stories
                .iter()
                .map(|s| s.seismic_weight.unwrap_or(0.0))
                .collect(),
            groups_x: build_groups(0),
            groups_y: build_groups(1),
            frame_time: Vec::new(),
            node_disp: Vec::new(),
            member_forces: Vec::new(),
            peak_member_forces: vec![None; n_elems],
            story_shear_x: Vec::new(),
            story_shear_y: Vec::new(),
            floor_accel_x: Vec::new(),
            floor_accel_y: Vec::new(),
            floor_vel_x: Vec::new(),
            floor_vel_y: Vec::new(),
            floor_disp_x: Vec::new(),
            floor_disp_y: Vec::new(),
            peak_shear_coeff_x: vec![0.0; n_story],
            peak_shear_coeff_y: vec![0.0; n_story],
        }
    }

    /// 階ごとの層せん断力・階絶対加速度・階速度・階変位を求める（1 方向分）。
    /// `m_r` は当該方向の `M·r`（自由 DOF 空間）、`xg` は当該時刻・当該方向の
    /// 地動加速度。層せん断力は「当該層以上に属する節点の慣性力」の累積
    /// （`Σ m_r[dof]·(a_free[dof]+xg)` の符号反転、`history::record_history_step`
    /// のベースシアと同じ符号規約）。
    #[allow(clippy::too_many_arguments)]
    fn compute_story_dir(
        groups: &[StoryDofGroup],
        m_r: &[f64],
        u_free: &[f64],
        v_free: &[f64],
        a_free: &[f64],
        xg: f64,
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
        let n = groups.len();
        let mut level_force = vec![0.0; n];
        let mut accel = vec![0.0; n];
        let mut vel = vec![0.0; n];
        let mut disp = vec![0.0; n];
        for (i, g) in groups.iter().enumerate() {
            if g.is_empty() {
                continue;
            }
            let (mut total_w, mut sum_a, mut sum_v, mut sum_u) = (0.0, 0.0, 0.0, 0.0);
            let (mut simple_a, mut simple_v, mut simple_u) = (0.0, 0.0, 0.0);
            let mut lf = 0.0;
            for &(dof, w) in g {
                let a_abs = a_free.get(dof).copied().unwrap_or(0.0) + xg;
                let v_val = v_free.get(dof).copied().unwrap_or(0.0);
                let u_val = u_free.get(dof).copied().unwrap_or(0.0);
                simple_a += a_abs;
                simple_v += v_val;
                simple_u += u_val;
                sum_a += w * a_abs;
                sum_v += w * v_val;
                sum_u += w * u_val;
                total_w += w;
                lf += m_r.get(dof).copied().unwrap_or(0.0) * a_abs;
            }
            let cnt = g.len() as f64;
            if total_w > 0.0 {
                accel[i] = sum_a / total_w;
                vel[i] = sum_v / total_w;
                disp[i] = sum_u / total_w;
            } else {
                accel[i] = simple_a / cnt;
                vel[i] = simple_v / cnt;
                disp[i] = simple_u / cnt;
            }
            level_force[i] = lf;
        }
        let mut shear = vec![0.0; n];
        let mut acc = 0.0;
        for i in (0..n).rev() {
            acc += level_force[i];
            shear[i] = -acc;
        }
        (shear, accel, vel, disp)
    }

    /// 1 ステップ分の記録を追加する。`step` は確定した時刻ステップ番号
    /// （0 が初期状態、`n_steps` が最終ステップ）。`u_red`/`v_red`/`a_red` は
    /// 縮約空間の変位・速度・加速度、`m_r_x`/`m_r_y` は自由 DOF 空間の `M·r`
    /// （時刻歴解析の呼び出し側で 1 回だけ組み立てたものを共有する）。
    /// `member_forces_now` は当該ステップの全要素の部材内力分布
    /// （[`member_forces_linear`] / [`member_forces_nonlinear`]）。
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn record_step(
        &mut self,
        step: u64,
        time: f64,
        model: &Model,
        dofmap: &DofMap,
        reducer: &Reducer,
        m_r_x: &[f64],
        m_r_y: &[f64],
        u_red: &[f64],
        v_red: &[f64],
        a_red: &[f64],
        xg_x: f64,
        xg_y: f64,
        member_forces_now: &[Option<MemberForces>],
    ) {
        // 部材内力包絡は全ステップ更新（間引かない）。
        merge_peak_member_forces(&mut self.peak_member_forces, member_forces_now);

        let u_free = reducer.expand_u(u_red);
        let v_free = reducer.expand_u(v_red);
        let a_free = reducer.expand_u(a_red);

        let (shear_x, accel_x, vel_x, disp_x) =
            Self::compute_story_dir(&self.groups_x, m_r_x, &u_free, &v_free, &a_free, xg_x);
        let (shear_y, accel_y, vel_y, disp_y) =
            Self::compute_story_dir(&self.groups_y, m_r_y, &u_free, &v_free, &a_free, xg_y);

        // 層せん断力係数のピークは全ステップ更新（間引かない）。
        for i in 0..self.weights.len() {
            let above: f64 = self.weights[i..].iter().sum();
            if above > 0.0 {
                let ci_x = shear_x[i].abs() / above;
                if ci_x > self.peak_shear_coeff_x[i] {
                    self.peak_shear_coeff_x[i] = ci_x;
                }
                let ci_y = shear_y[i].abs() / above;
                if ci_y > self.peak_shear_coeff_y[i] {
                    self.peak_shear_coeff_y[i] = ci_y;
                }
            }
        }

        // フレーム記録は間引く（record_every ごと。最終ステップは必ず含める）。
        if step.is_multiple_of(self.record_every as u64) || step == self.n_steps {
            self.frame_time.push(time);
            self.node_disp
                .push(expand_node_disp(model, dofmap, &u_free));
            self.member_forces.push(member_forces_now.to_vec());
            self.story_shear_x.push(shear_x);
            self.story_shear_y.push(shear_y);
            self.floor_accel_x.push(accel_x);
            self.floor_accel_y.push(accel_y);
            self.floor_vel_x.push(vel_x);
            self.floor_vel_y.push(vel_y);
            self.floor_disp_x.push(disp_x);
            self.floor_disp_y.push(disp_y);
        }
    }

    /// 記録を確定し [`ThRecording`] を返す。
    pub(crate) fn finish(self) -> ThRecording {
        ThRecording {
            record_every: self.record_every,
            frame_time: self.frame_time,
            node_disp: self.node_disp,
            story_x: StoryResponse {
                stories: self.story_ids.clone(),
                story_weight: self.weights.clone(),
                story_shear: self.story_shear_x,
                floor_accel: self.floor_accel_x,
                floor_vel: self.floor_vel_x,
                floor_disp: self.floor_disp_x,
                peak_shear_coeff: self.peak_shear_coeff_x,
            },
            story_y: StoryResponse {
                stories: self.story_ids,
                story_weight: self.weights,
                story_shear: self.story_shear_y,
                floor_accel: self.floor_accel_y,
                floor_vel: self.floor_vel_y,
                floor_disp: self.floor_disp_y,
                peak_shear_coeff: self.peak_shear_coeff_y,
            },
            member_forces: self.member_forces,
            peak_member_forces: self.peak_member_forces,
        }
    }
}
