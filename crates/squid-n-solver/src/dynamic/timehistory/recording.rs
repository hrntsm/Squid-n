//! 時刻歴応答の詳細記録（[`ThRecording`]）を組み立てる補助。
//!
//! - [`auto_record_every`] — 記録フレーム数が概ね 1000 になるよう `record_every` を自動決定
//! - [`ThRecorder`] — 記録の状態機械（層別集計・部材内力包絡・フレーム間引き）
//! - [`member_forces_linear`] / [`member_forces_nonlinear`] — 部材端力分布の算定
//!   （線形は `recover_forces`、非線形は `state_member_forces`）

use super::result::{StoryResponse, ThRecording};
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::model::Model;
use squid_n_element::beam::MemberForces;
use squid_n_element::behavior::{Ctx, ElementBehavior};

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
            let u_elem = crate::common::elem_loop::gather_u_elem(&gdofs, u_free);
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
    behaviors
        .iter()
        .map(|b| b.state_member_forces(&ctx))
        .collect()
}

/// 非線形経路の線材内力の欠落ガード（[`crate::linear::ensure_line_member_forces`]
/// の非線形版）。線材（梁・ファイバー・MS・ブレース）の `state_member_forces` が
/// `None` を返す要素実装の不備を、解析開始時点でエラーとして顕在化させる
/// （このまま続けると当該部材の応力履歴が全ステップ空のまま無言で欠落する）。
pub(crate) fn ensure_line_member_forces_nonlinear(
    model: &Model,
    member_forces: &[Option<MemberForces>],
) -> Result<(), squid_n_math::solver::SolveError> {
    use squid_n_core::model::ElementKind;
    let missing: Vec<u32> = model
        .elements
        .iter()
        .zip(member_forces)
        .filter(|(e, mf)| {
            matches!(
                e.kind,
                ElementKind::Beam
                    | ElementKind::Fiber
                    | ElementKind::MultiSpring
                    | ElementKind::Brace { .. }
            ) && mf.is_none()
        })
        .map(|(e, _)| e.id.0)
        .collect();
    if missing.is_empty() {
        return Ok(());
    }
    let head: Vec<String> = missing.iter().take(5).map(|id| id.to_string()).collect();
    let more = if missing.len() > 5 {
        format!(" 他{}件", missing.len() - 5)
    } else {
        String::new()
    };
    Err(squid_n_math::solver::SolveError::InvalidInput(format!(
        "非線形解析で線材の部材内力を取得できませんでした: 部材 ID {}{}。\
         要素実装の不具合です（state_member_forces 未実装。このまま続けると\
         応力図・断面検定から当該部材が無言で欠落します）。",
        head.join(", "),
        more
    )))
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

/// 部材内力分布を両端 2 点（最小 ξ・最大 ξ）のみへ間引く。
///
/// `ThRecording::member_forces`（フレームごとの記録）は 3D アニメーション等の
/// UI 側履歴ループが端部値のみ使うため、両端に絞ってメモリを削減する
/// （中間の評価断面は保持しない）。包絡 `peak_member_forces` は従来どおり
/// [`merge_peak_member_forces`] で全評価断面を保持する（本関数は適用しない）。
/// `at` が 2 点以下の要素はそのまま返す。
fn trim_member_forces_to_endpoints(forces: &[Option<MemberForces>]) -> Vec<Option<MemberForces>> {
    forces
        .iter()
        .map(|mf| {
            mf.as_ref().map(|f| {
                if f.at.len() <= 2 {
                    return f.clone();
                }
                let min =
                    f.at.iter()
                        .cloned()
                        .fold(f.at[0], |acc, x| if x.0 < acc.0 { x } else { acc });
                let max =
                    f.at.iter()
                        .cloned()
                        .fold(f.at[0], |acc, x| if x.0 > acc.0 { x } else { acc });
                MemberForces { at: vec![min, max] }
            })
        })
        .collect()
}

/// 節点順の全自由度変位（拘束・従属自由度を含む）を組み立てる。`u_free` は
/// 自由 DOF 空間（`dofmap` のアクティブ添字順）の展開済みベクトル
/// （単一実装は core 側）。
fn expand_node_disp(model: &Model, dofmap: &DofMap, u_free: &[f64]) -> Vec<[f64; 6]> {
    dofmap.expand_to_nodes(u_free, model.nodes.len())
}

/// 階に属する節点の自由 DOF の一覧。
///
/// - `avg`: 階応答（加速度・速度・変位）の代表値算定に使う当該方向並進 DOF と
///   質量重みの組。質量重みは `Node::mass` の当該方向成分
///   （なければ 0。質量加重平均、全て 0 なら単純平均にフォールバック）。
/// - `force`: 層せん断力の慣性力集計に使う DOF。**当該方向の並進 DOF のみ**を
///   含める（節点慣性力ベクトル `f_abs = M·a_free + ẍg・M・r` を並進成分だけ
///   集計する。`f_abs` は疎行列ベクトル積 `M·a_free` を経ているため、回転 DOF
///   との連成（一貫質量行列の非対角項）は既にこの並進成分に反映済みで、
///   回転 DOF 自体を集計へ含める必要はない）。
#[derive(Default)]
struct StoryDofGroup {
    avg: Vec<(usize, f64)>,
    force: Vec<usize>,
}

impl StoryDofGroup {
    fn is_empty(&self) -> bool {
        self.avg.is_empty() && self.force.is_empty()
    }
}

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
    /// `weight_above[i]` = 当該層以上（i 番目〜最上層）の地震用重量の累積和
    /// （P11: 層せん断力係数 Ci の分母 ΣWj を `record_step` 呼び出しごとに
    /// O(n_story) で再計算していたのを、階構成が解析中不変なことを利用して
    /// `new` で 1 回だけ O(n_story) で事前計算する。これにより `record_step`
    /// 側は O(1) 参照になり、全体で O(n_story²) → O(n_story) に落ちる）。
    weight_above: Vec<f64>,
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
    // 層応答ピーク（全ステップ更新、間引きなし）。フレーム記録（*_x/*_y の
    // Vec<Vec<f64>>）は record_every で間引くため、間引きフレームに現れない
    // 極大値を取り逃さないよう別途保持する。
    peak_story_shear_x: Vec<f64>,
    peak_story_shear_y: Vec<f64>,
    peak_floor_accel_x: Vec<f64>,
    peak_floor_accel_y: Vec<f64>,
    peak_floor_vel_x: Vec<f64>,
    peak_floor_vel_y: Vec<f64>,
    peak_floor_disp_x: Vec<f64>,
    peak_floor_disp_y: Vec<f64>,
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

        // 階の代表 Z 座標（所属節点の平均 Z）。どの階にも属さない節点の慣性力を
        // 最も近い階へ計上する際に使う。
        let story_repr_z: Vec<f64> = model
            .stories
            .iter()
            .map(|story| {
                let mut sum_z = 0.0;
                let mut cnt = 0.0f64;
                for &nid in &story.node_ids {
                    if let Some(node) = model.nodes.get(nid.index()) {
                        sum_z += node.coord[2];
                        cnt += 1.0;
                    }
                }
                if cnt > 0.0 {
                    sum_z / cnt
                } else {
                    0.0
                }
            })
            .collect();

        let build_groups = |dir_idx: usize| -> Vec<StoryDofGroup> {
            let mut groups: Vec<StoryDofGroup> = model
                .stories
                .iter()
                .map(|story| {
                    let mut g = StoryDofGroup::default();
                    for &nid in &story.node_ids {
                        let ni = nid.index();
                        let Some(node) = model.nodes.get(ni) else {
                            continue;
                        };
                        if let Some(a) = dofmap.active(ni * DOF_PER_NODE + dir_idx) {
                            let w = node.mass.map(|m| m[dir_idx]).unwrap_or(0.0).max(0.0);
                            g.avg.push((a as usize, w));
                            g.force.push(a as usize);
                        }
                    }
                    g
                })
                .collect();
            // どの階にも属さない節点（基礎レベルの自由節点など）の当該方向並進 DOF は、
            // Z 座標が最も近い階の層せん断力に計上する（1 層目〜最上層の慣性力の総和＝
            // ベースシアという恒等関係を、全並進 DOF を漏れなくいずれかの階へ
            // 割り当てることで保つ）。階応答の代表値（avg）には含めない。
            if !groups.is_empty() {
                let mut assigned = vec![false; model.nodes.len()];
                for story in &model.stories {
                    for &nid in &story.node_ids {
                        if let Some(f) = assigned.get_mut(nid.index()) {
                            *f = true;
                        }
                    }
                }
                for (ni, done) in assigned.iter().enumerate() {
                    if *done {
                        continue;
                    }
                    let Some(node) = model.nodes.get(ni) else {
                        continue;
                    };
                    if let Some(a) = dofmap.active(ni * DOF_PER_NODE + dir_idx) {
                        let nearest = story_repr_z
                            .iter()
                            .enumerate()
                            .min_by(|(_, za), (_, zb)| {
                                (node.coord[2] - *za)
                                    .abs()
                                    .partial_cmp(&(node.coord[2] - *zb).abs())
                                    .unwrap_or(std::cmp::Ordering::Equal)
                            })
                            .map(|(i, _)| i);
                        if let Some(i) = nearest {
                            groups[i].force.push(a as usize);
                        }
                    }
                }
            }
            groups
        };

        let weights: Vec<f64> = model
            .stories
            .iter()
            .map(|s| s.seismic_weight.unwrap_or(0.0))
            .collect();
        // P11: weight_above[i] = Σ_{j=i}^{n-1} weights[j] を末尾から1回の走査で
        // 前計算する（階構成・重量は解析中不変。record_step 側は毎回この結果を
        // 参照するだけになる）。
        let mut weight_above = vec![0.0; weights.len()];
        {
            let mut acc = 0.0;
            for i in (0..weights.len()).rev() {
                acc += weights[i];
                weight_above[i] = acc;
            }
        }

        Self {
            record_every,
            n_steps: n_steps as u64,
            story_ids: model.stories.iter().map(|s| s.id).collect(),
            weights,
            weight_above,
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
            peak_story_shear_x: vec![0.0; n_story],
            peak_story_shear_y: vec![0.0; n_story],
            peak_floor_accel_x: vec![0.0; n_story],
            peak_floor_accel_y: vec![0.0; n_story],
            peak_floor_vel_x: vec![0.0; n_story],
            peak_floor_vel_y: vec![0.0; n_story],
            peak_floor_disp_x: vec![0.0; n_story],
            peak_floor_disp_y: vec![0.0; n_story],
        }
    }

    /// 階ごとの層せん断力・階絶対加速度・階速度・階変位を求める（1 方向分）。
    /// `m_r` は当該方向の `M·r`（自由 DOF 空間）、`ma_free` は自由 DOF 空間の
    /// `M·a_free`（[`super::common::mass_accel_free`] で 1 ステップに 1 回だけ
    /// 算定したものを共有する）、`xg` は当該時刻・当該方向の地動加速度。
    ///
    /// 層せん断力は「当該層以上に属する節点の並進 DOF の慣性力」の累積の符号反転
    /// （節点慣性力ベクトル `f_abs = M·a_free + ẍg・M・r` の当該方向並進成分のみを
    /// 集計、`history::record_history_step` のベースシアと同じ定義）。
    #[allow(clippy::too_many_arguments)]
    fn compute_story_dir(
        groups: &[StoryDofGroup],
        m_r: &[f64],
        u_free: &[f64],
        v_free: &[f64],
        a_free: &[f64],
        ma_free: &[f64],
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
            for &(dof, w) in &g.avg {
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
            }
            if total_w > 0.0 {
                accel[i] = sum_a / total_w;
                vel[i] = sum_v / total_w;
                disp[i] = sum_u / total_w;
            } else if !g.avg.is_empty() {
                let cnt = g.avg.len() as f64;
                accel[i] = simple_a / cnt;
                vel[i] = simple_v / cnt;
                disp[i] = simple_u / cnt;
            }
            // 層せん断力: 節点慣性力ベクトル f_abs = M·a_free + ẍg・M・r の
            // 当該方向並進 DOF（g.force）のみを集計する。
            let mut lf = 0.0;
            for &dof in &g.force {
                let f_abs = ma_free.get(dof).copied().unwrap_or(0.0)
                    + xg * m_r.get(dof).copied().unwrap_or(0.0);
                lf += f_abs;
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
    /// （0 が初期状態、`n_steps` が最終ステップ）。`u_free`/`v_free`/`a_free` は
    /// 呼び出し側で展開済みの自由 DOF 空間の変位・速度・加速度（`dofmap` の
    /// アクティブ添字順）で、`m_r_x`/`m_r_y` は自由 DOF 空間の `M·r`
    /// （いずれも時刻歴解析の呼び出し側で 1 ステップに 1 回だけ展開・組み立てた
    /// ものを共有する。P9: 従来は本関数の内部で `Reducer::expand_u` を毎回
    /// 呼び直しており、呼び出し側の展開と合わせて `u_free` が 1 ステップに
    /// 2 回展開されていた）。`ma_free` は自由 DOF 空間の `M·a_free`
    /// （[`super::common::mass_accel_free`] で呼び出し側が 1 ステップに 1 回だけ
    /// 算定したものを共有する。層せん断力の節点慣性力ベクトル算定に使う）。
    /// `member_forces_now` は当該ステップの全要素の部材内力分布
    /// （[`member_forces_linear`] / [`member_forces_nonlinear`]）。
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn record_step(
        &mut self,
        step: u64,
        time: f64,
        model: &Model,
        dofmap: &DofMap,
        m_r_x: &[f64],
        m_r_y: &[f64],
        ma_free: &[f64],
        u_free: &[f64],
        v_free: &[f64],
        a_free: &[f64],
        xg_x: f64,
        xg_y: f64,
        member_forces_now: &[Option<MemberForces>],
    ) {
        // 部材内力包絡は全ステップ更新（間引かない）。
        merge_peak_member_forces(&mut self.peak_member_forces, member_forces_now);

        let (shear_x, accel_x, vel_x, disp_x) =
            Self::compute_story_dir(&self.groups_x, m_r_x, u_free, v_free, a_free, ma_free, xg_x);
        let (shear_y, accel_y, vel_y, disp_y) =
            Self::compute_story_dir(&self.groups_y, m_r_y, u_free, v_free, a_free, ma_free, xg_y);

        // 層せん断力係数のピークは全ステップ更新（間引かない）。P11: 分母
        // ΣWj（当該層以上の重量累積和）は `new` で事前計算済みの
        // `weight_above` を参照するだけにし、`record_step` 呼び出しごとの
        // O(n_story) 再計算（全体で O(n_story²)）を避ける。
        for i in 0..self.weights.len() {
            let above = self.weight_above[i];
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

        // 層応答ピーク（層せん断力・階絶対加速度・階速度・階変位の絶対値最大）は
        // フレーム間引きに関係なく全ステップで更新する（中-3: 間引きフレームの
        // 合間に生じるピークを取り逃さないため）。
        Self::update_peak_abs(&mut self.peak_story_shear_x, &shear_x);
        Self::update_peak_abs(&mut self.peak_story_shear_y, &shear_y);
        Self::update_peak_abs(&mut self.peak_floor_accel_x, &accel_x);
        Self::update_peak_abs(&mut self.peak_floor_accel_y, &accel_y);
        Self::update_peak_abs(&mut self.peak_floor_vel_x, &vel_x);
        Self::update_peak_abs(&mut self.peak_floor_vel_y, &vel_y);
        Self::update_peak_abs(&mut self.peak_floor_disp_x, &disp_x);
        Self::update_peak_abs(&mut self.peak_floor_disp_y, &disp_y);

        // フレーム記録は間引く（record_every ごと。最終ステップは必ず含める）。
        if step.is_multiple_of(self.record_every as u64) || step == self.n_steps {
            self.frame_time.push(time);
            self.node_disp.push(expand_node_disp(model, dofmap, u_free));
            self.member_forces
                .push(trim_member_forces_to_endpoints(member_forces_now));
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

    /// 絶対値最大の更新（`peak[i] = max(peak[i], |current[i]|)`）。長さが異なる場合は
    /// 短い方まで（要素構成・階構成は解析中不変のため通常は一致する）。
    fn update_peak_abs(peak: &mut [f64], current: &[f64]) {
        for (p, &c) in peak.iter_mut().zip(current.iter()) {
            let c_abs = c.abs();
            if c_abs > *p {
                *p = c_abs;
            }
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
                peak_story_shear: self.peak_story_shear_x,
                peak_floor_accel: self.peak_floor_accel_x,
                peak_floor_vel: self.peak_floor_vel_x,
                peak_floor_disp: self.peak_floor_disp_x,
            },
            story_y: StoryResponse {
                stories: self.story_ids,
                story_weight: self.weights,
                story_shear: self.story_shear_y,
                floor_accel: self.floor_accel_y,
                floor_vel: self.floor_vel_y,
                floor_disp: self.floor_disp_y,
                peak_shear_coeff: self.peak_shear_coeff_y,
                peak_story_shear: self.peak_story_shear_y,
                peak_floor_accel: self.peak_floor_accel_y,
                peak_floor_vel: self.peak_floor_vel_y,
                peak_floor_disp: self.peak_floor_disp_y,
            },
            member_forces: self.member_forces,
            peak_member_forces: self.peak_member_forces,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, MaterialCategory,
        Node, Section, Story,
    };

    /// 高-2 検証用: 密度による一貫質量を持つ 2 層（3 節点）の柱列。曲げ回転
    /// （Ry）を解放し、一貫質量行列の並進-回転連成（M_ux_ry 等）を意図的に
    /// 有効にする（`Dof6Mask(0b101110)` = Ux・Ry のみ自由。他の並進・回転は拘束）。
    fn two_story_density_mass_model(density: f64) -> Model {
        let free_ux_ry = Dof6Mask(0b101110);
        Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                    support_spring: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [0.0, 0.0, 3000.0],
                    restraint: free_ux_ry,
                    mass: None,
                    story: Some(StoryId(0)),
                    support_spring: None,
                },
                Node {
                    id: NodeId(2),
                    coord: [0.0, 0.0, 6000.0],
                    restraint: free_ux_ry,
                    mass: None,
                    story: Some(StoryId(1)),
                    support_spring: None,
                },
            ],
            elements: vec![
                ElementData {
                    id: ElemId(0),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [1.0, 0.0, 0.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                },
                ElementData {
                    id: ElemId(1),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [1.0, 0.0, 0.0],
                    },
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    force_regime: ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                },
            ],
            sections: vec![Section {
                id: SectionId(0),
                name: "col".into(),
                area: 10000.0,
                iy: 8.333e6,
                iz: 8.333e6,
                j: 1.0e6,
                depth: 100.0,
                width: 100.0,
                as_y: 0.0,
                as_z: 0.0,
                floor: None,
                panel_thickness: None,
                thickness: None,
                shape: None,
            }],
            materials: vec![Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "steel".into(),
                category: MaterialCategory::Steel,
                young: 205000.0,
                poisson: 0.3,
                density,
                shear: None,
                fc: None,
                fy: None,
            }],
            stories: vec![
                Story {
                    level_kind: Default::default(),
                    structure: Default::default(),
                    id: StoryId(0),
                    name: "1F".into(),
                    elevation: 3000.0,
                    node_ids: vec![NodeId(1)],
                    diaphragms: vec![],
                    seismic_weight: Some(1000.0),
                    weight_override: None,
                },
                Story {
                    level_kind: Default::default(),
                    structure: Default::default(),
                    id: StoryId(1),
                    name: "2F".into(),
                    elevation: 6000.0,
                    node_ids: vec![NodeId(2)],
                    diaphragms: vec![],
                    seismic_weight: Some(1000.0),
                    weight_override: None,
                },
            ],
            ..Default::default()
        }
    }

    /// 高-2: `StoryDofGroup::force`（層せん断力の慣性力集計対象）は当該方向の
    /// 並進 DOF のみを含み、回転 DOF を含まないこと。節点ごとに並進 DOF は
    /// 高々 1 つのため、各階の `force.len()` は「当該方向へ割り当てられた
    /// 節点数（=1）」と一致するはず（修正前は、同じ節点で活性な回転 DOF
    /// （本モデルでは Ry）も無条件に含んでいたため 2 になっていた）。
    #[test]
    fn test_story_force_dof_group_excludes_rotational_dof() {
        let model = two_story_density_mass_model(7.85e-9);
        let dofmap = DofMap::build(&model);
        let recorder = ThRecorder::new(&model, &dofmap, 1, model.elements.len(), None);

        assert_eq!(recorder.groups_x.len(), 2);
        for (i, g) in recorder.groups_x.iter().enumerate() {
            assert_eq!(
                g.force.len(),
                1,
                "story[{i}].force は当該方向の並進 DOF（節点1つ分）のみを含むはず: {:?}",
                g.force
            );
            // avg（階代表値）と force（せん断力集計）の DOF は一致するはず。
            assert_eq!(g.avg.len(), g.force.len());
            assert_eq!(g.force[0], g.avg[0].0);
        }
    }

    /// 高-2 (a): 2 層・一貫質量（密度）モデルで、1 層目（最下層）の層せん断力
    /// （＝全層の慣性力の累積）がベースシア（全体慣性力の合計）と一致すること。
    /// 回転 DOF を解放し並進-回転連成が実際に生じる状態で検証する
    /// （`story_shear[i]` は「当該層以上の慣性力の累積」であり、1 層目が
    /// 全体合計＝ベースシアと一致する。全層の `story_shear` を単純合計した値
    /// ではない点に注意）。
    #[test]
    fn test_story_shear_sum_matches_base_shear_with_consistent_mass() {
        let model = two_story_density_mass_model(7.85e-9);
        let dofmap = DofMap::build(&model);
        let reducer = crate::constraint::Reducer::build(&model, &dofmap);

        let damping = crate::damping::Damping::StiffnessProportional {
            h: 0.02,
            omega: 10.0,
            basis: crate::damping::StiffnessKind::Initial,
        };
        let dt = 0.001;
        let n_steps = 30;
        let wave = super::super::GroundMotion {
            dt,
            accel_x: {
                let mut a = vec![0.0; n_steps];
                a[0] = 2000.0;
                a[1] = -1000.0;
                a
            },
            accel_y: None,
            accel_theta: None,
        };
        let newmark = super::super::NewmarkCfg {
            beta: 0.25,
            gamma: 0.5,
            dt,
        };

        let result = super::super::linear_time_history_analysis(
            &model,
            &dofmap,
            &reducer,
            &wave,
            &newmark,
            &damping,
            &[0.0],
            &[0.0],
            false,
            None,
        )
        .expect("should run");

        let recording = result.recording.expect("recording");
        assert!(!recording.story_x.story_shear.is_empty());
        let mut checked = 0;
        for (k, &t) in recording.frame_time.iter().enumerate() {
            // frame_time はいずれかの time と一致するはず（浮動小数点の丸めに
            // 依存しないよう、記録済みの時刻刻みそのものと突き合わせる）。
            let Some(step) = result.time.iter().position(|&tt| tt == t) else {
                continue;
            };
            let layer0_shear = recording.story_x.story_shear[k][0];
            let expected = result.history.base_shear[step];
            assert!(
                (layer0_shear - expected).abs() < expected.abs().max(1.0) * 1e-6,
                "frame {k} (t={t}): layer0 shear {layer0_shear} should match base shear {expected}"
            );
            checked += 1;
        }
        assert!(checked > 0, "比較できたフレームが1件もありません");
    }
}
