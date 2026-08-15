//! 串団子モデル生成（せん断型多質点系、構造力学）。
//!
//! プッシュオーバー結果の層 Q-δ 関係を等包絡面積則でトリリニア骨格へ縮約する。
//!
//! - [`StoryTrilinear`] — 層のトリリニア骨格（Q-δ）。
//! - [`StoryStick`] — 串団子モデルの1質点（層）。
//! - [`LumpedMassType`] — モデル化タイプ（せん断型多質点系）。
//! - [`LumpedMassModel`] — 串団子モデル。
//! - [`fit_story_trilinear`] — 層 Q-δ 曲線を等包絡面積則でトリリニアへ縮約する。
//! - [`build_lumped_mass_model`] — プッシュオーバー結果から串団子モデルを生成する。

use crate::analysis::SeismicDir;
use crate::pushover::PushoverResult;
use squid_n_core::ids::StoryId;
use squid_n_core::model::Model;
use squid_n_core::units::GRAVITY_MM_S2;

/// 質点系の次元。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum StickDim {
    /// 2 次元（加力方向 1 本のせん断串）。
    #[default]
    Planar,
    /// 3 次元（各階 Ux, Uy, θz）。
    Spatial,
}

impl StickDim {
    pub fn label(self) -> &'static str {
        match self {
            Self::Planar => "2次元",
            Self::Spatial => "3次元",
        }
    }
}

/// 層並進剛性の定義。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum LumpedStiffnessSource {
    /// 地震静 EX/EY の層せん断 Q と層間変位 δ から K = Q/δ。
    #[default]
    StoryQd,
    /// 柱の ki = Qi/δi の合計（偏心率精算と同じ）。
    ColumnKi,
}

impl LumpedStiffnessSource {
    pub fn label(self) -> &'static str {
        match self {
            Self::StoryQd => "層 Q/δ（EX/EY）",
            Self::ColumnKi => "柱 ki",
        }
    }
}

/// 3 次元質点の層データ（剛心・ねじり・方向別骨格）。
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct StorySpatial {
    /// 回転慣性 J [t·mm²]（剛床マスターの RZ 質量）。
    pub j: f64,
    /// 質量重心 (x, y) [mm]。
    pub mass_xy: [f64; 2],
    /// 剛心 (x, y) [mm]。
    pub rigidity_xy: [f64; 2],
    /// X 方向初期剛性 [N/mm]。
    pub k1_x: f64,
    /// Y 方向初期剛性 [N/mm]。
    pub k1_y: f64,
    /// 剛心まわりのねじり剛性 KR [N·mm/rad]。
    pub kr: f64,
    /// X 方向トリリニア（非線形時。線形なら弾性相当）。
    pub skeleton_x: StoryTrilinear,
    /// Y 方向トリリニア。
    pub skeleton_y: StoryTrilinear,
}

/// 層のトリリニア骨格（Q-δ）。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StoryTrilinear {
    /// 初期剛性 K1 [N/mm]。
    pub k1: f64,
    /// 第1折点 (δ1[mm], Q1[N])。
    pub d1: f64,
    pub q1: f64,
    /// 第2折点 (δ2, Q2)。
    pub d2: f64,
    pub q2: f64,
    /// 第3折点＝終局 (δ3, Q3)。
    pub d3: f64,
    pub q3: f64,
}

impl StoryTrilinear {
    /// 第2勾配 K2 = (Q2−Q1)/(δ2−δ1)。
    pub fn k2(&self) -> f64 {
        if self.d2 > self.d1 {
            (self.q2 - self.q1) / (self.d2 - self.d1)
        } else {
            0.0
        }
    }
    /// 第3勾配 K3 = (Q3−Q2)/(δ3−δ2)。
    pub fn k3(&self) -> f64 {
        if self.d3 > self.d2 {
            (self.q3 - self.q2) / (self.d3 - self.d2)
        } else {
            0.0
        }
    }

    /// 線形ばね相当（折点を十分遠くに置いた弾性トリリニア）。
    pub fn elastic(k1: f64) -> Self {
        let k = k1.max(0.0);
        let d = 1.0e9;
        let q = k * d;
        Self {
            k1: k,
            d1: d,
            q1: q,
            d2: d,
            q2: q,
            d3: d,
            q3: q,
        }
    }
}

/// 串団子モデルの1質点（層）。
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct StoryStick {
    pub story: StoryId,
    /// 質量 [t]（= 地震重量 W / g）。
    pub mass: f64,
    /// 階高 [mm]。
    pub height: f64,
    /// 層の復元力特性（トリリニア）。
    pub skeleton: StoryTrilinear,
}

/// モデル化タイプ（せん断型多質点系、構造力学）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum LumpedMassType {
    /// 等価せん断型（曲げ剛性を剛とする）。
    #[default]
    EquivalentShear,
    /// 等価曲げせん断型（曲げ剛性を梁要素として考慮）。
    EquivalentBendingShear,
    /// 曲げせん断分離型（曲げ剛性を回転ばねとして考慮）。
    BendingShearSeparated,
}

impl LumpedMassType {
    pub fn label(&self) -> &'static str {
        match self {
            LumpedMassType::EquivalentShear => "等価せん断型",
            LumpedMassType::EquivalentBendingShear => "等価曲げせん断型",
            LumpedMassType::BendingShearSeparated => "曲げせん断分離型",
        }
    }
}

/// 串団子モデル。層ごとの質点と復元力特性を保持する。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct LumpedMassModel {
    pub model_type: LumpedMassType,
    pub stories: Vec<StoryStick>,
    #[serde(default)]
    pub dim: StickDim,
    #[serde(default)]
    pub stiffness_source: LumpedStiffnessSource,
    #[serde(default)]
    pub dir: SeismicDir,
    /// 非線形骨格（トリリニア）を使うか。false なら初期剛性の線形ばね。
    #[serde(default)]
    pub nonlinear: bool,
    /// 3 次元の層データ（`dim == Spatial` のとき `stories` と同順・同長）。
    #[serde(default)]
    pub spatial: Vec<StorySpatial>,
}

impl LumpedMassModel {
    /// 2 次元せん断串（既存テスト・増分 1 方向からの生成）。
    pub fn from_stories(model_type: LumpedMassType, stories: Vec<StoryStick>) -> Self {
        Self {
            model_type,
            stories,
            dim: StickDim::Planar,
            stiffness_source: LumpedStiffnessSource::StoryQd,
            dir: SeismicDir::X,
            nonlinear: true,
            spatial: Vec::new(),
        }
    }

    pub fn is_spatial(&self) -> bool {
        self.dim == StickDim::Spatial && self.spatial.len() == self.stories.len()
    }
}

/// 台形則で (0,0) から曲線終端までの包絡面積を求める。
pub(crate) fn envelope_area(pts: &[(f64, f64)]) -> f64 {
    let mut a = 0.0;
    let (mut pd, mut pq) = (0.0, 0.0);
    for &(d, q) in pts {
        a += 0.5 * (pq + q) * (d - pd);
        pd = d;
        pq = q;
    }
    a
}

/// 層 Q-δ 曲線（δ 昇順・正値）を等包絡面積則でトリリニアへ縮約する。
/// `secant_ratio`（0..1）: 第1折点＝割線剛性が K1 のこの比率以下となる変位。
pub fn fit_story_trilinear(curve: &[(f64, f64)], secant_ratio: f64) -> StoryTrilinear {
    // 正の変形のみ・δ 昇順に整える。ゼロ荷重ステップの解が丸め誤差程度の
    // 変形（〜1e-17 mm）を持つことがあるため、最大変形に対する相対許容差で
    // 実質ゼロの点を除外する（残すと K1 = 0/ε の縮退が起きる）。
    let d_max = curve.iter().map(|&(d, _)| d).fold(0.0, f64::max);
    let tol = d_max * 1e-9;
    let mut pts: Vec<(f64, f64)> = curve.iter().copied().filter(|&(d, _)| d > tol).collect();
    pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    pts.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-12);

    if pts.is_empty() {
        return StoryTrilinear {
            k1: 0.0,
            d1: 0.0,
            q1: 0.0,
            d2: 0.0,
            q2: 0.0,
            d3: 0.0,
            q3: 0.0,
        };
    }
    let (d_first, q_first) = pts[0];
    let (d3, q3) = *pts.last().unwrap();
    let k1 = if d_first > 0.0 {
        q_first / d_first
    } else {
        0.0
    };
    if k1 <= 0.0 || d3 <= d_first {
        // 単調1点・剛性不定は弾性トリリニア（折点なし）で返す。
        return StoryTrilinear {
            k1,
            d1: d3,
            q1: q3,
            d2: d3,
            q2: q3,
            d3,
            q3,
        };
    }
    // 第3勾配 K3 = 終端接線（[0, K1] にクランプ）。
    let k3 = if pts.len() >= 2 {
        let (dp, qp) = pts[pts.len() - 2];
        if d3 > dp {
            ((q3 - qp) / (d3 - dp)).clamp(0.0, k1)
        } else {
            0.0
        }
    } else {
        (q3 / d3).clamp(0.0, k1)
    };
    // 第1折点 δ1: 接線勾配が secant_ratio·K1 を初めて下回る直前の変位（弾性限）。
    // 第1勾配は K1。接線基準は割線基準より弾性限（折れ点）を鋭く捉える（降伏後剛性が
    // 小さい場合でも Q1=K1·δ1 が過大にならない）。
    let thr = secant_ratio * k1;
    let mut d1 = d3 * 0.5;
    let mut prev = (0.0, 0.0);
    let mut found = false;
    for &(d, q) in &pts {
        let tan = if d > prev.0 {
            (q - prev.1) / (d - prev.0)
        } else {
            k1
        };
        if tan < thr && prev.0 > 0.0 {
            d1 = prev.0;
            found = true;
            break;
        }
        prev = (d, q);
    }
    if !found {
        d1 = d3 * 0.5;
    }
    let d1 = d1.clamp(d_first, d3 * 0.9);
    let q1 = k1 * d1;

    // 等包絡面積: A_tri(δ2)=A_actual を解く。Q2 は第3勾配直線上 Q2=Q3−K3(δ3−δ2)。
    // A_tri は δ2 について線形（∂A/∂δ2 = ½[(Q1−Q3)+K3(δ3−δ1)] 一定）なので直接解ける。
    let a_actual = envelope_area(&pts);
    let a_tri = |d2: f64| {
        let q2 = q3 - k3 * (d3 - d2);
        0.5 * d1 * q1 + 0.5 * (q1 + q2) * (d2 - d1) + 0.5 * (q2 + q3) * (d3 - d2)
    };
    let slope = 0.5 * ((q1 - q3) + k3 * (d3 - d1));
    let d2 = if slope.abs() < 1e-30 {
        0.5 * (d1 + d3)
    } else {
        (d1 + (a_actual - a_tri(d1)) / slope).clamp(d1, d3)
    };
    let q2 = q3 - k3 * (d3 - d2);

    StoryTrilinear {
        k1,
        d1,
        q1,
        d2,
        q2,
        d3,
        q3,
    }
}

/// プッシュオーバー結果から串団子モデル（層ごとの質点・復元力特性）を生成する。
/// `secant_ratio`: 第1折点判定の割線剛性比（既定 0.75 程度）。
pub fn build_lumped_mass_model(
    model: &Model,
    pushover: &PushoverResult,
    model_type: LumpedMassType,
    secant_ratio: f64,
) -> LumpedMassModel {
    let layers = model.layers();
    let mut sticks = Vec::with_capacity(layers.len());
    for layer in &layers {
        let i = layer.index;
        // 長期荷重のみを載荷した初期点（λ=0、`push_apply_long_term` 有効時に
        // capacity_curve 先頭へ記録される）を層 Q-δ 曲線の原点とする。この点は
        // 水平力ゼロの状態だが、非対称な長期荷重により層にわずかな残留水平変形を
        // 持つことがある（丸め誤差ではなくミリメートル級になり得る）。総変位
        // （`total_disp`）は長期載荷フェーズから水平力増分フェーズへそのまま
        // 引き継がれるため、以降の各点の層間変形にもこの残留分が乗ったままであり、
        // 差し引かないと層剛性の相対分布（＝串団子モデルの振動モード）が歪む。
        // capacity_curve と steps は生成元で常に同じ並び・同じ長さで積まれる
        // （driver.rs の各 push 箇所参照）ので、対応するインデックスで拾える。
        let baseline: (f64, f64) = pushover
            .capacity_curve
            .iter()
            .zip(pushover.steps.iter())
            .find(|(_, step)| step.load_factor == 0.0)
            .and_then(|(cp, _)| {
                let d0 = cp.story_drift.get(i).copied()?;
                let q0 = cp.story_shear.get(i).copied()?;
                Some((d0, q0))
            })
            .unwrap_or((0.0, 0.0));
        // 層 i の Q-δ 曲線（各キャパシティ点の層せん断・層間変形。原点補正後）。
        // 補正後の長期のみ載荷点は (0, 0) に潰れ、`fit_story_trilinear` 側の
        // ゼロ点フィルタ（δ > 丸め許容差）でそのまま除外される。
        let curve: Vec<(f64, f64)> = pushover
            .capacity_curve
            .iter()
            .filter_map(|cp| {
                let d = (cp.story_drift.get(i).copied()? - baseline.0).abs();
                let q = (cp.story_shear.get(i).copied()? - baseline.1).abs();
                Some((d, q))
            })
            .collect();
        let skeleton = fit_story_trilinear(&curve, secant_ratio);

        // 質量 = 地震重量 / g（未設定なら節点質量の合計）。層の質量は上端の階に
        // 集中する（`Layer` 参照）。
        let mass = match layer.weight {
            Some(w) if w > 0.0 => w / GRAVITY_MM_S2,
            _ => layer
                .node_ids
                .iter()
                .filter_map(|nid| model.nodes.get(nid.index()))
                .filter_map(|n| n.mass)
                .map(|m| m[0].max(m[1]))
                .sum(),
        };

        sticks.push(StoryStick {
            // 層の識別は下端の階（＝層の呼び名になる床）で行う。
            story: layer.bottom,
            mass,
            height: layer.height.max(0.0),
            skeleton,
        });
    }
    LumpedMassModel::from_stories(model_type, sticks)
}
