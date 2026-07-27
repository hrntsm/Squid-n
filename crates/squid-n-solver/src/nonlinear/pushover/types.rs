//! プッシュオーバー解析の結果・イベント型（P5 §7.4）。
//!
//! - [`CapacityPoint`] — 性能曲線の 1 点
//! - [`HingeEvent`] / [`HingeLevel`] — ヒンジ発生事象とレベル
//! - [`DuctilityMethod`] — 塑性率の算定方式
//! - [`MechanismType`] — 崩壊機構種別
//! - [`ShearYieldEvent`] — せん断降伏イベント
//! - [`PushoverMemberResponse`] — 終局時の部材別応答
//! - [`PushoverResult`] / [`PushoverStep`] — 解析結果とステップ記録

use squid_n_core::ids::{ElemId, StoryId};

/// 増分解析の終了目標（P5 §7）。有効化した判定のうち**いずれか**に達した時点で
/// 変位増分を打ち切る。両方 `None` の場合は荷重制御（λ=1）までで終了する。
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct PushoverTarget {
    /// 目標最大変位 [mm]（頂部＝最上階マスター節点の加力方向変位）。`None` は判定しない。
    pub max_disp: Option<f64>,
    /// 目標最大層間変形角 [rad]（全層の最大値、例: 1/150）。`None` は判定しない。
    pub max_drift_angle: Option<f64>,
}

impl Default for PushoverTarget {
    /// 既定は層間変形角 1/150 のみを有効とする。
    fn default() -> Self {
        Self {
            max_disp: None,
            max_drift_angle: Some(1.0 / 150.0),
        }
    }
}

impl PushoverTarget {
    /// 目標変位のみを指定する（旧 API 互換。0 以下は判定なし＝荷重制御のみで終了）。
    pub fn from_max_disp(max_disp: f64) -> Self {
        Self {
            max_disp: (max_disp > 0.0).then_some(max_disp),
            max_drift_angle: None,
        }
    }

    /// いずれかの判定が有効か。
    pub fn is_enabled(&self) -> bool {
        self.max_disp.is_some() || self.max_drift_angle.is_some()
    }

    /// 現在の応答が目標に達したか（有効な判定の OR）。
    /// `max_drift_angle_now` は全層の最大層間変形角 [rad]
    /// （[`super::response::max_story_drift_angle`] で算定した値）。
    pub(crate) fn reached(&self, roof_disp: f64, max_drift_angle_now: f64) -> bool {
        if self.max_disp.is_some_and(|d| roof_disp >= d) {
            return true;
        }
        self.max_drift_angle
            .is_some_and(|a| max_drift_angle_now >= a)
    }
}

/// 性能曲線の1点（P5 §7.4）
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct CapacityPoint {
    pub step: u32,
    pub roof_disp: f64,
    pub base_shear: f64,
    pub story_shear: Vec<f64>,
    pub story_drift: Vec<f64>,
}

/// ヒンジ発生事象（P5 §7.4）
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct HingeEvent {
    pub step: u32,
    pub elem: ElemId,
    pub pos: f64,
    pub level: HingeLevel,
    pub ductility: f64,
}

/// ヒンジレベル（P5 §7.4）
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum HingeLevel {
    Crack,
    Yield,
    Ultimate,
}

/// 塑性率（ductility）の算定方式（ファイバーモデル（構造力学）の
/// 塑性率）。ユーザーが 3 方式から選択する。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum DuctilityMethod {
    /// (1) 塑性率基点歪みにより計算する方法（既定）。いずれかのセグメントの
    /// ひずみが塑性率基点ひずみ（RC: 引張 0.01・圧縮 0.005、鉄骨: 0.01）を
    /// 超えた時点の曲率を基点とし、μ=最大応答曲率/基点曲率。
    #[default]
    ReferenceStrain,
    /// (2) 重み付け平均塑性率 Jm による方法。Jm=Σσy·A·|ε|·μi/Σσy·A·|ε| が
    /// 1.0 以上となった時点の曲率を基点とする。
    WeightedAverageJm,
    /// (3) 降伏発生時を塑性率基点にする方法。いずれかのセグメントの塑性率が
    /// 1 を超えた（降伏した）時点の曲率を基点とする。
    FirstYield,
}

/// 崩壊機構種別（P5 §7.4）
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum MechanismType {
    Overall,
    StoryCollapse { story: StoryId },
    Partial,
}

/// せん断降伏イベント（段階的耐力喪失解析のせん断降伏判定）。
///
/// 部材端のせん断力（局所 Vy・Vz の材端最大値）がせん断降伏耐力 Qy
/// （[`compute_shear_yield_qy`] 参照）を超えたステップを記録する。曲げヒンジ
/// （[`HingeEvent`]）とは独立に判定され、曲げ降伏の有無に関わらず記録される。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ShearYieldEvent {
    pub step: u32,
    pub elem: ElemId,
}

/// 終局（最終確定ステップ）時の部材別応答（終局検定の設計用応力・
/// 部材別 Rp の直接反映に用いる）。プッシュオーバー最終ステップの部材端内力を
/// 局所座標へ射影し、強軸（局所 z まわり）・弱軸（局所 y まわり）の設計用曲げ・
/// せん断と軸力（圧縮正）、および部材変形角 Rp を保持する。
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct PushoverMemberResponse {
    pub elem: ElemId,
    /// 強軸（局所 z 軸まわり Mz）の設計用曲げモーメント [N·mm]（両端の最大絶対値）。
    pub m_strong: f64,
    /// 弱軸（局所 y 軸まわり My）の設計用曲げモーメント [N·mm]（両端の最大絶対値）。
    pub m_weak: f64,
    /// 強軸曲げに伴う設計用せん断力 Vy [N]（局所 y 方向、両端の最大絶対値）。
    pub shear_strong: f64,
    /// 弱軸曲げに伴う設計用せん断力 Vz [N]（局所 z 方向、両端の最大絶対値）。
    pub shear_weak: f64,
    /// 部材軸力 [N]（**圧縮正**、両端のうち圧縮側の代表値）。
    pub axial: f64,
    /// 終局時の部材変形角 Rp [rad]（弦回転角＝層間変形角相当の近似）。
    pub rp: f64,
    /// 終局時に部材が負担する**加力方向の水平力** [N]（材端力の載荷方向成分の
    /// 両端最大絶対値）。告示の βu（耐力壁・筋かいの水平耐力の和を保有水平耐力で
    /// 除した数値）の分子を、耐力壁・筋かい部材について集計するために用いる。
    pub horizontal_force: f64,
}

/// プッシュオーバー解析結果（P5 §7.4）
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PushoverResult {
    pub steps: Vec<PushoverStep>,
    pub capacity_curve: Vec<CapacityPoint>,
    pub hinges: Vec<HingeEvent>,
    /// せん断降伏イベント履歴（段階的耐力喪失解析の判定に使用、`strength_loss` モジュール参照）。
    pub shear_yields: Vec<ShearYieldEvent>,
    pub mechanism: MechanismType,
    pub qu: f64,
    /// 最終確定ステップ時の部材別応答（設計用応力・部材別 Rp の直接反映用、
    /// [`PushoverMemberResponse`]）。ステップが 1 つも確定しなかった場合は空。
    pub member_response: Vec<PushoverMemberResponse>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PushoverStep {
    pub load_factor: f64,
    pub top_disp: f64,
    pub base_shear: f64,
    pub story_drifts: Vec<f64>,
    /// 当該ステップ確定時点の全自由節点変位（`DofMap` のアクティブ添字順）。
    /// 段階的耐力喪失解析（`strength_loss` モジュール）が部材変形角を算定するための
    /// 記録で、既定では収集しない（オプトイン、`pushover_analysis_recording` 参照）。
    pub node_disp: Option<Vec<f64>>,
}
