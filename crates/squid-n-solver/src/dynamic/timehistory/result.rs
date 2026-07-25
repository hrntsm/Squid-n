//! 時刻歴応答解析の結果型。
//!
//! - [`ResponseResult`] — 解析結果（設計書 §10.5）
//! - [`ResponseHistory`] — UI 描画用の代表応答時刻歴
//! - [`TimeStepState`] — 1 時点の状態（チェックポイント／再開）

/// 時刻歴応答解析の結果（設計書 §10.5）。
/// 時系列の全量は結果I/O（§6）へストリーミングし、メモリに全保持しない。
/// 例外として UI 描画用の代表応答（1 節点変位・ベースシア・最上階変形角）のみ
/// `history` にステップごとの値を保持する。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ResponseResult {
    pub time: Vec<f64>,
    pub peak_disp: Vec<[f64; 6]>,
    pub story_drift_angle: Vec<f64>,
    pub cumulative_ductility: Vec<f64>,
    pub history: ResponseHistory,
}

/// UI 描画用の代表応答時刻歴（`time` と同じ長さ）。
/// 記録方向は入力加速度の絶対値和（Σ|ẍg|）が大きい方向を解析開始時に自動選択する
/// （`choose_record_dir_y` 参照）。X・Y いずれの加振でも代表応答がゼロにならない。
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct ResponseHistory {
    /// 記録節点（最も標高が高い、記録方向の自由度を持つ節点）。
    pub node: Option<squid_n_core::ids::NodeId>,
    /// 記録方向が Y なら true（X なら false）。
    pub record_dir_y: bool,
    /// 記録節点の記録方向相対変位 [mm]。
    pub node_disp: Vec<f64>,
    /// ベースシア(記録方向) [N]（全慣性力の合計、符号付き）。
    pub base_shear: Vec<f64>,
    /// 最上階の層間変形角 [rad]（符号付き。階が未定義なら 0）。
    pub top_drift_angle: Vec<f64>,
}

/// 時刻歴応答の1時点の状態（縮約空間）。チェックポイント／再開で使用。
#[derive(Clone, serde::Serialize, serde::Deserialize, PartialEq, Debug)]
pub struct TimeStepState {
    pub step: u64,
    pub time: f64,
    pub disp_red: Vec<f64>,
    pub vel_red: Vec<f64>,
    pub accel_red: Vec<f64>,
}
