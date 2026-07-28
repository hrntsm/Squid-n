//! 時刻歴応答解析の結果型。
//!
//! - [`ResponseResult`] — 解析結果（設計書 §10.5）
//! - [`ResponseHistory`] — UI 描画用の代表応答時刻歴
//! - [`ThRecording`] / [`StoryResponse`] — 3D アニメーション・層応答グラフ・
//!   部材履歴用の詳細記録（間引きあり）
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
    /// 3D アニメーション・層応答グラフ・部材履歴用の詳細記録（間引きあり）。
    /// 旧プロジェクトファイル（.scz）には無いフィールドのため、読込時は `None` で補う。
    #[serde(default)]
    pub recording: Option<ThRecording>,
    /// 非線形時刻歴（各部材の復元力特性を考慮した Newton 反復）で解析したか。
    /// 線形（Newmark-β・HHT-α）は false。旧プロジェクトファイル（.scz）には
    /// 無いフィールドのため、読込時は false（線形扱い）で補う。
    #[serde(default)]
    pub nonlinear: bool,
    /// 長期系荷重ケース（固定・積載等）を時刻歴開始前に静的載荷し、その応力状態を
    /// 初期条件としたか（非線形時刻歴の `NonlinearThCfg::apply_long_term` に対応。
    /// 線形時刻歴は重ね合わせ運用のため常に false）。旧プロジェクトファイル
    /// （.scz）には無いフィールドのため、読込時は false で補う。
    #[serde(default)]
    pub applied_long_term: bool,
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

/// 時刻歴応答の詳細記録（3D アニメーション・層応答グラフ・部材履歴用）。
///
/// メモリ対策として `record_every` ステップごとに 1 フレームだけ間引いて記録する
/// （既定は記録フレーム数が概ね 1000 になるよう自動決定、[`super::recording`] 参照）。
/// ただし `peak_disp`（`ResponseResult` 側）・[`Self::peak_member_forces`]・
/// [`StoryResponse`] の `peak_*` 各フィールド（`peak_shear_coeff`・
/// `peak_story_shear`・`peak_floor_accel`・`peak_floor_vel`・`peak_floor_disp`）は
/// 全ステップで更新し、間引かない。
///
/// 節点順・要素順は、それぞれ解析時の `model.nodes` / `model.elements` の
/// 添字順に一致する（UI 側はこの並びでモデルと突き合わせる）。
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct ThRecording {
    /// 記録間引き係数（`record_every` ステップごとに 1 フレーム記録）。
    pub record_every: usize,
    /// 記録フレームの時刻 [s]。
    pub frame_time: Vec<f64>,
    /// フレームごとの全節点変位（展開後の全自由度、node 順 `[ux,uy,uz,rx,ry,rz]`。
    /// 拘束・剛床従属自由度を含む相対変位、単位 mm・rad）。`[frame][node_idx]`。
    pub node_disp: Vec<Vec<[f64; 6]>>,
    /// X 方向（加振・応答の X 成分）の層応答。
    pub story_x: StoryResponse,
    /// Y 方向（加振・応答の Y 成分）の層応答。
    pub story_y: StoryResponse,
    /// フレームごとの部材端力分布（`model.elements` 順。線形解析は
    /// `recover_forces`、非線形解析は `state_member_forces` により算定。
    /// 内力分布を持たない要素は `None`）。`[frame][elem_idx]`。
    /// メモリ削減のため、各要素の `MemberForces.at` は両端 2 点（最小 ξ・最大 ξ）
    /// のみに間引いて保持する（UI の履歴ループは端部値のみ使用するため。
    /// 中間の評価断面は保持しない。全評価断面の包絡値は
    /// [`Self::peak_member_forces`] を参照）。
    pub member_forces: Vec<Vec<Option<squid_n_element::beam::MemberForces>>>,
    /// 全ステップ（間引きなし）での部材端力の包絡（各成分の絶対値最大値。
    /// 符号は極値そのものの符号を保持する）。`[elem_idx]`。
    pub peak_member_forces: Vec<Option<squid_n_element::beam::MemberForces>>,
}

/// 1 方向（X または Y）の層応答時刻歴（[`ThRecording`] 参照）。
///
/// 階の並びは `model.stories` の並び順（下層→上層、他の層別集計関数と同じ前提）。
/// `Node.story` が未設定の節点（基礎節点等）は集計対象外。
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct StoryResponse {
    /// 階リスト（記録対象、下→上。`model.stories` と同じ並び）。
    pub stories: Vec<squid_n_core::ids::StoryId>,
    /// 階ごとの地震用重量 [N]（`Story::seismic_weight`。層せん断力係数 Ci の
    /// 分母 ΣWj に用いる）。
    pub story_weight: Vec<f64>,
    /// フレームごとの層せん断力 [N]（慣性力ベース: 当該層以上に属する節点の
    /// Σ質量·(相対加速度＋地動加速度)、符号は加振方向の符号規約に合わせ
    /// `ResponseHistory::base_shear` と同じ）。`[frame][story]`。
    pub story_shear: Vec<Vec<f64>>,
    /// フレームごとの階絶対加速度 [mm/s²]（相対加速度＋地動加速度、階に属する
    /// 節点の質量加重平均。質量ゼロの階は単純平均）。`[frame][story]`。
    pub floor_accel: Vec<Vec<f64>>,
    /// フレームごとの階速度（相対）[mm/s]。`[frame][story]`。
    pub floor_vel: Vec<Vec<f64>>,
    /// フレームごとの階変位（相対）[mm]。`[frame][story]`。
    pub floor_disp: Vec<Vec<f64>>,
    /// 層せん断力係数の最大値 `Ci = max|Qi| / Σ(j≧i) Wj`（全ステップの最大値、
    /// 間引きなし）。`[story]`。
    pub peak_shear_coeff: Vec<f64>,
    /// 層せん断力の絶対値最大（全ステップ、間引きなし）。`[story]`。
    /// `story_shear`（フレーム記録、間引きあり）とは異なりピークを取り逃さない。
    /// 旧プロジェクトファイル（.scz）には無いフィールドのため、読込時は 0 埋め。
    #[serde(default)]
    pub peak_story_shear: Vec<f64>,
    /// 階絶対加速度の絶対値最大（全ステップ、間引きなし）。`[story]`。
    #[serde(default)]
    pub peak_floor_accel: Vec<f64>,
    /// 階速度（相対）の絶対値最大（全ステップ、間引きなし）。`[story]`。
    #[serde(default)]
    pub peak_floor_vel: Vec<f64>,
    /// 階変位（相対）の絶対値最大（全ステップ、間引きなし）。`[story]`。
    #[serde(default)]
    pub peak_floor_disp: Vec<f64>,
}
