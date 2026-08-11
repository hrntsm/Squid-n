//! プッシュオーバー解析の結果・イベント型（P5 §7.4）。
//!
//! - [`CapacityPoint`] — 性能曲線の 1 点
//! - [`HingeEvent`] / [`HingeLevel`] — ヒンジ発生事象とレベル
//! - [`DuctilityMethod`] — 塑性率の算定方式
//! - [`MechanismType`] — 崩壊機構種別
//! - [`ShearYieldEvent`] — せん断降伏イベント
//! - [`PushoverMemberResponse`] — 終局時の部材別応答
//! - [`PushoverResult`] / [`PushoverStep`] — 解析結果とステップ記録

use squid_n_core::ids::ElemId;

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

/// 増分解析がどのように終了したか（P5 §7.4）。
///
/// 従来は非収束・特異化を含む全ての打ち切りが無言で、荷重制御が低い荷重係数で
/// 収束不能になっても Qu が「その時点までの最大ベースシア」として正常な結果の
/// 顔で返っていた（保有水平耐力の過小評価を利用者が判別できない）。終了理由を
/// 結果へ明示し、目標到達以外の打ち切りを表示側で警告できるようにする。
///
/// 複数フェーズ（荷重制御→変位制御→弧長法）を経る場合は、目標到達が最優先、
/// それ以外は**最後に実行されたフェーズの終了理由**を記録する。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize, Default)]
pub enum PushoverTermination {
    /// 終了目標（頂部変位・最大層間変形角）へ到達して終了した。
    TargetReached,
    /// 荷重制御の λ 上限（段階制御=1、荷重増分のみ+目標有効=10）まで載荷して
    /// 終了した（目標は未到達）。
    LambdaCap {
        /// 到達した荷重係数。
        lambda: f64,
    },
    /// 押込みスケジュール（変位制御の上限変位・弧長法の最大ステップ数）を
    /// 完了して終了した（目標判定は未成立）。
    ScheduleCompleted,
    /// 増分半減を尽くしても釣合い反復が収束せず打ち切った。
    /// **Qu が過小評価の可能性がある**（目標未到達のまま性能曲線が途切れている）。
    NonConvergence {
        /// 打ち切ったフェーズ（"荷重制御"・"変位制御"・"弧長法"）。
        phase: String,
        /// 打ち切り時点の確定済み荷重係数。
        load_factor: f64,
    },
    /// 接線剛性の分解が失敗（崩壊機構の形成・耐力喪失による特異化）して
    /// 打ち切った。弧長法フェーズでは耐力喪失の終了判定として期待される終了。
    TangentSingular {
        /// 打ち切ったフェーズ。
        phase: String,
        /// 打ち切り時点の確定済み荷重係数。
        load_factor: f64,
    },
    /// 旧プロジェクトファイル（終了理由が未記録）。
    #[default]
    Unknown,
}

impl PushoverTermination {
    /// 目標未到達のまま途中で打ち切られた（Qu が過小評価の可能性がある）か。
    pub fn is_premature(&self) -> bool {
        matches!(
            self,
            PushoverTermination::NonConvergence { .. }
                | PushoverTermination::TangentSingular { .. }
        )
    }

    /// 表示用の説明文（UI・レポート共通）。
    pub fn describe(&self) -> String {
        match self {
            PushoverTermination::TargetReached => "終了目標へ到達".into(),
            PushoverTermination::LambdaCap { lambda } => {
                format!("荷重係数上限 λ={:.2} まで載荷", lambda)
            }
            PushoverTermination::ScheduleCompleted => "押込みスケジュール完了".into(),
            PushoverTermination::NonConvergence { phase, load_factor } => format!(
                "{}が λ={:.3} で収束不能（目標未到達。Qu 過小評価の可能性）",
                phase, load_factor
            ),
            PushoverTermination::TangentSingular { phase, load_factor } => format!(
                "{}で λ={:.3} にて接線剛性が特異化（機構形成・耐力喪失）",
                phase, load_factor
            ),
            PushoverTermination::Unknown => "終了理由未記録（旧形式の結果）".into(),
        }
    }
}

/// 増分解析の制御方式（P5 §7）。
///
/// 既定の段階制御と、比較検証用の荷重増分のみの 2 方式。いずれも外力は
/// Ai 分布の比例荷重パターン λ·q で共通し、λ の決め方だけが異なる。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PushoverControl {
    /// 荷重制御（λ=1 まで）→変位制御→弧長法（オプション）の段階切替（既定）。
    /// 耐力ピーク（崩壊機構形成）を変位制御で通過し、頭打ち・低下も追跡できる。
    #[default]
    Phased,
    /// 荷重増分のみ。変位制御・弧長法へは移行せず（`use_arc_length` は無視）、
    /// 終了目標が有効な場合は λ=1 を超えて同じ刻みで荷重増分を継続する。
    /// 増分半減でも収束しない（＝これ以上の荷重に釣合う解がない、耐力ピーク近傍）
    /// 時点で打ち切る。段階制御との結果比較（変位制御の要否確認）用。
    LoadOnly,
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

/// ヒンジ発生事象（P5 §7.4）。
///
/// **記録粒度に注意**: `track_hinges` は確定ステップごとに「その時点で閾値を
/// 超えている材端」をすべて記録するため、一度降伏した材端は以降のステップでも
/// 毎回記録される（発生の初回のみのイベント列ではなく、ステップごとの状態
/// スナップショットの連なり）。消費者は (elem, 端) で集約して最高レベル・
/// 最大塑性率・初回ステップを取り出すこと（`squid-n-app` の `aggregate_hinges`
/// 参照。塑性率の最大値はこの毎ステップ記録に依存している）。
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
    /// 層崩壊。`layer` は [`squid_n_core::model::Layer::index`]（下から 0 始まり）。
    StoryCollapse {
        layer: usize,
    },
    Partial,
}

/// せん断降伏イベント（SRC 柱・SRC 耐震壁の部材ランク判定に用いる）。
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

/// ヒンジ詳細図用の部材応答履歴（1 部材分）。結果サイズを抑えるため、ヒンジまたは
/// せん断降伏が記録された部材についてのみ [`PushoverResult::member_history`] に保持する。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct MemberHistory {
    pub elem: ElemId,
    /// 確定ステップごとの端応答（[`PushoverResult::steps`] と同じ並び・同じ長さ）。
    pub records: Vec<MemberStepState>,
}

/// 1 確定ステップの部材端応答（ヒンジ詳細図の応答経路の 1 点。格納量を抑えるため
/// f32）。曲げは危険断面＝剛域フェイス位置の局所成分、回転は弦（変形後の材端を
/// 結ぶ直線）からの材端回転で、M-θ 曲線・N-M 相関図の応答経路の描画に用いる。
#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct MemberStepState {
    /// 軸力 [N]（圧縮正）。
    pub n: f32,
    /// i 端の局所曲げモーメント My（弱軸まわり）[N·mm]。
    pub my_i: f32,
    /// i 端の局所曲げモーメント Mz（強軸まわり）[N·mm]。
    pub mz_i: f32,
    /// j 端の局所曲げモーメント My [N·mm]。
    pub my_j: f32,
    /// j 端の局所曲げモーメント Mz [N·mm]。
    pub mz_j: f32,
    /// i 端の弦からの材端回転（局所 y まわり）[rad]。
    pub ry_i: f32,
    /// i 端の弦からの材端回転（局所 z まわり）[rad]。
    pub rz_i: f32,
    /// j 端の弦からの材端回転（局所 y まわり）[rad]。
    pub ry_j: f32,
    /// j 端の弦からの材端回転（局所 z まわり）[rad]。
    pub rz_j: f32,
}

/// プッシュオーバー解析結果（P5 §7.4）
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PushoverResult {
    pub steps: Vec<PushoverStep>,
    pub capacity_curve: Vec<CapacityPoint>,
    /// ヒンジ記録（確定ステップごとの閾値超過スナップショットの連なり。
    /// 同一材端が複数ステップで重複して並ぶ。[`HingeEvent`] の記録粒度参照）。
    pub hinges: Vec<HingeEvent>,
    /// せん断降伏イベント履歴（SRC 柱・SRC 耐震壁の部材ランク判定に使用、
    /// [`ShearYieldEvent`] 参照）。
    pub shear_yields: Vec<ShearYieldEvent>,
    pub mechanism: MechanismType,
    pub qu: f64,
    /// 最終確定ステップ時の部材別応答（設計用応力・部材別 Rp の直接反映用、
    /// [`PushoverMemberResponse`]）。ステップが 1 つも確定しなかった場合は空。
    pub member_response: Vec<PushoverMemberResponse>,
    /// この結果を生成した制御方式（[`PushoverControl`]）。結果画面・CSV で
    /// どの方式の結果かを識別するために保持する。旧プロジェクトファイルには
    /// ないフィールドのため、読込時は既定値（段階制御）で補う。
    #[serde(default)]
    pub control: PushoverControl,
    /// ヒンジ詳細図用の部材応答履歴（ヒンジ・せん断降伏が記録された部材のみ）。
    /// 旧プロジェクトファイルにはないフィールドのため、読込時は空で補う。
    #[serde(default)]
    pub member_history: Vec<MemberHistory>,
    /// 終局（最終確定ステップ）時のファイバー断面状態（ヒンジ・せん断降伏が記録
    /// されたファイバー要素のみ。断面塑性化状況の可視化用）。旧プロジェクト
    /// ファイルにはないフィールドのため、読込時は空で補う。
    #[serde(default)]
    pub fiber_states: Vec<(ElemId, Vec<squid_n_element::behavior::FiberSectionState>)>,
    /// 解析がどのように終了したか（[`PushoverTermination`]）。目標到達以外の
    /// 打ち切り（非収束・特異化）は Qu が過小評価の可能性があるため、表示側は
    /// [`PushoverTermination::is_premature`] で警告すること。旧プロジェクト
    /// ファイルにはないフィールドのため、読込時は `Unknown` で補う。
    #[serde(default)]
    pub termination: PushoverTermination,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PushoverStep {
    pub load_factor: f64,
    pub top_disp: f64,
    pub base_shear: f64,
    pub story_drifts: Vec<f64>,
}
