//! 解析条件（GUI の「解析」タブの設定値に対応する。GUI 非依存）。

use squid_n_solver::statics::analysis::{AiMode, SeismicDir};

/// 解析タブの設定値（GUI 非依存）。
///
/// `.scz`（`squid-n-io` の `SczExtras::analysis_settings`）へ同梱される。
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct AnalysisSettings {
    /// 固有値解析のモード数
    pub n_modes: usize,
    /// 地震静的(Ai)の方向・Ai算定法・地域係数・地盤種別・標準せん断力係数
    pub seismic_dir: SeismicDir,
    pub ai_mode: AiMode,
    pub z: f64,
    pub soil: squid_n_load::ai::SoilClass,
    pub c0: f64,
    /// 増分解析（プッシュオーバー）: 方向・最大ステップ・目標変位 [mm]
    pub push_dir: SeismicDir,
    pub push_steps: usize,
    pub push_max_disp: f64,
    /// 増分解析: 目標変位[mm]による終了判定を使うか。
    pub push_use_max_disp: bool,
    /// 増分解析: 目標最大層間変形角による終了判定を使うか。
    pub push_use_drift_angle: bool,
    /// 増分解析: 目標最大層間変形角の分母 n（角度は 1/n [rad]）。
    pub push_drift_denom: f64,
    /// 増分解析: 塑性率（ductility）の算定方式（構造力学）。
    pub ductility_method: squid_n_solver::nonlinear::pushover::DuctilityMethod,
    /// 増分解析: 制御方式（段階制御／荷重増分のみ）。
    pub push_control: squid_n_solver::nonlinear::pushover::PushoverControl,
    /// 増分解析: 長期系荷重ケースを水平力増分の前に初期載荷するか。
    pub push_apply_long_term: bool,
    /// 質点系モデル生成: モデル化タイプ（等価せん断型など）。
    pub lumped_mass_type: squid_n_solver::dynamic::lumped_mass::LumpedMassType,
    /// 質点系モデル生成: 第1折点判定の割線剛性比（0..1、既定 0.75）。
    pub lumped_secant_ratio: f64,
    /// 時刻歴: 減衰比
    pub th_damping: f64,
    /// 時刻歴サンプル波: 刻み [s]・継続時間 [s]・周期 [s]・振幅 [mm/s²]
    pub th_dt: f64,
    pub th_duration: f64,
    pub th_period: f64,
    pub th_amp: f64,
    /// 時刻歴の入力方向
    pub th_dir: ThDir,
    /// 時刻歴の減衰モデル
    pub th_damping_model: ThDampingModel,
    /// Rayleigh の2次モード減衰比（1次は th_damping を使用）
    pub th_h2: f64,
    /// 時刻歴を非線形で解析するか。
    pub th_nonlinear: bool,
    /// 非線形時刻歴: 長期系荷重ケースを時刻歴開始前に静的載荷し初期条件とするか
    /// （`th_nonlinear` が true のときのみ意味を持つ）。
    pub th_apply_long_term: bool,
    /// 非線形時刻歴: 各時刻ステップの Newton 反復の最大回数（既定 50）。
    pub th_max_iter: usize,
    /// 非線形時刻歴: Newton 収束判定の相対許容誤差。
    pub th_tol: f64,
    /// 時刻歴の詳細記録のフレーム間引き係数。0 は自動決定。
    pub th_record_every: usize,
    /// 位相差入力（ねじれ加振）を考慮するか。
    pub phase_diff_enabled: bool,
    /// せん断波速度 Vs [m/s]。
    pub phase_diff_vs: f64,
    /// 矩形基礎長さ L [m]。
    pub phase_diff_length_m: f64,
    /// 入射角 θ [°]。
    pub phase_diff_incidence_deg: f64,
    /// 位相遅れ方向が Y なら true（X なら false）。
    pub phase_diff_dir_y: bool,
    /// 荷重組合せ自動生成の多雪区域フラグ。
    pub heavy_snow_zone: bool,
    /// 多雪区域の積雪荷重低減係数 δ1（既定 0.7）。
    pub snow_delta1: f64,
    /// 同 δ3（既定 0.35）。
    pub snow_delta3: f64,
    /// RC 短期許容せん断力の「損傷制御のための検討」（false=安全確保のための検討）。
    pub rc_damage_control: bool,
    /// 地震時短期の設計用せん断力 QD の決定方法（QD1/QD2/min）。
    pub qd_method: squid_n_design_jp::QdMethod,
    /// RC 梁付着検定の方式（1999 / 1991。既定 1999）。
    pub bond_method: squid_n_design_jp::BondMethod,
    /// 解析の並列スレッド数（0=自動(全コア)、1=単一スレッド(結果の完全再現性を保証)、n=固定）。
    pub threads: usize,
    /// 動的解析の質量モデルの方式（[`squid_n_core::model::MassMethod`]）。
    pub mass_method: squid_n_core::model::MassMethod,
    /// 質点系の次元（2 次元せん断串 / 3 次元 Ux,Uy,θz）。
    #[serde(default)]
    pub lumped_dim: squid_n_solver::dynamic::lumped_mass::StickDim,
    /// 層並進剛性の定義（層 Q/δ または柱 ki）。
    #[serde(default)]
    pub lumped_stiffness: squid_n_solver::dynamic::lumped_mass::LumpedStiffnessSource,
    /// 質点系を非線形で解くか。
    #[serde(default)]
    pub lumped_nonlinear: bool,
    /// 質点系の加振・解析方向。
    #[serde(default)]
    pub lumped_dir: SeismicDir,
    /// 質点系固有値のモード数（既定 3）。
    #[serde(default = "default_lumped_n_modes")]
    pub lumped_n_modes: usize,
    /// 質点系時刻歴の減衰比（既定 0.02）。
    #[serde(default = "default_lumped_th_damping")]
    pub lumped_th_damping: f64,
    /// 質点系時刻歴のサンプル波刻み [s]（既定 0.01）。
    #[serde(default = "default_lumped_th_dt")]
    pub lumped_th_dt: f64,
    /// 質点系時刻歴のサンプル波継続時間 [s]（既定 10）。
    #[serde(default = "default_lumped_th_duration")]
    pub lumped_th_duration: f64,
    /// 質点系時刻歴のサンプル波周期 [s]（既定 0.5）。
    #[serde(default = "default_lumped_th_period")]
    pub lumped_th_period: f64,
    /// 質点系時刻歴のサンプル波振幅 [mm/s²]（既定 1000）。
    #[serde(default = "default_lumped_th_amp")]
    pub lumped_th_amp: f64,
}

fn default_lumped_n_modes() -> usize {
    3
}
fn default_lumped_th_damping() -> f64 {
    0.02
}
fn default_lumped_th_dt() -> f64 {
    0.01
}
fn default_lumped_th_duration() -> f64 {
    10.0
}
fn default_lumped_th_period() -> f64 {
    0.5
}
fn default_lumped_th_amp() -> f64 {
    1000.0
}

/// 時刻歴の入力方向選択。X・Y に加え、同一波形を両方向へ同時入力する「X+Y」を持つ。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ThDir {
    X,
    Y,
    Xy,
}

/// 時刻歴の減衰モデル選択。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ThDampingModel {
    /// 初期剛性比例（C=2h/ω1·Ke）。
    StiffnessProportional,
    /// Rayleigh 減衰（1次・2次で目標減衰比）。
    Rayleigh,
    /// モード別減衰（各モードに減衰比 h を与える。非線形では初期剛性モード）。
    Modal,
    /// 瞬間（接線）剛性比例・α1 一定（C=2h/ω1e·Kt を毎ステップ再構成）。
    TangentAlpha1,
    /// 瞬間（接線）剛性比例・h1 一定（ω1 を毎ステップ更新して減衰比 h1 を保つ）。
    TangentH1,
}

impl Default for AnalysisSettings {
    fn default() -> Self {
        Self {
            n_modes: 3,
            seismic_dir: SeismicDir::X,
            ai_mode: AiMode::Approx,
            z: 1.0,
            soil: squid_n_load::ai::SoilClass::II,
            c0: 0.2,
            push_dir: SeismicDir::X,
            push_steps: 50,
            push_max_disp: 500.0,
            push_use_max_disp: false,
            push_use_drift_angle: true,
            push_drift_denom: 150.0,
            ductility_method: squid_n_solver::nonlinear::pushover::DuctilityMethod::default(),
            push_control: squid_n_solver::nonlinear::pushover::PushoverControl::default(),
            push_apply_long_term: true,
            lumped_mass_type: squid_n_solver::dynamic::lumped_mass::LumpedMassType::default(),
            lumped_secant_ratio: 0.75,
            th_damping: 0.02,
            th_dt: 0.01,
            th_duration: 10.0,
            th_period: 0.5,
            th_amp: 1000.0,
            th_dir: ThDir::X,
            th_damping_model: ThDampingModel::StiffnessProportional,
            th_h2: 0.02,
            th_nonlinear: false,
            th_apply_long_term: false,
            th_max_iter: 50,
            th_tol: 1e-6,
            th_record_every: 0,
            phase_diff_enabled: false,
            phase_diff_vs: 200.0,
            phase_diff_length_m: 20.0,
            phase_diff_incidence_deg: 30.0,
            phase_diff_dir_y: false,
            heavy_snow_zone: false,
            snow_delta1: 0.7,
            snow_delta3: 0.35,
            rc_damage_control: true,
            qd_method: squid_n_design_jp::QdMethod::Min,
            bond_method: squid_n_design_jp::BondMethod::Rc1999,
            threads: 0,
            mass_method: squid_n_core::model::MassMethod::default(),
            lumped_dim: squid_n_solver::dynamic::lumped_mass::StickDim::default(),
            lumped_stiffness: squid_n_solver::dynamic::lumped_mass::LumpedStiffnessSource::default(
            ),
            lumped_nonlinear: false,
            lumped_dir: SeismicDir::X,
            lumped_n_modes: 3,
            lumped_th_damping: 0.02,
            lumped_th_dt: 0.01,
            lumped_th_duration: 10.0,
            lumped_th_period: 0.5,
            lumped_th_amp: 1000.0,
        }
    }
}
