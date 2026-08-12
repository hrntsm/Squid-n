//! 解析条件（GUI の「解析」タブの設定値に対応する。GUI 非依存）。
//!
//! 各解析の純粋計算（[`crate::compute`]）と荷重ケースの自動同期（[`crate::prepare`]）が
//! これを読む。GUI と MCP サーバが**同じ条件で同じ結果を得られる**ようにするため、
//! app ではなく本クレートに置く。

use squid_n_solver::analysis::{AiMode, SeismicDir};

/// 解析タブの設定値（GUI 非依存。テストからも使う）。
///
/// `.scz`（`squid-n-io` の `SczExtras::analysis_settings`）へ同梱される。
/// モデルから導出できない独立した設定値であり、同梱しないと解析結果を
/// 生成した条件が失われ、結果の再現性が保てないため。
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
    pub ductility_method: squid_n_solver::pushover::DuctilityMethod,
    /// 増分解析: 制御方式（段階制御／荷重増分のみ）。
    pub push_control: squid_n_solver::pushover::PushoverControl,
    /// 増分解析: 長期系荷重ケース（固定・積載等）を水平力増分の前に初期載荷するか。
    /// 長期荷重ケースがないモデルでは無視される（ソルバ側の対応実装に依存）。
    pub push_apply_long_term: bool,
    /// 質点系モデル生成: モデル化タイプ（等価せん断型など）。
    pub lumped_mass_type: squid_n_solver::lumped_mass::LumpedMassType,
    /// 質点系モデル生成: 第1折点判定の割線剛性比（0..1、既定 0.75）。
    pub lumped_secant_ratio: f64,
    /// 時刻歴: 減衰比・サンプル波の刻み/継続時間/周期/振幅 [mm/s²]
    pub th_damping: f64,
    pub th_dt: f64,
    pub th_duration: f64,
    pub th_period: f64,
    pub th_amp: f64,
    /// 時刻歴の入力方向(サンプル波・CSV波形の作用方向)
    pub th_dir: ThDir,
    /// 時刻歴の減衰モデル
    pub th_damping_model: ThDampingModel,
    /// Rayleigh の2次モード減衰比(1次は th_damping を使用)
    pub th_h2: f64,
    /// 時刻歴の積分法
    pub th_integrator: ThIntegrator,
    /// 時刻歴を非線形（各部材の復元力特性を考慮した Newton 反復）で解析するか。
    /// ON のとき積分法は Newmark-β 固定（HHT-α は選択不可）。
    pub th_nonlinear: bool,
    /// 非線形時刻歴: 長期系荷重ケース（固定・積載等）を時刻歴開始前に静的載荷し、
    /// その応力状態を初期条件とするか。線形時刻歴は重ね合わせ運用のため対象外
    /// （`th_nonlinear` が true のときのみ意味を持つ）。
    pub th_apply_long_term: bool,
    /// 非線形時刻歴: 各時刻ステップの Newton 反復の最大回数
    /// （既定は増分解析＝プッシュオーバーの内部反復回数と同じ 50）。
    pub th_max_iter: usize,
    /// 非線形時刻歴: Newton 収束判定の相対許容誤差。
    pub th_tol: f64,
    /// 時刻歴の詳細記録（3D アニメーション・層応答グラフ・部材履歴用）の
    /// フレーム間引き係数（線形・HHT-α・非線形の 3 経路共通）。
    /// 0 は自動決定（記録フレーム数が概ね 1000 になるよう調整）。
    /// ピーク値（`peak_disp`・`peak_member_forces`・`peak_shear_coeff`）は
    /// 間引きの影響を受けず全ステップで更新される。
    pub th_record_every: usize,
    /// 位相差入力（ねじれ加振）を考慮する（構造動力学）。
    pub phase_diff_enabled: bool,
    /// せん断波速度 Vs [m/s]。
    pub phase_diff_vs: f64,
    /// 矩形基礎長さ L [m]（位相遅れ方向の辺長）。
    pub phase_diff_length_m: f64,
    /// 入射角 θ [°]。
    pub phase_diff_incidence_deg: f64,
    /// 位相遅れ方向が Y なら true（X なら false）。基準の並進波もこの方向を用いる。
    pub phase_diff_dir_y: bool,
    /// 荷重組合せ自動生成（種別ベース）の多雪区域フラグ（施行令86条・82条）。
    pub heavy_snow_zone: bool,
    /// 多雪区域の積雪荷重低減係数 δ1（長期 G+P+δ1・S。既定 0.7）。
    pub snow_delta1: f64,
    /// 同 δ3（地震時 G+P+δ3・S±K。既定 0.35）。
    pub snow_delta3: f64,
    /// RC 短期許容せん断力の「損傷制御のための検討」（false=安全確保のための検討）。
    /// RC規準・令82条（断面算定条件 RC造）に対応。
    pub rc_damage_control: bool,
    /// 地震時短期の設計用せん断力 QD の決定方法（QD1/QD2/min）。
    pub qd_method: squid_n_design_jp::QdMethod,
    /// RC 梁付着検定の方式（1999 / 1991。既定 1999）。
    pub bond_method: squid_n_design_jp::BondMethod,
    /// 解析の並列スレッド数（0=自動(全コア)、1=単一スレッド(結果の完全再現性を保証)、n=固定）。
    pub threads: usize,
    /// 動的解析（固有値・時刻歴・精算周期）の質量モデルの方式
    /// （[`squid_n_core::model::MassMethod`]）。階の自動生成の実行時にモデルへ
    /// 反映される（`generate_stories_action`）。
    pub mass_method: squid_n_core::model::MassMethod,
}

/// 時刻歴の入力方向選択（UI 用）。X・Y に加え、同一波形を両方向へ同時入力する
/// 「X+Y」を持つ（`SeismicDir` は静的地震荷重・増分解析共用のため
/// 拡張せず、時刻歴専用にこの型を新設する）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ThDir {
    X,
    Y,
    Xy,
}

/// 時刻歴の減衰モデル選択（UI 用）。構造動力学の減衰マトリクス。
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

/// 時刻歴の積分法選択（UI 用）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ThIntegrator {
    NewmarkBeta,
    HhtAlpha,
}

impl Default for AnalysisSettings {
    fn default() -> Self {
        Self {
            n_modes: 3,
            seismic_dir: SeismicDir::X,
            // 既定は略算周期 T = h(0.02+0.01α)（令88条・昭55建告1793号）。
            // 固有値解析を要しないため、地震荷重の同期が暗黙の解析を伴わない。
            // 精算（SemiPrecise）は固有値解析の明示実行を前提とするオプトインで、
            // 必要な場合に UI（解析タブ「T算定」）で選択する。
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
            ductility_method: squid_n_solver::pushover::DuctilityMethod::default(),
            push_control: squid_n_solver::pushover::PushoverControl::default(),
            push_apply_long_term: true,
            lumped_mass_type: squid_n_solver::lumped_mass::LumpedMassType::default(),
            lumped_secant_ratio: 0.75,
            th_damping: 0.02,
            th_dt: 0.01,
            th_duration: 10.0,
            th_period: 0.5,
            th_amp: 1000.0,
            th_dir: ThDir::X,
            th_damping_model: ThDampingModel::StiffnessProportional,
            th_h2: 0.02,
            th_integrator: ThIntegrator::NewmarkBeta,
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
        }
    }
}
