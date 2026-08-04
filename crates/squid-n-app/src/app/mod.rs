use std::time::SystemTime;

use squid_n_core::ids::{ElemId, LoadCaseId, NodeId, SectionId};
use squid_n_design_jp::{DesignCheck, DesignCtx, LoadTerm, MemberForcesAt};
use squid_n_edit::UndoStack;
use squid_n_solver::analysis::{AiMode, Analysis, SeismicDir};

/// 工程タブ（UI設計 §1.1）。進行ロックしない。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Tab {
    #[default]
    Model,
    Loads,
    Analysis,
    Results,
    Design,
    Report,
}

/// モデルタブ内のサブタブ。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ModelTab {
    #[default]
    Nodes,
    BoundaryConditions,
    Members,
    Sections,
    Materials,
    Slabs,
    /// 壁属性（開口・三方スリット）
    WallAttrs,
    /// フレーム外雑壁
    MiscWalls,
    /// 部材付帯情報（ハンチ・継手位置）
    MemberDetails,
    /// S造検定属性（継手・スカラップ欠損率、横座屈長さ・座屈長さの直接入力）
    SteelAttrs,
    /// 通り芯（各通りの呼称。構造計算には用いない）
    Axes,
}

/// 下ドックのタブ。ログに加え、横幅を要する編集テーブルを収容する
/// （横長テーブルは幅の狭い左ドックより下ドックの方が視認性が良い）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum BottomTab {
    #[default]
    Log,
    /// モデル編集テーブル（節点・部材・断面…の ModelTab サブタブ群）
    Model,
    /// 荷重編集テーブル
    Loads,
    /// モデル整合性チェック（診断）一覧
    Diagnostics,
    /// 準備計算の結果（階の分布・剛域・Ai 分布・風圧力・荷重集計）
    Preparation,
}

/// 左ドックのパネル。Zed のように下部バーのアイコンで切り替える。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum LeftPanel {
    #[default]
    Navigator,
    /// 作成パレット（梁・壁・スラブ作成モードと断面割当）
    DrawTools,
}

/// 右ドックのパネル。Zed のように下部バーのアイコンで切り替える。
///
/// 一貫計算の手順（① 解析入力を確定する準備計算 → ② 解く）はそれぞれ扱う情報量が
/// 多いため、独立したパネルに分ける（1 枚に積むと縦に長くなり、いま何をしている
/// 段階なのかが読み取りにくい）。どちらのパネルも 3D を見ながら設定・実行できる
/// よう右ドックに置く。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum RightPanel {
    #[default]
    Inspector,
    /// ① 準備計算（地震力・風圧力の算定諸元・計算条件・階の定義と、その実行）
    Preparation,
    /// ② 解析（静的解析・固有値・増分解析・時刻歴応答の実行）
    Analysis,
}

/// 結果タブ内の切替（3D 各種図・時刻歴グラフ・増分解析曲線）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ResultsView {
    #[default]
    Spatial,
    TimeHistory,
    Pushover,
    /// 質点系（串団子）モデル（増分解析から生成）。
    LumpedMass,
}

/// 設計タブ内の切替（検定表・終局検定表・MN相関曲面ビュー・数量積算）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum DesignView {
    #[default]
    Table,
    /// 終局検定（靭性保証型耐震設計指針による終局せん断・付着余裕度）。
    Ultimate,
    MnSurface,
    /// 数量積算（部位別の概算数量）。
    Quantities,
}

/// ナビゲータ（左ペイン）。階/部材群/ケース系の選択状態。
#[derive(Default)]
pub struct Navigator {
    pub expanded_floors: bool,
    pub expanded_groups: bool,
    pub expanded_load_cases: bool,
    pub expanded_result_cases: bool,
    pub focus_node: Option<NodeId>,
    pub focus_member: Option<ElemId>,
    pub focus_section: Option<SectionId>,
    pub focus_load_case: Option<LoadCaseId>,
    /// ナビゲータで選択中の結果表示対象（静的ケース／荷重組合せ）
    pub focus_result: Option<StaticKey>,
}

/// 静的解析結果の格納キー。ユーザー荷重ケースと地震静的(Ai)を型で区別し、
/// LoadCaseId(0) の二重使用(ユーザーケース0と地震結果の同居)を解消する。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum StaticCaseKey {
    /// ユーザー定義の荷重ケース
    User(LoadCaseId),
    /// 地震静的(Ai 分布)。方向別に共存できる
    Seismic(SeismicDir),
    /// 風荷重静的解析。方向別に共存できる
    Wind(SeismicDir),
}

/// 静的解析の実行対象（「単体実行」で選ぶ対象）。
///
/// 荷重ケース単体と荷重組合せは、いずれも同じ導線（単体実行／一括解析）から
/// 実行する。組合せの求解は荷重ケース単体の解析結果の線形和として行うため
/// （`Analysis::linear_combination`）、両者は「解析の実行対象」として同列に扱える。
///
/// `Combo` のインデックスは **`Model::combinations` 上の位置**
/// （結果側の位置を指す [`StaticKey::Combo`] とは別）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StaticTarget {
    Case(LoadCaseId),
    Combo(usize),
}

/// 表示対象の静的解析結果を指すキー。荷重ケース単体（ユーザー／地震静的）か
/// 荷重組合せかを区別する。
///
/// `Case` は `StaticCaseKey` をそのままネストする形を採る（`User`/`Seismic`/`Combo`
/// にフラット化する案もあったが、`current_static` やナビゲータでの
/// `bundle.statics` 引き当ては User/Seismic を区別する必要がなく
/// `StaticCaseKey` の等値比較 1 本で完結するため、ネストの方が呼び出し側の
/// 分岐が増えずシンプルに書ける）。
///
/// `Combo` のインデックスは **`ResultsBundle.combos` 上の位置**
/// （`model.combinations` のインデックスではない）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum StaticKey {
    Case(StaticCaseKey),
    Combo(usize),
}

/// stale（要再計算）状態（UI設計 §5）。
#[derive(Clone, Debug)]
pub struct Staleness {
    pub results_stale: bool,
    pub design_stale: bool,
    pub last_run: Option<SystemTime>,
    /// ファイル保存後に編集があったか（タイトル/ステータスの未保存マーカー用）。
    /// `mark_fresh`（解析完了）ではクリアされず、保存/読込時のみクリアする。
    pub unsaved_changes: bool,
    /// 診断（モデル整合性チェック）が未実行または編集後で古いか。
    /// `Default` では「まだ一度も実行していない」ことを表すため true とする
    /// （他フィールドと異なり、モデル新規作成・読込直後にも診断タブを開いた
    /// 時点で必ず一度実行させたいため）。
    pub diagnostics_stale: bool,
    /// 準備計算（剛域・荷重同期・Ai 分布の確定）が未実行または編集後で古いか。
    /// `diagnostics_stale` と同じ理由で `Default` は true（未実行）とする。
    /// 解析の実行前に `ensure_preparation` が参照する。
    pub preparation_stale: bool,
}

impl Default for Staleness {
    fn default() -> Self {
        Self {
            results_stale: false,
            design_stale: false,
            last_run: None,
            unsaved_changes: false,
            diagnostics_stale: true,
            preparation_stale: true,
        }
    }
}

impl Staleness {
    /// モデル/荷重が編集された → 下流を stale にする。
    pub fn mark_edited(&mut self) {
        self.results_stale = true;
        self.design_stale = true;
        self.unsaved_changes = true;
        self.diagnostics_stale = true;
        self.preparation_stale = true;
    }
    /// 計算に用いないモデルデータ（通り芯）が編集された → 未保存フラグだけ立てる。
    ///
    /// 通り芯は識別のための呼称であり、応力・断面算定・保有耐力のいずれにも入らない。
    /// [`Staleness::mark_edited`] を使うと通り芯を 1 本足しただけで解析結果・設計結果・
    /// 準備計算が陳腐化してしまうため、保存が必要なことだけを記録する。
    pub fn mark_non_calc_edited(&mut self) {
        self.unsaved_changes = true;
    }

    /// 解析が完了 → 最新化する。
    pub fn mark_fresh(&mut self) {
        self.results_stale = false;
        self.design_stale = false;
        self.last_run = Some(SystemTime::now());
    }
}

/// 診断の重要度。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DiagSeverity {
    Error,
    Warning,
    Info,
}

/// 診断が指す対象（クリックで 3D 選択・インスペクタへ反映するために持つ）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DiagTarget {
    Node(NodeId),
    Member(ElemId),
}

/// モデル整合性チェックの結果1件。
#[derive(Clone, Debug)]
pub struct Diagnostic {
    pub severity: DiagSeverity,
    pub message: String,
    pub target: Option<DiagTarget>,
}

#[derive(Default)]
pub struct Selection {
    pub nodes: Vec<squid_n_core::ids::NodeId>,
    pub members: Vec<squid_n_core::ids::ElemId>,
}

/// 床の中での小梁設計結果1件（`(スラブ id, 小梁インデックス, 設計結果)`）。
pub type JoistCheck = (
    squid_n_core::ids::SlabId,
    usize,
    squid_n_design_jp::floor::JoistDesignResult,
);
/// スラブ（床）設計結果1件（`(スラブ id, 設計結果)`）。
pub type SlabCheck = (
    squid_n_core::ids::SlabId,
    squid_n_design_jp::floor::SlabDesignResult,
);

/// 1 検定位置の結果（部材内の位置 `xi` と検定結果/検定不能）。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PositionCheck {
    /// 部材軸方向の無次元位置 (0.0=始端, 1.0=終端)。
    pub xi: f64,
    pub outcome: squid_n_design_jp::CheckOutcome,
}

/// 1 部材分の断面検定結果（検定位置の列。`positions` は `xi` 昇順）。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct MemberChecks {
    pub elem: ElemId,
    pub positions: Vec<PositionCheck>,
}

/// 節点単位の検定（柱梁接合部・パネルゾーン・冷間成形耐力比・耐震壁など）。
/// `label` は「接合部(RC)」等の種別表示用。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct JointCheck {
    pub node: squid_n_core::ids::NodeId,
    pub label: String,
    pub outcome: squid_n_design_jp::CheckOutcome,
}

/// フラットな `(部材, 位置, 検定結果)` の列を部材単位にグループ化し、部材内を
/// `xi` 昇順に並べ替える（`run_design_check` での検定結果組み立てに使う）。
/// 部材の出現順は入力列での初出順を保つ。
pub(crate) fn group_member_checks(
    flat: Vec<(ElemId, f64, squid_n_design_jp::CheckOutcome)>,
) -> Vec<MemberChecks> {
    let mut order: Vec<ElemId> = Vec::new();
    let mut by_elem: std::collections::HashMap<ElemId, Vec<PositionCheck>> =
        std::collections::HashMap::new();
    for (elem, xi, outcome) in flat {
        by_elem
            .entry(elem)
            .or_insert_with(|| {
                order.push(elem);
                Vec::new()
            })
            .push(PositionCheck { xi, outcome });
    }
    order
        .into_iter()
        .map(|elem| {
            let mut positions = by_elem.remove(&elem).unwrap_or_default();
            positions.sort_by(|a, b| a.xi.partial_cmp(&b.xi).unwrap_or(std::cmp::Ordering::Equal));
            MemberChecks { elem, positions }
        })
        .collect()
}

/// プロジェクトファイル（.scz）へ保存する解析結果一式。
///
/// 結果本体（[`ResultsBundle`]）に加えて、復元後に結果タブ・設計タブが
/// 保存時と同じ表示になるよう表示状態（`last_static`）と最終実行時刻も持つ。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SavedResults {
    pub bundle: ResultsBundle,
    /// 表示対象の静的解析結果。
    pub last_static: Option<StaticKey>,
    /// 解析の最終実行時刻。
    pub last_run: Option<SystemTime>,
}

#[derive(Default, Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ResultsBundle {
    pub statics: Vec<(StaticCaseKey, squid_n_solver::linear::StaticOnce)>,
    /// 荷重組合せの解析結果（組合せ名で保持）
    pub combos: Vec<(String, squid_n_solver::linear::StaticOnce)>,
    pub modal: Option<squid_n_solver::eigen::ModalResult>,
    pub member_forces: Vec<(ElemId, squid_n_element::beam::MemberForces)>,
    /// 仕口パネルのせん断モーメント `{MSX, MSY}` [N·mm]（接合部の節点ごと）。
    /// `member_forces` と同じく、直近に表示している静的結果のものを保持する。
    /// S 造パネルゾーン検定の設計用パネルモーメント `pM` に用いる。
    #[serde(default)]
    pub panel_moments: Vec<(squid_n_core::ids::NodeId, [f64; 2])>,
    /// 部材単位の断面検定結果（部材ごとに検定位置をグループ化）。
    pub member_checks: Vec<MemberChecks>,
    /// 節点単位の検定結果（柱梁接合部・パネルゾーン・冷間成形耐力比など）。
    pub joint_checks: Vec<JointCheck>,
    /// 床の中での小梁設計（単純支持梁）。実部材化された小梁は全体 FEM で検定する
    /// ためここには含めない。
    pub joist_checks: Vec<JoistCheck>,
    /// スラブ（床）の設計（一方向曲げ）。
    pub slab_checks: Vec<SlabCheck>,
    pub pushover: Option<squid_n_solver::pushover::PushoverResult>,
    pub time_history: Option<squid_n_solver::timehistory::ResponseResult>,
}

/// 解析タブの設定値（GUI 非依存。テストからも使う）。
#[derive(Clone, Copy, Debug)]
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
    /// 長期荷重ケースが無いモデルでは無視される（ソルバ側の対応実装に依存）。
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
    /// 同 δ2（暴風時 G+P+δ2・S±W。既定 0.35）。
    pub snow_delta2: f64,
    /// 同 δ3（地震時 G+P+δ3・S±K。既定 0.35）。
    pub snow_delta3: f64,
    /// RC 短期許容せん断力の「損傷制御のための検討」（false=安全確保のための検討）。
    /// RC規準・令82条（断面算定条件 RC造）に対応。
    pub rc_damage_control: bool,
    /// 地震時短期の設計用せん断力 QD の決定方法（QD1/QD2/min）。
    pub qd_method: squid_n_design_jp::QdMethod,
    /// 風荷重静的解析の基準風速 V0 [m/s]。
    pub v0: f64,
    /// 風荷重静的解析の地表面粗度区分。
    pub roughness: squid_n_load::wind::TerrainRoughness,
    /// 風荷重静的解析のパラペット高さ [mm]。
    pub parapet_mm: f64,
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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ThDir {
    X,
    Y,
    Xy,
}

/// 時刻歴の減衰モデル選択（UI 用）。構造動力学の減衰マトリクス。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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
            snow_delta2: 0.35,
            snow_delta3: 0.35,
            rc_damage_control: true,
            qd_method: squid_n_design_jp::QdMethod::Min,
            v0: 34.0,
            roughness: squid_n_load::wind::TerrainRoughness::III,
            parapet_mm: 0.0,
            threads: 0,
            mass_method: squid_n_core::model::MassMethod::default(),
        }
    }
}

/// 一括解析（`App::run_static_all`）の計算結果。荷重ケース単体と、その線形和で
/// 得た荷重組合せの両方を運ぶ。
#[derive(Default)]
pub struct StaticAllComputed {
    /// 荷重ケース単体の結果（格納キーと結果）。
    pub cases: Vec<(
        StaticCaseKey,
        Result<squid_n_solver::linear::StaticOnce, String>,
    )>,
    /// 荷重組合せの結果（組合せ名と結果）。
    pub combos: Vec<(String, Result<squid_n_solver::linear::StaticOnce, String>)>,
}

/// バックグラウンド解析ジョブ（増分解析／時刻歴／静的解析の単体実行・一括解析）が
/// 送る結果。
pub enum JobResult {
    Pushover(Result<squid_n_solver::pushover::PushoverResult, String>),
    /// 固有値解析（モード数は起動時の `analysis_cfg.n_modes`）。
    Modal(Result<squid_n_solver::eigen::ModalResult, String>),
    /// 時刻歴応答解析。`ResponseResult` は詳細記録を含み大きいため Box で運ぶ。
    TimeHistory(Box<Result<squid_n_solver::timehistory::ResponseResult, String>>),
    /// 線形静的・地震静的(Ai)・風荷重静的解析（`StaticCaseKey` で結果格納先を区別）。
    StaticCase {
        key: StaticCaseKey,
        res: Result<squid_n_solver::linear::StaticOnce, String>,
    },
    /// 単一の荷重組合せ解析（`bundle.combos` の名前一致検索で格納位置を決める）。
    Combo {
        name: String,
        res: Result<squid_n_solver::linear::StaticOnce, String>,
    },
    /// 一括解析（全荷重ケース単体＋全荷重組合せ）。`computed` は
    /// `Analysis::prepare` 失敗時（全件アボート）と個別解析結果の両方を運ぶ。
    /// `pre_errors` は UI スレッドで事前フィルタした「空の水平力ケース参照」等の
    /// エラーメッセージ。
    StaticAll {
        computed: Result<StaticAllComputed, String>,
        pre_errors: Vec<String>,
    },
}

/// バックグラウンド解析ジョブ。重い解析(増分解析・時刻歴・静的解析の単体実行と
/// 一括解析)を UI スレッドから逃がす(P8 §5)。
/// 結果は poll_job で受け取り適用する。
pub struct AnalysisJob {
    pub label: &'static str,
    pub started: std::time::SystemTime,
    rx: std::sync::mpsc::Receiver<JobResult>,
    /// ジョブ成功時に自動遷移する結果タブ・表示切替（GUI 専用）。
    #[cfg(feature = "gui")]
    pub jump_on_success: Option<(Tab, ResultsView)>,
}

/// ログの重要度。下ドック（ログパネル）での色分けに使う。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogLevel {
    Info,
    Notice,
    Error,
}

/// セッション内イベントログの1件。時刻はアプリ起動からの経過時間で持つ
/// （std のみでは壁時計のローカル時刻表記ができないため。表示は「mm:ss」）。
pub struct LogEntry {
    pub elapsed: std::time::Duration,
    pub level: LogLevel,
    pub message: String,
}

impl LogEntry {
    /// 表示用の経過時間文字列（「mm:ss」。60分を超えても分側の桁が増えるだけでよい）。
    pub fn timestamp_label(&self) -> String {
        let secs = self.elapsed.as_secs();
        format!("{:02}:{:02}", secs / 60, secs % 60)
    }
}

/// セッション内イベントログ。下ドック（ログパネル）に表示する。
pub struct EventLog {
    started: std::time::Instant,
    pub entries: Vec<LogEntry>,
}

impl Default for EventLog {
    fn default() -> Self {
        Self {
            started: std::time::Instant::now(),
            entries: Vec::new(),
        }
    }
}

impl EventLog {
    /// 保持件数の上限。無制限に溜め続けるとメモリと描画コストが増え続けるため、
    /// 上限を超えたら古いものから捨てて直近の履歴のみ保持する。
    const MAX_ENTRIES: usize = 1000;

    pub fn push(&mut self, level: LogLevel, message: impl Into<String>) {
        self.entries.push(LogEntry {
            elapsed: self.started.elapsed(),
            level,
            message: message.into(),
        });
        if self.entries.len() > Self::MAX_ENTRIES {
            let overflow = self.entries.len() - Self::MAX_ENTRIES;
            self.entries.drain(..overflow);
        }
    }
}

pub struct App {
    pub model: squid_n_core::model::Model,
    pub results: Option<ResultsBundle>,
    pub selection: Selection,
    pub undo: UndoStack,
    pub active_tab: Tab,
    /// 設計検定の荷重継続性区分（長期／短期）
    pub design_term: LoadTerm,
    /// 最後に実行した静的解析結果（荷重ケース／荷重組合せ）
    pub last_static: Option<StaticKey>,
    /// 解析実行中のエラーメッセージ
    pub last_error: Option<String>,
    /// 解析実行中の注意メッセージ（エラーではないが利用者に知らせたい事項。
    /// 例: 精算周期(SemiPrecise)選択時に固有値解析が未実行で EX/EY の地震荷重が
    /// 更新されなかった旨）。`last_error`（赤）とは別枠で情報色表示する。
    pub last_notice: Option<String>,
    /// セッション内イベントログ（下ドックのログパネルに表示）。
    pub log: EventLog,
    /// 実行中のバックグラウンド解析ジョブ（増分解析・時刻歴・静的解析の単体実行と
    /// 一括解析、P8 §5）。
    /// 完了は `poll_job` で検知して結果を適用する。
    pub job: Option<AnalysisJob>,
    /// 節点座標の編集バッファ（model.nodes に同期）
    pub node_edit: Vec<[String; 3]>,
    /// 節点テーブルのグリッド操作ウィジェット状態（選択・編集モード・フラッシュ等）
    #[cfg(feature = "gui")]
    pub node_grid: crate::grid::GridWidget,
    /// 節点追加フォームの入力中座標（境界条件の編集とは別の独立 UI）
    pub node_draft: [String; 3],
    /// 節点追加時に既存節点と同一座標だった場合の追加保留座標。
    /// セットされている間は確認ダイアログを表示し、ユーザの判断を待つ。
    pub pending_duplicate_node_coord: Option<[f64; 3]>,
    /// 保存サイズ超過時の保存保留（保存先パス, 解析結果の直列化サイズ [MB]）。
    /// セットされている間は「時刻歴の詳細記録を保存に含めますか？」の確認
    /// ダイアログを表示し、選択に応じて含めて保存／除外して保存／キャンセル。
    pub pending_save_recording: Option<(std::path::PathBuf, u64)>,
    /// stale（要再計算）状態と最終実行時刻
    pub staleness: Staleness,
    /// モデル整合性チェック（診断）の結果一覧。`run_diagnostics` で再構築する。
    pub diagnostics: Vec<Diagnostic>,
    /// 準備計算（解析前の前処理）の結果。解析前に階の分布・剛域・Ai 分布・
    /// 風圧力・荷重集計を確認するために保持する。`run_preparation`／
    /// 各解析実行時の `ensure_preparation` で再構築する。
    pub preparation: Option<PreparationResult>,
    /// ナビゲータ（左ペイン）状態
    pub nav: Navigator,
    /// モデルタブ内のサブタブ
    pub model_tab: ModelTab,
    /// 保有水平耐力（ルート3）判定の架構種別（Ds 表の行選択）
    pub design_frame: squid_n_design_jp::secondary::holding_capacity::FrameType,
    /// 保有水平耐力（ルート3）判定の部材ランク（Ds 表の列選択）。
    /// `design_rank_auto == true` の場合はフォールバック用（幅厚比を算定できない
    /// 層のみに適用される）。
    pub design_rank: squid_n_design_jp::secondary::holding_capacity::MemberRank,
    /// 保有水平耐力（ルート3）の部材ランクを鋼部材の幅厚比から自動判定するか（UI-13）。
    /// true の場合、鋼部材かつ断面形状(`Section.shape`)を持つ部材について
    /// 構造規定の幅厚比表（`steel_width_thickness_rank`）で算定し、
    /// 算定できなかった層のみ `design_rank`（選択値）にフォールバックする。
    pub design_rank_auto: bool,
    /// 耐力壁の種別（WA〜WD）判定で**壁式構造**の列を用いるか。
    /// 告示「耐力壁の種別」表は壁式構造で限界値が厳しい（τu/Fc: WA 0.1・WB 0.125・
    /// WC 0.15）。既定は false（壁式構造以外: WA 0.20・WB 0.25）。
    pub wall_structure: bool,
    /// 直近の保有水平耐力算定で用いた層別 βu（耐力壁・筋かいの水平耐力比）。
    /// `compute_holding_capacity` が設定する（表示用）。
    pub ds_beta_u_by_story: Vec<f64>,
    /// 架構種別が耐力壁付き／筋かい付きなのに耐力壁・筋かいを検出できず、βu を
    /// 算定できなかったため架構種別別 Ds 表へフォールバックしたか（表示用）。
    pub ds_beta_u_unavailable: bool,
    /// 部材ランク自動判定（`design_rank_auto`）で 1 本も算定できず、選択ランク
    /// （`design_rank`）へフォールバックした層の名前（下階→上階順）。
    /// `compute_holding_capacity` が設定する（表示用）。選択ランク（既定 FA）が
    /// 実状より甘い場合に Ds を過小評価する危険側となるため、設計タブで警告する。
    pub ds_rank_fallback_stories: Vec<String>,
    /// 終局検定（靭性保証型耐震設計指針）のヒンジ回転角 Rp [rad]（ν・cotφ 用。既定 0）。
    pub ultimate_rp: f64,
    /// 終局検定で軽量コンクリートのせん断終局耐力 0.9 倍低減を適用するか。
    pub ultimate_lightweight: bool,
    /// 終局検定で付着割裂耐力 Qbu の余裕度を算定するか。
    pub ultimate_include_bond: bool,
    /// 終局検定の上限強度倍率（Qmu = 上限強度倍率·2·Mu/内法。既定 1.0）。
    pub ultimate_upper_factor: f64,
    /// 終局検定で柱の Mu を ACI 規準（平面保持）で算定するか（false は at 式）。
    pub ultimate_mu_aci: bool,
    /// 終局検定の終局せん断強度に靭性指針式 Vu を用いるか（false=塑性理論式 Qsu）。
    pub ultimate_shear_ductility: bool,
    /// 終局検定で柱のせん断を 2 軸せん断として検定するか（RC 柱の 2 軸せん断余裕度）。
    pub ultimate_biaxial_shear: bool,
    /// 終局検定で柱の曲げを 2 軸曲げとして検定するか（RC 柱の 2 軸曲げ余裕度）。
    pub ultimate_biaxial_bending: bool,
    /// 終局検定の設計用応力（Qmu・需要曲げ）と部材別 Rp を増分解析応答から
    /// 直接反映するか（false は静的解析応答＋UI 一律 Rp）。増分解析未実行時は
    /// 自動的に静的応答へフォールバックする。
    pub ultimate_use_pushover: bool,
    /// 左ドック（ナビゲータ／編集パネル）の表示状態
    #[cfg(feature = "gui")]
    pub left_dock_open: bool,
    /// 左ドックの表示パネル（ナビゲータ／作成パレット）
    #[cfg(feature = "gui")]
    pub left_panel: LeftPanel,
    /// 右ドック（インスペクタ／準備計算／解析）の表示状態
    #[cfg(feature = "gui")]
    pub right_dock_open: bool,
    /// 右ドックの表示パネル（インスペクタ／準備計算／解析）
    #[cfg(feature = "gui")]
    pub right_panel: RightPanel,
    /// 下ドック（ログ／編集テーブル）の表示状態。既定で開き、イベントログを
    /// 起動直後から見えるようにする（処理の経過が常時追える Zed のターミナル相当）。
    #[cfg(feature = "gui")]
    pub bottom_dock_open: bool,
    /// 下ドックの表示タブ（ログ／モデル編集／荷重編集）
    #[cfg(feature = "gui")]
    pub bottom_tab: BottomTab,
    /// 結果タブ内の表示切替（3D / 時刻歴）
    #[cfg(feature = "gui")]
    pub results_view: ResultsView,
    /// 設計タブ内の表示切替（検定表 / MN相関曲面）
    #[cfg(feature = "gui")]
    pub design_view: DesignView,
    /// MN 相関曲面ビューの状態（断面選択・材料強度・表示切替・カメラ等）
    #[cfg(feature = "gui")]
    pub mn_view: crate::mn_view::MnViewState,
    /// ビューアの表示モード
    #[cfg(feature = "gui")]
    pub view_mode: crate::viewer::ViewMode,
    /// 応力図で表示中の成分（N/Qy/Qz/Mx/My/Mz の ON/OFF。複数同時表示）
    #[cfg(feature = "gui")]
    pub force_components: crate::viewer::ForceComponents,
    /// 応力図に数値ラベル（両端部・中央の値）を表示するか
    #[cfg(feature = "gui")]
    pub diagram_values: bool,
    /// CMQ 図で表示する成分（C: モーメント／Q: せん断）
    #[cfg(feature = "gui")]
    pub cmq_component: crate::viewer::CmqComponent,
    /// 検定比図の着色対象（最大＝全式の max、または特定の検定式のみ）
    #[cfg(feature = "gui")]
    pub check_ratio_filter: crate::viewer::CheckRatioFilter,
    /// 検定比図で検定位置ごとの正方形マーカーを表示するか
    #[cfg(feature = "gui")]
    pub check_ratio_markers: bool,
    /// 検定比図の数値ラベルを全部材に表示するか（false=注意域以上のみ。既定）
    #[cfg(feature = "gui")]
    pub check_ratio_label_all: bool,
    /// ヒンジ図でクリック選択された部材（ヒンジ詳細ウィンドウの表示対象。
    /// `None` はウィンドウ非表示）。
    #[cfg(feature = "gui")]
    pub hinge_detail_elem: Option<squid_n_core::ids::ElemId>,
    /// ヒンジ詳細ウィンドウの N-M 相関曲線キャッシュ（断面のファイバー分割・
    /// 曲面構築は数十msかかりうるため、選択部材が変わらない限り再計算しない）。
    #[cfg(feature = "gui")]
    pub hinge_mn_cache: Option<crate::viewer::hinge::MnCurveCache>,
    /// ヒンジ詳細ウィンドウの N-M 相関図（3D ワイヤーフレーム）用カメラ状態
    /// （`viewer::CameraState` を再利用し、断面詳細ビューと同じ操作感にする）。
    #[cfg(feature = "gui")]
    pub hinge_mn_camera: crate::viewer::CameraState,
    /// N/Q/M 図の表示切替（false=単色塗り／true=値に応じたコンター色分け）
    #[cfg(feature = "gui")]
    pub diagram_contour: bool,
    /// コンター表示のカラーマップ（既定は TONMANUAL §3 準拠の Viridis）
    #[cfg(feature = "gui")]
    pub contour_colormap: crate::theme::ColorMap,
    /// N/Q/M 図で変形図を重ねて表示するか（応力と変形を同時に確認する）
    #[cfg(feature = "gui")]
    pub overlay_deform: bool,
    /// 床（スラブ・小梁）と二次部材の表示（変形図で解析対象外の要素を隠せる）
    #[cfg(feature = "gui")]
    pub show_floor_secondary: bool,
    /// モード形の表示インデックス
    #[cfg(feature = "gui")]
    pub view_mode_idx: usize,
    /// ビューアのカメラ状態
    #[cfg(feature = "gui")]
    pub camera: crate::viewer::CameraState,
    /// 2D 構面表示の対象（`None` は全体表示＝従来の 3D ビュー）。
    /// 通り芯の再生成・モデルの入れ替えで添字がずれるため、描画のたびに
    /// 実在を検証し、解決できなければ全体表示へ戻す。
    #[cfg(feature = "gui")]
    pub frame_target: Option<squid_n_core::frame::FrameTarget>,
    /// ビューアの断面表示（部材を断面形状の押し出しソリッドで立体表示する）
    #[cfg(feature = "gui")]
    pub show_sections: bool,
    /// 変形図の表示倍率の手動係数（自動算定倍率への乗数、既定 1.0）。
    /// 自動倍率（バウンディングボックスと梁スパンから決定）にこの係数を掛けた
    /// 値が実効倍率になる。スライダーで拡大・縮小できる。
    #[cfg(feature = "gui")]
    pub deform_scale_factor: f32,
    /// 剛床代表節点（重心マスター）の可視化トグル（既定 OFF）。
    /// ON にすると代表点マーカー・面内拘束マーク・関連スレーブ節点への点線を描く。
    /// 点線は節点数が多いと他部材が見づらくなるため、既定は非表示にしている。
    #[cfg(feature = "gui")]
    pub show_diaphragm_master: bool,
    /// モデル化図で可視化する解析種別（静解析＝弾性／増分解析＝弾塑性）。
    /// 解析種別によって部材のモデル化（要素定式化）が変わるため切り替える。
    #[cfg(feature = "gui")]
    pub modeling_analysis: crate::viewer::ModelingAnalysis,
    /// 変形図・応力図の変形重ねで、梁を内部たわみ（Hermite 3 次曲線）で描くか
    /// （既定 ON）。ON では梁の内部たわみと、それに載る床・二次部材の追従を
    /// 曲線で表示する。OFF では梁を節点間の直線（弦）で描き、床・二次部材の追従も
    /// 線形補間にする（全体の変形を素直に見る用）。変位図だけでなく N/Q/M 図の
    /// 変形重ねにも適用する。
    #[cfg(feature = "gui")]
    pub show_beam_interpolation: bool,
    /// 時刻歴モード（[`crate::viewer::ViewMode::TimeHistory`]）の現在フレーム番号
    /// （`ThRecording::frame_time` の添字）。
    #[cfg(feature = "gui")]
    pub th_frame: usize,
    /// 時刻歴モードの再生中フラグ（ON でフレームを自動で進める）。
    #[cfg(feature = "gui")]
    pub th_playing: bool,
    /// 時刻歴モードの再生速度倍率（×0.25〜×2 等）。
    #[cfg(feature = "gui")]
    pub th_speed: f32,
    /// 時刻歴モードの再生経過時刻 [s]（`ThRecording::frame_time` に基づき現在
    /// フレームを決定する。スライダー操作時は選択フレームの時刻に同期する）。
    #[cfg(feature = "gui")]
    pub th_play_time: f64,
    /// 時刻歴モードでクリック選択された部材（履歴・検定ウィンドウの表示対象。
    /// `None` はウィンドウ非表示）。
    #[cfg(feature = "gui")]
    pub th_detail_elem: Option<squid_n_core::ids::ElemId>,
    /// 時刻歴詳細ウィンドウの梁・柱ループで表示する曲げ軸（true=強軸Mz／false=弱軸My）。
    #[cfg(feature = "gui")]
    pub th_detail_axis_z: bool,
    /// 時刻歴詳細ウィンドウの零長要素（免震・節点ばね）N-δ ループで選択中の成分
    /// （中-2）。`(部材, 成分)` を保持し、`th_detail_elem` と部材が一致しない場合は
    /// 要素種別ごとの既定（免震＝せん断、それ以外＝軸）へ戻す。`None` は未選択。
    #[cfg(feature = "gui")]
    pub th_detail_axial_component: Option<(
        squid_n_core::ids::ElemId,
        crate::viewer::th_detail::AxialComponent,
    )>,
    /// 時刻歴アニメーション（[`crate::viewer::ViewMode::TimeHistory`]）の変形倍率
    /// キャッシュ（高-2）。フレームごとに再正規化せず、記録全体のピーク変位から
    /// 1 回だけ算定した自動倍率（手動係数を掛ける前の値）を保持する。
    #[cfg(feature = "gui")]
    pub th_scale_cache: Option<crate::viewer::TimeHistoryScaleCache>,
    /// 床荷重分配の CMQ 結果（P2 §5.1）。描画用。
    pub beam_loads: Vec<squid_n_load::floor::BeamLoad>,
    /// `beam_loads` を算定した時点のモデル・設定ハッシュ
    /// （`compute_auto_load_sync_hash`）。CMQ 図表示中は毎フレーム
    /// `refresh_beam_loads` が呼ばれるため、モデルが変わっていなければ
    /// 床荷重分配（交差小梁スラブでは床格子サブ FEM 解析を含む）を
    /// スキップするためのキャッシュキー。`None` は未算定。
    pub beam_loads_hash: Option<u64>,
    /// 時刻歴応答データ（描画用）
    #[cfg(feature = "gui")]
    pub time_history_data: crate::time_history_view::TimeHistoryData,
    /// 時刻歴グラフの表示項目選択
    #[cfg(feature = "gui")]
    pub time_history_source: crate::time_history_view::TimeHistorySource,
    /// 時刻歴結果タブの表示モード（時刻歴波形／層応答分布）
    #[cfg(feature = "gui")]
    pub time_history_view_mode: crate::time_history_view::TimeHistoryViewMode,
    /// 層応答分布グラフの表示項目選択
    #[cfg(feature = "gui")]
    pub story_response_kind: crate::story_response::StoryResponseKind,
    /// 層応答分布グラフの方向選択（記録済みの X・Y いずれか）
    #[cfg(feature = "gui")]
    pub story_response_dir: crate::story_response::StoryRespDir,
    /// 質点系（串団子）時刻歴応答の結果（結果タブ「質点系モデル」で実行・表示）。
    pub stick_response: Option<squid_n_solver::lumped_mass::StickResponse>,
    /// 断面作成UI のドラフト（UI-3）
    #[cfg(feature = "gui")]
    pub section_draft: crate::section_editor::SectionEditorDraft,
    /// 断面カタログ選択UI のドラフト（Shape→Family→Name）
    #[cfg(feature = "gui")]
    pub catalog_draft: crate::section_editor::CatalogDraft,
    /// 境界条件タブ「免震支承の配置」フォームのドラフト状態（`PlaceSupportIsolator` 用）
    #[cfg(feature = "gui")]
    pub isolator_support_draft: crate::tables::nodes::IsolatorSupportDraft,
    /// 部材タブ「免震支承材を追加」フォームのドラフト状態（`AddIsolator` 用）
    #[cfg(feature = "gui")]
    pub isolator_member_draft: crate::tables::members::IsolatorMemberDraft,
    /// 断面タブ「制振要素」パネルのドラフト状態（`AddDamperDef`/`UpdateDamperDef` 用）
    #[cfg(feature = "gui")]
    pub damper_def_draft: crate::damper_def_editor::DamperDefDraft,
    /// ビューアの梁作成モード（ON 中はクリックで節点を選び 2 点で梁を作る）
    #[cfg(feature = "gui")]
    pub beam_draw_mode: bool,
    /// 梁作成モードで選択済みの始点節点（2 点目で梁を生成しリセット）
    #[cfg(feature = "gui")]
    pub beam_draw_first: Option<squid_n_core::ids::NodeId>,
    /// ビューアの壁作成モード（ON 中はクリックで節点を選び 4 点で壁を作る）
    #[cfg(feature = "gui")]
    pub wall_draw_mode: bool,
    /// 壁作成モードで選択済みの節点（4 点目で壁を生成しリセット）
    #[cfg(feature = "gui")]
    pub wall_draw_nodes: Vec<squid_n_core::ids::NodeId>,
    /// ビューアのスラブ作成モード（ON 中はクリックで境界節点を順に選び、
    /// 「確定」で 3〜N 節点のスラブを作る）
    #[cfg(feature = "gui")]
    pub slab_draw_mode: bool,
    /// スラブ作成モードで選択済みの境界節点（外周順。確定で AddSlab しリセット）
    #[cfg(feature = "gui")]
    pub slab_draw_nodes: Vec<squid_n_core::ids::NodeId>,
    /// 現在のプロジェクトファイル（.scz）パス。未保存なら None。
    pub project_path: Option<std::path::PathBuf>,
    /// 解析タブの設定値
    pub analysis_cfg: AnalysisSettings,
    /// 自動荷重同期（`sync_auto_load_cases_action`）が最後に行われた時点の
    /// モデル＋関連設定のハッシュ。次回呼び出し時に現在のハッシュと一致すれば
    /// DL/LL/EX/EY の再計算（床格子サブFEM解析等）を丸ごとスキップする。
    /// モデルの新規作成・読込では `None` にリセットする（永続化しない）。
    pub auto_load_sync_hash: Option<u64>,
    /// 最後の準備計算で生成した仕口パネルの諸元（節点 index の昇順）。
    /// 準備計算の結果タブに一覧表示する。永続化しない（モデルから毎回再生成する）。
    pub generated_panels: Vec<squid_n_element::panel_gen::GeneratedPanel>,
    /// 解析パネル「静的解析」で選択中の単体実行の対象（荷重ケース／荷重組合せ）。
    /// `None` は未選択（荷重ケースの先頭を既定として扱う）。
    #[cfg(feature = "gui")]
    pub analysis_target: Option<StaticTarget>,
    /// 荷重タブ「荷重組合せ」自動生成 UI のドラフト状態
    #[cfg(feature = "gui")]
    pub combo_draft: ComboDraft,
    /// モデルタブ「スラブ」追加フォームのドラフト状態
    #[cfg(feature = "gui")]
    pub slab_draft: crate::tables::slabs::SlabDraft,
    /// 解析タブ「階の定義」W[kN] 編集バッファ（kN、model.stories と同じ並び）。
    /// ドラッグ／フォーカス中でない行のみ model 値で上書きする。
    #[cfg(feature = "gui")]
    pub story_weight_edit: Vec<f64>,
    /// `story_weight_edit` の各行が現在操作中（ドラッグ中またはフォーカス中）か。
    /// true の間は model 値での上書きを止めて入力中の値を保つ。
    #[cfg(feature = "gui")]
    pub story_weight_active: Vec<bool>,
    /// モデルタブ「壁属性」フォームのドラフト状態
    #[cfg(feature = "gui")]
    pub wall_attr_draft: crate::tables::wall_attrs::WallAttrDraft,
    /// モデルタブ「雑壁」追加フォームのドラフト状態
    #[cfg(feature = "gui")]
    pub misc_wall_draft: crate::tables::misc_walls::MiscWallDraft,
    /// モデルタブ「通り芯」の通り名編集のドラフト状態
    #[cfg(feature = "gui")]
    pub axis_name_draft: crate::tables::axes::AxisNameDraft,
    /// 荷重タブ「荷重計算条件」フォームのドラフト状態
    #[cfg(feature = "gui")]
    pub load_cfg_draft: crate::tables::load_cfg::LoadCfgDraft,
    /// モデルタブ「部材付帯情報」フォームのドラフト状態
    #[cfg(feature = "gui")]
    pub member_detail_draft: crate::tables::member_details::MemberDetailDraft,
    /// モデルタブ「S造検定属性」フォームのドラフト状態
    #[cfg(feature = "gui")]
    pub steel_attr_draft: crate::tables::steel_attrs::SteelAttrDraft,
    /// 設計タブ「数量積算」ビューの状態（集計単位の切替）
    #[cfg(feature = "gui")]
    pub quantity_view: crate::quantity_view::QuantityViewState,
    /// 下ドック「準備計算」タブの状態（表示切替）
    #[cfg(feature = "gui")]
    pub prep_view: crate::prep_view::PrepViewState,
}

/// 荷重組合せ自動生成 UI のドラフト（GUI 専用）。DL/LL は必須、地震X/Y・積雪は任意。
#[cfg(feature = "gui")]
#[derive(Clone, Debug, Default)]
pub struct ComboDraft {
    pub dl: Option<LoadCaseId>,
    pub ll: Option<LoadCaseId>,
    pub seismic_x: Option<LoadCaseId>,
    pub seismic_y: Option<LoadCaseId>,
    pub snow: Option<LoadCaseId>,
}

impl Default for App {
    fn default() -> Self {
        Self {
            model: squid_n_core::model::Model::default(),
            results: None,
            selection: Selection::default(),
            undo: UndoStack::new(),
            active_tab: Tab::Model,
            design_term: LoadTerm::Long,
            last_static: None,
            last_error: None,
            last_notice: None,
            log: EventLog::default(),
            job: None,
            node_edit: Vec::new(),
            #[cfg(feature = "gui")]
            node_grid: crate::grid::GridWidget::new(),
            node_draft: ["0".to_string(), "0".to_string(), "0".to_string()],
            pending_duplicate_node_coord: None,
            pending_save_recording: None,
            staleness: Staleness::default(),
            diagnostics: Vec::new(),
            preparation: None,
            nav: Navigator::default(),
            model_tab: ModelTab::default(),
            // サンプル(門型ラーメン)が鋼構造のため既定は S ラーメン
            design_frame: squid_n_design_jp::secondary::holding_capacity::FrameType::SteelFrame,
            design_rank: squid_n_design_jp::secondary::holding_capacity::MemberRank::FA,
            design_rank_auto: false,
            wall_structure: false,
            ds_beta_u_by_story: Vec::new(),
            ds_beta_u_unavailable: false,
            ds_rank_fallback_stories: Vec::new(),
            ultimate_rp: 0.0,
            ultimate_lightweight: false,
            ultimate_include_bond: true,
            ultimate_upper_factor: 1.0,
            ultimate_mu_aci: false,
            ultimate_shear_ductility: false,
            ultimate_biaxial_shear: false,
            ultimate_biaxial_bending: false,
            ultimate_use_pushover: false,
            #[cfg(feature = "gui")]
            left_dock_open: true,
            #[cfg(feature = "gui")]
            left_panel: LeftPanel::default(),
            #[cfg(feature = "gui")]
            right_dock_open: true,
            #[cfg(feature = "gui")]
            right_panel: RightPanel::default(),
            #[cfg(feature = "gui")]
            bottom_dock_open: true,
            #[cfg(feature = "gui")]
            bottom_tab: BottomTab::default(),
            #[cfg(feature = "gui")]
            results_view: ResultsView::default(),
            #[cfg(feature = "gui")]
            design_view: DesignView::default(),
            #[cfg(feature = "gui")]
            mn_view: crate::mn_view::MnViewState::default(),
            #[cfg(feature = "gui")]
            view_mode: crate::viewer::ViewMode::default(),
            #[cfg(feature = "gui")]
            force_components: crate::viewer::ForceComponents::default(),
            #[cfg(feature = "gui")]
            diagram_values: false,
            #[cfg(feature = "gui")]
            cmq_component: crate::viewer::CmqComponent::default(),
            #[cfg(feature = "gui")]
            check_ratio_filter: crate::viewer::CheckRatioFilter::default(),
            #[cfg(feature = "gui")]
            check_ratio_markers: true,
            #[cfg(feature = "gui")]
            check_ratio_label_all: false,
            #[cfg(feature = "gui")]
            hinge_detail_elem: None,
            #[cfg(feature = "gui")]
            hinge_mn_cache: None,
            #[cfg(feature = "gui")]
            hinge_mn_camera: crate::viewer::CameraState::default(),
            #[cfg(feature = "gui")]
            diagram_contour: false,
            #[cfg(feature = "gui")]
            contour_colormap: crate::theme::ColorMap::default(),
            #[cfg(feature = "gui")]
            overlay_deform: false,
            #[cfg(feature = "gui")]
            show_floor_secondary: true,
            #[cfg(feature = "gui")]
            view_mode_idx: 0,
            #[cfg(feature = "gui")]
            camera: crate::viewer::CameraState::default(),
            #[cfg(feature = "gui")]
            frame_target: None,
            #[cfg(feature = "gui")]
            show_sections: false,
            #[cfg(feature = "gui")]
            deform_scale_factor: 1.0,
            #[cfg(feature = "gui")]
            show_diaphragm_master: false,
            #[cfg(feature = "gui")]
            modeling_analysis: crate::viewer::ModelingAnalysis::default(),
            #[cfg(feature = "gui")]
            show_beam_interpolation: true,
            #[cfg(feature = "gui")]
            th_frame: 0,
            #[cfg(feature = "gui")]
            th_playing: false,
            #[cfg(feature = "gui")]
            th_speed: 1.0,
            #[cfg(feature = "gui")]
            th_play_time: 0.0,
            #[cfg(feature = "gui")]
            th_detail_elem: None,
            #[cfg(feature = "gui")]
            th_detail_axis_z: true,
            #[cfg(feature = "gui")]
            th_detail_axial_component: None,
            #[cfg(feature = "gui")]
            th_scale_cache: None,
            beam_loads: Vec::new(),
            beam_loads_hash: None,
            #[cfg(feature = "gui")]
            time_history_data: crate::time_history_view::TimeHistoryData::default(),
            #[cfg(feature = "gui")]
            time_history_source: crate::time_history_view::TimeHistorySource::default(),
            #[cfg(feature = "gui")]
            time_history_view_mode: crate::time_history_view::TimeHistoryViewMode::default(),
            #[cfg(feature = "gui")]
            story_response_kind: crate::story_response::StoryResponseKind::default(),
            #[cfg(feature = "gui")]
            story_response_dir: crate::story_response::StoryRespDir::default(),
            stick_response: None,
            #[cfg(feature = "gui")]
            section_draft: crate::section_editor::SectionEditorDraft::default(),
            #[cfg(feature = "gui")]
            catalog_draft: crate::section_editor::CatalogDraft::default(),
            #[cfg(feature = "gui")]
            isolator_support_draft: crate::tables::nodes::IsolatorSupportDraft::default(),
            #[cfg(feature = "gui")]
            isolator_member_draft: crate::tables::members::IsolatorMemberDraft::default(),
            #[cfg(feature = "gui")]
            damper_def_draft: crate::damper_def_editor::DamperDefDraft::default(),
            #[cfg(feature = "gui")]
            beam_draw_mode: false,
            #[cfg(feature = "gui")]
            beam_draw_first: None,
            #[cfg(feature = "gui")]
            slab_draw_mode: false,
            #[cfg(feature = "gui")]
            slab_draw_nodes: Vec::new(),
            #[cfg(feature = "gui")]
            wall_draw_mode: false,
            #[cfg(feature = "gui")]
            wall_draw_nodes: Vec::new(),
            project_path: None,
            analysis_cfg: AnalysisSettings::default(),
            auto_load_sync_hash: None,
            generated_panels: Vec::new(),
            #[cfg(feature = "gui")]
            analysis_target: None,
            #[cfg(feature = "gui")]
            combo_draft: ComboDraft::default(),
            #[cfg(feature = "gui")]
            slab_draft: crate::tables::slabs::SlabDraft::default(),
            #[cfg(feature = "gui")]
            story_weight_edit: Vec::new(),
            #[cfg(feature = "gui")]
            story_weight_active: Vec::new(),
            #[cfg(feature = "gui")]
            wall_attr_draft: crate::tables::wall_attrs::WallAttrDraft::default(),
            #[cfg(feature = "gui")]
            misc_wall_draft: crate::tables::misc_walls::MiscWallDraft::default(),
            #[cfg(feature = "gui")]
            axis_name_draft: crate::tables::axes::AxisNameDraft::default(),
            #[cfg(feature = "gui")]
            load_cfg_draft: crate::tables::load_cfg::LoadCfgDraft::default(),
            #[cfg(feature = "gui")]
            member_detail_draft: crate::tables::member_details::MemberDetailDraft::default(),
            #[cfg(feature = "gui")]
            steel_attr_draft: crate::tables::steel_attrs::SteelAttrDraft::default(),
            #[cfg(feature = "gui")]
            quantity_view: crate::quantity_view::QuantityViewState::default(),
            #[cfg(feature = "gui")]
            prep_view: crate::prep_view::PrepViewState::default(),
        }
    }
}

/// 日本語フォントを egui に登録する（UI のラベルが豆腐□にならないように）。
///
/// egui の既定フォント（Ubuntu/Hack）は日本語グリフを持たないため、
/// OS のシステムフォントから日本語対応フォントを探して読み込む。
/// 見つからない場合は何もしない（英数字は既定フォントで表示される）。
#[cfg(feature = "gui")]
pub fn install_japanese_fonts(ctx: &egui::Context) {
    // OS ごとの代表的な日本語フォント候補（先に見つかったものを使用）。
    const CANDIDATES: &[&str] = &[
        // Windows
        "C:/Windows/Fonts/meiryo.ttc",
        "C:/Windows/Fonts/YuGothR.ttc",
        "C:/Windows/Fonts/YuGothM.ttc",
        "C:/Windows/Fonts/msgothic.ttc",
        // macOS
        "/System/Library/Fonts/ヒラギノ角ゴシック W3.ttc",
        "/System/Library/Fonts/Hiragino Sans GB.ttc",
        "/Library/Fonts/Osaka.ttf",
        // Linux (Noto / IPA / VL)
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/fonts-japanese-gothic.ttf",
        "/usr/share/fonts/truetype/vlgothic/VL-Gothic-Regular.ttf",
    ];

    let Some((path, bytes)) = CANDIDATES
        .iter()
        .find_map(|p| std::fs::read(p).ok().map(|b| (*p, b)))
    else {
        eprintln!(
            "[warn] 日本語フォントが見つかりませんでした。UI が文字化けする可能性があります。"
        );
        return;
    };

    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert(
        "jp".to_owned(),
        std::sync::Arc::new(egui::FontData::from_owned(bytes)),
    );
    // プロポーショナル・等幅の両ファミリーで日本語フォントを最優先にする。
    if let Some(family) = fonts.families.get_mut(&egui::FontFamily::Proportional) {
        family.insert(0, "jp".to_owned());
    }
    if let Some(family) = fonts.families.get_mut(&egui::FontFamily::Monospace) {
        family.push("jp".to_owned());
    }
    ctx.set_fonts(fonts);
    eprintln!("[info] 日本語フォントを読み込みました: {path}");
}

/// 標準荷重ケース名（DL・LL(架構用)・LL(地震用)・EX・EY・WX・WY）。
/// `squid_n_core::model` の定数を単一ソースオブトゥルースとして再公開する。
///
/// - `DL_CASE_NAME`: `sync_gravity_load_cases_action` がスラブの固定荷重
///   （仕上げ等）の分配と躯体自重（柱梁・壁・ダンパー・フレーム外雑壁）を
///   合算して同期する（レビュー §1.1・照合レビュー③梁自重/②壁荷重）。
/// - `LL_FRAME_CASE_NAME`: スラブ用途（`SlabUsage`）から令別表第1 の
///   骨組用積載を分配する（長期骨組解析用。令85条1項）。
/// - `LL_SEISMIC_CASE_NAME`（kind=LiveSeismic）: スラブ用途から令別表第1 の
///   **地震用**積載を分配する。地震用重量の集計
///   （`gravity_cases_for_seismic_weight` が LiveSeismic を優先採用）に
///   用いる（令85条1項・令88条）。
/// - `EX_CASE_NAME`/`EY_CASE_NAME`（kind=Seismic）:
///   `sync_seismic_load_cases_action` が階定義から Ai 分布の水平力を同期する。
/// - `WX_CASE_NAME`/`WY_CASE_NAME`（kind=Wind）:
///   `sync_wind_load_cases_action` が階定義から速度圧による層水平力を同期する。
pub use squid_n_core::model::{
    DL_CASE_NAME, EX_CASE_NAME, EY_CASE_NAME, LL_FRAME_CASE_NAME, LL_SEISMIC_CASE_NAME,
    WX_CASE_NAME, WY_CASE_NAME,
};

/// 旧スキーマの自重自動生成ケース名（読込時に DL へ移行される。
/// 未移行モデルに対する二重計上防止の除外判定にのみ使う）。
pub const SELF_WEIGHT_AUTO_LOAD_CASE_NAME: &str =
    squid_n_load::self_weight::SELF_WEIGHT_AUTO_LOAD_CASE_NAME;

// 節点ペアが鉛直材（柱）かどうかの判定。判定規則は `squid-n-core` を情報源とし、
// 荷重集計（`squid_n_load::story_gen`）・通り芯の自動生成と共通にする。
use squid_n_core::geom::is_vertical_pair;

/// 柱要素ごとの「支持する床数」と「積載荷重低減率」（令85条2項）を一覧する。
///
/// `Model.load_cfg.live_load_reduction == true` のときに UI 側で参考表示する
/// ための集計ヘルパー（`squid_n_load::live_load::{floors_supported_by_column,
/// column_live_load_reduction}` を薄くラップする）。**断面検定の長期軸力への
/// 実適用はまだ行っていない**（解析パイプラインへの侵襲が大きいため残課題。
/// 本関数は表示用の集計のみを提供する）。
///
/// 柱の判定は `ElementKind::Beam` の部材のうち両端節点の水平距離が 1mm 未満
/// （鉛直材）のものとする。所属階は `Node.story`（`ApplyStories`/階の自動生成で
/// 設定される）を用いるため、階が未生成のモデルでは全部材が 0 層扱いになる。
pub fn column_live_load_factors(model: &squid_n_core::model::Model) -> Vec<(ElemId, usize, f64)> {
    use squid_n_core::model::ElementKind;

    let node_story: Vec<Option<squid_n_core::ids::StoryId>> =
        model.nodes.iter().map(|n| n.story).collect();

    model
        .elements
        .iter()
        .filter(|e| e.kind == ElementKind::Beam && e.nodes.len() >= 2)
        .filter_map(|e| {
            let ni = e.nodes[0].index();
            let nj = e.nodes[1].index();
            if ni >= model.nodes.len() || nj >= model.nodes.len() {
                return None;
            }
            if !is_vertical_pair(model.nodes[ni].coord, model.nodes[nj].coord) {
                return None;
            }
            let floors = squid_n_load::live_load::floors_supported_by_column(model, e, &node_story);
            let factor = squid_n_load::live_load::column_live_load_reduction(floors);
            Some((e.id, floors, factor))
        })
        .collect()
}

/// 地震用重量に算入する重力ケースを `LoadCaseKind` から選択する（レビュー §1.7）。
///
/// - `kind == Dead` の全ケースを対象とする（標準構成では「DL」に躯体自重＋
///   スラブ固定荷重が自動同期される）。ただし旧スキーマの「自重(自動)」
///   （[`SELF_WEIGHT_AUTO_LOAD_CASE_NAME`]。未移行の場合のみ存在）は除外する
///   （その場合は密度からの自重直接算入と二重計上になるため。
///   [`density_self_weight_for_stories`] 参照）。
/// - `kind == LiveSeismic`（地震用積載）のケースがあれば併せて対象とする。
///   無ければ `kind == Live`（長期用積載）で代用する
///   （地震用の積載荷重には地震用の値を用いる（令85条）。地震用の値が
///   個別に定義されていなければ長期用の値をそのまま使う）。ただし
///   スラブ自動生成の骨組用積載ケース（[`LL_FRAME_CASE_NAME`]）は
///   **骨組用**の値を持つため、この代用対象から除外する（地震用値が明示的に
///   0 の用途で骨組用値へフォールバックし地震用重量が過大になるのを防ぐ。
///   スラブの地震用積載は常に [`LL_SEISMIC_CASE_NAME`] が担う）。
/// - いずれのケースも `kind` が設定されていない（全ケースが既定値 `Other`）
///   場合は、旧スキーマ・後方互換のため先頭ケースのみを返す
///   （並び順に依存する旧規約。新規モデルは kind 設定を推奨）。
fn gravity_cases_for_seismic_weight(model: &squid_n_core::model::Model) -> Vec<LoadCaseId> {
    use squid_n_core::model::LoadCaseKind;

    let any_kind_set = model
        .load_cases
        .iter()
        .any(|lc| lc.kind != LoadCaseKind::Other);
    if !any_kind_set {
        return model.load_cases.first().map(|c| c.id).into_iter().collect();
    }

    let mut result: Vec<LoadCaseId> = model
        .load_cases
        .iter()
        .filter(|lc| lc.kind == LoadCaseKind::Dead && lc.name != SELF_WEIGHT_AUTO_LOAD_CASE_NAME)
        .map(|lc| lc.id)
        .collect();

    let live_seismic: Vec<LoadCaseId> = model
        .load_cases
        .iter()
        .filter(|lc| lc.kind == LoadCaseKind::LiveSeismic)
        .map(|lc| lc.id)
        .collect();
    if !live_seismic.is_empty() {
        result.extend(live_seismic);
    } else {
        result.extend(
            model
                .load_cases
                .iter()
                .filter(|lc| lc.kind == LoadCaseKind::Live && lc.name != LL_FRAME_CASE_NAME)
                .map(|lc| lc.id),
        );
    }

    result
}

/// 階の自動生成で自重を材料密度から直接算入すべきか（地震用重量の二重計上防止）。
///
/// 標準構成では躯体自重は「DL」（kind=Dead・[`DL_CASE_NAME`]）へ自動同期され、
/// `gravity_cases_for_seismic_weight` が DL を重力ケースに含めるため、密度からの
/// 直接算入は行わない（`false`）。DL ケースが無い旧モデル・手動構成では従来
/// どおり密度から直接算入する（`true`）。
fn density_self_weight_for_stories(model: &squid_n_core::model::Model) -> bool {
    !model
        .load_cases
        .iter()
        .any(|lc| lc.kind == squid_n_core::model::LoadCaseKind::Dead && lc.name == DL_CASE_NAME)
}

/// 階の生成結果を `ApplyStories` で適用したときにモデルが変化するか。
///
/// 準備計算は実行のたびに階を作り直すため、モデルが変わっていないのに毎回
/// undo 履歴を積み、`mark_edited` で解析結果を stale にしてしまわないよう、
/// 適用前に「差分があるか」を判定する（`generate_stories_action` の冪等化）。
///
/// [`squid_n_edit::ApplyStories`] が書き換える対象をすべて突き合わせる:
/// 階・所属階・剛床拘束（非剛床の拘束を残したうえで剛床拘束を末尾へ置き換えるため、
/// 適用後の並びを組み立てて比較する）・剛床代表節点（ID の位置に同じ内容の節点が
/// 既にあること）・`generated_masters`・質量方式。
fn story_gen_changes_model(
    model: &squid_n_core::model::Model,
    gen: &squid_n_load::story_gen::StoryGenResult,
    mass_method: squid_n_core::model::MassMethod,
) -> bool {
    use squid_n_core::model::Constraint;
    if model.mass_method != mass_method
        || model.stories != gen.stories
        || model.generated_masters != gen.generated_masters
    {
        return true;
    }
    // 剛床代表節点: 範囲外なら追加が必要＝変化あり。
    if gen
        .rep_nodes
        .iter()
        .any(|rn| model.nodes.get(rn.id.index()) != Some(rn))
    {
        return true;
    }
    // 所属階（`ApplyStories` は model.nodes と node_story を zip して設定する）。
    // ただし剛床代表節点は node_story の適用後に `rep_nodes` で丸ごと置換されるため、
    // その節点の所属階は rep_nodes 側が正（node_story 側は None のまま）。
    // 上で rep_nodes の一致を確認済みなので、ここでは代表節点を除いて比較する。
    let rep: std::collections::HashSet<NodeId> = gen.rep_nodes.iter().map(|n| n.id).collect();
    if model
        .nodes
        .iter()
        .zip(gen.node_story.iter())
        .any(|(n, s)| !rep.contains(&n.id) && n.story != *s)
    {
        return true;
    }
    // 剛床拘束の並び替えも変化として扱う（適用後の並びをそのまま組み立てて比較）。
    let mut applied: Vec<Constraint> = model
        .constraints
        .iter()
        .filter(|c| !matches!(c, Constraint::RigidDiaphragm { .. }))
        .cloned()
        .collect();
    applied.extend(gen.constraints.iter().cloned());
    applied != model.constraints
}

/// 波形 CSV/テキストの内容を解析する（ヘッドレステスト可能な純粋関数）。
///
/// - `ThDir::X` / `ThDir::Y`: 1 行 1 値（カンマ区切りなら最後の列）を加速度(gal)として
///   読む（従来仕様）。数値化できない行は無視する。戻り値の第 2 要素は常に `None`。
/// - `ThDir::Xy`: 1 行をカンマ区切り 2 列（1 列目 X、2 列目 Y、ともに gal）として読む。
///   2 列に満たない行があればエラーを返す（「X+Y には2列のCSVが必要です」）。
///   数値化できない行（ヘッダ等）は無視する。
///
/// いずれも gal → mm/s²（内部単位系）へ ×10 で変換して返す。
/// 有効なデータ点が 2 点未満の場合はエラーを返す。
///
/// 呼び出し元（`run_time_history_from_csv`）は GUI 専用のため、非 GUI ビルドでは
/// テストからのみ使用される（`cfg(any(test, feature = "gui"))` で dead_code 警告を回避）。
#[cfg(any(test, feature = "gui"))]
fn parse_wave_csv(content: &str, dir: ThDir) -> Result<(Vec<f64>, Option<Vec<f64>>), String> {
    match dir {
        ThDir::X | ThDir::Y => {
            let accel: Vec<f64> = content
                .lines()
                .filter_map(|l| {
                    let field = l.split(',').next_back()?.trim();
                    field.parse::<f64>().ok()
                })
                .map(|gal| gal * 10.0)
                .collect();
            if accel.len() < 2 {
                return Err(
                    "波形データが読み取れませんでした（数値が 2 点未満）。1 行 1 値の CSV を指定してください。"
                        .to_string(),
                );
            }
            Ok((accel, None))
        }
        ThDir::Xy => {
            let mut xs = Vec::new();
            let mut ys = Vec::new();
            for line in content.lines() {
                let line = line.trim();
                if line.is_empty() {
                    continue;
                }
                let fields: Vec<&str> = line.split(',').collect();
                if fields.len() < 2 {
                    return Err("X+Y には2列のCSVが必要です".to_string());
                }
                let (Ok(x), Ok(y)) = (
                    fields[0].trim().parse::<f64>(),
                    fields[1].trim().parse::<f64>(),
                ) else {
                    // ヘッダ行等、数値化できない行は無視する。
                    continue;
                };
                xs.push(x * 10.0);
                ys.push(y * 10.0);
            }
            if xs.len() < 2 {
                return Err(
                    "波形データが読み取れませんでした（数値が 2 点未満）。X+Y には2列のCSVが必要です。"
                        .to_string(),
                );
            }
            Ok((xs, Some(ys)))
        }
    }
}

/// 部材種別判定（部材軸の鉛直成分による幾何判定）。
///
/// - |ez| ≥ 0.8: 柱（軸力＋二軸曲げの複合検定）
/// - |ez| ≤ 0.2: 梁（強軸曲げ＋せん断）
/// - それ以外: ブレース（軸力検定）
fn member_kind_of(
    elem: &squid_n_core::model::ElementData,
    model: &squid_n_core::model::Model,
) -> squid_n_design_jp::MemberKind {
    use squid_n_design_jp::MemberKind;
    let coords: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .take(2)
        .collect();
    let (Some(p0), Some(p1)) = (coords.first(), coords.get(1)) else {
        return MemberKind::Beam;
    };
    let dx = p1[0] - p0[0];
    let dy = p1[1] - p0[1];
    let dz = p1[2] - p0[2];
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    if len < 1e-9 {
        return MemberKind::Beam;
    }
    let ez = (dz / len).abs();
    if ez >= 0.8 {
        squid_n_design_jp::MemberKind::Column
    } else if ez <= 0.2 {
        squid_n_design_jp::MemberKind::Beam
    } else {
        squid_n_design_jp::MemberKind::Brace
    }
}

/// 壁要素の側柱（壁と**両端の**節点を共有する鉛直線材）に SRC 造の柱があるか。
///
/// SRC 耐震壁（部材種別 WA/WC の判定対象）かどうかの判定に用いる。壁自体の
/// 断面は RC 壁（`RcWall`）のままモデル化されるため、壁の構造種別は側柱の
/// 構造種別（`member_structure_kind`）から推定する。
///
/// 側柱は柱頭・柱脚の双方が壁の隅節点に一致するため、**両端**の共有を課す。
/// 片側 1 節点の共有まで許すと、壁上辺の隅節点から立ち上がる上階の柱にも
/// 反応し、SRC 階→RC 階の切替部で RC 側の耐震壁を SRC 壁と誤判定する。
/// 鉛直判定は側柱ピン化（`side_column`）と同じ 45° 余弦基準。
fn wall_has_src_boundary_column(
    wall: &squid_n_core::model::ElementData,
    model: &squid_n_core::model::Model,
) -> bool {
    use squid_n_core::model::ElementKind;
    use squid_n_core::structure_kind::{member_structure_kind, StructureKind};
    let wall_nodes: std::collections::HashSet<squid_n_core::ids::NodeId> =
        wall.nodes.iter().copied().collect();
    model.elements.iter().any(|e| {
        if e.id == wall.id
            || !matches!(
                e.kind,
                ElementKind::Beam | ElementKind::Fiber | ElementKind::MultiSpring
            )
            || e.nodes.len() < 2
            || !e.nodes.iter().take(2).all(|n| wall_nodes.contains(n))
        {
            return false;
        }
        let (Some(a), Some(b)) = (
            model.nodes.get(e.nodes[0].index()),
            model.nodes.get(e.nodes[1].index()),
        ) else {
            return false;
        };
        squid_n_core::geom::is_vertical_axis(a.coord, b.coord)
            && member_structure_kind(model, e) == StructureKind::Src
    })
}

/// `SectionShape::RcRect` の配筋情報から RC 終局耐力算定（rank-auto）用の入力を組み立てる。
///
/// # 変換規則
/// - 曲げ・せん断は強軸（せい=d）まわりを想定する。引張側主筋量 `at` は上下対称配筋を
///   仮定し、`main_x`（せい方向主筋）の総断面積の半分とする（非対称配筋の場合は別途検討）。
/// - `d_eff` = d - かぶり - 主筋径/2。
/// - `pw` = せん断補強筋 1 組の断面積(π/4・dia²)×組数 / (b・ピッチ)。ピッチが 0 以下なら 0。
/// - `sigma_y`: 断面（配筋）の主筋材質 → 部材材料の `fy` の順で解決し、どちらも
///   未設定なら 345 N/mm²（SD345 相当、要・原典照合）。
/// - `sigma_wy`: 断面（配筋）のせん断補強筋材質から解決し、未設定は 295 N/mm²
///   （SD295 相当＝規格上の最小グレードで、耐力を過小評価する安全側）。
/// - `fc`: 材料の `fc`（コンクリート設計基準強度）が未設定の場合は `None` を返し、
///   ランク算定の対象外（呼び出し側で選択値へフォールバック）とする。
fn rc_capacity_input_from_rect(
    b: f64,
    d: f64,
    rebar: &squid_n_core::section_shape::RcRebar,
    mat: &squid_n_core::model::Material,
    clear_span: f64,
) -> Option<squid_n_design_jp::secondary::rc_capacity::RcCapacityInput> {
    let fc = mat.fc?;
    let bar_area = |bs: &squid_n_core::section_shape::BarSet| -> f64 {
        bs.count as f64 * std::f64::consts::PI / 4.0 * bs.dia * bs.dia
    };
    // 上下対称配筋を仮定し、引張側主筋量は main_x 総断面積の半分。
    let at = bar_area(&rebar.main_x) / 2.0;
    let d_eff = d - rebar.cover - rebar.main_x.dia / 2.0;
    let shear_area =
        std::f64::consts::PI / 4.0 * rebar.shear.dia * rebar.shear.dia * rebar.shear.legs as f64;
    let pw = if rebar.shear.pitch > 0.0 {
        shear_area / (b * rebar.shear.pitch)
    } else {
        0.0
    };
    Some(squid_n_design_jp::secondary::rc_capacity::RcCapacityInput {
        b,
        d,
        at,
        d_eff,
        sigma_y: squid_n_core::material_grade::rebar_yield_strength(
            rebar.main_grade.as_deref(),
            Some(mat),
        )
        .unwrap_or(345.0), // SD345 相当、要・原典照合
        fc,
        pw,
        sigma_wy: squid_n_core::material_grade::shear_rebar_yield_strength(
            rebar.shear.grade.as_deref(),
        )
        .unwrap_or(squid_n_core::material_grade::SHEAR_REBAR_DEFAULT_FY),
        clear_span,
        // 軸方向圧縮応力度は呼び出し側(compute_holding_capacity)が既知の場合に上書きする。
        // ここでは既定値 0(軸力なし・安全側)とする。
        sigma_0: 0.0,
    })
}

/// 長期軸力の簡易近似として先頭荷重ケース(`model.load_cases.first()`)の結果を優先し、
/// `bundle.statics` に無ければ従来どおり最後に実行した静的解析結果(`member_forces`)を
/// 用いて部材の軸力を取得する。圧縮のときのみ σ0 \[N/mm²\]（= |N|/(b・D)）を返す。
/// 引張・軸力なし・対象部材の結果が無い場合は 0.0（安全側）。
///
/// `statics` は `StaticCaseKey` をキーとするため、ユーザー荷重ケースの結果
/// (`StaticCaseKey::User`)と地震静的の結果(`StaticCaseKey::Seismic`)は別々に
/// 格納される（旧実装では両者とも `LoadCaseId(0)` を共有し、後から実行した方が
/// 先頭荷重ケースの結果を上書きしてしまう問題があったが、型で区別したことで解消済み）。
/// 先頭荷重ケースが `statics` に無い（未実行）場合のみ `fallback_member_forces`
/// （最後に実行した静的解析の内力）を用いる。
///
/// # 符号規約（要確認済み・推測ではない）
/// `squid_n_element::beam::BeamElement::recover_forces` は局所剛性 K・u を
/// そのまま評価値とするため、始端(pos=0.0、`eval_sections`\[0\])では
/// `n = f_local[0] = -N`（N は引張正）となる。これは
/// `squid_n_solver::linear::test_linear_static_axial_cantilever` で
/// N=+1000N（引張）を与えたとき `forces.at[0].1[0]` ≈ -1000 になることで
/// 確認済み（すなわち f_local\[0\] は「圧縮正」）。よって `mf.at.first()`
/// (= pos=0.0、始端)の n は「圧縮正」（n>0 のとき圧縮）であり、
/// n<=0（引張または軸力なし）なら σ0=0（安全側）とする。
fn rc_sigma_0_from_gravity_or_last_static(
    statics: &[(StaticCaseKey, squid_n_solver::linear::StaticOnce)],
    fallback_member_forces: &[(ElemId, squid_n_element::beam::MemberForces)],
    gravity_lc: Option<LoadCaseId>,
    elem_id: ElemId,
    b: f64,
    d: f64,
) -> f64 {
    let member_forces = gravity_lc
        .and_then(|lc| {
            statics
                .iter()
                .find(|(id, _)| *id == StaticCaseKey::User(lc))
        })
        .map(|(_, s)| s.member_forces.as_slice())
        .unwrap_or(fallback_member_forces);

    // 軸力 N は引張正の部材内力（beam.rs recover_forces）。圧縮（N<0）のみ
    // σ0 に反映し、引張は 0 とする（安全側）。
    member_forces
        .iter()
        .find(|(id, _)| *id == elem_id)
        .and_then(|(_, mf)| mf.at.first())
        .map(|(_, f)| f[0])
        .filter(|n| *n < 0.0)
        .map(|n| -n / (b * d))
        .unwrap_or(0.0)
}

/// 鋼断面形状の最大板厚 [mm]（F 値の板厚区分判定用）。
/// 形状情報のない断面・RC 断面は 0（板厚 40mm 以下の区分扱い）。
fn steel_max_thickness(shape: &squid_n_core::section_shape::SectionShape) -> f64 {
    use squid_n_core::section_shape::SectionShape;
    match *shape {
        SectionShape::SteelH {
            web_thick,
            flange_thick,
            ..
        }
        | SectionShape::SteelChannel {
            web_thick,
            flange_thick,
            ..
        }
        | SectionShape::SteelTee {
            web_thick,
            flange_thick,
            ..
        } => web_thick.max(flange_thick),
        SectionShape::SteelBox { thick, .. }
        | SectionShape::SteelAngle { thick, .. }
        | SectionShape::SteelPipe { thick, .. }
        | SectionShape::SteelFlatBar { thick, .. }
        | SectionShape::SteelLipChannel { thick, .. }
        | SectionShape::CftBox { thick, .. }
        | SectionShape::CftPipe { thick, .. } => thick,
        // 中実丸鋼は板要素でないため径を板厚区分に用いる。
        SectionShape::SteelRoundBar { dia } => dia,
        SectionShape::SteelBuiltH {
            web_thick,
            upper_thick,
            lower_thick,
            ..
        } => web_thick.max(upper_thick).max(lower_thick),
        SectionShape::SrcRect {
            steel_web_thick,
            steel_flange_thick,
            ..
        } => steel_web_thick.max(steel_flange_thick),
        SectionShape::RcRect { .. }
        | SectionShape::RcCircle { .. }
        | SectionShape::RcWall { .. } => 0.0,
    }
}

/// 幅厚比ランク表の行（柱／梁）を部材から選ぶ。柱以外（梁・ブレース）は梁の行を用いる。
pub(crate) fn steel_member_use_of(
    elem: &squid_n_core::model::ElementData,
    model: &squid_n_core::model::Model,
) -> squid_n_design_jp::secondary::width_thickness::SteelMemberUse {
    use squid_n_design_jp::secondary::width_thickness::SteelMemberUse;
    match member_kind_of(elem, model) {
        squid_n_design_jp::MemberKind::Column => SteelMemberUse::Column,
        _ => SteelMemberUse::Beam,
    }
}

/// 鋼断面の幅厚比から部材ランク（FA〜FD）を判定する。
///
/// 構造規定の幅厚比表（部材種別×断面×部位×鋼種級。
/// [`squid_n_design_jp::secondary::width_thickness::s_member_rank_by_kihon`]）に
/// よる判定のみを行い、表の対象外形状（溝形・T形・山形・円形鋼管以外の
/// 幅厚比非対象断面等）は `None`（未判定。層は選択ランクへフォールバック）と
/// する。表に規範的根拠のない形状へ仮のしきい値で自動ランクを付けると、
/// 根拠のない値が Ds を決めてしまうため、未判定として手動選択に委ねる。
///
/// 保有水平耐力の Ds 算定（`compute_holding_capacity`）と準備計算の表示
/// （`build_prep_width_thickness`）が同一の判定を用いるための共通関数。
pub(crate) fn steel_width_thickness_rank(
    shape: &squid_n_core::section_shape::SectionShape,
    member_use: squid_n_design_jp::secondary::width_thickness::SteelMemberUse,
    material_name: &str,
) -> Option<squid_n_design_jp::secondary::holding_capacity::MemberRank> {
    squid_n_design_jp::secondary::width_thickness::s_member_rank_by_kihon(
        shape,
        member_use,
        material_name,
    )
}

/// 部材両端節点間の幾何長 \[mm\]（内法補正なしの簡易値。剛域等は考慮しない）。
fn elem_geometric_length(
    elem: &squid_n_core::model::ElementData,
    model: &squid_n_core::model::Model,
) -> f64 {
    let coords: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .take(2)
        .collect();
    let (Some(p0), Some(p1)) = (coords.first(), coords.get(1)) else {
        return 0.0;
    };
    let dx = p1[0] - p0[0];
    let dy = p1[1] - p0[1];
    let dz = p1[2] - p0[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// 一本部材グループ 1 本分の検定文脈（断面検定の採用応力。
/// 一本部材指定時の採用応力）。
struct BeamGroupOverride {
    /// 一本部材の全長 L [mm]（分割部材長の総和）。
    length: f64,
    /// 一本部材両端の強軸曲げ (M_i端, M_j端) [N·mm]。
    end_moments_z: Option<(f64, f64)>,
    /// 一本部材中央の強軸曲げ Mc [N·mm]。A 式（M0=(Q1+Q2)L/8 による復元値）と
    /// B 式（中央に位置する分割部材の内力）の大きい方を採用する。
    mid_moment_z: Option<f64>,
    /// グループ内 |Mz| 最大位置の (|M|, |Q|)（せん断スパン比の代表値）。
    shear_span: Option<(f64, f64)>,
    /// 一本部材の内法長（両外端の剛域控除後）[mm]。
    clear_length: f64,
}

/// `Model.beam_groups` の各グループについて検定文脈の合成値を求め、
/// 所属要素 ID → 合成値の対応表を返す。
///
/// - グループは軸方向に連続する梁要素の ID を**並び順**で持つ前提
///   （幾何学的な連続性・共線性の検証は行わない。並び順が実際の配置と
///   異なる場合、端部モーメント等の対応がずれる）。
/// - 要素または内力が欠けるグループ・要素数 2 未満のグループは無視する。
/// - 中央モーメントは、A 式 `Mc_A = (|Q1|+|Q2|)・L/8 − (|M1|+|M2|)/2`
///   （端部せん断と釣り合う等分布荷重の単純梁中央モーメントから端部
///   モーメントの平均を差し引いた復元値）と、B 式（グループ中央位置を
///   含む分割部材の、中央位置に最も近い評価行のモーメント）の絶対値の
///   大きい方（符号は B 式に合わせる）。
fn beam_group_overrides(
    model: &squid_n_core::model::Model,
    member_forces: &[(ElemId, squid_n_element::beam::MemberForces)],
) -> std::collections::HashMap<ElemId, std::rc::Rc<BeamGroupOverride>> {
    use std::collections::HashMap;
    use std::rc::Rc;
    let mut out: HashMap<ElemId, Rc<BeamGroupOverride>> = HashMap::new();

    for group in &model.beam_groups {
        if group.len() < 2 {
            continue;
        }
        // 各分割部材の (要素, 内力, 長さ) を並び順に収集。欠けがあればスキップ。
        let mut parts: Vec<(
            &squid_n_core::model::ElementData,
            &squid_n_element::beam::MemberForces,
            f64,
        )> = Vec::with_capacity(group.len());
        let mut ok = true;
        for id in group {
            let elem = model.elements.iter().find(|e| e.id == *id);
            let mf = member_forces
                .iter()
                .find(|(mid, _)| mid == id)
                .map(|(_, m)| m);
            match (elem, mf) {
                (Some(e), Some(m)) if !m.at.is_empty() => {
                    let l = elem_geometric_length(e, model);
                    if l <= 1e-9 {
                        ok = false;
                        break;
                    }
                    parts.push((e, m, l));
                }
                _ => {
                    ok = false;
                    break;
                }
            }
        }
        if !ok || parts.len() < 2 {
            continue;
        }

        let total: f64 = parts.iter().map(|p| p.2).sum();
        let row_at = |m: &squid_n_element::beam::MemberForces, target: f64| -> Option<[f64; 6]> {
            m.at.iter()
                .find(|(p, _)| (p - target).abs() < 1e-9)
                .map(|(_, f)| *f)
        };
        let first = parts[0].1;
        let last = parts[parts.len() - 1].1;
        let end_i = row_at(first, 0.0);
        let end_j = row_at(last, 1.0);
        let end_moments_z = match (end_i, end_j) {
            (Some(a), Some(b)) => Some((a[5], b[5])),
            _ => None,
        };

        // A 式: M0 = (Q1+Q2)・L/8（端部せん断と釣り合う等分布仮定）。
        let q1 = end_i.map(|f| f[1].abs()).unwrap_or(0.0);
        let q2 = end_j.map(|f| f[1].abs()).unwrap_or(0.0);
        let m0_a = (q1 + q2) * total / 8.0;
        let m_ends_avg = end_moments_z
            .map(|(a, b)| (a.abs() + b.abs()) / 2.0)
            .unwrap_or(0.0);
        let mc_a = m0_a - m_ends_avg;

        // B 式: グループ中央位置を含む分割部材の、中央位置に最も近い評価行。
        let target_s = total / 2.0;
        let mut acc = 0.0;
        let mut mc_b: Option<f64> = None;
        for (_, m, l) in &parts {
            if target_s <= acc + l + 1e-9 {
                let xi = ((target_s - acc) / l).clamp(0.0, 1.0);
                mc_b =
                    m.at.iter()
                        .min_by(|a, b| {
                            (a.0 - xi)
                                .abs()
                                .partial_cmp(&(b.0 - xi).abs())
                                .unwrap_or(std::cmp::Ordering::Equal)
                        })
                        .map(|(_, f)| f[5]);
                break;
            }
            acc += l;
        }
        let mid_moment_z = mc_b.map(|b| {
            let sign = if b >= 0.0 { 1.0 } else { -1.0 };
            sign * b.abs().max(mc_a)
        });

        let shear_span = parts
            .iter()
            .flat_map(|(_, m, _)| m.at.iter())
            .map(|(_, f)| (f[5].abs(), f[1].abs()))
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        let face_sum = parts[0].0.rigid_zone.face_i + parts[parts.len() - 1].0.rigid_zone.face_j;
        let clear_length = if total - face_sum > 0.0 {
            total - face_sum
        } else {
            total
        };

        let ov = Rc::new(BeamGroupOverride {
            length: total,
            end_moments_z,
            mid_moment_z,
            shear_span,
            clear_length,
        });
        for (e, _, _) in &parts {
            out.insert(e.id, ov.clone());
        }
    }
    out
}

/// 危険断面位置（§6.2.3、既定は柱フェイスと中央）を正規化座標 \[0,1\] で算定する。
/// `squid_n_element::beam::BeamElement::new` の `eval_sections` 算定と同じ規則
/// （xi_i は \[0.0, 0.5) へ、xi_j は (0.5, 1.0\] へクランプ）で face_i/face_j から
/// 求める。face=0（直交材が無い端）では節点芯（0.0/1.0）と一致する。
///
/// `detail`（`Model::member_detail(elem.id)`）が付帯情報を持つ場合は、その
/// 追加検定位置（ハンチ端・継手位置。`MemberDetailAttr::extra_check_positions`）
/// も含める（剛性には影響しない。§6.2.3「位置はユーザが追加・変更可能」）。
/// `squid_n_element::beam::BeamElement::new` の `eval_sections` と同じ実装を
/// 使うため、両者の位置一致判定（1e-6）が保証される。
fn design_positions(
    elem: &squid_n_core::model::ElementData,
    geom_len: f64,
    detail: Option<&squid_n_core::model::MemberDetailAttr>,
) -> Vec<f64> {
    let mut xs = if geom_len > 1e-12 {
        let xi_i = (elem.rigid_zone.face_i / geom_len).clamp(0.0, 0.5 - 1e-9);
        let xi_j = (1.0 - elem.rigid_zone.face_j / geom_len).clamp(0.5 + 1e-9, 1.0);
        vec![xi_i, 0.5, xi_j]
    } else {
        vec![0.0, 0.5, 1.0]
    };
    if let Some(detail) = detail {
        xs.extend(detail.extra_check_positions(&elem.rigid_zone, geom_len));
    }
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    xs.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    xs
}

/// `pos` が `positions` のいずれかと 1e-6 以内で一致するか判定する。
fn is_near_design_position(pos: f64, positions: &[f64]) -> bool {
    positions.iter().any(|p| (p - pos).abs() < 1e-6)
}

mod actions;
pub mod node_grid;
mod preparation;

pub use preparation::*;

#[cfg(feature = "gui")]
mod panels;

/// 保存（Windows/Linux: Ctrl+S、macOS: ⌘S）。
#[cfg(feature = "gui")]
pub(crate) const SHORTCUT_SAVE: egui::KeyboardShortcut =
    egui::KeyboardShortcut::new(egui::Modifiers::COMMAND, egui::Key::S);

/// 名前を付けて保存（Windows/Linux: Ctrl+Shift+S、macOS: ⇧⌘S）。
#[cfg(feature = "gui")]
pub(crate) const SHORTCUT_SAVE_AS: egui::KeyboardShortcut = egui::KeyboardShortcut::new(
    egui::Modifiers::COMMAND.plus(egui::Modifiers::SHIFT),
    egui::Key::S,
);

#[cfg(feature = "gui")]
impl eframe::App for App {
    // eframe のデフォルトは (12,12,12) ≒ 黒なので、テーマに合わせた白灰色で上書きする
    fn clear_color(&self, _visuals: &egui::Visuals) -> [f32; 4] {
        crate::theme::GRAY_100.to_normalized_gamma_f32()
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // バックグラウンド解析ジョブ（P8 §5）: 完了していれば結果を適用し、
        // 実行中は完了検知のため再描画を要求し続ける。
        if self.job.is_some() {
            self.poll_job();
            ui.ctx()
                .request_repaint_after(std::time::Duration::from_millis(100));
        }

        // 作成パレットが見えていない間は作成モードを強制解除する。モードのトグル・
        // 進捗表示・クリア処理はパレット内にしかないため、非表示のままモードが残ると
        // 3D クリックで意図しない部材が無言で生成される（可視性と発動可能性を一致させる）。
        if !(self.left_dock_open && self.left_panel == LeftPanel::DrawTools) {
            self.reset_draw_modes();
        }

        // 保存ショートカット。consume_shortcut がイベントを消費するため後続の
        // ウィジェットには流れない。セル編集中でも発火する（保存されるのは
        // 確定済みの状態。未確定の編集は確定時に未保存マーカーが再点灯する）。
        if ui
            .ctx()
            .input_mut(|i| i.consume_shortcut(&SHORTCUT_SAVE_AS))
        {
            self.save_project_dialog(true);
        } else if ui.ctx().input_mut(|i| i.consume_shortcut(&SHORTCUT_SAVE)) {
            self.save_project_dialog(false);
        }

        // 保存サイズ超過の確認ダイアログ（時刻歴の詳細記録を含めるかの選択）。
        // どのタブからの保存でも表示できるよう、ここで描画する。
        if self.pending_save_recording.is_some() {
            let mut choice: Option<bool> = None;
            let mut do_cancel = false;
            let mut open = true;
            egui::Window::new("保存サイズの確認")
                .title_bar(true)
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
                .open(&mut open)
                .show(ui.ctx(), |ui| {
                    if let Some((_, size_mb)) = &self.pending_save_recording {
                        ui.label(format!(
                            "解析結果のサイズが約 {} MB あります（時刻歴の詳細記録を含む）。",
                            size_mb
                        ));
                    }
                    ui.label("時刻歴の詳細記録（3D アニメーション・部材履歴ループ用）を保存に含めますか？");
                    ui.label("除外しても層応答・ピーク値等の集計結果は保存され、詳細記録は再解析で復元できます。");
                    ui.horizontal(|ui| {
                        if ui.button("含めて保存").clicked() {
                            choice = Some(true);
                        }
                        if ui.button("除外して保存").clicked() {
                            choice = Some(false);
                        }
                        if ui.button("キャンセル").clicked() {
                            do_cancel = true;
                        }
                    });
                });
            if !open || do_cancel {
                self.pending_save_recording = None;
            }
            if let Some(include) = choice {
                if let Some((path, _)) = self.pending_save_recording.take() {
                    self.save_project_to_opts(path, Some(include));
                }
            }
        }

        // 上部ツールバー: ファイルメニュー + 工程タブ（自由遷移）+ Undo/Redo
        egui::Panel::top("top_toolbar").show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                ui.menu_button("ファイル", |ui| {
                    if ui.button("📄 新規").clicked() {
                        // 新規モデルは標準荷重ケース（DL・LL(架構用)・LL(地震用)・EX・EY）付き。
                        self.load_model(squid_n_core::model::Model::with_default_load_cases());
                        self.project_path = None;
                        ui.close();
                    }
                    if ui.button("🏠 サンプル(門型ラーメン)").clicked() {
                        self.load_model(crate::sample::portal_frame());
                        self.project_path = None;
                        ui.close();
                    }
                    ui.separator();
                    if ui.button("📂 開く…").clicked() {
                        self.open_project_dialog();
                        ui.close();
                    }
                    let save_btn = egui::Button::new("💾 保存")
                        .shortcut_text(ui.ctx().format_shortcut(&SHORTCUT_SAVE));
                    if ui.add(save_btn).clicked() {
                        self.save_project_dialog(false);
                        ui.close();
                    }
                    let save_as_btn = egui::Button::new("💾 名前を付けて保存…")
                        .shortcut_text(ui.ctx().format_shortcut(&SHORTCUT_SAVE_AS));
                    if ui.add(save_as_btn).clicked() {
                        self.save_project_dialog(true);
                        ui.close();
                    }
                    ui.separator();
                    if ui
                        .button("📥 ST-Bridge 読込…")
                        .on_hover_text(
                            "ST-Bridge 2.0 サブセット（節点・部材・断面・材料・節点荷重）。支点・部材荷重・組合せは含まれません",
                        )
                        .clicked()
                    {
                        self.import_stbridge_dialog();
                        ui.close();
                    }
                    if ui
                        .button("📤 ST-Bridge 書出…")
                        .on_hover_text(
                            "標準 ST-Bridge 2.0.2（StbSecColumn_S 等＋形鋼ライブラリ）で書き出す。BIM・他ソフト向け。材料はグレード名で表し、支点・荷重・材料の E/ν は含まれません（完全一致の保存は .scz）",
                        )
                        .clicked()
                    {
                        self.export_stbridge_dialog();
                        ui.close();
                    }
                });
                ui.separator();
                let tabs = [
                    ("モデル", Tab::Model),
                    ("荷重", Tab::Loads),
                    ("解析", Tab::Analysis),
                    ("結果", Tab::Results),
                    ("設計", Tab::Design),
                    ("レポート", Tab::Report),
                ];
                for (label, tab) in &tabs {
                    let selected = self.active_tab == *tab;
                    let stale_marker = match *tab {
                        // 進行中の下流タブに stale バッジを付与（§5）
                        Tab::Results | Tab::Design if self.staleness.results_stale => "⚠",
                        _ => "",
                    };
                    let label_str = format!("{} {}", label, stale_marker);
                    // プリセットは工程が実際に変わったときのみ適用する。選択中タブの
                    // 再クリックで手動調整したドック配置が巻き戻らないようにするため。
                    if ui.selectable_label(selected, label_str).clicked() && !selected {
                        self.active_tab = *tab;
                        self.apply_tab_preset(*tab);
                    }
                }
                ui.separator();
                let can_undo = self.undo.can_undo();
                let can_redo = self.undo.can_redo();
                if ui
                    .add_enabled(can_undo, egui::Button::new("↶ Undo"))
                    .clicked()
                {
                    self.undo.undo(&mut self.model);
                    self.staleness.mark_edited();
                }
                if ui
                    .add_enabled(can_redo, egui::Button::new("↷ Redo"))
                    .clicked()
                {
                    self.undo.redo(&mut self.model);
                    self.staleness.mark_edited();
                }
                // 荷重継続性区分（長期/短期）の手動切替はツールバーに置かない。
                // 区分は表示対象の荷重ケース／組合せから自動判定される
                // （`select_displayed_result` → `is_short_term_combo`）ため、
                // 手動で切り替えても次の選択・再解析で必ず上書きされ、設定として
                // 機能していなかった。現在の区分は設計タブに読み取り専用で表示する。
            });
        });

        // 下：ステータスバー（高さは本文1行分。egui::Panel が枠線を描くため、
        // 内部の区切り線・矩形分割は不要になった）
        egui::Panel::bottom("status_bar")
            .exact_size(ui.text_style_height(&egui::TextStyle::Body) + 8.0)
            .show_inside(ui, |ui| {
                self.status_bar(ui);
            });

        // 左：パネル切替式（ナビゲータ／作成パレット）。Zed のようにステータスバーの
        // アイコンで切り替える（切替自体は status_bar が行う）。
        if self.left_dock_open {
            egui::Panel::left("left_dock")
                .resizable(true)
                .default_size(280.0)
                .size_range(180.0..=520.0)
                .show_inside(ui, |ui| {
                    egui::ScrollArea::both()
                        .auto_shrink([false, false])
                        .show(ui, |ui| match self.left_panel {
                            LeftPanel::Navigator => self.navigator_panel(ui),
                            LeftPanel::DrawTools => self.draw_tools_panel(ui),
                        });
                });
        }

        // 右：パネル切替式（インスペクタ／準備計算／解析）。Zed のようにステータス
        // バーのアイコンで切り替える（切替自体は status_bar が行う）。準備計算・解析は
        // 3D ビューを見ながら設定・実行できるようここに置くため、他パネルより
        // 縦に長くなりがちで、右ドック全体をスクロール可能にする。
        if self.right_dock_open {
            egui::Panel::right("right_dock")
                .resizable(true)
                .default_size(320.0)
                .size_range(220.0..=560.0)
                .show_inside(ui, |ui| {
                    egui::ScrollArea::vertical()
                        .auto_shrink([false, false])
                        .show(ui, |ui| match self.right_panel {
                            RightPanel::Inspector => self.inspector_panel(ui),
                            RightPanel::Preparation => self.preparation_panel(ui),
                            RightPanel::Analysis => self.analysis_panel(ui),
                        });
                });
        }

        // 下（中央領域内）：タブ切替（ログ／モデル編集／荷重編集）。
        // 横長テーブルは幅の狭い左ドックより下ドックの方が視認性が良いため、
        // モデル/荷重の編集テーブルもここに収容する。
        // 左右ドックより後に show_inside することで、中央領域の下部（左右ドックの間）に出す。
        if self.bottom_dock_open {
            egui::Panel::bottom("bottom_dock")
                .resizable(true)
                .default_size(200.0)
                .size_range(80.0..=520.0)
                .show_inside(ui, |ui| {
                    ui.horizontal(|ui| {
                        let log_label = format!("ログ ({})", self.log.entries.len());
                        if ui
                            .selectable_label(self.bottom_tab == BottomTab::Log, log_label)
                            .clicked()
                        {
                            self.bottom_tab = BottomTab::Log;
                        }
                        if ui
                            .selectable_label(self.bottom_tab == BottomTab::Model, "モデル")
                            .clicked()
                        {
                            self.bottom_tab = BottomTab::Model;
                        }
                        if ui
                            .selectable_label(self.bottom_tab == BottomTab::Loads, "荷重")
                            .clicked()
                        {
                            self.bottom_tab = BottomTab::Loads;
                        }
                        // 準備計算タブ: 未実行・要再実行なら「*」を付けて再実行を促す。
                        let prep_label = if self.staleness.preparation_stale {
                            "準備計算 *"
                        } else {
                            "準備計算"
                        };
                        if ui
                            .selectable_label(self.bottom_tab == BottomTab::Preparation, prep_label)
                            .clicked()
                        {
                            self.bottom_tab = BottomTab::Preparation;
                        }
                        // 診断タブのラベル: 実行済みで Error/Warning があれば件数を付す
                        // （未実行・0件なら「診断」のみでラベルを騒がしくしない）。
                        let (diag_errors, diag_warnings) = self.diagnostics_counts();
                        let diag_label = if !self.staleness.diagnostics_stale
                            && (diag_errors > 0 || diag_warnings > 0)
                        {
                            format!("診断 (E{}/W{})", diag_errors, diag_warnings)
                        } else {
                            "診断".to_string()
                        };
                        if ui
                            .selectable_label(self.bottom_tab == BottomTab::Diagnostics, diag_label)
                            .clicked()
                        {
                            self.bottom_tab = BottomTab::Diagnostics;
                        }
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.button("✕").clicked() {
                                self.bottom_dock_open = false;
                            }
                            if self.bottom_tab == BottomTab::Log && ui.button("クリア").clicked()
                            {
                                self.log.entries.clear();
                            }
                            if self.bottom_tab == BottomTab::Diagnostics
                                && ui.button("再チェック").clicked()
                            {
                                self.run_diagnostics();
                            }
                        });
                    });
                    ui.separator();
                    // 診断タブを開いた時点で stale なら遅延実行する（編集の度に毎フレーム
                    // 走らせるとモデル/荷重編集操作が重くなるため）。
                    if self.bottom_tab == BottomTab::Diagnostics && self.staleness.diagnostics_stale
                    {
                        self.run_diagnostics();
                    }
                    match self.bottom_tab {
                        BottomTab::Log => {
                            if self.log.entries.is_empty() {
                                ui.colored_label(crate::theme::GRAY_600, "ログはまだありません");
                            } else {
                                // id_salt: 4タブは同一パネル内で切り替わるため、明示しないと
                                // ScrollArea の Id が衝突しスクロール位置がタブ間で共有される。
                                egui::ScrollArea::vertical()
                                    .id_salt("bottom_log")
                                    .auto_shrink([false, false])
                                    .stick_to_bottom(true)
                                    .show(ui, |ui| {
                                        for entry in &self.log.entries {
                                            let color = match entry.level {
                                                LogLevel::Error => crate::theme::ERROR_RED,
                                                LogLevel::Notice => crate::theme::BEST_YELLOW,
                                                LogLevel::Info => crate::theme::GRAY_700,
                                            };
                                            // 長いメッセージはパネル幅で折り返して全文を表示する
                                            // （見切れるとホバーしないと内容が分からないため）。
                                            // メッセージ自身が含む改行もそのまま活かす。
                                            ui.add(
                                                egui::Label::new(
                                                    egui::RichText::new(format!(
                                                        "[{}] {}",
                                                        entry.timestamp_label(),
                                                        entry.message
                                                    ))
                                                    .color(color),
                                                )
                                                .wrap(),
                                            );
                                        }
                                    });
                            }
                        }
                        BottomTab::Model => {
                            egui::ScrollArea::both()
                                .id_salt("bottom_model")
                                .auto_shrink([false, false])
                                .show(ui, |ui| self.model_tab_panel(ui));
                        }
                        BottomTab::Loads => {
                            egui::ScrollArea::both()
                                .id_salt("bottom_loads")
                                .auto_shrink([false, false])
                                .show(ui, |ui| crate::tables::loads::loads_table(ui, self));
                        }
                        BottomTab::Preparation => {
                            crate::prep_view::preparation_panel(ui, self);
                        }
                        BottomTab::Diagnostics => {
                            if self.diagnostics.is_empty() {
                                ui.colored_label(
                                    crate::theme::GOOD_GREEN,
                                    "問題は見つかりませんでした",
                                );
                            } else {
                                egui::ScrollArea::vertical()
                                    .id_salt("bottom_diag")
                                    .auto_shrink([false, false])
                                    .show(ui, |ui| {
                                        // インデックスで回す（クリック時に selection/nav を
                                        // 書き換えるため self を可変借用する必要があり、
                                        // diagnostics への不変参照と両立できない）。
                                        for i in 0..self.diagnostics.len() {
                                            let (color, icon, message, target) = {
                                                let d = &self.diagnostics[i];
                                                let color = match d.severity {
                                                    DiagSeverity::Error => crate::theme::ERROR_RED,
                                                    DiagSeverity::Warning => {
                                                        crate::theme::BEST_YELLOW
                                                    }
                                                    DiagSeverity::Info => crate::theme::GRAY_700,
                                                };
                                                let icon = match d.severity {
                                                    DiagSeverity::Error | DiagSeverity::Warning => {
                                                        "⚠"
                                                    }
                                                    DiagSeverity::Info => "ℹ",
                                                };
                                                (color, icon, d.message.clone(), d.target)
                                            };
                                            let text = egui::RichText::new(format!(
                                                "{} {}",
                                                icon, message
                                            ))
                                            .color(color);
                                            if let Some(target) = target {
                                                let resp = ui
                                                    .selectable_label(false, text)
                                                    .on_hover_text("クリックで 3D 選択");
                                                if resp.clicked() {
                                                    match target {
                                                        DiagTarget::Member(id) => {
                                                            self.selection.members = vec![id];
                                                            self.selection.nodes.clear();
                                                            self.nav.focus_member = Some(id);
                                                        }
                                                        DiagTarget::Node(id) => {
                                                            self.selection.nodes = vec![id];
                                                            self.selection.members.clear();
                                                            self.nav.focus_node = Some(id);
                                                        }
                                                    }
                                                }
                                            } else {
                                                ui.label(text);
                                            }
                                        }
                                    });
                            }
                        }
                    }
                });
        }

        // 中央：モデル/荷重/解析タブでは常に3Dビュー（作成状況・モデルを見ながら
        // 設定・実行できるようにする。解析の設定フォームは右ドック側にある）。
        // それ以外の工程タブは各内容を表示する。
        egui::CentralPanel::default().show_inside(ui, |ui| match self.active_tab {
            Tab::Model | Tab::Loads | Tab::Analysis => crate::viewer::viewer_panel(ui, self),
            Tab::Results => self.results_tab_panel(ui),
            Tab::Design => self.design_tab_panel(ui),
            Tab::Report => self.report_tab_panel(ui),
        });
    }
}

/// 要素の構造種別が鋼系（S・CFT）か。
///
/// 判定は断面と材料から `squid_n_core::structure_kind` が行う。断面形状は
/// 見た目であって力学的な性質ではないため、材料の区分を主体に判定する。
fn elem_is_steel(
    elem: &squid_n_core::model::ElementData,
    model: &squid_n_core::model::Model,
) -> bool {
    squid_n_core::structure_kind::member_structure_kind(model, elem).is_steel_like()
}

#[cfg(test)]
mod tests;
