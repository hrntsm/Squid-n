//! 階（層）関連の型。
//!
//! **階と剛床は別の概念である。** 階は法規上の層（層間変形角・層せん断力・
//! 剛性率・偏心率が対象とする区画）であり、剛床は解析上の面内剛体拘束である。
//! 階が剛床を持たないことも、1 つの階が複数の剛床を持つこと（段差床）もある。
//! そのため剛床は階の一部としてではなく、拘束
//! （[`Constraint::RigidDiaphragm`]）として単一の情報源に保持する。
//! 階から剛床を引くときは [`Model::diaphragms_of`] を使う。
//!
//! - [`DiaphragmRef`] — 階に属する剛床の参照ビュー。
//! - [`StoryStructure`] — 階の主要構造種別。
//! - [`StoryLevelKind`] — 階の種別（一般／PH／地下）。
//! - [`Story`] — 階の定義。

use super::*;

/// 剛床のレベル許容差 [mm]。剛床のスレーブ節点は「階のレベルからこの範囲内に
/// ある節点」とする（[`Model::on_diaphragm_level`]）。
///
/// 階への帰属（[`Model::story_of_elevation`]）が**区間**であるのに対し、剛床への
/// 帰属は**床面**である。中間高さの節点（柱の分割点・階高の途中に取り付く梁）は
/// 階には属するが剛床には入らない。面内剛体として拘束してよいのは同一床面の
/// 節点だけであり、中間節点を含めると存在しない水平剛性が生じるためである。
pub const DIAPHRAGM_LEVEL_TOL_MM: f64 = 1.0;

/// 階名が与えられていないときの既定の階名（**床基準**）。
///
/// 階は床を代表する（[`Story::elevation`] はその階が代表する床のレベル）ため、
/// 階名も床の呼び名に合わせる。下から `index` 番目（0 始まり）の階は基部の 1 つ上の
/// 床であり、基部を 1FL とみなして `2F`・`3F` … と付ける。
///
/// 最上階も `RF` とはせず数字で通す。モデルの最上レベルが本当に屋根なのかは
/// モデルからは決められず（塔屋の床であることも、あとで上へ階を足すこともある）、
/// 確定していないものを屋根と名乗らせないためである。屋根であれば利用者が
/// `RF` へ付け替える。
///
/// ST-Bridge の `StbStory` も床基準（`1F` の `height` が GL）であるため、
/// 取り込んだモデルとアプリ内で作ったモデルで階名の意味が一致する。
pub fn default_story_name(index: usize) -> String {
    format!("{}F", index + 2)
}

/// 階に属する剛床の参照ビュー（[`Constraint::RigidDiaphragm`] の内容）。
///
/// 剛床の実体は拘束として保持されるため、階から剛床を辿る側は本ビューを介する
/// （[`Model::diaphragms_of`]）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DiaphragmRef<'a> {
    pub story: StoryId,
    pub master: NodeId,
    pub slaves: &'a [NodeId],
    /// この剛床が負担する地震用重量 [N]。多剛床の階では層の水平力 Pi を
    /// 剛床ごとの重量比で分配するために用いる（多剛床の設計用せん断力。
    /// 令88条・昭55建告1793号）。None は未算定（階に単一剛床なら層重量全量）。
    pub weight: Option<f64>,
    /// 副剛床の層せん断力係数 Ci の直接入力（令88条・昭55建告1793号の
    /// 層せん断力係数）。Some の剛床は主系統の Ai 分布から
    /// 除外され、水平力 = ci_override × 剛床重量（等価震度扱い。上階に同一系統の
    /// 剛床が積み上がらない副剛床を想定）として作用する。None は主系統（Ai 分布）。
    pub ci_override: Option<f64>,
}

/// 動的解析（固有値・時刻歴・精算周期）の質量モデルの方式。
///
/// 階の自動生成が剛床マスター節点へ与える質点質量（[`super::Node::mass`]）の
/// 算定方法と、解析側の全体質量行列の組立方法（部材密度による分布質量を
/// 含めるか）の両方を規定する。生成と組立で方式が食い違うと自重の二重計上や
/// 質量欠落が起きるため、モデル自身（[`super::Model::mass_method`]）が
/// 単一情報源として保持し、双方がこれを参照する。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum MassMethod {
    /// 補正質点方式（既定）: 部材密度による分布質量に加え、剛床マスターへ
    /// 「地震用重量のうち分布質量として計上されない分」（床・仕上げ・積載・
    /// 二次部材・雑壁など）を補正質点として与える。階の合計質量が地震用重量に
    /// 一致し、部材の分布質量による鉛直・局部の振動モードも保たれる。
    #[default]
    CorrectedLumped,
    /// 質点のみ方式: 質量は節点質量（剛床マスターへ与えた地震用重量の質点等）
    /// のみを用い、部材密度による分布質量は質量行列に算入しない
    /// （実務の水平質点系モデル化。鉛直方向・局部の振動モードは表現されない）。
    LumpedOnly,
}

/// 階の主要構造種別。設計用一次固有周期の略算式 T=h(0.02+0.01α) の
/// α（柱梁の大部分が鉄骨造である階の高さ比）の算定に用いる（令88条・告示1793号）。
///
/// 値は階に属する柱・梁の構造種別から自動判定する（準備計算の階生成。
/// [`StoryStructure::of_structure_kind`]）ため、利用者は入力しない。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum StoryStructure {
    #[default]
    Rc,
    S,
    Src,
}

impl StoryStructure {
    /// 部材の構造種別を階の構造種別へ畳み込む。
    ///
    /// CFT は SRC へ寄せる。略算周期 T = h(0.02 + 0.01α) は S の階が増えるほど
    /// T が長く Rt が小さくなって地震力が下がるため、鋼管にコンクリートを充填した
    /// CFT を S へ寄せないのが安全側になる。
    pub fn of_structure_kind(kind: crate::structure_kind::StructureKind) -> Self {
        use crate::structure_kind::StructureKind;
        match kind {
            StructureKind::Rc => StoryStructure::Rc,
            StructureKind::S => StoryStructure::S,
            StructureKind::Src | StructureKind::Cft => StoryStructure::Src,
        }
    }

    /// 種別ごとの部材本数から階の主要構造種別を決める。最多の種別を採用し、
    /// 同数の場合は RC → SRC → S の順で優先する。
    ///
    /// 略算周期 T = h(0.02 + 0.01α) は S の階が増えるほど T が長く、Rt が
    /// 小さくなって地震力が下がるため、判定が割れた場合は S を採らないのが
    /// 安全側になる（同数時の優先順の根拠）。対象部材が 1 本もない階は RC。
    pub fn majority(n_rc: usize, n_s: usize, n_src: usize) -> Self {
        let max = n_rc.max(n_s).max(n_src);
        if max == 0 || n_rc == max {
            StoryStructure::Rc
        } else if n_src == max {
            StoryStructure::Src
        } else {
            StoryStructure::S
        }
    }
}

/// 階の種別。地震層せん断力の算定方法を切り替える
/// （一般階=Ai分布、PH階=震度 k、地下階=水平震度 K）。
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum StoryLevelKind {
    #[default]
    Normal,
    /// 塔屋（PH）階。層せん断力 Qi = k·ΣWj（k は 0.5〜1.0 の指定震度）。
    Penthouse { k: f64 },
    /// 地下階。Qi = Q(i+1) + K·Wi、K = 0.1·(1 − H/40)·Z（H は地盤面からの深さ[m]、20m 超は 20m）。
    Basement { depth_m: f64 },
}

/// 階（法規上の層）の定義。
///
/// フィールドは**誰が決めるか**で 2 系統に分かれる。
///
/// - **利用者が決める**: [`Self::name`]・[`Self::elevation`]・[`Self::level_kind`]・
///   [`Self::weight_override`]。新規作成時の入力、または ST-Bridge の `StbStory`
///   から入り、準備計算では書き換えない。
/// - **準備計算が埋める**: [`Self::node_ids`]・[`Self::seismic_weight`]・
///   [`Self::structure`]。節点と部材が確定してはじめて決まる派生値であり、
///   階生成のたびに算定し直す。
///
/// [`Model::stories`] は [`Self::elevation`] の**昇順**に並ぶ（階への帰属区間が
/// 直下階のレベルで決まるため、この並びが崩れると帰属が壊れる）。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Story {
    pub id: StoryId,
    /// 階名（`2F`・`PH1` など）。利用者が決める自由文字列で、階の識別に用いる。
    pub name: String,
    /// 階のレベル [mm]。**その階が代表する床のレベル**であり、階への帰属区間の
    /// 上端でもある（[`Model::story_of_elevation`]）。
    pub elevation: f64,
    /// この階に属する節点（準備計算が [`Model::story_of_elevation`] で埋める）。
    pub node_ids: Vec<NodeId>,
    /// 設計に用いる地震用重量 [N]。準備計算の階生成が自動算定値を書き込むが、
    /// [`Self::weight_override`] が `Some` の場合はその手入力値が入る
    /// （解析・設計側はこのフィールドだけを読めばよい）。
    pub seismic_weight: Option<f64>,
    /// 地震用重量の手入力値 [N]。`Some` のときは準備計算で階を再生成しても
    /// 保持され、[`Self::seismic_weight`] へ優先して反映される。`None` は
    /// 自動算定値をそのまま用いる。旧スキーマは手入力なし扱い。
    #[serde(default)]
    pub weight_override: Option<f64>,
    /// 主要構造種別（略算周期の鉄骨造比 α 算定用）。断面形状からの自動判定値。
    /// 旧スキーマは RC 扱い。
    #[serde(default)]
    pub structure: StoryStructure,
    /// 階の種別（一般/PH/地下）。旧スキーマは一般階扱い。
    #[serde(default)]
    pub level_kind: StoryLevelKind,
}

impl Model {
    /// 建物の基部レベル [mm]（階への帰属区間の下端であり、`elevation` の基準 0）。
    ///
    /// 全構造節点（`generated_masters` ＝階生成が作る剛床代表節点を除く）の最小 Z
    /// 座標を基部とする。剛床代表節点は慣性力重心に置かれる仮想節点であり、実際の
    /// 構造高さには寄与しないため除外する。節点がない場合は 0 を返す。
    pub fn base_elevation(&self) -> f64 {
        let excluded: std::collections::HashSet<NodeId> =
            self.generated_masters.iter().copied().collect();
        let base = self
            .nodes
            .iter()
            .filter(|n| !excluded.contains(&n.id))
            .map(|n| n.coord[2])
            .fold(f64::INFINITY, f64::min);
        if base.is_finite() {
            base
        } else {
            0.0
        }
    }

    /// 各階への帰属区間 `(下端, 上端]` [mm]（[`Self::stories`] と同順・同長）。
    ///
    /// 下端は直下階のレベル（最下階は [`Self::base_elevation`]）、上端は当該階の
    /// レベルである。**下端は含まず上端を含む**ため、基部の節点（支点）はどの階にも
    /// 属さず、床レベルちょうどの節点はその階に属する。
    ///
    /// 区間の算出はここに集約する（[`Self::base_elevation`] が全節点の走査を伴う
    /// ため、節点ごとに区間を求め直すと二乗の計算量になる）。
    pub fn story_spans(&self) -> Vec<(f64, f64)> {
        let base = self.base_elevation();
        self.stories
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let bottom = if i == 0 {
                    base
                } else {
                    self.stories[i - 1].elevation
                };
                (bottom, s.elevation)
            })
            .collect()
    }

    /// レベル `z` [mm] が属する階を、[`Self::story_spans`] の区間列から引く。
    ///
    /// 階への帰属は**区間**である。中間高さの節点や段差床の節点も、区間に入れば
    /// 当該階に属する。剛床への帰属とは規則が異なる（[`Self::on_diaphragm_level`]）。
    /// どの区間にも入らない場合は `None`（基部レベル以下、または最上階より上）。
    pub fn story_at(&self, spans: &[(f64, f64)], z: f64) -> Option<StoryId> {
        spans
            .iter()
            .position(|&(bottom, top)| z > bottom && z <= top)
            .and_then(|i| self.stories.get(i))
            .map(|s| s.id)
    }

    /// 各節点の所属階（[`Self::nodes`] と同順・同長）。
    ///
    /// 階への帰属規則（区間）の単一情報源。準備計算の階生成も UI の表示も
    /// これを用いる。
    pub fn node_stories(&self) -> Vec<Option<StoryId>> {
        let spans = self.story_spans();
        self.nodes
            .iter()
            .map(|n| self.story_at(&spans, n.coord[2]))
            .collect()
    }

    /// レベル `z` [mm] が階 `story` の床面上にあるか（剛床のスレーブ判定）。
    ///
    /// 判定は階のレベルからの差が [`DIAPHRAGM_LEVEL_TOL_MM`] 以内かどうかで、
    /// 階への帰属（区間）とは規則が異なる。
    pub fn on_diaphragm_level(&self, story: StoryId, z: f64) -> bool {
        self.stories
            .get(story.index())
            .is_some_and(|s| (z - s.elevation).abs() <= DIAPHRAGM_LEVEL_TOL_MM)
    }

    /// 階 `story` に属する剛床（[`Constraint::RigidDiaphragm`]）を定義順に返す。
    ///
    /// 剛床は階の一部ではなく拘束として保持されるため、「この階の剛床」が要る
    /// ところは常にこのヘルパーを情報源とする。
    pub fn diaphragms_of(&self, story: StoryId) -> impl Iterator<Item = DiaphragmRef<'_>> {
        self.constraints.iter().filter_map(move |c| match c {
            Constraint::RigidDiaphragm {
                story: s,
                master,
                slaves,
                weight,
                ci_override,
            } if *s == story => Some(DiaphragmRef {
                story: *s,
                master: *master,
                slaves,
                weight: *weight,
                ci_override: *ci_override,
            }),
            _ => None,
        })
    }

    /// 節点 `id` がいずれかの剛床のマスターまたはスレーブか。
    pub fn node_on_rigid_diaphragm(&self, id: NodeId) -> bool {
        self.constraints.iter().any(|c| match c {
            Constraint::RigidDiaphragm { master, slaves, .. } => {
                *master == id || slaves.contains(&id)
            }
            _ => false,
        })
    }
}
