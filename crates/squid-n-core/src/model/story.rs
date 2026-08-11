//! 階・層関連の型。
//!
//! **階（[`Story`]）は床であり、層（[`Layer`]）は隣り合う 2 つの床の間である。**
//! [`Model::stories`] は基部の床から屋根の床までの床レベル列で、
//! **先頭は必ず基部**（`stories[0].elevation == ` [`Model::base_elevation`]）。
//! これを不変条件とし、階生成（`squid_n_load::story_gen`）が必ず成立させる。
//! したがって層の数は階の数より 1 つ少ない。
//!
//! 法規上の「i 階」（層間変形角・層せん断力・剛性率・偏心率が対象とする区画）は
//! 層のことであり、[`Model::layers`] が唯一の情報源である。層を数える処理は
//! [`Model::stories`] を直接走査してはならない（1 つ多く数えてしまう）。
//!
//! 層と床の対応は実務の慣行に合わせる（[`Layer`] 参照）。層の名前は**下端床**、
//! 重量・所属節点・階種別は**上端床**から採る。層 i の重量 Wi が上端床の重量なのは、
//! `Q1 = C1·(W1+W2+…)` が成り立つためである（基部の重量は地盤が直接受けるので
//! 最下層のせん断力に寄与しない）。
//!
//! **階と剛床も別の概念である。** 階が剛床を持たないことも、1 つの階が複数の
//! 剛床を持つこと（段差床）もある。そのため剛床は階の一部としてではなく、拘束
//! （[`Constraint::RigidDiaphragm`]）として単一の情報源に保持する。
//! 階から剛床を引くときは [`Model::diaphragms_of`] を使う。
//!
//! - [`DiaphragmRef`] — 階に属する剛床の参照ビュー。
//! - [`StoryStructure`] — 階の主要構造種別。
//! - [`StoryLevelKind`] — 層の種別（一般／PH／地下）。
//! - [`Story`] — 階（床）の定義。
//! - [`Layer`] — 層（階と階の間）。[`Model::layers`] が組み立てる導出値。

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
/// 階は床そのものである（[`Story::elevation`] はその階が代表する床のレベル）ため、
/// 階名も床の呼び名に合わせる。下から `index` 番目（0 始まり）の階は
/// `index == 0` が基部の床であり、`1F`・`2F` … と付ける。
///
/// 最上階も `RF` とはせず数字で通す。モデルの最上レベルが本当に屋根なのかは
/// モデルからは決められず（塔屋の床であることも、あとで上へ階を足すこともある）、
/// 確定していないものを屋根と名乗らせないためである。屋根であれば利用者が
/// `RF` へ付け替える。
///
/// ST-Bridge の `StbStory` も床基準（`1F` の `height` が GL）であるため、
/// 取り込んだモデルとアプリ内で作ったモデルで階名の意味が一致する。
pub fn default_story_name(index: usize) -> String {
    format!("{}F", index + 1)
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

/// 層の種別。地震層せん断力の算定方法を切り替える
/// （一般階=Ai分布、PH階=震度 k、地下階=水平震度 K）。
///
/// 層の属性だが、保持先は層の**上端床**の [`Story`] である（重量と同じ場所に
/// 集約する。[`Layer`] 参照）。最下の階（基部の床）はどの層の上端でもないため、
/// そこに設定された値は用いられない。
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum StoryLevelKind {
    #[default]
    Normal,
    /// 塔屋（PH）階。層せん断力 Qi = k·ΣWj（k は 0.5〜1.0 の指定震度）。
    Penthouse { k: f64 },
    /// 地下階。Qi = Q(i+1) + K·Wi、K = 0.1·(1 − H/40)·Z（H は地盤面からの深さ[m]、20m 超は 20m）。
    Basement { depth_m: f64 },
}

/// 階（床）の定義。法規上の「層」は [`Layer`] である。
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
/// [`Model::stories`] は [`Self::elevation`] の**昇順**に並び、**先頭は基部の床**
/// （[`Model::base_elevation`] と同レベル）である。階への帰属区間が直下階のレベルで
/// 決まるため、この並びが崩れると帰属が壊れる。
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

/// 層（隣り合う 2 つの階の間）。法規上の「i 階」はこれを指す。
///
/// [`Model::layers`] が [`Model::stories`] から組み立てる**導出値**であり、
/// モデルには保持しない（保持すると [`Story::node_ids`] と同種の同期ずれを
/// 1 つ増やすことになる）。層を数える処理は必ずこれを介し、
/// [`Model::stories`] を層として直接走査してはならない。
///
/// 層と床の対応は実務の慣行に従う。
///
/// | 量 | 由来 |
/// |---|---|
/// | 名前 | 下端床（法令の「i 階」は下の床の呼び名） |
/// | 階高 | 上端床の標高 − 下端床の標高 |
/// | 重量・所属節点・階種別 | **上端床** |
///
/// 重量が上端床なのは、層の質量が上端の床に集中するためである。
#[derive(Clone, Debug, PartialEq)]
pub struct Layer {
    /// 下から 0 始まりの層の番号。層を識別する添字。
    pub index: usize,
    /// 層の名前（下端床の階名）。
    pub name: String,
    /// 下端の階（床）。
    pub bottom: StoryId,
    /// 上端の階（床）。層の重量・所属節点・階種別はこの階が持つ。
    pub top: StoryId,
    /// 層の高さ（階高）[mm]。
    pub height: f64,
    /// 下端床の標高 [mm]。
    pub bottom_elevation: f64,
    /// 上端床の標高 [mm]。
    pub top_elevation: f64,
    /// 層の種別（一般/PH/地下）。
    pub level_kind: StoryLevelKind,
    /// 設計に用いる地震用重量 [N]（未算定なら `None`）。
    pub weight: Option<f64>,
    /// 層に属する節点（＝上端床の所属節点）。
    pub node_ids: Vec<NodeId>,
    /// 主要構造種別（略算周期の鉄骨造比 α 算定用）。
    pub structure: StoryStructure,
}

impl Model {
    /// 層（[`Layer`]）の一覧を下から順に返す。**層を数える処理の唯一の入口**。
    ///
    /// 階が床レベル列であるという不変条件（モジュールドキュメント参照）から、
    /// 層は隣り合う階の対そのものであり、層数は `stories.len() - 1` である。
    /// 階が 1 つ以下のモデルでは空を返す。
    pub fn layers(&self) -> Vec<Layer> {
        self.stories
            .windows(2)
            .enumerate()
            .map(|(i, w)| {
                let (bottom, top) = (&w[0], &w[1]);
                Layer {
                    index: i,
                    name: bottom.name.clone(),
                    bottom: bottom.id,
                    top: top.id,
                    height: top.elevation - bottom.elevation,
                    bottom_elevation: bottom.elevation,
                    top_elevation: top.elevation,
                    level_kind: top.level_kind,
                    weight: top.seismic_weight,
                    node_ids: top.node_ids.clone(),
                    structure: top.structure,
                }
            })
            .collect()
    }

    /// 層の数（`stories.len() - 1`、階が 1 つ以下なら 0）。
    ///
    /// [`Self::layers`] を組み立てずに個数だけ要るときに使う。
    pub fn layer_count(&self) -> usize {
        self.stories.len().saturating_sub(1)
    }

    /// 建物の基部レベル [mm]（`elevation` の基準 0）。**幾何としての基部**。
    ///
    /// 全構造節点（`generated_masters` ＝階生成が作る剛床代表節点を除く）の最小 Z
    /// 座標を基部とする。剛床代表節点は慣性力重心に置かれる仮想節点であり、実際の
    /// 構造高さには寄与しないため除外する。節点がない場合は 0 を返す。
    ///
    /// 不変条件が成立していれば `stories[0].elevation` と一致する。にもかかわらず
    /// 節点から求めるのは、**階生成が不変条件を成立させる側**だからである。階生成は
    /// まだ床基準になっていない階列（あるいは階が 1 つもないモデル）から床レベル列を
    /// 組み立てるため、そのブートストラップには階に依らない基部が要る。
    /// 帰属区間（[`Self::story_spans`]）は不変条件を前提とするのでこれを呼ばない。
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
    /// 下端は直下階のレベル、上端は当該階のレベルである。**下端は含まず上端を含む**
    /// ため、床レベルちょうどの節点はその階に属し、中間高さの節点は直上の階に属する。
    ///
    /// **最下階（基部の床）だけは下端を含む点区間** `[基部, 基部]` とする。
    /// 不変条件により最下階の標高は基部レベルそのものであり、`(下端, 上端]` の規則を
    /// そのまま当てはめると空区間になって柱脚・基礎梁の節点がどの階にも属さなくなる
    /// ためである。
    ///
    /// 区間の算出はここに集約する。
    ///
    /// 最下階の下端は `stories[0].elevation` と [`Self::base_elevation`] の**小さい方**
    /// とする。不変条件が成立していれば両者は一致するので通常は前者そのものだが、
    /// 不変条件がまだ成立していないモデル（階生成を通していない旧形式のファイル、
    /// 基部の階を持たない取り込みデータ）では基部側が下端になり、基部〜最下階の
    /// 節点が最下階へ収まる。これがないと、そうしたモデルで最下階の伏図が空になり、
    /// 節点が丸ごとどの階にも属さなくなる。
    pub fn story_spans(&self) -> Vec<(f64, f64)> {
        let first_bottom = self
            .stories
            .first()
            .map(|s| s.elevation.min(self.base_elevation()));
        self.stories
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let bottom = match i {
                    0 => first_bottom.unwrap_or(s.elevation),
                    _ => self.stories[i - 1].elevation,
                };
                (bottom, s.elevation)
            })
            .collect()
    }

    /// レベル `z` [mm] が属する階を、[`Self::story_spans`] の区間列から引く。
    ///
    /// 階への帰属は**区間**である。中間高さの節点や段差床の節点も、区間に入れば
    /// 当該階に属する。剛床への帰属とは規則が異なる（[`Self::on_diaphragm_level`]）。
    /// どの区間にも入らない場合は `None`（基部レベル未満、または最上階より上）。
    ///
    /// 区間列は標高の昇順で連続しているため二分探索で引く（伏図の描画が毎フレーム
    /// 全節点に対して呼ぶため、線形探索では階数に比例して重くなる）。
    pub fn story_at(&self, spans: &[(f64, f64)], z: f64) -> Option<StoryId> {
        // 上端が z 以上になる最初の区間を探す。区間は上端の昇順に並ぶ。
        let i = spans.partition_point(|&(_, top)| top < z);
        let &(bottom, top) = spans.get(i)?;
        // 最下階だけは下端を含む点区間（[基部, 基部]）。
        let above_bottom = if i == 0 { z >= bottom } else { z > bottom };
        if !above_bottom || z > top {
            return None;
        }
        self.stories.get(i).map(|s| s.id)
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
