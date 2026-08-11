//! 要素（部材）の型。
//!
//! - [`ElementKind`] — 要素種別（梁・シェル・ブレース・免震・ダンパー等）。
//! - [`ForceRegime`] — 応力評価の方式。
//! - [`LocalAxis`] — 部材ローカル軸の基準ベクトル。
//! - [`EndCondition`] — 部材端の接合条件。
//! - [`ZoneSource`] — 剛域長の出所（自動／手動）。
//! - [`RigidZone`] — 部材端の剛域（剛域長・フェイス距離）。
//! - [`ElementData`] — 要素の永続化データ。

use super::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ElementKind {
    Beam,
    Shell,
    /// ファイバー梁要素（積分点断面のファイバー分割による分布塑性モデル）。
    Fiber,
    /// マルチスプリング梁要素（端部塑性化域を軸ばね群で置換したモデル）。
    MultiSpring,
    Wall,
    PanelZone,
    /// 一般ブレース（軸材。軸剛性のみのトラス要素。材料力学）。
    /// 剛性は軸剛性のみのトラス要素（KB=E·A/L）で評価する。
    /// K 型ブレースの重量配分規則（`LoadCfg::k_brace_rule`）の適用対象。
    /// `tension_only`: 引張専用ブレースか（true の場合、弾性解析では剛性を1/2に
    /// モデル化する。弾塑性解析では初期剛性は1倍。本実装既定の「引張と圧縮が
    /// 対で存在するとみなす」モデル化）。
    Brace {
        tension_only: bool,
    },
    /// 節点バネ要素（ばね要素の変形と自由度。構造力学）。
    ///
    /// 部材の変形と自由度の考え方では、節点バネは θX=―（非考慮）、
    /// θY=○, θZ=○, γY=○, γZ=○, δX=○。すなわちねじり以外の曲げ・せん断・
    /// 軸方向の変形成分を独立なバネ剛性として持ちうる 2 節点要素。
    /// 各自由度のバネ定数は `ElementData::spring` に保持する（局所軸 6 成分）。
    NodalSpring,
    /// 免震支承材（各免震部材指針）。
    /// 2 節点要素で、水平は非線形せん断ばね（マルチシアスプリング＝積層ゴム系
    /// バイリニア、または摩擦ばね＝弾性すべり支承 Qmax=μN）、鉛直は弾性軸ばね。
    /// 特性は `Model::isolator_attrs` に要素 ID と対で保持する。
    Isolator,
    /// 制振ダンパー要素（各制振部材の力学モデル）。
    /// 2 節点の軸方向要素で、マクスウェル要素（バネ Kd と粘性ダッシュポットの直列）等で
    /// モデル化する。減衰要素の要素力は節点力として運動方程式へ与えられ、特性は
    /// `Model::damper_attrs` に要素 ID と対で保持する。
    Damper,
}

impl ElementKind {
    /// 剛性の算定に断面（`ElementData::section`）の割当が必須な要素種別か。
    /// 断面・材料の未割当を検出する検査は、必ず本判定で対象を絞ること。
    ///
    /// **材料は断面が持つ**（`Section::material`）ため、部材に要るのは断面の割当
    /// だけである。断面が材料を持たない場合も剛性を作れないので、検査は「断面が
    /// 割り当てられているか」と「その断面が材料を持つか」の 2 段になる
    /// （`Model::element_material`）。
    ///
    /// 必須なのは線材（梁・ファイバー梁・マルチスプリング梁・ブレース）と
    /// 面材（シェル・壁）で、断面諸元と材料定数から剛性を作るため、いずれかが
    /// 未割当だとゼロ剛性となり解析が成立しない。
    ///
    /// 一方、次の要素は断面を持たないのが正常な状態であり、未割当として
    /// 扱ってはならない。
    /// - 仕口パネル（`PanelZone`）: 剛性は取り付く柱・梁の断面から求めた実効体積 Ve
    ///   による。準備計算が自動生成するため、未割当として警告すると生成数だけ
    ///   警告が並び、本当に割当が漏れている部材が埋もれる。
    /// - 節点バネ（`NodalSpring`）: 剛性は `ElementData::spring`（局所軸 6 成分）。
    /// - 免震支承材（`Isolator`）・制振ダンパー（`Damper`）: 特性は
    ///   `Model::isolator_attrs`・`Model::damper_attrs` に持つ。
    ///
    /// 要素種別を追加したときに扱いを決め忘れないよう、網羅 `match` で書く
    /// （ワイルドカードを使わない）。
    pub fn requires_section_and_material(self) -> bool {
        match self {
            ElementKind::Beam
            | ElementKind::Fiber
            | ElementKind::MultiSpring
            | ElementKind::Brace { .. }
            | ElementKind::Shell
            | ElementKind::Wall => true,
            ElementKind::PanelZone
            | ElementKind::NodalSpring
            | ElementKind::Isolator
            | ElementKind::Damper => false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ForceRegime {
    UniaxialBendingShear,
    AxialBendingInteract,
    Auto,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LocalAxis {
    pub ref_vector: [f64; 3],
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum EndCondition {
    Fixed,
    Pinned,
    SemiRigid { k_theta: f64 },
}

/// 梁（水平材）のねじり剛性の扱い（建物一律のモデル化方針）。
///
/// 日本の一貫計算プログラムでは、床と一体になる大梁のねじり剛性を設計上
/// 期待しないのが通例のため、既定は「i 端のねじれを解放する」とする。
/// 判定と例外（解放すると材軸まわり回転が浮く節点がある部材は解放しない）は
/// `squid_n_element::beam::i_end_torsion_release` を参照。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum BeamTorsionMode {
    /// 水平材の i 端ねじれをピン（解放）とし、梁のねじり剛性を期待しない（既定）。
    #[default]
    ReleaseIEnd,
    /// ねじり剛性 GJ/L を両端で保持する。ねじりで釣り合わせるモデル化
    /// （床小梁の格子解析の剛接十字など）で用いる。
    Keep,
}

/// 仕口パネル（柱梁接合部パネル）のモデル化（建物一律のモデル化方針）。
///
/// 有効にすると、S 造（CFT を除く）の柱梁接合節点へ仕口パネル要素を設け、
/// 接合部のせん断変形を解析へ反映する。パネルが設けられた節点はせん断変形角
/// `γX`・`γY` の 2 自由度を追加で持ち、その節点へ取り付く部材はパネル寸法分の
/// オフセット位置で接合する。
///
/// 対象を S 造に限るのは、S 造の接合部が剛域長 0（`squid_n_element::beam` の
/// 剛域自動算定は RC/SRC の直交材のみを探す）であり、パネルのせん断変形を
/// 明示的に評価しても剛域と二重計上にならないため。RC・SRC の接合部は
/// 従来どおり剛域で接合部の有限寸法を評価する。
///
/// CFT も対象外とする。充填コンクリートと通しダイアフラムが接合部のせん断挙動へ
/// 関与し、鋼管のみの実効体積による弾性せん断パネルでは剛性を表せないため
/// （`squid_n_core::panel_zone::PanelJoint::has_filled_column`）。
///
/// 検定（S 造パネルゾーンの断面検定）は本設定によらず常に実行し、**CFT も
/// 検定の対象に含める**（モデル化と検定で対象範囲が異なる）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PanelZoneMode {
    /// 仕口パネルをモデル化する（既定）。
    #[default]
    Model,
    /// 仕口パネルをモデル化しない（接合部を剛節点として扱う従来のモデル化）。
    None,
}

impl PanelZoneMode {
    /// モデル化が有効か。
    pub fn is_enabled(self) -> bool {
        matches!(self, PanelZoneMode::Model)
    }
}

/// 剛域長の出所。Auto は再算定で上書きされる、Manual は保護される（設計書 §6.2.1）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ZoneSource {
    Auto,
    Manual,
}

/// 部材端の剛域（接合部の有限寸法）。可とう長 L' = L − 剛体アーム長。
/// 力学計算は sc-element 側。ここではモデルに保持・永続化するデータ。
///
/// **次の 3 つは別概念**（設計書 §6.2.1、計算根拠 4.1.4・4.1.5）。
/// - `length_i/j`: 剛域の自動算定・手動指定による剛域長 `λ = Lf − D_self/4`。
///   壁の考慮など、モデル化の設定によって変わりうる。
/// - `panel_offset_i/j`: 仕口パネルを設けた接合部で、部材がパネル面まで離れて
///   接合することによるオフセット。部材配置から決まる幾何量。
/// - `face_i/j`: 断面算定・危険断面位置（§6.2.3）に使う柱フェース距離 `D_orth/2`。
///   接合関係と断面せいだけで一意に決まる幾何量で、剛域長の設定には左右されない。
///
/// 剛性計算に効く**剛体アームの長さ**は [`Self::rigid_length_i`] /
/// [`Self::rigid_length_j`] で取る。剛域長とパネルオフセットの大きい方になる。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RigidZone {
    pub length_i: f64,
    pub length_j: f64,
    pub source_i: ZoneSource,
    pub source_j: ZoneSource,
    /// 柱フェース距離 [mm]（節点→フェース、= 接合する直交部材せい/2）。
    ///
    /// **`None` は「まだ算定していない」を表す**（直交材がなくフェース距離が
    /// 0 の端は `Some(0.0)`）。両者を 0 で混同すると、算定前に読んだ側が
    /// 「フェース距離 0＝節点間長」として計算を進めてしまい、危険断面位置や
    /// RC/SRC 梁の自重が静かに誤る。算定は
    /// [`crate::face_distance::apply_face_distances`]（幾何のみ・冪等）。
    ///
    /// 読み出しは [`Self::face_i_or_zero`]（表示用）と
    /// [`Self::clear_span_from`]（計算用）を使うこと。
    #[serde(default)]
    pub face_i: Option<f64>,
    /// 柱フェース距離 [mm]（j端）。意味は `face_i` と同様。
    #[serde(default)]
    pub face_j: Option<f64>,
    /// 仕口パネル分のオフセット [mm]（i 端）。パネルがない端は 0。
    ///
    /// 剛域長 `length_i` とは**別に保持する**。剛域の自動算定
    /// （`apply_auto_rigid_zones`）は `Auto` 端の `length_i` を無条件に再算定するため、
    /// 同じ場所へ入れると解析経路によってオフセットが消える。
    #[serde(default)]
    pub panel_offset_i: f64,
    /// 仕口パネル分のオフセット [mm]（j 端）。意味は `panel_offset_i` と同様。
    #[serde(default)]
    pub panel_offset_j: f64,
}

impl Default for RigidZone {
    fn default() -> Self {
        Self {
            length_i: 0.0,
            length_j: 0.0,
            source_i: ZoneSource::Auto,
            source_j: ZoneSource::Auto,
            face_i: None,
            face_j: None,
            panel_offset_i: 0.0,
            panel_offset_j: 0.0,
        }
    }
}

impl RigidZone {
    /// i 端の柱フェース距離 [mm]。**未算定は 0 として扱う**。
    ///
    /// 一覧表示・図など、未算定でも破綻しない用途にだけ使うこと。計算には
    /// [`Self::clear_span_from`] を使い、未算定を検出できるようにする。
    pub fn face_i_or_zero(&self) -> f64 {
        self.face_i.unwrap_or(0.0)
    }

    /// j 端の柱フェース距離 [mm]。意味は [`Self::face_i_or_zero`] と同様。
    pub fn face_j_or_zero(&self) -> f64 {
        self.face_j.unwrap_or(0.0)
    }

    /// 節点間長 `geom_len` から両端の柱フェース距離を差し引いた**内法長** [mm]。
    ///
    /// 両端とも算定済みのときだけ `Some` を返す。未算定（`None`）のまま
    /// 節点間長で代用すると、RC/SRC 梁の自重・数量・終局耐力が過大になるため、
    /// 呼び出し側に未算定を気づかせる。負にはならないよう 0 で下限を切る。
    pub fn clear_span_from(&self, geom_len: f64) -> Option<f64> {
        let (fi, fj) = (self.face_i?, self.face_j?);
        Some((geom_len - fi - fj).max(0.0))
    }

    /// 柱フェース距離が両端とも算定済みか。
    pub fn faces_computed(&self) -> bool {
        self.face_i.is_some() && self.face_j.is_some()
    }

    /// i 端の剛体アーム長 [mm]（剛域長と仕口パネル分オフセットの大きい方）。
    ///
    /// 可撓長の控除・剛域変換・幾何剛性・せん断降伏の内法高さ・座屈長さの剛度比は、
    /// いずれもこの値を用いる。
    pub fn rigid_length_i(&self) -> f64 {
        self.length_i.max(self.panel_offset_i)
    }

    /// j 端の剛体アーム長 [mm]。意味は [`Self::rigid_length_i`] と同様。
    pub fn rigid_length_j(&self) -> f64 {
        self.length_j.max(self.panel_offset_j)
    }
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ElementData {
    pub id: ElemId,
    pub kind: ElementKind,
    pub nodes: SmallVec<[NodeId; 8]>,
    pub section: Option<SectionId>,
    pub local_axis: LocalAxis,
    pub end_cond: [EndCondition; 2],
    pub force_regime: ForceRegime,
    /// 部材端の剛域。旧スキーマ（無し）は既定値（剛域長 0）で補完される。
    #[serde(default)]
    pub rigid_zone: RigidZone,
    /// 塑性化領域長さ Lp [mm]（None = 塑性化域を考慮しない従来モデル）。
    /// ファイバー要素では端部 Lp 区間に非線形断面を配置し中央を弾性とする
    /// モデル化（材端剛塑性ばねと適合するファイバーモデル化）に用いる。
    #[serde(default)]
    pub plastic_zone: Option<f64>,
    /// 節点バネ要素（`ElementKind::NodalSpring`）の局所軸バネ定数
    /// `[kx, ky, kz, krx, kry, krz]`（軸[N/mm]・せん断[N/mm]・回転[N·mm/rad]）。
    /// 部材の変形と自由度の一般的な取り扱い（構造力学）では、節点バネは
    /// ねじり（θX）を非考慮とするのが既定だが、本実装では全 6 成分を入力可能とし、
    /// `krx` を明示的に 0 とすることで既定挙動に合わせる（入力で 0 以外も指定できる）。
    /// `None` は他要素種別、またはバネ定数未指定（剛性ゼロ扱い）。
    #[serde(default)]
    pub spring: Option<[f64; 6]>,
}
