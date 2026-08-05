//! 階（層）関連の型。
//!
//! - [`DiaphragmDef`] — 剛床（マスター・スレーブ節点、重量分配）。
//! - [`StoryStructure`] — 階の主要構造種別。
//! - [`StoryLevelKind`] — 階の種別（一般／PH／地下）。
//! - [`Story`] — 階の定義。

use super::*;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DiaphragmDef {
    pub master: NodeId,
    pub slaves: Vec<NodeId>,
    pub rigid: bool,
    /// この剛床が負担する地震用重量 [N]。多剛床の階では層の水平力 Pi を
    /// 剛床ごとの重量比で分配するために用いる（多剛床の設計用せん断力。
    /// 令88条・昭55建告1793号）。None は未算定（階に単一剛床なら層重量全量）。
    #[serde(default)]
    pub weight: Option<f64>,
    /// 副剛床の層せん断力係数 Ci の直接入力（令88条・昭55建告1793号の
    /// 層せん断力係数）。Some の剛床は主系統の Ai 分布から
    /// 除外され、水平力 = ci_override × 剛床重量（等価震度扱い。上階に同一系統の
    /// 剛床が積み上がらない副剛床を想定）として作用する。None は主系統（Ai 分布）。
    #[serde(default)]
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

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Story {
    pub id: StoryId,
    pub name: String,
    pub elevation: f64,
    pub node_ids: Vec<NodeId>,
    pub diaphragms: Vec<DiaphragmDef>,
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
