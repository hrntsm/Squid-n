//! 応力解析の計算条件の型。
//!
//! - [`StressAnalysisCfg`] — 長期応力解析の計算条件（令82条）。

/// 長期応力解析の計算条件（令82条の応力解析）。
///
/// 計算条件の指定により、一部の部材（ブレース・柱・制振間柱）について
/// 長期軸力を負担させないことが可能である。
///
/// 制振間柱（damper-equipped mullion column）は本リポジトリに要素種別が未実装のため、
/// 対象外（既知の制約）。ブレースと柱（鉛直部材）のみ対応する。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StressAnalysisCfg {
    /// 長期応力解析でブレース（`ElementKind::Brace`）に軸力を負担させない。
    pub no_long_axial_brace: bool,
    /// 長期応力解析で柱（鉛直な `ElementKind::Beam`）に軸力を負担させない。
    pub no_long_axial_column: bool,
    /// 剛性率・偏心率算定時の雑壁剛性の n 倍法係数（雑壁の剛性評価。RC 規準。
    /// `Kw' = n·Aw'·ΣKc/ΣAc` の n。入力値）。`None` は雑壁剛性を考慮しない。
    #[serde(default)]
    pub misc_wall_n: Option<f64>,
    /// 層間変形角の制限値の分母（令82条の2）。原則 200（1/200）。帳壁・仕上げ等に
    /// 著しい損傷の恐れがない場合は 120（1/120）へ緩和できる。
    #[serde(default = "default_drift_limit_denom")]
    pub drift_limit_denom: f64,
    /// 線形（弾性）応力解析で引張専用ブレースを active-set 反復で扱うか。
    ///
    /// `false`（既定）: 引張専用ブレースは「引張・圧縮が対で存在する」とみなし、
    /// 弾性剛性を 1/2 にした一括解析で1回だけ解く（従来挙動）。
    /// `true`: 圧縮側に入ったブレースを無効化しながら収束するまで再解析する
    /// 真の引張専用解析を行う（active な引張ブレースは全剛性 EA/L を負担し、
    /// 圧縮ブレースは軸力を負担しない）。
    #[serde(default)]
    pub tension_only_iteration: bool,
    /// 剛域の算定で、部材フェース・部材せいに取り付く壁を考慮するか（技術基準
    /// 「剛域の計算」。既定は考慮する）。
    ///
    /// `true`（既定）: 柱には袖壁、梁には腰壁・垂壁を含めた寸法で算定する。
    /// 対象は現場打ちコンクリート壁で厚さ 100 mm 以上のもの（耐震壁・雑壁を問わない）。
    /// `false`: 部材の原断面だけで算定する。
    #[serde(default = "default_rigid_zone_consider_walls")]
    pub rigid_zone_consider_walls: bool,
}

fn default_rigid_zone_consider_walls() -> bool {
    true
}

fn default_drift_limit_denom() -> f64 {
    200.0
}

impl Default for StressAnalysisCfg {
    fn default() -> Self {
        StressAnalysisCfg {
            no_long_axial_brace: false,
            no_long_axial_column: false,
            misc_wall_n: None,
            drift_limit_denom: default_drift_limit_denom(),
            tension_only_iteration: false,
            rigid_zone_consider_walls: default_rigid_zone_consider_walls(),
        }
    }
}
