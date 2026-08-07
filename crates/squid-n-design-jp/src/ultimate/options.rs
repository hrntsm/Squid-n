//! 終局検定の入力・算定オプション。
//!
//! - [`MemberDemand`] — 部材の設計用需要（軸力・二軸曲げ・せん断・Rp 等）。
//! - [`MuMethod`] — 柱の曲げ終局強度 Mu の算定方法（at 式 / ACI）。
//! - [`ShearMethod`] — 終局せん断強度の算定方法（塑性理論式 / 靭性指針式）。
//! - [`UltimateShearOptions`] — 終局検定（塑性理論式）の算定オプション。

/// 部材の設計用需要（終局検定の入力）。**圧縮を正**とする軸力と、強軸・弱軸まわりの
/// 設計用曲げモーメント（応答値。二軸曲げ余裕度に用いる）。
///
/// `shear`・`rp` は**プッシュオーバー応答からの直接反映**（[`MemberDemand::from_pushover`]）
/// に用いる任意項目で、`None`（既定）のときは従来どおり両端ヒンジ `Qmu=2·Mu/内法` と
/// UI 一律指定 Rp（[`UltimateShearOptions::rp`]）を用いる。
#[derive(Clone, Copy, Debug, Default)]
pub struct MemberDemand {
    /// 設計軸力 [N]（**圧縮正**）。柱の Mu・軸余裕度に用いる。
    pub n_axial: f64,
    /// 強軸（せい方向）まわりの設計用曲げモーメント Mmx [N·mm]（符号は内部で abs）。
    pub mz: f64,
    /// 弱軸（幅方向）まわりの設計用曲げモーメント Mmy [N·mm]（符号は内部で abs）。
    pub my: f64,
    /// 設計用せん断力 Qm [N]（プッシュオーバー応答の強軸せん断。`Some` のとき
    /// `Qmu = 上限強度倍率·|Qm|` として両端ヒンジ略算（2·Mu/内法）を置き換える）。
    pub shear: Option<f64>,
    /// 弱軸の設計用せん断力 Qmy [N]（プッシュオーバー応答の弱軸せん断。`Some` のとき
    /// 2 軸せん断余裕度の弱軸需要 `Qmuy = 上限強度倍率·|Qmy|`（両端ヒンジ略算 2·Muy/内法
    /// を置き換える）に用いる。2 軸せん断を検定しない場合は無視される）。
    pub shear_weak: Option<f64>,
    /// 部材別のヒンジ回転角 Rp [rad]（プッシュオーバー終局時の部材変形角）。`Some`
    /// のとき ν・cotφ・μ・tanθ に用いる Rp を [`UltimateShearOptions::rp`] から置き換える。
    pub rp: Option<f64>,
    /// 長期せん断力 QL [N]（絶対値で扱う）。`Some` のとき梁のせん断・付着余裕率の
    /// 分子を `(Qsu − QL)`・`(Qbu − QL)` とする（余裕率
    /// `(Qsu−QL)/Qmu ≥ 1.0` の定義。`None` は従来どおり QL=0 扱い）。
    pub q_long: Option<f64>,
    /// 長期荷重による単純梁せん断力 Q0 [N]（絶対値で扱う）。せん断補強筋に
    /// MK785/SPR785/SPR685 を使用した部材では、余裕率の QL 控除を `QL=Q0` と
    /// 読み替える（各製品の技術評定の規定）。`None` のときは `q_long` を用いる。
    pub q_simple: Option<f64>,
}

impl MemberDemand {
    /// 軸力のみ（曲げ・せん断需要 0、Rp は UI 一律指定に従う）の需要を作る。
    pub fn axial(n_axial: f64) -> Self {
        Self {
            n_axial,
            mz: 0.0,
            my: 0.0,
            shear: None,
            shear_weak: None,
            rp: None,
            q_long: None,
            q_simple: None,
        }
    }

    /// プッシュオーバー応答から部材需要を作る。軸力（圧縮正）・強軸/弱軸の設計用曲げ・
    /// 強軸/弱軸の設計用せん断・部材別 Rp を直接反映する（`Qmu`・弱軸 Qmuy・Rp を
    /// 応答値で置き換える）。
    #[allow(clippy::too_many_arguments)]
    pub fn from_pushover(
        n_axial: f64,
        mz: f64,
        my: f64,
        shear: f64,
        shear_weak: f64,
        rp: f64,
    ) -> Self {
        Self {
            n_axial,
            mz,
            my,
            shear: Some(shear),
            shear_weak: Some(shear_weak),
            rp: Some(rp),
            q_long: None,
            q_simple: None,
        }
    }
}

/// 柱の曲げ終局強度 Mu の算定方法（技術基準解説書 at 式 / ACI318）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MuMethod {
    /// 構造規定式（at 式、軸力考慮の閉形式略算）。
    #[default]
    AtFormula,
    /// ACI 規準による平面保持解析（等価応力度ブロック法）。
    Aci,
}

/// 終局せん断強度の算定方法（塑性理論式／靭性指針式の選択に対応）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ShearMethod {
    /// 塑性理論式（藤井・森田式系、「終局強度型設計指針」）。既定。
    #[default]
    Plastic,
    /// 靭性指針式 Vu=min(Vu1,Vu2,Vu3)（AIJ「靭性保証型耐震設計指針」6.4）。
    Ductility,
}

/// 終局検定（塑性理論式）の算定オプション。
#[derive(Clone, Debug)]
pub struct UltimateShearOptions {
    /// 終局限界状態でのヒンジ領域の回転角 Rp [rad]（ν・cotφ に用いる。既定 0）。
    pub rp: f64,
    /// 軽量コンクリートを使用する場合 true（せん断終局耐力を 0.9 倍に低減）。
    pub lightweight: bool,
    /// 上限強度倍率（Qmu = 上限強度倍率·(Mu上+Mu下)/内法。既定 1.0）。
    pub upper_strength_factor: f64,
    /// せん断補強筋の降伏強度算定用強度 σwy [N/mm²]（断面に材料が割り当てられて
    /// いない場合の代表値。既定 295 = SD295 相当）。
    pub sigma_wy: f64,
    /// せん断補強筋の材質名。高強度せん断補強筋（`MK785`・`SPR785` 等）の製品別
    /// 規定を適用するかの判定に用いる。`None` は未割当で、普通強度として扱う。
    ///
    /// **利用者が指定する欄ではない。** 部材ごとに断面のせん断補強筋材料
    /// （`Section::shear_rebar_material`）の名前が入る（`rc_check` が載せ替える）。
    pub shear_grade: Option<String>,
    /// 付着割裂の検定を含める場合 true。
    pub include_bond: bool,
    /// 柱の曲げ終局強度 Mu の算定方法（既定 at 式）。
    pub mu_method: MuMethod,
    /// 終局せん断強度の算定方法（既定 塑性理論式）。靭性指針式を選ぶと Qsu 列に
    /// Vu=min(Vu1,Vu2,Vu3)（[`rc_shear_ductility`]）を用いる。
    pub shear_method: ShearMethod,
    /// 柱のせん断を 2 軸せん断として検定する場合 true（採用応力：RC のみ
    /// 指定により 2 軸せん断）。両軸の Qmu/Qsu を相互作用式
    /// `1/((Qmx/Qux)^αx+(Qmy/Quy)^αy)^(1/α)`（RC は α=2.0）で合成する。
    pub biaxial_shear: bool,
    /// 柱の曲げを 2 軸曲げとして検定する場合 true（採用応力）。
    /// 両軸の設計用曲げ Mm と終局曲げ強度 Mu を相互作用式
    /// `1/((Mmx/Mux)^αx+(Mmy/Muy)^αy)^(1/α)`（RC は α=2.0）で合成する。
    /// 設計用曲げ需要 Mmx/Mmy は [`MemberDemand`] の mz/my を用いる。
    pub biaxial_bending: bool,
}

impl Default for UltimateShearOptions {
    fn default() -> Self {
        Self {
            rp: 0.0,
            lightweight: false,
            upper_strength_factor: 1.0,
            sigma_wy: 295.0,
            shear_grade: None,
            include_bond: true,
            mu_method: MuMethod::default(),
            shear_method: ShearMethod::default(),
            biaxial_shear: false,
            biaxial_bending: false,
        }
    }
}
