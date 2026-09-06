//! M-N 相関曲面の基本データ型と材料定数。

/// 全塑性計算用のファイバ（またはバネ）。
#[derive(Clone, Debug)]
pub struct PlasticFiber {
    /// 断面内 y 座標 [mm]
    pub y: f64,
    /// 断面内 z 座標 [mm]
    pub z: f64,
    /// 負担断面積 [mm²]
    pub area: f64,
    /// 引張限界応力 [N/mm²]（≥0。コンクリートは 0）
    pub sigma_t: f64,
    /// 圧縮限界応力 [N/mm²]（≤0）
    pub sigma_c: f64,
    /// 弾性係数 [N/mm²]
    pub young: f64,
    /// 材料領域区分
    pub region: FiberRegion,
}

/// ファイバの材料領域区分。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FiberRegion {
    /// コンクリート（RC・SRC の躯体、CFT の充填部）
    Concrete,
    /// 主筋（点ファイバ）
    Rebar,
    /// 鋼材（形鋼・鋼管・内蔵鉄骨）
    Steel,
}

/// 降伏判定のモデル化手法。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum YieldModelKind {
    /// 部材端の単純降伏バネ（2バネ連成: |N|/N許容 + M/M許容 = 1 の線形相関）
    SimpleSpring,
    /// マルチスプリング（粗いバネ群、N-M 相関あり・多面体状）
    MultiSpring,
    /// マルチファイバー（細分割、滑らかな相関曲面）
    MultiFiber,
}

impl YieldModelKind {
    /// 表示用ラベル（日本語）。
    pub fn label(&self) -> &'static str {
        match self {
            YieldModelKind::SimpleSpring => "単純降伏バネ",
            YieldModelKind::MultiSpring => "マルチスプリング",
            YieldModelKind::MultiFiber => "マルチファイバー",
        }
    }
}

/// 材料強度パラメータ。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StrengthParams {
    /// 鋼材（形鋼）降伏強度 [N/mm²]
    pub steel_fy: f64,
    /// 鉄筋降伏強度 [N/mm²]
    pub rebar_fy: f64,
    /// コンクリート圧縮強度 [N/mm²]（正値で与える）
    pub concrete_fc: f64,
    /// 鋼材・鉄筋の弾性係数 [N/mm²]
    pub steel_e: f64,
}

impl Default for StrengthParams {
    fn default() -> Self {
        StrengthParams {
            steel_fy: 235.0,
            rebar_fy: 345.0,
            concrete_fc: 24.0,
            steel_e: 205000.0,
        }
    }
}

/// コンクリートの弾性係数 [N/mm²]（RC規準式。γ=23 固定）。
/// `fc<=0` では 0 を返すため、数値積分では呼出側で下限を保証すること。
pub fn concrete_young(fc: f64) -> f64 {
    squid_n_core::section_shape::concrete_young_modulus(fc.max(1.0))
}
