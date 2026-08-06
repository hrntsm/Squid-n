//! 地震・風の静的解析の設定型。

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SeismicDir {
    X,
    Y,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum AiMode {
    Approx,
    SemiPrecise,
}

/// 地震静的解析(Ai分布)の設定。
#[derive(Debug, Clone, Copy)]
pub struct SeismicCfg {
    pub dir: SeismicDir,
    pub mode: AiMode,
    /// 地域係数 Z（令88条）。
    pub z: f64,
    /// 地盤種別（Tc の決定に使用）。
    pub soil: squid_n_load::ai::SoilClass,
    /// 標準せん断力係数 C0（一次設計 0.2、保有 1.0）。
    pub c0: f64,
}

impl Default for SeismicCfg {
    /// 既定の Ai 算定法は略算（`AiMode::Approx`）。略算周期 T = h(0.02+0.01α)
    /// （令88条・昭55建告1793号、h: 建築物の高さ、α: 鉄骨造比）で求め、
    /// 固有値解析を要しない。精算周期（`SemiPrecise`）は固有値解析の明示実行を
    /// 前提とするオプトイン設定。
    fn default() -> Self {
        Self {
            dir: SeismicDir::X,
            // 既定は略算 T（告示式）。固有値 T は呼び出し側が明示的に指定する。
            mode: AiMode::Approx,
            z: 1.0,
            soil: squid_n_load::ai::SoilClass::II,
            c0: 0.2,
        }
    }
}
