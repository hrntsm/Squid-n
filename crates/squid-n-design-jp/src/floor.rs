//! 床の中での小梁・スラブ（床）の設計。
//!
//! 小梁は大梁を分割せず、床の中で**単純支持梁**として検定する。
//!
//! 荷重は床領域分配が材軸へ出した辺荷重・小梁自身の自重・架け側の二次部材から
//! 渡された集中荷重の重ね合わせで、二次部材の反力の逐次伝達（`squid_n_load::cascade`）が
//! 求める。本モジュールは部材力を受け取って検定するだけである。
//!
//! 単位は N-mm（面荷重 N/mm²、線荷重 N/mm）。

/// 小梁（単純支持梁）の設計結果。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct JoistDesignResult {
    /// スパン（支持間距離）[mm]。
    pub span: f64,
    /// 代表等分布荷重 w [N/mm]（合計荷重 / スパン。表示用であり検定には使わない）。
    pub w: f64,
    /// 最大曲げモーメント [N·mm]。
    pub m_max: f64,
    /// 最大せん断力 [N]。
    pub q_max: f64,
    /// 曲げ応力度 σ = M/Z [N/mm²]。
    pub sigma: f64,
    /// 許容曲げ応力度 [N/mm²]。
    pub sigma_allow: f64,
    /// 曲げ検定比 σ/σ_allow。
    pub bending_ratio: f64,
    /// たわみ [mm]（等分布なら 5wL⁴/(384EI)。二次部材は数値積分）。
    pub deflection: f64,
    /// たわみ比 δ/L。
    pub deflection_span_ratio: f64,
    /// たわみ検定比 (δ/L)/(1/limit_denom)。
    pub deflection_ratio: f64,
    /// 総合検定比（曲げ・たわみの最大）。未検定は 0。
    pub ratio: f64,
    /// 判定（`ratio <= 1`）。未検定は `false`。
    pub ok: bool,
    /// 分配荷重が無い・断面/材料不足などで部材力を算定していない（表には残し、判定は「未」）。
    #[serde(default)]
    pub unchecked: bool,
}

/// 部材力（曲げ・せん断・たわみ）を直接与えて小梁を検定する。
///
/// 逐次伝達（`squid_n_load::cascade`）が重ね合わせた荷重から求めた部材力を渡す。
/// `m_max` / `q_max` / `deflection` が設計値。`w` は表示用の代表等分布（合計/スパン）。
///
/// `section_modulus`・`sigma_allow`・`defl_limit_denom` が 0 以下の場合は該当検定比を
/// 0 とする（断面情報が不足する場合の安全なフォールバック）。
#[allow(clippy::too_many_arguments)]
pub fn design_joist_from_forces(
    span: f64,
    w: f64,
    m_max: f64,
    q_max: f64,
    deflection: f64,
    section_modulus: f64,
    sigma_allow: f64,
    defl_limit_denom: f64,
) -> JoistDesignResult {
    judge_joist(
        span,
        w,
        m_max.abs(),
        q_max.abs(),
        deflection.abs(),
        section_modulus,
        sigma_allow,
        defl_limit_denom,
    )
}

/// 共通の検定判定（曲げ応力度・たわみ制限）。
#[allow(clippy::too_many_arguments)]
fn judge_joist(
    span: f64,
    w: f64,
    m_max: f64,
    q_max: f64,
    deflection: f64,
    section_modulus: f64,
    sigma_allow: f64,
    defl_limit_denom: f64,
) -> JoistDesignResult {
    let sigma = if section_modulus > 0.0 {
        m_max / section_modulus
    } else {
        0.0
    };
    let bending_ratio = if sigma_allow > 0.0 {
        sigma / sigma_allow
    } else {
        0.0
    };
    let deflection_span_ratio = if span > 0.0 { deflection / span } else { 0.0 };
    let deflection_ratio = if defl_limit_denom > 0.0 {
        deflection_span_ratio * defl_limit_denom
    } else {
        0.0
    };
    let ratio = bending_ratio.max(deflection_ratio);
    JoistDesignResult {
        span,
        w,
        m_max,
        q_max,
        sigma,
        sigma_allow,
        bending_ratio,
        deflection,
        deflection_span_ratio,
        deflection_ratio,
        ratio,
        ok: ratio <= 1.0,
        unchecked: false,
    }
}

/// 分配結果が無く断面検定できない小梁の表行（判定は「未」。OK と誤認しない）。
pub fn joist_unchecked(span: f64) -> JoistDesignResult {
    JoistDesignResult {
        span,
        w: 0.0,
        m_max: 0.0,
        q_max: 0.0,
        sigma: 0.0,
        sigma_allow: 0.0,
        bending_ratio: 0.0,
        deflection: 0.0,
        deflection_span_ratio: 0.0,
        deflection_ratio: 0.0,
        ratio: 0.0,
        ok: false,
        unchecked: true,
    }
}

/// スラブ（一方向）の設計結果。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SlabDesignResult {
    /// 設計スパン（短辺）[mm]。
    pub span: f64,
    /// 面荷重 w [N/mm²]。
    pub w: f64,
    /// 単位幅あたり設計曲げモーメント M = wL²/coef [N·mm/mm]。
    pub moment: f64,
    /// 板厚 t [mm]。
    pub thickness: f64,
    /// 有効せい d = t − かぶり [mm]。
    pub effective_depth: f64,
    /// 単位幅あたり必要引張鉄筋量 As [mm²/mm]。
    pub as_req_per_mm: f64,
    /// 1m あたり必要引張鉄筋量 As [mm²/m]（表示用）。
    pub as_req_per_m: f64,
}

/// スラブを一方向版として設計し、設計曲げモーメントと必要鉄筋量を算定する。
///
/// - `span`: 設計スパン（短辺）[mm]、`w`: 面荷重 [N/mm²]。
/// - `moment_coef`: 曲げモーメント係数（M = wL²/coef。単純支持=8、連続端=10 等）。
/// - `thickness`: 板厚 [mm]、`cover`: 圧縮縁から鉄筋重心までのかぶり [mm]。
/// - `ft_rebar`: 鉄筋の許容引張応力度 [N/mm²]（長期）、`j_ratio`: 応力中心距離比 j（≒7/8）。
///
/// 必要鉄筋量 As = M / (ft · j · d)。有効せい d が 0 以下なら As=0。
#[allow(clippy::too_many_arguments)]
pub fn design_slab_oneway(
    span: f64,
    w: f64,
    moment_coef: f64,
    thickness: f64,
    cover: f64,
    ft_rebar: f64,
    j_ratio: f64,
) -> SlabDesignResult {
    let coef = if moment_coef > 0.0 { moment_coef } else { 8.0 };
    let moment = w * span * span / coef; // N·mm per mm width
    let d = (thickness - cover).max(0.0);
    let as_req_per_mm = if ft_rebar > 0.0 && j_ratio > 0.0 && d > 0.0 {
        moment / (ft_rebar * j_ratio * d)
    } else {
        0.0
    };
    SlabDesignResult {
        span,
        w,
        moment,
        thickness,
        effective_depth: d,
        as_req_per_mm,
        as_req_per_m: as_req_per_mm * 1000.0,
    }
}

/// 鋼小梁の既定ヤング係数 [N/mm²]（材料未設定時）。
pub const STEEL_YOUNG: f64 = 205_000.0;
/// 鋼小梁の既定 F 値 [N/mm²]（材料未設定・`fy` 未設定時）。
pub const STEEL_F_DEFAULT: f64 = 235.0;
/// たわみ制限の既定分母（δ/L ≤ 1/250）。
pub const DEFLECTION_LIMIT_DENOM: f64 = 250.0;
/// スラブ設計の既定かぶり [mm]（圧縮縁〜鉄筋重心）。
pub const SLAB_DEFAULT_COVER: f64 = 30.0;
/// スラブ設計の既定 j 比（応力中心距離 / 有効せい）。
pub const SLAB_J_RATIO: f64 = 7.0 / 8.0;
/// 異形鉄筋 SD295 の長期許容引張応力度 [N/mm²]。
pub const REBAR_FT_LONG_SD295: f64 = 195.0;

/// 小梁略算に使うヤング係数 E [N/mm²] と長期許容曲げ応力度 [N/mm²]。
///
/// 本経路の曲げ検定は鋼の断面係数 Z と長期 `ft = F/1.5` を前提とする。
/// - 材料なし: 既定鋼（`STEEL_YOUNG`・`STEEL_F_DEFAULT`）
/// - 鋼材: 材料の `young` と `fy`（無ければ `STEEL_F_DEFAULT`）から `ft`
/// - コンクリート・鉄筋: `None`（RC 小梁の許容曲げはこの略算では扱わない → 表は「未」）
pub fn joist_steel_e_and_ft(mat: Option<&squid_n_core::model::Material>) -> Option<(f64, f64)> {
    use squid_n_core::model::MaterialCategory;
    match mat {
        None => Some((STEEL_YOUNG, STEEL_F_DEFAULT / 1.5)),
        Some(m) if m.category == MaterialCategory::Steel => {
            let e = if m.young > 0.0 { m.young } else { STEEL_YOUNG };
            let f = m.fy.filter(|v| *v > 0.0).unwrap_or(STEEL_F_DEFAULT);
            Some((e, f / 1.5))
        }
        Some(_) => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_joist_bending_and_deflection_ratio() {
        // 等分布 w=10 N/mm・L=4000mm 相当の部材力（M=wL²/8、Q=wL/2、δ=5wL⁴/(384EI)）。
        let defl = 5.0 * 10.0 * 4000.0_f64.powi(4) / (384.0 * STEEL_YOUNG * 1.0e8);
        let r = design_joist_from_forces(4000.0, 10.0, 2.0e7, 2.0e4, defl, 1.0e6, 156.0, 250.0);
        assert!((r.m_max - 2.0e7).abs() < 1.0, "M={}", r.m_max);
        assert!((r.q_max - 2.0e4).abs() < 1.0, "Q={}", r.q_max);
        // σ = M/Z = 2e7/1e6 = 20。
        assert!((r.sigma - 20.0).abs() < 1e-9);
        assert!((r.bending_ratio - 20.0 / 156.0).abs() < 1e-9);
        // δ/L に対する検定比は (δ/L)/(1/250)。
        assert!((r.deflection_span_ratio - defl / 4000.0).abs() < 1e-15);
        assert!((r.deflection_ratio - defl / 4000.0 * 250.0).abs() < 1e-9);
    }

    #[test]
    fn test_joist_zero_section_is_safe() {
        // 断面情報ゼロでもパニックせず、検定比 0。
        let r = design_joist_from_forces(4000.0, 10.0, 2.0e7, 2.0e4, 10.0, 0.0, 0.0, 0.0);
        assert_eq!(r.bending_ratio, 0.0);
        assert_eq!(r.deflection_ratio, 0.0);
        assert!(r.ok);
    }

    #[test]
    fn test_slab_oneway_moment_and_rebar() {
        // w=0.005 N/mm², L=3000mm, 単純支持(coef=8) → M=wL²/8=5625 N·mm/mm。
        let r = design_slab_oneway(
            3000.0,
            0.005,
            8.0,
            150.0,
            SLAB_DEFAULT_COVER,
            REBAR_FT_LONG_SD295,
            SLAB_J_RATIO,
        );
        assert!((r.moment - 0.005 * 3000.0 * 3000.0 / 8.0).abs() < 1e-6);
        assert!((r.effective_depth - 120.0).abs() < 1e-9);
        // As = M/(ft·j·d)。
        let expect_as = r.moment / (REBAR_FT_LONG_SD295 * SLAB_J_RATIO * 120.0);
        assert!((r.as_req_per_mm - expect_as).abs() / expect_as < 1e-9);
        assert!((r.as_req_per_m - expect_as * 1000.0).abs() / (expect_as * 1000.0) < 1e-9);
    }
}
