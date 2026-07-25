//! RC 造耐震壁の終局せん断強度 Qu と開口低減率（非線形解析・保有水平耐力用）。
//!
//! # 位置付け
//! `squid-n-element`（Layer 3）・`squid-n-solver`（Layer 4）は
//! `squid-n-design-jp`（Layer 5）に依存できない（循環依存になる）ため、耐震壁の
//! せん断終局強度の本体を Layer 0 の本モジュールへ置き、
//! `squid_n_design_jp::rc::wall_nonlinear` は本モジュールへ委譲する
//! （[`crate::rc_capacity`]（梁・柱）と同じ構成）。
//!
//! これにより、プッシュオーバーで**耐震壁をせん断終局強度で頭打ちにする**
//! （壁エレメントの面内せん断を弾完全塑性とする）ことができる。従来は壁要素が
//! 線形弾性のままで、押し込むほど際限なく水平力を負担し、崩壊機構が形成されず
//! 保有水平耐力 Qu を過大評価していた（危険側）。
//!
//! # 準拠する規準・出典
//! - 終局せん断強度 Qu（荒川mean式系・耐震壁）: 2007年版建築物の構造関係技術基準
//!   解説書 P.281-282, 638-639／日本建築学会「鉄筋コンクリート終局強度設計に関する
//!   資料」P.132。
//! - 開口低減率 r（**耐力用**）: 同 資料 P.132、平19国交告第594号第1。

/// 耐震壁の終局せん断強度算定の入力（SI 系: 長さ [mm]・面積 [mm²]・応力 [N/mm²]）。
#[derive(Clone, Copy, Debug)]
pub struct RcWallShearInput {
    /// コンクリート設計基準強度 Fc [N/mm²]。
    pub fc: f64,
    /// 等価壁厚 te [mm]（I 形断面を等価長方形に置換した幅。壁厚 t の 1.5 倍以下）。
    pub te: f64,
    /// 壁厚 t [mm]（pwh = Pwh·t/te の換算に用いる）。
    pub t: f64,
    /// 付帯柱を含めた耐震壁の全長 D [mm]。
    pub d_wall: f64,
    /// 圧縮側柱のせい Dc [mm]（有効せい d = D − Dc/2）。付帯柱が無い場合は 0。
    pub dc_compression: f64,
    /// 引張側柱の主筋断面積 at [mm²]（pte = 100·at/(te·d) の分子）。
    pub tension_column_at: f64,
    /// 水平せん断補強筋（横筋）の材料強度 σwh [N/mm²]。
    pub sigma_wh: f64,
    /// 横筋比 Pwh（小数。pwh = Pwh·t/te、1.2% 上限）。
    pub pwh_ratio: f64,
    /// 全断面積に対する平均軸方向応力度 σ0 = N/A [N/mm²]（圧縮正）。
    pub sigma_0: f64,
    /// せん断スパン比 M/(Q·D)（適用範囲 1.0〜3.0 にクランプ）。
    pub shear_span_ratio: f64,
    /// 高強度せん断補強筋を用いる場合 true（Qu 係数 0.053→0.068）。
    pub high_strength_shear_rebar: bool,
    /// 開口（`(l0, h0, h, lw)`。l0・h0: 開口幅・高さ、h: 壁の上下梁中心間高さ、
    /// lw: 付帯柱中心間距離）。`None` は無開口（r=1）。
    pub opening: Option<(f64, f64, f64, f64)>,
}

/// 耐震壁の**耐力**用開口低減率 r2（無次元）。
///
/// ```text
/// r2 = 1 − max(r0, l0/lw, h0/h),   r0 = √(h0·l0/(h·lw))
/// ```
///
/// 平19国交告第594号第1 の「耐力壁のせん断耐力の低減率」。**剛性**の低減率
/// `r1 = 1 − 1.25·r0`（`squid_n_element::factory::wall_opening_reduction`）とは
/// **別式**であり、取り違えてはならない（耐力側に r1 を使うと開口の影響を
/// 過小評価する＝危険側）。
///
/// 無開口（`opening == None`）は 1.0。極端な開口では 0 にクランプする。
pub fn wall_opening_reduction_strength(opening: Option<(f64, f64, f64, f64)>) -> f64 {
    match opening {
        Some((l0, h0, h, lw)) if h > 0.0 && lw > 0.0 => {
            let r0 = (h0 * l0 / (h * lw)).max(0.0).sqrt();
            let reduce = r0.max(l0 / lw).max(h0 / h);
            (1.0 - reduce).clamp(0.0, 1.0)
        }
        _ => 1.0,
    }
}

/// 開口周比 r0 = √(h0·l0/(h·l))。`h`・`l` が 0 以下なら 0。
pub fn wall_opening_ratio_r0(h0: f64, l0: f64, h: f64, l: f64) -> f64 {
    if h <= 0.0 || l <= 0.0 {
        return 0.0;
    }
    ((h0 * l0) / (h * l)).max(0.0).sqrt()
}

/// 耐震壁の**剛性**用開口低減率 r1 = 1 − 1.25·r0（平19国交告第594号第1）。
///
/// **耐力**用の r2 = 1−max(r0, l0/lw, h0/h)（[`wall_opening_reduction_strength`]）
/// とは別式。剛性側に r2 を、耐力側に r1 を使う取り違えをしないこと。
/// 負になる場合は 0 にクランプする。
pub fn wall_opening_reduction_stiffness(r0: f64) -> f64 {
    (1.0 - 1.25 * r0.max(0.0)).max(0.0)
}

/// 耐震壁の終局せん断強度 Qu [N]（荒川mean式系。技術基準解説書 P.638-639）。
///
/// ```text
/// Qu = { k·pte^0.23·(Fc+18)/denom + 0.85·√(σwh·pwh) + 0.1·σ0 }·te·j·r
/// ```
/// - `k = 0.053`（既定。denom = M/(Q·D)+0.12）／`0.068`（高強度せん断補強筋。
///   denom = √(M/(Q·D)+0.12)）
/// - `pte = 100·at/(te·d)` \[%\]（等価引張鉄筋比）、`d = D − Dc/2`、`j = 7/8·d`
/// - `M/(Q·D)` は適用範囲 1.0〜3.0 にクランプ、`pwh` は 1.2% 上限、
///   `σ0` は 0〜0.4Fc にクランプ（引張は 0）
/// - 開口低減 r は**耐力用** [`wall_opening_reduction_strength`] を乗じる
///
/// 不正入力（Fc・te・D・at のいずれかが 0 以下、または d ≤ 0）は 0.0 を返す。
pub fn wall_shear_ultimate(inp: &RcWallShearInput) -> f64 {
    let d = inp.d_wall - inp.dc_compression / 2.0;
    if inp.fc <= 0.0
        || inp.te <= 0.0
        || inp.d_wall <= 0.0
        || inp.tension_column_at <= 0.0
        || d <= 0.0
    {
        return 0.0;
    }
    let pte = 100.0 * inp.tension_column_at / (inp.te * d);
    let j = 7.0 / 8.0 * d;
    let shear_span_ratio = inp.shear_span_ratio.clamp(1.0, 3.0);
    let k = if inp.high_strength_shear_rebar {
        0.068
    } else {
        0.053
    };
    let pwh = if inp.te > 0.0 {
        (inp.pwh_ratio.max(0.0) * inp.t / inp.te).min(0.012)
    } else {
        0.0
    };
    let denom = if inp.high_strength_shear_rebar {
        (shear_span_ratio + 0.12).sqrt()
    } else {
        shear_span_ratio + 0.12
    };
    let concrete_term = k * pte.powf(0.23) * (inp.fc + 18.0) / denom;
    let hoop_term = 0.85 * (pwh * inp.sigma_wh).max(0.0).sqrt();
    let sigma_0 = inp.sigma_0.clamp(0.0, 0.4 * inp.fc);
    let axial_term = 0.1 * sigma_0;
    let r = wall_opening_reduction_strength(inp.opening);
    (concrete_term + hoop_term + axial_term) * inp.te * j * r
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> RcWallShearInput {
        RcWallShearInput {
            fc: 24.0,
            te: 200.0,
            t: 200.0,
            d_wall: 4000.0,
            dc_compression: 0.0,
            tension_column_at: 1000.0,
            sigma_wh: 295.0,
            pwh_ratio: 0.0025,
            sigma_0: 0.0,
            shear_span_ratio: 1.0,
            high_strength_shear_rebar: false,
            opening: None,
        }
    }

    /// 開口低減（耐力用）は r2 = 1 − max(r0, l0/lw, h0/h)。
    /// 剛性用の r1 = 1 − 1.25·r0 とは別式であることを固定する。
    #[test]
    fn test_opening_reduction_strength_uses_max_rule() {
        // l0=1000, h0=1000, h=3000, lw=4000
        // r0 = √(1000·1000/(3000·4000)) = √0.08333 = 0.288675
        // l0/lw = 0.25、h0/h = 0.33333 → max = 0.33333
        let r = wall_opening_reduction_strength(Some((1000.0, 1000.0, 3000.0, 4000.0)));
        assert!((r - (1.0 - 1.0 / 3.0)).abs() < 1e-9, "r={}", r);
        // 剛性用 r1 = 1 − 1.25·0.288675 = 0.6392 とは一致しない。
        let r1 = 1.0 - 1.25 * (1.0f64 / 12.0).sqrt();
        assert!((r - r1).abs() > 1e-3, "耐力用 r2 と剛性用 r1 は別式");
    }

    #[test]
    fn test_opening_reduction_none_is_one() {
        assert!((wall_opening_reduction_strength(None) - 1.0).abs() < 1e-12);
    }

    /// 開口低減は Qu に線形に乗る。
    #[test]
    fn test_qu_scales_with_opening_reduction() {
        let mut inp = sample();
        let qu0 = wall_shear_ultimate(&inp);
        inp.opening = Some((1000.0, 1000.0, 3000.0, 4000.0));
        let r = wall_opening_reduction_strength(inp.opening);
        let qu1 = wall_shear_ultimate(&inp);
        assert!(
            (qu1 - qu0 * r).abs() < 1e-6,
            "qu1={} qu0*r={}",
            qu1,
            qu0 * r
        );
    }

    /// 軸圧縮は Qu を増やし、0.4Fc で頭打ちになる。
    #[test]
    fn test_qu_axial_term_clamped() {
        let mut inp = sample();
        let q0 = wall_shear_ultimate(&inp);
        inp.sigma_0 = 5.0;
        let q1 = wall_shear_ultimate(&inp);
        assert!(q1 > q0);
        inp.sigma_0 = 100.0; // 0.4Fc = 9.6 でクランプ
        let q2 = wall_shear_ultimate(&inp);
        let mut inp_c = sample();
        inp_c.sigma_0 = 0.4 * 24.0;
        assert!((q2 - wall_shear_ultimate(&inp_c)).abs() < 1e-9);
    }

    #[test]
    fn test_qu_invalid_inputs_are_zero() {
        let mut inp = sample();
        inp.tension_column_at = 0.0;
        assert_eq!(wall_shear_ultimate(&inp), 0.0);
        let mut inp = sample();
        inp.fc = 0.0;
        assert_eq!(wall_shear_ultimate(&inp), 0.0);
    }

    /// 代表値の手計算照合（無開口・軸力なし・M/(QD)=1.0）。
    #[test]
    fn test_qu_matches_handcalc() {
        let inp = sample();
        let d: f64 = 4000.0;
        let pte: f64 = 100.0 * 1000.0 / (200.0 * d);
        let j = 7.0 / 8.0 * d;
        let concrete = 0.053 * pte.powf(0.23) * (24.0 + 18.0) / (1.0 + 0.12);
        let hoop = 0.85 * (0.0025f64 * 295.0).sqrt();
        let expect = (concrete + hoop) * 200.0 * j;
        assert!(
            (wall_shear_ultimate(&inp) - expect).abs() < 1e-6,
            "{} vs {}",
            wall_shear_ultimate(&inp),
            expect
        );
    }
}
