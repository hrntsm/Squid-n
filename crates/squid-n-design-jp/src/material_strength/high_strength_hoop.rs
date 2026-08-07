//! 高強度せん断補強筋（各製品の大臣認定・技術評定値による許容せん断応力度・pw 上限）。
//!
//! `ShearBar.grade` に製品名/規格名（例 "UB785", "KH785", "SBPD1275" 等）が
//! 設定されている場合、通常鋼材（SD295〜SD490）の許容せん断応力度表とは別の
//! 高強度品用テーブルを用いる。
//!
//! # 簡略化・注意事項
//! - 高強度せん断補強筋の精算式は製品ごとに異なる（例: ウルボン1275 の √ を含む式、
//!   KH785 系の βc を用いる式など）が、本実装では未実装。「上記以外の高強度
//!   せん断補強筋の場合」に相当する一般化した暫定対応式
//!   （[`crate::rc::shear_capacity_high_strength`]）を全高強度製品に一律適用する。
//! - pw の上限値は各製品の大臣認定・技術評定値に基づく製品グループごとの定数表とし、
//!   グループ判別ができない（未知の高強度品名の）場合は安全側の 0.8% を用いる。

use crate::LoadTerm;

/// 高強度せん断補強筋の製品グループ（pw 上限値の判定用）。
///
/// 製品別 pw 上限表（短期。各製品の技術評定値。2026-07-11 原典図で照合済み）:
/// - ウルボン系（ウルボン785=UB785, ウルボン1275=SBPD1275）・SPR785:
///   1.2%（損傷制御）/1.0%（安全確保）、Fc 非依存。
/// - リバーボン785(KW785)・スミフープ等(KSS785)・HDC685: 0.8%、Fc 非依存。
/// - スーパーフープ KH785: `min(1.2%, 1.0%・Fc/27)`。
/// - スーパーフープ KH685・パワーリング SPR685: `min(1.2%, 1.2%・Fc/27)`。
/// - UHYフープ SHD685・エムケーフープ MK785: 1.2%（損傷制御・安全確保とも）、Fc 非依存。
/// - 上記以外（判別不能な高強度品）: 安全側に 0.8%。
///
/// 長期は全製品 0.6% で共通（[`high_strength_pw_cap`] 側で分岐）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HighStrengthGroup {
    /// ウルボン系（ウルボン785=UB785, ウルボン1275=SBPD1275）・SPR785。
    /// 短期上限 1.2%（損傷制御）/1.0%（安全確保）、Fc 非依存。
    UlbonSeries,
    /// リバーボン785(KW785)・スミフープ等(KSS785)・HDC685。
    /// 短期上限 0.8%（損傷制御・安全確保とも）、Fc 非依存。
    Kw785Series,
    /// スーパーフープ KH785。短期上限 `min(1.2%, 1.0%・Fc/27)`。
    Kh785,
    /// スーパーフープ KH685・パワーリング SPR685。
    /// 短期上限 `min(1.2%, 1.2%・Fc/27)`。
    Kh685Series,
    /// UHYフープ SHD685・エムケーフープ MK785。短期上限 1.2%（損傷制御・
    /// 安全確保とも）、Fc 非依存。
    Shd685OrMk785,
    /// 上記以外（判別不能な高強度品）。安全側に短期上限 0.8% とする。
    Other,
}

/// grade 文字列（大文字化・前方一致）から高強度せん断補強筋の製品グループ
/// を判定する。
pub fn high_strength_group(grade: &str) -> HighStrengthGroup {
    let g = grade.trim().to_uppercase();
    let matches_any = |candidates: &[&str]| {
        candidates
            .iter()
            .any(|c| g.starts_with(c.to_uppercase().as_str()))
    };

    if matches_any(&[
        "UB785",
        "SBPD1275",
        "ｳﾙﾎﾞﾝ785",
        "ｳﾙﾎﾞﾝ1275",
        "ウルボン785",
        "ウルボン1275",
        "SPR785",
    ]) {
        HighStrengthGroup::UlbonSeries
    } else if matches_any(&["KW785", "KSS785", "HDC685"]) {
        HighStrengthGroup::Kw785Series
    } else if matches_any(&["KH785"]) {
        HighStrengthGroup::Kh785
    } else if matches_any(&["KH685", "SPR685"]) {
        HighStrengthGroup::Kh685Series
    } else if matches_any(&["SHD685", "MK785"]) {
        HighStrengthGroup::Shd685OrMk785
    } else {
        HighStrengthGroup::Other
    }
}

/// 高強度せん断補強筋かどうかの判定は
/// [`squid_n_core::material_grade::is_high_strength_shear_grade`] に一本化している
/// （耐震壁の終局せん断強度を算定する `squid-n-element` からも同じ判定を使うため）。
pub use squid_n_core::material_grade::is_high_strength_shear_grade;

/// 高強度せん断補強筋の許容せん断応力度 w_ft [N/mm²]（製品表）。
///
/// 長期は全製品 195。短期は SBPD1275（ウルボン1275）のみ 585、他は全て 590
/// （未知の高強度品名を含む「その他」も 590 とする）。
pub fn high_strength_w_ft(grade: &str, long_term: bool) -> f64 {
    if long_term {
        return 195.0;
    }
    let g = grade.trim().to_uppercase();
    let is_sbpd1275 = g.starts_with("SBPD1275")
        || g.starts_with("ｳﾙﾎﾞﾝ1275".to_uppercase().as_str())
        || g.starts_with("ウルボン1275");
    if is_sbpd1275 {
        585.0
    } else {
        590.0
    }
}

/// 高強度せん断補強筋使用時の pw 上限値（製品グループ・長短期・
/// 損傷制御/安全確保・Fc に応じた表）。
///
/// 長期は全製品 0.6%（Fc 非依存）。`fc` は Fc(raw) [N/mm²]。スーパーフープ
/// KH785/KH685・パワーリング SPR685 は短期上限が Fc に依存する
/// （[`HighStrengthGroup`] の doc 参照）。
pub fn high_strength_pw_cap(grade: &str, term: LoadTerm, damage_control: bool, fc: f64) -> f64 {
    if term == LoadTerm::Long {
        return 0.006;
    }
    match high_strength_group(grade) {
        HighStrengthGroup::UlbonSeries => {
            if damage_control {
                0.012
            } else {
                0.010
            }
        }
        HighStrengthGroup::Kw785Series => 0.008,
        // スーパーフープ KH785: min(1.2%, 1.0%・Fc/27)。
        HighStrengthGroup::Kh785 => (0.012_f64).min(0.010 * fc / 27.0),
        // スーパーフープ KH685・パワーリング SPR685: min(1.2%, 1.2%・Fc/27)。
        HighStrengthGroup::Kh685Series => (0.012_f64).min(0.012 * fc / 27.0),
        // UHYフープ SHD685・エムケーフープ MK785: Fc に依存せず一律 1.2%。
        HighStrengthGroup::Shd685OrMk785 => 0.012,
        HighStrengthGroup::Other => 0.008,
    }
}

// ============================================================================
// 3b. 高強度せん断補強筋の終局検定用 σwy・ν0・pw 上限
//     （「06 終局検定」の製品別表。各製品の技術評定値）
// ============================================================================

/// 終局検定用の高強度せん断補強筋の製品判別（大文字化・前方一致）。
/// 1275 級（ウルボン1275=SBPD1275/1420・リバーボン1275=SBPDN1275/1420）なら true。
fn is_ultimate_hoop_1275_class(grade: &str) -> bool {
    let g = grade.trim().to_uppercase();
    [
        "SBPD1275",
        "SBPDN1275",
        "ウルボン1275",
        "リバーボン1275",
        "ｳﾙﾎﾞﾝ1275",
        "ﾘﾊﾞｰﾎﾞﾝ1275",
    ]
    .iter()
    .any(|c| g.starts_with(c.to_uppercase().as_str()))
}

/// 高強度せん断補強筋の**終局検定用** σwy（せん断補強筋の降伏強度算定用強度）
/// [N/mm²]。製品別の技術評定値の表による。`fc` は Fc(raw) [N/mm²]。
///
/// | 製品 | σwy |
/// |---|---|
/// | ウルボン1275(SBPD1275/1420)・リバーボン1275(SBPDN1275/1420) | min(25·Fc, 1275) |
/// | ウルボン785(UB785)・リバーボン785(KW785)・スミフープ/ストロングフープ/デーフープ(KSS785) | min(25·Fc, 785) |
/// | UHYフープ(SHD685)・パワーリング685(SPR685) | min(25·Fc, 685) |
/// | エヌエスハイデック685H(HDC685) | 685 |
/// | スーパーフープ(KH785) | 25·Fc (Fc＜27.4) / 785 (27.4≦Fc) |
/// | パワーリング785(SPR785) | 25·Fc (Fc＜32.0) / 785 (32.0≦Fc) |
/// | エムケーフープ785(MK785) | 25·Fc (Fc＜31.4) / 785 (31.4≦Fc) |
///
/// 判別できない製品名は `None`（呼び出し側の既定 σwy を用いる）。
/// 許容応力度検定用の w_ft（[`high_strength_w_ft`]）とは別物であることに注意。
pub fn ultimate_hoop_sigma_wy(grade: &str, fc: f64) -> Option<f64> {
    let g = grade.trim().to_uppercase();
    let m = |cands: &[&str]| {
        cands
            .iter()
            .any(|c| g.starts_with(c.to_uppercase().as_str()))
    };
    let v = if is_ultimate_hoop_1275_class(grade) {
        (25.0 * fc).min(1275.0)
    } else if m(&[
        "UB785",
        "KW785",
        "KSS785",
        "ウルボン785",
        "リバーボン785",
        "ｳﾙﾎﾞﾝ785",
        "ﾘﾊﾞｰﾎﾞﾝ785",
        "スミフープ",
        "ストロングフープ",
        "デーフープ",
    ]) {
        (25.0 * fc).min(785.0)
    } else if m(&["SHD685", "SPR685"]) {
        (25.0 * fc).min(685.0)
    } else if m(&["HDC685"]) {
        685.0
    } else if m(&["KH785"]) {
        if fc < 27.4 {
            25.0 * fc
        } else {
            785.0
        }
    } else if m(&["SPR785"]) {
        if fc < 32.0 {
            25.0 * fc
        } else {
            785.0
        }
    } else if m(&["MK785"]) {
        if fc < 31.4 {
            25.0 * fc
        } else {
            785.0
        }
    } else {
        return None;
    };
    Some(v)
}

/// 高強度せん断補強筋使用時のコンクリート圧縮強度有効係数 ν0（終局検定・
/// 塑性理論式用の製品別式）。
///
/// - 1275 級（ウルボン1275・リバーボン1275）: `ν0 = 0.7·(1.0 − Fc/140)`
///   （恒等的に標準式 `0.7 − Fc/200` と一致する。0.7/140 = 1/200）
/// - その他の判別できた高強度製品（785/685 級）: `ν0 = 0.7·(0.7 − Fc/200)`
/// - 判別できない製品名は `None`（標準式 `ν0 = 0.7 − Fc/200` を用いる）
pub fn ultimate_hoop_nu0(grade: &str, fc: f64) -> Option<f64> {
    if is_ultimate_hoop_1275_class(grade) {
        Some((0.7 * (1.0 - fc / 140.0)).max(0.0))
    } else if ultimate_hoop_sigma_wy(grade, fc).is_some() {
        Some((0.7 * (0.7 - fc / 200.0)).max(0.0))
    } else {
        None
    }
}

/// 高強度せん断補強筋の**終局検定用** pw 上限（小数）。
///
/// 1275 級は柱かつ Fc＜27 N/mm² のとき 0.8%、それ以外 1.2%。その他の判別できた
/// 高強度製品は 1.2%。判別できない製品名は `None`（上限なし＝pw·σwy ≤ ν·Fc/2 の
/// 一般制約のみ）。許容応力度検定用の [`high_strength_pw_cap`] とは別物。
pub fn ultimate_hoop_pw_cap(grade: &str, fc: f64, is_column: bool) -> Option<f64> {
    if is_ultimate_hoop_1275_class(grade) {
        Some(if is_column && fc < 27.0 { 0.008 } else { 0.012 })
    } else if ultimate_hoop_sigma_wy(grade, fc).is_some() {
        Some(0.012)
    } else {
        None
    }
}
