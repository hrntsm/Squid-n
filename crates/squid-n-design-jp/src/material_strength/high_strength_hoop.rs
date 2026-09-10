//! 高強度せん断補強筋（許容せん断応力度・pw 上限）。
//! `ShearBar.grade` に製品名がある場合は高強度品用テーブルを用いる。

use crate::LoadTerm;

/// 高強度せん断補強筋の製品グループ（pw 上限値の判定用）。
/// 長期は全製品 0.6% で共通。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HighStrengthGroup {
    /// ウルボン系・SPR785。
    UlbonSeries,
    /// リバーボン785・スミフープ等・HDC685。
    Kw785Series,
    /// スーパーフープ KH785。
    Kh785,
    /// スーパーフープ KH685・パワーリング SPR685。
    Kh685Series,
    /// UHYフープ SHD685・エムケーフープ MK785。
    Shd685OrMk785,
    /// 上記以外（判別不能な高強度品）。
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

/// 高強度せん断補強筋かどうかの判定（core に一本化）。
pub use squid_n_core::material_grade::is_high_strength_shear_grade;

/// 高強度せん断補強筋の許容せん断応力度 w_ft [N/mm²]（製品表）。
/// 長期 195、短期 585/590。
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

/// 高強度せん断補強筋使用時の pw 上限値（小数）。
/// 長期 0.6%、短期は製品グループ・Fc による。
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
        HighStrengthGroup::Kh785 => (0.012_f64).min(0.010 * fc / 27.0),
        HighStrengthGroup::Kh685Series => (0.012_f64).min(0.012 * fc / 27.0),
        HighStrengthGroup::Shd685OrMk785 => 0.012,
        HighStrengthGroup::Other => 0.008,
    }
}

/// 終局検定用の高強度せん断補強筋の製品判別（大文字化・前方一致）。
/// 1275 級なら true。
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

/// 高強度せん断補強筋の終局検定用 σwy [N/mm²]（製品別表）。`fc` は Fc(raw)。
/// 判別できない製品名は `None`。
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

/// 高強度せん断補強筋使用時のコンクリート圧縮強度有効係数 ν0（終局検定用）。
/// 判別できない製品名は `None`。
pub fn ultimate_hoop_nu0(grade: &str, fc: f64) -> Option<f64> {
    if is_ultimate_hoop_1275_class(grade) {
        Some((0.7 * (1.0 - fc / 140.0)).max(0.0))
    } else if ultimate_hoop_sigma_wy(grade, fc).is_some() {
        Some((0.7 * (0.7 - fc / 200.0)).max(0.0))
    } else {
        None
    }
}

/// 高強度せん断補強筋の終局検定用 pw 上限（小数）。
/// 判別できない製品名は `None`。
pub fn ultimate_hoop_pw_cap(grade: &str, fc: f64, is_column: bool) -> Option<f64> {
    if is_ultimate_hoop_1275_class(grade) {
        Some(if is_column && fc < 27.0 { 0.008 } else { 0.012 })
    } else if ultimate_hoop_sigma_wy(grade, fc).is_some() {
        Some(0.012)
    } else {
        None
    }
}
