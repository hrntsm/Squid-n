//! 鉄筋の許容応力度・降伏点。
//!
//! - [`rebar_allowable_tension`] — 異形鉄筋の許容引張・圧縮応力度 ft
//! - [`rebar_allowable_shear`] — せん断補強筋の許容引張応力度 w_ft
//! - [`rebar_sigma_y`] / [`rebar_sigma_y_of`] — 主筋の降伏点 σy（終局曲げ ΣMy 算定用）
//! - [`main_rebar_grade`] / [`shear_rebar_grade`] — 断面（配筋）の材質を第一とするグレード名解決

use squid_n_core::model::Material;
use squid_n_core::section_shape::RcRebar;

/// 異形鉄筋の許容引張・圧縮応力度 ft [N/mm²]。
///
/// SD345/SD390/SD490 は径 D29 以上（`dia >= 29.0`）で長期値が低減される
/// （215→195）。USD685（主筋として使う場合の高強度異形棒鋼）は技術評定値
/// どおり長期 215（径によらず、D29 以上の低減対象外）・短期 685 とする。
pub fn rebar_allowable_tension(grade: &str, dia: f64, long_term: bool) -> f64 {
    let g = grade.trim();
    if g == "USD685" {
        return if long_term { 215.0 } else { 685.0 };
    }
    if long_term {
        if g == "SR235" || g == "SR295" {
            155.0
        } else if g.starts_with("SD295") {
            195.0
        } else if g == "SD345" || g == "SD390" || g == "SD490" {
            if dia >= 29.0 {
                195.0
            } else {
                215.0
            }
        } else {
            195.0
        }
    } else if g == "SR235" {
        235.0
    } else if g == "SR295" || g.starts_with("SD295") {
        295.0
    } else if g == "SD345" {
        345.0
    } else if g == "SD390" {
        390.0
    } else if g == "SD490" {
        490.0
    } else {
        295.0
    }
}

/// せん断補強筋の許容引張応力度 w_ft [N/mm²]。
///
/// USD685 は技術評定値どおり長期 195・短期 590。SD490 短期はせん断のみ
/// F=390 に頭打ち。
pub fn rebar_allowable_shear(grade: &str, long_term: bool) -> f64 {
    let g = grade.trim();
    if g == "USD685" {
        return if long_term { 195.0 } else { 590.0 };
    }
    if long_term {
        if g == "SR235" {
            155.0
        } else {
            195.0
        }
    } else if g == "SR235" {
        // 短期は基準強度 F=235（令90条表。従来はフォールバック 295 に落ちて
        // F 値を 25% 超過する非保守側の誤りだった）。
        235.0
    } else if g == "SR295" || g.starts_with("SD295") {
        295.0
    } else if g == "SD345" {
        345.0
    } else if g == "SD390" {
        390.0
    } else if g == "SD490" {
        // F 値スケーリング: SD490 短期はせん断のみ F=390 に頭打ち。
        390.0
    } else {
        295.0
    }
}

/// 主筋のグレード名。断面（配筋）の主筋材質 [`RcRebar::main_grade`] を第一とし、
/// 未設定なら部材材料名を用いる（RC 部材の材料名が鉄筋グレード名を兼ねる従来挙動）。
pub fn main_rebar_grade<'a>(rebar: &'a RcRebar, mat: &'a Material) -> &'a str {
    rebar
        .main_grade
        .as_deref()
        .map(str::trim)
        .filter(|g| !g.is_empty())
        .unwrap_or(mat.name.as_str())
}

/// せん断補強筋のグレード名。断面（配筋）のせん断補強筋材質（`ShearBar::grade`）を
/// 第一とし、未設定なら部材材料名を用いる（従来挙動）。
pub fn shear_rebar_grade<'a>(rebar: &'a RcRebar, mat: &'a Material) -> &'a str {
    rebar
        .shear
        .grade
        .as_deref()
        .map(str::trim)
        .filter(|g| !g.is_empty())
        .unwrap_or(mat.name.as_str())
}

/// 主筋の降伏点 σy [N/mm²]（終局曲げ ΣMy 算定用）。断面（配筋）の主筋材質から
/// 解決し、未設定なら [`rebar_sigma_y`]（材料の `fy` → 材料名）へフォールバックする。
pub fn rebar_sigma_y_of(rebar: &RcRebar, mat: &Material) -> f64 {
    rebar
        .main_grade
        .as_deref()
        .and_then(squid_n_core::material_grade::rebar_grade_f_value)
        .unwrap_or_else(|| rebar_sigma_y(mat))
}

/// 主筋の降伏点 σy [N/mm²]（終局曲げ ΣMy 算定用）。
///
/// `Material.fy` があればそれを、なければ材料名（鉄筋グレード名）の数値部
/// （例 "SD345"→345）を、どちらもなければ 345（SD345 相当）を用いる。
/// 断面に主筋材質が設定されている場合は [`rebar_sigma_y_of`] を用いること。
pub fn rebar_sigma_y(mat: &Material) -> f64 {
    if let Some(fy) = mat.fy {
        if fy > 0.0 {
            return fy;
        }
    }
    let digits: String = mat.name.chars().filter(|c| c.is_ascii_digit()).collect();
    digits
        .parse::<f64>()
        .ok()
        .filter(|v| *v > 0.0)
        .unwrap_or(345.0)
}
