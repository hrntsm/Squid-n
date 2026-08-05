//! 鋼材の許容応力度（鋼構造設計規準 1973・構造規定。板厚 [mm] 区分対応の F 値）。
//!
//! F 値の対応表（グレード名→F 値）は `squid_n_core::material_grade` に
//! 一本化されており、本モジュールはそれを再エクスポートする。
//!
//! - [`steel_f_value`] — F 値（完全一致、板厚区分対応。core への再エクスポート）
//! - [`steel_f_value_prefix`] — F 値（前方一致、板厚区分対応。core への再エクスポート）
//! - [`plate_thickness`] — F 値区分判定に用いる最大板厚
//! - [`steel_ft`] — 許容引張／曲げ応力度 ft
//! - [`steel_fs`] — 許容せん断応力度 fs
//! - [`big_lambda`] — 限界細長比 Λ
//! - [`steel_fc`] — 許容圧縮応力度 fc（座屈考慮）

use crate::LoadTerm;
use squid_n_core::model::Section;
use squid_n_core::section_shape::SectionShape;

/// 鋼材の F 値 [N/mm²]（完全一致、板厚 [mm] 区分対応）。対応表の実体は
/// [`squid_n_core::material_grade::steel_f_value`] を参照。
pub use squid_n_core::material_grade::steel_f_value;

/// 鋼材の F 値 [N/mm²]（前方一致、板厚 [mm] 区分対応）。対応表の実体は
/// [`squid_n_core::material_grade::steel_f_value_prefix`] を参照。
pub use squid_n_core::material_grade::steel_f_value_prefix;

/// F 値の板厚区分判定に用いる最大板厚 [mm]。
///
/// `Section.shape` があれば形状の最大板厚、なければ `Section.thickness`、
/// いずれもなければ 40mm 以下区分として扱う。
pub fn plate_thickness(sec: &Section) -> f64 {
    if let Some(shape) = &sec.shape {
        match *shape {
            SectionShape::SteelH {
                web_thick,
                flange_thick,
                ..
            }
            | SectionShape::SteelChannel {
                web_thick,
                flange_thick,
                ..
            }
            | SectionShape::SteelTee {
                web_thick,
                flange_thick,
                ..
            } => return web_thick.max(flange_thick),
            SectionShape::SteelBox { thick, .. }
            | SectionShape::SteelAngle { thick, .. }
            | SectionShape::SteelPipe { thick, .. }
            | SectionShape::SteelFlatBar { thick, .. }
            | SectionShape::SteelLipChannel { thick, .. }
            | SectionShape::CftBox { thick, .. }
            | SectionShape::CftPipe { thick, .. } => return thick,
            // 中実丸鋼は板要素ではないため、板厚区分は全断面の径で判定する。
            SectionShape::SteelRoundBar { dia } => return dia,
            SectionShape::SteelBuiltH {
                web_thick,
                upper_thick,
                lower_thick,
                ..
            } => return web_thick.max(upper_thick).max(lower_thick),
            SectionShape::SrcRect {
                steel_web_thick,
                steel_flange_thick,
                ..
            } => return steel_web_thick.max(steel_flange_thick),
            SectionShape::RcRect { .. }
            | SectionShape::RcCircle { .. }
            | SectionShape::RcWall { .. } => {}
        }
    }
    sec.thickness.unwrap_or(40.0)
}

/// 長期・短期許容引張／曲げ（座屈無視）応力度 ft [N/mm²]。
///
/// `長期 = F/1.5`、`短期 = F`。
pub fn steel_ft(f: f64, term: LoadTerm) -> f64 {
    match term {
        LoadTerm::Long => f / 1.5,
        LoadTerm::Short => f,
    }
}

/// 長期・短期許容せん断応力度 fs [N/mm²]。
///
/// `長期 = F/(1.5·√3)`、`短期 = F/√3`。
pub fn steel_fs(f: f64, term: LoadTerm) -> f64 {
    match term {
        LoadTerm::Long => f / (1.5 * 3.0_f64.sqrt()),
        LoadTerm::Short => f / 3.0_f64.sqrt(),
    }
}

/// 限界細長比 `Λ = 1500/√(F/1.5)`。
pub fn big_lambda(f: f64) -> f64 {
    1500.0 / (f / 1.5).sqrt()
}

/// 長期・短期許容圧縮応力度 fc [N/mm²]（座屈考慮、鋼構造設計規準 1973）。
///
/// 有効細長比 `λ = lk/i`、限界細長比 `Λ = 1500/√(F/1.5)`、
/// 安全率 `ν = 3/2 + (2/3)(λ/Λ)²` として:
/// - `λ ≤ Λ`: `fcL = F·(1 − 0.4(λ/Λ)²) / ν`
/// - `λ > Λ`: `fcL = (18/65)·F/(λ/Λ)²`
///
/// 短期は長期の 1.5 倍。`λ=0` のとき `fcL = F/1.5`（= ft 長期）と一致し、
/// `λ=Λ` で両分岐は連続する（`fcL = (18/65)F`）。
pub fn steel_fc(f: f64, lambda: f64, term: LoadTerm) -> f64 {
    let big_l = big_lambda(f);
    let r = if big_l > 1e-9 { lambda / big_l } else { 0.0 };
    let fc_long = if lambda <= big_l {
        let nu = 1.5 + (2.0 / 3.0) * r * r;
        f * (1.0 - 0.4 * r * r) / nu
    } else {
        (18.0 / 65.0) * f / (r * r)
    };
    match term {
        LoadTerm::Long => fc_long,
        LoadTerm::Short => fc_long * 1.5,
    }
}
