//! 鋼構造の許容応力度と断面検定（許容応力度検定。
//! 根拠規準は鋼構造設計規準 1973・構造規定）。
//! 形状は `Section.shape` 優先、なければ `Section.name` から推定し、合わなければ `Other` とする。

use crate::{CheckOutcome, DesignCheck, DesignCtx, MemberForcesAt, MemberKind};
use squid_n_core::model::{Material, Section};
use squid_n_core::section_shape::SectionShape;

pub use crate::material_strength::{
    big_lambda, plate_thickness, steel_f_value, steel_f_value_prefix, steel_fc, steel_fs, steel_ft,
};

mod beam;
/// 鉄骨造梁の非線形復元力特性（全塑性 Mp・横座屈 Mcr・軸 Nu）。
/// 非線形解析の材端バネ骨格に用いる（鋼構造塑性設計指針）。
pub mod beam_nonlinear;
mod brace;
/// 鉄骨造柱の座屈長さ係数 K（鋼構造塑性設計指針、水平移動非拘束）。
pub mod buckling;
pub mod cold_formed;
mod column;
pub mod panel_zone;
mod section;

pub use section::{resolve_lb, steel_fb_h, steel_fb_h_new, steel_h_z_with_loss, steel_i_t};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ShapeCategory {
    H,
    Box,
    Pipe,
    Other,
}

/// `Section.name` の先頭アルファベットトークンから形状カテゴリを推定する。
/// 例: "H-300x300x10x15"→H、"BOX-200x200x12"→Box、"PIPE-216.3x8.2"→Pipe。
/// 該当しない場合は `Other`（一般断面フォールバック）。
fn classify_shape(name: &str) -> ShapeCategory {
    let token: String = name
        .trim()
        .to_uppercase()
        .chars()
        .take_while(|c| c.is_ascii_alphabetic())
        .collect();
    match token.as_str() {
        "H" => ShapeCategory::H,
        "BOX" => ShapeCategory::Box,
        "PIPE" | "P" => ShapeCategory::Pipe,
        _ => ShapeCategory::Other,
    }
}

/// 形状カテゴリと板厚 `(カテゴリ, tf, tw)` を解決する。
/// `Section.shape` があれば実寸、なければ `Section.thickness` を `tf ≈ tw` とする。
fn shape_of(sec: &Section) -> (ShapeCategory, f64, f64) {
    if let Some(shape) = &sec.shape {
        match *shape {
            SectionShape::SteelH {
                web_thick,
                flange_thick,
                ..
            } => return (ShapeCategory::H, flange_thick, web_thick),
            SectionShape::SteelBuiltH {
                web_thick,
                upper_thick,
                lower_thick,
                ..
            } => return (ShapeCategory::H, upper_thick.min(lower_thick), web_thick),
            SectionShape::SteelBox { thick, .. } => return (ShapeCategory::Box, thick, thick),
            SectionShape::SteelPipe { thick, .. } => return (ShapeCategory::Pipe, thick, thick),
            SectionShape::SteelChannel {
                web_thick,
                flange_thick,
                ..
            }
            | SectionShape::SteelTee {
                web_thick,
                flange_thick,
                ..
            } => return (ShapeCategory::Other, flange_thick, web_thick),
            SectionShape::SteelAngle { thick, .. } => return (ShapeCategory::Other, thick, thick),
            SectionShape::SteelFlatBar { thick, .. } => {
                return (ShapeCategory::Other, thick, thick)
            }
            SectionShape::SteelRoundBar { dia } => return (ShapeCategory::Other, dia, dia),
            SectionShape::SteelLipChannel { thick, .. } => {
                return (ShapeCategory::Other, thick, thick)
            }
            SectionShape::CftBox { thick, .. } => return (ShapeCategory::Box, thick, thick),
            SectionShape::CftPipe { thick, .. } => return (ShapeCategory::Pipe, thick, thick),
            SectionShape::RcRect { .. }
            | SectionShape::RcCircle { .. }
            | SectionShape::SrcRect { .. }
            | SectionShape::RcWall { .. }
            | SectionShape::RcSlab { .. } => return (ShapeCategory::Other, 0.0, 0.0),
        }
    }
    let t = sec.thickness.unwrap_or(0.0);
    (classify_shape(&sec.name), t, t)
}

/// せん断有効断面積 `(Ay, Az)` [mm²]（強軸 Qy・弱軸 Qz。梁・柱で共用の単一定義）。
/// 断面形状ごとに算定する（角部外半径 r は断面定義時の入力値）。
fn shear_area_2d(shape: ShapeCategory, sec: &Section, tf: f64, tw: f64) -> (f64, f64) {
    let h = sec.depth;
    let b = sec.width;
    match shape {
        ShapeCategory::H => {
            let ay = (tw * (h - 2.0 * tf).max(0.0)).max(0.0);
            let az = (2.0 * b * tf / 1.5).max(0.0);
            (ay, az)
        }
        ShapeCategory::Box => {
            let t = tw;
            let r = match &sec.shape {
                Some(SectionShape::SteelBox { corner_r, .. }) => corner_r.max(0.0),
                _ => 0.0,
            };
            let (ay, az) = if r > 1e-9 {
                let corner = (std::f64::consts::PI * t * (2.0 * r - t) / 4.0).max(0.0);
                (
                    2.0 * (t * (h - 2.0 * r).max(0.0) + corner),
                    2.0 * (t * (b - 2.0 * r).max(0.0) + corner),
                )
            } else {
                // 角部直角（未入力・CftBox・名前推定フォールバック）。
                (
                    2.0 * t * (h - 2.0 * t).max(0.0),
                    2.0 * t * (b - 2.0 * t).max(0.0),
                )
            };
            (ay.max(0.0), az.max(0.0))
        }
        ShapeCategory::Pipe => {
            let t = tw;
            let d = sec.depth;
            let a = (std::f64::consts::PI * t * (d - t) / 2.0).max(0.0);
            (a, a)
        }
        ShapeCategory::Other => {
            let ay = if sec.as_z > 0.0 { sec.as_z } else { sec.area };
            let az = if sec.as_y > 0.0 { sec.as_y } else { sec.area };
            (ay, az)
        }
    }
}

/// 分母が極小の場合に安全側デフォルトへ逃がすヘルパー。
fn safe_denom(x: f64) -> f64 {
    if x.abs() > 1e-9 {
        x
    } else {
        1e-9
    }
}

/// 断面係数 Z = I / (半せい)。半せいが極小なら 0（呼び出し側で 1.0 にフォールバック）。
fn section_modulus(i: f64, half_dim: f64) -> f64 {
    if half_dim > 1e-9 {
        i / half_dim
    } else {
        0.0
    }
}

fn nonzero(z: f64) -> f64 {
    if z.abs() > 1e-9 {
        z
    } else {
        1.0
    }
}

pub struct SteelDesign;

impl DesignCheck for SteelDesign {
    fn check(
        &self,
        forces: &MemberForcesAt,
        sec: &Section,
        mat: &Material,
        ctx: &DesignCtx,
    ) -> CheckOutcome {
        let t = plate_thickness(sec);
        // プリセット外の直接入力材料は fy を基準強度として用いる（それもなければ 235）。
        let f = steel_f_value_prefix(&mat.name, t)
            .or(mat.fy)
            .unwrap_or(235.0);
        let term = ctx.term;

        let cr = match ctx.kind {
            MemberKind::Beam => beam::check_beam(forces, sec, mat, ctx, f, term),
            MemberKind::Column => column::check_column(forces, sec, mat, ctx, f, term),
            MemberKind::Brace => brace::check_brace(forces, sec, ctx, f, term),
        };
        CheckOutcome::Checked(cr)
    }
}

/// テスト共通ヘルパー（`mat`/`rect_section`/`h_section`）。各サブモジュールの
/// テストから `super::super::test_support::*` として共有する。
#[cfg(test)]
pub(crate) mod test_support {
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::model::{Material, MaterialCategory, Section};
    use squid_n_core::section_shape::SectionShape;

    pub(crate) fn mat(name: &str) -> Material {
        Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: name.to_string(),
            category: MaterialCategory::Steel,
            young: 205_000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: None,
        }
    }

    pub(crate) fn rect_section(b: f64, d: f64, name: &str) -> Section {
        Section {
            id: SectionId(0),
            name: name.to_string(),
            area: b * d,
            iy: b * d.powi(3) / 12.0,
            iz: d * b.powi(3) / 12.0,
            j: 0.0,
            depth: d,
            width: b,
            as_y: 0.0,
            as_z: 0.0,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: None,
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        }
    }

    /// `SectionShape::SteelH` 付きの断面（実寸 tf/tw を持つ正規経路の検証用）。
    pub(crate) fn h_section(h: f64, b: f64, tw: f64, tf: f64) -> Section {
        let shape = SectionShape::SteelH {
            height: h,
            width: b,
            web_thick: tw,
            flange_thick: tf,
        };
        shape.to_section(SectionId(0), format!("H-{}x{}x{}x{}", h, b, tw, tf))
    }
}
