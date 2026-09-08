//! [`SectionShape`] から [`Section`] を生成するビルダ（[`SectionShape::to_section`]）。

use super::constants::{KAPPA_RC, N_S_EQ};
use super::geometry::h_web_shear_area;
use super::types::SectionShape;
use crate::ids::SectionId;
use crate::model::Section;

impl SectionShape {
    /// Build a fully-populated `squid_n_core::Section` from the shape parameters.
    ///
    /// `id` and `name` must be supplied by the caller; all section properties
    /// are computed automatically. 階（`Section::floor`）は形状から決まらないため
    /// `None` とし、必要な呼び出し側（ST-Bridge 取り込み・断面形状の変更）が
    /// 生成後に設定する。
    pub fn to_section(&self, id: SectionId, name: String) -> Section {
        let area = self.calc_area();
        let iy = self.calc_iy();
        let iz = self.calc_iz();
        let j = self.calc_j();
        let (depth, width, as_y, as_z) = match *self {
            SectionShape::SteelH {
                height,
                width,
                web_thick,
                flange_thick,
            } => (
                height,
                width,
                2.0 * width * flange_thick,
                h_web_shear_area(height, web_thick),
            ),
            SectionShape::SteelBox {
                height,
                width,
                thick,
                ..
            }
            | SectionShape::CftBox {
                height,
                width,
                thick,
            } => (
                height,
                width,
                2.0 * thick * (width - 2.0 * thick).max(0.0),
                2.0 * thick * (height - 2.0 * thick).max(0.0),
            ),
            SectionShape::SteelAngle {
                leg_a,
                leg_b,
                thick,
            } => (
                leg_a.max(leg_b),
                leg_a.min(leg_b),
                leg_b * thick,
                leg_a * thick,
            ),
            SectionShape::SteelChannel {
                height,
                width,
                web_thick,
                flange_thick,
            } => (
                height,
                width,
                2.0 * width * flange_thick,
                h_web_shear_area(height, web_thick),
            ),
            SectionShape::SteelTee {
                height,
                width,
                web_thick,
                flange_thick,
            } => (
                height,
                width,
                width * flange_thick,
                h_web_shear_area(height, web_thick),
            ),
            SectionShape::SteelPipe { outer_dia, .. } | SectionShape::CftPipe { outer_dia, .. } => {
                (outer_dia, outer_dia, area / 2.0, area / 2.0)
            }
            SectionShape::SteelFlatBar { width, thick } => (
                thick,
                width,
                width * thick / KAPPA_RC,
                width * thick / KAPPA_RC,
            ),
            SectionShape::SteelRoundBar { dia } => (dia, dia, area * 0.9, area * 0.9),
            SectionShape::SteelLipChannel {
                height,
                width,
                thick,
                ..
            } => (
                height,
                width,
                2.0 * (width - thick) * thick,
                h_web_shear_area(height, thick),
            ),
            SectionShape::SteelBuiltH {
                height,
                upper_width,
                upper_thick,
                lower_width,
                lower_thick,
                web_thick,
            } => (
                height,
                upper_width.max(lower_width),
                upper_width * upper_thick + lower_width * lower_thick,
                h_web_shear_area(height, web_thick),
            ),
            SectionShape::RcRect { b, d, .. } => (d, b, b * d / KAPPA_RC, b * d / KAPPA_RC),
            SectionShape::RcCircle { d, .. } => (d, d, area / KAPPA_RC, area / KAPPA_RC),
            SectionShape::SrcRect {
                b,
                d,
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
                ..
            } => {
                let rc_as = b * d / KAPPA_RC;
                let s_web = h_web_shear_area(steel_height, steel_web_thick);
                let s_flange = 2.0 * steel_width * steel_flange_thick;
                (
                    d,
                    b,
                    rc_as + (N_S_EQ - 1.0) * s_flange,
                    rc_as + (N_S_EQ - 1.0) * s_web,
                )
            }
            SectionShape::RcWall { thickness, .. } | SectionShape::RcSlab { thickness } => (
                1000.0,
                thickness,
                1000.0 * thickness / KAPPA_RC,
                1000.0 * thickness / KAPPA_RC,
            ),
        };
        let thickness = match *self {
            SectionShape::CftBox { thick, .. } | SectionShape::CftPipe { thick, .. } => Some(thick),
            SectionShape::RcWall { thickness, .. } | SectionShape::RcSlab { thickness } => {
                Some(thickness)
            }
            _ => None,
        };
        Section {
            id,
            name,
            floor: None,
            area,
            iy,
            iz,
            j,
            depth,
            width,
            as_y,
            as_z,
            panel_thickness: None,
            thickness,
            shape: Some(self.clone()),
            material: None,
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        }
    }
}
