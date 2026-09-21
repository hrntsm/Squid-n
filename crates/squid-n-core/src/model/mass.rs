//! 断面の分布質量特性。

use super::{ElementData, Material, Model, Section};
use crate::section_shape::{one_bar_area, RcRebar, SectionShape};
use crate::units::{
    concrete_unit_weight_kn_m3, to_internal::mass_density_from_unit_weight_kn_m3,
    ConcreteComposition,
};

/// 線材断面の単位長さ当たり質量と断面回転慣性。
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SectionMassProperties {
    /// 単位長さ当たり質量 [t/mm]。
    pub mass_per_length: f64,
    /// 断面の y 軸まわり回転慣性の単位長さ当たり値 [t·mm]。
    pub rotary_inertia_y_per_length: f64,
    /// 断面の z 軸まわり回転慣性の単位長さ当たり値 [t·mm]。
    pub rotary_inertia_z_per_length: f64,
}

impl SectionMassProperties {
    /// 均質な断面から質量特性を作る。
    pub fn uniform(density: f64, area: f64, iy: f64, iz: f64) -> Self {
        Self {
            mass_per_length: density.max(0.0) * area.max(0.0),
            rotary_inertia_y_per_length: density.max(0.0) * iy.max(0.0),
            rotary_inertia_z_per_length: density.max(0.0) * iz.max(0.0),
        }
    }

    /// 材軸まわりの質量用極二次モーメントの単位長さ当たり値 [t·mm]。
    pub fn polar_inertia_per_length(self) -> f64 {
        self.rotary_inertia_y_per_length + self.rotary_inertia_z_per_length
    }

    /// 指定長さの総質量 [t]。
    pub fn total_mass(self, length: f64) -> f64 {
        self.mass_per_length.max(0.0) * length.max(0.0)
    }

    /// 形状がある断面は材料領域ごとに、形状がない断面は断面諸元と主材料から質量特性を求める。
    pub fn from_section(
        section: &Section,
        main: Option<&Material>,
        rebar: Option<&Material>,
        shear_rebar: Option<&Material>,
        steel: Option<&Material>,
    ) -> Self {
        let Some(shape) = section.shape.as_ref() else {
            return Self::uniform(
                main.map_or(0.0, |m| m.density),
                section.area,
                section.iy,
                section.iz,
            );
        };

        shaped_mass(shape, main, rebar, shear_rebar, steel)
    }
}

impl Model {
    /// 要素断面の材料領域から分布質量特性を求める。
    pub fn element_mass_properties(&self, elem: &ElementData) -> SectionMassProperties {
        let Some(section) = self.element_section(elem) else {
            return SectionMassProperties::default();
        };
        SectionMassProperties::from_section(
            section,
            self.element_material(elem),
            self.element_rebar_material(elem),
            self.element_shear_rebar_material(elem),
            self.element_steel_material(elem),
        )
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct GeometryMass {
    area: f64,
    iy: f64,
    iz: f64,
}

impl GeometryMass {
    fn add_round_bar(&mut self, area: f64, dia: f64, y: f64, z: f64) {
        let own_i = area * dia * dia / 16.0;
        self.area += area;
        self.iy += own_i + area * z * z;
        self.iz += own_i + area * y * y;
    }

    fn subtract(self, other: Self) -> Self {
        Self {
            area: (self.area - other.area).max(0.0),
            iy: (self.iy - other.iy).max(0.0),
            iz: (self.iz - other.iz).max(0.0),
        }
    }
}

#[derive(Default)]
struct WeightedMass {
    mass_per_length: f64,
    rotary_inertia_y_per_length: f64,
    rotary_inertia_z_per_length: f64,
}

impl WeightedMass {
    fn add(&mut self, density: f64, geometry: GeometryMass) {
        let density = density.max(0.0);
        self.mass_per_length += density * geometry.area.max(0.0);
        self.rotary_inertia_y_per_length += density * geometry.iy.max(0.0);
        self.rotary_inertia_z_per_length += density * geometry.iz.max(0.0);
    }

    fn finish(self) -> SectionMassProperties {
        SectionMassProperties {
            mass_per_length: self.mass_per_length,
            rotary_inertia_y_per_length: self.rotary_inertia_y_per_length,
            rotary_inertia_z_per_length: self.rotary_inertia_z_per_length,
        }
    }
}

fn shaped_mass(
    shape: &SectionShape,
    main: Option<&Material>,
    rebar: Option<&Material>,
    shear_rebar: Option<&Material>,
    steel: Option<&Material>,
) -> SectionMassProperties {
    let main_density = main.map_or(0.0, |m| m.density);
    let concrete_density = main.map_or(0.0, |m| m.density);
    let reinforcing_density = reinforcement_density(rebar);
    let shear_density = reinforcement_density(shear_rebar);
    let embedded_steel_density = steel_density(steel);
    let mut result = WeightedMass::default();

    match shape {
        SectionShape::RcRect { b, d, rebar } => {
            let gross = rectangle_geometry(*b, *d);
            let main_rebar = rectangular_rebar_geometry(rebar, *b, *d);
            let shear = rectangular_shear_geometry(rebar, *b, *d);
            result.add(concrete_density, gross.subtract(main_rebar).subtract(shear));
            result.add(reinforcing_density, main_rebar);
            result.add(shear_density, shear);
        }
        SectionShape::RcCircle { d, rebar } => {
            let gross = circle_geometry(*d);
            let main_rebar = circular_rebar_geometry(rebar, *d);
            let shear = circular_shear_geometry(rebar, *d);
            result.add(concrete_density, gross.subtract(main_rebar).subtract(shear));
            result.add(reinforcing_density, main_rebar);
            result.add(shear_density, shear);
        }
        SectionShape::SrcRect {
            b,
            d,
            rebar,
            steel_height,
            steel_width,
            steel_web_thick,
            steel_flange_thick,
        } => {
            let gross = rectangle_geometry(*b, *d);
            let main_rebar = rectangular_rebar_geometry(rebar, *b, *d);
            let shear = rectangular_shear_geometry(rebar, *b, *d);
            let embedded = SectionShape::SteelH {
                height: *steel_height,
                width: *steel_width,
                web_thick: *steel_web_thick,
                flange_thick: *steel_flange_thick,
            };
            let embedded = GeometryMass {
                area: embedded.calc_area(),
                iy: embedded.calc_iy(),
                iz: embedded.calc_iz(),
            };
            result.add(
                concrete_density,
                gross
                    .subtract(main_rebar)
                    .subtract(shear)
                    .subtract(embedded),
            );
            result.add(reinforcing_density, main_rebar);
            result.add(shear_density, shear);
            result.add(embedded_steel_density, embedded);
        }
        SectionShape::CftBox { .. } | SectionShape::CftPipe { .. } => {
            let steel = GeometryMass {
                area: shape.calc_area(),
                iy: shape.calc_iy(),
                iz: shape.calc_iz(),
            };
            if let Some(core) = shape.cft_core_props() {
                let concrete = GeometryMass {
                    area: core.area,
                    iy: core.iy,
                    iz: core.iz,
                };
                result.add(main_density, steel);
                result.add(cft_core_density(main), concrete);
            } else {
                result.add(main_density, steel);
            }
        }
        SectionShape::RcWall { .. } | SectionShape::RcSlab { .. } => {
            result.add(
                concrete_density,
                GeometryMass {
                    area: shape.calc_area(),
                    iy: shape.calc_iy(),
                    iz: shape.calc_iz(),
                },
            );
        }
        _ => {
            result.add(
                main_density,
                GeometryMass {
                    area: shape.calc_area(),
                    iy: shape.calc_iy(),
                    iz: shape.calc_iz(),
                },
            );
        }
    }

    result.finish()
}

fn rectangle_geometry(width: f64, depth: f64) -> GeometryMass {
    GeometryMass {
        area: (width * depth).max(0.0),
        iy: (width * depth.powi(3) / 12.0).max(0.0),
        iz: (depth * width.powi(3) / 12.0).max(0.0),
    }
}

fn circle_geometry(dia: f64) -> GeometryMass {
    let area = std::f64::consts::PI * dia * dia / 4.0;
    let inertia = std::f64::consts::PI * dia.powi(4) / 64.0;
    GeometryMass {
        area: area.max(0.0),
        iy: inertia.max(0.0),
        iz: inertia.max(0.0),
    }
}

fn rectangular_rebar_geometry(rebar: &RcRebar, width: f64, depth: f64) -> GeometryMass {
    use crate::rc_rebar_geom::rebar_layer_depth_from_edge;

    let mut result = GeometryMass::default();
    let add_set_x = |result: &mut GeometryMass, set: &crate::section_shape::BarSet| {
        let area = one_bar_area(set.dia);
        let layers = set.layers.max(1);
        let span = (width - 2.0 * rebar.cover).max(0.0);
        for layer in 0..layers {
            let z0 =
                depth / 2.0 - rebar_layer_depth_from_edge(rebar.cover, rebar.shear.dia, set, layer);
            for i in 0..set.count {
                let y = if set.count == 1 {
                    0.0
                } else {
                    -span / 2.0 + span * i as f64 / (set.count - 1) as f64
                };
                for zsign in [1.0, -1.0] {
                    result.add_round_bar(area, set.dia, y, zsign * z0);
                }
            }
        }
    };
    let add_set_y = |result: &mut GeometryMass, set: &crate::section_shape::BarSet| {
        let area = one_bar_area(set.dia);
        let layers = set.layers.max(1);
        let span = (depth - 2.0 * rebar.cover).max(0.0);
        for layer in 0..layers {
            let y0 =
                width / 2.0 - rebar_layer_depth_from_edge(rebar.cover, rebar.shear.dia, set, layer);
            for i in 0..set.count {
                let z = -span / 2.0 + span * (i as f64 + 1.0) / (set.count + 1) as f64;
                for ysign in [1.0, -1.0] {
                    result.add_round_bar(area, set.dia, ysign * y0, z);
                }
            }
        }
    };
    add_set_x(&mut result, &rebar.main_x);
    add_set_y(&mut result, &rebar.main_y);
    result
}

fn circular_rebar_geometry(rebar: &RcRebar, dia: f64) -> GeometryMass {
    let total = rebar.main_x.count + rebar.main_y.count;
    if total == 0 {
        return GeometryMass::default();
    }
    let depth = rebar.cover + rebar.shear.dia;
    let radius = (dia / 2.0 - depth).max(0.0);
    let mut result = GeometryMass::default();
    let mut index = 0_u32;
    for set in [&rebar.main_x, &rebar.main_y] {
        for _ in 0..set.count {
            let theta = 2.0 * std::f64::consts::PI * index as f64 / total as f64;
            let y = radius * theta.cos();
            let z = radius * theta.sin();
            result.add_round_bar(one_bar_area(set.dia), set.dia, y, z);
            index += 1;
        }
    }
    result
}

fn rectangular_shear_geometry(rebar: &RcRebar, width: f64, depth: f64) -> GeometryMass {
    let shear = &rebar.shear;
    if shear.pitch <= 0.0 || shear.legs == 0 || shear.dia <= 0.0 {
        return GeometryMass::default();
    }
    let center_width = (width - 2.0 * rebar.cover - shear.dia).max(0.0);
    let center_depth = (depth - 2.0 * rebar.cover - shear.dia).max(0.0);
    let scale = shear.legs as f64 / 2.0;
    let line_density = scale * one_bar_area(shear.dia) / shear.pitch;
    GeometryMass {
        area: line_density * 2.0 * (center_width + center_depth),
        iy: line_density * (center_width * center_depth.powi(2) / 2.0 + center_depth.powi(3) / 6.0),
        iz: line_density * (center_depth * center_width.powi(2) / 2.0 + center_width.powi(3) / 6.0),
    }
}

fn circular_shear_geometry(rebar: &RcRebar, dia: f64) -> GeometryMass {
    let shear = &rebar.shear;
    if shear.pitch <= 0.0 || shear.legs == 0 || shear.dia <= 0.0 {
        return GeometryMass::default();
    }
    let radius = (dia / 2.0 - rebar.cover - shear.dia / 2.0).max(0.0);
    let line_density = shear.legs as f64 / 2.0 * one_bar_area(shear.dia) / shear.pitch;
    let circumference = 2.0 * std::f64::consts::PI * radius;
    let inertia = std::f64::consts::PI * radius.powi(3);
    GeometryMass {
        area: line_density * circumference,
        iy: line_density * inertia,
        iz: line_density * inertia,
    }
}

fn cft_core_density(material: Option<&Material>) -> f64 {
    material.map_or(0.0, |m| {
        m.fc.filter(|fc| *fc > 0.0).map_or(0.0, |fc| {
            mass_density_from_unit_weight_kn_m3(concrete_unit_weight_kn_m3(
                fc,
                m.concrete_class,
                ConcreteComposition::Plain,
            ))
        })
    })
}

fn reinforcement_density(material: Option<&Material>) -> f64 {
    material.map_or(0.0, |m| m.density)
}

fn steel_density(material: Option<&Material>) -> f64 {
    material.map_or(0.0, |m| m.density)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{MaterialId, SectionId};
    use crate::model::{Material, MaterialCategory};

    fn material(id: u32, category: MaterialCategory, density: f64, fc: Option<f64>) -> Material {
        Material {
            id: MaterialId(id),
            name: format!("m{id}"),
            category,
            young: 20_000.0,
            poisson: 0.2,
            density,
            shear: None,
            fc,
            fy: None,
            concrete_class: Default::default(),
            strength_factor: None,
        }
    }

    #[test]
    fn 形状なしは主材料の断面諸元へフォールバックする() {
        let section = Section {
            id: SectionId(0),
            name: "plain".into(),
            area: 100.0,
            iy: 200.0,
            iz: 300.0,
            ..Section::zero(SectionId(0), "plain".into())
        };
        let properties = SectionMassProperties::from_section(
            &section,
            Some(&material(0, MaterialCategory::Steel, 2.0, None)),
            None,
            None,
            None,
        );
        assert_eq!(properties.mass_per_length, 200.0);
        assert_eq!(properties.rotary_inertia_y_per_length, 400.0);
        assert_eq!(properties.rotary_inertia_z_per_length, 600.0);
    }

    #[test]
    fn rcはコンクリートと主筋とせん断補強筋を重複なく積分する() {
        use crate::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

        let shape = SectionShape::RcRect {
            b: 400.0,
            d: 400.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 4,
                    dia: 20.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 4,
                    dia: 20.0,
                    layers: 1,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
        };
        let mut section = shape.to_section(SectionId(0), "RC".into());
        section.material = Some(MaterialId(0));
        section.rebar_material = Some(MaterialId(1));
        section.shear_rebar_material = Some(MaterialId(1));
        let concrete = material(0, MaterialCategory::Concrete, 2.4e-9, Some(24.0));
        let steel = material(1, MaterialCategory::Rebar, 7.0, None);
        let properties = SectionMassProperties::from_section(
            &section,
            Some(&concrete),
            Some(&steel),
            Some(&steel),
            None,
        );
        let gross = rectangle_geometry(400.0, 400.0);
        let main = rectangular_rebar_geometry(
            match &shape {
                SectionShape::RcRect { rebar, .. } => rebar,
                _ => unreachable!(),
            },
            400.0,
            400.0,
        );
        let shear = rectangular_shear_geometry(
            match &shape {
                SectionShape::RcRect { rebar, .. } => rebar,
                _ => unreachable!(),
            },
            400.0,
            400.0,
        );
        let concrete_density = concrete.density;
        let expected = concrete_density * (gross.area - main.area - shear.area)
            + 7.0 * (main.area + shear.area);
        assert!((properties.mass_per_length - expected).abs() / expected.abs() < 1e-12);
        let expected_iy =
            concrete_density * (gross.iy - main.iy - shear.iy) + 7.0 * (main.iy + shear.iy);
        let expected_iz =
            concrete_density * (gross.iz - main.iz - shear.iz) + 7.0 * (main.iz + shear.iz);
        assert!(
            (properties.rotary_inertia_y_per_length - expected_iy).abs() / expected_iy.abs()
                < 1e-12
        );
        assert!(
            (properties.rotary_inertia_z_per_length - expected_iz).abs() / expected_iz.abs()
                < 1e-12
        );
        assert!(
            properties.mass_per_length
                < concrete_density * gross.area + 7.0 * (main.area + shear.area)
        );
    }

    #[test]
    fn srcは内蔵鉄骨をコンクリート領域へ二重計上しない() {
        use crate::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

        let shape = SectionShape::SrcRect {
            b: 600.0,
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 0,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 0,
                    dia: 22.0,
                    layers: 1,
                },
                cover: 50.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
            steel_height: 400.0,
            steel_width: 200.0,
            steel_web_thick: 9.0,
            steel_flange_thick: 12.0,
        };
        let section = shape.to_section(SectionId(0), "SRC".into());
        let concrete = material(0, MaterialCategory::Concrete, 2.0, Some(24.0));
        let steel = material(1, MaterialCategory::Steel, 8.0, None);
        let properties = SectionMassProperties::from_section(
            &section,
            Some(&concrete),
            Some(&steel),
            Some(&steel),
            Some(&steel),
        );
        let h = SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 9.0,
            flange_thick: 12.0,
        };
        let steel_area = h.calc_area();
        let shear = rectangular_shear_geometry(
            match &shape {
                SectionShape::SrcRect { rebar, .. } => rebar,
                _ => unreachable!(),
            },
            600.0,
            600.0,
        );
        let expected = concrete.density * (600.0 * 600.0 - steel_area - shear.area)
            + 8.0 * steel_area
            + 8.0 * shear.area;
        assert!((properties.mass_per_length - expected).abs() < 1e-9);
        let double_counted =
            concrete.density * (600.0 * 600.0 - shear.area) + 8.0 * (steel_area + shear.area);
        assert!(properties.mass_per_length < double_counted);
        assert!(properties.mass_per_length > 8.0 * steel_area);
    }

    #[test]
    fn cftは主材料のfcからコア密度を導く() {
        let shape = SectionShape::CftBox {
            height: 400.0,
            width: 400.0,
            thick: 12.0,
        };
        let section = shape.to_section(SectionId(0), "CFT".into());
        let steel = material(0, MaterialCategory::Steel, 8.0, Some(24.0));
        let properties =
            SectionMassProperties::from_section(&section, Some(&steel), None, None, None);
        let core = shape.cft_core_props().unwrap();
        let rho_core = cft_core_density(Some(&steel));
        let expected = 8.0 * shape.calc_area() + rho_core * core.area;
        assert!((properties.mass_per_length - expected).abs() < 1e-9);
        assert!((rho_core - 23.0e-6 / crate::units::GRAVITY_MM_S2).abs() < 1e-18);
    }

    #[test]
    fn cftはfc未設定ならコアを質量計上しない() {
        let shape = SectionShape::CftBox {
            height: 400.0,
            width: 400.0,
            thick: 12.0,
        };
        let section = shape.to_section(SectionId(0), "CFT".into());
        let steel = material(0, MaterialCategory::Steel, 8.0, None);
        let properties =
            SectionMassProperties::from_section(&section, Some(&steel), None, None, None);
        assert!((properties.mass_per_length - 8.0 * shape.calc_area()).abs() < 1e-9);
        assert_eq!(cft_core_density(Some(&steel)), 0.0);
    }

    #[test]
    fn 未設定の補強材と鉄骨材質は質量を補完しない() {
        use crate::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

        let shape = SectionShape::RcRect {
            b: 400.0,
            d: 400.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 4,
                    dia: 20.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 4,
                    dia: 20.0,
                    layers: 1,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
        };
        let section = shape.to_section(SectionId(0), "RC".into());
        let concrete = material(0, MaterialCategory::Concrete, 2.4e-9, Some(24.0));
        let properties =
            SectionMassProperties::from_section(&section, Some(&concrete), None, None, None);
        let gross = rectangle_geometry(400.0, 400.0);
        let main = match &shape {
            SectionShape::RcRect { rebar, .. } => rectangular_rebar_geometry(rebar, 400.0, 400.0),
            _ => unreachable!(),
        };
        let shear = match &shape {
            SectionShape::RcRect { rebar, .. } => rectangular_shear_geometry(rebar, 400.0, 400.0),
            _ => unreachable!(),
        };
        let expected = concrete.density * (gross.area - main.area - shear.area);
        assert!((properties.mass_per_length - expected).abs() < 1e-12);
    }

    #[test]
    fn 主筋とせん断補強筋は対応する材料だけを使う() {
        use crate::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

        let shape = SectionShape::RcRect {
            b: 400.0,
            d: 400.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 4,
                    dia: 20.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 4,
                    dia: 20.0,
                    layers: 1,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
        };
        let section = shape.to_section(SectionId(0), "RC".into());
        let concrete = material(0, MaterialCategory::Concrete, 2.4e-9, Some(24.0));
        let main = material(1, MaterialCategory::Rebar, 7.0, None);
        let shear = material(2, MaterialCategory::Rebar, 8.0, None);
        let gross = rectangle_geometry(400.0, 400.0);
        let main_geometry = match &shape {
            SectionShape::RcRect { rebar, .. } => rectangular_rebar_geometry(rebar, 400.0, 400.0),
            _ => unreachable!(),
        };
        let shear_geometry = match &shape {
            SectionShape::RcRect { rebar, .. } => rectangular_shear_geometry(rebar, 400.0, 400.0),
            _ => unreachable!(),
        };

        let main_only =
            SectionMassProperties::from_section(&section, Some(&concrete), Some(&main), None, None);
        assert!(
            (main_only.mass_per_length
                - concrete.density * (gross.area - main_geometry.area - shear_geometry.area)
                - main.density * main_geometry.area)
                .abs()
                < 1e-12
        );

        let shear_only = SectionMassProperties::from_section(
            &section,
            Some(&concrete),
            None,
            Some(&shear),
            None,
        );
        assert!(
            (shear_only.mass_per_length
                - concrete.density * (gross.area - main_geometry.area - shear_geometry.area)
                - shear.density * shear_geometry.area)
                .abs()
                < 1e-12
        );
    }

    #[test]
    fn srcの内蔵鉄骨材料未設定時は主材料を流用しない() {
        use crate::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

        let shape = SectionShape::SrcRect {
            b: 600.0,
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 0,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 0,
                    dia: 22.0,
                    layers: 1,
                },
                cover: 50.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
            steel_height: 400.0,
            steel_width: 200.0,
            steel_web_thick: 9.0,
            steel_flange_thick: 12.0,
        };
        let section = shape.to_section(SectionId(0), "SRC".into());
        let main = material(0, MaterialCategory::Steel, 8.0, None);
        let embedded = SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 9.0,
            flange_thick: 12.0,
        };
        let properties =
            SectionMassProperties::from_section(&section, Some(&main), None, None, None);

        let shear = match &shape {
            SectionShape::SrcRect { rebar, .. } => rectangular_shear_geometry(rebar, 600.0, 600.0),
            _ => unreachable!(),
        };
        assert!(
            (properties.mass_per_length
                - main.density
                    * (rectangle_geometry(600.0, 600.0).area - embedded.calc_area() - shear.area))
                .abs()
                < 1e-9
        );
    }

    #[test]
    fn 負の単位長さ質量は総質量を負にしない() {
        let properties = SectionMassProperties {
            mass_per_length: -1.0,
            ..SectionMassProperties::default()
        };
        assert_eq!(properties.total_mass(100.0), 0.0);
    }
}
