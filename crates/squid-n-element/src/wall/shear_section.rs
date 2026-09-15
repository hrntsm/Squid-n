use super::section_geometry::{Primitive, SectionGeometry};
use crate::transform::LocalFrame;
use squid_n_core::geom::vec3::dot;
use squid_n_core::model::{wall_element_geometry, ElementData, Model};
use squid_n_core::section_shape::{
    concrete_young_modulus, material_strip_section_properties, MaterialSectionStrip,
    MaterialStripSectionProperties, SectionShape,
};

#[derive(Clone, Copy, PartialEq)]
struct Elasticity {
    young: f64,
    shear: f64,
}

pub(super) struct ColumnSection {
    geometry: SectionGeometry,
    direction: [f64; 2],
    center: f64,
    materials: Vec<Elasticity>,
    pub extent: [f64; 2],
    pub element_index: usize,
}

pub(super) struct WallSection {
    pub columns: [Option<ColumnSection>; 2],
    length: f64,
    thickness: f64,
    material: Elasticity,
}

impl WallSection {
    pub fn new(data: &ElementData, model: &Model) -> Result<Self, String> {
        let geometry = wall_element_geometry(data, model).ok_or("壁の幾何が不正です")?;
        let section = model.element_section(data).ok_or("壁の断面が未指定です")?;
        let material = model.element_material(data).ok_or("壁の材料が未指定です")?;
        let thickness = match section.shape {
            Some(SectionShape::RcWall { thickness, .. }) => thickness,
            _ => section.thickness.unwrap_or(section.width),
        };
        let mut result = Self {
            columns: [None, None],
            length: geometry.lw,
            thickness,
            material: Elasticity {
                young: material.young,
                shear: material.shear_modulus(),
            },
        };
        if super::misc_wall::wall_is_seismic(data, model) {
            for (element_index, e) in model.elements.iter().enumerate() {
                if !super::side_column::is_side_column_member(e.kind) || e.nodes.len() != 2 {
                    continue;
                }
                let Some(side) = (0..2).find(|&side| {
                    let pair = [geometry.bottom[side], geometry.top[side]];
                    e.nodes.as_slice() == pair || e.nodes.as_slice() == [pair[1], pair[0]]
                }) else {
                    continue;
                };
                if result.columns[side].is_some() {
                    return Err("同じ壁辺に側柱が重複しています".into());
                }
                let sec = model.element_section(e).ok_or("側柱の断面が未指定です")?;
                let shape = sec.shape.as_ref().ok_or("側柱の断面形状が未定義です")?;
                let mat = model.element_material(e).ok_or("側柱の材料が未指定です")?;
                let mut materials = vec![Elasticity {
                    young: mat.young,
                    shear: mat.shear_modulus(),
                }];
                match shape {
                    SectionShape::SrcRect { .. } => {
                        let steel = model
                            .element_steel_material(e)
                            .ok_or("SRC側柱の内蔵鉄骨材料が未指定です")?;
                        materials.push(Elasticity {
                            young: steel.young,
                            shear: steel.shear_modulus(),
                        });
                    }
                    SectionShape::CftBox { .. } | SectionShape::CftPipe { .. } => {
                        let fc = mat
                            .fc
                            .filter(|f| f.is_finite() && *f > 0.0)
                            .ok_or("CFT側柱の充填コンクリート強度が未指定・不正です")?;
                        let young = concrete_young_modulus(fc);
                        materials.push(Elasticity {
                            young,
                            shear: young / 2.4,
                        });
                    }
                    _ => {}
                }
                let p0 = model.nodes[e.nodes[0].index()].coord;
                let p1 = model.nodes[e.nodes[1].index()].coord;
                let frame = LocalFrame::from_nodes(p0, p1, e.local_axis.ref_vector);
                let direction = [
                    dot(frame.rot[1], geometry.ex_bottom),
                    dot(frame.rot[2], geometry.ex_bottom),
                ];
                if (direction[0].hypot(direction[1]) - 1.0).abs() > 1e-10 {
                    return Err("側柱の材軸と壁下辺が直交していません".into());
                }
                let geometry = SectionGeometry::of(shape)?;
                let extrema: Vec<_> = geometry
                    .parts
                    .iter()
                    .flat_map(|(p, _)| p.breaks(direction))
                    .collect();
                let extent = [
                    extrema.iter().copied().fold(f64::INFINITY, f64::min),
                    extrema.iter().copied().fold(f64::NEG_INFINITY, f64::max),
                ];
                result.columns[side] = Some(ColumnSection {
                    geometry,
                    direction,
                    center: side as f64 * result.length,
                    materials,
                    extent,
                    element_index,
                });
            }
        }
        if ![result.length, result.thickness]
            .iter()
            .all(|v| v.is_finite() && *v > 0.0)
            || std::iter::once(&result.material)
                .chain(result.columns.iter().flatten().flat_map(|c| &c.materials))
                .any(|m| ![m.young, m.shear].iter().all(|v| v.is_finite() && *v > 0.0))
        {
            return Err("壁・側柱の寸法または材料定数が不正です".into());
        }
        Ok(result)
    }

    fn strip(&self, x: f64, length_mm: f64) -> Result<MaterialSectionStrip, String> {
        let intervals: Vec<Vec<_>> = self
            .columns
            .iter()
            .flatten()
            .map(|c| {
                c.geometry
                    .parts
                    .iter()
                    .filter_map(|(p, material)| {
                        p.interval(x - c.center, c.direction)
                            .map(|span| (span, material.map(|i| c.materials[i])))
                    })
                    .collect()
            })
            .collect();
        let wall = if x >= 0.0 && x <= self.length {
            Some([-self.thickness / 2.0, self.thickness / 2.0])
        } else {
            None
        };
        let mut edges: Vec<_> = intervals
            .iter()
            .flatten()
            .flat_map(|(span, _)| *span)
            .chain(wall.into_iter().flatten())
            .collect();
        edges.sort_by(f64::total_cmp);
        edges.dedup();
        let (mut width, mut eb, mut gb) = (0.0, 0.0, 0.0);
        for pair in edges.windows(2) {
            let mid = (pair[0] + pair[1]) / 2.0;
            let mut owner: Option<Elasticity> = None;
            let mut void = false;
            for column in &intervals {
                let region = column
                    .iter()
                    .rev()
                    .find(|(span, _)| span[0] <= mid && mid <= span[1]);
                void |= region.is_some_and(|(_, mat)| mat.is_none());
                let material = region.and_then(|(_, mat)| *mat);
                if let Some(mat) = material {
                    if owner.is_some_and(|other| other != mat) {
                        return Err("異なる材料定数の側柱断面が重なっています".into());
                    }
                    owner = Some(mat);
                }
            }
            if owner.is_none() && !void && wall.is_some_and(|span| span[0] <= mid && mid <= span[1])
            {
                owner = Some(self.material);
            }
            if let Some(mat) = owner {
                let b = pair[1] - pair[0];
                width += b;
                eb += b * mat.young;
                gb += b * mat.shear;
            }
        }
        if width <= 0.0 {
            return Err("壁のせん断断面が壁長方向に分離しています".into());
        }
        Ok(MaterialSectionStrip {
            length_mm,
            width_mm: width,
            young_mpa: eb / width,
            shear_mpa: gb / width,
        })
    }

    pub fn properties(&self) -> Result<MaterialStripSectionProperties, String> {
        let mut boundaries = vec![0.0, self.length];
        for c in self.columns.iter().flatten() {
            for (p, _) in &c.geometry.parts {
                boundaries.extend(p.breaks(c.direction).iter().map(|x| x + c.center));
                boundaries.extend(
                    wall_intersections(*p, c.direction, self.thickness)
                        .iter()
                        .map(|x| x + c.center),
                );
            }
        }
        boundaries.sort_by(f64::total_cmp);
        boundaries.dedup_by(|a, b| (*a - *b).abs() <= 1e-10);
        let mut previous: Option<MaterialStripSectionProperties> = None;
        let mut stable = 0;
        for power in 0..=13 {
            let n = 1 << power;
            let mut strips = Vec::with_capacity((boundaries.len() - 1) * n);
            for pair in boundaries.windows(2) {
                let step = (pair[1] - pair[0]) / n as f64;
                for i in 0..n {
                    strips.push(self.strip(pair[0] + (i as f64 + 0.5) * step, step)?);
                }
            }
            let mut p = material_strip_section_properties(&strips)
                .ok_or("壁のせん断剛性を算定できません")?;
            p.elastic_centroid_mm += boundaries[0];
            if let Some(old) = previous {
                let values = |p: &MaterialStripSectionProperties| {
                    [p.area_mm2, p.flexural_rigidity_n_mm2, p.shear_rigidity_n]
                };
                let close = values(&p)
                    .into_iter()
                    .zip(values(&old))
                    .all(|(a, b)| (a - b).abs() <= 1e-6 * a.abs())
                    && (p.elastic_centroid_mm - old.elastic_centroid_mm).abs()
                        <= 1e-6 * (boundaries.last().unwrap() - boundaries[0]);
                stable = if close { stable + 1 } else { 0 };
                if stable >= 2 {
                    return Ok(p);
                }
            }
            previous = Some(p);
        }
        Err("壁のせん断断面積分が収束しません（相対許容差1e-6）".into())
    }
}

fn wall_intersections(p: Primitive, [a, b]: [f64; 2], thickness: f64) -> Vec<f64> {
    let mut result = Vec::new();
    for z_wall in [-thickness / 2.0, thickness / 2.0] {
        match p {
            Primitive::Circle {
                center: [y, z],
                radius,
            } => {
                let square = radius * radius - (z_wall - (-b * y + a * z)).powi(2);
                if square > 0.0 {
                    result.extend([a * y + b * z - square.sqrt(), a * y + b * z + square.sqrt()]);
                }
            }
            Primitive::Rectangle { y, z } => {
                let vertices = [[y[0], z[0]], [y[1], z[0]], [y[1], z[1]], [y[0], z[1]]];
                for i in 0..4 {
                    let [y0, z0] = vertices[i];
                    let [y1, z1] = vertices[(i + 1) % 4];
                    let v0 = -b * y0 + a * z0;
                    let v1 = -b * y1 + a * z1;
                    if v0 != v1 {
                        let t = (z_wall - v0) / (v1 - v0);
                        if (0.0..=1.0).contains(&t) {
                            result.push(a * (y0 + t * (y1 - y0)) + b * (z0 + t * (z1 - z0)));
                        }
                    }
                }
            }
        }
    }
    result
}

/// 壁と側柱の重複を控除した面内せん断剛性 [N]。入力不備・積分未収束は理由を返す。
pub fn wall_shear_rigidity(data: &ElementData, model: &Model) -> Result<f64, String> {
    WallSection::new(data, model)?
        .properties()
        .map(|p| p.shear_rigidity_n)
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};

    fn rebar() -> RcRebar {
        RcRebar {
            main_x: BarSet {
                count: 8,
                dia: 22.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 4,
                dia: 22.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
            },
        }
    }
    fn section(shape: &SectionShape, angle: f64) -> WallSection {
        let direction = [angle.cos(), angle.sin()];
        let geometry = SectionGeometry::of(shape).unwrap();
        let breaks: Vec<_> = geometry
            .parts
            .iter()
            .flat_map(|(p, _)| p.breaks(direction))
            .collect();
        let extent = [
            breaks.iter().copied().fold(f64::INFINITY, f64::min),
            breaks.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        ];
        let material = Elasticity {
            young: 30000.0,
            shear: 12500.0,
        };
        WallSection {
            columns: [
                Some(ColumnSection {
                    geometry,
                    direction,
                    center: 0.0,
                    materials: vec![
                        material,
                        Elasticity {
                            young: 205000.0,
                            shear: 80000.0,
                        },
                    ],
                    extent,
                    element_index: 0,
                }),
                None,
            ],
            length: 4000.0,
            thickness: 150.0,
            material,
        }
    }

    #[test]
    fn rotated_rectangle_union_matches_geometric_area() {
        let shape = SectionShape::RcRect {
            b: 500.0,
            d: 800.0,
            rebar: rebar(),
        };
        for angle in [0.0, 0.3, 0.7, std::f64::consts::FRAC_PI_2] {
            let wall = section(&shape, angle);
            let p = wall.properties().unwrap();
            let mirrored = section(&shape, -angle).properties().unwrap();
            assert!((p.area_mm2 / mirrored.area_mm2 - 1.0).abs() < 2e-6);
            assert!((p.shear_rigidity_n / mirrored.shear_rigidity_n - 1.0).abs() < 2e-6);
            if angle == 0.0 {
                assert!((p.area_mm2 - 940000.0).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn circular_column_area_uses_exact_curved_overlap() {
        let shape = SectionShape::RcCircle {
            d: 600.0,
            rebar: rebar(),
        };
        let p = section(&shape, 0.0).properties().unwrap();
        let r = 300.0_f64;
        let h = 75.0_f64;
        let overlap = h * (r * r - h * h).sqrt() + r * r * (h / r).asin();
        let expected = 600000.0 + std::f64::consts::PI * r * r - overlap;
        assert!((p.area_mm2 / expected - 1.0).abs() < 2e-6);
    }

    #[test]
    fn composite_material_widths_preserve_each_modulus() {
        for shape in [
            SectionShape::SrcRect {
                b: 600.0,
                d: 800.0,
                rebar: rebar(),
                steel_height: 500.0,
                steel_width: 300.0,
                steel_web_thick: 12.0,
                steel_flange_thick: 20.0,
            },
            SectionShape::CftBox {
                height: 800.0,
                width: 600.0,
                thick: 20.0,
            },
            SectionShape::CftPipe {
                outer_dia: 600.0,
                thick: 20.0,
            },
        ] {
            let mut wall = section(&shape, 0.37);
            let original = wall.properties().unwrap();
            wall.columns[0].as_mut().unwrap().materials[1].shear *= 2.0;
            let changed = wall.properties().unwrap();
            assert!(changed.shear_rigidity_n > original.shear_rigidity_n);
            assert!((changed.area_mm2 / original.area_mm2 - 1.0).abs() < 2e-6);
            assert!(
                (changed.flexural_rigidity_n_mm2 / original.flexural_rigidity_n_mm2 - 1.0).abs()
                    < 2e-6
            );
        }
    }

    #[test]
    fn closed_tube_void_is_not_filled_with_wall_material() {
        let shape = SectionShape::SteelBox {
            height: 600.0,
            width: 400.0,
            thick: 20.0,
            corner_r: 0.0,
        };
        let wall = section(&shape, 0.0);
        let strip = wall.strip(100.0, 1.0).unwrap();
        assert_eq!(strip.width_mm, 40.0);
        let p = wall.properties().unwrap();
        let expected = 600000.0 - 300.0 * 150.0 + shape.calc_area();
        assert!((p.area_mm2 - expected).abs() < 1e-6);
    }

    #[test]
    fn steel_shapes_have_positive_converged_union_properties() {
        for shape in [
            SectionShape::SteelH {
                height: 600.0,
                width: 300.0,
                web_thick: 12.0,
                flange_thick: 20.0,
            },
            SectionShape::SteelBox {
                height: 600.0,
                width: 300.0,
                thick: 12.0,
                corner_r: 24.0,
            },
            SectionShape::SteelAngle {
                leg_a: 200.0,
                leg_b: 150.0,
                thick: 12.0,
            },
            SectionShape::SteelChannel {
                height: 300.0,
                width: 100.0,
                web_thick: 10.0,
                flange_thick: 15.0,
            },
            SectionShape::SteelTee {
                height: 300.0,
                width: 200.0,
                web_thick: 10.0,
                flange_thick: 15.0,
            },
            SectionShape::SteelPipe {
                outer_dia: 400.0,
                thick: 12.0,
            },
            SectionShape::SteelFlatBar {
                width: 300.0,
                thick: 15.0,
            },
            SectionShape::SteelRoundBar { dia: 100.0 },
            SectionShape::SteelBuiltH {
                height: 500.0,
                upper_width: 300.0,
                upper_thick: 20.0,
                lower_width: 200.0,
                lower_thick: 15.0,
                web_thick: 10.0,
            },
            SectionShape::SteelLipChannel {
                height: 300.0,
                width: 100.0,
                lip: 30.0,
                thick: 5.0,
            },
        ] {
            assert!(
                section(&shape, 0.37).properties().unwrap().shear_rigidity_n > 0.0,
                "{shape:?}"
            );
        }
    }
}
