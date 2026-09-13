use squid_n_core::section_shape::SectionShape;

#[derive(Clone, Copy, Debug)]
pub(super) enum Primitive {
    Rectangle { y: [f64; 2], z: [f64; 2] },
    Circle { center: [f64; 2], radius: f64 },
}

impl Primitive {
    pub fn interval(self, x: f64, direction: [f64; 2]) -> Option<[f64; 2]> {
        let [a, b] = direction;
        match self {
            Self::Circle {
                center: [y, z],
                radius,
            } => {
                let dx = x - a * y - b * z;
                let square = radius * radius - dx * dx;
                if square <= 0.0 {
                    return None;
                }
                let center = -b * y + a * z;
                let half = square.sqrt();
                Some([center - half, center + half])
            }
            Self::Rectangle { y, z } => {
                let mut span = [f64::NEG_INFINITY, f64::INFINITY];
                for (bounds, offset, slope) in [(y, a * x, -b), (z, b * x, a)] {
                    if slope.abs() < 1e-14 {
                        if offset < bounds[0] || offset > bounds[1] {
                            return None;
                        }
                    } else {
                        let lo = (bounds[0] - offset) / slope;
                        let hi = (bounds[1] - offset) / slope;
                        span[0] = span[0].max(lo.min(hi));
                        span[1] = span[1].min(lo.max(hi));
                    }
                }
                (span[1] > span[0]).then_some(span)
            }
        }
    }

    pub fn breaks(self, direction: [f64; 2]) -> Vec<f64> {
        let [a, b] = direction;
        match self {
            Self::Rectangle { y, z } => y
                .into_iter()
                .flat_map(|y| z.map(|z| a * y + b * z))
                .collect(),
            Self::Circle {
                center: [y, z],
                radius,
            } => vec![a * y + b * z - radius, a * y + b * z + radius],
        }
    }
}

/// 部材局所y・z座標の材料領域。後の領域が優先し、Noneは空洞を表す。
pub(super) struct SectionGeometry {
    pub parts: Vec<(Primitive, Option<usize>)>,
}

impl SectionGeometry {
    pub fn of(shape: &SectionShape) -> Result<Self, String> {
        let mut parts = Vec::new();
        let mut rect = |y: [f64; 2], z: [f64; 2], material| {
            parts.push((Primitive::Rectangle { y, z }, material));
        };
        match *shape {
            SectionShape::RcRect { b, d, .. } | SectionShape::SrcRect { b, d, .. } => {
                rect([-d / 2.0, d / 2.0], [-b / 2.0, b / 2.0], Some(0));
                if let SectionShape::SrcRect {
                    steel_height: h,
                    steel_width: w,
                    steel_web_thick: tw,
                    steel_flange_thick: tf,
                    ..
                } = *shape
                {
                    if h > d || w > b || h <= 2.0 * tf || w < tw {
                        return Err("SRC内蔵鉄骨の寸法が不正です".into());
                    }
                    rect([-h / 2.0, -h / 2.0 + tf], [-w / 2.0, w / 2.0], Some(1));
                    rect([h / 2.0 - tf, h / 2.0], [-w / 2.0, w / 2.0], Some(1));
                    rect(
                        [-h / 2.0 + tf, h / 2.0 - tf],
                        [-tw / 2.0, tw / 2.0],
                        Some(1),
                    );
                }
            }
            SectionShape::CftBox {
                height: h,
                width: w,
                thick: t,
            } => {
                if t <= 0.0 || 2.0 * t >= h.min(w) {
                    return Err("CFT鋼管厚が不正です".into());
                }
                rect([-h / 2.0, h / 2.0], [-w / 2.0, w / 2.0], Some(0));
                rect(
                    [-h / 2.0 + t, h / 2.0 - t],
                    [-w / 2.0 + t, w / 2.0 - t],
                    Some(1),
                );
            }
            SectionShape::SteelH {
                height: h,
                width: w,
                web_thick: tw,
                flange_thick: tf,
            } => {
                rect([-h / 2.0, -h / 2.0 + tf], [-w / 2.0, w / 2.0], Some(0));
                rect([h / 2.0 - tf, h / 2.0], [-w / 2.0, w / 2.0], Some(0));
                rect(
                    [-h / 2.0 + tf, h / 2.0 - tf],
                    [-tw / 2.0, tw / 2.0],
                    Some(0),
                );
            }
            SectionShape::SteelFlatBar { width: w, thick: h } => {
                rect([-h / 2.0, h / 2.0], [-w / 2.0, w / 2.0], Some(0))
            }
            SectionShape::SteelBuiltH {
                height: h,
                upper_width: uw,
                upper_thick: ut,
                lower_width: lw,
                lower_thick: lt,
                web_thick: tw,
            } => {
                rect([0.0, lt], [-lw / 2.0, lw / 2.0], Some(0));
                rect([h - ut, h], [-uw / 2.0, uw / 2.0], Some(0));
                rect([lt, h - ut], [-tw / 2.0, tw / 2.0], Some(0));
                center_rectangles(&mut parts);
            }
            SectionShape::SteelAngle {
                leg_a: a,
                leg_b: b,
                thick: t,
            } => {
                rect([0.0, a], [0.0, t], Some(0));
                rect([0.0, t], [t, b], Some(0));
                center_rectangles(&mut parts);
            }
            SectionShape::SteelTee {
                height: h,
                width: w,
                web_thick: tw,
                flange_thick: tf,
            } => {
                rect([0.0, h - tf], [-tw / 2.0, tw / 2.0], Some(0));
                rect([h - tf, h], [-w / 2.0, w / 2.0], Some(0));
                center_rectangles(&mut parts);
            }
            SectionShape::SteelChannel {
                height: h,
                width: w,
                web_thick: tw,
                flange_thick: tf,
            } => {
                rect([0.0, tf], [0.0, w], Some(0));
                rect([h - tf, h], [0.0, w], Some(0));
                rect([tf, h - tf], [0.0, tw], Some(0));
                center_rectangles(&mut parts);
            }
            SectionShape::SteelLipChannel {
                height: h,
                width: w,
                lip: l,
                thick: t,
            } => {
                if l > h / 2.0 {
                    return Err("リップが重なる断面です".into());
                }
                rect([0.0, h], [0.0, t], Some(0));
                rect([0.0, t], [t, w], Some(0));
                rect([h - t, h], [t, w], Some(0));
                rect([t, l], [w - t, w], Some(0));
                rect([h - l, h - t], [w - t, w], Some(0));
                center_rectangles(&mut parts);
            }
            SectionShape::RcCircle { d, .. } | SectionShape::SteelRoundBar { dia: d } => {
                parts.push((
                    Primitive::Circle {
                        center: [0.0; 2],
                        radius: d / 2.0,
                    },
                    Some(0),
                ));
            }
            SectionShape::SteelPipe {
                outer_dia: d,
                thick: t,
            }
            | SectionShape::CftPipe {
                outer_dia: d,
                thick: t,
            } => {
                if t <= 0.0 {
                    return Err("鋼管厚が不正です".into());
                }
                parts.push((
                    Primitive::Circle {
                        center: [0.0; 2],
                        radius: d / 2.0,
                    },
                    Some(0),
                ));
                parts.push((
                    Primitive::Circle {
                        center: [0.0; 2],
                        radius: d / 2.0 - t,
                    },
                    if matches!(shape, SectionShape::CftPipe { .. }) {
                        Some(1)
                    } else {
                        None
                    },
                ));
            }
            SectionShape::SteelBox {
                height: h,
                width: w,
                thick: t,
                corner_r: r,
            } => {
                if t <= 0.0 || r < 0.0 || r > h.min(w) / 2.0 {
                    return Err("角形鋼管の寸法が不正です".into());
                }
                rounded_rectangle(&mut parts, h, w, r, Some(0));
                rounded_rectangle(&mut parts, h - 2.0 * t, w - 2.0 * t, (r - t).max(0.0), None);
            }
            SectionShape::RcWall { .. } | SectionShape::RcSlab { .. } => {
                return Err("側柱に壁・床の断面は指定できません".into())
            }
        }
        if parts.iter().any(|(p, _)| match p {
            Primitive::Rectangle { y, z } => {
                !y.iter().chain(z).all(|v| v.is_finite()) || y[1] <= y[0] || z[1] <= z[0]
            }
            Primitive::Circle { center, radius } => {
                !center.iter().all(|v| v.is_finite()) || !radius.is_finite() || *radius <= 0.0
            }
        }) {
            return Err("側柱の断面寸法が不正です".into());
        }
        Ok(Self { parts })
    }
}

fn center_rectangles(parts: &mut [(Primitive, Option<usize>)]) {
    let mut area = 0.0;
    let mut moment = [0.0; 2];
    for (p, _) in parts.iter() {
        if let Primitive::Rectangle { y, z } = p {
            let a = (y[1] - y[0]) * (z[1] - z[0]);
            area += a;
            moment[0] += a * (y[0] + y[1]) / 2.0;
            moment[1] += a * (z[0] + z[1]) / 2.0;
        }
    }
    for (p, _) in parts {
        if let Primitive::Rectangle { y, z } = p {
            for v in y {
                *v -= moment[0] / area;
            }
            for v in z {
                *v -= moment[1] / area;
            }
        }
    }
}

fn rounded_rectangle(
    parts: &mut Vec<(Primitive, Option<usize>)>,
    h: f64,
    w: f64,
    r: f64,
    material: Option<usize>,
) {
    if r == 0.0 {
        parts.push((
            Primitive::Rectangle {
                y: [-h / 2.0, h / 2.0],
                z: [-w / 2.0, w / 2.0],
            },
            material,
        ));
        return;
    }
    if h > 2.0 * r {
        parts.push((
            Primitive::Rectangle {
                y: [-h / 2.0 + r, h / 2.0 - r],
                z: [-w / 2.0, w / 2.0],
            },
            material,
        ));
    }
    if w > 2.0 * r {
        parts.push((
            Primitive::Rectangle {
                y: [-h / 2.0, h / 2.0],
                z: [-w / 2.0 + r, w / 2.0 - r],
            },
            material,
        ));
    }
    for y in [-h / 2.0 + r, h / 2.0 - r] {
        for z in [-w / 2.0 + r, w / 2.0 - r] {
            parts.push((
                Primitive::Circle {
                    center: [y, z],
                    radius: r,
                },
                material,
            ));
        }
    }
}
