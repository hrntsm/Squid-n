//! 断面の分布質量特性。

use super::{ElementData, Material, Model, Section};
use crate::section_shape::{RcRebar, SectionShape};
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
            mass_per_length: density * area.max(0.0),
            rotary_inertia_y_per_length: density * iy.max(0.0),
            rotary_inertia_z_per_length: density * iz.max(0.0),
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
        Self::try_from_section(section, main, rebar, shear_rebar, steel)
            .unwrap_or_else(|error| panic!("断面質量特性を解決できません: {error}"))
    }

    pub fn try_from_section(
        section: &Section,
        main: Option<&Material>,
        rebar: Option<&Material>,
        shear_rebar: Option<&Material>,
        steel: Option<&Material>,
    ) -> Result<Self, String> {
        validate_section_materials(section, main, rebar, shear_rebar, steel)?;

        validate_section_geometry(section)?;

        let Some(shape) = section.shape.as_ref() else {
            let properties = Self::uniform(
                main.map_or(0.0, |m| m.density),
                section.area,
                section.iy,
                section.iz,
            );
            return if valid_mass_properties(properties) {
                Ok(properties)
            } else {
                Err(format!(
                    "断面{}の質量特性が有限な非負値ではありません",
                    section.name
                ))
            };
        };

        let properties = shaped_mass(shape, main, rebar, shear_rebar, steel);
        if valid_mass_properties(properties) {
            Ok(properties)
        } else {
            Err(format!(
                "断面{}の質量特性が有限な非負値ではありません",
                section.name
            ))
        }
    }
}

fn valid_mass_properties(properties: SectionMassProperties) -> bool {
    [
        properties.mass_per_length,
        properties.rotary_inertia_y_per_length,
        properties.rotary_inertia_z_per_length,
    ]
    .into_iter()
    .all(|value| value.is_finite() && value >= 0.0)
}

pub fn validate_section_materials(
    section: &Section,
    main: Option<&Material>,
    rebar: Option<&Material>,
    shear_rebar: Option<&Material>,
    steel: Option<&Material>,
) -> Result<(), String> {
    for (role, material) in [
        ("主材料", main),
        ("主筋材料", rebar),
        ("せん断補強筋材料", shear_rebar),
        ("内蔵鉄骨材料", steel),
    ] {
        if let Some(material) = material {
            if !material.density.is_finite() || material.density < 0.0 {
                return Err(format!(
                    "断面{}の{role}密度が有限な非負値ではありません",
                    section.name
                ));
            }
        }
    }

    let Some(shape) = section.shape.as_ref() else {
        if main.is_none()
            && [section.area, section.iy, section.iz]
                .into_iter()
                .any(|value| value.is_finite() && value > 0.0)
        {
            return Err(format!(
                "形状なし断面{}は正の断面諸元があるため主材料が必要です",
                section.name
            ));
        }
        return Ok(());
    };

    let concrete_shape = matches!(
        shape,
        SectionShape::RcRect { .. }
            | SectionShape::RcCircle { .. }
            | SectionShape::SrcRect { .. }
            | SectionShape::RcWall { .. }
            | SectionShape::RcSlab { .. }
    );
    if concrete_shape && main.is_none() {
        return Err(format!("RC/SRC断面{}の主材料が未設定です", section.name));
    }
    if concrete_shape
        && main.is_some_and(|material| material.category != super::MaterialCategory::Concrete)
    {
        return Err(format!(
            "RC/SRC断面{}の主材料がコンクリートではありません",
            section.name
        ));
    }
    if concrete_shape
        && main
            .and_then(|material| material.fc)
            .is_none_or(|fc| !fc.is_finite() || fc <= 0.0)
    {
        return Err(format!(
            "RC/SRC断面{}のコンクリート材料のFcが未設定または不正です",
            section.name
        ));
    }

    let steel_shape = matches!(
        shape,
        SectionShape::SteelH { .. }
            | SectionShape::SteelBox { .. }
            | SectionShape::SteelAngle { .. }
            | SectionShape::SteelChannel { .. }
            | SectionShape::SteelTee { .. }
            | SectionShape::SteelPipe { .. }
            | SectionShape::SteelFlatBar { .. }
            | SectionShape::SteelRoundBar { .. }
            | SectionShape::SteelBuiltH { .. }
            | SectionShape::SteelLipChannel { .. }
    );
    if steel_shape {
        let Some(main) = main else {
            return Err(format!("鋼材断面{}の主材料が未設定です", section.name));
        };
        if main.category != super::MaterialCategory::Steel {
            return Err(format!(
                "鋼材断面{}の主材料が鋼材ではありません",
                section.name
            ));
        }
    }

    if matches!(shape, SectionShape::SrcRect { .. }) && steel.is_none() {
        return Err(format!("SRC断面{}の内蔵鉄骨材料が未設定です", section.name));
    }
    if matches!(
        shape,
        SectionShape::RcRect { .. }
            | SectionShape::RcCircle { .. }
            | SectionShape::SrcRect { .. }
            | SectionShape::RcWall { .. }
    ) {
        for (role, material) in [("主筋材料", rebar), ("せん断補強筋材料", shear_rebar)]
        {
            if material.is_some_and(|material| material.category != super::MaterialCategory::Rebar)
            {
                return Err(format!("断面{}の{role}が鉄筋ではありません", section.name));
            }
        }
    }
    if matches!(
        shape,
        SectionShape::RcWall { .. } | SectionShape::RcSlab { .. }
    ) && shear_rebar.is_some()
    {
        return Err(format!(
            "断面{}の壁・床版はせん断補強筋材料を受け付けません",
            section.name
        ));
    }
    if matches!(shape, SectionShape::RcSlab { .. }) && rebar.is_some() {
        return Err(format!(
            "断面{}の床版は主筋材料を受け付けません",
            section.name
        ));
    }
    if matches!(shape, SectionShape::SrcRect { .. })
        && steel.is_some_and(|material| material.category != super::MaterialCategory::Steel)
    {
        return Err(format!(
            "SRC断面{}の内蔵鉄骨材料が鋼材ではありません",
            section.name
        ));
    }
    if matches!(
        shape,
        SectionShape::CftBox { .. } | SectionShape::CftPipe { .. }
    ) {
        let Some(main) = main else {
            return Err(format!("CFT断面{}の鋼管主材料が未設定です", section.name));
        };
        if main.category != super::MaterialCategory::Steel {
            return Err(format!(
                "CFT断面{}の鋼管主材料が鋼材ではありません",
                section.name
            ));
        }
        if main.fc.is_none_or(|fc| !fc.is_finite() || fc <= 0.0) {
            return Err(format!(
                "CFT断面{}の充填コンクリートFcが未設定または不正です",
                section.name
            ));
        }
    }

    Ok(())
}

fn validate_section_geometry(section: &Section) -> Result<(), String> {
    let Some(shape) = section.shape.as_ref() else {
        for (name, value) in [
            ("断面積", section.area),
            ("Iy", section.iy),
            ("Iz", section.iz),
        ] {
            if !value.is_finite() || value < 0.0 {
                return Err(format!(
                    "形状なし断面{}の{name}が有限な非負値ではありません",
                    section.name
                ));
            }
        }
        return Ok(());
    };

    let positive = |name: &str, value: f64| {
        if value.is_finite() && value > 0.0 {
            Ok(())
        } else {
            Err(format!(
                "断面{}の形状寸法{name}が有限な正値ではありません",
                section.name
            ))
        }
    };
    let nonnegative = |name: &str, value: f64| {
        if value.is_finite() && value >= 0.0 {
            Ok(())
        } else {
            Err(format!(
                "断面{}の形状寸法{name}が有限な非負値ではありません",
                section.name
            ))
        }
    };
    let relation = |description: &str, condition: bool| {
        if condition {
            Ok(())
        } else {
            Err(format!(
                "断面{}の形状寸法の関係が成立しません: {description}",
                section.name
            ))
        }
    };
    let mut dimensions = Vec::new();
    match shape {
        SectionShape::SteelH {
            height,
            width,
            web_thick,
            flange_thick,
        }
        | SectionShape::SteelChannel {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            dimensions.extend([
                ("height", *height),
                ("width", *width),
                ("web_thick", *web_thick),
                ("flange_thick", *flange_thick),
            ]);
            relation("height > 2 * flange_thick", *height > 2.0 * *flange_thick)?;
            relation("width >= web_thick", *width >= *web_thick)?;
        }
        SectionShape::SteelTee {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            dimensions.extend([
                ("height", *height),
                ("width", *width),
                ("web_thick", *web_thick),
                ("flange_thick", *flange_thick),
            ]);
            relation("height > flange_thick", *height > *flange_thick)?;
            relation("width >= web_thick", *width >= *web_thick)?;
        }
        SectionShape::SteelBox {
            height,
            width,
            thick,
            corner_r,
        } => {
            dimensions.extend([("height", *height), ("width", *width), ("thick", *thick)]);
            nonnegative("corner_r", *corner_r)?;
            relation("height > 2 * thick", *height > 2.0 * *thick)?;
            relation("width > 2 * thick", *width > 2.0 * *thick)?;
        }
        SectionShape::CftBox {
            height,
            width,
            thick,
        } => {
            dimensions.extend([("height", *height), ("width", *width), ("thick", *thick)]);
            relation("height > 2 * thick", *height > 2.0 * *thick)?;
            relation("width > 2 * thick", *width > 2.0 * *thick)?;
        }
        SectionShape::SteelAngle {
            leg_a,
            leg_b,
            thick,
        } => {
            dimensions.extend([("leg_a", *leg_a), ("leg_b", *leg_b), ("thick", *thick)]);
            relation("leg_a >= thick", *leg_a >= *thick)?;
            relation("leg_b >= thick", *leg_b >= *thick)?;
        }
        SectionShape::SteelPipe { outer_dia, thick }
        | SectionShape::CftPipe { outer_dia, thick } => {
            dimensions.extend([("outer_dia", *outer_dia), ("thick", *thick)]);
            relation("outer_dia > 2 * thick", *outer_dia > 2.0 * *thick)?;
        }
        SectionShape::SteelFlatBar { width, thick } => {
            dimensions.extend([("width", *width), ("thick", *thick)])
        }
        SectionShape::SteelRoundBar { dia } => dimensions.push(("dia", *dia)),
        SectionShape::SteelBuiltH {
            height,
            upper_width,
            upper_thick,
            lower_width,
            lower_thick,
            web_thick,
        } => {
            dimensions.extend([
                ("height", *height),
                ("upper_width", *upper_width),
                ("upper_thick", *upper_thick),
                ("lower_width", *lower_width),
                ("lower_thick", *lower_thick),
                ("web_thick", *web_thick),
            ]);
            relation(
                "height > upper_thick + lower_thick",
                *height > *upper_thick + *lower_thick,
            )?;
            relation("upper_width >= web_thick", *upper_width >= *web_thick)?;
            relation("lower_width >= web_thick", *lower_width >= *web_thick)?;
        }
        SectionShape::SteelLipChannel {
            height,
            width,
            lip,
            thick,
        } => {
            dimensions.extend([
                ("height", *height),
                ("width", *width),
                ("lip", *lip),
                ("thick", *thick),
            ]);
            relation("height > 2 * thick", *height > 2.0 * *thick)?;
            relation("width > thick", *width > *thick)?;
            relation("lip > thick", *lip > *thick)?;
            relation("height > lip + thick", *height > *lip + *thick)?;
        }
        SectionShape::RcRect { b, d, rebar } | SectionShape::SrcRect { b, d, rebar, .. } => {
            dimensions.extend([("b", *b), ("d", *d)]);
            validate_rebar_geometry(rebar, &positive, &nonnegative)?;
            validate_rectangular_rebar_fit(rebar, *b, *d)?;
            if let SectionShape::SrcRect {
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
                ..
            } = shape
            {
                dimensions.extend([
                    ("steel_height", *steel_height),
                    ("steel_width", *steel_width),
                    ("steel_web_thick", *steel_web_thick),
                    ("steel_flange_thick", *steel_flange_thick),
                ]);
                relation("steel_height <= d", *steel_height <= *d)?;
                relation("steel_width <= b", *steel_width <= *b)?;
            }
        }
        SectionShape::RcCircle { d, rebar } => {
            dimensions.push(("d", *d));
            validate_rebar_geometry(rebar, &positive, &nonnegative)?;
            validate_circular_rebar_fit(rebar, *d)?;
        }
        SectionShape::RcWall { thickness, ps } => {
            dimensions.push(("thickness", *thickness));
            nonnegative("ps", *ps)?;
            relation("ps <= 1", *ps <= 1.0)?;
        }
        SectionShape::RcSlab { thickness } => dimensions.push(("thickness", *thickness)),
    }
    for (name, value) in dimensions {
        positive(name, value)?;
    }
    Ok(())
}

fn validate_rebar_geometry(
    rebar: &RcRebar,
    positive: &impl Fn(&str, f64) -> Result<(), String>,
    nonnegative: &impl Fn(&str, f64) -> Result<(), String>,
) -> Result<(), String> {
    nonnegative("cover", rebar.cover)?;
    for (name, set) in [("main_x", &rebar.main_x), ("main_y", &rebar.main_y)] {
        if set.count > 0 {
            positive(&format!("{name}.dia"), set.dia)?;
        } else {
            nonnegative(&format!("{name}.dia"), set.dia)?;
        }
    }
    if rebar.shear.legs > 0 {
        positive("shear.dia", rebar.shear.dia)?;
        positive("shear.pitch", rebar.shear.pitch)?;
    } else {
        nonnegative("shear.dia", rebar.shear.dia)?;
        nonnegative("shear.pitch", rebar.shear.pitch)?;
    }
    Ok(())
}

fn validate_rectangular_rebar_fit(rebar: &RcRebar, width: f64, depth: f64) -> Result<(), String> {
    use crate::rc_rebar_geom::rebar_layer_depth_from_edge;

    let check = |description: &str, condition: bool| {
        if condition {
            Ok(())
        } else {
            Err(format!(
                "矩形断面の配筋が母断面内に収まりません: {description}"
            ))
        }
    };
    let check_set_x = |set: &crate::section_shape::BarSet| -> Result<(), String> {
        if set.count == 0 {
            return Ok(());
        }
        let span = width - 2.0 * rebar.cover;
        check("主筋の幅方向の中心または径", span >= 0.0)?;
        for layer in 0..set.layers.max(1) {
            let depth_from_edge =
                rebar_layer_depth_from_edge(rebar.cover, rebar.shear.dia, set, layer);
            check(
                "主筋のせい方向の中心または径",
                depth_from_edge + set.dia / 2.0 <= depth / 2.0,
            )?;
        }
        for i in 0..set.count {
            let y = if set.count == 1 {
                0.0
            } else {
                -span / 2.0 + span * i as f64 / (set.count - 1) as f64
            };
            check(
                "主筋の幅方向の中心または径",
                y.abs() + set.dia / 2.0 <= width / 2.0,
            )?;
        }
        Ok(())
    };
    let check_set_y = |set: &crate::section_shape::BarSet| -> Result<(), String> {
        if set.count == 0 {
            return Ok(());
        }
        let span = depth - 2.0 * rebar.cover;
        check("主筋のせい方向の中心または径", span >= 0.0)?;
        for layer in 0..set.layers.max(1) {
            let depth_from_edge =
                rebar_layer_depth_from_edge(rebar.cover, rebar.shear.dia, set, layer);
            check(
                "主筋の幅方向の中心または径",
                depth_from_edge + set.dia / 2.0 <= width / 2.0,
            )?;
        }
        for i in 0..set.count {
            let z = -span / 2.0 + span * (i as f64 + 1.0) / (set.count + 1) as f64;
            check(
                "主筋のせい方向の中心または径",
                z.abs() + set.dia / 2.0 <= depth / 2.0,
            )?;
        }
        Ok(())
    };
    check_set_x(&rebar.main_x)?;
    check_set_y(&rebar.main_y)?;
    if rebar.shear.legs > 0 {
        check(
            "せん断補強筋の幅方向の中心または径",
            2.0 * rebar.cover + rebar.shear.dia <= width,
        )?;
        check(
            "せん断補強筋のせい方向の中心または径",
            2.0 * rebar.cover + rebar.shear.dia <= depth,
        )?;
    }
    Ok(())
}

fn validate_circular_rebar_fit(rebar: &RcRebar, dia: f64) -> Result<(), String> {
    use crate::rc_rebar_geom::rebar_layer_depth_from_edge;

    let radius = dia / 2.0;
    for (name, set) in [("main_x", &rebar.main_x), ("main_y", &rebar.main_y)] {
        if set.count == 0 {
            continue;
        }
        for layer in 0..set.layers.max(1) {
            let depth_from_edge =
                rebar_layer_depth_from_edge(rebar.cover, rebar.shear.dia, set, layer);
            if depth_from_edge + set.dia / 2.0 > radius {
                return Err(format!(
                    "円形断面の配筋が母断面内に収まりません: {name} の中心または径"
                ));
            }
        }
    }
    if rebar.shear.legs > 0 && rebar.cover + rebar.shear.dia / 2.0 > radius {
        return Err("円形断面の配筋が母断面内に収まりません: せん断補強筋の中心または径".into());
    }
    Ok(())
}

impl Model {
    /// 要素断面の材料領域から分布質量特性を求める。
    pub fn element_mass_properties(
        &self,
        elem: &ElementData,
    ) -> Result<SectionMassProperties, String> {
        let Some(section) = self.element_section(elem) else {
            return Ok(SectionMassProperties::default());
        };
        SectionMassProperties::try_from_section(
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

#[derive(Default)]
struct WeightedMass {
    mass_per_length: f64,
    rotary_inertia_y_per_length: f64,
    rotary_inertia_z_per_length: f64,
}

impl WeightedMass {
    fn add(&mut self, density: f64, geometry: GeometryMass) {
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
    _rebar: Option<&Material>,
    _shear_rebar: Option<&Material>,
    _steel: Option<&Material>,
) -> SectionMassProperties {
    let main_density = main.map_or(0.0, |m| m.density);
    let rc_density = concrete_density(main, ConcreteComposition::Rc);
    let mut result = WeightedMass::default();

    match shape {
        SectionShape::RcRect { b, d, .. } => {
            let gross = rectangle_geometry(*b, *d);
            result.add(rc_density, gross);
        }
        SectionShape::RcCircle { d, .. } => {
            let gross = circle_geometry(*d);
            result.add(rc_density, gross);
        }
        SectionShape::SrcRect { b, d, .. } => {
            let gross = rectangle_geometry(*b, *d);
            result.add(concrete_density(main, ConcreteComposition::Src), gross);
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
        SectionShape::RcWall { thickness, .. } => {
            let gross = rectangle_geometry(1000.0, *thickness);
            result.add(rc_density, gross);
        }
        SectionShape::RcSlab { .. } => {
            result.add(
                concrete_density(main, ConcreteComposition::Plain),
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

fn concrete_density(material: Option<&Material>, composition: ConcreteComposition) -> f64 {
    material.map_or(0.0, |m| {
        m.fc.filter(|fc| *fc > 0.0).map_or(m.density, |fc| {
            mass_density_from_unit_weight_kn_m3(concrete_unit_weight_kn_m3(
                fc,
                m.concrete_class,
                composition,
            ))
        })
    })
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
    fn 形状なしで正の断面諸元があり主材料がなければエラーになる() {
        let section = Section {
            area: 100.0,
            ..Section::zero(SectionId(0), "unassigned".into())
        };
        assert!(
            SectionMassProperties::try_from_section(&section, None, None, None, None)
                .expect_err("正の断面諸元には主材料が必要")
                .contains("主材料")
        );
    }

    #[test]
    fn 形状なしでゼロ断面かつ主材料なしは意図的な無質量として許可する() {
        let section = Section::zero(SectionId(0), "massless".into());
        assert_eq!(
            SectionMassProperties::try_from_section(&section, None, None, None, None).unwrap(),
            SectionMassProperties::default()
        );
    }

    #[test]
    fn rcは鉄筋材料によらず標準rc密度を総断面へ適用する() {
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
        let expected_density = concrete_density(Some(&concrete), ConcreteComposition::Rc);
        assert_eq!(
            properties,
            SectionMassProperties::uniform(expected_density, gross.area, gross.iy, gross.iz,)
        );
        let changed_rebar = material(1, MaterialCategory::Rebar, 100.0, None);
        let changed = SectionMassProperties::from_section(
            &section,
            Some(&concrete),
            Some(&changed_rebar),
            Some(&changed_rebar),
            None,
        );
        assert_eq!(properties, changed);
    }

    #[test]
    fn rc密度はfcとコンクリート種類で変わる() {
        let shape = SectionShape::RcRect {
            b: 400.0,
            d: 400.0,
            rebar: crate::section_shape::RcRebar {
                main_x: crate::section_shape::BarSet {
                    count: 0,
                    dia: 0.0,
                    layers: 1,
                },
                main_y: crate::section_shape::BarSet {
                    count: 0,
                    dia: 0.0,
                    layers: 1,
                },
                cover: 40.0,
                shear: crate::section_shape::ShearBar {
                    dia: 0.0,
                    pitch: 0.0,
                    legs: 0,
                },
            },
        };
        let section = shape.to_section(SectionId(0), "RC".into());
        let normal = material(0, MaterialCategory::Concrete, 1.0, Some(24.0));
        let high_strength = material(1, MaterialCategory::Concrete, 1.0, Some(42.0));
        let normal_mass =
            SectionMassProperties::from_section(&section, Some(&normal), None, None, None);
        let high_strength_mass =
            SectionMassProperties::from_section(&section, Some(&high_strength), None, None, None);
        assert!(high_strength_mass.mass_per_length > normal_mass.mass_per_length);
    }

    #[test]
    fn rc壁は標準rc密度を総断面へ適用する() {
        let shape = SectionShape::RcWall {
            thickness: 200.0,
            ps: 0.0025,
        };
        let section = shape.to_section(SectionId(0), "W".into());
        let concrete = material(0, MaterialCategory::Concrete, 2.4e-9, Some(24.0));
        let properties =
            SectionMassProperties::try_from_section(&section, Some(&concrete), None, None, None)
                .unwrap();
        let gross = rectangle_geometry(1000.0, 200.0);
        let rho_rc = concrete_density(Some(&concrete), ConcreteComposition::Rc);
        assert_eq!(
            properties,
            SectionMassProperties::uniform(rho_rc, gross.area, gross.iy, gross.iz,)
        );
    }

    #[test]
    fn rc壁と床版は未定義のせん断補強筋質量を拒否する() {
        let rebar = material(1, MaterialCategory::Rebar, 7.85e-9, None);
        for shape in [
            SectionShape::RcWall {
                thickness: 200.0,
                ps: 0.0025,
            },
            SectionShape::RcSlab { thickness: 150.0 },
        ] {
            let section = shape.to_section(SectionId(0), "plate".into());
            let concrete = material(0, MaterialCategory::Concrete, 2.4e-9, Some(24.0));
            let error = SectionMassProperties::try_from_section(
                &section,
                Some(&concrete),
                Some(&rebar),
                Some(&rebar),
                None,
            )
            .expect_err("形状で定義されないせん断補強筋材料を受け付けない");
            assert!(error.contains("せん断補強筋"));
        }
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
        let rebar = material(2, MaterialCategory::Rebar, 8.0, None);
        let properties = SectionMassProperties::from_section(
            &section,
            Some(&concrete),
            Some(&rebar),
            Some(&rebar),
            Some(&steel),
        );
        let expected = concrete_density(Some(&concrete), ConcreteComposition::Src) * 600.0 * 600.0;
        assert!((properties.mass_per_length - expected).abs() < 1e-9);
        let heavier_steel = material(1, MaterialCategory::Steel, 100.0, None);
        let changed = SectionMassProperties::from_section(
            &section,
            Some(&concrete),
            Some(&rebar),
            Some(&rebar),
            Some(&heavier_steel),
        );
        assert_eq!(properties, changed);
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
    fn cftはfc未設定なら質量特性を解決できない() {
        let shape = SectionShape::CftBox {
            height: 400.0,
            width: 400.0,
            thick: 12.0,
        };
        let section = shape.to_section(SectionId(0), "CFT".into());
        let steel = material(0, MaterialCategory::Steel, 8.0, None);
        let error =
            SectionMassProperties::try_from_section(&section, Some(&steel), None, None, None)
                .expect_err("Fc未設定のCFTは質量を計算してはならない");
        assert!(error.contains("CFT"));
    }

    #[test]
    fn cftは鋼管主材料の密度と充填コンクリートの密度を分離する() {
        let shape = SectionShape::CftBox {
            height: 400.0,
            width: 400.0,
            thick: 12.0,
        };
        let section = shape.to_section(SectionId(0), "CFT".into());
        let steel = material(0, MaterialCategory::Steel, 8.0, Some(24.0));
        let properties =
            SectionMassProperties::try_from_section(&section, Some(&steel), None, None, None)
                .unwrap();
        let core = shape.cft_core_props().unwrap();
        let expected =
            steel.density * shape.calc_area() + cft_core_density(Some(&steel)) * core.area;
        assert!((properties.mass_per_length - expected).abs() < 1e-9);
    }

    #[test]
    fn cftの主材料がコンクリートならエラーになる() {
        let shape = SectionShape::CftBox {
            height: 400.0,
            width: 400.0,
            thick: 12.0,
        };
        let section = shape.to_section(SectionId(0), "CFT".into());
        let concrete = material(0, MaterialCategory::Concrete, 2.4e-9, Some(24.0));
        let error =
            SectionMassProperties::try_from_section(&section, Some(&concrete), None, None, None)
                .expect_err("CFTの主材料にコンクリートを使ってはならない");
        assert!(error.contains("鋼材"));
    }

    #[test]
    fn cftはfcの非有限値を受け付けない() {
        let shape = SectionShape::CftBox {
            height: 400.0,
            width: 400.0,
            thick: 12.0,
        };
        let section = shape.to_section(SectionId(0), "CFT".into());
        for fc in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let steel = material(0, MaterialCategory::Steel, 8.0, Some(fc));
            assert!(SectionMassProperties::try_from_section(
                &section,
                Some(&steel),
                None,
                None,
                None,
            )
            .is_err());
        }
    }

    #[test]
    fn srcは内蔵鉄骨材料未設定なら質量特性を解決できない() {
        use crate::section_shape::{BarSet, RcRebar, ShearBar};

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
        let concrete = material(0, MaterialCategory::Concrete, 2.4e-9, Some(24.0));
        let error =
            SectionMassProperties::try_from_section(&section, Some(&concrete), None, None, None)
                .expect_err("内蔵鉄骨材料未設定のSRCは質量を計算してはならない");
        assert!(error.contains("SRC"));
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
        let expected = concrete_density(Some(&concrete), ConcreteComposition::Rc) * gross.area;
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
        let main_only =
            SectionMassProperties::from_section(&section, Some(&concrete), Some(&main), None, None);
        assert!(
            (main_only.mass_per_length
                - concrete_density(Some(&concrete), ConcreteComposition::Rc) * gross.area)
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
                - concrete_density(Some(&concrete), ConcreteComposition::Rc) * gross.area)
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
        let error =
            SectionMassProperties::try_from_section(&section, Some(&main), None, None, None)
                .expect_err("主材料をSRCの内蔵鉄骨へ流用してはならない");
        assert!(error.contains("SRC"));
    }

    #[test]
    fn 負の単位長さ質量は総質量を負にしない() {
        let properties = SectionMassProperties {
            mass_per_length: -1.0,
            ..SectionMassProperties::default()
        };
        assert_eq!(properties.total_mass(100.0), 0.0);
    }

    #[test]
    fn standard_fc_material_rc_mass_is_integrated_once() {
        use crate::section_shape::{BarSet, ShearBar};

        let presets = crate::material_grade::material_presets();
        let concrete_preset = presets.iter().find(|p| p.name == "Fc24").unwrap();
        let rebar_preset = presets.iter().find(|p| p.name == "SD345").unwrap();
        let concrete = material(
            0,
            MaterialCategory::Concrete,
            concrete_preset.density,
            concrete_preset.fc,
        );
        let rebar = material(
            1,
            MaterialCategory::Rebar,
            rebar_preset.density,
            rebar_preset.fc,
        );
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
        let properties = SectionMassProperties::from_section(
            &section,
            Some(&concrete),
            Some(&rebar),
            Some(&rebar),
            None,
        );
        let gross = rectangle_geometry(400.0, 400.0);
        let expected = concrete_density(Some(&concrete), ConcreteComposition::Rc) * gross.area;
        assert!((properties.mass_per_length - expected).abs() < 1e-12);
    }

    #[test]
    fn 密度が負値または非有限値なら質量特性を解決できない() {
        let section = Section {
            id: SectionId(0),
            name: "invalid-density".into(),
            area: 100.0,
            iy: 200.0,
            iz: 300.0,
            ..Section::zero(SectionId(0), "invalid-density".into())
        };
        for density in [-1.0, f64::NAN, f64::INFINITY] {
            let error = SectionMassProperties::try_from_section(
                &section,
                Some(&material(0, MaterialCategory::Steel, density, None)),
                None,
                None,
                None,
            )
            .expect_err("異常な密度は質量特性へ進めてはならない");
            assert!(error.contains("密度"));
        }
    }

    #[test]
    fn rcはfc未設定のコンクリート材料を受け付けない() {
        use crate::section_shape::{BarSet, RcRebar, ShearBar};

        let shape = SectionShape::RcRect {
            b: 400.0,
            d: 400.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 0,
                    dia: 0.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 0,
                    dia: 0.0,
                    layers: 1,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 0.0,
                    pitch: 0.0,
                    legs: 0,
                },
            },
        };
        let section = shape.to_section(SectionId(0), "RC".into());
        let error = SectionMassProperties::try_from_section(
            &section,
            Some(&material(0, MaterialCategory::Concrete, 2.4e-9, None)),
            None,
            None,
            None,
        )
        .expect_err("RCのコンクリート密度はFcなしで解釈してはならない");
        assert!(error.contains("Fc"));
    }

    #[test]
    fn rc系断面は主材料の欠落や区分不正やfc不正を受け付けない() {
        use crate::section_shape::{BarSet, RcRebar, ShearBar};

        let rebar = RcRebar {
            main_x: BarSet {
                count: 0,
                dia: 0.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 0,
                dia: 0.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 0.0,
                pitch: 0.0,
                legs: 0,
            },
        };
        let shapes = [
            SectionShape::RcRect {
                b: 400.0,
                d: 400.0,
                rebar: rebar.clone(),
            },
            SectionShape::RcCircle {
                d: 400.0,
                rebar: rebar.clone(),
            },
            SectionShape::SrcRect {
                b: 400.0,
                d: 400.0,
                rebar,
                steel_height: 200.0,
                steel_width: 200.0,
                steel_web_thick: 9.0,
                steel_flange_thick: 12.0,
            },
            SectionShape::RcWall {
                thickness: 150.0,
                ps: 0.0025,
            },
            SectionShape::RcSlab { thickness: 150.0 },
        ];
        for (index, shape) in shapes.into_iter().enumerate() {
            let section = shape.to_section(SectionId(index as u32), "RC系".into());
            assert!(
                SectionMassProperties::try_from_section(&section, None, None, None, None).is_err()
            );
            assert!(SectionMassProperties::try_from_section(
                &section,
                Some(&material(0, MaterialCategory::Steel, 2.4e-9, Some(24.0))),
                None,
                None,
                None,
            )
            .is_err());
            assert!(SectionMassProperties::try_from_section(
                &section,
                Some(&material(0, MaterialCategory::Concrete, 2.4e-9, Some(0.0))),
                None,
                None,
                None,
            )
            .is_err());
        }
    }

    #[test]
    fn 質量入口は形状寸法と形状なし断面諸元の異常値を拒否する() {
        use crate::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

        let rebar = RcRebar {
            main_x: BarSet {
                count: 0,
                dia: 0.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 0,
                dia: 0.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 0.0,
                pitch: 0.0,
                legs: 0,
            },
        };
        for (b, d) in [
            (0.0, 400.0),
            (-1.0, 400.0),
            (f64::NAN, 400.0),
            (400.0, f64::INFINITY),
        ] {
            let section = SectionShape::RcRect {
                b,
                d,
                rebar: rebar.clone(),
            }
            .to_section(SectionId(0), "invalid-shape".into());
            assert!(SectionMassProperties::try_from_section(
                &section,
                Some(&material(0, MaterialCategory::Concrete, 2.4e-9, Some(24.0))),
                None,
                None,
                None,
            )
            .is_err());
        }
        for (area, iy, iz) in [
            (-1.0, 0.0, 0.0),
            (f64::NAN, 0.0, 0.0),
            (0.0, f64::INFINITY, 0.0),
        ] {
            let section = Section {
                area,
                iy,
                iz,
                ..Section::zero(SectionId(0), "invalid-properties".into())
            };
            assert!(
                SectionMassProperties::try_from_section(&section, None, None, None, None).is_err()
            );
        }
    }

    #[test]
    fn 矩形_rcの母断面外配筋を拒否する() {
        use crate::section_shape::{BarSet, RcRebar, ShearBar};

        let shape = SectionShape::RcRect {
            b: 100.0,
            d: 100.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 2,
                    dia: 20.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 0,
                    dia: 0.0,
                    layers: 1,
                },
                cover: 45.0,
                shear: ShearBar {
                    dia: 0.0,
                    pitch: 0.0,
                    legs: 0,
                },
            },
        };
        let section = shape.to_section(SectionId(0), "RC矩形".into());
        let concrete = material(0, MaterialCategory::Concrete, 2.4e-9, Some(24.0));
        let error =
            SectionMassProperties::try_from_section(&section, Some(&concrete), None, None, None)
                .expect_err("矩形RCの断面外主筋を拒否する");
        assert!(error.contains("母断面内"));
    }

    #[test]
    fn 円形_rcの母断面外配筋を拒否する() {
        use crate::section_shape::{BarSet, RcRebar, ShearBar};

        let shape = SectionShape::RcCircle {
            d: 100.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 4,
                    dia: 20.0,
                    layers: 2,
                },
                main_y: BarSet {
                    count: 0,
                    dia: 0.0,
                    layers: 1,
                },
                cover: 20.0,
                shear: ShearBar {
                    dia: 0.0,
                    pitch: 0.0,
                    legs: 0,
                },
            },
        };
        let section = shape.to_section(SectionId(0), "RC円形".into());
        let concrete = material(0, MaterialCategory::Concrete, 2.4e-9, Some(24.0));
        let error =
            SectionMassProperties::try_from_section(&section, Some(&concrete), None, None, None)
                .expect_err("円形RCの断面外主筋を拒否する");
        assert!(error.contains("母断面内"));
    }

    #[test]
    fn srcの母断面外内蔵鉄骨を拒否する() {
        use crate::section_shape::{BarSet, RcRebar, ShearBar};

        let shape = SectionShape::SrcRect {
            b: 400.0,
            d: 400.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 0,
                    dia: 0.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 0,
                    dia: 0.0,
                    layers: 1,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 0.0,
                    pitch: 0.0,
                    legs: 0,
                },
            },
            steel_height: 500.0,
            steel_width: 200.0,
            steel_web_thick: 9.0,
            steel_flange_thick: 12.0,
        };
        let section = shape.to_section(SectionId(0), "SRC".into());
        let concrete = material(0, MaterialCategory::Concrete, 2.4e-9, Some(24.0));
        let steel = material(1, MaterialCategory::Steel, 8.0, None);
        let error = SectionMassProperties::try_from_section(
            &section,
            Some(&concrete),
            None,
            None,
            Some(&steel),
        )
        .expect_err("SRCの母断面外鉄骨を拒否する");
        assert!(error.contains("steel_height <= d"));
    }

    #[test]
    fn rcとsrcの補助材料は材料区分を検証する() {
        use crate::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

        let rebar = RcRebar {
            main_x: BarSet {
                count: 1,
                dia: 20.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 0,
                dia: 0.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
            },
        };
        let rc = SectionShape::RcRect {
            b: 400.0,
            d: 400.0,
            rebar: rebar.clone(),
        }
        .to_section(SectionId(0), "RC".into());
        let concrete = material(0, MaterialCategory::Concrete, 2.4e-9, Some(24.0));
        let steel = material(1, MaterialCategory::Steel, 8.0, None);
        let rebar_material = material(2, MaterialCategory::Rebar, 8.0, None);
        assert!(SectionMassProperties::try_from_section(
            &rc,
            Some(&concrete),
            Some(&concrete),
            None,
            None
        )
        .is_err());
        assert!(SectionMassProperties::try_from_section(
            &rc,
            Some(&concrete),
            None,
            Some(&steel),
            None
        )
        .is_err());
        assert!(SectionMassProperties::try_from_section(
            &rc,
            Some(&concrete),
            Some(&steel),
            Some(&steel),
            None
        )
        .is_err());
        let src = SectionShape::SrcRect {
            b: 600.0,
            d: 600.0,
            rebar,
            steel_height: 400.0,
            steel_width: 200.0,
            steel_web_thick: 9.0,
            steel_flange_thick: 12.0,
        }
        .to_section(SectionId(1), "SRC".into());
        assert!(SectionMassProperties::try_from_section(
            &src,
            Some(&concrete),
            None,
            None,
            Some(&concrete)
        )
        .is_err());
        assert!(SectionMassProperties::try_from_section(
            &src,
            Some(&concrete),
            Some(&rebar_material),
            Some(&rebar_material),
            Some(&steel)
        )
        .is_ok());
    }

    #[test]
    fn 鋼材形状は鋼材主材料と成立する寸法関係を要求する() {
        let steel = material(0, MaterialCategory::Steel, 8.0, None);
        let concrete = material(1, MaterialCategory::Concrete, 2.4e-9, Some(24.0));
        let h = SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 9.0,
            flange_thick: 12.0,
        }
        .to_section(SectionId(0), "H".into());
        assert!(SectionMassProperties::try_from_section(&h, None, None, None, None).is_err());
        assert!(
            SectionMassProperties::try_from_section(&h, Some(&concrete), None, None, None).is_err()
        );
        assert!(
            SectionMassProperties::try_from_section(&h, Some(&steel), None, None, None).is_ok()
        );

        for shape in [
            SectionShape::SteelH {
                height: 24.0,
                width: 200.0,
                web_thick: 9.0,
                flange_thick: 12.0,
            },
            SectionShape::SteelPipe {
                outer_dia: 24.0,
                thick: 12.0,
            },
            SectionShape::SteelBox {
                height: 24.0,
                width: 200.0,
                thick: 12.0,
                corner_r: 0.0,
            },
        ] {
            let section = shape.to_section(SectionId(1), "invalid-steel".into());
            assert!(SectionMassProperties::try_from_section(
                &section,
                Some(&steel),
                None,
                None,
                None,
            )
            .is_err());
        }
    }

    #[test]
    fn 壁とスラブの補助材料の対応範囲を検証する() {
        let concrete = material(0, MaterialCategory::Concrete, 2.4e-9, Some(24.0));
        let steel = material(1, MaterialCategory::Steel, 8.0, None);
        let rebar = material(2, MaterialCategory::Rebar, 7.0, None);
        let wall = SectionShape::RcWall {
            thickness: 150.0,
            ps: 0.0025,
        }
        .to_section(SectionId(0), "RC壁".into());
        assert!(SectionMassProperties::try_from_section(
            &wall,
            Some(&concrete),
            Some(&steel),
            None,
            None,
        )
        .is_err());
        assert!(SectionMassProperties::try_from_section(
            &wall,
            Some(&concrete),
            Some(&rebar),
            None,
            None,
        )
        .is_ok());
        let slab =
            SectionShape::RcSlab { thickness: 150.0 }.to_section(SectionId(1), "RC床版".into());
        assert!(SectionMassProperties::try_from_section(
            &slab,
            Some(&concrete),
            Some(&rebar),
            None,
            None,
        )
        .is_err());
        for section in [wall, slab] {
            assert!(SectionMassProperties::try_from_section(
                &section,
                Some(&concrete),
                Some(&steel),
                None,
                None,
            )
            .is_err());
        }
    }
}
