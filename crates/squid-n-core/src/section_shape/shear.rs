/// 区分一定幅断面の性能。長さはmm、面積はmm²、断面二次モーメントはmm⁴。
#[derive(Clone, Copy, Debug)]
pub struct StripSectionProperties {
    pub area: f64,
    pub centroid: f64,
    pub inertia: f64,
    pub shear_area: f64,
}

/// 壁と矩形側柱の和集合断面。同一材料・壁厚中央に柱中心がある平行配置を前提とする。
/// 寸法はmm。左右柱は各（沿壁方向せい、壁直交方向幅）、Noneは柱なし。図心は左の壁端から測る。
/// 非正・非有限の寸法または算定不能な性能値はNone。
pub fn wall_rectangular_section_properties(
    wall_length_mm: f64,
    wall_thickness_mm: f64,
    columns: [Option<[f64; 2]>; 2],
) -> Option<StripSectionProperties> {
    let valid = |v: f64| v.is_finite() && v > 0.0;
    if !valid(wall_length_mm)
        || !valid(wall_thickness_mm)
        || columns.iter().flatten().flatten().any(|&v| !valid(v))
    {
        return None;
    }
    let mut rectangles = vec![[0.0, wall_length_mm, wall_thickness_mm]];
    for (center, column) in [0.0, wall_length_mm].into_iter().zip(columns) {
        if let Some([depth, width]) = column {
            rectangles.push([center - depth / 2.0, center + depth / 2.0, width]);
        }
    }
    let mut boundaries: Vec<f64> = rectangles.iter().flat_map(|r| [r[0], r[1]]).collect();
    if boundaries.iter().any(|v| !v.is_finite()) {
        return None;
    }
    boundaries.sort_by(f64::total_cmp);
    boundaries.dedup();
    let mut strips = Vec::new();
    for pair in boundaries.windows(2) {
        let width = rectangles
            .iter()
            .filter(|r| r[0] <= pair[0] && pair[1] <= r[1])
            .map(|r| r[2])
            .fold(0.0, f64::max);
        strips.push([pair[1] - pair[0], width]);
    }
    let mut properties = strip_section_properties(&strips)?;
    properties.centroid += boundaries[0];
    Some(properties)
}

/// 連続する区間の（長さ、幅）[mm]から断面性能を求める。図心は最初の区間始点からの距離。
/// 同一材料・幅方向対称の断面を前提とし、Q²/bの区分積分でせん断有効断面積を求める。
/// 空列・非正または非有限の寸法・算定不能な性能値はNone。
pub fn strip_section_properties(strips: &[[f64; 2]]) -> Option<StripSectionProperties> {
    let layers: Vec<_> = strips
        .iter()
        .map(|&[length_mm, width_mm]| MaterialSectionStrip {
            length_mm,
            width_mm,
            young_mpa: 1.0,
            shear_mpa: 1.0,
        })
        .collect();
    let p = material_strip_section_properties(&layers)?;
    Some(StripSectionProperties {
        area: p.area_mm2,
        centroid: p.elastic_centroid_mm,
        inertia: p.flexural_rigidity_n_mm2,
        shear_area: p.shear_rigidity_n,
    })
}

/// 断面の連続区間。幅と材料定数は区間内で一定、幅方向に対称とする。
#[derive(Clone, Copy, Debug)]
pub struct MaterialSectionStrip {
    pub length_mm: f64,
    pub width_mm: f64,
    pub young_mpa: f64,
    pub shear_mpa: f64,
}

/// 材料定数を反映した断面性能。弾性図心は最初の区間始点からの距離。
#[derive(Clone, Copy, Debug)]
pub struct MaterialStripSectionProperties {
    pub area_mm2: f64,
    pub elastic_centroid_mm: f64,
    pub flexural_rigidity_n_mm2: f64,
    pub shear_rigidity_n: f64,
}

/// 完全付着した区分一定幅・区分一定材料の断面を、平面保持とせん断流の釣合いから積分する。
/// 材料の既定値は持たず、全区間のE・Gを明示する。
/// 空列、非正・非有限の寸法または材料定数、算定不能な性能値はNone。
pub fn material_strip_section_properties(
    strips: &[MaterialSectionStrip],
) -> Option<MaterialStripSectionProperties> {
    if strips.is_empty()
        || strips.iter().any(|s| {
            [s.length_mm, s.width_mm, s.young_mpa, s.shear_mpa]
                .iter()
                .any(|v| !v.is_finite() || *v <= 0.0)
        })
    {
        return None;
    }
    let mut x = 0.0;
    let mut area = 0.0;
    let mut ea = 0.0;
    let mut first_moment = 0.0;
    for s in strips {
        let a = s.length_mm * s.width_mm;
        let weighted_a = a * s.young_mpa;
        area += a;
        ea += weighted_a;
        first_moment += weighted_a * (x + s.length_mm / 2.0);
        x += s.length_mm;
    }
    let centroid = first_moment / ea;
    let mut ei = 0.0;
    let mut integral = 0.0;
    let mut q_start = 0.0;
    x = 0.0;
    let gauss = (3.0_f64 / 5.0).sqrt();
    for strip in strips {
        let length = strip.length_mm;
        let eb = strip.young_mpa * strip.width_mm;
        let gb = strip.shear_mpa * strip.width_mm;
        let a = x - centroid;
        ei += eb * length * (length * length / 12.0 + (a + length / 2.0).powi(2));
        for (point, weight) in [(-gauss, 5.0 / 9.0), (0.0, 8.0 / 9.0), (gauss, 5.0 / 9.0)] {
            let distance = length * (point + 1.0) / 2.0;
            let q = q_start + eb * distance * (a + distance / 2.0);
            integral += weight * length / 2.0 * q * q / gb;
        }
        q_start += eb * length * (a + length / 2.0);
        x += length;
    }
    let shear_rigidity = ei * ei / integral;
    if ![area, ea, ei, integral, shear_rigidity]
        .iter()
        .all(|v| v.is_finite() && *v > 0.0)
        || !centroid.is_finite()
    {
        return None;
    }
    Some(MaterialStripSectionProperties {
        area_mm2: area,
        elastic_centroid_mm: centroid,
        flexural_rigidity_n_mm2: ei,
        shear_rigidity_n: shear_rigidity,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn material_strips_reflect_elastic_centroid_and_shear_energy() {
        let mut strips = [
            MaterialSectionStrip {
                length_mm: 1.0,
                width_mm: 1.0,
                young_mpa: 10.0,
                shear_mpa: 3.0,
            },
            MaterialSectionStrip {
                length_mm: 1.0,
                width_mm: 1.0,
                young_mpa: 20.0,
                shear_mpa: 5.0,
            },
        ];
        // E比1:2で図心7/6、EI=10×11/12。∫QE²/(Gb)=100×2093/16200。
        let p = material_strip_section_properties(&strips).unwrap();
        assert!((p.area_mm2 - 2.0).abs() < 1e-12);
        assert!((p.elastic_centroid_mm - 7.0 / 6.0).abs() < 1e-12);
        assert!((p.flexural_rigidity_n_mm2 - 55.0 / 6.0).abs() < 1e-12);
        assert!((p.shear_rigidity_n - 27225.0 / 4186.0).abs() < 1e-12);
        strips.reverse();
        let reverse = material_strip_section_properties(&strips).unwrap();
        assert!((p.shear_rigidity_n - reverse.shear_rigidity_n).abs() < 1e-12);
        for s in &mut strips {
            s.shear_mpa *= 2.0;
        }
        let doubled_g = material_strip_section_properties(&strips).unwrap();
        assert!((doubled_g.shear_rigidity_n / p.shear_rigidity_n - 2.0).abs() < 1e-12);
        assert!((doubled_g.elastic_centroid_mm - reverse.elastic_centroid_mm).abs() < 1e-12);
    }

    #[test]
    fn material_strips_recover_homogeneous_properties() {
        let geometry = [[2.0, 3.0], [3.0, 1.0], [1.0, 4.0]];
        let geometric = strip_section_properties(&geometry).unwrap();
        let strips: Vec<_> = geometry
            .iter()
            .map(|&[length_mm, width_mm]| MaterialSectionStrip {
                length_mm,
                width_mm,
                young_mpa: 30000.0,
                shear_mpa: 12500.0,
            })
            .collect();
        let p = material_strip_section_properties(&strips).unwrap();
        assert!((p.flexural_rigidity_n_mm2 / (30000.0 * geometric.inertia) - 1.0).abs() < 1e-12);
        assert!((p.shear_rigidity_n / (12500.0 * geometric.shear_area) - 1.0).abs() < 1e-12);
        let mut invalid = strips;
        invalid[0].shear_mpa = 0.0;
        assert!(material_strip_section_properties(&invalid).is_none());
    }

    #[test]
    fn wall_union_deducts_overlap_and_preserves_individual_columns() {
        let p =
            wall_rectangular_section_properties(4000.0, 150.0, [Some([600.0, 600.0]); 2]).unwrap();
        assert!((p.area - 1_230_000.0).abs() < 1e-7);
        assert!((p.centroid - 2000.0).abs() < 1e-9);
        let expected_kappa =
            super::super::wall_shear_shape_factor_isection(4600.0, 600.0, 600.0, 150.0);
        assert!((p.area / p.shear_area - expected_kappa).abs() < 1e-12);
        let p = wall_rectangular_section_properties(
            4000.0,
            150.0,
            [Some([600.0, 800.0]), Some([400.0, 500.0])],
        )
        .unwrap();
        // 壁600000＋左柱480000＋右柱200000−左重複45000−右重複30000。
        assert!((p.area - 1_205_000.0).abs() < 1e-7);
        let reverse = wall_rectangular_section_properties(
            4000.0,
            150.0,
            [Some([400.0, 500.0]), Some([600.0, 800.0])],
        )
        .unwrap();
        assert!((p.centroid + reverse.centroid - 4000.0).abs() < 1e-9);
        assert!((p.shear_area / reverse.shear_area - 1.0).abs() < 1e-12);
        let rotated = wall_rectangular_section_properties(
            4000.0,
            150.0,
            [Some([800.0, 600.0]), Some([400.0, 500.0])],
        )
        .unwrap();
        assert!((rotated.area - 1_190_000.0).abs() < 1e-7);
        assert!((p.shear_area - rotated.shear_area).abs() > 1.0);
    }

    #[test]
    fn wall_union_handles_narrow_and_overlapping_columns() {
        let p = wall_rectangular_section_properties(4000.0, 150.0, [Some([600.0, 100.0]), None])
            .unwrap();
        assert!((p.area - 630_000.0).abs() < 1e-7);
        let p =
            wall_rectangular_section_properties(100.0, 150.0, [Some([400.0, 500.0]); 2]).unwrap();
        assert!((p.area - 250_000.0).abs() < 1e-7);
        assert!((p.area / p.shear_area - 1.2).abs() < 1e-12);
    }

    #[test]
    fn asymmetric_section_matches_rational_integrals() {
        // 区間[0,1]の幅1、[1,2]の幅2。A=3、図心7/6、I=11/12、∫Q²/b=43/120。
        for scale in [0.001, 1.0, 1000.0] {
            let p = strip_section_properties(&[[scale, scale], [scale, 2.0 * scale]]).unwrap();
            assert!((p.area / scale.powi(2) - 3.0).abs() < 1e-12);
            assert!((p.centroid / scale - 7.0 / 6.0).abs() < 1e-12);
            assert!((p.inertia / scale.powi(4) - 11.0 / 12.0).abs() < 1e-12);
            assert!((p.shear_area / scale.powi(2) - 605.0 / 258.0).abs() < 1e-12);
            let reverse =
                strip_section_properties(&[[scale, 2.0 * scale], [scale, scale]]).unwrap();
            assert!((reverse.shear_area / p.shear_area - 1.0).abs() < 1e-12);
            assert!(((reverse.centroid + p.centroid) / scale - 2.0).abs() < 1e-12);
        }
    }

    #[test]
    fn rectangular_section_is_independent_of_partition() {
        let p = strip_section_properties(&[[4.0, 2.0]]).unwrap();
        let split = strip_section_properties(&[[1.0, 2.0], [2.0, 2.0], [1.0, 2.0]]).unwrap();
        assert!((p.shear_area - 8.0 * 5.0 / 6.0).abs() < 1e-12);
        assert!((p.shear_area - split.shear_area).abs() < 1e-12);
        assert!(strip_section_properties(&[]).is_none());
        assert!(strip_section_properties(&[[0.0, 1.0]]).is_none());
        assert!(strip_section_properties(&[[1.0, f64::NAN]]).is_none());
    }
}
