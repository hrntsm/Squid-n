use super::*;
use crate::transform::LocalFrame;
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, LocalAxis, Material, MaterialCategory, Model, Node,
    RigidZone, Section,
};

fn make_test_beam() -> BeamElement {
    BeamElement {
        id: ElemId(0),
        e: 205000.0,
        g: 78846.15,
        a: 80000.0,
        a_mass: 80000.0,
        iy: 1.0666667e9,
        iz: 1.0666667e9,
        j: 0.0,
        as_y: 66666.67,
        as_z: 66666.67,
        length: 3000.0,
        density: 0.0,
        nodes: [NodeId(0), NodeId(1)],
        axis: LocalFrame {
            rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        },
        rigid: RigidZone::default(),
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        torsion_release: [false, false],
        eval_sections: vec![0.0, 0.5, 1.0],
        section: None,
        material: None,
        committed_disp: [0.0; 12],
        trial_disp: [0.0; 12],
        local_stiffness_cache: std::sync::OnceLock::new(),
    }
}

/// 変位増分の長さが自由度数と食い違ったら、要素名を名指しして落とす。
///
/// 短い方へ黙って合わせると、変位の一部が欠けたまま解析が続行して誤った内力を返す。
/// 節点数が可変の要素（3 節点シェル等）が入ると長さが変わりうるため、
/// 走査範囲は `n_dof` に追従させたうえで不一致は落とす規約にしている。
#[test]
#[should_panic(expected = "変位増分の長さが自由度数と一致しません")]
fn update_state_rejects_mismatched_du_length() {
    use crate::behavior::{Ctx, ElementBehavior, LocalVec};
    use squid_n_core::model::Model;

    let model = Model::default();
    let ctx = Ctx { model: &model };
    let mut elem = make_test_beam();
    let du = LocalVec {
        data: smallvec::SmallVec::from_elem(0.0, 6),
    };
    elem.update_state(&du, false, &ctx);
}

/// SRC/CFT の複合換算が要素生成へ配線されていること。
#[test]
fn test_beam_new_src_cft_composite_props() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::model::{EndCondition, ForceRegime, LocalAxis, Model};
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar, E_STEEL, N_S_EQ};

    let src_shape = SectionShape::SrcRect {
        b: 600.0,
        d: 600.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 8,
                dia: 22.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 8,
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
    let cft_shape = SectionShape::CftBox {
        height: 400.0,
        width: 400.0,
        thick: 12.0,
    };

    let mut model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        sections: vec![
            Section {
                material: Some(MaterialId(0)),
                ..src_shape.to_section(SectionId(0), "SRC-600".into())
            },
            Section {
                material: Some(MaterialId(1)),
                ..cft_shape.to_section(SectionId(1), "CFT-400".into())
            },
        ],
        materials: vec![
            Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "FC24".into(),
                category: MaterialCategory::Concrete,
                young: 23000.0,
                poisson: 0.2,
                density: 2.4e-9,
                shear: None,
                fc: Some(24.0),
                fy: None,
            },
            Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(1),
                name: "BCR295(充填FC36)".into(),
                category: MaterialCategory::Steel,
                young: 205000.0,
                poisson: 0.3,
                density: 7.85e-9,
                shear: None,
                fc: Some(36.0),
                fy: Some(295.0),
            },
        ],
        ..Default::default()
    };
    let make_elem = |sec: u32| ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(squid_n_core::ids::SectionId(sec)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    let src_beam = BeamElement::new(&make_elem(0), &model);
    let p = src_shape.src_equivalent_props(23000.0, 0.2).unwrap();
    assert!((src_beam.a - p.area_ax).abs() < 1e-6);
    assert!((src_beam.iz - p.iy).abs() / p.iy < 1e-12);
    assert!((src_beam.j - p.j).abs() / p.j < 1e-12);
    assert!((src_beam.as_y - p.as_z).abs() < 1e-6);
    let ns = E_STEEL / 23000.0;
    assert!((ns - N_S_EQ).abs() > 1.0);
    assert!((src_beam.a_mass - 360_000.0).abs() < 1e-9);

    let cft_beam = BeamElement::new(&make_elem(1), &model);
    let pc = cft_shape.cft_equivalent_props(205000.0, 0.3, 36.0).unwrap();
    assert!((cft_beam.a - pc.area_ax).abs() < 1e-6);
    assert!((cft_beam.iz - pc.iy).abs() / pc.iy < 1e-12);
    assert!((cft_beam.j - pc.j).abs() / pc.j < 1e-12);

    model.materials[0].fc = None;
    let src_fallback = BeamElement::new(&make_elem(0), &model);
    assert!((src_fallback.a - src_shape.calc_axial_stiffness_area()).abs() < 1e-6);
    assert!((src_fallback.iz - model.sections[0].iy).abs() < 1e-6);
}

/// スラブ協力幅による強軸剛性増大。
#[test]
fn test_beam_new_slab_cooperation_width_amplifies_iy() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{FloorRegionId, MaterialId, SectionId, SlabId};
    use squid_n_core::model::{
        DistributionMethod, EndCondition, FloorRegion, ForceRegime, LocalAxis, Model, Slab,
        SlabPlate, SlabShape,
    };
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    };
    let shape = SectionShape::RcRect {
        b: 300.0,
        d: 600.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 4,
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
        },
    };
    let mut model = Model {
        nodes: vec![
            make_node(0, [0.0, 0.0, 3000.0]),
            make_node(1, [6000.0, 0.0, 3000.0]),
            make_node(2, [6000.0, 2500.0, 3000.0]),
            make_node(3, [0.0, 2500.0, 3000.0]),
        ],
        sections: vec![
            shape.to_section(SectionId(0), "RC-300x600".into()),
            SectionShape::RcSlab { thickness: 150.0 }.to_section(SectionId(1), "S15".into()),
        ],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "FC24".into(),
            category: MaterialCategory::Concrete,
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        }],
        floor_regions: vec![FloorRegion::new(
            FloorRegionId(0),
            vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        )],
        slabs: vec![Slab {
            id: SlabId(0),
            shape: SlabShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            plate: SlabPlate {
                section: Some(SectionId(1)),
                loads: vec![],
                usage: None,
                method: DistributionMethod::TriTrapezoid,
                one_way: None,
            },
        }],
        slab_thickness: 150.0,
        ..Default::default()
    };
    let elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    let (b, d, t, l) = (300.0_f64, 600.0_f64, 150.0_f64, 6000.0_f64);
    let a_clear = 2500.0 - b / 2.0 - b / 2.0;
    let ba = (0.5 - 0.6 * a_clear / l) * a_clear;
    assert!((ba - 616.0).abs() < 1e-9);
    let bf = b + ba;
    let (aw, af) = (b * d, (bf - b) * t);
    let g = (aw * d / 2.0 + af * (d - t / 2.0)) / (aw + af);
    let i0 = b * d.powi(3) / 12.0;
    let ie = i0
        + aw * (g - d / 2.0).powi(2)
        + (bf - b) * t.powi(3) / 12.0
        + af * (d - t / 2.0 - g).powi(2);

    let beam = BeamElement::new(&elem, &model);
    assert!(
        (beam.iz - ie).abs() / ie < 1e-12,
        "iz={} ie={}",
        beam.iz,
        ie
    );
    assert!(beam.iz / i0 > 1.3, "増大率が小さすぎる: {}", beam.iz / i0);
    assert!((beam.iy - model.sections[0].iz).abs() < 1e-9);

    model.slabs.clear();
    let beam0 = BeamElement::new(&elem, &model);
    assert!((beam0.iz - i0).abs() < 1e-9);
}

/// 床領域（大梁の1区画）が小梁で複数の床板（`Slab`）へ細分されていても、
/// 床領域の外周を走る大梁のスラブ協力幅は効く（境界の判定は `FloorRegion::boundary`
/// を優先するため。個々の床板の境界だけで判定すると、大梁の両端が別々の床板
/// にまたがり、どちらの床板にも「両端を含む境界」が無くなって増大が消える
/// 回帰が起きる）。
#[test]
fn test_beam_new_slab_cooperation_width_survives_joist_subdivided_region() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{FloorRegionId, MaterialId, SectionId, SlabId};
    use squid_n_core::model::{
        DistributionMethod, EndCondition, FloorRegion, ForceRegime, LocalAxis, Model, Slab,
        SlabPlate, SlabShape,
    };
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    };
    let shape = SectionShape::RcRect {
        b: 300.0,
        d: 600.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 4,
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
        },
    };
    let model = Model {
        nodes: vec![
            make_node(0, [0.0, 0.0, 3000.0]),
            make_node(1, [6000.0, 0.0, 3000.0]),
            make_node(2, [6000.0, 5000.0, 3000.0]),
            make_node(3, [0.0, 5000.0, 3000.0]),
            make_node(4, [6000.0, 2500.0, 3000.0]),
            make_node(5, [0.0, 2500.0, 3000.0]),
        ],
        sections: vec![
            shape.to_section(SectionId(0), "RC-300x600".into()),
            SectionShape::RcSlab { thickness: 150.0 }.to_section(SectionId(1), "S15".into()),
        ],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "FC24".into(),
            category: MaterialCategory::Concrete,
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        }],
        floor_regions: vec![FloorRegion {
            slab_ids: vec![SlabId(0), SlabId(1)],
            ..FloorRegion::new(
                FloorRegionId(0),
                vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            )
        }],
        slabs: vec![
            Slab {
                id: SlabId(0),
                shape: SlabShape::Enclosed {
                    boundary: vec![NodeId(0), NodeId(1), NodeId(4), NodeId(5)],
                },
                plate: SlabPlate {
                    section: Some(SectionId(1)),
                    loads: vec![],
                    usage: None,
                    method: DistributionMethod::TriTrapezoid,
                    one_way: None,
                },
            },
            Slab {
                id: SlabId(1),
                shape: SlabShape::Enclosed {
                    boundary: vec![NodeId(5), NodeId(4), NodeId(2), NodeId(3)],
                },
                plate: SlabPlate {
                    section: Some(SectionId(1)),
                    loads: vec![],
                    usage: None,
                    method: DistributionMethod::TriTrapezoid,
                    one_way: None,
                },
            },
        ],
        slab_thickness: 150.0,
        ..Default::default()
    };
    let elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    let (b, d) = (300.0_f64, 600.0_f64);
    let i0 = b * d.powi(3) / 12.0;
    let beam = BeamElement::new(&elem, &model);
    assert!(
        beam.iz / i0 > 1.0,
        "床領域が細分されていても大梁のスラブ協力幅は増大するはず: iz/i0={}",
        beam.iz / i0
    );
}

fn rc_beam_for_slab_factor(plate: Option<squid_n_core::model::SlabPlate>) -> (Model, ElementData) {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{FloorRegionId, MaterialId, SectionId, SlabId};
    use squid_n_core::model::{EndCondition, FloorRegion, ForceRegime, LocalAxis, Slab, SlabShape};
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};
    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    };
    let shape = SectionShape::RcRect {
        b: 300.0,
        d: 600.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 4,
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
        },
    };
    let mut beam_sec = shape.to_section(SectionId(0), "RC-300x600".into());
    beam_sec.material = Some(MaterialId(0));
    let mut slab_sec =
        SectionShape::RcSlab { thickness: 150.0 }.to_section(SectionId(1), "S15".into());
    slab_sec.material = Some(MaterialId(0));
    let boundary = vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)];
    let slabs = match plate {
        Some(plate) => vec![Slab {
            id: SlabId(0),
            shape: SlabShape::Enclosed {
                boundary: boundary.clone(),
            },
            plate,
        }],
        None => vec![],
    };
    let model = Model {
        nodes: vec![
            make_node(0, [0.0, 0.0, 3000.0]),
            make_node(1, [6000.0, 0.0, 3000.0]),
            make_node(2, [6000.0, 2500.0, 3000.0]),
            make_node(3, [0.0, 2500.0, 3000.0]),
        ],
        sections: vec![beam_sec, slab_sec],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "FC24".into(),
            category: MaterialCategory::Concrete,
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        }],
        floor_regions: vec![FloorRegion::new(FloorRegionId(0), boundary)],
        slabs,
        slab_thickness: 150.0,
        ..Default::default()
    };
    let elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    (model, elem)
}

/// 版なし床領域は協力幅を見込まない（建物一律のスラブ厚があっても 1.0）。
#[test]
fn test_plateless_region_does_not_increase_slab_stiffness() {
    use squid_n_core::model::SlabPlate;
    let (m_none, e) = rc_beam_for_slab_factor(None);
    let b_none = stiffness_breakdown(&m_none, &e);
    assert!(
        (b_none.slab - 1.0).abs() < 1e-12,
        "版なしは協力幅 1.0、got {}",
        b_none.slab
    );

    let (m_some, _) = rc_beam_for_slab_factor(Some(SlabPlate {
        section: Some(squid_n_core::ids::SectionId(1)),
        ..Default::default()
    }));
    let b_some = stiffness_breakdown(&m_some, &e);
    assert!(
        b_some.slab > 1.0,
        "版あり＋断面厚なら協力幅 > 1、got {}",
        b_some.slab
    );
}

/// 取り付き版ありは協力幅の対象外（囲まれた版だけが増大する）。
#[test]
fn test_attached_plate_does_not_increase_slab_stiffness() {
    use squid_n_core::ids::SlabId;
    use squid_n_core::model::{LoadTransfer, RegionAnchor, Slab, SlabPlate, SlabShape};
    let (mut m, e) = rc_beam_for_slab_factor(None);
    m.slabs = vec![Slab {
        id: SlabId(0),
        shape: SlabShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(0), NodeId(1)],
                span: [0.0, 1.0],
                transfer: LoadTransfer::Anchor,
            },
            extent: [1500.0, 1500.0],
        },
        plate: SlabPlate {
            section: Some(squid_n_core::ids::SectionId(1)),
            ..Default::default()
        },
    }];
    let b = stiffness_breakdown(&m, &e);
    assert!(
        (b.slab - 1.0).abs() < 1e-12,
        "取り付く床板は協力幅 1.0、got {}",
        b.slab
    );
}

/// S 造合成梁の剛性（スラブ考慮換算断面と鉄骨単独の平均。計算編 02「合成梁の
/// 断面性能」）。
#[test]
fn test_beam_new_composite_steel_beam_averages_stiffness() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{FloorRegionId, MaterialId, SectionId, SlabId};
    use squid_n_core::model::{
        DistributionMethod, EndCondition, FloorRegion, ForceRegime, LocalAxis, Model, Slab,
        SlabPlate, SlabShape,
    };
    use squid_n_core::section_shape::SectionShape;

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    };
    let shape = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    };
    let mut model = Model {
        nodes: vec![
            make_node(0, [0.0, 0.0, 3000.0]),
            make_node(1, [6000.0, 0.0, 3000.0]),
            make_node(2, [6000.0, 2500.0, 3000.0]),
            make_node(3, [0.0, 2500.0, 3000.0]),
        ],
        sections: vec![
            Section {
                material: Some(MaterialId(0)),
                ..shape.to_section(SectionId(0), "H-400x200".into())
            },
            SectionShape::RcSlab { thickness: 150.0 }.to_section(SectionId(1), "S15".into()),
        ],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SN400B".into(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: Some(235.0),
        }],
        floor_regions: vec![FloorRegion::new(
            FloorRegionId(0),
            vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        )],
        slabs: vec![Slab {
            id: SlabId(0),
            shape: SlabShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            plate: SlabPlate {
                section: Some(SectionId(1)),
                loads: vec![],
                usage: None,
                method: DistributionMethod::TriTrapezoid,
                one_way: None,
            },
        }],
        slab_thickness: 150.0,
        ..Default::default()
    };
    let elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    let sec = &model.sections[0];
    let (sa, si, sh) = (sec.area, sec.iy, 400.0_f64);
    let (es, t, l) = (205000.0_f64, 150.0_f64, 6000.0_f64);
    let a_clear = 2500.0 - 100.0 - 100.0;
    let ba = (0.5 - 0.6 * a_clear / l) * a_clear;
    assert!((ba - 621.0).abs() < 1e-9);
    let bf = 200.0 + ba;
    let ec = squid_n_core::section_shape::concrete_young_modulus(21.0);
    let ca = bf * t;
    let g = (ec * ca * (t / 2.0) + es * sa * (t + sh / 2.0)) / (ec * ca + es * sa);
    let i_comp = (ec / es) * (bf * t.powi(3) / 12.0 + ca * (g - t / 2.0).powi(2))
        + si
        + sa * (g - t - sh / 2.0).powi(2);
    let expected = (i_comp + si) / 2.0;

    let beam = BeamElement::new(&elem, &model);
    assert!(
        (beam.iz - expected).abs() / expected < 1e-12,
        "iz={} expected={}",
        beam.iz,
        expected
    );
    assert!(beam.iz > si && beam.iz < i_comp);
    assert!((beam.iy - model.sections[0].iz).abs() < 1e-9);

    model.slabs.clear();
    let beam0 = BeamElement::new(&elem, &model);
    assert!((beam0.iz - si).abs() < 1e-9);
}

#[test]
fn test_local_stiffness_symmetric() {
    let beam = make_test_beam();
    let k = beam.local_stiffness_raw();
    for i in 0..12 {
        for j in 0..12 {
            assert!(
                (k.get(i, j) - k.get(j, i)).abs() < 1e-9,
                "K[{i}][{j}] != K[{j}][{i}]: {} vs {}",
                k.get(i, j),
                k.get(j, i)
            );
        }
    }
}

#[test]
fn test_phi_zero_converges_to_bernoulli() {
    let mut beam = make_test_beam();
    beam.as_y = 1e30;
    beam.as_z = 1e30;
    let k_timo = beam.local_stiffness_raw();

    let e = beam.e;
    let iz = beam.iz;
    let iy = beam.iy;
    let a = beam.a;
    let l = beam.length;
    let g = beam.g;
    let jj = beam.j;

    let az = e * iz / (l * l * l);
    let ay = e * iy / (l * l * l);

    for i in 0..12 {
        for j in 0..12 {
            let norm_pair = if i <= j { (i, j) } else { (j, i) };
            let bernoulli = match norm_pair {
                (0, 0) | (6, 6) => e * a / l,
                (0, 6) => -e * a / l,
                (3, 3) | (9, 9) => g * jj / l,
                (3, 9) => -g * jj / l,
                (1, 1) | (7, 7) => 12.0 * az,
                (1, 7) => -12.0 * az,
                (1, 5) | (1, 11) => 6.0 * az * l,
                (5, 7) | (7, 11) => -6.0 * az * l,
                (5, 5) | (11, 11) => 4.0 * az * l * l,
                (5, 11) => 2.0 * az * l * l,
                (2, 2) | (8, 8) => 12.0 * ay,
                (2, 8) => -12.0 * ay,
                (2, 4) | (2, 10) => -6.0 * ay * l,
                (4, 8) | (8, 10) => 6.0 * ay * l,
                (4, 4) | (10, 10) => 4.0 * ay * l * l,
                (4, 10) => 2.0 * ay * l * l,
                _ => 0.0,
            };
            let timo = k_timo.get(i, j);
            assert!(
                (timo - bernoulli).abs() < 1e-6,
                "K[{i}][{j}]: timo={timo}, bernoulli={bernoulli}"
            );
        }
    }
}

#[test]
fn test_beam_axial_stiffness() {
    let beam = make_test_beam();
    let k = beam.local_stiffness_raw();
    let ea_l = beam.e * beam.a / beam.length;
    assert!((k.get(0, 0) - ea_l).abs() < 1e-9);
    assert!((k.get(0, 6) + ea_l).abs() < 1e-9);
    assert!((k.get(6, 0) + ea_l).abs() < 1e-9);
    assert!((k.get(6, 6) - ea_l).abs() < 1e-9);
}

#[test]
fn test_beam_torsion_stiffness() {
    let beam = make_test_beam();
    let k = beam.local_stiffness_raw();
    let gj_l = beam.g * beam.j / beam.length;
    assert!((k.get(3, 3) - gj_l).abs() < 1e-9);
    assert!((k.get(9, 9) - gj_l).abs() < 1e-9);
    assert!((k.get(3, 9) + gj_l).abs() < 1e-9);
}

#[test]
fn test_rigid_zone_preserves_rigid_body_rotation() {
    let mut beam = make_test_beam();
    beam.j = 5.0e8;
    beam.rigid = RigidZone {
        length_i: 300.0,
        length_j: 300.0,
        ..Default::default()
    };
    let k = beam.local_stiffness();

    let theta = 1.0;
    let l = beam.length;
    let u = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        theta,
        0.0,
        theta * l,
        0.0,
        0.0,
        0.0,
        theta,
    ];
    let mut fmax = 0.0_f64;
    for i in 0..12 {
        let mut fi = 0.0;
        for j in 0..12 {
            fi += k.get(i, j) * u[j];
        }
        fmax = fmax.max(fi.abs());
    }
    let scale = k.get(1, 1).abs().max(1.0);
    assert!(
        fmax / scale < 1e-9,
        "rigid-body z-rotation must produce ~zero nodal force: fmax={fmax}, scale={scale}"
    );

    let u_y = [
        0.0,
        0.0,
        0.0,
        0.0,
        theta,
        0.0,
        0.0,
        0.0,
        -theta * l,
        0.0,
        theta,
        0.0,
    ];
    let mut fmax_y = 0.0_f64;
    for i in 0..12 {
        let mut fi = 0.0;
        for j in 0..12 {
            fi += k.get(i, j) * u_y[j];
        }
        fmax_y = fmax_y.max(fi.abs());
    }
    assert!(
        fmax_y / scale < 1e-9,
        "rigid-body y-rotation must produce ~zero nodal force: fmax_y={fmax_y}, scale={scale}"
    );
}

#[test]
fn test_torsion_not_stiffened_by_rigid_zone() {
    let mut beam = make_test_beam();
    beam.j = 5.0e8;
    beam.rigid = RigidZone {
        length_i: 300.0,
        length_j: 300.0,
        ..Default::default()
    };
    let k = beam.local_stiffness();
    let gj_l = beam.g * beam.j / beam.length;
    assert!(
        (k.get(3, 3) - gj_l).abs() / gj_l < 1e-9,
        "ねじりは GJ/L: got {}, want {}",
        k.get(3, 3),
        gj_l
    );
    assert!((k.get(3, 9) + gj_l).abs() / gj_l < 1e-9);
}

#[test]
fn test_geometric_stiffness_consistent_with_rigid_zone() {
    use crate::behavior::ElementBehavior;
    let n = 1000.0;
    let kg = make_test_beam().geometric_stiffness(n);
    let expected_full = n / 3000.0 * 6.0 / 5.0;
    assert!((kg.get(1, 1) - expected_full).abs() / expected_full < 1e-9);

    let mut beam_rz = make_test_beam();
    beam_rz.rigid = RigidZone {
        length_i: 300.0,
        length_j: 300.0,
        ..Default::default()
    };
    let kg_rz = beam_rz.geometric_stiffness(n);
    let expected_flex = n / 2400.0 * 6.0 / 5.0;
    assert!(
        (kg_rz.get(1, 1) - expected_flex).abs() / expected_flex < 1e-9,
        "剛域ありは可撓長基準: got {}, want {}",
        kg_rz.get(1, 1),
        expected_flex
    );
    assert!(kg_rz.get(1, 1) > kg.get(1, 1));
}

#[test]
fn test_pinned_end_releases_moment() {
    let mut beam = make_test_beam();
    beam.end_cond = [EndCondition::Pinned, EndCondition::Fixed];
    let k = beam.local_stiffness();
    let k_fixed = make_test_beam().local_stiffness();
    assert!(k.get(4, 4) < k_fixed.get(4, 4) * 1e-6);
    assert!(k.get(5, 5) < k_fixed.get(5, 5) * 1e-6);
}

#[test]
fn test_fixed_ends_exact_equals_raw() {
    let beam = make_test_beam();
    let k = beam.local_stiffness();
    let raw = beam.local_stiffness_raw();
    for i in 0..12 {
        for j in 0..12 {
            assert!(
                (k.get(i, j) - raw.get(i, j)).abs() < 1e-9,
                "K[{i},{j}] {} != raw {}",
                k.get(i, j),
                raw.get(i, j)
            );
        }
    }
}

/// `local_stiffness` のキャッシュ（`local_stiffness_cache`）が数値結果を変えないこと。
/// 同一インスタンスへの複数回呼び出し・クローン後の呼び出しがいずれも、
/// キャッシュを持たない新規インスタンスの結果とビット一致すること。
#[test]
fn test_local_stiffness_cache_is_bit_exact() {
    let beam = make_test_beam();

    let k1 = beam.local_stiffness();
    let k2 = beam.local_stiffness();
    assert_eq!(k1.data, k2.data);

    let cloned = beam.clone();
    let k_cloned = cloned.local_stiffness();
    assert_eq!(k1.data, k_cloned.data);

    let fresh = make_test_beam();
    let k_fresh = fresh.local_stiffness();
    assert_eq!(k1.data, k_fresh.data);
}

#[test]
fn test_pinned_end_rotation_stiffness_exactly_zero() {
    let mut beam = make_test_beam();
    beam.end_cond = [EndCondition::Pinned, EndCondition::Fixed];
    let k = beam.local_stiffness();
    for r in [3usize, 4, 5] {
        for c in 0..12 {
            assert_eq!(
                k.get(r, c),
                0.0,
                "released rot DOF {r} row must be exactly 0 at col {c}"
            );
            assert_eq!(
                k.get(c, r),
                0.0,
                "released rot DOF {r} col must be exactly 0 at row {c}"
            );
        }
    }
}

#[test]
fn test_auto_rigid_zone_standard_formula() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    let col_sec = Section {
        id: SectionId(0),
        name: "col".to_string(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 600.0,
        width: 0.0,
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
    };
    let beam_sec = Section {
        id: SectionId(1),
        name: "beam".to_string(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 700.0,
        width: 0.0,
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
    };
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "conc".to_string(),
        category: MaterialCategory::Concrete,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    };

    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(2),
                coord: [4000.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                section: Some(SectionId(1)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![col_sec, beam_sec],
        materials: vec![mat],
        ..Default::default()
    };

    let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
    assert!((zone.length_i - 125.0).abs() < 1e-9);
    assert!(
        (zone.face_i_or_zero() - 300.0).abs() < 1e-9,
        "face_i={}",
        zone.face_i_or_zero()
    );
}

/// apply_auto_rigid_zones が ElementData::rigid_zone に反映され、
/// Manual 端が保護されることを確認する（剛域がモデル→解析へ接続されたこと）。
#[test]
fn test_apply_auto_rigid_zones_and_manual_protection() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{ElementKind, ZoneSource};

    let mk_sec = |id: u32, depth: f64| Section {
        id: SectionId(id),
        name: String::new(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth,
        width: 0.0,
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
    };
    let mk_node = |id: u32, c: [f64; 3]| Node {
        id: NodeId(id),
        coord: c,
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    };
    let mk_beam = |id: u32, a: u32, b: u32, sec: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(a), NodeId(b)],
        section: Some(SectionId(sec)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: squid_n_core::model::ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    let mut model = Model {
        nodes: vec![
            mk_node(0, [0.0, 0.0, 0.0]),
            mk_node(1, [0.0, 0.0, 3000.0]),
            mk_node(2, [4000.0, 0.0, 3000.0]),
        ],
        elements: vec![mk_beam(0, 0, 1, 0), mk_beam(1, 1, 2, 1)],
        sections: vec![mk_sec(0, 600.0), mk_sec(1, 700.0)],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: String::new(),
            category: MaterialCategory::Concrete,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        ..Default::default()
    };

    assert_eq!(model.elements[1].rigid_zone.length_i, 0.0);

    apply_auto_rigid_zones(&mut model, &RigidZoneRule::default());
    assert!(
        (model.elements[1].rigid_zone.length_i - 125.0).abs() < 1e-9,
        "λ_i={}",
        model.elements[1].rigid_zone.length_i
    );

    model.elements[1].rigid_zone.source_i = ZoneSource::Manual;
    model.elements[1].rigid_zone.length_i = 999.0;
    model.elements[1].rigid_zone.face_i = Some(0.0);
    apply_auto_rigid_zones(&mut model, &RigidZoneRule::default());
    assert_eq!(
        model.elements[1].rigid_zone.length_i, 999.0,
        "Manual 端が上書きされた"
    );
    assert!(
        (model.elements[1].rigid_zone.face_i_or_zero() - 300.0).abs() < 1e-9,
        "Manual 端でも face_i は再算定されるべき: face_i={}",
        model.elements[1].rigid_zone.face_i_or_zero()
    );
}

/// 危険断面位置: face_i/face_j から評価断面リストを算定する。
/// face=0 の端では [0.0, 0.5, 1.0] と完全一致する。
#[test]
fn test_eval_sections_from_face_distance() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{ElementKind, RigidZone};

    let sec = Section {
        id: SectionId(0),
        name: String::new(),
        area: 100.0,
        iy: 1.0e6,
        iz: 1.0e6,
        j: 1.0e6,
        depth: 300.0,
        width: 300.0,
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
    };
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: String::new(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    };
    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [4000.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: squid_n_core::model::ForceRegime::Auto,
            rigid_zone: RigidZone {
                face_i: Some(300.0),
                face_j: Some(250.0),
                ..Default::default()
            },
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![sec],
        materials: vec![mat],
        ..Default::default()
    };

    let beam = BeamElement::new(&model.elements[0], &model);
    let expected = [0.0, 0.075, 0.5, 0.9375, 1.0];
    assert_eq!(beam.eval_sections.len(), expected.len());
    for (a, b) in beam.eval_sections.iter().zip(expected.iter()) {
        assert!(
            (a - b).abs() < 1e-9,
            "eval_sections={:?}",
            beam.eval_sections
        );
    }

    let mut model_zero = model.clone();
    model_zero.elements[0].rigid_zone = RigidZone::default();
    let beam_zero = BeamElement::new(&model_zero.elements[0], &model_zero);
    assert_eq!(beam_zero.eval_sections, vec![0.0, 0.5, 1.0]);

    use squid_n_core::model::{Haunch, JointKind, MemberDetailAttr, MemberJoint};
    let mut model_detail = model.clone();
    model_detail.member_detail_attrs.push(MemberDetailAttr {
        elem: ElemId(0),
        haunch_i: Some(Haunch {
            length: 700.0,
            depth_increase: 200.0,
            width_increase: 0.0,
        }),
        haunch_j: None,
        joints: vec![MemberJoint {
            distance: 3000.0,
            kind: JointKind::Site,
        }],
    });
    let beam_detail = BeamElement::new(&model_detail.elements[0], &model_detail);
    let expected_detail = [0.0, 0.075, 0.25, 0.5, 0.75, 0.9375, 1.0];
    assert_eq!(beam_detail.eval_sections.len(), expected_detail.len());
    for (a, b) in beam_detail.eval_sections.iter().zip(expected_detail.iter()) {
        assert!(
            (a - b).abs() < 1e-9,
            "eval_sections={:?}",
            beam_detail.eval_sections
        );
    }

    let beam_base = BeamElement::new(&model.elements[0], &model);
    assert_eq!(
        beam_base.local_stiffness().data,
        beam_detail.local_stiffness().data,
        "付帯情報の有無で剛性行列が変わってはならない"
    );
}

/// 剛域算定用の RC 配筋（本数・径は最小限のダミー値。断面性能の絶対値は無関係）。
fn simple_rc_rebar() -> squid_n_core::section_shape::RcRebar {
    use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};
    RcRebar {
        main_x: BarSet {
            count: 4,
            dia: 16.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 4,
            dia: 16.0,
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

/// S造仕口（柱・梁とも鋼材形状）: 直交する RC/SRC 系の柱（梁）が存在しないため、
/// 仕口部に接続する柱(梁)がすべてＳの場合は剛域長さ0（λ=0）になる。
#[test]
fn test_auto_rigid_zone_steel_joint_is_zero() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::ElementKind;
    use squid_n_core::section_shape::SectionShape;

    let col_sec = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    }
    .to_section(SectionId(0), "col-H400".to_string());
    let beam_sec = SectionShape::SteelH {
        height: 500.0,
        width: 200.0,
        web_thick: 10.0,
        flange_thick: 16.0,
    }
    .to_section(SectionId(1), "beam-H500".to_string());
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "steel".to_string(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(235.0),
    };

    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(2),
                coord: [4000.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                section: Some(SectionId(1)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![col_sec, beam_sec],
        materials: vec![mat],
        ..Default::default()
    };

    let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
    assert_eq!(
        zone.length_i, 0.0,
        "S造仕口の剛域長は0のはず: length_i={}",
        zone.length_i
    );
}

/// S梁 + RC柱（混在節点）: 剛域は設けない。
/// S 梁が 1 本でも集まる仕口は対象外である。
#[test]
fn test_auto_rigid_zone_steel_beam_rc_column() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::ElementKind;
    use squid_n_core::section_shape::SectionShape;

    let col_sec = SectionShape::RcRect {
        b: 400.0,
        d: 600.0,
        rebar: simple_rc_rebar(),
    }
    .to_section(SectionId(0), "col-RC600".to_string());
    let beam_sec = SectionShape::SteelH {
        height: 500.0,
        width: 200.0,
        web_thick: 10.0,
        flange_thick: 16.0,
    }
    .to_section(SectionId(1), "beam-H500".to_string());
    let rc_mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "concrete".to_string(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 0.0,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
    let s_mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(1),
        name: "steel".to_string(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(235.0),
    };

    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(2),
                coord: [4000.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                section: Some(SectionId(1)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![col_sec, beam_sec],
        materials: vec![rc_mat, s_mat],
        ..Default::default()
    };

    let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
    assert_eq!(
        zone.length_i, 0.0,
        "S梁が集まる仕口では剛域を設けない: λ_i={}",
        zone.length_i
    );
    assert!(
        (zone.face_i_or_zero() - 300.0).abs() < 1e-9,
        "face_i={} (期待値=柱せい/2=300)",
        zone.face_i_or_zero()
    );
}

/// RC梁 + S柱のみ: 剛域長は 0 になる。
#[test]
fn test_auto_rigid_zone_rc_beam_steel_column_only_is_zero() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::ElementKind;
    use squid_n_core::section_shape::SectionShape;

    let col_sec = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    }
    .to_section(SectionId(0), "col-H400".to_string());
    let beam_sec = SectionShape::RcRect {
        b: 400.0,
        d: 600.0,
        rebar: simple_rc_rebar(),
    }
    .to_section(SectionId(1), "beam-RC600".to_string());
    let s_mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "steel".to_string(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(235.0),
    };
    let rc_mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(1),
        name: "concrete".to_string(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 0.0,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };

    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(2),
                coord: [4000.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                section: Some(SectionId(1)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![col_sec, beam_sec],
        materials: vec![s_mat, rc_mat],
        ..Default::default()
    };

    let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
    assert_eq!(
        zone.length_i, 0.0,
        "RC梁+S柱のみ: 剛域長は0のはず（RC/SRC直交材がない）。length_i={}",
        zone.length_i
    );
}

/// 耐震壁要素（ElementKind::Wall）が節点に接続していても、直交せい探索の対象は
/// Beam 要素のみなので結果に影響しない（耐震壁周辺の柱・梁の剛域は
/// 考慮しない扱い）。壁を追加しても標準ケース（柱600・梁700 → λ=125）と同じ結果。
#[test]
fn test_auto_rigid_zone_wall_does_not_affect_orthogonal_search() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    let col_sec = Section {
        id: SectionId(0),
        name: "col".to_string(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 600.0,
        width: 0.0,
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
    };
    let beam_sec = Section {
        id: SectionId(1),
        name: "beam".to_string(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 700.0,
        width: 0.0,
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
    };
    let wall_sec = Section {
        id: SectionId(2),
        name: "wall".to_string(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 1000.0,
        width: 0.0,
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
    };
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "conc".to_string(),
        category: MaterialCategory::Concrete,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    };

    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(2),
                coord: [4000.0, 0.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(3),
                coord: [0.0, 4000.0, 3000.0],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        elements: vec![
            ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            ElementData {
                id: ElemId(1),
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![NodeId(1), NodeId(2)],
                section: Some(SectionId(1)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
            ElementData {
                id: ElemId(2),
                kind: ElementKind::Wall,
                nodes: smallvec::smallvec![NodeId(1), NodeId(3)],
                section: Some(SectionId(2)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![col_sec, beam_sec, wall_sec],
        materials: vec![mat],
        ..Default::default()
    };

    let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
    assert!(
        (zone.length_i - 125.0).abs() < 1e-9,
        "壁のせいが紛れ込んでいないはず: λ_i={}",
        zone.length_i
    );
    assert!(
        (zone.face_i_or_zero() - 300.0).abs() < 1e-9,
        "壁のせいが紛れ込んでいないはず: face_i={}",
        zone.face_i_or_zero()
    );
}

/// 壁エレメントモデルの上下大梁の剛性倍率（壁エレメント置換モデルの上下大梁の断面性能）。
/// 4節点 Wall 要素の下辺2節点を結ぶ水平梁は iy/a が既定倍率（100倍）になる。
#[test]
fn test_beam_new_wall_girder_bottom_edge_scales_stiffness() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{ElementData, ElementKind, ForceRegime, LocalAxis, Model};

    let sec = Section {
        id: SectionId(0),
        name: "beam".to_string(),
        area: 60000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e7,
        depth: 600.0,
        width: 300.0,
        as_y: 50000.0,
        as_z: 50000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "conc".to_string(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: None,
        fy: None,
    };
    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    };
    let nodes = vec![
        make_node(0, [0.0, 0.0, 0.0]),
        make_node(1, [4000.0, 0.0, 0.0]),
        make_node(2, [4000.0, 0.0, 3000.0]),
        make_node(3, [0.0, 0.0, 3000.0]),
    ];
    let beam_elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    let model_no_wall = Model {
        nodes: nodes.clone(),
        elements: vec![beam_elem.clone()],
        sections: vec![sec.clone()],
        materials: vec![mat.clone()],
        ..Default::default()
    };
    let beam_no_wall = BeamElement::new(&beam_elem, &model_no_wall);

    let wall_elem = ElementData {
        id: ElemId(1),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let edge = |id: u32, n0: u32, n1: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(n0), NodeId(n1)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let model_with_wall = Model {
        nodes,
        elements: vec![
            beam_elem.clone(),
            wall_elem,
            edge(2, 3, 2),
            edge(3, 0, 3),
            edge(4, 1, 2),
        ],
        sections: vec![sec],
        materials: vec![mat],
        ..Default::default()
    };
    let beam_with_wall = BeamElement::new(&beam_elem, &model_with_wall);

    assert!(
        (beam_with_wall.iy / beam_no_wall.iy - WALL_GIRDER_STIFF_FACTOR).abs() < 1e-9,
        "iy倍率が既定100倍でない: with={} without={}",
        beam_with_wall.iy,
        beam_no_wall.iy
    );
    assert!(
        (beam_with_wall.a / beam_no_wall.a - WALL_GIRDER_STIFF_FACTOR).abs() < 1e-9,
        "a倍率が既定100倍でない: with={} without={}",
        beam_with_wall.a,
        beam_no_wall.a
    );
    assert!(
        (beam_with_wall.a_mass - beam_no_wall.a_mass).abs() < 1e-9,
        "a_massは変更されないはず"
    );
}

/// 壁の節点を1つしか共有しない梁（壁の上辺・下辺ではない）には倍率が掛からない。
#[test]
fn test_beam_new_wall_girder_requires_both_nodes_shared() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{ElementData, ElementKind, ForceRegime, LocalAxis, Model};

    let sec = Section {
        id: SectionId(0),
        name: "beam".to_string(),
        area: 60000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e7,
        depth: 600.0,
        width: 300.0,
        as_y: 50000.0,
        as_z: 50000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "conc".to_string(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: None,
        fy: None,
    };
    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    };
    let nodes = vec![
        make_node(0, [0.0, 0.0, 0.0]),
        make_node(1, [4000.0, 0.0, 0.0]),
        make_node(2, [4000.0, 0.0, 3000.0]),
        make_node(3, [0.0, 0.0, 3000.0]),
        make_node(4, [8000.0, 0.0, 0.0]),
    ];
    let beam_elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(1), NodeId(4)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let wall_elem = ElementData {
        id: ElemId(1),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let model = Model {
        nodes,
        elements: vec![beam_elem.clone(), wall_elem],
        sections: vec![sec.clone()],
        materials: vec![mat],
        ..Default::default()
    };
    let beam = BeamElement::new(&beam_elem, &model);
    assert!(
        (beam.iy - sec.iy).abs() < 1e-9,
        "壁節点を1つしか共有しない梁には倍率が掛からないはず: iy={}",
        beam.iy
    );
}

/// 鉛直材（柱）は壁節点を2つ共有していても水平材ではないため倍率は掛からない。
#[test]
fn test_beam_new_wall_girder_vertical_member_not_scaled() {
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{ElementData, ElementKind, ForceRegime, LocalAxis, Model};

    let sec = Section {
        id: SectionId(0),
        name: "column".to_string(),
        area: 60000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e7,
        depth: 600.0,
        width: 300.0,
        as_y: 50000.0,
        as_z: 50000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "conc".to_string(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: None,
        fy: None,
    };
    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    };
    let nodes = vec![
        make_node(0, [0.0, 0.0, 0.0]),
        make_node(1, [4000.0, 0.0, 0.0]),
        make_node(2, [4000.0, 0.0, 3000.0]),
        make_node(3, [0.0, 0.0, 3000.0]),
    ];
    let column_elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let wall_elem = ElementData {
        id: ElemId(1),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let model = Model {
        nodes,
        elements: vec![column_elem.clone(), wall_elem],
        sections: vec![sec.clone()],
        materials: vec![mat],
        ..Default::default()
    };
    let column = BeamElement::new(&column_elem, &model);
    assert!(
        (column.iy - sec.iy).abs() < 1e-9,
        "鉛直材は水平材ではないため倍率が掛からないはず: iy={}",
        column.iy
    );
}

/// フレーム内雑壁の柱への袖壁算入。大開口の壁は耐震壁不成立となり、
/// 面内（iz・as_y）は平行軸の定理による合成値と一致し、面外（iy・as_z）は不変。
#[test]
fn test_beam_new_misc_wall_wing_augments_column_inplane_stiffness() {
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, ForceRegime, LocalAxis, Model, WallAttr, WallOpening,
    };
    use squid_n_core::section_shape::SectionShape;

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    };
    let col_sec = Section {
        id: SectionId(0),
        name: "col".into(),
        area: 90_000.0,
        iy: 3.0e9,
        iz: 2.0e9,
        j: 1.0e7,
        depth: 300.0,
        width: 300.0,
        as_y: 50_000.0,
        as_z: 60_000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let wall_shape = SectionShape::RcWall {
        thickness: 150.0,
        ps: 0.0025,
    };
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "FC24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
    let nodes = vec![
        make_node(0, [0.0, 0.0, 0.0]),
        make_node(1, [4000.0, 0.0, 0.0]),
        make_node(2, [4000.0, 0.0, 3000.0]),
        make_node(3, [0.0, 0.0, 3000.0]),
    ];
    let column_elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let wall_elem = ElementData {
        id: ElemId(1),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: Some(SectionId(1)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let mut model = Model {
        nodes,
        elements: vec![column_elem.clone(), wall_elem],
        sections: vec![
            col_sec.clone(),
            wall_shape.to_section(SectionId(1), "W150".into()),
        ],
        materials: vec![mat],
        ..Default::default()
    };
    let openings = vec![WallOpening {
        width: 2400.0,
        height: 1500.0,
        offset: Some([800.0, 750.0]),
    }];
    model.wall_attrs.push(WallAttr {
        elem: ElemId(1),
        opening_area: 0.0,
        opening_weight: 0.0,
        slit: Default::default(),
        openings: openings.clone(),
        finish_intensity: 0.0,
    });
    model.wall_plates.push(squid_n_core::model::WallPlate {
        id: squid_n_core::ids::WallPlateId(0),
        shape: squid_n_core::model::WallPlateShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        },
        section: Some(SectionId(1)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings,
        loads: vec![],
        slit: Default::default(),
    });

    let column = BeamElement::new(&column_elem, &model);

    let d_col: f64 = 300.0;
    let lww = 650.0_f64;
    let aw = 150.0 * lww;
    let ac = col_sec.area;
    let e_i = -(d_col / 2.0 + lww / 2.0);
    let g = (aw * e_i) / (ac + aw);
    let self_i = 150.0 * lww.powi(3) / 12.0;
    let expected_iz = col_sec.iy + ac * g * g + self_i + aw * (e_i - g).powi(2);

    assert!(
        (column.a - (ac + aw)).abs() < 1e-6,
        "a={} expected={}",
        column.a,
        ac + aw
    );
    assert!(
        (column.iz - expected_iz).abs() / expected_iz < 1e-9,
        "iz={} expected={}",
        column.iz,
        expected_iz
    );
    assert!(
        (column.as_y - (col_sec.as_z + aw / 1.2)).abs() < 1e-6,
        "as_y={}",
        column.as_y
    );
    assert!((column.iy - col_sec.iz).abs() < 1e-6, "iy={}", column.iy);
    assert!(
        (column.as_z - col_sec.as_y).abs() < 1e-6,
        "as_z={}",
        column.as_z
    );
}

/// 同じ大開口壁の下辺梁（節点0-1）への腰壁算入。鉛直曲げ（要素座標系では
/// iz・as_y）へ平行軸の定理で合成され、耐震壁不成立のため上下大梁100倍は掛からない。
#[test]
fn test_beam_new_misc_wall_strip_augments_girder_iy_without_100x() {
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, ForceRegime, LocalAxis, Model, WallAttr, WallOpening,
    };
    use squid_n_core::section_shape::SectionShape;

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    };
    let beam_sec = Section {
        id: SectionId(0),
        name: "beam".into(),
        area: 200_000.0,
        iy: 5.0e9,
        iz: 1.0e9,
        j: 1.0e7,
        depth: 600.0,
        width: 300.0,
        as_y: 70_000.0,
        as_z: 70_000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let wall_shape = SectionShape::RcWall {
        thickness: 150.0,
        ps: 0.0025,
    };
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "FC24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
    let nodes = vec![
        make_node(0, [0.0, 0.0, 0.0]),
        make_node(1, [4000.0, 0.0, 0.0]),
        make_node(2, [4000.0, 0.0, 3000.0]),
        make_node(3, [0.0, 0.0, 3000.0]),
    ];
    let beam_elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let wall_elem = ElementData {
        id: ElemId(1),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: Some(SectionId(1)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let mut model = Model {
        nodes,
        elements: vec![beam_elem.clone(), wall_elem],
        sections: vec![
            beam_sec.clone(),
            wall_shape.to_section(SectionId(1), "W150".into()),
        ],
        materials: vec![mat],
        ..Default::default()
    };
    let openings = vec![WallOpening {
        width: 2400.0,
        height: 1500.0,
        offset: Some([800.0, 750.0]),
    }];
    model.wall_attrs.push(WallAttr {
        elem: ElemId(1),
        opening_area: 0.0,
        opening_weight: 0.0,
        slit: Default::default(),
        openings: openings.clone(),
        finish_intensity: 0.0,
    });
    model.wall_plates.push(squid_n_core::model::WallPlate {
        id: squid_n_core::ids::WallPlateId(0),
        shape: squid_n_core::model::WallPlateShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        },
        section: Some(SectionId(1)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings,
        loads: vec![],
        slit: Default::default(),
    });

    let beam = BeamElement::new(&beam_elem, &model);

    let d_beam: f64 = 600.0;
    let hw = 450.0_f64;
    let aw = 150.0 * hw;
    let ac = beam_sec.area;
    let e_i = d_beam / 2.0 + hw / 2.0;
    let g = (aw * e_i) / (ac + aw);
    let self_i = 150.0 * hw.powi(3) / 12.0;
    let expected_iz = beam_sec.iy + ac * g * g + self_i + aw * (e_i - g).powi(2);

    assert!(
        (beam.a - (ac + aw)).abs() < 1e-6,
        "a={} expected={}",
        beam.a,
        ac + aw
    );
    assert!(
        (beam.iz - expected_iz).abs() / expected_iz < 1e-9,
        "iz={} expected={}",
        beam.iz,
        expected_iz
    );
    assert!(
        (beam.as_y - (beam_sec.as_z + aw / 1.2)).abs() < 1e-6,
        "as_y={}",
        beam.as_y
    );
    assert!(
        beam.iz < beam_sec.iy * 10.0,
        "100倍が誤って適用されている可能性: iz={} base={}",
        beam.iz,
        beam_sec.iy
    );
    assert!((beam.iy - beam_sec.iz).abs() < 1e-6, "iy={}", beam.iy);
    assert!(
        (beam.as_z - beam_sec.as_y).abs() < 1e-6,
        "as_z={}",
        beam.as_z
    );
}

/// 柱際スリットは、切れている側の柱への袖壁算入だけを落とし、梁への腰壁・垂れ壁
/// 算入は残す。
#[test]
fn test_column_face_slit_drops_wing_wall_but_keeps_girder_strip() {
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, ForceRegime, LocalAxis, Model, WallAttr, WallOpening,
    };
    use squid_n_core::section_shape::SectionShape;

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    };
    let col_sec = Section {
        id: SectionId(0),
        name: "C300x300".into(),
        area: 90000.0,
        iy: 6.75e8,
        iz: 6.75e8,
        j: 1.0e9,
        depth: 300.0,
        width: 300.0,
        as_y: 75000.0,
        as_z: 75000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let beam_sec = Section {
        id: SectionId(2),
        name: "G400x600".into(),
        area: 240000.0,
        iy: 7.2e9,
        iz: 3.2e9,
        j: 1.0e10,
        depth: 600.0,
        width: 400.0,
        as_y: 200000.0,
        as_z: 200000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let wall_shape = SectionShape::RcWall {
        thickness: 150.0,
        ps: 0.0025,
    };
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "FC24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
    let column_elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let beam_elem = ElementData {
        id: ElemId(2),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(2)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let wall_elem = ElementData {
        id: ElemId(1),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: Some(SectionId(1)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    let build = |slit: squid_n_core::model::WallSlit| -> Model {
        let openings = vec![WallOpening {
            width: 2400.0,
            height: 1500.0,
            offset: Some([800.0, 750.0]),
        }];
        let mut model = Model {
            nodes: vec![
                make_node(0, [0.0, 0.0, 0.0]),
                make_node(1, [4000.0, 0.0, 0.0]),
                make_node(2, [4000.0, 0.0, 3000.0]),
                make_node(3, [0.0, 0.0, 3000.0]),
            ],
            elements: vec![column_elem.clone(), wall_elem.clone(), beam_elem.clone()],
            sections: vec![
                col_sec.clone(),
                wall_shape.to_section(SectionId(1), "W150".into()),
                beam_sec.clone(),
            ],
            materials: vec![mat.clone()],
            ..Default::default()
        };
        model.wall_attrs.push(WallAttr {
            elem: ElemId(1),
            opening_area: 0.0,
            opening_weight: 0.0,
            slit,
            openings: openings.clone(),
            finish_intensity: 0.0,
        });
        model.wall_plates.push(squid_n_core::model::WallPlate {
            id: squid_n_core::ids::WallPlateId(0),
            shape: squid_n_core::model::WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            section: Some(SectionId(1)),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings,
            loads: vec![],
            slit,
        });
        model
    };

    use squid_n_core::model::WallSlit;
    let plain = build(WallSlit::default());
    let slit = build(WallSlit {
        column_face: [true, true],
        beam_face: [false, false],
    });

    let column_plain = BeamElement::new(&column_elem, &plain);
    let column_slit = BeamElement::new(&column_elem, &slit);
    let beam_plain = BeamElement::new(&beam_elem, &plain);
    let beam_slit = BeamElement::new(&beam_elem, &slit);

    assert!(
        column_plain.a > col_sec.area + 1.0,
        "スリット無しでは袖壁が算入される: a={}",
        column_plain.a
    );
    assert!(
        (column_slit.a - col_sec.area).abs() < 1e-6,
        "柱際が切れた側へは袖壁を算入しない: a={}",
        column_slit.a
    );
    assert!(
        (column_slit.iz - column_plain.iz).abs() / column_plain.iz > 1e-6,
        "袖壁の有無で面内剛性が変わる"
    );

    assert!(
        beam_plain.a > beam_sec.area + 1.0,
        "腰壁が算入される: a={}",
        beam_plain.a
    );
    assert!(
        (beam_slit.a - beam_plain.a).abs() < 1e-9,
        "柱際スリットは梁への腰壁算入を変えない: {} != {}",
        beam_slit.a,
        beam_plain.a
    );
    assert!(
        (beam_slit.iz - beam_plain.iz).abs() < 1e-6,
        "柱際スリットは梁の面内剛性を変えない"
    );

    let bottom_slit = build(WallSlit {
        column_face: [false, false],
        beam_face: [true, false],
    });
    let beam_bottom_slit = BeamElement::new(&beam_elem, &bottom_slit);
    assert!(
        (beam_bottom_slit.a - beam_sec.area).abs() < 1e-6,
        "下辺の梁際を切った側へは腰壁を算入しない: a={}",
        beam_bottom_slit.a
    );
    let column_bottom_slit = BeamElement::new(&column_elem, &bottom_slit);
    assert!(
        (column_bottom_slit.a - column_plain.a).abs() < 1e-9,
        "梁際スリットは柱の袖壁算入を変えない"
    );
}

/// 耐震壁が成立する壁の周辺部材: 柱・梁とも雑壁算入されず、
/// 上下大梁は100倍のままとなる。
#[test]
fn test_beam_new_seismic_wall_no_misc_wall_augmentation() {
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{ElementData, ElementKind, ForceRegime, LocalAxis, Model};
    use squid_n_core::section_shape::SectionShape;

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    };
    let col_sec = Section {
        id: SectionId(0),
        name: "col".into(),
        area: 90_000.0,
        iy: 3.0e9,
        iz: 2.0e9,
        j: 1.0e7,
        depth: 300.0,
        width: 300.0,
        as_y: 50_000.0,
        as_z: 60_000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let beam_sec = Section {
        id: SectionId(1),
        name: "beam".into(),
        area: 200_000.0,
        iy: 5.0e9,
        iz: 1.0e9,
        j: 1.0e7,
        depth: 600.0,
        width: 300.0,
        as_y: 70_000.0,
        as_z: 70_000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let wall_shape = SectionShape::RcWall {
        thickness: 150.0,
        ps: 0.0025,
    };
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "FC24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
    let nodes = vec![
        make_node(0, [0.0, 0.0, 0.0]),
        make_node(1, [4000.0, 0.0, 0.0]),
        make_node(2, [4000.0, 0.0, 3000.0]),
        make_node(3, [0.0, 0.0, 3000.0]),
    ];
    let column_elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(3)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let beam_elem = ElementData {
        id: ElemId(1),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(1)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let wall_elem = ElementData {
        id: ElemId(2),
        kind: ElementKind::Wall,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        section: Some(SectionId(2)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let edge = |id: u32, n0: u32, n1: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(n0), NodeId(n1)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let model = Model {
        nodes,
        elements: vec![
            column_elem.clone(),
            beam_elem.clone(),
            wall_elem,
            edge(3, 3, 2),
            edge(4, 1, 2),
        ],
        sections: vec![
            col_sec.clone(),
            beam_sec.clone(),
            wall_shape.to_section(SectionId(2), "W150".into()),
        ],
        materials: vec![mat],
        ..Default::default()
    };

    let column = BeamElement::new(&column_elem, &model);
    assert!(
        (column.iz - col_sec.iy).abs() < 1e-6,
        "耐震壁成立時は柱に袖壁算入されないはず: iz={}",
        column.iz
    );
    assert!((column.a - col_sec.area).abs() < 1e-6, "a={}", column.a);
    assert!(
        (column.as_y - col_sec.as_z).abs() < 1e-6,
        "as_y={}",
        column.as_y
    );

    let beam = BeamElement::new(&beam_elem, &model);
    assert!(
        (beam.iz / beam_sec.iy - WALL_GIRDER_STIFF_FACTOR).abs() < 1e-9,
        "耐震壁成立時は従来どおり上下大梁100倍のはず: iz={} base={}",
        beam.iz,
        beam_sec.iy
    );
    assert!(
        (beam.a / beam_sec.area - WALL_GIRDER_STIFF_FACTOR).abs() < 1e-9,
        "a={} base={}",
        beam.a,
        beam_sec.area
    );
}

/// 断面レイヤ→要素座標系のクロス変換の回帰テスト（軸名の取り違え防止）。
///
/// 断面レイヤは「iy=強軸（せい方向 D³ 系）・as_z=ウェブ」の規約だが、要素座標系は
/// せい方向＝ローカル y のため、梁の鉛直たわみ（uy、Mz 面）の剛性は断面の強軸値
/// iy・as_z で、水平たわみ（uz、My 面）は弱軸値 iz・as_y で組み立てられなければ
/// ならない。クロス変換（construct.rs）を外すと本テストが失敗する。
#[test]
fn test_vertical_bending_stiffness_uses_section_strong_axis() {
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::model::ForceRegime;

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    };
    let sec = Section {
        id: SectionId(0),
        name: "H-400x200".into(),
        area: 8_412.0,
        iy: 2.37e8,
        iz: 1.60e7,
        j: 5.0e5,
        depth: 400.0,
        width: 200.0,
        as_y: 5_200.0,
        as_z: 3_200.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SN400".into(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(235.0),
    };
    let elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let model = Model {
        nodes: vec![
            make_node(0, [0.0, 0.0, 0.0]),
            make_node(1, [6000.0, 0.0, 0.0]),
        ],
        elements: vec![elem.clone()],
        sections: vec![sec.clone()],
        materials: vec![mat],
        ..Default::default()
    };

    let beam = BeamElement::new(&elem, &model);
    assert!((beam.axis.rot[1][2] - 1.0).abs() < 1e-12);

    let k = beam.local_stiffness_raw();
    let (e, g, l) = (beam.e, beam.g, beam.length);

    let phi_v = 12.0 * e * sec.iy / (g * sec.as_z * l * l);
    let expected_v = 12.0 * e * sec.iy / ((1.0 + phi_v) * l.powi(3));
    assert!(
        (k.get(1, 1) - expected_v).abs() / expected_v < 1e-12,
        "鉛直たわみ剛性が強軸値で組まれていない: k11={} expected={}",
        k.get(1, 1),
        expected_v
    );

    let phi_h = 12.0 * e * sec.iz / (g * sec.as_y * l * l);
    let expected_h = 12.0 * e * sec.iz / ((1.0 + phi_h) * l.powi(3));
    assert!(
        (k.get(2, 2) - expected_h).abs() / expected_h < 1e-12,
        "水平たわみ剛性が弱軸値で組まれていない: k22={} expected={}",
        k.get(2, 2),
        expected_h
    );

    assert!(k.get(1, 1) > k.get(2, 2) * 5.0);
}

/// トライアル追従の回帰テスト: update_state(du, commit=false) が internal_force に
/// 反映され、commit_state で確定、revert_state / restore_state でロールバック
/// できること。
#[test]
fn test_beam_trial_displacement_tracking() {
    use crate::behavior::{Ctx, ElementBehavior, LocalVec};
    let mut beam = make_test_beam();
    let model = Model::default();
    let ctx = Ctx { model: &model };

    assert!(beam
        .internal_force(&ctx)
        .data
        .iter()
        .all(|v| v.abs() < 1e-12));

    let mut du = LocalVec {
        data: smallvec::SmallVec::from_elem(0.0, 12),
    };
    du.data[6] = 1.0;
    let snap = beam.snapshot_state();
    beam.update_state(&du, false, &ctx);

    let f = beam.internal_force(&ctx);
    let ea_over_l = beam.e * beam.a / beam.length;
    assert!(
        (f.data[6] - ea_over_l).abs() / ea_over_l < 1e-12,
        "f6={} expected EA/L={}",
        f.data[6],
        ea_over_l
    );

    beam.commit_state();
    assert!((beam.committed_disp[6] - 1.0).abs() < 1e-15);

    beam.update_state(&du, false, &ctx);
    assert!((beam.trial_disp[6] - 2.0).abs() < 1e-15);
    beam.revert_state();
    assert!((beam.trial_disp[6] - 1.0).abs() < 1e-15);

    beam.restore_state(&*snap);
    assert!(beam
        .internal_force(&ctx)
        .data
        .iter()
        .all(|v| v.abs() < 1e-12));
    assert!(beam.committed_disp.iter().all(|v| *v == 0.0));
    assert!(beam.trial_disp.iter().all(|v| *v == 0.0));
}

/// recover_forces の内力場が i/j 分岐（ξ=0.5）をまたいで連続・整合であること。
/// スパン荷重なしでは N/Qy/Qz/Mx は全断面で一定、Mz/My は
/// dMz/dx = Qy・dMy/dx = −Qz を満たす単一の線形場になる。
#[test]
fn test_recover_forces_moment_field_continuous_across_half() {
    let mut beam = make_test_beam();
    beam.j = 1.0e8;
    beam.eval_sections = vec![0.0, 0.25, 0.45, 0.5, 0.55, 0.75, 1.0];
    let u = [
        0.1, 2.0, -1.5, 0.004, 0.002, -0.003, -0.2, -1.0, 0.5, -0.002, 0.004, 0.001,
    ];
    let mf = beam.recover_forces(&u);
    let l = beam.length;
    let f0 = mf.at.first().unwrap().1;
    for &(xi, f) in &mf.at {
        for (k, name) in [(0, "N"), (1, "Qy"), (2, "Qz"), (3, "Mx")] {
            let tol = 1e-6 * f0[k].abs().max(1.0);
            assert!(
                (f[k] - f0[k]).abs() < tol,
                "xi={xi} {name}={} が一定でない (端={})",
                f[k],
                f0[k]
            );
        }
        let mz_expected = f0[5] + f0[1] * xi * l;
        let my_expected = f0[4] - f0[2] * xi * l;
        let tol_mz = 1e-6 * mz_expected.abs().max(1.0);
        let tol_my = 1e-6 * my_expected.abs().max(1.0);
        assert!(
            (f[5] - mz_expected).abs() < tol_mz,
            "xi={xi} Mz={} expected={mz_expected}",
            f[5]
        );
        assert!(
            (f[4] - my_expected).abs() < tol_my,
            "xi={xi} My={} expected={my_expected}",
            f[4]
        );
    }
}

/// 純曲げ（両端逆向き回転 θ, −θ・並進ゼロ）では Qy=0 で Mz が全断面一定になる。
/// たわみ形 v=θx(1−x/L) は v''=−2θ/L（上に凸＝上端引張）で、下端引張正の
/// 断面力規約では Mz = EI·v'' = −2EIθ/L（負）となる符号まで検証する。
#[test]
fn test_recover_forces_pure_bending_constant_negative_moment() {
    let mut beam = make_test_beam();
    beam.eval_sections = vec![0.0, 0.25, 0.45, 0.5, 0.55, 0.75, 1.0];
    let theta = 1.0e-3;
    let mut u = [0.0; 12];
    u[5] = theta;
    u[11] = -theta;
    let mf = beam.recover_forces(&u);
    let expected = -2.0 * beam.e * beam.iz * theta / beam.length;
    for &(xi, f) in &mf.at {
        assert!(
            (f[5] - expected).abs() < expected.abs() * 1e-6,
            "xi={xi} Mz={} expected={expected}",
            f[5]
        );
        assert!(f[1].abs() < 1e-6, "xi={xi} 純曲げで Qy={} が生じた", f[1]);
    }
}

/// 袖壁の偏心 e は壁の節点入力順に依存しないこと。
#[test]
fn test_misc_wall_wing_eccentricity_is_independent_of_wall_node_order() {
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, ForceRegime, LocalAxis, Model, WallAttr, WallOpening,
    };
    use squid_n_core::section_shape::SectionShape;

    let make_node = |id: u32, coord: [f64; 3]| Node {
        id: NodeId(id),
        coord,
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    };
    let col_sec = Section {
        id: SectionId(0),
        name: "col".into(),
        area: 90_000.0,
        iy: 3.0e9,
        iz: 2.0e9,
        j: 1.0e7,
        depth: 300.0,
        width: 300.0,
        as_y: 50_000.0,
        as_z: 60_000.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let wall_shape = SectionShape::RcWall {
        thickness: 150.0,
        ps: 0.0025,
    };
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "FC24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    };
    let nodes = vec![
        make_node(0, [0.0, 0.0, 0.0]),
        make_node(1, [4000.0, 0.0, 0.0]),
        make_node(2, [8000.0, 0.0, 0.0]),
        make_node(3, [0.0, 0.0, 3000.0]),
        make_node(4, [4000.0, 0.0, 3000.0]),
        make_node(5, [8000.0, 0.0, 3000.0]),
    ];
    let column_elem = ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(1), NodeId(4)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let wall = |id: u32, ns: [u32; 4]| ElementData {
        id: ElemId(id),
        kind: ElementKind::Wall,
        nodes: ns.iter().map(|n| NodeId(*n)).collect(),
        section: Some(SectionId(1)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let attr = |id: u32| WallAttr {
        elem: ElemId(id),
        opening_area: 0.0,
        opening_weight: 0.0,
        slit: Default::default(),
        finish_intensity: 0.0,
        openings: vec![WallOpening {
            width: 2400.0,
            height: 1500.0,
            offset: Some([800.0, 750.0]),
        }],
    };

    let build = |wall_b_nodes: [u32; 4]| -> BeamElement {
        let mut model = Model {
            nodes: nodes.clone(),
            elements: vec![
                column_elem.clone(),
                wall(1, [0, 1, 4, 3]),
                wall(2, wall_b_nodes),
            ],
            sections: vec![
                col_sec.clone(),
                wall_shape.to_section(SectionId(1), "W150".into()),
            ],
            materials: vec![mat.clone()],
            ..Default::default()
        };
        model.wall_attrs.push(attr(1));
        model.wall_attrs.push(attr(2));
        BeamElement::new(&column_elem, &model)
    };

    let normal = build([1, 2, 5, 4]);
    let flipped = build([2, 1, 4, 5]);

    assert!(
        (normal.iz - flipped.iz).abs() < normal.iz.abs().max(1.0) * 1e-9,
        "壁の節点入力順で iz が変わってはならない: {} vs {}",
        normal.iz,
        flipped.iz
    );
    assert!(
        (normal.a - flipped.a).abs() < normal.a * 1e-9,
        "断面積も入力順に依存しない: {} vs {}",
        normal.a,
        flipped.a
    );
}

/// ねじれ解放の検証用モデル。2 本の柱（節点 0→1・2→3）の柱頭を X 方向の大梁で
/// つないだ 1 スパン 1 層の骨組み。`split_x` を真にすると大梁を中間節点 4 で
/// 2 分割し、「柱のない・一直線の梁だけが集まる節点」を作る。
fn torsion_test_model(split_x: bool) -> Model {
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::model::ForceRegime;

    let mk_node = |id: u32, c: [f64; 3]| Node {
        id: NodeId(id),
        coord: c,
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    };
    let mk_member = |id: u32, a: u32, b: u32, vertical: bool| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(a), NodeId(b)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: if vertical {
                [1.0, 0.0, 0.0]
            } else {
                [0.0, 0.0, 1.0]
            },
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let mut nodes = vec![
        mk_node(0, [0.0, 0.0, 0.0]),
        mk_node(1, [0.0, 0.0, 3000.0]),
        mk_node(2, [6000.0, 0.0, 0.0]),
        mk_node(3, [6000.0, 0.0, 3000.0]),
    ];
    nodes[0].restraint = squid_n_core::dof::Dof6Mask::FIXED;
    nodes[2].restraint = squid_n_core::dof::Dof6Mask::FIXED;
    let mut elements = vec![mk_member(0, 0, 1, true), mk_member(1, 2, 3, true)];
    if split_x {
        nodes.push(mk_node(4, [3000.0, 0.0, 3000.0]));
        elements.push(mk_member(2, 1, 4, false));
        elements.push(mk_member(3, 4, 3, false));
    } else {
        elements.push(mk_member(2, 1, 3, false));
    }
    Model {
        nodes,
        elements,
        sections: vec![Section {
            id: SectionId(0),
            name: "H".into(),
            area: 8000.0,
            iy: 1.0e8,
            iz: 2.0e8,
            j: 1.0e6,
            depth: 400.0,
            width: 200.0,
            as_y: 3000.0,
            as_z: 3000.0,
            floor: None,
            panel_thickness: None,
            thickness: None,
            shape: None,
            material: Some(MaterialId(0)),
            rebar_material: None,
            shear_rebar_material: None,
            steel_material: None,
        }],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "S".into(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(235.0),
        }],
        ..Default::default()
    }
}

/// 水平材（梁）は i 端ねじれが解放され、局所剛性のねじり行・列が 0 になる。
#[test]
fn test_beam_i_end_torsion_released_by_default() {
    let model = torsion_test_model(false);
    let beam = BeamElement::new(&model.elements[2], &model);
    assert!(beam.torsion_release[0], "梁の i 端ねじれが解放されていない");
    assert!(!beam.torsion_release[1], "j 端は解放しない");
    let k = beam.local_stiffness();
    for i in 0..12 {
        for &r in &[3usize, 9] {
            assert_eq!(
                k.get(r, i),
                0.0,
                "ねじれ解放後の局所剛性 K[{r}][{i}] が 0 でない"
            );
            assert_eq!(k.get(i, r), 0.0, "K[{i}][{r}] が 0 でない");
        }
    }

    let column = BeamElement::new(&model.elements[0], &model);
    assert!(column.torsion_release[0], "柱の i 端ねじれも解放される");
    let kc = column.local_stiffness();
    for i in 0..12 {
        for &r in &[3usize, 9] {
            assert_eq!(kc.get(r, i), 0.0, "柱のねじり行 K[{r}][{i}] が 0 でない");
        }
    }
    let raw = column.local_stiffness_raw();
    approx::assert_relative_eq!(
        raw.get(3, 3),
        column.g * column.j / column.length,
        max_relative = 1e-12
    );
}

/// 柱を中間節点で分割し、その節点に梁が取り付かない場合は、材軸（鉛直）まわりの
/// 回転を拘束するものがないため解放しない（梁の中間分割点と同じ規則）。
#[test]
fn test_column_torsion_release_skipped_at_collinear_column_node() {
    use squid_n_core::ids::SectionId;
    use squid_n_core::model::ForceRegime;
    let mut model = torsion_test_model(false);
    model.nodes.push(Node {
        id: NodeId(model.nodes.len() as u32),
        coord: [0.0, 0.0, 1500.0],
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    });
    let mid = NodeId(model.nodes.len() as u32 - 1);
    model.elements[0].nodes = smallvec::smallvec![NodeId(0), mid];
    let upper = ElemId(model.elements.len() as u32);
    model.elements.push(ElementData {
        id: upper,
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![mid, NodeId(1)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });
    let lower = BeamElement::new(&model.elements[0], &model);
    let upper_el = BeamElement::new(model.elements.last().expect("追加済み"), &model);
    assert!(
        !lower.torsion_release[0] && !upper_el.torsion_release[0],
        "鉛直材だけが集まる中間節点を持つ柱は解放してはならない"
    );
}

/// 柱がなく一直線の梁だけが集まる節点（大梁の中間分割点）では、ねじれを解放すると
/// 材軸まわり回転が浮いて剛性行列が特異になるため、解放しない。
#[test]
fn test_i_end_torsion_release_skipped_at_collinear_beam_node() {
    let model = torsion_test_model(true);
    let seg_a = BeamElement::new(&model.elements[2], &model);
    let seg_b = BeamElement::new(&model.elements[3], &model);
    assert!(
        !seg_a.torsion_release[0] && !seg_b.torsion_release[0],
        "ねじれ回転が浮く節点を持つ梁は解放してはならない"
    );
    let k = seg_a.local_stiffness();
    approx::assert_relative_eq!(
        k.get(3, 3),
        seg_a.g * seg_a.j / seg_a.length,
        max_relative = 1e-12
    );

    assert!(!seg_a.torsion_release[0]);
}

/// `BeamTorsionMode::Keep` ではねじり剛性を保持する（ねじりで釣り合わせる
/// モデル化のための切替）。
#[test]
fn test_beam_torsion_mode_keep_retains_torsion() {
    let mut model = torsion_test_model(false);
    model.beam_torsion = squid_n_core::model::BeamTorsionMode::Keep;
    let beam = BeamElement::new(&model.elements[2], &model);
    assert!(!beam.torsion_release[0]);
    let k = beam.local_stiffness();
    approx::assert_relative_eq!(
        k.get(3, 3),
        beam.g * beam.j / beam.length,
        max_relative = 1e-12
    );
}

/// ねじり剛性がない部材（J≤0）の rx は端条件がピンでも解放しない。解放しても
/// 静縮約の `Kbb` が特異になり縮約の意味がないため（ファイバー梁
/// `resolve_end_releases` と同じ規則。特異な `Kbb` は `invert_small` が `None` を
/// 返し補正項が省略される）。
#[test]
fn test_pinned_ends_without_torsion_keep_finite_stiffness() {
    let mut beam = make_test_beam();
    beam.end_cond = [EndCondition::Pinned, EndCondition::Pinned];
    let k = beam.local_stiffness();
    for i in 0..12 {
        for j in 0..12 {
            assert!(
                k.get(i, j).is_finite(),
                "両端ピン・J=0 の局所剛性 K[{i}][{j}] が有限でない: {}",
                k.get(i, j)
            );
        }
    }
    assert_eq!(k.get(3, 3), 0.0);
}

/// 剛域の適用条件・重なり処理のテスト用に、柱 2 本＋梁 1 本の門型を作る。
///
/// 節点 0(0,0,0)→1(0,0,3000) と 3(span,0,0)→2(span,0,3000) が柱、1→2 が梁。
/// 梁の両端に柱が付くので、両端の剛域長が算定される。材種は断面ごとに与える
/// 材料の区分で決まる（`squid_n_core::structure_kind`）。
fn t_joint_model(
    col_depth: f64,
    beam_depth: f64,
    span: f64,
    col_is_steel: bool,
    beam_is_steel: bool,
) -> Model {
    use squid_n_core::ids::{MaterialId, SectionId};

    let mk_mat = |id: u32, steel: bool| Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(id),
        name: String::new(),
        category: if steel {
            MaterialCategory::Steel
        } else {
            MaterialCategory::Concrete
        },
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    };
    let mk_sec = |id: u32, depth: f64, mat: u32| Section {
        id: SectionId(id),
        name: String::new(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(mat)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let mk_node = |id: u32, c: [f64; 3]| Node {
        id: NodeId(id),
        coord: c,
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    };
    let mk_elem = |id: u32, a: u32, b: u32, sec: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(a), NodeId(b)],
        section: Some(SectionId(sec)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: squid_n_core::model::ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    Model {
        nodes: vec![
            mk_node(0, [0.0, 0.0, 0.0]),
            mk_node(1, [0.0, 0.0, 3000.0]),
            mk_node(2, [span, 0.0, 3000.0]),
            mk_node(3, [span, 0.0, 0.0]),
        ],
        elements: vec![
            mk_elem(0, 0, 1, 0),
            mk_elem(1, 1, 2, 1),
            mk_elem(2, 3, 2, 0),
        ],
        sections: vec![mk_sec(0, col_depth, 0), mk_sec(1, beam_depth, 1)],
        materials: vec![mk_mat(0, col_is_steel), mk_mat(1, beam_is_steel)],
        ..Default::default()
    }
}

/// 剛域を設けるのは、節点に集合する柱・大梁がすべて RC/SRC のときだけ。
/// 1 本でも S 系があればその端の剛域は 0 になる。
#[test]
fn test_auto_rigid_zone_only_when_all_members_are_rc() {
    use squid_n_core::ids::ElemId;

    let all_rc = t_joint_model(600.0, 700.0, 4000.0, false, false);
    let zone = auto_rigid_zones(&all_rc, ElemId(1), &RigidZoneRule::default());
    assert!((zone.length_i - 125.0).abs() < 1e-9, "λ={}", zone.length_i);

    let steel_col = t_joint_model(600.0, 700.0, 4000.0, true, false);
    let zone = auto_rigid_zones(&steel_col, ElemId(1), &RigidZoneRule::default());
    assert_eq!(zone.length_i, 0.0, "S 柱が集まる節点では剛域を設けない");
    assert!(
        (zone.face_i_or_zero() - 300.0).abs() < 1e-9,
        "剛域が 0 でもフェース距離は付く: {}",
        zone.face_i_or_zero()
    );

    let steel_beam = t_joint_model(600.0, 700.0, 4000.0, false, true);
    let zone = auto_rigid_zones(&steel_beam, ElemId(1), &RigidZoneRule::default());
    assert_eq!(zone.length_i, 0.0, "S 梁自身にも剛域を設けない");

    let all_steel = t_joint_model(600.0, 700.0, 4000.0, true, true);
    let zone = auto_rigid_zones(&all_steel, ElemId(1), &RigidZoneRule::default());
    assert_eq!(zone.length_i, 0.0);
}

/// 両端の剛域長の合計が材長以上になる短い部材は、材長の中点から部材せいの
/// 1/4 の距離までを剛域とする。
#[test]
fn test_auto_rigid_zone_clamps_when_zones_overlap() {
    use squid_n_core::ids::ElemId;

    let model = t_joint_model(2000.0, 400.0, 1000.0, false, false);
    let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
    assert!(
        (zone.length_i - 400.0).abs() < 1e-9 && (zone.length_j - 400.0).abs() < 1e-9,
        "λi={} λj={}",
        zone.length_i,
        zone.length_j
    );
    assert!(zone.length_i + zone.length_j < 1000.0);

    let deep = t_joint_model(2000.0, 4000.0, 1000.0, false, false);
    let zone = auto_rigid_zones(&deep, ElemId(1), &RigidZoneRule::default());
    assert_eq!(zone.length_i, 0.0);
    assert_eq!(zone.length_j, 0.0);
}

/// 柱に袖壁が取り付く門型（柱 A の右側だけに壁）。
///
/// 節点 0(0,0,0)–1(0,0,3000) が柱 A、3(4000,0,0)–2(4000,0,3000) が柱 B、
/// 1–2 が梁。壁は柱 A から +X 方向へ 1000 mm 伸びる（節点 4/5 を追加）。
/// 柱・梁・壁ともコンクリート系。
fn portal_with_wing_wall(col_depth: f64, beam_depth: f64, wall_thickness: f64) -> Model {
    use squid_n_core::ids::{MaterialId, SectionId};

    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: String::new(),
        category: MaterialCategory::Concrete,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    };
    let mk_sec = |id: u32, depth: f64, thickness: Option<f64>| Section {
        id: SectionId(id),
        name: String::new(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let mk_node = |id: u32, c: [f64; 3]| Node {
        id: NodeId(id),
        coord: c,
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    };
    let mk_beam = |id: u32, a: u32, b: u32, sec: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(a), NodeId(b)],
        section: Some(SectionId(sec)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: squid_n_core::model::ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };

    Model {
        nodes: vec![
            mk_node(0, [0.0, 0.0, 0.0]),
            mk_node(1, [0.0, 0.0, 3000.0]),
            mk_node(2, [4000.0, 0.0, 3000.0]),
            mk_node(3, [4000.0, 0.0, 0.0]),
            mk_node(4, [1000.0, 0.0, 0.0]),
            mk_node(5, [1000.0, 0.0, 3000.0]),
        ],
        elements: vec![
            mk_beam(0, 0, 1, 0),
            mk_beam(1, 1, 2, 1),
            mk_beam(2, 3, 2, 0),
            ElementData {
                id: ElemId(3),
                kind: ElementKind::Wall,
                nodes: smallvec::smallvec![NodeId(0), NodeId(4), NodeId(5), NodeId(1)],
                section: Some(SectionId(2)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: squid_n_core::model::ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            },
        ],
        sections: vec![
            mk_sec(0, col_depth, None),
            mk_sec(1, beam_depth, None),
            mk_sec(2, 0.0, Some(wall_thickness)),
        ],
        materials: vec![mat],
        wall_plates: vec![squid_n_core::model::WallPlate {
            id: squid_n_core::ids::WallPlateId(0),
            shape: squid_n_core::model::WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(4), NodeId(5), NodeId(1)],
            },
            section: Some(SectionId(2)),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: vec![],
            loads: vec![],
            slit: Default::default(),
        }],
        ..Default::default()
    }
}

/// 剛域長は、取り付く壁の分だけ長くなる。
/// Lf = 300 + 200 = 500、λ = 500 − 700/4 = 325 となる。
#[test]
fn test_auto_rigid_zone_considers_attached_wall() {
    use squid_n_core::ids::ElemId;

    let model = portal_with_wing_wall(600.0, 700.0, 150.0);
    let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
    assert!(
        (zone.length_i - 325.0).abs() < 1e-9,
        "袖壁側の λ_i={}（期待値 325）",
        zone.length_i
    );
    assert!(
        (zone.face_i_or_zero() - 300.0).abs() < 1e-9,
        "フェース距離に壁を含めない: face_i={}（期待値 300）",
        zone.face_i_or_zero()
    );

    assert!(
        (zone.length_j - 125.0).abs() < 1e-9,
        "壁のない側の λ_j={}（期待値 125）",
        zone.length_j
    );
    assert!(
        (zone.face_j_or_zero() - 300.0).abs() < 1e-9,
        "face_j={}",
        zone.face_j_or_zero()
    );
}

/// 壁厚が 100 mm 未満の壁は剛域算定の対象外。
#[test]
fn test_auto_rigid_zone_ignores_thin_wall() {
    use squid_n_core::ids::ElemId;

    let model = portal_with_wing_wall(600.0, 700.0, 90.0);
    let zone = auto_rigid_zones(&model, ElemId(1), &RigidZoneRule::default());
    assert!(
        (zone.length_i - 125.0).abs() < 1e-9,
        "厚さ 90mm の壁は考慮しない: λ_i={}",
        zone.length_i
    );
}

/// 「壁を考慮する」を無効にすると原断面だけで算定する。
#[test]
fn test_auto_rigid_zone_wall_consideration_can_be_disabled() {
    use squid_n_core::ids::ElemId;

    let model = portal_with_wing_wall(600.0, 700.0, 150.0);
    let rule = RigidZoneRule {
        consider_walls: false,
    };
    let zone = auto_rigid_zones(&model, ElemId(1), &rule);
    assert!(
        (zone.length_i - 125.0).abs() < 1e-9,
        "原断面のみ: λ_i={}（期待値 125）",
        zone.length_i
    );
    assert!(
        (zone.face_i_or_zero() - 300.0).abs() < 1e-9,
        "face_i={}",
        zone.face_i_or_zero()
    );
}
