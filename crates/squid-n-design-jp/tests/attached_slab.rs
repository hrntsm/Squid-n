//! 取り付く床板・床板のない囲まれた床領域に対する梁のスラブ取付き判定。
use smallvec::SmallVec;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, SlabId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LoadTransfer, LocalAxis, Material,
    MaterialCategory, Model, Node, RegionAnchor, Slab, SlabPlate, SlabShape,
};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};
use squid_n_design_jp::beam_has_attached_slab;

fn node(id: u32, c: [f64; 3]) -> Node {
    Node {
        id: NodeId(id),
        coord: c,
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    }
}

fn beam(id: u32, a: u32, b: u32) -> ElementData {
    ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: {
            let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
            v.push(NodeId(a));
            v.push(NodeId(b));
            v
        },
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    }
}

fn t_shape_model(slabs: Vec<Slab>) -> Model {
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
    let mut sec = shape.to_section(SectionId(0), "G1".into());
    sec.material = Some(MaterialId(0));
    let mut slab = SectionShape::RcSlab { thickness: 150.0 }.to_section(SectionId(1), "S15".into());
    slab.material = Some(MaterialId(0));
    Model {
        nodes: vec![
            node(0, [0.0, 0.0, 3000.0]),
            node(1, [6000.0, 0.0, 3000.0]),
            node(2, [6000.0, 2500.0, 3000.0]),
            node(3, [0.0, 2500.0, 3000.0]),
        ],
        elements: vec![beam(0, 0, 1), beam(1, 1, 2), beam(2, 2, 3), beam(3, 3, 0)],
        sections: vec![sec, slab],
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
        slabs,
        slab_thickness: 150.0,
        ..Default::default()
    }
}

/// 取り付く床板ありは辺 0 の梁 true、直交梁 false。床板のない囲まれは false。
#[test]
fn test_beam_has_attached_slab_t_shape_and_plateless() {
    let attached = t_shape_model(vec![Slab {
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
            section: Some(SectionId(1)),
            ..Default::default()
        },
    }]);
    assert!(
        beam_has_attached_slab(&attached, &attached.elements[0]),
        "取付き辺 0 の梁"
    );
    assert!(
        !beam_has_attached_slab(&attached, &attached.elements[1]),
        "直交梁"
    );

    let plateless = t_shape_model(vec![]);
    assert!(
        !beam_has_attached_slab(&plateless, &plateless.elements[0]),
        "床板のない囲まれ"
    );
}

/// 床領域（大梁の1区画）が小梁で複数の床板へ細分されていても、床領域の外周を走る
/// 大梁のスラブ取付き判定は効く（判定を `FloorRegion::boundary` 優先にしているため。
/// 個々の床板の境界だけで判定すると、大梁の両端が別々の床板にまたがり、
/// どちらの床板にも「両端を含む」が成立しなくなる回帰が起きる）。
#[test]
fn test_beam_has_attached_slab_survives_joist_subdivided_region() {
    use squid_n_core::ids::FloorRegionId;
    use squid_n_core::model::{DistributionMethod, FloorRegion};

    let mut model = t_shape_model(vec![]);
    // 6000×2500 の床領域（節点 0-1-2-3）を、中央の小梁（節点 4-5）で 2 枚の床板へ
    // 細分する。床領域の外周を走る大梁（節点 0→1）の両端は、どちらの床板の境界にも
    // 同時には含まれない。
    model.nodes.push(node(4, [6000.0, 1250.0, 3000.0]));
    model.nodes.push(node(5, [0.0, 1250.0, 3000.0]));
    let plate = SlabPlate {
        section: Some(SectionId(1)),
        method: DistributionMethod::TriTrapezoid,
        ..Default::default()
    };
    model.slabs = vec![
        Slab {
            id: SlabId(0),
            shape: SlabShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(4), NodeId(5)],
            },
            plate: plate.clone(),
        },
        Slab {
            id: SlabId(1),
            shape: SlabShape::Enclosed {
                boundary: vec![NodeId(5), NodeId(4), NodeId(2), NodeId(3)],
            },
            plate,
        },
    ];
    model.floor_regions = vec![FloorRegion {
        slab_ids: vec![SlabId(0), SlabId(1)],
        ..FloorRegion::new(
            FloorRegionId(0),
            vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        )
    }];

    assert!(
        beam_has_attached_slab(&model, &model.elements[0]),
        "床領域が細分されていても大梁（節点 0-1）のスラブ取付きは判定できるはず"
    );
}
