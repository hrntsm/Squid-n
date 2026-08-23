//! 取り付き版・版なし囲まれに対する梁のスラブ取付き判定。
use smallvec::SmallVec;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, FloorRegionId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, FloorRegion, ForceRegime, LoadTransfer, LocalAxis,
    Material, MaterialCategory, Model, Node, RegionAnchor, RegionShape, SlabPlate,
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

fn t_shape_model(regions: Vec<FloorRegion>) -> Model {
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
        floor_regions: regions,
        slab_thickness: 150.0,
        ..Default::default()
    }
}

/// 取り付き版ありは辺 0 の梁 true、直交梁 false。版なし囲まれは false。
#[test]
fn test_beam_has_attached_slab_t_shape_and_plateless() {
    let attached = t_shape_model(vec![FloorRegion {
        id: FloorRegionId(0),
        name: "バルコニー".into(),
        shape: RegionShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(0), NodeId(1)],
                span: [0.0, 1.0],
                transfer: LoadTransfer::Anchor,
            },
            extent: [1500.0, 1500.0],
        },
        plate: Some(SlabPlate {
            section: Some(SectionId(1)),
            ..Default::default()
        }),
        secondary_joist_ids: vec![],
    }]);
    assert!(
        beam_has_attached_slab(&attached, &attached.elements[0]),
        "取付き辺 0 の梁"
    );
    assert!(
        !beam_has_attached_slab(&attached, &attached.elements[1]),
        "直交梁"
    );

    let plateless = t_shape_model(vec![FloorRegion::enclosed(
        FloorRegionId(0),
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
    )]);
    assert!(
        !beam_has_attached_slab(&plateless, &plateless.elements[0]),
        "版なし囲まれ"
    );
}
