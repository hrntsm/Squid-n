//! 数量積算のモデル走査テスト。

use smallvec::SmallVec;
use squid_n_core::model::{FloorRegion, Slab, SlabPlate, SlabShape};

use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, FloorRegionId, MaterialId, NodeId, SectionId, SlabId};
use squid_n_core::model::{
    DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material,
    MaterialCategory, Model, Node, RigidZone, Section,
};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

use super::*;

fn node(id: u32, x: f64, y: f64, z: f64) -> Node {
    Node {
        id: NodeId(id),
        coord: [x, y, z],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    }
}

fn line_elem(id: u32, n0: u32, n1: u32, sec: u32) -> ElementData {
    ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: {
            let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
            v.push(NodeId(n0));
            v.push(NodeId(n1));
            v
        },
        section: Some(SectionId(sec)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    }
}

fn rc_rebar() -> RcRebar {
    RcRebar {
        main_x: BarSet {
            count: 8,
            dia: 25.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 0,
            dia: 25.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 200.0,
            legs: 2,
        },
    }
}

fn rc_girder_section(id: u32) -> Section {
    let shape = SectionShape::RcRect {
        b: 400.0,
        d: 800.0,
        rebar: rc_rebar(),
    };
    with_rc_materials(shape.to_section(SectionId(id), format!("G{id}")))
}

/// 材料は断面が持つ。RC 断面へ主材料（コンクリート 0）と鉄筋（1）を割り当てる。
fn with_rc_materials(mut sec: Section) -> Section {
    sec.material = Some(MaterialId(0));
    sec.rebar_material = Some(MaterialId(1));
    sec.shear_rebar_material = Some(MaterialId(1));
    sec
}

/// 鉄筋の材料（材料名がグレード名、`fy` が降伏点）。
fn rebar_material(id: u32) -> Material {
    Material {
        strength_factor: None,
        id: MaterialId(id),
        name: "SD345".to_string(),
        category: MaterialCategory::Rebar,
        young: 205_000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(345.0),
        concrete_class: Default::default(),
    }
}

fn rc_column_section(id: u32) -> Section {
    let rebar = RcRebar {
        main_x: BarSet {
            count: 6,
            dia: 25.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 4,
            dia: 25.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 13.0,
            pitch: 100.0,
            legs: 2,
        },
    };
    let shape = SectionShape::RcRect {
        b: 700.0,
        d: 700.0,
        rebar,
    };
    with_rc_materials(shape.to_section(SectionId(id), format!("C{id}")))
}

fn rc_material(id: u32) -> Material {
    Material {
        strength_factor: None,
        id: MaterialId(id),
        name: "Fc24".to_string(),
        category: MaterialCategory::Concrete,
        young: 22_700.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
        concrete_class: Default::default(),
    }
}

fn steel_material(id: u32) -> Material {
    Material {
        strength_factor: None,
        id: MaterialId(id),
        name: "SN400B".to_string(),
        category: MaterialCategory::Steel,
        young: 205_000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(235.0),
        concrete_class: Default::default(),
    }
}

/// 2 柱＋上部大梁＋基礎梁の 1 スパン RC ラーメン。
fn rc_portal_model() -> Model {
    let nodes = vec![
        node(0, 0.0, 0.0, 0.0),
        node(1, 6_000.0, 0.0, 0.0),
        node(2, 0.0, 0.0, 3_500.0),
        node(3, 6_000.0, 0.0, 3_500.0),
    ];
    let mut girder = line_elem(2, 2, 3, 0);
    girder.rigid_zone.face_i = Some(350.0);
    girder.rigid_zone.face_j = Some(350.0);
    let mut fg = line_elem(3, 0, 1, 0);
    fg.rigid_zone.face_i = Some(350.0);
    fg.rigid_zone.face_j = Some(350.0);
    let elements = vec![
        line_elem(0, 0, 2, 1), // 柱
        line_elem(1, 1, 3, 1), // 柱
        girder,                // 大梁
        fg,                    // 基礎梁
    ];
    Model {
        nodes,
        elements,
        sections: vec![rc_girder_section(0), rc_column_section(1)],
        materials: vec![rc_material(0), rebar_material(1)],
        ..Default::default()
    }
}

#[test]
fn test_rc_portal_categories_and_concrete() {
    let model = rc_portal_model();
    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    assert_eq!(q.items.len(), 4);

    let cols: Vec<_> = q
        .items
        .iter()
        .filter(|i| i.category == MemberCategory::Column)
        .collect();
    let girders: Vec<_> = q
        .items
        .iter()
        .filter(|i| i.category == MemberCategory::Girder)
        .collect();
    let fgs: Vec<_> = q
        .items
        .iter()
        .filter(|i| i.category == MemberCategory::FoundationGirder)
        .collect();
    assert_eq!(cols.len(), 2);
    assert_eq!(girders.len(), 1);
    assert_eq!(fgs.len(), 1);

    // 柱: 0.7×0.7×3.5 = 1.715 m³、型枠 2×(0.7+0.7)×3.5 = 9.8 m²
    assert!((cols[0].concrete_m3 - 1.715).abs() < 1e-9);
    assert!((cols[0].formwork_m2 - 9.8).abs() < 1e-9);

    // 大梁: 内法 L=6000−350×2=5300 → 0.4×0.8×5.3 = 1.696 m³
    assert!((girders[0].concrete_m3 - 1.696).abs() < 1e-9);
    // 型枠（スラブなし）: 側面 0.8×2×5.3 + 底面 0.4×5.3 = 10.6 m²
    assert!((girders[0].formwork_m2 - (0.8 * 2.0 + 0.4) * 5.3).abs() < 1e-9);

    // 基礎梁の型枠: 側面 1 面＋底面 = (0.8+0.4)×5.3 = 6.36 m²
    assert!((fgs[0].formwork_m2 - 1.2 * 5.3).abs() < 1e-9);
}

#[test]
fn test_rc_girder_main_bars_and_stirrups() {
    let model = rc_portal_model();
    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let girder = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Girder)
        .unwrap();

    // 主筋: 両端とも連続梁なし → 両外端（タイプ7a）
    // 1 本長さ = 5300 + 35×25×2 = 7050mm、8 本 → 56.4 m
    let main = girder
        .rebar
        .iter()
        .find(|r| r.usage == RebarUsage::MainBar)
        .unwrap();
    assert!((main.total_length_m - 8.0 * 7.05).abs() < 1e-9);
    // D25 = 3.98 kg/m → 56.4 m = 224.472 kg
    assert!((main.weight_t - 8.0 * 7.05 * 3.98 / 1_000.0).abs() < 1e-9);

    // スターラップ: 一組 2×400+2×800 = 2400、本数 5300/200 = 26.5
    let st = girder
        .rebar
        .iter()
        .find(|r| r.usage == RebarUsage::Stirrup)
        .unwrap();
    assert!((st.total_length_m - 2.4 * 26.5).abs() < 1e-9);

    // 継手: 8 本 × (0.5 + 0.5×floor(5.3/5)) = 8 × 1.0
    assert!((girder.rebar_joints - 8.0).abs() < 1e-9);
}

#[test]
fn test_rc_column_rebar() {
    let model = rc_portal_model();
    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let col = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Column)
        .unwrap();

    // 主筋: X 6 本 + Y 4 本、各 H=3.5m
    let total_main_len: f64 = col
        .rebar
        .iter()
        .filter(|r| r.usage == RebarUsage::MainBar)
        .map(|r| r.total_length_m)
        .sum();
    assert!((total_main_len - 10.0 * 3.5).abs() < 1e-9);

    // フープ: 一組 2×700+2×700 = 2800、本数 3500/100 = 35 → 98 m
    let hoop = col
        .rebar
        .iter()
        .find(|r| r.usage == RebarUsage::Hoop)
        .unwrap();
    assert!((hoop.total_length_m - 2.8 * 35.0).abs() < 1e-9);

    // 継手: 10 本 × 1 個所（階高 < 7m）
    assert!((col.rebar_joints - 10.0).abs() < 1e-9);
}

#[test]
fn test_girder_formwork_slab_deduction() {
    // 大梁の両側にスラブが取り付く場合、側面せいからスラブ厚を控除する。
    let mut model = rc_portal_model();
    model.slab_thickness = 150.0;
    let mut slab_sec =
        SectionShape::RcSlab { thickness: 150.0 }.to_section(SectionId(2), "S15".into());
    slab_sec.material = Some(MaterialId(0));
    model.sections.push(slab_sec);
    // 大梁（節点 2-3）の両側（y=+5000 と y=−5000）に床板を配置する。
    model.nodes.push(node(4, 0.0, 5_000.0, 3_500.0));
    model.nodes.push(node(5, 6_000.0, 5_000.0, 3_500.0));
    model.nodes.push(node(6, 0.0, -5_000.0, 3_500.0));
    model.nodes.push(node(7, 6_000.0, -5_000.0, 3_500.0));
    for (sid, (a, b)) in [(0u32, (5u32, 4u32)), (1u32, (6u32, 7u32))] {
        model.slabs.push(Slab {
            id: SlabId(sid),
            shape: SlabShape::Enclosed {
                boundary: vec![NodeId(2), NodeId(3), NodeId(a), NodeId(b)],
            },
            plate: SlabPlate {
                section: Some(SectionId(2)),
                loads: vec![],
                usage: None,
                method: DistributionMethod::TriTrapezoid,
                one_way: None,
            },
        });
    }

    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let girder = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Girder)
        .unwrap();
    // 側面 (0.8−0.15)×2×5.3 + 底面 0.4×5.3
    let expected = ((0.8 - 0.15) * 2.0 + 0.4) * 5.3;
    assert!((girder.formwork_m2 - expected).abs() < 1e-9);
}

#[test]
fn test_joist_by_ratio() {
    // 柱に取り付かない水平梁は小梁として鉄筋比で概算する。
    let mut model = rc_portal_model();
    model.nodes.push(node(4, 2_000.0, 0.0, 3_500.0));
    model.nodes.push(node(5, 2_000.0, 3_000.0, 3_500.0));
    model.elements.push(line_elem(4, 4, 5, 0));

    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let joist = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Joist)
        .unwrap();
    // 0.4×0.8×3.0 = 0.96 m³、型枠 (0.4+1.6)×3.0 = 6.0 m²
    assert!((joist.concrete_m3 - 0.96).abs() < 1e-9);
    assert!((joist.formwork_m2 - 6.0).abs() < 1e-9);
    // 主筋 0.8% → 0.96×0.008×7.85 t
    let main = joist
        .rebar
        .iter()
        .find(|r| r.usage == RebarUsage::JoistMain)
        .unwrap();
    assert!((main.weight_t - 0.96 * 0.008 * 7.85).abs() < 1e-9);
}

#[test]
fn test_steel_member_weight() {
    // S 造: W = L×A×7.85（t/m³）。
    let mut model = rc_portal_model();
    let shape = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    };
    let a = shape.calc_area();
    // 材料は断面が持つ。
    let mut sec = shape.to_section(SectionId(2), "H-400x200x8x13".to_string());
    sec.material = Some(MaterialId(2));
    model.sections.push(sec);
    model.materials.push(steel_material(2));
    // 大梁を S 断面へ差し替え。
    model.elements[2].section = Some(SectionId(2));

    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let girder = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Girder)
        .unwrap();
    assert_eq!(girder.structure, StructureKind::S);
    assert!(girder.concrete_m3 == 0.0 && girder.formwork_m2 == 0.0);
    let steel = girder.steel.as_ref().unwrap();
    // 鉄骨は節点間距離（6.0m）で算定。
    assert!((steel.length_m - 6.0).abs() < 1e-9);
    assert!((steel.weight_t - a * 6_000.0 * 7.85e-9).abs() < 1e-12);
}

#[test]
fn test_brace_length_and_weight() {
    // ブレース長さは節点間距離 LB=√(L²+H²)。
    let mut model = rc_portal_model();
    let shape = SectionShape::SteelAngle {
        leg_a: 90.0,
        leg_b: 90.0,
        thick: 7.0,
    };
    let a = shape.calc_area();
    // 材料は断面が持つ。
    let mut sec = shape.to_section(SectionId(2), "L-90x90x7".to_string());
    sec.material = Some(MaterialId(2));
    model.sections.push(sec);
    model.materials.push(steel_material(2));
    let mut brace = line_elem(4, 0, 3, 2);
    brace.kind = ElementKind::Brace {
        tension_only: false,
    };
    model.elements.push(brace);

    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let br = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Brace)
        .unwrap();
    let lb = (6_000.0_f64.powi(2) + 3_500.0_f64.powi(2)).sqrt();
    let steel = br.steel.as_ref().unwrap();
    assert!((steel.length_m - lb / 1_000.0).abs() < 1e-9);
    assert!((steel.weight_t - a * lb * 7.85e-9).abs() < 1e-12);
}

#[test]
fn test_slab_quantity() {
    let mut model = rc_portal_model();
    // 6m×5m の床板。板厚はスラブ断面が持つ（建物一律の `slab_thickness` は
    // 剛性計算に見込む厚さであり、数量には使わない）。
    model.nodes.push(node(4, 0.0, 5_000.0, 3_500.0));
    model.nodes.push(node(5, 6_000.0, 5_000.0, 3_500.0));
    let slab_sec = squid_n_core::ids::SectionId(model.sections.len() as u32);
    model.sections.push(
        squid_n_core::section_shape::SectionShape::RcSlab { thickness: 150.0 }
            .to_section(slab_sec, "S15".into()),
    );
    model.slabs.push(Slab {
        id: SlabId(0),
        shape: SlabShape::Enclosed {
            boundary: vec![NodeId(2), NodeId(3), NodeId(5), NodeId(4)],
        },
        plate: SlabPlate {
            section: Some(slab_sec),
            loads: vec![],
            usage: None,
            method: DistributionMethod::TriTrapezoid,
            one_way: None,
        },
    });

    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let slab = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Slab)
        .unwrap();
    // 面積 30 m²、体積 30×0.15 = 4.5 m³、型枠 30 m²
    assert!((slab.formwork_m2 - 30.0).abs() < 1e-6);
    assert!((slab.concrete_m3 - 4.5).abs() < 1e-6);
    // 床筋: 4.5×1.0%×7.85 t
    let bar = slab.rebar.first().unwrap();
    assert!((bar.weight_t - 4.5 * 0.01 * 7.85).abs() < 1e-9);
}

#[test]
fn test_wall_quantity_with_opening() {
    let mut model = rc_portal_model();
    // 柱・梁で囲まれた 4 節点壁（節点 0-1-3-2）。
    let shape = SectionShape::RcWall {
        thickness: 200.0,
        ps: 0.0025,
    };
    let sec = shape.to_section(SectionId(2), "W20".to_string());
    model.sections.push(sec);
    let mut wall = line_elem(4, 0, 1, 2);
    wall.kind = ElementKind::Wall;
    wall.nodes = {
        let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
        for n in [0, 1, 3, 2] {
            v.push(NodeId(n));
        }
        v
    };
    model.elements.push(wall);
    model.wall_attrs.push(squid_n_core::model::WallAttr {
        elem: ElemId(4),
        opening_area: 2.0e6, // 2 m²
        opening_weight: 0.0,
        three_side_slit: false,
        openings: vec![],
    });

    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let w = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Wall)
        .unwrap();
    // 内法: L = 6000 − 700/2×2 = 5300（柱 700 角）、H = 3500 − 800/2×2 = 2700（梁せい 800）
    let net = 5.3_f64 * 2.7 - 2.0;
    assert!((w.concrete_m3 - net * 0.2).abs() < 1e-6);
    assert!((w.formwork_m2 - net * 2.0).abs() < 1e-6);
    // 壁筋: 横筋・縦筋の 2 件が計上される。
    assert_eq!(w.rebar.len(), 2);
    assert!(w.rebar_weight_t() > 0.0);
}

/// `compute_quantity_takeoff` が壁展開モデルを見ていることの回帰テスト
/// （申し送り §5.15）。壁の解析要素（`ElementKind::Wall`）は入力の正である
/// `Model`（`WallPlate`＋`WallRegion` 経由）には存在しない生成物（D5）のため、
/// 内部で壁展開しないと `MemberCategory::Wall` の数量が一切集計されない
/// （壁の数量が積算から静かに消える）。
#[test]
fn test_wall_quantity_via_wall_plate_is_included() {
    use squid_n_core::ids::{WallPlateId, WallRegionId};
    use squid_n_core::model::{WallPlate, WallPlateShape, WallRegion};

    let mut model = rc_portal_model();
    let shape = SectionShape::RcWall {
        thickness: 200.0,
        ps: 0.0025,
    };
    let sec = shape.to_section(SectionId(2), "W20".to_string());
    model.sections.push(sec);
    // 柱・梁で囲まれた4節点壁（節点 0-1-3-2）を壁版として構築する
    // （`test_wall_quantity_with_opening` の直接 `ElementData` 構築と同じ幾何）。
    model.wall_plates.push(WallPlate {
        id: WallPlateId(0),
        shape: WallPlateShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(3), NodeId(2)],
        },
        section: Some(SectionId(2)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings: Vec::new(),
        three_side_slit: false,
    });
    model.wall_regions.push(WallRegion {
        id: WallRegionId(0),
        name: String::new(),
        boundary: vec![NodeId(0), NodeId(1), NodeId(3), NodeId(2)],
        wall_plate_ids: vec![WallPlateId(0)],
        post_ids: Vec::new(),
    });
    // 入力モデル自体には壁の解析要素は一切ない（D5）。
    assert!(model.elements.iter().all(|e| e.kind != ElementKind::Wall));

    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let w = q.items.iter().find(|i| i.category == MemberCategory::Wall);
    assert!(
        w.is_some(),
        "壁展開モデルを経由しないと、model.elements に壁要素が 0 件のため \
         壁の数量が積算から欠落するはず: {:?}",
        q.items.iter().map(|i| i.category).collect::<Vec<_>>()
    );
    let w = w.unwrap();
    assert!(w.concrete_m3 > 0.0, "壁のコンクリート数量が正であるはず");
    assert!(w.formwork_m2 > 0.0, "壁の型枠数量が正であるはず");
}

/// 取り付く壁版（`Attached`。パラペット等）は解析要素を持たない（D5）ため、
/// `wall_quantity`（エレメント経由）ではなく `attached_wall_plate_quantity`
/// （`WallPlate` から直接算定）が数量へ算入する。カテゴリは
/// `OutOfFrameMiscWall` の後継として `MemberCategory::MiscWall` を使う。
#[test]
fn test_attached_wall_plate_quantity_is_included_as_misc_wall() {
    use squid_n_core::ids::WallPlateId;
    use squid_n_core::model::{RegionAnchor, WallPlate, WallPlateShape};

    let mut model = rc_portal_model();
    let shape = SectionShape::RcWall {
        thickness: 150.0,
        ps: 0.0,
    };
    let sec = shape.to_section(SectionId(2), "W15".to_string());
    model.sections.push(sec);
    // 頂部の大梁（節点2-3）に載るパラペット（立ち上がり高さ1000mm、全長）。
    model.wall_plates.push(WallPlate {
        id: WallPlateId(0),
        shape: WallPlateShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(2), NodeId(3)],
                span: [0.0, 1.0],
                transfer: squid_n_core::model::LoadTransfer::Anchor,
            },
            extent: [1000.0, 1000.0],
        },
        section: Some(SectionId(2)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings: Vec::new(),
        three_side_slit: false,
    });
    assert!(model.elements.iter().all(|e| e.kind != ElementKind::Wall));

    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let items: Vec<_> = q
        .items
        .iter()
        .filter(|i| i.category == MemberCategory::MiscWall)
        .collect();
    assert_eq!(items.len(), 1, "取り付く壁版1件が雑壁として数量に入るはず");
    let expected_m3 = 6_000.0 * 1_000.0 * 150.0 * 1e-9;
    assert!(
        (items[0].concrete_m3 - expected_m3).abs() / expected_m3 < 1e-9,
        "concrete_m3={} expected={}",
        items[0].concrete_m3,
        expected_m3
    );
}

#[test]
fn test_totals_aggregation() {
    let model = rc_portal_model();
    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let totals = q.totals();
    let sum: f64 = q.items.iter().map(|i| i.concrete_m3).sum();
    assert!((totals.concrete_m3 - sum).abs() < 1e-12);
    // 部位別小計に柱・大梁・基礎梁が現れる。
    let by_cat = q.totals_by_category();
    assert_eq!(by_cat.len(), 3);
}

#[test]
fn test_girder_haunch_from_member_detail() {
    use squid_n_core::model::{Haunch as CoreHaunch, MemberDetailAttr};

    // 大梁（ElemId(2)）の i 端にハンチ: 長さ 1000、せい増分 200、幅増分 200。
    let mut model = rc_portal_model();
    model.member_detail_attrs.push(MemberDetailAttr {
        elem: ElemId(2),
        haunch_i: Some(CoreHaunch {
            length: 1_000.0,
            depth_increase: 200.0,
            width_increase: 200.0,
        }),
        haunch_j: None,
        joints: vec![],
    });
    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let g = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::Girder)
        .unwrap();

    // 体積: 基準 0.4×0.8×5.3 に平均断面 (400+600)(800+1000)/4×1000 を加算。
    let base_m3 = 0.4 * 0.8 * 5.3;
    let haunch_m3 = (400.0 + 600.0) * (800.0 + 1_000.0) / 4.0 * 1_000.0 * 1e-9;
    assert!((g.concrete_m3 - (base_m3 + haunch_m3)).abs() < 1e-9);

    // 型枠（スラブなし）: 基準 (0.8×2+0.4)×5.3 に側面 (Di−D)/2×Li×2 と
    // 底面 (Bi−B)/2×Li を加算。
    let base_m2 = (0.8 * 2.0 + 0.4) * 5.3;
    let haunch_m2 =
        ((1_000.0 - 800.0) / 2.0 * 1_000.0 * 2.0 + (600.0 - 400.0) / 2.0 * 1_000.0) * 1e-6;
    assert!((g.formwork_m2 - (base_m2 + haunch_m2)).abs() < 1e-9);

    // 付帯情報を持たない基礎梁は従来どおり（ハンチなし）。
    let fg = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::FoundationGirder)
        .unwrap();
    assert!((fg.formwork_m2 - 1.2 * 5.3).abs() < 1e-9);
}

#[test]
fn test_foundation_girder_haunch_from_member_detail() {
    use squid_n_core::model::{Haunch as CoreHaunch, MemberDetailAttr};

    // 基礎梁（ElemId(3)）の両端にハンチ: 長さ 800、せい増分 300（幅増分なし）。
    let mut model = rc_portal_model();
    let haunch = CoreHaunch {
        length: 800.0,
        depth_increase: 300.0,
        width_increase: 0.0,
    };
    model.member_detail_attrs.push(MemberDetailAttr {
        elem: ElemId(3),
        haunch_i: Some(haunch),
        haunch_j: Some(haunch),
        joints: vec![],
    });
    let q = compute_quantity_takeoff(&model, &QuantityCfg::default());
    let fg = q
        .items
        .iter()
        .find(|i| i.category == MemberCategory::FoundationGirder)
        .unwrap();

    // 体積: 基準 0.4×0.8×5.3 に (400+400)(800+1100)/4×800 を両端分加算。
    let base_m3 = 0.4 * 0.8 * 5.3;
    let haunch_m3 = (400.0 + 400.0) * (800.0 + 1_100.0) / 4.0 * 800.0 * 2.0 * 1e-9;
    assert!((fg.concrete_m3 - (base_m3 + haunch_m3)).abs() < 1e-9);

    // 型枠: 側面 1 面＋底面 (0.8+0.4)×5.3 に側面 (Di−D)/2×Li を両端分加算
    // （幅増分 0 のため底面の加算なし）。
    let base_m2 = 1.2 * 5.3;
    let haunch_m2 = (1_100.0 - 800.0) / 2.0 * 800.0 * 2.0 * 1e-6;
    assert!((fg.formwork_m2 - (base_m2 + haunch_m2)).abs() < 1e-9);
}

fn girder_formwork(model: &Model) -> f64 {
    compute_quantity_takeoff(model, &QuantityCfg::default())
        .items
        .iter()
        .find(|i| i.elem == Some(ElemId(2)) && i.category == MemberCategory::Girder)
        .map(|i| i.formwork_m2)
        .expect("大梁型枠")
}

/// 床板のない囲まれた床領域は型枠控除なし。取り付く床板ありは取付き線の大梁が n_adj≥1。
#[test]
fn test_formwork_plateless_no_deduction_attached_deducts() {
    use squid_n_core::model::{LoadTransfer, RegionAnchor};
    use squid_n_core::section_shape::SectionShape;

    let empty = rc_portal_model();
    let form_empty = girder_formwork(&empty);

    let mut plateless = rc_portal_model();
    plateless.slab_thickness = 150.0;
    plateless.nodes.push(node(4, 0.0, 5_000.0, 3_500.0));
    plateless.nodes.push(node(5, 6_000.0, 5_000.0, 3_500.0));
    plateless.floor_regions.push(FloorRegion::new(
        FloorRegionId(0),
        vec![NodeId(2), NodeId(3), NodeId(5), NodeId(4)],
    ));
    let form_plateless = girder_formwork(&plateless);
    assert!(
        (form_plateless - form_empty).abs() < 1e-9,
        "床板のない囲まれは控除なし empty={form_empty} plateless={form_plateless}"
    );

    let mut attached = rc_portal_model();
    let mut slab_sec =
        SectionShape::RcSlab { thickness: 150.0 }.to_section(SectionId(2), "S15".into());
    slab_sec.material = Some(MaterialId(0));
    attached.sections.push(slab_sec);
    attached.slabs.push(Slab {
        id: SlabId(0),
        shape: SlabShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(2), NodeId(3)],
                span: [0.0, 1.0],
                transfer: LoadTransfer::Anchor,
            },
            extent: [-1500.0, -1500.0],
        },
        plate: SlabPlate {
            section: Some(SectionId(2)),
            ..Default::default()
        },
    });
    let form_att = girder_formwork(&attached);
    assert!(
        form_att < form_empty - 1e-6,
        "取り付く床板ありは型枠が減る empty={form_empty} att={form_att}"
    );
}
