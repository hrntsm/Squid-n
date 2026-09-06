//! 解析前処理（モデルを解ける状態へ整える）。
//!
//! **解析の直前に必ず通すこと。** 前処理を省くと、仕口パネルのない剛性で解いたり、
//! 地震力ゼロで増分解析したりすることになる。

use squid_n_core::model::Model;
use squid_n_core::region_rebuild::rebuild_floor_regions;
use squid_n_core::wall_region_rebuild::rebuild_wall_regions;

use crate::auto_loads::{apply_auto_load_cases, compute_auto_load_cases};
use crate::settings::AnalysisSettings;

/// 解析前処理（剛域・仕口パネル・荷重自動同期）の報告。
pub struct PrepareReport {
    /// 生成した仕口パネル（GUI の準備計算表が表示する）。
    pub panels: Vec<squid_n_element::springs::panel_gen::GeneratedPanel>,
    /// 荷重同期で発生した注意事項（SemiPrecise で固有周期未算定など）。
    pub notices: Vec<String>,
}

/// 剛域と仕口パネルを自動算定してモデルへ反映する。
/// 剛域算定は壁展開モデルに対して行い、既存要素へ書き戻す。
/// 仕口パネルは壁に依存しないため非展開モデルに対して算定する。
/// いずれも冪等で、呼び出し順にも依存しない。
pub fn apply_rigid_zones_and_panels(
    model: &mut Model,
) -> Vec<squid_n_element::springs::panel_gen::GeneratedPanel> {
    let rule = squid_n_element::frame::beam::RigidZoneRule {
        consider_walls: model.stress_cfg.rigid_zone_consider_walls,
    };
    if squid_n_load::wall_expand::model_has_wall_plates_to_expand(model) {
        let (mut expanded, _wall_index, _wall_report) =
            squid_n_load::wall_expand::expand_wall_elements(model);
        squid_n_element::frame::beam::apply_auto_rigid_zones(&mut expanded, &rule);
        debug_assert!(
            model.elements.len() <= expanded.elements.len()
                && model
                    .elements
                    .iter()
                    .zip(expanded.elements.iter())
                    .all(|(dst, src)| dst.id == src.id),
            "壁展開後の先頭要素列の ElemId が非展開モデルと一致しない。\
             expand_wall_elements が既存要素の途中へ挿入していないか確認すること。"
        );
        for (dst, src) in model.elements.iter_mut().zip(expanded.elements.iter()) {
            dst.rigid_zone = src.rigid_zone;
        }
    } else {
        squid_n_element::frame::beam::apply_auto_rigid_zones(model, &rule);
    }
    squid_n_element::springs::panel_gen::apply_auto_panel_zones(model)
}

/// 解析前処理を一括で行う（剛域・仕口パネル・荷重ケースの自動同期）。
pub fn prepare_model_for_analysis(
    model: &mut Model,
    settings: &AnalysisSettings,
    design_period: Option<f64>,
) -> PrepareReport {
    rebuild_floor_regions(model);
    rebuild_wall_regions(model);
    let panels = apply_rigid_zones_and_panels(model);
    let computed = compute_auto_load_cases(model, settings, design_period);
    apply_auto_load_cases(model, &computed.cases);
    PrepareReport {
        panels,
        notices: computed.notices,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::{ElemId, NodeId, SectionId, SlabId};
    use squid_n_core::model::{
        DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node,
        SlabPlate, SlabShape,
    };
    use squid_n_core::section_shape::SectionShape;

    fn node(id: u32, x: f64, y: f64) -> Node {
        Node {
            id: NodeId(id),
            coord: [x, y, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    fn beam(id: u32, i: u32, j: u32) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: None,
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

    /// 閉路 1 + 床板 2 枚。荷重同期の前に、大梁の区画（床領域）1 つへ帰属し直す
    /// （床板そのものは畳まない。申し送り「床領域・壁領域の再設計」参照）。
    #[test]
    fn prepare_groups_two_slabs_into_one_region_before_loads() {
        let mut model = Model::default();
        for (i, (x, y)) in [
            (0.0, 0.0),
            (2000.0, 0.0),
            (4000.0, 0.0),
            (4000.0, 4000.0),
            (2000.0, 4000.0),
            (0.0, 4000.0),
        ]
        .into_iter()
        .enumerate()
        {
            model.nodes.push(node(i as u32, x, y));
        }
        model.elements.extend([
            beam(0, 0, 1),
            beam(1, 1, 2),
            beam(2, 2, 3),
            beam(3, 3, 4),
            beam(4, 4, 5),
            beam(5, 5, 0),
        ]);
        let sid = SectionId(0);
        model
            .sections
            .push(SectionShape::RcSlab { thickness: 150.0 }.to_section(sid, "S150".into()));
        for (i, b) in [vec![0, 1, 4, 5], vec![1, 2, 3, 4]].into_iter().enumerate() {
            model.slabs.push(squid_n_core::model::Slab {
                id: SlabId(i as u32),
                shape: SlabShape::Enclosed {
                    boundary: b.into_iter().map(NodeId).collect(),
                },
                plate: SlabPlate {
                    section: Some(sid),
                    loads: Vec::new(),
                    usage: None,
                    method: DistributionMethod::TriTrapezoid,
                    one_way: None,
                },
            });
        }
        assert_eq!(model.slabs.len(), 2);
        prepare_model_for_analysis(&mut model, &AnalysisSettings::default(), None);
        assert_eq!(model.slabs.len(), 2, "床板は畳まずそのまま残る");
        assert_eq!(
            model.floor_regions.len(),
            1,
            "大梁が囲む区画は 1 つなので床領域も 1 つ"
        );
        assert_eq!(
            model.floor_regions[0].slab_ids.len(),
            2,
            "2 枚とも同じ床領域へ帰属"
        );
    }

    /// 剛域算定（`apply_rigid_zones_and_panels`）が壁展開モデルを見ていることの
    /// 回帰テスト。壁展開しないまま算定すると `model.elements` に壁要素が
    /// 0 件のため `rigid_zone_consider_walls`（既定 true。柱の袖壁・梁の腰壁/垂壁
    /// の張り出しを剛域長へ反映する技術基準の規定）が壁の有無に関わらず常に
    /// 無効化されてしまう（`dev_docs/handoff/床領域・壁領域の再設計_申し送り.md`
    /// §5.15 参照）。
    ///
    /// 柱・梁を全周 RC にそろえる（`all_rc_src_at` が全節点で成立する構成に
    /// する）必要がある点に注意。RC/S 混在フレームでは「1 本でも S 系が
    /// 集まる仕口には剛域を設けない」規則が優先し、壁の有無による差が
    /// 現れない（この規則自体が §5.14 の `wall_bay_model` 回帰網を偶然素通り
    /// させていた原因）。
    #[test]
    fn apply_rigid_zones_considers_wall_plates() {
        use squid_n_core::ids::{MaterialId, WallPlateId, WallRegionId};
        use squid_n_core::model::{
            Material, MaterialCategory, WallPlate, WallPlateShape, WallRegion,
        };
        use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

        // 主筋 3-D22・せん断補強筋 D10@100（`剛域`の算定自体は鉄筋量を見ないが、
        // `RcRect`/`to_section` が鉄筋情報を要求するため、名目値を与える）。
        fn rebar() -> RcRebar {
            RcRebar {
                main_x: BarSet {
                    count: 3,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 3,
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

        fn node3(id: u32, x: f64, y: f64, z: f64) -> Node {
            Node {
                id: NodeId(id),
                coord: [x, y, z],
                restraint: Default::default(),
                mass: None,
                story: None,
                support_spring: None,
            }
        }

        // 1 バイの全周 RC 架構（柱 4 本・頂部梁 4 本・柱脚間梁 4 本）。
        // Y=0 面（節点 0,1,5,4）に耐震壁 1 枚を想定する。
        fn base_frame() -> Model {
            let mut model = Model::default();
            let base = [
                (0.0, 0.0, 0.0),
                (4000.0, 0.0, 0.0),
                (4000.0, 3000.0, 0.0),
                (0.0, 3000.0, 0.0),
            ];
            let top = [
                (0.0, 0.0, 3000.0),
                (4000.0, 0.0, 3000.0),
                (4000.0, 3000.0, 3000.0),
                (0.0, 3000.0, 3000.0),
            ];
            for (i, &(x, y, z)) in base.iter().enumerate() {
                model.nodes.push(node3(i as u32, x, y, z));
            }
            for (i, &(x, y, z)) in top.iter().enumerate() {
                model.nodes.push(node3(4 + i as u32, x, y, z));
            }
            model.materials.push(Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "Fc24".into(),
                category: MaterialCategory::Concrete,
                young: 23000.0,
                poisson: 0.2,
                density: 2.4e-9,
                shear: None,
                fc: Some(24.0),
                fy: None,
            });
            let mut col_sec = SectionShape::RcRect {
                b: 300.0,
                d: 300.0,
                rebar: rebar(),
            }
            .to_section(SectionId(0), "柱 RC 300x300".into());
            col_sec.material = Some(MaterialId(0));
            model.sections.push(col_sec);
            let mut beam_sec = SectionShape::RcRect {
                b: 300.0,
                d: 400.0,
                rebar: rebar(),
            }
            .to_section(SectionId(1), "梁 RC 300x400".into());
            beam_sec.material = Some(MaterialId(0));
            model.sections.push(beam_sec);

            let member = |id: u32, i: u32, j: u32, sec: u32| {
                let mut e = beam(id, i, j);
                e.section = Some(SectionId(sec));
                e
            };
            for i in 0..4u32 {
                model.elements.push(member(i, i, 4 + i, 0));
            }
            let top_pairs = [(4u32, 5u32), (5, 6), (6, 7), (7, 4)];
            for (k, (i, j)) in top_pairs.iter().enumerate() {
                model.elements.push(member(4 + k as u32, *i, *j, 1));
            }
            let base_pairs = [(0u32, 1u32), (1, 2), (2, 3), (3, 0)];
            for (k, (i, j)) in base_pairs.iter().enumerate() {
                model.elements.push(member(8 + k as u32, *i, *j, 1));
            }
            model
        }

        fn wall_section_id() -> SectionId {
            SectionId(2)
        }

        let mut with_wall = base_frame();
        let mut wall_sec = SectionShape::RcWall {
            thickness: 150.0,
            ps: 0.0025,
        }
        .to_section(wall_section_id(), "耐震壁 t150".into());
        wall_sec.material = Some(MaterialId(0));
        with_wall.sections.push(wall_sec);
        with_wall.wall_plates.push(WallPlate {
            id: WallPlateId(0),
            shape: WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(5), NodeId(4)],
            },
            section: Some(wall_section_id()),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![],
            slit: Default::default(),
        });
        with_wall.wall_regions.push(WallRegion {
            id: WallRegionId(0),
            name: String::new(),
            boundary: vec![NodeId(0), NodeId(1), NodeId(5), NodeId(4)],
            wall_plate_ids: vec![WallPlateId(0)],
            posts: Vec::new(),
        });

        let mut without_wall = base_frame();

        apply_rigid_zones_and_panels(&mut with_wall);
        apply_rigid_zones_and_panels(&mut without_wall);

        // 壁の側柱（elem 0、Y=0 面）は、壁を考慮すると剛域長が変わる。
        let with_wall_col0 = with_wall.elements[0].rigid_zone;
        let without_wall_col0 = without_wall.elements[0].rigid_zone;
        assert_ne!(
            with_wall_col0.length_i, without_wall_col0.length_i,
            "壁の有無で側柱の剛域長 length_i が変わらない\
             （壁展開モデルを見ずに算定している疑いがある）: with={:?} without={:?}",
            with_wall_col0, without_wall_col0
        );
        assert_ne!(
            with_wall_col0.length_j, without_wall_col0.length_j,
            "壁の有無で側柱の剛域長 length_j が変わらない: with={:?} without={:?}",
            with_wall_col0, without_wall_col0
        );
        // 壁ありのほうが張り出しの分だけ剛域は長くなる（少なくとも一方の端で）。
        assert!(
            with_wall_col0.length_j > without_wall_col0.length_j,
            "壁を考慮すると剛域長は伸びるはず: with={:?} without={:?}",
            with_wall_col0,
            without_wall_col0
        );

        // 壁の頂部大梁（elem 4）も同様。
        let with_wall_beam4 = with_wall.elements[4].rigid_zone;
        let without_wall_beam4 = without_wall.elements[4].rigid_zone;
        assert!(
            with_wall_beam4.length_i > without_wall_beam4.length_i
                && with_wall_beam4.length_j > without_wall_beam4.length_j,
            "壁の頂部大梁は壁を考慮すると両端とも剛域長が伸びるはず: with={:?} without={:?}",
            with_wall_beam4,
            without_wall_beam4
        );

        // `apply_rigid_zones_and_panels` 自身は壁要素をモデルへ残さない（D5）。
        // 壁展開はこの関数の内部だけの一時的な操作であること（書き戻しの実装
        // ミスで壁要素が漏れ出していないか）を確認する。
        assert!(
            with_wall
                .elements
                .iter()
                .all(|e| e.kind != squid_n_core::model::ElementKind::Wall),
            "apply_rigid_zones_and_panels は壁要素を model.elements へ残してはならない"
        );
    }
}
