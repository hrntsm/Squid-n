//! 解析前処理（モデルを解ける状態へ整える）。
//!
//! **解析の直前に必ず通すこと。** 前処理を省くと、仕口パネルのない剛性で解いたり、
//! 地震力ゼロで増分解析したりすることになる（実際に MCP サーバ側でそれが起きて
//! いた。GUI 側は `ensure_preparation` で同じ処理を通していた）。

use squid_n_core::model::Model;
use squid_n_core::region_rebuild::rebuild_floor_regions;

use crate::auto_loads::{apply_auto_load_cases, compute_auto_load_cases};
use crate::settings::AnalysisSettings;

/// 解析前処理（剛域・仕口パネル・荷重自動同期）の報告。
pub struct PrepareReport {
    /// 生成した仕口パネル（GUI の準備計算表が表示する）。
    pub panels: Vec<squid_n_element::panel_gen::GeneratedPanel>,
    /// 荷重同期で発生した注意事項（SemiPrecise で固有周期未算定など）。
    pub notices: Vec<String>,
}

/// 剛域と仕口パネルを自動算定してモデルへ反映する。
///
/// - 剛域: `Model::stress_cfg.rigid_zone_consider_walls` に従って壁を考慮する
/// - 仕口パネル: `Model::panel_zone` が有効なら S 造（CFT を除く）の柱梁接合節点へ
///   パネルを設け、無効なら既存のパネルを取り除く。あわせて部材の
///   `RigidZone::panel_offset_i/j` を現在のパネル配置から求め直す
///
/// いずれも冪等で、書き込み先が異なるため呼び出し順にも依存しない。
/// 戻り値は生成した仕口パネルの一覧（GUI の準備計算表が表示する）。
pub fn apply_rigid_zones_and_panels(
    model: &mut Model,
) -> Vec<squid_n_element::panel_gen::GeneratedPanel> {
    let rule = squid_n_element::beam::RigidZoneRule {
        consider_walls: model.stress_cfg.rigid_zone_consider_walls,
    };
    squid_n_element::beam::apply_auto_rigid_zones(model, &rule);
    squid_n_element::panel_gen::apply_auto_panel_zones(model)
}

/// 解析前処理を一括で行う（剛域・仕口パネル・荷重ケースの自動同期）。
///
/// 剛域と仕口パネルのみが必要な場合は [`apply_rigid_zones_and_panels`] を使う。
pub fn prepare_model_for_analysis(
    model: &mut Model,
    settings: &AnalysisSettings,
    design_period: Option<f64>,
) -> PrepareReport {
    rebuild_floor_regions(model);
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
}
