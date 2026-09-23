//! 自重の荷重ケース内容の生成（標準構成では「DL」ケースへ同期される）。
//!
//! **地震用重量との関係:** 本内容を含む「DL」ケースを地震用重量の重力ケースに
//! 算入する場合、階の自動生成では密度からの自重直接算入を行わず、DL の設計自重を
//! 物理質量へ置換する `generate_stories_with_synced_self_weight` を使う
//! （密度から直接算入すると二重計上になる）。

use squid_n_core::model::{LoadCfg, MemberLoad, MemberLoadKind, Model, NodalLoad};

use crate::story_gen::{enumerate_self_weight, SelfWeightItem};

/// 旧スキーマの自重自動生成荷重ケース名（現行は自重を「DL」ケースへ統合して
/// 同期する。読込時の移行 `Model::migrate_legacy_auto_load_cases` が参照する）。
pub const SELF_WEIGHT_AUTO_LOAD_CASE_NAME: &str = "自重(自動)";

/// 重力方向（全体座標系 −Z）。
const DIR_DOWN: [f64; 3] = [0.0, 0.0, -1.0];

/// 自重(自動)ケースの内容（節点荷重・部材荷重）を生成する。
///
/// - **柱（2 節点の鉛直 `ElementKind::Beam`）**: 上端節点へ `total/2`、下端節点へ
///   `total/2 + extra_bottom` の節点荷重（下端は z 座標が低い側）。`extra_bottom` は
///   下階柱なしの RC/SRC 柱で梁最大せい相当を下端のみへ加える追加自重 [N]。
/// - **梁・ブレース（非柱の線材）**: 総重量（自重算定長・スラブ厚控除・仕上げ・
///   付加線重量を反映済み）を節点間全長で均した等分布荷重
///   `w = total/len [N/mm]`（dir = −Z）として与える。梁は自重による曲げ
///   （③梁自重による CMoQ 相当）を生じる。自重算定長の規則は
///   総量に反映済みであり、分布は全長均しとする（総量保存）。
///   K型ブレースの基準節点配分規則（地震用重量の集計規約）は応力解析には
///   適用せず、物理的な等分布のままとする。
/// - **二次部材（小梁・間柱）**: ここでは扱わない。逐次伝達（[`crate::cascade`]）が
///   床分配の辺荷重と一緒に受け持ち、主架構まで運ぶ。
/// - **ダンパー**: 装置＋支持部重量を両端節点へ 1/2 ずつの節点荷重とする。
/// - **壁・シェル**: 頂点への節点荷重（縁が切れていない梁際の辺へ。上下とも一体なら
///   四隅へ等分、片側の梁際が切れていれば反対側の 2 節点へ全量）。
/// - **フレーム外雑壁**: 近傍節点への節点荷重（`story_gen` と同じ配分）。
///
/// 同一節点への荷重は 1 件の `NodalLoad` に合算して返す。
pub fn self_weight_case_content(
    model: &Model,
    load_cfg: &LoadCfg,
) -> (Vec<NodalLoad>, Vec<MemberLoad>) {
    let mut node_force = vec![0.0_f64; model.nodes.len()];
    let mut member: Vec<MemberLoad> = Vec::new();

    for item in enumerate_self_weight(model, load_cfg) {
        match item {
            SelfWeightItem::Line {
                elem_idx,
                load,
                extra_bottom_load,
                is_column,
                ..
            } => {
                let elem = &model.elements[elem_idx];
                let ni = elem.nodes[0].index();
                let nj = elem.nodes[1].index();
                let (ci, cj) = (model.nodes[ni].coord, model.nodes[nj].coord);
                if is_column {
                    let (top, bottom) = if ci[2] <= cj[2] { (nj, ni) } else { (ni, nj) };
                    node_force[top] += load / 2.0;
                    node_force[bottom] += load / 2.0 + extra_bottom_load;
                } else if load > 0.0 {
                    let len = ((cj[0] - ci[0]).powi(2)
                        + (cj[1] - ci[1]).powi(2)
                        + (cj[2] - ci[2]).powi(2))
                    .sqrt();
                    if len > 0.0 {
                        let w = load / len;
                        member.push(MemberLoad::auto(
                            elem.id,
                            DIR_DOWN,
                            MemberLoadKind::Distributed {
                                a: 0.0,
                                b: len,
                                w1: w,
                                w2: w,
                            },
                        ));
                    } else {
                        node_force[ni] += load / 2.0;
                        node_force[nj] += load / 2.0;
                    }
                }
            }
            SelfWeightItem::Damper { ni, nj, load, .. } => {
                node_force[ni] += load / 2.0;
                node_force[nj] += load / 2.0;
            }
            SelfWeightItem::Panel { load_shares, .. } => {
                for (i, w) in load_shares {
                    node_force[i] += w;
                }
            }
        }
    }

    let nodal: Vec<NodalLoad> = node_force
        .iter()
        .enumerate()
        .filter(|(_, w)| **w > 0.0)
        .map(|(i, w)| {
            NodalLoad::auto(
                squid_n_core::ids::NodeId(i as u32),
                [0.0, 0.0, -w, 0.0, 0.0, 0.0],
            )
        })
        .collect();

    (nodal, member)
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::MaterialCategory;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Node, Section,
    };
    use squid_n_core::units::GRAVITY_MM_S2;

    fn simple_node(id: u32, coord: [f64; 3]) -> Node {
        Node {
            id: NodeId(id),
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    fn rc_section(area: f64, width: f64, depth: f64) -> Section {
        Section {
            id: SectionId(0),
            name: "RC".into(),
            area,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth,
            width,
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
        }
    }

    fn rc_material() -> Material {
        Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Fc24".into(),
            category: MaterialCategory::Concrete,
            young: 22000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        }
    }

    fn beam_elem(id: u32, a: u32, b: u32) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
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

    /// 柱1本＋梁1本のモデルで、柱は上端・下端の節点荷重（各 W/2、追加なし）、
    /// 梁は等分布部材荷重として生成され、合計が ρ·A·L·g と一致すること。
    #[test]
    fn test_self_weight_case_totals() {
        let model = Model {
            nodes: vec![
                simple_node(0, [0.0, 0.0, 0.0]),
                simple_node(1, [0.0, 0.0, 3000.0]),
                simple_node(2, [6000.0, 0.0, 3000.0]),
            ],
            sections: vec![rc_section(400.0 * 600.0, 400.0, 600.0)],
            materials: vec![rc_material()],
            // 柱（鉛直）と梁（水平）
            elements: vec![beam_elem(0, 0, 1), beam_elem(1, 1, 2)],
            ..Default::default()
        };

        let (nodal, member) = self_weight_case_content(&model, &LoadCfg::default());

        // 柱は節点荷重（上下 1/2 ずつ、追加なし）。梁は等分布部材荷重。
        assert_eq!(nodal.len(), 2, "柱の上下端に節点荷重が生じる");
        assert_eq!(member.len(), 1, "梁だけが部材荷重になる");
        let nodal_force = |node: u32| {
            nodal
                .iter()
                .find(|nl| nl.node == NodeId(node))
                .map(|nl| -nl.values[2])
                .unwrap_or(0.0)
        };

        // RC 柱は床上面から床上面（＝節点間距離 3000、フェイス控除なし）。
        let w_col = 2.4e-9 * 400.0 * 600.0 * 3000.0 * GRAVITY_MM_S2;
        // RC 梁は柱面間距離で、i 端に柱（せい 600）が取り付くので 600/2=300 を控除する。
        // j 端は取り付く直交材がないため控除は 0。
        let w_beam = 2.4e-9 * 400.0 * 600.0 * (6000.0 - 300.0) * GRAVITY_MM_S2;

        assert!(
            (nodal_force(1) - w_col / 2.0).abs() < 1e-9 * w_col,
            "上端={} W/2={}",
            nodal_force(1),
            w_col / 2.0
        );
        assert!(
            (nodal_force(0) - w_col / 2.0).abs() < 1e-9 * w_col,
            "下端={} W/2={}",
            nodal_force(0),
            w_col / 2.0
        );

        let member_total: f64 = member
            .iter()
            .map(|ml| match ml.kind {
                MemberLoadKind::Distributed { a, b, w1, w2 } => (b - a) * (w1 + w2) / 2.0,
                MemberLoadKind::Point { p, .. } => p,
            })
            .sum();
        assert!(
            (member_total - w_beam).abs() < 1e-9 * w_beam,
            "member={} w_beam={}",
            member_total,
            w_beam
        );

        let expected = w_col + w_beam;
        let total = nodal_force(0) + nodal_force(1) + member_total;
        assert!(
            (total - expected).abs() < 1e-6 * expected,
            "total={} expected={}",
            total,
            expected
        );
        // 梁荷重は重力方向
        assert!(member.iter().all(|ml| ml.dir == [0.0, 0.0, -1.0]));
    }

    /// 自重(自動)の総量（節点＋部材）が story_gen の地震用重量の自重集計と
    /// 一致すること（算定規則の単一ソースオブトゥルースの検証）。
    #[test]
    fn test_self_weight_matches_story_gen_totals() {
        let model = Model {
            nodes: vec![
                simple_node(0, [0.0, 0.0, 0.0]),
                simple_node(1, [0.0, 0.0, 3500.0]),
                simple_node(2, [5000.0, 0.0, 3500.0]),
                simple_node(3, [5000.0, 0.0, 0.0]),
            ],
            sections: vec![rc_section(500.0 * 500.0, 500.0, 500.0)],
            materials: vec![rc_material()],
            elements: vec![beam_elem(0, 0, 1), beam_elem(1, 1, 2), beam_elem(2, 3, 2)],
            ..Default::default()
        };

        let cfg = LoadCfg::default();
        let (nodal, member) = self_weight_case_content(&model, &cfg);
        let load_total: f64 = nodal.iter().map(|nl| -nl.values[2]).sum::<f64>()
            + member
                .iter()
                .map(|ml| match ml.kind {
                    MemberLoadKind::Distributed { a, b, w1, w2 } => (b - a) * (w1 + w2) / 2.0,
                    MemberLoadKind::Point { p, .. } => p,
                })
                .sum::<f64>();

        let weight_total: f64 = crate::story_gen::enumerate_self_weight(&model, &cfg)
            .iter()
            .map(|item| match item {
                crate::story_gen::SelfWeightItem::Line {
                    load,
                    extra_bottom_load,
                    ..
                } => *load + *extra_bottom_load,
                crate::story_gen::SelfWeightItem::Damper { load, .. } => *load,
                crate::story_gen::SelfWeightItem::Panel { load_shares, .. } => {
                    load_shares.iter().map(|(_, w)| w).sum()
                }
            })
            .sum();

        assert!(
            (load_total - weight_total).abs() < 1e-9 * weight_total.max(1.0),
            "load_total={} weight_total={}",
            load_total,
            weight_total
        );
    }

    /// 基部節点 0 に柱(0-1, 鉛直)と水平梁(0-2, 0-3)が取り付くモデルを組み立てる。
    /// 梁は `depths` のせいを持ち area=0（自重寄与なし）で、柱脚の最大せいだけを
    /// 検証できる。`is_concrete` が false なら柱・梁とも S 材とする。
    fn base_column_with_base_beams(is_concrete: bool, depths: &[f64]) -> Model {
        let mat = if is_concrete {
            rc_material()
        } else {
            Material {
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
                fy: None,
            }
        };
        let mut sections = vec![rc_section(90000.0, 300.0, 300.0)];
        let beam_nodes = [NodeId(2), NodeId(3)];
        for (k, &depth) in depths.iter().enumerate() {
            sections.push(Section {
                id: SectionId((k + 1) as u32),
                name: format!("Beam{depth}"),
                area: 0.0,
                iy: 1.0e8,
                iz: 1.0e8,
                j: 1.0e8,
                depth,
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
            });
        }
        let mut elements = vec![beam_elem(0, 0, 1)];
        for (k, node) in beam_nodes.iter().enumerate().take(depths.len()) {
            let mut e = beam_elem((k + 1) as u32, 0, node.0);
            e.section = Some(SectionId((k + 1) as u32));
            e.local_axis = LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            };
            elements.push(e);
        }
        Model {
            nodes: vec![
                simple_node(0, [0.0, 0.0, 0.0]),
                simple_node(1, [0.0, 0.0, 3000.0]),
                simple_node(2, [4000.0, 0.0, 0.0]),
                simple_node(3, [0.0, 4000.0, 0.0]),
            ],
            sections,
            materials: vec![mat],
            elements,
            ..Default::default()
        }
    }

    fn node_force(nodal: &[NodalLoad], node: u32) -> f64 {
        nodal
            .iter()
            .find(|nl| nl.node == NodeId(node))
            .map(|nl| -nl.values[2])
            .unwrap_or(0.0)
    }

    /// 通常の RC 柱（下階柱なし・基部に水平梁なし）: DL・地震用重量とも上端 W/2、下端 W/2。
    #[test]
    fn test_rc_column_self_weight_is_nodal_half_half() {
        let model = base_column_with_base_beams(true, &[]);
        let (nodal, member) = self_weight_case_content(&model, &LoadCfg::default());
        assert!(member.is_empty(), "梁が無いので部材荷重は出ない");
        let per_mm = 2.4e-9 * 90000.0 * GRAVITY_MM_S2;
        let w_col = per_mm * 3000.0;
        assert!(
            (node_force(&nodal, 1) - w_col / 2.0).abs() < 1e-9 * w_col,
            "DL 上端={}",
            node_force(&nodal, 1)
        );
        assert!(
            (node_force(&nodal, 0) - w_col / 2.0).abs() < 1e-9 * w_col,
            "DL 下端={}",
            node_force(&nodal, 0)
        );

        let gen = crate::story_gen::generate_stories(&model, None).unwrap();
        assert!(
            (gen.stories[1].seismic_weight.unwrap() - w_col / 2.0).abs() < 1e-9 * w_col,
            "地震用重量 上端階={}",
            gen.stories[1].seismic_weight.unwrap()
        );
        assert!(
            (gen.stories[0].seismic_weight.unwrap() - w_col / 2.0).abs() < 1e-9 * w_col,
            "地震用重量 基部階={}",
            gen.stories[0].seismic_weight.unwrap()
        );
    }

    /// 下階柱なし RC 柱: 下端に梁せい 600 と 800 が接続するとき、付加は最大せい 800 分を
    /// 下端節点のみへ加算し、上端へは配分しない。総量は w×(L+Dmax) を保存する。
    #[test]
    fn test_rc_base_column_extra_bottom_uses_max_beam_depth() {
        let model = base_column_with_base_beams(true, &[600.0, 800.0]);
        let (nodal, member) = self_weight_case_content(&model, &LoadCfg::default());
        assert!(member.is_empty(), "梁は area=0 なので部材荷重は出ない");
        let per_mm = 2.4e-9 * 90000.0 * GRAVITY_MM_S2;
        let w_col = per_mm * 3000.0;
        let wextra = per_mm * 800.0;
        assert!(
            (node_force(&nodal, 1) - w_col / 2.0).abs() < 1e-9 * w_col,
            "上端へ追加を配分しない: {}",
            node_force(&nodal, 1)
        );
        assert!(
            (node_force(&nodal, 0) - (w_col / 2.0 + wextra)).abs() < 1e-9 * w_col,
            "下端={} 期待={}",
            node_force(&nodal, 0),
            w_col / 2.0 + wextra
        );
        // 総量保存: 節点合計 = w×(L+Dmax)。
        let total = node_force(&nodal, 0) + node_force(&nodal, 1);
        let expected = per_mm * (3000.0 + 800.0);
        assert!(
            (total - expected).abs() < 1e-9 * expected,
            "total={total} expected={expected}"
        );

        let gen = crate::story_gen::generate_stories(&model, None).unwrap();
        assert!(
            (gen.stories[1].seismic_weight.unwrap() - w_col / 2.0).abs() < 1e-9 * w_col,
            "地震用重量 上端階={}",
            gen.stories[1].seismic_weight.unwrap()
        );
        assert!(
            (gen.stories[0].seismic_weight.unwrap() - (w_col / 2.0 + wextra)).abs() < 1e-9 * w_col,
            "地震用重量 基部階={}",
            gen.stories[0].seismic_weight.unwrap()
        );
    }

    /// 同条件の S 柱は梁せい付加をせず、DL・地震用重量とも上端 W/2、下端 W/2。
    #[test]
    fn test_steel_base_column_has_no_extra_bottom() {
        let model = base_column_with_base_beams(false, &[600.0, 800.0]);
        let (nodal, _member) = self_weight_case_content(&model, &LoadCfg::default());
        let per_mm = model.materials[0].design_unit_weight_n_per_mm3() * 90000.0;
        let w_col = per_mm * 3000.0;
        assert!(
            (node_force(&nodal, 1) - w_col / 2.0).abs() < 1e-9 * w_col,
            "DL 上端={}",
            node_force(&nodal, 1)
        );
        assert!(
            (node_force(&nodal, 0) - w_col / 2.0).abs() < 1e-9 * w_col,
            "S 柱は梁せい付加なし: 下端={}",
            node_force(&nodal, 0)
        );

        let gen = crate::story_gen::generate_stories(&model, None).unwrap();
        assert!(
            (gen.stories[1].seismic_weight.unwrap() - w_col / 2.0).abs() < 1e-9 * w_col,
            "地震用重量 上端階={}",
            gen.stories[1].seismic_weight.unwrap()
        );
        assert!(
            (gen.stories[0].seismic_weight.unwrap() - w_col / 2.0).abs() < 1e-9 * w_col,
            "地震用重量 基部階={}",
            gen.stories[0].seismic_weight.unwrap()
        );
    }
}
