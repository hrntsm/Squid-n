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
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, MassMethod, Material, Node,
        Section,
    };
    use squid_n_core::section_shape::SectionShape;
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

    /// CFT 柱 1 本（基部 z=0・上端 z=3000）。主材料は鋼材区分＋充填コンクリート Fc36。
    fn cft_column_model(shape: SectionShape) -> Model {
        let mut section = shape.to_section(SectionId(0), "CFT".into());
        section.material = Some(MaterialId(0));
        Model {
            nodes: vec![
                simple_node(0, [0.0, 0.0, 0.0]),
                simple_node(1, [0.0, 0.0, 3000.0]),
            ],
            sections: vec![section],
            materials: vec![Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "CFT".into(),
                category: MaterialCategory::Steel,
                young: 205000.0,
                poisson: 0.3,
                density: 7.85e-9,
                shear: None,
                fc: Some(36.0),
                fy: None,
            }],
            elements: vec![beam_elem(0, 0, 1)],
            ..Default::default()
        }
    }

    /// 角形 CFT 柱（400×400×16, Fc36）の単位長さ設計自重が
    /// `78.5e-6·As + 23.0e-6·Ac`（As=鋼管断面積, Ac=充填部断面積）に一致する。
    /// As=24576, Ac=368²=135424、普通コンクリート Fc36 の γC=23.0 kN/m³。
    #[test]
    fn test_square_cft_column_design_weight_includes_filling_concrete() {
        let model = cft_column_model(SectionShape::CftBox {
            height: 400.0,
            width: 400.0,
            thick: 16.0,
        });
        let (nodal, member) = self_weight_case_content(&model, &LoadCfg::default());
        assert!(member.is_empty(), "柱のみなので部材荷重は出ない");

        let as_area: f64 = 400.0 * 400.0 - 368.0 * 368.0;
        let ac_area: f64 = 368.0 * 368.0;
        assert!((as_area - 24576.0).abs() < 1e-9);
        assert!((ac_area - 135424.0).abs() < 1e-9);
        let per_mm = 78.5e-6 * as_area + 23.0e-6 * ac_area;
        let w_col = per_mm * 3000.0;
        assert!(
            (node_force(&nodal, 1) - w_col / 2.0).abs() < 1e-9 * w_col,
            "上端={}",
            node_force(&nodal, 1)
        );
        assert!(
            (node_force(&nodal, 0) - w_col / 2.0).abs() < 1e-9 * w_col,
            "下端={}",
            node_force(&nodal, 0)
        );
    }

    /// 円形 CFT 柱（D=500, t=12, Fc36）の設計自重も同式に一致する。
    #[test]
    fn test_circular_cft_column_design_weight_includes_filling_concrete() {
        let (d, t) = (500.0_f64, 12.0_f64);
        let di = d - 2.0 * t;
        let model = cft_column_model(SectionShape::CftPipe {
            outer_dia: d,
            thick: t,
        });
        let (nodal, _member) = self_weight_case_content(&model, &LoadCfg::default());

        let as_area = std::f64::consts::PI * (d * d - di * di) / 4.0;
        let ac_area = std::f64::consts::PI * di * di / 4.0;
        let per_mm = 78.5e-6 * as_area + 23.0e-6 * ac_area;
        let w_col = per_mm * 3000.0;
        let total = node_force(&nodal, 0) + node_force(&nodal, 1);
        assert!(
            (total - w_col).abs() < 1e-9 * w_col,
            "total={total} expected={w_col}"
        );
    }

    /// DL ケースの総量に CFT 充填コンクリート分が含まれる（鋼管のみの設計重量を上回る）。
    #[test]
    fn test_dl_self_weight_case_includes_cft_filling_concrete() {
        let model = cft_column_model(SectionShape::CftBox {
            height: 400.0,
            width: 400.0,
            thick: 16.0,
        });
        let (nodal, member) = self_weight_case_content(&model, &LoadCfg::default());
        let total = node_force(&nodal, 0)
            + node_force(&nodal, 1)
            + member
                .iter()
                .map(|ml| match ml.kind {
                    MemberLoadKind::Distributed { a, b, w1, w2 } => (b - a) * (w1 + w2) / 2.0,
                    MemberLoadKind::Point { p, .. } => p,
                })
                .sum::<f64>();

        let steel_only = 78.5e-6 * 24576.0 * 3000.0;
        let core = 23.0e-6 * 135424.0 * 3000.0;
        assert!(
            (total - (steel_only + core)).abs() < 1e-9 * total,
            "total={total} expected={}",
            steel_only + core
        );
        assert!(
            total > steel_only,
            "充填コンクリート分が計上されていない: total={total} steel_only={steel_only}"
        );
    }

    /// 地震用重量（generate_stories）にも CFT 充填コンクリート分が含まれ、
    /// 上端・下端へ 1/2 ずつ配分される。
    #[test]
    fn test_generate_stories_seismic_weight_includes_cft_filling_concrete() {
        let model = cft_column_model(SectionShape::CftBox {
            height: 400.0,
            width: 400.0,
            thick: 16.0,
        });
        let per_mm = 78.5e-6 * 24576.0 + 23.0e-6 * 135424.0;
        let w_col = per_mm * 3000.0;
        let gen = crate::story_gen::generate_stories(&model, None).unwrap();
        assert_eq!(gen.stories.len(), 2);
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

    /// 鉄骨重量割増 factor=1.3 は CFT 鋼管部にだけ掛かり、充填コンクリート部には
    /// 掛からない（設計重量・物理質量の両方）。
    #[test]
    fn test_cft_steel_weight_factor_applies_to_steel_only() {
        let cfg = LoadCfg {
            steel_weight_factor: 1.3,
            ..Default::default()
        };
        let mut model = cft_column_model(SectionShape::CftBox {
            height: 400.0,
            width: 400.0,
            thick: 16.0,
        });
        model.load_cfg = Some(cfg.clone());
        let (as_area, ac_area) = (24576.0, 135424.0);

        // 設計重量: 鋼管部のみ factor、充填部は γC のまま。
        let (nodal, _) = self_weight_case_content(&model, &cfg);
        let total = node_force(&nodal, 0) + node_force(&nodal, 1);
        let expected_design = (78.5e-6 * as_area * 1.3 + 23.0e-6 * ac_area) * 3000.0;
        assert!(
            (total - expected_design).abs() < 1e-9 * expected_design,
            "設計自重={total} expected={expected_design}"
        );

        // 物理質量: 鋼管部のみ factor、充填部は物理密度 γC/g のまま。
        let gen =
            crate::story_gen::generate_stories_with_opts(&model, &[], true, MassMethod::LumpedOnly)
                .unwrap();
        let m = gen.rep_nodes[1].mass.expect("LumpedOnly は質点を持つ");
        let rho_core = 23.0e-6 / GRAVITY_MM_S2;
        let expected_mass = (7.85e-9 * as_area * 1.3 + rho_core * ac_area) * 3000.0 / 2.0;
        assert!(
            (m[0] - expected_mass).abs() < 1e-9 * expected_mass,
            "質点質量={} expected={expected_mass}",
            m[0]
        );
    }

    /// CFT 柱 1 本（基部 z=0・上端 z=3000）と、柱脚節点に取り付く水平梁 2 本
    /// （area=0・せい `depths`）を持つモデル。主材料は鋼材区分＋充填 Fc36。
    fn cft_base_column_with_base_beams(depths: &[f64]) -> Model {
        let mut column = SectionShape::CftBox {
            height: 400.0,
            width: 400.0,
            thick: 16.0,
        }
        .to_section(SectionId(0), "CFT".into());
        column.material = Some(MaterialId(0));
        let mut sections = vec![column];
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
            materials: vec![Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "CFT".into(),
                category: MaterialCategory::Steel,
                young: 205000.0,
                poisson: 0.3,
                density: 7.85e-9,
                shear: None,
                fc: Some(36.0),
                fy: None,
            }],
            elements,
            ..Default::default()
        }
    }

    /// 下階柱なし CFT 柱: 柱脚に水平梁（最大せい 800）が接続するとき、下端付加の
    /// 設計重量・物理質量相当に充填コンクリート分（γC·Ac·Dmax）が含まれ、鋼管部には
    /// `effective_steel_factor()` が掛かる。`is_concrete` による `max_depth` の適用条件は
    /// 変わらないため、CFT でも RC/SRC と同じく最大せい 800 が付加される。
    #[test]
    fn test_cft_base_column_extra_bottom_includes_filling_concrete() {
        let cfg = LoadCfg {
            steel_weight_factor: 1.3,
            ..Default::default()
        };
        let mut model = cft_base_column_with_base_beams(&[600.0, 800.0]);
        model.load_cfg = Some(cfg.clone());

        let items = crate::story_gen::enumerate_self_weight(&model, &cfg);
        let (extra_bottom_load, extra_bottom_mass_equiv) = items
            .iter()
            .find_map(|item| match item {
                crate::story_gen::SelfWeightItem::Line {
                    elem_idx,
                    extra_bottom_load,
                    extra_bottom_mass_equiv,
                    ..
                } if *elem_idx == 0 => Some((*extra_bottom_load, *extra_bottom_mass_equiv)),
                _ => None,
            })
            .expect("CFT 柱の Line がある");

        let (as_area, ac_area) = (24576.0, 135424.0);
        let dmax = 800.0;
        // 設計: 鋼管部（γs=78.5 kN/m³）×factor ＋ 充填部（γC=23.0 kN/m³）を最大せい分。
        let expected_load = (78.5e-6 * as_area * 1.3 + 23.0e-6 * ac_area) * dmax;
        assert!(
            (extra_bottom_load - expected_load).abs() < 1e-9 * expected_load,
            "extra_bottom_load={extra_bottom_load} expected={expected_load}"
        );

        // 物理: 鋼管部（物理密度 7.85e-9 ×g）×factor ＋ 充填部（γC/g）を最大せい分。
        let rho_core = 23.0e-6 / GRAVITY_MM_S2;
        let expected_mass = (7.85e-9 * as_area * 1.3 + rho_core * ac_area) * dmax * GRAVITY_MM_S2;
        assert!(
            (extra_bottom_mass_equiv - expected_mass).abs() < 1e-9 * expected_mass,
            "extra_bottom_mass_equiv={extra_bottom_mass_equiv} expected={expected_mass}"
        );

        // 鋼管部に割増が掛かり、factor 無し（充填部込み）より大きいこと。
        let no_factor = (78.5e-6 * as_area + 23.0e-6 * ac_area) * dmax;
        assert!(
            extra_bottom_load > no_factor,
            "鋼管部に割増が掛かっていない: {extra_bottom_load} <= {no_factor}"
        );
    }
}
