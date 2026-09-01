//! 自重の荷重ケース内容の生成（標準構成では「DL」ケースへ同期される）。
//!
//! 柱梁自重・壁自重・ダンパー自重の重量算定規則
//! （[`crate::story_gen::enumerate_self_weight`] に一元化）を、長期応力解析用の
//! 部材荷重・節点荷重へ変換する。大梁の CMoQ のうち「③梁自重による CMoQ」と
//! 「②壁荷重による CMoQ」に相当する
//! 荷重経路を自動生成し、固定荷重（`LoadCaseKind::Dead`）として応力解析・
//! 荷重組合せへ接続する。
//!
//! 従来は自重が地震用重量（階の集計）にしか算入されず、長期応力解析には
//! 手動入力しない限り自重が載らなかった（照合レビューでの最重要指摘）。
//!
//! **地震用重量との関係:** 本内容（自重・フレーム外雑壁）を含む「DL」ケースを
//! 地震用重量の重力ケースに算入する場合は、階の自動生成で密度からの自重直接
//! 算入を無効にすること（[`crate::story_gen::generate_stories_with_opts`] の
//! `include_density_self_weight = false`。両方行うと二重計上になる）。

use squid_n_core::model::{LoadCfg, MemberLoad, MemberLoadKind, Model, NodalLoad};

use crate::story_gen::{enumerate_self_weight, misc_wall_weight_shares, SelfWeightItem};

/// 旧スキーマの自重自動生成荷重ケース名（現行は自重を「DL」ケースへ統合して
/// 同期する。読込時の移行 `Model::migrate_legacy_auto_load_cases` が参照する）。
pub const SELF_WEIGHT_AUTO_LOAD_CASE_NAME: &str = "自重(自動)";

/// 重力方向（全体座標系 −Z）。
const DIR_DOWN: [f64; 3] = [0.0, 0.0, -1.0];

/// 自重(自動)ケースの内容（節点荷重・部材荷重）を生成する。
///
/// - **線材（柱・梁・ブレース）**: 総重量（自重算定長・スラブ厚控除・仕上げ・
///   付加線重量を反映済み）を節点間全長で均した等分布荷重
///   `w = total/len [N/mm]`（dir = −Z）として与える。梁は自重による曲げ
///   （③梁自重による CMoQ 相当）、柱は軸力を生じる。自重算定長の規則は
///   総量に反映済みであり、分布は全長均しとする（総量保存）。
///   K型ブレースの基準節点配分規則（地震用重量の集計規約）は応力解析には
///   適用せず、物理的な等分布のままとする。
/// - **二次部材（小梁・間柱）**: ここでは扱わない。逐次伝達（[`crate::cascade`]）が
///   床分配の辺荷重と一緒に受け持ち、主架構まで運ぶ。
/// - **ダンパー**: 装置＋支持部重量を両端節点へ 1/2 ずつの節点荷重とする。
/// - **壁・シェル**: 頂点への節点荷重（四隅へ等分）。
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
            SelfWeightItem::Line { elem_idx, total } => {
                let elem = &model.elements[elem_idx];
                let ni = elem.nodes[0].index();
                let nj = elem.nodes[1].index();
                let (ci, cj) = (model.nodes[ni].coord, model.nodes[nj].coord);
                let len =
                    ((cj[0] - ci[0]).powi(2) + (cj[1] - ci[1]).powi(2) + (cj[2] - ci[2]).powi(2))
                        .sqrt();
                if total <= 0.0 {
                    continue;
                }
                if len > 0.0 {
                    let w = total / len;
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
                    node_force[ni] += total / 2.0;
                    node_force[nj] += total / 2.0;
                }
            }
            SelfWeightItem::Damper { ni, nj, total } => {
                node_force[ni] += total / 2.0;
                node_force[nj] += total / 2.0;
            }
            SelfWeightItem::Panel { shares } => {
                for (i, w) in shares {
                    node_force[i] += w;
                }
            }
            // 二次部材（小梁・間柱）: ここでは扱わない。逐次伝達
            // （[`crate::cascade`]）が床分配の辺荷重と一緒に受け持ち、支持相手が
            // 別の二次部材でも荷重を落とさずに主架構まで運ぶ。ここで両端節点へ
            // 1/2 ずつ載せると二重計上になる（申し送り §3.4 F6）。
            // 地震用重量の階配分（`story_gen`）は引き続き本項目を使う。
            SelfWeightItem::SecondaryLine { .. } => {}
        }
    }

    for (i, w) in misc_wall_weight_shares(model) {
        node_force[i] += w;
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

    /// 柱1本＋梁1本のモデルで、自重(自動)の総荷重が ρ·A·L·g と一致し、
    /// すべて等分布部材荷重（重力方向）として生成されること。
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
        assert!(nodal.is_empty(), "線材のみのモデルでは節点荷重は生じない");
        assert_eq!(member.len(), 2);

        // 総荷重 = ρ·A·(L柱 + L梁の内法長)·g（スラブ無し→スラブ厚の控除なし）。
        // RC 柱は床上面から床上面（＝節点間距離 3000。フェイス控除なし）。
        // RC 梁は柱面間距離で、i 端に柱（せい 600）が取り付くので 600/2=300 を控除する。
        // j 端は取り付く直交材がないため控除は 0。
        let expected = 2.4e-9 * 400.0 * 600.0 * (3000.0 + (6000.0 - 300.0)) * GRAVITY_MM_S2;
        let total: f64 = member
            .iter()
            .map(|ml| match ml.kind {
                MemberLoadKind::Distributed { a, b, w1, w2 } => (b - a) * (w1 + w2) / 2.0,
                MemberLoadKind::Point { p, .. } => p,
            })
            .sum();
        assert!(
            (total - expected).abs() < 1e-6 * expected,
            "total={} expected={}",
            total,
            expected
        );
        // 全て重力方向
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
                crate::story_gen::SelfWeightItem::Line { total, .. } => *total,
                crate::story_gen::SelfWeightItem::Damper { total, .. } => *total,
                crate::story_gen::SelfWeightItem::SecondaryLine { total, .. } => *total,
                crate::story_gen::SelfWeightItem::Panel { shares } => {
                    shares.iter().map(|(_, w)| w).sum()
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
}
