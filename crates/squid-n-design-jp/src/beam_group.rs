//! 一本部材グループ（`Model.beam_groups`）の検定文脈合成。

use std::collections::HashMap;

use squid_n_core::ids::ElemId;
use squid_n_core::model::Model;
use squid_n_element::beam::MemberForces;

use crate::BeamGroupContextOverride;

/// `Model.beam_groups` の各グループについて検定文脈の合成値を求め、
/// 所属要素 ID → 合成値の対応表を返す。
///
/// - グループは軸方向に連続する梁要素の ID を**並び順**で持つ前提
///   （幾何学的な連続性・共線性の検証は行わない。並び順が実際の配置と
///   異なる場合、端部モーメント等の対応がずれる）。
/// - 要素または内力が欠けるグループ・要素数 2 未満のグループは無視する。
/// - 中央モーメントは、A 式 `Mc_A = (|Q1|+|Q2|)・L/8 − (|M1|+|M2|)/2`
///   （端部せん断と釣り合う等分布荷重の単純梁中央モーメントから端部
///   モーメントの平均を差し引いた復元値）と、B 式（グループ中央位置を
///   含む分割部材の、中央位置に最も近い評価行のモーメント）の絶対値の
///   大きい方（符号は B 式に合わせる）。
pub fn beam_group_overrides(
    model: &Model,
    member_forces: &[(ElemId, MemberForces)],
) -> HashMap<ElemId, BeamGroupContextOverride> {
    let mut out: HashMap<ElemId, BeamGroupContextOverride> = HashMap::new();

    for group in &model.beam_groups {
        if group.len() < 2 {
            continue;
        }
        // 各分割部材の (要素, 内力, 長さ) を並び順に収集。欠けがあればスキップ。
        let mut parts: Vec<(&squid_n_core::model::ElementData, &MemberForces, f64)> =
            Vec::with_capacity(group.len());
        let mut ok = true;
        for id in group {
            let elem = model.element(*id);
            let mf = member_forces
                .iter()
                .find(|(mid, _)| mid == id)
                .map(|(_, m)| m);
            match (elem, mf) {
                (Some(e), Some(m)) if !m.at.is_empty() => {
                    let l = model.member_length(e);
                    if l <= 1e-9 {
                        ok = false;
                        break;
                    }
                    parts.push((e, m, l));
                }
                _ => {
                    ok = false;
                    break;
                }
            }
        }
        if !ok || parts.len() < 2 {
            continue;
        }

        let total: f64 = parts.iter().map(|p| p.2).sum();
        let row_at = |m: &MemberForces, target: f64| -> Option<[f64; 6]> {
            m.at.iter()
                .find(|(p, _)| (p - target).abs() < 1e-9)
                .map(|(_, f)| *f)
        };
        let first = parts[0].1;
        let last = parts[parts.len() - 1].1;
        let end_i = row_at(first, 0.0);
        let end_j = row_at(last, 1.0);
        let end_moments_z = match (end_i, end_j) {
            (Some(a), Some(b)) => Some((a[5], b[5])),
            _ => None,
        };

        // A 式: M0 = (Q1+Q2)・L/8（端部せん断と釣り合う等分布仮定）。
        let q1 = end_i.map(|f| f[1].abs()).unwrap_or(0.0);
        let q2 = end_j.map(|f| f[1].abs()).unwrap_or(0.0);
        let m0_a = (q1 + q2) * total / 8.0;
        let m_ends_avg = end_moments_z
            .map(|(a, b)| (a.abs() + b.abs()) / 2.0)
            .unwrap_or(0.0);
        let mc_a = m0_a - m_ends_avg;

        // B 式: グループ中央位置を含む分割部材の、中央位置に最も近い評価行。
        let target_s = total / 2.0;
        let mut acc = 0.0;
        let mut mc_b: Option<f64> = None;
        for (_, m, l) in &parts {
            if target_s <= acc + l + 1e-9 {
                let xi = ((target_s - acc) / l).clamp(0.0, 1.0);
                mc_b =
                    m.at.iter()
                        .min_by(|a, b| {
                            (a.0 - xi)
                                .abs()
                                .partial_cmp(&(b.0 - xi).abs())
                                .unwrap_or(std::cmp::Ordering::Equal)
                        })
                        .map(|(_, f)| f[5]);
                break;
            }
            acc += l;
        }
        let mid_moment_z = mc_b.map(|b| {
            let sign = if b >= 0.0 { 1.0 } else { -1.0 };
            sign * b.abs().max(mc_a)
        });

        let shear_span = parts
            .iter()
            .flat_map(|(_, m, _)| m.at.iter())
            .map(|(_, f)| (f[5].abs(), f[1].abs()))
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        let face_sum = parts[0].0.rigid_zone.face_i_or_zero()
            + parts[parts.len() - 1].0.rigid_zone.face_j_or_zero();
        let clear_length = if total - face_sum > 0.0 {
            total - face_sum
        } else {
            total
        };

        let ov = BeamGroupContextOverride {
            length: total,
            end_moments_z,
            mid_moment_z,
            shear_span,
            clear_length,
        };
        for (e, _, _) in &parts {
            out.insert(e.id, ov.clone());
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use smallvec::SmallVec;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node, RigidZone,
    };

    /// 一本部材グループの合成値（全長・端部/中央モーメント・せん断スパン代表値）の手計算照合。
    #[test]
    fn beam_group_overrides_combines_members() {
        let node = |id: u32, x: f64| Node {
            id: NodeId(id),
            coord: [x, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        let beam = |id: u32, n0: u32, n1: u32| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: {
                let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
                v.push(NodeId(n0));
                v.push(NodeId(n1));
                v
            },
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        };
        let model = Model {
            nodes: vec![node(0, 0.0), node(1, 3000.0), node(2, 6000.0)],
            elements: vec![beam(0, 0, 1), beam(1, 1, 2)],
            beam_groups: vec![vec![ElemId(0), ElemId(1)]],
            ..Default::default()
        };
        let mf = |rows: Vec<(f64, f64, f64)>| MemberForces {
            at: rows
                .into_iter()
                .map(|(p, q, m)| (p, [0.0, q, 0.0, 0.0, 0.0, m]))
                .collect(),
        };
        let member_forces = vec![
            (
                ElemId(0),
                mf(vec![
                    (0.0, 50_000.0, -200.0e6),
                    (0.5, 30_000.0, 20.0e6),
                    (1.0, 10_000.0, 100.0e6),
                ]),
            ),
            (
                ElemId(1),
                mf(vec![
                    (0.0, -10_000.0, 100.0e6),
                    (0.5, -30_000.0, 20.0e6),
                    (1.0, -50_000.0, -200.0e6),
                ]),
            ),
        ];

        let overrides = beam_group_overrides(&model, &member_forces);
        let ov = overrides.get(&ElemId(0)).expect("グループ所属");
        // 両要素が同じ合成値を共有する。
        assert_eq!(ov, overrides.get(&ElemId(1)).unwrap());
        // 全長 = 3000+3000。
        assert!((ov.length - 6000.0).abs() < 1e-9);
        // 端部モーメントは外端（要素0の pos0、要素1の pos1）。
        assert_eq!(ov.end_moments_z, Some((-200.0e6, -200.0e6)));
        // A式: M0 = (50k+50k)・6000/8 = 75e6、Mc_A = 75e6 − 200e6 < 0。
        // B式: グループ中央(3000mm)＝要素0の pos=1.0 の行 → +100e6。
        // 中央モーメント = max(|B|, Mc_A) に B の符号 → +100e6。
        assert!((ov.mid_moment_z.unwrap() - 100.0e6).abs() < 1e-3);
        // せん断スパン代表値: |M| 最大 200e6 の行の (200e6, 50e3)。
        let (m_rep, q_rep) = ov.shear_span.unwrap();
        assert!((m_rep - 200.0e6).abs() < 1e-3);
        assert!((q_rep - 50_000.0).abs() < 1e-6);
        // 剛域なし → 内法長 = 全長。
        assert!((ov.clear_length - 6000.0).abs() < 1e-9);

        // グループ未指定なら空。
        let mut model2 = model;
        model2.beam_groups.clear();
        assert!(beam_group_overrides(&model2, &member_forces).is_empty());
    }
}
