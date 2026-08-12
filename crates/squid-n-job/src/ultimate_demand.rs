//! 終局検定用の部材需要組み立て（GUI・MCP 共用）。
//!
//! QL（長期せん断）と Q0（単純梁せん断）は独立した `Option` 引数とする。
//! 一方の有無が他方の埋込を切り替えない（MCP が Q0 だけ足しても QL は変わらない）。

use std::collections::HashMap;

use squid_n_core::ids::ElemId;
use squid_n_design_jp::ultimate::MemberDemand;
use squid_n_element::beam::MemberForces;
use squid_n_solver::pushover::PushoverMemberResponse;

/// 静的内力の各部材について、せん断力 `|Q|` の最大値を QL マップにする。
///
/// GUI の静的終局分岐が、需要組み立て前に QL を明示渡すために使う。
pub fn q_long_map_from_member_forces(
    member_forces: &[(ElemId, MemberForces)],
) -> HashMap<ElemId, f64> {
    member_forces
        .iter()
        .map(|(id, mf)| {
            (
                *id,
                mf.at.iter().map(|(_, f)| f[1].abs()).fold(0.0, f64::max),
            )
        })
        .collect()
}

/// 静的内力から終局検定用の部材需要を組み立てる。
///
/// 軸力は始端値（圧縮正）、曲げは部材内の最大絶対値を採用する。
///
/// - `q_long_by_elem`: 長期せん断 QL [N]（絶対値）。`None` なら未設定。
/// - `q_simple_by_elem`: 単純梁せん断 Q0 [N]。`None` なら未設定。
///
/// 両引数は独立である。QL を内力から求める場合は呼び出し側で
/// [`q_long_map_from_member_forces`] を渡し、こちらでは自動埋込しない。
pub fn member_demand_from_static_forces(
    member_forces: &[(ElemId, MemberForces)],
    q_long_by_elem: Option<&HashMap<ElemId, f64>>,
    q_simple_by_elem: Option<&HashMap<ElemId, f64>>,
) -> Vec<(ElemId, MemberDemand)> {
    member_forces
        .iter()
        .filter_map(|(id, mf)| {
            let n_axial = mf.at.first().map(|(_, f)| f[0])?;
            let mz = mf.at.iter().map(|(_, f)| f[5].abs()).fold(0.0, f64::max);
            let my = mf.at.iter().map(|(_, f)| f[4].abs()).fold(0.0, f64::max);
            Some((
                *id,
                MemberDemand {
                    n_axial,
                    mz,
                    my,
                    q_long: q_long_by_elem.and_then(|m| m.get(id).copied()),
                    q_simple: q_simple_by_elem.and_then(|m| m.get(id).copied()),
                    ..Default::default()
                },
            ))
        })
        .collect()
}

/// 増分解析（プッシュオーバー）応答から終局検定用の部材需要を組み立てる。
///
/// 応答が空の場合は `None`（呼び出し側が静的応答へフォールバックする）。
///
/// - `q_long_by_elem`: 重力ケース集合から算定した長期せん断 QL。`None` なら未設定。
/// - `q_simple_by_elem`: 単純梁せん断 Q0。`None` なら未設定。
pub fn member_demand_from_pushover(
    member_response: &[PushoverMemberResponse],
    q_long_by_elem: Option<&HashMap<ElemId, f64>>,
    q_simple_by_elem: Option<&HashMap<ElemId, f64>>,
) -> Option<Vec<(ElemId, MemberDemand)>> {
    if member_response.is_empty() {
        return None;
    }
    Some(
        member_response
            .iter()
            .map(|r| {
                let mut d = MemberDemand::from_pushover(
                    r.axial,
                    r.m_strong,
                    r.m_weak,
                    r.shear_strong,
                    r.shear_weak,
                    r.rp,
                );
                d.q_long = q_long_by_elem.and_then(|m| m.get(&r.elem).copied());
                d.q_simple = q_simple_by_elem.and_then(|m| m.get(&r.elem).copied());
                (r.elem, d)
            })
            .collect(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_element::beam::MemberForces;

    fn mf(rows: &[(f64, f64, f64, f64)]) -> MemberForces {
        // (pos, N, Qy, Mz)
        MemberForces {
            at: rows
                .iter()
                .map(|&(p, n, q, m)| (p, [n, q, 0.0, 0.0, 0.0, m]))
                .collect(),
        }
    }

    #[test]
    fn static_both_none_leaves_ql_q0_unset() {
        let forces = vec![(
            ElemId(0),
            mf(&[
                (0.0, 1000.0, 50_000.0, 1.0e6),
                (1.0, 1000.0, 10_000.0, 2.0e6),
            ]),
        )];
        let demand = member_demand_from_static_forces(&forces, None, None);
        assert_eq!(demand.len(), 1);
        let (_, d) = &demand[0];
        assert!((d.n_axial - 1000.0).abs() < 1e-12);
        assert!((d.mz - 2.0e6).abs() < 1e-6);
        assert!(d.q_long.is_none());
        assert!(d.q_simple.is_none());
    }

    #[test]
    fn static_q0_only_does_not_imply_ql() {
        let forces = vec![(
            ElemId(0),
            mf(&[(0.0, 0.0, 50_000.0, 0.0), (1.0, 0.0, 10_000.0, 0.0)]),
        )];
        let mut q0 = HashMap::new();
        q0.insert(ElemId(0), 12_000.0);
        let demand = member_demand_from_static_forces(&forces, None, Some(&q0));
        let (_, d) = &demand[0];
        assert!(d.q_long.is_none(), "Q0 だけでは QL を埋めない");
        assert_eq!(d.q_simple, Some(12_000.0));
    }

    #[test]
    fn static_explicit_ql_and_q0() {
        let forces = vec![(
            ElemId(0),
            mf(&[(0.0, 0.0, 50_000.0, 0.0), (1.0, 0.0, 10_000.0, 0.0)]),
        )];
        let ql = q_long_map_from_member_forces(&forces);
        let mut q0 = HashMap::new();
        q0.insert(ElemId(0), 12_000.0);
        let demand = member_demand_from_static_forces(&forces, Some(&ql), Some(&q0));
        let (_, d) = &demand[0];
        assert_eq!(d.q_long, Some(50_000.0));
        assert_eq!(d.q_simple, Some(12_000.0));
    }

    #[test]
    fn pushover_empty_returns_none() {
        assert!(member_demand_from_pushover(&[], None, None).is_none());
    }

    #[test]
    fn pushover_maps_response_and_optional_ql_q0() {
        let resp = [PushoverMemberResponse {
            elem: ElemId(3),
            axial: 2000.0,
            m_strong: 3.0e6,
            m_weak: 1.0e6,
            shear_strong: 40_000.0,
            shear_weak: 5_000.0,
            rp: 0.01,
            horizontal_force: 0.0,
        }];
        let mut ql = HashMap::new();
        ql.insert(ElemId(3), 8_000.0);
        let mut q0 = HashMap::new();
        q0.insert(ElemId(3), 9_000.0);
        let demand = member_demand_from_pushover(&resp, Some(&ql), Some(&q0)).unwrap();
        assert_eq!(demand.len(), 1);
        let (id, d) = &demand[0];
        assert_eq!(*id, ElemId(3));
        assert!((d.n_axial - 2000.0).abs() < 1e-12);
        assert_eq!(d.q_long, Some(8_000.0));
        assert_eq!(d.q_simple, Some(9_000.0));
    }
}
