//! 断面検定 QD1 用の単純梁せん断 Q0 と、重力ケース選択（GUI・MCP 共用）。

use std::collections::HashMap;

use squid_n_core::ids::{ElemId, LoadCaseId};
use squid_n_core::model::{LoadCaseKind, MemberLoadKind, Model, LL_FRAME_CASE_NAME};
use squid_n_element::frame::beam::MemberForces;
use squid_n_load::self_weight::SELF_WEIGHT_AUTO_LOAD_CASE_NAME;

/// 地震用重量・QD 用 Q0 に使う重力ケース ID 列。
/// 自重専用ケースと骨組用積載ケースは除外し、種別未設定なら先頭ケースのみを用いる。
pub fn gravity_case_ids_for_seismic_weight(model: &Model) -> Vec<LoadCaseId> {
    let any_kind_set = model
        .load_cases
        .iter()
        .any(|lc| lc.kind != LoadCaseKind::Other);
    if !any_kind_set {
        return model.load_cases.first().map(|c| c.id).into_iter().collect();
    }

    let mut result: Vec<LoadCaseId> = model
        .load_cases
        .iter()
        .filter(|lc| lc.kind == LoadCaseKind::Dead && lc.name != SELF_WEIGHT_AUTO_LOAD_CASE_NAME)
        .map(|lc| lc.id)
        .collect();

    let live_seismic: Vec<LoadCaseId> = model
        .load_cases
        .iter()
        .filter(|lc| lc.kind == LoadCaseKind::LiveSeismic)
        .map(|lc| lc.id)
        .collect();
    if !live_seismic.is_empty() {
        result.extend(live_seismic);
    } else {
        result.extend(
            model
                .load_cases
                .iter()
                .filter(|lc| lc.kind == LoadCaseKind::Live && lc.name != LL_FRAME_CASE_NAME)
                .map(|lc| lc.id),
        );
    }

    result
}

/// 1 荷重ケースの部材荷重から、単純梁支持の端部せん断 Q0 [N] を算定する。
/// Q0 は両端反力の大きい方。
pub fn simple_beam_q0_by_elem(model: &Model, lc: LoadCaseId) -> HashMap<ElemId, f64> {
    let mut acc: HashMap<ElemId, (f64, f64)> = HashMap::new();
    let Some(case) = model.load_cases.iter().find(|c| c.id == lc) else {
        return HashMap::new();
    };
    for ml in &case.member {
        let Some(elem) = model.element(ml.elem) else {
            continue;
        };
        if elem.nodes.len() < 2 {
            continue;
        }
        let (Some(n0), Some(n1)) = (
            model.nodes.get(elem.nodes[0].index()),
            model.nodes.get(elem.nodes[elem.nodes.len() - 1].index()),
        ) else {
            continue;
        };
        let dx = [
            n1.coord[0] - n0.coord[0],
            n1.coord[1] - n0.coord[1],
            n1.coord[2] - n0.coord[2],
        ];
        let l = (dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]).sqrt();
        if l <= 0.0 {
            continue;
        }
        let e = [dx[0] / l, dx[1] / l, dx[2] / l];
        let dn = (ml.dir[0] * ml.dir[0] + ml.dir[1] * ml.dir[1] + ml.dir[2] * ml.dir[2]).sqrt();
        if dn <= 0.0 {
            continue;
        }
        let d = [ml.dir[0] / dn, ml.dir[1] / dn, ml.dir[2] / dn];
        let ax = d[0] * e[0] + d[1] * e[1] + d[2] * e[2];
        let trans = (1.0 - ax * ax).max(0.0).sqrt();
        if trans <= 1e-12 {
            continue;
        }
        let (w_total, x_bar) = match ml.kind {
            MemberLoadKind::Point { a, p } => (p.abs(), a.clamp(0.0, l)),
            MemberLoadKind::Distributed { a, b, w1, w2 } => {
                let (a, b) = (a.clamp(0.0, l), b.clamp(0.0, l));
                if b <= a {
                    continue;
                }
                let w_sum = w1 + w2;
                let total = w_sum / 2.0 * (b - a);
                let xb = if w_sum.abs() > 1e-12 {
                    a + (b - a) * (w1 + 2.0 * w2) / (3.0 * w_sum)
                } else {
                    (a + b) / 2.0
                };
                (total.abs(), xb)
            }
        };
        let entry = acc.entry(ml.elem).or_insert((0.0, 0.0));
        entry.0 += trans * w_total * (l - x_bar) / l;
        entry.1 += trans * w_total * x_bar / l;
    }
    acc.into_iter()
        .map(|(k, (ri, rj))| (k, ri.max(rj)))
        .collect()
}

/// [`gravity_case_ids_for_seismic_weight`] の全ケースについて Q0 を加算する。
pub fn simple_beam_q0_by_gravity_cases(model: &Model) -> HashMap<ElemId, f64> {
    let mut map: HashMap<ElemId, f64> = HashMap::new();
    for lc in gravity_case_ids_for_seismic_weight(model) {
        for (id, q) in simple_beam_q0_by_elem(model, lc) {
            *map.entry(id).or_insert(0.0) += q;
        }
    }
    map
}

/// 複数ケースの部材内力を位置ごとに加算する（線形重ね合わせ）。
/// 近傍位置（絶対差 1e-9 以内）は足し合わせ、欠ける側は 0 とみなす。
pub fn sum_member_forces_lists(
    lists: &[Vec<(ElemId, MemberForces)>],
) -> Vec<(ElemId, MemberForces)> {
    let mut by_elem: HashMap<ElemId, Vec<(f64, [f64; 6])>> = HashMap::new();
    const POS_EPS: f64 = 1e-9;
    for list in lists {
        for (id, mf) in list {
            let entry = by_elem.entry(*id).or_default();
            for (p, f) in &mf.at {
                if let Some((_, acc)) = entry.iter_mut().find(|(q, _)| (*q - *p).abs() <= POS_EPS) {
                    for i in 0..6 {
                        acc[i] += f[i];
                    }
                } else {
                    entry.push((*p, *f));
                }
            }
        }
    }
    let mut out: Vec<(ElemId, MemberForces)> = by_elem
        .into_iter()
        .map(|(id, mut at)| {
            at.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            (id, MemberForces { at })
        })
        .collect();
    out.sort_by_key(|(id, _)| id.0);
    out
}

/// [`gravity_case_ids_for_seismic_weight`] と同じ集合の解析済み内力を加算する。
///
/// `force_of` が `None` を返すケースは飛ばす。1 件も取れなければ `None`。
pub fn sum_analyzed_gravity_member_forces<F>(
    model: &Model,
    mut force_of: F,
) -> Option<Vec<(ElemId, MemberForces)>>
where
    F: FnMut(LoadCaseId) -> Option<Vec<(ElemId, MemberForces)>>,
{
    let lists: Vec<_> = gravity_case_ids_for_seismic_weight(model)
        .into_iter()
        .filter_map(&mut force_of)
        .collect();
    if lists.is_empty() {
        None
    } else {
        Some(sum_member_forces_lists(&lists))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sum_member_forces_merges_near_positions() {
        let a = vec![(
            ElemId(1),
            MemberForces {
                at: vec![(0.0, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])],
            },
        )];
        let b = vec![(
            ElemId(1),
            MemberForces {
                at: vec![(1e-12, [2.0, 0.0, 0.0, 0.0, 0.0, 0.0])],
            },
        )];
        let sum = sum_member_forces_lists(&[a, b]);
        assert_eq!(sum.len(), 1);
        assert_eq!(sum[0].1.at.len(), 1);
        assert!((sum[0].1.at[0].1[0] - 3.0).abs() < 1e-12);
    }
}
