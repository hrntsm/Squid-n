//! 節点単位の断面検定（柱梁接合部・パネルゾーン・冷間成形耐力比・耐震壁）の
//! 入力組み立て（令82条・各構造規準の断面検定）。
//!
//! `Model` と部材内力から入力を組み立てて一括実行する。

mod cold_formed;
mod common;
mod rc_joint;
mod src_panel;
mod steel_panel;
mod wall;

pub use self::common::ForcesAt;
use self::common::MemberInfo;

#[cfg(test)]
pub(crate) use crate::rc::wall::{rc_wall_shear_check, RcWallInput};
#[cfg(test)]
pub(crate) use crate::wall_opening::equivalent_opening;
#[cfg(test)]
pub(crate) use squid_n_core::model::ElementKind;

use crate::{CheckResult, LoadTerm};
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::Model;

/// モデルと部材内力から節点単位の検定を一括実行する。
///
/// 戻り値: `(節点, 種別ラベル, 検定結果)` のリスト。
///
/// 冷間成形角形鋼管の存在軸力に `NL + 1.5・NE` の割増を効かせたい場合は
/// [`collect_joint_checks_with_long`] を使う（本関数は割増なし＝当該ケースの
/// 軸力そのまま）。
pub fn collect_joint_checks(
    model: &Model,
    member_forces: &[(ElemId, ForcesAt<'_>)],
    term: LoadTerm,
) -> Vec<(NodeId, String, CheckResult)> {
    collect_joint_checks_with_long(model, member_forces, None, &[], term)
}

/// [`collect_joint_checks`] の長期内力付き版。
///
/// `long_member_forces` に長期（G+P）組合せの部材内力を渡すと、冷間成形
/// 角形鋼管の柱梁耐力比チェックの存在軸力を `N = NL + 1.5・NE`
/// （NE = 当該ケースの軸力 − NL）で算定する。None の場合は当該ケースの
/// 軸力をそのまま用いる。地震時組合せの結果を渡すことを想定する。
///
/// `panel_moments` には、仕口パネルをモデル化した接合部で解析が出力した
/// せん断モーメント `{MSX, MSY}` [N·mm] を節点ごとに渡す。該当する節点の
/// S 造パネルゾーン検定は、この値の絶対値の大きい方を設計用パネルモーメント
/// `pM` に用いる。空スライス、または該当節点が含まれない場合は、
/// 梁端モーメント・柱せん断から `pM` を組み立てる。**検定の実施自体はパネルのモデル化の有無に依らない**。
pub fn collect_joint_checks_with_long(
    model: &Model,
    member_forces: &[(ElemId, ForcesAt<'_>)],
    long_member_forces: Option<&[(ElemId, ForcesAt<'_>)]>,
    panel_moments: &[(NodeId, [f64; 2])],
    term: LoadTerm,
) -> Vec<(NodeId, String, CheckResult)> {
    let mut out = Vec::new();

    let mut members: Vec<MemberInfo<'_>> = Vec::new();
    for (eid, forces) in member_forces {
        let Some(elem) = model.element(*eid) else {
            continue;
        };
        if elem.nodes.len() < 2 {
            continue;
        }
        let sec = model.element_section(elem);
        let mat = model.element_material(elem);
        let rebar_mat = model.element_rebar_material(elem);
        let shear_mat = model.element_shear_rebar_material(elem);
        let steel_mat = model.element_steel_material(elem);
        let (Some(sec), Some(mat)) = (sec, mat) else {
            continue;
        };
        let length = model.member_length(elem);
        if length < 1e-9 {
            continue;
        }
        let (Some(p0), Some(p1)) = (
            model.nodes.get(elem.nodes[0].index()).map(|n| n.coord),
            model.nodes.get(elem.nodes[1].index()).map(|n| n.coord),
        ) else {
            continue;
        };
        members.push(MemberInfo {
            elem,
            sec,
            mat,
            rebar_mat,
            shear_mat,
            steel_mat,
            forces,
            kind: squid_n_core::structure_kind::structure_kind_of(Some(sec), Some(mat.category)),
            ez: ((p1[2] - p0[2]) / length).abs(),
            length,
        });
    }

    wall::check_walls(model, member_forces, &members, term, &mut out);

    for (ni, node) in model.nodes.iter().enumerate() {
        let nid = node.id;
        let _ = ni;
        let cols: Vec<&MemberInfo> = members
            .iter()
            .filter(|m| m.is_column() && m.elem.nodes.contains(&nid))
            .collect();
        let beams: Vec<&MemberInfo> = members
            .iter()
            .filter(|m| m.is_beam_horiz() && m.elem.nodes.contains(&nid))
            .collect();
        if cols.is_empty() || beams.is_empty() {
            continue;
        }

        let panel_moment = panel_moments.iter().find(|(n, _)| *n == nid).map(|(_, m)| {
            if m[0].abs() >= m[1].abs() {
                m[0]
            } else {
                m[1]
            }
        });

        rc_joint::check_rc_joint(&cols, &beams, nid, &mut out);
        steel_panel::check_s_panel(model, &cols, &beams, nid, panel_moment, &mut out);
        src_panel::check_src_panel(&cols, &beams, nid, term, &mut out);
        cold_formed::check_cold_formed(&cols, &beams, nid, long_member_forces, &mut out);
    }

    out
}

#[cfg(test)]
mod tests;
