//! 終局（最終確定ステップ）時の部材別応答の算定。
//!
//! - [`compute_member_response`] — 部材端内力を局所座標へ射影し、強軸・弱軸の
//!   設計用曲げ・せん断と軸圧縮力、部材変形角 Rp を [`PushoverMemberResponse`]
//!   として求める
//! - [`record_member_step`] — ヒンジ詳細図用に 1 確定ステップ分の部材端応答
//!   （軸力・剛域フェイスの局所曲げ・弦からの材端回転）を全部材について記録する

use super::geom::{axial_compression, dot3, member_end_forces_at_face};
use super::types::{MemberStepState, PushoverMemberResponse};
use squid_n_core::dof::DofMap;
use squid_n_core::model::{ElementData, Model};
use squid_n_element::behavior::{Ctx, ElementBehavior, LocalVec};
use squid_n_element::transform::LocalFrame;

/// 部材の変形角 R [rad]を節点変位から算定する。
///
/// `disp` は `DofMap` アクティブ添字順の全自由節点変位。
pub(crate) fn member_rp_angle(
    model: &Model,
    dofmap: &DofMap,
    disp: &[f64],
    elem: &ElementData,
) -> f64 {
    if elem.nodes.len() < 2 {
        return 0.0;
    }
    let ni = elem.nodes[0].index();
    let nj = elem.nodes[1].index();
    let (Some(pi), Some(pj)) = (model.nodes.get(ni), model.nodes.get(nj)) else {
        return 0.0;
    };
    let length = model.member_length(elem);
    if length <= 0.0 {
        return 0.0;
    }
    let get = |node_index: usize, dof: usize| -> f64 {
        let g = node_index * 6 + dof;
        dofmap
            .active(g)
            .and_then(|a| disp.get(a as usize).copied())
            .unwrap_or(0.0)
    };
    let vertical = squid_n_core::geom::is_vertical_axis(pi.coord, pj.coord);
    if vertical {
        let dux = get(nj, 0) - get(ni, 0);
        let duy = get(nj, 1) - get(ni, 1);
        (dux * dux + duy * duy).sqrt() / length
    } else {
        (get(nj, 2) - get(ni, 2)).abs() / length
    }
}

/// 部材が伝達する加力方向の水平力 [N] を材端力から求める。
pub(crate) fn horizontal_force_in_dir(f: &LocalVec, n_nodes: usize, dir_idx: usize) -> f64 {
    let n = n_nodes.min(f.data.len() / 6);
    if n < 2 {
        return 0.0;
    }
    let half = n / 2;
    let sum = |range: std::ops::Range<usize>| -> f64 {
        range.map(|k| f.data[k * 6 + dir_idx]).sum::<f64>()
    };
    sum(0..half).abs().max(sum(half..n).abs())
}

/// ヒンジ詳細図用: 1 確定ステップ分の部材端応答（[`MemberStepState`]）を全部材に
/// ついて算定する（`model.elements` と同じ並び。2 節点の線材以外はゼロ埋め）。
pub(crate) fn record_member_step(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    total_disp: &[f64],
) -> Vec<MemberStepState> {
    let ctx = Ctx { model };
    model
        .elements
        .iter()
        .zip(behaviors)
        .map(|(elem, b)| {
            if elem.nodes.len() != 2 {
                return MemberStepState::default();
            }
            let ni = elem.nodes[0].index();
            let nj = elem.nodes[1].index();
            let (Some(pi), Some(pj)) = (model.nodes.get(ni), model.nodes.get(nj)) else {
                return MemberStepState::default();
            };
            let dx = [
                pj.coord[0] - pi.coord[0],
                pj.coord[1] - pi.coord[1],
                pj.coord[2] - pi.coord[2],
            ];
            let length = (dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]).sqrt();
            if length <= 0.0 {
                return MemberStepState::default();
            }
            let frame = LocalFrame::from_nodes(pi.coord, pj.coord, elem.local_axis.ref_vector);
            let ex = frame.rot[0];
            let ey = frame.rot[1];
            let ez = frame.rot[2];

            let f = b.internal_force(&ctx);
            let (my_i, mz_i, my_j, mz_j) = match member_end_forces_at_face(model, elem, &f.data) {
                Some(fl) => (fl[4], fl[5], fl[10], fl[11]),
                None => {
                    let m_i = [f.data[3], f.data[4], f.data[5]];
                    let m_j = [f.data[9], f.data[10], f.data[11]];
                    (dot3(m_i, ey), dot3(m_i, ez), dot3(m_j, ey), dot3(m_j, ez))
                }
            };
            let f_i = [f.data[0], f.data[1], f.data[2]];
            let f_j = [f.data[6], f.data[7], f.data[8]];
            let n = axial_compression(f_i, f_j, ex);

            let get = |node_index: usize, dof: usize| -> f64 {
                let g = node_index * 6 + dof;
                dofmap
                    .active(g)
                    .and_then(|a| total_disp.get(a as usize).copied())
                    .unwrap_or(0.0)
            };
            let u_i = [get(ni, 0), get(ni, 1), get(ni, 2)];
            let u_j = [get(nj, 0), get(nj, 1), get(nj, 2)];
            let r_i = [get(ni, 3), get(ni, 4), get(ni, 5)];
            let r_j = [get(nj, 3), get(nj, 4), get(nj, 5)];
            let chord_v = (dot3(u_j, ey) - dot3(u_i, ey)) / length;
            let chord_w = (dot3(u_j, ez) - dot3(u_i, ez)) / length;
            let ry_i = dot3(r_i, ey) + chord_w;
            let rz_i = dot3(r_i, ez) - chord_v;
            let ry_j = dot3(r_j, ey) + chord_w;
            let rz_j = dot3(r_j, ez) - chord_v;

            MemberStepState {
                n: n as f32,
                my_i: my_i as f32,
                mz_i: mz_i as f32,
                my_j: my_j as f32,
                mz_j: mz_j as f32,
                ry_i: ry_i as f32,
                rz_i: rz_i as f32,
                ry_j: ry_j as f32,
                rz_j: rz_j as f32,
            }
        })
        .collect()
}

/// 最終確定ステップの部材別応答（[`PushoverMemberResponse`]）を算定する。
pub(crate) fn compute_member_response(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    total_disp: &[f64],
    dir: crate::statics::analysis::SeismicDir,
) -> Vec<PushoverMemberResponse> {
    let dir_idx = match dir {
        crate::statics::analysis::SeismicDir::X => 0usize,
        crate::statics::analysis::SeismicDir::Y => 1usize,
    };
    let ctx = Ctx { model };
    let mut out = Vec::with_capacity(model.elements.len());
    for (elem, b) in model.elements.iter().zip(behaviors) {
        if elem.nodes.len() < 2 {
            continue;
        }
        if elem.nodes.len() != 2 {
            let f = b.internal_force(&ctx);
            out.push(PushoverMemberResponse {
                elem: elem.id,
                m_strong: 0.0,
                m_weak: 0.0,
                shear_strong: 0.0,
                shear_weak: 0.0,
                axial: 0.0,
                rp: 0.0,
                horizontal_force: horizontal_force_in_dir(&f, elem.nodes.len(), dir_idx),
            });
            continue;
        }
        let (Some(pi), Some(pj)) = (
            model.nodes.get(elem.nodes[0].index()),
            model.nodes.get(elem.nodes[1].index()),
        ) else {
            continue;
        };
        let frame = LocalFrame::from_nodes(pi.coord, pj.coord, elem.local_axis.ref_vector);
        let ex = frame.rot[0];
        let ey = frame.rot[1];
        let ez = frame.rot[2];

        let f = b.internal_force(&ctx);
        let f_i = [f.data[0], f.data[1], f.data[2]];
        let f_j = [f.data[6], f.data[7], f.data[8]];

        let (m_strong, m_weak) = match member_end_forces_at_face(model, elem, &f.data) {
            Some(fl) => (fl[5].abs().max(fl[11].abs()), fl[4].abs().max(fl[10].abs())),
            None => {
                let m_i = [f.data[3], f.data[4], f.data[5]];
                let m_j = [f.data[9], f.data[10], f.data[11]];
                (
                    dot3(m_i, ez).abs().max(dot3(m_j, ez).abs()),
                    dot3(m_i, ey).abs().max(dot3(m_j, ey).abs()),
                )
            }
        };
        let shear_strong = dot3(f_i, ey).abs().max(dot3(f_j, ey).abs());
        let shear_weak = dot3(f_i, ez).abs().max(dot3(f_j, ez).abs());
        let axial = axial_compression(f_i, f_j, ex);
        let rp = member_rp_angle(model, dofmap, total_disp, elem);
        let horizontal_force = horizontal_force_in_dir(&f, elem.nodes.len(), dir_idx);

        out.push(PushoverMemberResponse {
            elem: elem.id,
            m_strong,
            m_weak,
            shear_strong,
            shear_weak,
            axial,
            rp,
            horizontal_force,
        });
    }
    out
}
