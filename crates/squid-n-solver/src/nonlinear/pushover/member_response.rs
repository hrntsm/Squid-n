//! 終局（最終確定ステップ）時の部材別応答の算定。
//!
//! - [`compute_member_response`] — 部材端内力を局所座標へ射影し、強軸・弱軸の
//!   設計用曲げ・せん断と軸圧縮力、部材変形角 Rp を [`PushoverMemberResponse`]
//!   として求める

use super::geom::{axial_compression, dot3};
use super::types::PushoverMemberResponse;
use squid_n_core::dof::DofMap;
use squid_n_core::model::{ElementData, Model};
use squid_n_element::behavior::{Ctx, ElemState, ElementBehavior};
use squid_n_element::transform::LocalFrame;

/// 部材の変形角 R [rad]（弦回転角＝層間変形角相当）を最終確定変位から算定する。
///
/// [`crate::strength_loss`] の `member_drift_angle` と同じ規則（鉛直材は材端の
/// 水平相対変位/材長、水平材は鉛直相対変位/材長）。`disp` は `DofMap` アクティブ
/// 添字順の全自由節点変位（プッシュオーバー最終ステップの `total_disp`）。
fn member_rp_angle(model: &Model, dofmap: &DofMap, disp: &[f64], elem: &ElementData) -> f64 {
    if elem.nodes.len() < 2 {
        return 0.0;
    }
    let ni = elem.nodes[0].index();
    let nj = elem.nodes[1].index();
    let (Some(pi), Some(pj)) = (model.nodes.get(ni), model.nodes.get(nj)) else {
        return 0.0;
    };
    let dx = pj.coord[0] - pi.coord[0];
    let dy = pj.coord[1] - pi.coord[1];
    let dz = pj.coord[2] - pi.coord[2];
    let length = (dx * dx + dy * dy + dz * dz).sqrt();
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
    let vertical = dz.abs() > (dx.abs() + dy.abs()) * 0.5;
    if vertical {
        let dux = get(nj, 0) - get(ni, 0);
        let duy = get(nj, 1) - get(ni, 1);
        (dux * dux + duy * duy).sqrt() / length
    } else {
        (get(nj, 2) - get(ni, 2)).abs() / length
    }
}

/// 最終確定ステップの部材別応答（[`PushoverMemberResponse`]）を算定する。
///
/// 各部材の材端内力（`ElementBehavior::internal_force` のグローバル成分）を
/// 局所座標系（`LocalFrame`）へ射影し、強軸（局所 z まわり Mz・せん断 Vy）・
/// 弱軸（局所 y まわり My・せん断 Vz）の設計用応力と軸圧縮力、部材変形角 Rp を
/// 部材ごとに求める（曲げ・せん断は両端の最大絶対値）。
pub(crate) fn compute_member_response(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    total_disp: &[f64],
    dir: crate::analysis::SeismicDir,
) -> Vec<PushoverMemberResponse> {
    let dir_idx = match dir {
        crate::analysis::SeismicDir::X => 0usize,
        crate::analysis::SeismicDir::Y => 1usize,
    };
    let state = ElemState::default();
    let ctx = Ctx { model };
    let mut out = Vec::with_capacity(model.elements.len());
    for (elem, b) in model.elements.iter().zip(behaviors) {
        if elem.nodes.len() < 2 {
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

        let f = b.internal_force(&state, &ctx);
        let f_i = [f.data[0], f.data[1], f.data[2]];
        let m_i = [f.data[3], f.data[4], f.data[5]];
        let f_j = [f.data[6], f.data[7], f.data[8]];
        let m_j = [f.data[9], f.data[10], f.data[11]];

        let m_strong = dot3(m_i, ez).abs().max(dot3(m_j, ez).abs());
        let m_weak = dot3(m_i, ey).abs().max(dot3(m_j, ey).abs());
        let shear_strong = dot3(f_i, ey).abs().max(dot3(f_j, ey).abs());
        let shear_weak = dot3(f_i, ez).abs().max(dot3(f_j, ez).abs());
        let axial = axial_compression(f_i, f_j, ex);
        let rp = member_rp_angle(model, dofmap, total_disp, elem);
        // 加力方向の水平力（βu の分子集計用）。材端力の載荷方向成分は釣合いにより
        // 両端で符号が反転するため、絶対値の大きい方を部材の代表値とする。
        let horizontal_force = f_i[dir_idx].abs().max(f_j[dir_idx].abs());

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
