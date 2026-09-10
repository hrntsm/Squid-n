//! 幾何ヘルパ。
//!
//! - [`dot3`] — 3 次元ベクトルの内積（`squid_n_core::geom::vec3::dot` の別名）
//! - [`axial_compression`] — 材端力から部材の軸方向圧縮力を算定
//! - [`member_end_forces_at_face`] — 材端力を局所座標・剛域フェイス位置へ変換

use squid_n_core::model::{ElementData, Model};
use squid_n_element::transform::LocalFrame;

pub(crate) use squid_n_core::geom::vec3::dot as dot3;

/// 部材の軸方向圧縮力 N_compress [N]を算定する。
///
/// 引張は 0 とし、両端のうち大きい方を部材の代表値とする。
pub(crate) fn axial_compression(f_i: [f64; 3], f_j: [f64; 3], ex: [f64; 3]) -> f64 {
    let from_i = dot3(f_i, ex).max(0.0);
    let from_j = (-dot3(f_j, ex)).max(0.0);
    from_i.max(from_j)
}

/// 部材の材端力を局所座標・剛域フェイス位置の 12 成分へ変換する.
///
/// 2 節点の線材専用。4 節点の耐震壁は `None` を返す。
pub(crate) fn member_end_forces_at_face(
    model: &Model,
    elem: &ElementData,
    f_global: &[f64],
) -> Option<[f64; 12]> {
    if elem.nodes.len() != 2 || f_global.len() < 12 {
        return None;
    }
    let pi = model.nodes.get(elem.nodes[0].index())?;
    let pj = model.nodes.get(elem.nodes[1].index())?;
    let frame = LocalFrame::from_nodes(pi.coord, pj.coord, elem.local_axis.ref_vector);
    let mut g = [0.0_f64; 12];
    g.copy_from_slice(&f_global[..12]);
    let local = frame.rotate_to_local(&g);
    let d = [
        pj.coord[0] - pi.coord[0],
        pj.coord[1] - pi.coord[1],
        pj.coord[2] - pi.coord[2],
    ];
    let length = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    let (li, lj) = squid_n_element::frame::rigid_arm::resolve_lengths(
        elem.rigid_zone.rigid_length_i(),
        elem.rigid_zone.rigid_length_j(),
        length,
    );
    Some(squid_n_element::frame::rigid_arm::to_flex_force(
        &local, li, lj,
    ))
}
