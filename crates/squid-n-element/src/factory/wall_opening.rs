//! 壁要素のせん断剛性に乗じる開口低減率。
//!
//! - [`wall_opening_reduction`] — 開口低減 r = 1 − 1.25·√(開口面積/壁面積)

use squid_n_core::model::{ElementData, Model};

/// 壁要素のせん断剛性に乗じる開口低減率 r = 1 − 1.25·√(開口面積/壁面積)。
/// `Model::wall_attrs` に該当がない・開口ゼロ・寸法不定では 1.0（低減なし）。
pub(crate) fn wall_opening_reduction(data: &ElementData, model: &Model) -> f64 {
    let Some(attr) = model.wall_attrs.iter().find(|w| w.elem == data.id) else {
        return 1.0;
    };
    let opening_area = attr.total_opening_area_for(model.multi_opening_mode);
    if opening_area <= 0.0 {
        return 1.0;
    }
    let (l, h) = match crate::wall::wall_element::wall_element_geometry(data, model) {
        Some(g) => (g.lw, g.h),
        None => {
            let coords: Vec<[f64; 3]> = data
                .nodes
                .iter()
                .filter_map(|nid| model.nodes.get(nid.index()))
                .map(|n| n.coord)
                .collect();
            if coords.len() < 3 {
                return 1.0;
            }
            let mut l = 0.0_f64;
            for i in 0..coords.len() {
                for j in (i + 1)..coords.len() {
                    let dx = coords[i][0] - coords[j][0];
                    let dy = coords[i][1] - coords[j][1];
                    l = l.max((dx * dx + dy * dy).sqrt());
                }
            }
            let zs = coords.iter().map(|c| c[2]);
            let h = zs.clone().fold(f64::MIN, f64::max) - zs.fold(f64::MAX, f64::min);
            (l, h)
        }
    };
    let r0 = (opening_area / (l * h)).clamp(0.0, 1.0).sqrt();
    squid_n_core::rc_wall_capacity::wall_opening_reduction_stiffness(r0)
}
