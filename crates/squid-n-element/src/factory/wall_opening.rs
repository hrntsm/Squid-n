//! 壁要素のせん断剛性に乗じる開口低減率。
//!
//! - [`wall_opening_reduction`] — RC 規準（耐震壁）の開口低減 r = 1 − 1.25·√(開口面積/壁面積)

use squid_n_core::model::{ElementData, Model};

/// 壁要素のせん断剛性に乗じる開口低減率 r = 1 − 1.25·√(開口面積/壁面積)
/// （RC規準（耐震壁）の開口低減。式の原典実装は
/// `squid-n-design-jp::wall_opening::opening_reduction_r`。element は design-jp に
/// 依存できないため、面積比による同値式をここで評価する）。
///
/// 壁面積は節点群の包絡寸法（最大水平距離 × 鉛直高さ）で近似する。
/// `Model::wall_attrs` に該当が無い・開口ゼロ・寸法不定では 1.0（低減なし）。
pub(crate) fn wall_opening_reduction(data: &ElementData, model: &Model) -> f64 {
    let Some(attr) = model.wall_attrs.iter().find(|w| w.elem == data.id) else {
        return 1.0;
    };
    // 複数開口の取り扱い（等価/包絡/自動判定）を適用した開口面積。
    // 包絡系モードでは包絡矩形の面積となり、生の面積和より大きくなり得る。
    let opening_area = attr.total_opening_area_for(model.multi_opening_mode);
    if opening_area <= 0.0 {
        return 1.0;
    }
    // 壁面積の分母は**壁エレメント要素と同じ幾何**（`wall_panel_geometry`）を用いる。
    // 壁長 lw は上下辺長さの平均（台形壁対応）、高さ h は上下辺中点間距離。
    // 従来は「全節点対の水平距離の最大（＝下辺長）」×「z の全幅」で近似しており、
    // 台形壁で開口周比の分母とせん断断面の壁長が食い違っていた。
    // 4 節点でない壁（フォールバック等価梁経路）は従来の包絡寸法で近似する。
    let (l, h) = match crate::wall_panel::wall_panel_geometry(data, model) {
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
    // r0 = √(開口面積/壁面積)、r1 = 1 − 1.25·r0（剛性用。式は core に集約）。
    let r0 = (opening_area / (l * h)).clamp(0.0, 1.0).sqrt();
    squid_n_core::rc_wall_capacity::wall_opening_reduction_stiffness(r0)
}
