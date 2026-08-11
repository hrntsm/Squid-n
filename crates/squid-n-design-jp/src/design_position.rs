//! 危険断面位置（断面検定を行う部材軸上の位置）。
//!
//! - [`design_positions`] — 1 部材の危険断面位置（正規化座標 \[0,1\]）
//! - [`is_near_design_position`] — 内力の評価位置が危険断面位置と一致するか
//!
//! GUI（`squid-n-app`）と MCP サーバ（`squid-n-mcp`）が同じ断面検定を別々の
//! 入口から実行するため、検定位置の規則はここに 1 つだけ置く。
//!
//! 要素側の**内力の評価断面**（`squid_n_element` の `eval_sections`）は、
//! ここに節点芯（0.0 / 1.0）を加えたものになる。検定は柱フェース位置で行い、
//! 節点芯の内力は検定に用いないため、両者は意図的に異なる。

use squid_n_core::model::{ElementData, Model};

/// 1 部材の危険断面位置を正規化座標 \[0,1\] で返す（設計書 §6.2.3）。
///
/// 既定は**柱フェース（i 端・j 端）と部材中央**の 3 点。フェース距離
/// （`RigidZone::face_i/face_j`）が未算定の端は節点芯（0.0 / 1.0）に一致する。
/// 部材付帯情報（ハンチ端・継手位置。剛性には影響しない）があれば、その
/// 追加検定位置も含める（§6.2.3「位置はユーザが追加・変更可能」）。
///
/// `geom_len` は部材の幾何長 \[mm\]（[`Model::member_length`]）。0 に縮退した
/// 部材は `[0.0, 0.5, 1.0]` を返す。返り値は昇順で、1e-9 以内の重複は畳む。
pub fn design_positions(elem: &ElementData, model: &Model, geom_len: f64) -> Vec<f64> {
    let mut xs = if geom_len > 1e-12 {
        let xi_i = (elem.rigid_zone.face_i_or_zero() / geom_len).clamp(0.0, 0.5 - 1e-9);
        let xi_j = (1.0 - elem.rigid_zone.face_j_or_zero() / geom_len).clamp(0.5 + 1e-9, 1.0);
        vec![xi_i, 0.5, xi_j]
    } else {
        vec![0.0, 0.5, 1.0]
    };
    if let Some(detail) = model.member_detail(elem.id) {
        xs.extend(detail.extra_check_positions(&elem.rigid_zone, geom_len));
    }
    xs.sort_by(|a, b| a.total_cmp(b));
    xs.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    xs
}

/// 内力の評価位置 `pos` が危険断面位置 `positions` のいずれかと一致するか
/// （1e-6 以内）。要素が返す評価断面には節点芯も含まれるため、検定側は
/// これで危険断面位置の行だけを拾う。
pub fn is_near_design_position(pos: f64, positions: &[f64]) -> bool {
    positions.iter().any(|p| (p - pos).abs() < 1e-6)
}
