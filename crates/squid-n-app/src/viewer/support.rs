//! 従来の支持記号（矢印・円弧・凡例）。
//!
//! `viewer` ハブからの構造分割。アルゴリズム変更は行わない。

use crate::theme;

use super::{support_symbols, Projector};

/// 立体の支点記号を描くか。質点ビューでは常に出さない。
pub(super) fn supports_visible(lumped_view: bool, show_supports: bool) -> bool {
    !lumped_view && show_supports
}

/// 3D ビュー上での支持条件の分類。`Dof6Mask` のビットパターンを意味的にまとめる。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum SupportKind {
    /// 拘束なし（自由節点）
    Free,
    /// ピン支持（並進 3 自由度を拘束、回転は自由）
    Pinned,
    /// 固定支持（全 6 自由度を拘束）
    Fixed,
    /// ローラー支持（並進の一部のみ拘束、回転は自由）
    Roller,
    /// その他の部分拘束（上記以外の組み合わせ）
    Custom,
}

/// `Dof6Mask` を `SupportKind` へ分類する。
pub(super) fn support_kind(restraint: Dof6Mask) -> SupportKind {
    const FIXED_BITS: u8 = Dof6Mask::FIXED.0;
    const PINNED_BITS: u8 = Dof6Mask::PINNED.0;
    match restraint.0 {
        0 => SupportKind::Free,
        FIXED_BITS => SupportKind::Fixed,
        PINNED_BITS => SupportKind::Pinned,
        _ => {
            let translational = restraint.0 & 0b000111; // Ux, Uy, Uz
            let rotational = restraint.0 & 0b111000; // Rx, Ry, Rz
            if translational != 0 && rotational == 0 {
                SupportKind::Roller
            } else {
                SupportKind::Custom
            }
        }
    }
}

use squid_n_core::dof::{Dof, Dof6Mask};
use squid_n_core::geom::vec3::cross as cross3;

pub(super) fn draw_arrow(
    painter: &egui::Painter,
    from: egui::Pos2,
    to: egui::Pos2,
    color: egui::Color32,
) {
    let stroke = egui::Stroke::new(2.0_f32, color);
    painter.line_segment([from, to], stroke);
    let dir = to - from;
    let len = dir.length();
    if len < 1e-3 {
        return;
    }
    let ux = dir.x / len;
    let uy = dir.y / len;
    let nx = -uy;
    let ny = ux;
    const HEAD: f32 = 6.0;
    let base = egui::pos2(to.x - ux * HEAD, to.y - uy * HEAD);
    let left = egui::pos2(base.x + nx * HEAD * 0.5, base.y + ny * HEAD * 0.5);
    let right = egui::pos2(base.x - nx * HEAD * 0.5, base.y - ny * HEAD * 0.5);
    painter.line_segment([to, left], stroke);
    painter.line_segment([to, right], stroke);
}

/// 回転軸 `axis`（非零ベクトル想定）に直交する面内の正規直交基底 `(u, v)` を返す。
/// 円弧・渦巻（[`support_symbols::draw_rotational_spring`]）など、軸まわりの円周上に
/// 点を生成する描画で共有する。軸が退化している（ゼロベクトル）場合は `None`。
pub(super) fn axis_basis(axis: [f64; 3]) -> Option<([f64; 3], [f64; 3])> {
    let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    if n < 1e-12 {
        return None;
    }
    let axis = [axis[0] / n, axis[1] / n, axis[2] / n];
    let ref_vec = if axis[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let u_raw = cross3(axis, ref_vec);
    let un = (u_raw[0] * u_raw[0] + u_raw[1] * u_raw[1] + u_raw[2] * u_raw[2]).sqrt();
    if un < 1e-12 {
        return None;
    }
    let u = [u_raw[0] / un, u_raw[1] / un, u_raw[2] / un];
    let v = cross3(axis, u);
    Some((u, v))
}

/// 節点を中心に `axis` まわりの回転を示す円弧（全周）を描く。
pub(super) fn draw_rotation_arc(
    painter: &egui::Painter,
    proj: &Projector,
    center_world: [f64; 3],
    axis: [f64; 3],
    radius_world: f64,
    color: egui::Color32,
) {
    let Some((u, v)) = axis_basis(axis) else {
        return;
    };

    let stroke = egui::Stroke::new(1.5_f32, color);
    const N: usize = 32;
    let mut prev: Option<egui::Pos2> = None;
    for i in 0..=N {
        let theta = i as f64 / N as f64 * std::f64::consts::TAU;
        let c = theta.cos();
        let s = theta.sin();
        let pt = [
            center_world[0] + radius_world * (c * u[0] + s * v[0]),
            center_world[1] + radius_world * (c * u[1] + s * v[1]),
            center_world[2] + radius_world * (c * u[2] + s * v[2]),
        ];
        let cur = proj.project(pt);
        if let Some(p0) = prev {
            painter.line_segment([p0, cur], stroke);
        }
        prev = Some(cur);
    }
}

/// 支持条件シンボルを 3D ビューに描画する。
///
/// 固定されている並進自由度の方向へ軸色の矢印を引き、
/// 固定されている回転自由度の軸まわりに円弧を描く。
/// 軸色は X=赤 / Y=緑 / Z=青（§3-2 規約）で方向を直感的に判別できる。
///
/// 現在は全体座標系（X/Y/Z）の軸方向に描画する。将来的に節点ごとに局所座標系を
/// 導入した際は、この関数が参照する軸ベクトルを局所座標系の軸へ差し替えればよい。
pub(super) fn draw_support_symbol(
    painter: &egui::Painter,
    proj: &Projector,
    node_coord: [f64; 3],
    restraint: Dof6Mask,
    arrow_px: f32,
    arc_px: f32,
) {
    if support_kind(restraint) == SupportKind::Free {
        return;
    }
    // スクリーン上で arrow_px / arc_px になるようワールド長を逆算
    let arrow_world = arrow_px as f64 / proj.scale() as f64;
    let arc_world = arc_px as f64 / proj.scale() as f64;
    let origin = proj.project(node_coord);

    // 並進自由度: 固定方向へ軸色の矢印
    let translational: [(Dof, [f64; 3], egui::Color32); 3] = [
        (Dof::Ux, [1.0, 0.0, 0.0], theme::AXIS_X),
        (Dof::Uy, [0.0, 1.0, 0.0], theme::AXIS_Y),
        (Dof::Uz, [0.0, 0.0, 1.0], theme::AXIS_Z),
    ];
    for (dof, dir, color) in translational {
        if restraint.is_fixed(dof) {
            let end = [
                node_coord[0] + dir[0] * arrow_world,
                node_coord[1] + dir[1] * arrow_world,
                node_coord[2] + dir[2] * arrow_world,
            ];
            draw_arrow(painter, origin, proj.project(end), color);
        }
    }

    // 回転自由度: 軸まわりの円弧
    let rotational: [(Dof, [f64; 3], egui::Color32); 3] = [
        (Dof::Rx, [1.0, 0.0, 0.0], theme::AXIS_X),
        (Dof::Ry, [0.0, 1.0, 0.0], theme::AXIS_Y),
        (Dof::Rz, [0.0, 0.0, 1.0], theme::AXIS_Z),
    ];
    for (dof, axis, color) in rotational {
        if restraint.is_fixed(dof) {
            draw_rotation_arc(painter, proj, node_coord, axis, arc_world, color);
        }
    }
}

/// 支持条件シンボルの凡例をビュー左下に描く。
/// `has_diaphragm` が真のとき剛床マーク、`has_spring` が真のとき支点ばね、
/// `has_isolator` が真のとき免震支承の説明行を追加する（実際にモデル内に
/// 存在する種別のみ表示。既存の支持記号凡例と同じ方針）。
pub(super) fn draw_support_legend(
    painter: &egui::Painter,
    has_diaphragm: bool,
    has_spring: bool,
    has_isolator: bool,
) {
    let rect = painter.clip_rect();
    let x0 = rect.min.x + 10.0;
    let mut y0 = rect.max.y - 10.0;

    // 剛床マークの説明（面内拘束 Ux/Uy/Rz）を最下段へ追加する。
    if has_diaphragm {
        painter.text(
            egui::pos2(x0, y0),
            egui::Align2::LEFT_BOTTOM,
            "剛床マーク: 面内拘束 (Ux/Uy/Rz)",
            egui::FontId::proportional(11.0),
            theme::GRAY_600,
        );
        // 以降の支持条件凡例を 1 行分上へずらす。
        y0 -= 16.0;
    }

    // 免震支承マーカーの説明（実際に配置されている場合のみ）。
    if has_isolator {
        support_symbols::draw_isolator_marker(
            painter,
            egui::pos2(x0 + 10.0, y0 - 8.0),
            theme::ISOLATOR_TEAL,
        );
        painter.text(
            egui::pos2(x0 + 28.0, y0),
            egui::Align2::LEFT_BOTTOM,
            "免震支承",
            egui::FontId::proportional(11.0),
            theme::GRAY_600,
        );
        y0 -= 16.0;
    }

    // 支点ばねの説明（実際に設定されている場合のみ。回転→並進の順で 2 行）。
    if has_spring {
        support_symbols::draw_spiral_icon_2d(
            painter,
            egui::pos2(x0 + 10.0, y0 - 7.0),
            6.0,
            theme::AXIS_X,
        );
        painter.text(
            egui::pos2(x0 + 28.0, y0),
            egui::Align2::LEFT_BOTTOM,
            "回転ばね支持 (渦巻線、X赤/Y緑/Z青)",
            egui::FontId::proportional(11.0),
            theme::GRAY_600,
        );
        y0 -= 16.0;

        support_symbols::draw_translational_spring(
            painter,
            egui::pos2(x0, y0 - 6.0),
            egui::pos2(x0 + 20.0, y0 - 6.0),
            theme::AXIS_X,
        );
        painter.text(
            egui::pos2(x0 + 28.0, y0),
            egui::Align2::LEFT_BOTTOM,
            "並進ばね支持 (コイル線、X赤/Y緑/Z青)",
            egui::FontId::proportional(11.0),
            theme::GRAY_600,
        );
        y0 -= 16.0;
    }

    // タイトル
    painter.text(
        egui::pos2(x0, y0 - 30.0),
        egui::Align2::LEFT_BOTTOM,
        "支持条件",
        egui::FontId::proportional(13.0),
        theme::GRAY_700,
    );
    // 並進固定サンプル: 矢印
    let arrow_y = y0 - 16.0;
    draw_arrow(
        painter,
        egui::pos2(x0, arrow_y),
        egui::pos2(x0 + 20.0, arrow_y),
        theme::AXIS_X,
    );
    painter.text(
        egui::pos2(x0 + 28.0, y0 - 12.0),
        egui::Align2::LEFT_BOTTOM,
        "並進固定 (X赤/Y緑/Z青)",
        egui::FontId::proportional(11.0),
        theme::GRAY_600,
    );
    // 回転固定サンプル: 円
    let arc_y = y0;
    painter.circle_stroke(
        egui::pos2(x0 + 10.0, arc_y - 6.0),
        7.0,
        egui::Stroke::new(1.5_f32, theme::AXIS_X),
    );
    painter.text(
        egui::pos2(x0 + 28.0, y0),
        egui::Align2::LEFT_BOTTOM,
        "回転固定 (X赤/Y緑/Z青)",
        egui::FontId::proportional(11.0),
        theme::GRAY_600,
    );
}

#[cfg(test)]
mod tests {
    use super::supports_visible;

    #[test]
    fn supports_hidden_in_lumped_view_even_when_toggle_on() {
        assert!(!supports_visible(true, true));
        assert!(!supports_visible(true, false));
    }

    #[test]
    fn supports_follow_toggle_in_frame_view() {
        assert!(supports_visible(false, true));
        assert!(!supports_visible(false, false));
    }
}
