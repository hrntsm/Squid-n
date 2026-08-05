//! 構面（軸組図・伏図）表示の 2D 専用処理。
//!
//! 既存の 3D ビューアは正射影のため、構面の法線方向へ正対させればそれがそのまま
//! 軸組図・伏図の見え方になる。本モジュールはそのための視線方向の決定と、図単体で
//! どの通り・どの階を見ているか判別するための基準線・目盛りの描画を担う。
//!
//! - [`view_direction`] — 構面の法線から、正対させる視線方向を選ぶ
//! - [`draw_frame_grid`] — 通り芯・階の基準線と名前を描く

use super::{CameraState, Projector};
use crate::theme;
use squid_n_core::frame::{Frame, FrameTarget};
use squid_n_core::model::{AxisGroupKind, Model};

/// 基準線の色（部材より沈めた細線）。
fn grid_stroke() -> egui::Stroke {
    egui::Stroke::new(0.8_f32, egui::Color32::from_black_alpha(48))
}

/// 通り名・階名のフォントサイズ [px]。
const LABEL_SIZE: f32 = 11.0;

/// 構面の法線から、正対させる視線方向（原点から視点位置へ向かうベクトル）を選ぶ。
///
/// 法線には向きの自由度（±）が残るが、どちらから見るかで図が左右反転する。
/// 軸組図として読めるよう、**画面の右がグローバル +X（構面内になければ +Y）** に
/// なる側を選ぶ。X 通りの軸組図では画面右が +Y、Y 通りの軸組図では画面右が +X、
/// 伏図では画面右が +X・上が +Y になる。
///
/// カメラはターンテーブル方式（[`CameraState::snap_to_direction`]）で、鉛直軸が
/// 常に画面の縦に保たれるため、鉛直な構面では画面の上がそのままグローバル Z になる。
pub(super) fn view_direction(normal: [f64; 3]) -> [f32; 3] {
    let n = [normal[0] as f32, normal[1] as f32, normal[2] as f32];
    let candidates = [n, [-n[0], -n[1], -n[2]]];
    let mut best = candidates[0];
    let mut best_score = f32::NEG_INFINITY;
    for cand in candidates {
        let mut cam = CameraState::default();
        cam.snap_to_direction(cand);
        // グローバル X・Y が画面横方向のどちら向きに写るか（画面右が正）。
        let rx = super::q_rotate(cam.rot, [1.0, 0.0, 0.0])[0];
        let ry = super::q_rotate(cam.rot, [0.0, 1.0, 0.0])[0];
        // X が構面内にあれば X の向きで、なければ Y の向きで判定する。
        let score = if rx.abs() > 0.5 { rx } else { ry };
        if score > best_score {
            best_score = score;
            best = cand;
        }
    }
    best
}

/// 構面の基準線（交差する通り・階のレベル）と名前を描く。
///
/// 全体表示の汎用グリッド（1m 方眼）の代わりに描く。方眼と基準線を同時に出すと
/// 線が二重になって読みづらいため、構面表示ではこちらだけを描く。
///
/// - **軸組図**（鉛直な構面）: 各階の床レベルに水平線を引いて左端に階名を、構面と
///   交差する通りの位置に鉛直線を引いて下端に通り名を置く。
/// - **伏図**（水平な構面）: 平行芯の各通りを平面上の直線として引き、端に通り名を置く。
///
/// `bbox` は構面に属する部材の外接直方体（`[min, max]`）。基準線はこの範囲に
/// 余白を加えた長さで引く。
pub(super) fn draw_frame_grid(
    painter: &egui::Painter,
    model: &Model,
    frame: &Frame,
    target: FrameTarget,
    bbox: ([f64; 3], [f64; 3]),
    proj: &Projector,
) {
    match target {
        FrameTarget::Story(_) => draw_plan_grid(painter, model, bbox, proj),
        FrameTarget::Axis { group, .. } => {
            draw_elevation_grid(painter, model, frame, group, bbox, proj)
        }
    }
}

/// 基準線を引く範囲の余白（外接直方体の対角長に対する割合）。
const MARGIN_RATIO: f64 = 0.08;

/// 外接直方体の対角長から余白を求める。
fn margin(bbox: &([f64; 3], [f64; 3])) -> f64 {
    let (lo, hi) = bbox;
    let d = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
    (d * MARGIN_RATIO).max(500.0)
}

/// 軸組図の基準線。階の床レベル（水平線）と、交差する通り（鉛直線）を描く。
fn draw_elevation_grid(
    painter: &egui::Painter,
    model: &Model,
    frame: &Frame,
    own_group: usize,
    bbox: ([f64; 3], [f64; 3]),
    proj: &Projector,
) {
    let (lo, hi) = bbox;
    let m = margin(&bbox);
    let stroke = grid_stroke();

    // 構面の面内水平方向 h ＝ 法線 × Z（鉛直な構面でのみ意味を持つ）。
    let n = frame.normal;
    let h = [n[1], -n[0], 0.0];
    let h_len = (h[0] * h[0] + h[1] * h[1]).sqrt();
    if h_len < 1e-9 {
        return;
    }
    let h = [h[0] / h_len, h[1] / h_len];
    // 構面上の基準点（外接直方体の中心を構面へ載せたもの）。面内位置は h 方向の
    // 座標で表し、両端へ余白を足して基準線を引く。
    let c = [(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5];
    let s_of = |x: f64, y: f64| (x - c[0]) * h[0] + (y - c[1]) * h[1];
    let p_at = |s: f64, z: f64| [c[0] + h[0] * s, c[1] + h[1] * s, z];
    let (s_lo, s_hi) = {
        // 外接直方体の 4 隅を面内座標へ写して範囲を採る。
        let corners = [
            (lo[0], lo[1]),
            (hi[0], lo[1]),
            (lo[0], hi[1]),
            (hi[0], hi[1]),
        ];
        let vals: Vec<f64> = corners.iter().map(|&(x, y)| s_of(x, y)).collect();
        (
            vals.iter().copied().fold(f64::INFINITY, f64::min) - m,
            vals.iter().copied().fold(f64::NEG_INFINITY, f64::max) + m,
        )
    };
    let (z_lo, z_hi) = (lo[2] - m, hi[2] + m);

    // 階の床レベル（水平線）と階名。
    for story in &model.stories {
        let z = story.elevation;
        if z < z_lo || z > z_hi {
            continue;
        }
        let a = proj.project(p_at(s_lo, z));
        let b = proj.project(p_at(s_hi, z));
        painter.line_segment([a, b], stroke);
        painter.text(
            a - egui::vec2(6.0, 0.0),
            egui::Align2::RIGHT_CENTER,
            &story.name,
            egui::FontId::proportional(LABEL_SIZE),
            theme::GRAY_600,
        );
    }

    // 構面と交差する通り（鉛直線）と通り名。自分自身のグループは、同じ向きの
    // 平行線なので交差せず、描いても構面と重なるだけなので除く。
    for (gi, group) in model.axes.iter().enumerate() {
        if gi == own_group {
            continue;
        }
        let AxisGroupKind::Parallel { origin, .. } = group.kind else {
            continue;
        };
        let Some(d) = group.kind.offset_dir() else {
            continue;
        };
        for ax in &group.axes {
            let Some(t) = ax.distance else { continue };
            // 通りの直線 p·d = t + origin·d と、構面（h 方向の直線）の交点を面内座標で求める。
            // 構面上の点は c + h·s なので、(c + h·s)·d = t + origin·d を s について解く。
            let denom = h[0] * d[0] + h[1] * d[1];
            if denom.abs() < 1e-9 {
                continue; // 構面と平行な通り（交差しない）
            }
            let rhs = t + origin[0] * d[0] + origin[1] * d[1];
            let s = (rhs - (c[0] * d[0] + c[1] * d[1])) / denom;
            if s < s_lo || s > s_hi {
                continue;
            }
            let a = proj.project(p_at(s, z_lo));
            let b = proj.project(p_at(s, z_hi));
            painter.line_segment([a, b], stroke);
            painter.text(
                a + egui::vec2(0.0, 6.0),
                egui::Align2::CENTER_TOP,
                &ax.name,
                egui::FontId::proportional(LABEL_SIZE),
                theme::GRAY_600,
            );
        }
    }
}

/// 伏図の基準線。平行芯の各通りを平面上の直線として描く。
fn draw_plan_grid(
    painter: &egui::Painter,
    model: &Model,
    bbox: ([f64; 3], [f64; 3]),
    proj: &Projector,
) {
    let (lo, hi) = bbox;
    let m = margin(&bbox);
    let stroke = grid_stroke();
    // 基準線は構面の標高に引く（伏図なので外接直方体の上面＝床レベル）。
    let z = hi[2];
    // 通りの直線を引く長さ（平面の対角長 + 余白）。
    let half = ((hi[0] - lo[0]).hypot(hi[1] - lo[1])) * 0.5 + m;
    let c = [(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5];

    for group in &model.axes {
        let AxisGroupKind::Parallel { origin, angle_deg } = group.kind else {
            continue;
        };
        let Some(d) = group.kind.offset_dir() else {
            continue;
        };
        // 芯線の方向（離れを測る向きを 90° 戻したもの）。
        let rad = angle_deg.to_radians();
        let dir = [rad.cos(), rad.sin()];
        for ax in &group.axes {
            let Some(t) = ax.distance else { continue };
            // 通りの直線上で、平面中心にもっとも近い点を基準に前後へ伸ばす。
            let base = [origin[0] + d[0] * t, origin[1] + d[1] * t];
            let along = (c[0] - base[0]) * dir[0] + (c[1] - base[1]) * dir[1];
            let mid = [base[0] + dir[0] * along, base[1] + dir[1] * along];
            // 平面の範囲から外れた通りは描かない（離れが範囲外）。
            let off = (mid[0] - c[0]).hypot(mid[1] - c[1]);
            if off > half {
                continue;
            }
            let a3 = [mid[0] - dir[0] * half, mid[1] - dir[1] * half, z];
            let b3 = [mid[0] + dir[0] * half, mid[1] + dir[1] * half, z];
            let (a, b) = (proj.project(a3), proj.project(b3));
            painter.line_segment([a, b], stroke);
            // 名前は線の両端のうち、画面の左上に近いほうへ置く（図の外側になる）。
            let anchor = if a.x + a.y <= b.x + b.y { a } else { b };
            painter.text(
                anchor,
                egui::Align2::CENTER_CENTER,
                &ax.name,
                egui::FontId::proportional(LABEL_SIZE),
                theme::GRAY_600,
            );
        }
    }
}

#[cfg(test)]
mod tests;
