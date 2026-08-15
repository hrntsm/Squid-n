//! 質点系の 3D 表示（球・階間ばね）。

use crate::app::App;
use crate::theme;
use squid_n_solver::analysis::SeismicDir;
use squid_n_solver::lumped_mass::LumpedMassResult;

use super::{playback, Projector, ViewMode};

pub(super) fn has_lumped(app: &App) -> bool {
    app.results
        .as_ref()
        .and_then(|r| r.lumped.as_ref())
        .is_some()
}

pub(super) fn is_lumped_view(mode: ViewMode) -> bool {
    matches!(mode, ViewMode::LumpedMode | ViewMode::LumpedTimeHistory)
}

/// 画面上の質点球半径 [pt]。面積が質量に比例するよう √(m/m_max) でスケールする。
/// 最大質量の階は 7、下限は 4（極小質量でも拾える大きさ）。
pub(crate) fn mass_marker_radius(mass: f64, mass_max: f64) -> f32 {
    const R_REF: f32 = 7.0;
    const R_MIN: f32 = 4.0;
    if mass_max <= 1e-12 {
        return R_REF;
    }
    let t = (mass.max(0.0) / mass_max).sqrt().clamp(0.0, 1.0);
    (R_REF * t as f32).max(R_MIN)
}

/// 骨組を重ねないときの串：全質点の平面位置を平均へ揃え、鉛直 1 列にする。
fn align_xy_to_mean(pts: &mut [[f64; 3]]) {
    let n = pts.len();
    if n == 0 {
        return;
    }
    let inv = 1.0 / n as f64;
    let cx = pts.iter().map(|p| p[0]).sum::<f64>() * inv;
    let cy = pts.iter().map(|p| p[1]).sum::<f64>() * inv;
    for p in pts {
        p[0] = cx;
        p[1] = cy;
    }
}

/// 各階質点の基準座標（上端床レベル）。
/// `align_vertical` が真なら平面位置を平均へ揃え、横ずれは変形だけになる。
fn rest_positions(app: &App, result: &LumpedMassResult, align_vertical: bool) -> Vec<[f64; 3]> {
    let lm = &result.model;
    let layers = app.model.layers();
    let n = lm.stories.len();
    let mut out = Vec::with_capacity(n);
    let mut z = app
        .model
        .stories
        .first()
        .map(|s| s.elevation)
        .unwrap_or(0.0);
    for i in 0..n {
        z += lm.stories[i].height;
        let xy = if lm.is_spatial() {
            lm.spatial[i].mass_xy
        } else {
            layers
                .get(i)
                .and_then(|l| {
                    app.model
                        .diaphragms_of(l.top)
                        .next()
                        .and_then(|d| app.model.nodes.get(d.master.index()))
                        .map(|n| [n.coord[0], n.coord[1]])
                })
                .unwrap_or([0.0, 0.0])
        };
        out.push([xy[0], xy[1], z]);
    }
    if align_vertical {
        align_xy_to_mean(&mut out);
    }
    out
}

fn disp_at(app: &App, result: &LumpedMassResult, mode: ViewMode, mode_idx: usize) -> Vec<[f64; 3]> {
    let n = result.model.stories.len();
    let zero = vec![[0.0; 3]; n];
    match mode {
        ViewMode::LumpedMode => {
            if result.model.is_spatial() {
                result
                    .modal
                    .shapes_xyz
                    .get(mode_idx)
                    .cloned()
                    .unwrap_or(zero)
            } else {
                let shape = result.modal.shapes.get(mode_idx);
                let mut d = zero;
                if let Some(s) = shape {
                    for (i, slot) in d.iter_mut().enumerate() {
                        let v = s.get(i).copied().unwrap_or(0.0);
                        match result.model.dir {
                            SeismicDir::X => slot[0] = v,
                            SeismicDir::Y => slot[1] = v,
                        }
                    }
                }
                d
            }
        }
        ViewMode::LumpedTimeHistory => {
            let Some(th) = result.response.as_ref() else {
                return zero;
            };
            if th.floor_disp.is_empty() {
                return zero;
            }
            let frame = playback::frame_at_time(&th.time, app.th_play_time)
                .min(th.floor_disp.len().saturating_sub(1));
            th.floor_disp.get(frame).cloned().unwrap_or(zero)
        }
        _ => zero,
    }
}

fn peak_xy<'a>(disps: impl Iterator<Item = &'a [f64; 3]>) -> f64 {
    disps
        .map(|d| d[0].abs().max(d[1].abs()))
        .fold(0.0_f64, f64::max)
}

/// 質点変位のピークがモデル対角の 10% になる自動倍率に、手動係数を掛ける。
/// 時刻歴は全フレームのピークで固定し、振幅の小さい時刻で倍率が発散しないようにする。
pub(super) fn display_scale(app: &App, mode: ViewMode, mode_idx: usize, model_size: f64) -> f64 {
    let Some(result) = app.results.as_ref().and_then(|r| r.lumped.as_ref()) else {
        return 0.0;
    };
    let peak = match mode {
        ViewMode::LumpedMode => peak_xy(disp_at(app, result, mode, mode_idx).iter()),
        ViewMode::LumpedTimeHistory => result
            .response
            .as_ref()
            .map(|th| peak_xy(th.floor_disp.iter().flat_map(|frame| frame.iter())))
            .unwrap_or(0.0),
        _ => 0.0,
    };
    if peak <= 1e-12 || model_size <= 1e-12 {
        return 0.0;
    }
    model_size * 0.1 / peak * f64::from(app.deform_scale_factor)
}

const GHOST_DASH: f32 = 6.0;
const GHOST_GAP: f32 = 4.0;
/// 変形前（基準位置）の線・塗りのアルファ。変形後と弁別できるよう低くする。
const GHOST_LINE_ALPHA: u8 = 90;
const GHOST_FILL_ALPHA: u8 = 55;

fn draw_stick_springs(
    painter: &egui::Painter,
    pts: &[egui::Pos2],
    base: egui::Pos2,
    stroke: egui::Stroke,
    dashed: bool,
) {
    if pts.is_empty() {
        return;
    }
    let mut segs = Vec::with_capacity(pts.len());
    segs.push([base, pts[0]]);
    for w in pts.windows(2) {
        segs.push([w[0], w[1]]);
    }
    if dashed {
        for seg in segs {
            painter.extend(egui::Shape::dashed_line(
                &seg,
                stroke,
                GHOST_DASH,
                GHOST_GAP,
            ));
        }
    } else {
        for seg in segs {
            painter.line_segment(seg, stroke);
        }
    }
}

fn draw_stick_masses(
    painter: &egui::Painter,
    pts: &[egui::Pos2],
    radii: &[f32],
    fill: egui::Color32,
    stroke: egui::Stroke,
) {
    for (i, &p) in pts.iter().enumerate() {
        let r = radii.get(i).copied().unwrap_or(7.0);
        painter.circle_filled(p, r, fill);
        painter.circle_stroke(p, r, stroke);
    }
}

pub(super) fn draw(
    painter: &egui::Painter,
    app: &App,
    proj: &Projector<'_>,
    mode: ViewMode,
    mode_idx: usize,
    model_size: f64,
) {
    let Some(result) = app.results.as_ref().and_then(|r| r.lumped.as_ref()) else {
        return;
    };
    let rest = rest_positions(app, result, !app.lumped_show_frame);
    if rest.is_empty() {
        return;
    }
    let disp = disp_at(app, result, mode, mode_idx);
    let scale = display_scale(app, mode, mode_idx, model_size);
    let rest_pts: Vec<egui::Pos2> = rest.iter().map(|&p| proj.project(p)).collect();
    let pts: Vec<egui::Pos2> = rest
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let d = disp.get(i).copied().unwrap_or([0.0; 3]);
            proj.project([p[0] + d[0] * scale, p[1] + d[1] * scale, p[2]])
        })
        .collect();

    let spring = theme::DATA_BLUE;
    let mass_color = theme::PARETO_RED;
    let mass_max = result
        .model
        .stories
        .iter()
        .map(|s| s.mass)
        .fold(0.0_f64, f64::max);
    let radii: Vec<f32> = result
        .model
        .stories
        .iter()
        .map(|s| mass_marker_radius(s.mass, mass_max))
        .collect();
    let base_z = app
        .model
        .stories
        .first()
        .map(|s| s.elevation)
        .unwrap_or(0.0);
    let base = proj.project([rest[0][0], rest[0][1], base_z]);

    // 質点モードは変形前を破線・高透過で先に描き、基準位置からの変化が読めるようにする。
    if mode == ViewMode::LumpedMode && scale > 1e-12 {
        draw_stick_springs(
            painter,
            &rest_pts,
            base,
            egui::Stroke::new(1.5, theme::translucent(spring, GHOST_LINE_ALPHA)),
            true,
        );
        draw_stick_masses(
            painter,
            &rest_pts,
            &radii,
            theme::translucent(mass_color, GHOST_FILL_ALPHA),
            egui::Stroke::new(1.0, theme::translucent(theme::GRAY_900, GHOST_LINE_ALPHA)),
        );
    }

    draw_stick_springs(
        painter,
        &pts,
        base,
        egui::Stroke::new(2.5, theme::translucent(spring, 220)),
        false,
    );
    draw_stick_masses(
        painter,
        &pts,
        &radii,
        mass_color,
        egui::Stroke::new(1.0, theme::GRAY_900),
    );
    if result.model.is_spatial() {
        for (i, &p) in pts.iter().enumerate() {
            let r = radii.get(i).copied().unwrap_or(7.0);
            let d = disp.get(i).copied().unwrap_or([0.0; 3]);
            let ang = d[2] * scale * 0.25;
            let tick = egui::vec2(ang.cos() as f32, -ang.sin() as f32) * (r * 1.7);
            painter.line_segment([p, p + tick], egui::Stroke::new(1.5, theme::BEST_YELLOW));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{align_xy_to_mean, mass_marker_radius};

    #[test]
    fn align_xy_to_mean_collapses_plan_to_centroid() {
        let mut pts = [
            [0.0, 0.0, 3000.0],
            [8000.0, 0.0, 6000.0],
            [4000.0, 2000.0, 9000.0],
        ];
        align_xy_to_mean(&mut pts);
        assert!((pts[0][0] - 4000.0).abs() < 1e-9);
        assert!((pts[0][1] - 2000.0 / 3.0).abs() < 1e-9);
        assert!((pts[1][0] - pts[0][0]).abs() < 1e-12);
        assert!((pts[1][1] - pts[0][1]).abs() < 1e-12);
        assert!((pts[2][0] - pts[0][0]).abs() < 1e-12);
        assert_eq!(pts[0][2], 3000.0);
        assert_eq!(pts[2][2], 9000.0);
    }

    #[test]
    fn mass_marker_radius_scales_with_sqrt_mass() {
        assert!((mass_marker_radius(10.0, 10.0) - 7.0).abs() < 1e-6);
        let mid = mass_marker_radius(5.0, 10.0);
        assert!((mid - 7.0 * 0.5_f32.sqrt()).abs() < 1e-5);
        assert!((mass_marker_radius(0.01, 10.0) - 4.0).abs() < 1e-6);
        assert!((mass_marker_radius(1.0, 0.0) - 7.0).abs() < 1e-6);
    }
}
