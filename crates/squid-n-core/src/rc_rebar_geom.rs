//! RC 配筋幾何の共通算定。

use crate::error::RebarGeometryError;
use crate::section_shape::{
    BarSet, RcBeamRebar, RcCircleColumnRebar, RcRebar, RcRectColumnRebar, RebarPoint, ShearBar,
};

/// 隣接主筋のあき k' [mm]（`max(25, 1.5・dia)`）。
fn clear_mm(dia: f64) -> f64 {
    25.0_f64.max(1.5 * dia)
}

/// 隣接主筋中心間の最小距離 s = dia + k' [mm]。
fn center_spacing_mm(dia: f64) -> f64 {
    dia + clear_mm(dia)
}

/// 多段配筋の段間あき k' [mm]（`max(25, 1.5・dia)`）。
pub fn rebar_layer_clear(main: &BarSet) -> f64 {
    clear_mm(main.dia)
}

/// 多段配筋の段中心間距離 s = dia + k' [mm]。
pub fn rebar_layer_spacing(main: &BarSet) -> f64 {
    center_spacing_mm(main.dia)
}

/// 縁から第 `layer` 段（0 始まり）の主筋中心までの距離 [mm]。
pub fn rebar_layer_depth_from_edge(cover: f64, shear_dia: f64, main: &BarSet, layer: u32) -> f64 {
    let k1 = cover + shear_dia + main.dia / 2.0;
    if layer == 0 {
        return k1;
    }
    k1 + layer as f64 * rebar_layer_spacing(main)
}

/// 引張縁 → 引張筋重心までの距離 dt [mm]。
pub fn tension_dt(cover: f64, shear_dia: f64, main: &BarSet) -> f64 {
    let layers = main.layers.max(1);
    if layers == 1 {
        return rebar_layer_depth_from_edge(cover, shear_dia, main, 0);
    }
    let d0 = rebar_layer_depth_from_edge(cover, shear_dia, main, 0);
    let d_last = rebar_layer_depth_from_edge(cover, shear_dia, main, layers - 1);
    0.5 * (d0 + d_last)
}

/// 断面の主筋（せい方向 `main_x`）の引張筋重心位置 dt [mm]。
pub fn rebar_tension_dt(rebar: &RcRebar) -> f64 {
    tension_dt(rebar.cover, rebar.shear.dia, &rebar.main_x)
}

/// 有効せい d_eff = D − dt [mm]。
pub fn tension_effective_depth(d: f64, cover: f64, shear_dia: f64, main: &BarSet) -> f64 {
    (d - tension_dt(cover, shear_dia, main)).max(0.0)
}

/// 有効せい d_eff = D − dt [mm]。
pub fn rebar_effective_depth(d: f64, rebar: &RcRebar) -> f64 {
    (d - rebar_tension_dt(rebar)).max(0.0)
}

/// せん断補強筋比 pw。pitch<=0 のときは 0。
pub fn pw_ratio(shear: &ShearBar, b: f64) -> f64 {
    if shear.pitch <= 0.0 || b <= 0.0 {
        return 0.0;
    }
    crate::section_shape::shear_legs_area(shear) / (b * shear.pitch)
}

/// 浮動小数の比較許容（`center_spacing_mm` に対する相対値）。
const EPS_REL: f64 = 1e-9;

/// 中心間距離 `actual` が最小距離 `required` 以上か（相対許容付き）。
fn spacing_ok(actual: f64, required: f64) -> bool {
    actual + required * EPS_REL >= required
}

/// 寸法が有限かつ正か。違反時は [`RebarGeometryError::InvalidDimension`]。
fn check_positive(field: &'static str, value: f64) -> Result<(), RebarGeometryError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(RebarGeometryError::InvalidDimension { field });
    }
    Ok(())
}

/// 寸法が有限かつ 0 以上か。違反時は [`RebarGeometryError::InvalidDimension`]。
fn check_non_negative(field: &'static str, value: f64) -> Result<(), RebarGeometryError> {
    if !value.is_finite() || value < 0.0 {
        return Err(RebarGeometryError::InvalidDimension { field });
    }
    Ok(())
}

/// 幅方向の配筋位置 [mm]。`count==1` は中心、`count>=2` は `±half` を含む等間隔。
fn distribute(count: u32, half: f64) -> Vec<f64> {
    if count == 1 {
        return vec![0.0];
    }
    let n = count as usize;
    (0..n)
        .map(|i| -half + 2.0 * half * i as f64 / (n - 1) as f64)
        .collect()
}

/// 全点の中心間距離が `min_spacing` 以上か。同一座標も距離 0 として拒否する。
fn ensure_min_spacing(points: &[RebarPoint], min_spacing: f64) -> Result<(), RebarGeometryError> {
    for i in 0..points.len() {
        for j in (i + 1)..points.len() {
            let dx = points[i].x - points[j].x;
            let dy = points[i].y - points[j].y;
            if !spacing_ok((dx * dx + dy * dy).sqrt(), min_spacing) {
                return Err(RebarGeometryError::SpacingOrSymmetry);
            }
        }
    }
    Ok(())
}

/// 全点が断面外形 `b`×`d` [mm] 内か。
fn ensure_in_bounds(points: &[RebarPoint], b: f64, d: f64) -> Result<(), RebarGeometryError> {
    let half_b = b / 2.0;
    let half_d = d / 2.0;
    for p in points {
        if p.x.abs() > half_b + half_b * EPS_REL || p.y.abs() > half_d + half_d * EPS_REL {
            return Err(RebarGeometryError::OutOfBounds);
        }
    }
    Ok(())
}

/// 配筋線上の主筋座標 [mm]。`intersections` は昇順の交点座標。
///
/// 交点を含め、余剰本数は中央ギャップ優先・中心対称なギャップ対へ同数ずつ配置する。
fn line_bar_coords(
    intersections: &[f64],
    count: u32,
    min_spacing: f64,
) -> Result<Vec<f64>, RebarGeometryError> {
    let n_int = intersections.len();
    let extras = (count as usize).saturating_sub(n_int);
    let mut coords = intersections.to_vec();
    if extras > 0 {
        let center = n_int / 2 - 1;
        let capacity = |k: usize| -> usize {
            let ratio = (intersections[k + 1] - intersections[k]) / min_spacing;
            if ratio < 1.0 {
                0
            } else {
                ((ratio + EPS_REL).floor() as usize).saturating_sub(1)
            }
        };
        let mut allocation = vec![0usize; n_int - 1];
        let mut remaining = extras;
        let central = remaining.min(capacity(center));
        allocation[center] = central;
        remaining -= central;
        for offset in 1..=center {
            if remaining == 0 {
                break;
            }
            let left = center - offset;
            let right = center + offset;
            let per = (remaining / 2)
                .min(capacity(left))
                .min(capacity(right));
            allocation[left] = per;
            allocation[right] = per;
            remaining -= per * 2;
        }
        if remaining != 0 {
            return Err(RebarGeometryError::SpacingOrSymmetry);
        }
        for (gap, &m) in intersections.windows(2).zip(allocation.iter()) {
            let (lo, hi) = (gap[0], gap[1]);
            for t in 1..=m {
                coords.push(lo + (hi - lo) * t as f64 / (m + 1) as f64);
            }
        }
    }
    if coords.len() != count as usize {
        return Err(RebarGeometryError::CountMismatch);
    }
    Ok(coords)
}

impl RcBeamRebar {
    /// 実鉄筋座標 [mm] を生成する。未入力（`is_unset`）なら空。
    ///
    /// 生成できない配置は [`RebarGeometryError`] を返す。
    pub fn bar_positions(&self, b: f64, d: f64) -> Result<Vec<RebarPoint>, RebarGeometryError> {
        self.place(b, d)
    }

    /// 実鉄筋座標を生成できるか検証する。未入力なら `Ok(())`。
    pub fn validate(&self, b: f64, d: f64) -> Result<(), RebarGeometryError> {
        self.place(b, d).map(|_| ())
    }

    /// 上端筋・下端筋をかぶり側から内側へ段配置し、幅方向は `distribute` で等間隔に並べる。
    fn place(&self, b: f64, d: f64) -> Result<Vec<RebarPoint>, RebarGeometryError> {
        if self.is_unset() {
            return Ok(Vec::new());
        }
        check_positive("b", b)?;
        check_positive("d", d)?;
        check_positive("main_dia", self.main_dia)?;
        check_non_negative("cover", self.cover)?;
        check_non_negative("stirrup_dia", self.stirrup.dia)?;

        for (layer, &count) in self.top.iter().enumerate() {
            if count == 0 {
                return Err(RebarGeometryError::InvalidLayer {
                    location: "上端筋",
                    layer,
                    count,
                });
            }
        }
        for (layer, &count) in self.bottom.iter().enumerate() {
            if count == 0 {
                return Err(RebarGeometryError::InvalidLayer {
                    location: "下端筋",
                    layer,
                    count,
                });
            }
        }

        let k0 = self.cover + self.stirrup.dia + self.main_dia / 2.0;
        let s = center_spacing_mm(self.main_dia);
        let half = b / 2.0 - k0;
        if half <= 0.0 {
            return Err(RebarGeometryError::OutOfBounds);
        }

        let top_inner_y =
            d / 2.0 - (k0 + self.top.len().saturating_sub(1) as f64 * s);
        let bottom_inner_y =
            -(d / 2.0 - (k0 + self.bottom.len().saturating_sub(1) as f64 * s));
        if !self.top.is_empty() && top_inner_y <= 0.0 {
            return Err(RebarGeometryError::LayerCollision);
        }
        if !self.bottom.is_empty() && bottom_inner_y >= 0.0 {
            return Err(RebarGeometryError::LayerCollision);
        }
        if !self.top.is_empty() && !self.bottom.is_empty() && top_inner_y <= bottom_inner_y {
            return Err(RebarGeometryError::LayerCollision);
        }

        let mut points = Vec::new();
        for (layer, &count) in self.top.iter().enumerate() {
            let y = d / 2.0 - (k0 + layer as f64 * s);
            for x in distribute(count, half) {
                points.push(RebarPoint { x, y });
            }
        }
        for (layer, &count) in self.bottom.iter().enumerate() {
            let y = -(d / 2.0 - (k0 + layer as f64 * s));
            for x in distribute(count, half) {
                points.push(RebarPoint { x, y });
            }
        }

        let expected = self.top.iter().map(|&c| c as usize).sum::<usize>()
            + self.bottom.iter().map(|&c| c as usize).sum::<usize>();
        if points.len() != expected {
            return Err(RebarGeometryError::CountMismatch);
        }
        ensure_min_spacing(&points, s)?;
        ensure_in_bounds(&points, b, d)?;
        Ok(points)
    }
}

impl RcRectColumnRebar {
    /// 実鉄筋座標 [mm] を生成する。未入力（`is_unset`）なら空。
    ///
    /// 生成できない配置は [`RebarGeometryError`] を返す。
    pub fn bar_positions(&self, b: f64, d: f64) -> Result<Vec<RebarPoint>, RebarGeometryError> {
        self.place(b, d)
    }

    /// 実鉄筋座標を生成できるか検証する。未入力なら `Ok(())`。
    pub fn validate(&self, b: f64, d: f64) -> Result<(), RebarGeometryError> {
        self.place(b, d).map(|_| ())
    }

    /// X 方向・Y 方向の段を中心対称な 2 本の配筋線として配置し、交点を 1 本に重複除去する。
    fn place(&self, b: f64, d: f64) -> Result<Vec<RebarPoint>, RebarGeometryError> {
        if self.is_unset() {
            return Ok(Vec::new());
        }
        let nx = self.x.len();
        let ny = self.y.len();
        if nx == 0 || ny == 0 {
            let location = if nx == 0 { "X方向" } else { "Y方向" };
            return Err(RebarGeometryError::InvalidLayer {
                location,
                layer: 0,
                count: 0,
            });
        }
        check_positive("b", b)?;
        check_positive("d", d)?;
        check_positive("main_dia", self.main_dia)?;
        check_non_negative("cover", self.cover)?;
        check_non_negative("hoop_dia", self.hoop.dia)?;

        let k0 = self.cover + self.hoop.dia + self.main_dia / 2.0;
        let s = center_spacing_mm(self.main_dia);

        let y_inner = d / 2.0 - k0 - (nx - 1) as f64 * s;
        let x_inner = b / 2.0 - k0 - (ny - 1) as f64 * s;
        if y_inner <= 0.0 || x_inner <= 0.0 {
            return Err(RebarGeometryError::LayerCollision);
        }
        if !spacing_ok(2.0 * y_inner, s) || !spacing_ok(2.0 * x_inner, s) {
            return Err(RebarGeometryError::SpacingOrSymmetry);
        }

        for (layer, &count) in self.x.iter().enumerate() {
            let required = (2 * ny) as u32;
            if count < required {
                return Err(RebarGeometryError::TooFewIntersectionBars {
                    axis: "X",
                    layer,
                    count,
                    required,
                });
            }
        }
        for (layer, &count) in self.y.iter().enumerate() {
            let required = (2 * nx) as u32;
            if count < required {
                return Err(RebarGeometryError::TooFewIntersectionBars {
                    axis: "Y",
                    layer,
                    count,
                    required,
                });
            }
        }

        let mut x_lines = Vec::with_capacity(2 * ny);
        for j in 0..ny {
            let x = b / 2.0 - k0 - j as f64 * s;
            x_lines.push(x);
            x_lines.push(-x);
        }
        x_lines.sort_by(f64::total_cmp);

        let mut y_lines = Vec::with_capacity(2 * nx);
        for i in 0..nx {
            let y = d / 2.0 - k0 - i as f64 * s;
            y_lines.push(y);
            y_lines.push(-y);
        }
        y_lines.sort_by(f64::total_cmp);

        let mut points = Vec::new();
        for (i, &count) in self.x.iter().enumerate() {
            let y = d / 2.0 - k0 - i as f64 * s;
            let coords = line_bar_coords(&x_lines, count, s)?;
            for line_y in [y, -y] {
                for &x in &coords {
                    points.push(RebarPoint { x, y: line_y });
                }
            }
        }
        for (j, &count) in self.y.iter().enumerate() {
            let x = b / 2.0 - k0 - j as f64 * s;
            let coords = line_bar_coords(&y_lines, count, s)?;
            for line_x in [x, -x] {
                for &y in &coords {
                    points.push(RebarPoint { x: line_x, y });
                }
            }
        }

        let mut unique: Vec<RebarPoint> = Vec::with_capacity(points.len());
        for p in points {
            if !unique.contains(&p) {
                unique.push(p);
            }
        }

        let expected = 2 * self.x.iter().map(|&c| c as usize).sum::<usize>()
            + 2 * self.y.iter().map(|&c| c as usize).sum::<usize>()
            - 4 * nx * ny;
        if unique.len() != expected {
            return Err(RebarGeometryError::CountMismatch);
        }
        ensure_min_spacing(&unique, s)?;
        ensure_in_bounds(&unique, b, d)?;
        Ok(unique)
    }
}

impl RcCircleColumnRebar {
    /// 実鉄筋座標 [mm] を生成する。未入力（`is_unset`）なら空。
    ///
    /// 生成できない配置は [`RebarGeometryError`] を返す。
    pub fn bar_positions(&self, d: f64) -> Result<Vec<RebarPoint>, RebarGeometryError> {
        self.place(d)
    }

    /// 実鉄筋座標を生成できるか検証する。未入力なら `Ok(())`。
    pub fn validate(&self, d: f64) -> Result<(), RebarGeometryError> {
        self.place(d).map(|_| ())
    }

    /// 円周上へ等間隔配置する。`i=0` は +x 軸、以後反時計回り。
    fn place(&self, d: f64) -> Result<Vec<RebarPoint>, RebarGeometryError> {
        if self.is_unset() {
            return Ok(Vec::new());
        }
        check_positive("d", d)?;
        check_positive("main_dia", self.main_dia)?;
        check_non_negative("cover", self.cover)?;
        check_non_negative("hoop_dia", self.hoop.dia)?;

        let r = d / 2.0 - self.cover - self.hoop.dia - self.main_dia / 2.0;
        if r <= 0.0 {
            return Err(RebarGeometryError::OutOfBounds);
        }
        let n = self.count;
        if n >= 2 {
            let chord = 2.0 * r * (std::f64::consts::PI / n as f64).sin();
            if !spacing_ok(chord, center_spacing_mm(self.main_dia)) {
                return Err(RebarGeometryError::SpacingOrSymmetry);
            }
        }

        let mut points = Vec::with_capacity(n as usize);
        for i in 0..n {
            let theta = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
            points.push(RebarPoint {
                x: r * theta.cos(),
                y: r * theta.sin(),
            });
        }
        Ok(points)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::RebarGeometryError;
    use crate::section_shape::{BeamStirrup, CircleColumnHoop, RectColumnHoop, ShearBar};

    fn rebar(layers: u32) -> RcRebar {
        let main = BarSet {
            count: 6,
            dia: 25.0,
            layers,
        };
        RcRebar {
            main_x: main.clone(),
            main_y: main,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
            },
            cover: 40.0,
        }
    }

    #[test]
    fn test_tension_dt_single_layer() {
        let bar = BarSet {
            count: 4,
            dia: 22.0,
            layers: 1,
        };
        let dt = tension_dt(40.0, 10.0, &bar);
        assert!((dt - (40.0 + 10.0 + 11.0)).abs() < 1e-9);
    }

    #[test]
    fn test_tension_dt_two_layers() {
        let bar = BarSet {
            count: 8,
            dia: 22.0,
            layers: 2,
        };
        let cover = 40.0;
        let shear_dia = 10.0;
        let k1 = cover + shear_dia + bar.dia / 2.0;
        let k_prime = 25.0_f64.max(1.5 * bar.dia);
        let k2 = k1 + bar.dia / 2.0 + k_prime + bar.dia / 2.0;
        let expected = (k1 + k2) / 2.0;
        let dt = tension_dt(cover, shear_dia, &bar);
        assert!((dt - expected).abs() < 1e-6);
    }

    #[test]
    fn test_rebar_tension_dt_considers_layers() {
        let k1 = 40.0 + 10.0 + 25.0 / 2.0;
        assert!((rebar_tension_dt(&rebar(1)) - k1).abs() < 1e-9);

        let two = rebar_tension_dt(&rebar(2));
        assert!((two - (k1 + 62.5 / 2.0)).abs() < 1e-9, "dt(2段)={two}");
        assert!(two > k1);

        let r = rebar(2);
        assert!((two - tension_dt(r.cover, r.shear.dia, &r.main_x)).abs() < 1e-12);
    }

    #[test]
    fn test_layer_depth_matches_tension_dt_average() {
        let bar = BarSet {
            count: 4,
            dia: 13.0, // 細い筋: k'=25 > 1.5φ（旧 2.5φ とは段間隔が違う）
            layers: 2,
        };
        let cover = 40.0;
        let shear = 10.0;
        let d0 = rebar_layer_depth_from_edge(cover, shear, &bar, 0);
        let d1 = rebar_layer_depth_from_edge(cover, shear, &bar, 1);
        let s = rebar_layer_spacing(&bar);
        assert!((s - (13.0 + 25.0)).abs() < 1e-12);
        assert!((d1 - d0 - s).abs() < 1e-12);
        assert!((tension_dt(cover, shear, &bar) - 0.5 * (d0 + d1)).abs() < 1e-12);
    }

    #[test]
    fn test_pw_ratio_zero_pitch() {
        let shear = ShearBar {
            dia: 10.0,
            pitch: 0.0,
            legs: 2,
        };
        assert_eq!(pw_ratio(&shear, 400.0), 0.0);
    }

    fn beam(top: Vec<u32>, bottom: Vec<u32>) -> RcBeamRebar {
        RcBeamRebar {
            main_dia: 22.0,
            top,
            bottom,
            cover: 40.0,
            stirrup: BeamStirrup {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
            },
        }
    }

    fn rect_column(x: Vec<u32>, y: Vec<u32>) -> RcRectColumnRebar {
        RcRectColumnRebar {
            main_dia: 25.0,
            x,
            y,
            cover: 50.0,
            hoop: RectColumnHoop {
                dia: 10.0,
                pitch: 100.0,
                legs_x: 3,
                legs_y: 3,
            },
        }
    }

    fn circle(count: u32) -> RcCircleColumnRebar {
        RcCircleColumnRebar {
            main_dia: 22.0,
            count,
            cover: 40.0,
            hoop: CircleColumnHoop {
                dia: 10.0,
                pitch: 100.0,
            },
        }
    }

    fn count_at_y(points: &[RebarPoint], y: f64) -> usize {
        points.iter().filter(|p| (p.y - y).abs() < 1e-9).count()
    }

    fn count_at_x(points: &[RebarPoint], x: f64) -> usize {
        points.iter().filter(|p| (p.x - x).abs() < 1e-9).count()
    }

    /// 梁: 総点数と段の y、幅方向の等間隔分布を確認する。
    #[test]
    fn test_beam_positions_layout() {
        let r = beam(vec![4, 2], vec![3, 2]);
        let pts = r.bar_positions(400.0, 600.0).unwrap();
        // k0 = 40 + 10 + 11 = 61、s = 55、half = 139。
        assert_eq!(pts.len(), 11);
        assert_eq!(count_at_y(&pts, 239.0), 4);
        assert_eq!(count_at_y(&pts, 184.0), 2);
        assert_eq!(count_at_y(&pts, -239.0), 3);
        assert_eq!(count_at_y(&pts, -184.0), 2);
        // 段中心間は s = 55。
        assert!((239.0_f64 - 184.0 - 55.0).abs() < 1e-9);

        let xs = |y: f64| -> Vec<f64> {
            let mut v: Vec<f64> = pts
                .iter()
                .filter(|p| (p.y - y).abs() < 1e-9)
                .map(|p| p.x)
                .collect();
            v.sort_by(f64::total_cmp);
            v
        };
        // n>=2 は両端 ±(b/2 - k0) を含む等間隔。
        let top0 = xs(239.0);
        assert!((top0[0] + 139.0).abs() < 1e-9);
        assert!((top0[3] - 139.0).abs() < 1e-9);
        assert!((top0[1] - top0[0] - 2.0 * 139.0 / 3.0).abs() < 1e-9);
        let top1 = xs(184.0);
        assert_eq!(top1.len(), 2);
        assert!((top1[0] + 139.0).abs() < 1e-9 && (top1[1] - 139.0).abs() < 1e-9);
        // 下端 1 段目は -139, 0, 139。
        let bottom0 = xs(-239.0);
        assert!(bottom0.iter().any(|x| x.abs() < 1e-9));
        assert_eq!(r.validate(400.0, 600.0), Ok(()));
    }

    /// 梁: n==1 の段は中心（x=0）に配置する。
    #[test]
    fn test_beam_single_bar_centered() {
        let r = beam(vec![1], vec![1]);
        let pts = r.bar_positions(400.0, 600.0).unwrap();
        assert_eq!(pts.len(), 2);
        assert!(pts.iter().all(|p| p.x.abs() < 1e-12));
        assert!(pts.iter().any(|p| (p.y - 239.0).abs() < 1e-9));
        assert!(pts.iter().any(|p| (p.y + 239.0).abs() < 1e-9));
    }

    /// 梁: 3 段以上の各段本数が入力どおり。
    #[test]
    fn test_beam_three_layers() {
        let r = beam(vec![3, 2, 1], vec![2, 2, 1]);
        let pts = r.bar_positions(400.0, 600.0).unwrap();
        assert_eq!(pts.len(), 11);
        for (y, n) in [
            (239.0, 3),
            (184.0, 2),
            (129.0, 1),
            (-239.0, 2),
            (-184.0, 2),
            (-129.0, 1),
        ] {
            assert_eq!(count_at_y(&pts, y), n, "y={y}");
        }
    }

    /// 梁: 段本数 0・寸法不足・中心越え・間隔不足を拒否する。
    #[test]
    fn test_beam_errors() {
        assert!(matches!(
            beam(vec![0], vec![]).validate(400.0, 600.0),
            Err(RebarGeometryError::InvalidLayer { .. })
        ));
        assert!(matches!(
            beam(vec![4], vec![4]).validate(100.0, 600.0),
            Err(RebarGeometryError::OutOfBounds)
        ));
        assert!(matches!(
            beam(vec![4, 2], vec![4, 2]).validate(400.0, 150.0),
            Err(RebarGeometryError::LayerCollision)
        ));
        assert!(matches!(
            beam(vec![20], vec![]).validate(400.0, 600.0),
            Err(RebarGeometryError::SpacingOrSymmetry)
        ));
    }

    /// 矩形柱: x=[3], y=[3] は四隅を共有して 8 点。
    #[test]
    fn test_rect_column_corners() {
        let r = rect_column(vec![3], vec![3]);
        let pts = r.bar_positions(500.0, 500.0).unwrap();
        assert_eq!(pts.len(), 8);
        assert_eq!(count_at_x(&pts, 177.5), 3);
        assert_eq!(count_at_y(&pts, 177.5), 3);
        assert_eq!(r.validate(500.0, 500.0), Ok(()));
    }

    /// 矩形柱: 余剰本数を中心ギャップへ配置し、各線の点数が入力と一致する。
    #[test]
    fn test_rect_column_multi() {
        let r = rect_column(vec![4, 2], vec![4]);
        let pts = r.bar_positions(500.0, 500.0).unwrap();
        assert_eq!(pts.len(), 12);
        assert_eq!(count_at_y(&pts, 177.5), 4);
        assert_eq!(count_at_y(&pts, -177.5), 4);
        assert_eq!(count_at_y(&pts, 115.0), 2);
        assert_eq!(count_at_y(&pts, -115.0), 2);
        assert_eq!(count_at_x(&pts, 177.5), 4);
        assert_eq!(count_at_x(&pts, -177.5), 4);
        assert_eq!(r.validate(500.0, 500.0), Ok(()));
    }

    /// 矩形柱: 交点本数が不足する段を拒否する。
    #[test]
    fn test_rect_column_too_few() {
        assert!(matches!(
            rect_column(vec![4, 2], vec![3]).validate(500.0, 500.0),
            Err(RebarGeometryError::TooFewIntersectionBars { axis: "Y", .. })
        ));
    }

    /// 矩形柱: 片方だけ空は InvalidLayer。
    #[test]
    fn test_rect_column_one_side_empty() {
        assert!(matches!(
            rect_column(vec![3], vec![]).validate(500.0, 500.0),
            Err(RebarGeometryError::InvalidLayer { .. })
        ));
        assert!(matches!(
            rect_column(vec![], vec![3]).validate(500.0, 500.0),
            Err(RebarGeometryError::InvalidLayer { .. })
        ));
    }

    /// 矩形柱: 寸法不足・中心越え・余剰本数が入りきらない配置を拒否する。
    #[test]
    fn test_rect_column_errors() {
        assert!(matches!(
            rect_column(vec![3], vec![3]).validate(100.0, 500.0),
            Err(RebarGeometryError::LayerCollision)
        ));
        assert!(matches!(
            rect_column(vec![3, 3, 3, 3], vec![3, 3, 3, 3]).validate(500.0, 500.0),
            Err(RebarGeometryError::LayerCollision)
        ));
        let tight = RcRectColumnRebar {
            main_dia: 22.0,
            x: vec![8],
            y: vec![2],
            cover: 40.0,
            hoop: RectColumnHoop {
                dia: 10.0,
                pitch: 100.0,
                legs_x: 3,
                legs_y: 3,
            },
        };
        assert!(matches!(
            tight.validate(300.0, 300.0),
            Err(RebarGeometryError::SpacingOrSymmetry)
        ));
    }

    /// 円形柱: 円周上の等間隔配置と i=0 の +x 軸を確認する。
    #[test]
    fn test_circle_positions() {
        let r = circle(8);
        let pts = r.bar_positions(600.0).unwrap();
        assert_eq!(pts.len(), 8);
        // r = 300 - 40 - 10 - 11 = 239。
        for p in &pts {
            assert!(((p.x * p.x + p.y * p.y).sqrt() - 239.0).abs() < 1e-9);
        }
        assert!((pts[0].x - 239.0).abs() < 1e-9);
        assert!(pts[0].y.abs() < 1e-9);
        assert_eq!(r.validate(600.0), Ok(()));
    }

    /// 円形柱: r<=0 と弦長不足を拒否する。
    #[test]
    fn test_circle_errors() {
        assert!(matches!(
            circle(8).validate(100.0),
            Err(RebarGeometryError::OutOfBounds)
        ));
        assert!(matches!(
            circle(8).validate(200.0),
            Err(RebarGeometryError::SpacingOrSymmetry)
        ));
    }

    /// 未入力の各型は validate が Ok、bar_positions が空。
    #[test]
    fn test_real_rebar_unset_positions() {
        let b = beam(vec![], vec![]);
        assert!(b.is_unset());
        assert_eq!(b.validate(400.0, 600.0), Ok(()));
        assert!(b.bar_positions(400.0, 600.0).unwrap().is_empty());

        let c = rect_column(vec![], vec![]);
        assert!(c.is_unset());
        assert_eq!(c.validate(500.0, 500.0), Ok(()));
        assert!(c.bar_positions(500.0, 500.0).unwrap().is_empty());

        let ci = circle(0);
        assert!(ci.is_unset());
        assert_eq!(ci.validate(600.0), Ok(()));
        assert!(ci.bar_positions(600.0).unwrap().is_empty());
    }
}
