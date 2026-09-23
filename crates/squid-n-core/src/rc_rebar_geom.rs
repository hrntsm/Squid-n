//! RC 配筋幾何の共通算定。

use crate::error::RebarGeometryError;
use crate::section_shape::{
    one_bar_area, BarSet, RcBeamRebar, RcCircleColumnRebar, RcRebar, RcRectColumnRebar, RebarPoint,
    ShearBar,
};

/// RC 矩形断面の辺。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RectEdge {
    Top,
    Bottom,
    Left,
    Right,
}

/// 一面の主筋諸元（本数・面積・縁からの重心距離・有効せい）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SteelSide {
    pub count: usize,
    /// 主筋面積 [mm²]。
    pub area_mm2: f64,
    /// その面側の縁から重心までの距離 dt [mm]。
    pub centroid_from_edge_mm: f64,
    /// 対向縁から重心までの距離（有効せい）[mm]。
    pub effective_depth_mm: f64,
}

/// 梁の曲げ検討用。指定した引張側とその反対側（圧縮側）の諸元。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BeamBendingSteel {
    pub tension: SteelSide,
    pub compression: SteelSide,
}

/// 隣接主筋のあき k' [mm]（`max(25, 1.5・dia)`）。
fn clear_mm(dia: f64) -> f64 {
    25.0_f64.max(1.5 * dia)
}

/// 隣接主筋中心間の最小距離 s = dia + k' [mm]。
fn center_spacing_mm(dia: f64) -> f64 {
    dia + clear_mm(dia)
}

/// 段別本数の面積 [mm²]。未入力なら空。
fn layer_areas(counts: &[u32], main_dia: f64) -> Vec<f64> {
    let area = one_bar_area(main_dia);
    counts.iter().map(|&c| c as f64 * area).collect()
}

/// 段別本数を重みとした縁からの重心距離 [mm]。主筋がなければ 0。
fn centroid_from_edge(counts: &[u32], main_dia: f64, cover: f64, stirrup_dia: f64) -> f64 {
    let total: u32 = counts.iter().sum();
    if total == 0 {
        return 0.0;
    }
    let k0 = cover + stirrup_dia + main_dia / 2.0;
    let s = center_spacing_mm(main_dia);
    let weighted: f64 = counts
        .iter()
        .enumerate()
        .map(|(i, &c)| c as f64 * (k0 + i as f64 * s))
        .sum();
    weighted / total as f64
}

/// 段別本数から一面の主筋諸元を作る。
fn beam_side(counts: &[u32], main_dia: f64, cover: f64, stirrup_dia: f64, d: f64) -> SteelSide {
    let count: usize = counts.iter().map(|&c| c as usize).sum();
    let centroid_from_edge_mm = centroid_from_edge(counts, main_dia, cover, stirrup_dia);
    SteelSide {
        count,
        area_mm2: count as f64 * one_bar_area(main_dia),
        centroid_from_edge_mm,
        effective_depth_mm: d - centroid_from_edge_mm,
    }
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

impl RcBeamRebar {
    /// 上端筋の段別面積 [mm²]。未入力なら空。
    pub fn layer_areas_top(&self) -> Vec<f64> {
        layer_areas(&self.top, self.main_dia)
    }

    /// 下端筋の段別面積 [mm²]。未入力なら空。
    pub fn layer_areas_bottom(&self) -> Vec<f64> {
        layer_areas(&self.bottom, self.main_dia)
    }

    /// 上端筋の総面積 [mm²]。
    pub fn top_area(&self) -> f64 {
        self.layer_areas_top().iter().sum()
    }

    /// 下端筋の総面積 [mm²]。
    pub fn bottom_area(&self) -> f64 {
        self.layer_areas_bottom().iter().sum()
    }

    /// 上端筋の縁からの重心距離 dt [mm]。主筋がなければ 0。
    pub fn top_centroid_from_edge(&self) -> f64 {
        centroid_from_edge(&self.top, self.main_dia, self.cover, self.stirrup.dia)
    }

    /// 下端筋の縁からの重心距離 dt [mm]。主筋がなければ 0。
    pub fn bottom_centroid_from_edge(&self) -> f64 {
        centroid_from_edge(&self.bottom, self.main_dia, self.cover, self.stirrup.dia)
    }

    /// 上端筋側の有効せい d − dt [mm]。
    pub fn top_effective_depth(&self, d: f64) -> f64 {
        d - self.top_centroid_from_edge()
    }

    /// 下端筋側の有効せい d − dt [mm]。
    pub fn bottom_effective_depth(&self, d: f64) -> f64 {
        d - self.bottom_centroid_from_edge()
    }

    /// 上下主筋の総面積 [mm²]。
    pub fn total_main_area(&self) -> f64 {
        let count: usize = self
            .top
            .iter()
            .chain(self.bottom.iter())
            .map(|&c| c as usize)
            .sum();
        count as f64 * one_bar_area(self.main_dia)
    }

    /// 指定した引張側の曲げ検討用諸元。`tension_is_top=true` は上端側を引張とする。
    pub fn bending_steel(&self, d: f64, tension_is_top: bool) -> BeamBendingSteel {
        let top = beam_side(&self.top, self.main_dia, self.cover, self.stirrup.dia, d);
        let bottom = beam_side(&self.bottom, self.main_dia, self.cover, self.stirrup.dia, d);
        if tension_is_top {
            BeamBendingSteel {
                tension: top,
                compression: bottom,
            }
        } else {
            BeamBendingSteel {
                tension: bottom,
                compression: top,
            }
        }
    }

    /// あばら筋 1 組（`legs` 本）の断面積 Aw [mm²]。
    pub fn aw_mm2(&self) -> f64 {
        self.stirrup.legs as f64 * one_bar_area(self.stirrup.dia)
    }

    /// あばら筋比 pw。`pitch<=0` または `b<=0` のときは 0。
    pub fn pw(&self, b: f64) -> f64 {
        if self.stirrup.pitch <= 0.0 || b <= 0.0 {
            return 0.0;
        }
        self.aw_mm2() / (b * self.stirrup.pitch)
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

impl RcRectColumnRebar {
    /// 重複のない実鉄筋本数を主筋総面積へ換算した値 [mm²]。未入力なら 0。
    pub fn total_main_area(&self) -> f64 {
        let nx = self.x.len();
        let ny = self.y.len();
        let sx: usize = self.x.iter().map(|&c| c as usize).sum();
        let sy: usize = self.y.iter().map(|&c| c as usize).sum();
        let unique = (2 * sx + 2 * sy).saturating_sub(4 * nx * ny);
        unique as f64 * one_bar_area(self.main_dia)
    }

    /// X 方向段に載る主筋の総面積 [mm²]（`2・Σx・A1`）。未入力なら 0。
    pub fn x_direction_area_mm2(&self) -> f64 {
        2.0 * self.x.iter().map(|&c| c as f64).sum::<f64>() * one_bar_area(self.main_dia)
    }

    /// Y 方向段に載る主筋の総面積 [mm²]（`2・Σy・A1`）。未入力なら 0。
    pub fn y_direction_area_mm2(&self) -> f64 {
        2.0 * self.y.iter().map(|&c| c as f64).sum::<f64>() * one_bar_area(self.main_dia)
    }

    /// X 方向の帯筋 1 組（`legs_x` 本）の断面積 Aw [mm²]。
    pub fn aw_x_mm2(&self) -> f64 {
        self.hoop.legs_x as f64 * one_bar_area(self.hoop.dia)
    }

    /// Y 方向の帯筋 1 組（`legs_y` 本）の断面積 Aw [mm²]。
    pub fn aw_y_mm2(&self) -> f64 {
        self.hoop.legs_y as f64 * one_bar_area(self.hoop.dia)
    }

    /// 指定した辺の最外段 1 列の主筋諸元。未入力（`is_unset`）なら全項目 0。
    ///
    /// `Top`/`Bottom` は `x` の最外段を `d` で、`Left`/`Right` は `y` の最外段を `b` で評価する。
    /// 重心距離はかぶり + 帯筋径 + 主筋径/2 [mm]、有効せいは辺長からその距離を引いた値 [mm]。
    pub fn edge_steel(&self, edge: RectEdge, b: f64, d: f64) -> SteelSide {
        if self.is_unset() {
            return SteelSide {
                count: 0,
                area_mm2: 0.0,
                centroid_from_edge_mm: 0.0,
                effective_depth_mm: 0.0,
            };
        }
        let (counts, depth) = match edge {
            RectEdge::Top | RectEdge::Bottom => (&self.x, d),
            RectEdge::Left | RectEdge::Right => (&self.y, b),
        };
        let count = counts.first().copied().unwrap_or(0) as usize;
        let k0 = self.cover + self.hoop.dia + self.main_dia / 2.0;
        SteelSide {
            count,
            area_mm2: count as f64 * one_bar_area(self.main_dia),
            centroid_from_edge_mm: k0,
            effective_depth_mm: depth - k0,
        }
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

impl RcCircleColumnRebar {
    /// 主筋総本数 [本]。
    pub fn main_count(&self) -> u32 {
        self.count
    }

    /// 主筋総面積 [mm²]。
    pub fn total_main_area(&self) -> f64 {
        self.count as f64 * one_bar_area(self.main_dia)
    }

    /// 等価正方形断面の辺長 [mm]（`sqrt(π・d²/4)`）。`d<=0` は 0。
    pub fn equivalent_square_side_mm(&self, d: f64) -> f64 {
        if d <= 0.0 {
            return 0.0;
        }
        (std::f64::consts::PI * d * d / 4.0).sqrt()
    }

    /// 引張側 1 辺の鉄筋量 [mm²]（全主筋量の 1/4）。未入力なら 0。
    pub fn equivalent_tension_area_mm2(&self) -> f64 {
        if self.is_unset() {
            return 0.0;
        }
        self.total_main_area() / 4.0
    }

    /// 等価正方形断面の有効せい [mm]（辺長 − かぶり − 帯筋径 − 主筋径/2）。主筋なし・0 未満は 0。
    pub fn equivalent_effective_depth_mm(&self, d: f64) -> f64 {
        if self.is_unset() {
            return 0.0;
        }
        let k0 = self.cover + self.hoop.dia + self.main_dia / 2.0;
        (self.equivalent_square_side_mm(d) - k0).max(0.0)
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

    /// 梁: 段別面積・総面積・重心・有効せい・Aw・pw。
    #[test]
    fn test_beam_section_properties() {
        let r = beam(vec![4, 2], vec![3, 2]);
        let a22 = one_bar_area(22.0);
        let a10 = one_bar_area(10.0);
        // k0 = 40 + 10 + 11 = 61、s = 22 + 33 = 55。
        assert_eq!(r.layer_areas_top(), vec![4.0 * a22, 2.0 * a22]);
        assert_eq!(r.layer_areas_bottom(), vec![3.0 * a22, 2.0 * a22]);
        assert!((r.top_area() - 6.0 * a22).abs() < 1e-9);
        assert!((r.bottom_area() - 5.0 * a22).abs() < 1e-9);
        assert!((r.total_main_area() - 11.0 * a22).abs() < 1e-9);

        let top_centroid = 61.0 + (2.0 / 6.0) * 55.0;
        let bottom_centroid = 61.0 + (2.0 / 5.0) * 55.0;
        assert!((r.top_centroid_from_edge() - top_centroid).abs() < 1e-9);
        assert!((r.bottom_centroid_from_edge() - bottom_centroid).abs() < 1e-9);
        assert!((r.top_effective_depth(600.0) - (600.0 - top_centroid)).abs() < 1e-9);
        assert!((r.bottom_effective_depth(600.0) - (600.0 - bottom_centroid)).abs() < 1e-9);

        assert!((r.aw_mm2() - 2.0 * a10).abs() < 1e-12);
        assert!((r.pw(400.0) - 2.0 * a10 / (400.0 * 100.0)).abs() < 1e-15);
        assert_eq!(r.pw(0.0), 0.0);
    }

    /// 梁: 引張側を明示した tension / compression 諸元。
    #[test]
    fn test_beam_bending_steel() {
        let r = beam(vec![4, 2], vec![3, 2]);

        let top = r.bending_steel(600.0, true);
        assert_eq!(top.tension.count, 6);
        assert_eq!(top.compression.count, 5);
        assert!((top.tension.area_mm2 - r.top_area()).abs() < 1e-9);
        assert!((top.compression.area_mm2 - r.bottom_area()).abs() < 1e-9);
        assert!((top.tension.effective_depth_mm - r.top_effective_depth(600.0)).abs() < 1e-9);
        assert!(
            (top.compression.effective_depth_mm - r.bottom_effective_depth(600.0)).abs() < 1e-9
        );

        let bottom = r.bending_steel(600.0, false);
        assert_eq!(bottom.tension.count, 5);
        assert_eq!(bottom.compression.count, 6);
        assert!((bottom.tension.area_mm2 - r.bottom_area()).abs() < 1e-9);
        assert!((bottom.compression.area_mm2 - r.top_area()).abs() < 1e-9);
    }

    /// 矩形柱: 交点を二重計上しない総面積と、X/Y 脚数別の Aw。
    #[test]
    fn test_rect_column_section_properties() {
        let hoop = || RectColumnHoop {
            dia: 10.0,
            pitch: 100.0,
            legs_x: 2,
            legs_y: 3,
        };
        let r = RcRectColumnRebar {
            main_dia: 22.0,
            x: vec![3],
            y: vec![3],
            cover: 40.0,
            hoop: hoop(),
        };
        let a22 = one_bar_area(22.0);
        let a10 = one_bar_area(10.0);
        // 2*3 + 2*3 - 4*1*1 = 8。
        assert!((r.total_main_area() - 8.0 * a22).abs() < 1e-9);
        assert!((r.aw_x_mm2() - 2.0 * a10).abs() < 1e-12);
        assert!((r.aw_y_mm2() - 3.0 * a10).abs() < 1e-12);

        let multi = RcRectColumnRebar {
            main_dia: 22.0,
            x: vec![4, 2],
            y: vec![4],
            cover: 40.0,
            hoop: hoop(),
        };
        // 2*(4+2) + 2*4 - 4*2*1 = 12。
        assert!((multi.total_main_area() - 12.0 * a22).abs() < 1e-9);
    }

    /// 矩形柱: 方向別主筋総面積は交点を控除しない `2・Σ` の値。
    #[test]
    fn test_rect_column_direction_areas() {
        let r = RcRectColumnRebar {
            main_dia: 22.0,
            x: vec![4, 2],
            y: vec![3],
            cover: 40.0,
            hoop: RectColumnHoop {
                dia: 10.0,
                pitch: 100.0,
                legs_x: 2,
                legs_y: 3,
            },
        };
        let a1 = one_bar_area(22.0);
        assert!((r.x_direction_area_mm2() - 12.0 * a1).abs() < 1e-9);
        assert!((r.y_direction_area_mm2() - 6.0 * a1).abs() < 1e-9);
    }

    /// 矩形柱: 未入力の方向別主筋総面積は 0。
    #[test]
    fn test_rect_column_direction_areas_unset() {
        let r = rect_column(vec![], vec![]);
        assert_eq!(r.x_direction_area_mm2(), 0.0);
        assert_eq!(r.y_direction_area_mm2(), 0.0);
    }

    /// 円形柱: 総本数と総面積。
    #[test]
    fn test_circle_section_properties() {
        let r = circle(8);
        assert_eq!(r.main_count(), 8);
        assert!((r.total_main_area() - 8.0 * one_bar_area(22.0)).abs() < 1e-9);
    }

    /// 矩形柱: 辺別の最外段 1 列のみを引張側鉄筋量として返す。
    #[test]
    fn test_rect_column_edge_steel() {
        let r = RcRectColumnRebar {
            main_dia: 22.0,
            x: vec![4, 2],
            y: vec![3],
            cover: 40.0,
            hoop: RectColumnHoop {
                dia: 10.0,
                pitch: 100.0,
                legs_x: 2,
                legs_y: 3,
            },
        };
        let a1 = one_bar_area(22.0);
        // k0 = 40 + 10 + 11 = 61。
        let top = r.edge_steel(RectEdge::Top, 600.0, 700.0);
        assert_eq!(top.count, 4);
        assert!((top.area_mm2 - 4.0 * a1).abs() < 1e-9);
        assert!((top.centroid_from_edge_mm - 61.0).abs() < 1e-9);
        assert!((top.effective_depth_mm - 639.0).abs() < 1e-9);

        let left = r.edge_steel(RectEdge::Left, 600.0, 700.0);
        assert_eq!(left.count, 3);
        assert!((left.area_mm2 - 3.0 * a1).abs() < 1e-9);
        assert!((left.centroid_from_edge_mm - 61.0).abs() < 1e-9);
        assert!((left.effective_depth_mm - 539.0).abs() < 1e-9);

        // 中間段 x[1]=2 は最外段の引張側鉄筋量に算入しない。
        assert_eq!(top.count, r.x[0] as usize);
        assert_ne!(top.count, (r.x[0] + r.x[1]) as usize);
    }

    /// 矩形柱: 未入力の辺別諸元はすべて 0。
    #[test]
    fn test_rect_column_edge_steel_unset() {
        let r = rect_column(vec![], vec![]);
        let side = r.edge_steel(RectEdge::Bottom, 600.0, 700.0);
        assert_eq!(side.count, 0);
        assert_eq!(side.area_mm2, 0.0);
        assert_eq!(side.centroid_from_edge_mm, 0.0);
        assert_eq!(side.effective_depth_mm, 0.0);
    }

    /// 円形柱: 等価正方形断面の辺長・引張側鉄筋量・有効せい。
    #[test]
    fn test_circle_equivalent_square() {
        let r = circle(8);
        let a1 = one_bar_area(22.0);
        assert!((r.total_main_area() - 8.0 * a1).abs() < 1e-9);
        assert!((r.equivalent_tension_area_mm2() - 2.0 * a1).abs() < 1e-9);

        let side = (std::f64::consts::PI * 600.0 * 600.0 / 4.0).sqrt();
        assert!((r.equivalent_square_side_mm(600.0) - side).abs() < 1e-9);
        assert!(
            (r.equivalent_square_side_mm(600.0) - 600.0 * std::f64::consts::PI.sqrt() / 2.0).abs()
                < 1e-9
        );
        assert!((r.equivalent_effective_depth_mm(600.0) - (side - 61.0)).abs() < 1e-9);
        assert_eq!(r.equivalent_square_side_mm(0.0), 0.0);
    }

    /// 円形柱: 未入力の等価正方形諸元は 0。
    #[test]
    fn test_circle_equivalent_square_unset() {
        let r = circle(0);
        assert_eq!(r.equivalent_tension_area_mm2(), 0.0);
        assert_eq!(r.equivalent_effective_depth_mm(600.0), 0.0);
    }

    /// 未入力の各型: 諸元 API が 0 または空を返す。
    #[test]
    fn test_real_rebar_unset_section_properties() {
        let b = beam(vec![], vec![]);
        assert!(b.layer_areas_top().is_empty());
        assert!(b.layer_areas_bottom().is_empty());
        assert_eq!(b.top_area(), 0.0);
        assert_eq!(b.bottom_area(), 0.0);
        assert_eq!(b.top_centroid_from_edge(), 0.0);
        assert_eq!(b.bottom_centroid_from_edge(), 0.0);
        assert_eq!(b.total_main_area(), 0.0);
        assert_eq!(b.bending_steel(600.0, true).tension.count, 0);
        assert_eq!(b.bending_steel(600.0, false).compression.count, 0);

        let c = rect_column(vec![], vec![]);
        assert_eq!(c.total_main_area(), 0.0);

        let ci = circle(0);
        assert_eq!(ci.main_count(), 0);
        assert_eq!(ci.total_main_area(), 0.0);
    }
}
