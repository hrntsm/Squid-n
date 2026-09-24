//! 断面形状から全塑性計算用のファイバ/バネ配置を生成する。

use squid_n_core::error::RebarGeometryError;
use squid_n_core::section_shape::{one_bar_area, RebarPoint, SectionShape};

use super::types::{concrete_young, FiberRegion, PlasticFiber, StrengthParams, YieldModelKind};

const NOMINAL_SLAB_WIDTH_MM: f64 = 1000.0;

/// ファイバ材料（限界応力と弾性係数、材料領域区分）。
#[derive(Clone, Copy)]
pub(crate) struct FiberMat {
    pub sigma_t: f64,
    pub sigma_c: f64,
    pub young: f64,
    pub region: FiberRegion,
}

/// 円環領域の分割解像度。
#[derive(Clone, Copy, Debug)]
pub struct AnnulusRes {
    /// 周方向分割数。
    pub n_theta: usize,
    /// 薄肉円環（鋼管壁）の径方向分割数。
    pub n_r_thin: usize,
    /// 中実円（丸鋼・RC 円形・CFT 充填部）の径方向分割数。
    pub n_r_solid: usize,
}

/// 矩形領域を目標寸法 `target` 以下のファイバに等分割して追加する。
pub(crate) fn mesh_rect(
    fibers: &mut Vec<PlasticFiber>,
    center: [f64; 2],
    w: f64,
    h: f64,
    target: f64,
    mat: FiberMat,
) {
    let [cy, cz] = center;
    let FiberMat {
        sigma_t,
        sigma_c,
        young,
        region,
    } = mat;
    let ny = (w / target).ceil().max(1.0) as usize;
    let nz = (h / target).ceil().max(1.0) as usize;
    let dy = w / ny as f64;
    let dz = h / nz as f64;
    for i in 0..ny {
        for j in 0..nz {
            fibers.push(PlasticFiber {
                y: cy - w / 2.0 + (i as f64 + 0.5) * dy,
                z: cz - h / 2.0 + (j as f64 + 0.5) * dz,
                area: dy * dz,
                sigma_t,
                sigma_c,
                young,
                region,
            });
        }
    }
}

/// 円環領域を周方向・径方向に分割して追加する。
fn mesh_annulus(
    fibers: &mut Vec<PlasticFiber>,
    outer_dia: f64,
    thick: f64,
    n_theta: usize,
    n_r: usize,
    mat: FiberMat,
) {
    let FiberMat {
        sigma_t,
        sigma_c,
        young,
        region,
    } = mat;
    let ro = outer_dia / 2.0;
    let ri = (ro - thick).max(0.0);
    let dr = (ro - ri) / n_r as f64;
    for ir in 0..n_r {
        let r_mid = ri + (ir as f64 + 0.5) * dr;
        let r_in = ri + ir as f64 * dr;
        let r_out = r_in + dr;
        let ring_area = std::f64::consts::PI * (r_out * r_out - r_in * r_in);
        let a = ring_area / n_theta as f64;
        for it in 0..n_theta {
            let th = 2.0 * std::f64::consts::PI * (it as f64 + 0.5) / n_theta as f64;
            fibers.push(PlasticFiber {
                y: r_mid * th.cos(),
                z: r_mid * th.sin(),
                area: a,
                sigma_t,
                sigma_c,
                young,
                region,
            });
        }
    }
}

/// H 形を板ごとにメッシュ化して追加する。
fn mesh_h_plates(
    fibers: &mut Vec<PlasticFiber>,
    height: f64,
    width: f64,
    web_thick: f64,
    flange_thick: f64,
    target: f64,
    mat: FiberMat,
) {
    let hw = height - 2.0 * flange_thick;
    mesh_rect(
        fibers,
        [0.0, (height - flange_thick) / 2.0],
        width,
        flange_thick,
        target,
        mat,
    );
    mesh_rect(
        fibers,
        [0.0, -(height - flange_thick) / 2.0],
        width,
        flange_thick,
        target,
        mat,
    );
    mesh_rect(fibers, [0.0, 0.0], web_thick, hw, target, mat);
}

/// 箱形の 4 枚板をメッシュ化して追加する。
fn mesh_box_plates(
    fibers: &mut Vec<PlasticFiber>,
    height: f64,
    width: f64,
    thick: f64,
    target: f64,
    mat: FiberMat,
) {
    let hw = height - 2.0 * thick;
    mesh_rect(
        fibers,
        [0.0, (height - thick) / 2.0],
        width,
        thick,
        target,
        mat,
    );
    mesh_rect(
        fibers,
        [0.0, -(height - thick) / 2.0],
        width,
        thick,
        target,
        mat,
    );
    for ysign in [1.0, -1.0] {
        mesh_rect(
            fibers,
            [ysign * (width - thick) / 2.0, 0.0],
            thick,
            hw,
            target,
            mat,
        );
    }
}

/// 実配筋座標 `RebarPoint{x,y}` をファイバ `PlasticFiber{y,z}` へ写して追加する
/// （`fiber.y = point.x`, `fiber.z = point.y`）。
fn rebar_fibers_from_points(
    fibers: &mut Vec<PlasticFiber>,
    points: &[RebarPoint],
    main_dia: f64,
    fy: f64,
    young: f64,
) {
    let a = one_bar_area(main_dia);
    for p in points {
        fibers.push(PlasticFiber {
            y: p.x,
            z: p.y,
            area: a,
            sigma_t: fy,
            sigma_c: -fy,
            young,
            region: FiberRegion::Rebar,
        });
    }
}

/// 断面形状からファイバ/バネ配置を生成する。
/// `kind` により解像度が変わる（細分割と粗い配置）。
/// 実配筋を生成できない場合は [`RebarGeometryError`] を返す。
pub fn plastic_fibers(
    shape: &SectionShape,
    strength: &StrengthParams,
    kind: YieldModelKind,
) -> Result<Vec<PlasticFiber>, RebarGeometryError> {
    let fine = !matches!(kind, YieldModelKind::MultiSpring);
    let target = if fine {
        max_dimension(shape) / 40.0
    } else {
        max_dimension(shape) / 4.0
    };
    let ring = if fine {
        AnnulusRes {
            n_theta: 48,
            n_r_thin: 4,
            n_r_solid: 12,
        }
    } else {
        AnnulusRes {
            n_theta: 8,
            n_r_thin: 1,
            n_r_solid: 2,
        }
    };
    plastic_fibers_at(shape, strength, target, ring)
}

/// 断面外形の最大寸法 [mm]（目標ファイバ寸法の基準）。
pub fn max_dimension(shape: &SectionShape) -> f64 {
    match *shape {
        SectionShape::SteelH { height, width, .. }
        | SectionShape::SteelBox { height, width, .. }
        | SectionShape::SteelChannel { height, width, .. }
        | SectionShape::SteelTee { height, width, .. } => height.max(width),
        SectionShape::SteelAngle { leg_a, leg_b, .. } => leg_a.max(leg_b),
        SectionShape::SteelPipe { outer_dia, .. } => outer_dia,
        SectionShape::SteelFlatBar { width, thick } => width.max(thick),
        SectionShape::SteelRoundBar { dia } => dia,
        SectionShape::SteelLipChannel { height, width, .. } => height.max(width),
        SectionShape::SteelBuiltH {
            height,
            upper_width,
            lower_width,
            ..
        } => height.max(upper_width).max(lower_width),
        SectionShape::RcBeamRect { b, d, .. }
        | SectionShape::RcColumnRect { b, d, .. }
        | SectionShape::SrcBeamRect { b, d, .. }
        | SectionShape::SrcColumnRect { b, d, .. } => b.max(d),
        SectionShape::RcColumnCircle { d, .. } => d,
        SectionShape::CftBox { height, width, .. } => height.max(width),
        SectionShape::CftPipe { outer_dia, .. } => outer_dia,
        SectionShape::RcWall { thickness, .. } | SectionShape::RcSlab { thickness } => {
            thickness.max(1000.0)
        }
    }
}

/// 目標ファイバ寸法 `target` [mm] と円環解像度 `ring` を明示して配置を生成する。
/// [`plastic_fibers`]（MN 曲面・M-φ 用）と要素ファイバ生成
/// （`squid-n-element` の `build_gauss_fibers`）が同じ配置規則を共用するための実体。
/// 実配筋を生成できない場合は [`RebarGeometryError`] を返す。
pub fn plastic_fibers_at(
    shape: &SectionShape,
    strength: &StrengthParams,
    target: f64,
    ring: AnnulusRes,
) -> Result<Vec<PlasticFiber>, RebarGeometryError> {
    let fy = strength.steel_fy;
    let fc = strength.concrete_fc;
    let steel = FiberMat {
        sigma_t: fy,
        sigma_c: -fy,
        young: strength.steel_e,
        region: FiberRegion::Steel,
    };
    let conc = FiberMat {
        sigma_t: 0.0,
        sigma_c: -fc,
        young: concrete_young(fc),
        region: FiberRegion::Concrete,
    };
    let mut fibers = Vec::new();

    match *shape {
        SectionShape::SteelH {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            mesh_h_plates(
                &mut fibers,
                height,
                width,
                web_thick,
                flange_thick,
                target,
                steel,
            );
        }
        SectionShape::SteelBox {
            height,
            width,
            thick,
            ..
        } => {
            mesh_box_plates(&mut fibers, height, width, thick, target, steel);
        }
        SectionShape::SteelAngle {
            leg_a,
            leg_b,
            thick,
        } => {
            mesh_rect(
                &mut fibers,
                [thick / 2.0, leg_a / 2.0],
                thick,
                leg_a,
                target,
                steel,
            );
            mesh_rect(
                &mut fibers,
                [thick + (leg_b - thick) / 2.0, thick / 2.0],
                leg_b - thick,
                thick,
                target,
                steel,
            );
        }
        SectionShape::SteelChannel {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let hw = height - 2.0 * flange_thick;
            mesh_rect(
                &mut fibers,
                [web_thick / 2.0, 0.0],
                web_thick,
                hw,
                target,
                steel,
            );
            for zsign in [1.0, -1.0] {
                mesh_rect(
                    &mut fibers,
                    [width / 2.0, zsign * (height - flange_thick) / 2.0],
                    width,
                    flange_thick,
                    target,
                    steel,
                );
            }
        }
        SectionShape::SteelTee {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let hw = height - flange_thick;
            mesh_rect(
                &mut fibers,
                [0.0, (height - flange_thick) / 2.0],
                width,
                flange_thick,
                target,
                steel,
            );
            mesh_rect(
                &mut fibers,
                [
                    0.0,
                    (height - flange_thick) / 2.0 - flange_thick / 2.0 - hw / 2.0,
                ],
                web_thick,
                hw,
                target,
                steel,
            );
        }
        SectionShape::SteelPipe { outer_dia, thick } => {
            mesh_annulus(
                &mut fibers,
                outer_dia,
                thick,
                ring.n_theta,
                ring.n_r_thin,
                steel,
            );
        }
        SectionShape::SteelFlatBar { width, thick } => {
            mesh_rect(&mut fibers, [0.0, 0.0], width, thick, target, steel);
        }
        SectionShape::SteelRoundBar { dia } => {
            mesh_annulus(
                &mut fibers,
                dia,
                dia / 2.0,
                ring.n_theta,
                ring.n_r_solid,
                steel,
            );
        }
        SectionShape::SteelLipChannel {
            height,
            width,
            lip,
            thick,
        } => {
            let t = thick;
            mesh_rect(
                &mut fibers,
                [t / 2.0, height / 2.0],
                t,
                height,
                target,
                steel,
            );
            for ysign in [1.0, -1.0] {
                mesh_rect(
                    &mut fibers,
                    [(t + width) / 2.0, height / 2.0 + ysign * (height - t) / 2.0],
                    width - t,
                    t,
                    target,
                    steel,
                );
                mesh_rect(
                    &mut fibers,
                    [
                        width - t / 2.0,
                        height / 2.0 + ysign * (height - lip - t) / 2.0,
                    ],
                    t,
                    lip - t,
                    target,
                    steel,
                );
            }
        }
        SectionShape::SteelBuiltH {
            height,
            upper_width,
            upper_thick,
            lower_width,
            lower_thick,
            web_thick,
        } => {
            let hw = (height - upper_thick - lower_thick).max(0.0);
            mesh_rect(
                &mut fibers,
                [0.0, height - upper_thick / 2.0],
                upper_width,
                upper_thick,
                target,
                steel,
            );
            mesh_rect(
                &mut fibers,
                [0.0, lower_thick / 2.0],
                lower_width,
                lower_thick,
                target,
                steel,
            );
            mesh_rect(
                &mut fibers,
                [0.0, lower_thick + hw / 2.0],
                web_thick,
                hw,
                target,
                steel,
            );
        }
        SectionShape::RcBeamRect { b, d, ref rebar } => {
            mesh_rect(&mut fibers, [0.0, 0.0], b, d, target, conc);
            let positions = rebar.bar_positions(b, d)?;
            rebar_fibers_from_points(
                &mut fibers,
                &positions,
                rebar.main_dia,
                strength.rebar_fy,
                strength.steel_e,
            );
        }
        SectionShape::RcColumnRect { b, d, ref rebar } => {
            mesh_rect(&mut fibers, [0.0, 0.0], b, d, target, conc);
            let positions = rebar.bar_positions(b, d)?;
            rebar_fibers_from_points(
                &mut fibers,
                &positions,
                rebar.main_dia,
                strength.rebar_fy,
                strength.steel_e,
            );
        }
        SectionShape::RcColumnCircle { d, ref rebar } => {
            mesh_annulus(&mut fibers, d, d / 2.0, ring.n_theta, ring.n_r_solid, conc);
            let positions = rebar.bar_positions(d)?;
            rebar_fibers_from_points(
                &mut fibers,
                &positions,
                rebar.main_dia,
                strength.rebar_fy,
                strength.steel_e,
            );
        }
        SectionShape::SrcBeamRect {
            b,
            d,
            ref rebar,
            steel_height,
            steel_width,
            steel_web_thick,
            steel_flange_thick,
        } => {
            mesh_rect(&mut fibers, [0.0, 0.0], b, d, target, conc);
            let positions = rebar.bar_positions(b, d)?;
            rebar_fibers_from_points(
                &mut fibers,
                &positions,
                rebar.main_dia,
                strength.rebar_fy,
                strength.steel_e,
            );
            mesh_h_plates(
                &mut fibers,
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
                target,
                steel,
            );
        }
        SectionShape::SrcColumnRect {
            b,
            d,
            ref rebar,
            steel_height,
            steel_width,
            steel_web_thick,
            steel_flange_thick,
        } => {
            mesh_rect(&mut fibers, [0.0, 0.0], b, d, target, conc);
            let positions = rebar.bar_positions(b, d)?;
            rebar_fibers_from_points(
                &mut fibers,
                &positions,
                rebar.main_dia,
                strength.rebar_fy,
                strength.steel_e,
            );
            mesh_h_plates(
                &mut fibers,
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
                target,
                steel,
            );
        }
        SectionShape::CftBox {
            height,
            width,
            thick,
        } => {
            mesh_box_plates(&mut fibers, height, width, thick, target, steel);
            mesh_rect(
                &mut fibers,
                [0.0, 0.0],
                width - 2.0 * thick,
                height - 2.0 * thick,
                target,
                conc,
            );
        }
        SectionShape::CftPipe { outer_dia, thick } => {
            mesh_annulus(
                &mut fibers,
                outer_dia,
                thick,
                ring.n_theta,
                ring.n_r_thin,
                steel,
            );
            let di = outer_dia - 2.0 * thick;
            if di > 0.0 {
                mesh_annulus(
                    &mut fibers,
                    di,
                    di / 2.0,
                    ring.n_theta,
                    ring.n_r_solid,
                    conc,
                );
            }
        }
        SectionShape::RcWall { thickness, .. } | SectionShape::RcSlab { thickness } => {
            mesh_rect(
                &mut fibers,
                [0.0, 0.0],
                NOMINAL_SLAB_WIDTH_MM,
                thickness,
                target,
                conc,
            );
        }
    }

    if matches!(
        shape,
        SectionShape::SteelAngle { .. }
            | SectionShape::SteelChannel { .. }
            | SectionShape::SteelTee { .. }
            | SectionShape::SteelLipChannel { .. }
            | SectionShape::SteelBuiltH { .. }
    ) {
        let a_sum: f64 = fibers.iter().map(|f| f.area).sum();
        if a_sum > 0.0 {
            let cy: f64 = fibers.iter().map(|f| f.area * f.y).sum::<f64>() / a_sum;
            let cz: f64 = fibers.iter().map(|f| f.area * f.z).sum::<f64>() / a_sum;
            for f in &mut fibers {
                f.y -= cy;
                f.z -= cz;
            }
        }
    }

    Ok(fibers)
}
