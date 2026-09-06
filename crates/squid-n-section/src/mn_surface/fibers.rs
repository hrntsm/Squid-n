//! 断面形状から全塑性計算用のファイバ/バネ配置を生成する。

use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape};

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

/// 主筋1セット分のバネを追加する。段位置は `rc_rebar_geom` と同一規約。
fn rebar_fibers_rect(
    fibers: &mut Vec<PlasticFiber>,
    rebar: &RcRebar,
    b: f64,
    d: f64,
    fy: f64,
    young: f64,
) {
    use squid_n_core::rc_rebar_geom::rebar_layer_depth_from_edge;
    use squid_n_core::section_shape::one_bar_area;

    let bar = |set: &BarSet| -> f64 { one_bar_area(set.dia) };

    let set = &rebar.main_x;
    if set.count > 0 {
        let a = bar(set);
        for layer in 0..set.layers.max(1) {
            let depth = rebar_layer_depth_from_edge(rebar.cover, rebar.shear.dia, set, layer);
            let z0 = d / 2.0 - depth;
            let span = b - 2.0 * rebar.cover;
            for i in 0..set.count {
                let y = if set.count == 1 {
                    0.0
                } else {
                    -span / 2.0 + span * i as f64 / (set.count - 1) as f64
                };
                for zsign in [1.0, -1.0] {
                    fibers.push(PlasticFiber {
                        y,
                        z: zsign * z0,
                        area: a,
                        sigma_t: fy,
                        sigma_c: -fy,
                        young,
                        region: FiberRegion::Rebar,
                    });
                }
            }
        }
    }

    let set = &rebar.main_y;
    if set.count > 0 {
        let a = bar(set);
        for layer in 0..set.layers.max(1) {
            let depth = rebar_layer_depth_from_edge(rebar.cover, rebar.shear.dia, set, layer);
            let y0 = b / 2.0 - depth;
            let span = d - 2.0 * rebar.cover;
            for i in 0..set.count {
                let z = -span / 2.0 + span * (i as f64 + 1.0) / (set.count + 1) as f64;
                for ysign in [1.0, -1.0] {
                    fibers.push(PlasticFiber {
                        y: ysign * y0,
                        z,
                        area: a,
                        sigma_t: fy,
                        sigma_c: -fy,
                        young,
                        region: FiberRegion::Rebar,
                    });
                }
            }
        }
    }
}

/// RC 円形断面の主筋バネ（合計本数を円周上へ等配）。
fn rebar_fibers_circle(
    fibers: &mut Vec<PlasticFiber>,
    rebar: &RcRebar,
    d: f64,
    fy: f64,
    young: f64,
) {
    let total = (rebar.main_x.count + rebar.main_y.count) as usize;
    if total == 0 {
        return;
    }
    let dia = if rebar.main_x.count > 0 {
        rebar.main_x.dia
    } else {
        rebar.main_y.dia
    };
    let a = squid_n_core::section_shape::one_bar_area(dia);
    let depth = rebar.cover + rebar.shear.dia + dia / 2.0;
    let r = (d / 2.0 - depth).max(0.0);
    for i in 0..total {
        let th = 2.0 * std::f64::consts::PI * i as f64 / total as f64;
        fibers.push(PlasticFiber {
            y: r * th.cos(),
            z: r * th.sin(),
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
pub fn plastic_fibers(
    shape: &SectionShape,
    strength: &StrengthParams,
    kind: YieldModelKind,
) -> Vec<PlasticFiber> {
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
        SectionShape::RcRect { b, d, .. } => b.max(d),
        SectionShape::RcCircle { d, .. } => d,
        SectionShape::SrcRect { b, d, .. } => b.max(d),
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
pub fn plastic_fibers_at(
    shape: &SectionShape,
    strength: &StrengthParams,
    target: f64,
    ring: AnnulusRes,
) -> Vec<PlasticFiber> {
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
        SectionShape::RcRect { b, d, ref rebar } => {
            mesh_rect(&mut fibers, [0.0, 0.0], b, d, target, conc);
            rebar_fibers_rect(
                &mut fibers,
                rebar,
                b,
                d,
                strength.rebar_fy,
                strength.steel_e,
            );
        }
        SectionShape::RcCircle { d, ref rebar } => {
            mesh_annulus(&mut fibers, d, d / 2.0, ring.n_theta, ring.n_r_solid, conc);
            rebar_fibers_circle(&mut fibers, rebar, d, strength.rebar_fy, strength.steel_e);
        }
        SectionShape::SrcRect {
            b,
            d,
            ref rebar,
            steel_height,
            steel_width,
            steel_web_thick,
            steel_flange_thick,
            ..
        } => {
            mesh_rect(&mut fibers, [0.0, 0.0], b, d, target, conc);
            rebar_fibers_rect(
                &mut fibers,
                rebar,
                b,
                d,
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

    fibers
}
