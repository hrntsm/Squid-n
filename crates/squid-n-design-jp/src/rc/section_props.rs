//! RC 断面諸元の抽出（検討方向 1 軸分の断面諸元と、その素となる鉄筋量の算定）。
//!
//! [`AxisProps`] — 検討方向 1 軸分の断面諸元。
//! [`one_bar_area`] — 主筋 1 本あたりの断面積（core の再エクスポート）。
//! [`rect_axis_props`] — 矩形断面 1 軸分の断面諸元。
//! [`rect_axis_props_strong`] — 強軸曲げ（mz）用の断面諸元。
//! [`rect_axis_props_weak`] — 弱軸曲げ（my）用の断面諸元。
//! [`circle_axis_props`] — 円形柱の等価矩形断面諸元。
//! [`RcRebarInfo`] — 構造規定・付着検定用の鉄筋情報（中立型）。
//! [`rebar_info_from_shape`] — 断面形状から鉄筋情報を引く。
//!
//! 主筋断面積・dt・pw は [`squid_n_core::rc_rebar_geom`] /
//! [`squid_n_core::section_shape::{one_bar_area, bar_set_area}`] を単一情報源とする。

use squid_n_core::model::Section;
pub(crate) use squid_n_core::rc_rebar_geom::{pw_ratio, tension_dt};
pub(crate) use squid_n_core::section_shape::{bar_set_area, one_bar_area};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape};

use crate::ultimate::rc_props::{rc_bar_props, RcDirection};

/// 検討方向 1 軸分の断面諸元。
#[derive(Clone, Copy)]
pub(crate) struct AxisProps {
    /// 検討方向の幅 [mm]（強軸曲げなら sec.width 等）。
    pub(crate) b: f64,
    /// 検討方向のせい D [mm]。
    pub(crate) d_full: f64,
    /// 引張縁から引張筋重心までの距離 dt [mm]。
    pub(crate) dt: f64,
    /// 有効せい d = D - dt [mm]。
    pub(crate) d: f64,
    /// 引張鉄筋断面積 at [mm²]（片側）。
    pub(crate) at: f64,
    /// 圧縮鉄筋断面積 ac [mm²]（片側、at と同値の対称複筋仮定）。
    pub(crate) ac: f64,
    /// 応力中心間距離 j = 7d/8 [mm]。
    pub(crate) j: f64,
    /// せん断補強筋比 pw。
    pub(crate) pw: f64,
}

/// 矩形断面 1 軸分の断面諸元を算定する。
///
/// `width_dir_b`: 検討方向の幅、`depth_dir_d`: 検討方向のせい、
/// `main`: 当該方向の主筋（強軸曲げは main_x、弱軸曲げは main_y）。
pub(crate) fn rect_axis_props(
    width_dir_b: f64,
    depth_dir_d: f64,
    main: &BarSet,
    rebar: &RcRebar,
) -> AxisProps {
    let dt = tension_dt(rebar.cover, rebar.shear.dia, main);
    let d = depth_dir_d - dt;
    let at = bar_set_area(main) / 2.0;
    AxisProps {
        b: width_dir_b,
        d_full: depth_dir_d,
        dt,
        d,
        at,
        ac: at,
        j: 7.0 * d / 8.0,
        pw: pw_ratio(&rebar.shear, width_dir_b),
    }
}

/// 強軸曲げ（mz）用の断面諸元。b=sec.width, D=sec.depth, 主筋=main_x。
pub(crate) fn rect_axis_props_strong(sec: &Section, rebar: &RcRebar) -> AxisProps {
    rect_axis_props(sec.width, sec.depth, &rebar.main_x, rebar)
}

/// 弱軸曲げ（my）用の断面諸元。b=sec.depth, D=sec.width, 主筋=main_y。
pub(crate) fn rect_axis_props_weak(sec: &Section, rebar: &RcRebar) -> AxisProps {
    rect_axis_props(sec.depth, sec.width, &rebar.main_y, rebar)
}

/// 円形柱の等価矩形断面諸元。b=(D/2)√π、せい=D。
/// 引張筋本数 nt = ng/4+1（ng = 全主筋本数、`rebar.main_x.count` を採用）。
/// 対称複筋（at=ac）を仮定する。
pub(crate) fn circle_axis_props(d_full: f64, rebar: &RcRebar) -> AxisProps {
    let b = (d_full / 2.0) * std::f64::consts::PI.sqrt();
    let ng = rebar.main_x.count as f64;
    let nt = ng / 4.0 + 1.0;
    let at = nt * one_bar_area(rebar.main_x.dia);
    let dt = tension_dt(rebar.cover, rebar.shear.dia, &rebar.main_x);
    let d = d_full - dt;
    AxisProps {
        b,
        d_full,
        dt,
        d,
        at,
        ac: at,
        j: 7.0 * d / 8.0,
        pw: pw_ratio(&rebar.shear, b),
    }
}

/// 断面形状から検討方向 1 軸分の断面諸元を引く。
///
/// 中立型 [`rc_bar_props`] に委譲し、未対応形状・未入力・実配筋を生成できない配筋・
/// 有効せい 0 以下なら `None`。
/// 旧 `RcCircle` は [`rc_bar_props`] 非対応のため [`circle_axis_props`] で扱う。
/// `tension_is_top` は梁の曲げ引張側（柱・円形柱では無視）。`j = 7d/8`。
pub(crate) fn axis_props_from_shape(
    shape: &SectionShape,
    direction: RcDirection,
    tension_is_top: bool,
) -> Option<AxisProps> {
    if let SectionShape::RcCircle { d, rebar } = shape {
        return Some(circle_axis_props(*d, rebar));
    }
    let p = rc_bar_props(shape, direction, tension_is_top, false)?;
    Some(AxisProps {
        b: p.b_dir,
        d_full: p.d_dir,
        dt: p.dt,
        d: p.d_eff,
        at: p.at,
        ac: p.ac,
        j: 7.0 * p.d_eff / 8.0,
        pw: p.pw,
    })
}

/// 構造規定・付着検定が必要とする鉄筋情報（旧 RcRebar 経路と新実モデルの共通化）。
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct RcRebarInfo {
    pub cover: f64,
    pub main_dia: f64,
    /// 全主筋本数（柱の最小本数規定・主筋比に用いる）。
    pub main_count: u32,
    /// 片側 1 列あたりの本数（円形柱は ng/4+1 相当）。表示・付着の補助。
    pub main_count_per_side: u32,
    /// 全主筋断面積 [mm²]。
    pub main_area: f64,
    /// 検定方向の引張側主筋本数（付着検定の本数）。
    pub tension_count: u32,
    /// 検定方向の引張側の段数（付着検定の `layers`。多段で 0.6 低減に用いる）。
    pub tension_layers: u32,
    /// 検定方向の引張側最外段の本数（付着検定 1999 の n1）。
    pub tension_first_layer_count: f64,
    /// 付着検定 1991 の φ に用いる引張筋本数。
    pub tension_count_1991: f64,
    pub shear_dia: f64,
    pub shear_pitch: f64,
    pub shear_legs: u32,
    pub is_circle: bool,
}

/// 断面形状から鉄筋情報を返す。`tension_is_top` は梁の引張側（付着・段数）。
/// 旧 RcRect / RcCircle / RcBeamRect / RcColumnRect / RcColumnCircle を対象とし、
/// 対象外・未入力・実配筋を生成できない配筋は None。
pub(crate) fn rebar_info_from_shape(
    shape: &SectionShape,
    tension_is_top: bool,
) -> Option<RcRebarInfo> {
    match shape {
        SectionShape::RcRect { rebar, .. } => Some(RcRebarInfo {
            cover: rebar.cover,
            main_dia: rebar.main_x.dia,
            main_count: rebar.main_x.count + rebar.main_y.count,
            main_count_per_side: rebar.main_x.count,
            main_area: bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y),
            tension_count: rebar.main_x.count,
            tension_layers: rebar.main_x.layers.max(1),
            tension_first_layer_count: rebar.main_x.count as f64
                / rebar.main_x.layers.max(1) as f64,
            tension_count_1991: (rebar.main_x.count as f64 / 2.0).max(1.0),
            shear_dia: rebar.shear.dia,
            shear_pitch: rebar.shear.pitch,
            shear_legs: rebar.shear.legs,
            is_circle: false,
        }),
        SectionShape::RcCircle { rebar, .. } => Some(RcRebarInfo {
            cover: rebar.cover,
            main_dia: rebar.main_x.dia,
            main_count: rebar.main_x.count,
            main_count_per_side: rebar.main_x.count / 4 + 1,
            main_area: bar_set_area(&rebar.main_x),
            tension_count: rebar.main_x.count,
            tension_layers: rebar.main_x.layers.max(1),
            tension_first_layer_count: rebar.main_x.count as f64
                / rebar.main_x.layers.max(1) as f64,
            tension_count_1991: (rebar.main_x.count as f64 / 2.0).max(1.0),
            shear_dia: rebar.shear.dia,
            shear_pitch: rebar.shear.pitch,
            shear_legs: rebar.shear.legs,
            is_circle: true,
        }),
        SectionShape::RcBeamRect { b, d, rebar } => {
            if rebar.is_unset() {
                return None;
            }
            rebar.validate(*b, *d).ok()?;
            let top_count: u32 = rebar.top.iter().sum();
            let bottom_count: u32 = rebar.bottom.iter().sum();
            let (tension_count, tension_layers, first_layer_count) = if tension_is_top {
                (
                    top_count,
                    rebar.top.len().max(1) as u32,
                    rebar.top.first().copied().unwrap_or(0),
                )
            } else {
                (
                    bottom_count,
                    rebar.bottom.len().max(1) as u32,
                    rebar.bottom.first().copied().unwrap_or(0),
                )
            };
            Some(RcRebarInfo {
                cover: rebar.cover,
                main_dia: rebar.main_dia,
                main_count: top_count + bottom_count,
                main_count_per_side: tension_count,
                main_area: rebar.total_main_area(),
                tension_count,
                tension_layers,
                tension_first_layer_count: first_layer_count as f64,
                tension_count_1991: tension_count as f64,
                shear_dia: rebar.stirrup.dia,
                shear_pitch: rebar.stirrup.pitch,
                shear_legs: rebar.stirrup.legs,
                is_circle: false,
            })
        }
        SectionShape::RcColumnRect { b, d, rebar } => {
            if rebar.is_unset() {
                return None;
            }
            rebar.validate(*b, *d).ok()?;
            let nx = rebar.x.len() as u32;
            let ny = rebar.y.len() as u32;
            let sx: u32 = rebar.x.iter().sum();
            let sy: u32 = rebar.y.iter().sum();
            let main_count = (2 * sx + 2 * sy).saturating_sub(4 * nx * ny);
            Some(RcRebarInfo {
                cover: rebar.cover,
                main_dia: rebar.main_dia,
                main_count,
                main_count_per_side: rebar.x.first().copied().unwrap_or(0),
                main_area: rebar.total_main_area(),
                tension_count: main_count,
                tension_layers: 1,
                tension_first_layer_count: main_count as f64,
                tension_count_1991: main_count as f64,
                shear_dia: rebar.hoop.dia,
                shear_pitch: rebar.hoop.pitch,
                shear_legs: rebar.hoop.legs_x.max(rebar.hoop.legs_y),
                is_circle: false,
            })
        }
        SectionShape::RcColumnCircle { d, rebar } => {
            if rebar.is_unset() {
                return None;
            }
            rebar.validate(*d).ok()?;
            Some(RcRebarInfo {
                cover: rebar.cover,
                main_dia: rebar.main_dia,
                main_count: rebar.count,
                main_count_per_side: rebar.count / 4 + 1,
                main_area: rebar.total_main_area(),
                tension_count: rebar.count,
                tension_layers: 1,
                tension_first_layer_count: rebar.count as f64,
                tension_count_1991: rebar.count as f64,
                shear_dia: rebar.hoop.dia,
                shear_pitch: rebar.hoop.pitch,
                shear_legs: 2,
                is_circle: true,
            })
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::SectionId;
    use squid_n_core::section_shape::{
        BeamStirrup, CircleColumnHoop, RcBeamRebar, RcCircleColumnRebar, RcRectColumnRebar,
        RectColumnHoop, ShearBar,
    };

    fn old_rect_rebar() -> RcRebar {
        RcRebar {
            main_x: BarSet {
                count: 6,
                dia: 22.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 4,
                dia: 22.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
            },
        }
    }

    fn old_rect_shape() -> SectionShape {
        SectionShape::RcRect {
            b: 300.0,
            d: 600.0,
            rebar: old_rect_rebar(),
        }
    }

    fn old_rect_section() -> Section {
        let mut sec = Section::zero(SectionId(0), "test".to_string());
        sec.width = 300.0;
        sec.depth = 600.0;
        sec
    }

    fn beam_shape() -> SectionShape {
        SectionShape::RcBeamRect {
            b: 400.0,
            d: 600.0,
            rebar: RcBeamRebar {
                main_dia: 22.0,
                top: vec![4, 2],
                bottom: vec![3, 2],
                cover: 40.0,
                stirrup: BeamStirrup {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
        }
    }

    fn column_rect_shape() -> SectionShape {
        SectionShape::RcColumnRect {
            b: 600.0,
            d: 700.0,
            rebar: RcRectColumnRebar {
                main_dia: 22.0,
                x: vec![4, 2],
                y: vec![4],
                cover: 40.0,
                hoop: RectColumnHoop {
                    dia: 10.0,
                    pitch: 100.0,
                    legs_x: 2,
                    legs_y: 3,
                },
            },
        }
    }

    fn column_circle_shape() -> SectionShape {
        SectionShape::RcColumnCircle {
            d: 600.0,
            rebar: RcCircleColumnRebar {
                main_dia: 22.0,
                count: 8,
                cover: 40.0,
                hoop: CircleColumnHoop {
                    dia: 10.0,
                    pitch: 100.0,
                },
            },
        }
    }

    #[test]
    fn test_axis_props_from_shape_old_rect_matches_legacy() {
        let shape = old_rect_shape();
        let rebar = old_rect_rebar();
        let sec = old_rect_section();
        let a1 = one_bar_area(22.0);
        let a10 = one_bar_area(10.0);

        let strong = axis_props_from_shape(&shape, RcDirection::Strong, true).unwrap();
        assert!((strong.b - 300.0).abs() < 1e-9);
        assert!((strong.d_full - 600.0).abs() < 1e-9);
        assert!((strong.dt - 61.0).abs() < 1e-9);
        assert!((strong.d - 539.0).abs() < 1e-9);
        assert!((strong.at - 3.0 * a1).abs() < 1e-9);
        assert!((strong.ac - 3.0 * a1).abs() < 1e-9);
        assert!((strong.j - 7.0 * 539.0 / 8.0).abs() < 1e-9);
        assert!((strong.pw - 2.0 * a10 / (300.0 * 100.0)).abs() < 1e-15);
        let legacy_strong = rect_axis_props_strong(&sec, &rebar);
        assert_eq!(strong.b, legacy_strong.b);
        assert_eq!(strong.d_full, legacy_strong.d_full);
        assert_eq!(strong.dt, legacy_strong.dt);
        assert_eq!(strong.d, legacy_strong.d);
        assert_eq!(strong.at, legacy_strong.at);
        assert_eq!(strong.ac, legacy_strong.ac);
        assert_eq!(strong.j, legacy_strong.j);
        assert_eq!(strong.pw, legacy_strong.pw);

        let weak = axis_props_from_shape(&shape, RcDirection::Weak, true).unwrap();
        assert!((weak.b - 600.0).abs() < 1e-9);
        assert!((weak.d_full - 300.0).abs() < 1e-9);
        assert!((weak.dt - 61.0).abs() < 1e-9);
        assert!((weak.d - 239.0).abs() < 1e-9);
        assert!((weak.at - 2.0 * a1).abs() < 1e-9);
        assert!((weak.ac - 2.0 * a1).abs() < 1e-9);
        let legacy_weak = rect_axis_props_weak(&sec, &rebar);
        assert_eq!(weak.b, legacy_weak.b);
        assert_eq!(weak.d_full, legacy_weak.d_full);
        assert_eq!(weak.dt, legacy_weak.dt);
        assert_eq!(weak.d, legacy_weak.d);
        assert_eq!(weak.at, legacy_weak.at);
        assert_eq!(weak.ac, legacy_weak.ac);
        assert_eq!(weak.j, legacy_weak.j);
        assert_eq!(weak.pw, legacy_weak.pw);
    }

    #[test]
    fn test_axis_props_from_shape_beam_top_tension() {
        let p = axis_props_from_shape(&beam_shape(), RcDirection::Strong, true).unwrap();
        let a1 = one_bar_area(22.0);
        let dt = 61.0 + 2.0 / 6.0 * 55.0;
        assert!((p.at - 6.0 * a1).abs() < 1e-9);
        assert!((p.ac - 5.0 * a1).abs() < 1e-9);
        assert!((p.dt - dt).abs() < 1e-9);
        assert!((p.d - (600.0 - dt)).abs() < 1e-9);
        assert!((p.j - 7.0 * (600.0 - dt) / 8.0).abs() < 1e-9);
    }

    #[test]
    fn test_axis_props_from_shape_column_rect() {
        let a1 = one_bar_area(22.0);
        let strong = axis_props_from_shape(&column_rect_shape(), RcDirection::Strong, true).unwrap();
        assert!((strong.at - 4.0 * a1).abs() < 1e-9);
        assert!((strong.ac - 4.0 * a1).abs() < 1e-9);

        let weak = axis_props_from_shape(&column_rect_shape(), RcDirection::Weak, true).unwrap();
        assert!((weak.b - 700.0).abs() < 1e-9);
        assert!((weak.d_full - 600.0).abs() < 1e-9);
        assert!((weak.at - 4.0 * a1).abs() < 1e-9);
    }

    #[test]
    fn test_axis_props_from_shape_circle_matches_props() {
        let shape = column_circle_shape();
        let p = axis_props_from_shape(&shape, RcDirection::Strong, true).unwrap();
        let r = rc_bar_props(&shape, RcDirection::Strong, true, false).unwrap();
        let side = (600.0 / 2.0) * std::f64::consts::PI.sqrt();
        assert!((p.b - side).abs() < 1e-9);
        assert!((p.b - r.b_dir).abs() < 1e-12);
        assert!((p.d_full - r.d_dir).abs() < 1e-12);
        assert!((p.dt - r.dt).abs() < 1e-12);
        assert!((p.d - r.d_eff).abs() < 1e-12);
        assert!((p.at - r.at).abs() < 1e-12);
        assert!((p.ac - r.ac).abs() < 1e-12);
        assert!((p.pw - r.pw).abs() < 1e-12);
    }

    #[test]
    fn test_axis_props_from_shape_old_circle_matches_legacy() {
        let rebar = old_rect_rebar();
        let shape = SectionShape::RcCircle {
            d: 600.0,
            rebar: rebar.clone(),
        };
        let p = axis_props_from_shape(&shape, RcDirection::Strong, true).unwrap();
        let legacy = circle_axis_props(600.0, &rebar);
        assert_eq!(p.b, legacy.b);
        assert_eq!(p.d_full, legacy.d_full);
        assert_eq!(p.dt, legacy.dt);
        assert_eq!(p.d, legacy.d);
        assert_eq!(p.at, legacy.at);
        assert_eq!(p.ac, legacy.ac);
        assert_eq!(p.j, legacy.j);
        assert_eq!(p.pw, legacy.pw);
    }

    #[test]
    fn test_axis_props_from_shape_none() {
        let unset_beam = SectionShape::RcBeamRect {
            b: 400.0,
            d: 600.0,
            rebar: RcBeamRebar {
                main_dia: 22.0,
                top: vec![],
                bottom: vec![],
                cover: 40.0,
                stirrup: BeamStirrup {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
        };
        assert!(axis_props_from_shape(&unset_beam, RcDirection::Strong, true).is_none());
        assert!(axis_props_from_shape(&unset_beam, RcDirection::Weak, true).is_none());

        let steel = SectionShape::SteelH {
            height: 500.0,
            width: 200.0,
            web_thick: 9.0,
            flange_thick: 16.0,
        };
        assert!(axis_props_from_shape(&steel, RcDirection::Strong, true).is_none());
    }

    fn old_rect_layered_shape() -> SectionShape {
        SectionShape::RcRect {
            b: 300.0,
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 6,
                    dia: 22.0,
                    layers: 2,
                },
                main_y: BarSet {
                    count: 4,
                    dia: 22.0,
                    layers: 1,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
        }
    }

    #[test]
    fn test_rebar_info_from_shape_old_rect() {
        let info = rebar_info_from_shape(&old_rect_layered_shape(), true).unwrap();
        assert!((info.main_dia - 22.0).abs() < 1e-9);
        assert_eq!(info.main_count, 10);
        assert_eq!(info.main_count_per_side, 6);
        assert!((info.main_area - 10.0 * one_bar_area(22.0)).abs() < 1e-9);
        assert_eq!(info.tension_count, 6);
        assert_eq!(info.tension_layers, 2);
        assert!((info.tension_first_layer_count - 3.0).abs() < 1e-9);
        assert!((info.tension_count_1991 - 3.0).abs() < 1e-9);
        assert!((info.shear_pitch - 100.0).abs() < 1e-9);
        assert!(!info.is_circle);
    }

    #[test]
    fn test_rebar_info_from_shape_beam() {
        let top = rebar_info_from_shape(&beam_shape(), true).unwrap();
        assert_eq!(top.main_count, 11);
        assert!((top.main_area - 11.0 * one_bar_area(22.0)).abs() < 1e-9);
        assert_eq!(top.tension_count, 6);
        assert_eq!(top.tension_layers, 2);
        assert!((top.tension_first_layer_count - 4.0).abs() < 1e-9);
        assert!((top.tension_count_1991 - 6.0).abs() < 1e-9);

        let bottom = rebar_info_from_shape(&beam_shape(), false).unwrap();
        assert_eq!(bottom.main_count, 11);
        assert!((bottom.main_area - 11.0 * one_bar_area(22.0)).abs() < 1e-9);
        assert_eq!(bottom.tension_count, 5);
        assert_eq!(bottom.tension_layers, 2);
        assert!((bottom.tension_first_layer_count - 3.0).abs() < 1e-9);
        assert!((bottom.tension_count_1991 - 5.0).abs() < 1e-9);
    }

    #[test]
    fn test_rebar_info_from_shape_column_rect() {
        let info = rebar_info_from_shape(&column_rect_shape(), true).unwrap();
        assert_eq!(info.main_count, 12);
        assert_eq!(info.main_count_per_side, 4);
        assert!((info.main_area - 12.0 * one_bar_area(22.0)).abs() < 1e-9);
        assert_eq!(info.tension_first_layer_count, 12.0);
        assert_eq!(info.tension_count_1991, 12.0);
        assert_eq!(info.shear_legs, 3);
        assert!(!info.is_circle);
    }

    #[test]
    fn test_rebar_info_from_shape_column_circle() {
        let info = rebar_info_from_shape(&column_circle_shape(), true).unwrap();
        assert_eq!(info.main_count, 8);
        assert_eq!(info.main_count_per_side, 3);
        assert!((info.main_area - 8.0 * one_bar_area(22.0)).abs() < 1e-9);
        assert_eq!(info.tension_count, 8);
        assert_eq!(info.tension_first_layer_count, 8.0);
        assert_eq!(info.tension_count_1991, 8.0);
        assert_eq!(info.shear_legs, 2);
        assert!(info.is_circle);
    }

    #[test]
    fn test_rebar_info_from_shape_circle() {
        let circle = SectionShape::RcCircle {
            d: 600.0,
            rebar: old_rect_rebar(),
        };
        let info = rebar_info_from_shape(&circle, true).unwrap();
        assert!((info.main_dia - 22.0).abs() < 1e-9);
        assert_eq!(info.main_count, 6);
        assert_eq!(info.main_count_per_side, 2);
        assert!((info.main_area - 6.0 * one_bar_area(22.0)).abs() < 1e-9);
        assert_eq!(info.tension_count, 6);
        assert_eq!(info.tension_layers, 1);
        assert!((info.tension_first_layer_count - 6.0).abs() < 1e-9);
        assert!((info.tension_count_1991 - 3.0).abs() < 1e-9);
        assert!((info.shear_pitch - 100.0).abs() < 1e-9);
        assert!(info.is_circle);
    }

    #[test]
    fn test_rebar_info_from_shape_none() {
        let unset_beam = SectionShape::RcBeamRect {
            b: 400.0,
            d: 600.0,
            rebar: RcBeamRebar {
                main_dia: 22.0,
                top: vec![],
                bottom: vec![],
                cover: 40.0,
                stirrup: BeamStirrup {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
        };
        assert!(rebar_info_from_shape(&unset_beam, true).is_none());

        let unset_column = SectionShape::RcColumnRect {
            b: 600.0,
            d: 700.0,
            rebar: RcRectColumnRebar {
                main_dia: 22.0,
                x: vec![],
                y: vec![],
                cover: 40.0,
                hoop: RectColumnHoop {
                    dia: 10.0,
                    pitch: 100.0,
                    legs_x: 2,
                    legs_y: 3,
                },
            },
        };
        assert!(rebar_info_from_shape(&unset_column, true).is_none());

        let unset_circle = SectionShape::RcColumnCircle {
            d: 600.0,
            rebar: RcCircleColumnRebar {
                main_dia: 22.0,
                count: 0,
                cover: 40.0,
                hoop: CircleColumnHoop {
                    dia: 10.0,
                    pitch: 100.0,
                },
            },
        };
        assert!(rebar_info_from_shape(&unset_circle, true).is_none());
    }

    /// 実配筋が幾何的に成立しない矩形柱は、諸元・鉄筋情報とも `None`。
    #[test]
    fn test_inconsistent_column_rebar_none() {
        let shape = SectionShape::RcColumnRect {
            b: 600.0,
            d: 700.0,
            rebar: RcRectColumnRebar {
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
            },
        };
        assert!(axis_props_from_shape(&shape, RcDirection::Strong, true).is_none());
        assert!(axis_props_from_shape(&shape, RcDirection::Weak, true).is_none());
        assert!(rebar_info_from_shape(&shape, true).is_none());
    }
}
