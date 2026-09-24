//! 断面形状から RC 検定方向の諸元を導く中立型。
//!
//! 旧 [`squid_n_core::section_shape::RcRebar`] 経路と新実配筋モデル経路の差異を
//! [`rc_bar_props`] が吸収し、検定ロジックへ共通の [`RcBarProps`] を渡す。

use squid_n_core::rc_rebar_geom::{pw_ratio, rebar_tension_dt, RectEdge};
use squid_n_core::section_shape::{
    bar_set_area, one_bar_area, RcBeamRebar, RcCircleColumnRebar, RcRebar, RcRectColumnRebar,
    SectionShape,
};

/// RC 検定の検討方向。
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum RcDirection {
    /// 強軸方向。
    Strong,
    /// 弱軸方向。
    Weak,
}

/// RC 検定方向の諸元。旧 RcRebar 経路と新実配筋モデル経路を共通化する。
///
/// `b_dir`・`d_dir` は検討方向の幅・せい [mm]、`at`・`ac`・`ag` は引張側・圧縮側・
/// 全主筋断面積 [mm²]、`dt` は引張縁〜引張鉄筋重心 [mm]、`d_eff` は有効せい [mm]、
/// `pw` は検討方向のせん断補強筋比。`top_bar` は梁の上端主筋を引張側とする場合 true で、
/// 付着検定の上端筋低減（`αt`）に用いる。`be`・`n_s` は靭性指針式のトラス機構有効幅 [mm]・
/// 中子筋本数で、不要時は 0。
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct RcBarProps {
    pub b_dir: f64,
    pub d_dir: f64,
    pub at: f64,
    pub ac: f64,
    pub ag: f64,
    pub d_eff: f64,
    pub dt: f64,
    pub main_dia: f64,
    pub n_tension: u32,
    pub cover: f64,
    pub shear_dia: f64,
    pub shear_pitch: f64,
    pub shear_legs: u32,
    pub pw: f64,
    pub top_bar: bool,
    pub be: f64,
    pub n_s: u32,
}

/// 断面形状から、指定方向・指定引張側の RC 諸元を返す。
///
/// 対象外（RC 以外・対応しない形状）・未入力・実配筋を生成できない配筋・
/// 有効せいが 0 以下なら `None`。
/// `tension_is_top` は梁の曲げ引張側で、柱・円形柱では無視する。
/// `shear_method_needs_be` が true のときのみ靭性指針式の `be`・`n_s` を算定する。
pub(crate) fn rc_bar_props(
    shape: &SectionShape,
    direction: RcDirection,
    tension_is_top: bool,
    shear_method_needs_be: bool,
) -> Option<RcBarProps> {
    match shape {
        SectionShape::RcRect { b, d, rebar } => {
            rc_rect_props(*b, *d, rebar, direction, shear_method_needs_be)
        }
        SectionShape::RcBeamRect { b, d, rebar } => rc_beam_rect_props(
            *b,
            *d,
            rebar,
            direction,
            tension_is_top,
            shear_method_needs_be,
        ),
        SectionShape::RcColumnRect { b, d, rebar } => rc_column_rect_props(
            *b,
            *d,
            rebar,
            direction,
            tension_is_top,
            shear_method_needs_be,
        ),
        SectionShape::RcColumnCircle { d, rebar } => {
            rc_column_circle_props(*d, rebar, shear_method_needs_be)
        }
        _ => None,
    }
}

/// 靭性指針式のトラス機構有効幅 `be` [mm] と中子筋本数 `n_s`。
///
/// `needs_be` が false なら `(0.0, 0)`。`be = (b_dir − 2(かぶり + せん断補強筋径/2)).max(1.0)`、
/// `n_s = (せん断補強脚数/2) − 1`（0 未満は 0）。
fn ductility_be_ns_from(
    b_dir: f64,
    cover: f64,
    shear_dia: f64,
    shear_legs: u32,
    needs_be: bool,
) -> (f64, u32) {
    if !needs_be {
        return (0.0, 0);
    }
    let be = (b_dir - 2.0 * (cover + shear_dia / 2.0)).max(1.0);
    let n_s = (shear_legs / 2).saturating_sub(1);
    (be, n_s)
}

/// せん断補強筋比 `Aw/(b·s)`。`b`・`s` が 0 以下なら 0。
fn pw_from_aw(aw: f64, b_dir: f64, pitch: f64) -> f64 {
    if b_dir <= 0.0 || pitch <= 0.0 {
        return 0.0;
    }
    aw / (b_dir * pitch)
}

/// 旧 `RcRect`（梁・柱兼用）の諸元。
fn rc_rect_props(
    b: f64,
    d: f64,
    rebar: &RcRebar,
    direction: RcDirection,
    needs_be: bool,
) -> Option<RcBarProps> {
    if b <= 0.0 || d <= 0.0 {
        return None;
    }
    let ag = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
    let (b_dir, d_dir, dt, at, main_dia, n_tension) = match direction {
        RcDirection::Strong => {
            let dt = rebar_tension_dt(rebar);
            (
                b,
                d,
                dt,
                bar_set_area(&rebar.main_x) / 2.0,
                rebar.main_x.dia,
                (rebar.main_x.count / 2).max(1),
            )
        }
        RcDirection::Weak => {
            let dt = rebar.cover + rebar.shear.dia + rebar.main_y.dia / 2.0;
            (
                d,
                b,
                dt,
                bar_set_area(&rebar.main_y) / 2.0,
                rebar.main_y.dia,
                (rebar.main_y.count / 2).max(1),
            )
        }
    };
    let d_eff = d_dir - dt;
    if d_eff <= 0.0 {
        return None;
    }
    let (be, n_s) = ductility_be_ns_from(
        b_dir,
        rebar.cover,
        rebar.shear.dia,
        rebar.shear.legs,
        needs_be,
    );
    Some(RcBarProps {
        b_dir,
        d_dir,
        at,
        ac: at,
        ag,
        d_eff,
        dt,
        main_dia,
        n_tension,
        cover: rebar.cover,
        shear_dia: rebar.shear.dia,
        shear_pitch: rebar.shear.pitch,
        shear_legs: rebar.shear.legs,
        pw: pw_ratio(&rebar.shear, b_dir),
        top_bar: false,
        be,
        n_s,
    })
}

/// 新 `RcBeamRect`（強軸のみ）の諸元。弱軸は `None`。
fn rc_beam_rect_props(
    b: f64,
    d: f64,
    rebar: &RcBeamRebar,
    direction: RcDirection,
    tension_is_top: bool,
    needs_be: bool,
) -> Option<RcBarProps> {
    if direction == RcDirection::Weak {
        return None;
    }
    if b <= 0.0 || d <= 0.0 || rebar.is_unset() || rebar.main_dia <= 0.0 {
        return None;
    }
    rebar.validate(b, d).ok()?;
    let bending = rebar.bending_steel(d, tension_is_top);
    let steel = bending.tension;
    if steel.effective_depth_mm <= 0.0 {
        return None;
    }
    let (be, n_s) = ductility_be_ns_from(
        b,
        rebar.cover,
        rebar.stirrup.dia,
        rebar.stirrup.legs,
        needs_be,
    );
    Some(RcBarProps {
        b_dir: b,
        d_dir: d,
        at: steel.area_mm2,
        ac: bending.compression.area_mm2,
        ag: rebar.total_main_area(),
        d_eff: steel.effective_depth_mm,
        dt: steel.centroid_from_edge_mm,
        main_dia: rebar.main_dia,
        n_tension: steel.count as u32,
        cover: rebar.cover,
        shear_dia: rebar.stirrup.dia,
        shear_pitch: rebar.stirrup.pitch,
        shear_legs: rebar.stirrup.legs,
        pw: rebar.pw(b),
        top_bar: tension_is_top,
        be,
        n_s,
    })
}

/// 新 `RcColumnRect` の諸元。強軸は上下辺、弱軸は左右辺の最外段 1 列を引張側とする。
fn rc_column_rect_props(
    b: f64,
    d: f64,
    rebar: &RcRectColumnRebar,
    direction: RcDirection,
    tension_is_top: bool,
    needs_be: bool,
) -> Option<RcBarProps> {
    if b <= 0.0 || d <= 0.0 || rebar.is_unset() || rebar.main_dia <= 0.0 {
        return None;
    }
    rebar.validate(b, d).ok()?;
    let (b_dir, d_dir, edge, aw, shear_legs) = match direction {
        RcDirection::Strong => (
            b,
            d,
            if tension_is_top {
                RectEdge::Top
            } else {
                RectEdge::Bottom
            },
            rebar.aw_x_mm2(),
            rebar.hoop.legs_x,
        ),
        RcDirection::Weak => (d, b, RectEdge::Left, rebar.aw_y_mm2(), rebar.hoop.legs_y),
    };
    let steel = rebar.edge_steel(edge, b, d);
    if steel.effective_depth_mm <= 0.0 {
        return None;
    }
    let (be, n_s) = ductility_be_ns_from(b_dir, rebar.cover, rebar.hoop.dia, shear_legs, needs_be);
    Some(RcBarProps {
        b_dir,
        d_dir,
        at: steel.area_mm2,
        ac: steel.area_mm2,
        ag: rebar.total_main_area(),
        d_eff: steel.effective_depth_mm,
        dt: steel.centroid_from_edge_mm,
        main_dia: rebar.main_dia,
        n_tension: steel.count as u32,
        cover: rebar.cover,
        shear_dia: rebar.hoop.dia,
        shear_pitch: rebar.hoop.pitch,
        shear_legs,
        pw: pw_from_aw(aw, b_dir, rebar.hoop.pitch),
        top_bar: false,
        be,
        n_s,
    })
}

/// 新 `RcColumnCircle` の諸元。等価正方形断面として扱い、方向によらず同じ値を返す。
///
/// 円形帯筋は 1 組 2 本（閉鎖フープ、検討方向あたり 2 本）として `shear_legs`・`pw` を算定する。
fn rc_column_circle_props(
    d: f64,
    rebar: &RcCircleColumnRebar,
    needs_be: bool,
) -> Option<RcBarProps> {
    if d <= 0.0 || rebar.is_unset() || rebar.main_dia <= 0.0 {
        return None;
    }
    rebar.validate(d).ok()?;
    let side = rebar.equivalent_square_side_mm(d);
    let d_eff = rebar.equivalent_effective_depth_mm(d);
    if side <= 0.0 || d_eff <= 0.0 {
        return None;
    }
    let cover = rebar.cover;
    let shear_dia = rebar.hoop.dia;
    let main_dia = rebar.main_dia;
    let dt = cover + shear_dia + main_dia / 2.0;
    let at = rebar.equivalent_tension_area_mm2();
    let n_tension = (at / one_bar_area(main_dia)).round() as u32;
    let (be, n_s) = ductility_be_ns_from(side, cover, shear_dia, 2, needs_be);
    Some(RcBarProps {
        b_dir: side,
        d_dir: side,
        at,
        ac: at,
        ag: rebar.total_main_area(),
        d_eff,
        dt,
        main_dia,
        n_tension,
        cover,
        shear_dia,
        shear_pitch: rebar.hoop.pitch,
        shear_legs: 2,
        pw: rebar.pw(side),
        top_bar: false,
        be,
        n_s,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::section_shape::{
        BarSet, BeamStirrup, CircleColumnHoop, RectColumnHoop, ShearBar,
    };

    fn old_rect() -> SectionShape {
        SectionShape::RcRect {
            b: 400.0,
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 8,
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
            },
        }
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
    fn test_rc_rect_strong_props() {
        let p = rc_bar_props(&old_rect(), RcDirection::Strong, true, false).unwrap();
        let a1 = one_bar_area(22.0);
        let a10 = one_bar_area(10.0);
        assert!((p.b_dir - 400.0).abs() < 1e-9);
        assert!((p.d_dir - 600.0).abs() < 1e-9);
        assert!((p.at - 4.0 * a1).abs() < 1e-9);
        assert!((p.ac - 4.0 * a1).abs() < 1e-9);
        assert!((p.ag - 12.0 * a1).abs() < 1e-9);
        assert!((p.dt - 61.0).abs() < 1e-9);
        assert!((p.d_eff - 539.0).abs() < 1e-9);
        assert!((p.pw - 2.0 * a10 / (400.0 * 100.0)).abs() < 1e-15);
        assert_eq!(p.n_tension, 4);
        assert_eq!(p.shear_legs, 2);
        assert!(!p.top_bar);
        assert_eq!(p.be, 0.0);
        assert_eq!(p.n_s, 0);
    }

    #[test]
    fn test_rc_rect_weak_props() {
        let p = rc_bar_props(&old_rect(), RcDirection::Weak, true, false).unwrap();
        let a1 = one_bar_area(22.0);
        assert!((p.b_dir - 600.0).abs() < 1e-9);
        assert!((p.d_dir - 400.0).abs() < 1e-9);
        assert!((p.at - 2.0 * a1).abs() < 1e-9);
        assert!((p.dt - 61.0).abs() < 1e-9);
        assert!((p.d_eff - 339.0).abs() < 1e-9);
        assert_eq!(p.n_tension, 2);
    }

    #[test]
    fn test_rc_rect_ductility_be_ns() {
        let p = rc_bar_props(&old_rect(), RcDirection::Strong, true, true).unwrap();
        assert!((p.be - (400.0 - 2.0 * (40.0 + 5.0))).abs() < 1e-9);
        assert_eq!(p.n_s, 0);
    }

    #[test]
    fn test_rc_beam_rect_top_tension() {
        let p = rc_bar_props(&beam_shape(), RcDirection::Strong, true, false).unwrap();
        let a1 = one_bar_area(22.0);
        assert!((p.at - 6.0 * a1).abs() < 1e-9);
        assert!((p.ac - 5.0 * a1).abs() < 1e-9);
        assert!((p.dt - (61.0 + 2.0 / 6.0 * 55.0)).abs() < 1e-9);
        assert!((p.ag - 11.0 * a1).abs() < 1e-9);
        assert_eq!(p.n_tension, 6);
        assert!(p.top_bar);
    }

    #[test]
    fn test_rc_beam_rect_bottom_tension() {
        let p = rc_bar_props(&beam_shape(), RcDirection::Strong, false, false).unwrap();
        let a1 = one_bar_area(22.0);
        assert!((p.at - 5.0 * a1).abs() < 1e-9);
        assert!((p.ac - 6.0 * a1).abs() < 1e-9);
        assert!((p.dt - (61.0 + 2.0 / 5.0 * 55.0)).abs() < 1e-9);
        assert_eq!(p.n_tension, 5);
        assert!(!p.top_bar);
    }

    #[test]
    fn test_rc_beam_rect_weak_none() {
        assert!(rc_bar_props(&beam_shape(), RcDirection::Weak, true, false).is_none());
    }

    #[test]
    fn test_rc_column_rect_strong_props() {
        let p = rc_bar_props(&column_rect_shape(), RcDirection::Strong, true, false).unwrap();
        let a1 = one_bar_area(22.0);
        let a10 = one_bar_area(10.0);
        assert!((p.b_dir - 600.0).abs() < 1e-9);
        assert!((p.d_dir - 700.0).abs() < 1e-9);
        assert!((p.at - 4.0 * a1).abs() < 1e-9);
        assert!((p.ag - 12.0 * a1).abs() < 1e-9);
        assert!((p.dt - 61.0).abs() < 1e-9);
        assert_eq!(p.n_tension, 4);
        assert_eq!(p.shear_legs, 2);
        assert!(!p.top_bar);
        assert!((p.pw - 2.0 * a10 / (600.0 * 100.0)).abs() < 1e-15);
    }

    #[test]
    fn test_rc_column_rect_weak_props() {
        let p = rc_bar_props(&column_rect_shape(), RcDirection::Weak, true, false).unwrap();
        let a1 = one_bar_area(22.0);
        let a10 = one_bar_area(10.0);
        // 旧 RcRect と同じく弱軸は b_dir=d, d_dir=b に入れ替える。
        assert!((p.b_dir - 700.0).abs() < 1e-9);
        assert!((p.d_dir - 600.0).abs() < 1e-9);
        assert!((p.at - 4.0 * a1).abs() < 1e-9);
        assert!((p.ag - 12.0 * a1).abs() < 1e-9);
        assert!((p.dt - 61.0).abs() < 1e-9);
        assert!((p.d_eff - 539.0).abs() < 1e-9);
        assert_eq!(p.n_tension, 4);
        assert_eq!(p.shear_legs, 3);
        assert!((p.pw - 3.0 * a10 / (700.0 * 100.0)).abs() < 1e-15);
    }

    #[test]
    fn test_rc_column_circle_equivalent_square() {
        let p = rc_bar_props(&column_circle_shape(), RcDirection::Strong, true, false).unwrap();
        let a1 = one_bar_area(22.0);
        let a10 = one_bar_area(10.0);
        let side = (std::f64::consts::PI * 600.0 * 600.0 / 4.0).sqrt();
        assert!((p.b_dir - side).abs() < 1e-9);
        assert!((p.d_dir - side).abs() < 1e-9);
        assert!((p.at - 2.0 * a1).abs() < 1e-9);
        assert!((p.ag - 8.0 * a1).abs() < 1e-9);
        assert!((p.dt - 61.0).abs() < 1e-9);
        assert_eq!(p.n_tension, 2);
        assert_eq!(p.shear_legs, 2);
        assert!(!p.top_bar);
        assert!((p.pw - 2.0 * a10 / (side * 100.0)).abs() < 1e-15);
        let weak = rc_bar_props(&column_circle_shape(), RcDirection::Weak, false, false).unwrap();
        assert_eq!(p, weak);
    }

    /// 円形柱の帯筋（1 組 2 本）が pw に反映され、許容・終局せん断が増える。
    #[test]
    fn test_rc_column_circle_hoop_contributes_to_shear() {
        use crate::rc::{axis_props_from_shape, rc_allow, shear_capacity_for};
        use crate::ultimate::UltimateShearOptions;
        use crate::LoadTerm;
        use squid_n_core::units::ConcreteClass;

        let circle = |pitch: f64| SectionShape::RcColumnCircle {
            d: 600.0,
            rebar: RcCircleColumnRebar {
                main_dia: 22.0,
                count: 8,
                cover: 40.0,
                hoop: CircleColumnHoop { dia: 10.0, pitch },
            },
        };
        let with_hoop = circle(100.0);
        let without_hoop = circle(100_000.0);

        let p = rc_bar_props(&with_hoop, RcDirection::Strong, true, false).unwrap();
        let p0 = rc_bar_props(&without_hoop, RcDirection::Strong, true, false).unwrap();
        assert!(p.pw > 0.0, "円形柱の pw が正: pw={}", p.pw);
        assert!(p0.pw < p.pw, "pitch 大では pw が小さい");

        let allow = rc_allow(24.0, ConcreteClass::Normal, "SD345", false);
        let props = axis_props_from_shape(&with_hoop, RcDirection::Strong, true).unwrap();
        let props0 = axis_props_from_shape(&without_hoop, RcDirection::Strong, true).unwrap();
        let qa = shear_capacity_for(&props, &allow, 1.5, LoadTerm::Short, true, true);
        let qa0 = shear_capacity_for(&props0, &allow, 1.5, LoadTerm::Short, true, true);
        assert!(qa > qa0, "許容せん断: with={qa}, without={qa0}");

        let opts = UltimateShearOptions::default();
        let qsu = super::super::rc_strength::member_shear_strength(&p, 24.0, 0.0, 3000.0, &opts);
        let qsu0 =
            super::super::rc_strength::member_shear_strength(&p0, 24.0, 0.0, 3000.0, &opts);
        assert!(qsu > qsu0, "終局せん断: with={qsu}, without={qsu0}");
    }

    #[test]
    fn test_rc_props_unset_none() {
        let beam = SectionShape::RcBeamRect {
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
        assert!(rc_bar_props(&beam, RcDirection::Strong, true, false).is_none());

        let column = SectionShape::RcColumnRect {
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
        assert!(rc_bar_props(&column, RcDirection::Strong, true, false).is_none());

        let circle = SectionShape::RcColumnCircle {
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
        assert!(rc_bar_props(&circle, RcDirection::Strong, true, false).is_none());
    }

    /// 実配筋が幾何的に成立しない矩形柱は諸元を生成しない。
    #[test]
    fn test_rc_props_inconsistent_column_none() {
        let column = SectionShape::RcColumnRect {
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
        assert!(rc_bar_props(&column, RcDirection::Strong, true, false).is_none());
        assert!(rc_bar_props(&column, RcDirection::Weak, true, false).is_none());
    }

    #[test]
    fn test_rc_props_non_rc_none() {
        let shape = SectionShape::SteelH {
            height: 500.0,
            width: 200.0,
            web_thick: 9.0,
            flange_thick: 16.0,
        };
        assert!(rc_bar_props(&shape, RcDirection::Strong, true, false).is_none());
    }
}
