//! RC 断面諸元の抽出（検討方向 1 軸分の断面諸元と、その素となる鉄筋量の算定）。
//!
//! [`AxisProps`] — 検討方向 1 軸分の断面諸元。
//! [`one_bar_area`] — 主筋 1 本あたりの断面積。
//! [`rect_axis_props`] — 矩形断面 1 軸分の断面諸元。
//! [`rect_axis_props_strong`] — 強軸曲げ（mz）用の断面諸元。
//! [`rect_axis_props_weak`] — 弱軸曲げ（my）用の断面諸元。
//! [`circle_axis_props`] — 円形柱の等価矩形断面諸元。
//!
//! 主筋断面積・dt・pw は [`squid_n_core::rc_rebar_geom`] /
//! [`squid_n_core::section_shape::bar_set_area`] を単一情報源とする。

use squid_n_core::model::Section;
pub(crate) use squid_n_core::rc_rebar_geom::{pw_ratio, tension_dt};
pub(crate) use squid_n_core::section_shape::bar_set_area;
use squid_n_core::section_shape::{BarSet, RcRebar};

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

/// 主筋 1 本あたりの断面積 [mm²]。
pub(crate) fn one_bar_area(dia: f64) -> f64 {
    let r = dia / 2.0;
    std::f64::consts::PI * r * r
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

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::rc_rebar_geom::rebar_tension_dt;
    use squid_n_core::section_shape::ShearBar;

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

    /// 接合部検定が使う dt は**多段配筋の段数を考慮する**（断面算定側と同一規約）。
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
}
