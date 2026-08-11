//! RC 断面諸元の抽出（検討方向 1 軸分の断面諸元と、その素となる鉄筋量の算定）。
//!
//! [`AxisProps`] — 検討方向 1 軸分の断面諸元。
//! [`one_bar_area`] — 主筋 1 本あたりの断面積。
//! [`bar_set_area`] — 主筋セットの総断面積。
//! [`tension_dt`] — 引張縁 → 引張筋重心までの距離 dt。
//! [`pw_ratio`] — せん断補強筋比 pw。
//! [`rect_axis_props`] — 矩形断面 1 軸分の断面諸元。
//! [`rect_axis_props_strong`] — 強軸曲げ（mz）用の断面諸元。
//! [`rect_axis_props_weak`] — 弱軸曲げ（my）用の断面諸元。
//! [`circle_axis_props`] — 円形柱の等価矩形断面諸元。

use squid_n_core::model::Section;
use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};

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

/// 主筋セットの総断面積 [mm²]。
pub(crate) fn bar_set_area(bar: &BarSet) -> f64 {
    bar.count as f64 * one_bar_area(bar.dia)
}

/// 断面の主筋（せい方向 `main_x`）の引張筋重心位置 dt [mm]。
///
/// [`tension_dt`] へ委譲し、**多段配筋の段数を考慮する**。接合部検定・SRC パネル・
/// 壁脚・PCa 水平接合面が用いる（断面算定側と配筋の解釈を揃えるため。かつては
/// 1 段目重心 `cover + shear.dia + main_x.dia/2` で固定しており、2 段配筋を
/// 入力すると「断面算定は 2 段として計算、接合部検定は 1 段として計算」という
/// 規約の割れが生じていた）。
pub(crate) fn rebar_tension_dt(rebar: &RcRebar) -> f64 {
    tension_dt(rebar.cover, rebar.shear.dia, &rebar.main_x)
}

/// 引張縁 → 引張筋重心までの距離 dt [mm]。
///
/// 1 段筋（`layers<=1`）は重心 k1 = cover + shear.dia + main.dia/2。
/// 2 段以上は RC 配筋指針式（2 段の場合）
/// `k2 = k1 + D1/2 + k' + D2/2`（`k' = max(25, 1.5・dia)`, `D1=D2=main.dia`）
/// により `dt = (k1+k2)/2` とする。3 段以上は各段が等間隔 `s = dia + k'` で
/// 並び、各段の本数が等しいと仮定して重心を平均で一般化する:
/// `dt = k1 + (layers-1)/2・s`（layers=2 で上式に一致）。
pub(crate) fn tension_dt(cover: f64, shear_dia: f64, main: &BarSet) -> f64 {
    let k1 = cover + shear_dia + main.dia / 2.0;
    if main.layers <= 1 {
        return k1;
    }
    let k_prime = 25.0_f64.max(1.5 * main.dia);
    let s = main.dia + k_prime;
    k1 + (main.layers as f64 - 1.0) / 2.0 * s
}

/// せん断補強筋比 pw = (legs・π/4・dia²) / (b・pitch)。pitch<=0 のときは 0。
pub(crate) fn pw_ratio(shear: &ShearBar, b: f64) -> f64 {
    if shear.pitch <= 0.0 || b <= 0.0 {
        return 0.0;
    }
    let aw = shear.legs as f64 * std::f64::consts::PI / 4.0 * shear.dia * shear.dia;
    aw / (b * shear.pitch)
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
    ///
    /// 1 段筋では 1 段目重心 `k1 = かぶり + 帯筋径 + 主筋径/2` に一致し、2 段筋では
    /// 段間隔ぶん引張縁から遠ざかる。かつては段数を無視して常に k1 を返していたため、
    /// 同じモデルでも断面算定は 2 段・接合部検定は 1 段として計算していた。
    #[test]
    fn test_rebar_tension_dt_considers_layers() {
        let k1 = 40.0 + 10.0 + 25.0 / 2.0;
        assert!((rebar_tension_dt(&rebar(1)) - k1).abs() < 1e-9);

        // 2 段: k' = max(25, 1.5·25) = 37.5、s = 25 + 37.5 = 62.5 → dt = k1 + s/2。
        let two = rebar_tension_dt(&rebar(2));
        assert!((two - (k1 + 62.5 / 2.0)).abs() < 1e-9, "dt(2段)={two}");
        assert!(two > k1, "2 段筋の重心は 1 段筋より引張縁から遠いはず");

        // 断面算定側（`tension_dt`）と一致する（規約の割れがないこと）。
        let r = rebar(2);
        assert!((two - tension_dt(r.cover, r.shear.dia, &r.main_x)).abs() < 1e-12);
    }
}
