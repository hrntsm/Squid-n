//! RC 配筋幾何の共通算定（断面検定・非線形解析・終局検定・MN／ファイバ配置の単一情報源）。
//!
//! - [`rebar_layer_clear`] — 段間あき k'
//! - [`rebar_layer_spacing`] — 段の中心間距離 s
//! - [`rebar_layer_depth_from_edge`] — 縁 → 第 n 段中心までの距離
//! - [`tension_dt`] — 引張縁 → 引張筋重心までの距離 dt
//! - [`rebar_tension_dt`] — せい方向主筋（`main_x`）の dt
//! - [`tension_effective_depth`] — 有効せい d_eff = D − dt
//! - [`pw_ratio`] — せん断補強筋比 pw

use crate::section_shape::{BarSet, RcRebar, ShearBar};

/// 多段配筋の段間あき k' [mm]（RC 配筋指針: `max(25, 1.5・dia)`）。
pub fn rebar_layer_clear(main: &BarSet) -> f64 {
    25.0_f64.max(1.5 * main.dia)
}

/// 多段配筋の段中心間距離 s = dia + k' [mm]。
pub fn rebar_layer_spacing(main: &BarSet) -> f64 {
    main.dia + rebar_layer_clear(main)
}

/// 縁から第 `layer` 段（0 始まり）の主筋中心までの距離 [mm]。
///
/// 1 段目（layer=0）は k1 = cover + shear.dia + main.dia/2。
/// 2 段目以降は中心間距離 [`rebar_layer_spacing`] だけ内側へ進む。
/// MN 曲面・非線形ファイバの主筋点配置と、検定用 [`tension_dt`] が同じ座標規約を使う。
pub fn rebar_layer_depth_from_edge(cover: f64, shear_dia: f64, main: &BarSet, layer: u32) -> f64 {
    let k1 = cover + shear_dia + main.dia / 2.0;
    if layer == 0 {
        return k1;
    }
    k1 + layer as f64 * rebar_layer_spacing(main)
}

/// 引張縁 → 引張筋重心までの距離 dt [mm]。
///
/// 各段の本数が等しいと仮定し、[`rebar_layer_depth_from_edge`] で置いた各段中心の
/// 平均とする。1 段なら k1、2 段なら (k1+k2)/2、一般に
/// `dt = k1 + (layers-1)/2・s`（RC 配筋指針の 2 段式に一致）。
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

/// 有効せい d_eff = D − dt [mm]（dt は [`tension_dt`] と同規約）。
pub fn tension_effective_depth(d: f64, cover: f64, shear_dia: f64, main: &BarSet) -> f64 {
    (d - tension_dt(cover, shear_dia, main)).max(0.0)
}

/// 有効せい d_eff = D − dt [mm]（dt は [`rebar_tension_dt`] と同規約）。
pub fn rebar_effective_depth(d: f64, rebar: &RcRebar) -> f64 {
    (d - rebar_tension_dt(rebar)).max(0.0)
}

/// せん断補強筋比 pw = (legs・π/4・dia²) / (b・pitch)。pitch<=0 のときは 0。
pub fn pw_ratio(shear: &ShearBar, b: f64) -> f64 {
    if shear.pitch <= 0.0 || b <= 0.0 {
        return 0.0;
    }
    let aw = shear.legs as f64 * std::f64::consts::PI / 4.0 * shear.dia * shear.dia;
    aw / (b * shear.pitch)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::section_shape::ShearBar;

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
}
