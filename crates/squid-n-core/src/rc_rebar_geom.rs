//! RC 配筋幾何の共通算定（断面検定・非線形解析・終局検定の単一情報源）。
//!
//! - [`tension_dt`] — 引張縁 → 引張筋重心までの距離 dt
//! - [`rebar_tension_dt`] — せい方向主筋（`main_x`）の dt
//! - [`tension_effective_depth`] — 有効せい d_eff = D − dt
//! - [`pw_ratio`] — せん断補強筋比 pw

use crate::section_shape::{BarSet, RcRebar, ShearBar};

/// 引張縁 → 引張筋重心までの距離 dt [mm]。
///
/// 1 段筋（`layers<=1`）は重心 k1 = cover + shear.dia + main.dia/2。
/// 2 段以上は RC 配筋指針式（2 段の場合）
/// `k2 = k1 + D1/2 + k' + D2/2`（`k' = max(25, 1.5・dia)`, `D1=D2=main.dia`）
/// により `dt = (k1+k2)/2` とする。3 段以上は各段が等間隔 `s = dia + k'` で
/// 並び、各段の本数が等しいと仮定して重心を平均で一般化する:
/// `dt = k1 + (layers-1)/2・s`（layers=2 で上式に一致）。
pub fn tension_dt(cover: f64, shear_dia: f64, main: &BarSet) -> f64 {
    let k1 = cover + shear_dia + main.dia / 2.0;
    if main.layers <= 1 {
        return k1;
    }
    let k_prime = 25.0_f64.max(1.5 * main.dia);
    let s = main.dia + k_prime;
    k1 + (main.layers as f64 - 1.0) / 2.0 * s
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
    fn test_pw_ratio_zero_pitch() {
        let shear = ShearBar {
            dia: 10.0,
            pitch: 0.0,
            legs: 2,
        };
        assert_eq!(pw_ratio(&shear, 400.0), 0.0);
    }
}
