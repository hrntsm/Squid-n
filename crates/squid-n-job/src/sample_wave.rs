//! 正弦減衰のサンプル地震波（外部波形ファイルなしで機能を試せる導線）。
//!
//! GUI（`squid-n-app`）と MCP サーバ（`squid-n-mcp`）が同じ式・同じ方向割り当てで
//! 波形を組み立てられるよう、本クレートに置く。

use crate::settings::{AnalysisSettings, ThDir};
use squid_n_solver::statics::analysis::SeismicDir;

/// 方向 `dir` に加速度列 `accel` を割り当てた `GroundMotion` を組み立てる。
/// X なら accel_x、Y なら accel_y に入れ、他方はゼロ列にする。
/// Xy（X+Y 同時入力）は同一波形を accel_x・accel_y の両方にそのまま入れる
/// 簡易仕様（位相差・別波形の指定はサポートしない。CSV 2 列入力は
/// 呼び出し側が別々の列を返すため、その場合は本関数を経由せず
/// 直接 `GroundMotion` を組み立てる）。
pub fn build_ground_motion(
    dt: f64,
    dir: ThDir,
    accel: Vec<f64>,
) -> squid_n_solver::dynamic::timehistory::GroundMotion {
    match dir {
        ThDir::X => squid_n_solver::dynamic::timehistory::GroundMotion {
            dt,
            accel_x: accel,
            accel_y: None,
            accel_theta: None,
        },
        ThDir::Y => {
            let n = accel.len();
            squid_n_solver::dynamic::timehistory::GroundMotion {
                dt,
                accel_x: vec![0.0; n],
                accel_y: Some(accel),
                accel_theta: None,
            }
        }
        ThDir::Xy => squid_n_solver::dynamic::timehistory::GroundMotion {
            dt,
            accel_x: accel.clone(),
            accel_y: Some(accel),
            accel_theta: None,
        },
    }
}

/// 正弦減衰のサンプル地震波を `cfg` から組み立てる。
pub fn sample_ground_motion(
    cfg: &AnalysisSettings,
) -> squid_n_solver::dynamic::timehistory::GroundMotion {
    let n = ((cfg.th_duration / cfg.th_dt).ceil() as usize).max(2);
    let omega = 2.0 * std::f64::consts::PI / cfg.th_period.max(1e-6);
    let accel: Vec<f64> = (0..n)
        .map(|i| {
            let t = i as f64 * cfg.th_dt;
            cfg.th_amp * (omega * t).sin() * (-0.3 * t).exp()
        })
        .collect();
    build_ground_motion(cfg.th_dt, cfg.th_dir, accel)
}

/// 質点系時刻歴用の正弦減衰サンプル波（立体時刻歴の dt/継続/周期/振幅とは独立）。
pub fn sample_lumped_ground_motion(
    cfg: &AnalysisSettings,
) -> squid_n_solver::dynamic::timehistory::GroundMotion {
    let n = ((cfg.lumped_th_duration / cfg.lumped_th_dt).ceil() as usize).max(2);
    let omega = 2.0 * std::f64::consts::PI / cfg.lumped_th_period.max(1e-6);
    let accel: Vec<f64> = (0..n)
        .map(|i| {
            let t = i as f64 * cfg.lumped_th_dt;
            cfg.lumped_th_amp * (omega * t).sin() * (-0.3 * t).exp()
        })
        .collect();
    let dir = match cfg.lumped_dir {
        SeismicDir::X => ThDir::X,
        SeismicDir::Y => ThDir::Y,
    };
    build_ground_motion(cfg.lumped_th_dt, dir, accel)
}

/// 質点系時刻歴に渡す地動加速度列（加振方向の成分）。
/// Y 加振で `accel_y` が無い波形は、X 列へ黙って落とさずエラーにする。
pub fn lumped_accel_from_wave(
    wave: &squid_n_solver::dynamic::timehistory::GroundMotion,
    dir: SeismicDir,
) -> Result<Vec<f64>, String> {
    match dir {
        SeismicDir::X => Ok(wave.accel_x.clone()),
        SeismicDir::Y => wave.accel_y.clone().ok_or_else(|| {
            "Y 方向の加速度列がありません。Y 加振用の波形を選ぶか、加振方向を X にしてください。"
                .into()
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::settings::AnalysisSettings;
    use squid_n_solver::statics::analysis::SeismicDir;

    #[test]
    fn build_ground_motion_routes_by_direction() {
        let accel = vec![1.0, 2.0, 3.0];
        let wave_x = build_ground_motion(0.01, ThDir::X, accel.clone());
        assert_eq!(wave_x.accel_x, accel);
        assert!(wave_x.accel_y.is_none());

        let wave_y = build_ground_motion(0.01, ThDir::Y, accel.clone());
        assert_eq!(wave_y.accel_x, vec![0.0; 3]);
        assert_eq!(wave_y.accel_y.as_deref(), Some(accel.as_slice()));
    }

    #[test]
    fn build_ground_motion_xy_duplicates_wave() {
        let accel = vec![1.0, 2.0];
        let wave = build_ground_motion(0.01, ThDir::Xy, accel.clone());
        assert_eq!(wave.accel_x, accel);
        assert_eq!(wave.accel_y.as_deref(), Some(accel.as_slice()));
    }

    #[test]
    fn sample_ground_motion_uses_cfg() {
        let cfg = AnalysisSettings {
            th_dt: 0.02,
            th_duration: 0.04,
            th_period: 0.5,
            th_amp: 10.0,
            th_dir: ThDir::Y,
            ..Default::default()
        };
        let wave = sample_ground_motion(&cfg);
        assert_eq!(wave.dt, 0.02);
        assert_eq!(wave.accel_x, vec![0.0; 2]);
        assert_eq!(wave.accel_y.as_ref().map(|v| v.len()), Some(2));
    }

    #[test]
    fn lumped_accel_from_wave_rejects_missing_y() {
        let wave = build_ground_motion(0.01, ThDir::X, vec![1.0, 2.0]);
        let err = lumped_accel_from_wave(&wave, SeismicDir::Y).unwrap_err();
        assert!(err.contains("Y 方向"), "{err}");
        assert_eq!(
            lumped_accel_from_wave(&wave, SeismicDir::X).unwrap(),
            vec![1.0, 2.0]
        );
    }
}
