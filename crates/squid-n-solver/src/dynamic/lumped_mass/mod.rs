//! 質点系（串団子）モデルの生成（せん断型多質点系、構造力学）。
//!
//! 立体フレームのプッシュオーバー結果から層ごとの Q-δ 関係を等包絡面積則でトリリニア骨格へ縮約する。

mod eigen;
mod model;
mod spatial;
mod time_history;

pub use eigen::{lumped_mass_eigen, LumpedMassModal};
pub use model::{
    build_lumped_mass_model, fit_story_trilinear, LumpedMassModel, LumpedMassType,
    LumpedStiffnessSource, StickDim, StorySpatial, StoryStick, StoryTrilinear,
};
pub use time_history::{lumped_mass_time_history, StickDirPeaks, StickResponse, STICK_NEWTON};

/// 質点系解析の保存用結果（モデル・固有値・時刻歴）。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct LumpedMassResult {
    pub model: LumpedMassModel,
    pub modal: LumpedMassModal,
    #[serde(default)]
    pub response: Option<StickResponse>,
}

#[cfg(test)]
use eigen::stick_omega1;
#[cfg(test)]
use model::envelope_area;
#[cfg(test)]
use squid_n_core::ids::StoryId;
#[cfg(test)]
use time_history::{fundamental_omega, solve_tridiagonal};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::statics::analysis::SeismicDir;

    #[test]
    fn test_fit_trilinear_equal_area_and_endpoints() {
        // 実曲線: 折れ点のあるなめらかな軟化曲線を細かくサンプル。
        // 0→(1,100) K1=100、(1,100)→(3,140) K2=20、(3,140)→(6,155) K3=5。
        let mut curve = Vec::new();
        for step in 1..=60 {
            let d = step as f64 * 0.1;
            let q = if d <= 1.0 {
                100.0 * d
            } else if d <= 3.0 {
                100.0 + 20.0 * (d - 1.0)
            } else {
                140.0 + 5.0 * (d - 3.0)
            };
            curve.push((d, q));
        }
        let tri = fit_story_trilinear(&curve, 0.9);
        // K1 = 初期剛性 100。
        assert!((tri.k1 - 100.0).abs() < 1.0, "k1={}", tri.k1);
        // 終端 (6, 155)。
        assert!((tri.d3 - 6.0).abs() < 1e-6 && (tri.q3 - 155.0).abs() < 1e-6);
        // 折点は昇順・耐力単調増加。
        assert!(tri.d1 < tri.d2 && tri.d2 <= tri.d3);
        assert!(tri.q1 <= tri.q2 + 1e-9 && tri.q2 <= tri.q3 + 1e-9);
        // 等包絡面積: トリリニアの面積 = 実曲線の面積。
        let a_actual = envelope_area(&curve);
        let a_tri = 0.5 * tri.d1 * tri.q1
            + 0.5 * (tri.q1 + tri.q2) * (tri.d2 - tri.d1)
            + 0.5 * (tri.q2 + tri.q3) * (tri.d3 - tri.d2);
        assert!(
            (a_tri - a_actual).abs() < 1e-3 * a_actual,
            "equal-area: a_tri={a_tri}, a_actual={a_actual}"
        );
    }

    #[test]
    fn test_fit_trilinear_k2_k3_helpers() {
        // 3勾配（K1=80 > K2=30 > K3=8）の軟化曲線。
        let curve: Vec<(f64, f64)> = (1..=50)
            .map(|s| {
                let d = s as f64 * 0.1;
                let q = if d <= 1.0 {
                    80.0 * d
                } else if d <= 2.5 {
                    80.0 + 30.0 * (d - 1.0)
                } else {
                    125.0 + 8.0 * (d - 2.5)
                };
                (d, q)
            })
            .collect();
        let tri = fit_story_trilinear(&curve, 0.9);
        assert!(
            tri.d1 < tri.d2 && tri.d2 < tri.d3,
            "distinct folds: {tri:?}"
        );
        assert!(
            tri.k1 >= tri.k2() && tri.k2() >= tri.k3() - 1e-6,
            "K1>=K2>=K3: k1={}, k2={}, k3={}",
            tri.k1,
            tri.k2(),
            tri.k3()
        );
        assert!(tri.k3() >= 0.0 && tri.k3() <= tri.k1);
    }

    #[test]
    fn test_fit_trilinear_bilinear_input_reduces_gracefully() {
        // バイリニア入力（K1=50→K=5）はトリリニアが縮退（d1≈d2）しても panic せず妥当。
        let curve: Vec<(f64, f64)> = (1..=30)
            .map(|s| {
                let d = s as f64 * 0.1;
                (d, 50.0 * d.min(2.0) + 5.0 * (d - 2.0).max(0.0))
            })
            .collect();
        let tri = fit_story_trilinear(&curve, 0.9);
        assert!((tri.k1 - 50.0).abs() < 1.0);
        assert!(tri.d1 <= tri.d2 && tri.d2 <= tri.d3);
        assert!((tri.d3 - 3.0).abs() < 1e-6 && (tri.q3 - 105.0).abs() < 1e-6);
    }

    #[test]
    fn test_fit_trilinear_empty_and_degenerate() {
        let tri = fit_story_trilinear(&[], 0.75);
        assert_eq!(tri.k1, 0.0);
        // 1点のみ（弾性）。
        let tri1 = fit_story_trilinear(&[(2.0, 200.0)], 0.75);
        assert!((tri1.k1 - 100.0).abs() < 1e-9);
    }

    fn stick(mass: f64, k1: f64, d1: f64, d2: f64, q2: f64, d3: f64, q3: f64) -> StoryStick {
        StoryStick {
            story: StoryId(0),
            mass,
            height: 3000.0,
            skeleton: StoryTrilinear {
                k1,
                d1,
                q1: k1 * d1,
                d2,
                q2,
                d3,
                q3,
            },
        }
    }

    #[test]
    fn test_solve_tridiagonal_identity() {
        // 単位行列: x=b。
        let x = solve_tridiagonal(
            &[0.0, 0.0, 0.0],
            &[1.0, 1.0, 1.0],
            &[0.0, 0.0, 0.0],
            &[3.0, 5.0, 7.0],
        );
        assert!(
            (x[0] - 3.0).abs() < 1e-12 && (x[1] - 5.0).abs() < 1e-12 && (x[2] - 7.0).abs() < 1e-12
        );
    }

    /// `build_lumped_mass_model` は、長期荷重のみを載荷した初期点の (δ,Q) を層 Q-δ 曲線の原点として差し引くこと。
    #[test]
    fn test_build_lumped_mass_model_corrects_long_term_origin() {
        use squid_n_core::model::{Model, Story};

        let model = Model {
            stories: vec![
                Story {
                    id: StoryId(0),
                    name: "1F".to_string(),
                    elevation: 0.0,
                    node_ids: Vec::new(),
                    seismic_weight: None,
                    weight_override: None,
                    level_kind: Default::default(),
                    structure: Default::default(),
                },
                Story {
                    id: StoryId(1),
                    name: "2F".to_string(),
                    elevation: 3000.0,
                    node_ids: Vec::new(),
                    seismic_weight: Some(1.0e6),
                    weight_override: None,
                    level_kind: Default::default(),
                    structure: Default::default(),
                },
            ],
            ..Default::default()
        };

        // 層0の Q-δ 曲線: λ=0（長期のみ、水平力ゼロ）で残留変形 -0.2mm。以降の
        // 各点は「残留変形 -0.2mm + K=100000 N/mm の弾性直線」の累積値
        // （`total_disp` が長期・水平力増分の両フェーズを通じて累積するのと同じ
        // 構成）とし、原点補正後に純粋な弾性直線（K=100000）へ戻ることを見る。
        let capacity_curve = vec![
            crate::nonlinear::pushover::CapacityPoint {
                step: 0,
                roof_disp: -0.1,
                base_shear: 0.0,
                story_shear: vec![1e-11],
                story_drift: vec![-0.2],
            },
            crate::nonlinear::pushover::CapacityPoint {
                step: 1,
                roof_disp: 5.0,
                base_shear: 500_000.0,
                story_shear: vec![500_000.0],
                story_drift: vec![-0.2 + 5.0],
            },
            crate::nonlinear::pushover::CapacityPoint {
                step: 2,
                roof_disp: 10.0,
                base_shear: 1_000_000.0,
                story_shear: vec![1_000_000.0],
                story_drift: vec![-0.2 + 10.0],
            },
        ];
        let steps = vec![
            crate::nonlinear::pushover::PushoverStep {
                load_factor: 0.0,
                top_disp: -0.1,
                base_shear: 0.0,
                story_drifts: vec![-0.2],
            },
            crate::nonlinear::pushover::PushoverStep {
                load_factor: 0.5,
                top_disp: 5.0,
                base_shear: 500_000.0,
                story_drifts: vec![-0.2 + 5.0],
            },
            crate::nonlinear::pushover::PushoverStep {
                load_factor: 1.0,
                top_disp: 10.0,
                base_shear: 1_000_000.0,
                story_drifts: vec![-0.2 + 10.0],
            },
        ];
        let pushover = crate::nonlinear::pushover::PushoverResult {
            steps,
            capacity_curve,
            hinges: Vec::new(),
            shear_yields: Vec::new(),
            mechanism: crate::nonlinear::pushover::MechanismType::Overall,
            qu: 1_000_000.0,
            member_response: Vec::new(),
            control: crate::nonlinear::pushover::PushoverControl::default(),
            member_history: Vec::new(),
            fiber_states: Vec::new(),
            termination: crate::nonlinear::pushover::PushoverTermination::TargetReached,
        };

        let lm = build_lumped_mass_model(&model, &pushover, LumpedMassType::EquivalentShear, 0.75);
        assert_eq!(lm.stories.len(), 1);
        let k1 = lm.stories[0].skeleton.k1;
        assert!(
            (k1 - 100_000.0).abs() < 1.0,
            "長期のみ載荷点の残留変形を原点補正した純粋な弾性剛性になっていること: k1={k1}"
        );
    }

    #[test]
    fn test_fundamental_omega_sdof() {
        // 1 質点: ω1=√(k/m)。
        let w = fundamental_omega(&[2.0], &[800.0]);
        assert!((w - (800.0_f64 / 2.0).sqrt()).abs() < 1e-6, "w={w}");
    }

    /// `lumped_mass_eigen` の解析解検証（構造力学）: 2 質点・等質量 m・等剛性 k の
    /// せん断型 [[2k,-k],[-k,k]] は特性方程式 λ²-3λ+1=0（λ=ω²/(k/m)）より
    /// λ=(3∓√5)/2 = 0.381966… / 2.618034…。faer の固有値が昇順であることと
    /// 数値の妥当性の両方を同時に検証する。
    #[test]
    fn test_lumped_mass_eigen_two_dof_analytic() {
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            dim: StickDim::Planar,
            stiffness_source: LumpedStiffnessSource::StoryQd,
            dir: SeismicDir::X,
            nonlinear: true,
            spatial: Vec::new(),
            stories: vec![
                stick(1.0, 1.0, 0.1, 0.3, 0.7, 1.0, 0.8),
                stick(1.0, 1.0, 0.1, 0.3, 0.7, 1.0, 0.8),
            ],
        };
        let modal = lumped_mass_eigen(&lm, 2).expect("固有値分解に成功する");
        assert_eq!(modal.omega2.len(), 2);
        let expected = [(3.0 - 5.0_f64.sqrt()) / 2.0, (3.0 + 5.0_f64.sqrt()) / 2.0];
        assert!(
            (modal.omega2[0] - expected[0]).abs() < 1e-9,
            "1次: {} vs {}",
            modal.omega2[0],
            expected[0]
        );
        assert!(
            (modal.omega2[1] - expected[1]).abs() < 1e-9,
            "2次: {} vs {}",
            modal.omega2[1],
            expected[1]
        );
        // 昇順（1次 < 2次）。
        assert!(modal.omega2[0] < modal.omega2[1]);
        // 周期は ω=√ω² から算定される（T=2π/ω）。
        assert!(
            (modal.period[0] - 2.0 * std::f64::consts::PI / modal.omega2[0].sqrt()).abs() < 1e-9
        );
    }

    /// `lumped_mass_eigen` の1次モードと逆反復法 `fundamental_omega` が同じ入力で近い値を返すこと。
    #[test]
    fn test_lumped_mass_eigen_matches_power_iteration_omega1() {
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            dim: StickDim::Planar,
            stiffness_source: LumpedStiffnessSource::StoryQd,
            dir: SeismicDir::X,
            nonlinear: true,
            spatial: Vec::new(),
            stories: vec![
                stick(2.0, 2000.0, 0.1, 0.3, 250.0, 1.0, 300.0),
                stick(1.5, 1500.0, 0.1, 0.3, 200.0, 1.0, 260.0),
                stick(1.0, 900.0, 0.1, 0.3, 150.0, 1.0, 200.0),
            ],
        };
        let modal = lumped_mass_eigen(&lm, 1).expect("固有値分解に成功する");
        let mass: Vec<f64> = lm.stories.iter().map(|s| s.mass).collect();
        let k1: Vec<f64> = lm.stories.iter().map(|s| s.skeleton.k1).collect();
        let w_power = fundamental_omega(&mass, &k1);
        let w_eigen = modal.omega2[0].sqrt();
        assert!(
            (w_eigen - w_power).abs() < 1e-6 * w_power,
            "eigen={w_eigen}, power_iteration={w_power}"
        );
    }

    /// 質量が 0 以下の層があると `SolveError::InvalidInput` を返す（実体のない
    /// 極端な高周波モードを紛れ込ませず、明示的にエラー報告する）。
    #[test]
    fn test_lumped_mass_eigen_rejects_non_positive_mass() {
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            dim: StickDim::Planar,
            stiffness_source: LumpedStiffnessSource::StoryQd,
            dir: SeismicDir::X,
            nonlinear: true,
            spatial: Vec::new(),
            stories: vec![stick(0.0, 1000.0, 0.1, 0.3, 250.0, 1.0, 300.0)],
        };
        let err = lumped_mass_eigen(&lm, 1).unwrap_err();
        assert!(matches!(
            err,
            squid_n_math::solver::SolveError::InvalidInput(_)
        ));
    }

    /// `stick_omega1` は、質量 0 以下の層があっても ω1 を桁違いに大きくしないこと。
    #[test]
    fn test_stick_omega1_survives_zero_mass_story_without_blowing_up() {
        let healthy = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            dim: StickDim::Planar,
            stiffness_source: LumpedStiffnessSource::StoryQd,
            dir: SeismicDir::X,
            nonlinear: true,
            spatial: Vec::new(),
            stories: vec![
                stick(2.0, 2000.0, 0.1, 0.3, 250.0, 1.0, 300.0),
                stick(1.5, 1500.0, 0.1, 0.3, 200.0, 1.0, 260.0),
            ],
        };
        let w_healthy = stick_omega1(&healthy);

        let with_zero_mass_story = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            dim: StickDim::Planar,
            stiffness_source: LumpedStiffnessSource::StoryQd,
            dir: SeismicDir::X,
            nonlinear: true,
            spatial: Vec::new(),
            stories: vec![
                stick(2.0, 2000.0, 0.1, 0.3, 250.0, 1.0, 300.0),
                stick(0.0, 1500.0, 0.1, 0.3, 200.0, 1.0, 260.0),
            ],
        };
        let w_zero_mass = stick_omega1(&with_zero_mass_story);

        assert!(w_zero_mass.is_finite() && w_zero_mass > 0.0);
        // 健全なケースと同程度のオーダー（1桁以内）に収まること。
        assert!(
            w_zero_mass < w_healthy * 10.0,
            "ω1 が異常に大きい（無減衰化の兆候）: zero_mass={w_zero_mass}, healthy={w_healthy}"
        );
    }

    /// `stick_omega1` は、層剛性 K1 が 0 の層があっても ω1 を 0 に潰さないこと。
    #[test]
    fn test_stick_omega1_survives_zero_stiffness_story_without_blowing_up() {
        let healthy = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            dim: StickDim::Planar,
            stiffness_source: LumpedStiffnessSource::StoryQd,
            dir: SeismicDir::X,
            nonlinear: true,
            spatial: Vec::new(),
            stories: vec![
                stick(2.0, 2000.0, 0.1, 0.3, 250.0, 1.0, 300.0),
                stick(1.5, 1500.0, 0.1, 0.3, 200.0, 1.0, 260.0),
            ],
        };
        let w_healthy = stick_omega1(&healthy);

        let with_zero_stiffness_story = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            dim: StickDim::Planar,
            stiffness_source: LumpedStiffnessSource::StoryQd,
            dir: SeismicDir::X,
            nonlinear: true,
            spatial: Vec::new(),
            stories: vec![
                stick(2.0, 0.0, 0.1, 0.3, 250.0, 1.0, 300.0),
                stick(1.5, 1500.0, 0.1, 0.3, 200.0, 1.0, 260.0),
            ],
        };
        let w_zero_k1 = stick_omega1(&with_zero_stiffness_story);

        assert!(
            w_zero_k1.is_finite() && w_zero_k1 > 0.0,
            "K1=0 の層があると ω1 が 0 に潰れ、a1=2h/ω1 が無条件にゼロになる: w_zero_k1={w_zero_k1}"
        );
        assert!(
            w_zero_k1 < w_healthy * 10.0,
            "ω1 が異常に大きい（クランプ後の逆反復法へのフォールバックが桁違いになっていないか）: \
             zero_k1={w_zero_k1}, healthy={w_healthy}"
        );
    }

    /// n_modes が層数を超える場合、返るモード数は層数まで切り詰められる
    /// （立体モデルの `solve_eigen` と同じ規約）。
    #[test]
    fn test_lumped_mass_eigen_truncates_to_story_count() {
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            dim: StickDim::Planar,
            stiffness_source: LumpedStiffnessSource::StoryQd,
            dir: SeismicDir::X,
            nonlinear: true,
            spatial: Vec::new(),
            stories: vec![stick(1.0, 1000.0, 0.1, 0.3, 250.0, 1.0, 300.0)],
        };
        let modal = lumped_mass_eigen(&lm, 5).expect("固有値分解に成功する");
        assert_eq!(modal.omega2.len(), 1);
        assert_eq!(modal.shapes.len(), 1);
        // 単一質点のモード形状は最上階=1.0 に正規化される。
        assert!((modal.shapes[0][0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_stick_th_zero_input_zero_response() {
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            dim: StickDim::Planar,
            stiffness_source: LumpedStiffnessSource::StoryQd,
            dir: SeismicDir::X,
            nonlinear: true,
            spatial: Vec::new(),
            stories: vec![stick(1.0, 1000.0, 0.1, 0.3, 140.0, 1.0, 160.0)],
        };
        let res = lumped_mass_time_history(&lm, &vec![0.0; 200], 0.01, 0.02);
        assert!(res.roof_disp.iter().all(|&v| v.abs() < 1e-9));
        assert_eq!(res.story_ductility[0], 0.0);
    }

    #[test]
    fn test_stick_th_responds_and_bounded() {
        // 正弦地動で応答が非ゼロかつ有限。
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            dim: StickDim::Planar,
            stiffness_source: LumpedStiffnessSource::StoryQd,
            dir: SeismicDir::X,
            nonlinear: true,
            spatial: Vec::new(),
            stories: vec![
                stick(1.0, 2000.0, 0.1, 0.3, 250.0, 1.0, 300.0),
                stick(1.0, 1500.0, 0.1, 0.3, 200.0, 1.0, 260.0),
            ],
        };
        let dt = 0.01;
        let accel: Vec<f64> = (0..300)
            .map(|i| 2000.0 * (2.0 * std::f64::consts::PI * 1.5 * i as f64 * dt).sin())
            .collect();
        let res = lumped_mass_time_history(&lm, &accel, dt, 0.03);
        assert_eq!(res.time.len(), 300);
        assert!(res.roof_disp.iter().all(|v| v.is_finite()));
        assert!(
            res.roof_disp.iter().any(|&v| v.abs() > 1e-3),
            "should show nonzero roof response"
        );
        assert_eq!(res.story_peak_drift.len(), 2);
        assert_eq!(res.drift_dir.x.len(), 2);
        assert!(
            (res.drift_dir.x[0] - res.story_peak_drift[0]).abs() < 1e-12
                && res.drift_dir.y[0].abs() < 1e-12,
            "2 次元 X 加振の層間は X=最大・Y=0"
        );
        let s2 = std::f64::consts::FRAC_1_SQRT_2;
        assert!((res.drift_dir.deg45[0] - res.story_peak_drift[0] * s2).abs() < 1e-9);
    }

    #[test]
    fn test_stick_th_yields_under_strong_input() {
        // 強い地動で層が降伏（塑性率 μ>1）。
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            dim: StickDim::Planar,
            stiffness_source: LumpedStiffnessSource::StoryQd,
            dir: SeismicDir::X,
            nonlinear: true,
            spatial: Vec::new(),
            stories: vec![stick(2.0, 1000.0, 0.5, 2.0, 700.0, 8.0, 800.0)],
        };
        let dt = 0.01;
        // 一定方向の強い引き込みで大変形。
        let accel: Vec<f64> = (0..400)
            .map(|i| {
                let t = i as f64 * dt;
                3000.0 * (2.0 * std::f64::consts::PI * 0.8 * t).sin()
            })
            .collect();
        let res = lumped_mass_time_history(&lm, &accel, dt, 0.02);
        assert!(res.roof_disp.iter().all(|v| v.is_finite()));
        assert!(
            res.story_ductility[0] > 1.0,
            "strong input should yield the story: μ={}",
            res.story_ductility[0]
        );
    }
}
