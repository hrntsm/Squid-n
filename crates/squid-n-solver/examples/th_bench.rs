//! 時刻歴応答解析のベンチマーク（最適化前ベースライン計測用）。
//!
//! 5層×3スパン×3スパンのS造立体フレーム（柱・梁、階設定、節点質量、
//! 長期荷重ケース）を solver 単体で生成し、以下 3 ケースの所要時間を計測する。
//!
//! - (a) 線形時刻歴（Newmark-β 平均加速度法）
//! - (b) 線形時刻歴（HHT-α 法）
//! - (c) 非線形時刻歴（`NonlinearThCfg` 既定、集中ばね系・`ForceRegime::Auto`）
//!
//! 各ケースは 3 回実行して最小値を採用する（OS ノイズの影響を減らすため）。
//! さらに、詳細記録（[`squid_n_solver::timehistory::ThRecording`]）を
//! 間引く `record_every` を極端に大きくした場合との差分から、記録コストの
//! 概算を分離する（record_every=None は自動間引き〜1000フレーム、
//! 大きな値を渡すと最終フレームのみが記録される＝記録をほぼ無効化した近似）。
//!
//! ```bash
//! cargo run -p squid-n-solver --example th_bench --release
//! ```

use std::time::Instant;

use squid_n_core::dof::{Dof6Mask, DofMap};
use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LoadCaseKind, LocalAxis,
    Material, MaterialCategory, MemberLoad, MemberLoadKind, Model, Node, Section, Story,
};
use squid_n_math::parallelism::{set_parallelism, Parallelism};
use squid_n_math::solver::SolveError;
use squid_n_solver::constraint::Reducer;
use squid_n_solver::damping::{Damping, DampingAccumulation, StiffnessKind};
use squid_n_solver::eigen::solve_eigen;
use squid_n_solver::timehistory::{
    linear_hht_alpha_analysis, linear_time_history_analysis, nonlinear_time_history_analysis,
    GroundMotion, HhtCfg, NewmarkCfg, NonlinearThCfg, ResponseResult,
};

/// 記録をほぼ無効化した近似として使う `record_every`。ステップ数を大きく
/// 上回る値を渡すと、フレーム記録は最終ステップのみになる
/// （`ThRecorder::record_step` は `step % record_every == 0 || step == n_steps` の
/// ときだけフレームを積むため）。API に「記録無効化」の専用フラグは無いため、
/// この近似を用いる。
const RECORD_EVERY_DISABLED: usize = 1_000_000_000;

/// nx×ny スパン・nz 層の S造立体ラーメン（柱・X/Y 大梁）を生成する。
/// 各階の節点に水平・鉛直質量を配置し、大梁に長期荷重ケース（固定荷重、
/// 等分布荷重）を載荷する。階設定（[`Story`]）も持たせ、層間変形角の
/// 集計や非線形解析の長期荷重初期化（`NonlinearThCfg::apply_long_term`）を
/// 実運用に近い形で通す。
fn make_frame(nx: usize, ny: usize, nz: usize) -> Model {
    let span = 6000.0; // [mm]
    let height = 3500.0; // [mm]
                         // 1節点あたりの水平・鉛直質量 [N・s²/mm]（床の分担重量を想定した概算値）。
    let node_mass = 800.0;
    // 回転自由度のダミー質量。非線形時刻歴（集中ばね系の梁要素）は回転自由度に
    // 慣性を持たないため、並進のみに質量を与えると質量行列が回転DOFで特異になり
    // 初期加速度が求まらない（solver からの案内メッセージのとおり「全自由度に
    // 質量を与える」ことで回避する）。応答への影響が無視できる程度の小さな値。
    let node_mass_rot = node_mass * 1.0e-3;

    let node_id = |ix: usize, iy: usize, iz: usize| -> NodeId {
        NodeId((iz * (nx + 1) * (ny + 1) + iy * (nx + 1) + ix) as u32)
    };

    let mut nodes = Vec::new();
    for iz in 0..=nz {
        for iy in 0..=ny {
            for ix in 0..=nx {
                let is_base = iz == 0;
                nodes.push(Node {
                    id: node_id(ix, iy, iz),
                    coord: [ix as f64 * span, iy as f64 * span, iz as f64 * height],
                    restraint: if is_base {
                        Dof6Mask::FIXED
                    } else {
                        Dof6Mask::FREE
                    },
                    mass: if is_base {
                        None
                    } else {
                        Some([
                            node_mass,
                            node_mass,
                            node_mass,
                            node_mass_rot,
                            node_mass_rot,
                            node_mass_rot,
                        ])
                    },
                    story: if is_base {
                        None
                    } else {
                        Some(StoryId((iz - 1) as u32))
                    },
                    support_spring: None,
                });
            }
        }
    }

    let mut elements = Vec::new();
    let mut beam_ids_for_load = Vec::new();
    let mut push_beam =
        |n0: NodeId, n1: NodeId, ref_vector: [f64; 3], section: SectionId| -> ElemId {
            let id = ElemId(elements.len() as u32);
            elements.push(ElementData {
                id,
                kind: ElementKind::Beam,
                nodes: smallvec::smallvec![n0, n1],
                section: Some(section),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis { ref_vector },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            });
            id
        };

    // 柱（全層・全通り、断面 SectionId(0)）
    for iz in 0..nz {
        for iy in 0..=ny {
            for ix in 0..=nx {
                push_beam(
                    node_id(ix, iy, iz),
                    node_id(ix, iy, iz + 1),
                    [1.0, 0.0, 0.0],
                    SectionId(0),
                );
            }
        }
    }
    // 大梁（各階、X/Y 両方向、断面 SectionId(1)）。長期荷重を載荷する対象として ID を控える。
    for iz in 1..=nz {
        for iy in 0..=ny {
            for ix in 0..nx {
                let id = push_beam(
                    node_id(ix, iy, iz),
                    node_id(ix + 1, iy, iz),
                    [0.0, 0.0, 1.0],
                    SectionId(1),
                );
                beam_ids_for_load.push(id);
            }
        }
        for iy in 0..ny {
            for ix in 0..=nx {
                let id = push_beam(
                    node_id(ix, iy, iz),
                    node_id(ix, iy + 1, iz),
                    [0.0, 0.0, 1.0],
                    SectionId(1),
                );
                beam_ids_for_load.push(id);
            }
        }
    }

    // 長期荷重ケース（固定荷重）: 全大梁に等分布荷重 w=10 N/mm（鉛直下向き）。
    // 非線形時刻歴（`NonlinearThCfg::apply_long_term`）の初期載荷に使う。
    let member: Vec<MemberLoad> = beam_ids_for_load
        .iter()
        .map(|&elem| MemberLoad {
            elem,
            dir: [0.0, 0.0, -1.0],
            kind: MemberLoadKind::Distributed {
                a: 0.0,
                b: span,
                w1: 10.0,
                w2: 10.0,
            },
        })
        .collect();
    let load_cases = vec![LoadCase {
        kind: LoadCaseKind::Dead,
        id: LoadCaseId(0),
        name: "長期".into(),
        nodal: Vec::new(),
        member,
    }];

    // 階設定（層間変形角の集計に使用。剛床拘束は付けず、節点は独立自由度のまま）。
    let mut stories = Vec::new();
    for iz in 1..=nz {
        let node_ids: Vec<NodeId> = (0..=ny)
            .flat_map(|iy| (0..=nx).map(move |ix| node_id(ix, iy, iz)))
            .collect();
        stories.push(Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId((iz - 1) as u32),
            name: format!("{iz}F"),
            elevation: iz as f64 * height,
            node_ids,
            diaphragms: vec![],
            seismic_weight: None,
            weight_override: None,
        });
    }

    Model {
        nodes,
        elements,
        sections: vec![
            Section {
                id: SectionId(0),
                name: "柱 H-400x400x13x21".into(),
                area: 21_870.0,
                iy: 6.6e8,
                iz: 6.6e8,
                j: 2.0e7,
                depth: 400.0,
                width: 400.0,
                as_y: 12_000.0,
                as_z: 12_000.0,
                panel_thickness: None,
                thickness: None,
                shape: None,
            },
            Section {
                id: SectionId(1),
                name: "梁 H-400x200x8x13".into(),
                area: 8_412.0,
                iy: 2.34e8,
                iz: 2.34e8,
                j: 6.0e5,
                depth: 400.0,
                width: 200.0,
                as_y: 4_000.0,
                as_z: 4_000.0,
                panel_thickness: None,
                thickness: None,
                shape: None,
            },
        ],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SN400".into(),
            category: MaterialCategory::Steel,
            young: 205_000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(235.0),
        }],
        load_cases,
        stories,
        ..Default::default()
    }
}

/// 正弦減衰波（模擬地震波）: `a(t) = amp・exp(-decay・t)・sin(omega・t)`。
/// `dt`・`n_steps` は呼び出し側で指定する。
fn sine_decay_wave(dt: f64, n_steps: usize, amp: f64, omega: f64, decay: f64) -> GroundMotion {
    let accel_x: Vec<f64> = (0..n_steps)
        .map(|i| {
            let t = i as f64 * dt;
            amp * (-decay * t).exp() * (omega * t).sin()
        })
        .collect();
    GroundMotion {
        dt,
        accel_x,
        accel_y: None,
        accel_theta: None,
    }
}

/// 結果の妥当性チェック: ピーク変位が有限かつ非ゼロで、非現実的な発散値でないこと。
fn validate(label: &str, result: &ResponseResult) {
    let peak = result
        .peak_disp
        .iter()
        .flat_map(|p| p.iter())
        .cloned()
        .fold(0.0_f64, f64::max);
    let all_finite = result
        .peak_disp
        .iter()
        .flat_map(|p| p.iter())
        .all(|v| v.is_finite());
    println!("    [検証] ピーク変位(全節点最大) = {peak:.4} mm, 有限値={all_finite}");
    assert!(
        all_finite,
        "{label}: 応答が非有限値（発散）です: peak={peak}"
    );
    assert!(
        peak > 1e-9 && peak < 1.0e6,
        "{label}: ピーク変位が異常です（0またはスケール逸脱）: peak={peak}"
    );
}

/// ピーク変位（全節点最大、`validate` と同じ算定式）。並列/逐次ケース間の
/// ビット一致検証（(c2) vs (c)）に使う。
fn peak_disp_max(result: &ResponseResult) -> f64 {
    result
        .peak_disp
        .iter()
        .flat_map(|p| p.iter())
        .cloned()
        .fold(0.0_f64, f64::max)
}

/// `f` を 3 回実行し最小の所要時間を採用する。最終回の結果で妥当性チェックを行い、
/// 所要時間と最終回の結果を返す（結果は並列/逐次ケース間のビット一致検証に使う）。
fn bench_min3<F>(label: &str, steps: usize, ndof: usize, mut f: F) -> (f64, ResponseResult)
where
    F: FnMut() -> Result<ResponseResult, SolveError>,
{
    let mut best = f64::INFINITY;
    let mut last: Option<ResponseResult> = None;
    for _ in 0..3 {
        let t0 = Instant::now();
        let r = f().unwrap_or_else(|e| panic!("{label}: 解析に失敗しました: {e}"));
        let elapsed = t0.elapsed().as_secs_f64();
        best = best.min(elapsed);
        last = Some(r);
    }
    println!("{label}: {best:.3} s (steps={steps}, ndof={ndof})");
    let result = last.expect("3回実行したため必ず Some");
    validate(label, &result);
    (best, result)
}

fn main() {
    // --- モデル生成: 5層×3スパン×3スパンのS造立体フレーム ---
    let (nx, ny, nz) = (3, 3, 5);
    let mut model = make_frame(nx, ny, nz);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    let ndof = reducer.n_indep;

    println!("=== 時刻歴応答解析ベンチマーク（最適化前ベースライン） ===");
    println!(
        "モデル: {nx}x{ny}スパン {nz}層  節点 {}  部材 {}  自由度数(縮約後) {ndof}",
        model.nodes.len(),
        model.elements.len(),
    );

    // --- 減衰・地震波: 1次固有振動数から Rayleigh 減衰・入力波の卓越振動数を決める ---
    let modal = solve_eigen(&model, &dofmap, &reducer, 1).expect("固有値解析に失敗");
    let omega1 = modal.omega2[0].sqrt();
    let period1 = 2.0 * std::f64::consts::PI / omega1;
    println!("1次固有周期: {period1:.4} s (ω1={omega1:.4} rad/s)");
    let damping = Damping::StiffnessProportional {
        h: 0.03,
        omega: omega1,
        basis: StiffnessKind::Initial,
    };

    let dt = 0.01;
    let n_steps_linear = 4096;
    // 非線形は Newton 反復（ステップごと最大 `NonlinearThCfg::newton.max_iter` 回、
    // 既定 20 回）を伴い、線形時刻歴（1ステップ1回の前進代入のみ）より大幅に
    // 計算コストが高い。重ければステップ数を落として計測する。
    let n_steps_nonlinear = 1024;

    // 入力波の振幅: 1次固有周期に近い周期で加振し、有意な応答を励起する。
    let wave_linear = sine_decay_wave(dt, n_steps_linear, 800.0, omega1, 0.3);
    let wave_nonlinear = sine_decay_wave(dt, n_steps_nonlinear, 3000.0, omega1, 0.5);

    println!();
    println!("--- ベースライン計測（各ケース3回実行、最小値を採用） ---");

    // (a) 線形時刻歴（Newmark-β 平均加速度法）
    let newmark = NewmarkCfg {
        beta: 0.25,
        gamma: 0.5,
        dt,
    };
    let (t_newmark, _) = bench_min3(
        "(a) 線形時刻歴 Newmark-β",
        n_steps_linear,
        ndof,
        || {
            linear_time_history_analysis(
                &model,
                &dofmap,
                &reducer,
                &wave_linear,
                &newmark,
                &damping,
                &[],
                &[],
                false,
                None,
            )
        },
    );

    // (b) 線形時刻歴（HHT-α 法）
    let hht = HhtCfg::new(dt);
    let (t_hht, _) = bench_min3("(b) 線形時刻歴 HHT-α", n_steps_linear, ndof, || {
        linear_hht_alpha_analysis(
            &model,
            &dofmap,
            &reducer,
            &wave_linear,
            &hht,
            &damping,
            &[],
            &[],
            false,
            None,
        )
    });

    // (c) 非線形時刻歴（NonlinearThCfg 既定、集中ばね系・ForceRegime::Auto）
    let (t_nonlinear, result_c) = bench_min3(
        "(c) 非線形時刻歴 既定Cfg",
        n_steps_nonlinear,
        ndof,
        || {
            nonlinear_time_history_analysis(
                &mut model,
                &dofmap,
                &reducer,
                &wave_nonlinear,
                &newmark,
                &damping,
                DampingAccumulation::NonCumulative,
                &[],
                &[],
                NonlinearThCfg::new(20, 1e-6),
            )
        },
    );

    // (c2) 非線形時刻歴（並列、Parallelism::Auto）。要素ループの並列化
    // （nonlinear.rs の Newton 反復・driver.rs・assembly.rs）が
    // Deterministic 経路とビット一致することを、逐次ケース (c) との
    // ピーク変位完全一致で検証する（並列化のビット一致検証を兼ねる）。
    set_parallelism(Parallelism::Auto);
    let (t_nonlinear_parallel, result_c2) = bench_min3(
        "(c2) 非線形時刻歴 並列(Auto)",
        n_steps_nonlinear,
        ndof,
        || {
            nonlinear_time_history_analysis(
                &mut model,
                &dofmap,
                &reducer,
                &wave_nonlinear,
                &newmark,
                &damping,
                DampingAccumulation::NonCumulative,
                &[],
                &[],
                NonlinearThCfg::new(20, 1e-6),
            )
        },
    );
    // 終了後は既定（Deterministic）へ戻す（以降のケースは従来どおり逐次で計測する）。
    set_parallelism(Parallelism::Deterministic);

    let peak_c = peak_disp_max(&result_c);
    let peak_c2 = peak_disp_max(&result_c2);
    assert_eq!(
        peak_c.to_bits(),
        peak_c2.to_bits(),
        "(c2) 並列のピーク変位が逐次(c)と完全一致しません: (c)={peak_c:.10} mm, (c2)={peak_c2:.10} mm"
    );
    println!(
        "  [検証] (c2) 並列のピーク変位が逐次(c)と完全一致: {peak_c2:.4} mm \
         （並列化のビット一致検証OK、逐次 {t_nonlinear:.3} s → 並列 {t_nonlinear_parallel:.3} s）"
    );

    // --- 記録コストの内訳（record_every を極端に大きくして詳細記録をほぼ無効化） ---
    println!();
    println!("--- 記録コストの内訳（record_every=大 との差分で概算） ---");

    let (t_newmark_norecord, _) = bench_min3(
        "(a') Newmark-β record_every=大",
        n_steps_linear,
        ndof,
        || {
            linear_time_history_analysis(
                &model,
                &dofmap,
                &reducer,
                &wave_linear,
                &newmark,
                &damping,
                &[],
                &[],
                false,
                Some(RECORD_EVERY_DISABLED),
            )
        },
    );
    println!(
        "  記録コスト概算(a): {:.3} s ({:.1}% of 総時間)",
        t_newmark - t_newmark_norecord,
        100.0 * (t_newmark - t_newmark_norecord) / t_newmark
    );

    let (t_hht_norecord, _) =
        bench_min3("(b') HHT-α record_every=大", n_steps_linear, ndof, || {
            linear_hht_alpha_analysis(
                &model,
                &dofmap,
                &reducer,
                &wave_linear,
                &hht,
                &damping,
                &[],
                &[],
                false,
                Some(RECORD_EVERY_DISABLED),
            )
        });
    println!(
        "  記録コスト概算(b): {:.3} s ({:.1}% of 総時間)",
        t_hht - t_hht_norecord,
        100.0 * (t_hht - t_hht_norecord) / t_hht
    );

    let mut cfg_norecord = NonlinearThCfg::new(20, 1e-6);
    cfg_norecord.record_every = Some(RECORD_EVERY_DISABLED);
    let (t_nonlinear_norecord, _) = bench_min3(
        "(c') 非線形 record_every=大",
        n_steps_nonlinear,
        ndof,
        || {
            nonlinear_time_history_analysis(
                &mut model,
                &dofmap,
                &reducer,
                &wave_nonlinear,
                &newmark,
                &damping,
                DampingAccumulation::NonCumulative,
                &[],
                &[],
                cfg_norecord,
            )
        },
    );
    println!(
        "  記録コスト概算(c): {:.3} s ({:.1}% of 総時間)",
        t_nonlinear - t_nonlinear_norecord,
        100.0 * (t_nonlinear - t_nonlinear_norecord) / t_nonlinear
    );
    println!(
        "  注記: 非線形は各ステップ最大 max_iter(=20) 回の Newton 反復を伴うため、\
         記録コストの比率は max_iter の実収束回数（塑性化の有無）にも依存する。\
         本モデルは弾性域〜軽微な塑性化のため、収束回数は数回程度で頭打ちのはず。"
    );

    println!();
    println!("=== ベースライン計測 完了 ===");
}
