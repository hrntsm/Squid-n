//! 部材スケルトン曲線の構築（公開エントリポイント）。
//!
//! 責務: [`crate::fiber_model`] のファイバ積分と [`crate::deformation`] の
//! M–θ 変換を組み合わせ、トリリニアの部材スケルトンを組み立てる。

use squid_n_core::model::Section;
use squid_n_material::{Bilinear, Concrete, HysteresisRule, UniaxialMaterial};
use squid_n_section::fiber::{section_response, SectionStrain};

use crate::deformation::{mphi_to_mtheta, PulloutContribution, ShearContribution};
use crate::fiber_model::{build_rc_fiber_section, compute_m_phi_curve_rc};
use crate::types::{MemberData, MemberSkeleton, Reinforcement, SkeletonOptions};

/// RC ファイバ格子の分割数（幅方向）。
const RC_GRID_W: usize = 16;
/// RC ファイバ格子の分割数（せい方向）。
const RC_GRID_D: usize = 32;
/// RC M–φ スイープのステップ数。
const RC_SWEEP_STEPS: usize = 800;

/// RC 部材スケルトンを構築する（仕様書 §7）。
///
/// 1. コンクリート格子＋主筋点ファイバのファイバ断面を構築。
/// 2. 軸力 n_axial を保ちながら M–φ を数値積分。ひび割れ・鉄筋降伏・コンクリート圧壊を
///    ひずみイベントで検出しトリリニア折点とする（規準式準拠のイベント駆動）。
/// 3. 反曲点比・塑性ヒンジ長で M–φ → M–θ へ変換。せん断変形・鉄筋抜出しを加算（§7 フロー4）。
///
/// 単位: 断面寸法・位置 [mm], 面積 [mm²], 軸力 [N], モーメント [N·mm], スパン [mm]。
pub fn build_rc_member_skeleton(
    section: &Section,
    reinforcement: &Reinforcement,
    concrete: &Concrete,
    steel: &Bilinear,
    opts: &SkeletonOptions,
    shear: &ShearContribution,
    pullout: &PulloutContribution,
) -> MemberSkeleton {
    let span = opts.span;
    let inflection_ratio = opts.inflection_ratio;
    let n_axial = opts.n_axial;
    let plastic_hinge_length = 0.5 * section.depth;

    // スイープ範囲を断面・材料から適応的に決定（降伏・終局曲率を十分に解像する）。
    let e0_conc = 2.0 * concrete.fc / concrete.ec0.abs();
    let eps_cr = concrete.ft / e0_conc;
    let eps_y = steel.fy / steel.e;
    let d_eff = section.depth
        - reinforcement
            .main_bars
            .iter()
            .map(|(_, z, _)| (section.depth / 2.0 - z).abs())
            .min_by(|a, b| a.total_cmp(b))
            .unwrap_or(section.depth / 2.0);
    let j = 7.0 * d_eff / 8.0;
    let ky_yield_est = eps_y / j;
    let ky_ultimate_est = concrete.ecu.abs() / (section.depth / 2.0);
    let max_curvature = (3.0 * ky_ultimate_est).max(2.0 * ky_yield_est).max(1e-4);

    let (fibers, mut mats, roles) = build_rc_fiber_section(
        section.width,
        section.depth,
        RC_GRID_W,
        RC_GRID_D,
        reinforcement,
        concrete,
        steel,
    );

    let events = compute_m_phi_curve_rc(
        &fibers,
        &mut mats,
        &roles,
        concrete,
        steel,
        n_axial,
        max_curvature,
        RC_SWEEP_STEPS,
    );

    // イベントから折点を取り出す。検出されなかった場合は推定曲率で補完。
    let (ky_crack, m_c) = events
        .crack
        .unwrap_or((eps_cr / (section.depth / 2.0), 0.0));
    let (ky_yield, m_y) = events
        .yield_pt
        .unwrap_or((ky_yield_est, section.iy * e0_conc * ky_yield_est));
    let (ky_ultimate, m_u) = events.ultimate.unwrap_or((ky_ultimate_est, m_y * 1.2));

    let convert = |ky: f64, m: f64| {
        mphi_to_mtheta(
            ky,
            m,
            Some(ky_yield),
            span,
            inflection_ratio,
            plastic_hinge_length,
            *shear,
            *pullout,
        )
        .0
    };
    let (m_c, m_y, m_u) = (m_c.abs(), m_y.abs(), m_u.abs());
    let theta_c = convert(ky_crack, m_c);
    let theta_y = convert(ky_yield, m_y);
    let theta_u = convert(ky_ultimate, m_u);

    let points = vec![(0.0, 0.0), (theta_c, m_c), (theta_y, m_y), (theta_u, m_u)];
    let hysteresis = HysteresisRule::Takeda {
        crack: (m_c, theta_c),
        yield_point: (m_y, theta_y),
        ultimate: (m_u, theta_u),
        alpha: opts.alpha,
    };

    MemberSkeleton::with_axial_entry(points, hysteresis, n_axial)
}

/// 既定のファイバ断面（呼出側提供）からスケルトンを構築する（汎用パス）。
/// `mats.len() == member.fibers.fibers.len()` が必要（ファイバごとの独立状態）。
/// RC の場合は `build_rc_member_skeleton` を用いること。
/// `alpha` は武田モデルの除荷剛性低下指数（外部設定。代表 0.4〜0.5）。
pub fn build_member_skeleton(
    member: &MemberData,
    n_axial: f64,
    mats: &mut [Box<dyn UniaxialMaterial>],
    alpha: f64,
) -> MemberSkeleton {
    assert_eq!(
        mats.len(),
        member.fibers.fibers.len(),
        "build_member_skeleton: mats.len() must equal fibers.len() (per-fiber state)"
    );
    let max_curvature = 0.01;
    let num_steps = 200;
    let plastic_hinge_length = 0.5 * member.section.depth;

    let mut points = Vec::with_capacity(num_steps + 1);
    for mat in mats.iter_mut() {
        mat.revert();
    }
    let dk = max_curvature / num_steps as f64;
    for i in 0..=num_steps {
        let ky = i as f64 * dk;
        let mut eps0 = 0.0;
        for _ in 0..50 {
            let (force, _) =
                section_response(member.fibers, SectionStrain { eps0, ky, kz: 0.0 }, mats);
            let residual = force.n - n_axial;
            if residual.abs() < n_axial.abs().max(1.0) * 1e-6 {
                break;
            }
            let (force_p, _) = section_response(
                member.fibers,
                SectionStrain {
                    eps0: eps0 + 1e-8,
                    ky,
                    kz: 0.0,
                },
                mats,
            );
            let dn = (force_p.n - force.n) / 1e-8;
            if dn.abs() < 1e-15 {
                break;
            }
            eps0 -= residual / dn;
        }
        let (force, _) = section_response(member.fibers, SectionStrain { eps0, ky, kz: 0.0 }, mats);
        for m in mats.iter_mut() {
            m.commit();
        }
        points.push((ky, force.my));
    }

    // 折点: M-φ 曲線の勾配変化を簡易抽出（汎用パス。RC は build_rc を使用）
    let trilinear = extract_trilinear_generic(&points);
    let ky_y = trilinear.get(2).map(|p| p.0);
    let mtheta: Vec<(f64, f64)> = trilinear
        .iter()
        .map(|&(ky, m)| {
            mphi_to_mtheta(
                ky,
                m,
                ky_y,
                member.span,
                member.inflection_ratio,
                plastic_hinge_length,
                ShearContribution::none(),
                PulloutContribution::none(),
            )
        })
        .collect();

    let pt = |i: usize| {
        (
            mtheta.get(i).map(|p| p.1).unwrap_or(0.0),
            mtheta.get(i).map(|p| p.0).unwrap_or(0.0),
        )
    };
    let ultimate = mtheta.last().map(|p| (p.1, p.0)).unwrap_or((0.0, 0.0));
    let hysteresis = HysteresisRule::Takeda {
        crack: pt(1),
        yield_point: pt(2),
        ultimate,
        alpha,
    };

    MemberSkeleton::with_axial_entry(mtheta, hysteresis, n_axial)
}

/// 汎用パスの折点抽出（勾配ヒューリスティック。RC には build_rc を用いること）。
fn extract_trilinear_generic(mphi: &[(f64, f64)]) -> Vec<(f64, f64)> {
    if mphi.is_empty() {
        return vec![(0.0, 0.0)];
    }
    let n = mphi.len();
    let crack_idx = mphi
        .iter()
        .position(|(_, m)| m.abs() > 0.0)
        .unwrap_or(1)
        .max(1);
    let init_slope = if mphi[crack_idx].0.abs() > 1e-15 {
        mphi[crack_idx].1 / mphi[crack_idx].0
    } else {
        mphi[1].1 / mphi[1].0
    };
    let yield_idx = (crack_idx + 1..n)
        .find(|&i| {
            let dmdk = (mphi[i].1 - mphi[i - 1].1) / (mphi[i].0 - mphi[i - 1].0).max(1e-15);
            init_slope > 0.0 && dmdk / init_slope < 0.3
        })
        .unwrap_or(n - 1);
    let ultimate_idx = n - 1;
    vec![
        (0.0, 0.0),
        (mphi[crack_idx.min(n - 1)].0, mphi[crack_idx.min(n - 1)].1),
        (mphi[yield_idx].0, mphi[yield_idx].1),
        (mphi[ultimate_idx].0, mphi[ultimate_idx].1),
    ]
}
