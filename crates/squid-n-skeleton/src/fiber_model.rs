//! RC 断面のファイバモデル構築と M–φ（モーメント–曲率）関係の数値積分。
//!
//! 責務: ファイバ断面の生成と、軸力一定条件下での曲率スイープによるイベント
//! （ひび割れ・鉄筋降伏・コンクリート圧壊）抽出。M–θ への変換や折点整形は
//! [`crate::deformation`] / [`crate::builder`] が担う。

use squid_n_material::{Bilinear, Concrete, UniaxialMaterial};
use squid_n_section::fiber::{
    rect_fiber_section, section_response, Fiber, FiberSection, SectionStrain,
};

use crate::Reinforcement;

/// 中立軸探索ニュートン法の最大反復数。
const AXIAL_MAX_ITER: usize = 50;
/// 中立軸探索の相対残差許容値。
const AXIAL_TOL: f64 = 1e-6;
/// 軸力残差の有限差分に用いる ε [ひずみ]。
const AXIAL_FD_EPS: f64 = 1e-8;

/// ファイバの役割（イベント抽出用）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum FiberRole {
    Concrete,
    Steel,
}

/// M–φ 解析のイベント結果。折点は (曲率 κ, モーメント M)。
pub(crate) struct MPhiEvents {
    /// ひび割れ曲率・モーメント
    pub crack: Option<(f64, f64)>,
    /// 降伏曲率・モーメント
    pub yield_pt: Option<(f64, f64)>,
    /// 終局（ピークモーメント）曲率・モーメント
    pub ultimate: Option<(f64, f64)>,
}

/// 軸力 `n_axial` を保つ中立軸ひずみ `eps0` をニュートン法で求める。
///
/// `ky`（曲率）を固定し、断面応答の軸力が `n_axial` に一致する `eps0` を返す。
/// 収束しない場合は最終反復値をそのまま返す（呼出側で許容）。
fn solve_axial_strain(
    fibers: &FiberSection,
    ky: f64,
    n_axial: f64,
    mats: &mut [Box<dyn UniaxialMaterial>],
) -> f64 {
    let mut eps0 = 0.0;
    for _ in 0..AXIAL_MAX_ITER {
        let (force, _) = section_response(fibers, SectionStrain { eps0, ky, kz: 0.0 }, mats);
        let residual = force.n - n_axial;
        if residual.abs() < n_axial.abs().max(1.0) * AXIAL_TOL {
            break;
        }
        let (force_p, _) = section_response(
            fibers,
            SectionStrain {
                eps0: eps0 + AXIAL_FD_EPS,
                ky,
                kz: 0.0,
            },
            mats,
        );
        let dn = (force_p.n - force.n) / AXIAL_FD_EPS;
        if dn.abs() < 1e-15 {
            break;
        }
        eps0 -= residual / dn;
    }
    eps0
}

/// ファイバ断面から M–φ 関係を数値積分で算定する。
/// 軸力 n_axial [N] を保ちながら曲率 κ [1/mm] を増やし、各ステップでファイバ状態を commit する。
/// ひび割れ（コンクリート引張ひび割れ）・降伏（鉄筋降伏）・終局（ピークモーメント）を検出する。
#[allow(clippy::too_many_arguments)]
pub(crate) fn compute_m_phi_curve_rc(
    fibers: &FiberSection,
    mats: &mut [Box<dyn UniaxialMaterial>],
    roles: &[FiberRole],
    concrete: &Concrete,
    steel: &Bilinear,
    n_axial: f64,
    max_curvature: f64,
    num_steps: usize,
) -> MPhiEvents {
    let e0_conc = 2.0 * concrete.fc / concrete.ec0.abs();
    let eps_cr = concrete.ft / e0_conc;
    let eps_cu = concrete.ecu;
    let eps_y = steel.fy / steel.e;

    let mut crack: Option<(f64, f64)> = None;
    let mut yield_pt: Option<(f64, f64)> = None;
    let mut peak_m = 0.0f64;
    let mut peak_ky = 0.0f64;
    let mut crushed = false;

    for mat in mats.iter_mut() {
        mat.revert();
    }

    let dk = max_curvature / num_steps as f64;
    for i in 0..=num_steps {
        let ky = i as f64 * dk;

        // eps0 をニュートン法で探索（N = n_axial）
        let eps0 = solve_axial_strain(fibers, ky, n_axial, mats);

        let (force, _) = section_response(fibers, SectionStrain { eps0, ky, kz: 0.0 }, mats);
        let m = force.my.abs();
        if m > peak_m {
            peak_m = m;
            peak_ky = ky;
        }

        // イベント検出（ファイバひずみで判定。kz=0 なので eps = eps0 + ky·z）
        if crack.is_none() || yield_pt.is_none() || !crushed {
            for (j, f) in fibers.fibers.iter().enumerate() {
                let eps = eps0 + ky * f.z;
                match roles[j] {
                    FiberRole::Concrete => {
                        if crack.is_none() && eps > eps_cr {
                            crack = Some((ky, force.my));
                        }
                        if eps < eps_cu {
                            crushed = true;
                        }
                    }
                    FiberRole::Steel => {
                        if yield_pt.is_none() && eps.abs() > eps_y {
                            yield_pt = Some((ky, force.my));
                        }
                    }
                }
            }
        }

        // ファイバ状態をコミット（履歴を進める）
        for mat in mats.iter_mut() {
            mat.commit();
        }

        if crushed && ky > peak_ky {
            break;
        }
    }

    // 終局 = ピークモーメント点（skeleton は正で格納）
    let ultimate = (peak_m > 0.0).then_some((peak_ky, peak_m));

    MPhiEvents {
        crack,
        yield_pt,
        ultimate,
    }
}

/// RC 断面のファイバモデルを構築する。
/// コンクリートは矩形グリッド、主筋は点ファイバ。各ファイバが独自の材料状態を持つ。
/// （主筋位置のコンクリート重複計上は微小な近似。厳密には断面積を控除する。）
pub(crate) fn build_rc_fiber_section(
    width: f64,
    depth: f64,
    nw: usize,
    nd: usize,
    reinforcement: &Reinforcement,
    concrete: &Concrete,
    steel: &Bilinear,
) -> (FiberSection, Vec<Box<dyn UniaxialMaterial>>, Vec<FiberRole>) {
    let capacity = nw * nd + reinforcement.main_bars.len();
    let mut fibers = Vec::with_capacity(capacity);
    let mut mats: Vec<Box<dyn UniaxialMaterial>> = Vec::with_capacity(capacity);
    let mut roles = Vec::with_capacity(capacity);

    // コンクリート格子（分割規則の情報源は `squid_n_section::fiber`）。
    for fiber in rect_fiber_section(width, depth, nw, nd, 0).fibers {
        fibers.push(fiber);
        mats.push(concrete.clone_box());
        roles.push(FiberRole::Concrete);
    }
    // 主筋（点ファイバ）
    for &(y, z, area) in &reinforcement.main_bars {
        fibers.push(Fiber {
            y,
            z,
            area,
            material: 1,
        });
        mats.push(steel.clone_box());
        roles.push(FiberRole::Steel);
    }
    (FiberSection { fibers }, mats, roles)
}
