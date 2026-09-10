use crate::common::assemble::{assemble_global_k, assemble_global_m};
use crate::common::constraint::Reducer;
use squid_n_core::dof::DofMap;
use squid_n_core::model::Model;
use squid_n_element::behavior::MassOption;
use squid_n_math::solver::{make_solver, LinearSolver, SolveError, SolverBackend};

const EIGEN_TOL: f64 = 1e-10;
const EIGEN_MAX_ITER: usize = 200;
/// 一般化 Jacobi で同時対角化した後の質量対角成分 m̂ᵢᵢ が、その最大値との
/// 相対でこの値未満の方向を「質量を持たない方向」として扱う
/// （質量ランク判定の相対許容誤差。[`gevd_jacobi`] 参照）。
const MASS_RANK_REL_TOL: f64 = 1e-9;

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ModalResult {
    pub omega2: Vec<f64>,
    pub period: Vec<f64>,
    /// モード形状（縮約後の独立自由度座標、長さ = `Reducer::n_indep`）。
    pub shapes: Vec<Vec<f64>>,
    /// モード形状を節点×6成分（UX,UY,UZ,RX,RY,RZ）へ展開したもの。
    /// 剛床のスレーブ自由度にはマスターと整合した値が入る。
    pub node_shapes: Vec<Vec<[f64; 6]>>,
    pub participation: Vec<[f64; 3]>,
    pub effective_mass: Vec<[f64; 3]>,
}

/// 固有値解析（部分空間反復）。
///
/// 要求モード数 `n_modes` が縮約後の自由度数を超える場合、返る結果のモード数は
/// 自由度数まで切り詰められる（`ModalResult::omega2.len()` は `n_modes` 以下に
/// なり得る）。呼び出し側は返り値の長さを確認すること。
pub fn solve_eigen(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    n_modes: usize,
) -> Result<ModalResult, SolveError> {
    if reducer.n_indep == 0 || n_modes == 0 {
        return Ok(ModalResult {
            omega2: vec![],
            period: vec![],
            shapes: vec![],
            node_shapes: vec![],
            participation: vec![],
            effective_mass: vec![],
        });
    }

    let k_free = assemble_global_k(model, dofmap);
    let k_red = reducer.reduce_k(&k_free);

    let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
    solver.factorize(&k_red).map_err(|e| match e {
        SolveError::NotPositiveDefinite => SolveError::InvalidInput(
            "固有値解析: 剛性行列が特異(非正定値)です。拘束が不足しているか、\
             構造が機構(不安定)になっている可能性があります。支持条件を確認してください。"
                .into(),
        ),
        other => other,
    })?;

    solve_eigen_with_solver(model, dofmap, reducer, n_modes, solver.as_ref())
}

/// `solver` は縮約後剛性行列 K_red に対して呼び出し側で既に `factorize` 済みであることを前提とする。
/// 要求自由度が 0 の場合はソルバに触れずに早期リターンする。
pub fn solve_eigen_with_solver(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    n_modes: usize,
    solver: &dyn LinearSolver,
) -> Result<ModalResult, SolveError> {
    let m_free = assemble_global_m(model, dofmap, MassOption::Consistent);
    let m_red = reducer.reduce_k(&m_free);
    let n = m_red.nrows();
    let n_modes = n_modes.min(n);
    if n == 0 || n_modes == 0 {
        return Ok(ModalResult {
            omega2: vec![],
            period: vec![],
            shapes: vec![],
            node_shapes: vec![],
            participation: vec![],
            effective_mass: vec![],
        });
    }

    let mass_trace: f64 = (0..n)
        .map(|i| m_red.get(i, i).copied().unwrap_or(0.0))
        .sum();
    if mass_trace <= 0.0 {
        return Err(SolveError::InvalidInput(
            "質量がゼロです。材料の密度(ρ)を設定するか、節点質量を与えてください。".into(),
        ));
    }
    if (0..n).any(|i| m_red.get(i, i).copied().unwrap_or(0.0) < 0.0) {
        return Err(SolveError::InvalidInput(
            "質量行列の対角に負値があります。節点質量(node.mass)や材料の密度(ρ)に\
             負の値が入力されていないか確認してください。"
                .into(),
        ));
    }

    let k_free = assemble_global_k(model, dofmap);
    let k_red = reducer.reduce_k(&k_free);

    let q = ((2 * n_modes).min(n_modes + 8)).max(n_modes + 4).min(n);

    let k_diag: Vec<f64> = (0..n)
        .map(|i| k_red.get(i, i).copied().unwrap_or(0.0))
        .collect();
    let m_diag: Vec<f64> = (0..n)
        .map(|i| m_red.get(i, i).copied().unwrap_or(0.0))
        .collect();
    let mut x = init_subspace(n, q, &k_diag, &m_diag);

    let mut theta_prev = vec![f64::MAX; n_modes];
    let mut is_converged = false;
    let mut last_eigenvalues = vec![f64::MAX; q];

    let mut y = vec![0.0; n * q];
    let mut x_new = vec![0.0; n * q];
    let mut x_col = vec![0.0; n];
    let mut rhs = vec![0.0; n];
    let mut yi: Vec<f64> = Vec::new();
    let mut k_bar = vec![0.0; q * q];
    let mut m_bar = vec![0.0; q * q];
    let mut proj_col_buf = vec![0.0; n];
    let mut proj_z_buf = vec![0.0; n * q];

    for _iteration in 0..EIGEN_MAX_ITER {
        for col in 0..q {
            for r in 0..n {
                x_col[r] = x[r * q + col];
            }
            squid_n_math::sparse::sparse_matvec_into(&m_red, &x_col, &mut rhs);
            solver.solve_into(&rhs, &mut yi)?;
            for r in 0..n {
                y[r * q + col] = yi[r];
            }
        }

        proj_yty(
            &y,
            &k_red,
            n,
            q,
            &mut proj_col_buf,
            &mut proj_z_buf,
            &mut k_bar,
        );
        proj_yty(
            &y,
            &m_red,
            n,
            q,
            &mut proj_col_buf,
            &mut proj_z_buf,
            &mut m_bar,
        );

        let (eigenvalues, eigvecs_q) = gevd_jacobi(&k_bar, &m_bar, q);

        for i in 0..n {
            for j in 0..q {
                let mut s = 0.0;
                for k in 0..q {
                    s += y[i * q + k] * eigvecs_q[k * q + j];
                }
                x_new[i * q + j] = s;
            }
        }
        std::mem::swap(&mut x, &mut x_new);

        let mut converged = 0;
        for m in 0..n_modes {
            let th = eigenvalues[m];
            let same = if th.is_finite() && theta_prev[m].is_finite() {
                (th - theta_prev[m]).abs() < EIGEN_TOL * th.max(1.0)
            } else {
                th == theta_prev[m]
            };
            if same {
                converged += 1;
            }
            theta_prev[m] = th;
        }
        last_eigenvalues = eigenvalues;
        if converged == n_modes {
            is_converged = true;
            break;
        }
    }

    if !is_converged {
        return Err(SolveError::NonConvergence(format!(
            "固有値解析(部分空間反復)が {} 回で収束しませんでした。モデルの質量・剛性の分布を確認してください。",
            EIGEN_MAX_ITER
        )));
    }

    let mass_rank = last_eigenvalues.iter().filter(|v| v.is_finite()).count();
    if theta_prev.iter().any(|v| !v.is_finite()) {
        return Err(SolveError::InvalidInput(format!(
            "固有値解析: 要求モード数({n_modes})に対し、質量が有効な独立自由度が{mass_rank}個しか見つかりませんでした。\
node.mass や材料の密度(ρ)で並進質量を追加するか、要求モード数を{mass_rank}以下に減らしてください。"
        )));
    }

    let mut omega2 = vec![0.0; n_modes];
    let mut period = vec![0.0; n_modes];
    let mut shapes = Vec::with_capacity(n_modes);

    for m in 0..n_modes {
        omega2[m] = theta_prev[m];
        period[m] = if omega2[m] > 0.0 {
            2.0 * std::f64::consts::PI / omega2[m].sqrt()
        } else {
            0.0
        };

        let mut phi = vec![0.0; n];
        for i in 0..n {
            phi[i] = x[i * q + m];
        }
        let norm2 = m_norm(&phi, &m_red);
        if norm2 > 0.0 {
            let inv = 1.0 / norm2.sqrt();
            for v in &mut phi {
                *v *= inv;
            }
        }
        shapes.push(phi);
    }

    let (participation, effective_mass, node_shapes) =
        compute_participation(&shapes, &m_free, reducer, dofmap, model);

    Ok(ModalResult {
        omega2,
        period,
        shapes,
        node_shapes,
        participation,
        effective_mass,
    })
}

/// 展開済み（`Reducer::expand_u` 済み）のモード形状を節点×6成分へ散布する。
///
/// `phi_free` は呼び出し側で 1 回だけ計算した値を渡す。
/// 剛床のスレーブ自由度にはマスターに従属した値が入る。fixed・非構造自由度は 0。
fn scatter_node_shape(phi_free: &[f64], dofmap: &DofMap, n_nodes: usize) -> Vec<[f64; 6]> {
    dofmap.expand_to_nodes(phi_free, n_nodes)
}

/// 部分空間反復の開始ベクトルを選ぶ。
///
/// 1本目は質量分布に比例した変位パターン、残りは剛性/質量比の小さい順に単位ベクトルを割り当てる。
/// 質量ゼロの自由度は比を +∞ とみなす。
fn init_subspace(n: usize, q: usize, k_diag: &[f64], m_diag: &[f64]) -> Vec<f64> {
    let mut x = vec![0.0; n * q];
    if q == 0 {
        return x;
    }
    for i in 0..n {
        x[i * q] = m_diag[i];
    }
    let mut ratios: Vec<(usize, f64)> = (0..n)
        .map(|i| {
            let r = if m_diag[i] > 0.0 {
                k_diag[i] / m_diag[i]
            } else {
                f64::INFINITY
            };
            (i, r)
        })
        .collect();
    ratios.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
    for col in 1..q {
        let dof = ratios[col - 1].0;
        x[dof * q + col] = 1.0;
    }
    x
}

/// 疎行列とベクトルの積 y = A·x。
fn spmv(mat: &faer::sparse::SparseColMat<usize, f64>, x: &[f64]) -> Vec<f64> {
    squid_n_math::sparse::sparse_matvec(mat, x)
}

/// Y^T·A·Y（q×q 対称行列）を、A の非ゼロ要素だけを使って計算する。
///
/// `col_buf`（長さ n）・`z_buf`（長さ n*q）・`result`（長さ q*q）は呼び出し側が確保して使い回す作業バッファ。
fn proj_yty(
    y: &[f64],
    mat_red: &faer::sparse::SparseColMat<usize, f64>,
    n: usize,
    q: usize,
    col_buf: &mut [f64],
    z_buf: &mut [f64],
    result: &mut [f64],
) {
    for j in 0..q {
        for r in 0..n {
            col_buf[r] = y[r * q + j];
        }
        squid_n_math::sparse::sparse_matvec_into(mat_red, col_buf, &mut z_buf[j * n..(j + 1) * n]);
    }

    for i in 0..q {
        for j in i..q {
            let mut s = 0.0;
            for a in 0..n {
                s += y[a * q + i] * z_buf[j * n + a];
            }
            result[i * q + j] = s;
            result[j * q + i] = s;
        }
    }
}

/// φᵀ·M·φ を M の非ゼロ要素だけを使って計算する。
fn m_norm(phi: &[f64], m_red: &faer::sparse::SparseColMat<usize, f64>) -> f64 {
    let m_phi = spmv(m_red, phi);
    phi.iter().zip(m_phi.iter()).map(|(p, mp)| p * mp).sum()
}

/// 一般化固有値問題 K*z = θ*M*z を一般化 Jacobi 法で解く。
///
/// 質量を持たない方向には θ=+∞ を割り当てる。
/// 質量を持つ方向の固有ベクトルは M̄ 正規直交（zᵀM̄z = 1）、質量ゼロ方向は単位ノルムに正規化して返す。
/// Returns (eigenvalues ascending; eigenvectors as columns)。
fn gevd_jacobi(k_in: &[f64], m_in: &[f64], n: usize) -> (Vec<f64>, Vec<f64>) {
    let s: Vec<f64> = (0..n)
        .map(|i| {
            let d = k_in[i * n + i];
            if d.is_finite() && d > 0.0 {
                1.0 / d.sqrt()
            } else {
                1.0
            }
        })
        .collect();
    let mut k = vec![0.0; n * n];
    let mut m = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            k[i * n + j] = s[i] * k_in[i * n + j] * s[j];
            m[i * n + j] = s[i] * m_in[i * n + j] * s[j];
        }
    }
    let mut vecs = vec![0.0; n * n];
    for i in 0..n {
        vecs[i * n + i] = 1.0;
    }

    const MAX_SWEEPS: usize = 100;
    const COUPLE_TOL: f64 = 1e-24;
    for _sweep in 0..MAX_SWEEPS {
        let m_diag_max = (0..n)
            .map(|i| m[i * n + i].max(0.0))
            .fold(0.0_f64, f64::max);
        let m_floor = (m_diag_max * MASS_RANK_REL_TOL).max(f64::MIN_POSITIVE);
        let mut rotated = false;
        for i in 0..n {
            for j in (i + 1)..n {
                let kii = k[i * n + i];
                let kjj = k[j * n + j];
                let kij = k[i * n + j];
                let mii = m[i * n + i];
                let mjj = m[j * n + j];
                let mij = m[i * n + j];

                let k_couple = kij * kij / (kii * kjj).max(f64::MIN_POSITIVE);
                let m_couple = mij * mij / (mii.max(m_floor) * mjj.max(m_floor));
                if k_couple < COUPLE_TOL && m_couple < COUPLE_TOL {
                    continue;
                }

                let a1 = kii * mij - mii * kij;
                let a2 = kjj * mij - mjj * kij;
                let a3 = kii * mjj - kjj * mii;
                let root = ((a3 * 0.5) * (a3 * 0.5) + a1 * a2).max(0.0).sqrt();
                let x = if a3 >= 0.0 {
                    a3 * 0.5 + root
                } else {
                    a3 * 0.5 - root
                };
                let (alpha, gamma) = if x.abs() > f64::MIN_POSITIVE && (a1 != 0.0 || a2 != 0.0) {
                    (a2 / x, -a1 / x)
                } else if kjj.abs() > f64::MIN_POSITIVE {
                    (-kij / kjj, 0.0)
                } else {
                    (0.0, 0.0)
                };
                if alpha == 0.0 && gamma == 0.0 {
                    continue;
                }
                rotated = true;

                for mat in [&mut k, &mut m] {
                    for row in 0..n {
                        let ai = mat[row * n + i];
                        let aj = mat[row * n + j];
                        mat[row * n + i] = ai + gamma * aj;
                        mat[row * n + j] = aj + alpha * ai;
                    }
                    for col in 0..n {
                        let ai = mat[i * n + col];
                        let aj = mat[j * n + col];
                        mat[i * n + col] = ai + gamma * aj;
                        mat[j * n + col] = aj + alpha * ai;
                    }
                }
                for row in 0..n {
                    let vi = vecs[row * n + i];
                    let vj = vecs[row * n + j];
                    vecs[row * n + i] = vi + gamma * vj;
                    vecs[row * n + j] = vj + alpha * vi;
                }
            }
        }
        if !rotated {
            break;
        }
    }

    let m_diag_max = (0..n)
        .map(|i| m[i * n + i].max(0.0))
        .fold(0.0_f64, f64::max);
    let mass_tol = MASS_RANK_REL_TOL * m_diag_max;
    let mut vals = vec![f64::INFINITY; n];
    for i in 0..n {
        let mii = m[i * n + i];
        if m_diag_max > 0.0 && mii > mass_tol {
            vals[i] = k[i * n + i] / mii;
        }
    }

    for col in 0..n {
        if vals[col].is_finite() {
            let inv = 1.0 / m[col * n + col].sqrt();
            for row in 0..n {
                vecs[row * n + col] *= inv;
            }
        }
    }
    for row in 0..n {
        for col in 0..n {
            vecs[row * n + col] *= s[row];
        }
    }
    for col in 0..n {
        if vals[col].is_finite() {
            continue;
        }
        let norm2: f64 = (0..n).map(|row| vecs[row * n + col].powi(2)).sum();
        if norm2 > 0.0 {
            let inv = 1.0 / norm2.sqrt();
            for row in 0..n {
                vecs[row * n + col] *= inv;
            }
        }
    }

    let mut idx: Vec<usize> = (0..n).collect();
    idx.sort_by(|&a, &b| {
        vals[a]
            .partial_cmp(&vals[b])
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut sorted_vals = vec![0.0; n];
    let mut sorted_vecs = vec![0.0; n * n];
    for (new_pos, &orig) in idx.iter().enumerate() {
        sorted_vals[new_pos] = vals[orig];
        for i in 0..n {
            sorted_vecs[i * n + new_pos] = vecs[i * n + orig];
        }
    }

    (sorted_vals, sorted_vecs)
}

/// [`compute_participation`] の戻り値
/// （刺激係数・有効質量比・節点単位のモード形状のタプル）。
type ParticipationResult = (Vec<[f64; 3]>, Vec<[f64; 3]>, Vec<Vec<[f64; 6]>>);

/// モード刺激係数・有効質量比を計算し、あわせて節点単位のモード形状も返す。
fn compute_participation(
    shapes: &[Vec<f64>],
    m_free: &faer::sparse::SparseColMat<usize, f64>,
    reducer: &Reducer,
    dofmap: &DofMap,
    model: &Model,
) -> ParticipationResult {
    let n_modes = shapes.len();
    let mut participation = vec![[0.0; 3]; n_modes];
    let mut effective_mass = vec![[0.0; 3]; n_modes];
    let mut node_shapes = Vec::with_capacity(n_modes);

    let n_free = dofmap.n_active();
    let n_nodes = model.nodes.len();

    let r_free: [Vec<f64>; 3] = std::array::from_fn(|dir_idx| {
        let mut r = vec![0.0; n_free];
        for ni in 0..n_nodes {
            let g = ni * squid_n_core::dof::DOF_PER_NODE + dir_idx;
            if let Some(active) = dofmap.active(g) {
                r[active as usize] = 1.0;
            }
        }
        r
    });

    for (m_idx, phi_red) in shapes.iter().enumerate() {
        let phi_free = reducer.expand_u(phi_red);

        let m_phi = spmv(m_free, &phi_free);

        let mut phi_m_phi = 0.0;
        for a in 0..n_free {
            phi_m_phi += phi_free[a] * m_phi[a];
        }

        for (dir_idx, r) in r_free.iter().enumerate() {
            let mut phi_m_r = 0.0;
            for a in 0..n_free {
                phi_m_r += m_phi[a] * r[a];
            }

            if phi_m_phi.abs() > 1e-30 {
                participation[m_idx][dir_idx] = phi_m_r / phi_m_phi;
                effective_mass[m_idx][dir_idx] = phi_m_r * phi_m_r / phi_m_phi;
            }
        }

        node_shapes.push(scatter_node_shape(&phi_free, dofmap, n_nodes));
    }

    (participation, effective_mass, node_shapes)
}

#[cfg(test)]
mod tests;
