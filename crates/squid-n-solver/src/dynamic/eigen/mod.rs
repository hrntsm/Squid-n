use crate::assemble::{assemble_global_k, assemble_global_m};
use crate::constraint::Reducer;
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
    /// 時刻歴のモード減衰（`crate::damping::Damping::modal`）など、縮約空間で
    /// 計算する消費者向け。節点単位の形状が必要な場合は [`Self::node_shapes`] を使う。
    pub shapes: Vec<Vec<f64>>,
    /// モード形状を節点×6成分（UX,UY,UZ,RX,RY,RZ）へ展開したもの
    /// （`shapes` を `Reducer::expand_u` → `DofMap` 散布した結果）。
    /// 可視化・レポートなど節点単位の消費者向け。剛床のスレーブ自由度にも
    /// マスターと整合した値が入る。
    pub node_shapes: Vec<Vec<[f64; 6]>>,
    pub participation: Vec<[f64; 3]>,
    pub effective_mass: Vec<[f64; 3]>,
}

pub fn solve_eigen(
    model: &Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    n_modes: usize,
) -> Result<ModalResult, SolveError> {
    // 自由度数（縮約後）が 0 なら分解すら不要（0×0 行列を factorize しない）。
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

    // 部分空間反復では 1 回の分解を（部分空間サイズ×反復回数）回の求解で
    // 再利用するため、直接法を明示する（反復法では再利用が効かない）。
    let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
    solver.factorize(&k_red)?;

    solve_eigen_with_solver(model, dofmap, reducer, n_modes, solver.as_ref())
}

/// [`solve_eigen`] の本体ロジック。呼び出し側（[`crate::analysis::Analysis`]）が
/// 既に縮約後剛性行列 `k_red` を分解済みソルバとして保持している場合に、
/// その分解を再利用して再分解のコストを省くための版。
///
/// `solver` は縮約後剛性行列 K_red（本関数が内部で `assemble_global_k` +
/// `reducer.reduce_k` により組み立てるものと同一の行列）に対して、
/// 呼び出し側で既に `factorize` 済みであることを前提とする（本関数は
/// factorize を行わない）。要求自由度が 0（`reducer.n_indep == 0`）の場合は
/// ソルバに一切触れずに早期リターンするため、未分解のソルバを渡しても安全。
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
    // 自由度数（縮約後）を超えるモードは存在しないので上限で抑える。
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

    // 質量ゼロ（密度・節点質量とも未設定）の検出。
    // M ≈ 0 のまま進めると GEVD が対角フォールバックし周期 0 の無意味な結果になる。
    let mass_trace: f64 = (0..n)
        .map(|i| m_red.get(i, i).copied().unwrap_or(0.0))
        .sum();
    if mass_trace <= 0.0 {
        return Err(SolveError::InvalidInput(
            "質量がゼロです。材料の密度(ρ)を設定するか、節点質量を与えてください。".into(),
        ));
    }

    let k_free = assemble_global_k(model, dofmap);
    let k_red = reducer.reduce_k(&k_free);

    // 部分空間サイズ q: Bathe の定石 q = min(2p, p+8) にならい、要求モード数 p に対して
    // オーバーサンプリングする（p が大きいときに q が際限なく増えて計算コストが
    // 爆発しないよう +8 側で頭打ちにする）。ただし少なくとも p+4 は確保し、
    // 行列次元 n は超えない。q=p+1（旧下限）では p=1〜2 の少モード要求時に基底が
    // 縮退してドリフトし、真値より高い固有値へ停滞することがあった。
    let q = ((2 * n_modes).min(n_modes + 8)).max(n_modes + 4).min(n);

    // 開始ベクトルは Bathe の部分空間反復の定石に従い、質量情報を使って選ぶ
    // （単純に自由度番号の若い順に単位ベクトルを選ぶと、回転自由度など質量ゼロの
    // 自由度ばかりを拾ってしまい、水平質点系モデルのように質量を持つ自由度が
    // 少数・偏在するモデルで q が実際の質量ランクより小さいと、質量を持つ自由度が
    // 開始部分空間に一本も入らず反復が正しい低次モードへ収束できないことがある）。
    let k_diag: Vec<f64> = (0..n)
        .map(|i| k_red.get(i, i).copied().unwrap_or(0.0))
        .collect();
    let m_diag: Vec<f64> = (0..n)
        .map(|i| m_red.get(i, i).copied().unwrap_or(0.0))
        .collect();
    let mut x = init_subspace(n, q, &k_diag, &m_diag);

    let mut theta_prev = vec![f64::MAX; n_modes];
    let mut is_converged = false;
    // 質量ランク不足の判定に使う: 最後に計算した部分空間内の固有値（昇順、
    // 質量ゼロ方向は +∞ になる）。
    let mut last_eigenvalues = vec![f64::MAX; q];

    for _iteration in 0..EIGEN_MAX_ITER {
        let mut y = vec![0.0; n * q];
        for col in 0..q {
            let x_col: Vec<f64> = (0..n).map(|r| x[r * q + col]).collect();
            let rhs = spmv(&m_red, &x_col);
            let yi = solver.solve(&rhs)?;
            for r in 0..n {
                y[r * q + col] = yi[r];
            }
        }

        let k_bar = proj_yty(&y, &k_red, n, q);
        let m_bar = proj_yty(&y, &m_red, n, q);

        let (eigenvalues, eigvecs_q) = gevd_jacobi(&k_bar, &m_bar, q);

        let mut x_new = vec![0.0; n * q];
        for i in 0..n {
            for j in 0..q {
                let mut s = 0.0;
                for k in 0..q {
                    s += y[i * q + k] * eigvecs_q[k * q + j];
                }
                x_new[i * q + j] = s;
            }
        }
        x = x_new;

        let mut converged = 0;
        for m in 0..n_modes {
            let th = eigenvalues[m];
            // 質量ゼロ方向（θ=+∞）が2回連続で現れた場合も「安定した」とみなし、
            // 無限大同士の減算で NaN になって収束判定が永久に false になるのを防ぐ。
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

    // 質量ランク不足チェック: 要求モード数 n_modes に対し、質量が有効な
    // （θ が有限な）方向が n_modes 個に満たない場合、f64::MAX 等を結果に混ぜず
    // 明示エラーとする。gevd_jacobi は質量ゼロ方向の θ を昇順の末尾に +∞ として
    // 返すため、theta_prev の先頭 n_modes 個のうち有限な個数がそのまま
    // （この部分空間内で判定できた）質量ランクになる。
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

    let (participation, effective_mass) =
        compute_participation(&shapes, &m_free, &m_red, reducer, dofmap, model);

    let node_shapes = shapes
        .iter()
        .map(|phi_red| expand_node_shape(phi_red, reducer, dofmap, model.nodes.len()))
        .collect();

    Ok(ModalResult {
        omega2,
        period,
        shapes,
        node_shapes,
        participation,
        effective_mass,
    })
}

/// 縮約座標のモード形状を節点×6成分へ展開する
/// （縮約独立自由度 → `Reducer::expand_u` → DofMap active順 → 節点×6散布）。
/// 静的解析の変位展開（`crate::analysis::Analysis` の `expand_disp`）と同じ経路で、
/// 剛床のスレーブ自由度にはマスターに従属した値が入る。fixed・非構造自由度は 0。
fn expand_node_shape(
    phi_red: &[f64],
    reducer: &Reducer,
    dofmap: &DofMap,
    n_nodes: usize,
) -> Vec<[f64; 6]> {
    let phi_free = reducer.expand_u(phi_red);
    let mut disp = vec![[0.0; 6]; n_nodes];
    for (ni, d6) in disp.iter_mut().enumerate() {
        for (d, slot) in d6.iter_mut().enumerate() {
            let g = ni * squid_n_core::dof::DOF_PER_NODE + d;
            if let Some(active) = dofmap.active(g) {
                *slot = phi_free[active as usize];
            }
        }
    }
    disp
}

/// 部分空間反復の開始ベクトルを Bathe の定石に従って選ぶ。
///
/// 1本目は質量分布に比例した変位パターン（各自由度の集中質量そのもの）。
/// 残り q-1 本は、剛性/質量比 k_ii/m_ii が小さい（＝質量が相対的に効いていて
/// 低次モードに寄与しやすい）自由度から順に単位ベクトルを割り当てる。
/// 質量ゼロの自由度は比を +∞ とみなし、質量を持つ自由度が尽きない限り選ばれない。
/// こうすることで、q が要求モード数程度に小さくても、質量を持つ自由度が
/// 少数・偏在するモデル（例: 水平質点系モデル化）で開始部分空間から
/// 質量を持つ方向が漏れることを防ぐ。
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
    ratios.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    for col in 1..q {
        let dof = ratios[col - 1].0;
        x[dof * q + col] = 1.0;
    }
    x
}

/// 疎行列とベクトルの積 y = A·x（格納済みの非ゼロ要素のみを走査する）。
///
/// `squid_n_math::sparse::sparse_matvec` への薄いラッパ。K/M は要素剛性・
/// 質量行列の組立時点で局所行列の全成分（上下三角とも）を triplet 化して
/// 足し込んでいるため（`squid_n_element::behavior::LocalMat::to_triplets`）、
/// 全体行列は上三角のみ・対称圧縮などではなく非ゼロ要素を対称に両方格納した
/// 「フル対称」形式になっている。したがって `sparse_matvec` の単純な
/// 列走査（`y[row] += val * x[col]`）だけで、かつての `get()` 全ペア走査と
/// 完全に同じ結果が得られる（`assemble_global_k`/`assemble_global_m` と
/// `Reducer::reduce_k` の実装で確認済み。§検証参照）。
fn spmv(mat: &faer::sparse::SparseColMat<usize, f64>, x: &[f64]) -> Vec<f64> {
    squid_n_math::sparse::sparse_matvec(mat, x)
}

/// Y^T·A·Y（q×q 対称行列）を、A の非ゼロ要素だけを使って計算する。
///
/// 従来は (a,b) 全ペアについて `mat.get()`（二分探索）で密に走査していたため
/// O(q²·n²) だった。ここでは A の列ごとの積 Z = A·Y（各列は [`spmv`] で
/// O(nnz)、q 列で O(nnz·q)）を先に求め、続いて Yᵀ·Z（O(n·q²)）を計算する
/// ことで、密行列走査を完全に排除する。結果の対称性は i≤j のみ計算して
/// 対角外を転写することで維持する（数値順序が変わるため最終桁のみ従来と
/// 異なり得るが、固有値解析は収束判定つき反復であり許容される）。
fn proj_yty(
    y: &[f64],
    mat_red: &faer::sparse::SparseColMat<usize, f64>,
    n: usize,
    q: usize,
) -> Vec<f64> {
    // z_cols[j] = A · y_col_j
    let z_cols: Vec<Vec<f64>> = (0..q)
        .map(|j| {
            let col: Vec<f64> = (0..n).map(|r| y[r * q + j]).collect();
            spmv(mat_red, &col)
        })
        .collect();

    let mut result = vec![0.0; q * q];
    for i in 0..q {
        for j in i..q {
            let mut s = 0.0;
            for a in 0..n {
                s += y[a * q + i] * z_cols[j][a];
            }
            result[i * q + j] = s;
            result[j * q + i] = s;
        }
    }
    result
}

/// φᵀ·M·φ を M の非ゼロ要素だけを使って計算する（[`spmv`] 1 回 + 内積）。
/// 従来の O(n²) 密走査を O(nnz) に落とす。
fn m_norm(phi: &[f64], m_red: &faer::sparse::SparseColMat<usize, f64>) -> f64 {
    let m_phi = spmv(m_red, phi);
    phi.iter().zip(m_phi.iter()).map(|(p, mp)| p * mp).sum()
}

/// Generalized eigenvalue problem K*z = θ*M*z（Bathe の一般化 Jacobi 法）。
///
/// M は理論上は半正定値だが、部分空間反復の作業次元 q が実際の質量ランク r を
/// 超える場合（回転自由度など質量を持たない自由度が混在するモデルでは一般的）、
/// 射影質量行列 M̄ は必ずランク落ち（半正定値だが正定値でない）になる。
/// このため Cholesky ベースの標準固有値問題化は使えない。また「M̄ を固有分解して
/// 質量部分空間と質量ゼロ部分空間に分離してから解く」方式は、反復ごとの
/// ランク判定が数値ノイズで揺らぐと「ランク落ち判定→ヌルベクトル注入→
/// K⁻¹ の冪乗反復による最低次モード方向への倒れ込み（平行化）→再ランク落ち」
/// という周期2のリミットサイクルに陥り永久に収束しない（剛床マスターの
/// 並進質量 t と回転慣性 t·mm² のようにスケール差が大きい質量分布で顕在化）。
///
/// そこで Bathe の一般化 Jacobi 法（Bathe, Finite Element Procedures, §11.3.3）で
/// K と M を**反復中のランク判定なしに**同時対角化する。各 2×2 ペアについて
/// 両行列の非対角成分を同時に零化する正則な合同変換 P（対角 1、非対角 α・γ）を
/// 掛ける掃引を収束まで繰り返す。M が半正定値でも常に正則な変換で進むため、
/// 基底の特異化が構造的に起きない。対角化後に θᵢ = k̂ᵢᵢ/m̂ᵢᵢ を読み出し、
/// 質量を持たない方向（m̂ᵢᵢ が相対許容誤差 [`MASS_RANK_REL_TOL`] 未満）には
/// θ=+∞（有限な f64::MAX ではなく明示的な無限大）を割り当て、呼び出し側で
/// 「要求モード数に対して質量ランクが不足している」ことを検出できるようにする。
///
/// 数値スケーリング: 並進質量（t）と回転慣性（t·mm²、並進の 10^6〜10^8 倍）が
/// 混在するモデルでは、m̂ᵢᵢ の大小が「質量の有無」ではなく単位系・基底ベクトルの
/// スケールを反映してしまい、相対許容誤差での質量判定が破綻する（剛床モデルで
/// 質量ランクが過少検出され、解けるはずの要求モード数で InvalidInput になる）。
/// そこで K̄ の対角で両行列を対称スケーリングする: S = diag(1/√k̄ᵢᵢ) とし
/// K̃ = S·K̄·S（単位対角）、M̃ = S·M̄·S。合同変換なので一般化固有値 θ は不変で、
/// 固有ベクトルは z = S·z̃ で戻る。対角化後の m̂ᵢᵢ は各方向の「柔性あたりの質量」
/// （≈1/θ）というスケール不変量になり、質量判定が単位系・基底スケールに
/// 依存しなくなる。
///
/// Returns (eigenvalues ascending, +∞ が質量ゼロ方向; eigenvectors as columns)。
/// 質量を持つ方向の固有ベクトルは M̄ 正規直交（zᵀM̄z = 1）、質量ゼロ方向は
/// 単位ノルムに正規化して返す。
fn gevd_jacobi(k_in: &[f64], m_in: &[f64], n: usize) -> (Vec<f64>, Vec<f64>) {
    // K̄ の対角によるスケーリング係数。K̄ は正定値（縮約後剛性の射影）のため
    // 対角は正のはずだが、数値的な退化に備え非正・非有限なら 1 とする。
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

    // 一般化 Jacobi 掃引: 全ペア (i,j) について K̃・M̃ の (i,j) 成分を同時に
    // 零化する合同変換を、回転が発生しなくなるまで繰り返す。
    // 結合度のしきい値は Bathe の収束判定に倣い「非対角/対角比の2乗」で測る。
    const MAX_SWEEPS: usize = 100;
    const COUPLE_TOL: f64 = 1e-24; // 結合度（比の2乗）のしきい値 = (1e-12)²
    for _sweep in 0..MAX_SWEEPS {
        // M̃ の対角は質量ゼロ方向で 0 になり得るため、比の分母には
        // 対角最大値×質量判定許容誤差を床として使う。
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

                // Bathe の係数: (i,j) 成分を K̃・M̃ 双方で零化する α・γ。
                let a1 = kii * mij - mii * kij;
                let a2 = kjj * mij - mjj * kij;
                let a3 = kii * mjj - kjj * mii;
                // K 正定値・M 半正定値なら理論上 判別式 ≥ 0。数値誤差の負は 0 に丸める。
                let root = ((a3 * 0.5) * (a3 * 0.5) + a1 * a2).max(0.0).sqrt();
                let x = if a3 >= 0.0 {
                    a3 * 0.5 + root
                } else {
                    a3 * 0.5 - root
                };
                let (alpha, gamma) = if x.abs() > f64::MIN_POSITIVE && (a1 != 0.0 || a2 != 0.0) {
                    (a2 / x, -a1 / x)
                } else if kjj.abs() > f64::MIN_POSITIVE {
                    // 退化ペア（例: 両方向とも質量ゼロで M̃ 成分がすべて 0）は
                    // K̃ 側だけをガウス消去式に零化する。
                    (-kij / kjj, 0.0)
                } else {
                    (0.0, 0.0)
                };
                if alpha == 0.0 && gamma == 0.0 {
                    continue;
                }
                rotated = true;

                // 合同変換 A ← PᵀAP、P = I + α·eᵢeⱼᵀ + γ·eⱼeᵢᵀ。
                // 列更新（A·P）: colᵢ += γ·colⱼ、colⱼ += α·colᵢ(旧)。
                // 続く行更新（Pᵀ·A）も同様。旧値を使うため両成分を同時に読む。
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

    // 対角から固有値を読み出す。質量判定はスケール不変な m̂ᵢᵢ（≈1/θ）の
    // 相対値で行い、質量を持たない方向は θ=+∞ とする。
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

    // 正規化とスケーリングの逆変換（z = S·z̃）。質量を持つ列は M̃ 正規直交
    // （z̃ᵀM̃z̃ = 1、合同変換なので逆変換後も zᵀM̄z = 1）にそろえる。
    // 質量ゼロ方向は M̄ ノルムが定義できないため、逆変換後に単位ノルムへ
    // そろえて基底の有界性を保つ（固有ベクトルの定数倍は部分空間反復の
    // 張る空間・収束判定 θ のいずれにも影響しない）。
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
    idx.sort_by(|&a, &b| vals[a].partial_cmp(&vals[b]).unwrap());

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

fn compute_participation(
    shapes: &[Vec<f64>],
    m_free: &faer::sparse::SparseColMat<usize, f64>,
    m_red: &faer::sparse::SparseColMat<usize, f64>,
    reducer: &Reducer,
    dofmap: &DofMap,
    model: &Model,
) -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
    let n_modes = shapes.len();
    let mut participation = vec![[0.0; 3]; n_modes];
    let mut effective_mass = vec![[0.0; 3]; n_modes];

    let n_free = dofmap.n_active();
    let n_nodes = model.nodes.len();

    for dir_idx in 0..3 {
        let mut r_free = vec![0.0; n_free];
        for ni in 0..n_nodes {
            let g = ni * squid_n_core::dof::DOF_PER_NODE + dir_idx;
            if let Some(active) = dofmap.active(g) {
                r_free[active as usize] = 1.0;
            }
        }

        for (m_idx, phi_red) in shapes.iter().enumerate() {
            let phi_free = reducer.expand_u(phi_red);

            // 従来は (a,b) 全ペアを `mat.get()` で密走査していた（モード×3方向で
            // O(n_free²)）。M の非ゼロ要素のみを使う spmv に置き換え O(nnz) にする。
            let m_phi = spmv(m_free, &phi_free);

            let mut phi_m_phi = 0.0;
            for a in 0..n_free {
                phi_m_phi += phi_free[a] * m_phi[a];
            }

            let mut phi_m_r = 0.0;
            for a in 0..n_free {
                phi_m_r += m_phi[a] * r_free[a];
            }

            if phi_m_phi.abs() > 1e-30 {
                participation[m_idx][dir_idx] = phi_m_r / phi_m_phi;
                effective_mass[m_idx][dir_idx] = phi_m_r * phi_m_r / phi_m_phi;
            }
        }
    }

    let _ = m_red;
    (participation, effective_mass)
}

#[cfg(test)]
mod tests;
