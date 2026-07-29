use faer::sparse::SparseColMat;

/// 疎行列ソルバの共通インタフェース。
/// 荷重ケース並列（`squid-n-solver` の batch API）で分解済みソルバを
/// スレッド間共有するため `Send + Sync` を要求する。
pub trait LinearSolver: Send + Sync {
    fn factorize(&mut self, k: &SparseColMat<usize, f64>) -> Result<(), SolveError>;
    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolveError>;

    /// [`Self::solve`] のバッファ再利用版。解を `out` に書き込む（`out` の長さが
    /// 解の次元と異なる場合のみ `resize` するため、時刻歴のようにステップ毎に同じ
    /// `out` を渡す使い方ではベクタ確保が起きない）。
    ///
    /// 既定実装は `solve` を呼んで結果をコピーするだけで、確保は削減されない。
    /// `CholeskySolver`／`LuSolver` は内部スクラッチを再利用する実装を持つ。
    fn solve_into(&self, rhs: &[f64], out: &mut Vec<f64>) -> Result<(), SolveError> {
        *out = self.solve(rhs)?;
        Ok(())
    }
}

#[derive(Debug, thiserror::Error)]
pub enum SolveError {
    #[error("not factorized yet")]
    NotFactorized,
    #[error("matrix not positive definite")]
    NotPositiveDefinite,
    #[error("dimension mismatch: K={k}, rhs={rhs}")]
    DimMismatch { k: usize, rhs: usize },
    #[error("faer error: {0}")]
    Backend(String),
    /// 入力モデル起因のエラー（拘束不足・断面/材料未割当など）。
    /// メッセージはユーザー向け診断文（日本語）を想定する。
    #[error("{0}")]
    InvalidInput(String),
    /// 反復法・固有値解析が規定回数内に収束しなかった。
    #[error("収束しませんでした: {0}")]
    NonConvergence(String),
}

#[derive(Clone, Copy, Debug)]
pub enum SolverBackend {
    DirectSparseCholesky,
    DirectSparseLu,
    IterativePcg {
        tol: f64,
        max_iter: usize,
    },
    /// 自由度数に応じて疎 Cholesky / PCG を自動選択する（対称正定値系向け）。
    /// PCG が収束しない場合は疎 Cholesky へ自動フォールバックする。
    Auto,
}

/// 因子分解済みソルバに単一 RHS 列を与えて解く共通処理。
/// `CholeskySolver`／`LuSolver` の `solve` 実装が共有する（次元検査→RHS 構築→解→収集）。
pub(crate) fn solve_dense_column<S: faer::linalg::solvers::Solve<f64>>(
    factor: &S,
    rhs: &[f64],
    n: usize,
) -> Result<Vec<f64>, SolveError> {
    if rhs.len() != n {
        return Err(SolveError::DimMismatch {
            k: n,
            rhs: rhs.len(),
        });
    }
    let b = faer::Mat::from_fn(n, 1, |i, _| rhs[i]);
    let x = factor.solve(b.as_ref());
    Ok((0..n).map(|i| x[(i, 0)]).collect())
}

/// [`solve_dense_column`] のバッファ再利用版。RHS/解の保持に使う `n×1` の `Mat`
/// スクラッチ（`scratch`）と出力 `Vec`（`out`）を呼び出し側から受け取り、サイズが
/// 変わらない限り再確保しない。`CholeskySolver`／`LuSolver` の `solve_into` が使う。
pub(crate) fn solve_dense_column_into<S: faer::linalg::solvers::Solve<f64>>(
    factor: &S,
    rhs: &[f64],
    n: usize,
    scratch: &mut faer::Mat<f64>,
    out: &mut Vec<f64>,
) -> Result<(), SolveError> {
    if rhs.len() != n {
        return Err(SolveError::DimMismatch {
            k: n,
            rhs: rhs.len(),
        });
    }
    if scratch.nrows() != n || scratch.ncols() != 1 {
        *scratch = faer::Mat::zeros(n, 1);
    }
    for i in 0..n {
        scratch[(i, 0)] = rhs[i];
    }
    factor.solve_in_place(scratch.as_mut());
    if out.len() != n {
        out.resize(n, 0.0);
    }
    for i in 0..n {
        out[i] = scratch[(i, 0)];
    }
    Ok(())
}

/// スパースパターン（列ポインタ・行添字）の不変シグネチャ。値は含めない。
/// symbolic 分解のキャッシュ有効性判定（`CholeskySolver`／`LuSolver`）に使う。
#[derive(Clone, PartialEq, Eq)]
pub(crate) struct SparsityPattern {
    col_ptr: Vec<usize>,
    row_idx: Vec<usize>,
}

impl SparsityPattern {
    pub(crate) fn of(k: &SparseColMat<usize, f64>) -> Self {
        let sym = k.symbolic();
        Self {
            col_ptr: sym.col_ptr().to_vec(),
            row_idx: sym.row_idx().to_vec(),
        }
    }
}

pub fn make_solver(backend: SolverBackend) -> Box<dyn LinearSolver> {
    match backend {
        SolverBackend::DirectSparseCholesky => Box::new(crate::cholesky::CholeskySolver::default()),
        SolverBackend::IterativePcg { tol, max_iter } => {
            Box::new(crate::pcg::PcgSolver::new(tol, max_iter))
        }
        SolverBackend::DirectSparseLu => Box::new(crate::lu::LuSolver::default()),
        SolverBackend::Auto => Box::new(crate::auto::AutoSolver::default()),
    }
}
