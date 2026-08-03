use faer::sparse::SparseColMat;
use squid_n_math::solver::{LinearSolver, SolveError};

use crate::spmv;

pub struct PcgGpu {
    inner: Option<spmv::CpuSpMv>,
    tol: f64,
    max_iter: usize,
    n: usize,
}

impl PcgGpu {
    pub fn new(_ctx: &super::GpuContext, tol: f64, max_iter: usize) -> Self {
        Self {
            inner: None,
            tol,
            max_iter,
            n: 0,
        }
    }
}

impl LinearSolver for PcgGpu {
    fn factorize(&mut self, k: &SparseColMat<usize, f64>) -> Result<(), SolveError> {
        use squid_n_math::pcg::PcgSolver;
        let mut cpu = PcgSolver::new(self.tol, self.max_iter);
        cpu.factorize(k)?;
        self.n = k.nrows();
        self.inner = Some(spmv::CpuSpMv::from_csc(k));
        Ok(())
    }

    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolveError> {
        let _spmv = self.inner.as_ref().ok_or(SolveError::NotFactorized)?;
        let _ = rhs;
        // GPU PCG（T1: SpMV カーネル）は未実装。かつては要素ゼロの n×n 行列を
        // CPU PCG へ分解させて解いており、エラーも panic も出さずに誤った変位を
        // 返す構造だった（make_solver 相当の経路へ差し込むと静かに誤答する）。
        // 実装されるまで明示エラーで停止する。
        Err(SolveError::Backend(
            "GPU PCG は未実装です（gpu フィーチャは開発中）。CPU ソルバを使用してください。".into(),
        ))
    }
}
