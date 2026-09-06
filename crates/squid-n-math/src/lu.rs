use crate::solver::{LinearSolver, SolveError, SparsityPattern};
use faer::sparse::linalg::solvers::{Lu, SymbolicLu};
use faer::sparse::SparseColMat;
use std::sync::Mutex;

/// 疎 LU 直接法ソルバ。対称正定値でない系にも使える。
/// 直前と同じスパースパターンであれば symbolic 分解を再利用する。
pub struct LuSolver {
    factor: Option<Lu<usize, f64>>,
    symbolic_cache: Option<(SymbolicLu<usize>, SparsityPattern)>,
    n: usize,
    scratch: Mutex<faer::Mat<f64>>,
}

impl Default for LuSolver {
    fn default() -> Self {
        Self {
            factor: None,
            symbolic_cache: None,
            n: 0,
            scratch: Mutex::new(faer::Mat::new()),
        }
    }
}

impl LinearSolver for LuSolver {
    fn factorize(&mut self, k: &SparseColMat<usize, f64>) -> Result<(), SolveError> {
        self.n = k.nrows();
        let symbolic = match &self.symbolic_cache {
            Some((sym, cached_pattern)) if cached_pattern.matches(k) => sym.clone(),
            _ => {
                let sym = SymbolicLu::try_new(k.symbolic())
                    .map_err(|e| SolveError::Backend(format!("symbolic: {e:?}")))?;
                self.symbolic_cache = Some((sym.clone(), SparsityPattern::of(k)));
                sym
            }
        };
        let lu = Lu::try_new_with_symbolic(symbolic, k.as_ref())
            .map_err(|e| SolveError::Backend(format!("LU factorize: {e:?}")))?;
        self.factor = Some(lu);
        Ok(())
    }

    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolveError> {
        let lu = self.factor.as_ref().ok_or(SolveError::NotFactorized)?;
        crate::solver::solve_dense_column(lu, rhs, self.n)
    }

    fn solve_into(&self, rhs: &[f64], out: &mut Vec<f64>) -> Result<(), SolveError> {
        let lu = self.factor.as_ref().ok_or(SolveError::NotFactorized)?;
        let mut scratch = self.scratch.lock().expect("スクラッチのロックに失敗");
        crate::solver::solve_dense_column_into(lu, rhs, self.n, &mut scratch, out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sparse::{assemble_csc, Triplet};

    #[test]
    fn test_lu_2dof_spring() {
        faer::set_global_parallelism(faer::Par::Seq);
        let k = assemble_csc(
            2,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 300.0,
                },
                Triplet {
                    row: 1,
                    col: 0,
                    val: -200.0,
                },
                Triplet {
                    row: 0,
                    col: 1,
                    val: -200.0,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: 200.0,
                },
            ],
        );
        let mut solver = LuSolver::default();
        solver.factorize(&k).unwrap();
        let x = solver.solve(&[0.0, 1000.0]).unwrap();
        approx::assert_relative_eq!(x[0], 10.0, max_relative = 1e-9);
        approx::assert_relative_eq!(x[1], 15.0, max_relative = 1e-9);
    }

    /// 非対称行列も解ける（Cholesky では対象外のケース）。
    #[test]
    fn test_lu_unsymmetric() {
        faer::set_global_parallelism(faer::Par::Seq);
        let k = assemble_csc(
            2,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 2.0,
                },
                Triplet {
                    row: 0,
                    col: 1,
                    val: 1.0,
                },
                Triplet {
                    row: 1,
                    col: 0,
                    val: 0.5,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: 3.0,
                },
            ],
        );
        let mut solver = LuSolver::default();
        solver.factorize(&k).unwrap();
        // [2 1; 0.5 3] x = [4; 6.5] -> x = [1; 2]
        let x = solver.solve(&[4.0, 6.5]).unwrap();
        approx::assert_relative_eq!(x[0], 1.0, max_relative = 1e-9);
        approx::assert_relative_eq!(x[1], 2.0, max_relative = 1e-9);
    }

    #[test]
    fn test_lu_not_factorized() {
        let solver = LuSolver::default();
        assert!(matches!(
            solver.solve(&[1.0]),
            Err(SolveError::NotFactorized)
        ));
    }

    /// 非対称だがスパースパターンが同一な 2 つの行列。symbolic キャッシュの再利用
    /// を検証するために使う（第 2 引数は非対角成分の重みに用いる）。
    fn unsymmetric_3dof(a: f64, b: f64) -> SparseColMat<usize, f64> {
        assemble_csc(
            3,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 5.0,
                },
                Triplet {
                    row: 0,
                    col: 1,
                    val: a,
                },
                Triplet {
                    row: 1,
                    col: 0,
                    val: a * 0.5,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: 6.0,
                },
                Triplet {
                    row: 1,
                    col: 2,
                    val: b,
                },
                Triplet {
                    row: 2,
                    col: 1,
                    val: b * 0.3,
                },
                Triplet {
                    row: 2,
                    col: 2,
                    val: 7.0,
                },
            ],
        )
    }

    /// symbolic キャッシュの本体テスト（[`crate::cholesky`] の同名テストと対）。
    /// 同一パターンで 2 回目の `factorize`（値は変わる）を行った結果が、その値で
    /// 毎回新規に symbolic 分解した場合とビット一致すること。
    #[test]
    fn test_reused_symbolic_matches_fresh_bit_exact() {
        faer::set_global_parallelism(faer::Par::Seq);
        let k1 = unsymmetric_3dof(1.0, 2.0);
        let k2 = unsymmetric_3dof(-0.7, 3.3);
        let rhs = [1.0, -2.0, 3.5];

        let mut reused = LuSolver::default();
        reused.factorize(&k1).unwrap();
        reused.factorize(&k2).unwrap();
        let x_reused = reused.solve(&rhs).unwrap();

        let mut fresh = LuSolver::default();
        fresh.factorize(&k2).unwrap();
        let x_fresh = fresh.solve(&rhs).unwrap();

        assert_eq!(
            x_reused, x_fresh,
            "symbolic 再利用と毎回新規構築でビット不一致"
        );

        for _ in 0..100 {
            reused.factorize(&k2).unwrap();
            let x = reused.solve(&rhs).unwrap();
            assert_eq!(x, x_fresh);
        }
    }

    #[test]
    fn test_solve_into_matches_solve() {
        faer::set_global_parallelism(faer::Par::Seq);
        let k = unsymmetric_3dof(1.0, 2.0);
        let mut solver = LuSolver::default();
        solver.factorize(&k).unwrap();
        let rhs = [1.0, -2.0, 3.5];

        let expected = solver.solve(&rhs).unwrap();
        let mut out = Vec::new();
        solver.solve_into(&rhs, &mut out).unwrap();
        assert_eq!(expected, out);
    }
}
