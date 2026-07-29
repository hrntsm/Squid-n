use crate::solver::{LinearSolver, SolveError, SparsityPattern};
use faer::sparse::linalg::solvers::{Llt, SymbolicLlt};
use faer::sparse::SparseColMat;
use faer::Side;
use std::sync::Mutex;

/// 疎 Cholesky（LLᵀ）直接法ソルバ。
///
/// `factorize` は symbolic 分解（AMD 順序付け＋elimination tree）をキャッシュし、
/// 直前と同じスパースパターン（`col_ptr`／`row_idx` が一致）であれば再利用して
/// 数値分解のみを行う。時刻歴解析の Newton 反復のように、接続関係（＝スパース
/// パターン）は変わらず係数値のみが変わる再分解を繰り返す場面で有効。
/// パターンが変わった場合は自動的に symbolic を作り直すため、呼び出し側の
/// `factorize` 呼び出し方は従来と変わらない。
pub struct CholeskySolver {
    factor: Option<Llt<usize, f64>>,
    /// キャッシュ済み symbolic 分解と、その構築に使ったパターン。
    /// `SymbolicLlt` は `Arc` 内包で `Clone` が安価。
    symbolic_cache: Option<(SymbolicLlt<usize>, SparsityPattern)>,
    n: usize,
    /// `solve_into` が使い回す RHS/解のスクラッチ（次元不変ならベクタ確保が起きない）。
    /// `LinearSolver` は `Send + Sync` を要求するため `Mutex` で内部可変性を持たせる
    /// （`&self` の `solve_into` から書き換えるため）。
    scratch: Mutex<faer::Mat<f64>>,
}

impl Default for CholeskySolver {
    fn default() -> Self {
        Self {
            factor: None,
            symbolic_cache: None,
            n: 0,
            scratch: Mutex::new(faer::Mat::new()),
        }
    }
}

impl LinearSolver for CholeskySolver {
    fn factorize(&mut self, k: &SparseColMat<usize, f64>) -> Result<(), SolveError> {
        self.n = k.nrows();
        // ヒット時（パターン不変）は比較用 Vec を確保しない（`matches` はスライス比較のみ）。
        let symbolic = match &self.symbolic_cache {
            Some((sym, cached_pattern)) if cached_pattern.matches(k) => sym.clone(),
            _ => {
                let sym = SymbolicLlt::try_new(k.symbolic(), Side::Lower)
                    .map_err(|e| SolveError::Backend(format!("symbolic: {e:?}")))?;
                self.symbolic_cache = Some((sym.clone(), SparsityPattern::of(k)));
                sym
            }
        };
        let llt = Llt::try_new_with_symbolic(symbolic, k.as_ref(), Side::Lower)
            .map_err(|_| SolveError::NotPositiveDefinite)?;
        self.factor = Some(llt);
        Ok(())
    }

    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolveError> {
        let llt = self.factor.as_ref().ok_or(SolveError::NotFactorized)?;
        crate::solver::solve_dense_column(llt, rhs, self.n)
    }

    fn solve_into(&self, rhs: &[f64], out: &mut Vec<f64>) -> Result<(), SolveError> {
        let llt = self.factor.as_ref().ok_or(SolveError::NotFactorized)?;
        let mut scratch = self.scratch.lock().expect("スクラッチのロックに失敗");
        crate::solver::solve_dense_column_into(llt, rhs, self.n, &mut scratch, out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solver::{make_solver, SolverBackend};
    use crate::sparse::{assemble_csc, Triplet};

    #[test]
    fn test_2dof_spring() {
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
        let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
        solver.factorize(&k).unwrap();
        let x = solver.solve(&[0.0, 1000.0]).unwrap();
        approx::assert_relative_eq!(x[0], 10.0, max_relative = 1e-9);
        approx::assert_relative_eq!(x[1], 15.0, max_relative = 1e-9);
    }

    #[test]
    fn test_2dof_deterministic() {
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
        let mut solver0 = make_solver(SolverBackend::DirectSparseCholesky);
        solver0.factorize(&k).unwrap();
        let x0 = solver0.solve(&[0.0, 1000.0]).unwrap();
        for _ in 0..100 {
            let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
            solver.factorize(&k).unwrap();
            let x = solver.solve(&[0.0, 1000.0]).unwrap();
            assert_eq!(x, x0);
        }
    }

    #[test]
    fn test_not_factorized() {
        let solver = CholeskySolver::default();
        let result = solver.solve(&[1.0, 2.0]);
        assert!(matches!(result, Err(SolveError::NotFactorized)));
    }

    #[test]
    fn test_dim_mismatch() {
        let k = assemble_csc(
            2,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 1.0,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: 1.0,
                },
            ],
        );
        let mut solver = CholeskySolver::default();
        solver.factorize(&k).unwrap();
        let result = solver.solve(&[1.0]);
        assert!(matches!(result, Err(SolveError::DimMismatch { .. })));
    }

    /// 3 自由度のバネ連成系（三重対角パターン）。行列の値だけを差し替えて
    /// symbolic キャッシュの再利用を検証するために使う。
    fn spring_chain_3dof(k12: f64, k23: f64) -> SparseColMat<usize, f64> {
        assemble_csc(
            3,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 300.0,
                },
                Triplet {
                    row: 1,
                    col: 0,
                    val: -k12,
                },
                Triplet {
                    row: 0,
                    col: 1,
                    val: -k12,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: k12 + k23,
                },
                Triplet {
                    row: 2,
                    col: 1,
                    val: -k23,
                },
                Triplet {
                    row: 1,
                    col: 2,
                    val: -k23,
                },
                Triplet {
                    row: 2,
                    col: 2,
                    val: k23 + 50.0,
                },
            ],
        )
    }

    /// symbolic キャッシュの本体テスト: 同一スパースパターンで 2 回目の
    /// `factorize`（値は変わる）が、そのパターンを毎回新規に symbolic 分解した
    /// 場合とビット一致すること。AMD 順序付けを使い回しても数値結果が変わらない
    /// ことの検証（決定性の要件）。
    #[test]
    fn test_reused_symbolic_matches_fresh_bit_exact() {
        faer::set_global_parallelism(faer::Par::Seq);
        let k1 = spring_chain_3dof(200.0, 100.0);
        let k2 = spring_chain_3dof(80.0, 260.0);
        let rhs = [1.0, -2.0, 3.5];

        // 1 回目で symbolic を構築し、2 回目（同一パターン・別の値）で再利用させる。
        let mut reused = CholeskySolver::default();
        reused.factorize(&k1).unwrap();
        reused.factorize(&k2).unwrap();
        let x_reused = reused.solve(&rhs).unwrap();

        // 比較対象: k2 のみで新規に factorize したソルバ（symbolic を毎回作り直す）。
        let mut fresh = CholeskySolver::default();
        fresh.factorize(&k2).unwrap();
        let x_fresh = fresh.solve(&rhs).unwrap();

        assert_eq!(
            x_reused, x_fresh,
            "symbolic 再利用と毎回新規構築でビット不一致"
        );

        // 100 回 factorize を繰り返しても毎回ビット一致すること（既存の決定性テストと同水準）。
        for _ in 0..100 {
            reused.factorize(&k2).unwrap();
            let x = reused.solve(&rhs).unwrap();
            assert_eq!(x, x_fresh);
        }
    }

    /// スパースパターンが変わった場合は symbolic を自動的に作り直し、
    /// 引き続き正しい解が得られること（キャッシュ無効化のフォールバック確認）。
    #[test]
    fn test_pattern_change_recomputes_symbolic() {
        faer::set_global_parallelism(faer::Par::Seq);
        let k_tridiag = spring_chain_3dof(200.0, 100.0);
        // (0,2)/(2,0) を追加した別パターン（正定値を保つ対角優位な値）。
        let k_dense = assemble_csc(
            3,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 400.0,
                },
                Triplet {
                    row: 1,
                    col: 0,
                    val: -100.0,
                },
                Triplet {
                    row: 0,
                    col: 1,
                    val: -100.0,
                },
                Triplet {
                    row: 2,
                    col: 0,
                    val: -50.0,
                },
                Triplet {
                    row: 0,
                    col: 2,
                    val: -50.0,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: 300.0,
                },
                Triplet {
                    row: 2,
                    col: 1,
                    val: -30.0,
                },
                Triplet {
                    row: 1,
                    col: 2,
                    val: -30.0,
                },
                Triplet {
                    row: 2,
                    col: 2,
                    val: 200.0,
                },
            ],
        );
        let rhs = [10.0, 0.0, 0.0];

        let mut solver = CholeskySolver::default();
        solver.factorize(&k_tridiag).unwrap();
        let _ = solver.solve(&rhs).unwrap();
        // パターンが変わる（(0,2)/(2,0) が追加される）→ symbolic を作り直すはず。
        solver.factorize(&k_dense).unwrap();
        let x = solver.solve(&rhs).unwrap();

        let mut fresh = CholeskySolver::default();
        fresh.factorize(&k_dense).unwrap();
        let x_fresh = fresh.solve(&rhs).unwrap();
        assert_eq!(x, x_fresh);
    }

    #[test]
    fn test_solve_into_matches_solve() {
        faer::set_global_parallelism(faer::Par::Seq);
        let k = spring_chain_3dof(200.0, 100.0);
        let mut solver = CholeskySolver::default();
        solver.factorize(&k).unwrap();
        let rhs = [1.0, -2.0, 3.5];

        let expected = solver.solve(&rhs).unwrap();
        let mut out = Vec::new();
        solver.solve_into(&rhs, &mut out).unwrap();
        assert_eq!(expected, out);

        // out を使い回しても（別の長さから始めても）同じ結果になること。
        let mut out2 = vec![f64::NAN; 1];
        solver.solve_into(&rhs, &mut out2).unwrap();
        assert_eq!(expected, out2);
    }
}
