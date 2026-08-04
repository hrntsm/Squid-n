//! Newton 反復の共通足場（反復上限・相対許容誤差・収束判定）。
//!
//! ソルバ内の Newton 反復は解析経路ごとにループ本体が大きく異なる
//! （弧長法の円筒拘束、変位制御の二重求解と λ 決定、動的解析の慣性・減衰項、
//! 組立てキャッシュ・作業バッファの使い回し）ため、ループ骨格は共通化せず、
//! 収束規約（反復上限・相対許容誤差・相対残差判定）だけを本モジュールへ集約する。
//!
//! 収束判定の基準ノルム（分母）は解析種別ごとに物理的意味が異なる
//! （弧長法は λ 依存の外力ノルム、動的解析は長期荷重を除く動的外力ノルム、
//! 静的漸増は目標外力ノルム）ため、その定義は各呼び出し側の責務とし、
//! 本型は「残差ノルム < tol × 基準ノルム」の形だけを共通に提供する。

/// Newton 反復の収束規約（反復上限と相対許容誤差）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NewtonCriteria {
    /// 反復の最大回数。
    pub max_iter: usize,
    /// 相対許容誤差。残差ノルムが `tol ×` 基準ノルムを下回れば収束とする。
    pub tol: f64,
}

impl NewtonCriteria {
    /// 反復上限と相対許容誤差を指定して作る。
    pub const fn new(max_iter: usize, tol: f64) -> Self {
        Self { max_iter, tol }
    }

    /// 相対残差判定。`r_norm < tol × ref_norm` で収束。
    /// 基準ノルム `ref_norm`（外力ノルム等）の定義は呼び出し側の責務。
    pub fn converged(&self, r_norm: f64, ref_norm: f64) -> bool {
        r_norm < self.tol * ref_norm
    }

    /// 反復レンジ `0..max_iter`（`for _iter in criteria.iters()` の形で使う）。
    pub fn iters(&self) -> std::ops::Range<usize> {
        0..self.max_iter
    }
}

/// 静的漸増解析（プッシュオーバーの荷重制御・変位制御、長期載荷）の共通規約。
///
/// 弾性域は 1〜2 回で収束し、上限 50 回は塑性進行時の余裕
/// （ステップ内で接線を組み直す準ニュートン形式のため多めに取る）。
/// 基準ノルムは外力ノルムと 1.0 の大きい方（`f_norm.max(1.0)`）。
pub const STATIC_NEWTON: NewtonCriteria = NewtonCriteria::new(50, 1e-6);

/// L2 ノルム（Newton 反復の残差・基準ノルム算定の共通形）。
pub fn l2_norm(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}
