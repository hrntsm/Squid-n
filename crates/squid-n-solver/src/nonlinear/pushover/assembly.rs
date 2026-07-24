//! 剛性行列の組立と内力ベクトルの算定。
//!
//! - [`assemble_k`] — 全体接線剛性行列（幾何剛性・変位制御ペナルティ対応）
//! - [`compute_f_int`] — 全自由節点の内力ベクトル
//!
//! いずれも `dynamic`/`timehistory` モジュールが `crate::pushover::{assemble_k,
//! compute_f_int}` として参照するため `pub(crate)` を維持する。

use squid_n_core::dof::DofMap;
use squid_n_core::model::Model;
use squid_n_element::behavior::{Ctx, ElemState, ElementBehavior};

/// 全体接線剛性行列を組み立てる。
///
/// `prescribed = Some((dof, penalty))` のとき、変位制御（`driver` の変位制御
/// フェーズ）のために対角 `[dof, dof]` へペナルティ剛性 `penalty` を加算する。
/// 従来は penalty を関数内で固定値 `1e16` としていたが、接線剛性のスケール（~1e5〜1e7）
/// に対して過大で、変位制御の残差計算で桁落ち（catastrophic cancellation）を起こし
/// 収束判定が原理的に成立しなかった。呼び出し側（`driver`）が接線剛性スケールに
/// 比例した well-conditioned な penalty を算定して渡す。第2要素は penalty であって
/// 目標変位ではない（目標変位は残差側で扱う）。
pub(crate) fn assemble_k(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    use_kg: bool,
    prescribed: Option<(usize, f64)>,
) -> faer::sparse::SparseColMat<usize, f64> {
    use squid_n_math::sparse::assemble_csc;
    let ctx = Ctx { model };
    let state = ElemState::default();
    let mut triplets = Vec::new();
    for (_elem, b) in model.elements.iter().zip(behaviors) {
        let gdofs = b.global_dofs(dofmap);
        let mut k = b.tangent_stiffness(&state, &ctx);
        if use_kg {
            let f = b.internal_force(&state, &ctx);
            let n = f.data.first().copied().unwrap_or(0.0);
            let kg = b.geometric_stiffness(n);
            for i in 0..12 {
                for j in 0..12 {
                    let sum = k.get(i, j) + kg.get(i, j);
                    k.set(i, j, sum);
                }
            }
        }
        triplets.extend(k.to_triplets(&gdofs));
    }
    if let Some((d, penalty)) = prescribed {
        triplets.push(squid_n_math::sparse::Triplet {
            row: d,
            col: d,
            val: penalty,
        });
    }
    assemble_csc(dofmap.n_active(), triplets)
}

pub(crate) fn compute_f_int(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
) -> Vec<f64> {
    let ctx = Ctx { model };
    let state = ElemState::default();
    let mut f = vec![0.0; dofmap.n_active()];
    for (_elem, b) in model.elements.iter().zip(behaviors) {
        let gdofs = b.global_dofs(dofmap);
        let f_local = b.internal_force(&state, &ctx);
        for (&g, &v) in gdofs.iter().zip(f_local.data.iter()) {
            if g != usize::MAX {
                f[g] += v;
            }
        }
    }
    f
}
