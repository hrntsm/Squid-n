//! 非線形反復の接線剛性行列と内力ベクトルの組み立て。
//!
//! - [`assemble_k`] — 全体接線剛性行列（幾何剛性対応）
//! - [`compute_f_int`] — 全自由節点の内力ベクトル
//! - [`add_support_spring_f_int`] — 支点ばね（`Node::support_spring`）の内力寄与
//!
//! 要素の**現在の状態**（`&[Box<dyn ElementBehavior>]`）から組み立てる経路であり、
//! Newton 反復を回す解析はすべてここを通る（増分解析・弧長法・非線形時刻歴）。
//! モデルから毎回要素を組み直す線形経路は [`super::assemble`] にある。

use crate::common::assemble::support_spring_terms;
use crate::common::csc_cache::CscCache;
use squid_n_core::dof::DofMap;
use squid_n_core::model::Model;
use squid_n_element::behavior::{Ctx, ElementBehavior};

/// 全体接線剛性行列を組み立てる。
pub(crate) fn assemble_k(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    use_kg: bool,
) -> faer::sparse::SparseColMat<usize, f64> {
    use squid_n_math::sparse::assemble_csc;
    let triplets = assemble_k_triplets(model, dofmap, behaviors, use_kg);
    assemble_csc(dofmap.n_active(), triplets)
}

/// [`assemble_k`] のキャッシュ版。結果は常に [`assemble_k`] とビット一致する。
pub(crate) fn assemble_k_cached(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    use_kg: bool,
    cache: &mut CscCache,
) -> faer::sparse::SparseColMat<usize, f64> {
    let triplets = assemble_k_triplets(model, dofmap, behaviors, use_kg);
    cache.assemble(dofmap.n_active(), &triplets)
}

/// [`assemble_k_cached`] の参照返し版。呼び出し側の triplet バッファ `buf`
/// （[`assemble_k_triplets_into`] 経由で `clear()` して再利用、容量は呼び出し間で
/// 維持される）を使い、`cache` が内部保持する行列への参照を返す（`.clone()` を
/// 伴わない）。非線形時刻歴の Newton 反復のように、結果をすぐ読むだけで所有権が
/// 要らない呼び出し元向け（[`CscCache::assemble_ref`] 参照）。結果は常に
/// [`assemble_k`] とビット一致する。
pub(crate) fn assemble_k_cached_ref<'a>(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    use_kg: bool,
    cache: &'a mut CscCache,
    buf: &mut Vec<squid_n_math::sparse::Triplet>,
) -> &'a faer::sparse::SparseColMat<usize, f64> {
    assemble_k_triplets_into(model, dofmap, behaviors, use_kg, buf);
    cache.assemble_ref(dofmap.n_active(), buf)
}

/// [`assemble_k`]・[`assemble_k_cached`] が共有する triplet 列の組立て。
fn assemble_k_triplets(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    use_kg: bool,
) -> Vec<squid_n_math::sparse::Triplet> {
    let mut triplets = Vec::new();
    assemble_k_triplets_into(model, dofmap, behaviors, use_kg, &mut triplets);
    triplets
}

/// [`assemble_k_triplets`] の結果を呼び出し側の既存バッファへ書き込む版
/// （`out` は先頭で `clear()` してから書き込むため、確保済みの容量は保持され、
/// Newton 反復のように毎回呼ぶ場面で再確保が発生しない）。計算内容・順序は
/// [`assemble_k_triplets`] と同一（ビット完全一致）。
fn assemble_k_triplets_into(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    use_kg: bool,
    out: &mut Vec<squid_n_math::sparse::Triplet>,
) {
    out.clear();
    let ctx = Ctx { model };
    let elem_triplets = |elem: &squid_n_core::model::ElementData,
                         b: &dyn ElementBehavior|
     -> Vec<squid_n_math::sparse::Triplet> {
        let gdofs = b.global_dofs(dofmap);
        let mut k = b.tangent_stiffness(&ctx);
        if use_kg {
            let f = b.internal_force(&ctx);
            let n = axial_force_tension_positive(model, elem, &f);
            let kg = b.geometric_stiffness(n);
            for i in 0..12 {
                for j in 0..12 {
                    let sum = k.get(i, j) + kg.get(i, j);
                    k.set(i, j, sum);
                }
            }
        }
        k.to_triplets(&gdofs)
    };
    crate::common::elem_loop::fold_behaviors_ordered(
        behaviors,
        |i, b| match model.elements.get(i) {
            Some(elem) => elem_triplets(elem, b),
            None => Vec::new(),
        },
        |triplets| out.extend(triplets),
    );
    for (active, k) in support_spring_terms(model, dofmap) {
        out.push(squid_n_math::sparse::Triplet {
            row: active,
            col: active,
            val: k,
        });
    }
}

/// 材端力（グローバル成分）から部材軸力 N [N]（**引張正**）を求める。
/// 2 節点未満・退化長さの要素は 0（幾何剛性は既定でゼロ行列）。
fn axial_force_tension_positive(
    model: &Model,
    elem: &squid_n_core::model::ElementData,
    f: &squid_n_element::behavior::LocalVec,
) -> f64 {
    if elem.nodes.len() < 2 || f.data.len() < 12 {
        return 0.0;
    }
    let (Some(pi), Some(pj)) = (
        model.nodes.get(elem.nodes[0].index()),
        model.nodes.get(elem.nodes[1].index()),
    ) else {
        return 0.0;
    };
    let d = [
        pj.coord[0] - pi.coord[0],
        pj.coord[1] - pi.coord[1],
        pj.coord[2] - pi.coord[2],
    ];
    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if len <= 0.0 {
        return 0.0;
    }
    let ex = [d[0] / len, d[1] / len, d[2] / len];
    f.data[6] * ex[0] + f.data[7] * ex[1] + f.data[8] * ex[2]
}

pub(crate) fn compute_f_int(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
) -> Vec<f64> {
    let ctx = Ctx { model };
    let mut f = vec![0.0; dofmap.n_active()];
    crate::common::elem_loop::fold_behaviors_ordered(
        behaviors,
        |_, b| {
            let gdofs = b.global_dofs(dofmap);
            let f_local = b.internal_force(&ctx);
            (gdofs, f_local)
        },
        |(gdofs, f_local)| {
            for (&g, &v) in gdofs.iter().zip(f_local.data.iter()) {
                if g != usize::MAX {
                    f[g] += v;
                }
            }
        },
    );
    f
}

/// 支点ばね（`Node::support_spring`）の内力寄与 `k_i・u_i` を、内力ベクトル `f`
/// （`compute_f_int` と同じ active DOF 順）へ加算する。
/// 試行全体変位 `u_trial`（active DOF 順）を呼び出し側から明示的に渡す。
/// 呼び出し側は `compute_f_int` の結果に本関数の寄与を加算すること。
pub(crate) fn add_support_spring_f_int(
    model: &Model,
    dofmap: &DofMap,
    u_trial: &[f64],
    f: &mut [f64],
) {
    for (active, k) in support_spring_terms(model, dofmap) {
        if let (Some(&u), Some(fa)) = (u_trial.get(active), f.get_mut(active)) {
            *fa += k * u;
        }
    }
}
