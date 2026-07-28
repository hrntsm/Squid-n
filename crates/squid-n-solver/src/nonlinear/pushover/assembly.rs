//! 剛性行列の組立と内力ベクトルの算定。
//!
//! - [`assemble_k`] — 全体接線剛性行列（幾何剛性対応）
//! - [`compute_f_int`] — 全自由節点の内力ベクトル
//! - [`add_support_spring_f_int`] — 支点ばね（`Node::support_spring`）の内力寄与
//!
//! `assemble_k`・`compute_f_int` は `dynamic`/`timehistory` モジュールが
//! `crate::pushover::{assemble_k, compute_f_int}` として参照するため `pub(crate)` を
//! 維持し、シグネチャは変更しない（支点ばねの内力寄与は別関数
//! [`add_support_spring_f_int`] として呼び出し側 [`super::driver`] が加算する）。

use crate::common::assemble::support_spring_terms;
use squid_n_core::dof::DofMap;
use squid_n_core::model::Model;
use squid_n_element::behavior::{Ctx, ElemState, ElementBehavior};

/// 全体接線剛性行列を組み立てる。
///
/// かつて変位制御ペナルティ用の対角加算引数を持っていたが、変位制御が
/// 「比例荷重パターンを保持した荷重係数決定方式」（`driver` の変位制御フェーズ）へ
/// 移行しペナルティ剛性が不要となったため撤去した。
pub(crate) fn assemble_k(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    use_kg: bool,
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
            // 幾何剛性には**部材軸力 N（引張正）**を渡す。`internal_force` は
            // グローバル成分を返す契約なので、材端力を要素局所 ex へ射影して得る
            // （`geom::axial_compression` と同じ符号規約: dot(f_j, ex) = +N）。
            // 従来は `f.data[0]`（＝節点 i のグローバル Fx）をそのまま渡しており、
            // 鉛直柱・斜材・任意方向材で軸力とは無関係な成分を用いていた
            // （P-Δ を有効化すると誤った幾何剛性になる潜在バグ）。
            let n = axial_force_tension_positive(model, _elem, &f);
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
    // 支点ばね（`Node::support_spring`）の対角加算。線形経路の
    // `assemble_global_k`（`common::assemble`）と同じ [`support_spring_terms`] を使う。
    for (active, k) in support_spring_terms(model, dofmap) {
        triplets.push(squid_n_math::sparse::Triplet {
            row: active,
            col: active,
            val: k,
        });
    }
    assemble_csc(dofmap.n_active(), triplets)
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
    // dot(f_j, ex) = +N（引張正）。
    f.data[6] * ex[0] + f.data[7] * ex[1] + f.data[8] * ex[2]
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

/// 支点ばね（`Node::support_spring`）の内力寄与 `k_i・u_i` を、内力ベクトル `f`
/// （`compute_f_int` と同じ active DOF 順）へ加算する。
///
/// `compute_f_int` は要素（`ElementBehavior`）が自ら保持するトライアル変位から
/// 内力を求める契約だが、支点ばねは要素を介さない節点属性のため、現在の
/// 試行全体変位 `u_trial`（active DOF 順、Newton 反復途中の未確定値でよい）を
/// 呼び出し側（[`super::driver`]）から明示的に渡す必要がある。線形ばね
/// （K が変位に依存しない）のため接線剛性側は `assemble_k` の対角加算のみで足りるが、
/// 内力側はこの関数で `k・u` を明示的に計上しないと、支点ばねに変位が生じても
/// 釣合い残差 `f_ext − f_int` に反映されず、非線形解析で支点ばねが実質無視される
/// （K だけに効いて残差計算に効かない不整合）。呼び出し側は `compute_f_int` の
/// 結果に本関数の寄与を加算すること。
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
