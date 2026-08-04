//! 要素ループの並列/逐次足場（全解析経路共通）。
//!
//! 要素ごとの計算は要素間にデータ依存が無いため、並列度設定
//! （`squid_n_math::parallelism`）が `Auto`/`Threads` のときは rayon で並列化する。
//! いずれのヘルパも**結果は逐次実行と完全にビット一致する**:
//!
//! - [`map_behaviors_ordered`] は要素番号順を保った
//!   `IndexedParallelIterator::collect` で集約するため、後段の要素順の畳み込み
//!   （triplet の extend・内力の累積）が逐次実行と同一順序になる。
//! - [`for_each_behavior_mut`] は各要素が自身の `&mut` のみを更新するため、
//!   書き込み先がスレッドスケジューリングに依存しない。
//!
//! `Deterministic`（既定）では従来どおり逐次実行する（時刻歴応答解析高速化・
//! 第2波申し送り 4.1 の設計方針）。かつてはこの分岐足場が組立・内力・trial 更新の
//! 各関数へコピーされており、順序保証の方針が分散していた。

use smallvec::SmallVec;
use squid_n_core::dof::DofMap;
use squid_n_core::model::Model;
use squid_n_element::behavior::{Ctx, ElementBehavior, LocalVec};

/// 要素ごとの読み取り計算 `f(要素番号, behavior)` を全要素へ適用し、
/// **要素番号順**の `Vec` で返す。
pub(crate) fn map_behaviors_ordered<T, F>(behaviors: &[Box<dyn ElementBehavior>], f: F) -> Vec<T>
where
    T: Send,
    F: Fn(usize, &dyn ElementBehavior) -> T + Sync,
{
    if squid_n_math::parallelism::is_parallel() {
        use rayon::prelude::*;
        behaviors
            .par_iter()
            .enumerate()
            .map(|(i, b)| f(i, b.as_ref()))
            .collect()
    } else {
        behaviors
            .iter()
            .enumerate()
            .map(|(i, b)| f(i, b.as_ref()))
            .collect()
    }
}

/// 要素ごとの可変更新 `f(behavior)` を全要素へ適用する。
pub(crate) fn for_each_behavior_mut<F>(behaviors: &mut [Box<dyn ElementBehavior>], f: F)
where
    F: Fn(&mut Box<dyn ElementBehavior>) + Sync + Send,
{
    if squid_n_math::parallelism::is_parallel() {
        use rayon::prelude::*;
        behaviors.par_iter_mut().for_each(f);
    } else {
        behaviors.iter_mut().for_each(f);
    }
}

/// 全自由 DOF 空間の変位増分 `du_free` を各要素の局所自由度へ写像し、
/// トライアル状態として反映する（`update_state(du, commit=false)`。確定・巻き戻しは
/// 呼び出し側の `commit_state`/`revert_state`）。
///
/// プッシュオーバーの全フェーズ（長期載荷・荷重制御・変位制御・弧長法）と
/// 非線形時刻歴の Newton 反復・長期載荷で共有する（かつては両モジュールに
/// 同一実装がコピーされていた）。
pub(crate) fn apply_du_trial(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &mut [Box<dyn ElementBehavior>],
    du_free: &[f64],
) {
    let ctx = Ctx { model };
    for_each_behavior_mut(behaviors, |b| {
        let gdofs = b.global_dofs(dofmap);
        let mut du_elem = LocalVec {
            data: SmallVec::from_elem(0.0, gdofs.len()),
        };
        for (i, &g) in gdofs.iter().enumerate() {
            if g != usize::MAX && g < du_free.len() {
                du_elem.data[i] = du_free[g];
            }
        }
        b.update_state(&du_elem, false, &ctx);
    });
}
