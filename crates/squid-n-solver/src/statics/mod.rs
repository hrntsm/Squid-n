//! 静的解析。
//!
//! - [`linear`] —       線形静的解析
//! - [`analysis`] —     地震・風の静的荷重生成と解析設定
//! - [`construction`] — 施工時解析（施工段階解析）
pub mod analysis;
pub mod construction;
pub mod linear;

/// 要素ごとの `(ElementBehavior, global_dofs)` のペア。`build_behavior` を1回だけ
/// 呼んで K 組立・内力回収の双方で使い回すキャッシュの要素型（`analysis::mod`・
/// `linear::mod` で共有）。clippy の type_complexity 回避を兼ねる。
pub(crate) type BehaviorEntry = (
    Box<dyn squid_n_element::behavior::ElementBehavior>,
    smallvec::SmallVec<[usize; 24]>,
);
