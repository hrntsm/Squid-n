//! 静的解析。
//!
//! - [`linear`] —   線形静的解析
//! - [`analysis`] — 地震・風の静的荷重生成と解析設定
pub mod analysis;
pub mod linear;

/// 要素ごとの `(ElementBehavior, global_dofs)` のペア。
pub(crate) type BehaviorEntry = (
    Box<dyn squid_n_element::behavior::ElementBehavior>,
    smallvec::SmallVec<[usize; 24]>,
);
