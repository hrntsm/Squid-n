//! 設計関連アクション（許容応力度検定・終局検定・保有水平耐力・床内検定）。
//!
//! `actions` モジュールからの構造分割。アルゴリズムの統合は行わず、
//! `impl App` の設計入口メソッドをここに移しただけである。

mod check;
mod holding;
mod period;
mod ultimate;
