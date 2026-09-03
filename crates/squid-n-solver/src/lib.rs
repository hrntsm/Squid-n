#![allow(clippy::needless_range_loop)]
#![allow(clippy::single_match)]
#![allow(clippy::identity_op)]

//! 解析ソルバークレート。
//!
//! 解析の種類ごとにモジュールを階層化している:
//!
//! - [`common`] —    解析共通の基盤（組み立て・拘束・接線剛性・状態スナップショット）
//! - [`statics`] —   静的解析（線形静的・地震/風荷重）
//! - [`nonlinear`] — 非線形（漸増）静的解析（プッシュオーバー・弧長法）
//! - [`dynamic`] —   動的解析（時刻歴・減衰・固有値）
//! - [`damage`] —    損傷指標
//!
//! モジュールパスは階層をそのまま辿る（例: `squid_n_solver::nonlinear::pushover`）。
//! 再エクスポートによる平坦なパスは設けない。

pub mod common;
pub mod damage;
pub mod dynamic;
pub mod nonlinear;
pub mod statics;
