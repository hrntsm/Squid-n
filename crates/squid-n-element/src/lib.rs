#![allow(clippy::needless_range_loop)]

//! 要素（エレメント）クレート。
//!
//! - [`behavior`] —  要素の振る舞いトレイトと局所行列・ベクトル・状態の型
//! - [`transform`] — 要素ローカル座標系（`LocalFrame`）
//! - [`frame`] —     線材要素（梁・トラス・材端集中ばね梁・ファイバー・マルチスプリング・部材荷重）
//! - [`springs`] —   ばね・パネル要素
//! - [`wall`] —      壁要素
//! - [`shell`] —     シェル要素
//! - [`factory`] —   要素データから振る舞いを生成するディスパッチャ
//!
//! このほかクレート内部専用の `linalg`（小行列の逆行列）を持つ。
//!
//! モジュールパスは階層をそのまま辿る（例: `squid_n_element::frame::beam`）。
//! 再エクスポートによる平坦なパスは設けない。

pub mod behavior;
pub mod factory;
pub mod frame;
pub(crate) mod linalg;
pub mod shell;
pub mod springs;
pub mod transform;
pub mod wall;
