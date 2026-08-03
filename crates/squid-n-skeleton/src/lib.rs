//! 部材スケルトン曲線（トリリニア）の算定。
//!
//! RC 部材のファイバ断面から M–φ（モーメント–曲率）を数値積分し、反曲点比・
//! 塑性ヒンジ・せん断変形・鉄筋抜出しを考慮して M–θ（モーメント–部材角）の
//! トリリニアスケルトンと武田履歴則を構築する（仕様書 §7）。
//!
//! # 適用範囲（M–φ エンジンの使い分け）
//!
//! ワークスペースには M–φ を扱う実装が 3 系統あり、役割が異なる:
//! - **本クレート**: RC 断面の材料構成則（ひび割れ・降伏・圧壊イベント）による
//!   トリリニア骨格の**参照実装**。本番の解析経路では使われず、略算式
//!   （`squid_n_core::rc_capacity`）との突合テスト（V&V）が主な利用先。
//! - `squid_n_section::mn_surface::m_phi`: 弾完全塑性ファイバの M–φ/M–θ。
//!   **GUI のヒンジ詳細表示専用**（`squid-n-app::mn_view`）。
//! - `squid_n_element::frame::fiber`: 非線形解析本体のファイバ定式化
//!   （解析結果を決めるのはこちら）。
//!
//! 3 者は塑性化域の集約規則が異なるため、同じ部材でも骨格は一致しない。
//! 表示・参照実装の骨格を解析結果の検証に流用しないこと。
//!
//! モジュール構成（責務分離）:
//! - [`types`]: データ型（[`MemberSkeleton`] / [`Reinforcement`] など）。
//! - [`fiber_model`]: RC ファイバ断面の生成と M–φ 数値積分（内部）。
//! - [`deformation`]: M–φ → M–θ 変換とせん断・抜出し寄与。
//! - [`builder`]: 公開ビルダ [`build_rc_member_skeleton`] / [`build_member_skeleton`]。

mod builder;
mod deformation;
mod fiber_model;
mod types;

pub use builder::{build_member_skeleton, build_rc_member_skeleton};
pub use deformation::{PulloutContribution, ShearContribution};
pub use types::{AxialInteraction, MemberData, MemberSkeleton, Reinforcement, SkeletonOptions};

#[cfg(test)]
mod tests;
