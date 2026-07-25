//! 階(Story)の自動生成。
//!
//! 節点の標高(Z)をクラスタリングして階を推定し、各階に剛床(ダイアフラム)と
//! 地震重量を設定する。地震静的解析(Ai分布)・プッシュオーバー・偏心率計算の
//! 前提データを 1 操作で用意するための機能。
//!
//! 重量は「自重(線材: ρ·A·L·g、壁・シェル: ρ·t·A·g) + 指定荷重ケースの
//! 鉛直下向き荷重」を節点に配分し、階ごとに合計する簡易法(節点支配)による。
//! 自重は左右対称な等分布荷重なので両端 1/2 ずつ、指定荷重ケースの部材荷重は
//! 単純支持梁の静定反力（`static_reactions`）で両端に配分する（令88条の地震用重量
//! 算定における CMoQo による梁せん断力 Q0 の実務的取扱いに相当。対称荷重では結果的に
//! 自重と同じ 1/2-1/2 になる）。
//!
//! 剛床代表節点は、剛床に含まれる節点の慣性力重心（重量重み付き重心）に
//! 専用の仮想節点として自動生成する（既存節点の流用ではない）。
//! 剛床（剛体ダイアフラム）の取扱いは構造力学（剛体運動の縮約）による。
//! 並進慣性重量は ΣiW、回転慣性重量は ΣiW·ir² となり、スレーブ節点の面内応答は
//! `crates/squid-n-solver/src/constraint.rs` の RigidDiaphragm 縮約で
//! ix = Gx − iry·Gθz, iy = Gy + irx·Gθz として復元される。
//!
//! 代表節点の `mass`（[`squid_n_core::model::Node::mass`]）は質量方式
//! （[`MassMethod`]、`Model::mass_method`）に従って設定する。
//! `CorrectedLumped`（既定）は、要素・節点側に残った分布質量が Reducer の
//! TᵀMT 縮約（`eigen.rs`）で自動的にマスターへ集約されることを踏まえ、
//! 代表節点へは「地震用重量のうち分布質量として計上されない分」
//! （主架構線材・壁パネル以外＝床・仕上げ・積載・二次部材・雑壁など）を
//! 補正質点として与える（二重計上を避ける）。`LumpedOnly` は分布質量を
//! 質量行列に算入しないため、代表節点へ地震用重量の全量を与える。
//!
//! 責務ごとに以下のサブモジュールへ分割している。
//!
//! - [`geom`] — 幾何ユーティリティ（面積・距離・鉛直判定）
//! - [`self_weight_calc`] — 自重（線材・壁・シェル・ダンパー）の列挙と算定
//! - [`misc_wall`] — フレーム外雑壁の重量集計
//! - [`reactions`] — 単純支持梁の静定反力
//! - [`generate`] — 階生成の本体（[`generate_stories_multi`] ほか）

use squid_n_core::dof::{Dof, Dof6Mask};
use squid_n_core::ids::{LoadCaseId, NodeId, StoryId};
use squid_n_core::model::{
    Constraint, DiaphragmDef, ElementData, ElementKind, KBraceWeightRule, LoadCfg, MassMethod,
    MemberLoadKind, MiscWallTransfer, Model, Node, Story,
};

/// 重力加速度 [mm/s²]（内部単位系 N-mm-s、質量 ton）。
/// レビュー §1.11: `squid-n-core` 側の定数（`capacity_spectrum.rs` も使用）と
/// ソースオブトゥルースを統一する。
use squid_n_core::units::GRAVITY_MM_S2;

/// 同一階とみなす標高差 [mm]。
const LEVEL_TOL_MM: f64 = 1.0;

mod generate;
mod geom;
mod misc_wall;
mod reactions;
mod self_weight_calc;

pub use generate::{
    carry_over_manual_story_settings, generate_stories, generate_stories_multi,
    generate_stories_with_opts, StoryGenResult,
};
pub(crate) use misc_wall::misc_wall_weight_shares;
pub(crate) use self_weight_calc::{enumerate_self_weight, SelfWeightItem};

// tests が `super::*` から直接呼ぶ内部関数（本体は各サブモジュールに一元化）。
#[cfg(test)]
use reactions::static_reactions;
#[cfg(test)]
use self_weight_calc::steel_density_ton_mm3;

#[cfg(test)]
mod tests;
