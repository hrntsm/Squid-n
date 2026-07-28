//! 3次元 M-N 相関曲面（降伏曲面）の算定。
//!
//! 部材の降伏判定に用いる N–My–Mz 空間の相関曲面を、モデル化手法別に算定する:
//!
//! - **端部単純降伏バネ** (`SimpleSpring`): 軸バネと回転バネの2バネ連成を
//!   線形相関 |N|/N許容 + M/M許容 = 1 で考慮する。曲面は N 軸を頂点とする
//!   双錐（N-M 平面内では直線）になり、ファイバ積分による曲面の
//!   ふくらみ（特に RC の圧縮側での耐力上昇）は表現できない。
//! - **マルチスプリング** (`MultiSpring`): 断面を少数の軸バネ群で置換したモデル。
//!   N-M 相関は表現できるが、バネ本数が少ないため曲面は多面体状（ファセット状）になる。
//! - **マルチファイバー** (`MultiFiber`): 断面を多数のファイバに細分割したモデル。
//!   滑らかで精度の高い相関曲面が得られる。
//!
//! 算定は剛塑性（全塑性応力分布）の支持点法による。平面保持のひずみ速度方向
//! (ε̇0, κ̇y, κ̇z) を単位球面上で掃引し、各方向でひずみ符号に応じた限界応力
//! （鋼: ±fy、コンクリート: 圧縮 -Fc / 引張 0）を積分した断面力 (N, My, Mz) が
//! 曲面上の支持点となる。マルチスプリング/マルチファイバーはバネ・ファイバ配置の
//! 解像度だけが異なり、同一の積分で評価する。
//!
//! 単位: 長さ [mm], 応力 [N/mm²], 軸力 [N], モーメント [N·mm]。
//! 座標・符号規約はファイバ断面（`fiber.rs`）と同一: ε = ε0 − κz·y + κy·z。
//!
//! 注: 既存の `MultiSpringElement`（P5.5 §3）は断面内 y 軸上の1次元バネ配置で一軸曲げのみを
//! 対象とするが、本モジュールの `MultiSpring` は3次元相関を表現するため
//! 2次元配置（粗い格子）へ一般化している。
//!
//! 責務ごとにサブモジュールへ分割する:
//! - [`types`] — 基本データ型（ファイバ・降伏モデル種別・強度パラメータ）と材料定数
//! - [`plastic`] — 全塑性応力分布による断面力の中核積分プリミティブ
//! - [`surface`] — 支持点法による M-N 相関曲面の構築
//! - [`m_phi`] — 塑性化域を考慮した M-φ / M-θ 曲線
//! - [`fibers`] — 断面形状からのファイバ/バネ配置の生成

pub mod fibers;
pub mod m_phi;
pub mod plastic;
pub mod surface;
pub mod types;

pub use fibers::{max_dimension, plastic_fibers, plastic_fibers_at, AnnulusRes};
pub use m_phi::{m_phi_curve, m_theta_curve, MPhiCurve};
pub use plastic::{axial_capacity, plastic_moment_at_n, plastic_point, slice_at_n};
pub use surface::{build_simple_spring_surface, build_surface, MnSurface};
pub use types::{concrete_young, FiberRegion, PlasticFiber, StrengthParams, YieldModelKind};

#[cfg(test)]
mod tests;
