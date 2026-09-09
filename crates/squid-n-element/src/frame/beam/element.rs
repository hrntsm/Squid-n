//! 梁要素とその内力のデータ型定義（ロジックを持たない純粋なデータ層）。

use crate::behavior::LocalMat;
use crate::transform::LocalFrame;
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{EndCondition, RigidZone};
use std::sync::OnceLock;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MemberForces {
    pub at: Vec<(f64, [f64; 6])>,
}

#[derive(Clone)]
pub struct BeamElement {
    pub id: ElemId,
    pub e: f64,
    pub g: f64,
    /// 軸剛性（EA）用断面積。
    pub a: f64,
    /// 質量算定用の幾何断面積。
    pub a_mass: f64,
    /// ローカル y 軸まわりの断面二次モーメント。`Section.iz` が入る。
    pub iy: f64,
    /// ローカル z 軸まわりの断面二次モーメント。`Section.iy` が入る。
    pub iz: f64,
    pub j: f64,
    /// ローカル y 方向せん断の有効せん断断面積。`Section.as_z` が入る。
    pub as_y: f64,
    /// ローカル z 方向せん断の有効せん断断面積。`Section.as_y` が入る。
    pub as_z: f64,
    pub length: f64,
    pub density: f64,
    pub nodes: [NodeId; 2],
    pub axis: LocalFrame,
    pub rigid: RigidZone,
    pub end_cond: [EndCondition; 2],
    /// 材端のねじれ（材軸まわり回転）を解放するか（i 端, j 端）。
    ///
    /// 解放端のねじりモーメントは 0 になる。
    pub torsion_release: [bool; 2],
    pub eval_sections: Vec<f64>,
    pub section: Option<squid_n_core::ids::SectionId>,
    pub material: Option<squid_n_core::ids::MaterialId>,
    /// 確定変位（グローバル系）。
    pub committed_disp: [f64; 12],
    /// トライアル変位（グローバル系）。
    pub trial_disp: [f64; 12],
    /// [`Self::local_stiffness`] の結果キャッシュ。
    ///
    /// 剛性を決めるフィールドは構築後に変更されない前提のため、初回呼び出しの結果を使い回してよい。
    ///
    /// 構築直後は必ず未計算（`OnceLock::new()`）で渡すこと。
    pub local_stiffness_cache: OnceLock<LocalMat>,
}
