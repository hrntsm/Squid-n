//! 梁要素とその内力のデータ型定義（ロジックを持たない純粋なデータ層）。

use crate::behavior::LocalMat;
use crate::transform::LocalFrame;
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{EndCondition, RigidZone};
use std::sync::OnceLock;

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct MemberForces {
    pub at: Vec<(f64, [f64; 6])>,
}

#[derive(Clone)]
pub struct BeamElement {
    pub id: ElemId,
    pub e: f64,
    pub g: f64,
    /// 軸剛性（EA）用断面積。SRC では鉄骨の等価換算断面を累加した値になる。
    pub a: f64,
    /// 質量算定（ρ·A·L）用の幾何断面積。SRC の等価換算で質量が過大に
    /// ならないよう `a` と区別する（材料力学の An はあくまで剛性用）。
    pub a_mass: f64,
    /// ローカル y 軸まわりの断面二次モーメント（My 面、たわみ z 方向）。
    /// 断面レイヤの規約（`Section.iy`=強軸）とは軸名が逆のため、
    /// `Section.iz`（弱軸）が入る（construct.rs のクロス変換参照）。
    pub iy: f64,
    /// ローカル z 軸まわりの断面二次モーメント（Mz 面、たわみ y 方向）。
    /// せい方向＝ローカル y のため強軸曲げがこちらに対応し、
    /// `Section.iy`（強軸）が入る（construct.rs のクロス変換参照）。
    pub iz: f64,
    pub j: f64,
    /// ローカル y 方向せん断の有効せん断断面積（Mz 面と対）。`Section.as_z`（ウェブ）が入る。
    pub as_y: f64,
    /// ローカル z 方向せん断の有効せん断断面積（My 面と対）。`Section.as_y`（フランジ）が入る。
    pub as_z: f64,
    pub length: f64,
    pub density: f64,
    pub nodes: [NodeId; 2],
    pub axis: LocalFrame,
    pub rigid: RigidZone,
    pub end_cond: [EndCondition; 2],
    /// 材端のねじれ（材軸まわり回転）を解放するか（i 端, j 端）。
    ///
    /// 梁のねじり剛性を期待しない既定モデル化（`beam::i_end_torsion_release`）で
    /// i 端が `true` になる。`end_cond` のピン・半剛とは独立で、ねじり自由度のみを
    /// 解放する（曲げは `end_cond` の指定どおり）。解放端のねじりモーメントは 0 で、
    /// ねじりは材長方向に一定なため部材全長で \\( M_x = 0 \\) となる。
    pub torsion_release: [bool; 2],
    pub eval_sections: Vec<f64>,
    pub section: Option<squid_n_core::ids::SectionId>,
    pub material: Option<squid_n_core::ids::MaterialId>,
    /// 確定変位（グローバル系）。commit_state で trial_disp から確定される。
    pub committed_disp: [f64; 12],
    /// トライアル変位（グローバル系）。Newton 反復中（update_state の
    /// commit=false）も蓄積され、internal_force はこちらを参照する。
    /// これを欠くと弾性要素の内力が反復中凍結し、非線形解析の収束が
    /// 線形（準ニュートン）に劣化するうえ、弾性要素が復元力を負担しない
    /// 誤った釣合いに収束する。
    pub trial_disp: [f64; 12],
    /// [`Self::local_stiffness`] の結果キャッシュ（剛域変換・端部条件適用後の
    /// 12×12、節点自由度ベース）。剛性を決めるフィールド（断面・材料・剛域・
    /// 端部条件など。`committed_disp`／`trial_disp` を除く全フィールド）は
    /// 構築後に変更されない前提のため、初回呼び出しの結果を使い回してよい
    /// （時刻歴解析の Newton 反復で毎回再構築されるコストを避ける）。
    ///
    /// `Clone` 時は `OnceLock<LocalMat>: Clone`（`LocalMat: Clone` 経由）の
    /// 既定動作により、計算済みならキャッシュ値ごと複製される（未計算なら空の
    /// まま複製される）。どちらも正しさに影響しない。
    ///
    /// 他のフィールドと同様 `pub`（構造体リテラルで直接構築する既存コードとの
    /// 互換を保つため）。構築直後は必ず未計算（`OnceLock::new()`）で渡すこと。
    pub local_stiffness_cache: OnceLock<LocalMat>,
}
