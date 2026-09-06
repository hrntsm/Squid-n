//! 一軸応力–ひずみ履歴則。
use std::fmt::Debug;

pub mod bilinear;
pub mod concrete;
pub mod concrete_cyclic;
pub mod mander;
pub mod menegotto_pinto;

pub use bilinear::Bilinear;
pub use concrete::Concrete;
pub use concrete_cyclic::{ConcreteCyclic, ConcreteEnvelope};
pub use menegotto_pinto::MenegottoPinto;

/// 材料状態の直列化・復元に関するエラー。
#[derive(Debug, thiserror::Error)]
pub enum MaterialStateError {
    /// バイト列からの復元に失敗した（バージョン不整合・破損など）。
    #[error("材料状態の復元に失敗しました: {0}")]
    Decode(String),
}

impl MaterialStateError {
    /// 任意の表示可能なエラーを [`MaterialStateError::Decode`] へ変換する。
    pub(crate) fn decode(e: impl std::fmt::Display) -> Self {
        MaterialStateError::Decode(e.to_string())
    }
}

/// 一軸応力–ひずみ履歴則を示すトレイト。
/// trial/commit/revert パターンで非線形解析の試行収束に対応する。
///
/// 単位規約: ひずみは無次元、応力・接線剛性は [N/mm²]。
pub trait UniaxialMaterial: Send + Sync + Debug {
    /// 試行ひずみ strain に対する (応力, 接線剛性)。
    /// 未経験状態で `strain = 0.0` を与えたときは、応力 0・接線剛性 = 初期弾性係数を返すこと。
    fn trial(&mut self, strain: f64) -> (f64, f64);
    /// `trial()` と数学的に同一の値を、状態を書き換えずに評価する。
    fn probe(&self, strain: f64) -> (f64, f64);
    /// 試行を確定する。
    fn commit(&mut self);
    /// 試行を破棄して直前のコミット状態へ戻す。
    fn revert(&mut self);
    /// ファイバごとに独立した状態インスタンスを作るための複製。
    fn clone_box(&self) -> Box<dyn UniaxialMaterial>;
    /// チェックポイント用: 材料の全状態をバイト列へ直列化
    fn serialize_state(&self) -> Vec<u8>;
    /// チェックポイント用: バイト列から材料状態を復元。
    /// 失敗時は状態を変えずに [`MaterialStateError`] を返す。
    fn deserialize_state(&mut self, data: &[u8]) -> Result<(), MaterialStateError>;
    /// 降伏値を外部から更新するフック（既定は何もしない）。
    fn set_yield(&mut self, _fy: f64) {}

    /// 塑性率評価用の参照応力。弾性材は 0。
    fn reference_stress(&self) -> f64 {
        0.0
    }
    /// 塑性率評価用の参照ひずみ。弾性材は 0。
    fn reference_strain(&self) -> f64 {
        0.0
    }

    /// コンクリート履歴の除荷則を解析種別で切替える（既定は何もしない）。
    fn set_concrete_hysteresis(&mut self, _dynamic: bool) {}
}

pub type ElasticSteel = Bilinear;
pub type ElasticConcrete = Concrete;
