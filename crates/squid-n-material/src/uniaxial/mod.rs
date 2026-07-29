//! 一軸応力–ひずみ履歴則（設計書 §7）。
//!
//! [`UniaxialMaterial`] トレイトを中核に、材料モデルごとにサブモジュールへ分割する:
//! - [`bilinear`] — バイリニア鋼材（弾性＋線形硬化＝kinematic hardening）
//! - [`menegotto_pinto`] — Menegotto–Pinto 鉄筋・鋼材（Filippou 1983 の履歴則・
//!   等方硬化を含む完全形。バウシンガー効果を滑らかに表現）
//! - [`concrete`] — コンクリート一軸履歴（放物線＋軟化＋テンションスティフニング）
//! - [`concrete_cyclic`] — コンクリート繰返し履歴（Yassin 1994 / Concrete02 系。
//!   Karsan–Jirsa 除荷・ひび割れ閉鎖・引張軟化）
//! - [`mander`] — Mander (1988) 拘束コンクリートの包絡線パラメータ算定

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

/// 一軸応力–ひずみ履歴則を示すトレイト（設計書 §7）。
/// trial/commit/revert パターンで非線形解析の試行収束に対応する。
///
/// 単位規約: ひずみは無次元、応力・接線剛性は [N/mm²]。
pub trait UniaxialMaterial: Send + Sync + Debug {
    /// 試行ひずみ strain に対する (応力 [N/mm²], 接線剛性 [N/mm²])。
    /// 状態は内部に試行値として保持。
    fn trial(&mut self, strain: f64) -> (f64, f64);
    /// `trial()` と数学的に同一の (応力, 接線剛性) を、**状態を書き換えずに**
    /// （直前の commit 状態のみを参照して）評価する。要素内 Newton 反復など
    /// 「確定させずに何度も評価したい」場面で `clone_box()+trial()` の代わりに使い、
    /// ヒープ確保（`Box` 複製）を避ける。実装は `trial()` と共通の内部関数を
    /// 呼ぶこと（計算コードの重複による将来の乖離を防ぐため）。
    fn probe(&self, strain: f64) -> (f64, f64);
    /// 試行を確定（収束後にコミット）。
    fn commit(&mut self);
    /// 試行を破棄して直前のコミット状態へ戻す（リジェクト時）。
    fn revert(&mut self);
    /// ファイバ断面などで「ファイバごとに独立した状態インスタンス」を作るための複製。
    /// 非線形履歴では各ファイバが独自の履歴変数を持つ必要があるため、
    /// 共有状態だと履歴が混入して破綮する（設計書 §6.3）。
    fn clone_box(&self) -> Box<dyn UniaxialMaterial>;
    /// チェックポイント用: 材料の全状態をバイト列へ直列化
    fn serialize_state(&self) -> Vec<u8>;
    /// チェックポイント用: バイト列から材料状態を復元。
    /// 失敗時は状態を変えずに [`MaterialStateError`] を返す。
    fn deserialize_state(&mut self, data: &[u8]) -> Result<(), MaterialStateError>;
    /// 降伏値（応力またはモーメント）を外部から更新するフック。
    /// N-M 相関により降伏面の大きさを解析中に変える要素（材端集中バネ等）が
    /// 用いる。対応しない材料は何もしない（既定実装）。
    fn set_yield(&mut self, _fy: f64) {}

    /// 塑性率（ductility）評価用の参照応力 σref。
    /// 鋼材・鉄筋は降伏強度 fy、コンクリートはシリンダー強度 fc。重み付け平均
    /// 塑性率 Jm（Σσref·A·|ε|·μi）や降伏判定の分子重みに用いる。弾性材は 0。
    fn reference_stress(&self) -> f64 {
        0.0
    }
    /// 塑性率評価用の参照ひずみ εref。
    /// 鋼材・鉄筋は降伏ひずみ fy/E、コンクリートは圧縮強度時ひずみ |εc0|。
    /// 各ファイバの塑性率 μi = |ε|/εref、降伏判定 |ε| ≥ εref に用いる。弾性材は 0。
    fn reference_strain(&self) -> f64 {
        0.0
    }

    /// コンクリート履歴の除荷則を解析種別で切替える（本実装の既定:
    /// 静的解析は逆行型、動的解析は原点指向型）。`dynamic=true` で原点指向型、
    /// `false` で逆行型。対応しない材料（鋼材等）は何もしない（既定実装）。
    fn set_concrete_hysteresis(&mut self, _dynamic: bool) {}
}

// ──────────────────────────── 既存別名（後方互換） ────────────────────────────

pub type ElasticSteel = Bilinear;
pub type ElasticConcrete = Concrete;
