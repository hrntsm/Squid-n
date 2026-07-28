//! 時刻歴応答解析（P6 §2〜§4）。
//!
//! Newmark-β 法（平均加速度・線形加速度）による線形時刻歴応答解析。
//! 基盤一様加振（相対変位形式）: `M·ü + C·u̇ + K·u = −M·r·ẍg(t)`。
//! 非線形時刻歴（各ステップ Newton 反復）は pushover.rs と同じ
//! commit/rollback 基盤を使う（§4、将来拡張）。
//!
//! 責務ごとにサブモジュールへ分割している:
//! - [`config`] — 入力設定（NewmarkCfg / HhtCfg / GroundMotion）
//! - [`result`] — 結果型（ResponseResult / ResponseHistory / TimeStepState）
//! - [`common`] — 積分スキーム共通の下位ルーチン
//! - [`history`] — 代表応答記録・層間変形角集計
//! - [`recording`] — 詳細記録（[`ThRecording`]）の組立（層別集計・部材内力包絡・間引き）
//! - [`linear`] — 線形時刻歴（Newmark-β 法）
//! - [`hht`] — 線形時刻歴（HHT-α 法）
//! - [`nonlinear`] — 非線形時刻歴（Newton 反復 + commit/rollback）

mod common;
mod config;
mod hht;
mod history;
mod linear;
mod nonlinear;
mod recording;
mod result;

pub use config::{GroundMotion, HhtCfg, NewmarkCfg};
pub use hht::linear_hht_alpha_analysis;
pub use linear::{
    linear_time_history_analysis, linear_time_history_from_state, linear_time_history_with_state,
};
pub use nonlinear::{nonlinear_time_history_analysis, NonlinearThCfg};
pub use result::{ResponseHistory, ResponseResult, StoryResponse, ThRecording, TimeStepState};

// `#[cfg(test)] mod tests` は `use super::*` 経由でこれらのシンボルを取得するため、
// テスト用に再エクスポートしてスコープに残す。
#[cfg(test)]
pub(crate) use crate::assemble::{assemble_global_k, assemble_global_m};
#[cfg(test)]
pub(crate) use squid_n_element::behavior::MassOption;

use crate::damping::Damping;

/// Rayleigh 減衰の係数 (α_m, β_k) を、2つの振動数と目標減衰比から計算する。
pub fn rayleigh_coeffs(omega1: f64, omega2: f64, h1: f64, h2: f64) -> (f64, f64) {
    Damping::rayleigh_coeffs(omega1, omega2, h1, h2)
}

/// 時刻歴ソルバ設定の決定性（R28）: Newmark/HHT 設定のビット一致確認。
#[cfg(test)]
mod tests;
