//! 線材（梁・柱・ブレース）要素。
//!
//! - [`beam`] —         弾性梁要素（剛域・端条件・SRC 等価換算を含む）
//! - [`truss`] —        トラス（一般ブレース）要素
//! - [`concentrated`] —  材端集中ばね梁要素
//! - [`fiber`] —         ファイバー梁要素
//! - [`multi_spring`] —  マルチスプリング（MS）梁要素
//! - [`member_load`] —   部材（梁）スパン荷重の等価節点力・固定端内力
//! - `prismatic` —      直線材が共有する定式化（クレート内部）（材端解放の静縮約・幾何剛性・質量）
//! - `section_lookup` — モデルからの断面・材料の引き当て（クレート内部）（未割当時のフォールバック）
//! - [`rigid_arm`] —     剛域（材端剛体アーム）の運動学変換（弾性梁・ファイバー梁で共有）
//! - [`panel_offset`] —  仕口パネルへ接合する部材の適合（パネル分オフセット・せん断変形角の連成）
pub mod beam;
pub mod concentrated;
pub mod fiber;
pub mod member_load;
pub mod multi_spring;
pub mod panel_offset;
pub(crate) mod prismatic;
pub mod rigid_arm;
pub(crate) mod section_lookup;
pub mod truss;
