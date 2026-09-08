//! 断面性能算定に用いる材料・換算定数。

/// SRC 断面の解析剛性算定に用いる鉄骨の等価ヤング係数比。
pub const N_S_EQ: f64 = 15.0;

/// RC 断面のせん断変形用断面積の形状係数 κ（As = A/κ）。
pub const KAPPA_RC: f64 = 1.2;

/// 鋼材のヤング係数 [N/mm²]。
pub const E_STEEL: f64 = 205000.0;

/// 鋼材のポアソン比。
pub(crate) const NU_STEEL: f64 = 0.3;

/// コンクリートのポアソン比。
pub(crate) const NU_CONCRETE: f64 = 0.2;

/// コンクリートの単位体積重量 γ [kN/m³]。
pub(crate) const GAMMA_CONCRETE: f64 = 23.0;
