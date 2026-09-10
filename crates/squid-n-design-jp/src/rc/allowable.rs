//! RC 検定に用いる許容応力度のまとめ（部材単位で term 依存の値を 1 回だけ計算する）。

use crate::material_strength::{
    concrete_allowable_compression_class, concrete_allowable_shear_class, rebar_allowable_shear,
    young_ratio_n,
};
pub(crate) use squid_n_core::units::ConcreteClass;

/// 検定に用いる許容応力度一式。
pub(crate) struct RcAllow {
    /// コンクリート許容圧縮応力度 fc [N/mm²]（長期/短期は算定済み）。
    pub(crate) fc: f64,
    /// コンクリート許容せん断応力度 fs [N/mm²]。
    pub(crate) fs: f64,
    /// せん断補強筋許容引張応力度 w_ft [N/mm²]。
    pub(crate) w_ft: f64,
    /// ヤング係数比 n。
    pub(crate) n_ratio: f64,
}

pub(crate) fn rc_allow(fc_raw: f64, class: ConcreteClass, grade: &str, long_term: bool) -> RcAllow {
    RcAllow {
        fc: concrete_allowable_compression_class(fc_raw, class, long_term),
        fs: concrete_allowable_shear_class(fc_raw, class, long_term),
        w_ft: rebar_allowable_shear(grade, long_term),
        n_ratio: young_ratio_n(fc_raw),
    }
}

/// 高強度せん断補強筋使用時の有効 damage_control。
/// `shear_grade` が `Some` かつ軽量のとき false にする。
pub(crate) fn effective_damage_control(
    damage_control: bool,
    shear_grade: Option<&str>,
    class: ConcreteClass,
) -> bool {
    if shear_grade.is_some() && class != ConcreteClass::Normal {
        false
    } else {
        damage_control
    }
}
