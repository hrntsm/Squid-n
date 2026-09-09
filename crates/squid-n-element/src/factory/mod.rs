//! 要素データから振る舞い（[`ElementBehavior`]）を生成するディスパッチャ。
//!
//! 責務ごとにサブモジュールへ分割している:
//!
//! - [`regime`] —       フォースレジーム判定
//! - [`wall_opening`] — 壁開口低減率
//! - [`springs`] —      バネ / 履歴則パラメータ算定
//! - [`input_check`] —  非線形解析の入力チェック（耐力を算定できない設定不備の検出）
//!
//! 本モジュールは要素種別ごとのディスパッチ（[`build_behavior`] /
//! [`build_nonlinear_behavior`]）と再エクスポートを担う。

use crate::behavior::ElementBehavior;
use squid_n_core::model::{ElementData, ElementKind, Model};

mod input_check;
mod regime;
mod springs;
mod wall_opening;

pub use input_check::{ensure_nonlinear_input, nonlinear_input_issues};
pub use regime::{resolve_force_regime, ResolvedRegime};
pub use springs::{
    plastic_zone_length, resolve_fiber_concrete_hysteresis, resolve_member_hysteresis,
    resolve_wall_concrete_hysteresis, resolve_wall_shear_hysteresis,
};
pub use squid_n_core::model::AnalysisKind;
pub(crate) use wall_opening::wall_opening_reduction;

use springs::{build_fiber, build_flexural_springs, yield_moment_and_axial};

#[cfg(test)]
use springs::{flexural_alpha_y, is_rc_like_section};
#[cfg(test)]
use squid_n_core::model::ForceRegime;

/// 線形弾性解析用の要素生成。
/// 線材（`Beam` / `Fiber` / `MultiSpring`）は `ForceRegime` に依らず常に弾性
/// [`crate::frame::beam::BeamElement`] を組む。非線形要素の振り分けは
/// [`build_nonlinear_behavior`] だけが行う。
/// 耐震壁の側柱（[`crate::wall::side_column::InPlaneReleasedColumn`]）は
/// 線形・非線形の双方で用いる。
pub fn build_behavior(data: &ElementData, model: &Model) -> Box<dyn ElementBehavior> {
    match data.kind {
        ElementKind::Beam => {
            if let Some(axis) = crate::wall::side_column::wall_side_column_release(data, model) {
                let elem = crate::frame::beam::BeamElement::new(data, model);
                return Box::new(crate::wall::side_column::InPlaneReleasedColumn::new(
                    elem, axis,
                ));
            }
            if let Some(ends) = crate::frame::panel_offset::resolve(data, model) {
                let elem = crate::frame::beam::BeamElement::new(data, model);
                return Box::new(crate::frame::panel_offset::PanelOffsetMember::new(
                    Box::new(elem),
                    ends,
                ));
            }
            Box::new(crate::frame::beam::BeamElement::new(data, model))
        }
        ElementKind::PanelZone => Box::new(crate::springs::panel::PanelZone::new(data, model)),
        ElementKind::Shell => Box::new(crate::shell::ShellElement::new(data, model)),
        ElementKind::MultiSpring => Box::new(crate::frame::beam::BeamElement::new(data, model)),
        ElementKind::Fiber => Box::new(crate::frame::beam::BeamElement::new(data, model)),
        ElementKind::Wall => {
            let stiffness_scale = if crate::wall::misc_wall::wall_is_seismic(data, model) {
                1.0
            } else {
                1e-9
            };
            match crate::wall::wall_element::WallElement::try_new_scaled(
                data,
                model,
                stiffness_scale,
            ) {
                Some(panel) => Box::new(panel),
                None => {
                    let mut elem = crate::frame::beam::BeamElement::new(data, model);
                    let r = wall_opening_reduction(data, model).max(1e-6);
                    elem.as_y *= r;
                    elem.as_z *= r;
                    elem.a *= stiffness_scale;
                    elem.iy *= stiffness_scale;
                    elem.iz *= stiffness_scale;
                    elem.j *= stiffness_scale;
                    Box::new(elem)
                }
            }
        }
        ElementKind::Brace { .. } => Box::new(crate::frame::truss::TrussElement::new(data, model)),
        ElementKind::NodalSpring => {
            Box::new(crate::springs::spring::NodalSpringElement::new(data, model))
        }
        ElementKind::Isolator => {
            Box::new(crate::springs::isolator::IsolatorElement::new(data, model))
        }
        ElementKind::Damper => {
            use squid_n_core::model::DamperKind;
            let kind = model.damper_props(data.id).unwrap_or_default().kind;
            let beh: Box<dyn ElementBehavior> = match kind {
                DamperKind::Maxwell => Box::new(crate::springs::damper::MaxwellDamperElement::new(
                    data, model,
                )),
                DamperKind::HystereticBilinear => Box::new(
                    crate::springs::damper::HystereticDamperElement::new(data, model),
                ),
            };
            beh
        }
    }
}

/// 部材耐力算定に用いる材料強度の基準。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum StrengthBasis {
    /// 公称値。時刻歴応答解析など。
    #[default]
    Nominal,
    /// 保有水平耐力計算用の材料強度。
    MaterialStrength,
}

impl StrengthBasis {
    /// 鋼材文脈の材料強度割増係数。
    pub(crate) fn steel_factor(self, mat: Option<&squid_n_core::model::Material>) -> f64 {
        match self {
            StrengthBasis::Nominal => 1.0,
            StrengthBasis::MaterialStrength => mat
                .map(squid_n_core::material_grade::material_strength_factor_steel)
                .unwrap_or(1.0),
        }
    }

    /// RC 主筋文脈の材料強度割増係数。
    pub(crate) fn rebar_factor(self, mat: Option<&squid_n_core::model::Material>) -> f64 {
        match self {
            StrengthBasis::Nominal => 1.0,
            StrengthBasis::MaterialStrength => mat
                .map(squid_n_core::material_grade::material_strength_factor_rebar)
                .unwrap_or(1.0),
        }
    }
}

/// 非線形解析用の要素生成。`ForceRegime` に基づき非線形要素を構築する。
/// 線形弾性解析は [`build_behavior`]（弾性 `BeamElement`）を使う。
/// 部材耐力算定に用いる材料強度の基準は `basis` で指定する。
/// 時刻歴応答解析は [`StrengthBasis::Nominal`]、
/// 保有水平耐力計算は [`StrengthBasis::MaterialStrength`] を渡す。
pub fn build_nonlinear_behavior(
    data: &ElementData,
    model: &Model,
    basis: StrengthBasis,
    kind: AnalysisKind,
) -> Box<dyn ElementBehavior> {
    match data.kind {
        ElementKind::Beam => {
            if let Some(axis) = crate::wall::side_column::wall_side_column_release(data, model) {
                let elem = crate::frame::beam::BeamElement::new(data, model);
                return Box::new(crate::wall::side_column::InPlaneReleasedColumn::new(
                    elem, axis,
                ));
            }
            let panel = crate::frame::panel_offset::resolve(data, model);
            let inner: Box<dyn ElementBehavior> = match resolve_force_regime(data, model) {
                ResolvedRegime::ConcentratedSpring => {
                    let elem = crate::frame::beam::BeamElement::new(data, model);
                    let rule = resolve_member_hysteresis(data, model, kind);
                    let (spring_i, spring_j, use_mn) =
                        build_flexural_springs(data, model, rule, basis);
                    let beam =
                        crate::frame::concentrated::ConcentratedSpringBeam::new_one_component(
                            elem, spring_i, spring_j,
                        );
                    let beam = if use_mn {
                        let (my0, n_allow) = yield_moment_and_axial(data, model, basis);
                        beam.with_mn_interaction(my0, n_allow)
                    } else {
                        beam
                    };
                    Box::new(beam)
                }
                ResolvedRegime::Fiber => Box::new(build_fiber(data, model, basis, kind)),
            };
            match panel {
                Some(ends) => Box::new(crate::frame::panel_offset::PanelOffsetMember::new(
                    inner, ends,
                )),
                None => inner,
            }
        }
        ElementKind::Fiber => Box::new(build_fiber(data, model, basis, kind)),
        ElementKind::MultiSpring => Box::new(crate::frame::multi_spring::MultiSpringElement::new(
            data, model, basis, kind,
        )),
        ElementKind::Brace { .. } => Box::new(crate::frame::truss::TrussElement::new(data, model)),
        ElementKind::Wall => {
            let qu = crate::wall::wall_element::WallElement::shear_capacity_of(data, model);
            if qu <= 0.0 {
                return build_behavior(data, model);
            }
            let stiffness_scale = if crate::wall::misc_wall::wall_is_seismic(data, model) {
                1.0
            } else {
                1e-9
            };
            match crate::wall::wall_element::WallElement::try_new_scaled(
                data,
                model,
                stiffness_scale,
            ) {
                Some(panel) => {
                    let panel = panel
                        .with_shear_capacity(qu)
                        .with_shear_hysteresis(resolve_wall_shear_hysteresis(data, model, kind));
                    let panel = if stiffness_scale >= 1.0 {
                        panel.with_fiber_flexure(data, model, basis, kind)
                    } else {
                        panel
                    };
                    Box::new(panel)
                }
                None => build_behavior(data, model),
            }
        }
        ElementKind::PanelZone => {
            Box::new(crate::springs::panel::PanelZone::new_nonlinear(data, model))
        }
        _ => build_behavior(data, model),
    }
}

#[cfg(test)]
mod tests;
