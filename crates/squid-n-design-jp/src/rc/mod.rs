//! RC 造の許容応力度と断面検定（RC 規準13〜18条・令82条による許容応力度計算）。

use crate::{CheckOutcome, DesignCheck, DesignCtx, MemberForcesAt, MemberKind};
use squid_n_core::model::{Material, Section};
use squid_n_core::section_shape::SectionShape;

mod beam;
/// 鉄筋コンクリート造梁の非線形復元力特性（曲げトリリニア・せん断・軸）。
pub mod beam_nonlinear;
mod bond;
mod column;
/// RC 柱の短期設計せん断力用 ΣMy（崩壊メカニズム判定）。
pub mod column_mechanism;
/// 鉄筋コンクリート造水平接合面の検討（PCa 打継ぎ面のせん断検定）。
pub mod horizontal_joint;
pub mod joint;
mod provisions;
pub mod wall;
/// 鉄筋コンクリート造耐震壁のせん断非線形特性（トリリニア Qc/βu/Qu）。
pub mod wall_nonlinear;

mod allowable;
mod design_shear;
pub(crate) mod section_props;
mod shear_capacity;

pub(crate) use bond::rc_beam_bond_check;
pub use bond::{rc_beam_bond_check_1991, Bond1991Result, BondCheckResult};
pub use column_mechanism::{
    compute_column_mechanism_sum_my, design_axial_for_mechanism, resolve_column_end_hinge,
    sum_my_from_end_hinges, ColumnEndHinge,
};
pub use wall_nonlinear::{
    wall_shear_beta_u, wall_shear_crack, wall_shear_trilinear, wall_shear_ultimate,
    WallShearTrilinear, WallShearTrilinearInput,
};

pub use crate::material_strength::{
    concrete_allowable_bond, concrete_allowable_compression, concrete_allowable_shear,
    concrete_allowable_shear_class, concrete_young_modulus, main_rebar_grade,
    rebar_allowable_shear, rebar_allowable_tension, rebar_sigma_y_of, shear_rebar_grade,
    young_ratio_n,
};

pub(crate) use allowable::*;
pub(crate) use column::interp_ma;
pub(crate) use design_shear::*;
pub(crate) use section_props::*;
pub(crate) use shear_capacity::*;

pub struct RcDesign;

impl DesignCheck for RcDesign {
    fn check(
        &self,
        forces: &MemberForcesAt,
        sec: &Section,
        mat: &Material,
        ctx: &DesignCtx,
    ) -> CheckOutcome {
        let fc_raw = mat.fc.unwrap_or(0.0);
        if fc_raw <= 0.0 {
            return CheckOutcome::Skipped {
                reason: "RC 検定: Fc 未設定（Material.fc が None/0 です。コンクリート強度を設定してください）".to_string(),
            };
        }

        let new_rebar_unset = match &sec.shape {
            Some(SectionShape::RcBeamRect { rebar, .. }) => rebar.is_unset(),
            Some(SectionShape::RcColumnRect { rebar, .. }) => rebar.is_unset(),
            Some(SectionShape::RcColumnCircle { rebar, .. }) => rebar.is_unset(),
            _ => false,
        };
        if new_rebar_unset {
            return CheckOutcome::Skipped {
                reason: "RC 検定: 配筋が未入力です".to_string(),
            };
        }

        let shape = match &sec.shape {
            Some(
                s @ (SectionShape::RcBeamRect { .. }
                | SectionShape::RcColumnRect { .. }
                | SectionShape::RcColumnCircle { .. }),
            ) => s,
            _ => {
                return CheckOutcome::Skipped {
                    reason: "RC 検定: 配筋情報なし\
                             （Section.shape が RcBeamRect/\
                             RcColumnRect/RcColumnCircle ではありません）"
                        .to_string(),
                };
            }
        };

        if ctx.rebar_material.is_none() {
            return CheckOutcome::Skipped {
                reason: "RC 検定: 主筋の材料が未割当（断面タブで主筋の材料を割り当ててください）"
                    .to_string(),
            };
        }
        if ctx.shear_rebar_material.is_none() {
            return CheckOutcome::Skipped {
                reason: "RC 検定: せん断補強筋の材料が未割当\
                         （断面タブでせん断補強筋の材料を割り当ててください）"
                    .to_string(),
            };
        }
        if let Some(msg) = squid_n_core::material_grade::shear_rebar_material_issue(
            ctx.shear_rebar_material.as_ref(),
        ) {
            return CheckOutcome::Skipped {
                reason: format!("RC 検定: {msg}"),
            };
        }

        if matches!(shape, SectionShape::RcBeamRect { .. }) && ctx.kind == MemberKind::Column {
            return CheckOutcome::Skipped {
                reason: "RC 検定: 梁用断面を柱部材に割り当てています（用途不一致）".to_string(),
            };
        }
        if matches!(
            shape,
            SectionShape::RcColumnRect { .. } | SectionShape::RcColumnCircle { .. }
        ) && matches!(ctx.kind, MemberKind::Beam | MemberKind::Brace)
        {
            return CheckOutcome::Skipped {
                reason: "RC 検定: 柱用断面を梁部材に割り当てています（用途不一致）".to_string(),
            };
        }

        let rebar_issue = match shape {
            SectionShape::RcBeamRect { b, d, rebar } => rebar.validate(*b, *d).err(),
            SectionShape::RcColumnRect { b, d, rebar } => rebar.validate(*b, *d).err(),
            SectionShape::RcColumnCircle { d, rebar } => rebar.validate(*d).err(),
            _ => None,
        };
        if let Some(e) = rebar_issue {
            return CheckOutcome::Skipped {
                reason: format!("RC 検定: 配筋が不整合です（{e}）"),
            };
        }

        let cr = if matches!(shape, SectionShape::RcBeamRect { .. }) {
            beam::beam_check(forces, sec, mat, ctx, shape, fc_raw)
        } else if matches!(
            shape,
            SectionShape::RcColumnRect { .. } | SectionShape::RcColumnCircle { .. }
        ) {
            column::column_check(forces, sec, mat, ctx, shape, fc_raw)
        } else {
            match ctx.kind {
                MemberKind::Beam | MemberKind::Brace => {
                    beam::beam_check(forces, sec, mat, ctx, shape, fc_raw)
                }
                MemberKind::Column => column::column_check(forces, sec, mat, ctx, shape, fc_raw),
            }
        };
        CheckOutcome::Checked(cr)
    }
}

#[cfg(test)]
mod tests;
