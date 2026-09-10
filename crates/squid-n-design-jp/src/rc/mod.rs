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

pub use bond::{rc_beam_bond_check, rc_beam_bond_check_1991, Bond1991Result, BondCheckResult};
pub use column_mechanism::{
    compute_column_mechanism_sum_my, design_axial_for_mechanism, resolve_column_end_hinge,
    sum_my_from_end_hinges, ColumnEndHinge,
};
pub use wall_nonlinear::{
    wall_shear_beta_u, wall_shear_crack, wall_shear_trilinear, wall_shear_ultimate,
    WallShearTrilinear, WallShearTrilinearInput,
};

pub use crate::material_strength::{
    concrete_allowable_bond, concrete_allowable_compression, concrete_allowable_compression_class,
    concrete_allowable_shear, concrete_allowable_shear_class, concrete_young_modulus,
    high_strength_group, high_strength_pw_cap, high_strength_w_ft, is_high_strength_shear_grade,
    main_rebar_grade, rebar_allowable_shear, rebar_allowable_tension, rebar_sigma_y_of,
    shear_rebar_grade, young_ratio_n, HighStrengthGroup,
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

        let shape = match &sec.shape {
            Some(s @ SectionShape::RcRect { .. }) => s,
            Some(s @ SectionShape::RcCircle { .. }) => s,
            _ => {
                return CheckOutcome::Skipped {
                    reason:
                        "RC 検定: 配筋情報なし（Section.shape が RcRect/RcCircle ではありません）"
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

        let cr = match ctx.kind {
            MemberKind::Beam | MemberKind::Brace => {
                beam::beam_check(forces, sec, mat, ctx, shape, fc_raw)
            }
            MemberKind::Column => column::column_check(forces, sec, mat, ctx, shape, fc_raw),
        };
        CheckOutcome::Checked(cr)
    }
}

#[cfg(test)]
mod tests;
