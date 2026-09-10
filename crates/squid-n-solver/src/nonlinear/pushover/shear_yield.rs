//! せん断降伏耐力 Qy の算定と降伏イベントの追跡（SRC 柱・SRC 耐震壁の
//! 部材ランク判定で「破壊モードがせん断破壊か」の判定に用いる）。
//!
//! - [`ShearThreshold`] / [`DirThreshold`] / [`ShearDir`] — 方向別 Qy しきい値
//! - [`compute_shear_yield_thresholds`] — 全部材のしきい値を組み立て
//! - [`effective_clear_span`] — 剛域控除後の内法スパン h0
//! - [`track_shear_yield`] — 各ステップのせん断降伏を軸力 σ0 を反映して判定

use super::geom::{axial_compression, dot3};
use super::types::ShearYieldEvent;
use squid_n_core::material_grade::{
    material_strength_factor_rebar, material_strength_factor_steel,
};
use squid_n_core::model::{ElementData, Material, Model, RigidZone, Section};
use squid_n_core::rc_capacity::{rc_capacity_input_from_rect, rc_qsu_simple, RcCapacityInput};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape};
use squid_n_element::behavior::{Ctx, ElementBehavior};
use squid_n_element::transform::LocalFrame;

/// せん断降伏耐力 Qy の判定しきい値（部材ごと、局所 y・z 方向、独立）。
pub(crate) struct ShearThreshold {
    pub(crate) y: DirThreshold,
    pub(crate) z: DirThreshold,
}

/// せん断降伏耐力 Qy の算定方式（方向別）。
pub(crate) enum DirThreshold {
    Static(f64),
    RcArakawa {
        /// σ0 抜きの入力一式（`sigma_0` は常に 0.0 のプレースホルダ。
        /// [`DirThreshold::qy`] が呼び出しのたびに軸力由来の値へ差し替える）。
        input: RcCapacityInput,
        /// 全断面積 [mm²]（= b・D。方向によらず同一値。σ0 = 圧縮軸力/gross_area
        /// の算定に用いる）。
        gross_area: f64,
        /// 内蔵鉄骨の全塑性せん断耐力 sAw・F/√3 [N]（SRC の累加項。RC は 0）。
        steel_qy: f64,
    },
}

impl DirThreshold {
    /// 圧縮軸力 `n_compress` [N]（0 以上。引張は呼び出し側で 0 として渡す）から Qy [N] を求める。
    pub(crate) fn qy(&self, n_compress: f64) -> f64 {
        match self {
            DirThreshold::Static(v) => *v,
            DirThreshold::RcArakawa {
                input,
                gross_area,
                steel_qy,
            } => {
                let sigma_0 = if *gross_area > 0.0 {
                    n_compress / gross_area
                } else {
                    0.0
                };
                let mut inp = *input;
                inp.sigma_0 = sigma_0;
                rc_qsu_simple(&inp) + steel_qy
            }
        }
    }
}

/// せん断降伏耐力 Qy 算定対象の方向（局所座標系。せい方向＝ローカル y）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ShearDir {
    /// 局所 y 方向（強軸曲げ＝Mz 面に伴うせん断、`Section.as_z`・`RcRebar.main_x` 対応）。
    Y,
    /// 局所 z 方向（弱軸曲げ＝My 面に伴うせん断、`Section.as_y`・`RcRebar.main_y` 対応）。
    Z,
}

/// 指定方向の荒川式用入力一式を組み立てる。
///
/// `clear_span` は剛域控除後の内法スパンを渡す。
#[allow(clippy::too_many_arguments)]
fn rc_rect_capacity_input(
    b: f64,
    d: f64,
    main: &BarSet,
    rebar: &RcRebar,
    mat: &Material,
    rebar_mat: Option<&Material>,
    shear_mat: Option<&Material>,
    clear_span: f64,
) -> Option<RcCapacityInput> {
    let mut input =
        rc_capacity_input_from_rect(b, d, main, rebar, mat, rebar_mat, shear_mat, clear_span)?;
    input.sigma_y *= rebar_mat.map(material_strength_factor_rebar).unwrap_or(1.1);
    Some(input)
}

/// 方向別のせん断降伏耐力しきい値（[`DirThreshold`]）を組み立てる。
#[derive(Clone, Copy)]
pub(crate) struct SecMaterials<'a> {
    pub material: Option<&'a Material>,
    pub rebar_mat: Option<&'a Material>,
    pub shear_mat: Option<&'a Material>,
    pub steel_mat: Option<&'a Material>,
}

/// 断面形状から精算できる場合は [`DirThreshold::RcArakawa`]、他は [`DirThreshold::Static`] とする。
fn build_dir_threshold(
    as_area: f64,
    mats: SecMaterials<'_>,
    section: Option<&Section>,
    dir: ShearDir,
    clear_span: f64,
) -> DirThreshold {
    let SecMaterials {
        material,
        rebar_mat,
        shear_mat,
        steel_mat,
    } = mats;
    if as_area <= 0.0 {
        return DirThreshold::Static(f64::INFINITY);
    }
    let Some(mat) = material else {
        return DirThreshold::Static(f64::INFINITY);
    };
    if let Some(Section {
        shape: Some(SectionShape::RcRect { b, d, rebar }),
        ..
    }) = section
    {
        let input = match dir {
            ShearDir::Y => rc_rect_capacity_input(
                *b,
                *d,
                &rebar.main_x,
                rebar,
                mat,
                rebar_mat,
                shear_mat,
                clear_span,
            ),
            ShearDir::Z => rc_rect_capacity_input(
                *d,
                *b,
                &rebar.main_y,
                rebar,
                mat,
                rebar_mat,
                shear_mat,
                clear_span,
            ),
        };
        if let Some(input) = input {
            if rc_qsu_simple(&input) > 0.0 {
                return DirThreshold::RcArakawa {
                    gross_area: input.b * input.d,
                    input,
                    steel_qy: 0.0,
                };
            }
        }
    }
    if let Some(Section {
        shape:
            Some(SectionShape::SrcRect {
                b,
                d,
                rebar,
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
            }),
        ..
    }) = section
    {
        let input = match dir {
            ShearDir::Y => rc_rect_capacity_input(
                *b,
                *d,
                &rebar.main_x,
                rebar,
                mat,
                rebar_mat,
                shear_mat,
                clear_span,
            ),
            ShearDir::Z => rc_rect_capacity_input(
                *d,
                *b,
                &rebar.main_y,
                rebar,
                mat,
                rebar_mat,
                shear_mat,
                clear_span,
            ),
        };
        let (sh, sb, tw, tf) = (
            *steel_height,
            *steel_width,
            *steel_web_thick,
            *steel_flange_thick,
        );
        let (s_aw, plate_t) = match dir {
            ShearDir::Y => ((tw * (sh - 2.0 * tf)).max(0.0), tw),
            ShearDir::Z => ((2.0 * sb * tf).max(0.0), tf),
        };
        let steel_name = steel_mat.map(|m| m.name.as_str()).unwrap_or("");
        let s_f = squid_n_core::material_grade::steel_f_value_prefix(steel_name, plate_t)
            .or_else(|| steel_mat.and_then(|m| m.fy))
            .unwrap_or(235.0);
        let factor = steel_mat
            .and_then(|m| m.strength_factor)
            .unwrap_or_else(|| {
                squid_n_core::material_grade::steel_material_strength_factor(steel_name)
            });
        let steel_qy = s_aw * s_f * factor / 3.0_f64.sqrt();
        if let Some(input) = input {
            if rc_qsu_simple(&input) + steel_qy > 0.0 {
                return DirThreshold::RcArakawa {
                    gross_area: input.b * input.d,
                    input,
                    steel_qy,
                };
            }
        }
    }
    if let Some(fy) = mat.fy {
        return DirThreshold::Static(
            as_area * fy * material_strength_factor_steel(mat) / 3.0_f64.sqrt(),
        );
    }
    let Some(fc) = mat.fc else {
        return DirThreshold::Static(f64::INFINITY);
    };
    DirThreshold::Static(as_area * 0.7 * fc.sqrt())
}

/// せん断降伏耐力 Qy [N] を算定する。
///
/// 軸力なし（σ0=0）の静的評価。
#[cfg(test)]
pub(crate) fn compute_shear_yield_qy(
    as_area: f64,
    mats: SecMaterials<'_>,
    section: Option<&Section>,
    dir: ShearDir,
    clear_span: f64,
) -> f64 {
    build_dir_threshold(as_area, mats, section, dir, clear_span).qy(0.0)
}

/// 部材長（節点間距離）[mm]。節点参照が欠落・退化の場合は None。
fn elem_length(model: &Model, elem: &ElementData) -> Option<f64> {
    let len = model.member_length(elem);
    (len > 0.0).then_some(len)
}

/// 剛域控除後の内法スパン h0 [mm]。
pub(crate) fn effective_clear_span(raw_length: f64, rigid_zone: &RigidZone) -> f64 {
    rigid_zone.flexible_length_from(raw_length)
}

pub(crate) fn compute_shear_yield_thresholds(model: &Model) -> Vec<ShearThreshold> {
    model
        .elements
        .iter()
        .map(|elem| {
            let sec = elem.section.and_then(|sid| model.sections.get(sid.index()));
            let mats = SecMaterials {
                material: model.element_material(elem),
                rebar_mat: model.element_rebar_material(elem),
                shear_mat: model.element_shear_rebar_material(elem),
                steel_mat: model.element_steel_material(elem),
            };
            let (as_y, as_z) = sec.map(|s| (s.as_y, s.as_z)).unwrap_or((0.0, 0.0));
            let raw_length = elem_length(model, elem).unwrap_or(0.0);
            let clear_span = effective_clear_span(raw_length, &elem.rigid_zone);
            ShearThreshold {
                y: build_dir_threshold(as_z, mats, sec, ShearDir::Y, clear_span),
                z: build_dir_threshold(as_y, mats, sec, ShearDir::Z, clear_span),
            }
        })
        .collect()
}

/// せん断降伏イベントの追跡（曲げとは独立の判定）。
pub(crate) fn track_shear_yield(
    model: &Model,
    behaviors: &[Box<dyn ElementBehavior>],
    thresholds: &[ShearThreshold],
    step: u32,
    events: &mut Vec<ShearYieldEvent>,
) {
    let ctx = Ctx { model };
    for (i, (elem, b)) in model.elements.iter().zip(behaviors).enumerate() {
        if elem.nodes.len() != 2 {
            continue;
        }
        let (Some(pi), Some(pj)) = (
            model.nodes.get(elem.nodes[0].index()),
            model.nodes.get(elem.nodes[1].index()),
        ) else {
            continue;
        };
        if elem_length(model, elem).is_none() {
            continue;
        }
        let frame = LocalFrame::from_nodes(pi.coord, pj.coord, elem.local_axis.ref_vector);
        let ex = frame.rot[0];
        let ey = frame.rot[1];
        let ez = frame.rot[2];

        let f = b.internal_force(&ctx);
        let f_i = [f.data[0], f.data[1], f.data[2]];
        let f_j = [f.data[6], f.data[7], f.data[8]];
        let vy = dot3(f_i, ey).abs().max(dot3(f_j, ey).abs());
        let vz = dot3(f_i, ez).abs().max(dot3(f_j, ez).abs());
        let n_compress = axial_compression(f_i, f_j, ex);

        let th = &thresholds[i];
        let qy_y = th.y.qy(n_compress);
        let qy_z = th.z.qy(n_compress);
        if vy >= qy_y || vz >= qy_z {
            events.push(ShearYieldEvent {
                step,
                elem: elem.id,
            });
        }
    }
}
