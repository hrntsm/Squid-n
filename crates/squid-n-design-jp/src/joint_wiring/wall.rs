//! 耐震壁（Wall 要素 × RcWall 形状）のせん断検定配線。

use super::common::{rc_dt, ForcesAt, MemberInfo};
use crate::rc::wall::{rc_wall_shear_check, RcWallInput, WallSideColumn};
use crate::rc::wall_nonlinear::{wall_shear_trilinear, WallShearTrilinearInput};
use crate::wall_opening::equivalent_opening;
use crate::{CheckComponent, CheckKind, CheckResult, LoadTerm};
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{ElementKind, Model};
use squid_n_core::section_shape::SectionShape;

/// 耐震壁（Wall 要素 × RcWall 形状）のせん断検定を一括で `out` へ追加する。
pub(super) fn check_walls(
    model: &Model,
    member_forces: &[(ElemId, ForcesAt<'_>)],
    members: &[MemberInfo<'_>],
    term: LoadTerm,
    out: &mut Vec<(NodeId, String, CheckResult)>,
) {
    for (eid, forces) in member_forces {
        let Some(elem) = model.element(*eid) else {
            continue;
        };
        if elem.kind != ElementKind::Wall {
            continue;
        }
        let Some(sec) = model.element_section(elem) else {
            continue;
        };
        let Some(SectionShape::RcWall { thickness, ps }) = sec.shape else {
            continue;
        };
        let Some(mat) = model.element_material(elem) else {
            continue;
        };
        let fc = mat.fc.unwrap_or(0.0);
        if fc <= 0.0 {
            continue;
        }
        let wall_shear_mat = model.element_shear_rebar_material(elem);
        let sigma_y_wall =
            squid_n_core::material_grade::rebar_yield_strength(model.element_rebar_material(elem))
                .unwrap_or(squid_n_core::material_grade::SHEAR_REBAR_DEFAULT_FY);
        let sigma_wh = squid_n_core::material_grade::shear_rebar_yield_strength(wall_shear_mat)
            .unwrap_or(squid_n_core::material_grade::SHEAR_REBAR_DEFAULT_FY);
        let high_strength_shear_rebar =
            squid_n_core::material_grade::is_high_strength_shear_material(wall_shear_mat);
        let coords: Vec<[f64; 3]> = elem
            .nodes
            .iter()
            .filter_map(|nid| model.nodes.get(nid.index()))
            .map(|n| n.coord)
            .collect();
        if coords.len() < 3 {
            continue;
        }
        let mut l = 0.0_f64;
        for i in 0..coords.len() {
            for jj in (i + 1)..coords.len() {
                let dx = coords[i][0] - coords[jj][0];
                let dy = coords[i][1] - coords[jj][1];
                l = l.max((dx * dx + dy * dy).sqrt());
            }
        }
        if l < 1e-9 {
            continue;
        }
        let h = coords.iter().map(|c| c[2]).fold(f64::MIN, f64::max)
            - coords.iter().map(|c| c[2]).fold(f64::MAX, f64::min);

        let attr = model.wall_attrs.iter().find(|w| w.elem == elem.id);

        let (mut l0p, mut h0p) = if h > 1e-9 && l > 1e-9 {
            match attr.and_then(|a| a.opening_dims_for(model.multi_opening_mode)) {
                Some(dims) if dims.len() == 1 => dims[0],
                Some(dims) => equivalent_opening(&dims, l, h),
                None => {
                    let area = attr
                        .map(|a| a.total_opening_area_for(model.multi_opening_mode))
                        .unwrap_or(0.0);
                    if area > 0.0 {
                        equivalent_opening(&[(area / h, h)], l, h)
                    } else {
                        (0.0, 0.0)
                    }
                }
            }
        } else {
            (0.0, 0.0)
        };
        l0p = l0p.clamp(0.0, l);
        h0p = h0p.clamp(0.0, h);

        if !squid_n_element::wall::misc_wall::wall_is_seismic(elem, model) {
            continue;
        }
        if !squid_n_element::wall::misc_wall::is_rc_wall(elem, model) {
            continue;
        }
        let wall_nodes = &elem.nodes;
        let mut side_columns = Vec::new();
        let mut sum_col_depth = 0.0;
        let mut col_gross_area = 0.0_f64;
        let mut col_main_area_max = 0.0_f64;
        let mut dc_max = 0.0_f64;
        for m in members {
            if !m.is_column() {
                continue;
            }
            let n0 = m.elem.nodes[0];
            let n1 = m.elem.nodes[1];
            if !(wall_nodes.contains(&n0) && wall_nodes.contains(&n1)) {
                continue;
            }
            let steel_shear = match m.sec.shape {
                Some(SectionShape::SrcRect {
                    steel_height,
                    steel_web_thick,
                    steel_flange_thick,
                    ..
                }) => {
                    let as_web =
                        (steel_web_thick * (steel_height - 2.0 * steel_flange_thick)).max(0.0);
                    let steel_name = m.steel_mat.map(|mm| mm.name.as_str()).unwrap_or("");
                    let f = crate::steel::steel_f_value_prefix(
                        steel_name,
                        steel_flange_thick.max(steel_web_thick),
                    )
                    .unwrap_or(235.0);
                    crate::steel::steel_fs(f, term) * as_web
                }
                _ => 0.0,
            };
            let bd_rebar = match m.sec.shape {
                Some(SectionShape::RcRect { b, d, ref rebar }) => Some((b, d, rebar)),
                Some(SectionShape::SrcRect {
                    b, d, ref rebar, ..
                }) => Some((b, d, rebar)),
                _ => None,
            };
            let Some((b, d, rebar)) = bd_rebar else {
                continue;
            };
            let dt = rc_dt(rebar);
            let pw = squid_n_core::rc_rebar_geom::pw_ratio(&rebar.shear, b);
            side_columns.push(WallSideColumn {
                b,
                d_eff: d - dt,
                pw,
                w_ft: crate::rc::rebar_allowable_shear(
                    m.shear_mat.map(|mm| mm.name.as_str()).unwrap_or(""),
                    term == LoadTerm::Long,
                ),
                steel_shear,
            });
            sum_col_depth += d;
            col_gross_area += b * d;
            dc_max = dc_max.max(d);
            let main_area = squid_n_core::section_shape::bar_set_area(&rebar.main_x)
                + squid_n_core::section_shape::bar_set_area(&rebar.main_y);
            col_main_area_max = col_main_area_max.max(main_area);
        }
        let l_clear = (l - sum_col_depth / 2.0).max(0.1 * l);
        let q_design = forces
            .iter()
            .map(|(_, f)| f[1].abs().max(f[2].abs()))
            .fold(0.0, f64::max);
        let inp = RcWallInput {
            t: thickness,
            l,
            l_clear,
            fc,
            ps,
            w_ft: crate::rc::rebar_allowable_shear(
                wall_shear_mat.map(|mm| mm.name.as_str()).unwrap_or(""),
                term == LoadTerm::Long,
            ),
            side_columns,
            opening: if l0p > 1e-9 && h0p > 1e-9 {
                Some((l0p, h0p, h, l))
            } else {
                None
            },
            q_design,
            long_term: term == LoadTerm::Long,
        };
        let cr = rc_wall_shear_check(&inp);
        out.push((elem.nodes[0], "耐震壁(RC)".to_string(), cr));

        let aw = thickness * l + col_gross_area;
        let d_wall = l + sum_col_depth / 2.0;
        if col_main_area_max > 0.0 && aw > 0.0 && d_wall > 0.0 {
            let te = (aw / d_wall).clamp(thickness, 1.5 * thickness);
            let n_comp = forces.iter().map(|(_, f)| -f[0]).fold(0.0_f64, f64::max);
            let sigma_0 = n_comp / aw;
            let shear_span_ratio = forces
                .iter()
                .max_by(|a, b| {
                    a.1[5]
                        .abs()
                        .partial_cmp(&b.1[5].abs())
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .and_then(|(_, f)| {
                    let q = f[1].abs().max(f[2].abs());
                    (q > 1e-6).then(|| f[5].abs() / q / d_wall)
                })
                .unwrap_or_else(|| h / (2.0 * d_wall));
            let tri_inp = WallShearTrilinearInput {
                fc,
                aw,
                tension_column_main_area: col_main_area_max,
                pw_vertical: ps,
                sigma_y_wall,
                te,
                t: thickness,
                d_wall,
                dc_compression: dc_max,
                tension_column_at: col_main_area_max,
                sigma_wh,
                pwh_ratio: ps,
                sigma_0,
                shear_span_ratio,
                high_strength_shear_rebar,
                opening: if l0p > 1e-9 && h0p > 1e-9 {
                    Some((l0p, h0p, h, l))
                } else {
                    None
                },
            };
            let tri = wall_shear_trilinear(&tri_inp);
            let ratio = if tri.qu > 0.0 { q_design / tri.qu } else { 0.0 };
            let detail = format!(
                "Qc={:.1} kN, βu={:.3}, Qu={:.1} kN, r={:.3}, QD={:.1} kN（せん断非線形トリリニア骨格）",
                tri.qc / 1000.0,
                tri.beta_u,
                tri.qu / 1000.0,
                tri.r_opening,
                q_design / 1000.0
            );
            out.push((
                elem.nodes[0],
                "耐震壁(RC)せん断非線形".to_string(),
                CheckResult {
                    basis: "技術基準解説書 耐震壁せん断非線形(Qc/βu/Qu)".to_string(),
                    detail: String::new(),
                    components: vec![CheckComponent {
                        kind: CheckKind::Shear,
                        ratio,
                        detail,
                    }],
                },
            ));
        }
    }
}
