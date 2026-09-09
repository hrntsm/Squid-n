//! モデルデータからの [`BeamElement`] 構築。

use super::element::BeamElement;
use super::stiffness_factors::{breakdown_with, composite_props_with};
use crate::frame::section_lookup::{get_material, get_section, sec_material};
use squid_n_core::ids::NodeId;
use squid_n_core::model::Model;

/// 危険断面位置を正規化座標 \[0,1\] で算定する。
///
/// 節点芯 0.0/1.0・部材中央 0.5 に加え、柱フェース位置
/// （`rigid_zone.face_i` / `face_j` を部材長で正規化。xi_i は \[0,0.5)、
/// xi_j は (0.5,1\] へクランプ）を含める。face=0 の端では
/// 節点芯と一致するため \[0.0, 0.5, 1.0\] になる。
/// 部材付帯情報（ハンチ端・継手位置）があればその追加検定位置も含める。
pub(crate) fn eval_sections_of(
    data: &squid_n_core::model::ElementData,
    model: &Model,
    length: f64,
) -> Vec<f64> {
    if length <= 1e-12 {
        return vec![0.0, 0.5, 1.0];
    }
    let xi_i = (data.rigid_zone.face_i_or_zero() / length).clamp(0.0, 0.5 - 1e-9);
    let xi_j = (1.0 - data.rigid_zone.face_j_or_zero() / length).clamp(0.5 + 1e-9, 1.0);
    let mut xs = vec![0.0, xi_i, 0.5, xi_j, 1.0];
    if let Some(detail) = model.member_detail(data.id) {
        xs.extend(detail.extra_check_positions(&data.rigid_zone, length));
    }
    xs.sort_by(|a, b| a.total_cmp(b));
    xs.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    xs
}

impl BeamElement {
    pub fn new(data: &squid_n_core::model::ElementData, model: &Model) -> Self {
        let geom = crate::transform::EndGeometry::of_element(data, model);
        let [n0, n1] = geom.nodes;
        let [p0, p1] = geom.coords;
        let len = geom.length;

        let axis = geom.local_frame(data.local_axis.ref_vector);
        let sec = get_section(model, data.section);
        let mat = get_material(model, sec_material(model, data));
        let g = mat.shear_modulus();

        let eval_sections = eval_sections_of(data, model, len);

        let as_y = if sec.as_y != 0.0 {
            sec.as_y
        } else {
            squid_n_core::model::rect_shear_area(sec.area)
        };
        let as_z = if sec.as_z != 0.0 {
            sec.as_z
        } else {
            squid_n_core::model::rect_shear_area(sec.area)
        };

        use squid_n_core::section_shape::SectionShape;
        let composite = sec
            .shape
            .as_ref()
            .and_then(|shape| composite_props_with(shape, &mat));

        let a_stiff = match (&composite, &sec.shape) {
            (Some(p), _) => p.area_ax,
            (None, Some(shape @ SectionShape::SrcRect { .. })) => shape.calc_axial_stiffness_area(),
            _ => sec.area,
        };
        let (sec_iy, sec_iz, j, sec_as_y, sec_as_z) = match &composite {
            Some(p) => (p.iy, p.iz, p.j, p.as_y, p.as_z),
            None => (sec.iy, sec.iz, sec.j, as_y, as_z),
        };

        let (iy, iz, as_y, as_z) = (sec_iz, sec_iy, sec_as_z, sec_as_y);

        let lp = ((p1[0] - p0[0]).powi(2) + (p1[1] - p0[1]).powi(2)).sqrt();
        let is_horizontal = lp > 1e-9 && (p1[2] - p0[2]).abs() <= 0.05 * lp;
        let factors = breakdown_with(model, data, &sec, mat.young, is_horizontal);
        let iz = iz * factors.slab;

        let wall_girder_factor = factors.wall_girder;
        let mut a_stiff = a_stiff * wall_girder_factor;
        let mut iy = iy * wall_girder_factor;
        let mut iz = iz * wall_girder_factor;
        let j = j * wall_girder_factor;
        let mut as_y = as_y * wall_girder_factor;
        let mut as_z = as_z * wall_girder_factor;

        let is_concrete_member = mat.fc.is_some();
        let misc_walls = if is_concrete_member {
            crate::wall::misc_wall::collect_misc_walls(model)
        } else {
            Vec::new()
        };
        if !misc_walls.is_empty() {
            let is_vertical_member = squid_n_core::geom::is_vertical_axis(p0, p1);

            let same_pair = |a: [NodeId; 2], b: (NodeId, NodeId)| -> bool {
                (a[0] == b.0 && a[1] == b.1) || (a[0] == b.1 && a[1] == b.0)
            };
            let compose = |i0: f64, ac: f64, contrib: &[(f64, f64, f64)]| -> f64 {
                let sum_aw: f64 = contrib.iter().map(|c| c.0).sum();
                if sum_aw <= 0.0 {
                    return i0;
                }
                let sum_aw_e: f64 = contrib.iter().map(|c| c.0 * c.1).sum();
                let g = sum_aw_e / (ac + sum_aw);
                let mut i_new = i0 + ac * g * g;
                for &(aw, e, self_i) in contrib {
                    i_new += self_i + aw * (e - g).powi(2);
                }
                i_new
            };

            if is_vertical_member {
                let d_col = sec.depth.max(sec.width);
                let ac = a_stiff;
                let mut contrib_y: Vec<(f64, f64, f64)> = Vec::new();
                let mut contrib_z: Vec<(f64, f64, f64)> = Vec::new();
                let mut a_add = 0.0;

                for wall in &misc_walls {
                    let (Some(bottom), Some(top)) = (wall.bottom_pair, wall.top_pair) else {
                        continue;
                    };
                    for s in 0..2 {
                        let pair = (bottom[s], top[s]);
                        if !same_pair([n0, n1], pair) {
                            continue;
                        }
                        let lww = (wall.wing_length(s) - d_col / 2.0).clamp(0.0, wall.lw);
                        if lww <= 0.0 {
                            continue;
                        }
                        let e_wall = wall.bottom_dir;
                        let dot_ey_signed = axis.rot[1][0] * e_wall[0]
                            + axis.rot[1][1] * e_wall[1]
                            + axis.rot[1][2] * e_wall[2];
                        let dot_ez_signed = axis.rot[2][0] * e_wall[0]
                            + axis.rot[2][1] * e_wall[1]
                            + axis.rot[2][2] * e_wall[2];
                        let dot_ey = dot_ey_signed.abs();
                        let dot_ez = dot_ez_signed.abs();

                        let aw = wall.t * lww;
                        let sign_s = if s == 0 { 1.0 } else { -1.0 };
                        let arm = d_col / 2.0 + lww / 2.0;
                        let self_i = wall.t * lww.powi(3) / 12.0;
                        a_add += aw;
                        if dot_ey >= dot_ez {
                            contrib_y.push((aw, sign_s * arm * dot_ey_signed, self_i));
                        } else {
                            contrib_z.push((aw, sign_s * arm * dot_ez_signed, self_i));
                        }
                    }
                }

                if !contrib_y.is_empty() {
                    iz = compose(iz, ac, &contrib_y);
                    as_y += contrib_y.iter().map(|c| c.0).sum::<f64>() / 1.2;
                }
                if !contrib_z.is_empty() {
                    iy = compose(iy, ac, &contrib_z);
                    as_z += contrib_z.iter().map(|c| c.0).sum::<f64>() / 1.2;
                }
                a_stiff += a_add;
            } else if is_horizontal {
                let d_beam = sec.depth;
                let ac = a_stiff;
                let mut contrib: Vec<(f64, f64, f64)> = Vec::new();
                let mut a_add = 0.0;

                for wall in &misc_walls {
                    let on = |pair: Option<[NodeId; 2]>| -> bool {
                        pair.is_some_and(|p| same_pair([n0, n1], (p[0], p[1])))
                    };
                    let (matched, hw_raw, sign) = if on(wall.bottom_pair) {
                        (true, wall.strip_height(false), 1.0)
                    } else if on(wall.top_pair) {
                        (true, wall.strip_height(true), -1.0)
                    } else {
                        (false, 0.0, 0.0)
                    };
                    if !matched {
                        continue;
                    }
                    let hw = (hw_raw - d_beam / 2.0).clamp(0.0, wall.h);
                    if hw <= 0.0 {
                        continue;
                    }
                    let aw = wall.t * hw;
                    let e_i = sign * (d_beam / 2.0 + hw / 2.0);
                    let self_i = wall.t * hw.powi(3) / 12.0;
                    a_add += aw;
                    contrib.push((aw, e_i, self_i));
                }

                if !contrib.is_empty() {
                    iz = compose(iz, ac, &contrib);
                    as_y += contrib.iter().map(|c| c.0).sum::<f64>() / 1.2;
                }
                a_stiff += a_add;
            }
        }

        Self {
            id: data.id,
            e: mat.young,
            g,
            a: a_stiff,
            a_mass: sec.area,
            iy,
            iz,
            j,
            as_y,
            as_z,
            length: len,
            density: mat.density,
            nodes: [n0, n1],
            axis,
            rigid: data.rigid_zone,
            end_cond: data.end_cond,
            torsion_release: [super::torsion::i_end_torsion_release(data, model), false],
            eval_sections,
            section: data.section,
            material: sec_material(model, data),
            committed_disp: [0.0; 12],
            trial_disp: [0.0; 12],
            local_stiffness_cache: std::sync::OnceLock::new(),
        }
    }
}
