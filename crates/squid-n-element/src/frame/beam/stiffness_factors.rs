//! スラブ協力幅・合成梁・壁エレメント上下大梁による剛性の増大率算定。

use squid_n_core::ids::NodeId;
use squid_n_core::model::Model;

/// スラブ協力幅 bf = b + ba(左) + ba(右) [mm]。
///
/// 対象は水平材のみ。適用不能時は None。
fn slab_cooperating_width(
    model: &Model,
    data: &squid_n_core::model::ElementData,
    b: f64,
) -> Option<(f64, f64)> {
    if b <= 0.0
        || data.nodes.len() < 2
        || (model.floor_regions.is_empty() && model.slabs.is_empty())
    {
        return None;
    }
    let n0 = data.nodes[0];
    let n1 = data.nodes[data.nodes.len() - 1];
    let (Some(node0), Some(node1)) = (model.nodes.get(n0.index()), model.nodes.get(n1.index()))
    else {
        return None;
    };
    let (p0, p1) = (node0.coord, node1.coord);
    let (dx, dy, dz) = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    let lp = (dx * dx + dy * dy).sqrt();
    if lp < 1e-9 || dz.abs() > 0.05 * lp {
        return None;
    }
    let l = (lp * lp + dz * dz).sqrt();
    let (ex, ey) = (dx / lp, dy / lp);
    let signed_dist =
        |coord: [f64; 3]| -> f64 { -(coord[0] - p0[0]) * ey + (coord[1] - p0[1]) * ex };

    let far_beam_width =
        |boundary: &[squid_n_core::ids::NodeId], target_s: f64, sign: f64| -> f64 {
            const TOL_MM: f64 = 1.0;
            for e in &model.elements {
                if !matches!(e.kind, squid_n_core::model::ElementKind::Beam) || e.nodes.len() < 2 {
                    continue;
                }
                let (m0, m1) = (e.nodes[0], e.nodes[e.nodes.len() - 1]);
                if m0 == n0 && m1 == n1 || m0 == n1 && m1 == n0 {
                    continue;
                }
                if !(boundary.contains(&m0) && boundary.contains(&m1)) {
                    continue;
                }
                let (Some(q0), Some(q1)) =
                    (model.nodes.get(m0.index()), model.nodes.get(m1.index()))
                else {
                    continue;
                };
                let s0 = signed_dist(q0.coord) * sign;
                let s1 = signed_dist(q1.coord) * sign;
                if (s0 - target_s).abs() > TOL_MM || (s1 - target_s).abs() > TOL_MM {
                    continue;
                }
                if let Some(sec) = e.section.and_then(|sid| model.sections.get(sid.index())) {
                    if sec.width > 0.0 {
                        return sec.width;
                    }
                }
            }
            b
        };

    let mut a_pos: f64 = 0.0;
    let mut a_neg: f64 = 0.0;
    let mut t_used: f64 = 0.0;
    let mut matched = false;

    let mut referenced = std::collections::HashSet::new();
    let mut candidates: Vec<(&[NodeId], f64)> = Vec::new();
    for region in &model.floor_regions {
        referenced.extend(region.slab_ids.iter().copied());
        let t = region
            .slab_ids
            .iter()
            .filter_map(|&id| model.slab(id))
            .filter_map(|s| model.slab_plate_thickness(s))
            .fold(0.0_f64, f64::max);
        if t > 0.0 {
            candidates.push((&region.boundary, t));
        }
    }
    for slab in &model.slabs {
        if referenced.contains(&slab.id) {
            continue;
        }
        let Some(boundary) = slab.boundary_nodes() else {
            continue;
        };
        let Some(t) = model.slab_plate_thickness(slab) else {
            continue;
        };
        candidates.push((boundary, t));
    }

    for (boundary, t) in candidates {
        if !(boundary.contains(&n0) && boundary.contains(&n1)) {
            continue;
        }
        matched = true;
        t_used = t_used.max(t);
        let mut s_pos: f64 = 0.0;
        let mut s_neg: f64 = 0.0;
        for nid in boundary {
            let Some(q) = model.nodes.get(nid.index()) else {
                continue;
            };
            let s = signed_dist(q.coord);
            s_pos = s_pos.max(s);
            s_neg = s_neg.max(-s);
        }
        if s_pos > 0.0 {
            let far_w = far_beam_width(boundary, s_pos, 1.0);
            a_pos = a_pos.max((s_pos - b / 2.0 - far_w / 2.0).max(0.0));
        }
        if s_neg > 0.0 {
            let far_w = far_beam_width(boundary, s_neg, -1.0);
            a_neg = a_neg.max((s_neg - b / 2.0 - far_w / 2.0).max(0.0));
        }
    }

    let ba = |a: f64| -> f64 {
        if a <= 0.0 {
            0.0
        } else if a < 0.5 * l {
            (0.5 - 0.6 * a / l) * a
        } else {
            0.1 * l
        }
    };
    let bf = b + ba(a_pos) + ba(a_neg);
    if !matched || bf <= b {
        return None;
    }
    let t = if t_used > 0.0 {
        t_used
    } else {
        model.slab_thickness
    };
    if t <= 0.0 {
        return None;
    }
    Some((bf, t))
}

/// スラブ協力幅による強軸曲げ剛性の増大率。
/// 適用不能時は 1.0。
pub(super) fn slab_stiffness_factor(
    model: &Model,
    data: &squid_n_core::model::ElementData,
    b: f64,
    d: f64,
) -> f64 {
    if d <= 0.0 {
        return 1.0;
    }
    let Some((bf, t)) = slab_cooperating_width(model, data, b) else {
        return 1.0;
    };
    let tf = t.min(d);
    let aw = b * d;
    let af = (bf - b) * tf;
    let g = (aw * d / 2.0 + af * (d - tf / 2.0)) / (aw + af);
    let i0 = b * d.powi(3) / 12.0;
    let ie = i0
        + aw * (g - d / 2.0).powi(2)
        + (bf - b) * tf.powi(3) / 12.0
        + af * (d - tf / 2.0 - g).powi(2);
    (ie / i0).max(1.0)
}

/// 床スラブコンクリートの設計基準強度の仮定値 [N/mm²]。
const COMPOSITE_SLAB_FC: f64 = 21.0;

/// S 造合成梁の強軸曲げ剛性の増大率。適用不能時は 1.0。
pub(super) fn composite_beam_stiffness_factor(
    model: &Model,
    data: &squid_n_core::model::ElementData,
    sec: &squid_n_core::model::Section,
    es: f64,
) -> f64 {
    let (sa, si, sh) = (sec.area, sec.iy, sec.depth);
    if sa <= 0.0 || si <= 0.0 || sh <= 0.0 || es <= 0.0 {
        return 1.0;
    }
    let Some((bf, t)) = slab_cooperating_width(model, data, sec.width.max(1.0)) else {
        return 1.0;
    };
    let ec = squid_n_core::section_shape::concrete_young_modulus(COMPOSITE_SLAB_FC);
    let hd = 0.0;
    let ca = bf * t;
    let denom = ec * ca + es * sa;
    if denom <= 0.0 {
        return 1.0;
    }
    let g = (ec * ca * (t / 2.0) + es * sa * (t + hd + sh / 2.0)) / denom;
    let i_comp = (ec / es) * (bf * t.powi(3) / 12.0 + ca * (g - t / 2.0).powi(2))
        + si
        + sa * (g - t - hd - sh / 2.0).powi(2);
    ((i_comp + si) / (2.0 * si)).max(1.0)
}

/// 壁エレメントモデルの上下大梁の剛性倍率。
pub const WALL_GIRDER_STIFF_FACTOR: f64 = 100.0;

/// 部材の剛性算定で断面性能へ乗じられる割増し率の内訳。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StiffnessBreakdown {
    /// 強軸曲げ剛性の増大率。対象外は 1.0。
    pub slab: f64,
    /// 上下大梁の剛性倍率（対象外は 1.0）。
    pub wall_girder: f64,
}

impl Default for StiffnessBreakdown {
    fn default() -> Self {
        Self {
            slab: 1.0,
            wall_girder: 1.0,
        }
    }
}

/// 断面・材料が確定した状態で割増し率を求める。
pub(super) fn breakdown_with(
    model: &Model,
    data: &squid_n_core::model::ElementData,
    sec: &squid_n_core::model::Section,
    es: f64,
    is_horizontal: bool,
) -> StiffnessBreakdown {
    use squid_n_core::section_shape::SectionShape;
    let slab = match &sec.shape {
        Some(SectionShape::RcRect { .. }) => {
            slab_stiffness_factor(model, data, sec.width, sec.depth)
        }
        Some(SectionShape::SteelH { .. }) => composite_beam_stiffness_factor(model, data, sec, es),
        _ => 1.0,
    };
    let wall_girder = if is_horizontal
        && data.nodes.len() >= 2
        && is_wall_top_bottom_girder(model, data.nodes[0], data.nodes[1])
    {
        WALL_GIRDER_STIFF_FACTOR
    } else {
        1.0
    };
    StiffnessBreakdown { slab, wall_girder }
}

/// 部材の剛性算定で適用される割増し率を、モデルと要素データから求める。
/// 断面・材料が引けない要素、2 節点未満の要素はすべて 1.0 を返す。
pub fn stiffness_breakdown(
    model: &Model,
    data: &squid_n_core::model::ElementData,
) -> StiffnessBreakdown {
    if data.nodes.len() < 2 {
        return StiffnessBreakdown::default();
    }
    let (Some(sec), Some(mat)) = (
        data.section.and_then(|sid| model.sections.get(sid.index())),
        model.element_material(data),
    ) else {
        return StiffnessBreakdown::default();
    };
    let (Some(p0), Some(p1)) = (
        model.nodes.get(data.nodes[0].index()),
        model.nodes.get(data.nodes[1].index()),
    ) else {
        return StiffnessBreakdown::default();
    };
    let (dx, dy, dz) = (
        p1.coord[0] - p0.coord[0],
        p1.coord[1] - p0.coord[1],
        p1.coord[2] - p0.coord[2],
    );
    let lp = (dx * dx + dy * dy).sqrt();
    let is_horizontal = lp > 1e-9 && dz.abs() <= 0.05 * lp;
    breakdown_with(model, data, sec, mat.young, is_horizontal)
}

/// 要素の SRC/CFT 等価断面性能を求める。対象外・算定不能では `None`。
pub fn composite_props_of(
    model: &Model,
    data: &squid_n_core::model::ElementData,
) -> Option<squid_n_core::section_shape::CompositeProps> {
    let sec = data
        .section
        .and_then(|sid| model.sections.get(sid.index()))?;
    let mat = model.element_material(data)?;
    composite_props_with(sec.shape.as_ref()?, mat)
}

pub(super) fn composite_props_with(
    shape: &squid_n_core::section_shape::SectionShape,
    mat: &squid_n_core::model::Material,
) -> Option<squid_n_core::section_shape::CompositeProps> {
    use squid_n_core::section_shape::SectionShape;
    match shape {
        SectionShape::SrcRect { .. } => mat
            .fc
            .is_some()
            .then(|| shape.src_equivalent_props(mat.young, mat.poisson))
            .flatten(),
        SectionShape::CftBox { .. } | SectionShape::CftPipe { .. } => mat
            .fc
            .and_then(|fc| shape.cft_equivalent_props(mat.young, mat.poisson, fc)),
        _ => None,
    }
}

/// 自部材（両端節点 n0, n1）が壁エレメントモデルの上辺・下辺大梁かどうかを判定する。
pub(super) fn is_wall_top_bottom_girder(model: &Model, n0: NodeId, n1: NodeId) -> bool {
    model.elements.iter().any(|e| {
        matches!(e.kind, squid_n_core::model::ElementKind::Wall)
            && e.nodes.len() >= 4
            && e.nodes.contains(&n0)
            && e.nodes.contains(&n1)
            && crate::wall::misc_wall::wall_is_seismic(e, model)
    })
}
