//! フレーム内雑壁の判定・幾何。
//!
//! 耐震壁にならなかった壁を周辺部材の断面性能として考慮するための、
//! 判定と幾何（袖壁長さ・腰壁/垂壁高さ）のみを提供する。

use squid_n_core::ids::NodeId;
use squid_n_core::model::{
    ElementData, ElementKind, MaterialCategory, Model, RegionAnchor, WallPlate, WallPlateShape,
    WallSlit,
};
use squid_n_core::section_shape::SectionShape;

/// RC 造の壁か（断面形状が [`SectionShape::RcWall`]、または材料の区分が
/// コンクリート）。
pub fn is_rc_wall(data: &ElementData, model: &Model) -> bool {
    let sec_is_rc_wall = data
        .section
        .and_then(|sid| model.sections.get(sid.index()))
        .is_some_and(|s| matches!(s.shape, Some(SectionShape::RcWall { .. })));
    let mat_is_rc = model
        .element_material(data)
        .is_some_and(|m| m.category == MaterialCategory::Concrete);
    sec_is_rc_wall || mat_is_rc
}

/// 耐震壁の成立判定。
///
/// 材種によらず課す条件:
/// - 上下辺が大梁（水平材）で囲まれていること（[`wall_is_framed`]）
/// - 耐震スリットが 4 辺のいずれにも無いこと（切れていると面内せん断を周辺の
///   柱梁へ伝えられない）
///
/// RC 壁（[`is_rc_wall`]）にのみ課す条件:
/// - 壁厚が 120mm 以上であること
/// - 開口周比 r0=√(開口面積/(l·h)) ≤ 0.4（複数開口モード適用後の面積）。
///   `l`・`h` は [`crate::wall::wall_element::wall_element_geometry`] の壁長・高さ
///   （台形壁では上下辺長さの平均を壁長とする）
///
/// 壁厚が特定できない（断面未設定の暫定壁）場合は、四周条件さえ満たせば
/// 成立扱い（true）とする。
pub fn wall_is_seismic(data: &ElementData, model: &Model) -> bool {
    let Some(g) = crate::wall::wall_element::wall_element_geometry(data, model) else {
        return false;
    };
    if !wall_is_framed_with(&g, model) {
        return false;
    }
    let attr = model.wall_attrs.iter().find(|w| w.elem == data.id);
    if attr.is_some_and(|a| a.slit.any()) {
        return false;
    }
    if !is_rc_wall(data, model) {
        return true;
    }
    let Some(t) = wall_thickness(data, model) else {
        return true;
    };
    if t < 120.0 {
        return false;
    }
    let opening_area = attr
        .map(|a| a.total_opening_area_for(model.multi_opening_mode))
        .unwrap_or(0.0);
    if opening_area <= 0.0 {
        return true;
    }
    if g.lw <= 0.0 || g.h <= 0.0 {
        return true;
    }
    let r0 = (opening_area / (g.lw * g.h)).max(0.0).sqrt();
    r0 <= 0.4
}

/// 壁の上下辺がともに大梁（水平材）で囲まれているか。
///
/// 上下辺のいずれかに大梁がない壁は耐震壁として扱わず、フレーム内雑壁とする。
/// 左右の鉛直辺（側柱）は要求しない。
///
/// 辺と部材の対応は節点の一致で判定する。壁の四隅とは別の節点を使う部材は
/// 対象外である。
pub fn wall_is_framed(data: &ElementData, model: &Model) -> bool {
    let Some(g) = crate::wall::wall_element::wall_element_geometry(data, model) else {
        return false;
    };
    wall_is_framed_with(&g, model)
}

/// [`crate::wall::wall_element::wall_element_geometry`] 済みの幾何に対する四周判定。
fn wall_is_framed_with(g: &crate::wall::wall_element::WallElementGeometry, model: &Model) -> bool {
    has_girder(model, g.bottom[0], g.bottom[1]) && has_girder(model, g.top[0], g.top[1])
}

/// 耐震壁とその周辺架構（上下の大梁・左右の側柱）で構造種別が食い違う場合に、
/// 是正内容を示す文を返す。
///
/// 判定は材料の区分（[`MaterialCategory`]）による。
pub fn wall_frame_category_issue(data: &ElementData, model: &Model) -> Option<String> {
    if !matches!(data.kind, ElementKind::Wall) || !wall_is_seismic(data, model) {
        return None;
    }
    let g = crate::wall::wall_element::wall_element_geometry(data, model)?;
    let (wall_label, want) = if is_rc_wall(data, model) {
        ("RC 造", MaterialCategory::Concrete)
    } else {
        ("S 造", MaterialCategory::Steel)
    };
    let edges = [
        (g.bottom[0], g.bottom[1]),
        (g.top[0], g.top[1]),
        (g.bottom[0], g.top[0]),
        (g.bottom[1], g.top[1]),
    ];
    for e in &model.elements {
        if !crate::wall::side_column::is_line_member(e.kind) || e.nodes.len() < 2 {
            continue;
        }
        let (n0, n1) = (e.nodes[0], e.nodes[1]);
        if !edges
            .iter()
            .any(|(a, b)| (*a == n0 && *b == n1) || (*a == n1 && *b == n0))
        {
            continue;
        }
        let Some(mat) = model.element_material(e) else {
            continue;
        };
        if mat.category != want {
            return Some(format!(
                "耐震壁 ID {} は{}ですが、周辺架構の部材 ID {} の材料「{}」の区分は{}です。\
                 耐震壁と周辺架構の構造種別を揃えてください。\
                 壁エレメントは壁と周辺架構を一体の耐震要素としてモデル化するため、\
                 混合構造は扱えません。",
                data.id.0,
                wall_label,
                e.id.0,
                mat.name,
                mat.category.label()
            ));
        }
    }
    None
}

/// 節点 `a`・`b` を両端に持つ大梁（水平材）が存在するか。
/// 勾配 5% までを水平とみなす。大梁として辺を構成しうるのは線材のみで、
/// ブレース（軸材）・面要素・バネは除く。
fn has_girder(model: &Model, a: NodeId, b: NodeId) -> bool {
    model.elements.iter().any(|e| {
        if !crate::wall::side_column::is_line_member(e.kind) || e.nodes.len() < 2 {
            return false;
        }
        let (n0, n1) = (e.nodes[0], e.nodes[1]);
        if !((n0 == a && n1 == b) || (n0 == b && n1 == a)) {
            return false;
        }
        let (Some(p0), Some(p1)) = (model.nodes.get(n0.index()), model.nodes.get(n1.index()))
        else {
            return false;
        };
        let (dx, dy, dz) = (
            p1.coord[0] - p0.coord[0],
            p1.coord[1] - p0.coord[1],
            p1.coord[2] - p0.coord[2],
        );
        let lp = (dx * dx + dy * dy).sqrt();
        lp > 1e-9 && dz.abs() <= 0.05 * lp
    })
}

/// 壁板厚 [mm]（RcWall 形状 → Section.thickness → Section.width の順）。
fn wall_thickness(data: &ElementData, model: &Model) -> Option<f64> {
    let sec = data
        .section
        .and_then(|sid| model.sections.get(sid.index()))?;
    let t = match &sec.shape {
        Some(SectionShape::RcWall { thickness, .. }) => *thickness,
        _ => sec.thickness.unwrap_or(sec.width),
    };
    (t > 0.0).then_some(t)
}

/// フレーム内雑壁 1 枚分の幾何情報（壁ローカル座標: 原点=下辺 a 節点、
/// x=下辺方向 0..lw、z=鉛直 0..h）。情報源は壁版である。
pub(crate) struct InFrameMiscWallGeometry {
    /// 壁板厚 [mm]
    pub t: f64,
    /// 壁長さ（下辺基準）[mm]
    pub lw: f64,
    /// 壁高さ [mm]
    pub h: f64,
    /// 下辺の節点対 [a, b]（a が壁ローカル x=0 側）。下辺に主架構が無い壁
    /// （梁から垂れる取り付く壁版）は `None`。
    pub bottom_pair: Option<[NodeId; 2]>,
    /// 上辺の節点対 [a, b]（下辺と対応付け済み）。上辺に主架構が無い壁
    /// （梁に載る取り付く壁版）は `None`。
    pub top_pair: Option<[NodeId; 2]>,
    /// 下辺方向の水平単位ベクトル（ローカル x の向き）。節点対を持たない辺が
    /// ありうるため、向きは節点から都度求めず幾何として持つ。
    pub bottom_dir: [f64; 3],
    /// 位置付き開口の包絡矩形 [x0, z0, x1, z1]（壁ローカル）。
    /// 位置付き開口がない場合は None。
    pub envelope: Option<[f64; 4]>,
    /// 由来する壁版。
    pub plate: squid_n_core::ids::WallPlateId,
    /// 耐震スリット。柱際の添字は [`Self::bottom_pair`]（＝ [`Self::wing_length`] の
    /// `side`）に、梁際の添字は 0 が下辺・1 が上辺（＝ [`Self::strip_height`] の
    /// `top`）に対応する。切れている辺の長さ・高さは 0 になる。
    pub slit: WallSlit,
}

impl InFrameMiscWallGeometry {
    /// 柱（side=0: a 側 x=0 の鉛直辺、side=1: b 側 x=lw）に取り付く
    /// 袖壁長さ [mm]（構造階高の 1/2 位置における包絡開口までの距離。
    /// 開口が h/2 を跨がない・位置不明の場合は壁を両側柱で折半 lw/2）。
    ///
    /// **柱際スリットのある側は 0 を返す。**
    pub fn wing_length(&self, side: usize) -> f64 {
        if self.slit.column_face.get(side).copied().unwrap_or(false) {
            return 0.0;
        }
        match self.envelope {
            Some([x0, z0, x1, z1]) if z0 <= self.h / 2.0 && self.h / 2.0 <= z1 => {
                if side == 0 {
                    x0.clamp(0.0, self.lw)
                } else {
                    (self.lw - x1).clamp(0.0, self.lw)
                }
            }
            _ => self.lw / 2.0,
        }
    }

    /// 梁（top=false: 下辺の梁に載る腰壁、top=true: 上辺の梁から垂れる垂壁）に
    /// 取り付く壁高さ [mm]（軸間距離の 1/2 位置における包絡開口までの距離。
    /// 開口が lw/2 を跨がない・位置不明の場合は壁を上下梁で折半 h/2）。
    ///
    /// **梁際スリットのある側は 0 を返す。**
    ///
    /// **反対側の梁が受け持たない壁は折半しない。**
    pub fn strip_height(&self, top: bool) -> f64 {
        let side = usize::from(top);
        if self.slit.beam_face[side] {
            return 0.0;
        }
        let other = usize::from(!top);
        let other_missing = self.slit.beam_face[other]
            || if top {
                self.bottom_pair.is_none()
            } else {
                self.top_pair.is_none()
            };
        let halved = if other_missing { self.h } else { self.h / 2.0 };
        match self.envelope {
            Some([x0, z0, x1, z1]) if x0 <= self.lw / 2.0 && self.lw / 2.0 <= x1 => {
                if top {
                    (self.h - z1).clamp(0.0, self.h)
                } else {
                    z0.clamp(0.0, self.h)
                }
            }
            _ => halved,
        }
    }
}

/// 剛域算定で用いる壁の最小板厚 [mm]。
pub(crate) const RIGID_ZONE_WALL_MIN_THICKNESS_MM: f64 = 100.0;

/// モデル中の全フレーム内雑壁（耐震壁として成立しない壁版）を収集する。
/// 柱際スリットのある壁も対象である。
pub(crate) fn collect_misc_walls(model: &Model) -> Vec<InFrameMiscWallGeometry> {
    collect_walls_where(model, |plate, model, _t| plate_is_misc_wall(plate, model))
}

/// フレーム内雑壁として周辺部材の断面性能へ剛性算入される壁版の一覧。
/// [`collect_misc_walls`] の収集結果から引く。
pub fn misc_stiffness_wall_plates(model: &Model) -> Vec<squid_n_core::ids::WallPlateId> {
    collect_misc_walls(model)
        .into_iter()
        .map(|g| g.plate)
        .collect()
}

/// 剛域算定に用いる壁を収集する。[`collect_misc_walls`] と違い耐震壁も含む。
/// 柱際スリットのある壁も含める。
pub(crate) fn collect_rigid_zone_walls(model: &Model) -> Vec<InFrameMiscWallGeometry> {
    collect_walls_where(model, |plate, model, t| {
        plate_is_rc_wall(plate, model) && t >= RIGID_ZONE_WALL_MIN_THICKNESS_MM
    })
}

/// 壁版をフレーム内雑壁として周辺部材の断面性能へ算入するか。
/// 壁領域全体を覆っていない壁版は常に雑壁である。
/// 覆っている壁版は、生成された要素に対する [`wall_is_seismic`] が成立すれば
/// 雑壁にしない（要素は節点集合の一致で引き当てる）。
/// 要素が見つからないときは算入しない。
fn plate_is_misc_wall(plate: &WallPlate, model: &Model) -> bool {
    if !model.wall_plate_covers_region(plate) {
        return true;
    }
    let Some(boundary) = plate.boundary_nodes() else {
        return false;
    };
    model
        .elements
        .iter()
        .find(|e| {
            matches!(e.kind, ElementKind::Wall)
                && e.nodes.len() == boundary.len()
                && boundary.iter().all(|n| e.nodes.contains(n))
        })
        .is_some_and(|e| !wall_is_seismic(e, model))
}

/// 壁版が RC 造の壁か（[`is_rc_wall`] の壁版版）。
fn plate_is_rc_wall(plate: &WallPlate, model: &Model) -> bool {
    let sec_is_rc_wall = model
        .wall_plate_section(plate)
        .is_some_and(|s| matches!(s.shape, Some(SectionShape::RcWall { .. })));
    let mat_is_rc = model
        .wall_plate_material(plate)
        .is_some_and(|m| m.category == MaterialCategory::Concrete);
    sec_is_rc_wall || mat_is_rc
}

/// 壁版を走査し、`accept` が true を返したものだけ壁ローカル座標系の
/// 幾何情報（[`InFrameMiscWallGeometry`]）へ変換して集める。
/// 柱際スリットのある壁も対象に含める。
fn collect_walls_where(
    model: &Model,
    accept: impl Fn(&WallPlate, &Model, f64) -> bool,
) -> Vec<InFrameMiscWallGeometry> {
    let mut out = Vec::new();
    for plate in &model.wall_plates {
        let Some(t) = model.wall_plate_thickness(plate) else {
            continue;
        };
        if !accept(plate, model, t) {
            continue;
        }
        let Some(mut geom) = plate_geometry(plate, model, t) else {
            continue;
        };
        geom.envelope = opening_envelope(plate);
        out.push(geom);
    }
    out
}

/// 位置付き開口の包絡矩形 [x0, z0, x1, z1]（壁ローカル）。
fn opening_envelope(plate: &WallPlate) -> Option<[f64; 4]> {
    let mut rect: Option<[f64; 4]> = None;
    for o in &plate.openings {
        let Some([x, z]) = o.offset else { continue };
        let (w, hh) = (o.width.max(0.0), o.height.max(0.0));
        if w <= 0.0 || hh <= 0.0 {
            continue;
        }
        rect = Some(match rect {
            None => [x, z, x + w, z + hh],
            Some(r) => [r[0].min(x), r[1].min(z), r[2].max(x + w), r[3].max(z + hh)],
        });
    }
    rect
}

/// 壁版 1 枚を壁ローカル座標系の幾何へ変換する（開口の包絡は呼び出し側が入れる）。
fn plate_geometry(plate: &WallPlate, model: &Model, t: f64) -> Option<InFrameMiscWallGeometry> {
    match &plate.shape {
        WallPlateShape::Enclosed { boundary } => enclosed_geometry(model, t, boundary, plate),
        WallPlateShape::Attached { anchor, extent } => {
            attached_geometry(model, t, anchor, (*extent)?, plate.id)
        }
    }
}

/// 柱・梁が囲む壁版の幾何。境界がちょうど 4 節点のものだけを扱う
/// （下辺 2 節点・上辺 2 節点という壁ローカル座標の前提が崩れるため）。
fn enclosed_geometry(
    model: &Model,
    t: f64,
    boundary: &[NodeId],
    plate: &WallPlate,
) -> Option<InFrameMiscWallGeometry> {
    if boundary.len() != 4 {
        return None;
    }
    let coords: Vec<[f64; 3]> = boundary
        .iter()
        .map(|nid| model.nodes.get(nid.index()).map(|n| n.coord))
        .collect::<Option<_>>()?;
    let mut order: Vec<usize> = (0..4).collect();
    order.sort_by(|&a, &b| coords[a][2].total_cmp(&coords[b][2]));
    let (b0, b1, t0, t1) = (order[0], order[1], order[2], order[3]);
    let (pa, pb) = (coords[b0], coords[b1]);
    let dxy = [pb[0] - pa[0], pb[1] - pa[1]];
    let lw = (dxy[0] * dxy[0] + dxy[1] * dxy[1]).sqrt();
    let h = 0.5 * ((coords[t0][2] + coords[t1][2]) - (pa[2] + pb[2]));
    if lw <= 0.0 || h <= 0.0 {
        return None;
    }
    let ex = [dxy[0] / lw, dxy[1] / lw];
    let proj = |p: [f64; 3]| -> f64 { (p[0] - pa[0]) * ex[0] + (p[1] - pa[1]) * ex[1] };
    let (ta, tb) = if proj(coords[t0]).abs() <= proj(coords[t1]).abs() {
        (t0, t1)
    } else {
        (t1, t0)
    };
    let faces = plate.column_face_nodes(model);
    let slit_of = |n: NodeId| -> bool {
        match faces {
            Some([f0, f1]) if f0 != f1 => {
                if n == f0 {
                    plate.slit.column_face[0]
                } else if n == f1 {
                    plate.slit.column_face[1]
                } else {
                    false
                }
            }
            _ => false,
        }
    };
    Some(InFrameMiscWallGeometry {
        t,
        lw,
        h,
        bottom_pair: Some([boundary[b0], boundary[b1]]),
        top_pair: Some([boundary[ta], boundary[tb]]),
        bottom_dir: [ex[0], ex[1], 0.0],
        envelope: None,
        plate: plate.id,
        slit: WallSlit {
            column_face: [slit_of(boundary[b0]), slit_of(boundary[b1])],
            beam_face: plate.slit.beam_face,
        },
    })
}

/// 取り付く壁版（パラペット・腰壁・垂れ壁）の幾何。
///
/// 取付き線がその梁の全長 `[0, 1]` を覆う場合だけを対象にする。梁の一部だけに
/// 載る壁は対象外とする。
/// 自立壁（[`RegionAnchor::FloorRegion`]）は主架構に取り付かないので対象外とする。
fn attached_geometry(
    model: &Model,
    t: f64,
    anchor: &RegionAnchor,
    extent: [f64; 2],
    plate: squid_n_core::ids::WallPlateId,
) -> Option<InFrameMiscWallGeometry> {
    let RegionAnchor::Line { nodes, span, .. } = anchor else {
        return None;
    };
    if (span[0] - 0.0).abs() > 1e-9 || (span[1] - 1.0).abs() > 1e-9 {
        return None;
    }
    if extent[0] * extent[1] < 0.0 {
        return None;
    }
    let up = extent[0] + extent[1] >= 0.0;
    let h = (extent[0].abs() + extent[1].abs()) / 2.0;
    if h <= 0.0 {
        return None;
    }
    let pa = model.nodes.get(nodes[0].index())?.coord;
    let pb = model.nodes.get(nodes[1].index())?.coord;
    let dxy = [pb[0] - pa[0], pb[1] - pa[1]];
    let lw = (dxy[0] * dxy[0] + dxy[1] * dxy[1]).sqrt();
    if lw <= 0.0 {
        return None;
    }
    let ex = [dxy[0] / lw, dxy[1] / lw, 0.0];
    let (bottom_pair, top_pair) = if up {
        (Some(*nodes), None)
    } else {
        (None, Some(*nodes))
    };
    Some(InFrameMiscWallGeometry {
        t,
        lw,
        h,
        bottom_pair,
        top_pair,
        bottom_dir: ex,
        envelope: None,
        plate,
        slit: WallSlit::default(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::MaterialCategory;
    use squid_n_core::model::{
        EndCondition, ForceRegime, LocalAxis, Material, Node, WallAttr, WallOpening,
    };

    fn make_model(thickness: f64) -> (Model, ElementData) {
        let make_node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        let shape = SectionShape::RcWall {
            thickness,
            ps: 0.0025,
        };
        let model = Model {
            nodes: vec![
                make_node(0, [0.0, 0.0, 0.0]),
                make_node(1, [4000.0, 0.0, 0.0]),
                make_node(2, [4000.0, 0.0, 3000.0]),
                make_node(3, [0.0, 0.0, 3000.0]),
            ],
            sections: vec![shape.to_section(SectionId(0), "W".into())],
            materials: vec![Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "FC24".into(),
                category: MaterialCategory::Concrete,
                young: 23000.0,
                poisson: 0.2,
                density: 2.4e-9,
                shear: None,
                fc: Some(24.0),
                fy: None,
            }],
            ..Default::default()
        };
        let mut model = model;
        let data = ElementData {
            id: ElemId(0),
            kind: ElementKind::Wall,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        crate::wall::add_surrounding_frame(&mut model, &data);
        (model, data)
    }

    /// 壁エレメントと同じ境界・断面の壁版と、それを覆う壁領域を積む。
    ///
    /// 雑壁の幾何は壁版が情報源であり、壁エレメントになるか（＝雑壁の算入対象から
    /// 外れるか）は「壁版が壁領域全体を覆うか」で決まるため、壁領域も要る。
    fn add_wall_plate(model: &mut Model, openings: Vec<WallOpening>, slit: WallSlit) {
        model.wall_regions.push(squid_n_core::model::WallRegion {
            id: squid_n_core::ids::WallRegionId(model.wall_regions.len() as u32),
            name: String::new(),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            wall_plate_ids: vec![squid_n_core::ids::WallPlateId(
                model.wall_plates.len() as u32
            )],
            posts: Vec::new(),
        });
        model.wall_plates.push(squid_n_core::model::WallPlate {
            id: squid_n_core::ids::WallPlateId(model.wall_plates.len() as u32),
            shape: squid_n_core::model::WallPlateShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            section: Some(SectionId(0)),
            opening_area: 0.0,
            opening_weight: 0.0,
            openings,
            loads: vec![],
            slit,
        });
    }

    #[test]
    fn test_wall_is_seismic_judgement() {
        let (mut model, data) = make_model(150.0);
        model.elements.push(data.clone());
        assert!(wall_is_seismic(&data, &model));
        let (mut model2, data2) = make_model(100.0);
        model2.elements.push(data2.clone());
        assert!(!wall_is_seismic(&data2, &model2));
        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area: 3.0e6,
            opening_weight: 0.0,
            slit: Default::default(),
            openings: vec![],
            finish_intensity: 0.0,
        });
        assert!(!wall_is_seismic(&data, &model));
        model.wall_attrs[0].opening_area = 0.0;
        model.wall_attrs[0].slit.column_face = [true, false];
        assert!(!wall_is_seismic(&data, &model));
    }

    /// 台形壁の開口周比 r0 は [`crate::wall::wall_element::wall_element_geometry`] の
    /// 壁長（上下辺平均）を単一情報源とする。
    #[test]
    fn 台形壁の開口周比は壁エレメント幾何の壁長を用いる() {
        use squid_n_core::model::WallAttr;

        let make_node = |id: u32, coord: [f64; 3]| Node {
            id: NodeId(id),
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        let shape = SectionShape::RcWall {
            thickness: 150.0,
            ps: 0.0025,
        };
        let mut model = Model {
            nodes: vec![
                make_node(0, [0.0, 0.0, 0.0]),
                make_node(1, [4000.0, 0.0, 0.0]),
                make_node(2, [3500.0, 0.0, 3000.0]),
                make_node(3, [500.0, 0.0, 3000.0]),
            ],
            sections: vec![shape.to_section(SectionId(0), "W".into())],
            materials: vec![Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "FC24".into(),
                category: MaterialCategory::Concrete,
                young: 23000.0,
                poisson: 0.2,
                density: 2.4e-9,
                shear: None,
                fc: Some(24.0),
                fy: None,
            }],
            ..Default::default()
        };
        let data = ElementData {
            id: ElemId(0),
            kind: ElementKind::Wall,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        crate::wall::add_surrounding_frame(&mut model, &data);
        model.elements.push(data.clone());

        let g = crate::wall::wall_element::wall_element_geometry(&data, &model).expect("幾何");
        assert!((g.lw_bottom - 4000.0).abs() < 1e-9);
        assert!((g.lw_top - 3000.0).abs() < 1e-9);
        assert!((g.lw - 3500.0).abs() < 1e-9);
        assert!((g.h - 3000.0).abs() < 1e-9);

        let opening_area = 1.8e6_f64;
        let r0_extent = (opening_area / (4000.0 * 3000.0)).sqrt();
        let r0_geom = (opening_area / (g.lw * g.h)).sqrt();
        assert!(r0_extent <= 0.4, "旧包絡では成立側: {r0_extent}");
        assert!(r0_geom > 0.4, "幾何壁長では不成立側: {r0_geom}");

        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area,
            opening_weight: 0.0,
            slit: Default::default(),
            openings: vec![],
            finish_intensity: 0.0,
        });
        assert!(
            !wall_is_seismic(&data, &model),
            "台形壁の r0 は壁エレメント幾何の壁長で評価する"
        );
    }

    /// 耐震壁は上下辺が大梁で囲まれた壁のみを対象とする。上下辺のいずれかが欠けた壁は
    /// 面内せん断の伝達先がないため不成立（フレーム内雑壁）とする。
    ///
    /// 側柱は要求しない。`make_model` は上下辺の大梁だけを置くため、この成立は
    /// 「側柱を持たない壁も耐震壁として成立する」ことを同時に示す。
    #[test]
    fn 上下辺に大梁がない壁は耐震壁として扱わない() {
        let (model, data) = make_model(150.0);
        assert!(wall_is_framed(&data, &model));
        assert!(wall_is_seismic(&data, &model));

        for drop_id in 1..=2u32 {
            let (mut model, data) = make_model(150.0);
            model.elements.retain(|e| e.id != ElemId(drop_id));
            assert!(
                !wall_is_seismic(&data, &model),
                "上下辺の一方（ElemId {drop_id}）が欠けた壁は耐震壁不成立"
            );
        }
    }

    /// 上下辺を構成するのは曲げを伝達する線材（大梁）に限る。軸材であるブレースや、
    /// 辺ではなく対角に架かる部材は、壁が負担した面内せん断の伝達先にならない。
    #[test]
    fn 上下辺を構成するのは辺に一致する大梁のみ() {
        let (mut model, data) = make_model(150.0);
        for e in model.elements.iter_mut().filter(|e| e.id == ElemId(1)) {
            e.kind = ElementKind::Brace {
                tension_only: false,
            };
        }
        assert!(!wall_is_seismic(&data, &model), "ブレースは大梁ではない");

        let (mut model, data) = make_model(150.0);
        for e in model.elements.iter_mut().filter(|e| e.id == ElemId(2)) {
            e.nodes = smallvec::smallvec![NodeId(0), NodeId(2)];
        }
        assert!(!wall_is_seismic(&data, &model), "対角材は上辺に一致しない");
    }

    /// 鋼板耐震壁は RC 規準に由来する条件（板厚 120mm・開口周比）を課さない。
    /// 鋼板は板厚 9〜16mm 程度のため、材種を問わず課すと成立しなくなる。
    ///
    /// **耐震スリットは材種を問わず課す。** 縁が切れていれば面内せん断を周辺の
    /// 柱梁へ伝えられないのは、鋼板耐震壁でも同じだからである。
    #[test]
    fn 鋼板耐震壁にはrc固有の成立条件を課さない() {
        use squid_n_core::model::WallAttr;

        let (mut model, mut data) = make_model(9.0);
        model.materials[0].category = MaterialCategory::Steel;
        model.materials[0].fc = None;
        model.materials[0].fy = Some(235.0);
        model.sections[0].shape = None;
        model.sections[0].thickness = Some(9.0);
        data.section = Some(SectionId(0));
        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area: 3.0e6,
            opening_weight: 0.0,
            slit: squid_n_core::model::WallSlit::default(),
            openings: vec![],
            finish_intensity: 0.0,
        });
        assert!(!is_rc_wall(&data, &model));
        assert!(
            wall_is_seismic(&data, &model),
            "鋼板耐震壁は板厚・開口の条件を課さない"
        );

        model.wall_attrs[0].slit.column_face = [true, false];
        assert!(
            !wall_is_seismic(&data, &model),
            "スリットは材種を問わず課す"
        );
        model.wall_attrs[0].slit = squid_n_core::model::WallSlit::default();

        let (model, data) = make_model(9.0);
        assert!(is_rc_wall(&data, &model));
        assert!(!wall_is_seismic(&data, &model));
    }

    /// 耐震壁と周辺架構の構造種別が食い違うモデルは入力不備として報告する。
    #[test]
    fn 壁と周辺架構の構造種別の食い違いを報告する() {
        let frame_section = |model: &mut Model, mat: MaterialId| -> SectionId {
            let id = SectionId(model.sections.len() as u32);
            let mut sec = SectionShape::SteelH {
                height: 400.0,
                width: 200.0,
                web_thick: 8.0,
                flange_thick: 13.0,
            }
            .to_section(id, "G".into());
            sec.material = Some(mat);
            model.sections.push(sec);
            id
        };

        let (mut model, data) = make_model(150.0);
        let rc_sec = frame_section(&mut model, MaterialId(0));
        for e in model.elements.iter_mut().filter(|e| e.id != ElemId(0)) {
            e.section = Some(rc_sec);
        }
        assert!(wall_frame_category_issue(&data, &model).is_none());

        let (mut model, data) = make_model(150.0);
        model.materials.push(Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(1),
            name: "SN400".into(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: Some(235.0),
        });
        let steel_sec = frame_section(&mut model, MaterialId(1));
        for e in model.elements.iter_mut().filter(|e| e.id == ElemId(1)) {
            e.section = Some(steel_sec);
        }
        let msg = wall_frame_category_issue(&data, &model).expect("食い違いを報告する");
        assert!(msg.contains("構造種別を揃えて"), "是正内容を示す: {msg}");
    }

    #[test]
    fn test_collect_misc_walls_and_lengths() {
        let (mut model, data) = make_model(150.0);
        model.elements.push(data);
        let openings = vec![WallOpening {
            width: 2400.0,
            height: 1500.0,
            offset: Some([800.0, 750.0]),
        }];
        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            slit: Default::default(),
            openings: openings.clone(),
            finish_intensity: 0.0,
        });
        add_wall_plate(&mut model, openings, WallSlit::default());
        let walls = collect_misc_walls(&model);
        assert_eq!(walls.len(), 1);
        let w = &walls[0];
        assert!((w.lw - 4000.0).abs() < 1e-9);
        assert!((w.h - 3000.0).abs() < 1e-9);
        assert!((w.wing_length(0) - 800.0).abs() < 1e-9);
        assert!((w.wing_length(1) - 800.0).abs() < 1e-9);
        assert!((w.strip_height(false) - 750.0).abs() < 1e-9);
        assert!((w.strip_height(true) - 750.0).abs() < 1e-9);
    }

    #[test]
    fn test_misc_wall_without_positioned_opening_splits_half() {
        let (mut model, data) = make_model(100.0);
        model.elements.push(data);
        add_wall_plate(&mut model, vec![], WallSlit::default());
        let walls = collect_misc_walls(&model);
        assert_eq!(walls.len(), 1);
        assert!((walls[0].wing_length(0) - 2000.0).abs() < 1e-9);
        assert!((walls[0].strip_height(true) - 1500.0).abs() < 1e-9);
    }

    #[test]
    fn test_column_face_slit_wall_is_collected_as_misc_wall() {
        let (mut model, data) = make_model(150.0);
        model.elements.push(data);
        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            slit: squid_n_core::model::WallSlit {
                column_face: [true, true],
                beam_face: [false, false],
            },
            openings: vec![],
            finish_intensity: 0.0,
        });
        add_wall_plate(
            &mut model,
            vec![],
            WallSlit {
                column_face: [true, true],
                beam_face: [false, false],
            },
        );
        let walls = collect_misc_walls(&model);
        assert_eq!(walls.len(), 1);
        assert_eq!(walls[0].slit.column_face, [true, true]);
    }

    /// 梁際スリットのある側は腰壁・垂れ壁の高さを 0 とし、反対側の梁が全高を負担する。
    ///
    /// 三方スリット（柱際 2 辺＋下辺）の壁は、上の梁だけが躯体と一体である。
    /// その梁は壁の全高を垂れ壁として負担するので、上下で折半しない。
    #[test]
    fn test_strip_height_follows_beam_face_slit() {
        let geom = |beam_face: [bool; 2]| InFrameMiscWallGeometry {
            t: 150.0,
            lw: 4000.0,
            h: 3000.0,
            bottom_pair: Some([NodeId(0), NodeId(1)]),
            top_pair: Some([NodeId(3), NodeId(2)]),
            bottom_dir: [1.0, 0.0, 0.0],
            envelope: None,
            plate: squid_n_core::ids::WallPlateId(0),
            slit: WallSlit {
                column_face: [false, false],
                beam_face,
            },
        };

        let plain = geom([false, false]);
        assert!((plain.strip_height(false) - 1500.0).abs() < 1e-9);
        assert!((plain.strip_height(true) - 1500.0).abs() < 1e-9);

        let bottom = geom([true, false]);
        assert!((bottom.strip_height(false) - 0.0).abs() < 1e-9);
        assert!((bottom.strip_height(true) - 3000.0).abs() < 1e-9);

        let top = geom([false, true]);
        assert!((top.strip_height(false) - 3000.0).abs() < 1e-9);
        assert!((top.strip_height(true) - 0.0).abs() < 1e-9);
    }

    /// スリットのある壁は耐震壁として成立しない。柱際でも梁際でも同じである。
    #[test]
    fn test_any_slit_breaks_seismic_wall() {
        for slit in [
            WallSlit {
                column_face: [true, false],
                beam_face: [false, false],
            },
            WallSlit {
                column_face: [false, false],
                beam_face: [true, false],
            },
            WallSlit {
                column_face: [false, false],
                beam_face: [false, true],
            },
        ] {
            let (mut model, data) = make_model(150.0);
            model.elements.push(data.clone());
            model.wall_attrs.push(WallAttr {
                elem: ElemId(0),
                opening_area: 0.0,
                opening_weight: 0.0,
                slit,
                openings: vec![],
                finish_intensity: 0.0,
            });
            assert!(
                !wall_is_seismic(&data, &model),
                "1 辺でも切れていれば不成立: {slit:?}"
            );
        }
    }

    /// 剛性算入されない壁版は `misc_stiffness_wall_plates` に載らない。
    ///
    /// モデル化図はこの一覧で「雑壁(周辺部材へ剛性算入)」と「荷重のみ(剛性に
    /// 算入しない)」を分ける。載る／載らないが実際の算入と食い違うと、図が
    /// 「算入している」と嘘をつく（三方スリット壁で実際に起きていた）。
    #[test]
    fn test_plate_without_thickness_is_not_counted_as_misc_stiffness() {
        let (mut model, data) = make_model(150.0);
        model.elements.push(data);
        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            slit: squid_n_core::model::WallSlit {
                column_face: [true, true],
                beam_face: [false, false],
            },
            openings: vec![],
            finish_intensity: 0.0,
        });
        add_wall_plate(
            &mut model,
            vec![],
            WallSlit {
                column_face: [true, true],
                beam_face: [false, false],
            },
        );
        assert_eq!(
            misc_stiffness_wall_plates(&model),
            vec![squid_n_core::ids::WallPlateId(0)],
            "板厚を引ける壁版は算入対象に載る"
        );

        model.wall_plates[0].section = None;
        assert!(
            misc_stiffness_wall_plates(&model).is_empty(),
            "板厚を引けない壁版は載らない"
        );
    }

    /// 柱際スリットは指定した側だけに効く。左右を独立に持つ意味を固定する。
    #[test]
    fn test_column_face_slit_is_per_side() {
        let (mut model, data) = make_model(150.0);
        model.elements.push(data);
        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            slit: squid_n_core::model::WallSlit {
                column_face: [true, false],
                beam_face: [false, false],
            },
            openings: vec![],
            finish_intensity: 0.0,
        });
        add_wall_plate(
            &mut model,
            vec![],
            WallSlit {
                column_face: [true, false],
                beam_face: [false, false],
            },
        );
        let walls = collect_misc_walls(&model);
        assert_eq!(walls.len(), 1);
        let faces = model.wall_plates[0]
            .column_face_nodes(&model)
            .expect("下辺 2 節点");
        let bottom = walls[0].bottom_pair.expect("下辺の節点対");
        let expected = [bottom[0] == faces[0], bottom[1] == faces[0]];
        assert_eq!(walls[0].slit.column_face, expected);
    }

    /// 壁展開を経ていないモデル（壁エレメントが 1 つも無い）でも、壁領域全体を覆う
    /// 壁版は雑壁として算入しない。展開の有無で周辺部材の剛性が変わらないようにする。
    #[test]
    fn 未展開モデルの覆う壁版は雑壁に算入しない() {
        let (mut model, _data) = make_model(150.0);
        add_wall_plate(&mut model, vec![], WallSlit::default());
        assert!(
            model.elements.iter().all(|e| e.kind != ElementKind::Wall),
            "壁エレメントを持たないモデル"
        );
        assert!(collect_misc_walls(&model).is_empty());
    }

    #[test]
    fn test_seismic_wall_not_collected() {
        let (mut model, data) = make_model(150.0);
        model.elements.push(data);
        add_wall_plate(&mut model, vec![], WallSlit::default());
        assert!(collect_misc_walls(&model).is_empty());
    }
}
