//! フレーム内雑壁（RC規準の耐震壁規定。フレーム内雑壁のモデル化）。
//!
//! 壁が開口等により耐震壁にならなかった場合、壁は壁エレメントとしてではなく
//! 周辺の RC/SRC 部材（柱の袖壁・梁の腰壁/垂壁）の断面性能として考慮される。
//! 複数開口が存在する場合は包絡開口により壁の長さを考慮し、剛性に用いる
//! 壁の長さは「構造階高および軸間距離の 1/2 の位置における包絡開口までの
//! 距離」を採用する。柱の回転により壁が傾斜して取り付く場合は傾斜を無視する。
//!
//! 本モジュールは判定と幾何（袖壁長さ・腰壁/垂壁高さ）のみを提供し、
//! 周辺部材への断面性能の合成は梁要素側（`beam.rs`）で行う。

use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{ElementData, ElementKind, MaterialCategory, Model};
use squid_n_core::section_shape::SectionShape;

/// RC 造の壁か（断面形状が [`SectionShape::RcWall`]、または材料の区分が
/// コンクリート）。
///
/// 構造種別の判定軸は材料の区分（[`MaterialCategory`]）である。壁形状
/// [`SectionShape::RcWall`] は壁厚と壁筋比を持つ RC 壁専用の形状のため、
/// 形状が付いていればそれだけで RC と判定する。
///
/// 耐震壁の成立条件のうち、スリット・壁厚・開口周比の 3 つは RC 規準の規定で
/// あり、RC 壁にのみ課す（[`wall_is_seismic`]）。鋼板耐震壁は板厚 9〜16mm 程度
/// で RC の 120mm 閾値に掛かるため、材種を問わず課すと耐震壁として成立しない。
pub fn is_rc_wall(data: &ElementData, model: &Model) -> bool {
    let sec_is_rc_wall = data
        .section
        .and_then(|sid| model.sections.get(sid.index()))
        .is_some_and(|s| matches!(s.shape, Some(SectionShape::RcWall { .. })));
    let mat_is_rc = data
        .material
        .and_then(|mid| model.materials.get(mid.index()))
        .is_some_and(|m| m.category == MaterialCategory::Concrete);
    sec_is_rc_wall || mat_is_rc
}

/// 耐震壁の成立判定。
///
/// 材種によらず課す条件:
/// - 上下辺が大梁（水平材）で囲まれていること（[`wall_is_framed`]）
///
/// RC 壁（[`is_rc_wall`]）にのみ課す条件（RC規準・耐震壁の判定）:
/// - スリット（三方スリット）がないこと
/// - 壁厚が 120mm 以上であること
/// - 開口周比 r0=√(開口面積/(l·h)) ≤ 0.4（複数開口モード適用後の面積）
///
/// 壁厚が特定できない（断面未設定の暫定壁）場合は、四周条件さえ満たせば
/// 成立扱い（true）とする。RC 固有の 3 条件の原典実装は
/// `squid-n-design-jp::wall_opening::is_seismic_wall` と同一規定で、四周条件だけは
/// モデルのトポロジを要するため本関数が担う。耐震壁か否かの答えが解析と検定で
/// 食い違わないよう、検定側（`squid-n-design-jp::joint_wiring`）も本関数を用いる。
///
/// 3D モデル化ビュー（`squid-n-app`）でも、壁を独立した壁エレメント（耐震壁）として
/// 描くか、周辺部材へ剛性算入されるフレーム内雑壁として描くかの判定に用いるため
/// 公開する。
pub fn wall_is_seismic(data: &ElementData, model: &Model) -> bool {
    if !wall_is_framed(data, model) {
        return false;
    }
    // 以降は RC 規準の耐震壁規定。鋼板耐震壁には課さない。
    if !is_rc_wall(data, model) {
        return true;
    }
    let Some(t) = wall_thickness(data, model) else {
        return true;
    };
    let attr = model.wall_attrs.iter().find(|w| w.elem == data.id);
    if attr.is_some_and(|a| a.three_side_slit) {
        return false;
    }
    if t < 120.0 {
        return false;
    }
    let opening_area = attr
        .map(|a| a.total_opening_area_for(model.multi_opening_mode))
        .unwrap_or(0.0);
    if opening_area <= 0.0 {
        return true;
    }
    let Some((lw, h)) = wall_extent(data, model) else {
        return true;
    };
    if lw <= 0.0 || h <= 0.0 {
        return true;
    }
    let r0 = (opening_area / (lw * h)).max(0.0).sqrt();
    r0 <= 0.4
}

/// 壁の上下辺がともに大梁（水平材）で囲まれているか。
///
/// 壁が負担した面内せん断は、壁エレメントの上下の剛梁を介して大梁へ伝達される。
/// 上下辺のいずれかに大梁が無い壁は伝達先を持たないため耐震壁として扱わず、
/// フレーム内雑壁とする（最下階で基礎梁を省略した入力も、下辺の伝達先が無いため
/// 対象外となる）。
///
/// 左右の鉛直辺（側柱）は要求しない。側柱を持たない耐震壁は、壁の縦筋が一様配筋
/// であるとみなして等価引張鉄筋比を pte=100·ps とする正規の対象であり
/// （[`crate::wall_panel::WallPanelElement::shear_capacity_of`]）、側柱を必須にすると
/// この経路へ到達できなくなる。
///
/// 辺と部材の対応は節点の一致で判定する。壁の四隅とは別の節点を使う部材は、
/// 壁エレメントの剛梁が拾えない（四隅の並進しか伝達しない）ため対象外である。
pub fn wall_is_framed(data: &ElementData, model: &Model) -> bool {
    // 幾何を取れない壁（4 節点未満・退化）は辺を定義できないため不成立。
    let Some(g) = crate::wall_panel::wall_panel_geometry(data, model) else {
        return false;
    };
    has_girder(model, g.bottom[0], g.bottom[1]) && has_girder(model, g.top[0], g.top[1])
}

/// 耐震壁とその周辺架構（上下の大梁・左右の側柱）で構造種別が食い違う場合に、
/// 是正内容を示す文を返す。
///
/// 壁エレメントは、面内せん断を壁が負担し曲げを周辺架構が負担する置換モデルであり、
/// 壁と周辺架構は一体の耐震要素として挙動する。RC 壁に S 骨組（あるいはその逆）を
/// 組み合わせた混合構造は、耐力式・剛性評価の前提が成り立たないため扱わない。
///
/// 判定は材料の区分（[`MaterialCategory`]）による。材料が未設定の部材は別の入力
/// チェックが捕捉するため、ここでは対象外とする。
pub fn wall_frame_category_issue(data: &ElementData, model: &Model) -> Option<String> {
    if !matches!(data.kind, ElementKind::Wall) || !wall_is_seismic(data, model) {
        return None;
    }
    let g = crate::wall_panel::wall_panel_geometry(data, model)?;
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
        if !crate::side_column::is_line_member(e.kind) || e.nodes.len() < 2 {
            continue;
        }
        let (n0, n1) = (e.nodes[0], e.nodes[1]);
        if !edges
            .iter()
            .any(|(a, b)| (*a == n0 && *b == n1) || (*a == n1 && *b == n0))
        {
            continue;
        }
        let Some(mat) = e.material.and_then(|mid| model.materials.get(mid.index())) else {
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
///
/// 水平材の判定規約は既存の実装と揃え、勾配 5% までを水平とみなす
/// （[`crate::beam::stiffness_breakdown`]）。
///
/// 大梁として辺を構成しうるのは線材のみで、ブレース（軸材）・面要素・バネは
/// 曲げを伝達しないため除く。
fn has_girder(model: &Model, a: NodeId, b: NodeId) -> bool {
    model.elements.iter().any(|e| {
        if !crate::side_column::is_line_member(e.kind) || e.nodes.len() < 2 {
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

/// 壁の包絡寸法（最大水平距離 lw × 鉛直高さ h）。
fn wall_extent(data: &ElementData, model: &Model) -> Option<(f64, f64)> {
    let coords: Vec<[f64; 3]> = data
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .collect();
    if coords.len() < 3 {
        return None;
    }
    let mut l = 0.0_f64;
    for i in 0..coords.len() {
        for j in (i + 1)..coords.len() {
            let dx = coords[i][0] - coords[j][0];
            let dy = coords[i][1] - coords[j][1];
            l = l.max((dx * dx + dy * dy).sqrt());
        }
    }
    let zmax = coords.iter().map(|c| c[2]).fold(f64::MIN, f64::max);
    let zmin = coords.iter().map(|c| c[2]).fold(f64::MAX, f64::min);
    Some((l, zmax - zmin))
}

/// フレーム内雑壁 1 枚分の幾何情報（壁ローカル座標: 原点=下辺 a 節点、
/// x=下辺方向 0..lw、z=鉛直 0..h）。
pub(crate) struct MiscWall {
    pub elem: ElemId,
    /// 壁板厚 [mm]
    pub t: f64,
    /// 壁長さ（下辺基準）[mm]
    pub lw: f64,
    /// 壁高さ [mm]
    pub h: f64,
    /// 下辺の節点対 [a, b]（a が壁ローカル x=0 側）
    pub bottom_pair: [NodeId; 2],
    /// 上辺の節点対 [a, b]（下辺と対応付け済み）
    pub top_pair: [NodeId; 2],
    /// 位置付き開口の包絡矩形 [x0, z0, x1, z1]（壁ローカル）。
    /// 位置付き開口が無い場合は None。
    pub envelope: Option<[f64; 4]>,
}

impl MiscWall {
    /// 柱（side=0: a 側 x=0 の鉛直辺、side=1: b 側 x=lw）に取り付く
    /// 袖壁長さ [mm]（構造階高の 1/2 位置における包絡開口までの距離。
    /// 開口が h/2 を跨がない・位置不明の場合は壁を両側柱で折半 lw/2）。
    pub fn wing_length(&self, side: usize) -> f64 {
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

    /// 梁（top=false: 下辺の梁の垂壁、top=true: 上辺の梁の腰壁）に取り付く
    /// 壁高さ [mm]（軸間距離の 1/2 位置における包絡開口までの距離。
    /// 開口が lw/2 を跨がない・位置不明の場合は壁を上下梁で折半 h/2）。
    ///
    /// ※下辺の梁にとって壁は上に載る「腰壁」、上辺の梁にとっては下に垂れる
    /// 「垂壁」だが、剛性算入上は取り付く壁高さのみが問題となる。
    pub fn strip_height(&self, top: bool) -> f64 {
        match self.envelope {
            Some([x0, z0, x1, z1]) if x0 <= self.lw / 2.0 && self.lw / 2.0 <= x1 => {
                if top {
                    (self.h - z1).clamp(0.0, self.h)
                } else {
                    z0.clamp(0.0, self.h)
                }
            }
            _ => self.h / 2.0,
        }
    }
}

/// モデル中の全フレーム内雑壁（耐震壁不成立の Wall 要素）を収集する。
/// 三方スリットの壁は周辺部材と縁が切れているため剛性算入の対象外とする
/// （自重は荷重側で別途評価される）。
pub(crate) fn collect_misc_walls(model: &Model) -> Vec<MiscWall> {
    let mut out = Vec::new();
    for data in &model.elements {
        if !matches!(data.kind, ElementKind::Wall) || data.nodes.len() < 4 {
            continue;
        }
        if wall_is_seismic(data, model) {
            continue;
        }
        let attr = model.wall_attrs.iter().find(|w| w.elem == data.id);
        if attr.is_some_and(|a| a.three_side_slit) {
            continue;
        }
        let Some(t) = wall_thickness(data, model) else {
            continue;
        };

        // 壁ローカル座標系の構築（wall_panel::try_new と同じ並べ替え）
        let ids: Vec<NodeId> = data.nodes.iter().take(4).copied().collect();
        let Some(coords) = ids
            .iter()
            .map(|nid| model.nodes.get(nid.index()).map(|n| n.coord))
            .collect::<Option<Vec<[f64; 3]>>>()
        else {
            continue;
        };
        let mut order: Vec<usize> = (0..4).collect();
        order.sort_by(|&a, &b| coords[a][2].partial_cmp(&coords[b][2]).unwrap());
        let (b0, b1, t0, t1) = (order[0], order[1], order[2], order[3]);
        let (pa, pb) = (coords[b0], coords[b1]);
        let dxy = [pb[0] - pa[0], pb[1] - pa[1]];
        let lw = (dxy[0] * dxy[0] + dxy[1] * dxy[1]).sqrt();
        let h = 0.5 * ((coords[t0][2] + coords[t1][2]) - (pa[2] + pb[2]));
        if lw <= 0.0 || h <= 0.0 {
            continue;
        }
        let ex = [dxy[0] / lw, dxy[1] / lw];
        let proj = |p: [f64; 3]| -> f64 { (p[0] - pa[0]) * ex[0] + (p[1] - pa[1]) * ex[1] };
        let (ta, tb) = if proj(coords[t0]).abs() <= proj(coords[t1]).abs() {
            (t0, t1)
        } else {
            (t1, t0)
        };

        // 位置付き開口の包絡矩形（複数開口は包絡開口により
        // 壁の長さを考慮する）
        let envelope = attr.and_then(|a| {
            let mut rect: Option<[f64; 4]> = None;
            for o in &a.openings {
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
        });

        out.push(MiscWall {
            elem: data.id,
            t,
            lw,
            h,
            bottom_pair: [ids[b0], ids[b1]],
            top_pair: [ids[ta], ids[tb]],
            envelope,
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{MaterialId, SectionId};
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
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        // 耐震壁は上下辺が大梁で囲まれた壁を対象とするため、上下辺に線材を置く。
        crate::wall::add_surrounding_frame(&mut model, &data);
        (model, data)
    }

    #[test]
    fn test_wall_is_seismic_judgement() {
        // 無開口・t=150 → 成立
        let (mut model, data) = make_model(150.0);
        model.elements.push(data.clone());
        assert!(wall_is_seismic(&data, &model));
        // 薄壁 t=100 → 不成立
        let (mut model2, data2) = make_model(100.0);
        model2.elements.push(data2.clone());
        assert!(!wall_is_seismic(&data2, &model2));
        // 大開口 r0>0.4（面積 > 0.16·lw·h = 1.92e6）→ 不成立
        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area: 3.0e6,
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![],
        });
        assert!(!wall_is_seismic(&data, &model));
        // 三方スリット → 不成立
        model.wall_attrs[0].opening_area = 0.0;
        model.wall_attrs[0].three_side_slit = true;
        assert!(!wall_is_seismic(&data, &model));
    }

    /// 耐震壁は上下辺が大梁で囲まれた壁のみを対象とする。上下辺のいずれかが欠けた壁は
    /// 面内せん断の伝達先が無いため不成立（フレーム内雑壁）とする。
    ///
    /// 側柱は要求しない。`make_model` は上下辺の大梁だけを置くため、この成立は
    /// 「側柱を持たない壁も耐震壁として成立する」ことを同時に示す。
    #[test]
    fn 上下辺に大梁が無い壁は耐震壁として扱わない() {
        // 上下辺が揃っていれば、側柱が無くても成立する。
        let (model, data) = make_model(150.0);
        assert!(wall_is_framed(&data, &model));
        assert!(wall_is_seismic(&data, &model));

        // 上下辺のうち一方を取り除くと不成立になる。下辺（基礎梁）を落とした
        // 最下階の壁も、下辺の伝達先が無いため対象外である。
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
        // 下辺の部材（ElemId 1）をブレースへ変えると不成立になる。
        let (mut model, data) = make_model(150.0);
        for e in model.elements.iter_mut().filter(|e| e.id == ElemId(1)) {
            e.kind = ElementKind::Brace {
                tension_only: false,
            };
        }
        assert!(!wall_is_seismic(&data, &model), "ブレースは大梁ではない");

        // 上辺の部材（ElemId 2）を対角（節点 0-2）へ架け替えると不成立になる。
        let (mut model, data) = make_model(150.0);
        for e in model.elements.iter_mut().filter(|e| e.id == ElemId(2)) {
            e.nodes = smallvec::smallvec![NodeId(0), NodeId(2)];
        }
        assert!(!wall_is_seismic(&data, &model), "対角材は上辺に一致しない");
    }

    /// 鋼板耐震壁は RC 規準に由来する条件（スリット・板厚 120mm・開口周比）を
    /// 課さない。鋼板は板厚 9〜16mm 程度のため、材種を問わず課すと成立しなくなる。
    #[test]
    fn 鋼板耐震壁にはrc固有の成立条件を課さない() {
        use squid_n_core::model::WallAttr;

        // 板厚 9mm・大開口・三方スリットのいずれも、鋼板壁では不成立の理由にしない。
        let (mut model, mut data) = make_model(9.0);
        model.materials[0].category = MaterialCategory::Steel;
        model.materials[0].fc = None;
        model.materials[0].fy = Some(235.0);
        // 壁形状 RcWall は RC 壁専用のため、鋼板壁は板厚のみを持つ断面とする。
        model.sections[0].shape = None;
        model.sections[0].thickness = Some(9.0);
        data.section = Some(SectionId(0));
        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area: 3.0e6,
            opening_weight: 0.0,
            three_side_slit: true,
            openings: vec![],
        });
        assert!(!is_rc_wall(&data, &model));
        assert!(
            wall_is_seismic(&data, &model),
            "鋼板耐震壁は板厚・開口・スリットの条件を課さない"
        );

        // 同じ寸法でも RC 壁なら板厚 120mm 未満で不成立になる。
        let (model, data) = make_model(9.0);
        assert!(is_rc_wall(&data, &model));
        assert!(!wall_is_seismic(&data, &model));
    }

    /// 耐震壁と周辺架構の構造種別が食い違うモデルは入力不備として報告する。
    #[test]
    fn 壁と周辺架構の構造種別の食い違いを報告する() {
        // RC 壁＋RC 架構は整合（`add_surrounding_frame` は材料を割り当てないため、
        // 判定が働くよう周辺架構へコンクリートの材料を与える）。
        let (mut model, data) = make_model(150.0);
        for e in model.elements.iter_mut().filter(|e| e.id != ElemId(0)) {
            e.material = Some(MaterialId(0));
        }
        assert!(wall_frame_category_issue(&data, &model).is_none());

        // 周辺架構だけ鋼材へ変えると食い違いとして報告する。
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
        for e in model.elements.iter_mut().filter(|e| e.id == ElemId(1)) {
            e.material = Some(MaterialId(1));
        }
        let msg = wall_frame_category_issue(&data, &model).expect("食い違いを報告する");
        assert!(msg.contains("構造種別を揃えて"), "是正内容を示す: {msg}");
    }

    #[test]
    fn test_collect_misc_walls_and_lengths() {
        // 大開口(2400×1500 @ [800, 750]) → r0=√(3.6e6/12e6)=0.548 > 0.4 で不成立
        let (mut model, data) = make_model(150.0);
        model.elements.push(data);
        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            three_side_slit: false,
            openings: vec![WallOpening {
                width: 2400.0,
                height: 1500.0,
                offset: Some([800.0, 750.0]),
            }],
        });
        let walls = collect_misc_walls(&model);
        assert_eq!(walls.len(), 1);
        let w = &walls[0];
        assert!((w.lw - 4000.0).abs() < 1e-9);
        assert!((w.h - 3000.0).abs() < 1e-9);
        // h/2=1500 は開口 z:[750,2250] 内 → 袖壁長さ: a側=800、b側=4000−3200=800
        assert!((w.wing_length(0) - 800.0).abs() < 1e-9);
        assert!((w.wing_length(1) - 800.0).abs() < 1e-9);
        // lw/2=2000 は開口 x:[800,3200] 内 → 腰壁(下辺梁)=750、垂壁(上辺梁)=3000−2250=750
        assert!((w.strip_height(false) - 750.0).abs() < 1e-9);
        assert!((w.strip_height(true) - 750.0).abs() < 1e-9);
    }

    #[test]
    fn test_misc_wall_without_positioned_opening_splits_half() {
        // 薄壁(t=100)・開口位置なし → 折半則
        let (mut model, data) = make_model(100.0);
        model.elements.push(data);
        let walls = collect_misc_walls(&model);
        assert_eq!(walls.len(), 1);
        assert!((walls[0].wing_length(0) - 2000.0).abs() < 1e-9);
        assert!((walls[0].strip_height(true) - 1500.0).abs() < 1e-9);
    }

    #[test]
    fn test_slit_wall_is_excluded_from_misc_walls() {
        let (mut model, data) = make_model(150.0);
        model.elements.push(data);
        model.wall_attrs.push(WallAttr {
            elem: ElemId(0),
            opening_area: 0.0,
            opening_weight: 0.0,
            three_side_slit: true,
            openings: vec![],
        });
        // スリット壁は耐震壁不成立だが、縁切りのため雑壁算入もしない
        assert!(collect_misc_walls(&model).is_empty());
    }

    #[test]
    fn test_seismic_wall_not_collected() {
        let (mut model, data) = make_model(150.0);
        model.elements.push(data);
        assert!(collect_misc_walls(&model).is_empty());
    }
}
