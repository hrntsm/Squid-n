//! RC 柱の短期設計せん断力用 ΣMy（崩壊メカニズム判定）。
//!
//! 参照実装マニュアル 2.5.3:
//! - 梁降伏形の端 → 寄与 = 梁 My の和 / 2
//! - 柱降伏形の端 → 寄与 = 柱 My
//! - 上下端とも梁降伏は考えない（両端が梁降伏なら下端を柱降伏へ落とす）
//! - 部分スリット壁が取り付く端は常に柱ヒンジ（モデル属性が無い間は呼び出し側が
//!   `force_column_hinge_*` で渡す。未配線時は false）
//!
//! 各端の判定（図 2.5.3-2 の実務再構成）:
//! - 強制柱ヒンジ、または加力方向の梁が無い、または 柱 My ≤ 梁 My 和 → 柱ヒンジ
//! - それ以外 → 梁ヒンジ

use squid_n_core::adjacency::NodeAdjacency;
use squid_n_core::model::{ElementData, ElementKind, Model};
use squid_n_core::rc_capacity::{rc_column_mu_simple, rc_mu_simple, RcCapacityInput};
use squid_n_core::section_shape::SectionShape;
use squid_n_element::transform::LocalFrame;

use super::section_props::{bar_set_area, rect_axis_props_strong, rect_axis_props_weak};
use crate::material_strength::rebar_sigma_y_of;
use crate::MemberKind;

/// 柱端のヒンジ種別。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ColumnEndHinge {
    /// 梁ヒンジ（寄与 = ΣbeamMy / 2）。
    Beam,
    /// 柱ヒンジ（寄与 = columnMy）。
    Column,
}

/// 1 端のヒンジを判定する。
///
/// `force_column_hinge`: 部分スリット壁等でその端を強制的に柱ヒンジにする場合 true。
pub fn resolve_column_end_hinge(
    column_my: f64,
    sum_beam_my: f64,
    force_column_hinge: bool,
) -> ColumnEndHinge {
    if force_column_hinge || sum_beam_my <= 1e-9 || column_my <= sum_beam_my + 1e-9 {
        ColumnEndHinge::Column
    } else {
        ColumnEndHinge::Beam
    }
}

/// 両端ヒンジから ΣMy を組み立てる。両端とも梁降伏なら下端を柱降伏へ落とす。
pub fn sum_my_from_end_hinges(
    hinge_i: ColumnEndHinge,
    hinge_j: ColumnEndHinge,
    column_my_i: f64,
    column_my_j: f64,
    sum_beam_my_i: f64,
    sum_beam_my_j: f64,
    // true なら端 i を「下端」として両端梁降伏時に柱へ落とす。
    i_is_bottom: bool,
) -> f64 {
    let (mut hi, mut hj) = (hinge_i, hinge_j);
    if hi == ColumnEndHinge::Beam && hj == ColumnEndHinge::Beam {
        if i_is_bottom {
            hi = ColumnEndHinge::Column;
        } else {
            hj = ColumnEndHinge::Column;
        }
    }
    let contrib = |h: ColumnEndHinge, col_my: f64, beam_sum: f64| match h {
        ColumnEndHinge::Column => col_my,
        ColumnEndHinge::Beam => 0.5 * beam_sum,
    };
    contrib(hi, column_my_i, sum_beam_my_i) + contrib(hj, column_my_j, sum_beam_my_j)
}

/// 設計軸力 N = NL + n·|NE|（圧縮正）。`n` は引張正の部材軸力。
pub fn design_axial_for_mechanism(n_long: f64, n_combo: f64, n_factor: f64) -> f64 {
    let n_l = (-n_long).max(0.0); // 長期圧縮（引張長期は 0）
    let n_e = (n_combo - n_long).abs();
    n_l + n_factor.max(0.0) * n_e
}

fn horizontal_unit(v: [f64; 3]) -> Option<[f64; 2]> {
    let h = [v[0], v[1]];
    let n = (h[0] * h[0] + h[1] * h[1]).sqrt();
    if n < 1e-9 {
        None
    } else {
        Some([h[0] / n, h[1] / n])
    }
}

fn alignment_score(load_h: [f64; 2], axis_h: [f64; 2]) -> f64 {
    let cos = load_h[0] * axis_h[0] + load_h[1] * axis_h[1];
    cos * cos
}

/// 梁を強軸/弱軸のどちらか一方にだけ割り当てる。
///
/// 45° 付近で両方向にヒットしないよう、スコアが大きい方を採用する。
/// 同点（ちょうど 45°）は `prefer_on_tie` が true の方向（強軸）に寄せる。
fn aligns_exclusively(
    load_h: [f64; 2],
    peer_h: [f64; 2],
    axis_h: [f64; 2],
    prefer_on_tie: bool,
) -> bool {
    let s = alignment_score(load_h, axis_h);
    if s < 0.5 {
        return false;
    }
    let sp = alignment_score(peer_h, axis_h);
    s > sp + 1e-12 || ((s - sp).abs() <= 1e-12 && prefer_on_tie)
}

fn beam_my_simple(model: &Model, elem: &ElementData) -> Option<f64> {
    let sec = elem
        .section
        .and_then(|sid| model.sections.get(sid.index()))?;
    let rebar = match &sec.shape {
        Some(SectionShape::RcRect { rebar, .. }) | Some(SectionShape::RcCircle { rebar, .. }) => {
            rebar
        }
        _ => return None,
    };
    let props = match &sec.shape {
        Some(SectionShape::RcRect { .. }) => rect_axis_props_strong(sec, rebar),
        Some(SectionShape::RcCircle { d, .. }) => {
            // 円形梁は稀。等価として強軸 props を円から取る経路は
            // `circle_axis_props` だが、ここでは矩形梁のみ対象とする。
            let _ = d;
            return None;
        }
        _ => return None,
    };
    let sigma_y = rebar_sigma_y_of(model.element_rebar_material(elem));
    if sigma_y <= 0.0 || props.at <= 0.0 || props.d <= 0.0 {
        return None;
    }
    let inp = RcCapacityInput {
        b: props.b,
        d: props.d_full,
        at: props.at,
        d_eff: props.d,
        sigma_y,
        fc: 0.0,
        pw: 0.0,
        sigma_wy: 0.0,
        clear_span: 0.0,
        sigma_0: 0.0,
    };
    Some(rc_mu_simple(&inp))
}

fn column_my_at_n(
    model: &Model,
    elem: &ElementData,
    n_axial: f64,
    // true = 強軸（mz / qy）、false = 弱軸（my / qz）。
    strong: bool,
) -> Option<f64> {
    let sec = elem
        .section
        .and_then(|sid| model.sections.get(sid.index()))?;
    let mat = model.element_material(elem)?;
    let fc = mat.fc.filter(|&v| v > 0.0)?;
    let (rebar, props, b, d_full, as_total) = match &sec.shape {
        Some(SectionShape::RcRect { rebar, .. }) => {
            let props = if strong {
                rect_axis_props_strong(sec, rebar)
            } else {
                rect_axis_props_weak(sec, rebar)
            };
            let (b, d_full) = if strong {
                (sec.width, sec.depth)
            } else {
                (sec.depth, sec.width)
            };
            let as_total = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
            (rebar, props, b, d_full, as_total)
        }
        Some(SectionShape::RcCircle { d, rebar }) => {
            let props = super::section_props::circle_axis_props(*d, rebar);
            let as_total =
                rebar.main_x.count as f64 * super::section_props::one_bar_area(rebar.main_x.dia);
            let b_eq = std::f64::consts::PI * d * d / 4.0 / d;
            (rebar, props, b_eq, *d, as_total)
        }
        _ => return None,
    };
    let _ = rebar;
    let sigma_y = rebar_sigma_y_of(model.element_rebar_material(elem));
    if sigma_y <= 0.0 {
        return None;
    }
    let inp = RcCapacityInput {
        b,
        d: d_full,
        at: props.at,
        d_eff: props.d,
        sigma_y,
        fc,
        pw: props.pw,
        sigma_wy: 0.0,
        clear_span: 0.0,
        sigma_0: 0.0,
    };
    Some(rc_column_mu_simple(&inp, as_total, n_axial))
}

fn sum_beam_my_at_node(
    model: &Model,
    adjacency: &NodeAdjacency,
    node: squid_n_core::ids::NodeId,
    load_h: [f64; 2],
    peer_h: [f64; 2],
    prefer_on_tie: bool,
) -> Result<f64, ()> {
    let mut sum = 0.0;
    for other in adjacency.elements_at(model, node) {
        if other.kind != ElementKind::Beam || other.nodes.len() < 2 {
            continue;
        }
        if MemberKind::of_element(other, model) != MemberKind::Beam {
            continue;
        }
        let (Some(n0), Some(n1)) = (
            other.nodes.first().and_then(|n| model.nodes.get(n.index())),
            other.nodes.get(1).and_then(|n| model.nodes.get(n.index())),
        ) else {
            continue;
        };
        let axis = [
            n1.coord[0] - n0.coord[0],
            n1.coord[1] - n0.coord[1],
            n1.coord[2] - n0.coord[2],
        ];
        let Some(axis_h) = horizontal_unit(axis) else {
            continue;
        };
        if !aligns_exclusively(load_h, peer_h, axis_h, prefer_on_tie) {
            continue;
        }
        // 加力方向に梁はあるが My が取れない（SRC・円形・入力不足等）→
        // 梁無し扱いにすると ΣMy が過小になり得るため Err とし、呼び出し側で
        // 端軸力ベースの柱 Mu 和（2·Mu 相当）へ落とす。
        let Some(my) = beam_my_simple(model, other) else {
            return Err(());
        };
        sum += my;
    }
    Ok(sum)
}

/// 柱のメカニズム ΣMy。戻り値は `(強軸=qy 用, 弱軸=qz 用)`。
///
/// 各方向は通常 `Some(ΣMy)`。柱 My 自体が取れないときだけその方向は `None`
/// （呼び出し側が検定位置軸力の `2·Mu` で代替）。
/// 梁 My が欠落した方向は、端の設計軸力による柱 Mu 和（`Mu_i+Mu_j`）を返す。
/// 外側の `None` は柱でない・幾何が取れない場合。
///
/// `n_*` は引張正。`n_axial_factor` はマニュアルの n（ルート 2-3 で 2.0、それ以外 1.0）。
/// 部分スリット壁の強制柱ヒンジはモデル未対応のため常に false。
#[allow(clippy::too_many_arguments)]
pub fn compute_column_mechanism_sum_my(
    model: &Model,
    adjacency: &NodeAdjacency,
    column: &ElementData,
    n_long_i: f64,
    n_long_j: f64,
    n_combo_i: f64,
    n_combo_j: f64,
    n_axial_factor: f64,
) -> Option<(Option<f64>, Option<f64>)> {
    if column.kind != ElementKind::Beam || column.nodes.len() < 2 {
        return None;
    }
    if MemberKind::of_element(column, model) != MemberKind::Column {
        return None;
    }
    let (Some(ni), Some(nj)) = (column.nodes.first().copied(), column.nodes.get(1).copied()) else {
        return None;
    };
    let (Some(p_i), Some(p_j)) = (
        model.nodes.get(ni.index()).map(|n| n.coord),
        model.nodes.get(nj.index()).map(|n| n.coord),
    ) else {
        return None;
    };
    let frame = LocalFrame::from_nodes(p_i, p_j, column.local_axis.ref_vector);
    let ey = frame.rot[1];
    let ez = frame.rot[2];
    let load_strong = horizontal_unit(ey)?; // qy / mz
    let load_weak = horizontal_unit(ez)?; // qz / my

    let n_i = design_axial_for_mechanism(n_long_i, n_combo_i, n_axial_factor);
    let n_j = design_axial_for_mechanism(n_long_j, n_combo_j, n_axial_factor);
    let i_is_bottom = p_i[2] <= p_j[2];

    let sum_for = |strong: bool, load_h: [f64; 2], peer_h: [f64; 2]| -> Option<f64> {
        let col_i = column_my_at_n(model, column, n_i, strong)?;
        let col_j = column_my_at_n(model, column, n_j, strong)?;
        let beam_i = sum_beam_my_at_node(model, adjacency, ni, load_h, peer_h, strong);
        let beam_j = sum_beam_my_at_node(model, adjacency, nj, load_h, peer_h, strong);
        match (beam_i, beam_j) {
            (Ok(bi), Ok(bj)) => {
                let hi = resolve_column_end_hinge(col_i, bi, false);
                let hj = resolve_column_end_hinge(col_j, bj, false);
                Some(sum_my_from_end_hinges(
                    hi,
                    hj,
                    col_i,
                    col_j,
                    bi,
                    bj,
                    i_is_bottom,
                ))
            }
            // 梁 My 欠落 → 端軸力ベースの柱 Mu 和（2·Mu 相当）で確定。
            _ => Some(col_i + col_j),
        }
    };

    Some((
        sum_for(true, load_strong, load_weak),
        sum_for(false, load_weak, load_strong),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hinge_column_when_no_beams_or_weaker_beams() {
        assert_eq!(
            resolve_column_end_hinge(100.0, 0.0, false),
            ColumnEndHinge::Column
        );
        assert_eq!(
            resolve_column_end_hinge(100.0, 100.0, false),
            ColumnEndHinge::Column
        );
        assert_eq!(
            resolve_column_end_hinge(100.0, 80.0, false),
            ColumnEndHinge::Beam
        );
        assert_eq!(
            resolve_column_end_hinge(100.0, 200.0, true),
            ColumnEndHinge::Column
        );
    }

    #[test]
    fn both_beam_hinges_force_bottom_to_column() {
        // 両端梁 → 下端(i) を柱へ。ΣMy = col_i + 0.5*beam_j
        let s = sum_my_from_end_hinges(
            ColumnEndHinge::Beam,
            ColumnEndHinge::Beam,
            100.0,
            120.0,
            300.0,
            400.0,
            true,
        );
        assert!((s - (100.0 + 0.5 * 400.0)).abs() < 1e-9, "s={s}");
    }

    #[test]
    fn mixed_hinges_add_contributions() {
        // i 柱 + j 梁 → col_i + 0.5*beam_j
        let s = sum_my_from_end_hinges(
            ColumnEndHinge::Column,
            ColumnEndHinge::Beam,
            100.0,
            120.0,
            300.0,
            400.0,
            true,
        );
        assert!((s - (100.0 + 200.0)).abs() < 1e-9);
    }

    #[test]
    fn design_axial_adds_abs_earthquake() {
        // NL=1000 (n_long=-1000), NE 増分 200 → N=1200
        let n = design_axial_for_mechanism(-1000.0, -1200.0, 1.0);
        assert!((n - 1200.0).abs() < 1e-9);
        let n2 = design_axial_for_mechanism(-1000.0, -1200.0, 2.0);
        assert!((n2 - 1400.0).abs() < 1e-9);
    }

    /// 加力方向の梁 My が取れない（SRC 等）とその方向のメカニズムは破棄される。
    #[test]
    fn missing_beam_my_discards_mechanism_direction() {
        use smallvec::smallvec;
        use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
        use squid_n_core::model::{
            ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material,
            MaterialCategory, Node, RigidZone,
        };
        use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};
        use squid_n_core::Dof6Mask;

        let rebar = RcRebar {
            cover: 40.0,
            main_x: BarSet {
                dia: 22.0,
                count: 4,
                layers: 1,
            },
            main_y: BarSet {
                dia: 22.0,
                count: 4,
                layers: 1,
            },
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
            },
        };
        let col_shape = SectionShape::RcRect {
            b: 500.0,
            d: 500.0,
            rebar: rebar.clone(),
        };
        let beam_shape = SectionShape::SrcRect {
            b: 300.0,
            d: 600.0,
            rebar,
            steel_height: 400.0,
            steel_width: 200.0,
            steel_web_thick: 9.0,
            steel_flange_thick: 16.0,
        };
        let materials = vec![
            Material {
                id: MaterialId(0),
                name: "Fc24".into(),
                category: MaterialCategory::Concrete,
                young: 21_000.0,
                poisson: 0.2,
                density: 2.4e-9,
                shear: None,
                fc: Some(24.0),
                fy: None,
                concrete_class: Default::default(),
                strength_factor: None,
            },
            Material {
                id: MaterialId(1),
                name: "SD345".into(),
                category: MaterialCategory::Rebar,
                young: 205_000.0,
                poisson: 0.3,
                density: 7.85e-9,
                shear: None,
                fc: None,
                fy: Some(345.0),
                concrete_class: Default::default(),
                strength_factor: None,
            },
        ];
        let mut col_sec = col_shape.to_section(SectionId(0), "C".into());
        col_sec.material = Some(MaterialId(0));
        col_sec.rebar_material = Some(MaterialId(1));
        let mut beam_sec = beam_shape.to_section(SectionId(1), "B-SRC".into());
        beam_sec.material = Some(MaterialId(0));
        beam_sec.rebar_material = Some(MaterialId(1));

        let line = |id: u32, n0: u32, n1: u32, sid: u32| ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(n0), NodeId(n1)],
            section: Some(SectionId(sid)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        };
        let node = |id: u32, x: f64, y: f64, z: f64| Node {
            id: NodeId(id),
            coord: [x, y, z],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        let mut model = Model {
            nodes: vec![
                node(0, 0.0, 0.0, 0.0),
                node(1, 0.0, 0.0, 4000.0),
                node(2, 6000.0, 0.0, 0.0),
                node(3, 6000.0, 0.0, 4000.0),
            ],
            elements: vec![
                line(0, 0, 1, 0), // 柱
                line(1, 0, 2, 1), // 下梁 SRC
                line(2, 1, 3, 1), // 上梁 SRC
            ],
            sections: vec![col_sec, beam_sec],
            materials,
            ..Default::default()
        };
        // 柱の強軸たわみ方向 ey が X（梁方向）を向く。
        model.elements[0].local_axis.ref_vector = [1.0, 0.0, 0.0];

        let adj = NodeAdjacency::build(&model);
        let sum = compute_column_mechanism_sum_my(
            &model,
            &adj,
            &model.elements[0],
            -800_000.0,
            -800_000.0,
            -1_000_000.0,
            -1_000_000.0,
            1.0,
        )
        .expect("柱として算定できる");
        assert!(
            sum.0.is_some(),
            "SRC 梁付き方向は端軸力の Mu_i+Mu_j で確定: {sum:?}"
        );
        assert!(sum.1.is_some(), "梁無し方向は柱ヒンジで Some: {sum:?}");
        // 両方向とも柱 Mu 和になるため同値（正方形柱）。
        assert!(
            (sum.0.unwrap() - sum.1.unwrap()).abs() / sum.0.unwrap() < 1e-9,
            "sum={sum:?}"
        );
    }

    #[test]
    fn diagonal_beam_assigns_to_one_direction_only() {
        let axis = [1.0_f64 / 2.0_f64.sqrt(), 1.0 / 2.0_f64.sqrt()];
        let strong = [1.0, 0.0];
        let weak = [0.0, 1.0];
        assert!(aligns_exclusively(strong, weak, axis, true));
        assert!(!aligns_exclusively(weak, strong, axis, false));
    }
}
