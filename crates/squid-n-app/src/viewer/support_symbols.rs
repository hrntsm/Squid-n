//! 支点ばね・免震支承の 3D シンボル描画。
//!
//! 従来の支持記号（矢印＝並進固定・円弧＝回転固定、[`super::support::draw_support_symbol`]）とは
//! 別種の支持条件を区別できるよう、専用の記号を追加する。
//! - 支点ばね（[`squid_n_core::model::Node::support_spring`]）: 並進はジグザグ（コイル）線、
//!   回転は渦巻線。拘束（`restraint`）で固定済みの成分は従来どおり矢印・円弧のまま
//!   （呼び出し側で判定し、ばね記号を描かない）。
//! - 免震支承（零長 `ElementKind::Isolator` 要素で支持される節点）: 上下の短い水平線
//!   （フランジプレート）に挟まれた円＋積層を示す横線数本のマーカー。
//!
//! 記号の点列生成（ジグザグ・渦巻）は egui の `Painter` に依存しない純関数へ切り出し、
//! 単体テストで形状を検証する。

use crate::theme;
use squid_n_core::dof::{Dof, Dof6Mask};
use squid_n_core::ids::ElemId;
use squid_n_core::model::{ElementKind, IsolatorProps, Model};
use squid_n_core::units::to_display::force_kn;

use super::Projector;

/// 節点間ジグザグ（コイル）線の点列をスクリーン座標で生成する（純関数）。
///
/// `from`→`to` を `coils` 周期でジグザグさせ、両端は `from`/`to` そのもの。
/// `coils == 0` または `from`/`to` が重なる場合は 2 点（直線扱い）を返す。
pub(super) fn zigzag_points(
    from: egui::Pos2,
    to: egui::Pos2,
    coils: usize,
    amplitude: f32,
) -> Vec<egui::Pos2> {
    let dir = to - from;
    let len = dir.length();
    if len < 1e-3 || coils == 0 {
        return vec![from, to];
    }
    let ux = dir.x / len;
    let uy = dir.y / len;
    // dir に直交する単位ベクトル（画面内で左右に振る向き）
    let nx = -uy;
    let ny = ux;
    let n = coils * 2;
    let mut pts = Vec::with_capacity(n + 2);
    pts.push(from);
    for i in 1..=n {
        let t = i as f32 / (n + 1) as f32;
        let base_x = from.x + dir.x * t;
        let base_y = from.y + dir.y * t;
        let side = if i % 2 == 1 { 1.0 } else { -1.0 };
        pts.push(egui::pos2(
            base_x + nx * amplitude * side,
            base_y + ny * amplitude * side,
        ));
    }
    pts.push(to);
    pts
}

/// 並進ばねのジグザグ（コイル）線を描画する。
pub(super) fn draw_translational_spring(
    painter: &egui::Painter,
    from: egui::Pos2,
    to: egui::Pos2,
    color: egui::Color32,
) {
    /// コイルの周期数（多すぎず視認できる程度）
    const COILS: usize = 4;
    /// ジグザグの振幅 [px]（固定 px＝非テキスト形状のため可、TONMANUAL §4）
    const AMPLITUDE: f32 = 3.0;
    let stroke = egui::Stroke::new(2.0_f32, color);
    let pts = zigzag_points(from, to, COILS, AMPLITUDE);
    painter.add(egui::Shape::line(pts, stroke));
}

/// 渦巻線の (半径比, 角度[rad]) 列を生成する（純関数）。
///
/// 半径比は 0.0→1.0 へ、角度は 0→`turns`*2π へ、ともに単調に増加する
/// （アルキメデス螺旋。回転固定の単一円弧と視覚的に区別するための形状）。
/// `n == 0` の場合は空を返す。
pub(super) fn spiral_fracs(turns: f64, n: usize) -> Vec<(f64, f64)> {
    if n == 0 {
        return Vec::new();
    }
    (0..=n)
        .map(|i| {
            let t = i as f64 / n as f64;
            (t, t * turns * std::f64::consts::TAU)
        })
        .collect()
}

/// 渦巻の周回数（回転固定の単一円弧と区別できる程度）。
const SPIRAL_TURNS: f64 = 1.5;
/// 渦巻の分割数。
const SPIRAL_SEGMENTS: usize = 40;

/// スクリーン平面上に渦巻アイコンを描く（凡例など、3D 投影を経ない用途）。
pub(super) fn draw_spiral_icon_2d(
    painter: &egui::Painter,
    center: egui::Pos2,
    radius: f32,
    color: egui::Color32,
) {
    let stroke = egui::Stroke::new(1.5_f32, color);
    let pts: Vec<egui::Pos2> = spiral_fracs(SPIRAL_TURNS, SPIRAL_SEGMENTS)
        .into_iter()
        .map(|(r_frac, theta)| {
            let r = radius * r_frac as f32;
            egui::pos2(
                center.x + r * (theta.cos() as f32),
                center.y + r * (theta.sin() as f32),
            )
        })
        .collect();
    painter.add(egui::Shape::line(pts, stroke));
}

/// 回転ばねの渦巻線を 3D 空間（節点まわり、`axis` に直交する面内）に描く。
/// 基底の作り方は [`super::support::draw_rotation_arc`] と同じ（[`super::support::axis_basis`]）。
pub(super) fn draw_rotational_spring(
    painter: &egui::Painter,
    proj: &Projector,
    center_world: [f64; 3],
    axis: [f64; 3],
    radius_world: f64,
    color: egui::Color32,
) {
    let Some((u, v)) = super::support::axis_basis(axis) else {
        return;
    };
    let stroke = egui::Stroke::new(1.5_f32, color);
    let mut prev: Option<egui::Pos2> = None;
    for (r_frac, theta) in spiral_fracs(SPIRAL_TURNS, SPIRAL_SEGMENTS) {
        let r = radius_world * r_frac;
        let c = theta.cos();
        let s = theta.sin();
        let pt = [
            center_world[0] + r * (c * u[0] + s * v[0]),
            center_world[1] + r * (c * u[1] + s * v[1]),
            center_world[2] + r * (c * u[2] + s * v[2]),
        ];
        let cur = proj.project(pt);
        if let Some(p0) = prev {
            painter.line_segment([p0, cur], stroke);
        }
        prev = Some(cur);
    }
}

/// 支点ばねシンボルを 3D ビューに描画する。
///
/// 拘束（`restraint`）で固定済みの成分は従来の矢印・円弧表示に委ねるため、
/// ここでは非固定かつばね値が非ゼロの成分のみジグザグ・渦巻を描く。
/// 軸色は X=赤 / Y=緑 / Z=青（TONMANUAL §3-2）で [`super::support::draw_support_symbol`] と揃える。
pub(super) fn draw_spring_symbol(
    painter: &egui::Painter,
    proj: &Projector,
    node_coord: [f64; 3],
    restraint: Dof6Mask,
    spring: &[f64; 6],
    arrow_px: f32,
    arc_px: f32,
) {
    let arrow_world = arrow_px as f64 / proj.scale() as f64;
    let arc_world = arc_px as f64 / proj.scale() as f64;
    let origin = proj.project(node_coord);

    let translational: [(Dof, [f64; 3], egui::Color32); 3] = [
        (Dof::Ux, [1.0, 0.0, 0.0], theme::AXIS_X),
        (Dof::Uy, [0.0, 1.0, 0.0], theme::AXIS_Y),
        (Dof::Uz, [0.0, 0.0, 1.0], theme::AXIS_Z),
    ];
    for (i, (dof, dir, color)) in translational.into_iter().enumerate() {
        if restraint.is_fixed(dof) || spring[i] == 0.0 {
            continue;
        }
        let end = [
            node_coord[0] + dir[0] * arrow_world,
            node_coord[1] + dir[1] * arrow_world,
            node_coord[2] + dir[2] * arrow_world,
        ];
        draw_translational_spring(painter, origin, proj.project(end), color);
    }

    let rotational: [(Dof, [f64; 3], egui::Color32); 3] = [
        (Dof::Rx, [1.0, 0.0, 0.0], theme::AXIS_X),
        (Dof::Ry, [0.0, 1.0, 0.0], theme::AXIS_Y),
        (Dof::Rz, [0.0, 0.0, 1.0], theme::AXIS_Z),
    ];
    for (i, (dof, axis, color)) in rotational.into_iter().enumerate() {
        let spring_idx = i + 3;
        if restraint.is_fixed(dof) || spring[spring_idx] == 0.0 {
            continue;
        }
        draw_rotational_spring(painter, proj, node_coord, axis, arc_world, color);
    }
}

/// 免震支承マーカーの幾何（スクリーン座標、`center` 中心のローカル配置）。
/// 純関数として切り出し、点列・線分の対称性を単体テストで検証する。
pub(super) struct IsolatorMarkerGeometry {
    pub flange_top: [egui::Pos2; 2],
    pub flange_bottom: [egui::Pos2; 2],
    pub layers: Vec<[egui::Pos2; 2]>,
    pub circle_center: egui::Pos2,
    pub circle_radius: f32,
}

/// 免震支承マーカーの幾何を生成する（純関数）。
///
/// 上下 2 本の水平線（フランジプレート）＋中央の円（支承本体）＋円内の
/// 水平な積層線（`n_layers` 本）という配置。円・積層線は上下のフランジ線の
/// 内側（中心寄り）に収める。
pub(super) fn isolator_marker_geometry(
    center: egui::Pos2,
    flange_half_width: f32,
    flange_gap: f32,
    circle_radius: f32,
    n_layers: usize,
) -> IsolatorMarkerGeometry {
    let top_y = center.y - flange_gap;
    let bottom_y = center.y + flange_gap;
    let flange_top = [
        egui::pos2(center.x - flange_half_width, top_y),
        egui::pos2(center.x + flange_half_width, top_y),
    ];
    let flange_bottom = [
        egui::pos2(center.x - flange_half_width, bottom_y),
        egui::pos2(center.x + flange_half_width, bottom_y),
    ];
    let layer_half_width = flange_half_width * 0.6;
    let mut layers = Vec::with_capacity(n_layers);
    for i in 0..n_layers {
        let t = if n_layers <= 1 {
            0.5
        } else {
            i as f32 / (n_layers as f32 - 1.0)
        };
        // 円の内側（半径の約半分の範囲）に均等配置
        let y = center.y + (t * 2.0 - 1.0) * circle_radius * 0.5;
        layers.push([
            egui::pos2(center.x - layer_half_width, y),
            egui::pos2(center.x + layer_half_width, y),
        ]);
    }
    IsolatorMarkerGeometry {
        flange_top,
        flange_bottom,
        layers,
        circle_center: center,
        circle_radius,
    }
}

/// フランジ線の半幅 [px]（固定 px＝非テキスト形状のため可、TONMANUAL §4）。
const ISOLATOR_FLANGE_HALF_WIDTH: f32 = 8.0;
/// フランジ線の中心からの上下オフセット [px]。
const ISOLATOR_FLANGE_GAP: f32 = 9.0;
/// 支承本体円の半径 [px]。
const ISOLATOR_CIRCLE_RADIUS: f32 = 6.0;
/// 積層を示す横線の本数。
const ISOLATOR_N_LAYERS: usize = 2;

/// 免震支承マーカーを描画する（上下フランジ線＋支承本体円＋積層線）。
/// 支点への配置に依らず、他の記号（矢印・円弧・ばねのジグザグ／渦巻）と
/// 弁別できる専用色（[`theme::ISOLATOR_TEAL`]）で描く。
pub(super) fn draw_isolator_marker(
    painter: &egui::Painter,
    center: egui::Pos2,
    color: egui::Color32,
) {
    let geo = isolator_marker_geometry(
        center,
        ISOLATOR_FLANGE_HALF_WIDTH,
        ISOLATOR_FLANGE_GAP,
        ISOLATOR_CIRCLE_RADIUS,
        ISOLATOR_N_LAYERS,
    );
    let flange_stroke = egui::Stroke::new(1.5_f32, color);
    painter.line_segment(geo.flange_top, flange_stroke);
    painter.line_segment(geo.flange_bottom, flange_stroke);
    painter.circle_stroke(geo.circle_center, geo.circle_radius, flange_stroke);
    let layer_stroke = egui::Stroke::new(1.0_f32, color);
    for l in &geo.layers {
        painter.line_segment(*l, layer_stroke);
    }
}

/// 免震支承で支持される対象節点の一覧を返す（節点 index・要素 ID・諸元）。
///
/// 支点配置は「接地節点（`restraint == Dof6Mask::FIXED`）と対象節点の間の
/// 零長 `ElementKind::Isolator` 要素」（申し送り仕様）。零長でない・両端 FIXED・
/// 両端非 FIXED の要素は支点配置のパターンに合致しないため対象外とする
/// （前者は一般部材としての免震要素、後2つは支点判定が一意に定まらない）。
pub(super) fn support_isolators(model: &Model) -> Vec<(usize, ElemId, IsolatorProps)> {
    let mut out = Vec::new();
    for elem in &model.elements {
        if elem.kind != ElementKind::Isolator || elem.nodes.len() != 2 {
            continue;
        }
        let Some((a_idx, a)) = model
            .nodes
            .iter()
            .enumerate()
            .find(|(_, n)| n.id == elem.nodes[0])
        else {
            continue;
        };
        let Some((b_idx, b)) = model
            .nodes
            .iter()
            .enumerate()
            .find(|(_, n)| n.id == elem.nodes[1])
        else {
            continue;
        };
        if a.coord != b.coord {
            continue;
        }
        let a_fixed = a.restraint == Dof6Mask::FIXED;
        let b_fixed = b.restraint == Dof6Mask::FIXED;
        let target_idx = match (a_fixed, b_fixed) {
            (true, false) => b_idx,
            (false, true) => a_idx,
            _ => continue,
        };
        if let Some(props) = model
            .isolator_attrs
            .iter()
            .find(|attr| attr.elem == elem.id)
            .map(|attr| attr.props)
        {
            out.push((target_idx, elem.id, props));
        }
    }
    out
}

/// 免震支承のホバー詳細ツールチップ（種別・主要諸元）。
/// 検定比図・モデル化図・ヒンジ図と同じ方針（`#[allow(deprecated)]`）で
/// `show_tooltip_at_pointer` を使用する（[`super`] のホバー実装を参照）。
pub(super) fn show_isolator_tooltip(ui: &egui::Ui, elem_id: ElemId, props: &IsolatorProps) {
    #[allow(deprecated)]
    egui::show_tooltip_at_pointer(
        ui.ctx(),
        ui.layer_id(),
        egui::Id::new("isolator_support_tooltip"),
        |ui| {
            ui.label(format!("免震支承（要素 #{}）", elem_id.0));
            ui.colored_label(
                theme::ISOLATOR_TEAL,
                crate::tables::nodes::isolator_kind_label(props.kind),
            );
            ui.label(format!(
                "K1={:.0}N/mm K2={:.0}N/mm Qd={:.1}kN Kv={:.0}N/mm μ={:.3}",
                props.k1,
                props.k2,
                force_kn(props.qd),
                props.kv,
                props.mu
            ));
        },
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    // ----- zigzag_points -----

    #[test]
    fn zigzag_points_endpoints_match_from_to() {
        let from = egui::pos2(0.0, 0.0);
        let to = egui::pos2(20.0, 0.0);
        let pts = zigzag_points(from, to, 4, 3.0);
        assert_eq!(pts.first().copied(), Some(from));
        assert_eq!(pts.last().copied(), Some(to));
        // 4 コイル → 内部点 8 個 + 両端 = 10 点
        assert_eq!(pts.len(), 10);
    }

    #[test]
    fn zigzag_points_alternates_perpendicular_side() {
        let from = egui::pos2(0.0, 0.0);
        let to = egui::pos2(20.0, 0.0);
        let pts = zigzag_points(from, to, 2, 3.0);
        // 水平線分なので直交方向は Y。内部点の y は符号が交互になる。
        let interior: Vec<f32> = pts[1..pts.len() - 1].iter().map(|p| p.y).collect();
        assert!(interior.len() >= 2);
        for w in interior.windows(2) {
            assert!(
                w[0] * w[1] < 0.0,
                "隣接する内部点は符号が反転するはず: {w:?}"
            );
        }
        for y in interior {
            assert!((y.abs() - 3.0).abs() < 1e-4);
        }
    }

    #[test]
    fn zigzag_points_zero_coils_or_degenerate_returns_two_points() {
        let from = egui::pos2(0.0, 0.0);
        let to = egui::pos2(20.0, 0.0);
        assert_eq!(zigzag_points(from, to, 0, 3.0), vec![from, to]);
        // from == to（長さ 0）も直線扱い（2 点）に落とす
        assert_eq!(zigzag_points(from, from, 4, 3.0), vec![from, from]);
    }

    // ----- spiral_fracs -----

    #[test]
    fn spiral_fracs_monotonic_and_bounded() {
        let fracs = spiral_fracs(1.5, 10);
        assert_eq!(fracs.len(), 11);
        assert_eq!(fracs[0], (0.0, 0.0));
        let (last_r, last_theta) = *fracs.last().unwrap();
        assert!((last_r - 1.0).abs() < 1e-12);
        assert!((last_theta - 1.5 * std::f64::consts::TAU).abs() < 1e-9);
        for w in fracs.windows(2) {
            assert!(w[1].0 >= w[0].0, "半径比は単調非減少のはず");
            assert!(w[1].1 >= w[0].1, "角度は単調非減少のはず");
        }
    }

    #[test]
    fn spiral_fracs_zero_segments_is_empty() {
        assert!(spiral_fracs(1.5, 0).is_empty());
    }

    // ----- isolator_marker_geometry -----

    #[test]
    fn isolator_marker_geometry_is_symmetric_about_center() {
        let center = egui::pos2(100.0, 50.0);
        let geo = isolator_marker_geometry(center, 8.0, 9.0, 6.0, 2);
        // フランジ線は中心を挟んで対称（上下とも中心からの距離が等しい）
        assert!((geo.flange_top[0].y - center.y + 9.0).abs() < 1e-6);
        assert!((geo.flange_bottom[0].y - center.y - 9.0).abs() < 1e-6);
        // フランジ線は水平（両端の y が一致）かつ中心を x 方向にまたぐ
        assert_eq!(geo.flange_top[0].y, geo.flange_top[1].y);
        assert!(geo.flange_top[0].x < center.x && geo.flange_top[1].x > center.x);
        // 積層線の本数が指定どおり、いずれも水平でフランジ線より短い
        assert_eq!(geo.layers.len(), 2);
        for l in &geo.layers {
            assert_eq!(l[0].y, l[1].y);
            let layer_width = l[1].x - l[0].x;
            let flange_width = geo.flange_top[1].x - geo.flange_top[0].x;
            assert!(layer_width < flange_width);
        }
        assert_eq!(geo.circle_center, center);
        assert_eq!(geo.circle_radius, 6.0);
    }

    #[test]
    fn isolator_marker_geometry_zero_layers_has_no_layer_lines() {
        let geo = isolator_marker_geometry(egui::pos2(0.0, 0.0), 8.0, 9.0, 6.0, 0);
        assert!(geo.layers.is_empty());
    }

    // ----- support_isolators -----

    use squid_n_core::ids::NodeId;
    use squid_n_core::model::{
        ElementData, EndCondition, ForceRegime, IsolatorAttr, IsolatorKind, LocalAxis, Node,
    };

    fn ground_and_target(ground_restraint: Dof6Mask, target_restraint: Dof6Mask) -> Model {
        let mut model = Model::default();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: ground_restraint,
            mass: None,
            story: None,
            support_spring: None,
        });
        model.nodes.push(Node {
            id: NodeId(1),
            coord: [0.0, 0.0, 0.0],
            restraint: target_restraint,
            mass: None,
            story: None,
            support_spring: None,
        });
        let elem_id = ElemId(0);
        model.elements.push(ElementData {
            id: elem_id,
            kind: ElementKind::Isolator,
            nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
            section: None,
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
        model.isolator_attrs.push(IsolatorAttr {
            elem: elem_id,
            props: IsolatorProps {
                kind: IsolatorKind::LeadRubber,
                ..IsolatorProps::default()
            },
        });
        model
    }

    #[test]
    fn support_isolators_finds_target_when_other_side_fixed() {
        let model = ground_and_target(Dof6Mask::FIXED, Dof6Mask::FREE);
        let found = support_isolators(&model);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].0, 1); // 対象節点（非固定側）の index
        assert_eq!(found[0].1, ElemId(0));
        assert_eq!(found[0].2.kind, IsolatorKind::LeadRubber);
    }

    #[test]
    fn support_isolators_order_independent() {
        // 接地節点が nodes[1] 側でも対象節点（nodes[0]）を正しく拾う
        let model = ground_and_target(Dof6Mask::FREE, Dof6Mask::FIXED);
        let found = support_isolators(&model);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].0, 0);
    }

    #[test]
    fn support_isolators_excludes_both_fixed_or_both_free() {
        let both_fixed = ground_and_target(Dof6Mask::FIXED, Dof6Mask::FIXED);
        assert!(support_isolators(&both_fixed).is_empty());
        let both_free = ground_and_target(Dof6Mask::FREE, Dof6Mask::FREE);
        assert!(support_isolators(&both_free).is_empty());
    }

    #[test]
    fn support_isolators_excludes_non_zero_length() {
        let mut model = ground_and_target(Dof6Mask::FIXED, Dof6Mask::FREE);
        model.nodes[1].coord = [1000.0, 0.0, 0.0];
        assert!(support_isolators(&model).is_empty());
    }
}
