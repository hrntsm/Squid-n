//! 構面の切り出しのテスト。

use super::*;
use crate::dof::Dof6Mask;
use crate::ids::{ElemId, NodeId};
use crate::model::{
    Axis, AxisGroup, AxisGroupKind, AxisSource, ElementData, EndCondition, ForceRegime, LocalAxis,
    Node, Story,
};

fn push_node(m: &mut Model, x: f64, y: f64, z: f64) -> NodeId {
    let id = NodeId(m.nodes.len() as u32);
    m.nodes.push(Node {
        id,
        coord: [x, y, z],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    });
    id
}

fn push_line(m: &mut Model, a: NodeId, b: NodeId) -> ElemId {
    let id = ElemId(m.elements.len() as u32);
    m.elements.push(ElementData {
        id,
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![a, b],
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed; 2],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });
    id
}

/// X 方向の平行芯グループ 1 つ（原点 (0,0)・方向角 270°）を持つモデル。
fn x_group(axes: Vec<Axis>) -> AxisGroup {
    AxisGroup {
        name: "X".into(),
        kind: AxisGroupKind::Parallel {
            origin: [0.0, 0.0],
            angle_deg: 270.0,
        },
        axes,
    }
}

fn auto_axis(name: &str, distance: f64, nodes: Vec<NodeId>) -> Axis {
    Axis {
        name: name.into(),
        distance: Some(distance),
        nodes,
        source: AxisSource::Auto,
    }
}

/// 通りの法線は、平行芯グループの幾何から厳密に決まる（離れを測る向き）。
/// 通りに属する部材は、その通り上の節点だけで構成される要素になる。
#[test]
fn axis_frame_uses_exact_geometry_and_selects_in_plane_members() {
    let mut m = Model::default();
    // X=0 の構面（柱 2 本と、それを結ぶ梁）。
    let a0 = push_node(&mut m, 0.0, 0.0, 0.0);
    let a1 = push_node(&mut m, 0.0, 0.0, 4000.0);
    let b0 = push_node(&mut m, 0.0, 6000.0, 0.0);
    let b1 = push_node(&mut m, 0.0, 6000.0, 4000.0);
    let col_a = push_line(&mut m, a0, a1);
    let col_b = push_line(&mut m, b0, b1);
    let girder = push_line(&mut m, a1, b1);
    // X=6000 の柱と、通りをまたぐ梁。
    let c0 = push_node(&mut m, 6000.0, 0.0, 0.0);
    let c1 = push_node(&mut m, 6000.0, 0.0, 4000.0);
    let col_c = push_line(&mut m, c0, c1);
    let cross = push_line(&mut m, a1, c1);

    m.axes = vec![x_group(vec![auto_axis("X1", 0.0, vec![a0, a1, b0, b1])])];

    let f = build_frame(&m, FrameTarget::Axis { group: 0, axis: 0 }).expect("構面");
    assert_eq!(f.label, "X1 通り");
    // 方向角 270° の離れは +X 向き＝この構面の法線。
    assert_eq!(f.normal, [1.0, 0.0, 0.0]);
    assert!(f.elem_on[col_a.index()], "通り上の柱");
    assert!(f.elem_on[col_b.index()], "通り上の柱");
    assert!(f.elem_on[girder.index()], "通り上の梁");
    assert!(!f.elem_on[col_c.index()], "別の通りの柱");
    assert!(!f.elem_on[cross.index()], "通りをまたぐ梁");
    assert_eq!(f.elem_count(), 3);
}

/// 所属節点リストにない中間節点があっても、座標が通り上にあれば拾う。
/// リストだけで判定すると、分割された大梁が丸ごと落ちる。
#[test]
fn axis_frame_includes_split_girder_via_coordinates() {
    let mut m = Model::default();
    let a0 = push_node(&mut m, 0.0, 0.0, 0.0);
    let a1 = push_node(&mut m, 0.0, 0.0, 4000.0);
    let b0 = push_node(&mut m, 0.0, 6000.0, 0.0);
    let b1 = push_node(&mut m, 0.0, 6000.0, 4000.0);
    // 大梁が中間節点で 2 要素に割れている（中間節点は柱節点ではない）。
    let mid = push_node(&mut m, 0.0, 3000.0, 4000.0);
    push_line(&mut m, a0, a1);
    push_line(&mut m, b0, b1);
    let g1 = push_line(&mut m, a1, mid);
    let g2 = push_line(&mut m, mid, b1);

    // 自動生成の通りは柱の材端節点しか持たない。
    m.axes = vec![x_group(vec![auto_axis("X1", 0.0, vec![a0, a1, b0, b1])])];

    let f = build_frame(&m, FrameTarget::Axis { group: 0, axis: 0 }).expect("構面");
    assert!(f.elem_on[g1.index()], "分割された大梁の 1 本目");
    assert!(f.elem_on[g2.index()], "分割された大梁の 2 本目");
    assert!(f.node_on[mid.index()], "中間節点も通り上");
}

/// 芯ずれ（通りの位置と柱の芯が一致しない）した部材は、所属節点リストで拾う。
/// 座標だけで判定すると落ちる。
#[test]
fn axis_frame_includes_offset_columns_via_node_list() {
    let mut m = Model::default();
    // 通りは X=3000 だが、柱は X=3500 に立つ。
    let a0 = push_node(&mut m, 3500.0, 0.0, 0.0);
    let a1 = push_node(&mut m, 3500.0, 0.0, 4000.0);
    let col = push_line(&mut m, a0, a1);
    m.axes = vec![x_group(vec![Axis {
        name: "X1a".into(),
        distance: Some(3000.0),
        nodes: vec![a0, a1],
        source: AxisSource::Manual,
    }])];

    let f = build_frame(&m, FrameTarget::Axis { group: 0, axis: 0 }).expect("構面");
    assert!(f.elem_on[col.index()], "芯ずれした柱も通りに属する");
}

/// 平行芯以外（円弧芯など）のグループは幾何を持たないため、所属節点群へ
/// 平面を当てはめて法線を求める。
#[test]
fn other_group_fits_plane_from_nodes() {
    let mut m = Model::default();
    // x = y の鉛直面に並ぶ柱 2 本。
    let a0 = push_node(&mut m, 0.0, 0.0, 0.0);
    let a1 = push_node(&mut m, 0.0, 0.0, 4000.0);
    let b0 = push_node(&mut m, 5000.0, 5000.0, 0.0);
    let b1 = push_node(&mut m, 5000.0, 5000.0, 4000.0);
    push_line(&mut m, a0, a1);
    push_line(&mut m, b0, b1);
    m.axes = vec![AxisGroup {
        name: "R".into(),
        kind: AxisGroupKind::Other,
        axes: vec![Axis {
            name: "R1".into(),
            distance: None,
            nodes: vec![a0, a1, b0, b1],
            source: AxisSource::Manual,
        }],
    }];

    let f = build_frame(&m, FrameTarget::Axis { group: 0, axis: 0 }).expect("構面");
    let s = 1.0 / 2.0_f64.sqrt();
    let dot = f.normal[0] * s - f.normal[1] * s;
    assert!((dot.abs() - 1.0).abs() < 1e-6, "法線: {:?}", f.normal);
    // 幾何がないため座標では拾えず、所属節点リストだけが頼りになる。
    assert_eq!(f.elem_count(), 2);
}

/// 階（伏図）は、その階の梁に加えて上端節点がその階に属する柱を含める。
/// 柱を落とすと梁の支持位置が図から読めなくなる。
#[test]
fn story_frame_includes_columns_below() {
    let mut m = Model::default();
    let base_a = push_node(&mut m, 0.0, 0.0, 0.0);
    let base_b = push_node(&mut m, 6000.0, 0.0, 0.0);
    let top_a = push_node(&mut m, 0.0, 0.0, 4000.0);
    let top_b = push_node(&mut m, 6000.0, 0.0, 4000.0);
    let col_a = push_line(&mut m, base_a, top_a);
    let col_b = push_line(&mut m, base_b, top_b);
    let girder = push_line(&mut m, top_a, top_b);
    // 上階へ伸びる柱（上端は 2 階レベルなので 1 階の伏図には出ない）。
    let up_a = push_node(&mut m, 0.0, 0.0, 8000.0);
    let col_up = push_line(&mut m, top_a, up_a);

    let story = StoryId(0);
    for n in [top_a, top_b] {
        m.nodes[n.index()].story = Some(story);
    }
    m.nodes[up_a.index()].story = Some(StoryId(1));
    m.stories.push(Story {
        id: story,
        name: "2FL".into(),
        elevation: 4000.0,
        node_ids: vec![top_a, top_b],
        seismic_weight: None,
        weight_override: None,
        structure: Default::default(),
        level_kind: Default::default(),
    });

    let f = build_frame(&m, FrameTarget::Story(story)).expect("構面");
    assert_eq!(f.label, "2FL");
    assert_eq!(f.normal, [0.0, 0.0, 1.0], "伏図の法線は鉛直");
    assert!(f.elem_on[girder.index()], "その階の梁");
    assert!(f.elem_on[col_a.index()], "上端がその階の柱");
    assert!(f.elem_on[col_b.index()], "上端がその階の柱");
    assert!(!f.elem_on[col_up.index()], "上階へ伸びる柱は含まない");
    assert_eq!(f.elem_count(), 3);
}

/// 存在しない対象を指定した場合は `None`（通り芯の再生成で添字がずれた場合など）。
#[test]
fn missing_target_returns_none() {
    let m = Model::default();
    assert!(build_frame(&m, FrameTarget::Axis { group: 0, axis: 0 }).is_none());
    assert!(build_frame(&m, FrameTarget::Story(StoryId(0))).is_none());
}

/// 所属部材が 0 本の通り（ST-Bridge から取り込んだ `Y0` など）でも構面は作れる。
/// 空であることは呼び出し側が [`Frame::elem_count`] で判断する。
#[test]
fn empty_axis_builds_frame_with_no_elements() {
    let mut m = Model::default();
    push_node(&mut m, 6000.0, 0.0, 0.0);
    m.axes = vec![x_group(vec![auto_axis("X0", 0.0, Vec::new())])];
    let f = build_frame(&m, FrameTarget::Axis { group: 0, axis: 0 }).expect("構面");
    assert_eq!(f.elem_count(), 0);
    assert_eq!(f.normal, [1.0, 0.0, 0.0], "法線は幾何から決まる");
}
