//! 通り芯の自動生成のテスト。

use super::*;
use crate::dof::Dof6Mask;
use crate::ids::ElemId;
use crate::model::{ElementData, EndCondition, ForceRegime, LocalAxis, Node};

/// 節点を追加して ID を返す。
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

/// 2 節点の線材を追加する。
fn push_line(m: &mut Model, a: NodeId, b: NodeId) -> ElemId {
    let id = ElemId(m.elements.len() as u32);
    m.elements.push(ElementData {
        id,
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![a, b],
        section: None,
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

/// 指定した平面位置に、高さ `h` の柱を 1 本立てる。
fn push_column(m: &mut Model, x: f64, y: f64, h: f64) -> (NodeId, NodeId) {
    let a = push_node(m, x, y, 0.0);
    let b = push_node(m, x, y, h);
    push_line(m, a, b);
    (a, b)
}

/// グループ名 → 通り名の一覧。
fn names(groups: &[AxisGroup], group_name: &str) -> Vec<String> {
    groups
        .iter()
        .find(|g| g.name == group_name)
        .map(|g| g.axes.iter().map(|a| a.name.clone()).collect())
        .unwrap_or_default()
}

/// グループ名 → 通りの離れの一覧。
fn distances(groups: &[AxisGroup], group_name: &str) -> Vec<f64> {
    groups
        .iter()
        .find(|g| g.name == group_name)
        .map(|g| g.axes.iter().filter_map(|a| a.distance).collect())
        .unwrap_or_default()
}

/// 2×2 の柱グリッドから X1・X2 / Y1・Y2 が座標昇順で生成される。
#[test]
fn generates_x_y_axes_in_coordinate_order() {
    let mut m = Model::default();
    for x in [0.0, 6000.0] {
        for y in [0.0, 5000.0] {
            push_column(&mut m, x, y, 3000.0);
        }
    }
    let groups = generate_axes(&m);

    assert_eq!(names(&groups, "X"), vec!["X1", "X2"]);
    assert_eq!(distances(&groups, "X"), vec![0.0, 6000.0]);
    assert_eq!(names(&groups, "Y"), vec!["Y1", "Y2"]);
    assert_eq!(distances(&groups, "Y"), vec![0.0, 5000.0]);

    // 新設グループの幾何は ST-Bridge の規約（X 群 angle=270 / Y 群 angle=0）。
    let x = groups.iter().find(|g| g.name == "X").unwrap();
    assert_eq!(
        x.kind,
        AxisGroupKind::Parallel {
            origin: [0.0, 0.0],
            angle_deg: 270.0
        }
    );
    // 各通りには、その位置に立つ柱の上下 2 節点（2 本ぶん＝4 節点）が属する。
    assert_eq!(x.axes[0].nodes.len(), 4);
    assert!(x.axes.iter().all(|a| a.source == AxisSource::Auto));
}

/// 負の座標を含んでも、番号は座標昇順で単調に増える。
#[test]
fn numbers_ascend_with_coordinate_including_negative() {
    let mut m = Model::default();
    for x in [6000.0, -6000.0, 0.0] {
        push_column(&mut m, x, 0.0, 3000.0);
    }
    let groups = generate_axes(&m);
    assert_eq!(names(&groups, "X"), vec!["X1", "X2", "X3"]);
    assert_eq!(distances(&groups, "X"), vec![-6000.0, 0.0, 6000.0]);
}

/// 許容差 1mm 以内の柱位置は同じ通りにまとまる。
#[test]
fn clusters_within_tolerance() {
    let mut m = Model::default();
    push_column(&mut m, 0.0, 0.0, 3000.0);
    push_column(&mut m, 0.5, 1000.0, 3000.0);
    push_column(&mut m, 6000.0, 0.0, 3000.0);
    let groups = generate_axes(&m);
    assert_eq!(names(&groups, "X"), vec!["X1", "X2"]);
    // 代表値はクラスタ先頭の離れ。
    assert_eq!(distances(&groups, "X"), vec![0.0, 6000.0]);
}

/// 柱でない部材（梁・斜材）は通り芯を作らない。
#[test]
fn ignores_non_vertical_members() {
    let mut m = Model::default();
    let a = push_node(&mut m, 0.0, 0.0, 3000.0);
    let b = push_node(&mut m, 6000.0, 0.0, 3000.0);
    let c = push_node(&mut m, 3000.0, 4000.0, 0.0);
    push_line(&mut m, a, b); // 梁
    push_line(&mut m, a, c); // 斜材
    let groups = generate_axes(&m);
    assert!(groups.is_empty(), "柱がなければグループを作らない");
}

/// 既存の手動・取り込み由来の通りは保護され、同じ位置には新しい通りを作らない。
/// 名前も既存と衝突せず、未使用の最小番号が使われる。
#[test]
fn keeps_manual_axes_and_avoids_name_collision() {
    let mut m = Model::default();
    push_column(&mut m, 0.0, 0.0, 3000.0);
    push_column(&mut m, 6000.0, 0.0, 3000.0);
    push_column(&mut m, 12000.0, 0.0, 3000.0);
    m.axes = vec![AxisGroup {
        name: "X".into(),
        kind: AxisGroupKind::Parallel {
            origin: [0.0, 0.0],
            angle_deg: 270.0,
        },
        axes: vec![
            Axis {
                name: "X1".into(),
                distance: Some(0.0),
                nodes: vec![NodeId(0)],
                source: AxisSource::Manual,
            },
            Axis {
                name: "X2".into(),
                distance: Some(6000.0),
                nodes: vec![],
                source: AxisSource::Manual,
            },
        ],
    }];

    let groups = generate_axes(&m);
    let x = groups.iter().find(|g| g.name == "X").unwrap();
    assert_eq!(x.axes.len(), 3, "既存 2 本 + 新規 1 本");
    // 既存 2 本はそのまま（所属節点も上書きしない）。
    assert_eq!(x.axes[0].name, "X1");
    assert_eq!(x.axes[0].nodes, vec![NodeId(0)]);
    assert_eq!(x.axes[0].source, AxisSource::Manual);
    assert_eq!(x.axes[1].name, "X2");
    assert!(x.axes[1].nodes.is_empty());
    // 新規は未使用の最小番号 X3、位置は 12000。
    assert_eq!(x.axes[2].name, "X3");
    assert_eq!(x.axes[2].distance, Some(12000.0));
    assert_eq!(x.axes[2].source, AxisSource::Auto);
}

/// 既存の自動生成分は毎回作り直される（再実行しても増殖しない）。
#[test]
fn regenerates_auto_axes_idempotently() {
    let mut m = Model::default();
    push_column(&mut m, 0.0, 0.0, 3000.0);
    push_column(&mut m, 6000.0, 0.0, 3000.0);
    m.axes = generate_axes(&m);
    let again = generate_axes(&m);
    assert_eq!(m.axes, again);
}

/// 取り込み由来のグループが別名・別角度でも、幾何で突き合わせてそこへ追加する。
/// 方向角 90° のグループでは離れの符号が反転する。
#[test]
fn adds_into_existing_group_matched_by_geometry() {
    let mut m = Model::default();
    push_column(&mut m, 6000.0, 0.0, 3000.0);
    m.axes = vec![AxisGroup {
        name: "けた行".into(),
        // angle=90° → 離れを測る向きは (-1, 0)。X=6000 の離れは -6000。
        kind: AxisGroupKind::Parallel {
            origin: [0.0, 0.0],
            angle_deg: 90.0,
        },
        axes: Vec::new(),
    }];

    let groups = generate_axes(&m);
    assert!(
        !groups.iter().any(|g| g.name == "X"),
        "X 方向のグループが既にあるので新設しない"
    );
    let g = groups.iter().find(|g| g.name == "けた行").unwrap();
    assert_eq!(g.axes.len(), 1);
    assert_eq!(g.axes[0].name, "けた行1");
    assert_eq!(g.axes[0].distance, Some(-6000.0));
}

/// 円弧芯などの `Other` グループはそのまま保持され、自動生成の対象にならない。
#[test]
fn preserves_other_groups() {
    let mut m = Model::default();
    push_column(&mut m, 0.0, 0.0, 3000.0);
    let arc = AxisGroup {
        name: "R".into(),
        kind: AxisGroupKind::Other,
        axes: vec![Axis {
            name: "R1".into(),
            distance: None,
            nodes: vec![NodeId(0)],
            source: AxisSource::Manual,
        }],
    };
    m.axes = vec![arc.clone()];

    let groups = generate_axes(&m);
    assert_eq!(groups.iter().find(|g| g.name == "R"), Some(&arc));
    assert_eq!(names(&groups, "X"), vec!["X1"]);
}

/// 既存グループと名前が衝突する場合、新設グループの名前をずらす。
#[test]
fn new_group_name_avoids_collision() {
    let mut m = Model::default();
    push_column(&mut m, 0.0, 0.0, 3000.0);
    m.axes = vec![AxisGroup {
        // 名前は "X" だが Y 方向（離れは +Y 向き）のグループ。
        name: "X".into(),
        kind: AxisGroupKind::Parallel {
            origin: [0.0, 0.0],
            angle_deg: 0.0,
        },
        axes: Vec::new(),
    }];

    let groups = generate_axes(&m);
    // Y 方向は既存の "X" グループ（angle=0）へ入り、X 方向は "X2" として新設される。
    assert_eq!(names(&groups, "X"), vec!["X1"]);
    assert_eq!(names(&groups, "X2"), vec!["X21"]);
    assert_eq!(distances(&groups, "X2"), vec![0.0]);
}

/// 所属要素は「すべての材端節点がその通りに属する要素」として算出される。
#[test]
fn axis_elements_collects_members_on_the_axis() {
    let mut m = Model::default();
    // X=0 に柱 2 本（Y=0 と Y=5000）と、それを結ぶ梁（同じ通りの構面）。
    let (_, top_a) = push_column(&mut m, 0.0, 0.0, 3000.0);
    let (_, top_b) = push_column(&mut m, 0.0, 5000.0, 3000.0);
    let beam_on_axis = push_line(&mut m, top_a, top_b);
    // X=6000 の柱と、通りをまたぐ梁。
    let (_, top_c) = push_column(&mut m, 6000.0, 0.0, 3000.0);
    let beam_across = push_line(&mut m, top_a, top_c);

    let groups = generate_axes(&m);
    let x1 = &groups.iter().find(|g| g.name == "X").unwrap().axes[0];
    let elems = m.axis_elements(x1);
    assert!(elems.contains(&beam_on_axis), "通り上の梁は所属する");
    assert!(!elems.contains(&beam_across), "通りをまたぐ梁は所属しない");
    // 柱 2 本 + 梁 1 本。
    assert_eq!(elems.len(), 3);
}
