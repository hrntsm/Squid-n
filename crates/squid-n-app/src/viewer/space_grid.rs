//! 3D ビューの立体グリッド（通り芯 × 階レベル）の描画と、格子点へのスナップ。
//!
//! 通り芯は構造計算に用いない識別用のデータだが、モデリングの下敷きとしては有用で
//! ある。通りと階が決まっていれば柱・大梁を置く位置は格子点に限られるため、そこへ
//! スナップできると節点を 1 つずつ作らずに部材を引ける。
//!
//! # 何を描くか
//!
//! **各レベルの平面格子だけ**を描き、鉛直線は描かない。鉛直線は柱と重なる位置に
//! 引かれるため、柱のある架構では情報が増えず線だけが増える。通り名は最下レベルの
//! 端にだけ添える（全レベルに繰り返すと文字だらけになる）。
//!
//! # スナップの規則
//!
//! 格子点（通りの交点 × 階レベル）をピックの候補に加え、**既存節点を優先**する。
//! 同じ位置に節点を重ねて作ると、見た目では気づけない二重節点ができるためである。
//! 節点のない格子点を選んだ場合は、部材の生成と同じ 1 回の undo で節点も作る。

use squid_n_core::dof::Dof6Mask;
use squid_n_core::frame_gen::{space_grid, GridLine, SpaceGrid};
use squid_n_core::geom::{default_local_ref_vector, is_vertical_pair};
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Model};
use squid_n_edit::{AddMember, AddNode, CompositeCommand, EditCommand};

use super::Projector;
use crate::theme;

/// 格子点のピック許容距離 [px]（既存節点のピックと同じ）。
pub const GRID_PICK_THRESHOLD: f32 = 10.0;

/// 同じ座標の節点とみなす許容差 [mm]。
const NODE_MERGE_TOL_MM: f64 = 1.0;

/// 立体グリッドを描けるか（通り芯と階の両方があるか）。
pub fn has_grid(model: &Model) -> bool {
    !space_grid(model).is_empty()
}

/// 平面格子を描く（各レベルの X 通り・Y 通りの線と、最下レベルの通り名）。
pub fn draw(painter: &egui::Painter, proj: &Projector, model: &Model) {
    let grid = space_grid(model);
    if grid.is_empty() {
        return;
    }
    let (x0, x1) = span(&grid.x_lines);
    let (y0, y1) = span(&grid.y_lines);
    // 補助線なので、部材より淡い色で沈める。
    let stroke = egui::Stroke::new(1.0_f32, theme::translucent(theme::GRAY_600, 70));
    for lv in &grid.levels {
        let z = lv.elevation;
        for gx in &grid.x_lines {
            let seg = [
                proj.project([gx.coord, y0, z]),
                proj.project([gx.coord, y1, z]),
            ];
            painter.line_segment(seg, stroke);
        }
        for gy in &grid.y_lines {
            let seg = [
                proj.project([x0, gy.coord, z]),
                proj.project([x1, gy.coord, z]),
            ];
            painter.line_segment(seg, stroke);
        }
    }

    // 通り名は最下レベルの端にだけ添える。格子の外側へ少し逃がして線と重ねない。
    let Some(base) = grid.levels.first() else {
        return;
    };
    let z = base.elevation;
    let pad = ((x1 - x0).max(y1 - y0) * 0.04).max(500.0);
    let font = egui::FontId::proportional(11.0);
    for gx in &grid.x_lines {
        painter.text(
            proj.project([gx.coord, y0 - pad, z]),
            egui::Align2::CENTER_CENTER,
            &gx.name,
            font.clone(),
            theme::GRAY_600,
        );
    }
    for gy in &grid.y_lines {
        painter.text(
            proj.project([x0 - pad, gy.coord, z]),
            egui::Align2::CENTER_CENTER,
            &gy.name,
            font.clone(),
            theme::GRAY_600,
        );
    }
}

/// 格子線列の両端の座標（昇順に並んでいる前提。空なら 0）。
fn span(lines: &[GridLine]) -> (f64, f64) {
    let lo = lines.first().map(|l| l.coord).unwrap_or(0.0);
    let hi = lines.last().map(|l| l.coord).unwrap_or(0.0);
    (lo, hi)
}

/// 部材作成モードでクリックした点の解決先。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SnapPoint {
    /// 既存の節点。
    Node(NodeId),
    /// 節点のない格子点（部材の生成とあわせて節点を作る）。
    Grid([f64; 3]),
}

impl SnapPoint {
    /// 作成モードの案内に出すラベル。
    pub fn label(&self) -> String {
        match self {
            SnapPoint::Node(n) => format!("N{}", n.0),
            SnapPoint::Grid(_) => "格子点".to_string(),
        }
    }
}

/// クリック位置を、既存節点または格子点へスナップする。
///
/// 既存節点が許容距離内にあればそれを優先し、無ければ格子点を探す。どちらも
/// 見つからなければ `None`。
pub fn pick(
    model: &Model,
    proj: &Projector,
    pts: &[egui::Pos2],
    node_visible: &[bool],
    click: egui::Pos2,
) -> Option<SnapPoint> {
    if let Some((i, d)) = super::pick_nearest_node(pts, node_visible, click) {
        if d <= GRID_PICK_THRESHOLD {
            return Some(SnapPoint::Node(model.nodes[i].id));
        }
    }
    nearest_grid_point(&space_grid(model), proj, click).map(SnapPoint::Grid)
}

/// 画面上でクリック位置にもっとも近い格子点の座標（許容距離内のみ）。
fn nearest_grid_point(grid: &SpaceGrid, proj: &Projector, click: egui::Pos2) -> Option<[f64; 3]> {
    let mut best: Option<([f64; 3], f32)> = None;
    for p in grid.points() {
        let d = (proj.project(p) - click).length();
        if d <= GRID_PICK_THRESHOLD && best.is_none_or(|(_, bd)| d < bd) {
            best = Some((p, d));
        }
    }
    best.map(|(p, _)| p)
}

/// スナップ点を節点 ID へ解決する。節点のない格子点は `pending` へ積み、
/// [`AddNode`] が末尾へ追加したときの ID を先に返す。
///
/// 既存節点と同じ座標の格子点はその節点を使い回し、二重節点を作らない。
/// `pending` 内の重複も同じ規則で畳む。
fn resolve(model: &Model, pending: &mut Vec<[f64; 3]>, point: SnapPoint) -> NodeId {
    let c = match point {
        SnapPoint::Node(n) => return n,
        SnapPoint::Grid(c) => c,
    };
    // 既存節点に同じ座標があればそれを使う（表示対象外の節点も拾う）。
    if let Some(n) = model.nodes.iter().find(|n| same_coord(n.coord, c)) {
        return n.id;
    }
    if let Some(i) = pending.iter().position(|p| same_coord(*p, c)) {
        return NodeId((model.nodes.len() + i) as u32);
    }
    pending.push(c);
    NodeId((model.nodes.len() + pending.len() - 1) as u32)
}

/// 2 点が同じ位置か（[`NODE_MERGE_TOL_MM`] 以内）。
fn same_coord(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() <= NODE_MERGE_TOL_MM)
}

/// 2 つのスナップ点から部材を作るコマンドと、生成される部材 ID を組み立てる。
///
/// 節点のない格子点にはその場で節点を作り、節点追加と部材追加を 1 つの
/// [`CompositeCommand`] にまとめる。これにより undo 1 回で節点ごと取り消せる。
/// 2 点が同じ節点へ解決される場合は長さ 0 の部材になるため `None` を返す。
///
/// 局所座標系の基準ベクトルは、材軸が鉛直なら グローバル X、それ以外は
/// グローバル Z とする（[`squid_n_core::frame_gen`] の柱・大梁と同じ規則）。
/// 鉛直材へ Z を与えると材軸と平行になり、基準ベクトルとして働かないためである。
pub fn beam_command(
    model: &Model,
    a: SnapPoint,
    b: SnapPoint,
) -> Option<(CompositeCommand, ElemId)> {
    let mut pending: Vec<[f64; 3]> = Vec::new();
    let na = resolve(model, &mut pending, a);
    let nb = resolve(model, &mut pending, b);
    if na == nb {
        return None;
    }
    let ref_vector = default_local_ref_vector(is_vertical(model, &pending, na, nb));
    let mut children: Vec<Box<dyn EditCommand>> = pending
        .iter()
        .map(|c| {
            Box::new(AddNode {
                coord: *c,
                restraint: Dof6Mask::FREE,
            }) as Box<dyn EditCommand>
        })
        .collect();
    let elem_id = ElemId(model.elements.len() as u32);
    children.push(Box::new(AddMember {
        elem: ElementData {
            id: elem_id,
            kind: ElementKind::Beam,
            nodes: [na, nb].into_iter().collect(),
            section: None,
            local_axis: LocalAxis { ref_vector },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        },
    }));
    Some((
        CompositeCommand {
            label: "梁追加".to_string(),
            children,
        },
        elem_id,
    ))
}

/// 材軸が鉛直か（[`is_vertical_pair`] と同規約）。
///
/// まだモデルに無い節点は `pending` 側から座標を引く（`AddNode` は末尾へ追加する
/// ため、`model.nodes.len()` 以降の ID が `pending` の添字に対応する）。
fn is_vertical(model: &Model, pending: &[[f64; 3]], a: NodeId, b: NodeId) -> bool {
    let coord = |n: NodeId| -> Option<[f64; 3]> {
        match model.nodes.get(n.index()) {
            Some(nd) => Some(nd.coord),
            None => pending.get(n.index() - model.nodes.len()).copied(),
        }
    };
    let (Some(ca), Some(cb)) = (coord(a), coord(b)) else {
        return false;
    };
    is_vertical_pair(ca, cb)
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::frame_gen::{frame_model, FrameSpec};

    /// 既存節点どうしを結ぶときは節点を作らず、部材 1 本だけを追加する。
    #[test]
    fn test_beam_between_existing_nodes() {
        let mut model = frame_model(&FrameSpec::default()).unwrap();
        let n_nodes = model.nodes.len();
        let n_elems = model.elements.len();
        let (cmd, id) = beam_command(
            &model,
            SnapPoint::Node(NodeId(0)),
            SnapPoint::Node(NodeId(5)),
        )
        .expect("別の節点なので梁を作れる");
        cmd.apply(&mut model);
        assert_eq!(model.nodes.len(), n_nodes);
        assert_eq!(model.elements.len(), n_elems + 1);
        assert_eq!(id, ElemId(n_elems as u32));
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    /// 節点のない格子点を選ぶと、節点追加と部材追加が 1 回の undo にまとまる。
    #[test]
    fn test_grid_point_creates_node_in_one_undo() {
        let spec = FrameSpec {
            with_girders: false,
            with_slabs: false,
            ..FrameSpec::default()
        };
        let mut model = frame_model(&spec).unwrap();
        let n_nodes = model.nodes.len();
        let n_elems = model.elements.len();
        // 格子の外の点を 2 つ選ぶ（どちらも既存節点と重ならない座標）。
        let a = SnapPoint::Grid([20000.0, 0.0, 4000.0]);
        let b = SnapPoint::Grid([20000.0, 6000.0, 4000.0]);
        let (cmd, _) = beam_command(&model, a, b).unwrap();
        let inverse = cmd.apply(&mut model);
        assert_eq!(model.nodes.len(), n_nodes + 2);
        assert_eq!(model.elements.len(), n_elems + 1);
        assert!(model.validate().is_ok(), "{:?}", model.validate());

        // undo 1 回で節点ごと戻る。
        inverse.apply(&mut model);
        assert_eq!(model.nodes.len(), n_nodes);
        assert_eq!(model.elements.len(), n_elems);
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    /// 既存節点と同じ座標の格子点は、その節点を使い回して二重節点を作らない。
    #[test]
    fn test_grid_point_reuses_existing_node() {
        let mut model = frame_model(&FrameSpec::default()).unwrap();
        let n_nodes = model.nodes.len();
        let c0 = model.nodes[0].coord;
        let (cmd, _) = beam_command(
            &model,
            SnapPoint::Grid(c0),
            SnapPoint::Grid([20000.0, 0.0, 4000.0]),
        )
        .unwrap();
        cmd.apply(&mut model);
        assert_eq!(model.nodes.len(), n_nodes + 1, "重なる 1 点は再利用する");
        let elem = model.elements.last().unwrap();
        assert_eq!(elem.nodes[0], NodeId(0));
    }

    /// 同じ点を 2 度選んでも長さ 0 の部材は作らない。
    #[test]
    fn test_same_point_is_rejected() {
        let model = frame_model(&FrameSpec::default()).unwrap();
        assert!(beam_command(
            &model,
            SnapPoint::Node(NodeId(0)),
            SnapPoint::Node(NodeId(0))
        )
        .is_none());
        let c0 = model.nodes[0].coord;
        assert!(beam_command(&model, SnapPoint::Node(NodeId(0)), SnapPoint::Grid(c0)).is_none());
    }

    /// 鉛直材の局所座標系の基準ベクトルはグローバル X にする。
    ///
    /// 鉛直材へ Z を与えると材軸と平行になり、基準ベクトルとして働かない。
    /// 格子点スナップでは上下のレベルを結んで柱を引けるため、水平材と同じ
    /// 基準ベクトルを使い回せない。
    #[test]
    fn test_vertical_member_uses_x_reference() {
        let mut model = frame_model(&FrameSpec::default()).unwrap();
        // 同じ平面位置で 2F・3F の格子点を結ぶ（＝柱）。
        let (cmd, id) = beam_command(
            &model,
            SnapPoint::Grid([20000.0, 0.0, 4000.0]),
            SnapPoint::Grid([20000.0, 0.0, 7500.0]),
        )
        .unwrap();
        cmd.apply(&mut model);
        assert_eq!(
            model.elements[id.index()].local_axis.ref_vector,
            [1.0, 0.0, 0.0]
        );

        // 水平材はこれまでどおりグローバル Z。
        let (cmd, id) = beam_command(
            &model,
            SnapPoint::Grid([20000.0, 0.0, 4000.0]),
            SnapPoint::Grid([20000.0, 6000.0, 4000.0]),
        )
        .unwrap();
        cmd.apply(&mut model);
        assert_eq!(
            model.elements[id.index()].local_axis.ref_vector,
            [0.0, 0.0, 1.0]
        );
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    /// 通り芯のないモデルでは格子を描かない。
    #[test]
    fn test_has_grid() {
        assert!(has_grid(&frame_model(&FrameSpec::default()).unwrap()));
        assert!(!has_grid(&Model::default()));
    }
}
