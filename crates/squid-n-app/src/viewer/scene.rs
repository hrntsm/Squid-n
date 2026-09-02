//! 部材描画・スラブ/壁・グリッド・軸ガジェット。
//!
//! `viewer` ハブからの構造分割。アルゴリズム変更は行わない。

use crate::app::App;
use crate::theme;

use super::{
    camera::q_rotate, support::draw_arrow, CameraState, DiagramPlane, FrameFilter, Projector,
};

pub(super) fn diagram_offset_dir(
    p_i: [f64; 3],
    p_j: [f64; 3],
    ref_vector: [f64; 3],
    plane: DiagramPlane,
) -> [f64; 3] {
    let frame = squid_n_element::transform::LocalFrame::from_nodes(p_i, p_j, ref_vector);
    match plane {
        DiagramPlane::Ey => frame.rot[1],
        DiagramPlane::Ez => frame.rot[2],
    }
}

/// 応力図の張り出し方向を構面内へ倒す（2D 構面表示）。
///
/// 3D では成分ごとに部材の局所 ey / ez 面へ張り出すが、構面を正対で見ると
/// **面外へ張り出す成分は視線方向に潰れて線になり、何も読めなくなる**
/// （たとえば局所 ez が構面の法線と平行な梁の My 図）。そこで構面表示では、
/// 張り出し方向を「材軸に直交し、かつ構面に含まれる向き」へ倒す。値と図形は
/// 変えず向きだけを回すため、読み取れる数値は 3D と同じである。どの成分の図かは
/// 成分ごとの固定色と凡例・数値ラベルで判別する。
///
/// 倒した先の向きは `normal × 材軸`。符号は元の張り出し方向に合わせ、元の方向が
/// 構面に垂直（成分が完全に面外）で符号を決められない場合は正側へ倒す。
pub(super) fn in_plane_offset_dir(
    dir: [f64; 3],
    p_i: [f64; 3],
    p_j: [f64; 3],
    normal: [f64; 3],
) -> [f64; 3] {
    let axis = [p_j[0] - p_i[0], p_j[1] - p_i[1], p_j[2] - p_i[2]];
    let t = cross3(normal, axis);
    let len = (t[0] * t[0] + t[1] * t[1] + t[2] * t[2]).sqrt();
    if len < 1e-12 {
        // 材軸が構面の法線と平行（構面を貫く部材）。面内に張り出し方向を採れない。
        return dir;
    }
    let t = [t[0] / len, t[1] / len, t[2] / len];
    let sign = dir[0] * t[0] + dir[1] * t[1] + dir[2] * t[2];
    if sign < 0.0 {
        [-t[0], -t[1], -t[2]]
    } else {
        t
    }
}

/// 材軸を持つ線材か（先頭 2 節点を結ぶ線分＝部材線として描いてよい要素か）。
///
/// 壁・シェルは面要素で材軸を持たない（壁は多角形として別に描く）。仕口パネルは
/// 「接合部の節点 ＋ 取り付く部材の他端」を節点列に持つ接合部要素であり、先頭
/// 2 節点は取り付く柱・梁そのものと同じ節点対になる（`pick_nearest_member` が
/// ピック対象から外しているのと同じ理由）。
pub(super) fn draws_as_line(kind: squid_n_core::model::ElementKind) -> bool {
    element_draw_shape(kind) == DrawShape::Line
}

/// 3D ビューでの要素の描き方。部材線・選択ハイライトとも本区分に従う。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum DrawShape {
    /// 材軸の線分（先頭 2 節点を結ぶ）。
    Line,
    /// 節点列の多角形（面要素）。
    Polygon,
    /// 描かない。
    None,
}

/// 要素種別ごとの描き方。部材線（[`draws_as_line`]）・面要素のポリゴン・選択
/// ハイライトが同じ規約を共有するための単一情報源。
///
/// 壁・シェルは面要素で材軸を持たないため多角形で描く。仕口パネルは「接合部の
/// 節点 ＋ 取り付く部材の他端」を節点列に持つ接合部要素で、材軸も輪郭も持たない
/// ため描かない（`pick_nearest_member` がピック対象から外しているのと同じ理由）。
/// 先頭 2 節点を結ぶと取り付く柱・梁とまったく同じ線分になるため、線で描くと
/// 内部たわみ表示で部材が二重に見え、選択ハイライトでは選択していない柱・梁が
/// 選択されているように見えてしまう。
///
/// 要素種別を追加したときに描き方を決め忘れないよう、網羅 `match` で書く。
pub(super) fn element_draw_shape(kind: squid_n_core::model::ElementKind) -> DrawShape {
    use squid_n_core::model::ElementKind as K;
    match kind {
        K::Wall | K::Shell => DrawShape::Polygon,
        K::PanelZone => DrawShape::None,
        K::Beam
        | K::Fiber
        | K::MultiSpring
        | K::Brace { .. }
        | K::NodalSpring
        | K::Isolator
        | K::Damper => DrawShape::Line,
    }
}

use squid_n_core::geom::vec3::cross as cross3;

/// 荷重分配オブジェクト（床板・要素にならない壁版）の破線パターン（描画長 / 間隔, px）。
const PLATE_DASH: f32 = 6.0;
const PLATE_GAP: f32 = 4.0;
/// 同・塗りの不透明度。実部材（壁エレメントは 50）より薄くして手前に出さない。
const PLATE_FILL_ALPHA: u8 = 28;
/// 同・輪郭の不透明度。
const PLATE_STROKE_ALPHA: u8 = 220;

/// 荷重分配オブジェクトの多角形（淡い半透明フィル＋破線の輪郭）を 1 枚描く。
///
/// 床板と要素にならない壁版で書式を共有する。**線種が「解析要素かどうか」を、
/// 色が「床か壁か」を表す**規約であり、実部材（実線・濃い塗り）と弁別できる。
fn draw_load_plate_polygon(
    painter: &egui::Painter,
    coords: &[[f64; 3]],
    proj: &Projector,
    color: egui::Color32,
    fill: bool,
) {
    let poly: Vec<egui::Pos2> = coords.iter().copied().map(|c| proj.project(c)).collect();
    if poly.len() < 3 {
        return;
    }
    if fill {
        painter.add(egui::Shape::convex_polygon(
            poly.clone(),
            theme::translucent(color, PLATE_FILL_ALPHA),
            egui::Stroke::NONE,
        ));
    }
    let mut closed = poly.clone();
    closed.push(poly[0]);
    painter.extend(egui::Shape::dashed_line(
        &closed,
        egui::Stroke::new(1.5_f32, theme::translucent(color, PLATE_STROKE_ALPHA)),
        PLATE_DASH,
        PLATE_GAP,
    ));
}

/// 床板（版）の輪郭・塗り。大梁または小梁で囲まれた床板と、取り付く床板の両方を描く。
/// 二次部材（小梁・間柱）は実部材と同じ経路（`draw_mode_rest_ghost` の二次部材
/// ブロック）で別途描画する。
///
/// `coords3` は節点インデックス順の**表示用 3D 座標**で、変形図・モード形・時刻歴では
/// 変形後の位置が入る。周囲の梁が変形するのに床板だけが元の位置に残ると、床が梁から
/// 浮いて見えて変形の読み取りを妨げるため、床板も同じ座標へ載せる。
pub(super) fn draw_slabs(
    painter: &egui::Painter,
    app: &App,
    filter: FrameFilter,
    proj: &Projector,
    coords3: &[[f64; 3]],
) {
    for slab in &app.model.slabs {
        if !slab_visible_on_frame(slab, filter) {
            continue;
        }
        let Some(coords) = slab.boundary_coords_with(|n| coords3.get(n.index()).copied()) else {
            continue;
        };
        // 面は淡い半透明の暖色フィル（壁の青と弁別）。
        draw_load_plate_polygon(painter, &coords, proj, theme::BEST_YELLOW, true);
    }
}

/// 要素にならない壁版の輪郭・塗り。
///
/// 壁版は入力なので、解析要素になるか否かにかかわらず図に出す。要素になる壁版
/// （[`squid_n_core::model::Model::wall_plate_becomes_element`]）は壁エレメントとして
/// 実線で描かれるため、ここでは描かない（同じ多角形の二重描画を避ける）。
///
/// 対象は要素にならない壁版すべてで、間柱で分割された壁版・5 節点以上の壁領域内の
/// 壁版・断面未割当の壁版に加え、取り付く壁版（腰壁・垂壁・パラペット・自立壁）も含む。
/// 取付き先が点（`RegionAnchor::Point`）の壁版は座標を組み立てられないため描けない
/// （壁の取付き先としては使わない組み合わせ。[`squid_n_core::model::WallPlate`] 参照）。
///
/// `coords3` の意味は [`draw_slabs`] と同じ（変形図では変形後の座標が入る）。
pub(super) fn draw_wall_plates(
    painter: &egui::Painter,
    app: &App,
    filter: FrameFilter,
    proj: &Projector,
    coords3: &[[f64; 3]],
) {
    for plate in &app.model.wall_plates {
        if app.model.wall_plate_becomes_element(plate) {
            continue;
        }
        if !wall_plate_visible_on_frame(plate, filter) {
            continue;
        }
        let Some(coords) =
            plate.boundary_coords_with(&app.model, |n| coords3.get(n.index()).copied())
        else {
            continue;
        };
        // 色は「壁」を表す青（床板の暖色と弁別）。線種の破線が「解析要素ではない」を
        // 表し、壁エレメント（青・実線・濃い塗り）と区別が付く。
        draw_load_plate_polygon(
            painter,
            &coords,
            proj,
            theme::DATA_BLUE,
            plate_fill_is_valid(&app.model, plate),
        );
    }
}

/// 壁版の内部を塗ってよいか。
///
/// 取り付く壁版の立ち上がり高さが両端で符号反転すると、境界の 4 点が自己交差する
/// 蝶ネクタイ形になる（[`squid_n_core::model::WallPlate::area`] がこの形を避けて
/// 面積を積分で求めているのと同じ理由）。塗りは凸多角形を前提に三角形へ分割する
/// ため、この形では輪郭からはみ出す。輪郭の破線だけを描いて、入力が異常である
/// ことをそのまま見せる。
pub(super) fn plate_fill_is_valid(
    model: &squid_n_core::Model,
    plate: &squid_n_core::model::WallPlate,
) -> bool {
    use squid_n_core::model::WallPlateShape;
    match &plate.shape {
        // 高さが解決できない壁版（階高を引けない自立壁）は形が定まらない。
        // `boundary_coords_with` も `None` を返すのでここへは来ないが、塗って
        // よいかを判定する側で「解決できたときだけ真」を保っておく。
        WallPlateShape::Attached { .. } => model
            .wall_plate_extent(plate)
            .is_some_and(|e| e[0] * e[1] >= 0.0),
        WallPlateShape::Enclosed { .. } => true,
    }
}

/// 壁版が現在の構面表示に含まれるか（[`slab_visible_on_frame`] の壁版版）。
pub(super) fn wall_plate_visible_on_frame(
    plate: &squid_n_core::model::WallPlate,
    filter: FrameFilter,
) -> bool {
    use squid_n_core::model::{RegionAnchor, WallPlateShape};
    match &plate.shape {
        WallPlateShape::Enclosed { boundary } => {
            boundary.iter().all(|n| filter.shows_node(n.index()))
        }
        WallPlateShape::Attached { anchor, .. } => match anchor {
            // 取付き先の節点が構面上にあれば描く（床板の取り付き版と同じ規約）。
            RegionAnchor::Line { nodes, .. } => nodes.iter().any(|n| filter.shows_node(n.index())),
            RegionAnchor::FloorRegion { nodes, .. } => {
                nodes.iter().any(|n| filter.shows_node(n.index()))
            }
            // 壁の取付き先としては使わない（`boundary_coords` も `None` を返す）。
            RegionAnchor::Point(_) => false,
        },
    }
}

fn slab_visible_on_frame(slab: &squid_n_core::model::Slab, filter: FrameFilter) -> bool {
    use squid_n_core::model::{RegionAnchor, SlabShape};
    match &slab.shape {
        SlabShape::Enclosed { boundary } => boundary.iter().all(|n| filter.shows_node(n.index())),
        SlabShape::Attached { anchor, .. } => match anchor {
            RegionAnchor::Line { nodes, .. } => nodes.iter().any(|n| filter.shows_node(n.index())),
            RegionAnchor::Point(n) => filter.shows_node(n.index()),
            // 床板では到達しない（`slab.rs::boundary_coords` と同じ理由）。
            RegionAnchor::FloorRegion { .. } => false,
        },
    }
}

/// モード形の変形前架構（破線・高透過）。変形後の紫実線の下に描き、基準位置からの変化を読む。
/// 質点モードの変形前串と同じ破線（6 pt / 隙間 4 pt）とアルファ（線 90、塗り 55）。
#[allow(clippy::too_many_arguments)]
pub(super) fn draw_mode_rest_ghost(
    painter: &egui::Painter,
    app: &App,
    model: &squid_n_core::model::Model,
    pts_rest: &[egui::Pos2],
    node_visible: &[bool],
    filter: FrameFilter,
    show_secondary: bool,
    show_sections: bool,
) {
    const DASH: f32 = 6.0;
    const GAP: f32 = 4.0;
    const LINE_A: u8 = 90;
    const FILL_A: u8 = 55;
    let line = theme::translucent(theme::HILITE_PURPLE, LINE_A);
    let stroke_w = if show_sections { 1.0_f32 } else { 1.5_f32 };
    let stroke = egui::Stroke::new(stroke_w, line);

    for (i, &p) in pts_rest.iter().enumerate() {
        if !node_visible.get(i).copied().unwrap_or(false) {
            continue;
        }
        painter.circle_filled(p, 3.0, theme::translucent(theme::DATA_BLUE, FILL_A));
    }

    for elem in &model.elements {
        if !filter.shows(elem.id) {
            continue;
        }
        if element_draw_shape(elem.kind) == DrawShape::Polygon && elem.nodes.len() >= 3 {
            let poly: Vec<egui::Pos2> = elem
                .nodes
                .iter()
                .filter_map(|n| {
                    let idx = n.index();
                    (idx < pts_rest.len()).then(|| pts_rest[idx])
                })
                .collect();
            if poly.len() == elem.nodes.len() {
                painter.add(egui::Shape::convex_polygon(
                    poly.clone(),
                    theme::translucent(theme::DATA_BLUE, FILL_A),
                    egui::Stroke::NONE,
                ));
                let mut closed = poly;
                closed.push(closed[0]);
                painter.extend(egui::Shape::dashed_line(&closed, stroke, DASH, GAP));
            }
            continue;
        }
        if !draws_as_line(elem.kind) || elem.nodes.len() < 2 {
            continue;
        }
        let n0 = elem.nodes[0].index();
        let n1 = elem.nodes[1].index();
        if n0 >= pts_rest.len() || n1 >= pts_rest.len() {
            continue;
        }
        painter.extend(egui::Shape::dashed_line(
            &[pts_rest[n0], pts_rest[n1]],
            stroke,
            DASH,
            GAP,
        ));
    }

    if show_secondary {
        let sec = egui::Stroke::new(
            if show_sections { 1.0_f32 } else { 1.5_f32 },
            theme::translucent(theme::SECONDARY_AMBER, LINE_A),
        );
        for sm in app.model.joists().chain(app.model.posts()) {
            let n0 = sm.nodes[0].index();
            let n1 = sm.nodes[1].index();
            if !filter.shows_node(n0) || !filter.shows_node(n1) {
                continue;
            }
            if n0 < pts_rest.len() && n1 < pts_rest.len() {
                painter.extend(egui::Shape::dashed_line(
                    &[pts_rest[n0], pts_rest[n1]],
                    sec,
                    DASH,
                    GAP,
                ));
            }
        }
    }
}

pub(super) fn order_wall_nodes(
    model: &squid_n_core::model::Model,
    node_ids: &[squid_n_core::ids::NodeId],
) -> Vec<squid_n_core::ids::NodeId> {
    // 各節点の座標を取得（見つからなければ並べ替えせず返す）
    let coords: Vec<[f64; 3]> = node_ids
        .iter()
        .map(|id| {
            model
                .nodes
                .iter()
                .find(|n| n.id == *id)
                .map(|n| n.coord)
                .unwrap_or([0.0; 3])
        })
        .collect();
    if coords.len() < 3 {
        return node_ids.to_vec();
    }

    // 重心
    let n = coords.len() as f64;
    let centroid = [
        coords.iter().map(|c| c[0]).sum::<f64>() / n,
        coords.iter().map(|c| c[1]).sum::<f64>() / n,
        coords.iter().map(|c| c[2]).sum::<f64>() / n,
    ];

    // 面の法線（最初の非共線な 3 点の外積）。面内基底 u, v を作る。
    let sub = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let norm = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();

    let u = {
        let d = sub(coords[1], coords[0]);
        let len = norm(d);
        if len < 1e-9 {
            [1.0, 0.0, 0.0]
        } else {
            [d[0] / len, d[1] / len, d[2] / len]
        }
    };
    // u に直交し面内に収まる v を、法線×u から作る
    let mut normal = [0.0; 3];
    for c in coords.iter().skip(2) {
        let cand = cross(sub(coords[1], coords[0]), sub(*c, coords[0]));
        if norm(cand) > 1e-9 {
            normal = cand;
            break;
        }
    }
    let v = {
        let cand = cross(normal, u);
        let len = norm(cand);
        if len < 1e-9 {
            // 退化（共線）時は並べ替えしない
            return node_ids.to_vec();
        }
        [cand[0] / len, cand[1] / len, cand[2] / len]
    };

    // 重心からの相対ベクトルを (u, v) に投影し偏角でソート
    let mut indexed: Vec<(usize, f64)> = coords
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let r = sub(*c, centroid);
            let pu = r[0] * u[0] + r[1] * u[1] + r[2] * u[2];
            let pv = r[0] * v[0] + r[1] * v[1] + r[2] * v[2];
            (i, pv.atan2(pu))
        })
        .collect();
    indexed.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    indexed.into_iter().map(|(i, _)| node_ids[i]).collect()
}

pub(super) fn draw_grid_and_axes(painter: &egui::Painter, rect: egui::Rect, projector: &Projector) {
    let center3 = projector.center3();
    let scale = projector.scale();
    let proj = |p: [f64; 3]| projector.project(p);

    /// グリッド間隔 [mm]（1 m）。
    const STEP: f64 = 1000.0;
    // ダーク半透明・線幅 0.5（淡グレー背景の上で奥行きを示す）
    let grid_stroke = egui::Stroke::new(0.5_f32, egui::Color32::from_black_alpha(36));
    let origin: [f64; 3] = [0.0; 3];

    // ビューポートに映るワールド範囲を計算。対角ピクセル長 / scale で大まかな半径を得て
    // 余裕（1.5 倍）を持たせる（回転で端が見切れないように）。
    let view_radius = (rect.width().hypot(rect.height()) / scale) as f64 * 0.75;

    // 各軸の描画範囲: center3 ± view_radius を STEP の倍数に丸める
    let range = [
        (
            ((center3[0] - view_radius) / STEP).floor() * STEP,
            ((center3[0] + view_radius) / STEP).ceil() * STEP,
        ),
        (
            ((center3[1] - view_radius) / STEP).floor() * STEP,
            ((center3[1] + view_radius) / STEP).ceil() * STEP,
        ),
        (
            ((center3[2] - view_radius) / STEP).floor() * STEP,
            ((center3[2] + view_radius) / STEP).ceil() * STEP,
        ),
    ];

    // XY 平面（z=0）の格子線を描く。a=X, b=Y 方向に原点基準で線を引く。
    let a = 0usize; // X
    let b = 1usize; // Y
    let a_lo = (range[a].0 / STEP).round() as i64;
    let a_hi = (range[a].1 / STEP).round() as i64;
    for k in a_lo..=a_hi {
        let av = k as f64 * STEP;
        let p0 = [av, range[b].0, origin[2]];
        let p1 = [av, range[b].1, origin[2]];
        painter.line_segment([proj(p0), proj(p1)], grid_stroke);
    }
    let b_lo = (range[b].0 / STEP).round() as i64;
    let b_hi = (range[b].1 / STEP).round() as i64;
    for k in b_lo..=b_hi {
        let bv = k as f64 * STEP;
        let q0 = [range[a].0, bv, origin[2]];
        let q1 = [range[a].1, bv, origin[2]];
        painter.line_segment([proj(q0), proj(q1)], grid_stroke);
    }

    // 原点からの座標軸（赤=X / 緑=Y / 青=Z）。正方向=濃色 / 負方向=淡色。
    for (axis, col, name) in [
        (0usize, theme::AXIS_X, "X"),
        (1, theme::AXIS_Y, "Y"),
        (2, theme::AXIS_Z, "Z"),
    ] {
        // 正方向: 原点 → range の上端
        let mut pe = origin;
        pe[axis] = range[axis].1;
        painter.line_segment([proj(origin), proj(pe)], egui::Stroke::new(1.5_f32, col));
        painter.text(
            proj(pe),
            egui::Align2::LEFT_BOTTOM,
            format!("{} ({:.1})", name, range[axis].1),
            egui::FontId::proportional(11.0),
            col,
        );
        // 負方向: 原点 → range の下端（淡色）
        let mut pn = origin;
        pn[axis] = range[axis].0;
        painter.line_segment(
            [proj(origin), proj(pn)],
            egui::Stroke::new(1.0_f32, theme::lighten(col, 0.45)),
        );
        painter.text(
            proj(pn),
            egui::Align2::RIGHT_TOP,
            format!("{:.1}", range[axis].0),
            egui::FontId::proportional(10.0),
            theme::lighten(col, 0.45),
        );
    }

    // 原点マーカー（黒点 + "O" ラベル）
    let op = proj(origin);
    painter.circle_filled(op, 3.0, theme::GRAY_900);
    painter.text(
        egui::pos2(op.x + 6.0, op.y - 6.0),
        egui::Align2::LEFT_BOTTOM,
        "O",
        egui::FontId::proportional(11.0),
        theme::GRAY_900,
    );
}

/// ビューポート右下にカメラの向きへ追従する座標系アイコン（XYZ 軸ガジェット）を描く。
///
/// CAD ソフトで一般的な、画面端に固定された小さな座標系。各軸をカメラの回転
/// クォータニオンで投影し、Z（手前）成分でソートして奥から描くことで
/// 手前の軸が上に重なる。軸色は 3D ビューと同一（赤=X / 緑=Y / 青=Z）。
/// 左下は支持条件凡例、右上は ViewCube が使うため右下に置く。
pub(super) fn draw_axis_gadget(painter: &egui::Painter, cam: &CameraState) {
    let rect = painter.clip_rect();
    let center = egui::pos2(rect.max.x - 45.0, rect.max.y - 45.0);
    const LEN: f32 = 28.0;

    let axes: [([f32; 3], egui::Color32, &str); 3] = [
        ([1.0, 0.0, 0.0], theme::AXIS_X, "X"),
        ([0.0, 1.0, 0.0], theme::AXIS_Y, "Y"),
        ([0.0, 0.0, 1.0], theme::AXIS_Z, "Z"),
    ];

    // 各軸をカメラ回転で投影。r[0]=右, r[1]=上（画面Yは下向きなので反転）, r[2]=手前
    let mut projected: Vec<(egui::Vec2, egui::Color32, &str, f32)> = axes
        .iter()
        .map(|(v, col, name)| {
            let r = q_rotate(cam.rot, *v);
            (egui::vec2(r[0], -r[1]), *col, *name, r[2])
        })
        .collect();
    // r[2]（手前=正）が小さい（奥）順に描く → 手前の軸が最後に描かれ上に来る
    projected.sort_by(|a, b| a.3.partial_cmp(&b.3).unwrap_or(std::cmp::Ordering::Equal));

    // 背景円（軸が背景と混ざらないよう淡い白）
    painter.circle_filled(center, LEN + 8.0, theme::translucent(theme::WHITE, 200));

    for (dir, col, name, _) in &projected {
        let end = center + *dir * LEN;
        draw_arrow(painter, center, end, *col);
        let label_pos = center + *dir * (LEN + 10.0);
        painter.text(
            label_pos,
            egui::Align2::CENTER_CENTER,
            *name,
            egui::FontId::proportional(12.0),
            *col,
        );
    }
    // 中心点
    painter.circle_filled(center, 2.0, theme::GRAY_900);
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::model::ElementKind;

    fn enclosed_plate(boundary: Vec<squid_n_core::ids::NodeId>) -> squid_n_core::model::WallPlate {
        squid_n_core::model::WallPlate {
            id: squid_n_core::ids::WallPlateId(0),
            shape: squid_n_core::model::WallPlateShape::Enclosed { boundary },
            section: None,
            opening_area: 0.0,
            opening_weight: 0.0,
            openings: Vec::new(),
            loads: vec![],
            slit: Default::default(),
        }
    }

    /// 立ち上がり高さが両端で符号反転する取り付く壁版は、塗らずに輪郭だけ描く。
    ///
    /// 境界の 4 点が自己交差する蝶ネクタイ形になり、凸多角形前提の塗りが輪郭から
    /// はみ出すためである。面積の算定が同じ形を避けているのと同じ理由による
    /// （`WallPlate::area`）。
    #[test]
    fn 符号反転する取り付く壁版は塗らない() {
        use squid_n_core::ids::NodeId;
        use squid_n_core::model::{RegionAnchor, WallPlateShape};
        let model = squid_n_core::Model::default();
        let mut plate = enclosed_plate(Vec::new());
        let mut with_extent = |extent: [f64; 2]| {
            plate.shape = WallPlateShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: [0.0, 1.0],
                    transfer: Default::default(),
                },
                extent: Some(extent),
            };
            plate_fill_is_valid(&model, &plate)
        };
        assert!(with_extent([900.0, 900.0]), "同じ向きの立ち上がりは塗る");
        assert!(with_extent([900.0, 0.0]), "片端 0 は自己交差しない");
        assert!(with_extent([-900.0, -300.0]), "下向き同士も塗る");
        assert!(!with_extent([900.0, -900.0]), "符号が反転する壁は塗らない");

        // 囲まれた壁版は境界そのものなので、この理由では塗りを止めない。
        assert!(plate_fill_is_valid(
            &model,
            &enclosed_plate(vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)])
        ));
    }

    /// 構面表示中は、境界節点がすべてその構面にある壁版だけを描く。
    ///
    /// 床板（`slab_visible_on_frame`）と同じ規約である。1 点でも構面外にあると
    /// 多角形が構面から飛び出して描かれ、構面図として読めなくなる。
    #[test]
    fn 囲まれた壁版は境界節点が全て構面上にあるときだけ描く() {
        use squid_n_core::ids::NodeId;
        let on = [true, true, true, false];
        let filter = FrameFilter {
            elem_on: None,
            node_on: Some(&on),
        };
        // 節点 3 が構面外。
        assert!(!wall_plate_visible_on_frame(
            &enclosed_plate(vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)]),
            filter
        ));
        assert!(wall_plate_visible_on_frame(
            &enclosed_plate(vec![NodeId(0), NodeId(1), NodeId(2)]),
            filter
        ));
        // 構面表示していないときはすべて描く。
        let all = FrameFilter {
            elem_on: None,
            node_on: None,
        };
        assert!(wall_plate_visible_on_frame(
            &enclosed_plate(vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)]),
            all
        ));
    }

    /// 取り付く壁版は取付き先の節点が構面上にあれば描く（床板と同じ規約）。
    /// 取付き先が点の壁版は座標を組み立てられないため描かない。
    #[test]
    fn 取り付く壁版は取付き先の節点で構面を判定する() {
        use squid_n_core::ids::NodeId;
        use squid_n_core::model::{RegionAnchor, WallPlateShape};
        let on = [true, false, false];
        let filter = FrameFilter {
            elem_on: None,
            node_on: Some(&on),
        };
        let mut plate = enclosed_plate(Vec::new());
        plate.shape = WallPlateShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(0), NodeId(1)],
                span: [0.0, 1.0],
                transfer: Default::default(),
            },
            extent: Some([900.0, 900.0]),
        };
        assert!(wall_plate_visible_on_frame(&plate, filter));

        plate.shape = WallPlateShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(1), NodeId(2)],
                span: [0.0, 1.0],
                transfer: Default::default(),
            },
            extent: Some([900.0, 900.0]),
        };
        assert!(!wall_plate_visible_on_frame(&plate, filter));

        plate.shape = WallPlateShape::Attached {
            anchor: RegionAnchor::Point(NodeId(0)),
            extent: Some([900.0, 900.0]),
        };
        assert!(!wall_plate_visible_on_frame(&plate, filter));
    }

    #[test]
    fn 仕口パネルと面要素は部材線として描かない() {
        // 仕口パネルの節点列は「接合部の節点 ＋ 取り付く部材の他端」なので、先頭
        // 2 節点を結ぶと取り付く柱・梁と同じ線分になる。全部材が直線のうちは実部材と
        // 重なって見えないが、内部たわみ表示で梁・柱を曲線にすると弦の直線だけが
        // 残り、部材が二重に描かれてしまうため線材として扱わない。
        assert!(!draws_as_line(ElementKind::PanelZone));
        assert!(!draws_as_line(ElementKind::Wall));
        assert!(!draws_as_line(ElementKind::Shell));
        // 材軸を持つ 2 節点要素は従来どおり線で描く。
        assert!(draws_as_line(ElementKind::Beam));
        assert!(draws_as_line(ElementKind::Fiber));
        assert!(draws_as_line(ElementKind::MultiSpring));
        assert!(draws_as_line(ElementKind::Brace {
            tension_only: false
        }));
        assert!(draws_as_line(ElementKind::NodalSpring));
        assert!(draws_as_line(ElementKind::Isolator));
        assert!(draws_as_line(ElementKind::Damper));
    }

    #[test]
    fn 要素の描き方は種別ごとに一意に決まる() {
        // 仕口パネルは部材線も選択ハイライトも描かない。先頭 2 節点が取り付く
        // 柱・梁と同じ節点対になるため、線を引くと選択していない柱・梁が
        // 選択されているように見えてしまう。
        assert_eq!(element_draw_shape(ElementKind::PanelZone), DrawShape::None);
        // 面要素は多角形（ハイライトはその輪郭）。
        assert_eq!(element_draw_shape(ElementKind::Wall), DrawShape::Polygon);
        assert_eq!(element_draw_shape(ElementKind::Shell), DrawShape::Polygon);
        // 材軸を持つ要素は線分。
        for kind in [
            ElementKind::Beam,
            ElementKind::Fiber,
            ElementKind::MultiSpring,
            ElementKind::Brace {
                tension_only: false,
            },
            ElementKind::NodalSpring,
            ElementKind::Isolator,
            ElementKind::Damper,
        ] {
            assert_eq!(element_draw_shape(kind), DrawShape::Line, "{kind:?}");
        }
    }
}
