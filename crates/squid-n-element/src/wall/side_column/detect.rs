//! 側柱判定（自部材が耐震壁の側柱かどうかの幾何判定）。
//!
//! 判定結果として解放すべき局所曲げ面（[`ReleaseAxis`]）を返す。

use super::ReleaseAxis;
use crate::transform::LocalFrame;
use squid_n_core::geom::vec3::{cross, dot, sub, unit};
use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, ElementKind, Model};

/// 曲げを伝達する線材（柱・梁として扱う要素種別）か。
///
/// 大梁として壁の辺を構成しうるのは線材のみで、ブレース（軸材）・面要素・バネは
/// 曲げを伝達しないため除く。ファイバー梁・マルチスプリング梁は増分解析で用いる
/// 正規の線材のため、解析種別で辺の有無が変わらないよう対象に含める。
///
/// 大梁の判定（[`crate::misc_wall::wall_is_framed`]）と周辺架構の構造種別の判定
/// （[`crate::misc_wall::wall_frame_category_issue`]）で対象種別が食い違わないよう、
/// 判定を本関数へ集約する。
pub fn is_line_member(kind: ElementKind) -> bool {
    matches!(
        kind,
        ElementKind::Beam | ElementKind::Fiber | ElementKind::MultiSpring
    )
}

/// 耐震壁の側柱として扱う要素種別か。
///
/// 線材（[`is_line_member`]）のうち `Beam` に限る。側柱は面内両端ピンとして
/// [`super::InPlaneReleasedColumn`] へ差し替えるが、この要素は `BeamElement` を
/// 内包する実装のため、要素生成（`crate::factory`）がピン化できるのは `Beam` だけ
/// である。
///
/// ピン化できない種別を側柱として数えると、その断面を壁のせん断断面へ算入しながら
/// 柱自身も面内せん断を負担するため、**面内せん断の二重計上**になる。側柱を数える
/// 箇所（ピン化・せん断断面への算入・Qu・入力診断）は、要素生成が実際にピン化できる
/// 範囲と一致させる必要があるため、判定を本関数へ集約する。
///
/// `Fiber`・`MultiSpring` の側柱に対応するには
/// [`super::InPlaneReleasedColumn`] を任意の要素挙動に対して静的縮約できるよう
/// 一般化する必要がある（`dev_docs/handoff/申し送り.md`）。
pub fn is_side_column_member(kind: ElementKind) -> bool {
    matches!(kind, ElementKind::Beam)
}

/// 自部材（`data`）が耐震壁（壁エレメントモデル）の側柱（面内両端ピンの柱）かどうかを
/// 判定し、そうであれば解放すべき局所曲げ面を返す。
///
/// 条件:
/// 1. 自部材が側柱として扱う種別（[`is_side_column_member`]）かつ鉛直材であること
///    （dz が dx・dy に対して支配的）。
/// 2. `model.elements` 中に節点数4以上の `ElementKind::Wall` があり、それが耐震壁として
///    成立すること（[`crate::misc_wall::wall_is_seismic`]）。
/// 3. その壁の四隅を z で下辺2・上辺2 に分け、下辺の軸方向への射影で上辺と対応付けた
///    （`wall_panel.rs::try_new` と同じロジック）とき、自部材の両端節点が
///    「下辺a-上辺a」または「下辺b-上辺b」のいずれかの鉛直辺の2節点と一致すること。
///
/// 側柱を面内両端ピンとするのは、面内せん断を壁エレメントが全部負担するモデルにおいて
/// 側柱にも面内曲げ・せん断を持たせると**二重計上**になるという置換モデルの内部整合の
/// 要請である。壁材料が RC か鋼かとは無関係のため、材種による条件は課さない
/// （RC 規準に由来する条件は [`crate::misc_wall::wall_is_seismic`] が持つ）。
///
/// 解放曲げ面は、壁面法線（下辺方向×鉛直の外積）と柱の局所 ey・ez の内積絶対値が
/// 大きい方（＝回転軸が壁法線に平行な方）とする。
pub fn wall_side_column_release(data: &ElementData, model: &Model) -> Option<ReleaseAxis> {
    let (n0, n1, p0, p1) = side_column_candidate(data, model)?;

    for wall in &model.elements {
        if !matches!(wall.kind, ElementKind::Wall) || wall.nodes.len() < 4 {
            continue;
        }
        // 事前フィルタ: 自部材の両端節点が壁の 4 隅に含まれない壁は側柱の辺に
        // なり得ないため、耐震壁成立判定（`wall_is_seismic` は上下大梁の探索で
        // 全要素を走査する）や幾何算定へ進まない。これを欠くと側柱判定が
        // Beam 1 本あたり O(壁数 × 要素数) になり、全部材を組む要素生成ループで
        // 計算量が爆発する。
        let contains = |nid: NodeId| wall.nodes.iter().take(4).any(|&x| x == nid);
        if !(contains(n0) && contains(n1)) {
            continue;
        }
        // 耐震壁が不成立（フレーム内雑壁）の場合、柱は側柱としてピン化せず、
        // 通常の柱として袖壁付きの断面性能算入（`beam.rs`）を受ける
        // （RC規準の耐震壁規定。フレーム内雑壁の扱い）。
        if !crate::misc_wall::wall_is_seismic(wall, model) {
            continue;
        }
        let Some((sides, normal)) = wall_side_edges(wall, model) else {
            continue;
        };

        // 自部材の両端節点が同一鉛直辺（下辺a-上辺a、または下辺b-上辺b）と一致するか
        let matches_side = |side: (NodeId, NodeId)| -> bool {
            (side.0 == n0 && side.1 == n1) || (side.0 == n1 && side.1 == n0)
        };
        if !(matches_side(sides[0]) || matches_side(sides[1])) {
            continue;
        }

        return Some(release_axis_for_normal(data, p0, p1, normal));
    }
    None
}

/// 側柱候補の共通前提チェック（種別・2 節点・鉛直材）。満たせば両端の
/// 節点 ID と座標を返す。[`wall_side_column_release`] と
/// [`SideColumnEdges::release_axis`] で共有する。
fn side_column_candidate(
    data: &ElementData,
    model: &Model,
) -> Option<(NodeId, NodeId, [f64; 3], [f64; 3])> {
    if !is_side_column_member(data.kind) || data.nodes.len() < 2 {
        return None;
    }
    let n0 = data.nodes[0];
    let n1 = data.nodes[1];
    let node0 = model.nodes.get(n0.index())?;
    let node1 = model.nodes.get(n1.index())?;
    let (p0, p1) = (node0.coord, node1.coord);
    // 鉛直材の判定（全クレート共通の 45° 余弦基準）
    if !squid_n_core::geom::is_vertical_axis(p0, p1) {
        return None;
    }
    Some((n0, n1, p0, p1))
}

/// 壁の鉛直辺 2 本（下辺a-上辺a・下辺b-上辺b の節点対）と壁面法線。
type SideEdges = ([(NodeId, NodeId); 2], [f64; 3]);

/// 壁 1 枚の鉛直辺 2 本（下辺a-上辺a・下辺b-上辺b の節点対）と壁面法線を返す。
/// 四隅を z で下辺 2・上辺 2 に分け、下辺の軸方向への射影で上辺と対応付ける
/// （`wall_panel.rs::try_new` と同じロジック）。退化した壁（節点欠落・辺長ゼロ・
/// 法線が定まらない）は `None`。
fn wall_side_edges(wall: &ElementData, model: &Model) -> Option<SideEdges> {
    let ids: Vec<NodeId> = wall.nodes.iter().take(4).copied().collect();
    let coords = ids
        .iter()
        .map(|nid| model.nodes.get(nid.index()).map(|n| n.coord))
        .collect::<Option<Vec<_>>>()?;

    // z で下辺2節点・上辺2節点に分ける（wall_panel.rs::try_new と同じロジック）
    let mut order: Vec<usize> = (0..4).collect();
    order.sort_by(|&a, &b| coords[a][2].total_cmp(&coords[b][2]));
    let (b0, b1, t0, t1) = (order[0], order[1], order[2], order[3]);

    let (pa, pb) = (coords[b0], coords[b1]);
    let ex_bot = unit(sub(pb, pa))?;
    // 上辺は下辺の a に近い方を a とする（対応付け）
    let (ta, tb) = {
        let d0 = dot(sub(coords[t0], pa), ex_bot).abs();
        let d1 = dot(sub(coords[t1], pa), ex_bot).abs();
        if d0 <= d1 {
            (t0, t1)
        } else {
            (t1, t0)
        }
    };

    // 壁面法線 = 下辺方向 × 鉛直
    let up = [0.0, 0.0, 1.0];
    let normal = unit(cross(ex_bot, up))?;

    Some(([(ids[b0], ids[ta]), (ids[b1], ids[tb])], normal))
}

/// 壁面法線から解放すべき局所曲げ面を定める（回転軸が壁法線に平行な方）。
fn release_axis_for_normal(
    data: &ElementData,
    p0: [f64; 3],
    p1: [f64; 3],
    normal: [f64; 3],
) -> ReleaseAxis {
    let axis = LocalFrame::from_nodes(p0, p1, data.local_axis.ref_vector);
    let dot_ey = dot(axis.rot[1], normal).abs();
    let dot_ez = dot(axis.rot[2], normal).abs();
    if dot_ey >= dot_ez {
        ReleaseAxis::LocalY
    } else {
        ReleaseAxis::LocalZ
    }
}

/// 耐震壁の鉛直辺（節点対）→ 壁面法線 の事前インデックス。
///
/// [`wall_side_column_release`] は 1 部材の判定のたびに `model.elements` を
/// 走査するため、全部材を分類する描画・一覧ループでは合計 O(部材数²) になる。
/// そのようなループでは本インデックスをループ前に 1 回構築
/// （[`SideColumnEdges::build`] は O(壁数 × 要素数)）して
/// [`SideColumnEdges::release_axis`] を引くことで、1 部材あたり定数時間で
/// 同じ判定結果を得られる。
pub struct SideColumnEdges {
    /// 節点対（NodeId 昇順に正規化）→ 壁面法線。
    edges: std::collections::HashMap<(NodeId, NodeId), [f64; 3]>,
}

impl SideColumnEdges {
    fn key(a: NodeId, b: NodeId) -> (NodeId, NodeId) {
        if a.0 <= b.0 {
            (a, b)
        } else {
            (b, a)
        }
    }

    /// 耐震壁として成立する全壁の鉛直辺を収集する。
    pub fn build(model: &Model) -> Self {
        let mut edges = std::collections::HashMap::new();
        for wall in &model.elements {
            if !matches!(wall.kind, ElementKind::Wall) || wall.nodes.len() < 4 {
                continue;
            }
            if !crate::misc_wall::wall_is_seismic(wall, model) {
                continue;
            }
            let Some((sides, normal)) = wall_side_edges(wall, model) else {
                continue;
            };
            for (a, b) in sides {
                // 複数の壁が同じ辺を共有する場合は要素順で先の壁を採る
                // （`wall_side_column_release` の走査順と同じ規則）。
                edges.entry(Self::key(a, b)).or_insert(normal);
            }
        }
        Self { edges }
    }

    /// [`wall_side_column_release`] と同じ判定（構築済みインデックス版）。
    pub fn release_axis(&self, data: &ElementData, model: &Model) -> Option<ReleaseAxis> {
        let (n0, n1, p0, p1) = side_column_candidate(data, model)?;
        let normal = *self.edges.get(&Self::key(n0, n1))?;
        Some(release_axis_for_normal(data, p0, p1, normal))
    }
}
