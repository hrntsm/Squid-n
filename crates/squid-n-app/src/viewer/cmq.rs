//! CMQ 図（両端固定端モーメント・単純梁モーメント・せん断）。
//!
//! 描画ソースは表示中荷重ケース（`nav.focus_load_case`。応力図と異なり解析実行を
//! 要しない）の `member`（部材荷重）そのもの。床分配・自重・取り付く壁版の線
//! アンカー・手入力荷重のいずれも同じ図に重ねて描く（CMQ図を荷重ケースの全荷重へ
//! 揃える改修。`dev_docs/handoff/CMQ図を荷重ケースの全荷重へ_申し送り.md`）。

use std::collections::HashMap;

use crate::app::App;
use crate::theme;

use super::{
    diagram,
    scene::{diagram_offset_dir, in_plane_offset_dir},
    CmqComponent, DiagramPlane, FrameFilter, Projector,
};
use squid_n_core::geom::vec3::dist as member_len3;
use squid_n_core::ids::ElemId;
use squid_n_core::model::{LoadCase, MemberLoadKind, Model};

fn is_primary_beam_for_cmq(model: &Model, elem: &squid_n_core::model::ElementData) -> bool {
    if elem.kind != squid_n_core::model::ElementKind::Beam || elem.nodes.len() != 2 {
        return false;
    }
    let (n0, n1) = (elem.nodes[0], elem.nodes[1]);
    let is_materialized_joist = model.floor_regions.iter().any(|slab| {
        slab.joist_lines().iter().any(|j| {
            (j.support[0] == n0 && j.support[1] == n1) || (j.support[0] == n1 && j.support[1] == n0)
        })
    });
    !is_materialized_joist
}

/// 一つの主架構の大梁（`ElemId`）に載る、表示中荷重ケースの全 `MemberLoad` を束ねた
/// グループ。荷重は（種別, 世界座標の作用方向）の対で持ち、局所 ey/ez 面への投影は
/// 描画時に行う（投影に要素の現在の節点座標が要るため）。
struct CmqElemGroup {
    /// 対象の大梁。構面表示の絞り込み（`FrameFilter`）で使う。
    elem: ElemId,
    n0: usize,
    n1: usize,
    ref_vec: [f64; 3],
    loads: Vec<(MemberLoadKind, [f64; 3])>,
}

/// 表示中荷重ケースの `member` を要素（大梁）単位でグループ化する。大梁の中間区間
/// （小梁がとりつく位置）の荷重も同じ `ElemId` に変換済みのため、大梁1本=1グループに
/// なる。小梁・柱・スラブには `MemberLoad` が付かない（または実部材化小梁として
/// `is_primary_beam_for_cmq` で除外される）ため自然に描画対象から外れる。柱への
/// 節点集中荷重（`NodalLoad`）はそもそも `member` に含まれないため、同じ理由で
/// 梁の図に出ない（梁に載らない荷重であり、これは正しい）。
fn group_member_loads_by_elem(model: &Model, case: &LoadCase) -> Vec<CmqElemGroup> {
    let mut order: Vec<ElemId> = Vec::new();
    let mut groups: HashMap<ElemId, CmqElemGroup> = HashMap::new();
    for ml in &case.member {
        let Some(elem) = model.element(ml.elem) else {
            continue;
        };
        if !is_primary_beam_for_cmq(model, elem) {
            continue;
        }
        let group = groups.entry(ml.elem).or_insert_with(|| {
            order.push(ml.elem);
            CmqElemGroup {
                elem: ml.elem,
                n0: elem.nodes[0].index(),
                n1: elem.nodes[1].index(),
                ref_vec: elem.local_axis.ref_vector,
                loads: Vec::new(),
            }
        });
        group.loads.push((ml.kind.clone(), ml.dir));
    }
    order
        .into_iter()
        .filter_map(|id| groups.remove(&id))
        .collect()
}

/// `MemberLoadKind` の大きさ（強度・集中荷重）を、世界座標の作用方向 `dir` から
/// 局所軸の単位ベクトル `axis`（ey または ez）へ投影する。位置（`a`/`b`）は
/// 部材長に沿った値のためそのまま、強度・大きさだけを `dot(dir, axis)` 倍する。
fn project_load(kind: MemberLoadKind, dir: [f64; 3], axis: [f64; 3]) -> MemberLoadKind {
    let s = dir[0] * axis[0] + dir[1] * axis[1] + dir[2] * axis[2];
    match kind {
        MemberLoadKind::Point { a, p } => MemberLoadKind::Point { a, p: p * s },
        MemberLoadKind::Distributed { a, b, w1, w2 } => MemberLoadKind::Distributed {
            a,
            b,
            w1: w1 * s,
            w2: w2 * s,
        },
    }
}

/// グループ内の全荷重の両端固定端モーメントを合算する（C 図）。
fn sum_fixed_end_moments(loads: &[MemberLoadKind], l: f64) -> (f64, f64) {
    loads
        .iter()
        .map(|ld| squid_n_load::floor::fixed_end_moments(ld, l))
        .fold((0.0, 0.0), |(ai, aj), (ci, cj)| (ai + ci, aj + cj))
}

/// グループ内の全荷重の単純梁反力を合算する（Q 図）。
fn sum_simple_reactions(loads: &[MemberLoadKind], l: f64) -> (f64, f64) {
    loads
        .iter()
        .map(|ld| squid_n_load::floor::simple_reactions(ld, l))
        .fold((0.0, 0.0), |(ai, aj), (ri, rj)| (ai + ri, aj + rj))
}

/// M（単純梁中央モーメント）図の折れ線サンプリング位置 ξ∈[0,1] を返す。
/// 等分割に加え、`loads` に含まれる区間分布荷重の両端 a/L, b/L・集中荷重の a/L を
/// 折れ点として正確に出すため追加する。
fn cmq_m_sample_xis(loads: &[MemberLoadKind], l: f64) -> Vec<f64> {
    const N: usize = 32;
    let mut xis: Vec<f64> = (0..=N).map(|k| k as f64 / N as f64).collect();
    if l > 1e-9 {
        for load in loads {
            match *load {
                MemberLoadKind::Point { a, .. } => xis.push((a / l).clamp(0.0, 1.0)),
                MemberLoadKind::Distributed { a, b, .. } => {
                    xis.push((a / l).clamp(0.0, 1.0));
                    xis.push((b / l).clamp(0.0, 1.0));
                }
            }
        }
    }
    xis.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    xis.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    xis
}

/// ポリゴンを塗り（`convex_polygon`, `Stroke::NONE`）と輪郭（閉じない折れ線
/// `Shape::line`）に分けて描画する。塗り+輪郭を1シェイプにする従来方式（閉路）だと、
/// p0/p1 で軸線と曲線が浅い角度で接する折り返し点の epaint マイター結合が発散し、
/// 部材軸方向に画面外まで伸びるスパイク描画になるため、輪郭は閉じない折れ線にする。
pub(super) fn paint_diagram_polygon(
    painter: &egui::Painter,
    points: Vec<egui::Pos2>,
    fill: egui::Color32,
    stroke_color: egui::Color32,
) {
    painter.add(egui::Shape::convex_polygon(
        points.clone(),
        fill,
        egui::Stroke::NONE,
    ));
    painter.add(egui::Shape::line(
        points,
        egui::Stroke::new(1.5_f32, stroke_color),
    ));
}

/// 1 大梁・1 軸（ey または ez）分の描画対象。荷重は `axis`（局所軸の単位ベクトル。
/// 世界座標）へ投影済みの `MemberLoadKind` 列で持つ。
struct CmqTrace<'g> {
    group: &'g CmqElemGroup,
    l: f64,
    /// 描画位置の張り出し方向（構面表示では `in_plane_offset_dir` で構面内へ倒し済み）。
    offset_dir: [f64; 3],
    loads: Vec<MemberLoadKind>,
}

/// 部材ローカルに沿って CMQ 図（両端固定端モーメント C・単純梁中央モーメント M・
/// せん断 Q）を描く。
///
/// 応力図と同じ「面」の区別（強軸 ey・弱軸 ez。`app.cmq_axes`）を持つ。部材の局所軸が
/// 傾いている（斜め柱・ひねりのある断面等）場合、鉛直荷重でも ey・ez 両方に成分が
/// 生じうるため、実際に `MemberLoad.dir` を局所軸へ投影して評価する
/// （[`project_load`]）。両軸を同時表示するときは同一スケールで重ねて描く。
pub(super) fn draw_cmq_diagram(
    painter: &egui::Painter,
    app: &App,
    coords3: &[[f64; 3]],
    proj: &Projector,
    filter: FrameFilter,
    frame_normal: Option<[f64; 3]>,
) {
    let scale = proj.scale();

    fn info_text(painter: &egui::Painter, msg: &str) {
        painter.text(
            egui::pos2(
                painter.clip_rect().min.x + 10.0,
                painter.clip_rect().min.y + 30.0,
            ),
            egui::Align2::LEFT_TOP,
            msg,
            egui::FontId::proportional(13.0),
            theme::GRAY_600,
        );
    }

    let Some(case) = app.cmq_display_load_case() else {
        info_text(
            painter,
            "荷重ケースがありません。荷重タブで作成してください",
        );
        return;
    };

    let axes: Vec<DiagramPlane> = [
        (app.cmq_axes.ey, DiagramPlane::Ey),
        (app.cmq_axes.ez, DiagramPlane::Ez),
    ]
    .into_iter()
    .filter_map(|(on, plane)| on.then_some(plane))
    .collect();
    if axes.is_empty() {
        info_text(painter, "「軸:」で強軸・弱軸のどちらかを選んでください");
        return;
    }

    // 表示中ケースの部材荷重を要素（大梁）単位でグループ化し、座標が有効
    // （範囲内・非ゼロ長）なものだけを対象にする。
    let groups: Vec<CmqElemGroup> = group_member_loads_by_elem(&app.model, case)
        .into_iter()
        .filter(|g| {
            filter.shows(g.elem)
                && g.n0 < coords3.len()
                && g.n1 < coords3.len()
                && member_len3(coords3[g.n0], coords3[g.n1]) >= 1e-9
        })
        .collect();

    if groups.is_empty() {
        info_text(
            painter,
            &format!(
                "荷重ケース「{}」に大梁への部材荷重がありません。\
                 荷重タブで荷重を追加するか、ナビゲータで他のケースを選択してください",
                case.name
            ),
        );
        return;
    }

    // 各大梁・各表示軸ごとに、荷重を局所軸へ投影したトレースを作る。
    let traces: Vec<CmqTrace> = groups
        .iter()
        .flat_map(|g| {
            let p_i = coords3[g.n0];
            let p_j = coords3[g.n1];
            let l = member_len3(p_i, p_j);
            axes.iter().map(move |&plane| {
                // 局所 ey/ez の単位ベクトル（世界座標）。荷重の投影軸として使う。
                let axis_dir = diagram_offset_dir(p_i, p_j, g.ref_vec, plane);
                // 構面表示では張り出しを構面内へ倒す（応力図と同じ規約）。投影に使う
                // `axis_dir` 自体はここでは倒さない（物理的な軸を変えないため）。
                let offset_dir = match frame_normal {
                    Some(n) => in_plane_offset_dir(axis_dir, p_i, p_j, n),
                    None => axis_dir,
                };
                let loads = g
                    .loads
                    .iter()
                    .map(|(kind, dir)| project_load(kind.clone(), *dir, axis_dir))
                    .collect();
                CmqTrace {
                    group: g,
                    l,
                    offset_dir,
                    loads,
                }
            })
        })
        .collect();

    // 選択中の軸をまとめて 1 つのスケールで正規化する（応力図の同単位成分の
    // 共有スケールと同じ規約）。
    let max_c = traces
        .iter()
        .map(|t| {
            let (c_i, c_j) = sum_fixed_end_moments(&t.loads, t.l);
            c_i.abs().max(c_j.abs())
        })
        .fold(0.0_f64, f64::max);
    let max_q = traces
        .iter()
        .map(|t| {
            let (q_i, q_j) = sum_simple_reactions(&t.loads, t.l);
            q_i.abs().max(q_j.abs())
        })
        .fold(0.0_f64, f64::max);
    // M（単純梁中央モーメント）の最大値: スパンをサンプリングして評価する。
    let max_m = traces
        .iter()
        .map(|t| {
            cmq_m_sample_xis(&t.loads, t.l)
                .into_iter()
                .fold(0.0_f64, |acc, xi| {
                    acc.max(
                        squid_n_load::floor::simple_beam_moment_at(&t.loads, t.l, xi * t.l).abs(),
                    )
                })
        })
        .fold(0.0_f64, f64::max);
    if max_c < 1e-12 && max_q < 1e-12 && max_m < 1e-12 {
        return;
    }
    // 最大値で 60px 相当のワールド長（一様スケール正射影なので px/scale=ワールド長）
    let c_amp = 60.0 / max_c.max(1e-12) / scale as f64;
    let q_amp = 60.0 / max_q.max(1e-12) / scale as f64;
    let m_amp = 60.0 / max_m.max(1e-12) / scale as f64;

    // 張り出しピーク px が閾値未満の潰れた図形はスキップ（マイター発散対策。
    // N/Q/M 図と共有する `diagram::MIN_DIAGRAM_PX`）。
    for t in &traces {
        let p_i = coords3[t.group.n0];
        let p_j = coords3[t.group.n1];
        let p0 = proj.project(p_i);
        let p1 = proj.project(p_j);
        let ey = t.offset_dir;

        match app.cmq_component {
            CmqComponent::C => {
                let (c_i, c_j) = sum_fixed_end_moments(&t.loads, t.l);
                // 張り出しピーク px が閾値未満の潰れたポリゴンはスキップ（上記コメント参照）
                let peak_px = (60.0 * c_i.abs().max(c_j.abs()) / max_c.max(1e-12)) as f32;
                if peak_px < diagram::MIN_DIAGRAM_PX {
                    continue;
                }
                // C 図（モーメント）: 両端の合算 c_i, c_j を結ぶ折れ線ポリゴン。M図の規約
                // （引張側に描く。sagging 正=-ey側=下、hogging 負=+ey側=上）に合わせ、
                // 固定端モーメント（hogging=引張は上端）は +ey 側=梁上側に描く。
                // c_i/c_j は固定端モーメントの符号規約上、両端で逆符号（i端+, j端-）で
                // 保持されているため、j 端は符号反転して i 端と同じ側（+ey 側）に描く。
                let c_poly = vec![
                    p0,
                    proj.project_offset(p_i, ey, c_i * c_amp),
                    proj.project_offset(p_j, ey, -c_j * c_amp),
                    p1,
                ];
                // C 図（モーメント）= 通常データ（青）
                paint_diagram_polygon(
                    painter,
                    c_poly,
                    theme::translucent(theme::DATA_BLUE, 60),
                    theme::DATA_BLUE,
                );
            }
            CmqComponent::M => {
                // M 図（単純梁としての中央モーメント）: スパンを分割サンプリングし、
                // グループ内の全荷重の simple_beam_moment_at を合算した値を、N/Q/M 図と
                // 同じ規約（正の sagging モーメントが梁下側=-ey 側）でプロットする。
                // 区間分布荷重の境界・集中荷重は折れ点 ξ=a/L, b/L を含める。
                let xis = cmq_m_sample_xis(&t.loads, t.l);
                // 先に値と対応するワールド位置を求め、ピーク px を判定してから描画する
                let mut val_max = 0.0_f64;
                let samples: Vec<(f64, [f64; 3])> = xis
                    .into_iter()
                    .map(|xi| {
                        let val =
                            squid_n_load::floor::simple_beam_moment_at(&t.loads, t.l, xi * t.l);
                        val_max = val_max.max(val.abs());
                        let base3 = [
                            p_i[0] + (p_j[0] - p_i[0]) * xi,
                            p_i[1] + (p_j[1] - p_i[1]) * xi,
                            p_i[2] + (p_j[2] - p_i[2]) * xi,
                        ];
                        (val, base3)
                    })
                    .collect();
                // 張り出しピーク px が閾値未満の潰れたポリゴンはスキップ（上記コメント参照）
                let peak_px = (60.0 * val_max / max_m.max(1e-12)) as f32;
                if peak_px < diagram::MIN_DIAGRAM_PX {
                    continue;
                }
                let mut m_poly = Vec::with_capacity(samples.len() + 2);
                m_poly.push(p0);
                // 直前の点とスクリーン距離が近すぎるサンプル点は重複点として除外する
                // （ゼロ長セグメントも epaint のマイター結合発散の原因になるため。
                // N/Q/M 図と共有する `diagram::MIN_SEGMENT_PX`）。p0/p1 は常に残す。
                let mut last = p0;
                for (val, base3) in samples {
                    let pt = proj.project_offset(base3, ey, -val * m_amp);
                    if (pt.x - last.x).hypot(pt.y - last.y) < diagram::MIN_SEGMENT_PX {
                        continue;
                    }
                    last = pt;
                    m_poly.push(pt);
                }
                m_poly.push(p1);
                // M 図（中央モーメント）= 強調紫。C（青）・Q（緑）と弁別する
                paint_diagram_polygon(
                    painter,
                    m_poly,
                    theme::translucent(theme::HILITE_PURPLE, 60),
                    theme::HILITE_PURPLE,
                );
            }
            CmqComponent::Q => {
                let (q_i, q_j) = sum_simple_reactions(&t.loads, t.l);
                // 張り出しピーク px が閾値未満の潰れたポリゴンはスキップ（上記コメント参照）
                let peak_px = (60.0 * q_i.abs().max(q_j.abs()) / max_q.max(1e-12)) as f32;
                if peak_px < diagram::MIN_DIAGRAM_PX {
                    continue;
                }
                // Q 図（せん断）: 両端の合算 q_i, q_j を結ぶ折れ線ポリゴン（+ey 側に描画）
                let q_poly = vec![
                    p0,
                    proj.project_offset(p_i, ey, q_i * q_amp),
                    proj.project_offset(p_j, ey, q_j * q_amp),
                    p1,
                ];
                // Q 図（せん断）= 良好系（緑）。C（青）と弁別する
                paint_diagram_polygon(
                    painter,
                    q_poly,
                    theme::translucent(theme::GOOD_GREEN, 60),
                    theme::GOOD_GREEN,
                );
            }
        }
    }

    // 凡例（選択中の成分・軸のみ表示）
    let axis_label = match (app.cmq_axes.ey, app.cmq_axes.ez) {
        (true, true) => "強軸+弱軸",
        (true, false) => "強軸",
        (false, true) => "弱軸",
        (false, false) => unreachable!("axes.is_empty() で早期 return 済み"),
    };
    let legend = match app.cmq_component {
        CmqComponent::C => format!("CMQ図 C(max={:.2}, {}) 青", max_c, axis_label),
        CmqComponent::M => format!("CMQ図 M(max={:.2}, {}) 紫", max_m, axis_label),
        CmqComponent::Q => format!("CMQ図 Q(max={:.2}, {}) 緑", max_q, axis_label),
    };
    painter.text(
        egui::pos2(
            painter.clip_rect().min.x + 10.0,
            painter.clip_rect().min.y + 10.0,
        ),
        egui::Align2::LEFT_TOP,
        legend,
        egui::FontId::proportional(14.0),
        theme::GRAY_700,
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::NodeId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCaseKind, LocalAxis, MemberLoad,
        Node,
    };

    fn mk_node(id: u32, x: f64, y: f64, z: f64) -> Node {
        Node {
            id: NodeId(id),
            coord: [x, y, z],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        }
    }

    fn mk_beam(id: u32, i: u32, j: u32, ref_vector: [f64; 3]) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: None,
            local_axis: LocalAxis { ref_vector },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    fn mk_wall(id: u32, i: u32, j: u32) -> ElementData {
        ElementData {
            kind: ElementKind::Wall,
            ..mk_beam(id, i, j, [1.0, 0.0, 0.0])
        }
    }

    // `ElementKind` は柱・梁を区別しない（幾何で決まる。`squid_n_core::frame::is_column`
    // 参照）ため、`is_primary_beam_for_cmq` が実際に除外するのは非 `Beam` 種別の要素
    // （壁・仕口パネル等）と実部材化小梁だけ。柱への集中荷重が CMQ 図に出ない理由は
    // 別にある: `LoadTransfer::Columns`（雑壁・取り付く壁版の柱伝達）は `NodalLoad`
    // を生成し、`LoadCase.member` に一切入らないため（`squid_n_load::wall_attached`）。
    //
    /// 表示中ケースの `member` は、由来を問わず（床分配・自重・取り付く壁版・
    /// 手入力のいずれでも）大梁ごとに束ねられる。非 `Beam` 種別（壁等）への
    /// 部材荷重は対象外（`is_primary_beam_for_cmq`）。
    #[test]
    fn group_member_loads_by_elem_merges_any_source_and_excludes_non_beam_kind() {
        let model = Model {
            nodes: vec![mk_node(0, 0.0, 0.0, 0.0), mk_node(1, 4000.0, 0.0, 0.0)],
            elements: vec![mk_beam(0, 0, 1, [0.0, 0.0, 1.0]), mk_wall(1, 0, 1)],
            ..Default::default()
        };
        let case = LoadCase {
            id: squid_n_core::ids::LoadCaseId(0),
            name: "DL".into(),
            kind: LoadCaseKind::Dead,
            nodal: Vec::new(),
            member: vec![
                // 床分配由来（自動）
                MemberLoad::auto(
                    ElemId(0),
                    [0.0, 0.0, -1.0],
                    MemberLoadKind::Distributed {
                        a: 0.0,
                        b: 4000.0,
                        w1: 1.0,
                        w2: 1.0,
                    },
                ),
                // 手入力（取り付く壁版・梁自重相当）
                MemberLoad::manual(
                    ElemId(0),
                    [0.0, 0.0, -1.0],
                    MemberLoadKind::Point {
                        a: 2000.0,
                        p: 100.0,
                    },
                ),
                // 壁（非 Beam 種別）への部材荷重は CMQ 対象外
                MemberLoad::auto(
                    ElemId(1),
                    [0.0, 0.0, -1.0],
                    MemberLoadKind::Distributed {
                        a: 0.0,
                        b: 3000.0,
                        w1: 1.0,
                        w2: 1.0,
                    },
                ),
            ],
        };

        let groups = group_member_loads_by_elem(&model, &case);
        assert_eq!(groups.len(), 1, "壁(ElemId(1))はグループに現れないはず");
        assert_eq!(groups[0].elem, ElemId(0));
        assert_eq!(
            groups[0].loads.len(),
            2,
            "床分配・手入力の両方が同じ大梁のグループへ束ねられるはず"
        );
    }

    /// 局所軸へ投影すると、ey（強軸）成分とez（弱軸）成分の二乗和が
    /// 元の大きさの二乗と一致する（ey・ez は直交単位ベクトルのため）。
    #[test]
    fn project_load_splits_into_orthogonal_components() {
        let dir = [0.6, 0.0, -0.8]; // 正規化済み（斜め方向）
        let ey = [0.0, 0.0, -1.0];
        let ez = [1.0, 0.0, 0.0];
        let kind = MemberLoadKind::Point { a: 100.0, p: 10.0 };

        let proj_ey = project_load(kind.clone(), dir, ey);
        let proj_ez = project_load(kind, dir, ez);
        let (MemberLoadKind::Point { p: p_ey, .. }, MemberLoadKind::Point { p: p_ez, .. }) =
            (proj_ey, proj_ez)
        else {
            unreachable!()
        };
        assert!(
            (p_ey * p_ey + p_ez * p_ez - 10.0 * 10.0).abs() < 1e-9,
            "ey成分・ez成分の二乗和は元の大きさの二乗と一致するはず: p_ey={p_ey} p_ez={p_ez}"
        );
        // dir はほぼ ey 向き（0.8）で ez 成分（0.6）より大きい
        assert!(p_ey.abs() > p_ez.abs());
    }

    /// 荷重方向が局所軸と直交するとき、その面への投影はゼロになる。
    #[test]
    fn project_load_zero_when_orthogonal() {
        let dir = [0.0, 1.0, 0.0];
        let ey = [0.0, 0.0, -1.0];
        let kind = MemberLoadKind::Distributed {
            a: 0.0,
            b: 1.0,
            w1: 5.0,
            w2: 5.0,
        };
        let MemberLoadKind::Distributed { w1, w2, .. } = project_load(kind, dir, ey) else {
            unreachable!()
        };
        assert!(w1.abs() < 1e-12 && w2.abs() < 1e-12);
    }
}
