//! CMQ 図（両端固定端モーメント・単純梁モーメント・せん断）。
//!
//! `viewer` ハブからの構造分割。アルゴリズム変更は行わない。

use crate::app::App;
use crate::theme;

use super::{
    diagram,
    scene::{diagram_offset_dir, in_plane_offset_dir},
    CmqComponent, DiagramPlane, FrameFilter, Projector,
};
use squid_n_core::geom::vec3::dist as member_len3;

fn is_primary_beam_for_cmq(
    model: &squid_n_core::model::Model,
    elem: &squid_n_core::model::ElementData,
) -> bool {
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

/// 一つの主架構の大梁（`ElemId`）に載る全 `MemberLoadKind` を束ねたグループ。
struct CmqElemGroup {
    /// 対象の大梁。構面表示の絞り込み（`FrameFilter`）で使う。
    elem: squid_n_core::ids::ElemId,
    n0: usize,
    n1: usize,
    ref_vec: [f64; 3],
    /// C/M/Q 評価用。グループ内の全 `MemberLoad` の荷重種別（`MemberLoadKind`）。
    loads: Vec<squid_n_core::model::MemberLoadKind>,
}

/// `app.cmq_display_member_loads()`（主架構変換後の部材荷重）を要素（大梁）単位で
/// グループ化する。大梁の中間区間（小梁がとりつく位置）の荷重も同じ `ElemId` に
/// 変換されているため、大梁1本=1グループになる。小梁・柱・スラブには `MemberLoad`
/// が付かない（または実部材化小梁として `is_primary_beam_for_cmq` で除外される）ため
/// 自然に描画対象から外れる。描画順は初出順（`app.beam_loads` に現れた順）で安定する。
fn group_member_loads_by_elem(app: &App) -> Vec<CmqElemGroup> {
    let member_loads = app.cmq_display_member_loads();
    let mut order: Vec<squid_n_core::ids::ElemId> = Vec::new();
    let mut groups: std::collections::HashMap<squid_n_core::ids::ElemId, CmqElemGroup> =
        std::collections::HashMap::new();
    for ml in member_loads {
        let Some(elem) = app.model.element(ml.elem) else {
            continue;
        };
        if !is_primary_beam_for_cmq(&app.model, elem) {
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
        group.loads.push(ml.kind);
    }
    order
        .into_iter()
        .filter_map(|id| groups.remove(&id))
        .collect()
}

/// グループ内の全荷重の両端固定端モーメントを合算する（C 図）。
fn sum_fixed_end_moments(loads: &[squid_n_core::model::MemberLoadKind], l: f64) -> (f64, f64) {
    loads
        .iter()
        .map(|ld| squid_n_load::floor::fixed_end_moments(ld, l))
        .fold((0.0, 0.0), |(ai, aj), (ci, cj)| (ai + ci, aj + cj))
}

/// グループ内の全荷重の単純梁反力を合算する（Q 図）。
fn sum_simple_reactions(loads: &[squid_n_core::model::MemberLoadKind], l: f64) -> (f64, f64) {
    loads
        .iter()
        .map(|ld| squid_n_load::floor::simple_reactions(ld, l))
        .fold((0.0, 0.0), |(ai, aj), (ri, rj)| (ai + ri, aj + rj))
}

/// M（単純梁中央モーメント）図の折れ線サンプリング位置 ξ∈[0,1] を返す。
/// 等分割に加え、`loads` に含まれる区間分布荷重の両端 a/L, b/L・集中荷重の a/L を
/// 折れ点として正確に出すため追加する。
fn cmq_m_sample_xis(loads: &[squid_n_core::model::MemberLoadKind], l: f64) -> Vec<f64> {
    use squid_n_core::model::MemberLoadKind;
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

/// 部材ローカルに沿って CMQ 図（両端固定端モーメント C・単純梁中央モーメント M・
/// せん断 Q）を描く。
///
/// N/Q/M 図と同様、張り出し方向は要素ローカル y 軸（曲げ平面内）をワールド空間で
/// とってから投影する。CMQ は鉛直床荷重による強軸曲げのため、水平梁では鉛直面内の
/// 図となり、ビューを回転しても要素座標系に固定される。
///
/// 描画ソースは `app.beam_loads`（スラブ・小梁の生の荷重分配）ではなく、主架構へ
/// 変換後の部材荷重（[`group_member_loads_by_elem`]）。これにより大梁1本=1図形になり
/// （小梁がとりつく大梁で図が分裂しない）、小梁・スラブは自然に描画対象から外れる。
pub(super) fn draw_cmq_diagram(
    painter: &egui::Painter,
    app: &App,
    coords3: &[[f64; 3]],
    proj: &Projector,
    filter: FrameFilter,
    frame_normal: Option<[f64; 3]>,
) {
    let scale = proj.scale();
    if app.beam_loads.is_empty() {
        // スラブ自体がないのか、スラブはあるが床荷重（強度）が 0 なのかを区別して案内する。
        let msg = if app.model.floor_regions.is_empty() {
            "スラブが未定義です。モデルタブの「スラブ」でスラブと床荷重を定義すると CMQ 図を表示できます"
        } else {
            "スラブの床荷重が 0 です。荷重タブ（スラブ）で固定荷重・用途（積載）を設定すると CMQ 図を表示できます"
        };
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
        return;
    }

    // 主架構へ変換後の部材荷重を要素（大梁）単位でグループ化し、座標が有効
    // （範囲内・非ゼロ長）なものだけを対象にする。
    let groups: Vec<CmqElemGroup> = group_member_loads_by_elem(app)
        .into_iter()
        .filter(|g| {
            filter.shows(g.elem)
                && g.n0 < coords3.len()
                && g.n1 < coords3.len()
                && member_len3(coords3[g.n0], coords3[g.n1]) >= 1e-9
        })
        .collect();

    let max_c = groups
        .iter()
        .map(|g| {
            let l = member_len3(coords3[g.n0], coords3[g.n1]);
            let (c_i, c_j) = sum_fixed_end_moments(&g.loads, l);
            c_i.abs().max(c_j.abs())
        })
        .fold(0.0_f64, f64::max);
    let max_q = groups
        .iter()
        .map(|g| {
            let l = member_len3(coords3[g.n0], coords3[g.n1]);
            let (q_i, q_j) = sum_simple_reactions(&g.loads, l);
            q_i.abs().max(q_j.abs())
        })
        .fold(0.0_f64, f64::max);
    // M（単純梁中央モーメント）の最大値: スパンをサンプリングして評価する。
    let max_m = groups
        .iter()
        .map(|g| {
            let l = member_len3(coords3[g.n0], coords3[g.n1]);
            cmq_m_sample_xis(&g.loads, l)
                .into_iter()
                .fold(0.0_f64, |acc, xi| {
                    acc.max(squid_n_load::floor::simple_beam_moment_at(&g.loads, l, xi * l).abs())
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
    for g in &groups {
        let p_i = coords3[g.n0];
        let p_j = coords3[g.n1];
        let l = member_len3(p_i, p_j);
        let ey = diagram_offset_dir(p_i, p_j, g.ref_vec, DiagramPlane::Ey);
        // 構面表示では張り出しを構面内へ倒す（応力図と同じ規約）。
        let ey = match frame_normal {
            Some(n) => in_plane_offset_dir(ey, p_i, p_j, n),
            None => ey,
        };
        let p0 = proj.project(p_i);
        let p1 = proj.project(p_j);

        match app.cmq_component {
            CmqComponent::C => {
                let (c_i, c_j) = sum_fixed_end_moments(&g.loads, l);
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
                let xis = cmq_m_sample_xis(&g.loads, l);
                // 先に値と対応するワールド位置を求め、ピーク px を判定してから描画する
                let mut val_max = 0.0_f64;
                let samples: Vec<(f64, [f64; 3])> = xis
                    .into_iter()
                    .map(|xi| {
                        let val = squid_n_load::floor::simple_beam_moment_at(&g.loads, l, xi * l);
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
                let (q_i, q_j) = sum_simple_reactions(&g.loads, l);
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

    // 凡例（選択中の成分のみ表示）
    let legend = match app.cmq_component {
        CmqComponent::C => format!("CMQ図 C(max={:.2}) 青", max_c),
        CmqComponent::M => format!("CMQ図 M(max={:.2}) 紫", max_m),
        CmqComponent::Q => format!("CMQ図 Q(max={:.2}) 緑", max_q),
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
