//! 構面（通り・階）の切り出し。
//!
//! 通り芯 1 本、または階 1 つに属する節点・要素を選び出し、それを正対で見るための
//! 面の法線を返す。2D の軸組図・伏図を描くための下ごしらえであり、構造計算には
//! 用いない（描画対象の絞り込みのみ）。
//!
//! - [`FrameTarget`] — 見る対象（通り／階）。
//! - [`Frame`] — 切り出した構面（所属節点・所属要素・法線・表示名）。
//! - [`build_frame`] — モデルから構面を切り出す。

use crate::geom::{best_fit_plane_normal, is_vertical_pair};
use crate::ids::StoryId;
use crate::model::{ElementKind, Model};

/// 通り上とみなす、通りからの離れの許容差 [mm]。
/// 通り芯の生成（[`crate::axis_gen::AXIS_TOL_MM`]）と同じ値を使う。
pub const FRAME_TOL_MM: f64 = crate::axis_gen::AXIS_TOL_MM;

/// 2D で見る構面の対象。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameTarget {
    /// 通り芯 1 本（軸組図）。`group` は [`Model::axes`] の添字、`axis` はその
    /// グループ内の添字。
    Axis { group: usize, axis: usize },
    /// 階 1 つ（伏図）。
    Story(StoryId),
}

/// 切り出した構面。
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    /// 表示名（`X1 通り`・`2FL` など）。
    pub label: String,
    /// 構面の単位法線。正対で見る向きを決める。
    pub normal: [f64; 3],
    /// 節点が構面に属するか（[`Model::nodes`] と同順・同長）。
    pub node_on: Vec<bool>,
    /// 要素が構面に属するか（[`Model::elements`] と同順・同長）。
    pub elem_on: Vec<bool>,
}

impl Frame {
    /// 構面に属する要素の数。
    pub fn elem_count(&self) -> usize {
        self.elem_on.iter().filter(|&&b| b).count()
    }
}

/// モデルから構面を切り出す。対象が存在しない場合は `None`。
///
/// **通り**（軸組図）に属する節点は、「通り芯の所属節点リストに載っている **または**
/// そのグループでの離れが [`FRAME_TOL_MM`] 以内」のいずれかを満たすものとする。
/// リストだけでは、柱間の大梁が中間節点で分割されているときに中間節点が柱節点で
/// ないため大梁が丸ごと落ちる。座標だけでは、通りの位置と柱の芯がずれた
/// **芯ずれ**（ST-Bridge の実ファイルで見られる）の柱が落ちる。両方を拾うため和を採る。
///
/// **階**（伏図）に属する節点は、その階に所属する節点とする。要素は「全材端節点が
/// その階に属するもの」に加え、**上端節点がその階に属する柱**を含める（柱は上下 2 つの
/// 階レベルにまたがるため、前者だけでは伏図から柱が消え、梁の支持位置が読めなくなる）。
/// 柱の所属階の規則は階の主要構造種別の判定と同じ「材端節点のうち最も高い節点の所属階」
/// （[`Model::member_story`]）。
pub fn build_frame(model: &Model, target: FrameTarget) -> Option<Frame> {
    match target {
        FrameTarget::Axis { group, axis } => build_axis_frame(model, group, axis),
        FrameTarget::Story(id) => build_story_frame(model, id),
    }
}

fn build_axis_frame(model: &Model, gi: usize, ai: usize) -> Option<Frame> {
    let group = model.axes.get(gi)?;
    let ax = group.axes.get(ai)?;

    let listed: std::collections::HashSet<_> = ax.nodes.iter().copied().collect();
    let node_on: Vec<bool> = model
        .nodes
        .iter()
        .map(|n| {
            if listed.contains(&n.id) {
                return true;
            }
            match (ax.distance, group.kind.distance_of(n.coord[0], n.coord[1])) {
                (Some(want), Some(got)) => (got - want).abs() <= FRAME_TOL_MM,
                _ => false,
            }
        })
        .collect();

    // 法線: 平行芯は幾何から厳密に決まる（離れを測る向き＝面の法線で、厳密に鉛直な面
    // になる）。平行芯以外（円弧芯・放射芯・作図芯）は幾何を持たないため、所属節点群へ
    // 平面を当てはめて求める。
    let normal = match group.kind.offset_dir() {
        Some(d) => [d[0], d[1], 0.0],
        None => {
            let pts: Vec<[f64; 3]> = ax
                .nodes
                .iter()
                .filter_map(|id| model.nodes.get(id.index()).map(|n| n.coord))
                .collect();
            best_fit_plane_normal(&pts).unwrap_or([1.0, 0.0, 0.0])
        }
    };

    let elem_on = elements_fully_on(model, &node_on);
    Some(Frame {
        label: format!("{} 通り", ax.name),
        normal,
        node_on,
        elem_on,
    })
}

fn build_story_frame(model: &Model, id: StoryId) -> Option<Frame> {
    let story = model.stories.iter().find(|s| s.id == id)?;
    let listed: std::collections::HashSet<_> = story.node_ids.iter().copied().collect();
    let node_on: Vec<bool> = model
        .nodes
        .iter()
        .map(|n| n.story == Some(id) || listed.contains(&n.id))
        .collect();

    let mut elem_on = elements_fully_on(model, &node_on);
    // 上端節点がその階に属する柱を加える（伏図の柱位置）。
    for (i, e) in model.elements.iter().enumerate() {
        if elem_on[i] || !is_column(model, e) {
            continue;
        }
        if model.member_story(e) == Some(id) {
            elem_on[i] = true;
        }
    }

    Some(Frame {
        label: story.name.clone(),
        normal: [0.0, 0.0, 1.0],
        node_on,
        elem_on,
    })
}

/// 全材端節点が構面に属する要素を選ぶ。
fn elements_fully_on(model: &Model, node_on: &[bool]) -> Vec<bool> {
    model
        .elements
        .iter()
        .map(|e| {
            !e.nodes.is_empty()
                && e.nodes
                    .iter()
                    .all(|n| node_on.get(n.index()).copied().unwrap_or(false))
        })
        .collect()
}

/// 柱（鉛直な線材）か。
fn is_column(model: &Model, e: &crate::model::ElementData) -> bool {
    if !matches!(e.kind, ElementKind::Beam) || e.nodes.len() != 2 {
        return false;
    }
    let (Some(a), Some(b)) = (
        model.nodes.get(e.nodes[0].index()),
        model.nodes.get(e.nodes[1].index()),
    ) else {
        return false;
    };
    is_vertical_pair(a.coord, b.coord)
}

#[cfg(test)]
mod tests;
