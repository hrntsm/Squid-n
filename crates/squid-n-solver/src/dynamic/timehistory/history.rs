//! UI 描画用の代表応答記録と層間変形角の集計。
//!
//! - [`choose_record_dir_y`] — 記録方向（X/Y）の自動選択
//! - [`pick_record_node`] — 記録節点（最上部）の選択
//! - [`record_history_step`] — 1 ステップ分の代表応答記録
//! - [`total_mass`] — 記録方向の合計質量 rᵀ·M·r
//! - [`update_story_drift`] — 層間変形角（各層最大値）の更新

use super::config::GroundMotion;
use super::result::ResponseHistory;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::model::Model;

/// 記録方向を自動選択する: `accel_y` が Some かつ Σ|accel_y| > Σ|accel_x| なら Y、
/// そうでなければ X（従来互換）。
pub(crate) fn choose_record_dir_y(wave: &GroundMotion) -> bool {
    let sum_x: f64 = wave.accel_x.iter().map(|v| v.abs()).sum();
    let sum_y: f64 = wave
        .accel_y
        .as_ref()
        .map(|a| a.iter().map(|v| v.abs()).sum())
        .unwrap_or(0.0);
    wave.accel_y.is_some() && sum_y > sum_x
}

/// 記録節点を選ぶ: 記録方向（`dir_idx`: 0=X, 1=Y）が自由な節点のうち
/// 最も標高(Z)が高いもの。
pub(crate) fn pick_record_node(
    model: &Model,
    dofmap: &DofMap,
    dir_idx: usize,
) -> Option<squid_n_core::ids::NodeId> {
    model
        .nodes
        .iter()
        .filter(|n| {
            dofmap
                .active(n.id.index() * DOF_PER_NODE + dir_idx)
                .is_some()
        })
        .max_by(|a, b| {
            a.coord[2]
                .partial_cmp(&b.coord[2])
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|n| n.id)
}

/// 層の上端・下端それぞれの代表節点（その階の所属節点の先頭）。
///
/// 下端が基部の階であっても同じ規則で引く。基部を「`story == None` の節点」として
/// 探すのは誤りである（階生成が不活性化した剛床代表節点も `story == None` で残るため、
/// 基部でない節点を掴みうる）。
fn layer_end_nodes(
    model: &Model,
    layer: &squid_n_core::model::Layer,
) -> (
    Option<squid_n_core::ids::NodeId>,
    Option<squid_n_core::ids::NodeId>,
) {
    let first_of = |sid: squid_n_core::ids::StoryId| {
        model
            .stories
            .get(sid.index())
            .and_then(|s| s.node_ids.first().copied())
    };
    (first_of(layer.top), first_of(layer.bottom))
}

/// 最上層の現在の層間変形角（符号付き、記録方向 `dir_idx`）。層が未定義なら 0。
fn current_top_drift(model: &Model, dofmap: &DofMap, u_free: &[f64], dir_idx: usize) -> f64 {
    let layers = model.layers();
    let Some(layer) = layers.last() else {
        return 0.0;
    };
    if layer.height <= 0.0 {
        return 0.0;
    }
    let (top, bot) = layer_end_nodes(model, layer);
    if let (Some(tn), Some(bn)) = (top, bot) {
        (node_disp(u_free, dofmap, tn, dir_idx) - node_disp(u_free, dofmap, bn, dir_idx))
            / layer.height
    } else {
        0.0
    }
}

/// 1 ステップ分の代表応答を記録する。
/// `dir_idx` は記録方向（0=X, 1=Y）、`rmr` は当該方向の rᵀ·M·r（合計質量、
/// [`total_mass`] 参照）、`ma_free` は自由 DOF 空間の `M·a_free`
/// （[`super::common::mass_accel_free`] で 1 ステップに 1 回だけ算定したものを
/// 呼び出し側から共有する）、`xg` は当該時刻の記録方向の地動加速度。
///
/// ベースシアは「当該方向の並進 DOF の節点慣性力 `f_abs = M·a_free + ẍg・M·r` の総和」
/// の符号反転として定義する（[`super::recording::ThRecorder`] の層せん断力と同じ
/// 定義。1 層目の層せん断力＝ベースシアという恒等関係を保つ）。
#[allow(clippy::too_many_arguments)]
pub(crate) fn record_history_step(
    history: &mut ResponseHistory,
    model: &Model,
    dofmap: &DofMap,
    dir_idx: usize,
    rmr: f64,
    u_free: &[f64],
    ma_free: &[f64],
    xg: f64,
) {
    let disp = history
        .node
        .map(|n| node_disp(u_free, dofmap, n, dir_idx))
        .unwrap_or(0.0);
    history.node_disp.push(disp);
    let mut sum_ma = 0.0;
    for ni in 0..model.nodes.len() {
        if let Some(a) = dofmap.active(ni * DOF_PER_NODE + dir_idx) {
            sum_ma += ma_free.get(a as usize).copied().unwrap_or(0.0);
        }
    }
    history.base_shear.push(-(sum_ma + xg * rmr));
    history
        .top_drift_angle
        .push(current_top_drift(model, dofmap, u_free, dir_idx));
}

/// rᵀ·M·r （記録方向 `dir_idx` の合計質量）。ベースシア計算に使う。
pub(crate) fn total_mass(m_r: &[f64], dofmap: &DofMap, n_nodes: usize, dir_idx: usize) -> f64 {
    let mut s = 0.0;
    for ni in 0..n_nodes {
        if let Some(a) = dofmap.active(ni * DOF_PER_NODE + dir_idx) {
            s += m_r[a as usize];
        }
    }
    s
}

/// 層間変形角を更新する（各層の最大値を追跡）。X 方向の水平変位差／階高。
pub(crate) fn update_story_drift(
    model: &Model,
    dofmap: &DofMap,
    u_free: &[f64],
    story_drift_angle: &mut [f64],
) {
    for layer in model.layers() {
        let Some(slot) = story_drift_angle.get_mut(layer.index) else {
            break;
        };
        if layer.height <= 0.0 {
            continue;
        }
        let (top, bot) = layer_end_nodes(model, &layer);
        if let (Some(tn), Some(bn)) = (top, bot) {
            // 層間変形角は従来通り X 方向（0）で評価する（ResponseHistory の
            // 記録方向とは独立）。
            let du = (node_disp(u_free, dofmap, tn, 0) - node_disp(u_free, dofmap, bn, 0)).abs();
            let angle = du / layer.height;
            if angle > *slot {
                *slot = angle;
            }
        }
    }
}

/// 節点の並進自由度 `dir_idx`（0=X, 1=Y, 2=Z）の相対変位を返す。
fn node_disp(
    u_free: &[f64],
    dofmap: &DofMap,
    node_id: squid_n_core::ids::NodeId,
    dir_idx: usize,
) -> f64 {
    let ni = node_id.index();
    let g = ni * DOF_PER_NODE + dir_idx;
    if let Some(a) = dofmap.active(g) {
        u_free[a as usize]
    } else {
        0.0
    }
}
