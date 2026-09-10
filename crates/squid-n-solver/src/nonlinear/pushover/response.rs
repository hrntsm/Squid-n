//! 層・全体の応答量算定。
//!
//! - [`compute_base_shear`] — ベースシア（内力の釣合いから）
//! - [`compute_story_shear`] — 層せん断力
//! - [`compute_story_drift`] — 層間変位
//! - [`story_heights`] — 階高（elevation の隣接差分）
//! - [`max_story_drift_angle`] — 全層の最大層間変形角
//! - [`story_reference_node`] — 階を代表する節点（剛床マスター、なければ最重量節点）
//! - [`get_roof_disp`] / [`get_roof_dof`] — 屋根（最上階の代表節点）の変位・DOF

use crate::statics::analysis::SeismicDir;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::NodeId;
use squid_n_core::model::{Model, Story};

/// 載荷方向 [`SeismicDir`] を並進 DOF の添字（X→0, Y→1）へ変換する。
fn dir_index(dir: SeismicDir) -> usize {
    match dir {
        SeismicDir::X => 0,
        SeismicDir::Y => 1,
    }
}

/// ベースシア（層せん断の総和）を内力の釣合いから求める。
pub(crate) fn compute_base_shear(
    model: &Model,
    dofmap: &DofMap,
    f_int: &[f64],
    dir: SeismicDir,
) -> f64 {
    let dir_idx = dir_index(dir);
    let mut v = 0.0;
    for node in &model.nodes {
        let g = node.id.index() * 6 + dir_idx;
        if let Some(a) = dofmap.active(g) {
            v += f_int[a as usize];
        }
    }
    v
}

/// 層せん断力を内力の釣合いから求める。
///
/// 第 i 層以上の層に属する節点の載荷方向水平内力の合計（上層から累積）。
/// 層がなければ空ベクトルを返す。
pub(crate) fn compute_story_shear(
    model: &Model,
    dofmap: &DofMap,
    f_int: &[f64],
    dir: SeismicDir,
) -> Vec<f64> {
    let dir_idx = dir_index(dir);
    let layers = model.layers();
    let n = layers.len();
    let mut level_force = vec![0.0; n];
    for layer in &layers {
        for nid in &layer.node_ids {
            let g = nid.index() * 6 + dir_idx;
            if let Some(a) = dofmap.active(g) {
                if let Some(&v) = f_int.get(a as usize) {
                    level_force[layer.index] += v;
                }
            }
        }
    }
    let mut shear = vec![0.0; n];
    let mut acc = 0.0;
    for i in (0..n).rev() {
        acc += level_force[i];
        shear[i] = acc;
    }
    shear
}

/// 階を代表する節点。
///
/// 剛床がある階はマスター、なければ質量最大の節点（同点は ID 最小）を返す。
/// 代表を採れない階は `None`。
pub fn story_reference_node(model: &Model, story: &Story) -> Option<NodeId> {
    if let Some(dia) = model.diaphragms_of(story.id).next() {
        return Some(dia.master);
    }
    story
        .node_ids
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .max_by(|a, b| {
            let ma = a.mass.map(|m| m[0]).unwrap_or(0.0);
            let mb = b.mass.map(|m| m[0]).unwrap_or(0.0);
            ma.total_cmp(&mb).then(b.id.0.cmp(&a.id.0))
        })
        .map(|n| n.id)
}

/// 層間変位を階の代表節点の水平変位差から求める。
///
/// 代表がない／拘束済みの階は変位 0 とみなす。
pub(crate) fn compute_story_drift(
    model: &Model,
    dofmap: &DofMap,
    total_disp: &[f64],
    dir: SeismicDir,
) -> Vec<f64> {
    let dir_idx = dir_index(dir);
    let disp_of = |sid: squid_n_core::ids::StoryId| -> f64 {
        model
            .stories
            .get(sid.index())
            .and_then(|story| story_reference_node(model, story))
            .and_then(|node| {
                let g = node.index() * 6 + dir_idx;
                dofmap
                    .active(g)
                    .and_then(|a| total_disp.get(a as usize).copied())
            })
            .unwrap_or(0.0)
    };
    model
        .layers()
        .iter()
        .map(|l| disp_of(l.top) - disp_of(l.bottom))
        .collect()
}

/// 各層の階高 [mm]（[`squid_n_core::model::Layer::height`]）。
/// 逆転・重複入力による非正値は 0 とする（層間変形角の算定では 0 除算を避けるため
/// [`max_story_drift_angle`] 側で高さ 0 の層を判定対象外にする）。
pub(crate) fn story_heights(model: &Model) -> Vec<f64> {
    model.layers().iter().map(|l| l.height.max(0.0)).collect()
}

/// 全層の最大層間変形角 [rad]（|層間変位| / 階高 の最大値）。
/// 階高 0 以下の層（標高の逆転・重複入力）は判定対象外とする。
pub(crate) fn max_story_drift_angle(drifts: &[f64], heights: &[f64]) -> f64 {
    drifts
        .iter()
        .zip(heights.iter())
        .filter(|(_, &h)| h > 0.0)
        .map(|(&d, &h)| d.abs() / h)
        .fold(0.0_f64, f64::max)
}

pub(crate) fn get_roof_disp(
    total_disp: &[f64],
    model: &Model,
    dofmap: &DofMap,
    dir: SeismicDir,
) -> f64 {
    if let Some(story) = model.stories.last() {
        if let Some(node) = story_reference_node(model, story) {
            let dof_idx = dir_index(dir);
            let g = node.index() * 6 + dof_idx;
            if let Some(a) = dofmap.active(g) {
                let idx = a as usize;
                if idx < total_disp.len() {
                    return total_disp[idx];
                }
            }
        }
    }
    0.0
}

pub(crate) fn get_roof_dof(model: &Model, dofmap: &DofMap, dir: SeismicDir) -> Option<usize> {
    let dir_idx = dir_index(dir);
    if let Some(story) = model.stories.last() {
        if let Some(node) = story_reference_node(model, story) {
            let g = node.index() * 6 + dir_idx;
            return dofmap.active(g).map(|a| a as usize);
        }
    }
    None
}
