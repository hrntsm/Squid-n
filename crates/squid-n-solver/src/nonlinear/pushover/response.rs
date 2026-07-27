//! 層・全体の応答量算定（P5 §7.4）。
//!
//! - [`compute_base_shear`] — ベースシア（内力の釣合いから）
//! - [`compute_story_shear`] — 層せん断力
//! - [`compute_story_drift`] — 層間変位
//! - [`story_heights`] — 階高（elevation の隣接差分）
//! - [`max_story_drift_angle`] — 全層の最大層間変形角
//! - [`get_roof_disp`] / [`get_roof_dof`] — 屋根（最上階マスター）の変位・DOF

use crate::analysis::{base_elevation, SeismicDir};
use squid_n_core::dof::DofMap;
use squid_n_core::model::Model;

/// 載荷方向 [`SeismicDir`] を並進 DOF の添字（X→0, Y→1）へ変換する。
fn dir_index(dir: SeismicDir) -> usize {
    match dir {
        SeismicDir::X => 0,
        SeismicDir::Y => 1,
    }
}

/// ベースシア（層せん断の総和）を内力の釣合いから求める（P5 §7.4）。
///
/// 静的釣合いでは各自由節点の水平内力 = 外力。よって全自由節点の載荷方向
/// 並進 DOF にわたる内力の総和が、構造全体が支持点へ伝える水平力＝ベースシア
/// に等しい。DOF 添字を直接足す旧実装（`f_int[0..roof].sum()`）は誤り。
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

/// 層せん断力を内力の釣合いから求める（P5 §7.4、P7 の Qu 突合に使用）。
///
/// 第 i 層のせん断力 Q_i = 第 i 層以上の階に属する節点へ作用する
/// 載荷方向水平内力の合計（上層から累積）。階に属さない中間節点は
/// 集計対象外（階の自動生成はレベル単位で節点をクラスタリングするため、
/// 通常のフレームでは全自由節点がいずれかの階に属する）。
/// stories が空なら空ベクトルを返す。
pub(crate) fn compute_story_shear(
    model: &Model,
    dofmap: &DofMap,
    f_int: &[f64],
    dir: SeismicDir,
) -> Vec<f64> {
    let dir_idx = dir_index(dir);
    let n = model.stories.len();
    let mut level_force = vec![0.0; n];
    for (i, story) in model.stories.iter().enumerate() {
        for nid in &story.node_ids {
            let g = nid.index() * 6 + dir_idx;
            if let Some(a) = dofmap.active(g) {
                if let Some(&v) = f_int.get(a as usize) {
                    level_force[i] += v;
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

/// 層間変位を剛床マスター節点の水平変位差から求める。
/// 第 i 層の層間変位 = マスター変位(第 i 層) − マスター変位(1 つ下の階)。
/// 最下層は基部（変位 0）との差。マスターが無い／拘束済みの階は変位 0 とみなす。
pub(crate) fn compute_story_drift(
    model: &Model,
    dofmap: &DofMap,
    total_disp: &[f64],
    dir: SeismicDir,
) -> Vec<f64> {
    let dir_idx = dir_index(dir);
    let mut prev = 0.0;
    model
        .stories
        .iter()
        .map(|story| {
            let d = story
                .diaphragms
                .first()
                .and_then(|dia| {
                    let g = dia.master.index() * 6 + dir_idx;
                    dofmap
                        .active(g)
                        .and_then(|a| total_disp.get(a as usize).copied())
                })
                .unwrap_or(0.0);
            let drift = d - prev;
            prev = d;
            drift
        })
        .collect()
}

/// 各階の階高 [mm] を階の標高（`Story::elevation`）の隣接差分から求める。
/// 最下層は最下端節点標高（[`base_elevation`]、生成マスター節点を除く最小 z）との差。
/// 逆転・重複入力による非正値は 0 とする（層間変形角の算定では 0 除算を避けるため
/// [`max_story_drift_angle`] 側で高さ 0 の層を判定対象外にする）。
pub(crate) fn story_heights(model: &Model) -> Vec<f64> {
    let mut prev = base_elevation(model);
    model
        .stories
        .iter()
        .map(|s| {
            let h = (s.elevation - prev).max(0.0);
            prev = s.elevation;
            h
        })
        .collect()
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
        if let Some(dia) = story.diaphragms.first() {
            let ni = dia.master.index();
            let dof_idx = dir_index(dir);
            let g = ni * 6 + dof_idx;
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
        if let Some(dia) = story.diaphragms.first() {
            let ni = dia.master.index();
            let g = ni * 6 + dir_idx;
            return dofmap.active(g).map(|a| a as usize);
        }
    }
    None
}
