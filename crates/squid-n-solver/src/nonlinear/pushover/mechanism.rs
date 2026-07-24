//! 崩壊機構の判定（P5 §7.4 / §11.5）。
//!
//! - [`compute_static_indeterminacy`] — 平面骨組の静的不静定次数
//! - [`determine_mechanism`] — 降伏ヒンジ分布から崩壊機構種別を分類

use super::types::{HingeEvent, HingeLevel, MechanismType};
use crate::analysis::SeismicDir;
use squid_n_core::ids::StoryId;
use squid_n_core::model::Model;

/// 部材端ヒンジが属する階を返す。ヒンジ位置側の節点 story を優先し、
/// 未割当（基礎節点など story=None）の場合は相手端の節点 story で補完する。
fn hinge_story(model: &Model, h: &HingeEvent) -> Option<StoryId> {
    let elem = model.elements.iter().find(|e| e.id == h.elem)?;
    if elem.nodes.len() < 2 {
        return None;
    }
    let (near, far) = if h.pos < 0.5 {
        (elem.nodes[0], elem.nodes[1])
    } else {
        (elem.nodes[1], elem.nodes[0])
    };
    model
        .nodes
        .get(near.index())
        .and_then(|n| n.story)
        .or_else(|| model.nodes.get(far.index()).and_then(|n| n.story))
}

/// 平面骨組の静的不静定次数 r = 3m − 3n + r_support を算出する（P5 §11.5）。
///
/// - m: 部材数（`model.elements.len()`）
/// - n: 節点数（`model.nodes.len()`）
/// - r_support: 各節点で拘束された平面 DoF (ux, uz, ry) の総数
///
/// 3D 6DOF モデルを pushover 方向の平面骨組と見なして次数を計算する。
/// 機構成立条件は `形成降伏ヒンジ数 >= r + 1`（運動学的判定）。
///
/// 加力方向で平面 DoF が異なる: X 加力は X–Z 面 `ux(0), uz(2), ry(4)`、Y 加力は
/// Y–Z 面 `uy(1), uz(2), rx(3)`。従来は方向によらず X–Z 面固定で数えており、
/// 非対称拘束（ピン・ローラー）を持つ Y 加力モデルで支点拘束数を誤っていた
/// （基部が全 6 自由度拘束の一般的なモデルでは X/Y いずれも 3 で偶然一致し隠蔽）。
pub(crate) fn compute_static_indeterminacy(model: &Model, dir: SeismicDir) -> usize {
    let m = model.elements.len();
    let n = model.nodes.len();
    // 加力方向の平面内 DoF ビット（並進2＋面内回転1）。
    let plane_bits: [u8; 3] = match dir {
        SeismicDir::X => [0, 2, 4], // ux, uz, ry
        SeismicDir::Y => [1, 2, 3], // uy, uz, rx
    };
    let r_support: usize = model
        .nodes
        .iter()
        .map(|node| {
            let bits = node.restraint.0;
            plane_bits
                .iter()
                .filter(|&&b| bits & (1u8 << b) != 0)
                .count()
        })
        .sum();
    (3 * m + r_support).saturating_sub(3 * n)
}

/// 崩壊機構の判定（P5 §7.4 / §11.5）。
///
/// 降伏以上（Yield/Ultimate）の塑性ヒンジのみを対象とし、運動学的機構成立判定
/// `形成降伏ヒンジ数 >= 静的不静定次数 + 1` でゲートした上で、階分布から機構種別を分類:
/// - 形成降伏ヒンジ数 < r + 1 → まだ機構未成立（Partial）
/// - 複数階モデルで降伏ヒンジが単一階に集中 → 層崩壊（StoryCollapse）
/// - それ以外（複数階に分布／単一階構造）→ 全体崩壊（Overall）
pub(crate) fn determine_mechanism(
    hinges: &[HingeEvent],
    model: &Model,
    dir: SeismicDir,
) -> MechanismType {
    use std::collections::{BTreeMap, BTreeSet};

    let yielded: Vec<&HingeEvent> = hinges
        .iter()
        .filter(|h| matches!(h.level, HingeLevel::Yield | HingeLevel::Ultimate))
        .collect();

    // 運動学的機構成立ゲート: 形成降伏ヒンジ数 >= r+1
    let distinct_ends: BTreeSet<(u32, u8)> = yielded
        .iter()
        .map(|h| (h.elem.index() as u32, if h.pos < 0.5 { 0u8 } else { 1u8 }))
        .collect();
    let r = compute_static_indeterminacy(model, dir);
    if yielded.is_empty() || distinct_ends.len() < r + 1 {
        return MechanismType::Partial;
    }

    // 降伏ヒンジの階分布を集計。
    let mut per_story: BTreeMap<u32, usize> = BTreeMap::new();
    let mut story_ids: BTreeMap<u32, StoryId> = BTreeMap::new();
    let mut unmapped = 0usize;
    for h in &yielded {
        match hinge_story(model, h) {
            Some(s) => {
                *per_story.entry(s.index() as u32).or_default() += 1;
                story_ids.insert(s.index() as u32, s);
            }
            None => unmapped += 1,
        }
    }

    let n_model_stories = model.stories.len();
    if n_model_stories > 1 && per_story.len() == 1 && unmapped == 0 {
        // 単一階に塑性化が集中 → 層崩壊機構。
        let key = *per_story.keys().next().unwrap();
        MechanismType::StoryCollapse {
            story: story_ids[&key],
        }
    } else {
        // 複数階に分布、または単一階構造 → 全体崩壊機構。
        MechanismType::Overall
    }
}
