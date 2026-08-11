//! 崩壊機構の判定（P5 §7.4 / §11.5）。
//!
//! - [`compute_static_indeterminacy`] — 平面骨組の静的不静定次数
//! - [`determine_mechanism`] — 降伏ヒンジ分布から崩壊機構種別を分類

use super::types::{HingeEvent, HingeLevel, MechanismType};
use crate::analysis::SeismicDir;
use squid_n_core::model::Model;

/// 部材端ヒンジが属する**層**の番号（[`squid_n_core::model::Layer::index`]）を返す。
///
/// 層は上端の階（床）で識別できる（層 i の上端は `stories[i+1]`）。ヒンジ位置側と
/// 相手端の節点のうち**高い側**の所属階を採り、その階を上端とする層に帰属させる。
/// 柱脚のヒンジは基部の階に属するが、それが表すのは最下層の柱脚であり、
/// 高い側（柱頭）を採ることで最下層へ正しく帰属する。
/// 最下の階（基部の床）を上端とする層は存在しないため、その場合は `None`。
fn hinge_layer(model: &Model, h: &HingeEvent) -> Option<usize> {
    let elem = model.elements.iter().find(|e| e.id == h.elem)?;
    if elem.nodes.len() < 2 {
        return None;
    }
    let (near, far) = if h.pos < 0.5 {
        (elem.nodes[0], elem.nodes[1])
    } else {
        (elem.nodes[1], elem.nodes[0])
    };
    let story_of =
        |id: squid_n_core::ids::NodeId| model.nodes.get(id.index()).and_then(|n| n.story);
    let z_of = |id: squid_n_core::ids::NodeId| {
        model
            .nodes
            .get(id.index())
            .map(|n| n.coord[2])
            .unwrap_or(f64::NEG_INFINITY)
    };
    let top_side = if z_of(far) > z_of(near) { far } else { near };
    let sid = story_of(top_side)
        .or_else(|| story_of(near))
        .or_else(|| story_of(far))?;
    // 上端の階が `stories[i+1]` である層の番号は `i = index - 1`。
    sid.index().checked_sub(1)
}

/// 平面骨組の静的不静定次数 r = 3m − 3n + r_support を算出する（P5 §11.5）。
///
/// - m: 加力平面に関与する線材の数（下記）
/// - n: それらの線材が接続する節点の数
/// - r_support: それらの節点で拘束された平面 DoF の総数
///
/// 3D 6DOF モデルを pushover 方向の平面骨組と見なして次数を計算する。
/// 機構成立条件は `形成降伏ヒンジ数 >= r + 1`（運動学的判定）。
///
/// 加力方向で平面 DoF が異なる: X 加力は X–Z 面 `ux(0), uz(2), ry(4)`、Y 加力は
/// Y–Z 面 `uy(1), uz(2), rx(3)`。従来は方向によらず X–Z 面固定で数えており、
/// 非対称拘束（ピン・ローラー）を持つ Y 加力モデルで支点拘束数を誤っていた
/// （基部が全 6 自由度拘束の一般的なモデルでは X/Y いずれも 3 で偶然一致し隠蔽）。
///
/// ## 部材・節点の集計対象（平面骨組への射影）
/// 集計対象は 2 節点の線材（梁・ファイバー梁・マルチスプリング梁・ブレース）の
/// うち、部材軸が加力直交の水平方向へ卓越しない（|軸単位ベクトルの直交水平成分|
/// ≦ 0.707）ものに限る。従来はモデル全体の要素数・節点数（直交方向の梁・壁・
/// ばね等を含む）をそのまま用いており、多方向スパンを持つ 3D モデルでは直交部材の
/// 本数だけ r が水増しされ、機構成立ゲート（≧ r+1）が過大になって層崩壊機構が
/// Partial と誤判定されていた。平面骨組の次数式をヒンジ判定対象（線材、
/// `track_hinges`）と同じ集合の上で評価するための近似であり、立体的な連成を厳密に
/// 扱うものではない。
pub(crate) fn compute_static_indeterminacy(model: &Model, dir: SeismicDir) -> usize {
    use squid_n_core::model::ElementKind;
    // 加力方向の平面内 DoF ビット（並進2＋面内回転1）と、加力直交の水平軸。
    let (plane_bits, orth_axis): ([u8; 3], usize) = match dir {
        SeismicDir::X => ([0, 2, 4], 1), // ux, uz, ry / 直交軸 Y
        SeismicDir::Y => ([1, 2, 3], 0), // uy, uz, rx / 直交軸 X
    };
    let mut counted = vec![false; model.nodes.len()];
    let mut m = 0usize;
    for e in &model.elements {
        let is_line = matches!(
            e.kind,
            ElementKind::Beam
                | ElementKind::Fiber
                | ElementKind::MultiSpring
                | ElementKind::Brace { .. }
        );
        if !is_line || e.nodes.len() != 2 {
            continue;
        }
        let (i0, i1) = (e.nodes[0].index(), e.nodes[1].index());
        let (Some(n0), Some(n1)) = (model.nodes.get(i0), model.nodes.get(i1)) else {
            continue;
        };
        let d = [
            n1.coord[0] - n0.coord[0],
            n1.coord[1] - n0.coord[1],
            n1.coord[2] - n0.coord[2],
        ];
        let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        // 加力直交方向へ卓越する部材（直交方向の梁など）は載荷平面外とみなす。
        if len > 1e-9 && (d[orth_axis] / len).abs() > 0.707 {
            continue;
        }
        m += 1;
        counted[i0] = true;
        counted[i1] = true;
    }
    let n = counted.iter().filter(|&&c| c).count();
    let r_support: usize = model
        .nodes
        .iter()
        .enumerate()
        .filter(|(i, _)| counted[*i])
        .map(|(_, node)| {
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
/// `形成降伏ヒンジ数 >= 静的不静定次数 + 1` でゲートした上で、層分布から機構種別を分類:
/// - 形成降伏ヒンジ数 < r + 1 → まだ機構未成立（Partial）
/// - 複数層モデルで降伏ヒンジが単一の層に集中 → 層崩壊（StoryCollapse）
/// - それ以外（複数の層に分布／単層構造）→ 全体崩壊（Overall）
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

    // 降伏ヒンジの層分布を集計。
    let mut per_story: BTreeMap<usize, usize> = BTreeMap::new();
    let mut unmapped = 0usize;
    for h in &yielded {
        match hinge_layer(model, h) {
            Some(l) => *per_story.entry(l).or_default() += 1,
            None => unmapped += 1,
        }
    }

    if model.layer_count() > 1 && per_story.len() == 1 && unmapped == 0 {
        // 単一の層に塑性化が集中 → 層崩壊機構。
        MechanismType::StoryCollapse {
            layer: *per_story.keys().next().unwrap(),
        }
    } else {
        // 複数の層に分布、または単層構造 → 全体崩壊機構。
        MechanismType::Overall
    }
}
