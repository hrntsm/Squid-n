//! 部材ランク（FA..FD）の集約と層 Ds の自動分類。
use super::holding_capacity::{ds_value, FrameType, MemberRank};
use squid_n_solver::nonlinear::pushover::MechanismType;

/// ランクを 0(FA)..3(FD) の整数インデックスに変換する。
fn rank_index(r: MemberRank) -> u8 {
    match r {
        MemberRank::FA => 0,
        MemberRank::FB => 1,
        MemberRank::FC => 2,
        MemberRank::FD => 3,
    }
}

/// 整数インデックスをランクに変換する。インデックスが 3 を超える場合は FD を返す。
fn index_rank(i: u8) -> MemberRank {
    match i {
        0 => MemberRank::FA,
        1 => MemberRank::FB,
        2 => MemberRank::FC,
        _ => MemberRank::FD,
    }
}

/// 複数の部材ランクのうち最も不利（FD 寄り）なものを返す。`ranks` が空なら `None`。
///
/// 保有水平耐力（ルート3）の層ランク自動判定（UI-13）で、1 層に属する複数の
/// 鋼部材ランクから層の代表ランクを選ぶために使う。
pub fn worst_rank(ranks: &[MemberRank]) -> Option<MemberRank> {
    ranks.iter().map(|r| rank_index(*r)).max().map(index_rank)
}

/// 層 Ds 値を計算する。
///
/// # 規則
/// 1. 層の代表ランク = `ranks` 中で最も不利（FD 寄り）な部材ランク。
///    `ranks` が空の場合は FA を使用する。
/// 2. 崩壊機構補正:
///    - [`MechanismType::StoryCollapse`] または [`MechanismType::Partial`] の場合、
///      代表ランクを 1 段階不利側へ移動（FA→FB→FC→FD、FD は据え置き）。
///    - [`MechanismType::Overall`] は補正なし。
/// 3. 補正後のランクと `frame` を [`ds_value`] に渡して返す。
pub fn story_ds(ranks: &[MemberRank], frame: FrameType, mechanism: &MechanismType) -> f64 {
    let worst_index = ranks.iter().map(|r| rank_index(*r)).max().unwrap_or(0);

    let corrected_index = match mechanism {
        MechanismType::StoryCollapse { .. } | MechanismType::Partial => (worst_index + 1).min(3),
        MechanismType::Overall => worst_index,
    };

    let representative = index_rank(corrected_index);
    ds_value(frame, representative)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 最も不利（FD 寄り）なランクの選択。空は `None`。
    #[test]
    fn test_worst_rank_cases() {
        for (ranks, expected) in [
            (
                vec![MemberRank::FA, MemberRank::FC, MemberRank::FB],
                Some(MemberRank::FC),
            ),
            (vec![MemberRank::FD, MemberRank::FA], Some(MemberRank::FD)),
            (vec![], None),
        ] {
            assert_eq!(worst_rank(&ranks), expected, "{ranks:?}");
        }
    }

    /// 層 Ds: 代表ランク（最悪）＋崩壊機構補正（StoryCollapse・Partial で 1 段階不利、
    /// Overall は補正なし、空は FA、FD は据え置き）を表で確認する。
    #[test]
    fn test_story_ds_cases() {
        for (ranks, frame, mechanism, expected) in [
            (
                vec![MemberRank::FA, MemberRank::FC, MemberRank::FB],
                FrameType::RcFrame,
                MechanismType::Overall,
                0.40,
            ),
            (
                vec![MemberRank::FA, MemberRank::FC, MemberRank::FB],
                FrameType::RcFrame,
                MechanismType::StoryCollapse { layer: 0 },
                0.45,
            ),
            (
                vec![MemberRank::FA, MemberRank::FC, MemberRank::FB],
                FrameType::RcFrame,
                MechanismType::Partial,
                0.45,
            ),
            (
                vec![MemberRank::FA],
                FrameType::SteelFrame,
                MechanismType::Overall,
                0.25,
            ),
            (vec![], FrameType::RcFrame, MechanismType::Overall, 0.30),
            (
                vec![MemberRank::FD],
                FrameType::RcFrame,
                MechanismType::Overall,
                0.45,
            ),
            (
                vec![MemberRank::FD],
                FrameType::RcFrame,
                MechanismType::StoryCollapse { layer: 0 },
                0.45,
            ),
        ] {
            let ds = story_ds(&ranks, frame, &mechanism);
            assert!(
                (ds - expected).abs() < 1e-9,
                "story_ds({ranks:?}, {frame:?}, {mechanism:?}) expected {expected}, got {ds}"
            );
        }
    }
}
