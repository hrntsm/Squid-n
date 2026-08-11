//! 部材ランク（FA..FD）の集約と層 Ds の自動分類。
//!
//! 個々の部材ランクの判定は各判定モジュール
//! （鋼: [`crate::secondary::width_thickness::s_member_rank_by_kihon`]、
//! SRC: [`crate::secondary::src_rank`]、RC: 告示の部材種別表）が担い、
//! 本モジュールは複数部材ランクの集約（[`worst_rank`]）と
//! 層 Ds の算定（[`story_ds`]）のみを持つ。
use super::holding_capacity::{ds_value, FrameType, MemberRank};
use squid_n_solver::pushover::MechanismType;

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
    // 代表ランク: ranks が空なら FA とみなす
    let worst_index = ranks.iter().map(|r| rank_index(*r)).max().unwrap_or(0);

    // 崩壊機構補正: StoryCollapse または Partial → 1段階不利
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

    // ===== worst_rank テスト =====

    #[test]
    fn test_worst_rank_picks_fd_leaning() {
        let ranks = [MemberRank::FA, MemberRank::FC, MemberRank::FB];
        assert_eq!(worst_rank(&ranks), Some(MemberRank::FC));
    }

    #[test]
    fn test_worst_rank_empty_is_none() {
        assert_eq!(worst_rank(&[]), None);
    }

    // ===== story_ds テスト =====

    /// ranks=[FA,FC,FB], RcFrame, Overall → 代表 FC → ds_value(RcFrame,FC) = 0.40
    #[test]
    fn test_story_ds_rc_frame_overall() {
        let ranks = vec![MemberRank::FA, MemberRank::FC, MemberRank::FB];
        let ds = story_ds(&ranks, FrameType::RcFrame, &MechanismType::Overall);
        assert!((ds - 0.40).abs() < 1e-9, "expected 0.40, got {}", ds);
    }

    /// 同上で StoryCollapse → 代表 FC → FD → ds_value(RcFrame,FD) = 0.45
    #[test]
    fn test_story_ds_rc_frame_story_collapse() {
        let ranks = vec![MemberRank::FA, MemberRank::FC, MemberRank::FB];
        let ds = story_ds(
            &ranks,
            FrameType::RcFrame,
            &MechanismType::StoryCollapse { layer: 0 },
        );
        assert!((ds - 0.45).abs() < 1e-9, "expected 0.45, got {}", ds);
    }

    /// ranks=[FA], SteelFrame, Overall → 代表 FA → ds_value(SteelFrame,FA) = 0.25
    #[test]
    fn test_story_ds_steel_frame_fa_overall() {
        let ranks = vec![MemberRank::FA];
        let ds = story_ds(&ranks, FrameType::SteelFrame, &MechanismType::Overall);
        assert!((ds - 0.25).abs() < 1e-9, "expected 0.25, got {}", ds);
    }

    /// 空 ranks → FA 扱い → ds_value(RcFrame, FA) = 0.30
    #[test]
    fn test_story_ds_empty_ranks() {
        let ds = story_ds(&[], FrameType::RcFrame, &MechanismType::Overall);
        assert!(
            (ds - 0.30).abs() < 1e-9,
            "expected 0.30 for empty ranks, got {}",
            ds
        );
    }

    /// Partial でも1段階不利になる: [FA,FC,FB], RcFrame, Partial → FC → FD → 0.45
    #[test]
    fn test_story_ds_partial_downgrades_one_step() {
        let ranks = vec![MemberRank::FA, MemberRank::FC, MemberRank::FB];
        let ds = story_ds(&ranks, FrameType::RcFrame, &MechanismType::Partial);
        assert!((ds - 0.45).abs() < 1e-9, "expected 0.45, got {}", ds);
    }

    /// FD は据え置き（StoryCollapse でも FD → FD）
    #[test]
    fn test_story_ds_fd_stays_fd() {
        let ranks = vec![MemberRank::FD];
        let ds_overall = story_ds(&ranks, FrameType::RcFrame, &MechanismType::Overall);
        let ds_collapse = story_ds(
            &ranks,
            FrameType::RcFrame,
            &MechanismType::StoryCollapse { layer: 0 },
        );
        // FD は最悪なので補正後も FD のまま
        assert!(
            (ds_overall - 0.45).abs() < 1e-9,
            "FD Overall expected 0.45, got {}",
            ds_overall
        );
        assert!(
            (ds_collapse - 0.45).abs() < 1e-9,
            "FD StoryCollapse expected 0.45 (FD stays FD), got {}",
            ds_collapse
        );
    }
}
