//! 時刻歴再生の時刻・フレーム管理。
//!
//! `viewer` ハブからの構造分割。アルゴリズム変更は行わない。

pub(super) fn advance_play_time(current: f64, dt_real: f32, speed: f32, duration: f64) -> f64 {
    if duration <= 0.0 {
        return 0.0;
    }
    let next = current + dt_real as f64 * speed as f64;
    next.rem_euclid(duration)
}

/// 再生経過時刻 `t` に対応するフレーム番号を返す（`frame_time` は昇順を仮定）。
/// `t` 以下で最大の時刻を持つフレームを選ぶ（`t` が全フレームの時刻より小さければ 0）。
pub(super) fn frame_at_time(frame_time: &[f64], t: f64) -> usize {
    if frame_time.is_empty() {
        return 0;
    }
    match frame_time
        .binary_search_by(|probe| probe.partial_cmp(&t).unwrap_or(std::cmp::Ordering::Equal))
    {
        Ok(i) => i,
        Err(0) => 0,
        Err(i) => (i - 1).min(frame_time.len() - 1),
    }
}

#[cfg(test)]
mod th_playback_tests {
    use super::*;

    /// 再生時刻は dt×速度だけ単調に進む（周回しない範囲）。
    #[test]
    fn advance_play_time_accumulates() {
        let t = advance_play_time(1.0, 0.1, 2.0, 10.0);
        // dt_real は f32 のため f64 変換で微小誤差が入る（許容差は f32 精度基準）。
        assert!((t - 1.2).abs() < 1e-6, "t={t}");
    }

    /// 総時間を超えたら先頭へ周回する。
    #[test]
    fn advance_play_time_wraps_at_duration() {
        let t = advance_play_time(9.5, 1.0, 1.0, 10.0);
        assert!((t - 0.5).abs() < 1e-9, "got {t}");
    }

    /// duration が 0 以下なら常に 0。
    #[test]
    fn advance_play_time_zero_duration() {
        assert_eq!(advance_play_time(5.0, 1.0, 1.0, 0.0), 0.0);
    }

    /// 各フレーム時刻ちょうどではそのフレーム番号を返す。
    #[test]
    fn frame_at_time_exact_hits() {
        let ft = [0.0, 0.5, 1.0, 1.5];
        assert_eq!(frame_at_time(&ft, 0.0), 0);
        assert_eq!(frame_at_time(&ft, 0.5), 1);
        assert_eq!(frame_at_time(&ft, 1.5), 3);
    }

    /// 中間の時刻は「その時刻以下で最大」のフレームになる。
    #[test]
    fn frame_at_time_between_frames() {
        let ft = [0.0, 0.5, 1.0, 1.5];
        assert_eq!(frame_at_time(&ft, 0.9), 1);
        assert_eq!(frame_at_time(&ft, 1.49), 2);
    }

    /// 範囲外（負の時刻）は 0 にクランプする。
    #[test]
    fn frame_at_time_before_start() {
        let ft = [0.2, 0.5];
        assert_eq!(frame_at_time(&ft, -1.0), 0);
    }

    /// 空配列は 0 を返す。
    #[test]
    fn frame_at_time_empty() {
        assert_eq!(frame_at_time(&[], 1.0), 0);
    }
}
