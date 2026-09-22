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

    /// 再生時刻は dt×速度だけ進み、総時間を超えたら先頭へ周回する。
    /// duration が 0 以下なら常に 0。ケースは (現在時刻, dt, 速度, 総時間, 期待値)。
    #[test]
    fn advance_play_time_cases() {
        let cases = [
            (1.0, 0.1_f32, 2.0_f32, 10.0, 1.2),
            (9.5, 1.0, 1.0, 10.0, 0.5),
            (5.0, 1.0, 1.0, 0.0, 0.0),
        ];
        for (current, dt_real, speed, duration, expected) in cases {
            let t = advance_play_time(current, dt_real, speed, duration);
            assert!((t - expected).abs() < 1e-6, "t={t} expected={expected}");
        }
    }

    /// 再生経過時刻に対応するフレーム番号。フレーム時刻ちょうどはその番号、
    /// 中間は「その時刻以下で最大」、負の時刻・空配列は 0。
    #[test]
    fn frame_at_time_cases() {
        let ft = [0.0, 0.5, 1.0, 1.5];
        let cases = [(0.0, 0), (0.5, 1), (1.5, 3), (0.9, 1), (1.49, 2)];
        for (t, expected) in cases {
            assert_eq!(frame_at_time(&ft, t), expected, "t={t}");
        }
        assert_eq!(frame_at_time(&[0.2, 0.5], -1.0), 0, "負の時刻");
        assert_eq!(frame_at_time(&[], 1.0), 0, "空配列");
    }
}
