//! 要素種別に依存しない汎用の数値ヘルパ。

/// 小行列（n≤6）のガウス・ジョルダン逆行列（部分ピボッティング付き）。
/// 特異（ピボット候補の最大絶対値が行列スケール比 1e-12 未満）の場合は `None`。
///
/// 戻り値はフラット化した n×n 逆行列（先頭 `n*n` 要素のみ有効）。
/// 作業配列はスタック上に長さ 72（=6×12）で確保する。
/// n>6 は範囲外で添字パニックする。
pub(crate) fn invert_small(a: &[f64], n: usize) -> Option<[f64; 36]> {
    debug_assert!(n <= 6, "invert_small: n は解放自由度の上限 6 まで");
    let scale = a.iter().fold(0.0_f64, |m, &v| m.max(v.abs()));
    if !scale.is_finite() || scale <= 0.0 {
        return None;
    }
    let tol = 1e-12 * scale;
    let w = 2 * n;
    let mut aug = [0.0_f64; 72];
    for i in 0..n {
        for j in 0..n {
            aug[i * w + j] = a[i * n + j];
        }
        aug[i * w + n + i] = 1.0;
    }
    for col in 0..n {
        let mut best = col;
        let mut best_abs = aug[col * w + col].abs();
        for row in (col + 1)..n {
            let v = aug[row * w + col].abs();
            if v > best_abs {
                best = row;
                best_abs = v;
            }
        }
        if best_abs < tol {
            return None;
        }
        if best != col {
            for j in 0..w {
                aug.swap(col * w + j, best * w + j);
            }
        }
        let pivot = aug[col * w + col];
        for j in 0..w {
            aug[col * w + j] /= pivot;
        }
        for row in 0..n {
            if row != col {
                let factor = aug[row * w + col];
                if factor != 0.0 {
                    for j in 0..w {
                        aug[row * w + j] -= factor * aug[col * w + j];
                    }
                }
            }
        }
    }
    let mut inv = [0.0_f64; 36];
    for i in 0..n {
        for j in 0..n {
            inv[i * n + j] = aug[i * w + n + j];
        }
    }
    Some(inv)
}

#[cfg(test)]
mod tests {
    use super::invert_small;

    #[test]
    fn test_invert_small_identity_roundtrip() {
        let a = vec![4.0, 1.0, 1.0, 3.0];
        let inv = invert_small(&a, 2).expect("正則行列は逆行列を持つ");
        for i in 0..2 {
            for j in 0..2 {
                let mut s = 0.0;
                for k in 0..2 {
                    s += a[i * 2 + k] * inv[k * 2 + j];
                }
                let expect = if i == j { 1.0 } else { 0.0 };
                assert!((s - expect).abs() < 1e-12, "A·A⁻¹[{i}][{j}] = {s}");
            }
        }
    }

    /// 対角先頭が 0 でも行交換（部分ピボッティング）で解けること。
    #[test]
    fn test_invert_small_pivoting_zero_diagonal() {
        let a = vec![0.0, 1.0, 1.0, 0.0];
        let inv = invert_small(&a, 2).expect("置換行列は正則");
        assert!((inv[0] - 0.0).abs() < 1e-12 && (inv[1] - 1.0).abs() < 1e-12);
        assert!((inv[2] - 1.0).abs() < 1e-12 && (inv[3] - 0.0).abs() < 1e-12);
    }

    /// 特異行列は None。
    #[test]
    fn test_invert_small_singular_returns_none() {
        assert!(invert_small(&[0.0; 4], 2).is_none(), "零行列");
        assert!(
            invert_small(&[1.0, 2.0, 2.0, 4.0], 2).is_none(),
            "ランク落ち"
        );
    }

    /// 特異判定は**行列スケールとの比**で行う（絶対値ではない）。
    #[test]
    fn test_invert_small_singularity_is_relative_to_scale() {
        assert!(
            invert_small(&[1.0e6, 0.0, 0.0, 1.0e-7], 2).is_none(),
            "スケール比 1e-13 は特異"
        );
        let inv = invert_small(&[1.0e6, 0.0, 0.0, 1.0e-5], 2)
            .expect("スケール比 1e-11 は正則として解ける");
        assert!((inv[0] - 1.0e-6).abs() < 1e-18, "inv[0] = {}", inv[0]);
        assert!((inv[3] - 1.0e5).abs() < 1e-9, "inv[3] = {}", inv[3]);
    }

    /// 上限の n=6 でも解ける。
    #[test]
    fn test_invert_small_max_size_six() {
        let mut a = vec![0.0; 36];
        for i in 0..6 {
            for j in 0..6 {
                a[i * 6 + j] = if i == j { 10.0 } else { 1.0 };
            }
        }
        let inv = invert_small(&a, 6).expect("対角優位な行列は正則");
        for i in 0..6 {
            for j in 0..6 {
                let mut s = 0.0;
                for k in 0..6 {
                    s += a[i * 6 + k] * inv[k * 6 + j];
                }
                let expect = if i == j { 1.0 } else { 0.0 };
                assert!((s - expect).abs() < 1e-12, "A·A⁻¹[{i}][{j}] = {s}");
            }
        }
    }
}
