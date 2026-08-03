//! 梁固有ロジックを持たない汎用数値ヘルパ。
//!
//! 小行列のガウス・ジョルダン逆行列。剛性の静縮約（[`super::stiffness`]）や
//! 集中ばね・側柱要素（`concentrated`・`side_column`）から利用される。

/// 小行列のガウス・ジョルダン逆行列（部分ピボッティング付き）。
/// 特異（ピボット候補の最大絶対値が行列スケール比 1e-12 未満）の場合は `None`。
///
/// 従来はピボット選択なし・特異時にピボットを 1.0 へ差し替えて続行しており、
/// (a) 対角に小さい値が来る材端解放の組合せで精度が桁で落ちる、
/// (b) 特異な縮約行列（＝機構化した部材）から逆行列ではない別の行列が返り、
/// もっともらしい剛性が全体 K に混入する、という危険側の無音回避だった。
/// 呼び出し側は `None` を「補正項の省略」（機構を全体求解の特異検出に委ねる）
/// または「反復の打ち切り」として明示的に扱うこと。
pub(crate) fn invert_small(a: &[f64], n: usize) -> Option<Vec<f64>> {
    let scale = a.iter().fold(0.0_f64, |m, &v| m.max(v.abs()));
    if !scale.is_finite() || scale <= 0.0 {
        return None;
    }
    let tol = 1e-12 * scale;
    let w = 2 * n;
    let mut aug = vec![0.0; n * w];
    for i in 0..n {
        for j in 0..n {
            aug[i * w + j] = a[i * n + j];
        }
        aug[i * w + n + i] = 1.0;
    }
    for col in 0..n {
        // 部分ピボッティング: col 列で絶対値最大の行を選ぶ。
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
    let mut inv = vec![0.0; n * n];
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
        // A·A⁻¹ = I
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
    /// 従来はピボット 0 → 1.0 差し替えで誤った行列を返していた。
    #[test]
    fn test_invert_small_pivoting_zero_diagonal() {
        let a = vec![0.0, 1.0, 1.0, 0.0];
        let inv = invert_small(&a, 2).expect("置換行列は正則");
        assert!((inv[0] - 0.0).abs() < 1e-12 && (inv[1] - 1.0).abs() < 1e-12);
        assert!((inv[2] - 1.0).abs() < 1e-12 && (inv[3] - 0.0).abs() < 1e-12);
    }

    /// 特異行列は None（従来はもっともらしい行列を無言で返していた）。
    #[test]
    fn test_invert_small_singular_returns_none() {
        assert!(invert_small(&[0.0; 4], 2).is_none(), "零行列");
        assert!(
            invert_small(&[1.0, 2.0, 2.0, 4.0], 2).is_none(),
            "ランク落ち"
        );
    }
}
