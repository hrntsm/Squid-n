//! CSC 疎行列の組立てキャッシュ（`squid_n_math::sparse::CscAssembler`・
//! `WeightedSumCache` の安全なラッパー）。
//!
//! 非線形時刻歴応答解析の Newton 反復のように、「同じ箇所から毎回ほぼ同じ triplet 列
//! （または同じ入力行列の組）を渡して CSC 行列を組み立てる」処理を繰り返す場面で使う。
//! [`squid_n_math::sparse::CscAssembler`]／[`squid_n_math::sparse::WeightedSumCache`]
//! は「2 回目以降に渡す入力の非ゼロ数・座標・並び順が初回と完全一致していること」を
//! 前提とし、パターン変化はリリースビルドでは検証しない（呼び出し側の責務）。
//!
//! しかし弾塑性要素の接線剛性は、完全塑性域への遷移で成分が厳密に 0.0 を跨ぐことが
//! あり（例: 完全弾塑性ばねの降伏後剛性が正確に 0）、triplet の非ゼロ数（長さ）が
//! Newton 反復間で変わり得る。本モジュールはこの「長さの変化」をリリースビルドでも
//! 検知し、変化していれば安全側（[`CscAssembler::new`]／[`WeightedSumCache::new`] に
//! よるパターンの作り直し）へフォールバックする。作り直した場合の結果も
//! [`squid_n_math::sparse::assemble_csc`]／[`squid_n_math::sparse::weighted_sum_csc`]
//! とビット一致する（各キャッシュ構造体自身のドキュメント参照）ため、フォールバックが
//! 起きても数値結果は変わらず、高速パスが使えない回だけコストが元に戻る。
//!
//! 座標・並び順そのものの一致は（コストが O(nnz) と軽くないため）ここでは検証せず、
//! [`CscAssembler::assemble_into`]／[`WeightedSumCache::combine_into`] 内部の
//! `debug_assert` に委ねる（デバッグビルド・テスト実行時に不整合があれば必ず落ちる）。

use faer::sparse::SparseColMat;
use squid_n_math::sparse::{CscAssembler, Triplet, WeightedSumCache};

/// [`CscAssembler`] のキャッシュラッパー。triplet 列から CSC 行列を組み立てる
/// （`assemble_k`・`Reducer::reduce_k` の高速化に使う）。
pub struct CscCache {
    assembler: Option<CscAssembler>,
    /// 直前に渡した triplet 列の長さ（非ゼロ数）。次回呼び出しでこれと異なれば
    /// パターンが変わったとみなし、[`CscAssembler::new`] で作り直す。
    len: usize,
    n: usize,
}

impl CscCache {
    pub fn new() -> Self {
        Self {
            assembler: None,
            len: 0,
            n: 0,
        }
    }

    /// `triplets` から CSC 行列を組み立てる。結果は常に
    /// `squid_n_math::sparse::assemble_csc(n, triplets.to_vec())` とビット一致する。
    ///
    /// 直前呼び出しと次元 `n`・triplet 列の長さが一致すれば
    /// [`CscAssembler::assemble_into`] の高速パス（ソート不要、O(nnz)）を使う。
    /// 一致しなければ（次元変化、または弾塑性要素の接線剛性が厳密 0.0 を跨いで
    /// 非ゼロ数が変わった場合）[`CscAssembler::new`] でパターンを作り直す
    /// （この回のみ通常の `assemble_csc` 相当のコストに戻るが、結果は変わらない）。
    pub fn assemble(&mut self, n: usize, triplets: &[Triplet]) -> SparseColMat<usize, f64> {
        let rebuild = match &self.assembler {
            Some(_) => self.n != n || self.len != triplets.len(),
            None => true,
        };
        if rebuild {
            self.assembler = Some(CscAssembler::new(n, triplets));
            self.len = triplets.len();
            self.n = n;
        } else if let Some(asm) = self.assembler.as_mut() {
            asm.assemble_into(triplets);
        }
        self.assembler
            .as_ref()
            .expect("直前の分岐で必ず Some を構築済み")
            .matrix()
            .clone()
    }
}

impl Default for CscCache {
    fn default() -> Self {
        Self::new()
    }
}

/// [`WeightedSumCache`] のキャッシュラッパー。複数行列の重み付き和（`K_eff = K_t +
/// c2·C + c1·M` 等）を組み立てる。設計方針は [`CscCache`] と同じ（各入力行列の
/// 非ゼロ数が前回と一致するかで高速パス／作り直しを切り替える）。
pub struct WeightedSumGuard {
    cache: Option<WeightedSumCache>,
    /// 直前呼び出しの各入力行列の非ゼロ数（`mats` と同じ順）。
    input_nnz: Vec<usize>,
    n: usize,
}

impl WeightedSumGuard {
    pub fn new() -> Self {
        Self {
            cache: None,
            input_nnz: Vec::new(),
            n: 0,
        }
    }

    /// `mats: &[(coef, &SparseColMat)]` の重み付き和を組み立てる。結果は常に
    /// `squid_n_math::sparse::weighted_sum_csc(n, mats)` とビット一致する。
    ///
    /// 直前呼び出しと次元・各入力行列の非ゼロ数が一致すれば
    /// [`WeightedSumCache::combine_into`] の高速パス（triplet 化・ソート不要）を使う。
    /// 一致しなければ [`WeightedSumCache::new`] で出力パターンを作り直す
    /// （[`CscCache::assemble`] と同じフォールバック方針）。
    pub fn combine(
        &mut self,
        n: usize,
        mats: &[(f64, &SparseColMat<usize, f64>)],
    ) -> SparseColMat<usize, f64> {
        let cur_nnz: Vec<usize> = mats.iter().map(|(_, m)| m.val().len()).collect();
        let rebuild = self.cache.is_none() || self.n != n || self.input_nnz != cur_nnz;
        if rebuild {
            self.cache = Some(WeightedSumCache::new(n, mats));
            self.input_nnz = cur_nnz;
            self.n = n;
        } else if let Some(c) = self.cache.as_mut() {
            c.combine_into(mats);
        }
        self.cache
            .as_ref()
            .expect("直前の分岐で必ず Some を構築済み")
            .matrix()
            .clone()
    }
}

impl Default for WeightedSumGuard {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_math::sparse::assemble_csc;

    fn dense_eq(a: &SparseColMat<usize, f64>, b: &SparseColMat<usize, f64>, n: usize) {
        for i in 0..n {
            for j in 0..n {
                let av = *a.get(i, j).unwrap_or(&0.0);
                let bv = *b.get(i, j).unwrap_or(&0.0);
                assert_eq!(
                    av.to_bits(),
                    bv.to_bits(),
                    "({i},{j}) で不一致: {av} != {bv}"
                );
            }
        }
    }

    #[test]
    fn test_csc_cache_matches_assemble_csc_across_pattern_change() {
        let n = 2;
        // 1 回目: (0,0),(1,1) の 2 要素。
        let t1 = vec![
            Triplet {
                row: 0,
                col: 0,
                val: 10.0,
            },
            Triplet {
                row: 1,
                col: 1,
                val: 20.0,
            },
        ];
        let mut cache = CscCache::new();
        let got1 = cache.assemble(n, &t1);
        dense_eq(&got1, &assemble_csc(n, t1.clone()), n);

        // 2 回目: 同じ座標・並び順で値だけ変更（高速パス）。
        let t2 = vec![
            Triplet {
                row: 0,
                col: 0,
                val: 100.0,
            },
            Triplet {
                row: 1,
                col: 1,
                val: 200.0,
            },
        ];
        let got2 = cache.assemble(n, &t2);
        dense_eq(&got2, &assemble_csc(n, t2.clone()), n);

        // 3 回目: 非ゼロ数が変わる（弾塑性要素の接線剛性が 0.0 を跨いだ想定）。
        // 安全側で作り直され、結果は変わらない。
        let t3 = vec![Triplet {
            row: 0,
            col: 0,
            val: 5.0,
        }];
        let got3 = cache.assemble(n, &t3);
        dense_eq(&got3, &assemble_csc(n, t3.clone()), n);

        // 4 回目: 非ゼロ数が元に戻る（再度パターンが変わる）。
        let got4 = cache.assemble(n, &t1);
        dense_eq(&got4, &assemble_csc(n, t1), n);
    }

    #[test]
    fn test_weighted_sum_guard_matches_weighted_sum_csc_across_nnz_change() {
        let n = 2;
        let m = assemble_csc(
            n,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 1.0,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: 1.0,
                },
            ],
        );
        let k_full = assemble_csc(
            n,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 100.0,
                },
                Triplet {
                    row: 1,
                    col: 0,
                    val: -50.0,
                },
                Triplet {
                    row: 0,
                    col: 1,
                    val: -50.0,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: 100.0,
                },
            ],
        );
        // 完全塑性化で非対角成分が消えた想定（非ゼロ数が減る）。
        let k_yielded = assemble_csc(
            n,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 10.0,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: 10.0,
                },
            ],
        );

        let mut guard = WeightedSumGuard::new();
        let got1 = guard.combine(n, &[(2.0, &m), (0.05, &k_full)]);
        dense_eq(
            &got1,
            &squid_n_math::sparse::weighted_sum_csc(n, &[(2.0, &m), (0.05, &k_full)]),
            n,
        );

        // 係数だけ変更（高速パス）。
        let got2 = guard.combine(n, &[(3.0, &m), (0.02, &k_full)]);
        dense_eq(
            &got2,
            &squid_n_math::sparse::weighted_sum_csc(n, &[(3.0, &m), (0.02, &k_full)]),
            n,
        );

        // k の非ゼロ数が変わる（安全側フォールバック）。
        let got3 = guard.combine(n, &[(3.0, &m), (0.02, &k_yielded)]);
        dense_eq(
            &got3,
            &squid_n_math::sparse::weighted_sum_csc(n, &[(3.0, &m), (0.02, &k_yielded)]),
            n,
        );
    }
}
