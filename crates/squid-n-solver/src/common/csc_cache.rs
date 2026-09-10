//! CSC 疎行列の組立てキャッシュ（`squid_n_math::sparse::CscAssembler`・
//! `WeightedSumCache` の薄いラッパー）。
//!
//! 非線形時刻歴応答解析の Newton 反復のように、「同じ箇所から毎回ほぼ同じ triplet 列
//! （または同じ入力行列の組）を渡して CSC 行列を組み立てる」処理を繰り返す場面で使う。
//!
//! triplet の非ゼロパターンが変わっても作り直し、結果は
//! [`squid_n_math::sparse::assemble_csc`]／[`squid_n_math::sparse::weighted_sum_csc`]
//! とビット一致する。

use faer::sparse::SparseColMat;
use squid_n_math::sparse::{CscAssembler, Triplet, WeightedSumCache};

/// [`CscAssembler`] のキャッシュラッパー。triplet 列から CSC 行列を組み立てる
/// （`assemble_k`・`Reducer::reduce_k` の高速化に使う）。
pub struct CscCache {
    assembler: Option<CscAssembler>,
    n: usize,
}

impl CscCache {
    pub fn new() -> Self {
        Self {
            assembler: None,
            n: 0,
        }
    }

    /// `triplets` から CSC 行列を組み立てる。結果は常に
    /// `squid_n_math::sparse::assemble_csc(n, triplets.to_vec())` とビット一致する。
    pub fn assemble(&mut self, n: usize, triplets: &[Triplet]) -> SparseColMat<usize, f64> {
        self.assemble_ref(n, triplets).clone()
    }

    /// [`Self::assemble`] と同じ結果を、内部保持する行列への参照として返す
    /// （`.clone()` を伴わない。Newton 反復のように結果をすぐに読むだけで
    /// 所有権が要らない呼び出し元向け）。返す参照の寿命は `self` への
    /// 可変借用と結びつくため、次に `self` を可変に使う（別の `assemble`/
    /// `assemble_ref` を呼ぶ等）までしか保持できない。
    pub fn assemble_ref(&mut self, n: usize, triplets: &[Triplet]) -> &SparseColMat<usize, f64> {
        match self.assembler.as_mut() {
            Some(asm) if self.n == n => {
                asm.assemble_into(triplets);
            }
            _ => {
                self.assembler = Some(CscAssembler::new(n, triplets));
                self.n = n;
            }
        }
        self.assembler
            .as_ref()
            .expect("直前の分岐で必ず Some を構築済み")
            .matrix()
    }
}

impl Default for CscCache {
    fn default() -> Self {
        Self::new()
    }
}

/// [`WeightedSumCache`] のキャッシュラッパー。複数行列の重み付き和（`K_eff = K_t +
/// c2·C + c1·M` 等）を組み立てる。
pub struct WeightedSumGuard {
    cache: Option<WeightedSumCache>,
    n: usize,
}

impl WeightedSumGuard {
    pub fn new() -> Self {
        Self { cache: None, n: 0 }
    }

    /// `mats: &[(coef, &SparseColMat)]` の重み付き和を組み立てる。結果は常に
    /// `squid_n_math::sparse::weighted_sum_csc(n, mats)` とビット一致する。
    #[allow(dead_code)]
    pub fn combine(
        &mut self,
        n: usize,
        mats: &[(f64, &SparseColMat<usize, f64>)],
    ) -> SparseColMat<usize, f64> {
        self.combine_ref(n, mats).clone()
    }

    /// [`Self::combine`] と同じ結果を、内部保持する行列への参照として返す
    /// （`.clone()` を伴わない）。[`CscCache::assemble_ref`] と同様、返す参照の
    /// 寿命は `self` への可変借用と結びつく。
    pub fn combine_ref(
        &mut self,
        n: usize,
        mats: &[(f64, &SparseColMat<usize, f64>)],
    ) -> &SparseColMat<usize, f64> {
        match self.cache.as_mut() {
            Some(c) if self.n == n => {
                c.combine_into(mats);
            }
            _ => {
                self.cache = Some(WeightedSumCache::new(n, mats));
                self.n = n;
            }
        }
        self.cache
            .as_ref()
            .expect("直前の分岐で必ず Some を構築済み")
            .matrix()
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

    /// triplet の個数が同じまま座標だけが変わっても、正しい行列が組み上がること。
    #[test]
    fn test_csc_cache_same_len_different_coords() {
        let n = 3;
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
        let t2 = vec![
            Triplet {
                row: 0,
                col: 1,
                val: 100.0,
            },
            Triplet {
                row: 2,
                col: 2,
                val: 200.0,
            },
        ];
        let mut cache = CscCache::new();
        let got1 = cache.assemble(n, &t1);
        dense_eq(&got1, &assemble_csc(n, t1), n);
        let got2 = cache.assemble(n, &t2);
        dense_eq(&got2, &assemble_csc(n, t2), n);
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
