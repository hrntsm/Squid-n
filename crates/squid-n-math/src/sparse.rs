use faer::sparse::SparseColMat;

#[derive(Clone, Copy, Debug)]
pub struct Triplet {
    pub row: usize,
    pub col: usize,
    pub val: f64,
}

pub fn assemble_csc(n: usize, mut triplets: Vec<Triplet>) -> SparseColMat<usize, f64> {
    triplets.sort_by_key(|a| (a.col, a.row));
    let mut merged: Vec<faer::sparse::Triplet<usize, usize, f64>> =
        Vec::with_capacity(triplets.len());
    for t in triplets {
        match merged.last_mut() {
            Some(m) if m.row == t.row && m.col == t.col => m.val += t.val,
            _ => merged.push(faer::sparse::Triplet::new(t.row, t.col, t.val)),
        }
    }
    SparseColMat::try_new_from_triplets(n, n, &merged).expect("valid triplets")
}

/// 組立済み CSC 疎行列の非ゼロ要素を Triplet のリストへ変換する。
/// 減衰行列の組立（C = a0·M + a1·K）など、行列の加重和を再組立する用途で使う。
pub fn sparse_to_triplets(mat: &SparseColMat<usize, f64>) -> Vec<Triplet> {
    let (sym, vals) = mat.parts();
    let ncols = sym.ncols();
    let mut out = Vec::with_capacity(vals.len());
    for j in 0..ncols {
        let range = sym.col_range(j);
        let rows = sym.row_idx_of_col_raw(j);
        for (k, &row) in rows.iter().enumerate() {
            out.push(Triplet {
                row,
                col: j,
                val: vals[range.start + k],
            });
        }
    }
    out
}

/// 複数の CSC 行列を係数付きで加算し、新しい CSC を返す。
/// `mats: &[(coef, &SparseColMat)]` の各要素を coef 倍して足す。
pub fn weighted_sum_csc(
    n: usize,
    mats: &[(f64, &SparseColMat<usize, f64>)],
) -> SparseColMat<usize, f64> {
    let mut triplets = Vec::new();
    for (coef, mat) in mats {
        for t in sparse_to_triplets(mat) {
            triplets.push(Triplet {
                row: t.row,
                col: t.col,
                val: coef * t.val,
            });
        }
    }
    assemble_csc(n, triplets)
}

/// 疎行列とベクトルの積 y = A·x を計算する（CSC 形式）。
pub fn sparse_matvec(mat: &SparseColMat<usize, f64>, x: &[f64]) -> Vec<f64> {
    let n = mat.nrows();
    let mut y = vec![0.0; n];
    sparse_matvec_into(mat, x, &mut y);
    y
}

/// [`sparse_matvec`] のバッファ再利用版。`out` の長さは `mat.nrows()` と一致させておくこと
/// （呼び出し側で `out` を使い回すことで、時刻歴のようにステップごとに呼ぶ場合の
/// ベクタ確保を避けられる）。`out` は呼び出し前の値に関わらずゼロクリアしてから書き込む。
///
/// # Panics
/// `out.len() != mat.nrows()` の場合。
pub fn sparse_matvec_into(mat: &SparseColMat<usize, f64>, x: &[f64], out: &mut [f64]) {
    assert_eq!(
        out.len(),
        mat.nrows(),
        "出力バッファの長さが行数と一致しない"
    );
    out.fill(0.0);
    let (sym, vals) = mat.parts();
    let ncols = sym.ncols();
    for j in 0..ncols {
        let range = sym.col_range(j);
        let rows = sym.row_idx_of_col_raw(j);
        let xj = x[j];
        if xj == 0.0 {
            continue;
        }
        for (k, &row) in rows.iter().enumerate() {
            out[row] += vals[range.start + k] * xj;
        }
    }
}

/// (col,row) の安定ソート＋出現順の重複加算により CSC パターンを構築し、
/// 元の `triplets`（引数の並び順のまま）の各要素が対応する値スロット
/// （マージ後の値配列内インデックス）を返す。
///
/// [`assemble_csc`] と全く同じ規則（`sort_by_key` の安定性により、同一 (col,row) の
/// 複数エントリは元の相対順序を保ったまま加算される）に従うため、ここで得られる
/// `SparseColMat` は `assemble_csc(n, triplets.to_vec())` とビット一致する。
/// [`CscAssembler`]・[`WeightedSumCache`] が共有する内部ロジック。
fn build_pattern_and_slots(
    n: usize,
    triplets: &[Triplet],
) -> (SparseColMat<usize, f64>, Vec<usize>) {
    let mut order: Vec<usize> = (0..triplets.len()).collect();
    order.sort_by_key(|&i| (triplets[i].col, triplets[i].row));

    let mut merged: Vec<faer::sparse::Triplet<usize, usize, f64>> =
        Vec::with_capacity(triplets.len());
    let mut slot_of = vec![0usize; triplets.len()];
    for &i in &order {
        let t = triplets[i];
        let extends_last = matches!(merged.last(), Some(m) if m.row == t.row && m.col == t.col);
        if extends_last {
            merged.last_mut().expect("直前の判定で存在を確認済み").val += t.val;
        } else {
            merged.push(faer::sparse::Triplet::new(t.row, t.col, t.val));
        }
        slot_of[i] = merged.len() - 1;
    }
    let mat = SparseColMat::try_new_from_triplets(n, n, &merged).expect("valid triplets");
    (mat, slot_of)
}

/// スパースパターン（非ゼロ位置の集合）が反復間で不変な場合に、triplet の値だけを
/// 更新して CSC 行列を安価に再構築するためのアセンブラ。
///
/// 時刻歴解析の Newton 反復のように、要素接続（＝非ゼロパターン）は変わらず係数値
/// のみが変わる組立てを繰り返す場面で使う。初回 [`CscAssembler::new`] で
/// [`assemble_csc`] と同じ規則によりパターンと「triplet 添字→値スロット」の写像を
/// 構築し、以降の [`CscAssembler::assemble_into`] は**ソートせず** O(nnz) で値配列を
/// 再構築する（結果は [`assemble_csc`] とビット一致）。
///
/// # 使用例
/// ```
/// use squid_n_math::sparse::{CscAssembler, Triplet};
///
/// let triplets = vec![
///     Triplet { row: 0, col: 0, val: 1.0 },
///     Triplet { row: 1, col: 1, val: 2.0 },
/// ];
/// let mut asm = CscAssembler::new(2, &triplets);
/// let mat = asm.assemble_into(&triplets); // 初回と同じ座標列・値
/// assert_eq!(*mat.get(0, 0).unwrap(), 1.0);
///
/// // 座標（row, col）は同一のまま、値だけを変えて再組立て（ソート不要）。
/// let triplets2 = vec![
///     Triplet { row: 0, col: 0, val: 10.0 },
///     Triplet { row: 1, col: 1, val: 20.0 },
/// ];
/// let mat2 = asm.assemble_into(&triplets2);
/// assert_eq!(*mat2.get(0, 0).unwrap(), 10.0);
/// ```
///
/// # 前提（呼び出し側の責務）
/// 2 回目以降に渡す `triplets` は、**長さ・各要素の (row, col) 座標・並び順**が
/// 初回構築時と完全に一致していること（値のみ変わってよい）。`debug_assert` で
/// この前提を検証するが、リリースビルドでは検証されないため、呼び出し側は
/// パターン不変性を自ら担保すること。
pub struct CscAssembler {
    /// `assemble_into` に渡される triplet 列の (col, row) を、パターン検証用に保持する。
    orig_coords: Vec<(usize, usize)>,
    /// `orig_coords[i]` が対応する、マージ後の値配列内スロット位置。
    slot_of: Vec<usize>,
    mat: SparseColMat<usize, f64>,
}

impl CscAssembler {
    /// `triplets` から [`assemble_csc`] と同じ規則でパターンを構築する。
    pub fn new(n: usize, triplets: &[Triplet]) -> Self {
        let (mat, slot_of) = build_pattern_and_slots(n, triplets);
        let orig_coords = triplets.iter().map(|t| (t.col, t.row)).collect();
        Self {
            orig_coords,
            slot_of,
            mat,
        }
    }

    /// `triplets` の値でパターン不変のまま再組立てし、更新後の行列への参照を返す。
    /// ソートは行わず O(nnz) で完了する。結果は `assemble_csc(n, triplets.to_vec())`
    /// とビット一致する（[`build_pattern_and_slots`] のドキュメント参照）。
    pub fn assemble_into(&mut self, triplets: &[Triplet]) -> &SparseColMat<usize, f64> {
        debug_assert_eq!(
            triplets.len(),
            self.orig_coords.len(),
            "triplet 列の長さがパターン構築時と異なる"
        );
        debug_assert!(
            triplets
                .iter()
                .zip(self.orig_coords.iter())
                .all(|(t, &(c, r))| t.col == c && t.row == r),
            "triplet の (row,col) 座標・並び順がパターン構築時と異なる"
        );
        let (_, vals) = self.mat.parts_mut();
        vals.fill(0.0);
        for (i, t) in triplets.iter().enumerate() {
            vals[self.slot_of[i]] += t.val;
        }
        &self.mat
    }

    /// 直近に組み立てた行列（`assemble_into` 未呼び出しなら `new` 直後の状態）。
    pub fn matrix(&self) -> &SparseColMat<usize, f64> {
        &self.mat
    }
}

/// 複数の CSC 行列（非ゼロパターンが反復間で不変）の重み付き和を安価に更新するための
/// キャッシュ。[`weighted_sum_csc`] を毎回呼ぶ代わりに使う。
///
/// 初回 [`WeightedSumCache::new`] で出力パターン（各入力行列の和の非ゼロ位置）と、
/// 「各入力行列の値配列内位置→出力の値スロット」の写像を構築する。以降の
/// [`WeightedSumCache::combine_into`] は各入力行列の値配列を走査して axpy
/// （`out += coef * val`）するだけで、triplet 化・ソートは行わない。
///
/// 減衰行列の組立て（C = a0·M + a1·K）のように、`M`・`K` の非ゼロパターンが
/// 解析を通じて不変な場面で使う。
///
/// # 使用例
/// ```
/// use squid_n_math::sparse::{assemble_csc, WeightedSumCache, Triplet};
///
/// let m = assemble_csc(2, vec![
///     Triplet { row: 0, col: 0, val: 1.0 },
///     Triplet { row: 1, col: 1, val: 1.0 },
/// ]);
/// let k = assemble_csc(2, vec![
///     Triplet { row: 0, col: 0, val: 100.0 },
///     Triplet { row: 1, col: 1, val: 100.0 },
/// ]);
/// let mut cache = WeightedSumCache::new(2, &[(2.0, &m), (0.05, &k)]);
/// let c = cache.combine_into(&[(2.0, &m), (0.05, &k)]);
/// assert_eq!(*c.get(0, 0).unwrap(), 7.0);
/// ```
///
/// # 前提（呼び出し側の責務）
/// 2 回目以降に渡す `mats` は、要素数・各行列の非ゼロパターン（各行列自身の
/// col_ptr/row_idx の並び）が初回構築時と完全に一致していること（係数・値のみ
/// 変わってよい）。`debug_assert` で非ゼロ数の一致を検証するが、パターンそのものの
/// 一致まではリリースビルドで検証されないため、呼び出し側が担保すること。
pub struct WeightedSumCache {
    /// フラット化した全入力（`mats` を順に、各行列の値配列の並び順）の
    /// 各要素が対応する、出力の値配列内スロット位置。
    slot_of: Vec<usize>,
    /// 各入力行列の非ゼロ数（`combine_into` での整合性チェック用）。
    input_nnz: Vec<usize>,
    mat: SparseColMat<usize, f64>,
}

impl WeightedSumCache {
    /// `mats: &[(coef, &SparseColMat)]` から [`weighted_sum_csc`] と同じ規則で
    /// 出力パターンを構築する。
    pub fn new(n: usize, mats: &[(f64, &SparseColMat<usize, f64>)]) -> Self {
        // sparse_to_triplets は各行列を列優先・列内は格納順（＝値配列の並び順）で
        // 走査するため、これをそのままフラット化すれば weighted_sum_csc と同じ
        // 入力順序の triplet 列になる。
        let mut triplets: Vec<Triplet> = Vec::new();
        let mut input_nnz = Vec::with_capacity(mats.len());
        for (coef, mat) in mats {
            let nnz = mat.val().len();
            input_nnz.push(nnz);
            for t in sparse_to_triplets(mat) {
                triplets.push(Triplet {
                    row: t.row,
                    col: t.col,
                    val: coef * t.val,
                });
            }
        }
        let (mat, slot_of) = build_pattern_and_slots(n, &triplets);
        Self {
            slot_of,
            input_nnz,
            mat,
        }
    }

    /// `mats` の値でパターン不変のまま重み付き和を再計算し、更新後の行列への参照を
    /// 返す。各行列の値配列を axpy するだけで、triplet 化・ソートは行わない。
    /// 結果は `weighted_sum_csc(n, mats)` とビット一致する。
    pub fn combine_into(
        &mut self,
        mats: &[(f64, &SparseColMat<usize, f64>)],
    ) -> &SparseColMat<usize, f64> {
        debug_assert_eq!(
            mats.len(),
            self.input_nnz.len(),
            "入力行列の個数がパターン構築時と異なる"
        );
        let (_, vals) = self.mat.parts_mut();
        vals.fill(0.0);
        let mut offset = 0;
        for (k, (coef, mat)) in mats.iter().enumerate() {
            let mv = mat.val();
            debug_assert_eq!(
                mv.len(),
                self.input_nnz[k],
                "行列 {k} の非ゼロ数がパターン構築時と異なる"
            );
            for (j, &v) in mv.iter().enumerate() {
                vals[self.slot_of[offset + j]] += coef * v;
            }
            offset += mv.len();
        }
        &self.mat
    }

    /// 直近に組み立てた行列（`combine_into` 未呼び出しなら `new` 直後の状態）。
    pub fn matrix(&self) -> &SparseColMat<usize, f64> {
        &self.mat
    }
}

/// 単一の CSC 行列をスカラ倍した新しい行列を返す（`mat` のパターンをそのまま複製し、
/// 値のみ `alpha` 倍する）。`weighted_sum_csc(n, &[(alpha, mat)])` と結果はビット一致
/// するが、triplet 化・ソートを経ないぶん軽量（`assemble_c_tangent` の α1·K_t のような
/// 単一行列のスカラ倍に使う）。
pub fn scale_csc(mat: &SparseColMat<usize, f64>, alpha: f64) -> SparseColMat<usize, f64> {
    let mut out = mat.clone();
    scale_csc_into(mat, alpha, &mut out);
    out
}

/// [`scale_csc`] のバッファ再利用版。`out` は事前に `mat` と同一パターン（同じ
/// 非ゼロ数）を持つこと（典型的には初回に `mat.clone()` などで用意し、以降使い回す）。
///
/// # Panics（debug のみ）
/// `out` の非ゼロ数が `mat` と異なる場合。
pub fn scale_csc_into(
    mat: &SparseColMat<usize, f64>,
    alpha: f64,
    out: &mut SparseColMat<usize, f64>,
) {
    let src_len = mat.val().len();
    let (_, dst_vals) = out.parts_mut();
    debug_assert_eq!(src_len, dst_vals.len(), "out のパターンが mat と一致しない");
    for (d, &s) in dst_vals.iter_mut().zip(mat.val().iter()) {
        *d = s * alpha;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_assemble_csc_deterministic() {
        let n = 2;
        let triplets_a = vec![
            Triplet {
                row: 0,
                col: 0,
                val: 100.0,
            },
            Triplet {
                row: 1,
                col: 0,
                val: -200.0,
            },
            Triplet {
                row: 0,
                col: 1,
                val: -200.0,
            },
            Triplet {
                row: 1,
                col: 1,
                val: 200.0,
            },
        ];
        let triplets_b = vec![
            Triplet {
                row: 1,
                col: 1,
                val: 200.0,
            },
            Triplet {
                row: 0,
                col: 1,
                val: -200.0,
            },
            Triplet {
                row: 1,
                col: 0,
                val: -200.0,
            },
            Triplet {
                row: 0,
                col: 0,
                val: 100.0,
            },
        ];
        let mat_a = assemble_csc(n, triplets_a);
        let mat_b = assemble_csc(n, triplets_b);
        for i in 0..n {
            for j in 0..n {
                assert_eq!(
                    *mat_a.get(i, j).unwrap_or(&0.0),
                    *mat_b.get(i, j).unwrap_or(&0.0)
                );
            }
        }
    }

    #[test]
    fn test_assemble_csc_merge() {
        let n = 1;
        let triplets = vec![
            Triplet {
                row: 0,
                col: 0,
                val: 10.0,
            },
            Triplet {
                row: 0,
                col: 0,
                val: 20.0,
            },
        ];
        let mat = assemble_csc(n, triplets);
        assert_eq!(*mat.get(0, 0).unwrap_or(&0.0), 30.0);
    }

    #[test]
    fn test_sparse_to_triplets_roundtrip() {
        let n = 3;
        let triplets = vec![
            Triplet {
                row: 0,
                col: 0,
                val: 1.0,
            },
            Triplet {
                row: 1,
                col: 0,
                val: 2.0,
            },
            Triplet {
                row: 1,
                col: 1,
                val: 3.0,
            },
            Triplet {
                row: 2,
                col: 2,
                val: 4.0,
            },
            Triplet {
                row: 0,
                col: 2,
                val: 5.0,
            },
        ];
        let mat = assemble_csc(n, triplets);
        let recovered = sparse_to_triplets(&mat);
        let rebuilt = assemble_csc(n, recovered);
        for i in 0..n {
            for j in 0..n {
                let a = *mat.get(i, j).unwrap_or(&0.0);
                let b = *rebuilt.get(i, j).unwrap_or(&0.0);
                assert_eq!(a, b, "mismatch at ({},{})", i, j);
            }
        }
    }

    #[test]
    fn test_weighted_sum_csc() {
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
        let k = assemble_csc(
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
        let c = weighted_sum_csc(n, &[(2.0, &m), (0.05, &k)]);
        assert!((*c.get(0, 0).unwrap_or(&0.0) - 7.0).abs() < 1e-12);
        assert!((*c.get(1, 1).unwrap_or(&0.0) - 7.0).abs() < 1e-12);
        assert!((*c.get(1, 0).unwrap_or(&0.0) - (-2.5)).abs() < 1e-12);
        assert!((*c.get(0, 1).unwrap_or(&0.0) - (-2.5)).abs() < 1e-12);
    }

    fn assert_dense_bit_eq(a: &SparseColMat<usize, f64>, b: &SparseColMat<usize, f64>, n: usize) {
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

    /// 桁落ちを起こす値（1e16 → 1.0 → -1e16 の順で加算すると結果が変わる）を
    /// 同一 (row,col) に複数回積んだ triplet 列。加算順序を誤ると検出できる。
    fn cancellation_prone_triplets() -> Vec<Triplet> {
        vec![
            Triplet {
                row: 0,
                col: 0,
                val: 1e16,
            },
            Triplet {
                row: 1,
                col: 1,
                val: 5.0,
            },
            Triplet {
                row: 0,
                col: 0,
                val: 1.0,
            },
            Triplet {
                row: 1,
                col: 0,
                val: -3.0,
            },
            Triplet {
                row: 0,
                col: 0,
                val: -1e16,
            },
            Triplet {
                row: 0,
                col: 1,
                val: 2.5,
            },
        ]
    }

    #[test]
    fn test_csc_assembler_matches_assemble_csc_bit_exact() {
        let n = 2;
        let triplets = cancellation_prone_triplets();

        let reference = assemble_csc(n, triplets.clone());
        let mut asm = CscAssembler::new(n, &triplets);
        let assembled = asm.assemble_into(&triplets);
        assert_dense_bit_eq(&reference, assembled, n);

        // 座標は同一のまま値だけを変えて再組立て（複数回加算パターンを保つ）。
        let triplets2: Vec<Triplet> = triplets
            .iter()
            .map(|t| Triplet {
                val: t.val * 3.0 + 1.0,
                ..*t
            })
            .collect();
        let reference2 = assemble_csc(n, triplets2.clone());
        let assembled2 = asm.assemble_into(&triplets2);
        assert_dense_bit_eq(&reference2, assembled2, n);
    }

    #[test]
    fn test_weighted_sum_cache_matches_weighted_sum_csc_bit_exact() {
        let n = 2;
        let m = assemble_csc(n, cancellation_prone_triplets());
        let k = assemble_csc(
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

        let reference = weighted_sum_csc(n, &[(2.0, &m), (0.05, &k)]);
        let mut cache = WeightedSumCache::new(n, &[(2.0, &m), (0.05, &k)]);
        let combined = cache.combine_into(&[(2.0, &m), (0.05, &k)]);
        assert_dense_bit_eq(&reference, combined, n);

        // 係数だけを変えて再計算（パターンは m・k とも不変）。
        let reference2 = weighted_sum_csc(n, &[(-1.5, &m), (7.0, &k)]);
        let combined2 = cache.combine_into(&[(-1.5, &m), (7.0, &k)]);
        assert_dense_bit_eq(&reference2, combined2, n);
    }

    #[test]
    fn test_scale_csc_matches_weighted_sum_csc_bit_exact() {
        let n = 2;
        let k = assemble_csc(
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
        let alpha = 0.618;
        let reference = weighted_sum_csc(n, &[(alpha, &k)]);
        let scaled = scale_csc(&k, alpha);
        assert_dense_bit_eq(&reference, &scaled, n);

        let mut out = k.clone();
        scale_csc_into(&k, alpha, &mut out);
        assert_dense_bit_eq(&reference, &out, n);
    }

    #[test]
    fn test_sparse_matvec_into_matches_sparse_matvec() {
        let n = 2;
        let k = assemble_csc(
            n,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 300.0,
                },
                Triplet {
                    row: 1,
                    col: 0,
                    val: -200.0,
                },
                Triplet {
                    row: 0,
                    col: 1,
                    val: -200.0,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: 200.0,
                },
            ],
        );
        let x = [1.5, -2.25];
        let expected = sparse_matvec(&k, &x);
        let mut out = vec![f64::NAN; n];
        sparse_matvec_into(&k, &x, &mut out);
        assert_eq!(expected, out);
    }
}
