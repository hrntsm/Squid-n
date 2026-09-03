//! 直線材（等断面の線材）が共有する定式化。
//!
//! 弾性梁（[`super::beam`]）・ファイバー梁（[`super::fiber`]）・材端集中ばね梁
//! （[`super::concentrated`]）は、要素の内部構成こそ違うが、次の 3 つについては
//! 同一の式を用いる。要素ごとに書き分ける理由がなく、片方だけを直すと解析結果が
//! 静かに食い違うため、ここへ集約する。
//!
//! - [`condense_end_releases`] — 材端解放（ピン・半剛・ねじれ解放）の静縮約
//! - [`geometric_stiffness`] —  P-δ の幾何剛性
//! - [`lumped_mass`] / [`consistent_mass`] — 質量行列
//!
//! いずれも要素局所系（節点自由度 12）で返す。全体系への回転（`axis.to_global`）は
//! 要素ごとに座標系が異なるため呼び出し側の責務とする。

use crate::behavior::LocalMat;
use crate::linalg::invert_small;

/// 材端解放を静縮約した局所剛性 12×12。
///
/// `releases` は解放する要素端回転自由度と、節点回転との間に挟む回転ばね剛性
/// `k_s` [N·mm/rad] の組。ピンは `k_s = 0`（厳密なモーメント解放）、半剛は
/// 接合部回転剛性を与える。空なら剛接として `k_elem` をそのまま返す
/// （ペナルティ近似を用いない厳密な扱い）。
///
/// 内部並びは \[外部 0..11（節点 ux,uy,uz,rx,ry,rz ×2）, 内部 12..（解放した
/// 要素端回転を `releases` の順に並べる）\] とし、内部自由度を静縮約する
/// （K\* = Kaa − Kab·Kbb⁻¹·Kba）。
///
/// 解放できるのは両端の回転 3 成分ずつであるため `releases` は最大 6 個、
/// 系の大きさは n ≤ 18 に収まる。作業配列をスタックに置くのはこのため。
///
/// 縮約行列 Kbb が特異（解放自由度が機構化）な場合は、もっともらしい剛性を作らず
/// 補正項を省略した Kaa を返す。解放回転の外部行はばね剛性を除きゼロのため、
/// 全体求解が特異を検出して自由度名指しの診断で停止する。
pub(crate) fn condense_end_releases(k_elem: &LocalMat, releases: &[(usize, f64)]) -> LocalMat {
    const NA: usize = 12;

    if releases.is_empty() {
        return LocalMat {
            n: NA,
            data: k_elem.data.clone(),
        };
    }

    let nb = releases.len();
    let n = NA + nb;
    debug_assert!(
        n <= 18,
        "condense_end_releases: 解放自由度は両端の回転 6 まで"
    );
    // n ≤ 18 なので n*n ≤ 324。反復のたびのヒープ確保を避けて固定長配列で扱う。
    let mut k = [0.0_f64; 324];

    // 要素 DOF → 組立 DOF 写像。解放回転は内部（12..）へ、それ以外は同位置。
    let mut map = [0usize; NA];
    for (i, m) in map.iter_mut().enumerate() {
        *m = i;
    }
    for (idx, &(r, _)) in releases.iter().enumerate() {
        map[r] = NA + idx;
    }

    // 要素剛性を配置（解放回転の行・列は内部自由度へ移る）
    for i in 0..NA {
        for j in 0..NA {
            k[map[i] * n + map[j]] += k_elem.get(i, j);
        }
    }

    // 回転ばね: 節点回転 r ↔ 内部の要素端回転 (12+idx)
    for (idx, &(r, ks)) in releases.iter().enumerate() {
        let ir = NA + idx;
        k[r * n + r] += ks;
        k[ir * n + ir] += ks;
        k[r * n + ir] -= ks;
        k[ir * n + r] -= ks;
    }

    // 内部 DOF (12..n) を静縮約: K* = Kaa − Kab·Kbb⁻¹·Kba
    let mut kaa = [0.0_f64; NA * NA];
    let mut kab = [0.0_f64; NA * 6];
    let mut kba = [0.0_f64; 6 * NA];
    let mut kbb = [0.0_f64; 36];

    for i in 0..NA {
        for j in 0..NA {
            kaa[i * NA + j] = k[i * n + j];
        }
        for j in 0..nb {
            kab[i * nb + j] = k[i * n + (NA + j)];
            kba[j * NA + i] = k[(NA + j) * n + i];
        }
    }
    for i in 0..nb {
        for j in 0..nb {
            kbb[i * nb + j] = k[(NA + i) * n + (NA + j)];
        }
    }

    let Some(kbb_inv) = invert_small(&kbb[..nb * nb], nb) else {
        let mut kstar = LocalMat::zeros(NA);
        for i in 0..NA {
            for j in 0..NA {
                kstar.set(i, j, kaa[i * NA + j]);
            }
        }
        return kstar;
    };

    // kab_kbbinv = Kab · Kbb⁻¹
    let mut kab_kbbinv = [0.0_f64; NA * 6];
    for i in 0..NA {
        for j in 0..nb {
            let mut s = 0.0;
            for l in 0..nb {
                s += kab[i * nb + l] * kbb_inv[l * nb + j];
            }
            kab_kbbinv[i * nb + j] = s;
        }
    }

    let mut kstar = LocalMat::zeros(NA);
    for i in 0..NA {
        for j in 0..NA {
            let mut s = kaa[i * NA + j];
            for l in 0..nb {
                s -= kab_kbbinv[i * nb + l] * kba[l * NA + j];
            }
            kstar.set(i, j, s);
        }
    }
    kstar
}

/// 軸力 `n_axial` による幾何剛性（P-δ）を、剛体アーム変換まで済ませた
/// 局所 12×12 で返す。
///
/// 弾性剛性と整合させるため、可撓長 `flex_len`（= L − λi − λj）で組み、
/// 可撓端自由度を剛体アームで節点自由度へ写す。剛域があれば P-δ は可撓部でのみ
/// 生じ、剛域は剛体アームとして働く。剛域なし（λi = λj = 0）では全長の
/// 幾何剛性に一致する。可撓長が実質ゼロの部材はゼロ行列を返す。
///
/// 局所系の規約（§4.1）により、xz 面（uz・ry）では並進-回転の結合項の符号が
/// xy 面（uy・rz）と逆になる。
pub(crate) fn geometric_stiffness(
    n_axial: f64,
    flex_len: f64,
    rigid_i: f64,
    rigid_j: f64,
) -> LocalMat {
    let l = flex_len;
    if l < 1e-12 {
        return LocalMat::zeros(12);
    }
    let c = n_axial / l;
    let mut kg = LocalMat::zeros(12);
    {
        let mut s = |i: usize, j: usize, v: f64| {
            kg.set(i, j, v);
            if i != j {
                kg.set(j, i, v);
            }
        };
        // xy 面（uy=1, rz=5 / uy_j=7, rz_j=11）
        s(1, 1, c * 6.0 / 5.0);
        s(7, 7, c * 6.0 / 5.0);
        s(1, 7, -c * 6.0 / 5.0);
        s(1, 5, c * l / 10.0);
        s(1, 11, c * l / 10.0);
        s(5, 7, -c * l / 10.0);
        s(7, 11, -c * l / 10.0);
        s(5, 5, c * 2.0 * l * l / 15.0);
        s(11, 11, c * 2.0 * l * l / 15.0);
        s(5, 11, -c * l * l / 30.0);
        // xz 面（uz=2, ry=4 / uz_j=8, ry_j=10）
        s(2, 2, c * 6.0 / 5.0);
        s(8, 8, c * 6.0 / 5.0);
        s(2, 8, -c * 6.0 / 5.0);
        s(2, 4, -c * l / 10.0);
        s(2, 10, -c * l / 10.0);
        s(4, 8, c * l / 10.0);
        s(8, 10, c * l / 10.0);
        s(4, 4, c * 2.0 * l * l / 15.0);
        s(10, 10, c * 2.0 * l * l / 15.0);
        s(4, 10, -c * l * l / 30.0);
    }
    crate::frame::rigid_arm::transform_stiffness(&kg, rigid_i, rigid_j)
}

/// 集中質量（対角）の局所 12×12。並進 3 成分へ `mass/2` ずつ配る。
///
/// 並進 3 成分が等しい対角行列は回転不変（Rᵀ·(m/2)I·R = (m/2)I）のため、
/// 呼び出し側で全体系へ回す必要はない。
pub(crate) fn lumped_mass(mass: f64) -> LocalMat {
    let mut mm = LocalMat::zeros(12);
    for d in [0, 1, 2, 6, 7, 8] {
        mm.set(d, d, mass / 2.0);
    }
    mm
}

/// 整合質量（Hermite 梁の一貫質量）の局所 12×12。
///
/// `torsion_term` は部材軸まわりの回転慣性 ρ·J·l/6 [t·mm²]。0 を渡すと
/// ねじれ自由度（rx）の質量を持たない行列になる。
///
/// DOF は連続ではないためインデックス配列で指定する。
///   Uy-Rz 面: \[Uy_i=1, Rz_i=5, Uy_j=7, Rz_j=11\]
///   Uz-Ry 面: \[Uz_i=2, Ry_i=4, Uz_j=8, Ry_j=10\]（回転符号は逆）
///
/// 整合質量は軸方向（m/3 系）と曲げ方向（156m/420 系）で係数が異なり回転不変では
/// ないため、呼び出し側で要素局所系から全体系へ回すこと（M_global = Rᵀ M_local R）。
/// これを欠くと鉛直柱・斜材で質量が誤った全体軸へ配分される。
pub(crate) fn consistent_mass(mass: f64, l: f64, torsion_term: f64) -> LocalMat {
    let mut mm = LocalMat::zeros(12);
    let c1 = mass / 6.0;
    let c2 = mass / 420.0;
    let l2 = l * l;
    // 軸（Ux): index 0, 6
    mm.set(0, 0, 2.0 * c1);
    mm.set(0, 6, 1.0 * c1);
    mm.set(6, 0, 1.0 * c1);
    mm.set(6, 6, 2.0 * c1);
    // ねじれ（Rx): index 3, 9
    let ct = torsion_term;
    mm.set(3, 3, 2.0 * ct);
    mm.set(3, 9, 1.0 * ct);
    mm.set(9, 3, 1.0 * ct);
    mm.set(9, 9, 2.0 * ct);
    // 曲げ: Hermite 梁の一貫質量（4×4 ブロック）。
    let b4 = |mm: &mut LocalMat, idx: [usize; 4], sign: f64| {
        let [d0, r0, d1, r1] = idx;
        // 並進-並進
        mm.set(d0, d0, 156.0 * c2);
        mm.set(d0, d1, 54.0 * c2);
        mm.set(d1, d0, 54.0 * c2);
        mm.set(d1, d1, 156.0 * c2);
        // 並進-回転
        mm.set(d0, r0, 22.0 * l * c2 * sign);
        mm.set(r0, d0, 22.0 * l * c2 * sign);
        mm.set(d0, r1, -13.0 * l * c2 * sign);
        mm.set(r1, d0, -13.0 * l * c2 * sign);
        mm.set(d1, r0, 13.0 * l * c2 * sign);
        mm.set(r0, d1, 13.0 * l * c2 * sign);
        mm.set(d1, r1, -22.0 * l * c2 * sign);
        mm.set(r1, d1, -22.0 * l * c2 * sign);
        // 回転-回転
        mm.set(r0, r0, 4.0 * l2 * c2);
        mm.set(r0, r1, -3.0 * l2 * c2);
        mm.set(r1, r0, -3.0 * l2 * c2);
        mm.set(r1, r1, 4.0 * l2 * c2);
    };
    b4(&mut mm, [1, 5, 7, 11], 1.0);
    b4(&mut mm, [2, 4, 8, 10], -1.0);
    mm
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 対称な要素剛性を与えたとき、縮約結果も対称であること。
    /// K* = Kaa − Kab·Kbb⁻¹·Kba は Kbb が対称なら対称になる。
    #[test]
    fn 縮約後の剛性は対称() {
        let k = sample_k();
        let kstar = condense_end_releases(&k, &[(5, 0.0), (11, 1.0e7)]);
        for i in 0..12 {
            for j in 0..12 {
                let (a, b) = (kstar.get(i, j), kstar.get(j, i));
                assert!(
                    (a - b).abs() < 1e-9 * (1.0 + a.abs()),
                    "K*[{i}][{j}] 非対称"
                );
            }
        }
    }

    /// ばね剛性 0（ピン）で解放した自由度は、縮約後に行・列とも 0 になる。
    /// これが「厳密なモーメント解放」の実体で、ペナルティ近似ではない。
    #[test]
    fn ピン解放した自由度の行と列は零になる() {
        let k = sample_k();
        let kstar = condense_end_releases(&k, &[(5, 0.0)]);
        for i in 0..12 {
            assert!(kstar.get(5, i).abs() < 1e-6, "行 5 の {i} 列が非零");
            assert!(kstar.get(i, 5).abs() < 1e-6, "列 5 の {i} 行が非零");
        }
    }

    /// 解放が空なら要素剛性をそのまま返す（剛接は厳密に扱う）。
    #[test]
    fn 解放が空なら要素剛性をそのまま返す() {
        let k = sample_k();
        let kstar = condense_end_releases(&k, &[]);
        assert_eq!(kstar.n, 12);
        assert_eq!(kstar.data, k.data);
    }

    /// 縮約は「内部自由度を直接解いた結果」と一致する。
    /// 外部自由度に単位変位 e を与え、内部自由度を Kbb·ub = −Kba·ua で解いて
    /// 得た外部反力 Kaa·ua + Kab·ub が、K*·ua と一致することを確かめる。
    #[test]
    fn 縮約は内部自由度を直接解いた結果と一致する() {
        let k = sample_k();
        let releases = [(5usize, 0.0_f64), (11usize, 2.0e7_f64)];
        let kstar = condense_end_releases(&k, &releases);

        // 縮約前の 14×14 を同じ規則で組み直す。
        let (na, nb) = (12usize, releases.len());
        let n = na + nb;
        let mut full = vec![0.0; n * n];
        let mut map: Vec<usize> = (0..12).collect();
        for (idx, &(r, _)) in releases.iter().enumerate() {
            map[r] = na + idx;
        }
        for i in 0..12 {
            for j in 0..12 {
                full[map[i] * n + map[j]] += k.get(i, j);
            }
        }
        for (idx, &(r, ks)) in releases.iter().enumerate() {
            let ir = na + idx;
            full[r * n + r] += ks;
            full[ir * n + ir] += ks;
            full[r * n + ir] -= ks;
            full[ir * n + r] -= ks;
        }

        for col in 0..na {
            // ua = e_col、ub は Kbb·ub = −Kba·ua を 2×2 で直接解く。
            let kbb = [
                full[(na) * n + na],
                full[(na) * n + na + 1],
                full[(na + 1) * n + na],
                full[(na + 1) * n + na + 1],
            ];
            let rhs = [-full[(na) * n + col], -full[(na + 1) * n + col]];
            let det = kbb[0] * kbb[3] - kbb[1] * kbb[2];
            assert!(det.abs() > 1e-30, "テスト用 Kbb が特異");
            let ub = [
                (rhs[0] * kbb[3] - kbb[1] * rhs[1]) / det,
                (kbb[0] * rhs[1] - rhs[0] * kbb[2]) / det,
            ];
            for row in 0..na {
                let direct = full[row * n + col]
                    + full[row * n + na] * ub[0]
                    + full[row * n + na + 1] * ub[1];
                let condensed = kstar.get(row, col);
                let tol = 1e-6 * (1.0 + direct.abs());
                assert!(
                    (direct - condensed).abs() < tol,
                    "K*[{row}][{col}] = {condensed}, 直接解 = {direct}"
                );
            }
        }
    }

    /// 軸力ゼロなら幾何剛性はゼロ行列。可撓長が実質ゼロでもゼロ行列。
    #[test]
    fn 幾何剛性は軸力零と可撓長零で零行列() {
        let kg = geometric_stiffness(0.0, 3000.0, 0.0, 0.0);
        assert!(kg.data.iter().all(|v| *v == 0.0), "軸力零で非零");
        let kg = geometric_stiffness(1.0e5, 0.0, 0.0, 0.0);
        assert!(kg.data.iter().all(|v| *v == 0.0), "可撓長零で非零");
    }

    /// 幾何剛性は剛体並進に対して力を生じない（列和が零）。
    /// 剛域なしの場合に、横方向の剛体並進 uy（DOF 1・7）で確認する。
    #[test]
    fn 幾何剛性は剛体並進で力を生じない() {
        let kg = geometric_stiffness(1.0e5, 3000.0, 0.0, 0.0);
        for row in 0..12 {
            let f = kg.get(row, 1) + kg.get(row, 7);
            assert!(f.abs() < 1e-9, "uy 剛体並進で行 {row} に力 {f}");
        }
    }

    /// 集中質量・整合質量とも、並進自由度の総和が部材質量に一致する
    /// （質量の保存。整合質量は Hermite 形状関数の重み 156+54+54+156 = 420 による）。
    #[test]
    fn 質量行列の並進成分の総和は部材質量に等しい() {
        const M: f64 = 12.5;
        let lumped = lumped_mass(M);
        let sum: f64 = [0usize, 6].iter().map(|&d| lumped.get(d, d)).sum();
        assert!((sum - M).abs() < 1e-12, "集中質量の軸方向総和 {sum}");

        // 曲げ面の並進自由度 Uy_i=1・Uy_j=7 の 4 成分（156+54+54+156 = 420）。
        let cm = consistent_mass(M, 3000.0, 0.0);
        let mut total = 0.0;
        for i in [1usize, 7] {
            for j in [1usize, 7] {
                total += cm.get(i, j);
            }
        }
        assert!((total - M).abs() < 1e-9, "整合質量の並進総和 {total}");
    }

    /// `torsion_term` を 0 にすると、ねじれ自由度の質量が消える。
    /// ファイバー梁は 0 を渡してねじれ質量を持たない（弾性梁は ρ·J·l/6 を渡す）。
    #[test]
    fn ねじれ項零なら回転自由度の質量は零() {
        let cm = consistent_mass(12.5, 3000.0, 0.0);
        for d in [3usize, 9] {
            assert_eq!(cm.get(d, d), 0.0, "DOF {d} にねじれ質量が残る");
        }
        let cm = consistent_mass(12.5, 3000.0, 2.0);
        assert_eq!(cm.get(3, 3), 4.0);
        assert_eq!(cm.get(3, 9), 2.0);
    }

    /// 試験用の対称な要素剛性。単純梁の弾性剛性に近い値を素朴に置く
    /// （縮約の性質を確かめるためのもので、実際の断面性能ではない）。
    fn sample_k() -> LocalMat {
        let mut k = LocalMat::zeros(12);
        let (ea_l, ei, l) = (2.0e6_f64, 1.0e12_f64, 3000.0_f64);
        k.set(0, 0, ea_l);
        k.set(6, 6, ea_l);
        k.set(0, 6, -ea_l);
        k.set(6, 0, -ea_l);
        k.set(3, 3, 5.0e9);
        k.set(9, 9, 5.0e9);
        k.set(3, 9, -5.0e9);
        k.set(9, 3, -5.0e9);
        for (d0, r0, d1, r1, sign) in [
            (1usize, 5usize, 7usize, 11usize, 1.0_f64),
            (2, 4, 8, 10, -1.0),
        ] {
            let (c1, c2, c3) = (12.0 * ei / (l * l * l), 6.0 * ei / (l * l), 4.0 * ei / l);
            k.set(d0, d0, c1);
            k.set(d1, d1, c1);
            k.set(d0, d1, -c1);
            k.set(d1, d0, -c1);
            k.set(d0, r0, c2 * sign);
            k.set(r0, d0, c2 * sign);
            k.set(d0, r1, c2 * sign);
            k.set(r1, d0, c2 * sign);
            k.set(d1, r0, -c2 * sign);
            k.set(r0, d1, -c2 * sign);
            k.set(d1, r1, -c2 * sign);
            k.set(r1, d1, -c2 * sign);
            k.set(r0, r0, c3);
            k.set(r1, r1, c3);
            k.set(r0, r1, c3 / 2.0);
            k.set(r1, r0, c3 / 2.0);
        }
        k
    }
}
