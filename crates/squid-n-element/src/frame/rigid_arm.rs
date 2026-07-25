//! 剛域（材端の剛体アーム）の運動学変換。
//!
//! 部材端の剛域を扱うため、可撓長 \\( L' = L - \lambda_i - \lambda_j \\) で組んだ
//! 12×12 の剛性・内力を、可撓端自由度から節点自由度へ剛体アームで写す。
//!
//! ```text
//! u_flex = Tr · u_node,   K_node = Trᵀ · K_flex · Tr,   f_node = Trᵀ · f_flex
//! ```
//!
//! `Tr` は単位行列に、剛体アームによる並進-回転結合の非零成分を加えたものである。
//! 可撓端は i 端で節点から材軸内側へ +λi·ex、j 端で −λj·ex にあるため、
//! 運動学 `u_flex = u_node + θ_node × r`（r_i = (+λi,0,0), r_j = (−λj,0,0)）を
//! 成分展開すると次になる（回転・軸方向自由度は不変）。
//!
//! ```text
//! i 端: uy' = uy + λi·rz,  uz' = uz − λi·ry
//! j 端: uy' = uy − λj·rz,  uz' = uz + λj·ry
//! ```
//!
//! 弾性梁（[`crate::beam`]）とファイバー梁（[`crate::fiber`]）が同じ剛域の扱いを
//! 共有するため、変換をここへ一元化する。

use crate::behavior::LocalMat;

/// 剛域長が両端ともゼロ（変換が恒等）か。
pub fn is_identity(li: f64, lj: f64) -> bool {
    li.abs() < 1e-12 && lj.abs() < 1e-12
}

/// 節点自由度の変位から可撓端自由度の変位を得る（`u_flex = Tr · u_node`）。
pub fn to_flex_disp(u_node: &[f64; 12], li: f64, lj: f64) -> [f64; 12] {
    let mut u = *u_node;
    if is_identity(li, lj) {
        return u;
    }
    u[1] = u_node[1] + li * u_node[5];
    u[2] = u_node[2] - li * u_node[4];
    u[7] = u_node[7] - lj * u_node[11];
    u[8] = u_node[8] + lj * u_node[10];
    u
}

/// 可撓端自由度の内力を節点自由度の内力へ写す（`f_node = Trᵀ · f_flex`）。
/// 剛体アームのモーメント（アーム長 × 可撓端せん断）が節点回転自由度へ加わる。
pub fn to_node_force(f_flex: &[f64; 12], li: f64, lj: f64) -> [f64; 12] {
    let mut f = *f_flex;
    if is_identity(li, lj) {
        return f;
    }
    f[5] += li * f_flex[1];
    f[4] -= li * f_flex[2];
    f[11] -= lj * f_flex[7];
    f[10] += lj * f_flex[8];
    f
}

/// 可撓端自由度の剛性を節点自由度の剛性へ写す（`K_node = Trᵀ · K_flex · Tr`）。
pub fn transform_stiffness(k_flex: &LocalMat, li: f64, lj: f64) -> LocalMat {
    if is_identity(li, lj) {
        return LocalMat {
            n: k_flex.n,
            data: k_flex.data.clone(),
        };
    }
    // Tr（単位行列＋剛体アームの並進-回転結合項）。
    let mut tr = LocalMat::zeros(12);
    for i in 0..12 {
        tr.set(i, i, 1.0);
    }
    tr.set(1, 5, li);
    tr.set(2, 4, -li);
    tr.set(7, 11, -lj);
    tr.set(8, 10, lj);

    // K_node = Trᵀ · K_flex · Tr
    let mut tmp = LocalMat::zeros(12);
    for i in 0..12 {
        for j in 0..12 {
            let mut s = 0.0;
            for k in 0..12 {
                s += k_flex.get(i, k) * tr.get(k, j);
            }
            tmp.set(i, j, s);
        }
    }
    let mut kn = LocalMat::zeros(12);
    for i in 0..12 {
        for j in 0..12 {
            let mut s = 0.0;
            for k in 0..12 {
                s += tr.get(k, i) * tmp.get(k, j);
            }
            kn.set(i, j, s);
        }
    }
    kn
}

/// 節点自由度の材端力を可撓端（剛域フェイス）の材端力へ戻す
/// （[`to_node_force`] の逆写像 `f_flex = (Trᵀ)⁻¹ f_node`）。
///
/// 並進成分は剛体アームで変わらない。回転成分からは剛体アームのモーメント
/// （アーム長 × 材端せん断）を差し引くため、**剛域フェイスでの材端モーメント**が
/// 得られる。断面の降伏判定・設計用応力は危険断面＝剛域フェイスで評価するため、
/// 節点位置のモーメント（アーム分だけ大きい）ではなくこちらを用いる。
pub fn to_flex_force(f_node: &[f64; 12], li: f64, lj: f64) -> [f64; 12] {
    let mut f = *f_node;
    if is_identity(li, lj) {
        return f;
    }
    f[5] = f_node[5] - li * f_node[1];
    f[4] = f_node[4] + li * f_node[2];
    f[11] = f_node[11] + lj * f_node[7];
    f[10] = f_node[10] - lj * f_node[8];
    f
}

/// 剛域長 `li`・`lj` を可撓長が正になる範囲へ解決する。
///
/// 剛域長の合計が節点間長 `length` 以上になる病的な入力（自動算定が想定しない
/// 極端に短い部材など）では、可撓長がゼロ以下となり要素が構成できない。その場合は
/// 剛域なし `(0, 0)` として扱い、要素が退化しないようにする。負値は 0 に丸める。
pub fn resolve_lengths(li: f64, lj: f64, length: f64) -> (f64, f64) {
    let (li, lj) = (li.max(0.0), lj.max(0.0));
    if length > 0.0 && length - li - lj > 1e-9 * length {
        (li, lj)
    } else {
        (0.0, 0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 剛域長ゼロでは変換が恒等（変位・内力・剛性とも素通り）。
    #[test]
    fn 剛域長ゼロの変換は恒等() {
        let u: [f64; 12] = std::array::from_fn(|i| i as f64);
        assert_eq!(to_flex_disp(&u, 0.0, 0.0), u);
        assert_eq!(to_node_force(&u, 0.0, 0.0), u);
    }

    /// 剛体回転（節点 i まわり）では可撓端も剛体的に動く。
    /// θz まわりに回した場合、可撓端 i の uy は λi·θz になる。
    #[test]
    fn 剛体回転で可撓端は剛体的に動く() {
        let (li, lj, l) = (300.0, 200.0, 3000.0);
        let theta = 1.0e-4;
        // 節点 i を原点として θz 回転: uy_i=0, rz_i=θ, uy_j=l·θ, rz_j=θ
        let mut u = [0.0_f64; 12];
        u[5] = theta;
        u[7] = l * theta;
        u[11] = theta;
        let uf = to_flex_disp(&u, li, lj);
        // 可撓端 i は x=λi にあるので uy = λi·θ
        assert!((uf[1] - li * theta).abs() < 1e-12);
        // 可撓端 j は x=L−λj にあるので uy = (L−λj)·θ
        assert!((uf[7] - (l - lj) * theta).abs() < 1e-12);
        // 可撓長ぶんの相対たわみは生じない（剛体回転）
        let chord = (uf[7] - uf[1]) / (l - li - lj);
        assert!((chord - theta).abs() < 1e-12);
    }

    /// 材端力の可撓端 ⇄ 節点の写像が互いに逆になる。
    #[test]
    fn 材端力の可撓端変換と節点変換は互いに逆() {
        let (li, lj) = (300.0, 200.0);
        let f_flex: [f64; 12] = std::array::from_fn(|i| (i as f64 + 1.0) * 1.5);
        let f_node = to_node_force(&f_flex, li, lj);
        let back = to_flex_force(&f_node, li, lj);
        for i in 0..12 {
            assert!((back[i] - f_flex[i]).abs() < 1e-9, "dof {i}");
        }
    }

    /// 剛性変換は対称性を保ち、剛体アームのモーメント項を回転自由度へ加える。
    #[test]
    fn 剛性変換は対称性を保つ() {
        let (li, lj) = (300.0, 200.0);
        // 対称な適当行列
        let mut k = LocalMat::zeros(12);
        for i in 0..12 {
            for j in 0..12 {
                k.set(i, j, ((i * 12 + j) % 7) as f64 + ((j * 12 + i) % 7) as f64);
            }
        }
        let kn = transform_stiffness(&k, li, lj);
        for i in 0..12 {
            for j in 0..12 {
                assert!((kn.get(i, j) - kn.get(j, i)).abs() < 1e-9);
            }
        }
    }
}
