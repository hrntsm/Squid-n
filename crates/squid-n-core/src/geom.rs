//! モデル幾何の共通判定。
//!
//! 複数のクレート（荷重集計・通り芯生成・GUI の集計表示）が同じ判定規則を
//! 必要とするものだけを置く。3 次元ベクトルの基本演算（内積・外積・単位化）は
//! [`vec3`] に分ける。

pub mod vec3;

use vec3::{cross, unit};

/// 鉛直材（柱）とみなす両端の水平距離の上限 [mm]。
pub const VERTICAL_TOL_MM: f64 = 1.0;

/// 節点ペアが鉛直材（柱）かどうか。両端の水平距離（XY 平面）が
/// [`VERTICAL_TOL_MM`] 未満なら鉛直とみなす。
///
/// 仕上げ周長式・雑壁の柱探索・柱脚梁せい付加・通り芯の自動生成が
/// 共通で用いる（判定規則の情報源を 1 つに保つ）。
pub fn is_vertical_pair(a: [f64; 3], b: [f64; 3]) -> bool {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt() < VERTICAL_TOL_MM
}

/// 部材軸を鉛直（柱系）とみなす方向余弦 |ez| の下限（45° 基準）。
pub const VERTICAL_COS_TOL: f64 = 0.707;

/// 部材軸（両端座標）が鉛直（柱系）か。部材軸単位ベクトルの z 成分
/// |ez| > [`VERTICAL_COS_TOL`]（水平から 45° 超）で判定する。長さ 0 は偽。
///
/// 力学レジーム選択・プッシュオーバーの変形角定義・層せん断集計・
/// 偏心率/剛性率・ST-Bridge 入出力が共通で用いる**単一規約**。
/// 設計検定の部材種別（柱 |ez|≥0.8／梁 ≤0.2／中間ブレースの 3 区分）は
/// 検定式の選択という別目的の規約であり、本判定へは統合しない。
/// [`is_vertical_pair`]（水平距離の実寸トレランス）は「厳密に直立した柱」の
/// 抽出用でこれも別物。
pub fn is_vertical_axis(a: [f64; 3], b: [f64; 3]) -> bool {
    vec3::unit_from(a, b).is_some_and(|d| axis_dominates(d, 2))
}

/// 部材軸が座標軸方向へ卓越するとみなす方向余弦の下限（45° 基準）。
/// 鉛直（Z 軸）に適用したものが [`VERTICAL_COS_TOL`] である。
pub const AXIS_COS_TOL: f64 = VERTICAL_COS_TOL;

/// 単位方向ベクトル `dir` が座標軸 `axis`（0=X, 1=Y, 2=Z）の方向へ卓越するか。
/// |方向余弦| > [`AXIS_COS_TOL`]（その軸から 45° 未満）で判定する。
///
/// 「X 方向に効く梁」「加力直交方向へ卓越する部材」のように、鉛直に限らない
/// 軸方向の卓越判定はすべてこの規約に従う（判定の情報源を 1 つに保つ）。
pub fn axis_dominates(dir: [f64; 3], axis: usize) -> bool {
    dir.get(axis).is_some_and(|c| c.abs() > AXIS_COS_TOL)
}

/// 2 本の部材が概ね直交すると扱う軸内積（方向余弦の内積）の上限。
///
/// [`VERTICAL_COS_TOL`] と同じ 45° 基準だが、こちらは**部材どうしの相対角**に
/// 対する規約（柱フェース距離の直交材探索・剛域算定が用いる）。
pub const ORTHOGONAL_DOT_MAX: f64 = 0.707;

/// 部材の単位軸ベクトル（始端節点 → 終端節点）。
///
/// 節点が 2 つ未満・節点参照が範囲外・長さが縮退している部材は零ベクトルを
/// 返す（呼び出し側で内積を取ると 0 になり、直交材としても平行材としても
/// 拾われない）。線材（`ElementKind::Beam`）に用いることを想定し、終端は
/// `nodes` の**末尾**を採る。
pub fn element_axis(model: &crate::model::Model, e: &crate::model::ElementData) -> [f64; 3] {
    if e.nodes.len() < 2 {
        return [0.0, 0.0, 0.0];
    }
    let (Some(n0), Some(n1)) = (
        model.nodes.get(e.nodes[0].index()),
        model.nodes.get(e.nodes[e.nodes.len() - 1].index()),
    ) else {
        return [0.0, 0.0, 0.0];
    };
    vec3::unit_from(n0.coord, n1.coord).unwrap_or([0.0, 0.0, 0.0])
}

/// 平面多角形（3D 座標、頂点が同一平面上と仮定）の面積 \[mm²\]。
///
/// Newell の公式 `N = 1/2 Σ(Vi × Vi+1)`, `Area = |N|` による。凸・非凸いずれも、
/// 頂点が境界を一周する順序で与えられていれば成立する。頂点が 3 個未満なら 0。
///
/// 壁・シェル要素の自重算定とスラブ・壁の数量拾いが共通で用いる。
pub fn polygon_area_3d(pts: &[[f64; 3]]) -> f64 {
    if pts.len() < 3 {
        return 0.0;
    }
    let n = pts.len();
    let mut normal = [0.0_f64; 3];
    for i in 0..n {
        let (p0, p1) = (pts[i], pts[(i + 1) % n]);
        let c = vec3::cross(p0, p1);
        normal = vec3::add(normal, c);
    }
    0.5 * vec3::norm(normal)
}

/// 点群に最も当てはまる平面の単位法線を、主成分分析（全最小二乗）で求める。
///
/// 重心まわりの共分散行列の**最小固有値の固有ベクトル**を法線に採る。通常の
/// 線形回帰（\\(z = ax + by + c\\) の最小二乗）は 1 つの \\((x,y)\\) に 1 つの
/// \\(z\\) しか対応づけられず**鉛直面を表せない**ため使わない（構面はほぼ常に
/// 鉛直面であり、同じ平面位置に何本も柱が立つ）。
///
/// 点が一直線に並ぶ場合は平面が一意に定まらないため、**その直線と鉛直軸を含む
/// 平面**の法線を返す（構面を素直に鉛直と読む）。直線自体が鉛直な場合は
/// さらに定まらないため、X 方向を法線とする（YZ 平面の構面と読む）。
///
/// 点が 3 個未満、またはすべて同一点の場合は `None`。
pub fn best_fit_plane_normal(pts: &[[f64; 3]]) -> Option<[f64; 3]> {
    if pts.len() < 3 {
        return None;
    }
    let n = pts.len() as f64;
    let c = [
        pts.iter().map(|p| p[0]).sum::<f64>() / n,
        pts.iter().map(|p| p[1]).sum::<f64>() / n,
        pts.iter().map(|p| p[2]).sum::<f64>() / n,
    ];
    let mut cov = [[0.0f64; 3]; 3];
    for p in pts {
        let d = [p[0] - c[0], p[1] - c[1], p[2] - c[2]];
        for (i, di) in d.iter().enumerate() {
            for (j, dj) in d.iter().enumerate() {
                cov[i][j] += di * dj;
            }
        }
    }
    let (vals, vecs) = jacobi_eigen_sym3(cov);
    // 固有値の降順で並べ替えたときの添字（λ0 ≧ λ1 ≧ λ2）。
    let mut order = [0usize, 1, 2];
    order.sort_by(|&a, &b| vals[b].total_cmp(&vals[a]));
    let (l0, l1) = (vals[order[0]], vals[order[1]]);
    if l0 <= 1e-12 {
        // すべて同一点。
        return None;
    }
    // λ1 が λ0 に対して無視できるほど小さい ＝ 点が一直線に並ぶ（平面が定まらない）。
    if l1 <= 1e-9 * l0 {
        let line = [vecs[0][order[0]], vecs[1][order[0]], vecs[2][order[0]]];
        // その直線と鉛直軸を含む平面の法線 ＝ 直線 × Z。
        return unit(cross(line, [0.0, 0.0, 1.0])).or(Some([1.0, 0.0, 0.0]));
    }
    let normal = [vecs[0][order[2]], vecs[1][order[2]], vecs[2][order[2]]];
    unit(normal)
}

/// 3×3 実対称行列の固有値・固有ベクトルを巡回 Jacobi 法で求める。
///
/// 返り値は `(固有値, 固有ベクトル行列)`。固有ベクトルは**列**に入る
/// （`vecs[行][k]` が第 k 固有ベクトルの成分）。3×3 の一度きりの計算のため、
/// 反復回数を固定した素朴な実装で十分な精度が出る。
fn jacobi_eigen_sym3(mut a: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..24 {
        // 非対角の最大成分を消す回転を掛ける。
        let (mut p, mut q, mut max) = (0usize, 1usize, 0.0f64);
        for (i, j) in [(0usize, 1usize), (0, 2), (1, 2)] {
            if a[i][j].abs() > max {
                max = a[i][j].abs();
                p = i;
                q = j;
            }
        }
        if max < 1e-14 {
            break;
        }
        let theta = 0.5 * (2.0 * a[p][q]).atan2(a[p][p] - a[q][q]);
        let (s, c) = theta.sin_cos();
        let mut b = a;
        for k in 0..3 {
            b[p][k] = c * a[p][k] + s * a[q][k];
            b[q][k] = -s * a[p][k] + c * a[q][k];
        }
        let mut d = b;
        for k in 0..3 {
            d[k][p] = c * b[k][p] + s * b[k][q];
            d[k][q] = -s * b[k][p] + c * b[k][q];
        }
        a = d;
        let mut nv = v;
        for k in 0..3 {
            nv[k][p] = c * v[k][p] + s * v[k][q];
            nv[k][q] = -s * v[k][p] + c * v[k][q];
        }
        v = nv;
    }
    ([a[0][0], a[1][1], a[2][2]], v)
}

#[cfg(test)]
mod tests;
