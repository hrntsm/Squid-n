//! モデル幾何の共通判定。
//!
//! 複数のクレート（荷重集計・通り芯生成・GUI の集計表示）が同じ判定規則を
//! 必要とするものだけを置く。

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
    let (dx, dy, dz) = (b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    len > 1e-12 && (dz / len).abs() > VERTICAL_COS_TOL
}

/// ベクトルを正規化する（長さが 0 に近ければ `None`）。
fn normalize(v: [f64; 3]) -> Option<[f64; 3]> {
    let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    (n > 1e-12).then(|| [v[0] / n, v[1] / n, v[2] / n])
}

/// 外積。
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
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
        return normalize(cross(line, [0.0, 0.0, 1.0])).or(Some([1.0, 0.0, 0.0]));
    }
    let normal = [vecs[0][order[2]], vecs[1][order[2]], vecs[2][order[2]]];
    normalize(normal)
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
