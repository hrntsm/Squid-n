//! 全塑性応力分布による断面力の積分プリミティブ。

use super::types::PlasticFiber;

/// 全塑性応力分布による断面力（支持点）。
pub fn plastic_point(fibers: &[PlasticFiber], e0: f64, ky: f64, kz: f64) -> [f64; 3] {
    let mut n = 0.0;
    let mut my = 0.0;
    let mut mz = 0.0;
    for f in fibers {
        let eps = e0 - kz * f.y + ky * f.z;
        let sigma = if eps > 0.0 {
            f.sigma_t
        } else if eps < 0.0 {
            f.sigma_c
        } else {
            0.0
        };
        let sa = sigma * f.area;
        n += sa;
        my += sa * f.z;
        mz += -sa * f.y;
    }
    [n, my, mz]
}

/// 軸耐力 (圧縮 Nc ≤ 0, 引張 Nt ≥ 0)。
pub fn axial_capacity(fibers: &[PlasticFiber]) -> (f64, f64) {
    let nc: f64 = fibers.iter().map(|f| f.sigma_c * f.area).sum();
    let nt: f64 = fibers.iter().map(|f| f.sigma_t * f.area).sum();
    (nc, nt)
}

/// 曲げ方向 (ky, kz) を固定し、軸力が `n_target` となる全塑性モーメントを返す。
/// `n_target` が軸耐力範囲外なら `None`。
pub fn plastic_moment_at_n(
    fibers: &[PlasticFiber],
    ky: f64,
    kz: f64,
    n_target: f64,
) -> Option<[f64; 2]> {
    let (nc, nt) = axial_capacity(fibers);
    if n_target < nc || n_target > nt || fibers.is_empty() {
        return None;
    }

    let mut order: Vec<usize> = (0..fibers.len()).collect();
    order.sort_by(|&a, &b| {
        let da = ky * fibers[a].z - kz * fibers[a].y;
        let db = ky * fibers[b].z - kz * fibers[b].y;
        db.partial_cmp(&da).unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut n = nc;
    let mut my: f64 = fibers.iter().map(|f| f.sigma_c * f.area * f.z).sum();
    let mut mz: f64 = fibers.iter().map(|f| -f.sigma_c * f.area * f.y).sum();

    for &i in &order {
        let f = &fibers[i];
        let dn = (f.sigma_t - f.sigma_c) * f.area;
        if n + dn >= n_target {
            let t = if dn > 0.0 { (n_target - n) / dn } else { 0.0 };
            let ds = t * (f.sigma_t - f.sigma_c) * f.area;
            my += ds * f.z;
            mz += -ds * f.y;
            return Some([my, mz]);
        }
        n += dn;
        my += (f.sigma_t - f.sigma_c) * f.area * f.z;
        mz += -(f.sigma_t - f.sigma_c) * f.area * f.y;
    }
    Some([my, mz])
}

/// 軸力一定での My–Mz 相関曲線を `n_pts` 点で返す。軸耐力範囲外なら空。
pub fn slice_at_n(fibers: &[PlasticFiber], n_target: f64, n_pts: usize) -> Vec<[f64; 2]> {
    let mut pts = Vec::with_capacity(n_pts);
    for j in 0..n_pts {
        let beta = 2.0 * std::f64::consts::PI * j as f64 / n_pts as f64;
        let (ky, kz) = (beta.cos(), beta.sin());
        if let Some(m) = plastic_moment_at_n(fibers, ky, kz, n_target) {
            pts.push(m);
        }
    }
    pts
}
