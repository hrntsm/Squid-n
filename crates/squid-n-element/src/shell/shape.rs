//! 双一次形状関数・ヤコビアン・2×2 ガウス積分点。
//!
//! - [`shape_2d`] / [`dshape_2d`] — 形状関数と自然座標微分
//! - [`jacobian`] / [`jacobian_det`] / [`jacobian_inv_transpose`] — ヤコビアン関連
//! - [`dshape_cart`] — 直交座標微分 J⁻¹·[dN_dξ; dN_dη]
//! - [`GAUSS_PTS_2`] — 2×2 ガウス積分点と重み

pub(crate) fn shape_2d(xi: f64, eta: f64) -> [f64; 4] {
    [
        0.25 * (1.0 - xi) * (1.0 - eta),
        0.25 * (1.0 + xi) * (1.0 - eta),
        0.25 * (1.0 + xi) * (1.0 + eta),
        0.25 * (1.0 - xi) * (1.0 + eta),
    ]
}

pub(crate) fn dshape_2d(xi: f64, eta: f64) -> [[f64; 4]; 2] {
    let dxi = [
        -0.25 * (1.0 - eta),
        0.25 * (1.0 - eta),
        0.25 * (1.0 + eta),
        -0.25 * (1.0 + eta),
    ];
    let deta = [
        -0.25 * (1.0 - xi),
        -0.25 * (1.0 + xi),
        0.25 * (1.0 + xi),
        0.25 * (1.0 - xi),
    ];
    [dxi, deta]
}

#[allow(non_snake_case)]
pub(crate) fn jacobian(xi: f64, eta: f64, nodes: &[[f64; 3]; 4]) -> [[f64; 2]; 2] {
    let dN = dshape_2d(xi, eta);
    let mut jac = [[0.0; 2]; 2];
    for i in 0..4 {
        jac[0][0] += dN[0][i] * nodes[i][0];
        jac[0][1] += dN[0][i] * nodes[i][1];
        jac[1][0] += dN[1][i] * nodes[i][0];
        jac[1][1] += dN[1][i] * nodes[i][1];
    }
    jac
}

pub(crate) fn jacobian_det(jac: &[[f64; 2]; 2]) -> f64 {
    jac[0][0] * jac[1][1] - jac[0][1] * jac[1][0]
}

pub(crate) fn jacobian_inv_transpose(jac: &[[f64; 2]; 2]) -> [[f64; 2]; 2] {
    let det = jacobian_det(jac);
    if det.abs() < 1e-30 {
        return [[1.0, 0.0], [0.0, 1.0]];
    }
    let inv_det = 1.0 / det;
    [
        [jac[1][1] * inv_det, -jac[1][0] * inv_det],
        [-jac[0][1] * inv_det, jac[0][0] * inv_det],
    ]
}

/// Cartesian derivatives: [dNdx; dNdy] = J^{-1} * [dN_dxi; dN_deta]
#[allow(non_snake_case)]
pub(crate) fn dshape_cart(xi: f64, eta: f64, nodes: &[[f64; 3]; 4]) -> [[f64; 4]; 2] {
    let jac = jacobian(xi, eta, nodes);
    let jit = jacobian_inv_transpose(&jac);
    let dN = dshape_2d(xi, eta);
    let mut dNc = [[0.0; 4]; 2];
    for i in 0..4 {
        dNc[0][i] = jit[0][0] * dN[0][i] + jit[1][0] * dN[1][i];
        dNc[1][i] = jit[0][1] * dN[0][i] + jit[1][1] * dN[1][i];
    }
    dNc
}

pub(crate) const G2: f64 = 0.577_350_269_189_625_7;
pub(crate) const GAUSS_PTS_2: [(f64, f64, f64); 4] = [
    (-G2, -G2, 1.0),
    (G2, -G2, 1.0),
    (G2, G2, 1.0),
    (-G2, G2, 1.0),
];
