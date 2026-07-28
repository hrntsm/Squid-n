use super::fibers::{mesh_rect, FiberMat};
use super::*;
use approx::assert_relative_eq;
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

fn steel_rect_fibers(b: f64, d: f64, fy: f64, n: usize) -> Vec<PlasticFiber> {
    let mut fibers = Vec::new();
    mesh_rect(
        &mut fibers,
        [0.0, 0.0],
        b,
        d,
        d / n as f64,
        FiberMat {
            sigma_t: fy,
            sigma_c: -fy,
            young: 205000.0,
            region: super::types::FiberRegion::Steel,
        },
    );
    fibers
}

#[test]
fn test_axial_capacity_steel_rect() {
    let fibers = steel_rect_fibers(100.0, 200.0, 235.0, 40);
    let (nc, nt) = axial_capacity(&fibers);
    let npl = 100.0 * 200.0 * 235.0;
    assert_relative_eq!(nt, npl, max_relative = 1e-9);
    assert_relative_eq!(nc, -npl, max_relative = 1e-9);
}

#[test]
fn test_plastic_moment_steel_rect() {
    // 矩形鋼断面の全塑性モーメント Mp = fy·b·d²/4
    let (b, d, fy) = (100.0, 200.0, 235.0);
    let fibers = steel_rect_fibers(b, d, fy, 200);
    let m = plastic_moment_at_n(&fibers, 1.0, 0.0, 0.0).unwrap();
    let mp_exact = fy * b * d * d / 4.0;
    assert_relative_eq!(m[0], mp_exact, max_relative = 1e-2);
    assert_relative_eq!(m[1], 0.0, epsilon = mp_exact * 1e-9);
}

#[test]
fn test_mn_interaction_steel_rect() {
    // 矩形鋼断面の厳密解: M/Mp = 1 − (N/Npl)²
    let (b, d, fy) = (100.0, 200.0, 235.0);
    let fibers = steel_rect_fibers(b, d, fy, 400);
    let npl = b * d * fy;
    let mp = fy * b * d * d / 4.0;
    for ratio in [0.25, 0.5, 0.75] {
        let m = plastic_moment_at_n(&fibers, 1.0, 0.0, ratio * npl).unwrap();
        let m_exact = mp * (1.0 - ratio * ratio);
        assert_relative_eq!(m[0], m_exact, max_relative = 1e-2);
    }
}

#[test]
fn test_plastic_moment_outside_range() {
    let fibers = steel_rect_fibers(100.0, 200.0, 235.0, 20);
    let npl = 100.0 * 200.0 * 235.0;
    assert!(plastic_moment_at_n(&fibers, 1.0, 0.0, npl * 1.01).is_none());
    assert!(plastic_moment_at_n(&fibers, 1.0, 0.0, -npl * 1.01).is_none());
}

fn sample_rc_shape() -> SectionShape {
    SectionShape::RcRect {
        b: 500.0,
        d: 500.0,
        rebar: RcRebar {
            main_grade: None,
            main_x: BarSet {
                count: 4,
                dia: 22.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 2,
                dia: 22.0,
                layers: 1,
            },
            cover: 50.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        },
    }
}

#[test]
fn test_rc_rect_capacity() {
    let strength = StrengthParams::default();
    let fibers = plastic_fibers(&sample_rc_shape(), &strength, YieldModelKind::MultiFiber);
    // 引張耐力 = 主筋のみ: (4×2 + 2×2) 本 × a × fy
    let a_bar = std::f64::consts::PI * 22.0 * 22.0 / 4.0;
    let nt_exact = 12.0 * a_bar * strength.rebar_fy;
    let (nc, nt) = axial_capacity(&fibers);
    assert_relative_eq!(nt, nt_exact, max_relative = 1e-9);
    // 圧縮耐力 = コンクリート全断面 + 主筋
    let nc_exact = -(500.0 * 500.0 * strength.concrete_fc + 12.0 * a_bar * strength.rebar_fy);
    assert_relative_eq!(nc, nc_exact, max_relative = 1e-9);
}

#[test]
fn test_rc_moment_increases_with_moderate_compression() {
    // RC 断面は適度な圧縮軸力下で曲げ耐力が増す（相関曲線のふくらみ）
    let strength = StrengthParams::default();
    let fibers = plastic_fibers(&sample_rc_shape(), &strength, YieldModelKind::MultiFiber);
    let (nc, _) = axial_capacity(&fibers);
    let m0 = plastic_moment_at_n(&fibers, 1.0, 0.0, 0.0).unwrap()[0];
    let m_comp = plastic_moment_at_n(&fibers, 1.0, 0.0, 0.3 * nc).unwrap()[0];
    assert!(
        m_comp > m0,
        "M at 0.3Nc ({m_comp}) must exceed M at N=0 ({m0})"
    );
}

#[test]
fn test_build_surface_grid_shape_and_poles() {
    let fibers = steel_rect_fibers(100.0, 200.0, 235.0, 40);
    let surf = build_surface(&fibers, YieldModelKind::MultiFiber, 16, 32);
    assert_eq!(surf.grid.len(), 17);
    assert!(surf.grid.iter().all(|row| row.len() == 32));
    // 極（α=0/π）は純引張/純圧縮で一定
    let npl = 100.0 * 200.0 * 235.0;
    for p in &surf.grid[0] {
        assert_relative_eq!(p[0], npl, max_relative = 1e-9);
    }
    for p in &surf.grid[16] {
        assert_relative_eq!(p[0], -npl, max_relative = 1e-9);
    }
    // 全点が有限値
    assert!(surf
        .grid
        .iter()
        .flatten()
        .all(|p| p.iter().all(|v| v.is_finite())));
}

#[test]
fn test_simple_spring_surface_linear_interaction() {
    let fibers = steel_rect_fibers(100.0, 200.0, 235.0, 100);
    let surf = build_simple_spring_surface(&fibers, 16, 32);
    let npl = 100.0 * 200.0 * 235.0;
    // 極は軸耐力に一致
    for p in &surf.grid[0] {
        assert_relative_eq!(p[0], npl, max_relative = 1e-6);
    }
    for p in &surf.grid[16] {
        assert_relative_eq!(p[0], -npl, max_relative = 1e-6);
    }
    // 全格子点が |N|/N許容 + √((My/Mp_y)² + (Mz/Mp_z)²) = 1 を満たす
    for p in surf.grid.iter().flatten() {
        let n_ref = if p[0] >= 0.0 {
            surf.n_tens
        } else {
            surf.n_comp.abs()
        };
        let f =
            p[0].abs() / n_ref + ((p[1] / surf.mp_y).powi(2) + (p[2] / surf.mp_z).powi(2)).sqrt();
        assert_relative_eq!(f, 1.0, max_relative = 1e-9);
    }
    // 赤道（α=π/2、i=8）は N=0 の全塑性モーメント楕円: β=0 で My = Mp_y
    let equator = &surf.grid[8];
    assert_relative_eq!(equator[0][0], 0.0, epsilon = npl * 1e-12);
    assert_relative_eq!(equator[0][1], surf.mp_y, max_relative = 1e-9);
}

#[test]
fn test_slice_at_n_symmetric() {
    let fibers = steel_rect_fibers(100.0, 200.0, 235.0, 100);
    let pts = slice_at_n(&fibers, 0.0, 16);
    assert_eq!(pts.len(), 16);
    // β=0 は純 y 軸まわり曲げ → My = Mp_y
    let mp_y = 235.0 * 100.0 * 200.0 * 200.0 / 4.0;
    assert_relative_eq!(pts[0][0], mp_y, max_relative = 1e-2);
    // 対称性: β と β+π で符号反転
    assert_relative_eq!(pts[0][0], -pts[8][0], max_relative = 1e-9);
}

#[test]
fn test_multispring_is_coarser_than_fiber() {
    let strength = StrengthParams::default();
    let shape = sample_rc_shape();
    let ms = plastic_fibers(&shape, &strength, YieldModelKind::MultiSpring);
    let fib = plastic_fibers(&shape, &strength, YieldModelKind::MultiFiber);
    assert!(
        ms.len() < fib.len() / 10,
        "MS ({}) must be much coarser than fiber ({})",
        ms.len(),
        fib.len()
    );
    // 軸耐力は離散化によらず一致する（面積保存）
    let (nc_ms, nt_ms) = axial_capacity(&ms);
    let (nc_f, nt_f) = axial_capacity(&fib);
    assert_relative_eq!(nc_ms, nc_f, max_relative = 1e-9);
    assert_relative_eq!(nt_ms, nt_f, max_relative = 1e-9);
}

#[test]
fn test_steel_h_plastic_moment() {
    // H-400×200×8×13 の Mp ≈ fy × Zp（Zp = 手計算）
    let shape = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    };
    let strength = StrengthParams::default();
    let fibers = plastic_fibers(&shape, &strength, YieldModelKind::MultiFiber);
    // Zp = B·tf·(H−tf) + tw·(H−2tf)²/4
    let zp =
        200.0 * 13.0 * (400.0 - 13.0) + 8.0 * (400.0 - 2.0 * 13.0) * (400.0 - 2.0 * 13.0) / 4.0;
    let m = plastic_moment_at_n(&fibers, 1.0, 0.0, 0.0).unwrap();
    assert_relative_eq!(m[0], 235.0 * zp, max_relative = 2e-2);
}

#[test]
fn test_tee_centroid_correction() {
    // 非対称断面（T形）: 図心補正後、純軸力で M が出ないこと
    let shape = SectionShape::SteelTee {
        height: 200.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 12.0,
    };
    let strength = StrengthParams::default();
    let fibers = plastic_fibers(&shape, &strength, YieldModelKind::MultiFiber);
    let a_sum: f64 = fibers.iter().map(|f| f.area).sum();
    let cz: f64 = fibers.iter().map(|f| f.area * f.z).sum::<f64>() / a_sum;
    assert_relative_eq!(cz, 0.0, epsilon = 1e-9);
}

#[test]
fn test_m_phi_initial_stiffness_and_plateau() {
    // 矩形鋼断面: 初期剛性 EI₀ ≈ E·I、終局は全塑性 Mp に漸近
    let (b, d, fy, e) = (100.0, 200.0, 235.0, 205000.0);
    let fibers = steel_rect_fibers(b, d, fy, 200);
    let curve = m_phi_curve(&fibers, 1.0, 0.0, 0.0, 60).unwrap();
    // 離散断面の I（ファイバ位置ベース）と比較
    let i_disc: f64 = fibers.iter().map(|f| f.area * f.z * f.z).sum();
    assert_relative_eq!(curve.ei0, e * i_disc, max_relative = 1e-6);
    // 最終点は Mp = fy·b·d²/4 の 99% 以上
    let mp = fy * b * d * d / 4.0;
    let m_last = curve.points.last().unwrap()[1];
    assert!(
        m_last > 0.99 * mp && m_last <= mp * (1.0 + 1e-9),
        "m_last = {m_last}, mp = {mp}"
    );
    // 単調非減少
    for w in curve.points.windows(2) {
        assert!(w[1][1] >= w[0][1] - 1e-6);
    }
}

#[test]
fn test_m_phi_with_axial_force_reduces_plateau() {
    // N = 0.5·Npl では平坦部が Mp·(1-0.25) に漸近（厳密解 M/Mp = 1-(N/Npl)²）
    let (b, d, fy) = (100.0, 200.0, 235.0);
    let fibers = steel_rect_fibers(b, d, fy, 200);
    let npl = b * d * fy;
    let mp = fy * b * d * d / 4.0;
    let curve = m_phi_curve(&fibers, 1.0, 0.0, 0.5 * npl, 60).unwrap();
    let m_last = curve.points.last().unwrap()[1];
    assert_relative_eq!(m_last, 0.75 * mp, max_relative = 2e-2);
}

#[test]
fn test_m_phi_outside_axial_range() {
    let fibers = steel_rect_fibers(100.0, 200.0, 235.0, 20);
    let npl = 100.0 * 200.0 * 235.0;
    assert!(m_phi_curve(&fibers, 1.0, 0.0, npl * 1.01, 20).is_none());
    assert!(m_phi_curve(&fibers, 1.0, 0.0, -npl * 1.01, 20).is_none());
}

#[test]
fn test_m_theta_elastic_slope_and_plastic_rotation() {
    // 弾性域: θ = M·L/(6EI₀)。塑性域: θp = Lp·(φ - M/EI₀) が加算される。
    let fibers = steel_rect_fibers(100.0, 200.0, 235.0, 200);
    let curve = m_phi_curve(&fibers, 1.0, 0.0, 0.0, 60).unwrap();
    let (span, lp) = (4000.0, 100.0);
    let mtheta = m_theta_curve(&curve, span, lp);
    assert_eq!(mtheta.len(), curve.points.len());
    // 弾性域の点（最初の非零点）: 傾き 6EI₀/L
    let [th1, m1] = mtheta[1];
    assert_relative_eq!(m1 / th1, 6.0 * curve.ei0 / span, max_relative = 1e-6);
    // 最終点（全塑性近傍）: θ = M·L/6EI₀ + Lp·(φ - M/EI₀) を満たす
    let [phi_l, m_l] = *curve.points.last().unwrap();
    let [th_l, m_l2] = *mtheta.last().unwrap();
    assert_relative_eq!(m_l2, m_l, max_relative = 1e-12);
    let expected = m_l * span / (6.0 * curve.ei0) + lp * (phi_l - m_l / curve.ei0);
    assert_relative_eq!(th_l, expected, max_relative = 1e-9);
    // 塑性回転分は正
    assert!(th_l > m_l * span / (6.0 * curve.ei0));
}

#[test]
fn test_m_phi_rc_section() {
    // RC断面: 引張側コンクリートが効かない弾完全塑性でも有限・単調な M-φ になる
    let strength = StrengthParams::default();
    let fibers = plastic_fibers(&sample_rc_shape(), &strength, YieldModelKind::MultiFiber);
    let (nc, _) = axial_capacity(&fibers);
    let curve = m_phi_curve(&fibers, 1.0, 0.0, 0.2 * nc, 40).unwrap();
    assert!(curve.ei0.is_finite() && curve.ei0 > 0.0);
    for w in curve.points.windows(2) {
        assert!(w[1][1].is_finite());
        assert!(w[1][1] >= w[0][1] - 1.0); // 数値誤差の微小許容
    }
    // 終局は全塑性耐力（plastic_moment_at_n）に漸近する
    let mp = plastic_moment_at_n(&fibers, 1.0, 0.0, 0.2 * nc).unwrap()[0];
    let m_last = curve.points.last().unwrap()[1];
    assert!(m_last > 0.95 * mp, "m_last = {m_last}, mp = {mp}");
}

#[test]
fn test_m_phi_multispring_is_piecewise() {
    // マルチスプリングの M-φ は少数バネの逐次降伏による折れ線
    // （= ファイバーより早く初期降伏の折れ点が現れる）。ここでは両者の
    // 初期剛性がほぼ一致し、終局耐力もほぼ一致することを確認する。
    let strength = StrengthParams::default();
    let shape = sample_rc_shape();
    let ms = plastic_fibers(&shape, &strength, YieldModelKind::MultiSpring);
    let fib = plastic_fibers(&shape, &strength, YieldModelKind::MultiFiber);
    let c_ms = m_phi_curve(&ms, 1.0, 0.0, 0.0, 60).unwrap();
    let c_f = m_phi_curve(&fib, 1.0, 0.0, 0.0, 60).unwrap();
    assert_relative_eq!(c_ms.ei0, c_f.ei0, max_relative = 5e-2);
    let m_ms = c_ms.points.last().unwrap()[1];
    let m_f = c_f.points.last().unwrap()[1];
    assert_relative_eq!(m_ms, m_f, max_relative = 8e-2);
}
