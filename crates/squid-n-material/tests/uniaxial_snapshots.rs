//! 単軸履歴則（Concrete/Bilinear/MP）のスナップショットテスト（仕様書 §8.1）。
//! 規定の繰り返しひずみ履歴に対する (ε, σ) ループを insta で固定し、回帰を検出する。

use squid_n_material::{Bilinear, Concrete, ConcreteCyclic, MenegottoPinto, UniaxialMaterial};

/// 規定の繰り返しひずみ履歴: 段階的に振幅を増やす正負交番。
fn cyclic_strain_history(eps_y: f64) -> Vec<f64> {
    let mut hist = vec![0.0];
    for amp in [1.0, 2.0, 3.0, 4.0] {
        hist.extend([eps_y * amp, 0.0, -eps_y * amp, 0.0]);
    }
    hist
}

fn run_uniaxial_loop(mat: &mut dyn UniaxialMaterial, history: &[f64]) -> String {
    let mut out = String::from("eps,sigma,tangent\n");
    for &eps in history {
        let (sigma, t) = mat.trial(eps);
        out.push_str(&format!("{:.6},{:.6},{:.6}\n", eps, sigma, t));
        mat.commit();
    }
    out
}

#[test]
fn snapshot_bilinear_loop() {
    let mut mat = Bilinear::new(205000.0, 235.0, 0.01);
    let eps_y = 235.0 / 205000.0;
    let hist = cyclic_strain_history(eps_y);
    let csv = run_uniaxial_loop(&mut mat, &hist);
    insta::assert_snapshot!("bilinear_loop", csv);
}

#[test]
fn snapshot_menegotto_pinto_loop() {
    let mut mat = MenegottoPinto::new(200000.0, 345.0);
    let eps_y = 345.0 / 200000.0;
    let hist = cyclic_strain_history(eps_y);
    let csv = run_uniaxial_loop(&mut mat, &hist);
    insta::assert_snapshot!("menegotto_pinto_loop", csv);
}

#[test]
fn snapshot_concrete_cyclic_loop() {
    // Yassin (1994)/Concrete02 系: 引張ひび割れ → 圧縮包絡線 → Karsan–Jirsa 除荷 →
    // ひび割れ閉鎖を含む再圧縮、の順で全分岐を通す履歴。
    let mut mat = ConcreteCyclic::kent_park(30.0, 0.002, 6.0, 0.008, 2.0, 1000.0);
    let e0 = 2.0 * 30.0 / 0.002;
    let eps_cr = 2.0 / e0;
    let hist = vec![
        0.0,
        eps_cr,
        eps_cr * 3.0,
        0.0,
        -0.002,
        -0.004,
        -0.001,
        0.0005,
        -0.004,
        -0.006,
        0.0,
    ];
    let csv = run_uniaxial_loop(&mut mat, &hist);
    insta::assert_snapshot!("concrete_cyclic_loop", csv);
}

#[test]
fn snapshot_concrete_loop() {
    // コンクリート: 引張ひび割れ〜圧縮降伏の履歴
    let mut mat = Concrete::new(30.0, 2.0);
    let e0 = 2.0 * 30.0 / 0.002;
    let eps_cr = 2.0 / e0;
    // 引張ひび割れ → 圧縮ピーク → 圧縮軟化 → 引張再載荷
    let hist = vec![
        0.0,
        eps_cr * 0.5,
        eps_cr,
        eps_cr * 2.0,
        0.0,
        -0.0005,
        -0.002,
        -0.003,
        -0.0035,
        0.0,
        eps_cr * 0.5,
        eps_cr * 1.5,
        0.0,
    ];
    let csv = run_uniaxial_loop(&mut mat, &hist);
    insta::assert_snapshot!("concrete_loop", csv);
}
