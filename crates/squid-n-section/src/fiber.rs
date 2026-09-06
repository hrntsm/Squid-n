use squid_n_material::UniaxialMaterial;

/// ファイバ（断面小片）。単位: `y`,`z` [mm], `area` [mm²]。
/// `material` は材料種別の参照タグ。状態は `mats[fiber_index]` が持つ。
pub struct Fiber {
    pub y: f64,
    pub z: f64,
    pub area: f64,
    pub material: usize,
}

/// ファイバ断面：多数のファイバで構成される。
pub struct FiberSection {
    pub fibers: Vec<Fiber>,
}

/// 断面ひずみ（平面保持）。`eps0` [無次元], `ky`,`kz` [1/mm]。
pub struct SectionStrain {
    pub eps0: f64,
    pub ky: f64,
    pub kz: f64,
}

/// 断面力（N, My, Mz）。`n` [N], `my`,`mz` [N·mm]。
pub struct SectionForce {
    pub n: f64,
    pub my: f64,
    pub mz: f64,
}

/// 接線断面剛性（3×3：N–My–Mz と ε0–κy–κz の関係）。単位は N, N·mm, mm。
pub struct SectionStiffness {
    pub d: [[f64; 3]; 3],
}

/// 各ファイバのひずみ = ε0 − κz·y + κy·z（平面保持）から (σ, Et) を積分する。
///
/// **契約:** `mats.len() == sec.fibers.len()`。各ファイバは独立した履歴状態を持つこと。
pub fn section_response(
    sec: &FiberSection,
    strain: SectionStrain,
    mats: &mut [Box<dyn UniaxialMaterial>],
) -> (SectionForce, SectionStiffness) {
    assert_eq!(
        mats.len(),
        sec.fibers.len(),
        "section_response: mats.len() must equal fibers.len() (per-fiber state)"
    );
    integrate_fibers(sec, |i, fiber| {
        let eps = strain.eps0 - strain.kz * fiber.y + strain.ky * fiber.z;
        mats[i].trial(eps)
    })
}

/// ファイバー単位の応力と接線係数から、断面力と接線断面剛性 D を積分する。
/// 積分公式はこの関数だけが持つ（2 経路で共有しビット一致を保つ）。
/// 符号規約: `My = Σσ·A·z`、`Mz = −Σσ·A·y`。D は対称。
pub fn integrate_fibers(
    sec: &FiberSection,
    mut per_fiber: impl FnMut(usize, &Fiber) -> (f64, f64),
) -> (SectionForce, SectionStiffness) {
    let mut n = 0.0;
    let mut my = 0.0;
    let mut mz = 0.0;
    let mut d = [[0.0; 3]; 3];

    for (i, fiber) in sec.fibers.iter().enumerate() {
        let (sigma, et) = per_fiber(i, fiber);
        let a = fiber.area;

        n += sigma * a;
        my += sigma * a * fiber.z;
        mz += -sigma * a * fiber.y;

        let ea = et * a;
        let ey = ea * fiber.y;
        let ez = ea * fiber.z;

        d[0][0] += ea;
        d[0][1] += ez;
        d[0][2] += -ey;

        d[1][0] += ez;
        d[1][1] += ez * fiber.z;
        d[1][2] += -ey * fiber.z;

        d[2][0] += -ey;
        d[2][1] += -ey * fiber.z;
        d[2][2] += ey * fiber.y;
    }

    (SectionForce { n, my, mz }, SectionStiffness { d })
}

/// テンプレート材料からファイバ数分の独立した状態インスタンスを生成する。
pub fn uniform_fiber_mats(
    template: &dyn UniaxialMaterial,
    n: usize,
) -> Vec<Box<dyn UniaxialMaterial>> {
    (0..n).map(|_| template.clone_box()).collect()
}

/// 矩形断面をファイバに分割する。`material` は材料種別タグ。
pub fn rect_fiber_section(
    width: f64,
    depth: f64,
    nw: usize,
    nd: usize,
    material: usize,
) -> FiberSection {
    let mut fibers = Vec::with_capacity(nw * nd);
    let dw = width / nw as f64;
    let dd = depth / nd as f64;
    for i in 0..nw {
        for j in 0..nd {
            let y = (i as f64 + 0.5) * dw - width / 2.0;
            let z = (j as f64 + 0.5) * dd - depth / 2.0;
            fibers.push(Fiber {
                y,
                z,
                area: dw * dd,
                material,
            });
        }
    }
    FiberSection { fibers }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use squid_n_material::{Bilinear, Concrete};

    fn steel_mats(sec: &FiberSection) -> Vec<Box<dyn UniaxialMaterial>> {
        uniform_fiber_mats(&Bilinear::new(205000.0, 235.0, 0.01), sec.fibers.len())
    }

    #[test]
    fn test_section_axial_strict() {
        let sec = rect_fiber_section(100.0, 200.0, 10, 20, 0);
        let mut mats = steel_mats(&sec);
        let (force, stiff) = section_response(
            &sec,
            SectionStrain {
                eps0: 0.001,
                ky: 0.0,
                kz: 0.0,
            },
            &mut mats,
        );
        // 離散断面積 A_disc = Σ area
        let a_disc: f64 = sec.fibers.iter().map(|f| f.area).sum();
        let expected_n = 0.001 * 205000.0 * a_disc;
        assert_relative_eq!(force.n, expected_n, max_relative = 1e-9);
        assert_relative_eq!(force.my, 0.0, epsilon = 1e-9);
        assert_relative_eq!(force.mz, 0.0, epsilon = 1e-9);
        assert_relative_eq!(stiff.d[0][0], 205000.0 * a_disc, max_relative = 1e-9);
    }

    #[test]
    fn test_section_pure_bending_strict() {
        let sec = rect_fiber_section(100.0, 200.0, 20, 40, 0);
        let mut mats = steel_mats(&sec);
        let ky = 1e-6;
        let (force, stiff) = section_response(
            &sec,
            SectionStrain {
                eps0: 0.0,
                ky,
                kz: 0.0,
            },
            &mut mats,
        );
        // 離散 Iy = Σ area·z²。弾性域はファイバ積分が厳密に E·Iy_disc に一致する。
        let iy_disc: f64 = sec.fibers.iter().map(|f| f.area * f.z * f.z).sum();
        let expected_my = ky * 205000.0 * iy_disc;
        assert_relative_eq!(force.my, expected_my, max_relative = 1e-9);
        assert_relative_eq!(force.n, 0.0, epsilon = 1e-9);
        assert_relative_eq!(stiff.d[1][1], 205000.0 * iy_disc, max_relative = 1e-9);
    }

    #[test]
    fn test_section_mismatched_mats_panics() {
        let sec = rect_fiber_section(100.0, 200.0, 10, 20, 0);
        let mut mats: Vec<Box<dyn UniaxialMaterial>> =
            vec![Box::new(Bilinear::new(205000.0, 235.0, 0.01))];
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            section_response(
                &sec,
                SectionStrain {
                    eps0: 0.0,
                    ky: 0.0,
                    kz: 0.0,
                },
                &mut mats,
            )
        }));
        assert!(
            result.is_err(),
            "must panic when mats.len() != fibers.len()"
        );
    }

    #[test]
    fn test_section_yield_progression() {
        // 純曲げで曲率を増やし、ファイバが順次降伏して M-φ が弾性予測から乖離することを確認。
        // ファイバごとに独立状態を持つため、降伏進展が正しく追跡できる。
        let sec = rect_fiber_section(100.0, 200.0, 4, 40, 0);
        let mut mats = steel_mats(&sec);
        let eps_y = 235.0 / 205000.0;
        let z_max = 100.0;
        let ky_y = eps_y / z_max; // 最外縁降伏開始曲率
        let mut last_m = 0.0;
        let n_steps = 50;
        for i in 1..=n_steps {
            let ky = ky_y * 3.0 * (i as f64) / (n_steps as f64);
            let (force, _) = section_response(
                &sec,
                SectionStrain {
                    eps0: 0.0,
                    ky,
                    kz: 0.0,
                },
                &mut mats,
            );
            for m in mats.iter_mut() {
                m.commit();
            }
            last_m = force.my;
        }
        // 降伏後の M は弾性予測 E·I·ky より小さい（剛性低下）
        let iy_disc: f64 = sec.fibers.iter().map(|f| f.area * f.z * f.z).sum();
        let ky_final = ky_y * 3.0;
        let elastic_pred = ky_final * 205000.0 * iy_disc;
        assert!(
            last_m < elastic_pred,
            "post-yield M ({}) must be below elastic prediction ({})",
            last_m,
            elastic_pred
        );
    }

    #[test]
    fn test_section_concrete_softening_mphi() {
        // コンクリート断面の圧縮側が軟化してもファイバ状態が独立していれば
        // 積分が破綮しない（共有状態だと履歴混入で発散）
        let sec = rect_fiber_section(100.0, 200.0, 4, 20, 0);
        let mut mats = uniform_fiber_mats(&Concrete::new(30.0, 2.0), sec.fibers.len());
        let mut max_m = 0.0f64;
        let mut min_m = 0.0f64;
        for i in 0..=100 {
            let kz = -0.00002 * (i as f64) / 100.0 * 200.0; // 曲率増加
            let (force, _) = section_response(
                &sec,
                SectionStrain {
                    eps0: 0.0,
                    ky: 0.0,
                    kz,
                },
                &mut mats,
            );
            for m in mats.iter_mut() {
                m.commit();
            }
            max_m = max_m.max(force.mz.abs());
            min_m = min_m.min(force.mz);
        }
        // ピーク後軟化で |M| が減少に転じることを確認（少なくとも有限値で発散しない）
        assert!(max_m.is_finite() && max_m > 0.0);
    }
}
