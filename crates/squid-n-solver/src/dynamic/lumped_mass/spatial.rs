//! 3 次元質点系（各階 Ux, Uy, θz）の固有値と時刻歴。

use super::eigen::LumpedMassModal;
use super::model::{LumpedMassModel, StorySpatial};
use super::time_history::{
    dir_ductility, story_spring, StickDirPeaks, StickResponse, STICK_NEWTON,
};
use crate::statics::analysis::SeismicDir;
use faer::Side;
use squid_n_material::{HysteresisMaterial, UniaxialMaterial};
use squid_n_math::solver::SolveError;

/// 剛心位置の 3×3 層剛性（質量重心座標へ変換済み）。
///
/// 剛心相対位置 `rx = Xs−Xg`, `ry = Ys−Yg`。DOF は質量重心の (Ux, Uy, θz)。
pub(crate) fn story_k_at_mass_center(s: &StorySpatial) -> [[f64; 3]; 3] {
    let rx = s.rigidity_xy[0] - s.mass_xy[0];
    let ry = s.rigidity_xy[1] - s.mass_xy[1];
    // 並進は骨格の初期剛性（非線形なら増分トリリニアの K1、線形なら弾性ばね）。
    // k1_x/k1_y は結果表示用の控えで、ここは復元力と同一の骨格を使う。
    let kx = s.skeleton_x.k1.max(0.0);
    let ky = s.skeleton_y.k1.max(0.0);
    let kr = s.kr.max(0.0);
    [
        [kx, 0.0, -kx * ry],
        [0.0, ky, ky * rx],
        [-kx * ry, ky * rx, kx * ry * ry + ky * rx * rx + kr],
    ]
}

fn assemble_spatial_k(spatial: &[StorySpatial], k_scale: impl Fn(usize, usize) -> f64) -> Vec<f64> {
    // k_scale(story, component) で kx/ky/kr をスケール。component 0=x,1=y,2=θ
    let n = spatial.len();
    let nd = 3 * n;
    let mut k = vec![0.0; nd * nd];
    let add = |k: &mut [f64], i: usize, j: usize, v: f64| {
        k[i * nd + j] += v;
    };
    for i in 0..n {
        let mut s = spatial[i];
        s.skeleton_x.k1 *= k_scale(i, 0);
        s.skeleton_y.k1 *= k_scale(i, 1);
        s.kr *= k_scale(i, 2);
        let ks = story_k_at_mass_center(&s);
        let top = 3 * i;
        for a in 0..3 {
            for b in 0..3 {
                add(&mut k, top + a, top + b, ks[a][b]);
            }
        }
        if i > 0 {
            let bot = 3 * (i - 1);
            for a in 0..3 {
                for b in 0..3 {
                    add(&mut k, bot + a, bot + b, ks[a][b]);
                    add(&mut k, top + a, bot + b, -ks[a][b]);
                    add(&mut k, bot + a, top + b, -ks[a][b]);
                }
            }
        }
    }
    k
}

fn mass_diag(lm: &LumpedMassModel) -> Result<Vec<f64>, SolveError> {
    let n = lm.stories.len();
    let mut m = vec![0.0; 3 * n];
    for i in 0..n {
        if lm.stories[i].mass <= 0.0 {
            return Err(SolveError::InvalidInput(format!(
                "層 {:?} の質量が 0 以下のため固有値解析できません",
                lm.stories[i].story
            )));
        }
        let j = lm.spatial[i].j;
        if j <= 0.0 {
            return Err(SolveError::InvalidInput(format!(
                "層 {:?} の回転慣性 J が 0 以下のため 3 次元質点系の固有値解析できません",
                lm.stories[i].story
            )));
        }
        m[3 * i] = lm.stories[i].mass;
        m[3 * i + 1] = lm.stories[i].mass;
        m[3 * i + 2] = j;
    }
    Ok(m)
}

pub(crate) fn lumped_mass_eigen_spatial(
    lm: &LumpedMassModel,
    n_modes: usize,
) -> Result<LumpedMassModal, SolveError> {
    let n = lm.stories.len();
    if n == 0 || n_modes == 0 || !lm.is_spatial() {
        return Ok(LumpedMassModal::default());
    }
    let mass = mass_diag(lm)?;
    let nd = mass.len();
    let k = assemble_spatial_k(&lm.spatial, |_, _| 1.0);
    let sqrt_m: Vec<f64> = mass.iter().map(|&mi| mi.sqrt()).collect();
    let a = faer::Mat::from_fn(nd, nd, |i, j| k[i * nd + j] / (sqrt_m[i] * sqrt_m[j]));
    let eig = a
        .self_adjoint_eigen(Side::Lower)
        .map_err(|e| SolveError::NonConvergence(format!("3次元質点系の固有値分解: {e:?}")))?;
    let s = eig.S();
    let u = eig.U();
    let take = n_modes.min(nd);
    let mut omega2 = Vec::with_capacity(take);
    let mut period = Vec::with_capacity(take);
    let mut shapes = Vec::with_capacity(take);
    let mut shapes_xyz = Vec::with_capacity(take);
    for j in 0..take {
        let w2 = s[j].max(0.0);
        let w = w2.sqrt();
        omega2.push(w2);
        period.push(if w > 0.0 {
            2.0 * std::f64::consts::PI / w
        } else {
            f64::INFINITY
        });
        let mut xyz = vec![[0.0; 3]; n];
        for i in 0..n {
            xyz[i] = [
                u[(3 * i, j)] / sqrt_m[3 * i],
                u[(3 * i + 1, j)] / sqrt_m[3 * i + 1],
                u[(3 * i + 2, j)] / sqrt_m[3 * i + 2],
            ];
        }
        // 頂部水平変位のノルムで正規化（0 なら頂部 θz）。
        let top = xyz[n - 1];
        let h = (top[0] * top[0] + top[1] * top[1]).sqrt();
        let scale = if h > 1e-30 {
            h
        } else if top[2].abs() > 1e-30 {
            top[2].abs()
        } else {
            1.0
        };
        for v in xyz.iter_mut() {
            v[0] /= scale;
            v[1] /= scale;
            v[2] /= scale;
        }
        let planar: Vec<f64> = xyz
            .iter()
            .map(|v| (v[0] * v[0] + v[1] * v[1]).sqrt().copysign(v[0] + v[1]))
            .collect();
        shapes.push(planar);
        shapes_xyz.push(xyz);
    }
    Ok(LumpedMassModal {
        omega2,
        period,
        shapes,
        shapes_xyz,
    })
}

fn k_matvec(k: &[f64], x: &[f64], nd: usize, scale: f64) -> Vec<f64> {
    let mut y = vec![0.0; nd];
    for i in 0..nd {
        let mut s = 0.0;
        for j in 0..nd {
            s += k[i * nd + j] * x[j];
        }
        y[i] = scale * s;
    }
    y
}

fn solve_dense(a: &[f64], b: &[f64], nd: usize) -> Vec<f64> {
    let mut m = vec![0.0; nd * (nd + 1)];
    for i in 0..nd {
        for j in 0..nd {
            m[i * (nd + 1) + j] = a[i * nd + j];
        }
        m[i * (nd + 1) + nd] = b[i];
    }
    for col in 0..nd {
        let mut piv = col;
        let mut best = m[col * (nd + 1) + col].abs();
        for i in (col + 1)..nd {
            let v = m[i * (nd + 1) + col].abs();
            if v > best {
                best = v;
                piv = i;
            }
        }
        if best < 1e-30 {
            continue;
        }
        if piv != col {
            for j in col..=nd {
                m.swap(col * (nd + 1) + j, piv * (nd + 1) + j);
            }
        }
        let diag = m[col * (nd + 1) + col];
        for i in (col + 1)..nd {
            let f = m[i * (nd + 1) + col] / diag;
            for j in col..=nd {
                m[i * (nd + 1) + j] -= f * m[col * (nd + 1) + j];
            }
        }
    }
    let mut x = vec![0.0; nd];
    for i in (0..nd).rev() {
        let mut s = m[i * (nd + 1) + nd];
        for j in (i + 1)..nd {
            s -= m[i * (nd + 1) + j] * x[j];
        }
        let d = m[i * (nd + 1) + i];
        x[i] = if d.abs() > 1e-30 { s / d } else { 0.0 };
    }
    x
}

fn relative_at_rigidity(u: &[f64], i: usize, s: &StorySpatial) -> [f64; 3] {
    let rx = s.rigidity_xy[0] - s.mass_xy[0];
    let ry = s.rigidity_xy[1] - s.mass_xy[1];
    let top = 3 * i;
    let (ux, uy, rz) = if i == 0 {
        (u[top], u[top + 1], u[top + 2])
    } else {
        let bot = 3 * (i - 1);
        (
            u[top] - u[bot],
            u[top + 1] - u[bot + 1],
            u[top + 2] - u[bot + 2],
        )
    };
    [ux - ry * rz, uy + rx * rz, rz]
}

fn add_story_force(f: &mut [f64], i: usize, s: &StorySpatial, fx: f64, fy: f64, mz: f64) {
    let rx = s.rigidity_xy[0] - s.mass_xy[0];
    let ry = s.rigidity_xy[1] - s.mass_xy[1];
    // 剛心力を質量重心へ: Fx, Fy, Mz + Fy*rx - Fx*ry
    let fx_g = fx;
    let fy_g = fy;
    let mz_g = mz + fy * rx - fx * ry;
    let top = 3 * i;
    f[top] += fx_g;
    f[top + 1] += fy_g;
    f[top + 2] += mz_g;
    if i > 0 {
        let bot = 3 * (i - 1);
        f[bot] -= fx_g;
        f[bot + 1] -= fy_g;
        f[bot + 2] -= mz_g;
    }
}

pub(crate) fn lumped_mass_time_history_spatial(
    lm: &LumpedMassModel,
    accel: &[f64],
    dt: f64,
    h: f64,
) -> StickResponse {
    let n = lm.stories.len();
    if n == 0 || dt <= 0.0 || accel.is_empty() || !lm.is_spatial() {
        return StickResponse::empty(n);
    }
    let mass = match mass_diag(lm) {
        Ok(m) => m,
        Err(_) => return StickResponse::empty(n),
    };
    let nd = mass.len();
    let k_init = assemble_spatial_k(&lm.spatial, |_, _| 1.0);
    let mut springs_x: Vec<HysteresisMaterial> = lm
        .spatial
        .iter()
        .map(|s| story_spring(&s.skeleton_x))
        .collect();
    let mut springs_y: Vec<HysteresisMaterial> = lm
        .spatial
        .iter()
        .map(|s| story_spring(&s.skeleton_y))
        .collect();

    let omega1 = super::eigen::stick_omega1(lm);
    let a1 = if omega1 > 0.0 { 2.0 * h / omega1 } else { 0.0 };
    let beta = 0.25;
    let gamma = 0.5;
    let c1 = 1.0 / (beta * dt * dt);
    let c2 = gamma / (beta * dt);

    let dir_idx = match lm.dir {
        SeismicDir::X => 0,
        SeismicDir::Y => 1,
    };

    let mut u = vec![0.0; nd];
    let mut v = vec![0.0; nd];
    let mut a = vec![0.0; nd];
    let mut time = Vec::with_capacity(accel.len());
    let mut roof = Vec::with_capacity(accel.len());
    let mut floor_disp = Vec::with_capacity(accel.len());
    let mut peak_drift = vec![0.0_f64; n];
    let mut peak_shear = vec![0.0_f64; n];
    let mut drift_dir = StickDirPeaks::zeros(n);
    let mut shear_dir = StickDirPeaks::zeros(n);
    let mut non_converged_steps = 0usize;
    let mut peak_force_scale = 0.0_f64;

    for (step, &ag) in accel.iter().enumerate() {
        let mut p = vec![0.0; nd];
        for i in 0..n {
            p[3 * i + dir_idx] = -mass[3 * i + dir_idx] * ag;
        }
        let u_prev = u.clone();
        let v_prev = v.clone();
        let a_prev = a.clone();
        let mut u_tr = u_prev.clone();
        let mut step_converged = false;

        for _iter in STICK_NEWTON.iters() {
            let mut f_int = vec![0.0; nd];
            let mut kt_x = vec![0.0; n];
            let mut kt_y = vec![0.0; n];
            for i in 0..n {
                let rel = relative_at_rigidity(&u_tr, i, &lm.spatial[i]);
                let (qx, kx) = springs_x[i].trial(rel[0]);
                let (qy, ky) = springs_y[i].trial(rel[1]);
                kt_x[i] = kx.max(1e-6);
                kt_y[i] = ky.max(1e-6);
                let mz = lm.spatial[i].kr.max(0.0) * rel[2];
                add_story_force(&mut f_int, i, &lm.spatial[i], qx, qy, mz);
            }
            let a_tr: Vec<f64> = (0..nd)
                .map(|i| {
                    c1 * (u_tr[i] - u_prev[i])
                        - (1.0 / (beta * dt)) * v_prev[i]
                        - (1.0 / (2.0 * beta) - 1.0) * a_prev[i]
                })
                .collect();
            let v_tr: Vec<f64> = (0..nd)
                .map(|i| v_prev[i] + dt * ((1.0 - gamma) * a_prev[i] + gamma * a_tr[i]))
                .collect();
            let cv = k_matvec(&k_init, &v_tr, nd, a1);
            let mut r = vec![0.0; nd];
            let mut rnorm = 0.0;
            for i in 0..nd {
                r[i] = p[i] - mass[i] * a_tr[i] - cv[i] - f_int[i];
                rnorm += r[i] * r[i];
            }
            let ma: Vec<f64> = (0..nd).map(|i| mass[i] * a_tr[i]).collect();
            let scale = crate::common::newton::dynamic_force_scale(&p, &ma, &cv);
            peak_force_scale = peak_force_scale.max(scale);
            let ref_norm = crate::common::newton::dynamic_reference_norm(scale, peak_force_scale);
            if STICK_NEWTON.converged(rnorm.sqrt(), ref_norm) {
                step_converged = true;
                break;
            }
            let kt = assemble_spatial_k(&lm.spatial, |i, c| match c {
                0 => kt_x[i] / lm.spatial[i].skeleton_x.k1.max(1e-9),
                1 => kt_y[i] / lm.spatial[i].skeleton_y.k1.max(1e-9),
                _ => 1.0,
            });
            let mut keff = vec![0.0; nd * nd];
            let cd = c2 * a1;
            for i in 0..nd {
                for j in 0..nd {
                    keff[i * nd + j] = kt[i * nd + j] + cd * k_init[i * nd + j];
                }
                keff[i * nd + i] += c1 * mass[i];
            }
            let du = solve_dense(&keff, &r, nd);
            for i in 0..nd {
                u_tr[i] += du[i];
            }
        }
        if !step_converged {
            non_converged_steps += 1;
        }
        for s in springs_x.iter_mut().chain(springs_y.iter_mut()) {
            s.commit();
        }
        let a_new: Vec<f64> = (0..nd)
            .map(|i| {
                c1 * (u_tr[i] - u_prev[i])
                    - (1.0 / (beta * dt)) * v_prev[i]
                    - (1.0 / (2.0 * beta) - 1.0) * a_prev[i]
            })
            .collect();
        let v_new: Vec<f64> = (0..nd)
            .map(|i| v_prev[i] + dt * ((1.0 - gamma) * a_prev[i] + gamma * a_new[i]))
            .collect();
        u = u_tr;
        v = v_new;
        a = a_new;

        for i in 0..n {
            let rel = relative_at_rigidity(&u, i, &lm.spatial[i]);
            peak_drift[i] = peak_drift[i].max(drift_dir.accumulate(i, rel[0], rel[1]));
            let (qx, _) = {
                let mut sp = springs_x[i].clone();
                sp.trial(rel[0])
            };
            let (qy, _) = {
                let mut sp = springs_y[i].clone();
                sp.trial(rel[1])
            };
            peak_shear[i] = peak_shear[i].max(shear_dir.accumulate(i, qx, qy));
        }
        time.push(step as f64 * dt);
        let top = 3 * (n - 1) + dir_idx;
        roof.push(u[top]);
        let mut frame = vec![[0.0; 3]; n];
        for i in 0..n {
            frame[i] = [u[3 * i], u[3 * i + 1], u[3 * i + 2]];
        }
        floor_disp.push(frame);
    }

    let d1x: Vec<f64> = lm.spatial.iter().map(|s| s.skeleton_x.d1).collect();
    let d1y: Vec<f64> = lm.spatial.iter().map(|s| s.skeleton_y.d1).collect();
    let ductility_dir = dir_ductility(&drift_dir, &d1x, &d1y);
    let ductility = ductility_dir.story_max();

    StickResponse {
        time,
        roof_disp: roof,
        story_peak_drift: peak_drift,
        story_peak_shear: peak_shear,
        story_ductility: ductility,
        non_converged_steps,
        floor_disp,
        drift_dir,
        shear_dir,
        ductility_dir,
    }
}

#[cfg(test)]
mod tests {
    use super::super::model::{
        LumpedMassModel, LumpedMassType, LumpedStiffnessSource, StickDim, StorySpatial, StoryStick,
        StoryTrilinear,
    };
    use super::*;
    use crate::statics::analysis::SeismicDir;

    #[test]
    fn spatial_eigen_one_story_matches_sda() {
        let mass = 10.0;
        let kx = 100.0;
        let ky = 400.0;
        let j = 1.0e8;
        let kr = 1.0e10;
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            stories: vec![StoryStick {
                story: squid_n_core::ids::StoryId(0),
                mass,
                height: 3000.0,
                skeleton: StoryTrilinear::elastic(kx),
            }],
            dim: StickDim::Spatial,
            stiffness_source: LumpedStiffnessSource::StoryQd,
            dir: SeismicDir::X,
            nonlinear: false,
            spatial: vec![StorySpatial {
                j,
                mass_xy: [0.0, 0.0],
                rigidity_xy: [0.0, 0.0],
                k1_x: kx,
                k1_y: ky,
                kr,
                skeleton_x: StoryTrilinear::elastic(kx),
                skeleton_y: StoryTrilinear::elastic(ky),
            }],
        };
        let modal = lumped_mass_eigen_spatial(&lm, 3).unwrap();
        assert_eq!(modal.period.len(), 3);
        let mut got = modal.period.clone();
        got.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut expected = [
            2.0 * std::f64::consts::PI * (j / kr).sqrt(),
            2.0 * std::f64::consts::PI * (mass / ky).sqrt(),
            2.0 * std::f64::consts::PI * (mass / kx).sqrt(),
        ];
        expected.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for (g, want) in got.iter().zip(expected.iter()) {
            assert!((g - want).abs() / want < 1e-6, "period {g} vs {want}");
        }
    }

    #[test]
    fn spatial_eigen_uses_skeleton_k1_not_stale_field() {
        let mass = 10.0;
        let kx = 100.0;
        let ky = 400.0;
        let j = 1.0e8;
        let kr = 1.0e10;
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            stories: vec![StoryStick {
                story: squid_n_core::ids::StoryId(0),
                mass,
                height: 3000.0,
                skeleton: StoryTrilinear::elastic(kx),
            }],
            dim: StickDim::Spatial,
            stiffness_source: LumpedStiffnessSource::StoryQd,
            dir: SeismicDir::X,
            nonlinear: true,
            spatial: vec![StorySpatial {
                j,
                mass_xy: [0.0, 0.0],
                rigidity_xy: [0.0, 0.0],
                k1_x: 1.0,
                k1_y: 1.0,
                kr,
                skeleton_x: StoryTrilinear::elastic(kx),
                skeleton_y: StoryTrilinear::elastic(ky),
            }],
        };
        let modal = lumped_mass_eigen_spatial(&lm, 3).unwrap();
        let mut got = modal.period.clone();
        got.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut expected = [
            2.0 * std::f64::consts::PI * (j / kr).sqrt(),
            2.0 * std::f64::consts::PI * (mass / ky).sqrt(),
            2.0 * std::f64::consts::PI * (mass / kx).sqrt(),
        ];
        expected.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for (g, want) in got.iter().zip(expected.iter()) {
            assert!((g - want).abs() / want < 1e-6, "period {g} vs {want}");
        }
    }

    #[test]
    fn spatial_th_x_motion_fills_directional_peaks() {
        let mass = 10.0;
        let k = 100.0;
        let lm = LumpedMassModel {
            model_type: LumpedMassType::EquivalentShear,
            stories: vec![StoryStick {
                story: squid_n_core::ids::StoryId(0),
                mass,
                height: 3000.0,
                skeleton: StoryTrilinear::elastic(k),
            }],
            dim: StickDim::Spatial,
            stiffness_source: LumpedStiffnessSource::StoryQd,
            dir: SeismicDir::X,
            nonlinear: false,
            spatial: vec![StorySpatial {
                j: 1.0e8,
                mass_xy: [0.0, 0.0],
                rigidity_xy: [0.0, 0.0],
                k1_x: k,
                k1_y: k,
                kr: 1.0e10,
                skeleton_x: StoryTrilinear::elastic(k),
                skeleton_y: StoryTrilinear::elastic(k),
            }],
        };
        let dt = 0.01;
        let accel: Vec<f64> = (0..200)
            .map(|i| 500.0 * (2.0 * std::f64::consts::PI * 0.5 * i as f64 * dt).sin())
            .collect();
        let res = lumped_mass_time_history_spatial(&lm, &accel, dt, 0.02);
        let dx = res.drift_dir.x[0];
        let dy = res.drift_dir.y[0];
        assert!(dx > 1e-3, "X 加振なのに層間Xがほぼ 0: {dx}");
        assert!(
            dy < 0.05 * dx,
            "偏心なしなら層間Yは X の数%以下: Y={dy} X={dx}"
        );
        let s2 = std::f64::consts::FRAC_1_SQRT_2;
        assert!(
            (res.drift_dir.deg45[0] - dx * s2).abs() / dx < 0.05,
            "45° は X 成分の 1/√2: 45={}, X={dx}",
            res.drift_dir.deg45[0]
        );
        assert!(
            (res.story_peak_drift[0] - dx).abs() / dx < 0.05,
            "合成最大は層間Xに一致: max={} X={dx}",
            res.story_peak_drift[0]
        );
        assert!((res.ductility_dir.x[0] - res.story_ductility[0]).abs() < 1e-12);
        assert!(res.ductility_dir.y[0] < res.ductility_dir.x[0]);
    }
}
