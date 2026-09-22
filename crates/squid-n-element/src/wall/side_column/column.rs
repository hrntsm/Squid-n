//! 側柱の要素本体（面内両端ピンの柱）。
//!
//! 静的縮約による剛性計算と `ElementBehavior` 実装、および解放曲げ面を表す
//! `ReleaseAxis`。

use crate::behavior::{Ctx, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::frame::beam::BeamElement;
use crate::transform::LocalFrame;
use smallvec::SmallVec;

/// 解放する局所曲げ面（回転自由度）。
///
/// - `LocalY`: 局所 y 軸回りの回転（ry, 要素ローカル自由度 4・10）を解放。
///   曲げ面は局所 x-z 面（たわみ方向 = 局所 z 軸）。
/// - `LocalZ`: 局所 z 軸回りの回転（rz, 要素ローカル自由度 5・11）を解放。
///   曲げ面は局所 x-y 面（たわみ方向 = 局所 y 軸）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ReleaseAxis {
    LocalY,
    LocalZ,
    /// 柱の局所 y・z 成分で表した解放回転軸の単位ベクトル。
    LocalDirection([f64; 2]),
}

/// 壁面内曲げに対応する両端回転を静縮約した側柱。
pub struct InPlaneReleasedColumn {
    pub(super) inner: BeamElement,
    release_axis: ReleaseAxis,
}

impl InPlaneReleasedColumn {
    pub fn new(inner: BeamElement, release_axis: ReleaseAxis) -> Self {
        Self {
            inner,
            release_axis,
        }
    }

    /// 壁法線を回転軸とする両端回転を内部自由度として静縮約する。
    fn released_local_stiffness(&self) -> LocalMat {
        let [ny, nz] = match self.release_axis {
            ReleaseAxis::LocalY => [1.0, 0.0],
            ReleaseAxis::LocalZ => [0.0, 1.0],
            ReleaseAxis::LocalDirection(n) => n,
        };
        let frame = LocalFrame {
            rot: [[1.0, 0.0, 0.0], [0.0, ny, nz], [0.0, -nz, ny]],
        };
        let inverse = LocalFrame {
            rot: std::array::from_fn(|i| std::array::from_fn(|j| frame.rot[j][i])),
        };
        let k = inverse.to_global(&self.inner.local_stiffness());
        let condensed = crate::frame::prismatic::condense_end_releases(&k, &[(4, 0.0), (10, 0.0)])
            .unwrap_or_else(|| {
                panic!(
                    "壁柱の端部解放剛性を縮約できません: Kbb が特異です（解放条件を確認してください）"
                )
            });
        frame.to_global(&condensed)
    }

    fn released_local_mass(&self, opt: MassOption) -> LocalMat {
        let [ny, nz] = match self.release_axis {
            ReleaseAxis::LocalY => [1.0, 0.0],
            ReleaseAxis::LocalZ => [0.0, 1.0],
            ReleaseAxis::LocalDirection(n) => n,
        };
        let frame = LocalFrame {
            rot: [[1.0, 0.0, 0.0], [0.0, ny, nz], [0.0, -nz, ny]],
        };
        let inverse = LocalFrame {
            rot: std::array::from_fn(|i| std::array::from_fn(|j| frame.rot[j][i])),
        };
        let mass = self.inner.axis.to_local(&self.inner.mass_matrix(opt));
        let mass = inverse.to_global(&mass);
        let stiffness = inverse.to_global(&self.inner.local_stiffness());
        let condensed = crate::frame::prismatic::condense_end_releases_with_mass(
            &stiffness,
            &mass,
            self.inner.mass_properties,
            0.0,
            0.0,
            &[(4, 0.0), (10, 0.0)],
        )
        .unwrap_or_else(|| {
            panic!(
                "壁柱の端部解放質量を縮約できません: Kbb が特異です（解放条件を確認してください）"
            )
        });
        frame.to_global(&condensed)
    }

    /// 縮約後の局所剛性を用いた断面力の復元。
    fn recover_forces_released(
        &self,
        u_elem_global: &[f64; 12],
    ) -> crate::frame::beam::MemberForces {
        let u_local = self.inner.axis.rotate_to_local(u_elem_global);
        let k_local = self.released_local_stiffness();
        let mut f_local = [0.0; 12];
        for (i, fi) in f_local.iter_mut().enumerate() {
            let mut s = 0.0;
            for (j, &uj) in u_local.iter().enumerate() {
                s += k_local.get(i, j) * uj;
            }
            *fi = s;
        }

        crate::frame::beam::member_forces_from_end_forces(
            &f_local,
            self.inner.length,
            &self.inner.eval_sections,
        )
    }
}

crate::behavior::forward_element_behavior!(InPlaneReleasedColumn, inner, {
    n_dof: forward,
    global_dofs: forward,
    tangent_stiffness: custom,
    internal_force: custom,
    update_state: forward,
    mass_matrix: custom,
    recover_forces: custom,
    state_member_forces: custom,
    geometric_stiffness: forward,
    snapshot_state: forward,
    restore_state: forward,
    commit_state: forward,
    revert_state: forward,
    serialize_checkpoint: forward,
    deserialize_checkpoint: forward,
    panel_moments_from: forward,
    ductility_probe: forward,
    fiber_section_states: forward,
    end_spring_rotations: forward,
    set_time_step: forward,
}, custom {
    fn tangent_stiffness(&self, _ctx: &Ctx) -> LocalMat {
        self.inner.axis.to_global(&self.released_local_stiffness())
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        match opt {
            MassOption::Lumped => self.inner.mass_matrix(opt),
            MassOption::Consistent => self.inner.axis.to_global(&self.released_local_mass(opt)),
        }
    }

    fn internal_force(&self, _ctx: &Ctx) -> LocalVec {
        let k = self.inner.axis.to_global(&self.released_local_stiffness());
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        for i in 0..12 {
            let mut s = 0.0;
            for j in 0..12 {
                s += k.get(i, j) * self.inner.trial_disp[j];
            }
            f.data[i] = s;
        }
        f
    }

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::frame::beam::MemberForces> {
        if u_elem.len() < 12 {
            return None;
        }
        let mut arr = [0.0; 12];
        arr.copy_from_slice(&u_elem[..12]);
        Some(self.recover_forces_released(&arr))
    }

    /// 蓄積した trial 変位から復元する。
    fn state_member_forces(&self, _ctx: &Ctx) -> Option<crate::frame::beam::MemberForces> {
        Some(self.recover_forces_released(&self.inner.trial_disp))
    }
});
