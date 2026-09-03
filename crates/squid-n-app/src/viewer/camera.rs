//! 3Dカメラ（クォータニオン回転・ターンテーブル操作）。
//!
//! `viewer` ハブからの構造分割。アルゴリズム変更は行わない。

// ===== クォータニオン（3Dカメラ回転用, [w, x, y, z]）=====
// 合成（q_axis_angle / q_mul / q_norm）は CameraState 内部だけが使う。
// 回転の適用 q_rotate は投影・ViewCube・構面ビューからも呼ぶ。
type Quat = [f32; 4];

/// 軸 `axis`（正規化済み想定）まわり `ang` ラジアンの回転クォータニオン。
fn q_axis_angle(axis: [f32; 3], ang: f32) -> Quat {
    let h = ang * 0.5;
    let s = h.sin();
    [h.cos(), axis[0] * s, axis[1] * s, axis[2] * s]
}

/// クォータニオン積 a⊗b。
fn q_mul(a: Quat, b: Quat) -> Quat {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// 正規化（数値誤差の累積を抑える）。
fn q_norm(q: Quat) -> Quat {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if n < 1e-9 {
        [1.0, 0.0, 0.0, 0.0]
    } else {
        [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
    }
}

/// ベクトル v をクォータニオン q で回転する。
pub(super) fn q_rotate(q: Quat, v: [f32; 3]) -> [f32; 3] {
    let qv = [q[1], q[2], q[3]];
    let t = [
        2.0 * (qv[1] * v[2] - qv[2] * v[1]),
        2.0 * (qv[2] * v[0] - qv[0] * v[2]),
        2.0 * (qv[0] * v[1] - qv[1] * v[0]),
    ];
    [
        v[0] + q[0] * t[0] + (qv[1] * t[2] - qv[2] * t[1]),
        v[1] + q[0] * t[1] + (qv[2] * t[0] - qv[0] * t[2]),
        v[2] + q[0] * t[2] + (qv[0] * t[1] - qv[1] * t[0]),
    ]
}

/// 3D→2D 投影（§3-2: ターンテーブル回転 + 正射影）。
///
/// 構造モデルは実寸比が意味を持つため、§3-2 の「各軸を [-1,1] に正規化」は採らず、
/// 全軸一様スケールで投影してプロポーションを保持する。
/// ビュー軸は X=右・Y=上・Z=手前。
///
/// 回転はターンテーブル方式: 水平ドラッグ＝ワールド Z 軸（鉛直軸）まわりの旋回、
/// 垂直ドラッグ＝画面 X 軸まわりの俯仰。ロールが発生しないため、建物の鉛直軸は
/// 常に画面上で縦に保たれる（自由回転のアークボールはロールが蓄積し視点が傾く）。
#[derive(Clone)]
pub struct CameraState {
    /// 回転（クォータニオン）。`yaw`/`pitch` から導出したキャッシュ
    pub(crate) rot: [f32; 4],
    /// ワールド Z 軸まわりの旋回角 [rad]
    pub(crate) yaw: f32,
    /// 画面 X 軸まわりの俯仰角 [rad]。0=真上（平面図）〜 -π/2=正面 〜 -π=真下
    pub(crate) pitch: f32,
    /// 画面パン（px）
    pub(crate) pan: [f32; 2],
    /// ズーム倍率（§3-2: 既定 3.0、範囲 0.5–10.0）
    pub(crate) zoom: f32,
}

impl Default for CameraState {
    fn default() -> Self {
        // 45° の斜めビュー（平面を 45° 振ってから 45° 見下ろす）。
        // XY 平面のグリッドが斜めから見えるようにする。
        let yaw = std::f32::consts::FRAC_PI_4;
        let pitch = -std::f32::consts::FRAC_PI_4;
        Self {
            rot: Self::rot_from(yaw, pitch),
            yaw,
            pitch,
            pan: [0.0, 0.0],
            zoom: 3.0,
        }
    }
}

impl CameraState {
    /// ドラッグ回転の感度 [rad/px]
    const ROT_SENS: f32 = 0.005;
    /// スクロール 1 単位あたりのズーム変化率。
    const ZOOM_SENS: f32 = 0.01;
    /// ズーム倍率の下限・上限（UI設計 §3-2）。
    const ZOOM_MIN: f32 = 0.5;
    const ZOOM_MAX: f32 = 10.0;

    /// `yaw`/`pitch` からビュー回転を導出する（旋回→俯仰の順で合成）。
    fn rot_from(yaw: f32, pitch: f32) -> Quat {
        q_norm(q_mul(
            q_axis_angle([1.0, 0.0, 0.0], pitch),
            q_axis_angle([0.0, 0.0, 1.0], yaw),
        ))
    }

    /// ドラッグ量（px）によるターンテーブル回転。
    /// 俯仰は真上（0）〜真下（-π）でクランプし、天地の反転を防ぐ。
    pub(crate) fn turntable_drag(&mut self, dx_px: f32, dy_px: f32) {
        self.yaw += dx_px * Self::ROT_SENS;
        self.pitch = (self.pitch + dy_px * Self::ROT_SENS).clamp(-std::f32::consts::PI, 0.0);
        self.rot = Self::rot_from(self.yaw, self.pitch);
    }

    /// 描画領域のポインタ入力（ドラッグ・スクロール・ピンチ）をカメラへ反映する。
    ///
    /// 操作の割り当ては 3D を描く全てのビュー（ビューア・M-N 相関図・ヒンジ詳細）で
    /// 共通とする。**同じ 3D をどのパネルで触っても同じ操作感になることが利用者から
    /// 見た要件**であり、感度やクランプ範囲がパネルごとに割れてはならない。
    ///
    /// - 左ドラッグ＝ターンテーブル回転（UI設計 §3-2）
    /// - 右ドラッグ＝パン（規約外の補助操作）
    /// - スクロール／トラックパッドのピンチ＝ズーム
    ///
    /// `allow_rotate` が偽のときは左ドラッグもパンに割り当てる。構面表示中に回転を
    /// 許すと正対が崩れ、構面内に描く基準線も傾くためで、2D CAD の操作に揃える。
    ///
    /// ズームは**ポインタが描画領域上にあるときだけ**反応させる。同一画面に複数の
    /// ビューやプロットが並ぶため、スクロールが背後のビューまで届くと操作が混線する。
    /// `hovered()` は手前のレイヤー（ヒンジ詳細などの `egui::Window`）による遮蔽も
    /// 考慮するため、ポップアップが重なっている間は手前のビューだけが反応する。
    pub(crate) fn apply_pointer_input(
        &mut self,
        ui: &egui::Ui,
        response: &egui::Response,
        allow_rotate: bool,
    ) {
        if response.dragged_by(egui::PointerButton::Primary) {
            let d = response.drag_delta();
            if allow_rotate {
                self.turntable_drag(d.x, d.y);
            } else {
                self.pan[0] += d.x;
                self.pan[1] += d.y;
            }
        }
        if response.dragged_by(egui::PointerButton::Secondary) {
            let d = response.drag_delta();
            self.pan[0] += d.x;
            self.pan[1] += d.y;
        }
        if response.hovered() {
            let scroll_y = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll_y != 0.0 {
                self.zoom *= 1.0 + scroll_y * Self::ZOOM_SENS;
            }
            let pinch = ui.input(|i| i.zoom_delta());
            if pinch != 1.0 {
                self.zoom *= pinch;
            }
        }
        self.zoom = self.zoom.clamp(Self::ZOOM_MIN, Self::ZOOM_MAX);
    }

    /// 視点方向 `d`（ワールド座標、原点から視点位置へ向かうベクトル）へ即時スナップする。
    /// ViewCube の面・コーナークリックから呼ばれる。`d` が鉛直（真上/真下）の場合、
    /// 旋回角は方位角から定まらないため 0 とし、X 軸が画面右を向く正対の平面ビューにする。
    pub(crate) fn snap_to_direction(&mut self, d: [f32; 3]) {
        let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if n < 1e-6 {
            return;
        }
        let (dx, dy, dz) = (d[0] / n, d[1] / n, d[2] / n);
        // ターンテーブル rot = R_x(pitch)∘R_z(yaw) で q_rotate(rot, d) = [0,0,1]（視線正面）
        // となる角度: yaw は方位角 φ=atan2(dy,dx) から、pitch は仰角から定まる。
        self.yaw = if dx.abs() > 1e-6 || dy.abs() > 1e-6 {
            -std::f32::consts::FRAC_PI_2 - dy.atan2(dx)
        } else {
            0.0
        };
        self.pitch = dz.clamp(-1.0, 1.0).asin() - std::f32::consts::FRAC_PI_2;
        self.rot = Self::rot_from(self.yaw, self.pitch);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ワールド Z 軸（鉛直軸）のビュー空間での向き。
    /// 画面上で縦 ⇔ x 成分が 0、かつ上向き ⇔ y 成分が非負（project は y を反転して描画する）。
    fn world_z_in_view(cam: &CameraState) -> [f32; 3] {
        q_rotate(cam.rot, [0.0, 0.0, 1.0])
    }

    #[test]
    fn 既定ビューで鉛直軸は画面上で縦() {
        let cam = CameraState::default();
        let z = world_z_in_view(&cam);
        assert!(z[0].abs() < 1e-5, "Z 軸が画面上で傾いている: {z:?}");
        assert!(z[1] >= -1e-6, "Z 軸が画面上で下向き: {z:?}");
    }

    #[test]
    fn ドラッグ回転を繰り返しても鉛直軸は傾かない() {
        // アークボール時代の不具合: 斜めドラッグの繰り返しでロールが蓄積し、
        // 鉛直軸が画面上で斜めに傾いていた。ターンテーブルでは起きないことを確認する。
        let mut cam = CameraState::default();
        let drags = [
            (30.0, -20.0),
            (-50.0, 40.0),
            (100.0, 100.0),
            (-15.0, -80.0),
            (200.0, 5.0),
            (-3.0, 60.0),
        ];
        for _ in 0..50 {
            for (dx, dy) in drags {
                cam.turntable_drag(dx, dy);
                let z = world_z_in_view(&cam);
                assert!(z[0].abs() < 1e-4, "Z 軸が画面上で傾いた: {z:?}");
                assert!(z[1] >= -1e-4, "Z 軸が画面上で下向きになった: {z:?}");
            }
        }
    }

    #[test]
    fn 俯仰は真上と真下でクランプされる() {
        let mut cam = CameraState::default();
        cam.turntable_drag(0.0, 1e6); // 大きく下ドラッグ → 真上（平面図）で停止
        assert!((cam.pitch - 0.0).abs() < 1e-6);
        cam.turntable_drag(0.0, -1e6); // 大きく上ドラッグ → 真下で停止
        assert!((cam.pitch + std::f32::consts::PI).abs() < 1e-6);
    }
}
