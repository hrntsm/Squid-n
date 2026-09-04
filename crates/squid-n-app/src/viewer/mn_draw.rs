//! N-M 相関曲面の 3D 表示に共通する描画プリミティブ。
//!
//! 相関曲面を 3D で描く画面は 2 つある。断面詳細ビュー（`crate::mn_view`）は 3 つの
//! 降伏モデルの曲面を重ねて軸力スライス平面を添え、ヒンジ詳細ウィンドウ
//! （[`crate::viewer::hinge`]）は 1 つの曲面に増分解析の応答経路を重ねる。
//!
//! 重ねるものは違うが、**曲面そのものの描き方は同じ**である。格子解像度・
//! 正規化基準・投影スケール・座標軸・ワイヤーフレームの張り方がそれで、
//! 以前は両画面がそれぞれ自前に持っていた（`draw_axes` と `draw_mn_axes` は
//! 本体がバイト単位で一致し、doc コメント自身が「同じ考え方の自己完結版」と
//! 重複を認めていた）。その定型だけをここへ集める。
//!
//! **画面ごとの差異は意図的に呼び出し側へ残す。** 何を重ねるか（スライス平面か
//! 応答経路か）、曲面をどの色・どの不透明度で描くかは、その図が何を見せたいかを
//! 表す情報である。ここへオプションとして畳み込むと、`show_slice = false` の
//! 意味を読むためにこのモジュールを開くことになり、画面 1 つずつの読みやすさが
//! 落ちる。

use crate::theme;
use crate::viewer::{project, CameraState};
use squid_n_section::mn_surface::MnSurface;

/// N-M 相関曲面の格子解像度（経線方向・周方向）。
///
/// 2 つの画面で値を揃え、断面詳細ビューとヒンジ詳細で同じ精度の曲面を描く。
pub(crate) const N_ALPHA: usize = 24;
pub(crate) const N_BETA: usize = 48;

/// 正規化ワールド座標を画面へ落とすための投影設定。
///
/// カメラ・スケール・画面中心は常に 3 つ揃って必要になるため、1 つの値にまとめる。
#[derive(Clone, Copy)]
pub(crate) struct MnView<'a> {
    cam: &'a CameraState,
    scale: f32,
    screen_center: [f32; 2],
}

impl<'a> MnView<'a> {
    /// 描画領域とカメラから投影設定を作る。
    ///
    /// 正規化世界座標はおよそ ±1.0〜1.3 に収まる。短辺の 0.32 倍を基準スケールとし、
    /// 既定ズーム 3.0 で画面の大部分を占めるようにする
    /// （`viewer_panel` と同じ考え方）。
    pub(crate) fn new(rect: &egui::Rect, cam: &'a CameraState) -> Self {
        let min_dim = rect.width().min(rect.height());
        Self {
            cam,
            scale: 0.32 * min_dim * (cam.zoom / 3.0),
            screen_center: [rect.center().x, rect.center().y],
        }
    }

    /// 正規化ワールド座標 `[My_n, Mz_n, N_n]` を画面座標へ投影する。
    pub(crate) fn project(&self, p: [f64; 3]) -> egui::Pos2 {
        let s = project(p, [0.0; 3], self.cam, self.scale, self.screen_center);
        egui::pos2(s[0], s[1])
    }

    /// 曲面の格子点 `[N, My, Mz]` を正規化してから投影する。
    fn project_grid(&self, g: &[f64; 3], refs: [f64; 3]) -> egui::Pos2 {
        self.project(to_world(g, refs))
    }
}

/// 曲面の耐力から正規化基準 `[My 基準, Mz 基準, N 基準]` を作る。
///
/// 3 成分とも下限 1.0 でゼロ割を防ぐ。軸力は圧縮側（負値）と引張側の絶対値が
/// 大きい方を基準に採り、曲面が正規化座標のおよそ ±1.0 に収まるようにする。
pub(crate) fn surface_refs(surf: &MnSurface) -> [f64; 3] {
    [
        surf.mp_y.abs().max(1.0),
        surf.mp_z.abs().max(1.0),
        surf.n_comp.abs().max(surf.n_tens).max(1.0),
    ]
}

/// 曲面の格子点 `[N, My, Mz]` を正規化ワールド座標 `[My_n, Mz_n, N_n]` へ変換する。
///
/// X=My 基準、Y=Mz 基準、Z=N 基準。N を第 3 成分へ置いて画面の上下軸に対応させる。
fn to_world(g: &[f64; 3], refs: [f64; 3]) -> [f64; 3] {
    [g[1] / refs[0], g[2] / refs[1], g[0] / refs[2]]
}

/// 原点から ±1.3 の座標軸線とラベル「My」「Mz」「N」を描く。
pub(crate) fn draw_axes(painter: &egui::Painter, view: &MnView<'_>) {
    const EXT: f64 = 1.3;
    let axes: [([f64; 3], egui::Color32, &str); 3] = [
        ([EXT, 0.0, 0.0], theme::AXIS_X, "My"),
        ([0.0, EXT, 0.0], theme::AXIS_Y, "Mz"),
        ([0.0, 0.0, EXT], theme::AXIS_Z, "N"),
    ];
    for (dir, color, label) in axes {
        let neg = [-dir[0], -dir[1], -dir[2]];
        painter.line_segment(
            [view.project(neg), view.project(dir)],
            egui::Stroke::new(1.5_f32, color),
        );
        painter.text(
            view.project(dir),
            egui::Align2::LEFT_BOTTOM,
            label,
            egui::FontId::proportional(13.0),
            color,
        );
    }
}

/// 曲面をワイヤーフレーム（周方向・経線方向の格子線）で描画する。
///
/// `alpha` は線の不透明度。曲面に何を重ねるかで見やすい濃さが変わるため、
/// 呼び出し側が決める（モジュール doc の「差異は呼び出し側へ残す」を参照）。
pub(crate) fn draw_wireframe(
    painter: &egui::Painter,
    surf: &MnSurface,
    refs: [f64; 3],
    view: &MnView<'_>,
    color: egui::Color32,
    alpha: u8,
) {
    let stroke = egui::Stroke::new(1.0_f32, theme::translucent(color, alpha));

    let n_beta = match surf.grid.first() {
        Some(row) if !row.is_empty() => row.len(),
        _ => return,
    };

    // 周方向（各経線上、j=n_beta-1 と j=0 が接続する閉曲線）
    for row in &surf.grid {
        for j in 0..n_beta {
            let a = view.project_grid(&row[j], refs);
            let b = view.project_grid(&row[(j + 1) % n_beta], refs);
            painter.line_segment([a, b], stroke);
        }
    }
    // 経線方向（引張極→圧縮極）
    for j in 0..n_beta {
        for i in 0..surf.grid.len().saturating_sub(1) {
            let a = view.project_grid(&surf.grid[i][j], refs);
            let b = view.project_grid(&surf.grid[i + 1][j], refs);
            painter.line_segment([a, b], stroke);
        }
    }
}

/// 3D 領域下端のカメラ操作ヒント。
pub(crate) fn draw_camera_hint(ui: &mut egui::Ui) {
    ui.add(egui::Label::new(
        egui::RichText::new("左ドラッグ:回転 / 右ドラッグ:移動 / スクロール:ズーム").size(11.0),
    ));
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_section::mn_surface::YieldModelKind;

    fn surface(n_comp: f64, n_tens: f64, mp_y: f64, mp_z: f64) -> MnSurface {
        MnSurface {
            kind: YieldModelKind::MultiFiber,
            grid: Vec::new(),
            n_comp,
            n_tens,
            mp_y,
            mp_z,
        }
    }

    #[test]
    fn surface_refs_は圧縮側と引張側の絶対値が大きい方を軸力基準に採る() {
        // 圧縮軸耐力は負値で持つ。絶対値で比べて大きい方を採る。
        let s = surface(-5000.0, 1200.0, 300.0, 800.0);
        assert_eq!(surface_refs(&s), [300.0, 800.0, 5000.0]);

        let s = surface(-900.0, 1200.0, 300.0, 800.0);
        assert_eq!(surface_refs(&s), [300.0, 800.0, 1200.0]);
    }

    #[test]
    fn surface_refs_は耐力が0でも1_0を下回らない() {
        // ゼロ割の防止。曲面が縮退していても投影が発散しない。
        assert_eq!(surface_refs(&surface(0.0, 0.0, 0.0, 0.0)), [1.0, 1.0, 1.0]);
    }

    #[test]
    fn to_world_は格子点の並びを画面の並びへ入れ替える() {
        // 格子点は [N, My, Mz]、ワールドは [My_n, Mz_n, N_n]。
        // N を第 3 成分へ置いて画面の上下軸に対応させる。
        let w = to_world(&[100.0, 200.0, 400.0], [2.0, 4.0, 5.0]);
        assert_eq!(w, [100.0, 100.0, 20.0]);
    }

    #[test]
    fn view_scaleは短辺基準で既定ズーム3_0のとき短辺の0_32倍になる() {
        let rect = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(400.0, 250.0));
        let cam = CameraState::default();
        assert!((cam.zoom - 3.0).abs() < 1e-6, "既定ズームは 3.0");
        let view = MnView::new(&rect, &cam);
        // 短辺 250 の 0.32 倍。
        assert!((view.scale - 80.0).abs() < 1e-4, "scale = {}", view.scale);
    }
}
