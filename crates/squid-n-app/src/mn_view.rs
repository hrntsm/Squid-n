//! 3次元 M-N 相関曲面（降伏曲面）ビュー。
//!
//! 部材端の降伏判定に用いるモデル化手法（端部単純降伏バネ／マルチスプリング／
//! マルチファイバー）ごとの N–My–Mz 相関曲面の違いを、3D ワイヤーフレームと
//! 任意軸力位置での My–Mz スライス（2D 相関曲線）で比較表示する。
//!
//! 下段の2Dプロットは My–Mz 相関曲線に加え、塑性化領域長さ Lp を考慮した
//! 材端 M-θ 骨格曲線（材端剛塑性ばねモデルと適合するファイバーモデル化の
//! 考え方）にも切り替えられる。
//!
//! 計算コアは `squid_n_section::mn_surface`（既存実装）。本ファイルはその結果を
//! 可視化するのみで、力学的な計算ロジックは持たない。

use crate::app::App;
use crate::theme;
use crate::viewer::{project, CameraState};
use squid_n_core::section_shape::SectionShape;
use squid_n_core::units::to_display::{force_kn, moment_kn_m};
use squid_n_section::mn_surface::{
    build_simple_spring_surface, build_surface, m_phi_curve, m_theta_curve, plastic_fibers,
    slice_at_n, MnSurface, PlasticFiber, StrengthParams, YieldModelKind,
};

/// 曲面の格子解像度（経線方向・周方向）。
const N_ALPHA: usize = 24;
const N_BETA: usize = 48;
/// スライス曲線の分割数。
const SLICE_PTS: usize = 64;

/// モデル化手法ごとの表示色（§3 データビジュアライゼーション配色）。
fn model_color(kind: YieldModelKind) -> egui::Color32 {
    match kind {
        YieldModelKind::SimpleSpring => theme::PARETO_RED,
        YieldModelKind::MultiSpring => theme::GOOD_GREEN,
        YieldModelKind::MultiFiber => theme::DATA_BLUE,
    }
}

/// 断面・材料強度から算定した曲面/ファイバのキャッシュ。
/// `section_idx` と `strength` が前回と同じ間は再利用する。
struct MnCache {
    section_idx: usize,
    strength: StrengthParams,
    simple: MnSurface,
    ms: MnSurface,
    fiber: MnSurface,
    /// マルチスプリング用バネ配置（軸力スライス計算に使用）
    ms_fibers: Vec<PlasticFiber>,
    /// マルチファイバー用ファイバ配置（軸力スライス計算・単純バネの耐力算定に使用）
    fiber_fibers: Vec<PlasticFiber>,
}

/// 下段2Dプロットの表示モード。
#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub enum SlicePlotMode {
    /// 軸力一定での My-Mz 相関曲線
    #[default]
    MyMz,
    /// 塑性化領域長さ Lp を考慮した材端 M-θ 骨格曲線
    MTheta,
}

/// M-θ サブキャッシュのキー。前回と同じ間は `m_phi_curve` の再計算（数十ms）を
/// 省略する。`MnCache` は section/strength の変化で作り直されるため、そこに
/// ぶら下げると `ensure_cache` 後に可変参照が必要になり借用が競合する。
/// そのため `MnViewState` 側に独立したサブキャッシュとして持たせ、
/// キーに section_idx・strength も含めて一致判定する。
#[derive(Clone, Copy, PartialEq)]
struct MThetaKey {
    section_idx: usize,
    strength: StrengthParams,
    n_target: f64,
    lp: f64,
    span: f64,
    bend_dir_z: bool,
}

/// M-θ プロット用の計算結果（モデル別の [θ(rad), M(N·mm)] 点列。軸力範囲外は `None`）。
struct MThetaData {
    fiber: Option<Vec<[f64; 2]>>,
    ms: Option<Vec<[f64; 2]>>,
    /// 単純降伏バネ（材端剛塑性ばね）: (0,0)→(θy,Mlim)→(θmax,Mlim) の折れ線
    simple: Option<Vec<[f64; 2]>>,
}

/// M-N 相関曲面ビューの状態（`App` が保持する）。
pub struct MnViewState {
    /// `app.core.model.sections` のインデックス
    pub section_idx: usize,
    pub strength: StrengthParams,
    pub show_simple: bool,
    pub show_ms: bool,
    pub show_fiber: bool,
    /// スライス軸力の比率。-1.0(圧縮耐力)〜+1.0(引張耐力)。
    pub n_ratio: f64,
    /// 下段2Dプロットの表示モード（My-Mz相関 / M-θ骨格）。
    pub slice_mode: SlicePlotMode,
    /// 塑性化領域長さ Lp [mm]。0.0 は未設定扱いで、断面選択時に断面せい D の
    /// 0.5倍を自動設定する（断面切替のたびに再設定する。`ensure_cache` 参照）。
    pub lp: f64,
    /// 部材内法スパン L [mm]（M-θ 換算の弾性回転項に使用）。
    pub span: f64,
    /// 曲げ方向。false=Myまわり(既定) / true=Mzまわり。
    pub bend_dir_z: bool,
    /// 3D ビュー用カメラ（`viewer::CameraState` を再利用し、既存3Dビューと
    /// 同じ操作感を持たせる）
    pub camera: CameraState,
    cache: Option<MnCache>,
    /// M-θ プロット用サブキャッシュ（`MThetaKey` が前回と同じなら再利用）。
    m_theta_cache: Option<(MThetaKey, MThetaData)>,
}

impl Default for MnViewState {
    fn default() -> Self {
        Self {
            section_idx: 0,
            strength: StrengthParams::default(),
            show_simple: true,
            show_ms: true,
            show_fiber: true,
            n_ratio: 0.0,
            slice_mode: SlicePlotMode::default(),
            lp: 0.0,
            span: 4000.0,
            bend_dir_z: false,
            camera: CameraState::default(),
            cache: None,
            m_theta_cache: None,
        }
    }
}

/// エントリポイント: 左に操作パネル、右に可視化領域（3D + 2Dスライス）。
pub fn mn_surface_panel(ui: &mut egui::Ui, app: &mut App) {
    if app.core.model.sections.is_empty() {
        ui.colored_label(
            theme::GRAY_600,
            "断面が定義されていません。モデルタブの「断面」で断面を追加してください。",
        );
        return;
    }
    if app.ui.scoped.mn_view.section_idx >= app.core.model.sections.len() {
        app.ui.scoped.mn_view.section_idx = 0;
    }

    ui.horizontal(|ui| {
        ui.allocate_ui_with_layout(
            egui::vec2(260.0, ui.available_height()),
            egui::Layout::top_down(egui::Align::Min),
            |ui| {
                egui::ScrollArea::vertical()
                    .id_salt("mn_view_control_panel")
                    .show(ui, |ui| {
                        control_panel(ui, app);
                    });
            },
        );
        ui.separator();
        ui.vertical(|ui| {
            visualization(ui, app);
        });
    });
}

/// 左ペイン: 断面・材料強度・表示切替・軸力スライダー・数値サマリ。
fn control_panel(ui: &mut egui::Ui, app: &mut App) {
    ui.strong("断面");
    let selected_text = app
        .core
        .model
        .sections
        .get(app.ui.scoped.mn_view.section_idx)
        .map(|s| s.name.clone())
        .unwrap_or_default();
    egui::ComboBox::from_id_salt("mn_view_section")
        .selected_text(selected_text)
        .show_ui(ui, |ui| {
            for (i, sec) in app.core.model.sections.iter().enumerate() {
                ui.selectable_value(&mut app.ui.scoped.mn_view.section_idx, i, &sec.name);
            }
        });

    let shape = app
        .core
        .model
        .sections
        .get(app.ui.scoped.mn_view.section_idx)
        .and_then(|s| s.shape.as_ref());
    let is_rc = matches!(
        shape,
        Some(SectionShape::RcRect { .. } | SectionShape::RcCircle { .. })
    );
    let is_steel = shape.is_some() && !is_rc;

    ui.add_space(8.0);
    ui.strong("材料強度 [N/mm²]");
    // RC断面は鉄筋fy/コンクリートFcのみ、鋼断面は鋼材fyのみを表示する
    // （断面形状未定義の場合は種別が判別できないため両方表示しておく）。
    if is_steel || shape.is_none() {
        ui.horizontal(|ui| {
            ui.label("鋼材 fy:");
            ui.add(
                egui::DragValue::new(&mut app.ui.scoped.mn_view.strength.steel_fy)
                    .speed(1.0)
                    .range(1.0..=1000.0),
            );
        });
    }
    if is_rc || shape.is_none() {
        ui.horizontal(|ui| {
            ui.label("鉄筋 fy:");
            ui.add(
                egui::DragValue::new(&mut app.ui.scoped.mn_view.strength.rebar_fy)
                    .speed(1.0)
                    .range(1.0..=1000.0),
            );
        });
        ui.horizontal(|ui| {
            ui.label("コンクリート Fc:");
            ui.add(
                egui::DragValue::new(&mut app.ui.scoped.mn_view.strength.concrete_fc)
                    .speed(0.5)
                    .range(1.0..=100.0),
            );
        });
    }

    ui.add_space(8.0);
    ui.strong("表示モデル");
    ui.horizontal(|ui| {
        ui.colored_label(model_color(YieldModelKind::SimpleSpring), "■");
        ui.checkbox(
            &mut app.ui.scoped.mn_view.show_simple,
            YieldModelKind::SimpleSpring.label(),
        );
    });
    ui.horizontal(|ui| {
        ui.colored_label(model_color(YieldModelKind::MultiSpring), "■");
        ui.checkbox(
            &mut app.ui.scoped.mn_view.show_ms,
            YieldModelKind::MultiSpring.label(),
        );
    });
    ui.horizontal(|ui| {
        ui.colored_label(model_color(YieldModelKind::MultiFiber), "■");
        ui.checkbox(
            &mut app.ui.scoped.mn_view.show_fiber,
            YieldModelKind::MultiFiber.label(),
        );
    });

    ui.add_space(8.0);
    ui.strong("スライス軸力 N/Nmax");
    ui.add(egui::Slider::new(
        &mut app.ui.scoped.mn_view.n_ratio,
        -1.0..=1.0,
    ));

    ui.add_space(8.0);
    ui.strong("2Dプロット");
    ui.horizontal(|ui| {
        let sel_mymz = app.ui.scoped.mn_view.slice_mode == SlicePlotMode::MyMz;
        let sel_mtheta = app.ui.scoped.mn_view.slice_mode == SlicePlotMode::MTheta;
        if ui.selectable_label(sel_mymz, "My-Mz相関").clicked() {
            app.ui.scoped.mn_view.slice_mode = SlicePlotMode::MyMz;
        }
        if ui
            .selectable_label(sel_mtheta, "M-θ骨格（塑性化域）")
            .clicked()
        {
            app.ui.scoped.mn_view.slice_mode = SlicePlotMode::MTheta;
        }
    });
    if app.ui.scoped.mn_view.slice_mode == SlicePlotMode::MTheta {
        ui.horizontal(|ui| {
            ui.label("塑性化領域長さ Lp [mm]:");
            ui.add(
                egui::DragValue::new(&mut app.ui.scoped.mn_view.lp)
                    .speed(10.0)
                    .range(1.0..=5000.0),
            );
        });
        ui.horizontal(|ui| {
            ui.label("内法スパン L [mm]:");
            ui.add(
                egui::DragValue::new(&mut app.ui.scoped.mn_view.span)
                    .speed(50.0)
                    .range(100.0..=30000.0),
            );
        });
        ui.horizontal(|ui| {
            ui.label("曲げ方向:");
            let sel_my = !app.ui.scoped.mn_view.bend_dir_z;
            let sel_mz = app.ui.scoped.mn_view.bend_dir_z;
            if ui.selectable_label(sel_my, "Myまわり").clicked() {
                app.ui.scoped.mn_view.bend_dir_z = false;
            }
            if ui.selectable_label(sel_mz, "Mzまわり").clicked() {
                app.ui.scoped.mn_view.bend_dir_z = true;
            }
        });
    }

    ui.add_space(8.0);
    ui.strong("耐力サマリ");
    if let Some(shape) = shape.cloned() {
        let section_idx = app.ui.scoped.mn_view.section_idx;
        ensure_cache(&mut app.ui.scoped.mn_view, section_idx, &shape);
        if let Some(cache) = &app.ui.scoped.mn_view.cache {
            summary_table(ui, cache);
        }
        if app.ui.scoped.mn_view.slice_mode == SlicePlotMode::MTheta {
            ui.add_space(4.0);
            ui.add(egui::Label::new(
                egui::RichText::new(
                    "M-θ は逆対称曲げ・反曲点中央・端部塑性化領域 Lp の仮定による骨格曲線",
                )
                .size(11.0)
                .color(theme::GRAY_600),
            ));
        }
    } else {
        ui.colored_label(theme::GRAY_600, "断面形状が未定義です。");
    }
}

/// 各モデルの Nc/Nt/Mpy/Mpz を並べた数値サマリ表。
fn summary_table(ui: &mut egui::Ui, cache: &MnCache) {
    egui::Grid::new("mn_view_summary")
        .num_columns(5)
        .striped(true)
        .show(ui, |ui| {
            ui.strong("モデル");
            ui.strong("Nc[kN]");
            ui.strong("Nt[kN]");
            ui.strong("Mpy[kN·m]");
            ui.strong("Mpz[kN·m]");
            ui.end_row();

            for surf in [&cache.simple, &cache.ms, &cache.fiber] {
                ui.colored_label(model_color(surf.kind), surf.kind.label());
                ui.label(format!("{:.1}", force_kn(surf.n_comp)));
                ui.label(format!("{:.1}", force_kn(surf.n_tens)));
                ui.label(format!("{:.1}", moment_kn_m(surf.mp_y)));
                ui.label(format!("{:.1}", moment_kn_m(surf.mp_z)));
                ui.end_row();
            }
        });
}

/// 断面せい D [mm]（Lp 自動設定に用いる）。SteelPipe/RcCircle は径、
/// SteelAngle は leg_a を D とみなす。
fn section_depth(shape: &SectionShape) -> f64 {
    match *shape {
        SectionShape::SteelH { height, .. }
        | SectionShape::SteelBox { height, .. }
        | SectionShape::SteelChannel { height, .. }
        | SectionShape::SteelTee { height, .. } => height,
        SectionShape::SteelAngle { leg_a, .. } => leg_a,
        SectionShape::SteelFlatBar { thick, .. } => thick,
        SectionShape::SteelRoundBar { dia, .. } => dia,
        SectionShape::SteelBuiltH { height, .. } => height,
        SectionShape::SteelLipChannel { height, .. } => height,
        SectionShape::SteelPipe { outer_dia, .. } => outer_dia,
        SectionShape::RcRect { d, .. }
        | SectionShape::RcCircle { d, .. }
        | SectionShape::SrcRect { d, .. } => d,
        SectionShape::CftBox { height, .. } => height,
        SectionShape::CftPipe { outer_dia, .. } => outer_dia,
        SectionShape::RcWall { thickness, .. } | SectionShape::RcSlab { thickness } => thickness,
    }
}

/// キャッシュが古ければ再計算する（`section_idx` または `strength` が変化した場合）。
/// 断面切替時は塑性化領域長さ Lp を新断面の 0.5D へ自動リセットする。
fn ensure_cache(state: &mut MnViewState, section_idx: usize, shape: &SectionShape) {
    let section_changed = match &state.cache {
        Some(c) => c.section_idx != section_idx,
        None => true,
    };
    let stale = match &state.cache {
        Some(c) => c.section_idx != section_idx || c.strength != state.strength,
        None => true,
    };
    if !stale {
        return;
    }

    if section_changed {
        state.lp = 0.5 * section_depth(shape);
    }

    let strength = state.strength;
    // マルチファイバー用の細分割ファイバ配置。単純バネの耐力算定にも流用する
    // （squid_n_section::mn_surface::plastic_fibers の解像度は SimpleSpring/MultiFiber で同一）。
    let fiber_fibers = plastic_fibers(shape, &strength, YieldModelKind::MultiFiber);
    let ms_fibers = plastic_fibers(shape, &strength, YieldModelKind::MultiSpring);

    let fiber = build_surface(&fiber_fibers, YieldModelKind::MultiFiber, N_ALPHA, N_BETA);
    let ms = build_surface(&ms_fibers, YieldModelKind::MultiSpring, N_ALPHA, N_BETA);
    let simple = build_simple_spring_surface(&fiber_fibers, N_ALPHA, N_BETA);

    state.cache = Some(MnCache {
        section_idx,
        strength,
        simple,
        ms,
        fiber,
        ms_fibers,
        fiber_fibers,
    });
}

/// `n_ratio`（-1.0〜1.0）をファイバーモデルの軸耐力基準で実軸力 [N] へ変換する。
fn n_from_ratio(cache: &MnCache, n_ratio: f64) -> f64 {
    if n_ratio >= 0.0 {
        n_ratio * cache.fiber.n_tens
    } else {
        n_ratio * cache.fiber.n_comp.abs()
    }
}

/// M-θ サブキャッシュが古ければ（`key` が前回と異なれば）再計算する。
/// `m_phi_curve` は数十msかかりうるため、毎フレーム呼ばないための入口。
fn ensure_m_theta_cache(
    slot: &mut Option<(MThetaKey, MThetaData)>,
    key: MThetaKey,
    cache: &MnCache,
) {
    if let Some((k, _)) = slot {
        if *k == key {
            return;
        }
    }

    let (ky, kz) = if key.bend_dir_z {
        (0.0, 1.0)
    } else {
        (1.0, 0.0)
    };
    const N_STEPS: usize = 60;

    let fiber_mphi = m_phi_curve(&cache.fiber_fibers, ky, kz, key.n_target, N_STEPS);
    let fiber = fiber_mphi
        .as_ref()
        .map(|c| m_theta_curve(c, key.span, key.lp));

    let ms_mphi = m_phi_curve(&cache.ms_fibers, ky, kz, key.n_target, N_STEPS);
    let ms = ms_mphi.as_ref().map(|c| m_theta_curve(c, key.span, key.lp));

    // 単純降伏バネ（材端剛塑性ばね）: 弾性部材の EI0 はマルチファイバーモデルの
    // MPhiCurve を共用する（弾性部材は共通という前提）。
    let simple = fiber_mphi.as_ref().and_then(|fc| {
        let mp = if key.bend_dir_z {
            cache.simple.mp_z
        } else {
            cache.simple.mp_y
        };
        let n_ref = if key.n_target >= 0.0 {
            cache.simple.n_tens.max(1.0)
        } else {
            cache.simple.n_comp.abs().max(1.0)
        };
        let m_lim = mp * (1.0 - key.n_target.abs() / n_ref);
        if m_lim <= 0.0 {
            return None;
        }
        let ei0 = fc.ei0.max(1.0);
        let theta_y = m_lim * key.span / (6.0 * ei0);
        let theta_max = fiber
            .as_ref()
            .and_then(|f| f.last())
            .map(|p| p[0])
            .unwrap_or(theta_y * 3.0)
            .max(theta_y);
        Some(vec![[0.0, 0.0], [theta_y, m_lim], [theta_max, m_lim]])
    });

    *slot = Some((key, MThetaData { fiber, ms, simple }));
}

/// 右ペイン: 断面が未選択・形状未定義の場合は案内、それ以外は 3D + 2D を描画する。
fn visualization(ui: &mut egui::Ui, app: &mut App) {
    let Some(sec) = app
        .core
        .model
        .sections
        .get(app.ui.scoped.mn_view.section_idx)
    else {
        return;
    };
    let Some(shape) = sec.shape.clone() else {
        ui.colored_label(
            theme::GRAY_600,
            "断面形状が未定義です。断面エディタで形状を設定してください。",
        );
        return;
    };

    let section_idx = app.ui.scoped.mn_view.section_idx;
    ensure_cache(&mut app.ui.scoped.mn_view, section_idx, &shape);
    let Some(cache) = app.ui.scoped.mn_view.cache.as_ref() else {
        return;
    };

    let n_ratio = app.ui.scoped.mn_view.n_ratio;
    let show = [
        app.ui.scoped.mn_view.show_simple,
        app.ui.scoped.mn_view.show_ms,
        app.ui.scoped.mn_view.show_fiber,
    ];
    let n_target = n_from_ratio(cache, n_ratio);

    // --- 3D ワイヤーフレーム（上6割） ---
    let total_h = ui.available_height();
    let (rect, response) = ui.allocate_exact_size(
        egui::vec2(ui.available_width(), (total_h * 0.6).max(80.0)),
        egui::Sense::click_and_drag(),
    );

    let mut cam = app.ui.scoped.mn_view.camera.clone();
    if response.dragged_by(egui::PointerButton::Primary) {
        // ターンテーブル回転（viewer と同じ操作感。N 軸＝Z を画面上で縦に保つ）
        let d = response.drag_delta();
        cam.turntable_drag(d.x, d.y);
    }
    if response.dragged_by(egui::PointerButton::Secondary) {
        let d = response.drag_delta();
        cam.pan[0] += d.x;
        cam.pan[1] += d.y;
    }
    // viewer と異なり同一画面に 2D プロットや操作パネルが並ぶため、
    // ズームは 3D 領域にポインタがあるときのみ反応させる。
    if response.hovered() {
        let scroll_y = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll_y != 0.0 {
            cam.zoom *= 1.0 + scroll_y * 0.01;
        }
        let pinch = ui.input(|i| i.zoom_delta());
        if pinch != 1.0 {
            cam.zoom *= pinch;
        }
    }
    cam.zoom = cam.zoom.clamp(0.5, 10.0);

    draw_3d(ui, &rect, &cam, cache, show, n_target);
    app.ui.scoped.mn_view.camera = cam;

    ui.separator();

    // --- 2D プロット（下4割）: My-Mz相関 または M-θ骨格に切替 ---
    match app.ui.scoped.mn_view.slice_mode {
        SlicePlotMode::MyMz => draw_slice_plot(ui, cache, show, n_target),
        SlicePlotMode::MTheta => {
            let key = MThetaKey {
                section_idx: app.ui.scoped.mn_view.section_idx,
                strength: app.ui.scoped.mn_view.strength,
                n_target,
                lp: app.ui.scoped.mn_view.lp,
                span: app.ui.scoped.mn_view.span,
                bend_dir_z: app.ui.scoped.mn_view.bend_dir_z,
            };
            ensure_m_theta_cache(&mut app.ui.scoped.mn_view.m_theta_cache, key, cache);
            if let Some((_, data)) = &app.ui.scoped.mn_view.m_theta_cache {
                draw_m_theta_plot(ui, data, show);
            }
        }
    }
}

/// 3D 領域の描画本体（ワイヤーフレーム3面・座標軸・スライス平面）。
fn draw_3d(
    ui: &mut egui::Ui,
    rect: &egui::Rect,
    cam: &CameraState,
    cache: &MnCache,
    show: [bool; 3],
    n_target: f64,
) {
    let painter = ui.painter_at(*rect);
    painter.rect_filled(*rect, 0.0, theme::VIEW_BG);
    let screen_center = [rect.center().x, rect.center().y];

    // 正規化基準（ファイバーモデル基準、ゼロ割防止）。
    let n_ref = cache.fiber.n_comp.abs().max(cache.fiber.n_tens).max(1.0);
    let my_ref = cache.fiber.mp_y.abs().max(1.0);
    let mz_ref = cache.fiber.mp_z.abs().max(1.0);
    let refs = [my_ref, mz_ref, n_ref];

    // 正規化世界座標はおよそ ±1.0〜1.3 に収まる。min_dim の 0.32 倍を基準スケールとし、
    // 既定ズーム 3.0 で画面の大部分を占めるようにする（viewer_panel と同様の考え方）。
    let min_dim = rect.width().min(rect.height());
    let scale = 0.32 * min_dim * (cam.zoom / 3.0);

    draw_axes(&painter, cam, scale, screen_center);

    if show[0] {
        draw_wireframe(
            &painter,
            &cache.simple,
            refs,
            cam,
            scale,
            screen_center,
            model_color(YieldModelKind::SimpleSpring),
        );
    }
    if show[1] {
        draw_wireframe(
            &painter,
            &cache.ms,
            refs,
            cam,
            scale,
            screen_center,
            model_color(YieldModelKind::MultiSpring),
        );
    }
    if show[2] {
        draw_wireframe(
            &painter,
            &cache.fiber,
            refs,
            cam,
            scale,
            screen_center,
            model_color(YieldModelKind::MultiFiber),
        );
    }

    draw_slice_plane(&painter, n_target, n_ref, cam, scale, screen_center);

    ui.add(egui::Label::new(
        egui::RichText::new("左ドラッグ:回転 / 右ドラッグ:移動 / スクロール:ズーム").size(11.0),
    ));
}

/// M-N 曲面の格子点 [N, My, Mz] を正規化ワールド座標 [My_n, Mz_n, N_n] へ変換する
/// （X=My基準、Y=Mz基準、Z=N基準。Z を上にするため N を第3成分に置く）。
fn to_world(g: &[f64; 3], refs: [f64; 3]) -> [f64; 3] {
    [g[1] / refs[0], g[2] / refs[1], g[0] / refs[2]]
}

/// 曲面をワイヤーフレーム（周方向・経線方向の格子線）で描画する。
fn draw_wireframe(
    painter: &egui::Painter,
    surf: &MnSurface,
    refs: [f64; 3],
    cam: &CameraState,
    scale: f32,
    screen_center: [f32; 2],
    color: egui::Color32,
) {
    let center3 = [0.0; 3];
    let proj = |g: &[f64; 3]| {
        let p = project(to_world(g, refs), center3, cam, scale, screen_center);
        egui::pos2(p[0], p[1])
    };
    let stroke = egui::Stroke::new(1.0_f32, theme::translucent(color, 180));

    let n_beta = match surf.grid.first() {
        Some(row) if !row.is_empty() => row.len(),
        _ => return,
    };

    // 周方向（各経線上、j=n_beta-1 と j=0 が接続する閉曲線）
    for row in &surf.grid {
        for j in 0..n_beta {
            let a = proj(&row[j]);
            let b = proj(&row[(j + 1) % n_beta]);
            painter.line_segment([a, b], stroke);
        }
    }
    // 経線方向（引張極→圧縮極）
    for j in 0..n_beta {
        for i in 0..surf.grid.len().saturating_sub(1) {
            let a = proj(&surf.grid[i][j]);
            let b = proj(&surf.grid[i + 1][j]);
            painter.line_segment([a, b], stroke);
        }
    }
}

/// 原点から ±1.3 の座標軸線とラベル「My」「Mz」「N」を描く。
fn draw_axes(painter: &egui::Painter, cam: &CameraState, scale: f32, screen_center: [f32; 2]) {
    let center3 = [0.0; 3];
    let proj = |p: [f64; 3]| {
        let s = project(p, center3, cam, scale, screen_center);
        egui::pos2(s[0], s[1])
    };
    const EXT: f64 = 1.3;
    let axes: [([f64; 3], egui::Color32, &str); 3] = [
        ([EXT, 0.0, 0.0], theme::AXIS_X, "My"),
        ([0.0, EXT, 0.0], theme::AXIS_Y, "Mz"),
        ([0.0, 0.0, EXT], theme::AXIS_Z, "N"),
    ];
    for (dir, color, label) in axes {
        let neg = [-dir[0], -dir[1], -dir[2]];
        painter.line_segment([proj(neg), proj(dir)], egui::Stroke::new(1.5_f32, color));
        painter.text(
            proj(dir),
            egui::Align2::LEFT_BOTTOM,
            label,
            egui::FontId::proportional(13.0),
            color,
        );
    }
}

/// 現在のスライス軸力位置に半透明の水平面（正方形 ±1.15）と N 値ラベルを描く。
fn draw_slice_plane(
    painter: &egui::Painter,
    n_target: f64,
    n_ref: f64,
    cam: &CameraState,
    scale: f32,
    screen_center: [f32; 2],
) {
    let center3 = [0.0; 3];
    let z = n_target / n_ref;
    const H: f64 = 1.15;
    let corners = [[-H, -H, z], [H, -H, z], [H, H, z], [-H, H, z]];
    let poly: Vec<egui::Pos2> = corners
        .iter()
        .map(|p| {
            let s = project(*p, center3, cam, scale, screen_center);
            egui::pos2(s[0], s[1])
        })
        .collect();
    painter.add(egui::Shape::convex_polygon(
        poly,
        theme::translucent(theme::HILITE_PURPLE, 30),
        egui::Stroke::new(1.0_f32, theme::translucent(theme::HILITE_PURPLE, 120)),
    ));

    let label_pos = project([H, H, z], center3, cam, scale, screen_center);
    painter.text(
        egui::pos2(label_pos[0], label_pos[1]),
        egui::Align2::LEFT_CENTER,
        format!("N = {:.1} kN", force_kn(n_target)),
        egui::FontId::proportional(12.0),
        theme::HILITE_PURPLE,
    );
}

/// 2D スライスプロット（My–Mz 相関曲線、egui_plot）を描く。
fn draw_slice_plot(ui: &mut egui::Ui, cache: &MnCache, show: [bool; 3], n_target: f64) {
    let height = ui.available_height();
    egui_plot::Plot::new("mn_slice")
        .data_aspect(1.0)
        .x_axis_label("My [kN·m]")
        .y_axis_label("Mz [kN·m]")
        .legend(egui_plot::Legend::default())
        .height(height)
        .show(ui, |plot_ui| {
            // 単純降伏バネ: 2バネ連成の線形相関 |N|/N許容 + M/M許容 = 1 により、
            // 軸力に応じて (1 − |N|/N許容) 倍に相似縮小する楕円になる
            // （軸力によらず線形に縮む点がファイバ積分系モデルとの違い）。
            if show[0] {
                let n_ref = if n_target >= 0.0 {
                    cache.simple.n_tens.max(1.0)
                } else {
                    cache.simple.n_comp.abs().max(1.0)
                };
                let m_scale = 1.0 - n_target.abs() / n_ref;
                if m_scale > 0.0 {
                    let my = m_scale * moment_kn_m(cache.simple.mp_y);
                    let mz = m_scale * moment_kn_m(cache.simple.mp_z);
                    let pts: Vec<[f64; 2]> = (0..=SLICE_PTS)
                        .map(|k| {
                            let th = 2.0 * std::f64::consts::PI * k as f64 / SLICE_PTS as f64;
                            [my * th.cos(), mz * th.sin()]
                        })
                        .collect();
                    plot_ui.line(
                        egui_plot::Line::new(
                            YieldModelKind::SimpleSpring.label(),
                            egui_plot::PlotPoints::from(pts),
                        )
                        .color(model_color(YieldModelKind::SimpleSpring))
                        .width(2.0_f32),
                    );
                }
            }
            if show[1] {
                plot_slice_curve(
                    plot_ui,
                    &cache.ms_fibers,
                    n_target,
                    YieldModelKind::MultiSpring,
                );
            }
            if show[2] {
                plot_slice_curve(
                    plot_ui,
                    &cache.fiber_fibers,
                    n_target,
                    YieldModelKind::MultiFiber,
                );
            }
        });
}

/// 軸力一定でのファイバ集合の My-Mz 相関曲線を Line として描く。
fn plot_slice_curve(
    plot_ui: &mut egui_plot::PlotUi<'_>,
    fibers: &[PlasticFiber],
    n_target: f64,
    kind: YieldModelKind,
) {
    let pts = slice_at_n(fibers, n_target, SLICE_PTS);
    if pts.is_empty() {
        return;
    }
    let mut xy: Vec<[f64; 2]> = pts
        .iter()
        .map(|p| [moment_kn_m(p[0]), moment_kn_m(p[1])])
        .collect();
    xy.push(xy[0]); // 始点を末尾に複製して閉じる
    plot_ui.line(
        egui_plot::Line::new(kind.label(), egui_plot::PlotPoints::from(xy))
            .color(model_color(kind))
            .width(2.0_f32),
    );
}

/// M-θ 骨格曲線プロット（塑性化領域考慮、egui_plot）を描く。
/// 単純降伏バネは材端剛塑性ばね（弾性部材+剛塑性ヒンジ）の折れ線として、
/// マルチスプリング／マルチファイバーは `MThetaData` にキャッシュ済みの
/// 断面 M-φ 由来の骨格曲線として描く。
fn draw_m_theta_plot(ui: &mut egui::Ui, data: &MThetaData, show: [bool; 3]) {
    let height = ui.available_height();
    egui_plot::Plot::new("mn_m_theta")
        .x_axis_label("θ [×10⁻³ rad]")
        .y_axis_label("M [kN·m]")
        .legend(egui_plot::Legend::default())
        .height(height)
        .show(ui, |plot_ui| {
            if show[0] {
                if let Some(pts) = &data.simple {
                    plot_m_theta_line(plot_ui, pts, YieldModelKind::SimpleSpring);
                }
            }
            if show[1] {
                if let Some(pts) = &data.ms {
                    plot_m_theta_line(plot_ui, pts, YieldModelKind::MultiSpring);
                }
            }
            if show[2] {
                if let Some(pts) = &data.fiber {
                    plot_m_theta_line(plot_ui, pts, YieldModelKind::MultiFiber);
                }
            }
        });
}

/// [θ(rad), M(N·mm)] 点列を表示単位（θ:×10⁻³rad, M:kN·m）へ換算して Line を描く。
fn plot_m_theta_line(plot_ui: &mut egui_plot::PlotUi<'_>, pts: &[[f64; 2]], kind: YieldModelKind) {
    if pts.is_empty() {
        return;
    }
    let xy: Vec<[f64; 2]> = pts
        .iter()
        .map(|p| [p[0] * 1e3, moment_kn_m(p[1])])
        .collect();
    plot_ui.line(
        egui_plot::Line::new(kind.label(), egui_plot::PlotPoints::from(xy))
            .color(model_color(kind))
            .width(2.0_f32),
    );
}
