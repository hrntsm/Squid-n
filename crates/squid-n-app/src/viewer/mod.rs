use crate::app::App;
use crate::theme;

mod viewcube;
use squid_n_core::dof::{Dof, Dof6Mask};

mod check_ratio;
mod diagram;
mod frame_view;
// ヒンジ詳細ウィンドウのキャッシュ型（`MnCurveCache`）を App から参照できるよう
// モジュール自体を crate 内に公開する（型自体は pub(crate)）。
pub(crate) mod hinge;
mod lumped;
mod modeling;
mod solid;
// 立体グリッドのスナップ点（`SnapPoint`）を App の作成モード状態が保持するため、
// モジュールを crate 内へ公開する。
pub(crate) mod space_grid;
mod support_symbols;
pub(crate) mod th_detail;

mod camera;
mod cmq;
mod deform;
mod pick;
mod playback;
mod scene;
mod support;

/// ビューアの表示モード。
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum ViewMode {
    /// 形状のみ
    #[default]
    Shape,
    /// 変形図（線形静的結果）
    Deformed,
    /// モード形（固有値結果）
    Mode,
    /// 応力図（N/Q/M。表示する成分は [`ForceComponent`] で切り替える）
    Force,
    /// CMQ 図（両端固定端モーメント C とせん断 Q）
    Cmq,
    /// 検定比図（部材検定の最大検定比で着色）
    CheckRatio,
    /// モデル化図（解析上どの要素モデルで扱っているかを着色・記号で可視化）
    Modeling,
    /// ヒンジ図（増分解析のヒンジ発生位置を可視化）
    Hinge,
    /// 時刻歴アニメーション（時刻歴応答解析の詳細記録 `ThRecording` を再生表示）
    TimeHistory,
    /// 質点系の固有値モード（球・ばね）
    LumpedMode,
    /// 質点系の時刻歴再生（球・ばね）
    LumpedTimeHistory,
}

/// モデル化図で可視化する解析種別。
///
/// 同じモデルでも解析種別によって部材のモデル化（要素定式化）が変わるため、
/// どちらを可視化するかを切り替える。
/// - 静解析（線形）は断面の降伏を考えないため、部材は原則すべて弾性でモデル化される。
/// - 増分解析（弾塑性）は降伏を考慮するため、軸力変動する部材はファイバー要素、
///   剛床上で軸力変動が小さい梁は材端集中塑性（材端回転ばね）へ振り分けられる。
///
/// いずれの種別でも耐震壁の側柱は面内両端ピンとしてモデル化される（トポロジ由来の
/// 解放のため解析種別に依らない）。
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum ModelingAnalysis {
    /// 静解析（線形）: 断面の降伏を考えず全部材を弾性でモデル化する。
    #[default]
    Static,
    /// 増分解析（弾塑性）: 降伏を考慮し、ファイバー要素と材端集中塑性を使い分ける。
    Incremental,
}

/// 応力図（[`ViewMode::Force`]）で表示できる成分。
///
/// 部材内力ベクトル `[N, Qy, Qz, Mx, My, Mz]` の 6 成分に 1 対 1 で対応し、
/// 列挙順＝内力ベクトルの添字である（[`Self::force_index`]）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ForceComponent {
    /// 軸力 N（引張正）
    #[default]
    N,
    /// 強軸せん断 Qy（Mz 面）
    Qy,
    /// 弱軸せん断 Qz（My 面）
    Qz,
    /// ねじりモーメント Mx
    Mx,
    /// 弱軸曲げ My
    My,
    /// 強軸曲げ Mz
    Mz,
}

/// 応力図の張り出し面（部材局所座標のどちらの軸へ図を出すか）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum DiagramPlane {
    /// 局所 ey 面（せい方向。強軸曲げ Mz・強軸せん断 Qy・N・Mx）
    Ey,
    /// 局所 ez 面（幅方向。弱軸曲げ My・弱軸せん断 Qz）
    Ez,
}

impl ForceComponent {
    /// 表示順（凡例・チェックボックスの並び）。内力ベクトルの添字順と同じ。
    pub(crate) const ALL: [ForceComponent; 6] = [
        ForceComponent::N,
        ForceComponent::Qy,
        ForceComponent::Qz,
        ForceComponent::Mx,
        ForceComponent::My,
        ForceComponent::Mz,
    ];

    /// 部材内力ベクトル `[N, Qy, Qz, Mx, My, Mz]` 内の添字。
    pub(crate) fn force_index(self) -> usize {
        match self {
            ForceComponent::N => 0,
            ForceComponent::Qy => 1,
            ForceComponent::Qz => 2,
            ForceComponent::Mx => 3,
            ForceComponent::My => 4,
            ForceComponent::Mz => 5,
        }
    }

    /// 図・凡例・チェックボックスの記号ラベル。
    pub(crate) fn label(self) -> &'static str {
        match self {
            ForceComponent::N => "N",
            ForceComponent::Qy => "Qy",
            ForceComponent::Qz => "Qz",
            ForceComponent::Mx => "Mx",
            ForceComponent::My => "My",
            ForceComponent::Mz => "Mz",
        }
    }

    /// 成分固定色（複数成分を重ねたときの識別に用いる。単色塗り・輪郭線・
    /// 数値ラベル・凡例の色見本で共通）。CMQ 図の配色（C=青・M=紫・Q=緑）と
    /// 同系統になるよう、曲げ＝紫系／せん断＝緑系／軸力＝青とする。
    pub(crate) fn color(self) -> egui::Color32 {
        match self {
            ForceComponent::N => theme::DATA_BLUE,
            ForceComponent::Qy => theme::GOOD_GREEN,
            ForceComponent::Qz => theme::ISOLATOR_TEAL,
            ForceComponent::Mx => theme::SECONDARY_AMBER,
            ForceComponent::My => theme::PARETO_RED,
            ForceComponent::Mz => theme::HILITE_PURPLE,
        }
    }

    /// モーメント成分か（表示単位の切り替えに用いる）。
    pub(crate) fn is_moment(self) -> bool {
        matches!(
            self,
            ForceComponent::Mx | ForceComponent::My | ForceComponent::Mz
        )
    }

    /// 表示単位（内部単位 N・N·mm から換算した表示系）。
    pub(crate) fn unit(self) -> &'static str {
        if self.is_moment() {
            "kN·m"
        } else {
            "kN"
        }
    }

    /// 内部単位（N・N·mm）から表示単位（kN・kN·m）への換算係数。
    pub(crate) fn display_scale(self) -> f64 {
        if self.is_moment() {
            1.0e-6
        } else {
            1.0e-3
        }
    }

    /// 張り出し面。強軸曲げの組（Qy・Mz）と軸力・ねじりは局所 ey 面、
    /// 弱軸曲げの組（Qz・My）は局所 ez 面へ出す。これにより 6 成分を同時表示
    /// しても、直交 2 面に分かれて重なりが減る。
    pub(crate) fn plane(self) -> DiagramPlane {
        match self {
            ForceComponent::Qz | ForceComponent::My => DiagramPlane::Ez,
            _ => DiagramPlane::Ey,
        }
    }

    /// 図として張り出す値の符号（内力値に乗じる）。
    ///
    /// 断面力の符号規約は `Qy = dMz/dx`・`Qz = −dMy/dx`（5.2）で、My だけ
    /// せん断との関係が反転している。応力図は「正のモーメントを引張側へ
    /// 張り出す」規約のため、My は符号を反転させて張り出すことで、強軸側
    /// （Mz）を 90° 回した場合と同じ側（引張側）に図が出る。
    pub(crate) fn plot_sign(self) -> f64 {
        match self {
            ForceComponent::My => -1.0,
            _ => 1.0,
        }
    }

    /// 曲げモーメント成分のとき、3 次エルミート補間の勾配に用いるせん断成分。
    ///
    /// 張り出し値 `plot_sign·M` に対して `d(張り出し値)/dx = 対応せん断` が
    /// 成り立つ組（Mz→Qy、My→Qz）を返す。せん断・軸力・ねじりは補間しない。
    pub(crate) fn moment_gradient_source(self) -> Option<ForceComponent> {
        match self {
            ForceComponent::Mz => Some(ForceComponent::Qy),
            ForceComponent::My => Some(ForceComponent::Qz),
            _ => None,
        }
    }
}

/// 応力図で表示中の成分の組（6 成分の ON/OFF）。
///
/// 既定は M 図プリセット（My・Mz）。複数成分を同時に表示でき、同単位の成分
/// （力 kN: N・Qy・Qz、モーメント kN·m: Mx・My・Mz）は共有 max で正規化して
/// 描く。力とモーメントは単位が異なるためグループ間では共有しない。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ForceComponents([bool; 6]);

impl Default for ForceComponents {
    fn default() -> Self {
        Self::PRESET_M
    }
}

impl ForceComponents {
    /// N 図プリセット（軸力のみ）。
    pub const PRESET_N: Self = Self([true, false, false, false, false, false]);
    /// Q 図プリセット（両方向のせん断）。
    pub const PRESET_Q: Self = Self([false, true, true, false, false, false]);
    /// M 図プリセット（両方向の曲げ）。既定。
    pub const PRESET_M: Self = Self([false, false, false, false, true, true]);

    /// 成分が表示中か。
    pub fn is_on(self, c: ForceComponent) -> bool {
        self.0[c.force_index()]
    }

    /// チェックボックスへ渡す可変参照。
    pub fn flag_mut(&mut self, c: ForceComponent) -> &mut bool {
        &mut self.0[c.force_index()]
    }

    /// 表示中の成分を表示順（内力ベクトルの添字順）で列挙する。
    pub fn selected(self) -> impl Iterator<Item = ForceComponent> {
        ForceComponent::ALL
            .into_iter()
            .filter(move |c| self.is_on(*c))
    }
}

/// CMQ 図で表示する成分（C: 固定端モーメント／M: 単純梁中央モーメント／Q: せん断）。
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum CmqComponent {
    /// 固定端モーメント C 図
    #[default]
    C,
    /// 単純梁としての曲げモーメント M 図（中央モーメントの目安）
    M,
    /// せん断 Q 図
    Q,
}

/// 検定比図の着色対象（最大＝全式の max、または特定の検定式のみ）。
///
/// `Kind` を選ぶと、部材・節点の色や中点ラベル・位置別マーカーが当該検定式
/// だけの検定比（`CheckResult::components` から抽出）に基づいて決まる。
/// 対象の式が存在しない検定位置は「フィルタ対象外」として着色・マーカー
/// ともに描かない（詳細は `check_ratio.rs` の `ratio_for_filter` を参照）。
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum CheckRatioFilter {
    /// 全検定式中の最大検定比（既定）
    #[default]
    Max,
    /// 特定の検定式のみ
    Kind(squid_n_design_jp::CheckKind),
}

/// 描画対象の部材の絞り込み（2D 構面表示）。
///
/// 全体表示では常に「描く」を返し、構面表示では [`squid_n_core::frame::Frame`] が
/// 判定した所属だけを描く。表示モードごとの描画（応力図・検定比・ヒンジ・モデル化・
/// 断面ソリッド）が同じ絞り込みに従うよう、判定はこの 1 か所に集約する。
#[derive(Clone, Copy, Default)]
pub(crate) struct FrameFilter<'a> {
    elem_on: Option<&'a [bool]>,
    node_on: Option<&'a [bool]>,
}

impl<'a> FrameFilter<'a> {
    fn new(frame: Option<&'a squid_n_core::frame::Frame>) -> Self {
        Self {
            elem_on: frame.map(|f| f.elem_on.as_slice()),
            node_on: frame.map(|f| f.node_on.as_slice()),
        }
    }

    /// 節点の配列添字で判定する。要素として持たない描画（支点記号・支点ばね・
    /// 免震マーカー・スラブ・二次部材・剛床代表点）を構面へ絞り込むために使う。
    /// 「床・二次部材」トグルとは独立で、構面の所属だけを見る。
    pub(crate) fn shows_node(&self, i: usize) -> bool {
        match self.node_on {
            Some(on) => on.get(i).copied().unwrap_or(false),
            None => true,
        }
    }

    /// 要素の配列添字で判定する。
    pub(crate) fn shows_index(&self, i: usize) -> bool {
        match self.elem_on {
            Some(on) => on.get(i).copied().unwrap_or(false),
            None => true,
        }
    }

    /// 要素 ID で判定する（`Model::validate` の「id == 添字」不変条件による）。
    pub(crate) fn shows(&self, id: squid_n_core::ids::ElemId) -> bool {
        self.shows_index(id.index())
    }
}

/// 3D→2D 投影の文脈（回転中心・カメラ・スケール・描画領域中心）を束ねる。
///
/// 多数の描画関数へ `(center3, cam, scale, screen_center)` を個別に引き回す代わりに、
/// この 1 つの参照で受け渡し、投影数式の単一情報源とする（§3-2: ターンテーブル
/// 回転＋正射影。ビュー軸は X=右・Y=上・Z=手前）。深度ソートや面陰影で回転後の
/// カメラ空間ベクトルが要る箇所のため、`to_cam`（回転まで）と `cam_to_screen`
/// （画面写像）に分けて公開する。
#[derive(Clone, Copy)]
pub(crate) struct Projector<'a> {
    /// モデル中心（回転中心）。
    center3: [f64; 3],
    cam: &'a CameraState,
    /// px/世界長。
    scale: f32,
    /// 描画領域中心（px）。
    screen_center: [f32; 2],
}

impl<'a> Projector<'a> {
    pub(crate) fn new(
        center3: [f64; 3],
        cam: &'a CameraState,
        scale: f32,
        screen_center: [f32; 2],
    ) -> Self {
        Self {
            center3,
            cam,
            scale,
            screen_center,
        }
    }

    /// px/世界長。ワールド長 ⇔ 画面 px の換算に使う。
    pub(crate) fn scale(&self) -> f32 {
        self.scale
    }

    /// モデル中心（回転中心）。グリッド範囲の算定などワールド基準の計算に使う。
    pub(crate) fn center3(&self) -> [f64; 3] {
        self.center3
    }

    /// ワールド座標を、回転中心基準でカメラ回転を掛けたカメラ空間ベクトルへ変換する
    /// （r[0]=右, r[1]=上, r[2]=手前）。深度ソート・面陰影で中間ベクトルが要る。
    pub(crate) fn cam_space(&self, p: [f64; 3]) -> [f32; 3] {
        let v = [
            (p[0] - self.center3[0]) as f32,
            (p[1] - self.center3[1]) as f32,
            (p[2] - self.center3[2]) as f32,
        ];
        q_rotate(self.cam.rot, v)
    }

    /// カメラ空間ベクトルをスクリーン座標へ写す（パン加算・スケール・画面 Y 反転）。
    pub(crate) fn cam_to_screen(&self, r: [f32; 3]) -> egui::Pos2 {
        egui::pos2(
            self.screen_center[0] + self.cam.pan[0] + r[0] * self.scale,
            self.screen_center[1] + self.cam.pan[1] - r[1] * self.scale,
        )
    }

    /// ワールド座標 `p` をスクリーン座標へ投影する。
    pub(crate) fn project(&self, p: [f64; 3]) -> egui::Pos2 {
        self.cam_to_screen(self.cam_space(p))
    }

    /// `base3` から `dir3` 方向へ `off_world` だけ張り出した点を投影する。
    pub(crate) fn project_offset(
        &self,
        base3: [f64; 3],
        dir3: [f64; 3],
        off_world: f64,
    ) -> egui::Pos2 {
        self.project([
            base3[0] + dir3[0] * off_world,
            base3[1] + dir3[1] * off_world,
            base3[2] + dir3[2] * off_world,
        ])
    }
}

/// ワールド座標 `p` を投影する（`[f32; 2]` 版。M-N 相関曲面ビュー用の下位ラッパ）。
/// 投影数式は [`Projector`] を単一情報源とする。
pub(crate) fn project(
    p: [f64; 3],
    center3: [f64; 3],
    cam: &CameraState,
    scale: f32,
    screen_center: [f32; 2],
) -> [f32; 2] {
    let pos = Projector::new(center3, cam, scale, screen_center).project(p);
    [pos.x, pos.y]
}

use camera::q_rotate;
use cmq::draw_cmq_diagram;
use deform::{
    bbox_diagonal, deform_display_scale, display_disp, frame_bbox, model_bbox, model_bbox_size,
    time_history_deform_scale, BeamDeflection, DEFORM_CURVE_SEGMENTS,
};
use pick::{member_load_pickable, pick_nearest_member, pick_nearest_node};
use playback::{advance_play_time, frame_at_time};
use scene::{
    draw_axis_gadget, draw_grid_and_axes, draw_mode_rest_ghost, draw_slabs_and_joists,
    draws_as_line, element_draw_shape, order_wall_nodes, DrawShape,
};
use squid_n_core::geom::vec3::dist as member_len3;
use support::{
    draw_support_legend, draw_support_symbol, support_kind, supports_visible, SupportKind,
};

pub use camera::CameraState;
pub use deform::TimeHistoryScaleCache;

pub fn viewer_panel(ui: &mut egui::Ui, app: &mut App) {
    let mut mode = app.view_mode;
    let mut mode_idx = app.view_mode_idx;
    let mut force_components = app.force_components;
    let mut cmq_component = app.cmq_component;
    let mut check_ratio_filter = app.check_ratio_filter;
    let mut modeling_analysis = app.modeling_analysis;
    // 時刻歴の詳細記録（`ThRecording`）がある場合のみ「時刻歴」モードを選択肢に出す。
    let has_th_recording = app
        .results
        .as_ref()
        .and_then(|r| r.time_history.as_ref())
        .and_then(|t| t.recording.as_ref())
        .is_some();

    // --- コントロール ---
    // 中央パネルが狭い場合（左パネルを広げた時など）にボタン列が右パネルへ
    // はみ出さないよう、折り返し可能なレイアウトにする。
    ui.horizontal_wrapped(|ui| {
        ui.label("表示:");
        ui.selectable_value(&mut mode, ViewMode::Shape, "形状");
        ui.selectable_value(&mut mode, ViewMode::Deformed, "変形");
        ui.selectable_value(&mut mode, ViewMode::Mode, "モード");
        ui.selectable_value(&mut mode, ViewMode::Force, "応力図");
        ui.selectable_value(&mut mode, ViewMode::Cmq, "CMQ図");
        ui.selectable_value(&mut mode, ViewMode::CheckRatio, "検定比");
        ui.selectable_value(&mut mode, ViewMode::Hinge, "ヒンジ");
        ui.selectable_value(&mut mode, ViewMode::Modeling, "モデル化");
        if has_th_recording {
            ui.selectable_value(&mut mode, ViewMode::TimeHistory, "時刻歴");
        }
        if lumped::has_lumped(app) {
            ui.selectable_value(&mut mode, ViewMode::LumpedMode, "質点モード");
            ui.selectable_value(&mut mode, ViewMode::LumpedTimeHistory, "質点時刻歴");
        }
        ui.separator();
        // 断面表示: 部材を断面形状の押し出しソリッドで立体表示（全モードと併用可）
        ui.toggle_value(&mut app.show_sections, "断面表示");
        // 床（スラブ・小梁）・二次部材の表示切替（全モードと併用可。
        // CMQ 図は主架構の図のため設定によらず常に非表示）
        ui.toggle_value(&mut app.show_floor_secondary, "床・二次部材");
        // 支点記号。質点ビューでは立体の柱脚拘束は関係ないので選択肢自体を出さない。
        if !lumped::is_lumped_view(mode) {
            ui.toggle_value(&mut app.show_supports, "支点")
                .on_hover_text("拘束された節点の矢印・円弧、支点ばね、免震マーカー");
        }
        // 剛床代表点（重心マスター）の表示切替。剛床がある場合のみ選択肢を出す。
        // ON にすると代表点マーカー・面内拘束マーク・スレーブへの点線を描く。
        let has_diaphragm_constraint = app
            .model
            .constraints
            .iter()
            .any(|c| matches!(c, squid_n_core::model::Constraint::RigidDiaphragm { .. }));
        if has_diaphragm_constraint {
            ui.toggle_value(&mut app.show_diaphragm_master, "剛床代表点");
        }
        // 立体グリッド（通り芯 × 階レベル）の表示切替。通り芯と階の両方がある
        // モデルでしか格子を作れないため、そのときだけ選択肢を出す。
        if space_grid::has_grid(&app.model) {
            ui.toggle_value(&mut app.show_space_grid, "通り芯グリッド")
                .on_hover_text(
                    "各階レベルに通り芯の平面格子を描きます。\
                     梁作成モードでは格子点にスナップし、節点が無ければ梁とあわせて作ります",
                );
        }
        ui.separator();
        // §3-2 の操作規約をヒント表示（左ドラッグ=回転／スクロール=ズーム）
        ui.add_enabled(
            false,
            egui::Label::new(
                egui::RichText::new("左ドラッグ:回転 / 右ドラッグ:移動 / スクロール:ズーム")
                    .size(11.0),
            ),
        );
    });
    if mode == ViewMode::Cmq {
        ui.horizontal(|ui| {
            ui.label("成分:");
            ui.selectable_value(&mut cmq_component, CmqComponent::C, "C(モーメント)");
            ui.selectable_value(&mut cmq_component, CmqComponent::M, "M(中央)");
            ui.selectable_value(&mut cmq_component, CmqComponent::Q, "Q(せん断)");
        });
    }
    // モデル化図: 可視化する解析種別（静解析＝弾性／増分解析＝弾塑性）を切り替える。
    // 静解析は断面の降伏を考えないため全部材が弾性、増分解析は降伏を考慮するため
    // ファイバー要素と材端集中塑性を使い分ける、という違いを見比べられる。
    if mode == ViewMode::Modeling {
        ui.horizontal_wrapped(|ui| {
            ui.label("解析種別:");
            ui.selectable_value(
                &mut modeling_analysis,
                ModelingAnalysis::Static,
                "静解析(弾性)",
            );
            ui.selectable_value(
                &mut modeling_analysis,
                ModelingAnalysis::Incremental,
                "増分解析(弾塑性)",
            );
            ui.separator();
            ui.add_enabled(
                false,
                egui::Label::new(
                    egui::RichText::new("部材の色＝解析上の要素モデル。○=端部ピン／□=半剛")
                        .size(11.0),
                ),
            );
        });
    }
    // 応力図: 6 成分（N/Qy/Qz/Mx/My/Mz）をチェックボックスで個別に ON/OFF し、
    // 選んだ成分をすべて同時に描く。よく使う組はプリセットボタンで切り替える。
    // 単色塗り／コンター（値に応じた色分け）と数値ラベルの表示もここで切替える。
    // コンター ON 時のみカラーマップ選択（既定 Viridis。TONMANUAL §3）を表示する。
    if mode == ViewMode::Force {
        ui.horizontal_wrapped(|ui| {
            ui.label("成分:");
            for c in ForceComponent::ALL {
                // ラベルを成分固定色で描き、図・凡例・数値ラベルの色と対応づける。
                ui.checkbox(
                    force_components.flag_mut(c),
                    egui::RichText::new(c.label()).color(c.color()),
                );
            }
            ui.separator();
            if ui.button("N図").clicked() {
                force_components = ForceComponents::PRESET_N;
            }
            if ui.button("Q図").clicked() {
                force_components = ForceComponents::PRESET_Q;
            }
            if ui.button("M図").clicked() {
                force_components = ForceComponents::PRESET_M;
            }
        });
        ui.horizontal_wrapped(|ui| {
            // 応力図に変形図を重ねる（変位は自動倍率で節点座標に加味され、
            // 図も変形後の材軸に沿って描かれる）
            ui.toggle_value(&mut app.overlay_deform, "変形表示");
            ui.toggle_value(&mut app.diagram_contour, "コンター");
            if app.diagram_contour {
                let mut colormap = app.contour_colormap;
                egui::ComboBox::from_id_salt("contour_colormap")
                    .selected_text(colormap.label())
                    .show_ui(ui, |ui| {
                        for cm in [
                            theme::ColorMap::Viridis,
                            theme::ColorMap::Plasma,
                            theme::ColorMap::Turbo,
                            theme::ColorMap::Jet,
                            theme::ColorMap::BlueWhiteRed,
                        ] {
                            ui.selectable_value(&mut colormap, cm, cm.label());
                        }
                    });
                app.contour_colormap = colormap;
            }
            ui.toggle_value(&mut app.diagram_values, "値を表示")
                .on_hover_text(
                    "各部材の両端部と中央（ξ=0・0.5・1.0）の値を kN・kN·m で表示します\
                     （その成分の最大値の 1% 未満は表示しません）。",
                );
        });
    }
    // 検定比図: 検定式フィルタ（最大／式別、結果に現れる式のみ選択肢に出す）と
    // 位置別マーカーの表示切替。
    if mode == ViewMode::CheckRatio {
        fn checked_components(
            outcome: &squid_n_design_jp::CheckOutcome,
        ) -> Option<&[squid_n_design_jp::CheckComponent]> {
            match outcome {
                squid_n_design_jp::CheckOutcome::Checked(cr) => Some(cr.components.as_slice()),
                squid_n_design_jp::CheckOutcome::Skipped { .. } => None,
            }
        }
        let available_kinds = app
            .results
            .as_ref()
            .map(|r| {
                check_ratio::available_check_kinds(
                    r.member_checks
                        .iter()
                        .flat_map(|m| m.positions.iter())
                        .filter_map(|p| checked_components(&p.outcome))
                        .chain(
                            r.joint_checks
                                .iter()
                                .filter_map(|j| checked_components(&j.outcome)),
                        ),
                )
            })
            .unwrap_or_default();
        ui.horizontal_wrapped(|ui| {
            ui.label("検定式:");
            ui.selectable_value(&mut check_ratio_filter, CheckRatioFilter::Max, "最大");
            for k in &available_kinds {
                ui.selectable_value(
                    &mut check_ratio_filter,
                    CheckRatioFilter::Kind(*k),
                    k.label(),
                );
            }
            ui.separator();
            ui.checkbox(&mut app.check_ratio_markers, "位置別マーカー");
            ui.checkbox(&mut app.check_ratio_label_all, "全部材に数値ラベル")
                .on_hover_text(
                    "既定では検定比 0.8 以上の部材にのみ数値ラベルを表示し、\
                     それ未満は色の濃淡（グラデーション）で余裕度を示します。",
                );
        });
    }
    if mode == ViewMode::Mode {
        let n_modes = app
            .results
            .as_ref()
            .and_then(|r| r.modal.as_ref())
            .map(|m| m.period.len())
            .unwrap_or(0);
        if n_modes > 0 {
            ui.horizontal(|ui| {
                ui.label("モード:");
                let mut idx = mode_idx.min(n_modes - 1);
                ui.add(egui::Slider::new(&mut idx, 0..=n_modes - 1).text(""));
                mode_idx = idx;
                if let Some(t) = app
                    .results
                    .as_ref()
                    .and_then(|r| r.modal.as_ref())
                    .and_then(|m| m.period.get(idx))
                {
                    ui.label(format!("T={:.3} s", t));
                }
            });
        }
    }
    if mode == ViewMode::LumpedMode {
        let n_modes = app
            .results
            .as_ref()
            .and_then(|r| r.lumped.as_ref())
            .map(|m| m.modal.period.len())
            .unwrap_or(0);
        if n_modes > 0 {
            ui.horizontal(|ui| {
                ui.label("モード:");
                let mut idx = mode_idx.min(n_modes - 1);
                ui.add(egui::Slider::new(&mut idx, 0..=n_modes - 1).text(""));
                mode_idx = idx;
                if let Some(t) = app
                    .results
                    .as_ref()
                    .and_then(|r| r.lumped.as_ref())
                    .and_then(|m| m.modal.period.get(idx))
                {
                    ui.label(format!("T={:.3} s", t));
                }
            });
        }
    }
    if lumped::is_lumped_view(mode) {
        ui.horizontal(|ui| {
            ui.checkbox(&mut app.lumped_show_frame, "骨組を重ねる");
        });
    }
    // 時刻歴モード: フレームスライダー・再生制御（§実装内容1）。
    // 現在フレームは `app.th_frame`、再生経過時刻は `app.th_play_time`
    // （`frame_time` に基づき現在フレームへ写像。末尾でループ）で管理する。
    if mode == ViewMode::TimeHistory {
        if app.staleness.results_stale {
            // 中-1(a): モデル編集後は添字ずれ（部材削除・並び替え）で別部材のデータを
            // 表示する恐れがあるため、再解析するまで変形アニメーション・部材クリックを
            // 無効化する（フレームスライダー自体も表示しない）。
            ui.colored_label(
                theme::WARN_TEXT,
                "⚠ モデルが編集されています。解析を再実行してください\
                 （変形アニメーション・部材クリックは無効化しています）。",
            );
        } else if let Some(recording) = app
            .results
            .as_ref()
            .and_then(|r| r.time_history.as_ref())
            .and_then(|t| t.recording.as_ref())
        {
            let n_frames = recording.frame_time.len();
            if n_frames > 0 {
                let duration = recording.frame_time.last().copied().unwrap_or(0.0);
                app.th_frame = app.th_frame.min(n_frames - 1);
                ui.horizontal_wrapped(|ui| {
                    if ui
                        .button(if app.th_playing { "⏸" } else { "▶" })
                        .on_hover_text("再生 / 一時停止")
                        .clicked()
                    {
                        app.th_playing = !app.th_playing;
                    }
                    ui.label("速度:");
                    for s in [0.25_f32, 0.5, 1.0, 2.0] {
                        ui.selectable_value(&mut app.th_speed, s, format!("×{s}"));
                    }
                    ui.separator();
                    let mut frame = app.th_frame;
                    if ui
                        .add(egui::Slider::new(&mut frame, 0..=n_frames - 1).text(""))
                        .changed()
                    {
                        app.th_frame = frame;
                        app.th_play_time = recording.frame_time[frame];
                    }
                    let t = recording.frame_time[app.th_frame];
                    ui.label(format!("t={:.2}s / {:.2}s", t, duration));
                });
                // 再生中は実時間×速度でフレームを進め、連続描画のため毎フレーム再描画を要求する。
                if app.th_playing {
                    let dt = ui.input(|i| i.stable_dt);
                    app.th_play_time =
                        advance_play_time(app.th_play_time, dt, app.th_speed, duration);
                    app.th_frame = frame_at_time(&recording.frame_time, app.th_play_time);
                    ui.ctx().request_repaint();
                }
            } else {
                ui.label("時刻歴の記録フレームがありません。");
            }
        } else {
            ui.label("時刻歴の詳細記録がありません（再解析すると記録されます）。");
        }
    }
    if mode == ViewMode::LumpedTimeHistory {
        if let Some(th) = app
            .results
            .as_ref()
            .and_then(|r| r.lumped.as_ref())
            .and_then(|l| l.response.as_ref())
        {
            let n_frames = th.time.len();
            if n_frames > 0 {
                let duration = th.time.last().copied().unwrap_or(0.0);
                app.th_frame = app.th_frame.min(n_frames - 1);
                ui.horizontal_wrapped(|ui| {
                    if ui.button(if app.th_playing { "⏸" } else { "▶" }).clicked() {
                        app.th_playing = !app.th_playing;
                    }
                    ui.label("速度:");
                    for s in [0.25_f32, 0.5, 1.0, 2.0] {
                        ui.selectable_value(&mut app.th_speed, s, format!("×{s}"));
                    }
                    let mut frame = app.th_frame;
                    if ui
                        .add(egui::Slider::new(&mut frame, 0..=n_frames - 1).text(""))
                        .changed()
                    {
                        app.th_frame = frame;
                        app.th_play_time = th.time.get(frame).copied().unwrap_or(0.0);
                        app.th_playing = false;
                    }
                    ui.label(format!("t={:.3} s", app.th_play_time));
                });
                if app.th_playing {
                    let dt = ui.input(|i| i.stable_dt);
                    app.th_play_time =
                        playback::advance_play_time(app.th_play_time, dt, app.th_speed, duration);
                    app.th_frame = playback::frame_at_time(&th.time, app.th_play_time);
                    ui.ctx().request_repaint();
                }
            }
        } else {
            ui.colored_label(theme::GRAY_600, "質点系時刻歴の結果がありません。");
        }
    }
    // 変形表示オプション行: 変形を表示するモード（変形・モード・応力図の変形重ね）で
    // 表示する。「内部たわみ」トグルで梁の Hermite 曲線表示（＋床・二次部材の曲線
    // 追従）と直線表示（全体の変形）を切り替え、変形倍率スライダーで自動算定倍率への
    // 手動係数を対数調整（「リセット」で 1.0）する。
    let show_deform_options = matches!(
        mode,
        ViewMode::Deformed
            | ViewMode::Mode
            | ViewMode::TimeHistory
            | ViewMode::LumpedMode
            | ViewMode::LumpedTimeHistory
    ) || (mode == ViewMode::Force && app.overlay_deform);
    if show_deform_options {
        ui.horizontal(|ui| {
            ui.toggle_value(&mut app.show_beam_interpolation, "内部たわみ")
                .on_hover_text(
                    "梁を内部たわみ（Hermite 曲線）で描き、床・二次部材も曲線に追従。\
                     OFF で梁を直線（弦）にし全体の変形を見る",
                );
            ui.separator();
            ui.label("変形倍率:");
            ui.add(
                egui::Slider::new(&mut app.deform_scale_factor, 0.1..=10.0)
                    .logarithmic(true)
                    .text("×（自動比）"),
            );
            if ui.button("リセット").clicked() {
                app.deform_scale_factor = 1.0;
            }
        });
    }

    app.view_mode = mode;
    app.view_mode_idx = mode_idx;
    app.force_components = force_components;
    app.cmq_component = cmq_component;
    app.check_ratio_filter = check_ratio_filter;
    app.modeling_analysis = modeling_analysis;

    // CMQ 図はモデル編集に常に追従させるため、表示中は毎フレーム再計算する
    // （スラブ数は小さい前提）。
    if app.view_mode == ViewMode::Cmq {
        app.refresh_beam_loads();
    }

    // --- 表示範囲（全体 / 通り / 階）---
    // 通り芯・階の 1 構面だけを正対で描く 2D 表示。表示モード（形状・変形・応力図…）
    // とは独立で、どのモードでも構面へ絞り込める。
    frame_range_controls(ui, app);

    // 構面を解決する。通り芯の再生成・モデルの入れ替えで添字がずれることがあるため、
    // 毎フレーム実在を検証し、解決できなければ全体表示へ戻す。
    let frame = app
        .frame_target
        .and_then(|t| squid_n_core::frame::build_frame(&app.model, t));
    if frame.is_none() {
        app.frame_target = None;
    }

    ui.separator();

    // --- 描画領域 ---
    let (rect, response) = ui.allocate_exact_size(
        egui::vec2(ui.available_width(), ui.available_height()),
        egui::Sense::click_and_drag(),
    );

    // カメラ操作（§3-2: 左ドラッグ=回転 / スクロール=ズーム）。
    // パンは規約外の補助操作として右ドラッグに割り当てる。
    // 構面表示中は回転させると正対が崩れ、構面内に描く基準線も傾くため回転を禁じ、
    // 左ドラッグもパンに割り当てる（2D CAD の操作に揃える）。
    let mut cam = app.camera.clone();
    if response.dragged_by(egui::PointerButton::Primary) {
        let d = response.drag_delta();
        if frame.is_some() {
            cam.pan[0] += d.x;
            cam.pan[1] += d.y;
        } else {
            // ターンテーブル回転（鉛直軸を画面上で縦に保つ。CameraState のドキュメント参照）。
            cam.turntable_drag(d.x, d.y);
        }
    }
    if response.dragged_by(egui::PointerButton::Secondary) {
        let d = response.drag_delta();
        cam.pan[0] += d.x;
        cam.pan[1] += d.y;
    }
    // スクロールズーム（係数 0.01、0.5–10.0 にクランプ）。トラックパッドのピンチも反映。
    // ポインタが描画領域上にあるときのみ反応させる。`hovered()` は手前のレイヤー
    // （ヒンジ詳細などの egui::Window）による遮蔽も考慮するため、ポップアップが
    // 重なっている間は手前のビューだけが反応する。
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

    // 構面表示中は、その構面の法線方向へ毎フレーム正対させる（回転操作は上で禁じて
    // いるが、全体表示から切り替えた直後の向きもここで確定する）。
    if let Some(f) = &frame {
        cam.snap_to_direction(frame_view::view_direction(f.normal));
    }

    // ViewCube（右上）: 面クリック=標準ビュー / コーナークリック=アイソメへ即時スナップ。
    // モデルより手前の固定 UI のため、当たり判定を部材ピックより先に行い、
    // キューブ上のクリックはピック処理へ流さない。
    // 構面表示中は視点が固定のため出さない。
    let cube_layout = viewcube::Layout {
        center: egui::pos2(rect.max.x - 55.0, rect.min.y + 55.0),
        scale: 22.0,
    };
    let show_cube = frame.is_none();
    let cube_hover = show_cube
        .then(|| {
            response
                .hover_pos()
                .and_then(|p| viewcube::hit_test(&cam, &cube_layout, p))
        })
        .flatten();
    if cube_hover.is_some() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
    }
    let mut cube_clicked = false;
    if show_cube && response.clicked() {
        if let Some(pos) = response.interact_pointer_pos() {
            if let Some(hit) = viewcube::hit_test(&cam, &cube_layout, pos) {
                cam.snap_to_direction(viewcube::hit_direction(hit));
                cube_clicked = true;
            }
        }
    }

    let painter = ui.painter_at(rect);
    // §3-2: 3D 背景は白（ドックより淡く、モデルを主役にする）
    painter.rect_filled(rect, 0.0, theme::VIEW_BG);

    let center = [rect.center().x, rect.center().y];

    // 投影スケールとモデル中心（回転中心）。一様スケールで実寸比を保持する。
    // モデルが空でもグリッド・軸を描画するため早期 return はしない。
    // 構面表示中は、その構面に属する部材だけの外接直方体を基準にする（全体基準の
    // ままだと、大きな建物の 1 構面が小さく画面の端へ寄ってしまう）。
    let (bmin, bmax) = match &frame {
        Some(f) => frame_bbox(&app.model, f).unwrap_or_else(|| model_bbox(&app.model)),
        None => model_bbox(&app.model),
    };
    let center3 = [
        (bmin[0] + bmax[0]) * 0.5,
        (bmin[1] + bmax[1]) * 0.5,
        (bmin[2] + bmax[2]) * 0.5,
    ];
    let model_size = if frame.is_some() {
        bbox_diagonal(bmin, bmax)
    } else {
        model_bbox_size(&app.model)
    };
    let min_dim = rect.width().min(rect.height());
    let fit = if model_size > 1e-9 {
        0.8 * min_dim / model_size as f32
    } else {
        1.0
    };
    // 既定ズーム 3.0 でモデル対角が描画領域の約 80% に収まるよう基準化。
    let scale = fit * (cam.zoom / 3.0);
    // 以降の描画で共有する投影文脈（カメラ確定後に 1 度だけ構築）。
    let proj = Projector::new(center3, &cam, scale, center);

    // グリッド・軸（§3-2: 赤=X / 緑=Y / 青=Z）。モデルの背後に先に描く。
    // 構面表示中は、汎用の 1m 方眼の代わりに通り芯・階の基準線を描く
    // （同時に出すと線が二重になって読みづらいため）。
    match (&frame, app.frame_target) {
        (Some(f), Some(t)) => {
            frame_view::draw_frame_grid(&painter, &app.model, f, t, (bmin, bmax), &proj)
        }
        _ => draw_grid_and_axes(&painter, rect, &proj),
    }

    // 立体グリッド（通り芯 × 階レベルの平面格子）。方眼の上・架構の下に描き、
    // モデリングの下敷きとして見えるようにする。構面表示中は構面の基準線と
    // 二重になるため描かない。
    if app.show_space_grid && frame.is_none() {
        space_grid::draw(&painter, &proj, &app.model);
    }

    // 節点座標（変形・モード時と、応力図の変形重ね表示時は変位を加味）
    let disp = match mode {
        ViewMode::Deformed => app.current_static().map(|s| s.disp.clone()),
        ViewMode::Force if app.overlay_deform => app.current_static().map(|s| s.disp.clone()),
        // `ModalResult::shapes` は剛床等の縮約後独立自由度座標のため直接は使えない。
        // ソルバが節点×6へ展開済みの `node_shapes` を用いる。
        ViewMode::Mode => app
            .results
            .as_ref()
            .and_then(|r| r.modal.as_ref())
            .and_then(|m| m.node_shapes.get(mode_idx))
            .cloned(),
        // 時刻歴アニメーション: 現在フレーム（`app.th_frame`）の全節点変位（node 順、
        // 展開済み。`ThRecording::node_disp` は既に `Deformed` と同じ形の
        // `Vec<[f64;6]>` のため、以降の変形描画経路をそのまま流用できる）。
        // モデル編集後（中-1）は再解析するまでアニメーションを無効化し、無変形の
        // ままにする（`disp=None` で以降の変位加算・N/Q/M 重ねも行われない）。
        ViewMode::TimeHistory if !app.staleness.results_stale => app
            .results
            .as_ref()
            .and_then(|r| r.time_history.as_ref())
            .and_then(|t| t.recording.as_ref())
            .and_then(|rec| {
                rec.node_disp
                    .get(app.th_frame.min(rec.node_disp.len().saturating_sub(1)))
            })
            .cloned(),
        _ => None,
    };

    // 主架構要素に接続しない節点（スラブ境界・小梁支持点・二次部材の節点）は
    // 解析自由度が割り当てられず変位が常にゼロのため（`DofMap` 参照）、最寄りの
    // 主架構部材の変位から補間し、床・二次部材を変形へ追従させる。梁に載る節点は
    // 梁の Hermite 変形曲線上へ載る（`interpolate_unreferenced_disp`）。続けて剛床
    // 代表節点の鉛直変位をスレーブ平均で補い、代表点も床の変形へ追従させる。
    let disp = disp.map(|d| display_disp(&app.model, d, app.show_beam_interpolation));

    // 実効表示倍率（自動倍率 × 手動係数）。時刻歴アニメーションは記録全体の
    // ピーク変位から 1 回だけ算定した固定倍率を使う（高-2、[`time_history_deform_scale`]）。
    // それ以外は現在フレームの変位から都度算定する（[`deform_display_scale`]）。
    let deform_scale_actual = if mode == ViewMode::TimeHistory {
        if app.staleness.results_stale {
            0.0
        } else {
            time_history_deform_scale(app, model_size)
        }
    } else {
        deform_display_scale(
            &app.model,
            disp.as_deref(),
            model_size,
            app.show_beam_interpolation,
            app.deform_scale_factor,
        )
    };

    // 表示用の節点 3D 座標（変形図・モード形では変位を加味）。
    // 断面ソリッド描画でも 3D 座標が要るため、投影前の座標を保持する。
    let coords3: Vec<[f64; 3]> = app
        .model
        .nodes
        .iter()
        .enumerate()
        .map(|(i, node)| {
            let mut p = node.coord;
            if let Some(d) = &disp {
                p[0] += d[i][0] * deform_scale_actual;
                p[1] += d[i][1] * deform_scale_actual;
                p[2] += d[i][2] * deform_scale_actual;
            }
            p
        })
        .collect();
    let pts: Vec<egui::Pos2> = coords3.iter().map(|&p| proj.project(p)).collect();

    // 解析対象の節点（主架構要素が接続する節点・拘束のマスター節点）。判定規則は
    // 解析（`DofMap::build`）と共通。
    let structural = squid_n_core::dof::structural_nodes(&app.model);

    // 節点の表示可否。解析対象外の節点（スラブ境界・小梁支持点・二次部材の節点）は
    // 床・二次部材と一体の存在なので、「床・二次部材」トグル OFF では節点も描かない
    // （部材が消えて節点だけが空中に浮いて見えるのを防ぐ）。非表示の節点は
    // 作成モードのピック対象からも外し、見えない点が選ばれないようにする。
    // 構面表示中は、その構面に属さない節点も描かない。
    let node_visible: Vec<bool> = structural
        .iter()
        .enumerate()
        .map(|(i, &s)| {
            (s || app.show_floor_secondary)
                && frame
                    .as_ref()
                    .is_none_or(|f| f.node_on.get(i).copied().unwrap_or(false))
        })
        .collect();
    // 部材の描画対象。構面表示中はその構面に属する部材だけを描く
    // （判定規則は `squid_n_core::frame`）。全モード共通で参照する。
    let filter = FrameFilter::new(frame.as_ref());

    // --- クリック処理（ViewCube 上のクリックはスナップ済みのため除外） ---
    if response.clicked() && !cube_clicked {
        if let Some(click_pos) = response.interact_pointer_pos() {
            if app.load_pick_active() {
                // 荷重の対象ピック待ち：節点荷重なら節点、部材荷重なら部材を仮選択する
                // （確定は Enter。案内バーは `crate::load_editor`）。
                let picks_node = app.load_editor.as_ref().is_some_and(|e| e.picks_node());
                if picks_node {
                    // 節点ピッキング許容距離（px）
                    const NODE_PICK_THRESHOLD: f32 = 10.0;
                    if let Some((i, d)) = pick_nearest_node(&pts, &node_visible, click_pos) {
                        if d <= NODE_PICK_THRESHOLD {
                            let node_id = app.model.nodes[i].id;
                            if let Some(editor) = app.load_editor.as_mut() {
                                editor.set_picked_node(node_id);
                            }
                            app.nav.focus_node = Some(node_id);
                            app.selection.nodes = vec![node_id];
                        }
                    }
                } else {
                    // 部材ピッキング許容距離（px）
                    const PICK_THRESHOLD: f32 = 8.0;
                    if let Some((id, d)) = pick_nearest_member(&app.model, &pts, click_pos, filter)
                    {
                        if d <= PICK_THRESHOLD {
                            // 壁・スラブ等の非線材には部材荷重を載せられない
                            // （`is_member_load_target` と同じ集合に限る）。
                            if member_load_pickable(&app.model, id) {
                                // モデルの不変借用はここで終える（`set_picked_member`
                                // へはブレースか否かの判定結果だけを渡す）。
                                let is_brace = crate::load_editor::is_brace(&app.model, id);
                                if let Some(editor) = app.load_editor.as_mut() {
                                    editor.set_picked_member(id, is_brace);
                                }
                                app.nav.focus_member = Some(id);
                                app.selection.members = vec![id];
                            }
                        }
                    }
                }
            } else if app.beam_draw_mode {
                // 梁作成モード：クリック位置を既存節点または格子点へスナップする。
                // グリッド表示が OFF のときは、見えていない格子点を拾わないよう
                // 既存節点だけを対象にする。
                // 構面表示中は格子を描かないため、スナップの対象からも外す
                // （正射影で重なった別構面の格子点を拾い、見ていない構面へ
                // 節点と梁を作ってしまう）。
                let picked = if app.show_space_grid && frame.is_none() {
                    space_grid::pick(&app.model, &proj, &pts, &node_visible, click_pos)
                } else {
                    // 節点ピッキング許容距離（px）
                    const NODE_PICK_THRESHOLD: f32 = 10.0;
                    pick_nearest_node(&pts, &node_visible, click_pos)
                        .filter(|(_, d)| *d <= NODE_PICK_THRESHOLD)
                        .map(|(i, _)| space_grid::SnapPoint::Node(app.model.nodes[i].id))
                };
                if let Some(point) = picked {
                    match app.beam_draw_first {
                        None => {
                            // 1 点目：始点として記憶（この時点ではモデルを変更しない）
                            app.beam_draw_first = Some(point);
                        }
                        Some(first) => {
                            // 2 点目：始点と異なれば梁を生成。節点のない格子点は
                            // 節点追加とあわせて 1 回の undo にまとめる。
                            if let Some((cmd, new_id)) =
                                space_grid::beam_command(&app.model, first, point)
                            {
                                app.undo.run(&mut app.model, Box::new(cmd));
                                app.staleness.mark_edited();
                                app.nav.focus_member = Some(new_id);
                            }
                            // 次の梁に備えて始点をリセット
                            app.beam_draw_first = None;
                        }
                    }
                }
            } else if app.wall_draw_mode {
                // 壁作成モード：クリック位置に最も近い節点を選ぶ
                let best = pick_nearest_node(&pts, &node_visible, click_pos);
                // 節点ピッキング許容距離（px）
                const NODE_PICK_THRESHOLD: f32 = 10.0;
                if let Some((i, d)) = best {
                    if d <= NODE_PICK_THRESHOLD {
                        let node_id = app.model.nodes[i].id;
                        // 同一節点の重複選択は無視
                        if !app.wall_draw_nodes.contains(&node_id) {
                            app.wall_draw_nodes.push(node_id);
                        }
                        // 4 点そろったら壁を生成
                        if app.wall_draw_nodes.len() == 4 {
                            let ordered = order_wall_nodes(&app.model, &app.wall_draw_nodes);
                            let new_id = squid_n_core::ids::ElemId(app.model.elements.len() as u32);
                            let elem = squid_n_core::model::ElementData {
                                id: new_id,
                                kind: squid_n_core::model::ElementKind::Wall,
                                nodes: ordered.into_iter().collect(),
                                section: None,
                                local_axis: squid_n_core::model::LocalAxis {
                                    ref_vector: [0.0, 0.0, 1.0],
                                },
                                end_cond: [
                                    squid_n_core::model::EndCondition::Fixed,
                                    squid_n_core::model::EndCondition::Fixed,
                                ],
                                force_regime: squid_n_core::model::ForceRegime::Auto,
                                rigid_zone: Default::default(),
                                plastic_zone: None,
                                spring: None,
                            };
                            app.undo
                                .run(&mut app.model, Box::new(squid_n_edit::AddMember { elem }));
                            app.staleness.mark_edited();
                            app.nav.focus_member = Some(new_id);
                            app.wall_draw_nodes.clear();
                        }
                    }
                }
            } else if app.slab_draw_mode {
                // スラブ作成モード：クリック位置に最も近い節点を外周順に追加する。
                let best = pick_nearest_node(&pts, &node_visible, click_pos);
                // 節点ピッキング許容距離（px）
                const NODE_PICK_THRESHOLD: f32 = 10.0;
                if let Some((i, d)) = best {
                    if d <= NODE_PICK_THRESHOLD {
                        let node_id = app.model.nodes[i].id;
                        // 同一節点の重複選択は無視（外周は各節点1回）。
                        if !app.slab_draw_nodes.contains(&node_id) {
                            app.slab_draw_nodes.push(node_id);
                        }
                    }
                }
            } else {
                // 通常モード：クリック位置に最も近い部材線分を選び、閾値内なら選択。
                // ピッキング許容距離（px）
                const PICK_THRESHOLD: f32 = 8.0;
                match pick_nearest_member(&app.model, &pts, click_pos, filter) {
                    Some((id, d)) if d <= PICK_THRESHOLD => {
                        app.selection.members = vec![id];
                        app.nav.focus_member = Some(id);
                        // ヒンジ図モードでは、クリックした部材のヒンジ詳細ウィンドウを開く。
                        if mode == ViewMode::Hinge {
                            app.hinge_detail_elem = Some(id);
                        }
                        // 時刻歴モードでは、クリックした部材の履歴・検定ウィンドウを開く
                        // （中-1(a): モデル編集後は添字ずれの恐れがあるため無効化）。
                        if mode == ViewMode::TimeHistory && !app.staleness.results_stale {
                            app.th_detail_elem = Some(id);
                        }
                    }
                    _ => {
                        app.selection.members.clear();
                    }
                }
            }
        }
    }

    // --- スラブ・小梁 ---
    // 荷重分配オブジェクト（解析部材ではない）であることが分かるよう、
    // 構造部材（実線・青/グレー系）と異なる暖色半透明フィル＋破線のフォーマットで描く。
    // 部材線・断面ソリッドより先に描き、架構が床の上に重なるようにする。
    // CMQ 図は全体解析（主架構）に関するものなので、小梁・スラブは表示しない。
    // 「床・二次部材」トグル OFF 時も表示しない。
    //
    // 床・二次部材（スラブ・小梁・間柱）の表示可否は、中心線と断面ソリッドで共通の
    // 判定とする（断面表示だけがトグルを無視して小梁を描いてしまわないよう、判定を
    // ここで 1 つの変数に集約して各描画へ渡す）。
    let lumped_only = lumped::is_lumped_view(mode) && !app.lumped_show_frame;
    let show_secondary = !lumped_only && mode != ViewMode::Cmq && app.show_floor_secondary;
    if show_secondary {
        draw_slabs_and_joists(&painter, app, &pts, filter);
    }

    // --- 断面ソリッド ---
    // 節点・部材線より先に描き、線・シンボル類は上に重ねる（材軸が見えるように）。
    let mut solids_skipped = 0usize;
    if app.show_sections && !lumped_only {
        solids_skipped = solid::draw_section_solids(
            &painter,
            &app.model,
            &coords3,
            &proj,
            show_secondary,
            filter,
        );
    }

    // モード形は変形前を破線・高透過で先に描き、基準位置からの変化が読めるようにする。
    // 質点モードの変形前串と同じ規約（破線 6/4 pt、線アルファ 90）。
    if !lumped_only && mode == ViewMode::Mode && deform_scale_actual > 1e-12 {
        let pts_rest: Vec<egui::Pos2> = app
            .model
            .nodes
            .iter()
            .map(|n| proj.project(n.coord))
            .collect();
        draw_mode_rest_ghost(
            &painter,
            app,
            &pts_rest,
            &node_visible,
            filter,
            show_secondary,
            app.show_sections,
        );
    }

    // 節点（梁/壁作成モードで選択中の節点・選択中の節点は強調表示）。
    // 解析対象外の節点は「床・二次部材」トグルに追従して表示・非表示を切り替える。
    if !lumped_only {
        for (i, &p) in pts.iter().enumerate() {
            if !node_visible[i] {
                continue;
            }
            let node_id = app.model.nodes[i].id;
            let is_first = app.beam_draw_first == Some(space_grid::SnapPoint::Node(node_id));
            let is_wall_pick = app.wall_draw_nodes.contains(&node_id);
            let is_slab_pick = app.slab_draw_nodes.contains(&node_id);
            // 節点の選択（ナビゲータの荷重ツリー・荷重の対象ピック）。部材の選択
            // ハイライトと対をなし、どの節点が対象なのかを 3D 上で示す。
            let is_selected = app.selection.nodes.contains(&node_id);
            let (radius, color) = if is_first || is_wall_pick || is_slab_pick {
                // 作成モードで選択中の節点 = 重要（赤）
                (5.0, theme::PARETO_RED)
            } else if is_selected {
                // 選択中の節点 = 結果の強調（ハイライト紫。部材の選択色と揃える）
                (5.0, theme::HILITE_PURPLE)
            } else {
                // 通常の節点 = データ点（青）
                (3.0, theme::DATA_BLUE)
            };
            painter.circle_filled(egui::pos2(p[0], p[1]), radius, color);
        }

        // 梁作成モードの始点が節点のない格子点の場合、まだモデルに節点が無いため
        // 上のループでは描かれない。選択中であることが分かるよう、同じ色で印を置く。
        if let Some(space_grid::SnapPoint::Grid(c)) = app.beam_draw_first {
            painter.circle_stroke(
                proj.project(c),
                5.0,
                egui::Stroke::new(2.0_f32, theme::PARETO_RED),
            );
        }

        // 部材（線）
        let line_color = if matches!(mode, ViewMode::Deformed | ViewMode::Mode) {
            // 変形図・モード形 = 結果の強調（ハイライト紫）
            theme::HILITE_PURPLE
        } else {
            // 通常の部材 = 沈めたニュートラル（gray-700）
            theme::GRAY_700
        };
        // 断面表示中は中心線を細く淡くし、ソリッドの上に材軸として薄く重ねる
        let line_stroke = if app.show_sections {
            egui::Stroke::new(1.0_f32, theme::translucent(line_color, 110))
        } else {
            egui::Stroke::new(2.0_f32, line_color)
        };
        for elem in &app.model.elements {
            if !filter.shows(elem.id) {
                continue;
            }
            // 壁・シェル（面要素）は半透明ポリゴンで描画
            if element_draw_shape(elem.kind) == DrawShape::Polygon && elem.nodes.len() >= 3 {
                let poly: Vec<egui::Pos2> = elem
                    .nodes
                    .iter()
                    .filter_map(|n| {
                        let idx = n.index();
                        (idx < pts.len()).then(|| pts[idx])
                    })
                    .collect();
                if poly.len() == elem.nodes.len() {
                    painter.add(egui::Shape::convex_polygon(
                        poly,
                        theme::translucent(theme::DATA_BLUE, 50),
                        egui::Stroke::new(1.5_f32, theme::DATA_BLUE),
                    ));
                }
                continue;
            }
            // 線材でない要素（面要素・仕口パネル）は材軸を持たないため線で描かない
            // （`draws_as_line`）。特に仕口パネルの節点列は「接合部の節点 ＋ 取り付く
            // 部材の他端」なので、先頭 2 節点を結ぶと取り付く柱・梁とまったく同じ線分に
            // なる。全部材を直線で描くうちは実部材と重なって見えないが、内部たわみ表示で
            // 梁・柱を曲線にすると弦の直線だけが残り、部材が二重に描かれて見えてしまう。
            if !draws_as_line(elem.kind) || elem.nodes.len() < 2 {
                continue;
            }
            let n0 = elem.nodes[0].index();
            let n1 = elem.nodes[1].index();
            if n0 >= pts.len() || n1 >= pts.len() {
                continue;
            }

            // 変形を表示する全モード（変形図・モード形・応力図の変形重ね）で、「内部
            // たわみ」トグルが ON のとき、梁は端部の並進・回転から Hermite 3 次で曲げ
            // 変形を内挿して曲線描画する（節点間の直線ではたわみが見えないため）。
            // トグル OFF では梁も直線で描き、全体の変形だけを素直に見る。変形を表示
            // していない（`disp` が None）モードでは常に直線。
            let curved_beam =
                app.show_beam_interpolation && elem.kind == squid_n_core::model::ElementKind::Beam;
            if let (true, Some(d)) = (curved_beam, &disp) {
                let p_i = app.model.nodes[n0].coord;
                let p_j = app.model.nodes[n1].coord;
                if member_len3(p_i, p_j) > 1e-9 {
                    let poly3 =
                        BeamDeflection::new(p_i, p_j, d[n0], d[n1], elem.local_axis.ref_vector)
                            .polyline(deform_scale_actual, DEFORM_CURVE_SEGMENTS);
                    let screen: Vec<egui::Pos2> = poly3.iter().map(|&p| proj.project(p)).collect();
                    painter.add(egui::Shape::line(screen, line_stroke));
                    continue;
                }
            }

            // 通常（未変形・その他要素・ゼロ長梁）は節点間を直線で結ぶ。
            painter.line_segment([pts[n0], pts[n1]], line_stroke);
        }

        // 二次部材（小梁・間柱）: 解析対象外だが実在部材なので実線で描く
        // （解析対象外を示す暖色アンバー。スラブの暖色と同族で、主架構の
        // 青/グレーと弁別。断面表示中はソリッドが上に描かれているため
        // 材軸線として薄く重ねる）。
        if show_secondary {
            let secondary_stroke = if app.show_sections {
                egui::Stroke::new(1.0_f32, theme::translucent(theme::SECONDARY_AMBER, 110))
            } else {
                egui::Stroke::new(1.5_f32, theme::SECONDARY_AMBER)
            };
            for sm in &app.model.secondary_members {
                let n0 = sm.nodes[0].index();
                let n1 = sm.nodes[1].index();
                if !filter.shows_node(n0) || !filter.shows_node(n1) {
                    continue;
                }
                if n0 < pts.len() && n1 < pts.len() {
                    painter.line_segment([pts[n0], pts[n1]], secondary_stroke);
                }
            }
        }
    }

    if lumped::is_lumped_view(mode) {
        lumped::draw(&painter, app, &proj, mode, mode_idx, model_size);
    }

    // 構面に部材が 1 本もない場合の注記。ST-Bridge から取り込んだ、所属節点を
    // 持たない通り（`Y0`・`X2a` など）を選ぶと空の図になるため、モデルや表示の
    // 不具合と紛れないよう理由を示す。
    if let Some(f) = &frame {
        if f.elem_count() == 0 {
            painter.text(
                painter.clip_rect().center(),
                egui::Align2::CENTER_CENTER,
                format!("{} に属する部材はありません", f.label),
                egui::FontId::proportional(13.0),
                theme::GRAY_600,
            );
        }
    }

    // 断面を描けなかった線材（断面未割当・形状情報なし）があれば右上に注記
    if app.show_sections && solids_skipped > 0 {
        painter.text(
            egui::pos2(
                painter.clip_rect().max.x - 10.0,
                painter.clip_rect().min.y + 10.0,
            ),
            egui::Align2::RIGHT_TOP,
            format!("断面未定義の部材 {} 本は線のみ表示", solids_skipped),
            egui::FontId::proportional(11.0),
            theme::GRAY_600,
        );
    }

    // --- 応力図（N/Q/M）: 部材ローカルに沿って描画 ---
    // 変形重ね（`disp` が Some）かつ内部たわみ表示が有効なとき、梁の張り出しは
    // 変形後の Hermite 曲線を基準線に描く。判定に必要な変位と表示倍率を渡す。
    if mode == ViewMode::Force {
        diagram::draw_force_diagram(
            &painter,
            app,
            app.force_components,
            &coords3,
            disp.as_deref(),
            deform_scale_actual,
            &proj,
            filter,
            frame.as_ref().map(|f| f.normal),
        );
    }
    if mode == ViewMode::Cmq {
        draw_cmq_diagram(
            &painter,
            app,
            &coords3,
            &proj,
            filter,
            frame.as_ref().map(|f| f.normal),
        );
    }
    if mode == ViewMode::Modeling {
        modeling::draw_modeling(&painter, app, &pts, &coords3, &proj, filter);
        // ホバー詳細（ViewCube ホバー中は除く。検定比図と同じ最近傍部材探索・
        // 8px 閾値で最寄り部材を求め、ヒットしたらモデル化の詳細を表示）。
        if cube_hover.is_none() {
            if let Some(hover_pos) = response.hover_pos() {
                const HOVER_PICK_THRESHOLD: f32 = 8.0;
                if let Some((id, d)) = pick_nearest_member(&app.model, &pts, hover_pos, filter) {
                    if d <= HOVER_PICK_THRESHOLD {
                        modeling::show_modeling_tooltip(ui, app, id);
                    }
                }
            }
        }
    }
    if mode == ViewMode::CheckRatio {
        check_ratio::draw_check_ratio(&painter, app, &pts, filter);
        // B-3: ホバー詳細（ViewCube ホバー中は除く。通常モードのクリック選択と
        // 同じ最近傍部材探索・8px 閾値で最寄り部材を求め、ヒットしたらツールチップ表示）。
        //
        // 節点検定（接合部・仕口パネル・耐震壁）は部材の線とは別に節点位置へ
        // 描くため、節点を先に判定する（節点マーカーの上にポインタがあるときは
        // 節点の詳細を優先する。マーカー半径より少し広い閾値で拾う）。
        if cube_hover.is_none() {
            if let Some(hover_pos) = response.hover_pos() {
                const HOVER_PICK_THRESHOLD: f32 = 8.0;
                let node_hit = check_ratio::pick_nearest_checked_node(app, &pts, hover_pos, filter)
                    .filter(|&(_, d)| d <= check_ratio::NODE_HOVER_THRESHOLD);
                if let Some((idx, _)) = node_hit {
                    if let Some(node) = app.model.nodes.get(idx) {
                        check_ratio::show_node_check_tooltip(ui, app, node.id);
                    }
                } else if let Some((id, d)) =
                    pick_nearest_member(&app.model, &pts, hover_pos, filter)
                {
                    if d <= HOVER_PICK_THRESHOLD {
                        check_ratio::show_check_tooltip(ui, app, id);
                    }
                }
            }
        }
    }
    if mode == ViewMode::Hinge {
        hinge::draw_hinge(&painter, app, &pts, filter);
        // ホバー詳細（ViewCube ホバー中は除く。検定比図と同じ最近傍部材探索・
        // 8px 閾値で最寄り部材を求め、ヒットしたらヒンジ詳細を表示）。
        if cube_hover.is_none() {
            if let Some(hover_pos) = response.hover_pos() {
                const HOVER_PICK_THRESHOLD: f32 = 8.0;
                if let Some((id, d)) = pick_nearest_member(&app.model, &pts, hover_pos, filter) {
                    if d <= HOVER_PICK_THRESHOLD {
                        hinge::show_hinge_tooltip(ui, app, id);
                    }
                }
            }
        }
    }

    // 変形の実効倍率（自動倍率 × 手動係数）の注記。
    // 実変位を表示している時のみ描く（モード形は固有ベクトルの規模が任意のため
    // 倍率に物理的な意味がなく、表示しない）。質点時刻歴は質点変位のピークから
    // 別に算定した倍率を使う。
    let scale_note = if mode == ViewMode::LumpedTimeHistory {
        lumped::display_scale(app, mode, mode_idx, model_size)
    } else if deform_scale_actual > 0.0 && mode != ViewMode::Mode {
        deform_scale_actual
    } else {
        0.0
    };
    if scale_note > 0.0 {
        // N/Q/M 図の凡例（min.y+10）・コンターバー＋ラベル（min.y+30〜56 程度）と
        // 重ならない位置へ
        let y = match mode {
            ViewMode::Force if app.diagram_contour => 70.0,
            ViewMode::Force => 30.0,
            _ => 10.0,
        };
        // 手動係数が 1.0 のときは「自動」、それ以外は「自動×係数」を併記する。
        let note = if (app.deform_scale_factor - 1.0).abs() < 1e-3 {
            format!("変形倍率 ×{:.0}（自動）", scale_note)
        } else {
            format!(
                "変形倍率 ×{:.0}（自動×{:.2}）",
                scale_note, app.deform_scale_factor
            )
        };
        painter.text(
            egui::pos2(
                painter.clip_rect().min.x + 10.0,
                painter.clip_rect().min.y + y,
            ),
            egui::Align2::LEFT_TOP,
            note,
            egui::FontId::proportional(12.0),
            theme::GRAY_600,
        );
    }

    // 選択ハイライト（描き方の規約は `element_draw_shape`）。
    for &elem_id in &app.selection.members {
        let Some(elem) = app.model.element(elem_id) else {
            continue;
        };
        let stroke = egui::Stroke::new(4.0_f32, theme::PARETO_RED);
        match element_draw_shape(elem.kind) {
            DrawShape::None => {}
            DrawShape::Polygon => {
                // 面要素は輪郭を閉じた折れ線で強調する（塗りは通常描画のまま）。
                let poly: Vec<egui::Pos2> = elem
                    .nodes
                    .iter()
                    .filter_map(|n| {
                        let idx = n.index();
                        (idx < pts.len()).then(|| pts[idx])
                    })
                    .collect();
                if poly.len() >= 3 && poly.len() == elem.nodes.len() {
                    painter.add(egui::Shape::closed_line(poly, stroke));
                }
            }
            DrawShape::Line => {
                if elem.nodes.len() < 2 {
                    continue;
                }
                let n0 = elem.nodes[0].index();
                let n1 = elem.nodes[1].index();
                if n0 < pts.len() && n1 < pts.len() {
                    painter.line_segment([pts[n0], pts[n1]], stroke);
                }
            }
        }
    }

    // --- 剛床代表点（トグル ON 時）: 代表点マーカーと関連スレーブへの点線 ---
    // 剛床代表節点（重心マスター）は面内挙動を担う仮想節点で実部材に接続しない。
    // 関連付けられたスレーブ節点へ点線を引き、所属関係を可視化する。代表点・
    // スレーブとも変形後座標（`coords3` 由来の `pts`）で描くため変形へ追従する。
    // 点線は節点数が多いと他部材が見づらくなるため、トグルで表示を切り替える。
    //
    // 階の生成（`story_gen`）は当該レベルの全節点をスレーブに登録するため、
    // スラブ境界・小梁支持点・二次部材の節点（解析自由度を持たない）も
    // スレーブ一覧に含まれる。これらは剛床の縮約対象にならない（`DofMap` が
    // 全自由度を不活性にし、拘束行列の生成も `dofmap.active` で素通りする）ので、
    // 点線は解析対象の節点に限って描く。
    if app.show_diaphragm_master {
        const DASH: f32 = 5.0;
        const GAP: f32 = 4.0;
        for c in &app.model.constraints {
            let squid_n_core::model::Constraint::RigidDiaphragm { master, slaves, .. } = c else {
                continue;
            };
            let mi = master.index();
            if mi >= pts.len() || !filter.shows_node(mi) {
                continue;
            }
            let mp = pts[mi];
            for sl in slaves {
                let si = sl.index();
                if si >= pts.len() || !structural.get(si).copied().unwrap_or(false) {
                    continue;
                }
                painter.extend(egui::Shape::dashed_line(
                    &[mp, pts[si]],
                    egui::Stroke::new(1.0_f32, theme::translucent(theme::HILITE_PURPLE, 140)),
                    DASH,
                    GAP,
                ));
            }
            // 代表点マーカー（強調リング）。通常の青節点の上に紫リングを重ねる。
            painter.circle_stroke(mp, 6.0, egui::Stroke::new(2.0_f32, theme::HILITE_PURPLE));
        }
    }

    // --- 支持条件シンボル ---
    // 固定方向へ軸色の矢印、回転軸まわりに円弧を描く。
    // 部材・応力図の上に重ねて描き、支持方向を一目で判別できるようにする。
    // スクリーン上で矢印 18px・円弧半径 12px になるようワールド長を逆算する。
    //
    // 質点モード・質点時刻歴では立体の柱脚拘束は串に関係ないので出さない。
    // ほかの表示はツールバー「支点」トグル（既定 ON）に従う。剛床代表点の面内拘束
    // マークは「剛床代表点」トグル側で、支点トグルとは独立に出す。
    //
    // 剛床（RigidDiaphragm）マスター節点は特別扱いする。マスターに設定される
    // 拘束（Uz/Rx/Ry）は零剛性自由度による特異行列を避けるための数値上の
    // ダミー拘束であり、剛床が物理的に拘束するのは面内自由度（Ux/Uy/Rz）。
    // そのため剛床マークはダミー拘束ではなく面内拘束（Ux/Uy/Rz）を表示する
    // （支点拘束との整合。従来はダミー拘束をそのまま描き、剛床が拘束しない
    // 自由度を表示していた）。
    const SUPPORT_ARROW_PX: f32 = 18.0;
    const SUPPORT_ARC_PX: f32 = 12.0;
    let lumped_view = lumped::is_lumped_view(mode);
    let draw_supports = supports_visible(lumped_view, app.show_supports);
    // 剛床マスター節点の index 集合。
    let diaphragm_masters: std::collections::HashSet<usize> = app
        .model
        .constraints
        .iter()
        .filter_map(|c| match c {
            squid_n_core::model::Constraint::RigidDiaphragm { master, .. } => Some(master.index()),
            _ => None,
        })
        .collect();
    // 剛床の面内拘束マスク（Ux, Uy, Rz）。
    let diaphragm_mask = {
        let mut m = Dof6Mask::FREE;
        m.set_fixed(Dof::Ux);
        m.set_fixed(Dof::Uy);
        m.set_fixed(Dof::Rz);
        m
    };
    let mut has_support = false;
    let mut has_diaphragm = false;
    if !lumped_view {
        for (i, node) in app.model.nodes.iter().enumerate() {
            if !filter.shows_node(i) {
                continue;
            }
            let is_master = diaphragm_masters.contains(&i);
            // 剛床マスターの面内拘束マークは代表点トグル ON 時のみ描く（既定は非表示に
            // して他部材を見やすくする。点線・マーカーと表示を一致させる）。
            if is_master && !app.show_diaphragm_master {
                continue;
            }
            if !is_master && !draw_supports {
                continue;
            }
            // 表示する拘束: 剛床マスターは面内拘束（Ux/Uy/Rz）、それ以外は節点拘束。
            let restraint = if is_master {
                diaphragm_mask
            } else {
                node.restraint
            };
            if support_kind(restraint) == SupportKind::Free {
                continue;
            }
            // 支点シンボルは変形後座標に描く。実支点は変位ゼロで原位置に留まり、
            // 剛床マスターは床の面内変形に追従する（剛床の重心マークが変形へ移動する）。
            let coord = coords3.get(i).copied().unwrap_or(node.coord);
            if is_master {
                has_diaphragm = true;
            } else {
                has_support = true;
            }
            draw_support_symbol(
                &painter,
                &proj,
                coord,
                restraint,
                SUPPORT_ARROW_PX,
                SUPPORT_ARC_PX,
            );
        }
    }

    // --- 支点ばね記号 ---
    // 拘束で固定済みの成分は上のループで従来の矢印・円弧を描画済みのため、
    // ここでは非固定かつばね値が非ゼロの成分にのみジグザグ（並進）・渦巻（回転）を描く。
    // 剛床マスター節点はダミー拘束の仮想節点でありばね支持を持たないため対象外。
    let mut has_spring = false;
    if draw_supports {
        for (i, node) in app.model.nodes.iter().enumerate() {
            if diaphragm_masters.contains(&i) || !filter.shows_node(i) {
                continue;
            }
            let Some(spring) = node.support_spring else {
                continue;
            };
            let coord = coords3.get(i).copied().unwrap_or(node.coord);
            has_spring = true;
            support_symbols::draw_spring_symbol(
                &painter,
                &proj,
                coord,
                node.restraint,
                &spring,
                SUPPORT_ARROW_PX,
                SUPPORT_ARC_PX,
            );
        }
    }

    // --- 免震支承マーカー ---
    // 支点配置は「接地節点（restraint=FIXED）と対象節点の間の零長 Isolator 要素」
    // （`support_symbols::support_isolators` が判定）。対象節点側にマーカーを描く。
    let support_isolators = if draw_supports {
        support_symbols::support_isolators(&app.model)
    } else {
        Vec::new()
    };
    let has_isolator = !support_isolators.is_empty();
    for &(idx, _elem_id, _props) in support_isolators
        .iter()
        .filter(|&&(i, _, _)| filter.shows_node(i))
    {
        let coord = coords3
            .get(idx)
            .copied()
            .unwrap_or_else(|| app.model.nodes[idx].coord);
        support_symbols::draw_isolator_marker(&painter, proj.project(coord), theme::ISOLATOR_TEAL);
    }
    // 免震支承マーカーのホバー詳細（ViewCube ホバー中は除く。節点近傍・8px 閾値）。
    if cube_hover.is_none() {
        if let Some(hover_pos) = response.hover_pos() {
            const HOVER_PICK_THRESHOLD: f32 = 8.0;
            let nearest = support_isolators
                .iter()
                .filter_map(|&(idx, elem_id, props)| {
                    pts.get(idx)
                        .map(|&p| (elem_id, props, (hover_pos - p).length()))
                })
                .filter(|&(_, _, d)| d <= HOVER_PICK_THRESHOLD)
                .min_by(|a, b| a.2.total_cmp(&b.2));
            if let Some((elem_id, props, _)) = nearest {
                support_symbols::show_isolator_tooltip(ui, elem_id, &props);
            }
        }
    }

    if has_support || has_diaphragm || has_spring || has_isolator {
        draw_support_legend(&painter, has_diaphragm, has_spring, has_isolator);
    }

    // 右上に ViewCube、右下にカメラ追従の座標系アイコン（常に手前に表示。
    // 左下は支持条件凡例が使うため、これらは右側へ配置する）
    if show_cube {
        viewcube::draw(&painter, &cam, &cube_layout, cube_hover);
    }
    draw_axis_gadget(&painter, &cam);

    // カメラ状態を保存
    app.camera = cam;

    // ヒンジ詳細ウィンドウ（ヒンジ図でクリックした部材があれば表示。表示中は
    // 他の表示モードへ切り替えても閉じるまで残す）。
    hinge::show_hinge_detail_window(ui, app);
    // 時刻歴詳細ウィンドウ（時刻歴モードでクリックした部材があれば表示）。
    th_detail::show_th_detail_window(ui, app);
}

fn frame_range_controls(ui: &mut egui::Ui, app: &mut App) {
    use squid_n_core::frame::FrameTarget;

    // 選択肢（通り・階）を平坦な一覧にする。前後送りはこの並び順で行う。
    let mut choices: Vec<(FrameTarget, String)> = Vec::new();
    for (gi, group) in app.model.axes.iter().enumerate() {
        for (ai, ax) in group.axes.iter().enumerate() {
            choices.push((
                FrameTarget::Axis {
                    group: gi,
                    axis: ai,
                },
                format!("{} / {}", group.name, ax.name),
            ));
        }
    }
    let axis_count = choices.len();
    for st in &app.model.stories {
        choices.push((FrameTarget::Story(st.id), st.name.clone()));
    }
    if choices.is_empty() {
        return;
    }

    ui.horizontal_wrapped(|ui| {
        ui.label("表示範囲:");
        let mut target = app.frame_target;
        if ui
            .selectable_label(target.is_none(), "全体(3D)")
            .on_hover_text("モデル全体を 3D で表示します")
            .clicked()
        {
            target = None;
        }
        // 現在の選択の位置（一覧内の添字）。
        let cur = target.and_then(|t| choices.iter().position(|(c, _)| *c == t));
        let label = cur
            .map(|i| choices[i].1.clone())
            .unwrap_or_else(|| "選択…".to_string());
        let is_frame = target.is_some();
        ui.scope(|ui| {
            if !is_frame {
                ui.style_mut().visuals.override_text_color = Some(theme::GRAY_600);
            }
            egui::ComboBox::from_id_salt("frame_target")
                .selected_text(label)
                .show_ui(ui, |ui| {
                    for (i, (t, name)) in choices.iter().enumerate() {
                        if i == 0 && axis_count > 0 {
                            ui.label(egui::RichText::new("通り（軸組図）").size(11.0));
                        }
                        if i == axis_count && axis_count < choices.len() {
                            ui.separator();
                            ui.label(egui::RichText::new("階（伏図）").size(11.0));
                        }
                        if ui.selectable_label(target == Some(*t), name).clicked() {
                            target = Some(*t);
                        }
                    }
                })
                .response
                .on_hover_text(
                    "選んだ通り・階の構面だけを、正対の 2D で表示します。\
                     表示モード（形状・変形・応力図など）はそのまま使えます。",
                );
        });
        // 前後送り（X1 → X2 → … と順に見ていく操作）。
        let step = |target: &mut Option<FrameTarget>, delta: i64| {
            let n = choices.len() as i64;
            let cur = target
                .and_then(|t| choices.iter().position(|(c, _)| *c == t))
                .map(|i| i as i64)
                .unwrap_or(if delta > 0 { -1 } else { n });
            let next = (cur + delta).rem_euclid(n) as usize;
            *target = Some(choices[next].0);
        };
        if ui.button("◀").on_hover_text("前の通り／階").clicked() {
            step(&mut target, -1);
        }
        if ui.button("▶").on_hover_text("次の通り／階").clicked() {
            step(&mut target, 1);
        }
        if target.is_some() {
            ui.add_enabled(
                false,
                egui::Label::new(
                    egui::RichText::new("左ドラッグ:移動 / スクロール:ズーム（回転なし）")
                        .size(11.0),
                ),
            );
        }
        // 対象を切り替えたら、その構面へ自動でフィットし直す（パン・ズームを既定へ
        // 戻す）。前の構面で寄せた表示のまま切り替えると、次の構面が画面外へ
        // 外れたままになるため。
        if app.frame_target != target {
            app.camera.pan = [0.0, 0.0];
            app.camera.zoom = 3.0;
        }
        app.frame_target = target;
    });
}

#[cfg(test)]
mod frame_filter_tests {
    use super::*;
    use squid_n_core::frame::Frame;
    use squid_n_core::ids::ElemId;

    /// 全体表示（構面なし）では、すべての部材・節点を描く。
    /// 既定値がこの意味を持つことは、テストや将来の描画経路が
    /// `FrameFilter::default()` を使ったときの安全側の挙動として重要。
    #[test]
    fn no_frame_shows_everything() {
        let f = FrameFilter::default();
        assert!(f.shows(ElemId(0)));
        assert!(f.shows(ElemId(9999)));
        assert!(f.shows_node(0));
        assert!(f.shows_node(9999));
    }

    /// 構面表示では、その構面に属する部材・節点だけを描く。範囲外の添字は
    /// 「描かない」に倒す（陳腐化した参照で誤って描かないため）。
    #[test]
    fn frame_limits_to_members_on_it() {
        let frame = Frame {
            label: "X1 通り".into(),
            normal: [1.0, 0.0, 0.0],
            node_on: vec![true, false, true],
            elem_on: vec![false, true],
        };
        let f = FrameFilter::new(Some(&frame));
        assert!(!f.shows(ElemId(0)));
        assert!(f.shows(ElemId(1)));
        assert!(!f.shows(ElemId(2)), "範囲外は描かない");
        assert!(f.shows_node(0));
        assert!(!f.shows_node(1));
        assert!(f.shows_node(2));
        assert!(!f.shows_node(3), "範囲外は描かない");
    }

    /// 応力図の張り出しを構面内へ倒す: 面外へ向いた張り出し方向が、材軸に直交する
    /// 面内の向きへ回り、符号は元の向きに従う。
    #[test]
    fn in_plane_offset_flattens_out_of_plane_direction() {
        // X=0 の構面（法線 +X）に載る、Y 方向の梁。
        let (p_i, p_j) = ([0.0, 0.0, 4000.0], [0.0, 6000.0, 4000.0]);
        let n = [1.0, 0.0, 0.0];

        // 弱軸曲げの張り出し（局所 ez = X 方向）は視線方向に潰れる。倒すと
        // 材軸（+Y）にも法線（+X）にも直交する ±Z へ向く。
        let out = scene::in_plane_offset_dir([1.0, 0.0, 0.0], p_i, p_j, n);
        assert!(out[2].abs() > 0.999, "面内（鉛直）へ倒れる: {out:?}");
        assert!(out[0].abs() < 1e-9 && out[1].abs() < 1e-9);

        // 元から面内（+Z）の張り出しは向きが変わらない。
        let keep = scene::in_plane_offset_dir([0.0, 0.0, 1.0], p_i, p_j, n);
        assert!((keep[2] - 1.0).abs() < 1e-9, "{keep:?}");
        // 逆向き（−Z）なら符号もそのまま保つ。
        let flip = scene::in_plane_offset_dir([0.0, 0.0, -1.0], p_i, p_j, n);
        assert!((flip[2] + 1.0).abs() < 1e-9, "{flip:?}");
    }

    /// 材軸が構面の法線と平行な部材（伏図の柱）は、面内に張り出し方向を採れないため
    /// 元の向きのままにする（0 除算で向きを失わせない）。
    #[test]
    fn in_plane_offset_keeps_direction_for_members_piercing_the_frame() {
        let (p_i, p_j) = ([0.0, 0.0, 0.0], [0.0, 0.0, 4000.0]);
        let dir = [1.0, 0.0, 0.0];
        let out = scene::in_plane_offset_dir(dir, p_i, p_j, [0.0, 0.0, 1.0]);
        assert_eq!(out, dir);
    }
}
