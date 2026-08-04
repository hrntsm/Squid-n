//! モデル化図（解析上どの要素モデルで部材を扱っているかの可視化）の描画。
//!
//! 同じ形状のモデルでも、解析種別によって部材の要素定式化（モデル化）は変わる。
//! 本ビューは [`ModelingAnalysis`]（静解析＝弾性／増分解析＝弾塑性）を切り替えつつ、
//! 各部材が解析上どのモデルへ振り分けられるかを色と記号で示し、意図どおりの
//! モデル化になっているか（例: 耐震壁の側柱が面内両端ピンになっているか、剛床上の
//! 梁が材端集中塑性で、軸力変動する柱がファイバーになっているか）を視覚的に確認
//! できるようにする。
//!
//! 形状だけでは分からないモデル化の要素も併せて描く。
//! - **剛域**: 部材端の剛域長 `length_i/j` を、ハッチング入りのブロック（矩形の輪郭
//!   ＋斜めハッチ）で示す。線材とは図形の種類が違うため弾性材の細い線と紛れない。
//! - **材端集中塑性**: 材端（剛域フェイス）の塑性ヒンジ位置に塗り円（●）を置く。
//! - **ファイバー／MS**: 解析上は可撓部の材端（積分点 ξ=∓1、剛域があればその
//!   フェイス）にファイバー断面を置き、その積分重み＝塑性化域長 Lp の区間だけが
//!   塑性化する。中央 \[Lp, L'−Lp\] は弾性。よって端部 Lp 区間を太線で強調し、
//!   中央を弾性色の細線とし、ファイバー断面の位置に断面記号を描く。Lp は要素生成と
//!   同じ既定（[`squid_n_element::factory::plastic_zone_length`]）・同じクランプで
//!   解決するため、表示は解析のモデル化と一致する。
//! - **端部接合条件**: ピン（○）・半剛（□）を材端（剛域がある場合は剛域フェイス）に描く。
//! - **壁エレメント**: 耐震壁は壁エレメント置換モデル（壁柱＋両端ピンの上下剛梁）の
//!   「エ」状で描く。剛梁は実要素ではなく四隅の並進を壁柱端へ写す拘束のため、実部材と
//!   重ならないよう四辺とも内側へ寄せ、四隅節点へは破線の引出線でつなぐ。増分解析では
//!   壁柱の端部 Lp をファイバー色の太線＋断面記号で、面内せん断ばね（Qu 頭打ち）を
//!   壁柱に沿うジグザグで示す。フレーム内雑壁（周辺部材へ剛性算入）は半透明ポリゴンで
//!   区別する。
//! - **壁の付帯梁**: 耐震壁の上下大梁は、断面性能へ倍率を乗じた剛性で解析へ入る。
//!   剛梁ではないため線種は分類色のまま保ち、壁エレメント色の細い平行線を添えて
//!   付帯梁であることを示す。
//! - **仕口パネル**: モデル化されていれば、接合部が占める領域（幅＝柱せい、
//!   高さ＝梁せい）を梁の構面ごとに四角形で描く。パネルへ接合する部材は
//!   パネル分のオフセットが剛域長へ書き込まれている（`panel_gen`）ため、同じ区間に
//!   剛域のブロックも重なって描かれる。四角形は「この接合部にせん断変形角の
//!   自由度を持つパネル要素がある」ことを、ハッチは「その部材のこの区間が剛体
//!   アームである」ことを示す。
//!
//! 分類ロジックは要素生成（`squid_n_element::factory`）と同じ判定関数
//! （[`resolve_force_regime`] / [`wall_side_column_release`] / [`wall_is_seismic`]）を
//! 用いるため、実際に解析へ渡る要素種別と一致する。

use crate::app::App;
use crate::theme;
use squid_n_core::adjacency::NodeAdjacency;
use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, ElementKind, EndCondition, Model};
use squid_n_element::factory::{resolve_force_regime, ResolvedRegime};
use squid_n_element::misc_wall::wall_is_seismic;
use squid_n_element::side_column::{wall_side_column_release, SideColumnEdges};
use squid_n_element::wall_panel::wall_panel_geometry;

use super::{ModelingAnalysis, Projector};

/// 部材の解析モデル分類。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum ModelClass {
    /// 弾性材（断面の降伏を考慮しない。静解析の全部材・増分解析の弾性梁など）。
    Elastic,
    /// 材端集中塑性（材端回転ばね）。剛床上で軸力変動が小さい梁のモデル化。
    ConcentratedPlastic,
    /// ファイバー要素（分布塑性・軸-曲げ連成）。軸力変動する柱などのモデル化。
    Fiber,
    /// 耐震壁の側柱（面内両端ピン）。解析種別に依らない。
    SideColumnPin,
    /// 壁エレメント（耐震壁。壁エレメント置換モデル）。増分解析ではせん断降伏を考慮。
    Wall,
    /// フレーム内雑壁（耐震壁不成立。剛性を周辺の柱・梁へ算入する）。
    WallMisc,
    /// 仕口パネル（柱梁接合部パネル）。
    Panel,
    /// トラス／軸材（ブレースなど軸剛性のみ）。
    Truss,
    /// バネ・免震・ダンパー等その他の要素。
    Other,
}

impl ModelClass {
    /// 凡例・着色に用いる色。
    fn color(self) -> egui::Color32 {
        use egui::Color32;
        match self {
            // 弾性＝降伏を考えない中立色（グレー）
            ModelClass::Elastic => theme::GRAY_600,
            // 材端集中塑性＝緑
            ModelClass::ConcentratedPlastic => Color32::from_rgb(0x16, 0xA3, 0x4A),
            // ファイバー（分布塑性）＝オレンジ
            ModelClass::Fiber => Color32::from_rgb(0xEA, 0x58, 0x0C),
            // 側柱ピン＝強調紫
            ModelClass::SideColumnPin => theme::HILITE_PURPLE,
            // 壁エレメント＝青
            ModelClass::Wall => Color32::from_rgb(0x25, 0x63, 0xEB),
            // 雑壁＝淡い暖色（周辺部材へ剛性算入。構造壁エレメントと区別）
            ModelClass::WallMisc => theme::SECONDARY_AMBER,
            // 仕口パネル＝藍
            ModelClass::Panel => Color32::from_rgb(0x6D, 0x28, 0xD9),
            // トラス／軸材＝ティール
            ModelClass::Truss => Color32::from_rgb(0x0D, 0x94, 0x88),
            // その他＝淡いグレー
            ModelClass::Other => theme::GRAY_300,
        }
    }

    /// 凡例・ツールチップに表示する短いラベル。
    fn label(self) -> &'static str {
        match self {
            ModelClass::Elastic => "弾性材",
            ModelClass::ConcentratedPlastic => "材端集中塑性",
            ModelClass::Fiber => "ファイバー(分布塑性)",
            ModelClass::SideColumnPin => "側柱(面内両端ピン)",
            ModelClass::Wall => "壁エレメント",
            ModelClass::WallMisc => "雑壁(周辺部材へ剛性算入)",
            ModelClass::Panel => "仕口パネル",
            ModelClass::Truss => "トラス/軸材",
            ModelClass::Other => "その他(バネ/免震/ダンパー)",
        }
    }
}

/// 部材 `data` が解析種別 `analysis` の下でどのモデルへ振り分けられるかを分類する。
///
/// 判定は要素生成（`squid_n_element::factory::build_behavior` /
/// `build_nonlinear_behavior`）と同じ関数に委譲するため、実際に解析へ渡る要素種別と
/// 一致する。
pub(super) fn classify(
    data: &ElementData,
    model: &Model,
    analysis: ModelingAnalysis,
) -> ModelClass {
    classify_with(data, model, analysis, None)
}

/// [`classify`] の全部材ループ向け変種。側柱判定（1 部材ごとに全要素を走査する）を
/// 事前構築した [`SideColumnEdges`] の定数時間参照へ差し替えられる。
/// `None` は 1 部材だけ分類する呼び出し（ツールチップ・テスト）用で、
/// [`wall_side_column_release`] による従来の走査判定になる。
pub(super) fn classify_with(
    data: &ElementData,
    model: &Model,
    analysis: ModelingAnalysis,
    side_cols: Option<&SideColumnEdges>,
) -> ModelClass {
    match data.kind {
        // 梁・柱（Beam）とファイバー梁（Fiber）は解析種別で扱いが変わる。
        ElementKind::Beam | ElementKind::Fiber => {
            // 耐震壁の側柱は面内両端ピン（トポロジ由来の解放。解析種別に依らない）。
            let is_side_column = match side_cols {
                Some(idx) => idx.release_axis(data, model).is_some(),
                None => wall_side_column_release(data, model).is_some(),
            };
            if is_side_column {
                return ModelClass::SideColumnPin;
            }
            match analysis {
                // 静解析（線形）は断面の降伏を考えず弾性でモデル化する。
                ModelingAnalysis::Static => ModelClass::Elastic,
                // 増分解析は降伏を考慮。Fiber 種別は常にファイバー、Beam は
                // フォースレジーム判定で材端集中塑性／ファイバーへ振り分ける。
                ModelingAnalysis::Incremental => {
                    if data.kind == ElementKind::Fiber {
                        ModelClass::Fiber
                    } else {
                        match resolve_force_regime(data, model) {
                            ResolvedRegime::ConcentratedSpring => ModelClass::ConcentratedPlastic,
                            ResolvedRegime::Fiber => ModelClass::Fiber,
                        }
                    }
                }
            }
        }
        // マルチスプリング梁は端部塑性化域を軸ばね群で置換したモデル。
        // 増分解析では材端集中塑性、静解析（線形）では弾性として扱う。
        ElementKind::MultiSpring => match analysis {
            ModelingAnalysis::Static => ModelClass::Elastic,
            ModelingAnalysis::Incremental => ModelClass::ConcentratedPlastic,
        },
        // 壁は耐震壁成立なら壁エレメント、不成立なら雑壁（周辺部材へ剛性算入）。
        ElementKind::Wall => {
            if wall_is_seismic(data, model) {
                ModelClass::Wall
            } else {
                ModelClass::WallMisc
            }
        }
        ElementKind::PanelZone => ModelClass::Panel,
        ElementKind::Brace { .. } => ModelClass::Truss,
        // 面要素・バネ・免震・ダンパーなど。
        ElementKind::Shell
        | ElementKind::NodalSpring
        | ElementKind::Isolator
        | ElementKind::Damper => ModelClass::Other,
    }
}

// ===== 描画ヘルパ =====

/// スクリーン座標の線形補間。
fn lerp(a: egui::Pos2, b: egui::Pos2, t: f32) -> egui::Pos2 {
    egui::pos2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
}

/// 3D 2 点間の距離。
fn len3(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt()
}

/// 3D 点 `p` を、単位ベクトル `u` の方向へ `s` だけ動かす。
/// 壁エレメントの剛梁を壁の内側へ寄せるなど、ワールド座標での平行移動に使う。
fn shift3(p: [f64; 3], u: [f64; 3], s: f64) -> [f64; 3] {
    [p[0] + u[0] * s, p[1] + u[1] * s, p[2] + u[2] * s]
}

/// 3D ベクトル `a`→`b` の単位ベクトル（退化時は `None`）。
fn unit3(a: [f64; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    (l > 1e-9).then(|| [d[0] / l, d[1] / l, d[2] / l])
}

/// マーカー中心（節点/材端から材軸方向へ少し内側へ寄せた点）。
fn inward(at: egui::Pos2, toward: egui::Pos2, off: f32) -> egui::Pos2 {
    let d = toward - at;
    let len = d.length();
    if len > 1e-3 {
        egui::pos2(at.x + d.x / len * off, at.y + d.y / len * off)
    } else {
        at
    }
}

/// 端部ピンマーカー（白抜きの円）＝回転自由（ピン）の慣用記号。
fn draw_pin_marker(
    painter: &egui::Painter,
    at: egui::Pos2,
    toward: egui::Pos2,
    color: egui::Color32,
) {
    let c = inward(at, toward, 9.0);
    painter.circle_filled(c, 4.0, theme::WHITE);
    painter.circle_stroke(c, 4.0, egui::Stroke::new(1.5_f32, color));
}

/// 端部半剛（`SemiRigid`）マーカー（小さな正方形）。
fn draw_semi_rigid_marker(
    painter: &egui::Painter,
    at: egui::Pos2,
    toward: egui::Pos2,
    color: egui::Color32,
) {
    let c = inward(at, toward, 9.0);
    let rect = egui::Rect::from_center_size(c, egui::vec2(7.0, 7.0));
    painter.rect_filled(rect, 1.0, theme::WHITE);
    painter.rect_stroke(
        rect,
        1.0,
        egui::Stroke::new(1.5_f32, color),
        egui::StrokeKind::Middle,
    );
}

/// 材端集中塑性の塑性ヒンジマーカー（塗り円 ●）。
fn draw_hinge_marker(
    painter: &egui::Painter,
    at: egui::Pos2,
    toward: egui::Pos2,
    color: egui::Color32,
) {
    let c = inward(at, toward, 9.0);
    painter.circle_filled(c, 4.0, color);
}

/// 材軸 `a`→`b` に直交する単位ベクトル（スクリーン座標）。材端チック・ファイバー
/// 断面記号など、材軸に直交して描く記号の向きに使う。
fn perpendicular(a: egui::Pos2, b: egui::Pos2) -> egui::Vec2 {
    let d = b - a;
    let len = d.length();
    if len > 1e-3 {
        egui::vec2(-d.y / len, d.x / len)
    } else {
        egui::vec2(0.0, -1.0)
    }
}

/// 剛域ブロックの材軸直交半幅（px）。
const RIGID_HALF_W: f32 = 4.0;
/// 剛域ブロックのハッチング間隔（px）と、1 区間あたりの最大本数。
const RIGID_HATCH_STEP: f32 = 4.0;
const RIGID_HATCH_MAX: usize = 64;

/// 剛域（材端の剛域長区間）を**ハッチング入りのブロック**で描く。
///
/// 「線の色を変える」だけの区別では弾性材（細いグレー線）と紛らわしいため、
/// 材軸に沿った矩形の輪郭（2 本の平行線＋両端のキャップ）とその内部の斜めハッチで
/// 「面」として描き、線材とは図形の種類そのものが違う表現にする。両端のキャップが
/// そのまま剛域フェイスの位置を示す。
///
/// 白などで下地を塗り潰すと、接合部で他部材の記号を消してしまう（描画順に依存する
/// 抜けが出る）ため、塗り潰しは行わない。
fn draw_rigid_zone(painter: &egui::Painter, a: egui::Pos2, b: egui::Pos2) {
    let d = b - a;
    let len = d.length();
    if len < 1e-3 {
        return;
    }
    let u = d / len;
    let n = egui::vec2(-u.y, u.x) * RIGID_HALF_W;
    let stroke = egui::Stroke::new(1.5_f32, theme::GRAY_900);
    painter.line_segment([a - n, b - n], stroke);
    painter.line_segment([a + n, b + n], stroke);
    painter.line_segment([a - n, a + n], stroke);
    painter.line_segment([b - n, b + n], stroke);

    // 内部の斜めハッチ。区間が長いときは本数を頭打ちにして間隔を広げる。
    let step = (len / RIGID_HATCH_MAX as f32).max(RIGID_HATCH_STEP);
    let hatch = egui::Stroke::new(1.0_f32, theme::translucent(theme::GRAY_900, 170));
    let mut t = 0.0_f32;
    while t + step <= len {
        painter.line_segment([a + u * t - n, a + u * (t + step) + n], hatch);
        t += step;
    }
}

/// ファイバー断面（可撓部端の積分点 ξ=∓1）の記号。材軸に直交する短いバーと、
/// その上に並ぶ 3 点の塗り円で「断面をファイバーへ分割している位置」を表す。
///
/// 位置は解析上の積分点（可撓部の材端＝剛域があればそのフェイス）だが、記号が
/// 剛域ブロックや他部材の記号と重なって判読できないため、材軸方向へわずかに
/// 内側へ寄せて描く（他の材端記号と同じ描画上のオフセット規約）。
fn draw_fiber_section_marker(
    painter: &egui::Painter,
    at: egui::Pos2,
    toward: egui::Pos2,
    color: egui::Color32,
) {
    const HALF: f32 = 6.0;
    let c = inward(at, toward, 8.0);
    let n = perpendicular(at, toward);
    painter.line_segment(
        [c - n * HALF, c + n * HALF],
        egui::Stroke::new(2.0_f32, color),
    );
    for t in [-1.0_f32, 0.0, 1.0] {
        painter.circle_filled(c + n * (HALF * 0.55 * t), 1.8, color);
    }
}

/// 部材線の脇に添える細い平行線の、材軸からの距離（px）。剛域ブロックの半幅
/// （[`RIGID_HALF_W`]）より外へ出し、ハッチと重ならない位置に置く。
const PARALLEL_OFFSET: f32 = 7.0;

/// 壁の付帯梁（上下大梁）であることを示す、部材線に沿う細い平行線。
///
/// 付帯梁は剛梁ではなく、断面性能へ倍率を乗じた**通常の梁**として解析へ入る。
/// 分類色（弾性材・ファイバー等）の情報を失わないよう部材線そのものは変えず、
/// 壁エレメント色の細線を脇に添えて「この梁は壁の付帯梁である」ことだけを示す。
fn draw_wall_girder_mark(painter: &egui::Painter, a: egui::Pos2, b: egui::Pos2) {
    let n = perpendicular(a, b) * PARALLEL_OFFSET;
    painter.line_segment(
        [a + n, b + n],
        egui::Stroke::new(1.5_f32, ModelClass::Wall.color()),
    );
}

/// せん断ばねのジグザグ 1 本を、`a`→`b` に沿って `offset` px だけ脇へずらして描く。
///
/// 壁エレメントの面内せん断は、壁柱が全長で持つ 1 自由度のばね（終局せん断強度 Qu で
/// 頭打ち）である。特定の断面の応力で判定する量ではないため、材端へ記号を置かず
/// 材軸に沿わせて「この部材が全長で持つ性質」として描く。
fn draw_shear_spring(
    painter: &egui::Painter,
    a: egui::Pos2,
    b: egui::Pos2,
    offset: f32,
    color: egui::Color32,
) {
    const AMPLITUDE: f32 = 3.0;
    const WAVELENGTH: f32 = 9.0;
    let d = b - a;
    let len = d.length();
    if len < WAVELENGTH * 2.0 {
        return;
    }
    let u = d / len;
    let n = egui::vec2(-u.y, u.x);
    let base = |t: f32| a + u * t + n * offset;
    let steps = ((len / WAVELENGTH).floor() as usize).max(2);
    let step = len / steps as f32;
    let stroke = egui::Stroke::new(1.5_f32, color);
    let mut prev = base(0.0);
    for i in 1..=steps {
        let t = step * i as f32;
        // 折れ点は山・谷を交互に取り、最後は振幅 0 へ戻して線を閉じる。
        let amp = if i == steps {
            0.0
        } else if i % 2 == 1 {
            AMPLITUDE
        } else {
            -AMPLITUDE
        };
        let next = base(t) + n * amp;
        painter.line_segment([prev, next], stroke);
        prev = next;
    }
}

/// 可とう区間の端点フラクション（材軸パラメータ s∈[0,1] の両端）と可とう長 [mm]。
/// 剛域長の解決は要素側（[`squid_n_element::rigid_arm::resolve_lengths`]）と共通で、
/// 可撓長が残らない入力は剛域なしとして扱う。
fn flexible_span(elem: &ElementData, l: f64) -> (f32, f32, f64) {
    if l <= 1e-9 {
        return (0.0, 1.0, l);
    }
    let (li, lj) = squid_n_element::rigid_arm::resolve_lengths(
        elem.rigid_zone.rigid_length_i(),
        elem.rigid_zone.rigid_length_j(),
        l,
    );
    ((li / l) as f32, 1.0 - (lj / l) as f32, l - li - lj)
}

/// モデル化図を描く。`pts` は節点スクリーン座標、`coords3` は節点 3D 座標
/// （いずれも `app.model.nodes` と同じ順序）、`proj` は投影文脈。基本形状の上に、
/// 解析モデル分類ごとの色で部材を塗り、剛域・塑性ヒンジ・ファイバー域・端部接合条件
/// などモデル化の要素を記号で重ねる。
pub(super) fn draw_modeling(
    painter: &egui::Painter,
    app: &App,
    pts: &[egui::Pos2],
    coords3: &[[f64; 3]],
    proj: &Projector,
    frame_filter: super::FrameFilter,
) {
    let model = &app.model;
    let analysis = app.modeling_analysis;

    // 凡例に載せる情報を収集する。
    let mut present: Vec<ModelClass> = Vec::new();
    let mut sym = Symbols::default();
    // 仕口パネルの見付き寸法算定で使う隣接マップ。パネルが 1 つも無いモデルでは
    // 構築しない（遅延初期化）。
    let mut beam_adjacency: Option<NodeAdjacency> = None;
    // 壁の付帯梁の絞り込みに使う耐震壁の節点集合（描画 1 回につき一度だけ作る）。
    let wall_nodes = seismic_wall_nodes(model);
    // 側柱判定の事前インデックス（描画 1 回につき一度だけ作る。1 部材ごとの
    // 全要素走査を避ける）。
    let side_cols = SideColumnEdges::build(model);

    for elem in &model.elements {
        if !frame_filter.shows(elem.id) {
            continue;
        }
        let class = classify_with(elem, model, analysis, Some(&side_cols));
        if !present.contains(&class) {
            present.push(class);
        }

        match class {
            ModelClass::Wall => draw_wall_element(
                painter,
                model,
                pts,
                coords3,
                proj,
                elem,
                analysis,
                class.color(),
                &mut sym,
            ),
            ModelClass::WallMisc => draw_wall_polygon(painter, pts, elem, class.color(), true),
            ModelClass::Panel => draw_panel_zone(
                painter,
                model,
                beam_adjacency.get_or_insert_with(|| NodeAdjacency::build(model)),
                pts,
                coords3,
                proj,
                elem,
                class.color(),
            ),
            _ => draw_line_member(
                painter,
                model,
                pts,
                coords3,
                elem,
                class,
                &wall_nodes,
                &mut sym,
            ),
        }
    }

    draw_legend(painter, analysis, &present, &sym);
}

/// 記号凡例に載せるフラグ（描画中に実際に現れた記号のみ凡例へ出す）。
#[derive(Default)]
struct Symbols {
    pin: bool,
    semi: bool,
    hinge: bool,
    rigid: bool,
    /// ファイバー断面（可撓部端の積分点）の記号を描いた。
    fiber_section: bool,
    /// 壁の付帯梁（上下大梁）の平行線を描いた。
    wall_girder: bool,
    /// 壁エレメントの面内せん断ばね（ジグザグ）を描いた。
    wall_shear: bool,
    /// 壁エレメントの剛梁と四隅節点をつなぐ引出線を描いた。
    wall_leader: bool,
}

/// 耐震壁の節点の集合。
///
/// 付帯梁になりうるのは耐震壁の節点を両端に持つ部材だけなので、
/// [`is_wall_girder`] の安価な絞り込みに使う。`stiffness_breakdown` は内部でモデル
/// 全体を走査するため、部材ごとに素で呼ぶと部材数の二乗に比例する。1 回の描画につき
/// 一度だけ集合を作り、通過した部材にだけ本判定を掛ける。
fn seismic_wall_nodes(model: &Model) -> std::collections::HashSet<NodeId> {
    model
        .elements
        .iter()
        .filter(|e| e.kind == ElementKind::Wall && wall_is_seismic(e, model))
        .flat_map(|e| e.nodes.iter().copied())
        .collect()
}

/// 部材 `elem` が耐震壁の付帯梁（上下大梁）か。
///
/// 判定・倍率ともに要素生成（[`squid_n_element::beam::stiffness_breakdown`]）へ委ねる
/// ため、図に現れる付帯梁は実際に倍率が乗って解析へ入る梁と一致する。倍率の値は
/// 剛性計算条件で変わりうるため図には出さず、付帯梁であることのみを示す。
///
/// 倍率が乗るのは梁要素として組まれる線材だけのため、線材以外は対象外とする
/// （壁要素自身も先頭 2 節点が下辺と一致すると `stiffness_breakdown` は倍率を
/// 返すが、壁は `BeamElement` として組まれないため倍率は解析に効かない）。
///
/// `wall_nodes`（[`seismic_wall_nodes`]）は本判定を掛ける部材を絞り込むためのもので、
/// 付帯梁の条件（耐震壁の節点を 2 つとも両端に持つ水平材）より広い集合である。
/// 最終判定は `stiffness_breakdown` に委ねるため、絞り込んでも解析との一致は崩れない。
fn is_wall_girder(
    elem: &ElementData,
    model: &Model,
    wall_nodes: &std::collections::HashSet<NodeId>,
) -> bool {
    matches!(
        elem.kind,
        ElementKind::Beam | ElementKind::Fiber | ElementKind::MultiSpring
    ) && elem.nodes.len() >= 2
        && wall_nodes.contains(&elem.nodes[0])
        && wall_nodes.contains(&elem.nodes[1])
        && squid_n_element::beam::stiffness_breakdown(model, elem).wall_girder > 1.0
}

/// 部材 `elem` が「端部塑性化域モデル」（材端 ξ=∓1 にファイバー断面を置き、
/// 中央を弾性とするモデル）で解析されるか。
///
/// 該当するのは増分解析のファイバー要素と MS 要素で、いずれも実体は
/// `FiberBeam::build_plastic_zone`（MS はファイバ分割が粗いだけ）。`class` は
/// 解析種別を織り込んだ分類結果のため、静解析（全部材弾性）では偽になる。
fn is_end_plastic_zone_model(elem: &ElementData, class: ModelClass) -> bool {
    match class {
        ModelClass::Fiber => true,
        ModelClass::ConcentratedPlastic => elem.kind == ElementKind::MultiSpring,
        _ => false,
    }
}

/// 端部塑性化域モデルの有効な塑性化域長 Lp [mm]（可撓長 `l_flex` 基準）。
/// 要素生成と同じ既定・同じクランプを用いるため、表示は解析のモデル化と一致する。
fn plastic_zone_len(elem: &ElementData, model: &Model, l_flex: f64) -> f64 {
    if l_flex <= 1e-9 {
        return 0.0;
    }
    squid_n_element::fiber::clamp_plastic_zone(
        squid_n_element::factory::plastic_zone_length(elem, model),
        l_flex,
    )
}

/// 線材（梁・柱・ファイバー・側柱）のモデル化を描く。
/// `wall_nodes` は壁の付帯梁の絞り込み用（[`seismic_wall_nodes`]）。
#[allow(clippy::too_many_arguments)]
fn draw_line_member(
    painter: &egui::Painter,
    model: &Model,
    pts: &[egui::Pos2],
    coords3: &[[f64; 3]],
    elem: &ElementData,
    class: ModelClass,
    wall_nodes: &std::collections::HashSet<NodeId>,
    sym: &mut Symbols,
) {
    if elem.nodes.len() < 2 {
        return;
    }
    let n0 = elem.nodes[0].index();
    let n1 = elem.nodes[1].index();
    if n0 >= pts.len() || n1 >= pts.len() || n0 >= coords3.len() || n1 >= coords3.len() {
        return;
    }
    let (p0, p1) = (pts[n0], pts[n1]);
    let l = len3(coords3[n0], coords3[n1]);
    let color = class.color();

    // 可とう区間（剛域フェイス間）。すべての線材モデルが剛域を可撓長から控除し、
    // 可撓端自由度を剛体アームで節点自由度へ写す（`squid_n_element::rigid_arm`）。
    let end_plastic = is_end_plastic_zone_model(elem, class);
    let (s_i, s_j, l_flex) = flexible_span(elem, l);

    // 材端の記号を置く位置（剛域があればそのフェイス）。
    let fa = lerp(p0, p1, s_i);
    let fb = lerp(p0, p1, s_j);

    if end_plastic {
        // 端部 Lp 区間 = ファイバー断面の積分重み（塑性化域）。中央は弾性。
        // Lp は可撓長基準のため、可とう区間 [fa, fb] のパラメータで置く。
        let lp = if l_flex > 1e-9 {
            (plastic_zone_len(elem, model, l_flex) / l_flex) as f32
        } else {
            0.0
        };
        let a = lerp(fa, fb, lp);
        let b = lerp(fa, fb, 1.0 - lp);
        painter.line_segment(
            [a, b],
            egui::Stroke::new(3.0_f32, ModelClass::Elastic.color()),
        );
        let zone_stroke = egui::Stroke::new(5.0_f32, color);
        painter.line_segment([fa, a], zone_stroke);
        painter.line_segment([b, fb], zone_stroke);
    } else {
        // 可とう区間の基準線。
        painter.line_segment([fa, fb], egui::Stroke::new(3.0_f32, color));
    }

    // 壁の付帯梁（上下大梁）。部材全長に沿わせ、剛域を含む梁全体の性質として示す。
    if is_wall_girder(elem, model, wall_nodes) {
        draw_wall_girder_mark(painter, p0, p1);
        sym.wall_girder = true;
    }

    // 剛域バー（材端）。
    if s_i > 0.0 {
        draw_rigid_zone(painter, p0, fa);
        sym.rigid = true;
    }
    if s_j < 1.0 {
        draw_rigid_zone(painter, fb, p1);
        sym.rigid = true;
    }

    // ファイバー断面（積分点 ξ=∓1＝可撓部の材端）の位置。剛域バーの上に重ねる。
    if end_plastic {
        draw_fiber_section_marker(painter, fa, fb, color);
        draw_fiber_section_marker(painter, fb, fa, color);
        sym.fiber_section = true;
    }

    // 端部の接合条件・塑性ヒンジ。側柱は面内両端ピンのため両端に○。
    if class == ModelClass::SideColumnPin {
        draw_pin_marker(painter, fa, fb, color);
        draw_pin_marker(painter, fb, fa, color);
        sym.pin = true;
        return;
    }
    for (end_idx, near, far) in [(0usize, fa, fb), (1usize, fb, fa)] {
        match elem.end_cond[end_idx] {
            EndCondition::Pinned => {
                draw_pin_marker(painter, near, far, color);
                sym.pin = true;
            }
            EndCondition::SemiRigid { .. } => {
                draw_semi_rigid_marker(painter, near, far, color);
                sym.semi = true;
            }
            // 剛接端: 材端集中塑性（材端回転ばね）なら塑性ヒンジ位置に ● を置く。
            // 端部塑性化域モデル（MS）は回転ばねではなくファイバー断面のため、
            // 断面記号のみとし ● は描かない。
            EndCondition::Fixed => {
                if class == ModelClass::ConcentratedPlastic && !end_plastic {
                    draw_hinge_marker(painter, near, far, color);
                    sym.hinge = true;
                }
            }
        }
    }
}

/// 壁エレメント（耐震壁）を壁エレメント置換モデルの「エ」状で描く。
///
/// 上下の剛梁は実要素ではなく、四隅節点の並進を壁柱端の並進・回転へ写す拘束である。
/// 壁の上下辺には実部材（付帯梁）が同じ位置に存在するため、剛梁をそのまま辺上に描くと
/// 実梁が剛梁でありその端部がピンであるかのように読めてしまう。そこで剛梁は**四辺とも
/// 内側へ寄せて**描き、剛梁端と四隅節点は破線の引出線でつなぐ。○（ピン）は剛梁端に
/// 置き、四隅節点の回転自由度に壁エレメントが剛性を与えないことを示す。
///
/// 増分解析では、壁柱の軸・曲げはファイバー（端部 Lp が塑性化域、中央は弾性）、
/// 面内せん断は Qu で頭打ちにするばねでモデル化される。前者は端部 Lp の太線と
/// 断面記号で、後者は壁柱に沿うジグザグで示す。
///
/// 幾何を取れない場合はポリゴンへフォールバックする。
#[allow(clippy::too_many_arguments)]
fn draw_wall_element(
    painter: &egui::Painter,
    model: &Model,
    pts: &[egui::Pos2],
    coords3: &[[f64; 3]],
    proj: &Projector,
    elem: &ElementData,
    analysis: ModelingAnalysis,
    color: egui::Color32,
    sym: &mut Symbols,
) {
    let Some(g) = wall_panel_geometry(elem, model) else {
        draw_wall_polygon(painter, pts, elem, color, false);
        return;
    };
    let (b0, b1) = (g.bottom[0].index(), g.bottom[1].index());
    let (t0, t1) = (g.top[0].index(), g.top[1].index());
    if [b0, b1, t0, t1].iter().any(|&i| i >= coords3.len()) {
        draw_wall_polygon(painter, pts, elem, color, false);
        return;
    }
    // 壁面内の直交 2 方向。ex は下辺 a→b、ez は下辺中点→上辺中点。
    let ex = g.ex_bottom;
    let Some(ez) = unit3(g.bottom_center, g.top_center) else {
        draw_wall_polygon(painter, pts, elem, color, false);
        return;
    };

    // 内側への寄せ量。ズームに依らず実梁のすぐ内側に見えるよう画面基準（px）で決め、
    // 極端なズームアウトで壁高さを食い潰さないよう h の 15% で頭打ちにする。
    let inset = (10.0 / (proj.scale() as f64).max(1e-9)).min(0.15 * g.h);

    // 剛梁の四隅（四辺とも内側へ寄せた位置）。a 側は +ex、b 側は −ex、
    // 下辺は +ez、上辺は −ez へ動かす。
    let corner = |p: [f64; 3], sx: f64, sz: f64| {
        proj.project(shift3(shift3(p, ex, sx * inset), ez, sz * inset))
    };
    let (sb0, sb1) = (
        corner(coords3[b0], 1.0, 1.0),
        corner(coords3[b1], -1.0, 1.0),
    );
    let (st0, st1) = (
        corner(coords3[t0], 1.0, -1.0),
        corner(coords3[t1], -1.0, -1.0),
    );

    // 剛梁端 → 四隅節点の引出線（破線＝実要素ではない）。
    let leader = egui::Stroke::new(1.0_f32, theme::translucent(theme::GRAY_900, 150));
    for (from, to) in [(sb0, b0), (sb1, b1), (st0, t0), (st1, t1)] {
        painter.extend(egui::Shape::dashed_line(
            &[from, proj.project(coords3[to])],
            leader,
            4.0,
            3.0,
        ));
    }
    sym.wall_leader = true;

    // 上下の剛梁（剛域と同じ表記のハッチ入りブロック。剛体アームであることを示す）。
    draw_rigid_zone(painter, sb0, sb1);
    draw_rigid_zone(painter, st0, st1);
    sym.rigid = true;

    // 壁柱（上下剛梁の中点を結ぶ仮想中央柱）。増分解析でファイバー化される壁柱は
    // 端部 Lp だけが塑性化するため、その比率を可撓長（＝壁高さ h）基準で求めて渡す。
    let bc = proj.project(shift3(g.bottom_center, ez, inset));
    let tc = proj.project(shift3(g.top_center, ez, -inset));
    let lp_ratio = if analysis == ModelingAnalysis::Incremental && g.h > 1e-9 {
        squid_n_element::wall_panel::wall_column_fiber_lp(elem, model).map(|lp| (lp / g.h) as f32)
    } else {
        None
    };
    draw_wall_column(painter, bc, tc, lp_ratio, color, sym);

    // 面内せん断ばね（Qu 頭打ち）。壁柱が全長で持つ 1 自由度のため材軸に沿わせる。
    if analysis == ModelingAnalysis::Incremental
        && squid_n_element::wall_panel::WallPanelElement::shear_capacity_of(elem, model) > 0.0
    {
        draw_shear_spring(painter, bc, tc, PARALLEL_OFFSET, color);
        sym.wall_shear = true;
    }

    // 剛梁端のピン。四隅節点の回転自由度に壁エレメントが剛性を与えないことを示す。
    draw_pin_marker(painter, sb0, sb1, color);
    draw_pin_marker(painter, sb1, sb0, color);
    draw_pin_marker(painter, st0, st1, color);
    draw_pin_marker(painter, st1, st0, color);
    sym.pin = true;
}

/// 壁柱（壁エレメントの仮想中央柱）を `bc`→`tc` に描く。
///
/// 増分解析でファイバー化される壁柱は、端部 Lp 区間だけがファイバー断面の積分重みで
/// 塑性化し、中央は弾性である。中央は壁エレメント色の細線として「エ」の形を保ちつつ、
/// 塑性化を担う端部 Lp と断面記号はファイバー色で描き、塑性化機構がファイバーである
/// ことを示す。`lp_ratio` が `None`（静解析・ファイバー化されない壁）のときは全長を
/// 壁エレメント色の線で描く。
fn draw_wall_column(
    painter: &egui::Painter,
    bc: egui::Pos2,
    tc: egui::Pos2,
    lp_ratio: Option<f32>,
    color: egui::Color32,
    sym: &mut Symbols,
) {
    let Some(lp) = lp_ratio else {
        painter.line_segment([bc, tc], egui::Stroke::new(3.0_f32, color));
        return;
    };

    let fiber = ModelClass::Fiber.color();
    let a = lerp(bc, tc, lp);
    let b = lerp(bc, tc, 1.0 - lp);
    // 中央弾性区間は壁エレメント色の細線、端部 Lp はファイバー色の太線。
    painter.line_segment([a, b], egui::Stroke::new(3.0_f32, color));
    let zone = egui::Stroke::new(5.0_f32, fiber);
    painter.line_segment([bc, a], zone);
    painter.line_segment([b, tc], zone);
    // ファイバー断面（積分点 ξ=∓1＝壁柱の材端）。
    draw_fiber_section_marker(painter, bc, tc, fiber);
    draw_fiber_section_marker(painter, tc, bc, fiber);
    sym.fiber_section = true;
}

/// 壁を半透明ポリゴンで描く（雑壁、または壁エレメント幾何を取れない壁のフォールバック）。
/// `dashed` が真のとき輪郭を破線にして雑壁であることを示す。
fn draw_wall_polygon(
    painter: &egui::Painter,
    pts: &[egui::Pos2],
    elem: &ElementData,
    color: egui::Color32,
    dashed: bool,
) {
    if elem.nodes.len() < 3 {
        return;
    }
    let poly: Vec<egui::Pos2> = elem
        .nodes
        .iter()
        .filter_map(|n| {
            let idx = n.index();
            (idx < pts.len()).then(|| pts[idx])
        })
        .collect();
    if poly.len() != elem.nodes.len() {
        return;
    }
    let stroke = egui::Stroke::new(1.5_f32, color);
    if dashed {
        // 塗りのみ描き、輪郭は破線で重ねる（雑壁＝構造壁エレメントでないことを示す）。
        painter.add(egui::Shape::convex_polygon(
            poly.clone(),
            theme::translucent(color, 35),
            egui::Stroke::NONE,
        ));
        let mut ring = poly;
        ring.push(ring[0]);
        painter.extend(egui::Shape::dashed_line(&ring, stroke, 6.0, 4.0));
    } else {
        painter.add(egui::Shape::convex_polygon(
            poly,
            theme::translucent(color, 45),
            stroke,
        ));
    }
}

/// 接合部に取り付く部材から、パネルの見付き半寸法を求める。
///
/// 解析側が部材の剛域長へ書き込むオフセットと同じ値
/// （[`squid_n_core::panel_zone::panel_half_extent`]）を使うため、図に現れる
/// 四角形は解析上のパネル寸法と一致する。
fn panel_half_extent_at(
    model: &Model,
    adjacency: &NodeAdjacency,
    node: NodeId,
) -> squid_n_core::panel_zone::PanelHalfExtent {
    squid_n_core::panel_zone::panel_half_extent(model, node, adjacency.elements_at(model, node))
}

/// 水平材（はり）の材軸方向を、平行なものをまとめて集める。
///
/// 左右の梁は同じ構面を作るため、四角形は構面ごとに 1 枚だけ描く。
fn panel_beam_axes(model: &Model, adjacency: &NodeAdjacency, node: NodeId) -> Vec<[f64; 3]> {
    use squid_n_core::panel_zone::{member_orientation, member_unit_axis, MemberOrientation};
    let mut axes: Vec<[f64; 3]> = Vec::new();
    for e in adjacency.elements_at(model, node) {
        if member_orientation(model, e) != Some(MemberOrientation::Beam) {
            continue;
        }
        let Some(axis) = member_unit_axis(model, e) else {
            continue;
        };
        let parallel = axes
            .iter()
            .any(|a| (a[0] * axis[0] + a[1] * axis[1] + a[2] * axis[2]).abs() > 0.99);
        if !parallel {
            axes.push(axis);
        }
    }
    axes
}

/// 仕口パネル（柱梁接合部パネル）を、接合部に設ける四角形として描く。
///
/// 梁が取り付く構面ごとに、幅＝柱せい・高さ＝梁せいの四角形を接合部中心に描く。
/// これは解析でパネルが占める領域（部材がその面で接合する範囲）そのものである。
///
/// 同じ区間には部材側の剛域ブロック（ハッチ）も重なって描かれる。両者は別の
/// 情報で、ハッチは「その部材のこの区間が剛体アームである」、四角形は
/// 「この接合部に γX・γY の 2 自由度を持つパネル要素がある」を示す。
///
/// 寸法が求まらない接合部（直交材の断面が未割当など）は四角形を描けないため、
/// 中心のひし形マーカーで位置だけを示す。
#[allow(clippy::too_many_arguments)]
fn draw_panel_zone(
    painter: &egui::Painter,
    model: &Model,
    adjacency: &NodeAdjacency,
    pts: &[egui::Pos2],
    coords3: &[[f64; 3]],
    proj: &Projector,
    elem: &ElementData,
    color: egui::Color32,
) {
    let Some(node) = elem.nodes.first().copied() else {
        return;
    };
    let center = node.index();
    if center >= pts.len() || center >= coords3.len() {
        return;
    }
    let c = pts[center];
    let c3 = coords3[center];

    let extent = panel_half_extent_at(model, adjacency, node);
    let mut drawn = false;
    if extent.beam_half > 0.0 && extent.column_half > 0.0 {
        for axis in panel_beam_axes(model, adjacency, node) {
            // 構面内の 4 隅（中心 ± 幅方向 ± 鉛直方向）。
            let corner = |sw: f64, sh: f64| -> [f64; 3] {
                [
                    c3[0] + sw * extent.column_half * axis[0],
                    c3[1] + sw * extent.column_half * axis[1],
                    c3[2] + sh * extent.beam_half,
                ]
            };
            let quad: Vec<egui::Pos2> = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
                .iter()
                .map(|&(sw, sh)| proj.project(corner(sw, sh)))
                .collect();
            // 剛域ハッチと重なるため、塗りは輪郭を邪魔しない程度に薄くする。
            painter.add(egui::Shape::convex_polygon(
                quad,
                theme::translucent(color, 40),
                egui::Stroke::new(1.5_f32, color),
            ));
            drawn = true;
        }
    }

    if !drawn {
        // 寸法が求まらない場合は位置だけを示す（接続節点へ細線＋ひし形）。
        for n in elem.nodes.iter().skip(1) {
            let i = n.index();
            if i < pts.len() {
                painter.line_segment(
                    [c, lerp(c, pts[i], 0.35)],
                    egui::Stroke::new(1.5_f32, theme::translucent(color, 160)),
                );
            }
        }
        const R: f32 = 7.0;
        let diamond = [
            egui::pos2(c.x, c.y - R),
            egui::pos2(c.x + R, c.y),
            egui::pos2(c.x, c.y + R),
            egui::pos2(c.x - R, c.y),
        ];
        painter.add(egui::Shape::convex_polygon(
            diamond.to_vec(),
            theme::translucent(color, 90),
            egui::Stroke::new(1.5_f32, color),
        ));
    }
}

/// モデル化図の凡例をビュー左上に描く（支持条件凡例は左下のため衝突しない）。
fn draw_legend(
    painter: &egui::Painter,
    analysis: ModelingAnalysis,
    present: &[ModelClass],
    sym: &Symbols,
) {
    let rect = painter.clip_rect();
    let x0 = rect.min.x + 10.0;
    let mut y = rect.min.y + 12.0;
    const LINE_H: f32 = 16.0;
    const FONT: f32 = 11.0;

    let title = match analysis {
        ModelingAnalysis::Static => "モデル化（静解析＝弾性）",
        ModelingAnalysis::Incremental => "モデル化（増分解析＝弾塑性）",
    };
    painter.text(
        egui::pos2(x0, y),
        egui::Align2::LEFT_TOP,
        title,
        egui::FontId::proportional(13.0),
        theme::GRAY_700,
    );
    y += LINE_H + 2.0;

    for class in present {
        painter.line_segment(
            [
                egui::pos2(x0, y + FONT * 0.5),
                egui::pos2(x0 + 20.0, y + FONT * 0.5),
            ],
            egui::Stroke::new(3.0_f32, class.color()),
        );
        painter.text(
            egui::pos2(x0 + 28.0, y),
            egui::Align2::LEFT_TOP,
            class.label(),
            egui::FontId::proportional(FONT),
            theme::GRAY_600,
        );
        y += LINE_H;
    }

    // 記号の凡例（実際に現れた記号のみ）。
    let text = |painter: &egui::Painter, y: f32, s: &str| {
        painter.text(
            egui::pos2(x0 + 28.0, y),
            egui::Align2::LEFT_TOP,
            s,
            egui::FontId::proportional(FONT),
            theme::GRAY_600,
        );
    };
    if sym.rigid {
        draw_rigid_zone(
            painter,
            egui::pos2(x0 + 2.0, y + FONT * 0.5),
            egui::pos2(x0 + 18.0, y + FONT * 0.5),
        );
        text(painter, y, "剛域・壁エレメントの剛梁");
        y += LINE_H;
    }
    if sym.wall_leader {
        painter.extend(egui::Shape::dashed_line(
            &[
                egui::pos2(x0, y + FONT * 0.5),
                egui::pos2(x0 + 20.0, y + FONT * 0.5),
            ],
            egui::Stroke::new(1.0_f32, theme::translucent(theme::GRAY_900, 150)),
            4.0,
            3.0,
        ));
        text(painter, y, "破線 = 剛梁と四隅節点のつながり");
        y += LINE_H;
    }
    if sym.wall_girder {
        let yc = y + FONT * 0.5;
        painter.line_segment(
            [egui::pos2(x0, yc - 2.0), egui::pos2(x0 + 20.0, yc - 2.0)],
            egui::Stroke::new(3.0_f32, theme::GRAY_600),
        );
        painter.line_segment(
            [egui::pos2(x0, yc + 3.0), egui::pos2(x0 + 20.0, yc + 3.0)],
            egui::Stroke::new(1.5_f32, ModelClass::Wall.color()),
        );
        text(painter, y, "壁の付帯梁（上下大梁。剛性に倍率）");
        y += LINE_H;
    }
    if sym.wall_shear {
        draw_shear_spring(
            painter,
            egui::pos2(x0, y + FONT * 0.5),
            egui::pos2(x0 + 20.0, y + FONT * 0.5),
            0.0,
            ModelClass::Wall.color(),
        );
        text(painter, y, "壁の面内せん断ばね（Qu で頭打ち）");
        y += LINE_H;
    }
    if sym.pin {
        let c = egui::pos2(x0 + 10.0, y + FONT * 0.5);
        painter.circle_filled(c, 4.0, theme::WHITE);
        painter.circle_stroke(c, 4.0, egui::Stroke::new(1.5_f32, theme::GRAY_600));
        text(painter, y, "○ 端部ピン（回転自由）");
        y += LINE_H;
    }
    if sym.semi {
        let r = egui::Rect::from_center_size(
            egui::pos2(x0 + 10.0, y + FONT * 0.5),
            egui::vec2(7.0, 7.0),
        );
        painter.rect_filled(r, 1.0, theme::WHITE);
        painter.rect_stroke(
            r,
            1.0,
            egui::Stroke::new(1.5_f32, theme::GRAY_600),
            egui::StrokeKind::Middle,
        );
        text(painter, y, "□ 端部半剛（回転ばね）");
        y += LINE_H;
    }
    if sym.hinge {
        painter.circle_filled(
            egui::pos2(x0 + 10.0, y + FONT * 0.5),
            4.0,
            ModelClass::ConcentratedPlastic.color(),
        );
        text(painter, y, "● 材端塑性ヒンジ");
        y += LINE_H;
    }
    if sym.fiber_section {
        // 断面記号（材軸を水平とみなした向き）と、太線＝塑性化域 Lp の説明。
        draw_fiber_section_marker(
            painter,
            egui::pos2(x0 + 2.0, y + FONT * 0.5),
            egui::pos2(x0 + 20.0, y + FONT * 0.5),
            ModelClass::Fiber.color(),
        );
        text(painter, y, "ファイバー断面（積分点 ξ=∓1、可撓部の材端）");
        y += LINE_H;
        painter.line_segment(
            [
                egui::pos2(x0, y + FONT * 0.5),
                egui::pos2(x0 + 20.0, y + FONT * 0.5),
            ],
            egui::Stroke::new(5.0_f32, ModelClass::Fiber.color()),
        );
        text(painter, y, "太線 = 塑性化域 Lp／細線 = 中央弾性");
    }
}

/// 壁エレメントのモデル化詳細（壁の寸法と、増分解析での降伏機構）。
///
/// 壁エレメントの降伏はファイバー（軸・曲げ）と面内せん断ばね（Qu 頭打ち）の
/// 2 機構からなる。図では前者を壁柱端部の Lp、後者を壁柱に沿うジグザグで示すため、
/// ここではその根拠となる数値を並べる。
fn show_wall_modeling_detail(ui: &mut egui::Ui, app: &App, elem: &ElementData) {
    let Some(g) = squid_n_element::wall_panel::wall_panel_geometry(elem, &app.model) else {
        ui.label("幾何を取得できないため半透明ポリゴンで表示");
        return;
    };
    ui.label(format!("壁長 lw={:.0} mm ／ 壁高 h={:.0} mm", g.lw, g.h));
    if app.modeling_analysis != ModelingAnalysis::Incremental {
        return;
    }
    match squid_n_element::wall_panel::wall_column_fiber_lp(elem, &app.model) {
        Some(lp) => {
            ui.label("ファイバー断面: 壁柱の材端 2 箇所（積分点 ξ=∓1）");
            ui.label(format!("塑性化域 Lp={lp:.0} mm（0.5·lw）／中央弾性"));
        }
        None => {
            ui.label("軸・曲げ: 弾性（ファイバー断面を組めない）");
        }
    }
    let qu = squid_n_element::wall_panel::WallPanelElement::shear_capacity_of(elem, &app.model);
    if qu > 0.0 {
        ui.label(format!("面内せん断: Qu={:.0} kN で頭打ち", qu / 1.0e3));
    } else {
        ui.label("面内せん断: 弾性（Qu を算定できない）");
    }
}

/// モデル化図のホバー詳細ツールチップ。部材の解析モデル分類・端条件・剛域・
/// 塑性化域などのモデル化情報を表示する。
pub(super) fn show_modeling_tooltip(ui: &egui::Ui, app: &App, elem_id: squid_n_core::ids::ElemId) {
    let Some(elem) = app.model.elements.iter().find(|e| e.id == elem_id) else {
        return;
    };
    let class = classify(elem, &app.model, app.modeling_analysis);
    let end_label = |c: EndCondition| -> &'static str {
        match c {
            EndCondition::Fixed => "剛",
            EndCondition::Pinned => "ピン",
            EndCondition::SemiRigid { .. } => "半剛",
        }
    };

    #[allow(deprecated)]
    egui::show_tooltip_at_pointer(
        ui.ctx(),
        ui.layer_id(),
        egui::Id::new("modeling_tooltip"),
        |ui| {
            ui.label(format!("部材 #{}", elem_id.0));
            ui.colored_label(class.color(), class.label());
            if class == ModelClass::Wall {
                show_wall_modeling_detail(ui, app, elem);
                return;
            }
            // 耐震壁の付帯梁（上下大梁）。断面性能へ倍率が乗った剛性で解析へ入る。
            if is_wall_girder(elem, &app.model, &seismic_wall_nodes(&app.model)) {
                ui.label("壁の付帯梁（上下大梁。剛性に倍率）");
            }
            let is_frame_line = matches!(
                elem.kind,
                ElementKind::Beam | ElementKind::Fiber | ElementKind::MultiSpring
            ) && wall_side_column_release(elem, &app.model).is_none();
            let end_plastic = is_end_plastic_zone_model(elem, class);
            if is_frame_line {
                ui.label(format!(
                    "端条件: i={} / j={}",
                    end_label(elem.end_cond[0]),
                    end_label(elem.end_cond[1])
                ));
                // 梁のねじり剛性を期待しない既定モデル化（i 端ねじれ解放）が
                // この部材に適用されているかを明示する（適用されない例外がある）。
                if squid_n_element::beam::i_end_torsion_release(elem, &app.model) {
                    ui.label("ねじれ: i 端ピン（部材全長で Mx=0）");
                }
                let rz = &elem.rigid_zone;
                if rz.length_i > 0.0 || rz.length_j > 0.0 {
                    ui.label(format!(
                        "剛域長: i={:.0} / j={:.0} mm",
                        rz.length_i, rz.length_j
                    ));
                }
            }
            // 端部塑性化域モデル（ファイバー／MS）は、可撓部の材端（積分点 ξ=∓1、
            // 剛域があればそのフェイス）へ置いたファイバー断面で塑性化域 Lp 区間を
            // 代表し、中央は弾性とする。
            if end_plastic {
                let l = elem
                    .nodes
                    .first()
                    .zip(elem.nodes.get(1))
                    .and_then(|(i, j)| {
                        let a = app.model.nodes.get(i.index())?;
                        let b = app.model.nodes.get(j.index())?;
                        Some(len3(a.coord, b.coord))
                    })
                    .unwrap_or(0.0);
                let (_, _, l_flex) = flexible_span(elem, l);
                let lp = plastic_zone_len(elem, &app.model, l_flex);
                let src = if elem.plastic_zone.is_some() {
                    "指定値"
                } else {
                    "既定 0.5D"
                };
                ui.label("ファイバー断面: 可撓部の材端 2 箇所（積分点 ξ=∓1）");
                ui.label(format!("塑性化域 Lp={:.0} mm（{}）／中央弾性", lp, src));
                if l_flex < l - 1e-9 {
                    ui.label(format!("可撓長 L'={:.0} mm（Lp は L' 基準）", l_flex));
                }
            }
        },
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use smallvec::smallvec;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use squid_n_core::model::{ForceRegime, LocalAxis, RigidZone};
    use squid_n_core::section_shape::SectionShape;

    /// 指定した種別・フォースレジームの 2 節点部材を作る（テスト用の最小構成）。
    fn elem(kind: ElementKind, regime: ForceRegime) -> ElementData {
        ElementData {
            id: ElemId(0),
            kind,
            nodes: smallvec![NodeId(0), NodeId(1)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: regime,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    /// 静解析では断面の降伏を考えないため、梁はフォースレジームに依らず弾性。
    #[test]
    fn test_static_beam_is_elastic() {
        let model = Model::default();
        for regime in [
            ForceRegime::Auto,
            ForceRegime::UniaxialBendingShear,
            ForceRegime::AxialBendingInteract,
        ] {
            let e = elem(ElementKind::Beam, regime);
            assert_eq!(
                classify(&e, &model, ModelingAnalysis::Static),
                ModelClass::Elastic
            );
        }
    }

    /// 増分解析では、集中ばね指定の梁は材端集中塑性、軸-曲げ連成指定はファイバー。
    #[test]
    fn test_incremental_beam_regime_split() {
        let model = Model::default();
        let concentrated = elem(ElementKind::Beam, ForceRegime::UniaxialBendingShear);
        assert_eq!(
            classify(&concentrated, &model, ModelingAnalysis::Incremental),
            ModelClass::ConcentratedPlastic
        );
        let fiber = elem(ElementKind::Beam, ForceRegime::AxialBendingInteract);
        assert_eq!(
            classify(&fiber, &model, ModelingAnalysis::Incremental),
            ModelClass::Fiber
        );
    }

    /// 仕口パネルの見付き半寸法は、接合部に取り付く部材のせいから決まる。
    /// 水平半寸法は柱せいの 1/2、鉛直半寸法は梁せいの 1/2 で、解析側が剛域長へ
    /// 書き込むオフセットと同じ値になる。
    #[test]
    fn test_panel_half_extent_from_member_depths() {
        let model = cross_joint_model();
        let adjacency = NodeAdjacency::build(&model);
        let extent = panel_half_extent_at(&model, &adjacency, NodeId(0));
        assert!(
            (extent.column_half - 200.0).abs() < 1e-9,
            "水平半寸法は柱せい 400 の 1/2"
        );
        assert!(
            (extent.beam_half - 300.0).abs() < 1e-9,
            "鉛直半寸法は梁せい 600 の 1/2"
        );

        let axes = panel_beam_axes(&model, &adjacency, NodeId(0));
        assert_eq!(axes.len(), 1, "左右の梁は同一構面へまとめる: {axes:?}");
        assert!(axes[0][0].abs() > 0.99, "構面の向きは X 方向");
    }

    /// 直交する 2 方向に梁が取り付く接合部は、構面ごとに四角形を描くため
    /// 材軸も 2 本になる。
    #[test]
    fn test_panel_beam_axes_has_one_entry_per_frame_plane() {
        let node = |id: u32, coord: [f64; 3]| squid_n_core::model::Node {
            id: NodeId(id),
            coord,
            restraint: squid_n_core::dof::Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        let model = Model {
            nodes: vec![
                node(0, [0.0, 0.0, 3000.0]),
                node(1, [6000.0, 0.0, 3000.0]),
                node(2, [0.0, 6000.0, 3000.0]),
                node(3, [0.0, 0.0, 0.0]),
            ],
            sections: vec![depth_section(0, 600.0), depth_section(1, 400.0)],
            elements: vec![
                joint_member(0, 0, 1, 0), // X 方向の梁
                joint_member(1, 0, 2, 0), // Y 方向の梁
                joint_member(2, 0, 3, 1), // 柱（下向き）
            ],
            ..Default::default()
        };
        let adjacency = NodeAdjacency::build(&model);
        assert_eq!(
            panel_beam_axes(&model, &adjacency, NodeId(0)).len(),
            2,
            "直交 2 構面"
        );
        let extent = panel_half_extent_at(&model, &adjacency, NodeId(0));
        assert!((extent.beam_half - 300.0).abs() < 1e-9);
    }

    /// 断面が求まらない接合部は四角形を描けないため、見付き半寸法が 0 になる
    /// （描画側はひし形マーカーへフォールバックする）。
    #[test]
    fn test_panel_half_extent_without_sections() {
        let model = Model::default();
        let adjacency = NodeAdjacency::build(&model);
        let extent = panel_half_extent_at(&model, &adjacency, NodeId(0));
        assert_eq!(extent.column_half, 0.0);
        assert_eq!(extent.beam_half, 0.0);
        assert!(panel_beam_axes(&model, &adjacency, NodeId(0)).is_empty());
    }

    /// せいだけを与えた断面（見付き寸法の算定に必要なのはせいのみ）。
    fn depth_section(id: u32, depth: f64) -> squid_n_core::model::Section {
        squid_n_core::model::Section {
            id: SectionId(id),
            name: String::new(),
            area: 1.0e4,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e7,
            depth,
            width: depth,
            as_y: 4.0e3,
            as_z: 4.0e3,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }
    }

    fn joint_member(id: u32, n0: u32, n1: u32, sec: u32) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(n0), NodeId(n1)],
            section: Some(SectionId(sec)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: RigidZone::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    /// 節点 0 を接合部とする十字型（左右に梁せい 600、上下に柱せい 400）。
    fn cross_joint_model() -> Model {
        let node = |id: u32, coord: [f64; 3]| squid_n_core::model::Node {
            id: NodeId(id),
            coord,
            restraint: squid_n_core::dof::Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        Model {
            nodes: vec![
                node(0, [0.0, 0.0, 3000.0]),
                node(1, [-6000.0, 0.0, 3000.0]),
                node(2, [6000.0, 0.0, 3000.0]),
                node(3, [0.0, 0.0, 0.0]),
                node(4, [0.0, 0.0, 6000.0]),
            ],
            sections: vec![depth_section(0, 600.0), depth_section(1, 400.0)],
            elements: vec![
                joint_member(0, 1, 0, 0), // 左梁: j 端が接合部
                joint_member(1, 0, 2, 0), // 右梁: i 端が接合部
                joint_member(2, 3, 0, 1), // 下柱: j 端が接合部
                joint_member(3, 0, 4, 1), // 上柱: i 端が接合部
            ],
            ..Default::default()
        }
    }

    /// ブレース・パネルゾーン・その他要素の分類は解析種別に依らず一定。
    #[test]
    fn test_brace_panel_other_classes() {
        let model = Model::default();
        for analysis in [ModelingAnalysis::Static, ModelingAnalysis::Incremental] {
            assert_eq!(
                classify(
                    &elem(
                        ElementKind::Brace {
                            tension_only: false
                        },
                        ForceRegime::Auto
                    ),
                    &model,
                    analysis
                ),
                ModelClass::Truss
            );
            assert_eq!(
                classify(
                    &elem(ElementKind::PanelZone, ForceRegime::Auto),
                    &model,
                    analysis
                ),
                ModelClass::Panel
            );
            assert_eq!(
                classify(
                    &elem(ElementKind::Isolator, ForceRegime::Auto),
                    &model,
                    analysis
                ),
                ModelClass::Other
            );
        }
    }

    /// マルチスプリング梁は静解析で弾性、増分解析で材端集中塑性。
    #[test]
    fn test_multispring_class_by_analysis() {
        let model = Model::default();
        let e = elem(ElementKind::MultiSpring, ForceRegime::Auto);
        assert_eq!(
            classify(&e, &model, ModelingAnalysis::Static),
            ModelClass::Elastic
        );
        assert_eq!(
            classify(&e, &model, ModelingAnalysis::Incremental),
            ModelClass::ConcentratedPlastic
        );
    }

    /// 4000×3000 の壁 1 枚と、その四周の柱・梁を持つモデル。
    /// 耐震壁は四周を柱・梁に囲まれた壁を対象とするため、四周が揃っていないと
    /// 板厚に依らず雑壁になる。
    fn wall_model(thickness: f64) -> (Model, ElementData) {
        let node = |id: u32, coord: [f64; 3]| squid_n_core::model::Node {
            id: NodeId(id),
            coord,
            restraint: squid_n_core::dof::Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        let mut wall = elem(ElementKind::Wall, ForceRegime::Auto);
        wall.nodes = smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)];
        wall.section = Some(SectionId(0));
        wall.material = Some(MaterialId(0));

        let edge = |id: u32, n0: u32, n1: u32| {
            let mut e = elem(ElementKind::Beam, ForceRegime::Auto);
            e.id = ElemId(id);
            e.nodes = smallvec![NodeId(n0), NodeId(n1)];
            e
        };
        let shape = SectionShape::RcWall {
            thickness,
            ps: 0.0025,
        };
        let model = Model {
            nodes: vec![
                node(0, [0.0, 0.0, 0.0]),
                node(1, [4000.0, 0.0, 0.0]),
                node(2, [4000.0, 0.0, 3000.0]),
                node(3, [0.0, 0.0, 3000.0]),
            ],
            elements: vec![
                wall.clone(),
                edge(1, 0, 1), // 下辺
                edge(2, 3, 2), // 上辺
                edge(3, 0, 3), // 左の鉛直辺
                edge(4, 1, 2), // 右の鉛直辺
            ],
            sections: vec![shape.to_section(SectionId(0), "W".into())],
            ..Default::default()
        };
        (model, wall)
    }

    /// 壁は耐震壁成立で壁エレメント、板厚 120mm 未満（耐震壁不成立）で雑壁。
    #[test]
    fn test_wall_seismic_vs_misc() {
        // 板厚 150mm・四周あり → 耐震壁成立 → 壁エレメント。
        let (model, wall) = wall_model(150.0);
        assert_eq!(
            classify(&wall, &model, ModelingAnalysis::Static),
            ModelClass::Wall
        );

        // 板厚 100mm → 耐震壁不成立 → 雑壁。
        let (model, wall) = wall_model(100.0);
        assert_eq!(
            classify(&wall, &model, ModelingAnalysis::Static),
            ModelClass::WallMisc
        );
    }

    /// 上下辺の大梁が一方でも欠けた壁は、板厚が足りていても雑壁として描く。
    /// 着色は要素生成と同じ判定に基づくため、解析側の扱いと一致する。
    #[test]
    fn 上下辺の大梁が欠けた壁は雑壁として描く() {
        for drop_id in 1..=2u32 {
            let (mut model, wall) = wall_model(150.0);
            model.elements.retain(|e| e.id != ElemId(drop_id));
            assert_eq!(
                classify(&wall, &model, ModelingAnalysis::Static),
                ModelClass::WallMisc,
                "上下辺の一方（ElemId {drop_id}）が欠けた壁は雑壁"
            );
        }
    }

    /// 側柱（左右の鉛直辺）が無くても、上下辺の大梁が揃っていれば耐震壁として描く。
    /// 側柱を持たない壁は壁筋比から等価引張鉄筋比を算定する正規の対象である。
    #[test]
    fn 側柱が無くても耐震壁として描く() {
        let (mut model, wall) = wall_model(150.0);
        model
            .elements
            .retain(|e| e.id != ElemId(3) && e.id != ElemId(4));
        assert_eq!(
            classify(&wall, &model, ModelingAnalysis::Static),
            ModelClass::Wall
        );
    }

    /// 壁の上下大梁（付帯梁）は、断面性能へ倍率が乗った剛性で解析へ入る。
    /// 図では分類色を保ったまま平行線を添えるため、判定が解析と一致していること
    /// （[`squid_n_element::beam::stiffness_breakdown`] と同じ結果）を確認する。
    #[test]
    fn 壁の上下大梁だけを付帯梁として判定する() {
        let (mut model, _) = wall_model(150.0);
        // 付帯梁の判定には断面・材料が要る（`stiffness_breakdown` の前提）。
        model.materials.push(squid_n_core::model::Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "FC24".into(),
            category: squid_n_core::model::MaterialCategory::Concrete,
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        });
        model.sections.push(depth_section(1, 600.0));
        for e in model.elements.iter_mut().filter(|e| e.id != ElemId(0)) {
            e.section = Some(SectionId(1));
            e.material = Some(MaterialId(0));
        }

        let girders: Vec<u32> = model
            .elements
            .iter()
            .filter(|e| is_wall_girder(e, &model, &seismic_wall_nodes(&model)))
            .map(|e| e.id.0)
            .collect();
        assert_eq!(
            girders,
            vec![1, 2],
            "付帯梁は上下の水平材のみ（鉛直辺は対象外）"
        );
    }

    /// 端部塑性化域モデル（材端 ξ=∓1 にファイバー断面を置くモデル）の判定。
    /// 増分解析のファイバー要素と MS 要素のみが該当し、静解析（全部材弾性）や
    /// 材端回転ばねの梁は該当しない。
    #[test]
    fn 端部塑性化域モデルの判定は増分解析のファイバーとmsのみ() {
        let model = Model::default();
        let fiber = elem(ElementKind::Fiber, ForceRegime::Auto);
        let ms = elem(ElementKind::MultiSpring, ForceRegime::Auto);
        let spring_beam = elem(ElementKind::Beam, ForceRegime::UniaxialBendingShear);

        for e in [&fiber, &ms, &spring_beam] {
            // 静解析は全部材が弾性のため該当しない。
            let class = classify(e, &model, ModelingAnalysis::Static);
            assert!(!is_end_plastic_zone_model(e, class));
        }

        assert!(is_end_plastic_zone_model(
            &fiber,
            classify(&fiber, &model, ModelingAnalysis::Incremental)
        ));
        assert!(is_end_plastic_zone_model(
            &ms,
            classify(&ms, &model, ModelingAnalysis::Incremental)
        ));
        // 材端回転ばねの梁は「材端集中塑性」だが端部塑性化域モデルではない。
        assert!(!is_end_plastic_zone_model(
            &spring_beam,
            classify(&spring_beam, &model, ModelingAnalysis::Incremental)
        ));
    }

    /// 塑性化域長 Lp の表示値は要素生成の既定と一致する
    /// （`plastic_zone` 指定時はその値、未指定なら断面せいの 0.5 倍）。
    #[test]
    fn 塑性化域長は要素生成の既定と一致する() {
        // 断面せい 600mm → 既定 Lp = 300mm。
        let shape = SectionShape::SteelH {
            height: 600.0,
            width: 200.0,
            web_thick: 11.0,
            flange_thick: 17.0,
        };
        let model = Model {
            sections: vec![shape.to_section(SectionId(0), "H-600x200".into())],
            ..Default::default()
        };
        let mut e = elem(ElementKind::Fiber, ForceRegime::Auto);
        e.section = Some(SectionId(0));
        assert!((plastic_zone_len(&e, &model, 3000.0) - 300.0).abs() < 1e-6);

        // 指定値が優先される。
        e.plastic_zone = Some(600.0);
        assert!((plastic_zone_len(&e, &model, 3000.0) - 600.0).abs() < 1e-6);

        // 可撓長の 45% を超える指定はクランプされる（要素生成と同じ規則）。
        e.plastic_zone = Some(3000.0);
        assert!((plastic_zone_len(&e, &model, 3000.0) - 1350.0).abs() < 1e-6);
    }

    /// 可とう区間・可撓長の算定は要素側（`rigid_arm::resolve_lengths`）と一致する。
    /// 剛域長の合計が部材長以上の入力は剛域なしとして扱う。
    #[test]
    fn 可とう区間は要素側の剛域解決と一致する() {
        let mut e = elem(ElementKind::Beam, ForceRegime::Auto);
        e.rigid_zone = RigidZone {
            length_i: 400.0,
            length_j: 200.0,
            face_i: 400.0,
            face_j: 200.0,
            ..Default::default()
        };
        let (s_i, s_j, l_flex) = flexible_span(&e, 3000.0);
        assert!((s_i - 400.0 / 3000.0).abs() < 1e-6);
        assert!((s_j - (1.0 - 200.0 / 3000.0)).abs() < 1e-6);
        assert!((l_flex - 2400.0).abs() < 1e-9);

        // 可撓長が残らない入力は剛域なし。
        e.rigid_zone = RigidZone {
            length_i: 2000.0,
            length_j: 1500.0,
            ..Default::default()
        };
        let (s_i, s_j, l_flex) = flexible_span(&e, 3000.0);
        assert_eq!((s_i, s_j), (0.0, 1.0));
        assert!((l_flex - 3000.0).abs() < 1e-9);
    }
}
