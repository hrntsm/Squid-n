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
//! - **ファイバー／MS**: 解析上は材端（積分点 ξ=∓1、すなわち節点位置）にファイバー
//!   断面を置き、その積分重み＝塑性化域長 Lp の区間だけが塑性化する。中央
//!   \[Lp, L−Lp\] は弾性。よって端部 Lp 区間を太線で強調し、中央を弾性色の細線とし、
//!   ファイバー断面の位置に断面記号を描く。Lp は要素生成と同じ既定
//!   （[`squid_n_element::factory::plastic_zone_length`]）で解決する。
//!   これらの要素は剛域を考慮しない（積分点は剛域フェイスではなく節点位置）ため、
//!   剛域は「未考慮」を示す破線アウトラインで描く。
//! - **端部接合条件**: ピン（○）・半剛（□）を材端（剛域がある場合は剛域フェイス）に描く。
//! - **壁エレメント**: 耐震壁は壁エレメント置換モデル（壁柱＋両端ピンの上下剛梁）の
//!   「エ」状で描く。フレーム内雑壁（周辺部材へ剛性算入）は半透明ポリゴンで区別する。
//! - **パネルゾーン**: モデル化されていれば接合部中心にマーカーを描く。
//!
//! 分類ロジックは要素生成（`squid_n_element::factory`）と同じ判定関数
//! （[`resolve_force_regime`] / [`wall_side_column_release`] / [`wall_is_seismic`]）を
//! 用いるため、実際に解析へ渡る要素種別と一致する。

use crate::app::App;
use crate::theme;
use squid_n_core::model::{ElementData, ElementKind, EndCondition, Model};
use squid_n_element::factory::{resolve_force_regime, ResolvedRegime};
use squid_n_element::misc_wall::wall_is_seismic;
use squid_n_element::side_column::wall_side_column_release;
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
    /// パネルゾーン（柱梁接合部パネル）。
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
            // パネルゾーン＝藍
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
            ModelClass::Wall => "壁エレメント(エ型)",
            ModelClass::WallMisc => "雑壁(周辺部材へ剛性算入)",
            ModelClass::Panel => "パネルゾーン",
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
    match data.kind {
        // 梁・柱（Beam）とファイバー梁（Fiber）は解析種別で扱いが変わる。
        ElementKind::Beam | ElementKind::Fiber => {
            // 耐震壁の側柱は面内両端ピン（トポロジ由来の解放。解析種別に依らない）。
            if wall_side_column_release(data, model).is_some() {
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

/// 解析上考慮されない剛域（ファイバー／MS 要素）を淡い破線の輪郭だけで示す。
///
/// これらの要素（`FiberBeam::build_plastic_zone` ベース）は剛域変換を行わず、
/// 積分点を節点位置に置くため、剛域長を入力しても解析には効かない。入力値を
/// 黙って隠すとモデル化図が入力と食い違って見えるため、ハッチのない破線の
/// 「空のブロック」として、考慮される剛域と区別して描く。
fn draw_ignored_rigid_zone(painter: &egui::Painter, a: egui::Pos2, b: egui::Pos2) {
    let n = perpendicular(a, b) * RIGID_HALF_W;
    let stroke = egui::Stroke::new(1.0_f32, theme::translucent(theme::GRAY_900, 110));
    for side in [-1.0_f32, 1.0] {
        painter.extend(egui::Shape::dashed_line(
            &[a + n * side, b + n * side],
            stroke,
            4.0,
            3.0,
        ));
    }
    painter.line_segment([a - n, a + n], stroke);
    painter.line_segment([b - n, b + n], stroke);
}

/// ファイバー断面（端部積分点 ξ=∓1）の記号。材軸に直交する短いバーと、その上に
/// 並ぶ 3 点の塗り円で「断面をファイバーへ分割している位置」を表す。
///
/// 位置は解析上の積分点（材端＝節点位置）だが、接合部では複数部材の記号が
/// 同一点に重なって判読できないため、材軸方向へわずかに内側へ寄せて描く
/// （他の材端記号と同じ描画上のオフセット規約）。
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

/// 剛域を考慮した可とう区間の端点フラクション（材軸パラメータ s∈[0,1] の両端）。
/// 剛域長が可とう長を食い尽くさないよう各端 0.45 で頭打ちにする。
fn flexible_span(elem: &ElementData, l: f64) -> (f32, f32) {
    if l <= 1e-9 {
        return (0.0, 1.0);
    }
    let fi = (elem.rigid_zone.length_i.max(0.0) / l).clamp(0.0, 0.45) as f32;
    let fj = (elem.rigid_zone.length_j.max(0.0) / l).clamp(0.0, 0.45) as f32;
    (fi, 1.0 - fj)
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
) {
    let model = &app.model;
    let analysis = app.modeling_analysis;

    // 凡例に載せる情報を収集する。
    let mut present: Vec<ModelClass> = Vec::new();
    let mut sym = Symbols::default();

    for elem in &model.elements {
        let class = classify(elem, model, analysis);
        if !present.contains(&class) {
            present.push(class);
        }

        match class {
            ModelClass::Wall => {
                draw_wall_element(painter, model, pts, proj, elem, class.color(), &mut sym)
            }
            ModelClass::WallMisc => draw_wall_polygon(painter, pts, elem, class.color(), true),
            ModelClass::Panel => draw_panel_zone(painter, pts, elem, class.color()),
            _ => draw_line_member(painter, model, pts, coords3, elem, class, &mut sym),
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
    /// 剛域の入力はあるが要素が考慮しない（ファイバー／MS 要素）ケースが現れた。
    rigid_ignored: bool,
    /// ファイバー断面（端部積分点）の記号を描いた。
    fiber_section: bool,
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

/// 端部塑性化域モデルの有効な塑性化域長 Lp を材長比（s∈[0,1]）で返す。
/// 要素生成と同じ既定・同じクランプを用いるため、表示は解析のモデル化と一致する。
fn plastic_zone_frac(elem: &ElementData, model: &Model, l: f64) -> f32 {
    if l <= 1e-9 {
        return 0.0;
    }
    let lp = squid_n_element::fiber::clamp_plastic_zone(
        squid_n_element::factory::plastic_zone_length(elem, model),
        l,
    );
    (lp / l) as f32
}

/// 線材（梁・柱・ファイバー・側柱）のモデル化を描く。
fn draw_line_member(
    painter: &egui::Painter,
    model: &Model,
    pts: &[egui::Pos2],
    coords3: &[[f64; 3]],
    elem: &ElementData,
    class: ModelClass,
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

    // 端部塑性化域モデル（ファイバー／MS）は剛域を考慮せず、材端＝節点位置を
    // 積分点とするため、可とう区間ではなく全長を基準に描く。
    let end_plastic = is_end_plastic_zone_model(elem, class);
    let (s_i, s_j) = if end_plastic {
        (0.0, 1.0)
    } else {
        flexible_span(elem, l)
    };

    // 材端の記号を置く位置（剛域を考慮する要素は剛域フェイス、端部塑性化域
    // モデルは節点位置）。
    let fa = lerp(p0, p1, s_i);
    let fb = lerp(p0, p1, s_j);

    if end_plastic {
        // 端部 Lp 区間 = ファイバー断面の積分重み（塑性化域）。中央は弾性。
        let lp = plastic_zone_frac(elem, model, l).min(0.5);
        let a = lerp(p0, p1, lp);
        let b = lerp(p0, p1, 1.0 - lp);
        painter.line_segment(
            [a, b],
            egui::Stroke::new(3.0_f32, ModelClass::Elastic.color()),
        );
        let zone_stroke = egui::Stroke::new(5.0_f32, color);
        painter.line_segment([p0, a], zone_stroke);
        painter.line_segment([b, p1], zone_stroke);
        // 剛域の入力があっても要素は考慮しない旨を破線で示す。
        let (r_i, r_j) = flexible_span(elem, l);
        if r_i > 0.0 {
            draw_ignored_rigid_zone(painter, p0, lerp(p0, p1, r_i));
            sym.rigid_ignored = true;
        }
        if r_j < 1.0 {
            draw_ignored_rigid_zone(painter, lerp(p0, p1, r_j), p1);
            sym.rigid_ignored = true;
        }
        // ファイバー断面（積分点 ξ=∓1）の位置。
        draw_fiber_section_marker(painter, p0, p1, color);
        draw_fiber_section_marker(painter, p1, p0, color);
        sym.fiber_section = true;
    } else {
        // 可とう区間の基準線。
        painter.line_segment([fa, fb], egui::Stroke::new(3.0_f32, color));

        // 剛域バー（材端）。
        if s_i > 0.0 {
            draw_rigid_zone(painter, p0, fa);
            sym.rigid = true;
        }
        if s_j < 1.0 {
            draw_rigid_zone(painter, fb, p1);
            sym.rigid = true;
        }
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
/// 壁柱（上下剛梁の中点を結ぶ仮想中央柱）を鉛直線で、上下の剛梁を暗色の太線で描き、
/// 四隅（剛梁端＝ピン接合）に○を置く。幾何を取れない場合はポリゴンへフォールバックする。
fn draw_wall_element(
    painter: &egui::Painter,
    model: &Model,
    pts: &[egui::Pos2],
    proj: &Projector,
    elem: &ElementData,
    color: egui::Color32,
    sym: &mut Symbols,
) {
    let Some(g) = wall_panel_geometry(elem, model) else {
        draw_wall_polygon(painter, pts, elem, color, false);
        return;
    };
    let (b0, b1) = (g.bottom[0].index(), g.bottom[1].index());
    let (t0, t1) = (g.top[0].index(), g.top[1].index());
    if [b0, b1, t0, t1].iter().any(|&i| i >= pts.len()) {
        draw_wall_polygon(painter, pts, elem, color, false);
        return;
    }
    let (pb0, pb1, pt0, pt1) = (pts[b0], pts[b1], pts[t0], pts[t1]);
    let bc = proj.project(g.bottom_center);
    let tc = proj.project(g.top_center);

    // 上下の剛梁（剛域と同じ表記の太いバー）。
    draw_rigid_zone(painter, pb0, pb1);
    draw_rigid_zone(painter, pt0, pt1);
    sym.rigid = true;

    // 壁柱（中央鉛直材）。
    painter.line_segment([bc, tc], egui::Stroke::new(3.0_f32, color));

    // 四隅のピン（剛梁端＝ピン接合）。剛梁の他端側へ寄せて描く。
    draw_pin_marker(painter, pb0, pb1, color);
    draw_pin_marker(painter, pb1, pb0, color);
    draw_pin_marker(painter, pt0, pt1, color);
    draw_pin_marker(painter, pt1, pt0, color);
    sym.pin = true;
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

/// パネルゾーン（柱梁接合部パネル）を接合部中心のマーカー（塗りひし形）で描く。
fn draw_panel_zone(
    painter: &egui::Painter,
    pts: &[egui::Pos2],
    elem: &ElementData,
    color: egui::Color32,
) {
    let Some(center) = elem.nodes.first().map(|n| n.index()) else {
        return;
    };
    if center >= pts.len() {
        return;
    }
    let c = pts[center];
    // 接続節点へ細線を引き、接合部パネルであることを示す。
    for n in elem.nodes.iter().skip(1) {
        let i = n.index();
        if i < pts.len() {
            painter.line_segment(
                [c, lerp(c, pts[i], 0.35)],
                egui::Stroke::new(1.5_f32, theme::translucent(color, 160)),
            );
        }
    }
    // 中心にひし形マーカー。
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
        text(painter, y, "剛域");
        y += LINE_H;
    }
    if sym.rigid_ignored {
        draw_ignored_rigid_zone(
            painter,
            egui::pos2(x0 + 2.0, y + FONT * 0.5),
            egui::pos2(x0 + 18.0, y + FONT * 0.5),
        );
        text(painter, y, "剛域（この要素では未考慮）");
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
        text(painter, y, "ファイバー断面（積分点 ξ=∓1、材端）");
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
                let rz = &elem.rigid_zone;
                if rz.length_i > 0.0 || rz.length_j > 0.0 {
                    let note = if end_plastic { "（未考慮）" } else { "" };
                    ui.label(format!(
                        "剛域長: i={:.0} / j={:.0} mm{}",
                        rz.length_i, rz.length_j, note
                    ));
                }
            }
            // 端部塑性化域モデル（ファイバー／MS）は、材端（積分点 ξ=∓1）へ置いた
            // ファイバー断面で塑性化域 Lp 区間を代表し、中央は弾性とする。
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
                let lp = squid_n_element::fiber::clamp_plastic_zone(
                    squid_n_element::factory::plastic_zone_length(elem, &app.model),
                    l,
                );
                let src = if elem.plastic_zone.is_some() {
                    "指定値"
                } else {
                    "既定 0.5D"
                };
                ui.label("ファイバー断面: 材端 2 箇所（積分点 ξ=∓1）");
                ui.label(format!("塑性化域 Lp={:.0} mm（{}）／中央弾性", lp, src));
                ui.label("※ 積分点は節点位置（剛域は考慮しない）");
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

    /// 壁は耐震壁成立で壁エレメント、板厚 120mm 未満（耐震壁不成立）で雑壁。
    #[test]
    fn test_wall_seismic_vs_misc() {
        let mut wall = elem(ElementKind::Wall, ForceRegime::Auto);
        wall.nodes = smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)];
        wall.section = Some(SectionId(0));
        wall.material = Some(MaterialId(0));

        // 板厚 150mm → 耐震壁成立 → 壁エレメント。
        let seismic = SectionShape::RcWall {
            thickness: 150.0,
            ps: 0.0025,
        };
        let model_seismic = Model {
            sections: vec![seismic.to_section(SectionId(0), "W150".into())],
            ..Default::default()
        };
        assert_eq!(
            classify(&wall, &model_seismic, ModelingAnalysis::Static),
            ModelClass::Wall
        );

        // 板厚 100mm → 耐震壁不成立 → 雑壁。
        let misc = SectionShape::RcWall {
            thickness: 100.0,
            ps: 0.0025,
        };
        let model_misc = Model {
            sections: vec![misc.to_section(SectionId(0), "W100".into())],
            ..Default::default()
        };
        assert_eq!(
            classify(&wall, &model_misc, ModelingAnalysis::Static),
            ModelClass::WallMisc
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
        // 断面せい 600mm → 既定 Lp = 300mm。部材長 3000mm なので比は 0.1。
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
        assert!((plastic_zone_frac(&e, &model, 3000.0) - 0.1).abs() < 1e-6);

        // 指定値が優先される。
        e.plastic_zone = Some(600.0);
        assert!((plastic_zone_frac(&e, &model, 3000.0) - 0.2).abs() < 1e-6);

        // 部材長の 45% を超える指定はクランプされる（要素生成と同じ規則）。
        e.plastic_zone = Some(3000.0);
        assert!((plastic_zone_frac(&e, &model, 3000.0) - 0.45).abs() < 1e-6);
    }
}
