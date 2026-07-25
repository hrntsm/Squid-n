//! モデル化図（解析上どの要素モデルで部材を扱っているかの可視化）の描画。
//!
//! 同じ形状のモデルでも、解析種別によって部材の要素定式化（モデル化）は変わる。
//! 本ビューは [`ModelingAnalysis`]（静解析＝弾性／増分解析＝弾塑性）を切り替えつつ、
//! 各部材が解析上どのモデルへ振り分けられるかを色と記号で示し、意図どおりの
//! モデル化になっているか（例: 耐震壁の側柱が面内両端ピンになっているか、剛床上の
//! 梁が材端集中塑性で、軸力変動する柱がファイバーになっているか）を視覚的に確認
//! できるようにする。
//!
//! 分類ロジックは要素生成（`squid_n_element::factory`）と同じ判定関数
//! （[`resolve_force_regime`] / [`wall_side_column_release`]）を用いるため、
//! 実際に解析へ渡る要素種別と一致する。

use crate::app::App;
use crate::theme;
use squid_n_core::model::{ElementData, ElementKind, EndCondition, Model};
use squid_n_element::factory::{resolve_force_regime, ResolvedRegime};
use squid_n_element::side_column::wall_side_column_release;

use super::ModelingAnalysis;

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
    /// 壁エレメント（壁パネル置換モデル。増分解析ではせん断降伏を考慮）。
    Wall,
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
        ElementKind::Wall => ModelClass::Wall,
        ElementKind::Brace { .. } => ModelClass::Truss,
        // 面要素・接合部・バネ・免震・ダンパーなど。
        ElementKind::Shell
        | ElementKind::PanelZone
        | ElementKind::NodalSpring
        | ElementKind::Isolator
        | ElementKind::Damper => ModelClass::Other,
    }
}

/// 端部ピンマーカー（節点から材軸方向へ少し内側に置いた白抜きの円）を描く。
/// `node` は端部の節点スクリーン座標、`toward` は他端側の点（内側方向の決定に使う）。
fn draw_pin_marker(
    painter: &egui::Painter,
    node: egui::Pos2,
    toward: egui::Pos2,
    color: egui::Color32,
) {
    const OFFSET: f32 = 9.0;
    const RADIUS: f32 = 4.0;
    let dir = toward - node;
    let len = dir.length();
    let center = if len > 1e-3 {
        egui::pos2(node.x + dir.x / len * OFFSET, node.y + dir.y / len * OFFSET)
    } else {
        node
    };
    // 白抜きの円（内部は背景色で塗り、輪郭を色付き）＝ピン（回転自由）の慣用記号。
    painter.circle_filled(center, RADIUS, theme::WHITE);
    painter.circle_stroke(center, RADIUS, egui::Stroke::new(1.5_f32, color));
}

/// 端部半剛（`SemiRigid`）マーカー（節点内側に置いた小さな正方形）を描く。
fn draw_semi_rigid_marker(
    painter: &egui::Painter,
    node: egui::Pos2,
    toward: egui::Pos2,
    color: egui::Color32,
) {
    const OFFSET: f32 = 9.0;
    const HALF: f32 = 3.5;
    let dir = toward - node;
    let len = dir.length();
    let center = if len > 1e-3 {
        egui::pos2(node.x + dir.x / len * OFFSET, node.y + dir.y / len * OFFSET)
    } else {
        node
    };
    let rect = egui::Rect::from_center_size(center, egui::vec2(HALF * 2.0, HALF * 2.0));
    painter.rect_filled(rect, 1.0, theme::WHITE);
    painter.rect_stroke(
        rect,
        1.0,
        egui::Stroke::new(1.5_f32, color),
        egui::StrokeKind::Middle,
    );
}

/// モデル化図を描く。`pts` は `viewer_panel` で計算済みの節点スクリーン座標
/// （`app.model.nodes` と同じ順序）。基本形状（節点・部材線）の上に、解析モデル
/// 分類ごとの色で部材を塗り、端部の接合条件（ピン・半剛）を記号で重ねる。
pub(super) fn draw_modeling(painter: &egui::Painter, app: &App, pts: &[egui::Pos2]) {
    let model = &app.model;
    let analysis = app.modeling_analysis;

    // 凡例に載せるため、実際に現れた分類を出現順で集める。
    let mut present: Vec<ModelClass> = Vec::new();
    let mut any_pin = false;
    let mut any_semi = false;

    for elem in &model.elements {
        let class = classify(elem, model, analysis);
        let color = class.color();
        if !present.contains(&class) {
            present.push(class);
        }

        // 壁（面要素）は半透明ポリゴン＋色付き輪郭で描く。
        if elem.kind == ElementKind::Wall && elem.nodes.len() >= 3 {
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
                    theme::translucent(color, 45),
                    egui::Stroke::new(2.0_f32, color),
                ));
            }
            continue;
        }

        if elem.nodes.len() < 2 {
            continue;
        }
        let n0 = elem.nodes[0].index();
        let n1 = elem.nodes[1].index();
        if n0 >= pts.len() || n1 >= pts.len() {
            continue;
        }
        let (p0, p1) = (pts[n0], pts[n1]);

        // 線材: 両端を結ぶ線を分類色で描く。
        painter.line_segment([p0, p1], egui::Stroke::new(3.0_f32, color));

        // 端部の接合条件を記号で重ねる。
        // - 側柱は面内両端ピンのため、両端にピンマーカーを描く。
        // - それ以外は入力された端条件（Pinned / SemiRigid）を端ごとに描く。
        if class == ModelClass::SideColumnPin {
            draw_pin_marker(painter, p0, p1, color);
            draw_pin_marker(painter, p1, p0, color);
            any_pin = true;
        } else {
            for (end_idx, near, far) in [(0usize, p0, p1), (1usize, p1, p0)] {
                match elem.end_cond[end_idx] {
                    EndCondition::Pinned => {
                        draw_pin_marker(painter, near, far, color);
                        any_pin = true;
                    }
                    EndCondition::SemiRigid { .. } => {
                        draw_semi_rigid_marker(painter, near, far, color);
                        any_semi = true;
                    }
                    EndCondition::Fixed => {}
                }
            }
        }
    }

    draw_legend(painter, analysis, &present, any_pin, any_semi);
}

/// モデル化図の凡例をビュー左上に描く（支持条件凡例は左下のため衝突しない）。
fn draw_legend(
    painter: &egui::Painter,
    analysis: ModelingAnalysis,
    present: &[ModelClass],
    any_pin: bool,
    any_semi: bool,
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
        // 色サンプル（短い線分）
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

    // 記号の凡例（現れた場合のみ）
    if any_pin {
        let cx = x0 + 10.0;
        let cy = y + FONT * 0.5;
        painter.circle_filled(egui::pos2(cx, cy), 4.0, theme::WHITE);
        painter.circle_stroke(
            egui::pos2(cx, cy),
            4.0,
            egui::Stroke::new(1.5_f32, theme::GRAY_600),
        );
        painter.text(
            egui::pos2(x0 + 28.0, y),
            egui::Align2::LEFT_TOP,
            "○ 端部ピン（回転自由）",
            egui::FontId::proportional(FONT),
            theme::GRAY_600,
        );
        y += LINE_H;
    }
    if any_semi {
        let cx = x0 + 10.0;
        let cy = y + FONT * 0.5;
        let r = egui::Rect::from_center_size(egui::pos2(cx, cy), egui::vec2(7.0, 7.0));
        painter.rect_filled(r, 1.0, theme::WHITE);
        painter.rect_stroke(
            r,
            1.0,
            egui::Stroke::new(1.5_f32, theme::GRAY_600),
            egui::StrokeKind::Middle,
        );
        painter.text(
            egui::pos2(x0 + 28.0, y),
            egui::Align2::LEFT_TOP,
            "□ 端部半剛（回転ばね）",
            egui::FontId::proportional(FONT),
            theme::GRAY_600,
        );
    }
}

/// モデル化図のホバー詳細ツールチップ。部材の解析モデル分類と端条件を表示する。
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
            if matches!(elem.kind, ElementKind::Beam | ElementKind::Fiber)
                && wall_side_column_release(elem, &app.model).is_none()
            {
                ui.label(format!(
                    "端条件: i={} / j={}",
                    end_label(elem.end_cond[0]),
                    end_label(elem.end_cond[1])
                ));
            }
        },
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use smallvec::smallvec;
    use squid_n_core::ids::{ElemId, NodeId};
    use squid_n_core::model::{ForceRegime, LocalAxis, RigidZone};

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

    /// 壁・ブレース・その他要素の分類は解析種別に依らず一定。
    #[test]
    fn test_wall_brace_other_classes() {
        let model = Model::default();
        for analysis in [ModelingAnalysis::Static, ModelingAnalysis::Incremental] {
            assert_eq!(
                classify(
                    &elem(ElementKind::Wall, ForceRegime::Auto),
                    &model,
                    analysis
                ),
                ModelClass::Wall
            );
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
}
