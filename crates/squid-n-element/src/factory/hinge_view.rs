//! 解析で実際に使用した非線形特性（M-θ 骨格 / N-M 相関 / N-M 曲面）の
//! 読み取り専用ビュー。GUI が解析と同じ計算経路・パラメータ解決でヒンジの
//! 背景曲線を描くための API。
//!
//! 単位: 長さ [mm]、力 [N]、モーメント [N·mm]、回転角 [rad]。

use squid_n_core::error::RebarGeometryError;
use squid_n_core::model::{AnalysisKind, ElementData, ElementKind, Model};
use squid_n_section::fiber::Fiber;
use squid_n_section::mn_surface::{
    build_surface, concrete_young, FiberRegion, MnSurface, PlasticFiber, StrengthParams,
    YieldModelKind,
};

use super::regime::{resolve_force_regime, ResolvedRegime};
use super::springs::{build_flexural_springs, yield_moment_and_axial};
use super::{resolve_member_hysteresis, StrengthBasis};
use crate::frame::concentrated::MnInteraction;
use crate::frame::fiber::{
    build_gauss_fiber_pair, fiber_strength_params, fiber_yield_covers_shape, resolve_fiber_yield,
    FIBER_ND, FIBER_NW,
};
use crate::frame::multi_spring::{MS_ND, MS_NW};

/// 解析が要素ごとに使用する非線形モデルの種別。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AnalysisHingeModel {
    /// 材端集中ばね（M-θ 骨格 + N-M 線形相関）。
    ConcentratedSpring,
    /// ファイバー要素（N-M 曲面）。
    Fiber,
    /// マルチスプリング要素（N-M 曲面）。
    MultiSpring,
    /// 非線形ヒンジモデルを持たない（線形・ブレース・壁・その他）。
    Other,
}

/// 解析で使用した非線形特性のビュー。
pub struct HingeView {
    /// 解析モデル種別。
    pub model: AnalysisHingeModel,
    /// M-θ 骨格 [theta(rad), moment(N·mm)]。持たないモデルは `None`。
    pub backbone: Option<Vec<[f64; 2]>>,
    /// N-M 線形相関（材端集中ばね）。持たないモデル・非対応の履歴則は `None`。
    pub mn_linear: Option<MnInteraction>,
    /// 3D N-M 曲面（ファイバー／MS）。持たないモデルは `None`。
    pub mn_surface: Option<MnSurface>,
}

impl HingeView {
    fn none(model: AnalysisHingeModel) -> Self {
        HingeView {
            model,
            backbone: None,
            mn_linear: None,
            mn_surface: None,
        }
    }
}

/// 解析で実際に使用した非線形特性を返す。
///
/// `axial_force_compression_positive` [N] は材端集中ばねの N-M 線形相関に用いる
/// 軸力。`MnInteraction::moment_limit` は絶対値のみを使うため引張正・圧縮正の
/// 別は問わない。`n_alpha`・`n_beta` は N-M 曲面の経線・周方向分割数。
pub fn build_hinge_view(
    data: &ElementData,
    model: &Model,
    basis: StrengthBasis,
    kind: AnalysisKind,
    axial_force_compression_positive: f64,
    n_alpha: usize,
    n_beta: usize,
) -> Result<HingeView, RebarGeometryError> {
    match data.kind {
        ElementKind::Beam => beam_view(
            data,
            model,
            basis,
            kind,
            axial_force_compression_positive,
            n_alpha,
            n_beta,
        ),
        ElementKind::Fiber => fiber_view(data, model, basis, kind, n_alpha, n_beta),
        ElementKind::MultiSpring => multi_spring_view(data, model, basis, kind, n_alpha, n_beta),
        _ => Ok(HingeView::none(AnalysisHingeModel::Other)),
    }
}

/// [`build_hinge_view`] が `Beam` に対して材端集中ばね分岐を取るか。
/// 壁側柱の面内解放がある場合と、ファイバー／MS に解決される場合は `false`。
pub fn resolves_to_concentrated_spring(data: &ElementData, model: &Model) -> bool {
    data.kind == ElementKind::Beam
        && crate::wall::side_column::wall_side_column_release(data, model).is_none()
        && matches!(
            resolve_force_regime(data, model),
            ResolvedRegime::ConcentratedSpring
        )
}

/// `Beam` はフォースレジームで材端集中ばね／ファイバーに分岐する。
/// 壁側柱の面内解放は非線形ヒンジを持たないため `Other`。
#[allow(clippy::too_many_arguments)]
fn beam_view(
    data: &ElementData,
    model: &Model,
    basis: StrengthBasis,
    kind: AnalysisKind,
    axial_force_compression_positive: f64,
    n_alpha: usize,
    n_beta: usize,
) -> Result<HingeView, RebarGeometryError> {
    if resolves_to_concentrated_spring(data, model) {
        if super::input_check::member_strength_issue(data, model).is_some() {
            return Ok(HingeView::none(AnalysisHingeModel::ConcentratedSpring));
        }
        let rule = resolve_member_hysteresis(data, model, kind);
        let (_i, _j, backbone) = build_flexural_springs(data, model, rule, basis);
        let (my0, n_allow) = yield_moment_and_axial(data, model, basis);
        let mn = backbone.use_mn.then(|| MnInteraction::new(my0, n_allow));
        let points = backbone.points_with_mn(mn.as_ref(), axial_force_compression_positive);
        return Ok(HingeView {
            model: AnalysisHingeModel::ConcentratedSpring,
            backbone: Some(points),
            mn_linear: mn,
            mn_surface: None,
        });
    }
    if crate::wall::side_column::wall_side_column_release(data, model).is_some() {
        return Ok(HingeView::none(AnalysisHingeModel::Other));
    }
    fiber_view(data, model, basis, kind, n_alpha, n_beta)
}

/// ファイバー要素（解析の `nw=12`, `nd=20`）の N-M 曲面。
fn fiber_view(
    data: &ElementData,
    model: &Model,
    basis: StrengthBasis,
    kind: AnalysisKind,
    n_alpha: usize,
    n_beta: usize,
) -> Result<HingeView, RebarGeometryError> {
    Ok(surface_view(
        AnalysisHingeModel::Fiber,
        YieldModelKind::MultiFiber,
        analysis_plastic_fibers(data, model, basis, kind, FIBER_NW, FIBER_ND)?,
        n_alpha,
        n_beta,
    ))
}

/// マルチスプリング要素（解析の `MS_NW × MS_ND`）の N-M 曲面。
fn multi_spring_view(
    data: &ElementData,
    model: &Model,
    basis: StrengthBasis,
    kind: AnalysisKind,
    n_alpha: usize,
    n_beta: usize,
) -> Result<HingeView, RebarGeometryError> {
    Ok(surface_view(
        AnalysisHingeModel::MultiSpring,
        YieldModelKind::MultiSpring,
        analysis_plastic_fibers(data, model, basis, kind, MS_NW, MS_ND)?,
        n_alpha,
        n_beta,
    ))
}

fn surface_view(
    model_kind: AnalysisHingeModel,
    surface_kind: YieldModelKind,
    fibers: Option<Vec<PlasticFiber>>,
    n_alpha: usize,
    n_beta: usize,
) -> HingeView {
    match fibers.filter(|f| !f.is_empty()) {
        Some(fibers) => HingeView {
            model: model_kind,
            backbone: None,
            mn_linear: None,
            mn_surface: Some(build_surface(&fibers, surface_kind, n_alpha, n_beta)),
        },
        None => HingeView::none(model_kind),
    }
}

/// 解析が実際に生成する端部ファイバ断面を `PlasticFiber` 群へ変換する。
/// 断面未定義、または材料強度（Fc・fy）を解決できない場合は `Ok(None)`。
/// 実配筋を生成できない場合は [`RebarGeometryError`] を返す。
pub(crate) fn analysis_plastic_fibers(
    data: &ElementData,
    model: &Model,
    basis: StrengthBasis,
    kind: AnalysisKind,
    nw: usize,
    nd: usize,
) -> Result<Option<Vec<PlasticFiber>>, RebarGeometryError> {
    let Some(sec) = data.section.and_then(|sid| model.sections.get(sid.index())) else {
        return Ok(None);
    };
    if super::input_check::member_strength_issue(data, model).is_some() {
        return Ok(None);
    }
    if !fiber_yield_covers_shape(sec.shape.as_ref(), &resolve_fiber_yield(model, data)) {
        return Ok(None);
    }
    let strength = fiber_strength_params(data, model, basis);
    let [(section, _mats), _] =
        build_gauss_fiber_pair(data, model, basis, kind, sec.width, sec.depth, nw, nd)?;
    Ok(Some(
        section
            .fibers
            .iter()
            .map(|f| to_plastic_fiber(f, &strength))
            .collect(),
    ))
}

/// ファイバの材料区分タグ（0=コンクリート／1=主筋／2=鋼材）から全塑性計算用の
/// 限界応力・弾性係数・領域区分を決める。
fn to_plastic_fiber(f: &Fiber, strength: &StrengthParams) -> PlasticFiber {
    match f.material {
        1 => PlasticFiber {
            y: f.y,
            z: f.z,
            area: f.area,
            sigma_t: strength.rebar_fy,
            sigma_c: -strength.rebar_fy,
            young: 205000.0,
            region: FiberRegion::Rebar,
        },
        2 => PlasticFiber {
            y: f.y,
            z: f.z,
            area: f.area,
            sigma_t: strength.steel_fy,
            sigma_c: -strength.steel_fy,
            young: strength.steel_e,
            region: FiberRegion::Steel,
        },
        _ => PlasticFiber {
            y: f.y,
            z: f.z,
            area: f.area,
            sigma_t: 0.0,
            sigma_c: -strength.concrete_fc,
            young: concrete_young(strength.concrete_fc),
            region: FiberRegion::Concrete,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::behavior::ElementBehavior;
    use crate::frame::multi_spring::MultiSpringElement;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
    use squid_n_core::model::{
        Constraint, EndCondition, ForceRegime, HysteresisModel, LocalAxis, Material,
        MaterialCategory, Node, Section,
    };
    use squid_n_core::section_shape::{
        RcBeamRebar, RcRectColumnRebar, RectColumnHoop, SectionShape,
    };

    /// 剛床所属の水平梁・鉛直柱を持つ小モデル。梁 [0,1] は集中ばね、
    /// 柱 [0,2] はファイバーに判定される。
    fn make_model(shape: Option<SectionShape>, fc: Option<f64>) -> Model {
        Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                    support_spring: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [5000.0, 0.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                    support_spring: None,
                },
                Node {
                    id: NodeId(2),
                    coord: [0.0, 0.0, 3000.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                    support_spring: None,
                },
            ],
            constraints: vec![Constraint::rigid_diaphragm(
                StoryId(0),
                NodeId(2),
                vec![NodeId(1)],
            )],
            sections: vec![Section {
                id: SectionId(0),
                name: "sec".into(),
                area: 8000.0,
                iy: 1.0e8,
                iz: 1.0e8,
                j: 1.0e8,
                depth: 400.0,
                width: 200.0,
                as_y: 6666.7,
                as_z: 6666.7,
                floor: None,
                panel_thickness: None,
                thickness: None,
                shape,
                material: Some(MaterialId(0)),
                rebar_material: None,
                shear_rebar_material: None,
                steel_material: None,
            }],
            materials: vec![Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "mat".into(),
                category: MaterialCategory::Steel,
                young: 205000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
                fc,
                fy: Some(235.0),
            }],
            ..Default::default()
        }
    }

    fn elem(kind: ElementKind, nodes: [NodeId; 2]) -> ElementData {
        ElementData {
            id: ElemId(0),
            kind,
            nodes: smallvec::smallvec![nodes[0], nodes[1]],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    fn rc_shape() -> SectionShape {
        use squid_n_core::section_shape::BeamStirrup;
        SectionShape::RcBeamRect {
            b: 400.0,
            d: 700.0,
            rebar: RcBeamRebar {
                main_dia: 22.0,
                top: vec![4],
                bottom: vec![4],
                cover: 50.0,
                stirrup: BeamStirrup {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
        }
    }

    fn rc_column_shape() -> SectionShape {
        SectionShape::RcColumnRect {
            b: 400.0,
            d: 700.0,
            rebar: RcRectColumnRebar {
                main_dia: 22.0,
                x: vec![4],
                y: vec![4],
                cover: 50.0,
                hoop: RectColumnHoop {
                    dia: 10.0,
                    pitch: 100.0,
                    legs_x: 2,
                    legs_y: 2,
                },
            },
        }
    }

    fn src_shape() -> SectionShape {
        SectionShape::SrcColumnRect {
            b: 500.0,
            d: 700.0,
            rebar: RcRectColumnRebar {
                main_dia: 22.0,
                x: vec![4],
                y: vec![4],
                cover: 50.0,
                hoop: RectColumnHoop {
                    dia: 10.0,
                    pitch: 100.0,
                    legs_x: 2,
                    legs_y: 2,
                },
            },
            steel_height: 400.0,
            steel_width: 200.0,
            steel_web_thick: 9.0,
            steel_flange_thick: 12.0,
        }
    }

    /// 集中ばねの表示骨格の折れ点が、材料へ渡した k_rot・降伏モーメント・
    /// 降伏後勾配と一致する。
    #[test]
    fn concentrated_spring_backbone_matches_material() {
        let model = make_model(None, None);
        let beam = elem(ElementKind::Beam, [NodeId(0), NodeId(1)]);
        let basis = StrengthBasis::Nominal;
        let kind = AnalysisKind::Incremental;

        let view = build_hinge_view(&beam, &model, basis, kind, 0.0, 8, 24).expect("配筋は妥当");
        assert_eq!(view.model, AnalysisHingeModel::ConcentratedSpring);
        let points = view.backbone.expect("集中ばねは骨格を返す");
        assert_eq!(points.len(), 3, "標準型は原点・降伏点・降伏後端点");
        assert_eq!(points[0], [0.0, 0.0]);

        let (si, _sj, backbone) =
            build_flexural_springs(&beam, &model, HysteresisModel::Standard, basis);
        let (yield_moment, _) = yield_moment_and_axial(&beam, &model, basis);
        let theta_y = yield_moment / backbone.k_rot;
        assert!((points[1][0] - theta_y).abs() < 1e-12);
        assert!((points[1][1] - yield_moment).abs() < 1e-9);
        assert!((points[2][0] - 4.0 * theta_y).abs() < 1e-12);
        let expected_end = yield_moment + backbone.post_yield_stiffness * 3.0 * theta_y;
        assert!((points[2][1] - expected_end).abs() < 1e-9 * yield_moment);

        let (m_at_yield, k0) = si.probe(theta_y);
        assert!((k0 - backbone.k_rot).abs() < 1e-9 * backbone.k_rot);
        assert!((m_at_yield - yield_moment).abs() < 1e-6 * yield_moment);
        let (m_end, _) = si.probe(points[2][0]);
        assert!((m_end - points[2][1]).abs() < 1e-6 * yield_moment);
    }

    /// N-M 線形相関は `yield_moment_and_axial` と一致し、履歴材料では返さない。
    #[test]
    fn concentrated_spring_mn_linear_matches_yield_and_missing_for_history() {
        let basis = StrengthBasis::Nominal;
        let kind = AnalysisKind::Incremental;

        let model = make_model(None, None);
        let beam = elem(ElementKind::Beam, [NodeId(0), NodeId(1)]);
        let view = build_hinge_view(&beam, &model, basis, kind, 0.0, 8, 24).expect("配筋は妥当");
        let mn = view.mn_linear.expect("標準型は N-M 線形相関を返す");
        let (my0, n_allow) = yield_moment_and_axial(&beam, &model, basis);
        assert!((mn.my0 - my0).abs() < 1e-9);
        assert!((mn.n_allow - n_allow.max(1.0)).abs() < 1e-9);

        let mut rc_model = make_model(Some(rc_shape()), Some(24.0));
        rc_model.materials[0].category = MaterialCategory::Concrete;
        rc_model.materials.push(Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(1),
            name: "SD345".into(),
            category: MaterialCategory::Rebar,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(345.0),
        });
        rc_model.sections[0].rebar_material = Some(MaterialId(1));
        rc_model.set_member_hysteresis(ElemId(0), HysteresisModel::Takeda);
        let rc_beam = elem(ElementKind::Beam, [NodeId(0), NodeId(1)]);
        assert!(
            crate::factory::input_check::member_strength_issue(&rc_beam, &rc_model).is_none(),
            "正しい RC モデルは入力不備なし（早期 return 経路を通らない）"
        );
        let rc_view =
            build_hinge_view(&rc_beam, &rc_model, basis, kind, 0.0, 8, 24).expect("配筋は妥当");
        assert_eq!(rc_view.model, AnalysisHingeModel::ConcentratedSpring);
        assert!(
            rc_view.backbone.is_some(),
            "武田型（履歴材料）でも M-θ 骨格は返す"
        );
        assert!(
            rc_view.mn_linear.is_none(),
            "武田型（履歴材料）は N-M 線形相関を返さない"
        );
    }

    /// 断面未定義の集中ばねは骨格・N-M 線形相関を返さない（既定値で近似しない）。
    #[test]
    fn concentrated_spring_without_section_returns_no_backbone() {
        let model = make_model(None, None);
        let mut beam = elem(ElementKind::Beam, [NodeId(0), NodeId(1)]);
        beam.section = None;
        let view = build_hinge_view(
            &beam,
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
            0.0,
            8,
            24,
        )
        .expect("配筋は妥当");
        assert_eq!(view.model, AnalysisHingeModel::ConcentratedSpring);
        assert!(view.backbone.is_none());
        assert!(view.mn_linear.is_none());
    }

    /// 材料強度（fy・Fc）を解決できない集中ばねは骨格・N-M 線形相関を返さない。
    #[test]
    fn concentrated_spring_without_strength_returns_no_backbone() {
        let mut model = make_model(None, None);
        model.materials[0].fy = None;
        let beam = elem(ElementKind::Beam, [NodeId(0), NodeId(1)]);
        let view = build_hinge_view(
            &beam,
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
            0.0,
            8,
            24,
        )
        .expect("配筋は妥当");
        assert_eq!(view.model, AnalysisHingeModel::ConcentratedSpring);
        assert!(view.backbone.is_none());
        assert!(view.mn_linear.is_none());
    }

    /// 履歴則を変えると use_nm と折れ点が対応して変わる。
    #[test]
    fn backbone_changes_with_hysteresis_rule() {
        let basis = StrengthBasis::Nominal;
        let kind = AnalysisKind::Incremental;
        let mut model = make_model(None, None);
        let beam = elem(ElementKind::Beam, [NodeId(0), NodeId(1)]);
        model.elements.push(beam.clone());

        model.set_member_hysteresis(ElemId(0), HysteresisModel::Retrograde);
        let retro = build_hinge_view(&beam, &model, basis, kind, 0.0, 8, 24).expect("配筋は妥当");
        let (_ri, _rj, retro_backbone) =
            build_flexural_springs(&beam, &model, HysteresisModel::Retrograde, basis);
        assert!(retro.mn_linear.is_none());
        assert_eq!(retro.backbone.unwrap(), retro_backbone.points);
        assert_eq!(retro_backbone.points.len(), 4, "逆行型はトリリニア");

        model.set_member_hysteresis(ElemId(0), HysteresisModel::SteelBuckling);
        let buckle = build_hinge_view(&beam, &model, basis, kind, 0.0, 8, 24).expect("配筋は妥当");
        let (_bi, _bj, buckle_backbone) =
            build_flexural_springs(&beam, &model, HysteresisModel::SteelBuckling, basis);
        assert!(buckle.mn_linear.is_some());
        assert_eq!(buckle.backbone.unwrap(), buckle_backbone.points);
        assert_eq!(buckle_backbone.points.len(), 5, "座屈考慮型は 5 折れ点");
    }

    /// N-M 線形相関時は降伏点・降伏後端点が `moment_limit` で置換される。
    #[test]
    fn mn_limit_replaces_yield_points() {
        let model = make_model(None, None);
        let beam = elem(ElementKind::Beam, [NodeId(0), NodeId(1)]);
        let basis = StrengthBasis::Nominal;
        let kind = AnalysisKind::Incremental;

        let axial = 2.0e5;
        let view = build_hinge_view(&beam, &model, basis, kind, axial, 8, 24).expect("配筋は妥当");
        let mn = view.mn_linear.expect("N-M 線形相関");
        let points = view.backbone.expect("骨格");
        let expected = mn.moment_limit(axial);
        let (_i, _j, backbone) =
            build_flexural_springs(&beam, &model, HysteresisModel::Standard, basis);
        let theta_y = expected / backbone.k_rot;
        assert!((points[1][1] - expected).abs() < 1e-9 * expected.abs());
        assert!((points[1][0] - theta_y).abs() < 1e-12);
        assert!((points[2][0] - 4.0 * theta_y).abs() < 1e-12);
        let expected_end = expected + backbone.post_yield_stiffness * 3.0 * theta_y;
        assert!((points[2][1] - expected_end).abs() < 1e-9 * expected.abs());

        // 解析と同じ `set_yield` を通した材料の応答と一致する。
        let (_i2, mut sj, _b2) =
            build_flexural_springs(&beam, &model, HysteresisModel::Standard, basis);
        sj.set_yield(expected);
        for p in &points {
            let (m, _) = sj.probe(p[0]);
            assert!(
                (m - p[1]).abs() < 1e-6 * expected.abs(),
                "θ={} で材料応答 {} と骨格 {} が不一致",
                p[0],
                m,
                p[1]
            );
        }
    }

    /// バイリニア系（Bilinear／TsujiYamada）の降伏後枝は、`set_yield` を適用した
    /// 実際の材料の `probe` 応答と一致する。
    #[test]
    fn bilinear_and_tsuji_post_yield_matches_set_yield_material() {
        let basis = StrengthBasis::Nominal;
        let kind = AnalysisKind::Incremental;
        let axial = 1.5e5;

        for rule in [HysteresisModel::Standard, HysteresisModel::TsujiYamada] {
            let mut model = make_model(None, None);
            let beam = elem(ElementKind::Beam, [NodeId(0), NodeId(1)]);
            model.elements.push(beam.clone());
            model.set_member_hysteresis(ElemId(0), rule);

            let view =
                build_hinge_view(&beam, &model, basis, kind, axial, 8, 24).expect("配筋は妥当");
            let mn = view.mn_linear.expect("N-M 線形相関");
            let points = view.backbone.expect("骨格");
            assert_eq!(points.len(), 3, "{rule:?} は 3 折れ点");
            let expected = mn.moment_limit(axial);

            let (_i, mut sj, backbone) = build_flexural_springs(&beam, &model, rule, basis);
            sj.set_yield(expected);
            for p in &points {
                let (m, _) = sj.probe(p[0]);
                assert!(
                    (m - p[1]).abs() < 1e-6 * expected.abs(),
                    "{rule:?} θ={} で材料応答 {} と骨格 {} が不一致",
                    p[0],
                    m,
                    p[1]
                );
            }
            let theta_y = expected / backbone.k_rot;
            assert!((points[2][0] - 4.0 * theta_y).abs() < 1e-12);
            let expected_end = expected + backbone.post_yield_stiffness * 3.0 * theta_y;
            assert!((points[2][1] - expected_end).abs() < 1e-9 * expected.abs());
        }
    }

    /// 座屈考慮型は `set_yield` 後の材料応答（θ 節点は元の θy 基準）と一致する。
    #[test]
    fn steel_buckling_mn_points_match_set_yield_material() {
        let mut model = make_model(None, None);
        let beam = elem(ElementKind::Beam, [NodeId(0), NodeId(1)]);
        model.elements.push(beam.clone());
        model.set_member_hysteresis(ElemId(0), HysteresisModel::SteelBuckling);
        let basis = StrengthBasis::Nominal;
        let kind = AnalysisKind::Incremental;

        let axial = 1.0e5;
        let view = build_hinge_view(&beam, &model, basis, kind, axial, 8, 24).expect("配筋は妥当");
        let mn = view.mn_linear.expect("N-M 線形相関");
        let points = view.backbone.expect("骨格");
        let expected = mn.moment_limit(axial);
        assert_eq!(points.len(), 5);

        let (_i, mut sj, backbone) =
            build_flexural_springs(&beam, &model, HysteresisModel::SteelBuckling, basis);
        sj.set_yield(expected);
        for p in &points {
            let (m, _) = sj.probe(p[0]);
            assert!(
                (m - p[1]).abs() < 1e-6 * expected.abs(),
                "θ={} で材料応答 {} と骨格 {} が不一致",
                p[0],
                m,
                p[1]
            );
        }
        assert!(backbone.use_mn);
    }

    /// ファイバーの N-M 曲面は解析用 `FiberSection` のファイバを単一情報源とする。
    #[test]
    fn fiber_surface_uses_analysis_fiber_section() {
        let model = make_model(None, None);
        let col = elem(ElementKind::Fiber, [NodeId(0), NodeId(2)]);
        let basis = StrengthBasis::Nominal;
        let kind = AnalysisKind::Incremental;

        let lp = crate::factory::plastic_zone_length(&col, &model);
        let beam = crate::frame::fiber::FiberBeam::with_plastic_zone(&col, &model, lp, basis, kind);
        let analysis = &beam.gauss_points[0].section.fibers;

        let plastic = analysis_plastic_fibers(&col, &model, basis, kind, FIBER_NW, FIBER_ND)
            .expect("配筋は妥当")
            .expect("断面ありはファイバを返す");
        assert_eq!(plastic.len(), analysis.len());
        for (p, f) in plastic.iter().zip(analysis.iter()) {
            assert_eq!(p.y, f.y);
            assert_eq!(p.z, f.z);
            assert_eq!(p.area, f.area);
            let tag = match p.region {
                FiberRegion::Concrete => 0,
                FiberRegion::Rebar => 1,
                FiberRegion::Steel => 2,
            };
            assert_eq!(tag, f.material);
        }

        let view = build_hinge_view(&col, &model, basis, kind, 0.0, 8, 24).expect("配筋は妥当");
        assert_eq!(view.model, AnalysisHingeModel::Fiber);
        assert!(view.backbone.is_none());
        let surface = view.mn_surface.expect("ファイバーは N-M 曲面を返す");
        let nc: f64 = plastic.iter().map(|p| p.sigma_c * p.area).sum();
        let nt: f64 = plastic.iter().map(|p| p.sigma_t * p.area).sum();
        assert!((surface.n_comp - nc).abs() < 1e-9 * nc.abs().max(1.0));
        assert!((surface.n_tens - nt).abs() < 1e-9 * nt.abs().max(1.0));
    }

    /// MS の N-M 曲面は解析の `MS_NW × MS_ND` 格子を単一情報源とする。
    #[test]
    fn multi_spring_surface_uses_analysis_grid() {
        let model = make_model(None, None);
        let ms = elem(ElementKind::MultiSpring, [NodeId(0), NodeId(2)]);
        let basis = StrengthBasis::Nominal;
        let kind = AnalysisKind::Incremental;

        let plastic = analysis_plastic_fibers(&ms, &model, basis, kind, MS_NW, MS_ND)
            .expect("配筋は妥当")
            .expect("断面ありはファイバを返す");
        assert_eq!(plastic.len(), MS_NW * MS_ND);

        let element = MultiSpringElement::new(&ms, &model, basis, kind);
        let states = element
            .fiber_section_states()
            .expect("MS 要素はファイバー断面の状態を返す");
        assert_eq!(states[0].fibers.len(), MS_NW * MS_ND);

        let view = build_hinge_view(&ms, &model, basis, kind, 0.0, 8, 24).expect("配筋は妥当");
        assert_eq!(view.model, AnalysisHingeModel::MultiSpring);
        assert!(view.mn_surface.is_some());
    }

    /// 断面未定義では曲面を生成せず `None` を返す。
    #[test]
    fn undefined_section_returns_no_surface() {
        let model = make_model(None, None);
        let mut col = elem(ElementKind::Fiber, [NodeId(0), NodeId(2)]);
        col.section = None;
        let view = build_hinge_view(
            &col,
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
            0.0,
            8,
            24,
        )
        .expect("配筋は妥当");
        assert_eq!(view.model, AnalysisHingeModel::Fiber);
        assert!(view.mn_surface.is_none());
        assert!(view.backbone.is_none());
    }

    /// コンクリート系断面で Fc が未設定のとき、ファイバ材料の生成で panic せず
    /// 曲面を返さない（表示 API の「近似は返さない」契約）。
    #[test]
    fn concrete_shape_without_fc_returns_no_surface() {
        let mut model = make_model(Some(rc_column_shape()), None);
        model.materials[0].category = MaterialCategory::Concrete;
        let col = elem(ElementKind::Fiber, [NodeId(0), NodeId(2)]);
        let issue = crate::factory::input_check::member_strength_issue(&col, &model)
            .expect("Fc 未設定は不備と判定される");
        assert!(issue.contains("Fc"), "Fc 未設定の経路を通る: {issue}");
        assert!(analysis_plastic_fibers(
            &col,
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
            FIBER_NW,
            FIBER_ND,
        )
        .expect("配筋は妥当")
        .is_none());
        let view = build_hinge_view(
            &col,
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
            0.0,
            8,
            24,
        )
        .expect("配筋は妥当");
        assert_eq!(view.model, AnalysisHingeModel::Fiber);
        assert!(view.mn_surface.is_none());
    }

    /// 耐震壁とその側柱（鉛直な `Beam`）を持つ壁展開モデル相当。返り値は側柱要素。
    ///
    /// 壁の解析要素は入力の正である `Model` には保存されず解析時にのみ生成される
    /// （`squid-n-load` の `expand_wall_elements`）。`squid-n-element` は
    /// `squid-n-load` に依存できないため、ここでは展開後相当のモデルを直接組み立てる。
    /// 生モデル（壁要素なし）との分類差は `squid-n-app` の回帰テストで固定する。
    fn wall_side_column_model() -> (Model, ElementData) {
        let node = |id: NodeId, coord: [f64; 3]| Node {
            id,
            coord,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        };
        let mut model = Model {
            nodes: vec![
                node(NodeId(0), [0.0, 0.0, 0.0]),
                node(NodeId(1), [4000.0, 0.0, 0.0]),
                node(NodeId(2), [4000.0, 0.0, 3000.0]),
                node(NodeId(3), [0.0, 0.0, 3000.0]),
            ],
            sections: vec![SectionShape::RcWall {
                thickness: 150.0,
                ps: 0.0025,
            }
            .to_section(SectionId(0), "W150".into())],
            materials: vec![Material {
                strength_factor: None,
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "FC24".into(),
                category: MaterialCategory::Concrete,
                young: 23000.0,
                poisson: 0.2,
                density: 2.4e-9,
                shear: None,
                fc: Some(24.0),
                fy: None,
            }],
            ..Default::default()
        };
        let wall = ElementData {
            id: ElemId(0),
            kind: ElementKind::Wall,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        };
        model.elements.push(wall.clone());
        crate::wall::add_surrounding_frame(&mut model, &wall);
        let mut column = elem(ElementKind::Beam, [NodeId(0), NodeId(3)]);
        column.id = ElemId(9);
        model.elements.push(column.clone());
        (model, column)
    }

    /// [`resolves_to_concentrated_spring`] の判定が [`build_hinge_view`] の分岐
    /// （`Beam` に材端集中ばねビューを返すか）と一致する。
    #[test]
    fn resolves_to_concentrated_spring_matches_build_hinge_view() {
        let basis = StrengthBasis::Nominal;
        let kind = AnalysisKind::Incremental;
        let check = |data: &ElementData, model: &Model| {
            let view = build_hinge_view(data, model, basis, kind, 0.0, 8, 24).expect("配筋は妥当");
            assert_eq!(
                resolves_to_concentrated_spring(data, model),
                view.model == AnalysisHingeModel::ConcentratedSpring,
                "要素種別・レジーム {:?} でヘルパーとビューの分岐が不一致",
                data.kind
            );
        };

        let model = make_model(None, None);
        check(&elem(ElementKind::Beam, [NodeId(0), NodeId(1)]), &model);
        check(&elem(ElementKind::Beam, [NodeId(0), NodeId(2)]), &model);
        check(&elem(ElementKind::Fiber, [NodeId(0), NodeId(2)]), &model);
        check(
            &elem(ElementKind::MultiSpring, [NodeId(0), NodeId(2)]),
            &model,
        );

        let (wall_model, column) = wall_side_column_model();
        check(&column, &wall_model);
        assert!(
            !resolves_to_concentrated_spring(&column, &wall_model),
            "壁側柱は集中ばね分岐を取らない"
        );
    }

    /// 鋼材断面で fy が未設定のとき、ファイバ材料の生成で panic せず曲面を
    /// 返さない。
    #[test]
    fn steel_shape_without_fy_returns_no_surface() {
        let shape = SectionShape::SteelBox {
            height: 400.0,
            width: 200.0,
            thick: 9.0,
            corner_r: 0.0,
        };
        let mut model = make_model(Some(shape), None);
        model.materials[0].fy = None;
        let col = elem(ElementKind::Fiber, [NodeId(0), NodeId(2)]);
        assert!(analysis_plastic_fibers(
            &col,
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
            FIBER_NW,
            FIBER_ND,
        )
        .expect("配筋は妥当")
        .is_none());
        let view = build_hinge_view(
            &col,
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
            0.0,
            8,
            24,
        )
        .expect("配筋は妥当");
        assert_eq!(view.model, AnalysisHingeModel::Fiber);
        assert!(view.mn_surface.is_none());
    }

    /// SRC 矩形で内蔵鉄骨材料が未割当のとき、入力チェックは要素材料 fy で
    /// 不備なしと判定するが、ファイバ生成の鋼材領域は降伏点を解決できない。
    /// 表示 API は panic せず曲面を返さない。
    #[test]
    fn src_shape_without_steel_material_returns_no_surface() {
        let mut model = make_model(Some(src_shape()), Some(24.0));
        model.materials[0].category = MaterialCategory::Concrete;
        model.materials.push(Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(1),
            name: "SD345".into(),
            category: MaterialCategory::Rebar,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: Some(345.0),
        });
        model.sections[0].rebar_material = Some(MaterialId(1));
        let col = elem(ElementKind::Fiber, [NodeId(0), NodeId(2)]);

        assert!(
            crate::factory::input_check::member_strength_issue(&col, &model).is_none(),
            "内蔵鉄骨材料が未割当でも要素材料 fy で入力不備なしと判定される"
        );
        assert!(
            resolve_fiber_yield(&model, &col).steel.is_none(),
            "ファイバ生成が要求する鋼材降伏点は解決できない"
        );
        assert!(analysis_plastic_fibers(
            &col,
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
            FIBER_NW,
            FIBER_ND,
        )
        .expect("配筋は妥当")
        .is_none());

        let view = build_hinge_view(
            &col,
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
            0.0,
            8,
            24,
        )
        .expect("配筋は妥当");
        assert_eq!(view.model, AnalysisHingeModel::Fiber);
        assert!(view.mn_surface.is_none());
    }
}
