//! 解析で実際に使用した非線形特性（M-θ 骨格 / N-M 相関 / N-M 曲面）の
//! 読み取り専用ビュー。
//!
//! GUI（`squid-n-app`）が解析と同じ計算経路・同じパラメータ解決でヒンジの
//! 背景曲線を描くための API。表示側に解析の式・既定値を複製しない。
//! 断面・材料の不足などでモデルを再現できない場合は該当フィールドを `None` とし、
//! 近似は返さない。
//!
//! 単位: 長さ [mm]、力 [N]、モーメント [N·mm]、回転角 [rad]。

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
use crate::frame::fiber::{build_gauss_fiber_pair, fiber_strength_params, FIBER_ND, FIBER_NW};
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
) -> HingeView {
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
        _ => HingeView::none(AnalysisHingeModel::Other),
    }
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
) -> HingeView {
    if crate::wall::side_column::wall_side_column_release(data, model).is_some() {
        return HingeView::none(AnalysisHingeModel::Other);
    }
    match resolve_force_regime(data, model) {
        ResolvedRegime::ConcentratedSpring => {
            if super::input_check::member_strength_issue(data, model).is_some() {
                return HingeView::none(AnalysisHingeModel::ConcentratedSpring);
            }
            let rule = resolve_member_hysteresis(data, model, kind);
            let (_i, _j, backbone) = build_flexural_springs(data, model, rule, basis);
            let (my0, n_allow) = yield_moment_and_axial(data, model, basis);
            let mn = backbone.use_mn.then(|| MnInteraction::new(my0, n_allow));
            let points = backbone.points_with_mn(mn.as_ref(), axial_force_compression_positive);
            HingeView {
                model: AnalysisHingeModel::ConcentratedSpring,
                backbone: Some(points),
                mn_linear: mn,
                mn_surface: None,
            }
        }
        ResolvedRegime::Fiber => fiber_view(data, model, basis, kind, n_alpha, n_beta),
    }
}

/// ファイバー要素（解析の `nw=12`, `nd=20`）の N-M 曲面。
fn fiber_view(
    data: &ElementData,
    model: &Model,
    basis: StrengthBasis,
    kind: AnalysisKind,
    n_alpha: usize,
    n_beta: usize,
) -> HingeView {
    surface_view(
        AnalysisHingeModel::Fiber,
        YieldModelKind::MultiFiber,
        analysis_plastic_fibers(data, model, basis, kind, FIBER_NW, FIBER_ND),
        n_alpha,
        n_beta,
    )
}

/// マルチスプリング要素（解析の `MS_NW × MS_ND`）の N-M 曲面。
fn multi_spring_view(
    data: &ElementData,
    model: &Model,
    basis: StrengthBasis,
    kind: AnalysisKind,
    n_alpha: usize,
    n_beta: usize,
) -> HingeView {
    surface_view(
        AnalysisHingeModel::MultiSpring,
        YieldModelKind::MultiSpring,
        analysis_plastic_fibers(data, model, basis, kind, MS_NW, MS_ND),
        n_alpha,
        n_beta,
    )
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

/// 解析が実際に生成する端部ファイバ断面を、解析と同じ解像度・強度解決で
/// `PlasticFiber` 群へ変換する（N-M 曲面の単一情報源）。
///
/// 断面未定義、または材料強度（Fc・fy）を解決できない場合は `None` を返す。
/// 後者はファイバ材料の生成（[`build_gauss_fiber_pair`]）が panic する条件
/// であり、表示のために解析入力を変更せず `None` で「近似しない」を表す。
pub(crate) fn analysis_plastic_fibers(
    data: &ElementData,
    model: &Model,
    basis: StrengthBasis,
    kind: AnalysisKind,
    nw: usize,
    nd: usize,
) -> Option<Vec<PlasticFiber>> {
    let sec = data
        .section
        .and_then(|sid| model.sections.get(sid.index()))?;
    if super::input_check::member_strength_issue(data, model).is_some() {
        return None;
    }
    let strength = fiber_strength_params(data, model, basis);
    let [(section, _mats), _] =
        build_gauss_fiber_pair(data, model, basis, kind, sec.width, sec.depth, nw, nd);
    Some(
        section
            .fibers
            .iter()
            .map(|f| to_plastic_fiber(f, &strength))
            .collect(),
    )
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
    use crate::frame::multi_spring::MultiSpringElement;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
    use squid_n_core::model::{
        Constraint, EndCondition, ForceRegime, HysteresisModel, LocalAxis, Material,
        MaterialCategory, Node, Section,
    };
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

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
        SectionShape::RcRect {
            b: 400.0,
            d: 700.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 4,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 4,
                    dia: 22.0,
                    layers: 1,
                },
                cover: 50.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
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

        let view = build_hinge_view(&beam, &model, basis, kind, 0.0, 8, 24);
        assert_eq!(view.model, AnalysisHingeModel::ConcentratedSpring);
        let points = view.backbone.expect("集中ばねは骨格を返す");
        assert_eq!(points.len(), 3, "標準型は原点・降伏点・降伏後端点");
        assert_eq!(points[0], [0.0, 0.0]);

        let (si, _sj, backbone) =
            build_flexural_springs(&beam, &model, HysteresisModel::Standard, basis);
        let theta_y = backbone.yield_moment / backbone.k_rot;
        assert!((points[1][0] - theta_y).abs() < 1e-12);
        assert!((points[1][1] - backbone.yield_moment).abs() < 1e-9);
        assert!((points[2][0] - 4.0 * theta_y).abs() < 1e-12);
        let expected_end = backbone.yield_moment + backbone.post_yield_stiffness * 3.0 * theta_y;
        assert!((points[2][1] - expected_end).abs() < 1e-9 * backbone.yield_moment);

        let (m_at_yield, k0) = si.probe(theta_y);
        assert!((k0 - backbone.k_rot).abs() < 1e-9 * backbone.k_rot);
        assert!((m_at_yield - backbone.yield_moment).abs() < 1e-6 * backbone.yield_moment);
        let (m_end, _) = si.probe(points[2][0]);
        assert!((m_end - points[2][1]).abs() < 1e-6 * backbone.yield_moment);
    }

    /// N-M 線形相関は `yield_moment_and_axial` と一致し、履歴材料では返さない。
    #[test]
    fn concentrated_spring_mn_linear_matches_yield_and_missing_for_history() {
        let basis = StrengthBasis::Nominal;
        let kind = AnalysisKind::Incremental;

        let model = make_model(None, None);
        let beam = elem(ElementKind::Beam, [NodeId(0), NodeId(1)]);
        let view = build_hinge_view(&beam, &model, basis, kind, 0.0, 8, 24);
        let mn = view.mn_linear.expect("標準型は N-M 線形相関を返す");
        let (my0, n_allow) = yield_moment_and_axial(&beam, &model, basis);
        assert!((mn.my0 - my0).abs() < 1e-9);
        assert!((mn.n_allow - n_allow.max(1.0)).abs() < 1e-9);

        let mut rc_model = make_model(Some(rc_shape()), Some(24.0));
        rc_model.materials[0].fy = Some(345.0);
        let rc_beam = elem(ElementKind::Beam, [NodeId(0), NodeId(1)]);
        let rc_view = build_hinge_view(&rc_beam, &rc_model, basis, kind, 0.0, 8, 24);
        assert_eq!(rc_view.model, AnalysisHingeModel::ConcentratedSpring);
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
        );
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
        );
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
        let retro = build_hinge_view(&beam, &model, basis, kind, 0.0, 8, 24);
        let (_ri, _rj, retro_backbone) =
            build_flexural_springs(&beam, &model, HysteresisModel::Retrograde, basis);
        assert!(retro.mn_linear.is_none());
        assert_eq!(retro.backbone.unwrap(), retro_backbone.points);
        assert_eq!(retro_backbone.points.len(), 4, "逆行型はトリリニア");

        model.set_member_hysteresis(ElemId(0), HysteresisModel::SteelBuckling);
        let buckle = build_hinge_view(&beam, &model, basis, kind, 0.0, 8, 24);
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
        let view = build_hinge_view(&beam, &model, basis, kind, axial, 8, 24);
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

            let view = build_hinge_view(&beam, &model, basis, kind, axial, 8, 24);
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
        let view = build_hinge_view(&beam, &model, basis, kind, axial, 8, 24);
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

        let view = build_hinge_view(&col, &model, basis, kind, 0.0, 8, 24);
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
            .expect("断面ありはファイバを返す");
        assert_eq!(plastic.len(), MS_NW * MS_ND);

        let element = MultiSpringElement::new(&ms, &model, basis, kind);
        assert_eq!(
            element.inner.gauss_points[0].section.fibers.len(),
            MS_NW * MS_ND
        );

        let view = build_hinge_view(&ms, &model, basis, kind, 0.0, 8, 24);
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
        );
        assert_eq!(view.model, AnalysisHingeModel::Fiber);
        assert!(view.mn_surface.is_none());
        assert!(view.backbone.is_none());
    }

    /// コンクリート系断面で Fc が未設定のとき、ファイバ材料の生成で panic せず
    /// 曲面を返さない（表示 API の「近似は返さない」契約）。
    #[test]
    fn concrete_shape_without_fc_returns_no_surface() {
        let mut model = make_model(Some(rc_shape()), None);
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
        .is_none());
        let view = build_hinge_view(
            &col,
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
            0.0,
            8,
            24,
        );
        assert_eq!(view.model, AnalysisHingeModel::Fiber);
        assert!(view.mn_surface.is_none());
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
        .is_none());
        let view = build_hinge_view(
            &col,
            &model,
            StrengthBasis::Nominal,
            AnalysisKind::Incremental,
            0.0,
            8,
            24,
        );
        assert_eq!(view.model, AnalysisHingeModel::Fiber);
        assert!(view.mn_surface.is_none());
    }
}
