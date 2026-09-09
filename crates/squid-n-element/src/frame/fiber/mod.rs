use crate::behavior::{Ctx, DuctilityProbe, ElementBehavior, LocalMat, LocalVec, MassOption};
use smallvec::SmallVec;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::NodeId;
use squid_n_core::model::{AnalysisKind, HysteresisModel};
use squid_n_core::section_shape::SectionShape;
use squid_n_material::uniaxial::{MenegottoPinto, UniaxialMaterial};
use squid_n_section::fiber::{Fiber, FiberSection};
use std::any::Any;

/// 塑性化域長 Lp [mm] を部材長 `l` [mm] に対して有効な範囲へクランプする。
/// 各端 45% を上限、1e-6·L を下限とする。
pub fn clamp_plastic_zone(lp: f64, l: f64) -> f64 {
    lp.clamp(1.0e-6 * l, 0.45 * l)
}

/// 鋼材・鉄筋のファイバ材料を生成する。
///
/// # Panics
///
/// fy 未設定・0 以下で panic する。
pub(crate) fn steel_fiber_material(e: f64, fy: Option<f64>) -> Box<dyn UniaxialMaterial> {
    let Some(fy) = fy.filter(|fy| *fy > 0.0) else {
        panic!(
            "ファイバー断面の鋼材・主筋に降伏強度 fy が未設定です。\
             解析前に factory::ensure_nonlinear_input で入力チェックを行ってください"
        );
    };
    Box::new(MenegottoPinto::new(e, fy))
}

/// コンクリートのファイバ材料を生成する。
///
/// `rule` は逆行型・原点指向型・Karsan–Jirsa 型のいずれかを渡す。
///
/// # Panics
///
/// Fc 未設定、曲げバネ用履歴則の混入で panic する。
pub(crate) fn concrete_fiber_material(
    fc: Option<f64>,
    rule: HysteresisModel,
) -> Box<dyn UniaxialMaterial> {
    let Some(fc) = fc else {
        panic!(
            "ファイバー断面のコンクリート領域に Fc が未設定です。\
             解析前に factory::ensure_nonlinear_input で入力チェックを行ってください"
        );
    };
    match rule {
        HysteresisModel::KarsanJirsa => {
            if fc <= 60.0 {
                let ec = squid_n_material::newrc::NewRcEnvelope::new(fc).ec;
                Box::new(squid_n_material::ConcreteCyclic::newrc(
                    fc,
                    0.01,
                    2.0,
                    ec / 10.0,
                ))
            } else {
                let e0 = 2.0 * fc / 0.002;
                Box::new(squid_n_material::ConcreteCyclic::kent_park(
                    fc,
                    0.002,
                    0.0,
                    0.0035,
                    2.0,
                    e0 / 10.0,
                ))
            }
        }
        HysteresisModel::Retrograde | HysteresisModel::OriginOriented => {
            if fc <= 60.0 {
                let mut m = squid_n_material::ConcreteNewRc::new(fc, 2.0);
                m.set_concrete_hysteresis(rule == HysteresisModel::OriginOriented);
                Box::new(m)
            } else {
                Box::new(squid_n_material::uniaxial::Concrete::new(fc, 2.0))
            }
        }
        other => panic!(
            "コンクリートのファイバ材料の除荷則として解釈できません: {other:?}\
            （逆行型・原点指向型・Karsan-Jirsa型のみ。\
             factory::resolve_fiber_concrete_hysteresis で解決した値を渡してください）"
        ),
    }
}

/// ファイバ断面の各領域の降伏点 [N/mm²]。
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct FiberYield {
    /// 主材料の fy（形状を持たない断面の格子に用いる）。
    pub main: Option<f64>,
    /// 主筋の σy（RC・SRC 断面）。
    pub rebar: Option<f64>,
    /// 鋼材領域の fy（SRC は内蔵鉄骨、それ以外は主材料）。
    pub steel: Option<f64>,
}

pub(crate) fn resolve_fiber_yield(
    model: &squid_n_core::model::Model,
    data: &squid_n_core::model::ElementData,
) -> FiberYield {
    let main = model.element_material(data).and_then(|m| m.fy);
    let rebar =
        squid_n_core::material_grade::rebar_yield_strength(model.element_rebar_material(data));
    let steel = match model.element_section(data).and_then(|s| s.shape.as_ref()) {
        Some(SectionShape::SrcRect {
            steel_flange_thick, ..
        }) => {
            let thick = *steel_flange_thick;
            model.element_steel_material(data).and_then(|m| {
                squid_n_core::material_grade::steel_f_value_prefix(&m.name, thick).or(m.fy)
            })
        }
        _ => main,
    };
    FiberYield { main, rebar, steel }
}

pub(crate) fn resolve_steel_fiber_fy(
    shape: Option<&SectionShape>,
    steel_mat: Option<&squid_n_core::model::Material>,
    mat_fy: Option<f64>,
) -> Option<f64> {
    match shape {
        Some(SectionShape::SrcRect {
            steel_flange_thick, ..
        }) => steel_mat
            .and_then(|m| {
                squid_n_core::material_grade::steel_f_value_prefix(&m.name, *steel_flange_thick)
                    .or(m.fy)
            })
            .or(mat_fy),
        _ => mat_fy,
    }
}

/// 同じ諸元のファイバー断面をガウス点 2 点分つくる。
/// 各ガウス点は独立した断面と材料インスタンスを持つ。
/// 断面・材料が未割当のときはゼロ剛性。
#[allow(clippy::too_many_arguments)]
fn build_gauss_fiber_pair(
    data: &squid_n_core::model::ElementData,
    model: &squid_n_core::model::Model,
    basis: crate::factory::StrengthBasis,
    kind: AnalysisKind,
    width: f64,
    depth: f64,
    nw: usize,
    nd: usize,
) -> [(FiberSection, Vec<Box<dyn UniaxialMaterial>>); 2] {
    let sec = model.element_section(data);
    let mat_ref = model.element_material(data);
    let e = mat_ref.map(|m| m.young).unwrap_or(0.0);
    let shape = sec.and_then(|s| s.shape.as_ref());
    let fc = mat_ref.and_then(|m| m.fc);
    let yield_ = resolve_fiber_yield(model, data);
    let steel_factor = basis.steel_factor(mat_ref);
    let rebar_factor = basis.rebar_factor(mat_ref);
    let concrete_rule = crate::factory::resolve_fiber_concrete_hysteresis(data, model, kind);
    let build = || {
        build_gauss_fibers(
            width,
            depth,
            nw,
            nd,
            shape,
            fc,
            e,
            yield_,
            steel_factor,
            rebar_factor,
            concrete_rule,
        )
    };
    [build(), build()]
}

/// ガウス点のファイバー断面と材料を構築する。
/// 形状がない場合は width×depth の中実格子とする。
///
/// ファイバの材料区分タグ（`Fiber::material`）:
/// 0=コンクリート、1=主筋、2=鋼材。
#[allow(clippy::too_many_arguments)]
pub(crate) fn build_gauss_fibers(
    width: f64,
    depth: f64,
    nw: usize,
    nd: usize,
    shape: Option<&SectionShape>,
    fc: Option<f64>,
    e: f64,
    yield_: FiberYield,
    steel_factor: f64,
    rebar_factor: f64,
    concrete_rule: HysteresisModel,
) -> (FiberSection, Vec<Box<dyn UniaxialMaterial>>) {
    let mut result = shape
        .filter(|s| !matches!(s, SectionShape::RcWall { .. }))
        .map(|s| build_shape_fibers(s, fc, e, yield_, steel_factor, rebar_factor, concrete_rule));

    let (mut fibers, mats) = match result.take() {
        Some(r) => r,
        None => {
            let base: Box<dyn UniaxialMaterial> = if fc.is_some() {
                concrete_fiber_material(fc, concrete_rule)
            } else {
                steel_fiber_material(e, yield_.main.map(|fy| fy * steel_factor))
            };
            let tag = if fc.is_some() { 0 } else { 2 };
            let grid = squid_n_section::fiber::rect_fiber_section(width, depth, nw, nd, tag);
            let fibers = grid.fibers;
            let mats: Vec<Box<dyn UniaxialMaterial>> =
                (0..fibers.len()).map(|_| base.clone_box()).collect();
            (fibers, mats)
        }
    };

    for f in &mut fibers {
        let (y, z) = (f.y, f.z);
        f.y = z;
        f.z = -y;
    }
    (FiberSection { fibers }, mats)
}

/// 断面形状に応じたファイバ配置と材料の構築。
/// 主筋の降伏点は断面の主筋材質から解決し、なければ部材材料の fy を用いる。
#[allow(clippy::too_many_arguments)]
fn build_shape_fibers(
    shape: &SectionShape,
    fc: Option<f64>,
    e: f64,
    yield_: FiberYield,
    steel_factor: f64,
    rebar_factor: f64,
    concrete_rule: HysteresisModel,
) -> (Vec<Fiber>, Vec<Box<dyn UniaxialMaterial>>) {
    use squid_n_section::mn_surface::{
        max_dimension, plastic_fibers_at, AnnulusRes, FiberRegion, StrengthParams,
    };

    let rebar_fy = yield_.rebar.or(yield_.main);
    let steel_fy = yield_.steel;
    let strength = StrengthParams {
        steel_fy: steel_fy.unwrap_or(235.0) * steel_factor,
        rebar_fy: rebar_fy.unwrap_or(345.0) * rebar_factor,
        concrete_fc: fc.unwrap_or(24.0),
        steel_e: e,
    };
    let target = (max_dimension(shape) / 16.0).max(1.0);
    let ring = AnnulusRes {
        n_theta: 24,
        n_r_thin: 2,
        n_r_solid: 8,
    };
    let placed = plastic_fibers_at(shape, &strength, target, ring);

    let mut concrete: Option<Box<dyn UniaxialMaterial>> = None;
    let mut steel: Option<Box<dyn UniaxialMaterial>> = None;
    let mut rebar: Option<Box<dyn UniaxialMaterial>> = None;

    let mut fibers = Vec::with_capacity(placed.len());
    let mut mats: Vec<Box<dyn UniaxialMaterial>> = Vec::with_capacity(placed.len());
    for f in &placed {
        let (tag, mat) = match f.region {
            FiberRegion::Concrete => {
                let template =
                    concrete.get_or_insert_with(|| concrete_fiber_material(fc, concrete_rule));
                (0usize, template.clone_box())
            }
            FiberRegion::Rebar => {
                let template = rebar.get_or_insert_with(|| {
                    steel_fiber_material(205000.0, rebar_fy.map(|fy| fy * rebar_factor))
                });
                (1usize, template.clone_box())
            }
            FiberRegion::Steel => {
                let template = steel.get_or_insert_with(|| {
                    steel_fiber_material(e, steel_fy.map(|fy| fy * steel_factor))
                });
                (2usize, template.clone_box())
            }
        };
        fibers.push(Fiber {
            y: f.y,
            z: f.z,
            area: f.area,
            material: tag,
        });
        mats.push(mat);
    }
    (fibers, mats)
}

/// 材端解放で内部自由度へ分離した要素端回転。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EndRelease {
    /// 可撓端系ローカル自由度 index（3,4,5 = i 端、9,10,11 = j 端）。
    pub dof: usize,
    /// 回転ばね剛性 [N·mm/rad]（ピンは 0）。
    pub spring: f64,
}

/// 端条件とねじれ解放から、解放する回転自由度を決める。
/// ねじり剛性がない部材（J≤0）の rx は解放しない。
fn resolve_end_releases(
    end_cond: &[squid_n_core::model::EndCondition; 2],
    torsion_release: [bool; 2],
    has_torsion: bool,
) -> SmallVec<[EndRelease; 6]> {
    use squid_n_core::model::EndCondition;
    const ROT_DOFS: [(usize, usize); 6] = [(3, 0), (4, 0), (5, 0), (9, 1), (10, 1), (11, 1)];
    let mut out = SmallVec::new();
    for &(dof, end) in ROT_DOFS.iter() {
        let is_torsion = dof == 3 || dof == 9;
        let spring = match end_cond[end] {
            EndCondition::Fixed if is_torsion && torsion_release[end] => 0.0,
            EndCondition::Fixed => continue,
            EndCondition::Pinned => 0.0,
            EndCondition::SemiRigid { k_theta } => k_theta,
        };
        if is_torsion && !has_torsion {
            continue;
        }
        out.push(EndRelease { dof, spring });
    }
    out
}

/// スナップショットの型。ヒンジ無しモデルはヒンジ状態が空。
pub type FiberBeamSnapshot = (
    [f64; 12],
    [f64; 12],
    Vec<Vec<Box<dyn UniaxialMaterial>>>,
    Vec<f64>,
    Vec<f64>,
    Vec<f64>,
    Vec<f64>,
);

#[derive(serde::Serialize, serde::Deserialize)]
struct FiberBeamCheckpoint {
    trial_disp: [f64; 12],
    committed_disp: [f64; 12],
    gauss_points: Vec<Vec<Vec<u8>>>,
    /// 材端解放の内部自由度（要素端回転）。
    trial_int: Vec<f64>,
    committed_int: Vec<f64>,
    /// 塑性増分ヒンジ状態（κ4 + θb4 = 8 値。ヒンジ無しモデルは空）。
    trial_hinge: Vec<f64>,
    committed_hinge: Vec<f64>,
}

/// 読み込み互換用。
#[derive(serde::Deserialize)]
struct FiberBeamCheckpointV2 {
    trial_disp: [f64; 12],
    committed_disp: [f64; 12],
    gauss_points: Vec<Vec<Vec<u8>>>,
    trial_int: Vec<f64>,
    committed_int: Vec<f64>,
}

/// 読み込み互換用。
#[derive(serde::Deserialize)]
struct FiberBeamCheckpointLegacy {
    trial_disp: [f64; 12],
    committed_disp: [f64; 12],
    gauss_points: Vec<Vec<Vec<u8>>>,
}

/// 塑性増分ヒンジの定義と状態。
#[derive(Clone)]
pub struct HingeState {
    /// 塑性化域長 Lp [mm]。
    pub lp: f64,
    pub k_el: LocalMat,
    /// 公称弾性 D 対角 [EA, E·Iy_elem, E·Iz_elem]。
    d_nom: [f64; 3],
    /// 端部ファイバー断面の弾性曲げ剛性 [端][軸]（軸 0=κy, 1=κz）。
    sec_ei: [[f64; 2]; 2],
    /// ヒンジが有効な端（端条件 Fixed のみ）。
    active: [bool; 2],
    /// ヒンジ断面曲率 [i_κy, i_κz, j_κy, j_κz]（トライアル/確定）。
    pub trial_kappa: [f64; 4],
    pub committed_kappa: [f64; 4],
    /// 可撓端回転（梁側）[slot4, slot5, slot10, slot11]（トライアル/確定）。
    trial_thb: [f64; 4],
    committed_thb: [f64; 4],
}

/// ヒンジ自由度のスロット表 [端][軸]（軸 0=κy→ry、1=κz→rz）。
const HINGE_SLOTS: [[usize; 2]; 2] = [[4, 5], [10, 11]];
/// ヒンジ回転の向き（i 端 +1・j 端 −1。B 行列の自端回転係数の符号）。
const HINGE_SIGN: [f64; 2] = [1.0, -1.0];

pub struct GaussPoint {
    pub xi: f64,
    pub weight: f64,
    pub section: FiberSection,
    pub mats: Vec<Box<dyn UniaxialMaterial>>,
    pub trial_stress: Vec<f64>,
    pub trial_et: Vec<f64>,
    b: [[f64; 12]; 3],
    /// 断面応答のキャッシュ。`trial_stress`/`trial_et` を書き換えた直後に更新すること。
    cached_force: [f64; 3],
    cached_stiff: [[f64; 3]; 3],
}

impl GaussPoint {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        xi: f64,
        weight: f64,
        section: FiberSection,
        mut mats: Vec<Box<dyn UniaxialMaterial>>,
        l: f64,
        phi_y: f64,
        phi_z: f64,
    ) -> Self {
        let n = section.fibers.len();
        let trial_et: Vec<f64> = mats.iter_mut().map(|m| m.trial(0.0).1).collect();
        let b = FiberBeam::compute_b_matrix(xi, l, phi_y, phi_z);
        let mut gp = GaussPoint {
            xi,
            weight,
            section,
            mats,
            trial_stress: vec![0.0; n],
            trial_et,
            b,
            cached_force: [0.0; 3],
            cached_stiff: [[0.0; 3]; 3],
        };
        gp.refresh_response();
        gp
    }

    /// `trial_stress`/`trial_et` を書き換えた直後に呼ぶこと。
    fn refresh_response(&mut self) {
        let (stress, et) = (&self.trial_stress, &self.trial_et);
        let (f, s) =
            squid_n_section::fiber::integrate_fibers(&self.section, |i, _| (stress[i], et[i]));
        self.cached_force = [f.n, f.my, f.mz];
        self.cached_stiff = s.d;
    }
}

/// ファイバー梁要素。
pub struct FiberBeam {
    /// 節点間長 L [mm]。
    pub length: f64,
    /// 材端の剛域長 λi, λj [mm]。
    pub rigid_i: f64,
    pub rigid_j: f64,
    /// 可撓長 L' = `length` − λi − λj [mm]。
    pub flex_length: f64,
    pub nodes: [NodeId; 2],
    pub gauss_points: Vec<GaussPoint>,
    pub density: f64,
    /// ねじり定数 J [mm⁴]。
    pub torsion_j: f64,
    /// せん断弾性係数 G [N/mm²]。
    pub g: f64,
    /// せん断変形係数 φy。GAs ≤ 0 なら 0。
    pub phi_y: f64,
    /// せん断変形係数 φz。
    pub phi_z: f64,
    pub k_shear: LocalMat,
    /// 要素ローカル系→グローバル系の回転。内部状態はローカル系で保持する。
    pub axis: crate::transform::LocalFrame,
    /// 塑性増分ヒンジ。None = 全長ファイバー積分モデル。
    pub hinge: Option<HingeState>,
    /// 材端解放で分離した要素端回転。空なら全端剛接。
    pub releases: SmallVec<[EndRelease; 6]>,
    /// 内部自由度の現在値（`releases` と同順）。
    pub trial_int: SmallVec<[f64; 6]>,
    pub committed_int: SmallVec<[f64; 6]>,
    /// 内力を評価する危険断面位置（正規化座標 \[0,1\]）。
    pub eval_sections: Vec<f64>,
    pub committed_disp: [f64; 12],
    pub trial_disp: [f64; 12],
}

impl FiberBeam {
    pub fn new(
        data: &squid_n_core::model::ElementData,
        model: &squid_n_core::model::Model,
        basis: crate::factory::StrengthBasis,
        kind: AnalysisKind,
    ) -> Self {
        let n0 = &model.nodes[data.nodes[0].index()];
        let n1 = &model.nodes[data.nodes[1].index()];
        let length = squid_n_core::geom::vec3::dist(n0.coord, n1.coord);
        let (rigid_i, rigid_j) = crate::frame::rigid_arm::resolve_lengths(
            data.rigid_zone.rigid_length_i(),
            data.rigid_zone.rigid_length_j(),
            length,
        );
        let flex_length = length - rigid_i - rigid_j;

        let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
        let mat_ref = model.element_material(data);
        let density = mat_ref.map(|m| m.density).unwrap_or(0.0);
        let e = mat_ref.map(|m| m.young).unwrap_or(0.0);
        let g = mat_ref.map(|m| m.shear_modulus()).unwrap_or(0.0);
        let width = sec.map(|s| s.width).unwrap_or(0.0);
        let depth = sec.map(|s| s.depth).unwrap_or(0.0);
        let torsion_j = sec.map(|s| s.j).unwrap_or(0.0);

        let sec_iy = sec.map(|s| s.iy).unwrap_or(0.0);
        let sec_iz = sec.map(|s| s.iz).unwrap_or(0.0);
        let sec_as_y = sec.map(|s| s.as_y).unwrap_or(0.0);
        let sec_as_z = sec.map(|s| s.as_z).unwrap_or(0.0);
        let phi_of = |ei: f64, gas: f64| {
            if gas > 0.0 && ei > 0.0 && flex_length > 0.0 {
                12.0 * ei / (gas * flex_length * flex_length)
            } else {
                0.0
            }
        };
        let phi_y = phi_of(e * sec_iy, g * sec_as_z);
        let phi_z = phi_of(e * sec_iz, g * sec_as_y);
        let k_shear =
            Self::compute_shear_stiffness(flex_length, phi_y, phi_z, g * sec_as_z, g * sec_as_y);

        let nw = 12;
        let nd = 20;
        let [(sec_a, mats_a), (sec_b, mats_b)] =
            build_gauss_fiber_pair(data, model, basis, kind, width, depth, nw, nd);
        let gauss_points = vec![
            GaussPoint::new(
                -0.5773502691896257,
                1.0,
                sec_a,
                mats_a,
                flex_length,
                phi_y,
                phi_z,
            ),
            GaussPoint::new(
                0.5773502691896257,
                1.0,
                sec_b,
                mats_b,
                flex_length,
                phi_y,
                phi_z,
            ),
        ];

        let axis = crate::transform::LocalFrame::from_nodes(
            n0.coord,
            n1.coord,
            data.local_axis.ref_vector,
        );

        let releases = resolve_end_releases(
            &data.end_cond,
            [
                crate::frame::beam::i_end_torsion_release(data, model),
                false,
            ],
            torsion_j > 0.0 && g > 0.0,
        );
        let trial_int = SmallVec::from_elem(0.0, releases.len());

        FiberBeam {
            length,
            rigid_i,
            rigid_j,
            flex_length,
            releases,
            committed_int: trial_int.clone(),
            trial_int,
            nodes: [data.nodes[0], data.nodes[1]],
            gauss_points,
            density,
            torsion_j,
            g,
            phi_y,
            phi_z,
            k_shear,
            axis,
            hinge: None,
            eval_sections: crate::frame::beam::eval_sections_of(data, model, length),
            committed_disp: [0.0; 12],
            trial_disp: [0.0; 12],
        }
    }

    fn compute_shear_stiffness(l: f64, phi_y: f64, phi_z: f64, gas_y: f64, gas_z: f64) -> LocalMat {
        let mut k = LocalMat::zeros(12);
        if l <= 0.0 {
            return k;
        }
        let planes: [([(usize, f64); 4], f64); 2] = [
            (
                [
                    (1, 2.0 * phi_y / (2.0 * (1.0 + phi_y) * l)),
                    (7, -2.0 * phi_y / (2.0 * (1.0 + phi_y) * l)),
                    (5, phi_y / (2.0 * (1.0 + phi_y))),
                    (11, phi_y / (2.0 * (1.0 + phi_y))),
                ],
                gas_y,
            ),
            (
                [
                    (2, -2.0 * phi_z / (2.0 * (1.0 + phi_z) * l)),
                    (8, 2.0 * phi_z / (2.0 * (1.0 + phi_z) * l)),
                    (4, phi_z / (2.0 * (1.0 + phi_z))),
                    (10, phi_z / (2.0 * (1.0 + phi_z))),
                ],
                gas_z,
            ),
        ];
        for (bg, gas) in planes {
            if gas <= 0.0 {
                continue;
            }
            for &(i, bi) in &bg {
                for &(j, bj) in &bg {
                    let v = gas * l * bi * bj;
                    if v != 0.0 {
                        k.set(i, j, k.get(i, j) + v);
                    }
                }
            }
        }
        k
    }

    /// 諸元は要素座標系で与える。両端剛接としてヒンジを両端有効にする。剛域なし。
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn from_raw_parts(
        nodes: [NodeId; 2],
        length: f64,
        axis: crate::transform::LocalFrame,
        density: f64,
        e: f64,
        g: f64,
        area: f64,
        iy_elem: f64,
        iz_elem: f64,
        as_y_elem: f64,
        as_z_elem: f64,
        torsion_j: f64,
        lp: f64,
        sections: [(FiberSection, Vec<Box<dyn UniaxialMaterial>>); 2],
    ) -> Self {
        let flex_length = length;
        let phi_of = |ei: f64, gas: f64| {
            if gas > 0.0 && ei > 0.0 && flex_length > 0.0 {
                12.0 * ei / (gas * flex_length * flex_length)
            } else {
                0.0
            }
        };
        let phi_y = phi_of(e * iz_elem, g * as_y_elem);
        let phi_z = phi_of(e * iy_elem, g * as_z_elem);
        let k_shear =
            Self::compute_shear_stiffness(flex_length, phi_y, phi_z, g * as_y_elem, g * as_z_elem);
        let lp = clamp_plastic_zone(lp, flex_length);
        let [(sec_a, mats_a), (sec_b, mats_b)] = sections;
        let w_end = 2.0 * lp / flex_length;
        let gauss_points = vec![
            GaussPoint::new(-1.0, w_end, sec_a, mats_a, flex_length, phi_y, phi_z),
            GaussPoint::new(1.0, w_end, sec_b, mats_b, flex_length, phi_y, phi_z),
        ];
        let mut fb = FiberBeam {
            length,
            rigid_i: 0.0,
            rigid_j: 0.0,
            flex_length,
            releases: SmallVec::new(),
            committed_int: SmallVec::new(),
            trial_int: SmallVec::new(),
            nodes,
            gauss_points,
            density,
            torsion_j,
            g,
            phi_y,
            phi_z,
            k_shear,
            axis,
            hinge: None,
            eval_sections: vec![0.0, 0.5, 1.0],
            committed_disp: [0.0; 12],
            trial_disp: [0.0; 12],
        };
        let d_nom = [e * area, e * iy_elem, e * iz_elem];
        let mut k_el = LocalMat::zeros(12);
        for sgn in [-1.0_f64, 1.0] {
            let xi = sgn / 3.0_f64.sqrt();
            let w_phys = flex_length / 2.0;
            let b = Self::compute_b_matrix(xi, flex_length, fb.phi_y, fb.phi_z);
            for i in 0..12 {
                for j in 0..12 {
                    let mut val = 0.0;
                    for (p, dp) in d_nom.iter().enumerate() {
                        val += b[p][i] * dp * b[p][j];
                    }
                    if val != 0.0 {
                        k_el.set(i, j, k_el.get(i, j) + val * w_phys);
                    }
                }
            }
        }
        for i in 0..12 {
            for j in 0..12 {
                let v = fb.k_shear.get(i, j);
                if v != 0.0 {
                    k_el.set(i, j, k_el.get(i, j) + v);
                }
            }
        }
        if let Some(kt) = fb.torsion_stiffness() {
            k_el.set(3, 3, k_el.get(3, 3) + kt);
            k_el.set(9, 9, k_el.get(9, 9) + kt);
            k_el.set(3, 9, k_el.get(3, 9) - kt);
            k_el.set(9, 3, k_el.get(9, 3) - kt);
        }
        let sec_ei = std::array::from_fn(|end| {
            let (_, d) = Self::section_response_from_cache(&fb.gauss_points[end]);
            [d[1][1], d[2][2]]
        });
        fb.hinge = Some(HingeState {
            lp,
            k_el,
            d_nom,
            sec_ei,
            active: [true, true],
            trial_kappa: [0.0; 4],
            committed_kappa: [0.0; 4],
            trial_thb: [0.0; 4],
            committed_thb: [0.0; 4],
        });
        fb
    }

    pub fn with_plastic_zone(
        data: &squid_n_core::model::ElementData,
        model: &squid_n_core::model::Model,
        lp: f64,
        basis: crate::factory::StrengthBasis,
        kind: AnalysisKind,
    ) -> Self {
        Self::build_plastic_zone(data, model, lp, 12, 20, basis, kind)
    }

    /// `nw × nd` は端部断面のファイバ分割数。
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn build_plastic_zone(
        data: &squid_n_core::model::ElementData,
        model: &squid_n_core::model::Model,
        lp: f64,
        nw: usize,
        nd: usize,
        basis: crate::factory::StrengthBasis,
        kind: AnalysisKind,
    ) -> Self {
        let mut fb = Self::new(data, model, basis, kind);
        let l = fb.flex_length;
        if l <= 0.0 {
            return fb;
        }
        let lp = clamp_plastic_zone(lp, l);

        let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
        let mat_ref = model.element_material(data);
        let e = mat_ref.map(|m| m.young).unwrap_or(0.0);
        let width = sec.map(|s| s.width).unwrap_or(0.0);
        let depth = sec.map(|s| s.depth).unwrap_or(0.0);
        let area = sec.map(|s| s.area).unwrap_or(width * depth);
        let iy = sec.map(|s| s.iz).unwrap_or(0.0);
        let iz = sec.map(|s| s.iy).unwrap_or(0.0);

        let w_end = 2.0 * lp / l;
        let [(sec_a, mats_a), (sec_b, mats_b)] =
            build_gauss_fiber_pair(data, model, basis, kind, width, depth, nw, nd);
        fb.gauss_points = vec![
            GaussPoint::new(-1.0, w_end, sec_a, mats_a, l, fb.phi_y, fb.phi_z),
            GaussPoint::new(1.0, w_end, sec_b, mats_b, l, fb.phi_y, fb.phi_z),
        ];

        let d_nom = [e * area, e * iy, e * iz];
        let mut k_el = LocalMat::zeros(12);
        for sgn in [-1.0_f64, 1.0] {
            let xi = sgn / 3.0_f64.sqrt();
            let w_phys = l / 2.0;
            let b = Self::compute_b_matrix(xi, l, fb.phi_y, fb.phi_z);
            for i in 0..12 {
                for j in 0..12 {
                    let mut val = 0.0;
                    for (p, dp) in d_nom.iter().enumerate() {
                        val += b[p][i] * dp * b[p][j];
                    }
                    if val != 0.0 {
                        k_el.set(i, j, k_el.get(i, j) + val * w_phys);
                    }
                }
            }
        }
        for i in 0..12 {
            for j in 0..12 {
                let v = fb.k_shear.get(i, j);
                if v != 0.0 {
                    k_el.set(i, j, k_el.get(i, j) + v);
                }
            }
        }
        if let Some(kt) = fb.torsion_stiffness() {
            k_el.set(3, 3, k_el.get(3, 3) + kt);
            k_el.set(9, 9, k_el.get(9, 9) + kt);
            k_el.set(3, 9, k_el.get(3, 9) - kt);
            k_el.set(9, 3, k_el.get(9, 3) - kt);
        }

        let sec_ei = std::array::from_fn(|end| {
            let (_, d) = Self::section_response_from_cache(&fb.gauss_points[end]);
            [d[1][1], d[2][2]]
        });
        let active = std::array::from_fn(|end| {
            matches!(data.end_cond[end], squid_n_core::model::EndCondition::Fixed)
        });
        fb.hinge = Some(HingeState {
            lp,
            k_el,
            d_nom,
            sec_ei,
            active,
            trial_kappa: [0.0; 4],
            committed_kappa: [0.0; 4],
            trial_thb: [0.0; 4],
            committed_thb: [0.0; 4],
        });
        fb
    }

    pub fn flex_disp(&self) -> [f64; 12] {
        crate::frame::rigid_arm::to_flex_disp(&self.trial_disp, self.rigid_i, self.rigid_j)
    }

    fn elem_disp(&self, u_flex: &[f64; 12]) -> [f64; 12] {
        let mut u = *u_flex;
        for (k, rel) in self.releases.iter().enumerate() {
            u[rel.dof] = self.trial_int[k];
        }
        u
    }

    fn update_section_trial(&mut self, u_elem: &[f64; 12]) {
        let l = self.flex_length;
        if l <= 0.0 {
            return;
        }
        for gp in &mut self.gauss_points {
            let b = gp.b;
            let eps0 = b[0][0] * u_elem[0] + b[0][6] * u_elem[6];
            let ky = b[1][2] * u_elem[2]
                + b[1][4] * u_elem[4]
                + b[1][8] * u_elem[8]
                + b[1][10] * u_elem[10];
            let kz = b[2][1] * u_elem[1]
                + b[2][5] * u_elem[5]
                + b[2][7] * u_elem[7]
                + b[2][11] * u_elem[11];
            for (i, fiber) in gp.section.fibers.iter().enumerate() {
                let eps = eps0 - kz * fiber.y + ky * fiber.z;
                let (sigma, et) = gp.mats[i].trial(eps);
                gp.trial_stress[i] = sigma;
                gp.trial_et[i] = et;
            }
            gp.refresh_response();
        }
    }

    fn elem_tangent(&self) -> LocalMat {
        let mut k = LocalMat::zeros(12);
        let l = self.flex_length;
        if l <= 0.0 {
            return k;
        }
        if let Some(h) = &self.hinge {
            return self.hinge_tangent(h);
        }
        let half = l / 2.0;

        for gp in &self.gauss_points {
            let (_, d) = Self::section_response_from_cache(gp);
            let w = gp.weight * half;
            let b = gp.b;

            for i in 0..12 {
                for p in 0..3 {
                    let bpi = b[p][i];
                    if bpi == 0.0 {
                        continue;
                    }
                    for j in 0..12 {
                        let mut val = 0.0;
                        for q in 0..3 {
                            val += d[p][q] * b[q][j];
                        }
                        if val != 0.0 {
                            let old = k.get(i, j);
                            k.set(i, j, old + bpi * val * w);
                        }
                    }
                }
            }
        }

        for i in 0..12 {
            for j in 0..12 {
                let v = self.k_shear.get(i, j);
                if v != 0.0 {
                    k.set(i, j, k.get(i, j) + v);
                }
            }
        }

        if let Some(kt) = self.torsion_stiffness() {
            k.set(3, 3, k.get(3, 3) + kt);
            k.set(9, 9, k.get(9, 9) + kt);
            k.set(3, 9, k.get(3, 9) - kt);
            k.set(9, 3, k.get(9, 3) - kt);
        }
        k
    }

    /// ねじり剛性 GJ/L。J≤0 では None。
    fn torsion_stiffness(&self) -> Option<f64> {
        (self.torsion_j > 0.0 && self.length > 0.0).then(|| self.g * self.torsion_j / self.length)
    }

    /// `u_elem` と整合させるには先に [`Self::update_section_trial`] を呼ぶこと。
    fn elem_internal_force(&self, u_elem: &[f64; 12]) -> [f64; 12] {
        let mut f = [0.0_f64; 12];
        let l = self.flex_length;
        if l <= 0.0 {
            return f;
        }
        if let Some(h) = &self.hinge {
            return Self::hinge_internal_force(h, u_elem);
        }
        let half = l / 2.0;

        for gp in &self.gauss_points {
            let (force, _) = Self::section_response_from_cache(gp);
            let w = gp.weight * half;
            let b = gp.b;
            for (i, fi) in f.iter_mut().enumerate() {
                *fi += (b[0][i] * force[0] + b[1][i] * force[1] + b[2][i] * force[2]) * w;
            }
        }

        for i in 0..12 {
            let mut si = 0.0;
            for j in 0..12 {
                si += self.k_shear.get(i, j) * u_elem[j];
            }
            f[i] += si;
        }

        if let Some(kt) = self.torsion_stiffness() {
            let drx = u_elem[3] - u_elem[9];
            f[3] += kt * drx;
            f[9] -= kt * drx;
        }
        f
    }

    fn solve_internal_dofs(&mut self) {
        /// 内部釣合いの最大反復数（弾性域は 1 回、降伏を跨いでも数回で収まる）。
        const MAX_ITER: usize = 20;
        let u_flex = self.flex_disp();
        if self.releases.is_empty() {
            let u_elem = self.elem_disp(&u_flex);
            self.update_trial_state(&u_elem);
            return;
        }
        let n = self.releases.len();
        for _ in 0..MAX_ITER {
            let u_elem = self.elem_disp(&u_flex);
            self.update_trial_state(&u_elem);
            let f_elem = self.elem_internal_force(&u_elem);

            let mut r = [0.0_f64; 6];
            for (k, rel) in self.releases.iter().enumerate() {
                r[k] = f_elem[rel.dof] + rel.spring * (u_elem[rel.dof] - u_flex[rel.dof]);
            }
            let scale = [3usize, 4, 5, 9, 10, 11]
                .iter()
                .map(|&i| f_elem[i].abs())
                .fold(1.0_f64, f64::max);
            if r[..n].iter().all(|v| v.abs() <= 1e-10 * scale) {
                return;
            }

            let k_elem = self.elem_tangent();
            let mut kbb = [0.0_f64; 36];
            for (a, ra) in self.releases.iter().enumerate() {
                for (b, rb) in self.releases.iter().enumerate() {
                    kbb[a * n + b] = k_elem.get(ra.dof, rb.dof);
                }
                kbb[a * n + a] += ra.spring;
            }
            let Some(kbb_inv) = crate::linalg::invert_small(&kbb[..n * n], n) else {
                break;
            };
            let mut du = [0.0_f64; 6];
            for (a, dua) in du[..n].iter_mut().enumerate() {
                let mut s = 0.0;
                for (b, rb) in r[..n].iter().enumerate() {
                    s += kbb_inv[a * n + b] * rb;
                }
                *dua = -s;
            }
            if du[..n].iter().any(|v| !v.is_finite()) {
                break;
            }
            for (k, d) in du[..n].iter().enumerate() {
                self.trial_int[k] += d;
            }
        }
        let u_elem = self.elem_disp(&u_flex);
        self.update_trial_state(&u_elem);
    }

    fn update_trial_state(&mut self, u_elem: &[f64; 12]) {
        if self.hinge.is_some() {
            self.solve_hinges(u_elem);
        } else {
            self.update_section_trial(u_elem);
        }
    }

    fn condense_releases(&self, k_elem: &LocalMat) -> LocalMat {
        let releases: SmallVec<[(usize, f64); 6]> =
            self.releases.iter().map(|r| (r.dof, r.spring)).collect();
        crate::frame::prismatic::condense_end_releases(k_elem, &releases)
    }

    fn section_response_from_cache(gp: &GaussPoint) -> ([f64; 3], [[f64; 3]; 3]) {
        (gp.cached_force, gp.cached_stiff)
    }

    /// ひずみ－変位行列（行 0: 軸ひずみ、行 1: κy、行 2: κz）。
    fn compute_b_matrix(xi: f64, l: f64, phi_y: f64, phi_z: f64) -> [[f64; 12]; 3] {
        let inv_l = 1.0 / l;
        let inv_l2 = 1.0 / (l * l);
        let mut b = [[0.0; 12]; 3];
        b[0][0] = -inv_l;
        b[0][6] = inv_l;
        let cz = 1.0 / (1.0 + phi_z);
        b[1][2] = 6.0 * xi * inv_l2 * cz;
        b[1][4] = (1.0 - 3.0 * xi + phi_z) * inv_l * cz;
        b[1][8] = -6.0 * xi * inv_l2 * cz;
        b[1][10] = -(1.0 + 3.0 * xi + phi_z) * inv_l * cz;
        let cy = 1.0 / (1.0 + phi_y);
        b[2][1] = -6.0 * xi * inv_l2 * cy;
        b[2][5] = (1.0 - 3.0 * xi + phi_y) * inv_l * cy;
        b[2][7] = 6.0 * xi * inv_l2 * cy;
        b[2][11] = -(1.0 + 3.0 * xi + phi_y) * inv_l * cy;
        b
    }

    /// ヒンジの有効自由度 (端, 軸) の一覧（軸 0=κy, 1=κz）。
    fn hinge_dofs(h: &HingeState) -> SmallVec<[(usize, usize); 4]> {
        let mut dofs = SmallVec::new();
        for end in 0..2 {
            if !h.active[end] {
                continue;
            }
            for axis in 0..2 {
                if h.sec_ei[end][axis] > 0.0 && h.d_nom[1 + axis] > 0.0 {
                    dofs.push((end, axis));
                }
            }
        }
        dofs
    }

    fn hinge_uhat_from(dofs: &[(usize, usize)], thb: &[f64; 4], u_elem: &[f64; 12]) -> [f64; 12] {
        let mut u = *u_elem;
        for &(end, axis) in dofs {
            u[HINGE_SLOTS[end][axis]] = thb[end * 2 + axis];
        }
        u
    }

    fn update_hinge_section_trial(&mut self, end: usize, eps0: f64, ky: f64, kz: f64) {
        let gp = &mut self.gauss_points[end];
        for (i, fiber) in gp.section.fibers.iter().enumerate() {
            let eps = eps0 - kz * fiber.y + ky * fiber.z;
            let (sigma, et) = gp.mats[i].trial(eps);
            gp.trial_stress[i] = sigma;
            gp.trial_et[i] = et;
        }
        gp.refresh_response();
    }

    fn solve_hinges(&mut self, u_elem: &[f64; 12]) {
        const MAX_ITER: usize = 40;
        let l = self.flex_length;
        let (lp, d_nom, sec_ei, dofs, mut kappa, mut thb) = {
            let Some(h) = self.hinge.as_ref() else {
                return;
            };
            (
                h.lp,
                h.d_nom,
                h.sec_ei,
                Self::hinge_dofs(h),
                h.trial_kappa,
                h.trial_thb,
            )
        };
        let n = dofs.len();
        let eps0 = (u_elem[6] - u_elem[0]) / l;
        if n == 0 {
            for end in 0..2 {
                let (ky, kz) = (kappa[end * 2], kappa[end * 2 + 1]);
                self.update_hinge_section_trial(end, eps0, ky, kz);
            }
            return;
        }
        let b_end = [self.gauss_points[0].b, self.gauss_points[1].b];

        for _ in 0..MAX_ITER {
            let mut m = [[0.0_f64; 2]; 2];
            let mut dm = [[[0.0_f64; 2]; 2]; 2];
            for end in 0..2 {
                let (ky, kz) = (kappa[end * 2], kappa[end * 2 + 1]);
                self.update_hinge_section_trial(end, eps0, ky, kz);
                let (force, d) = Self::section_response_from_cache(&self.gauss_points[end]);
                m[end] = [force[1], force[2]];
                dm[end] = [[d[1][1], d[1][2]], [d[2][1], d[2][2]]];
            }
            for &(end, axis) in dofs.iter() {
                let gamma = HINGE_SIGN[end]
                    * lp
                    * (kappa[end * 2 + axis] - m[end][axis] / sec_ei[end][axis]);
                thb[end * 2 + axis] = u_elem[HINGE_SLOTS[end][axis]] - gamma;
            }
            let uh = Self::hinge_uhat_from(&dofs, &thb, u_elem);

            let mut r = [0.0_f64; 4];
            let mut scale = 1.0_f64;
            for (p, &(end, axis)) in dofs.iter().enumerate() {
                let brow = &b_end[end][1 + axis];
                let kb: f64 = brow.iter().zip(uh.iter()).map(|(b, u)| b * u).sum();
                let m_b = d_nom[1 + axis] * kb;
                r[p] = m_b - m[end][axis];
                scale = scale.max(m_b.abs()).max(m[end][axis].abs());
            }
            if r[..n].iter().all(|v| v.abs() <= 1e-9 * scale) {
                break;
            }

            let mut jac = [0.0_f64; 16];
            for (p, &(ep, ap)) in dofs.iter().enumerate() {
                let brow = &b_end[ep][1 + ap];
                for (q, &(eq, aq)) in dofs.iter().enumerate() {
                    let mut v = 0.0;
                    for &(epp, app) in dofs.iter() {
                        if epp != eq {
                            continue;
                        }
                        let g = -HINGE_SIGN[epp]
                            * lp
                            * ((if app == aq { 1.0 } else { 0.0 })
                                - dm[epp][app][aq] / sec_ei[epp][app]);
                        v += d_nom[1 + ap] * brow[HINGE_SLOTS[epp][app]] * g;
                    }
                    if ep == eq {
                        v -= dm[ep][ap][aq];
                    }
                    jac[p * n + q] = v;
                }
            }
            let Some(jinv) = crate::linalg::invert_small(&jac[..n * n], n) else {
                break;
            };
            let mut dk = [0.0_f64; 4];
            for (p, dkp) in dk.iter_mut().take(n).enumerate() {
                let mut s = 0.0;
                for q in 0..n {
                    s += jinv[p * n + q] * r[q];
                }
                *dkp = -s;
            }
            if dk[..n].iter().any(|v| !v.is_finite()) {
                break;
            }
            for (p, &(end, axis)) in dofs.iter().enumerate() {
                kappa[end * 2 + axis] += dk[p];
            }
        }

        for end in 0..2 {
            let (ky, kz) = (kappa[end * 2], kappa[end * 2 + 1]);
            self.update_hinge_section_trial(end, eps0, ky, kz);
        }
        for &(end, axis) in dofs.iter() {
            let (force, _) = Self::section_response_from_cache(&self.gauss_points[end]);
            let gamma = HINGE_SIGN[end]
                * lp
                * (kappa[end * 2 + axis] - force[1 + axis] / sec_ei[end][axis]);
            thb[end * 2 + axis] = u_elem[HINGE_SLOTS[end][axis]] - gamma;
        }
        let h = self.hinge.as_mut().unwrap();
        h.trial_kappa = kappa;
        h.trial_thb = thb;
    }

    fn hinge_internal_force(h: &HingeState, u_elem: &[f64; 12]) -> [f64; 12] {
        let dofs = Self::hinge_dofs(h);
        let uh = Self::hinge_uhat_from(&dofs, &h.trial_thb, u_elem);
        let mut f = [0.0_f64; 12];
        for (i, fi) in f.iter_mut().enumerate() {
            let mut s = 0.0;
            for (j, &u) in uh.iter().enumerate() {
                s += h.k_el.get(i, j) * u;
            }
            *fi = s;
        }
        f
    }

    fn hinge_tangent(&self, h: &HingeState) -> LocalMat {
        let dofs = Self::hinge_dofs(h);
        let n = dofs.len();
        if n == 0 {
            return LocalMat {
                n: 12,
                data: h.k_el.data.clone(),
            };
        }
        let b_end = [self.gauss_points[0].b, self.gauss_points[1].b];
        let mut dm = [[[0.0_f64; 2]; 2]; 2];
        for end in 0..2 {
            let (_, d) = Self::section_response_from_cache(&self.gauss_points[end]);
            dm[end] = [[d[1][1], d[1][2]], [d[2][1], d[2][2]]];
        }
        let mut g = [0.0_f64; 16];
        for (p, &(ep, ap)) in dofs.iter().enumerate() {
            for (q, &(eq, aq)) in dofs.iter().enumerate() {
                if ep != eq {
                    continue;
                }
                g[p * n + q] = -HINGE_SIGN[ep]
                    * h.lp
                    * ((if ap == aq { 1.0 } else { 0.0 }) - dm[ep][ap][aq] / h.sec_ei[ep][ap]);
            }
        }
        let mut jac = [0.0_f64; 16];
        for (p, &(ep, ap)) in dofs.iter().enumerate() {
            let brow = &b_end[ep][1 + ap];
            for q in 0..n {
                let mut v = 0.0;
                for (pp, &(epp, app)) in dofs.iter().enumerate() {
                    v += h.d_nom[1 + ap] * brow[HINGE_SLOTS[epp][app]] * g[pp * n + q];
                }
                let (eq, aq) = dofs[q];
                if ep == eq {
                    v -= dm[ep][ap][aq];
                }
                jac[p * n + q] = v;
            }
        }
        let Some(jinv) = crate::linalg::invert_small(&jac[..n * n], n) else {
            return LocalMat {
                n: 12,
                data: h.k_el.data.clone(),
            };
        };
        let mut fx = [0.0_f64; 48];
        for q in 0..n {
            for (pp, &(epp, app)) in dofs.iter().enumerate() {
                let gv = g[pp * n + q];
                if gv == 0.0 {
                    continue;
                }
                let slot = HINGE_SLOTS[epp][app];
                for i in 0..12 {
                    fx[i * n + q] += h.k_el.get(i, slot) * gv;
                }
            }
        }
        let mut k = LocalMat {
            n: 12,
            data: h.k_el.data.clone(),
        };
        for (p, &(ep, ap)) in dofs.iter().enumerate() {
            let brow = &b_end[ep][1 + ap];
            for i in 0..12 {
                let mut c = 0.0;
                for q in 0..n {
                    c += fx[i * n + q] * jinv[q * n + p];
                }
                if c == 0.0 {
                    continue;
                }
                for j in 0..12 {
                    let v = c * h.d_nom[1 + ap] * brow[j];
                    if v != 0.0 {
                        k.set(i, j, k.get(i, j) - v);
                    }
                }
            }
        }
        for i in 0..12 {
            for j in (i + 1)..12 {
                let avg = 0.5 * (k.get(i, j) + k.get(j, i));
                k.set(i, j, avg);
                k.set(j, i, avg);
            }
        }
        k
    }
}

impl ElementBehavior for FiberBeam {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        crate::behavior::node_global_dofs(&self.nodes, dof)
    }

    fn tangent_stiffness(&self, _ctx: &Ctx) -> LocalMat {
        if self.flex_length <= 0.0 {
            return LocalMat::zeros(12);
        }
        let k_elem = self.elem_tangent();
        let k_end = self.condense_releases(&k_elem);
        let k_node =
            crate::frame::rigid_arm::transform_stiffness(&k_end, self.rigid_i, self.rigid_j);
        self.axis.to_global(&k_node)
    }

    fn internal_force(&self, _ctx: &Ctx) -> LocalVec {
        if self.flex_length <= 0.0 {
            return LocalVec {
                data: SmallVec::from_elem(0.0, 12),
            };
        }
        let u_flex = self.flex_disp();
        let u_elem = self.elem_disp(&u_flex);
        let f_elem = self.elem_internal_force(&u_elem);

        let mut f_flex = f_elem;
        for rel in &self.releases {
            f_flex[rel.dof] = rel.spring * (u_flex[rel.dof] - u_elem[rel.dof]);
        }

        let f_node = crate::frame::rigid_arm::to_node_force(&f_flex, self.rigid_i, self.rigid_j);
        let f_global = self.axis.rotate_to_global(&f_node);
        LocalVec {
            data: SmallVec::from_slice(&f_global),
        }
    }

    fn state_member_forces(&self, ctx: &Ctx) -> Option<crate::frame::beam::MemberForces> {
        let f_global = self.internal_force(ctx);
        let arr: [f64; 12] = std::array::from_fn(|i| f_global.data[i]);
        let f_local = self.axis.rotate_to_local(&arr);
        Some(crate::frame::beam::member_forces_from_end_forces(
            &f_local,
            self.length,
            &self.eval_sections,
        ))
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        let du_global: [f64; 12] = std::array::from_fn(|i| du.data[i]);
        let du_local = self.axis.rotate_to_local(&du_global);
        for i in 0..12 {
            self.trial_disp[i] += du_local[i];
        }
        if self.flex_length <= 0.0 {
            return;
        }
        self.solve_internal_dofs();
        if commit {
            for gp in &mut self.gauss_points {
                for mat in &mut gp.mats {
                    mat.commit();
                }
            }
            self.committed_disp = self.trial_disp;
            self.committed_int = self.trial_int.clone();
            if let Some(h) = &mut self.hinge {
                h.committed_kappa = h.trial_kappa;
                h.committed_thb = h.trial_thb;
            }
        }
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        let total_area: f64 = self
            .gauss_points
            .first()
            .map(|gp| gp.section.fibers.iter().map(|f| f.area).sum())
            .unwrap_or(0.0);
        let total_mass = self.density * total_area * self.length;
        match opt {
            MassOption::Lumped => crate::frame::prismatic::lumped_mass(total_mass),
            MassOption::Consistent => {
                let mm = crate::frame::prismatic::consistent_mass(total_mass, self.length, 0.0);
                self.axis.to_global(&mm)
            }
        }
    }

    fn geometric_stiffness(&self, n: f64) -> LocalMat {
        let kg_node = crate::frame::prismatic::geometric_stiffness(
            n,
            self.flex_length,
            self.rigid_i,
            self.rigid_j,
        );
        self.axis.to_global(&kg_node)
    }

    fn snapshot_state(&self) -> Box<dyn Any> {
        let gauss_data: Vec<Vec<Box<dyn UniaxialMaterial>>> = self
            .gauss_points
            .iter()
            .map(|gp| gp.mats.iter().map(|m| m.clone_box()).collect())
            .collect();
        let (trial_hinge, committed_hinge) = match &self.hinge {
            Some(h) => (
                h.trial_kappa
                    .iter()
                    .chain(h.trial_thb.iter())
                    .copied()
                    .collect(),
                h.committed_kappa
                    .iter()
                    .chain(h.committed_thb.iter())
                    .copied()
                    .collect(),
            ),
            None => (Vec::new(), Vec::new()),
        };
        Box::new((
            self.trial_disp,
            self.committed_disp,
            gauss_data,
            self.trial_int.to_vec(),
            self.committed_int.to_vec(),
            trial_hinge,
            committed_hinge,
        ))
    }

    fn restore_state(&mut self, state: &dyn Any) {
        let (trial, committed, mats_data, trial_int, committed_int, th, ch) =
            crate::behavior::downcast_snapshot::<FiberBeamSnapshot>("FiberBeam", state);
        self.trial_disp = *trial;
        self.committed_disp = *committed;
        for (gp, gp_mats) in self.gauss_points.iter_mut().zip(mats_data) {
            for (mat, new_mat) in gp.mats.iter_mut().zip(gp_mats) {
                *mat = new_mat.clone_box();
            }
        }
        self.trial_int = SmallVec::from_slice(trial_int);
        self.committed_int = SmallVec::from_slice(committed_int);
        if let Some(h) = &mut self.hinge {
            if th.len() == 8 && ch.len() == 8 {
                h.trial_kappa.copy_from_slice(&th[..4]);
                h.trial_thb.copy_from_slice(&th[4..]);
                h.committed_kappa.copy_from_slice(&ch[..4]);
                h.committed_thb.copy_from_slice(&ch[4..]);
            }
        }
    }

    fn commit_state(&mut self) {
        for gp in &mut self.gauss_points {
            for mat in &mut gp.mats {
                mat.commit();
            }
        }
        self.committed_disp = self.trial_disp;
        self.committed_int = self.trial_int.clone();
        if let Some(h) = &mut self.hinge {
            h.committed_kappa = h.trial_kappa;
            h.committed_thb = h.trial_thb;
        }
    }

    fn revert_state(&mut self) {
        for gp in &mut self.gauss_points {
            for mat in &mut gp.mats {
                mat.revert();
            }
        }
        self.trial_disp = self.committed_disp;
        self.trial_int = self.committed_int.clone();
        if let Some(h) = &mut self.hinge {
            h.trial_kappa = h.committed_kappa;
            h.trial_thb = h.committed_thb;
        }
    }

    fn serialize_checkpoint(&self) -> Vec<u8> {
        let gauss_points: Vec<Vec<Vec<u8>>> = self
            .gauss_points
            .iter()
            .map(|gp| {
                gp.mats
                    .iter()
                    .map(|m| m.serialize_state())
                    .collect::<Vec<_>>()
            })
            .collect();
        let (trial_hinge, committed_hinge) = match &self.hinge {
            Some(h) => (
                h.trial_kappa
                    .iter()
                    .chain(h.trial_thb.iter())
                    .copied()
                    .collect(),
                h.committed_kappa
                    .iter()
                    .chain(h.committed_thb.iter())
                    .copied()
                    .collect(),
            ),
            None => (Vec::new(), Vec::new()),
        };
        let cp = FiberBeamCheckpoint {
            trial_disp: self.trial_disp,
            committed_disp: self.committed_disp,
            gauss_points,
            trial_int: self.trial_int.to_vec(),
            committed_int: self.committed_int.to_vec(),
            trial_hinge,
            committed_hinge,
        };
        bincode::serialize(&cp).expect("serialize checkpoint")
    }

    fn deserialize_checkpoint(
        &mut self,
        data: &[u8],
    ) -> Result<(), crate::behavior::CheckpointError> {
        let cp = match bincode::deserialize::<FiberBeamCheckpoint>(data) {
            Ok(cp) => cp,
            Err(_) => match bincode::deserialize::<FiberBeamCheckpointV2>(data) {
                Ok(v2) => FiberBeamCheckpoint {
                    trial_disp: v2.trial_disp,
                    committed_disp: v2.committed_disp,
                    gauss_points: v2.gauss_points,
                    trial_int: v2.trial_int,
                    committed_int: v2.committed_int,
                    trial_hinge: Vec::new(),
                    committed_hinge: Vec::new(),
                },
                Err(_) => {
                    let legacy: FiberBeamCheckpointLegacy = bincode::deserialize(data)
                        .map_err(|e| crate::behavior::CheckpointError::Decode(e.to_string()))?;
                    FiberBeamCheckpoint {
                        trial_disp: legacy.trial_disp,
                        committed_disp: legacy.committed_disp,
                        gauss_points: legacy.gauss_points,
                        trial_int: vec![0.0; self.releases.len()],
                        committed_int: vec![0.0; self.releases.len()],
                        trial_hinge: Vec::new(),
                        committed_hinge: Vec::new(),
                    }
                }
            },
        };
        self.trial_disp = cp.trial_disp;
        self.committed_disp = cp.committed_disp;
        for (gp, gp_mats) in self.gauss_points.iter_mut().zip(cp.gauss_points) {
            for (mat, mat_bytes) in gp.mats.iter_mut().zip(gp_mats) {
                mat.deserialize_state(&mat_bytes)?;
            }
        }
        if cp.trial_int.len() == self.releases.len() {
            self.trial_int = SmallVec::from_slice(&cp.trial_int);
            self.committed_int = SmallVec::from_slice(&cp.committed_int);
        }
        if let Some(h) = &mut self.hinge {
            if cp.trial_hinge.len() == 8 && cp.committed_hinge.len() == 8 {
                h.trial_kappa.copy_from_slice(&cp.trial_hinge[..4]);
                h.trial_thb.copy_from_slice(&cp.trial_hinge[4..]);
                h.committed_kappa.copy_from_slice(&cp.committed_hinge[..4]);
                h.committed_thb.copy_from_slice(&cp.committed_hinge[4..]);
            } else {
                h.trial_kappa = [0.0; 4];
                h.trial_thb = [0.0; 4];
                h.committed_kappa = [0.0; 4];
                h.committed_thb = [0.0; 4];
            }
        }
        Ok(())
    }

    fn ductility_probe(&self) -> Option<DuctilityProbe> {
        let l = self.flex_length;
        if l <= 0.0 || self.gauss_points.is_empty() {
            return None;
        }
        let td = self.elem_disp(&self.flex_disp());
        let mut best: Option<(f64, usize, f64, f64, f64)> = None;
        if let Some(h) = &self.hinge {
            let eps0 = (td[6] - td[0]) / l;
            for gi in 0..self.gauss_points.len().min(2) {
                let (ky, kz) = (h.trial_kappa[gi * 2], h.trial_kappa[gi * 2 + 1]);
                let kappa = (ky * ky + kz * kz).sqrt();
                if best.is_none_or(|(bk, ..)| kappa > bk) {
                    best = Some((kappa, gi, eps0, ky, kz));
                }
            }
        } else {
            for (gi, gp) in self.gauss_points.iter().enumerate() {
                let b = gp.b;
                let eps0 = b[0][0] * td[0] + b[0][6] * td[6];
                let ky = b[1][2] * td[2] + b[1][4] * td[4] + b[1][8] * td[8] + b[1][10] * td[10];
                let kz = b[2][1] * td[1] + b[2][5] * td[5] + b[2][7] * td[7] + b[2][11] * td[11];
                let kappa = (ky * ky + kz * kz).sqrt();
                if best.is_none_or(|(bk, ..)| kappa > bk) {
                    best = Some((kappa, gi, eps0, ky, kz));
                }
            }
        }
        let (kappa, gi, eps0, ky, kz) = best?;
        let gp = &self.gauss_points[gi];
        let mut max_t = 0.0_f64;
        let mut max_c = 0.0_f64;
        let mut max_yr = 0.0_f64;
        let mut jm_num = 0.0_f64;
        let mut jm_den = 0.0_f64;
        for (i, fiber) in gp.section.fibers.iter().enumerate() {
            let eps = eps0 - kz * fiber.y + ky * fiber.z;
            max_t = max_t.max(eps);
            max_c = max_c.max(-eps);
            let sref = gp.mats[i].reference_stress();
            let eref = gp.mats[i].reference_strain();
            if sref > 0.0 && eref > 0.0 {
                let mu_i = eps.abs() / eref;
                max_yr = max_yr.max(mu_i);
                let w = sref * fiber.area * eps.abs();
                jm_num += w * mu_i;
                jm_den += w;
            }
        }
        let jm = if jm_den > 0.0 { jm_num / jm_den } else { 0.0 };
        Some(DuctilityProbe {
            curvature: kappa,
            max_tension_strain: max_t,
            max_compression_strain: max_c,
            max_yield_ratio: max_yr,
            jm,
        })
    }

    fn fiber_section_states(&self) -> Option<Vec<crate::behavior::FiberSectionState>> {
        use crate::behavior::{FiberSectionState, FiberStateSample};
        let l = self.flex_length;
        if l <= 0.0 || self.gauss_points.is_empty() {
            return None;
        }
        let td = self.elem_disp(&self.flex_disp());
        let eps0_hinge = (td[6] - td[0]) / l;
        let n_sections = if self.hinge.is_some() {
            self.gauss_points.len().min(2)
        } else {
            self.gauss_points.len()
        };
        let mut out = Vec::with_capacity(n_sections);
        for (gi, gp) in self.gauss_points.iter().take(n_sections).enumerate() {
            let (eps0, ky, kz) = if let Some(h) = &self.hinge {
                (eps0_hinge, h.trial_kappa[gi * 2], h.trial_kappa[gi * 2 + 1])
            } else {
                let b = gp.b;
                let eps0 = b[0][0] * td[0] + b[0][6] * td[6];
                let ky = b[1][2] * td[2] + b[1][4] * td[4] + b[1][8] * td[8] + b[1][10] * td[10];
                let kz = b[2][1] * td[1] + b[2][5] * td[5] + b[2][7] * td[7] + b[2][11] * td[11];
                (eps0, ky, kz)
            };
            let fibers = gp
                .section
                .fibers
                .iter()
                .enumerate()
                .map(|(i, fiber)| {
                    let eps = eps0 - kz * fiber.y + ky * fiber.z;
                    let eref = gp.mats[i].reference_strain();
                    FiberStateSample {
                        y: fiber.y,
                        z: fiber.z,
                        area: fiber.area,
                        strain: eps,
                        yield_ratio: if eref > 0.0 { eps.abs() / eref } else { 0.0 },
                        material: fiber.material,
                    }
                })
                .collect();
            out.push(FiberSectionState { xi: gp.xi, fibers });
        }
        (!out.is_empty()).then_some(out)
    }
}

#[cfg(test)]
mod tests;
