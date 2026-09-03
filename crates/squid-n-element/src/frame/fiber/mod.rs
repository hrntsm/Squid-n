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
///
/// 両端の塑性化域が全長を食い尽くさないよう各端 45% を上限、下限は数値上の
/// ゼロ割回避のため 1e-6·L とする。要素生成（[`FiberBeam::build_plastic_zone`]）と
/// モデル化図の表示で同じ値を用いるため公開する。
pub fn clamp_plastic_zone(lp: f64, l: f64) -> f64 {
    lp.clamp(1.0e-6 * l, 0.45 * l)
}

/// 鋼材・鉄筋のファイバ材料を生成する。
///
/// Menegotto–Pinto（基本式 Menegotto & Pinto 1973、履歴則 Filippou et al. 1983。
/// 既定 b=0.01, R0=20, a1=18.5, a2=0.15）を用いる。単調載荷ではバイリニア
/// （硬化率 0.01）とほぼ同じ骨格（降伏点近傍が滑らかに丸まる）で、繰返し載荷では
/// バウシンガー効果を表現する。
///
/// # Panics
///
/// ファイバー断面は降伏進展を追うことが目的のため、降伏点未設定は入力不備で
/// あり、解析前の入力チェック（[`crate::factory::ensure_nonlinear_input`]）が
/// エラーで停止する。fy 無し（または 0 以下）でここへ到達するのは呼び出し側の
/// 契約違反（プログラムエラー）のため panic する（弾性で無音に代替しない）。
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
/// 骨格は `fc ≤ 60` で NewRC 構成則、超過で放物線＋線形軟化モデル。
/// 除荷則 `rule` は [`crate::factory::resolve_fiber_concrete_hysteresis`] で
/// 解決した値（逆行型・原点指向型・Karsan–Jirsa 型のいずれか）を渡す:
/// - 逆行型／原点指向型: NewRC（2.1.4）の除荷則切替。放物線モデル（fc>60）は
///   原点指向型のみ対応のため、逆行型指定でも原点指向型で評価する。
/// - Karsan–Jirsa 型: Yassin (1994) の繰返し履歴（`ConcreteCyclic`）。骨格は
///   fc≤60 で NewRC（εcu=0.01）、超過で修正 Kent–Park
///   （εc0=0.002・εcu=0.0035・残留 0＝放物線モデルと同一）。
///   引張は ft=2.0 N/mm²・軟化勾配 Ets=E0/10（いずれも既定）。
///
/// # Panics
///
/// ファイバー断面は材料非線形（ひび割れ・圧壊）を追うことが目的のため、
/// Fc 未設定は入力不備であり、解析前の入力チェック
/// （[`crate::factory::ensure_nonlinear_input`]）がエラーで停止する。
/// Fc 無し（または曲げバネ用履歴則の混入）でここへ到達するのは呼び出し側の
/// 契約違反（プログラムエラー）のため panic する
/// （弾性で無音に代替しない。耐力の過大評価＝危険側になるため）。
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
                // NewRC の除荷則切替（原点指向型= dynamic 相当、逆行型= static 相当）。
                m.set_concrete_hysteresis(rule == HysteresisModel::OriginOriented);
                Box::new(m)
            } else {
                // 放物線モデルは原点指向型のみ対応（逆行型指定でも原点指向型で評価）。
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

/// ファイバ断面の各領域の降伏点 [N/mm²]。**材料は断面が持つ**ため、断面の
/// 主材料・主筋材料・内蔵鉄骨材料からここで解決し、以降は解決済みの値だけを渡す。
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct FiberYield {
    /// 主材料の fy（形状を持たない断面の格子に用いる）。
    pub main: Option<f64>,
    /// 主筋の σy（RC・SRC 断面）。
    pub rebar: Option<f64>,
    /// 鋼材領域の fy（SRC は内蔵鉄骨、それ以外は主材料）。
    pub steel: Option<f64>,
}

/// 断面が持つ材料からファイバの降伏点を解決する。
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

/// 非線形解析の入力チェック（`factory::ensure_nonlinear_input`）と要素生成が
/// 同じ解決規則を共有する。
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
///
/// [`build_gauss_fibers`] は呼ぶたびに独立した断面と材料インスタンスを返す。
/// 各ガウス点は自分の履歴状態（塑性ひずみ・除荷点）を持つため、1 組を複製したり
/// 共有したりはできず、**必ず同じ引数で 2 回呼ぶ**必要がある。この定型と、
/// その前段にある材料文脈（形状・fc・降伏諸元・材料強度割増・除荷則）の解決が
/// マルチファイバー梁の構築 2 か所に複製されていたため、ここへ集約する。
///
/// 断面・材料が未割当のときの既定はゼロ剛性（[`FiberBeamElement::new`] と同じ方針。
/// 解析前チェックで捕捉される前提で、架空の断面を作らない）。
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
    // 保有水平耐力計算（basis==MaterialStrength）時のみ材料強度割増を適用する
    // （鋼材文脈・RC 主筋文脈で係数が異なる。せん断補強筋は割増対象外）。
    let steel_factor = basis.steel_factor(mat_ref);
    let rebar_factor = basis.rebar_factor(mat_ref);
    // RC 断面はコンクリート格子＋主筋分離（構造力学のファイバーモデル）。
    // コンクリート除荷則は解析種別と部材個別指定から解決する。
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

/// ガウス点のファイバー断面と材料を構築する（構造力学のファイバーモデル）。
///
/// 断面形状（[`SectionShape`]）がある場合は、MN 相関曲面と同じ配置規則
/// （`squid_n_section::mn_surface::plastic_fibers_at`）で**形状どおりの領域**へ
/// ファイバを配置する。H 形はフランジ＋ウェブ、角形鋼管・鋼管は管壁のみ、
/// CFT は管壁＋充填コンクリート、RC はコンクリート領域＋主筋点ファイバ、
/// SRC はさらに内蔵鉄骨、のように中空・薄肉断面が正しく表現される
/// （従来は形状によらず width×depth の中実矩形格子で、角形鋼管等の面積・剛性・
/// 耐力を大幅に過大評価していた）。形状がない場合（壁エレメントの壁柱など）は
/// 従来どおり width×depth の中実格子とする。
/// `fc≤60` はコンクリートに NewRC、超過は放物線モデルを用いる。
///
/// ファイバの材料区分タグ（`Fiber::material`、塑性化マップの色分けに使用）:
/// 0=コンクリート、1=主筋、2=鋼材（形鋼・鋼管・内蔵鉄骨）。
/// `concrete_rule` はコンクリートの除荷則
/// （[`crate::factory::resolve_fiber_concrete_hysteresis`] で解決した値）。
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
            // 形状なし（壁エレメントの壁柱など）: 従来どおりの中実矩形格子。
            // 基本格子（コンクリート or 鋼材）。保有水平耐力計算時は鋼材文脈の
            // 材料強度割増（steel_factor）を fy に乗じる（時刻歴応答解析等は 1.0）。
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

    // 配置の座標規約は y=幅方向・z=せい方向だが、要素座標系はせい方向＝ローカル y
    // （LocalFrame: ey=ref_vector 直交化）のため、x 軸まわりの 90° 回転
    // (y,z)←(z,−y) で並べ替え、強軸曲げ（せい方向の応力勾配）が Mz 面
    // （κz・∫y²dA、(uy,rz) ブロック）に対応するようにする。
    // 純回転（行列式 +1）のため鏡像化はしない。現行の `RcRebar` は上下・左右対称
    // 配置しか表現できないため回転の向き（±90°）は結果に影響しないが、将来
    // 非対称配筋（上端筋≠下端筋等）を導入する場合は「せいの上端が +ey 側」となる
    // 向きであることを要再検証。
    for f in &mut fibers {
        let (y, z) = (f.y, f.z);
        f.y = z;
        f.z = -y;
    }
    (FiberSection { fibers }, mats)
}

/// 断面形状に応じたファイバ配置と材料の構築（[`build_gauss_fibers`] の形状あり経路）。
///
/// 配置は MN 相関曲面（`mn_surface::plastic_fibers_at`）と同一規則を共用し、
/// 各ファイバの材料領域区分（[`FiberRegion`]）から非線形材料を割り当てる:
/// - コンクリート領域: NewRC（fc≤60）／放物線モデル。fc 未設定は
///   [`concrete_fiber_material`] が panic する（該当モデルは入力チェックが
///   解析前に停止する）。
/// - 主筋: Menegotto–Pinto 鉄筋（E=205000、σy は断面の主筋材質 → 部材材料の fy の
///   順で解決、`rebar_factor` 割増）。
/// - 鋼材領域（形鋼・鋼管・内蔵鉄骨）: Menegotto–Pinto 鋼材（部材材料の E・fy、
///   `steel_factor` 割増。fy 未設定は [`steel_fiber_material`] が panic する。
///   該当モデルは入力チェックが解析前に停止する）。
///
/// 解像度は最大寸法/16（従来の 12×20 中実格子と同程度のファイバ数）、円環は
/// 周 24 分割とし、MN 曲面の細分割（/40・周 48）より粗く増分解析の計算量を抑える。
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

    // 主筋の降伏点は**断面（配筋）の主筋材質**から解決し、なければ部材材料の fy を
    // 用いる（`rebar_yield_strength`）。未解決（None）のまま主筋・鋼材ファイバを
    // 生成しようとすると `steel_fiber_material` が panic する（弾性で無音に代替
    // しない）。該当モデルは [`crate::factory::ensure_nonlinear_input`] が解析前に
    // エラーで停止するため、通常の解析経路では到達しない。
    let rebar_fy = yield_.rebar.or(yield_.main);
    // 鋼材領域の降伏点: SRC は断面の内蔵鉄骨材料 → 主材料 fy の順で解決済み。
    let steel_fy = yield_.steel;
    // StrengthParams はファイバの**配置**（形状分割・領域区分）にのみ使い、
    // 材料モデルの強度には入らない（材料は下の各領域テンプレートで構築する）。
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

    // 各領域の材料テンプレートは**その領域のファイバが実在するときのみ**構築する
    // （遅延構築）。純 RC 断面など鋼材領域を持たない断面で fy 未設定を理由に、
    // 純鋼材断面などコンクリート領域を持たない断面で Fc 未設定を理由に、
    // それぞれ panic しないため。
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

/// 材端解放（ピン・半剛）で内部自由度へ分離した要素端回転。
///
/// 剛接端は「節点回転＝要素端回転」を厳密に満たすため内部自由度を作らない。
/// ピン・半剛の端のみ要素端回転を内部自由度へ分離し、節点回転との間に回転ばね
/// `spring` を挟んで静縮約する（弾性梁 `BeamElement::condense_end_springs` と同じ
/// 定式化。ピンは `spring = 0` で厳密なモーメント解放）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EndRelease {
    /// 可撓端系ローカル自由度 index（3,4,5 = i 端 rx/ry/rz、9,10,11 = j 端）。
    pub dof: usize,
    /// 節点回転と要素端回転の間の回転ばね剛性 [N·mm/rad]（ピンは 0）。
    pub spring: f64,
}

/// 端条件とねじれ解放から、解放する回転自由度を決める。
///
/// `EndCondition::Fixed` は解放しない。ピン・半剛は当該端の rx/ry/rz を解放する。
/// `torsion_release` が立つ端は、端条件が剛接でも rx（ねじれ）のみ解放する
/// （梁のねじり剛性を期待しない既定モデル化。`beam::torsion` 参照）。
/// ただし **ねじり剛性がない部材（J≤0）の rx は解放しない**。解放しても縮約行列
/// `Kbb` の対角がゼロになり特異化するだけで、モーメント解放としての意味がないため。
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

/// [`FiberBeam::snapshot_state`] が返すスナップショットの型。
/// （トライアル変位・確定変位・各ガウス点の材料・内部自由度のトライアル/確定値・
/// 塑性増分ヒンジ状態のトライアル/確定値。ヒンジ無しモデルは空 Vec）
pub type FiberBeamSnapshot = (
    [f64; 12],
    [f64; 12],
    Vec<Vec<Box<dyn UniaxialMaterial>>>,
    Vec<f64>,
    Vec<f64>,
    Vec<f64>,
    Vec<f64>,
);

/// `FiberBeam` のチェックポイント（現行形式）。
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

/// 塑性増分ヒンジ状態を持たない一つ前の形式のチェックポイント（読み込み互換用）。
#[derive(serde::Deserialize)]
struct FiberBeamCheckpointV2 {
    trial_disp: [f64; 12],
    committed_disp: [f64; 12],
    gauss_points: Vec<Vec<Vec<u8>>>,
    trial_int: Vec<f64>,
    committed_int: Vec<f64>,
}

/// 材端解放の内部自由度を持たない旧形式のチェックポイント（読み込み互換用）。
#[derive(serde::Deserialize)]
struct FiberBeamCheckpointLegacy {
    trial_disp: [f64; 12],
    committed_disp: [f64; 12],
    gauss_points: Vec<Vec<Vec<u8>>>,
}

/// 塑性増分ヒンジ（端部塑性化域の直列モデル）の定義と状態。
///
/// 塑性化域考慮モデルを「**全可撓長の弾性梁** + **端部の塑性増分ヒンジ**」の
/// 直列として解くための構成。ヒンジは端部ファイバー断面（`gauss_points` の
/// ξ=∓1 断面）の曲率 κ を内部未知数とし、各トライアルで両端 2 軸の内部平衡
///
/// \\[ m_B = D_{nom} \cdot B(\xi_{end}) \cdot \hat u = m_{sec}(\varepsilon_0, \kappa) \\]
///
/// （弾性梁の端部モーメント＝断面モーメント）を要素内 Newton で解く。
/// ヒンジ回転（節点回転と可撓端回転の差）は断面の弾性線を**超える塑性超過分**
///
/// \\[ \gamma = s \cdot L_p \cdot (\kappa - m_{sec}(\kappa)/EI_{sec}) \\]
///
/// のみを持つ（s は端の向き: i 端 +1・j 端 −1。B 行列の自端回転係数の符号に一致）。
/// 弾性状態では γ=0 となり要素は弾性梁 `k_el` に厳密一致し、降伏後は塑性回転が
/// Lp に局所化して要素剛性が実際に低下する。従来の「端部ガウス点の B 積分
/// （重み 2Lp/L'）＋中央弾性剛性の加算」は、端部断面が全塑性化しても要素剛性が
/// 積分重み分（数 %）しか低下しない変位法の限界があった。
#[derive(Clone)]
pub struct HingeState {
    /// 塑性化域長 Lp [mm]。
    pub lp: f64,
    /// 全可撓長の弾性剛性（曲げ＋軸＋せん断＋ねじり、可撓端系 12×12）。
    pub k_el: LocalMat,
    /// 公称弾性 D 対角 [EA, E·Iy_elem, E·Iz_elem]（`m_B = D·B·û` の評価用）。
    d_nom: [f64; 3],
    /// 端部ファイバー断面の弾性曲げ剛性 [端][軸]（軸 0=κy=EIy_sec, 1=κz=EIz_sec、
    /// ファイバー離散化後の値。γ=0 の弾性整合はこの値で成立する）。
    sec_ei: [[f64; 2]; 2],
    /// ヒンジが有効な端（端条件 Fixed のみ。ピン・半剛端は材端解放側で扱う）。
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
    /// B 行列（ひずみ－変位行列）のキャッシュ。`xi`（ガウス点固定）・`l`
    /// （可撓長）・`phi_y`/`phi_z`（要素生成後は不変）のみに依存するため、
    /// 要素生成時に 1 回だけ計算して保持する（`compute_b_matrix` と同一値）。
    b: [[f64; 12]; 3],
    /// 断面応答（軸力・両軸曲げモーメント、および対応する接線）のキャッシュ。
    /// `trial_stress`/`trial_et` を書き換える全経路（`update_section_trial`・
    /// `update_hinge_section_trial`）で、書き換え直後に `refresh_response` を
    /// 呼んで更新するため、読み出し側は常に trial 状態と整合した値を参照できる。
    cached_force: [f64; 3],
    cached_stiff: [[f64; 3]; 3],
}

impl GaussPoint {
    /// ガウス点を生成する。`l`（可撓長）・`phi_y`・`phi_z` は要素生成後に不変
    /// なので、B 行列をここで 1 回だけ計算してキャッシュする。
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
        // 接線キャッシュを各ファイバの初期弾性接線で初期化する。
        // 未初期化（0）のままだと、最初の update_state より前に tangent_stiffness を
        // 呼ぶ経路（pushover の初回 assemble_k）で剛性が 0 になり特異化する。
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

    /// `trial_stress`/`trial_et`（ファイバー単位のトライアル状態）から断面応答
    /// （force, stiff）の総和を計算してキャッシュへ格納する。積分公式は
    /// [`squid_n_section::fiber::integrate_fibers`]（材料の `trial` を毎回呼ぶ
    /// 経路と同一の実体）を用いるため、結果はビット一致する。`trial_stress`/
    /// `trial_et` を書き換える経路（`update_section_trial`・
    /// `update_hinge_section_trial`）は、書き換え直後に必ずこれを呼ぶこと。
    fn refresh_response(&mut self) {
        let (stress, et) = (&self.trial_stress, &self.trial_et);
        let (f, s) =
            squid_n_section::fiber::integrate_fibers(&self.section, |i, _| (stress[i], et[i]));
        self.cached_force = [f.n, f.my, f.mz];
        self.cached_stiff = s.d;
    }
}

/// ファイバー梁要素（変位法、Timoshenko 適合内挿＋Saint-Venant ねじり）。
///
/// せん断変形は Timoshenko 適合内挿（φ 依存の曲率形状関数＋一定せん断ひずみ場
/// による変位法内挿）で直列に合成する。
/// 曲率場を 1/(1+φ) で補正し、曲げ面ごとの一定せん断ひずみ
/// （γy・γz、符号規約は `compute_shear_stiffness` の doc 参照。剛体回転で
/// 恒等的にゼロ）に弾性せん断剛性 GAs を作用させることで、断面剛性が φ の
/// 算定基礎と一致する一様弾性断面では弾性 Timoshenko 梁（`BeamElement`）の
/// 剛性と厳密に一致する。
/// φ = 12EI/(GAs·L²) は**公称断面諸元**（Section.iy/iz・as_y/as_z と
/// Material.young/shear_modulus）から算定して凍結する（降伏後も内挿は
/// 弾性時の配分を保つ。曲げの Hermite 内挿と同型の近似）。凍結の方針は
/// OpenSees と同じだが、OpenSees がファイバー断面の初期接線から算定するのに
/// 対し、本実装は線形解析（`BeamElement`）の φ と一致させるため公称値を
/// 用いる（RC 等でファイバー実効初期剛性が公称値と乖離する場合、φ は
/// 公称値ベースの近似となる）。
/// GAs ≤ 0（せん断有効断面積が未設定等）の場合は φ=0（せん断剛直 =
/// Euler-Bernoulli）へフォールバックする。
///
/// # 剛域
///
/// 部材端に剛域（`ElementData::rigid_zone`）があるとき、断面積分・せん断・幾何剛性は
/// **可撓長** `flex_length` = L − λi − λj で組み、可撓端自由度を剛体アームで節点
/// 自由度へ写す（[`crate::frame::rigid_arm`]。弾性梁 `BeamElement` と同じ変換）。
/// 端部の積分点 ξ=∓1 は剛域フェイスに位置し、塑性化域 Lp も剛域フェイスから測る。
///
/// 軸方向の扱いは弾性梁と異なる。弾性梁は軸断面積を A·(L'/L) に補正して軸剛性を
/// EA/L（節点間長基準）へ戻す（剛域は曲げのみを剛とする方針）が、ファイバー要素は
/// 断面積分が軸力と曲げを連成させるため、軸だけを分離して補正すると断面が返す軸力
/// （N-M 相関の基礎）が実断面のひずみ状態と食い違う。そのため本要素では剛域を
/// **軸方向にも剛**として扱い、軸剛性は EA/L' となる（剛体オフセットの運動学に
/// そのまま従う扱い）。ねじりは断面積分と連成しない独立項なので、弾性梁と同じく
/// 節点間長基準 GJ/L とする。
pub struct FiberBeam {
    /// 節点間長 L [mm]（質量・ねじり剛性の基準）。
    pub length: f64,
    /// 材端の剛域長 λi, λj [mm]（可撓長 = `length` − λi − λj）。
    pub rigid_i: f64,
    pub rigid_j: f64,
    /// 可撓長 L' = `length` − λi − λj [mm]。断面積分・B 行列・せん断・幾何剛性の基準。
    pub flex_length: f64,
    pub nodes: [NodeId; 2],
    pub gauss_points: Vec<GaussPoint>,
    pub density: f64,
    /// ねじり定数 J [mm⁴]（Section.j から取得）。
    /// Saint-Venant ねじり剛性 G·J/L の計算に用いる。
    pub torsion_j: f64,
    /// せん断弾性係数 G [N/mm²]（Material.shear_modulus）。
    /// ねじり剛性の計算に用いる。
    pub g: f64,
    /// せん断変形係数 φy（局所 y 並進－rz 回転＝強軸曲げ面）。クロス変換規約
    /// （beam/construct.rs と同一）により断面 iy（強軸）・as_z（ウェブ）から
    /// φy = 12E·iy_sec/(G·as_z_sec·L²) として算定して凍結。
    /// GAs ≤ 0 なら 0（Euler フォールバック）。
    pub phi_y: f64,
    /// せん断変形係数 φz（局所 z 並進－ry 回転＝弱軸曲げ面）。クロス変換規約に
    /// より断面 iz（弱軸）・as_y（フランジ）から φz = 12E·iz_sec/(G·as_y_sec·L²)。
    pub phi_z: f64,
    /// せん断ひずみ場の弾性剛性寄与（ローカル系 12×12、両曲げ面の
    /// GAs·L·Bγᵀ·Bγ の和）。γ は一定場のため定数行列として前計算する。
    pub k_shear: LocalMat,
    /// 要素ローカル系→グローバル系の回転（柱・斜材で必須）。
    /// 内部状態（trial_disp 等）はローカル系で保持し、トレイト境界で回転する。
    pub axis: crate::transform::LocalFrame,
    /// 塑性増分ヒンジ（端部塑性化域の直列モデル）。
    /// None = 全長ファイバー積分モデル（B 積分のみ）。
    pub hinge: Option<HingeState>,
    /// 材端解放（ピン・半剛）で分離した要素端回転（内部自由度）。空なら全端剛接。
    pub releases: SmallVec<[EndRelease; 6]>,
    /// 内部自由度の現在値（`releases` と同順。可撓端系ローカルの要素端回転）。
    pub trial_int: SmallVec<[f64; 6]>,
    pub committed_int: SmallVec<[f64; 6]>,
    /// 内力を評価する危険断面位置（正規化座標 \[0,1\]）。弾性梁
    /// （`BeamElement::eval_sections`）と同じ規則で与え、非線形解析の部材内力
    /// （`state_member_forces`）を線形解析と同じ断面で取り出せるようにする。
    pub eval_sections: Vec<f64>,
    pub committed_disp: [f64; 12],
    pub trial_disp: [f64; 12],
}

impl FiberBeam {
    /// ファイバー梁の生成（材料強度の基準は `basis` で指定する）。
    /// 時刻歴応答解析など、材料強度割増を伴わない解析用の薄いラッパー。
    /// ファイバー梁の生成（材料強度の基準 `basis` を明示指定する版）。
    /// 保有水平耐力計算（プッシュオーバー）は
    /// `StrengthBasis::MaterialStrength` を渡す。
    pub fn new(
        data: &squid_n_core::model::ElementData,
        model: &squid_n_core::model::Model,
        basis: crate::factory::StrengthBasis,
        kind: AnalysisKind,
    ) -> Self {
        let n0 = &model.nodes[data.nodes[0].index()];
        let n1 = &model.nodes[data.nodes[1].index()];
        let length = squid_n_core::geom::vec3::dist(n0.coord, n1.coord);
        // 剛域長と可撓長。断面積分・B 行列・せん断・幾何剛性はすべて可撓長基準で
        // 組み、可撓端自由度を剛体アームで節点自由度へ写す（弾性梁と同じ扱い）。
        let (rigid_i, rigid_j) = crate::frame::rigid_arm::resolve_lengths(
            data.rigid_zone.rigid_length_i(),
            data.rigid_zone.rigid_length_j(),
            length,
        );
        let flex_length = length - rigid_i - rigid_j;

        let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
        let mat_ref = model.element_material(data);
        let density = mat_ref.map(|m| m.density).unwrap_or(0.0);
        // 断面・材料の未割当は解析前チェック（solver の precheck_model・
        // factory の ensure_nonlinear_input）で捕捉される前提。ここでの既定は
        // ゼロ剛性とし、チェックを通らない経路から来ても「もっともらしい断面」で
        // 無音に解析が通ることはなく、特異行列として顕在化させる
        // （従来は E=205000・100×200 の架空の鋼断面として静かに解析されていた）。
        let e = mat_ref.map(|m| m.young).unwrap_or(0.0);
        let g = mat_ref.map(|m| m.shear_modulus()).unwrap_or(0.0);
        let width = sec.map(|s| s.width).unwrap_or(0.0);
        let depth = sec.map(|s| s.depth).unwrap_or(0.0);
        let torsion_j = sec.map(|s| s.j).unwrap_or(0.0);

        // Timoshenko 適合内挿の φ（弾性断面諸元から算定して凍結）。
        // 断面レイヤ→要素座標系のクロス変換（beam/construct.rs・ファイバ格子の
        // 90°回転と同一規約）: (uy, rz) ブロック＝強軸曲げ（Mz 面）には
        // 断面 iy（強軸）と as_z（ウェブ）、(uz, ry) ブロック＝弱軸曲げ（My 面）
        // には 断面 iz（弱軸）と as_y（フランジ）を対応させる。
        // GAs ≤ 0（未設定等）は φ=0（Euler フォールバック）。
        let sec_iy = sec.map(|s| s.iy).unwrap_or(0.0);
        let sec_iz = sec.map(|s| s.iz).unwrap_or(0.0);
        let sec_as_y = sec.map(|s| s.as_y).unwrap_or(0.0);
        let sec_as_z = sec.map(|s| s.as_z).unwrap_or(0.0);
        // φ は可撓長基準（弾性梁が可撓長で raw 剛性を組むのと同じ規約）。
        let phi_of = |ei: f64, gas: f64| {
            if gas > 0.0 && ei > 0.0 && flex_length > 0.0 {
                12.0 * ei / (gas * flex_length * flex_length)
            } else {
                0.0
            }
        };
        // 要素 (uy,rz) 面 ← 断面 iy・as_z / 要素 (uz,ry) 面 ← 断面 iz・as_y
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

        // 材端解放（ピン・半剛＋梁の既定ねじれ解放）。ねじり剛性がない部材の
        // rx は解放しない。
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

    /// せん断ひずみ場（一定）の弾性剛性 Σ GAs·L·Bγᵀ·Bγ を前計算する。
    ///
    /// γy = φy/(2(1+φy))·(rz_i + rz_j − 2(uy_j − uy_i)/L)（uy–rz 面）、
    /// γz = φz/(2(1+φz))·(ry_i + ry_j + 2(uz_j − uz_i)/L)（uz–ry 面）。
    /// いずれも剛体回転（回転角＝弦回転）で恒等的にゼロとなる客観的な測度。
    /// φ 補正後の曲率剛性と合算すると一様弾性断面で Timoshenko 厳密剛性になる。
    fn compute_shear_stiffness(l: f64, phi_y: f64, phi_z: f64, gas_y: f64, gas_z: f64) -> LocalMat {
        let mut k = LocalMat::zeros(12);
        if l <= 0.0 {
            return k;
        }
        // (Bγ の非零成分, GAs) を面ごとに組み立てて GAs·L·Bγᵀ·Bγ を加算
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

    /// 断面・軸などを直接指定して塑性増分ヒンジ付きファイバー要素を組み立てる
    /// （`ElementData` を持たない合成部材用。耐震壁の壁柱など）。
    /// 諸元は**要素座標系**で与える（クロス変換済み: `iy_elem` は κy=(uz,ry) 面、
    /// `iz_elem` は κz=(uy,rz) 面。`as_z_elem` は (uy,rz) 面のせん断有効断面積）。
    /// 両端剛接としてヒンジを両端有効にする。剛域なし。
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
        // 要素 (uy,rz) 面 = κz = iz_elem・as_y_elem / (uz,ry) 面 = κy = iy_elem・as_z_elem
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
        // K_el（全可撓長の弾性剛性）とヒンジの構築（`build_plastic_zone` と同一手順）。
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

    /// 塑性化域考慮のファイバー要素（材端剛塑性ばねモデルと適合する
    /// ファイバーモデル化）。端部の塑性化領域（長さ `lp`）にファイバー断面を
    /// 配置（積分点 ξ=∓1）し、要素は「全可撓長の弾性梁＋端部塑性増分ヒンジ」の
    /// 直列モデル（[`HingeState`]）として解く。
    /// 剛域があるときの基準長 L' は可撓長（積分点は剛域フェイス）。
    /// 塑性化域考慮のファイバー要素の生成（材料強度の基準 `basis` を明示指定する版）。
    pub fn with_plastic_zone(
        data: &squid_n_core::model::ElementData,
        model: &squid_n_core::model::Model,
        lp: f64,
        basis: crate::factory::StrengthBasis,
        kind: AnalysisKind,
    ) -> Self {
        Self::build_plastic_zone(data, model, lp, 12, 20, basis, kind)
    }

    /// 塑性化域考慮要素の実体。
    /// `nw × nd` は端部断面のファイバ分割数
    /// （マルチファイバー: 12×20、マルチスプリング: 2×5 の粗い配置）。
    /// 塑性化域考慮要素の実体（材料強度の基準 `basis` を明示指定する版）。
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
        // 基準長は可撓長（剛域がなければ節点間長に等しい）。積分点 ξ=∓1 は
        // 剛域フェイス、塑性化域 Lp も剛域フェイスから測る。
        let l = fb.flex_length;
        if l <= 0.0 {
            return fb;
        }
        // Lp は可撓長の 45% までにクランプ（両端合計で可撓長を超えない）
        let lp = clamp_plastic_zone(lp, l);

        let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
        let mat_ref = model.element_material(data);
        // 断面・材料の未割当時の既定はゼロ剛性（`Self::new` と同じ方針。
        // 解析前チェックで捕捉される前提で、架空の断面を作らない）。
        let e = mat_ref.map(|m| m.young).unwrap_or(0.0);
        let width = sec.map(|s| s.width).unwrap_or(0.0);
        let depth = sec.map(|s| s.depth).unwrap_or(0.0);
        let area = sec.map(|s| s.area).unwrap_or(width * depth);
        // 断面レイヤ→要素座標系のクロス変換（beam/construct.rs と同一規約）。
        // 断面 iy（強軸）は要素座標系では z 軸まわり（Mz 面）＝EIz へ、
        // 断面 iz（弱軸）は y 軸まわり（My 面）＝EIy へ対応する。
        let iy = sec.map(|s| s.iz).unwrap_or(0.0);
        let iz = sec.map(|s| s.iy).unwrap_or(0.0);

        // 端部積分点: ξ=∓1、重み w·(L/2) = Lp → w = 2Lp/L
        let w_end = 2.0 * lp / l;
        let [(sec_a, mats_a), (sec_b, mats_b)] =
            build_gauss_fiber_pair(data, model, basis, kind, width, depth, nw, nd);
        fb.gauss_points = vec![
            GaussPoint::new(-1.0, w_end, sec_a, mats_a, l, fb.phi_y, fb.phi_z),
            GaussPoint::new(1.0, w_end, sec_b, mats_b, l, fb.phi_y, fb.phi_z),
        ];

        // 全可撓長の弾性剛性 K_el: B(ξ)ᵀ·diag(EA,EIy,EIz)·B(ξ) を 2 点 Gauss
        // （区間 [−1,1]、被積分関数は ξ の 2 次のため厳密）で積分し、
        // 一定せん断ひずみ場・ねじりを加算する。弾性状態の要素剛性は
        // （ヒンジ回転 γ=0 のため）この K_el に厳密一致する。
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

        // 端部ファイバー断面の弾性曲げ剛性（離散化後）。ヒンジ回転 γ の
        // 「弾性線を超える塑性超過分」の基準線に用いる（γ=0 の弾性整合は
        // 公称値でなく断面自身の弾性剛性で成立させる）。
        let sec_ei = std::array::from_fn(|end| {
            let (_, d) = Self::section_response_from_cache(&fb.gauss_points[end]);
            [d[1][1], d[2][2]]
        });
        // ヒンジは剛接端のみ有効（ピン・半剛端は材端解放の内部自由度側で扱い、
        // モーメントが伝わらない/接合部ばね経由のため塑性ヒンジは形成させない）。
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

    /// 現在のトライアル変位（ローカル系・節点自由度）を可撓端自由度へ写した値。
    ///
    /// 断面ひずみ・中央弾性部・せん断ひずみ場はいずれも可撓部の変形で決まるため、
    /// これらの評価には節点変位ではなく可撓端変位を用いる。剛域がなければ
    /// `trial_disp` と一致する。
    pub fn flex_disp(&self) -> [f64; 12] {
        crate::frame::rigid_arm::to_flex_disp(&self.trial_disp, self.rigid_i, self.rigid_j)
    }

    /// 要素の変形自由度（可撓端系 12）。解放した端回転は節点回転ではなく内部自由度
    /// （`trial_int`）の値を用いる。全端剛接なら `u_flex` と一致する。
    fn elem_disp(&self, u_flex: &[f64; 12]) -> [f64; 12] {
        let mut u = *u_flex;
        for (k, rel) in self.releases.iter().enumerate() {
            u[rel.dof] = self.trial_int[k];
        }
        u
    }

    /// 要素変形 `u_elem` に対する各ガウス点のファイバーひずみ・応力・接線を更新する。
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
            // trial_stress/trial_et を書き換えた直後に断面応答キャッシュを更新する。
            gp.refresh_response();
        }
    }

    /// 可撓端系 12×12 の接線剛性（剛体アーム変換・材端解放の縮約より前）。
    /// 全長ファイバー積分モデルは断面積分＋一定せん断ひずみ場＋ねじり、
    /// 塑性増分ヒンジモデルは弾性梁＋ヒンジの整合接線を返す。
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

        // せん断ひずみ場（一定 γ、弾性 GAs）の剛性を加算。
        // φ 補正済み曲率剛性との和で一様弾性断面の Timoshenko 厳密剛性になる。
        for i in 0..12 {
            for j in 0..12 {
                let v = self.k_shear.get(i, j);
                if v != 0.0 {
                    k.set(i, j, k.get(i, j) + v);
                }
            }
        }

        // ねじり剛性（Saint-Venant）を rx DOF (index 3, 9) に付加。ねじりは断面積分と
        // 連成しない独立項のため、弾性梁（4.1.4）と同じく節点間長基準 GJ/L とし、
        // 剛域では増大させない（剛体アーム変換は rx 自由度に作用しないため、
        // 可撓端系で加算しても節点系で加算しても同じ）。
        if let Some(kt) = self.torsion_stiffness() {
            k.set(3, 3, k.get(3, 3) + kt);
            k.set(9, 9, k.get(9, 9) + kt);
            k.set(3, 9, k.get(3, 9) - kt);
            k.set(9, 3, k.get(9, 3) - kt);
        }
        k
    }

    /// ねじり剛性 GJ/L（節点間長基準）。J≤0 では None。
    fn torsion_stiffness(&self) -> Option<f64> {
        (self.torsion_j > 0.0 && self.length > 0.0).then(|| self.g * self.torsion_j / self.length)
    }

    /// 要素変形 `u_elem` に対する可撓端系 12 の内力（剛体アーム変換・材端解放の
    /// 縮約より前）。断面応答はキャッシュ（`trial_stress`）を用いるため、
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

        // せん断ひずみ場（線形弾性: K_shear·u）。
        // γ は剛体運動でゼロの客観的測度なので偽内力は生じない。
        for i in 0..12 {
            let mut si = 0.0;
            for j in 0..12 {
                si += self.k_shear.get(i, j) * u_elem[j];
            }
            f[i] += si;
        }

        // ねじり内力（Saint-Venant）
        if let Some(kt) = self.torsion_stiffness() {
            let drx = u_elem[3] - u_elem[9];
            f[3] += kt * drx;
            f[9] -= kt * drx;
        }
        f
    }

    /// 内部自由度（解放した要素端回転）を内部釣合いへ収束させる。
    ///
    /// 残差は `R_k = f_elem[dof_k] + k_s·(u_elem[dof_k] − u_flex[dof_k])`。
    /// ピン（k_s=0）なら「当該端の要素モーメント＝0」、半剛なら「要素モーメント＝
    /// ばねモーメント」を意味する。弾性域では 1 反復で厳密に収束する（線形）。
    /// 収束後、断面のトライアル状態は確定した `u_elem` と整合した状態で残る。
    fn solve_internal_dofs(&mut self) {
        /// 内部釣合いの最大反復数（弾性域は 1 回、降伏を跨いでも数回で収まる）。
        const MAX_ITER: usize = 20;
        let u_flex = self.flex_disp();
        if self.releases.is_empty() {
            let u_elem = self.elem_disp(&u_flex);
            self.update_trial_state(&u_elem);
            return;
        }
        // releases は最大 6（EndRelease の SmallVec 上限）。ヒープ確保を避けるため
        // 固定長配列（n はそれ以下の実長）で扱う。
        let n = self.releases.len();
        for _ in 0..MAX_ITER {
            let u_elem = self.elem_disp(&u_flex);
            self.update_trial_state(&u_elem);
            let f_elem = self.elem_internal_force(&u_elem);

            let mut r = [0.0_f64; 6];
            for (k, rel) in self.releases.iter().enumerate() {
                r[k] = f_elem[rel.dof] + rel.spring * (u_elem[rel.dof] - u_flex[rel.dof]);
            }
            // 収束判定は要素の回転自由度内力のスケール基準（残差はモーメント [N·mm]）。
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
            // 縮約行列が特異なら更新を打ち切り、直前の状態を保つ。
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
        // 収束打ち切り時も断面状態を最終 u_elem と整合させる。
        let u_elem = self.elem_disp(&u_flex);
        self.update_trial_state(&u_elem);
    }

    /// 要素変形 `u_elem` に対するトライアル状態の更新。
    /// 塑性増分ヒンジモデルはヒンジの内部平衡を解き、
    /// 全長ファイバー積分モデルは B 行列由来の断面ひずみで更新する。
    fn update_trial_state(&mut self, u_elem: &[f64; 12]) {
        if self.hinge.is_some() {
            self.solve_hinges(u_elem);
        } else {
            self.update_section_trial(u_elem);
        }
    }

    /// 材端解放した要素端回転を静縮約し、可撓端系 12×12 へ戻す
    /// （K* = Kaa − Kab·Kbb⁻¹·Kba。弾性梁 `condense_end_springs` と同じ定式化）。
    fn condense_releases(&self, k_elem: &LocalMat) -> LocalMat {
        let releases: SmallVec<[(usize, f64); 6]> =
            self.releases.iter().map(|r| (r.dof, r.spring)).collect();
        crate::frame::prismatic::condense_end_releases(k_elem, &releases)
    }

    /// ガウス点の断面応答（force, stiff）を返す。総和は `GaussPoint::refresh_response`
    /// で trial_stress/trial_et 書き換え直後に計算済みのため、ここでは
    /// キャッシュを読むだけでよい（ファイバー再走査なし）。
    fn section_response_from_cache(gp: &GaussPoint) -> ([f64; 3], [[f64; 3]; 3]) {
        (gp.cached_force, gp.cached_stiff)
    }

    /// ひずみ－変位行列（行 0: 軸ひずみ、行 1: κy、行 2: κz）。
    ///
    /// 曲率行は Timoshenko 適合内挿（φ 依存形状関数）: Euler-Bernoulli の
    /// Hermite 曲率場に対し、回転 DOF の定数項を (1±3ξ) → (1±3ξ+φ) とし
    /// 全体を 1/(1+φ) 倍する。φ=0 で従来の Hermite 曲率場へ厳密に退化する。
    /// 一定せん断ひずみ場（`compute_shear_stiffness`）と合算すると、
    /// 一様弾性断面で Timoshenko 厳密剛性を再現する（被積分関数は ξ の
    /// 2 次のままなので 2 点 Gauss で厳密）。
    fn compute_b_matrix(xi: f64, l: f64, phi_y: f64, phi_z: f64) -> [[f64; 12]; 3] {
        let inv_l = 1.0 / l;
        let inv_l2 = 1.0 / (l * l);
        let mut b = [[0.0; 12]; 3];
        b[0][0] = -inv_l;
        b[0][6] = inv_l;
        // κy（uz–ry 面、φz）
        let cz = 1.0 / (1.0 + phi_z);
        b[1][2] = 6.0 * xi * inv_l2 * cz;
        b[1][4] = (1.0 - 3.0 * xi + phi_z) * inv_l * cz;
        b[1][8] = -6.0 * xi * inv_l2 * cz;
        b[1][10] = -(1.0 + 3.0 * xi + phi_z) * inv_l * cz;
        // κz（uy–rz 面、φy）
        let cy = 1.0 / (1.0 + phi_y);
        b[2][1] = -6.0 * xi * inv_l2 * cy;
        b[2][5] = (1.0 - 3.0 * xi + phi_y) * inv_l * cy;
        b[2][7] = 6.0 * xi * inv_l2 * cy;
        b[2][11] = -(1.0 + 3.0 * xi + phi_y) * inv_l * cy;
        b
    }

    // ===== 塑性増分ヒンジ（端部塑性化域の直列モデル）=====

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

    /// 有効ヒンジ自由度の回転スロットを可撓端回転 θb で置き換えた変位ベクトル û。
    fn hinge_uhat_from(dofs: &[(usize, usize)], thb: &[f64; 4], u_elem: &[f64; 12]) -> [f64; 12] {
        let mut u = *u_elem;
        for &(end, axis) in dofs {
            u[HINGE_SLOTS[end][axis]] = thb[end * 2 + axis];
        }
        u
    }

    /// 端部断面のトライアル状態を (ε0, κy, κz) で直接更新する（ヒンジモデル用。
    /// B 行列による曲率復元は行わない）。
    fn update_hinge_section_trial(&mut self, end: usize, eps0: f64, ky: f64, kz: f64) {
        let gp = &mut self.gauss_points[end];
        for (i, fiber) in gp.section.fibers.iter().enumerate() {
            let eps = eps0 - kz * fiber.y + ky * fiber.z;
            let (sigma, et) = gp.mats[i].trial(eps);
            gp.trial_stress[i] = sigma;
            gp.trial_et[i] = et;
        }
        // trial_stress/trial_et を書き換えた直後に断面応答キャッシュを更新する。
        gp.refresh_response();
    }

    /// ヒンジの内部平衡を解く。未知数は有効端の断面曲率 κ（最大 4）で、
    /// 各端・各軸について「弾性梁の端部モーメント m_B = D_nom·B(ξ_end)·û」と
    /// 「断面モーメント m_sec(ε0, κ)」の釣合いを Newton 反復で満たす。
    /// 収束後、`trial_kappa`・`trial_thb` と端部断面のトライアル状態が
    /// 最終の û と整合した状態になる。
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
        // 軸ひずみは弾性（B 行 0）。両端共通で、断面の N-M 相関にのみ使う。
        let eps0 = (u_elem[6] - u_elem[0]) / l;
        if n == 0 {
            // ヒンジ無効端のみ: 断面状態を現在の κ（=0 のまま）で整合させる。
            for end in 0..2 {
                let (ky, kz) = (kappa[end * 2], kappa[end * 2 + 1]);
                self.update_hinge_section_trial(end, eps0, ky, kz);
            }
            return;
        }
        // ヒンジモデルは常にガウス点 2 点（ξ=∓1、i端/j端）を持つため、
        // 各ガウス点のキャッシュ済み B 行列がそのまま b_end[0]/b_end[1] になる。
        let b_end = [self.gauss_points[0].b, self.gauss_points[1].b];

        for _ in 0..MAX_ITER {
            // 1) 断面応答（両端）とヒンジ回転 γ → θb
            let mut m = [[0.0_f64; 2]; 2]; // [端][軸] 断面モーメント
            let mut dm = [[[0.0_f64; 2]; 2]; 2]; // [端][軸][軸] 断面接線
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

            // 2) 弾性梁側の端部モーメント m_B と残差 R
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

            // 3) Jacobian: J[p][q] = Σ_{p'} D·B[slot_{p'}]·G[p'][q] − δ_end·dm
            //    G[p'][q] = ∂θb_{p'}/∂κ_q = −s·Lp·(δ − dm/EI_sec)（同一端のみ非零）
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
            // ヤコビアンが特異なら反復を打ち切り、直前の状態を保つ。
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

        // 最終 κ で断面状態・θb を整合させて書き戻す。
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

    /// ヒンジモデルの内力: f = K_el·û（û は有効端の回転スロットを可撓端回転 θb で
    /// 置き換えた変位）。内部平衡の解では回転スロットの f が断面モーメントと
    /// 一致するため、履歴に整合した復元力になる。
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

    /// ヒンジモデルの整合接線 K* = K_el − (K_el[:,slots]·G)·J⁻¹·(D·B 行)。
    /// 弾性（G=0）では K_el に厳密一致する。内部変数消去の整合接線は一般に
    /// 非対称になり得るため、対称ソルバ（Cholesky）前提の全体組立に合わせて
    /// 対称化して返す（内力は厳密なので収束解は変わらない）。
    fn hinge_tangent(&self, h: &HingeState) -> LocalMat {
        let dofs = Self::hinge_dofs(h);
        let n = dofs.len();
        if n == 0 {
            return LocalMat {
                n: 12,
                data: h.k_el.data.clone(),
            };
        }
        // ヒンジモデルは常にガウス点 2 点（ξ=∓1、i端/j端）を持つため、
        // 各ガウス点のキャッシュ済み B 行列がそのまま b_end[0]/b_end[1] になる。
        let b_end = [self.gauss_points[0].b, self.gauss_points[1].b];
        // 現在のトライアル状態の断面接線。
        let mut dm = [[[0.0_f64; 2]; 2]; 2];
        for end in 0..2 {
            let (_, d) = Self::section_response_from_cache(&self.gauss_points[end]);
            dm[end] = [[d[1][1], d[1][2]], [d[2][1], d[2][2]]];
        }
        // G[p][q] = ∂θb_p/∂κ_q（同一端のみ非零）
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
        // J[p][q] = Σ_{p'} D·B[slot_{p'}]·G[p'][q] − δ_end·dm
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
        // ヤコビアンが特異な場合は補正なしの弾性剛性 K_el を接線として返す
        // （内力は履歴に整合した厳密値のため収束解は変わらない。反復回数が
        // 増えるだけで、誤った接線を混入させるより安全）。
        let Some(jinv) = crate::linalg::invert_small(&jac[..n * n], n) else {
            return LocalMat {
                n: 12,
                data: h.k_el.data.clone(),
            };
        };
        // ∂f/∂x[q] = Σ_{p'} K_el[:, slot_{p'}]·G[p'][q]（12×n、n≤4 → 12*n≤48）。
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
        // ∂R/∂u[p] = D·B 行（12）
        // K* = K_el − fx·J⁻¹·(D·B 行)
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
        // 対称化（doc コメント参照）。
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
        // 組立順は弾性梁（4.1.4）と同じ「可撓長で要素剛性 → 材端解放の静縮約 →
        // 剛体アームで節点自由度へ → 全体座標変換」。
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
        // 可撓端の変位（剛体アームで節点変位から写す。剛域なしでは節点変位そのもの）と、
        // 解放端では内部自由度で置き換えた要素変形。
        let u_flex = self.flex_disp();
        let u_elem = self.elem_disp(&u_flex);
        let f_elem = self.elem_internal_force(&u_elem);

        // 解放端の節点回転が受け持つのは回転ばねのモーメントのみ（ピンは 0）。
        // それ以外の自由度は要素内力がそのまま可撓端の内力になる。
        let mut f_flex = f_elem;
        for rel in &self.releases {
            f_flex[rel.dof] = rel.spring * (u_flex[rel.dof] - u_elem[rel.dof]);
        }

        // 可撓端の内力 → 節点自由度（剛体アームのモーメント寄与を含む）→ グローバル系。
        let f_node = crate::frame::rigid_arm::to_node_force(&f_flex, self.rigid_i, self.rigid_j);
        let f_global = self.axis.rotate_to_global(&f_node);
        LocalVec {
            data: SmallVec::from_slice(&f_global),
        }
    }

    /// 現在のファイバー状態から部材内力分布を返す。
    ///
    /// 端部節点力は [`Self::internal_force`]（各積分点の断面応答＝ファイバーの
    /// 履歴状態から算定した復元力）であり、接線剛性 × 全変位ではないため
    /// 降伏後も正しい。これを釣合いで材軸方向へ分配する。
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
        // 入力 du はグローバル系。内部状態（trial_disp, B行列ひずみ）はローカル系で
        // 扱うため、まずローカル系へ回転してから累積する。
        let du_global: [f64; 12] = std::array::from_fn(|i| du.data[i]);
        let du_local = self.axis.rotate_to_local(&du_global);
        for i in 0..12 {
            self.trial_disp[i] += du_local[i];
        }
        if self.flex_length <= 0.0 {
            return;
        }
        // 材端解放がある場合は内部自由度を内部釣合いへ収束させる。断面のトライアル
        // 状態はこの中で最終の要素変形と整合するよう更新される。
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
                // ねじれ項は 0。弾性梁は ρ·J·l/6 を持つが、ファイバー梁は断面を
                // ファイバーへ分割する定式化上 J を保持しておらず、従来から
                // 部材軸まわりの回転慣性を持たない。
                let mm = crate::frame::prismatic::consistent_mass(total_mass, self.length, 0.0);
                // 整合質量は回転不変ではないため全体系へ回す（beam と同じ契約）。
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
        // 旧形式のチェックポイントも読めるようにする（V2: 塑性増分ヒンジ状態なし、
        // Legacy: 材端解放の内部自由度もなし。いずれも欠落分はゼロ初期化で復元する）。
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

    /// 塑性率評価用の危険断面プローブ（構造力学のファイバーモデル）。
    /// 現在の `trial_disp`（ローカル系）から各ガウス点の曲率を復元し、曲率が
    /// 最大のガウス点（危険断面）についてファイバーひずみを集約する。
    fn ductility_probe(&self) -> Option<DuctilityProbe> {
        let l = self.flex_length;
        if l <= 0.0 || self.gauss_points.is_empty() {
            return None;
        }
        // 曲率は要素変形から復元する（剛域は剛体アーム、解放端は内部自由度）。
        let td = self.elem_disp(&self.flex_disp());
        // 曲率が最大のガウス点（危険断面）を選ぶ。
        let mut best: Option<(f64, usize, f64, f64, f64)> = None; // (|κ|, idx, eps0, ky, kz)
        if let Some(h) = &self.hinge {
            // 塑性増分ヒンジモデル: 断面曲率は内部平衡の解（trial_kappa）そのもの。
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

    /// ヒンジ詳細表示用: 各ガウス点断面のファイバー状態（位置・ひずみ・降伏比）。
    /// 断面の一般化ひずみ（軸ひずみ・曲率）の復元規則は [`Self::ductility_probe`] と
    /// 同じ（塑性増分ヒンジモデルは内部平衡の解 trial_kappa、通常のガウス点は
    /// B マトリクス）で、こちらは危険断面 1 点ではなく全ガウス点・全ファイバーを返す。
    fn fiber_section_states(&self) -> Option<Vec<crate::behavior::FiberSectionState>> {
        use crate::behavior::{FiberSectionState, FiberStateSample};
        let l = self.flex_length;
        if l <= 0.0 || self.gauss_points.is_empty() {
            return None;
        }
        let td = self.elem_disp(&self.flex_disp());
        let eps0_hinge = (td[6] - td[0]) / l;
        // 塑性増分ヒンジモデルは端部 2 断面のみ曲率を持つ。
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
