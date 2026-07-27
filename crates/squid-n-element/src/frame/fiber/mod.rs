use crate::behavior::{
    Ctx, DuctilityProbe, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption,
};
use smallvec::SmallVec;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::NodeId;
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape};
use squid_n_material::uniaxial::{Bilinear, UniaxialMaterial};
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

/// ガウス点のファイバー断面と材料を構築する（構造力学のファイバーモデル）。
/// RC 断面（RcRect/RcCircle）はコンクリートファイバー格子に加え、主筋を点ファイバー
/// （バイリニア鋼材）として**分離**して配置する（従来は均質コンクリート断面で
/// 引張側鉄筋を無視していた）。それ以外（鋼材・複合断面）は均質格子とする。
/// `fc≤60` はコンクリートに NewRC、超過は放物線モデルを用いる。
#[allow(clippy::too_many_arguments)]
fn build_gauss_fibers(
    width: f64,
    depth: f64,
    nw: usize,
    nd: usize,
    shape: Option<&SectionShape>,
    fc: Option<f64>,
    e: f64,
    fy: Option<f64>,
    steel_factor: f64,
    rebar_factor: f64,
) -> (FiberSection, Vec<Box<dyn UniaxialMaterial>>) {
    // 基本格子（コンクリート or 鋼材）。保有水平耐力計算時は鋼材文脈の材料強度
    // 割増（steel_factor）を fy に乗じる（時刻歴応答解析等は steel_factor=1.0）。
    let base: Box<dyn UniaxialMaterial> = match fc {
        Some(fc) if fc <= 60.0 => Box::new(squid_n_material::ConcreteNewRc::new(fc, 2.0)),
        Some(fc) => Box::new(squid_n_material::uniaxial::Concrete::new(fc, 2.0)),
        None => Box::new(Bilinear::new(e, fy.unwrap_or(1e20) * steel_factor, 0.01)),
    };
    let grid = squid_n_section::fiber::rect_fiber_section(width, depth, nw, nd, 0);
    let mut fibers = grid.fibers;
    let mut mats: Vec<Box<dyn UniaxialMaterial>> =
        (0..fibers.len()).map(|_| base.clone_box()).collect();

    // RC 断面: 主筋を点ファイバー（バイリニア鋼材、fy 既定 SD345=345）として追加。
    // 保有水平耐力計算時は主筋の材料強度割増（rebar_factor）を乗じる
    // （時刻歴応答解析等は rebar_factor=1.0）。
    if fc.is_some() {
        let rebar_fy = fy.unwrap_or(345.0) * rebar_factor;
        let rebar_e = 205000.0;
        match shape {
            Some(SectionShape::RcRect { rebar, b, d }) => {
                add_rebar_fibers_rect(&mut fibers, &mut mats, rebar, *b, *d, rebar_e, rebar_fy);
            }
            Some(SectionShape::RcCircle { rebar, d }) => {
                add_rebar_fibers_circle(&mut fibers, &mut mats, rebar, *d, rebar_e, rebar_fy);
            }
            _ => {}
        }
    }

    // `rect_fiber_section`（および主筋配置）の座標規約は y=幅方向・z=せい方向だが、
    // 要素座標系はせい方向＝ローカル y（LocalFrame: ey=ref_vector 直交化）のため、
    // x 軸まわりの 90° 回転 (y,z)←(z,−y) で並べ替え、強軸曲げ（せい方向の応力勾配）が
    // Mz 面（κz・∫y²dA、(uy,rz) ブロック）に対応するようにする。
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

/// 矩形 RC 断面の主筋点ファイバーを追加する（`mn_surface::rebar_fibers_rect` と同じ
/// 配置規則: せい方向主筋 main_x を上下面へ、幅方向主筋 main_y を側面内分点へ）。
/// 座標系は `rect_fiber_section` と同じ（y=幅方向、z=せい方向。強軸曲げは z）。
fn add_rebar_fibers_rect(
    fibers: &mut Vec<Fiber>,
    mats: &mut Vec<Box<dyn UniaxialMaterial>>,
    rebar: &RcRebar,
    b: f64,
    d: f64,
    e: f64,
    fy: f64,
) {
    let bar_area = |set: &BarSet| std::f64::consts::PI * set.dia * set.dia / 4.0;
    let push = |y: f64,
                z: f64,
                a: f64,
                mats: &mut Vec<Box<dyn UniaxialMaterial>>,
                fibers: &mut Vec<Fiber>| {
        fibers.push(Fiber {
            y,
            z,
            area: a,
            material: 1,
        });
        mats.push(Box::new(Bilinear::new(e, fy, 0.01)));
    };
    // せい方向主筋（上下面）。
    let set = &rebar.main_x;
    if set.count > 0 {
        let a = bar_area(set);
        for layer in 0..set.layers.max(1) {
            let z0 = d / 2.0 - rebar.cover - layer as f64 * 2.5 * set.dia;
            let span = b - 2.0 * rebar.cover;
            for i in 0..set.count {
                let y = if set.count == 1 {
                    0.0
                } else {
                    -span / 2.0 + span * i as f64 / (set.count - 1) as f64
                };
                for zsign in [1.0, -1.0] {
                    push(y, zsign * z0, a, mats, fibers);
                }
            }
        }
    }
    // 幅方向主筋（側面内分点）。
    let set = &rebar.main_y;
    if set.count > 0 {
        let a = bar_area(set);
        for layer in 0..set.layers.max(1) {
            let y0 = b / 2.0 - rebar.cover - layer as f64 * 2.5 * set.dia;
            let span = d - 2.0 * rebar.cover;
            for i in 0..set.count {
                let z = -span / 2.0 + span * (i as f64 + 1.0) / (set.count + 1) as f64;
                for ysign in [1.0, -1.0] {
                    push(ysign * y0, z, a, mats, fibers);
                }
            }
        }
    }
}

/// 円形 RC 断面の主筋点ファイバーを追加する（main_x+main_y の合計本数を円周へ等配）。
fn add_rebar_fibers_circle(
    fibers: &mut Vec<Fiber>,
    mats: &mut Vec<Box<dyn UniaxialMaterial>>,
    rebar: &RcRebar,
    d: f64,
    e: f64,
    fy: f64,
) {
    let total = (rebar.main_x.count + rebar.main_y.count) as usize;
    if total == 0 {
        return;
    }
    let dia = if rebar.main_x.count > 0 {
        rebar.main_x.dia
    } else {
        rebar.main_y.dia
    };
    let a = std::f64::consts::PI * dia * dia / 4.0;
    let r = d / 2.0 - rebar.cover;
    for i in 0..total {
        let th = 2.0 * std::f64::consts::PI * i as f64 / total as f64;
        fibers.push(Fiber {
            y: r * th.cos(),
            z: r * th.sin(),
            area: a,
            material: 1,
        });
        mats.push(Box::new(Bilinear::new(e, fy, 0.01)));
    }
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

/// 端条件から解放する回転自由度を決める。
///
/// `EndCondition::Fixed` は解放しない。ピン・半剛は当該端の rx/ry/rz を解放する。
/// ただし **ねじり剛性が無い部材（J≤0）の rx は解放しない**。解放しても縮約行列
/// `Kbb` の対角がゼロになり特異化するだけで、モーメント解放としての意味がないため。
fn resolve_end_releases(
    end_cond: &[squid_n_core::model::EndCondition; 2],
    has_torsion: bool,
) -> SmallVec<[EndRelease; 6]> {
    use squid_n_core::model::EndCondition;
    const ROT_DOFS: [(usize, usize); 6] = [(3, 0), (4, 0), (5, 0), (9, 1), (10, 1), (11, 1)];
    let mut out = SmallVec::new();
    for &(dof, end) in ROT_DOFS.iter() {
        let spring = match end_cond[end] {
            EndCondition::Fixed => continue,
            EndCondition::Pinned => 0.0,
            EndCondition::SemiRigid { k_theta } => k_theta,
        };
        if (dof == 3 || dof == 9) && !has_torsion {
            continue;
        }
        out.push(EndRelease { dof, spring });
    }
    out
}

/// [`FiberBeam::snapshot_state`] が返すスナップショットの型。
/// （トライアル変位・確定変位・各ガウス点の材料・内部自由度のトライアル/確定値）
pub type FiberBeamSnapshot = (
    [f64; 12],
    [f64; 12],
    Vec<Vec<Box<dyn UniaxialMaterial>>>,
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
}

/// 材端解放の内部自由度を持たない旧形式のチェックポイント（読み込み互換用）。
#[derive(serde::Deserialize)]
struct FiberBeamCheckpointLegacy {
    trial_disp: [f64; 12],
    committed_disp: [f64; 12],
    gauss_points: Vec<Vec<Vec<u8>>>,
}

pub struct GaussPoint {
    pub xi: f64,
    pub weight: f64,
    pub section: FiberSection,
    pub mats: Vec<Box<dyn UniaxialMaterial>>,
    pub trial_stress: Vec<f64>,
    pub trial_et: Vec<f64>,
}

impl GaussPoint {
    pub fn new(
        xi: f64,
        weight: f64,
        section: FiberSection,
        mut mats: Vec<Box<dyn UniaxialMaterial>>,
    ) -> Self {
        let n = section.fibers.len();
        // 接線キャッシュを各ファイバの初期弾性接線で初期化する。
        // 未初期化（0）のままだと、最初の update_state より前に tangent_stiffness を
        // 呼ぶ経路（pushover の初回 assemble_k）で剛性が 0 になり特異化する。
        let trial_et: Vec<f64> = mats.iter_mut().map(|m| m.trial(0.0).1).collect();
        GaussPoint {
            xi,
            weight,
            section,
            mats,
            trial_stress: vec![0.0; n],
            trial_et,
        }
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
/// 自由度へ写す（[`crate::rigid_arm`]。弾性梁 `BeamElement` と同じ変換）。
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
    /// 塑性化域考慮モデルの中央弾性部剛性（ローカル系 12×12）。
    /// None = 従来の全長ファイバー積分モデル。
    pub k_mid: Option<LocalMat>,
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
    ) -> Self {
        let n0 = &model.nodes[data.nodes[0].index()];
        let n1 = &model.nodes[data.nodes[1].index()];
        let dx = n1.coord[0] - n0.coord[0];
        let dy = n1.coord[1] - n0.coord[1];
        let dz = n1.coord[2] - n0.coord[2];
        let length = (dx * dx + dy * dy + dz * dz).sqrt();
        // 剛域長と可撓長。断面積分・B 行列・せん断・幾何剛性はすべて可撓長基準で
        // 組み、可撓端自由度を剛体アームで節点自由度へ写す（弾性梁と同じ扱い）。
        let (rigid_i, rigid_j) = crate::rigid_arm::resolve_lengths(
            data.rigid_zone.length_i,
            data.rigid_zone.length_j,
            length,
        );
        let flex_length = length - rigid_i - rigid_j;

        let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
        let mat_ref = data
            .material
            .and_then(|mid| model.materials.get(mid.index()));
        let density = mat_ref.map(|m| m.density).unwrap_or(0.0);
        let e = mat_ref.map(|m| m.young).unwrap_or(205000.0);
        let g = mat_ref.map(|m| m.shear_modulus()).unwrap_or(78846.0);
        let width = sec.map(|s| s.width).unwrap_or(100.0);
        let depth = sec.map(|s| s.depth).unwrap_or(200.0);
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
        let shape = sec.and_then(|s| s.shape.as_ref());
        let fc = mat_ref.and_then(|m| m.fc);
        let fy = mat_ref.and_then(|m| m.fy);
        // 保有水平耐力計算（basis==MaterialStrength）時のみ材料強度割増を適用する
        // （鋼材文脈・RC 主筋文脈で係数が異なる。せん断補強筋は割増対象外）。
        let steel_factor = basis.steel_factor(mat_ref);
        let rebar_factor = basis.rebar_factor(mat_ref);
        // RC 断面はコンクリート格子＋主筋分離（構造力学のファイバーモデル）。
        let (sec_a, mats_a) = build_gauss_fibers(
            width,
            depth,
            nw,
            nd,
            shape,
            fc,
            e,
            fy,
            steel_factor,
            rebar_factor,
        );
        let (sec_b, mats_b) = build_gauss_fibers(
            width,
            depth,
            nw,
            nd,
            shape,
            fc,
            e,
            fy,
            steel_factor,
            rebar_factor,
        );
        let gauss_points = vec![
            GaussPoint::new(-0.5773502691896257, 1.0, sec_a, mats_a),
            GaussPoint::new(0.5773502691896257, 1.0, sec_b, mats_b),
        ];

        let axis = crate::transform::LocalFrame::from_nodes(
            n0.coord,
            n1.coord,
            data.local_axis.ref_vector,
        );

        // 材端解放（ピン・半剛）。ねじり剛性が無い部材の rx は解放しない。
        let releases = resolve_end_releases(&data.end_cond, torsion_j > 0.0 && g > 0.0);
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
            k_mid: None,
            eval_sections: crate::beam::eval_sections_of(data, model, length),
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

    /// 塑性化域考慮のファイバー要素（材端剛塑性ばねモデルと適合する
    /// ファイバーモデル化）。端部の塑性化領域（長さ `lp`）にファイバー断面を
    /// 配置（積分点 ξ=∓1、重み Lp）し、中央 [Lp, L'−Lp] は断面諸元
    /// （EA・EIy・EIz）による弾性剛性として厳密に B 積分する。
    /// 剛域があるときの基準長 L' は可撓長（積分点は剛域フェイス）。
    /// 塑性化域考慮のファイバー要素の生成（材料強度の基準 `basis` を明示指定する版）。
    pub fn with_plastic_zone(
        data: &squid_n_core::model::ElementData,
        model: &squid_n_core::model::Model,
        lp: f64,
        basis: crate::factory::StrengthBasis,
    ) -> Self {
        Self::build_plastic_zone(data, model, lp, 12, 20, basis)
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
    ) -> Self {
        let mut fb = Self::new(data, model, basis);
        // 基準長は可撓長（剛域がなければ節点間長に等しい）。積分点 ξ=∓1 は
        // 剛域フェイス、塑性化域 Lp も剛域フェイスから測る。
        let l = fb.flex_length;
        if l <= 0.0 {
            return fb;
        }
        // Lp は可撓長の 45% までにクランプ（両端合計で可撓長を超えない）
        let lp = clamp_plastic_zone(lp, l);

        let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
        let mat_ref = data
            .material
            .and_then(|mid| model.materials.get(mid.index()));
        let e = mat_ref.map(|m| m.young).unwrap_or(205000.0);
        let width = sec.map(|s| s.width).unwrap_or(100.0);
        let depth = sec.map(|s| s.depth).unwrap_or(200.0);
        let area = sec.map(|s| s.area).unwrap_or(width * depth);
        // 断面レイヤ→要素座標系のクロス変換（beam/construct.rs と同一規約）。
        // 断面 iy（強軸）は要素座標系では z 軸まわり（Mz 面）＝EIz へ、
        // 断面 iz（弱軸）は y 軸まわり（My 面）＝EIy へ対応する。
        let iy = sec.map(|s| s.iz).unwrap_or(1.0);
        let iz = sec.map(|s| s.iy).unwrap_or(1.0);

        // 端部積分点: ξ=∓1、重み w·(L/2) = Lp → w = 2Lp/L
        let w_end = 2.0 * lp / l;
        let shape = sec.and_then(|s| s.shape.as_ref());
        let fc = mat_ref.and_then(|m| m.fc);
        let fy = mat_ref.and_then(|m| m.fy);
        // 保有水平耐力計算（basis==MaterialStrength）時のみ材料強度割増を適用する。
        let steel_factor = basis.steel_factor(mat_ref);
        let rebar_factor = basis.rebar_factor(mat_ref);
        // RC 断面はコンクリート格子＋主筋分離（構造力学のファイバーモデル）。
        let (sec_a, mats_a) = build_gauss_fibers(
            width,
            depth,
            nw,
            nd,
            shape,
            fc,
            e,
            fy,
            steel_factor,
            rebar_factor,
        );
        let (sec_b, mats_b) = build_gauss_fibers(
            width,
            depth,
            nw,
            nd,
            shape,
            fc,
            e,
            fy,
            steel_factor,
            rebar_factor,
        );
        fb.gauss_points = vec![
            GaussPoint::new(-1.0, w_end, sec_a, mats_a),
            GaussPoint::new(1.0, w_end, sec_b, mats_b),
        ];

        // 中央弾性部 [Lp, L'−Lp] の剛性: B(ξ)ᵀ·diag(EA,EIy,EIz)·B(ξ) を
        // 2点 Gauss（区間 [−h, h]、h = 1−2Lp/L'）で厳密積分（被積分関数は ξ の2次）
        let h = 1.0 - 2.0 * lp / l;
        let d_el = [e * area, e * iy, e * iz];
        let mut k_mid = LocalMat::zeros(12);
        for sgn in [-1.0, 1.0] {
            let xi = sgn * h / 3.0_f64.sqrt();
            let w_phys = h * l / 2.0;
            let b = Self::compute_b_matrix(xi, l, fb.phi_y, fb.phi_z);
            for i in 0..12 {
                for j in 0..12 {
                    let mut val = 0.0;
                    for (p, dp) in d_el.iter().enumerate() {
                        val += b[p][i] * dp * b[p][j];
                    }
                    if val != 0.0 {
                        k_mid.set(i, j, k_mid.get(i, j) + val * w_phys);
                    }
                }
            }
        }
        fb.k_mid = Some(k_mid);
        fb
    }

    /// 現在のトライアル変位（ローカル系・節点自由度）を可撓端自由度へ写した値。
    ///
    /// 断面ひずみ・中央弾性部・せん断ひずみ場はいずれも可撓部の変形で決まるため、
    /// これらの評価には節点変位ではなく可撓端変位を用いる。剛域がなければ
    /// `trial_disp` と一致する。
    pub fn flex_disp(&self) -> [f64; 12] {
        crate::rigid_arm::to_flex_disp(&self.trial_disp, self.rigid_i, self.rigid_j)
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
            let b = Self::compute_b_matrix(gp.xi, l, self.phi_y, self.phi_z);
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
        }
    }

    /// 可撓端系 12×12 の接線剛性（剛体アーム変換・材端解放の縮約より前）。
    /// 断面積分＋中央弾性部＋一定せん断ひずみ場＋ねじりを含む。
    fn elem_tangent(&self) -> LocalMat {
        let mut k = LocalMat::zeros(12);
        let l = self.flex_length;
        if l <= 0.0 {
            return k;
        }
        let half = l / 2.0;

        for gp in &self.gauss_points {
            let (_, d) = Self::section_response_from_cache(gp);
            let w = gp.weight * half;
            let b = Self::compute_b_matrix(gp.xi, l, self.phi_y, self.phi_z);

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

        // 塑性化域考慮モデル: 中央弾性部の剛性を加算
        if let Some(km) = &self.k_mid {
            for i in 0..12 {
                for j in 0..12 {
                    let old = k.get(i, j);
                    k.set(i, j, old + km.get(i, j));
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
        let half = l / 2.0;

        for gp in &self.gauss_points {
            let (force, _) = Self::section_response_from_cache(gp);
            let w = gp.weight * half;
            let b = Self::compute_b_matrix(gp.xi, l, self.phi_y, self.phi_z);
            for (i, fi) in f.iter_mut().enumerate() {
                *fi += (b[0][i] * force[0] + b[1][i] * force[1] + b[2][i] * force[2]) * w;
            }
        }

        // 中央弾性部（線形: K_mid·u）とせん断ひずみ場（線形弾性: K_shear·u）。
        // γ は剛体運動でゼロの客観的測度なので偽内力は生じない。
        for i in 0..12 {
            let mut si = 0.0;
            for j in 0..12 {
                if let Some(km) = &self.k_mid {
                    si += km.get(i, j) * u_elem[j];
                }
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
            self.update_section_trial(&u_elem);
            return;
        }
        let n = self.releases.len();
        for _ in 0..MAX_ITER {
            let u_elem = self.elem_disp(&u_flex);
            self.update_section_trial(&u_elem);
            let f_elem = self.elem_internal_force(&u_elem);

            let mut r = vec![0.0_f64; n];
            for (k, rel) in self.releases.iter().enumerate() {
                r[k] = f_elem[rel.dof] + rel.spring * (u_elem[rel.dof] - u_flex[rel.dof]);
            }
            // 収束判定は要素の回転自由度内力のスケール基準（残差はモーメント [N·mm]）。
            let scale = [3usize, 4, 5, 9, 10, 11]
                .iter()
                .map(|&i| f_elem[i].abs())
                .fold(1.0_f64, f64::max);
            if r.iter().all(|v| v.abs() <= 1e-10 * scale) {
                return;
            }

            let k_elem = self.elem_tangent();
            let mut kbb = vec![0.0_f64; n * n];
            for (a, ra) in self.releases.iter().enumerate() {
                for (b, rb) in self.releases.iter().enumerate() {
                    kbb[a * n + b] = k_elem.get(ra.dof, rb.dof);
                }
                kbb[a * n + a] += ra.spring;
            }
            let kbb_inv = super::beam::invert_small(&kbb, n);
            let mut du = vec![0.0_f64; n];
            for (a, dua) in du.iter_mut().enumerate() {
                let mut s = 0.0;
                for (b, rb) in r.iter().enumerate() {
                    s += kbb_inv[a * n + b] * rb;
                }
                *dua = -s;
            }
            // 数値異常（縮約行列が特異）なら更新を打ち切り、直前の状態を保つ。
            if du.iter().any(|v| !v.is_finite()) {
                break;
            }
            for (k, d) in du.iter().enumerate() {
                self.trial_int[k] += d;
            }
        }
        // 収束打ち切り時も断面状態を最終 u_elem と整合させる。
        let u_elem = self.elem_disp(&u_flex);
        self.update_section_trial(&u_elem);
    }

    /// 材端解放した要素端回転を静縮約し、可撓端系 12×12 へ戻す
    /// （K* = Kaa − Kab·Kbb⁻¹·Kba。弾性梁 `condense_end_springs` と同じ定式化）。
    fn condense_releases(&self, k_elem: &LocalMat) -> LocalMat {
        if self.releases.is_empty() {
            return LocalMat {
                n: 12,
                data: k_elem.data.clone(),
            };
        }
        let nb = self.releases.len();
        let n = 12 + nb;
        let mut k = vec![0.0_f64; n * n];
        // 解放回転は内部（12..）へ、それ以外は同位置へ写す。
        let mut map = [0usize; 12];
        for (i, m) in map.iter_mut().enumerate() {
            *m = i;
        }
        for (idx, rel) in self.releases.iter().enumerate() {
            map[rel.dof] = 12 + idx;
        }
        for i in 0..12 {
            for j in 0..12 {
                k[map[i] * n + map[j]] += k_elem.get(i, j);
            }
        }
        // 回転ばね: 節点回転 dof ↔ 内部の要素端回転 (12+idx)
        for (idx, rel) in self.releases.iter().enumerate() {
            let (r, ir, ks) = (rel.dof, 12 + idx, rel.spring);
            k[r * n + r] += ks;
            k[ir * n + ir] += ks;
            k[r * n + ir] -= ks;
            k[ir * n + r] -= ks;
        }

        let na = 12;
        let mut kbb = vec![0.0_f64; nb * nb];
        for i in 0..nb {
            for j in 0..nb {
                kbb[i * nb + j] = k[(na + i) * n + (na + j)];
            }
        }
        let kbb_inv = super::beam::invert_small(&kbb, nb);
        let mut kstar = LocalMat::zeros(na);
        for i in 0..na {
            for j in 0..na {
                let mut s = k[i * n + j];
                for a in 0..nb {
                    for b in 0..nb {
                        s -= k[i * n + (na + a)] * kbb_inv[a * nb + b] * k[(na + b) * n + j];
                    }
                }
                kstar.set(i, j, s);
            }
        }
        kstar
    }

    fn beam_global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &n in &self.nodes {
            let ni = n.index();
            for d in 0..6 {
                let g = ni * 6 + d;
                gdofs.push(dof.active(g).map(|a| a as usize).unwrap_or(usize::MAX));
            }
        }
        gdofs
    }

    fn section_response_from_cache(gp: &GaussPoint) -> ([f64; 3], [[f64; 3]; 3]) {
        let mut force = [0.0; 3];
        let mut stiff = [[0.0; 3]; 3];
        for (i, fiber) in gp.section.fibers.iter().enumerate() {
            let a = fiber.area;
            let sigma = gp.trial_stress[i];
            let et = gp.trial_et[i];
            force[0] += sigma * a;
            force[1] += sigma * a * fiber.z;
            force[2] += -sigma * a * fiber.y;
            stiff[0][0] += et * a;
            stiff[0][1] += et * a * fiber.z;
            stiff[0][2] += -et * a * fiber.y;
            stiff[1][1] += et * a * fiber.z * fiber.z;
            stiff[1][2] += -et * a * fiber.y * fiber.z;
            stiff[2][2] += et * a * fiber.y * fiber.y;
        }
        stiff[1][0] = stiff[0][1];
        stiff[2][0] = stiff[0][2];
        stiff[2][1] = stiff[1][2];
        (force, stiff)
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
}

impl ElementBehavior for FiberBeam {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        self.beam_global_dofs(dof)
    }

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        if self.flex_length <= 0.0 {
            return LocalMat::zeros(12);
        }
        // 組立順は弾性梁（4.1.4）と同じ「可撓長で要素剛性 → 材端解放の静縮約 →
        // 剛体アームで節点自由度へ → 全体座標変換」。
        let k_elem = self.elem_tangent();
        let k_end = self.condense_releases(&k_elem);
        let k_node = crate::rigid_arm::transform_stiffness(&k_end, self.rigid_i, self.rigid_j);
        self.axis.to_global(&k_node)
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
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
        let f_node = crate::rigid_arm::to_node_force(&f_flex, self.rigid_i, self.rigid_j);
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
    fn state_member_forces(
        &self,
        state: &ElemState,
        ctx: &Ctx,
    ) -> Option<crate::beam::MemberForces> {
        let f_global = self.internal_force(state, ctx);
        let arr: [f64; 12] = std::array::from_fn(|i| f_global.data[i]);
        let f_local = self.axis.rotate_to_local(&arr);
        Some(crate::beam::member_forces_from_end_forces(
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
        }
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        let total_area: f64 = self
            .gauss_points
            .first()
            .map(|gp| gp.section.fibers.iter().map(|f| f.area).sum())
            .unwrap_or(0.0);
        let total_mass = self.density * total_area * self.length;
        let mut mm = LocalMat::zeros(12);
        match opt {
            MassOption::Lumped => {
                for d in [0, 1, 2, 6, 7, 8] {
                    mm.set(d, d, total_mass / 2.0);
                }
            }
            MassOption::Consistent => {
                let c1 = total_mass / 6.0;
                let c2 = total_mass / 420.0;
                let l = self.length;
                let l2 = l * l;
                mm.set(0, 0, 2.0 * c1);
                mm.set(0, 6, 1.0 * c1);
                mm.set(6, 0, 1.0 * c1);
                mm.set(6, 6, 2.0 * c1);
                let b4 = |mm: &mut LocalMat, i0: usize, j0: usize, sign: f64| {
                    mm.set(i0, j0, 156.0 * c2);
                    mm.set(i0, j0 + 1, 22.0 * l * c2 * sign);
                    mm.set(i0, j0 + 2, 54.0 * c2);
                    mm.set(i0, j0 + 3, -13.0 * l * c2 * sign);
                    mm.set(i0 + 1, j0, 22.0 * l * c2 * sign);
                    mm.set(i0 + 1, j0 + 1, 4.0 * l2 * c2);
                    mm.set(i0 + 1, j0 + 2, 13.0 * l * c2 * sign);
                    mm.set(i0 + 1, j0 + 3, -3.0 * l2 * c2);
                    mm.set(i0 + 2, j0, 54.0 * c2);
                    mm.set(i0 + 2, j0 + 1, 13.0 * l * c2 * sign);
                    mm.set(i0 + 2, j0 + 2, 156.0 * c2);
                    mm.set(i0 + 2, j0 + 3, -22.0 * l * c2 * sign);
                    mm.set(i0 + 3, j0, -13.0 * l * c2 * sign);
                    mm.set(i0 + 3, j0 + 1, -3.0 * l2 * c2);
                    mm.set(i0 + 3, j0 + 2, -22.0 * l * c2 * sign);
                    mm.set(i0 + 3, j0 + 3, 4.0 * l2 * c2);
                };
                b4(&mut mm, 1, 1, 1.0);
                b4(&mut mm, 2, 2, -1.0);
            }
        }
        mm
    }

    fn geometric_stiffness(&self, n: f64) -> LocalMat {
        // 幾何剛性も弾性剛性と整合させる: 可撓長で組み、剛体アームで節点自由度へ写す
        // （剛域があれば P-δ は可撓部でのみ生じる。弾性梁と同じ扱い）。
        let l = self.flex_length;
        if l < 1e-12 {
            return LocalMat::zeros(12);
        }
        let c = n / l;
        let mut kg = LocalMat::zeros(12);
        let mut s = |i: usize, j: usize, v: f64| {
            kg.set(i, j, v);
            if i != j {
                kg.set(j, i, v);
            }
        };
        s(1, 1, c * 6.0 / 5.0);
        s(7, 7, c * 6.0 / 5.0);
        s(1, 7, -c * 6.0 / 5.0);
        s(1, 5, c * l / 10.0);
        s(1, 11, c * l / 10.0);
        s(5, 7, -c * l / 10.0);
        s(7, 11, -c * l / 10.0);
        s(5, 5, c * 2.0 * l * l / 15.0);
        s(11, 11, c * 2.0 * l * l / 15.0);
        s(5, 11, -c * l * l / 30.0);
        s(2, 2, c * 6.0 / 5.0);
        s(8, 8, c * 6.0 / 5.0);
        s(2, 8, -c * 6.0 / 5.0);
        s(2, 4, -c * l / 10.0);
        s(2, 10, -c * l / 10.0);
        s(4, 8, c * l / 10.0);
        s(8, 10, c * l / 10.0);
        s(4, 4, c * 2.0 * l * l / 15.0);
        s(10, 10, c * 2.0 * l * l / 15.0);
        s(4, 10, -c * l * l / 30.0);
        // 剛体アーム変換 → グローバル系へ回転
        let kg_node = crate::rigid_arm::transform_stiffness(&kg, self.rigid_i, self.rigid_j);
        self.axis.to_global(&kg_node)
    }

    fn snapshot_state(&self) -> Box<dyn Any> {
        let gauss_data: Vec<Vec<Box<dyn UniaxialMaterial>>> = self
            .gauss_points
            .iter()
            .map(|gp| gp.mats.iter().map(|m| m.clone_box()).collect())
            .collect();
        Box::new((
            self.trial_disp,
            self.committed_disp,
            gauss_data,
            self.trial_int.to_vec(),
            self.committed_int.to_vec(),
        ))
    }

    fn restore_state(&mut self, state: &dyn Any) {
        if let Some((trial, committed, mats_data, trial_int, committed_int)) =
            state.downcast_ref::<FiberBeamSnapshot>()
        {
            self.trial_disp = *trial;
            self.committed_disp = *committed;
            for (gp, gp_mats) in self.gauss_points.iter_mut().zip(mats_data) {
                for (mat, new_mat) in gp.mats.iter_mut().zip(gp_mats) {
                    *mat = new_mat.clone_box();
                }
            }
            self.trial_int = SmallVec::from_slice(trial_int);
            self.committed_int = SmallVec::from_slice(committed_int);
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
    }

    fn revert_state(&mut self) {
        for gp in &mut self.gauss_points {
            for mat in &mut gp.mats {
                mat.revert();
            }
        }
        self.trial_disp = self.committed_disp;
        self.trial_int = self.committed_int.clone();
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
        let cp = FiberBeamCheckpoint {
            trial_disp: self.trial_disp,
            committed_disp: self.committed_disp,
            gauss_points,
            trial_int: self.trial_int.to_vec(),
            committed_int: self.committed_int.to_vec(),
        };
        bincode::serialize(&cp).expect("serialize checkpoint")
    }

    fn deserialize_checkpoint(
        &mut self,
        data: &[u8],
    ) -> Result<(), crate::behavior::CheckpointError> {
        // 材端解放の内部自由度を含まない旧形式のチェックポイントも読めるようにする
        // （内部自由度はゼロ＝解放なしの状態として復元する）。現行形式は末尾に
        // 2 つの Vec<f64> を持つため、旧形式のバイト列は現行形式としては読めない。
        let cp = match bincode::deserialize::<FiberBeamCheckpoint>(data) {
            Ok(cp) => cp,
            Err(_) => {
                let legacy: FiberBeamCheckpointLegacy = bincode::deserialize(data)
                    .map_err(|e| crate::behavior::CheckpointError::Decode(e.to_string()))?;
                FiberBeamCheckpoint {
                    trial_disp: legacy.trial_disp,
                    committed_disp: legacy.committed_disp,
                    gauss_points: legacy.gauss_points,
                    trial_int: vec![0.0; self.releases.len()],
                    committed_int: vec![0.0; self.releases.len()],
                }
            }
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
        for (gi, gp) in self.gauss_points.iter().enumerate() {
            let b = Self::compute_b_matrix(gp.xi, l, self.phi_y, self.phi_z);
            let eps0 = b[0][0] * td[0] + b[0][6] * td[6];
            let ky = b[1][2] * td[2] + b[1][4] * td[4] + b[1][8] * td[8] + b[1][10] * td[10];
            let kz = b[2][1] * td[1] + b[2][5] * td[5] + b[2][7] * td[7] + b[2][11] * td[11];
            let kappa = (ky * ky + kz * kz).sqrt();
            if best.is_none_or(|(bk, ..)| kappa > bk) {
                best = Some((kappa, gi, eps0, ky, kz));
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

    fn set_concrete_hysteresis(&mut self, dynamic: bool) {
        for gp in &mut self.gauss_points {
            for mat in &mut gp.mats {
                mat.set_concrete_hysteresis(dynamic);
            }
        }
    }
}

#[cfg(test)]
mod tests;
