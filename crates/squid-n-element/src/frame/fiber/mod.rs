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

        FiberBeam {
            length,
            rigid_i,
            rigid_j,
            flex_length,
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
        let mut k = LocalMat::zeros(12);
        // 断面積分・せん断・中央弾性部は可撓長基準（剛域があれば節点間長より短い）。
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

        // 可撓端自由度 → 節点自由度（剛体アーム変換）。剛域なしでは恒等。
        let mut k = crate::rigid_arm::transform_stiffness(&k, self.rigid_i, self.rigid_j);

        // ねじり剛性（Saint-Venant）を rx DOF (index 3, 9) に付加。ねじりは断面積分と
        // 連成しない独立項のため、弾性梁（4.1.4）と同じく節点間長基準 GJ/L とし、
        // 剛域では増大させない（剛体アーム変換も rx 自由度には作用しない）。
        if self.torsion_j > 0.0 && self.length > 0.0 {
            let kt = self.g * self.torsion_j / self.length;
            k.set(3, 3, k.get(3, 3) + kt);
            k.set(9, 9, k.get(9, 9) + kt);
            k.set(3, 9, k.get(3, 9) - kt);
            k.set(9, 3, k.get(9, 3) - kt);
        }

        // ローカル接線剛性をグローバル節点系へ回転（R^T·K·R）
        self.axis.to_global(&k)
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        let l = self.flex_length;
        if l <= 0.0 {
            return f;
        }
        let half = l / 2.0;
        // 可撓端の変位（剛体アームで節点変位から写す。剛域なしでは節点変位そのもの）。
        let u_flex = self.flex_disp();

        for gp in &self.gauss_points {
            let (force, _) = Self::section_response_from_cache(gp);
            let w = gp.weight * half;
            let b = Self::compute_b_matrix(gp.xi, l, self.phi_y, self.phi_z);
            let n = force[0];
            let my = force[1];
            let mz = force[2];

            for i in 0..12 {
                let val = b[0][i] * n + b[1][i] * my + b[2][i] * mz;
                f.data[i] += val * w;
            }
        }

        // 塑性化域考慮モデル: 中央弾性部の内力（線形: K_mid·u_flex）を加算
        if let Some(km) = &self.k_mid {
            for i in 0..12 {
                let mut si = 0.0;
                for j in 0..12 {
                    si += km.get(i, j) * u_flex[j];
                }
                f.data[i] += si;
            }
        }

        // せん断ひずみ場の内力（線形弾性: K_shear·u_flex。γ は剛体運動でゼロの
        // 客観的測度なので、trial 変位との積で偽内力は生じない）
        for i in 0..12 {
            let mut si = 0.0;
            for j in 0..12 {
                si += self.k_shear.get(i, j) * u_flex[j];
            }
            f.data[i] += si;
        }

        // 可撓端の内力 → 節点自由度（剛体アームのモーメント寄与を含む）。
        let f_flex: [f64; 12] = std::array::from_fn(|i| f.data[i]);
        let mut f_node = crate::rigid_arm::to_node_force(&f_flex, self.rigid_i, self.rigid_j);

        // ねじり内力（Saint-Venant。節点間長基準・剛域は作用しない）
        if self.torsion_j > 0.0 && self.length > 0.0 {
            let kt = self.g * self.torsion_j / self.length;
            let drx = self.trial_disp[3] - self.trial_disp[9];
            f_node[3] += kt * drx;
            f_node[9] -= kt * drx;
        }

        // ローカル内力をグローバル系へ回転（committed/trial はローカル保持のため）
        let f_global = self.axis.rotate_to_global(&f_node);
        LocalVec {
            data: SmallVec::from_slice(&f_global),
        }
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        // 入力 du はグローバル系。内部状態（trial_disp, B行列ひずみ）はローカル系で
        // 扱うため、まずローカル系へ回転してから累積する。
        let du_global: [f64; 12] = std::array::from_fn(|i| du.data[i]);
        let du_local = self.axis.rotate_to_local(&du_global);
        for i in 0..12 {
            self.trial_disp[i] += du_local[i];
        }
        let l = self.flex_length;
        if l <= 0.0 {
            return;
        }
        // ひずみは可撓端の変位から評価する（剛域は剛体アームとして働く）。
        let u = self.flex_disp();

        for gp in &mut self.gauss_points {
            let b = Self::compute_b_matrix(gp.xi, l, self.phi_y, self.phi_z);
            let eps0 = b[0][0] * u[0] + b[0][6] * u[6];
            let ky = b[1][2] * u[2] + b[1][4] * u[4] + b[1][8] * u[8] + b[1][10] * u[10];
            let kz = b[2][1] * u[1] + b[2][5] * u[5] + b[2][7] * u[7] + b[2][11] * u[11];
            for (i, fiber) in gp.section.fibers.iter().enumerate() {
                let eps = eps0 - kz * fiber.y + ky * fiber.z;
                let (sigma, et) = gp.mats[i].trial(eps);
                gp.trial_stress[i] = sigma;
                gp.trial_et[i] = et;
            }
        }
        if commit {
            for gp in &mut self.gauss_points {
                for mat in &mut gp.mats {
                    mat.commit();
                }
            }
            self.committed_disp = self.trial_disp;
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
        Box::new((self.trial_disp, self.committed_disp, gauss_data))
    }

    fn restore_state(&mut self, state: &dyn Any) {
        if let Some((trial, committed, mats_data)) =
            state.downcast_ref::<([f64; 12], [f64; 12], Vec<Vec<Box<dyn UniaxialMaterial>>>)>()
        {
            self.trial_disp = *trial;
            self.committed_disp = *committed;
            for (gp, gp_mats) in self.gauss_points.iter_mut().zip(mats_data) {
                for (mat, new_mat) in gp.mats.iter_mut().zip(gp_mats) {
                    *mat = new_mat.clone_box();
                }
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
    }

    fn revert_state(&mut self) {
        for gp in &mut self.gauss_points {
            for mat in &mut gp.mats {
                mat.revert();
            }
        }
        self.trial_disp = self.committed_disp;
    }

    fn serialize_checkpoint(&self) -> Vec<u8> {
        #[derive(serde::Serialize, serde::Deserialize)]
        struct FiberBeamCheckpoint {
            trial_disp: [f64; 12],
            committed_disp: [f64; 12],
            gauss_points: Vec<Vec<Vec<u8>>>,
        }
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
        };
        bincode::serialize(&cp).expect("serialize checkpoint")
    }

    fn deserialize_checkpoint(
        &mut self,
        data: &[u8],
    ) -> Result<(), crate::behavior::CheckpointError> {
        #[derive(serde::Serialize, serde::Deserialize)]
        struct FiberBeamCheckpoint {
            trial_disp: [f64; 12],
            committed_disp: [f64; 12],
            gauss_points: Vec<Vec<Vec<u8>>>,
        }
        let cp: FiberBeamCheckpoint = bincode::deserialize(data)
            .map_err(|e| crate::behavior::CheckpointError::Decode(e.to_string()))?;
        self.trial_disp = cp.trial_disp;
        self.committed_disp = cp.committed_disp;
        for (gp, gp_mats) in self.gauss_points.iter_mut().zip(cp.gauss_points) {
            for (mat, mat_bytes) in gp.mats.iter_mut().zip(gp_mats) {
                mat.deserialize_state(&mat_bytes)?;
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
        // 曲率は可撓端変位から復元する（剛域は剛体アームとして働く）。
        let td = self.flex_disp();
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
