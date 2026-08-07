//! 仕口パネル（柱梁接合部パネル）の検証（V&V）。
//!
//! 資料には数値例がないため、次の 3 本を軸に検証する。
//!
//! 1. **回帰**: パネルを設けないモデルの解析結果が、パネル導入前と完全に一致する
//!    （追加自由度が 1 つも払い出されないこと）。
//! 2. **解析解照合**: 単一接合部で、パネルのせん断変形角が
//!    `γ = pM / (G・Ve)` の解析解と一致する。
//! 3. **釣り合い**: 資料 (2.10.3-3) の節点せん断モーメント釣り合い
//!    `M^p_S + Σ M^i_S = 0` の残差が数値誤差の範囲に収まる。
//!
//! さらに、パネルを入れると接合部がせん断変形する分だけ架構が柔らかくなる
//! （変位が増える）という定性的な整合も確認する。

use squid_n_core::dof::{Dof6Mask, DofMap};
use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material,
    MaterialCategory, Model, NodalLoad, Node, RigidZone, Section,
};
use squid_n_core::panel_zone::{beam_panel_depth, PanelGeometry};
use squid_n_core::section_shape::SectionShape;

const COL_H: f64 = 400.0;
const COL_B: f64 = 400.0;
const COL_TW: f64 = 13.0;
const COL_TF: f64 = 21.0;
const BEAM_H: f64 = 600.0;
const BEAM_TF: f64 = 17.0;
/// 柱フェース距離（＝柱せい/2）。梁端のパネル分オフセット。
const FACE_BEAM: f64 = COL_H / 2.0;
/// 梁フェース距離（＝梁せい/2）。柱端のパネル分オフセット。
const FACE_COL: f64 = BEAM_H / 2.0;

fn node(id: u32, coord: [f64; 3], restraint: Dof6Mask) -> Node {
    Node {
        id: NodeId(id),
        coord,
        restraint,
        mass: None,
        story: None,
        support_spring: None,
    }
}

fn steel_section(id: u32, shape: SectionShape, depth: f64, width: f64, area: f64) -> Section {
    Section {
        id: SectionId(id),
        name: String::new(),
        area,
        iy: 2.0e8,
        iz: 2.0e8,
        j: 1.0e7,
        depth,
        width,
        as_y: area * 0.4,
        as_z: area * 0.4,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: Some(shape),
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    }
}

fn member(id: u32, n0: u32, n1: u32, sec: u32, rigid: RigidZone) -> ElementData {
    ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(n0), NodeId(n1)],
        section: Some(SectionId(sec)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: rigid,
        plastic_zone: None,
        spring: None,
    }
}

/// 面外自由度（Uy・Rx・Rz）の拘束マスク。
///
/// 部材 2 本だけの最小モデルは面外方向に機構を持つ（柱の材軸まわり回転＝Rz は
/// 既定のねじり解放で拘束が外れ、片持ちの梁はそれを接地できない）。検証したいのは
/// パネルの面内挙動なので、平面架構として面外を拘束する。
fn out_of_plane_fixed() -> Dof6Mask {
    let mut m = Dof6Mask::FREE;
    m.set_fixed(squid_n_core::dof::Dof::Uy);
    m.set_fixed(squid_n_core::dof::Dof::Rx);
    m.set_fixed(squid_n_core::dof::Dof::Rz);
    m
}

/// L 型の 1 節点フレーム（柱脚固定・梁先端自由）。X-Z 面内の平面架構とする。
///
/// - 節点 0: 接合部（柱頭 ＝ 梁の i 端）
/// - 節点 1: 梁の先端（鉛直荷重を載荷）
/// - 節点 2: 柱脚（固定）
///
/// `with_panel` が真なら接合部へ仕口パネルを設ける。
fn l_frame(with_panel: bool) -> Model {
    l_frame_with(with_panel, false)
}

/// `rigid_joint` が真なら、パネルを設けずに接合部の有限寸法だけを剛域として
/// 与える。パネル有りモデルと剛域条件が揃うため、両者の差は
/// **パネルのせん断変形のみ**に由来する。
///
/// パネル有りのモデルは準備計算と同じ経路
/// （[`squid_n_element::panel_gen::apply_auto_panel_zones`]）で生成する。
/// パネル要素の追加とオフセットの剛域長への書き込みが一体の処理であり、
/// 手組みすると実際のモデル化と食い違うためである。
fn l_frame_with(with_panel: bool, rigid_joint: bool) -> Model {
    let rigid = RigidZone {
        face_i: FACE_BEAM,
        face_j: 0.0,
        length_i: if rigid_joint { FACE_BEAM } else { 0.0 },
        ..Default::default()
    };
    let col_rigid = RigidZone {
        face_i: 0.0,
        face_j: FACE_COL,
        length_j: if rigid_joint { FACE_COL } else { 0.0 },
        ..Default::default()
    };

    let elements = vec![
        member(0, 0, 1, 0, rigid),     // 梁（水平材）: i 端が接合部
        member(1, 2, 0, 1, col_rigid), // 柱（鉛直材）: j 端が接合部
    ];

    let mut model = Model {
        nodes: vec![
            node(0, [0.0, 0.0, 3000.0], out_of_plane_fixed()),
            node(1, [6000.0, 0.0, 3000.0], out_of_plane_fixed()),
            node(2, [0.0, 0.0, 0.0], Dof6Mask::FIXED),
        ],
        sections: vec![
            steel_section(
                0,
                SectionShape::SteelH {
                    height: BEAM_H,
                    width: 200.0,
                    web_thick: 11.0,
                    flange_thick: BEAM_TF,
                },
                BEAM_H,
                200.0,
                1.34e4,
            ),
            steel_section(
                1,
                SectionShape::SteelH {
                    height: COL_H,
                    width: COL_B,
                    web_thick: COL_TW,
                    flange_thick: COL_TF,
                },
                COL_H,
                COL_B,
                2.187e4,
            ),
        ],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SN400B".into(),
            category: MaterialCategory::Steel,
            young: 205_000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        elements,
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(1),
            name: "P".into(),
            nodal: vec![NodalLoad {
                node: NodeId(1),
                // 梁先端に鉛直下向き荷重（接合部に曲げを与える）
                values: [0.0, 0.0, -50_000.0, 0.0, 0.0, 0.0],
            }],
            member: Vec::new(),
        }],
        ..Default::default()
    };
    if with_panel {
        let panels = squid_n_element::panel_gen::apply_auto_panel_zones(&mut model);
        assert_eq!(panels.len(), 1, "接合部にパネルが 1 つ生成される");
    }
    model
}

/// パネルの実効体積 Ve と せん断剛性 G·Ve。
fn panel_stiffness(model: &Model) -> (f64, f64) {
    let col = &model.sections[1];
    let beam = &model.sections[0];
    let geom = PanelGeometry::from_column(col).expect("H 形柱");
    let db = beam_panel_depth(beam);
    let ve = geom.effective_volume(db);
    let mat = &model.materials[0];
    (ve, mat.shear_modulus() * ve)
}

/// 1: パネルを設けないモデルでは追加自由度が 1 つも払い出されず、
///    独立自由度数が従来（節点 × 6）と一致する。
#[test]
fn test_no_panel_model_is_unchanged() {
    let model = l_frame(false);
    let dofmap = DofMap::build(&model);
    // 節点 0・1 は面内 3 成分（Ux・Uz・Ry）のみ自由、節点 2 は固定。
    assert_eq!(dofmap.n_active(), 6);
    for ni in 0..model.nodes.len() {
        assert!(!dofmap.has_panel_dof(ni), "パネル無しで追加自由度は出ない");
    }
}

/// パネルを設けると、その節点にだけ γX・γY の 2 自由度が増える。
#[test]
fn test_panel_adds_exactly_two_dofs() {
    let base = DofMap::build(&l_frame(false)).n_active();
    let model = l_frame(true);
    let dofmap = DofMap::build(&model);
    assert_eq!(dofmap.n_active(), base + 2);
    assert!(dofmap.has_panel_dof(0), "接合部節点にパネル自由度");
    assert!(!dofmap.has_panel_dof(1));
}

/// `K·u = f` を直接解き、独立自由度の解ベクトルを返す（追加自由度を含む）。
///
/// `linear_static_once` は節点 6 成分の変位しか返さないため、パネルの
/// せん断変形角を取り出すには全体系を自前で解く必要がある。
fn solve_free(model: &Model, dofmap: &DofMap) -> Vec<f64> {
    use squid_n_math::solver::LinearSolver;
    let k = squid_n_solver::assemble::assemble_global_k(model, dofmap);
    let f = squid_n_solver::assemble::assemble_global_f(model, dofmap, LoadCaseId(1));
    let mut solver = squid_n_math::lu::LuSolver::default();
    solver.factorize(&k).expect("分解できる");
    solver.solve(&f).expect("解ける")
}

/// 疎行列とベクトルの積 `K·u`。
fn spmv(k: &faer::sparse::SparseColMat<usize, f64>, u: &[f64]) -> Vec<f64> {
    let mut out = vec![0.0; k.nrows()];
    let col_ptr = k.col_ptr();
    let row_idx = k.row_idx();
    let val = k.val();
    for c in 0..k.ncols() {
        for i in col_ptr[c]..col_ptr[c + 1] {
            out[row_idx[i]] += val[i] * u[c];
        }
    }
    out
}

/// 2: 解析解照合。
///
/// パネルのせん断変形角 `γ` と、パネルが負担するせん断モーメント `M` の間には
/// `M = K・γ`（`K = G・Ve`）が成り立たなければならない。解析で得た `γ` から
/// パネル要素の内力を評価し、この関係を確認する。
///
/// あわせて、パネルの内力が「部材がパネル自由度へ寄せるモーメント」と符号反転で
/// 釣り合う（資料 (2.10.3-3)）ことも確認する。
#[test]
fn test_panel_shear_angle_matches_closed_form() {
    let model = l_frame(true);
    let (ve, k_panel) = panel_stiffness(&model);
    assert!(ve > 0.0 && k_panel > 0.0);

    let dofmap = DofMap::build(&model);
    let u = solve_free(&model, &dofmap);

    let gx = dofmap.panel_dof(0, 0).expect("γX") as usize;
    let gy = dofmap.panel_dof(0, 1).expect("γY") as usize;
    let gamma = [u[gx], u[gy]];

    // 面内（X-Z 面）に載荷しているため、γY（X'-Z' 面のせん断変形角）が生じる。
    assert!(
        gamma[1].abs() > 1e-12,
        "パネルがせん断変形しているはず: γ={gamma:?}"
    );

    // パネル要素へ解を与えて内力（＝せん断モーメント）を取り出す。
    use squid_n_element::behavior::{Ctx, ElementBehavior, LocalVec};
    let panel_data = model
        .elements
        .iter()
        .find(|e| matches!(e.kind, ElementKind::PanelZone))
        .expect("パネル要素");
    let mut panel = squid_n_element::panel::PanelZone::new(panel_data, &model);
    let ctx = Ctx { model: &model };
    let du = LocalVec {
        data: smallvec::smallvec![gamma[0], gamma[1]],
    };
    panel.update_state(&du, true, &ctx);
    let m = panel.panel_moments();

    // M = K・γ（弾性パネルの定義そのもの。実装が [Tp] 変換を挟んでも成り立つ）。
    for (i, (&mi, &gi)) in m.iter().zip(gamma.iter()).enumerate() {
        let expected = k_panel * gi;
        assert!(
            (mi - expected).abs() <= 1e-9 * expected.abs().max(1.0),
            "成分 {i}: M={mi} は K·γ={expected} と一致すべき"
        );
    }
}

/// 3: 釣り合い（資料 (2.10.3-3)）。
///
/// パネル自由度には外力が加わらないため、平衡方程式の当該行の残差
/// `(K·u − f)` は数値誤差の範囲で 0 でなければならない。これは
/// 「パネル要素の `M^p_S` ＋ 各部材が寄せる `M^i_S` の総和が 0」と同値である。
#[test]
fn test_panel_dof_equilibrium_residual_is_zero() {
    let model = l_frame(true);
    let dofmap = DofMap::build(&model);
    let k = squid_n_solver::assemble::assemble_global_k(&model, &dofmap);
    let f = squid_n_solver::assemble::assemble_global_f(&model, &dofmap, LoadCaseId(1));
    let u = solve_free(&model, &dofmap);
    let ku = spmv(&k, &u);

    let gx = dofmap.panel_dof(0, 0).expect("γX") as usize;
    let gy = dofmap.panel_dof(0, 1).expect("γY") as usize;

    // パネル自由度に外力は加わらない。
    assert_eq!(f[gx], 0.0);
    assert_eq!(f[gy], 0.0);

    // 残差の尺度は系全体の内力スケールで正規化する。
    let scale = ku.iter().fold(0.0_f64, |m, v| m.max(v.abs())).max(1.0);
    for (label, g) in [("γX", gx), ("γY", gy)] {
        let residual = (ku[g] - f[g]).abs() / scale;
        assert!(
            residual < 1e-9,
            "{label} の釣り合い残差が大きい: {residual}"
        );
    }
}

/// パネルのせん断変形の分だけ架構が柔らかくなる。
///
/// 比較対象は「同じ剛域（接合部の有限寸法）を持ち、パネルが剛（せん断変形しない）」
/// モデルとする。剛域の有無で比べると、パネル導入に伴う剛域の追加で逆に硬くなる
/// ため、パネルのせん断変形の効果だけを取り出せない。
#[test]
fn test_panel_shear_adds_flexibility() {
    let rigid_joint =
        squid_n_solver::linear::linear_static_once(&l_frame_with(false, true), LoadCaseId(1))
            .expect("剛パネル（剛域のみ）の解析");
    let with_panel = squid_n_solver::linear::linear_static_once(&l_frame(true), LoadCaseId(1))
        .expect("パネル有りの解析");

    // 荷重点（節点 1）の鉛直変位（下向き荷重なので負）。
    let d_rigid = rigid_joint.disp[1][2].abs();
    let d_panel = with_panel.disp[1][2].abs();
    assert!(d_rigid > 0.0 && d_panel > 0.0, "変位が生じている");
    assert!(
        d_panel > d_rigid,
        "パネルのせん断変形で柔らかくなるはず: 剛パネル {d_rigid}, パネル有り {d_panel}"
    );
}

/// 仕口パネルの導入は「接合部の有限寸法（剛域）が付いて硬くなる」効果と
/// 「パネルがせん断変形して柔らかくなる」効果の両方を持つ。接合部を無視した
/// 従来モデル（剛域なし・パネルなし）との比較では、前者が勝って全体としては
/// 硬くなる。この向きが逆転した場合は剛域の折り込み（`panel_offset::resolve`）が
/// 効いていないことを意味するため、回帰として押さえる。
#[test]
fn test_panel_is_stiffer_than_ignoring_joint_size() {
    let plain = squid_n_solver::linear::linear_static_once(&l_frame(false), LoadCaseId(1))
        .expect("接合部無視モデルの解析");
    let with_panel = squid_n_solver::linear::linear_static_once(&l_frame(true), LoadCaseId(1))
        .expect("パネル有りの解析");
    assert!(
        with_panel.disp[1][2].abs() < plain.disp[1][2].abs(),
        "剛域が折り込まれていない: 接合部無視 {}, パネル有り {}",
        plain.disp[1][2].abs(),
        with_panel.disp[1][2].abs()
    );
}
