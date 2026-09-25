//! 非線形解析の入力チェック（[`super::nonlinear_input_issues`]）のテスト。

use super::*;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    EndCondition, ForceRegime, LocalAxis, Material, MaterialCategory, Node, Section,
};
use squid_n_core::section_shape::{RcBeamRebar, SectionShape};

fn steel_material() -> Material {
    Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SN400".into(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(235.0),
    }
}

/// コンクリート区分の材料（RC・SRC 断面の部材に割り当てる）。
fn concrete_material() -> Material {
    Material {
        category: MaterialCategory::Concrete,
        name: "FC24".into(),
        fy: None,
        fc: Some(24.0),
        ..steel_material()
    }
}

/// 主筋・せん断補強筋の材料（`MaterialId(1)`）。RC・SRC 断面へ割り当てる。
fn rebar_material() -> Material {
    Material {
        id: MaterialId(1),
        name: "SD345".into(),
        category: MaterialCategory::Rebar,
        fy: Some(345.0),
        fc: None,
        ..steel_material()
    }
}

/// 内蔵鉄骨の材料（`MaterialId(2)`）。鋼種名から F 値を引くため名前が要る。
fn steel_grade_material(grade: &str) -> Material {
    Material {
        id: MaterialId(2),
        name: grade.into(),
        ..steel_material()
    }
}

fn rc_section() -> Section {
    let mut sec = rc_section_shape();
    sec.rebar_material = Some(MaterialId(1));
    sec.shear_rebar_material = Some(MaterialId(1));
    sec
}

fn rc_section_shape() -> Section {
    use squid_n_core::section_shape::BeamStirrup;
    SectionShape::RcBeamRect {
        b: 400.0,
        d: 600.0,
        rebar: RcBeamRebar {
            main_dia: 22.0,
            top: vec![6],
            bottom: vec![4],
            cover: 40.0,
            stirrup: BeamStirrup {
                dia: 10.0,
                pitch: 200.0,
                legs: 2,
            },
        },
    }
    .to_section(SectionId(0), "G1".into())
}

/// `rc_section` の主筋材料を取り除いた断面（未割当の入力不備を模擬）。
fn rc_section_without_rebar_material() -> Section {
    let mut sec = rc_section();
    sec.rebar_material = None;
    sec
}

/// 1 部材（2 節点の梁）だけのモデル。断面形状・材料は引数で差し替える。
///
/// 材料は断面が持つため、`material` は断面の主材料として割り当てる。主筋
/// （`MaterialId(1)`）・内蔵鉄骨（`MaterialId(2)`）はモデルへ常に登録しておき、
/// 断面側の割り当ての有無で不備を作り分ける。
fn beam_model(mut section: Section, material: Material) -> Model {
    section.material = Some(MaterialId(0));
    beam_model_inner(
        section,
        vec![material, rebar_material(), steel_grade_material("SN400B")],
    )
}

/// 材料一覧まで指定する版。
fn beam_model_inner(section: Section, materials: Vec<Material>) -> Model {
    let mk = |id: u32, c: [f64; 3]| Node {
        id: NodeId(id),
        coord: c,
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    };
    Model {
        nodes: vec![mk(0, [0.0, 0.0, 0.0]), mk(1, [6000.0, 0.0, 0.0])],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        sections: vec![section],
        materials,
        ..Default::default()
    }
}

/// 有効な入力（耐力算定に必要な強度が揃っている）は不備なしと判定される。
#[test]
fn test_valid_inputs_produce_no_issues() {
    // 鋼材部材（形状未設定）＋ 正の fy
    let mut sec = rc_section();
    sec.shape = None;
    let model = beam_model(sec, steel_material());
    assert!(nonlinear_input_issues(&model).is_empty());
    assert!(ensure_nonlinear_input(&model).is_ok());

    // RC 断面 ＋ 正の Fc
    let model = beam_model(rc_section(), concrete_material());
    assert!(nonlinear_input_issues(&model).is_empty());

    // 鋼断面 ＋ 正の fy
    let model = beam_model(steel_h_section(), steel_material());
    assert!(nonlinear_input_issues(&model).is_empty());

    // SRC 断面 ＋ 内蔵鉄骨の材料（主材料 fy 未設定でも降伏強度を解決できる）
    let model = beam_model(src_section(), concrete_material());
    assert!(nonlinear_input_issues(&model).is_empty());

    // 鋼断面 ＋ コンクリート区分の材料 ＋ fy。
    // 構造種別は材料の区分で決まる仕様であり、断面形状は力学的な性質ではないため
    // 区分の矛盾とはしない。
    let mut mat = concrete_material();
    mat.fy = Some(235.0);
    let model = beam_model(steel_h_section(), mat);
    assert!(
        nonlinear_input_issues(&model).is_empty(),
        "{:?}",
        nonlinear_input_issues(&model)
    );
}

/// 主筋の材料が未割当、または材料はあっても fy が無い RC 部材はエラーとする
/// （材料名 "SD345" からは推定しない）。既定 345 N/mm² で埋めると SD295 の
/// 部材で曲げ降伏耐力を過大評価する（危険側）。
#[test]
fn test_issue_when_rebar_yield_strength_unresolvable() {
    let model = beam_model(rc_section_without_rebar_material(), concrete_material());
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("主筋の材料"), "{}", issues[0]);

    let mut mats = vec![
        concrete_material(),
        rebar_material(),
        steel_grade_material("SN400B"),
    ];
    mats[1].fy = None;
    let mut sec = rc_section();
    sec.material = Some(MaterialId(0));
    let model = beam_model_inner(sec, mats);
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("主筋の材料"), "{}", issues[0]);
}

/// RC 断面なのに Fc が未設定または 0 以下の部材はエラーとする。
/// Fc=0 相当で解析を通すと Mc=0 となりヒンジが一切検出されない（危険側）。
#[test]
fn test_issue_when_rc_member_fc_missing_or_not_positive() {
    for fc in [None, Some(0.0)] {
        let mut mat = concrete_material();
        mat.fc = fc;
        let model = beam_model(rc_section(), mat);
        let issues = nonlinear_input_issues(&model);
        assert_eq!(issues.len(), 1, "{:?}", issues);
        assert!(issues[0].contains("Fc"), "{}", issues[0]);
        assert!(ensure_nonlinear_input(&model).is_err());
    }
}

/// せん断補強筋に未対応グレード（KH785）を割り当てた部材はエラーとする。
/// 未対応グレードを普通強度式で代替すると耐力を過大評価する（危険側）。
#[test]
fn test_issue_when_shear_rebar_grade_unsupported() {
    let mut mats = vec![
        concrete_material(),
        rebar_material(),
        steel_grade_material("SN400B"),
    ];
    mats[1].name = "KH785".into();
    let mut sec = rc_section();
    sec.material = Some(MaterialId(0));
    let model = beam_model_inner(sec, mats);
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("KH785"), "{}", issues[0]);
    assert!(issues[0].contains("未対応"), "{}", issues[0]);
    assert!(ensure_nonlinear_input(&model).is_err());
}

/// 対応グレード（SR235）でも fy が未設定なら入力不備として停止する。
/// 既定 295 で代替すると σwy が 235→295 に増え、耐力を過大評価する（危険側）。
#[test]
fn test_issue_when_supported_shear_rebar_fy_missing() {
    let make = |fy: Option<f64>| {
        let mut shear = rebar_material();
        shear.id = MaterialId(3);
        shear.name = "SR235".into();
        shear.fy = fy;
        let mut sec = rc_section();
        sec.material = Some(MaterialId(0));
        sec.shear_rebar_material = Some(MaterialId(3));
        beam_model_inner(
            sec,
            vec![
                concrete_material(),
                rebar_material(),
                steel_grade_material("SN400B"),
                shear,
            ],
        )
    };

    let model = make(None);
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("SR235"), "{}", issues[0]);
    assert!(issues[0].contains("fy"), "{}", issues[0]);
    assert!(ensure_nonlinear_input(&model).is_err());

    // fy を設定すれば不備なし。
    let model = make(Some(235.0));
    assert!(
        nonlinear_input_issues(&model).is_empty(),
        "{:?}",
        nonlinear_input_issues(&model)
    );
}

/// 断面形状未設定の部材で正の耐力を算定できない材料はエラーとする。
/// - fy なし: せん断降伏耐力が ∞ となり降伏しない。
/// - fy=0（非正値）: 「設定済み」と素通しすると要素生成（`steel_fiber_material`）が
///   解析スレッド内で panic し、UI には「解析スレッドが異常終了しました」としか
///   表示されず原因が利用者に伝わらない（時刻歴解析スレッドの panic 不具合の回帰）。
/// - Fc=0（非正値）: コンクリートのファイバが剛性 0 となり剛性行列が特異化する。
#[test]
fn test_issue_when_shapeless_member_lacks_positive_strength() {
    let shapeless = || {
        let mut sec = rc_section();
        sec.shape = None;
        sec
    };

    let mut mat = steel_material();
    mat.fy = None;
    let model = beam_model(shapeless(), mat);
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("fy"), "{}", issues[0]);

    let mut mat = steel_material();
    mat.fy = Some(0.0);
    let model = beam_model(shapeless(), mat);
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("fy"), "{}", issues[0]);
    assert!(ensure_nonlinear_input(&model).is_err());

    let mut mat = steel_material();
    mat.fy = None;
    mat.fc = Some(0.0);
    let model = beam_model(shapeless(), mat);
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("Fc"), "{}", issues[0]);
}

/// H 形鋼断面を持つ部材の断面（鋼材ファイバ領域あり）。
fn steel_h_section() -> Section {
    SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    }
    .to_section(SectionId(0), "H400".into())
}

/// 鋼材断面形状なのに fy 未設定の部材はエラーとする。
/// ファイバー断面は降伏進展を追うことが目的のため、弾性で代替すると
/// 鋼材がいくら応力が上がっても降伏せず耐力を過大評価する（危険側）。
#[test]
fn test_issue_when_steel_shape_has_no_fy() {
    let mut mat = steel_material();
    mat.fy = None;
    mat.fc = Some(24.0);
    let model = beam_model(steel_h_section(), mat);
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("fy"), "{}", issues[0]);
    assert!(ensure_nonlinear_input(&model).is_err());
}

/// SRC 断面（内蔵鉄骨あり）。内蔵鉄骨の材料は `MaterialId(2)` を割り当てる。
fn src_section() -> Section {
    let mut sec = src_section_bare();
    sec.rebar_material = Some(MaterialId(1));
    sec.shear_rebar_material = Some(MaterialId(1));
    sec.steel_material = Some(MaterialId(2));
    sec
}

/// 内蔵鉄骨・鉄筋の材料を割り当てていない SRC 断面。
fn src_section_bare() -> Section {
    let rebar = match rc_section().shape {
        Some(SectionShape::RcBeamRect { rebar, .. }) => rebar,
        _ => unreachable!(),
    };
    SectionShape::SrcBeamRect {
        b: 500.0,
        d: 700.0,
        rebar,
        steel_height: 300.0,
        steel_width: 150.0,
        steel_web_thick: 6.5,
        steel_flange_thick: 9.0,
    }
    .to_section(SectionId(0), "SRC".into())
}

/// SRC 断面で内蔵鉄骨の材料も主材料 fy も解決できない部材はエラーとする。
/// Fc・主筋の材料が揃っていても、内蔵鉄骨のファイバに降伏強度が要る。
#[test]
fn test_issue_when_src_section_has_no_steel_yield() {
    let mut sec = src_section();
    sec.steel_material = None;
    let model = beam_model(sec, concrete_material());
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("降伏強度"), "{}", issues[0]);
}

/// 断面に材料が割り当てられていない部材はエラーとする。
#[test]
fn test_issue_when_member_has_no_material() {
    let mut sec = rc_section();
    sec.shape = None;
    let mut model = beam_model(sec, steel_material());
    model.sections[0].material = None;
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(
        issues[0].contains("材料が設定されていません"),
        "{}",
        issues[0]
    );
}

/// 弾性でモデル化することが仕様の要素（節点バネ）は検査対象外。
#[test]
fn test_elastic_only_element_kinds_are_not_checked() {
    let mut sec = rc_section();
    sec.shape = None;
    let mut mat = steel_material();
    mat.fy = None;
    let mut model = beam_model(sec, mat);
    model.elements[0].kind = ElementKind::NodalSpring;
    assert!(nonlinear_input_issues(&model).is_empty());
}

/// 配筋を持つ RC 断面に鋼材区分の材料が付いた部材はエラーとする。
/// 鋼材として検定・ヒンジ算定すると耐力を大きく過大評価する（危険側）。
#[test]
fn test_issue_when_rc_section_has_steel_material() {
    let model = beam_model(rc_section(), steel_material());
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("区分が鋼材"), "{}", issues[0]);
}

/// 線材の主材料に鉄筋を割り当てるのは入力の誤りとする。
/// RC 断面の主筋・せん断補強筋は断面の専用の欄で持つ。
#[test]
fn test_issue_when_member_material_is_rebar() {
    let mut mat = concrete_material();
    mat.category = MaterialCategory::Rebar;
    mat.name = "SD345".into();
    let model = beam_model(rc_section(), mat);
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("区分が鉄筋"), "{}", issues[0]);
}

/// 複数件の不備はメッセージへ 5 件まで列挙し、残りは件数で示す。
#[test]
fn test_error_message_lists_head_and_remaining_count() {
    let mut mat = concrete_material();
    mat.fc = None;
    let mut model = beam_model(rc_section(), mat);
    let base = model.elements[0].clone();
    for i in 1..8u32 {
        let mut e = base.clone();
        e.id = ElemId(i);
        model.elements.push(e);
    }
    assert_eq!(nonlinear_input_issues(&model).len(), 8);
    let msg = ensure_nonlinear_input(&model).expect_err("不備があればエラー");
    assert_eq!(msg.lines().count(), MAX_LISTED + 1);
    assert!(msg.contains("他 3 件"), "{}", msg);
}
