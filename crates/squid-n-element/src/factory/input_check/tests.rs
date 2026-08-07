//! 非線形解析の入力チェック（[`super::nonlinear_input_issues`]）のテスト。

use super::*;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    EndCondition, ForceRegime, LocalAxis, Material, MaterialCategory, Node, Section,
};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

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
    // 材料は断面が持つ。主筋・せん断補強筋は `MaterialId(1)`。
    sec.rebar_material = Some(MaterialId(1));
    sec.shear_rebar_material = Some(MaterialId(1));
    sec
}

fn rc_section_shape() -> Section {
    SectionShape::RcRect {
        b: 400.0,
        d: 600.0,
        rebar: RcRebar {
            main_x: BarSet {
                count: 6,
                dia: 22.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 4,
                dia: 22.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
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

/// 鋼断面（形状未設定）＋ fy 設定済みの部材は不備なし。
#[test]
fn test_no_issue_for_steel_member_with_fy() {
    let mut sec = rc_section();
    sec.shape = None;
    let model = beam_model(sec, steel_material());
    assert!(nonlinear_input_issues(&model).is_empty());
    assert!(ensure_nonlinear_input(&model).is_ok());
}

/// RC 断面＋ Fc 設定済みの部材は不備なし。
#[test]
fn test_no_issue_for_rc_member_with_fc() {
    let model = beam_model(rc_section(), concrete_material());
    assert!(nonlinear_input_issues(&model).is_empty());
}

/// 主筋の材料が未割当の RC 部材はエラーとする。
/// 既定 345 N/mm² で埋めると SD295 の部材で曲げ降伏耐力を過大評価する（危険側）。
#[test]
fn test_issue_when_rebar_material_unset() {
    let model = beam_model(rc_section_without_rebar_material(), concrete_material());
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("主筋の材料"), "{}", issues[0]);
}

/// 主筋の材料はあっても fy が無ければ σy を決められないためエラーとする
/// （材料名 "SD345" からは推定しない）。
#[test]
fn test_issue_when_rebar_material_has_no_fy() {
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

/// RC 断面なのに Fc が未設定の部材はエラーとする。
/// Fc=0 相当で解析を通すと Mc=0 となりヒンジが一切検出されない（危険側）。
#[test]
fn test_issue_when_rc_member_has_no_fc() {
    let mut mat = concrete_material();
    mat.fc = None;
    let model = beam_model(rc_section(), mat);
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("Fc"), "{}", issues[0]);
    assert!(ensure_nonlinear_input(&model).is_err());
}

/// RC 断面で Fc が 0 以下の部材もエラーとする（未設定と同じく耐力を算定できない）。
#[test]
fn test_issue_when_rc_member_fc_not_positive() {
    let mut mat = concrete_material();
    mat.fc = Some(0.0);
    let model = beam_model(rc_section(), mat);
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("Fc"), "{}", issues[0]);
}

/// fy も Fc もない材料の部材はエラーとする（せん断降伏耐力が ∞ となり降伏しない）。
#[test]
fn test_issue_when_material_has_no_strength() {
    let mut sec = rc_section();
    sec.shape = None;
    let mut mat = steel_material();
    mat.fy = None;
    let model = beam_model(sec, mat);
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("fy"), "{}", issues[0]);
}

/// 回帰テスト（時刻歴解析スレッドの panic 不具合）: 断面形状未設定の部材で
/// fy が 0（非正値）の材料は「設定済み」と素通しせずエラーとする。
/// 素通しすると入力チェック後の要素生成（`steel_fiber_material`）が解析
/// スレッド内で panic し、UI には「解析スレッドが異常終了しました」としか
/// 表示されず原因が利用者に伝わらない。
#[test]
fn test_issue_when_shapeless_member_fy_not_positive() {
    let mut sec = rc_section();
    sec.shape = None;
    let mut mat = steel_material();
    mat.fy = Some(0.0);
    let model = beam_model(sec, mat);
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("fy"), "{}", issues[0]);
    assert!(ensure_nonlinear_input(&model).is_err());
}

/// 断面形状未設定の部材で Fc が 0（非正値）の材料もエラーとする
/// （コンクリートのファイバが剛性 0 となり剛性行列が特異化する）。
#[test]
fn test_issue_when_shapeless_member_fc_not_positive() {
    let mut sec = rc_section();
    sec.shape = None;
    let mut mat = steel_material();
    mat.fy = None;
    mat.fc = Some(0.0);
    let model = beam_model(sec, mat);
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

/// 鋼材断面形状＋ fy 設定済みは不備なし。
#[test]
fn test_no_issue_for_steel_shape_with_fy() {
    let model = beam_model(steel_h_section(), steel_material());
    assert!(nonlinear_input_issues(&model).is_empty());
}

/// 鋼材断面形状なのに fy 未設定の部材はエラーとする。
/// ファイバー断面は降伏進展を追うことが目的のため、弾性で代替すると
/// 鋼材がいくら応力が上がっても降伏せず耐力を過大評価する（危険側）。
#[test]
fn test_issue_when_steel_shape_has_no_fy() {
    let mut mat = steel_material();
    mat.fy = None;
    mat.fc = Some(24.0); // fc があっても鋼材形状は fy が必須
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
        Some(SectionShape::RcRect { rebar, .. }) => rebar,
        _ => unreachable!(),
    };
    SectionShape::SrcRect {
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

/// SRC 断面は内蔵鉄骨の材料から降伏強度を解決できれば、主材料 fy 未設定でも不備なし。
#[test]
fn test_no_issue_for_src_section_with_steel_material() {
    let model = beam_model(src_section(), concrete_material());
    assert!(nonlinear_input_issues(&model).is_empty());
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

/// 鋼断面にコンクリート区分の材料が付いても区分の不備とはしない。
/// 構造種別は材料の区分で決まる仕様であり、H 形のコンクリート部材は
/// 正しい入力である（断面形状は見た目であって力学的な性質ではない）。
///
/// ファイバー断面は形鋼の板要素を鋼材ファイバとして組み立てるため fy を要求するが、
/// これは区分の矛盾ではなく材料強度の不足として扱う。
#[test]
fn test_no_category_issue_for_steel_shape_with_concrete_material() {
    let mut mat = concrete_material();
    mat.fy = Some(235.0);
    let model = beam_model(steel_h_section(), mat);
    assert!(
        nonlinear_input_issues(&model).is_empty(),
        "{:?}",
        nonlinear_input_issues(&model)
    );
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
