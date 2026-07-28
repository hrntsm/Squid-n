//! 非線形解析の入力チェック（[`super::nonlinear_input_issues`]）のテスト。

use super::*;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{EndCondition, ForceRegime, LocalAxis, Material, Node, Section};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

fn steel_material() -> Material {
    Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SN400".into(),
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(235.0),
    }
}

fn rc_section() -> Section {
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
                grade: Some("SD295A".into()),
            },
            main_grade: Some("SD345".into()),
        },
    }
    .to_section(SectionId(0), "G1".into())
}

/// `rc_section` の主筋材質を取り除いた断面（材質未設定の入力不備を模擬）。
fn rc_section_without_main_grade() -> Section {
    let mut sec = rc_section();
    if let Some(SectionShape::RcRect { rebar, .. }) = sec.shape.as_mut() {
        rebar.main_grade = None;
    }
    sec
}

/// 1 部材（2 節点の梁）だけのモデル。断面形状・材料は引数で差し替える。
fn beam_model(section: Section, material: Material) -> Model {
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
            material: Some(MaterialId(0)),
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
        materials: vec![material],
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
    let mut mat = steel_material();
    mat.name = "FC24".into();
    mat.fy = None;
    mat.fc = Some(24.0);
    let model = beam_model(rc_section(), mat);
    assert!(nonlinear_input_issues(&model).is_empty());
}

/// 主筋の材質が未設定でも、部材材料に fy があれば解決できるため不備ではない。
#[test]
fn test_no_issue_when_main_grade_unset_but_material_has_fy() {
    let mut mat = steel_material();
    mat.name = "FC24".into();
    mat.fc = Some(24.0);
    mat.fy = Some(345.0);
    let model = beam_model(rc_section_without_main_grade(), mat);
    assert!(nonlinear_input_issues(&model).is_empty());
}

/// 主筋の材質も材料の fy も無い RC 部材はエラーとする。
/// 既定 345 N/mm² で埋めると SD295 の部材で曲げ降伏耐力を過大評価する（危険側）。
#[test]
fn test_issue_when_main_rebar_grade_unset() {
    let mut mat = steel_material();
    mat.name = "FC24".into();
    mat.fy = None;
    mat.fc = Some(24.0);
    let model = beam_model(rc_section_without_main_grade(), mat);
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("主筋の材質"), "{}", issues[0]);
}

/// RC 断面なのに Fc が未設定の部材はエラーとする。
/// Fc=0 相当で解析を通すと Mc=0 となりヒンジが一切検出されない（危険側）。
#[test]
fn test_issue_when_rc_member_has_no_fc() {
    let mut mat = steel_material();
    mat.fy = None;
    let model = beam_model(rc_section(), mat);
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("Fc"), "{}", issues[0]);
    assert!(ensure_nonlinear_input(&model).is_err());
}

/// RC 断面で Fc が 0 以下の部材もエラーとする（未設定と同じく耐力を算定できない）。
#[test]
fn test_issue_when_rc_member_fc_not_positive() {
    let mut mat = steel_material();
    mat.fy = None;
    mat.fc = Some(0.0);
    let model = beam_model(rc_section(), mat);
    let issues = nonlinear_input_issues(&model);
    assert_eq!(issues.len(), 1, "{:?}", issues);
    assert!(issues[0].contains("Fc"), "{}", issues[0]);
}

/// fy も Fc も無い材料の部材はエラーとする（せん断降伏耐力が ∞ となり降伏しない）。
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

/// 材料が割り当てられていない部材はエラーとする。
#[test]
fn test_issue_when_member_has_no_material() {
    let mut sec = rc_section();
    sec.shape = None;
    let mut model = beam_model(sec, steel_material());
    model.elements[0].material = None;
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

/// 複数件の不備はメッセージへ 5 件まで列挙し、残りは件数で示す。
#[test]
fn test_error_message_lists_head_and_remaining_count() {
    let mut mat = steel_material();
    mat.fy = None;
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
