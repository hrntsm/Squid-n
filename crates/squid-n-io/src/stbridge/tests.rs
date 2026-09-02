//! ST-Bridge 入出力の統合テスト（往復・取り込み報告・断面形状・id 正規化など）。

use super::*;
use smallvec::smallvec;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::SlabPlate;
use squid_n_core::model::{
    AxisGroupKind, AxisSource, ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis,
    Material, MaterialCategory, Model, Node, Section, Story,
};
use squid_n_core::section_shape::SectionShape;

/// 標準グレード名 `SN400B` の材料（物性は `material_std` の標準表と一致させる）。
fn sn400b(id: u32) -> Material {
    Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(id),
        name: "SN400B".into(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(235.0),
    }
}

/// 標準往復用の代表モデル（鋼 H 断面・標準グレード材料・階所属節点つき）。
/// ST-Bridge の幾何スコープに収まる要素のみで構成する（材料の E/ν・荷重は対象外）。
fn representative_model() -> Model {
    let mut m = Model::default();
    for (i, c) in [
        [0.0, 0.0, 0.0],
        [6000.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [6000.0, 0.0, 3000.0],
    ]
    .iter()
    .enumerate()
    {
        m.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: squid_n_core::dof::Dof6Mask::FREE,
            mass: None,
            story: if i >= 2 { Some(StoryId(0)) } else { None },
            support_spring: None,
        });
    }
    m.stories.push(Story {
        level_kind: Default::default(),
        structure: Default::default(),
        id: StoryId(0),
        name: "1F".into(),
        elevation: 3000.0,
        node_ids: vec![NodeId(2), NodeId(3)],
        seismic_weight: None,
        weight_override: None,
    });
    m.materials.push(sn400b(0));
    // 柱用・梁用で別断面（共有断面の分割を避け、意味的往復を単純化）。
    let col_h = SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let beam_h = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    };
    // 名前にエスケープ対象を含める。
    push_section(&mut m, col_h.to_section(SectionId(0), "C&1<2".into()));
    push_section(&mut m, beam_h.to_section(SectionId(1), "G1".into()));
    // 柱2本（鉛直, section 0）＋大梁1本（水平, section 1）。
    let mk = |id: u32, ni: u32, nj: u32, sec: u32, refv: [f64; 3]| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(ni), NodeId(nj)],
        section: Some(SectionId(sec)),
        local_axis: LocalAxis { ref_vector: refv },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    m.elements.push(mk(0, 0, 2, 0, [1.0, 0.0, 0.0]));
    m.elements.push(mk(1, 1, 3, 0, [1.0, 0.0, 0.0]));
    m.elements.push(mk(2, 2, 3, 1, [0.0, 0.0, 1.0]));
    m
}

/// 2 つの参照ベクトルが（浮動小数の往復誤差を許して）ほぼ一致するか。
fn ref_vec_close(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() < 1e-9)
}

/// 意味的に一致するか（標準 ST-Bridge の幾何スコープのフィールドのみ）。
/// 材料の E/ν や荷重は ST-Bridge の対象外なので比較しない。
fn assert_semantic_eq(a: &Model, b: &Model) {
    assert_eq!(a.nodes.len(), b.nodes.len(), "node count");
    for (x, y) in a.nodes.iter().zip(&b.nodes) {
        assert_eq!(x.id, y.id);
        assert_eq!(x.coord, y.coord, "coord");
        assert_eq!(x.story, y.story, "story");
    }
    assert_eq!(a.stories.len(), b.stories.len());
    for (x, y) in a.stories.iter().zip(&b.stories) {
        assert_eq!(x.id, y.id);
        assert_eq!(x.name, y.name);
        assert_eq!(x.elevation, y.elevation);
    }
    assert_eq!(a.materials.len(), b.materials.len(), "material count");
    for (x, y) in a.materials.iter().zip(&b.materials) {
        assert_eq!(x.name, y.name, "material grade name");
        assert_eq!(x.young, y.young);
        assert_eq!(x.poisson, y.poisson);
        assert_eq!(x.fy, y.fy);
        assert_eq!(x.fc, y.fc);
    }
    assert_eq!(a.sections.len(), b.sections.len(), "section count");
    for (x, y) in a.sections.iter().zip(&b.sections) {
        assert_eq!(x.id, y.id);
        assert_eq!(x.name, y.name, "section name (escape)");
        assert!((x.area - y.area).abs() < 1e-6, "area");
        assert!((x.iy - y.iy).abs().max((x.iz - y.iz).abs()) < 1.0, "iy/iz");
        assert_eq!(x.depth, y.depth);
        assert_eq!(x.width, y.width);
        // 材料は断面が持つ。主材料・鉄筋・内蔵鉄骨のすべてを比べる。
        assert_eq!(x.material, y.material, "断面の主材料");
        assert_eq!(x.rebar_material, y.rebar_material, "主筋の材料");
        assert_eq!(
            x.shear_rebar_material, y.shear_rebar_material,
            "せん断補強筋の材料"
        );
        assert_eq!(x.steel_material, y.steel_material, "内蔵鉄骨の材料");
    }
    assert_eq!(a.elements.len(), b.elements.len());
    for (x, y) in a.elements.iter().zip(&b.elements) {
        assert_eq!(x.id, y.id);
        assert_eq!(x.nodes.as_slice(), y.nodes.as_slice(), "connectivity");
        assert_eq!(x.section, y.section);
        assert!(
            ref_vec_close(x.local_axis.ref_vector, y.local_axis.ref_vector),
            "ref_vector {:?} vs {:?}",
            x.local_axis.ref_vector,
            y.local_axis.ref_vector
        );
    }
}

#[test]
fn test_roundtrip_semantic() {
    let m = representative_model();
    let xml = export_stbridge(&m).expect("export");
    let m2 = import_stbridge(&xml).expect("import");
    assert_semantic_eq(&m, &m2);
}

#[test]
fn test_roundtrip_twice_stable() {
    // import→export→再import で安定（DoD §8.3）。
    let m = representative_model();
    let xml1 = export_stbridge(&m).unwrap();
    let m2 = import_stbridge(&xml1).unwrap();
    let xml2 = export_stbridge(&m2).unwrap();
    assert_eq!(xml1, xml2, "export は冪等であるべき");
    let m3 = import_stbridge(&xml2).unwrap();
    assert_semantic_eq(&m2, &m3);
}

#[test]
fn test_column_girder_classification() {
    let m = representative_model();
    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbColumn "), "鉛直材は StbColumn");
    assert!(xml.contains("<StbGirder "), "水平材は StbGirder");
}

#[test]
fn test_reject_non_stbridge() {
    let r = import_stbridge("<foo/>");
    assert!(matches!(r, Err(StbError::Version(_))));
}

#[test]
fn test_reject_v1() {
    let r = import_stbridge("<ST_BRIDGE version=\"1.4.0\"><StbModel/></ST_BRIDGE>");
    assert!(matches!(r, Err(StbError::Version(_))));
}

#[test]
fn test_read_stbridge_file_shift_jis() {
    use encoding_rs::SHIFT_JIS;
    let m = representative_model();
    let xml = export_stbridge(&m).unwrap();
    // Shift_JIS には変換できない文字（XML 宣言の UTF-8 等）を避けるため、
    // 日本語を含む注釈を付与した上で Shift_JIS へエンコードする。
    let with_jp = format!("<!-- 柱と梁のモデル -->\n{}", xml);
    let (encoded, _, _) = SHIFT_JIS.encode(&with_jp);

    let dir = crate::test_util::test_tmp().join("squid_n_test_stb_sjis");
    let _ = std::fs::create_dir_all(&dir);
    let path = dir.join("shift_jis.stb");
    std::fs::write(&path, encoded.as_ref()).unwrap();

    let decoded = read_stbridge_file(&path).expect("Shift_JIS デコード");
    let m2 = import_stbridge(&decoded).expect("取り込み");
    assert!(m2.validate().is_ok());
    assert_eq!(m2.nodes.len(), m.nodes.len());
}

#[test]
fn test_read_stbridge_file_utf8_bom() {
    let m = representative_model();
    let xml = export_stbridge(&m).unwrap();
    let bytes = {
        let mut b = vec![0xEF, 0xBB, 0xBF];
        b.extend_from_slice(xml.as_bytes());
        b
    };
    let dir = crate::test_util::test_tmp().join("squid_n_test_stb_bom");
    let _ = std::fs::create_dir_all(&dir);
    let path = dir.join("utf8_bom.stb");
    std::fs::write(&path, bytes).unwrap();

    let decoded = read_stbridge_file(&path).expect("UTF-8 BOM デコード");
    assert!(decoded.starts_with("<?xml") || decoded.starts_with("<!--"));
    let m2 = import_stbridge(&decoded).expect("取り込み");
    assert!(m2.validate().is_ok());
}

#[test]
fn test_imported_model_validates() {
    let m = representative_model();
    let xml = export_stbridge(&m).unwrap();
    let m2 = import_stbridge(&xml).unwrap();
    assert!(m2.validate().is_ok(), "取り込んだモデルは検証を通る");
}

use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};

fn rebar() -> RcRebar {
    RcRebar {
        main_x: BarSet {
            count: 3,
            dia: 22.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 3,
            dia: 22.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
        },
    }
}

fn member(id: u32, kind_col: bool, sec: u32) -> ElementData {
    // kind_col=true は鉛直（柱）、false は水平（梁）になるよう節点を選ぶ。
    let (a, b) = if kind_col {
        (NodeId(0), NodeId(2)) // 鉛直
    } else {
        (NodeId(2), NodeId(3)) // 水平
    };
    ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec![a, b],
        section: Some(SectionId(sec)),
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

/// 4 節点だけ持つ骨組（部材・断面は各テストで差し込む）。
fn frame_nodes() -> Model {
    let mut m = Model::default();
    for (i, c) in [
        [0.0, 0.0, 0.0],
        [6000.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [6000.0, 0.0, 3000.0],
    ]
    .iter()
    .enumerate()
    {
        m.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: squid_n_core::dof::Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    m.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SN400B".into(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(235.0),
    });
    m
}

/// 材料は断面が持つ。断面へ材料を割り当ててモデルへ足す。
///
/// 主材料は [`frame_nodes`] の材料 0（SN400B）。配筋を持つ断面には主筋・せん断補強筋
/// （SD345）を、SRC 断面にはさらに内蔵鉄骨（SN490B）を、モデルへ足して割り当てる。
fn push_section(m: &mut Model, mut sec: Section) {
    sec.material = Some(MaterialId(0));
    let has_rebar = matches!(
        sec.shape,
        Some(
            SectionShape::RcRect { .. }
                | SectionShape::RcCircle { .. }
                | SectionShape::SrcRect { .. }
                | SectionShape::RcWall { .. }
        )
    );
    if has_rebar {
        let id = ensure_material(m, "SD345", MaterialCategory::Rebar, Some(345.0));
        sec.rebar_material = Some(id);
        sec.shear_rebar_material = Some(id);
    }
    if matches!(sec.shape, Some(SectionShape::SrcRect { .. })) {
        let id = ensure_material(m, "SN490B", MaterialCategory::Steel, Some(325.0));
        sec.steel_material = Some(id);
    }
    m.sections.push(sec);
}

/// 同名の材料があればその id を、なければ足してその id を返す。
fn ensure_material(
    m: &mut Model,
    name: &str,
    category: MaterialCategory,
    fy: Option<f64>,
) -> MaterialId {
    if let Some(found) = m.materials.iter().find(|x| x.name == name) {
        return found.id;
    }
    let id = MaterialId(m.materials.len() as u32);
    m.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id,
        name: name.into(),
        category,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy,
    });
    id
}

/// Raw モード（既定）は従来どおり StbSecRaw を出力し、標準要素は出さない。
/// 標準モード: 鋼 H 断面が形鋼ライブラリ参照付きの StbSecColumn_S として出力される。
#[test]
fn test_standard_mode_steel_column() {
    let mut m = frame_nodes();
    let h = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    };
    push_section(&mut m, h.to_section(SectionId(0), "C1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecColumn_S "), "鋼柱は StbSecColumn_S");
    assert!(xml.contains("<StbSecSteel>"), "形鋼ライブラリを出す");
    assert!(
        xml.contains("<StbSecRoll-H name=\"H-400x200x8x13\""),
        "H 形鋼図形が定義される: {xml}"
    );
    assert!(
        xml.contains("shape=\"H-400x200x8x13\""),
        "断面が図形名を参照する"
    );
    assert!(
        !xml.contains("<StbSecRaw "),
        "形状がある鋼断面は Raw にしない"
    );
}

/// 標準モード: RC 矩形が梁として使われると StbSecBeam_RC（幾何）で出力される。
#[test]
fn test_standard_mode_rc_beam() {
    let mut m = frame_nodes();
    let rc = SectionShape::RcRect {
        b: 400.0,
        d: 700.0,
        rebar: rebar(),
    };
    push_section(&mut m, rc.to_section(SectionId(0), "G1".into()));
    m.elements.push(member(0, false, 0)); // 梁

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecBeam_RC "), "RC 梁は StbSecBeam_RC");
    assert!(
        xml.contains("<StbSecBeam_RC_Straight width=\"400\" depth=\"700\"/>"),
        "矩形図形が幅・せいで出力される: {xml}"
    );
}

/// 標準モード: 柱と梁で共有される鋼断面は 2 要素に分割され、部材の id_section が
/// それぞれ別 id を指す。
#[test]
fn test_standard_mode_shared_section_split() {
    let mut m = frame_nodes();
    let h = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    };
    push_section(&mut m, h.to_section(SectionId(0), "S1".into()));
    m.elements.push(member(0, true, 0)); // 柱が section 0 を使用
    m.elements.push(member(1, false, 0)); // 梁も section 0 を使用（共有）

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecColumn_S "), "柱用に StbSecColumn_S");
    assert!(xml.contains("<StbSecBeam_S "), "梁用に StbSecBeam_S");
    // 形鋼図形は 1 つに重複排除される。
    assert_eq!(
        xml.matches("<StbSecRoll-H ").count(),
        1,
        "形鋼図形は重複排除される"
    );
    // id は 1 始まり（positiveInteger）。柱は id_section=1、梁は分割された新 id=2 を参照。
    assert!(
        xml.contains("<StbColumn ") && xml.contains("id_section=\"1\""),
        "柱は元の断面 id を参照: {xml}"
    );
    assert!(
        xml.contains("<StbGirder ") && xml.contains("id_section=\"2\""),
        "梁は分割された新しい断面 id を参照: {xml}"
    );
}

/// 標準モード: 形状を持たない断面（SRC/CFT/未定義含む）は StbSecRaw へフォールバックする。
#[test]
fn test_standard_mode_fallback_raw_for_shapeless() {
    let mut m = frame_nodes();
    m.sections.push(Section {
        id: SectionId(0),
        name: "X1".into(),
        area: 1.0e4,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e6,
        depth: 300.0,
        width: 300.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    m.elements.push(member(0, true, 0));

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecRaw "),
        "形状のない断面は Raw にフォールバック"
    );
}

/// 標準モードで書き出したファイルを import で読み戻せる（往復）。
/// 鋼 H（柱）＋ RC 矩形（梁）が形状・断面性能とも復元され、検証を通る。
#[test]
fn test_standard_import_roundtrip_steel_and_rc() {
    let mut m = frame_nodes();
    let h = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    };
    push_section(&mut m, h.to_section(SectionId(0), "C1".into()));
    let rc = SectionShape::RcRect {
        b: 400.0,
        d: 700.0,
        rebar: rebar(),
    };
    push_section(&mut m, rc.to_section(SectionId(1), "G1".into()));
    m.elements.push(member(0, true, 0)); // 柱 → 鋼断面
    m.elements.push(member(1, false, 1)); // 梁 → RC 断面

    let xml = export_stbridge(&m).unwrap();
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());

    assert_eq!(back.sections.len(), 2);
    assert!(
        matches!(back.sections[0].shape, Some(SectionShape::SteelH { .. })),
        "鋼柱断面の形状が復元される: {:?}",
        back.sections[0].shape
    );
    assert!(
        matches!(back.sections[1].shape, Some(SectionShape::RcRect { .. })),
        "RC 梁断面の形状が復元される: {:?}",
        back.sections[1].shape
    );
    // 断面性能（弾性）は形状から再算定され、元と一致する。
    assert_eq!(back.sections[0].area, m.sections[0].area);
    assert_eq!(back.sections[0].iy, m.sections[0].iy);
    assert_eq!(back.sections[0].iz, m.sections[0].iz);
    assert_eq!(back.sections[1].area, m.sections[1].area);
    assert_eq!(back.sections[1].iy, m.sections[1].iy);
    // 部材の断面参照が正しく張り替わる。
    assert_eq!(back.elements[0].section, Some(SectionId(0)));
    assert_eq!(back.elements[1].section, Some(SectionId(1)));
}

/// 方向別に異なる本数・径・段数・かぶり・帯筋を持つ配筋（往復ずれ・取り違えを検出）。
/// 標準 ST-Bridge が保存できる配筋（主筋本数は X/Y で別、径は単一 `D_main`、1 段）。
/// ST-Bridge の主筋径は `D_main` 1 つ・段別本数のみのため、X/Y で径を変えたり多段に
/// したりは標準では往復しない（[`super`] モジュールドキュメント参照）。
fn rebar_distinct() -> RcRebar {
    RcRebar {
        main_x: BarSet {
            count: 4,
            dia: 25.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 3,
            dia: 25.0,
            layers: 1,
        },
        cover: 45.0,
        shear: ShearBar {
            dia: 13.0,
            pitch: 150.0,
            legs: 4,
        },
    }
}

/// 標準モード: RC 矩形柱の配筋（主筋・帯筋・かぶり）が往復で完全に保存される。
#[test]
fn test_standard_roundtrip_rc_rect_column_rebar() {
    let mut m = frame_nodes();
    let shape = SectionShape::RcRect {
        b: 600.0,
        d: 700.0,
        rebar: rebar_distinct(),
    };
    push_section(&mut m, shape.to_section(SectionId(0), "C1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecBarArrangementColumn_RC "),
        "柱配筋要素が書き出される: {xml}"
    );
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    // 形状（b・d・配筋すべて）が完全一致で復元される。
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "RC 矩形柱の配筋が往復で保存される"
    );
}

/// 標準モード: RC 円形柱の配筋が往復で完全に保存される。
#[test]
fn test_standard_roundtrip_rc_circle_column_rebar() {
    let mut m = frame_nodes();
    let shape = SectionShape::RcCircle {
        d: 800.0,
        rebar: rebar_distinct(),
    };
    push_section(&mut m, shape.to_section(SectionId(0), "C1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecBarColumn_RC_CircleSame "),
        "円形配筋要素: {xml}"
    );
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "RC 円形柱の配筋が往復で保存される"
    );
}

/// 標準モード: RC 矩形梁の配筋が往復で完全に保存される。
#[test]
fn test_standard_roundtrip_rc_beam_rebar() {
    let mut m = frame_nodes();
    let shape = SectionShape::RcRect {
        b: 400.0,
        d: 700.0,
        rebar: rebar_distinct(),
    };
    push_section(&mut m, shape.to_section(SectionId(0), "G1".into()));
    m.elements.push(member(0, false, 0)); // 梁

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecBarArrangementBeam_RC "),
        "梁配筋要素が書き出される: {xml}"
    );
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "RC 梁の配筋が往復で保存される"
    );
}

/// file id が重複する STB は fail-loud でエラーにする（無言のジオメトリ破損防止）。
/// 重複 id があると「配列添字 == id.index()」の不変条件が壊れ、部材が別実体の
/// 節点を参照してしまうため、取り込み時に検出してエラーとする。
#[test]
fn test_import_duplicate_node_id_is_error() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="0" X="1000" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
</StbModel></ST_BRIDGE>"#;
    let r = import_stbridge(xml);
    assert!(
        r.is_err(),
        "重複 file id はエラーにすべき（無言のジオメトリ破損を防ぐ）"
    );
}

/// 配筋要素のない（幾何のみの）RC 断面ファイルも、無筋相当の既定配筋で読める。
#[test]
fn test_import_rc_without_bar_arrangement_uses_default() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbSections>
    <StbSecColumn_RC id="0" name="C"><StbSecFigureColumn_RC><StbSecColumn_RC_Rect width_X="500" width_Y="600"/></StbSecFigureColumn_RC></StbSecColumn_RC>
  </StbSections>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert_eq!(m.sections.len(), 1);
    match &m.sections[0].shape {
        Some(SectionShape::RcRect { b, d, .. }) => {
            assert_eq!(*b, 500.0);
            assert_eq!(*d, 600.0);
        }
        other => panic!("RcRect を期待: {other:?}"),
    }
}

/// 標準モードで柱・梁に分割された共有鋼断面が、import で元の 1 断面へ統合され、
/// 両部材が同じ id を参照する（検証を通る）。
///
/// 書き出しは共有断面を柱用（`StbSecColumn_S`）・梁用（`StbSecBeam_S`）へ分割するが、
/// 分割後の 2 定義は符号も階も同じで断面性能も一致する。断面の同一性キーは符号＋階
/// なので、取り込みでこの 2 定義は 1 件へ統合され、往復で断面が増えない。
#[test]
fn test_standard_import_recovers_split_shared_section() {
    let mut m = frame_nodes();
    let h = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    };
    push_section(&mut m, h.to_section(SectionId(0), "S1".into()));
    m.elements.push(member(0, true, 0)); // 柱
    m.elements.push(member(1, false, 0)); // 梁（同じ断面を共有）

    let xml = export_stbridge(&m).unwrap();
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections.len(),
        1,
        "分割された 2 定義は符号＋階と断面性能が一致するため 1 断面へ統合される"
    );
    assert!(
        matches!(back.sections[0].shape, Some(SectionShape::SteelH { .. })),
        "H 形鋼として復元される"
    );
    assert_eq!(back.sections[0].name, "S1");
    assert_eq!(back.elements[0].section, Some(SectionId(0)));
    assert_eq!(
        back.elements[1].section,
        Some(SectionId(0)),
        "柱・梁とも統合後の同じ断面を参照する"
    );
}

/// 符号＋階が同じでも**材料が違えば統合しない**。
///
/// 材料は断面が持つため、材料だけが違う定義を 1 断面へまとめると片方の材料が
/// 無言で捨てられる。符号へ連番を付けて両方の定義を残す。
#[test]
fn test_import_does_not_merge_sections_with_different_materials() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
    <StbNode id="2" X="6000" Y="0" Z="3000"/>
  </StbNodes>
  <StbMaterials>
    <StbMaterial id="0" name="SN400B" young="205000" poisson="0.3" density="0"/>
    <StbMaterial id="1" name="SN490B" young="205000" poisson="0.3" density="0"/>
  </StbMaterials>
  <StbSections>
    <StbSecColumn_S id="0" name="C1"><StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="H1" strength_main="SN400B"/></StbSecSteelFigureColumn_S></StbSecColumn_S>
    <StbSecBeam_S id="1" name="C1"><StbSecSteelFigureBeam_S><StbSecSteelBeam_S_Straight shape="H1" strength_main="SN490B"/></StbSecSteelFigureBeam_S></StbSecBeam_S>
    <StbSecSteel><StbSecRoll-H name="H1" type="H" A="300" B="150" t1="6.5" t2="9" r="0"/></StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/>
    <StbGirder id="1" id_node_start="1" id_node_end="2" id_section="1"/>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(
        m.sections.len(),
        2,
        "材料が違うので統合されず 2 断面のまま: {:?}",
        m.sections.iter().map(|s| &s.name).collect::<Vec<_>>()
    );
    // それぞれが自分の材料を持つ（片方が無言で捨てられていない）。
    let names: Vec<Option<&str>> = m
        .sections
        .iter()
        .map(|s| {
            s.material
                .and_then(|id| m.materials.get(id.index()))
                .map(|mm| mm.name.as_str())
        })
        .collect();
    assert!(names.contains(&Some("SN400B")), "{names:?}");
    assert!(names.contains(&Some("SN490B")), "{names:?}");
    // 衝突した符号は連番を付けて残したことを報告する。
    assert!(
        report
            .notes
            .iter()
            .chain(report.warnings.iter())
            .any(|n| n.contains("C1")),
        "符号の改番を報告する: notes={:?} warnings={:?}",
        report.notes,
        report.warnings
    );
}

/// 同じ断面を指す部材が別々の `id_material` を持つファイルは、先に解決した材料を
/// 採ったうえで**警告する**。材料は断面が持つため後勝ちの材料は行き場がなく、
/// 黙って捨てると利用者が食い違いに気づけない。
#[test]
fn test_import_warns_when_members_conflict_on_section_material() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
    <StbNode id="2" X="6000" Y="0" Z="0"/>
    <StbNode id="3" X="6000" Y="0" Z="3000"/>
  </StbNodes>
  <StbMaterials>
    <StbMaterial id="0" name="SN400B" young="205000" poisson="0.3" density="0"/>
    <StbMaterial id="1" name="SN490B" young="205000" poisson="0.3" density="0"/>
  </StbMaterials>
  <StbSections>
    <StbSecColumn_S id="0" name="C1"><StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="H1"/></StbSecSteelFigureColumn_S></StbSecColumn_S>
    <StbSecSteel><StbSecRoll-H name="H1" type="H" A="300" B="150" t1="6.5" t2="9" r="0"/></StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0" id_material="0"/>
    <StbColumn id="1" id_node_bottom="2" id_node_top="3" id_section="0" id_material="1"/>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(m.sections.len(), 1, "断面定義は 1 件");
    // 先に解決した材料（柱 0 の SN400B）が断面へ付く。
    let name = m.sections[0]
        .material
        .and_then(|id| m.materials.get(id.index()))
        .map(|mm| mm.name.as_str());
    assert_eq!(name, Some("SN400B"));
    assert!(
        report.warnings.iter().any(|w| w.contains("別々の材料")),
        "材料の食い違いを警告する: {:?}",
        report.warnings
    );
}

/// 柱・梁で共有する RC 矩形断面が、配筋ごと往復する。
/// 書き出しで柱用・梁用へ分割されるが、取り込みで 1 断面へ統合される。
#[test]
fn test_standard_roundtrip_shared_rc_rect_rebar() {
    let mut m = frame_nodes();
    let shape = SectionShape::RcRect {
        b: 500.0,
        d: 800.0,
        rebar: rebar_distinct(),
    };
    push_section(&mut m, shape.to_section(SectionId(0), "RC1".into()));
    m.elements.push(member(0, true, 0)); // 柱
    m.elements.push(member(1, false, 0)); // 梁（共有）

    let back = import_stbridge(&export_stbridge(&m).unwrap()).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections.len(),
        1,
        "分割された共有 RC 断面は 1 件へ統合される"
    );
    // 元の形状・配筋が保存されている。
    assert_eq!(back.sections[0].shape, m.sections[0].shape);
    assert_eq!(back.elements[0].section, Some(SectionId(0)));
    assert_eq!(back.elements[1].section, Some(SectionId(0)));
}

/// せん断補強筋の材料が未割当でも配筋は完全一致で往復する
/// （strength_band 属性を出力しない経路）。
#[test]
fn test_standard_roundtrip_rc_rebar_without_shear_material() {
    let mut m = frame_nodes();
    let shape = SectionShape::RcRect {
        b: 400.0,
        d: 600.0,
        rebar: rebar_distinct(),
    };
    let mut sec = shape.to_section(SectionId(0), "C1".into());
    sec.shear_rebar_material = None;
    m.sections.push(sec);
    m.elements.push(member(0, true, 0));

    let back = import_stbridge(&export_stbridge(&m).unwrap()).expect("import");
    assert_eq!(back.sections[0].shape, m.sections[0].shape);
    assert_eq!(back.sections[0].shear_rebar_material, None);
}

/// 非整数の径・ピッチ・かぶりも桁落ちなく往復する。
#[test]
fn test_standard_roundtrip_rc_rebar_non_integer() {
    let mut m = frame_nodes();
    // 主筋径は単一 `D_main`・1 段のみ標準往復する（X/Y で径・段数は変えない）。
    let r = RcRebar {
        main_x: BarSet {
            count: 6,
            dia: 12.7,
            layers: 1,
        },
        main_y: BarSet {
            count: 4,
            dia: 12.7,
            layers: 1,
        },
        cover: 40.5,
        shear: ShearBar {
            dia: 6.35,
            pitch: 133.3,
            legs: 2,
        },
    };
    let shape = SectionShape::RcRect {
        b: 450.0,
        d: 650.0,
        rebar: r,
    };
    push_section(&mut m, shape.to_section(SectionId(0), "C1".into()));
    m.elements.push(member(0, true, 0));

    let back = import_stbridge(&export_stbridge(&m).unwrap()).expect("import");
    assert_eq!(back.sections[0].shape, m.sections[0].shape);
}

/// 帯筋の材料名にタブ等の制御空白が含まれても往復で保存される（esc の制御文字対策）。
#[test]
fn test_standard_roundtrip_shear_rebar_material_with_control_chars() {
    let mut m = frame_nodes();
    let name = "KH\t785\nX";
    m.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(m.materials.len() as u32),
        name: name.into(),
        category: MaterialCategory::Rebar,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: Some(785.0),
    });
    let shear_mat = MaterialId(m.materials.len() as u32 - 1);
    let shape = SectionShape::RcRect {
        b: 400.0,
        d: 700.0,
        rebar: rebar_distinct(),
    };
    let mut sec = shape.to_section(SectionId(0), "C1".into());
    sec.shear_rebar_material = Some(shear_mat);
    m.sections.push(sec);
    m.elements.push(member(0, true, 0));

    let back = import_stbridge(&export_stbridge(&m).unwrap()).expect("import");
    assert_eq!(back.sections[0].shape, m.sections[0].shape);
    let back_mat = back.sections[0]
        .shear_rebar_material
        .map(|id| back.materials[id.index()].name.as_str());
    assert_eq!(
        back_mat,
        Some(name),
        "制御空白を含む材料名が往復で保存される"
    );
}

/// 円形 RC を梁に使うと（ST-Bridge に円形梁図形がないため）StbSecRaw へフォールバックし、
/// 形状・配筋は失われるが物性は残り、検証は通る（ドキュメント化された既知の挙動）。
#[test]
fn test_standard_rc_circle_beam_falls_back_to_raw() {
    let mut m = frame_nodes();
    let shape = SectionShape::RcCircle {
        d: 700.0,
        rebar: rebar_distinct(),
    };
    push_section(&mut m, shape.to_section(SectionId(0), "CB1".into()));
    m.elements.push(member(0, false, 0)); // 梁（水平材）で円形を使う

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecRaw "), "円形梁は Raw にフォールバック");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    // 形状・配筋は失われる（shape=None）が、弾性物性は残る。
    assert!(back.sections[0].shape.is_none(), "円形梁は形状が往復しない");
    assert_eq!(back.sections[0].area, m.sections[0].area, "物性は残る");
}

/// 実 ST-Bridge 風の配筋属性（呼び名径 D22・標準名 D_band/N_main_X_1st）を best-effort で読む。
#[test]
fn test_import_rc_rebar_third_party_names() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbSections>
    <StbSecColumn_RC id="0" name="C">
      <StbSecFigureColumn_RC><StbSecColumn_RC_Rect width_X="600" width_Y="600"/></StbSecFigureColumn_RC>
      <StbSecBarArrangementColumn_RC>
        <StbSecBarColumn_RC_RectSame N_main_X_1st="4" N_main_Y_1st="3" D_main="D22" D_band="D10" pitch_band="100"/>
      </StbSecBarArrangementColumn_RC>
    </StbSecColumn_RC>
  </StbSections>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    match &m.sections[0].shape {
        Some(SectionShape::RcRect { rebar, .. }) => {
            assert_eq!(rebar.main_x.count, 4);
            assert_eq!(rebar.main_y.count, 3);
            assert_eq!(rebar.main_x.dia, 22.0, "呼び名 D22 → 22mm");
            assert_eq!(rebar.shear.dia, 10.0, "呼び名 D10 → 10mm");
            assert_eq!(rebar.shear.pitch, 100.0);
        }
        other => panic!("RcRect を期待: {other:?}"),
    }
}

/// 実 ST-Bridge の段別主筋本数（`N_main_X_1st`/`_2nd`、梁の `N_main_bottom`/`_2nd`）を
/// 合算し、非ゼロの段数を `layers` に反映することを確認する。
#[test]
fn test_import_rc_rebar_layered_counts() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbSections>
    <StbSecColumn_RC id="0" name="C">
      <StbSecFigureColumn_RC><StbSecColumn_RC_Rect width_X="700" width_Y="700"/></StbSecFigureColumn_RC>
      <StbSecBarArrangementColumn_RC>
        <StbSecBarColumn_RC_RectSame N_main_X_1st="4" N_main_X_2nd="3" N_main_Y_1st="5" D_main="D25" D_band="D13" pitch_band="100"/>
      </StbSecBarArrangementColumn_RC>
    </StbSecColumn_RC>
  </StbSections>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    match &m.sections[0].shape {
        Some(SectionShape::RcRect { rebar, .. }) => {
            assert_eq!(rebar.main_x.count, 7, "X 方向は 1・2 段目を合算 (4+3)");
            assert_eq!(rebar.main_x.layers, 2, "非ゼロの段数 = 2");
            assert_eq!(rebar.main_y.count, 5, "Y 方向は 1 段目のみ");
            assert_eq!(rebar.main_y.layers, 1, "非ゼロの段数 = 1");
            assert_eq!(rebar.main_x.dia, 25.0, "呼び名 D25 → 25mm");
        }
        other => panic!("RcRect を期待: {other:?}"),
    }
}

/// 実 ST-Bridge の鋼管形鋼ライブラリ名（`StbSecRoll-Pipe`）を取り込み、鋼管柱の
/// 断面性能（物性ゼロでない）を復元できることを確認する。Squid 方言（`StbSecPipe`）
/// だけでなく標準名も受けることの回帰テスト。
#[test]
fn test_import_steel_roll_pipe_library() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="P1">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="P-267.4x6" strength_main="STKN400B"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecSteel>
      <StbSecRoll-Pipe name="P-267.4x6" D="267.4" t="6"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    // 形鋼参照が解決され、物性ゼロの警告が出ていないこと。
    assert!(
        report.warnings.iter().all(|w| !w.contains("物性ゼロ")),
        "鋼管の形鋼参照が解決されるべき: {:?}",
        report.warnings
    );
    let sec = &m.sections[0];
    assert!(
        sec.area > 0.0,
        "鋼管断面の断面積が復元される: A={}",
        sec.area
    );
    match &sec.shape {
        Some(SectionShape::SteelPipe { outer_dia, thick }) => {
            assert_eq!(*outer_dia, 267.4);
            assert_eq!(*thick, 6.0);
        }
        other => panic!("SteelPipe を期待: {other:?}"),
    }
}

/// 実 ST-Bridge の階所属（`StbStory` 直下 `StbNodeIdList/StbNodeId`）を取り込み、
/// 節点の `story` と `Story.node_ids` の双方へ反映することを確認する。
#[test]
fn test_import_story_node_list() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="0" Y="0" Z="3000"/>
    <StbNode id="3" X="4000" Y="0" Z="3000"/>
  </StbNodes>
  <StbStories>
    <StbStory id="0" name="1F" height="0"/>
    <StbStory id="1" name="2F" height="3000">
      <StbNodeIdList>
        <StbNodeId id="2"/>
        <StbNodeId id="3"/>
      </StbNodeIdList>
    </StbStory>
  </StbStories>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    // 節点 2・3 は 2F（StoryId(1)）に所属し、0・1 はいずれの階にも属さない。
    assert_eq!(m.nodes[2].story, Some(StoryId(1)), "節点2 → 2F");
    assert_eq!(m.nodes[3].story, Some(StoryId(1)), "節点3 → 2F");
    assert_eq!(m.nodes[0].story, None, "節点0 は階リスト外");
    // Story.node_ids へも反映される。
    assert_eq!(
        m.stories[1].node_ids,
        vec![NodeId(2), NodeId(3)],
        "2F の所属節点"
    );
    assert!(m.stories[0].node_ids.is_empty(), "1F は所属節点なし");
}

/// 標準モード: 平鋼（中実矩形）が `StbSecColumn_S`＋`StbSecRoll-FlatBar` として往復する。
#[test]
fn test_standard_roundtrip_flat_bar() {
    let mut m = frame_nodes();
    let shape = SectionShape::SteelFlatBar {
        width: 100.0,
        thick: 12.0,
    };
    push_section(&mut m, shape.to_section(SectionId(0), "FB1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecColumn_S "), "鋼柱要素: {xml}");
    assert!(xml.contains("<StbSecRoll-FlatBar "), "平鋼の形鋼ライブラリ");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(back.sections[0].shape, m.sections[0].shape, "平鋼が往復");
    // 断面性能（中実矩形）が算定されている。
    assert!((back.sections[0].area - 1200.0).abs() < 1e-6, "A=width·t");
}

/// 標準モード: 中実丸鋼が `StbSecColumn_S`＋`StbSecRoll-RoundBar` として往復する。
#[test]
fn test_standard_roundtrip_round_bar() {
    let mut m = frame_nodes();
    let shape = SectionShape::SteelRoundBar { dia: 32.0 };
    push_section(&mut m, shape.to_section(SectionId(0), "RB1".into()));
    m.elements.push(member(0, true, 0));

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecRoll-RoundBar "),
        "中実丸鋼の形鋼ライブラリ"
    );
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "中実丸鋼が往復"
    );
}

/// import: 実 ST-Bridge の平鋼・丸鋼ライブラリ名を直接読み取れる。
#[test]
fn test_import_flat_and_round_bar_library() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="FB">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="FB-90x9"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecColumn_S id="1" name="RB">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="RB-25"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecSteel>
      <StbSecRoll-FlatBar name="FB-90x9" B="90" t="9"/>
      <StbSecRoll-RoundBar name="RB-25" D="25"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    let shapes: Vec<_> = m.sections.iter().map(|s| s.shape.clone()).collect();
    assert!(
        shapes.contains(&Some(SectionShape::SteelFlatBar {
            width: 90.0,
            thick: 9.0
        })),
        "平鋼が復元される: {shapes:?}"
    );
    assert!(
        shapes.contains(&Some(SectionShape::SteelRoundBar { dia: 25.0 })),
        "中実丸鋼が復元される: {shapes:?}"
    );
}

/// 標準モード: リップ溝形が `StbSecColumn_S`＋`StbSecRoll-LipC` として往復する。
#[test]
fn test_standard_roundtrip_lip_channel() {
    let mut m = frame_nodes();
    let shape = SectionShape::SteelLipChannel {
        height: 150.0,
        width: 75.0,
        lip: 20.0,
        thick: 2.3,
    };
    push_section(&mut m, shape.to_section(SectionId(0), "LipC1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecRoll-LipC "),
        "リップ溝形の形鋼ライブラリ: {xml}"
    );
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "リップ溝形が往復"
    );
}

/// import: 実 ST-Bridge のリップ溝形ライブラリ名（`StbSecRoll-LipC`）を直接読み取れる。
#[test]
fn test_import_lip_channel_library() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="LC">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="LipC-200x75x20x3.2"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecSteel>
      <StbSecRoll-LipC name="LipC-200x75x20x3.2" A="200" B="75" C="20" t="3.2"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(
        report.warnings.iter().all(|w| !w.contains("物性ゼロ")),
        "リップ溝形の形鋼参照が解決されるべき: {:?}",
        report.warnings
    );
    assert_eq!(
        m.sections[0].shape,
        Some(SectionShape::SteelLipChannel {
            height: 200.0,
            width: 75.0,
            lip: 20.0,
            thick: 3.2
        }),
        "リップ溝形が復元される"
    );
    assert!(m.sections[0].area > 0.0);
}

/// 標準モード: 非対称組立 H が `StbSecBuild-H`（下フランジ方言属性付き）として往復する。
#[test]
fn test_standard_roundtrip_built_h() {
    let mut m = frame_nodes();
    let shape = SectionShape::SteelBuiltH {
        height: 500.0,
        upper_width: 150.0,
        upper_thick: 9.0,
        lower_width: 300.0,
        lower_thick: 19.0,
        web_thick: 9.0,
    };
    push_section(&mut m, shape.to_section(SectionId(0), "BH1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecBuild-H "),
        "組立 H の形鋼ライブラリ: {xml}"
    );
    assert!(xml.contains("B2="), "下フランジの方言属性が付く");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "非対称組立 H が完全往復"
    );
}

/// import: `StbSecBuild-H`（下フランジ属性なし＝第三者の対称 H）は `SteelH` として読む。
#[test]
fn test_import_symmetric_build_h_is_steel_h() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="BH">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="BH-400"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecSteel>
      <StbSecBuild-H name="BH-400" A="400" B="200" t1="8" t2="12"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert_eq!(
        m.sections[0].shape,
        Some(SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 8.0,
            flange_thick: 12.0
        }),
        "下フランジ属性がなければ対称 H"
    );
}

/// 標準モード: 角形鋼管柱の角部外半径 r（`StbSecRoll-BOX` の r 属性）が
/// `SectionShape::SteelBox.corner_r` として完全往復する。
#[test]
fn test_standard_roundtrip_steel_box_corner_r() {
    let mut m = frame_nodes();
    let shape = SectionShape::SteelBox {
        height: 300.0,
        width: 300.0,
        thick: 12.0,
        corner_r: 30.0,
    };
    push_section(&mut m, shape.to_section(SectionId(0), "BOX1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("r=\"30\""), "角部外半径 r が出力される: {xml}");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "角形鋼管の角部外半径 r が完全往復"
    );
}

/// 同寸で角部半径だけ異なる 2 つの角形鋼管が、それぞれの r を保って往復すること。
/// 形鋼ライブラリは名前で重複排除するため、従来は名前に corner_r が含まれず
/// 同一名に潰れ、後着断面の r が先着の値に化けていた。
#[test]
fn test_standard_roundtrip_steel_box_distinct_corner_r() {
    let mut m = frame_nodes();
    let with_r = SectionShape::SteelBox {
        height: 300.0,
        width: 300.0,
        thick: 12.0,
        corner_r: 30.0,
    };
    let without_r = SectionShape::SteelBox {
        height: 300.0,
        width: 300.0,
        thick: 12.0,
        corner_r: 0.0,
    };
    push_section(&mut m, with_r.to_section(SectionId(0), "BOX-R30".into()));
    push_section(&mut m, without_r.to_section(SectionId(1), "BOX-R0".into()));
    m.elements.push(member(0, true, 0));
    m.elements.push(member(1, true, 1));

    let xml = export_stbridge(&m).unwrap();
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "corner_r=30 の断面が自身の r を保って往復"
    );
    // corner_r=0 は ST-Bridge スキーマ（r は正値必須）の制約で便宜値 r=t として
    // 出力されるため、再取り込みでは corner_r=t になる（既存仕様）。ここで
    // 検証するのは「同寸別 r の断面（corner_r=30）の値に化けない」こと。
    match back.sections[1].shape {
        Some(SectionShape::SteelBox {
            corner_r, thick, ..
        }) => {
            assert_eq!(
                corner_r, thick,
                "corner_r=0 の断面は便宜値 r=t のまま（別断面の r=30 に化けない）"
            );
        }
        ref other => panic!("SteelBox のはず: {:?}", other),
    }
}

/// import: `r` 属性がない `StbSecRoll-BOX` は角部直角（corner_r=0.0）として読む。
#[test]
fn test_import_box_without_r_attr_is_corner_r_zero() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="BOX">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="BOX-300"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecSteel>
      <StbSecRoll-BOX name="BOX-300" type="ELSE" A="300" B="300" t="12"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert_eq!(
        m.sections[0].shape,
        Some(SectionShape::SteelBox {
            height: 300.0,
            width: 300.0,
            thick: 12.0,
            corner_r: 0.0,
        }),
        "r 属性がなければ角部直角（corner_r=0.0）"
    );
}

/// 標準モード: CFT 角形柱が `StbSecColumn_CFT`＋形鋼ライブラリとして往復する。
#[test]
fn test_standard_roundtrip_cft_box() {
    let mut m = frame_nodes();
    let shape = SectionShape::CftBox {
        height: 400.0,
        width: 400.0,
        thick: 16.0,
    };
    push_section(&mut m, shape.to_section(SectionId(0), "CFT1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecColumn_CFT "), "CFT 柱要素: {xml}");
    assert!(xml.contains("<StbSecRoll-BOX "), "充填鋼管の形鋼ライブラリ");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "CFT 角形が往復"
    );
}

/// 標準モード: CFT 円形柱が往復する。
#[test]
fn test_standard_roundtrip_cft_pipe() {
    let mut m = frame_nodes();
    let shape = SectionShape::CftPipe {
        outer_dia: 500.0,
        thick: 12.0,
    };
    push_section(&mut m, shape.to_section(SectionId(0), "CFT2".into()));
    m.elements.push(member(0, true, 0));

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecColumn_CFT "));
    assert!(xml.contains("<StbSecPipe "));
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "CFT 円形が往復"
    );
}

/// 標準モード: SRC 柱（コンクリート＋内蔵鉄骨＋配筋＋鋼種）が完全に往復する。
#[test]
fn test_standard_roundtrip_src_column() {
    let mut m = frame_nodes();
    let shape = SectionShape::SrcRect {
        b: 800.0,
        d: 800.0,
        rebar: rebar_distinct(),
        steel_height: 400.0,
        steel_width: 200.0,
        steel_web_thick: 8.0,
        steel_flange_thick: 13.0,
    };
    push_section(&mut m, shape.to_section(SectionId(0), "SRC1".into()));
    m.elements.push(member(0, true, 0)); // 柱

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecColumn_SRC "), "SRC 柱要素: {xml}");
    assert!(
        xml.contains("strength_steel=\"SN490B\""),
        "鋼種が書き出される"
    );
    assert!(xml.contains("<StbSecRoll-H "), "内蔵鉄骨の形鋼ライブラリ");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(
        back.sections[0].shape, m.sections[0].shape,
        "SRC 柱が形状・配筋・内蔵鉄骨・鋼種とも往復する"
    );
}

/// 標準モード: SRC 梁も往復する（`StbSecBeam_SRC`）。
#[test]
fn test_standard_roundtrip_src_beam() {
    let mut m = frame_nodes();
    let shape = SectionShape::SrcRect {
        b: 500.0,
        d: 800.0,
        rebar: rebar_distinct(),
        steel_height: 450.0,
        steel_width: 200.0,
        steel_web_thick: 9.0,
        steel_flange_thick: 14.0,
    };
    push_section(&mut m, shape.to_section(SectionId(0), "SG1".into()));
    m.elements.push(member(0, false, 0)); // 梁

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecBeam_SRC "), "SRC 梁要素: {xml}");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert_eq!(back.sections[0].shape, m.sections[0].shape, "SRC 梁が往復");
}

/// CFT を梁に使うと（ST-Bridge に CFT 梁がないため）Raw へフォールバックする。
#[test]
fn test_standard_cft_beam_falls_back_to_raw() {
    let mut m = frame_nodes();
    let shape = SectionShape::CftBox {
        height: 300.0,
        width: 300.0,
        thick: 12.0,
    };
    push_section(&mut m, shape.to_section(SectionId(0), "CB".into()));
    m.elements.push(member(0, false, 0)); // 梁

    let xml = export_stbridge(&m).unwrap();
    assert!(xml.contains("<StbSecRaw "), "CFT 梁は Raw にフォールバック");
    let back = import_stbridge(&xml).expect("import");
    assert!(back.validate().is_ok(), "{:?}", back.validate());
    assert!(back.sections[0].shape.is_none());
}

/// 形鋼ライブラリが断面要素より後ろに現れても解決できる（順序非依存）。
#[test]
fn test_standard_import_steel_library_order_independent() {
    // export は StbSecSteel を末尾に書き出す。これを import できることを確認する。
    let mut m = frame_nodes();
    let h = SectionShape::SteelH {
        height: 350.0,
        width: 175.0,
        web_thick: 7.0,
        flange_thick: 11.0,
    };
    push_section(&mut m, h.to_section(SectionId(0), "C1".into()));
    m.elements.push(member(0, true, 0));

    let xml = export_stbridge(&m).unwrap();
    // 形鋼ライブラリが断面要素の後ろにあること（前提の確認）。
    let steel_pos = xml.find("<StbSecSteel>").unwrap();
    let col_pos = xml.find("<StbSecColumn_S").unwrap();
    assert!(col_pos < steel_pos, "前提: 断面要素 → 形鋼ライブラリの順");

    let back = import_stbridge(&xml).expect("import");
    assert!(
        matches!(back.sections[0].shape, Some(SectionShape::SteelH { .. })),
        "後方の形鋼ライブラリを解決して形状復元"
    );
    assert_eq!(back.sections[0].area, m.sections[0].area);
}

/// 他社ファイルでよくある 1 始まり・非連番の id（node/material/section/member）を
/// 0 始まり連番へ正規化し、参照を張り替えて検証を通す。
#[test]
fn test_import_normalizes_noncontiguous_ids() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="11" X="0" Y="0" Z="0"/>
    <StbNode id="12" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbMaterials><StbMaterial id="5" name="SN400B" young="205000" poisson="0.3" density="0"/></StbMaterials>
  <StbSections>
    <StbSecColumn_S id="9" name="C1"><StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="H1"/></StbSecSteelFigureColumn_S></StbSecColumn_S>
    <StbSecSteel><StbSecRoll-H name="H1" type="H" A="300" B="150" t1="6.5" t2="9" r="0"/></StbSecSteel>
  </StbSections>
  <StbMembers><StbColumn id="7" id_node_bottom="11" id_node_top="12" id_section="9" id_material="5" rx="0" ry="1" rz="0"/></StbMembers>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert!(
        m.validate().is_ok(),
        "非連番 id を正規化して検証を通る: {:?}",
        m.validate()
    );
    assert_eq!(m.nodes.len(), 2);
    assert_eq!(m.nodes[0].id, NodeId(0));
    assert_eq!(m.nodes[1].id, NodeId(1));
    assert_eq!(m.materials[0].id, MaterialId(0));
    assert_eq!(m.sections[0].id, SectionId(0));
    assert_eq!(m.elements[0].id, ElemId(0));
    // 参照が正規化後の index に張り替わっている。
    assert_eq!(m.elements[0].nodes.as_slice(), &[NodeId(0), NodeId(1)]);
    assert_eq!(m.elements[0].section, Some(SectionId(0)));
    assert_eq!(m.sections[0].material, Some(MaterialId(0)));
    assert!(matches!(
        m.sections[0].shape,
        Some(SectionShape::SteelH { .. })
    ));
}

/// ST-Bridge 標準の属性名（大文字 X/Y/Z 座標）の節点も読める。
#[test]
fn test_import_accepts_uppercase_coordinate_attrs() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes><StbNode id="0" X="1000" Y="2000" Z="3000"/></StbNodes>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert_eq!(m.nodes.len(), 1);
    assert_eq!(m.nodes[0].coord, [1000.0, 2000.0, 3000.0]);
}

/// ブレース（斜材）が `StbBrace` として往復する（Raw / Standard 両モード）。
#[test]
fn test_roundtrip_brace() {
    let mut m = frame_nodes();
    let pipe = SectionShape::SteelPipe {
        outer_dia: 100.0,
        thick: 5.0,
    };
    push_section(&mut m, pipe.to_section(SectionId(0), "BR".into()));
    // 節点0→3 の斜材（引張専用）。
    m.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Brace { tension_only: true },
        nodes: smallvec![NodeId(0), NodeId(3)],
        section: Some(SectionId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 1.0, 0.0],
        },
        end_cond: [EndCondition::Pinned, EndCondition::Pinned],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });

    let raw_xml = export_stbridge(&m).unwrap();
    assert!(
        raw_xml.contains("<StbBrace "),
        "ブレースは StbBrace で書き出される"
    );
    for xml in [raw_xml, export_stbridge(&m).unwrap()] {
        let back = import_stbridge(&xml).expect("import");
        assert!(back.validate().is_ok(), "{:?}", back.validate());
        assert_eq!(back.elements.len(), 1);
        assert_eq!(
            back.elements[0].kind,
            ElementKind::Brace { tension_only: true },
            "ブレース種別（tension_only 含む）が往復する"
        );
        assert_eq!(back.elements[0].nodes.as_slice(), &[NodeId(0), NodeId(3)]);
        assert_eq!(back.elements[0].section, Some(SectionId(0)));
        assert_eq!(back.sections[0].material, Some(MaterialId(0)));
    }
}

/// 標準書き出しは断面側にグレード名で材料を付す（鋼は strength_main、RC は strength_concrete）。
#[test]
fn test_standard_writes_section_material() {
    // 鋼柱: strength_main に材料名（グレード）。
    let mut m = frame_nodes(); // 材料 0 = "SN400B"
    let h = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    };
    push_section(&mut m, h.to_section(SectionId(0), "C".into()));
    m.elements.push(member(0, true, 0));
    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("strength_main=\"SN400B\""),
        "鋼断面に材料名（strength_main）を付す: {xml}"
    );

    // RC 柱: strength_concrete にコンクリートのグレード名（id は 1 始まり）。
    let mut m2 = frame_nodes();
    let rc = SectionShape::RcRect {
        b: 500.0,
        d: 500.0,
        rebar: rebar(),
    };
    push_section(&mut m2, rc.to_section(SectionId(0), "C".into()));
    m2.elements.push(member(0, true, 0));
    let xml2 = export_stbridge(&m2).unwrap();
    assert!(
        xml2.contains("<StbSecColumn_RC id=\"1\" name=\"C\" strength_concrete=\"SN400B\""),
        "RC 断面にコンクリートのグレード名を付す: {xml2}"
    );
}

/// 実 STB 相当: 部材が id_material を持たず断面が鋼種（strength_main）を持つファイルで、
/// 断面の材料を部材へ伝播する（材料名で突き合わせ）。
#[test]
fn test_import_propagates_steel_grade_material_to_member() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbMaterials><StbMaterial id="0" name="SN400B" young="205000" poisson="0.3" density="0"/></StbMaterials>
  <StbSections>
    <StbSecColumn_S id="0" name="C"><StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="H1" strength_main="SN400B"/></StbSecSteelFigureColumn_S></StbSecColumn_S>
    <StbSecSteel><StbSecRoll-H name="H1" type="H" A="300" B="150" t1="6.5" t2="9" r="0"/></StbSecSteel>
  </StbSections>
  <StbMembers><StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/></StbMembers>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(
        m.sections[0].material,
        Some(MaterialId(0)),
        "断面の鋼種から断面の材料が決まる"
    );
}

/// 実 STB 相当: RC 断面の id_material を（id_material 無しの）部材へ伝播する。
#[test]
fn test_import_propagates_rc_material_to_member() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbMaterials><StbMaterial id="5" name="Fc24" young="21000" poisson="0.2" density="0"/></StbMaterials>
  <StbSections>
    <StbSecColumn_RC id="0" name="C" id_material="5"><StbSecFigureColumn_RC><StbSecColumn_RC_Rect width_X="500" width_Y="500"/></StbSecFigureColumn_RC></StbSecColumn_RC>
  </StbSections>
  <StbMembers><StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/></StbMembers>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    // 材料 id=5 は正規化で index 0 になる。
    assert_eq!(
        m.sections[0].material,
        Some(MaterialId(0)),
        "断面の id_material が断面の材料になる"
    );
}

/// 対応範囲内のファイルは取り込み報告がクリーン（欠落なし）。
#[test]
fn test_import_report_clean_for_supported_model() {
    let mut m = frame_nodes();
    let h = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    };
    push_section(&mut m, h.to_section(SectionId(0), "C".into()));
    m.elements.push(member(0, true, 0));
    let xml = export_stbridge(&m).unwrap();
    let (_m, report) = import_stbridge_with_report(&xml).expect("import");
    assert!(
        report.is_clean(),
        "対応範囲のモデルは警告なし: {:?}",
        report.warnings
    );
}

/// 未対応要素（基礎・杭）は警告として報告され、無言で欠落しない。
#[test]
fn test_import_report_lists_unsupported_elements() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbMembers>
    <StbColumn id="0" id_node_bottom="0" id_node_top="1"/>
    <StbFooting id="1" name="F1"/>
    <StbFooting id="2" name="F2"/>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(m.elements.len(), 1, "対応する柱のみ取り込む");
    assert!(!report.is_clean());
    let joined = report.warnings.join(" | ");
    assert!(
        joined.contains("StbFooting×2"),
        "基礎2件の欠落を報告: {joined}"
    );
}

/// 明示リストにない未知の部材・断面・荷重要素も「取り込み対象外」として通知される
/// （fail-loud）。一方、形鋼ライブラリのコンテナ StbSecSteel は誤検出しない。
#[test]
fn test_import_report_unknown_elements_are_reported() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="C">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="H1"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecSteel>
      <StbSecRoll-H name="H1" A="300" B="150" t1="6.5" t2="9"/>
    </StbSecSteel>
    <StbSecFutureThing id="1" name="X"/>
  </StbSections>
  <StbMembers>
    <StbColumns>
      <StbColumn id="0" name="C1" id_node_bottom="0" id_node_top="1" id_section="0" kind_structure="S"/>
      <StbNovelMember id="1"/>
    </StbColumns>
  </StbMembers>
  <StbLoads>
    <StbLoadCase id="0" name="L1">
      <StbNodalLoad id_node="1" fz="-5"/>
      <StbLoadMember id="0"/>
    </StbLoadCase>
  </StbLoads>
</StbModel></ST_BRIDGE>"#;
    let (_m, report) = import_stbridge_with_report(xml).expect("import");
    let joined = report.warnings.join(" | ");
    // 未知の部材・断面・荷重が名指しで通知される。
    assert!(joined.contains("StbNovelMember×1"), "未知の部材: {joined}");
    assert!(
        joined.contains("StbSecFutureThing×1"),
        "未知の断面: {joined}"
    );
    assert!(joined.contains("StbLoadMember×1"), "未対応の荷重: {joined}");
    // 形鋼ライブラリのコンテナは誤検出しない。
    assert!(
        !joined.contains("StbSecSteel×"),
        "コンテナは誤検出しない: {joined}"
    );
}

/// StbSlab（境界節点ループ StbNodeIdOrder）と StbSecSlab_RC（厚さ）を取り込む。
#[test]
fn test_import_slab_with_node_order_and_thickness() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="4000" Y="3000" Z="0"/>
    <StbNode id="3" X="0" Y="3000" Z="0"/>
  </StbNodes>
  <StbSections>
    <StbSecSlab_RC id="7" name="S1">
      <StbSecFigureSlab_RC>
        <StbSecSlab_RC_Straight thickness="180"/>
      </StbSecFigureSlab_RC>
    </StbSecSlab_RC>
  </StbSections>
  <StbMembers>
    <StbSlab id="0" name="S1" id_section="7" kind_structure="RC">
      <StbNodeIdOrder>0 1 2 3</StbNodeIdOrder>
    </StbSlab>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(m.slabs.len(), 1, "スラブを1件取り込む");
    assert!(m.floor_regions.is_empty(), "大梁がないので床領域は0件");
    let s = &m.slabs[0];
    assert_eq!(
        s.boundary_nodes().unwrap(),
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        "境界節点ループが順序どおり"
    );
    assert_eq!(
        m.slab_plate_thickness(s),
        Some(180.0),
        "断面参照から厚さを解決"
    );
    assert!(
        report
            .warnings
            .iter()
            .any(|w| w.contains("床板") && w.contains("割り当て")),
        "大梁なしの浮き床板は警告: {:?}",
        report.warnings
    );
}

/// StbNodeIdOrder が CDATA 形式でも境界を取り込めること。
#[test]
fn test_import_slab_node_order_cdata() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="4000" Y="3000" Z="0"/>
    <StbNode id="3" X="0" Y="3000" Z="0"/>
  </StbNodes>
  <StbMembers>
    <StbSlab id="0" name="S1" kind_structure="RC">
      <StbNodeIdOrder><![CDATA[0 1 2 3]]></StbNodeIdOrder>
    </StbSlab>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, _report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(m.slabs.len(), 1, "CDATA の節点ループを取り込む");
    assert_eq!(
        m.slabs[0].boundary_nodes().unwrap(),
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)]
    );
}

/// 自己終了 <StbNodeIdOrder/> の後に無関係な子要素のテキストがあっても、
/// 取り込み窓が閉じられて境界へ誤混入しないこと（レビュー指摘の回帰テスト）。
#[test]
fn test_import_slab_self_closing_node_order_does_not_capture_stray_text() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="4000" Y="3000" Z="0"/>
    <StbNode id="3" X="0" Y="3000" Z="0"/>
  </StbNodes>
  <StbMembers>
    <StbSlab id="0" name="S1" kind_structure="RC">
      <StbNodeIdOrder/>
      <Foo>999</Foo>
      <StbNodeIdOrder>0 1 2 3</StbNodeIdOrder>
    </StbSlab>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, _report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(m.slabs.len(), 1);
    // 999 が混入せず、実 StbNodeIdOrder の 0 1 2 3 のみになる。
    assert_eq!(
        m.slabs[0].boundary_nodes().unwrap(),
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        "自己終了タグ後の無関係テキストを取り込まない"
    );
}

/// StbWall（境界節点ループ）と StbSecWall_RC（厚さ）を壁要素として取り込む。
#[test]
fn test_import_wall_with_node_order_and_thickness() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="4000" Y="0" Z="3000"/>
    <StbNode id="3" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecWall_RC id="9" name="W1">
      <StbSecFigureWall_RC>
        <StbSecWall_RC_Straight thickness="200"/>
      </StbSecFigureWall_RC>
    </StbSecWall_RC>
  </StbSections>
  <StbMembers>
    <StbWall id="0" name="W1" id_section="9" kind_structure="RC">
      <StbNodeIdOrder>0 1 2 3</StbNodeIdOrder>
    </StbWall>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    // 壁の解析要素（`ElementKind::Wall`）は取り込み時には作らない（D5）。取り込みは
    // `WallPlate` を組み立て、`rebuild_wall_regions` が検出済みの壁領域へ帰属させる。
    assert!(
        m.elements
            .iter()
            .all(|e| e.kind != squid_n_core::model::ElementKind::Wall),
        "壁要素は準備計算からの生成物であり取り込み時には作らない"
    );
    let plates: Vec<_> = m.wall_plates.iter().collect();
    assert_eq!(plates.len(), 1, "壁版を1件取り込む");
    let p = plates[0];
    assert_eq!(
        p.boundary_nodes(),
        Some(&[NodeId(0), NodeId(1), NodeId(2), NodeId(3)][..]),
        "境界節点ループが順序どおり"
    );
    let sec = p.section.and_then(|s| m.sections.get(s.index()));
    assert_eq!(
        sec.and_then(|s| s.thickness),
        Some(200.0),
        "壁断面の厚さを解決"
    );
    // 本フィクスチャは柱・梁を持たないため壁領域（region_gen::wall の検出対象）は
    // 検出されず、壁版はどの壁領域にも帰属しない（警告になる。壁版の帰属確認は
    // full_model.rs の実フィクスチャで行う）。
    assert_eq!(
        report.warnings,
        vec!["壁領域の作り直しで壁版 1 枚が領域に割り当てられなかった".to_string()]
    );
}

/// 頂部梁の上に立つパラペット（StbWall）は、どの壁領域にも収まらないが
/// 下辺が頂部梁に全長覆われているため、取り込み後に取り付く壁版へ自動変換される
/// （床側 D20 に相当。`wall_region_rebuild::rebuild_wall_regions` 参照）。
#[test]
fn test_import_converts_unenclosed_parapet_to_attached_wall_plate() {
    use squid_n_core::model::{RegionAnchor, WallPlateShape};

    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
    <StbNode id="2" X="4000" Y="0" Z="0"/>
    <StbNode id="3" X="4000" Y="0" Z="3000"/>
    <StbNode id="4" X="0" Y="0" Z="4500"/>
    <StbNode id="5" X="4000" Y="0" Z="4500"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="C1"><StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="H1"/></StbSecSteelFigureColumn_S></StbSecColumn_S>
    <StbSecBeam_S id="1" name="G1"><StbSecSteelFigureBeam_S><StbSecSteelBeam_S_Straight shape="H1"/></StbSecSteelFigureBeam_S></StbSecBeam_S>
    <StbSecSteel><StbSecRoll-H name="H1" type="H" A="300" B="150" t1="6.5" t2="9" r="0"/></StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="0"/>
    <StbColumn id="1" id_node_bottom="2" id_node_top="3" id_section="0"/>
    <StbGirder id="2" id_node_start="0" id_node_end="2" id_section="1"/>
    <StbGirder id="3" id_node_start="1" id_node_end="3" id_section="1"/>
    <StbWall id="0" name="Parapet">
      <StbNodeIdOrder>1 3 5 4</StbNodeIdOrder>
    </StbWall>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(
        m.wall_regions.len(),
        1,
        "柱・梁の1区画が壁領域として検出される"
    );
    assert!(
        m.wall_regions[0].wall_plate_ids.is_empty(),
        "パラペットは壁領域に帰属しない"
    );
    assert_eq!(m.wall_plates.len(), 1);
    let plate = &m.wall_plates[0];
    assert!(plate.is_attached(), "パラペットは取り付く壁版へ変換される");
    match &plate.shape {
        WallPlateShape::Attached {
            anchor: RegionAnchor::Line { nodes, .. },
            extent,
        } => {
            let a = m.nodes[nodes[0].index()].coord;
            let b = m.nodes[nodes[1].index()].coord;
            assert!((a[2] - 3000.0).abs() < 1e-6, "取付き線は頂部梁の高さ");
            assert!((b[2] - 3000.0).abs() < 1e-6);
            assert!((extent[0] - 1500.0).abs() < 1e-6, "{extent:?}");
            assert!((extent[1] - 1500.0).abs() < 1e-6, "{extent:?}");
        }
        other => panic!("Line の Attached ではない: {other:?}"),
    }
    // 帰属なし・照合できなかった旧領域の警告は出ない（自動変換は成功扱いで警告しない。
    // 床側 D20 と同じ挙動）。断面未割当（本フィクスチャは id_section を持たない）の
    // 警告だけが残る。
    assert_eq!(
        report.warnings,
        vec!["断面未割当の壁版を 1 枚取り込みました。解析要素としては生成しません".to_string()]
    );
}

/// 自己終了 <StbSlab/> の後の StbWall の節点ループが、陳腐化したスラブ状態に
/// 取り込まれず正しく壁へ入ること（レビュー指摘の回帰テスト）。
#[test]
fn test_self_closing_slab_does_not_steal_wall_nodes() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="4000" Y="0" Z="3000"/>
    <StbNode id="3" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbMembers>
    <StbSlab id="0" name="S0" id_section="1"/>
    <StbWall id="1" name="W1" kind_structure="RC">
      <StbNodeIdOrder>0 1 2 3</StbNodeIdOrder>
    </StbWall>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, _report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(
        m.wall_plates.len(),
        1,
        "壁版が取り込まれる（節点を横取りされない）"
    );
    assert_eq!(
        m.wall_plates[0].boundary_nodes(),
        Some(&[NodeId(0), NodeId(1), NodeId(2), NodeId(3)][..])
    );
}

/// 壁（境界＋厚さ）を含むモデルが export→import で往復すること。
///
/// 壁の解析要素（`ElementKind::Wall`）はモデルに残らない生成物（D5）のため、
/// ここでは入力の正である `WallPlate`/`WallRegion` を直接構築する（生の
/// `ElementData` を `model.elements` へ直接置くのは、この移行後は
/// 「準備計算・出力の直前に壁展開関数が組み立てる一時的な生成物」の形であり、
/// 保存対象のモデルとしては不正な状態になったため使わない）。
#[test]
fn test_wall_roundtrip_export_import() {
    use squid_n_core::ids::{WallPlateId, WallRegionId};
    use squid_n_core::model::{WallPlate, WallPlateShape, WallRegion};
    let mut model = Model::default();
    for (i, (x, z)) in [(0.0, 0.0), (4000.0, 0.0), (4000.0, 3000.0), (0.0, 3000.0)]
        .into_iter()
        .enumerate()
    {
        model.nodes.push(squid_n_core::model::Node {
            id: NodeId(i as u32),
            coord: [x, 0.0, z],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    // 厚さ 250 の壁断面と、それを参照する壁要素。
    model.sections.push(squid_n_core::model::Section {
        id: SectionId(0),
        name: "W".into(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 0.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: Some(250.0),
        shape: None,
        material: None,
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.wall_plates.push(WallPlate {
        id: WallPlateId(0),
        shape: WallPlateShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        },
        section: Some(SectionId(0)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings: Vec::new(),
        slit: Default::default(),
    });
    model.wall_regions.push(WallRegion {
        id: WallRegionId(0),
        name: String::new(),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        wall_plate_ids: vec![WallPlateId(0)],
        posts: Vec::new(),
    });
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    let xml = export_stbridge(&model).expect("export");
    let (m2, _report) = import_stbridge_with_report(&xml).expect("import");
    assert!(m2.validate().is_ok(), "{:?}", m2.validate());
    // 出力は壁版（入力の正。D5）から `<StbWall>` を書く。取り込み側は
    // `WallPlate` を組み立てるため、往復後の姿は `wall_plates` で確認する。
    assert!(
        m2.elements.iter().all(|e| e.kind != ElementKind::Wall),
        "壁要素は生成物であり保存されたモデルには残らない"
    );
    let plates: Vec<_> = m2.wall_plates.iter().collect();
    assert_eq!(plates.len(), 1, "壁版1件");
    assert_eq!(
        plates[0].boundary_nodes(),
        Some(&[NodeId(0), NodeId(1), NodeId(2), NodeId(3)][..]),
        "境界が往復"
    );
    let t = plates[0].section.and_then(|s| m2.sections.get(s.index()));
    assert_eq!(t.and_then(|s| s.thickness), Some(250.0), "厚さが往復");

    // 往復を重ねても断面数が増殖しないこと。従来は壁専用の厚さ断面が
    // StbSecRaw と StbSecWall_RC の両方で出力され、1 サイクルごとに
    // 未参照断面が 1 枚ずつ増えていた。
    assert_eq!(
        m2.sections.len(),
        model.sections.len(),
        "1 サイクル目で断面数が保たれる"
    );
    let xml2 = export_stbridge(&m2).expect("export 2nd");
    let (m3, _) = import_stbridge_with_report(&xml2).expect("import 2nd");
    assert_eq!(
        m3.sections.len(),
        m2.sections.len(),
        "2 サイクル目でも断面数が保たれる（増殖しない）"
    );
    assert!(m3.validate().is_ok(), "{:?}", m3.validate());
}

/// 4 節点でない囲まれた壁版も ST-Bridge では往復する（解析要素にはしない）。
#[test]
fn test_non_quad_wall_plate_roundtrip_export_import() {
    use squid_n_core::ids::WallPlateId;
    use squid_n_core::model::{WallPlate, WallPlateShape};
    let mut model = Model::default();
    for (i, (x, z)) in [
        (0.0, 0.0),
        (4000.0, 0.0),
        (4000.0, 3000.0),
        (2000.0, 3000.0),
        (0.0, 3000.0),
    ]
    .into_iter()
    .enumerate()
    {
        model.nodes.push(squid_n_core::model::Node {
            id: NodeId(i as u32),
            coord: [x, 0.0, z],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.sections.push(squid_n_core::model::Section {
        id: SectionId(0),
        name: "W".into(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 0.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: Some(180.0),
        shape: None,
        material: None,
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    model.wall_plates.push(WallPlate {
        id: WallPlateId(0),
        shape: WallPlateShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3), NodeId(4)],
        },
        section: Some(SectionId(0)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings: Vec::new(),
        slit: Default::default(),
    });
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    let xml = export_stbridge(&model).expect("export");
    assert!(
        xml.contains("<StbWall"),
        "5 節点の壁版も StbWall として出す"
    );
    let (m2, report) = import_stbridge_with_report(&xml).expect("import");
    assert!(m2.validate().is_ok(), "{:?}", m2.validate());
    assert_eq!(m2.wall_plates.len(), 1);
    assert_eq!(
        m2.wall_plates[0].boundary_nodes().map(|n| n.len()),
        Some(5),
        "5 節点が往復する"
    );
    // 解析要素にならない壁版は正常な状態なので、取り込みでは知らせない。
    assert!(
        !report
            .warnings
            .iter()
            .any(|w| w.contains("4 節点でない壁版")),
        "解析要素にならないことを知らせないこと: {:?}",
        report.warnings
    );
}

/// スラブ（境界＋厚さ）を含むモデルが export→import で往復すること。
#[test]
fn test_slab_roundtrip_export_import() {
    use squid_n_core::ids::SlabId;
    use squid_n_core::model::{DistributionMethod, Slab, SlabShape};
    let mut model = Model::default();
    for (i, (x, y)) in [(0.0, 0.0), (4000.0, 0.0), (4000.0, 3000.0), (0.0, 3000.0)]
        .into_iter()
        .enumerate()
    {
        model.nodes.push(squid_n_core::model::Node {
            id: NodeId(i as u32),
            coord: [x, y, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    // 板厚 200 mm のスラブ断面。符号・板厚・コンクリート材料が往復する。
    model.materials.push(squid_n_core::model::Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: squid_n_core::ids::MaterialId(0),
        name: "Fc24".into(),
        category: squid_n_core::model::MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    let slab_sec = squid_n_core::ids::SectionId(0);
    let mut sec = squid_n_core::section_shape::SectionShape::RcSlab { thickness: 200.0 }
        .to_section(slab_sec, "S20".into());
    sec.material = Some(squid_n_core::ids::MaterialId(0));
    model.sections.push(sec);
    // 大梁 4 本で境界を閉じる（床領域として帰属させ、浮き床板警告を避けるため）。
    let mk_beam = |id: u32, i: u32, j: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec![NodeId(i), NodeId(j)],
        section: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    model.elements.extend([
        mk_beam(0, 0, 1),
        mk_beam(1, 1, 2),
        mk_beam(2, 2, 3),
        mk_beam(3, 3, 0),
    ]);
    model.slabs.push(Slab {
        id: SlabId(0),
        shape: SlabShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        },
        plate: SlabPlate {
            section: Some(slab_sec),
            loads: Vec::new(),
            usage: None,
            method: DistributionMethod::TriTrapezoid,
            one_way: None,
        },
    });
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    let xml = export_stbridge(&model).expect("export");
    let (m2, report) = import_stbridge_with_report(&xml).expect("import");
    assert!(m2.validate().is_ok(), "{:?}", m2.validate());
    assert_eq!(m2.slabs.len(), 1, "スラブ1件");
    assert_eq!(
        m2.slabs[0].boundary_nodes().unwrap(),
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        "境界が往復"
    );
    assert_eq!(
        m2.slab_plate_thickness(&m2.slabs[0]),
        Some(200.0),
        "厚さが往復"
    );
    let sec2 = m2.slab_section(&m2.slabs[0]).expect("断面が往復");
    assert_eq!(sec2.name, "S20", "符号が往復");
    assert_eq!(
        sec2.material
            .and_then(|mid| m2.materials.get(mid.index()))
            .map(|mm| mm.name.as_str()),
        Some("Fc24"),
        "コンクリート材料が往復"
    );
    assert!(report.is_clean(), "警告なし {:?}", report.warnings);
}

/// 形鋼ライブラリに定義のない断面参照は、物性ゼロで取り込みつつ警告する。
#[test]
fn test_import_report_warns_unresolved_steel_ref() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbSections>
    <StbSecColumn_S id="0" name="C"><StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="MISSING"/></StbSecSteelFigureColumn_S></StbSecColumn_S>
  </StbSections>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert_eq!(m.sections.len(), 1);
    assert!(m.sections[0].shape.is_none(), "未解決参照は物性ゼロ断面");
    assert!(
        report
            .warnings
            .iter()
            .any(|w| w.contains("形鋼参照を解決できず")),
        "未解決の形鋼参照を報告: {:?}",
        report.warnings
    );
}

// ===== レビュー指摘の回帰テスト =====

/// [高] StbPost（間柱, bottom/top）を含むファイルが取り込みエラーで中断せず、
/// 間柱は二次部材（解析対象外・CMQ 用）として取り込まれる。
#[test]
fn test_import_stbpost_bottom_top() {
    use squid_n_core::model::SecondaryMemberKind;
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbMembers><StbPost id="0" id_node_bottom="0" id_node_top="1"/></StbMembers>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("StbPost で中断しない");
    assert!(m.elements.is_empty(), "間柱は解析要素にしない");
    assert_eq!(m.unassigned_posts.len(), 1);
    assert_eq!(m.unassigned_posts[0].kind, SecondaryMemberKind::Post);
    assert_eq!(m.unassigned_posts[0].nodes, [NodeId(0), NodeId(1)]);
}

/// [高] SRC 内蔵鉄骨の参照が未解決なら警告する（無言のゼロ鉄骨を防ぐ）。
#[test]
fn test_import_report_warns_unresolved_src_steel() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbSections>
    <StbSecColumn_SRC id="0" name="SC" strength_steel="SN490B">
      <StbSecFigureColumn_SRC><StbSecColumn_SRC_Rect width_X="800" width_Y="800"/></StbSecFigureColumn_SRC>
      <StbSecSteelFigureColumn_SRC><StbSecSteelColumn_SRC_Same shape="MISSING_H"/></StbSecSteelFigureColumn_SRC>
    </StbSecColumn_SRC>
  </StbSections>
</StbModel></ST_BRIDGE>"#;
    let (_m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(
        report
            .warnings
            .iter()
            .any(|w| w.contains("内蔵鉄骨参照を解決できず")),
        "SRC 内蔵鉄骨の未解決を報告: {:?}",
        report.warnings
    );
}

/// 標準 ST-Bridge では材料は断面のグレード名で表す。断面が持つ材料は
/// 書き出し→再取り込みで保存され、その断面を使う全部材に効く。
#[test]
fn test_section_grade_material_roundtrips() {
    let mut m = frame_nodes(); // 材料0="SN400B"
    let h = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    };
    let mut sec = h.to_section(SectionId(0), "S".into());
    sec.material = Some(MaterialId(0));
    m.sections.push(sec);
    m.elements.push(member(0, true, 0));
    m.elements.push(member(1, false, 0));

    let back = import_stbridge(&export_stbridge(&m).unwrap()).expect("import");
    // 柱・梁が同じ断面を使うため、材料も断面ごとに 1 つで足りる。
    assert_eq!(back.sections[0].material, Some(MaterialId(0)), "断面の材料");
    assert_eq!(back.materials[0].name, "SN400B");
}

/// [中] 柱と梁で材料が違うならそれは別の断面になる。それぞれの断面へ
/// 自分の材料のグレード名を書き出す。
#[test]
fn test_column_and_beam_sections_write_own_material() {
    let mut m = frame_nodes();
    m.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(1),
        name: "SN490B".into(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(325.0),
    });
    let h = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    };
    let mut col_sec = h.clone().to_section(SectionId(0), "C".into());
    col_sec.material = Some(MaterialId(0));
    let mut beam_sec = h.to_section(SectionId(1), "G".into());
    beam_sec.material = Some(MaterialId(1));
    m.sections.push(col_sec);
    m.sections.push(beam_sec);
    m.elements.push(member(0, true, 0));
    m.elements.push(member(1, false, 1));

    let xml = export_stbridge(&m).unwrap();
    assert!(
        xml.contains("<StbSecColumn_S ") && xml.contains("strength_main=\"SN400B\""),
        "柱断面に SN400B: {xml}"
    );
    assert!(
        xml.contains("<StbSecBeam_S ") && xml.contains("strength_main=\"SN490B\""),
        "梁断面に SN490B: {xml}"
    );
}

/// [中] 存在しない断面を参照する部材は、リンクを外しつつ警告する。
#[test]
fn test_import_report_warns_dangling_section_ref() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
  </StbNodes>
  <StbMembers><StbColumn id="0" id_node_bottom="0" id_node_top="1" id_section="99"/></StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert_eq!(m.elements[0].section, None);
    assert!(
        report.warnings.iter().any(|w| w.contains("存在しない断面")),
        "ダングリング断面参照を報告: {:?}",
        report.warnings
    );
}

/// [低] 鋼ブレース断面 StbSecBrace_S を取り込み、ブレースが断面を持つ。
#[test]
fn test_import_stbsecbrace_s() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="6000" Y="0" Z="3000"/>
  </StbNodes>
  <StbSections>
    <StbSecBrace_S id="0" name="BR"><StbSecSteelFigureBrace_S><StbSecSteelBrace_S_Same shape="P1"/></StbSecSteelFigureBrace_S></StbSecBrace_S>
    <StbSecSteel><StbSecPipe name="P1" D="100" t="5"/></StbSecSteel>
  </StbSections>
  <StbMembers><StbBrace id="0" id_node_start="0" id_node_end="1" id_section="0" tension_only="true"/></StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(
        m.elements[0].section,
        Some(SectionId(0)),
        "ブレースが断面を持つ"
    );
    assert!(
        matches!(m.sections[0].shape, Some(SectionShape::SteelPipe { .. })),
        "ブレース断面が鋼管として復元"
    );
    assert!(
        report.is_clean(),
        "StbSecBrace_S は未対応ではない: {:?}",
        report.warnings
    );
}

/// [低] esc は XML 1.0 で表現できない制御文字（例: form feed）を除去する。
#[test]
fn test_export_strips_illegal_control_chars() {
    let mut m = frame_nodes();
    let mut sec = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    }
    .to_section(SectionId(0), "S".into());
    sec.name = "A\u{0C}B".into(); // form feed を含む名前
    m.sections.push(sec);
    m.elements.push(member(0, true, 0));
    let xml = export_stbridge(&m).unwrap();
    assert!(!xml.contains('\u{0C}'), "不正な制御文字が出力に残らない");
    assert!(import_stbridge(&xml).is_ok(), "出力は XML として読み戻せる");
}

/// [低] 未対応要素リストに StbOpen（開口）が含まれ、欠落が報告される。
#[test]
fn test_import_report_lists_stbopen() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbMembers><StbOpen id="0" id_wall="1"/></StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (_m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(
        report.warnings.iter().any(|w| w.contains("StbOpen")),
        "StbOpen の欠落を報告: {:?}",
        report.warnings
    );
}

/// ST-Bridge は境界条件（支点）を持たないため、取り込み時に最下レベル
/// （Z 最小、許容差 1mm）の節点をピン支点（並進固定・回転自由）に自動設定し、notes で通知する。
/// notes は欠落警告ではないため `is_clean` には影響しない。
#[test]
fn test_import_auto_fixes_base_level_supports() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="100"/>
    <StbNode id="1" X="6000" Y="0" Z="100.5"/>
    <StbNode id="2" X="0" Y="0" Z="3500"/>
    <StbNode id="3" X="6000" Y="0" Z="3500"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="C">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="H1"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecSteel>
      <StbSecRoll-H name="H1" A="300" B="150" t1="6.5" t2="9"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumns>
      <StbColumn id="0" name="C1" id_node_bottom="0" id_node_top="2" id_section="0" kind_structure="S"/>
      <StbColumn id="1" name="C2" id_node_bottom="1" id_node_top="3" id_section="0" kind_structure="S"/>
    </StbColumns>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");

    use squid_n_core::dof::Dof6Mask;
    // 最下レベル: Z=100 と Z=100.5（許容差 1mm 以内）の 2 節点がピン支点になる。
    assert_eq!(m.nodes[0].restraint, Dof6Mask::PINNED);
    assert_eq!(m.nodes[1].restraint, Dof6Mask::PINNED);
    // 上部節点は自由のまま。
    assert_eq!(m.nodes[2].restraint, Dof6Mask::FREE);
    assert_eq!(m.nodes[3].restraint, Dof6Mask::FREE);
    // notes で通知され、欠落警告（is_clean）には影響しない。
    assert!(
        report
            .notes
            .iter()
            .any(|n| n.contains("ピン支点に設定") && n.contains("2 箇所")),
        "notes: {:?}",
        report.notes
    );
    assert!(report.is_clean(), "warnings: {:?}", report.warnings);
}

/// 支点の自動設定は、最下レベルで**柱脚が取り付く**節点だけをピン支点にする。
/// 柱が取り付かず梁（地中梁）だけが取り付く最下レベル節点は支点にしない。
#[test]
fn test_import_auto_support_excludes_beam_only_base_nodes() {
    // 最下レベル Z=0 に節点 0,1,4。柱 C1(0→2)・C2(1→4... ではなく 1→3)。
    // 地中梁 G1(0→4)・G2(4→1) は水平材で節点 4 に柱はない。
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="6000" Y="0" Z="0"/>
    <StbNode id="2" X="0" Y="0" Z="3000"/>
    <StbNode id="3" X="6000" Y="0" Z="3000"/>
    <StbNode id="4" X="3000" Y="0" Z="0"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="0" name="C">
      <StbSecSteelFigureColumn_S><StbSecSteelColumn_S_Same shape="H1"/></StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecBeam_S id="1" name="G">
      <StbSecSteelFigureBeam_S><StbSecSteelBeam_S_Straight shape="H1"/></StbSecSteelFigureBeam_S>
    </StbSecBeam_S>
    <StbSecSteel>
      <StbSecRoll-H name="H1" A="300" B="150" t1="6.5" t2="9"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbColumns>
      <StbColumn id="0" name="C1" id_node_bottom="0" id_node_top="2" id_section="0" kind_structure="S"/>
      <StbColumn id="1" name="C2" id_node_bottom="1" id_node_top="3" id_section="0" kind_structure="S"/>
    </StbColumns>
    <StbGirders>
      <StbGirder id="2" name="G1" id_node_start="0" id_node_end="4" id_section="1" kind_structure="S"/>
      <StbGirder id="3" name="G2" id_node_start="4" id_node_end="1" id_section="1" kind_structure="S"/>
    </StbGirders>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");

    use squid_n_core::dof::Dof6Mask;
    // 柱脚が取り付く 0,1 はピン支点。梁だけの最下レベル節点 4 は自由のまま。
    assert_eq!(m.nodes[0].restraint, Dof6Mask::PINNED, "柱脚 0 はピン");
    assert_eq!(m.nodes[1].restraint, Dof6Mask::PINNED, "柱脚 1 はピン");
    assert_eq!(
        m.nodes[4].restraint,
        Dof6Mask::FREE,
        "梁だけが取り付く最下レベル節点 4 は支点にしない"
    );
    // 柱頭は自由のまま。
    assert_eq!(m.nodes[2].restraint, Dof6Mask::FREE);
    assert_eq!(m.nodes[3].restraint, Dof6Mask::FREE);
    // notes は「柱が取り付く節点 2 箇所」を通知する。
    assert!(
        report
            .notes
            .iter()
            .any(|n| n.contains("柱が取り付く節点") && n.contains("2 箇所")),
        "notes: {:?}",
        report.notes
    );
    assert!(report.is_clean(), "warnings: {:?}", report.warnings);
}

/// 小梁（StbBeam）は二次部材（解析対象外・CMQ 用）として取り込まれ、
/// 大梁（StbGirder）は従来どおり解析要素になる。断面・材料（グレード伝播）も
/// 二次部材へ解決される。
#[test]
fn test_import_stbbeam_as_secondary_member() {
    use squid_n_core::model::SecondaryMemberKind;
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="6000" Y="0" Z="0"/>
    <StbNode id="2" X="2000" Y="0" Z="0"/>
    <StbNode id="3" X="2000" Y="4000" Z="0"/>
  </StbNodes>
  <StbSections>
    <StbSecBeam_S id="0" name="G">
      <StbSecSteelFigureBeam_S><StbSecSteelBeam_S_Straight shape="H1" strength_main="SN400B"/></StbSecSteelFigureBeam_S>
    </StbSecBeam_S>
    <StbSecSteel>
      <StbSecRoll-H name="H1" A="300" B="150" t1="6.5" t2="9"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbGirders>
      <StbGirder id="0" name="G1" id_node_start="0" id_node_end="1" id_section="0" kind_structure="S"/>
    </StbGirders>
    <StbBeams>
      <StbBeam id="1" name="B1" id_node_start="2" id_node_end="3" id_section="0" kind_structure="S"/>
    </StbBeams>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert_eq!(m.elements.len(), 1, "大梁のみ解析要素");
    assert_eq!(m.joists().count(), 1);
    let sm = m.joists().next().expect("小梁 1 本");
    assert_eq!(sm.kind, SecondaryMemberKind::Joist);
    assert_eq!(sm.nodes, [NodeId(2), NodeId(3)]);
    assert!(sm.section.is_some(), "断面参照が解決されるはず");
    assert!(
        m.sections[sm.section.unwrap().index()].material.is_some(),
        "断面にグレード材料が設定されるはず"
    );
    assert!(
        report.notes.iter().any(|n| n.contains("小梁 1 本")),
        "二次部材の取り込みを通知: {:?}",
        report.notes
    );
    assert!(m.validate().is_ok());
}

/// 二次部材（小梁・間柱）が ST-Bridge 書き出し（StbBeam/StbPost）→再取り込みで
/// 保存されること（往復）。
#[test]
fn test_secondary_members_roundtrip() {
    use squid_n_core::model::{SecondaryMember, SecondaryMemberKind};

    let mut m = frame_nodes();
    let h = SectionShape::SteelH {
        height: 300.0,
        width: 150.0,
        web_thick: 6.5,
        flange_thick: 9.0,
    };
    push_section(&mut m, h.to_section(SectionId(0), "G".into()));
    m.elements.push(member(0, false, 0));
    // 小梁と間柱を 1 本ずつ（節点は既存節点を使う）。
    m.unassigned_joists.push(SecondaryMember {
        kind: SecondaryMemberKind::Joist,
        nodes: [NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        name: "B1".into(),
    });
    m.unassigned_posts.push(SecondaryMember {
        kind: SecondaryMemberKind::Post,
        nodes: [NodeId(0), NodeId(2)],
        section: Some(SectionId(0)),
        name: "P1".into(),
    });
    m.validate().expect("元モデルは validate を通る");

    let xml = export_stbridge(&m).expect("export");
    assert!(xml.contains("<StbBeams>"), "小梁を書き出す: {xml}");
    assert!(xml.contains("<StbPosts>"), "間柱を書き出す: {xml}");

    let (back, _report) = import_stbridge_with_report(&xml).expect("re-import");
    assert_eq!(back.joists().count() + back.posts().count(), 2);
    let kinds: Vec<SecondaryMemberKind> =
        back.joists().chain(back.posts()).map(|s| s.kind).collect();
    assert!(kinds.contains(&SecondaryMemberKind::Joist));
    assert!(kinds.contains(&SecondaryMemberKind::Post));
    assert_eq!(back.elements.len(), 1, "大梁は解析要素のまま");
    assert!(back.validate().is_ok());
}

/// 厚さが分かるスラブ（StbSecSlab_RC）には、取り込み時に自重
/// 断面を共有する床が複数あっても、往復で断面が増えない。
///
/// 書き出しは**内部断面ごと**に `StbSecSlab_RC` を 1 つだけ出す。床ごとに出すと
/// 同名の断面が枚数分並び、再取り込みのたびに符号が `S15`・`S15#2` … と増殖する。
#[test]
fn test_slab_shared_section_does_not_multiply_on_roundtrip() {
    use squid_n_core::ids::{SectionId, SlabId};
    use squid_n_core::model::{DistributionMethod, Slab, SlabShape};

    let mut model = Model::default();
    // 2 スパン分の 6 節点で床 2 枚を作り、同じ断面を共有させる。
    for (i, (x, y)) in [
        (0.0, 0.0),
        (4000.0, 0.0),
        (8000.0, 0.0),
        (0.0, 3000.0),
        (4000.0, 3000.0),
        (8000.0, 3000.0),
    ]
    .into_iter()
    .enumerate()
    {
        model.nodes.push(squid_n_core::model::Node {
            id: NodeId(i as u32),
            coord: [x, y, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    let slab_sec = SectionId(0);
    model.sections.push(
        squid_n_core::section_shape::SectionShape::RcSlab { thickness: 150.0 }
            .to_section(slab_sec, "S15".into()),
    );
    for (i, b) in [[0, 1, 4, 3], [1, 2, 5, 4]].into_iter().enumerate() {
        model.slabs.push(Slab {
            id: SlabId(i as u32),
            shape: SlabShape::Enclosed {
                boundary: b.into_iter().map(NodeId).collect(),
            },
            plate: SlabPlate {
                section: Some(slab_sec),
                loads: Vec::new(),
                usage: None,
                method: DistributionMethod::TriTrapezoid,
                one_way: None,
            },
        });
    }
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    // 2 往復しても断面は 1 つのまま（符号に連番が付かない）。
    let mut m = model;
    for round in 1..=2 {
        let xml = export_stbridge(&m).expect("export");
        let (next, _) = import_stbridge_with_report(&xml).expect("import");
        assert_eq!(next.slabs.len(), 2, "{round} 往復目: 床 2 枚");
        let names: Vec<&str> = next.sections.iter().map(|sc| sc.name.as_str()).collect();
        assert_eq!(names, vec!["S15"], "{round} 往復目: 断面は 1 つ {names:?}");
        assert_eq!(
            next.slabs[0].section(),
            next.slabs[1].section(),
            "{round} 往復目: 2 枚が同じ断面を共有する"
        );
        m = next;
    }
}

/// スラブ断面（`StbSecSlab_RC`）を内部の断面として取り込み、床へ割り当てる。
///
/// 自重は面荷重へ焼き込まず、断面の板厚と材料から使うたびに算定する
/// （`Model::slab_self_weight_intensity`）。板厚や材料を変えたときに自重が
/// 追随しない食い違いを作らないためである。
#[test]
fn test_import_slab_section_and_self_weight() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="4000" Y="3000" Z="0"/>
    <StbNode id="3" X="0" Y="3000" Z="0"/>
  </StbNodes>
  <StbSections>
    <StbSecSlab_RC id="0" name="S150" strength_concrete="Fc24">
      <StbSecFigureSlab_RC><StbSecSlab_RC_Straight depth="150"/></StbSecFigureSlab_RC>
    </StbSecSlab_RC>
  </StbSections>
  <StbMembers>
    <StbSlabs>
      <StbSlab id="0" name="S1" id_section="0" kind_structure="RC">
        <StbNodeIdOrder>0 1 2 3</StbNodeIdOrder>
      </StbSlab>
    </StbSlabs>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert_eq!(m.slabs.len(), 1);
    let slab = &m.slabs[0];
    // 断面が作られ、符号は StbSecSlab_RC の name をそのまま採る。
    let sec = m.slab_section(slab).expect("スラブ断面が割り当たる");
    assert_eq!(sec.name, "S150");
    assert_eq!(m.slab_plate_thickness(slab), Some(150.0));
    assert!(slab.plate.loads.is_empty(), "自重は面荷重へ焼き込まない");
    // 150 mm × 24 kN/m³ = 3.6 kN/m² = 3.6e-3 N/mm²
    assert!(
        (m.slab_self_weight_intensity(slab)
            .expect("自重を算定できる")
            - 3.6e-3)
            .abs()
            < 1e-9,
        "自重の面荷重強度"
    );
    assert!(
        (m.slab_dead_intensity(slab) - 3.6e-3).abs() < 1e-9,
        "分配強度に自重が乗る"
    );
    assert!(
        report.notes.iter().any(|n| n.contains("スラブ断面 1 件")),
        "取り込みを通知: {:?}",
        report.notes
    );
}

/// 実 ST-Bridge の通り芯（`StbAxes` > `StbParallelAxes` > `StbParallelAxis`）を
/// グループの幾何・通り名・所属節点まで取り込み、書き戻して往復することを確認する。
///
/// 所属節点は座標から導けない（`X1a` は `distance=3000` だが所属節点の X は 3500 という
/// 芯ずれが実務で普通に起きる）ため、リストをそのまま保持することを併せて確かめる。
#[test]
fn test_import_export_parallel_axes() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="10" X="1000" Y="0" Z="0"/>
    <StbNode id="11" X="3500" Y="0" Z="0"/>
    <StbNode id="12" X="1000" Y="6000" Z="0"/>
  </StbNodes>
  <StbAxes>
    <StbParallelAxes group_name="Y" X="0.0" Y="0.0" angle="0.0">
      <StbParallelAxis id="1" name="Y1" distance="0.0">
        <StbNodeIdList>
          <StbNodeId id="10"/>
          <StbNodeId id="11"/>
        </StbNodeIdList>
      </StbParallelAxis>
      <StbParallelAxis id="2" name="Y2" distance="6000.0">
        <StbNodeIdList><StbNodeId id="12"/></StbNodeIdList>
      </StbParallelAxis>
    </StbParallelAxes>
    <StbParallelAxes group_name="X" X="0.0" Y="0.0" angle="270.0">
      <StbParallelAxis id="3" name="X1" distance="1000.0">
        <StbNodeIdList>
          <StbNodeId id="10"/>
          <StbNodeId id="12"/>
        </StbNodeIdList>
      </StbParallelAxis>
      <StbParallelAxis id="4" name="X1a" distance="3000.0">
        <StbNodeIdList><StbNodeId id="11"/></StbNodeIdList>
      </StbParallelAxis>
    </StbParallelAxes>
  </StbAxes>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert!(
        report.is_clean(),
        "通り芯は対応済みなので欠落警告は出ない: {:?}",
        report.warnings
    );

    assert_eq!(m.axes.len(), 2);
    let y = &m.axes[0];
    assert_eq!(y.name, "Y");
    assert_eq!(
        y.kind,
        AxisGroupKind::Parallel {
            origin: [0.0, 0.0],
            angle_deg: 0.0
        }
    );
    assert_eq!(
        y.axes.iter().map(|a| a.name.as_str()).collect::<Vec<_>>(),
        vec!["Y1", "Y2"]
    );
    // 節点 id は 0 始まり連番へ正規化される（file id 10/11/12 → 0/1/2）。
    assert_eq!(y.axes[0].nodes, vec![NodeId(0), NodeId(1)]);
    // 取り込んだ通りは利用者の入力と同格に扱い、自動生成で作り直さない。
    assert!(y.axes.iter().all(|a| a.source == AxisSource::Manual));

    let x = &m.axes[1];
    assert_eq!(x.axes[1].name, "X1a");
    assert_eq!(x.axes[1].distance, Some(3000.0));
    // 芯ずれ（通りは X=3000、節点は X=3500）でも所属はリストのとおり保つ。
    assert_eq!(x.axes[1].nodes, vec![NodeId(1)]);
    assert_eq!(m.nodes[1].coord[0], 3500.0);

    // 書き戻して再取り込みしても通り芯が一致する。
    let out = export_stbridge(&m).expect("export");
    let (again, _) = import_stbridge_with_report(&out).expect("re-import");
    assert_eq!(m.axes, again.axes, "通り芯が往復で一致する");
}

/// 円弧芯・放射芯・作図芯は幾何を表す型を持たないため `Other` として所属節点だけを
/// 取り込む（＝データを捨てずに読める）。書き出しでは平行芯のみを出す。
#[test]
fn test_import_non_parallel_axes_as_other() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="1" X="0" Y="0" Z="0"/>
    <StbNode id="2" X="5000" Y="0" Z="0"/>
  </StbNodes>
  <StbAxes>
    <StbArcAxes group_name="R" X="0.0" Y="0.0">
      <StbArcAxis id="1" name="R1" radius="5000.0">
        <StbNodeIdList><StbNodeId id="2"/></StbNodeIdList>
      </StbArcAxis>
    </StbArcAxes>
    <StbRadialAxes group_name="A" X="0.0" Y="0.0">
      <StbRadialAxis id="2" name="A1" angle="30.0"/>
    </StbRadialAxes>
  </StbAxes>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert!(report.is_clean(), "警告: {:?}", report.warnings);

    assert_eq!(m.axes.len(), 2);
    assert_eq!(m.axes[0].name, "R");
    assert_eq!(m.axes[0].kind, AxisGroupKind::Other);
    assert_eq!(m.axes[0].axes[0].name, "R1");
    assert_eq!(m.axes[0].axes[0].distance, None, "幾何は保持しない");
    assert_eq!(m.axes[0].axes[0].nodes, vec![NodeId(1)], "所属節点は保つ");
    assert_eq!(m.axes[1].axes[0].name, "A1");

    // 平行芯グループが 1 つもないモデルは StbAxes 自体を出力しない。
    let out = export_stbridge(&m).expect("export");
    assert!(!out.contains("<StbAxes>"), "{out}");
}

// ---------------------------------------------------------------------------
// 断面の同一性キー（符号＋階）と属性の扱いの報告
// ---------------------------------------------------------------------------

/// 階（`floor`）を持つ断面を取り込み、符号と階の両方が保持される。
/// ST-Bridge は同じ符号の断面を階ごとに別定義で持つため、階を落とすと別断面が
/// 区別できなくなる。
#[test]
fn test_import_keeps_section_floor() {
    let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<ST_BRIDGE version="2.0.2"><StbModel>
  <StbNodes>
    <StbNode id="1" X="0" Y="0" Z="0"/>
    <StbNode id="2" X="0" Y="0" Z="4000"/>
  </StbNodes>
  <StbSections>
    <StbSecColumn_S id="1" name="C1" floor="1">
      <StbSecSteelFigureColumn_S>
        <StbSecSteelColumn_S_Same shape="H-300x150x6.5x9"/>
      </StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecColumn_S id="2" name="C1" floor="2">
      <StbSecSteelFigureColumn_S>
        <StbSecSteelColumn_S_Same shape="BOX-300x300x12"/>
      </StbSecSteelFigureColumn_S>
    </StbSecColumn_S>
    <StbSecSteel>
      <StbSecRoll-H name="H-300x150x6.5x9" type="H" A="300" B="150" t1="6.5" t2="9"/>
      <StbSecRoll-BOX name="BOX-300x300x12" type="BOX" A="300" B="300" t="12"/>
    </StbSecSteel>
  </StbSections>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert_eq!(m.sections.len(), 2, "階が違えば別断面");
    assert_eq!(m.sections[0].name, "C1");
    assert_eq!(m.sections[0].floor.as_deref(), Some("1"));
    assert_eq!(m.sections[1].name, "C1");
    assert_eq!(m.sections[1].floor.as_deref(), Some("2"));
    // 階は書き出しでも保持する（往復で同一性キーが崩れない）。
    let out = export_stbridge(&m).expect("export");
    assert!(out.contains(r#"floor="1""#), "{out}");
    assert!(out.contains(r#"floor="2""#), "{out}");
    let back = import_stbridge(&out).expect("re-import");
    assert_eq!(back.sections.len(), 2);
    assert_eq!(back.sections[0].floor.as_deref(), Some("1"));
    assert_eq!(back.sections[1].floor.as_deref(), Some("2"));
}

/// 符号＋階が同じで内容も同じ断面定義は 1 件へ統合し、参照していた部材は
/// 統合先を指す。統合したことは notes で通知する。
#[test]
fn test_import_merges_identical_duplicate_sections() {
    let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<ST_BRIDGE version="2.0.2"><StbModel>
  <StbNodes>
    <StbNode id="1" X="0" Y="0" Z="0"/>
    <StbNode id="2" X="5000" Y="0" Z="0"/>
    <StbNode id="3" X="10000" Y="0" Z="0"/>
  </StbNodes>
  <StbSections>
    <StbSecBeam_S id="1" name="b3">
      <StbSecSteelFigureBeam_S>
        <StbSecSteelBeam_S_Straight shape="H-300x150x6.5x9"/>
      </StbSecSteelFigureBeam_S>
    </StbSecBeam_S>
    <StbSecBeam_S id="2" name="b3">
      <StbSecSteelFigureBeam_S>
        <StbSecSteelBeam_S_Straight shape="H-300x150x6.5x9"/>
      </StbSecSteelFigureBeam_S>
    </StbSecBeam_S>
    <StbSecSteel>
      <StbSecRoll-H name="H-300x150x6.5x9" type="H" A="300" B="150" t1="6.5" t2="9"/>
    </StbSecSteel>
  </StbSections>
  <StbMembers>
    <StbGirders>
      <StbGirder id="1" name="G1" id_node_start="1" id_node_end="2" id_section="1"/>
      <StbGirder id="2" name="G2" id_node_start="2" id_node_end="3" id_section="2"/>
    </StbGirders>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());
    assert_eq!(m.sections.len(), 1, "同一内容の重複定義は統合される");
    assert_eq!(
        m.elements[0].section, m.elements[1].section,
        "統合先を両部材が参照する"
    );
    assert!(
        report.notes.iter().any(|n| n.contains("統合")),
        "notes: {:?}",
        report.notes
    );
}

/// 符号＋階が同じでも断面性能が違う定義は捨てず、符号へ連番を付けて残す。
/// 定義を 1 件も失わずに「符号＋階は一意」の不変条件を保つための扱い。
#[test]
fn test_import_renames_conflicting_duplicate_sections() {
    let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<ST_BRIDGE version="2.0.2"><StbModel>
  <StbNodes>
    <StbNode id="1" X="0" Y="0" Z="0"/>
    <StbNode id="2" X="5000" Y="0" Z="0"/>
  </StbNodes>
  <StbSections>
    <StbSecBeam_S id="1" name="b3">
      <StbSecSteelFigureBeam_S>
        <StbSecSteelBeam_S_Straight shape="H-300x150x6.5x9"/>
      </StbSecSteelFigureBeam_S>
    </StbSecBeam_S>
    <StbSecBeam_S id="2" name="b3">
      <StbSecSteelFigureBeam_S>
        <StbSecSteelBeam_S_Straight shape="H-400x200x8x13"/>
      </StbSecSteelFigureBeam_S>
    </StbSecBeam_S>
    <StbSecSteel>
      <StbSecRoll-H name="H-300x150x6.5x9" type="H" A="300" B="150" t1="6.5" t2="9"/>
      <StbSecRoll-H name="H-400x200x8x13" type="H" A="400" B="200" t1="8" t2="13"/>
    </StbSecSteel>
  </StbSections>
</StbModel></ST_BRIDGE>"#;
    let (m, report) = import_stbridge_with_report(xml).expect("import");
    assert_eq!(m.sections.len(), 2, "内容が違う定義は捨てない");
    assert_eq!(m.sections[0].name, "b3");
    assert_eq!(m.sections[1].name, "b3#2", "符号へ連番を付けて一意にする");
    assert!(
        report.warnings.iter().any(|w| w.contains("b3#2")),
        "warnings: {:?}",
        report.warnings
    );
}

/// ファイルに存在した属性は、取り込んだものも取り込まなかったものもすべて報告する。
/// 無視リストを持たないため `guid` も未取り込みとして現れる。
#[test]
fn test_import_reports_attribute_dispositions() {
    let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<ST_BRIDGE version="2.0.2"><StbModel>
  <StbNodes>
    <StbNode id="1" guid="abc" X="0" Y="0" Z="0" kind="ON_GIRDER"/>
  </StbNodes>
</StbModel></ST_BRIDGE>"#;
    let (_m, report) = import_stbridge_with_report(xml).expect("import");
    let find = |attr: &str| {
        report
            .attributes
            .iter()
            .find(|a| a.element == "StbNode" && a.attribute == attr)
            .unwrap_or_else(|| panic!("{attr} の扱いが報告されていない: {:?}", report.attributes))
    };
    assert_eq!(find("X").imported, 1, "座標は取り込む");
    assert!(!find("X").is_dropped());
    assert!(find("guid").is_dropped(), "guid は取り込まない");
    assert!(find("kind").is_dropped(), "kind は取り込まない");
    // 報告は要素名・属性名の昇順（HashMap の走査順に依存しない）。
    let mut sorted = report.attributes.clone();
    sorted.sort_by(|a, b| {
        a.element
            .cmp(&b.element)
            .then_with(|| a.attribute.cmp(&b.attribute))
    });
    assert_eq!(sorted, report.attributes, "報告は整列済み");
    assert!(report.dropped_attributes().count() >= 2);
}

/// `StbStory` が標高の昇順に並んでいないファイルでも、取り込み後の
/// `Model.stories` は標高昇順・`StoryId` ＝配列位置になること。
///
/// 階への帰属区間は直下階のレベルで決まる（`Model::story_spans`）ため、
/// 並びが崩れたまま取り込むと節点が無言で別の階へ入る。
#[test]
fn test_import_sorts_stories_by_elevation() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="0" Y="0" Z="3000"/>
    <StbNode id="2" X="0" Y="0" Z="6000"/>
  </StbNodes>
  <StbStories>
    <StbStory id="0" name="RF" height="6000">
      <StbNodeIdList><StbNodeId id="2"/></StbNodeIdList>
    </StbStory>
    <StbStory id="1" name="1F" height="0"/>
    <StbStory id="2" name="2F" height="3000">
      <StbNodeIdList><StbNodeId id="1"/></StbNodeIdList>
    </StbStory>
  </StbStories>
</StbModel></ST_BRIDGE>"#;
    let m = import_stbridge(xml).expect("import");
    assert!(m.validate().is_ok(), "{:?}", m.validate());

    let names: Vec<&str> = m.stories.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names, vec!["1F", "2F", "RF"], "標高昇順へ並べ替わる");
    assert!(
        m.stories.iter().enumerate().all(|(i, s)| s.id.index() == i),
        "StoryId ＝配列位置"
    );
    // 所属階の参照も並べ替え後の ID を指す。
    assert_eq!(m.nodes[1].story, Some(StoryId(1)), "節点1 → 2F");
    assert_eq!(m.nodes[2].story, Some(StoryId(2)), "節点2 → RF");
    assert_eq!(m.stories[1].node_ids, vec![NodeId(1)]);
    assert_eq!(m.stories[2].node_ids, vec![NodeId(2)]);
}

/// StbSlab は大梁または小梁で囲まれた床板のみ。StbSecSlab_RC は書き出した床板の断面だけ。
#[test]
fn test_export_skips_plateless_and_attached_orphan_sections() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::SlabId;
    use squid_n_core::model::{LoadTransfer, RegionAnchor, Slab, SlabShape};

    fn nd(id: u32, c: [f64; 3]) -> Node {
        Node {
            id: NodeId(id),
            coord: c,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        }
    }
    let mut slab_sec =
        SectionShape::RcSlab { thickness: 150.0 }.to_section(SectionId(0), "S15".into());
    slab_sec.material = Some(MaterialId(0));
    let mut m = Model {
        nodes: vec![
            nd(0, [0.0, 0.0, 3000.0]),
            nd(1, [6000.0, 0.0, 3000.0]),
            nd(2, [6000.0, 2500.0, 3000.0]),
            nd(3, [0.0, 2500.0, 3000.0]),
        ],
        sections: vec![slab_sec],
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
    m.slabs = vec![
        Slab {
            id: SlabId(0),
            shape: SlabShape::Enclosed {
                boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            },
            plate: SlabPlate {
                section: Some(SectionId(0)),
                ..Default::default()
            },
        },
        Slab {
            id: SlabId(1),
            shape: SlabShape::Attached {
                anchor: RegionAnchor::Line {
                    nodes: [NodeId(0), NodeId(1)],
                    span: [0.0, 1.0],
                    transfer: LoadTransfer::Anchor,
                },
                extent: [-1500.0, -1500.0],
            },
            plate: SlabPlate {
                section: Some(SectionId(0)),
                ..Default::default()
            },
        },
    ];
    let xml = export_stbridge(&m).expect("export");
    let n_slab = xml.matches("<StbSlab ").count();
    let n_sec = xml.matches("<StbSecSlab_RC ").count();
    assert_eq!(n_slab, 1, "大梁または小梁で囲まれた床板だけ StbSlab\n{xml}");
    assert_eq!(n_sec, 1, "書き出した床板の断面だけ StbSecSlab_RC\n{xml}");
}

/// 大梁閉路 1 + StbSlab 1 の最小ラーメン。取り込み後は囲まれ 1・小梁 0。
#[test]
fn test_import_enclosed_frame_with_slab_folds_to_one_region() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="4000" Y="4000" Z="0"/>
    <StbNode id="3" X="0" Y="4000" Z="0"/>
  </StbNodes>
  <StbSections>
    <StbSecBeam_S id="0" name="G">
      <StbSecSteelFigureBeam_S><StbSecSteelBeam_S_Straight shape="H1" strength_main="SN400B"/></StbSecSteelFigureBeam_S>
    </StbSecBeam_S>
    <StbSecSteel>
      <StbSecRoll-H name="H1" A="300" B="150" t1="6.5" t2="9"/>
    </StbSecSteel>
    <StbSecSlab_RC id="7" name="S1">
      <StbSecFigureSlab_RC>
        <StbSecSlab_RC_Straight thickness="150"/>
      </StbSecFigureSlab_RC>
    </StbSecSlab_RC>
  </StbSections>
  <StbMembers>
    <StbGirders>
      <StbGirder id="0" id_node_start="0" id_node_end="1" id_section="0"/>
      <StbGirder id="1" id_node_start="1" id_node_end="2" id_section="0"/>
      <StbGirder id="2" id_node_start="2" id_node_end="3" id_section="0"/>
      <StbGirder id="3" id_node_start="3" id_node_end="0" id_section="0"/>
    </StbGirders>
    <StbSlab id="0" name="S1" id_section="7" kind_structure="RC">
      <StbNodeIdOrder>0 1 2 3</StbNodeIdOrder>
    </StbSlab>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, _) = import_stbridge_with_report(xml).expect("import");
    assert_eq!(m.floor_regions.len(), 1, "大梁閉路は 1 区画");
    assert_eq!(m.floor_regions[0].slab_ids.len(), 1, "床板が 1 枚帰属する");
    assert_eq!(m.slabs.len(), 1);
    assert!(!m.slabs[0].is_attached(), "囲まれ床板");
    assert_eq!(
        m.unassigned_posts.len()
            + m.unassigned_joists.len()
            + m.floor_regions
                .iter()
                .map(|r| r.secondary_joists.len())
                .sum::<usize>()
            + m.wall_regions.iter().map(|r| r.posts.len()).sum::<usize>(),
        0,
        "小梁 0"
    );
}

/// 大梁 1 本 + 跳ね出し StbSlab は取り付き領域になる。
#[test]
fn test_import_cantilever_slab_becomes_attached() {
    let xml = r#"<?xml version="1.0"?>
<ST_BRIDGE version="2.0.0"><StbModel>
  <StbNodes>
    <StbNode id="0" X="0" Y="0" Z="0"/>
    <StbNode id="1" X="4000" Y="0" Z="0"/>
    <StbNode id="2" X="4000" Y="1500" Z="0"/>
    <StbNode id="3" X="0" Y="1500" Z="0"/>
  </StbNodes>
  <StbSections>
    <StbSecBeam_S id="0" name="G">
      <StbSecSteelFigureBeam_S><StbSecSteelBeam_S_Straight shape="H1" strength_main="SN400B"/></StbSecSteelFigureBeam_S>
    </StbSecBeam_S>
    <StbSecSteel>
      <StbSecRoll-H name="H1" A="300" B="150" t1="6.5" t2="9"/>
    </StbSecSteel>
    <StbSecSlab_RC id="7" name="S1">
      <StbSecFigureSlab_RC>
        <StbSecSlab_RC_Straight thickness="150"/>
      </StbSecFigureSlab_RC>
    </StbSecSlab_RC>
  </StbSections>
  <StbMembers>
    <StbGirders>
      <StbGirder id="0" id_node_start="0" id_node_end="1" id_section="0"/>
    </StbGirders>
    <StbSlab id="0" name="S1" id_section="7" kind_structure="RC">
      <StbNodeIdOrder>0 1 2 3</StbNodeIdOrder>
    </StbSlab>
  </StbMembers>
</StbModel></ST_BRIDGE>"#;
    let (m, _) = import_stbridge_with_report(xml).expect("import");
    assert!(
        m.slabs.iter().any(|s| s.is_attached()),
        "片持ち相当は is_attached: {:?}",
        m.slabs.iter().map(|s| &s.shape).collect::<Vec<_>>()
    );
}
