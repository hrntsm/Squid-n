use super::*;
use crate::dof::Dof6Mask;
use crate::model::MaterialCategory;

fn make_grid_model(n: usize) -> Model {
    let nodes: Vec<Node> = (0..n)
        .map(|i| Node {
            id: NodeId(i as u32),
            coord: [i as f64 * 1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        })
        .collect();
    Model {
        nodes,
        ..Default::default()
    }
}

#[test]
fn test_10k_node_traverse() {
    let n = 10_000;
    let model = make_grid_model(n);
    let t = std::time::Instant::now();
    let mut s = 0.0;
    for nd in &model.nodes {
        s += nd.coord[0];
    }
    assert!(t.elapsed().as_millis() < 50, "traverse too slow");
    std::hint::black_box(s);
}

#[test]
fn test_validate_ok() {
    let model = make_grid_model(3);
    assert!(model.validate().is_ok());
}

#[test]
fn test_validate_duplicate_node() {
    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0; 3],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(0),
                coord: [1.0; 3],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        ..Default::default()
    };
    assert!(model.validate().is_err());
}

#[test]
fn test_validate_dangling_elem_node() {
    let model = Model {
        nodes: vec![Node {
            id: NodeId(0),
            coord: [0.0; 3],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        }],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(5)],
            section: None,
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        ..Default::default()
    };
    assert!(model.validate().is_err());
}

#[test]
fn test_validate_dangling_slab_boundary() {
    use crate::model::{DistributionMethod, Slab};
    let model = Model {
        nodes: vec![Node {
            id: NodeId(0),
            coord: [0.0; 3],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        }],
        slabs: vec![Slab {
            id: crate::ids::SlabId(0),
            // 存在しない節点 5 を境界に含む（陳腐化した参照）。
            boundary: vec![NodeId(0), NodeId(5)],
            joists: vec![],
            loads: vec![],
            method: DistributionMethod::TriTrapezoid,
            kind: Default::default(),
            one_way: None,
            edge_supported: None,
            usage: None,
            section: None,
        }],
        ..Default::default()
    };
    assert!(
        model.validate().is_err(),
        "存在しない節点を参照するスラブ境界は検出されるはず"
    );
}

#[test]
fn test_shear_modulus_explicit() {
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        strength_factor: None,
        name: "Test".to_string(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: Some(80000.0),
        fc: None,
        fy: None,
    };
    assert_eq!(mat.shear_modulus(), 80000.0);
}

#[test]
fn test_shear_modulus_derived() {
    let mat = Material {
        concrete_class: Default::default(),
        id: MaterialId(0),
        strength_factor: None,
        name: "Test".to_string(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    };
    let expected = 205000.0 / (2.0 * (1.0 + 0.3));
    assert!((mat.shear_modulus() - expected).abs() < 1e-9);
}

/// `concrete_class` を持たない入力は既定（Normal）で補完されること。
/// 材料の区分 `category` は必須で、欠けている入力は読み込みエラーになる
/// （区分を推測すると鋼材が黙って RC 扱いになりうるため、既定値を置かない）。
#[test]
fn test_material_serde_defaults_concrete_class() {
    let json = r#"{
            "id": 0,
            "name": "FC24",
            "category": "Concrete",
            "young": 23000.0,
            "poisson": 0.2,
            "density": 2.4e-9,
            "fc": 24.0
        }"#;
    let mat: Material = serde_json::from_str(json).unwrap();
    assert_eq!(mat.concrete_class, crate::units::ConcreteClass::Normal);
    assert_eq!(mat.category, MaterialCategory::Concrete);
    assert_eq!(mat.fc, Some(24.0));

    // 区分がない入力は受け付けない。
    let without_category = r#"{
            "id": 0,
            "name": "FC24",
            "young": 23000.0,
            "poisson": 0.2,
            "density": 2.4e-9,
            "fc": 24.0
        }"#;
    assert!(serde_json::from_str::<Material>(without_category).is_err());

    // ラウンドトリップ（Lightweight1 が保存・復元できること）。
    let mat2 = Material {
        concrete_class: crate::units::ConcreteClass::Lightweight1,
        ..mat
    };
    let s = serde_json::to_string(&mat2).unwrap();
    let back: Material = serde_json::from_str(&s).unwrap();
    assert_eq!(
        back.concrete_class,
        crate::units::ConcreteClass::Lightweight1
    );
}

#[test]
fn test_rect_shear_area() {
    let area = 80000.0;
    let as_ = rect_shear_area(area);
    assert!((as_ - area * 5.0 / 6.0).abs() < 1e-9);
}

/// 個別開口が非空なら面積和を優先し、空なら opening_area にフォールバックする。
#[test]
fn test_wall_attr_total_opening_area_prefers_openings() {
    let mut attr = WallAttr {
        elem: ElemId(0),
        opening_area: 999.0,
        opening_weight: 0.0,
        three_side_slit: false,
        openings: vec![
            WallOpening {
                width: 1000.0,
                height: 2000.0,
                offset: None,
            },
            WallOpening {
                width: 500.0,
                height: 800.0,
                offset: Some([3000.0, 500.0]),
            },
        ],
    };
    assert!((attr.total_opening_area() - (2.0e6 + 4.0e5)).abs() < 1e-9);
    assert_eq!(
        attr.opening_dims(),
        Some(vec![(1000.0, 2000.0), (500.0, 800.0)])
    );

    attr.openings.clear();
    assert!((attr.total_opening_area() - 999.0).abs() < 1e-9);
    assert_eq!(attr.opening_dims(), None);

    // 面積ゼロの開口だけなら寸法列は None(面積のみ扱い)
    attr.openings.push(WallOpening {
        width: 0.0,
        height: 1000.0,
        offset: None,
    });
    assert_eq!(attr.opening_dims(), None);
    assert_eq!(attr.total_opening_area(), 0.0);
}

fn op(w: f64, h: f64, offset: Option<[f64; 2]>) -> WallOpening {
    WallOpening {
        width: w,
        height: h,
        offset,
    }
}

fn attr_with(openings: Vec<WallOpening>) -> WallAttr {
    WallAttr {
        elem: ElemId(0),
        opening_area: 0.0,
        opening_weight: 0.0,
        three_side_slit: false,
        openings,
    }
}

/// 包絡モード: 位置を持つ開口は外接矩形1つに統合、位置不明は個別のまま。
#[test]
fn test_openings_for_mode_envelope() {
    let attr = attr_with(vec![
        op(1000.0, 1000.0, Some([0.0, 0.0])),
        op(500.0, 800.0, Some([2000.0, 1200.0])),
        op(300.0, 300.0, None), // 位置不明
    ]);
    let out = attr.openings_for_mode(MultiOpeningMode::Envelope);
    assert_eq!(out.len(), 2);
    // 包絡矩形: x0=0,z0=0,x1=2500,z1=2000
    assert!((out[0].width - 2500.0).abs() < 1e-9);
    assert!((out[0].height - 2000.0).abs() < 1e-9);
    assert_eq!(out[0].offset, Some([0.0, 0.0]));
    assert!((out[1].width - 300.0).abs() < 1e-9);
    // 包絡モードの面積は包絡矩形基準(生の面積和より大きい)
    let a_env = attr.total_opening_area_for(MultiOpeningMode::Envelope);
    assert!(a_env > attr.total_opening_area());
}

/// 自動判定: 近接対のみ包絡を繰り返し、離れた開口は残る。
#[test]
fn test_openings_for_mode_auto_merges_close_pairs_only() {
    // 開口1と2は水平間隔200(≤min幅)で包絡可能。開口3は間隔5000で不可。
    let attr = attr_with(vec![
        op(1000.0, 2000.0, Some([0.0, 0.0])),
        op(800.0, 2000.0, Some([1200.0, 0.0])),
        op(900.0, 2000.0, Some([7000.0, 0.0])),
    ]);
    let out = attr.openings_for_mode(MultiOpeningMode::Auto);
    assert_eq!(out.len(), 2);
    // 包絡結果: 幅 0..2000
    assert!((out[0].width - 2000.0).abs() < 1e-9);
    assert!((out[1].width - 900.0).abs() < 1e-9);
    // 等価モードは元のまま
    assert_eq!(
        attr.openings_for_mode(MultiOpeningMode::Equivalent).len(),
        3
    );
}

/// 自動判定の包絡可能条件(耐震壁の複数開口の取り扱いの判定図。RC 規準):
/// l < 1.5h または l < 1m(l: 開口間距離、h: 包絡開口とした場合の高さ)。
#[test]
fn test_can_envelope_boundary() {
    // h(包絡高さ)=2000 → 1.5h=3000
    let a = op(1000.0, 2000.0, Some([0.0, 0.0]));
    // 開口間距離 2999 < 1.5h → 包絡可
    let b = op(1000.0, 2000.0, Some([3999.0, 0.0]));
    assert!(a.can_envelope(&b));
    // 開口間距離 3000 = 1.5h(かつ ≥1m) → 不可
    let c = op(1000.0, 2000.0, Some([4000.0, 0.0]));
    assert!(!a.can_envelope(&c));

    // 低い開口(h=500 → 1.5h=750 < 1m)でも l < 1m なら包絡可
    let e = op(1000.0, 500.0, Some([0.0, 0.0]));
    let f = op(1000.0, 500.0, Some([1999.0, 0.0])); // l=999 < 1000
    assert!(e.can_envelope(&f));
    let g = op(1000.0, 500.0, Some([2000.0, 0.0])); // l=1000(≥1m かつ ≥1.5h)
    assert!(!e.can_envelope(&g));

    // 位置不明は不可
    let d = op(1000.0, 2000.0, None);
    assert!(!a.can_envelope(&d));
}

/// 旧スキーマ(openings 無し)の WallAttr が読み込めること(serde 後方互換)。
#[test]
fn test_wall_attr_serde_backward_compat() {
    let json = r#"{"elem":3,"opening_area":1200.0,"three_side_slit":true}"#;
    let attr: WallAttr = serde_json::from_str(json).unwrap();
    assert_eq!(attr.elem, ElemId(3));
    assert!(attr.openings.is_empty());
    assert!((attr.total_opening_area() - 1200.0).abs() < 1e-9);
    assert!(attr.three_side_slit);
}

#[test]
fn test_section_new_fields_default() {
    let sec = Section {
        id: SectionId(0),
        name: "Test".to_string(),
        area: 100.0,
        iy: 1000.0,
        iz: 2000.0,
        j: 500.0,
        depth: 0.0,
        width: 0.0,
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
    };
    assert_eq!(sec.depth, 0.0);
    assert!(sec.panel_thickness.is_none());
}

#[test]
fn test_element_data_plastic_zone_default_missing_field() {
    // 旧スキーマ（plastic_zone フィールドがない JSON）からの互換性を確認する。
    let json = r#"{
            "id": 0,
            "kind": "Beam",
            "nodes": [0, 1],
            "section": null,
            "material": null,
            "local_axis": { "ref_vector": [1.0, 0.0, 0.0] },
            "end_cond": ["Fixed", "Fixed"],
            "force_regime": "Auto"
        }"#;
    let elem: ElementData = serde_json::from_str(json).unwrap();
    assert_eq!(elem.plastic_zone, None);
    assert_eq!(elem.rigid_zone, RigidZone::default());
}

/// 長期系（固定・積載・積雪・種別未指定）は長期、地震用積載・風・地震は短期
/// （令82条の応力解析。長期軸力無効化条件の適用範囲）。
#[test]
fn test_load_case_kind_is_long_term() {
    assert!(LoadCaseKind::Dead.is_long_term());
    assert!(LoadCaseKind::Live.is_long_term());
    assert!(LoadCaseKind::Snow.is_long_term());
    assert!(LoadCaseKind::Other.is_long_term());
    assert!(!LoadCaseKind::LiveSeismic.is_long_term());
    assert!(!LoadCaseKind::Wind.is_long_term());
    assert!(!LoadCaseKind::Seismic.is_long_term());
}

#[test]
fn test_stress_cfg_default_is_false() {
    let cfg = StressAnalysisCfg::default();
    assert!(!cfg.no_long_axial_brace);
    assert!(!cfg.no_long_axial_column);
    assert_eq!(Model::default().stress_cfg, cfg);
}

#[test]
fn test_model_stress_cfg_default_missing_field() {
    // 旧スキーマ（stress_cfg フィールドがない JSON）からの互換性を確認する。
    let json = r#"{
            "nodes": [], "elements": [], "sections": [], "materials": [],
            "stories": [], "slabs": [], "constraints": [], "load_cases": [],
            "combinations": []
        }"#;
    let model: Model = serde_json::from_str(json).unwrap();
    assert_eq!(model.stress_cfg, StressAnalysisCfg::default());
}

/// 旧スキーマ（support_spring フィールドがない JSON）の Node が読み込めること
/// （serde 後方互換。既定は None＝ばね支持なし）。
#[test]
fn test_node_support_spring_default_missing_field() {
    let json = r#"{
            "id": 0,
            "coord": [0.0, 0.0, 0.0],
            "restraint": 0,
            "mass": null,
            "story": null
        }"#;
    let node: Node = serde_json::from_str(json).unwrap();
    assert_eq!(node.support_spring, None);
}

/// 旧スキーマ（relief_velocity/c2_ratio フィールドがない JSON）の DamperProps が
/// 読み込めること（serde 後方互換。既定は両方とも None＝リリーフなし。
/// 既存前例の qy/k2_ratio と同じ扱い）。
#[test]
fn test_damper_props_relief_default_missing_field() {
    let json = r#"{
            "kind": "Maxwell",
            "kd": 100000.0,
            "c0": 1000.0,
            "alpha": 1.0
        }"#;
    let props: DamperProps = serde_json::from_str(json).unwrap();
    assert_eq!(props.relief_velocity, None);
    assert_eq!(props.c2_ratio, None);
    // qy/k2_ratio も既存前例どおり既定値で補完される（DamperProps::default() と同じ値）。
    assert_eq!(props.qy, DamperProps::default().qy);
    assert_eq!(props.k2_ratio, DamperProps::default().k2_ratio);
}

/// 旧スキーマ（damper_defs フィールドがない JSON）の Model が読み込めること
/// （serde 後方互換。既定は空の Vec）。
#[test]
fn test_model_damper_defs_default_missing_field() {
    let json = r#"{
            "nodes": [], "elements": [], "sections": [], "materials": [],
            "stories": [], "slabs": [], "constraints": [], "load_cases": [],
            "combinations": []
        }"#;
    let model: Model = serde_json::from_str(json).unwrap();
    assert!(model.damper_defs.is_empty());
}

/// msgpack（.scz の実際の直列化形式）でも同様に後方互換が効くこと
/// （JSON だけでなくバイナリ形式での確認。rmp-serde は位置ベース配列として
/// 直列化するため、`#[serde(default)]` による補完は**末尾のフィールドが
/// 欠けている場合のみ**有効。新フィールドを構造体の途中に追加すると、
/// 旧データの後続フィールドの値がずれて読み込まれてしまう）。
#[test]
fn test_node_support_spring_msgpack_backward_compat() {
    // 旧版 Node 相当（末尾 support_spring 抜き）を模した最小構造体で msgpack 化し、
    // 現行の Node へデシリアライズできることを確認する。
    #[derive(serde::Serialize)]
    struct LegacyNode {
        id: NodeId,
        coord: [f64; 3],
        restraint: Dof6Mask,
        mass: Option<[f64; 6]>,
        story: Option<StoryId>,
    }
    let legacy = LegacyNode {
        id: NodeId(0),
        coord: [1.0, 2.0, 3.0],
        restraint: Dof6Mask::FIXED,
        mass: None,
        story: None,
    };
    let bytes = rmp_serde::to_vec(&legacy).expect("legacy msgpack serialize");
    let node: Node = rmp_serde::from_slice(&bytes).expect("legacy msgpack deserialize as Node");
    assert_eq!(node.id, NodeId(0));
    assert_eq!(node.coord, [1.0, 2.0, 3.0]);
    assert_eq!(node.restraint, Dof6Mask::FIXED);
    assert_eq!(node.support_spring, None);
}

#[test]
fn test_validate_index_mismatch() {
    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0; 3],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            },
            Node {
                id: NodeId(5),
                coord: [1.0; 3],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
                support_spring: None,
            },
        ],
        ..Default::default()
    };
    assert!(model.validate().is_err());
}

#[test]
fn test_default_member_hysteresis_table() {
    // 本実装の既定の非線形特性（各履歴則の原典）: 梁曲げは
    // RC/SRC/CFT=武田型、S=標準型。
    assert_eq!(default_member_hysteresis(true), HysteresisModel::Takeda);
    assert_eq!(default_member_hysteresis(false), HysteresisModel::Standard);
}

#[test]
fn test_set_member_hysteresis_roundtrip() {
    let mut model = Model::default();
    let e = ElemId(3);
    // 既定は None（＝Auto）。
    assert_eq!(model.member_hysteresis(e), None);
    let old = model.set_member_hysteresis(e, HysteresisModel::OriginOriented);
    assert_eq!(old, None);
    assert_eq!(
        model.member_hysteresis(e),
        Some(HysteresisModel::OriginOriented)
    );
    // 上書き。
    let old = model.set_member_hysteresis(e, HysteresisModel::Takeda);
    assert_eq!(old, Some(HysteresisModel::OriginOriented));
    // Auto で解除。
    let old = model.set_member_hysteresis(e, HysteresisModel::Auto);
    assert_eq!(old, Some(HysteresisModel::Takeda));
    assert_eq!(model.member_hysteresis(e), None);
    assert!(model.member_hysteresis_attrs.is_empty());
}

/// 標準荷重ケース一式（DL・LL(架構用)・LL(地震用)・EX・EY）の構成と、
/// `Model::with_default_load_cases` が validate を通ることを確認する。
#[test]
fn test_default_load_cases_and_model() {
    let cases = default_load_cases();
    let names: Vec<&str> = cases.iter().map(|c| c.name.as_str()).collect();
    assert_eq!(
        names,
        vec![
            DL_CASE_NAME,
            LL_FRAME_CASE_NAME,
            LL_SEISMIC_CASE_NAME,
            EX_CASE_NAME,
            EY_CASE_NAME
        ]
    );
    let kinds: Vec<LoadCaseKind> = cases.iter().map(|c| c.kind).collect();
    assert_eq!(
        kinds,
        vec![
            LoadCaseKind::Dead,
            LoadCaseKind::Live,
            LoadCaseKind::LiveSeismic,
            LoadCaseKind::Seismic,
            LoadCaseKind::Seismic
        ]
    );
    // id == 添字の規約・内容は空。
    for (i, c) in cases.iter().enumerate() {
        assert_eq!(c.id.index(), i);
        assert!(c.nodal.is_empty() && c.member.is_empty());
    }
    let model = Model::with_default_load_cases();
    assert!(model.validate().is_ok());
    assert_eq!(model.load_cases.len(), 5);
    // 新規モデルは標準荷重組合せ（長期 G+P、短期地震 G+P±Kx・G+P±Ky）も持つ。
    assert_eq!(model.combinations, default_combinations());
}

/// 標準荷重組合せ（`default_combinations`）の構成を確認する。
/// 長期 DL+LL（DL+LL(架構用)）と短期地震 DL+LL±EX・DL+LL±EY の計5組合せ。
/// 組合せ名は荷重ケースの直接的な名前（DL・LL・EX・EY）を用いる。
#[test]
fn test_default_combinations() {
    let combos = default_combinations();
    let names: Vec<&str> = combos.iter().map(|c| c.name.as_str()).collect();
    assert_eq!(
        names,
        vec![
            "DL + LL",
            "DL + LL + EX",
            "DL + LL - EX",
            "DL + LL + EY",
            "DL + LL - EY"
        ]
    );
    // 長期 DL+LL: DL(0)+LL(架構用=1) を各1.0で参照する。
    assert_eq!(
        combos[0].terms,
        vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), 1.0)]
    );
    // 短期地震: DL+LL に EX(3)/EY(4) を ±1.0 で加える。
    assert_eq!(
        combos[1].terms,
        vec![
            (LoadCaseId(0), 1.0),
            (LoadCaseId(1), 1.0),
            (LoadCaseId(3), 1.0)
        ]
    );
    assert_eq!(
        combos[2].terms,
        vec![
            (LoadCaseId(0), 1.0),
            (LoadCaseId(1), 1.0),
            (LoadCaseId(3), -1.0)
        ]
    );
    assert_eq!(
        combos[3].terms,
        vec![
            (LoadCaseId(0), 1.0),
            (LoadCaseId(1), 1.0),
            (LoadCaseId(4), 1.0)
        ]
    );
    assert_eq!(
        combos[4].terms,
        vec![
            (LoadCaseId(0), 1.0),
            (LoadCaseId(1), 1.0),
            (LoadCaseId(4), -1.0)
        ]
    );
    // 参照する荷重ケース ID は default_load_cases() の DL/LL(架構用)/EX/EY に対応する。
    let cases = default_load_cases();
    assert_eq!(cases[0].name, DL_CASE_NAME);
    assert_eq!(cases[1].name, LL_FRAME_CASE_NAME);
    assert_eq!(cases[3].name, EX_CASE_NAME);
    assert_eq!(cases[4].name, EY_CASE_NAME);
}

/// 旧スキーマの自動生成ケース名の移行: 改名（床荷重(自動)→DL 等）と、
/// 「自重(自動)」の DL への統合（組合せ参照の付け替え・重複項の除去・
/// id == 添字規約の維持）を確認する。
#[test]
fn test_migrate_legacy_auto_load_cases() {
    let mk = |i: u32, name: &str, kind: LoadCaseKind| LoadCase {
        id: LoadCaseId(i),
        name: name.into(),
        nodal: Vec::new(),
        member: Vec::new(),
        kind,
    };
    // 旧構成: 手動ケース + 床荷重(自動) + 自重(自動) + 床積載(自動)。
    let mut model = Model {
        load_cases: vec![
            mk(0, "手動", LoadCaseKind::Other),
            mk(1, "床荷重(自動)", LoadCaseKind::Dead),
            mk(2, "自重(自動)", LoadCaseKind::Dead),
            mk(3, "床積載(自動)", LoadCaseKind::Live),
        ],
        combinations: vec![
            // 自重と床荷重の両方を参照する組合せ → 自重項は除去される。
            LoadCombination {
                name: "G+P".into(),
                terms: vec![
                    (LoadCaseId(1), 1.0),
                    (LoadCaseId(2), 1.0),
                    (LoadCaseId(3), 1.0),
                ],
            },
            // 自重のみ参照する組合せ → DL へ付け替え。
            LoadCombination {
                name: "自重のみ".into(),
                terms: vec![(LoadCaseId(2), 1.0)],
            },
        ],
        ..Default::default()
    };
    model.migrate_legacy_auto_load_cases();
    assert!(model.validate().is_ok(), "{:?}", model.validate());

    let names: Vec<&str> = model.load_cases.iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names, vec!["手動", DL_CASE_NAME, LL_FRAME_CASE_NAME]);
    let dl_id = model.load_cases[1].id;
    // G+P: 自重項が除去され、床積載(→LL(架構用)、id 3→2)の参照が詰め直される。
    assert_eq!(
        model.combinations[0].terms,
        vec![(dl_id, 1.0), (LoadCaseId(2), 1.0)]
    );
    // 自重のみ: DL へ付け替え。
    assert_eq!(model.combinations[1].terms, vec![(dl_id, 1.0)]);
}

/// 「自重(自動)」だけがある旧モデルは DL へ改名される（削除しない）。
#[test]
fn test_migrate_legacy_self_weight_only_renames_to_dl() {
    let mut model = Model {
        load_cases: vec![LoadCase {
            id: LoadCaseId(0),
            name: "自重(自動)".into(),
            nodal: Vec::new(),
            member: Vec::new(),
            kind: LoadCaseKind::Dead,
        }],
        ..Default::default()
    };
    model.migrate_legacy_auto_load_cases();
    assert_eq!(model.load_cases.len(), 1);
    assert_eq!(model.load_cases[0].name, DL_CASE_NAME);
    assert_eq!(model.load_cases[0].kind, LoadCaseKind::Dead);
}

/// 断面・材料の割当が必須なのは剛性を断面諸元から作る線材・面材のみ。
#[test]
fn test_requires_section_and_material_covers_line_and_area_elements() {
    for kind in [
        ElementKind::Beam,
        ElementKind::Fiber,
        ElementKind::MultiSpring,
        ElementKind::Brace {
            tension_only: false,
        },
        ElementKind::Brace { tension_only: true },
        ElementKind::Shell,
        ElementKind::Wall,
    ] {
        assert!(
            kind.requires_section_and_material(),
            "{kind:?} は断面・材料から剛性を作る"
        );
    }
}

/// 仕口パネル・節点バネ・免震・ダンパーは断面を持たないのが正常な状態であり、
/// 未割当の検出対象にしてはならない（準備計算が自動生成する仕口パネル要素が
/// そのまま「断面未割当」警告になっていた不具合の再発防止）。
#[test]
fn test_requires_section_and_material_excludes_property_driven_elements() {
    for kind in [
        ElementKind::PanelZone,
        ElementKind::NodalSpring,
        ElementKind::Isolator,
        ElementKind::Damper,
    ] {
        assert!(
            !kind.requires_section_and_material(),
            "{kind:?} は専用の特性値から剛性を作るため断面・材料を持たない"
        );
    }
}

// ---------------------------------------------------------------------------
// 断面の同一性キー（符号＋階）
// ---------------------------------------------------------------------------

fn named_section(id: u32, name: &str, floor: Option<&str>) -> Section {
    Section {
        id: crate::ids::SectionId(id),
        name: name.to_string(),
        floor: floor.map(str::to_string),
        area: 1.0e4,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 300.0,
        width: 300.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    }
}

/// 同一性キーは符号と階の組で、階が違えば別断面になる。
#[test]
fn test_section_key_pairs_name_and_floor() {
    let a = named_section(0, "C1", Some("1"));
    let b = named_section(1, "C1", Some("2"));
    let c = named_section(2, "C1", None);
    assert_ne!(a.key(), b.key(), "階が違えば別のキー");
    assert_ne!(a.key(), c.key(), "階の有無も区別する");
    assert_eq!(a.key(), ("C1", Some("1")));
    assert_eq!(c.key(), ("C1", None));
}

/// 表示用ラベルは階を持つ断面だけ括弧で階を添える。
#[test]
fn test_section_display_name() {
    assert_eq!(named_section(0, "C1", Some("2")).display_name(), "C1 (2)");
    assert_eq!(named_section(0, "C1", None).display_name(), "C1");
}

/// `section_key_taken` は自分自身を除外できる（改名で自分と衝突しないため）。
#[test]
fn test_section_key_taken_skips_self() {
    let sections = vec![
        named_section(0, "C1", Some("1")),
        named_section(1, "C1", Some("2")),
    ];
    assert!(section_key_taken(&sections, ("C1", Some("1")), None));
    assert!(
        !section_key_taken(&sections, ("C1", Some("1")), Some(0)),
        "自分自身は衝突扱いにしない"
    );
    assert!(
        section_key_taken(&sections, ("C1", Some("1")), Some(1)),
        "他の断面との衝突は検出する"
    );
    assert!(!section_key_taken(&sections, ("C1", Some("3")), None));
    assert!(!section_key_taken(&sections, ("C1", None), None));
}

/// `properties_eq` は同一性キーを見ず、断面性能・形状・材料を比べる。
/// 取り込みで符号＋階が衝突した定義を統合してよいかの判定に使う。
#[test]
fn test_section_properties_eq_ignores_key() {
    let a = named_section(0, "C1", Some("1"));
    let mut b = named_section(1, "C9", Some("PH1"));
    assert!(
        a.properties_eq(&b),
        "符号・階・ID が違っても中身が同じなら真"
    );
    b.iy *= 2.0;
    assert!(!a.properties_eq(&b));
}

/// 材料だけが違う断面は `properties_eq` で偽になる。
/// 材料は断面が持つため、統合すると片方の材料が無言で捨てられる。
#[test]
fn test_section_properties_eq_compares_materials() {
    let base = named_section(0, "C1", Some("1"));
    for (label, mutate) in [
        (
            "主材料",
            (|s: &mut Section| s.material = Some(MaterialId(1))) as fn(&mut Section),
        ),
        ("主筋", |s: &mut Section| {
            s.rebar_material = Some(MaterialId(1))
        }),
        ("せん断補強筋", |s: &mut Section| {
            s.shear_rebar_material = Some(MaterialId(1))
        }),
        ("内蔵鉄骨", |s: &mut Section| {
            s.steel_material = Some(MaterialId(1))
        }),
    ] {
        let mut other = base.clone();
        mutate(&mut other);
        assert!(
            !base.properties_eq(&other),
            "{label}だけが違う断面は別の断面として扱う"
        );
    }
}

// ---- 階と剛床の分離（階帰属は区間・剛床帰属は床面） ----

/// 標高 `z` の節点を並べ、階を `levels` で定義したモデル。
fn make_story_model(zs: &[f64], levels: &[(&str, f64)]) -> Model {
    let nodes: Vec<Node> = zs
        .iter()
        .enumerate()
        .map(|(i, &z)| Node {
            id: NodeId(i as u32),
            coord: [0.0, 0.0, z],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        })
        .collect();
    let stories: Vec<Story> = levels
        .iter()
        .enumerate()
        .map(|(i, &(name, elevation))| Story {
            id: StoryId(i as u32),
            name: name.into(),
            elevation,
            node_ids: Vec::new(),
            seismic_weight: None,
            weight_override: None,
            structure: Default::default(),
            level_kind: Default::default(),
        })
        .collect();
    Model {
        nodes,
        stories,
        ..Default::default()
    }
}

/// 階への帰属区間は「直下階のレベル超〜当該階のレベル以下」。
/// 最下階（基部の床）だけは下端を含む点区間で、柱脚・基礎梁の節点が属する。
#[test]
fn test_story_spans_are_half_open_intervals_above_base() {
    let m = make_story_model(
        &[0.0, 4000.0, 7500.0],
        &[("1F", 0.0), ("2F", 4000.0), ("3F", 7500.0)],
    );
    assert_eq!(m.base_elevation(), 0.0);
    assert_eq!(
        m.story_spans(),
        vec![(0.0, 0.0), (0.0, 4000.0), (4000.0, 7500.0)]
    );

    let spans = m.story_spans();
    assert_eq!(
        m.story_at(&spans, 0.0),
        Some(StoryId(0)),
        "基部の節点は基部の階に属する"
    );
    assert_eq!(
        m.story_at(&spans, 4000.0),
        Some(StoryId(1)),
        "床レベルは当該階"
    );
    assert_eq!(m.story_at(&spans, 7500.0), Some(StoryId(2)));
    assert_eq!(m.story_at(&spans, 9000.0), None, "最上階より上は属さない");
    assert_eq!(m.story_at(&spans, -100.0), None, "基部より下は属さない");
}

/// 不変条件がまだ成立していないモデル（基部の階を持たない旧形式のファイル）でも、
/// 最下階の区間は基部から始まり、基部〜最下階の節点がどの階にも属さなくなることはない。
///
/// これがないと、階生成を通す前に伏図を開いた時点で最下階が空になる。
#[test]
fn test_lowest_span_starts_at_base_when_invariant_not_yet_established() {
    // 旧形式: 層の上端の床だけを階として持つ（基部の階がない）。
    let m = make_story_model(&[0.0, 4000.0, 7500.0], &[("2F", 4000.0), ("3F", 7500.0)]);
    assert_eq!(m.base_elevation(), 0.0);
    assert_eq!(
        m.story_spans(),
        vec![(0.0, 4000.0), (4000.0, 7500.0)],
        "最下階の下端は基部まで下がる"
    );
    let stories = m.node_stories();
    assert_eq!(stories[0], Some(StoryId(0)), "基部の節点が最下階へ収まる");
    assert_eq!(stories[1], Some(StoryId(0)));
    assert_eq!(stories[2], Some(StoryId(1)));

    // 基部から許容差以内だが厳密には上にある階でも、柱脚が無所属にならない。
    let m = make_story_model(&[0.0, 4000.0], &[("1F", 0.5), ("2F", 4000.0)]);
    assert_eq!(m.node_stories()[0], Some(StoryId(0)));
}

/// 層は隣り合う階の間であり、層数は階数より 1 つ少ない。
/// 名前は下端の階、重量・所属節点・階種別は上端の階から採る。
#[test]
fn test_layers_pair_adjacent_stories() {
    let mut m = make_story_model(
        &[0.0, 4000.0, 7500.0],
        &[("1F", 0.0), ("2F", 4000.0), ("3F", 7500.0)],
    );
    m.stories[1].seismic_weight = Some(1000.0);
    m.stories[2].seismic_weight = Some(800.0);
    m.stories[0].seismic_weight = Some(300.0);

    let layers = m.layers();
    assert_eq!(m.layer_count(), 2);
    assert_eq!(layers.len(), 2);

    assert_eq!(layers[0].index, 0);
    assert_eq!(
        layers[0].name, "1F",
        "層の名前は下端の階名（法令の「1 階」）"
    );
    assert_eq!(layers[0].bottom, StoryId(0));
    assert_eq!(layers[0].top, StoryId(1));
    assert_eq!(layers[0].height, 4000.0);
    assert_eq!(
        layers[0].weight,
        Some(1000.0),
        "層の重量は上端の階が持つ（基部の重量は層に入らない）"
    );

    assert_eq!(layers[1].name, "2F");
    assert_eq!(layers[1].height, 3500.0);
    assert_eq!(layers[1].weight, Some(800.0));
}

/// 中間高さの節点（柱の分割点）は階には属するが、剛床の床面には載らない。
#[test]
fn test_mid_height_node_belongs_to_story_but_not_to_diaphragm_level() {
    let m = make_story_model(&[0.0, 2000.0, 4000.0], &[("1F", 0.0), ("2F", 4000.0)]);
    let stories = m.node_stories();
    assert_eq!(stories[0], Some(StoryId(0)), "基部の節点は基部の階");
    assert_eq!(stories[1], Some(StoryId(1)), "中間節点も階には属する");
    assert_eq!(stories[2], Some(StoryId(1)));

    assert!(
        !m.on_diaphragm_level(StoryId(1), 2000.0),
        "中間節点は床面ではない"
    );
    assert!(m.on_diaphragm_level(StoryId(1), 4000.0));
    // 許容差ちょうどは床面に含める。
    assert!(m.on_diaphragm_level(StoryId(1), 4000.0 + DIAPHRAGM_LEVEL_TOL_MM));
    assert!(!m.on_diaphragm_level(StoryId(1), 4000.0 + DIAPHRAGM_LEVEL_TOL_MM * 2.0));
}

/// 段差床（同じ階に 2 つのレベル）は同一階に属し、剛床は 2 つに分かれる。
#[test]
fn test_split_level_floor_shares_story_with_two_diaphragms() {
    let mut m = make_story_model(&[0.0, 4000.0, 4500.0], &[("1F", 0.0), ("2F", 4500.0)]);
    let stories = m.node_stories();
    assert_eq!(stories[1], Some(StoryId(1)), "段差の下側も同じ階");
    assert_eq!(stories[2], Some(StoryId(1)));

    m.constraints
        .push(Constraint::rigid_diaphragm(StoryId(1), NodeId(1), vec![]));
    m.constraints
        .push(Constraint::rigid_diaphragm(StoryId(1), NodeId(2), vec![]));
    assert_eq!(m.diaphragms_of(StoryId(1)).count(), 2);
    assert!(m.node_on_rigid_diaphragm(NodeId(1)));
    assert!(!m.node_on_rigid_diaphragm(NodeId(0)));
}

/// 剛床を持たない階も成立する（階と剛床は別概念）。
#[test]
fn test_story_without_diaphragm_is_valid() {
    let m = make_story_model(&[0.0, 4000.0], &[("1F", 4000.0)]);
    assert_eq!(m.diaphragms_of(StoryId(0)).count(), 0);
    assert!(!m.node_on_rigid_diaphragm(NodeId(1)));
    assert!(m.validate().is_ok());
}

/// `visit_story_ids` が階自身・節点の所属階・剛床拘束のすべてを走査する。
#[test]
fn test_visit_story_ids_covers_all_references() {
    let mut m = make_story_model(&[0.0, 4000.0], &[("1F", 4000.0)]);
    m.nodes[1].story = Some(StoryId(0));
    m.constraints
        .push(Constraint::rigid_diaphragm(StoryId(0), NodeId(1), vec![]));

    let mut seen = 0usize;
    m.visit_story_ids(|sid| {
        seen += 1;
        *sid = StoryId(sid.0 + 10);
    });
    assert_eq!(seen, 3, "階自身・節点の所属階・剛床拘束の 3 箇所");
    assert_eq!(m.stories[0].id, StoryId(10));
    assert_eq!(m.nodes[1].story, Some(StoryId(10)));
    assert_eq!(m.diaphragms_of(StoryId(10)).count(), 1);
}

/// 階が標高の昇順に並んでいないモデルは検証で弾く。階への帰属区間は直下階の
/// レベルで決まるため、並びが崩れると節点が無言で別の階へ入る。
#[test]
fn test_validate_rejects_stories_out_of_elevation_order() {
    let mut m = make_story_model(&[0.0, 4000.0, 7500.0], &[("1F", 4000.0), ("2F", 7500.0)]);
    assert!(m.validate().is_ok());

    m.stories[1].elevation = 1000.0;
    let err = m.validate().expect_err("標高の逆転を検出する");
    let msg = format!("{err}");
    assert!(msg.contains("昇順"), "{msg}");
}
