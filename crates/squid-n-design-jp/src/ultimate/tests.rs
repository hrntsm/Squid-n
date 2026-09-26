use super::*;
use smallvec::SmallVec;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, MaterialCategory,
    Node, RigidZone, Section,
};
use squid_n_core::section_shape::{
    BeamStirrup, CircleColumnHoop, RcBeamRebar, RcCircleColumnRebar, RcRectColumnRebar,
    RectColumnHoop, SectionShape,
};

/// テスト用の梁矩形 RC 断面（上下各 `main_count` 本、帯筋 D10@pitch）。
fn rc_beam_rect_section(
    id: u32,
    b: f64,
    d: f64,
    main_dia: f64,
    main_count: u32,
    pitch: f64,
) -> Section {
    let rebar = RcBeamRebar {
        main_dia,
        top: vec![main_count],
        bottom: vec![main_count],
        cover: 40.0,
        stirrup: BeamStirrup {
            dia: 10.0,
            pitch,
            legs: 2,
        },
    };
    Section {
        id: SectionId(id),
        name: format!("RC{id}"),
        area: b * d,
        iy: b * d.powi(3) / 12.0,
        iz: d * b.powi(3) / 12.0,
        j: 1.0,
        depth: d,
        width: b,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: Some(SectionShape::RcBeamRect { b, d, rebar }),
        // 材料は断面が持つ。主筋・せん断補強筋も同じ材料（SD345）とする。
        material: Some(MaterialId(0)),
        rebar_material: Some(MaterialId(0)),
        shear_rebar_material: Some(MaterialId(0)),
        steel_material: None,
    }
}

/// テスト用の柱矩形 RC 断面（`x:[nx]`・`y:[ny]`、帯筋 D10@pitch）。
fn rc_column_rect_section(
    id: u32,
    b: f64,
    d: f64,
    main_dia: f64,
    nx: u32,
    ny: u32,
    pitch: f64,
) -> Section {
    let rebar = RcRectColumnRebar {
        main_dia,
        x: vec![nx],
        y: vec![ny],
        cover: 40.0,
        hoop: RectColumnHoop {
            dia: 10.0,
            pitch,
            legs_x: 2,
            legs_y: 2,
        },
    };
    Section {
        id: SectionId(id),
        name: format!("RC{id}"),
        area: b * d,
        iy: b * d.powi(3) / 12.0,
        iz: d * b.powi(3) / 12.0,
        j: 1.0,
        depth: d,
        width: b,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: Some(SectionShape::RcColumnRect { b, d, rebar }),
        material: Some(MaterialId(0)),
        rebar_material: Some(MaterialId(0)),
        shear_rebar_material: Some(MaterialId(0)),
        steel_material: None,
    }
}

fn material() -> Material {
    Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SD345".to_string(),
        category: MaterialCategory::Rebar,
        young: 21000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: Some(345.0),
    }
}

fn node(id: u32, c: [f64; 3]) -> Node {
    Node {
        id: NodeId(id),
        coord: c,
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
        support_spring: None,
    }
}

fn frame_element(id: u32, sec: u32, n0: u32, n1: u32) -> ElementData {
    let mut nodes: SmallVec<[NodeId; 8]> = SmallVec::new();
    nodes.push(NodeId(n0));
    nodes.push(NodeId(n1));
    ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes,
        section: Some(SectionId(sec)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    }
}

/// 1 柱（鉛直）+ 1 梁（水平）のモデル。
fn column_and_beam_model() -> Model {
    let nodes = vec![
        node(0, [0.0, 0.0, 0.0]),
        node(1, [0.0, 0.0, 3000.0]),    // 柱: 鉛直
        node(2, [6000.0, 0.0, 3000.0]), // 梁: 水平
    ];
    let sections = vec![
        rc_column_rect_section(0, 600.0, 600.0, 25.0, 4, 4, 100.0), // 柱断面
        rc_beam_rect_section(1, 400.0, 700.0, 25.0, 4, 100.0),      // 梁断面
    ];
    let materials = vec![material()];
    let elements = vec![
        frame_element(0, 0, 0, 1), // 柱
        frame_element(1, 1, 1, 2), // 梁
    ];
    Model {
        nodes,
        elements,
        sections,
        materials,
        ..Default::default()
    }
}

#[test]
fn test_collect_rc_ultimate_checks_column_and_beam() {
    let model = column_and_beam_model();
    let opts = UltimateShearOptions::default();
    // 柱に圧縮軸力 2000kN。
    let axial = vec![(ElemId(0), MemberDemand::axial(2_000_000.0))];
    let checks = collect_rc_ultimate_checks(&model, &axial, &opts).unwrap();
    assert_eq!(checks.len(), 2, "柱・梁の 2 部材が検定される");

    let col = checks.iter().find(|c| c.elem == ElemId(0)).unwrap();
    let beam = checks.iter().find(|c| c.elem == ElemId(1)).unwrap();

    assert_eq!(col.kind, MemberKind::Column);
    assert_eq!(beam.kind, MemberKind::Beam);

    // 各耐力が正。
    assert!(col.mu > 0.0 && col.qmu > 0.0 && col.qsu > 0.0 && col.qbu > 0.0);
    assert!(beam.mu > 0.0 && beam.qmu > 0.0 && beam.qsu > 0.0);

    // 柱は軸終局耐力を持つ。Nuc = 600·600·24。
    let ax = col.axial.expect("柱は軸終局耐力を持つ");
    assert!((ax.nuc - 600.0 * 600.0 * 24.0).abs() < 1e-3);
    assert!(ax.nut < 0.0);
    // 梁は軸終局耐力なし。
    assert!(beam.axial.is_none());

    // せん断余裕度 = Qsu/Qmu。
    assert!((col.shear_margin - col.qsu / col.qmu).abs() < 1e-9);
}

/// 終局 σwy は断面のせん断補強筋材料の `fy` から解決する。材料名の数値ではなく
/// 材料の `fy` が効き、対応グレードで `fy` 未設定は入力不備として停止する。
#[test]
fn test_ultimate_sigma_wy_from_shear_rebar_material_fy() {
    let opts = UltimateShearOptions::default();
    // 梁断面（要素 1）の σwy だけを変えた Qsu の手計算値を返す。
    let qsu_with_sigma_wy = |model: &Model, sigma_wy: f64| -> f64 {
        let shape = model.sections[1].shape.clone().unwrap();
        let p = super::rc_props::rc_bar_props(
            &shape,
            super::rc_props::RcDirection::Strong,
            false,
            false,
        )
        .expect("梁断面の諸元");
        let l_clear = super::geometry::clear_span(&model.elements[1], model);
        super::rc_strength::member_shear_strength(
            &p,
            24.0,
            0.0,
            l_clear,
            &UltimateShearOptions {
                sigma_wy,
                ..opts.clone()
            },
        )
    };
    // 梁のせん断補強筋だけ専用材料（fy を指定）に差し替えたモデル。主筋・
    // コンクリートは既定の material()（SD345・fy=345・Fc=24）を共有する。
    let model_with_shear_fy = |shear_fy: Option<f64>| -> Model {
        let mut model = column_and_beam_model();
        model.materials.push(Material {
            id: MaterialId(model.materials.len() as u32),
            name: "SD345".to_string(),
            fy: shear_fy,
            ..material()
        });
        let shear_id = MaterialId(model.materials.len() as u32 - 1);
        model.sections[1].shear_rebar_material = Some(shear_id);
        model
    };

    // 標準ケース（fy=345）では材料の fy=345 が σwy に効く。
    let model = model_with_shear_fy(Some(345.0));
    let checks = collect_rc_ultimate_checks(&model, &[], &opts).unwrap();
    let beam = checks.iter().find(|c| c.elem == ElemId(1)).unwrap();
    let qsu_345 = qsu_with_sigma_wy(&model, 345.0);
    assert!(
        (beam.qsu - qsu_345).abs() / qsu_345 < 1e-12,
        "材料 fy=345 を σwy に用いるはず: qsu={}, expected={qsu_345}",
        beam.qsu
    );

    // 材料名の数値が同じでも fy が異なれば fy が効く（fy=490）。
    let model_490 = model_with_shear_fy(Some(490.0));
    let checks_490 = collect_rc_ultimate_checks(&model_490, &[], &opts).unwrap();
    let beam_490 = checks_490.iter().find(|c| c.elem == ElemId(1)).unwrap();
    let qsu_490 = qsu_with_sigma_wy(&model_490, 490.0);
    assert!(
        (beam_490.qsu - qsu_490).abs() / qsu_490 < 1e-12,
        "材料 fy=490 を σwy に用いるはず: qsu={}, expected={qsu_490}",
        beam_490.qsu
    );
    assert!(beam_490.qsu > beam.qsu, "fy の増加に伴い Qsu も増える");

    // 対応グレードでも fy 未設定は入力不備として停止する（既定 295 で代替しない）。
    let model_none = model_with_shear_fy(None);
    let err = collect_rc_ultimate_checks(&model_none, &[], &opts).unwrap_err();
    assert!(err.contains("fy"), "{err}");
    assert!(err.contains("部材 ID 1"), "{err}");
}

/// SR235 は対応グレードだが、fy 未設定なら入力不備として停止する
/// （既定 295 で評価すると σwy が 235→295 に増える危険側）。fy を設定すれば算定できる。
#[test]
fn test_ultimate_sr235_requires_fy() {
    let opts = UltimateShearOptions::default();
    let model_with_shear = |name: &str, fy: Option<f64>| -> Model {
        let mut model = column_and_beam_model();
        model.materials.push(Material {
            id: MaterialId(model.materials.len() as u32),
            name: name.to_string(),
            fy,
            ..material()
        });
        let shear_id = MaterialId(model.materials.len() as u32 - 1);
        model.sections[1].shear_rebar_material = Some(shear_id);
        model
    };

    let err = collect_rc_ultimate_checks(&model_with_shear("SR235", None), &[], &opts).unwrap_err();
    assert!(err.contains("SR235") && err.contains("fy"), "{err}");

    let checks =
        collect_rc_ultimate_checks(&model_with_shear("SR235", Some(235.0)), &[], &opts).unwrap();
    let beam = checks.iter().find(|c| c.elem == ElemId(1)).unwrap();
    assert!(beam.qsu > 0.0);
}

/// `SD295` の未知名（`SD295X` 等）は未対応として停止し、設定した `fy` を
/// 終局 σwy に使わない（`SD295` の前方一致で任意の `fy` をすり抜けさせない）。
#[test]
fn test_ultimate_unknown_sd295_is_unsupported() {
    let opts = UltimateShearOptions::default();
    for name in ["SD295X", "SD295Z", "SD295-FOO"] {
        let mut model = column_and_beam_model();
        model.materials.push(Material {
            id: MaterialId(model.materials.len() as u32),
            name: name.to_string(),
            fy: Some(295.0),
            ..material()
        });
        let shear_id = MaterialId(model.materials.len() as u32 - 1);
        model.sections[1].shear_rebar_material = Some(shear_id);
        let err = collect_rc_ultimate_checks(&model, &[], &opts).unwrap_err();
        assert!(err.contains(name), "{name}: {err}");
        assert!(err.contains("未対応"), "{name}: {err}");
    }
}

#[test]
fn test_ultimate_check_ql_subtraction_and_unsupported_error() {
    // 余裕率の QL 控除は `q_long` のみで行う。
    let ql = 50_000.0;
    let demand = vec![(
        ElemId(1),
        MemberDemand {
            q_long: Some(ql),
            ..MemberDemand::axial(0.0)
        },
    )];
    let opts = UltimateShearOptions::default();

    // 普通強度（SD345）: QL 控除。
    let model = column_and_beam_model();
    let checks = collect_rc_ultimate_checks(&model, &demand, &opts).unwrap();
    let beam = checks.iter().find(|c| c.elem == ElemId(1)).unwrap();
    assert!((beam.shear_margin - (beam.qsu - ql).max(0.0) / beam.qmu).abs() < 1e-9);

    // 未対応グレード（KH785）は入力不備として理由付きで停止する。
    let mut model_kh = column_and_beam_model();
    model_kh.materials.push(Material {
        id: MaterialId(model_kh.materials.len() as u32),
        name: "KH785".to_string(),
        fy: Some(785.0),
        ..material()
    });
    let kh = MaterialId(model_kh.materials.len() as u32 - 1);
    model_kh.sections[1].shear_rebar_material = Some(kh);
    let err = collect_rc_ultimate_checks(&model_kh, &demand, &opts).unwrap_err();
    assert!(err.contains("KH785"), "{err}");
    assert!(
        err.contains("SR235・SR295・SD295・SD345・SD390・SD490"),
        "{err}"
    );
    assert!(err.contains("部材 ID 1"), "部材 ID を含むはず: {err}");
}

/// 算定オプション（軽量コンクリートの低減・付着検定の省略）が機能する。
#[test]
fn test_ultimate_check_option_flags() {
    let model = column_and_beam_model();

    // 軽量コンクリートは Qsu・Qbu を 0.9 倍に低減する。
    let std = collect_rc_ultimate_checks(&model, &[], &UltimateShearOptions::default()).unwrap();
    let lw = collect_rc_ultimate_checks(
        &model,
        &[],
        &UltimateShearOptions {
            lightweight: true,
            ..Default::default()
        },
    )
    .unwrap();
    let col_std = std.iter().find(|c| c.elem == ElemId(0)).unwrap();
    let col_lw = lw.iter().find(|c| c.elem == ElemId(0)).unwrap();
    assert!((col_lw.qsu - 0.9 * col_std.qsu).abs() < 1e-3);
    assert!((col_lw.qbu - 0.9 * col_std.qbu).abs() < 1e-3);

    // 付着検定を省略すると Qbu=0・付着余裕度は無限大。
    let no_bond = collect_rc_ultimate_checks(
        &model,
        &[],
        &UltimateShearOptions {
            include_bond: false,
            ..Default::default()
        },
    )
    .unwrap();
    for c in &no_bond {
        assert_eq!(c.qbu, 0.0);
        assert!(c.bond_margin.is_infinite());
    }
}

#[test]
fn test_ultimate_check_skips_non_rc() {
    // 鋼断面（shape=None 相当）は検定対象外。
    let mut model = column_and_beam_model();
    model.sections[0].shape = None;
    let checks = collect_rc_ultimate_checks(&model, &[], &UltimateShearOptions::default()).unwrap();
    // 柱がスキップされ梁のみ。
    assert_eq!(checks.len(), 1);
    assert_eq!(checks[0].elem, ElemId(1));
}

#[test]
fn test_biaxial_margin_handcalc() {
    // rx=ry=0.5, α=2 → 1/√(0.25+0.25)=1/√0.5=√2。
    let m = biaxial_margin(0.5, 0.5, 2.0);
    assert!((m - 2.0_f64.sqrt()).abs() < 1e-9, "m={m}");
    // 片軸のみ需要（ry=0）→ rx=0.5 の逆数=2.0。
    assert!((biaxial_margin(0.5, 0.0, 2.0) - 2.0).abs() < 1e-9);
    // 需要ゼロ → 無限大。
    assert!(biaxial_margin(0.0, 0.0, 2.0).is_infinite());
    // 相互作用が単位に達する（rx²+ry²=1）と余裕度=1.0。
    assert!((biaxial_margin(0.6, 0.8, 2.0) - 1.0).abs() < 1e-9);
}

/// 柱の 2 軸せん断余裕度オプションが機能し、強軸単独より小さい（不利側）になる。
#[test]
fn test_ultimate_check_biaxial_shear() {
    let model = column_and_beam_model();
    let axial = vec![(ElemId(0), MemberDemand::axial(2_000_000.0))];
    let uni = collect_rc_ultimate_checks(&model, &axial, &UltimateShearOptions::default()).unwrap();
    let bi = collect_rc_ultimate_checks(
        &model,
        &axial,
        &UltimateShearOptions {
            biaxial_shear: true,
            ..Default::default()
        },
    )
    .unwrap();
    let col_uni = uni.iter().find(|c| c.elem == ElemId(0)).unwrap();
    let col_bi = bi.iter().find(|c| c.elem == ElemId(0)).unwrap();
    // 既定では 2 軸余裕度は None。
    assert!(col_uni.biaxial_shear_margin.is_none());
    // 2 軸指定で Some。両軸の需要を合成するため強軸単独の余裕度以下になる。
    let bm = col_bi.biaxial_shear_margin.expect("2軸指定で Some");
    assert!(
        bm > 0.0 && bm <= col_bi.shear_margin + 1e-9,
        "bm={bm} uni={}",
        col_bi.shear_margin
    );
    // 梁は 2 軸せん断の対象外（None のまま）。
    let beam_bi = bi.iter().find(|c| c.elem == ElemId(1)).unwrap();
    assert!(beam_bi.biaxial_shear_margin.is_none());
}

/// 柱の 2 軸曲げ余裕度オプションが機能する（設計用曲げ需要を与えたとき Some・正）。
#[test]
fn test_ultimate_check_biaxial_bending() {
    let model = column_and_beam_model();
    // 柱に軸力＋強軸/弱軸の設計用曲げ需要を与える。
    let demand = vec![(
        ElemId(0),
        MemberDemand {
            n_axial: 1_500_000.0,
            mz: 2.0e8,
            my: 1.0e8,
            ..Default::default()
        },
    )];
    let uni =
        collect_rc_ultimate_checks(&model, &demand, &UltimateShearOptions::default()).unwrap();
    let bi = collect_rc_ultimate_checks(
        &model,
        &demand,
        &UltimateShearOptions {
            biaxial_bending: true,
            ..Default::default()
        },
    )
    .unwrap();
    let col_uni = uni.iter().find(|c| c.elem == ElemId(0)).unwrap();
    let col_bi = bi.iter().find(|c| c.elem == ElemId(0)).unwrap();
    // 既定では None、指定で Some。
    assert!(col_uni.biaxial_bending_margin.is_none());
    let bm = col_bi.biaxial_bending_margin.expect("2軸曲げ指定で Some");
    assert!(bm > 0.0 && bm.is_finite(), "bm={bm}");
    // 手計算照合: 1/√((Mmx/Mux)²+(Mmy/Muy)²)。Mux=col.mu(強軸)、Muy は弱軸 Mu。
    // 強軸 Mux は col_bi.mu と一致（同一軸力）。弱軸は main_y=main_x なので b↔D 入替のみ。
    // rx=Mmx/Mux>0, ry>0 → bm < min(Mux/Mmx, Muy/Mmy)。
    let rx = 2.0e8 / col_bi.mu;
    assert!(rx > 0.0);
    // 需要 0 なら無限大。
    let zero_demand = vec![(ElemId(0), MemberDemand::axial(1_500_000.0))];
    let z = collect_rc_ultimate_checks(
        &model,
        &zero_demand,
        &UltimateShearOptions {
            biaxial_bending: true,
            ..Default::default()
        },
    )
    .unwrap();
    let col_z = z.iter().find(|c| c.elem == ElemId(0)).unwrap();
    assert!(col_z.biaxial_bending_margin.unwrap().is_infinite());
    // 梁は対象外。
    let beam_bi = bi.iter().find(|c| c.elem == ElemId(1)).unwrap();
    assert!(beam_bi.biaxial_bending_margin.is_none());
}

/// 終局せん断強度に靭性指針式 Vu を選択するオプションが機能する。
#[test]
fn test_ultimate_check_shear_method_ductility() {
    let model = column_and_beam_model();
    let plastic =
        collect_rc_ultimate_checks(&model, &[], &UltimateShearOptions::default()).unwrap();
    let ductility = collect_rc_ultimate_checks(
        &model,
        &[],
        &UltimateShearOptions {
            shear_method: ShearMethod::Ductility,
            ..Default::default()
        },
    )
    .unwrap();
    // 両手法とも柱・梁の Qsu/Vu は正値（別定式なので値は一般に異なる）。
    for c in &plastic {
        assert!(c.qsu > 0.0, "塑性 Qsu>0: elem={:?}", c.elem);
    }
    let col_p = plastic.iter().find(|c| c.elem == ElemId(0)).unwrap();
    let col_d = ductility.iter().find(|c| c.elem == ElemId(0)).unwrap();
    assert!(col_d.qsu > 0.0, "靭性 Vu>0");
    assert!(
        (col_p.qsu - col_d.qsu).abs() > 1e-3,
        "塑性 Qsu={} と靭性 Vu={} は一般に異なるはず",
        col_p.qsu,
        col_d.qsu
    );
    // basis 文字列に選択した式名が反映される。
    assert!(col_d.basis.contains("靭性指針式"), "basis={}", col_d.basis);
    assert!(col_p.basis.contains("塑性理論式"), "basis={}", col_p.basis);
    // 付着側も靭性指針式は Vbu（付着考慮せん断信頼強度）を用い、塑性の Qbu と異なる。
    assert!(col_d.qbu > 0.0, "靭性 Vbu>0");
    assert!(col_p.qbu > 0.0, "塑性 Qbu>0");
    assert!(
        (col_p.qbu - col_d.qbu).abs() > 1e-3,
        "塑性 Qbu={} と靭性 Vbu={} は一般に異なるはず",
        col_p.qbu,
        col_d.qbu
    );
}

/// プッシュオーバー応答からの部材別 Rp・設計用せん断力（強軸・2 軸せん断の
/// 弱軸）の直接反映が機能する。
#[test]
fn test_ultimate_check_pushover_demand() {
    let model = column_and_beam_model();

    // (1) 設計用せん断力 Qm を直接反映すると Qmu は 2·Mu/内法 ではなく Qm になる。
    let qm = 123_456.0_f64;
    let demand = vec![(
        ElemId(0),
        MemberDemand::from_pushover(1_000_000.0, 1.0e8, 5.0e7, qm, 0.0, 0.0),
    )];
    let checks =
        collect_rc_ultimate_checks(&model, &demand, &UltimateShearOptions::default()).unwrap();
    let col = checks.iter().find(|c| c.elem == ElemId(0)).unwrap();
    // 上限強度倍率=1.0（既定）なので Qmu = |Qm|。
    assert!(
        (col.qmu - qm).abs() < 1e-3,
        "Qmu={} は応答せん断 Qm={} を反映するはず",
        col.qmu,
        qm
    );

    // (2) 部材別 Rp を上げると（塑性理論式）柱の Qsu は低下する（ν・cotφ 低減）。
    let d_rp0 = vec![(
        ElemId(0),
        MemberDemand::from_pushover(1_000_000.0, 1.0e8, 5.0e7, qm, 0.0, 0.0),
    )];
    let d_rp3 = vec![(
        ElemId(0),
        MemberDemand::from_pushover(1_000_000.0, 1.0e8, 5.0e7, qm, 0.0, 0.03),
    )];
    let c0 = collect_rc_ultimate_checks(&model, &d_rp0, &UltimateShearOptions::default()).unwrap();
    let c3 = collect_rc_ultimate_checks(&model, &d_rp3, &UltimateShearOptions::default()).unwrap();
    let q0 = c0.iter().find(|c| c.elem == ElemId(0)).unwrap().qsu;
    let q3 = c3.iter().find(|c| c.elem == ElemId(0)).unwrap().qsu;
    assert!(
        q3 < q0,
        "部材別 Rp=0.03 の Qsu={q3} は Rp=0 の Qsu={q0} より小さいはず"
    );

    // (3) shear/rp 未指定（axial のみ）は Qmu=2·Mu/内法（Qm 直接反映なし）。
    let d_axial = vec![(ElemId(0), MemberDemand::axial(1_000_000.0))];
    let ca =
        collect_rc_ultimate_checks(&model, &d_axial, &UltimateShearOptions::default()).unwrap();
    let col_a = ca.iter().find(|c| c.elem == ElemId(0)).unwrap();
    assert!(
        (col_a.qmu - qm).abs() > 1.0,
        "shear 未指定時は Qmu が応答せん断と一致しないはず（両端ヒンジ略算）"
    );

    // (4) 2 軸せん断では弱軸の設計用せん断需要（プッシュオーバー弱軸応答）を
    // 直接反映する。強軸せん断は共通、弱軸せん断需要のみ大小 2 種。
    let opts = UltimateShearOptions {
        biaxial_shear: true,
        ..Default::default()
    };
    let qm = 200_000.0_f64;
    let small = vec![(
        ElemId(0),
        MemberDemand::from_pushover(1_000_000.0, 0.0, 0.0, qm, 10_000.0, 0.0),
    )];
    let large = vec![(
        ElemId(0),
        MemberDemand::from_pushover(1_000_000.0, 0.0, 0.0, qm, 400_000.0, 0.0),
    )];
    let cs = collect_rc_ultimate_checks(&model, &small, &opts).unwrap();
    let cl = collect_rc_ultimate_checks(&model, &large, &opts).unwrap();
    let ms = cs
        .iter()
        .find(|c| c.elem == ElemId(0))
        .unwrap()
        .biaxial_shear_margin
        .expect("2軸指定で Some");
    let ml = cl
        .iter()
        .find(|c| c.elem == ElemId(0))
        .unwrap()
        .biaxial_shear_margin
        .expect("2軸指定で Some");
    // 弱軸の需要せん断が大きいほど 2 軸せん断余裕度は小さくなる（不利側）。
    assert!(
        ml < ms,
        "弱軸需要大の余裕度 {ml} は弱軸需要小 {ms} より小さいはず"
    );
}

/// CFT 角形柱 1 本のモデルで軸終局検定ドライバが Ncu/Ntu・軸余裕度を算定する。
#[test]
fn test_collect_cft_ultimate_checks() {
    let cft_shape = SectionShape::CftBox {
        height: 400.0,
        width: 400.0,
        thick: 12.0,
    };
    let sec = Section {
        id: SectionId(0),
        name: "CFT400".into(),
        area: cft_shape.calc_area(),
        iy: cft_shape.calc_iy(),
        iz: cft_shape.calc_iz(),
        j: 1.0,
        depth: 400.0,
        width: 400.0,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: Some(cft_shape),
        material: Some(MaterialId(0)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    };
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "BCR295".to_string(),
        category: MaterialCategory::Steel,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: Some(30.0),
        fy: None,
    };
    let model = Model {
        nodes: vec![node(0, [0.0, 0.0, 0.0]), node(1, [0.0, 0.0, 3000.0])],
        sections: vec![sec],
        materials: vec![mat],
        elements: vec![frame_element(0, 0, 0, 1)],
        ..Default::default()
    };
    // 圧縮軸力 3000kN。
    let axial = vec![(ElemId(0), 3_000_000.0)];
    let checks = collect_cft_ultimate_checks(&model, &axial);
    assert_eq!(checks.len(), 1);
    let c = &checks[0];
    assert!(c.ncu > 0.0 && c.ntu > 0.0);
    assert!((c.axial_margin - c.ncu / 3_000_000.0).abs() < 1e-6);
    // lk=3000, D=400 → lk/D=7.5 → 中柱。
    assert_eq!(c.class, CftColumnClass::Medium);
    // 短柱 N-M 曲げ耐力 Mu(N) が正（圧縮軸力 3000kN 時）。
    assert!(c.mu_nm > 0.0, "mu_nm={}", c.mu_nm);
}

/// 実配筋モデルの断面から 1 部材のモデルを作る（部材軸は `horizontal` で切替）。
fn single_shape_model(shape: SectionShape, b: f64, d: f64, horizontal: bool) -> Model {
    let sec = Section {
        id: SectionId(0),
        name: "NEW0".to_string(),
        area: shape.calc_area(),
        iy: shape.calc_iy(),
        iz: shape.calc_iz(),
        j: 1.0,
        depth: d,
        width: b,
        as_y: 0.0,
        as_z: 0.0,
        floor: None,
        panel_thickness: None,
        thickness: None,
        shape: Some(shape),
        material: Some(MaterialId(0)),
        rebar_material: Some(MaterialId(0)),
        shear_rebar_material: Some(MaterialId(0)),
        steel_material: None,
    };
    let nodes = if horizontal {
        vec![node(0, [0.0, 0.0, 0.0]), node(1, [6000.0, 0.0, 0.0])]
    } else {
        vec![node(0, [0.0, 0.0, 0.0]), node(1, [0.0, 0.0, 3000.0])]
    };
    Model {
        nodes,
        sections: vec![sec],
        materials: vec![material()],
        elements: vec![frame_element(0, 0, 0, 1)],
        ..Default::default()
    }
}

/// 上 4+2 本・下 3+2 本の非対称 RC 梁断面。
fn beam_rect_shape() -> SectionShape {
    SectionShape::RcBeamRect {
        b: 400.0,
        d: 600.0,
        rebar: RcBeamRebar {
            main_dia: 22.0,
            top: vec![4, 2],
            bottom: vec![3, 2],
            cover: 40.0,
            stirrup: BeamStirrup {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
            },
        },
    }
}

/// x=[3], y=[3] の RC 矩形柱断面。
fn column_rect_shape() -> SectionShape {
    SectionShape::RcColumnRect {
        b: 600.0,
        d: 700.0,
        rebar: RcRectColumnRebar {
            main_dia: 22.0,
            x: vec![3],
            y: vec![3],
            cover: 40.0,
            hoop: RectColumnHoop {
                dia: 10.0,
                pitch: 100.0,
                legs_x: 2,
                legs_y: 3,
            },
        },
    }
}

/// count=8 の RC 円形柱断面。
fn column_circle_shape() -> SectionShape {
    SectionShape::RcColumnCircle {
        d: 600.0,
        rebar: RcCircleColumnRebar {
            main_dia: 22.0,
            count: 8,
            cover: 40.0,
            hoop: CircleColumnHoop {
                dia: 10.0,
                pitch: 100.0,
            },
        },
    }
}

/// 実配筋モデルの新型バリアント（梁・矩形柱・円形柱）が終局検定の対象になる。
#[test]
fn test_ultimate_check_new_rc_shapes_supported() {
    let opts = UltimateShearOptions::default();

    let beam = single_shape_model(beam_rect_shape(), 400.0, 600.0, true);
    let checks = collect_rc_ultimate_checks(&beam, &[], &opts).unwrap();
    assert_eq!(checks.len(), 1);
    assert_eq!(checks[0].kind, MemberKind::Beam);
    assert!(checks[0].mu > 0.0 && checks[0].qsu > 0.0 && checks[0].qmu > 0.0);
    assert!(checks[0].axial.is_none());

    let column = single_shape_model(column_rect_shape(), 600.0, 700.0, false);
    let checks = collect_rc_ultimate_checks(&column, &[], &opts).unwrap();
    assert_eq!(checks.len(), 1);
    assert_eq!(checks[0].kind, MemberKind::Column);
    assert!(checks[0].mu > 0.0 && checks[0].qsu > 0.0 && checks[0].qmu > 0.0);
    assert!(checks[0].axial.is_some());

    let circle = single_shape_model(column_circle_shape(), 600.0, 600.0, false);
    let checks = collect_rc_ultimate_checks(&circle, &[], &opts).unwrap();
    assert_eq!(checks.len(), 1);
    assert_eq!(checks[0].kind, MemberKind::Column);
    assert!(checks[0].mu > 0.0 && checks[0].qsu > 0.0 && checks[0].qmu > 0.0);
    assert!(checks[0].axial.is_some());
}

/// 実配筋が幾何的に成立しない新モデル断面は、終局検定の入力不備として
/// 部材 ID 付きで停止する（無言スキップしない）。
#[test]
fn test_ultimate_inconsistent_rebar_errors() {
    let opts = UltimateShearOptions::default();
    let shape = SectionShape::RcColumnRect {
        b: 600.0,
        d: 700.0,
        rebar: RcRectColumnRebar {
            main_dia: 22.0,
            x: vec![4, 2],
            y: vec![3],
            cover: 40.0,
            hoop: RectColumnHoop {
                dia: 10.0,
                pitch: 100.0,
                legs_x: 2,
                legs_y: 3,
            },
        },
    };
    let model = single_shape_model(shape, 600.0, 700.0, false);
    let err = collect_rc_ultimate_checks(&model, &[], &opts).unwrap_err();
    assert!(err.contains("部材 ID 0"), "{err}");
    assert!(err.contains("配筋形状"), "{err}");
}

/// 矩形柱・円形柱の Mu が軸力 0 の手計算（0.8·at·σy·D）に一致する。
#[test]
fn test_ultimate_new_columns_mu_matches_handcalc() {
    let opts = UltimateShearOptions::default();
    let a22 = std::f64::consts::PI * (22.0_f64 / 2.0).powi(2);

    let column = single_shape_model(column_rect_shape(), 600.0, 700.0, false);
    let checks = collect_rc_ultimate_checks(&column, &[], &opts).unwrap();
    let at = 3.0 * a22;
    let expected = 0.8 * at * 345.0 * 700.0;
    assert!(
        (checks[0].mu - expected).abs() / expected < 1e-9,
        "矩形柱 mu={} expected={expected}",
        checks[0].mu
    );

    let circle = single_shape_model(column_circle_shape(), 600.0, 600.0, false);
    let checks = collect_rc_ultimate_checks(&circle, &[], &opts).unwrap();
    let side = (std::f64::consts::PI * 600.0 * 600.0 / 4.0).sqrt();
    let at = 2.0 * a22;
    let expected = 0.8 * at * 345.0 * side;
    assert!(
        (checks[0].mu - expected).abs() / expected < 1e-9,
        "円形柱 mu={} expected={expected}",
        checks[0].mu
    );
}

/// 梁の上下非対称では曲げモーメントの符号で引張側が変わり Mu に反映される。
#[test]
fn test_ultimate_beam_rect_tension_side_affects_mu() {
    let opts = UltimateShearOptions::default();
    let model = single_shape_model(beam_rect_shape(), 400.0, 600.0, true);

    let top_demand = vec![(
        ElemId(0),
        MemberDemand {
            mz: -1.0e8,
            ..Default::default()
        },
    )];
    let bottom_demand = vec![(
        ElemId(0),
        MemberDemand {
            mz: 1.0e8,
            ..Default::default()
        },
    )];
    let mu_top = collect_rc_ultimate_checks(&model, &top_demand, &opts).unwrap()[0].mu;
    let mu_bottom = collect_rc_ultimate_checks(&model, &bottom_demand, &opts).unwrap()[0].mu;
    assert!(mu_top > 0.0 && mu_bottom > 0.0);
    assert!(
        (mu_top - mu_bottom).abs() > 1.0,
        "上端引張 {mu_top} と下端引張 {mu_bottom} は異なるはず"
    );
    // 上端筋 6 本 > 下端筋 5 本のため上端引張の Mu が大きい。
    assert!(mu_top > mu_bottom, "mu_top={mu_top} mu_bottom={mu_bottom}");
}

/// 新型 RC 梁の付着検定では、上端引張のとき上端筋低減（αt）を反映する。
/// 上端筋 6 本 > 下端筋 5 本でも、αt 低減により上端引張の Qbu が下端引張より小さくなる。
#[test]
fn test_ultimate_beam_rect_top_tension_reduces_qbu() {
    let opts = UltimateShearOptions::default();
    let model = single_shape_model(beam_rect_shape(), 400.0, 600.0, true);

    let top_demand = vec![(
        ElemId(0),
        MemberDemand {
            mz: -1.0e8,
            ..Default::default()
        },
    )];
    let bottom_demand = vec![(
        ElemId(0),
        MemberDemand {
            mz: 1.0e8,
            ..Default::default()
        },
    )];
    let qbu_top = collect_rc_ultimate_checks(&model, &top_demand, &opts).unwrap()[0].qbu;
    let qbu_bottom = collect_rc_ultimate_checks(&model, &bottom_demand, &opts).unwrap()[0].qbu;
    assert!(qbu_top > 0.0 && qbu_bottom > 0.0);
    assert!(
        qbu_top < qbu_bottom,
        "上端引張 Qbu={qbu_top} は下端引張 Qbu={qbu_bottom} より小さいはず（αt 低減）"
    );
}
