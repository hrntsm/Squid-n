//! 壁を含む最小モデルの統合テスト（壁領域再設計 Step 7+8 着手前の安全網）。
//!
//! # 目的
//!
//! `dev_docs/handoff/床領域・壁領域の再設計_申し送り.md` §3.2 E8 で決めたとおり、
//! 実フィクスチャ（`tests/fixtures/model.stb`、`full_model.rs`）には壁要素が 0 件で、
//! 壁関連の機能（`WallAttr` の開口低減・自重、取り付く壁版の自重配分）は
//! いずれも単体テストでしか検証されていない。`Model.floor_regions`（26 件）を
//! 巻き込まない**独立した壁専用フィクスチャ**として、耐震壁 1 パネル＋フレーム外雑壁
//! 1 本を含む最小の立体架構を用意し、`App` の全解析入口を通した代表スカラを
//! ピン止めする。`WallRegion`/`WallAttr`/`WallPlate` の型を作り替える Step 7+8 の
//! 着手時、このテストの差分が「型変更が計算結果を変えていないか」の一次判定になる。
//!
//! # モデル
//!
//! 4m(X)×3m(Y)×3m(Z) の 1 スパン・1 層。柱 4 本（うち柱 0・1〔壁の側柱〕は RC、
//! 柱 2・3 は H-300x300 鋼）・梁 8 本（うち id 4〔頂部 4-5〕・id 8〔柱脚間 0-1、
//! いずれも壁の上下大梁〕は RC、残り 6 本は H-400x200 鋼。柱脚どうし・柱頭どうしの
//! 両方で閉路。id 4-7 が頂部、id 8-11 が柱脚〔基礎大梁想定〕）・耐震壁 1 枚
//! （Y=0 面、RC t=150、開口 1 つ）・フレーム外雑壁 1 本（Y=3000 面の梁上端に沿う想定、
//! height=900・Column 伝達）。柱脚 4 節点は固定支点。荷重は ST-Bridge を経由しないため
//! `App::run_preparation` の自動同期（`sync_auto_load_cases_action`）が
//! DL・LL・EX・EY を生成する。
//!
//! **柱脚どうしの梁（基礎大梁）を持つ。** 2026-08 に新設した当初は柱頭側の梁のみで、
//! `region_gen::wall::scan_wall_region_boundaries` が境界を 1 つも検出できなかった
//! （各鉛直構面が「柱 2 本＋頂部の梁 1 本」の開いた U 字にしかならないため）。
//! 壁側 `region_gen` の実データ検証は実フィクスチャ（`full_model.rs`）の
//! 軸組で代替していたが、`WallRegion`/`WallPlate` の型実装（Step 7+8 本体）は
//! 本フィクスチャでの境界検出を前提にするため、柱脚どうしの梁を追加して 4 面とも
//! 閉じた矩形（4 節点ループ）にした（`test_region_gen_wall_finds_all_four_faces` で確認）。
//!
//! **柱脚間の梁を足したことで、耐震壁の側柱・上下大梁を RC にする必要が生じた。**
//! 壁エレメントは壁と周辺架構を一体の耐震要素としてモデル化するため、耐震壁の
//! 四周（上下大梁・左右側柱）の構造種別が壁自身と一致しない場合はエラーになる
//! （`wall_frame_category_issue`）。柱脚間の梁がなかった当初は壁が「四周を持つ」と
//! 判定されず（`wall_is_framed` が false）、このチェック自体が働いていなかった
//! （＝もともと本フィクスチャは、この検定の観点では気づかれずに不正な混合構造
//! だった）。柱 0・1・大梁 4・8 の材料を RC（`MaterialId(1)`）へ差し替えて是正した。
//!
//! **側柱・壁上下大梁は本物の `SectionShape::RcRect`＋主筋（300x300・300x400、
//! 主筋 3-D22・せん断補強筋 D10@100）を使う。** 以前は「鋼断面（H 形）の断面性能を
//! そのまま流用し材料だけ RC へ差し替える」トリックを使っていたが、これは
//! 実在しない断面（形状は鋼、材料は RC）で、増分解析（プッシュオーバー）・
//! 保有水平耐力の Ds 群回帰テスト（`snapshot_wall_ds_group_and_holding_capacity`）を
//! 追加しようとした際に破綻した（ファイバー断面が `shape` だけで鋼材と判定し
//! 材料の `fy` を要求してエラーになる、側柱の主筋がないと壁の等価引張鉄筋比 pte が
//! 算定できない）。本物の RC 矩形に差し替えたことで、線形の代表スカラ・
//! 固有周期も現実の RC フレームの値に変わった（後述）。
//!
//! 追加した基礎大梁は柱脚（固定支点）どうしをつなぐため、線形静解析・固有値解析の
//! いずれにも力・変位としては現れない（両端固定の部材は固定端間で力を伝達しない
//! ため）。ただし RC への材料差し替え（ヤング係数・密度が鋼と異なる）は側柱・
//! 上下大梁の剛性・自重を実際に変えるため、代表スカラは総じて動く（実測して
//! スナップショットを更新した。値を予想で決め打ちしていない）。
//!
//! **`test_wall_element_changes_eigen_period` は方向を無視した `period[0]` の比較を
//! やめた。** 本物の RC 矩形に差し替えたところ、壁の有無で `period[0]` が指す
//! 方向自体が入れ替わり（壁ありは壁の影響を受けない Y 方向が最も柔らかく
//! `period[0]` を占め、壁なしは壁を失った X 方向が最も柔らかくなって `period[0]`
//! を占める）、無関係な方向どうしを比較して「壁が効いていない」という誤った
//! 結論になっていた。`participation`/`effective_mass` の X 成分が卓越するモードの
//! 周期を比較するよう修正した（`dominant_x_period` 参照。壁は Y=0 面で X 方向に
//! スパンするため、面内せん断で効くのは X 方向のみ）。

use squid_n_app::app::{App, StaticCaseKey};
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, WallPlateId};
use squid_n_core::model::{
    AreaLoad, ElementData, ElementKind, EndCondition, ForceRegime, LoadTransfer, LocalAxis,
    Material, MaterialCategory, Model, Node, RegionAnchor, RigidZone, Section, WallOpening,
    WallPlate, WallPlateShape,
};
use squid_n_core::wall_region_rebuild::rebuild_wall_regions;
use squid_n_section::shape::{BarSet, RcRebar, SectionShape, ShearBar};
use squid_n_solver::statics::analysis::SeismicDir;

/// 有効数字 4 桁の指数表記（`full_model.rs::sig4` と同じ丸め規則。両ファイルの
/// スナップショットを比較できるよう合わせている）。
fn sig4(v: f64) -> String {
    if !v.is_finite() {
        return format!("{v}");
    }
    let v = if v == 0.0 { 0.0 } else { v };
    format!("{v:.3e}")
}

/// 指定した静的結果を取り出す（`full_model.rs::static_of` と同じ規則）。
fn static_res_for(app: &App, key: StaticCaseKey) -> &squid_n_solver::statics::linear::StaticOnce {
    &app.core
        .scoped
        .results
        .as_ref()
        .expect("解析結果が格納されているはず")
        .statics
        .iter()
        .find(|(k, _)| *k == key)
        .expect("指定した荷重ケースの静的結果")
        .1
}

/// 壁 1 パネル＋雑壁 1 本を含む、柱 4 本・梁 4 本の 1 スパン立体架構。
fn wall_bay_model() -> Model {
    let mut model = Model::default();

    // 節点: 柱脚 4（固定）+ 柱頭 4（自由）。
    let base = [
        [0.0, 0.0, 0.0],
        [4000.0, 0.0, 0.0],
        [4000.0, 3000.0, 0.0],
        [0.0, 3000.0, 0.0],
    ];
    let top = [
        [0.0, 0.0, 3000.0],
        [4000.0, 0.0, 3000.0],
        [4000.0, 3000.0, 3000.0],
        [0.0, 3000.0, 3000.0],
    ];
    for (i, c) in base.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    for (i, c) in top.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(4 + i as u32),
            coord: *c,
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
            support_spring: None,
        });
    }

    // 断面: 柱 H-300x300、梁 H-400x200（鋼、SN400B）。
    let col_shape = SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    let beam_shape = SectionShape::SteelH {
        height: 400.0,
        width: 200.0,
        web_thick: 8.0,
        flange_thick: 13.0,
    };
    model
        .sections
        .push(col_shape.to_section(SectionId(0), "柱 H-300x300x10x15".into()));
    model
        .sections
        .push(beam_shape.to_section(SectionId(1), "梁 H-400x200x8x13".into()));
    // 断面: 耐震壁（RC t=150）。
    // `shape` に `SectionShape::RcWall` を持たせる（dig 2026-08-26 Q2=A）。
    // `shape: None` のままだと `compute_holding_capacity` の部材ランク判定
    // （告示「耐力壁の種別」表 WA〜WD）が壁を常にスキップし選択ランクへ
    // フォールバックするため、壁固有の Ds ロジックを一切通らない回帰テストに
    // なってしまう。`ps`（壁筋比）は回帰検出用の一般的な値（0.25%）。
    // area・iy・iz・j・as_y・as_z は壁エレメント自身の 4 節点剛性式では
    // 参照されない（側柱・大梁の断面と違い、壁の面内剛性は `thickness`・`ps`・
    // 節点座標から直接組み立てる。`wall_element.rs` 参照）ため、名目値のまま
    // 変更していない。
    model.sections.push(Section {
        id: SectionId(2),
        name: "耐震壁 t150".into(),
        area: 150.0 * 3000.0,
        iy: 1.0e9,
        iz: 1.0e9,
        j: 1.0e9,
        depth: 3000.0,
        width: 150.0,
        as_y: 1.0e5,
        as_z: 1.0e5,
        floor: None,
        panel_thickness: None,
        thickness: Some(150.0),
        shape: Some(SectionShape::RcWall {
            thickness: 150.0,
            ps: 0.0025,
        }),
        material: Some(MaterialId(1)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    // 断面: 耐震壁の側柱・上下大梁（柱 0・1 → id 3、大梁 4-5・0-1 → id 4）。
    // 壁エレメントは壁と周辺架構を一体の耐震要素としてモデル化するため、
    // 耐震壁の四周（上下大梁・左右側柱）は壁と同じ構造種別（RC）でなければならない
    // （`wall_frame_category_issue`）。柱脚どうしの梁を足して壁面を「四周を持つ」
    // 状態にする以上、四周をすべて RC にそろえる必要がある。
    //
    // **本物の `SectionShape::RcRect`＋主筋（自己矛盾のない断面）を使う。**
    // 以前は「鋼断面（`col_shape`／`beam_shape`）の断面性能をそのまま流用し材料だけ
    // RC へ差し替える」トリックを使っていたが、これは実在しない断面（形状は鋼、
    // 材料は RC）だった。増分解析のファイバー断面は `shape` だけを見て鋼材と
    // 判定するため材料の `fy` を要求してエラーになり、保有水平耐力の壁の等価
    // 引張鉄筋比 pte も側柱の主筋（`RcRect` の `rebar`）がなければ算定できない。
    // どちらも「RC 壁の側柱に鋼断面を使う」という組み合わせ自体が実在しえないことが
    // 原因であり、鋼断面の数値流用は削って本物の RC 矩形（300x300・300x400、
    // 主筋 3-D22・せん断補強筋 D10@100）に差し替えた。
    let side_column_rebar = RcRebar {
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
    };
    let mut side_column_section = SectionShape::RcRect {
        b: 300.0,
        d: 300.0,
        rebar: side_column_rebar.clone(),
    }
    .to_section(SectionId(3), "側柱 RC 300x300".into());
    side_column_section.material = Some(MaterialId(1));
    side_column_section.rebar_material = Some(MaterialId(2));
    side_column_section.shear_rebar_material = Some(MaterialId(2));
    model.sections.push(side_column_section);
    let mut wall_girder_section = SectionShape::RcRect {
        b: 300.0,
        d: 400.0,
        rebar: side_column_rebar,
    }
    .to_section(SectionId(4), "壁上下大梁 RC 300x400".into());
    wall_girder_section.material = Some(MaterialId(1));
    wall_girder_section.rebar_material = Some(MaterialId(2));
    wall_girder_section.shear_rebar_material = Some(MaterialId(2));
    model.sections.push(wall_girder_section);

    // 材料: 鋼 SN400B（柱・梁）、RC Fc24（耐震壁・側柱・壁上下大梁）、
    // 主筋・せん断補強筋 SD345（側柱・壁上下大梁）。
    model.materials.push(Material {
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
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(1),
        name: "Fc24".into(),
        category: MaterialCategory::Concrete,
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(2),
        name: "SD345".into(),
        category: MaterialCategory::Rebar,
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(345.0),
    });
    // 断面: パラペット（取り付く壁版 = 壁版 1）。板厚 120 の RC。
    model.sections.push(Section {
        id: SectionId(5),
        name: "パラペット t120".into(),
        area: 120.0 * 900.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 900.0,
        width: 120.0,
        as_y: 1.0e4,
        as_z: 1.0e4,
        floor: None,
        panel_thickness: None,
        thickness: Some(120.0),
        shape: Some(SectionShape::RcWall {
            thickness: 120.0,
            ps: 0.0025,
        }),
        material: Some(MaterialId(1)),
        rebar_material: None,
        shear_rebar_material: None,
        steel_material: None,
    });
    for sec in model.sections.iter_mut().take(2) {
        sec.material = Some(MaterialId(0));
    }

    let beam = |id: u32, i: u32, j: u32, sec: u32, ref_v: [f64; 3]| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
        section: Some(SectionId(sec)),
        local_axis: LocalAxis { ref_vector: ref_v },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    };
    // 柱 4 本（id 0-3）。柱 0・1（壁の側柱）は RC 断面（id 3）、柱 2・3 は鋼断面（id 0）。
    for i in 0..4u32 {
        let sec = if i < 2 { 3 } else { 0 };
        model.elements.push(beam(i, i, 4 + i, sec, [1.0, 0.0, 0.0]));
    }
    // 梁 4 本（id 4-7。頂部で閉路: 4-5, 5-6, 6-7, 7-4）。id 4（4-5、壁の頂部）だけ
    // RC 断面（id 4）、それ以外（壁に接しない 3 本）は鋼断面（id 1）。
    let beam_pairs = [(4u32, 5u32), (5, 6), (6, 7), (7, 4)];
    for (k, (i, j)) in beam_pairs.iter().enumerate() {
        let sec = if k == 0 { 4 } else { 1 };
        model
            .elements
            .push(beam(4 + k as u32, *i, *j, sec, [0.0, 0.0, 1.0]));
    }
    // 梁 4 本（id 8-11。柱脚どうしで閉路: 0-1, 1-2, 2-3, 3-0。基礎大梁想定）。
    // 柱脚は固定支点どうしのため線形解析の応力・変位には現れないが（§冒頭のモジュール doc
    // 参照）、region_gen::wall が壁側の鉛直構面を閉じた矩形として検出するために要る
    // （柱頭側の梁だけでは各構面が開いた U 字になり、境界を 1 つも検出できない）。
    // id 8（0-1、壁の柱脚間）だけ RC 断面（id 4）、それ以外は鋼断面（id 1）。
    let base_beam_pairs = [(0u32, 1u32), (1, 2), (2, 3), (3, 0)];
    for (k, (i, j)) in base_beam_pairs.iter().enumerate() {
        let sec = if k == 0 { 4 } else { 1 };
        model
            .elements
            .push(beam(8 + k as u32, *i, *j, sec, [0.0, 0.0, 1.0]));
    }

    // 耐震壁 1 枚（Y=0 面: 節点 0,1,5,4 の矩形ループ）。開口あり。
    //
    // 壁の解析要素は入力の正ではなく生成物（D5）のため、ここでは壁版
    // （`WallPlate`）を直接構築し、`rebuild_wall_regions` に壁領域（4 面すべて
    // 検出される。モジュール doc §「柱脚どうしの梁」参照）への帰属を任せる
    // （`region_gen::wall` の面走査が Y=0 面の閉路を検出し、この壁版の重心が
    // その閉路に収まることで帰属が決まる。ノード順・巻き方向は面走査側の
    // 走査結果と一致している必要はない。`match_candidate` は全頂点が同一構面上に
    // あることと局所座標での内包だけを見る）。`squid_n_load::wall_expand::
    // expand_wall_elements` が準備計算・解析入口の直前でこの壁版から
    // `ElementKind::Wall` を生成する。`next_id = 既存要素の最大 ID + 1` で
    // 決定的に採番されるが、値そのものは呼び出し経路に依存する。準備計算
    // （`apply_auto_panel_zones` が仕口パネルを 12〜15 番へ先に生成する経路）を
    // 経た後の解析時点では、壁要素の ID は 12 ではなく 16 になる。
    model.wall_plates.push(WallPlate {
        id: WallPlateId(0),
        shape: WallPlateShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(5), NodeId(4)],
        },
        section: Some(SectionId(2)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings: vec![WallOpening {
            width: 900.0,
            height: 1200.0,
            offset: Some([1550.0, 0.0]),
        }],
        loads: vec![],
        slit: Default::default(),
    });

    // 取り付く壁版 1 枚（Y=3000 面の梁 6-7 に載るパラペット。立ち上がり 900、
    // 荷重は取付き線の両端＝柱頭の節点 6・7 へ集中する）。
    //
    // 自重は断面（板厚 120・RC）から求め、外装の仕上げ 0.3kN/m² を `loads` で
    // 上乗せする。壁版の面荷重を代表スカラのピン止め対象へ入れるためでもある。
    // どのフィクスチャも `loads` を持たないと、仕上げ・増打ちを自重へ算入する
    // 経路が壊れても代表スカラが動かず、静かに落ちる。
    model.wall_plates.push(WallPlate {
        id: WallPlateId(1),
        shape: WallPlateShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(6), NodeId(7)],
                span: [0.0, 1.0],
                transfer: LoadTransfer::Columns,
            },
            extent: Some([900.0, 900.0]),
        },
        section: Some(SectionId(5)),
        opening_area: 0.0,
        opening_weight: 0.0,
        openings: vec![],
        loads: vec![AreaLoad {
            kind: "仕上げ".into(),
            value: 3.0e-4,
        }],
        slit: Default::default(),
    });

    // 壁領域を柱・梁の閉路から検出し、直前に積んだ壁版を帰属させる
    // （ST-Bridge 取り込みが `build_walls` の直後に呼ぶのと同じ経路。
    // §5.10 参照）。
    let report = rebuild_wall_regions(&mut model);
    assert_eq!(report.regions, 4, "1 スパンの 4 鉛直構面すべてが検出される");
    assert_eq!(
        report.wall_plates_assigned, 1,
        "壁版が Y=0 面の壁領域へ帰属する"
    );
    assert_eq!(report.unassigned_wall_plates, 0);

    model
}

fn wall_bay_app() -> App {
    let mut app = App::default();
    app.core.analysis_cfg.threads = 1;
    app.core.model = wall_bay_model();
    // 架構種別（Ds 表の行を選ぶ設定。`App::design_frame`）は既定で SteelFrame
    // のままだと、耐震壁を持つ本フィクスチャでも Ds 計算が鋼構造の表
    // （`ds_steel`）を使ってしまい、RC 耐力壁の Ds 表（`ds_rc`）が一度も
    // 通らない。本フィクスチャは RC 耐震壁付き構造なので明示的に宣言する。
    app.core.design_frame = squid_n_design_jp::secondary::holding_capacity::FrameType::RcWall;
    app
}

#[test]
fn test_wall_bay_model_is_valid() {
    let model = wall_bay_model();
    assert!(model.validate().is_ok(), "{:?}", model.validate());
    assert_eq!(model.nodes.len(), 8);
    // 壁の解析要素は生成物であり `model.elements` には含まれない（D5）。
    // 柱 4 本 + 頂部梁 4 本 + 基礎大梁 4 本 = 12。
    assert_eq!(model.elements.len(), 12);
    assert!(
        model.elements.iter().all(|e| e.kind != ElementKind::Wall),
        "壁要素は準備計算からの生成物であり model.elements には含まれない"
    );
    assert_eq!(model.wall_plates.len(), 2, "耐震壁 1 枚＋パラペット 1 枚");
    assert_eq!(model.wall_regions.len(), 4, "1 スパンの 4 鉛直構面すべて");
}

/// GUI診断（`App::run_diagnostics`）が壁展開モデル（D5・dig Q4）を見ていることの回帰
/// テスト。壁要素は `run_diagnostics` の内部で `expand_wall_elements` により初めて
/// `model.elements` へ現れるため、この展開が壊れる（または元の `self.model` を渡す
/// 実装に後退する）と、`model.elements` に壁要素が 0 件のまま
/// `wall_frame_category_issue`（耐震壁と周辺架構の構造種別食い違い）が一度も
/// 呼ばれず、壁の不備が診断タブから静かに消える（解析実行時に初めて気づく劣化になる。
/// §5.14 dig Q4 参照）。
#[test]
fn test_wall_frame_mismatch_appears_in_gui_diagnostics() {
    let mut model = wall_bay_model();
    // 側柱の断面（id 3、柱 0・1 が共有）の材料を鋼へ差し替え、耐震壁（RC）との
    // 構造種別を意図的に食い違わせる。
    model.sections[3].material = Some(MaterialId(0));

    let mut app = App::default();
    app.core.analysis_cfg.threads = 1;
    app.core.model = model;
    app.run_diagnostics();

    assert!(
        app.core
            .scoped
            .diagnostics
            .iter()
            .any(|d| d.message.contains("耐震壁") && d.message.contains("構造種別")),
        "壁展開モデルを経由しないと、model.elements の壁要素が 0 件のため \
         この構造種別食い違いを検知できないはず: {:?}",
        app.core
            .scoped
            .diagnostics
            .iter()
            .map(|d| &d.message)
            .collect::<Vec<_>>()
    );
}

/// `App::run_design_check` が壁展開モデルを見ていることの回帰テスト（申し送り
/// §5.15）。`run_member_design_checks`（内部の `joint_wiring::wall::check_walls`）
/// は `results.member_forces` の壁 `ElemId` を `model.element` で引き直すため、
/// `self.model`（壁展開前）をそのまま渡すと該当 `ElemId` が見つからず
/// `continue` し、耐震壁のせん断断面検定が常にスキップされる。
#[test]
fn test_wall_shear_check_appears_after_run_design_check() {
    let mut app = wall_bay_app();
    app.run_preparation();
    assert!(
        app.core.scoped.last_error.is_none(),
        "準備計算: {:?}",
        app.core.scoped.last_error
    );
    app.run_static_all();
    assert!(
        app.core.scoped.last_error.is_none(),
        "静的解析: {:?}",
        app.core.scoped.last_error
    );
    app.run_design_check();

    let results = app
        .core
        .scoped
        .results
        .as_ref()
        .expect("解析結果が格納されているはず");
    assert!(
        results
            .joint_checks
            .iter()
            .any(|jc| jc.label.contains("耐震壁")),
        "壁展開モデルを経由しないと、model.elements の壁要素が 0 件のため \
         耐震壁のせん断断面検定（check_walls）が一度も実行されないはず: {:?}",
        results
            .joint_checks
            .iter()
            .map(|jc| &jc.label)
            .collect::<Vec<_>>()
    );
}

#[test]
fn test_wall_bay_model_runs_full_pipeline() {
    let mut app = wall_bay_app();
    app.run_preparation();
    assert!(
        app.core
            .scoped
            .last_error
            .as_deref()
            .unwrap_or("")
            .is_empty()
            || app
                .core
                .scoped
                .last_error
                .as_deref()
                .unwrap_or("")
                .starts_with('⚠'),
        "準備計算でエラー: {:?}",
        app.core.scoped.last_error
    );
    app.core.scoped.last_error = None;

    app.run_static_all();
    assert!(
        app.core.scoped.last_error.is_none(),
        "静的解析でエラー: {:?}",
        app.core.scoped.last_error
    );

    app.run_eigen(app.core.analysis_cfg.n_modes);
    assert!(
        app.core.scoped.last_error.is_none(),
        "固有値解析でエラー: {:?}",
        app.core.scoped.last_error
    );

    assert!(
        app.core.scoped.results.is_some(),
        "解析結果が格納されているはず"
    );
    assert!(
        app.core
            .scoped
            .preparation
            .as_ref()
            .unwrap()
            .summary
            .total_seismic_weight
            > 0.0,
        "地震用重量が正であること（壁・雑壁の自重を含む）"
    );
}

/// 代表スカラのスナップショット（有効数字 4 桁）。
///
/// `full_model.rs::snapshot_key_scalars` と同じ趣旨。壁関連の型（`WallRegion`・
/// `WallAttr`・`WallPlate`）を作り替える際、この値の変化を V&V の対象とすること。
#[test]
fn snapshot_wall_bay_scalars() {
    let mut app = wall_bay_app();
    app.run_preparation();
    app.core.scoped.last_error = None;
    app.run_static_all();
    app.run_eigen(app.core.analysis_cfg.n_modes);

    let mut out = String::new();
    let mut line = |k: &str, v: String| {
        out.push_str(k);
        out.push_str(" = ");
        out.push_str(&v);
        out.push('\n');
    };

    let prep = app
        .core
        .scoped
        .preparation
        .as_ref()
        .expect("準備計算の結果");
    line(
        "prep.total_seismic_weight",
        sig4(prep.summary.total_seismic_weight),
    );

    let modal = app
        .core
        .scoped
        .results
        .as_ref()
        .expect("解析結果")
        .modal
        .as_ref()
        .expect("固有値");
    for (i, t) in modal.period.iter().enumerate() {
        line(&format!("eigen.T[{i}]"), sig4(*t));
    }

    // 耐震壁要素の材端力。`member_forces` は `Vec<(ElemId, MemberForces)>` で
    // 位置は保証されない。壁の解析要素は準備計算からの生成物（D5）で、その
    // `ElemId` は `apply_auto_panel_zones`（仕口パネル要素の生成）が先に走った
    // 後の要素数に依存し決め打ちできないため、`expand_wall_elements` を
    // `app.core.model`（`run_preparation` 済み。パネルゾーンまで生成済みで、それ以降
    // `run_static_all` まで `model.elements` は変わらない）へ実際に適用して
    // 生成 `ElemId` を求める（`run_static_all` が内部で行う展開と同じ入力・
    // 同じ決定的な採番規則なので同じ ID になる）。
    let (_expanded_for_id, wall_index, wall_expand_report) =
        squid_n_load::wall_expand::expand_wall_elements(&app.core.model);
    assert_eq!(wall_expand_report.generated, 1, "壁要素が1件生成されるはず");
    let wall_elem_id = wall_index
        .generated_elem_ids()
        .next()
        .expect("生成された壁要素の ElemId");
    let wall_member_forces = |res: &squid_n_solver::statics::linear::StaticOnce| {
        res.member_forces
            .iter()
            .find(|(id, _)| *id == wall_elem_id)
            .expect("耐震壁要素の材端力")
            .1
            .at
            .clone()
    };

    // DL（鉛直荷重）は対称な架構では壁と柱の負担差が小さく、この壁単体では
    // 材端力がほぼゼロになる（剛性比の高い壁柱換算の軸力・曲げが小さいだけで、
    // 壁自体は剛性に確実に効いている。固有周期が壁の有無で 0.0797s→0.0522s と
    // 動くことは `test_wall_element_changes_eigen_period` で固定済み）。
    // **この 2 行は値の大きさではなく `member_forces` から壁要素が消えていないこと
    // 自体を監視する行**（Step 8 の参照張り替えで壁が結果から欠落する回帰を、
    // 上の `.expect()` パニックとして検知する）。壁の面内せん断・開口低減を
    // 実際に動かす荷重は地震（EX）のため、変化を追う代表値は EX 側に置く。
    let dl_key = app
        .core
        .model
        .load_cases
        .iter()
        .find(|c| c.kind == squid_n_core::model::LoadCaseKind::Dead)
        .map(|c| c.id)
        .expect("DL ケース");
    let dl_res = static_res_for(&app, StaticCaseKey::User(dl_key));
    let wall_forces = wall_member_forces(dl_res);
    line("static.DL.wall_forces_len", wall_forces.len().to_string());
    let dl_force_sum: f64 = wall_forces
        .iter()
        .flat_map(|(_, f)| f.iter())
        .map(|v| v.abs())
        .sum();
    line("static.DL.wall_forces_abs_sum", sig4(dl_force_sum));

    let ex_res = static_res_for(&app, StaticCaseKey::Seismic(SeismicDir::X));
    let ex_wall_forces = wall_member_forces(ex_res);
    let ex_force_sum: f64 = ex_wall_forces
        .iter()
        .flat_map(|(_, f)| f.iter())
        .map(|v| v.abs())
        .sum();
    line("static.EX.wall_forces_abs_sum", sig4(ex_force_sum));
    // 柱頭（節点 4、耐震壁側の柱）の水平変位。壁が面内せん断で剛性を
    // 持つ限り、壁なしのフレームより小さくなる（回帰時の一次判定に使える）。
    line("static.EX.top_disp_x[node4]", sig4(ex_res.disp[4][0]));

    insta::assert_snapshot!(out);
}

/// 固有値解析結果のうち、X 方向（`participation`/`effective_mass` の第 0 成分）が
/// 卓越するモードの周期を返す（存在しなければ `None`）。
///
/// `ModalResult::period[0]` を単純比較してはいけない。**モードの並び順は
/// 固有値（周期の長い順）で決まり、方向とは無関係**のため、壁の有無で
/// 架構の X・Y 方向剛性の相対関係が変わると、`period[0]` が指す方向自体が
/// 入れ替わりうる（本フィクスチャで実測: 壁ありは Y 方向が最も柔らかく
/// `period[0]` を占め、壁なしは壁を失った X 方向が最も柔らかくなって
/// `period[0]` を占める。どちらも壁の有無に関係ない Y 方向どうし・
/// 別モードどうしを比べてしまい、壁の効果を見ていなかった）。
fn dominant_x_period(modal: &squid_n_solver::dynamic::eigen::ModalResult) -> Option<f64> {
    modal
        .effective_mass
        .iter()
        .enumerate()
        .max_by(|a, b| a.1[0].partial_cmp(&b.1[0]).unwrap())
        .map(|(i, _)| modal.period[i])
}

/// 壁要素が固有周期（＝剛性行列）に確実に寄与していることの確認。
///
/// 壁は Y=0 面（X 方向にスパンする面）にあり、面内せん断で効くのは
/// **X 方向**の水平剛性のみ（Y 方向の面外曲げ剛性は無視できるほど小さい）。
/// そのため比較対象は「X 方向が卓越するモードの周期」に限定する必要がある
/// （`dominant_x_period` 参照。単純に `period[0]` を比べると、壁の有無で
/// `period[0]` が指す方向自体が入れ替わり、Y 方向どうしを比較して
/// 「壁が効いていない」という誤った結論になる）。
///
/// `snapshot_wall_bay_scalars` の DL 側材端力がほぼゼロになる理由の裏付け
/// （壁が剛性に効いていないのではなく、この対称な架構・鉛直荷重条件では
/// 壁柱換算の軸力・曲げが小さいだけであること）も兼ねる。
#[test]
fn test_wall_element_changes_eigen_period() {
    let mut with_wall = wall_bay_app();
    with_wall.run_preparation();
    with_wall.core.scoped.last_error = None;
    with_wall.run_eigen(3);
    let t_with = dominant_x_period(
        with_wall
            .core
            .scoped
            .results
            .as_ref()
            .unwrap()
            .modal
            .as_ref()
            .unwrap(),
    )
    .expect("X 方向卓越モードが求まるはず");

    let mut model_without = wall_bay_model();
    // 壁の解析要素は生成物のため、入力側の壁版を取り除けば生成されなくなる
    // （D5）。壁領域からの参照（`wall_plate_ids`）もあわせて外し、ダングリング
    // 参照による `validate()` エラーを避ける。
    //
    // 取り除くのは耐震壁（壁版 0）だけで、パラペット（壁版 1）は双方に残す。
    // パラペットまで消すとその自重の分だけ質量も変わり、周期の差が「壁の面内
    // せん断剛性の寄与」を表さなくなる。
    for r in &mut model_without.wall_regions {
        r.wall_plate_ids.clear();
    }
    model_without.wall_plates.retain(|p| p.id != WallPlateId(0));
    // ID＝配列インデックスの不変条件を保つため詰め直す。
    for (i, p) in model_without.wall_plates.iter_mut().enumerate() {
        p.id = WallPlateId(i as u32);
    }
    let mut without_wall = App::default();
    without_wall.core.analysis_cfg.threads = 1;
    without_wall.core.model = model_without;
    without_wall.run_preparation();
    without_wall.core.scoped.last_error = None;
    without_wall.run_eigen(3);
    let t_without = dominant_x_period(
        without_wall
            .core
            .scoped
            .results
            .as_ref()
            .unwrap()
            .modal
            .as_ref()
            .unwrap(),
    )
    .expect("X 方向卓越モードが求まるはず");

    // 実測: with≈0.0527s（壁あり、X 方向卓越は 2 次モード）、
    // without≈0.0725s（壁なし、X 方向卓越は 1 次モード）で約 27% の差。
    // 壁の面内せん断剛性が X 方向の水平剛性を実際に支配していることの
    // 直接的な裏付けになる大きさのため、しきい値は安全側に 10% とする。
    assert!(
        (t_with - t_without).abs() > t_without * 0.1,
        "壁の有無で X 方向卓越モードの周期が実質的に変わらない（壁が剛性に寄与していない）: \
         with={t_with}, without={t_without}"
    );
}

/// 本フィクスチャの 4 つの鉛直構面（X=0・X=4000・Y=0〔壁面〕・Y=3000〔雑壁面〕）が、
/// いずれも `region_gen::wall` で閉じた矩形として検出できることの確認。
///
/// 柱脚どうしの梁を追加する前は「柱 2 本＋頂部の梁 1 本」の開いた U 字にしかならず
/// 境界を 1 つも検出できなかった（モジュール doc 参照）。`WallRegion`/`WallPlate` の
/// 型実装（Step 7+8 本体）は本フィクスチャでの境界検出を前提にするため、退行しないよう
/// 面数・面積を固定する。
#[test]
fn test_region_gen_wall_finds_all_four_faces() {
    use squid_n_core::region_gen::scan_wall_region_boundaries;

    let model = wall_bay_model();
    let scan = scan_wall_region_boundaries(&model);
    assert_eq!(scan.unclosed, 0, "半辺の後続は一意に定まるはず");
    assert_eq!(
        scan.boundaries.len(),
        4,
        "4 つの鉛直構面すべてが閉じた矩形になるはず"
    );

    let mut areas: Vec<f64> = scan.boundaries.iter().map(|b| b.area(&model)).collect();
    areas.sort_by(f64::total_cmp);
    // X=0・X=4000 面（3m×3m）が 2 面、Y=0・Y=3000 面（4m×3m）が 2 面。
    let expected = [
        3000.0 * 3000.0,
        3000.0 * 3000.0,
        4000.0 * 3000.0,
        4000.0 * 3000.0,
    ];
    for (a, e) in areas.iter().zip(expected.iter()) {
        assert!(
            (a - e).abs() < 1.0,
            "境界面積 {areas:?}（期待値 {expected:?}）"
        );
    }
}

/// 壁を含む Ds 群・保有水平耐力・増分解析の代表スカラをピン止めする
/// （dig 2026-08-26 Q2=A）。
///
/// ElemId→領域参照への張り替え（`holding.rs`/`pushover.rs`。dig Q2 の本題）に
/// 着手する前の安全網。壁の耐力壁種別判定（告示「耐力壁の種別」表。τu/Fc で
/// WA〜WD、実装上は `MemberRank::FA`〜`FD` に統合）・保有水平耐力・増分解析の
/// いずれも、これまで壁を含むモデルで一度も回帰されていなかった
/// （`full_model.rs` の実フィクスチャは壁要素 0 件のため。モジュール doc 参照）。
/// 参照張り替えで壁の応答（`resp_by_elem`）や壁長・壁厚の参照が壊れると、
/// 壁は判定不能としてスキップされ層は選択ランクへ静かにフォールバックする
/// （`holding.rs` 参照）。これはエラーにならないため、値の変化でしか検出できない。
#[test]
fn snapshot_wall_ds_group_and_holding_capacity() {
    let mut app = wall_bay_app();
    app.run_preparation();
    assert!(
        app.core.scoped.last_error.is_none(),
        "準備計算: {:?}",
        app.core.scoped.last_error
    );
    app.run_static_all();
    assert!(
        app.core.scoped.last_error.is_none(),
        "静的解析: {:?}",
        app.core.scoped.last_error
    );
    app.run_eigen(app.core.analysis_cfg.n_modes);
    assert!(
        app.core.scoped.last_error.is_none(),
        "固有値解析: {:?}",
        app.core.scoped.last_error
    );
    app.run_pushover();
    assert!(
        app.core.scoped.last_error.is_none(),
        "増分解析: {:?}",
        app.core.scoped.last_error
    );

    let (holding, ranks) = app
        .compute_holding_capacity()
        .expect("保有水平耐力が算定できるはず");

    let mut out = String::new();
    let mut line = |k: &str, v: String| out.push_str(&format!("{k} = {v}\n"));
    assert_eq!(holding.stories.len(), 1, "本フィクスチャは 1 層");
    let s = &holding.stories[0];
    assert!(
        s.qu > 0.0 && s.qu.is_finite(),
        "保有水平耐力 Qu が異常: {}",
        s.qu
    );
    // RC 造の Ds 表（`ds_rc`）が取りうる値の範囲は 0.30〜0.55（`App::design_frame`
    // を `FrameType::RcWall` にしないと鋼構造の表 `ds_steel`（0.25〜0.55）が使われ
    // てしまい、この範囲チェックを鋼構造の下限 0.25 まで緩めない限り気づけない
    // 誤りになる。`wall_bay_app` 参照）。
    assert!(
        (0.3..=0.55).contains(&s.ds),
        "構造特性係数 Ds が RC 造の規定範囲外: {}",
        s.ds
    );
    assert!(
        s.qun > 0.0 && s.qun.is_finite(),
        "必要保有水平耐力 Qun が異常: {}",
        s.qun
    );
    line("holding.qu", sig4(s.qu));
    line("holding.ds", sig4(s.ds));
    line("holding.qun", sig4(s.qun));
    assert_eq!(ranks.len(), 1, "本フィクスチャは 1 層");
    line("holding.rank", format!("{:?}", ranks[0]));

    insta::assert_snapshot!(out);
}

/// `App::compute_holding_capacity` の部材ランク自動判定
/// （`design_rank_auto == true`）が壁展開モデルを見ていることの回帰テスト
/// （申し送り §5.15）。この経路は `self.model.elements` を直接走査するため、
/// 壁展開前のモデルでは耐震壁が一度も `ElementKind::Wall` として分類されず
/// `wall_members` へ積まれない。`design_frame == FrameType::RcWall`
/// （耐力壁付き）を宣言しているのに層内の耐力壁が 1 枚も検出できないと、
/// `ds_beta_u_unavailable` が true になり βu を使わない簡易 Ds 表へ
/// フォールバックする（`holding.rs` 参照）。壁展開モデルを見ていれば、
/// この 1 バイ・1 枚壁のフィクスチャでは耐震壁が検出され false になるはず。
///
/// `snapshot_wall_ds_group_and_holding_capacity`（`design_rank_auto` 既定 false）
/// はこの分岐を一度も通らないため検知できない（§5.15 未検証一覧参照）。
#[test]
fn test_holding_capacity_auto_rank_detects_wall() {
    let mut app = wall_bay_app();
    app.run_preparation();
    assert!(
        app.core.scoped.last_error.is_none(),
        "準備計算: {:?}",
        app.core.scoped.last_error
    );
    app.run_static_all();
    assert!(
        app.core.scoped.last_error.is_none(),
        "静的解析: {:?}",
        app.core.scoped.last_error
    );
    app.run_eigen(app.core.analysis_cfg.n_modes);
    assert!(
        app.core.scoped.last_error.is_none(),
        "固有値解析: {:?}",
        app.core.scoped.last_error
    );
    app.run_pushover();
    assert!(
        app.core.scoped.last_error.is_none(),
        "増分解析: {:?}",
        app.core.scoped.last_error
    );

    app.core.design_rank_auto = true;
    let (_holding, _ranks) = app
        .compute_holding_capacity()
        .expect("保有水平耐力が算定できるはず");

    assert!(
        !app.core.scoped.ds_beta_u_unavailable,
        "壁展開モデルを見ていないと、耐震壁が model.elements 側から検出できず \
         wall_members が空のまま βu 算定不能（ds_beta_u_unavailable=true）に \
         フォールバックするはず"
    );
}
