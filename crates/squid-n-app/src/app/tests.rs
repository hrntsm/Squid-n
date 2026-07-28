use super::*;

#[test]
fn test_is_steel() {
    assert!(is_steel("SN400"));
    assert!(is_steel("SS400"));
    assert!(is_steel("SM490"));
    assert!(!is_steel("SD345"));
    assert!(!is_steel(" Concrete"));
}

#[test]
fn test_run_design_check_empty_model() {
    let mut app = App::default();
    app.run_design_check();
    assert!(app.results.is_none() || app.results.as_ref().unwrap().member_checks.is_empty());
}

/// `EventLog` の保持件数上限（1000件）: 上限を1件超えて push すると、
/// 件数は上限に保たれ、最も古い1件（先頭）が捨てられる。
#[test]
fn test_event_log_caps_entries() {
    let mut log = EventLog::default();
    for i in 0..1001 {
        log.push(LogLevel::Info, format!("msg{i}"));
    }
    assert_eq!(log.entries.len(), 1000);
    // 先頭（msg0）が捨てられ、msg1 が先頭に繰り上がっている。
    assert_eq!(log.entries.first().unwrap().message, "msg1");
    assert_eq!(log.entries.last().unwrap().message, "msg1000");
}

/// `report_error` が `last_error` とログの両方へ反映されることを確認する。
#[test]
fn test_report_error_updates_last_error_and_log() {
    let mut app = App::default();
    app.report_error("テストエラー");
    assert_eq!(app.last_error.as_deref(), Some("テストエラー"));
    let last = app.log.entries.last().expect("ログにエントリがあるはず");
    assert_eq!(last.level, LogLevel::Error);
    assert_eq!(last.message, "テストエラー");
}

/// エラー報告は下ドックを開くだけでなくログタブへ切り替える
/// （診断・テーブル表示中でもエラー本文が見えるように）。
#[cfg(feature = "gui")]
#[test]
#[allow(clippy::field_reassign_with_default)]
fn test_report_error_switches_bottom_tab_to_log() {
    let mut app = App::default();
    app.bottom_tab = BottomTab::Diagnostics;
    app.bottom_dock_open = false;
    app.report_error("テストエラー");
    assert!(app.bottom_dock_open);
    assert_eq!(app.bottom_tab, BottomTab::Log);
}

/// モデル差し替えで作成モードと選択バッファが解除される
/// （旧モデルの節点 id が残ると意図しない部材が生成されうるため）。
#[cfg(feature = "gui")]
#[test]
#[allow(clippy::field_reassign_with_default)]
fn test_load_model_resets_draw_modes() {
    let mut app = App::default();
    app.beam_draw_mode = true;
    app.beam_draw_first = Some(squid_n_core::ids::NodeId(3));
    app.wall_draw_mode = true;
    app.wall_draw_nodes.push(squid_n_core::ids::NodeId(1));
    app.slab_draw_nodes.push(squid_n_core::ids::NodeId(2));
    app.load_model(crate::sample::portal_frame());
    assert!(!app.beam_draw_mode);
    assert!(app.beam_draw_first.is_none());
    assert!(!app.wall_draw_mode);
    assert!(app.wall_draw_nodes.is_empty());
    assert!(app.slab_draw_nodes.is_empty());
}

/// 一本部材指定（beam_groups）: 2 分割梁のグループ合成値
/// （全長・端部/中央モーメント・せん断スパン代表値）の手計算照合。
#[test]
fn test_beam_group_overrides_combines_members() {
    use smallvec::SmallVec;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{MaterialId, NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Model, Node, RigidZone,
    };
    use squid_n_element::beam::MemberForces;

    let node = |id: u32, x: f64| Node {
        id: NodeId(id),
        coord: [x, 0.0, 0.0],
        restraint: Dof6Mask::FREE,
        mass: None,
        story: None,
    };
    let beam = |id: u32, n0: u32, n1: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: {
            let mut v: SmallVec<[NodeId; 8]> = SmallVec::new();
            v.push(NodeId(n0));
            v.push(NodeId(n1));
            v
        },
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone::default(),
        plastic_zone: None,
        spring: None,
    };
    let model = Model {
        nodes: vec![node(0, 0.0), node(1, 3000.0), node(2, 6000.0)],
        elements: vec![beam(0, 0, 1), beam(1, 1, 2)],
        beam_groups: vec![vec![ElemId(0), ElemId(1)]],
        ..Default::default()
    };
    let mf = |rows: Vec<(f64, f64, f64)>| MemberForces {
        at: rows
            .into_iter()
            .map(|(p, q, m)| (p, [0.0, q, 0.0, 0.0, 0.0, m]))
            .collect(),
    };
    let member_forces = vec![
        (
            ElemId(0),
            mf(vec![
                (0.0, 50_000.0, -200.0e6),
                (0.5, 30_000.0, 20.0e6),
                (1.0, 10_000.0, 100.0e6),
            ]),
        ),
        (
            ElemId(1),
            mf(vec![
                (0.0, -10_000.0, 100.0e6),
                (0.5, -30_000.0, 20.0e6),
                (1.0, -50_000.0, -200.0e6),
            ]),
        ),
    ];

    let overrides = beam_group_overrides(&model, &member_forces);
    let ov = overrides.get(&ElemId(0)).expect("グループ所属");
    // 両要素が同じ合成値を共有する。
    assert!(std::rc::Rc::ptr_eq(ov, overrides.get(&ElemId(1)).unwrap()));
    // 全長 = 3000+3000。
    assert!((ov.length - 6000.0).abs() < 1e-9);
    // 端部モーメントは外端（要素0の pos0、要素1の pos1）。
    assert_eq!(ov.end_moments_z, Some((-200.0e6, -200.0e6)));
    // A式: M0 = (50k+50k)・6000/8 = 75e6、Mc_A = 75e6 − 200e6 < 0。
    // B式: グループ中央(3000mm)＝要素0の pos=1.0 の行 → +100e6。
    // 中央モーメント = max(|B|, Mc_A) に B の符号 → +100e6。
    assert!((ov.mid_moment_z.unwrap() - 100.0e6).abs() < 1e-3);
    // せん断スパン代表値: |M| 最大 200e6 の行の (200e6, 50e3)。
    let (m_rep, q_rep) = ov.shear_span.unwrap();
    assert!((m_rep - 200.0e6).abs() < 1e-3);
    assert!((q_rep - 50_000.0).abs() < 1e-6);
    // 剛域なし → 内法長 = 全長。
    assert!((ov.clear_length - 6000.0).abs() < 1e-9);

    // グループ未指定なら空。
    let mut model2 = model;
    model2.beam_groups.clear();
    assert!(beam_group_overrides(&model2, &member_forces).is_empty());
}

/// 剛域自動算定・危険断面フィルタのテスト用モデル。
/// `sample::portal_frame`（対角材を含む変則的な接続）と異なり、
/// 柱(node0-node1)・梁(node1-node2)・柱(node2-node3)が各節点で厳密に直交する
/// 素直なポータルフレーム（柱 H-300x300x10x15・梁 H-400x200x8x13、SN400B）。
/// - node0(柱1脚部)・node3(柱2脚部): 他要素と接続しない → face=0（節点芯のまま）
/// - node1(柱1頭部/梁始端)・node2(梁終端/柱2頭部): 柱・梁が直交 → face>0
fn aligned_portal_frame() -> squid_n_core::model::Model {
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material,
        MemberLoad, MemberLoadKind, Model, Node,
    };
    use squid_n_section::shape::SectionShape;

    let mut model = Model::default();

    let coords = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 3000.0],
        [4000.0, 0.0, 3000.0],
        [4000.0, 0.0, 0.0],
    ];
    for (i, c) in coords.iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: if i == 0 || i == 3 {
                squid_n_core::dof::Dof6Mask::FIXED
            } else {
                squid_n_core::dof::Dof6Mask::FREE
            },
            mass: None,
            story: None,
        });
    }

    // RC 造ラーメン（S 造は剛域長 0 となるため、
    // 剛域自動算定の配管検証には RC 断面を用いる）。
    let rebar = squid_n_core::section_shape::RcRebar {
        main_grade: None,
        main_x: squid_n_core::section_shape::BarSet {
            count: 4,
            dia: 22.0,
            layers: 1,
        },
        main_y: squid_n_core::section_shape::BarSet {
            count: 4,
            dia: 22.0,
            layers: 1,
        },
        cover: 40.0,
        shear: squid_n_core::section_shape::ShearBar {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
            grade: None,
        },
    };
    let col_shape = SectionShape::RcRect {
        b: 300.0,
        d: 300.0,
        rebar: rebar.clone(),
    };
    let beam_shape = SectionShape::RcRect {
        b: 200.0,
        d: 400.0,
        rebar,
    };
    model
        .sections
        .push(col_shape.to_section(SectionId(0), "柱 RC-300x300".into()));
    model
        .sections
        .push(beam_shape.to_section(SectionId(1), "梁 RC-200x400".into()));

    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: squid_n_core::ids::MaterialId(0),
        name: "FC24".into(),
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });

    let members = [
        (0u32, 0u32, 1u32, 0u32, [1.0, 0.0, 0.0]),
        (1, 1, 2, 1, [0.0, 0.0, 1.0]),
        (2, 2, 3, 0, [1.0, 0.0, 0.0]),
    ];
    for (id, i, j, sec, ref_vector) in members {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: Some(SectionId(sec)),
            material: Some(squid_n_core::ids::MaterialId(0)),
            local_axis: LocalAxis { ref_vector },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }

    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "長期".into(),
        nodal: Vec::new(),
        member: vec![MemberLoad {
            elem: ElemId(1),
            dir: [0.0, 0.0, -1.0],
            kind: MemberLoadKind::Distributed {
                a: 0.0,
                b: 4000.0,
                w1: 10.0,
                w2: 10.0,
            },
        }],
    });

    model
}

/// 剛域自動算定が解析パイプラインへ接続されていること（設計書 §6.2.1、標準実装）。
/// 解析エントリ(`run_linear_static`)を通す前は既定の 0（未適用）のままだが、
/// 通した後は `apply_rigid_zones_for_analysis` により `elem.rigid_zone` が
/// 自動算定値へ更新される。
#[test]
fn test_run_linear_static_applies_auto_rigid_zones() {
    let mut app = App::default();
    app.load_model(aligned_portal_frame());

    // 適用前は既定の 0（apply_auto_rigid_zones 未実行）。
    assert_eq!(app.model.elements[1].rigid_zone.length_i, 0.0);
    assert_eq!(app.model.elements[1].rigid_zone.face_i, 0.0);

    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    // 梁(id=1)の i端(node1, 柱と直交)。
    // λ_i = D_orth/2 − D_self/4 = 柱せい/2 − 梁せい/4 = 150 − 100 = 50
    // face_i = D_orth/2 = 柱せい/2 = 150
    let beam = &app.model.elements[1];
    assert!(
        (beam.rigid_zone.length_i - 50.0).abs() < 1e-9,
        "length_i={}",
        beam.rigid_zone.length_i
    );
    assert!(
        (beam.rigid_zone.face_i - 150.0).abs() < 1e-9,
        "face_i={}",
        beam.rigid_zone.face_i
    );

    // 柱(id=0)の j端(node1, 梁と直交)。
    // λ_j = D_orth/2 − D_self/4 = 梁せい/2 − 柱せい/4 = 200 − 75 = 125
    // face_j = D_orth/2 = 梁せい/2 = 200
    let col = &app.model.elements[0];
    assert!(
        (col.rigid_zone.length_j - 125.0).abs() < 1e-9,
        "length_j={}",
        col.rigid_zone.length_j
    );
    assert!(
        (col.rigid_zone.face_j - 200.0).abs() < 1e-9,
        "face_j={}",
        col.rigid_zone.face_j
    );
    // 柱脚(node0)は他要素と接続しないため face_i は 0 のまま。
    assert_eq!(col.rigid_zone.face_i, 0.0);
}

/// `run_design_check` が危険断面位置（§6.2.3、既定は柱フェイスと中央）のみを
/// 検定し、剛域が有る端の節点芯は検定対象外になることを確認する。
/// 剛域が無い端（face=0）では従来どおり節点芯が検定対象に残る。
#[test]
fn test_run_design_check_filters_to_design_positions() {
    let mut app = App::default();
    app.load_model(aligned_portal_frame());
    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let member_checks = &app.results.as_ref().unwrap().member_checks;
    assert!(!member_checks.is_empty());
    let positions_of = |elem: ElemId| -> Vec<f64> {
        member_checks
            .iter()
            .find(|m| m.elem == elem)
            .map(|m| m.positions.iter().map(|p| p.xi).collect())
            .unwrap_or_default()
    };

    // 梁(id=1): 両端とも柱と直交(face>0)のため、節点芯 0.0/1.0 は検定対象外。
    let beam_positions: Vec<f64> = positions_of(ElemId(1));
    assert!(
        !beam_positions.iter().any(|p| *p < 1e-6),
        "梁の節点芯(i端)が検定対象に残っている: {:?}",
        beam_positions
    );
    assert!(
        !beam_positions.iter().any(|p| (*p - 1.0).abs() < 1e-6),
        "梁の節点芯(j端)が検定対象に残っている: {:?}",
        beam_positions
    );
    assert!(
        beam_positions.iter().any(|p| (*p - 0.5).abs() < 1e-6),
        "梁の中央が検定対象から抜けている: {:?}",
        beam_positions
    );

    // 柱(id=0): 脚部(node0)は他要素と接続しない(face_i=0)ため節点芯 0.0 のままが
    // 危険断面位置に一致し、検定対象に残る(従来挙動と一致)。
    // 頭部(node1)は梁と直交(face_j>0)のため節点芯 1.0 は検定対象外になる。
    let col_positions: Vec<f64> = positions_of(ElemId(0));
    assert!(
        col_positions.iter().any(|p| *p < 1e-6),
        "剛域の無い柱脚(節点芯)が検定対象から抜けている: {:?}",
        col_positions
    );
    assert!(
        !col_positions.iter().any(|p| (*p - 1.0).abs() < 1e-6),
        "柱頭の節点芯が検定対象に残っている: {:?}",
        col_positions
    );
}

/// 部材付帯情報（`MemberDetailAttr`）を持つ部材で設計検定を実行すると、
/// ハンチ端・継手位置の検定結果が `checks` に含まれること
/// （`design_positions` が `Model::member_detail` の追加検定位置を
/// 取り込んでいるかの確認。§6.2.3「位置はユーザが追加・変更可能」）。
#[test]
fn test_run_design_check_includes_member_detail_positions() {
    use squid_n_core::model::{Haunch, JointKind, MemberDetailAttr, MemberJoint};

    let mut model = aligned_portal_frame();
    // 梁(id=1, i端-j端: node1-node2, 全長4000mm)にハンチ(i端)と継手を追加する。
    // i端は柱と直交(face_i=150、自動剛域算定後)のため、ハンチ端位置は
    // (150+700)/4000 = 0.2125、継手位置は 1000/4000 = 0.25 になる。
    model.member_detail_attrs.push(MemberDetailAttr {
        elem: ElemId(1),
        haunch_i: Some(Haunch {
            length: 700.0,
            depth_increase: 100.0,
            width_increase: 0.0,
        }),
        haunch_j: None,
        joints: vec![MemberJoint {
            distance: 1000.0,
            kind: JointKind::Site,
        }],
    });

    let mut app = App::default();
    app.load_model(model);
    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let member_checks = &app.results.as_ref().unwrap().member_checks;
    let beam_positions: Vec<f64> = member_checks
        .iter()
        .find(|m| m.elem == ElemId(1))
        .map(|m| m.positions.iter().map(|p| p.xi).collect())
        .unwrap_or_default();

    assert!(
        beam_positions.iter().any(|p| (*p - 0.2125).abs() < 1e-6),
        "ハンチ端の検定位置が抜けている: {:?}",
        beam_positions
    );
    assert!(
        beam_positions.iter().any(|p| (*p - 0.25).abs() < 1e-6),
        "継手位置の検定位置が抜けている: {:?}",
        beam_positions
    );
}

#[test]
fn test_staleness_mark_edited_marks_downstream() {
    let mut s = Staleness::default();
    assert!(!s.results_stale);
    s.mark_edited();
    assert!(s.results_stale);
    assert!(s.design_stale);
    let now = SystemTime::now();
    s.last_run = Some(now);
    s.mark_fresh();
    assert!(!s.results_stale);
    assert!(!s.design_stale);
    assert!(s.last_run.is_some());
}

#[test]
fn test_tab_default_is_model() {
    assert_eq!(Tab::Model, Tab::default());
}

#[test]
fn test_seismic_flow_requires_then_uses_stories() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());

    // 階なし → 明示エラー（サイレントゼロ結果ではない）
    app.run_seismic(SeismicDir::X);
    assert!(app.last_error.is_some());

    // 階の自動生成 → 地震静的が成功する
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert_eq!(app.model.stories.len(), 1);
    assert!(app.model.stories[0].seismic_weight.unwrap() > 0.0);

    // ユーザー荷重ケース0("長期")を先に実行しておく。
    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let user_disp = app.results.as_ref().unwrap().statics[0].1.disp.clone();

    app.run_seismic(SeismicDir::X);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    // 地震静的の結果は StaticCaseKey::Seismic(X) に格納され、直前に実行した
    // ユーザーケース0(StaticCaseKey::User)の結果を上書きしない
    // (旧実装ではどちらも LoadCaseId(0) を共有し、後勝ちで上書きされていた)。
    let r = app.results.as_ref().unwrap();
    assert_eq!(
        r.statics.len(),
        2,
        "ユーザーケース0と地震静的Xの結果が両方残っているはず"
    );
    let seismic_disp = r
        .statics
        .iter()
        .find(|(k, _)| *k == StaticCaseKey::Seismic(SeismicDir::X))
        .expect("地震静的Xの結果が残っているはず")
        .1
        .disp
        .clone();
    let kept_user_disp = r
        .statics
        .iter()
        .find(|(k, _)| *k == StaticCaseKey::User(LoadCaseId(0)))
        .expect("ユーザーケース0の結果が地震静的実行後も残っているはず")
        .1
        .disp
        .clone();
    assert_eq!(
        kept_user_disp, user_disp,
        "ユーザーケース0の結果は地震静的の実行後も変わらないはず（衝突していない）"
    );
    // 柱頭が X 方向へ変位している(地震静的の結果)
    assert!(seismic_disp[2][0].abs() > 1e-3, "{}", seismic_disp[2][0]);

    // ナビゲータでそれぞれのキーを選択すれば current_static が個別に引ける
    app.nav.focus_result = Some(StaticKey::Case(StaticCaseKey::User(LoadCaseId(0))));
    assert_eq!(app.current_static().unwrap().disp, kept_user_disp);
    app.nav.focus_result = Some(StaticKey::Case(StaticCaseKey::Seismic(SeismicDir::X)));
    assert_eq!(app.current_static().unwrap().disp, seismic_disp);

    // undo で WY・EY・EX の同期 → 階定義 → DL(自重)の同期の順に戻る
    // （generate_stories_action が DL 同期 → 階適用 → EX/EY 同期 → WX/WY 同期の
    // 順に undo 履歴を積む。平面架構のため X 方向の風は見付け幅 0 で構築されず、
    // 風は WY の 1 件のみ。以降の解析実行時の同期は冪等で履歴を積まない）
    app.undo.undo(&mut app.model); // WY
    assert!(app
        .model
        .load_cases
        .iter()
        .all(|lc| lc.name != WY_CASE_NAME));
    app.undo.undo(&mut app.model); // EY
    app.undo.undo(&mut app.model); // EX
    assert!(app
        .model
        .load_cases
        .iter()
        .all(|lc| lc.name != EX_CASE_NAME && lc.name != EY_CASE_NAME));
    app.undo.undo(&mut app.model); // 階定義
    assert!(app.model.stories.is_empty());
    app.undo.undo(&mut app.model); // DL(自重)
    assert!(app
        .model
        .load_cases
        .iter()
        .all(|lc| lc.name != DL_CASE_NAME));
}

/// 性能修正: Ai算定法が既定の略算（`AiMode::Approx`）の場合、固有値解析を
/// 一切実行せずに `sync_seismic_load_cases_action` が EX/EY を同期できる
/// （暗黙の固有値解析・暗黙の `Analysis::prepare` の廃止）。
#[test]
fn test_sync_seismic_approx_mode_syncs_ex_ey_without_eigen() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    assert_eq!(app.analysis_cfg.ai_mode, AiMode::Approx, "既定は略算のはず");
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(
        app.last_notice.is_none(),
        "略算モードでは注意メッセージは出ないはず: {:?}",
        app.last_notice
    );
    assert!(app.results.is_none(), "固有値解析は実行されていないはず");

    let ex = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == EX_CASE_NAME)
        .expect("EXケースが同期されるはず");
    assert!(!ex.nodal.is_empty(), "EXには水平力が入っているはず");
    let ey = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == EY_CASE_NAME)
        .expect("EYケースが同期されるはず");
    assert!(!ey.nodal.is_empty(), "EYには水平力が入っているはず");
}

/// 性能修正: 精算周期（`AiMode::SemiPrecise`）を選択したが固有値解析が
/// 未実行の場合、`sync_seismic_load_cases_action` は EX/EY を更新せず、
/// `last_notice` に実行を促すメッセージを設定する（`last_error` は使わない。
/// 解析自体は継続してよいため）。
#[test]
fn test_sync_seismic_semiprecise_without_eigen_sets_notice_and_skips() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    // まず既定(略算)で EX/EY を生成しておく。
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let ex_before = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == EX_CASE_NAME)
        .cloned()
        .expect("EXケースが同期されているはず");

    // 精算周期へ切り替え、固有値解析は実行しない。
    app.analysis_cfg.ai_mode = AiMode::SemiPrecise;
    app.last_notice = None;
    app.sync_seismic_load_cases_action();

    assert!(
        app.last_notice.is_some(),
        "固有値解析未実行時は注意メッセージが設定されるはず"
    );
    let ex_after = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == EX_CASE_NAME)
        .cloned()
        .expect("EXケースは残っているはず（削除されない）");
    assert_eq!(
        ex_before, ex_after,
        "固有値解析未実行時はEXケースが更新されないはず"
    );

    // run_seismic も同様に、解析を行わず last_error で案内する。
    app.last_error = None;
    app.run_seismic(SeismicDir::X);
    assert!(
        app.last_error.is_some(),
        "SemiPreciseで固有値解析未実行ならrun_seismicはエラーを返すはず"
    );
}

/// 性能修正: `sync_auto_load_cases_action` は前回同期時からモデル・関連設定
/// （`analysis_cfg` の一部）が変わっていなければ DL/LL/EX/EY の再計算を
/// 丸ごとスキップする。ハッシュが一致する状態を人為的に作り、既存の
/// （手で壊した）荷重ケース内容が上書きされない＝スキップされたことを確認する。
#[test]
fn test_sync_auto_load_cases_action_skips_when_hash_unchanged() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let ex_idx = app
        .model
        .load_cases
        .iter()
        .position(|lc| lc.name == EX_CASE_NAME)
        .expect("EXケースが生成されているはず");
    assert!(
        !app.model.load_cases[ex_idx].nodal.is_empty(),
        "前提: EXには水平力が入っているはず"
    );
    // EX ケースの内容を手で壊す。
    app.model.load_cases[ex_idx].nodal.clear();
    app.model.load_cases[ex_idx].member.clear();

    // 「この(壊れた)モデル状態で同期済み」であるとキャッシュへ偽装する
    // （`compute_auto_load_sync_hash` と同じロジック。Approx モードなので
    // 固有周期 T のハッシュ組み込みは対象外）。
    fn fake_hash(app: &App) -> u64 {
        use std::hash::{Hash, Hasher};
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        if let Ok(bytes) = bincode::serialize(&app.model) {
            bytes.hash(&mut hasher);
        }
        std::mem::discriminant(&app.analysis_cfg.ai_mode).hash(&mut hasher);
        app.analysis_cfg.z.to_bits().hash(&mut hasher);
        (app.analysis_cfg.soil as u8).hash(&mut hasher);
        app.analysis_cfg.c0.to_bits().hash(&mut hasher);
        app.analysis_cfg.v0.to_bits().hash(&mut hasher);
        (app.analysis_cfg.roughness as u8).hash(&mut hasher);
        app.analysis_cfg.parapet_mm.to_bits().hash(&mut hasher);
        hasher.finish()
    }
    app.auto_load_sync_hash = Some(fake_hash(&app));

    app.sync_auto_load_cases_action();

    let ex_after = &app.model.load_cases[ex_idx];
    assert!(
        ex_after.nodal.is_empty() && ex_after.member.is_empty(),
        "ハッシュ一致時は同期がスキップされ、壊した内容がそのまま残るはず"
    );
}

/// 剛床代表節点は慣性力重心に自動生成される。再度自動生成しても
/// 既存の代表節点を再利用するため節点数が増えないことを確認する
/// （story_gen + edit の統合: `generate_stories` → `ApplyStories` の往復）。
#[test]
fn test_generate_stories_action_reuses_rep_node_on_regenerate() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    let n0 = app.model.nodes.len();
    assert!(app.model.generated_masters.is_empty());

    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let n1 = app.model.nodes.len();
    assert_eq!(n1, n0 + 1, "剛床代表節点が 1 つ新規生成される");
    assert_eq!(app.model.generated_masters.len(), 1);
    let master_after_first = app.model.generated_masters[0];
    assert!(app.model.validate().is_ok());

    // 再生成しても代表節点は再利用され、節点数は増えない。
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert_eq!(
        app.model.nodes.len(),
        n1,
        "再生成でノード数が増えてはいけない（代表節点の再利用）"
    );
    assert_eq!(app.model.generated_masters, vec![master_after_first]);
    assert!(app.model.validate().is_ok());

    // 固有値解析・地震静的解析が正常に動作する（生成された剛床を含む縮約の統合確認）。
    app.run_eigen(1);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    app.run_seismic(SeismicDir::X);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
}

#[test]
fn test_time_history_sample_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.analysis_cfg.th_duration = 2.0;
    app.run_time_history_sample();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let th = app.results.as_ref().unwrap().time_history.as_ref().unwrap();
    assert!(th.history.node_disp.len() > 100);
    assert!(
        th.history.node_disp.iter().any(|v| v.abs() > 1e-6),
        "応答がゼロのままです"
    );
    assert!(th.history.node.is_some());
}

#[test]
fn test_time_history_y_direction_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.analysis_cfg.th_duration = 2.0;
    app.analysis_cfg.th_dir = ThDir::Y;
    app.run_time_history_sample();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let th = app.results.as_ref().unwrap().time_history.as_ref().unwrap();
    assert!(
        th.history.record_dir_y,
        "th_dir=Y なのに代表応答の記録方向が X のままです"
    );
    assert!(
        th.history.node_disp.iter().any(|v| v.abs() > 1e-6),
        "応答がゼロのままです"
    );
}

#[test]
fn test_build_ground_motion_routes_by_direction() {
    // wave 構築のみを検証する純粋関数のテスト（th_dir=Y でも accel_x 側に
    // 誤って入らないことを確認する）。
    let accel = vec![1.0, 2.0, 3.0];
    let wave_x = App::build_ground_motion(0.01, ThDir::X, accel.clone());
    assert_eq!(wave_x.accel_x, accel);
    assert!(wave_x.accel_y.is_none());

    let wave_y = App::build_ground_motion(0.01, ThDir::Y, accel.clone());
    assert_eq!(wave_y.accel_x, vec![0.0; accel.len()]);
    assert_eq!(wave_y.accel_y, Some(accel.clone()));
}

/// ThDir::Xy: 同一波形を accel_x・accel_y の両方に入れる（簡易仕様）。
#[test]
fn test_build_ground_motion_xy_duplicates_wave() {
    let accel = vec![1.0, 2.0, 3.0];
    let wave = App::build_ground_motion(0.01, ThDir::Xy, accel.clone());
    assert_eq!(wave.accel_x, accel);
    assert_eq!(wave.accel_y, Some(accel));
}

// ===== parse_wave_csv テスト =====

#[test]
fn test_parse_wave_csv_single_column_x_or_y() {
    let content = "10.0\n20.0\n30.0\n";
    let (accel, second) = parse_wave_csv(content, ThDir::X).unwrap();
    assert_eq!(accel, vec![100.0, 200.0, 300.0]); // gal→mm/s²(×10)
    assert!(second.is_none());

    // カンマ区切りなら最後の列を使う（従来仕様）。
    let content_csv = "0.0,10.0\n0.01,20.0\n0.02,30.0\n";
    let (accel, second) = parse_wave_csv(content_csv, ThDir::Y).unwrap();
    assert_eq!(accel, vec![100.0, 200.0, 300.0]);
    assert!(second.is_none());
}

#[test]
fn test_parse_wave_csv_single_column_too_few_points_is_err() {
    assert!(parse_wave_csv("10.0\n", ThDir::X).is_err());
    assert!(parse_wave_csv("", ThDir::X).is_err());
}

#[test]
fn test_parse_wave_csv_xy_two_columns() {
    let content = "10.0,5.0\n20.0,15.0\n30.0,25.0\n";
    let (xs, ys) = parse_wave_csv(content, ThDir::Xy).unwrap();
    assert_eq!(xs, vec![100.0, 200.0, 300.0]);
    assert_eq!(ys, Some(vec![50.0, 150.0, 250.0]));
}

#[test]
fn test_parse_wave_csv_xy_header_line_is_skipped() {
    // ヘッダ行（数値化不可）は無視され、残りの2行が (X, Y) として読める。
    let content = "x,y\n10.0,5.0\n20.0,15.0\n";
    let (xs, ys) = parse_wave_csv(content, ThDir::Xy).unwrap();
    assert_eq!(xs, vec![100.0, 200.0]);
    assert_eq!(ys, Some(vec![50.0, 150.0]));
}

#[test]
fn test_parse_wave_csv_xy_insufficient_columns_is_err() {
    let content = "10.0,5.0\n20.0\n30.0,25.0\n";
    let err = parse_wave_csv(content, ThDir::Xy).unwrap_err();
    assert_eq!(err, "X+Y には2列のCSVが必要です");
}

#[test]
fn test_parse_wave_csv_xy_too_few_points_is_err() {
    assert!(parse_wave_csv("10.0,5.0\n", ThDir::Xy).is_err());
}

#[test]
fn test_time_history_xy_sample_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.analysis_cfg.th_duration = 2.0;
    app.analysis_cfg.th_dir = ThDir::Xy;
    app.run_time_history_sample();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let th = app.results.as_ref().unwrap().time_history.as_ref().unwrap();
    assert!(
        th.history.node_disp.iter().any(|v| v.abs() > 1e-6),
        "応答がゼロのままです"
    );
}

/// 2 層等質量等剛性せん断モデル（軸ばね 2 本の直列、Ux 方向のみ自由）。
/// portal_frame は平面骨組で弱軸・面外方向の縮約後自由度が多く、
/// 固有値解析(部分空間反復)が n_modes=2 で不安定になりやすいため、
/// Rayleigh 減衰(1次・2次固有値が必要)のテストには本モデルを用いる。
fn shear_2dof_model() -> squid_n_core::model::Model {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node,
        Section,
    };
    const FREE_UX: Dof6Mask = Dof6Mask(0b111110);
    let k = 1000.0_f64;
    let m = 1.0_f64;
    let young = k * 1000.0; // EA/L = young*1/1000 = k
    let node = |id: u32, x: f64, restraint: Dof6Mask, mass: Option<[f64; 6]>| Node {
        id: NodeId(id),
        coord: [x, 0.0, 0.0],
        restraint,
        mass,
        story: None,
    };
    let beam = |id: u32, a: u32, b: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: smallvec::smallvec![NodeId(a), NodeId(b)],
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
    };
    Model {
        nodes: vec![
            node(0, 0.0, Dof6Mask::FIXED, None),
            node(1, 1000.0, FREE_UX, Some([m, 0.0, 0.0, 0.0, 0.0, 0.0])),
            node(2, 2000.0, FREE_UX, Some([m, 0.0, 0.0, 0.0, 0.0, 0.0])),
        ],
        elements: vec![beam(0, 0, 1), beam(1, 1, 2)],
        sections: vec![Section {
            id: SectionId(0),
            name: "spring".into(),
            area: 1.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 1.0,
            width: 1.0,
            as_y: 1.0,
            as_z: 1.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "mat".into(),
            young,
            poisson: 0.0,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        }],
        ..Default::default()
    }
}

#[test]
fn test_time_history_rayleigh_and_hht() {
    let mut app = App::default();
    app.load_model(shear_2dof_model());
    app.analysis_cfg.th_duration = 2.0;
    app.analysis_cfg.th_damping_model = ThDampingModel::Rayleigh;
    app.analysis_cfg.th_integrator = ThIntegrator::HhtAlpha;
    app.run_time_history_sample();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let th = app.results.as_ref().unwrap().time_history.as_ref().unwrap();
    assert!(!th.history.node_disp.is_empty());
    assert!(
        th.history.node_disp.iter().any(|v| v.abs() > 1e-6),
        "応答がゼロのままです"
    );
}

#[test]
fn test_set_story_weight_via_ui_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let story_id = app.model.stories[0].id;
    let old_weight = app.model.stories[0].seismic_weight;

    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::SetStoryWeight {
            story: story_id,
            weight: Some(12345.0),
        }),
    );
    assert_eq!(app.model.stories[0].seismic_weight, Some(12345.0));

    app.undo.undo(&mut app.model);
    assert_eq!(app.model.stories[0].seismic_weight, old_weight);
}

#[test]
fn test_pushover_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    app.analysis_cfg.push_steps = 10;
    app.run_pushover();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let po = app.results.as_ref().unwrap().pushover.as_ref().unwrap();
    assert!(!po.capacity_curve.is_empty());
}

/// プッシュオーバー結果から質点系（串団子）モデルを生成する配線の end-to-end 確認。
#[test]
fn test_lumped_mass_model_from_pushover() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    app.analysis_cfg.push_steps = 10;
    app.run_pushover();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let po = app.results.as_ref().unwrap().pushover.as_ref().unwrap();

    let lm = squid_n_solver::lumped_mass::build_lumped_mass_model(
        &app.model,
        po,
        app.analysis_cfg.lumped_mass_type,
        app.analysis_cfg.lumped_secant_ratio,
    );
    // 層数分の質点が生成され、各層のトリリニア骨格が妥当（K1>0・折点昇順）。
    assert_eq!(lm.stories.len(), app.model.stories.len());
    assert!(!lm.stories.is_empty());
    for stick in &lm.stories {
        let sk = &stick.skeleton;
        assert!(sk.k1 > 0.0, "K1>0: {sk:?}");
        assert!(sk.d1 <= sk.d2 && sk.d2 <= sk.d3, "折点昇順: {sk:?}");
        assert!(stick.mass >= 0.0);
    }
}

/// 制振ダンパーの作成→諸元変更→削除を app の undo スタック経由で確認する
/// （部材表 UI が発行する編集コマンドの統合確認）。
#[test]
fn test_damper_create_edit_delete_via_undo() {
    use squid_n_core::model::{
        DamperProps, ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis,
    };
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    let n = app.model.nodes.len();
    assert!(n >= 2);
    let (i_node, j_node) = (app.model.nodes[0].id, app.model.nodes[1].id);
    let new_id = squid_n_core::ids::ElemId(app.model.elements.len() as u32);
    let elem = ElementData {
        id: new_id,
        kind: ElementKind::Damper,
        nodes: [i_node, j_node].into_iter().collect(),
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    // 作成。
    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::AddDamper {
            elem,
            props: DamperProps::default(),
        }),
    );
    assert_eq!(app.model.damper_props(new_id), Some(DamperProps::default()));
    // 諸元変更。
    let edited = DamperProps {
        kd: 150_000.0,
        c0: 3_000.0,
        alpha: 0.35,
        ..Default::default()
    };
    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::SetDamperProps {
            elem: new_id,
            props: Some(edited),
        }),
    );
    assert_eq!(app.model.damper_props(new_id), Some(edited));
    // 削除（要素も特性も消える）。
    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::DeleteMember { id: new_id }),
    );
    assert_eq!(app.model.damper_props(new_id), None);
    assert!(app.model.elements.iter().all(|e| e.id != new_id));
}

/// `poll_job` が完了するまで待つ（タイムアウト5秒でパニック、10ms 間隔でポーリング）。
fn wait_for_job(app: &mut App) {
    let start = std::time::Instant::now();
    while !app.poll_job() {
        assert!(
            start.elapsed() < std::time::Duration::from_secs(5),
            "ジョブが時間内に完了しませんでした"
        );
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

#[test]
fn test_async_pushover_job_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    app.analysis_cfg.push_steps = 10;

    app.start_pushover_job();
    assert!(app.job.is_some());

    wait_for_job(&mut app);

    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(app.job.is_none());
    let po = app.results.as_ref().unwrap().pushover.as_ref().unwrap();
    assert!(!po.capacity_curve.is_empty());
}

#[test]
fn test_async_time_history_job_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.analysis_cfg.th_duration = 2.0;
    let wave = App::sample_wave(&app.analysis_cfg);

    app.start_time_history_job(wave);
    assert!(app.job.is_some());

    wait_for_job(&mut app);

    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(app.job.is_none());
    let th = app.results.as_ref().unwrap().time_history.as_ref().unwrap();
    assert!(th.history.node_disp.len() > 100);
    assert!(
        th.history.node_disp.iter().any(|v| v.abs() > 1e-6),
        "応答がゼロのままです"
    );
}

#[test]
fn test_start_job_while_running_is_rejected() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    app.analysis_cfg.push_steps = 10;

    app.start_pushover_job();
    assert!(app.job.is_some());

    // 実行中に再度 start しても2つ目は無視され、job は上書きされない。
    app.start_time_history_job(App::sample_wave(&app.analysis_cfg));
    assert!(app.job.is_some());
    assert_eq!(app.job.as_ref().unwrap().label, "増分解析");

    wait_for_job(&mut app);

    assert!(app.job.is_none());
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(app.results.as_ref().unwrap().pushover.is_some());
    assert!(app.results.as_ref().unwrap().time_history.is_none());
}

/// `start_linear_static_job` はバックグラウンドで `run_linear_static` と同じ結果
/// （変位・格納キー・検定結果）を与える。
#[test]
fn test_async_linear_static_job_flow() {
    let mut app_sync = App::default();
    app_sync.load_model(crate::sample::portal_frame());
    app_sync.run_linear_static(LoadCaseId(0));
    assert!(app_sync.last_error.is_none(), "{:?}", app_sync.last_error);
    let expected_disp = app_sync.results.as_ref().unwrap().statics[0].1.disp.clone();

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.start_linear_static_job(LoadCaseId(0));
    assert!(app.job.is_some());
    assert_eq!(app.job.as_ref().unwrap().label, "線形静的解析");

    wait_for_job(&mut app);

    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(app.job.is_none());
    let bundle = app.results.as_ref().unwrap();
    let (_, static_once) = bundle
        .statics
        .iter()
        .find(|(k, _)| *k == StaticCaseKey::User(LoadCaseId(0)))
        .expect("線形静的解析結果が格納されるはず");
    assert_eq!(static_once.disp, expected_disp);
    assert_eq!(
        app.last_static,
        Some(StaticKey::Case(StaticCaseKey::User(LoadCaseId(0))))
    );
    assert!(!bundle.member_checks.is_empty());
}

/// `start_combination_job` はバックグラウンドで `run_combination` と同じ結果を与える。
#[test]
fn test_async_combination_job_flow() {
    let combo = squid_n_core::model::LoadCombination {
        name: "G+Kx".into(),
        terms: vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), 1.0)],
    };

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::AddCombination {
            combo: combo.clone(),
        }),
    );

    app.start_combination_job(0);
    assert!(app.job.is_some());
    assert_eq!(app.job.as_ref().unwrap().label, "荷重組合せ解析");

    wait_for_job(&mut app);

    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(app.job.is_none());
    let bundle = app.results.as_ref().unwrap();
    assert_eq!(bundle.combos.len(), 1);
    assert_eq!(bundle.combos[0].0, combo.name);
    assert!(!bundle.member_checks.is_empty());
    assert_eq!(app.last_static, Some(StaticKey::Combo(0)));
}

/// `start_all_combinations_job` はバックグラウンドで `run_all_combinations` と
/// 同じ結果（combos の名前・変位）を与える。決定性のため `threads=1` を明示する。
#[test]
fn test_async_all_combinations_job_flow() {
    let combos = vec![
        squid_n_core::model::LoadCombination {
            name: "G+Kx".into(),
            terms: vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), 1.0)],
        },
        squid_n_core::model::LoadCombination {
            name: "G-Kx".into(),
            terms: vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), -1.0)],
        },
    ];

    let mut app_sync = App::default();
    app_sync.load_model(crate::sample::portal_frame());
    app_sync.analysis_cfg.threads = 1;
    for combo in &combos {
        app_sync.undo.run(
            &mut app_sync.model,
            Box::new(squid_n_edit::AddCombination {
                combo: combo.clone(),
            }),
        );
    }
    app_sync.run_all_combinations();
    assert!(app_sync.last_error.is_none(), "{:?}", app_sync.last_error);

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.analysis_cfg.threads = 1;
    for combo in &combos {
        app.undo.run(
            &mut app.model,
            Box::new(squid_n_edit::AddCombination {
                combo: combo.clone(),
            }),
        );
    }
    app.start_all_combinations_job();
    assert!(app.job.is_some());
    assert_eq!(app.job.as_ref().unwrap().label, "全組合せ一括解析");

    wait_for_job(&mut app);

    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(app.job.is_none());
    let bundle_sync = app_sync.results.as_ref().unwrap();
    let bundle = app.results.as_ref().unwrap();
    assert_eq!(bundle.combos.len(), bundle_sync.combos.len());
    for ((name, res), (name_sync, res_sync)) in bundle.combos.iter().zip(bundle_sync.combos.iter())
    {
        assert_eq!(name, name_sync);
        assert_eq!(res.disp, res_sync.disp);
    }
    assert_eq!(app.last_static, app_sync.last_static);
}

/// `start_seismic_job` はバックグラウンドで `run_seismic` と同じ結果を与える。
#[test]
fn test_async_seismic_job_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    app.start_seismic_job(SeismicDir::X);
    assert!(app.job.is_some());
    assert_eq!(app.job.as_ref().unwrap().label, "地震静的解析");

    wait_for_job(&mut app);

    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(app.job.is_none());
    let bundle = app.results.as_ref().unwrap();
    assert!(bundle
        .statics
        .iter()
        .any(|(k, _)| *k == StaticCaseKey::Seismic(SeismicDir::X)));
    assert_eq!(
        app.last_static,
        Some(StaticKey::Case(StaticCaseKey::Seismic(SeismicDir::X)))
    );
    assert!(!bundle.member_checks.is_empty());
}

/// `start_wind_job` はバックグラウンドで `run_wind` と同じ結果を与える
/// （サンプルの門型ラーメンは Y 方向の風のみ見付け幅を持つ。`test_run_wind_static`
/// と同じ理由）。
#[test]
fn test_async_wind_job_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    app.start_wind_job(SeismicDir::Y);
    assert!(app.job.is_some());
    assert_eq!(app.job.as_ref().unwrap().label, "風荷重静的解析");

    wait_for_job(&mut app);

    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(app.job.is_none());
    let bundle = app.results.as_ref().unwrap();
    assert!(bundle
        .statics
        .iter()
        .any(|(k, _)| *k == StaticCaseKey::Wind(SeismicDir::Y)));
    assert_eq!(
        app.last_static,
        Some(StaticKey::Case(StaticCaseKey::Wind(SeismicDir::Y)))
    );
}

#[test]
fn test_save_and_open_project_roundtrip() {
    let dir = std::env::temp_dir().join("squid_n_app_test_scz");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("roundtrip.scz");

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.staleness.mark_edited();
    app.save_project_to(path.clone());
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(!app.staleness.unsaved_changes);
    assert_eq!(app.project_path.as_ref(), Some(&path));
    // ショートカット保存はダイアログが出ず無反応になるため、成功通知で明示する。
    assert!(
        app.last_notice
            .as_deref()
            .is_some_and(|n| n.contains("保存しました") && n.contains("roundtrip.scz")),
        "{:?}",
        app.last_notice
    );

    let saved_model = app.model.clone();
    let mut app2 = App::default();
    app2.open_project_from(path.clone());
    assert!(app2.last_error.is_none(), "{:?}", app2.last_error);
    assert!(app2.model.eq_ignoring_dofmap(&saved_model));
    assert_eq!(app2.project_path.as_ref(), Some(&path));

    std::fs::remove_file(&path).ok();
}

#[test]
fn test_open_project_missing_file_sets_error() {
    let mut app = App::default();
    app.open_project_from(std::path::PathBuf::from(
        "/nonexistent/dir/does_not_exist.scz",
    ));
    assert!(app.last_error.is_some());
}

#[test]
fn test_export_and_import_stbridge_roundtrip() {
    let dir = std::env::temp_dir().join("squid_n_app_test_stbridge");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("roundtrip.stb");

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    let original = app.model.clone();
    app.export_stbridge_to(path.clone());
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let mut app2 = App::default();
    app2.import_stbridge_from(path.clone());
    // ST-Bridge は支点を持たないため、支点の自動設定の通知だけが出る
    // （それ以外の欠落警告は無い）。
    let msg = app2.last_error.as_deref().unwrap_or("");
    assert!(
        msg.contains("ピン支点に設定"),
        "支点自動設定の通知が出るはず: {msg}"
    );
    assert!(app2.model.validate().is_ok());
    // ST-Bridge プロジェクト(.scz)とは別物なので project_path は更新されない。
    assert!(app2.project_path.is_none());

    // 標準 ST-Bridge は幾何サブセットのため完全一致は求めない（拘束・荷重・材料の
    // E/ν は対象外）が、節点数・部材数・座標・接続関係は保たれる。
    assert_eq!(app2.model.nodes.len(), original.nodes.len());
    assert_eq!(app2.model.elements.len(), original.elements.len());
    for (a, b) in app2.model.nodes.iter().zip(original.nodes.iter()) {
        assert_eq!(a.coord, b.coord);
    }
    for (a, b) in app2.model.elements.iter().zip(original.elements.iter()) {
        assert_eq!(a.kind, b.kind, "要素種別が保たれる");
        assert_eq!(a.nodes, b.nodes, "節点接続が保たれる");
    }

    std::fs::remove_file(&path).ok();
}

#[test]
fn test_export_stbridge_standard_mode_writes_steel_library() {
    let dir = std::env::temp_dir().join("squid_n_app_test_stbridge_std");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("standard.stb");

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.export_stbridge_to(path.clone());
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let xml = std::fs::read_to_string(&path).unwrap();
    // 門型ラーメンのサンプルは鋼 H 断面（柱・梁）を持つため、標準断面要素と
    // 形鋼ライブラリが書き出される。
    assert!(xml.contains("<StbSecColumn_S "), "鋼柱は StbSecColumn_S");
    assert!(xml.contains("<StbSecBeam_S "), "鋼梁は StbSecBeam_S");
    assert!(xml.contains("<StbSecSteel>"), "形鋼ライブラリを出す");
    assert!(
        !xml.contains("<StbSecRaw "),
        "形状を持つ断面は Raw にしない"
    );

    std::fs::remove_file(&path).ok();
}

#[test]
fn test_stbridge_standard_mode_roundtrip_through_app() {
    // 断面形状モードで書き出したファイルを GUI 経路（import_stbridge_from）で
    // 読み戻せる（検証エラーなくモデルが差し替わり、断面形状が復元される）。
    let dir = std::env::temp_dir().join("squid_n_app_test_stbridge_std_rt");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("standard_rt.stb");

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    let n_sections = app.model.sections.len();
    app.export_stbridge_to(path.clone());
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let mut app2 = App::default();
    app2.import_stbridge_from(path.clone());
    // 支点の自動設定の通知以外の警告（欠落・断面未解決など）が無いこと
    // ＝標準モードのファイルを読み戻せることを確認する。
    let msg = app2.last_error.as_deref().unwrap_or("");
    assert!(
        msg.is_empty() || msg.contains("ピン支点に設定"),
        "標準モードのファイルを読み戻せる: {msg}"
    );
    assert!(
        !msg.contains("スキップ") && !msg.contains("破棄"),
        "欠落警告は無いはず: {msg}"
    );
    assert!(app2.model.validate().is_ok());
    assert_eq!(app2.model.sections.len(), n_sections);
    // サンプルは鋼 H 断面のみ。読み戻した断面も H 形鋼として復元される。
    assert!(
        app2.model.sections.iter().all(|s| matches!(
            s.shape,
            Some(squid_n_core::section_shape::SectionShape::SteelH { .. })
        )),
        "断面形状が H 形鋼として復元される"
    );

    std::fs::remove_file(&path).ok();
}

#[test]
fn test_import_stbridge_missing_file_sets_error() {
    let mut app = App::default();
    app.import_stbridge_from(std::path::PathBuf::from(
        "/nonexistent/dir/does_not_exist.stb",
    ));
    assert!(app.last_error.is_some());
}

#[test]
fn test_combination_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());

    let combo = squid_n_core::model::LoadCombination {
        name: "G+Kx".into(),
        terms: vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), 1.0)],
    };
    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::AddCombination { combo }),
    );

    app.run_combination(0);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let bundle = app.results.as_ref().unwrap();
    assert_eq!(bundle.combos.len(), 1);
    assert!(!bundle.member_checks.is_empty());
    assert_eq!(app.last_static, Some(StaticKey::Combo(0)));
}

/// 結果表示の切替 `select_displayed_result`（結果タブのドロップダウン・ナビゲータ共通）:
/// 選んだ組合せへ member_forces（応力図・断面検定が参照）を差し替え、長期/短期区分を
/// 組合せ名から再判定し、断面検定を再実行することを確認する。
#[test]
fn test_select_displayed_result_switches_forces_and_term() {
    use squid_n_design_jp::LoadTerm;

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.analysis_cfg.threads = 1;
    // 長期 DL+LL（重力 LC0 のみ）と短期 DL+LL+EX（地震 LC1 入り）の 2 組合せ。
    for combo in [
        squid_n_core::model::LoadCombination {
            name: "DL + LL".into(),
            terms: vec![(LoadCaseId(0), 1.0)],
        },
        squid_n_core::model::LoadCombination {
            name: "DL + LL + EX".into(),
            terms: vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), 1.0)],
        },
    ] {
        app.undo.run(
            &mut app.model,
            Box::new(squid_n_edit::AddCombination { combo }),
        );
    }
    app.run_all_combinations();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    // 一括解析後は最後の組合せ（短期 DL+LL+EX）が表示対象。
    assert_eq!(app.last_static, Some(StaticKey::Combo(1)));
    assert_eq!(app.design_term, LoadTerm::Short);
    // 長期・短期で部材内力が異なる（表示切替が実質的に効く前提）。
    {
        let b = app.results.as_ref().unwrap();
        assert!(!b.member_forces.is_empty());
        assert_ne!(
            b.combos[0].1.member_forces[0].1.at,
            b.combos[1].1.member_forces[0].1.at
        );
    }

    // 長期組合せ（Combo(0)）へ表示切替 → focus/last/member_forces/design_term が長期へ。
    app.select_displayed_result(StaticKey::Combo(0));
    assert_eq!(app.nav.focus_result, Some(StaticKey::Combo(0)));
    assert_eq!(app.last_static, Some(StaticKey::Combo(0)));
    assert_eq!(app.design_term, LoadTerm::Long);
    {
        let b = app.results.as_ref().unwrap();
        assert_eq!(b.member_forces[0].1.at, b.combos[0].1.member_forces[0].1.at);
    }

    // 短期組合せ（Combo(1)）へ戻す → design_term が短期へ、member_forces も一致。
    app.select_displayed_result(StaticKey::Combo(1));
    assert_eq!(app.design_term, LoadTerm::Short);
    {
        let b = app.results.as_ref().unwrap();
        assert_eq!(b.member_forces[0].1.at, b.combos[1].1.member_forces[0].1.at);
    }

    // 存在しないキーは no-op（表示対象は変わらない）。
    app.select_displayed_result(StaticKey::Combo(99));
    assert_eq!(app.last_static, Some(StaticKey::Combo(1)));
}

/// `run_all_combinations` は個別に `run_combination` を実行した場合と
/// 同じ結果（combos の名前・変位）を与える（並列/一括経路と単発経路の一致確認）。
/// 決定性のため `threads=1`（Deterministic）を明示する。
#[test]
fn test_run_all_combinations_matches_individual_runs() {
    let combos = vec![
        squid_n_core::model::LoadCombination {
            name: "G+Kx".into(),
            terms: vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), 1.0)],
        },
        squid_n_core::model::LoadCombination {
            name: "G-Kx".into(),
            terms: vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), -1.0)],
        },
    ];

    let mut app_batch = App::default();
    app_batch.load_model(crate::sample::portal_frame());
    app_batch.analysis_cfg.threads = 1;
    for combo in combos.clone() {
        app_batch.undo.run(
            &mut app_batch.model,
            Box::new(squid_n_edit::AddCombination { combo }),
        );
    }
    app_batch.run_all_combinations();
    assert!(app_batch.last_error.is_none(), "{:?}", app_batch.last_error);

    let mut app_each = App::default();
    app_each.load_model(crate::sample::portal_frame());
    app_each.analysis_cfg.threads = 1;
    for combo in combos {
        app_each.undo.run(
            &mut app_each.model,
            Box::new(squid_n_edit::AddCombination { combo }),
        );
    }
    app_each.run_combination(0);
    assert!(app_each.last_error.is_none(), "{:?}", app_each.last_error);
    app_each.run_combination(1);
    assert!(app_each.last_error.is_none(), "{:?}", app_each.last_error);

    let bundle_batch = app_batch.results.as_ref().unwrap();
    let bundle_each = app_each.results.as_ref().unwrap();
    assert_eq!(bundle_batch.combos.len(), bundle_each.combos.len());
    for ((name_b, res_b), (name_e, res_e)) in
        bundle_batch.combos.iter().zip(bundle_each.combos.iter())
    {
        assert_eq!(name_b, name_e);
        assert_eq!(res_b.disp, res_e.disp);
    }
    assert_eq!(app_batch.last_static, Some(StaticKey::Combo(1)));
}

/// 荷重組合せが 1 件も無い場合はエラーメッセージを設定し、結果は変更しない。
#[test]
fn test_run_all_combinations_no_combos_is_error() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    assert!(app.model.combinations.is_empty());

    app.run_all_combinations();
    assert!(app.last_error.is_some());
    assert!(app.results.is_none());
}

#[test]
fn test_current_static_priority() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let expected_disp = app.results.as_ref().unwrap().statics[0].1.disp.clone();

    // ナビゲータで存在しない Combo を選択していても last_static にフォールバックする
    app.nav.focus_result = Some(StaticKey::Combo(9));
    let fallback = app
        .current_static()
        .expect("無効な選択時は last_static にフォールバックするはず");
    assert_eq!(fallback.disp, expected_disp);

    // Case を選択すれば該当ケースの結果が返る
    app.nav.focus_result = Some(StaticKey::Case(StaticCaseKey::User(LoadCaseId(0))));
    let by_case = app
        .current_static()
        .expect("Case 選択時は該当ケースの結果が返るはず");
    assert_eq!(by_case.disp, expected_disp);
}

#[test]
fn test_holding_capacity_flow() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());

    // 階が未定義 → Err
    assert!(app.compute_holding_capacity().is_err());

    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    app.run_seismic(SeismicDir::X);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    // プッシュオーバー未実行 → Err
    assert!(app.compute_holding_capacity().is_err());

    app.analysis_cfg.push_steps = 10;
    app.run_pushover();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let (result, story_ranks) = app
        .compute_holding_capacity()
        .expect("前提が揃えば Ok のはず");
    assert_eq!(result.stories.len(), 1);
    assert!(result.stories[0].qun > 0.0);
    // Qu はプッシュオーバー最終点の層せん断（capacity_curve.story_shear）から取得される。
    assert!(result.stories[0].qu > 0.0, "{}", result.stories[0].qu);
    // design_rank_auto=false（既定）→ 全層フォールバック（選択値 design_rank）。
    assert_eq!(story_ranks, vec![app.design_rank]);
    assert!(result.member_ranks.is_empty());
}

/// 架構種別が「S ブレース」なのに筋かい部材を検出できない場合、βu を算定できないため
/// βu=0（純ラーメン）の行を使ってはならない（Ds を過小評価する）。架構種別別の
/// Ds 表へフォールバックし、その旨のフラグが立つことを確認する。
#[test]
fn test_holding_capacity_falls_back_when_brace_undetected() {
    use squid_n_design_jp::secondary::holding_capacity::{ds_value, FrameType, MemberRank};

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame()); // 筋かいの無いラーメン
    app.generate_stories_action();
    app.run_seismic(SeismicDir::X);
    app.analysis_cfg.push_steps = 10;
    app.run_pushover();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    app.design_rank_auto = false;
    app.design_rank = MemberRank::FA;
    app.design_frame = FrameType::SteelBrace;
    let (result, _) = app.compute_holding_capacity().expect("Ok のはず");

    assert!(
        app.ds_beta_u_unavailable,
        "筋かい未検出なら βu 算定不可のフラグが立つはず"
    );
    // 純ラーメンの行（S造 FA=0.25）ではなく、S ブレースの行（FA=0.30）が使われる。
    let expected = ds_value(FrameType::SteelBrace, MemberRank::FA);
    assert!((expected - 0.30).abs() < 1e-9);
    assert!(
        (result.stories[0].ds - expected).abs() < 1e-9,
        "Ds={} は架構種別別の値 {} であるべき（βu=0 行の 0.25 ではない）",
        result.stories[0].ds,
        expected
    );
}

/// UI-13: `design_rank_auto = true` で鋼部材の幅厚比から部材ランクを自動判定する。
/// portal_frame の柱(H-300x300x10x15)・梁(H-400x200x8x13)を、構造規定の
/// 幅厚比表（鋼構造設計規準、`s_member_rank_by_kihon`）で
/// 手計算した結果と一致することを確認する。
/// - 柱(SN400B=400級): フランジ 150/15=10.0（>9.5 → FB）、ウェブ 27.0（≦43 → FA）→ FB
/// - 梁(400級): フランジ 100/13≈7.69（≦9 → FA）、ウェブ 46.75（≦60 → FA）→ FA
#[test]
fn test_holding_capacity_rank_auto_from_width_thickness() {
    use squid_n_design_jp::secondary::holding_capacity::MemberRank;
    use squid_n_design_jp::secondary::member_rank::worst_rank;
    use squid_n_design_jp::secondary::width_thickness::{s_member_rank_by_kihon, SteelMemberUse};

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    app.run_seismic(SeismicDir::X);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    app.analysis_cfg.push_steps = 10;
    app.run_pushover();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    app.design_rank_auto = true;
    let (result, story_ranks) = app
        .compute_holding_capacity()
        .expect("shape 付き鋼断面があれば Ok のはず");

    assert!(
        !result.member_ranks.is_empty(),
        "鋼部材の幅厚比からランクが算定されているはず"
    );

    // 柱 H-300x300x10x15（SN400B）: フランジ 10.0 → FB が支配。
    let col_rank = s_member_rank_by_kihon(
        app.model.sections[0].shape.as_ref().unwrap(),
        SteelMemberUse::Column,
        "SN400B",
    )
    .unwrap();
    assert_eq!(col_rank, MemberRank::FB);
    // 梁 H-400x200x8x13（SN400B）: フランジ・ウェブとも FA。
    let beam_rank = s_member_rank_by_kihon(
        app.model.sections[1].shape.as_ref().unwrap(),
        SteelMemberUse::Beam,
        "SN400B",
    )
    .unwrap();
    assert_eq!(beam_rank, MemberRank::FA);

    for (elem_id, rank) in &result.member_ranks {
        let expected = if elem_id.0 == 2 { beam_rank } else { col_rank };
        assert_eq!(
            *rank, expected,
            "ElemId({}) のランクが手計算値と一致しません",
            elem_id.0
        );
    }
    // 唯一の層の代表ランクは柱・梁のうち最悪値（FD 寄り）。
    assert_eq!(story_ranks.len(), 1);
    assert_eq!(story_ranks[0], worst_rank(&[col_rank, beam_rank]).unwrap());
}

/// SectionShape::RcRect の配筋情報から `rc_capacity_input_from_rect` で
/// `RcCapacityInput` を組み立てる経路そのものを検証する（RcRect→入力構築）。
/// 得られた入力から `rc_qsu_simple`/`rc_qmu_simple` → `rc_member_rank` の結果が、
/// 同じ式を独立に書き下した手計算と一致することを確認する。
#[test]
fn test_rc_capacity_input_from_rect_matches_handcalc() {
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::Material;
    use squid_n_core::section_shape::{BarSet, RcRebar, ShearBar};
    use squid_n_design_jp::secondary::member_rank::{rc_member_rank, RankCriteria};
    use squid_n_design_jp::secondary::rc_capacity::{rc_qmu_simple, rc_qsu_simple};

    let b = 400.0;
    let d = 600.0;
    let rebar = RcRebar {
        main_grade: None,
        main_x: BarSet {
            count: 8,
            dia: 22.0,
            layers: 2,
        },
        main_y: BarSet {
            count: 4,
            dia: 19.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 150.0,
            legs: 2,
            grade: Some("SD295A".into()),
        },
    };
    // 材料名は "FC24"（is_steel が false になる、かつ fc 設定あり）を想定。
    let mat = Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "FC24".into(),
        young: 23000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None, // 未設定 → sigma_y は 345(SD345相当)にフォールバックするはず
    };
    let clear_span = 3000.0;

    let input = rc_capacity_input_from_rect(b, d, &rebar, &mat, clear_span)
        .expect("fc が設定されているので Some のはず");

    // 変換規則の確認: at=main_x総断面積の半分、d_eff=d-cover-dia/2、
    // pw=せん断補強筋断面積・組数/(b・ピッチ)、sigma_y は fy 未設定なので 345 固定、
    // sigma_wy は常に 295 固定。
    let main_area = 8.0 * std::f64::consts::PI / 4.0 * 22.0 * 22.0;
    let at_expected = main_area / 2.0;
    let d_eff_expected = 600.0 - 40.0 - 22.0 / 2.0;
    let shear_area = std::f64::consts::PI / 4.0 * 10.0 * 10.0 * 2.0;
    let pw_expected = shear_area / (400.0 * 150.0);
    assert!((input.at - at_expected).abs() < 1e-9);
    assert!((input.d_eff - d_eff_expected).abs() < 1e-9);
    assert!((input.pw - pw_expected).abs() < 1e-12);
    assert_eq!(input.sigma_y, 345.0);
    assert_eq!(input.sigma_wy, 295.0);
    assert_eq!(input.fc, 24.0);
    assert_eq!(input.clear_span, clear_span);

    // rc_qsu_simple/rc_qmu_simple の結果を、式を独立に書き下した手計算と照合する。
    // Mu = 0.9·at·σy·d（技術基準解説書 P.623。d = 有効せい）。
    let j = 7.0 * d_eff_expected / 8.0;
    let mu_handcalc = 0.9 * at_expected * 345.0 * d_eff_expected;
    let qmu_handcalc = 2.0 * mu_handcalc / clear_span;
    let pt = 100.0 * at_expected / (400.0 * d_eff_expected);
    let shear_span_ratio = (clear_span / (2.0 * d_eff_expected)).clamp(1.0, 3.0);
    let pw_clamped = pw_expected.clamp(0.0, 0.012);
    let concrete_term = 0.068 * pt.powf(0.23) * (24.0 + 18.0) / (shear_span_ratio + 0.12);
    let hoop_term = 0.85 * (pw_clamped * 295.0_f64).sqrt();
    let qsu_handcalc = (concrete_term + hoop_term) * 400.0 * j;

    let qmu = rc_qmu_simple(&input);
    let qsu = rc_qsu_simple(&input);
    assert!(
        (qmu - qmu_handcalc).abs() < 1e-3,
        "Qmu={} vs handcalc={}",
        qmu,
        qmu_handcalc
    );
    assert!(
        (qsu - qsu_handcalc).abs() < 1e-3,
        "Qsu={} vs handcalc={}",
        qsu,
        qsu_handcalc
    );

    let rank = rc_member_rank(qsu, qmu, &RankCriteria::default());
    let rank_handcalc = rc_member_rank(qsu_handcalc, qmu_handcalc, &RankCriteria::default());
    assert_eq!(rank, rank_handcalc);
    // Qsu/Qmu ≈ 2.12（曲げ降伏が十分先行する健全な配筋）なので FA になるはず。
    assert_eq!(
        rank,
        squid_n_design_jp::secondary::holding_capacity::MemberRank::FA
    );
}

/// UI-13(RC): SectionShape::RcRect + fc 付き材料（コンクリート、is_steel=false）を
/// 持つ小さな門型ラーメンを組み、rank-auto で member_ranks に RC 部材のランクが入り、
/// `rc_capacity_input_from_rect` → `rc_qsu_simple`/`rc_qmu_simple` → `rc_member_rank`
/// の手計算と一致することを確認する。
#[test]
fn test_holding_capacity_rank_auto_rc_rect_from_shape() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material,
        MemberLoad, MemberLoadKind, Model, NodalLoad, Node,
    };
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};
    use squid_n_design_jp::secondary::ds_group::{rc_beam_type, rc_column_type};
    use squid_n_design_jp::secondary::holding_capacity::MemberRank;

    let rebar = RcRebar {
        main_grade: Some("SD345".into()),
        main_x: BarSet {
            count: 8,
            dia: 22.0,
            layers: 2,
        },
        main_y: BarSet {
            count: 4,
            dia: 19.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 150.0,
            legs: 2,
            grade: None,
        },
    };
    let rc_shape = SectionShape::RcRect {
        b: 400.0,
        d: 600.0,
        rebar: rebar.clone(),
    };

    let mut model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [4000.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(2),
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(3),
                coord: [4000.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ],
        sections: vec![rc_shape.to_section(SectionId(0), "RC-400x600".into())],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "FC24".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        }],
        ..Default::default()
    };
    let members = [
        (0u32, 0u32, 2u32, [1.0, 0.0, 0.0]),
        (1, 1, 3, [1.0, 0.0, 0.0]),
        (2, 2, 3, [0.0, 0.0, 1.0]),
    ];
    for (id, i, j, ref_vector) in members {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis { ref_vector },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(0),
        name: "長期".into(),
        nodal: Vec::new(),
        member: vec![MemberLoad {
            elem: ElemId(2),
            dir: [0.0, 0.0, -1.0],
            kind: MemberLoadKind::Distributed {
                a: 0.0,
                b: 4000.0,
                w1: 10.0,
                w2: 10.0,
            },
        }],
    });
    model.load_cases.push(LoadCase {
        kind: Default::default(),
        id: LoadCaseId(1),
        name: "地震X".into(),
        nodal: vec![
            NodalLoad {
                node: NodeId(2),
                values: [20000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            },
            NodalLoad {
                node: NodeId(3),
                values: [20000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            },
        ],
        member: Vec::new(),
    });

    let mut app = App::default();
    app.load_model(model);
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    app.run_seismic(SeismicDir::X);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    // RC の簡易断面・fy 未設定材料はヒンジ耐力の既定値(鋼材既定 235N/mm²)を用いる
    // 都合上、既定の push_max_disp=500mm では機構形成後に特異行列となり得るため、
    // 微小変位のみを対象とする(ここではランク判定経路の配線確認が目的で、
    // 崩壊形の精算は対象外)。
    app.analysis_cfg.push_steps = 3;
    app.analysis_cfg.push_use_max_disp = true;
    app.analysis_cfg.push_max_disp = 3.0;
    app.analysis_cfg.push_use_drift_angle = false;
    app.run_pushover();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    app.design_rank_auto = true;
    let (result, _story_ranks) = app
        .compute_holding_capacity()
        .expect("RC 矩形 + fc 付き材料があれば Ok のはず");

    assert!(
        !result.member_ranks.is_empty(),
        "RC 部材(RcRect+fc)のせん断余裕度からランクが算定されているはず"
    );

    // 告示の RC 部材種別（多変数表）を、プッシュオーバー終局時の応答（τu・σ0）から
    // 手計算で再現して照合する。
    //
    // 断面 400x600、主筋 main_x=8-D22（at=総断面積の半分）、かぶり40、
    // 柱の内法 h0=3000（節点間距離。剛域・フェイス無し）、梁は 4000。
    // 目標変位 3mm の微小変位状態のため τu・σ0 はいずれも小さく、
    //   柱: h0/D = 3000/600 = 5.0 ≧ 2.5、pt = 100·at/(b·d_eff) ≈ 0.69% ≦ 0.8、
    //       σ0/Fc・τu/Fc ともに FA 限界以下 → FA
    //   梁: τu/Fc ≦ 0.15 → FA
    // となる。せん断先行（Qsu < Qmu）でもないため脆性 FD にも該当しない。
    let mat = &app.model.materials[0];
    let fc = mat.fc.expect("fc 設定済み");
    let resp: std::collections::HashMap<ElemId, _> = app
        .results
        .as_ref()
        .unwrap()
        .pushover
        .as_ref()
        .unwrap()
        .member_response
        .iter()
        .map(|r| (r.elem, *r))
        .collect();
    let gross = 400.0 * 600.0;

    let expected_rank_for = |elem_id: ElemId, clear_span: f64, is_column: bool| {
        let input = rc_capacity_input_from_rect(400.0, 600.0, &rebar, mat, clear_span)
            .expect("fc 設定済みなので Some");
        let r = resp.get(&elem_id).expect("終局時応答があるはず");
        let tau_over_fc = (r.shear_strong / gross) / fc;
        if is_column {
            let sigma0_over_fc = (r.axial / gross) / fc;
            let pt_percent = 100.0 * input.at / (400.0 * input.d_eff);
            rc_column_type(
                clear_span / 600.0,
                sigma0_over_fc,
                pt_percent,
                tau_over_fc,
                false,
            )
        } else {
            rc_beam_type(tau_over_fc, false)
        }
    };
    let col0_rank = expected_rank_for(ElemId(0), 3000.0, true);
    let col1_rank = expected_rank_for(ElemId(1), 3000.0, true);
    let beam_rank = expected_rank_for(ElemId(2), 4000.0, false);

    // 微小変位状態では全部材 FA になる（応力度がいずれも FA 限界以下）。
    assert_eq!(col0_rank, MemberRank::FA);
    assert_eq!(beam_rank, MemberRank::FA);

    for (elem_id, rank) in &result.member_ranks {
        let expected = match elem_id.0 {
            2 => beam_rank,
            1 => col1_rank,
            _ => col0_rank,
        };
        assert_eq!(
            *rank, expected,
            "ElemId({}) のランクが告示表による手計算値と一致しません",
            elem_id.0
        );
    }
}

/// `rc_sigma_0_from_gravity_or_last_static`: 圧縮軸力から σ0 が正しく算定されることを、
/// 実際に静的解析を実行して確認する。
///
/// モデル: 鉛直片持ち柱（節点0=基部, 固定, z=0 / 節点1=先端, 自由, z=3000）に
/// RC矩形断面 400x600 を設定し、先端節点へ下向き(圧縮)集中荷重 P=100,000N を
/// 与える。軸力のみが生じる単純な釣合いなので、内力の軸力の大きさは
/// 弾性係数・断面性能によらず厳密に P と一致する。
///
/// 符号規約の確認: squid-n-solver::linear::test_linear_static_axial_cantilever
/// で N=+1000N(引張)のとき forces.at[0].1[0]≈-1000 であることを確認済みなので、
/// 圧縮(先端を下向きに押す)では forces.at[0].1[0]≈+P（正）になるはず。
#[test]
fn test_rc_sigma_0_from_compression_axial_force() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material, Model,
        NodalLoad, Node,
    };
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    let b = 400.0;
    let d = 600.0;
    let rebar = RcRebar {
        main_grade: None,
        main_x: BarSet {
            count: 8,
            dia: 22.0,
            layers: 2,
        },
        main_y: BarSet {
            count: 4,
            dia: 19.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 150.0,
            legs: 2,
            grade: None,
        },
    };
    let rc_shape = SectionShape::RcRect {
        b,
        d,
        rebar: rebar.clone(),
    };

    let p = 100_000.0; // 圧縮荷重 [N]
    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ],
        sections: vec![rc_shape.to_section(SectionId(0), "RC-400x600".into())],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "FC24".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        }],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        load_cases: vec![LoadCase {
            kind: Default::default(),
            id: LoadCaseId(0),
            name: "圧縮".into(),
            nodal: vec![NodalLoad {
                node: NodeId(1),
                values: [0.0, 0.0, -p, 0.0, 0.0, 0.0],
            }],
            member: Vec::new(),
        }],
        ..Default::default()
    };

    let mut app = App::default();
    app.load_model(model);
    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let member_forces = &app.results.as_ref().unwrap().member_forces;
    let (_, mf) = member_forces
        .iter()
        .find(|(id, _)| *id == ElemId(0))
        .expect("elem 0 の内力があるはず");
    let n_raw = mf.at.first().expect("eval_sections[0] があるはず").1[0];
    // 軸力は引張正の部材内力なので、圧縮 P に対して n_raw = -P となる。
    assert!(
        (n_raw + p).abs() < 1e-6,
        "n_raw={} (expected {})",
        n_raw,
        -p
    );

    let statics = &app.results.as_ref().unwrap().statics;
    let gravity_lc = app.model.load_cases.first().map(|c| c.id);
    let sigma_0 =
        rc_sigma_0_from_gravity_or_last_static(statics, member_forces, gravity_lc, ElemId(0), b, d);
    let expected_sigma_0 = p / (b * d);
    assert!(
        (sigma_0 - expected_sigma_0).abs() < 1e-9,
        "sigma_0={} expected={}",
        sigma_0,
        expected_sigma_0
    );
}

/// `rc_sigma_0_from_gravity_or_last_static`: 先頭荷重ケース(gravity_lc)の静的解析結果が
/// `bundle.statics` にあれば、最後に実行した(かつ結果が異なる)静的解析ではなく
/// 先頭荷重ケースの結果が優先されることを確認する。
///
/// モデル: `test_rc_sigma_0_from_compression_axial_force` と同じ片持ち柱に、
/// 先頭荷重ケース(id=0,"長期")として圧縮荷重 P1、2番目のケース(id=1,"地震")として
/// 引張荷重 P2 を設定する。両ケースをこの順に実行すると
/// `bundle.member_forces`(=最後に実行したケース)は引張(id=1)の結果になり、
/// これをそのまま使うと σ0=0(引張は 0 とみなす安全側処理)になってしまう。
/// 優先順位が正しく効いていれば、`bundle.statics` 内の id=0(長期)の圧縮軸力から
/// σ0=P1/(b・D) (>0) が算定される。
#[test]
fn test_rc_sigma_0_prefers_gravity_load_case_over_last_static() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material, Model,
        NodalLoad, Node,
    };
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    let b = 400.0;
    let d = 600.0;
    let rebar = RcRebar {
        main_grade: None,
        main_x: BarSet {
            count: 8,
            dia: 22.0,
            layers: 2,
        },
        main_y: BarSet {
            count: 4,
            dia: 19.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 150.0,
            legs: 2,
            grade: None,
        },
    };
    let rc_shape = SectionShape::RcRect {
        b,
        d,
        rebar: rebar.clone(),
    };

    let p1 = 100_000.0; // 先頭ケース(長期)の圧縮荷重 [N]
    let p2 = 60_000.0; // 2番目のケース(地震想定)の引張荷重 [N]
    let model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ],
        sections: vec![rc_shape.to_section(SectionId(0), "RC-400x600".into())],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "FC24".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        }],
        elements: vec![ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }],
        load_cases: vec![
            LoadCase {
                kind: Default::default(),
                id: LoadCaseId(0),
                name: "長期".into(),
                nodal: vec![NodalLoad {
                    node: NodeId(1),
                    values: [0.0, 0.0, -p1, 0.0, 0.0, 0.0], // 下向き=圧縮
                }],
                member: Vec::new(),
            },
            LoadCase {
                kind: Default::default(),
                id: LoadCaseId(1),
                name: "地震".into(),
                nodal: vec![NodalLoad {
                    node: NodeId(1),
                    values: [0.0, 0.0, p2, 0.0, 0.0, 0.0], // 上向き=引張
                }],
                member: Vec::new(),
            },
        ],
        ..Default::default()
    };

    let mut app = App::default();
    app.load_model(model);
    // 先頭ケース(長期,圧縮)→2番目のケース(地震,引張)の順に実行し、
    // 「最後に実行した静的解析結果」は引張(id=1)になるようにする。
    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    app.run_linear_static(LoadCaseId(1));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let bundle = app.results.as_ref().unwrap();
    // 最後に実行した静的解析結果(bundle.member_forces)は引張なので、
    // これをそのまま使うと σ0=0 になってしまうことの確認(比較対象)。
    let sigma_0_last_only =
        rc_sigma_0_from_gravity_or_last_static(&[], &bundle.member_forces, None, ElemId(0), b, d);
    assert_eq!(sigma_0_last_only, 0.0, "引張のみなら σ0=0 のはず(比較対象)");

    // 優先順位が正しく効いていれば、先頭ケース(長期,id=0)の圧縮軸力から
    // σ0=P1/(b・D) (>0) が算定される。
    let gravity_lc = app.model.load_cases.first().map(|c| c.id);
    assert_eq!(gravity_lc, Some(LoadCaseId(0)));
    let sigma_0 = rc_sigma_0_from_gravity_or_last_static(
        &bundle.statics,
        &bundle.member_forces,
        gravity_lc,
        ElemId(0),
        b,
        d,
    );
    let expected_sigma_0 = p1 / (b * d);
    assert!(
        (sigma_0 - expected_sigma_0).abs() < 1e-9,
        "sigma_0={} expected={}(先頭ケースの圧縮軸力が優先されるはず)",
        sigma_0,
        expected_sigma_0
    );
}

/// Z=0 平面の矩形（4000×6000）+外周4本の梁 + スラブ1枚（TriTrapezoid）を持つモデルを作る。
/// 辺 i = boundary[i] → boundary[(i+1)%4] の順に梁を並べる（refresh_beam_loads の対応付けと一致）。
fn make_slab_test_model() -> squid_n_core::model::Model {
    use squid_n_core::ids::SlabId;
    use squid_n_core::model::{
        AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime,
        LocalAxis, Node, Slab,
    };

    let mk_node = |id: u32, x: f64, y: f64| Node {
        id: NodeId(id),
        coord: [x, y, 0.0],
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    let nodes = vec![
        mk_node(0, 0.0, 0.0),
        mk_node(1, 4000.0, 0.0),
        mk_node(2, 4000.0, 6000.0),
        mk_node(3, 0.0, 6000.0),
    ];
    let mk_beam = |id: u32, i: u32, j: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let elements = vec![
        mk_beam(0, 0, 1),
        mk_beam(1, 1, 2),
        mk_beam(2, 2, 3),
        mk_beam(3, 3, 0),
    ];
    let slab = Slab {
        usage: None,
        edge_supported: None,
        thickness: None,
        kind: Default::default(),
        one_way: None,
        id: SlabId(0),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        joists: vec![],
        loads: vec![AreaLoad {
            kind: "DL".into(),
            value: 0.005,
        }],
        method: DistributionMethod::TriTrapezoid,
    };
    squid_n_core::model::Model {
        nodes,
        elements,
        slabs: vec![slab],
        ..Default::default()
    }
}

#[test]
fn test_refresh_beam_loads_maps_edges_to_members() {
    let model = make_slab_test_model();
    model
        .validate()
        .expect("テストモデルは validate を通るはず");

    let mut app = App {
        model,
        ..App::default()
    };
    app.refresh_beam_loads();

    assert_eq!(app.beam_loads.len(), 4, "外周4辺すべてに荷重が対応付くはず");
    for bl in &app.beam_loads {
        let elem = app
            .model
            .elements
            .iter()
            .find(|e| e.id == bl.elem)
            .expect("beam_loads.elem は実在する部材IDを指すはず");
        assert_eq!(elem.kind, squid_n_core::model::ElementKind::Beam);
        assert!(
            bl.cmq.c_i.abs() > 1e-9 || bl.cmq.q_i.abs() > 1e-9,
            "CMQ が非ゼロのはず: {:?} {:?}",
            bl.cmq.c_i,
            bl.cmq.q_i
        );
    }

    // 梁が1本欠けたモデルでも荷重は捨てず、対応する辺は節点対（Span）として
    // 保持される（slab_load_case_content が主架構へ変換する。二次部材対応）。
    let mut missing = app.model.clone();
    missing.elements.pop();
    app.model = missing;
    app.refresh_beam_loads();
    assert_eq!(app.beam_loads.len(), 4);
    let unresolved: Vec<_> = app
        .beam_loads
        .iter()
        .filter(|bl| {
            matches!(bl.target, squid_n_load::floor::LoadTarget::Span(_))
                && bl.elem == ElemId(u32::MAX)
        })
        .collect();
    assert_eq!(
        unresolved.len(),
        1,
        "欠けた辺は未解決の節点対として残るはず"
    );
}

/// 正方形スラブ（4000×4000）+ 外周4本の梁を持つモデル
/// （`make_slab_test_model` の正方形版。正方形は `TriTrapezoid` で全辺
/// 三角形分布になるため §1.1 のスラブ→荷重ケース同期の検算がしやすい）。
fn make_square_slab_test_model() -> squid_n_core::model::Model {
    use squid_n_core::ids::SlabId;
    use squid_n_core::model::{
        AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime,
        LocalAxis, Node, Slab,
    };

    let mk_node = |id: u32, x: f64, y: f64| Node {
        id: NodeId(id),
        coord: [x, y, 0.0],
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    let nodes = vec![
        mk_node(0, 0.0, 0.0),
        mk_node(1, 4000.0, 0.0),
        mk_node(2, 4000.0, 4000.0),
        mk_node(3, 0.0, 4000.0),
    ];
    let mk_beam = |id: u32, i: u32, j: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let elements = vec![
        mk_beam(0, 0, 1),
        mk_beam(1, 1, 2),
        mk_beam(2, 2, 3),
        mk_beam(3, 3, 0),
    ];
    let slab = Slab {
        usage: None,
        edge_supported: None,
        thickness: None,
        kind: Default::default(),
        one_way: None,
        id: SlabId(0),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        joists: vec![],
        loads: vec![AreaLoad {
            kind: "DL".into(),
            value: 0.005,
        }],
        method: DistributionMethod::TriTrapezoid,
    };
    squid_n_core::model::Model {
        nodes,
        elements,
        slabs: vec![slab],
        ..Default::default()
    }
}

/// レビュー §1.1（最重要）: スラブ荷重が `sync_gravity_load_cases_action` で
/// 「DL」荷重ケースへ実際に書き込まれ、応力解析から参照可能に
/// なることを確認する。正方形スラブは全辺三角形分布（2区間）になるため
/// `MemberLoadKind::Distributed` への変換規則を直接検算できる。
#[test]
fn test_sync_gravity_load_cases_action_square_slab_triangle_distribution() {
    use squid_n_core::model::{LoadCaseKind, MemberLoadKind};

    let model = make_square_slab_test_model();
    model
        .validate()
        .expect("テストモデルは validate を通るはず");
    let mut app = App {
        model,
        ..App::default()
    };

    app.sync_gravity_load_cases_action();

    let case = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == DL_CASE_NAME)
        .expect("DLケースが作られるはず");
    assert_eq!(case.kind, LoadCaseKind::Dead);
    assert_eq!(case.member.len(), 8, "4辺 × 2区間（三角形分布）= 8件");
    assert!(case.nodal.is_empty(), "小梁が無いので節点荷重は空のはず");

    // 各梁にちょうど2区間ずつ入っていることを確認
    for elem_id in 0..4u32 {
        let n_segs = case
            .member
            .iter()
            .filter(|m| m.elem == ElemId(elem_id))
            .count();
        assert_eq!(n_segs, 2, "梁#{elem_id} には三角形分布の2区間が入るはず");
        for m in case.member.iter().filter(|m| m.elem == ElemId(elem_id)) {
            assert_eq!(m.dir, [0.0, 0.0, -1.0], "作用方向は鉛直下向き固定のはず");
        }
    }

    // 鉛直合計 = w × 面積（保存則）
    let total: f64 = case
        .member
        .iter()
        .map(|m| match m.kind {
            MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
            MemberLoadKind::Point { p, .. } => p,
        })
        .sum();
    let expected = 0.005 * 4000.0 * 4000.0;
    assert!(
        (total - expected).abs() < 1e-6,
        "total={total} expected={expected}"
    );

    // 再同期しても重複しない（全置換）
    app.sync_gravity_load_cases_action();
    let cases: Vec<_> = app
        .model
        .load_cases
        .iter()
        .filter(|lc| lc.name == DL_CASE_NAME)
        .collect();
    assert_eq!(cases.len(), 1, "再同期でケースが重複してはいけない");
    assert_eq!(cases[0].member.len(), 8, "再同期で荷重が重複してはいけない");

    // undo で元に戻る（新規作成だったケースが丸ごと消える）
    app.undo.undo(&mut app.model);
    assert!(
        !app.model
            .load_cases
            .iter()
            .any(|lc| lc.name == DL_CASE_NAME),
        "undo で「DL」ケースが消えるはず"
    );
}

/// 床 Phase A-2: 用途（`SlabUsage`）を設定したスラブは、`sync_gravity_load_cases_action`
/// で固定荷重（DL・「DL」kind=Dead）と積載荷重（LL・「床積載(自動)」
/// kind=Live）の 2 ケースに分離されることを確認する。LL は令別表第1 の
/// **骨組用**積載（事務室=1800 N/m²）を用い、DL とは独立に総和保存する。
#[test]
fn test_sync_gravity_load_cases_action_separates_dead_and_live() {
    use squid_n_core::model::{LoadCaseKind, MemberLoadKind, SlabUsage};

    let mut model = make_square_slab_test_model();
    // 事務室用途を設定（骨組用 LL = 1800 N/m² = 1.8e-3 N/mm²）。
    model.slabs[0].usage = Some(SlabUsage::Office);
    model
        .validate()
        .expect("テストモデルは validate を通るはず");
    let mut app = App {
        model,
        ..App::default()
    };

    app.sync_gravity_load_cases_action();

    fn sum_vertical(model: &squid_n_core::model::Model, name: &str) -> f64 {
        model
            .load_cases
            .iter()
            .find(|lc| lc.name == name)
            .map(|c| {
                c.member
                    .iter()
                    .map(|m| match m.kind {
                        MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
                        MemberLoadKind::Point { p, .. } => p,
                    })
                    .sum()
            })
            .unwrap_or(0.0)
    }

    // DL ケース: 従来どおり loads(0.005) を分配。
    let dl = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == DL_CASE_NAME)
        .expect("DLケースが作られるはず");
    assert_eq!(dl.kind, LoadCaseKind::Dead);
    let area = 4000.0 * 4000.0;
    assert!((sum_vertical(&app.model, DL_CASE_NAME) - 0.005 * area).abs() < 1e-6);

    // LL ケース: 骨組用積載 1.8e-3 N/mm² を分配（DL とは別ケース・kind=Live）。
    let ll = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == LL_FRAME_CASE_NAME)
        .expect("LL(架構用)ケースが作られるはず");
    assert_eq!(ll.kind, LoadCaseKind::Live);
    assert!((sum_vertical(&app.model, LL_FRAME_CASE_NAME) - 1.8e-3 * area).abs() < 1e-6);

    // 用途を外すと LL ケースは空同期され、寄与が無くなる（新規なら作られない）。
    app.model.slabs[0].usage = None;
    app.sync_gravity_load_cases_action();
    assert!(
        (sum_vertical(&app.model, LL_FRAME_CASE_NAME)).abs() < 1e-12,
        "用途を外したら積載寄与は 0 になるはず"
    );
}

/// 床 Phase F-3b: 交差小梁スラブは床格子サブモデルの支点反力を大梁接続点へ渡す。
/// 支点反力総和は平行小梁モデルの小梁反力総和（w·Σspacing·L）と一致し（総和保存）、
/// 実部材化された小梁を含むスラブは二重計上回避のため対象外（None）になる。
#[test]
fn test_slab_grillage_node_reactions_total_and_gate() {
    use squid_n_core::ids::SlabId;
    use squid_n_core::model::{
        AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime,
        JoistLine, LocalAxis, Node, Section, Slab,
    };

    let mk_node = |id: u32, x: f64, y: f64| Node {
        id: NodeId(id),
        coord: [x, y, 0.0],
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    let mk_beam = |id: u32, i: u32, j: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let section = Section {
        id: SectionId(0),
        name: "H".into(),
        area: 10000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e6,
        depth: 400.0,
        width: 200.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    };
    let spacing = 2000.0_f64;
    let slab = Slab {
        usage: None,
        edge_supported: None,
        thickness: None,
        kind: Default::default(),
        one_way: None,
        id: SlabId(0),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        joists: vec![
            JoistLine {
                dir: [0.0, 1.0],
                spacing,
                support: [NodeId(4), NodeId(5)], // 縦（x=2000）
                section: Some(SectionId(0)),
                pinned_onto: None,
            },
            JoistLine {
                dir: [1.0, 0.0],
                spacing,
                support: [NodeId(6), NodeId(7)], // 横（y=2000）
                section: Some(SectionId(0)),
                pinned_onto: None,
            },
        ],
        loads: vec![AreaLoad {
            kind: "DL".into(),
            value: 0.005,
        }],
        method: DistributionMethod::TriTrapezoid,
    };
    let model = squid_n_core::model::Model {
        nodes: vec![
            mk_node(0, 0.0, 0.0),
            mk_node(1, 4000.0, 0.0),
            mk_node(2, 4000.0, 4000.0),
            mk_node(3, 0.0, 4000.0),
            mk_node(4, 2000.0, 0.0),
            mk_node(5, 2000.0, 4000.0),
            mk_node(6, 0.0, 2000.0),
            mk_node(7, 4000.0, 2000.0),
        ],
        elements: vec![
            mk_beam(0, 0, 1),
            mk_beam(1, 1, 2),
            mk_beam(2, 2, 3),
            mk_beam(3, 3, 0),
        ],
        sections: vec![section],
        slabs: vec![slab],
        ..Default::default()
    };
    model.validate().expect("交差小梁モデルは validate を通る");
    let mut app = App {
        model,
        ..App::default()
    };

    let w = 0.005_f64;
    let beam_map = app.beam_elem_map();
    let reactions = app
        .slab_grillage_node_reactions(&app.model.slabs[0], w, &beam_map)
        .expect("交差格子の支点反力が得られるはず");
    // 4 支点（N4..N7）へ配分。
    assert_eq!(reactions.len(), 4, "支点は4節点");
    let total: f64 = reactions.iter().map(|(_, r)| r).sum();
    let expected = w * spacing * 4000.0 * 2.0; // w·spacing·L × 2本
    assert!(
        (total - expected).abs() / expected < 1e-6,
        "格子支点反力総和={total} 期待(平行モデル小梁反力総和)={expected}"
    );

    // 実部材化された小梁を含むと二重計上回避のため None（本体 FEM が伝達）。
    app.model.elements.push(mk_beam(4, 4, 5)); // N4-N5 に実 Beam
    let beam_map = app.beam_elem_map();
    assert!(
        app.slab_grillage_node_reactions(&app.model.slabs[0], w, &beam_map)
            .is_none(),
        "実部材化小梁を含むスラブは格子荷重の対象外（None）"
    );

    // 分配法が小梁二段階伝達（三角/一方向）でないスラブは、distribute_slab_w が
    // 小梁点反力を出さず全面積を境界へ Edge 分配するため、格子反力を上乗せすると
    // 二重計上になる。この場合は None（既存挙動を維持）でなければならない。
    app.model.elements.pop(); // 実 Beam を戻す（他条件は満たす）。
    app.model.slabs[0].method = DistributionMethod::TributaryArea;
    let beam_map = app.beam_elem_map();
    assert!(
        app.slab_grillage_node_reactions(&app.model.slabs[0], w, &beam_map)
            .is_none(),
        "分配法が三角/一方向でないスラブは格子荷重の対象外（二重計上回避）"
    );
}

/// 床 Phase A-3: 用途を設定したスラブは地震用積載（LiveSeismic）ケースも同期され、
/// 地震用重量の重力ケース選択（`gravity_cases_for_seismic_weight`）が
/// 骨組用 Live ではなく地震用 LiveSeismic を採用することを確認する
/// （令85条1項の地震用値〔事務室=800 N/m²〕を地震用重量に用いる）。
#[test]
fn test_sync_gravity_load_cases_action_seismic_live_case() {
    use squid_n_core::model::{LoadCaseKind, MemberLoadKind, SlabUsage};

    let mut model = make_square_slab_test_model();
    model.slabs[0].usage = Some(SlabUsage::Office);
    model
        .validate()
        .expect("テストモデルは validate を通るはず");
    let mut app = App {
        model,
        ..App::default()
    };
    app.sync_gravity_load_cases_action();

    let sum_vertical = |name: &str| -> f64 {
        app.model
            .load_cases
            .iter()
            .find(|lc| lc.name == name)
            .map(|c| {
                c.member
                    .iter()
                    .map(|m| match m.kind {
                        MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
                        MemberLoadKind::Point { p, .. } => p,
                    })
                    .sum()
            })
            .unwrap_or(0.0)
    };

    let area = 4000.0 * 4000.0;
    // 地震用積載ケース: 地震用値 800 N/m² = 8e-4 N/mm²。
    let ls = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == LL_SEISMIC_CASE_NAME)
        .expect("LL(地震用)ケースが作られるはず");
    assert_eq!(ls.kind, LoadCaseKind::LiveSeismic);
    assert!((sum_vertical(LL_SEISMIC_CASE_NAME) - 8e-4 * area).abs() < 1e-6);

    // 地震用重量の重力ケース: DL(床荷重) と LiveSeismic(床地震用積載) を含み、
    // 骨組用 Live(床積載) は含まない（LiveSeismic 優先）。
    let dl_id = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == DL_CASE_NAME)
        .unwrap()
        .id;
    let live_id = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == LL_FRAME_CASE_NAME)
        .unwrap()
        .id;
    let ls_id = ls.id;
    let gravity = gravity_cases_for_seismic_weight(&app.model);
    assert!(gravity.contains(&dl_id), "DL(床荷重)は地震用重量に含むはず");
    assert!(
        gravity.contains(&ls_id),
        "LiveSeismic(床地震用積載)は地震用重量に含むはず"
    );
    assert!(
        !gravity.contains(&live_id),
        "骨組用 Live(床積載)は地震用重量に含めないはず（LiveSeismic 優先）"
    );
}

/// 床 Phase A-3 レビュー指摘: 用途の地震用値が明示的に 0（骨組用のみ正）の場合、
/// 地震用積載(LiveSeismic)ケースは生成されないが、骨組用 Live(床積載)ケースへ
/// フォールバックして地震用重量が過大にならないことを確認する
/// （自動生成の骨組用 Live ケースは地震用重量の代用対象から除外される）。
#[test]
fn test_gravity_cases_excludes_auto_frame_live_when_no_seismic() {
    use squid_n_core::model::SlabUsage;

    let mut model = make_square_slab_test_model();
    // 骨組用のみ正・地震用 0 の用途（serde 由来を想定した異常系）。
    model.slabs[0].usage = Some(SlabUsage::Custom {
        floor: 3e-3,
        frame: 2e-3,
        seismic: 0.0,
    });
    model
        .validate()
        .expect("テストモデルは validate を通るはず");
    let mut app = App {
        model,
        ..App::default()
    };
    app.sync_gravity_load_cases_action();

    // 地震用値 0 なので LiveSeismic ケースは生成されない。
    assert!(
        !app.model
            .load_cases
            .iter()
            .any(|lc| lc.name == LL_SEISMIC_CASE_NAME),
        "地震用値 0 ならLL(地震用)は作られないはず"
    );
    // 骨組用 Live ケースは生成される。
    let live_id = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == LL_FRAME_CASE_NAME)
        .expect("LL(架構用)は作られるはず")
        .id;
    // 地震用重量の重力ケースに、骨組用 Live(床積載)を含めてはならない。
    let gravity = gravity_cases_for_seismic_weight(&app.model);
    assert!(
        !gravity.contains(&live_id),
        "自動生成の骨組用 Live(床積載)は地震用重量にフォールバックしてはならない"
    );
}

/// 床 Phase E: 床の中での小梁・スラブ設計。断面を割り当てた小梁は単純支持梁として
/// 検定され、矩形スラブは一方向版として設計曲げ・必要鉄筋量が算定される。
#[test]
fn test_floor_design_checks_joist_and_slab() {
    use squid_n_core::ids::SectionId;
    use squid_n_core::model::{JoistLine, Section, SlabUsage};

    let mut model = make_square_slab_test_model();
    // 事務室用途（床用積載 2900 N/m² = 2.9e-3 N/mm²）＋固定荷重 0.005。
    model.slabs[0].usage = Some(SlabUsage::Office);
    model.slabs[0].thickness = Some(150.0);
    // 鋼小梁の断面（Iy=1e8 mm⁴, せい 400mm → Z=5e5 mm³）。
    model.sections.push(Section {
        id: SectionId(0),
        name: "H-400".into(),
        area: 10000.0,
        iy: 1.0e8,
        iz: 1.0e7,
        j: 1.0e6,
        depth: 400.0,
        width: 200.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    // 対辺の中間節点 N4(2000,0)・N5(2000,4000) を追加し、その間に小梁を架ける
    // （境界大梁で直接結ばれていない＝実部材化されていない現実的な小梁）。
    let mk_mid = |id: u32, x: f64, y: f64| squid_n_core::model::Node {
        id: NodeId(id),
        coord: [x, y, 0.0],
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    model.nodes.push(mk_mid(4, 2000.0, 0.0));
    model.nodes.push(mk_mid(5, 2000.0, 4000.0));
    // 支持 N4(2000,0)–N5(2000,4000)、スパン 4000、負担幅 2000、断面 S0。
    model.slabs[0].joists.push(JoistLine {
        dir: [0.0, 1.0],
        spacing: 2000.0,
        support: [NodeId(4), NodeId(5)],
        section: Some(SectionId(0)),
        pinned_onto: None,
    });
    model.validate().expect("validate");
    let app = App {
        model,
        ..App::default()
    };

    let (joists, slabs) = app.floor_design_checks();
    assert_eq!(joists.len(), 1, "断面付き小梁が1件設計される");
    let (_sid, _ji, jr) = &joists[0];
    // w = (固定0.005 + 床用積載2.9e-3) × spacing2000。M = wL²/8。
    let w_udl = (0.005 + 2.9e-3) * 2000.0;
    assert!((jr.w - w_udl).abs() / w_udl < 1e-9, "w={}", jr.w);
    assert!((jr.m_max - w_udl * 4000.0 * 4000.0 / 8.0).abs() < 1.0);
    assert!(jr.span > 0.0 && jr.bending_ratio > 0.0);

    assert_eq!(slabs.len(), 1, "矩形スラブが1件設計される");
    let (_sid, sr) = &slabs[0];
    assert!((sr.span - 4000.0).abs() < 1e-6, "短辺スパン");
    assert!(sr.moment > 0.0 && sr.as_req_per_m > 0.0);
    assert!((sr.effective_depth - 120.0).abs() < 1e-6, "d=t-かぶり");
}

/// 実部材化された小梁は全体 FEM で検定するため、床設計（小梁）の対象外になる。
#[test]
fn test_floor_design_skips_materialized_joist() {
    use squid_n_core::ids::SectionId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, JoistLine, LocalAxis, Section,
    };

    let mut model = make_square_slab_test_model();
    model.sections.push(Section {
        id: SectionId(0),
        name: "H".into(),
        area: 1.0,
        iy: 1.0e8,
        iz: 1.0,
        j: 1.0,
        depth: 400.0,
        width: 1.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    let mk_mid = |id: u32, x: f64, y: f64| squid_n_core::model::Node {
        id: NodeId(id),
        coord: [x, y, 0.0],
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    model.nodes.push(mk_mid(4, 2000.0, 0.0));
    model.nodes.push(mk_mid(5, 2000.0, 4000.0));
    model.slabs[0].joists.push(JoistLine {
        dir: [0.0, 1.0],
        spacing: 2000.0,
        support: [NodeId(4), NodeId(5)],
        section: Some(SectionId(0)),
        pinned_onto: None,
    });
    // 支持 N4–N5 を両端に持つ実 Beam を追加（実部材化相当）。
    let next = model.elements.len() as u32;
    model.elements.push(ElementData {
        id: ElemId(next),
        kind: ElementKind::Beam,
        nodes: [NodeId(4), NodeId(5)].into_iter().collect(),
        section: Some(SectionId(0)),
        material: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Pinned, EndCondition::Pinned],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });
    model.validate().expect("validate");
    let app = App {
        model,
        ..App::default()
    };

    let (joists, _slabs) = app.floor_design_checks();
    assert!(
        joists.is_empty(),
        "実部材化された小梁は床設計の対象外（全体 FEM で検定）"
    );
}

/// 床 Phase F: 交差小梁（十字）のスラブは床格子サブモデル（二方向）で検定される。
/// 2本の交差小梁が両方とも設計され、対称配置なので検定比が一致する。
#[test]
fn test_floor_design_uses_grillage_for_crossing_joists() {
    use squid_n_core::ids::SectionId;
    use squid_n_core::model::{JoistLine, Section, SlabUsage};

    let mut model = make_square_slab_test_model();
    model.slabs[0].usage = Some(SlabUsage::Office);
    // 鋼小梁断面。
    model.sections.push(Section {
        id: SectionId(0),
        name: "H-400".into(),
        area: 10000.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e6,
        depth: 400.0,
        width: 200.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    // 対辺の中点 N4..N7 を追加し、十字に交差する2本の小梁を配置。
    let mk = |id: u32, x: f64, y: f64| squid_n_core::model::Node {
        id: NodeId(id),
        coord: [x, y, 0.0],
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    model.nodes.push(mk(4, 2000.0, 0.0));
    model.nodes.push(mk(5, 2000.0, 4000.0));
    model.nodes.push(mk(6, 0.0, 2000.0));
    model.nodes.push(mk(7, 4000.0, 2000.0));
    model.slabs[0].joists = vec![
        JoistLine {
            dir: [0.0, 1.0],
            spacing: 2000.0,
            support: [NodeId(4), NodeId(5)],
            section: Some(SectionId(0)),
            pinned_onto: None,
        },
        JoistLine {
            dir: [1.0, 0.0],
            spacing: 2000.0,
            support: [NodeId(6), NodeId(7)],
            section: Some(SectionId(0)),
            pinned_onto: None,
        },
    ];
    model.validate().expect("validate");
    let app = App {
        model,
        ..App::default()
    };

    let (joists, _slabs) = app.floor_design_checks();
    assert_eq!(joists.len(), 2, "交差2小梁が格子で設計される");
    // FEM 由来の検定比（曲げ・たわみ）が正で、対称配置なので一致。
    assert!(joists[0].2.ratio > 0.0 && joists[1].2.ratio > 0.0);
    assert!(
        (joists[0].2.ratio - joists[1].2.ratio).abs() / joists[0].2.ratio < 1e-3,
        "対称な十字は検定比が一致: {} vs {}",
        joists[0].2.ratio,
        joists[1].2.ratio
    );
    // 曲げモーメントも FEM 実値（>0）。
    assert!(joists[0].2.m_max > 0.0);
}

/// 床 Phase E レビュー指摘: スラブ設計のスパンは一方向指定に一致する
/// （長辺方向へ一方向指定した場合、短辺ではなく長辺で設計する）。
#[test]
fn test_slab_design_span_respects_one_way() {
    use squid_n_core::ids::SlabId;
    use squid_n_core::model::{AreaLoad, DistributionMethod, OneWayDir, Slab};

    // lx=6000（辺0方向, X）、ly=3000（辺3方向, Y）の矩形スラブ。
    let mk_node = |id: u32, x: f64, y: f64| squid_n_core::model::Node {
        id: NodeId(id),
        coord: [x, y, 0.0],
        restraint: Default::default(),
        mass: None,
        story: None,
    };
    let base_slab = |one_way: Option<OneWayDir>| Slab {
        id: SlabId(0),
        boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        joists: vec![],
        loads: vec![AreaLoad {
            kind: "DL".into(),
            value: 0.005,
        }],
        method: DistributionMethod::OneWay,
        kind: Default::default(),
        one_way,
        edge_supported: None,
        usage: None,
        thickness: Some(150.0),
    };
    let mk_model = |one_way: Option<OneWayDir>| squid_n_core::model::Model {
        nodes: vec![
            mk_node(0, 0.0, 0.0),
            mk_node(1, 6000.0, 0.0),
            mk_node(2, 6000.0, 3000.0),
            mk_node(3, 0.0, 3000.0),
        ],
        slabs: vec![base_slab(one_way)],
        ..Default::default()
    };

    // 一方向 X → スパン = lx = 6000（長辺）。
    let app_x = App {
        model: mk_model(Some(OneWayDir::X)),
        ..App::default()
    };
    let (_j, slabs_x) = app_x.floor_design_checks();
    assert!(
        (slabs_x[0].1.span - 6000.0).abs() < 1e-6,
        "X一方向は長辺6000で設計"
    );

    // 一方向 Y → スパン = ly = 3000（短辺）。
    let app_y = App {
        model: mk_model(Some(OneWayDir::Y)),
        ..App::default()
    };
    let (_j, slabs_y) = app_y.floor_design_checks();
    assert!(
        (slabs_y[0].1.span - 3000.0).abs() < 1e-6,
        "Y一方向は短辺3000で設計"
    );

    // 指定なし → 短辺 min(6000,3000)=3000（安全側）。
    let app_n = App {
        model: mk_model(None),
        ..App::default()
    };
    let (_j, slabs_n) = app_n.floor_design_checks();
    assert!(
        (slabs_n[0].1.span - 3000.0).abs() < 1e-6,
        "両方向は短辺3000で設計"
    );
}

/// レビュー §1.7: 地震用重量に使う荷重ケースの選択が、並び順ではなく
/// `LoadCaseKind` に基づくことを確認する（Dead+LiveSeismic 優先、
/// LiveSeismic が無ければ Dead+Live、種別が一つも設定されていなければ
/// 従来互換で先頭ケースのみ）。
#[test]
fn test_gravity_cases_for_seismic_weight_selection() {
    use squid_n_core::model::{LoadCase, LoadCaseKind};

    let mk_lc = |i: u32, name: &str, kind: LoadCaseKind| LoadCase {
        id: LoadCaseId(i),
        name: name.to_string(),
        nodal: Vec::new(),
        member: Vec::new(),
        kind,
    };

    // 種別が一つも設定されていない（全て既定値 Other） → 先頭ケースのみ
    let model_no_kind = squid_n_core::model::Model {
        load_cases: vec![
            mk_lc(0, "LC0", LoadCaseKind::Other),
            mk_lc(1, "LC1", LoadCaseKind::Other),
        ],
        ..Default::default()
    };
    assert_eq!(
        gravity_cases_for_seismic_weight(&model_no_kind),
        vec![LoadCaseId(0)],
        "種別未設定モデルは従来互換で先頭ケースのみ"
    );

    // LiveSeismic が無い → Dead + Live
    let model_dead_live = squid_n_core::model::Model {
        load_cases: vec![
            mk_lc(0, "固定", LoadCaseKind::Dead),
            mk_lc(1, "積載(長期)", LoadCaseKind::Live),
            mk_lc(2, "積雪", LoadCaseKind::Snow),
        ],
        ..Default::default()
    };
    assert_eq!(
        gravity_cases_for_seismic_weight(&model_dead_live),
        vec![LoadCaseId(0), LoadCaseId(1)],
        "LiveSeismic が無ければ Dead+Live"
    );

    // LiveSeismic があれば Live ではなく LiveSeismic を優先
    let model_dead_live_seismic = squid_n_core::model::Model {
        load_cases: vec![
            mk_lc(0, "固定", LoadCaseKind::Dead),
            mk_lc(1, "積載(長期)", LoadCaseKind::Live),
            mk_lc(2, "積載(地震用)", LoadCaseKind::LiveSeismic),
        ],
        ..Default::default()
    };
    assert_eq!(
        gravity_cases_for_seismic_weight(&model_dead_live_seismic),
        vec![LoadCaseId(0), LoadCaseId(2)],
        "LiveSeismic があれば Live ではなく LiveSeismic を採用"
    );

    // 複数 Dead ケースも全て対象
    let model_multi_dead = squid_n_core::model::Model {
        load_cases: vec![
            mk_lc(0, "固定1", LoadCaseKind::Dead),
            mk_lc(1, "固定2", LoadCaseKind::Dead),
            mk_lc(2, "地震荷重", LoadCaseKind::Seismic),
        ],
        ..Default::default()
    };
    assert_eq!(
        gravity_cases_for_seismic_weight(&model_multi_dead),
        vec![LoadCaseId(0), LoadCaseId(1)],
        "複数の Dead ケースは全て対象、Seismic は対象外"
    );
}

/// テスト用の荷重ケース（種別付き）を作る。
fn kind_lc(
    i: u32,
    name: &str,
    kind: squid_n_core::model::LoadCaseKind,
) -> squid_n_core::model::LoadCase {
    squid_n_core::model::LoadCase {
        id: LoadCaseId(i),
        name: name.to_string(),
        nodal: Vec::new(),
        member: Vec::new(),
        kind,
    }
}

/// 種別から組合せを自動生成: Dead/Live/Snow/Wind の種別を設定したモデルで
/// 標準組合せ（長期・短期積雪・短期暴風±）が undo 可能に一括生成されること。
#[test]
fn test_auto_generate_combinations_from_kinds() {
    use squid_n_core::model::LoadCaseKind;

    let mut app = App::default();
    app.model.load_cases = vec![
        kind_lc(0, "固定", LoadCaseKind::Dead),
        kind_lc(1, "積載", LoadCaseKind::Live),
        kind_lc(2, "積雪", LoadCaseKind::Snow),
        kind_lc(3, "風", LoadCaseKind::Wind),
    ];

    app.auto_generate_combinations_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    // 多雪区域=false: DL+LL(1) + DL+LL+SL(1) + 風±(2) = 4 ケース
    // （地震(EX/EY)は kind だけでは方向を判別できないため対象外の仕様）。
    let names: Vec<&str> = app
        .model
        .combinations
        .iter()
        .map(|c| c.name.as_str())
        .collect();
    assert_eq!(
        names,
        vec!["DL + LL", "DL + LL + SL", "DL + LL + WX", "DL + LL - WX"]
    );

    // DL+LL の中身は Dead(0)+Live(1) を各1.0で参照する。
    assert_eq!(
        app.model.combinations[0].terms,
        vec![(LoadCaseId(0), 1.0), (LoadCaseId(1), 1.0)]
    );

    // 各組合せは個別コマンドで追加されているため、全 undo で消える。
    for _ in 0..app.model.combinations.len() {
        app.undo.undo(&mut app.model);
    }
    assert!(app.model.combinations.is_empty());
}

/// 多雪区域フラグ（AnalysisSettings::heavy_snow_zone）を立てると
/// 0.7S・0.35S 系の組合せも生成されること。
#[test]
fn test_auto_generate_combinations_heavy_snow() {
    use squid_n_core::model::LoadCaseKind;

    let mut app = App::default();
    app.analysis_cfg.heavy_snow_zone = true;
    app.model.load_cases = vec![
        kind_lc(0, "固定", LoadCaseKind::Dead),
        kind_lc(1, "積載", LoadCaseKind::Live),
        kind_lc(2, "積雪", LoadCaseKind::Snow),
        kind_lc(3, "風", LoadCaseKind::Wind),
    ];

    app.auto_generate_combinations_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let names: Vec<&str> = app
        .model
        .combinations
        .iter()
        .map(|c| c.name.as_str())
        .collect();
    assert!(names.contains(&"DL + LL + 0.7SL"), "{names:?}");
    assert!(names.contains(&"DL + LL + 0.35SL + WX"), "{names:?}");
    assert!(names.contains(&"DL + LL + 0.35SL - WX"), "{names:?}");
}

/// Dead ケースが無い場合はエラーメッセージが設定され、組合せは生成されないこと。
/// Live 欠如も同様。
#[test]
fn test_auto_generate_combinations_missing_dead_or_live_is_error() {
    use squid_n_core::model::LoadCaseKind;

    // Dead 無し
    let mut app = App::default();
    app.model.load_cases = vec![kind_lc(0, "積載", LoadCaseKind::Live)];
    app.auto_generate_combinations_action();
    assert!(app.last_error.as_deref().unwrap().contains("固定荷重"));
    assert!(app.model.combinations.is_empty());

    // Live 無し
    let mut app = App::default();
    app.model.load_cases = vec![kind_lc(0, "固定", LoadCaseKind::Dead)];
    app.auto_generate_combinations_action();
    assert!(app.last_error.as_deref().unwrap().contains("積載荷重"));
    assert!(app.model.combinations.is_empty());
}

/// SetLoadCfg が App の undo スタック経由で機能すること
/// （荷重計算条件タブの編集経路のヘッドレス確認）。
#[test]
fn test_set_load_cfg_via_app_undo() {
    use squid_n_core::model::{KBraceWeightRule, LoadCfg};

    let mut app = App::default();
    assert!(app.model.load_cfg.is_none());

    let cfg = LoadCfg {
        steel_weight_factor: 1.1,
        k_brace_rule: KBraceWeightRule::BaseNodesOnly,
        live_load_reduction: true,
        ..Default::default()
    };
    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::SetLoadCfg {
            cfg: Some(cfg.clone()),
        }),
    );
    assert_eq!(app.model.load_cfg, Some(cfg));

    app.undo.undo(&mut app.model);
    assert!(app.model.load_cfg.is_none());
}

/// 3層1本柱のモデルで `column_live_load_factors` が
/// 支持床数（3,2,1）と低減率（0.90,0.95,1.00）を返すこと（令85条2項）。
#[test]
fn test_column_live_load_factors_three_story() {
    use squid_n_core::ids::StoryId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node,
    };

    let mut model = squid_n_core::model::Model::default();
    // 4節点(z=0,3000,6000,9000)。z>0 の節点に所属階(1F=story0..3F=story2)を設定。
    for (i, z) in [0.0, 3000.0, 6000.0, 9000.0].iter().enumerate() {
        model.nodes.push(Node {
            id: NodeId(i as u32),
            coord: [0.0, 0.0, *z],
            restraint: if i == 0 {
                squid_n_core::dof::Dof6Mask::FIXED
            } else {
                squid_n_core::dof::Dof6Mask::FREE
            },
            mass: None,
            story: if i == 0 {
                None
            } else {
                Some(StoryId(i as u32 - 1))
            },
        });
    }
    // 柱3本（各階1本）＋ 水平の梁1本（柱でないため集計対象外の確認用）
    let mut push_elem = |id: u32, a: u32, b: u32| {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    };
    push_elem(0, 0, 1);
    push_elem(1, 1, 2);
    push_elem(2, 2, 3);
    // 水平材（同一 Z の節点を追加して繋ぐ）
    model.nodes.push(Node {
        id: NodeId(4),
        coord: [4000.0, 0.0, 9000.0],
        restraint: squid_n_core::dof::Dof6Mask::FREE,
        mass: None,
        story: Some(StoryId(2)),
    });
    model.elements.push(ElementData {
        id: ElemId(3),
        kind: ElementKind::Beam,
        nodes: [NodeId(3), NodeId(4)].into_iter().collect(),
        section: None,
        material: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });

    let factors = column_live_load_factors(&model);
    // 水平梁(ElemId(3))は含まれない。
    assert_eq!(
        factors,
        vec![
            (ElemId(0), 3, 0.90),
            (ElemId(1), 2, 0.95),
            (ElemId(2), 1, 1.00),
        ]
    );
}

/// 準備計算は階の定義を含む: 階が未定義でも `run_preparation` 1 回で階が生成され、
/// 主要構造種別が柱・梁の断面形状から自動判定される（門型ラーメンは全て鋼断面 → S）。
#[test]
fn test_run_preparation_generates_stories_and_infers_structure() {
    use squid_n_core::model::StoryStructure;
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    assert!(app.model.stories.is_empty(), "前提: 階は未定義");

    app.run_preparation();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert_eq!(app.model.stories.len(), 1, "準備計算が階を生成するはず");
    assert_eq!(
        app.model.stories[0].structure,
        StoryStructure::S,
        "鋼断面の柱梁だけの階は S と判定されるはず"
    );
    assert!(!app.staleness.preparation_stale);
}

/// 準備計算は冪等: モデルが変わっていなければ 2 回目以降の実行で undo 履歴を積まず、
/// 解析結果を stale にもしない（毎回階を再生成する構成でも、実質的な差分が無ければ
/// `ApplyStories` を発行しない）。
#[test]
fn test_run_preparation_is_idempotent() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_preparation();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    // 解析まで済ませて結果を最新状態にする。
    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(!app.staleness.results_stale);
    let undo_label = app.undo.undo_label().map(|s| s.to_string());
    let stories_before = app.model.stories.clone();
    let nodes_before = app.model.nodes.len();

    // 2 回目の準備計算ではモデルが変わらない。
    app.run_preparation();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert_eq!(app.model.stories, stories_before);
    assert_eq!(app.model.nodes.len(), nodes_before);
    assert_eq!(
        app.undo.undo_label().map(|s| s.to_string()),
        undo_label,
        "差分が無ければ undo 履歴を積まないはず"
    );
    assert!(
        !app.staleness.results_stale,
        "差分が無ければ解析結果を stale にしないはず"
    );
}

/// 階を生成できないモデル（節点なし）でも準備計算は中断せず、階の生成エラーを
/// 提示したうえで階を前提としない項目の集計まで進む。
#[test]
fn test_run_preparation_continues_when_story_generation_fails() {
    let mut app = App::default();
    app.load_model(squid_n_core::model::Model::default());

    app.run_preparation();
    assert!(
        app.last_error
            .as_deref()
            .unwrap()
            .contains("階の生成エラー"),
        "{:?}",
        app.last_error
    );
    let prep = app
        .preparation
        .as_ref()
        .expect("階が作れなくても準備計算の集計は行われるはず");
    assert!(prep.stories.is_empty());
    assert!(!app.staleness.preparation_stale);
}

/// 準備計算は実行のたびに階を再生成するが、利用者の手入力（地震用重量の手入力値・
/// 階の種別）は引き継がれる。手入力を解除すれば次の準備計算で自動算定値へ戻る。
#[test]
fn test_run_preparation_regenerates_stories_keeping_manual_input() {
    use squid_n_core::model::StoryLevelKind;
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_preparation();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let auto_weight = app.model.stories[0].seismic_weight.unwrap();
    assert!(auto_weight > 0.0);

    // 地震用重量の手入力と階の種別（PH）を設定する。
    let story = app.model.stories[0].id;
    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::SetStoryWeight {
            story,
            weight: Some(auto_weight * 2.0),
        }),
    );
    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::SetStoryLevelKind {
            story,
            level_kind: StoryLevelKind::Penthouse { k: 0.6 },
        }),
    );

    // 再度の準備計算でも手入力は失われない。
    app.run_preparation();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert_eq!(
        app.model.stories[0].weight_override,
        Some(auto_weight * 2.0)
    );
    assert_eq!(app.model.stories[0].seismic_weight, Some(auto_weight * 2.0));
    assert_eq!(
        app.model.stories[0].level_kind,
        StoryLevelKind::Penthouse { k: 0.6 }
    );

    // 手入力を解除すると次の準備計算で自動算定値へ戻る。
    let story = app.model.stories[0].id;
    app.undo.run(
        &mut app.model,
        Box::new(squid_n_edit::SetStoryWeight {
            story,
            weight: None,
        }),
    );
    app.run_preparation();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert_eq!(app.model.stories[0].weight_override, None);
    // PH 階でも階自体の重量集計は変わらない（Ai 分布での扱いだけが変わる）。
    assert!((app.model.stories[0].seismic_weight.unwrap() - auto_weight).abs() < 1e-6);
}

/// 準備計算は風圧力も荷重ケース（WX/WY）へ同期する。門型ラーメンは Y 方向の風
/// （見付け幅 = X 方向の座標範囲）のみ算定できるため WY だけが生成される。
/// 生成された WY は風荷重静的解析（`run_wind`）と同じ水平力を持つ。
#[test]
fn test_run_preparation_syncs_wind_load_cases() {
    use squid_n_core::model::LoadCaseKind;
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_preparation();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let wy = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == WY_CASE_NAME)
        .expect("WYケースが生成されるはず");
    assert_eq!(wy.kind, LoadCaseKind::Wind);
    assert!(!wy.nodal.is_empty(), "WYに層水平力が入っているはず");
    assert!(
        app.model
            .load_cases
            .iter()
            .all(|lc| lc.name != WX_CASE_NAME),
        "見付け幅 0 の X 方向の風は生成されないはず"
    );

    // 風荷重静的解析が組み立てる水平力と一致する。
    let cfg = squid_n_solver::analysis::WindStaticCfg {
        dir: SeismicDir::Y,
        v0: app.analysis_cfg.v0,
        roughness: app.analysis_cfg.roughness,
        cpi: 0.0,
        parapet_mm: app.analysis_cfg.parapet_mm,
    };
    let built = squid_n_solver::analysis::build_wind_load_case_from_model(&app.model, cfg).unwrap();
    assert_eq!(wy.nodal, built.nodal);
}

/// 荷重ケースの実行導線は、標準の水平力ケース（EX/EY・WX/WY）を方向別の結果キーへ
/// 振り分ける（地震静的・風荷重静的の専用導線を廃し、荷重ケース解析へ統合した）。
#[test]
fn test_load_case_job_routes_standard_lateral_cases() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_preparation();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let id_of = |app: &App, name: &str| {
        app.model
            .load_cases
            .iter()
            .find(|lc| lc.name == name)
            .map(|lc| lc.id)
            .unwrap()
    };
    let ex = id_of(&app, EX_CASE_NAME);
    let wy = id_of(&app, WY_CASE_NAME);
    let dl = id_of(&app, DL_CASE_NAME);
    assert_eq!(
        app.standard_lateral_case(ex),
        Some(StaticCaseKey::Seismic(SeismicDir::X))
    );
    assert_eq!(
        app.standard_lateral_case(wy),
        Some(StaticCaseKey::Wind(SeismicDir::Y))
    );
    assert_eq!(app.standard_lateral_case(dl), None, "DL は通常の線形静的");

    // EX の実行は地震静的ジョブとして走り、結果は Seismic(X) へ格納される。
    app.start_load_case_job(ex);
    assert_eq!(app.job.as_ref().unwrap().label, "地震静的解析");
    wait_for_job(&mut app);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert_eq!(
        app.last_static,
        Some(StaticKey::Case(StaticCaseKey::Seismic(SeismicDir::X)))
    );

    // DL の実行は線形静的ジョブとして走り、結果は User キーへ格納される。
    app.start_load_case_job(dl);
    assert_eq!(app.job.as_ref().unwrap().label, "線形静的解析");
    wait_for_job(&mut app);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert_eq!(
        app.last_static,
        Some(StaticKey::Case(StaticCaseKey::User(dl)))
    );
}

/// 種別からの標準組合せ自動生成は、準備計算が生成する標準ケース名で方向を判別し、
/// 地震（±EX/±EY）・暴風（±WX/±WY）の組合せまで含めて生成する。
#[test]
fn test_auto_generate_combinations_uses_standard_case_names() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_preparation();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    app.model.combinations.clear();
    // 門型ラーメンのサンプルはスラブを持たず積載荷重ケースが自動生成されない
    // （組合せ生成には固定・積載の両方が要る）ため、空の積載ケースを足す。
    let next_id = LoadCaseId(app.model.load_cases.len() as u32);
    app.model.load_cases.push(squid_n_core::model::LoadCase {
        id: next_id,
        name: LL_FRAME_CASE_NAME.into(),
        kind: squid_n_core::model::LoadCaseKind::Live,
        nodal: Vec::new(),
        member: Vec::new(),
    });

    app.auto_generate_combinations_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let names: Vec<&str> = app
        .model
        .combinations
        .iter()
        .map(|c| c.name.as_str())
        .collect();
    assert!(names.contains(&"DL + LL + EX"), "{names:?}");
    assert!(names.contains(&"DL + LL - EY"), "{names:?}");
    // 門型ラーメンでは WY のみ生成されるため、暴風は Y 方向の組合せのみ。
    assert!(names.contains(&"DL + LL + WY"), "{names:?}");
    assert!(!names.iter().any(|n| n.contains("WX")), "{names:?}");
}

/// 風荷重静的解析（run_wind）: 階の定義後に実行でき、結果が
/// `StaticCaseKey::Wind(dir)` に格納されること。
///
/// サンプルの門型ラーメンは XZ 平面内の平面架構のため、Y 方向の風
/// （見付け幅 = X 方向の座標範囲 4000mm）のみ解析できる。X 方向の風は
/// 見付け幅（Y 範囲）が 0 のため明示エラーになることも併せて確認する。
#[test]
fn test_run_wind_static() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());

    // 階なし → 明示エラー
    app.run_wind(SeismicDir::Y);
    assert!(app.last_error.is_some());

    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    // 平面架構の面外（X風）は見付け幅 0 の明示エラー
    app.run_wind(SeismicDir::X);
    assert!(app.last_error.is_some());

    app.run_wind(SeismicDir::Y);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let r = app.results.as_ref().unwrap();
    let wind = r
        .statics
        .iter()
        .find(|(k, _)| *k == StaticCaseKey::Wind(SeismicDir::Y))
        .expect("風静的Yの結果が格納されるはず");
    // 柱頭が Y 方向へ変位している（風方向の水平力が作用した証拠）
    assert!(wind.1.disp[2][1].abs() > 1e-9, "{}", wind.1.disp[2][1]);
    assert_eq!(
        app.last_static,
        Some(StaticKey::Case(StaticCaseKey::Wind(SeismicDir::Y)))
    );
}

/// 終局検定（靭性保証型耐震設計指針）の App 経由の一括算定を検証する。
/// RC 矩形の柱・梁について `compute_ultimate_checks` が部材別の終局せん断・付着
/// 余裕度を返し、柱には軸終局耐力（Nuc/Nut）が付くことを確認する。
#[test]
fn test_compute_ultimate_checks_rc_frame() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node,
    };
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};
    use squid_n_design_jp::MemberKind;

    let rebar = RcRebar {
        main_grade: None,
        main_x: BarSet {
            count: 8,
            dia: 25.0,
            layers: 1,
        },
        main_y: BarSet {
            count: 8,
            dia: 25.0,
            layers: 1,
        },
        cover: 40.0,
        shear: ShearBar {
            dia: 10.0,
            pitch: 100.0,
            legs: 2,
            grade: None,
        },
    };
    let col_shape = SectionShape::RcRect {
        b: 600.0,
        d: 600.0,
        rebar: rebar.clone(),
    };
    let beam_rebar = RcRebar {
        main_grade: None,
        main_x: BarSet {
            count: 6,
            dia: 25.0,
            layers: 1,
        },
        ..rebar
    };
    let beam_shape = SectionShape::RcRect {
        b: 400.0,
        d: 700.0,
        rebar: beam_rebar,
    };

    let mut model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(2),
                coord: [6000.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ],
        sections: vec![
            col_shape.to_section(SectionId(0), "C600".into()),
            beam_shape.to_section(SectionId(1), "B400x700".into()),
        ],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SD345".into(),
            young: 23000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: Some(345.0),
        }],
        ..Default::default()
    };
    let members = [
        (0u32, 0u32, 1u32, 0u32, [1.0, 0.0, 0.0]), // 柱（鉛直）
        (1, 1, 2, 1, [0.0, 0.0, 1.0]),             // 梁（水平）
    ];
    for (id, i, j, sec, ref_vector) in members {
        model.elements.push(ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
            section: Some(SectionId(sec)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis { ref_vector },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }

    let mut app = App::default();
    app.load_model(model);
    let checks = app
        .compute_ultimate_checks()
        .expect("RC 矩形部材があれば Ok のはず");
    assert_eq!(checks.len(), 2, "柱・梁の 2 部材が検定される");

    let col = checks.iter().find(|c| c.elem == ElemId(0)).unwrap();
    let beam = checks.iter().find(|c| c.elem == ElemId(1)).unwrap();
    assert_eq!(col.kind, MemberKind::Column);
    assert_eq!(beam.kind, MemberKind::Beam);
    // 各耐力・余裕度が正常に算定される。
    assert!(col.qsu > 0.0 && col.qmu > 0.0 && col.shear_margin > 0.0);
    assert!(col.axial.is_some(), "柱は軸終局耐力を持つ");
    assert!(beam.axial.is_none(), "梁は軸終局耐力なし");
    // 付着検定 ON（既定）なので Qbu も算定される。
    assert!(col.qbu > 0.0 && col.bond_margin.is_finite());
}

/// CFT 柱の軸終局検定を App 経由で算定する。
#[test]
fn test_compute_cft_ultimate_checks() {
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::MaterialId;
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node,
    };
    use squid_n_core::section_shape::SectionShape;

    let cft_shape = SectionShape::CftBox {
        height: 400.0,
        width: 400.0,
        thick: 12.0,
    };
    let mut model = Model {
        nodes: vec![
            Node {
                id: NodeId(0),
                coord: [0.0, 0.0, 0.0],
                restraint: Dof6Mask::FIXED,
                mass: None,
                story: None,
            },
            Node {
                id: NodeId(1),
                coord: [0.0, 0.0, 3000.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            },
        ],
        sections: vec![cft_shape.to_section(SectionId(0), "CFT400".into())],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "BCR295".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: Some(30.0),
            fy: None,
        }],
        ..Default::default()
    };
    model.elements.push(ElementData {
        id: ElemId(0),
        kind: ElementKind::Beam,
        nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    });

    let mut app = App::default();
    app.load_model(model);
    let checks = app
        .compute_cft_ultimate_checks()
        .expect("CFT 柱があれば Ok のはず");
    assert_eq!(checks.len(), 1);
    assert!(checks[0].ncu > 0.0 && checks[0].ntu > 0.0);
}

// ------------------------------------------------------------------
// 標準荷重ケース（DL・LL(架構用)・LL(地震用)・EX・EY）
// ------------------------------------------------------------------

/// 新規モデル（`Model::with_default_load_cases`）は標準5ケースと標準荷重組合せを持ち、
/// `load_model` を通しても保持されることを確認する。
#[test]
fn test_new_model_has_default_load_cases() {
    let mut app = App::default();
    app.load_model(squid_n_core::model::Model::with_default_load_cases());
    let names: Vec<&str> = app
        .model
        .load_cases
        .iter()
        .map(|c| c.name.as_str())
        .collect();
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
    // 標準荷重組合せ（長期 DL+LL、短期地震 DL+LL±EX・DL+LL±EY）も既定で用意される。
    let combo_names: Vec<&str> = app
        .model
        .combinations
        .iter()
        .map(|c| c.name.as_str())
        .collect();
    assert_eq!(
        combo_names,
        vec![
            "DL + LL",
            "DL + LL + EX",
            "DL + LL - EX",
            "DL + LL + EY",
            "DL + LL - EY"
        ]
    );
}

/// DL の自動同期にスラブ固定荷重と躯体自重の両方が含まれることを確認する
/// （DL＝自重＋スラブ重量の自動計算。「自重(自動)」ケースは作られない）。
#[test]
fn test_sync_gravity_dl_includes_self_weight_and_slab() {
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::model::{Material, MemberLoadKind, Section};

    let mut model = make_square_slab_test_model();
    // 全梁に断面・材料（密度あり）を与え、自重を発生させる。
    model.sections.push(Section {
        id: SectionId(0),
        name: "RC400x600".into(),
        area: 400.0 * 600.0,
        iy: 1.0e8,
        iz: 1.0e8,
        j: 1.0e8,
        depth: 600.0,
        width: 400.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    });
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "Fc24".into(),
        young: 22000.0,
        poisson: 0.2,
        density: 2.4e-9,
        shear: None,
        fc: Some(24.0),
        fy: None,
    });
    for e in &mut model.elements {
        e.section = Some(SectionId(0));
        e.material = Some(MaterialId(0));
    }
    model
        .validate()
        .expect("テストモデルは validate を通るはず");
    let mut app = App {
        model,
        ..App::default()
    };
    app.sync_gravity_load_cases_action();

    let dl = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == DL_CASE_NAME)
        .expect("DLケースが作られるはず");
    let total: f64 = dl
        .member
        .iter()
        .map(|m| match m.kind {
            MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
            MemberLoadKind::Point { p, .. } => p,
        })
        .sum::<f64>()
        + dl.nodal.iter().map(|nl| -nl.values[2]).sum::<f64>();
    let slab_dl = 0.005 * 4000.0 * 4000.0;
    let (sw_nodal, sw_member) = squid_n_load::self_weight::self_weight_case_content(
        &app.model,
        &squid_n_core::model::LoadCfg::default(),
    );
    let self_weight: f64 = sw_member
        .iter()
        .map(|m| match m.kind {
            MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
            MemberLoadKind::Point { p, .. } => p,
        })
        .sum::<f64>()
        + sw_nodal.iter().map(|nl| -nl.values[2]).sum::<f64>();
    assert!(self_weight > 0.0, "自重が発生しているはず");
    assert!(
        (total - (slab_dl + self_weight)).abs() < 1e-6,
        "DL 合計 {total} = スラブ {slab_dl} + 自重 {self_weight} のはず"
    );
    // 旧「自重(自動)」ケースは作られない。
    assert!(app
        .model
        .load_cases
        .iter()
        .all(|lc| lc.name != SELF_WEIGHT_AUTO_LOAD_CASE_NAME));
}

/// 地震用重量が二重計上されないことを確認する: DL に自重が含まれる構成での
/// 階の自動生成の階重量 = 自重総量 + 手動重力ケースの鉛直荷重総量。
/// 再生成しても同じ値になる（冪等）。
#[test]
fn test_generate_stories_seismic_weight_no_double_count() {
    use squid_n_core::model::MemberLoadKind;

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    // 自重の節点配分（等分布荷重の静定反力 = 両端 1/2 ずつ）のうち、
    // 階レベル（z=3500 の節点 2,3）へ配分される分だけが階重量に算入される
    // （柱の下半分は基部 z=0 へ配分され階に属さない）。
    let (sw_nodal, sw_member) = squid_n_load::self_weight::self_weight_case_content(
        &app.model,
        &squid_n_core::model::LoadCfg::default(),
    );
    let mut node_share = vec![0.0_f64; app.model.nodes.len()];
    for nl in &sw_nodal {
        node_share[nl.node.index()] += -nl.values[2];
    }
    for ml in &sw_member {
        let elem = app.model.elements.iter().find(|e| e.id == ml.elem).unwrap();
        let total = match ml.kind {
            MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
            MemberLoadKind::Point { p, .. } => p,
        };
        node_share[elem.nodes[0].index()] += total / 2.0;
        node_share[elem.nodes[1].index()] += total / 2.0;
    }
    let self_weight_at_story: f64 = app
        .model
        .nodes
        .iter()
        .filter(|n| n.coord[2] > 0.0)
        .map(|n| node_share[n.id.index()])
        .sum();
    assert!(self_weight_at_story > 0.0, "自重が発生しているはず");
    // サンプル LC0「長期」（kind=Dead）: 梁等分布 10 N/mm × 6000 mm = 60 kN。
    let case_loads = 10.0 * 6000.0;
    let w = app.model.stories[0].seismic_weight.unwrap();
    assert!(
        (w - (self_weight_at_story + case_loads)).abs() < 1e-6,
        "階重量 {w} = 自重(階配分) {self_weight_at_story} + ケース荷重 {case_loads} のはず（二重計上なし）"
    );

    // 再生成しても増えない（冪等）。
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    let w2 = app.model.stories[0].seismic_weight.unwrap();
    assert!((w2 - w).abs() < 1e-6, "再生成で階重量が変わってはいけない");
}

/// 階の自動生成後に EX・EY ケースへ Ai 分布の水平力が同期されることを確認する。
#[test]
fn test_generate_stories_syncs_ex_ey_cases() {
    use squid_n_core::model::LoadCaseKind;

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let ex = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == EX_CASE_NAME)
        .expect("EXケースが作られるはず");
    assert_eq!(ex.kind, LoadCaseKind::Seismic);
    let fx: f64 = ex.nodal.iter().map(|nl| nl.values[0]).sum();
    assert!(fx > 0.0, "EX は +X 方向の水平力を持つはず: {fx}");
    assert!(
        ex.nodal.iter().all(|nl| nl.values[1] == 0.0),
        "EX に Y 成分は無いはず"
    );

    let ey = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == EY_CASE_NAME)
        .expect("EYケースが作られるはず");
    let fy: f64 = ey.nodal.iter().map(|nl| nl.values[1]).sum();
    assert!(fy > 0.0, "EY は +Y 方向の水平力を持つはず: {fy}");
    // 層せん断力の総和は方向によらず同じ（同じ Ai 分布）。
    assert!((fx - fy).abs() < 1e-9, "fx={fx} fy={fy}");
}

/// `load_model` が旧スキーマの自動生成ケース名を標準名へ移行することを確認する
/// （床荷重(自動)→DL、自重(自動)は DL へ統合、床積載(自動)→LL(架構用)）。
#[test]
fn test_load_model_migrates_legacy_case_names() {
    use squid_n_core::model::{LoadCase, LoadCaseKind};

    let mut model = crate::sample::portal_frame();
    model.load_cases.clear();
    let mk = |i: u32, name: &str, kind: LoadCaseKind| LoadCase {
        id: LoadCaseId(i),
        name: name.into(),
        nodal: Vec::new(),
        member: Vec::new(),
        kind,
    };
    model
        .load_cases
        .push(mk(0, "床荷重(自動)", LoadCaseKind::Dead));
    model
        .load_cases
        .push(mk(1, "自重(自動)", LoadCaseKind::Dead));
    model
        .load_cases
        .push(mk(2, "床積載(自動)", LoadCaseKind::Live));

    let mut app = App::default();
    app.load_model(model);
    let names: Vec<&str> = app
        .model
        .load_cases
        .iter()
        .map(|c| c.name.as_str())
        .collect();
    assert_eq!(names, vec![DL_CASE_NAME, LL_FRAME_CASE_NAME]);
    assert!(app.model.validate().is_ok());
}

/// 空の地震荷重ケース（未生成の EX 等）を参照する荷重組合せは実行せず、
/// 案内エラーになることを確認する（地震項が黙って 0 になるのを防ぐ）。
#[test]
fn test_run_combination_errors_on_empty_seismic_case() {
    use squid_n_core::model::{LoadCase, LoadCaseKind, LoadCombination};

    let mut model = crate::sample::portal_frame();
    // 空の EX ケース（階未生成のため内容なし）と、それを参照する組合せ。
    let ex_id = LoadCaseId(model.load_cases.len() as u32);
    model.load_cases.push(LoadCase {
        id: ex_id,
        name: EX_CASE_NAME.into(),
        nodal: Vec::new(),
        member: Vec::new(),
        kind: LoadCaseKind::Seismic,
    });
    model.combinations.push(LoadCombination {
        name: "G + P + Kx".into(),
        terms: vec![(LoadCaseId(0), 1.0), (ex_id, 1.0)],
    });

    let mut app = App::default();
    app.load_model(model);
    app.run_combination(0);
    let err = app.last_error.as_deref().unwrap_or("");
    assert!(
        err.contains("EX") && err.contains("空"),
        "空の EX 参照はエラーで案内するはず: {err}"
    );
}

/// ST-Bridge が荷重情報を持たない場合、読込時に標準荷重ケース
/// （DL・LL(架構用)・LL(地震用)・EX・EY）が自動作成されることを確認する
/// （本実装のエクスポートは幾何サブセットで荷重を書き出さないため、
/// 書き出し→読み戻しで確認できる）。
#[test]
fn test_import_stbridge_without_loads_creates_default_cases() {
    let dir = std::env::temp_dir().join("squid_n_app_test_stbridge_default_lc");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("no_loads.stb");

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.export_stbridge_to(path.clone());
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let mut app2 = App::default();
    app2.import_stbridge_from(path.clone());
    // 支点の自動設定の通知は出る（欠落警告ではない）。
    let msg = app2.last_error.as_deref().unwrap_or("");
    assert!(msg.is_empty() || msg.contains("ピン支点に設定"), "{msg}");
    let names: Vec<&str> = app2
        .model
        .load_cases
        .iter()
        .map(|c| c.name.as_str())
        .collect();
    assert_eq!(
        names,
        vec![
            DL_CASE_NAME,
            LL_FRAME_CASE_NAME,
            LL_SEISMIC_CASE_NAME,
            EX_CASE_NAME,
            EY_CASE_NAME
        ],
        "荷重の無い STB は標準荷重ケースが自動作成されるはず"
    );
    assert!(app2.model.validate().is_ok());

    std::fs::remove_file(&path).ok();
}

/// ST-Bridge が荷重ケース（`StbLoadCase`）を持つ場合は、ファイルの荷重ケースを
/// そのまま採用し、標準荷重ケースを追加しないことを確認する。
#[test]
fn test_import_stbridge_with_loads_keeps_file_cases() {
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
  </StbSections>
  <StbMembers>
    <StbColumns>
      <StbColumn id="0" name="C1" id_node_bottom="0" id_node_top="1" id_section="0" kind_structure="S"/>
    </StbColumns>
  </StbMembers>
  <StbLoads>
    <StbLoadCase id="0" name="L1">
      <StbNodalLoad id_node="1" fz="-5000"/>
    </StbLoadCase>
  </StbLoads>
</StbModel></ST_BRIDGE>"#;
    let dir = std::env::temp_dir().join("squid_n_app_test_stbridge_with_lc");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("with_loads.stb");
    std::fs::write(&path, xml).unwrap();

    let mut app = App::default();
    app.import_stbridge_from(path.clone());
    assert_eq!(
        app.model.load_cases.len(),
        1,
        "STB 自身の荷重ケースを採用し、標準ケースは追加しない: {:?}",
        app.last_error
    );
    assert_eq!(app.model.load_cases[0].name, "L1");

    std::fs::remove_file(&path).ok();
}

/// 報告された問題の再発防止: 支点情報の無い ST-Bridge を読み込んだ直後に
/// DL の線形静的解析がそのまま実行できる（支点の自動設定（ピン）＋標準荷重
/// ケースの自動作成＋DL 自重同期のエンドツーエンド）。
///
/// モデルは 1 層の立体フレーム（柱4本＋外周梁4本）。支点をピンにする既定では
/// 平面フレーム（支点が一直線）は面外の機構になり解けないため、実建物と同じく
/// 支点が平面的に分布する立体モデルで確認する。
#[test]
fn test_import_stbridge_then_run_dl_succeeds() {
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material, Model, Node,
    };
    use squid_n_section::shape::SectionShape;

    let dir = std::env::temp_dir().join("squid_n_app_test_stbridge_run_dl");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("run_dl.stb");

    // 1 層立体フレーム: 柱脚 4 節点（矩形配置, z=0）＋柱頭 4 節点（z=3000）。
    let mut model = Model::default();
    let plan = [(0.0, 0.0), (6000.0, 0.0), (6000.0, 4000.0), (0.0, 4000.0)];
    for (k, z) in [0.0, 3000.0].into_iter().enumerate() {
        for (i, (x, y)) in plan.iter().enumerate() {
            model.nodes.push(Node {
                id: NodeId((i + 4 * k) as u32),
                coord: [*x, *y, z],
                restraint: squid_n_core::dof::Dof6Mask::FREE,
                mass: None,
                story: None,
            });
        }
    }
    let col_shape = SectionShape::SteelH {
        height: 300.0,
        width: 300.0,
        web_thick: 10.0,
        flange_thick: 15.0,
    };
    model
        .sections
        .push(col_shape.to_section(SectionId(0), "柱".into()));
    model.materials.push(Material {
        strength_factor: None,
        concrete_class: Default::default(),
        id: MaterialId(0),
        name: "SN400B".into(),
        young: 205000.0,
        poisson: 0.3,
        density: 7.85e-9,
        shear: None,
        fc: None,
        fy: Some(235.0),
    });
    // 柱 4 本（i→i+4）＋外周梁 4 本（柱頭を一周）。
    let conn: [(u32, u32); 8] = [
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
    ];
    for (i, (a, b)) in conn.iter().enumerate() {
        let vertical = *b == *a + 4;
        model.elements.push(ElementData {
            id: ElemId(i as u32),
            kind: ElementKind::Beam,
            nodes: [NodeId(*a), NodeId(*b)].into_iter().collect(),
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: if vertical {
                    [1.0, 0.0, 0.0]
                } else {
                    [0.0, 0.0, 1.0]
                },
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }
    model
        .validate()
        .expect("テストモデルは validate を通るはず");

    let mut app = App::default();
    app.load_model(model);
    app.export_stbridge_to(path.clone());
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let mut app2 = App::default();
    app2.import_stbridge_from(path.clone());
    let dl_id = app2
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == DL_CASE_NAME)
        .expect("標準荷重ケース DL が自動作成されるはず")
        .id;

    app2.run_linear_static(dl_id);
    assert!(
        app2.last_error.is_none(),
        "支点自動設定により DL 解析が成功するはず: {:?}",
        app2.last_error
    );
    let results = app2.results.as_ref().expect("解析結果が格納されるはず");
    assert!(results
        .statics
        .iter()
        .any(|(k, _)| *k == StaticCaseKey::User(dl_id)));
    // DL には自重が同期され、柱に軸力（鉛直変位）が生じている。
    let dl = app2
        .model
        .load_cases
        .iter()
        .find(|lc| lc.id == dl_id)
        .unwrap();
    assert!(
        !(dl.nodal.is_empty() && dl.member.is_empty()),
        "DL に自重が同期されているはず"
    );

    std::fs::remove_file(&path).ok();
}

/// 二次部材（小梁）と、その節点で区切られたパネルスラブを持つモデル
/// （ST-Bridge 取り込みの典型形）の DL 同期と解析:
///
/// - 小梁は解析要素ではなく、その支持節点（大梁スパン中間・要素非接続）に
///   落ちる床荷重・小梁自重は大梁の**中間集中荷重（CMQ）**へ変換される。
/// - 実梁が無いスラブ辺（小梁上の辺・大梁の中間区間）の荷重も捨てられず
///   主架構へ伝達され、鉛直荷重の総和が保存される。
/// - そのまま線形静的解析が成功する（小梁支持節点は解析自由度から除外）。
#[test]
fn test_secondary_joist_panel_slab_dl_cmq_and_solve() {
    use squid_n_core::ids::{MaterialId, SectionId, SlabId};
    use squid_n_core::model::{
        AreaLoad, DistributionMethod, ElementData, ElementKind, EndCondition, ForceRegime,
        LocalAxis, Material, MemberLoadKind, Model, Node, SecondaryMember, SecondaryMemberKind,
        Section, Slab,
    };

    let mk_node = |id: u32, x: f64, y: f64, z: f64, fixed: bool| Node {
        id: NodeId(id),
        coord: [x, y, z],
        restraint: if fixed {
            squid_n_core::dof::Dof6Mask::FIXED
        } else {
            squid_n_core::dof::Dof6Mask::FREE
        },
        mass: None,
        story: None,
    };
    // 柱脚 0-3（固定）、柱頭 4-7、小梁支持点 8-9（大梁スパン中間・要素非接続）。
    let plan = [(0.0, 0.0), (8000.0, 0.0), (8000.0, 6000.0), (0.0, 6000.0)];
    let mut nodes = Vec::new();
    for (i, (x, y)) in plan.iter().enumerate() {
        nodes.push(mk_node(i as u32, *x, *y, 0.0, true));
    }
    for (i, (x, y)) in plan.iter().enumerate() {
        nodes.push(mk_node(4 + i as u32, *x, *y, 3500.0, false));
    }
    nodes.push(mk_node(8, 4000.0, 0.0, 3500.0, false));
    nodes.push(mk_node(9, 4000.0, 6000.0, 3500.0, false));

    let mk_beam = |id: u32, i: u32, j: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: [NodeId(i), NodeId(j)].into_iter().collect(),
        section: Some(SectionId(0)),
        material: Some(MaterialId(0)),
        local_axis: LocalAxis {
            ref_vector: if i + 4 == j {
                [1.0, 0.0, 0.0]
            } else {
                [0.0, 0.0, 1.0]
            },
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    // 柱 4 本 + 外周大梁 4 本（大梁 y=0 は ElemId(4)、節点 4→5）。
    let elements = vec![
        mk_beam(0, 0, 4),
        mk_beam(1, 1, 5),
        mk_beam(2, 2, 6),
        mk_beam(3, 3, 7),
        mk_beam(4, 4, 5),
        mk_beam(5, 5, 6),
        mk_beam(6, 6, 7),
        mk_beam(7, 7, 4),
    ];
    let mk_slab = |id: u32, boundary: Vec<u32>| Slab {
        id: SlabId(id),
        boundary: boundary.into_iter().map(NodeId).collect(),
        joists: vec![],
        loads: vec![AreaLoad {
            kind: "DL".into(),
            value: 0.005,
        }],
        method: DistributionMethod::TriTrapezoid,
        usage: None,
        edge_supported: None,
        thickness: None,
        kind: Default::default(),
        one_way: None,
    };
    let model = Model {
        nodes,
        elements,
        sections: vec![Section {
            id: SectionId(0),
            name: "RC400x600".into(),
            area: 400.0 * 600.0,
            iy: 1.0e9,
            iz: 1.0e9,
            j: 1.0e9,
            depth: 600.0,
            width: 400.0,
            // RC の有効せん断断面積 As = A/κ（κ=1.2）。解析前チェックが As=0 を
            // 入力不足として弾くため、直接入力断面でも実値を与える。
            as_y: 400.0 * 600.0 / 1.2,
            as_z: 400.0 * 600.0 / 1.2,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "Fc24".into(),
            young: 22000.0,
            poisson: 0.2,
            density: 2.4e-9,
            shear: None,
            fc: Some(24.0),
            fy: None,
        }],
        // 小梁: 大梁 y=0 の中間 (4000,0) と大梁 y=6000 の中間 (4000,6000) を結ぶ。
        secondary_members: vec![SecondaryMember {
            kind: SecondaryMemberKind::Joist,
            nodes: [NodeId(8), NodeId(9)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            name: "B1".into(),
        }],
        // 小梁で区切られた 2 枚のパネルスラブ（計 8000×6000）。
        slabs: vec![mk_slab(0, vec![4, 8, 9, 7]), mk_slab(1, vec![8, 5, 6, 9])],
        ..Default::default()
    };
    model
        .validate()
        .expect("テストモデルは validate を通るはず");

    let mut app = App::default();
    app.load_model(model);
    app.sync_gravity_load_cases_action();

    let dl = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == DL_CASE_NAME)
        .expect("DLケースが作られるはず");

    // 鉛直荷重の総和保存: スラブ DL + 躯体自重（小梁自重を含む）。
    let dl_total: f64 = dl
        .member
        .iter()
        .map(|m| match m.kind {
            MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
            MemberLoadKind::Point { p, .. } => p,
        })
        .sum::<f64>()
        + dl.nodal.iter().map(|nl| -nl.values[2]).sum::<f64>();
    let (sw_nodal, sw_member) = squid_n_load::self_weight::self_weight_case_content(
        &app.model,
        &squid_n_core::model::LoadCfg::default(),
    );
    let sw_total: f64 = sw_member
        .iter()
        .map(|m| match m.kind {
            MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
            MemberLoadKind::Point { p, .. } => p,
        })
        .sum::<f64>()
        + sw_nodal.iter().map(|nl| -nl.values[2]).sum::<f64>();
    let slab_dl = 0.005 * 8000.0 * 6000.0;
    assert!(
        (dl_total - (slab_dl + sw_total)).abs() < 1e-6 * (slab_dl + sw_total),
        "DL 合計 {dl_total} = スラブ {slab_dl} + 自重 {sw_total} のはず（荷重を捨てない）"
    );

    // 小梁反力（+小梁自重の半分）が大梁 y=0（ElemId(4)）の中間集中荷重 a=4000 に載る。
    let point_on_girder = dl
        .member
        .iter()
        .filter(|m| m.elem == ElemId(4))
        .find_map(|m| match m.kind {
            MemberLoadKind::Point { a, p } => Some((a, p)),
            _ => None,
        })
        .expect("大梁 y=0 に小梁反力の中間集中荷重（CMQ）が載るはず");
    assert!(
        (point_on_girder.0 - 4000.0).abs() < 1.0,
        "集中荷重位置はスパン中央 4000mm のはず: {}",
        point_on_girder.0
    );
    assert!(point_on_girder.1 > 0.0);

    // 要素非接続の小梁支持節点（8, 9）への節点荷重は残らない（全て CMQ 変換済み）。
    assert!(
        dl.nodal
            .iter()
            .all(|nl| nl.node != NodeId(8) && nl.node != NodeId(9)),
        "小梁支持節点への節点荷重は大梁の集中荷重へ変換されるはず"
    );

    // そのまま線形静的解析が成功する（小梁支持節点は解析自由度から除外される）。
    let dl_id = dl.id;
    app.run_linear_static(dl_id);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(app
        .results
        .as_ref()
        .unwrap()
        .statics
        .iter()
        .any(|(k, _)| *k == StaticCaseKey::User(dl_id)));
}

/// 診断: 支点が1つも定義されていない空モデルは支点なし Error を返す。
#[test]
fn test_run_diagnostics_flags_missing_support() {
    let mut app = App::default();
    app.run_diagnostics();
    assert!(app
        .diagnostics
        .iter()
        .any(|d| d.severity == DiagSeverity::Error && d.message.contains("支点")));
}

/// サンプル（門型ラーメン、柱脚固定）では支点なし Error が出ない
/// （他の診断が出るかどうかはモデル次第のため断定しない）。
#[test]
fn test_run_diagnostics_no_missing_support_for_sample() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_diagnostics();
    assert!(!app
        .diagnostics
        .iter()
        .any(|d| d.severity == DiagSeverity::Error && d.message.contains("支点")));
}

/// 断面未割当の部材があれば Warning が出て、target がその部材を指す。
#[test]
fn test_run_diagnostics_flags_unassigned_section() {
    let mut model = crate::sample::portal_frame();
    let target_id = model.elements[0].id;
    model.elements[0].section = None;

    let mut app = App::default();
    app.load_model(model);
    app.run_diagnostics();

    let diag = app
        .diagnostics
        .iter()
        .find(|d| matches!(d.target, Some(DiagTarget::Member(id)) if id == target_id))
        .expect("断面未割当の Warning が出るはず");
    assert_eq!(diag.severity, DiagSeverity::Warning);
    assert!(diag.message.contains("断面"));
}

/// `mark_edited` 後は診断が再実行待ち（stale）に戻る。
#[test]
fn test_mark_edited_marks_diagnostics_stale() {
    let mut app = App::default();
    app.run_diagnostics();
    assert!(!app.staleness.diagnostics_stale);
    app.staleness.mark_edited();
    assert!(app.staleness.diagnostics_stale);
}

// ---- グリッド操作（§9.2 ヘッドレス UI テスト。T5） ----
// widget（egui）を介さず、grid_core の純ロジックと NodeGridAdapter を
// widget と同じ順序で呼び、モデル・undo の挙動を検証する。

mod grid_headless {
    use super::super::node_grid::NodeGridAdapter;
    use super::*;
    use crate::grid::{
        commit_cell_text, plan_paste, CellRef, CommitOutcome, GridAdapter, PastePlan,
    };
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, NodeId};
    use squid_n_core::model::Node;

    /// 節点を n 個持つ App を作る（座標は X=1000*i で識別できるようにする）
    fn app_with_nodes(n: usize) -> App {
        let mut app = App::default();
        for i in 0..n {
            app.model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: [i as f64 * 1000.0, 0.0, 0.0],
                restraint: Dof6Mask::FREE,
                mass: None,
                story: None,
            });
        }
        app
    }

    /// 節点 a-b を結ぶ梁を追加する（参照中の節点を作るため）
    fn push_beam(app: &mut App, a: u32, b: u32) {
        use squid_n_core::model::{ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis};
        app.model.elements.push(ElementData {
            id: ElemId(app.model.elements.len() as u32),
            kind: ElementKind::Beam,
            nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }

    fn block(rows: &[&[&str]]) -> Vec<Vec<String>> {
        rows.iter()
            .map(|r| r.iter().map(|s| s.to_string()).collect())
            .collect()
    }

    /// widget の do_paste と同じ順序で plan_paste → apply_block を行う
    fn paste(
        app: &mut App,
        block: &[Vec<String>],
        anchor: CellRef,
    ) -> Result<PastePlan, Vec<String>> {
        let mut adapter = NodeGridAdapter {
            model: &mut app.model,
            undo: &mut app.undo,
            edited: false,
        };
        let plan = plan_paste(block, anchor, adapter.rows(), adapter.cols(), |r, c, t| {
            adapter.validate_cell(r, c, t)
        })?;
        adapter.apply_block(&plan.set, plan.extra_rows);
        Ok(plan)
    }

    /// §9.2: ペースト適用が複合コマンド 1 個になり、undo 1 回で行追加ごと戻る。
    /// はみ出し行の自動追加（extra_rows）も検証する
    #[test]
    fn test_grid_paste_composite_single_undo_restores_appended_rows() {
        let mut app = app_with_nodes(2);
        let before = app.model.clone();
        // 2 行の表に 3 行 ×2 列を貼る → 1 行はみ出し（自動追加）
        let plan = paste(
            &mut app,
            &block(&[&["10", "20"], &["30", "40"], &["50", "60"]]),
            CellRef { row: 0, col: 0 },
        )
        .expect("正常ペーストは成功する");
        assert_eq!(plan.extra_rows, 1, "はみ出し行数の算定");
        assert_eq!(app.model.nodes.len(), 3, "確認なしで行が自動追加される");
        assert_eq!(app.model.nodes[0].coord, [10.0, 20.0, 0.0]);
        assert_eq!(app.model.nodes[2].coord, [50.0, 60.0, 0.0]);
        assert_eq!(app.undo.undo_label(), Some("節点座標の貼り付け"));
        // undo 1 回で行追加ごと元に戻る
        app.undo.undo(&mut app.model);
        assert!(app.model.eq_ignoring_dofmap(&before));
    }

    /// §9.2: 全体拒否時にモデルが一切変化しない（undo スタックにも積まれない）
    #[test]
    fn test_grid_paste_reject_leaves_model_untouched() {
        let mut app = app_with_nodes(2);
        let before = app.model.clone();
        let err = paste(
            &mut app,
            &block(&[&["1", "abc"]]),
            CellRef { row: 0, col: 0 },
        )
        .expect_err("不正セルを含むペーストは全体拒否");
        assert_eq!(err.len(), 1);
        assert!(app.model.eq_ignoring_dofmap(&before));
        assert!(!app.undo.can_undo(), "拒否時は undo 履歴にも積まれない");
    }

    /// §9.2: 空モデル（節点 0）への貼り付けがプレースホルダ経由で成立し、
    /// 自動で N 行追加 → undo 1 回で空に戻る
    #[test]
    fn test_grid_paste_into_empty_model() {
        let mut app = app_with_nodes(0);
        let plan = paste(
            &mut app,
            &block(&[&["1", "2", "3"], &["4", "5", "6"]]),
            CellRef { row: 0, col: 0 },
        )
        .expect("空モデルへの貼り付けが成立する");
        assert_eq!(plan.extra_rows, 2);
        assert_eq!(app.model.nodes.len(), 2);
        assert_eq!(app.model.nodes[0].coord, [1.0, 2.0, 3.0]);
        assert_eq!(app.model.nodes[1].coord, [4.0, 5.0, 6.0]);
        app.undo.undo(&mut app.model);
        assert!(app.model.nodes.is_empty(), "undo 1 回で空に戻る");
    }

    /// §9.2: Delete クリアの適用と undo（節点テーブルのクリア = 0 埋め）
    #[test]
    fn test_grid_clear_cells_and_undo() {
        let mut app = app_with_nodes(3);
        let before = app.model.clone();
        let n = {
            let mut adapter = NodeGridAdapter {
                model: &mut app.model,
                undo: &mut app.undo,
                edited: false,
            };
            adapter.clear_cells(&[(1, 0), (1, 1), (2, 0)])
        };
        assert_eq!(n, 3);
        assert_eq!(app.model.nodes[1].coord, [0.0, 0.0, 0.0]);
        assert_eq!(app.model.nodes[2].coord, [0.0, 0.0, 0.0]);
        app.undo.undo(&mut app.model);
        assert!(app.model.eq_ignoring_dofmap(&before), "undo 1 回で復元");
    }

    /// §9.2: プレースホルダでの編集確定が行追加＋値設定の 1 コマンドになり、
    /// undo 1 回で行ごと戻る
    #[test]
    fn test_grid_placeholder_commit_appends_row() {
        let mut app = app_with_nodes(1);
        let outcome = {
            let mut adapter = NodeGridAdapter {
                model: &mut app.model,
                undo: &mut app.undo,
                edited: false,
            };
            // プレースホルダ行（row == rows()）の Y 列に確定
            commit_cell_text(&mut adapter, CellRef { row: 1, col: 1 }, "6000")
        };
        assert_eq!(outcome, CommitOutcome::Applied { appended: true });
        assert_eq!(app.model.nodes.len(), 2);
        assert_eq!(
            app.model.nodes[1].coord,
            [0.0, 6000.0, 0.0],
            "他列は既定値 0"
        );
        app.undo.undo(&mut app.model);
        assert_eq!(app.model.nodes.len(), 1, "undo 1 回で行追加ごと戻る");
    }

    /// §9.2: Backspace（空バッファでの編集開始）→ Enter の空確定は「変更なし」
    /// であり、範囲クリアもモデル変更も undo 履歴も発生しない
    #[test]
    fn test_grid_empty_commit_is_no_change() {
        let mut app = app_with_nodes(2);
        let before = app.model.clone();
        let outcome = {
            let mut adapter = NodeGridAdapter {
                model: &mut app.model,
                undo: &mut app.undo,
                edited: false,
            };
            commit_cell_text(&mut adapter, CellRef { row: 0, col: 0 }, "  ")
        };
        assert_eq!(outcome, CommitOutcome::NoChange);
        assert!(app.model.eq_ignoring_dofmap(&before));
        assert!(!app.undo.can_undo());
    }

    /// §9.2: 不正値の編集確定は Rejected でモデル無変化
    #[test]
    fn test_grid_invalid_commit_is_rejected() {
        let mut app = app_with_nodes(2);
        let before = app.model.clone();
        let outcome = {
            let mut adapter = NodeGridAdapter {
                model: &mut app.model,
                undo: &mut app.undo,
                edited: false,
            };
            commit_cell_text(&mut adapter, CellRef { row: 0, col: 0 }, "abc")
        };
        assert!(matches!(outcome, CommitOutcome::Rejected(_)));
        assert!(app.model.eq_ignoring_dofmap(&before));
        assert!(!app.undo.can_undo());
    }

    /// §9.2: 行削除が複合コマンド 1 個になり、undo 1 回で ID 繰り上げごと復元される
    #[test]
    fn test_grid_delete_rows_single_undo_restores_ids() {
        let mut app = app_with_nodes(5);
        push_beam(&mut app, 0, 4); // 削除対象外の節点を結ぶ梁（ID 繰り上げの検証用）
        let before = app.model.clone();
        {
            let mut adapter = NodeGridAdapter {
                model: &mut app.model,
                undo: &mut app.undo,
                edited: false,
            };
            // widget は昇順で渡す（降順化はアダプタの責務）
            adapter.delete_rows(&[1, 3]);
        }
        assert_eq!(app.model.nodes.len(), 3);
        assert_eq!(
            app.model.elements[0].nodes[1],
            NodeId(2),
            "参照 ID が繰り上がる"
        );
        assert_eq!(app.undo.undo_label(), Some("節点 2 行の削除"));
        app.undo.undo(&mut app.model);
        assert!(
            app.model.eq_ignoring_dofmap(&before),
            "undo 1 回で ID・参照ごと復元"
        );
    }

    /// §9.2: 参照中の節点を含む行削除は validate_row_deletion が拒否し、
    /// （widget が delete_rows を呼ばないため）モデルは一切変化しない
    #[test]
    fn test_grid_delete_referenced_row_is_rejected() {
        let mut app = app_with_nodes(3);
        push_beam(&mut app, 0, 1);
        let before = app.model.clone();
        let (blocked, free) = {
            let adapter = NodeGridAdapter {
                model: &mut app.model,
                undo: &mut app.undo,
                edited: false,
            };
            (
                adapter.validate_row_deletion(1),
                adapter.validate_row_deletion(2),
            )
        };
        assert!(blocked.is_err(), "参照中の節点は削除不可");
        assert!(blocked.unwrap_err().contains("参照"), "参照元を理由に示す");
        assert!(free.is_ok());
        // widget は 1 行でも拒否があれば delete_rows を呼ばない（all-or-nothing）
        assert!(app.model.eq_ignoring_dofmap(&before));
        assert!(!app.undo.can_undo());
    }
}
/// 剛床（階の剛床定義）に載る梁でも、応力図・検定比図の元データ
/// （`member_forces` と `member_checks`）が生成されること（回帰）。
///
/// `ForceRegime::Auto` の「剛床に載る水平材＝材端集中ばね」は非線形解析だけの
/// 振り分けであり、線形解析の要素生成が従うと当該梁の内力が丸ごと欠落し、
/// 3D ビューの応力図・検定比図に何も表示されなくなる
/// （剛床に載らない基礎梁と柱だけが残る症状）。
#[test]
fn test_rigid_floor_beam_has_forces_and_checks() {
    use squid_n_core::model::{DiaphragmDef, Story};

    let mut model = aligned_portal_frame();
    // 梁の両端（node1・node2）を剛床に載せる。自由度の拘束は付けない
    // （`constraints` は空のまま）ので解析可能性は変わらない。
    model.stories.push(Story {
        level_kind: Default::default(),
        structure: Default::default(),
        id: squid_n_core::ids::StoryId(0),
        name: "2F".into(),
        elevation: 3000.0,
        node_ids: vec![NodeId(1), NodeId(2)],
        diaphragms: vec![DiaphragmDef {
            ci_override: None,
            weight: None,
            master: NodeId(1),
            slaves: vec![NodeId(2)],
            rigid: true,
        }],
        seismic_weight: None,
        weight_override: None,
    });

    let mut app = App::default();
    app.load_model(model);
    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let results = app.results.as_ref().expect("解析結果");
    let beam_forces = results
        .member_forces
        .iter()
        .find(|(id, _)| *id == ElemId(1))
        .map(|(_, mf)| mf)
        .expect("剛床に載る梁の部材内力が回収されていない");
    let m_max = beam_forces
        .at
        .iter()
        .map(|(_, f)| f[5].abs())
        .fold(0.0_f64, f64::max);
    assert!(m_max > 1.0, "梁の曲げが 0 のまま: {m_max}");

    let beam_checks = results
        .member_checks
        .iter()
        .find(|m| m.elem == ElemId(1))
        .expect("剛床に載る梁の検定結果が生成されていない");
    assert!(
        !beam_checks.positions.is_empty(),
        "梁の検定位置が空（検定比図で無着色になる）"
    );
}

// ── 準備計算（解析前の前処理） ──────────────────────────────────────

/// 準備計算は階が未定義なら自動生成し、階の分布・剛域・Ai 分布・荷重集計を
/// 揃えて `preparation` へ格納する（解析前に内容を確認できるようにする）。
#[test]
fn test_run_preparation_populates_result() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    assert!(app.preparation.is_none(), "読込直後は未実行");
    assert!(app.staleness.preparation_stale);

    app.run_preparation();
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(!app.staleness.preparation_stale, "実行後は最新化される");

    let prep = app.preparation.as_ref().expect("準備計算の結果があるはず");
    // 階が未定義だったので自動生成される。
    assert_eq!(app.model.stories.len(), 1);
    assert_eq!(prep.stories.len(), 1);
    let story = &prep.stories[0];
    assert!(story.weight > 0.0, "地震用重量が算定される");
    assert_eq!(story.cumulative_weight, story.weight, "最上階の ΣWj = Wi");
    // 階高は GL（柱脚レベル 0）から梁レベル 3500mm まで。
    assert!((story.height - 3500.0).abs() < 1e-6, "{}", story.height);

    // 建物概要。
    assert_eq!(prep.summary.n_nodes, app.model.nodes.len());
    // 剛床代表節点（面外拘束を持つ）は支点として数えない。
    assert_eq!(prep.summary.n_supports, 2, "柱脚 2 点が固定");
    assert!((prep.summary.height_mm - 3500.0).abs() < 1e-6);
    assert!(prep.summary.total_seismic_weight > 0.0);

    // Ai 分布（略算周期）。1 層なので α=Ai=1、Ci = Z·Rt·Ai·C0。
    let sm = prep.seismic.as_ref().expect("Ai 分布が算定されるはず");
    assert_eq!(sm.rows.len(), 1);
    assert!((sm.rows[0].alpha - 1.0).abs() < 1e-9);
    assert!((sm.rows[0].ai - 1.0).abs() < 1e-9);
    assert!((sm.rows[0].ci - sm.z * sm.rt * sm.c0).abs() < 1e-9);
    // Qi = Ci·Wi、最上層なので Pi = Qi、基部せん断力 Q1 = Qi。
    assert!((sm.rows[0].qi - sm.rows[0].ci * sm.rows[0].weight).abs() < 1e-6);
    assert!((sm.rows[0].pi - sm.rows[0].qi).abs() < 1e-9);
    assert!((sm.base_shear - sm.rows[0].qi).abs() < 1e-9);
    assert!(!sm.clamped_negative_pi);

    // 風圧力は X・Y の両方向を算定する（見付幅が風向で変わるため）。
    // サンプルは XZ 平面の平面架構なので、X 方向の風（見付幅は Y 方向の
    // 座標範囲 = 0）は算定できず、理由が `wind_note` に入る。
    assert_eq!(prep.wind.len(), 1, "{:?}", prep.wind_note);
    assert_eq!(prep.wind[0].dir, SeismicDir::Y);
    assert!(prep.wind[0].q > 0.0);
    assert!(
        prep.wind_note.as_ref().unwrap().contains("X 方向"),
        "{:?}",
        prep.wind_note
    );

    // 荷重集計（DL には自重・床荷重が同期されるので鉛直下向き = 負）。
    let dl = prep
        .load_cases
        .iter()
        .find(|r| r.name == squid_n_core::model::DL_CASE_NAME)
        .expect("DL ケースが同期されるはず");
    assert!(dl.sum_force[2] < 0.0, "{:?}", dl.sum_force);
}

/// 準備計算の Ai 分布は、実際に EX ケースへ同期される水平力の合計と一致する
/// （確認表示と解析入力が同じ算定に基づくことの担保）。
#[test]
fn test_preparation_ai_matches_synced_seismic_case() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_preparation();

    let sm = app
        .preparation
        .as_ref()
        .unwrap()
        .seismic
        .as_ref()
        .expect("Ai 分布");
    let sum_pi: f64 = sm.rows.iter().map(|r| r.pi).sum();

    let ex = app
        .model
        .load_cases
        .iter()
        .find(|lc| lc.name == squid_n_core::model::EX_CASE_NAME)
        .expect("EX ケースが同期されるはず");
    let sum_fx: f64 = ex.nodal.iter().map(|n| n.values[0]).sum();
    assert!(
        (sum_pi - sum_fx).abs() < 1e-6 * sum_pi.abs().max(1.0),
        "ΣPi={} vs ΣFx={}",
        sum_pi,
        sum_fx
    );
}

/// 準備計算は剛域を算定してモデルへ反映し、その内容を一覧化する。
/// 可とう長 L' = L − λi − λj・剛域比が表の値と整合すること。
#[test]
fn test_preparation_lists_rigid_zones() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_preparation();

    let prep = app.preparation.as_ref().unwrap();
    assert_eq!(
        prep.rigid_zone_candidates,
        app.model.elements.len(),
        "サンプルは全要素が梁要素"
    );
    // サンプルは S 造のため剛域長 λ は 0 だが、危険断面位置の基準となる
    // 柱フェース距離は付く（λ とフェース距離は別概念。設計書 §6.2.1）。
    assert!(
        !prep.rigid_zones.is_empty(),
        "柱梁が直交接続するのでフェース距離が付く"
    );
    assert!(
        prep.rigid_zones
            .iter()
            .any(|r| r.face_i > 0.0 || r.face_j > 0.0),
        "フェース距離が算定されるはず"
    );
    for r in &prep.rigid_zones {
        assert!(r.length > 0.0);
        assert!((r.clear_length - (r.length - r.zone_i - r.zone_j)).abs() < 1e-9);
        assert!((r.ratio - (r.zone_i + r.zone_j) / r.length).abs() < 1e-12);
        // モデル側の剛域長と一致する（表示が実際の解析入力と同じであること）。
        let elem = app
            .model
            .elements
            .iter()
            .find(|e| e.id == r.elem)
            .expect("部材");
        assert_eq!(elem.rigid_zone.length_i, r.zone_i);
        assert_eq!(elem.rigid_zone.length_j, r.zone_j);
    }
}

/// 階が未定義のまま準備計算の集計だけを行うと、Ai 分布・風圧力は算定できず
/// 理由（`*_note`）が入る。集計自体はエラーにならない。
#[test]
fn test_preparation_without_stories_reports_notes() {
    let mut app = App::default();
    // 階を作らずに解析だけ実行する経路（`ensure_preparation`）を通す。
    app.load_model(crate::sample::portal_frame());
    app.run_linear_static(LoadCaseId(0));

    let prep = app.preparation.as_ref().expect("解析前に準備計算が走る");
    assert!(prep.stories.is_empty());
    assert!(prep.seismic.is_none());
    assert!(prep.seismic_note.as_ref().unwrap().contains("階"));
    assert!(prep.wind.is_empty());
    assert!(prep.wind_note.is_some());
    // 剛域・荷重集計は階に依存しないので算定される。
    assert!(!prep.rigid_zones.is_empty());
    assert!(!prep.load_cases.is_empty());
}

/// 解析の実行は準備計算を最新化するが、階の自動生成は行わない
/// （解析実行が暗黙にモデルの階構成を書き換えないため）。
#[test]
fn test_analysis_ensures_preparation_without_generating_stories() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    assert!(app.staleness.preparation_stale);

    app.run_linear_static(LoadCaseId(0));
    assert!(app.last_error.is_none(), "{:?}", app.last_error);
    assert!(!app.staleness.preparation_stale, "解析前に準備計算が走る");
    assert!(app.preparation.is_some());
    assert!(app.model.stories.is_empty(), "階は自動生成しない");
}

/// モデル編集で準備計算は stale に戻り、再実行で最新化される。
#[test]
fn test_mark_edited_marks_preparation_stale() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_preparation();
    assert!(!app.staleness.preparation_stale);
    app.staleness.mark_edited();
    assert!(app.staleness.preparation_stale);
    app.run_preparation();
    assert!(!app.staleness.preparation_stale);
}

/// 精算周期（SemiPrecise）で固有値解析が未実行なら Ai 分布は算定せず、
/// 固有値解析の実行を促す理由を返す（勝手に略算へフォールバックしない）。
#[test]
fn test_preparation_semiprecise_without_eigen_reports_note() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    app.analysis_cfg.ai_mode = AiMode::SemiPrecise;
    app.staleness.mark_edited();
    app.run_preparation();

    let prep = app.preparation.as_ref().unwrap();
    assert!(prep.seismic.is_none());
    assert!(
        prep.seismic_note.as_ref().unwrap().contains("固有値解析"),
        "{:?}",
        prep.seismic_note
    );

    // 固有値解析を実行すると、その1次周期で Ai 分布が算定できるようになる。
    app.run_eigen(3);
    app.run_preparation();
    let prep = app.preparation.as_ref().unwrap();
    let sm = prep.seismic.as_ref().expect("固有値実行後は算定できる");
    assert_eq!(sm.t_mode, AiMode::SemiPrecise);
    let t1 = app.results.as_ref().unwrap().modal.as_ref().unwrap().period[0];
    assert!((sm.t - t1).abs() < 1e-12);
}

/// モデル読込は準備計算の結果・診断をリセットする（前モデルの結果が残らない）。
#[test]
fn test_load_model_resets_preparation() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_preparation();
    assert!(app.preparation.is_some());

    app.load_model(crate::sample::portal_frame());
    assert!(app.preparation.is_none());
    assert!(app.diagnostics.is_empty());
    assert!(app.staleness.preparation_stale);
}

/// 準備計算の CSV には階の分布・Ai 分布・剛域・荷重集計の各セクションが出る。
#[test]
fn test_build_preparation_csv() {
    let mut app = App::default();
    assert!(
        crate::summary::build_preparation_csv(&app).is_empty(),
        "未実行なら空"
    );

    app.load_model(crate::sample::portal_frame());
    app.run_preparation();
    let csv = crate::summary::build_preparation_csv(&app);
    for section in [
        "[建物概要]",
        "[階の分布]",
        "[地震力 (Ai分布)]",
        "[剛域]",
        "[荷重集計]",
    ] {
        assert!(csv.contains(section), "{section} が無い:\n{csv}");
    }
    assert!(csv.contains("階,Wi[kN],ΣWj[kN],αi,Ai,Ci,Qi[kN],Pi[kN],種別"));
}

/// 準備計算は断面性能（断面諸量・使用部材数・材料）を一覧化する。
#[test]
fn test_preparation_lists_section_properties() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_preparation();

    let prep = app.preparation.as_ref().unwrap();
    assert_eq!(prep.sections.len(), app.model.sections.len());
    for (row, sec) in prep.sections.iter().zip(app.model.sections.iter()) {
        // 表示値はモデルが持つ解析入力そのもの（ここで再計算はしない）。
        assert_eq!(row.section, sec.id);
        assert_eq!(row.area, sec.area);
        assert_eq!(row.iy, sec.iy);
        assert_eq!(row.iz, sec.iz);
        assert_eq!(row.j, sec.j);
        assert!(row.shape_label.is_some(), "形状定義を持つ断面");
        assert!(row.n_elements > 0, "サンプルは全断面が使われている");
        // 断面二次半径 i = √(I/A)。
        assert!((row.ry - (sec.iy / sec.area).sqrt()).abs() < 1e-9);
        assert!((row.rz - (sec.iz / sec.area).sqrt()).abs() < 1e-9);
        assert_eq!(row.material.as_deref(), Some("SN400B"));
    }
    // 柱 H-300x300 は 2 本、梁 H-400x200 は 1 本。
    assert_eq!(prep.sections[0].n_elements, 2);
    assert_eq!(prep.sections[1].n_elements, 1);
}

/// 準備計算は鋼断面の幅厚比・部材ランクを、断面 × 用途（柱／梁）× 材料で
/// まとめて一覧化する。ランクは保有水平耐力の Ds 算定と同じ判定を用いる。
#[test]
fn test_preparation_lists_width_thickness() {
    use squid_n_design_jp::secondary::width_thickness::SteelMemberUse;

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_preparation();

    let prep = app.preparation.as_ref().unwrap();
    // 柱（H-300x300）2 本 + 梁（H-400x200）1 本 → 2 行にまとまる。
    assert_eq!(prep.width_thickness.len(), 2, "{:?}", prep.width_thickness);

    let col = prep
        .width_thickness
        .iter()
        .find(|r| r.member_use == SteelMemberUse::Column)
        .expect("柱の行");
    assert_eq!(col.n_elements, 2);
    assert_eq!(col.material, "SN400B");
    // H-300x300x10x15: フランジ 300/(2·15)=10、ウェブ (300−30)/10=27 → max 27。
    assert!((col.max_ratio.unwrap() - 27.0).abs() < 1e-9);
    assert!(col.rank.is_some());

    let beam = prep
        .width_thickness
        .iter()
        .find(|r| r.member_use == SteelMemberUse::Beam)
        .expect("梁の行");
    assert_eq!(beam.n_elements, 1);
    // ランク判定は保有水平耐力の Ds 算定と同じ共通関数を通っていること。
    let shape = app.model.sections[1].shape.as_ref().unwrap();
    assert_eq!(
        beam.rank,
        steel_width_thickness_rank(shape, SteelMemberUse::Beam, "SN400B")
    );
}

/// 準備計算の結果はプロジェクトファイル（.scz）へ保存され、読込で復元される。
/// 復元できた場合は実行済み扱い（stale でない）になる。
#[test]
fn test_preparation_persisted_in_project_file() {
    let dir = std::env::temp_dir();
    let path = dir.join("squid_n_prep_persist_test.scz");
    let _ = std::fs::remove_file(&path);

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_preparation();
    let saved = app.preparation.as_ref().unwrap();
    let saved_base_shear = saved.seismic.as_ref().unwrap().base_shear;
    let saved_stories = saved.stories.len();
    let saved_sections = saved.sections.len();
    app.save_project_to(path.clone());
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let mut reopened = App::default();
    reopened.open_project_from(path.clone());
    assert!(reopened.last_error.is_none(), "{:?}", reopened.last_error);

    let restored = reopened
        .preparation
        .as_ref()
        .expect("準備計算の結果が復元されるはず");
    assert_eq!(restored.stories.len(), saved_stories);
    assert_eq!(restored.sections.len(), saved_sections);
    assert!(
        (restored.seismic.as_ref().unwrap().base_shear - saved_base_shear).abs() < 1e-9,
        "基部せん断力が一致しない"
    );
    assert!(
        !reopened.staleness.preparation_stale,
        "復元できたら実行済み扱いにする"
    );

    let _ = std::fs::remove_file(&path);
}

/// 準備計算が古い（モデル編集後に未再実行）状態で保存した場合は同梱せず、
/// 読込側は未実行のままにする（古い結果を最新と誤認させない）。
#[test]
fn test_stale_preparation_not_persisted() {
    let dir = std::env::temp_dir();
    let path = dir.join("squid_n_prep_stale_test.scz");
    let _ = std::fs::remove_file(&path);

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_preparation();
    app.staleness.mark_edited();
    app.save_project_to(path.clone());
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let mut reopened = App::default();
    reopened.open_project_from(path.clone());
    assert!(reopened.last_error.is_none(), "{:?}", reopened.last_error);
    assert!(reopened.preparation.is_none());
    assert!(reopened.staleness.preparation_stale);

    let _ = std::fs::remove_file(&path);
}

/// 準備計算の CSV に断面性能・幅厚比のセクションが出る。
#[test]
fn test_build_preparation_csv_sections() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_preparation();
    let csv = crate::summary::build_preparation_csv(&app);
    assert!(csv.contains("[断面性能]"), "{csv}");
    assert!(csv.contains("[幅厚比・部材ランク]"), "{csv}");
    assert!(csv.contains("H 形鋼"), "{csv}");
}

/// 剛性の割増しも等価換算も生じ得ないモデル（スラブ剛性なし・壁なし・
/// SRC/CFT なし）では、部材剛性の表は空になる（部材ごとの判定を行わない）。
#[test]
fn test_preparation_member_stiffness_empty_for_plain_model() {
    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_preparation();

    let prep = app.preparation.as_ref().unwrap();
    assert!(
        prep.member_stiffness.is_empty(),
        "{:?}",
        prep.member_stiffness
    );
    assert_eq!(
        prep.member_stiffness_candidates,
        app.model.elements.len(),
        "候補数（梁要素数）は数える"
    );
}

/// SRC/CFT 断面の部材は等価断面性能が算定され、部材剛性の表に現れる。
/// 表示値は要素構築が実際に用いる等価換算と一致する。
#[test]
fn test_preparation_member_stiffness_reports_composite_props() {
    use squid_n_core::ids::SectionId;
    use squid_n_core::section_shape::SectionShape;

    let mut model = crate::sample::portal_frame();
    // 柱を CFT 角形に差し替え、充填コンクリート強度 Fc を持つ鋼管材料を割り当てる。
    let cft = SectionShape::CftBox {
        height: 400.0,
        width: 400.0,
        thick: 16.0,
    };
    model.sections[0] = cft.to_section(SectionId(0), "CFT-□400x400x16".into());
    model.materials[0].fc = Some(36.0);

    let mut app = App::default();
    app.load_model(model);
    app.run_preparation();

    let prep = app.preparation.as_ref().unwrap();
    let row = prep
        .member_stiffness
        .iter()
        .find(|r| r.kind == squid_n_design_jp::MemberKind::Column)
        .expect("CFT 柱の行があるはず");
    let c = row.composite.expect("等価断面性能が算定されるはず");
    // 充填コンクリート分だけ鋼管のみより剛性が大きくなる。
    assert!(c.iy > row.section_iy, "{} vs {}", c.iy, row.section_iy);
    assert!(c.area_ax > 0.0);
    // 割増しは無いので実効値＝等価換算値。
    assert_eq!(row.slab_factor, 1.0);
    assert_eq!(row.wall_girder_factor, 1.0);
    assert_eq!(row.effective_iy, c.iy);
    assert_eq!(row.effective_area, c.area_ax);

    // 要素構築が用いる等価換算と一致する（表示と解析入力の一致）。
    let elem = app
        .model
        .elements
        .iter()
        .find(|e| e.id == row.elem)
        .unwrap();
    let props = squid_n_element::beam::composite_props_of(&app.model, elem)
        .expect("要素側でも算定できるはず");
    assert_eq!(props.iy, c.iy);
    assert_eq!(props.area_ax, c.area_ax);
}

/// 解析結果はプロジェクトファイル（.scz）へ保存され、読込で復元される。
/// 復元できた場合は再計算不要（stale でない）扱いになる。
#[test]
fn test_results_persisted_in_project_file() {
    let dir = std::env::temp_dir();
    let path = dir.join("squid_n_results_persist_test.scz");
    let _ = std::fs::remove_file(&path);

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.generate_stories_action();
    app.run_linear_static(LoadCaseId(0));
    app.run_eigen(3);
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let saved = app.results.as_ref().unwrap();
    let saved_disp = saved.statics[0].1.disp.clone();
    let saved_period = saved.modal.as_ref().unwrap().period.clone();
    let saved_checks = saved.member_checks.len();
    let saved_key = app.last_static;
    assert!(!app.staleness.results_stale);

    app.save_project_to(path.clone());
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let mut reopened = App::default();
    reopened.open_project_from(path.clone());
    assert!(reopened.last_error.is_none(), "{:?}", reopened.last_error);

    let restored = reopened.results.as_ref().expect("解析結果が復元されるはず");
    assert_eq!(restored.statics[0].1.disp, saved_disp);
    assert_eq!(restored.modal.as_ref().unwrap().period, saved_period);
    assert_eq!(restored.member_checks.len(), saved_checks);
    assert_eq!(reopened.last_static, saved_key);
    assert!(!reopened.staleness.results_stale);
    assert!(!reopened.staleness.design_stale);
    assert!(reopened.staleness.last_run.is_some());

    let _ = std::fs::remove_file(&path);
}

/// 解析結果が古い（モデル編集後に未再解析）状態で保存した場合は同梱せず、
/// 読込側は結果なし・要再計算のままにする。
#[test]
fn test_stale_results_not_persisted() {
    let dir = std::env::temp_dir();
    let path = dir.join("squid_n_results_stale_test.scz");
    let _ = std::fs::remove_file(&path);

    let mut app = App::default();
    app.load_model(crate::sample::portal_frame());
    app.run_linear_static(LoadCaseId(0));
    app.staleness.mark_edited();
    app.save_project_to(path.clone());
    assert!(app.last_error.is_none(), "{:?}", app.last_error);

    let mut reopened = App::default();
    reopened.open_project_from(path.clone());
    assert!(reopened.last_error.is_none(), "{:?}", reopened.last_error);
    // 結果自体が復元されない（`results_stale` は結果が無い既定状態のまま）。
    assert!(reopened.results.is_none());
    assert_eq!(reopened.last_static, None);

    let _ = std::fs::remove_file(&path);
}
