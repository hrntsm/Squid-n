//! 系レベル V&V: 剛域を考慮したプッシュオーバーの崩壊機構が Ds へ伝わることの検証。
//!
//! 要素レベル（`squid-n-element` の `fiber/tests.rs`）とプッシュオーバーレベル
//! （`squid-n-solver` の `pushover/tests.rs`）に加えて、
//! **プッシュオーバー → 崩壊機構 → 層 Ds** の一連の経路を通しで確認する。
//!
//! 告示の Ds は「部材群としての種別」に加えて崩壊機構で補正される
//! （[`squid_n_design_jp::secondary::member_rank::story_ds`]: 層崩壊・部分崩壊は
//! 代表ランクを 1 段階不利側へ移す）。剛域は崩壊荷重・崩壊機構の成立時期を変える
//! ため、この経路が破綻していないことを系レベルで担保する。

use squid_n_core::dof::{Dof6Mask, DofMap};
use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    Constraint, ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material,
    MaterialCategory, Model, Node, RigidZone, Section, Story,
};
use squid_n_design_jp::secondary::holding_capacity::{FrameType, MemberRank};
use squid_n_design_jp::secondary::member_rank::story_ds;
use squid_n_solver::analysis::SeismicDir;
use squid_n_solver::constraint::Reducer;
use squid_n_solver::pushover::{pushover_analysis, HingeLevel, MechanismType, PushoverResult};

/// 節点（座標・拘束）を作る補助。
fn node(id: u32, coord: [f64; 3], restraint: Dof6Mask, story: Option<StoryId>) -> Node {
    Node {
        id: NodeId(id),
        coord,
        restraint,
        mass: None,
        story,
        support_spring: None,
    }
}

/// ファイバー要素（2 節点）を作る補助。
fn fiber_elem(id: u32, i: u32, j: u32, section: u32, rigid: f64) -> ElementData {
    ElementData {
        id: ElemId(id),
        kind: ElementKind::Fiber,
        nodes: smallvec::smallvec![NodeId(i), NodeId(j)],
        section: Some(SectionId(section)),
        local_axis: LocalAxis {
            ref_vector: [1.0, 0.0, 0.0],
        },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: RigidZone {
            length_i: rigid,
            length_j: rigid,
            face_i: rigid,
            face_j: rigid,
            ..Default::default()
        },
        plastic_zone: None,
        spring: None,
    }
}

/// 1 層 1 スパンの門形フレーム（柱 100×100・はり 300×300）。
/// `rigid` に柱の剛域長 λ [mm] を与える。はりは十分強くして柱の崩壊機構に固定する。
fn portal_frame(rigid: f64, seismic_weight: f64) -> Model {
    let sec = |id: u32, b: f64, d: f64| -> Section {
        Section {
            id: SectionId(id),
            name: format!("s{id}"),
            area: b * d,
            iy: b * d.powi(3) / 12.0,
            iz: d * b.powi(3) / 12.0,
            j: 1.0e6,
            depth: d,
            width: b,
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
        }
    };
    Model {
        nodes: vec![
            node(0, [0.0, 0.0, 0.0], Dof6Mask::FIXED, None),
            // ファイバー要素は Z 軸柱の頂部ねじり DOF（Rz）に剛性を持たないため拘束する。
            node(1, [0.0, 0.0, 3000.0], Dof6Mask(0b100000), Some(StoryId(0))),
            node(
                2,
                [5000.0, 0.0, 3000.0],
                Dof6Mask(0b100000),
                Some(StoryId(0)),
            ),
            node(3, [5000.0, 0.0, 0.0], Dof6Mask::FIXED, None),
        ],
        elements: vec![
            fiber_elem(0, 0, 1, 0, rigid), // 左柱
            fiber_elem(1, 1, 2, 1, 0.0),   // はり（強い断面・剛域なし）
            fiber_elem(2, 3, 2, 0, rigid), // 右柱
        ],
        sections: vec![sec(0, 100.0, 100.0), sec(1, 300.0, 300.0)],
        materials: vec![Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "steel".to_string(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: Some(0.0),
            fc: None,
            fy: Some(235.0),
        }],
        stories: vec![
            // 階は床であり、先頭は基部の床（`Model::layers` の不変条件）。
            Story {
                level_kind: Default::default(),
                structure: Default::default(),
                id: StoryId(0),
                name: "1F".to_string(),
                elevation: 0.0,
                node_ids: vec![],
                seismic_weight: None,
                weight_override: None,
            },
            Story {
                level_kind: Default::default(),
                structure: Default::default(),
                id: StoryId(1),
                name: "2F".to_string(),
                elevation: 3000.0,
                node_ids: vec![NodeId(1), NodeId(2)],
                seismic_weight: Some(seismic_weight),
                weight_override: None,
            },
        ],
        constraints: vec![Constraint::rigid_diaphragm(
            StoryId(1),
            NodeId(1),
            vec![NodeId(2)],
        )],
        ..Default::default()
    }
}

fn run_pushover(rigid: f64, seismic_weight: f64, steps: usize) -> PushoverResult {
    let model = portal_frame(rigid, seismic_weight);
    let dofmap = DofMap::build(&model);
    let reducer = Reducer::build(&model, &dofmap);
    pushover_analysis(
        &model,
        &dofmap,
        &reducer,
        SeismicDir::X,
        steps,
        0.0,
        false,
        false,
        0.0,
    )
    .expect("pushover should run end-to-end")
}

/// 剛域を考慮したプッシュオーバーでも崩壊機構が成立し、Ds の機構補正が
/// 「補正なし（全体崩壊）」側に決まること。
#[test]
fn 剛域つきプッシュオーバーの崩壊機構がdsへ伝わる() {
    let result = run_pushover(300.0, 600_000.0, 400);

    // 柱両端の 4 ヒンジで運動学的機構が成立している。
    let yielded = result
        .hinges
        .iter()
        .filter(|h| !matches!(h.level, HingeLevel::Crack))
        .count();
    assert!(yielded >= 4, "降伏ヒンジが不足: {yielded}");
    assert!(
        !matches!(result.mechanism, MechanismType::Partial),
        "剛域つきで崩壊機構が成立しない（Partial）"
    );

    // 崩壊機構 → 層 Ds。全体崩壊ならランク補正なし。
    let ranks = [MemberRank::FB];
    let ds = story_ds(&ranks, FrameType::SteelFrame, &result.mechanism);
    let ds_no_correction = squid_n_design_jp::secondary::holding_capacity::ds_value(
        FrameType::SteelFrame,
        MemberRank::FB,
    );
    assert!(
        (ds - ds_no_correction).abs() < 1e-12,
        "全体崩壊なのに Ds が補正された: ds={ds}, 無補正={ds_no_correction}"
    );
}

/// 崩壊機構が成立しない（部分崩壊）ままだと Ds が 1 ランク不利側へ補正されること。
/// 剛域の有無に関わらず成り立つ Ds 側の規則で、上のテストと対になる
/// 「機構判定が Ds を動かす」ことの確認。
#[test]
fn 機構が成立しない場合はdsが不利側へ補正される() {
    // 地震用重量を小さくして降伏に至らせない（＝機構が成立しない）。
    let result = run_pushover(300.0, 1_000.0, 20);
    assert!(
        matches!(result.mechanism, MechanismType::Partial),
        "降伏させていないのに機構が成立した"
    );

    let ranks = [MemberRank::FB];
    let ds = story_ds(&ranks, FrameType::SteelFrame, &result.mechanism);
    let ds_fb = squid_n_design_jp::secondary::holding_capacity::ds_value(
        FrameType::SteelFrame,
        MemberRank::FB,
    );
    let ds_fc = squid_n_design_jp::secondary::holding_capacity::ds_value(
        FrameType::SteelFrame,
        MemberRank::FC,
    );
    assert!(ds_fc > ds_fb, "前提: FC の Ds は FB より大きい");
    assert!(
        (ds - ds_fc).abs() < 1e-12,
        "部分崩壊で Ds が 1 ランク不利側へ補正されていない: ds={ds}, 期待={ds_fc}"
    );
}

/// 剛域は保有水平耐力（崩壊荷重）を可撓長基準へ引き上げること。
/// `squid-n-solver` 側の V&V と同じ現象を、設計側（Ds 経路）の入口である
/// `PushoverResult` の水準で再確認する。
#[test]
fn 剛域は保有水平耐力を可撓長基準へ引き上げる() {
    let collapse_shear = |r: &PushoverResult| -> Option<f64> {
        let mut seen: std::collections::BTreeSet<(u32, u8)> = std::collections::BTreeSet::new();
        let mut events: Vec<_> = r
            .hinges
            .iter()
            .filter(|h| !matches!(h.level, HingeLevel::Crack))
            .collect();
        events.sort_by_key(|h| h.step);
        for h in events {
            seen.insert((h.elem.index() as u32, u8::from(h.pos >= 0.5)));
            if seen.len() >= 4 {
                return r
                    .capacity_curve
                    .iter()
                    .find(|c| c.step == h.step)
                    .map(|c| c.base_shear);
            }
        }
        None
    };
    let q0 = collapse_shear(&run_pushover(0.0, 600_000.0, 400)).expect("剛域なしで機構不成立");
    let q1 = collapse_shear(&run_pushover(300.0, 600_000.0, 400)).expect("剛域ありで機構不成立");
    let theory = 3000.0 / (3000.0 - 2.0 * 300.0);
    let ratio = q1 / q0;
    assert!(
        (ratio / theory - 1.0).abs() < 0.05,
        "保有水平耐力の比が可撓長基準から外れている: 実測 {ratio:.4}, 理論 {theory:.4}"
    );
}
