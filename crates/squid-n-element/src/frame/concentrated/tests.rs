use super::*;
use approx::assert_relative_eq;
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::RigidZone;
use squid_n_material::uniaxial::Bilinear;

fn make_test_beam() -> crate::beam::BeamElement {
    crate::beam::BeamElement {
        id: ElemId(0),
        e: 205000.0,
        g: 78846.15,
        a: 80000.0,
        a_mass: 80000.0,
        iy: 1.0666667e9,
        iz: 1.0666667e9,
        j: 0.0,
        as_y: 66666.67,
        as_z: 66666.67,
        length: 3000.0,
        density: 0.0,
        nodes: [NodeId(0), NodeId(1)],
        axis: crate::transform::LocalFrame {
            rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        },
        rigid: RigidZone::default(),
        end_cond: [
            squid_n_core::model::EndCondition::Fixed,
            squid_n_core::model::EndCondition::Fixed,
        ],
        eval_sections: vec![0.0, 0.5, 1.0],
        section: None,
        material: None,
        committed_disp: [0.0; 12],
        trial_disp: [0.0; 12],
    }
}

fn make_test_element() -> ConcentratedSpringBeam {
    let elastic = make_test_beam();
    let spring_i = Box::new(Bilinear::new(1.0e10, 1.0e20, 0.01));
    let spring_j = Box::new(Bilinear::new(1.0e10, 1.0e20, 0.01));
    ConcentratedSpringBeam::new_one_component(elastic, spring_i, spring_j)
}

fn make_yield_element() -> ConcentratedSpringBeam {
    let mut elastic = make_test_beam();
    elastic.iz = 1.0e8;
    elastic.iy = 1.0e8;
    let spring_i = Box::new(Bilinear::new(1.0e12, 1.0e7, 0.01));
    let spring_j = Box::new(Bilinear::new(1.0e12, 1.0e7, 0.01));
    ConcentratedSpringBeam::new_one_component(elastic, spring_i, spring_j)
}

/// Box<dyn UniaxialMaterial> から Bilinear の降伏値を読み出す（テスト用）。
fn spring_fy(spring: &dyn UniaxialMaterial) -> f64 {
    let mut b = Bilinear::new(1.0, 1.0, 0.0);
    b.deserialize_state(&spring.serialize_state()).unwrap();
    b.fy
}

#[test]
fn test_internal_force_no_double_count() {
    let mut elem = make_test_element();
    let ctx = Ctx {
        model: &squid_n_core::model::Model::default(),
    };
    let state = ElemState::default();
    let du = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, -0.001],
    };
    elem.update_state(&du, true, &ctx);
    let k = elem.tangent_stiffness(&state, &ctx);
    let f = elem.internal_force(&state, &ctx);
    let mut k_u = [0.0; 12];
    for i in 0..12 {
        let mut s = 0.0;
        for j in 0..12 {
            s += k.get(i, j) * elem.elastic.committed_disp[j];
        }
        k_u[i] = s;
    }
    for i in 0..12 {
        assert_relative_eq!(f.data[i], k_u[i], epsilon = 1.0);
    }
}

/// 材端ばねは強軸曲げ（局所 rz。DOF 5・11）にのみ作用し、軸・ねじり・弱軸曲げ
/// （rx・ry）には現れないこと。要素局所系は「y 軸＝断面のせい方向」規約のため、
/// 梁の鉛直曲げ（強軸）は rz まわりであり、塑性ヒンジもこの面に生じる。
#[test]
fn test_dof_only_rz() {
    let mut elem = make_test_element();
    let ctx = Ctx {
        model: &squid_n_core::model::Model::default(),
    };
    let state = ElemState::default();
    let du = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    };
    elem.update_state(&du, true, &ctx);
    let f = elem.internal_force(&state, &ctx);
    assert!(f.data[3].abs() < 1.0, "rx_i should not have spring moment");
    assert!(f.data[4].abs() < 1.0, "ry_i should not have spring moment");
    assert!(f.data[9].abs() < 1.0, "rx_j should not have spring moment");
    assert!(f.data[10].abs() < 1.0, "ry_j should not have spring moment");

    let k = elem.tangent_stiffness(&state, &ctx);
    let k_sym = |i: usize, j: usize| {
        if k.get(i, j) != k.get(j, i) {
            (k.get(i, j) - k.get(j, i)).abs() < 1e-6
        } else {
            true
        }
    };
    for i in 0..12 {
        for j in 0..12 {
            assert!(k_sym(i, j), "K[{i}][{j}] != K[{j}][{i}]");
        }
    }
}

#[test]
fn test_spring_yield() {
    let mut elem = make_yield_element();
    let ctx = Ctx {
        model: &squid_n_core::model::Model::default(),
    };
    let state = ElemState::default();

    let rot_yield = 1.0e7 / 1.0e12;
    let du_large = LocalVec {
        data: smallvec::smallvec![
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            rot_yield * 10.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ],
    };

    let k_elastic = elem.tangent_stiffness(&state, &ctx);

    elem.update_state(&du_large, true, &ctx);
    let k_yielded = elem.tangent_stiffness(&state, &ctx);

    let k55_elastic = k_elastic.get(5, 5);
    let k55_yielded = k_yielded.get(5, 5);
    assert!(
        k55_yielded < k55_elastic * 0.99,
        "yielded tangent should drop: elastic={} yielded={}",
        k55_elastic,
        k55_yielded
    );
}

#[test]
fn test_commit_revert() {
    let mut elem = make_test_element();
    let ctx = Ctx {
        model: &squid_n_core::model::Model::default(),
    };
    let du = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    };

    // ばね変形は相対回転 γ = θn − θb（節点回転 0.001 がばねと可撓部に分配される）。
    elem.update_state(&du, false, &ctx);
    let g1 = elem.trial_rot_i;
    assert!(g1 > 0.0 && g1 < 0.001, "γ は節点回転の一部: {g1}");
    assert_relative_eq!(elem.trial_rot_i + elem.trial_thb_i, 0.001, epsilon = 1e-9);
    assert_relative_eq!(elem.rot_i, 0.0, epsilon = 1e-12);
    elem.revert_state();
    assert_relative_eq!(elem.trial_rot_i, 0.0, epsilon = 1e-12);
    assert_relative_eq!(elem.rot_i, 0.0, epsilon = 1e-12);

    elem.update_state(&du, false, &ctx);
    elem.commit_state();
    assert_relative_eq!(elem.trial_rot_i, g1, epsilon = 1e-12);
    assert_relative_eq!(elem.rot_i, g1, epsilon = 1e-12);

    let du2 = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    };
    elem.update_state(&du2, false, &ctx);
    assert_relative_eq!(elem.trial_rot_i + elem.trial_thb_i, 0.003, epsilon = 1e-9);
    assert!(elem.trial_rot_i > g1, "節点回転の増加でばね変形も増える");
    elem.revert_state();
    assert_relative_eq!(elem.trial_rot_i, g1, epsilon = 1e-12);
    assert_relative_eq!(elem.rot_i, g1, epsilon = 1e-12);
}

#[test]
fn test_snapshot_restore() {
    let mut elem = make_test_element();
    let ctx = Ctx {
        model: &squid_n_core::model::Model::default(),
    };
    let du = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    };

    elem.update_state(&du, true, &ctx);
    let g1 = elem.rot_i;
    assert!(g1 > 0.0, "コミット済みばね変形が非零");
    let snap = elem.snapshot_state();

    let du2 = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    };
    elem.update_state(&du2, false, &ctx);
    assert!(elem.trial_rot_i > g1, "トライアルばね変形が進む");

    elem.restore_state(&*snap);
    assert_relative_eq!(elem.rot_i, g1, epsilon = 1e-12);
    assert_relative_eq!(elem.trial_rot_i, g1, epsilon = 1e-12);
}

#[test]
fn test_tangent_stiffness_symmetric() {
    let mut elem = make_test_element();
    let ctx = Ctx {
        model: &squid_n_core::model::Model::default(),
    };
    let state = ElemState::default();

    let du = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    };
    elem.update_state(&du, true, &ctx);
    let k = elem.tangent_stiffness(&state, &ctx);
    for i in 0..12 {
        for j in 0..12 {
            assert!(
                (k.get(i, j) - k.get(j, i)).abs() < 1e-6,
                "K[{i}][{j}] != K[{j}][{i}]: {} vs {}",
                k.get(i, j),
                k.get(j, i)
            );
        }
    }
}

#[test]
fn test_spring_model_default() {
    let elem = make_test_element();
    assert_eq!(elem.model, SpringModel::OneComponent);
    let k_node = compute_kstar(&elem.elastic, 1.0e10, 1.0e10);
    let u = &elem.elastic.committed_disp;
    let mut f = [0.0; 12];
    for i in 0..12 {
        let mut s = 0.0;
        for j in 0..12 {
            s += k_node.get(i, j) * u[j];
        }
        f[i] = s;
    }
    assert!(
        f.iter().all(|&v| v.abs() < 1e-12),
        "zero disp => zero force"
    );
}

#[test]
fn test_condense_springs_zero_stiffness() {
    let beam = make_test_beam();
    let k_raw = beam.local_stiffness_raw();
    let k_pinned = condense_springs(&k_raw, 0.0, 0.0);
    let k_fixed = condense_springs(&k_raw, 1e30, 1e30);
    assert!(
        k_pinned.get(5, 5) < k_fixed.get(5, 5) * 0.5,
        "pinned rz_i should be much softer than fixed"
    );
}

/// 材端ばねが作用しない自由度（rx・ry）はばね剛性に依らないこと。
#[test]
fn test_rx_ry_unaffected_by_spring() {
    let beam = make_test_beam();
    let k_raw = beam.local_stiffness_raw();
    let k_soft = condense_springs(&k_raw, 1.0, 1.0);
    let k_stiff = condense_springs(&k_raw, 1e30, 1e30);
    for &dof in &[3, 4, 9, 10] {
        assert_relative_eq!(k_soft.get(dof, dof), k_stiff.get(dof, dof), epsilon = 1.0);
    }
}

#[test]
fn test_concentrated_spring_checkpoint_roundtrip() {
    let mut elem = make_test_element();
    let ctx = Ctx {
        model: &squid_n_core::model::Model::default(),
    };
    let du = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0005],
    };
    elem.update_state(&du, true, &ctx);

    let snap_before = elem.snapshot_state();
    let checkpoint = elem.serialize_checkpoint();

    let mut restored = make_test_element();
    restored.deserialize_checkpoint(&checkpoint).unwrap();
    let snap_after = restored.snapshot_state();

    // スナップショットの型で比較（ばね変形・可撓端回転 + 弾性梁の committed/trial 変位）
    type Snap = (
        Vec<Box<dyn UniaxialMaterial>>,
        [f64; 4],
        [f64; 4],
        [f64; 12],
        [f64; 12],
    );
    let before = snap_before.downcast_ref::<Snap>().unwrap();
    let after = snap_after.downcast_ref::<Snap>().unwrap();
    for k in 0..4 {
        assert_relative_eq!(before.1[k], after.1[k], epsilon = 1e-12);
        assert_relative_eq!(before.2[k], after.2[k], epsilon = 1e-12);
    }
    // 弾性梁部分の変位もチェックポイントを往復して保存されること
    // （update_state で非零になっているため、欠落していればここで検出される）。
    assert!(
        before.3.iter().any(|v| v.abs() > 1e-15),
        "前提: committed が非零"
    );
    assert!(
        before.4.iter().any(|v| v.abs() > 1e-15),
        "前提: trial が非零"
    );
    for i in 0..12 {
        assert_relative_eq!(before.3[i], after.3[i], epsilon = 1e-12);
        assert_relative_eq!(before.4[i], after.4[i], epsilon = 1e-12);
    }
}
#[test]
fn test_mn_interaction_reduces_spring_yield() {
    // 軸力 |N| = 0.5·n_allow で降伏モーメントが my0 の半分に更新される
    let my0 = 1.0e7;
    let elastic = make_test_beam(); // E=205000, A=80000, L=3000 → EA/L=5.4667e6
    let ea_over_l = 205000.0 * 80000.0 / 3000.0;
    let n_allow = ea_over_l; // 軸変位 0.5mm で |N|/n_allow = 0.5 になるよう設定
    let spring_i = Box::new(Bilinear::new(1.0e12, my0, 0.01));
    let spring_j = Box::new(Bilinear::new(1.0e12, my0, 0.01));
    let mut elem = ConcentratedSpringBeam::new_one_component(elastic, spring_i, spring_j)
        .with_mn_interaction(my0, n_allow);

    let ctx = Ctx {
        model: &squid_n_core::model::Model::default(),
    };
    // j端に軸方向（ローカルx=グローバルx）圧縮変位 0.5mm
    let du = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
    };
    elem.update_state(&du, false, &ctx);
    assert_relative_eq!(spring_fy(&*elem.spring_i), 0.5 * my0, max_relative = 1e-9);
    assert_relative_eq!(spring_fy(&*elem.spring_j), 0.5 * my0, max_relative = 1e-9);

    // トライアル追従化により反復中の増分は累積されるため、+0.5 を追加すると
    // 累積軸変位は 0 に戻り、低減も解除される（Newton 反復として正しい挙動）。
    let du_t = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
    };
    elem.update_state(&du_t, false, &ctx);
    assert_relative_eq!(spring_fy(&*elem.spring_i), my0, max_relative = 1e-9);

    // 引張でも同じ低減（|N| 基準）: 新しい要素に引張変位 +0.5mm を与えて検証。
    let elastic_t = make_test_beam();
    let spring_ti = Box::new(Bilinear::new(1.0e12, my0, 0.01));
    let spring_tj = Box::new(Bilinear::new(1.0e12, my0, 0.01));
    let mut elem_t = ConcentratedSpringBeam::new_one_component(elastic_t, spring_ti, spring_tj)
        .with_mn_interaction(my0, n_allow);
    elem_t.update_state(&du_t, false, &ctx);
    assert_relative_eq!(spring_fy(&*elem_t.spring_i), 0.5 * my0, max_relative = 1e-9);
}

#[test]
fn test_mn_interaction_disabled_keeps_yield() {
    // mn 未設定なら軸力がかかっても降伏モーメントは変わらない
    let my0 = 1.0e7;
    let elastic = make_test_beam();
    let spring_i = Box::new(Bilinear::new(1.0e12, my0, 0.01));
    let spring_j = Box::new(Bilinear::new(1.0e12, my0, 0.01));
    let mut elem = ConcentratedSpringBeam::new_one_component(elastic, spring_i, spring_j);
    let ctx = Ctx {
        model: &squid_n_core::model::Model::default(),
    };
    let du = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
    };
    elem.update_state(&du, false, &ctx);
    assert_relative_eq!(spring_fy(&*elem.spring_i), my0, max_relative = 1e-12);
}

#[test]
fn test_mn_interaction_floor_at_high_axial() {
    // |N| が n_allow を超えても降伏モーメントは 0.02·my0 で下げ止まる
    let my0 = 1.0e7;
    let elastic = make_test_beam();
    let ea_over_l = 205000.0 * 80000.0 / 3000.0;
    let spring_i = Box::new(Bilinear::new(1.0e12, my0, 0.01));
    let spring_j = Box::new(Bilinear::new(1.0e12, my0, 0.01));
    let mut elem = ConcentratedSpringBeam::new_one_component(elastic, spring_i, spring_j)
        .with_mn_interaction(my0, ea_over_l);
    let ctx = Ctx {
        model: &squid_n_core::model::Model::default(),
    };
    let du = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    };
    elem.update_state(&du, false, &ctx);
    assert_relative_eq!(spring_fy(&*elem.spring_i), 0.02 * my0, max_relative = 1e-9);
}

#[test]
fn test_mn_interaction_yield_moment_in_response() {
    // 降伏後のバネモーメント上限が M_lim に低減されることを応答で確認:
    // 軸圧縮 0.5mm（M_lim = 0.5·my0）の状態で大回転を与えると、
    // バネの trial モーメントは ≈ M_lim で頭打ちになる
    let my0 = 1.0e7;
    let elastic = make_test_beam();
    let ea_over_l = 205000.0 * 80000.0 / 3000.0;
    let spring_i = Box::new(Bilinear::new(1.0e12, my0, 0.0));
    let spring_j = Box::new(Bilinear::new(1.0e12, my0, 0.0));
    let mut elem = ConcentratedSpringBeam::new_one_component(elastic, spring_i, spring_j)
        .with_mn_interaction(my0, ea_over_l);
    let ctx = Ctx {
        model: &squid_n_core::model::Model::default(),
    };
    // 軸圧縮 + i端大回転を同時に与える
    let du = LocalVec {
        data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.1, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
    };
    elem.update_state(&du, false, &ctx);
    // バネ i の trial 応力（モーメント）は M_lim = 0.5·my0 で飽和
    let (m, _) = elem.spring_i.clone_box().trial(0.1);
    assert_relative_eq!(m, 0.5 * my0, max_relative = 1e-6);
}

/// 材端ばねは**強軸**（要素局所 rz。断面のせい方向の曲げ）に作用すること。
///
/// 要素局所系は「y 軸＝断面のせい方向」規約のため、強軸曲げは (uy, rz) ブロック
/// （`beam/construct.rs` のクロス変換で断面 iy＝強軸が要素 iz に入る）。
/// 骨格の降伏モーメント（強軸 Zp·σy）と初期剛性（6EI/l、I は強軸）も強軸で与える
/// ため、ばねを弱軸（ry）に入れると強軸曲げが永久に降伏せず保有水平耐力を
/// 過大評価する（危険側）。強軸/弱軸で剛性が大きく異なる断面で確認する。
#[test]
fn test_spring_acts_on_strong_axis_rz() {
    let mut beam = make_test_beam();
    // 要素 iz（Mz 面＝強軸）を iy（My 面＝弱軸）の 10 倍にする。
    beam.iz = 1.0e9;
    beam.iy = 1.0e8;
    let k_raw = beam.local_stiffness_raw();
    let k_stiff = condense_springs(&k_raw, 1e30, 1e30);
    let k_soft = condense_springs(&k_raw, 1.0, 1.0);

    // 強軸（rz: DOF 5・11）はばねが柔らかいと大きく低下する。
    for &dof in &[5usize, 11] {
        assert!(
            k_soft.get(dof, dof) < k_stiff.get(dof, dof) * 0.5,
            "強軸 DOF {dof} にばねが効いていない: soft={} stiff={}",
            k_soft.get(dof, dof),
            k_stiff.get(dof, dof)
        );
    }
    // 弱軸（ry: DOF 4・10）はばねの影響を受けない。
    for &dof in &[4usize, 10] {
        assert_relative_eq!(k_soft.get(dof, dof), k_stiff.get(dof, dof), epsilon = 1.0);
    }
}

/// 材端ばねが十分に剛なら、`compute_kstar` は弾性梁の `local_stiffness()` と一致する。
/// 剛域・端部条件（ピン）を持つ部材でも一致すること＝可撓長と端部条件が
/// 弾性梁と同じ土台で組まれていることの検証。
#[test]
fn test_compute_kstar_matches_elastic_beam_with_rigid_zone_and_pin() {
    use squid_n_core::model::EndCondition;

    let cases: [(RigidZone, [EndCondition; 2], &str); 4] = [
        (
            RigidZone::default(),
            [EndCondition::Fixed; 2],
            "剛域なし・剛接",
        ),
        (
            RigidZone {
                length_i: 300.0,
                length_j: 200.0,
                ..Default::default()
            },
            [EndCondition::Fixed; 2],
            "剛域あり・剛接",
        ),
        (
            RigidZone::default(),
            [EndCondition::Pinned, EndCondition::Fixed],
            "剛域なし・i端ピン",
        ),
        (
            RigidZone {
                length_i: 300.0,
                length_j: 200.0,
                ..Default::default()
            },
            [EndCondition::Pinned, EndCondition::Fixed],
            "剛域あり・i端ピン",
        ),
    ];

    for (rigid, end_cond, label) in cases {
        let mut beam = make_test_beam();
        beam.rigid = rigid;
        beam.end_cond = end_cond;

        let k_ref = beam.local_stiffness();
        // ばね剛性を十分大きく取れば材端ばねは剛接と同等になる。ばね剛性を
        // 梁の回転剛性（≒3e11）より極端に大きくすると静縮約 Kaa−Kab·Kbb⁻¹·Kba が
        // 桁落ちするため、比 1e6〜1e7 程度に留める（直列剛性の誤差は 1e-7 以下）。
        let k = compute_kstar(&beam, 1e18, 1e18);
        for i in 0..12 {
            for j in 0..12 {
                let scale = k_ref.get(i, j).abs().max(1.0);
                assert!(
                    (k.get(i, j) - k_ref.get(i, j)).abs() <= scale * 1e-6,
                    "{label}: K[{i}][{j}] が弾性梁と不一致: {} vs {}",
                    k.get(i, j),
                    k_ref.get(i, j)
                );
            }
        }
    }
}

/// i 端ピンの部材では、材端ばねの有無に依らず i 端の強軸モーメント剛性が 0 に
/// なること（`end_cond` が材端集中ばね梁でも反映されている）。
/// 従来は `local_stiffness_raw()`（端部条件未反映）から組んでいたため、
/// ピン端が両端剛接として解かれていた。
#[test]
fn test_compute_kstar_respects_pinned_end() {
    use squid_n_core::model::EndCondition;
    let mut beam = make_test_beam();
    beam.end_cond = [EndCondition::Pinned, EndCondition::Fixed];
    let k = compute_kstar(&beam, 1e12, 1e12);
    assert!(
        k.get(5, 5).abs() < 1.0,
        "i 端ピンなのに強軸モーメント剛性が残っている: {}",
        k.get(5, 5)
    );
    assert!(
        k.get(11, 11).abs() > 1.0,
        "j 端（剛接）の強軸モーメント剛性は残るはず: {}",
        k.get(11, 11)
    );
}
