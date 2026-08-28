use super::*;
use squid_n_core::model::{
    FloorRegion, LoadTransfer, MemberLoadKind, RegionAnchor, Slab, SlabPlate, SlabShape,
};

#[test]
fn test_fem_uniform() {
    let cmq = fem_uniform(10.0, 4000.0);
    let expected = 10.0 * 4000.0_f64.powi(2) / 12.0;
    assert!((cmq.c_i - expected).abs() < 1e-6);
    assert_eq!(cmq.q_i, 10.0 * 4000.0 / 2.0);
}

#[test]
fn test_fem_triangle_spec() {
    let w0 = 10.0_f64;
    let l = 4000.0_f64;
    let cmq = fem_triangle(w0, l);
    let expected = 5.0 * w0 * l.powi(2) / 96.0;
    assert!(
        (cmq.c_i - expected).abs() < 1e-3,
        "FEM={} expected={}",
        cmq.c_i,
        expected
    );
    assert!((expected - 8.3333e6).abs() < 1.0e3, "expected={}", expected);
}

#[test]
fn test_fem_trapezoid_limits() {
    let w0 = 10.0_f64;
    let l = 6000.0_f64;
    // a→L/2（中央区間消滅）→ 対称三角形 5w0L²/96
    let tri_limit = fem_trapezoid(w0, l / 2.0, 0.0, l);
    let expected_tri = 5.0 * w0 * l.powi(2) / 96.0;
    assert!(
        (tri_limit.c_i - expected_tri).abs() / expected_tri < 1e-9,
        "三角形極限 c_i={} expected={}",
        tri_limit.c_i,
        expected_tri
    );
    // a→0（立上り消滅）→ 等分布 w0L²/12
    let uni_limit = fem_trapezoid(w0, 0.0, l, l);
    let expected_uni = w0 * l.powi(2) / 12.0;
    assert!(
        (uni_limit.c_i - expected_uni).abs() / expected_uni < 1e-9,
        "等分布極限 c_i={} expected={}",
        uni_limit.c_i,
        expected_uni
    );
}

#[test]
fn test_fem_trapezoid_numeric() {
    // 一般の台形を数値積分と照合: FEM = (1/L²)∫ w(x)·x·(L-x)² dx
    let w0 = 7.0_f64;
    let l = 5000.0_f64;
    let a = 1500.0_f64;
    let cmq = fem_trapezoid(w0, a, l - 2.0 * a, l);
    let n = 2_000_000;
    let dx = l / n as f64;
    let mut integral = 0.0;
    let mut total = 0.0;
    for k in 0..n {
        let x = (k as f64 + 0.5) * dx;
        let wx = if x < a {
            w0 * x / a
        } else if x > l - a {
            w0 * (l - x) / a
        } else {
            w0
        };
        integral += wx * x * (l - x).powi(2) * dx;
        total += wx * dx;
    }
    let fem_num = integral / (l * l);
    assert!(
        (cmq.c_i - fem_num).abs() / fem_num < 1e-4,
        "c_i={} 数値積分={}",
        cmq.c_i,
        fem_num
    );
    // せん断 q_i+q_j = 総荷重
    assert!(
        (cmq.q_i + cmq.q_j - total).abs() / total < 1e-4,
        "Q合計={} 総荷重={}",
        cmq.q_i + cmq.q_j,
        total
    );
}

/// `sum_fixed_end_moments` の合算値と `expected: Cmq` の c_i/c_j を相対誤差で照合する。
fn assert_fem_matches(loads: &[MemberLoadKind], l: f64, expected: Cmq, label: &str) {
    let (c_i, c_j): (f64, f64) = loads
        .iter()
        .map(|ld| fixed_end_moments(ld, l))
        .fold((0.0, 0.0), |(ai, aj), (ci, cj)| (ai + ci, aj + cj));
    assert!(
        (c_i - expected.c_i).abs() / expected.c_i.abs().max(1e-9) < 1e-6,
        "{label}: c_i={c_i} expected={}",
        expected.c_i
    );
    assert!(
        (c_j - expected.c_j).abs() / expected.c_j.abs().max(1e-9) < 1e-6,
        "{label}: c_j={c_j} expected={}",
        expected.c_j
    );
}

#[test]
fn test_fixed_end_moments_matches_fem_uniform() {
    // emit_shape の Uniform{w} と同じ区間分割: 全長1区間
    let w = 10.0_f64;
    let l = 4000.0_f64;
    let loads = vec![MemberLoadKind::Distributed {
        a: 0.0,
        b: l,
        w1: w,
        w2: w,
    }];
    assert_fem_matches(&loads, l, fem_uniform(w, l), "uniform");
}

#[test]
fn test_fixed_end_moments_matches_fem_triangle() {
    // emit_shape の Triangle{w0}（中央ピーク）と同じ区間分割: 2区間
    let w0 = 10.0_f64;
    let l = 4000.0_f64;
    let mid = l / 2.0;
    let loads = vec![
        MemberLoadKind::Distributed {
            a: 0.0,
            b: mid,
            w1: 0.0,
            w2: w0,
        },
        MemberLoadKind::Distributed {
            a: mid,
            b: l,
            w1: w0,
            w2: 0.0,
        },
    ];
    assert_fem_matches(&loads, l, fem_triangle(w0, l), "triangle");
}

#[test]
fn test_fixed_end_moments_matches_fem_trapezoid() {
    // emit_shape の Trapezoid{w0,a,b}（a: 両端立上り幅、b: 中央フラット幅）と
    // 同じ区間分割: 3区間 [0,a]:0→w0 / [a,a+b]:w0→w0 / [a+b,L]:w0→0
    let w0 = 7.0_f64;
    let l = 5000.0_f64;
    let a = 1500.0_f64;
    let b = l - 2.0 * a;
    let loads = vec![
        MemberLoadKind::Distributed {
            a: 0.0,
            b: a,
            w1: 0.0,
            w2: w0,
        },
        MemberLoadKind::Distributed {
            a,
            b: a + b,
            w1: w0,
            w2: w0,
        },
        MemberLoadKind::Distributed {
            a: a + b,
            b: l,
            w1: w0,
            w2: 0.0,
        },
    ];
    assert_fem_matches(&loads, l, fem_trapezoid(w0, a, b, l), "trapezoid");
}

#[test]
fn test_simple_beam_moment_at_uniform_midspan() {
    let w = 10.0_f64;
    let l = 4000.0_f64;
    let loads = vec![MemberLoadKind::Distributed {
        a: 0.0,
        b: l,
        w1: w,
        w2: w,
    }];
    let expected_mid = w * l * l / 8.0;
    assert!((simple_beam_moment_at(&loads, l, l / 2.0) - expected_mid).abs() / expected_mid < 1e-9);
    // 端部はゼロ、対称性 M(x)=M(L−x)
    assert!(simple_beam_moment_at(&loads, l, 0.0).abs() < 1e-6);
    assert!(simple_beam_moment_at(&loads, l, l).abs() < 1e-6);
    let x = 0.3 * l;
    assert!(
        (simple_beam_moment_at(&loads, l, x) - simple_beam_moment_at(&loads, l, l - x)).abs()
            < 1e-6
    );
}

#[test]
fn test_simple_beam_moment_at_triangle_midspan() {
    let w0 = 10.0_f64;
    let l = 4000.0_f64;
    let mid = l / 2.0;
    let loads = vec![
        MemberLoadKind::Distributed {
            a: 0.0,
            b: mid,
            w1: 0.0,
            w2: w0,
        },
        MemberLoadKind::Distributed {
            a: mid,
            b: l,
            w1: w0,
            w2: 0.0,
        },
    ];
    let expected_mid = w0 * l * l / 12.0;
    assert!((simple_beam_moment_at(&loads, l, mid) - expected_mid).abs() / expected_mid < 1e-9);
    assert!(simple_beam_moment_at(&loads, l, 0.0).abs() < 1e-6);
    assert!(simple_beam_moment_at(&loads, l, l).abs() < 1e-6);
}

#[test]
fn test_simple_beam_moment_at_trapezoid_midspan() {
    let w0 = 10.0_f64;
    let l = 6000.0_f64;
    let a = 1500.0_f64;
    let b = l - 2.0 * a;
    let loads = vec![
        MemberLoadKind::Distributed {
            a: 0.0,
            b: a,
            w1: 0.0,
            w2: w0,
        },
        MemberLoadKind::Distributed {
            a,
            b: a + b,
            w1: w0,
            w2: w0,
        },
        MemberLoadKind::Distributed {
            a: a + b,
            b: l,
            w1: w0,
            w2: 0.0,
        },
    ];
    // 中央値の閉形式 M0=w0(3L²−4a²)/24 と照合
    let expected_mid = w0 * (3.0 * l * l - 4.0 * a * a) / 24.0;
    let mid = simple_beam_moment_at(&loads, l, l / 2.0);
    assert!(
        (mid - expected_mid).abs() / expected_mid < 1e-9,
        "台形中央値={mid} 期待値={expected_mid}"
    );
    // 端部はゼロ、対称性
    assert!(simple_beam_moment_at(&loads, l, 0.0).abs() < 1e-6);
    assert!(simple_beam_moment_at(&loads, l, l).abs() < 1e-6);
    let x = 0.15 * l;
    assert!(
        (simple_beam_moment_at(&loads, l, x) - simple_beam_moment_at(&loads, l, l - x)).abs()
            < 1e-6
    );
}

#[test]
fn test_simple_reactions_and_moment_point_load() {
    let p = 100.0_f64;
    let l = 5000.0_f64;
    let a = 2000.0_f64; // i 端から 2000 の非対称位置
    let load = MemberLoadKind::Point { a, p };
    // 反力 R_i=P·b/L, R_j=P·a/L
    let b = l - a;
    let (r_i, r_j) = simple_reactions(&load, l);
    assert!((r_i - p * b / l).abs() / (p * b / l) < 1e-9);
    assert!((r_j - p * a / l).abs() / (p * a / l) < 1e-9);
    assert!((r_i + r_j - p).abs() < 1e-6, "反力の和は総荷重に一致する");

    let loads = vec![load];
    // 荷重点で M = P·a·b/L
    let expected = p * a * b / l;
    assert!((simple_beam_moment_at(&loads, l, a) - expected).abs() / expected < 1e-9);
    // 端部はゼロ
    assert!(simple_beam_moment_at(&loads, l, 0.0).abs() < 1e-6);
    assert!(simple_beam_moment_at(&loads, l, l).abs() < 1e-6);
    // 区分線形: 荷重点手前でそれぞれ一定勾配
    let x1 = 0.1 * l;
    let x2 = 0.2 * l;
    let slope = simple_beam_moment_at(&loads, l, x2) - simple_beam_moment_at(&loads, l, x1);
    let expected_slope = r_i * (x2 - x1);
    assert!((slope - expected_slope).abs() / expected_slope < 1e-6);
}

#[test]
fn test_simple_beam_moment_at_symmetric_pair_of_asymmetric_points() {
    // 個々には非対称な集中荷重でも、鏡映対で組み合わせれば合算モーメントは対称になる。
    let p = 80.0_f64;
    let l = 6000.0_f64;
    let a = 1800.0_f64; // 非対称位置
    let loads = vec![
        MemberLoadKind::Point { a, p },
        MemberLoadKind::Point { a: l - a, p },
    ];
    let x = 0.2 * l;
    assert!(
        (simple_beam_moment_at(&loads, l, x) - simple_beam_moment_at(&loads, l, l - x)).abs()
            < 1e-6
    );
    assert!(simple_beam_moment_at(&loads, l, 0.0).abs() < 1e-6);
    assert!(simple_beam_moment_at(&loads, l, l).abs() < 1e-6);
}

fn make_square_slab_model(side: f64, method: DistributionMethod, w: f64) -> (Model, Slab) {
    make_rect_slab_model(side, side, method, w)
}

fn make_rect_slab_model(lx: f64, ly: f64, method: DistributionMethod, w: f64) -> (Model, Slab) {
    use squid_n_core::ids::{NodeId, SlabId};
    use squid_n_core::model::{AreaLoad, Node};
    let mk = |id: u32, x: f64, y: f64| Node {
        id: NodeId(id),
        coord: [x, y, 0.0],
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    };
    let model = Model {
        nodes: vec![
            mk(0, 0.0, 0.0),
            mk(1, lx, 0.0),
            mk(2, lx, ly),
            mk(3, 0.0, ly),
        ],
        ..Default::default()
    };
    let slab = Slab {
        id: SlabId(0),
        shape: SlabShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        },
        plate: SlabPlate {
            section: None,
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            usage: None,
            method,
            one_way: None,
        },
    };
    (model, slab)
}

fn total_load(loads: &[BeamLoad]) -> f64 {
    // 鉛直釣合いより、各梁の総荷重 = 端せん断の和 q_i + q_j。
    loads.iter().map(|l| l.cmq.q_i + l.cmq.q_j).sum()
}

#[test]
fn test_slab_conservation_square_triangle() {
    // 設計書 §7.3: 1辺 a=4000, w=0.005 → 総和 = w·a² = 80000 N（厳密）
    let w = 0.005_f64;
    let a = 4000.0_f64;
    let (model, slab) = make_square_slab_model(a, DistributionMethod::TriTrapezoid, w);
    let loads = distribute_slab(&model, &slab);
    let expected = w * a * a;
    assert!(
        (total_load(&loads) - expected).abs() < 1e-6,
        "総和={} expected={}",
        total_load(&loads),
        expected
    );
    // 各大梁ピーク強度 w0 = w·a/2 = 10, FEM = 5·w0·a²/96
    for l in &loads {
        if let LoadShape::Triangle { w0 } = l.shape {
            assert!((w0 - 10.0).abs() < 1e-9, "w0={}", w0);
            let fem = 5.0 * w0 * a * a / 96.0;
            assert!((l.cmq.c_i - fem).abs() < 1e-3, "FEM={}", l.cmq.c_i);
        }
    }
}

#[test]
fn test_slab_conservation_rect_all_methods() {
    let w = 0.005_f64;
    let (lx, ly) = (4000.0_f64, 6000.0_f64);
    let expected = w * lx * ly;
    for method in [
        DistributionMethod::TriTrapezoid,
        DistributionMethod::OneWay,
        DistributionMethod::TributaryArea,
    ] {
        let (model, slab) = make_rect_slab_model(lx, ly, method, w);
        let loads = distribute_slab(&model, &slab);
        assert!(
            (total_load(&loads) - expected).abs() / expected < 1e-9,
            "method={:?} 総和={} expected={}",
            method,
            total_load(&loads),
            expected
        );
    }
}

// ------------------------------------------------------------------
// §1.13: 一方向スラブの伝達方向指定
// ------------------------------------------------------------------

#[test]
fn test_one_way_direction_x_and_y() {
    use squid_n_core::model::OneWayDir;
    let w = 0.004_f64;
    let (lx, ly) = (5000.0_f64, 3000.0_f64);
    let expected = w * lx * ly;

    // one_way=Y: 伝達方向Yに直交する辺0・2（X方向の辺、長さlx）が負担。従来互換と同じ結果。
    let (model, mut slab) = make_rect_slab_model(lx, ly, DistributionMethod::OneWay, w);
    slab.plate.one_way = Some(OneWayDir::Y);
    let loads_y = distribute_slab(&model, &slab);
    assert!((total_load(&loads_y) - expected).abs() / expected < 1e-9);
    for l in &loads_y {
        assert!(matches!(
            l.target,
            LoadTarget::Edge(0) | LoadTarget::Edge(2)
        ));
        if let LoadShape::Uniform { w: wl } = l.shape {
            assert!((wl - w * ly / 2.0).abs() / (w * ly / 2.0) < 1e-9);
        }
    }

    // one_way=X: 伝達方向Xに直交する辺1・3（Y方向の辺、長さly）が負担。
    slab.plate.one_way = Some(OneWayDir::X);
    let loads_x = distribute_slab(&model, &slab);
    assert!((total_load(&loads_x) - expected).abs() / expected < 1e-9);
    for l in &loads_x {
        assert!(matches!(
            l.target,
            LoadTarget::Edge(1) | LoadTarget::Edge(3)
        ));
        if let LoadShape::Uniform { w: wl } = l.shape {
            assert!((wl - w * lx / 2.0).abs() / (w * lx / 2.0) < 1e-9);
        }
    }
}

// ------------------------------------------------------------------
// 多角形床組（矩形でない4辺形・五角形）
// ------------------------------------------------------------------

fn mk_node(id: u32, x: f64, y: f64) -> squid_n_core::model::Node {
    use squid_n_core::ids::NodeId;
    squid_n_core::model::Node {
        id: NodeId(id),
        coord: [x, y, 0.0],
        restraint: Default::default(),
        mass: None,
        story: None,
        support_spring: None,
    }
}

fn polygon_slab_model(pts: &[(f64, f64)], method: DistributionMethod, w: f64) -> (Model, Slab) {
    use squid_n_core::ids::{NodeId, SlabId};
    use squid_n_core::model::AreaLoad;
    let nodes: Vec<_> = pts
        .iter()
        .enumerate()
        .map(|(i, (x, y))| mk_node(i as u32, *x, *y))
        .collect();
    let boundary: Vec<NodeId> = (0..pts.len() as u32).map(NodeId).collect();
    let model = Model {
        nodes,
        ..Default::default()
    };
    let slab = Slab {
        id: SlabId(0),
        shape: SlabShape::Enclosed { boundary },
        plate: SlabPlate {
            section: None,
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            usage: None,
            method,
            one_way: None,
        },
    };
    (model, slab)
}

#[test]
fn test_polygon_trapezoid_conservation() {
    // 矩形でない台形(4頂点、辺2の閉合条件を満たさない) → 多角形経路
    let pts = [
        (0.0, 0.0),
        (6000.0, 0.0),
        (4000.0, 3000.0),
        (1000.0, 3000.0),
    ];
    let w = 0.003_f64;
    let (model, slab) = polygon_slab_model(&pts, DistributionMethod::TriTrapezoid, w);
    // slab_dimensions が None（多角形経路）になることを確認
    assert!(slab_dimensions(&model, &slab).is_none());
    let loads = distribute_slab(&model, &slab);
    assert!(!loads.is_empty());

    let coords: Vec<[f64; 3]> = pts.iter().map(|(x, y)| [*x, *y, 0.0]).collect();
    let sampled_area = total_load(&loads) / w;
    let true_area = polygon_area(&coords);
    assert!(
        (sampled_area - true_area).abs() / true_area < 0.01,
        "sampled={} true={}",
        sampled_area,
        true_area
    );
}

#[test]
fn test_polygon_pentagon_conservation() {
    // 凸五角形
    let pts = [
        (0.0, 0.0),
        (5000.0, 0.0),
        (6000.0, 3000.0),
        (2500.0, 5000.0),
        (-1000.0, 3000.0),
    ];
    let w = 0.0025_f64;
    let (model, slab) = polygon_slab_model(&pts, DistributionMethod::TributaryArea, w);
    let loads = distribute_slab(&model, &slab);
    assert!(!loads.is_empty());
    // 辺インデックスが 0..5 の範囲内。
    for l in &loads {
        match l.target {
            LoadTarget::Edge(e) => assert!(e < 5),
            LoadTarget::Node(_) => panic!("polygon path should not emit node targets"),
            LoadTarget::Span { .. } => panic!("polygon path should not emit span targets"),
        }
    }

    let coords: Vec<[f64; 3]> = pts.iter().map(|(x, y)| [*x, *y, 0.0]).collect();
    let sampled_area = total_load(&loads) / w;
    let true_area = polygon_area(&coords);
    assert!(
        (sampled_area - true_area).abs() / true_area < 0.01,
        "sampled={} true={}",
        sampled_area,
        true_area
    );
}

#[test]
fn test_polygon_one_way_fallback() {
    // one_way 指定でも非矩形なら多角形経路にフォールバックする。
    use squid_n_core::model::OneWayDir;
    let pts = [
        (0.0, 0.0),
        (6000.0, 0.0),
        (4000.0, 3000.0),
        (1000.0, 3000.0),
    ];
    let w = 0.002_f64;
    let (model, mut slab) = polygon_slab_model(&pts, DistributionMethod::OneWay, w);
    slab.plate.one_way = Some(OneWayDir::X);
    let loads = distribute_slab(&model, &slab);
    let coords: Vec<[f64; 3]> = pts.iter().map(|(x, y)| [*x, *y, 0.0]).collect();
    let sampled_area = total_load(&loads) / w;
    let true_area = polygon_area(&coords);
    assert!((sampled_area - true_area).abs() / true_area < 0.01);
}

// ------------------------------------------------------------------
// 片持ちスラブ
// ------------------------------------------------------------------

#[test]
fn test_cantilever_conservation() {
    use squid_n_core::ids::{NodeId, SlabId};
    use squid_n_core::model::AreaLoad;
    let (l_attach, depth) = (4000.0_f64, 1500.0_f64);
    let w = 0.003_f64;
    let nodes = vec![
        mk_node(0, 0.0, 0.0),
        mk_node(1, l_attach, 0.0),
        mk_node(2, l_attach, depth),
        mk_node(3, 0.0, depth),
    ];
    let model = Model {
        nodes,
        ..Default::default()
    };
    // 取付き線 0→1 の左（+Y）側へ `depth` 跳ね出す片持ち床板。
    let slab = Slab {
        id: SlabId(0),
        shape: SlabShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(0), NodeId(1)],
                span: [0.0, 1.0],
                transfer: LoadTransfer::Anchor,
            },
            extent: [depth, depth],
        },
        plate: SlabPlate {
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            ..Default::default()
        },
    };
    let loads = distribute_slab(&model, &slab);
    assert_eq!(loads.len(), 1);
    let l = &loads[0];
    assert!(matches!(l.target, LoadTarget::Edge(0)));
    let expected_total = w * l_attach * depth; // 矩形なので厳密に一致
    assert!(
        (total_load(&loads) - expected_total).abs() / expected_total < 1e-9,
        "総和={} expected={}",
        total_load(&loads),
        expected_total
    );
    if let LoadShape::Uniform { w: wl } = l.shape {
        assert!((wl - w * depth).abs() / (w * depth) < 1e-9);
    } else {
        panic!("expected uniform shape");
    }
}

// ------------------------------------------------------------------
// 小梁2段階伝達
// ------------------------------------------------------------------

#[test]
fn test_joist_two_stage_transfer_conservation() {
    use squid_n_core::ids::{FloorRegionId, NodeId, SlabId};
    use squid_n_core::model::{AreaLoad, JoistLine};
    // 幅方向(X) 9000mm、小梁はY方向に架かり(L_joist=ly=4000)、spacing=3000で
    // 境界から半間隔ずつ離れた2本の小梁（3000,6000）を配置(9000=3*3000)。
    let (lx, ly) = (9000.0_f64, 4000.0_f64);
    let w = 0.0035_f64;
    let spacing = 3000.0_f64;
    let nodes = vec![
        mk_node(0, 0.0, 0.0),
        mk_node(1, lx, 0.0),
        mk_node(2, lx, ly),
        mk_node(3, 0.0, ly),
        // 小梁支持節点(辺0上, 辺2上)
        mk_node(4, 3000.0, 0.0),
        mk_node(5, 3000.0, ly),
        mk_node(6, 6000.0, 0.0),
        mk_node(7, 6000.0, ly),
    ];
    let model = Model {
        nodes,
        ..Default::default()
    };
    let joists = vec![
        JoistLine {
            dir: [0.0, 1.0],
            spacing,
            support: [NodeId(4), NodeId(5)],
            section: None,
            pinned_onto: None,
        },
        JoistLine {
            dir: [0.0, 1.0],
            spacing,
            support: [NodeId(6), NodeId(7)],
            section: None,
            pinned_onto: None,
        },
    ];
    let slab = Slab {
        id: SlabId(0),
        shape: SlabShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        },
        plate: SlabPlate {
            section: None,
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            usage: None,
            method: DistributionMethod::TriTrapezoid,
            one_way: None,
        },
    };
    let mut region = FloorRegion::new(
        FloorRegionId(0),
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
    );
    region.joists = joists;
    region.slab_ids = vec![slab.id];
    let model = Model {
        slabs: vec![slab],
        ..model
    };
    let loads = distribute_region(&model, &region, |_| w);

    let expected_total = w * lx * ly;
    assert!(
        (total_load(&loads) - expected_total).abs() / expected_total < 1e-9,
        "総和={} expected={}",
        total_load(&loads),
        expected_total
    );

    // 節点反力(小梁): 4本(2小梁×両端)、各 R = w*spacing*ly/2
    let node_entries: Vec<_> = loads
        .iter()
        .filter(|l| matches!(l.target, LoadTarget::Node(_)))
        .collect();
    assert_eq!(node_entries.len(), 4);
    let expected_r = w * spacing * ly / 2.0;
    for l in &node_entries {
        if let LoadShape::Point { p, .. } = l.shape {
            assert!((p - expected_r).abs() / expected_r < 1e-9, "R={}", p);
        } else {
            panic!("expected point load");
        }
    }

    // 境界辺(辺1・3、小梁と平行)は remainder=lx-2*spacing=3000 を折半 → 各1500 = spacing/2
    // `distribute_region` は Edge を Span{nodes:[n0,n1], ..} へ解決済み（辺1=N1-N2、辺3=N3-N0）。
    let edge_entries: Vec<_> = loads
        .iter()
        .filter(|l| {
            matches!(
                l.target,
                LoadTarget::Span {
                    nodes: [NodeId(1), NodeId(2)],
                    ..
                } | LoadTarget::Span {
                    nodes: [NodeId(3), NodeId(0)],
                    ..
                }
            )
        })
        .collect();
    assert_eq!(edge_entries.len(), 2);
    for l in &edge_entries {
        if let LoadShape::Uniform { w: wl } = l.shape {
            let expected_wl = w * spacing / 2.0;
            assert!((wl - expected_wl).abs() / expected_wl < 1e-9, "wl={}", wl);
        } else {
            panic!("expected uniform load");
        }
    }
}

/// 実部材化された小梁（支持2節点を両端に持つ実 Beam が存在）は、点反力ではなく
/// 小梁自身への等分布荷重（`LoadTarget::Span`）として分配される（床 Phase B-2）。
/// 総和は保存し、Node 点反力は生じない。
#[test]
fn test_materialized_joist_uses_span_distributed_load() {
    use squid_n_core::ids::{ElemId, FloorRegionId, NodeId, SlabId};
    use squid_n_core::model::{
        AreaLoad, ElementData, ElementKind, EndCondition, ForceRegime, JoistLine, LocalAxis,
    };
    let (lx, ly) = (9000.0_f64, 4000.0_f64);
    let w = 0.0035_f64;
    let spacing = 3000.0_f64;
    let nodes = vec![
        mk_node(0, 0.0, 0.0),
        mk_node(1, lx, 0.0),
        mk_node(2, lx, ly),
        mk_node(3, 0.0, ly),
        mk_node(4, 3000.0, 0.0),
        mk_node(5, 3000.0, ly),
        mk_node(6, 6000.0, 0.0),
        mk_node(7, 6000.0, ly),
    ];
    // 小梁2本を実 Beam として生成（N4-N5, N6-N7）。
    let mk_beam = |id: u32, a: u32, b: u32| ElementData {
        id: ElemId(id),
        kind: ElementKind::Beam,
        nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
        section: None,
        local_axis: LocalAxis {
            ref_vector: [0.0, 0.0, 1.0],
        },
        end_cond: [EndCondition::Pinned, EndCondition::Pinned],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
    };
    let model = Model {
        nodes,
        elements: vec![mk_beam(0, 4, 5), mk_beam(1, 6, 7)],
        ..Default::default()
    };
    let joists = vec![
        JoistLine {
            dir: [0.0, 1.0],
            spacing,
            support: [NodeId(4), NodeId(5)],
            section: None,
            pinned_onto: None,
        },
        JoistLine {
            dir: [0.0, 1.0],
            spacing,
            support: [NodeId(6), NodeId(7)],
            section: None,
            pinned_onto: None,
        },
    ];
    let slab = Slab {
        id: SlabId(0),
        shape: SlabShape::Enclosed {
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
        },
        plate: SlabPlate {
            section: None,
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            usage: None,
            method: DistributionMethod::TriTrapezoid,
            one_way: None,
        },
    };
    let mut region = FloorRegion::new(
        FloorRegionId(0),
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
    );
    region.joists = joists;
    region.slab_ids = vec![slab.id];
    let model = Model {
        slabs: vec![slab],
        ..model
    };
    let loads = distribute_region(&model, &region, |_| w);

    // 総和保存。
    let expected_total = w * lx * ly;
    assert!(
        (total_load(&loads) - expected_total).abs() / expected_total < 1e-9,
        "総和={} expected={}",
        total_load(&loads),
        expected_total
    );

    // 実部材化されたので Node 点反力はなく、Span 等分布が2本分できる。
    assert!(
        !loads
            .iter()
            .any(|l| matches!(l.target, LoadTarget::Node(_))),
        "実部材化した小梁は点反力を生じない"
    );
    // 実部材化した小梁自身への Span（支持節点 4-5・6-7）だけを見る。境界辺
    // （小梁と平行な辺1・3の remainder）も `distribute_region` の解決で
    // Span になるため、小梁の支持節点対で絞り込む。
    let span_entries: Vec<_> = loads
        .iter()
        .filter(|l| {
            matches!(
                l.target,
                LoadTarget::Span {
                    nodes: [NodeId(4), NodeId(5)],
                    ..
                } | LoadTarget::Span {
                    nodes: [NodeId(6), NodeId(7)],
                    ..
                }
            )
        })
        .collect();
    assert_eq!(span_entries.len(), 2, "小梁2本が Span 分布になる");
    for l in &span_entries {
        // 各小梁の等分布荷重は w*spacing（トリビュタリ幅）。
        if let LoadShape::Uniform { w: wl } = l.shape {
            let expected = w * spacing;
            assert!((wl - expected).abs() / expected < 1e-9, "wl={wl}");
        } else {
            panic!("expected uniform span load");
        }
    }
}

// ------------------------------------------------------------------
// 剛域考慮CMQ
// ------------------------------------------------------------------

#[test]
fn test_rigid_zone_zero_lambda_matches_existing() {
    let l = 5000.0_f64;
    for mode in [
        RigidZoneCmqMode::IncludeInCmq,
        RigidZoneCmqMode::TransferToColumn,
        RigidZoneCmqMode::Ignore,
    ] {
        // 等分布
        let w = 8.0_f64;
        let res = cmq_with_rigid_zone(&LoadShape::Uniform { w }, l, 0.0, 0.0, mode);
        let expected = fem_uniform(w, l);
        assert!((res.cmq.c_i - expected.c_i).abs() / expected.c_i.abs() < 1e-9);
        assert!((res.cmq.q_i - expected.q_i).abs() / expected.q_i.abs() < 1e-9);
        assert_eq!(res.column_loads, (0.0, 0.0));

        // 三角形
        let w0 = 12.0_f64;
        let res_t = cmq_with_rigid_zone(&LoadShape::Triangle { w0 }, l, 0.0, 0.0, mode);
        let expected_t = fem_triangle(w0, l);
        assert!(
            (res_t.cmq.c_i - expected_t.c_i).abs() / expected_t.c_i.abs() < 1e-4,
            "triangle c_i={} expected={}",
            res_t.cmq.c_i,
            expected_t.c_i
        );
        assert!((res_t.cmq.q_i - expected_t.q_i).abs() / expected_t.q_i.abs() < 1e-4);

        // 台形
        let a = 1200.0_f64;
        let cmq_expected = fem_trapezoid(w0, a, l - 2.0 * a, l);
        let res_tr = cmq_with_rigid_zone(
            &LoadShape::Trapezoid {
                w0,
                a,
                b: l - 2.0 * a,
            },
            l,
            0.0,
            0.0,
            mode,
        );
        assert!(
            (res_tr.cmq.c_i - cmq_expected.c_i).abs() / cmq_expected.c_i.abs() < 1e-4,
            "trapezoid c_i={} expected={}",
            res_tr.cmq.c_i,
            cmq_expected.c_i
        );
        assert!((res_tr.cmq.q_i - cmq_expected.q_i).abs() / cmq_expected.q_i.abs() < 1e-4);
    }
}

#[test]
fn test_rigid_zone_uniform_symmetric_hand_calc() {
    // 全長Lの等分布w、λi=λj=λ（対称）。手計算導出:
    // C_i = wL²/12 + wLλ/6 − wλ²/6 （IncludeInCmqモード）
    let l = 6000.0_f64;
    let lam = 500.0_f64;
    let w = 6.0_f64;
    let res = cmq_with_rigid_zone(
        &LoadShape::Uniform { w },
        l,
        lam,
        lam,
        RigidZoneCmqMode::IncludeInCmq,
    );
    let expected_c = w * l * l / 12.0 + w * l * lam / 6.0 - w * lam * lam / 6.0;
    assert!(
        (res.cmq.c_i - expected_c).abs() / expected_c < 1e-9,
        "c_i={} expected={}",
        res.cmq.c_i,
        expected_c
    );
    // 対称なので c_j = -c_i
    assert!((res.cmq.c_j + res.cmq.c_i).abs() / expected_c < 1e-9);
    // せん断は剛域の有無に関わらず全荷重の半分ずつ(対称・IncludeInCmqで全荷重保存)
    let expected_q = w * l / 2.0;
    assert!((res.cmq.q_i - expected_q).abs() / expected_q < 1e-9);
    assert!((res.cmq.q_j - expected_q).abs() / expected_q < 1e-9);
}

#[test]
fn test_rigid_zone_mode_conservation() {
    // 非対称な剛域長でも、モードによる荷重保存の恒等式が成り立つことを確認。
    let l = 7000.0_f64;
    let lam_i = 300.0_f64;
    let lam_j = 600.0_f64;
    let w = 5.0_f64;
    let total = w * l;

    let include = cmq_with_rigid_zone(
        &LoadShape::Uniform { w },
        l,
        lam_i,
        lam_j,
        RigidZoneCmqMode::IncludeInCmq,
    );
    assert!(
        ((include.cmq.q_i + include.cmq.q_j) - total).abs() / total < 1e-9,
        "IncludeInCmq: q_i+q_j={} total={}",
        include.cmq.q_i + include.cmq.q_j,
        total
    );
    assert_eq!(include.column_loads, (0.0, 0.0));

    let transfer = cmq_with_rigid_zone(
        &LoadShape::Uniform { w },
        l,
        lam_i,
        lam_j,
        RigidZoneCmqMode::TransferToColumn,
    );
    let transfer_total =
        transfer.cmq.q_i + transfer.cmq.q_j + transfer.column_loads.0 + transfer.column_loads.1;
    assert!(
        (transfer_total - total).abs() / total < 1e-9,
        "TransferToColumn total={} expected={}",
        transfer_total,
        total
    );

    let ignore = cmq_with_rigid_zone(
        &LoadShape::Uniform { w },
        l,
        lam_i,
        lam_j,
        RigidZoneCmqMode::Ignore,
    );
    // Ignore と TransferToColumn は梁側 CMQ が一致する（剛域内荷重を加算しない点で同じ）。
    assert!((ignore.cmq.c_i - transfer.cmq.c_i).abs() < 1e-6);
    assert!((ignore.cmq.q_i - transfer.cmq.q_i).abs() < 1e-9);
    assert_eq!(ignore.column_loads, (0.0, 0.0));
    // Ignore は可撓部分の荷重のみを保存する(剛域内荷重は捨てる)。
    let l_flex = l - lam_i - lam_j;
    let expected_flex_total = w * l_flex;
    assert!(
        ((ignore.cmq.q_i + ignore.cmq.q_j) - expected_flex_total).abs() / expected_flex_total
            < 1e-9
    );
}

// ------------------------------------------------------------------
// 出隅の片持ちスラブ（点に取り付く領域）
// ------------------------------------------------------------------

#[test]
fn test_corner_slab_all_load_to_column_node() {
    use squid_n_core::ids::{NodeId, SlabId};
    use squid_n_core::model::AreaLoad;
    let (lx, ly) = (3000.0_f64, 2000.0_f64);
    let w = 0.004_f64;
    let nodes = vec![
        mk_node(0, 0.0, 0.0),
        mk_node(1, lx, 0.0),
        mk_node(2, lx, ly),
        mk_node(3, 0.0, ly),
    ];
    let model = Model {
        nodes,
        ..Default::default()
    };
    // 柱（節点 0）に取り付き、X へ lx・Y へ ly 張り出す出隅の片持ち床板。
    let slab = Slab {
        id: SlabId(0),
        shape: SlabShape::Attached {
            anchor: RegionAnchor::Point(NodeId(0)),
            extent: [lx, ly],
        },
        plate: SlabPlate {
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            ..Default::default()
        },
    };
    let loads = distribute_slab(&model, &slab);
    // 全荷重が単一の節点荷重（boundary[0] = NodeId(0)）としてのみ現れる。
    assert_eq!(loads.len(), 1);
    let l = &loads[0];
    assert_eq!(l.target, LoadTarget::Node(NodeId(0)));
    let expected_total = w * lx * ly;
    assert!(
        (total_load(&loads) - expected_total).abs() / expected_total < 1e-9,
        "総和={} expected={}",
        total_load(&loads),
        expected_total
    );
    if let LoadShape::Point { p, .. } = l.shape {
        assert!((p - expected_total).abs() / expected_total < 1e-9);
    } else {
        panic!("expected point load");
    }
}

#[test]
fn test_point_in_slab_boundary_includes_edges() {
    let (model, slab) = make_square_slab_model(4000.0, DistributionMethod::TriTrapezoid, 0.01);
    assert!(point_in_slab_boundary(&model, &slab, [2000.0, 2000.0]));
    assert!(point_in_slab_boundary(&model, &slab, [0.0, 2000.0]));
    assert!(point_in_slab_boundary(&model, &slab, [0.0, 0.0]));
    assert!(!point_in_slab_boundary(&model, &slab, [-10.0, 2000.0]));
}

// ------------------------------------------------------------------
// 版なし床領域・取り付き領域（線＋柱へ集中）
// ------------------------------------------------------------------

/// 床板を持たない床領域（吹抜けの繋ぎ小梁など）は床荷重を発生させない。
#[test]
fn test_plateless_region_distributes_nothing() {
    use squid_n_core::ids::{FloorRegionId, NodeId};
    let nodes = vec![
        mk_node(0, 0.0, 0.0),
        mk_node(1, 4000.0, 0.0),
        mk_node(2, 4000.0, 4000.0),
        mk_node(3, 0.0, 4000.0),
    ];
    let model = Model {
        nodes,
        ..Default::default()
    };
    let region = FloorRegion::new(
        FloorRegionId(0),
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
    );
    assert!(region.slab_ids.is_empty());
    assert!(
        distribute_region(&model, &region, |s| model.slab_dead_intensity(s)).is_empty(),
        "床板がなければ分配は起きない"
    );
}

/// 取付き線に載る床板で「両端の柱へ集中」を選ぶと、全荷重が両端へ半分ずつ渡る。
#[test]
fn test_attached_line_to_columns_splits_half() {
    use squid_n_core::ids::{NodeId, SlabId};
    use squid_n_core::model::AreaLoad;
    let (l, d, w) = (4000.0_f64, 1500.0_f64, 0.003_f64);
    let nodes = vec![mk_node(0, 0.0, 0.0), mk_node(1, l, 0.0)];
    let model = Model {
        nodes,
        ..Default::default()
    };
    let slab = Slab {
        id: SlabId(0),
        shape: SlabShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(0), NodeId(1)],
                span: [0.0, 1.0],
                transfer: LoadTransfer::Columns,
            },
            extent: [d, d],
        },
        plate: SlabPlate {
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            ..Default::default()
        },
    };
    let loads = distribute_slab(&model, &slab);
    assert_eq!(loads.len(), 2, "両端の 2 節点へ渡る");
    let total: f64 = loads
        .iter()
        .map(|bl| match bl.shape {
            LoadShape::Point { p, .. } => p,
            _ => 0.0,
        })
        .sum();
    // 総和保存: w × 面積（= 取付き長さ × 跳ね出し）。
    assert!((total - w * l * d).abs() / (w * l * d) < 1e-9, "{total}");
    for bl in &loads {
        assert!(matches!(bl.target, LoadTarget::Node(_)));
        if let LoadShape::Point { p, .. } = bl.shape {
            assert!(
                (p - w * l * d / 2.0).abs() / (w * l * d / 2.0) < 1e-9,
                "半分ずつ"
            );
        }
    }
}

/// 代表床板が取り付く床板（片持ち等）だと、小梁ラインがあっても二段階分配を使わない。
#[test]
fn test_uses_joist_distribution_false_for_attached() {
    use squid_n_core::ids::{FloorRegionId, NodeId, SectionId, SlabId};
    use squid_n_core::model::{FloorRegion, JoistLine, LoadTransfer, RegionAnchor, SlabShape};

    let mut model = Model::default();
    for (i, c) in [
        [0.0, 0.0, 0.0],
        [4000.0, 0.0, 0.0],
        [4000.0, 4000.0, 0.0],
        [0.0, 4000.0, 0.0],
    ]
    .iter()
    .enumerate()
    {
        model.nodes.push(squid_n_core::model::Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    model.slabs.push(Slab {
        id: SlabId(0),
        shape: SlabShape::Attached {
            anchor: RegionAnchor::Line {
                nodes: [NodeId(0), NodeId(1)],
                span: [0.0, 1.0],
                transfer: LoadTransfer::Anchor,
            },
            extent: [1500.0, 1500.0],
        },
        plate: SlabPlate::default(),
    });
    let mut region = FloorRegion::new(
        FloorRegionId(0),
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
    );
    region.joists = vec![JoistLine {
        dir: [0.0, 1.0],
        spacing: 2000.0,
        support: [NodeId(0), NodeId(1)],
        section: Some(SectionId(0)),
        pinned_onto: None,
    }];
    region.slab_ids = vec![SlabId(0)];
    assert!(!super::uses_joist_distribution(&model, &region));
}

/// 床領域が床板を 2 枚以上持つ場合、手入力小梁ラインがあっても二段階分配は使わない
/// （代表床板〔先頭〕以外の面積・荷重を無視して総和保存が崩れるため）。
/// `distribute_region` は通常経路（各床板を独立に分配）へ落ち、両方の床板の荷重が
/// 出力に現れる。
#[test]
fn test_uses_joist_distribution_false_for_multiple_slabs() {
    use squid_n_core::ids::{FloorRegionId, NodeId, SectionId, SlabId};
    use squid_n_core::model::{DistributionMethod, FloorRegion, JoistLine, SlabShape};

    let mut model = Model::default();
    // 6000×5000 の床領域（節点 0-1-2-3）。中央の小梁（節点 4-5）が 2 枚の床板
    // （0: y=0..2500, 1: y=2500..5000）へ細分する。
    for (i, c) in [
        [0.0, 0.0, 0.0],
        [6000.0, 0.0, 0.0],
        [6000.0, 5000.0, 0.0],
        [0.0, 5000.0, 0.0],
        [6000.0, 2500.0, 0.0],
        [0.0, 2500.0, 0.0],
    ]
    .iter()
    .enumerate()
    {
        model.nodes.push(squid_n_core::model::Node {
            id: NodeId(i as u32),
            coord: *c,
            restraint: Default::default(),
            mass: None,
            story: None,
            support_spring: None,
        });
    }
    let mk_slab = |id: u32, boundary: Vec<NodeId>| Slab {
        id: SlabId(id),
        shape: SlabShape::Enclosed { boundary },
        plate: SlabPlate {
            method: DistributionMethod::TriTrapezoid,
            ..SlabPlate::default()
        },
    };
    model
        .slabs
        .push(mk_slab(0, vec![NodeId(0), NodeId(1), NodeId(4), NodeId(5)]));
    model
        .slabs
        .push(mk_slab(1, vec![NodeId(5), NodeId(4), NodeId(2), NodeId(3)]));
    let mut region = FloorRegion::new(
        FloorRegionId(0),
        vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
    );
    region.joists = vec![JoistLine {
        dir: [1.0, 0.0],
        spacing: 6000.0,
        support: [NodeId(5), NodeId(4)],
        section: Some(SectionId(0)),
        pinned_onto: None,
    }];
    region.slab_ids = vec![SlabId(0), SlabId(1)];

    assert!(
        !super::uses_joist_distribution(&model, &region),
        "床板が2枚以上ある床領域は二段階分配を使わない"
    );

    // 総和保存: 両方の床板の面荷重が失われず出力へ現れる。
    let loads = super::distribute_region(&model, &region, |_| 1.0e-3);
    let total: f64 = loads.iter().map(|bl| bl.cmq.q_i + bl.cmq.q_j).sum();
    let expected = 1.0e-3 * 6000.0 * 5000.0;
    assert!(
        (total - expected).abs() / expected < 1e-9,
        "total={total} expected={expected}"
    );
}
