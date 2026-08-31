use super::*;
use squid_n_core::model::{
    DistributionMethod, FloorRegion, LoadTransfer, MemberLoadKind, RegionAnchor, Slab, SlabPlate,
    SlabShape,
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

/// 床領域が複数の床板を持つとき、`distribute_region` は各床板を独立に分配し、
/// 総和（面荷重 × 全床板面積）を保存する。
#[test]
fn test_distribute_region_conserves_total_over_multiple_slabs() {
    use squid_n_core::ids::{FloorRegionId, NodeId, SlabId};

    let mut model = Model::default();
    // 6000×5000 の床領域（節点 0-1-2-3）。中央の小梁位置（節点 4-5）で 2 枚の床板
    // （0: y=0..2500, 1: y=2500..5000）へ細分されている。
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
    region.slab_ids = vec![SlabId(0), SlabId(1)];

    let loads = super::distribute_region(&model, &region, |_| 1.0e-3);
    let total: f64 = loads.iter().map(|bl| bl.cmq.q_i + bl.cmq.q_j).sum();
    let expected = 1.0e-3 * 6000.0 * 5000.0;
    assert!(
        (total - expected).abs() / expected < 1e-9,
        "total={total} expected={expected}"
    );
}
