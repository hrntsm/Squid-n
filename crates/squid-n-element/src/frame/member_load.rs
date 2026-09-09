//! 部材（梁）スパン荷重の等価節点力と固定端内力を計算する。
//!
//! ローカル 12 自由度の並びは i 端 [N, Vy, Vz, Mx, My, Mz] = index 0..6、j 端 = index 6..12。

use crate::transform::LocalFrame;
use squid_n_core::geom::vec3;
use squid_n_core::model::{MemberLoad, MemberLoadKind};

#[derive(Clone, Copy, Debug)]
enum Comp {
    Point { a: f64, p: f64 },
    Dist { a: f64, b: f64, w1: f64, w2: f64 },
}

/// 返り値 [Option<Comp>;3]: index 0=軸(x), 1=曲げ面y, 2=曲げ面z。
fn resolve(load: &MemberLoad, frame: &LocalFrame) -> [Option<Comp>; 3] {
    let d = load.dir;
    let dl = vec3::norm(d);
    if dl < 1e-12 {
        return [None, None, None];
    }
    let d = [d[0] / dl, d[1] / dl, d[2] / dl];
    let c = [
        vec3::dot(d, frame.rot[0]),
        vec3::dot(d, frame.rot[1]),
        vec3::dot(d, frame.rot[2]),
    ];
    let mut out = [None, None, None];
    for (axis, out_slot) in out.iter_mut().enumerate() {
        let ck = c[axis];
        if ck.abs() < 1e-15 {
            continue;
        }
        *out_slot = Some(match load.kind {
            MemberLoadKind::Point { a, p } => Comp::Point { a, p: p * ck },
            MemberLoadKind::Distributed { a, b, w1, w2 } => Comp::Dist {
                a,
                b,
                w1: w1 * ck,
                w2: w2 * ck,
            },
        });
    }
    out
}

fn n_vi(xi: f64) -> f64 {
    1.0 - 3.0 * xi * xi + 2.0 * xi * xi * xi
}
fn n_ti(xi: f64, l: f64) -> f64 {
    l * (xi - 2.0 * xi * xi + xi * xi * xi)
}
fn n_vj(xi: f64) -> f64 {
    3.0 * xi * xi - 2.0 * xi * xi * xi
}
fn n_tj(xi: f64, l: f64) -> f64 {
    l * (-xi * xi + xi * xi * xi)
}

fn gauss_dist<F: Fn(f64) -> f64>(a: f64, b: f64, w1: f64, w2: f64, f: F) -> f64 {
    if (b - a).abs() < 1e-12 {
        return 0.0;
    }
    const G: [f64; 3] = [-0.7745966692414834, 0.0, 0.7745966692414834];
    const W: [f64; 3] = [0.5555555555555556, 0.8888888888888888, 0.5555555555555556];
    let mid = 0.5 * (a + b);
    let half = 0.5 * (b - a);
    let mut s = 0.0;
    for k in 0..3 {
        let x = mid + half * G[k];
        let t = (x - a) / (b - a);
        let w = w1 + (w2 - w1) * t;
        s += W[k] * w * f(x);
    }
    s * half
}

/// スパン荷重を材端へ配る方式。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SpanLoadTransfer {
    #[default]
    Consistent,
    StaticallyEquivalent,
}

/// 部材の全スパン荷重に対する等価節点力ベクトル（local 12）。
pub fn consistent_load_local(
    loads: &[MemberLoad],
    frame: &LocalFrame,
    length: f64,
    transfer: SpanLoadTransfer,
) -> [f64; 12] {
    let l = length.max(1e-9);
    let mut q = [0.0; 12];
    for load in loads {
        let comps = resolve(load, frame);
        for (axis, comp) in comps.iter().enumerate() {
            let Some(comp) = comp else { continue };
            match (axis, transfer) {
                (0, _) => add_axial_consistent(&mut q, comp, l),
                (1, SpanLoadTransfer::Consistent) => add_bending_consistent(&mut q, comp, l, 1),
                (1, SpanLoadTransfer::StaticallyEquivalent) => {
                    add_bending_static(&mut q, comp, l, 1)
                }
                (_, SpanLoadTransfer::Consistent) => add_bending_consistent(&mut q, comp, l, 2),
                (_, SpanLoadTransfer::StaticallyEquivalent) => {
                    add_bending_static(&mut q, comp, l, 2)
                }
            }
        }
    }
    q
}

/// q[0]=N_i, q[6]=N_j。
fn add_axial_consistent(q: &mut [f64; 12], comp: &Comp, l: f64) {
    match *comp {
        Comp::Point { a, p } => {
            let xi = (a / l).clamp(0.0, 1.0);
            q[0] += p * (1.0 - xi);
            q[6] += p * xi;
        }
        Comp::Dist { a, b, w1, w2 } => {
            q[0] += gauss_dist(a, b, w1, w2, |s| 1.0 - s / l);
            q[6] += gauss_dist(a, b, w1, w2, |s| s / l);
        }
    }
}

/// 曲げ面（plane=1: y面 → Vy,Mz / plane=2: z面 → Vz,My）。
fn add_bending_consistent(q: &mut [f64; 12], comp: &Comp, l: f64, plane: usize) {
    let (iv, im, jv, jm, msign) = if plane == 1 {
        (1usize, 5usize, 7usize, 11usize, 1.0)
    } else {
        (2usize, 4usize, 8usize, 10usize, -1.0)
    };
    match *comp {
        Comp::Point { a, p } => {
            let xi = (a / l).clamp(0.0, 1.0);
            q[iv] += p * n_vi(xi);
            q[im] += msign * p * n_ti(xi, l);
            q[jv] += p * n_vj(xi);
            q[jm] += msign * p * n_tj(xi, l);
        }
        Comp::Dist { a, b, w1, w2 } => {
            q[iv] += gauss_dist(a, b, w1, w2, |s| n_vi(s / l));
            q[im] += msign * gauss_dist(a, b, w1, w2, |s| n_ti(s / l, l));
            q[jv] += gauss_dist(a, b, w1, w2, |s| n_vj(s / l));
            q[jm] += msign * gauss_dist(a, b, w1, w2, |s| n_tj(s / l, l));
        }
    }
}

/// 曲げ面成分を材端モーメント無しで両端へ配る。
fn add_bending_static(q: &mut [f64; 12], comp: &Comp, l: f64, plane: usize) {
    let (iv, jv) = if plane == 1 { (1usize, 7usize) } else { (2, 8) };
    let (r, r_j) = match *comp {
        Comp::Point { a, p } => {
            let xi = (a / l).clamp(0.0, 1.0);
            (p, p * xi)
        }
        Comp::Dist { a, b, w1, w2 } => {
            if b <= a {
                return;
            }
            let r = integ2(a, b, w1, w2, a, b, |_s| 1.0);
            let first_moment = integ2(a, b, w1, w2, a, b, |s| s);
            (r, first_moment / l)
        }
    };
    q[iv] += r - r_j;
    q[jv] += r_j;
}

fn comp_resultant(comp: &Comp, lo: f64, hi: f64) -> f64 {
    if hi <= lo {
        return 0.0;
    }
    match *comp {
        Comp::Point { a, p } => {
            if a >= lo && a < hi {
                p
            } else {
                0.0
            }
        }
        Comp::Dist { a, b, w1, w2 } => {
            let l = lo.max(a);
            let h = hi.min(b);
            if h <= l {
                return 0.0;
            }
            integ2(a, b, w1, w2, l, h, |_s| 1.0)
        }
    }
}

fn comp_moment(comp: &Comp, lo: f64, hi: f64, xref: f64) -> f64 {
    if hi <= lo {
        return 0.0;
    }
    match *comp {
        Comp::Point { a, p } => {
            if a >= lo && a < hi {
                p * (xref - a)
            } else {
                0.0
            }
        }
        Comp::Dist { a, b, w1, w2 } => {
            let l = lo.max(a);
            let h = hi.min(b);
            if h <= l {
                return 0.0;
            }
            integ2(a, b, w1, w2, l, h, |s| xref - s)
        }
    }
}

fn integ2<F: Fn(f64) -> f64>(a: f64, b: f64, w1: f64, w2: f64, l: f64, h: f64, f: F) -> f64 {
    const G: [f64; 2] = [-0.5773502691896257, 0.5773502691896257];
    let mid = 0.5 * (l + h);
    let half = 0.5 * (h - l);
    let denom = b - a;
    let mut s_sum = 0.0;
    for &g in &G {
        let s = mid + half * g;
        let t = if denom.abs() < 1e-12 {
            0.0
        } else {
            (s - a) / denom
        };
        let w = w1 + (w2 - w1) * t;
        s_sum += w * f(s);
    }
    s_sum * half
}

/// 断面 xi（i 端からの正規化位置 0..1）における固定端内力を
/// local 内力 [N, Qy, Qz, Mx, My, Mz] で返す。
///
/// `transfer` が [`SpanLoadTransfer::StaticallyEquivalent`] の場合、
/// 曲げ・せん断・ねじりの固定端内力は零を返し、軸力成分のみを返す。
pub fn fixed_internal_local(
    loads: &[MemberLoad],
    frame: &LocalFrame,
    length: f64,
    xi: f64,
    transfer: SpanLoadTransfer,
) -> [f64; 6] {
    let l = length.max(1e-9);
    let x = xi * l;
    let xr = (1.0 - xi) * l;
    let q = consistent_load_local(loads, frame, l, transfer);
    let ff = q.map(|v| -v);

    let mut comps_x: Vec<Comp> = Vec::new();
    let mut comps_y: Vec<Comp> = Vec::new();
    let mut comps_z: Vec<Comp> = Vec::new();
    for load in loads {
        let r = resolve(load, frame);
        if let Some(c) = r[0] {
            comps_x.push(c);
        }
        if let Some(c) = r[1] {
            comps_y.push(c);
        }
        if let Some(c) = r[2] {
            comps_z.push(c);
        }
    }
    let res_i = |comps: &[Comp]| comps.iter().map(|c| comp_resultant(c, 0.0, x)).sum::<f64>();
    let mom_i = |comps: &[Comp]| comps.iter().map(|c| comp_moment(c, 0.0, x, x)).sum::<f64>();
    let mom_jx = |comps: &[Comp]| comps.iter().map(|c| comp_moment(c, x, l, x)).sum::<f64>();

    let sy_i = res_i(&comps_y);
    let sz_i = res_i(&comps_z);

    let mut f = [0.0; 6];
    f[0] = ff[0] * (1.0 - xi) + ff[6] * xi;
    if transfer == SpanLoadTransfer::StaticallyEquivalent {
        return f;
    }
    f[1] = ff[1] + sy_i;
    f[2] = ff[2] + sz_i;

    if xi < 0.5 {
        f[3] = -ff[3];
        f[5] = -ff[5] + ff[1] * x + mom_i(&comps_y);
        f[4] = -ff[4] - ff[2] * x - mom_i(&comps_z);
    } else {
        f[3] = ff[9];
        f[5] = ff[11] + ff[7] * xr - mom_jx(&comps_y);
        f[4] = ff[10] - ff[8] * xr + mom_jx(&comps_z);
    }
    f
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::ElemId;
    use squid_n_core::model::{MemberLoad, MemberLoadKind};

    fn horiz_frame() -> LocalFrame {
        LocalFrame::from_nodes([0.0, 0.0, 0.0], [1000.0, 0.0, 0.0], [0.0, 0.0, 1.0])
    }

    fn udl(w: f64, l: f64) -> MemberLoad {
        MemberLoad::manual(
            ElemId(0),
            [0.0, 0.0, -1.0],
            MemberLoadKind::Distributed {
                a: 0.0,
                b: l,
                w1: w,
                w2: w,
            },
        )
    }

    #[test]
    fn udl_fixed_end_moment_is_wl2_over_12() {
        let l = 1000.0;
        let w = 2.0;
        let frame = horiz_frame();
        let loads = vec![udl(w, l)];
        let q = consistent_load_local(&loads, &frame, l, SpanLoadTransfer::Consistent);
        let expected_shear = w * l / 2.0;
        assert!((q[1].abs() - expected_shear).abs() < 1e-6, "q1={}", q[1]);
        assert!((q[7].abs() - expected_shear).abs() < 1e-6, "q7={}", q[7]);
        let fem = w * l * l / 12.0;
        assert!((q[5].abs() - fem).abs() < 1e-3, "q5={} fem={}", q[5], fem);
        assert!((q[11].abs() - fem).abs() < 1e-3, "q11={}", q[11]);
    }

    #[test]
    fn udl_clamped_midspan_moment_is_wl2_over_24() {
        let l = 1000.0;
        let w = 2.0;
        let frame = horiz_frame();
        let loads = vec![udl(w, l)];
        let mid = fixed_internal_local(&loads, &frame, l, 0.5, SpanLoadTransfer::Consistent);
        let expected = w * l * l / 24.0;
        assert!(
            (mid[5].abs() - expected).abs() < 1e-2,
            "mid Mz={} expected={}",
            mid[5],
            expected
        );
    }

    #[test]
    fn point_mid_fixed_end_moment_is_pl_over_8() {
        let l = 1000.0;
        let p = 100.0;
        let frame = horiz_frame();
        let loads = vec![MemberLoad::manual(
            ElemId(0),
            [0.0, 0.0, -1.0],
            MemberLoadKind::Point { a: l / 2.0, p },
        )];
        let q = consistent_load_local(&loads, &frame, l, SpanLoadTransfer::Consistent);
        let fem = p * l / 8.0;
        assert!((q[5].abs() - fem).abs() < 1e-6, "q5={} fem={}", q[5], fem);
        assert!((q[1].abs() - p / 2.0).abs() < 1e-6, "q1={}", q[1]);
    }

    /// 両端固定梁の UDL に対する固定端内力場が、i/j 分岐（ξ=0.5）をまたいで
    /// 連続かつ符号付き理論解と一致すること。下向き荷重（下端引張正の規約）で
    /// M(ξ) = wL²(6ξ−6ξ²−1)/12（端部 −wL²/12・中央 +wL²/24）、
    /// Q(ξ) = wL(1−2ξ)/2。
    #[test]
    fn udl_clamped_internal_field_continuous_and_signed() {
        let l = 1000.0;
        let w = 2.0;
        let frame = horiz_frame();
        let loads = vec![udl(w, l)];
        for &xi in &[0.0, 0.25, 0.45, 0.5, 0.55, 0.75, 1.0] {
            let f = fixed_internal_local(&loads, &frame, l, xi, SpanLoadTransfer::Consistent);
            let m_expected = w * l * l * (6.0 * xi - 6.0 * xi * xi - 1.0) / 12.0;
            let q_expected = w * l * (1.0 - 2.0 * xi) / 2.0;
            assert!(
                (f[5] - m_expected).abs() < 1e-2,
                "xi={xi} Mz={} expected={m_expected}",
                f[5]
            );
            assert!(
                (f[1] - q_expected).abs() < 1e-6,
                "xi={xi} Qy={} expected={q_expected}",
                f[1]
            );
        }
    }

    /// ブレース（トラス要素）の等分布荷重は、材端モーメントを生じずに
    /// 両端へ半分ずつ配られる。合力は等価節点力方式と一致する
    /// （建物総重量・支点反力が方式によって変わらないことの確認）。
    #[test]
    fn brace_udl_has_no_end_moment_and_preserves_resultant() {
        let l = 1000.0;
        let w = 2.0;
        let frame = horiz_frame();
        let loads = vec![udl(w, l)];
        let q = consistent_load_local(&loads, &frame, l, SpanLoadTransfer::StaticallyEquivalent);
        for &k in &[3usize, 4, 5, 9, 10, 11] {
            assert!(q[k].abs() < 1e-9, "q[{k}]={} は 0 のはず", q[k]);
        }
        let half = w * l / 2.0;
        assert!((q[1].abs() - half).abs() < 1e-6, "q1={}", q[1]);
        assert!((q[7].abs() - half).abs() < 1e-6, "q7={}", q[7]);
        let qc = consistent_load_local(&loads, &frame, l, SpanLoadTransfer::Consistent);
        assert!(
            ((q[1] + q[7]) - (qc[1] + qc[7])).abs() < 1e-6,
            "合力が方式で変わってはいけない"
        );
    }

    /// 材軸から外れた位置の集中荷重は、単純梁反力と同じ配分（てこの原理）で
    /// 両端へ分かれる。作用位置の 1 次モーメントが保存される。
    #[test]
    fn brace_point_load_splits_by_lever_rule() {
        let l = 1000.0;
        let p = 100.0;
        let a = 250.0;
        let frame = horiz_frame();
        let loads = vec![MemberLoad::manual(
            ElemId(0),
            [0.0, 0.0, -1.0],
            MemberLoadKind::Point { a, p },
        )];
        let q = consistent_load_local(&loads, &frame, l, SpanLoadTransfer::StaticallyEquivalent);
        assert!((q[1].abs() - p * (1.0 - a / l)).abs() < 1e-6, "q1={}", q[1]);
        assert!((q[7].abs() - p * a / l).abs() < 1e-6, "q7={}", q[7]);
        assert!(
            q[5].abs() < 1e-9 && q[11].abs() < 1e-9,
            "材端モーメントは 0"
        );
    }

    /// 材端ちょうどに載る集中荷重も落とさずに配る。
    #[test]
    fn brace_point_load_at_far_end_goes_entirely_to_that_end() {
        let l = 1000.0;
        let p = 100.0;
        let frame = horiz_frame();
        let loads = vec![MemberLoad::manual(
            ElemId(0),
            [0.0, 0.0, -1.0],
            MemberLoadKind::Point { a: l, p },
        )];
        let q = consistent_load_local(&loads, &frame, l, SpanLoadTransfer::StaticallyEquivalent);
        assert!(q[1].abs() < 1e-9, "i 端の分担は 0 のはず q1={}", q[1]);
        assert!(
            (q[7].abs() - p).abs() < 1e-6,
            "j 端が全量を持つ q7={}",
            q[7]
        );
    }

    /// 分布荷重が材長の一部に載る場合も、合力は等価節点力方式と一致する
    /// （静定分配が積分区間を材長で切り取っていないことの確認）。
    #[test]
    fn brace_partial_span_load_preserves_resultant() {
        let l = 1000.0;
        let frame = horiz_frame();
        let loads = vec![MemberLoad::manual(
            ElemId(0),
            [0.0, 0.0, -1.0],
            MemberLoadKind::Distributed {
                a: 200.0,
                b: 700.0,
                w1: 1.0,
                w2: 4.0,
            },
        )];
        let qs = consistent_load_local(&loads, &frame, l, SpanLoadTransfer::StaticallyEquivalent);
        let qc = consistent_load_local(&loads, &frame, l, SpanLoadTransfer::Consistent);
        assert!(
            ((qs[1] + qs[7]) - (qc[1] + qc[7])).abs() < 1e-6,
            "合力が方式で変わってはいけない: static={} consistent={}",
            qs[1] + qs[7],
            qc[1] + qc[7]
        );
        assert!(((qs[1] + qs[7]).abs() - 1250.0).abs() < 1e-6);
    }

    /// ブレースは材軸直交成分を負担しないため、固定端内力はどの断面でも
    /// 曲げ・せん断が 0 になる（荷重ベクトル側で両端へ流し切っている）。
    #[test]
    fn brace_fixed_internal_has_no_bending() {
        let l = 1000.0;
        let w = 2.0;
        let frame = horiz_frame();
        let loads = vec![udl(w, l)];
        for &xi in &[0.0, 0.25, 0.5, 0.75, 1.0] {
            let f = fixed_internal_local(
                &loads,
                &frame,
                l,
                xi,
                SpanLoadTransfer::StaticallyEquivalent,
            );
            for &k in &[1usize, 2, 3, 4, 5] {
                assert!(f[k].abs() < 1e-9, "xi={xi} f[{k}]={} は 0 のはず", f[k]);
            }
        }
    }

    #[test]
    fn triangle_via_trapezoid_matches_known_fem() {
        let l = 1000.0;
        let w = 3.0;
        let frame = horiz_frame();
        let loads = vec![MemberLoad::manual(
            ElemId(0),
            [0.0, 0.0, -1.0],
            MemberLoadKind::Distributed {
                a: 0.0,
                b: l,
                w1: 0.0,
                w2: w,
            },
        )];
        let q = consistent_load_local(&loads, &frame, l, SpanLoadTransfer::Consistent);
        let fem_i = w * l * l / 30.0;
        let fem_j = w * l * l / 20.0;
        assert!(
            (q[5].abs() - fem_i).abs() < 1e-2,
            "q5={} fem_i={}",
            q[5],
            fem_i
        );
        assert!(
            (q[11].abs() - fem_j).abs() < 1e-2,
            "q11={} fem_j={}",
            q[11],
            fem_j
        );
    }
}
