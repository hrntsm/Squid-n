//! 断面幾何量のヘルパ関数。
//!
//! - [`rect_torsion_j`] — 矩形断面の St.Venant ねじり定数
//! - [`h_web_shear_area`] — H 形のウェブせん断断面積
//! - [`angle_centroid`] — 山形鋼の図心
//! - [`tee_centroid`] — T 形鋼の図心

/// 矩形断面の St.Venant ねじり定数（材料力学）。
///
/// J = (b³·h/16)·[16/3 − 3.36·(b/h)·(1 − (1/12)(b/h)⁴)]（b: 短辺, h: 長辺）
///
/// アスペクト比によらず同一式を適用する（b/h→0 で β→1/3 に漸近）。
pub(crate) fn rect_torsion_j(b: f64, d: f64) -> f64 {
    let bs = b.min(d);
    let h = b.max(d);
    if bs <= 0.0 || h <= 0.0 {
        return 0.0;
    }
    let c = bs / h;
    bs.powi(3) * h / 16.0 * (16.0 / 3.0 - 3.36 * c * (1.0 - c.powi(4) / 12.0))
}

/// H 形（内蔵鉄骨含む）のウェブせん断断面積（ウェブ全せい×ウェブ厚。
/// 設計検定側 `squid-n-design-jp::steel::shear_area` と同一規約）。
pub(crate) fn h_web_shear_area(height: f64, web_thick: f64) -> f64 {
    (height * web_thick).max(0.0)
}

pub(crate) fn angle_centroid(leg_a: f64, leg_b: f64, thick: f64) -> (f64, f64, f64) {
    let a1 = leg_a * thick;
    let a2 = (leg_b - thick) * thick;
    let a_total = a1 + a2;
    if a_total < 1e-30 {
        return (0.0, 0.0, 0.0);
    }
    let cy = (a1 * leg_a / 2.0 + a2 * thick / 2.0) / a_total;
    let cx = (a1 * thick / 2.0 + a2 * (thick + (leg_b - thick) / 2.0)) / a_total;
    (cx, cy, a_total)
}

/// リップ溝形鋼（cold-formed lipped channel）の非重複矩形分解と図心・断面積。
///
/// せい `height`（H, Y方向）・フランジ幅 `width`（B, Z方向。ウェブ外面〜フランジ先端）・
/// リップ長 `lip`（C, Y方向）・板厚 `thick`（t、全要素一様）の薄肉開断面を、
/// ウェブ／上下フランジ／上下リップの 5 枚の矩形へ重なりなく分解する。
/// 上下対称のため図心 y = H/2。戻り値は `(z_bar, area)`（z はウェブ外面 z=0 起点）。
///
/// 分解（各矩形の (z範囲, y範囲)）:
/// - ウェブ:   z∈[0,t],     y∈[0,H]
/// - フランジ: z∈[t,B],     y∈[0,t] と [H−t,H]
/// - リップ:   z∈[B−t,B],   y∈[t,C] と [H−C,H−t]
pub(crate) fn lip_channel_centroid_z(height: f64, width: f64, lip: f64, thick: f64) -> (f64, f64) {
    let a_web = thick * height;
    let a_flange = (width - thick) * thick;
    let a_lip = thick * (lip - thick);
    let a_total = a_web + 2.0 * a_flange + 2.0 * a_lip;
    if a_total < 1e-30 {
        return (0.0, 0.0);
    }
    let z_web = thick / 2.0;
    let z_flange = (thick + width) / 2.0;
    let z_lip = width - thick / 2.0;
    let z_bar = (a_web * z_web + 2.0 * a_flange * z_flange + 2.0 * a_lip * z_lip) / a_total;
    (z_bar, a_total)
}

/// 非対称組立 H 形鋼（上下フランジの幅・厚が異なる welded H）の図心 y と断面積。
///
/// せい `height`（H, 外〜外）・上フランジ `upper_width`×`upper_thick`・下フランジ
/// `lower_width`×`lower_thick`・ウェブ厚 `web_thick`。上下フランジ＋ウェブの 3 枚へ
/// 分解する。左右対称（フランジはウェブ中心）のため図心 z=0。戻り値 `(y_bar, area)`
/// （y は下フランジ下端 y=0 起点）。
#[allow(clippy::too_many_arguments)]
pub(crate) fn built_h_centroid_y(
    height: f64,
    upper_width: f64,
    upper_thick: f64,
    lower_width: f64,
    lower_thick: f64,
    web_thick: f64,
) -> (f64, f64) {
    let hw = (height - upper_thick - lower_thick).max(0.0);
    let a_uf = upper_width * upper_thick;
    let a_lf = lower_width * lower_thick;
    let a_w = web_thick * hw;
    let a_total = a_uf + a_lf + a_w;
    if a_total < 1e-30 {
        return (0.0, 0.0);
    }
    let y_uf = height - upper_thick / 2.0;
    let y_lf = lower_thick / 2.0;
    let y_w = lower_thick + hw / 2.0;
    let y_bar = (a_uf * y_uf + a_lf * y_lf + a_w * y_w) / a_total;
    (y_bar, a_total)
}

pub(crate) fn tee_centroid(height: f64, width: f64, web_thick: f64, flange_thick: f64) -> f64 {
    let a_f = width * flange_thick;
    let a_w = (height - flange_thick) * web_thick;
    let a_total = a_f + a_w;
    if a_total < 1e-30 {
        return 0.0;
    }
    (a_f * (height - flange_thick / 2.0) + a_w * (height - flange_thick) / 2.0) / a_total
}

/// 水平な帯（`(y下端, y上端, 幅)`）の集合から、強軸まわりの塑性断面係数 Zp [mm³]
/// を求める。
///
/// 帯は **z 方向（幅方向）に重ならない**分解であること（y 方向の重なりは許す。
/// 同じ y での幅は合算される）。塑性中立軸は**等面積軸**（断面積を上下 2 等分する
/// 水平軸）で、弾性図心とは一般に一致しない。
///
/// `Zp = ∫|y − y_p| dA`（y_p は等面積軸）。矩形 b×d で `b·d²/4`、H 形で
/// `B·tf·(H−tf) + tw·(H−2tf)²/4` に一致する。
///
/// 退化した入力（総面積 0）は 0.0 を返す。
pub(crate) fn plastic_modulus_strips(strips: &[(f64, f64, f64)]) -> f64 {
    // y 方向の重なりを解くため、全境界で区切って各区間の合計幅を求める。
    let mut ys: Vec<f64> = strips
        .iter()
        .flat_map(|&(lo, hi, _)| [lo, hi])
        .filter(|v| v.is_finite())
        .collect();
    ys.sort_by(f64::total_cmp);
    ys.dedup_by(|a, b| (*a - *b).abs() < 1e-12);

    let bands: Vec<(f64, f64, f64)> = ys
        .windows(2)
        .filter_map(|w| {
            let (lo, hi) = (w[0], w[1]);
            if hi <= lo {
                return None;
            }
            let mid = 0.5 * (lo + hi);
            let width: f64 = strips
                .iter()
                .filter(|&&(a, b, _)| a <= mid && mid <= b)
                .map(|&(_, _, wd)| wd)
                .sum();
            (width > 0.0).then_some((lo, hi, width))
        })
        .collect();

    let total: f64 = bands.iter().map(|&(lo, hi, w)| (hi - lo) * w).sum();
    if total <= 0.0 {
        return 0.0;
    }

    // 等面積軸 y_p を下から積み上げて求める（跨る帯の中で線形に解く）。
    let half = total / 2.0;
    let mut acc = 0.0;
    let mut y_p = bands[0].0;
    for &(lo, hi, w) in &bands {
        let a = (hi - lo) * w;
        if acc + a >= half {
            y_p = lo + (half - acc) / w;
            break;
        }
        acc += a;
        y_p = hi;
    }

    // Zp = Σ ∫|y − y_p| dA。帯が y_p を跨ぐ場合は上下へ分けて積分する。
    bands
        .iter()
        .map(|&(lo, hi, w)| {
            if hi <= y_p {
                w * ((y_p - lo).powi(2) - (y_p - hi).powi(2)) / 2.0
            } else if lo >= y_p {
                w * ((hi - y_p).powi(2) - (lo - y_p).powi(2)) / 2.0
            } else {
                w * ((y_p - lo).powi(2) + (hi - y_p).powi(2)) / 2.0
            }
        })
        .sum()
}
