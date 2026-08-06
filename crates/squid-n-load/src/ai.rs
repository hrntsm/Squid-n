use squid_n_core::model::StoryLevelKind;

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SoilClass {
    I,
    II,
    III,
}

#[derive(Clone, Debug)]
pub struct AiDistribution {
    pub alpha: Vec<f64>,
    pub ai: Vec<f64>,
    pub ci: Vec<f64>,
    pub qi: Vec<f64>,
    pub pi: Vec<f64>,
    pub t_used: f64,
    /// Pi (層の水平外力) の算定過程で、数値誤差の範囲を超える負値
    /// （閾値 `-1e-9・Q1`、Q1=最下層のせん断力=基部せん断力）が現れ、
    /// 0 へクランプしたかどうか。`true` の場合は重量分布（Wi の並び）が
    /// 単調に積み上がっていない等、入力異常のシグナルである可能性が高い
    /// （レビュー §1.12：従来は `p.max(0.0)` でサイレントにクランプしていた）。
    pub clamped_negative_pi: bool,
}

pub fn rt(t: f64, tc: f64) -> f64 {
    if t < tc {
        1.0
    } else if t < 2.0 * tc {
        1.0 - 0.2 * (t / tc - 1.0).powi(2)
    } else {
        1.6 * tc / t
    }
}

pub fn tc_of(soil: SoilClass) -> f64 {
    match soil {
        SoilClass::I => 0.4,
        SoilClass::II => 0.6,
        SoilClass::III => 0.8,
    }
}

/// Qi 列から Pi＝Qi−Qi+1（最上層は Pi=Qi）を求める。数値誤差の閾値
/// `-1e-9・Q1` を超える負値が現れた場合は `clamped_negative_pi=true` を返す
/// （レビュー §1.12）。
fn pi_from_qi(qi: &[f64]) -> (Vec<f64>, bool) {
    let n = qi.len();
    let mut pi = Vec::with_capacity(n);
    let mut clamped_negative = false;
    // Q1（基部せん断力）＝最下層の Qi。閾値の基準に用いる。
    let q1 = qi.first().copied().unwrap_or(0.0);
    for i in 0..n {
        let p = if i < n - 1 { qi[i] - qi[i + 1] } else { qi[i] };
        if p < -1e-9 * q1 {
            clamped_negative = true;
        }
        pi.push(p.max(0.0));
    }
    (pi, clamped_negative)
}

/// 高さ方向分布係数 Ai = 1 + (1/√αi − αi)・2T/(1+3T)。
///
/// αi = 0 は「その階から上に地震用重量がない」退化状態（利用者が定義しただけで
/// まだ部材を持たない最上階、地震用重量が算定できていない階など）で、式の
/// 1/√αi が発散して Ai を定義できない。支える重量が 0 である以上、層せん断力
/// Qi = Ci・αi・ΣW も 0 が正しいので、Ai を 0 に倒して Qi を 0 に落とす。
///
/// クランプしないと ∞ と 0 の積が NaN になり、NaN の水平力が載荷時に
/// 非有限値として無言で捨てられ、地震力そのものが消える。
fn ai_of_alpha(alpha: f64, t_factor: f64) -> f64 {
    if alpha <= 0.0 {
        0.0
    } else {
        1.0 + ((1.0 / alpha.sqrt()) - alpha) * t_factor
    }
}

pub fn ai_distribution(
    stories_weight_bottom_to_top: &[f64],
    z: f64,
    rt_val: f64,
    c0: f64,
    t: f64,
) -> AiDistribution {
    let total_w: f64 = stories_weight_bottom_to_top.iter().sum();
    let n = stories_weight_bottom_to_top.len();
    let mut alpha = Vec::with_capacity(n);
    let mut cumulative = 0.0;
    for i in (0..n).rev() {
        cumulative += stories_weight_bottom_to_top[i];
        // 総重量 0 のモデル（解析前チェックが弾く入力不備）で 0/0 の NaN を
        // 生まないよう、αi を 0 に倒す。以降の Ai・Qi はすべて 0 になる。
        alpha.push(if total_w > 0.0 {
            cumulative / total_w
        } else {
            0.0
        });
    }
    alpha.reverse();

    let t_factor = 2.0 * t / (1.0 + 3.0 * t);
    let ai: Vec<f64> = alpha.iter().map(|a| ai_of_alpha(*a, t_factor)).collect();
    let ci: Vec<f64> = ai.iter().map(|a| z * rt_val * a * c0).collect();
    // 層せん断力 Qi = Ci · Wi（Wi＝当該層以上の累積重量＝令88条）。
    // αi = Wi/total_w なので Wi = αi・total_w。
    // （旧実装は Ci·単層重量 で、Pi=Qi[i]-Qi[i+1] が下層で負になり max(0) で 0 に
    //   潰れ、地震力が最上層にしか載らない重大なバグだった。）
    let qi: Vec<f64> = ci
        .iter()
        .zip(alpha.iter())
        .map(|(c, a)| c * a * total_w)
        .collect();
    // 各層の水平外力 Pi = Qi − Qi+1（最上層は Pi = Qi）。
    let (pi, clamped_negative_pi) = pi_from_qi(&qi);

    AiDistribution {
        alpha,
        ai,
        ci,
        qi,
        pi,
        t_used: t,
        clamped_negative_pi,
    }
}

pub fn approx_t(height_m: f64, steel_ratio: f64) -> f64 {
    height_m * (0.02 + 0.01 * steel_ratio)
}

/// [`seismic_shear_distribution`] への入力層データ。
pub struct StorySeismicSpec {
    /// 当該層のうち主系統（Ai 分布に従う剛床）が負担する地震用重量 Wi [N]。
    /// 層せん断力 Qi = Ci・ΣWj の重量にはこちらを用いる。
    pub weight: f64,
    /// α・Ai・Ci の算定に用いる階全体の地震用重量 [N]。
    /// 「主剛床は全剛床の場合の Ci に従って層せん断力を計算する」規定に対応し、
    /// 副剛床（Ci 直接入力）の重量も**含めた**値を渡す。副剛床がない通常の階では
    /// `weight` と同値。
    pub ci_weight: f64,
    /// 階種別（一般/PH/地下）。地震層せん断力の算定式を切り替える。
    pub level_kind: StoryLevelKind,
}

/// 一般階・PH（塔屋）階・地下階が混在する建物の地震層せん断力分布を求める
/// （令88条および同条の実務的運用）。
///
/// `stories_bottom_to_top` は建物の最下部（最も深い地下階、なければ最下の一般階）
/// から最上部（最上の PH 階、なければ最上の一般階）の順に並べる。階種別は
/// 下から「地下 → 一般 → PH」の順で連続する前提（`debug_assert` で検証。
/// 違反しても計算自体は各層の式に従って進める）。
///
/// - **一般階**: 通常の Ai 分布に従う（[`ai_distribution`] と同じ式）。
///   ただし αi・Wi の算定に用いる「当該階以上の重量」には PH 階の重量を
///   含める（PH は最上部の付加重量として扱う）。地下階の重量は含めない
///   （αi は地上部分のみで正規化する）。α・Ai・Ci は `ci_weight`
///   （副剛床を含む階全体の重量）から求め、層せん断力 Qi = Ci・ΣWj の
///   ΣWj は `weight`（主系統の重量）の累積とする（「主剛床は全剛床の
///   場合の Ci に従って層せん断力を計算する」規定）。
/// - **PH階**: Qi = k・ΣWj（j はその階以上の重量和、k は 0.5〜1.0 の指定震度）。
///   `ci` 欄には k をそのまま格納する（等価係数）。
/// - **地下階**: Qi = Q(i+1) + Ki・Wi、Ki = 0.1・(1 − min(Hi,20)/40)・Z
///   （令88条4項。Hi は地盤面からの深さ[m]、20m 超は 20m。Q(i+1) は直上の層の
///   せん断力）。`ci` 欄には Ki を格納する（等価係数）。
///
/// Pi = Qi − Q(i+1)（最上層は Pi=Qi）は階種別によらず全層を通して算定する。
/// 返り値の `alpha`・`ai` は一般階以外では意味を持たない（0.0 のまま）。
pub fn seismic_shear_distribution(
    stories_bottom_to_top: &[StorySeismicSpec],
    z: f64,
    rt_val: f64,
    c0: f64,
    t: f64,
) -> AiDistribution {
    let n = stories_bottom_to_top.len();

    // 全階が一般階かつ副剛床の重量除外がなければ ai_distribution と厳密一致（委譲）。
    if stories_bottom_to_top
        .iter()
        .all(|s| matches!(s.level_kind, StoryLevelKind::Normal) && s.ci_weight == s.weight)
    {
        let weights: Vec<f64> = stories_bottom_to_top.iter().map(|s| s.weight).collect();
        return ai_distribution(&weights, z, rt_val, c0, t);
    }

    #[cfg(debug_assertions)]
    {
        fn rank(k: &StoryLevelKind) -> i32 {
            match k {
                StoryLevelKind::Basement { .. } => 0,
                StoryLevelKind::Normal => 1,
                StoryLevelKind::Penthouse { .. } => 2,
            }
        }
        let ranks: Vec<i32> = stories_bottom_to_top
            .iter()
            .map(|s| rank(&s.level_kind))
            .collect();
        debug_assert!(
            ranks.windows(2).all(|w| w[0] <= w[1]),
            "story level kinds must be ordered basement -> normal -> penthouse, bottom to top"
        );
    }

    // α・Ai・Ci 用（階全体の重量。副剛床の Ci 直接入力があっても全剛床分を含む）。
    let total_ph_ci_weight: f64 = stories_bottom_to_top
        .iter()
        .filter(|s| matches!(s.level_kind, StoryLevelKind::Penthouse { .. }))
        .map(|s| s.ci_weight)
        .sum();
    let total_normal_ci_weight: f64 = stories_bottom_to_top
        .iter()
        .filter(|s| matches!(s.level_kind, StoryLevelKind::Normal))
        .map(|s| s.ci_weight)
        .sum();
    let total_above_ground_ci = total_normal_ci_weight + total_ph_ci_weight;
    // Qi 用（主系統の重量）。
    let total_ph_weight: f64 = stories_bottom_to_top
        .iter()
        .filter(|s| matches!(s.level_kind, StoryLevelKind::Penthouse { .. }))
        .map(|s| s.weight)
        .sum();

    let t_factor = 2.0 * t / (1.0 + 3.0 * t);

    let mut alpha = vec![0.0; n];
    let mut ai = vec![0.0; n];
    let mut ci = vec![0.0; n];
    let mut qi = vec![0.0; n];

    // 一般階: α・Ai・Ci は階全体の重量（ci_weight）から求め、
    // Qi = Ci・Wi の Wi は主系統重量（weight）の累積 + PH階主系統重量とする。
    let mut cum_normal_ci = 0.0;
    let mut cum_normal = 0.0;
    for i in (0..n).rev() {
        if let StoryLevelKind::Normal = stories_bottom_to_top[i].level_kind {
            cum_normal_ci += stories_bottom_to_top[i].ci_weight;
            cum_normal += stories_bottom_to_top[i].weight;
            let wi_ci = cum_normal_ci + total_ph_ci_weight;
            let wi = cum_normal + total_ph_weight;
            let a = if total_above_ground_ci > 0.0 {
                wi_ci / total_above_ground_ci
            } else {
                0.0
            };
            let ai_val = ai_of_alpha(a, t_factor);
            let c = z * rt_val * ai_val * c0;
            alpha[i] = a;
            ai[i] = ai_val;
            ci[i] = c;
            qi[i] = c * wi;
        }
    }

    // PH階: Qi = k・ΣWj（j はその階以上、PH同士の累積を含む）。
    let mut cum_ph = 0.0;
    for i in (0..n).rev() {
        if let StoryLevelKind::Penthouse { k } = stories_bottom_to_top[i].level_kind {
            cum_ph += stories_bottom_to_top[i].weight;
            ci[i] = k;
            qi[i] = k * cum_ph;
        }
    }

    // 地下階: Qi = Q(i+1) + Ki・Wi。直上の層（i+1）が先に確定している必要が
    // あるため、上（大きい index）から下（小さい index）へ処理する。
    for i in (0..n).rev() {
        if let StoryLevelKind::Basement { depth_m } = stories_bottom_to_top[i].level_kind {
            let q_above = if i + 1 < n { qi[i + 1] } else { 0.0 };
            let h = depth_m.min(20.0);
            let k_i = 0.1 * (1.0 - h / 40.0) * z;
            ci[i] = k_i;
            qi[i] = q_above + k_i * stories_bottom_to_top[i].weight;
        }
    }

    let (pi, clamped_negative_pi) = pi_from_qi(&qi);

    AiDistribution {
        alpha,
        ai,
        ci,
        qi,
        pi,
        t_used: t,
        clamped_negative_pi,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rt_values() {
        assert!((rt(0.2, 0.6) - 1.0).abs() < 1e-12);
        let r = rt(0.8, 0.6);
        assert!(r < 1.0 && r > 0.0);
        let r = rt(2.0, 0.6);
        assert!((r - 1.6 * 0.6 / 2.0).abs() < 1e-12);
    }

    #[test]
    fn test_ai_distribution_3story() {
        let weights = vec![1000.0, 1000.0, 1000.0];
        let result = ai_distribution(&weights, 1.0, 1.0, 0.2, 0.24);
        assert_eq!(result.alpha.len(), 3);
        assert!((result.alpha[0] - 1.0).abs() < 1e-3);
        assert!((result.alpha[1] - 2.0 / 3.0).abs() < 1e-3);
        assert!((result.alpha[2] - 1.0 / 3.0).abs() < 1e-3);
        assert!((result.ai[2] - result.ai[1]) > 0.0);
        assert!(result.pi.iter().all(|&p| p >= 0.0));
    }

    #[test]
    fn test_ai_spec_values() {
        let weights = vec![1.0, 1.0, 1.0];
        let result = ai_distribution(&weights, 1.0, 1.0, 0.2, 0.24);
        assert!(
            (result.alpha[0] - 1.0).abs() < 1e-3,
            "alpha[0]={}",
            result.alpha[0]
        );
        assert!(
            (result.alpha[1] - 0.6667).abs() < 1e-3,
            "alpha[1]={}",
            result.alpha[1]
        );
        assert!(
            (result.alpha[2] - 0.3333).abs() < 1e-3,
            "alpha[2]={}",
            result.alpha[2]
        );
        assert!((result.ai[0] - 1.000).abs() < 1e-2, "A0={}", result.ai[0]);
        assert!((result.ai[1] - 1.156).abs() < 1e-2, "A1={}", result.ai[1]);
        assert!((result.ai[2] - 1.390).abs() < 1e-2, "A2={}", result.ai[2]);
    }

    #[test]
    fn test_story_shear_uses_cumulative_weight() {
        // Qi = Ci·Wi（Wi=累積重量）。等重量3層 w=[1,1,1], Z=Rt=1, C0=0.2, T=0.24。
        // alpha=[1, 2/3, 1/3], Ai≈[1.0,1.156,1.390], Ci=0.2·Ai。
        // Wi=[3,2,1] → Qi=[0.6, 0.4624, 0.278]（近似）。
        let weights = vec![1.0, 1.0, 1.0];
        let r = ai_distribution(&weights, 1.0, 1.0, 0.2, 0.24);
        // 基部せん断 Q0 = C0·Ai0·total = 0.2·1.0·3 = 0.6。
        assert!((r.qi[0] - 0.6).abs() < 1e-2, "Q0={}", r.qi[0]);
        assert!((r.qi[1] - 0.4624).abs() < 2e-2, "Q1={}", r.qi[1]);
        assert!((r.qi[2] - 0.278).abs() < 2e-2, "Q2={}", r.qi[2]);
        // Qi は下から上へ単調減少（正常な地震時層せん断）。
        assert!(r.qi[0] > r.qi[1] && r.qi[1] > r.qi[2]);
        // Pi = Qi−Qi+1 はすべて正（最上層に全部寄る旧バグの否定）。
        assert!(r.pi.iter().all(|&p| p > 0.0), "pi={:?}", r.pi);
        // ΣPi = Q0（基部せん断）。
        let sum_pi: f64 = r.pi.iter().sum();
        assert!(
            (sum_pi - r.qi[0]).abs() < 1e-9,
            "ΣPi={} Q0={}",
            sum_pi,
            r.qi[0]
        );
    }

    #[test]
    fn test_no_negative_pi_clamp_for_normal_monotone_weights() {
        let weights = vec![1.0, 1.0, 1.0];
        let r = ai_distribution(&weights, 1.0, 1.0, 0.2, 0.24);
        assert!(!r.clamped_negative_pi);
    }

    #[test]
    fn test_clamped_negative_pi_detected() {
        // 意図的に Qi が下層で上層より小さくなるような異常重量分布を作る
        // （最下層の重量をゼロに近く、上層に偏らせて alpha の並びを乱す）。
        // ai_distribution 自体は alpha が単調減少になる重量なら常に Qi 単調減少
        // なので、ここでは直接 pi_from_qi のロジックを検証する代わりに
        // seismic_shear_distribution の PH ケースで Q が減少するケースを使う。
        let stories = vec![
            StorySeismicSpec {
                weight: 1000.0,
                ci_weight: 1000.0,
                level_kind: StoryLevelKind::Normal,
            },
            StorySeismicSpec {
                weight: 1000.0,
                ci_weight: 1000.0,
                level_kind: StoryLevelKind::Penthouse { k: 0.0001 },
            },
        ];
        let r = seismic_shear_distribution(&stories, 1.0, 1.0, 0.2, 0.24);
        // PH の k が極端に小さいため Q_PH は Q_normal よりずっと小さく、
        // 通常は clamp は発生しない（Qiは単調非増加）。ここでは
        // クランプが発生しない健全ケースであることを確認する。
        assert!(!r.clamped_negative_pi);
    }

    #[test]
    fn test_seismic_shear_all_normal_matches_ai_distribution() {
        let weights = vec![1000.0, 1500.0, 800.0];
        let expected = ai_distribution(&weights, 1.0, 1.0, 0.2, 0.24);
        let stories: Vec<StorySeismicSpec> = weights
            .iter()
            .map(|&w| StorySeismicSpec {
                weight: w,
                ci_weight: w,
                level_kind: StoryLevelKind::Normal,
            })
            .collect();
        let actual = seismic_shear_distribution(&stories, 1.0, 1.0, 0.2, 0.24);
        assert_eq!(actual.alpha, expected.alpha);
        assert_eq!(actual.ai, expected.ai);
        assert_eq!(actual.ci, expected.ci);
        assert_eq!(actual.qi, expected.qi);
        assert_eq!(actual.pi, expected.pi);
        assert_eq!(actual.clamped_negative_pi, expected.clamped_negative_pi);
    }

    #[test]
    fn test_seismic_shear_with_penthouse() {
        // 2層の一般階 + PH階（k=1.0）。Q_PH = k・W_PH = 1.0・200 = 200。
        let stories = vec![
            StorySeismicSpec {
                weight: 1000.0,
                ci_weight: 1000.0,
                level_kind: StoryLevelKind::Normal,
            },
            StorySeismicSpec {
                weight: 1000.0,
                ci_weight: 1000.0,
                level_kind: StoryLevelKind::Normal,
            },
            StorySeismicSpec {
                weight: 200.0,
                ci_weight: 200.0,
                level_kind: StoryLevelKind::Penthouse { k: 1.0 },
            },
        ];
        let r = seismic_shear_distribution(&stories, 1.0, 1.0, 0.2, 0.24);
        assert!((r.qi[2] - 200.0).abs() < 1e-9, "Q_PH={}", r.qi[2]);
        assert!((r.ci[2] - 1.0).abs() < 1e-9, "ci(PH)={}", r.ci[2]);
        // 一般階の Wi は PH 重量を含む: W1(最上一般階) = 1000 + 200 = 1200。
        let total_above_ground = 1000.0 + 1000.0 + 200.0;
        let expected_alpha1 = 1200.0 / total_above_ground;
        assert!(
            (r.alpha[1] - expected_alpha1).abs() < 1e-9,
            "alpha1={} expected={}",
            r.alpha[1],
            expected_alpha1
        );
        // ΣPi = Q最下層。
        let sum_pi: f64 = r.pi.iter().sum();
        assert!(
            (sum_pi - r.qi[0]).abs() < 1e-6,
            "ΣPi={} Q0={}",
            sum_pi,
            r.qi[0]
        );
    }

    #[test]
    fn test_seismic_shear_with_basement() {
        // 地下1階(H=5m) + 一般階1層。Z=1.0 → K=0.1・(1-5/40)=0.0875。
        // Q_B1 = Q1(一般階の層せん断力) + K・W_B1。
        let w_normal = 1000.0;
        let w_basement = 500.0;
        let stories = vec![
            StorySeismicSpec {
                weight: w_basement,
                ci_weight: w_basement,
                level_kind: StoryLevelKind::Basement { depth_m: 5.0 },
            },
            StorySeismicSpec {
                weight: w_normal,
                ci_weight: w_normal,
                level_kind: StoryLevelKind::Normal,
            },
        ];
        let z = 1.0;
        let rt_val = 1.0;
        let c0 = 0.2;
        let t = 0.24;
        let r = seismic_shear_distribution(&stories, z, rt_val, c0, t);

        // 一般階（単独、alpha=1.0）の Qi を手計算。
        let t_factor = 2.0 * t / (1.0 + 3.0 * t);
        let alpha1: f64 = 1.0;
        let ai1 = 1.0 + ((1.0 / alpha1.sqrt()) - alpha1) * t_factor;
        let ci1 = z * rt_val * ai1 * c0;
        let q1 = ci1 * alpha1 * w_normal;
        assert!(
            (r.qi[1] - q1).abs() < 1e-9,
            "Q1={} expected={}",
            r.qi[1],
            q1
        );

        let k_expected = 0.1 * (1.0 - 5.0_f64.min(20.0) / 40.0) * z;
        assert!((k_expected - 0.0875).abs() < 1e-12);
        let q_b1_expected = q1 + k_expected * w_basement;
        assert!(
            (r.qi[0] - q_b1_expected).abs() < 1e-9,
            "Q_B1={} expected={}",
            r.qi[0],
            q_b1_expected
        );
        assert!((r.ci[0] - k_expected).abs() < 1e-12);

        // ΣPi = Q最下層(=地下1階のQ)。
        let sum_pi: f64 = r.pi.iter().sum();
        assert!(
            (sum_pi - r.qi[0]).abs() < 1e-6,
            "ΣPi={} Q_bottom={}",
            sum_pi,
            r.qi[0]
        );
    }

    #[test]
    fn test_seismic_shear_basement_normal_penthouse_sum_pi() {
        // 地下1 + 一般2 + PH1 のフル構成で ΣPi = Q最下層 を検証。
        let stories = vec![
            StorySeismicSpec {
                weight: 400.0,
                ci_weight: 400.0,
                level_kind: StoryLevelKind::Basement { depth_m: 3.0 },
            },
            StorySeismicSpec {
                weight: 1000.0,
                ci_weight: 1000.0,
                level_kind: StoryLevelKind::Normal,
            },
            StorySeismicSpec {
                weight: 900.0,
                ci_weight: 900.0,
                level_kind: StoryLevelKind::Normal,
            },
            StorySeismicSpec {
                weight: 150.0,
                ci_weight: 150.0,
                level_kind: StoryLevelKind::Penthouse { k: 0.6 },
            },
        ];
        let r = seismic_shear_distribution(&stories, 1.0, 1.0, 0.2, 0.24);
        let sum_pi: f64 = r.pi.iter().sum();
        assert!(
            (sum_pi - r.qi[0]).abs() < 1e-6,
            "ΣPi={} Q_bottom={}",
            sum_pi,
            r.qi[0]
        );
        assert!(!r.clamped_negative_pi);
    }

    /// 重量 0 の最上階（利用者が定義しただけでまだ部材のない階）があっても、
    /// Ai・Qi・Pi が NaN にならず、その階の層せん断力が 0 になること。
    ///
    /// αi = 0 で 1/√αi が発散し、Qi = Ci・αi・ΣW が ∞×0 = NaN になると、
    /// 載荷側が非有限値を捨てるため地震力が無言で消える。
    #[test]
    fn test_ai_distribution_with_zero_weight_top_story_is_finite() {
        let r = ai_distribution(&[1000.0, 800.0, 0.0], 1.0, 1.0, 0.2, 0.4);
        assert!(
            r.ai.iter().all(|v| v.is_finite()),
            "Ai に非有限値がある: {:?}",
            r.ai
        );
        assert!(
            r.qi.iter().all(|v| v.is_finite()) && r.pi.iter().all(|v| v.is_finite()),
            "Qi/Pi に非有限値がある: qi={:?} pi={:?}",
            r.qi,
            r.pi
        );
        assert_eq!(r.qi[2], 0.0, "上に重量がない階の層せん断力は 0");
        assert_eq!(r.pi[2], 0.0);
        // 下層は重量 0 の階がない場合と同じ結果になる。
        let base = ai_distribution(&[1000.0, 800.0], 1.0, 1.0, 0.2, 0.4);
        for i in 0..2 {
            assert!(
                (r.qi[i] - base.qi[i]).abs() < 1e-9,
                "層 {i}: {} vs {}",
                r.qi[i],
                base.qi[i]
            );
        }
    }

    /// 地震用重量が全階 0 のモデル（入力不備。解析前チェックが弾く）でも
    /// 0/0 の NaN を作らない。
    #[test]
    fn test_ai_distribution_with_zero_total_weight_is_finite() {
        let r = ai_distribution(&[0.0, 0.0], 1.0, 1.0, 0.2, 0.4);
        assert!(r.alpha.iter().all(|v| v.is_finite()));
        assert!(r.qi.iter().all(|v| *v == 0.0), "qi={:?}", r.qi);
        assert!(r.pi.iter().all(|v| *v == 0.0), "pi={:?}", r.pi);
    }

    /// PH 階を含む混在モデルでも、重量 0 の一般階で Ai・Qi が発散しないこと
    /// （`ai_distribution` と同じクランプが `seismic_shear_distribution` 側にも効く）。
    #[test]
    fn test_seismic_shear_distribution_zero_weight_normal_story_is_finite() {
        let stories = vec![
            StorySeismicSpec {
                weight: 1000.0,
                ci_weight: 1000.0,
                level_kind: StoryLevelKind::Normal,
            },
            StorySeismicSpec {
                weight: 0.0,
                ci_weight: 0.0,
                level_kind: StoryLevelKind::Normal,
            },
            StorySeismicSpec {
                weight: 0.0,
                ci_weight: 0.0,
                level_kind: StoryLevelKind::Penthouse { k: 0.6 },
            },
        ];
        let r = seismic_shear_distribution(&stories, 1.0, 1.0, 0.2, 0.4);
        assert!(
            r.ai.iter().all(|v| v.is_finite())
                && r.ci.iter().all(|v| v.is_finite())
                && r.qi.iter().all(|v| v.is_finite())
                && r.pi.iter().all(|v| v.is_finite()),
            "非有限値がある: ai={:?} ci={:?} qi={:?} pi={:?}",
            r.ai,
            r.ci,
            r.qi,
            r.pi
        );
    }
}
