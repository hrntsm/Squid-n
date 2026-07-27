use squid_n_core::ids::LoadCaseId;
use squid_n_core::model::LoadCombination;

/// 多雪区域の積雪荷重低減係数（令86条 多雪区域の荷重組合せ）。
///
/// - `delta1`: 長期積雪 `DL+LL+δ1・SL` の低減係数（既定 0.7）
/// - `delta2`: 暴風時 `DL+LL+δ2・SL±WX/WY` の低減係数（既定 0.35）
/// - `delta3`: 地震時 `DL+LL+δ3・SL±EX/EY` の低減係数（既定 0.35）
///
/// 本実装では直接入力が可能（デフォルト δ1=0.7、δ2=δ3=0.35）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SnowFactors {
    pub delta1: f64,
    pub delta2: f64,
    pub delta3: f64,
}

impl Default for SnowFactors {
    fn default() -> Self {
        SnowFactors {
            delta1: 0.7,
            delta2: 0.35,
            delta3: 0.35,
        }
    }
}

/// [`standard_combinations`] への入力ケース指定。
pub struct ComboInput {
    pub dl: LoadCaseId,
    pub ll: LoadCaseId,
    pub seismic_x: Option<LoadCaseId>,
    pub seismic_y: Option<LoadCaseId>,
    pub wind_x: Option<LoadCaseId>,
    pub wind_y: Option<LoadCaseId>,
    pub snow: Option<LoadCaseId>,
    /// 多雪区域か否か（建築基準法施行令86条・同82条）。
    /// `true` の場合、長期に `δ1・S` を加算し、短期暴風・短期地震に
    /// `δ2・S`／`δ3・S` を加算した組合せも追加で生成する。
    pub heavy_snow_zone: bool,
    /// 多雪区域の積雪荷重低減係数。`None` は既定値（δ1=0.7、δ2=δ3=0.35）。
    pub snow_factors: Option<SnowFactors>,
}

fn push_gp(combos: &mut Vec<LoadCombination>, dl: LoadCaseId, ll: LoadCaseId) {
    combos.push(LoadCombination {
        name: "DL + LL".into(),
        terms: vec![(dl, 1.0), (ll, 1.0)],
    });
}

/// 建築基準法施行令82条の標準荷重組合せを生成する。
///
/// 組合せ名は荷重ケースの直接的な名前（DL：固定・LL：積載・EX/EY：地震 X/Y・
/// WX/WY：風 X/Y・SL：積雪）で表す。算定式との対応は
/// G→DL・P→LL・K→EX/EY・W→WX/WY・S→SL（積雪は単一ケースのため方向なし）。
///
/// - 長期: `DL+LL`。多雪区域はさらに `DL+LL+0.7SL`。
/// - 短期積雪: `DL+LL+SL`（`snow` が指定されている場合）。
/// - 短期地震: `DL+LL±EX`・`DL+LL±EY`（±両方向）。多雪区域はさらに
///   `DL+LL+0.35SL±EX`（X・Y 各方向）。
/// - 短期暴風: `DL+LL±WX`・`DL+LL±WY`（±両方向）。多雪区域はさらに
///   `DL+LL+0.35SL±WX`（X・Y 各方向）。
///
/// 各ケースは `seismic_x`/`seismic_y`/`wind_x`/`wind_y`/`snow` が
/// `Some` の場合のみ生成される（レビュー §1.10）。
pub fn standard_combinations(input: &ComboInput) -> Vec<LoadCombination> {
    let mut combos = Vec::new();
    let dl = input.dl;
    let ll = input.ll;
    let sf = input.snow_factors.unwrap_or_default();

    // 長期: DL+LL
    push_gp(&mut combos, dl, ll);

    // 多雪区域の長期: DL+LL+δ1・SL
    if input.heavy_snow_zone {
        if let Some(snow) = input.snow {
            combos.push(LoadCombination {
                name: format!("DL + LL + {}SL", trim_f64(sf.delta1)),
                terms: vec![(dl, 1.0), (ll, 1.0), (snow, sf.delta1)],
            });
        }
    }

    // 短期積雪: DL+LL+SL
    if let Some(snow) = input.snow {
        combos.push(LoadCombination {
            name: "DL + LL + SL".into(),
            terms: vec![(dl, 1.0), (ll, 1.0), (snow, 1.0)],
        });
    }

    // 短期地震（±両方向、多雪区域は δ3・SL 付きも追加）。
    push_directional(
        &mut combos,
        dl,
        ll,
        input.seismic_x,
        "EX",
        input.snow,
        input.heavy_snow_zone,
        sf.delta3,
    );
    push_directional(
        &mut combos,
        dl,
        ll,
        input.seismic_y,
        "EY",
        input.snow,
        input.heavy_snow_zone,
        sf.delta3,
    );

    // 短期暴風（±両方向、多雪区域は δ2・SL 付きも追加）。
    push_directional(
        &mut combos,
        dl,
        ll,
        input.wind_x,
        "WX",
        input.snow,
        input.heavy_snow_zone,
        sf.delta2,
    );
    push_directional(
        &mut combos,
        dl,
        ll,
        input.wind_y,
        "WY",
        input.snow,
        input.heavy_snow_zone,
        sf.delta2,
    );

    combos
}

/// 係数を組合せ名向けに整形する（末尾の 0 を落とす。例: 0.70 → "0.7"）。
fn trim_f64(v: f64) -> String {
    let s = format!("{v:.3}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

/// 地震・暴風いずれかの片方向（EX/EY/WX/WY）について、`DL+LL±X` と
/// 多雪区域なら `DL+LL+δ・SL±X`（δ: 暴風時 δ2／地震時 δ3）を追加する共通ヘルパー。
#[allow(clippy::too_many_arguments)]
fn push_directional(
    combos: &mut Vec<LoadCombination>,
    dl: LoadCaseId,
    ll: LoadCaseId,
    case: Option<LoadCaseId>,
    label: &str,
    snow: Option<LoadCaseId>,
    heavy_snow_zone: bool,
    delta: f64,
) {
    let Some(case) = case else {
        return;
    };
    combos.push(LoadCombination {
        name: format!("DL + LL + {label}"),
        terms: vec![(dl, 1.0), (ll, 1.0), (case, 1.0)],
    });
    combos.push(LoadCombination {
        name: format!("DL + LL - {label}"),
        terms: vec![(dl, 1.0), (ll, 1.0), (case, -1.0)],
    });
    if heavy_snow_zone {
        if let Some(snow) = snow {
            let d = trim_f64(delta);
            combos.push(LoadCombination {
                name: format!("DL + LL + {d}SL + {label}"),
                terms: vec![(dl, 1.0), (ll, 1.0), (snow, delta), (case, 1.0)],
            });
            combos.push(LoadCombination {
                name: format!("DL + LL + {d}SL - {label}"),
                terms: vec![(dl, 1.0), (ll, 1.0), (snow, delta), (case, -1.0)],
            });
        }
    }
}

/// 旧API（後方互換）。断面検定などから使う単純版
/// （長期 DL+LL / 短期積雪 DL+LL+SL / 短期地震 DL+LL±EX/EY の正負両加力）。
/// 内部では [`standard_combinations`] に委譲する。暴風（WX/WY）・多雪区域の
/// 係数付き組合せが必要な場合は [`standard_combinations`] を直接使う。
pub fn auto_combinations(
    dl_case: LoadCaseId,
    ll_case: LoadCaseId,
    seismic_x: Option<LoadCaseId>,
    seismic_y: Option<LoadCaseId>,
    snow_case: Option<LoadCaseId>,
) -> Vec<LoadCombination> {
    let input = ComboInput {
        dl: dl_case,
        ll: ll_case,
        seismic_x,
        seismic_y,
        wind_x: None,
        wind_y: None,
        snow: snow_case,
        heavy_snow_zone: false,
        snow_factors: None,
    };
    standard_combinations(&input)
}

/// 荷重組合せ名から断面検定の荷重継続性区分（長期/短期）を判定する。
///
/// 令82条（標準組合せ）・令86条（多雪区域）: DL+LL（多雪区域では
/// DL+LL+0.7SL も）が長期（常時・積雪時の長期）、地震（EX/EY）・風（WX/WY）を含む
/// 組合せおよび短期積雪（DL+LL+SL）は短期（令82条）。
/// [`standard_combinations`] の命名規約（"DL + LL ± EX"・"DL + LL + 0.7SL" 等）に
/// 基づき、追加項の記号で判定する。旧名（"G + P ± Kx" 等）の保存データも、
/// 地震記号 K/E・風記号 W・積雪 S を含むかで同様に判定できる（後方互換）。
pub fn is_short_term_combo(name: &str) -> bool {
    let upper = name.to_uppercase();
    // 地震（記号 E。旧名の K も含む）・風（W）を含めば短期。
    if upper.contains('K') || upper.contains('E') || upper.contains('W') {
        return true;
    }
    // 多雪区域の長期積雪 δ1・S（係数 <1.0 の S 項。例 "0.7SL"・"0.65SL"）は
    // 長期（令82条一号）。係数なしの S（DL+LL+SL）は短期積雪。
    if let Some(pos) = upper.find('S') {
        let coef: String = upper[..pos]
            .chars()
            .rev()
            .take_while(|c| c.is_ascii_digit() || *c == '.')
            .collect::<Vec<_>>()
            .into_iter()
            .rev()
            .collect();
        if let Ok(v) = coef.parse::<f64>() {
            return v >= 1.0;
        }
        return true;
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_auto_combos() {
        let combos = auto_combinations(
            LoadCaseId(1),
            LoadCaseId(2),
            Some(LoadCaseId(3)),
            Some(LoadCaseId(4)),
            None,
        );
        // DL+LL, DL+LL±EX, DL+LL±EY の 5 組合せ
        assert_eq!(combos.len(), 5);
        assert_eq!(combos[0].name, "DL + LL");
        assert_eq!(combos[1].name, "DL + LL + EX");
        assert_eq!(combos[2].name, "DL + LL - EX");
        assert_eq!(combos[3].name, "DL + LL + EY");
        assert_eq!(combos[4].name, "DL + LL - EY");
        // 負側加力は係数 -1.0
        assert_eq!(combos[2].terms[2].1, -1.0);
        assert_eq!(combos[4].terms[2].1, -1.0);
    }

    #[test]
    fn test_is_short_term_combo() {
        assert!(!is_short_term_combo("DL + LL"));
        assert!(is_short_term_combo("DL + LL + EX"));
        assert!(is_short_term_combo("DL + LL - EX"));
        assert!(is_short_term_combo("DL + LL + EY"));
        assert!(is_short_term_combo("DL + LL + SL"));
        assert!(is_short_term_combo("DL + LL + WX"));
        assert!(is_short_term_combo("DL + LL - WY"));
        // 多雪区域: 長期 0.7SL は長期、0.35SL 付き短期は短期。
        assert!(!is_short_term_combo("DL + LL + 0.7SL"));
        assert!(is_short_term_combo("DL + LL + 0.35SL + EX"));
        assert!(is_short_term_combo("DL + LL + 0.35SL - WY"));
        // 旧名（G+P・Kx/Wx 等）の保存データも従来どおり判定できる（後方互換）。
        assert!(!is_short_term_combo("G + P"));
        assert!(is_short_term_combo("G + P + Kx"));
        assert!(is_short_term_combo("G + P + Wx"));
        assert!(!is_short_term_combo("G + P + 0.7S"));
    }

    #[test]
    fn test_auto_combos_no_snow_matches_legacy_shape() {
        // 多雪区域=false・風=None の従来相当構成では、長期1 + 短期積雪0
        // + 地震(±EX,±EY)=4 の計 5 ケース。
        let combos = auto_combinations(
            LoadCaseId(1),
            LoadCaseId(2),
            Some(LoadCaseId(3)),
            Some(LoadCaseId(4)),
            None,
        );
        assert_eq!(combos.len(), 5);
        let names: Vec<&str> = combos.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(
            names,
            vec![
                "DL + LL",
                "DL + LL + EX",
                "DL + LL - EX",
                "DL + LL + EY",
                "DL + LL - EY"
            ]
        );
    }

    #[test]
    fn test_standard_combinations_all_cases_heavy_snow() {
        let input = ComboInput {
            dl: LoadCaseId(1),
            ll: LoadCaseId(2),
            seismic_x: Some(LoadCaseId(3)),
            seismic_y: Some(LoadCaseId(4)),
            wind_x: Some(LoadCaseId(5)),
            wind_y: Some(LoadCaseId(6)),
            snow: Some(LoadCaseId(7)),
            heavy_snow_zone: true,
            snow_factors: None,
        };
        let combos = standard_combinations(&input);
        // DL+LL(1) + DL+LL+0.7SL(1) + DL+LL+SL(1)
        // + EX系4 + EY系4 + WX系4 + WY系4 = 3 + 16 = 19
        assert_eq!(combos.len(), 19);

        let by_name = |n: &str| {
            combos
                .iter()
                .find(|c| c.name == n)
                .unwrap_or_else(|| panic!("missing combo {n}"))
        };

        assert_eq!(
            by_name("DL + LL").terms,
            vec![(LoadCaseId(1), 1.0), (LoadCaseId(2), 1.0)]
        );
        assert_eq!(
            by_name("DL + LL + 0.7SL").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(7), 0.7)
            ]
        );
        assert_eq!(
            by_name("DL + LL + SL").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(7), 1.0)
            ]
        );
        assert_eq!(
            by_name("DL + LL + EX").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(3), 1.0)
            ]
        );
        assert_eq!(
            by_name("DL + LL - EX").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(3), -1.0)
            ]
        );
        assert_eq!(
            by_name("DL + LL + 0.35SL + EX").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(7), 0.35),
                (LoadCaseId(3), 1.0)
            ]
        );
        assert_eq!(
            by_name("DL + LL + 0.35SL - EY").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(7), 0.35),
                (LoadCaseId(4), -1.0)
            ]
        );
        assert_eq!(
            by_name("DL + LL + WX").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(5), 1.0)
            ]
        );
        assert_eq!(
            by_name("DL + LL - WY").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(6), -1.0)
            ]
        );
        assert_eq!(
            by_name("DL + LL + 0.35SL + WY").terms,
            vec![
                (LoadCaseId(1), 1.0),
                (LoadCaseId(2), 1.0),
                (LoadCaseId(7), 0.35),
                (LoadCaseId(6), 1.0)
            ]
        );
    }

    #[test]
    fn test_snow_factors_direct_input() {
        // δ1/δ2/δ3 の直接入力（デフォルト 0.7/0.35/0.35、直接入力可能）。
        // 名前・係数の両方に反映される。
        let input = ComboInput {
            dl: LoadCaseId(1),
            ll: LoadCaseId(2),
            seismic_x: Some(LoadCaseId(3)),
            seismic_y: None,
            wind_x: Some(LoadCaseId(5)),
            wind_y: None,
            snow: Some(LoadCaseId(7)),
            heavy_snow_zone: true,
            snow_factors: Some(SnowFactors {
                delta1: 0.65,
                delta2: 0.3,
                delta3: 0.4,
            }),
        };
        let combos = standard_combinations(&input);
        let by_name = |n: &str| {
            combos
                .iter()
                .find(|c| c.name == n)
                .unwrap_or_else(|| panic!("missing combo {n}"))
        };
        // 長期積雪: δ1=0.65
        assert_eq!(by_name("DL + LL + 0.65SL").terms[2], (LoadCaseId(7), 0.65));
        // 地震時: δ3=0.4、暴風時: δ2=0.3
        assert_eq!(
            by_name("DL + LL + 0.4SL + EX").terms[2],
            (LoadCaseId(7), 0.4)
        );
        assert_eq!(
            by_name("DL + LL + 0.3SL + WX").terms[2],
            (LoadCaseId(7), 0.3)
        );
        // 長短期判定: δ1 付きは長期、δ3 付き地震は短期。
        assert!(!is_short_term_combo("DL + LL + 0.65SL"));
        assert!(is_short_term_combo("DL + LL + 0.4SL + EX"));
        assert!(is_short_term_combo("DL + LL + SL"));
    }

    #[test]
    fn test_standard_combinations_no_heavy_snow_no_wind() {
        let input = ComboInput {
            dl: LoadCaseId(1),
            ll: LoadCaseId(2),
            seismic_x: Some(LoadCaseId(3)),
            seismic_y: Some(LoadCaseId(4)),
            wind_x: None,
            wind_y: None,
            snow: Some(LoadCaseId(5)),
            heavy_snow_zone: false,
            snow_factors: None,
        };
        let combos = standard_combinations(&input);
        // DL+LL(1) + DL+LL+SL(1) + EX系2 + EY系2 = 6（多雪でないので 0.7SL・0.35SL 系は無し）
        assert_eq!(combos.len(), 6);
        assert!(combos.iter().all(|c| !c.name.contains("0.35SL")));
        assert!(combos.iter().all(|c| !c.name.contains("0.7SL")));
        assert!(combos.iter().all(|c| !c.name.contains('W')));
    }

    #[test]
    fn test_default_combinations_matches_auto_combinations() {
        // squid-n-core の default_combinations（新規モデルの既定）は、標準ケースの
        // 並び（0:DL, 1:LL(架構用), 3:EX, 4:EY）に対する auto_combinations と
        // 完全一致する（名前・係数構成とも）。両者の命名規約がずれていないことを保証する。
        let expected = auto_combinations(
            LoadCaseId(0),
            LoadCaseId(1),
            Some(LoadCaseId(3)),
            Some(LoadCaseId(4)),
            None,
        );
        let actual = squid_n_core::model::default_combinations();
        assert_eq!(
            actual, expected,
            "default_combinations が auto_combinations（DL/LL/EX/EY）と一致していない"
        );
        // 表示名で長短期の判別が正しく機能する（DL+LL は長期、地震4件は短期）。
        assert!(!is_short_term_combo(&actual[0].name));
        for c in &actual[1..] {
            assert!(is_short_term_combo(&c.name), "{} は短期のはず", c.name);
        }
    }

    #[test]
    fn test_standard_combinations_empty_optional_cases() {
        let input = ComboInput {
            dl: LoadCaseId(1),
            ll: LoadCaseId(2),
            seismic_x: None,
            seismic_y: None,
            wind_x: None,
            wind_y: None,
            snow: None,
            heavy_snow_zone: false,
            snow_factors: None,
        };
        let combos = standard_combinations(&input);
        assert_eq!(combos.len(), 1);
        assert_eq!(combos[0].name, "DL + LL");
    }
}
