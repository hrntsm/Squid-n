//! 材料グレード名（`SS400`・`SD345`・`Fc21` 等）から規格値を解決する対応表。
//!
//! 鋼材の基準強度 F・鉄筋の基準強度・コンクリート設計基準強度 Fc の
//! 「名称 → 数値」対応は本モジュールに一本化する（H12 建告第2464号、
//! JIS 規格、大臣認定品の基準強度）。設計計算（squid-n-design-jp）・
//! ST-Bridge 取込（squid-n-io）・UI プリセット（squid-n-app）は
//! いずれも本対応表を参照し、独立の表を持たない。
//!
//! - [`steel_f_value`] — 鋼材の基準強度 F（完全一致、板厚区分対応）
//! - [`steel_f_value_prefix`] — 同（前方一致。`SN490B` → `SN490` 等）
//! - [`rebar_f_value`] — 鉄筋の基準強度（`SD345` → 345 等）
//! - [`rebar_grade_f_value`] — 同（高強度鉄筋の製品名を含む。`KH785` → 785 等）
//! - [`rebar_yield_strength`] / [`shear_rebar_yield_strength`] — 配筋の材質からの σy・σwy 解決
//! - [`parse_concrete_fc`] — コンクリート `FcXX` 名称の解釈
//! - [`material_presets`] — UI に提示する標準材料プリセット一覧

use crate::model::MaterialCategory;
use crate::section_shape::{concrete_young_modulus_gamma, E_STEEL};
use crate::units::{
    concrete_unit_weight_kn_m3, to_internal::mass_density_from_unit_weight_kn_m3, ConcreteClass,
    ConcreteComposition, STEEL_UNIT_WEIGHT_KN_M3,
};

/// 板厚 2 区分（`t<=40` / `40<t<=100`）の F 値を返す。
/// 100mm 超は規定がないため最終区分値をそのまま用いる（非保守的になり得る）。
fn bucket2(t: f64, le40: f64, gt40: f64) -> f64 {
    if t <= 40.0 {
        le40
    } else {
        gt40
    }
}

/// 板厚 3 区分（`t<=40` / `40<t<=75` / `75<t<=100`）の F 値を返す（SM520 用）。
fn bucket3(t: f64, le40: f64, le75: f64, gt75: f64) -> f64 {
    if t <= 40.0 {
        le40
    } else if t <= 75.0 {
        le75
    } else {
        gt75
    }
}

/// 鋼材グレード一覧（前方一致の探索対象。`SN490B` のような接尾辞付き名称を
/// 解決するため、[`steel_f_value_prefix`] は最長一致のグレードを選ぶ）。
pub const STEEL_GRADES: &[&str] = &[
    // JIS 規格品
    "SS400", "SS490", "SM400", "SM490", "SM520", "SN400", "SN490", "STK400", "STK490", "STKN400",
    "STKN490", "STKR400", "STKR490", "SNR400", "SNR490", "SSC400", "SWH400",
    // 冷間成形角形鋼管（大臣認定品。BCR235 は旧グレードの互換）
    "BCR235", "BCR295", "BCP235", "BCP325",
    // 建築構造用 TMCP 鋼材（HBL 等の大臣認定品の一般名。板厚 40mm 超でも F 低減なし）
    "TMCP325", "TMCP355", "TMCP385", "TMCP440",
    // 建築構造用高性能 590N/mm² 鋼材・建築構造用低降伏点鋼材
    "SA440", "LY100", "LY225",
];

/// 鋼材の基準強度 F [N/mm²]（完全一致、板厚 [mm] 区分対応。H12 建告第2464号ほか）。
///
/// JIS 規格品は厚さ 40mm 以下 / 40mm 超 100mm 以下の 2 区分
/// （SM520 のみ 40/75/100mm の 3 区分）。大臣認定品（BCR/BCP・TMCP・SA440・LY）は
/// 板厚区分を持たない。100mm を超える板厚は規定がないため最終区分値を
/// そのまま用いる（非保守的になり得るため実運用では要確認）。
///
/// 戻り値は F 値。長期許容引張・圧縮・曲げ `ft = F/1.5`、
/// 長期許容せん断 `fs = F/(1.5·√3)`。短期は長期の 1.5 倍（=F, F/√3）。
pub fn steel_f_value(grade: &str, thickness: f64) -> Option<f64> {
    match grade {
        // 400 N/mm² 級（F=235/215）
        "SS400" | "SM400" | "SN400" | "STK400" | "STKN400" | "STKR400" | "SNR400" | "SSC400"
        | "SWH400" => Some(bucket2(thickness, 235.0, 215.0)),
        // SS490（F=275/255）
        "SS490" => Some(bucket2(thickness, 275.0, 255.0)),
        // 490 N/mm² 級（F=325/295）
        "SM490" | "SN490" | "STK490" | "STKN490" | "STKR490" | "SNR490" => {
            Some(bucket2(thickness, 325.0, 295.0))
        }
        // 520 N/mm² 級（F=355/335/325）
        "SM520" => Some(bucket3(thickness, 355.0, 335.0, 325.0)),
        // 冷間成形角形鋼管（大臣認定品。板厚区分なし）
        "BCR295" => Some(295.0),
        "BCR235" => Some(235.0),
        "BCP235" => Some(235.0),
        "BCP325" => Some(325.0),
        // 建築構造用 TMCP 鋼材（板厚 40mm 超 100mm 以下でも F 低減なし）
        "TMCP325" => Some(325.0),
        "TMCP355" => Some(355.0),
        "TMCP385" => Some(385.0),
        "TMCP440" => Some(440.0),
        // 建築構造用高性能 590N/mm² 鋼材
        "SA440" => Some(440.0),
        // 建築構造用低降伏点鋼材（基準強度 F: 100N 級=80、225N 級=205）
        "LY100" => Some(80.0),
        "LY225" => Some(205.0),
        _ => None,
    }
}

/// 鋼材の基準強度 F [N/mm²]（前方一致、板厚 [mm] 区分対応）。
///
/// `SN490B`・`SM490YA` のような JIS 種別記号付きの名称を、
/// [`STEEL_GRADES`] の最長一致で解決する（`SN490B` は `SN400` ではなく
/// `SN490` に一致）。未知の名称は `None`。
pub fn steel_f_value_prefix(name: &str, thickness: f64) -> Option<f64> {
    STEEL_GRADES
        .iter()
        .filter(|g| name.starts_with(*g))
        .max_by_key(|g| g.len())
        .and_then(|g| steel_f_value(g, thickness))
}

/// 保有水平耐力計算（プッシュオーバー）用の材料強度割増係数。
///
/// 材料強度の基準強度は表の数値の 1.1 倍以下（JIS 規格品・大臣認定品）、
/// ただし 590N 級（SA440・TMCP440。HBL®440/G440 等の認定条件）は 1.05 倍以下と
/// できる規定（H12 建告第2464号の運用・各認定条件）に基づく。
/// 名称から鋼材グレードを解決できない材料（直接入力材料）は割増しない（1.0）。
///
/// 本係数は保有水平耐力計算（プッシュオーバー）の部材耐力算定にのみ用い、
/// 許容応力度計算（一次設計）には適用しない。
pub fn steel_material_strength_factor(name: &str) -> f64 {
    let Some(grade) = STEEL_GRADES
        .iter()
        .filter(|g| name.starts_with(*g))
        .max_by_key(|g| g.len())
    else {
        return 1.0;
    };
    match *grade {
        // 590N 級（建築構造用高性能 590N/mm² 鋼材・TMCP 590N 級）は 1.05 倍
        "SA440" | "TMCP440" => 1.05,
        _ => 1.1,
    }
}

/// 保有水平耐力計算（プッシュオーバー）で、材料の降伏強度 fy を
/// **鋼材**として用いる文脈（鋼材断面の集中ばね・純鋼材ファイバー・
/// 曲げヒンジ・せん断降伏閾値）の材料強度割増係数。
///
/// 直接入力の割増係数（[`crate::model::Material::strength_factor`]）があれば
/// それを優先し、なければ材料名から自動判定する
/// （[`steel_material_strength_factor`]: 鋼材グレード=1.1、590N 級=1.05、
/// 名称から解決できない材料=1.0）。
pub fn material_strength_factor_steel(mat: &crate::model::Material) -> f64 {
    mat.strength_factor
        .unwrap_or_else(|| steel_material_strength_factor(&mat.name))
}

/// 保有水平耐力計算（プッシュオーバー）で、材料の降伏強度 fy を
/// **RC 主筋**として用いる文脈（RC 断面の集中ばね・主筋ファイバー・
/// 曲げヒンジ・せん断降伏の主筋 σy）の材料強度割増係数。
///
/// 直接入力の割増係数（[`crate::model::Material::strength_factor`]）があれば
/// それを優先し、なければ 1.1（鉄筋の材料強度は基準強度の 1.1 倍以下と
/// できる規定）。fy 未設定で既定値（SD345 相当の 345）を用いる場合にも
/// 同係数を乗じる。**せん断補強筋は割増対象外**（本係数を用いないこと）。
pub fn material_strength_factor_rebar(mat: &crate::model::Material) -> f64 {
    mat.strength_factor.unwrap_or(1.1)
}

/// 鉄筋の基準強度 [N/mm²]（H12 建告第2464号）。
///
/// 異形鉄筋 `SD` ・丸鋼 `SR` は続く数値が基準強度を表す
/// （`SD295A`・`SD345`・`SR235` 等）。未知の名称は `None`。
pub fn rebar_f_value(name: &str) -> Option<f64> {
    let rest = name
        .strip_prefix("SD")
        .or_else(|| name.strip_prefix("SR"))?;
    let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
    digits.parse::<f64>().ok().filter(|v| *v > 0.0)
}

/// 鉄筋のグレード名から降伏点（基準強度）[N/mm²] を解決する。
///
/// [`rebar_f_value`]（異形鉄筋 `SD`・丸鋼 `SR` の接頭辞規則）で解けない名称は、
/// 大臣認定品の高強度鉄筋として**名称末尾の数値**を強度とみなす
/// （`USD685` → 685、`KH785`（スーパーフープ）→ 785、`SBPD1275` → 1275 等。
/// いずれも製品名の数値が降伏点 [N/mm²] を表す命名規則）。
/// 数値を含まない名称は `None`。
pub fn rebar_grade_f_value(name: &str) -> Option<f64> {
    let name = name.trim();
    if let Some(v) = rebar_f_value(name) {
        return Some(v);
    }
    let digits: String = name
        .chars()
        .rev()
        .take_while(|c| c.is_ascii_digit())
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
        .collect();
    digits.parse::<f64>().ok().filter(|v| *v > 0.0)
}

/// RC 主筋の降伏点 σy [N/mm²] を解決する。
///
/// 断面（配筋）の主筋材質 [`crate::section_shape::RcRebar::main_grade`] を第一に、
/// なければ部材材料の `fy` を用いる。どちらもない場合は `None` を返し、**既定値で
/// 埋めない**（未入力のまま既定 345 N/mm² を用いると、SD295 の部材で耐力を過大評価
/// する＝危険側になるため。非線形解析は `None` を入力不備として停止する）。
pub fn rebar_yield_strength(
    main_grade: Option<&str>,
    mat: Option<&crate::model::Material>,
) -> Option<f64> {
    if let Some(v) = main_grade.and_then(rebar_grade_f_value) {
        return Some(v);
    }
    mat.and_then(|m| m.fy).filter(|v| *v > 0.0)
}

/// せん断補強筋の降伏点 σwy [N/mm²] を解決する。
///
/// 断面（配筋）のせん断補強筋材質 [`crate::section_shape::ShearBar::grade`] から
/// 解決する。未設定は `None` を返し、呼び出し側は普通強度せん断補強筋の
/// SD295 相当（295）を既定とする（規格上の最小グレードであり、実際がより高強度でも
/// 耐力を過小評価する側＝安全側に外れる）。
pub fn shear_rebar_yield_strength(grade: Option<&str>) -> Option<f64> {
    grade.and_then(rebar_grade_f_value)
}

/// せん断補強筋の材質が未設定の場合に用いる降伏点 σwy [N/mm²]（SD295 相当）。
pub const SHEAR_REBAR_DEFAULT_FY: f64 = 295.0;

/// UI で選択できる主筋のグレード（表示順。JIS G 3112 の異形棒鋼と高強度異形棒鋼）。
pub const MAIN_REBAR_GRADES: &[&str] = &["SD295A", "SD295B", "SD345", "SD390", "SD490", "USD685"];

/// UI で選択できる鉄筋の呼び名サイズ（`D10`〜`D41`。表示順）。
///
/// 数値は**呼び名の数値**であり、`BarSet::dia` / `ShearBar::dia` はこの値で保持する
/// （許容応力度の径依存判定（`dia >= 29.0` ＝ D29 以上）と単位を揃えるため。
/// 公称直径 [mm] ではない）。
pub const REBAR_NOMINAL_SIZES: &[f64] = &[
    10.0, 13.0, 16.0, 19.0, 22.0, 25.0, 29.0, 32.0, 35.0, 38.0, 41.0,
];

/// UI で選択できるせん断補強筋のグレード（表示順）。
/// 普通強度（`SD*`）に加え、大臣認定の高強度せん断補強筋を含む。
pub const SHEAR_REBAR_GRADES: &[&str] = &[
    "SD295A", "SD295B", "SD345", "SD390", "KH785", "UB785", "KSS785", "SHD685", "SPR785", "MK785",
    "SBPD1275",
];

/// コンクリートのグレード名 `FcXX` から設計基準強度 Fc [N/mm²] を取り出す。
/// 大文字小文字を問わず `Fc` で始まり、続く数値を Fc とする（`Fc21`→21）。
pub fn parse_concrete_fc(name: &str) -> Option<f64> {
    let rest = name
        .strip_prefix("Fc")
        .or_else(|| name.strip_prefix("FC"))
        .or_else(|| name.strip_prefix("fc"))?;
    let digits: String = rest
        .trim()
        .chars()
        .take_while(|c| c.is_ascii_digit() || *c == '.')
        .collect();
    digits.parse::<f64>().ok().filter(|v| *v > 0.0)
}

/// グレード名から材料の区分を推定する。
///
/// **ST-Bridge の取込専用**である。ST-Bridge は材料を「グレード名」で表し
/// （`Fc21`・`SN400B`・`SD345`）、名称が物性を一意に定める規格化された体系なので、
/// 名前からの推定が成り立つ。
///
/// モデル内部では区分を [`crate::model::Material::category`] として明示的に持ち、
/// 構造種別の判定に名前は用いない（`crate::structure_kind`）。利用者が任意の名前を
/// 付けた材料でも正しく扱うためである。
///
/// 判定できない名称は `None` を返し、呼び出し側が既定を決める。
pub fn category_of_grade(name: &str) -> Option<MaterialCategory> {
    let upper = name.trim().to_uppercase();
    if upper.starts_with("FC") {
        return Some(MaterialCategory::Concrete);
    }
    if upper.starts_with("SD") || upper.starts_with("SR") || upper.starts_with("KH") {
        return Some(MaterialCategory::Rebar);
    }
    const STEEL_PREFIX: &[&str] = &["SS", "SN", "SM", "STK", "ST", "SA", "BC", "TMCP", "LY"];
    if STEEL_PREFIX.iter().any(|p| upper.starts_with(p)) {
        return Some(MaterialCategory::Steel);
    }
    None
}

/// UI に提示する標準材料プリセット（内部単位系 N-mm-s。密度は ton/mm³）。
#[derive(Clone, Debug, PartialEq)]
pub struct MaterialPreset {
    pub name: &'static str,
    pub category: MaterialCategory,
    /// ヤング係数 E [N/mm²]
    pub young: f64,
    /// ポアソン比 ν
    pub poisson: f64,
    /// 質量密度 [ton/mm³]
    pub density: f64,
    /// コンクリート設計基準強度 Fc [N/mm²]（鋼材・鉄筋は None）
    pub fc: Option<f64>,
    /// 基準強度 F（板厚 40mm 以下）／鉄筋降伏点 [N/mm²]（コンクリートは None）
    pub fy: Option<f64>,
}

/// UI プリセットとして提示する鋼材グレード（表示順）。
const PRESET_STEEL: &[&str] = &[
    "SS400", "SN400", "SM400", "SM490", "SN490", "BCR295", "BCP235", "TMCP325", "TMCP355",
    "TMCP385", "TMCP440", "SA440", "LY100", "LY225",
];

/// UI プリセットとして提示する鉄筋グレード（表示順）。
const PRESET_REBAR: &[&str] = &["SD295", "SD345", "SD390"];

/// UI プリセットとして提示するコンクリート強度（表示順）。
const PRESET_CONCRETE_FC: &[f64] = &[
    18.0, 21.0, 24.0, 27.0, 30.0, 33.0, 36.0, 40.0, 42.0, 45.0, 50.0, 55.0, 60.0,
];

/// プリセットのコンクリート名（`Fc18`〜`Fc60`）。`PRESET_CONCRETE_FC` と同順。
const PRESET_CONCRETE_NAMES: &[&str] = &[
    "Fc18", "Fc21", "Fc24", "Fc27", "Fc30", "Fc33", "Fc36", "Fc40", "Fc42", "Fc45", "Fc50", "Fc55",
    "Fc60",
];

/// 標準材料プリセット一覧を生成する。
///
/// - 鋼材・鉄筋: E=205000、ν=0.3、γs=77 kN/m³（≒7.85 t/m³）。
///   `fy` は基準強度 F（板厚 40mm 以下）。設計計算では名称から
///   [`steel_f_value_prefix`] で板厚区分込みの F を再解決する。
/// - コンクリート: ν=0.2。E は Ec=3.35·10⁴·(γ/24)²·(Fc/60)^(1/3)
///   （γ は Fc 帯に応じた普通コンクリートの気乾単位体積重量）。
///   密度は単位体積重量表の γRC（鉄筋込み）から導出する。
pub fn material_presets() -> Vec<MaterialPreset> {
    let steel_density = mass_density_from_unit_weight_kn_m3(STEEL_UNIT_WEIGHT_KN_M3);
    let mut out = Vec::new();
    for &name in PRESET_STEEL {
        out.push(MaterialPreset {
            name,
            category: MaterialCategory::Steel,
            young: E_STEEL,
            poisson: 0.3,
            density: steel_density,
            fc: None,
            fy: steel_f_value(name, 0.0),
        });
    }
    for &name in PRESET_REBAR {
        out.push(MaterialPreset {
            name,
            category: MaterialCategory::Rebar,
            young: E_STEEL,
            poisson: 0.3,
            density: steel_density,
            fc: None,
            fy: rebar_f_value(name),
        });
    }
    for (&fc, &name) in PRESET_CONCRETE_FC.iter().zip(PRESET_CONCRETE_NAMES) {
        let gamma_c =
            concrete_unit_weight_kn_m3(fc, ConcreteClass::Normal, ConcreteComposition::Plain);
        let gamma_rc =
            concrete_unit_weight_kn_m3(fc, ConcreteClass::Normal, ConcreteComposition::Rc);
        out.push(MaterialPreset {
            name,
            category: MaterialCategory::Concrete,
            young: concrete_young_modulus_gamma(fc, gamma_c),
            poisson: 0.2,
            density: mass_density_from_unit_weight_kn_m3(gamma_rc),
            fc: Some(fc),
            fy: None,
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ST-Bridge の代表的なグレード名から区分を判定できることを確認する。
    #[test]
    fn test_category_of_grade() {
        for (name, expected) in [
            ("Fc21", MaterialCategory::Concrete),
            ("FC60", MaterialCategory::Concrete),
            ("SN400B", MaterialCategory::Steel),
            ("SS400", MaterialCategory::Steel),
            ("SM490A", MaterialCategory::Steel),
            ("STKR400", MaterialCategory::Steel),
            ("BCR295", MaterialCategory::Steel),
            ("TMCP325", MaterialCategory::Steel),
            ("LY225", MaterialCategory::Steel),
            ("SD295A", MaterialCategory::Rebar),
            ("SD345", MaterialCategory::Rebar),
            ("SR235", MaterialCategory::Rebar),
            ("KH785", MaterialCategory::Rebar),
        ] {
            assert_eq!(
                category_of_grade(name),
                Some(expected),
                "グレード名 {name} の区分"
            );
        }
    }

    /// 判定できない名称は `None` を返し、取込側が既定を決める。
    #[test]
    fn test_category_of_grade_unknown() {
        for name in ["コンクリート", "普通強度", "", "   ", "X999"] {
            assert_eq!(category_of_grade(name), None, "未知の名称 {name:?}");
        }
    }

    /// 判定の順序（コンクリート → 鉄筋 → 鋼材）を確認する。
    /// `SD`/`SR` は鋼材の前方一致（`S…`）より先に判定する必要がある。
    #[test]
    fn test_category_of_grade_order() {
        // 鉄筋は鋼材より先に判定する。
        assert_eq!(category_of_grade("SD390"), Some(MaterialCategory::Rebar));
        // 大文字小文字は区別しない。
        assert_eq!(category_of_grade("fc24"), Some(MaterialCategory::Concrete));
        assert_eq!(category_of_grade("sn400b"), Some(MaterialCategory::Steel));
        // 前後の空白は無視する。
        assert_eq!(
            category_of_grade("  SD345  "),
            Some(MaterialCategory::Rebar)
        );
    }

    /// プリセット表の区分は、名前から推定した区分と一致する。
    /// 一致しない項目があるとプリセットで追加した材料と ST-Bridge から
    /// 取り込んだ同名の材料で構造種別が食い違う。
    #[test]
    fn test_presets_agree_with_category_of_grade() {
        for p in material_presets() {
            if let Some(guessed) = category_of_grade(p.name) {
                assert_eq!(guessed, p.category, "プリセット {} の区分", p.name);
            }
        }
    }

    /// H12 建告第2464号の基準強度表と一致することを確認する（板厚 40mm 以下）。
    #[test]
    fn test_steel_f_value_le40() {
        for (g, f) in [
            ("SS400", 235.0),
            ("SN400", 235.0),
            ("SM400", 235.0),
            ("SM490", 325.0),
            ("SN490", 325.0),
            ("SM520", 355.0),
            ("SS490", 275.0),
            ("BCR295", 295.0),
            ("BCP235", 235.0),
            ("BCP325", 325.0),
            ("TMCP325", 325.0),
            ("TMCP355", 355.0),
            ("TMCP385", 385.0),
            ("TMCP440", 440.0),
            ("SA440", 440.0),
            ("LY100", 80.0),
            ("LY225", 205.0),
        ] {
            assert_eq!(steel_f_value(g, 40.0), Some(f), "{g}");
        }
    }

    /// 板厚 40mm 超の低減（JIS 規格品）と、TMCP・SA440・LY・BCR/BCP が
    /// 板厚によらず一定であることを確認する。
    #[test]
    fn test_steel_f_value_gt40() {
        assert_eq!(steel_f_value("SS400", 41.0), Some(215.0));
        assert_eq!(steel_f_value("SM490", 41.0), Some(295.0));
        assert_eq!(steel_f_value("SN490", 41.0), Some(295.0));
        assert_eq!(steel_f_value("SM520", 41.0), Some(335.0));
        assert_eq!(steel_f_value("SM520", 76.0), Some(325.0));
        for g in [
            "TMCP325", "TMCP355", "TMCP385", "TMCP440", "SA440", "BCR295", "LY225",
        ] {
            assert_eq!(steel_f_value(g, 41.0), steel_f_value(g, 40.0), "{g}");
        }
    }

    /// 前方一致解決: JIS 種別記号付き名称・最長一致を確認する。
    #[test]
    fn test_steel_f_value_prefix() {
        assert_eq!(steel_f_value_prefix("SN490B", 40.0), Some(325.0));
        assert_eq!(steel_f_value_prefix("SN400C", 40.0), Some(235.0));
        assert_eq!(steel_f_value_prefix("SM490YA", 40.0), Some(325.0));
        assert_eq!(steel_f_value_prefix("STKN400W", 40.0), Some(235.0));
        assert_eq!(steel_f_value_prefix("STKN490B", 40.0), Some(325.0));
        assert_eq!(steel_f_value_prefix("SNR400A", 40.0), Some(235.0));
        assert_eq!(steel_f_value_prefix("SNR490B", 40.0), Some(325.0));
        assert_eq!(steel_f_value_prefix("未知", 40.0), None);
    }

    /// 材料強度割増係数: 既知鋼材=1.1、590N 級（SA440/TMCP440）=1.05、未知=1.0。
    #[test]
    fn test_steel_material_strength_factor() {
        assert_eq!(steel_material_strength_factor("SS400"), 1.1);
        assert_eq!(steel_material_strength_factor("SN490B"), 1.1);
        assert_eq!(steel_material_strength_factor("BCR295"), 1.1);
        assert_eq!(steel_material_strength_factor("LY225"), 1.1);
        assert_eq!(steel_material_strength_factor("TMCP385"), 1.1);
        assert_eq!(steel_material_strength_factor("SA440"), 1.05);
        assert_eq!(steel_material_strength_factor("TMCP440"), 1.05);
        assert_eq!(steel_material_strength_factor("未知の材料"), 1.0);
        assert_eq!(steel_material_strength_factor("SD345"), 1.0);
    }

    /// 文脈別係数: 直接入力の割増係数が最優先、なければ鋼材=名称判定・主筋=1.1。
    #[test]
    fn test_material_strength_factor_by_context() {
        let mk = |name: &str, factor: Option<f64>| crate::model::Material {
            id: crate::ids::MaterialId(0),
            name: name.to_string(),
            category: MaterialCategory::Steel,
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: None,
            concrete_class: Default::default(),
            strength_factor: factor,
        };
        // 鋼材文脈: 名称から自動判定。
        assert_eq!(material_strength_factor_steel(&mk("SS400", None)), 1.1);
        assert_eq!(material_strength_factor_steel(&mk("SA440", None)), 1.05);
        assert_eq!(material_strength_factor_steel(&mk("カスタム", None)), 1.0);
        // 主筋文脈: 名称によらず 1.1。
        assert_eq!(material_strength_factor_rebar(&mk("Fc24", None)), 1.1);
        assert_eq!(material_strength_factor_rebar(&mk("SD345", None)), 1.1);
        // 直接入力の割増係数は両文脈で最優先。
        assert_eq!(
            material_strength_factor_steel(&mk("カスタム", Some(1.2))),
            1.2
        );
        assert_eq!(material_strength_factor_rebar(&mk("Fc24", Some(1.0))), 1.0);
    }

    #[test]
    fn test_rebar_f_value() {
        assert_eq!(rebar_f_value("SD295A"), Some(295.0));
        assert_eq!(rebar_f_value("SD345"), Some(345.0));
        assert_eq!(rebar_f_value("SD390"), Some(390.0));
        assert_eq!(rebar_f_value("SR235"), Some(235.0));
        assert_eq!(rebar_f_value("SS400"), None);
    }

    /// 高強度鉄筋の製品名は末尾の数値を降伏点とみなす。
    #[test]
    fn test_rebar_grade_f_value_high_strength() {
        assert_eq!(rebar_grade_f_value("SD345"), Some(345.0));
        assert_eq!(rebar_grade_f_value("USD685"), Some(685.0));
        assert_eq!(rebar_grade_f_value("KH785"), Some(785.0));
        assert_eq!(rebar_grade_f_value("UB785"), Some(785.0));
        assert_eq!(rebar_grade_f_value("SBPD1275"), Some(1275.0));
        assert_eq!(rebar_grade_f_value("不明"), None);
    }

    /// 主筋 σy は「断面の主筋材質 → 部材材料の fy」の順で解決し、
    /// どちらもなければ None（既定値で埋めない）。
    #[test]
    fn test_rebar_yield_strength_resolution_order() {
        let mut mat = crate::model::Material {
            strength_factor: None,
            concrete_class: Default::default(),
            id: crate::ids::MaterialId(0),
            name: "Fc24".into(),
            category: MaterialCategory::Concrete,
            young: 23000.0,
            poisson: 0.2,
            density: 0.0,
            shear: None,
            fc: Some(24.0),
            fy: None,
        };
        // 断面の主筋材質が最優先。
        assert_eq!(
            rebar_yield_strength(Some("SD295A"), Some(&mat)),
            Some(295.0)
        );
        // 材質未設定なら材料の fy。
        mat.fy = Some(390.0);
        assert_eq!(rebar_yield_strength(None, Some(&mat)), Some(390.0));
        // 両方なければ None（呼び出し側が入力不備として扱う）。
        mat.fy = None;
        assert_eq!(rebar_yield_strength(None, Some(&mat)), None);
    }

    /// せん断補強筋 σwy は材質から解決し、未設定は None（既定 295 は呼び出し側）。
    #[test]
    fn test_shear_rebar_yield_strength() {
        assert_eq!(shear_rebar_yield_strength(Some("SD295A")), Some(295.0));
        assert_eq!(shear_rebar_yield_strength(Some("KH785")), Some(785.0));
        assert_eq!(shear_rebar_yield_strength(None), None);
        assert_eq!(SHEAR_REBAR_DEFAULT_FY, 295.0);
    }

    #[test]
    fn test_parse_concrete_fc() {
        assert_eq!(parse_concrete_fc("Fc21"), Some(21.0));
        assert_eq!(parse_concrete_fc("FC36"), Some(36.0));
        assert_eq!(parse_concrete_fc("fc60"), Some(60.0));
        assert_eq!(parse_concrete_fc("SD345"), None);
    }

    /// プリセット一覧: 件数・代表値・コンクリートの γ 帯依存を確認する。
    #[test]
    fn test_material_presets() {
        let presets = material_presets();
        assert_eq!(presets.len(), 14 + 3 + 13);
        let find = |name: &str| {
            presets
                .iter()
                .find(|p| p.name == name)
                .unwrap_or_else(|| panic!("preset {name} not found"))
        };

        let ss400 = find("SS400");
        assert_eq!(ss400.category, MaterialCategory::Steel);
        assert_eq!(ss400.young, 205000.0);
        assert_eq!(ss400.fy, Some(235.0));

        let ly100 = find("LY100");
        assert_eq!(ly100.fy, Some(80.0));

        let sd345 = find("SD345");
        assert_eq!(sd345.category, MaterialCategory::Rebar);
        assert_eq!(sd345.fy, Some(345.0));

        // コンクリート: Fc≤36 は γ=23、36<Fc≤48 は γ=23.5、48<Fc は γ=24 で Ec を算定。
        let fc24 = find("Fc24");
        assert_eq!(fc24.category, MaterialCategory::Concrete);
        assert_eq!(fc24.fc, Some(24.0));
        let ec24 = 3.35e4 * (23.0f64 / 24.0).powi(2) * (24.0f64 / 60.0).powf(1.0 / 3.0);
        assert!((fc24.young - ec24).abs() < 1e-9);
        let fc42 = find("Fc42");
        let ec42 = 3.35e4 * (23.5f64 / 24.0).powi(2) * (42.0f64 / 60.0).powf(1.0 / 3.0);
        assert!((fc42.young - ec42).abs() < 1e-9);
        let fc60 = find("Fc60");
        let ec60 = 3.35e4 * (60.0f64 / 60.0).powf(1.0 / 3.0);
        assert!((fc60.young - ec60).abs() < 1e-9);

        // 密度: 鋼は γs=77 kN/m³、コンクリートは γRC（Fc≤36 で 24.0 kN/m³）。
        let rho_steel = mass_density_from_unit_weight_kn_m3(77.0);
        assert!((ss400.density - rho_steel).abs() < 1e-18);
        let rho_rc = mass_density_from_unit_weight_kn_m3(24.0);
        assert!((fc24.density - rho_rc).abs() < 1e-18);
    }
}
