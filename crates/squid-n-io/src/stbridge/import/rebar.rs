//! ST-Bridge の配筋（`StbSecBarArrangement*`）属性の解析（best-effort）。

use super::xml::Attrs;
use squid_n_core::section_shape::{
    BeamStirrup, CircleColumnHoop, RcBeamRebar, RcCircleColumnRebar, RcRectColumnRebar,
    RectColumnHoop,
};

/// 配筋属性に現れる材質のグレード名（主筋・せん断補強筋）。
///
/// 材質は断面の材料として持つため（`Section::rebar_material` ほか）、形状には
/// 入れずここで分けて返す。グレード名から材料を起こすのは取り込みの境界の役目。
#[derive(Default, Clone, Debug)]
pub(super) struct RebarGrades {
    pub main: Option<String>,
    pub shear: Option<String>,
}

/// 幾何のみの RC 梁断面に用いる既定配筋（無筋相当）。
pub(super) fn default_beam_rebar() -> RcBeamRebar {
    RcBeamRebar {
        main_dia: 0.0,
        top: Vec::new(),
        bottom: Vec::new(),
        cover: 0.0,
        stirrup: BeamStirrup {
            dia: 0.0,
            pitch: 0.0,
            legs: 0,
        },
    }
}

/// 幾何のみの RC 矩形柱断面に用いる既定配筋（無筋相当）。
pub(super) fn default_rect_column_rebar() -> RcRectColumnRebar {
    RcRectColumnRebar {
        main_dia: 0.0,
        x: Vec::new(),
        y: Vec::new(),
        cover: 0.0,
        hoop: RectColumnHoop {
            dia: 0.0,
            pitch: 0.0,
            legs_x: 0,
            legs_y: 0,
        },
    }
}

/// 幾何のみの RC 円形柱断面に用いる既定配筋（無筋相当）。
pub(super) fn default_circle_column_rebar() -> RcCircleColumnRebar {
    RcCircleColumnRebar {
        main_dia: 0.0,
        count: 0,
        cover: 0.0,
        hoop: CircleColumnHoop {
            dia: 0.0,
            pitch: 0.0,
        },
    }
}

/// 鉄筋径の文字列を mm へ解釈する。数値ならそのまま、`D22`/`D10` のような呼び名は
/// 先頭の `D`/`d` を除いた数値を径とする（best-effort。厳密な JIS 公称径ではない）。
fn parse_bar_dia(v: &str) -> Option<f64> {
    if let Ok(x) = v.parse::<f64>() {
        return Some(x);
    }
    let t = v.trim();
    if let Some(rest) = t.strip_prefix(['D', 'd']) {
        return rest.trim().parse::<f64>().ok();
    }
    None
}

/// 候補キーのいずれかから f64 を取る（欠落は 0）。
fn attr_f64(a: &Attrs, keys: &[&str]) -> f64 {
    for k in keys {
        if let Some(v) = a.get(k) {
            if let Ok(x) = v.parse::<f64>() {
                return x;
            }
        }
    }
    0.0
}

/// 候補キーのいずれかから鉄筋径 [mm] を取る（呼び名 `D22` も解釈。欠落は 0）。
fn attr_dia(a: &Attrs, keys: &[&str]) -> f64 {
    for k in keys {
        if let Some(v) = a.get(k) {
            if let Some(x) = parse_bar_dia(v) {
                return x;
            }
        }
    }
    0.0
}

/// 候補キーのいずれかから u32 を取る（欠落は 0）。
fn attr_u32(a: &Attrs, keys: &[&str]) -> u32 {
    for k in keys {
        if let Some(v) = a.get(k) {
            if let Ok(x) = v.parse::<u32>() {
                return x;
            }
        }
    }
    0
}

/// 属性名 `{prefix}{n}st/nd/rd/th` の段番号 `n` を取り出す（1 始まり）。
fn ordinal_stage(name: &str, prefix: &str) -> Option<u32> {
    let rest = name.strip_prefix(prefix)?;
    let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
    if digits.is_empty() {
        return None;
    }
    let suffix = &rest[digits.len()..];
    matches!(suffix, "st" | "nd" | "rd" | "th")
        .then(|| digits.parse::<u32>().ok())
        .flatten()
}

/// `{prefix}{n}st/nd/rd/th` 属性の本数を段番号順に集める（4 段目以降も含む）。
fn stage_counts(a: &Attrs, prefixes: &[&str]) -> Vec<u32> {
    let mut found: Vec<(u32, u32)> = Vec::new();
    for name in a.names() {
        for p in prefixes {
            if let Some(n) = ordinal_stage(name, p) {
                if let Some(x) = a.get(name).and_then(|v| v.parse::<u32>().ok()) {
                    found.push((n, x));
                }
            }
        }
    }
    found.sort_by_key(|(n, _)| *n);
    found.into_iter().map(|(_, x)| x).collect()
}

/// 段別属性のない合算本数を 1 段として扱う（見つからなければ空）。
fn total_as_stage(a: &Attrs, keys: &[&str]) -> Vec<u32> {
    for k in keys {
        if let Some(x) = a.get(k).and_then(|v| v.parse::<u32>().ok()) {
            if x > 0 {
                return vec![x];
            }
        }
    }
    Vec::new()
}

/// 配筋要素に現れる主筋径の異なる値を重複なく集める。
fn distinct_main_dias(a: &Attrs) -> Vec<f64> {
    let mut vals: Vec<f64> = Vec::new();
    for name in a.names() {
        let lower = name.to_ascii_lowercase();
        let is_main_dia =
            lower == "d_main" || lower.starts_with("d_main_") || lower.starts_with("dia_main");
        if !is_main_dia {
            continue;
        }
        if let Some(x) = a.get(name).and_then(|v| parse_bar_dia(v)) {
            if x > 0.0 && !vals.iter().any(|y| (y - x).abs() < 1e-9) {
                vals.push(x);
            }
        }
    }
    vals
}

/// 主筋の段間隔を指定する独自属性名（内部規約と異なるもの）を 1 つ返す。
fn stage_spacing_attr(a: &Attrs) -> Option<String> {
    a.names()
        .into_iter()
        .find(|n| {
            let l = n.to_ascii_lowercase();
            l.contains("main")
                && (l.contains("pitch") || l.contains("space") || l.contains("interval"))
                && !l.contains("stirrup")
                && !l.contains("band")
        })
        .map(str::to_string)
}

/// 段数・異径・段間隔の表現限界を警告文へ変換する。
fn push_limit_warnings(a: &Attrs, stage_count: usize, warnings: &mut Vec<String>) {
    if stage_count > 3 {
        warnings.push(format!(
            "RC 配筋の主筋が {stage_count} 段あります。ST-Bridge 標準は 1〜3 段のため 4 段目以降は独自属性です（本数は保持します）"
        ));
    }
    let dias = distinct_main_dias(a);
    if dias.len() > 1 {
        let list = dias
            .iter()
            .map(|d| d.to_string())
            .collect::<Vec<_>>()
            .join(" / ");
        warnings.push(format!(
            "RC 配筋の主筋径が複数あります（{list} mm）。内部モデルは単一径のため D_main の値を採用しました"
        ));
    }
    if let Some(name) = stage_spacing_attr(a) {
        warnings.push(format!(
            "RC 配筋に段間隔属性 {name} があります。内部モデルは段間中心距離 max(25mm, 1.5×主筋径) を規約とするため、指定値は用いません"
        ));
    }
}

/// 配筋属性から材質グレード名を取る。
fn parse_grades(a: &Attrs) -> RebarGrades {
    RebarGrades {
        main: a
            .get("strength_main")
            .or_else(|| a.get("strength_main_X"))
            .or_else(|| a.get("strength_main_top"))
            .or_else(|| a.get("strength_bar_main"))
            .cloned(),
        shear: a
            .get("strength_band")
            .or_else(|| a.get("strength_stirrup"))
            .or_else(|| a.get("strength_bar_band"))
            .or_else(|| a.get("strength_main_band"))
            .cloned(),
    }
}

/// `StbSecBarBeam_RC_*` から梁の実配筋を復元する。上端筋・下端筋を段別本数の列
/// （かぶり側から内側へ）として読む。段数が 3 を超える・主筋径が混在する・独自の
/// 段間隔属性がある場合は警告を返す（合算や代表径への丸めはしない）。
pub(super) fn parse_beam_rebar(a: &Attrs) -> (RcBeamRebar, RebarGrades, Vec<String>) {
    let mut warnings = Vec::new();
    let mut top = stage_counts(a, &["N_main_top_", "N_main_X_"]);
    if top.is_empty() {
        top = total_as_stage(a, &["count_main_top", "count_main_X", "N_main_top"]);
    }
    let mut bottom = stage_counts(a, &["N_main_bottom_", "N_main_Y_"]);
    if bottom.is_empty() {
        bottom = total_as_stage(a, &["count_main_bottom", "count_main_Y", "N_main_bottom"]);
    }
    let stage_count = top.len().max(bottom.len());
    push_limit_warnings(a, stage_count, &mut warnings);
    let rebar = RcBeamRebar {
        main_dia: attr_dia(
            a,
            &[
                "D_main",
                "dia_main",
                "dia_main_X",
                "dia_main_top",
                "D_main_top",
            ],
        ),
        top,
        bottom,
        cover: attr_f64(
            a,
            &["cover", "kaburi", "depth_cover_top", "depth_cover_bottom"],
        ),
        stirrup: BeamStirrup {
            dia: attr_dia(a, &["D_stirrup", "dia_stirrup", "D_band", "dia_band"]),
            pitch: attr_f64(a, &["pitch_stirrup", "pitch_band"]),
            legs: attr_u32(
                a,
                &[
                    "N_stirrup",
                    "count_stirrup",
                    "N_band_direction_X",
                    "count_band",
                ],
            ),
        },
    };
    (rebar, parse_grades(a), warnings)
}

/// `StbSecBarColumn_RC_Rect*` から矩形柱の実配筋を復元する。
///
/// X 方向・Y 方向を段別本数の列として読む。帯筋は X/Y の脚数を別々に保持する。
/// 段数が 3 を超える・主筋径が混在する・独自の段間隔属性がある場合は警告を返す。
pub(super) fn parse_rect_column_rebar(a: &Attrs) -> (RcRectColumnRebar, RebarGrades, Vec<String>) {
    let mut warnings = Vec::new();
    let mut x = stage_counts(a, &["N_main_X_"]);
    if x.is_empty() {
        x = total_as_stage(a, &["count_main_X", "N_main_X"]);
    }
    let mut y = stage_counts(a, &["N_main_Y_"]);
    if y.is_empty() {
        y = total_as_stage(a, &["count_main_Y", "N_main_Y"]);
    }
    let stage_count = x.len().max(y.len());
    push_limit_warnings(a, stage_count, &mut warnings);
    let legs_x = attr_u32(a, &["N_band_direction_X", "count_band_X", "count_band"]);
    let legs_y = {
        let v = attr_u32(a, &["N_band_direction_Y", "count_band_Y"]);
        if v > 0 {
            v
        } else {
            legs_x
        }
    };
    let rebar = RcRectColumnRebar {
        main_dia: attr_dia(
            a,
            &[
                "D_main",
                "dia_main",
                "dia_main_X",
                "D_main_X",
                "dia_main_Y",
                "D_main_Y",
            ],
        ),
        x,
        y,
        cover: attr_f64(
            a,
            &[
                "cover",
                "kaburi",
                "depth_cover_start_X",
                "depth_cover_end_X",
                "depth_cover_start_Y",
                "depth_cover_end_Y",
            ],
        ),
        hoop: RectColumnHoop {
            dia: attr_dia(a, &["D_band", "dia_band", "D_hoop", "dia_hoop"]),
            pitch: attr_f64(a, &["pitch_band", "pitch_hoop"]),
            legs_x,
            legs_y,
        },
    };
    (rebar, parse_grades(a), warnings)
}

/// `StbSecBarColumn_RC_Circle*` から円形柱の実配筋を復元する。主筋は全本数
/// （円周上へ等配）として `N_main` を優先し、`N_main_X_*` も受ける。
/// 段数が 3 を超える・主筋径が混在する・独自の段間隔属性がある場合は警告を返す。
pub(super) fn parse_circle_column_rebar(
    a: &Attrs,
) -> (RcCircleColumnRebar, RebarGrades, Vec<String>) {
    let mut warnings = Vec::new();
    let x = stage_counts(a, &["N_main_X_"]);
    let y = stage_counts(a, &["N_main_Y_"]);
    push_limit_warnings(a, x.len().max(y.len()), &mut warnings);
    let rebar = RcCircleColumnRebar {
        main_dia: attr_dia(a, &["D_main", "dia_main", "dia_main_X", "D_main_X"]),
        count: circle_main_count(a),
        cover: attr_f64(
            a,
            &[
                "cover",
                "kaburi",
                "depth_cover_start_X",
                "depth_cover_end_X",
                "depth_cover_start_Y",
                "depth_cover_end_Y",
            ],
        ),
        hoop: CircleColumnHoop {
            dia: attr_dia(a, &["D_band", "dia_band", "D_hoop", "dia_hoop"]),
            pitch: attr_f64(a, &["pitch_band", "pitch_hoop"]),
        },
    };
    (rebar, parse_grades(a), warnings)
}

/// 円形柱の主筋全本数。`N_main` を優先し、なければ `N_main_X_*` を合算する。
fn circle_main_count(a: &Attrs) -> u32 {
    if let Some(x) = a.get("N_main").and_then(|v| v.parse::<u32>().ok()) {
        return x;
    }
    let x: u32 = stage_counts(a, &["N_main_X_"]).iter().sum();
    if x > 0 {
        return x;
    }
    let y: u32 = stage_counts(a, &["N_main_Y_"]).iter().sum();
    if y > 0 {
        return y;
    }
    attr_u32(a, &["count_main_X", "N_main_X"])
}
