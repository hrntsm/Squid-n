//! 断面形状の寸法表記（[`SectionShape::dimension_label`]）。
//!
//! 断面リスト上で形状と各寸法を 1 つの文字列で示すための表記で、記号は
//! ASCII のみを用いる。表記の一覧と各記号の意味は
//! `docs/model_io/断面形状の表記.md` を参照。

use super::types::SectionShape;

/// 寸法 1 つの表記。整数値は小数点以下を落とし、端数のある値はそのまま出す
/// （`300.0` → `300`、`6.5` → `6.5`、`216.3` → `216.3`）。
fn dim(v: f64) -> String {
    if v.is_finite() && v.fract() == 0.0 {
        format!("{}", v as i64)
    } else {
        // 冷間成形材の板厚 3.2 のように小数第 1 位までで足りるが、
        // 丸めで別寸法が同じ表記になるのを避けるため 3 桁まで許し末尾 0 を落とす。
        let s = format!("{v:.3}");
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    }
}

/// 寸法列を `x` で連結する（`H-` などの接頭辞は呼び出し側が付ける）。
fn dims(vs: &[f64]) -> String {
    vs.iter().map(|v| dim(*v)).collect::<Vec<_>>().join("x")
}

impl SectionShape {
    /// 形状と各寸法を表す表記（例 `H-500x250x9x16`・`BD-300x600`）。
    ///
    /// 記号は ASCII のみで、断面の種別が接頭辞から一意に読み取れるようにしている。
    /// 円形は用途で記号を分ける（鋼管 `P-`、中実丸鋼 `RB-`、RC 円形 `RD-`）。
    /// SRC は内蔵鉄骨の寸法まで含める（外形が同じで内蔵鉄骨だけが違う断面を
    /// 表記で見分けられるようにするため）。
    pub fn dimension_label(&self) -> String {
        match self {
            SectionShape::SteelH {
                height,
                width,
                web_thick,
                flange_thick,
            } => format!("H-{}", dims(&[*height, *width, *web_thick, *flange_thick])),
            SectionShape::SteelBuiltH {
                height,
                upper_width,
                upper_thick,
                lower_width,
                lower_thick,
                web_thick,
            } => format!(
                "BH-{}",
                dims(&[
                    *height,
                    *upper_width,
                    *upper_thick,
                    *lower_width,
                    *lower_thick,
                    *web_thick,
                ])
            ),
            // 角部外半径 r は表記に含めない（断面性能ではなくせん断有効断面積の
            // 補正にのみ用いる寸法で、表記に入れると列が長くなるため）。
            SectionShape::SteelBox {
                height,
                width,
                thick,
                ..
            } => format!("BOX-{}", dims(&[*height, *width, *thick])),
            SectionShape::SteelPipe { outer_dia, thick } => {
                format!("P-{}", dims(&[*outer_dia, *thick]))
            }
            SectionShape::SteelAngle {
                leg_a,
                leg_b,
                thick,
            } => format!("L-{}", dims(&[*leg_a, *leg_b, *thick])),
            SectionShape::SteelChannel {
                height,
                width,
                web_thick,
                flange_thick,
            } => format!("CH-{}", dims(&[*height, *width, *web_thick, *flange_thick])),
            SectionShape::SteelLipChannel {
                height,
                width,
                lip,
                thick,
            } => format!("LC-{}", dims(&[*height, *width, *lip, *thick])),
            SectionShape::SteelTee {
                height,
                width,
                web_thick,
                flange_thick,
            } => format!("T-{}", dims(&[*height, *width, *web_thick, *flange_thick])),
            SectionShape::SteelFlatBar { width, thick } => {
                format!("FB-{}", dims(&[*width, *thick]))
            }
            SectionShape::SteelRoundBar { dia } => format!("RB-{}", dim(*dia)),
            SectionShape::RcRect { b, d, .. } => format!("BD-{}", dims(&[*b, *d])),
            SectionShape::RcCircle { d, .. } => format!("RD-{}", dim(*d)),
            SectionShape::SrcRect {
                b,
                d,
                steel_height,
                steel_width,
                steel_web_thick,
                steel_flange_thick,
                ..
            } => format!(
                "SRC-{}+H-{}",
                dims(&[*b, *d]),
                dims(&[
                    *steel_height,
                    *steel_width,
                    *steel_web_thick,
                    *steel_flange_thick,
                ])
            ),
            SectionShape::CftBox {
                height,
                width,
                thick,
            } => format!("CFT-BOX-{}", dims(&[*height, *width, *thick])),
            SectionShape::CftPipe { outer_dia, thick } => {
                format!("CFT-P-{}", dims(&[*outer_dia, *thick]))
            }
            SectionShape::RcWall { thickness, .. } => format!("W-t{}", dim(*thickness)),
        }
    }
}
