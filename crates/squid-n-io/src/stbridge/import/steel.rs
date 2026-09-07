//! 形鋼ライブラリ要素（`StbSecRoll-*` / `StbSecBuild-*` / `StbSecPipe`）からの断面形状復元。

use super::xml::{get_f64_any, Attrs};
use squid_n_core::section_shape::SectionShape;

/// 形鋼ライブラリ要素と属性から [`SectionShape`] を復元する。
pub(super) fn steel_shape_from(tag: &str, a: &Attrs) -> Option<SectionShape> {
    let a_ = |keys: &[&str]| get_f64_any(a, keys).ok();
    match tag {
        t if t.ends_with("-H") => {
            let height = a_(&["A"])?;
            let web_thick = a_(&["t1"])?;
            let upper_width = a_(&["B"])?;
            let upper_thick = a_(&["t2"])?;
            match (a_(&["B2", "B_lower"]), a_(&["t2_lower", "t2_2"])) {
                (Some(lower_width), Some(lower_thick)) => Some(SectionShape::SteelBuiltH {
                    height,
                    upper_width,
                    upper_thick,
                    lower_width,
                    lower_thick,
                    web_thick,
                }),
                _ => Some(SectionShape::SteelH {
                    height,
                    width: upper_width,
                    web_thick,
                    flange_thick: upper_thick,
                }),
            }
        }
        t if t.ends_with("-BOX") => {
            let thick = a_(&["t", "t1"])?;
            let corner_r = a_(&["r"]).unwrap_or(0.0);
            Some(SectionShape::SteelBox {
                height: a_(&["A"])?,
                width: a_(&["B"])?,
                thick,
                corner_r,
            })
        }
        t if t == "StbSecPipe" || t.ends_with("-Pipe") => Some(SectionShape::SteelPipe {
            outer_dia: a_(&["D", "A"])?,
            thick: a_(&["t", "t1"])?,
        }),
        t if t.ends_with("-L") => Some(SectionShape::SteelAngle {
            leg_a: a_(&["A"])?,
            leg_b: a_(&["B"])?,
            thick: a_(&["t1", "t"])?,
        }),
        t if t.ends_with("-C") => Some(SectionShape::SteelChannel {
            height: a_(&["A"])?,
            width: a_(&["B"])?,
            web_thick: a_(&["t1"])?,
            flange_thick: a_(&["t2"])?,
        }),
        t if t.ends_with("-T") => Some(SectionShape::SteelTee {
            height: a_(&["A"])?,
            width: a_(&["B"])?,
            web_thick: a_(&["t1"])?,
            flange_thick: a_(&["t2"])?,
        }),
        t if t.ends_with("-FlatBar") => Some(SectionShape::SteelFlatBar {
            width: a_(&["B", "A", "width"])?,
            thick: a_(&["t", "t1"])?,
        }),
        t if t.ends_with("-RoundBar") => {
            let dia = a_(&["D", "A"]).or_else(|| a_(&["R"]).map(|r| r * 2.0))?;
            Some(SectionShape::SteelRoundBar { dia })
        }
        t if t.ends_with("-LipC") => Some(SectionShape::SteelLipChannel {
            height: a_(&["A", "H"])?,
            width: a_(&["B"])?,
            lip: a_(&["C"])?,
            thick: a_(&["t", "t1"])?,
        }),
        _ => None,
    }
}
