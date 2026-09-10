//! 鋼断面の代表最大幅厚比の算定（形状寸法からの簡易法）。

use squid_n_core::section_shape::SectionShape;

/// 鋼断面の代表最大幅厚比を形状寸法から算定する。
///
/// 形状ごとに板要素の幅厚比を求め、最大値を返す（簡易法）。
/// 円形鋼管・RC 断面は対象外（`None`）。
/// 板厚が 0 以下、または板要素の内法寸法が 0 未満になる不正な寸法の場合は `None` を返す。
pub fn max_width_thickness(shape: &SectionShape) -> Option<f64> {
    /// 板厚が正で内法寸法が非負なら比を返す。不正な寸法は None。
    fn ratio(clear: f64, thick: f64) -> Option<f64> {
        if thick <= 0.0 || clear < 0.0 {
            None
        } else {
            Some(clear / thick)
        }
    }

    match *shape {
        SectionShape::SteelH {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let flange = ratio(width, 2.0 * flange_thick)?;
            let web = ratio(height - 2.0 * flange_thick, web_thick)?;
            Some(flange.max(web))
        }
        SectionShape::SteelBox {
            height,
            width,
            thick,
            ..
        } => {
            let hi = ratio(height - 2.0 * thick, thick)?;
            let wi = ratio(width - 2.0 * thick, thick)?;
            Some(hi.max(wi))
        }
        SectionShape::SteelBuiltH {
            height,
            upper_width,
            upper_thick,
            lower_width,
            lower_thick,
            web_thick,
        } => {
            let uf = ratio(upper_width, 2.0 * upper_thick)?;
            let lf = ratio(lower_width, 2.0 * lower_thick)?;
            let web = ratio(height - upper_thick - lower_thick, web_thick)?;
            Some(uf.max(lf).max(web))
        }
        SectionShape::SteelChannel {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let flange = ratio(width, flange_thick)?;
            let web = ratio(height - 2.0 * flange_thick, web_thick)?;
            Some(flange.max(web))
        }
        SectionShape::SteelTee {
            height,
            width,
            web_thick,
            flange_thick,
        } => {
            let flange = ratio(width, flange_thick)?;
            let web = ratio(height - flange_thick, web_thick)?;
            Some(flange.max(web))
        }
        SectionShape::SteelAngle {
            leg_a,
            leg_b,
            thick,
        } => ratio(leg_a.max(leg_b), thick),
        SectionShape::SteelPipe { .. } => None,
        SectionShape::CftBox {
            height,
            width,
            thick,
        } => {
            let hi = ratio(height - 2.0 * thick, thick)?;
            let wi = ratio(width - 2.0 * thick, thick)?;
            Some(hi.max(wi))
        }
        SectionShape::CftPipe { .. } => None,
        SectionShape::SteelFlatBar { .. }
        | SectionShape::SteelRoundBar { .. }
        | SectionShape::SteelLipChannel { .. }
        | SectionShape::RcRect { .. }
        | SectionShape::RcCircle { .. }
        | SectionShape::SrcRect { .. }
        | SectionShape::RcWall { .. }
        | SectionShape::RcSlab { .. } => None,
    }
}
